#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Adafruit_INA219.h>

// ====== CONFIG ======
// If your board uses different I2C pins, uncomment and set:
// #define I2C_SDA 8
// #define I2C_SCL 9

// BLE UUIDs (do not change unless you also change the receiver)
static const char* SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
static const char* V_UUID       = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
static const char* I_UUID       = "beb5483e-36e1-4688-b7f5-ea07361b26a9";
static const char* P_UUID       = "beb5483e-36e1-4688-b7f5-ea07361b26aa";
static const char* PAVG_UUID    = "beb5483e-36e1-4688-b7f5-ea07361b26ab";

// ====== GLOBALS ======
Adafruit_INA219 ina219; // default I2C addr 0x40

NimBLEServer* g_server = nullptr;
NimBLECharacteristic *g_vChar=nullptr, *g_iChar=nullptr, *g_pChar=nullptr, *g_pavgChar=nullptr;

// 30 s rolling average buffer for power (float W)
static const uint32_t WINDOW_MS = 30000;
static const int      MAX_SAMPLES = 512; // supports ~17 Hz over 30 s
static float          pBuf[MAX_SAMPLES];
static uint32_t       tBuf[MAX_SAMPLES];
static int            nSamples = 0;

// ====== HELPERS ======
static inline void notifyFloatLE(NimBLECharacteristic* ch, float f) {
  uint32_t u; memcpy(&u, &f, 4);                  // reinterpret float bits
  uint8_t b[4] = { (uint8_t)u, (uint8_t)(u>>8), (uint8_t)(u>>16), (uint8_t)(u>>24) };
  ch->setValue(b, 4);
  ch->notify();
}

static inline void addPowerSample(float p) {
  const uint32_t now = millis();
  if (nSamples < MAX_SAMPLES) {
    pBuf[nSamples] = p; tBuf[nSamples] = now; nSamples++;
  } else {
    memmove(pBuf, pBuf+1, sizeof(float)*(MAX_SAMPLES-1));
    memmove(tBuf, tBuf+1, sizeof(uint32_t)*(MAX_SAMPLES-1));
    pBuf[MAX_SAMPLES-1] = p; tBuf[MAX_SAMPLES-1] = now;
  }
  int start = 0;
  while (start < nSamples && (now - tBuf[start]) > WINDOW_MS) start++;
  if (start > 0) {
    memmove(pBuf, pBuf+start, sizeof(float)*(nSamples-start));
    memmove(tBuf, tBuf+start, sizeof(uint32_t)*(nSamples-start));
    nSamples -= start;
  }
}

static inline float avgPower30s() {
  if (!nSamples) return 0.0f;
  double s = 0.0;
  for (int i=0;i<nSamples;i++) s += pBuf[i];
  return (float)(s / nSamples);
}

// ====== BLE CALLBACKS ======
class ServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*) { Serial.println("Central connected"); }
  void onConnect(NimBLEServer*, ble_gap_conn_desc* d) { Serial.printf("Handle=%d\n", d->conn_handle); }
  void onDisconnect(NimBLEServer*) { Serial.println("Central disconnected; advertising"); NimBLEDevice::startAdvertising(); }
  void onMTUChange(uint16_t MTU, ble_gap_conn_desc*) { Serial.printf("MTU=%u\n", MTU); }
};

static void startAdvertising() {
  auto* adv = NimBLEDevice::getAdvertising();

  NimBLEAdvertisementData advData, scanData;
  advData.setName("ESP32S3-TX");
  advData.setCompleteServices(NimBLEUUID(SERVICE_UUID));
  scanData.setName("ESP32S3-TX");

  adv->setAdvertisementData(advData);
  adv->setScanResponseData(scanData);
  adv->addServiceUUID(SERVICE_UUID);
  adv->start();
  Serial.println("Advertising started");
}

// ====== SETUP ======
void setup() {
  delay(2000);
  Serial.begin(115200);
  delay(2000);

  // I2C
  #if defined(I2C_SDA) && defined(I2C_SCL)
    Wire.begin(I2C_SDA, I2C_SCL);
  #else
    Wire.begin(); // use board defaults
  #endif

  // INA219 init
  if (!ina219.begin()) {
    Serial.println("INA219 not found. Check SDA/SCL/3V3/GND and address (0x40).");
    while (1) delay(10);
  }
  // Calibration:
  // - Works well with Adafruit breakout’s 0.1Ω shunt, ≤~2 A nominal.
  // - If you use a different shunt or current range, change calibration accordingly.
  ina219.setCalibration_32V_2A();
  Serial.println("INA219 ready");

  // BLE init
  NimBLEDevice::init("ESP32S3-TX");
  NimBLEDevice::setDeviceName("ESP32S3-TX");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9); // high TX power

  g_server = NimBLEDevice::createServer();
  g_server->setCallbacks(new ServerCB());

  auto* svc = g_server->createService(SERVICE_UUID);

  g_vChar    = svc->createCharacteristic(V_UUID,    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  g_iChar    = svc->createCharacteristic(I_UUID,    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  g_pChar    = svc->createCharacteristic(P_UUID,    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  g_pavgChar = svc->createCharacteristic(PAVG_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  // seed zeros
  notifyFloatLE(g_vChar, 0.0f);
  notifyFloatLE(g_iChar, 0.0f);
  notifyFloatLE(g_pChar, 0.0f);
  notifyFloatLE(g_pavgChar, 0.0f);

  svc->start();
  startAdvertising();
}

// ====== LOOP ======
void loop() {
  // ~5 Hz sensor/notify, matches INA219 conversion speed comfortably
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last >= 200UL) {
    last = now;

    // Measurements
    float busV  = ina219.getBusVoltage_V();          // Volts (load side)
    float currA = ina219.getCurrent_mA() / 1000.0f;  // Amps (uses calibration)
    float pW    = busV * currA;                      // Watts (or use ina219.getPower_mW()/1000)

    // Rolling average
    addPowerSample(pW);
    float pAvg = avgPower30s();

    // Notify BLE subscribers
    notifyFloatLE(g_vChar, busV);
    notifyFloatLE(g_iChar, currA);
    notifyFloatLE(g_pChar, pW);
    notifyFloatLE(g_pavgChar, pAvg);

    // Throttled serial print (~1 Hz)
    static uint8_t decim=0; if (++decim >= 5) {
      decim = 0;
      Serial.printf("V=%.3f V  I=%.3f A  P=%.3f W  Pavg30s=%.3f W\n", busV, currA, pW, pAvg);
    }
  }
}
