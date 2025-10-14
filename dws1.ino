/*************************************************************
 * UNO R4 WiFi — TF-Luna x3 via TCA9548A + NAU7802 load-cell
 * MQTT + Capture Button, non-blocking scale sampler, JSON nulls,
 * deferred MQTT logging outside callback, retained CMD cleared.
 *
 * Topics:
 *   In  : measure/cmd   (CAP/CAPTURE/CAPTUR, TARE/TAR/ZERO/Z)
 *   In  : measure/test  (echo-only to prove RX path)
 *   Out : measure/data  (JSON)
 *   Out : measure/log   (all logs mirrored from Serial)
 *
 * Wiring:
 *   CAPTURE button to PIN 12, other side to GND (INPUT_PULLUP).
 *
 * Revision: V-12/10/25 01:30 (Add_Capture_Button+Shared_Trigger_Path)
 *************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <WiFiS3.h>
#include <PubSubClient.h>
#include <TFLI2C.h>
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>
#include <math.h>

/* ------------------------------ Config ------------------------------ */

// Buttons
static const uint8_t PIN_CAPTURE_IN = 12;     // active LOW to GND; internal PULLUP enabled

// Misc I/O
static const uint8_t PIN_RESET_OUT  = 11;      // reset pulse to external I2C PCB (active LOW)
static const uint8_t PIN_LASER_OUT  = 10;      // laser driver

// TF-Luna DRDY/RTS inputs
static const uint8_t PIN_RTS_1 = 4;  // Length
static const uint8_t PIN_RTS_2 = 5;  // Height
static const uint8_t PIN_RTS_3 = 6;  // Width

// I2C / TCA9548A
static const uint8_t  TCA_ADDR     = 0x70;
static const uint8_t  TFLUNA_ADDR  = 0x10;    // TF-Luna default
static const uint32_t I2C_CLOCK_HZ = 100000;  // conservative

// TCA channels
static const uint8_t  TCA_CH_TF1   = 0;
static const uint8_t  TCA_CH_TF2   = 1;
static const uint8_t  TCA_CH_TF3   = 2;
static const uint8_t  TCA_CH_NAU   = 3;

// TF-Luna sampling
static const uint8_t  SAMPLES_PER_SENSOR = 6;
static const uint16_t DRDY_TIMEOUT_MS    = 50;

// NAU7802
static float          NAU_CAL_FACTOR     = 2280.0f;  // set with known mass

// Laser policy
static const uint16_t LASER_ON_MS         = 5000;
static const long     VARIANCE_THRESHOLD_G = 5000;

// WiFi / MQTT
static const char WIFI_SSID[]      = "REDLITE";
static const char WIFI_PASS[]      = "wawaqw541296";
static const char MQTT_SERVER[]    = "10.1.1.85";
static const uint16_t MQTT_PORT    = 1883;
static const char MQTT_CLIENT_ID[] = "uno-r4-measure";

// Topics
static const char TOPIC_CMD[]  = "measure/cmd";
static const char TOPIC_TEST[] = "measure/test";
static const char TOPIC_DATA[] = "measure/data";
static const char TOPIC_LOG[]  = "measure/log";

/* --------------------------- Globals/State --------------------------- */

static TFLI2C  tfl;
static NAU7802 g_scale;
static bool    g_scalePresent = false;

static int32_t g_factoryZeroOffset = 0;  // baseline at init
static int32_t g_tareOffsetRaw     = 0;  // additional offset from tare

static WiFiClient   wifiClient;
static PubSubClient mqttClient(wifiClient);

static volatile bool g_trigCapture = false;   // shared trigger from MQTT or button
static volatile bool g_trigTare    = false;

static long     g_lastStableWeight = 0;
static uint8_t  g_commFailCount    = 0;

static uint32_t g_nextHeartbeatMs  = 0;
static uint32_t g_nextLaserOffAt   = 0;
static uint8_t  g_laserOn          = 0;

// Callback-safe logging buffer
static bool     g_inCallback       = false;
static char     g_cbLogBuf[192];
static bool     g_cbLogHas         = false;

/* ------------------------------ Debounce ----------------------------- */

class DebouncedButton {
public:
  void begin(uint8_t pin, uint16_t dbMs) {
    _pin = pin; _db = dbMs;
    pinMode(_pin, INPUT_PULLUP);
    _lastLevel  = digitalRead(_pin);
    _lastChange = millis();
  }
  bool pressedEdge() {
    bool lvl = digitalRead(_pin);
    if (lvl != _lastLevel) {
      uint32_t now = millis();
      if ((uint16_t)(now - _lastChange) >= _db) {
        bool was = _lastLevel;
        _lastLevel = lvl;
        _lastChange = now;
        return (was == HIGH && lvl == LOW);   // falling edge = press
      }
    }
    return false;
  }
private:
  uint8_t  _pin = 0xFF;
  uint16_t _db  = 30;
  uint32_t _lastChange = 0;
  bool     _lastLevel  = HIGH;
};

static DebouncedButton btnCapture;

/* ------------------------------- Utils ------------------------------- */

static void _logPublish(const char* s) {
  if (mqttClient.connected()) mqttClient.publish(TOPIC_LOG, s, false);
}
static void logLine(const char* s) {
  Serial.println(s);
  if (!g_inCallback) _logPublish(s);               // never publish while in callback
  else { strncpy(g_cbLogBuf, s, sizeof(g_cbLogBuf)-1); g_cbLogBuf[sizeof(g_cbLogBuf)-1]=0; g_cbLogHas=true; }
}
static void logf(const char* fmt, ...) {
  char line[256];
  va_list ap; va_start(ap, fmt);
  vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);
  logLine(line);
}
static bool equalsIgnoreCase(const char* a, const char* b) {
  if (!a || !b) return false;
  while (*a && *b) {
    char ca = *a, cb = *b;
    if (ca >= 'A' && ca <= 'Z') ca += 32;
    if (cb >= 'A' && cb <= 'Z') cb += 32;
    if (ca != cb) return false;
    ++a; ++b;
  }
  return *a == 0 && *b == 0;
}
static void strtrim(char* s) {
  if (!s) return;
  size_t len = strlen(s);
  size_t i = 0; while (i < len && (s[i]==' '||s[i]=='\t'||s[i]=='\r'||s[i]=='\n')) i++;
  size_t j = len; while (j>i && (s[j-1]==' '||s[j-1]=='\t'||s[j-1]=='\r'||s[j-1]=='\n')) j--;
  if (i > 0) memmove(s, s + i, j - i);
  s[j - i] = 0;
}

/* ---- JSON helpers: numbers become "null" if NaN/inf to keep JSON valid ---- */

static void numOrNull(char* out, size_t len, float v, uint8_t dp) {
  if (!isfinite(v)) { strncpy(out, "null", len); out[len-1]=0; return; }
  char fmt[8]; snprintf(fmt, sizeof(fmt), "%%.%uf", dp);
  snprintf(out, len, fmt, v);
}

/* ------------------------------ Laser ------------------------------- */

static void laserBegin(uint8_t pin) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); g_laserOn = 0; g_nextLaserOffAt = 0; }
static void laserTrigger(uint16_t onMs) { digitalWrite(PIN_LASER_OUT, HIGH); g_laserOn = 1; g_nextLaserOffAt = millis() + onMs; }
static void laserLoop() {
  if (g_laserOn && (int32_t)(millis() - g_nextLaserOffAt) >= 0) {
    digitalWrite(PIN_LASER_OUT, LOW); g_laserOn = 0; g_nextLaserOffAt = 0;
  }
}

/* ------------------------------ I2C/TCA ------------------------------ */

static bool tcaSelect(uint8_t ch) { if (ch > 7) return false; Wire.beginTransmission(TCA_ADDR); Wire.write(1<<ch); return Wire.endTransmission() == 0; }
static bool selectNAU() { if (!g_scalePresent) return false; return tcaSelect(TCA_CH_NAU); }

/* ------------------------- Non-blocking NAU -------------------------- */

struct WeightSampler {
  bool active=false, success=false;
  uint8_t targetN=0, count=0;
  int64_t acc=0;
  uint32_t timeoutAt=0;
  long gramsNet=0, gramsGross=0, tareGrams=0;

  void start(uint8_t n, uint16_t maxMs) {
    if (!g_scalePresent) { active=false; success=false; return; }
    active = true; success = false; targetN = n; count = 0; acc = 0;
    gramsNet = gramsGross = tareGrams = 0;
    timeoutAt = millis() + maxMs;
    logf("[SCALE] Sampler start N=%u window=%ums", n, (unsigned)maxMs);
  }
  void service(bool tareBusy) {
    if (!active) return;
    if ((int32_t)(millis() - timeoutAt) >= 0) { active=false; success=false; logLine("[SCALE] Sampler timeout"); return; }
    if (tareBusy) return;
    if (!selectNAU()) return;
    if (!g_scale.available()) return;

    int32_t raw = g_scale.getReading();
    acc += raw; count++;
    if (count >= targetN) {
      const int32_t rawAvg = (int32_t)(acc / targetN);
      const float grossF   = (float)(rawAvg - g_factoryZeroOffset) / NAU_CAL_FACTOR;
      const int32_t combinedOffset = g_factoryZeroOffset + g_tareOffsetRaw;
      const float netF    = (float)(rawAvg - combinedOffset) / NAU_CAL_FACTOR;

      gramsGross = lroundf(grossF);
      gramsNet   = lroundf(netF);
      tareGrams  = lroundf((float)g_tareOffsetRaw / NAU_CAL_FACTOR);

      success = true; active = false;
      logf("[SCALE] Sampler done net=%ldg gross=%ldg tare=%ldg", gramsNet, gramsGross, tareGrams);
    }
  }
  bool done() const { return !active; }
} g_ws;

struct TareOp {
  bool active=false, success=false;
  uint8_t targetN=0, count=0;
  int64_t acc=0;
  uint32_t timeoutAt=0;
  void start(uint8_t n, uint16_t maxMs) {
    if (!g_scalePresent) { active=false; success=false; return; }
    active=true; success=false; targetN=n; count=0; acc=0; timeoutAt=millis()+maxMs;
    logf("[SCALE] Tare start N=%u window=%ums", n, (unsigned)maxMs);
  }
  void service() {
    if (!active) return;
    if ((int32_t)(millis() - timeoutAt) >= 0) { active=false; success=false; logLine("[SCALE] Tare timeout"); return; }
    if (!selectNAU()) return;
    if (!g_scale.available()) return;

    int32_t raw = g_scale.getReading();
    acc += raw; count++;
    if (count >= targetN) {
      const int32_t rawAvg = (int32_t)(acc / targetN);
      g_tareOffsetRaw = rawAvg - g_factoryZeroOffset;
      const int32_t combined = g_factoryZeroOffset + g_tareOffsetRaw;
      g_scale.setZeroOffset(combined);
      success = true; active = false;
      const long tare_g = lroundf((float)g_tareOffsetRaw / NAU_CAL_FACTOR);
      logf("[SCALE] Tare OK, tare_g=%ld", tare_g);
    }
  }
  bool done() const { return !active; }
} g_tare;

/* ----------------------------- TF-Luna ------------------------------- */

static bool waitRTSHigh(uint8_t pin, uint16_t timeout_ms) {
  uint32_t start = millis();
  while (digitalRead(pin) == LOW) {
    if ((uint16_t)(millis() - start) >= timeout_ms) return false;
    delayMicroseconds(500);
  }
  return true;
}
static void tflStartContinuous(uint8_t addr) {
  tfl.Soft_Reset(addr); delay(50);
  tfl.Set_Enable(addr); tfl.Set_Cont_Mode(addr);
  uint16_t frameRate = 100; tfl.Set_Frame_Rate(frameRate, addr);
  int16_t dump=0; (void)dump; tfl.getData(dump, addr);
}
static bool tflInitOnCh(uint8_t ch, uint8_t addr, const char* name) {
  if (!tcaSelect(ch)) { logf("[TF] %s: TCA select failed", name); return false; }
  delay(5); tflStartContinuous(addr); delay(5);
  logf("[TF] %s: init OK on ch %u addr 0x%02X", name, ch, addr); return true;
}
static int16_t tflReadOnceDRDY(uint8_t addr, uint8_t rtsPin) {
  if (!waitRTSHigh(rtsPin, DRDY_TIMEOUT_MS)) { int16_t dump=0; tfl.getData(dump, addr); return -1; }
  int16_t cm=-1; if (tfl.getData(cm, addr)) return cm; return -1;
}
static int16_t tflAverageOnCh(uint8_t ch, uint8_t rtsPin, uint8_t addr) {
  if (!tcaSelect(ch)) return -1; delay(3);
  long sum=0; uint8_t good=0;
  for (uint8_t i=0;i<SAMPLES_PER_SENSOR;i++){ int16_t cm=tflReadOnceDRDY(addr,rtsPin); if(cm>=0){sum+=cm;good++;} delay(2); }
  if (!good) return -1;
  return (int16_t)((sum + (good/2)) / good);
}

/* ------------------------------- NET/MQTT ---------------------------- */

static void noteCommFailure() {
  if (++g_commFailCount >= 5) {
    digitalWrite(PIN_RESET_OUT, LOW); delay(150); digitalWrite(PIN_RESET_OUT, HIGH);
    g_commFailCount = 0; logLine("! RESET: external I2C PCB reset pulse");
  }
}

static void wifiEnsure() {
  if (WiFi.status() == WL_CONNECTED) return;
  logf("[WIFI] Connecting to SSID '%s'…", WIFI_SSID);
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

static bool strEq(const char* a, const char* b) { return strcmp(a,b) == 0; }
static bool isCaptureCmd(const char* s) {
  return equalsIgnoreCase(s,"CAP") || equalsIgnoreCase(s,"CAPTUR") || equalsIgnoreCase(s,"CAPTURE");
}
static bool isTareCmd(const char* s) {
  return equalsIgnoreCase(s,"TARE") || equalsIgnoreCase(s,"TAR") || equalsIgnoreCase(s,"ZERO") || equalsIgnoreCase(s,"Z");
}

// Callback: do NOT publish here. Buffer for MQTT log and set flags.
static void mqttCallback(char* topic, byte* payload, unsigned int length) {
  g_inCallback = true;

  char msg[128];
  unsigned int n = min(length, (unsigned int)(sizeof(msg)-1));
  memcpy(msg, payload, n); msg[n] = 0;

  char line[192];
  snprintf(line, sizeof(line), "[MQTT] RX %s: %s", topic, msg);
  Serial.println(line);
  strncpy(g_cbLogBuf, line, sizeof(g_cbLogBuf)-1); g_cbLogBuf[sizeof(g_cbLogBuf)-1]=0;
  g_cbLogHas = true;

  char tmp[128]; strncpy(tmp, msg, sizeof(tmp)-1); tmp[sizeof(tmp)-1]=0; strtrim(tmp);

  if (strEq(topic, TOPIC_CMD)) {
    if (isCaptureCmd(tmp)) g_trigCapture = true;   // main loop handles it
    if (isTareCmd(tmp))    g_trigTare    = true;
  }

  g_inCallback = false;
}

static void mqttEnsure() {
  if (mqttClient.connected()) return;

  wifiEnsure();
  if (WiFi.status() != WL_CONNECTED) return;

  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(512);

  logf("[MQTT] Connecting to %s:%u as '%s'…", MQTT_SERVER, MQTT_PORT, MQTT_CLIENT_ID);
  if (!mqttClient.connect(MQTT_CLIENT_ID)) {
    logf("[MQTT] Connect failed (rc=%d)", mqttClient.state());
    return;
  }

  // Clear retained so we only react to NEW commands
  mqttClient.publish(TOPIC_CMD,  "", true);
  mqttClient.publish(TOPIC_TEST, "", true);

  bool s1 = mqttClient.subscribe(TOPIC_CMD);
  bool s2 = mqttClient.subscribe(TOPIC_TEST);
  logf("[MQTT] Connected, subscribed: cmd=%d test=%d (retained cleared)", (int)s1, (int)s2);

  // Optional RX probe (non-retained)
  char probe[32]; snprintf(probe, sizeof(probe), "probe-%lu", (unsigned long)millis());
  mqttClient.publish(TOPIC_TEST, probe, false);
}

/* -------------------------------- Setup ------------------------------ */

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  logLine("# Booting measure rig…");

  pinMode(PIN_RESET_OUT, OUTPUT); digitalWrite(PIN_RESET_OUT, HIGH);
  pinMode(PIN_RTS_1, INPUT_PULLUP); pinMode(PIN_RTS_2, INPUT_PULLUP); pinMode(PIN_RTS_3, INPUT_PULLUP);
  btnCapture.begin(PIN_CAPTURE_IN, /*debounce ms*/30);
  laserBegin(PIN_LASER_OUT);
  logf("[GPIO] Pins configured (CAP button on D%u, active LOW)", PIN_CAPTURE_IN);

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  logf("[I2C] Started at %lu Hz", (unsigned long)I2C_CLOCK_HZ);

  // TCA presence
  Wire.beginTransmission(TCA_ADDR);
  if (Wire.endTransmission() != 0) logLine("[TCA] ERROR: TCA9548A not found at 0x70");
  else                              logLine("[TCA] Found TCA9548A at 0x70");

  // TF-Luna init per channel
  if (!tflInitOnCh(TCA_CH_TF1, TFLUNA_ADDR, "TF1(Length)")) logLine("[TF] WARN: TF1 init failed");
  if (!tflInitOnCh(TCA_CH_TF2, TFLUNA_ADDR, "TF2(Height)")) logLine("[TF] WARN: TF2 init failed");
  if (!tflInitOnCh(TCA_CH_TF3, TFLUNA_ADDR, "TF3(Width)"))  logLine("[TF] WARN: TF3 init failed");

  // NAU init
  if (tcaSelect(TCA_CH_NAU) && g_scale.begin(Wire)) {
    g_scalePresent = true;
    g_scale.setGain(NAU7802_GAIN_128);
    g_scale.setSampleRate(NAU7802_SPS_80);
    if (!g_scale.calibrateAFE()) logLine("[SCALE] WARN: AFE calibration failed");

    uint8_t primed=0; int64_t acc=0; uint32_t until=millis()+200;
    while ((int32_t)(millis()-until) < 0 && primed < 16) {
      if (tcaSelect(TCA_CH_NAU) && g_scale.available()) { acc += (int32_t)g_scale.getReading(); primed++; }
    }
    g_factoryZeroOffset = (primed>0) ? (int32_t)(acc/primed) : 0;
    g_tareOffsetRaw     = 0;
    g_scale.setZeroOffset(g_factoryZeroOffset);
    g_scale.setCalibrationFactor(NAU_CAL_FACTOR);
    logf("[SCALE] NAU7802 OK, factoryZero=%ld, cal=%.2f", (long)g_factoryZeroOffset, NAU_CAL_FACTOR);
  } else {
    g_scalePresent = false; logLine("[SCALE] NAU7802 not detected: tare disabled, weights null");
  }

  // WiFi up and MQTT connect
  wifiEnsure();
  uint32_t wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis()-wifiStart < 7000) { delay(50); }
  if (WiFi.status() == WL_CONNECTED) {
    String ip = WiFi.localIP().toString();
    logf("[WIFI] Connected: RSSI=%d IP=%s", WiFi.RSSI(), ip.c_str());
  } else {
    logLine("[WIFI] Connect timeout; will retry in background");
  }

  mqttEnsure();

  logLine("# Ready. CAPTURE via MQTT or button. Old retained cmds are cleared.");
  g_nextHeartbeatMs = millis() + 200000;
}

/* -------------------------- Capture/publish -------------------------- */

static void publishJson_any(float h,float w,float l, bool haveScale, float net_kg, float gross_kg, long tare_g){
  if (!mqttClient.connected()) return;

  char hTxt[16], wTxt[16], lTxt[16], netTxt[16], grossTxt[16], tareTxt[16];
  numOrNull(hTxt, sizeof(hTxt), h, 1);
  numOrNull(wTxt, sizeof(wTxt), w, 1);
  numOrNull(lTxt, sizeof(lTxt), l, 1);

  if (haveScale) {
    numOrNull(netTxt,   sizeof(netTxt),   net_kg,   3);
    numOrNull(grossTxt, sizeof(grossTxt), gross_kg, 3);
    snprintf(tareTxt, sizeof(tareTxt), "%ld", tare_g);
  } else {
    strcpy(netTxt, "null"); strcpy(grossTxt, "null"); strcpy(tareTxt, "null");
  }

  char payload[256];
  snprintf(payload,sizeof(payload),
    "{\"height\":%s,\"width\":%s,\"length\":%s,"
    "\"weight\":%s,\"weight_net\":%s,\"weight_gross\":%s,\"tare_g\":%s}",
    hTxt,wTxt,lTxt, netTxt,netTxt,grossTxt,tareTxt);

  mqttClient.publish(TOPIC_DATA, payload, false);
  logLine("[MQTT] Publish data OK");
}

struct PendingCapture {
  bool active=false; float height_cm=NAN, width_cm=NAN, length_cm=NAN;
  void start(float h,float w,float l){
    active=true; height_cm=h; width_cm=w; length_cm=l;
    logf("[CAPTURE] START length=%.1f height=%.1f width=%.1f", l, h, w);
  }
  void tryPublishIfReady(){
    if (!active) return;
    if (!g_scalePresent){
      logf("[CAPTURE] DISTANCES_READY L=%.1f H=%.1f W=%.1f (cm) [NO_SCALE]", length_cm,height_cm,width_cm);
      publishJson_any(height_cm,width_cm,length_cm, /*haveScale=*/false, NAN,NAN,0);
      active=false; return;
    }
    if (!g_ws.done()) return;
    if (g_ws.success){
      if (labs(g_ws.gramsNet - g_lastStableWeight) >= VARIANCE_THRESHOLD_G) laserTrigger(LASER_ON_MS);
      g_lastStableWeight = g_ws.gramsNet;
      const float net_kg   = g_ws.gramsNet   / 1000.0f;
      const float gross_kg = g_ws.gramsGross / 1000.0f;
      logf("[CAPTURE] WEIGHT_READY net=%.3fkg gross=%.3fkg tare=%ldg", net_kg,gross_kg,g_ws.tareGrams);
      publishJson_any(height_cm,width_cm,length_cm, /*haveScale=*/true, net_kg,gross_kg,g_ws.tareGrams);
    } else {
      logf("[CAPTURE] DISTANCES_READY L=%.1f H=%.1f W=%.1f (cm) [SAMPLER_FAIL]", length_cm,height_cm,width_cm);
      publishJson_any(height_cm,width_cm,length_cm, /*haveScale=*/false, NAN,NAN,0);
    }
    active=false;
  }
} g_pending;

static void startCapture() {
  logLine("[CAPTURE] TRIGGERED (begin TF-Luna reads)");
  int16_t length_cm_i = tflAverageOnCh(TCA_CH_TF1, PIN_RTS_1, TFLUNA_ADDR); if (length_cm_i < 0) noteCommFailure();
  int16_t height_cm_i = tflAverageOnCh(TCA_CH_TF2, PIN_RTS_2, TFLUNA_ADDR); if (height_cm_i < 0) noteCommFailure();
  int16_t width_cm_i  = tflAverageOnCh(TCA_CH_TF3, PIN_RTS_3, TFLUNA_ADDR); if (width_cm_i  < 0) noteCommFailure();

  const float length_cm = (length_cm_i >= 0) ? (float)length_cm_i : NAN;
  const float height_cm = (height_cm_i >= 0) ? (float)height_cm_i : NAN;
  const float width_cm  = (width_cm_i  >= 0) ? (float)width_cm_i  : NAN;

  logf("[CAPTURE] TF-Luna results raw L=%d H=%d W=%d", length_cm_i,height_cm_i,width_cm_i);

  g_pending.start(height_cm,width_cm,length_cm);

  if (g_scalePresent) {
    g_ws.start(/*N=*/8, /*maxMs=*/300);
  }
}

/* -------------------------------- Loop ------------------------------- */

void loop() {
  mqttEnsure();
  mqttClient.loop();     // process inbound packets

  // Flush any callback-buffered log after we return from the callback
  if (g_cbLogHas) { _logPublish(g_cbLogBuf); g_cbLogHas = false; }

  laserLoop();

  // Button trigger (shared path with MQTT)
  if (btnCapture.pressedEdge()) {
    logLine("[BTN] CAPTURE pressed");
    g_trigCapture = true;
  }

  g_tare.service();
  g_ws.service(/*tareBusy=*/g_tare.active);
  g_pending.tryPublishIfReady();

  if ((int32_t)(millis() - g_nextHeartbeatMs) >= 0) {
    logf("[SYS] alive laser=%u", g_laserOn);
    g_nextHeartbeatMs = millis() + 2000;
  }

  if (g_trigCapture) {
    g_trigCapture = false;
    logLine("[CAPTURE] Command accepted");
    if (!g_pending.active && !g_tare.active && !g_ws.active) startCapture();
    else logLine("[CAPTURE] Ignored; capture already pending/busy");
  }

  if (g_trigTare) {
    g_trigTare = false;
    if (g_scalePresent && !g_tare.active) { logLine("[MQTT] TARE requested"); g_tare.start(/*N=*/64, /*maxMs=*/1500); }
    else if (!g_scalePresent) { logLine("# Tare ignored (no scale detected)"); }
  }

  delay(1);
}
