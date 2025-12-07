/*************************************************************
 * DMSV1 — UNO R4 WiFi + TF-Luna x3 via TCA9548A (no scale)
 * 
 * Features:
 *   - TF-Luna sensors run in continuous mode on 3 TCA channels
 *   - Background monitor watches dimensions over time
 *   - If ANY 2 axes change by >= 10% vs last reading, laser ON
 *     for 8 seconds as a "box positioned" indicator
 *   - PC / MQTT still triggers a proper "Measure" capture
 *   - CAPTURE JSON includes raw distances AND box dimensions:
 *       box = reference_distance - raw_distance
 *   - A0 LOW → continuous raw + box dims printed to Serial
 *   - NeoPixel strip (8 LEDs) shows subsystem status:
 *       0: System heartbeat (blink; green OK, red fault)
 *       1: WiFi
 *       2: MQTT
 *       3: I2C / TCA
 *       4: TF-Luna sensors
 *       5: Laser
 *       6: Live mode (A0)
 *       7: Measurement status:
 *            Blue  = waiting for capture
 *            Purple= capture running
 *            Green = capture completed (short)
 *            Red   = capture error
 *************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <WiFiS3.h>
#include <PubSubClient.h>
#include <TFLI2C.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

/* ------------------------------ Config ------------------------------ */

// Buttons
static const uint8_t PIN_CAPTURE_IN = 12;     // active LOW to GND; internal PULLUP enabled
static const uint8_t PIN_LIVE_MODE  = A0;     // active LOW to GND; live serial streaming

// Misc I/O
static const uint8_t PIN_RESET_OUT  = 11;     // reset pulse to external I2C PCB (active LOW)
static const uint8_t PIN_LASER_OUT  = 10;     // laser driver

// NeoPixel strip
static const uint8_t  NEOPIXEL_PIN   = 7;     // DIN of NeoPixel strip
static const uint16_t NEOPIXEL_COUNT = 8;

// TF-Luna DRDY/RTS inputs
static const uint8_t PIN_RTS_1 = 4;  // Length
static const uint8_t PIN_RTS_2 = 5;  // Height
static const uint8_t PIN_RTS_3 = 6;  // Width

// I2C / TCA9548A
static const uint8_t  TCA_ADDR     = 0x70;
static const uint8_t  TFLUNA_ADDR  = 0x10;    // TF-Luna default
static const uint32_t I2C_CLOCK_HZ = 50000;  // conservative

// TCA channels
static const uint8_t  TCA_CH_TF1   = 2;
static const uint8_t  TCA_CH_TF2   = 3;
static const uint8_t  TCA_CH_TF3   = 4;

// TF-Luna sampling
static const uint8_t  SAMPLES_PER_SENSOR   = 6;
static const uint16_t DRDY_TIMEOUT_MS      = 50;

// Laser policy
static const uint16_t LASER_ON_MS          = 8000;    // 8 seconds laser on-time
static const uint16_t MONITOR_INTERVAL_MS  = 200;     // background monitor period (ms)
static const float    DIM_CHANGE_FRACTION  = 0.05f;   // 05% change threshold
static const uint8_t  AXES_FOR_LASER       = 2;       // any 2 axes must change

// Fixed reference distances (sensor → back wall with NO box)
static const float REF_LENGTH_CM = 80.0f;   // L_ref
static const float REF_HEIGHT_CM = 89.0f;   // H_ref
static const float REF_WIDTH_CM  = 70.0f;   // W_ref

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

/* ------------ Status / measurement state codes (uint8_t) ------------ */

#define ST_OFF      0   // off
#define ST_PENDING  1   // blue  (starting / waiting)
#define ST_OK       2   // green (good)
#define ST_WARN     3   // yellow/amber (degraded)
#define ST_ERROR    4   // red   (bad)
#define ST_ACTIVE   5   // purple (actively doing something)

#define MEAS_WAITING 0   // waiting for capture request
#define MEAS_RUNNING 1   // capture in progress
#define MEAS_DONE    2   // capture completed OK (short-lived)
#define MEAS_ERROR   3   // capture error

/* --------------------------- Globals/State --------------------------- */

static TFLI2C  tfl;

static WiFiClient   wifiClient;
static PubSubClient mqttClient(wifiClient);

// NeoPixel
static Adafruit_NeoPixel strip(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Shared triggers
static volatile bool g_trigCapture = false;   // shared trigger from MQTT or button

// Comm / system
static bool     g_tcaPresent       = false;
static uint8_t  g_commFailCount    = 0;
static uint32_t g_nextHeartbeatMs  = 0;

// Laser state
static uint32_t g_nextLaserOffAt   = 0;
static uint8_t  g_laserOn          = 0;

// Background box-monitor timing
static uint32_t g_nextMonitorMs    = 0;

// Live print throttling when A0 is LOW
static uint32_t g_nextLivePrintMs  = 0;

// Last dimension readings (cm) used for 10% change logic
static float g_lastHeightCm = NAN;
static float g_lastWidthCm  = NAN;
static float g_lastLengthCm = NAN;

// TF-Luna health
static bool g_tfInit1Ok = false;
static bool g_tfInit2Ok = false;
static bool g_tfInit3Ok = false;
static bool g_tfReadError = false;   // set if we see read failures

// Heartbeat pixel timing (Pixel 0)
static uint32_t g_hbNextToggleMs = 0;
static bool     g_hbOn           = false;

// Measurement status (Pixel 7)
static uint8_t  g_measState         = MEAS_WAITING;
static uint32_t g_measStateUntilMs  = 0;

// Callback-safe logging buffer
static bool     g_inCallback       = false;
static char     g_cbLogBuf[192];
static bool     g_cbLogHas         = false;

/* ------------------------------ Debounce ----------------------------- */

class DebouncedButton {
public:
  void begin(uint8_t pin, uint16_t dbMs) {
    _pin = pin;
    _db  = dbMs;
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
  if (!g_inCallback) {
    _logPublish(s);               // never publish while in callback
  } else {
    strncpy(g_cbLogBuf, s, sizeof(g_cbLogBuf)-1);
    g_cbLogBuf[sizeof(g_cbLogBuf)-1]=0;
    g_cbLogHas=true;
  }
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
  size_t i = 0;
  while (i < len && (s[i]==' '||s[i]=='\t'||s[i]=='\r'||s[i]=='\n')) i++;
  size_t j = len;
  while (j>i && (s[j-1]==' '||s[j-1]=='\t'||s[j-1]=='\n')) j--;
  if (i > 0) memmove(s, s + i, j - i);
  s[j - i] = 0;
}

/* ---- JSON helpers ---- */

static void numOrNull(char* out, size_t len, float v, uint8_t dp) {
  if (!isfinite(v)) {
    strncpy(out, "null", len);
    out[len-1]=0;
    return;
  }
  char fmt[8];
  snprintf(fmt, sizeof(fmt), "%%.%uf", dp);
  snprintf(out, len, fmt, v);
}

/* --------------------------- NeoPixel status ------------------------- */

static uint8_t g_pixStatus[NEOPIXEL_COUNT];

// Map status code → RGB color
static uint32_t colorForLevel(uint8_t lvl) {
  switch (lvl) {
    case ST_OFF:     return strip.Color(0, 0, 0);
    case ST_PENDING: return strip.Color(0, 0, 64);     // blue
    case ST_OK:      return strip.Color(0, 64, 0);     // green
    case ST_WARN:    return strip.Color(64, 32, 0);    // yellow/amber
    case ST_ERROR:   return strip.Color(64, 0, 0);     // red
    case ST_ACTIVE:  return strip.Color(48, 0, 64);    // purple
    default:         return strip.Color(0, 0, 0);
  }
}

static void setPixStatus(uint8_t idx, uint8_t lvl) {
  if (idx >= NEOPIXEL_COUNT) return;
  g_pixStatus[idx] = lvl;
}

static void statusLedsBegin() {
  strip.begin();
  strip.setBrightness(40);  // tweak brightness to taste
  for (uint8_t i = 0; i < NEOPIXEL_COUNT; i++) {
    g_pixStatus[i] = ST_OFF;
    strip.setPixelColor(i, 0);
  }
  strip.show();
  g_hbNextToggleMs = millis() + 500;
  g_hbOn = false;
}

static void statusLedsUpdate() {
  // Determine global fault condition
  bool wifiOk   = (WiFi.status() == WL_CONNECTED);
  bool mqttOk   = mqttClient.connected();
  bool tcaOk    = g_tcaPresent;
  bool tfOk     = (g_tfInit1Ok && g_tfInit2Ok && g_tfInit3Ok && !g_tfReadError);
  bool anyError = (!wifiOk || !mqttOk || !tcaOk || !tfOk);

  // Pixel 1: WiFi
  if (!wifiOk) setPixStatus(1, ST_PENDING);   // trying / not connected yet
  else         setPixStatus(1, ST_OK);

  // Pixel 2: MQTT
  if (!mqttOk) setPixStatus(2, ST_PENDING);
  else         setPixStatus(2, ST_OK);

  // Pixel 3: I2C / TCA
  if (!tcaOk)               setPixStatus(3, ST_ERROR);
  else if (g_commFailCount) setPixStatus(3, ST_WARN);
  else                      setPixStatus(3, ST_OK);

  // Pixel 4: TF-Luna sensors
  if (!(g_tfInit1Ok && g_tfInit2Ok && g_tfInit3Ok)) {
    setPixStatus(4, ST_PENDING);          // not all initialised / starting
  } else if (g_tfReadError) {
    setPixStatus(4, ST_WARN);             // intermittent read issues
  } else {
    setPixStatus(4, ST_OK);
  }

  // Pixel 5: Laser
  if (g_laserOn) setPixStatus(5, ST_ACTIVE);
  else           setPixStatus(5, ST_OK);

  // Pixel 6: Live mode (A0)
  if (digitalRead(PIN_LIVE_MODE) == LOW) setPixStatus(6, ST_PENDING); // live streaming
  else                                   setPixStatus(6, ST_OFF);

  // Pixel 7: Measurement status
  switch (g_measState) {
    case MEAS_WAITING: setPixStatus(7, ST_PENDING); break;  // blue
    case MEAS_RUNNING: setPixStatus(7, ST_ACTIVE);  break;  // purple
    case MEAS_DONE:    setPixStatus(7, ST_OK);      break;  // green
    case MEAS_ERROR:   setPixStatus(7, ST_ERROR);   break;  // red
    default:           setPixStatus(7, ST_OFF);     break;
  }

  // Pixel 0: System heartbeat (blink)
  {
    uint16_t interval = anyError ? 150 : 500;  // fast blink on fault, slow on OK
    if ((int32_t)(millis() - g_hbNextToggleMs) >= 0) {
      g_hbOn = !g_hbOn;
      g_hbNextToggleMs = millis() + interval;
    }
    if (g_hbOn) setPixStatus(0, anyError ? ST_ERROR : ST_OK);
    else        setPixStatus(0, ST_OFF);
  }

  // Push to strip
  for (uint8_t i = 0; i < NEOPIXEL_COUNT; i++) {
    strip.setPixelColor(i, colorForLevel(g_pixStatus[i]));
  }
  strip.show();
}

/* ------------------------------ Laser ------------------------------- */

static void laserBegin(uint8_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  g_laserOn = 0;
  g_nextLaserOffAt = 0;
}

static void laserTrigger(uint16_t onMs) {
  digitalWrite(PIN_LASER_OUT, HIGH);
  g_laserOn = 1;
  g_nextLaserOffAt = millis() + onMs;
}

static void laserLoop() {
  if (g_laserOn && (int32_t)(millis() - g_nextLaserOffAt) >= 0) {
    digitalWrite(PIN_LASER_OUT, LOW);
    g_laserOn = 0;
    g_nextLaserOffAt = 0;
  }
}

/* ------------------------------ I2C/TCA ------------------------------ */

static bool tcaSelect(uint8_t ch) {
  if (ch > 7) return false;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1<<ch);
  return Wire.endTransmission() == 0;
}

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
  tfl.Soft_Reset(addr);
  delay(50);
  tfl.Set_Enable(addr);
  tfl.Set_Cont_Mode(addr);
  uint16_t frameRate = 100; // 100 Hz
  tfl.Set_Frame_Rate(frameRate, addr);
  int16_t dump=0;
  (void)dump;
  tfl.getData(dump, addr);   // throw away one sample
}

static bool tflInitOnCh(uint8_t ch, uint8_t addr, const char* name, bool &flagOk) {
  if (!tcaSelect(ch)) {
    logf("[TF] %s: TCA select failed", name);
    flagOk = false;
    return false;
  }
  delay(5);
  tflStartContinuous(addr);
  delay(5);
  logf("[TF] %s: init OK on ch %u addr 0x%02X", name, ch, addr);
  flagOk = true;
  return true;
}

static int16_t tflReadOnceDRDY(uint8_t addr, uint8_t rtsPin) {
  if (!waitRTSHigh(rtsPin, DRDY_TIMEOUT_MS)) {
    int16_t dump=0;
    tfl.getData(dump, addr);
    return -1;
  }
  int16_t cm=-1;
  if (tfl.getData(cm, addr)) return cm;
  return -1;
}

static int16_t tflAverageOnCh(uint8_t ch, uint8_t rtsPin, uint8_t addr) {
  if (!tcaSelect(ch)) return -1;
  delay(3);
  long   sum  = 0;
  uint8_t good = 0;
  for (uint8_t i=0;i<SAMPLES_PER_SENSOR;i++) {
    int16_t cm = tflReadOnceDRDY(addr, rtsPin);
    if (cm >= 0) {
      sum += cm;
      good++;
    }
    delay(2);
  }
  if (!good) return -1;
  return (int16_t)((sum + (good/2)) / good);
}

/* ------------------------------- NET/MQTT ---------------------------- */

static void noteCommFailure() {
  if (++g_commFailCount >= 5) {
    digitalWrite(PIN_RESET_OUT, LOW);
    delay(150);
    digitalWrite(PIN_RESET_OUT, HIGH);
    g_commFailCount = 0;
    logLine("! RESET: external I2C PCB reset pulse");
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
  return equalsIgnoreCase(s,"CAP") ||
         equalsIgnoreCase(s,"CAPTUR") ||
         equalsIgnoreCase(s,"CAPTURE");
}

// Callback: do NOT publish here. Buffer for MQTT log and set flags.
static void mqttCallback(char* topic, byte* payload, unsigned int length) {
  g_inCallback = true;

  char msg[128];
  unsigned int n = min(length, (unsigned int)(sizeof(msg)-1));
  memcpy(msg, payload, n);
  msg[n] = 0;

  char line[192];
  snprintf(line, sizeof(line), "[MQTT] RX %s: %s", topic, msg);
  Serial.println(line);
  strncpy(g_cbLogBuf, line, sizeof(g_cbLogBuf)-1);
  g_cbLogBuf[sizeof(g_cbLogBuf)-1]=0;
  g_cbLogHas = true;

  char tmp[128];
  strncpy(tmp, msg, sizeof(tmp)-1);
  tmp[sizeof(tmp)-1]=0;
  strtrim(tmp);

  if (strEq(topic, TOPIC_CMD)) {
    if (isCaptureCmd(tmp)) {
      g_trigCapture = true;   // main loop handles it
    }
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

  logf("[MQTT] Connecting to %s:%u as '%s'…",
       MQTT_SERVER, MQTT_PORT, MQTT_CLIENT_ID);

  if (!mqttClient.connect(MQTT_CLIENT_ID)) {
    logf("[MQTT] Connect failed (rc=%d)", mqttClient.state());
    return;
  }

  // Clear retained so we only react to NEW commands
  mqttClient.publish(TOPIC_CMD,  "", true);
  mqttClient.publish(TOPIC_TEST, "", true);

  bool s1 = mqttClient.subscribe(TOPIC_CMD);
  bool s2 = mqttClient.subscribe(TOPIC_TEST);
  logf("[MQTT] Connected, subscribed: cmd=%d test=%d (retained cleared)",
       (int)s1, (int)s2);

  // Optional RX probe (non-retained)
  char probe[32];
  snprintf(probe, sizeof(probe), "probe-%lu", (unsigned long)millis());
  mqttClient.publish(TOPIC_TEST, probe, false);
}

/* -------------------------------- Setup ------------------------------ */

void setup() {
  Serial.begin(115200);
  logLine("# Booting DMSV1 measure rig (NeoPixel status + meas state)…");

  pinMode(PIN_RESET_OUT, OUTPUT);
  digitalWrite(PIN_RESET_OUT, HIGH);

  pinMode(PIN_RTS_1, INPUT_PULLUP);
  pinMode(PIN_RTS_2, INPUT_PULLUP);
  pinMode(PIN_RTS_3, INPUT_PULLUP);

  btnCapture.begin(PIN_CAPTURE_IN, /*debounce ms*/30);

  pinMode(PIN_LIVE_MODE, INPUT_PULLUP);  // A0 live-mode switch

  laserBegin(PIN_LASER_OUT);
  statusLedsBegin();

  logf("[GPIO] CAP button D%u, NeoPixel on D%u, live mode A0", PIN_CAPTURE_IN, NEOPIXEL_PIN);

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  logf("[I2C] Started at %lu Hz", (unsigned long)I2C_CLOCK_HZ);

  // TCA presence
  Wire.beginTransmission(TCA_ADDR);
  if (Wire.endTransmission() != 0) {
    logLine("[TCA] ERROR: TCA9548A not found at 0x70");
    g_tcaPresent = false;
  } else {
    logLine("[TCA] Found TCA9548A at 0x70");
    g_tcaPresent = true;
  }

  // TF-Luna init per channel
  tflInitOnCh(TCA_CH_TF1, TFLUNA_ADDR, "TF1(Length)", g_tfInit1Ok);
  tflInitOnCh(TCA_CH_TF2, TFLUNA_ADDR, "TF2(Height)", g_tfInit2Ok);
  tflInitOnCh(TCA_CH_TF3, TFLUNA_ADDR, "TF3(Width)",  g_tfInit3Ok);

  // WiFi up and MQTT connect
  wifiEnsure();
  uint32_t wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis()-wifiStart < 7000) {
    delay(50);
  }
  if (WiFi.status() == WL_CONNECTED) {
    String ip = WiFi.localIP().toString();
    logf("[WIFI] Connected: RSSI=%d IP=%s", WiFi.RSSI(), ip.c_str());
  } else {
    logLine("[WIFI] Connect timeout; will retry in background");
  }

  mqttEnsure();

  logLine("# Ready. Box in corner → dims change → laser on.");
  logLine("# CAPTURE → raw distances + box dims in JSON.");
  logLine("# A0 LOW → continuous raw + box dims on Serial.");
  logLine("# NeoPixel: 0=HB, 1=WiFi, 2=MQTT, 3=I2C, 4=TF, 5=Laser, 6=Live, 7=Measure.");

  g_nextHeartbeatMs = millis() + 2000;
  g_nextMonitorMs   = millis() + 1000;  // give I2C board ~1s before first monitor
  g_nextLivePrintMs = millis();
  g_measState       = MEAS_WAITING;
  g_measStateUntilMs= 0;
}

/* -------------------------- Capture/publish -------------------------- */

// Compute box dimension from raw distance and reference, clamp at >=0
static float boxDimFromRaw(float ref_cm, float raw_cm) {
  if (!isfinite(raw_cm)) return NAN;
  float box = ref_cm - raw_cm;
  if (box < 0.0f) box = 0.0f;   // avoid negative sizes due to noise
  return box;
}

static void publishJson(float h_raw,float w_raw,float l_raw) {
  if (!mqttClient.connected()) return;

  // Box dimensions (ref - raw)
  float h_box = boxDimFromRaw(REF_HEIGHT_CM, h_raw);
  float w_box = boxDimFromRaw(REF_WIDTH_CM,  w_raw);
  float l_box = boxDimFromRaw(REF_LENGTH_CM, l_raw);

  char hRawTxt[16], wRawTxt[16], lRawTxt[16];
  char hBoxTxt[16], wBoxTxt[16], lBoxTxt[16];

  numOrNull(hRawTxt, sizeof(hRawTxt), h_raw, 1);
  numOrNull(wRawTxt, sizeof(wRawTxt), w_raw, 1);
  numOrNull(lRawTxt, sizeof(lRawTxt), l_raw, 1);

  numOrNull(hBoxTxt, sizeof(hBoxTxt), h_box, 1);
  numOrNull(wBoxTxt, sizeof(wBoxTxt), w_box, 1);
  numOrNull(lBoxTxt, sizeof(lBoxTxt), l_box, 1);

  // JSON: computed box dimensions (height/width/length) plus raw distances
  float dimension_max = h_box;
  if (isfinite(w_box) && (!isfinite(dimension_max) || w_box > dimension_max)) dimension_max = w_box;
  if (isfinite(l_box) && (!isfinite(dimension_max) || l_box > dimension_max)) dimension_max = l_box;

  char dimTxt[16];
  numOrNull(dimTxt, sizeof(dimTxt), dimension_max, 1);

  char payload[256];
  snprintf(payload,sizeof(payload),
    "{\"height\":%s,\"width\":%s,\"length\":%s,"
    "\"dimension\":%s,"
    "\"height_raw\":%s,\"width_raw\":%s,\"length_raw\":%s,"
    "\"height_box\":%s,\"width_box\":%s,\"length_box\":%s}",
    hBoxTxt,wBoxTxt,lBoxTxt,
    dimTxt,
    hRawTxt,wRawTxt,lRawTxt,
    hBoxTxt,wBoxTxt,lBoxTxt);

  mqttClient.publish(TOPIC_DATA, payload, false);
  logLine("[MQTT] Publish data OK");
}

static void startCapture() {
  logLine("[CAPTURE] TRIGGERED (begin TF-Luna reads)");

  int16_t length_cm_i = tflAverageOnCh(TCA_CH_TF1, PIN_RTS_1, TFLUNA_ADDR);
  if (length_cm_i < 0) noteCommFailure();

  int16_t height_cm_i = tflAverageOnCh(TCA_CH_TF2, PIN_RTS_2, TFLUNA_ADDR);
  if (height_cm_i < 0) noteCommFailure();

  int16_t width_cm_i  = tflAverageOnCh(TCA_CH_TF3, PIN_RTS_3, TFLUNA_ADDR);
  if (width_cm_i  < 0) noteCommFailure();

  bool okCapture = (length_cm_i >= 0 && height_cm_i >= 0 && width_cm_i >= 0);

  const float length_cm = (length_cm_i >= 0) ? (float)length_cm_i : NAN;
  const float height_cm = (height_cm_i >= 0) ? (float)height_cm_i : NAN;
  const float width_cm  = (width_cm_i  >= 0) ? (float)width_cm_i  : NAN;

  logf("[CAPTURE] TF-Luna results raw L=%d H=%d W=%d",
       length_cm_i,height_cm_i,width_cm_i);

  // Also log computed box dims so you can see sanity on Serial
  float boxL = boxDimFromRaw(REF_LENGTH_CM, length_cm);
  float boxH = boxDimFromRaw(REF_HEIGHT_CM, height_cm);
  float boxW = boxDimFromRaw(REF_WIDTH_CM,  width_cm);
  logf("[CAPTURE] Box dims L=%.1f H=%.1f W=%.1f (cm)", boxL, boxH, boxW);

  publishJson(height_cm, width_cm, length_cm);

  // Update measurement status LED (pixel 7)
  if (okCapture) {
    g_measState = MEAS_DONE;
  } else {
    g_measState = MEAS_ERROR;
  }
  g_measStateUntilMs = millis() + 1000;  // keep DONE/ERROR for 1s, then back to WAITING
}

/* --------------------- Background box monitor ------------------------ */

static bool changed10pct(float oldV, float newV) {
  if (!isfinite(oldV) || !isfinite(newV)) return false;
  if (oldV == 0.0f) return false;  // avoid division nonsense
  float diff = fabsf(newV - oldV);
  float frac = diff / fabsf(oldV);
  return (frac >= DIM_CHANGE_FRACTION);
}

static void monitorBoxPlacement() {
  // Read current dimensions (raw distances in cm)
  int16_t length_cm_i = tflAverageOnCh(TCA_CH_TF1, PIN_RTS_1, TFLUNA_ADDR);
  int16_t height_cm_i = tflAverageOnCh(TCA_CH_TF2, PIN_RTS_2, TFLUNA_ADDR);
  int16_t width_cm_i  = tflAverageOnCh(TCA_CH_TF3, PIN_RTS_3, TFLUNA_ADDR);

  if (length_cm_i < 0 || height_cm_i < 0 || width_cm_i < 0) {
    static uint8_t errCount = 0;
    if (++errCount >= 10) {
      logLine("[MON] TF-Luna read fail (one or more axes <0)");
      errCount = 0;
    }
    g_tfReadError = true;
    return;
  }

  g_tfReadError = false;

  float length_cm = (float)length_cm_i;
  float height_cm = (float)height_cm_i;
  float width_cm  = (float)width_cm_i;

  // LIVE MODE: when A0 is LOW, periodically print current raw + box dims
  if (digitalRead(PIN_LIVE_MODE) == LOW) {
    if ((int32_t)(millis() - g_nextLivePrintMs) >= 0) {
      float liveBoxL = boxDimFromRaw(REF_LENGTH_CM, length_cm);
      float liveBoxH = boxDimFromRaw(REF_HEIGHT_CM, height_cm);
      float liveBoxW = boxDimFromRaw(REF_WIDTH_CM,  width_cm);
      logf("[LIVE] raw L=%.1f H=%.1f W=%.1f (cm)", length_cm, height_cm, width_cm);
      logf("[LIVE] box L=%.1f H=%.1f W=%.1f (cm)", liveBoxL, liveBoxH, liveBoxW);
      g_nextLivePrintMs = millis() + 200;  // 5 Hz print rate
    }
  }

  // First valid reading seeds the "last" state, no laser
  if (!isfinite(g_lastHeightCm) ||
      !isfinite(g_lastWidthCm)  ||
      !isfinite(g_lastLengthCm)) {
    g_lastHeightCm = height_cm;
    g_lastWidthCm  = width_cm;
    g_lastLengthCm = length_cm;
    logf("[MON] Seed dims L=%.1f H=%.1f W=%.1f (cm)",
         length_cm, height_cm, width_cm);
    return;
  }

  uint8_t changedAxes = 0;
  if (changed10pct(g_lastHeightCm, height_cm)) changedAxes++;
  if (changed10pct(g_lastWidthCm,  width_cm))  changedAxes++;
  if (changed10pct(g_lastLengthCm, length_cm)) changedAxes++;

  if (changedAxes >= AXES_FOR_LASER) {
    logf("[MON] Dim change >=%.0f%% on %u axes -> LASER %ums",
         DIM_CHANGE_FRACTION*100.0f,
         (unsigned)changedAxes,
         (unsigned)LASER_ON_MS);
    laserTrigger(LASER_ON_MS);
  }

  // Update last-known dimensions
  g_lastHeightCm = height_cm;
  g_lastWidthCm  = width_cm;
  g_lastLengthCm = length_cm;
}

/* -------------------------------- Loop ------------------------------- */

void loop() {
  mqttEnsure();
  mqttClient.loop();     // process inbound packets

  // Flush any callback-buffered log after we return from the callback
  if (g_cbLogHas) {
    _logPublish(g_cbLogBuf);
    g_cbLogHas = false;
  }

  laserLoop();

  // Measurement state timeout (drop DONE/ERROR back to WAITING)
  if (g_measState == MEAS_DONE || g_measState == MEAS_ERROR) {
    if ((int32_t)(millis() - g_measStateUntilMs) >= 0) {
      g_measState = MEAS_WAITING;
    }
  }

  statusLedsUpdate();

  // Button triggers (shared path with MQTT)
  if (btnCapture.pressedEdge()) {
    logLine("[BTN] CAPTURE pressed");
    g_trigCapture = true;
  }

  // Background dimension monitor (auto-laser logic + live mode)
  if ((int32_t)(millis() - g_nextMonitorMs) >= 0) {
    monitorBoxPlacement();
    g_nextMonitorMs = millis() + MONITOR_INTERVAL_MS;
  }

  // Heartbeat log (software)
  if ((int32_t)(millis() - g_nextHeartbeatMs) >= 0) {
    logf("[SYS] alive laser=%u", g_laserOn);
    g_nextHeartbeatMs = millis() + 2000;
  }

  // CAPTURE command handling
  if (g_trigCapture) {
    g_trigCapture = false;
    g_measState   = MEAS_RUNNING;   // measurement in progress → purple on Pixel 7
    logLine("[CAPTURE] Command accepted");
    startCapture();
  }

  delay(1);
}
