/*
 * UNO‑Q Measurement Rig – TF‑Luna Sensors + Bridge
 *
 * This sketch runs on the STM32U585 MCU of the Arduino UNO Q board and
 * provides real‑time control of three TF‑Luna time‑of‑flight range finders
 * connected directly on I2C with unique addresses.  It implements the same
 * measurement and laser logic as the UNO R4 WiFi version, but instead of
 * publishing over MQTT directly it communicates with the Linux side of
 * the UNO Q via the Router Bridge RPC interface.  A companion Python
 * script running on the embedded Debian system receives measurement
 * notifications and handles network publishing and UI updates.
 *
 * Version: Ver-2601110800
 */

#include <Arduino.h>
#include <Wire.h>
#include <TFLI2C.h>
#include <Adafruit_NeoPixel.h>
#include <Arduino_RouterBridge.h>

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

// TF‑Luna DRDY/RTS inputs
static const uint8_t PIN_RTS_1 = 4;  // Length
static const uint8_t PIN_RTS_2 = 5;  // Height
static const uint8_t PIN_RTS_3 = 6;  // Width

// I2C addresses (unique per TF‑Luna)
static const uint8_t  TFLUNA_ADDR_HEIGHT = 0x10;
static const uint8_t  TFLUNA_ADDR_WIDTH  = 0x20;
static const uint8_t  TFLUNA_ADDR_LENGTH = 0x30;
static const uint32_t I2C_CLOCK_HZ = 50000;  // conservative

// TF‑Luna sampling
static const uint8_t  SAMPLES_PER_SENSOR    = 6;
static const uint16_t DRDY_TIMEOUT_MS      = 50;

// Laser policy
static const uint16_t LASER_ON_MS          = 8000;    // 8 seconds laser on‑time
static const uint16_t MONITOR_INTERVAL_MS  = 200;     // background monitor period (ms)
static const float    DIM_CHANGE_FRACTION  = 0.05f;   // 5% change threshold
static const uint8_t  AXES_FOR_LASER       = 2;       // any 2 axes must change

// Fixed reference distances (sensor → back wall with NO box)
static const float REF_LENGTH_CM = 80.0f;   // L_ref
static const float REF_HEIGHT_CM = 89.0f;   // H_ref
static const float REF_WIDTH_CM  = 70.0f;   // W_ref

/* ------------ Status / measurement state codes (uint8_t) ------------ */

#define ST_OFF      0   // off
#define ST_PENDING  1   // blue  (starting / waiting)
#define ST_OK       2   // green (good)
#define ST_WARN     3   // yellow/amber (degraded)
#define ST_ERROR    4   // red   (bad)
#define ST_ACTIVE   5   // purple (actively doing something)

#define MEAS_WAITING 0   // waiting for capture request
#define MEAS_RUNNING 1   // capture in progress
#define MEAS_DONE    2   // capture completed OK (short‑lived)
#define MEAS_ERROR   3   // capture error

/* --------------------------- Globals/State --------------------------- */

static TFLI2C  tfl;

// NeoPixel
static Adafruit_NeoPixel strip(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Shared triggers
static volatile bool g_trigCapture = false;   // shared trigger from Bridge or button

// Comm / system
static uint8_t  g_commFailCount    = 0;
static uint32_t g_nextHeartbeatMs  = 0;

// Laser state
static uint32_t g_nextLaserOffAt   = 0;
static uint8_t  g_laserOn          = 0;

// Background box‑monitor timing
static uint32_t g_nextMonitorMs    = 0;

// Live print throttling when A0 is LOW
static uint32_t g_nextLivePrintMs  = 0;

// Last dimension readings (cm) used for 5% change logic
static float g_lastHeightCm = NAN;
static float g_lastWidthCm  = NAN;
static float g_lastLengthCm = NAN;

// TF‑Luna health
static bool g_tfInit1Ok = false;
static bool g_tfInit2Ok = false;
static bool g_tfInit3Ok = false;
static bool g_tfReadError = false;   // set if we see read failures

// Heartbeat pixel timing (Pixel 0)
static uint32_t g_hbNextToggleMs = 0;
static bool     g_hbOn           = false;

// Measurement status (Pixel 7)
static uint8_t  g_measState         = MEAS_WAITING;
static uint32_t g_measStateUntilMs  = 0;

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

static void logLine(const char* s) {
  // For UNO Q sketches, Serial prints go to the serial monitor in the MCU IDE.
  Serial.println(s);
}

static void logf(const char* fmt, ...) {
  char line[256];
  va_list ap; va_start(ap, fmt);
  vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);
  logLine(line);
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
  // Pixel 3: I2C status (comm failures)
  if (g_commFailCount) setPixStatus(3, ST_WARN);
  else                 setPixStatus(3, ST_OK);
  // Pixel 4: TF‑Luna sensors
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
    bool anyError = (g_commFailCount || g_tfReadError);
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

/* ----------------------------- TF‑Luna ------------------------------- */

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
  uint16_t frameRate = 100; // 100 Hz
  tfl.Set_Frame_Rate(frameRate, addr);
  int16_t dump=0;
  (void)dump;
  tfl.getData(dump, addr);   // throw away one sample
}

static bool tflInit(uint8_t addr, const char* name, bool &flagOk) {

  Wire.beginTransmission(addr);
  if (Wire.endTransmission() != 0) {
    logf("[TF] %s: sensor not found at addr 0x%02X", name, addr);
    flagOk = false;
Wire.beginTransmission(addr);
  if (Wire.endTransmission() != 0) {
    logf("[TF] %s: sensor not found at addr 0x%02X", name, addr);
    flagOk = false;
    return false;
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

static int16_t tflAverage(uint8_t rtsPin, uint8_t addr) {
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

/* ------------------------- Bridge Functions -------------------------- */

// Triggered from Python: request a new capture
static void bridgeCapture() {
  g_trigCapture = true;
}

/* ------------------------------ Capture/publish -------------------------- */

// Compute box dimension from raw distance and reference, clamp at >=0
static float boxDimFromRaw(float ref_cm, float raw_cm) {
  if (!isfinite(raw_cm)) return NAN;
  float box = ref_cm - raw_cm;
  if (box < 0.0f) box = 0.0f;   // avoid negative sizes due to noise
  return box;
}

static void notifyMeasurement(float h_raw,float w_raw,float l_raw) {
  Bridge.notify("measurement_data", h_raw, w_raw, l_raw);
}

static void startCapture() {
  logLine("[CAPTURE] TRIGGERED (begin TF‑Luna reads)");
  int16_t length_cm_i = tflAverage(PIN_RTS_1, TFLUNA_ADDR_LENGTH);
  if (length_cm_i < 0) g_commFailCount++;
  int16_t height_cm_i = tflAverage(PIN_RTS_2, TFLUNA_ADDR_HEIGHT);
  if (height_cm_i < 0) g_commFailCount++;
  int16_t width_cm_i  = tflAverage(PIN_RTS_3, TFLUNA_ADDR_WIDTH);
  if (width_cm_i  < 0) g_commFailCount++;
  bool okCapture = (length_cm_i >= 0 && height_cm_i >= 0 && width_cm_i >= 0);
  const float length_cm = (length_cm_i >= 0) ? (float)length_cm_i : NAN;
  const float height_cm = (height_cm_i >= 0) ? (float)height_cm_i : NAN;
  const float width_cm  = (width_cm_i  >= 0) ? (float)width_cm_i  : NAN;
  logf("[CAPTURE] TF‑Luna results raw L=%d H=%d W=%d",
       length_cm_i,height_cm_i,width_cm_i);
  float boxL = boxDimFromRaw(REF_LENGTH_CM, length_cm);
  float boxH = boxDimFromRaw(REF_HEIGHT_CM, height_cm);
  float boxW = boxDimFromRaw(REF_WIDTH_CM,  width_cm);
  logf("[CAPTURE] Box dims L=%.1f H=%.1f W=%.1f (cm)", boxL, boxH, boxW);
  notifyMeasurement(height_cm, width_cm, length_cm);
  if (okCapture) {
    g_measState = MEAS_DONE;
  } else {
    g_measState = MEAS_ERROR;
  }
  g_measStateUntilMs = millis() + 1000;
}

/* --------------------- Background box monitor ------------------------ */

static bool changedFrac(float oldV, float newV) {
  if (!isfinite(oldV) || !isfinite(newV)) return false;
  if (oldV == 0.0f) return false;
  float diff = fabsf(newV - oldV);
  float frac = diff / fabsf(oldV);
  return (frac >= DIM_CHANGE_FRACTION);
}

static void monitorBoxPlacement() {
  int16_t length_cm_i = tflAverage(PIN_RTS_1, TFLUNA_ADDR_LENGTH);
  int16_t height_cm_i = tflAverage(PIN_RTS_2, TFLUNA_ADDR_HEIGHT);
  int16_t width_cm_i  = tflAverage(PIN_RTS_3, TFLUNA_ADDR_WIDTH);
  if (length_cm_i < 0 || height_cm_i < 0 || width_cm_i < 0) {
    static uint8_t errCount = 0;
    if (++errCount >= 10) {
      logLine("[MON] TF‑Luna read fail (one or more axes <0)");
      errCount = 0;
    }
    g_tfReadError = true;
    return;
  }
  g_tfReadError = false;
  float length_cm = (float)length_cm_i;
  float height_cm = (float)height_cm_i;
  float width_cm  = (float)width_cm_i;
  if (digitalRead(PIN_LIVE_MODE) == LOW) {
    if ((int32_t)(millis() - g_nextLivePrintMs) >= 0) {
      float liveBoxL = boxDimFromRaw(REF_LENGTH_CM, length_cm);
      float liveBoxH = boxDimFromRaw(REF_HEIGHT_CM, height_cm);
      float liveBoxW = boxDimFromRaw(REF_WIDTH_CM,  width_cm);
      logf("[LIVE] raw L=%.1f H=%.1f W=%.1f (cm)", length_cm, height_cm, width_cm);
      logf("[LIVE] box L=%.1f H=%.1f W=%.1f (cm)", liveBoxL, liveBoxH, liveBoxW);
      g_nextLivePrintMs = millis() + 200;
    }
  }
  if (!isfinite(g_lastHeightCm) || !isfinite(g_lastWidthCm)  || !isfinite(g_lastLengthCm)) {
    g_lastHeightCm = height_cm;
    g_lastWidthCm  = width_cm;
    g_lastLengthCm = length_cm;
    logf("[MON] Seed dims L=%.1f H=%.1f W=%.1f (cm)",
         length_cm, height_cm, width_cm);
    return;
  }
  uint8_t changedAxes = 0;
  if (changedFrac(g_lastHeightCm, height_cm)) changedAxes++;
  if (changedFrac(g_lastWidthCm,  width_cm))  changedAxes++;
  if (changedFrac(g_lastLengthCm, length_cm)) changedAxes++;
  if (changedAxes >= AXES_FOR_LASER) {
    logf("[MON] Dim change >=%.0f%% on %u axes -> LASER %ums",
         DIM_CHANGE_FRACTION*100.0f,
         (unsigned)changedAxes,
         (unsigned)LASER_ON_MS);
    laserTrigger(LASER_ON_MS);
  }
  g_lastHeightCm = height_cm;
  g_lastWidthCm  = width_cm;
  g_lastLengthCm = length_cm;
}

/* -------------------------------- Setup ------------------------------ */

void setup() {
  Serial.begin(115200);
  logLine("# Booting UNO‑Q measure rig (NeoPixel status + meas state)…");
  pinMode(PIN_RESET_OUT, OUTPUT);
  digitalWrite(PIN_RESET_OUT, HIGH);
  pinMode(PIN_RTS_1, INPUT_PULLUP);
  pinMode(PIN_RTS_2, INPUT_PULLUP);
  pinMode(PIN_RTS_3, INPUT_PULLUP);
  btnCapture.begin(PIN_CAPTURE_IN, /*debounce ms*/30);
  pinMode(PIN_LIVE_MODE, INPUT_PULLUP);
  laserBegin(PIN_LASER_OUT);
  statusLedsBegin();
  logf("[GPIO] CAP button D%u, NeoPixel on D%u, live mode A0", PIN_CAPTURE_IN, NEOPIXEL_PIN);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  logf("[I2C] Started at %lu Hz", (unsigned long)I2C_CLOCK_HZ);
  tflInit(TFLUNA_ADDR_LENGTH, "TF1(Length)", g_tfInit1Ok);
  tflInit(TFLUNA_ADDR_HEIGHT, "TF2(Height)", g_tfInit2Ok);
  tflInit(TFLUNA_ADDR_WIDTH, "TF3(Width)",  g_tfInit3Ok);
  Bridge.begin();
  delay(2000);
  bool start = false;
  while (!start) {
    Bridge.call("linux_started").result(start);
    delay(200);
  }
  Bridge.provide("capture", bridgeCapture);
  Bridge.notify("mcu_ready");
  logLine("# Ready. UNO‑Q Bridge initialised.");
  g_nextHeartbeatMs = millis() + 2000;
  g_nextMonitorMs   = millis() + 1000;
  g_nextLivePrintMs = millis();
  g_measState       = MEAS_WAITING;
  g_measStateUntilMs= 0;
}

/* -------------------------------- Loop ------------------------------- */

void loop() {
  Bridge.loop();
  laserLoop();
  if (g_measState == MEAS_DONE || g_measState == MEAS_ERROR) {
    if ((int32_t)(millis() - g_measStateUntilMs) >= 0) {
      g_measState = MEAS_WAITING;
    }
  }
  statusLedsUpdate();
  if (btnCapture.pressedEdge()) {
    logLine("[BTN] CAPTURE pressed");
    g_trigCapture = true;
  }
  if ((int32_t)(millis() - g_nextMonitorMs) >= 0) {
    monitorBoxPlacement();
    g_nextMonitorMs = millis() + MONITOR_INTERVAL_MS;
  }
  if ((int32_t)(millis() - g_nextHeartbeatMs) >= 0) {
    logf("[SYS] alive laser=%u", g_laserOn);
    g_nextHeartbeatMs = millis() + 2000;
  }
  if (g_trigCapture) {
    g_trigCapture = false;
    g_measState   = MEAS_RUNNING;
    logLine("[CAPTURE] Command accepted");
    startCapture();
  }
  delay(1);
}
