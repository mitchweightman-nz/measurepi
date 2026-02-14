/*
 * UNO Q Measurement Rig – TF-Luna Sensors + RouterBridge
 *
 * MCU side sketch:
 * - Talks to Linux side using Arduino_RouterBridge
 * - Linux side runs bridge.py and mqtt.py
 *
 * Key fix:
 * - Only registers Bridge.provide("capture") AFTER linux_started() returns true.
 *   This prevents "method capture not available (2)".
 */

#include <Arduino.h>
#include <Wire.h>
#include <TFLI2C.h>
#include <Adafruit_NeoPixel.h>
#include <Arduino_RouterBridge.h>

#include <math.h>
#include <stdarg.h>

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

// I2C addresses (unique per TF-Luna)
static const uint8_t  TFLUNA_ADDR_HEIGHT = 0x10;
static const uint8_t  TFLUNA_ADDR_WIDTH  = 0x20;
static const uint8_t  TFLUNA_ADDR_LENGTH = 0x30;
static const uint32_t I2C_CLOCK_HZ       = 50000;  // conservative

// TF-Luna sampling
static const uint8_t  SAMPLES_PER_SENSOR = 6;
static const uint16_t DRDY_TIMEOUT_MS    = 50;

// Laser policy
static const uint16_t LASER_ON_MS         = 8000;    // 8 seconds laser on-time
static const uint16_t MONITOR_INTERVAL_MS = 200;     // background monitor period (ms)
static const float    DIM_CHANGE_FRACTION = 0.05f;   // 5% change threshold
static const uint8_t  AXES_FOR_LASER      = 2;       // any 2 axes must change

// Fixed reference distances (sensor -> back wall with NO box)
static const float REF_LENGTH_CM = 80.0f;
static const float REF_HEIGHT_CM = 89.0f;
static const float REF_WIDTH_CM  = 70.0f;

/* ------------ Status / measurement state codes (uint8_t) ------------ */

#define ST_OFF      0   // off
#define ST_PENDING  1   // blue
#define ST_OK       2   // green
#define ST_WARN     3   // amber
#define ST_ERROR    4   // red
#define ST_ACTIVE   5   // purple

#define MEAS_WAITING 0
#define MEAS_RUNNING 1
#define MEAS_DONE    2
#define MEAS_ERROR   3

/* --------------------------- Globals/State --------------------------- */

static TFLI2C  tfl;

// NeoPixel
static Adafruit_NeoPixel strip(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Shared triggers
static volatile bool g_trigCapture = false;

// Comm / system
static uint8_t  g_commFailCount    = 0;
static uint32_t g_nextHeartbeatMs  = 0;

// Laser state
static uint32_t g_nextLaserOffAt   = 0;
static uint8_t  g_laserOn          = 0;

// Background box-monitor timing
static uint32_t g_nextMonitorMs    = 0;

// Live print throttling when A0 is LOW
static uint32_t g_nextLivePrintMs  = 0;

// Last dimension readings (cm) used for % change logic
static float g_lastHeightCm = NAN;
static float g_lastWidthCm  = NAN;
static float g_lastLengthCm = NAN;

// TF-Luna health
static bool g_tfInit1Ok = false;
static bool g_tfInit2Ok = false;
static bool g_tfInit3Ok = false;
static bool g_tfReadError = false;

// Heartbeat pixel timing (Pixel 0)
static uint32_t g_hbNextToggleMs = 0;
static bool     g_hbOn           = false;

// Measurement status (Pixel 7)
static uint8_t  g_measState         = MEAS_WAITING;
static uint32_t g_measStateUntilMs  = 0;

// Init + sampling state
static uint8_t  g_initSensorIndex = 0;
static uint8_t  g_initStep = 0;
static uint32_t g_initNextMs = 0;
static bool     g_systemReady = false;
static uint32_t g_bridgeNextPingMs = 0;

// Bridge provide registration
static bool g_bridgeProvided = false;

enum OperationKind : uint8_t {
  OP_NONE = 0,
  OP_CAPTURE,
  OP_MONITOR,
};

struct SensorInfo {
  uint8_t addr;
  uint8_t rtsPin;
};

static const SensorInfo kSensors[] = {
  {TFLUNA_ADDR_LENGTH, PIN_RTS_1},
  {TFLUNA_ADDR_HEIGHT, PIN_RTS_2},
  {TFLUNA_ADDR_WIDTH,  PIN_RTS_3},
};

struct SensorSampler {
  uint8_t addr;
  uint8_t rtsPin;
  uint8_t sampleIndex;
  uint8_t goodSamples;
  long sum;
  uint32_t nextSampleMs;
  uint32_t waitStartMs;
  bool waitingRts;
  bool done;
  int16_t result;
};

static OperationKind g_opKind = OP_NONE;
static bool g_opActive = false;
static uint8_t g_opSensorIndex = 0;
static SensorSampler g_sampler;
static int16_t g_opResults[3] = {-1, -1, -1};

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
        return (was == HIGH && lvl == LOW);
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

static uint32_t colorForLevel(uint8_t lvl) {
  switch (lvl) {
    case ST_OFF:     return strip.Color(0, 0, 0);
    case ST_PENDING: return strip.Color(0, 0, 64);
    case ST_OK:      return strip.Color(0, 64, 0);
    case ST_WARN:    return strip.Color(64, 32, 0);
    case ST_ERROR:   return strip.Color(64, 0, 0);
    case ST_ACTIVE:  return strip.Color(48, 0, 64);
    default:         return strip.Color(0, 0, 0);
  }
}

static void setPixStatus(uint8_t idx, uint8_t lvl) {
  if (idx >= NEOPIXEL_COUNT) return;
  g_pixStatus[idx] = lvl;
}

static void statusLedsBegin() {
  strip.begin();
  strip.setBrightness(40);
  for (uint8_t i = 0; i < NEOPIXEL_COUNT; i++) {
    g_pixStatus[i] = ST_OFF;
    strip.setPixelColor(i, 0);
  }
  strip.show();
  g_hbNextToggleMs = millis() + 500;
  g_hbOn = false;
}

static void statusLedsUpdate() {
  if (g_commFailCount) setPixStatus(3, ST_WARN);
  else                 setPixStatus(3, ST_OK);

  if (!(g_tfInit1Ok && g_tfInit2Ok && g_tfInit3Ok)) {
    setPixStatus(4, ST_PENDING);
  } else if (g_tfReadError) {
    setPixStatus(4, ST_WARN);
  } else {
    setPixStatus(4, ST_OK);
  }

  if (g_laserOn) setPixStatus(5, ST_ACTIVE);
  else           setPixStatus(5, ST_OK);

  if (digitalRead(PIN_LIVE_MODE) == LOW) setPixStatus(6, ST_PENDING);
  else                                   setPixStatus(6, ST_OFF);

  switch (g_measState) {
    case MEAS_WAITING: setPixStatus(7, ST_PENDING); break;
    case MEAS_RUNNING: setPixStatus(7, ST_ACTIVE);  break;
    case MEAS_DONE:    setPixStatus(7, ST_OK);      break;
    case MEAS_ERROR:   setPixStatus(7, ST_ERROR);   break;
    default:           setPixStatus(7, ST_OFF);     break;
  }

  {
    bool anyError = (g_commFailCount || g_tfReadError);
    uint16_t interval = anyError ? 150 : 500;
    if ((int32_t)(millis() - g_hbNextToggleMs) >= 0) {
      g_hbOn = !g_hbOn;
      g_hbNextToggleMs = millis() + interval;
    }
    if (g_hbOn) setPixStatus(0, anyError ? ST_ERROR : ST_OK);
    else        setPixStatus(0, ST_OFF);
  }

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

/* ----------------------------- TF-Luna ------------------------------- */

static void samplerBegin(SensorSampler &sampler, uint8_t addr, uint8_t rtsPin) {
  sampler.addr = addr;
  sampler.rtsPin = rtsPin;
  sampler.sampleIndex = 0;
  sampler.goodSamples = 0;
  sampler.sum = 0;
  sampler.nextSampleMs = millis();
  sampler.waitStartMs = 0;
  sampler.waitingRts = false;
  sampler.done = false;
  sampler.result = -1;
}

static void samplerStep(SensorSampler &sampler) {
  if (sampler.done) return;

  uint32_t now = millis();

  if (sampler.sampleIndex >= SAMPLES_PER_SENSOR) {
    if (sampler.goodSamples == 0) {
      sampler.result = -1;
    } else {
      sampler.result = (int16_t)((sampler.sum + (sampler.goodSamples / 2)) / sampler.goodSamples);
    }
    sampler.done = true;
    return;
  }

  if (!sampler.waitingRts) {
    if ((int32_t)(now - sampler.nextSampleMs) < 0) return;
    sampler.waitingRts = true;
    sampler.waitStartMs = now;
  }

  if (digitalRead(sampler.rtsPin) == HIGH) {
    int16_t cm = -1;
    if (tfl.getData(cm, sampler.addr) && cm >= 0) {
      sampler.sum += cm;
      sampler.goodSamples++;
    }
    sampler.waitingRts = false;
    sampler.sampleIndex++;
    sampler.nextSampleMs = now + 2;
    return;
  }

  if ((uint16_t)(now - sampler.waitStartMs) >= DRDY_TIMEOUT_MS) {
    int16_t dump = 0;
    tfl.getData(dump, sampler.addr);
    sampler.waitingRts = false;
    sampler.sampleIndex++;
    sampler.nextSampleMs = now + 2;
  }
}

/* ------------------------- Bridge Functions -------------------------- */

static void bridgeCapture() {
  g_trigCapture = true;
}

/* -------------------------- Capture/publish -------------------------- */

static float boxDimFromRaw(float ref_cm, float raw_cm) {
  if (!isfinite(raw_cm)) return NAN;
  float box = ref_cm - raw_cm;
  if (box < 0.0f) box = 0.0f;
  return box;
}

static void notifyMeasurement(float h_raw, float w_raw, float l_raw) {
  Bridge.notify("measurement_data", h_raw, w_raw, l_raw);
}

static void processCaptureResults(int16_t length_cm_i, int16_t height_cm_i, int16_t width_cm_i) {
  if (length_cm_i < 0) g_commFailCount++;
  if (height_cm_i < 0) g_commFailCount++;
  if (width_cm_i  < 0) g_commFailCount++;

  bool okCapture = (length_cm_i >= 0 && height_cm_i >= 0 && width_cm_i >= 0);

  logf("[CAPTURE] TF-Luna results raw L=%d H=%d W=%d", length_cm_i, height_cm_i, width_cm_i);

  const float length_cm = (length_cm_i >= 0) ? (float)length_cm_i : NAN;
  const float height_cm = (height_cm_i >= 0) ? (float)height_cm_i : NAN;
  const float width_cm  = (width_cm_i  >= 0) ? (float)width_cm_i  : NAN;

  float boxL = boxDimFromRaw(REF_LENGTH_CM, length_cm);
  float boxH = boxDimFromRaw(REF_HEIGHT_CM, height_cm);
  float boxW = boxDimFromRaw(REF_WIDTH_CM,  width_cm);

  logf("[CAPTURE] Box dims L=%.1f H=%.1f W=%.1f (cm)", boxL, boxH, boxW);

  notifyMeasurement(height_cm, width_cm, length_cm);

  if (okCapture) g_measState = MEAS_DONE;
  else           g_measState = MEAS_ERROR;

  g_measStateUntilMs = millis() + 1000;
}

static bool changedFrac(float oldV, float newV) {
  if (!isfinite(oldV) || !isfinite(newV)) return false;
  if (oldV == 0.0f) return false;
  float diff = fabsf(newV - oldV);
  float frac = diff / fabsf(oldV);
  return (frac >= DIM_CHANGE_FRACTION);
}

static void processMonitorResults(int16_t length_cm_i, int16_t height_cm_i, int16_t width_cm_i) {
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

  if (!isfinite(g_lastHeightCm) || !isfinite(g_lastWidthCm) || !isfinite(g_lastLengthCm)) {
    g_lastHeightCm = height_cm;
    g_lastWidthCm  = width_cm;
    g_lastLengthCm = length_cm;
    return;
  }

  uint8_t changedAxes = 0;
  if (changedFrac(g_lastHeightCm, height_cm)) changedAxes++;
  if (changedFrac(g_lastWidthCm,  width_cm))  changedAxes++;
  if (changedFrac(g_lastLengthCm, length_cm)) changedAxes++;

  if (changedAxes >= AXES_FOR_LASER) {
    laserTrigger(LASER_ON_MS);
  }

  g_lastHeightCm = height_cm;
  g_lastWidthCm  = width_cm;
  g_lastLengthCm = length_cm;
}

/* ---------------------- Operation step logic -------------------------- */

static void startOperation(OperationKind kind) {
  g_opKind = kind;
  g_opActive = true;
  g_opSensorIndex = 0;
  g_opResults[0] = -1;
  g_opResults[1] = -1;
  g_opResults[2] = -1;
  samplerBegin(g_sampler, kSensors[0].addr, kSensors[0].rtsPin);
}

static void finishOperation() {
  if (g_opKind == OP_CAPTURE) {
    processCaptureResults(g_opResults[0], g_opResults[1], g_opResults[2]);
  } else if (g_opKind == OP_MONITOR) {
    processMonitorResults(g_opResults[0], g_opResults[1], g_opResults[2]);
  }
  g_opKind = OP_NONE;
  g_opActive = false;
}

static void operationStep() {
  if (!g_opActive) return;

  samplerStep(g_sampler);
  if (!g_sampler.done) return;

  if (g_opSensorIndex < 3) {
    g_opResults[g_opSensorIndex] = g_sampler.result;
  }

  g_opSensorIndex++;
  if (g_opSensorIndex >= 3) {
    finishOperation();
    return;
  }

  samplerBegin(g_sampler, kSensors[g_opSensorIndex].addr, kSensors[g_opSensorIndex].rtsPin);
}

/* ------------------------- Init sensors step -------------------------- */

static void initSensorsStep() {
  if (g_initSensorIndex >= 3) return;

  uint32_t now = millis();
  if ((int32_t)(now - g_initNextMs) < 0) return;

  SensorInfo info = kSensors[g_initSensorIndex];

  const char* name = (g_initSensorIndex == 0) ? "TF1(Length)"
                     : (g_initSensorIndex == 1) ? "TF2(Height)"
                     : "TF3(Width)";

  bool* flagOk = (g_initSensorIndex == 0) ? &g_tfInit1Ok
                 : (g_initSensorIndex == 1) ? &g_tfInit2Ok
                 : &g_tfInit3Ok;

  switch (g_initStep) {
    case 0: {
      Wire.beginTransmission(info.addr);
      if (Wire.endTransmission() != 0) {
        logf("[TF] %s: sensor not found at addr 0x%02X", name, info.addr);
        *flagOk = false;
        g_initSensorIndex++;
        return;
      }
      tfl.Soft_Reset(info.addr);
      g_initNextMs = now + 50;
      g_initStep = 1;
      break;
    }

    case 1:
      tfl.Set_Enable(info.addr);
      g_initNextMs = now + 5;
      g_initStep = 2;
      break;

    case 2:
      tfl.Set_Cont_Mode(info.addr);
      g_initNextMs = now + 5;
      g_initStep = 3;
      break;

    case 3: {
      uint16_t frameRate = 100;
      tfl.Set_Frame_Rate(frameRate, info.addr);
      g_initNextMs = now + 5;
      g_initStep = 4;
      break;
    }

    case 4: {
      int16_t dump = 0;
      tfl.getData(dump, info.addr);
      logf("[TF] %s: init OK addr 0x%02X", name, info.addr);
      *flagOk = true;
      g_initStep = 0;
      g_initSensorIndex++;
      break;
    }

    default:
      g_initStep = 0;
      break;
  }
}

/* -------------------------- Bridge init step -------------------------- */

static void bridgeInitStep() {
  if (g_systemReady) return;

  uint32_t now = millis();
  if (g_bridgeNextPingMs != 0 && (int32_t)(now - g_bridgeNextPingMs) < 0) return;

  bool start = false;
  Bridge.call("linux_started").result(start);

  if (start) {

    if (!g_bridgeProvided) {
      Bridge.provide("capture", bridgeCapture);
      g_bridgeProvided = true;
      logLine("[BRIDGE] Provided method: capture");
    }

    Bridge.notify("mcu_ready");
    logLine("# Ready. UNO Q Bridge initialised.");

    g_nextHeartbeatMs = millis() + 2000;
    g_nextMonitorMs   = millis() + 1000;
    g_nextLivePrintMs = millis();

    g_measState = MEAS_WAITING;
    g_measStateUntilMs = 0;

    g_systemReady = true;
    return;
  }

  g_bridgeNextPingMs = now + 200;
}

/* -------------------------------- Setup ------------------------------ */

void setup() {
  Serial.begin(115200);
  logLine("# Booting UNO Q measure rig (Bridge mode)...");

  pinMode(PIN_RESET_OUT, OUTPUT);
  digitalWrite(PIN_RESET_OUT, HIGH);

  pinMode(PIN_RTS_1, INPUT_PULLUP);
  pinMode(PIN_RTS_2, INPUT_PULLUP);
  pinMode(PIN_RTS_3, INPUT_PULLUP);

  btnCapture.begin(PIN_CAPTURE_IN, 30);
  pinMode(PIN_LIVE_MODE, INPUT_PULLUP);

  laserBegin(PIN_LASER_OUT);
  statusLedsBegin();

  logf("[GPIO] CAP button D%u, NeoPixel on D%u, live mode A0", PIN_CAPTURE_IN, NEOPIXEL_PIN);

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  logf("[I2C] Started at %lu Hz", (unsigned long)I2C_CLOCK_HZ);

  Bridge.begin();

  g_bridgeNextPingMs = millis() + 250;
}

/* -------------------------------- Loop ------------------------------- */

void loop() {
  laserLoop();

  if (!g_systemReady) {
    initSensorsStep();
    bridgeInitStep();
    statusLedsUpdate();
    return;
  }

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

  if (g_trigCapture && !g_opActive) {
    g_trigCapture = false;
    g_measState   = MEAS_RUNNING;

    logLine("[CAPTURE] Command accepted");
    logLine("[CAPTURE] TRIGGERED (begin TF-Luna reads)");

    startOperation(OP_CAPTURE);
  }

  if ((int32_t)(millis() - g_nextMonitorMs) >= 0 && !g_opActive) {
    startOperation(OP_MONITOR);
    g_nextMonitorMs = millis() + MONITOR_INTERVAL_MS;
  }

  if ((int32_t)(millis() - g_nextHeartbeatMs) >= 0) {
    logf("[SYS] alive laser=%u", g_laserOn);
    g_nextHeartbeatMs = millis() + 2000;
  }

  operationStep();
}
