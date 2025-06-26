#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MeasurePi.py — Integrated Flask dashboard + hardware loop for Raspberry Pi 5
— Updated to use cedargrove_nau7802 for NAU7802 scale
"""
# ─── Standard library ────────────────────────────────────────────────────────
import json
import math
import os
import sys
import threading
import time
from pathlib import Path

# ─── Third-party / hardware libraries ───────────────────────────────────────
import board
import busio
import digitalio            # only needed for direct GPIO if you keep that path
import lgpio                # used for XSHUT pins

import paho.mqtt.client as mqtt

# VL53L0X ToF sensor
from adafruit_vl53l0x import VL53L0X                # use this OR the alias below
# or, if you prefer the alias style in the new init function:
# import adafruit_vl53l0x as vl53

# HX7802 / NAU7802 scale (choose one library—Cedargrove OR Adafruit)
from cedargrove_nau7802 import NAU7802              # <- if you’re using Cedargrove
# from adafruit_nau7802 import NAU7802             # <- comment out if not used

# 20×4 I²C LCD
import adafruit_character_lcd.character_lcd_i2c as charlcd

# ─── Flask web service ───────────────────────────────────────────────────────
from flask import Flask, jsonify, render_template, request


# Configuration
MQTT_BROKER = os.getenv("MQTT_BROKER", "10.1.1.85")
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))
MQTT_TOPIC = "measure/data"
CALIB_FILE = Path.home() / ".measure_calib.json"
LOOP_SEC = 0.5
MAX_RAW_HISTORY = 200

gpio_handle = None

SHT_LOX1_PIN = 17   # physical 11
SHT_LOX2_PIN = 27   # physical 13
SHT_LOX3_PIN = 23   # physical 16
BUTTON_PIN = 18

KNOWN_H1, KNOWN_W1, KNOWN_L1 = 21.0, 26.0, 26.0
KNOWN_H2, KNOWN_W2, KNOWN_L2 = 10.5, 11.0, 15.5
KNOWN_WEIGHT_KG = 2.15

rounding_settings = {
    "height": "ceil",
    "length": "ceil",
    "width": "ceil",
    "weight": "none",
}

sensor1 = sensor2 = sensor3 = None
nau = None
lcd = None

baselineH = baselineW = baselineL = 0.0
slopeH = interceptH = slopeW = interceptW = slopeL = interceptL = 0.0
scale_tare = 0.0
scale_factor = 1.0

current_measurement = {}
measurement_history = []
raw_mqtt_history = []
_data_lock = threading.Lock()
_last_lcd_text = ""

mqtt_client = mqtt.Client(client_id="measure_pi")


def _on_connect(client, *_):
    print("[MQTT] Connected" if client.is_connected() else "[MQTT] Connection failed")


mqtt_client.on_connect = _on_connect


def _wait_for_button(prompt):
    print(f"[CAL] {prompt} — press button …")
    while lgpio.gpio_read(gpio_handle, BUTTON_PIN) == 1:
        time.sleep(0.05)
    while lgpio.gpio_read(gpio_handle, BUTTON_PIN) == 0:
        time.sleep(0.05)
    time.sleep(0.05)

# -------------------------------------------------------------------------
# Pin constants you already have elsewhere:
#   SHT_LOX1_PIN, SHT_LOX2_PIN, SHT_LOX3_PIN, BUTTON_PIN
# -------------------------------------------------------------------------

def _init_hardware():
    """Initialise three VL53L0X, NAU7802 scale, 20×4 LCD – robust version."""
    global sensor1, sensor2, sensor3, nau, lcd, gpio_handle

    gpio_handle = lgpio.gpiochip_open(0)
    xshut_pins  = [SHT_LOX1_PIN, SHT_LOX2_PIN, SHT_LOX3_PIN]
    new_addrs   = [0x30, 0x31, 0x32]

    # Hold XSHUT low on all sensors
    for pin in xshut_pins:
        lgpio.gpio_claim_output(gpio_handle, pin, 0)

    # Button
    lgpio.gpio_claim_input(gpio_handle, BUTTON_PIN)

    # I²C bus
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400_000)

    # Helper to print scan results
    def _scan(tag=""):
        while not i2c.try_lock():
            pass
        addrs = [hex(a) for a in i2c.scan()]
        i2c.unlock()
        print(f"[I2C] {tag} scan → {addrs}")
        return addrs

    sensors = []
    # Bring sensors up one-by-one
    for idx, (pin, addr) in enumerate(zip(xshut_pins, new_addrs)):
        lgpio.gpio_write(gpio_handle, pin, 1)      # release reset
        time.sleep(0.05)                           # boot time

        present = _scan(f"after XSHUT {idx}")
        if '0x29' not in present:
            print(f"[WARN] Sensor {idx}: no device at 0x29; skipping.")
            sensors.append(None)
            continue

        try:
            s = vl53.VL53L0X(i2c)   # default address 0x29
            s.set_address(addr)
            sensors.append(s)
            print(f"[VL53] Sensor {idx} re-addressed → 0x{addr:02X}")
        except Exception as e:
            print(f"[ERROR] Sensor {idx} init failed: {e}")
            sensors.append(None)

    # Always return three elements
    while len(sensors) < 3:
        sensors.append(None)
    sensor1, sensor2, sensor3 = sensors

    # Final sanity scan
    _scan("final")

    # NAU7802 scale
    try:
        nau = NAU7802(i2c)
        nau.enable(True)
        nau.calibrate()
    except Exception as e:
        print(f"[WARN] NAU7802 not initialised: {e}")
        nau = None

    # LCD
    try:
        lcd = charlcd.Character_LCD_I2C(i2c, 20, 4)
        lcd.clear()
        lcd.message = "Starting …"
    except Exception as e:
        print(f"[WARN] LCD init failed: {e}")
        lcd = None


def _read_scale_raw(samples=10):
    values = []
    for _ in range(samples):
        if nau.available():
            values.append(nau.read())
        time.sleep(0.05)
    return sum(values) / len(values) if values else 0.0


def _calibrate_tof():
    global baselineH, baselineW, baselineL
    global slopeH, interceptH, slopeW, interceptW, slopeL, interceptL
    lcd.clear()
    lcd.message = "Calibrating ToF …"
    _wait_for_button("Remove object")
    sums = [sum((globals()[f"sensor{i}"].range) for _ in range(10)) for i in (1, 2, 3)]
    baselineH, baselineW, baselineL = [s / 10.0 / 10.0 for s in sums]
    _wait_for_button("Place BOX 1")
    sums1 = [sum((globals()[f"sensor{i}"].range) for _ in range(10)) for i in (1, 2, 3)]
    avg1 = [
        (s / 10.0 / 10.0 - b) for s, b in zip(sums1, (baselineH, baselineW, baselineL))
    ]
    _wait_for_button("Place BOX 2")
    sums2 = [sum((globals()[f"sensor{i}"].range) for _ in range(10)) for i in (1, 2, 3)]
    avg2 = [
        (s / 10.0 / 10.0 - b) for s, b in zip(sums2, (baselineH, baselineW, baselineL))
    ]
    slopeH = (KNOWN_H2 - KNOWN_H1) / (avg2[0] - avg1[0])
    interceptH = KNOWN_H1 - slopeH * avg1[0]
    slopeW = (KNOWN_W2 - KNOWN_W1) / (avg2[1] - avg1[1])
    interceptW = KNOWN_W1 - slopeW * avg1[1]
    slopeL = (KNOWN_L2 - KNOWN_L1) / (avg2[2] - avg1[2])
    interceptL = KNOWN_L1 - slopeL * avg1[2]
    print("[CAL] ToF complete")


def _calibrate_scale():
    global scale_tare, scale_factor
    lcd.clear()
    lcd.message = "Calibrating scale …"
    _wait_for_button("Tare empty scale")
    scale_tare = _read_scale_raw()
    _wait_for_button(f"Place {KNOWN_WEIGHT_KG} kg")
    raw_with_weight = _read_scale_raw()
    scale_factor = (raw_with_weight - scale_tare) / KNOWN_WEIGHT_KG
    print(f"[CAL] Scale factor = {scale_factor:.2f}")


def _save_calibration():
    data = {
        "baselineH": baselineH,
        "baselineW": baselineW,
        "baselineL": baselineL,
        "slopeH": slopeH,
        "interceptH": interceptH,
        "slopeW": slopeW,
        "interceptW": interceptW,
        "slopeL": slopeL,
        "interceptL": interceptL,
        "scale_tare": scale_tare,
        "scale_factor": scale_factor,
    }
    CALIB_FILE.write_text(json.dumps(data))
    print(f"[CAL] Saved → {CALIB_FILE}")


def _load_calibration():
    global baselineH, baselineW, baselineL, slopeH, interceptH, slopeW, interceptW, slopeL, interceptL, scale_tare, scale_factor
    if not CALIB_FILE.exists():
        return False
    try:
        data = json.loads(CALIB_FILE.read_text())
        baselineH = data["baselineH"]
        baselineW = data["baselineW"]
        baselineL = data["baselineL"]
        slopeH = data["slopeH"]
        interceptH = data["interceptH"]
        slopeW = data["slopeW"]
        interceptW = data["interceptW"]
        slopeL = data["slopeL"]
        interceptL = data["interceptL"]
        scale_tare = data["scale_tare"]
        scale_factor = data["scale_factor"]
        return True
    except Exception as e:
        print(f"[CAL] Failed to load: {e}")
        return False


def _apply_rounding(meas):
    out = {}
    for k, v in meas.items():
        if k in ("height", "length", "width") and rounding_settings.get(k) == "ceil":
            out[k] = math.ceil(v)
        elif k == "weight" and rounding_settings.get("weight") != "none":
            try:
                prec = int(rounding_settings["weight"])
            except Exception:
                prec = 2
            out[k] = round(v, prec)
        else:
            out[k] = v
    return out


def _hardware_loop():
    global _last_lcd_text
    while True:
        try:
            h = sensor1.range / 10.0 - baselineH
            w = sensor2.range / 10.0 - baselineW
            l = sensor3.range / 10.0 - baselineL
            corrH = h * slopeH + interceptH
            corrW = w * slopeW + interceptW
            corrL = l * slopeL + interceptL
            raw = _read_scale_raw()
            wt = (raw - scale_tare) / scale_factor
            meas = {"height": corrH, "width": corrW, "length": corrL, "weight": wt}
            ts = time.time()
            with _data_lock:
                current_measurement.clear()
                current_measurement.update(meas)
                measurement_history.append(
                    {"timestamp": ts, "measurement": meas.copy()}
                )
                raw_mqtt_history.append(json.dumps(meas))
                if len(measurement_history) > MAX_RAW_HISTORY:
                    measurement_history.pop(0)
                if len(raw_mqtt_history) > MAX_RAW_HISTORY:
                    raw_mqtt_history.pop(0)
            mqtt_client.publish(MQTT_TOPIC, json.dumps(meas))
            lcd_text = (
                f"H:{corrH:5.1f}cm W:{corrW:5.1f}cm\nL:{corrL:5.1f}cm Wt:{wt:5.2f}kg"
            )
            if lcd_text != _last_lcd_text:
                lcd.clear()
                lcd.message = lcd_text
                _last_lcd_text = lcd_text
        except Exception as e:
            print(f"[HW] Error: {e}")
        time.sleep(LOOP_SEC)


# Flask app
app = Flask(__name__)


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/json")
def json_out():
    with _data_lock:
        curr = _apply_rounding(current_measurement) if current_measurement else {}
        hist = measurement_history[-20:]
    return jsonify({"current": curr, "history": hist})


@app.route("/api/raw")
def raw_out():
    with _data_lock:
        raw = list(raw_mqtt_history)
    return jsonify({"raw": raw})


@app.route("/api/settings", methods=["GET", "POST"])
def settings():
    global rounding_settings
    if request.method == "GET":
        return jsonify(rounding_settings)
    data = request.get_json(force=True) or {}
    for k in rounding_settings.keys():
        if k in data:
            rounding_settings[k] = data[k]
    return jsonify({"status": "updated", "rounding_settings": rounding_settings})


@app.route("/api/lcd")
def lcd_display():
    with _data_lock:
        meas = _apply_rounding(current_measurement) if current_measurement else {}
    if not meas:
        return "LCD: READY"
    return (
        f"H: {meas.get('height', 0)} cm\n"
        f"L: {meas.get('length', 0)} cm\n"
        f"W: {meas.get('width', 0)} cm\n"
        f"Wt: {meas.get('weight', 0)} kg"
    )


@app.route("/api/measurements")
def get_measurements():
    with _data_lock:
        m = current_measurement.copy()
    return jsonify(
        {
            "length": m.get("length", 0),
            "width": m.get("width", 0),
            "height": m.get("height", 0),
            "weight": m.get("weight", 0),
        }
    )


@app.route("/api/serial/command", methods=["POST"])
def serial_command():
    payload = request.get_json(force=True) or {}
    cmd = str(payload.get("command", ""))
    mqtt_client.publish("measure/cmd", cmd)
    return jsonify({"status": "sent", "command": cmd})


@app.route("/api/latest_mqtt_post")
def latest_mqtt_post():
    with _data_lock:
        if not raw_mqtt_history:
            return jsonify({"error": "No MQTT messages available"}), 404
        latest_post_str = raw_mqtt_history[-1]
    try:
        latest_post_json = json.loads(latest_post_str)
        return jsonify(latest_post_json)
    except json.JSONDecodeError:
        # This case should ideally not happen if raw_mqtt_history only stores valid JSON strings
        return jsonify({"error": "Failed to parse stored MQTT message"}), 500


if __name__ == "__main__":
    try:
        _init_hardware()
        if not _load_calibration():
            _calibrate_tof()
            _calibrate_scale()
            _save_calibration()
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
        mqtt_client.loop_start()
        threading.Thread(target=_hardware_loop, daemon=True).start()
        ssl_cert = Path("/home/redlite/measure_pi/cert.pem")
        ssl_key = Path("/home/redlite/measure_pi/key.pem")
        ssl_ctx = (
            (str(ssl_cert), str(ssl_key))
            if ssl_cert.exists() and ssl_key.exists()
            else None
        )
        app.run(host="0.0.0.0", port=int(os.getenv("PORT", 5000)), ssl_context=ssl_ctx)
    finally:
        if gpio_handle is not None:
            lgpio.gpiochip_close(gpio_handle)
        mqtt_client.loop_stop()
