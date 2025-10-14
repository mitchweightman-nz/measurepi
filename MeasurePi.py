#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MeasurePi.py — Integrated Flask dashboard + MQTT client for Raspberry Pi
- Receives sensor data from an external source (e.g., Arduino) via MQTT.
- Displays data on an optional LCD.
- Serves data via a Flask web interface.
- Refactored by AI to remove direct sensor reading and calibration.
"""
# ─── Standard library ────────────────────────────────────────────────────────
import json
import math
import os
import sys
import threading
import time
from pathlib import Path
import traceback

# ─── Third-party / hardware libraries ───────────────────────────────────────
import board 
import busio 

import paho.mqtt.client as mqtt

import adafruit_character_lcd.character_lcd_i2c as charlcd

# ─── Flask web service ───────────────────────────────────────────────────────
from flask import Flask, jsonify, render_template, request


# ─── Configuration Constants ─────────────────────────────────────────────────
MQTT_BROKER = os.getenv("MQTT_BROKER", "10.1.1.85")
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))
MQTT_TOPIC_SUB = "measure/data"
MQTT_COMMAND_TOPIC = "measure/cmd"

MAX_RAW_HISTORY = 200 

SSL_CERT_PATH_ENV = "SSL_CERT_PATH"
SSL_KEY_PATH_ENV = "SSL_KEY_PATH"
DEFAULT_SSL_CERT_PATH = str(Path.home() / "measure_pi" / "cert.pem")
DEFAULT_SSL_KEY_PATH = str(Path.home() / "measure_pi" / "private.pem")

DEFAULT_ROUNDING_SETTINGS = {
    "height": "ceil", 
    "length": "ceil",
    "width": "ceil",
}

# ─── Global State Variables ──────────────────────────────────────────────────
lcd = None  # I2C Character LCD object (initialized in _init_lcd_if_present)

# Shared state for measurements, accessed by MQTT callback and Flask routes
current_measurement = {}  
measurement_history = []  
raw_mqtt_history = []     
_data_lock = threading.Lock()  
_last_lcd_text = ""       

rounding_settings = DEFAULT_ROUNDING_SETTINGS.copy()

mqtt_client = mqtt.Client(client_id=f"measure_pi_client_{os.getpid()}", protocol=mqtt.MQTTv311)


# ─── MQTT Callbacks ──────────────────────────────────────────────────────────
def _on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print(f"[MQTT] Connected successfully to broker {MQTT_BROKER}.")
        client.subscribe(MQTT_TOPIC_SUB)
        print(f"[MQTT] Subscribed to topic: {MQTT_TOPIC_SUB}")
    else:
        print(f"[MQTT] Connection to broker failed with code {rc}. Check broker address and network.")

mqtt_client.on_connect = _on_connect

def _on_disconnect(client, userdata, rc, properties=None):
    print(f"[MQTT] Disconnected from broker with code {rc}.")

mqtt_client.on_disconnect = _on_disconnect

def _safe_float(value):
    try:
        if value is None:
            return None
        if isinstance(value, str):
            trimmed = value.strip()
            if trimmed == "":
                return None
            if trimmed.lower() == "null":
                return None
            return float(trimmed)
        if isinstance(value, (int, float)):
            return float(value)
    except (TypeError, ValueError):
        return None
    return None


def _safe_int(value):
    try:
        if value is None:
            return None
        if isinstance(value, str):
            trimmed = value.strip()
            if trimmed == "" or trimmed.lower() == "null":
                return None
            return int(float(trimmed))
        if isinstance(value, (int, float)):
            return int(value)
    except (TypeError, ValueError):
        return None
    return None


def _on_message(client, userdata, msg):
    global _last_lcd_text
    # print(f"[MQTT] Received message on topic '{msg.topic}': {msg.payload.decode()}")

    if msg.topic == MQTT_TOPIC_SUB:
        try:
            payload_str = msg.payload.decode('utf-8')
            data = json.loads(payload_str)

            if not all(k in data for k in ["height", "width", "length"]):
                print(f"[MQTT] Warning: Received data missing expected keys. Data: {data}")
                return

            new_measurements = {
                "height": _safe_float(data.get("height")),
                "width": _safe_float(data.get("width")),
                "length": _safe_float(data.get("length")),
                "timestamp": time.time()
            }

            # Optional fields published by the UNO firmware
            new_measurements["weight"] = _safe_float(data.get("weight"))
            new_measurements["weight_net"] = _safe_float(data.get("weight_net"))
            new_measurements["weight_gross"] = _safe_float(data.get("weight_gross"))
            new_measurements["tare_g"] = _safe_int(data.get("tare_g"))

            if new_measurements.get("weight") is None and new_measurements.get("weight_net") is not None:
                new_measurements["weight"] = new_measurements["weight_net"]

            with _data_lock:
                current_measurement.clear()
                current_measurement.update(new_measurements)

                measurement_history.append(new_measurements.copy())
                if len(measurement_history) > MAX_RAW_HISTORY:
                    measurement_history.pop(0)

                raw_mqtt_history.append(payload_str)
                if len(raw_mqtt_history) > MAX_RAW_HISTORY:
                    raw_mqtt_history.pop(0)

            if lcd:
                try:
                    display_data = _apply_rounding(new_measurements)

                    disp_h = display_data.get('height') or 0.0
                    disp_w = display_data.get('width') or 0.0
                    disp_l = display_data.get('length') or 0.0

                    weight_val = new_measurements.get("weight")
                    if weight_val is None:
                        weight_text = "Wt:--"
                    else:
                        weight_text = f"Wt:{weight_val:.2f}kg"

                    lcd_l1 = f"H:{disp_h:<5.1f} W:{disp_w:<5.1f}"
                    lcd_l2 = f"L:{disp_l:<5.1f}cm {weight_text}"
                    lcd_l3 = "MQTT Data Updated"
                    lcd_l4 = time.strftime("%H:%M:%S", time.localtime(new_measurements["timestamp"]))

                    lcd_text = (
                        f"{lcd_l1[:20]}\n"
                        f"{lcd_l2[:20]}\n"
                        f"{lcd_l3[:20]}\n"
                        f"{lcd_l4[:20]}"
                    )

                    if lcd_text != _last_lcd_text:
                        lcd.clear()
                        lcd.message = lcd_text
                        _last_lcd_text = lcd_text
                except Exception as e:
                    print(f"[LCD] Error updating LCD from MQTT message: {e}")
                    _last_lcd_text = ""

        except json.JSONDecodeError:
            print(f"[MQTT] Error decoding JSON from topic '{msg.topic}': {msg.payload.decode()}")
        except KeyError as e:
            print(f"[MQTT] Missing key {e} in JSON from topic '{msg.topic}': {msg.payload.decode()}")
        except Exception as e:
            print(f"[MQTT] Error processing message from topic '{msg.topic}': {e}")
            traceback.print_exc()

mqtt_client.on_message = _on_message


# ─── Hardware Interaction (LCD Only) ────────────────────────────
def _init_lcd_if_present():
    global lcd
    print("[LCDINIT] Attempting to initialize LCD...")
    try:
        i2c = busio.I2C(board.SCL, board.SDA, frequency=100_000)
        print(f"[LCDINIT] I2C bus for LCD initialized on SCL={board.SCL}, SDA={board.SDA}.")
        
        lcd_candidate = charlcd.Character_LCD_I2C(i2c, 20, 4)
        lcd_candidate.backlight = True
        lcd_candidate.clear()
        lcd_candidate.message = "MeasurePi Client\nListening to MQTT\nfor sensor data..."
        lcd = lcd_candidate 
        print("[LCDINIT] LCD (20x4 I2C) initialized successfully.")
    except ValueError as e: 
        print(f"[LCDINIT] LCD not found on I2C bus (ValueError): {e}. LCD functionality disabled.")
        lcd = None
    except RuntimeError as e: 
        print(f"[LCDINIT] LCD I2C RuntimeError: {e}. LCD functionality disabled.")
        lcd = None
    except Exception as e:
        print(f"[LCDINIT] LCD initialization failed with unexpected error: {e}. LCD functionality disabled.")
        lcd = None
    
    if lcd is None:
        print("[LCDINIT] Proceeding without LCD.")


def _apply_rounding(measurements: dict) -> dict:
    if not isinstance(measurements, dict): return {}
    rounded_measurements = measurements.copy()

    for key, value in measurements.items():
        if key not in ["height", "width", "length"] or not isinstance(value, (int, float)):
            continue 

        rule = rounding_settings.get(key)
        try:
            if rule == "ceil":
                rounded_measurements[key] = math.ceil(value)
            elif rule == "floor":
                rounded_measurements[key] = math.floor(value)
            elif rule == "none" or rule is None:
                rounded_measurements[key] = round(value, 1) 
            elif isinstance(rule, str) and rule.isdigit():
                precision = int(rule)
                rounded_measurements[key] = round(value, precision)
            else: 
                rounded_measurements[key] = round(value, 1) 
        except Exception as e:
            print(f"[ROUNDING] Error applying rule '{rule}' for key '{key}', value '{value}': {e}")
            rounded_measurements[key] = round(value, 1) 

    return rounded_measurements


# ─── Flask Web Application Routes ───────────────────────────────────────────
app = Flask(__name__)

@app.route("/")
def index_route():
    return render_template("index.html")

@app.route("/json")
def json_data_route():
    with _data_lock:
        current_data_copy = current_measurement.copy() if current_measurement else {}
        history_to_send = [item.copy() for item in measurement_history[-20:]]
    current_rounded = _apply_rounding(current_data_copy)

    for key in ["weight", "weight_net", "weight_gross"]:
        value = current_data_copy.get(key)
        if isinstance(value, (int, float)):
            current_rounded[key] = round(value, 3)
        elif value is None:
            current_rounded[key] = None

    if "tare_g" in current_data_copy:
        tare_value = current_data_copy.get("tare_g")
        current_rounded["tare_g"] = int(tare_value) if isinstance(tare_value, (int, float)) else None

    return jsonify({"current": current_rounded, "history": history_to_send})

@app.route("/api/raw")
def raw_mqtt_history_route():
    with _data_lock:
        raw_data_list = list(raw_mqtt_history) 
    return jsonify({"raw_mqtt_payloads": raw_data_list})

@app.route("/api/settings", methods=["GET", "POST"])
def settings_api_route():
    global rounding_settings
    if request.method == "GET":
        with _data_lock: 
            settings_copy = rounding_settings.copy()
        return jsonify(settings_copy)
    
    if not request.is_json: return jsonify({"error": "Request must be JSON"}), 400
    new_settings_data = request.get_json()
    if not isinstance(new_settings_data, dict): return jsonify({"error": "JSON payload must be an object"}), 400

    updated_any = False
    temp_new_settings = rounding_settings.copy()

    for key, value in new_settings_data.items():
        if key in temp_new_settings: 
            if value in ["ceil", "floor", "none"] or (isinstance(value, str) and value.isdigit() and 0 <= int(value) <= 10):
                temp_new_settings[key] = value
                updated_any = True
                print(f"[API-SETTINGS] Setting '{key}' updated to '{value}'.")
            else:
                print(f"[API-SETTINGS] Invalid value '{value}' for setting '{key}'. Ignored.")
        else:
            print(f"[API-SETTINGS] Unknown setting key '{key}'. Ignored (expected: height, width, length).")
    
    if updated_any:
        with _data_lock: 
            rounding_settings = temp_new_settings
        print(f"[API-SETTINGS] Global rounding_settings updated: {rounding_settings}")
        return jsonify({"status": "updated", "new_settings": rounding_settings})
    else:
        return jsonify({"status": "no valid changes applied", "current_settings": rounding_settings}), 200

@app.route("/api/lcd_text")
def lcd_text_api_route():
    with _data_lock:
        current_data_copy = current_measurement.copy() if current_measurement else {}

    if not current_data_copy:
        return "LCD Status:\nWaiting for MQTT\nSensor Data..."

    display_data = _apply_rounding(current_data_copy)
    disp_h = display_data.get('height', 0.0)
    disp_w = display_data.get('width', 0.0)
    disp_l = display_data.get('length', 0.0)
    timestamp = current_data_copy.get('timestamp', time.time())

    weight_val = current_data_copy.get('weight')
    weight_str = f"Wt:{weight_val:.2f}kg" if isinstance(weight_val, (int, float)) else "Wt:--"

    lcd_l1 = f"H:{disp_h:<5.1f} W:{disp_w:<5.1f}"[:20]
    lcd_l2 = f"L:{disp_l:<5.1f}cm {weight_str}"[:20]
    lcd_l3 = "MQTT Data Feed"[:20]
    lcd_l4 = time.strftime("%H:%M:%S", time.localtime(timestamp))[:20]

    return f"{lcd_l1}\n{lcd_l2}\n{lcd_l3}\n{lcd_l4}"

@app.route("/api/measurements_current")
def current_measurements_api_route():
    with _data_lock:
        measurements_copy = current_measurement.copy() 
    return jsonify(measurements_copy) 

@app.route("/api/command", methods=["POST"])
def command_api_route():
    if not request.is_json: return jsonify({"error": "Request must be JSON"}), 400
    payload = request.get_json()
    command_to_send = str(payload.get("command", "")).strip()
    if not command_to_send: return jsonify({"error": "Command field is missing or empty"}), 400
    
    if mqtt_client.is_connected():
        mqtt_client.publish(MQTT_COMMAND_TOPIC, command_to_send)
        print(f"[API-CMD] Command '{command_to_send}' published to MQTT topic '{MQTT_COMMAND_TOPIC}'.")
        return jsonify({"status": "sent", "command": command_to_send})
    else:
        print(f"[API-CMD] Failed to send command '{command_to_send}': MQTT client not connected.")
        return jsonify({"status": "error", "message": "MQTT client not connected"}), 503

# ─── Main Application Logic ───────────────────────────────────────────────────
def run_application():
    print("[SYSTEM] MeasurePi MQTT Client & Web Server is starting up...")

    try:
        _init_lcd_if_present()

        if MQTT_BROKER:
            try:
                print(f"[MQTT] Attempting to connect to broker at {MQTT_BROKER}:{MQTT_PORT}...")
                mqtt_client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
                mqtt_client.loop_start() 
            except Exception as e:
                print(f"[ERROR] MQTT connection to {MQTT_BROKER}:{MQTT_PORT} failed: {e}")
                if lcd:
                    lcd.clear()
                    lcd.message = f"MQTT Connect ERR:\n{MQTT_BROKER[:20]}"
                    time.sleep(2)
        else:
            print("[MQTT] MQTT_BROKER not configured. MQTT features disabled.")
            if lcd: lcd.message = "MQTT Disabled:\nBroker not set." ; time.sleep(2)
        
        ssl_cert_file_path = Path(os.getenv(SSL_CERT_PATH_ENV, DEFAULT_SSL_CERT_PATH))
        ssl_key_file_path = Path(os.getenv(SSL_KEY_PATH_ENV, DEFAULT_SSL_KEY_PATH))
        
        flask_ssl_context = None
        if ssl_cert_file_path.is_file() and ssl_key_file_path.is_file():
            flask_ssl_context = (str(ssl_cert_file_path), str(ssl_key_file_path))
            print(f"[SYSTEM] SSL for Flask app ENABLED using cert: {ssl_cert_file_path}, key: {ssl_key_file_path}")
        else:
            print("[SYSTEM] SSL for Flask app DISABLED. Certificate or key file not found.")
            if not ssl_cert_file_path.is_file(): print(f"         Missing SSL Cert: {ssl_cert_file_path} (set env var {SSL_CERT_PATH_ENV} to override)")
            if not ssl_key_file_path.is_file(): print(f"         Missing SSL Key:  {ssl_key_file_path} (set env var {SSL_KEY_PATH_ENV} to override)")

        flask_port = int(os.getenv("FLASK_PORT", os.getenv("PORT", 5000)))
        protocol = "https" if flask_ssl_context else "http"
        print(f"[SYSTEM] Starting Flask web server on {protocol}://0.0.0.0:{flask_port}...")
        app.run(host="0.0.0.0", port=flask_port, ssl_context=flask_ssl_context, threaded=True, debug=False)

    except KeyboardInterrupt:
        print("\n[SYSTEM] Shutdown requested by user (KeyboardInterrupt).")
    except Exception as e:
        print(f"[ERROR] UNHANDLED EXCEPTION in main application execution: {e}")
        traceback.print_exc()
    finally:
        print("[SYSTEM] Initiating shutdown sequence...")
        
        if mqtt_client:
            if mqtt_client.is_connected(): 
                print("[MQTT] Disconnecting MQTT client...")
                mqtt_client.loop_stop() 
                mqtt_client.disconnect()
                print("[MQTT] MQTT client disconnected.")
            else: 
                mqtt_client.loop_stop(force=True) 

        if lcd:
            try:
                print("[LCD] Clearing LCD and turning off backlight...")
                lcd.clear()
                lcd.message = "MeasurePi Client\nShutting Down..."
                time.sleep(1)
                lcd.backlight = False
            except Exception as e:
                print(f"[LCD] Error during LCD shutdown: {e}")
        
        print("[SYSTEM] MeasurePi application has shut down.")

# ─── Application Entry Point ────────────────────────────────────────────────
if __name__ == "__main__":
    run_application()
