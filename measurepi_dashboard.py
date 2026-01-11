#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
measurepi_dashboard.py — Simplified MeasurePi dashboard for the UNO Q

This module provides a lightweight Flask dashboard and MQTT client for the
Arduino UNO Q.  It subscribes to box measurement messages published by the
UNO Q bridge (`bridge_mqtt.py`) and exposes a web API compatible with the
original MeasurePi Raspberry Pi dashboard.  By moving the dashboard to the
UNO Q’s Linux side, all functionality previously running on the Raspberry Pi
can reside on a single device.

The dashboard intentionally omits hardware‑specific features such as the
character LCD, serial scale reader and UPnP port mapping.  If you wish to
restore those features you can refer to the original MeasurePi.py from the
repository and port the code as needed.  For most deployments on the UNO Q,
the simplified dashboard is sufficient.

Version: Ver-2601111000
"""

# ─── Standard library ────────────────────────────────────────────────────────
import json
import math
import os
import threading
import time
import traceback
from pathlib import Path

# ─── Third‑party libraries ──────────────────────────────────────────────────
import paho.mqtt.client as mqtt
from flask import Flask, jsonify, make_response, render_template, request
import urllib.parse

# ─── Configuration Constants ─────────────────────────────────────────────────
# MQTT broker configuration.  These defaults match the original MeasurePi
# dashboard but can be overridden via environment variables.
MQTT_BROKER = os.getenv("MQTT_BROKER", "localhost")
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))
MQTT_TOPIC_SUB = os.getenv("MQTT_DATA_TOPIC", "measure/data")
MQTT_TOPIC_LOG = os.getenv("MQTT_LOG_TOPIC", "measure/log")
MQTT_COMMAND_TOPIC = os.getenv("MQTT_CMD_TOPIC", "measure/cmd")

# Number of raw MQTT payloads and log messages to keep in memory
MAX_RAW_HISTORY = 200

# Rounding rules for box dimensions; keys are axis names and values are
# rounding modes: "ceil", "floor", "none" or an integer number of decimal
# places.
DEFAULT_ROUNDING_SETTINGS = {
    "height": "ceil",
    "length": "ceil",
    "width": "ceil",
}

# ─── Global State Variables ─────────────────────────────────────────────────
current_measurement = {}
measurement_history = []
raw_mqtt_history = []
measure_log_history = []
_data_lock = threading.Lock()
rounding_settings = DEFAULT_ROUNDING_SETTINGS.copy()

# Track whether an external scale is detected; in this simplified version we
# assume no serial scale is connected.  The flag may be set via MQTT data.
scale_present_flag = None
awaiting_manual_weight = False
pending_manual_weight_value = None

mqtt_client = mqtt.Client(client_id=f"measurepi_dashboard_{os.getpid()}", protocol=mqtt.MQTTv311)

# ─── Helper Functions ────────────────────────────────────────────────────────
def _safe_float(value):
    """Return a float if the input is numeric-like, otherwise return None."""
    try:
        if value is None:
            return None
        if isinstance(value, str):
            trimmed = value.strip()
            if trimmed == "" or trimmed.lower() == "null":
                return None
            return float(trimmed)
        if isinstance(value, (int, float)):
            return float(value)
    except (TypeError, ValueError):
        return None
    return None


def _safe_int(value):
    """Return an integer if the input is int-like, otherwise return None."""
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


def _apply_rounding(measurements: dict) -> dict:
    """Apply rounding rules to the height, width and length values."""
    if not isinstance(measurements, dict):
        return {}
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
                rounded_measurements[key] = float(value)
            elif isinstance(rule, str) and rule.isdigit():
                precision = int(rule)
                rounded_measurements[key] = round(value, precision)
            else:
                # default to one decimal place if rule is unrecognized
                rounded_measurements[key] = round(value, 1)
        except Exception:
            rounded_measurements[key] = round(value, 1)
    return rounded_measurements


# ─── MQTT Callbacks ──────────────────────────────────────────────────────────
def _on_connect(client, userdata, flags, rc, properties=None):
    """Handle successful or failed connection to the MQTT broker."""
    if rc == 0:
        print(f"[MQTT] Connected successfully to broker {MQTT_BROKER}:{MQTT_PORT}.")
        client.subscribe(MQTT_TOPIC_SUB)
        client.subscribe(MQTT_TOPIC_LOG)
        print(f"[MQTT] Subscribed to topics: {MQTT_TOPIC_SUB}, {MQTT_TOPIC_LOG}")
    else:
        print(f"[MQTT] Connection failed with code {rc}.")


def _on_disconnect(client, userdata, rc, properties=None):
    print(f"[MQTT] Disconnected from broker with code {rc}.")


def _on_message(client, userdata, msg):
    """Process incoming MQTT messages for measurement data and logs."""
    global scale_present_flag, awaiting_manual_weight, pending_manual_weight_value
    # Data messages from the UNO Q bridge
    if msg.topic == MQTT_TOPIC_SUB:
        try:
            payload_str = msg.payload.decode("utf-8")
            data = json.loads(payload_str)
            timestamp = time.time()

            def pick_dimension_value(key_candidates):
                for candidate in key_candidates:
                    value = _safe_float(data.get(candidate))
                    if value is not None:
                        return value
                return None

            single_dimension = pick_dimension_value(["dimension", "dimension_cm", "dim"])
            new_measurements = {
                "height": pick_dimension_value(["height", "height_box", "height_cm"]) or single_dimension,
                "width": pick_dimension_value(["width", "width_box", "width_cm"]) or single_dimension,
                "length": pick_dimension_value(["length", "length_box", "length_cm"]) or single_dimension,
                "timestamp": timestamp,
            }
            # Bail out if no dimensions were provided
            if all(new_measurements.get(axis) is None for axis in ("height", "width", "length")):
                print(f"[MQTT] Warning: Received data missing dimension values. Data: {data}")
                return

            # Optional weight fields; preserve net/gross if present
            new_measurements["weight"] = _safe_float(data.get("weight"))
            new_measurements["weight_net"] = _safe_float(data.get("weight_net"))
            new_measurements["weight_gross"] = _safe_float(data.get("weight_gross"))
            new_measurements["tare_g"] = _safe_int(data.get("tare_g"))

            have_scale = any(
                new_measurements.get(key) is not None for key in ("weight", "weight_net", "weight_gross", "tare_g")
            )
            new_measurements["scale_present"] = have_scale

            # Copy additional metadata fields if present
            for meta_key in ["manual_weight", "manual_weight_required", "manual_weight_timestamp"]:
                if meta_key in data:
                    new_measurements[meta_key] = data[meta_key]

            with _data_lock:
                current_measurement.clear()
                current_measurement.update(new_measurements)
                measurement_history.append(new_measurements.copy())
                if len(measurement_history) > MAX_RAW_HISTORY:
                    measurement_history.pop(0)
                raw_mqtt_history.append(payload_str)
                if len(raw_mqtt_history) > MAX_RAW_HISTORY:
                    raw_mqtt_history.pop(0)

            scale_present_flag = have_scale
        except json.JSONDecodeError:
            print(f"[MQTT] Error decoding JSON: {msg.payload!r}")
        except Exception as e:
            print(f"[MQTT] Error processing message: {e}")
            traceback.print_exc()
    elif msg.topic == MQTT_TOPIC_LOG:
        try:
            payload_str = msg.payload.decode("utf-8", errors="replace")
        except Exception:
            payload_str = repr(msg.payload)
        entry = {"topic": msg.topic, "payload": payload_str, "timestamp": time.time()}
        with _data_lock:
            measure_log_history.append(entry)
            if len(measure_log_history) > MAX_RAW_HISTORY:
                measure_log_history.pop(0)
    else:
        try:
            payload_str = msg.payload.decode("utf-8", errors="replace")
        except Exception:
            payload_str = repr(msg.payload)
        print(f"[MQTT] Received message on unexpected topic '{msg.topic}': {payload_str}")


# Attach callbacks to the MQTT client
mqtt_client.on_connect = _on_connect
mqtt_client.on_disconnect = _on_disconnect
mqtt_client.on_message = _on_message

# ─── Flask Web Application ───────────────────────────────────────────────────
app = Flask(__name__)


def _extract_origin_from_referer(referer_value):
    if not referer_value:
        return None
    parsed_referer = urllib.parse.urlparse(referer_value)
    if not parsed_referer.scheme or not parsed_referer.netloc:
        return None
    return f"{parsed_referer.scheme}://{parsed_referer.netloc}"


def _parse_allowed_origins():
    raw_value = os.getenv("CORS_ALLOWED_ORIGINS", "")
    allowed = set(["https://nzc.gosweetspot.com"])
    for item in raw_value.split(","):
        trimmed = item.strip()
        if trimmed:
            allowed.add(trimmed.lower())
    return allowed


_ALLOWED_ORIGINS = _parse_allowed_origins()


def _is_origin_allowed(origin):
    if not origin:
        return False
    if not _ALLOWED_ORIGINS:
        return True
    return origin.lower() in _ALLOWED_ORIGINS


def _merge_vary(response, values):
    existing = response.headers.get("Vary", "")
    merged_values = [item.strip() for item in existing.split(",") if item.strip()] if existing else []
    for value in values:
        if value not in merged_values:
            merged_values.append(value)
    if merged_values:
        response.headers["Vary"] = ", ".join(merged_values)


def _add_cors_headers(response):
    request_origin = request.headers.get("Origin")
    if not request_origin:
        request_origin = _extract_origin_from_referer(request.headers.get("Referer"))
    if not _is_origin_allowed(request_origin):
        return response
    response.headers["Access-Control-Allow-Origin"] = request_origin
    response.headers["Access-Control-Allow-Credentials"] = "true"
    request_headers = request.headers.get(
        "Access-Control-Request-Headers", "Content-Type, Authorization"
    )
    response.headers["Access-Control-Allow-Headers"] = request_headers
    request_method = request.headers.get("Access-Control-Request-Method")
    response.headers["Access-Control-Allow-Methods"] = request_method if request_method else "GET, POST, OPTIONS"
    _merge_vary(response, ["Origin", "Referer"])
    return response


@app.before_request
def handle_options_request():
    if request.method == "OPTIONS":
        response = make_response("", 204)
        return _add_cors_headers(response)
    return None


@app.after_request
def add_cors_headers(response):
    return _add_cors_headers(response)


@app.route("/")
def index_route():
    """Render a simple landing page indicating the API is running."""
    return render_template("index.html")


@app.route("/json")
def json_data_route():
    """Return the current and recent measurement history as JSON."""
    with _data_lock:
        current_data_copy = current_measurement.copy() if current_measurement else {}
        history_to_send = [item.copy() for item in measurement_history[-20:]]
    current_rounded = _apply_rounding(current_data_copy)
    # Round weight fields to 3 decimals
    for key in ["weight", "weight_net", "weight_gross"]:
        value = current_data_copy.get(key)
        if isinstance(value, (int, float)):
            current_rounded[key] = round(value, 3)
        elif value is None:
            current_rounded[key] = None
    # Derive dimension from the largest axis
    dimension_candidates = [
        v
        for v in [
            current_rounded.get("height"),
            current_rounded.get("width"),
            current_rounded.get("length"),
            current_data_copy.get("dimension"),
            current_data_copy.get("dimension_cm"),
        ]
        if isinstance(v, (int, float))
    ]
    current_rounded["dimension"] = max(dimension_candidates) if dimension_candidates else None
    # Copy metadata if present
    if "tare_g" in current_data_copy:
        tare_value = current_data_copy.get("tare_g")
        current_rounded["tare_g"] = int(tare_value) if isinstance(tare_value, (int, float)) else None
    for meta_key in ["scale_present", "manual_weight_required", "manual_weight", "manual_weight_timestamp"]:
        if meta_key in current_data_copy:
            current_rounded[meta_key] = current_data_copy[meta_key]
    return jsonify({"current": current_rounded, "history": history_to_send})


@app.route("/api/raw")
def raw_mqtt_history_route():
    """Return the raw MQTT payloads and log messages received."""
    with _data_lock:
        raw_data_list = list(raw_mqtt_history)
        log_data_list = [entry.copy() for entry in measure_log_history]
    return jsonify({"raw_mqtt_payloads": raw_data_list, "log_messages": log_data_list})


@app.route("/api/settings", methods=["GET", "POST"])
def settings_api_route():
    """Get or update rounding settings for height, width and length."""
    global rounding_settings
    if request.method == "GET":
        with _data_lock:
            settings_copy = rounding_settings.copy()
        return jsonify(settings_copy)
    if not request.is_json:
        return jsonify({"error": "Request must be JSON"}), 400
    new_settings_data = request.get_json()
    if not isinstance(new_settings_data, dict):
        return jsonify({"error": "JSON payload must be an object"}), 400
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
    """Return a multi-line string representing what would be shown on the LCD."""
    with _data_lock:
        current_data_copy = current_measurement.copy() if current_measurement else {}
    if not current_data_copy:
        return "LCD Status:\nWaiting for MQTT\nSensor Data..."
    display_data = _apply_rounding(current_data_copy)
    disp_h = display_data.get("height", 0.0)
    disp_w = display_data.get("width", 0.0)
    disp_l = display_data.get("length", 0.0)
    timestamp = current_data_copy.get("timestamp", time.time())
    weight_val = current_data_copy.get("weight")
    weight_str = f"Wt:{weight_val:.2f}kg" if isinstance(weight_val, (int, float)) else "Wt:--"
    lcd_l1 = f"H:{disp_h:<5.1f} W:{disp_w:<5.1f}"[:20]
    lcd_l2 = f"L:{disp_l:<5.1f}cm {weight_str}"[:20]
    lcd_l3 = "MQTT Data Feed"[:20]
    lcd_l4 = time.strftime("%H:%M:%S", time.localtime(timestamp))[:20]
    return f"{lcd_l1}\n{lcd_l2}\n{lcd_l3}\n{lcd_l4}"


@app.route("/api/measurements_current")
def current_measurements_api_route():
    """Return the latest measurement object as JSON."""
    with _data_lock:
        measurements_copy = current_measurement.copy()
    return jsonify(measurements_copy)


@app.route("/api/command", methods=["POST"])
def command_api_route():
    """Publish an arbitrary command string to the MQTT command topic."""
    if not request.is_json:
        return jsonify({"error": "Request must be JSON"}), 400
    payload = request.get_json()
    command_to_send = str(payload.get("command", "")).strip()
    if not command_to_send:
        return jsonify({"error": "Command field is missing or empty"}), 400
    if mqtt_client.is_connected():
        mqtt_client.publish(MQTT_COMMAND_TOPIC, command_to_send)
        print(f"[API-CMD] Command '{command_to_send}' published to MQTT topic '{MQTT_COMMAND_TOPIC}'.")
        return jsonify({"status": "sent", "command": command_to_send})
    else:
        print(f"[API-CMD] Failed to send command '{command_to_send}': MQTT client not connected.")
        return jsonify({"status": "error", "message": "MQTT client not connected"}), 503


@app.route("/api/capture", methods=["POST"])
def capture_api_route():
    """Request a new measurement from the UNO Q bridge via MQTT."""
    global awaiting_manual_weight
    with _data_lock:
        current_scale_state = scale_present_flag
        if current_scale_state is False:
            awaiting_manual_weight = True
        else:
            awaiting_manual_weight = False
    if mqtt_client.is_connected():
        mqtt_client.publish(MQTT_COMMAND_TOPIC, "CAP")
        print(f"[API-CAPTURE] Capture command published to MQTT topic '{MQTT_COMMAND_TOPIC}'.")
        response_payload = {
            "status": "sent",
            "message": "Capture requested.",
            "scale_present": current_scale_state,
            "manual_weight_required": current_scale_state is False,
        }
        if current_scale_state is False:
            response_payload["message"] = "Capture requested. Scale offline – please enter weight manually."
        return jsonify(response_payload)
    else:
        print("[API-CAPTURE] Failed to publish capture command: MQTT client not connected.")
        return jsonify({"status": "error", "message": "MQTT client not connected"}), 503


@app.route("/api/manual_weight", methods=["POST"])
def manual_weight_api_route():
    """Accept a user‑submitted weight when a physical scale is unavailable."""
    global pending_manual_weight_value, awaiting_manual_weight, scale_present_flag
    if not request.is_json:
        return jsonify({"error": "Request must be JSON"}), 400
    payload = request.get_json()
    if not isinstance(payload, dict):
        return jsonify({"error": "JSON payload must be an object"}), 400
    manual_weight_value = _safe_float(payload.get("weight"))
    if manual_weight_value is None:
        return jsonify({"error": "A numeric 'weight' value (kg) is required"}), 400
    if manual_weight_value < 0:
        return jsonify({"error": "Weight must be zero or greater"}), 400
    manual_weight_value = float(manual_weight_value)
    manual_weight_value = round(manual_weight_value, 3)
    timestamp = time.time()
    applied_immediately = False
    with _data_lock:
        pending_manual_weight_value = manual_weight_value
        awaiting_manual_weight = False
        if current_measurement:
            current_weight = current_measurement.get("weight")
            if not isinstance(current_weight, (int, float)):
                current_measurement["weight"] = manual_weight_value
                current_measurement["weight_net"] = manual_weight_value
                current_measurement["weight_gross"] = manual_weight_value
                current_measurement["manual_weight"] = True
                current_measurement["manual_weight_timestamp"] = timestamp
                current_measurement["manual_weight_required"] = False
                current_measurement["scale_present"] = False
                applied_immediately = True
        if measurement_history:
            last_entry = measurement_history[-1]
            last_weight = last_entry.get("weight") if isinstance(last_entry, dict) else None
            if not isinstance(last_weight, (int, float)):
                last_entry["weight"] = manual_weight_value
                last_entry["weight_net"] = manual_weight_value
                last_entry["weight_gross"] = manual_weight_value
                last_entry["manual_weight"] = True
                last_entry["manual_weight_timestamp"] = timestamp
                last_entry["manual_weight_required"] = False
                last_entry["scale_present"] = False
                applied_immediately = True
        if applied_immediately:
            pending_manual_weight_value = None
            scale_present_flag = False
    print(f"[MANUAL-WEIGHT] {'Applied' if applied_immediately else 'Stored'} manual weight {manual_weight_value:.3f}kg")
    return jsonify({
        "status": "accepted",
        "applied": applied_immediately,
        "weight": manual_weight_value,
    })


# ─── Main Application Logic ───────────────────────────────────────────────────
def run_application():
    """Start the MQTT client and Flask server."""
    print("[SYSTEM] MeasurePi dashboard is starting up...")
    flask_port = max(7000, int(os.getenv("FLASK_PORT", os.getenv("PORT", 7000))))
    try:
        # Connect MQTT client
        if MQTT_BROKER:
            try:
                print(f"[MQTT] Connecting to broker at {MQTT_BROKER}:{MQTT_PORT}...")
                mqtt_client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
                mqtt_client.loop_start()
            except Exception as e:
                print(f"[ERROR] MQTT connection to {MQTT_BROKER}:{MQTT_PORT} failed: {e}")
        else:
            print("[MQTT] MQTT_BROKER not configured. MQTT features disabled.")
        # Run Flask app without SSL on UNO Q (SSL can be added if certs are installed)
        print(f"[SYSTEM] Starting Flask web server on http://0.0.0.0:{flask_port}...")
        app.run(host="0.0.0.0", port=flask_port, threaded=True, debug=False)
    except KeyboardInterrupt:
        print("\n[SYSTEM] Shutdown requested by user (KeyboardInterrupt).")
    except Exception as e:
        print(f"[ERROR] Unhandled exception in main application: {e}")
        traceback.print_exc()
    finally:
        print("[SYSTEM] Initiating shutdown sequence...")
        try:
            if mqtt_client.is_connected():
                print("[MQTT] Disconnecting MQTT client...")
                mqtt_client.loop_stop()
                mqtt_client.disconnect()
                print("[MQTT] MQTT client disconnected.")
            else:
                mqtt_client.loop_stop(force=True)
        except Exception:
            pass
        print("[SYSTEM] MeasurePi dashboard has shut down.")


if __name__ == "__main__":
    run_application()