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
import importlib.util
import json
import math
import os
import re
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
import serial

# ─── Flask web service ───────────────────────────────────────────────────────
import urllib.parse

from flask import Flask, jsonify, make_response, render_template, request
# ─── Configuration Constants ─────────────────────────────────────────────────
MQTT_BROKER = os.getenv("MQTT_BROKER", "10.1.1.85")
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))
MQTT_TOPIC_SUB = "measure/data"
MQTT_TOPIC_LOG = "measure/log"
MQTT_COMMAND_TOPIC = "measure/cmd"
MQTT_CAPTURE_TOPIC = "measure/cmd"

MAX_RAW_HISTORY = 200

SSL_CERT_PATH_ENV = "SSL_CERT_PATH"
SSL_KEY_PATH_ENV = "SSL_KEY_PATH"
DEFAULT_SSL_CERT_PATH = str(Path.home() / "measure_pi" / "cert.pem")
DEFAULT_SSL_KEY_PATH = str(Path.home() / "measure_pi" / "private.pem")
CORS_ALLOWED_ORIGINS_ENV = "CORS_ALLOWED_ORIGINS"
DEFAULT_EXTRA_ALLOWED_ORIGINS = {"https://nzc.gosweetspot.com"}
x
DEFAULT_ROUNDING_SETTINGS = {
    "height": "ceil",
    "length": "ceil",
    "width": "ceil",
}

ENABLE_UPNP = os.getenv("ENABLE_UPNP", "true").lower() not in ("0", "false", "no")
UPNP_DISCOVERY_TIMEOUT_MS = int(os.getenv("UPNP_DISCOVERY_TIMEOUT_MS", 2000))
UPNP_LEASE_DURATION = int(os.getenv("UPNP_LEASE_DURATION", 0))

SERIAL_SCALE_PORT = os.getenv("SERIAL_SCALE_PORT", "/dev/ttyUSB0")
SERIAL_SCALE_BAUDRATE = int(os.getenv("SERIAL_SCALE_BAUDRATE", 9600))
SERIAL_SCALE_ENABLED = os.getenv("ENABLE_SERIAL_SCALE", "true").lower() not in (
    "0",
    "false",
    "no",
)
SERIAL_SCALE_RECONNECT_DELAY = 3

# ─── Global State Variables ──────────────────────────────────────────────────
lcd = None  # I2C Character LCD object (initialized in _init_lcd_if_present)

# Shared state for measurements, accessed by MQTT callback and Flask routes
current_measurement = {}  
measurement_history = []
raw_mqtt_history = []
measure_log_history = []
_data_lock = threading.Lock()
_last_lcd_text = ""
_serial_scale_thread = None

rounding_settings = DEFAULT_ROUNDING_SETTINGS.copy()

mqtt_client = mqtt.Client(client_id=f"measure_pi_client_{os.getpid()}", protocol=mqtt.MQTTv311)

# Track whether the external scale is available and if manual weight input
# is pending. These values are updated as MQTT payloads arrive and when
# manual weight overrides are submitted via the API.
scale_present_flag = None  # None = unknown, True = scale detected, False = no scale
awaiting_manual_weight = False
pending_manual_weight_value = None


# ─── MQTT Callbacks ──────────────────────────────────────────────────────────
def _on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print(f"[MQTT] Connected successfully to broker {MQTT_BROKER}.")
        client.subscribe(MQTT_TOPIC_SUB)
        client.subscribe(MQTT_TOPIC_LOG)
        print(f"[MQTT] Subscribed to topic: {MQTT_TOPIC_SUB}")
        print(f"[MQTT] Subscribed to topic: {MQTT_TOPIC_LOG}")
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


def _initialize_upnp_client():
    if not ENABLE_UPNP:
        print("[UPNP] UPnP configuration disabled via ENABLE_UPNP.")
        return None

    if importlib.util.find_spec("miniupnpc") is None:
        print("[UPNP] miniupnpc not installed. Skipping UPnP configuration.")
        return None

    import miniupnpc

    client = miniupnpc.UPnP()
    client.discoverdelay = max(100, UPNP_DISCOVERY_TIMEOUT_MS)

    try:
        discovered = client.discover()
    except Exception as exc:
        print(f"[UPNP] Discovery failed: {exc}")
        return None

    if discovered == 0:
        print("[UPNP] No UPnP-enabled gateway discovered.")
        return None

    try:
        client.selectigd()
    except Exception as exc:
        print(f"[UPNP] Failed to select Internet Gateway Device: {exc}")
        return None

    print(f"[UPNP] Selected gateway {client.urlbase} (LAN: {client.lanaddr})")

    try:
        external_ip = client.externalipaddress()
        print(f"[UPNP] External IP address: {external_ip}")
    except Exception as exc:
        print(f"[UPNP] Unable to retrieve external IP address: {exc}")

    return client


def _ensure_upnp_mapping(client, external_port, internal_port, protocol, description):
    existing_mapping = client.getspecificportmapping(external_port, protocol)
    if existing_mapping:
        (
            mapped_internal_client,
            mapped_internal_port,
            mapped_protocol,
            mapped_description,
            mapped_enabled,
            mapped_lease_duration,
        ) = existing_mapping

        if mapped_internal_client == client.lanaddr and mapped_internal_port == internal_port:
            print(
                f"[UPNP] Port {external_port}/{protocol} already mapped to {mapped_internal_client}:{mapped_internal_port}."
            )
            return True

        print(
            "[UPNP] Port mapping conflict for "
            f"{external_port}/{protocol}: existing mapping points to "
            f"{mapped_internal_client}:{mapped_internal_port} ({mapped_description}), "
            f"desired {client.lanaddr}:{internal_port}."
        )
        return False

    try:
        client.addportmapping(
            external_port,
            protocol,
            client.lanaddr,
            internal_port,
            description,
            "",
            UPNP_LEASE_DURATION,
        )
        print(
            f"[UPNP] Mapped {protocol} port {external_port} -> {client.lanaddr}:{internal_port} ({description})."
        )
        return True
    except Exception as exc:
        print(
            f"[UPNP] Failed to map {protocol} port {external_port} -> {client.lanaddr}:{internal_port}: {exc}"
        )
        return False


def _configure_upnp_port_mappings(flask_port):
    client = _initialize_upnp_client()
    if not client:
        return

    mappings = [
        (MQTT_PORT, MQTT_PORT, "TCP", "MeasurePi MQTT"),
        (flask_port, flask_port, "TCP", "MeasurePi Flask"),
    ]

    for external_port, internal_port, protocol, description in mappings:
        _ensure_upnp_mapping(client, external_port, internal_port, protocol, description)


def _on_message(client, userdata, msg):
    global _last_lcd_text, scale_present_flag, pending_manual_weight_value, awaiting_manual_weight
    # print(f"[MQTT] Received message on topic '{msg.topic}': {msg.payload.decode()}")

    if msg.topic == MQTT_TOPIC_SUB:
        try:
            payload_str = msg.payload.decode('utf-8')
            data = json.loads(payload_str)

            timestamp = time.time()

            def pick_dimension_value(key_candidates):
                for candidate in key_candidates:
                    value = _safe_float(data.get(candidate))
                    if value is not None:
                        return value
                return None

            # Accept either explicit h/w/l keys, box_* fallbacks, or a single
            # "dimension" value (replicated across all three axes).
            single_dimension = pick_dimension_value(["dimension", "dimension_cm", "dim"])
            new_measurements = {
                "height": pick_dimension_value(["height", "height_box", "height_cm"]) or single_dimension,
                "width": pick_dimension_value(["width", "width_box", "width_cm"]) or single_dimension,
                "length": pick_dimension_value(["length", "length_box", "length_cm"]) or single_dimension,
                "timestamp": timestamp,
            }

            if all(new_measurements.get(axis) is None for axis in ("height", "width", "length")):
                print(f"[MQTT] Warning: Received data missing dimension values. Data: {data}")
                return

            # Optional fields published by the UNO firmware
            new_measurements["weight"] = _safe_float(data.get("weight"))
            new_measurements["weight_net"] = _safe_float(data.get("weight_net"))
            new_measurements["weight_gross"] = _safe_float(data.get("weight_gross"))
            new_measurements["tare_g"] = _safe_int(data.get("tare_g"))

            if new_measurements.get("weight") is None:
                gross_val = new_measurements.get("weight_gross")
                net_val = new_measurements.get("weight_net")
                if gross_val is not None:
                    new_measurements["weight"] = gross_val
                elif net_val is not None:
                    new_measurements["weight"] = net_val

            have_scale = any(
                new_measurements.get(key) is not None
                for key in ("weight", "weight_net", "weight_gross", "tare_g")
            )
            new_measurements["scale_present"] = have_scale

            if not have_scale:
                new_measurements["manual_weight_required"] = True
                manual_override = pending_manual_weight_value
                if isinstance(manual_override, (int, float)):
                    manual_value = float(manual_override)
                    new_measurements["weight"] = manual_value
                    new_measurements["weight_net"] = manual_value
                    new_measurements["weight_gross"] = manual_value
                    new_measurements["manual_weight"] = True
                    new_measurements["manual_weight_timestamp"] = time.time()
                    pending_manual_weight_value = None
                    awaiting_manual_weight = False
            else:
                new_measurements["manual_weight_required"] = False
                pending_manual_weight_value = None
                awaiting_manual_weight = False

            scale_present_flag = have_scale

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

    elif msg.topic == MQTT_TOPIC_LOG:
        try:
            payload_str = msg.payload.decode('utf-8', errors='replace')
        except Exception:
            payload_str = repr(msg.payload)

        entry = {
            "topic": msg.topic,
            "payload": payload_str,
            "timestamp": time.time(),
        }

        with _data_lock:
            measure_log_history.append(entry)
            if len(measure_log_history) > MAX_RAW_HISTORY:
                measure_log_history.pop(0)

    else:
        # Ignore unrelated topics but keep a trace for debugging
        try:
            payload_str = msg.payload.decode('utf-8', errors='replace')
        except Exception:
            payload_str = repr(msg.payload)
        print(f"[MQTT] Received message on unexpected topic '{msg.topic}': {payload_str}")

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
                rounded_measurements[key] = float(value)
            elif isinstance(rule, str) and rule.isdigit():
                precision = int(rule)
                rounded_measurements[key] = round(value, precision)
            else:
                rounded_measurements[key] = round(value, 1)
        except Exception as e:
            print(f"[ROUNDING] Error applying rule '{rule}' for key '{key}', value '{value}': {e}")
            rounded_measurements[key] = round(value, 1)

    return rounded_measurements


# ─── Serial Scale Integration ────────────────────────────────────────────────
def _parse_serial_scale_line(line: str):
    if not isinstance(line, str):
        return None

    match = re.search(r"([-+]?\d+(?:\.\d+)?)", line)
    if not match:
        return None

    return _safe_float(match.group(1))


def _apply_serial_scale_weight(weight_value, source_line=None):
    global scale_present_flag, pending_manual_weight_value, awaiting_manual_weight

    if not isinstance(weight_value, (int, float)):
        return

    weight_float = float(weight_value)
    timestamp = time.time()

    with _data_lock:
        pending_manual_weight_value = None
        awaiting_manual_weight = False
        scale_present_flag = True

        updated_measurement = current_measurement.copy() if current_measurement else {}
        updated_measurement.setdefault("timestamp", timestamp)

        updated_measurement.update(
            {
                "weight": weight_float,
                "weight_net": weight_float,
                "weight_gross": weight_float,
                "manual_weight": False,
                "manual_weight_required": False,
                "scale_present": True,
                "weight_timestamp": timestamp,
                "weight_source": "serial_scale",
            }
        )

        if SERIAL_SCALE_PORT:
            updated_measurement["serial_scale_port"] = SERIAL_SCALE_PORT

        current_measurement.clear()
        current_measurement.update(updated_measurement)

        measurement_history.append(updated_measurement.copy())
        if len(measurement_history) > MAX_RAW_HISTORY:
            measurement_history.pop(0)

    if source_line:
        print(f"[SERIAL-SCALE] Weight {weight_float} parsed from line: '{source_line}'")
    else:
        print(f"[SERIAL-SCALE] Weight {weight_float} received from serial scale.")


def _serial_scale_reader_loop():
    global scale_present_flag

    if not SERIAL_SCALE_PORT:
        print("[SERIAL-SCALE] SERIAL_SCALE_PORT is not configured; skipping serial scale reader.")
        return

    print(
        f"[SERIAL-SCALE] Starting reader on {SERIAL_SCALE_PORT} at {SERIAL_SCALE_BAUDRATE} baud..."
    )

    while True:
        try:
            with serial.Serial(SERIAL_SCALE_PORT, SERIAL_SCALE_BAUDRATE, timeout=1) as ser:
                print(
                    f"[SERIAL-SCALE] Connected to {SERIAL_SCALE_PORT}. Listening for weight data..."
                )
                while True:
                    line_bytes = ser.readline()
                    if not line_bytes:
                        continue

                    try:
                        decoded = line_bytes.decode("utf-8", errors="ignore").strip()
                    except Exception as e:
                        print(f"[SERIAL-SCALE] Failed to decode serial data: {e}")
                        continue

                    if not decoded:
                        continue

                    weight_val = _parse_serial_scale_line(decoded)
                    if weight_val is None:
                        continue

                    _apply_serial_scale_weight(weight_val, decoded)

        except serial.SerialException as e:
            print(
                f"[SERIAL-SCALE] Serial exception on {SERIAL_SCALE_PORT}: {e}. Reconnecting in {SERIAL_SCALE_RECONNECT_DELAY}s..."
            )
            with _data_lock:
                scale_present_flag = False
            time.sleep(SERIAL_SCALE_RECONNECT_DELAY)
        except Exception as e:
            print(
                f"[SERIAL-SCALE] Unexpected error while reading from {SERIAL_SCALE_PORT}: {e}. Reconnecting in {SERIAL_SCALE_RECONNECT_DELAY}s..."
            )
            traceback.print_exc()
            with _data_lock:
                scale_present_flag = False
            time.sleep(SERIAL_SCALE_RECONNECT_DELAY)


def _start_serial_scale_reader_thread():
    global _serial_scale_thread

    if not SERIAL_SCALE_ENABLED:
        print("[SERIAL-SCALE] Serial scale reader disabled via ENABLE_SERIAL_SCALE flag.")
        return

    if _serial_scale_thread and _serial_scale_thread.is_alive():
        return

    _serial_scale_thread = threading.Thread(
        target=_serial_scale_reader_loop, name="SerialScaleReader", daemon=True
    )
    _serial_scale_thread.start()


# ─── Flask Web Application Routes ───────────────────────────────────────────
app = Flask(__name__)


def _extract_origin_from_referer(referer_value):
    if not referer_value:
        return None

    parsed_referer = urllib.parse.urlparse(referer_value)
    if not parsed_referer.scheme or not parsed_referer.netloc:
        return None

    return f"{parsed_referer.scheme}://{parsed_referer.netloc}"


def _parse_allowed_origins():
    """
    Return a set of allowed origins (scheme + host [+ port]) configured via
    the `CORS_ALLOWED_ORIGINS` environment variable.

    The variable accepts a comma-separated list, for example:
    "https://nzc.gosweetspot.com,https://nzc.redlite.nz:9001"

    Empty values are ignored and the result is normalized to lowercase for
    comparisons.
    """

    raw_value = os.getenv(CORS_ALLOWED_ORIGINS_ENV, "")
    allowed = set(DEFAULT_EXTRA_ALLOWED_ORIGINS)

    for item in raw_value.split(","):
        trimmed = item.strip()
        if trimmed:
            allowed.add(trimmed.lower())

    return allowed


_ALLOWED_ORIGINS = _parse_allowed_origins()


def _is_origin_allowed(origin):
    """
    Determine whether the given origin should be echoed in CORS headers.

    If no explicit allow-list is configured, all origins are permitted (the
    previous default behavior). When a list is provided, the comparison is
    case-insensitive and expects an exact scheme/host[/port] match.
    """

    if not origin:
        return False

    if not _ALLOWED_ORIGINS:
        return True

    return origin.lower() in _ALLOWED_ORIGINS


def _merge_vary(response, values):
    existing = response.headers.get("Vary", "")
    merged_values = []

    if existing:
        merged_values.extend([item.strip() for item in existing.split(",") if item.strip()])

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
    if request_method:
        response.headers["Access-Control-Allow-Methods"] = request_method
    else:
        response.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"

    # Avoid wildcards so browsers can send credentials. Reflect the request origin
    # (or fall back to Referer) and signal caches that responses may vary accordingly.
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
    if dimension_candidates:
        current_rounded["dimension"] = max(dimension_candidates)
    else:
        current_rounded["dimension"] = None

    if "tare_g" in current_data_copy:
        tare_value = current_data_copy.get("tare_g")
        current_rounded["tare_g"] = int(tare_value) if isinstance(tare_value, (int, float)) else None

    for meta_key in ["scale_present", "manual_weight_required", "manual_weight", "manual_weight_timestamp"]:
        if meta_key in current_data_copy:
            current_rounded[meta_key] = current_data_copy[meta_key]

    return jsonify({"current": current_rounded, "history": history_to_send})

@app.route("/api/raw")
def raw_mqtt_history_route():
    with _data_lock:
        raw_data_list = list(raw_mqtt_history)
        log_data_list = [entry.copy() for entry in measure_log_history]
    return jsonify({"raw_mqtt_payloads": raw_data_list, "log_messages": log_data_list})

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


@app.route("/api/capture", methods=["POST"])
def capture_api_route():
    global awaiting_manual_weight

    with _data_lock:
        current_scale_state = scale_present_flag
        if current_scale_state is False:
            awaiting_manual_weight = True
        else:
            awaiting_manual_weight = False

    if mqtt_client.is_connected():
        mqtt_client.publish(MQTT_CAPTURE_TOPIC, "CAP")
        print(f"[API-CAPTURE] Capture command published to MQTT topic '{MQTT_CAPTURE_TOPIC}'.")

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
    print("[SYSTEM] MeasurePi MQTT Client & Web Server is starting up...")

    flask_port = int(os.getenv("FLASK_PORT", os.getenv("PORT", 5000)))

    try:
        _init_lcd_if_present()

        _start_serial_scale_reader_thread()

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

        protocol = "https" if flask_ssl_context else "http"
        _configure_upnp_port_mappings(flask_port)
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
