#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mqtt_server.py – MQTT process for the UNO Q measure rig

Runs as a standalone process that connects to the MQTT broker, publishes
measurement/log payloads received over a local IPC socket, and forwards
capture commands back to the bridge process.
"""

import html
import json
import os
import socket
import socketserver
import threading
import time

import paho.mqtt.client as mqtt


# ─── Logging ───────────────────────────────────────────────

def _sanitize_log(msg: str) -> str:
    msg_str = str(msg)
    escaped = html.escape(msg_str)
    sanitized_chars = []
    for ch in escaped:
        if ch in "\n\r" or not ch.isprintable():
            sanitized_chars.append(" ")
        else:
            sanitized_chars.append(ch)
    return "".join(sanitized_chars)


def _log(msg: str) -> None:
    print(_sanitize_log(msg))


# ─── Configuration ─────────────────────────────────────────

def _get_int_env(name: str, default: int) -> int:
    raw = os.getenv(name)
    if raw is None:
        return default
    try:
        return int(raw)
    except (TypeError, ValueError):
        _log(f"[CONFIG] Invalid value for {name}='{raw}', using default {default}.")
        return default


DEFAULT_MQTT_BROKER = "10.1.1.85"
DEFAULT_MQTT_PORT = 1883

MQTT_BROKER = os.getenv("MQTT_BROKER", DEFAULT_MQTT_BROKER)
MQTT_PORT = _get_int_env("MQTT_PORT", DEFAULT_MQTT_PORT)
MQTT_USER = os.getenv("MQTT_USER")
MQTT_PASS = os.getenv("MQTT_PASS")

MQTT_CMD_TOPIC = os.getenv("MQTT_CMD_TOPIC", "measure/cmd")
MQTT_DATA_TOPIC = os.getenv("MQTT_DATA_TOPIC", "measure/data")
MQTT_LOG_TOPIC = os.getenv("MQTT_LOG_TOPIC", "measure/log")
MQTT_CLIENT_ID = os.getenv("MQTT_CLIENT_ID", "uno-q-bridge")

MQTT_IPC_HOST = os.getenv("MQTT_IPC_HOST", "127.0.0.1")
MQTT_IPC_PORT = _get_int_env("MQTT_IPC_PORT", 8765)
BRIDGE_CMD_HOST = os.getenv("BRIDGE_CMD_HOST", "127.0.0.1")
BRIDGE_CMD_PORT = _get_int_env("BRIDGE_CMD_PORT", 8766)

mqtt_client = mqtt.Client(client_id=MQTT_CLIENT_ID, protocol=mqtt.MQTTv311)

if MQTT_USER:
    if MQTT_PASS is not None:
        mqtt_client.username_pw_set(MQTT_USER, MQTT_PASS)
    else:
        mqtt_client.username_pw_set(MQTT_USER)


# ─── MQTT Handling ─────────────────────────────────────────

def _publish(topic: str, payload: str) -> None:
    try:
        mqtt_client.publish(topic, payload, qos=0, retain=False)
    except Exception as exc:
        _log(f"[MQTT] Failed to publish to {topic}: {exc}")


def _send_bridge_command(command: str) -> None:
    payload = json.dumps({"command": command}, separators=(",", ":")).encode("utf-8") + b"\n"
    try:
        with socket.create_connection((BRIDGE_CMD_HOST, BRIDGE_CMD_PORT), timeout=1.5) as sock:
            sock.sendall(payload)
        _log(f"[BRIDGE] Sent command '{command}' to bridge.")
    except Exception as exc:
        _log(f"[BRIDGE] Failed to send command '{command}' to bridge: {exc}")


def _on_mqtt_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        _log(f"[MQTT] Connected to {MQTT_BROKER}:{MQTT_PORT} as {MQTT_CLIENT_ID}.")
        client.subscribe(MQTT_CMD_TOPIC)
        _log(f"[MQTT] Subscribed to command topic '{MQTT_CMD_TOPIC}'.")
    else:
        _log(f"[MQTT] Connection failed with code {rc}.")


def _on_mqtt_disconnect(client, userdata, rc, properties=None):
    if rc != 0:
        _log(f"[MQTT] Unexpected disconnect (rc={rc}). Will retry automatically.")


def _on_mqtt_message(client, userdata, msg):
    try:
        payload = msg.payload.decode("utf-8", errors="replace").strip()
    except Exception:
        payload = repr(msg.payload)
    _log(f"[MQTT] Received message on '{msg.topic}': {payload}")
    if msg.topic != MQTT_CMD_TOPIC:
        return
    if payload.lower().startswith("cap"):
        _send_bridge_command("capture")


mqtt_client.on_connect = _on_mqtt_connect
mqtt_client.on_disconnect = _on_mqtt_disconnect
mqtt_client.on_message = _on_mqtt_message


# ─── IPC Server ────────────────────────────────────────────

class _BridgePayloadHandler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        for line in self.rfile:
            payload = line.decode("utf-8", errors="replace").strip()
            if not payload:
                continue
            try:
                message = json.loads(payload)
            except json.JSONDecodeError:
                _log(f"[IPC] Invalid JSON payload: {payload!r}")
                continue
            msg_type = message.get("type")
            if msg_type == "measurement":
                payload_data = message.get("payload", {})
                _publish(MQTT_DATA_TOPIC, json.dumps(payload_data))
                _log("[MQTT] Published measurement payload.")
            elif msg_type == "log":
                log_message = message.get("message", "")
                _publish(MQTT_LOG_TOPIC, str(log_message))
            else:
                _log(f"[IPC] Unknown payload type: {msg_type!r}")


def _start_ipc_server() -> socketserver.TCPServer:
    server = socketserver.ThreadingTCPServer(
        (MQTT_IPC_HOST, MQTT_IPC_PORT),
        _BridgePayloadHandler,
    )
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    _log(f"[IPC] Listening for bridge payloads on {MQTT_IPC_HOST}:{MQTT_IPC_PORT}.")
    return server


def _start_mqtt() -> None:
    try:
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
        mqtt_client.loop_start()
    except Exception as exc:
        _log(f"[MQTT] Failed to connect to broker at {MQTT_BROKER}:{MQTT_PORT}: {exc}")


# ─── Main ──────────────────────────────────────────────────

def main() -> None:
    _log("[SYSTEM] UNO Q MQTT server starting…")
    _start_mqtt()
    ipc_server = _start_ipc_server()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        _log("[SYSTEM] Shutdown requested by user (Ctrl-C).")
    finally:
        ipc_server.shutdown()
        ipc_server.server_close()
        try:
            mqtt_client.loop_stop()
            mqtt_client.disconnect()
        except Exception:
            pass
        _log("[SYSTEM] MQTT server terminated.")


if __name__ == "__main__":
    main()
