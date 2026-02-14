#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import html
import json
import os
import socket
import socketserver
import threading
import time
from typing import Optional

import paho.mqtt.client as mqtt


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


def _get_int_env(name: str, default: int) -> int:
    raw = os.getenv(name)
    if raw is None:
        return default
    try:
        return int(raw)
    except Exception:
        return default


DEFAULT_MQTT_BROKER = "127.0.0.1"
DEFAULT_MQTT_PORT = 1883

MQTT_BROKER = os.getenv("MQTT_BROKER", DEFAULT_MQTT_BROKER)
MQTT_PORT = _get_int_env("MQTT_PORT", DEFAULT_MQTT_PORT)

MQTT_CMD_TOPIC = "measure/cmd"
MQTT_DATA_TOPIC = "measure/data"
MQTT_LOG_TOPIC = "measure/log"

MQTT_IPC_BIND_HOST = "0.0.0.0"
MQTT_IPC_PORT = 8765

BRIDGE_CMD_HOST = "127.0.0.1"
BRIDGE_CMD_PORT = 8766

mqtt_client = mqtt.Client(protocol=mqtt.MQTTv311)


def _publish(topic: str, payload: str) -> None:
    mqtt_client.publish(topic, payload, qos=0, retain=False)


def _send_bridge_command(command: str) -> None:
    payload = json.dumps({"command": command}).encode() + b"\n"
    with socket.create_connection((BRIDGE_CMD_HOST, BRIDGE_CMD_PORT), timeout=1.5) as sock:
        sock.sendall(payload)


def _on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        _log(f"[MQTT] Connected to {MQTT_BROKER}:{MQTT_PORT}")
        client.subscribe(MQTT_CMD_TOPIC)
    else:
        _log(f"[MQTT] Connection failed: {rc}")


def _on_message(client, userdata, msg):
    payload = msg.payload.decode(errors="replace").strip()
    _log(f"[MQTT] {msg.topic}: {payload}")
    if msg.topic == MQTT_CMD_TOPIC and payload.lower().startswith("cap"):
        _send_bridge_command("capture")


mqtt_client.on_connect = _on_connect
mqtt_client.on_message = _on_message


class _BridgePayloadHandler(socketserver.StreamRequestHandler):
    def handle(self):
        for line in self.rfile:
            payload = line.decode(errors="replace").strip()
            if not payload:
                continue
            try:
                message = json.loads(payload)
            except Exception:
                continue

            if message.get("type") == "measurement":
                _publish(MQTT_DATA_TOPIC, json.dumps(message.get("payload", {})))
            elif message.get("type") == "log":
                _publish(MQTT_LOG_TOPIC, message.get("message", ""))


def _start_ipc_server():
    server = socketserver.ThreadingTCPServer(
        (MQTT_IPC_BIND_HOST, MQTT_IPC_PORT),
        _BridgePayloadHandler,
    )
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    _log(f"[IPC] Listening on {MQTT_IPC_BIND_HOST}:{MQTT_IPC_PORT}")
    return server


def main(stop_event: Optional[threading.Event] = None):
    if stop_event is None:
        stop_event = threading.Event()

    mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
    mqtt_client.loop_start()

    ipc_server = _start_ipc_server()

    try:
        while not stop_event.is_set():
            time.sleep(1)
    finally:
        ipc_server.shutdown()
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
