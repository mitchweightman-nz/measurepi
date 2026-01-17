#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bridge_mqtt.py – Python bridge for the UNO Q measure rig

This script runs on the Linux side of an Arduino UNO Q board and
coordinates communication between the microcontroller sketch and the
local MQTT process. It listens for raw measurement data from the MCU
via the Router Bridge RPC interface, computes box dimensions from the
raw distances, and forwards JSON payloads to the MQTT process over a
local IPC socket. It also listens for capture commands over a local
command socket and forwards requests back to the MCU.

Environment variables control the IPC endpoints and reference distances
used for calculating the box size:

• MQTT_IPC_HOST / MQTT_IPC_PORT – host/port for sending payloads to the MQTT process
• BRIDGE_CMD_HOST / BRIDGE_CMD_PORT – host/port for receiving command requests
• REF_LENGTH_CM, REF_HEIGHT_CM, REF_WIDTH_CM – reference distances, in centimetres

Version: Ver-2601172142
"""

import html
import json
import math
import os
import socket
import socketserver
import threading
import time
from typing import Optional

from arduino.app_utils import Bridge, App


# ─── Configuration ─────────────────────────────────────────

_IPC_READY = False


def _sanitize_log(msg: str) -> str:
    """Sanitize a log message before printing/publishing."""
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
    msg = _sanitize_log(str(msg))
    print(msg)
    if _IPC_READY:
        _send_ipc_message({"type": "log", "message": msg})


def _get_int_env(name: str, default: int) -> int:
    raw = os.getenv(name)
    if raw is None:
        return default
    try:
        return int(raw)
    except (TypeError, ValueError):
        _log(f"[CONFIG] Invalid value for {name}='{raw}', using default {default}.")
        return default


def _get_float_env(name: str, default: float) -> float:
    raw = os.getenv(name)
    if raw is None:
        return default
    try:
        return float(raw)
    except (TypeError, ValueError):
        _log(f"[CONFIG] Invalid value for {name}='{raw}', using default {default}.")
        return default


MQTT_IPC_HOST = os.getenv("MQTT_IPC_HOST", "127.0.0.1")
MQTT_IPC_PORT = _get_int_env("MQTT_IPC_PORT", 8765)
BRIDGE_CMD_HOST = os.getenv("BRIDGE_CMD_HOST", "127.0.0.1")
BRIDGE_CMD_PORT = _get_int_env("BRIDGE_CMD_PORT", 8766)

REF_LENGTH_CM = _get_float_env("REF_LENGTH_CM", 80.0)
REF_HEIGHT_CM = _get_float_env("REF_HEIGHT_CM", 89.0)
REF_WIDTH_CM = _get_float_env("REF_WIDTH_CM", 70.0)

bridge = Bridge()


def linux_started() -> bool:
    return True


def mcu_ready() -> None:
    _log("[BRIDGE] MCU signalled it is ready.")


def _safe_float(value) -> Optional[float]:
    try:
        if value is None:
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


def _box_dim_from_raw(ref_cm: float, raw: Optional[float]) -> Optional[float]:
    if raw is None or not math.isfinite(raw):
        return None
    box = ref_cm - raw
    return max(box, 0.0)


def measurement_data(height_raw, width_raw, length_raw) -> None:
    h_raw = _safe_float(height_raw)
    w_raw = _safe_float(width_raw)
    l_raw = _safe_float(length_raw)

    h_box = _box_dim_from_raw(REF_HEIGHT_CM, h_raw)
    w_box = _box_dim_from_raw(REF_WIDTH_CM, w_raw)
    l_box = _box_dim_from_raw(REF_LENGTH_CM, l_raw)

    dims = [d for d in (h_box, w_box, l_box) if isinstance(d, float)]
    dimension_max = max(dims) if dims else None

    payload = {
        "height": h_box,
        "width": w_box,
        "length": l_box,
        "dimension": dimension_max,
        "height_raw": h_raw,
        "width_raw": w_raw,
        "length_raw": l_raw,
        "height_box": h_box,
        "width_box": w_box,
        "length_box": l_box,
    }

    _send_ipc_message({"type": "measurement", "payload": payload})


bridge.provide("linux_started", linux_started)
bridge.provide("mcu_ready", mcu_ready)
bridge.provide("measurement_data", measurement_data)

def _send_ipc_message(message: dict) -> None:
    try:
        data = json.dumps(message, separators=(",", ":")).encode("utf-8") + b"\n"
        with socket.create_connection((MQTT_IPC_HOST, MQTT_IPC_PORT), timeout=1.5) as sock:
            sock.sendall(data)
    except Exception:
        pass


_IPC_READY = True


class _BridgeCommandHandler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        for line in self.rfile:
            payload = line.decode("utf-8", errors="replace").strip()
            if not payload:
                continue
            try:
                message = json.loads(payload)
            except json.JSONDecodeError:
                _log(f"[BRIDGE] Invalid command payload: {payload!r}")
                continue
            command = message.get("command")
            if command == "capture":
                _log("[BRIDGE] Capture command received via IPC. Invoking MCU 'capture' function…")
                try:
                    bridge.call("capture")
                except Exception as exc:
                    _log(f"[BRIDGE] Error invoking 'capture': {exc}")
            else:
                _log(f"[BRIDGE] Unknown command received via IPC: {command!r}")


def _start_command_server() -> socketserver.TCPServer:
    server = socketserver.ThreadingTCPServer(
        (BRIDGE_CMD_HOST, BRIDGE_CMD_PORT),
        _BridgeCommandHandler,
    )
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    _log(f"[BRIDGE] Command server listening on {BRIDGE_CMD_HOST}:{BRIDGE_CMD_PORT}.")
    return server


def _user_loop() -> None:
    time.sleep(0.1)


def main() -> None:
    _log("[SYSTEM] UNO Q MQTT bridge starting…")
    command_server = _start_command_server()

    try:
        App.run(user_loop=_user_loop)
    finally:
        command_server.shutdown()
        command_server.server_close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        _log("[SYSTEM] Shutdown requested by user (Ctrl-C).")
    except Exception as exc:
        _log(f"[ERROR] Unhandled exception: {exc}")
    finally:
        _log("[SYSTEM] Bridge script terminated.")
