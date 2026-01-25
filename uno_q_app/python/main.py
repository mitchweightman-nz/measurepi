#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py — UNO Q app entrypoint

Starts the MQTT server, dashboard, and bridge in a single process so the
Arduino App CLI can manage one service.
"""

import os
import threading
import traceback

import bridge
import dashboard
import mqtt


def _start_background(name, target, stop_event):
    def _runner():
        try:
            target(stop_event)
        except Exception:
            print(f"[SYSTEM] Background worker {name} terminated unexpectedly.")
            traceback.print_exc()
            stop_event.set()
            os._exit(1)

    thread = threading.Thread(target=_runner, name=name)
    thread.start()
    return thread


def main() -> None:
    stop_event = threading.Event()
    mqtt_thread = _start_background("mqtt", mqtt.main, stop_event)
    dashboard_thread = _start_background("dashboard", dashboard.main, stop_event)
    try:
        bridge.main(stop_event)
    finally:
        stop_event.set()
        mqtt_thread.join()
        dashboard_thread.join()


if __name__ == "__main__":
    main()
