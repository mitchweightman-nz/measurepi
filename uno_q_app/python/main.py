#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py — UNO Q app entrypoint

Starts the MQTT server, dashboard, and bridge in a single process so the
Arduino App CLI can manage one service.
"""

import threading

import bridge
import dashboard
import mqtt


def _start_background(name, target):
    thread = threading.Thread(target=target, name=name, daemon=True)
    thread.start()
    return thread


def main() -> None:
    _start_background("mqtt", mqtt.main)
    _start_background("dashboard", dashboard.main)
    bridge.main()


if __name__ == "__main__":
    main()
