# Project Agent Guide

This guide applies to the entire `measurepi` repository unless a more specific `AGENTS.md` appears in a subdirectory.

## Expectations

- Keep changes focused and clearly scoped.
- Use concise commit messages that summarize what changed and why.
- Prefer clear, self-documenting code; add brief comments when logic is non-obvious.
- Do **not** wrap import statements in `try/except` blocks.

## Repository Overview

- `measure_uno_q.ino` runs on the UNO Q MCU and drives the sensors/laser/state machine.
- `bridge_mqtt.py` runs on the UNO Q Linux side, exposes RPC handlers (`linux_started`, `mcu_ready`, `measurement_data`), and publishes MQTT payloads.
- `measurepi_dashboard.py` + `templates/index.html` provide the Flask dashboard and JSON APIs.
- The `legacy/` directory contains older Raspberry Pi / UNO R4 WiFi assets.

## Runtime Requirements

- Python 3 with `paho-mqtt` and `Flask`.
- The UNO Q `arduino.app_utils` library for Router Bridge RPC on the Linux side.
- Keep README configuration tables aligned with the environment variables used in `bridge_mqtt.py` and `measurepi_dashboard.py`.

## Python and JavaScript Style

- Favor consistent formatting (Black-compatible Python, Prettier-friendly JavaScript).
- Use descriptive names for variables and functions.
- Keep functions small and handle edge cases explicitly.
- Validate user or external input before use.

## Testing

- Run relevant tests or linters when modifying executable code.
- Record the commands and results in the change summary.
- If no automated tests exist, perform a quick manual check and describe what you verified.

## Documentation

- Update README or inline comments when behavior or usage changes.
- Keep instructions current so future contributors can follow them easily.
