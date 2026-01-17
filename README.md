# MeasurePi UNO Q Measurement Rig

MeasurePi is a self-contained application for the **Arduino UNO Q** that measures parcel dimensions using three TF-Luna time-of-flight sensors. It includes a lightweight web dashboard and publishes measurements over MQTT. The project is split into three components:

1. **Microcontroller sketch (`measure_uno_q.ino`)**
   * Runs on the STM32U585 microcontroller.
   * Handles timing, laser control, sensor sampling, and the state machine.
   * Communicates with the Linux side of the UNO Q via the Router Bridge RPC interface.

2. **MQTT bridge (`bridge_mqtt.py`)**
   * Runs on the embedded Debian system (MPU).
   * Publishes measurements to an MQTT broker using `paho-mqtt`.
   * Exposes RPC handlers (`linux_started`, `mcu_ready`, `measurement_data`) for the MCU.
   * Listens for capture commands on MQTT and forwards them to the MCU.

3. **Dashboard (`measurepi_dashboard.py` + `templates/index.html`)**
   * Runs a Flask dashboard for live readings.
   * Subscribes to measurement/log topics from the bridge.
   * Provides a small JSON API for dashboards and integrations.

## Prerequisites

* Arduino UNO Q running the default Debian image.
* A measurement rig wired with:
  * Three TF-Luna I²C sensors with unique addresses (0x10 height, 0x20 width, 0x30 length).
  * An eight-pixel NeoPixel strip on pin 7.
  * A laser driver on pin 10.
* Access to an MQTT broker (local or remote).
* Network access on the UNO Q to install packages via `apt`.
* Python 3 with `paho-mqtt`, `Flask`, and the UNO Q `arduino.app_utils` bridge library.

## Running on UNO Q

1. **Clone this repository** onto the UNO Q.
   ```bash
   sudo apt update
   sudo apt install -y git
   git clone https://github.com/mitchweightman-nz/measurepi
   cd measurepi
   ```

2. **Install Python dependencies** (if they are not already baked into your image).
   ```bash
   python3 -m pip install paho-mqtt Flask
   ```

3. **Start the bridge and dashboard**.
   ```bash
   python3 bridge_mqtt.py
   python3 measurepi_dashboard.py
   ```

## Configuration

The application reads environment variables so you can adjust settings without editing code. Set them in the systemd unit or inline before starting the scripts.

### Bridge (`bridge_mqtt.py`) variables

| Variable | Default | Purpose |
| --- | --- | --- |
| `MQTT_BROKER` | `10.1.1.85` | MQTT broker hostname or IP. |
| `MQTT_PORT` | `1883` | MQTT broker port. |
| `MQTT_USER` | (unset) | MQTT username. |
| `MQTT_PASS` | (unset) | MQTT password. |
| `MQTT_CMD_TOPIC` | `measure/cmd` | MQTT topic for capture commands. |
| `MQTT_DATA_TOPIC` | `measure/data` | MQTT topic for measurement payloads. |
| `MQTT_LOG_TOPIC` | `measure/log` | MQTT topic for bridge logs. |
| `MQTT_CLIENT_ID` | `uno-q-bridge` | MQTT client identifier. |
| `REF_LENGTH_CM` | `80.0` | Reference distance for the length sensor. |
| `REF_HEIGHT_CM` | `89.0` | Reference distance for the height sensor. |
| `REF_WIDTH_CM` | `70.0` | Reference distance for the width sensor. |

### Dashboard (`measurepi_dashboard.py`) variables

| Variable | Default | Purpose |
| --- | --- | --- |
| `MQTT_BROKER` | `localhost` | MQTT broker hostname or IP. |
| `MQTT_PORT` | `1883` | MQTT broker port. |
| `MQTT_DATA_TOPIC` | `measure/data` | MQTT topic for measurement payloads. |
| `MQTT_LOG_TOPIC` | `measure/log` | MQTT topic for log payloads. |
| `MQTT_CMD_TOPIC` | `measure/cmd` | MQTT topic for command payloads. |
| `FLASK_PORT` / `PORT` | `7000` | Dashboard HTTP port. |
| `CORS_ALLOWED_ORIGINS` | (unset) | Comma-separated list of allowed origins (adds to the default `https://nzc.gosweetspot.com`). |

Example:
```bash
export MQTT_BROKER=192.168.1.10
export MQTT_USER=myuser
export MQTT_PASS=secret
python3 bridge_mqtt.py
```

## Bridge & Dashboard Behavior

* The bridge registers RPC handlers for the MCU: `linux_started`, `mcu_ready`, and `measurement_data`.
* When the bridge receives a command payload that starts with `CAP` (case-insensitive), it calls the MCU `capture` function.
* The dashboard subscribes to `MQTT_DATA_TOPIC` and `MQTT_LOG_TOPIC` and exposes JSON APIs for integrations.

## Legacy Hardware Files

Older Raspberry Pi and UNO R4 WiFi assets have been moved into the `legacy/` directory:

* `legacy/raspberry_pi/` contains the previous Raspberry Pi dashboard, SSL renewal helper, and deployment artifacts.
* `legacy/uno_r4/` contains the UNO R4 WiFi sketches.

## Developing and Testing Locally

You can run the Python dashboard and MQTT bridge on a regular computer for testing:

```bash
python3 bridge_mqtt.py
python3 measurepi_dashboard.py
```

Set the environment variables above to match your test environment. When running off-board, you may need to stub the Router Bridge calls.

## Dashboard API

| Route | Method | Description |
| --- | --- | --- |
| `/` | GET | Dashboard HTML. |
| `/json` | GET | Current rounded measurement plus history. |
| `/api/raw` | GET | Raw MQTT payloads and log messages. |
| `/api/settings` | GET/POST | Read or update rounding rules for `height`, `width`, `length`. |
| `/api/lcd_text` | GET | 4-line LCD-style string. |
| `/api/measurements_current` | GET | Current measurement payload. |
| `/api/command` | POST | Publish a raw command to `MQTT_CMD_TOPIC`. |
| `/api/capture` | POST | Publish a capture command and report scale state. |
| `/api/manual_weight` | POST | Provide a manual weight in kilograms. |

## Contributing

Pull requests and bug reports are welcome. Please test both the MCU sketch and the Python app before submitting changes.
