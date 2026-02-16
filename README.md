# MeasurePi UNO Q Measurement Rig

MeasurePi is a self-contained application for the **Arduino UNO Q** that measures parcel dimensions using three TF-Luna time-of-flight sensors. It includes a lightweight web dashboard and publishes measurements over MQTT. The project is split into two components:

1. **Microcontroller sketch (`uno_q_app/sketch/sketch.ino`)**
   * Runs on the STM32U585 microcontroller.
   * Handles timing, laser control, sensor sampling, and the state machine.
   * Communicates with the Linux side of the UNO Q via the Router Bridge RPC interface.
   * The sketch lives in `uno_q_app/sketch/sketch.ino` so it is built as part of the UNO Q app.

2. **Python application**
   * Runs on the embedded Debian system (MPU).
   * Publishes measurements to an MQTT broker using `paho-mqtt`.
   * Serves a Flask dashboard for live readings.
   * Entry points: `python/main.py` (app supervisor), `python/bridge.py` (UNO Q bridge), and `python/mqtt.py` (MQTT process).

## Prerequisites

* Arduino UNO Q running the default Debian image.
* A measurement rig wired with:
  * Three TF-Luna I²C sensors with unique addresses (0x10 height, 0x20 width, 0x30 length).
  * An eight-pixel NeoPixel strip on pin 7.
  * A laser driver on pin 10.
* Access to an MQTT broker (local or remote).
* Network access on the UNO Q to install packages via `apt`.
* **Arduino App CLI** (`arduino-app-cli`).

## TF-Luna → Arduino UNO Q Wiring (Outline)

Each TF-Luna sensor is on the same I²C bus with a unique address. Tie the power and I²C lines together, then route each sensor's RTS/DRDY line to its own GPIO so the sketch can monitor readiness.

| TF-Luna signal | Arduino UNO Q connection | Notes |
| --- | --- | --- |
| VCC | 5V | Power the sensor(s) from the UNO Q 5V rail. |
| GND | GND | Common ground shared across all sensors. |
| SDA | SDA (I²C data) | Shared across all sensors. |
| SCL | SCL (I²C clock) | Shared across all sensors. |
| RTS/DRDY (Length) | D4 | `PIN_RTS_1` in the sketch. |
| RTS/DRDY (Height) | D5 | `PIN_RTS_2` in the sketch. |
| RTS/DRDY (Width) | D6 | `PIN_RTS_3` in the sketch. |

Additional related wiring used by the measurement rig:

* NeoPixel strip DIN → D7. (NeoPixel VCC and GND to 5V/GND.)
* Laser driver control → D10.
* Capture button → D12 (active LOW to GND with internal pull-up).
* Live-mode toggle → A0 (active LOW to GND).
* Optional external I²C reset line → D11 (active LOW).

## Deployment on UNO Q

The repository includes a helper script, `deploy_uno_q.sh`, that automates installation and deployment. If the `uno_q_app` directory is missing, the script will generate the app bundle from the repository sources (sketch, bridge, dashboard, and templates).

1. **Clone this repository** onto the UNO Q.
   ```bash
   sudo apt update
   sudo apt install -y git
   git clone https://github.com/mitchweightman-nz/measurepi
   cd measurepi
   ```

2. **Run the deployment script**.
   ```bash
   chmod +x deploy_uno_q.sh
   ./deploy_uno_q.sh
   ```

The script will:

1. Update the package index and install Python and Git if missing.
2. Verify `arduino-app-cli` is available.
3. Copy `uno_q_app` into `~/ArduinoApps/measurepi`.
4. Install Python dependencies from `uno_q_app/python/requirements.txt`.
5. Build and start the app:
   ```bash
   arduino-app-cli app build measurepi
   arduino-app-cli app start measurepi
   ```

You can monitor logs with:
```bash
arduino-app-cli app logs measurepi
```

## Configuration

The application reads environment variables so you can adjust settings without editing code. Set them in the systemd unit or inline before starting the app.

| Variable | Default | Purpose |
| --- | --- | --- |
| `MQTT_BROKER` | `10.1.1.85` | MQTT broker hostname or IP (mqtt.py). |
| `MQTT_PORT` | `1883` | MQTT broker port (mqtt.py). |
| `MQTT_USER` | (unset) | MQTT username (mqtt.py). |
| `MQTT_PASS` | (unset) | MQTT password (mqtt.py). |
| `MQTT_CMD_TOPIC` | `measure/cmd` | Topic for capture commands (mqtt.py). |
| `MQTT_DATA_TOPIC` | `measure/data` | Topic for published measurement payloads (mqtt.py). |
| `MQTT_LOG_TOPIC` | `measure/log` | Topic for log messages (mqtt.py). |
| `MQTT_CLIENT_ID` | `uno-q-bridge` | MQTT client identifier (mqtt.py). |
| `MQTT_IPC_HOST` | `127.0.0.1` | Host for bridge → MQTT IPC (both processes). |
| `MQTT_IPC_PORT` | `8765` | Port for bridge → MQTT IPC (both processes). |
| `BRIDGE_CMD_HOST` | `127.0.0.1` | Host for MQTT → bridge command IPC (both processes). |
| `BRIDGE_CMD_PORT` | `8766` | Port for MQTT → bridge command IPC (both processes). |
| `MQTT_IPC_BIND_HOST` | `0.0.0.0` | Bind host for the MQTT IPC server (mqtt.py). |
| `BRIDGE_CMD_BIND_HOST` | `0.0.0.0` | Bind host for the bridge command server (bridge.py). |
| `REF_LENGTH_CM` | `80.0` | Reference distance for the length sensor (bridge.py). |
| `REF_HEIGHT_CM` | `89.0` | Reference distance for the height sensor (bridge.py). |
| `REF_WIDTH_CM` | `70.0` | Reference distance for the width sensor (bridge.py). |
| `DASHBOARD_PORT` | `5000` | Flask dashboard port. |
| `ALLOWED_ORIGINS` | `*` | Comma-separated list of allowed dashboard origins. |

Example:
```bash
export MQTT_BROKER=192.168.1.10
export MQTT_USER=myuser
export MQTT_PASS=secret
arduino-app-cli app stop measurepi
arduino-app-cli app start measurepi
```

## UNO Q App Structure

Arduino App Lab expects a specific directory layout. This repository provides a ready-made `uno_q_app` directory, and the deployment script can generate one if it is missing:

```
uno_q_app/
├── app.yaml              # App metadata and default environment variables
├── python/
│   ├── main.py           # Entry point (starts bridge, MQTT, dashboard)
│   ├── bridge.py
│   ├── dashboard.py
│   ├── mqtt.py
│   ├── requirements.txt
│   └── templates/
│       └── index.html
└── sketch/
    ├── sketch.ino        # MCU sketch
    └── sketch.yaml       # FQBN and library dependencies
```

The `sketch.yaml` targets `arduino:zephyr:unoq` and includes the required libraries (TFLI2C, Adafruit NeoPixel, Arduino RouterBridge).

## Legacy Hardware Files

Older Raspberry Pi and UNO R4 WiFi assets have been moved into the `legacy/` directory:

* `legacy/raspberry_pi/` contains the previous Raspberry Pi dashboard, SSL renewal helper, and deployment artifacts.
* `legacy/uno_r4/` contains the UNO R4 WiFi sketches.

## Developing and Testing Locally

You can run the Python dashboard and MQTT bridge on a regular computer for testing. The
bridge, MQTT server, and dashboard now run as separate processes, so start each script
in its own terminal session:

```bash
python3 uno_q_app/python/bridge.py
python3 uno_q_app/python/mqtt.py
python3 uno_q_app/python/dashboard.py
```

Set the environment variables above to match your test environment. When running off-board, you may need to stub the Router Bridge calls.

## Contributing

Pull requests and bug reports are welcome. Please test both the MCU sketch and the Python app before submitting changes.
