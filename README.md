# MeasurePi UNO Q Measurement Rig

MeasurePi is a self-contained application for the **Arduino UNO Q** that measures parcel dimensions using three TF-Luna time-of-flight sensors. It includes a lightweight web dashboard and publishes measurements over MQTT. The project is split into two components:

1. **Microcontroller sketch (`measure_uno_q.ino`)**
   * Runs on the STM32U585 microcontroller.
   * Handles timing, laser control, sensor sampling, and the state machine.
   * Communicates with the Linux side of the UNO Q via the Router Bridge RPC interface.
   * The same sketch is mirrored in `uno_q_app/sketch/sketch.ino` so it is built as part of the UNO Q app.

2. **Python application**
   * Runs on the embedded Debian system (MPU).
   * Publishes measurements to an MQTT broker using `paho-mqtt`.
   * Serves a Flask dashboard for live readings.
   * Entry point: `uno_q_app/python/main.py` (a copy of `bridge_mqtt.py`).

## Prerequisites

* Arduino UNO Q running the default Debian image.
* A measurement rig wired with:
  * Three TF-Luna I²C sensors with unique addresses (0x10 height, 0x20 width, 0x30 length).
  * An eight-pixel NeoPixel strip on pin 7.
  * A laser driver on pin 10.
* Access to an MQTT broker (local or remote).
* Network access on the UNO Q to install packages via `apt`.
* **Arduino App CLI** (`arduino-app-cli`).

## Deployment on UNO Q

The repository includes a helper script, `deploy_uno_q.sh`, that automates installation and deployment.

1. **Clone this repository** onto the UNO Q.
   ```bash
   sudo apt update
   sudo apt install -y git
   git clone https://github.com/your-org/measurepi.git
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
| `MQTT_HOST` | `localhost` | MQTT broker hostname. |
| `MQTT_PORT` | `1883` | MQTT broker port. |
| `MQTT_USER` | (unset) | MQTT username. |
| `MQTT_PASS` | (unset) | MQTT password. |
| `MQTT_TOPIC` | `measurepi` | Topic prefix for published measurements. |
| `REF_LENGTH_CM` | `80.0` | Reference distance for the length sensor. |
| `REF_HEIGHT_CM` | `89.0` | Reference distance for the height sensor. |
| `REF_WIDTH_CM` | `70.0` | Reference distance for the width sensor. |
| `DASHBOARD_PORT` | `5000` | Flask dashboard port. |
| `ALLOWED_ORIGINS` | `*` | Comma-separated list of allowed dashboard origins. |

Example:
```bash
export MQTT_HOST=192.168.1.10
export MQTT_USER=myuser
export MQTT_PASS=secret
arduino-app-cli app stop measurepi
arduino-app-cli app start measurepi
```

## UNO Q App Structure

Arduino App Lab expects a specific directory layout. This repository provides a ready-made `uno_q_app` directory:

```
uno_q_app/
├── app.yaml              # App metadata and default environment variables
├── python/
│   ├── main.py           # Entry point (MQTT bridge)
│   ├── measurepi_dashboard.py
│   └── requirements.txt
└── sketch/
    ├── sketch.ino        # MCU sketch (copied from measure_uno_q.ino)
    └── sketch.yaml       # FQBN and library dependencies
```

The `sketch.yaml` targets `arduino:zephyr:unoq` and includes the required libraries (TFLI2C, Adafruit NeoPixel, Arduino RouterBridge).

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

## Contributing

Pull requests and bug reports are welcome. Please test both the MCU sketch and the Python app before submitting changes.
