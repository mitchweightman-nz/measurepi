# MeasurePi UNO Q Measurement Rig

This repository contains a self‑contained application for the **Arduino UNO Q** board that measures parcel dimensions using three TF‑Luna time‑of‑flight sensors, provides a simple web dashboard and publishes the results over MQTT. The project is split into two components:

1. **Microcontroller sketch (`measure_uno_q.ino`)** – runs on the STM32U585 microcontroller and handles the low‑level timing, laser control, sensor multiplexing and state‑machine logic. The sketch communicates with the Linux side of the UNO Q via the Router Bridge RPC interface. The same code is included under `uno_q_app/sketch/sketch.ino` so that it is automatically built as part of the UNO Q app.
2. **Python application** – runs on the embedded Debian system (MPU) and uses the `paho‑mqtt` and `Flask` libraries to publish measurements to an MQTT broker and serve a dashboard. The entry point is `uno_q_app/python/main.py` (a copy of `bridge_mqtt.py`), which interacts with the microcontroller via `arduino.app_utils`.

The project can be deployed on a standard Raspberry Pi, but this document focuses on installing it on an Arduino UNO Q using Arduino App Lab.

## Prerequisites

* An Arduino UNO Q board running the default Debian image.
* A 3D‑printed measurement rig wired with:
  - Three TF‑Luna I²C sensors connected via a TCA9548A multiplexer.
  - An eight‑pixel NeoPixel strip on pin 7 for status indication.
  - A laser driver on pin 10.
* Access to an MQTT broker (locally or remotely). You may adjust broker settings via environment variables as described below.
* Network access on the UNO Q to run `apt` commands. Installing software on the UNO Q is done using the standard Debian package manager, e.g. `sudo apt install package-name`. In our deployment script we use:
  ```sh
  sudo apt update && sudo apt install python3 python3-pip git
  ```
  to ensure the Python runtime and Git are available.
* The **Arduino App CLI** (`arduino-app-cli`), which is pre‑installed on UNO Q images. It allows creating, building and managing apps directly on the device using commands such as `arduino-app-cli app new`, `arduino-app-cli app build` and `arduino-app-cli app start`.

## Deployment on UNO Q

The repository includes a helper script, `deploy_uno_q.sh`, that automates installation and deployment. Follow these steps on your UNO Q:

1. **Clone this repository** onto the UNO Q. You can use `git clone` if Git is installed. For example:
   ```sh
   sudo apt update && sudo apt install -y git
   git clone https://github.com/your-org/measurepi.git
   cd measurepi
   ```
2. **Run the deployment script**. Make it executable and run it from the repository root:
   ```sh
   chmod +x deploy_uno_q.sh
   ./deploy_uno_q.sh
   ```
   The script will:
   * Update the package index and install Python and Git if they are missing.
   * Verify that `arduino‑app‑cli` is available and abort if not.
   * Copy the `uno_q_app` folder into `~/ArduinoApps/measurepi`.
   * Install Python dependencies listed in `uno_q_app/python/requirements.txt` using `pip`.
   * Build the app (`arduino-app-cli app build measurepi`) and start it (`arduino-app-cli app start measurepi`).
   You can monitor logs with:
   ```sh
   arduino-app-cli app logs measurepi
   ```

3. **Configure MQTT and reference dimensions** (optional). The application reads a number of environment variables to allow configuration without editing code:

   | Variable          | Default      | Purpose                                                      |
   |-------------------|--------------|--------------------------------------------------------------|
   | `MQTT_HOST`       | `localhost`  | Hostname of the MQTT broker.                                 |
   | `MQTT_PORT`       | `1883`       | TCP port of the MQTT broker.                                 |
   | `MQTT_USER`       | (unset)      | Username for MQTT authentication.                            |
   | `MQTT_PASS`       | (unset)      | Password for MQTT authentication.                            |
   | `MQTT_TOPIC`      | `measurepi`  | Topic prefix for published measurements.                     |
   | `REF_LENGTH_CM`   | `80.0`       | Reference distance for the length sensor.                    |
   | `REF_HEIGHT_CM`   | `89.0`       | Reference distance for the height sensor.                    |
   | `REF_WIDTH_CM`    | `70.0`       | Reference distance for the width sensor.                     |
   | `DASHBOARD_PORT`  | `5000`       | Port to serve the Flask dashboard on.                       |
   | `ALLOWED_ORIGINS` | `*`          | CORS/CSRF protection: comma‑separated list of allowed origins for the dashboard. |

   You can set these variables in the systemd unit or export them before starting the app manually. For example:
   ```sh
   export MQTT_HOST=192.168.1.10
   export MQTT_USER=myuser
   export MQTT_PASS=secret
   arduino-app-cli app stop measurepi
   arduino-app-cli app start measurepi
   ```

## Structure of the UNO Q App

Arduino App Lab expects a specific directory structure for apps deployed on the UNO Q. This repository provides a ready‑made `uno_q_app` directory that conforms to that structure:

```
uno_q_app/
├── app.yaml                 # Metadata file describing the app
├── python/
│   ├── main.py              # Entry point (bridge and MQTT logic)
│   ├── measurepi_dashboard.py # Flask dashboard
│   └── requirements.txt     # Python dependencies
└── sketch/
    ├── sketch.ino           # MCU sketch (copied from measure_uno_q.ino)
    └── sketch.yaml          # FQBN and library dependencies
```

The `app.yaml` file registers the app name (`measurepi`) and defines default environment variables. The `sketch/sketch.yaml` specifies that the project targets the UNO Q (`arduino:zephyr:unoq`) and lists the libraries used by the sketch (TFLI2C, Adafruit NeoPixel and Arduino RouterBridge). A tutorial on UNO Q app structure shows a similar layout with `app.yaml`, `python/main.py`, `python/requirements.txt`, `sketch/sketch.ino` and `sketch/sketch.yaml`. The `arduino-app-cli` tool uses this structure to build and manage the application on the board.

## Developing and Testing Locally

Although this project is intended for the UNO Q, you can run the Python dashboard and MQTT bridge on a regular computer for testing. Run the bridge directly with:
```sh
python3 bridge_mqtt.py
```
and the dashboard with:
```sh
python3 measurepi_dashboard.py
```
Be sure to set the environment variables above to point at your simulated sensors or adjust the code accordingly. On the UNO Q the bridge automatically talks to the MCU via the Router Bridge; when running off‑board you may want to stub out those calls.

## Contributing

Pull requests and bug reports are welcome. Please ensure that changes are thoroughly tested on both the MCU and Python sides before submission. See PR #37 for context on past improvements to this project.
