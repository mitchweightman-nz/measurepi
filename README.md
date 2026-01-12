MeasurePi UNO Q Measurement Rig
===============================

This repository contains aself‑contained application for the **Arduino UNO Q** board thatmeasures parcel dimensions 
using three TF‑Luna time‑of‑flight sensors, providesa simple web dashboard and publishes the results over MQTT. 
The project issplit into two components:

1.      **Microcontroller sketch (measure\_uno\_q.ino)** – runs on the STM32U585 microcontroller and handles the low‑leveltiming, 
        laser control, sensor multiplexing and state‑machine logic. 
        The sketchcommunicates with the Linux side of the UNO Q via the Router Bridge RPCinterface. The same code is included under                  
        uno\_q\_app/sketch/sketch.ino so that it is automatically built as part of the UNO Q app.

2.      **Python application** – runs on the embeddedDebian system (MPU) and uses the paho‑mqtt and 
        Flask libraries to publish measurements to an MQTT broker and              
        serve a dashboard. The entry pointis uno\_q\_app/python/main.py (a copy of bridge\_mqtt.py), 
        which interacts with the microcontroller via arduino.app\_utils.

The project can be deployed on astandard Raspberry Pi, but this document focuses on installing it on anArduino UNO Q using Arduino App Lab.

Prerequisites
-------------

·      An Arduino UNO Q boardrunning the default Debian image.

·      A 3D‑printed measurement rig wiredwith:

·      Three TF‑Luna I²C sensorsconnected via a TCA9548A multiplexer.

·      An eight‑pixel NeoPixel strip onpin 7 for status indication.

·      A laser driver on pin 10.

·      Access to an MQTT broker (locallyor remotely). You may adjust broker settings via environment variables asdescribed below.

·      Network access on the UNO Qto run apt commands. The Arduino documentationnotes that installing software on the UNO Q is done using the standardDebian package         manager, e.g. sudo apt installpackage-name[\[1\]](https://docs.arduino.cc/tutorials/uno-q/debian-guide/). In ourdeployment script we use sudo apt update and          sudo apt install python3 python3-pip git toensure the Python runtime and Git are available
       [\[2\]](https://docs.arduino.cc/tutorials/uno-q/debian-guide/#package-management).

·      The **Arduino App CLI**(arduino-app-cli), which is pre‑installed onUNO Q images. It allows creating, building and managing apps directly onthe device. The            official documentation demonstrates how to create an app, buildit and start/stop it using commands such as arduino-app-cli appnew, arduino-app-cli appbuild           and arduino-app-cli appstart
       [\[3\]](https://docs.arduino.cc/software/app-lab/tutorials/cli/#using-arduino-app-cli).

Deploymenton UNO Q
------------------

The repository includes a helper script, deploy\_uno\_q.sh, that automatesinstallation and deployment. Follow these steps on your UNO Q:

1.     **Clone this repository** onto the UNO Q. You can use git clone if Git is installed. For example:

    sudo apt update && sudo apt install -ygitgit clone https://github.com/your‑org/measurepi.gitcd measurepi

2.     **Run the deployment script**. Make it executable and run it from the repository root:

    chmod +x deploy\_uno\_q.sh./deploy\_uno\_q.sh

           The script will:

3.     Update the package index andinstall Python and Git if they are missing.

4.     Verify that arduino‑app‑cli is available and abort ifnot.

5.     Copy the uno\_q\_app folder into ~/ArduinoApps/measurepi.

6.     Install Python dependencies listedin uno\_q\_app/python/requirements.txt using pip.

6.     Build the app (arduino-app-cli app build measurepi) and start it (arduino-app-cli app start measurepi).
       You can monitor logs with arduino-app-cli app logs measurepi.
    
7.  **Configure MQTT and reference dimensions** (optional). The application reads a number of environment variables to allow configuration without editing code:
    

Variable

Default

Purpose

MQTT\_HOST

localhost

Hostname of the MQTT broker.

MQTT\_PORT

1883

TCP port of the MQTT broker.

MQTT\_USER

(unset)

Username for MQTT authentication.

MQTT\_PASS

(unset)

Password for MQTT authentication.

MQTT\_TOPIC

measurepi

Topic prefix for published measurements.

REF\_LENGTH\_CM

80.0

Reference distance for the length sensor.

REF\_HEIGHT\_CM

89.0

Reference distance for the height sensor.

REF\_WIDTH\_CM

70.0

Reference distance for the width sensor.

DASHBOARD\_PORT

5000

Port to serve the Flask dashboard on.

ALLOWED\_ORIGINS

\*

CORS/CSRF protection: comma‑separated list

of allowed origins for the dashboard.

You can set these variables in the systemd unit or pass them whenstarting the app manually. For example:

export MQTT\_HOST=192.168.1.10export MQTT\_USER=myuserexport MQTT\_PASS=secretarduino-app-cli app stop measurepiarduino-app-cli app start measurepi

Structureof the UNO Q App
-------------------------

Arduino App Lab expects a specific directory structure forapps deployed on the UNO Q. This repository provides a ready‑made uno\_q\_app directory that conforms to thatstructure:

uno\_q\_app/├── app.yaml               # Metadata file describing theapp├── python/│   ├──main.py            # Entry point (bridgeand MQTT logic)│   ├──measurepi\_dashboard.py  # Flask dashboard│   └──requirements.txt   # Python dependencies└── sketch/    ├──sketch.ino         # MCU sketch (copiedfrom measure\_uno\_q.ino)    └──sketch.yaml        # FQBN and librarydependencies

The app.yaml file registers the app name (measurepi) and defines default environmentvariables. The sketch/sketch.yaml specifies that theproject targets the UNO Q (arduino:zephyr:unoq) and lists thelibraries used by the sketch (TFLI2C, Adafruit NeoPixel andArduino RouterBridge). A tutorial on UNO Q app structure shows asimilar layout with app.yaml, python/main.py, python/requirements.txt, sketch/sketch.ino and sketch/sketch.yaml[\[4\]](https://shawnhymel.com/3074/how-to-use-the-command-line-cli-with-the-arduino-uno-q/#:~:text=Project Structure).The arduino-app-cli tool uses this structure tobuild and manage the application on the board.

Developing and Testing Locally
------------------------------

Although this project is intended for the UNO Q, you can run thePython dashboard and MQTT bridge on a regular computer for testing. Run thebridge directly with:

python3 bridge\_mqtt.py

and the dashboard with:

python3 measurepi\_dashboard.py

Be sure to set the environment variables above to point at yoursimulated sensors or adjust the code accordingly. On the UNO Q the bridgeautomatically talks to the MCU via the Router Bridge; when running off‑boardyou may want to stub out those calls.

Contributing
------------

Pull requests and bug reports are welcome. Please ensure that changesare thoroughly tested on both the MCU and Python sides before submission. SeePR #37 for context on past improvements to this project.

[\[1\]](https://docs.arduino.cc/tutorials/uno-q/debian-guide/) Debian Linux Basics for UNO Q | Arduino Documentation

[https://docs.arduino.cc/tutorials/uno-q/debian-guide/](https://docs.arduino.cc/tutorials/uno-q/debian-guide/)

[\[2\]](https://docs.arduino.cc/tutorials/uno-q/debian-guide/#package-management) Debian Linux Basics for UNO Q | Arduino Documentation

[https://docs.arduino.cc/tutorials/uno-q/debian-guide/#package-management](https://docs.arduino.cc/tutorials/uno-q/debian-guide/#package-management)

[\[3\]](https://docs.arduino.cc/software/app-lab/tutorials/cli/#using-arduino-app-cli) Arduino App CLI: Manage Apps from the Command Line | ArduinoDocumentation

[https://docs.arduino.cc/software/app-lab/tutorials/cli/#using-arduino-app-cli](https://docs.arduino.cc/software/app-lab/tutorials/cli/#using-arduino-app-cli)

[\[4\]](https://shawnhymel.com/3074/how-to-use-the-command-line-cli-with-the-arduino-uno-q/#:~:text=Project Structure) How to Use the Command Line (CLI) With the Arduino UNO Q - Shawn Hymel

[https://shawnhymel.com/3074/how-to-use-the-command-line-cli-with-the-arduino-uno-q/](https://shawnhymel.com/3074/how-to-use-the-command-line-cli-with-the-arduino-uno-q/)
