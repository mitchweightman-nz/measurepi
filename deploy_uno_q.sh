#!/bin/bash
#
# Deployment helper for the MeasurePi UNO Q application.
#
# This script automates installation of Python dependencies, creation of
# the UNO Q App, and starting the application using the `arduino-app-cli`
# tool.  Run this on the Linux side of your UNO Q after cloning the
# repository.  You can invoke it from the root of the repository like so:
#
#   chmod +x deploy_uno_q.sh
#   ./deploy_uno_q.sh
#
# The script will:
#   1. Update the package index and install Python and Git if needed.
#   2. Ensure the Arduino App CLI is available.
#   3. Copy the `uno_q_app` folder into your `~/ArduinoApps` directory,
#      naming the app `measurepi`.
#   4. Install Python package requirements with pip.
#   5. Build and start the app via `arduino-app-cli`.
#
# See the README.md for more details.

set -euo pipefail

# Determine the absolute path to this script and repository root
REPO_DIR="$(cd "$(dirname "$0")" && pwd)"

# Ensure apt is up‑to‑date and required tools are installed
echo "Updating package lists and installing prerequisites..."
sudo apt update -y
sudo apt install -y python3 python3-pip git > /dev/null

# Check that arduino-app-cli exists on the system
if ! command -v arduino-app-cli >/dev/null 2>&1; then
  echo "Error: arduino-app-cli is not installed."
  echo "Please install the Arduino App Lab tools per the UNO Q documentation before running this script."
  exit 1
fi

# Define application name and paths
APP_NAME="measurepi"
UNO_APPS_DIR="$HOME/ArduinoApps"
APP_DEST_DIR="$UNO_APPS_DIR/$APP_NAME"

echo "Preparing application directory $APP_DEST_DIR..."
mkdir -p "$UNO_APPS_DIR"
# Remove any existing app directory to avoid stale files
rm -rf "$APP_DEST_DIR"
# Copy the uno_q_app folder into the apps directory
cp -r "$REPO_DIR/uno_q_app" "$APP_DEST_DIR"

# Install Python requirements
REQ_FILE="$APP_DEST_DIR/python/requirements.txt"
if [ -f "$REQ_FILE" ]; then
  echo "Installing Python dependencies..."
  python3 -m pip install --user -r "$REQ_FILE"
else
  echo "Warning: requirements.txt not found at $REQ_FILE – skipping Python dependency installation."
fi

# Build the Arduino app
echo "Building the UNO Q app..."
arduino-app-cli app build "$APP_NAME"

# Start (or restart) the app
echo "Starting the UNO Q app..."
arduino-app-cli app stop "$APP_NAME" || true
arduino-app-cli app start "$APP_NAME"

echo "Deployment complete.  Use 'arduino-app-cli app logs measurepi' to monitor the application logs."
