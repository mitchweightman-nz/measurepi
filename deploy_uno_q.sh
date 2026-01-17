#!/usr/bin/env bash
set -euo pipefail

APP_NAME="measurepi"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_DIR="${REPO_ROOT}/uno_q_app"
TARGET_DIR="${HOME}/ArduinoApps/${APP_NAME}"
TEMP_DIR=""

log() {
  printf '\n[%s] %s\n' "${APP_NAME}" "$1"
}

if ! command -v arduino-app-cli >/dev/null 2>&1; then
  echo "Error: arduino-app-cli is not installed or not on PATH." >&2
  echo "Install Arduino App CLI before running this script." >&2
  exit 1
fi

cleanup() {
  if [[ -n "${TEMP_DIR}" && -d "${TEMP_DIR}" ]]; then
    rm -rf "${TEMP_DIR}"
  fi
}

create_app_bundle() {
  local destination="$1"
  local sketch_source="${REPO_ROOT}/measure_uno_q.ino"
  local bridge_source="${REPO_ROOT}/bridge_mqtt.py"
  local dashboard_source="${REPO_ROOT}/measurepi_dashboard.py"
  local templates_source="${REPO_ROOT}/templates"

  if [[ ! -f "${sketch_source}" || ! -f "${bridge_source}" || ! -f "${dashboard_source}" ]]; then
    echo "Error: missing required source files in ${REPO_ROOT}." >&2
    exit 1
  fi

  log "uno_q_app not found; generating app bundle from repository sources"
  mkdir -p "${destination}/sketch" "${destination}/python"

  cp "${sketch_source}" "${destination}/sketch/sketch.ino"
  cp "${bridge_source}" "${destination}/python/main.py"
  cp "${dashboard_source}" "${destination}/python/measurepi_dashboard.py"

  if [[ -d "${templates_source}" ]]; then
    cp -a "${templates_source}" "${destination}/python/templates"
  fi

  cat <<'EOF' > "${destination}/python/requirements.txt"
paho-mqtt
Flask
EOF

  cat <<'EOF' > "${destination}/sketch/sketch.yaml"
fqbn: arduino:zephyr:unoq
libraries:
  - name: TFLI2C
  - name: Adafruit NeoPixel
  - name: Arduino RouterBridge
EOF

  cat <<'EOF' > "${destination}/app.yaml"
name: measurepi
version: 1.0.0
description: MeasurePi UNO Q app bundle
services:
  - name: measurepi
    type: python
    entrypoint: python/main.py
EOF
}

if [[ ! -d "${SOURCE_DIR}" ]]; then
  TEMP_DIR="$(mktemp -d)"
  trap cleanup EXIT
  create_app_bundle "${TEMP_DIR}/uno_q_app"
  SOURCE_DIR="${TEMP_DIR}/uno_q_app"
fi

log "Updating package index and installing dependencies"
sudo apt update
sudo apt install -y git python3 python3-pip

log "Deploying UNO Q app to ${TARGET_DIR}"
mkdir -p "${HOME}/ArduinoApps"
rm -rf "${TARGET_DIR}"
cp -a "${SOURCE_DIR}" "${TARGET_DIR}"

if [[ -f "${TARGET_DIR}/python/requirements.txt" ]]; then
  log "Installing Python dependencies"
  python3 -m pip install --user -r "${TARGET_DIR}/python/requirements.txt"
else
  log "Skipping Python dependencies (requirements.txt not found)"
fi

log "Building and starting ${APP_NAME}"
arduino-app-cli app build "${APP_NAME}"
arduino-app-cli app start "${APP_NAME}"

log "Deployment complete"
