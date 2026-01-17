#!/usr/bin/env bash
set -euo pipefail

APP_NAME="measurepi"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_DIR="${REPO_ROOT}/uno_q_app"
TARGET_DIR="${HOME}/ArduinoApps/${APP_NAME}"

log() {
  printf '\n[%s] %s\n' "${APP_NAME}" "$1"
}

if ! command -v arduino-app-cli >/dev/null 2>&1; then
  echo "Error: arduino-app-cli is not installed or not on PATH." >&2
  echo "Install Arduino App CLI before running this script." >&2
  exit 1
fi

if [[ ! -d "${SOURCE_DIR}" ]]; then
  echo "Error: expected ${SOURCE_DIR} to exist." >&2
  echo "Ensure the repository includes the uno_q_app directory." >&2
  exit 1
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
