#!/usr/bin/env bash
set -euo pipefail

APP_ID="measurepi"
APP_DST="${HOME}/ArduinoApps/${APP_ID}"
REPO_URL="https://github.com/mitchweightman-nz/measurepi"
REPO_DIR="${HOME}/src/${APP_ID}"

echo "== Preflight =="
command -v arduino-app-cli >/dev/null 2>&1 || { echo "ERROR: arduino-app-cli not found"; exit 1; }
command -v git >/dev/null 2>&1 || { echo "ERROR: git not found (sudo apt install -y git)"; exit 1; }
command -v python3 >/dev/null 2>&1 || { echo "ERROR: python3 not found (sudo apt install -y python3 python3-venv)"; exit 1; }

echo "== Stop running app (if present) =="
arduino-app-cli app stop "${APP_ID}" >/dev/null 2>&1 || true

echo "== Remove existing installed app folder =="
rm -rf "${APP_DST}"

echo "== Fetch latest repo =="
mkdir -p "$(dirname "${REPO_DIR}")"
if [ -d "${REPO_DIR}/.git" ]; then
  git -C "${REPO_DIR}" fetch --all --prune
  git -C "${REPO_DIR}" reset --hard origin/main
else
  git clone "${REPO_URL}" "${REPO_DIR}"
fi

echo "== Ensure uno_q_app exists (repo provides it, deploy script can generate if missing) =="
if [ ! -d "${REPO_DIR}/uno_q_app" ]; then
  echo "uno_q_app not found, attempting to generate via deploy_uno_q.sh"
  chmod +x "${REPO_DIR}/deploy_uno_q.sh"
  (cd "${REPO_DIR}" && ./deploy_uno_q.sh)
fi

echo "== Install app into ArduinoApps =="
mkdir -p "$(dirname "${APP_DST}")"
cp -a "${REPO_DIR}/uno_q_app" "${APP_DST}"

echo "== Install Python deps (best effort) =="
REQ="${APP_DST}/python/requirements.txt"
if [ -f "${REQ}" ]; then
  python3 -m pip install -U pip >/dev/null 2>&1 || true
  python3 -m pip install -r "${REQ}"
else
  echo "WARNING: ${REQ} not found, skipping pip install"
fi

echo "== Build and start app =="
arduino-app-cli app build "${APP_ID}"
arduino-app-cli app start "${APP_ID}"

echo "== Tail logs (Ctrl+C to exit) =="
arduino-app-cli app logs "${APP_ID}"
