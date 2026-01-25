#!/usr/bin/env bash
set -euo pipefail

APP_ID="measurepi"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_APP_DIR="${REPO_ROOT}/uno_q_app"
DST_APP_DIR="${HOME}/ArduinoApps/${APP_ID}"

PY_DIR_REL="python"
VENV_DIR_REL="${PY_DIR_REL}/.venv"
REQ_FILE_REL="${PY_DIR_REL}/requirements.txt"

log() { printf "\n[%s] %s\n" "${APP_ID}" "$*"; }
die() { echo "ERROR: $*" >&2; exit 1; }

need_cmd() { command -v "$1" >/dev/null 2>&1 || die "Missing command: $1"; }

log "Preflight"
need_cmd arduino-app-cli
need_cmd python3
need_cmd rsync

[[ -d "${SRC_APP_DIR}" ]] || die "Expected '${SRC_APP_DIR}' to exist (UNO Q app bundle not found)."

# Ensure target base dir exists
mkdir -p "${HOME}/ArduinoApps"

# Stop app if running (best effort)
log "Stopping app (best effort)"
arduino-app-cli app stop "${APP_ID}" >/dev/null 2>&1 || true

# Sync app bundle into ArduinoApps (idempotent)
log "Syncing app bundle to ${DST_APP_DIR}"
mkdir -p "${DST_APP_DIR}"

# Preserve .venv between deploys unless requirements changed: we keep it, rsync will not delete if excluded.
rsync -a --delete \
  --exclude "/${VENV_DIR_REL}/" \
  "${SRC_APP_DIR}/" "${DST_APP_DIR}/"

# Python dependencies in per-app venv
REQ_FILE="${DST_APP_DIR}/${REQ_FILE_REL}"
VENV_DIR="${DST_APP_DIR}/${VENV_DIR_REL}"

if [[ -f "${REQ_FILE}" ]]; then
  log "Ensuring Python venv at ${VENV_DIR}"
  if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
    python3 -m venv "${VENV_DIR}"
  fi

  # Upgrade pip/setuptools/wheel inside venv
  "${VENV_DIR}/bin/python" -m pip install -U pip setuptools wheel >/dev/null

  log "Installing Python deps from ${REQ_FILE}"
  "${VENV_DIR}/bin/python" -m pip install -r "${REQ_FILE}"
else
  log "No ${REQ_FILE_REL} found; skipping Python deps install"
fi

# Build & start app
log "Building app: ${APP_ID}"
arduino-app-cli app build "${APP_ID}"

log "Starting app: ${APP_ID}"
arduino-app-cli app start "${APP_ID}"

log "Done."
echo "Next:"
echo "  arduino-app-cli app status ${APP_ID}"
echo "  arduino-app-cli app logs   ${APP_ID}"
