#!/usr/bin/env bash
[ -n "$BASH_VERSION" ] || { echo "This script requires bash"; exit 1; }
set -euo pipefail


APP_ID="measurepi"
REPO_URL="https://github.com/mitchweightman-nz/measurepi"
REPO_DIR="${HOME}/src/${APP_ID}"

log() { printf "\n[%s] %s\n" "${APP_ID}" "$*"; }
die() { echo "ERROR: $*" >&2; exit 1; }
need_cmd() { command -v "$1" >/dev/null 2>&1 || die "Missing command: $1"; }

log "Preflight"
need_cmd arduino-app-cli
need_cmd git
need_cmd python3
need_cmd rsync

mkdir -p "$(dirname "${REPO_DIR}")"

log "Fetching latest repo into ${REPO_DIR}"
if [[ -d "${REPO_DIR}/.git" ]]; then
  git -C "${REPO_DIR}" fetch --all --prune
  git -C "${REPO_DIR}" reset --hard origin/main
else
  git clone "${REPO_URL}" "${REPO_DIR}"
fi

# Run deploy script from the repo (single source of truth)
DEPLOY_SCRIPT="${REPO_DIR}/deploy_uno_q.sh"
[[ -f "${DEPLOY_SCRIPT}" ]] || die "deploy_uno_q.sh not found in repo after update."
chmod +x "${DEPLOY_SCRIPT}"

log "Deploying updated app bundle"
(cd "${REPO_DIR}" && ./deploy_uno_q.sh)

log "Tailing logs (Ctrl+C to exit)"
arduino-app-cli app logs "${APP_ID}"

