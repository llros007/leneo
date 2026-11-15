#!/usr/bin/env bash
# Keyboard only — runs on PC or board.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROLE=keyboard
export HOST="${HOST:-192.168.124.34}"
exec "${SCRIPT_DIR}/run_leneo_remote.sh" keyboard "$@"
