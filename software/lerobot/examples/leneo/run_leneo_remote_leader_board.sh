#!/usr/bin/env bash
# Leader USB on the board; follower server on same host (localhost).
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export HOST="${HOST:-127.0.0.1}"
export TELEOP_PORT="${TELEOP_PORT:-auto}"
exec "${SCRIPT_DIR}/run_leneo_remote_pc.sh" "$@"
