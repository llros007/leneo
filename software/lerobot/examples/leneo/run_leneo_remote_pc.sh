#!/usr/bin/env bash
# Leader on this machine (PC or board) + keyboard → remote follower on board.
set -euo pipefail
# Use le312 (base env lacks lerobot deps). Scripts activate conda automatically.
export CONDA_ENV="${CONDA_ENV:-le312}"
export PYTHONUNBUFFERED=1
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROLE=client
export HOST="${HOST:-192.168.124.34}"
export TELEOP_PORT="${TELEOP_PORT:-auto}"
export TELEOP_ID="${TELEOP_ID:-7vw_leader_arm}"
exec "${SCRIPT_DIR}/run_leneo_remote.sh" client "$@"
