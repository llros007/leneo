#!/usr/bin/env bash
# PC: right Joy-Con → remote LeNeo on the board.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export CONDA_ENV="${CONDA_ENV:-le312}"
export PYTHONUNBUFFERED=1
export JOYCON_SOFTWARE_DIR="${JOYCON_SOFTWARE_DIR:-$HOME/workspace/lerobot/XLeRobot/software}"
if [ ! -d "${JOYCON_SOFTWARE_DIR}/joyconrobotics" ]; then
  echo "找不到 joyconrobotics: ${JOYCON_SOFTWARE_DIR}" >&2
  exit 1
fi
HOST="${HOST:-192.168.124.35}"
# shellcheck source=leneo_run_env.sh
source "${SCRIPT_DIR}/leneo_run_env.sh"
leneo_exec_python examples/leneo/leneo_remote_client_joycon.py --host="${HOST}" --port="${PORT:-9595}" "$@"
