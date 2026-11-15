#!/usr/bin/env bash
# On the board: Joy-Con paired to this board controls the local LeNeo server.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export HOST=127.0.0.1
export JOYCON_SOFTWARE_DIR="${JOYCON_SOFTWARE_DIR:-$HOME/workspace/lerobot/XLeRobot/software}"
exec "${SCRIPT_DIR}/run_leneo_remote_joycon_pc.sh" "$@"
