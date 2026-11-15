#!/usr/bin/env bash
# On the board desktop: USB keyboard controls the local LeNeo server (127.0.0.1:9595).
# Do not run this over plain SSH; keys must be pressed in this graphical terminal.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export HOST=127.0.0.1
exec "${SCRIPT_DIR}/run_leneo_remote_keyboard_pc.sh" "$@"
