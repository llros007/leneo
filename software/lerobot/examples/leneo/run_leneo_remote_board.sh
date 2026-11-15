#!/usr/bin/env bash
# Board follower server — wrapper for unified run_leneo_remote.sh
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROLE=server
exec "${SCRIPT_DIR}/run_leneo_remote.sh" server "$@"
