#!/usr/bin/env bash
# PC-only: follower + leader (+ lift) USB on this computer — no board / no network to RK3588.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROLE=local
export LENEO_RUNTIME="${LENEO_RUNTIME:-auto}"
exec "${SCRIPT_DIR}/run_leneo_remote.sh" local "$@"
