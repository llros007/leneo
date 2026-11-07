#!/usr/bin/env bash
set -euo pipefail
SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SIM_DIR"
exec "${LENEO_PYTHON:-python}" -m common.prepare_runtime "$@"
