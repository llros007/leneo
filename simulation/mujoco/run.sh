#!/usr/bin/env bash
set -euo pipefail
SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f "$SIM_DIR/../assets/modern_kitchen/manifest.json" ]]; then "$SIM_DIR/../prepare.sh"; fi
exec "${LENEO_PYTHON:-python}" "$SIM_DIR/run.py" "$@"
