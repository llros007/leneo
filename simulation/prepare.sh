#!/usr/bin/env bash
set -euo pipefail
SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${LENEO_PYTHON:-python}" "$SIM_DIR/common/prepare_assets.py" "$@"
