#!/usr/bin/env bash
set -euo pipefail
SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-$HOME/isaac-sim-6.2.0}"
if [[ ! -x "$ISAAC_SIM_ROOT/python.sh" ]]; then echo "Isaac Sim 6.2 runtime missing: $ISAAC_SIM_ROOT" >&2; exit 1; fi
if [[ "$(cat "$ISAAC_SIM_ROOT/VERSION")" != 6.2.* ]]; then echo 'Isaac Sim 6.2 required' >&2; exit 1; fi
if [[ ! -f "$SIM_DIR/../assets/modern_kitchen/manifest.json" ]]; then "$SIM_DIR/../prepare.sh"; fi
# Isolate the bundled runtime from any currently active Conda Isaac Sim 5.x.
unset PYTHONPATH PYTHONHOME CONDA_PREFIX PYTHONEXE
quality=fast
for ((i=1; i<=$#; i++)); do
    if [[ "${!i}" == --quality ]]; then j=$((i+1)); quality="${!j}"; fi
    if [[ "${!i}" == --quality=full ]]; then quality=full; fi
done
robot_dir="$SIM_DIR/../assets/isaac_robot"
if [[ "$quality" == fast ]]; then
    "$SIM_DIR/../prepare_runtime.sh"
    robot_dir="$SIM_DIR/../assets/isaac_robot_fast"
fi
if [[ ! -f "$robot_dir/robot_path.txt" ]] || { [[ "$quality" == fast ]] && ! cmp -s "$robot_dir/source_signature.txt" <("${LENEO_PYTHON:-python}" -c 'import json,sys;sys.stdout.write(json.load(open(sys.argv[1]))["signature"])' "$SIM_DIR/../assets/runtime_robot/manifest.json"); }; then
    "$ISAAC_SIM_ROOT/python.sh" --no-ros-env "$SIM_DIR/convert_robot.py" --quality "$quality"
fi
exec "$ISAAC_SIM_ROOT/python.sh" --no-ros-env "$SIM_DIR/run.py" "$@"
