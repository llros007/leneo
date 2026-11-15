#!/usr/bin/env bash
# Use existing lerobot uv venv (paths differ per board).
set -euo pipefail
export PATH="${HOME}/.local/bin:${PATH}"
if [ -d "${HOME}/ai/lerobot/.venv" ]; then
  VENV="${HOME}/ai/lerobot/.venv"
elif [ -d "${HOME}/ai/bot/lerobot-main/.venv" ]; then
  VENV="${HOME}/ai/bot/lerobot-main/.venv"
else
  echo "No lerobot .venv under ~/ai/lerobot or ~/ai/bot/lerobot-main" >&2
  echo "On board: cd ~/ai/lerobot && uv sync --locked --extra feetech" >&2
  exit 1
fi
LR="${HOME}/ai/leneo/lerobot"
# Do not rm .venv here: leneo/lerobot often symlinks to ~/ai/lerobot.
source "${VENV}/bin/activate"
export PYTHONPATH="${LR}/src"
cd "${LR}"
exec uv run --active --no-sync "$@"
