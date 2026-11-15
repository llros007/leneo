#!/usr/bin/env bash
# Shared runtime for LeNeo remote scripts — PC (conda/uv) or RK3588 board (uv venv).
# Source from other wrappers:  source "$(dirname "$0")/leneo_run_env.sh"
set -euo pipefail

_leneo_script_dir() {
  local _src="${BASH_SOURCE[1]:-${BASH_SOURCE[0]}}"
  cd "$(dirname "${_src}")" && pwd
}

leneo_resolve_repo() {
  if [ -n "${LENEO_REPO:-}" ] && [ -f "${LENEO_REPO}/pyproject.toml" ]; then
    LENEO_REPO="$(cd "${LENEO_REPO}" && pwd)"
    return 0
  fi
  local _dir
  _dir="$(_leneo_script_dir)"
  if [ -f "${_dir}/../../pyproject.toml" ]; then
    LENEO_REPO="$(cd "${_dir}/../.." && pwd)"
    return 0
  fi
  for candidate in \
    "${HOME}/ai/leneo/lerobot" \
    "${HOME}/workspace/lerobot/lerobot" \
    "${HOME}/ai/lerobot"; do
    if [ -f "${candidate}/pyproject.toml" ]; then
      LENEO_REPO="$(cd "${candidate}" && pwd)"
      return 0
    fi
  done
  echo "Cannot find lerobot repo (set LENEO_REPO=...)" >&2
  return 1
}

leneo_detect_runtime() {
  if [ -n "${LENEO_RUNTIME:-}" ] && [ "${LENEO_RUNTIME}" != "auto" ]; then
    return 0
  fi
  if [ -f "${HOME}/ai/leneo/run_with_board_uv.sh" ] && [ -d "${HOME}/ai/lerobot/.venv" ]; then
    LENEO_RUNTIME=board
    return 0
  fi
  if command -v conda >/dev/null 2>&1 && [ -n "${CONDA_ENV:-le312}" ]; then
    LENEO_RUNTIME=conda
    return 0
  fi
  if command -v uv >/dev/null 2>&1 || [ -x "${HOME}/.local/bin/uv" ]; then
    LENEO_RUNTIME=uv
    return 0
  fi
  LENEO_RUNTIME=system
}

leneo_activate_python() {
  leneo_resolve_repo
  leneo_detect_runtime
  export PATH="${HOME}/.local/bin:${PATH}"
  case "${LENEO_RUNTIME}" in
    board)
      # run_with_board_uv sets PYTHONPATH and cwd
      return 0
      ;;
    conda)
      # shellcheck disable=SC1091
      source "$(conda info --base)/etc/profile.d/conda.sh"
      conda activate "${CONDA_ENV:-le312}"
      export PYTHONPATH="${LENEO_REPO}/src"
      cd "${LENEO_REPO}"
      ;;
    uv)
      export PYTHONPATH="${LENEO_REPO}/src"
      cd "${LENEO_REPO}"
      ;;
    system)
      export PYTHONPATH="${LENEO_REPO}/src"
      cd "${LENEO_REPO}"
      ;;
  esac
}

leneo_run_python() {
  leneo_activate_python
  case "${LENEO_RUNTIME}" in
    board)
      "${HOME}/ai/leneo/run_with_board_uv.sh" python "$@"
      ;;
    uv)
      uv run --no-sync python "$@"
      ;;
    *)
      python "$@"
      ;;
  esac
}

leneo_exec_python() {
  leneo_run_python "$@"
  exit $?
}
