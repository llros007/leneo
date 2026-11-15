#!/usr/bin/env bash
# LeNeo remote teleop — one entry for PC or board.
#
# Roles:
#   server          — follower + lift on this machine (usually the board), TCP 9595
#   client          — leader arm + keyboard → remote server
#   keyboard        — keyboard only (no leader arm)
#
# Examples:
#   Board (systemd / manual):
#     ROLE=server ./examples/leneo/run_leneo_remote.sh
#
#   PC, leader on PC USB:
#     ROLE=client HOST=192.168.124.34 ./examples/leneo/run_leneo_remote.sh
#
#   Board, leader USB on board (server on same host):
#     ROLE=client HOST=127.0.0.1 ./examples/leneo/run_leneo_remote.sh
#
#   PC or board, keyboard only:
#     ROLE=keyboard HOST=192.168.124.34 ./examples/leneo/run_leneo_remote.sh
#
#   PC only (follower + leader USB on same computer, no RK3588 board):
#     ROLE=local ./examples/leneo/run_leneo_remote.sh
#     # or: ./examples/leneo/run_leneo_local_pc.sh
#
set -euo pipefail
export PYTHONUNBUFFERED=1
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=leneo_run_env.sh
source "${SCRIPT_DIR}/leneo_run_env.sh"

ROLE="${ROLE:-${1:-}}"
if [ -z "${ROLE}" ]; then
  echo "Usage: ROLE=server|client|keyboard|local $0" >&2
  echo "   or: $0 server|client|keyboard|local" >&2
  exit 1
fi
shift 2>/dev/null || true

HOST="${HOST:-192.168.124.34}"
PORT="${PORT:-9595}"
TELEOP_PORT="${TELEOP_PORT:-auto}"
TELEOP_ID="${TELEOP_ID:-7vw_leader_arm}"
TELEOP_RETRY="${TELEOP_RETRY:-2}"
ROBOT_PORT="${ROBOT_PORT:-auto}"
LIFT_PORT="${LIFT_PORT:-auto}"
ROBOT_ID="${ROBOT_ID:-blue_follower_arm}"
LIFT_RPM="${LIFT_RPM:-500}"
LIFT_ENABLED="${LIFT_ENABLED:-auto}"
BIND="${BIND:-0.0.0.0}"
PORT_RETRY="${PORT_RETRY:-2}"

case "${ROLE}" in
  server | board | follower)
    LIFT_ARGS=(--lift.port="${LIFT_PORT}" --lift.speed_rpm="${LIFT_RPM}")
    case "${LIFT_ENABLED}" in
      false | 0) LIFT_ARGS=(--no-lift.enabled) ;;
      auto | true | 1) LIFT_ARGS+=(--lift.enabled) ;;
      *)
        echo "LIFT_ENABLED must be auto, true, or false (got ${LIFT_ENABLED})" >&2
        exit 1
        ;;
    esac
    leneo_exec_python examples/leneo/leneo_remote_server.py \
      --robot.port="${ROBOT_PORT}" \
      --robot.id="${ROBOT_ID}" \
      "${LIFT_ARGS[@]}" \
      --port.retry_interval="${PORT_RETRY}" \
      --bind="${BIND}" \
      --port="${PORT}" \
      "$@"
    ;;
  client | teleop | leader)
    if [ "${TELEOP_PORT}" = "auto" ]; then
      leneo_activate_python
      TELEOP_PORT="$(python examples/leneo/find_leader_port.py --wait --retry-interval="${TELEOP_RETRY}" | tail -n1)" || exit 1
      echo "Using leader port: ${TELEOP_PORT}"
    fi
    leneo_exec_python examples/leneo/leneo_remote_client.py \
      --host="${HOST}" \
      --port="${PORT}" \
      --teleop.port="${TELEOP_PORT}" \
      --teleop.id="${TELEOP_ID}" \
      "$@"
    ;;
  keyboard | kb)
    leneo_exec_python examples/leneo/leneo_remote_client_keyboard.py \
      --host="${HOST}" \
      --port="${PORT}" \
      "$@"
    ;;
  local | pc | standalone)
    # All hardware on this machine: background follower server + foreground leader client.
    HOST=127.0.0.1
    BIND="${BIND:-127.0.0.1}"
    LIFT_ARGS=(--lift.port="${LIFT_PORT}" --lift.speed_rpm="${LIFT_RPM}")
    case "${LIFT_ENABLED}" in
      false | 0) LIFT_ARGS=(--no-lift.enabled) ;;
      auto | true | 1) LIFT_ARGS+=(--lift.enabled) ;;
    esac
    SERVER_PID=""
    cleanup_local() {
      if [ -n "${SERVER_PID}" ] && kill -0 "${SERVER_PID}" 2>/dev/null; then
        kill "${SERVER_PID}" 2>/dev/null || true
        wait "${SERVER_PID}" 2>/dev/null || true
      fi
    }
    trap cleanup_local EXIT INT TERM
    echo "PC-only teleop: starting follower on ${BIND}:${PORT} ..."
    leneo_run_python examples/leneo/leneo_remote_server.py \
      --robot.port="${ROBOT_PORT}" \
      --robot.id="${ROBOT_ID}" \
      "${LIFT_ARGS[@]}" \
      --port.retry_interval="${PORT_RETRY}" \
      --bind="${BIND}" \
      --port="${PORT}" &
    SERVER_PID=$!
    for _ in $(seq 1 60); do
      if (echo >/dev/tcp/127.0.0.1/"${PORT}") 2>/dev/null; then
        break
      fi
      if ! kill -0 "${SERVER_PID}" 2>/dev/null; then
        echo "Follower server exited during startup." >&2
        exit 1
      fi
      sleep 0.5
    done
    if ! (echo >/dev/tcp/127.0.0.1/"${PORT}") 2>/dev/null; then
      echo "Timed out waiting for port ${PORT}" >&2
      exit 1
    fi
    echo "Follower ready. Starting leader client (Ctrl+C stops both) ..."
    ROLE=client HOST=127.0.0.1 "${SCRIPT_DIR}/run_leneo_remote.sh" client "$@"
    ;;
  *)
    echo "Unknown ROLE=${ROLE} (use server, client, keyboard, or local)" >&2
    exit 1
    ;;
esac
