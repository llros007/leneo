#!/usr/bin/env python
"""SO101 leader arm + keyboard → remote LeNeo follower (PC or board).

Run the client on the machine where the **leader USB** is plugged in.
Use ``--host=127.0.0.1`` when server and leader are both on the board.

Example::

    ROLE=client HOST=192.168.124.34 ./examples/leneo/run_leneo_remote.sh
    ROLE=client HOST=127.0.0.1 ./examples/leneo/run_leneo_remote.sh   # leader on board
"""

from __future__ import annotations

import argparse
import logging
import socket
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from lerobot.robots.leneo import Leneo, LeneoConfig
from lerobot.teleoperators.keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.teleoperators.so_leader import SO101Leader
from lerobot.teleoperators.so_leader.config_so_leader import SOLeaderTeleopConfig
from lerobot.utils.robot_utils import precise_sleep

from find_leader_port import wait_for_leader_port
from leneo_remote_protocol import DEFAULT_REMOTE_PORT, send_action

logger = logging.getLogger(__name__)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="LeNeo remote teleop (PC: leader + keyboard)")
    p.add_argument("--host", default="192.168.124.32", help="Board IP")
    p.add_argument("--port", type=int, default=DEFAULT_REMOTE_PORT)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument(
        "--teleop.port",
        dest="teleop_port",
        default="auto",
        help="Leader USB serial, or 'auto' (Feetech IDs 1–6, no wheels 7–8)",
    )
    p.add_argument("--teleop.id", dest="teleop_id", default="7vw_leader_arm")
    p.add_argument("--lift.speed_rpm", dest="lift_speed_rpm", type=float, default=500.0)
    p.add_argument("--lift.key_up", dest="lift_key_up", default="r")
    p.add_argument("--lift.key_down", dest="lift_key_down", default="f")
    return p.parse_args()


def merge_action(
    teleop_action: dict[str, float],
    keyboard_keys: dict,
    base_helper: Leneo,
    *,
    lift_speed_rpm: float,
    lift_key_up: str,
    lift_key_down: str,
) -> dict[str, float]:
    out = dict(teleop_action)
    base = base_helper._from_keyboard_to_base_action(keyboard_keys)
    out.update(base)
    lift_rpm = 0.0
    if lift_key_up in keyboard_keys:
        lift_rpm = abs(lift_speed_rpm)
    elif lift_key_down in keyboard_keys:
        lift_rpm = -abs(lift_speed_rpm)
    out["lift.rpm"] = lift_rpm
    return out


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    args = parse_args()

    teleop_port = args.teleop_port
    if teleop_port in ("auto", ""):
        teleop_port = wait_for_leader_port(interval=2.0)
        logger.info("Leader arm on %s", teleop_port)

    teleop_cfg = SOLeaderTeleopConfig(id=args.teleop_id, port=teleop_port)
    teleop = SO101Leader(teleop_cfg)

    # Not connected: only keyboard → base velocity mapping (speed levels).
    base_helper = Leneo(LeneoConfig(id="_keyboard_helper", port="/dev/null"))

    keyboard = KeyboardTeleop(KeyboardTeleopConfig(id=f"{args.teleop_id}_keyboard"))
    keys = base_helper.teleop_keys
    logger.info(
        "Keyboard base (focus this terminal): forward=%s backward=%s left=%s right=%s speed+=%s speed-=%s",
        keys["forward"],
        keys["backward"],
        keys["rotate_left"],
        keys["rotate_right"],
        keys["speed_up"],
        keys["speed_down"],
    )
    logger.info("Keyboard lift: %s=up %s=down (%.0f rpm on board)", args.lift_key_up, args.lift_key_down, args.lift_speed_rpm)

    logger.info("Connecting leader arm on %s (may take ~15s; close other programs using this USB)...", teleop_port)
    teleop.connect()
    keyboard.connect()

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    logger.info("Connecting to %s:%s ...", args.host, args.port)
    try:
        sock.connect((args.host, args.port))
    except OSError as exc:
        logger.error("Cannot reach board %s:%s — %s", args.host, args.port, exc)
        raise
    sock.settimeout(None)
    logger.info("TCP connected. Waiting for board follower/lift to attach...")
    logger.info("If the link drops immediately, on the board run: journalctl -u leneo-remote -n 30")

    period = 1.0 / args.fps
    try:
        while True:
            t0 = time.perf_counter()
            raw = teleop.get_action()
            kb = keyboard.get_action()
            action = merge_action(
                raw,
                kb,
                base_helper,
                lift_speed_rpm=args.lift_speed_rpm,
                lift_key_up=args.lift_key_up,
                lift_key_down=args.lift_key_down,
            )
            send_action(sock, action)
            dt = time.perf_counter() - t0
            precise_sleep(max(period - dt, 0.0))
    except ConnectionResetError:
        logger.error(
            "Board closed the connection (follower/lift init failed?). "
            "SSH to the board: journalctl -u leneo-remote -n 40"
        )
        raise
    except KeyboardInterrupt:
        logger.info("Stopping (Ctrl+C)")
    finally:
        try:
            send_action(sock, {"x.vel": 0.0, "theta.vel": 0.0, "lift.rpm": 0.0})
        except Exception:
            pass
        sock.close()
        keyboard.disconnect()
        teleop.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
