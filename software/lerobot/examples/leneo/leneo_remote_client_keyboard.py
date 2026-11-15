#!/usr/bin/env python
"""PC: keyboard only → remote LeNeo base + lift (no leader arm).

Base: i/k/u/o, speed n/m. Lift: r up / f down. Focus this terminal.
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
from lerobot.utils.robot_utils import precise_sleep

from leneo_remote_client import merge_action
from leneo_remote_protocol import DEFAULT_REMOTE_PORT, send_action

logger = logging.getLogger(__name__)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="LeNeo remote keyboard (base + lift only)")
    p.add_argument("--host", default="192.168.124.32", help="Board IP")
    p.add_argument("--port", type=int, default=DEFAULT_REMOTE_PORT)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--lift.speed_rpm", dest="lift_speed_rpm", type=float, default=500.0)
    p.add_argument("--lift.key_up", dest="lift_key_up", default="r")
    p.add_argument("--lift.key_down", dest="lift_key_down", default="f")
    return p.parse_args()


def main() -> int:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        force=True,
    )
    args = parse_args()
    print(f"LeNeo 远程键盘客户端 → {args.host}:{args.port}（Ctrl+C 退出）", flush=True)

    base_helper = Leneo(LeneoConfig(id="_keyboard_helper", port="/dev/null"))
    keyboard = KeyboardTeleop(KeyboardTeleopConfig(id="remote_keyboard"))
    keys = base_helper.teleop_keys
    logger.info(
        "Keyboard base: %s/%s move, %s/%s turn, %s/%s speed | lift %s/%s",
        keys["forward"],
        keys["backward"],
        keys["rotate_left"],
        keys["rotate_right"],
        keys["speed_up"],
        keys["speed_down"],
        args.lift_key_up,
        args.lift_key_down,
    )

    keyboard.connect()
    if not keyboard.is_connected:
        print(
            "警告: 键盘监听未启动（Wayland/无图形界面时常见）。请在本机 X11 终端运行，或 SSH 加 -X。",
            flush=True,
        )

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(8.0)
    logger.info("Connecting to %s:%s ...", args.host, args.port)
    try:
        sock.connect((args.host, args.port))
    except (ConnectionRefusedError, TimeoutError, OSError) as exc:
        print(
            f"连不上板子 {args.host}:{args.port} ({exc}).\n"
            "请在板子上先运行: ~/ai/leneo/run_leneo_remote_board.sh",
            flush=True,
        )
        keyboard.disconnect()
        return 1
    sock.settimeout(None)
    print("已连接。请用本窗口按键：i/k/u/o 小车，r/f 升降，n/m 调速。", flush=True)

    period = 1.0 / args.fps
    last_status = 0.0
    try:
        while True:
            t0 = time.perf_counter()
            kb = keyboard.get_action()
            action = merge_action(
                {},
                kb,
                base_helper,
                lift_speed_rpm=args.lift_speed_rpm,
                lift_key_up=args.lift_key_up,
                lift_key_down=args.lift_key_down,
            )
            send_action(sock, action)
            now = time.perf_counter()
            if now - last_status > 2.0:
                pressed = [k for k, v in kb.items() if v]
                xv = action.get("x.vel", 0.0)
                tv = action.get("theta.vel", 0.0)
                lift = action.get("lift.rpm", 0.0)
                print(
                    f"  心跳 | 按键={pressed or '-'} | x={xv:.2f} theta={tv:.0f} lift_rpm={lift:.0f}",
                    flush=True,
                )
                last_status = now
            precise_sleep(max(period - (time.perf_counter() - t0), 0.0))
    except KeyboardInterrupt:
        logger.info("Stopping (Ctrl+C)")
    finally:
        try:
            send_action(sock, {"x.vel": 0.0, "theta.vel": 0.0, "lift.rpm": 0.0})
        except Exception:
            pass
        sock.close()
        keyboard.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
