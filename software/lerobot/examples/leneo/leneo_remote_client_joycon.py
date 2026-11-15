#!/usr/bin/env python
"""PC: right Joy-Con → remote LeNeo on the board (TCP 9595).

Same buttons as ``leneo_joycon_teleop.py``, but the follower and lift stay on the board.

  SL → MOVE: X forward, B back, Y left, A right, R up / ZR down (lift)
  SR → ARM: stick end-effector, R/ZR height, Y/A gripper
  Plus → reset arm targets to 0°

Example::

    conda activate le312
    HOST=192.168.124.35 ./examples/leneo/run_leneo_remote_joycon_pc.sh
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
from lerobot.utils.robot_utils import precise_sleep

from leneo_joycon_teleop import (
    OfficialTeleopArm,
    SO101Kinematics,
    _apply_fit_to_joy,
    _base_action_move_mode,
    _import_joycon_robotics,
    _joycon_available_hid_or_evdev,
    connect_joycon,
)
from leneo_remote_protocol import DEFAULT_REMOTE_PORT, send_action

logger = logging.getLogger(__name__)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="LeNeo remote Joy-Con (PC → board)")
    p.add_argument("--host", default="192.168.124.35", help="Board IP")
    p.add_argument("--port", type=int, default=DEFAULT_REMOTE_PORT)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--joycon.side", dest="side", default="right", choices=("right", "left"))
    p.add_argument("--lift.speed_rpm", dest="lift_speed_rpm", type=float, default=500.0)
    p.add_argument("--arm.grip_step", dest="arm_grip_step", type=float, default=0.54)
    p.add_argument("--arm.z_step", dest="arm_z_step", type=float, default=0.0024)
    return p.parse_args()


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s", force=True)
    args = parse_args()
    print(f"LeNeo 远程 Joy-Con → {args.host}:{args.port}（Ctrl+C 退出）", flush=True)

    _import_joycon_robotics(None)
    _joycon_available_hid_or_evdev(args.side)
    base_helper = Leneo(LeneoConfig(id="_joycon_helper", port="/dev/null"))
    arm = OfficialTeleopArm(SO101Kinematics())

    logger.info("Connecting %s Joy-Con ...", args.side)
    joy = connect_joycon(args.side, dof_speed=[2, 2, 2, 1, 1, 1])
    joy.enable_arm_stick = False
    input_mode = "move"
    sr_prev = sl_prev = 0

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(8.0)
    logger.info("Connecting to %s:%s ...", args.host, args.port)
    try:
        sock.connect((args.host, args.port))
    except (ConnectionRefusedError, TimeoutError, OSError) as exc:
        print(f"连不上板子 {args.host}:{args.port} ({exc})", flush=True)
        try:
            joy.disconnect()
        except Exception:
            pass
        return 1
    sock.settimeout(None)
    print(
        "已连接。SL=底盘(X前 B后 Y左 A右，R升 ZR降)；SR=机械臂(摇杆+R/ZR高度，Y/A夹爪)；Plus 回零。",
        flush=True,
    )

    period = 1.0 / args.fps
    last_status = 0.0
    try:
        while True:
            t0 = time.perf_counter()
            _, grip, btn = joy.get_control()
            jr = joy.joycon
            sr = jr.get_button_right_sr() if args.side == "right" else 0
            sl = jr.get_button_right_sl() if args.side == "right" else 0
            if sl == 1 and sl_prev == 0:
                input_mode = "move"
                joy.enable_arm_stick = False
                logger.info("MODE: MOVE")
            if sr == 1 and sr_prev == 0:
                input_mode = "arm"
                joy.enable_arm_stick = True
                logger.info("MODE: ARM")
            sr_prev, sl_prev = sr, sl

            action: dict[str, float] = {"x.vel": 0.0, "theta.vel": 0.0, "lift.rpm": 0.0}
            if btn == 8:
                arm.zero()
                joy.position = joy.offset_position_m.copy()
                joy._twist_rad = 0.0
                action.update(arm.as_leneo_action())
                logger.info("Plus: arm targets reset")
            elif input_mode == "arm":
                now = time.perf_counter()
                dt = now - getattr(joy, "_twist_t", now)
                joy._twist_t = now
                gz = float(joy.gyro.gyro_in_rad[0][2])
                joy._twist_rad = getattr(joy, "_twist_rad", 0.0) + gz * dt
                orientation = list(joy.orientation_rad)
                orientation[2] = joy._twist_rad
                joy.orientation_rad = orientation
                if jr.get_button_r() == 1:
                    joy.position[2] += args.arm_z_step
                if jr.get_button_zr() == 1:
                    joy.position[2] -= args.arm_z_step
                _apply_fit_to_joy(joy, prefer="y")
                pose = list(joy.position) + list(joy.orientation_rad)
                arm.handle_joycon_input(pose, grip)
                g = arm.target_deg["gripper"]
                if jr.get_button_y() == 1:
                    g = min(90.0, g + args.arm_grip_step)
                if jr.get_button_a() == 1:
                    g = max(0.0, g - args.arm_grip_step)
                arm.target_deg["gripper"] = g
                joy.gripper_state = g
                action.update(arm.as_leneo_action())
            else:
                action.update(_base_action_move_mode(joy, base_helper))
                rpm = 0.0
                if jr.get_button_r() == 1 and jr.get_button_zr() != 1:
                    rpm = abs(args.lift_speed_rpm)
                elif jr.get_button_zr() == 1 and jr.get_button_r() != 1:
                    rpm = -abs(args.lift_speed_rpm)
                action["lift.rpm"] = rpm

            send_action(sock, action)
            now = time.perf_counter()
            if now - last_status > 2.0:
                print(
                    f"  心跳 | mode={input_mode} x={action.get('x.vel', 0):.2f} "
                    f"theta={action.get('theta.vel', 0):.0f} lift={action.get('lift.rpm', 0):.0f}",
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
        try:
            joy.disconnect()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
