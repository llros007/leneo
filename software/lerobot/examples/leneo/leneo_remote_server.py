#!/usr/bin/env python
"""LeNeo follower server: TCP actions → local arm, base, and optional lift.

Usually on the board; same script on a PC if the follower hardware is there.

Example::

    ROLE=server ./examples/leneo/run_leneo_remote.sh
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

from jh_foc_lift import LiftController

from leneo_board_serial_detect import resolve_ports
from leneo_remote_protocol import DEFAULT_REMOTE_PORT, recv_action_line

logger = logging.getLogger(__name__)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="LeNeo remote follower (board side)")
    p.add_argument(
        "--robot.port",
        dest="robot_port",
        default="auto",
        help="Feetech bus device, or 'auto' (scan IDs 1–8 @ 1 Mbps)",
    )
    p.add_argument("--robot.id", dest="robot_id", default="blue_follower_arm")
    p.add_argument("--bind", default="0.0.0.0", help="Listen address")
    p.add_argument("--port", type=int, default=DEFAULT_REMOTE_PORT)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument(
        "--lift.port",
        dest="lift_port",
        default="auto",
        help="JH_FOC lift serial, or 'auto' (115200 fingerprint)",
    )
    p.add_argument("--lift.enabled", dest="lift_enabled", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--lift.speed_rpm", dest="lift_speed_rpm", type=float, default=500.0)
    p.add_argument("--lift.motor", dest="lift_motor", default="42", help="JH_FOC preset: 42 or 60")
    p.add_argument(
        "--port.retry_interval",
        dest="port_retry_interval",
        type=float,
        default=2.0,
        help="Seconds between USB re-scans when port is auto",
    )
    p.add_argument(
        "--port.wait_timeout",
        dest="port_wait_timeout",
        type=float,
        default=None,
        help="Max seconds to wait for auto ports on connect (default: infinite)",
    )
    return p.parse_args()


def _apply_lift(lift: LiftController | None, action: dict[str, float]) -> tuple[dict[str, float], LiftController | None]:
    rpm = action.pop("lift.rpm", None)
    if lift is None or rpm is None:
        return action, lift
    rpm = float(rpm)
    try:
        lift.update(up=rpm > 0, down=rpm < 0, active=True, speed_rpm=abs(rpm))
    except Exception as exc:
        logger.warning("Lift command failed, disabling lift: %s", exc)
        try:
            lift.close()
        except Exception:
            pass
        return action, None
    return action, lift


def serve_client(
    robot: Leneo,
    conn: socket.socket,
    fps: int,
    lift: LiftController | None,
) -> None:
    conn.settimeout(1.0)
    buf = bytearray()
    period = 1.0 / fps
    last_action: dict[str, float] | None = None
    try:
        while True:
            loop_start = time.perf_counter()
            try:
                chunk = conn.recv(4096)
                if not chunk:
                    logger.info("Client closed connection")
                    break
                buf.extend(chunk)
                while True:
                    action, buf = recv_action_line(buf)
                    if action is None:
                        break
                    last_action = action
            except TimeoutError:
                pass
            except socket.timeout:
                pass

            if last_action is not None:
                act, lift = _apply_lift(lift, dict(last_action))
                robot.send_action(act)

            dt = time.perf_counter() - loop_start
            precise_sleep(max(period - dt, 0.0))
    finally:
        if lift is not None:
            lift.stop()
        robot.stop_base()


def _set_robot_serial_port(robot: Leneo, port: str) -> None:
    """``Leneo`` builds ``FeetechMotorsBus`` at init — update bus when auto-detect picks another tty."""
    if robot.config.port == port and robot.bus.port == port:
        return
    if robot.is_connected:
        robot.disconnect()
    robot.config.port = port
    robot.bus.port = port
    if hasattr(robot.bus, "port_handler"):
        robot.bus.port_handler.setPortName(port)


def _apply_port_map(robot: Leneo, args: argparse.Namespace) -> None:
    mapping = resolve_ports(
        args._robot_port_spec,
        args._lift_port_spec,
        lift_enabled=args.lift_enabled,
        retry_interval=args.port_retry_interval,
        wait_timeout=args.port_wait_timeout,
    )
    if mapping.robot_port is None:
        raise RuntimeError("No LeNeo Feetech bus found (expected motor IDs 1–8)")
    if robot.is_connected and robot.config.port != mapping.robot_port:
        logger.warning(
            "LeNeo bus moved %s -> %s; disconnect and reconnect client after USB change",
            robot.config.port,
            mapping.robot_port,
        )
    _set_robot_serial_port(robot, mapping.robot_port)
    args.robot_port = mapping.robot_port
    if mapping.lift_port:
        args.lift_port = mapping.lift_port
    elif args._lift_port_spec in ("auto", ""):
        args.lift_port = ""
        logger.info("No JH_FOC lift USB — teleop arm/base only (r/f ignored)")
    if mapping.robot_ids:
        logger.info("Arm bus motor IDs: %s", list(mapping.robot_ids))


def _connect_hardware(
    robot: Leneo,
    args: argparse.Namespace,
    lift: LiftController | None,
) -> LiftController | None:
    if args._robot_port_spec == "auto" or args._lift_port_spec == "auto":
        _apply_port_map(robot, args)

    if robot.is_connected:
        try:
            robot.bus.sync_read("Present_Position", robot.arm_motors[:1], num_retry=0)
        except Exception:
            logger.warning("Follower bus lost — reconnecting")
            robot.disconnect()

    if not robot.is_connected:
        logger.info("Connecting LeNeo on %s ...", args.robot_port)
        # Headless (systemd): never block on input() calibration prompts.
        robot.connect(calibrate=False)
        logger.info("LeNeo follower connected on %s", args.robot_port)
    if args.lift_enabled and lift is None and args.lift_port and args.lift_port not in ("auto", ""):
        try:
            logger.info("JH_FOC lift on %s (%.0f rpm)", args.lift_port, args.lift_speed_rpm)
            lift = LiftController(
                port=args.lift_port,
                motor_type=args.lift_motor,
                speed_rpm=args.lift_speed_rpm,
            )
        except Exception as exc:
            logger.warning("Lift disabled: %s", exc)
    return lift


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    args = parse_args()
    args._robot_port_spec = args.robot_port
    args._lift_port_spec = args.lift_port
    boot_port = args.robot_port if args.robot_port != "auto" else "/dev/ttyACM0"
    cfg = LeneoConfig(id=args.robot_id, port=boot_port)
    robot = Leneo(cfg)
    lift: LiftController | None = None

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.bind, args.port))
    server.listen(1)
    robot_desc = args._robot_port_spec if args._robot_port_spec == "auto" else args.robot_port
    if args._robot_port_spec == "auto":
        robot_desc = "auto (Feetech IDs 1–8)"
    lift_desc = (
        "off"
        if not args.lift_enabled
        else ("auto (JH_FOC)" if args._lift_port_spec == "auto" else args.lift_port)
    )
    logger.info(
        "Listening on %s:%s — robot %s @ %s, lift %s (USB scan every %.0fs on connect)",
        args.bind,
        args.port,
        args.robot_id,
        robot_desc,
        lift_desc,
        args.port_retry_interval,
    )

    try:
        while True:
            conn, addr = server.accept()
            logger.info("Client connected from %s", addr)
            try:
                lift = _connect_hardware(robot, args, lift)
            except Exception as exc:
                logger.error("Hardware connect failed: %s", exc)
                conn.close()
                continue
            try:
                serve_client(robot, conn, args.fps, lift)
            finally:
                conn.close()
                if robot.is_connected:
                    robot.disconnect()
                logger.info("Waiting for next client...")
    except KeyboardInterrupt:
        logger.info("Shutting down")
    finally:
        server.close()
        if lift is not None:
            lift.close()
        if robot.is_connected:
            robot.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
