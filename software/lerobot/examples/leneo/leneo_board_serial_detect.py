"""Detect LeNeo Feetech bus vs JH_FOC lift by motor IDs / serial fingerprint.

Used by ``leneo_remote_server.py`` when ``--robot.port=auto`` / ``--lift.port=auto``.
Retries every few seconds if USB is not ready at boot; re-scans on each client connect.
"""

from __future__ import annotations

import argparse
import glob
import json
import logging
import re
import time
from dataclasses import asdict, dataclass
from pathlib import Path

logger = logging.getLogger(__name__)

LENEO_FEETECH_IDS = frozenset(range(1, 9))
WHEEL_IDS = frozenset((7, 8))
FEETECH_BAUD = 1_000_000
JH_BAUD = 115200

TELEMETRY_RE = re.compile(
    r"s:([-+]?\d+(?:\.\d+)?)p:([-+]?\d+(?:\.\d+)?)",
    re.I,
)

_CACHE_PATH = Path.home() / ".cache/huggingface/lerobot/leneo/board_serial.json"


@dataclass(frozen=True)
class PortMap:
    robot_port: str | None
    lift_port: str | None
    robot_ids: tuple[int, ...] = ()


def list_candidate_ports() -> list[str]:
    ports = sorted(set(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")))
    hint = _load_cache()
    if hint:
        preferred = [hint.get("robot_port"), hint.get("lift_port")]
        ordered = [p for p in preferred if p in ports]
        ordered += [p for p in ports if p not in ordered]
        return ordered
    return ports


def probe_leneo_feetech(port: str) -> frozenset[int]:
    """Return responding motor IDs 1–8 on the LeNeo bus (1 Mbps), or empty."""
    try:
        from lerobot.motors.feetech import FeetechMotorsBus
    except ImportError:
        logger.warning("Feetech not available — cannot probe %s", port)
        return frozenset()

    try:
        bus = FeetechMotorsBus(port, {})
        bus._connect(handshake=False)
        bus.set_baudrate(FEETECH_BAUD)
        found = bus.broadcast_ping(num_retry=1, raise_on_error=False)
        bus.port_handler.closePort()
    except Exception as exc:
        logger.debug("Feetech probe %s: %s", port, exc)
        return frozenset()

    if not found:
        return frozenset()
    ids = frozenset(found.keys())
    # Follower bus: arm 1–6 plus wheel motors 7–8 (distinguish from leader-only USB).
    if ids & WHEEL_IDS or (ids & LENEO_FEETECH_IDS and len(ids) >= 6):
        return ids
    return frozenset()


def probe_jh_foc(port: str) -> bool:
    """True if port looks like a JH_FOC driver (115200), not the Feetech arm bus."""
    try:
        import serial
    except ImportError:
        return False

    try:
        ser = serial.Serial(port, JH_BAUD, timeout=0.3)
        time.sleep(0.25)
        ser.reset_input_buffer()
        ser.write(b"\r\n")
        time.sleep(0.15)
        chunk = ser.read(512)
        if not chunk:
            time.sleep(0.4)
            chunk = ser.read(512)
        ser.close()
    except OSError as exc:
        logger.debug("JH_FOC probe %s: %s", port, exc)
        return False

    if not chunk:
        return _probe_jh_foc_handshake(port)
    text = chunk.decode("utf-8", errors="ignore")
    if TELEMETRY_RE.search(text):
        return True
    if "驱动" in text or "JH" in text or "Protect" in text or "Limit" in text:
        return True
    if "speed" in text.lower() or "encoder" in text.lower():
        return True
    # JH boards often print a short banner; avoid treating random binary as lift.
    if len(text.strip()) >= 4 and sum(c.isprintable() or c in "\r\n\t" for c in text) / max(len(text), 1) > 0.85:
        return "电机" in text or "Foc" in text or "FOC" in text
    return False


def _probe_jh_foc_handshake(port: str) -> bool:
    """JH boards are often silent until speed mode; open briefly like the lift driver."""
    motor_dir = Path.home() / "ai" / "leneo" / "motor"
    if not (motor_dir / "motor_keyboard_control.py").is_file():
        return False
    try:
        import sys

        if str(motor_dir) not in sys.path:
            sys.path.insert(0, str(motor_dir))
        from motor_keyboard_control import JHFocMotor

        motor = JHFocMotor(port)
        time.sleep(0.35)
        motor.enter_speed_mode(current=2.0, accel=500.0)
        motor.set_speed(0)
        motor.close()
        return True
    except Exception as exc:
        logger.debug("JH_FOC handshake %s: %s", port, exc)
        return False


def detect_ports() -> PortMap:
    ports = list_candidate_ports()
    robot_port: str | None = None
    lift_port: str | None = None
    robot_ids: tuple[int, ...] = ()

    for port in ports:
        ids = probe_leneo_feetech(port)
        if ids:
            robot_port = port
            robot_ids = tuple(sorted(ids))
            logger.info("LeNeo Feetech on %s — motor IDs %s", port, list(robot_ids))
            break

    for port in ports:
        if port == robot_port:
            continue
        if probe_jh_foc(port):
            lift_port = port
            logger.info("JH_FOC lift on %s", port)
            break

    if robot_port is None and len(ports) == 1 and lift_port is None:
        if probe_jh_foc(ports[0]):
            lift_port = ports[0]
            logger.info("Single USB device: JH_FOC lift on %s (no arm bus)", ports[0])

    if robot_port is not None or lift_port is not None:
        _save_cache(robot_port, lift_port, robot_ids)

    return PortMap(robot_port=robot_port, lift_port=lift_port, robot_ids=robot_ids)


def wait_for_ports(
    *,
    need_robot: bool = True,
    want_lift: bool = False,
    interval: float = 2.0,
    timeout: float | None = None,
) -> PortMap:
    """Poll until required devices appear (``interval`` seconds between tries)."""
    deadline = time.monotonic() + timeout if timeout is not None else None
    attempt = 0
    while True:
        attempt += 1
        mapping = detect_ports()
        robot_ok = not need_robot or mapping.robot_port is not None
        # Lift is optional: do not block forever when only the arm USB is plugged.
        if robot_ok:
            return mapping
        if not need_robot and want_lift and mapping.lift_port is not None:
            return mapping
        if deadline is not None and time.monotonic() >= deadline:
            logger.warning(
                "Port wait timed out after %.0fs (robot=%s lift=%s)",
                timeout,
                mapping.robot_port,
                mapping.lift_port,
            )
            return mapping
        logger.info(
            "USB scan #%s: robot=%s lift=%s — retry in %.0fs",
            attempt,
            mapping.robot_port or "—",
            mapping.lift_port or "—",
            interval,
        )
        time.sleep(interval)


def resolve_ports(
    robot_port: str,
    lift_port: str,
    *,
    lift_enabled: bool,
    retry_interval: float = 2.0,
    wait_timeout: float | None = None,
) -> PortMap:
    """Resolve ``auto`` ports; fixed paths are passed through."""
    auto_robot = robot_port in ("auto", "")
    auto_lift = lift_port in ("auto", "")

    if not auto_robot and not auto_lift:
        return PortMap(
            robot_port=robot_port,
            lift_port=lift_port if lift_enabled else None,
        )

    need_robot = auto_robot
    want_lift = lift_enabled and auto_lift
    if need_robot or want_lift:
        mapping = wait_for_ports(
            need_robot=need_robot,
            want_lift=want_lift,
            interval=retry_interval,
            timeout=wait_timeout if need_robot else 0.0,
        )
    else:
        mapping = detect_ports()

    robot = mapping.robot_port if auto_robot else robot_port
    if auto_lift:
        lift = mapping.lift_port if lift_enabled else None
    else:
        lift = lift_port if lift_enabled else None

    if lift_enabled and lift is None and auto_lift:
        logger.info("No JH_FOC lift found — running arm only")

    return PortMap(robot_port=robot, lift_port=lift, robot_ids=mapping.robot_ids)


def _load_cache() -> dict | None:
    try:
        if _CACHE_PATH.is_file():
            return json.loads(_CACHE_PATH.read_text())
    except (json.JSONDecodeError, OSError):
        pass
    return None


def _save_cache(robot_port: str | None, lift_port: str | None, robot_ids: tuple[int, ...]) -> None:
    try:
        _CACHE_PATH.parent.mkdir(parents=True, exist_ok=True)
        _CACHE_PATH.write_text(
            json.dumps(
                {
                    "robot_port": robot_port,
                    "lift_port": lift_port,
                    "robot_ids": list(robot_ids),
                    "updated_at": time.time(),
                },
                indent=2,
            )
        )
    except OSError as exc:
        logger.debug("Could not write port cache: %s", exc)


def _export_shell(mapping: PortMap, lift_enabled: bool) -> None:
    robot = mapping.robot_port or ""
    lift = mapping.lift_port or ""
    en = "true" if lift_enabled and mapping.lift_port else "false"
    print(f'ROBOT_PORT="{robot}"')
    print(f'LIFT_PORT="{lift}"')
    print(f'LIFT_ENABLED="{en}"')
    if mapping.robot_ids:
        print(f'ROBOT_IDS="{",".join(str(i) for i in mapping.robot_ids)}"')


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    p = argparse.ArgumentParser(description="Detect LeNeo arm vs JH_FOC lift serial ports")
    p.add_argument("--robot.port", dest="robot_port", default="auto")
    p.add_argument("--lift.port", dest="lift_port", default="auto")
    p.add_argument("--lift.enabled", dest="lift_enabled", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--retry-interval", type=float, default=2.0)
    p.add_argument("--timeout", type=float, default=None, help="Max seconds to wait (default: infinite)")
    p.add_argument("--once", action="store_true", help="Single detect_ports() without wait loop")
    p.add_argument("--export-shell", action="store_true", help="Print ROBOT_PORT= / LIFT_PORT= for bash eval")
    p.add_argument("--json", action="store_true", help="Print PortMap as JSON")
    args = p.parse_args()

    if args.once:
        mapping = detect_ports()
    else:
        mapping = resolve_ports(
            args.robot_port,
            args.lift_port,
            lift_enabled=args.lift_enabled,
            retry_interval=args.retry_interval,
            wait_timeout=args.timeout,
        )

    if args.export_shell:
        _export_shell(mapping, args.lift_enabled)
    elif args.json:
        print(json.dumps(asdict(mapping)))
    else:
        print(f"robot={mapping.robot_port} ids={list(mapping.robot_ids)} lift={mapping.lift_port}")

    if args.robot_port in ("auto", "") and mapping.robot_port is None:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
