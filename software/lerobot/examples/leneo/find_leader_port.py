#!/usr/bin/env python
"""Find SO101 leader bus (Feetech IDs 1–6, no wheel IDs 7–8).

Usage::

    PYTHONPATH=src python examples/leneo/find_leader_port.py
    PYTHONPATH=src python examples/leneo/find_leader_port.py --wait --retry-interval 2
"""

from __future__ import annotations

import argparse
import glob
import logging
import sys
import time

from lerobot.motors.feetech import FeetechMotorsBus

logger = logging.getLogger(__name__)

LEADER_IDS = frozenset(range(1, 7))
WHEEL_IDS = frozenset((7, 8))


def list_ports() -> list[str]:
    return sorted(set(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")))


def probe_port_ids(port: str) -> frozenset[int]:
    try:
        bus = FeetechMotorsBus(port, {})
        bus._connect(handshake=False)
        bus.set_baudrate(1_000_000)
        found = bus.broadcast_ping(num_retry=1, raise_on_error=False)
        bus.port_handler.closePort()
    except Exception:
        return frozenset()
    if not found:
        return frozenset()
    return frozenset(found.keys())


def is_leader_bus(ids: frozenset[int]) -> bool:
    if not ids:
        return False
    if ids & WHEEL_IDS:
        return False
    return ids >= LEADER_IDS or len(ids) >= 4


def find_leader_port() -> str | None:
    ports = list_ports()
    hits: list[str] = []
    for port in ports:
        ids = probe_port_ids(port)
        if is_leader_bus(ids):
            hits.append(port)
    if not hits:
        return None
    if len(hits) > 1:
        logger.warning("Multiple leader-like ports %s — using %s", hits, hits[0])
    return hits[0]


def wait_for_leader_port(interval: float = 2.0, timeout: float | None = None) -> str:
    deadline = time.monotonic() + timeout if timeout is not None else None
    attempt = 0
    while True:
        attempt += 1
        port = find_leader_port()
        if port:
            return port
        ports = list_ports()
        if deadline is not None and time.monotonic() >= deadline:
            raise RuntimeError(f"No leader port after {timeout}s (seen: {ports})")
        logger.info(
            "Leader USB not ready (ports=%s) — retry in %.0fs (#%s)",
            ports or "none",
            interval,
            attempt,
        )
        time.sleep(interval)


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    p = argparse.ArgumentParser(description="Find SO101 leader serial port")
    p.add_argument("--wait", action="store_true", help="Retry until leader USB appears")
    p.add_argument("--retry-interval", type=float, default=2.0)
    p.add_argument("--timeout", type=float, default=None)
    args = p.parse_args()

    ports = list_ports()
    if ports:
        print("Checking:", ", ".join(ports), file=sys.stderr)
    if args.wait:
        try:
            print(wait_for_leader_port(interval=args.retry_interval, timeout=args.timeout))
            return 0
        except RuntimeError as exc:
            print(str(exc), file=sys.stderr)
            return 1

    port = find_leader_port()
    if port:
        print(port)
        return 0
    print(
        "No leader port (Feetech 1–6, no wheels 7–8). Plug USB or use --wait.",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
