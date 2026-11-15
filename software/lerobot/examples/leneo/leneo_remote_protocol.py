# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
"""Newline-delimited JSON actions for LeNeo remote teleop."""

from __future__ import annotations

import json
import socket
from typing import Any

DEFAULT_REMOTE_PORT = 9595


def encode_action(action: dict[str, float]) -> bytes:
    payload = {k: float(v) for k, v in action.items()}
    return (json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8")


def send_action(sock: socket.socket, action: dict[str, float]) -> None:
    sock.sendall(encode_action(action))


def recv_action_line(buffer: bytearray) -> tuple[dict[str, float] | None, bytearray]:
    while True:
        nl = buffer.find(b"\n")
        if nl < 0:
            return None, buffer
        line = bytes(buffer[:nl])
        del buffer[: nl + 1]
        if not line.strip():
            continue
        data: dict[str, Any] = json.loads(line.decode("utf-8"))
        return {k: float(v) for k, v in data.items()}, buffer
