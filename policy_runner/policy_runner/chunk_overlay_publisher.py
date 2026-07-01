from __future__ import annotations

import json
import socket
from typing import Callable

from .robot_state_client import parse_udp_endpoint


CHUNK_OVERLAY_SCHEMA_VERSION = "robotics_lab.chunk_overlay.v1"


class ChunkOverlayPublisher:
    """Best-effort UDP publisher for predicted policy action chunks."""

    def __init__(
        self,
        endpoint: str,
        socket_factory: Callable[..., socket.socket] = socket.socket,
    ) -> None:
        self.endpoint = endpoint
        self._socket = socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setblocking(False)
        parsed = parse_udp_endpoint(endpoint)
        self._address = (parsed.host, parsed.port)

    def publish(
        self,
        *,
        seq: int,
        policy_dt_sec: float,
        left: list[list[float]] | None,
        right: list[list[float]] | None,
        host_time_ns: int,
    ) -> None:
        try:
            horizon = len(left) if left is not None else len(right or [])
            packet = {
                "schema_version": CHUNK_OVERLAY_SCHEMA_VERSION,
                "host_time_ns": int(host_time_ns),
                "seq": int(seq),
                "policy_dt_sec": float(policy_dt_sec),
                "horizon": int(horizon),
                "left": left,
                "right": right,
            }
            data = json.dumps(packet, separators=(",", ":")).encode("utf-8")
            self._socket.sendto(data, self._address)
        except (BlockingIOError, OSError, TypeError, ValueError):
            return

    def close(self) -> None:
        self._socket.close()
