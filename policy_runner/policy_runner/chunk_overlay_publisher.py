from __future__ import annotations

import json
import re
import socket
from typing import Callable

from .robot_state_client import parse_udp_endpoint


CHUNK_OVERLAY_SCHEMA_VERSION = "robotics_lab.chunk_overlay.v3"


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
        self._addresses: list[tuple[str, int]] = []
        for item in re.split(r"[\s,]+", endpoint.strip()):
            if not item:
                continue
            try:
                parsed = parse_udp_endpoint(item)
            except (TypeError, ValueError):
                continue
            self._addresses.append((parsed.host, parsed.port))
        self._address = self._addresses[0] if self._addresses else None

    def publish(
        self,
        *,
        seq: int,
        policy_dt_sec: float,
        left: list[list[float]] | None,
        right: list[list[float]] | None,
        host_time_ns: int,
        left_delta: list[list[float]] | None = None,
        right_delta: list[list[float]] | None = None,
        left_grip_cmd: list[float] | None = None,
        right_grip_cmd: list[float] | None = None,
        inference_timing: dict[str, object] | None = None,
        camera_diagnostics: dict[str, object] | None = None,
        execute_steps: int | None = None,
        runway_steps: int | None = None,
        chunk_metadata: dict[str, object] | None = None,
    ) -> None:
        try:
            horizon = len(left) if left is not None else len(right or [])
            packet = {
                "schema": CHUNK_OVERLAY_SCHEMA_VERSION,
                "schema_version": CHUNK_OVERLAY_SCHEMA_VERSION,
                "host_time_ns": int(host_time_ns),
                "seq": int(seq),
                "policy_dt_sec": float(policy_dt_sec),
                "horizon": int(horizon),
                "left": left,
                "right": right,
            }
            if left_delta is not None:
                packet["left_delta"] = left_delta
            if right_delta is not None:
                packet["right_delta"] = right_delta
            # Per-row gripper COMMAND (the runner's mapping of the raw model opening: close-bias,
            # close-snap, hold-open) - what a step-synchronised consumer (cm_bridge FollowPacer)
            # should actuate when that row's step completes. The raw model value stays in
            # column 7 of the rows/deltas for display and analysis.
            if left_grip_cmd is not None:
                packet["left_grip_cmd"] = left_grip_cmd
            if right_grip_cmd is not None:
                packet["right_grip_cmd"] = right_grip_cmd
            if inference_timing is not None:
                packet["inference_timing"] = inference_timing
            if camera_diagnostics is not None:
                packet["camera_diagnostics"] = camera_diagnostics
            if execute_steps is not None:
                packet["execute_steps"] = int(execute_steps)
            if runway_steps is not None:
                packet["runway_steps"] = int(runway_steps)
            if chunk_metadata is not None:
                packet["chunk_metadata"] = chunk_metadata
            data = json.dumps(packet, separators=(",", ":")).encode("utf-8")
        except (BlockingIOError, OSError, TypeError, ValueError):
            return
        for address in self._addresses:
            try:
                self._socket.sendto(data, address)
            except (BlockingIOError, OSError, TypeError, ValueError):
                continue

    def close(self) -> None:
        self._socket.close()
