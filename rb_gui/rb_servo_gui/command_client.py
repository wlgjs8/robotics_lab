from __future__ import annotations

import json
import math
import socket
import time
from typing import Any, Mapping


class CommandClient:
    def __init__(self, host: str = "127.0.0.1", port: int = 50010) -> None:
        self.host = host
        self.port = int(port)
        self._seq = time.monotonic_ns()
        self.sent_packets: list[Mapping[str, Any]] = []

    def next_seq(self) -> int:
        self._seq += 1
        return self._seq

    @staticmethod
    def _finite_six(values: tuple[float, ...], label: str) -> list[float]:
        if len(values) != 6:
            raise ValueError(f"{label} must have 6 values")
        parsed = [float(v) for v in values]
        if any(not math.isfinite(v) for v in parsed):
            raise ValueError(f"{label} values must be finite")
        return parsed

    def build_lifecycle(self, mode: str, *, timeout_sec: float = 0.2) -> dict[str, Any]:
        return {"seq": self.next_seq(), "mode": mode, "timeout_sec": timeout_sec, "left": {}, "right": {}}

    def build_joint_target(
        self,
        left_q: tuple[float, ...],
        right_q: tuple[float, ...],
        *,
        timeout_sec: float = 0.2,
    ) -> dict[str, Any]:
        if len(left_q) != 6 or len(right_q) != 6:
            raise ValueError("joint targets must have 6 values per arm")
        return {
            "seq": self.next_seq(),
            "mode": "JointTarget",
            "timeout_sec": timeout_sec,
            "coupled_timeout": True,
            "left": {"q_target_deg": [float(v) for v in left_q]},
            "right": {"q_target_deg": [float(v) for v in right_q]},
        }

    def build_tcp_pose_target(
        self,
        *,
        left_pose: tuple[float, ...] | None = None,
        right_pose: tuple[float, ...] | None = None,
        timeout_sec: float = 0.2,
    ) -> dict[str, Any]:
        if left_pose is None and right_pose is None:
            raise ValueError("at least one TCP target is required")
        packet: dict[str, Any] = {
            "schema_version": 1,
            "seq": self.next_seq(),
            "mode": "Hold",
            "host_time_ns": time.monotonic_ns(),
            "timeout_sec": timeout_sec,
            "coupled_timeout": True,
            "left": {},
            "right": {},
        }
        if left_pose is not None:
            packet["left"] = {"mode": "TcpPoseTarget", "tcp_target_stand": self._finite_six(left_pose, "left TCP target")}
        if right_pose is not None:
            packet["right"] = {"mode": "TcpPoseTarget", "tcp_target_stand": self._finite_six(right_pose, "right TCP target")}
        return packet

    def build_tcp_delta_stand(
        self,
        *,
        left_delta: tuple[float, ...] | None = None,
        right_delta: tuple[float, ...] | None = None,
        timeout_sec: float = 0.2,
    ) -> dict[str, Any]:
        if left_delta is None and right_delta is None:
            raise ValueError("at least one TCP stand delta is required")
        packet: dict[str, Any] = {
            "schema_version": 1,
            "seq": self.next_seq(),
            "mode": "TcpDeltaStand" if left_delta is not None and right_delta is not None else "Hold",
            "host_time_ns": time.monotonic_ns(),
            "timeout_sec": timeout_sec,
            "coupled_timeout": True,
            "left": {},
            "right": {},
        }
        if left_delta is not None:
            packet["left"] = {"mode": "TcpDeltaStand", "tcp_delta_stand": self._finite_six(left_delta, "left TCP stand delta")}
        if right_delta is not None:
            packet["right"] = {"mode": "TcpDeltaStand", "tcp_delta_stand": self._finite_six(right_delta, "right TCP stand delta")}
        return packet

    def send(self, packet: Mapping[str, Any]) -> None:
        payload = json.dumps(packet, separators=(",", ":")).encode("utf-8")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.sendto(payload, (self.host, self.port))
        self.sent_packets.append(dict(packet))

    def send_lifecycle(self, mode: str, *, timeout_sec: float = 0.2) -> dict[str, Any]:
        packet = self.build_lifecycle(mode, timeout_sec=timeout_sec)
        self.send(packet)
        return packet
