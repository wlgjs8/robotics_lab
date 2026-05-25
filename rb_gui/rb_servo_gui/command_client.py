from __future__ import annotations

import json
import math
import socket
import time
import uuid
from typing import Any, Mapping


class CommandClient:
    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 50010,
        *,
        source_id: str = "rb_gui",
        session_id: str | None = None,
    ) -> None:
        self.host = host
        self.port = int(port)
        self.source_id = source_id
        self.session_id = session_id or uuid.uuid4().hex
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

    @staticmethod
    def _finite_quaternion_xyzw(values: tuple[float, ...], label: str) -> list[float]:
        if len(values) != 4:
            raise ValueError(f"{label} quaternion must have 4 values")
        parsed = [float(v) for v in values]
        if any(not math.isfinite(v) for v in parsed):
            raise ValueError(f"{label} quaternion values must be finite")
        norm = math.sqrt(sum(v * v for v in parsed))
        if not math.isfinite(norm) or norm <= 0.0:
            raise ValueError(f"{label} quaternion must be non-zero")
        return parsed

    def _tcp_pose_payload(
        self,
        pose: tuple[float, ...],
        *,
        quaternion_xyzw: tuple[float, ...] | None,
        label: str,
    ) -> list[float] | dict[str, Any]:
        parsed_pose = self._finite_six(pose, label)
        if quaternion_xyzw is None:
            return parsed_pose
        parsed_quaternion = self._finite_quaternion_xyzw(quaternion_xyzw, label)
        return {
            "x": parsed_pose[0],
            "y": parsed_pose[1],
            "z": parsed_pose[2],
            "rx": parsed_pose[3],
            "ry": parsed_pose[4],
            "rz": parsed_pose[5],
            "quaternion_xyzw": parsed_quaternion,
        }

    def build_lifecycle(self, mode: str, *, timeout_sec: float = 0.2) -> dict[str, Any]:
        return self._with_source({"seq": self.next_seq(), "mode": mode, "timeout_sec": timeout_sec, "left": {}, "right": {}})

    def build_joint_target(
        self,
        left_q: tuple[float, ...],
        right_q: tuple[float, ...],
        *,
        timeout_sec: float = 0.2,
    ) -> dict[str, Any]:
        if len(left_q) != 6 or len(right_q) != 6:
            raise ValueError("joint targets must have 6 values per arm")
        return self._with_source({
            "seq": self.next_seq(),
            "mode": "JointTarget",
            "timeout_sec": timeout_sec,
            "coupled_timeout": True,
            "left": {"q_target_deg": [float(v) for v in left_q]},
            "right": {"q_target_deg": [float(v) for v in right_q]},
        })

    def build_tcp_pose_target(
        self,
        *,
        left_pose: tuple[float, ...] | None = None,
        right_pose: tuple[float, ...] | None = None,
        left_quaternion_xyzw: tuple[float, ...] | None = None,
        right_quaternion_xyzw: tuple[float, ...] | None = None,
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
            packet["left"] = {
                "mode": "TcpPoseTarget",
                "tcp_target_stand": self._tcp_pose_payload(
                    left_pose,
                    quaternion_xyzw=left_quaternion_xyzw,
                    label="left TCP target",
                ),
            }
        if right_pose is not None:
            packet["right"] = {
                "mode": "TcpPoseTarget",
                "tcp_target_stand": self._tcp_pose_payload(
                    right_pose,
                    quaternion_xyzw=right_quaternion_xyzw,
                    label="right TCP target",
                ),
            }
        return self._with_source(packet)

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
        return self._with_source(packet)

    def build_tcp_delta_local(
        self,
        *,
        left_delta: tuple[float, ...] | None = None,
        right_delta: tuple[float, ...] | None = None,
        timeout_sec: float = 0.2,
    ) -> dict[str, Any]:
        if left_delta is None and right_delta is None:
            raise ValueError("at least one TCP local delta is required")
        packet: dict[str, Any] = {
            "schema_version": 1,
            "seq": self.next_seq(),
            "mode": "TcpDeltaLocal" if left_delta is not None and right_delta is not None else "Hold",
            "host_time_ns": time.monotonic_ns(),
            "timeout_sec": timeout_sec,
            "coupled_timeout": True,
            "left": {},
            "right": {},
        }
        if left_delta is not None:
            packet["left"] = {"mode": "TcpDeltaLocal", "tcp_delta_local": self._finite_six(left_delta, "left TCP local delta")}
        if right_delta is not None:
            packet["right"] = {"mode": "TcpDeltaLocal", "tcp_delta_local": self._finite_six(right_delta, "right TCP local delta")}
        return self._with_source(packet)

    def _with_source(self, packet: dict[str, Any]) -> dict[str, Any]:
        packet["source_id"] = self.source_id
        packet["session_id"] = self.session_id
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
