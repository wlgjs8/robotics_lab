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

    @staticmethod
    def _optional_positive_finite(value: float | None, label: str) -> float | None:
        if value is None:
            return None
        parsed = float(value)
        if not math.isfinite(parsed) or parsed <= 0.0:
            raise ValueError(f"{label} must be finite and positive")
        return parsed

    @staticmethod
    def _orientation_mode(value: str) -> str:
        mode = str(value).strip().lower()
        if mode not in {"constant", "slerp"}:
            raise ValueError("orientation_mode must be constant or slerp")
        return mode

    def build_tcp_linear_move(
        self,
        *,
        left_pose: tuple[float, ...] | None = None,
        right_pose: tuple[float, ...] | None = None,
        left_quaternion_xyzw: tuple[float, ...] | None = None,
        right_quaternion_xyzw: tuple[float, ...] | None = None,
        duration_sec: float | None = None,
        linear_speed_m_s: float | None = None,
        angular_speed_rad_s: float | None = None,
        orientation_mode: str = "constant",
        timeout_sec: float = 0.2,
    ) -> dict[str, Any]:
        if left_pose is None and right_pose is None:
            raise ValueError("at least one TCP linear target is required")
        parsed_duration = self._optional_positive_finite(duration_sec, "duration_sec")
        parsed_linear_speed = self._optional_positive_finite(linear_speed_m_s, "linear_speed_m_s")
        parsed_angular_speed = self._optional_positive_finite(angular_speed_rad_s, "angular_speed_rad_s")
        if parsed_duration is None and parsed_linear_speed is None:
            raise ValueError("duration_sec or linear_speed_m_s is required")
        parsed_orientation_mode = self._orientation_mode(orientation_mode)
        packet: dict[str, Any] = {
            "schema_version": 1,
            "seq": self.next_seq(),
            "mode": "TcpLinearMove" if left_pose is not None and right_pose is not None else "Hold",
            "host_time_ns": time.monotonic_ns(),
            "timeout_sec": timeout_sec,
            "coupled_timeout": True,
            "left": {},
            "right": {},
        }

        def arm_payload(pose: tuple[float, ...], quaternion_xyzw: tuple[float, ...] | None, label: str) -> dict[str, Any]:
            payload: dict[str, Any] = {
                "mode": "TcpLinearMove",
                "target_tcp_stand": self._tcp_pose_payload(
                    pose,
                    quaternion_xyzw=quaternion_xyzw,
                    label=label,
                ),
                "orientation_mode": parsed_orientation_mode,
            }
            if parsed_duration is not None:
                payload["duration_sec"] = parsed_duration
            if parsed_linear_speed is not None:
                payload["linear_speed_m_s"] = parsed_linear_speed
            if parsed_angular_speed is not None:
                payload["angular_speed_rad_s"] = parsed_angular_speed
            return payload

        if left_pose is not None:
            packet["left"] = arm_payload(left_pose, left_quaternion_xyzw, "left TCP linear target")
        if right_pose is not None:
            packet["right"] = arm_payload(right_pose, right_quaternion_xyzw, "right TCP linear target")
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

    def build_tcp_twist_local(
        self,
        *,
        left_twist: tuple[float, ...] | None = None,
        right_twist: tuple[float, ...] | None = None,
        timeout_sec: float = 0.2,
    ) -> dict[str, Any]:
        if left_twist is None and right_twist is None:
            raise ValueError("at least one TCP local twist is required")
        packet: dict[str, Any] = {
            "schema_version": 1,
            "seq": self.next_seq(),
            "mode": "TcpTwistLocal" if left_twist is not None and right_twist is not None else "Hold",
            "host_time_ns": time.monotonic_ns(),
            "timeout_sec": timeout_sec,
            "coupled_timeout": True,
            "left": {},
            "right": {},
        }
        if left_twist is not None:
            packet["left"] = {"mode": "TcpTwistLocal", "tcp_twist_local": self._finite_six(left_twist, "left TCP local twist")}
        if right_twist is not None:
            packet["right"] = {"mode": "TcpTwistLocal", "tcp_twist_local": self._finite_six(right_twist, "right TCP local twist")}
        return self._with_source(packet)

    def build_tcp_twist_stand(
        self,
        *,
        left_twist: tuple[float, ...] | None = None,
        right_twist: tuple[float, ...] | None = None,
        timeout_sec: float = 0.2,
    ) -> dict[str, Any]:
        if left_twist is None and right_twist is None:
            raise ValueError("at least one TCP stand twist is required")
        packet: dict[str, Any] = {
            "schema_version": 1,
            "seq": self.next_seq(),
            "mode": "TcpTwistStand" if left_twist is not None and right_twist is not None else "Hold",
            "host_time_ns": time.monotonic_ns(),
            "timeout_sec": timeout_sec,
            "coupled_timeout": True,
            "left": {},
            "right": {},
        }
        if left_twist is not None:
            packet["left"] = {"mode": "TcpTwistStand", "tcp_twist_stand": self._finite_six(left_twist, "left TCP stand twist")}
        if right_twist is not None:
            packet["right"] = {"mode": "TcpTwistStand", "tcp_twist_stand": self._finite_six(right_twist, "right TCP stand twist")}
        return self._with_source(packet)

    def build_joint_velocity(
        self,
        *,
        left_velocity: tuple[float, ...] | None = None,
        right_velocity: tuple[float, ...] | None = None,
        timeout_sec: float = 0.2,
    ) -> dict[str, Any]:
        if left_velocity is None and right_velocity is None:
            raise ValueError("at least one joint velocity is required")
        packet: dict[str, Any] = {
            "schema_version": 1,
            "seq": self.next_seq(),
            "mode": "JointVelocity" if left_velocity is not None and right_velocity is not None else "Hold",
            "host_time_ns": time.monotonic_ns(),
            "timeout_sec": timeout_sec,
            "coupled_timeout": True,
            "left": {},
            "right": {},
        }
        if left_velocity is not None:
            packet["left"] = {"mode": "JointVelocity", "dq_target_deg_s": self._finite_six(left_velocity, "left joint velocity")}
        if right_velocity is not None:
            packet["right"] = {"mode": "JointVelocity", "dq_target_deg_s": self._finite_six(right_velocity, "right joint velocity")}
        return self._with_source(packet)

    def build_tcp_circle_move(
        self,
        *,
        left: bool = True,
        right: bool = True,
        diameter_m: float = 0.15,
        period_sec: float = 4.0,
        plane: str = "xy",
        repeat: int = 50,
    ) -> dict[str, Any]:
        # Full server-side circle payload. The server traces the whole circle
        # autonomously over period*repeat once it receives a command; the caller
        # must re-send THIS SAME packet (same seq) to keep it fresh — sending a
        # new seq resets the circle to the current TCP. timeout_sec covers the
        # full duration so the circle stays fresh between keep-alive sends.
        if not left and not right:
            raise ValueError("at least one arm is required for TcpCircleMove")
        duration_sec = float(period_sec) * int(repeat)
        arm = {
            "mode": "TcpCircleMove",
            "command_family": "server_circle",
            "plane": str(plane),
            "diameter_m": float(diameter_m),
            "period_sec": float(period_sec),
            "repeat": int(repeat),
            "phase_advance_sec": 0.0,
            "center_mode": "start_on_circle",
            "orientation_mode": "constant",
            "frame": "stand",
        }
        packet: dict[str, Any] = {
            "schema_version": 1,
            "seq": self.next_seq(),
            "mode": "TcpCircleMove" if left and right else "Hold",
            "host_time_ns": time.monotonic_ns(),
            "timeout_sec": max(0.2, duration_sec + 0.2),
            "coupled_timeout": True,
            "left": dict(arm) if left else {"mode": "Hold"},
            "right": dict(arm) if right else {"mode": "Hold"},
        }
        return self._with_source(packet)

    def build_set_safety_floor_z(self, floor_z_m: float, *, timeout_sec: float = 0.2) -> dict[str, Any]:
        value = float(floor_z_m)
        if not math.isfinite(value):
            raise ValueError("floor_z_m must be finite")
        # Leaseless non-motion command; the server only accepts values within its
        # configured safety.floor_constraint runtime bounds.
        return self._with_source({
            "seq": self.next_seq(),
            "mode": "SetSafetyFloorZ",
            "timeout_sec": timeout_sec,
            "floor_z_m": value,
            "left": {},
            "right": {},
        })

    def send(self, packet: Mapping[str, Any]) -> None:
        payload = json.dumps(packet, separators=(",", ":")).encode("utf-8")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.sendto(payload, (self.host, self.port))
        self.sent_packets.append(dict(packet))

    def send_lifecycle(self, mode: str, *, timeout_sec: float = 0.2) -> dict[str, Any]:
        packet = self.build_lifecycle(mode, timeout_sec=timeout_sec)
        self.send(packet)
        return packet

    def send_set_safety_floor_z(self, floor_z_m: float, *, timeout_sec: float = 0.2) -> dict[str, Any]:
        packet = self.build_set_safety_floor_z(floor_z_m, timeout_sec=timeout_sec)
        self.send(packet)
        return packet
