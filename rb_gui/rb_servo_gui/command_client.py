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
        # When True, send() rides an already-held lease instead of bracketing
        # every command with Acquire/Release.
        self.hold_lease = False

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
            "left": {"mode": "JointTarget", "q_target_deg": [float(v) for v in left_q]},
            "right": {"mode": "JointTarget", "q_target_deg": [float(v) for v in right_q]},
        })

    def build_init_motion(
        self,
        left_q: tuple[float, ...],
        right_q: tuple[float, ...],
        *,
        timeout_sec: float = 0.2,
    ) -> dict[str, Any]:
        # Collision-free init plan: a JointTarget profile asks the server to plan
        # and stream a collision-free + floor-safe joint path to the init pose.
        # The long timeout must cover plan + execution.
        if len(left_q) != 6 or len(right_q) != 6:
            raise ValueError("joint targets must have 6 values per arm")
        return self._with_source({
            "seq": self.next_seq(),
            "mode": "JointTarget",
            "timeout_sec": timeout_sec,
            "coupled_timeout": True,
            "left": {
                "mode": "JointTarget",
                "q_target_deg": [float(v) for v in left_q],
                "joint_target_profile": "init_motion",
            },
            "right": {
                "mode": "JointTarget",
                "q_target_deg": [float(v) for v in right_q],
                "joint_target_profile": "init_motion",
            },
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
        timeout_sec: float | None = None,
    ) -> dict[str, Any]:
        if left_pose is None and right_pose is None:
            raise ValueError("at least one TCP linear target is required")
        parsed_duration = self._optional_positive_finite(duration_sec, "duration_sec")
        parsed_linear_speed = self._optional_positive_finite(linear_speed_m_s, "linear_speed_m_s")
        parsed_angular_speed = self._optional_positive_finite(angular_speed_rad_s, "angular_speed_rad_s")
        if parsed_duration is None and parsed_linear_speed is None:
            raise ValueError("duration_sec or linear_speed_m_s is required")
        parsed_orientation_mode = self._orientation_mode(orientation_mode)
        # Keep this one-shot TcpLinearMove command FRESH long enough for the server's
        # async collision-free decision to complete AND hand the path to the Cartesian
        # executor (whose own finite-path continuation then carries it across staleness).
        # The prior fixed 0.2 s expired DURING the decision, so on a slow decision the
        # executor only ever saw a synthetic Hold and the MoveL never started.
        # Hold/E-stop still cancel; the server execution_timeout bounds runaway.
        if timeout_sec is None:
            base = parsed_duration if parsed_duration is not None else 0.0
            timeout_sec = max(2.0, base + 0.5)
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

    def _with_source(self, packet: dict[str, Any]) -> dict[str, Any]:
        packet["source_id"] = self.source_id
        packet["session_id"] = self.session_id
        return packet

    def build_freedrive(
        self,
        *,
        left: bool | None = None,
        right: bool | None = None,
        timeout_sec: float = 0.2,
    ) -> dict[str, Any]:
        """Per-arm direct-teaching (free-drive) toggle.

        left/right: True enters free-drive (hand-guidable), False exits and
        resyncs, None leaves that arm untouched (Hold). At least one arm must be
        specified. Server-side this is a sticky lifecycle command gated by
        servo.allow_freedrive.
        """
        if left is None and right is None:
            raise ValueError("freedrive requires at least one of left/right")
        packet: dict[str, Any] = {
            "seq": self.next_seq(),
            "mode": "Freedrive",
            "timeout_sec": timeout_sec,
            "left": {"mode": "Freedrive", "freedrive_on": bool(left)} if left is not None else {"mode": "Hold"},
            "right": {"mode": "Freedrive", "freedrive_on": bool(right)} if right is not None else {"mode": "Hold"},
        }
        return self._with_source(packet)

    def send_freedrive(
        self,
        *,
        left: bool | None = None,
        right: bool | None = None,
        timeout_sec: float = 0.2,
    ) -> dict[str, Any]:
        packet = self.build_freedrive(left=left, right=right, timeout_sec=timeout_sec)
        self.send(packet)
        return packet

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

    def build_set_safety_floor_enabled(self, enabled: bool, *, timeout_sec: float = 0.2) -> dict[str, Any]:
        # Leaseless non-motion command: runtime enforce on/off for the stand floor.
        # The server only honours enable=true when floor_constraint.enable=true in config.
        return self._with_source({
            "seq": self.next_seq(),
            "mode": "SetSafetyFloorEnabled",
            "timeout_sec": timeout_sec,
            "floor_enabled": bool(enabled),
            "left": {},
            "right": {},
        })

    def build_set_safety_roi_bounds(
        self,
        roi_min_m: tuple[float, float, float],
        roi_max_m: tuple[float, float, float],
        *,
        timeout_sec: float = 0.2,
    ) -> dict[str, Any]:
        if len(roi_min_m) != 3 or len(roi_max_m) != 3:
            raise ValueError("roi_min_m and roi_max_m must each have 3 values")
        lo = [float(v) for v in roi_min_m]
        hi = [float(v) for v in roi_max_m]
        if any(not math.isfinite(v) for v in lo + hi):
            raise ValueError("roi bounds must be finite")
        if any(lo[k] > hi[k] for k in range(3)):
            raise ValueError("roi_min_m must not exceed roi_max_m on any axis")
        # Leaseless non-motion command; the server only accepts bounds within its
        # configured safety.roi_box runtime envelope (per axis).
        return self._with_source({
            "seq": self.next_seq(),
            "mode": "SetSafetyRoiBounds",
            "timeout_sec": timeout_sec,
            "roi_min_m": lo,
            "roi_max_m": hi,
            "left": {},
            "right": {},
        })

    def build_set_user_safety_floor_plane(
        self,
        point_m: tuple[float, float, float],
        normal: tuple[float, float, float],
        *,
        margin_m: float = 0.0,
        enable: bool = True,
        timeout_sec: float = 0.2,
    ) -> dict[str, Any]:
        if len(point_m) != 3 or len(normal) != 3:
            raise ValueError("point_m and normal must each have 3 values")
        point = [float(v) for v in point_m]
        n = [float(v) for v in normal]
        margin = float(margin_m)
        if any(not math.isfinite(v) for v in point + n) or not math.isfinite(margin):
            raise ValueError("user floor plane values must be finite")
        norm = math.sqrt(sum(v * v for v in n))
        if norm < 1e-9:
            raise ValueError("user floor normal must be non-degenerate")
        # Normalize client-side so the server's unit-normal check passes; the server
        # still validates tilt / point bounds / margin against safety.user_floor_constraint.
        n = [v / norm for v in n]
        # Leaseless non-motion command. enable=False turns the constraint off
        # unconditionally (the point/normal are ignored server-side in that case).
        return self._with_source({
            "seq": self.next_seq(),
            "mode": "SetUserSafetyFloorPlane",
            "timeout_sec": timeout_sec,
            "user_floor_point_m": point,
            "user_floor_normal": n,
            "user_floor_margin_m": margin,
            "user_floor_enable": bool(enable),
            "left": {},
            "right": {},
        })

    # Modes that take (or require) the command-source lease when sent. After a
    # one-shot GUI command the lease is released immediately so a streaming
    # client (policy_runner teleop) can take over without waiting for the
    # server-side lease timeout.
    _LEASED_MODES = {
        "ArmMotion",
        "DisarmMotion",
        # ResetFault requires the lease server-side (commandRequiresLease);
        # without the bracket a GUI ResetFault is rejected with
        # command_source_lease_required whenever no lease is active.
        "ResetFault",
        "JointTarget",
        "TcpPoseTarget",
        "TcpLinearMove",
        # Per-arm direct teaching (free-drive) toggles servo authority and
        # requires the lease server-side (commandRequiresLease), so it brackets
        # like ResetFault/ArmMotion.
        "Freedrive",
        "Hold",
    }

    def send(self, packet: Mapping[str, Any]) -> None:
        # Atomic lease bracket: only AcquireLease/ArmMotion can TAKE the lease
        # on the server; plain motion commands (e.g. JointTarget) merely ride an
        # existing one. Acquire right before the command and release right
        # after, so one-shot GUI commands work without camping on the lease
        # between clicks. The server enforces strictly increasing seq per
        # source, so ALL THREE packets get freshly issued seqs here.
        # While a lease is explicitly held (Take control), motion commands ride
        # it: the lease owner's commands renew the server-side lease timeout, so
        # no per-command Acquire/Release bracket is needed (and tearing it down
        # between streaming packets would let another source grab it).
        bracket = str(packet.get("mode")) in self._LEASED_MODES and not self.hold_lease
        out = dict(packet)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            if bracket:
                sock.sendto(json.dumps(self._lease_packet("AcquireLease"), separators=(",", ":")).encode("utf-8"), (self.host, self.port))
                out["seq"] = self.next_seq()
            sock.sendto(json.dumps(out, separators=(",", ":")).encode("utf-8"), (self.host, self.port))
            if bracket:
                sock.sendto(json.dumps(self._lease_packet("ReleaseLease"), separators=(",", ":")).encode("utf-8"), (self.host, self.port))
        self.sent_packets.append(out)

    def _lease_packet(self, mode: str) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "seq": self.next_seq(),
            "mode": mode,
            "source_id": self.source_id,
            "session_id": self.session_id,
        }

    def acquire_lease(self) -> dict[str, Any]:
        """Explicitly take and hold the command-source lease (Take control ON).

        Subsequent motion commands ride the held lease until release_lease().
        Returns the AcquireLease packet that was sent."""
        packet = self._lease_packet("AcquireLease")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.sendto(json.dumps(packet, separators=(",", ":")).encode("utf-8"), (self.host, self.port))
        self.hold_lease = True
        return packet

    def release_lease(self) -> dict[str, Any]:
        """Release a held lease (Take control OFF) and resume one-shot bracketing."""
        self.hold_lease = False
        packet = self._lease_packet("ReleaseLease")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.sendto(json.dumps(packet, separators=(",", ":")).encode("utf-8"), (self.host, self.port))
        return packet

    def send_lifecycle(self, mode: str, *, timeout_sec: float = 0.2) -> dict[str, Any]:
        packet = self.build_lifecycle(mode, timeout_sec=timeout_sec)
        self.send(packet)
        return packet

    def send_set_safety_floor_z(self, floor_z_m: float, *, timeout_sec: float = 0.2) -> dict[str, Any]:
        packet = self.build_set_safety_floor_z(floor_z_m, timeout_sec=timeout_sec)
        self.send(packet)
        return packet

    def send_set_safety_floor_enabled(self, enabled: bool, *, timeout_sec: float = 0.2) -> dict[str, Any]:
        packet = self.build_set_safety_floor_enabled(enabled, timeout_sec=timeout_sec)
        self.send(packet)
        return packet

    def send_set_safety_roi_bounds(
        self,
        roi_min_m: tuple[float, float, float],
        roi_max_m: tuple[float, float, float],
        *,
        timeout_sec: float = 0.2,
    ) -> dict[str, Any]:
        packet = self.build_set_safety_roi_bounds(roi_min_m, roi_max_m, timeout_sec=timeout_sec)
        self.send(packet)
        return packet

    def send_set_user_safety_floor_plane(
        self,
        point_m: tuple[float, float, float],
        normal: tuple[float, float, float],
        *,
        margin_m: float = 0.0,
        enable: bool = True,
        timeout_sec: float = 0.2,
    ) -> dict[str, Any]:
        packet = self.build_set_user_safety_floor_plane(
            point_m, normal, margin_m=margin_m, enable=enable, timeout_sec=timeout_sec)
        self.send(packet)
        return packet
