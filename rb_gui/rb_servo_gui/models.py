from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping
import math
import time

DOF = 6
TCP_DISPLAY_MODES = ("auto", "actual", "reference", "both")
CIRCLE_OVERLAY_SCHEMA_VERSION = "robotics_lab.circle_overlay.v1"


def finite_joint_array(value: Any) -> tuple[float, ...] | None:
    if not isinstance(value, list | tuple) or len(value) != DOF:
        return None
    out: list[float] = []
    for item in value:
        if not isinstance(item, int | float) or not math.isfinite(float(item)):
            return None
        out.append(float(item))
    return tuple(out)


@dataclass(frozen=True)
class Pose6D:
    x: float
    y: float
    z: float
    rx: float
    ry: float
    rz: float
    quaternion_xyzw: tuple[float, float, float, float] | None = None

    @classmethod
    def parse(cls, value: Any) -> "Pose6D | None":
        try:
            quaternion_xyzw = None
            if isinstance(value, Mapping):
                values = (value["x"], value["y"], value["z"], value["rx"], value["ry"], value["rz"])
                quaternion_xyzw = _parse_quaternion_xyzw(value)
            elif isinstance(value, list | tuple) and len(value) == 6:
                values = tuple(value)
            else:
                return None
            parsed = tuple(float(item) for item in values)
        except Exception:
            return None
        if not all(math.isfinite(item) for item in parsed):
            return None
        return cls(*parsed, quaternion_xyzw=quaternion_xyzw)

    def as_tuple(self) -> tuple[float, float, float, float, float, float]:
        return (self.x, self.y, self.z, self.rx, self.ry, self.rz)


def _finite_vec3(value: Any) -> tuple[float, float, float] | None:
    if not isinstance(value, list | tuple) or len(value) != 3:
        return None
    try:
        parsed = tuple(float(item) for item in value)
    except Exception:
        return None
    if not all(math.isfinite(item) for item in parsed):
        return None
    return parsed  # type: ignore[return-value]


def _parse_quaternion_xyzw(value: Mapping[str, Any]) -> tuple[float, float, float, float] | None:
    raw = value.get("quaternion_xyzw")
    if isinstance(raw, list | tuple) and len(raw) == 4:
        values = raw
    elif all(key in value for key in ("qx", "qy", "qz", "qw")):
        values = (value["qx"], value["qy"], value["qz"], value["qw"])
    else:
        return None
    try:
        parsed = tuple(float(item) for item in values)
    except Exception:
        return None
    if len(parsed) != 4 or not all(math.isfinite(item) for item in parsed):
        return None
    norm = math.sqrt(sum(item * item for item in parsed))
    if not math.isfinite(norm) or norm <= 0.0:
        return None
    return (parsed[0] / norm, parsed[1] / norm, parsed[2] / norm, parsed[3] / norm)


@dataclass(frozen=True)
class CircleOverlaySnapshot:
    schema_version: str
    host_time_ns: int
    received_monotonic: float
    run_id: str
    arm: str
    profile: str
    controller: str
    tracking_source: str
    plane: str
    center_stand: tuple[float, float, float]
    axis1_stand: tuple[float, float, float] | None
    axis2_stand: tuple[float, float, float] | None
    radius_m: float
    period_sec: float
    phase_rad: float
    desired_pose_stand: Pose6D
    current_error_m: float | None
    running_rms_error_m: float | None
    running_p95_error_m: float | None
    estimated_latency_ms: float | None
    sample_count: int
    command_count: int | None
    physical_motion_expected: bool | None
    result_so_far: str | None
    raw: Mapping[str, Any]

    @classmethod
    def parse(
        cls,
        data: Mapping[str, Any],
        *,
        received_monotonic: float | None = None,
    ) -> "CircleOverlaySnapshot | None":
        if data.get("schema_version") != CIRCLE_OVERLAY_SCHEMA_VERSION:
            return None
        host_time_ns = data.get("host_time_ns")
        if not isinstance(host_time_ns, int) or host_time_ns < 0:
            return None
        center = _finite_vec3(data.get("center_stand"))
        radius_m = _optional_finite(data.get("radius_m"))
        period_sec = _optional_finite(data.get("period_sec"))
        phase_rad = _optional_finite(data.get("phase_rad"))
        desired_pose = Pose6D.parse(data.get("desired_pose_stand"))
        if center is None or radius_m is None or radius_m <= 0.0:
            return None
        if period_sec is None or period_sec <= 0.0 or phase_rad is None or desired_pose is None:
            return None
        arm = data.get("arm")
        plane = data.get("plane")
        if arm not in {"left", "right"} or plane not in {"xy", "xz", "yz"}:
            return None
        sample_count = data.get("sample_count")
        if not isinstance(sample_count, int) or sample_count < 0:
            return None
        command_count = data.get("command_count")
        if command_count is not None and (not isinstance(command_count, int) or command_count < 0):
            return None
        axis1 = _finite_vec3(data.get("axis1_stand"))
        axis2 = _finite_vec3(data.get("axis2_stand"))
        return cls(
            schema_version=CIRCLE_OVERLAY_SCHEMA_VERSION,
            host_time_ns=host_time_ns,
            received_monotonic=time.monotonic() if received_monotonic is None else received_monotonic,
            run_id=str(data.get("run_id", "")),
            arm=str(arm),
            profile=str(data.get("profile", "")),
            controller=str(data.get("controller", "")),
            tracking_source=str(data.get("tracking_source", "")),
            plane=str(plane),
            center_stand=center,
            axis1_stand=axis1,
            axis2_stand=axis2,
            radius_m=radius_m,
            period_sec=period_sec,
            phase_rad=phase_rad,
            desired_pose_stand=desired_pose,
            current_error_m=_optional_finite(data.get("current_error_m")),
            running_rms_error_m=_optional_finite(data.get("running_rms_error_m")),
            running_p95_error_m=_optional_finite(data.get("running_p95_error_m")),
            estimated_latency_ms=_optional_finite(data.get("estimated_latency_ms")),
            sample_count=sample_count,
            command_count=int(command_count) if isinstance(command_count, int) else None,
            physical_motion_expected=_optional_bool(data.get("physical_motion_expected")),
            result_so_far=str(data["result_so_far"]) if isinstance(data.get("result_so_far"), str) else None,
            raw=data,
        )

    def stale(self, *, now: float | None = None, threshold_sec: float = 1.0) -> bool:
        now_value = time.monotonic() if now is None else now
        return now_value - self.received_monotonic > threshold_sec


@dataclass(frozen=True)
class CommandSourceSnapshot:
    active: bool | None = None
    source_id: str | None = None
    session_id: str | None = None
    active_source_id: str | None = None
    active_session_id: str | None = None
    lease_timeout_sec: float | None = None
    expires_time_ns: int | None = None
    expired: bool | None = None
    command_requires_lease: bool | None = None
    command_has_lease: bool | None = None
    raw: Mapping[str, Any] | None = None

    @classmethod
    def parse(cls, value: Any) -> "CommandSourceSnapshot":
        if not isinstance(value, Mapping):
            return cls()
        return cls(
            active=_optional_bool(value.get("active")),
            source_id=_optional_str(value.get("source_id")),
            session_id=_optional_str(value.get("session_id")),
            active_source_id=_optional_str(value.get("active_source_id")),
            active_session_id=_optional_str(value.get("active_session_id")),
            lease_timeout_sec=_optional_finite(value.get("lease_timeout_sec")),
            expires_time_ns=_optional_int(value.get("expires_time_ns")),
            expired=_optional_bool(value.get("expired")),
            command_requires_lease=_optional_bool(value.get("command_requires_lease")),
            command_has_lease=_optional_bool(value.get("command_has_lease")),
            raw=value,
        )

    @property
    def display_source_id(self) -> str | None:
        return self.active_source_id or self.source_id

    @property
    def display_session_id(self) -> str | None:
        return self.active_session_id or self.session_id


@dataclass(frozen=True)
class ArmSnapshot:
    mode: str
    q_actual_deg: tuple[float, ...] | None
    q_sent_deg: tuple[float, ...] | None
    q_previous_sent_deg: tuple[float, ...] | None
    has_valid_joint_state: bool
    connection_state: str
    send_ok: bool
    error_code: int | None = None
    q_ref_deg: tuple[float, ...] | None = None
    tcp_stand: Pose6D | None = None
    tcp_base: Pose6D | None = None
    tcp_actual_stand: Pose6D | None = None
    tcp_actual_base: Pose6D | None = None
    tcp_ref_stand: Pose6D | None = None
    tcp_ref_base: Pose6D | None = None
    tcp_command_stand: Pose6D | None = None
    has_valid_tcp_pose: bool = False
    tcp_actual_valid: bool = False
    tcp_ref_valid: bool = False
    tcp_tracking_source: str | None = None
    tcp_tracking_source_recommendation: str | None = None
    controller_simulation_mode: Mapping[str, Any] | None = None
    cartesian_gate: Mapping[str, Any] | None = None
    cartesian_available: bool | None = None
    cartesian_unavailable_reason: str | None = None
    controller_simulation_cartesian_enabled: bool | None = None
    controller_simulation_cartesian_enabled_for_current_command: bool | None = None
    controller_simulation_streaming_cartesian_available: bool | None = None
    physical_motion_expected: bool | None = None
    controller_simulation_physical_motion_detected: bool | None = None
    controller_simulation_diagnostic_override_active: bool | None = None
    tcp_deferred: bool = True
    send_duration_us: float | None = None
    cartesian_solve: "CartesianSolveSnapshot | None" = None

    @classmethod
    def parse(
        cls,
        data: Mapping[str, Any],
        *,
        fallback_cartesian_gate: Mapping[str, Any] | None = None,
        fallback_state: Mapping[str, Any] | None = None,
    ) -> "ArmSnapshot | None":
        q_actual = finite_joint_array(data.get("q_actual_deg"))
        q_sent = finite_joint_array(data.get("q_sent_deg"))
        q_prev = finite_joint_array(data.get("q_previous_sent_deg"))
        q_ref = finite_joint_array(data.get("q_ref_deg"))
        has_valid_joint_state = bool(data.get("has_valid_joint_state", False)) and q_actual is not None and q_sent is not None and q_prev is not None
        error_code = data.get("error_code")
        tcp_stand = Pose6D.parse(data.get("tcp_stand"))
        tcp_base = Pose6D.parse(data.get("tcp_base"))
        tcp_actual_stand = Pose6D.parse(data.get("tcp_actual_stand")) or tcp_stand
        tcp_actual_base = Pose6D.parse(data.get("tcp_actual_base")) or tcp_base
        tcp_ref_stand = Pose6D.parse(data.get("tcp_ref_stand"))
        tcp_ref_base = Pose6D.parse(data.get("tcp_ref_base"))
        tcp_command_stand = Pose6D.parse(data.get("tcp_command_stand"))
        has_valid_tcp_pose_raw = data.get("has_valid_tcp_pose")
        has_valid_tcp_pose = bool(has_valid_tcp_pose_raw) if isinstance(has_valid_tcp_pose_raw, bool) else tcp_stand is not None
        tcp_actual_valid = _optional_bool(data.get("tcp_actual_valid"))
        if tcp_actual_valid is None:
            tcp_actual_valid = has_valid_tcp_pose and tcp_actual_stand is not None
        tcp_ref_valid = bool(data.get("tcp_ref_valid", False)) and tcp_ref_stand is not None
        tcp_deferred = bool(
            data.get(
                "tcp_deferred",
                tcp_actual_stand is None
                and tcp_actual_base is None
                and tcp_ref_stand is None
                and tcp_ref_base is None
                and not has_valid_tcp_pose,
            )
        )
        controller_simulation_mode = data.get("controller_simulation_mode")
        cartesian_gate = data.get("cartesian_gate")
        if not isinstance(cartesian_gate, Mapping):
            cartesian_gate = fallback_cartesian_gate if isinstance(fallback_cartesian_gate, Mapping) else None
        return cls(
            mode=str(data.get("mode", "Unknown")),
            q_actual_deg=q_actual,
            q_sent_deg=q_sent,
            q_previous_sent_deg=q_prev,
            q_ref_deg=q_ref,
            has_valid_joint_state=has_valid_joint_state,
            connection_state=str(data.get("connection_state", "Disconnected")),
            send_ok=bool(data.get("send_ok", False)),
            error_code=int(error_code) if isinstance(error_code, int) else None,
            tcp_stand=tcp_stand,
            tcp_base=tcp_base,
            tcp_actual_stand=tcp_actual_stand,
            tcp_actual_base=tcp_actual_base,
            tcp_ref_stand=tcp_ref_stand,
            tcp_ref_base=tcp_ref_base,
            tcp_command_stand=tcp_command_stand,
            has_valid_tcp_pose=has_valid_tcp_pose and tcp_stand is not None,
            tcp_actual_valid=bool(tcp_actual_valid),
            tcp_ref_valid=tcp_ref_valid,
            tcp_tracking_source=str(data["tcp_tracking_source"]) if isinstance(data.get("tcp_tracking_source"), str) else None,
            tcp_tracking_source_recommendation=(
                str(data["tcp_tracking_source_recommendation"])
                if isinstance(data.get("tcp_tracking_source_recommendation"), str)
                else None
            ),
            controller_simulation_mode=controller_simulation_mode if isinstance(controller_simulation_mode, Mapping) else None,
            cartesian_gate=cartesian_gate,
            cartesian_available=_optional_bool(_field_value(data, "cartesian_available", cartesian_gate, fallback_state)),
            cartesian_unavailable_reason=_optional_str(
                _field_value(data, "cartesian_unavailable_reason", cartesian_gate, fallback_state)
            ),
            controller_simulation_cartesian_enabled=_optional_bool(
                _field_value(data, "controller_simulation_cartesian_enabled", cartesian_gate, fallback_state)
            ),
            controller_simulation_cartesian_enabled_for_current_command=_optional_bool(
                _field_value(data, "controller_simulation_cartesian_enabled_for_current_command", cartesian_gate, fallback_state)
            ),
            controller_simulation_streaming_cartesian_available=_optional_bool(
                _field_value(data, "controller_simulation_streaming_cartesian_available", cartesian_gate, fallback_state)
            ),
            physical_motion_expected=_optional_bool(
                _field_value(data, "physical_motion_expected", cartesian_gate, fallback_state)
            ),
            controller_simulation_physical_motion_detected=_optional_bool(
                _field_value(data, "controller_simulation_physical_motion_detected", cartesian_gate, fallback_state)
            ),
            controller_simulation_diagnostic_override_active=_optional_bool(
                data.get("controller_simulation_diagnostic_override_active")
            ),
            tcp_deferred=tcp_deferred,
            send_duration_us=float(data["send_duration_us"]) if isinstance(data.get("send_duration_us"), int | float) else None,
            cartesian_solve=CartesianSolveSnapshot.parse(data.get("cartesian_solve")),
        )

    def selected_tcp_pose(self, mode: str = "auto") -> Pose6D | None:
        selected = self.selected_tcp_source(mode)
        if selected == "tcp_ref_stand":
            return self.tcp_ref_stand
        if selected in {"tcp_actual_stand", "tcp_stand"}:
            return self.tcp_actual_stand or self.tcp_stand
        return None

    def selected_tcp_source(self, mode: str = "auto") -> str:
        if mode == "both":
            return self.selected_tcp_source("auto")
        if mode == "reference":
            return "tcp_ref_stand" if self.tcp_ref_valid and self.tcp_ref_stand is not None else "none"
        if mode == "actual":
            return "tcp_actual_stand" if self.tcp_actual_valid and self.tcp_actual_stand is not None else "none"
        if mode == "auto":
            ref_requested = self.tcp_tracking_source == "tcp_ref_stand" or self.tcp_tracking_source_recommendation in {
                "tcp_ref_stand",
                "reference_for_controller_simulation",
            }
            if ref_requested and self.tcp_ref_valid and self.tcp_ref_stand is not None:
                return "tcp_ref_stand"
            if self.tcp_actual_valid and self.tcp_actual_stand is not None:
                return "tcp_actual_stand"
            if self.has_valid_tcp_pose and self.tcp_stand is not None:
                return "tcp_stand"
        return "none"

    @property
    def controller_simulation_cartesian_available(self) -> bool | None:
        if self.controller_simulation_streaming_cartesian_available is not None:
            return self.controller_simulation_streaming_cartesian_available
        if self.controller_simulation_cartesian_enabled_for_current_command is not None:
            return self.controller_simulation_cartesian_enabled_for_current_command
        return self.controller_simulation_cartesian_enabled


@dataclass(frozen=True)
class CartesianSolveSnapshot:
    attempted: bool | None = None
    success: bool | None = None
    status: str | None = None
    reason: str | None = None
    position_error_m: float | None = None
    orientation_error_rad: float | None = None
    ik_iterations: int | None = None
    ik_duration_us: float | None = None
    ik_timed_out: bool | None = None
    path_active: bool | None = None
    path_s: float | None = None
    path_line_deviation_m: float | None = None
    path_orientation_error_rad: float | None = None
    path_done: bool | None = None

    @classmethod
    def parse(cls, value: Any) -> "CartesianSolveSnapshot | None":
        if not isinstance(value, Mapping):
            return None
        return cls(
            attempted=_optional_bool(value.get("attempted")),
            success=_optional_bool(value.get("success")),
            status=str(value["status"]) if isinstance(value.get("status"), str) else None,
            reason=str(value["reason"]) if isinstance(value.get("reason"), str) else None,
            position_error_m=_optional_finite(value.get("position_error_m")),
            orientation_error_rad=_optional_finite(value.get("orientation_error_rad")),
            ik_iterations=int(value["ik_iterations"]) if isinstance(value.get("ik_iterations"), int) else None,
            ik_duration_us=_optional_finite(value.get("ik_duration_us")),
            ik_timed_out=_optional_bool(value.get("ik_timed_out")),
            path_active=_optional_bool(value.get("path_active")),
            path_s=_optional_finite(value.get("path_s")),
            path_line_deviation_m=_optional_finite(value.get("path_line_deviation_m")),
            path_orientation_error_rad=_optional_finite(value.get("path_orientation_error_rad")),
            path_done=_optional_bool(value.get("path_done")),
        )


def _optional_bool(value: Any) -> bool | None:
    return value if isinstance(value, bool) else None


def _optional_str(value: Any) -> str | None:
    return str(value) if isinstance(value, str) and value else None


def _optional_int(value: Any) -> int | None:
    return int(value) if isinstance(value, int) else None


def _optional_finite(value: Any) -> float | None:
    if not isinstance(value, int | float):
        return None
    parsed = float(value)
    return parsed if math.isfinite(parsed) else None


def _field_value(
    data: Mapping[str, Any],
    key: str,
    *fallbacks: Mapping[str, Any] | None,
) -> Any:
    if key in data:
        return data.get(key)
    for fallback in fallbacks:
        if isinstance(fallback, Mapping) and key in fallback:
            return fallback.get(key)
    return None


@dataclass(frozen=True)
class StateSnapshot:
    tick: int
    received_monotonic: float
    left: ArmSnapshot
    right: ArmSnapshot
    motion_state: str
    safety_verdict: str
    fault_latched: bool
    fault_reason: str
    logger_health: Mapping[str, Any]
    mounts: Mapping[str, Any]
    observed_mode: str | None
    observed_backend: str | None
    command_source: CommandSourceSnapshot
    cartesian_gate: Mapping[str, Any] | None
    self_collision: Mapping[str, Any] | None
    raw: Mapping[str, Any]

    @classmethod
    def parse(cls, data: Mapping[str, Any], *, received_monotonic: float | None = None) -> "StateSnapshot | None":
        if int(data.get("schema_version", -1)) != 1:
            return None
        left_raw = data.get("left")
        right_raw = data.get("right")
        if not isinstance(left_raw, Mapping) or not isinstance(right_raw, Mapping):
            return None
        top_cartesian_gate = data.get("cartesian_gate")
        left_cartesian_gate = None
        right_cartesian_gate = None
        if isinstance(top_cartesian_gate, Mapping):
            left_gate = top_cartesian_gate.get("left")
            right_gate = top_cartesian_gate.get("right")
            left_cartesian_gate = left_gate if isinstance(left_gate, Mapping) else top_cartesian_gate
            right_cartesian_gate = right_gate if isinstance(right_gate, Mapping) else top_cartesian_gate
        else:
            top_cartesian_gate = None
        left = ArmSnapshot.parse(left_raw, fallback_cartesian_gate=left_cartesian_gate, fallback_state=data)
        right = ArmSnapshot.parse(right_raw, fallback_cartesian_gate=right_cartesian_gate, fallback_state=data)
        if left is None or right is None:
            return None
        tick = data.get("tick")
        if not isinstance(tick, int) or tick < 0:
            return None
        return cls(
            tick=tick,
            received_monotonic=time.monotonic() if received_monotonic is None else received_monotonic,
            left=left,
            right=right,
            motion_state=str(data.get("motion_state", "Disconnected")),
            safety_verdict=str(data.get("safety_verdict", "Unknown")),
            fault_latched=bool(data.get("fault_latched", False)),
            fault_reason=str(data.get("fault_reason", "")),
            logger_health=data.get("logger_health", {}) if isinstance(data.get("logger_health", {}), Mapping) else {},
            mounts=data.get("mounts", {}) if isinstance(data.get("mounts", {}), Mapping) else {},
            observed_mode=_optional_str(data.get("observed_mode")),
            observed_backend=_optional_str(data.get("observed_backend")),
            command_source=CommandSourceSnapshot.parse(data.get("command_source")),
            cartesian_gate=top_cartesian_gate if isinstance(top_cartesian_gate, Mapping) else None,
            self_collision=data.get("self_collision") if isinstance(data.get("self_collision"), Mapping) else None,
            raw=data,
        )

    def stale(self, *, now: float | None = None, threshold_sec: float = 0.5) -> bool:
        now_value = time.monotonic() if now is None else now
        return now_value - self.received_monotonic > threshold_sec

    @property
    def active_command_mode(self) -> str:
        if self.left.mode == self.right.mode:
            return self.left.mode
        return f"left={self.left.mode},right={self.right.mode}"
