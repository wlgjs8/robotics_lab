from __future__ import annotations

from dataclasses import dataclass
import math
import os
import re
from pathlib import Path
from typing import Literal

from .command_client import CommandClient
from .models import StateSnapshot
from .state_receiver import StateStore


_FLOOR_Z_LINE_RE = re.compile(r"^(\s*z_min_m\s*:\s*)([0-9.eE+-]+)(.*)$")


def persist_floor_z_to_config(config_path: str | Path, floor_z_m: float) -> tuple[bool, str]:
    """Rewrite floor_constraint.z_min_m in the server config yaml (text-level,
    comments preserved) so a viser "Send floor z" survives a stack restart.

    Only the FIRST z_min_m line inside the floor_constraint block is touched.
    Returns (ok, short message for the GUI status field)."""
    path = Path(config_path)
    try:
        lines = path.read_text(encoding="utf-8").splitlines(keepends=True)
    except OSError as exc:
        return False, f"yaml unchanged: {type(exc).__name__}: {exc}"

    in_floor_block = False
    block_indent = -1
    for index, line in enumerate(lines):
        stripped = line.strip()
        indent = len(line) - len(line.lstrip())
        if stripped.startswith("floor_constraint:"):
            in_floor_block = True
            block_indent = indent
            continue
        if not in_floor_block:
            continue
        # Leaving the block: a non-empty, non-comment line at or above the
        # block's own indentation level.
        if stripped and not stripped.startswith("#") and indent <= block_indent:
            break
        match = _FLOOR_Z_LINE_RE.match(line.rstrip("\n"))
        if match:
            newline = "\n" if line.endswith("\n") else ""
            lines[index] = f"{match.group(1)}{floor_z_m:.3f}{match.group(3)}{newline}"
            try:
                path.write_text("".join(lines), encoding="utf-8")
            except OSError as exc:
                return False, f"yaml unchanged: {type(exc).__name__}: {exc}"
            return True, f"saved z_min_m={floor_z_m:.3f} to {path.name}"
    return False, f"yaml unchanged: floor_constraint.z_min_m not found in {path.name}"

Mode = Literal["mock", "simulation", "real"]
Backend = Literal["mock", "simulator", "rbpodo", "unknown"]

_VALID_MODES: set[str] = {"mock", "simulation", "real"}
_VALID_BACKENDS: set[str] = {"mock", "simulator", "rbpodo", "unknown"}
_SIMULATOR_ALIASES: set[str] = {"rbsim_local", "rbsim", "rb_simulator"}


@dataclass(frozen=True)
class ObservedModeBackend:
    mode: Mode
    backend: Backend
    warnings: tuple[str, ...] = ()


def normalize_observed_mode_backend(mode: str | None, backend: str | None = None) -> ObservedModeBackend:
    raw_mode = (mode or "mock").strip()
    raw_backend = (backend or "").strip()
    warnings: list[str] = []

    if raw_mode in _SIMULATOR_ALIASES:
        normalized_mode: Mode = "simulation"
        warnings.append(f"deprecated observed mode {raw_mode!r} normalized to 'simulation'")
    elif raw_mode in _VALID_MODES:
        normalized_mode = raw_mode  # type: ignore[assignment]
    else:
        normalized_mode = "mock"
        warnings.append(f"invalid observed mode {raw_mode!r} normalized to 'mock'")

    if raw_backend in _SIMULATOR_ALIASES:
        normalized_backend: Backend = "simulator"
        warnings.append(f"deprecated observed backend {raw_backend!r} normalized to 'simulator'")
    elif raw_backend in _VALID_BACKENDS:
        normalized_backend = raw_backend  # type: ignore[assignment]
    elif raw_backend:
        normalized_backend = "unknown"
        warnings.append(f"unknown observed backend {raw_backend!r} displayed as 'unknown'")
    elif normalized_mode == "simulation":
        normalized_backend = "simulator"
    elif normalized_mode == "real":
        normalized_backend = "rbpodo"
    else:
        normalized_backend = "mock"

    return ObservedModeBackend(normalized_mode, normalized_backend, tuple(warnings))


@dataclass(frozen=True)
class Readiness:
    configured: bool = True
    running: bool = False
    connected: bool = False
    ready: bool = False
    fault: bool = False
    no_go_reason: str = ""
    cartesian_available: bool | None = None
    cartesian_no_go_reason: str = ""


class OperatorSafety:
    lifecycle_modes = {"ArmMotion", "DisarmMotion", "Hold", "EmergencyStop", "ResetFault"}
    motion_modes = {
        "JointTarget",
        "JointVelocity",
        "TcpPoseTarget",
        "TcpDeltaStand",
        "TcpDeltaLocal",
        "TcpLinearMove",
        "TcpTwistStand",
        "TcpTwistLocal",
        "TcpCircleMove",
    }
    # Actions that are always allowed (no motion gate): emergency/stop + fault reset.
    _non_motion_actions = {"EmergencyStop", "Hold", "ResetFault"}
    _latched_motion_states = {"FaultLatched", "EmergencyLatched"}

    def __init__(
        self,
        store: StateStore,
        command_client: CommandClient,
        *,
        desired_mode: Mode | str = "mock",
        observed_server_mode: Mode | str = "mock",
        observed_backend: Backend | str | None = None,
        ops_available: bool = False,
        max_jog_step_deg: float = 2.0,
        max_tcp_linear_step_m: float = 0.005,
        max_tcp_angular_step_rad: float = 0.02,
        max_tcp_linear_velocity_m_s: float = 0.05,
        max_tcp_angular_velocity_rad_s: float = 0.2,
        max_joint_velocity_deg_s: float = 10.0,
        command_timeout_sec: float = 0.2,
        init_left_joint_deg: tuple[float, ...] | None = None,
        init_right_joint_deg: tuple[float, ...] | None = None,
        init_motion_timeout_sec: float = 10.0,
    ) -> None:
        desired = normalize_observed_mode_backend(str(desired_mode), None)
        observed = normalize_observed_mode_backend(str(observed_server_mode), None if observed_backend is None else str(observed_backend))
        self.store = store
        self.command_client = command_client
        self.desired_mode = desired.mode
        self.observed_server_mode = observed.mode
        self.observed_backend = observed.backend
        self.config_warnings = desired.warnings + observed.warnings
        self.ops_available = ops_available
        self.max_jog_step_deg = float(max_jog_step_deg)
        self.max_tcp_linear_step_m = float(max_tcp_linear_step_m)
        self.max_tcp_angular_step_rad = float(max_tcp_angular_step_rad)
        self.max_tcp_linear_velocity_m_s = float(max_tcp_linear_velocity_m_s)
        self.max_tcp_angular_velocity_rad_s = float(max_tcp_angular_velocity_rad_s)
        self.max_joint_velocity_deg_s = float(max_joint_velocity_deg_s)
        self.command_timeout_sec = float(command_timeout_sec)
        self.init_left_joint_deg = self._validated_joint6(init_left_joint_deg)
        self.init_right_joint_deg = self._validated_joint6(init_right_joint_deg)
        self.init_motion_timeout_sec = float(init_motion_timeout_sec)
        self.status_message = "; ".join(self.config_warnings) if self.config_warnings else "starting"
        self.recording_intent = False
        self.last_tcp_command = "none"

    def set_desired_mode(self, mode: Mode | str) -> None:
        normalized = normalize_observed_mode_backend(str(mode), None)
        self.desired_mode = normalized.mode
        if not self.ops_available:
            self.status_message = f"desired {self.desired_mode}; running process is not reconfigured without ops surface"

    def latest_valid(self) -> StateSnapshot | None:
        latest = self.store.latest()
        if latest is None or self.store.is_stale():
            return None
        return latest

    @staticmethod
    def _validated_joint6(values: tuple[float, ...] | None) -> tuple[float, ...] | None:
        if values is None or len(values) != 6:
            return None
        try:
            parsed = tuple(float(value) for value in values)
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(value) for value in parsed):
            return None
        return parsed

    def readiness(self) -> Readiness:
        # Derived purely from the live server state stream: the server is the
        # sole motion authority, so the GUI mirrors what it reports instead of
        # consulting launch-time env flags. cartesian_available reflects the same
        # state-derived TCP gate the buttons use.
        latest = self.latest_valid()
        if latest is None:
            return Readiness(configured=True, no_go_reason="state stream missing or stale")
        connected = latest.left.connection_state == "Connected" and latest.right.connection_state == "Connected"
        joint_valid = latest.left.has_valid_joint_state and latest.right.has_valid_joint_state
        backend_fault = bool(latest.left.error_code or latest.right.error_code or latest.safety_verdict in {"Fault", "BackendFault", "RobotFault"})
        server_fault = latest.fault_latched or latest.motion_state in self._latched_motion_states
        cartesian_reason = self.tcp_command_disabled_reason()
        cartesian_kwargs = {
            "cartesian_available": cartesian_reason is None,
            "cartesian_no_go_reason": cartesian_reason or "",
        }
        if server_fault:
            return Readiness(configured=True, running=True, connected=connected, ready=False, fault=True, no_go_reason="server fault latched", **cartesian_kwargs)
        if backend_fault:
            return Readiness(configured=True, running=True, connected=connected, ready=False, fault=True, no_go_reason="robot/backend fault", **cartesian_kwargs)
        if not joint_valid:
            return Readiness(configured=True, running=True, connected=connected, ready=False, fault=False, no_go_reason="joint state invalid", **cartesian_kwargs)
        return Readiness(configured=True, running=True, connected=connected, ready=connected, fault=False, **cartesian_kwargs)

    def blocked_reason(self, action: str) -> str | None:
        # State-derived, server-as-authority gate. Real/sim execution gating and
        # env readiness retired: mode mismatch never blocks; only a missing/stale
        # state stream, invalid joints, or a latched fault stop a motion command.
        latest = self.latest_valid()
        if latest is None:
            return "state stream missing or stale"
        motion = action not in self._non_motion_actions
        if motion and (not latest.left.has_valid_joint_state or not latest.right.has_valid_joint_state):
            return "joint state invalid"
        if motion and latest.motion_state in self._latched_motion_states:
            return f"motion latched ({latest.motion_state}); reset fault before motion"
        if motion and latest.fault_latched:
            return f"fault latched: {latest.fault_reason or 'unknown'}; reset and arm before motion"
        return None

    @staticmethod
    def _arm_has_tcp_pose(latest: StateSnapshot, arm: Literal["left", "right"]) -> bool:
        arm_state = latest.left if arm == "left" else latest.right
        return bool(arm_state.has_valid_tcp_pose and arm_state.tcp_stand is not None and not arm_state.tcp_deferred)

    @staticmethod
    def _arm_cartesian_reason(latest: StateSnapshot, arm: Literal["left", "right"]) -> str | None:
        """None when this arm can take a Cartesian command, else the reason.

        Availability comes straight from the server: the controller-simulation
        streaming flag wins when present, otherwise the plain Cartesian gate.
        An unknown (None) gate is treated as available — the server rejects the
        command itself if Cartesian is truly closed."""
        arm_state = latest.left if arm == "left" else latest.right
        if not (arm_state.has_valid_tcp_pose and arm_state.tcp_stand is not None and not arm_state.tcp_deferred):
            return f"{arm} FK/TCP pose unavailable"
        available = arm_state.controller_simulation_cartesian_available
        if available is None:
            available = arm_state.cartesian_available
        if available is False:
            return arm_state.cartesian_unavailable_reason or f"{arm} Cartesian unavailable (server gate)"
        return None

    def tcp_command_disabled_reason(self, arm: Literal["left", "right"] | None = None) -> str | None:
        # RB_GUI_ENABLE_TCP_POSE_COMMANDS / mode / backend / env-readiness locks
        # retired: TCP pose commands are available in every run mode. Availability
        # is derived from the live per-arm server Cartesian gate; the server stays
        # the authority (CartesianUnavailable, safety clamps, fault latch).
        reason = self.blocked_reason("TcpDeltaStand")
        if reason:
            return reason
        latest = self.latest_valid()
        if latest is None:
            return "state stream missing or stale"
        if arm is not None:
            return self._arm_cartesian_reason(latest, arm)
        left_reason = self._arm_cartesian_reason(latest, "left")
        right_reason = self._arm_cartesian_reason(latest, "right")
        if left_reason is None or right_reason is None:
            return None
        return left_reason

    def control_disabled_states(self) -> dict[str, bool]:
        """Return visual disabled-state for controls.

        Callback-level blocking remains the authority; this method keeps the GUI
        honest by disabling controls whenever an action would be rejected. Keys
        cover every motion tab (joint jog/velocity, TCP pose/linear/twist, circle,
        lifecycle) so a control is never live-but-dead.
        """
        tcp_reason = self.tcp_command_disabled_reason()
        states: dict[str, bool] = {
            "jog": self.blocked_reason("JointTarget") is not None,
            "velocity": self.blocked_reason("JointVelocity") is not None,
            "init_motion": self.init_motion_disabled_reason() is not None,
            "tcp_pose": tcp_reason is not None,
            "tcp_linear": tcp_reason is not None,
            "twist": tcp_reason is not None,
            "circle": tcp_reason is not None,
        }
        for mode in self.lifecycle_modes:
            states[f"lifecycle:{mode}"] = self.blocked_reason(mode) is not None
        return states

    def control_disabled_reasons(self) -> dict[str, str | None]:
        """Block reason per control key (None when allowed).

        Mirrors control_disabled_states but carries the human-readable reason so
        button-group tabs (which viser cannot grey) can surface it proactively in
        their status line instead of only after a rejected click."""
        tcp_reason = self.tcp_command_disabled_reason()
        return {
            "jog": self.blocked_reason("JointTarget"),
            "velocity": self.blocked_reason("JointVelocity"),
            "init_motion": self.init_motion_disabled_reason(),
            "tcp_pose": tcp_reason,
            "tcp_linear": tcp_reason,
            "twist": tcp_reason,
            "circle": tcp_reason,
        }

    def send_lifecycle(self, mode: str) -> tuple[bool, str]:
        if mode not in self.lifecycle_modes:
            return False, f"unsupported lifecycle mode {mode}"
        reason = self.blocked_reason(mode)
        if reason:
            return False, reason
        self.command_client.send_lifecycle(mode, timeout_sec=self.command_timeout_sec)
        return True, f"sent {mode}"

    def send_set_floor_z(self, floor_z_m: float) -> tuple[bool, str]:
        # Non-motion, leaseless safety adjustment: no motion-block check, but
        # require a live state stream reporting the constraint enabled so the
        # operator sees the applied value (the server bounds-checks the request).
        latest = self.latest_valid()
        if latest is None:
            return False, "state stream missing or stale"
        floor = latest.floor_constraint
        if floor is None or not bool(floor.get("enabled", False)):
            return False, "floor constraint disabled on server"
        try:
            self.command_client.send_set_safety_floor_z(
                float(floor_z_m), timeout_sec=self.command_timeout_sec
            )
        except ValueError as exc:
            return False, str(exc)
        sent = f"sent SetSafetyFloorZ {float(floor_z_m) * 1000:.0f}mm"
        # Persist to the server config yaml (explicit user request: a viser
        # "Send floor z" is also the new startup default). Requires the stack
        # launcher to expose the config path; a send without it stays
        # runtime-only and says so.
        config_path = os.environ.get("RB_GUI_SERVER_CONFIG_PATH", "").strip()
        if not config_path:
            return True, f"{sent} (runtime only: RB_GUI_SERVER_CONFIG_PATH not set)"
        _, save_message = persist_floor_z_to_config(config_path, float(floor_z_m))
        return True, f"{sent} ({save_message})"

    def init_motion_disabled_reason(self) -> str | None:
        reason = self.blocked_reason("JointTarget")
        if reason:
            return reason
        if self.init_left_joint_deg is None or self.init_right_joint_deg is None:
            return "init motion target not configured"
        if self.init_motion_timeout_sec <= 0.0 or not math.isfinite(self.init_motion_timeout_sec):
            return "init motion timeout must be positive and finite"
        latest = self.latest_valid()
        if latest is None:
            return "state stream missing or stale"
        if latest.motion_state not in {"ArmedHold", "Running"}:
            return "ArmMotion first; init motion requires ArmedHold or Running"
        return None

    def send_init_motion(self) -> tuple[bool, str]:
        reason = self.init_motion_disabled_reason()
        if reason:
            return False, reason
        assert self.init_left_joint_deg is not None
        assert self.init_right_joint_deg is not None
        self.command_client.send(
            self.command_client.build_joint_target(
                self.init_left_joint_deg,
                self.init_right_joint_deg,
                timeout_sec=self.init_motion_timeout_sec,
            )
        )
        return True, "sent InitMotion JointTarget"

    def set_init_joints(
        self,
        left_q_deg: tuple[float, ...] | None,
        right_q_deg: tuple[float, ...] | None,
    ) -> tuple[bool, str]:
        # Update the InitMotion target pose at runtime (used by the WayPoint
        # "set as init" button). Persistence to JSON is handled by the caller.
        left = self._validated_joint6(left_q_deg)
        right = self._validated_joint6(right_q_deg)
        if left is None or right is None:
            return False, "init joints require finite 6-DOF values for both arms"
        self.init_left_joint_deg = left
        self.init_right_joint_deg = right
        return True, "init motion pose updated"

    def jog_joint(self, arm: Literal["left", "right"], joint_index: int, delta_deg: float) -> tuple[bool, str]:
        reason = self.blocked_reason("JointTarget")
        if reason:
            return False, reason
        latest = self.latest_valid()
        if latest is None:
            return False, "state stream missing or stale"
        if joint_index < 0 or joint_index >= 6:
            return False, "joint index out of range"
        if not math.isfinite(delta_deg):
            return False, "non-finite jog delta rejected"
        if latest.left.q_sent_deg is None or latest.left.q_actual_deg is None or latest.right.q_sent_deg is None or latest.right.q_actual_deg is None:
            return False, "joint state invalid"
        clamped_delta = max(-self.max_jog_step_deg, min(self.max_jog_step_deg, float(delta_deg)))
        left = list(latest.left.q_sent_deg if latest.left.has_valid_joint_state else latest.left.q_actual_deg)
        right = list(latest.right.q_sent_deg if latest.right.has_valid_joint_state else latest.right.q_actual_deg)
        target = left if arm == "left" else right
        target[joint_index] += clamped_delta
        if not all(math.isfinite(v) for v in left + right):
            return False, "non-finite target rejected"
        self.command_client.send(self.command_client.build_joint_target(tuple(left), tuple(right), timeout_sec=self.command_timeout_sec))
        return True, f"sent {arm} J{joint_index + 1} jog {clamped_delta:+.3f} deg"

    def send_joint_target(
        self,
        *,
        left_q_deg: tuple[float, ...] | None,
        right_q_deg: tuple[float, ...] | None,
    ) -> tuple[bool, str]:
        # Absolute dual-arm JointTarget (used by WayPoint moveJ). Designed to be
        # re-sent at a hold cadence: the standard command_timeout_sec freshness
        # acts as the deadman, so releasing the hold lets the server hold in
        # place once commands go stale.
        reason = self.blocked_reason("JointTarget")
        if reason:
            return False, reason
        if self.latest_valid() is None:
            return False, "state stream missing or stale"
        left = self._validated_joint6(left_q_deg)
        right = self._validated_joint6(right_q_deg)
        if left is None or right is None:
            return False, "joint target requires finite 6-DOF values for both arms"
        self.command_client.send(
            self.command_client.build_joint_target(left, right, timeout_sec=self.command_timeout_sec)
        )
        return True, "sent JointTarget"

    def tcp_jog_unavailable(self) -> tuple[bool, str]:
        return False, "TCP jog unavailable: FK/IK is deferred; no Cartesian motion command sent"

    def _validated_tcp_delta(self, delta: tuple[float, ...], frame_label: str) -> tuple[bool, str | tuple[float, ...]]:
        if len(delta) != 6:
            return False, f"TCP {frame_label} delta must have 6 values"
        try:
            delta_values = tuple(float(value) for value in delta)
        except (TypeError, ValueError):
            return False, f"non-finite TCP {frame_label} delta rejected"
        if any(not math.isfinite(value) for value in delta_values):
            return False, f"non-finite TCP {frame_label} delta rejected"
        if any(abs(value) > self.max_tcp_linear_step_m for value in delta_values[:3]):
            return False, f"TCP linear delta exceeds {self.max_tcp_linear_step_m:.3f} m limit"
        if any(abs(value) > self.max_tcp_angular_step_rad for value in delta_values[3:]):
            return False, f"TCP angular delta exceeds {self.max_tcp_angular_step_rad:.3f} rad limit"
        return True, delta_values

    def _tcp_delta_label(self, delta_values: tuple[float, ...]) -> str:
        axis_names = ("X", "Y", "Z", "roll", "pitch", "yaw")
        moved = [
            f"{'+' if value >= 0.0 else '-'}{axis_names[index]} {abs(value):.3f}{' m' if index < 3 else ' rad'}"
            for index, value in enumerate(delta_values)
            if abs(value) > 0.0
        ]
        return ", ".join(moved) if moved else "zero delta"

    def send_tcp_delta_stand(
        self,
        arm: Literal["left", "right"],
        delta: tuple[float, ...],
    ) -> tuple[bool, str]:
        reason = self.tcp_command_disabled_reason(arm)
        if reason:
            return False, reason
        ok, validated = self._validated_tcp_delta(delta, "stand")
        if not ok:
            return False, str(validated)
        delta_values = validated  # type: ignore[assignment]
        packet = self.command_client.build_tcp_delta_stand(
            left_delta=delta_values if arm == "left" else None,
            right_delta=delta_values if arm == "right" else None,
            timeout_sec=self.command_timeout_sec,
        )
        self.command_client.send(packet)
        self.last_tcp_command = f"TcpDeltaStand {arm} {self._tcp_delta_label(delta_values)}"
        latest = self.latest_valid()
        verdict = latest.safety_verdict if latest is not None else "unavailable"
        return True, f"sent {self.last_tcp_command}; server verdict: {verdict}"

    def send_tcp_delta_local(
        self,
        arm: Literal["left", "right"],
        delta: tuple[float, ...],
    ) -> tuple[bool, str]:
        reason = self.tcp_command_disabled_reason(arm)
        if reason:
            return False, reason
        ok, validated = self._validated_tcp_delta(delta, "local")
        if not ok:
            return False, str(validated)
        delta_values = validated  # type: ignore[assignment]
        packet = self.command_client.build_tcp_delta_local(
            left_delta=delta_values if arm == "left" else None,
            right_delta=delta_values if arm == "right" else None,
            timeout_sec=self.command_timeout_sec,
        )
        self.command_client.send(packet)
        self.last_tcp_command = f"TcpDeltaLocal {arm} {self._tcp_delta_label(delta_values)}"
        latest = self.latest_valid()
        verdict = latest.safety_verdict if latest is not None else "unavailable"
        return True, f"sent {self.last_tcp_command}; server verdict: {verdict}"

    def _validated_tcp_twist(self, twist: tuple[float, ...]) -> tuple[bool, object]:
        try:
            twist_values = tuple(float(value) for value in twist)
        except (TypeError, ValueError):
            return False, "non-finite TCP twist rejected"
        if len(twist_values) != 6 or any(not math.isfinite(v) for v in twist_values):
            return False, "non-finite TCP twist rejected"
        if any(abs(v) > self.max_tcp_linear_velocity_m_s for v in twist_values[:3]):
            return False, f"TCP linear velocity exceeds {self.max_tcp_linear_velocity_m_s:.3f} m/s limit"
        if any(abs(v) > self.max_tcp_angular_velocity_rad_s for v in twist_values[3:]):
            return False, f"TCP angular velocity exceeds {self.max_tcp_angular_velocity_rad_s:.3f} rad/s limit"
        return True, twist_values

    def _send_tcp_twist(
        self,
        arm: Literal["left", "right"],
        twist: tuple[float, ...],
        *,
        frame: Literal["local", "stand"],
    ) -> tuple[bool, str]:
        reason = self.tcp_command_disabled_reason(arm)
        if reason:
            return False, reason
        ok, validated = self._validated_tcp_twist(twist)
        if not ok:
            return False, str(validated)
        twist_values = validated  # type: ignore[assignment]
        if frame == "local":
            packet = self.command_client.build_tcp_twist_local(
                left_twist=twist_values if arm == "left" else None,
                right_twist=twist_values if arm == "right" else None,
                timeout_sec=self.command_timeout_sec,
            )
            mode_name = "TcpTwistLocal"
        else:
            packet = self.command_client.build_tcp_twist_stand(
                left_twist=twist_values if arm == "left" else None,
                right_twist=twist_values if arm == "right" else None,
                timeout_sec=self.command_timeout_sec,
            )
            mode_name = "TcpTwistStand"
        self.command_client.send(packet)
        self.last_tcp_command = f"{mode_name} {arm}"
        latest = self.latest_valid()
        verdict = latest.safety_verdict if latest is not None else "unavailable"
        return True, f"sent {self.last_tcp_command}; server verdict: {verdict}"

    def send_tcp_twist_local(self, arm: Literal["left", "right"], twist: tuple[float, ...]) -> tuple[bool, str]:
        return self._send_tcp_twist(arm, twist, frame="local")

    def send_tcp_twist_stand(self, arm: Literal["left", "right"], twist: tuple[float, ...]) -> tuple[bool, str]:
        return self._send_tcp_twist(arm, twist, frame="stand")

    def send_joint_velocity(
        self,
        arm: Literal["left", "right"],
        velocity: tuple[float, ...],
    ) -> tuple[bool, str]:
        reason = self.blocked_reason("JointVelocity")
        if reason:
            return False, reason
        try:
            vel = tuple(float(value) for value in velocity)
        except (TypeError, ValueError):
            return False, "non-finite joint velocity rejected"
        if len(vel) != 6 or any(not math.isfinite(v) for v in vel):
            return False, "non-finite joint velocity rejected"
        if any(abs(v) > self.max_joint_velocity_deg_s for v in vel):
            return False, f"joint velocity exceeds {self.max_joint_velocity_deg_s:.3f} deg/s limit"
        packet = self.command_client.build_joint_velocity(
            left_velocity=vel if arm == "left" else None,
            right_velocity=vel if arm == "right" else None,
            timeout_sec=self.command_timeout_sec,
        )
        self.command_client.send(packet)
        self.last_tcp_command = f"JointVelocity {arm}"
        latest = self.latest_valid()
        verdict = latest.safety_verdict if latest is not None else "unavailable"
        return True, f"sent JointVelocity {arm}; server verdict: {verdict}"

    def build_circle_packet(
        self,
        diameter_m: float = 0.15,
        period_sec: float = 4.0,
        *,
        arm: Literal["left", "right", "both"] = "both",
        plane: str = "xy",
        repeat: int = 50,
    ) -> tuple[bool, str, dict[str, object] | None]:
        """Validate + build ONE TcpCircleMove packet (fixed seq, full payload).

        The caller re-sends the SAME returned packet to keep the circle fresh;
        sending a new seq would reset the circle to the current TCP.
        """
        arms = ("left", "right") if arm == "both" else (arm,)
        for one in arms:
            reason = self.tcp_command_disabled_reason(one)  # type: ignore[arg-type]
            if reason:
                return False, reason, None
        try:
            diameter = float(diameter_m)
            period = float(period_sec)
        except (TypeError, ValueError):
            return False, "non-finite circle parameters rejected", None
        if not (math.isfinite(diameter) and math.isfinite(period)):
            return False, "non-finite circle parameters rejected", None
        if diameter <= 0.0 or diameter > 0.20:
            return False, "circle diameter must be in (0, 0.20] m", None
        if period < 3.0:
            return False, "circle period must be >= 3.0 s", None
        if plane not in {"xy", "xz", "yz"}:
            return False, "circle plane must be xy, xz, or yz", None
        if int(repeat) < 1:
            return False, "circle repeat must be >= 1", None
        packet = self.command_client.build_tcp_circle_move(
            left=arm in {"left", "both"},
            right=arm in {"right", "both"},
            diameter_m=diameter,
            period_sec=period,
            plane=plane,
            repeat=int(repeat),
        )
        return True, f"TcpCircleMove {arm} d={diameter:.3f}m p={period:.2f}s plane={plane}", packet

    def send_tcp_circle_move(
        self,
        diameter_m: float = 0.15,
        period_sec: float = 4.0,
        *,
        arm: Literal["left", "right", "both"] = "both",
        plane: str = "xy",
        repeat: int = 50,
    ) -> tuple[bool, str]:
        ok, message, packet = self.build_circle_packet(
            diameter_m, period_sec, arm=arm, plane=plane, repeat=repeat
        )
        if not ok or packet is None:
            return False, message
        self.command_client.send(packet)
        self.last_tcp_command = message
        latest = self.latest_valid()
        verdict = latest.safety_verdict if latest is not None else "unavailable"
        return True, f"sent {message}; server verdict: {verdict}"

    def send_tcp_pose_target(
        self,
        *,
        left_pose: tuple[float, ...] | None = None,
        right_pose: tuple[float, ...] | None = None,
        left_quaternion_xyzw: tuple[float, ...] | None = None,
        right_quaternion_xyzw: tuple[float, ...] | None = None,
    ) -> tuple[bool, str]:
        reason = self.tcp_command_disabled_reason()
        if reason:
            return False, reason
        if left_pose is None and right_pose is None:
            return False, "no TCP target selected"
        values = (
            list(left_pose or ())
            + list(right_pose or ())
            + list(left_quaternion_xyzw or ())
            + list(right_quaternion_xyzw or ())
        )
        try:
            finite_values = [float(value) for value in values]
        except (TypeError, ValueError):
            return False, "non-finite TCP target rejected"
        if any(not math.isfinite(value) for value in finite_values):
            return False, "non-finite TCP target rejected"
        for quaternion in (left_quaternion_xyzw, right_quaternion_xyzw):
            if quaternion is None:
                continue
            if len(quaternion) != 4:
                return False, "TCP target quaternion must have 4 values"
            norm = math.sqrt(sum(float(value) * float(value) for value in quaternion))
            if not math.isfinite(norm) or norm <= 0.0:
                return False, "TCP target quaternion must be non-zero"
        try:
            packet = self.command_client.build_tcp_pose_target(
                left_pose=left_pose,
                right_pose=right_pose,
                left_quaternion_xyzw=left_quaternion_xyzw,
                right_quaternion_xyzw=right_quaternion_xyzw,
                timeout_sec=self.command_timeout_sec,
            )
        except ValueError as exc:
            return False, str(exc)
        self.command_client.send(packet)
        arms = []
        if left_pose is not None:
            arms.append("left")
        if right_pose is not None:
            arms.append("right")
        return True, f"sent {'+'.join(arms)} TCP pose target (server may report CartesianUnavailable until IK is implemented)"

    def send_tcp_linear_move(
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
    ) -> tuple[bool, str]:
        reason = self.tcp_command_disabled_reason()
        if reason:
            return False, reason
        if left_pose is None and right_pose is None:
            return False, "no TCP linear target selected"
        for arm, pose in (("left", left_pose), ("right", right_pose)):
            if pose is None:
                continue
            arm_reason = self.tcp_command_disabled_reason(arm)  # type: ignore[arg-type]
            if arm_reason:
                return False, arm_reason
        values = (
            list(left_pose or ())
            + list(right_pose or ())
            + list(left_quaternion_xyzw or ())
            + list(right_quaternion_xyzw or ())
            + [value for value in (duration_sec, linear_speed_m_s, angular_speed_rad_s) if value is not None]
        )
        try:
            finite_values = [float(value) for value in values]
        except (TypeError, ValueError):
            return False, "non-finite TCP linear target rejected"
        if any(not math.isfinite(value) for value in finite_values):
            return False, "non-finite TCP linear target rejected"
        if duration_sec is None and linear_speed_m_s is None:
            return False, "duration_sec or linear_speed_m_s is required"
        for label, value in (
            ("duration_sec", duration_sec),
            ("linear_speed_m_s", linear_speed_m_s),
            ("angular_speed_rad_s", angular_speed_rad_s),
        ):
            if value is not None and float(value) <= 0.0:
                return False, f"{label} must be positive"
        if str(orientation_mode).strip().lower() not in {"constant", "slerp"}:
            return False, "orientation_mode must be constant or slerp"
        for quaternion in (left_quaternion_xyzw, right_quaternion_xyzw):
            if quaternion is None:
                continue
            if len(quaternion) != 4:
                return False, "TCP linear target quaternion must have 4 values"
            norm = math.sqrt(sum(float(value) * float(value) for value in quaternion))
            if not math.isfinite(norm) or norm <= 0.0:
                return False, "TCP linear target quaternion must be non-zero"
        try:
            packet = self.command_client.build_tcp_linear_move(
                left_pose=left_pose,
                right_pose=right_pose,
                left_quaternion_xyzw=left_quaternion_xyzw,
                right_quaternion_xyzw=right_quaternion_xyzw,
                duration_sec=duration_sec,
                linear_speed_m_s=linear_speed_m_s,
                angular_speed_rad_s=angular_speed_rad_s,
                orientation_mode=orientation_mode,
                timeout_sec=self.command_timeout_sec,
            )
        except ValueError as exc:
            return False, str(exc)
        self.command_client.send(packet)
        arms = []
        if left_pose is not None:
            arms.append("left")
        if right_pose is not None:
            arms.append("right")
        self.last_tcp_command = f"TcpLinearMove {'+'.join(arms)} {str(orientation_mode).strip().lower()}"
        return True, f"sent {'+'.join(arms)} TCP linear move"

    def set_recording_intent(self, active: bool) -> str:
        self.recording_intent = bool(active)
        return "recording intent marked; server logger health is read-only in this milestone"
