from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Literal

from .command_client import CommandClient
from .models import StateSnapshot
from .state_receiver import StateStore

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


class OperatorSafety:
    lifecycle_modes = {"ArmMotion", "DisarmMotion", "Hold", "EmergencyStop", "ResetFault"}
    motion_modes = {"JointTarget", "JointVelocity", "TcpPoseTarget", "TcpDeltaStand", "TcpDeltaLocal"}

    def __init__(
        self,
        store: StateStore,
        command_client: CommandClient,
        *,
        desired_mode: Mode | str = "mock",
        observed_server_mode: Mode | str = "mock",
        observed_backend: Backend | str | None = None,
        sim_readiness: Readiness | None = None,
        ops_available: bool = False,
        enable_tcp_pose_commands: bool = False,
        max_jog_step_deg: float = 2.0,
        command_timeout_sec: float = 0.2,
    ) -> None:
        desired = normalize_observed_mode_backend(str(desired_mode), None)
        observed = normalize_observed_mode_backend(str(observed_server_mode), None if observed_backend is None else str(observed_backend))
        self.store = store
        self.command_client = command_client
        self.desired_mode = desired.mode
        self.observed_server_mode = observed.mode
        self.observed_backend = observed.backend
        self.config_warnings = desired.warnings + observed.warnings
        self.sim_readiness = sim_readiness or Readiness(no_go_reason="simulator/rbpodo readiness not proven")
        self.ops_available = ops_available
        self.enable_tcp_pose_commands = bool(enable_tcp_pose_commands)
        self.max_jog_step_deg = float(max_jog_step_deg)
        self.command_timeout_sec = float(command_timeout_sec)
        self.status_message = "; ".join(self.config_warnings) if self.config_warnings else "starting"
        self.recording_intent = False

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

    def readiness(self) -> Readiness:
        latest = self.latest_valid()
        if latest is None:
            return Readiness(configured=True, no_go_reason="state stream missing or stale")
        connected = latest.left.connection_state == "Connected" and latest.right.connection_state == "Connected"
        joint_valid = latest.left.has_valid_joint_state and latest.right.has_valid_joint_state
        backend_fault = bool(latest.left.error_code or latest.right.error_code or latest.safety_verdict in {"Fault", "BackendFault", "RobotFault"})
        server_fault = latest.fault_latched or latest.motion_state in {"FaultLatched", "EmergencyLatched"}
        fault = server_fault or backend_fault
        if self.observed_server_mode == "simulation":
            if server_fault:
                return Readiness(configured=True, running=True, connected=connected, ready=False, fault=True, no_go_reason="server fault latched")
            if backend_fault:
                return Readiness(configured=True, running=True, connected=connected, ready=False, fault=True, no_go_reason="robot/backend fault")
            if not joint_valid:
                return Readiness(configured=True, running=True, connected=connected, ready=False, fault=False, no_go_reason="joint state invalid")
            return Readiness(
                configured=self.sim_readiness.configured,
                running=self.sim_readiness.running,
                connected=connected and self.sim_readiness.connected,
                ready=connected and self.sim_readiness.ready,
                fault=False,
                no_go_reason="" if connected and self.sim_readiness.ready else (self.sim_readiness.no_go_reason or "simulation readiness tests have not passed"),
            )
        if self.observed_server_mode == "real":
            return Readiness(configured=True, running=True, connected=connected, ready=False, fault=fault, no_go_reason="real mode is connect/status only")
        if server_fault:
            return Readiness(configured=True, running=True, connected=connected, ready=False, fault=True, no_go_reason="server fault latched")
        if backend_fault:
            return Readiness(configured=True, running=True, connected=connected, ready=False, fault=True, no_go_reason="robot/backend fault")
        if not joint_valid:
            return Readiness(configured=True, running=True, connected=connected, ready=False, fault=False, no_go_reason="joint state invalid")
        return Readiness(configured=True, running=True, connected=connected, ready=connected, fault=False)

    def blocked_reason(self, action: str) -> str | None:
        if self.desired_mode == "real" or self.observed_server_mode == "real":
            return "real mode is connect/status only; motion commands are disabled"
        if self.desired_mode != self.observed_server_mode:
            return "desired mode differs from observed server mode; no unsafe hot-switch is performed"
        latest = self.latest_valid()
        if latest is None:
            return "state stream missing or stale"
        if (not latest.left.has_valid_joint_state or not latest.right.has_valid_joint_state) and action not in {"EmergencyStop", "Hold", "ResetFault"}:
            return "joint state invalid"
        if self.desired_mode == "simulation":
            if not self.sim_readiness.ready:
                return self.sim_readiness.no_go_reason or "simulation readiness tests have not passed"
        if latest.fault_latched and action not in {"ResetFault", "EmergencyStop", "Hold"}:
            return "fault is latched; reset and arm before motion"
        return None


    def control_disabled_states(self) -> dict[str, bool]:
        """Return visual disabled-state for controls.

        Callback-level blocking remains the authority; this method keeps the GUI
        honest by disabling controls whenever an action would be rejected.
        """
        states: dict[str, bool] = {
            "jog": self.blocked_reason("JointTarget") is not None,
            "tcp_jog": True,
            "tcp_pose": (not self.enable_tcp_pose_commands) or self.blocked_reason("TcpPoseTarget") is not None,
        }
        for mode in self.lifecycle_modes:
            states[f"lifecycle:{mode}"] = self.blocked_reason(mode) is not None
        return states

    def send_lifecycle(self, mode: str) -> tuple[bool, str]:
        if mode not in self.lifecycle_modes:
            return False, f"unsupported lifecycle mode {mode}"
        reason = self.blocked_reason(mode)
        if reason:
            return False, reason
        self.command_client.send_lifecycle(mode, timeout_sec=self.command_timeout_sec)
        return True, f"sent {mode}"

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

    def tcp_jog_unavailable(self) -> tuple[bool, str]:
        return False, "TCP jog unavailable: FK/IK is deferred; no Cartesian motion command sent"

    def send_tcp_pose_target(
        self,
        *,
        left_pose: tuple[float, ...] | None = None,
        right_pose: tuple[float, ...] | None = None,
    ) -> tuple[bool, str]:
        if not self.enable_tcp_pose_commands:
            return False, "TCP pose command disabled until FK/IK milestone is enabled"
        reason = self.blocked_reason("TcpPoseTarget")
        if reason:
            return False, reason
        if left_pose is None and right_pose is None:
            return False, "no TCP target selected"
        values = list(left_pose or ()) + list(right_pose or ())
        try:
            finite_values = [float(value) for value in values]
        except (TypeError, ValueError):
            return False, "non-finite TCP target rejected"
        if any(not math.isfinite(value) for value in finite_values):
            return False, "non-finite TCP target rejected"
        try:
            packet = self.command_client.build_tcp_pose_target(
                left_pose=left_pose,
                right_pose=right_pose,
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

    def set_recording_intent(self, active: bool) -> str:
        self.recording_intent = bool(active)
        return "recording intent marked; server logger health is read-only in this milestone"
