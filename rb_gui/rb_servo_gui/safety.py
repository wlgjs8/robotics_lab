from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Literal

from .command_client import CommandClient
from .models import StateSnapshot
from .state_receiver import StateStore

Mode = Literal["mock", "simulation", "real"]


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
        desired_mode: Mode = "mock",
        observed_server_mode: Mode = "mock",
        sim_readiness: Readiness | None = None,
        ops_available: bool = False,
        max_jog_step_deg: float = 2.0,
        command_timeout_sec: float = 0.2,
    ) -> None:
        self.store = store
        self.command_client = command_client
        self.desired_mode = desired_mode
        self.observed_server_mode = observed_server_mode
        self.sim_readiness = sim_readiness or Readiness(no_go_reason="rbpodo/rbsim readiness not proven")
        self.ops_available = ops_available
        self.max_jog_step_deg = float(max_jog_step_deg)
        self.command_timeout_sec = float(command_timeout_sec)
        self.status_message = "starting"
        self.recording_intent = False

    def set_desired_mode(self, mode: Mode) -> None:
        self.desired_mode = mode
        if not self.ops_available:
            self.status_message = f"desired {mode}; running process is not reconfigured without ops surface"

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
        fault = latest.fault_latched or latest.motion_state in {"FaultLatched", "EmergencyLatched"}
        if self.observed_server_mode == "simulation":
            return self.sim_readiness
        if self.observed_server_mode == "real":
            return Readiness(configured=True, running=True, connected=connected, ready=False, fault=fault, no_go_reason="real mode is connect/status only")
        return Readiness(configured=True, running=True, connected=connected, ready=connected and not fault, fault=fault)

    def blocked_reason(self, action: str) -> str | None:
        if self.desired_mode == "real" or self.observed_server_mode == "real":
            return "real mode is connect/status only; motion commands are disabled"
        if self.desired_mode != self.observed_server_mode:
            return "desired mode differs from observed server mode; no unsafe hot-switch is performed"
        latest = self.latest_valid()
        if latest is None:
            return "state stream missing or stale"
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
            "tcp_pose": self.blocked_reason("TcpPoseTarget") is not None,
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
