from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any

from .config import SafetyConfig
from .robot_state_client import StateSnapshot
from .servo_command_client import CommandIntent


@dataclass(frozen=True)
class SafetyDecision:
    allowed: bool
    reason: str = "ok"


@dataclass(frozen=True)
class ActionRequirements:
    requires_camera: bool = False
    requires_kinematics: bool = False
    simulation_only: bool = False


class SafetyGate:
    def __init__(self, mode: str, config: SafetyConfig, stale_timeout_sec: float):
        self.mode = mode
        self.config = config
        self.stale_timeout_sec = stale_timeout_sec

    def evaluate(
        self,
        snapshot: StateSnapshot | None,
        intent: CommandIntent | None,
        requirements: ActionRequirements | None = None,
        now_monotonic: float | None = None,
    ) -> SafetyDecision:
        if intent is None:
            return SafetyDecision(True)
        requirements = requirements or ActionRequirements()
        if snapshot is None:
            return SafetyDecision(False, "state_stream_absent")
        now = time.monotonic() if now_monotonic is None else now_monotonic
        if snapshot.is_stale(now, self.stale_timeout_sec):
            return SafetyDecision(False, "state_stream_stale")

        payload = snapshot.payload
        if bool(payload.get("fault_latched", False)):
            return SafetyDecision(False, "fault_latched")
        if str(payload.get("motion_state", "")) in {"FaultLatched", "EmergencyLatched"}:
            return SafetyDecision(False, "motion_state_latched")
        if self.config.require_valid_joint_state and not _has_valid_joint_state(payload):
            return SafetyDecision(False, "invalid_joint_state")
        if requirements.requires_camera and self.config.camera_stale:
            return SafetyDecision(False, "camera_stale")
        if requirements.requires_kinematics and not self.config.kinematics_available:
            return SafetyDecision(False, "kinematics_unavailable")
        if requirements.simulation_only and self.mode == "real" and intent.is_motion:
            if not self.config.allow_real_motion:
                return SafetyDecision(False, "real_motion_not_allowed")
        if self.mode == "real" and intent.is_motion and not self.config.allow_real_motion:
            return SafetyDecision(False, "real_motion_not_allowed")
        return SafetyDecision(True)


def _has_valid_joint_state(payload: dict[str, Any]) -> bool:
    for arm in ("left", "right"):
        arm_payload = payload.get(arm, {})
        if not isinstance(arm_payload, dict):
            return False
        if not bool(arm_payload.get("has_valid_joint_state", False)):
            return False
        q = arm_payload.get("q_actual_deg")
        if not isinstance(q, list) or len(q) != 6:
            return False
    return True
