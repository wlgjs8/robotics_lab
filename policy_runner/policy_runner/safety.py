from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any

from .config import SafetyConfig
from .geometry import GeometryStatus
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
    requires_geometry: bool = False
    requires_camera_geometry: bool = False
    requires_valid_tcp_pose: bool = False
    requires_valid_joint_state: bool = True
    simulation_only: bool = False
    requires_observed_simulation: bool = False
    requires_simulator_backend_if_available: bool = False
    cartesian_motion: bool = False


class SafetyGate:
    def __init__(
        self,
        mode: str,
        config: SafetyConfig,
        stale_timeout_sec: float,
        geometry_status: GeometryStatus | None = None,
    ):
        self.mode = mode
        self.config = config
        self.stale_timeout_sec = stale_timeout_sec
        self.geometry_status = geometry_status

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
        if (
            self.config.require_valid_joint_state
            and requirements.requires_valid_joint_state
            and not _has_valid_joint_state(payload)
        ):
            return SafetyDecision(False, "invalid_joint_state")
        if requirements.requires_camera and not self.config.camera_available:
            return SafetyDecision(False, "camera_unavailable")
        if requirements.requires_camera and self.config.camera_stale:
            return SafetyDecision(False, "camera_stale")
        if requirements.requires_kinematics and not self.config.kinematics_available:
            return SafetyDecision(False, "kinematics_unavailable")
        observed_mode = _observed_mode(payload)
        effective_mode = observed_mode or self.mode
        if requirements.cartesian_motion and (observed_mode == "real" or self.mode == "real"):
            return SafetyDecision(False, "real_cartesian_not_allowed")
        if requirements.cartesian_motion and str(payload.get("safety_verdict", "")) == "CartesianUnavailable":
            return SafetyDecision(False, "cartesian_unavailable")
        if requirements.requires_observed_simulation and observed_mode != "simulation":
            return SafetyDecision(False, "observed_mode_not_simulation")
        if requirements.requires_observed_simulation and self.mode != "simulation":
            return SafetyDecision(False, "configured_mode_not_simulation")
        if requirements.requires_simulator_backend_if_available:
            observed_backend = _observed_backend(payload)
            if observed_backend is not None and observed_backend != "simulator":
                return SafetyDecision(False, "observed_backend_not_simulator")
        geometry_decision = self._evaluate_geometry_requirements(requirements, payload, effective_mode)
        if not geometry_decision.allowed:
            return geometry_decision
        if observed_mode == "real" and intent.is_motion and not self.config.allow_real_motion:
            return SafetyDecision(False, "real_motion_not_allowed")
        if requirements.simulation_only and self.mode == "real" and intent.is_motion:
            if not self.config.allow_real_motion:
                return SafetyDecision(False, "real_motion_not_allowed")
        if self.mode == "real" and intent.is_motion and not self.config.allow_real_motion:
            return SafetyDecision(False, "real_motion_not_allowed")
        return SafetyDecision(True)

    def _evaluate_geometry_requirements(
        self,
        requirements: ActionRequirements,
        payload: dict[str, Any],
        effective_mode: str,
    ) -> SafetyDecision:
        if requirements.requires_valid_tcp_pose and not _has_valid_tcp_pose(payload):
            return SafetyDecision(False, "invalid_tcp_pose")
        if not (requirements.requires_geometry or requirements.requires_camera_geometry):
            return SafetyDecision(True)
        if self.geometry_status is None:
            return SafetyDecision(False, "geometry_unavailable")
        status = self.geometry_status
        if status.status == "unavailable":
            return SafetyDecision(False, status.load_error or "geometry_unavailable")
        if not status.robot_mounts_available:
            return SafetyDecision(False, "robot_mount_geometry_unavailable")
        if requirements.requires_camera_geometry and not status.camera_geometry_available:
            return SafetyDecision(False, "camera_geometry_unavailable")
        if status.status == "configured_estimate":
            if effective_mode == "real" and not self.config.allow_configured_estimate_geometry_in_real:
                return SafetyDecision(False, "configured_estimate_geometry_not_allowed_in_real")
            if effective_mode == "simulation" and not self.config.allow_configured_estimate_geometry_in_simulation:
                return SafetyDecision(False, "configured_estimate_geometry_not_allowed_in_simulation")
        elif not _is_real_policy_geometry_status(status.status):
            return SafetyDecision(False, "geometry_status_not_policy_ready")
        if effective_mode == "real" and not status.geometry_valid_for_real_policy:
            return SafetyDecision(False, "geometry_not_valid_for_real_policy")
        return SafetyDecision(True)


def _observed_mode(payload: dict[str, Any]) -> str | None:
    for key in ("observed_mode", "run_mode", "mode"):
        value = payload.get(key)
        if isinstance(value, str) and value.lower() in {"mock", "simulation", "real"}:
            return value.lower()
    return None


def _observed_backend(payload: dict[str, Any]) -> str | None:
    for key in ("observed_backend", "backend_type", "backend"):
        value = payload.get(key)
        if isinstance(value, str) and value.lower() in {"mock", "simulator", "rbpodo"}:
            return value.lower()
    return None


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


def _has_valid_tcp_pose(payload: dict[str, Any]) -> bool:
    for arm in ("left", "right"):
        arm_payload = payload.get(arm, {})
        if not isinstance(arm_payload, dict):
            return False
        if not bool(arm_payload.get("has_valid_tcp_pose", False)):
            return False
    return True


def _is_real_policy_geometry_status(status: str) -> bool:
    return status.strip().lower() in {"measured", "calibrated", "accepted"}
