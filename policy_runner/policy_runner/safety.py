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
    camera_stale_timeout_sec: float | None = None
    requires_kinematics: bool = False
    requires_geometry: bool = False
    requires_camera_geometry: bool = False
    requires_valid_tcp_pose: bool = False
    requires_valid_joint_state: bool = True
    simulation_only: bool = False
    requires_observed_simulation: bool = False
    requires_simulator_backend_if_available: bool = False
    cartesian_motion: bool = False

    def __post_init__(self) -> None:
        if self.camera_stale_timeout_sec is not None and self.camera_stale_timeout_sec <= 0.0:
            raise ValueError("camera_stale_timeout_sec must be positive")


@dataclass(frozen=True)
class CameraReadiness:
    available: bool = False
    last_observed_monotonic_sec: float | None = None
    stale: bool = False


class SafetyGate:
    def __init__(
        self,
        mode: str,
        config: SafetyConfig,
        stale_timeout_sec: float,
        geometry_status: GeometryStatus | None = None,
        camera_readiness: CameraReadiness | None = None,
    ):
        self.mode = mode
        self.config = config
        self.stale_timeout_sec = stale_timeout_sec
        self.geometry_status = geometry_status
        self.camera_readiness = camera_readiness

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
        camera_decision = self._evaluate_camera_requirements(requirements, snapshot, now)
        if not camera_decision.allowed:
            return camera_decision
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

    def _evaluate_camera_requirements(
        self,
        requirements: ActionRequirements,
        snapshot: StateSnapshot,
        now_monotonic: float,
    ) -> SafetyDecision:
        if not requirements.requires_camera:
            return SafetyDecision(True)
        timeout_sec = requirements.camera_stale_timeout_sec or self.config.camera_stale_timeout_sec
        readiness = camera_readiness_from_snapshot(snapshot, now_monotonic) or self.camera_readiness
        if readiness is None:
            if not self.config.camera_available:
                return SafetyDecision(False, "camera_unavailable")
            if self.config.camera_stale:
                return SafetyDecision(False, "camera_stale")
            return SafetyDecision(True)
        if not readiness.available:
            return SafetyDecision(False, "camera_unavailable")
        if readiness.stale:
            return SafetyDecision(False, "camera_stale")
        if readiness.last_observed_monotonic_sec is None:
            return SafetyDecision(False, "camera_unavailable")
        if now_monotonic - readiness.last_observed_monotonic_sec > timeout_sec:
            return SafetyDecision(False, "camera_stale")
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


def camera_readiness_from_snapshot(
    snapshot: StateSnapshot,
    now_monotonic: float | None = None,
) -> CameraReadiness | None:
    raw = snapshot.payload.get("camera_readiness")
    if raw is None:
        raw = snapshot.payload.get("camera")
    if not isinstance(raw, dict):
        return None
    now = time.monotonic() if now_monotonic is None else now_monotonic
    if _camera_readiness_missing(raw):
        return CameraReadiness(available=False)
    available = bool(raw.get("available", raw.get("healthy", False)))
    stale = bool(raw.get("stale", False))
    last_observed = _camera_last_observed_monotonic(raw, now)
    return CameraReadiness(
        available=available,
        last_observed_monotonic_sec=last_observed,
        stale=stale,
    )


def _observed_backend(payload: dict[str, Any]) -> str | None:
    for key in ("observed_backend", "backend_type", "backend"):
        value = payload.get(key)
        if isinstance(value, str) and value.lower() in {"mock", "simulator", "rbpodo"}:
            return value.lower()
    return None


def _camera_readiness_missing(raw: dict[str, Any]) -> bool:
    for key in ("missing", "required_missing", "required_camera_missing"):
        value = raw.get(key)
        if isinstance(value, bool) and value:
            return True
    complete = raw.get("complete")
    if isinstance(complete, bool) and not complete:
        return True
    required_present = raw.get("required_present")
    if isinstance(required_present, bool) and not required_present:
        return True
    return False


def _camera_last_observed_monotonic(raw: dict[str, Any], now_monotonic: float) -> float | None:
    value = raw.get("last_observed_monotonic_sec")
    if _is_number(value):
        return float(value)
    for key in ("latest_observation_age_sec", "last_observation_age_sec", "bundle_age_sec", "latest_bundle_age_sec"):
        value = raw.get(key)
        if _is_number(value):
            return now_monotonic - float(value)
    for key in ("latest_observation_age_ms", "last_observation_age_ms", "bundle_age_ms", "latest_bundle_age_ms"):
        value = raw.get(key)
        if _is_number(value):
            return now_monotonic - (float(value) / 1000.0)
    return None


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


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
