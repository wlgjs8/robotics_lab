from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import re
import threading
import time
from typing import Any, Mapping, Sequence
import uuid

from .cog_calibration import (
    CogCalibrationError,
    CogPoseMeasurement,
    CogSample,
    CogSampleAccumulator,
    GravityWrenchEstimate,
    estimate_controller_compensated_gravity,
    estimate_payload,
    save_blocked_calibration_report,
    save_calibration_report,
)
from .models import StateSnapshot
from .safety import OperatorSafety


_ACTIVE_STATES = {"waiting_lease", "armed", "moving", "settling", "sampling", "review"}
_SENDING_STATES = {"moving", "settling", "sampling"}


def natural_sort_key(value: str) -> tuple[tuple[int, Any], ...]:
    return tuple(
        (1, int(part)) if part.isdigit() else (0, part.casefold())
        for part in re.split(r"(\d+)", value)
        if part
    )


@dataclass(frozen=True)
class CogIdentificationConfig:
    enable: bool
    observation_model: str
    wrench_convention: str | None
    min_poses: int
    arrival_tolerance_deg: float
    settle_sec: float
    samples_per_pose: int
    max_force_stddev_n: float
    max_torque_stddev_nm: float
    max_force_fit_rms_n: float
    max_torque_fit_rms_nm: float
    max_design_condition_number: float
    raw: Mapping[str, Any]

    @classmethod
    def parse(cls, value: Any) -> "CogIdentificationConfig":
        if not isinstance(value, Mapping):
            raise ValueError("server payload-identification config is unavailable")
        if value.get("enable") is not True:
            raise ValueError("payload identification is disabled by server config")
        observation_model = value.get("observation_model")
        if observation_model not in {"rigid_payload", "controller_compensated_linear"}:
            raise ValueError(
                "server payload-identification observation_model must be "
                "rigid_payload or controller_compensated_linear"
            )
        wrench_convention = value.get("wrench_convention")
        if observation_model == "rigid_payload":
            if wrench_convention not in {"payload_load", "sensor_reaction"}:
                raise ValueError(
                    "server payload-identification wrench_convention must be "
                    "payload_load or sensor_reaction for rigid_payload"
                )
        elif wrench_convention not in {None, ""}:
            raise ValueError(
                "server payload-identification wrench_convention must be empty for "
                "controller_compensated_linear"
            )
        else:
            wrench_convention = None

        def finite_positive(name: str, *, allow_zero: bool = False) -> float:
            raw = value.get(name)
            if not isinstance(raw, (int, float)) or isinstance(raw, bool):
                raise ValueError(f"server payload-identification config missing {name}")
            parsed = float(raw)
            if not math.isfinite(parsed) or (parsed < 0.0 if allow_zero else parsed <= 0.0):
                raise ValueError(f"server payload-identification config invalid {name}")
            return parsed

        min_poses = value.get("min_poses")
        samples_per_pose = value.get("samples_per_pose")
        if not isinstance(min_poses, int) or isinstance(min_poses, bool) or min_poses < 5:
            raise ValueError("server payload-identification min_poses must be at least 5")
        if (
            not isinstance(samples_per_pose, int)
            or isinstance(samples_per_pose, bool)
            or samples_per_pose <= 0
        ):
            raise ValueError("server payload-identification samples_per_pose must be positive")
        return cls(
            enable=True,
            observation_model=observation_model,
            wrench_convention=wrench_convention,
            min_poses=min_poses,
            arrival_tolerance_deg=finite_positive("arrival_tolerance_deg"),
            settle_sec=finite_positive("settle_sec", allow_zero=True),
            samples_per_pose=samples_per_pose,
            max_force_stddev_n=finite_positive("max_force_stddev_n"),
            max_torque_stddev_nm=finite_positive("max_torque_stddev_nm"),
            max_force_fit_rms_n=finite_positive("max_force_fit_rms_n"),
            max_torque_fit_rms_nm=finite_positive("max_torque_fit_rms_nm"),
            max_design_condition_number=finite_positive(
                "max_design_condition_number"
            ),
            raw=dict(value),
        )


@dataclass(frozen=True)
class CogWaypoint:
    name: str
    q_target_deg: tuple[float, ...]
    digest: str


def resolve_cog_waypoints(
    waypoints: Mapping[str, Any],
    *,
    prefix: str,
    arm: str,
    min_poses: int,
) -> tuple[CogWaypoint, ...]:
    if arm not in {"left", "right"}:
        raise ValueError("active arm must be left or right")
    clean_prefix = str(prefix).strip()
    if not clean_prefix:
        raise ValueError("enter a non-empty waypoint prefix")
    q_key = f"{arm}_q"
    resolved: list[CogWaypoint] = []
    for name in sorted(
        (key for key in waypoints if isinstance(key, str) and key.startswith(clean_prefix)),
        key=natural_sort_key,
    ):
        entry = waypoints.get(name)
        target = entry.get(q_key) if isinstance(entry, Mapping) else None
        if not isinstance(target, (list, tuple)) or len(target) != 6:
            raise ValueError(f"waypoint {name!r} is missing {q_key}")
        try:
            parsed = tuple(float(item) for item in target)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"waypoint {name!r} has invalid {q_key}") from exc
        if not all(math.isfinite(item) for item in parsed):
            raise ValueError(f"waypoint {name!r} has non-finite {q_key}")
        encoded = json.dumps(parsed, separators=(",", ":")).encode("utf-8")
        resolved.append(CogWaypoint(name, parsed, hashlib.sha256(encoded).hexdigest()))
    if len(resolved) < min_poses:
        raise ValueError(
            f"prefix {clean_prefix!r} resolved {len(resolved)} pose(s); need at least {min_poses}"
        )
    targets = {waypoint.q_target_deg for waypoint in resolved}
    if len(targets) != len(resolved):
        raise ValueError("payload-identification waypoints must have unique joint targets")
    return tuple(resolved)


@dataclass(frozen=True)
class CogGuiStatus:
    state: str
    arm: str | None
    current_index: int
    waypoint_count: int
    current_name: str | None
    max_joint_error_deg: float | None
    settle_elapsed_sec: float
    sample_count: int
    current_force_stddev_n: float | None
    current_torque_stddev_nm: float | None
    accepted_count: int
    message: str
    result: GravityWrenchEstimate | None
    saved_report: str | None
    pose_states: tuple[tuple[str, str], ...]

    @property
    def active(self) -> bool:
        return self.state in _ACTIVE_STATES


class CogGuiSession:
    """Thread-safe GUI orchestration around the server-owned motion/safety path."""

    def __init__(self, safety: OperatorSafety) -> None:
        self._safety = safety
        self._lock = threading.RLock()
        self._state = "idle"
        self._arm: str | None = None
        self._config: CogIdentificationConfig | None = None
        self._waypoints: tuple[CogWaypoint, ...] = ()
        self._pose_states: dict[str, str] = {}
        self._current_index = 0
        self._max_joint_error_deg: float | None = None
        self._settle_started: float | None = None
        self._settle_elapsed_sec = 0.0
        self._accumulator: CogSampleAccumulator | None = None
        self._accepted: list[CogPoseMeasurement] = []
        self._message = "idle"
        self._deadman_until = 0.0
        self._result: GravityWrenchEstimate | None = None
        self._saved_report: str | None = None
        self._run_id: str | None = None
        self._provenance: dict[str, Any] = {}
        self._release_pending = False

    def status(self) -> CogGuiStatus:
        with self._lock:
            current = self._current_waypoint_locked()
            sample_count = self._accumulator.sample_count if self._accumulator else 0
            force_stddev = None
            torque_stddev = None
            if self._accumulator is not None and sample_count > 0:
                summary = self._accumulator.freeze().summary()
                force_stddev = summary.max_force_stddev_n
                torque_stddev = summary.max_torque_stddev_nm
            return CogGuiStatus(
                state=self._state,
                arm=self._arm,
                current_index=self._current_index,
                waypoint_count=len(self._waypoints),
                current_name=current.name if current else None,
                max_joint_error_deg=self._max_joint_error_deg,
                settle_elapsed_sec=self._settle_elapsed_sec,
                sample_count=sample_count,
                current_force_stddev_n=force_stddev,
                current_torque_stddev_nm=torque_stddev,
                accepted_count=len(self._accepted),
                message=self._message,
                result=self._result,
                saved_report=self._saved_report,
                pose_states=tuple(
                    (waypoint.name, self._pose_states.get(waypoint.name, "pending"))
                    for waypoint in self._waypoints
                ),
            )

    def preflight_reason(self, arm: str) -> str | None:
        if arm not in {"left", "right"}:
            return "payload identification arm must be left or right"
        return self._safety.payload_identification_disabled_reason(arm)

    def start(
        self,
        *,
        arm: str,
        waypoints: Sequence[CogWaypoint],
        config: CogIdentificationConfig,
    ) -> tuple[bool, str]:
        with self._lock:
            if self._state in _ACTIVE_STATES:
                return False, "a payload-identification session is already active"
            if len(waypoints) < config.min_poses:
                return False, f"need at least {config.min_poses} resolved waypoints"
            ok, message = self._safety.begin_payload_identification_session(arm)  # no motion
            if not ok:
                self._state = "blocked"
                self._message = message
                return False, message
            self._state = "waiting_lease"
            self._arm = arm
            self._config = config
            self._waypoints = tuple(waypoints)
            self._pose_states = {waypoint.name: "pending" for waypoint in waypoints}
            self._current_index = 0
            self._max_joint_error_deg = None
            self._settle_started = None
            self._settle_elapsed_sec = 0.0
            self._accumulator = None
            self._accepted = []
            self._result = None
            self._saved_report = None
            self._run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ-") + uuid.uuid4().hex[:8]
            self._provenance = {
                "server_config": dict(config.raw),
                "waypoints": [
                    {"name": item.name, "q_target_deg": item.q_target_deg, "sha256": item.digest}
                    for item in waypoints
                ],
            }
            self._deadman_until = 0.0
            self._message = message
            self._release_pending = False
            return True, message

    def run_pulse(self, latest: StateSnapshot | None, *, stale: bool) -> tuple[bool, str]:
        now = time.monotonic()
        with self._lock:
            if self._state == "waiting_lease":
                if not self._lease_owned_locked(latest):
                    self._message = "waiting for server lease ownership confirmation"
                    return False, self._message
                self._state = "armed"
                self._message = "lease confirmed; hold Run/Continue to move"
            if self._state not in {"armed", "moving", "settling", "sampling"}:
                return False, f"Run/Continue unavailable while session is {self._state}"
            reason = self._health_reason_locked(latest, stale=stale, require_profile=False)
            if reason:
                self._block_locked(reason)
                return False, reason
            current = self._current_waypoint_locked()
            if current is None or self._arm is None:
                self._block_locked("current payload-identification waypoint is unavailable")
                return False, self._message
            assert self._config is not None
            self._deadman_until = now + self._safety.command_timeout_sec
            if self._state == "armed":
                self._state = "moving"
                self._pose_states[current.name] = "moving"
            ok, message = self._safety.send_payload_identification_target(
                arm=self._arm, q_target_deg=current.q_target_deg
            )
            if not ok:
                self._block_locked(message)
                return False, message
            self._message = message
            return True, message

    def on_snapshot(self, latest: StateSnapshot) -> None:
        now = time.monotonic()
        with self._lock:
            if self._state == "waiting_lease" and self._lease_owned_locked(latest):
                self._state = "armed"
                self._message = "lease confirmed; hold Run/Continue to move"
                return
            if self._state not in _SENDING_STATES:
                return
            if now > self._deadman_until:
                self._pause_locked("Run/Continue released; command renewal stopped")
                return
            reason = self._health_reason_locked(latest, stale=False, require_profile=False)
            if reason:
                self._block_locked(reason)
                return
            current = self._current_waypoint_locked()
            if current is None or self._arm is None or self._config is None:
                self._block_locked("payload-identification session lost its current target")
                return
            arm_state = latest.left if self._arm == "left" else latest.right
            actual = arm_state.q_actual_deg
            if actual is None:
                self._block_locked(f"{self._arm} actual joints are unavailable")
                return
            error = max(abs(a - b) for a, b in zip(actual, current.q_target_deg))
            self._max_joint_error_deg = error
            sample_time = latest.received_monotonic
            if error > self._config.arrival_tolerance_deg:
                if self._state != "moving":
                    self._accumulator = None
                    self._settle_started = None
                    self._settle_elapsed_sec = 0.0
                self._state = "moving"
                self._pose_states[current.name] = "moving"
                return
            if self._state == "moving":
                self._state = "settling"
                self._pose_states[current.name] = "settling"
                self._settle_started = sample_time
                self._settle_elapsed_sec = 0.0
                return
            if self._state == "settling":
                if self._settle_started is None:
                    self._settle_started = sample_time
                self._settle_elapsed_sec = max(0.0, sample_time - self._settle_started)
                if self._settle_elapsed_sec < self._config.settle_sec:
                    return
                reason = self._health_reason_locked(latest, stale=False, require_profile=True)
                if reason:
                    self._message = reason
                    return
                self._state = "sampling"
                self._pose_states[current.name] = "sampling"
                self._accumulator = CogSampleAccumulator(current.name, current.q_target_deg)
            if self._state != "sampling" or self._accumulator is None:
                return
            reason = self._health_reason_locked(latest, stale=False, require_profile=True)
            if reason:
                self._block_locked(reason)
                return
            ft = arm_state.force_torque
            assert ft is not None
            if ft.freshness_advanced is not True:
                return
            assert ft.freshness_value is not None
            assert ft.raw_sensor_wrench is not None
            assert ft.t_tcp_sensor is not None
            assert ft.wrench_tcp is not None
            assert ft.gravity_tcp is not None
            assert arm_state.q_actual_deg is not None
            assert arm_state.tcp_actual_stand is not None
            self._accumulator.add(
                CogSample(
                    freshness_value=ft.freshness_value,
                    wrench_tcp=ft.wrench_tcp,
                    gravity_tcp=ft.gravity_tcp,
                    received_monotonic=latest.received_monotonic,
                    raw_sensor_wrench=ft.raw_sensor_wrench,
                    t_tcp_sensor=ft.t_tcp_sensor.as_tuple(),
                    q_actual_deg=arm_state.q_actual_deg,
                    tcp_actual_stand=arm_state.tcp_actual_stand.as_tuple(),
                )
            )
            if self._accumulator.sample_count >= self._config.samples_per_pose:
                self._finish_pose_locked(current)

    def watchdog(self, latest: StateSnapshot | None, *, stale: bool) -> None:
        with self._lock:
            if self._state in _SENDING_STATES and time.monotonic() > self._deadman_until:
                self._pause_locked("Run/Continue released; command renewal stopped")
            elif self._state in _ACTIVE_STATES and self._state != "waiting_lease":
                reason = self._health_reason_locked(latest, stale=stale, require_profile=False)
                if reason:
                    self._block_locked(reason)
        self.release_if_pending()

    def retry(self) -> tuple[bool, str]:
        with self._lock:
            if self._state != "review":
                return False, "Retry is available only for a rejected noisy pose"
            current = self._current_waypoint_locked()
            if current is None:
                return False, "current waypoint unavailable"
            self._accumulator = None
            self._settle_started = None
            self._settle_elapsed_sec = 0.0
            self._pose_states[current.name] = "pending"
            self._state = "armed"
            self._message = f"retry {current.name}; hold Run/Continue"
            return True, self._message

    def skip(self) -> tuple[bool, str]:
        with self._lock:
            if self._state not in {"armed", "review"}:
                return False, "Skip is available only while armed or reviewing a pose"
            current = self._current_waypoint_locked()
            if current is None:
                return False, "current waypoint unavailable"
            self._pose_states[current.name] = "skipped"
            self._advance_locked()
            return True, self._message

    def stop(self) -> tuple[bool, str]:
        with self._lock:
            if self._state not in _ACTIVE_STATES and not self._safety.command_client.hold_lease:
                self._state = "stopped"
                self._message = "session stopped"
                return True, self._message
            self._state = "stopped"
            self._message = "session stopped; accepted samples preserved"
            self._release_pending = True
        ok, release_message = self._safety.end_payload_identification_session()
        with self._lock:
            self._release_pending = False
            self._message = f"{self._message}; {release_message}"
            return ok, self._message

    def calculate(self, diagnostic_output_dir: str | None = None) -> tuple[bool, str]:
        with self._lock:
            if self._config is None:
                return False, "payload-identification session has no server config"
            if len(self._accepted) < self._config.min_poses:
                return False, (
                    f"accepted {len(self._accepted)} pose(s); need at least "
                    f"{self._config.min_poses}"
                )
            try:
                if self._config.observation_model == "controller_compensated_linear":
                    estimate = estimate_controller_compensated_gravity(
                        tuple(self._accepted),
                        min_poses=self._config.min_poses,
                        max_condition_number=self._config.max_design_condition_number,
                    )
                else:
                    assert self._config.wrench_convention is not None
                    estimate = estimate_payload(
                        tuple(self._accepted),
                        wrench_convention=self._config.wrench_convention,
                        min_poses=self._config.min_poses,
                        max_condition_number=self._config.max_design_condition_number,
                    )
            except CogCalibrationError as exc:
                return self._calculation_blocked_locked(
                    code=exc.code,
                    detail=exc.detail,
                    diagnostic_output_dir=diagnostic_output_dir,
                )
            force_rms = estimate.force_fit_rms_n
            torque_rms = estimate.torque_fit_rms_nm
            if force_rms > self._config.max_force_fit_rms_n:
                return self._calculation_blocked_locked(
                    code="force_fit_rms_exceeded",
                    detail=(
                        f"force fit RMS {force_rms:.4g} N exceeds "
                        f"{self._config.max_force_fit_rms_n:.4g} N; the configured "
                        f"{self._config.observation_model} gravity-wrench model does "
                        "not match this capture (verify EFT source processing and "
                        "T_tcp_sensor before changing fit bounds)"
                    ),
                    diagnostic_output_dir=diagnostic_output_dir,
                    candidate_estimate=estimate,
                )
            if torque_rms > self._config.max_torque_fit_rms_nm:
                return self._calculation_blocked_locked(
                    code="torque_fit_rms_exceeded",
                    detail=(
                        f"torque fit RMS {torque_rms:.4g} Nm exceeds "
                        f"{self._config.max_torque_fit_rms_nm:.4g} Nm; the configured "
                        f"{self._config.observation_model} gravity-wrench model does "
                        "not match this capture (verify EFT reference point and "
                        "T_tcp_sensor before changing fit bounds)"
                    ),
                    diagnostic_output_dir=diagnostic_output_dir,
                    candidate_estimate=estimate,
                )
            self._result = estimate
            self._state = "calculated"
            self._message = (
                "PROVISIONAL / NOT APPLIED — controller-compensated residual model complete"
                if self._config.observation_model == "controller_compensated_linear"
                else "PROVISIONAL / NOT APPLIED — physical payload calculation complete"
            )
            return True, self._message

    def _calculation_blocked_locked(
        self,
        *,
        code: str,
        detail: str,
        diagnostic_output_dir: str | None,
        candidate_estimate: GravityWrenchEstimate | None = None,
    ) -> tuple[bool, str]:
        self._result = None
        self._state = "calculation_blocked"
        self._message = f"calculation blocked [{code}]: {detail}"
        if self._saved_report is not None:
            self._message += f"; diagnostic evidence already saved: {self._saved_report}"
            return False, self._message
        if (
            diagnostic_output_dir is None
            or self._arm is None
            or self._run_id is None
        ):
            return False, self._message
        try:
            paths = save_blocked_calibration_report(
                output_dir=diagnostic_output_dir,
                run_id=self._run_id,
                arm=self._arm,
                poses=tuple(self._accepted),
                failure_code=code,
                failure_detail=detail,
                provenance=dict(self._provenance),
                candidate_estimate=candidate_estimate,
            )
        except (CogCalibrationError, OSError, TypeError, ValueError) as exc:
            self._message += f"; diagnostic report save failed: {exc}"
            return False, self._message
        self._saved_report = str(paths.report_json)
        self._message += f"; diagnostic evidence saved: {self._saved_report}"
        return False, self._message

    def save(self, output_dir: str) -> tuple[bool, str]:
        with self._lock:
            if self._result is None or self._arm is None or self._run_id is None:
                return False, "calculate a provisional result before saving"
            try:
                paths = save_calibration_report(
                    output_dir=output_dir,
                    run_id=self._run_id,
                    arm=self._arm,
                    poses=tuple(self._accepted),
                    estimate=self._result,
                    provenance=dict(self._provenance),
                )
            except (CogCalibrationError, OSError, ValueError) as exc:
                self._message = f"report save failed: {exc}"
                return False, self._message
            self._saved_report = str(paths.report_json)
            self._message = f"saved provisional report: {self._saved_report}"
            return True, self._message

    def release_if_pending(self) -> None:
        with self._lock:
            if not self._release_pending:
                return
            self._release_pending = False
        self._safety.end_payload_identification_session()

    def _finish_pose_locked(self, current: CogWaypoint) -> None:
        assert self._accumulator is not None
        assert self._config is not None
        pose = self._accumulator.freeze()
        summary = pose.summary()
        force_std = summary.max_force_stddev_n
        torque_std = summary.max_torque_stddev_nm
        if (
            force_std > self._config.max_force_stddev_n
            or torque_std > self._config.max_torque_stddev_nm
        ):
            self._state = "review"
            self._pose_states[current.name] = "rejected"
            self._message = (
                f"{current.name} noisy: max force std {force_std:.4g} N, "
                f"torque std {torque_std:.4g} Nm; Retry or Skip"
            )
            return
        self._accepted.append(pose)
        self._pose_states[current.name] = "accepted"
        self._advance_locked()

    def _advance_locked(self) -> None:
        self._current_index += 1
        self._accumulator = None
        self._settle_started = None
        self._settle_elapsed_sec = 0.0
        self._max_joint_error_deg = None
        if self._current_index >= len(self._waypoints):
            self._state = "complete"
            self._message = (
                f"capture complete: {len(self._accepted)} accepted pose(s); "
                "Hold/lease release pending; Calculate remains provisional"
            )
            self._release_pending = True
        else:
            current = self._waypoints[self._current_index]
            self._state = "armed"
            self._message = f"next waypoint {current.name}; hold Run/Continue"

    def _pause_locked(self, reason: str) -> None:
        current = self._current_waypoint_locked()
        if current is not None:
            self._pose_states[current.name] = "pending"
        self._state = "armed"
        self._accumulator = None
        self._settle_started = None
        self._settle_elapsed_sec = 0.0
        self._message = reason

    def _block_locked(self, reason: str) -> None:
        self._state = "blocked"
        self._message = reason
        self._release_pending = True

    def _current_waypoint_locked(self) -> CogWaypoint | None:
        if 0 <= self._current_index < len(self._waypoints):
            return self._waypoints[self._current_index]
        return None

    def _lease_owned_locked(self, latest: StateSnapshot | None) -> bool:
        if latest is None:
            return False
        owner = latest.command_source
        return bool(
            owner.active is True
            and owner.display_source_id == self._safety.command_client.source_id
            and owner.display_session_id == self._safety.command_client.session_id
        )

    def _health_reason_locked(
        self,
        latest: StateSnapshot | None,
        *,
        stale: bool,
        require_profile: bool,
    ) -> str | None:
        if latest is None or stale:
            return "state stream missing or stale"
        if latest.fault_latched or latest.motion_state in self._safety._latched_motion_states:
            return f"robot fault latched: {latest.fault_reason or latest.motion_state}"
        if latest.safety_verdict != "Ok":
            return f"safety verdict blocks payload identification: {latest.safety_verdict}"
        if not self._lease_owned_locked(latest):
            return "GUI command lease lost or owned by another source"
        if self._arm is None:
            return "payload-identification arm is unavailable"
        arm_state = latest.left if self._arm == "left" else latest.right
        if not arm_state.has_valid_joint_state or arm_state.error_code not in {None, 0}:
            return f"{self._arm} joint/backend state is invalid"
        ft = arm_state.force_torque
        if ft is None or ft.enabled is not True:
            return f"{self._arm} F/T pipeline is unavailable"
        if ft.healthy is not True or ft.stale is not False:
            return f"{self._arm} F/T sensor is unhealthy or stale"
        if (
            ft.raw_sensor_wrench is None
            or ft.t_tcp_sensor is None
            or ft.wrench_tcp is None
            or ft.gravity_tcp is None
            or ft.freshness_value is None
        ):
            return f"{self._arm} payload-identification telemetry is incomplete"
        if (
            arm_state.q_actual_deg is None
            or arm_state.tcp_actual_valid is not True
            or arm_state.tcp_actual_stand is None
        ):
            return f"{self._arm} actual pose telemetry is incomplete"
        force_control = arm_state.force_control
        if force_control is not None and force_control.hard_limit_exceeded is True:
            return f"{self._arm} force hard limit is active"
        if require_profile:
            if ft.payload_identification_inhibit is not True:
                return f"waiting for {self._arm} payload-identification force inhibit"
            if ft.joint_target_profile != "payload_identification":
                return f"waiting for {self._arm} payload-identification profile confirmation"
        return None
