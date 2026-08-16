from __future__ import annotations

import json
import os
import time
from collections import Counter
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any


ROLLOUT_SUMMARY_SCHEMA = "robotics_lab.policy_runner.rollout_summary.v1"


class RolloutMode(str, Enum):
    OFFLINE_EVAL = "offline_eval"
    SIM_DRYRUN = "sim_dryrun"
    CONTROLLER_SIM = "controller_sim"
    REAL_READONLY = "real_readonly"
    REAL_POLICY = "real_policy"

    @classmethod
    def choices(cls) -> tuple[str, ...]:
        return tuple(item.value for item in cls)


class RolloutModeValidationError(ValueError):
    pass


def parse_rollout_mode(value: str | RolloutMode) -> RolloutMode:
    if isinstance(value, RolloutMode):
        return value
    try:
        return RolloutMode(str(value))
    except ValueError as exc:
        choices = ", ".join(RolloutMode.choices())
        raise RolloutModeValidationError(
            f"invalid rollout-mode {value!r}; expected one of: {choices}"
        ) from exc


@dataclass(frozen=True)
class RolloutModePolicy:
    mode: RolloutMode
    send_dryrun_commands: bool = False

    @classmethod
    def from_value(
        cls,
        value: str | RolloutMode,
        *,
        send_dryrun_commands: bool = False,
    ) -> "RolloutModePolicy":
        return cls(parse_rollout_mode(value), bool(send_dryrun_commands))

    @property
    def may_send_commands(self) -> bool:
        if self.mode == RolloutMode.OFFLINE_EVAL:
            return False
        if self.mode == RolloutMode.SIM_DRYRUN:
            return self.send_dryrun_commands
        if self.mode == RolloutMode.REAL_READONLY:
            return False
        return self.mode in {RolloutMode.CONTROLLER_SIM, RolloutMode.REAL_POLICY}

    @property
    def allows_controller_simulation_cartesian(self) -> bool:
        return self.mode == RolloutMode.CONTROLLER_SIM

    @property
    def allows_physical_real_motion(self) -> bool:
        return self.mode == RolloutMode.REAL_POLICY

    @property
    def requires_live_state(self) -> bool:
        return self.mode != RolloutMode.OFFLINE_EVAL

    @property
    def cameras_required_if_checkpoint_expects(self) -> bool:
        return self.mode != RolloutMode.OFFLINE_EVAL

    @property
    def report_fields(self) -> dict[str, object]:
        return {
            "rollout_mode": self.mode.value,
            "may_send_commands": self.may_send_commands,
            "allows_controller_simulation_cartesian": self.allows_controller_simulation_cartesian,
            "allows_physical_real_motion": self.allows_physical_real_motion,
            "cameras_required_if_checkpoint_expects": self.cameras_required_if_checkpoint_expects,
        }

    def validate_config(
        self,
        config: Any,
        *,
        checkpoint_camera_names: list[str] | tuple[str, ...] = (),
        geometry_status: Any | None = None,
        checkpoint_arm_mask: list[float] | tuple[float, ...] | None = None,
        checkpoint_has_nonzero_gripper_commands: bool = False,
    ) -> None:
        mode = str(getattr(config, "mode", "") or "").lower()
        safety = getattr(config, "safety", None)
        if safety is None:
            raise RolloutModeValidationError("policy_runner config is missing safety settings")

        if checkpoint_camera_names and self.cameras_required_if_checkpoint_expects:
            camera = getattr(config, "camera", None)
            if camera is None or not bool(getattr(camera, "enable", False)):
                raise RolloutModeValidationError(
                    "checkpoint expects cameras, so live rollout requires camera.enable=true"
                )

        if self.mode == RolloutMode.OFFLINE_EVAL:
            return
        if self.mode == RolloutMode.SIM_DRYRUN:
            if mode == "real":
                raise RolloutModeValidationError(
                    "sim_dryrun must not use config mode=real; use controller_sim or real_readonly"
                )
            return
        if self.mode == RolloutMode.CONTROLLER_SIM:
            if mode != "real":
                raise RolloutModeValidationError(
                    "controller_sim requires config mode=real because it connects to rbpodo controllers"
                )
            if bool(getattr(safety, "allow_real_motion", False)):
                raise RolloutModeValidationError(
                    "controller_sim must keep safety.allow_real_motion=false; "
                    "use the controller_simulation carve-out instead of physical real approval"
                )
            if not bool(getattr(safety, "allow_rbpodo_controller_simulation_cartesian", False)):
                raise RolloutModeValidationError(
                    "controller_sim requires safety.allow_rbpodo_controller_simulation_cartesian=true"
                )
            return
        if self.mode == RolloutMode.REAL_READONLY:
            if mode != "real":
                raise RolloutModeValidationError("real_readonly requires config mode=real")
            return
        if self.mode == RolloutMode.REAL_POLICY:
            self._validate_real_policy(
                config,
                geometry_status=geometry_status,
                checkpoint_arm_mask=checkpoint_arm_mask,
                checkpoint_has_nonzero_gripper_commands=checkpoint_has_nonzero_gripper_commands,
            )
            return
        raise RolloutModeValidationError(f"unhandled rollout-mode {self.mode.value}")

    def _validate_real_policy(
        self,
        config: Any,
        *,
        geometry_status: Any | None,
        checkpoint_arm_mask: list[float] | tuple[float, ...] | None,
        checkpoint_has_nonzero_gripper_commands: bool,
    ) -> None:
        mode = str(getattr(config, "mode", "") or "").lower()
        safety = getattr(config, "safety", None)
        if mode != "real":
            raise RolloutModeValidationError("real_policy requires config mode=real")
        if not bool(getattr(safety, "allow_real_motion", False)):
            raise RolloutModeValidationError(
                "real_policy requires safety.allow_real_motion=true plus physical gates"
            )
        if bool(getattr(safety, "allow_rbpodo_controller_simulation_cartesian", False)):
            raise RolloutModeValidationError(
                "real_policy must not rely on controller_simulation Cartesian approval"
            )
        if geometry_status is None:
            raise RolloutModeValidationError("real_policy requires measured runtime geometry")
        status = str(getattr(geometry_status, "status", "") or "").strip().lower()
        allow_estimate_geometry = bool(
            getattr(safety, "allow_configured_estimate_geometry_in_real", False)
        )
        if status not in {"measured", "calibrated", "accepted"}:
            # Explicit operator carve-out: configured_estimate geometry is
            # acceptable when the config declares it for real (e.g. ee_local
            # checkpoints that do not consume the unmeasured tool/camera
            # extrinsics; the robot mounts in active_calibration.yaml are
            # mechanically measured, only the overall status stays estimate).
            if not (status == "configured_estimate" and allow_estimate_geometry):
                raise RolloutModeValidationError(
                    f"real_policy requires measured geometry; got geometry status={status or 'unknown'} "
                    "(configured_estimate is accepted only with "
                    "safety.allow_configured_estimate_geometry_in_real=true)"
                )
        if not bool(getattr(geometry_status, "geometry_valid_for_real_policy", False)):
            if not allow_estimate_geometry:
                raise RolloutModeValidationError(
                    "real_policy requires geometry_valid_for_real_policy=true"
                )
        retarget_status = str(getattr(safety, "retarget_status", "") or "missing")
        if retarget_status not in {"measured", "accepted"}:
            raise RolloutModeValidationError(
                f"real_policy requires retarget_status=measured or accepted; got {retarget_status}"
            )
        if not bool(getattr(safety, "measured_retarget_available", False)):
            raise RolloutModeValidationError(
                "real_policy requires measured_retarget_available=true"
            )
        collision_model_status = str(getattr(safety, "collision_model_status", "") or "missing")
        if collision_model_status in {"missing", "configured_estimate"}:
            raise RolloutModeValidationError(
                f"real_policy requires collision_model_status measured or validated; "
                f"got {collision_model_status}"
            )
        if collision_model_status not in {"measured", "validated"}:
            raise RolloutModeValidationError(
                f"real_policy requires collision_model_status measured or validated; "
                f"got {collision_model_status}"
            )
        if not bool(getattr(safety, "measured_collision_model_available", False)):
            raise RolloutModeValidationError(
                "real_policy requires measured_collision_model_available=true"
            )
        workspace_envelope_status = str(
            getattr(safety, "workspace_envelope_status", "") or "missing"
        )
        if workspace_envelope_status not in {"measured", "validated"}:
            raise RolloutModeValidationError(
                f"real_policy requires workspace_envelope_status measured or validated; "
                f"got {workspace_envelope_status}"
            )
        minimum_inter_arm_distance_m = getattr(safety, "minimum_inter_arm_distance_m", None)
        if minimum_inter_arm_distance_m is None:
            raise RolloutModeValidationError(
                "real_policy requires minimum_inter_arm_distance_m"
            )
        if checkpoint_arm_mask is not None:
            selected_arms = _selected_arms_from_safety(safety)
            checkpoint_arms = _arms_from_checkpoint_mask(checkpoint_arm_mask)
            if selected_arms != checkpoint_arms:
                raise RolloutModeValidationError(
                    "real_policy selected_arm/selected_arms must match checkpoint arm_mask; "
                    f"selected_arms={list(selected_arms)} checkpoint_arm_mask={list(checkpoint_arm_mask)}"
                )
        if not bool(getattr(safety, "measured_gripper_available", False)):
            raise RolloutModeValidationError(
                "real_policy requires measured_gripper_available=true for flow-policy gripper channels"
            )
        if checkpoint_has_nonzero_gripper_commands:
            if not bool(getattr(safety, "allow_real_gripper_motion", False)):
                raise RolloutModeValidationError(
                    "real_policy checkpoint has nonzero gripper commands; "
                    "requires allow_real_gripper_motion=true"
                )
            if os.environ.get("RB_ALLOW_REAL_GRIPPER") != "1":
                raise RolloutModeValidationError(
                    "real_policy checkpoint has nonzero gripper commands; "
                    "requires RB_ALLOW_REAL_GRIPPER=1"
                )


@dataclass
class RolloutSummaryRecorder:
    policy: RolloutModePolicy
    checkpoint_path: str
    config_path: str
    command_family: str = "unknown"
    camera_names: list[str] = field(default_factory=list)
    safety_decisions: Counter[str] = field(default_factory=Counter)
    dropped_reasons: Counter[str] = field(default_factory=Counter)
    sent_command_count: int = 0
    dropped_command_count: int = 0
    proposed_intent_count: int = 0
    image_decode_count: int = 0
    missing_camera_count: int = 0
    state_seen_count: int = 0
    backend_seen: str | None = None
    run_mode_seen: str | None = None
    operation_mode_seen: str | None = None
    physical_motion_expected: bool | None = None
    selected_arm: str | None = None
    selected_arms: list[str] = field(default_factory=list)
    left_arm_mask: float | None = None
    right_arm_mask: float | None = None
    gripper_command_count: int = 0
    gripper_dropped_count: int = 0
    allow_real_gripper_motion: bool = False
    collision_model_status: str = "missing"
    force_recovery: dict[str, Any] = field(default_factory=dict)
    camera_runtime: dict[str, Any] = field(default_factory=dict)
    inference_diagnostics: dict[str, Any] = field(default_factory=dict)
    # These three are SUMMARY-ONLY fields (they are read once, by to_dict). Rebuilding them
    # every control tick cost ~5.7 ms/tick: inference_diagnostics_snapshot() deep-copies a
    # rolling window of 64 inference events AND takes the lock shared with the inference
    # thread. The window fills at one event per chunk (~235 ms), i.e. ~15 s -- exactly the
    # observed ramp from 0.7 ms to 5.7 ms per tick followed by a plateau. That inflated the
    # loop from 3.2 ms to 8.8 ms, which quantizes the policy step deadline and pinned the
    # achieved rate near 21 Hz. Refresh on a wall-clock interval instead; to_dict() forces a
    # final refresh so the written summary is still current.
    diagnostics_refresh_sec: float = 0.5
    _diag_next_refresh: float = 0.0

    def record_state(self, snapshot: Any) -> None:
        payload = getattr(snapshot, "payload", snapshot)
        if not isinstance(payload, dict):
            return
        self.state_seen_count += 1
        self.backend_seen = _first_str(
            payload,
            ("observed_backend", "backend_type", "backend"),
            self.backend_seen,
        )
        self.run_mode_seen = _first_str(
            payload,
            ("observed_mode", "run_mode", "mode"),
            self.run_mode_seen,
        )
        self.operation_mode_seen = _first_nested_str(
            payload,
            ("operation_mode", "controller_operation_mode"),
            self.operation_mode_seen,
        )
        physical_motion_expected = _first_nested_bool(
            payload,
            ("physical_motion_expected",),
            self.physical_motion_expected,
        )
        self.physical_motion_expected = physical_motion_expected

    def record_source(self, source: Any, *, force: bool = False) -> None:
        self.image_decode_count = int(getattr(source, "image_decode_count", self.image_decode_count) or 0)
        self.missing_camera_count = int(
            getattr(source, "missing_camera_count", self.missing_camera_count) or 0
        )
        source_cameras = getattr(source, "camera_names", None)
        if source_cameras is not None:
            self.camera_names = [str(name) for name in source_cameras]
        self.command_family = str(getattr(source, "command_family", self.command_family) or self.command_family)
        source_selected_arms = getattr(source, "checkpoint_selected_arms", None)
        if source_selected_arms is not None:
            self.selected_arms = [str(arm) for arm in source_selected_arms]
            self.selected_arm = _selected_arm_label(self.selected_arms)
        source_arm_mask = getattr(source, "checkpoint_arm_mask", None)
        if source_arm_mask is not None:
            mask = list(source_arm_mask)
            if len(mask) >= 1:
                self.left_arm_mask = float(mask[0])
            if len(mask) >= 2:
                self.right_arm_mask = float(mask[1])
        self.gripper_command_count = int(
            getattr(source, "gripper_command_count", self.gripper_command_count) or 0
        )
        self.gripper_dropped_count = int(
            getattr(source, "gripper_dropped_count", self.gripper_dropped_count) or 0
        )
        now = time.monotonic()
        if not force and now < self._diag_next_refresh:
            return
        self._diag_next_refresh = now + max(0.0, float(self.diagnostics_refresh_sec))
        force_status = getattr(source, "force_recovery_status", None)
        if callable(force_status):
            self.force_recovery = dict(force_status())
        camera_status = getattr(source, "camera_runtime_status", None)
        if callable(camera_status):
            self.camera_runtime = dict(camera_status())
        diagnostics = getattr(source, "inference_diagnostics_snapshot", None)
        if callable(diagnostics):
            try:
                self.inference_diagnostics = dict(diagnostics())
            except Exception:  # noqa: BLE001 - summary diagnostics are best effort.
                pass

    def record_decision(self, decision: Any) -> None:
        reason = str(getattr(decision, "reason", "") or "ok")
        self.safety_decisions[reason] += 1

    def record_sent(self, intent: Any) -> None:
        self.sent_command_count += 1
        family = command_family_from_intent(intent)
        if family != "unknown":
            self.command_family = family

    def record_dropped(self, reason: str, intent: Any | None = None) -> None:
        self.dropped_command_count += 1
        self.dropped_reasons[str(reason)] += 1
        family = command_family_from_intent(intent)
        if family != "unknown":
            self.command_family = family

    def record_proposed_intent(self, intent: Any | None) -> None:
        if intent is None:
            return
        self.proposed_intent_count += 1
        family = command_family_from_intent(intent)
        if family != "unknown":
            self.command_family = family

    def to_dict(self, source: Any | None = None) -> dict[str, Any]:
        if source is not None:
            self.record_source(source, force=True)
        selected_arm = self.selected_arm or _selected_arm_label(self.selected_arms)
        data = {
            **self.policy.report_fields,
            "checkpoint_path": self.checkpoint_path,
            "config_path": self.config_path,
            "command_family": self.command_family,
            "camera_names": list(self.camera_names),
            "missing_camera_count": int(self.missing_camera_count),
            "image_decode_count": int(self.image_decode_count),
            "safety_decision_counts": dict(sorted(self.safety_decisions.items())),
            "sent_command_count": int(self.sent_command_count),
            "dropped_command_count": int(self.dropped_command_count),
            "dropped_reason_counts": dict(sorted(self.dropped_reasons.items())),
            "proposed_intent_count": int(self.proposed_intent_count),
            "physical_motion_expected": self.physical_motion_expected,
            "state_seen_count": int(self.state_seen_count),
            "backend_seen": self.backend_seen,
            "run_mode_seen": self.run_mode_seen,
            "operation_mode_seen": self.operation_mode_seen,
            "selected_arm": selected_arm,
            "selected_arms": list(self.selected_arms),
            "left_arm_mask": self.left_arm_mask,
            "right_arm_mask": self.right_arm_mask,
            "arm_mask": {
                "left": self.left_arm_mask,
                "right": self.right_arm_mask,
            },
            "gripper_command_count": int(self.gripper_command_count),
            "gripper_dropped_count": int(self.gripper_dropped_count),
            "allow_real_gripper_motion": bool(self.allow_real_gripper_motion),
            "collision_model_status": self.collision_model_status,
            "force_recovery": dict(self.force_recovery),
            "camera_runtime": dict(self.camera_runtime),
            "inference_diagnostics": dict(self.inference_diagnostics),
        }
        return data

    def to_document(self, source: Any | None = None) -> dict[str, Any]:
        return {
            "schema": ROLLOUT_SUMMARY_SCHEMA,
            "rollout_summary": self.to_dict(source),
        }


class ReadOnlyActionSource:
    """Pass-through source that records proposed intents before run() drops them."""

    def __init__(self, source: Any, recorder: RolloutSummaryRecorder):
        self._source = source
        self._recorder = recorder

    @property
    def requirements(self) -> Any:
        return getattr(self._source, "requirements", None)

    @property
    def camera_names(self) -> Any:
        return getattr(self._source, "camera_names", [])

    @property
    def command_family(self) -> str:
        return str(getattr(self._source, "command_family", "unknown") or "unknown")

    @property
    def image_decode_count(self) -> int:
        return int(getattr(self._source, "image_decode_count", 0) or 0)

    @property
    def missing_camera_count(self) -> int:
        return int(getattr(self._source, "missing_camera_count", 0) or 0)

    @property
    def checkpoint_selected_arms(self) -> Any:
        return getattr(self._source, "checkpoint_selected_arms", [])

    @property
    def checkpoint_arm_mask(self) -> Any:
        return getattr(self._source, "checkpoint_arm_mask", None)

    @property
    def gripper_command_count(self) -> int:
        return int(getattr(self._source, "gripper_command_count", 0) or 0)

    @property
    def gripper_dropped_count(self) -> int:
        return int(getattr(self._source, "gripper_dropped_count", 0) or 0)

    @property
    def force_recovery_terminal_abort_reason(self) -> str | None:
        return getattr(self._source, "force_recovery_terminal_abort_reason", None)

    @property
    def terminal_abort_reason(self) -> str | None:
        value = getattr(self._source, "terminal_abort_reason", None)
        if value is not None:
            return str(value)
        return self.force_recovery_terminal_abort_reason

    def force_recovery_status(self) -> dict[str, Any]:
        status = getattr(self._source, "force_recovery_status", None)
        return dict(status()) if callable(status) else {}

    def camera_runtime_status(self) -> dict[str, Any]:
        status = getattr(self._source, "camera_runtime_status", None)
        return dict(status()) if callable(status) else {}

    def inference_diagnostics_snapshot(self) -> dict[str, Any]:
        snapshot = getattr(self._source, "inference_diagnostics_snapshot", None)
        return dict(snapshot()) if callable(snapshot) else {}

    def next_intent(self, snapshot: Any, now_monotonic: float) -> Any | None:
        intent = self._source.next_intent(snapshot, now_monotonic)
        self._recorder.record_proposed_intent(intent)
        self._recorder.record_source(self._source)
        return intent

    def close(self) -> None:
        close = getattr(self._source, "close", None)
        if callable(close):
            close()


def command_family_from_intent(intent: Any | None) -> str:
    if intent is None:
        return "unknown"
    mode = str(getattr(intent, "mode", "") or "")
    if mode == "TcpPoseTarget":
        return "tcp_target_pose"
    if mode and mode != "Hold":
        return mode
    for arm in ("left", "right"):
        arm_payload = getattr(intent, arm, None)
        if isinstance(arm_payload, dict):
            arm_mode = arm_payload.get("mode")
            if arm_mode == "TcpPoseTarget":
                return "tcp_target_pose"
            if isinstance(arm_mode, str) and arm_mode and arm_mode != "Hold":
                return arm_mode
    return mode or "unknown"


def write_rollout_summary(
    recorder: RolloutSummaryRecorder,
    path: str | Path,
    *,
    source: Any | None = None,
) -> None:
    summary_path = Path(path)
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(
        json.dumps(recorder.to_document(source), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _first_str(payload: dict[str, Any], keys: tuple[str, ...], current: str | None) -> str | None:
    for key in keys:
        value = payload.get(key)
        if isinstance(value, str) and value:
            return value
    return current


def _first_nested_str(
    payload: dict[str, Any],
    keys: tuple[str, ...],
    current: str | None,
) -> str | None:
    top = _first_str(payload, keys, None)
    if top is not None:
        return top
    for arm in ("left", "right"):
        arm_payload = payload.get(arm)
        if isinstance(arm_payload, dict):
            arm_value = _first_str(arm_payload, keys, None)
            if arm_value is not None:
                return arm_value
            gate = arm_payload.get("cartesian_gate")
            if isinstance(gate, dict):
                gate_value = _first_str(gate, keys, None)
                if gate_value is not None:
                    return gate_value
    gate = payload.get("cartesian_gate")
    if isinstance(gate, dict):
        gate_value = _first_str(gate, keys, None)
        if gate_value is not None:
            return gate_value
    return current


def _first_nested_bool(
    payload: dict[str, Any],
    keys: tuple[str, ...],
    current: bool | None,
) -> bool | None:
    for key in keys:
        value = payload.get(key)
        if isinstance(value, bool):
            return value
    for arm in ("left", "right"):
        arm_payload = payload.get(arm)
        if isinstance(arm_payload, dict):
            for key in keys:
                value = arm_payload.get(key)
                if isinstance(value, bool):
                    return value
            gate = arm_payload.get("cartesian_gate")
            if isinstance(gate, dict):
                for key in keys:
                    value = gate.get(key)
                    if isinstance(value, bool):
                        return value
    gate = payload.get("cartesian_gate")
    if isinstance(gate, dict):
        for key in keys:
            value = gate.get(key)
            if isinstance(value, bool):
                return value
    return current


def _selected_arms_from_safety(safety: Any) -> tuple[str, ...]:
    selected_arms = getattr(safety, "selected_arms", ())
    if selected_arms:
        return tuple(arm for arm in ("left", "right") if arm in set(selected_arms))
    selected_arm = str(getattr(safety, "selected_arm", "") or "both")
    if selected_arm == "left":
        return ("left",)
    if selected_arm == "right":
        return ("right",)
    return ("left", "right")


def _arms_from_checkpoint_mask(mask: list[float] | tuple[float, ...]) -> tuple[str, ...]:
    arms: list[str] = []
    values = list(mask)
    if len(values) >= 1 and float(values[0]) > 0.0:
        arms.append("left")
    if len(values) >= 2 and float(values[1]) > 0.0:
        arms.append("right")
    return tuple(arms) or ("left", "right")


def _selected_arm_label(selected_arms: list[str]) -> str | None:
    arms = [arm for arm in ("left", "right") if arm in set(selected_arms)]
    if arms == ["left"]:
        return "left"
    if arms == ["right"]:
        return "right"
    if arms == ["left", "right"]:
        return "both"
    return None
