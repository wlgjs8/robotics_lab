from __future__ import annotations

import json
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
            self._validate_real_policy(config, geometry_status=geometry_status)
            return
        raise RolloutModeValidationError(f"unhandled rollout-mode {self.mode.value}")

    def _validate_real_policy(self, config: Any, *, geometry_status: Any | None) -> None:
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
        if status not in {"measured", "calibrated", "accepted"}:
            raise RolloutModeValidationError(
                f"real_policy requires measured geometry; got geometry status={status or 'unknown'}"
            )
        if not bool(getattr(geometry_status, "geometry_valid_for_real_policy", False)):
            raise RolloutModeValidationError(
                "real_policy requires geometry_valid_for_real_policy=true"
            )
        if not bool(getattr(safety, "measured_retarget_available", False)):
            raise RolloutModeValidationError(
                "real_policy requires measured_retarget_available=true"
            )
        if not bool(getattr(safety, "measured_collision_model_available", False)):
            raise RolloutModeValidationError(
                "real_policy requires measured_collision_model_available=true"
            )
        if not bool(getattr(safety, "measured_gripper_available", False)):
            raise RolloutModeValidationError(
                "real_policy requires measured_gripper_available=true for flow-policy gripper channels"
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

    def record_source(self, source: Any) -> None:
        self.image_decode_count = int(getattr(source, "image_decode_count", self.image_decode_count) or 0)
        self.missing_camera_count = int(
            getattr(source, "missing_camera_count", self.missing_camera_count) or 0
        )
        source_cameras = getattr(source, "camera_names", None)
        if source_cameras is not None:
            self.camera_names = [str(name) for name in source_cameras]
        self.command_family = str(getattr(source, "command_family", self.command_family) or self.command_family)

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
            self.record_source(source)
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
    if mode and mode != "Hold":
        return mode
    for arm in ("left", "right"):
        arm_payload = getattr(intent, arm, None)
        if isinstance(arm_payload, dict):
            arm_mode = arm_payload.get("mode")
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
