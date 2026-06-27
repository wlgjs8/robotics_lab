from __future__ import annotations

import copy
import json
import math
from dataclasses import dataclass
from typing import Any, Iterable

from .servo_command_client import CommandIntent


ARM_INIT_COMMAND_SCHEMA = "robotics_lab.arm_init_cmd.v1"
ARM_INIT_STATE_SCHEMA = "robotics_lab.arm_init_state.v1"


@dataclass(frozen=True)
class ArmInitCommand:
    arms: str
    action: str = "toggle"
    left_q_deg: tuple[float, ...] | None = None
    right_q_deg: tuple[float, ...] | None = None


@dataclass(frozen=True)
class ArmInitTransitions:
    started: tuple[str, ...] = ()
    done: tuple[str, ...] = ()
    cancelled: tuple[str, ...] = ()
    failed: tuple[str, ...] = ()

    @property
    def any(self) -> bool:
        return bool(self.started or self.done or self.cancelled or self.failed)


@dataclass
class _ArmRuntime:
    on: bool = False
    q_deg: tuple[float, ...] | None = None
    state: str = "policy"
    fail: bool = False
    message: str = ""
    server_status: str = "idle"


def parse_arm_init_command(data: bytes | dict[str, Any]) -> ArmInitCommand:
    if isinstance(data, bytes):
        try:
            payload = json.loads(data.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValueError("arm init command must be JSON") from exc
    else:
        payload = data
    if not isinstance(payload, dict):
        raise ValueError("arm init command must be a JSON object")
    if payload.get("schema") != ARM_INIT_COMMAND_SCHEMA:
        raise ValueError("unsupported arm init command schema")
    arms = payload.get("arms")
    if arms not in {"both", "left", "right"}:
        raise ValueError("arm init command arms must be both, left, or right")
    action = payload.get("action", "toggle")
    if action not in {"start", "cancel", "toggle"}:
        raise ValueError("arm init command action must be start, cancel, or toggle")
    return ArmInitCommand(
        arms=str(arms),
        action=str(action),
        left_q_deg=_optional_joint6(payload.get("left_q_deg"), "left_q_deg"),
        right_q_deg=_optional_joint6(payload.get("right_q_deg"), "right_q_deg"),
    )


class ArmInitOverrideController:
    """Runtime per-arm InitMotion latch owned by policy_runner.run()."""

    def __init__(
        self,
        *,
        timeout_sec: float = 0.2,
        auto_clear_on_done: bool = True,
        auto_clear_on_failed: bool = False,
        resume_flow_on_done: bool = True,
        reset_flow_source_on_start: bool = True,
        reset_flow_source_on_resume: bool = True,
    ) -> None:
        self.timeout_sec = float(timeout_sec)
        self.auto_clear_on_done = bool(auto_clear_on_done)
        self.auto_clear_on_failed = bool(auto_clear_on_failed)
        self.resume_flow_on_done = bool(resume_flow_on_done)
        self.reset_flow_source_on_start = bool(reset_flow_source_on_start)
        self.reset_flow_source_on_resume = bool(reset_flow_source_on_resume)
        self._left = _ArmRuntime()
        self._right = _ArmRuntime()
        self.last_command = ""
        self.error = ""
        self.changed = False
        self._pending: dict[str, set[str]] = {
            "started": set(),
            "done": set(),
            "cancelled": set(),
            "failed": set(),
        }

    @property
    def left_on(self) -> bool:
        return self._left.on

    @property
    def right_on(self) -> bool:
        return self._right.on

    @property
    def left_q_deg(self) -> tuple[float, ...] | None:
        return self._left.q_deg

    @property
    def right_q_deg(self) -> tuple[float, ...] | None:
        return self._right.q_deg

    def handle_command(self, command: ArmInitCommand) -> bool:
        self.changed = False
        self.error = ""
        self.last_command = command.arms
        if command.left_q_deg is not None:
            self._left.q_deg = command.left_q_deg
        if command.right_q_deg is not None:
            self._right.q_deg = command.right_q_deg

        arms = _arms_for_selector(command.arms)
        action = command.action
        if action == "toggle":
            action = "cancel" if any(self._runtime(arm).on for arm in arms) else "start"
        if action == "cancel":
            return self._cancel_arms(arms)
        return self._start_arms(arms)

    def handle_payloads(self, payloads: Iterable[dict[str, Any]]) -> bool:
        changed = False
        for payload in payloads:
            if not isinstance(payload, dict) or payload.get("schema") != ARM_INIT_COMMAND_SCHEMA:
                continue
            try:
                changed = self.handle_command(parse_arm_init_command(payload)) or changed
            except ValueError as exc:
                self.error = str(exc)
        return changed

    def consume_transitions(self) -> ArmInitTransitions:
        transitions = ArmInitTransitions(
            started=tuple(sorted(self._pending["started"])),
            done=tuple(sorted(self._pending["done"])),
            cancelled=tuple(sorted(self._pending["cancelled"])),
            failed=tuple(sorted(self._pending["failed"])),
        )
        for values in self._pending.values():
            values.clear()
        return transitions

    def update_from_snapshot(self, snapshot: Any) -> bool:
        payload = getattr(snapshot, "payload", None)
        if not isinstance(payload, dict):
            return False
        block = payload.get("init_motion")
        if not isinstance(block, dict):
            return False
        changed = False
        for arm in ("left", "right"):
            runtime = self._runtime(arm)
            arm_block = block.get(arm)
            if not isinstance(arm_block, dict):
                arm_block = block
            status = _norm_status(arm_block.get("status"))
            message = str(arm_block.get("message", "") or "")
            if status:
                runtime.server_status = status
            if message:
                runtime.message = message
            if not runtime.on:
                continue
            if status in {"planning", "executing"}:
                next_state = f"init {status}"
                if runtime.state != next_state:
                    runtime.state = next_state
                    changed = True
                continue
            if status == "done":
                self._mark_done(arm)
                changed = True
                continue
            if status == "failed":
                self._mark_failed(arm, message or str(arm_block.get("fail_mode", "") or "init_failed"))
                changed = True
        self.changed = self.changed or changed
        return changed

    def compose_intent(self, intent: CommandIntent | None) -> CommandIntent | None:
        if not self.left_on and not self.right_on:
            return intent
        left = _arm_payload(intent.left if intent is not None else None)
        right = _arm_payload(intent.right if intent is not None else None)
        timeout_sec = intent.timeout_sec if intent is not None else self.timeout_sec
        coupled_timeout = intent.coupled_timeout if intent is not None else True
        if self.left_on:
            if self._left.fail:
                left = {"mode": "Hold"}
            elif self.left_q_deg is None:
                self.error = "missing_left_init_q_deg"
                return intent
            else:
                left = _init_arm_payload(self.left_q_deg)
        if self.right_on:
            if self._right.fail:
                right = {"mode": "Hold"}
            elif self.right_q_deg is None:
                self.error = "missing_right_init_q_deg"
                return intent
            else:
                right = _init_arm_payload(self.right_q_deg)
        return CommandIntent(
            _mixed_top_mode(left, right),
            timeout_sec=timeout_sec,
            left=left,
            right=right,
            coupled_timeout=coupled_timeout,
        )

    def stamp_snapshot(self, snapshot: Any) -> None:
        payload = getattr(snapshot, "payload", None)
        if isinstance(payload, dict):
            payload["arm_init"] = self.status_block()

    def status_block(self) -> dict[str, Any]:
        return {
            "schema": ARM_INIT_STATE_SCHEMA,
            "init_override_left": bool(self.left_on),
            "init_override_right": bool(self.right_on),
            "left_state": self._state_for("left"),
            "right_state": self._state_for("right"),
            "left_server_status": self._left.server_status,
            "right_server_status": self._right.server_status,
            "left_message": self._left.message,
            "right_message": self._right.message,
            "last_command": self.last_command,
            "error": self.error,
            "auto_clear_on_done": self.auto_clear_on_done,
            "auto_clear_on_failed": self.auto_clear_on_failed,
            "resume_flow_on_done": self.resume_flow_on_done,
        }

    def source_mask_for(self, base_mask: Any) -> Any:
        if base_mask is None:
            return None
        try:
            import numpy as np

            mask = np.asarray(base_mask, dtype=np.float32).copy()
        except Exception:
            return None
        if mask.shape[0] < 2:
            return None
        if self.left_on:
            mask[0] = 0.0
        if self.right_on:
            mask[1] = 0.0
        return mask

    def _runtime(self, arm: str) -> _ArmRuntime:
        return self._left if arm == "left" else self._right

    def _start_arms(self, arms: tuple[str, ...]) -> bool:
        missing = [
            arm
            for arm in arms
            if self._runtime(arm).q_deg is None
        ]
        if missing:
            self.error = "missing_init_q_deg" if len(missing) > 1 else f"missing_{missing[0]}_init_q_deg"
            return False
        changed = False
        for arm in arms:
            runtime = self._runtime(arm)
            if not runtime.on or runtime.fail:
                self._pending["started"].add(arm)
                changed = True
            runtime.on = True
            runtime.fail = False
            runtime.state = "init requested"
            runtime.message = ""
            runtime.server_status = "requested"
        self.changed = changed
        return changed

    def _cancel_arms(self, arms: tuple[str, ...]) -> bool:
        changed = False
        for arm in arms:
            runtime = self._runtime(arm)
            if runtime.on or runtime.state.startswith("init"):
                self._pending["cancelled"].add(arm)
                changed = True
            runtime.on = False
            runtime.fail = False
            runtime.state = "cancelled"
            runtime.server_status = "cancelled"
        self.changed = changed
        return changed

    def _mark_done(self, arm: str) -> None:
        runtime = self._runtime(arm)
        if runtime.state != "init done":
            self._pending["done"].add(arm)
        runtime.fail = False
        runtime.message = ""
        runtime.server_status = "done"
        if self.auto_clear_on_done and self.resume_flow_on_done:
            runtime.on = False
            runtime.state = "policy"
        else:
            runtime.state = "init done"

    def _mark_failed(self, arm: str, message: str) -> None:
        runtime = self._runtime(arm)
        if not runtime.fail:
            self._pending["failed"].add(arm)
        runtime.fail = True
        runtime.message = message
        runtime.server_status = "failed"
        if self.auto_clear_on_failed:
            runtime.on = False
            runtime.state = "policy"
        else:
            runtime.on = True
            runtime.state = "init failed"

    def _state_for(self, arm: str) -> str:
        runtime = self._runtime(arm)
        if runtime.on:
            return runtime.state if runtime.state.startswith("init") else "init requested"
        if runtime.state in {"cancelled", "init done"}:
            return runtime.state
        return "policy"


def source_arm_mask_copy(source: object) -> Any:
    if not hasattr(source, "arm_mask"):
        return None
    try:
        import numpy as np

        return np.asarray(getattr(source, "arm_mask"), dtype=np.float32).copy()
    except Exception:
        return None


def apply_source_arm_mask(source: object, base_mask: Any, controller: ArmInitOverrideController) -> None:
    mask = controller.source_mask_for(base_mask)
    if mask is None:
        return
    try:
        setattr(source, "arm_mask", mask)
    except Exception:
        return


def apply_source_override_transitions(
    source: object,
    transitions: ArmInitTransitions,
    snapshot: Any,
    *,
    reset_on_start: bool = True,
    reset_on_resume: bool = True,
) -> None:
    if not transitions.any:
        return
    for event, arms, reset_enabled in (
        ("start", transitions.started, reset_on_start),
        ("done", transitions.done, reset_on_resume),
        ("cancel", transitions.cancelled, reset_on_resume),
        ("failed", transitions.failed, False),
    ):
        if not arms:
            continue
        hook = getattr(source, f"on_arm_init_override_{event}", None)
        if callable(hook):
            try:
                hook(tuple(arms), snapshot)
                continue
            except Exception:
                pass
        if reset_enabled:
            reset_source_after_override_change(source, arms=arms)


def reset_source_after_override_change(source: object, *, arms: Iterable[str] | None = None) -> None:
    for name in ("_clear_target_pose_state", "reset_engagement"):
        hook = getattr(source, name, None)
        if callable(hook):
            try:
                if arms is None:
                    hook()
                else:
                    try:
                        hook(tuple(arms))
                    except TypeError:
                        hook()
            except Exception:
                pass


def intent_uses_source_requirements(intent: CommandIntent | None, controller: ArmInitOverrideController) -> bool:
    if intent is None:
        return False
    for arm_name, payload in (("left", intent.left), ("right", intent.right)):
        if arm_name == "left" and controller.left_on:
            continue
        if arm_name == "right" and controller.right_on:
            continue
        mode = payload.get("mode") if isinstance(payload, dict) else None
        if isinstance(mode, str) and mode not in {"", "Hold"}:
            return True
        if isinstance(payload, dict) and payload.get("gripper_target") is not None:
            return True
    return False


def _optional_joint6(value: Any, label: str) -> tuple[float, ...] | None:
    if value is None:
        return None
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        raise ValueError(f"{label} must contain 6 values")
    parsed = tuple(float(item) for item in value)
    if any(not math.isfinite(item) for item in parsed):
        raise ValueError(f"{label} values must be finite")
    return parsed


def _arms_for_selector(arms: str) -> tuple[str, ...]:
    if arms == "both":
        return ("left", "right")
    return (arms,)


def _norm_status(value: Any) -> str:
    text = str(value or "").strip().lower()
    if text in {"", "not_attempted"}:
        return ""
    if text in {"idle", "planning", "executing", "done", "failed"}:
        return text
    return text.replace("_", " ")


def _arm_payload(value: dict[str, Any] | None) -> dict[str, Any]:
    if not isinstance(value, dict):
        return {"mode": "Hold"}
    payload = copy.deepcopy(value)
    if not payload:
        payload["mode"] = "Hold"
    elif payload.get("mode") is None:
        payload["mode"] = "Hold"
    return payload


def _init_arm_payload(q_deg: tuple[float, ...]) -> dict[str, Any]:
    return {
        "mode": "JointTarget",
        "q_target_deg": [float(value) for value in q_deg],
        "joint_target_profile": "init_motion",
    }


def _mixed_top_mode(left: dict[str, Any], right: dict[str, Any]) -> str:
    left_mode = str(left.get("mode", "Hold"))
    right_mode = str(right.get("mode", "Hold"))
    if left_mode == right_mode:
        return left_mode
    return "Hold"
