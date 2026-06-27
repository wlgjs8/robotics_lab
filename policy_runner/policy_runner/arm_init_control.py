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
    if action != "toggle":
        raise ValueError("arm init command action must be toggle")
    return ArmInitCommand(
        arms=str(arms),
        action=str(action),
        left_q_deg=_optional_joint6(payload.get("left_q_deg"), "left_q_deg"),
        right_q_deg=_optional_joint6(payload.get("right_q_deg"), "right_q_deg"),
    )


class ArmInitOverrideController:
    """Runtime per-arm InitMotion latch owned by policy_runner.run()."""

    def __init__(self, *, timeout_sec: float = 0.2) -> None:
        self.timeout_sec = float(timeout_sec)
        self.left_on = False
        self.right_on = False
        self.left_q_deg: tuple[float, ...] | None = None
        self.right_q_deg: tuple[float, ...] | None = None
        self.last_command = ""
        self.error = ""
        self.changed = False

    def handle_command(self, command: ArmInitCommand) -> bool:
        self.changed = False
        self.error = ""
        self.last_command = command.arms
        if command.left_q_deg is not None:
            self.left_q_deg = command.left_q_deg
        if command.right_q_deg is not None:
            self.right_q_deg = command.right_q_deg

        if command.arms == "both":
            target_on = not (self.left_on or self.right_on)
            if target_on and (self.left_q_deg is None or self.right_q_deg is None):
                self.error = "missing_init_q_deg"
                return False
            self.changed = (self.left_on != target_on) or (self.right_on != target_on)
            self.left_on = target_on
            self.right_on = target_on
            return self.changed

        if command.arms == "left":
            target_on = not self.left_on
            if target_on and self.left_q_deg is None:
                self.error = "missing_left_init_q_deg"
                return False
            self.changed = self.left_on != target_on
            self.left_on = target_on
            return self.changed

        target_on = not self.right_on
        if target_on and self.right_q_deg is None:
            self.error = "missing_right_init_q_deg"
            return False
        self.changed = self.right_on != target_on
        self.right_on = target_on
        return self.changed

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

    def compose_intent(self, intent: CommandIntent | None) -> CommandIntent | None:
        if not self.left_on and not self.right_on:
            return intent
        left = _arm_payload(intent.left if intent is not None else None)
        right = _arm_payload(intent.right if intent is not None else None)
        timeout_sec = intent.timeout_sec if intent is not None else self.timeout_sec
        coupled_timeout = intent.coupled_timeout if intent is not None else True
        if self.left_on:
            if self.left_q_deg is None:
                self.error = "missing_left_init_q_deg"
                return intent
            left = _init_arm_payload(self.left_q_deg)
        if self.right_on:
            if self.right_q_deg is None:
                self.error = "missing_right_init_q_deg"
                return intent
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
            "left_state": "init_motion" if self.left_on else "policy",
            "right_state": "init_motion" if self.right_on else "policy",
            "last_command": self.last_command,
            "error": self.error,
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


def reset_source_after_override_change(source: object) -> None:
    for name in ("_clear_target_pose_state", "reset_engagement"):
        hook = getattr(source, name, None)
        if callable(hook):
            try:
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
