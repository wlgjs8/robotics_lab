from __future__ import annotations

from policy_runner.robot_state_client import StateSnapshot
from policy_runner.servo_command_client import CommandIntent
from policy_runner.safety import ActionRequirements
from policy_runner.spacemouse import HidSpaceMouseReader, SpaceMouseReader, SpaceMouseSample


class SpaceMouseJointVelocityActionSource:
    requirements = ActionRequirements()

    def __init__(
        self,
        reader: SpaceMouseReader | None = None,
        selected_arm: str = "left",
        max_joint_velocity_deg_s: tuple[float, ...] = (5.0, 5.0, 5.0, 8.0, 8.0, 10.0),
        deadband: float = 0.08,
        smoothing_alpha: float = 0.2,
        require_deadman: bool = True,
        deadman_button: int = 0,
        timeout_sec: float = 0.2,
    ):
        if selected_arm not in {"left", "right", "both"}:
            raise ValueError("selected_arm must be left, right, or both")
        if len(max_joint_velocity_deg_s) != 6:
            raise ValueError("max_joint_velocity_deg_s must contain 6 values")
        if deadband < 0.0:
            raise ValueError("deadband must be non-negative")
        if not 0.0 <= smoothing_alpha <= 1.0:
            raise ValueError("smoothing_alpha must be between 0 and 1")
        if deadman_button < 0:
            raise ValueError("deadman_button must be non-negative")
        self.reader = reader if reader is not None else HidSpaceMouseReader()
        self.selected_arm = selected_arm
        self.max_joint_velocity_deg_s = tuple(float(v) for v in max_joint_velocity_deg_s)
        self.deadband = float(deadband)
        self.smoothing_alpha = float(smoothing_alpha)
        self.require_deadman = bool(require_deadman)
        self.deadman_button = int(deadman_button)
        self.timeout_sec = timeout_sec
        self._previous_velocity: tuple[float, ...] | None = None

    def next_intent(self, snapshot: StateSnapshot, now_monotonic: float) -> CommandIntent | None:
        _ = snapshot, now_monotonic
        sample = self.reader.read(timeout_sec=0.0)
        if sample is None:
            return None
        if not self._deadman_active(sample):
            self._previous_velocity = None
            return None
        velocity = self._velocity_from_sample(sample)
        left = list(velocity) if self.selected_arm in {"left", "both"} else None
        right = list(velocity) if self.selected_arm in {"right", "both"} else None
        return CommandIntent.joint_velocity(left=left, right=right, timeout_sec=self.timeout_sec)

    def close(self) -> None:
        self.reader.close()

    def _deadman_active(self, sample: SpaceMouseSample) -> bool:
        if not self.require_deadman:
            return True
        if self.deadman_button >= len(sample.buttons):
            return False
        return sample.buttons[self.deadman_button]

    def _velocity_from_sample(self, sample: SpaceMouseSample) -> tuple[float, ...]:
        axes = (sample.tx, sample.ty, sample.tz, sample.rx, sample.ry, sample.rz)
        target = tuple(
            _clamp(_apply_deadband(axis, self.deadband) * max_velocity, max_velocity)
            for axis, max_velocity in zip(axes, self.max_joint_velocity_deg_s)
        )
        if self._previous_velocity is None:
            self._previous_velocity = target
            return target
        smoothed = tuple(
            previous + self.smoothing_alpha * (current - previous)
            for previous, current in zip(self._previous_velocity, target)
        )
        clamped = tuple(
            _clamp(value, max_velocity)
            for value, max_velocity in zip(smoothed, self.max_joint_velocity_deg_s)
        )
        self._previous_velocity = clamped
        return clamped


def _apply_deadband(value: float, deadband: float) -> float:
    value = float(value)
    if abs(value) <= deadband:
        return 0.0
    return value


def _clamp(value: float, max_abs: float) -> float:
    limit = abs(float(max_abs))
    return max(-limit, min(limit, float(value)))
