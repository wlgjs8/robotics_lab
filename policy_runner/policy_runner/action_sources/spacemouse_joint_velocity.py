from __future__ import annotations

from policy_runner.robot_state_client import StateSnapshot
from policy_runner.servo_command_client import CommandIntent
from policy_runner.safety import ActionRequirements
from policy_runner.spacemouse import SpaceMouseReader, SpaceMouseSample


class SpaceMouseJointVelocitySource:
    def __init__(
        self,
        reader: SpaceMouseReader,
        max_joint_velocity_deg_s: tuple[float, ...],
        selected_arm: str = "both",
        deadband: float = 0.08,
        smoothing_alpha: float = 0.2,
        require_deadman: bool = True,
        deadman_button: int = 0,
        timeout_sec: float = 0.2,
        simulation_only: bool = True,
    ):
        if len(max_joint_velocity_deg_s) != 6:
            raise ValueError("max_joint_velocity_deg_s must contain 6 values")
        if selected_arm not in {"left", "right", "both"}:
            raise ValueError("selected_arm must be left, right, or both")
        if not 0.0 <= smoothing_alpha <= 1.0:
            raise ValueError("smoothing_alpha must be in [0, 1]")
        self.reader = reader
        self.max_joint_velocity_deg_s = tuple(float(v) for v in max_joint_velocity_deg_s)
        self.selected_arm = selected_arm
        self.deadband = float(deadband)
        self.smoothing_alpha = float(smoothing_alpha)
        self.require_deadman = require_deadman
        self.deadman_button = deadman_button
        self.timeout_sec = timeout_sec
        self._last_velocity: list[float] | None = None
        self.requirements = ActionRequirements(simulation_only=simulation_only)

    def next_intent(self, snapshot: StateSnapshot, now_monotonic: float) -> CommandIntent | None:
        _ = snapshot, now_monotonic
        sample = self.reader.read()
        if sample is None:
            return None
        if self.require_deadman and not _button_pressed(sample, self.deadman_button):
            self._last_velocity = None
            return None
        velocity = self.map_sample(sample)
        return CommandIntent.joint_velocity(
            left=velocity if self.selected_arm in {"left", "both"} else None,
            right=velocity if self.selected_arm in {"right", "both"} else None,
            timeout_sec=self.timeout_sec,
        )

    def map_sample(self, sample: SpaceMouseSample) -> list[float]:
        axes = [sample.tx, sample.ty, sample.tz, sample.rx, sample.ry, sample.rz]
        raw = [
            _axis_to_velocity(axis, limit, self.deadband)
            for axis, limit in zip(axes, self.max_joint_velocity_deg_s)
        ]
        if self._last_velocity is None:
            smoothed = raw
        else:
            alpha = self.smoothing_alpha
            smoothed = [
                alpha * raw_value + (1.0 - alpha) * previous
                for raw_value, previous in zip(raw, self._last_velocity)
            ]
        self._last_velocity = smoothed
        return smoothed


def _button_pressed(sample: SpaceMouseSample, index: int) -> bool:
    return 0 <= index < len(sample.buttons) and bool(sample.buttons[index])


def _axis_to_velocity(axis: float, limit: float, deadband: float) -> float:
    axis = max(-1.0, min(1.0, float(axis)))
    if abs(axis) < deadband:
        axis = 0.0
    return axis * abs(float(limit))
