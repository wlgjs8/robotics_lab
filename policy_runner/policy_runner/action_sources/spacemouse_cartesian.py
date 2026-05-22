from __future__ import annotations

from policy_runner.action_sources.tcp_delta import (
    CARTESIAN_ACTION_REQUIREMENTS,
    clamp_tcp_delta,
    tcp_delta_stand_intent,
)
from policy_runner.robot_state_client import StateSnapshot
from policy_runner.servo_command_client import CommandIntent
from policy_runner.spacemouse import HidSpaceMouseReader, SpaceMouseReader, SpaceMouseSample


class SpaceMouseCartesianActionSource:
    requirements = CARTESIAN_ACTION_REQUIREMENTS

    def __init__(
        self,
        reader: SpaceMouseReader | None = None,
        selected_arm: str = "left",
        frame: str = "stand",
        max_linear_step_m: float = 0.002,
        max_angular_step_rad: float = 0.01,
        deadband: float = 0.08,
        require_deadman: bool = True,
        deadman_button: int = 0,
        timeout_sec: float = 0.2,
    ):
        if selected_arm not in {"left", "right", "both"}:
            raise ValueError("selected_arm must be left, right, or both")
        if frame != "stand":
            raise ValueError("only stand-frame TcpDeltaStand is enabled")
        if max_linear_step_m < 0.0:
            raise ValueError("max_linear_step_m must be non-negative")
        if max_angular_step_rad < 0.0:
            raise ValueError("max_angular_step_rad must be non-negative")
        if deadband < 0.0:
            raise ValueError("deadband must be non-negative")
        if deadman_button < 0:
            raise ValueError("deadman_button must be non-negative")
        self.reader = reader if reader is not None else HidSpaceMouseReader()
        self.selected_arm = selected_arm
        self.frame = frame
        self.max_linear_step_m = float(max_linear_step_m)
        self.max_angular_step_rad = float(max_angular_step_rad)
        self.deadband = float(deadband)
        self.require_deadman = bool(require_deadman)
        self.deadman_button = int(deadman_button)
        self.timeout_sec = timeout_sec

    def next_intent(self, snapshot: StateSnapshot, now_monotonic: float) -> CommandIntent | None:
        _ = snapshot, now_monotonic
        sample = self.reader.read(timeout_sec=0.0)
        if sample is None:
            return None
        if not self._deadman_active(sample):
            return None
        delta = self._delta_from_sample(sample)
        if all(value == 0.0 for value in delta):
            return None
        left = delta if self.selected_arm in {"left", "both"} else None
        right = delta if self.selected_arm in {"right", "both"} else None
        return tcp_delta_stand_intent(left=left, right=right, timeout_sec=self.timeout_sec)

    def close(self) -> None:
        self.reader.close()

    def _deadman_active(self, sample: SpaceMouseSample) -> bool:
        if not self.require_deadman:
            return True
        if self.deadman_button >= len(sample.buttons):
            return False
        return sample.buttons[self.deadman_button]

    def _delta_from_sample(self, sample: SpaceMouseSample) -> tuple[float, ...]:
        axes = (sample.tx, sample.ty, sample.tz, sample.rx, sample.ry, sample.rz)
        scaled = (
            _apply_deadband(axes[0], self.deadband) * self.max_linear_step_m,
            _apply_deadband(axes[1], self.deadband) * self.max_linear_step_m,
            _apply_deadband(axes[2], self.deadband) * self.max_linear_step_m,
            _apply_deadband(axes[3], self.deadband) * self.max_angular_step_rad,
            _apply_deadband(axes[4], self.deadband) * self.max_angular_step_rad,
            _apply_deadband(axes[5], self.deadband) * self.max_angular_step_rad,
        )
        return clamp_tcp_delta(scaled, self.max_linear_step_m, self.max_angular_step_rad)


def _apply_deadband(value: float, deadband: float) -> float:
    value = float(value)
    if abs(value) <= deadband:
        return 0.0
    return value
