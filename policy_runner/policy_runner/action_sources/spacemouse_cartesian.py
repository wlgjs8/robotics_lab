from __future__ import annotations

import math
import warnings

from policy_runner.action_sources.tcp_delta import (
    CARTESIAN_ACTION_REQUIREMENTS,
    clamp_tcp_twist,
    tcp_twist_local_intent,
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
        frame: str = "local",
        max_linear_velocity_m_s: float = 0.03,
        max_angular_velocity_rad_s: float = 0.2,
        deadband: float = 0.08,
        response_curve_gamma: float = 3.0,
        require_deadman: bool = True,
        deadman_button: int = 0,
        timeout_sec: float = 0.2,
        max_linear_step_m: float | None = None,
        max_angular_step_rad: float | None = None,
    ):
        max_linear_velocity_m_s, max_angular_velocity_rad_s = _resolve_velocity_limits(
            max_linear_velocity_m_s=max_linear_velocity_m_s,
            max_angular_velocity_rad_s=max_angular_velocity_rad_s,
            max_linear_step_m=max_linear_step_m,
            max_angular_step_rad=max_angular_step_rad,
        )
        if selected_arm not in {"left", "right", "both"}:
            raise ValueError("selected_arm must be left, right, or both")
        if frame != "local":
            raise ValueError("only local-frame TcpTwistLocal is enabled")
        if max_linear_velocity_m_s < 0.0:
            raise ValueError("max_linear_velocity_m_s must be non-negative")
        if max_angular_velocity_rad_s < 0.0:
            raise ValueError("max_angular_velocity_rad_s must be non-negative")
        if deadband < 0.0:
            raise ValueError("deadband must be non-negative")
        if response_curve_gamma < 1.0:
            raise ValueError("response_curve_gamma must be >= 1.0")
        if deadman_button < 0:
            raise ValueError("deadman_button must be non-negative")
        if not require_deadman:
            raise ValueError("spacemouse Cartesian control requires deadman")
        self.reader = reader if reader is not None else HidSpaceMouseReader()
        self.selected_arm = selected_arm
        self.frame = frame
        self.max_linear_velocity_m_s = float(max_linear_velocity_m_s)
        self.max_angular_velocity_rad_s = float(max_angular_velocity_rad_s)
        self.deadband = float(deadband)
        self.response_curve_gamma = float(response_curve_gamma)
        self.require_deadman = bool(require_deadman)
        self.deadman_button = int(deadman_button)
        self.timeout_sec = timeout_sec
        self._was_armed = False

    def next_intent(self, snapshot: StateSnapshot, now_monotonic: float) -> CommandIntent | None:
        _ = snapshot, now_monotonic
        sample = self.reader.read(timeout_sec=0.0)
        if sample is None:
            return None
        armed = self._deadman_active(sample)
        if not armed:
            if self._was_armed:
                self._was_armed = False
                return self._zero_twist_intent()
            return None
        self._was_armed = True
        twist = self._twist_from_sample(sample)
        if all(value == 0.0 for value in twist):
            return None
        left = twist if self.selected_arm in {"left", "both"} else None
        right = twist if self.selected_arm in {"right", "both"} else None
        return tcp_twist_local_intent(left=left, right=right, timeout_sec=self.timeout_sec)

    def close(self) -> None:
        self.reader.close()

    def _deadman_active(self, sample: SpaceMouseSample) -> bool:
        if not self.require_deadman:
            return True
        if self.deadman_button >= len(sample.buttons):
            return False
        return sample.buttons[self.deadman_button]

    def _zero_twist_intent(self) -> CommandIntent:
        zero = (0.0,) * 6
        left = zero if self.selected_arm in {"left", "both"} else None
        right = zero if self.selected_arm in {"right", "both"} else None
        return tcp_twist_local_intent(left=left, right=right, timeout_sec=self.timeout_sec)

    def _twist_from_sample(self, sample: SpaceMouseSample) -> tuple[float, ...]:
        axes = (sample.tx, sample.ty, sample.tz, sample.rx, sample.ry, sample.rz)
        scaled = (
            _apply_soft_deadband(axes[0], self.deadband, self.response_curve_gamma)
            * self.max_linear_velocity_m_s,
            _apply_soft_deadband(axes[1], self.deadband, self.response_curve_gamma)
            * self.max_linear_velocity_m_s,
            _apply_soft_deadband(axes[2], self.deadband, self.response_curve_gamma)
            * self.max_linear_velocity_m_s,
            _apply_soft_deadband(axes[3], self.deadband, self.response_curve_gamma)
            * self.max_angular_velocity_rad_s,
            _apply_soft_deadband(axes[4], self.deadband, self.response_curve_gamma)
            * self.max_angular_velocity_rad_s,
            _apply_soft_deadband(axes[5], self.deadband, self.response_curve_gamma)
            * self.max_angular_velocity_rad_s,
        )
        return clamp_tcp_twist(
            scaled,
            self.max_linear_velocity_m_s,
            self.max_angular_velocity_rad_s,
        )


def _resolve_velocity_limits(
    *,
    max_linear_velocity_m_s: float,
    max_angular_velocity_rad_s: float,
    max_linear_step_m: float | None,
    max_angular_step_rad: float | None,
) -> tuple[float, float]:
    if max_linear_step_m is not None:
        warnings.warn(
            "max_linear_step_m is deprecated for SpaceMouse Cartesian twist; "
            "use max_linear_velocity_m_s",
            DeprecationWarning,
            stacklevel=3,
        )
        max_linear_velocity_m_s = max_linear_step_m
    if max_angular_step_rad is not None:
        warnings.warn(
            "max_angular_step_rad is deprecated for SpaceMouse Cartesian twist; "
            "use max_angular_velocity_rad_s",
            DeprecationWarning,
            stacklevel=3,
        )
        max_angular_velocity_rad_s = max_angular_step_rad
    return float(max_linear_velocity_m_s), float(max_angular_velocity_rad_s)


def _apply_soft_deadband(value: float, deadband: float, gamma: float) -> float:
    value = float(value)
    if abs(value) <= deadband:
        return 0.0
    sign = math.copysign(1.0, value)
    magnitude = (abs(value) - deadband) / (1.0 - deadband)
    magnitude = max(0.0, min(1.0, magnitude))
    remapped = magnitude ** gamma
    return sign * remapped * (1.0 - deadband)
