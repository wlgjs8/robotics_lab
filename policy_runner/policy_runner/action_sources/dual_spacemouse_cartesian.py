from __future__ import annotations

import warnings

from policy_runner.action_sources.tcp_delta import (
    CARTESIAN_ACTION_REQUIREMENTS,
    clamp_tcp_twist,
    tcp_twist_local_intent,
)
from policy_runner.robot_state_client import StateSnapshot
from policy_runner.servo_command_client import CommandIntent
from policy_runner.spacemouse import HidSpaceMouseReader, SpaceMouseReader, SpaceMouseSample


class DualSpaceMouseCartesianActionSource:
    requirements = CARTESIAN_ACTION_REQUIREMENTS

    def __init__(
        self,
        left_reader: SpaceMouseReader | None = None,
        right_reader: SpaceMouseReader | None = None,
        frame: str = "local",
        max_linear_velocity_m_s: float = 0.03,
        max_angular_velocity_rad_s: float = 0.2,
        deadband: float = 0.08,
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
        if frame != "local":
            raise ValueError("only local-frame TcpTwistLocal is enabled")
        if max_linear_velocity_m_s < 0.0:
            raise ValueError("max_linear_velocity_m_s must be non-negative")
        if max_angular_velocity_rad_s < 0.0:
            raise ValueError("max_angular_velocity_rad_s must be non-negative")
        if deadband < 0.0:
            raise ValueError("deadband must be non-negative")
        self.left_reader = left_reader if left_reader is not None else HidSpaceMouseReader(device_number=0)
        self.right_reader = right_reader if right_reader is not None else HidSpaceMouseReader(device_number=1)
        self.frame = frame
        self.max_linear_velocity_m_s = float(max_linear_velocity_m_s)
        self.max_angular_velocity_rad_s = float(max_angular_velocity_rad_s)
        self.deadband = float(deadband)
        self.timeout_sec = timeout_sec

    def next_intent(self, snapshot: StateSnapshot, now_monotonic: float) -> CommandIntent | None:
        _ = snapshot, now_monotonic
        left = self._twist_from_reader(self.left_reader)
        right = self._twist_from_reader(self.right_reader)
        if left is None and right is None:
            return None
        return tcp_twist_local_intent(left=left, right=right, timeout_sec=self.timeout_sec)

    def close(self) -> None:
        self.left_reader.close()
        self.right_reader.close()

    def _twist_from_reader(
        self,
        reader: SpaceMouseReader,
    ) -> tuple[float, ...] | None:
        sample = reader.read(timeout_sec=0.0)
        if sample is None:
            return None
        twist = _twist_from_sample(
            sample,
            max_linear_velocity_m_s=self.max_linear_velocity_m_s,
            max_angular_velocity_rad_s=self.max_angular_velocity_rad_s,
            deadband=self.deadband,
        )
        if all(value == 0.0 for value in twist):
            return None
        return twist


def _twist_from_sample(
    sample: SpaceMouseSample,
    *,
    max_linear_velocity_m_s: float,
    max_angular_velocity_rad_s: float,
    deadband: float,
) -> tuple[float, ...]:
    axes = (sample.tx, sample.ty, sample.tz, sample.rx, sample.ry, sample.rz)
    scaled = (
        _apply_deadband(axes[0], deadband) * max_linear_velocity_m_s,
        _apply_deadband(axes[1], deadband) * max_linear_velocity_m_s,
        _apply_deadband(axes[2], deadband) * max_linear_velocity_m_s,
        _apply_deadband(axes[3], deadband) * max_angular_velocity_rad_s,
        _apply_deadband(axes[4], deadband) * max_angular_velocity_rad_s,
        _apply_deadband(axes[5], deadband) * max_angular_velocity_rad_s,
    )
    return clamp_tcp_twist(
        scaled,
        max_linear_velocity_m_s,
        max_angular_velocity_rad_s,
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
            "max_linear_step_m is deprecated for dual SpaceMouse Cartesian twist; "
            "use max_linear_velocity_m_s",
            DeprecationWarning,
            stacklevel=3,
        )
        max_linear_velocity_m_s = max_linear_step_m
    if max_angular_step_rad is not None:
        warnings.warn(
            "max_angular_step_rad is deprecated for dual SpaceMouse Cartesian twist; "
            "use max_angular_velocity_rad_s",
            DeprecationWarning,
            stacklevel=3,
        )
        max_angular_velocity_rad_s = max_angular_step_rad
    return float(max_linear_velocity_m_s), float(max_angular_velocity_rad_s)


def _apply_deadband(value: float, deadband: float) -> float:
    value = float(value)
    if abs(value) <= deadband:
        return 0.0
    return value
