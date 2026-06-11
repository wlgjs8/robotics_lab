from __future__ import annotations

import math
import os
import time
import warnings

from policy_runner.action_sources.tcp_delta import (
    CARTESIAN_ACTION_REQUIREMENTS,
    cartesian_action_requirements,
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
        response_curve_gamma: float = 3.0,
        linear_axis_signs: tuple[float, ...] = (1.0, 1.0, 1.0),
        angular_axis_signs: tuple[float, ...] = (1.0, 1.0, 1.0),
        angular_axis_order: tuple[str, ...] = ("rx", "ry", "rz"),
        sample_hold_timeout_sec: float = 0.05,
        left_deadman_button: int = 0,
        right_deadman_button: int = 0,
        timeout_sec: float = 0.2,
        allow_rbpodo_controller_simulation: bool = False,
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
        if response_curve_gamma < 1.0:
            raise ValueError("response_curve_gamma must be >= 1.0")
        if sample_hold_timeout_sec <= 0.0:
            raise ValueError("sample_hold_timeout_sec must be positive")
        if left_deadman_button < 0 or right_deadman_button < 0:
            raise ValueError("deadman buttons must be non-negative")
        self.left_reader = left_reader if left_reader is not None else HidSpaceMouseReader(device_number=0)
        self.right_reader = right_reader if right_reader is not None else HidSpaceMouseReader(device_number=1)
        self.frame = frame
        self.max_linear_velocity_m_s = float(max_linear_velocity_m_s)
        self.max_angular_velocity_rad_s = float(max_angular_velocity_rad_s)
        self.deadband = float(deadband)
        self.response_curve_gamma = float(response_curve_gamma)
        self.linear_axis_signs = _axis_signs(linear_axis_signs, "linear_axis_signs")
        self.angular_axis_signs = _axis_signs(angular_axis_signs, "angular_axis_signs")
        self.angular_axis_order = _angular_axis_order(angular_axis_order)
        self.sample_hold_timeout_sec = float(sample_hold_timeout_sec)
        self.left_deadman_button = int(left_deadman_button)
        self.right_deadman_button = int(right_deadman_button)
        self.timeout_sec = timeout_sec
        self.requirements = cartesian_action_requirements(
            allow_rbpodo_controller_simulation=allow_rbpodo_controller_simulation
        )
        self._left_was_armed = False
        self._right_was_armed = False
        self._left_last_twist: tuple[float, ...] | None = None
        self._right_last_twist: tuple[float, ...] | None = None
        self._left_last_sample_monotonic: float | None = None
        self._right_last_sample_monotonic: float | None = None
        # POLICY_RUNNER_TELEOP_DEBUG=1: print raw samples/armed/twist at 10 Hz.
        self._debug = os.environ.get("POLICY_RUNNER_TELEOP_DEBUG", "") == "1"
        self._debug_raw: dict[str, SpaceMouseSample | None] = {"left": None, "right": None}
        self._debug_last_print = 0.0

    def next_intent(self, snapshot: StateSnapshot, now_monotonic: float) -> CommandIntent | None:
        _ = snapshot
        (
            left,
            self._left_was_armed,
            left_released,
            left_armed,
            self._left_last_twist,
            self._left_last_sample_monotonic,
        ) = self._twist_from_reader(
            self.left_reader,
            self.left_deadman_button,
            self._left_was_armed,
            self._left_last_twist,
            self._left_last_sample_monotonic,
            now_monotonic,
            side="left",
        )
        (
            right,
            self._right_was_armed,
            right_released,
            right_armed,
            self._right_last_twist,
            self._right_last_sample_monotonic,
        ) = self._twist_from_reader(
            self.right_reader,
            self.right_deadman_button,
            self._right_was_armed,
            self._right_last_twist,
            self._right_last_sample_monotonic,
            now_monotonic,
            side="right",
        )
        if left_armed is False and right_armed is False and (left_released or right_released):
            left = _ZERO_TWIST
            right = _ZERO_TWIST
        if self._debug:
            self._debug_print(left, right, left_armed, right_armed)
        if left is None and right is None and left_armed is not True and right_armed is not True:
            return None
        return tcp_twist_local_intent(left=left, right=right, timeout_sec=self.timeout_sec)

    def _debug_print(
        self,
        left_twist: tuple[float, ...] | None,
        right_twist: tuple[float, ...] | None,
        left_armed: bool | None,
        right_armed: bool | None,
    ) -> None:
        now = time.monotonic()
        if now - self._debug_last_print < 0.1:
            return
        self._debug_last_print = now
        parts = []
        for label, side, twist, armed in (
            ("LEFT ", "left", left_twist, left_armed),
            ("RIGHT", "right", right_twist, right_armed),
        ):
            sample = self._debug_raw[side]
            if sample is None:
                parts.append(f"{label} no-sample")
                continue
            axes = ",".join(
                f"{v:+.2f}"
                for v in (sample.tx, sample.ty, sample.tz, sample.rx, sample.ry, sample.rz)
            )
            pressed = [i for i, b in enumerate(sample.buttons) if b]
            twist_str = (
                "none"
                if twist is None
                else ",".join(f"{v:+.3f}" for v in twist)
            )
            parts.append(f"{label} axes=({axes}) btns={pressed} armed={armed} twist=({twist_str})")
        print(f"[SM] {' | '.join(parts)}", flush=True)

    def close(self) -> None:
        self.left_reader.close()
        self.right_reader.close()

    def _twist_from_reader(
        self,
        reader: SpaceMouseReader,
        deadman_button: int,
        was_armed: bool,
        last_twist: tuple[float, ...] | None,
        last_sample_monotonic: float | None,
        now_monotonic: float,
        *,
        side: str = "left",
    ) -> tuple[tuple[float, ...] | None, bool, bool, bool | None, tuple[float, ...] | None, float | None]:
        sample = reader.read(timeout_sec=0.0)
        if self._debug and sample is not None:
            self._debug_raw[side] = sample
        if sample is None:
            if last_twist is None or last_sample_monotonic is None:
                return None, was_armed, False, None, last_twist, last_sample_monotonic
            if now_monotonic - last_sample_monotonic <= self.sample_hold_timeout_sec:
                return last_twist, True, False, True, last_twist, last_sample_monotonic
            return _ZERO_TWIST, False, True, False, None, None
        armed = _deadman_active(sample, deadman_button)
        if not armed:
            if was_armed:
                return _ZERO_TWIST, False, True, False, None, None
            return None, False, False, False, None, None
        twist = _twist_from_sample(
            sample,
            max_linear_velocity_m_s=self.max_linear_velocity_m_s,
            max_angular_velocity_rad_s=self.max_angular_velocity_rad_s,
            deadband=self.deadband,
            response_curve_gamma=self.response_curve_gamma,
            linear_axis_signs=self.linear_axis_signs,
            angular_axis_signs=self.angular_axis_signs,
            angular_axis_order=self.angular_axis_order,
        )
        if all(value == 0.0 for value in twist):
            return None, True, False, True, _ZERO_TWIST, sample.timestamp_monotonic
        return twist, True, False, True, twist, sample.timestamp_monotonic


_ZERO_TWIST = (0.0,) * 6


def _deadman_active(sample: SpaceMouseSample, deadman_button: int) -> bool:
    if deadman_button >= len(sample.buttons):
        return False
    return sample.buttons[deadman_button]


_ANGULAR_AXIS_NAMES = ("rx", "ry", "rz")


def _axis_signs(value, label: str) -> tuple[float, float, float]:
    signs = tuple(float(v) for v in value)
    if len(signs) != 3 or any(sign not in (-1.0, 1.0) for sign in signs):
        raise ValueError(f"{label} must be 3 entries of -1 or 1")
    return signs


def _angular_axis_order(value) -> tuple[int, int, int]:
    """Normalize ('ry','rx','rz')-style names (or 0..2 indices) to indices.

    Output angular axis i reads the named/indexed INPUT axis, so
    ('ry','rx','rz') swaps rot_x and rot_y."""
    indices = []
    for entry in value:
        if isinstance(entry, str):
            name = entry.lower()
            if name not in _ANGULAR_AXIS_NAMES:
                raise ValueError("angular_axis_order entries must be rx/ry/rz or 0..2")
            indices.append(_ANGULAR_AXIS_NAMES.index(name))
        else:
            indices.append(int(entry))
    if sorted(indices) != [0, 1, 2]:
        raise ValueError("angular_axis_order must be a permutation of rx/ry/rz")
    return (indices[0], indices[1], indices[2])


def _twist_from_sample(
    sample: SpaceMouseSample,
    *,
    max_linear_velocity_m_s: float,
    max_angular_velocity_rad_s: float,
    deadband: float,
    response_curve_gamma: float,
    linear_axis_signs: tuple[float, float, float] = (1.0, 1.0, 1.0),
    angular_axis_signs: tuple[float, float, float] = (1.0, 1.0, 1.0),
    angular_axis_order: tuple[int, int, int] = (0, 1, 2),
) -> tuple[float, ...]:
    # Raw-axis remap before deadband/gamma: angular sources are permuted
    # (axis swap), then per-axis signs mirror the operator-perceived direction.
    angular_sources = (sample.rx, sample.ry, sample.rz)
    axes = (
        linear_axis_signs[0] * sample.tx,
        linear_axis_signs[1] * sample.ty,
        linear_axis_signs[2] * sample.tz,
        angular_axis_signs[0] * angular_sources[angular_axis_order[0]],
        angular_axis_signs[1] * angular_sources[angular_axis_order[1]],
        angular_axis_signs[2] * angular_sources[angular_axis_order[2]],
    )
    scaled = (
        _apply_soft_deadband(axes[0], deadband, response_curve_gamma)
        * max_linear_velocity_m_s,
        _apply_soft_deadband(axes[1], deadband, response_curve_gamma)
        * max_linear_velocity_m_s,
        _apply_soft_deadband(axes[2], deadband, response_curve_gamma)
        * max_linear_velocity_m_s,
        _apply_soft_deadband(axes[3], deadband, response_curve_gamma)
        * max_angular_velocity_rad_s,
        _apply_soft_deadband(axes[4], deadband, response_curve_gamma)
        * max_angular_velocity_rad_s,
        _apply_soft_deadband(axes[5], deadband, response_curve_gamma)
        * max_angular_velocity_rad_s,
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


def _apply_soft_deadband(value: float, deadband: float, gamma: float) -> float:
    value = float(value)
    if abs(value) <= deadband:
        return 0.0
    sign = math.copysign(1.0, value)
    magnitude = (abs(value) - deadband) / (1.0 - deadband)
    magnitude = max(0.0, min(1.0, magnitude))
    remapped = magnitude ** gamma
    return sign * remapped * (1.0 - deadband)
