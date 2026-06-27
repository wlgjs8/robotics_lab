from __future__ import annotations

import math
import os
import time
import warnings
from dataclasses import dataclass

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
        require_deadman: bool = True,
        activation_deadband: float | None = None,
        startup_requires_neutral: bool = True,
        startup_neutral_hold_sec: float = 0.3,
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
        if activation_deadband is not None and activation_deadband < 0.0:
            raise ValueError("activation_deadband must be non-negative")
        if startup_neutral_hold_sec < 0.0:
            raise ValueError("startup_neutral_hold_sec must be non-negative")
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
        # Buttonless (require_deadman=False): cap deflection is the intent gate.
        # IDLE->ACTIVE needs the larger activation_deadband (hysteresis vs the
        # per-axis deadband used while streaming); startup/reconnect requires
        # the cap held neutral for startup_neutral_hold_sec first.
        self.require_deadman = bool(require_deadman)
        self.activation_deadband = (
            float(activation_deadband)
            if activation_deadband is not None
            else self.deadband * 1.5
        )
        self.startup_requires_neutral = bool(startup_requires_neutral)
        self.startup_neutral_hold_sec = float(startup_neutral_hold_sec)
        self.left_deadman_button = int(left_deadman_button)
        self.right_deadman_button = int(right_deadman_button)
        self.timeout_sec = timeout_sec
        self.requirements = cartesian_action_requirements(
            allow_rbpodo_controller_simulation=allow_rbpodo_controller_simulation
        )
        self._left_state = _SideState()
        self._right_state = _SideState()
        # POLICY_RUNNER_TELEOP_DEBUG=1: print raw samples/armed/twist at 10 Hz.
        self._debug = os.environ.get("POLICY_RUNNER_TELEOP_DEBUG", "") == "1"
        self._debug_raw: dict[str, SpaceMouseSample | None] = {"left": None, "right": None}
        self._debug_last_print = 0.0

    def next_intent(self, snapshot: StateSnapshot, now_monotonic: float) -> CommandIntent | None:
        _ = snapshot
        left, left_released, left_present = self._twist_from_reader(
            self.left_reader,
            self.left_deadman_button,
            self._left_state,
            now_monotonic,
            side="left",
        )
        right, right_released, right_present = self._twist_from_reader(
            self.right_reader,
            self.right_deadman_button,
            self._right_state,
            now_monotonic,
            side="right",
        )
        if not left_present and not right_present and (left_released or right_released):
            # An arm just disengaged and nothing is streaming: zero BOTH arms
            # once so no stale twist survives on either side.
            left = _ZERO_TWIST
            right = _ZERO_TWIST
        if self._debug:
            self._debug_print(left, right)
        if left is None and right is None and not left_present and not right_present:
            return None
        return tcp_twist_local_intent(left=left, right=right, timeout_sec=self.timeout_sec)

    def _side_state_label(self, state: "_SideState") -> str:
        if self.require_deadman:
            return "ARMED" if state.active else "DISARMED"
        if self.startup_requires_neutral and not state.neutral_confirmed:
            return "NEUTRAL_WAIT"
        return "ACTIVE" if state.active else "IDLE"

    def _debug_print(
        self,
        left_twist: tuple[float, ...] | None,
        right_twist: tuple[float, ...] | None,
    ) -> None:
        now = time.monotonic()
        if now - self._debug_last_print < 0.1:
            return
        self._debug_last_print = now
        parts = []
        for label, side, twist, state in (
            ("LEFT ", "left", left_twist, self._left_state),
            ("RIGHT", "right", right_twist, self._right_state),
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
            parts.append(
                f"{label} axes=({axes}) btns={pressed} state={self._side_state_label(state)} twist=({twist_str})"
            )
        print(f"[SM] {' | '.join(parts)}", flush=True)

    def close(self) -> None:
        self.left_reader.close()
        self.right_reader.close()

    def _twist_from_reader(
        self,
        reader: SpaceMouseReader,
        deadman_button: int,
        state: "_SideState",
        now_monotonic: float,
        *,
        side: str = "left",
    ) -> tuple[tuple[float, ...] | None, bool, bool]:
        """Per-arm intent gate. Returns (twist, released, present).

        present=True while the arm is engaged (deadman held / cap deflected);
        released=True exactly once when it disengages (the zero-twist tick).

        deadman mode (require_deadman=True): button pressed == engaged; zero
        twist while pressed keeps streaming presence (legacy behavior).
        buttonless mode: IDLE -> ACTIVE when the raw cap deflection exceeds
        activation_deadband (hysteresis vs the streaming deadband); returning
        to neutral sends one zero twist and goes back to IDLE. Startup and
        stale-reconnect require the cap held neutral first.
        """
        sample = reader.read(timeout_sec=0.0)
        if self._debug and sample is not None:
            self._debug_raw[side] = sample

        if sample is None:
            if state.last_twist is None or state.last_sample_monotonic is None:
                return None, False, False
            if now_monotonic - state.last_sample_monotonic <= self.sample_hold_timeout_sec:
                return state.last_twist, False, True
            # Stale: zero once and drop the engagement. Buttonless mode must
            # re-confirm neutral before moving again (the device may wake up
            # deflected, e.g. carried while asleep).
            was_active = state.active
            state.reset_engagement()
            state.reset_neutral_interlock()
            return (_ZERO_TWIST, True, False) if was_active else (None, False, False)

        if self.require_deadman:
            if not _deadman_active(sample, deadman_button):
                if state.active:
                    state.reset_engagement()
                    return _ZERO_TWIST, True, False
                return None, False, False
        else:
            raw_mag = max(
                abs(sample.tx), abs(sample.ty), abs(sample.tz),
                abs(sample.rx), abs(sample.ry), abs(sample.rz),
            )
            if self.startup_requires_neutral and not state.neutral_confirmed:
                if raw_mag <= self.deadband:
                    if state.neutral_since is None:
                        state.neutral_since = now_monotonic
                    if now_monotonic - state.neutral_since >= self.startup_neutral_hold_sec:
                        state.neutral_confirmed = True
                else:
                    state.neutral_since = None
                if not state.neutral_confirmed:
                    return None, False, False
            if not state.active and raw_mag <= self.activation_deadband:
                # IDLE: below the activation threshold nothing is generated
                # (no zero-twist spam, no ArmMotion).
                return None, False, False

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
            if self.require_deadman:
                # Deadman held at neutral: stay engaged, hold zero.
                state.active = True
                state.last_twist = _ZERO_TWIST
                state.last_sample_monotonic = sample.timestamp_monotonic
                return None, False, True
            if state.active:
                # Buttonless: cap returned to neutral -> one zero twist, IDLE.
                state.reset_engagement()
                return _ZERO_TWIST, True, False
            return None, False, False

        state.active = True
        state.last_twist = twist
        state.last_sample_monotonic = sample.timestamp_monotonic
        return twist, False, True


@dataclass
class _SideState:
    """Per-arm engagement state for DualSpaceMouseCartesianActionSource."""

    active: bool = False
    last_twist: tuple[float, ...] | None = None
    last_sample_monotonic: float | None = None
    # Buttonless startup/reconnect interlock.
    neutral_confirmed: bool = False
    neutral_since: float | None = None

    def reset_engagement(self) -> None:
        self.active = False
        self.last_twist = None
        self.last_sample_monotonic = None

    def reset_neutral_interlock(self) -> None:
        self.neutral_confirmed = False
        self.neutral_since = None


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
