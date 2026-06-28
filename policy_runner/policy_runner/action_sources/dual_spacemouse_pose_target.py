from __future__ import annotations

import math
import os
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass

from policy_runner.action_sources.tcp_pose_target import (
    CARTESIAN_ACTION_REQUIREMENTS,
    cartesian_action_requirements,
    clamp_pose_delta,
    tcp_pose_target_stand_intent,
)
from policy_runner.robot_state_client import StateSnapshot
from policy_runner.servo_command_client import CommandIntent
from policy_runner.spacemouse import HidSpaceMouseReader, SpaceMouseReader, SpaceMouseSample


def _resolve_spacemouse_log_path() -> str | None:
    """Enabled by POLICY_RUNNER_SPACEMOUSE_TELEOP_LOG or the unified
    POLICY_RUNNER_TELEOP_CAPTURE (1/on/auto/true/yes -> new KST file in logs/)."""
    val = (
        os.environ.get("POLICY_RUNNER_SPACEMOUSE_TELEOP_LOG")
        or os.environ.get("POLICY_RUNNER_TELEOP_CAPTURE")
    )
    if not val:
        return None
    if val.lower() in ("1", "on", "auto", "true", "yes"):
        from datetime import datetime
        from pathlib import Path

        logs_dir = Path(__file__).resolve().parents[3] / "logs"
        logs_dir.mkdir(parents=True, exist_ok=True)
        return str(logs_dir / f"teleop_spacemouse_{datetime.now().strftime('%Y%m%d_%H%M%S')}_KST.log")
    return val


class _SpaceMouseStepLogger:
    """Per-step SpaceMouse diagnostic (raw stick vs applied signed delta, in stand
    frame), keyed by CLOCK_MONOTONIC ns so it aligns with the servo_log. Fail-safe:
    log() never raises."""

    def __init__(self, path: str):
        from datetime import datetime

        self._fh = open(path, "w", buffering=1, encoding="utf-8")
        self.path = path
        self._fh.write(f"# dual_spacemouse per-step log  started={datetime.now().isoformat()}\n")
        self._fh.write(
            "# fields: mono_ns=<CLOCK_MONOTONIC ns == servo loop_start_time_ns>  side  "
            "deadman=<0/1>  active=<0/1>  raw_stick=(tx,ty,tz,rx,ry,rz)  "
            "app_delta_mm=(dx,dy,dz) app_delta_deg=(dr,dp,dy)  lead_mm=<|target-live|>\n"
        )

    def log(self, side, *, mono_ns, deadman, active, sample, delta, target, live) -> None:
        try:
            rs = (
                "-" if sample is None
                else f"({sample.tx:+.2f},{sample.ty:+.2f},{sample.tz:+.2f},"
                     f"{sample.rx:+.2f},{sample.ry:+.2f},{sample.rz:+.2f})"
            )
            if delta is None:
                ad_mm = ad_deg = "-"
            else:
                ad_mm = f"({delta[0] * 1000:+.1f},{delta[1] * 1000:+.1f},{delta[2] * 1000:+.1f})"
                ad_deg = (
                    f"({math.degrees(delta[3]):+.2f},{math.degrees(delta[4]):+.2f},"
                    f"{math.degrees(delta[5]):+.2f})"
                )
            lead = "-"
            if target is not None and live is not None:
                lead = f"{1000 * math.sqrt(sum((target[i] - live[i]) ** 2 for i in range(3))):.1f}"
            self._fh.write(
                f"mono_ns={mono_ns}  side={side[0].upper()}  deadman={int(bool(deadman))}  "
                f"active={int(bool(active))}  raw_stick={rs}  "
                f"app_delta_mm={ad_mm} app_delta_deg={ad_deg}  lead_mm={lead}\n"
            )
        except Exception:
            pass

    def close(self) -> None:
        try:
            self._fh.close()
        except Exception:
            pass


class DualSpaceMousePoseTargetActionSource:
    requirements = CARTESIAN_ACTION_REQUIREMENTS

    def __init__(
        self,
        left_reader: SpaceMouseReader | None = None,
        right_reader: SpaceMouseReader | None = None,
        max_linear_step_m: float = 0.001,
        max_angular_step_rad: float = 0.01,
        max_target_lead_m: float = 0.05,
        max_target_lead_rad: float = 0.35,
        deadband: float = 0.08,
        activation_deadband: float | None = None,
        response_curve_gamma: float = 3.0,
        linear_axis_signs: tuple[float, ...] = (1.0, 1.0, 1.0),
        angular_axis_signs: tuple[float, ...] = (1.0, 1.0, 1.0),
        angular_axis_order: tuple[str, ...] = ("rx", "ry", "rz"),
        sample_stale_timeout_sec: float = 0.05,
        require_deadman: bool = True,
        startup_requires_neutral: bool = True,
        startup_neutral_hold_sec: float = 0.3,
        left_deadman_button: int = 0,
        right_deadman_button: int = 0,
        gripper_buttons_enable: bool = False,
        gripper_open_button: int = 0,
        gripper_close_button: int = 1,
        gripper_open_percent: float = 100.0,
        gripper_close_percent: float = 10.0,
        timeout_sec: float = 0.2,
        allow_rbpodo_controller_simulation: bool = False,
    ):
        if max_linear_step_m < 0.0:
            raise ValueError("max_linear_step_m must be non-negative")
        if max_angular_step_rad < 0.0:
            raise ValueError("max_angular_step_rad must be non-negative")
        if max_target_lead_m < 0.0:
            raise ValueError("max_target_lead_m must be non-negative")
        if max_target_lead_rad < 0.0:
            raise ValueError("max_target_lead_rad must be non-negative")
        if deadband < 0.0:
            raise ValueError("deadband must be non-negative")
        if response_curve_gamma < 1.0:
            raise ValueError("response_curve_gamma must be >= 1.0")
        if sample_stale_timeout_sec <= 0.0:
            raise ValueError("sample_stale_timeout_sec must be positive")
        if activation_deadband is not None and activation_deadband < 0.0:
            raise ValueError("activation_deadband must be non-negative")
        if startup_neutral_hold_sec < 0.0:
            raise ValueError("startup_neutral_hold_sec must be non-negative")
        if left_deadman_button < 0 or right_deadman_button < 0:
            raise ValueError("deadman buttons must be non-negative")
        if gripper_open_button < 0 or gripper_close_button < 0:
            raise ValueError("gripper buttons must be non-negative")
        if gripper_open_button == gripper_close_button:
            raise ValueError("gripper open and close buttons must differ")
        if not 0.0 <= gripper_open_percent <= 100.0:
            raise ValueError("gripper_open_percent must be in [0, 100]")
        if not 0.0 <= gripper_close_percent <= 100.0:
            raise ValueError("gripper_close_percent must be in [0, 100]")
        self.left_reader = left_reader if left_reader is not None else HidSpaceMouseReader(device_number=0)
        self.right_reader = right_reader if right_reader is not None else HidSpaceMouseReader(device_number=1)
        self.max_linear_step_m = float(max_linear_step_m)
        self.max_angular_step_rad = float(max_angular_step_rad)
        self.max_target_lead_m = float(max_target_lead_m)
        self.max_target_lead_rad = float(max_target_lead_rad)
        self.deadband = float(deadband)
        self.activation_deadband = (
            float(activation_deadband)
            if activation_deadband is not None
            else self.deadband * 1.5
        )
        self.response_curve_gamma = float(response_curve_gamma)
        self.linear_axis_signs = _axis_signs(linear_axis_signs, "linear_axis_signs")
        self.angular_axis_signs = _axis_signs(angular_axis_signs, "angular_axis_signs")
        self.angular_axis_order = _angular_axis_order(angular_axis_order)
        self.sample_stale_timeout_sec = float(sample_stale_timeout_sec)
        self.require_deadman = bool(require_deadman)
        self.startup_requires_neutral = bool(startup_requires_neutral)
        self.startup_neutral_hold_sec = float(startup_neutral_hold_sec)
        self.left_deadman_button = int(left_deadman_button)
        self.right_deadman_button = int(right_deadman_button)
        self.gripper_buttons_enable = bool(gripper_buttons_enable)
        self.gripper_open_button = int(gripper_open_button)
        self.gripper_close_button = int(gripper_close_button)
        self.gripper_open_percent = float(gripper_open_percent)
        self.gripper_close_percent = float(gripper_close_percent)
        self.timeout_sec = float(timeout_sec)
        self.requirements = cartesian_action_requirements(
            allow_rbpodo_controller_simulation=allow_rbpodo_controller_simulation
        )
        self._left_state = _SideState()
        self._right_state = _SideState()
        self.failed = False
        self.failure_message: str | None = None
        self._debug = os.environ.get("POLICY_RUNNER_TELEOP_DEBUG", "") == "1"
        self._debug_last_print = 0.0
        try:
            _p = _resolve_spacemouse_log_path()
            self._step_log = _SpaceMouseStepLogger(_p) if _p else None
        except Exception:
            self._step_log = None

    @property
    def engaged(self) -> bool:
        return self._left_state.active or self._right_state.active

    def reset_engagement(self) -> None:
        self._left_state.reset_engagement()
        self._right_state.reset_engagement()

    def close(self) -> None:
        if getattr(self, "_step_log", None) is not None:
            self._step_log.close()
        self.left_reader.close()
        self.right_reader.close()

    def next_intent(self, snapshot: StateSnapshot, now_monotonic: float) -> CommandIntent | None:
        self.failed = False
        self.failure_message = None
        left = self._target_from_reader(
            self.left_reader,
            self.left_deadman_button,
            self._left_state,
            snapshot,
            now_monotonic,
            side="left",
        )
        right = self._target_from_reader(
            self.right_reader,
            self.right_deadman_button,
            self._right_state,
            snapshot,
            now_monotonic,
            side="right",
        )
        if left.failure or right.failure:
            self.failed = True
            self.failure_message = left.failure_message or right.failure_message
            return CommandIntent.hold(timeout_sec=self.timeout_sec)
        if self._debug:
            self._debug_print(left.pose, right.pose)
        left_gripper = left.gripper_target
        right_gripper = right.gripper_target
        if left.pose is None and right.pose is None:
            if left_gripper is not None or right_gripper is not None:
                return CommandIntent.gripper_target(
                    left=left_gripper,
                    right=right_gripper,
                    timeout_sec=self.timeout_sec,
                )
            if left.released or right.released:
                return CommandIntent.hold(timeout_sec=self.timeout_sec)
            return None
        return tcp_pose_target_stand_intent(
            left=left.pose,
            right=right.pose,
            left_gripper=left_gripper,
            right_gripper=right_gripper,
            timeout_sec=self.timeout_sec,
            tcp_target_profile="spacemouse_precise",
            metadata={
                "action_source": "dual_spacemouse_pose_target",
                "source_conditioning_mode": "none",
            },
        )

    def _target_from_reader(
        self,
        reader: SpaceMouseReader,
        deadman_button: int,
        state: "_SideState",
        snapshot: StateSnapshot,
        now_monotonic: float,
        *,
        side: str,
    ) -> "_SideResult":
        try:
            sample = reader.read(timeout_sec=0.0)
        except Exception as exc:
            state.reset_all()
            return _SideResult(failure=True, failure_message=str(exc))

        if sample is None:
            if state.active:
                state.reset_all()
                return _SideResult(released=True)
            return _SideResult()

        if now_monotonic - sample.timestamp_monotonic > self.sample_stale_timeout_sec:
            was_active = state.active
            state.reset_all()
            return _SideResult(released=was_active)

        gripper_target = self._gripper_target_from_buttons(sample, state)
        if self._debug and any(sample.buttons):
            # Diagnostic: which raw button index each physical button reports, so
            # gripper open/close mapping can be set correctly (VERBOSE=1 only).
            print(
                f"[SM-btn] {side} buttons={tuple(int(b) for b in sample.buttons)} "
                f"(close_idx={self.gripper_close_button} open_idx={self.gripper_open_button})",
                flush=True,
            )

        if self.require_deadman:
            if not _deadman_active(sample, deadman_button):
                was_active = state.active
                state.reset_engagement()
                return _SideResult(released=was_active, gripper_target=gripper_target)
        else:
            raw_mag = _raw_magnitude(sample)
            if self.startup_requires_neutral and not state.neutral_confirmed:
                if raw_mag <= self.deadband:
                    if state.neutral_since is None:
                        state.neutral_since = now_monotonic
                    if now_monotonic - state.neutral_since >= self.startup_neutral_hold_sec:
                        state.neutral_confirmed = True
                else:
                    state.neutral_since = None
                if not state.neutral_confirmed:
                    return _SideResult(gripper_target=gripper_target)
            if not state.active and raw_mag <= self.activation_deadband:
                return _SideResult(gripper_target=gripper_target)

        live_pose = _tcp_pose_stand(snapshot, side)
        if live_pose is None:
            was_active = state.active
            state.reset_engagement()
            return _SideResult(released=was_active, gripper_target=gripper_target)
        if state.target_pose is None:
            state.target_pose = live_pose

        delta = _pose_delta_from_sample(
            sample,
            max_linear_step_m=self.max_linear_step_m,
            max_angular_step_rad=self.max_angular_step_rad,
            deadband=self.deadband,
            response_curve_gamma=self.response_curve_gamma,
            linear_axis_signs=self.linear_axis_signs,
            angular_axis_signs=self.angular_axis_signs,
            angular_axis_order=self.angular_axis_order,
        )
        has_delta = any(value != 0.0 for value in delta)
        if not state.active and not has_delta:
            return _SideResult(gripper_target=gripper_target)
        state.active = True
        if has_delta:
            state.neutral_pose_sent = False
            state.target_pose = _compose_local_delta(state.target_pose, delta)
            state.target_pose = _clamp_target_lead(
                state.target_pose,
                live_pose,
                max_target_lead_m=self.max_target_lead_m,
                max_target_lead_rad=self.max_target_lead_rad,
            )
        elif state.neutral_pose_sent:
            state.reset_engagement()
            return _SideResult(released=True, gripper_target=gripper_target)
        else:
            state.neutral_pose_sent = True
        if self._step_log is not None:
            self._step_log.log(
                side,
                mono_ns=int(now_monotonic * 1e9),
                deadman=_deadman_active(sample, deadman_button) if self.require_deadman else True,
                active=state.active,
                sample=sample,
                delta=delta,
                target=state.target_pose,
                live=live_pose,
            )
        return _SideResult(pose=state.target_pose, gripper_target=gripper_target)

    def _gripper_target_from_buttons(
        self,
        sample: SpaceMouseSample,
        state: "_SideState",
    ) -> float | None:
        if not self.gripper_buttons_enable:
            return None
        open_down = _button_active(sample.buttons, self.gripper_open_button)
        close_down = _button_active(sample.buttons, self.gripper_close_button)
        was_open_down = _button_active(state.last_buttons, self.gripper_open_button)
        was_close_down = _button_active(state.last_buttons, self.gripper_close_button)
        state.last_buttons = sample.buttons
        if open_down and close_down:
            return None
        if open_down and not was_open_down:
            return self.gripper_open_percent
        if close_down and not was_close_down:
            return self.gripper_close_percent
        return None

    def _debug_print(self, left_pose: tuple[float, ...] | None, right_pose: tuple[float, ...] | None) -> None:
        now = time.monotonic()
        if now - self._debug_last_print < 0.1:
            return
        self._debug_last_print = now
        left = "none" if left_pose is None else ",".join(f"{v:+.3f}" for v in left_pose)
        right = "none" if right_pose is None else ",".join(f"{v:+.3f}" for v in right_pose)
        print(f"[SM] left_target=({left}) right_target=({right})", flush=True)


@dataclass
class _SideState:
    active: bool = False
    target_pose: tuple[float, ...] | None = None
    neutral_pose_sent: bool = False
    neutral_confirmed: bool = False
    neutral_since: float | None = None
    last_buttons: tuple[bool, ...] = ()

    def reset_engagement(self) -> None:
        self.active = False
        self.target_pose = None
        self.neutral_pose_sent = False

    def reset_all(self) -> None:
        self.reset_engagement()
        self.neutral_confirmed = False
        self.neutral_since = None
        self.last_buttons = ()


@dataclass(frozen=True)
class _SideResult:
    pose: tuple[float, ...] | None = None
    gripper_target: float | None = None
    released: bool = False
    failure: bool = False
    failure_message: str | None = None


_ANGULAR_AXIS_NAMES = ("rx", "ry", "rz")


def _deadman_active(sample: SpaceMouseSample, deadman_button: int) -> bool:
    return _button_active(sample.buttons, deadman_button)


def _button_active(buttons: tuple[bool, ...], button: int) -> bool:
    if button >= len(buttons):
        return False
    return bool(buttons[button])


def _raw_magnitude(sample: SpaceMouseSample) -> float:
    return max(abs(sample.tx), abs(sample.ty), abs(sample.tz), abs(sample.rx), abs(sample.ry), abs(sample.rz))


def _axis_signs(value: Sequence[float], label: str) -> tuple[float, float, float]:
    signs = tuple(float(v) for v in value)
    if len(signs) != 3 or any(sign not in (-1.0, 1.0) for sign in signs):
        raise ValueError(f"{label} must be 3 entries of -1 or 1")
    return signs  # type: ignore[return-value]


def _angular_axis_order(value: Sequence[str | int]) -> tuple[int, int, int]:
    indices: list[int] = []
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


def _pose_delta_from_sample(
    sample: SpaceMouseSample,
    *,
    max_linear_step_m: float,
    max_angular_step_rad: float,
    deadband: float,
    response_curve_gamma: float,
    linear_axis_signs: tuple[float, float, float],
    angular_axis_signs: tuple[float, float, float],
    angular_axis_order: tuple[int, int, int],
) -> tuple[float, ...]:
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
        _apply_soft_deadband(axes[0], deadband, response_curve_gamma) * max_linear_step_m,
        _apply_soft_deadband(axes[1], deadband, response_curve_gamma) * max_linear_step_m,
        _apply_soft_deadband(axes[2], deadband, response_curve_gamma) * max_linear_step_m,
        _apply_soft_deadband(axes[3], deadband, response_curve_gamma) * max_angular_step_rad,
        _apply_soft_deadband(axes[4], deadband, response_curve_gamma) * max_angular_step_rad,
        _apply_soft_deadband(axes[5], deadband, response_curve_gamma) * max_angular_step_rad,
    )
    return clamp_pose_delta(scaled, max_linear_step_m, max_angular_step_rad)


def _tcp_pose_stand(snapshot: StateSnapshot, side: str) -> tuple[float, ...] | None:
    arm = snapshot.payload.get(side)
    if not isinstance(arm, Mapping):
        return None
    raw = arm.get("tcp_ref_stand") or arm.get("tcp_actual_stand")
    if not isinstance(raw, Mapping):
        return None
    return _pose_from_mapping(raw)


def _pose_from_mapping(raw: Mapping[str, object]) -> tuple[float, ...] | None:
    try:
        x = float(raw.get("x", 0.0))
        y = float(raw.get("y", 0.0))
        z = float(raw.get("z", 0.0))
        quat = raw.get("quaternion_xyzw")
        if isinstance(quat, Sequence) and len(quat) == 4:
            rotation = _quat_to_matrix(tuple(float(value) for value in quat))
        elif all(key in raw for key in ("qx", "qy", "qz", "qw")):
            rotation = _quat_to_matrix(
                (
                    float(raw.get("qx", 0.0)),
                    float(raw.get("qy", 0.0)),
                    float(raw.get("qz", 0.0)),
                    float(raw.get("qw", 1.0)),
                )
            )
        else:
            rotation = _rpy_to_matrix(
                float(raw.get("rx", 0.0)),
                float(raw.get("ry", 0.0)),
                float(raw.get("rz", 0.0)),
            )
    except (TypeError, ValueError):
        return None
    rx, ry, rz = _matrix_to_rpy(rotation)
    return (x, y, z, rx, ry, rz)


def _compose_local_delta(pose: tuple[float, ...], delta: tuple[float, ...]) -> tuple[float, ...]:
    x, y, z, rx, ry, rz = pose
    rotation = _rpy_to_matrix(rx, ry, rz)
    dx, dy, dz = _mat_vec(rotation, delta[:3])
    delta_rotation = _rpy_to_matrix(delta[3], delta[4], delta[5])
    next_rotation = _mat_mul(rotation, delta_rotation)
    nrx, nry, nrz = _matrix_to_rpy(next_rotation)
    return (x + dx, y + dy, z + dz, nrx, nry, nrz)


def _clamp_target_lead(
    target: tuple[float, ...],
    live: tuple[float, ...],
    *,
    max_target_lead_m: float,
    max_target_lead_rad: float,
) -> tuple[float, ...]:
    tx, ty, tz, trx, try_, trz = target
    lx, ly, lz, lrx, lry, lrz = live
    delta_pos = (tx - lx, ty - ly, tz - lz)
    dist = math.sqrt(sum(value * value for value in delta_pos))
    if max_target_lead_m == 0.0:
        pos = (lx, ly, lz)
    elif dist > max_target_lead_m and dist > 0.0:
        scale = max_target_lead_m / dist
        pos = (lx + delta_pos[0] * scale, ly + delta_pos[1] * scale, lz + delta_pos[2] * scale)
    else:
        pos = (tx, ty, tz)

    live_rot = _rpy_to_matrix(lrx, lry, lrz)
    target_rot = _rpy_to_matrix(trx, try_, trz)
    rel_rot = _mat_mul(_mat_transpose(live_rot), target_rot)
    rel_vec = _so3_log(rel_rot)
    angle = math.sqrt(sum(value * value for value in rel_vec))
    if max_target_lead_rad == 0.0:
        rot = live_rot
    elif angle > max_target_lead_rad and angle > 0.0:
        scale = max_target_lead_rad / angle
        rot = _mat_mul(live_rot, _so3_exp((rel_vec[0] * scale, rel_vec[1] * scale, rel_vec[2] * scale)))
    else:
        rot = target_rot
    rx, ry, rz = _matrix_to_rpy(rot)
    return (pos[0], pos[1], pos[2], rx, ry, rz)


def _apply_soft_deadband(value: float, deadband: float, gamma: float) -> float:
    value = float(value)
    if abs(value) <= deadband:
        return 0.0
    sign = math.copysign(1.0, value)
    magnitude = (abs(value) - deadband) / max(1e-12, 1.0 - deadband)
    magnitude = max(0.0, min(1.0, magnitude))
    return sign * (magnitude ** gamma) * (1.0 - deadband)


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> tuple[float, ...]:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        cy * cp,
        cy * sp * sr - sy * cr,
        cy * sp * cr + sy * sr,
        sy * cp,
        sy * sp * sr + cy * cr,
        sy * sp * cr - cy * sr,
        -sp,
        cp * sr,
        cp * cr,
    )


def _matrix_to_rpy(m: Sequence[float]) -> tuple[float, float, float]:
    m = tuple(float(value) for value in m)
    pitch = math.asin(max(-1.0, min(1.0, -m[6])))
    cp = math.cos(pitch)
    if abs(cp) > 1e-9:
        roll = math.atan2(m[7], m[8])
        yaw = math.atan2(m[3], m[0])
    else:
        roll = 0.0
        yaw = math.atan2(-m[1], m[4])
    return (_wrap_pi(roll), _wrap_pi(pitch), _wrap_pi(yaw))


def _quat_to_matrix(q: Sequence[float]) -> tuple[float, ...]:
    qx, qy, qz, qw = (float(value) for value in q)
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0:
        return _rpy_to_matrix(0.0, 0.0, 0.0)
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    return (
        1 - 2 * (qy * qy + qz * qz),
        2 * (qx * qy - qz * qw),
        2 * (qx * qz + qy * qw),
        2 * (qx * qy + qz * qw),
        1 - 2 * (qx * qx + qz * qz),
        2 * (qy * qz - qx * qw),
        2 * (qx * qz - qy * qw),
        2 * (qy * qz + qx * qw),
        1 - 2 * (qx * qx + qy * qy),
    )


def _mat_mul(a: Sequence[float], b: Sequence[float]) -> tuple[float, ...]:
    return tuple(
        sum(float(a[3 * r + k]) * float(b[3 * k + c]) for k in range(3))
        for r in range(3)
        for c in range(3)
    )


def _mat_transpose(m: Sequence[float]) -> tuple[float, ...]:
    return (float(m[0]), float(m[3]), float(m[6]), float(m[1]), float(m[4]), float(m[7]), float(m[2]), float(m[5]), float(m[8]))


def _mat_vec(m: Sequence[float], v: Sequence[float]) -> tuple[float, float, float]:
    return (
        float(m[0]) * float(v[0]) + float(m[1]) * float(v[1]) + float(m[2]) * float(v[2]),
        float(m[3]) * float(v[0]) + float(m[4]) * float(v[1]) + float(m[5]) * float(v[2]),
        float(m[6]) * float(v[0]) + float(m[7]) * float(v[1]) + float(m[8]) * float(v[2]),
    )


def _so3_log(m: Sequence[float]) -> tuple[float, float, float]:
    trace = float(m[0]) + float(m[4]) + float(m[8])
    cos_theta = max(-1.0, min(1.0, (trace - 1.0) * 0.5))
    theta = math.acos(cos_theta)
    if theta < 1e-9:
        return (
            0.5 * (float(m[7]) - float(m[5])),
            0.5 * (float(m[2]) - float(m[6])),
            0.5 * (float(m[3]) - float(m[1])),
        )
    scale = theta / (2.0 * math.sin(theta))
    return (
        scale * (float(m[7]) - float(m[5])),
        scale * (float(m[2]) - float(m[6])),
        scale * (float(m[3]) - float(m[1])),
    )


def _so3_exp(v: Sequence[float]) -> tuple[float, ...]:
    x, y, z = (float(value) for value in v)
    theta = math.sqrt(x * x + y * y + z * z)
    if theta < 1e-9:
        return (1.0, -z, y, z, 1.0, -x, -y, x, 1.0)
    kx, ky, kz = x / theta, y / theta, z / theta
    c = math.cos(theta)
    s = math.sin(theta)
    v1 = 1.0 - c
    return (
        c + kx * kx * v1,
        kx * ky * v1 - kz * s,
        kx * kz * v1 + ky * s,
        ky * kx * v1 + kz * s,
        c + ky * ky * v1,
        ky * kz * v1 - kx * s,
        kz * kx * v1 - ky * s,
        kz * ky * v1 + kx * s,
        c + kz * kz * v1,
    )


def _wrap_pi(value: float) -> float:
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi
