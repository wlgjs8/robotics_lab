from __future__ import annotations

import math
from typing import Any, Mapping

from .models import ArmSnapshot, Pose6D, StateSnapshot
from .safety import OperatorSafety
from .scene import _ROBOT_JOINT_NAMES

_SELECTED_MODE_COLOR = "green"
_INACTIVE_MODE_COLOR = "gray"
_JOINT_MONITOR_UNITS = ("deg", "rad")
_STAND_WORLD_MONITOR_UNITS = ("deg", "rad")
_STAND_WORLD_POSE_FIELDS = ("x", "y", "z", "rx", "ry", "rz")


def _mode_button_color(mode: str, desired_mode: str) -> str:
    return _SELECTED_MODE_COLOR if mode == desired_mode else _INACTIVE_MODE_COLOR


def _format_joints(q_values: tuple[float, ...] | None) -> str:
    if q_values is None:
        return "invalid"
    return ", ".join(f"{value:.2f}" for value in q_values)


def _joint_monitor_unit(handles: dict[str, Any]) -> str:
    selector = handles.get("joint_monitor_unit", "deg")
    unit = selector if isinstance(selector, str) else getattr(selector, "value", "deg")
    return unit if unit in _JOINT_MONITOR_UNITS else "deg"


def _update_joint_monitor_unit_buttons(handles: dict[str, Any]) -> None:
    selected = _joint_monitor_unit(handles)
    for unit, button in handles.get("joint_monitor_unit_buttons", {}).items():
        try:
            button.color = _mode_button_color(unit, selected)
        except Exception:
            pass


def _stand_world_monitor_unit(handles: dict[str, Any]) -> str:
    selector = handles.get("stand_world_monitor_unit", "deg")
    unit = selector if isinstance(selector, str) else getattr(selector, "value", "deg")
    return unit if unit in _STAND_WORLD_MONITOR_UNITS else "deg"


def _update_stand_world_monitor_unit_buttons(handles: dict[str, Any]) -> None:
    selected = _stand_world_monitor_unit(handles)
    for unit, button in handles.get("stand_world_monitor_unit_buttons", {}).items():
        try:
            button.color = _mode_button_color(unit, selected)
        except Exception:
            pass


def _format_joint_monitor_value(q_values: tuple[float, ...] | None, index: int, *, valid: bool, unit: str) -> str:
    if not valid or q_values is None:
        return "invalid"
    if len(q_values) != len(_ROBOT_JOINT_NAMES) or not all(math.isfinite(float(value)) for value in q_values):
        return "invalid"
    if index < 0 or index >= len(_ROBOT_JOINT_NAMES):
        return "invalid"
    if unit == "rad":
        return f"{math.radians(q_values[index]):.4f} rad"
    return f"{q_values[index]:.2f} deg"


def _format_stand_world_pose_value(pose: Pose6D | None, field: str, *, valid: bool, unit: str) -> str:
    if not valid or pose is None or field not in _STAND_WORLD_POSE_FIELDS:
        return "invalid"
    value = getattr(pose, field, None)
    if not isinstance(value, int | float) or not math.isfinite(float(value)):
        return "invalid"
    parsed = float(value)
    if field in ("x", "y", "z"):
        return f"{parsed * 1000.0:.1f} mm"
    if unit == "rad":
        return f"{parsed:.4f} rad"
    return f"{math.degrees(parsed):.2f} deg"


def _arm_fk_status(arm: ArmSnapshot) -> str:
    if not arm.has_valid_joint_state:
        return "invalid joint state"
    if arm.tcp_deferred:
        return "deferred"
    if not arm.has_valid_tcp_pose:
        return "invalid TCP pose"
    return "available"


def _format_fk_status(latest: StateSnapshot | None, *, stale: bool) -> str:
    if latest is None:
        return "FK: no state"
    if stale:
        return "State stream stale"
    left = _arm_fk_status(latest.left)
    right = _arm_fk_status(latest.right)
    if left == right:
        return f"FK: {left}"
    return f"FK: left {left}, right {right}"


def _optional_finite(value: Any) -> float | None:
    if not isinstance(value, int | float):
        return None
    parsed = float(value)
    return parsed if math.isfinite(parsed) else None


def _format_arm_cartesian_solve(arm: str, raw_arm: Any) -> str:
    if isinstance(raw_arm, ArmSnapshot):
        solve_obj = raw_arm.cartesian_solve
        if solve_obj is None:
            return f"{arm}=unavailable"
        solve: Mapping[str, Any] = {
            "attempted": solve_obj.attempted,
            "status": solve_obj.status,
            "position_error_m": solve_obj.position_error_m,
            "orientation_error_rad": solve_obj.orientation_error_rad,
            "ik_iterations": solve_obj.ik_iterations,
            "ik_duration_us": solve_obj.ik_duration_us,
            "ik_timed_out": solve_obj.ik_timed_out,
            "path_active": solve_obj.path_active,
            "path_s": solve_obj.path_s,
            "path_line_deviation_m": solve_obj.path_line_deviation_m,
            "path_orientation_error_rad": solve_obj.path_orientation_error_rad,
            "path_done": solve_obj.path_done,
        }
    elif isinstance(raw_arm, Mapping):
        raw_solve = raw_arm.get("cartesian_solve")
        if not isinstance(raw_solve, Mapping):
            return f"{arm}=unavailable"
        solve = raw_solve
    else:
        return f"{arm}=unavailable"
    parts: list[str] = [f"{arm}"]
    status = solve.get("status")
    if isinstance(status, str) and status:
        parts.append(status)
    if isinstance(solve.get("attempted"), bool):
        parts.append(f"attempted={solve['attempted']}")
    position_error_m = _optional_finite(solve.get("position_error_m"))
    if position_error_m is not None:
        parts.append(f"pos_err={position_error_m:.6g} m")
    orientation_error_rad = _optional_finite(solve.get("orientation_error_rad"))
    if orientation_error_rad is not None:
        parts.append(f"ori_err={orientation_error_rad:.6g} rad")
    ik_iterations = solve.get("ik_iterations")
    if isinstance(ik_iterations, int):
        parts.append(f"iter={ik_iterations}")
    ik_duration_us = _optional_finite(solve.get("ik_duration_us"))
    if ik_duration_us is not None:
        parts.append(f"dur={ik_duration_us:.3g} us")
    ik_timed_out = solve.get("ik_timed_out")
    if isinstance(ik_timed_out, bool):
        parts.append(f"timed_out={ik_timed_out}")
    path_active = solve.get("path_active")
    if isinstance(path_active, bool):
        parts.append(f"path_active={path_active}")
    path_s = _optional_finite(solve.get("path_s"))
    if path_s is not None:
        parts.append(f"path_s={path_s:.3f}")
    path_line_deviation_m = _optional_finite(solve.get("path_line_deviation_m"))
    if path_line_deviation_m is not None:
        parts.append(f"line_dev={path_line_deviation_m:.6g} m")
    path_orientation_error_rad = _optional_finite(solve.get("path_orientation_error_rad"))
    if path_orientation_error_rad is not None:
        parts.append(f"path_ori_err={path_orientation_error_rad:.6g} rad")
    path_done = solve.get("path_done")
    if isinstance(path_done, bool):
        parts.append(f"path_done={path_done}")
    return " ".join(parts)


def _format_cartesian_solve_status(latest: StateSnapshot | None, *, stale: bool) -> str:
    if latest is None:
        return "IK: no state"
    if stale:
        return "State stream stale"
    return "IK: " + "; ".join(
        (
            _format_arm_cartesian_solve("left", latest.left),
            _format_arm_cartesian_solve("right", latest.right),
        )
    )


def _set_disabled(handle: Any, disabled: bool) -> None:
    try:
        handle.disabled = disabled
    except Exception:
        pass


def _format_tcp_command_status(safety: OperatorSafety, latest: StateSnapshot | None, *, stale: bool) -> str:
    parts: list[str] = []
    if safety.last_tcp_command != "none":
        parts.append(f"last sent: {safety.last_tcp_command}")
    if latest is not None and not stale:
        parts.append(f"server verdict: {latest.safety_verdict}")
    reason = safety.tcp_command_disabled_reason()
    if reason:
        parts.append(f"disabled: {reason}")
    else:
        parts.append("enabled: TCP PTP, TCP Linear, and low-level delta debug available")
    return "; ".join(parts)


def _update_joint_monitor(handles: dict[str, Any], latest: StateSnapshot | None, *, stale: bool) -> None:
    if "joint_monitor_status" not in handles:
        return
    unit = _joint_monitor_unit(handles)
    _update_joint_monitor_unit_buttons(handles)
    value_handles = handles.get("joint_monitor_values", {})
    if latest is None:
        handles["joint_monitor_status"].value = f"No state stream, unit={unit}"
        for arm in ("left", "right"):
            for handle in value_handles.get(arm, ()):
                handle.value = "invalid"
        return
    state = "stale" if stale else "live"
    handles["joint_monitor_status"].value = f"{state}, unit={unit}, tick={latest.tick}"
    for arm, arm_state in (("left", latest.left), ("right", latest.right)):
        for index, handle in enumerate(value_handles.get(arm, ())):
            handle.value = _format_joint_monitor_value(
                arm_state.q_actual_deg,
                index,
                valid=arm_state.has_valid_joint_state,
                unit=unit,
            )


def _update_stand_world_monitor(handles: dict[str, Any], latest: StateSnapshot | None, *, stale: bool) -> None:
    if "stand_world_monitor_status" not in handles:
        return
    unit = _stand_world_monitor_unit(handles)
    _update_stand_world_monitor_unit_buttons(handles)
    value_handles = handles.get("stand_world_monitor_values", {})
    if latest is None:
        handles["stand_world_monitor_status"].value = f"No state stream, xyz=mm, rpy={unit}"
        for arm in ("left", "right"):
            for handle in value_handles.get(arm, {}).values():
                handle.value = "invalid"
        return
    state = "stale" if stale else "live"
    handles["stand_world_monitor_status"].value = f"{state}, xyz=mm, rpy={unit}, tick={latest.tick}"
    if stale:
        for arm in ("left", "right"):
            for handle in value_handles.get(arm, {}).values():
                handle.value = "invalid"
        return
    for arm, arm_state in (("left", latest.left), ("right", latest.right)):
        valid = bool(arm_state.has_valid_tcp_pose and arm_state.tcp_stand is not None and not arm_state.tcp_deferred)
        for field, handle in value_handles.get(arm, {}).items():
            handle.value = _format_stand_world_pose_value(
                arm_state.tcp_stand,
                field,
                valid=valid,
                unit=unit,
            )
