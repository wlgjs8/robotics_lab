from __future__ import annotations

import math
from typing import Any, Mapping

from .models import ArmSnapshot, CircleOverlaySnapshot, Pose6D, StateSnapshot
from .safety import OperatorSafety
from .scene import _ROBOT_JOINT_NAMES

_SELECTED_MODE_COLOR = "green"
_INACTIVE_MODE_COLOR = "gray"
_JOINT_MONITOR_UNITS = ("deg", "rad")
_STAND_WORLD_MONITOR_UNITS = ("deg", "rad")
_STAND_WORLD_POSE_FIELDS = ("x", "y", "z", "rx", "ry", "rz")
_TCP_DISPLAY_MODES = ("auto", "actual", "reference", "both")
_SCENE_ASSET_ERROR_KEYS = (
    "urdf_error",
    "stand_mesh_error",
    "urdf_update_error",
    "scene_error",
)
_ASSET_INSTALL_HINT = "Install with python3 -m pip install -e rb_gui"


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


def _tcp_display_mode(handles: dict[str, Any]) -> str:
    selector = handles.get("tcp_display_mode", "auto")
    mode = selector if isinstance(selector, str) else getattr(selector, "value", "auto")
    return mode if mode in _TCP_DISPLAY_MODES else "auto"


def _update_tcp_display_buttons(handles: dict[str, Any]) -> None:
    selected = _tcp_display_mode(handles)
    for mode, button in handles.get("tcp_display_buttons", {}).items():
        try:
            button.color = _mode_button_color(mode, selected)
        except Exception:
            pass


def _format_controller_simulation_info(arm: ArmSnapshot) -> str:
    parts: list[str] = []
    mode = arm.controller_simulation_mode
    if isinstance(mode, Mapping):
        recommended = mode.get("recommended_tracking_pose")
        if isinstance(recommended, str):
            parts.append(f"controller_sim_recommended={recommended}")
    if arm.physical_motion_expected is not None:
        parts.append(f"physical_motion_expected={arm.physical_motion_expected}")
    elif isinstance(mode, Mapping):
        expected = mode.get("physical_motion_expected")
        if isinstance(expected, bool):
            parts.append(f"physical_motion_expected={expected}")
    if arm.controller_simulation_diagnostic_override_active is not None:
        parts.append(f"diagnostics_override={arm.controller_simulation_diagnostic_override_active}")
    return ", ".join(parts)


def _format_arm_tcp_tracking(arm_name: str, arm: ArmSnapshot, display_mode: str) -> str:
    selected = arm.selected_tcp_source(display_mode)
    parts = [
        f"{arm_name}",
        f"display={display_mode}",
        f"selected_source={selected}",
        f"actual_valid={arm.tcp_actual_valid}",
        f"ref_valid={arm.tcp_ref_valid}",
    ]
    if arm.tcp_tracking_source:
        parts.append(f"tcp_tracking_source={arm.tcp_tracking_source}")
    if arm.tcp_tracking_source_recommendation:
        parts.append(f"tcp_tracking_source_recommendation={arm.tcp_tracking_source_recommendation}")
    if arm.cartesian_available is not None:
        parts.append(f"cartesian_available={arm.cartesian_available}")
    if arm.controller_simulation_cartesian_enabled is not None:
        parts.append(f"controller_simulation_cartesian_enabled={arm.controller_simulation_cartesian_enabled}")
    if arm.controller_simulation_streaming_cartesian_available is not None:
        parts.append(
            "controller_simulation_streaming_cartesian_available="
            f"{arm.controller_simulation_streaming_cartesian_available}"
        )
    if arm.controller_simulation_physical_motion_detected is not None:
        parts.append(
            "controller_simulation_physical_motion_detected="
            f"{arm.controller_simulation_physical_motion_detected}"
        )
    controller_sim = _format_controller_simulation_info(arm)
    if controller_sim:
        parts.append(controller_sim)
    return " ".join(parts)


def _format_tcp_tracking_status(
    latest: StateSnapshot | None,
    *,
    stale: bool,
    display_mode: str = "auto",
) -> str:
    if latest is None:
        return "TCP tracking: no state"
    if stale:
        return "State stream stale"
    return "TCP tracking: " + "; ".join(
        (
            _format_arm_tcp_tracking("left", latest.left, display_mode),
            _format_arm_tcp_tracking("right", latest.right, display_mode),
        )
    )


def _format_bool_token(value: bool | None) -> str:
    if value is True:
        return "true"
    if value is False:
        return "false"
    return "missing"


def _format_pgmode_value(value: Any) -> str:
    if isinstance(value, bool):
        return _format_bool_token(value)
    if value is None:
        return "missing"
    return str(value)


def _arm_gate_value(arm: ArmSnapshot, key: str) -> Any:
    if isinstance(arm.cartesian_gate, Mapping) and key in arm.cartesian_gate:
        return arm.cartesian_gate.get(key)
    return getattr(arm, key, None)


def _same_or_per_arm(left: Any, right: Any) -> Any:
    if left == right:
        return left
    return f"left={_format_pgmode_value(left)},right={_format_pgmode_value(right)}"


def _pgmode_field(latest: StateSnapshot, key: str, *, top_fallback: Any = None) -> Any:
    left = _arm_gate_value(latest.left, key)
    right = _arm_gate_value(latest.right, key)
    if left is None and right is None:
        return top_fallback
    return _same_or_per_arm(left, right)


def _pgmode_bool_field(latest: StateSnapshot, key: str) -> bool | str | None:
    left = _arm_gate_value(latest.left, key)
    right = _arm_gate_value(latest.right, key)
    if not isinstance(left, bool):
        left = None
    if not isinstance(right, bool):
        right = None
    return _same_or_per_arm(left, right)


def _pgmode_cartesian_available(latest: StateSnapshot) -> bool | str | None:
    left = latest.left.controller_simulation_cartesian_available
    right = latest.right.controller_simulation_cartesian_available
    if left is None and right is None:
        left = latest.left.cartesian_available
        right = latest.right.cartesian_available
    return _same_or_per_arm(left, right)


def _pgmode_selected_tcp(latest: StateSnapshot, display_mode: str) -> str:
    return str(
        _same_or_per_arm(
            latest.left.selected_tcp_source(display_mode),
            latest.right.selected_tcp_source(display_mode),
        )
    )


def _policy_runner_lease_status(latest: StateSnapshot) -> str:
    command_source = latest.command_source
    source = command_source.display_source_id
    if command_source.expired is True:
        return "expired"
    if command_source.active is True and source == "policy_runner":
        return "active"
    if command_source.active is True and source:
        return "held_by_other"
    if command_source.active is False:
        return "inactive"
    return "missing"


def _format_pgmode_status(
    latest: StateSnapshot | None,
    *,
    stale: bool,
    display_mode: str = "auto",
) -> str:
    if latest is None:
        return "pgmode_sim: no state"
    if stale:
        return "State stream stale"

    backend = _pgmode_field(latest, "backend_type", top_fallback=latest.observed_backend)
    run_mode = _pgmode_field(latest, "run_mode", top_fallback=latest.observed_mode)
    operation_mode = _pgmode_field(latest, "operation_mode")
    physical_motion_expected = _pgmode_bool_field(latest, "physical_motion_expected")
    cartesian_available = _pgmode_cartesian_available(latest)
    physical_motion_detected = _pgmode_bool_field(latest, "controller_simulation_physical_motion_detected")
    selected_tcp = _pgmode_selected_tcp(latest, display_mode)
    lease_status = _policy_runner_lease_status(latest)
    source = latest.command_source.display_source_id
    session = latest.command_source.display_session_id
    command = latest.active_command_mode

    required: dict[str, Any] = {
        "backend": backend,
        "run_mode": run_mode,
        "operation_mode": operation_mode,
        "physical_motion_expected": physical_motion_expected,
        "cartesian_available": cartesian_available,
        "policy_runner_lease": lease_status,
        "source": source,
        "command": command,
        "selected_tcp": selected_tcp,
    }
    missing = [
        key
        for key, value in required.items()
        if value is None or value == "missing" or (key == "selected_tcp" and value == "none")
    ]
    parts = [
        "pgmode_sim:",
        f"backend={_format_pgmode_value(backend)}",
        f"run_mode={_format_pgmode_value(run_mode)}",
        f"operation_mode={_format_pgmode_value(operation_mode)}",
        f"physical_motion_expected={_format_pgmode_value(physical_motion_expected)}",
        f"cartesian_available={_format_pgmode_value(cartesian_available)}",
        f"policy_runner_lease={lease_status}",
        f"source={_format_pgmode_value(source)}",
        f"command={_format_pgmode_value(command)}",
        f"selected_tcp={selected_tcp}",
    ]
    if session:
        parts.append(f"session={session}")
    if latest.command_source.lease_timeout_sec is not None:
        parts.append(f"lease_timeout_sec={latest.command_source.lease_timeout_sec:g}")
    if latest.command_source.expires_time_ns is not None:
        parts.append(f"expires_time_ns={latest.command_source.expires_time_ns}")
    warnings: list[str] = []
    if physical_motion_expected is not False:
        warnings.append("physical_motion_expected_not_false")
    if physical_motion_detected is not False and physical_motion_detected is not None:
        warnings.append("controller_simulation_physical_motion_detected")
    if warnings:
        parts.append("warning=" + "|".join(warnings))
    if missing:
        parts.append("degraded missing=" + "|".join(missing))
    return " ".join(parts)


def _format_mm(value: float | None) -> str:
    if value is None:
        return "n/a"
    return f"{value * 1000.0:.1f} mm"


def _format_latency_ms(value: float | None) -> str:
    if value is None:
        return "n/a"
    return f"{value:.1f} ms"


def _format_circle_overlay_status(
    overlay: CircleOverlaySnapshot | None,
    *,
    stale: bool,
    enabled: bool = True,
) -> str:
    if not enabled:
        return "Circle overlay: disabled"
    if overlay is None:
        return "Circle overlay: no packets"
    state = "stale" if stale else "live"
    parts = [
        f"Circle overlay: {state}",
        f"run_id={overlay.run_id or 'n/a'}",
        f"profile={overlay.profile or 'n/a'}",
        f"controller={overlay.controller or 'n/a'}",
        f"arm={overlay.arm}",
        f"tracking_source={overlay.tracking_source or 'n/a'}",
        f"error={_format_mm(overlay.current_error_m)}",
        f"rms={_format_mm(overlay.running_rms_error_m)}",
        f"p95={_format_mm(overlay.running_p95_error_m)}",
        f"latency={_format_latency_ms(overlay.estimated_latency_ms)}",
        f"samples={overlay.sample_count}",
    ]
    if overlay.command_count is not None:
        parts.append(f"commands={overlay.command_count}")
    if overlay.physical_motion_expected is not None:
        parts.append(f"physical_motion_expected={overlay.physical_motion_expected}")
    if overlay.result_so_far:
        parts.append(f"result_so_far={overlay.result_so_far}")
    return ", ".join(parts)


def _format_scene_asset_status(scene_handles: Mapping[str, Any]) -> str:
    errors: list[str] = []
    for key in _SCENE_ASSET_ERROR_KEYS:
        value = scene_handles.get(key)
        if value:
            errors.append(f"{key}={value}")
    for key, value in sorted(scene_handles.items()):
        if key in _SCENE_ASSET_ERROR_KEYS:
            continue
        if key.endswith("_error") and value:
            errors.append(f"{key}={value}")
    if not errors:
        return "stand/URDF assets loaded"
    if not any(_ASSET_INSTALL_HINT in item for item in errors):
        errors.append(_ASSET_INSTALL_HINT)
    return "; ".join(errors)


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
