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


def _format_force_vec6(values: tuple[float, ...]) -> str:
    return "[" + ",".join(f"{value:+.4f}" for value in values) + "]"


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


def _format_arm_force_status(arm: ArmSnapshot) -> str:
    parts: list[str] = []
    force_torque = arm.force_torque
    if force_torque is not None:
        if force_torque.enabled is not None:
            parts.append(f"sensor_enabled={force_torque.enabled}")
        if force_torque.source is not None:
            parts.append(f"source={force_torque.source}")
        if force_torque.source_assurance is not None:
            parts.append(f"assurance={force_torque.source_assurance}")
        if force_torque.sensor_health_verified is not None:
            parts.append(f"health_verified={force_torque.sensor_health_verified}")
        if force_torque.safety_rated is not None:
            parts.append(f"safety_rated={force_torque.safety_rated}")
        if force_torque.healthy is not None:
            parts.append(f"healthy={force_torque.healthy}")
        if force_torque.stale is not None:
            parts.append(f"sensor_stale={force_torque.stale}")
        if force_torque.freshness_value is not None:
            parts.append(f"freshness={force_torque.freshness_value}")
        if force_torque.reason is not None:
            parts.append(f"sensor_reason={force_torque.reason}")
        if force_torque.auto_tare_enabled:
            tare = force_torque.tare_state or "unknown"
            if force_torque.tare_sample_count is not None and tare == "collecting":
                tare += f"({force_torque.tare_sample_count})"
            parts.append(f"zero={tare}")
            if force_torque.tare_valid is not None:
                parts.append(f"zero_valid={force_torque.tare_valid}")
            if force_torque.tare_reason:
                parts.append(f"zero_reason={force_torque.tare_reason}")
    force_control = arm.force_control
    if force_control is not None:
        if force_control.enabled is not None:
            parts.append(f"controller_enabled={force_control.enabled}")
        if force_control.operating_mode is not None:
            parts.append(f"mode={force_control.operating_mode}")
        if force_control.state is not None:
            parts.append(f"controller={force_control.state}")
        if force_control.contact_active is not None:
            parts.append(f"contact={force_control.contact_active}")
        if force_control.normal_contact_active is not None:
            parts.append(f"normal_contact={force_control.normal_contact_active}")
        if force_control.transverse_contact_active is not None:
            parts.append(
                f"transverse_contact={force_control.transverse_contact_active}"
            )
        if force_control.rotational_contact_active is not None:
            parts.append(
                f"rotational_contact={force_control.rotational_contact_active}"
            )
        if force_control.compliance_active is not None:
            parts.append(f"compliance={force_control.compliance_active}")
        if force_control.normal_regulating is not None:
            parts.append(f"normal_regulating={force_control.normal_regulating}")
        if force_control.transverse_regulating is not None:
            parts.append(
                f"transverse_regulating={force_control.transverse_regulating}"
            )
        if force_control.rotational_regulating is not None:
            parts.append(f"rotational_regulating={force_control.rotational_regulating}")
        if force_control.loading_projection_active is not None:
            parts.append(
                f"loading_projection={force_control.loading_projection_active}"
            )
        if force_control.compliance_equilibrium_source is not None:
            parts.append(
                f"equilibrium_source={force_control.compliance_equilibrium_source}"
            )
        if force_control.compliance_equilibrium_stand is not None:
            equilibrium = force_control.compliance_equilibrium_stand
            parts.append(
                "equilibrium_stand="
                f"[{equilibrium.x:.4f},{equilibrium.y:.4f},{equilibrium.z:.4f},"
                f"{equilibrium.rx:.3f},{equilibrium.ry:.3f},{equilibrium.rz:.3f}]"
            )
        if force_control.compliance_recenter_active is not None:
            parts.append(f"recenter={force_control.compliance_recenter_active}")
        vector_fields = (
            ("control_wrench_surface", force_control.control_wrench_surface),
            ("wrench_error_surface", force_control.wrench_error_surface),
            ("compliance_offset_surface", force_control.compliance_offset_surface),
            ("compliance_velocity_surface", force_control.compliance_velocity_surface),
            (
                "compliance_acceleration_surface",
                force_control.compliance_acceleration_surface,
            ),
            ("raw_policy_delta_surface", force_control.raw_policy_delta_surface),
            (
                "accepted_policy_delta_surface",
                force_control.accepted_policy_delta_surface,
            ),
        )
        for label, values in vector_fields:
            if values is not None:
                parts.append(f"{label}={_format_force_vec6(values)}")
        if force_control.measured_force_n is not None:
            parts.append(f"normal_force={force_control.measured_force_n:.3f}N")
        if force_control.fast_normal_force_n is not None:
            parts.append(f"fast_normal={force_control.fast_normal_force_n:.3f}N")
        if force_control.fast_force_norm_n is not None:
            parts.append(f"force_norm={force_control.fast_force_norm_n:.3f}N")
        if force_control.fast_torque_norm_nm is not None:
            parts.append(f"torque_norm={force_control.fast_torque_norm_nm:.3f}Nm")
        if force_control.contact_threshold_exceeded is not None:
            parts.append(f"contact_threshold={force_control.contact_threshold_exceeded}")
        if force_control.hard_limit_threshold_exceeded is not None:
            parts.append(
                f"hard_threshold={force_control.hard_limit_threshold_exceeded}"
            )
        if force_control.hard_limit_sample_count is not None:
            parts.append(f"hard_count={force_control.hard_limit_sample_count}")
        if force_control.hard_limit_exceeded is not None:
            parts.append(f"hard_limit={force_control.hard_limit_exceeded}")
        if force_control.target_force_n is not None:
            parts.append(f"target={force_control.target_force_n:.3f}N")
        if force_control.correction_m is not None:
            parts.append(f"unload={force_control.correction_m * 1000.0:.3f}mm")
        if force_control.saturated is not None:
            parts.append(f"saturated={force_control.saturated}")
        if force_control.compliance_limit_axes is not None:
            labels = ("x", "y", "z", "rx", "ry", "rz")
            active = [
                label for label, limited in zip(
                    labels, force_control.compliance_limit_axes, strict=True
                ) if limited
            ]
            parts.append(f"limit_axes={'+'.join(active) if active else 'none'}")
        if force_control.compliance_limit_reason is not None:
            parts.append(f"limit_reason={force_control.compliance_limit_reason}")
        if force_control.motion_epoch is not None:
            parts.append(f"controller_epoch={force_control.motion_epoch}")
        if force_control.fault_reason is not None:
            parts.append(f"fault={force_control.fault_reason}")
    return ", ".join(parts) if parts else "telemetry unavailable"


def _format_force_status(latest: StateSnapshot | None, *, stale: bool) -> str:
    if latest is None:
        return "Force: no state"
    if stale:
        return "State stream stale"
    epoch = str(latest.motion_epoch) if latest.motion_epoch is not None else "unavailable"
    return (
        f"Force: motion_epoch={epoch}; "
        f"left {_format_arm_force_status(latest.left)}; "
        f"right {_format_arm_force_status(latest.right)}"
    )


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


def _format_self_collision_status(
    latest: StateSnapshot | None,
    *,
    stale: bool,
) -> str:
    if latest is None:
        return "self-collision: no state"
    if stale:
        return "State stream stale"
    sc = latest.self_collision
    if not isinstance(sc, Mapping):
        return "self-collision: disabled"
    if not bool(sc.get("enabled", False)):
        return "self-collision: disabled"

    margin = sc.get("margin_m")
    margin_txt = f"{float(margin) * 1000:.0f}mm" if isinstance(margin, (int, float)) else "?"
    if not bool(sc.get("checked", False)):
        return f"self-collision: ON margin={margin_txt} (geometry unavailable)"

    clearance = sc.get("min_clearance_m")
    clearance_txt = (
        f"{float(clearance) * 1000:.0f}mm" if isinstance(clearance, (int, float)) else "?"
    )
    violated = bool(sc.get("violated", False))
    status = "VIOLATED" if violated else "ok"
    out = f"self-collision: {status} clearance={clearance_txt} margin={margin_txt}"
    # Name the closest checked pair (the COMMANDED-pose verdict the guard acts on)
    # so the operator sees WHICH parts are closest without reading server logs.
    pair_txt = _closest_pair_text(sc)
    if pair_txt:
        out += f" [{pair_txt}]"
    return out


def _short_geom(name: Any) -> str:
    """Shorten a collision-geometry name for display, e.g.
    'dual_rb3_730e_left_link4_2' -> 'left_link4_2', 'stand_body_shoulder_0' -> 'shoulder_0'."""
    if not isinstance(name, str) or not name:
        return "?"
    return name.replace("dual_rb3_730e_", "").replace("stand_body_", "").replace("stand_", "")


def _closest_pair_text(sc: Mapping[str, Any]) -> str:
    pairs = sc.get("near_pairs")
    if not isinstance(pairs, (list, tuple)) or not pairs:
        return ""
    closest = None
    for p in pairs:
        if not isinstance(p, Mapping) or not isinstance(p.get("clearance_m"), (int, float)):
            continue
        if closest is None or float(p["clearance_m"]) < float(closest["clearance_m"]):
            closest = p
    if closest is None:
        return ""
    return f"{_short_geom(closest.get('name_a'))} ↔ {_short_geom(closest.get('name_b'))}"


def _format_floor_constraint_status(
    latest: StateSnapshot | None,
    *,
    stale: bool,
) -> str:
    if latest is None:
        return "floor: no state"
    if stale:
        return "State stream stale"
    floor = latest.floor_constraint
    if not isinstance(floor, Mapping) or not bool(floor.get("enabled", False)):
        return "floor: disabled"

    z_min = floor.get("z_min_m")
    z_txt = f"{float(z_min) * 1000:.1f}mm" if isinstance(z_min, (int, float)) else "?"

    def arm_part(key: str) -> str:
        arm = floor.get(key)
        if not isinstance(arm, Mapping) or not bool(arm.get("checked", False)):
            return f"{key[0].upper()}:?"
        tcp_z = arm.get("tcp_z_m")
        if not isinstance(tcp_z, (int, float)) or not isinstance(z_min, (int, float)):
            return f"{key[0].upper()}:?"
        margin_mm = (float(tcp_z) - float(z_min)) * 1000.0
        return f"{key[0].upper()}:{margin_mm:.0f}mm"

    violated_arms = [
        key for key in ("left", "right")
        if isinstance(floor.get(key), Mapping) and bool(floor[key].get("violated", False))
    ]
    monitor = " monitor_only" if bool(floor.get("monitor_only", False)) else ""
    if violated_arms:
        return f"floor: VIOLATED({','.join(violated_arms)}) z={z_txt}{monitor}"
    return f"floor: ON z={z_txt} margin {arm_part('left')} {arm_part('right')}{monitor}"


def _format_user_floor_constraint_status(
    latest: StateSnapshot | None,
    *,
    stale: bool,
) -> str:
    if latest is None:
        return "user floor: no state"
    if stale:
        return "State stream stale"
    uf = latest.user_floor_constraint
    if not isinstance(uf, Mapping) or not bool(uf.get("enabled", False)):
        return "user floor: off"

    def arm_part(key: str) -> str:
        arm = uf.get(key)
        if not isinstance(arm, Mapping) or not bool(arm.get("checked", False)):
            return f"{key[0].upper()}:?"
        sd = arm.get("signed_dist_m")
        if not isinstance(sd, (int, float)):
            return f"{key[0].upper()}:?"
        return f"{key[0].upper()}:{float(sd) * 1000:.0f}mm"

    normal = uf.get("normal")
    tilt_txt = "?"
    if isinstance(normal, (list, tuple)) and len(normal) == 3:
        try:
            nz = max(-1.0, min(1.0, float(normal[2])))
            tilt_txt = f"{math.degrees(math.acos(nz)):.0f}°"
        except (TypeError, ValueError):
            tilt_txt = "?"
    violated_arms = [
        key for key in ("left", "right")
        if isinstance(uf.get(key), Mapping) and bool(uf[key].get("violated", False))
    ]
    monitor = " monitor_only" if bool(uf.get("monitor_only", False)) else ""
    if violated_arms:
        return f"user floor: VIOLATED({','.join(violated_arms)}) tilt={tilt_txt}{monitor}"
    return f"user floor: ON tilt={tilt_txt} dist {arm_part('left')} {arm_part('right')}{monitor}"


def _format_roi_box_status(
    latest: StateSnapshot | None,
    *,
    stale: bool,
) -> str:
    if latest is None:
        return "roi: no state"
    if stale:
        return "State stream stale"
    roi = latest.roi_box
    if not isinstance(roi, Mapping) or not bool(roi.get("enabled", False)):
        return "roi: disabled"

    def arm_part(key: str) -> str:
        arm = roi.get(key)
        if not isinstance(arm, Mapping) or not bool(arm.get("checked", False)):
            return f"{key[0].upper()}:?"
        margin = arm.get("min_margin_m")
        face = arm.get("closest_face")
        if not isinstance(margin, (int, float)):
            return f"{key[0].upper()}:?"
        face_txt = f"@{face}" if isinstance(face, str) else ""
        return f"{key[0].upper()}:{float(margin) * 1000:.0f}mm{face_txt}"

    violated_arms = [
        key for key in ("left", "right")
        if isinstance(roi.get(key), Mapping) and bool(roi[key].get("violated", False))
    ]
    monitor = " monitor_only" if bool(roi.get("monitor_only", False)) else ""
    if violated_arms:
        return f"roi: OUTSIDE({','.join(violated_arms)}){monitor}"
    return f"roi: ON margin {arm_part('left')} {arm_part('right')}{monitor}"


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


def _format_clearance_mm(label: str, value: Any) -> str:
    parsed = _optional_finite(value)
    return f"{label}={parsed * 1000.0:.1f}mm" if parsed is not None else f"{label}=n/a"


def _format_goal_not_clear(diag: Mapping[str, Any]) -> str:
    name_a = str(diag.get("goal_nearest_pair_name_a", "") or "")
    name_b = str(diag.get("goal_nearest_pair_name_b", "") or "")
    clearance = _optional_finite(
        diag.get("goal_nearest_pair_distance_m", diag.get("nearest_pair_distance_m"))
    )
    threshold = _optional_finite(diag.get("goal_clear_threshold_external_m")) if bool(
        diag.get("goal_nearest_pair_external", False)
    ) else _optional_finite(diag.get("goal_clear_threshold_self_m"))
    if name_a or name_b:
        pair = f"{name_a or 'unknown'} <-> {name_b or 'unknown'}"
        if clearance is not None and threshold is not None:
            return (
                "InitMotion FAILED: goal_not_clear: "
                f"pair {pair}, clearance {clearance * 1000.0:.1f} mm "
                f"< required {threshold * 1000.0:.1f} mm"
            )
        return f"InitMotion FAILED: goal_not_clear: pair {pair}"
    return (
        "InitMotion FAILED: Init 자세가 충돌/바닥 침범 "
        f"({_format_clearance_mm('goal_clear', diag.get('goal_clear_m'))}) - Init pose 재설정 필요"
    )


def _format_init_motion_status(latest: StateSnapshot | None, *, stale: bool) -> str:
    if latest is None:
        return "InitMotion: no state"
    if stale:
        return "InitMotion: State stream stale"
    diag = latest.init_motion
    if not isinstance(diag, Mapping):
        return "InitMotion: no runtime diagnostics"
    status = str(diag.get("status", "idle"))
    fail_mode = str(diag.get("fail_mode", "none"))
    message = str(diag.get("message", "") or "")
    wp_index = diag.get("waypoint_index")
    wp_count = diag.get("waypoint_count")
    iterations = diag.get("iterations")
    tree_start = diag.get("tree_start")
    tree_goal = diag.get("tree_goal")
    planning_time = _optional_finite(diag.get("planning_time_s"))
    dist_to_goal = _optional_finite(diag.get("dist_to_goal_deg"))

    if status == "failed":
        if fail_mode == "goal_not_clear":
            return _format_goal_not_clear(diag)
        if fail_mode == "escape_failed":
            return (
                "InitMotion FAILED: 시작 자세가 끼임 - 탈출 실패 "
                f"({_format_clearance_mm('start_clear', diag.get('start_clear_m'))})"
            )
        if fail_mode == "rrt_budget":
            time_part = f", {planning_time:.2f}s" if planning_time is not None else ""
            return (
                "InitMotion FAILED: 경로 탐색 시간 초과 "
                f"(tree {tree_start}/{tree_goal}, iters {iterations}{time_part}) - 샘플 밴드/예산 확대 필요"
            )
        if fail_mode in {"exec_timeout", "exec_stalled"}:
            dist_part = f", dist {dist_to_goal:.2f}deg" if dist_to_goal is not None else ""
            return f"InitMotion FAILED: 실행 중단 (wp {wp_index}/{wp_count}{dist_part})"
        if fail_mode == "nonfinite":
            return "InitMotion FAILED: non-finite start/goal"
        return "InitMotion FAILED: " + (message or fail_mode)
    if status == "planning":
        return "InitMotion planning"
    if status == "executing":
        dist_part = f", dist {dist_to_goal:.2f}deg" if dist_to_goal is not None else ""
        return f"InitMotion executing (wp {wp_index}/{wp_count}{dist_part})"
    if status == "done":
        return "InitMotion done"
    return "InitMotion idle"


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


def _reflect_gate_reason(handle: Any, reason: str | None, *, idle_text: str = "idle") -> None:
    """Proactively surface a control's block reason in a status text handle.

    viser button_groups (jog/velocity/twist nudge rows) cannot be greyed, so the
    operator otherwise only learns a control is blocked after a rejected click.
    This writes "DISABLED: <reason>" while blocked and clears it back to idle when
    the gate opens — but never overwrites a fresh "OK:/BLOCKED:" click result, so
    live command feedback is preserved."""
    if handle is None:
        return
    if reason:
        handle.value = "DISABLED: " + reason
    elif str(getattr(handle, "value", "")).startswith("DISABLED:"):
        handle.value = idle_text


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


def _eft_monitor_values(arm_state: Any, *, stale: bool) -> tuple[str, str, str]:
    """(force, torque, |F|) display strings for one arm's external F/T sensor
    (rbpodo sdata.eft_*, sensor frame, N / Nm). 'invalid' when the feed is
    stale, the server flags it invalid, or the fields are absent (mock)."""
    eft = getattr(arm_state, "eft_wrench", None) if arm_state is not None else None
    valid = bool(getattr(arm_state, "eft_valid", False)) and eft is not None and not stale
    if not valid:
        return ("invalid", "invalid", "invalid")
    force = " ".join(f"{v:+.1f}" for v in eft[:3])
    torque = " ".join(f"{v:+.2f}" for v in eft[3:])
    magnitude = f"{math.sqrt(eft[0] ** 2 + eft[1] ** 2 + eft[2] ** 2):.1f}"
    return (force, torque, magnitude)


def _eft_monitor_axis_values(arm_state: Any, *, stale: bool) -> tuple[str, str, str, str, str, str, str]:
    """(fx, fy, fz, |F|, tx, ty, tz) display strings for one arm's external F/T
    sensor, same validity gate as `_eft_monitor_values`. One number per cell so a
    narrow monitor card renders each axis on its own row without a horizontal
    scrollbar. 'invalid' per cell when the feed is stale/invalid/absent."""
    eft = getattr(arm_state, "eft_wrench", None) if arm_state is not None else None
    valid = bool(getattr(arm_state, "eft_valid", False)) and eft is not None and not stale
    if not valid:
        return ("invalid",) * 7
    fx, fy, fz = (f"{v:+.1f}" for v in eft[:3])
    tx, ty, tz = (f"{v:+.2f}" for v in eft[3:])
    magnitude = f"{math.sqrt(eft[0] ** 2 + eft[1] ** 2 + eft[2] ** 2):.1f}"
    return (fx, fy, fz, magnitude, tx, ty, tz)


def _update_eft_monitor_handles(handles: dict[str, Any], arm: str, arm_state: Any, *, stale: bool) -> None:
    eft_handles = handles.get("eft_monitor_values", {}).get(arm, {})
    if not eft_handles:
        return
    force, torque, magnitude = _eft_monitor_values(arm_state, stale=stale)
    for field, value in (("force", force), ("torque", torque), ("magnitude", magnitude)):
        handle = eft_handles.get(field)
        if handle is not None:
            handle.value = value


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
            _update_eft_monitor_handles(handles, arm, None, stale=True)
        return
    state = "stale" if stale else "live"
    handles["stand_world_monitor_status"].value = f"{state}, xyz=mm, rpy={unit}, tick={latest.tick}"
    if stale:
        for arm in ("left", "right"):
            for handle in value_handles.get(arm, {}).values():
                handle.value = "invalid"
            _update_eft_monitor_handles(handles, arm, None, stale=True)
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
        _update_eft_monitor_handles(handles, arm, arm_state, stale=stale)
