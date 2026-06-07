from __future__ import annotations

import argparse
import math
import os
import threading
import time
from html import escape
from pathlib import Path
from typing import Any, Mapping

from .command_client import CommandClient
from .geometry import (
    _add_matrix3,
    _add_vec3,
    _angular_step_radians,
    _delta_transform,
    _identity3,
    _linear_step_meters,
    _matmul3,
    _matrix_to_wxyz,
    _mount_position,
    _mount_pose_from_mounts,
    _multiply_transform,
    _normalize_wxyz,
    _pose6_from_mounts,
    _pose6_from_state_arm,
    _pose6_from_transform,
    _pose_orientation_wxyz,
    _pose_position,
    _pose_transform,
    _pose_wxyz,
    _quat_to_matrix,
    _rotate_vec,
    _rotation_vector_from_matrix,
    _rpy_to_wxyz,
    _scale_matrix3,
    _se3_log_translation,
    _skew3,
    _tcp_local_delta_from_target,
    _tcp_pose_from_urdf,
    _transform_to_pose6,
    _transpose3,
    _wxyz_to_rpy,
    _wxyz_to_xyzw,
    _xyzw_to_wxyz,
)
from .models import ArmSnapshot, CircleOverlaySnapshot, Pose6D, StateSnapshot
from .overlay_receiver import CircleOverlayReceiver, CircleOverlayStore, parse_udp_bind
from .safety import OperatorSafety, Readiness, normalize_observed_mode_backend
from .scene import (
    _DEFAULT_LEFT_POSE,
    _DEFAULT_RIGHT_POSE,
    _DEFAULT_STAND_MESH_POSE,
    _ROBOT_JOINT_NAMES,
    _add_robot_urdfs,
    _add_scene_fallback,
    _add_stand_mesh,
    _asset_path,
    _descriptions_dir,
    format_scene_asset_startup_status,
    _install_tcp_target_callbacks,
    _joint_cfg_radians,
    _joint_marker_position,
    _repo_descriptions_dir,
    _robot_urdf_path,
    _stand_mesh_path,
    _update_urdf_config,
    update_circle_overlay,
    update_scene_markers,
)
from .state_receiver import StateReceiver, StateStore
from .status_panel import (
    _JOINT_MONITOR_UNITS,
    _STAND_WORLD_MONITOR_UNITS,
    _STAND_WORLD_POSE_FIELDS,
    _TCP_DISPLAY_MODES,
    _arm_fk_status,
    _format_arm_cartesian_solve,
    _format_cartesian_solve_status,
    _format_circle_overlay_status,
    _format_scene_asset_status,
    _format_fk_status,
    _format_joint_monitor_value,
    _format_joints,
    _format_pgmode_status,
    _format_stand_world_pose_value,
    _format_tcp_command_status,
    _format_tcp_tracking_status,
    _joint_monitor_unit,
    _mode_button_color,
    _optional_finite,
    _set_disabled,
    _stand_world_monitor_unit,
    _tcp_display_mode,
    _update_joint_monitor,
    _update_joint_monitor_unit_buttons,
    _update_stand_world_monitor,
    _update_stand_world_monitor_unit_buttons,
    _update_tcp_display_buttons,
)

# mock is intentionally not operator-selectable: the GUI targets simulation
# (incl. rbpodo pgmode controller-simulation) and real (status-only).
_DESIRED_MODES = ("simulation", "real")
_TCP_FRAME_STAND = "Stand/world"
_TCP_FRAME_LOCAL = "TCP local"
_TCP_FRAME_OPTIONS = (_TCP_FRAME_STAND, _TCP_FRAME_LOCAL)
_TCP_LINEAR_ARM_OPTIONS = ("left", "right", "both")
_TCP_LINEAR_ORIENTATION_MODES = ("constant", "slerp")
_DEFAULT_INIT_LEFT_JOINTS_DEG = (-124.660, 32.485, 119.074, -96.294, -81.798, -30.615)
_DEFAULT_INIT_RIGHT_JOINTS_DEG = (111.949, -49.304, -120.057, 75.305, 87.436, 49.983)
_OPERATOR_MONITOR_WIDTH_EM = 18.0
_OPERATOR_MONITOR_GAP_EM = 1.0


def _env_int(name: str, fallback: int) -> int:
    try:
        return int(os.environ.get(name, str(fallback)))
    except ValueError:
        return fallback


def _env_bool(name: str, fallback: bool = False) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return fallback
    return raw.strip().lower() in {"1", "true", "yes", "on"}


def _env_optional_bool(name: str) -> bool | None:
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return None
    return raw.strip().lower() in {"1", "true", "yes", "on"}


def _env_float(name: str, fallback: float) -> float:
    try:
        value = float(os.environ.get(name, str(fallback)))
    except ValueError:
        return fallback
    return value if math.isfinite(value) else fallback


def _env_positive_float(name: str, fallback: float) -> float:
    value = _env_float(name, fallback)
    return value if value > 0.0 else fallback


def _env_joint6(name: str, fallback: tuple[float, ...]) -> tuple[float, ...] | None:
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return fallback
    try:
        values = tuple(float(part.strip()) for part in raw.split(","))
    except ValueError:
        return None
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        return None
    return values


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RB servo browser GUI.")
    parser.add_argument(
        "--circle-overlay-bind",
        default=None,
        help=(
            "UDP endpoint for circle benchmark overlay packets, for example "
            "udp://0.0.0.0:50261. Use 'none' or an empty value to disable. "
            "Defaults to RB_GUI_CIRCLE_OVERLAY_BIND or disabled."
        ),
    )
    parser.add_argument(
        "--check-assets",
        action="store_true",
        help="Print rb_gui URDF, mesh, and visualization dependency diagnostics, then exit.",
    )
    return parser.parse_args(argv)


def _circle_overlay_bind_from_args_env(args: argparse.Namespace) -> tuple[str, int] | None:
    endpoint = args.circle_overlay_bind
    if endpoint is None:
        endpoint = os.environ.get("RB_GUI_CIRCLE_OVERLAY_BIND", "none")
    return parse_udp_bind(endpoint)


def _sim_readiness_from_env(observed: Any) -> Readiness:
    ready = _env_bool("RB_GUI_SIM_READINESS_READY", False)
    running = _env_bool("RB_GUI_SIM_READINESS_RUNNING", ready)
    connected = _env_bool("RB_GUI_SIM_READINESS_CONNECTED", ready)
    no_go_reason = os.environ.get("RB_GUI_SIM_READINESS_NO_GO", "simulator/rbpodo readiness tests have not passed")
    if ready:
        no_go_reason = ""
    # The local simulator backend is always a valid sim stack. An rbpodo
    # controller in pgmode simulation also counts as a sim stack, but only when
    # the operator opts in (RB_GUI_ENABLE_CONTROLLER_SIM_CARTESIAN=1); real mode
    # never qualifies.
    controller_sim_optin = _env_bool("RB_GUI_ENABLE_CONTROLLER_SIM_CARTESIAN", False)
    valid_sim_stack = observed.mode == "simulation" and (
        observed.backend == "simulator"
        or (observed.backend == "rbpodo" and controller_sim_optin)
    )
    if not valid_sim_stack:
        ready = False
        running = False
        connected = False
        no_go_reason = "observed server is not the simulator stack"
    return Readiness(
        running=running,
        connected=connected,
        ready=ready,
        no_go_reason=no_go_reason,
        cartesian_available=_env_optional_bool("RB_GUI_CARTESIAN_AVAILABLE"),
        cartesian_no_go_reason=os.environ.get("RB_GUI_CARTESIAN_NO_GO", "server Cartesian/IK readiness not proven"),
    )


def _update_desired_mode_buttons(handles: dict[str, Any], desired_mode: str) -> None:
    for mode, button in handles.get("mode_buttons", {}).items():
        try:
            button.color = _mode_button_color(mode, desired_mode)
        except Exception:
            pass


def _tcp_frame_mode(handles: dict[str, Any]) -> str:
    mode = handles.get("tcp_frame_mode", _TCP_FRAME_LOCAL)
    return mode if mode in _TCP_FRAME_OPTIONS else _TCP_FRAME_LOCAL


def _update_tcp_frame_buttons(handles: dict[str, Any]) -> None:
    selected = _tcp_frame_mode(handles)
    for mode, button in handles.get("tcp_frame_buttons", {}).items():
        try:
            button.color = _mode_button_color(mode, selected)
        except Exception:
            pass


def _tcp_linear_arm(handles: dict[str, Any]) -> str:
    selected = handles.get("tcp_linear_arm", "both")
    return selected if selected in _TCP_LINEAR_ARM_OPTIONS else "both"


def _tcp_linear_orientation_mode(handles: dict[str, Any]) -> str:
    selected = handles.get("tcp_linear_orientation_mode", "slerp")
    return selected if selected in _TCP_LINEAR_ORIENTATION_MODES else "slerp"


def _update_tcp_linear_selection_buttons(handles: dict[str, Any]) -> None:
    selected_arm = _tcp_linear_arm(handles)
    for arm, button in handles.get("tcp_linear_arm_buttons", {}).items():
        try:
            button.color = _mode_button_color(arm, selected_arm)
        except Exception:
            pass
    selected_mode = _tcp_linear_orientation_mode(handles)
    for mode, button in handles.get("tcp_linear_orientation_buttons", {}).items():
        try:
            button.color = _mode_button_color(mode, selected_mode)
        except Exception:
            pass


def _tcp_target_pose(scene_handles: dict[str, Any], arm: str) -> tuple[float, float, float, float, float, float] | None:
    pose = scene_handles.get(f"{arm}_tcp_target_pose")
    if pose is None:
        return None
    try:
        values = tuple(float(value) for value in pose)
    except Exception:
        return None
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        return None
    return values  # type: ignore[return-value]


def _tcp_target_wxyz(scene_handles: dict[str, Any], arm: str) -> tuple[float, float, float, float] | None:
    wxyz = scene_handles.get(f"{arm}_tcp_target_wxyz")
    try:
        if wxyz is not None:
            values = tuple(float(value) for value in wxyz)
            if len(values) == 4 and all(math.isfinite(value) for value in values):
                return _normalize_wxyz(values)  # type: ignore[arg-type]
    except Exception:
        pass
    pose = _tcp_target_pose(scene_handles, arm)
    if pose is None:
        return None
    return _pose_orientation_wxyz(pose)


def _tcp_target_position_wxyz(
    scene_handles: dict[str, Any],
    arm: str,
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]] | None:
    pose = _tcp_target_pose(scene_handles, arm)
    wxyz = _tcp_target_wxyz(scene_handles, arm)
    if pose is None or wxyz is None:
        return None
    return (pose[0], pose[1], pose[2]), wxyz


def _apply_tcp_delta_to_target(
    scene_handles: dict[str, Any],
    arm: str,
    delta: tuple[float, float, float, float, float, float],
    frame_mode: str,
) -> bool:
    target = _tcp_target_position_wxyz(scene_handles, arm)
    if target is None:
        return False
    position, wxyz = target
    target_transform = _pose_transform(position, wxyz)
    delta_transform = _delta_transform(delta)
    if frame_mode == _TCP_FRAME_LOCAL:
        next_transform = _multiply_transform(target_transform, delta_transform)
    else:
        next_transform = _multiply_transform(delta_transform, target_transform)
    pose, next_wxyz = _transform_to_pose6(next_transform)
    scene_handles[f"{arm}_tcp_target_pose"] = pose
    scene_handles[f"{arm}_tcp_target_wxyz"] = next_wxyz
    scene_handles[f"{arm}_tcp_target_user_moved"] = True
    handle = scene_handles.get(f"{arm}_tcp_target")
    if handle is not None:
        try:
            handle.position = pose[:3]
        except Exception:
            pass
        try:
            handle.wxyz = next_wxyz
        except Exception:
            pass
    return True


def _clear_tcp_target_user_moved(scene_handles: dict[str, Any], arms: tuple[str, ...] = ("left", "right")) -> None:
    for arm in arms:
        scene_handles.pop(f"{arm}_tcp_target_user_moved", None)


def _send_init_motion_and_reset_targets(
    safety: OperatorSafety,
    scene_handles: dict[str, Any],
) -> tuple[bool, str]:
    ok, message = safety.send_init_motion()
    if ok:
        _clear_tcp_target_user_moved(scene_handles)
        message = f"{message}; TCP targets will follow current TCP"
    return ok, message


def _send_tcp_pose_target_from_marker(
    safety: OperatorSafety,
    scene_handles: dict[str, Any],
    arm: str,
) -> tuple[bool, str]:
    pose = _tcp_target_pose(scene_handles, arm)
    if pose is None:
        return False, f"{arm} TCP target unavailable"
    wxyz = _tcp_target_wxyz(scene_handles, arm)
    quaternion_xyzw = _wxyz_to_xyzw(wxyz) if wxyz is not None else None
    return safety.send_tcp_pose_target(
        left_pose=pose if arm == "left" else None,
        right_pose=pose if arm == "right" else None,
        left_quaternion_xyzw=quaternion_xyzw if arm == "left" else None,
        right_quaternion_xyzw=quaternion_xyzw if arm == "right" else None,
    )


def _tcp_marker_payloads(
    scene_handles: dict[str, Any],
    arm_group: str,
) -> tuple[
    tuple[float, float, float, float, float, float] | None,
    tuple[float, float, float, float, float, float] | None,
    tuple[float, float, float, float] | None,
    tuple[float, float, float, float] | None,
    str | None,
]:
    selected_arms = ("left", "right") if arm_group == "both" else (arm_group,)
    left_pose = right_pose = None
    left_quaternion_xyzw = right_quaternion_xyzw = None
    for arm in selected_arms:
        if arm not in {"left", "right"}:
            return None, None, None, None, f"unsupported TCP arm group {arm_group}"
        pose = _tcp_target_pose(scene_handles, arm)
        if pose is None:
            return None, None, None, None, f"{arm} TCP target unavailable"
        wxyz = _tcp_target_wxyz(scene_handles, arm)
        quaternion_xyzw = _wxyz_to_xyzw(wxyz) if wxyz is not None else None
        if arm == "left":
            left_pose = pose
            left_quaternion_xyzw = quaternion_xyzw
        else:
            right_pose = pose
            right_quaternion_xyzw = quaternion_xyzw
    return left_pose, right_pose, left_quaternion_xyzw, right_quaternion_xyzw, None


def _send_tcp_linear_move_from_marker(
    safety: OperatorSafety,
    scene_handles: dict[str, Any],
    arm_group: str,
    *,
    duration_sec: float | None,
    linear_speed_m_s: float | None,
    angular_speed_rad_s: float | None,
    orientation_mode: str,
) -> tuple[bool, str]:
    left_pose, right_pose, left_quaternion_xyzw, right_quaternion_xyzw, error = _tcp_marker_payloads(scene_handles, arm_group)
    if error:
        return False, error
    return safety.send_tcp_linear_move(
        left_pose=left_pose,
        right_pose=right_pose,
        left_quaternion_xyzw=left_quaternion_xyzw,
        right_quaternion_xyzw=right_quaternion_xyzw,
        duration_sec=duration_sec,
        linear_speed_m_s=linear_speed_m_s,
        angular_speed_rad_s=angular_speed_rad_s,
        orientation_mode=orientation_mode,
    )


def _apply_tcp_delta_and_send_pose_target(
    safety: OperatorSafety,
    scene_handles: dict[str, Any],
    arm: str,
    delta: tuple[float, float, float, float, float, float],
    frame_mode: str,
) -> tuple[bool, str]:
    if not _apply_tcp_delta_to_target(scene_handles, arm, delta, frame_mode):
        return False, f"{arm} TCP target unavailable"
    return _send_tcp_pose_target_from_marker(safety, scene_handles, arm)


def _build_joint_monitor(server: Any, handles: dict[str, Any], *, order: float | None = None) -> None:
    with server.gui.add_folder("Joint Monitor", expand_by_default=True, order=order):
        handles["joint_monitor_unit"] = "deg"
        handles["joint_monitor_unit_buttons"] = {}
        for unit in _JOINT_MONITOR_UNITS:
            unit_button = server.gui.add_button(unit, color=_mode_button_color(unit, _joint_monitor_unit(handles)))
            handles["joint_monitor_unit_buttons"][unit] = unit_button

            @unit_button.on_click
            def _(_: Any, unit: str = unit) -> None:
                handles["joint_monitor_unit"] = unit
                _update_joint_monitor_unit_buttons(handles)

        handles["joint_monitor_status"] = server.gui.add_text("Status", initial_value="No state stream", disabled=True)
        handles["joint_monitor_values"] = {"left": [], "right": []}
        for arm in ("left", "right"):
            with server.gui.add_folder(arm, expand_by_default=True):
                for index, joint_name in enumerate(_ROBOT_JOINT_NAMES):
                    handle = server.gui.add_text(f"{arm} J{index + 1} {joint_name}", initial_value="invalid", disabled=True)
                    handles["joint_monitor_values"][arm].append(handle)


def _build_stand_world_monitor(server: Any, handles: dict[str, Any], *, order: float | None = None) -> None:
    with server.gui.add_folder("Stand/World Monitor", expand_by_default=True, order=order):
        handles["stand_world_monitor_unit"] = "deg"
        handles["stand_world_monitor_unit_buttons"] = {}
        for unit in _STAND_WORLD_MONITOR_UNITS:
            unit_button = server.gui.add_button(unit, color=_mode_button_color(unit, _stand_world_monitor_unit(handles)))
            handles["stand_world_monitor_unit_buttons"][unit] = unit_button

            @unit_button.on_click
            def _(_: Any, unit: str = unit) -> None:
                handles["stand_world_monitor_unit"] = unit
                _update_stand_world_monitor_unit_buttons(handles)

        handles["stand_world_monitor_status"] = server.gui.add_text(
            "Status",
            initial_value="No state stream",
            disabled=True,
        )
        handles["stand_world_monitor_values"] = {"left": {}, "right": {}}
        for arm in ("left", "right"):
            with server.gui.add_folder(arm, expand_by_default=True):
                for field in _STAND_WORLD_POSE_FIELDS:
                    handle = server.gui.add_text(f"{arm} {field}", initial_value="invalid", disabled=True)
                    handles["stand_world_monitor_values"][arm][field] = handle


def _operator_monitor_layout() -> tuple[float, float]:
    return (
        _env_positive_float("RB_GUI_MONITOR_WIDTH_EM", _OPERATOR_MONITOR_WIDTH_EM),
        _env_positive_float("RB_GUI_MONITOR_GAP_EM", _OPERATOR_MONITOR_GAP_EM),
    )


def _operator_monitor_static_html(monitor_width_em: float, gap_em: float) -> str:
    return f"""
<style>
  :root {{
    --rb-monitor-gap: {gap_em:.3f}em;
    --rb-monitor-target-width: {monitor_width_em:.3f}em;
    --rb-monitor-width: min(
      var(--rb-monitor-target-width),
      max(13.5em, calc((100vw - (3 * var(--rb-monitor-gap))) / 2))
    );
  }}
  .rb-monitor-card {{
    position: fixed;
    width: var(--rb-monitor-width);
    box-sizing: border-box;
    z-index: 19;
    background: rgba(255, 255, 255, 0.96);
    color: #1f2933;
    border: 1px solid rgba(15, 23, 42, 0.18);
    box-shadow: 0 0.35em 1.2em rgba(15, 23, 42, 0.16);
    font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
    font-size: 12px;
  }}
  .rb-monitor-header-card {{
    top: 1em;
    min-height: 4.45em;
    padding: 0.65em 0.8em 0.55em;
    border-radius: 0.45em 0.45em 0 0;
    border-bottom: 0;
  }}
  .rb-monitor-body-card {{
    top: 5.45em;
    max-height: calc(100vh - 6.45em);
    overflow: auto;
    padding: 0.6em 0.8em 0.75em;
    border-radius: 0 0 0.45em 0.45em;
  }}
  .rb-monitor-joint-card {{ left: var(--rb-monitor-gap); }}
  .rb-monitor-stand-card {{ left: calc((2 * var(--rb-monitor-gap)) + var(--rb-monitor-width)); }}
  .rb-monitor-title {{
    font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    font-weight: 650;
    font-size: 13px;
    margin-bottom: 0.45em;
  }}
  .rb-monitor-units {{
    display: flex;
    gap: 0.45em;
    align-items: center;
    font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    font-size: 12px;
  }}
  .rb-monitor-units label {{
    display: inline-flex;
    gap: 0.2em;
    align-items: center;
    cursor: pointer;
  }}
  .rb-monitor-status {{
    font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    font-size: 11px;
    color: #52606d;
    margin-bottom: 0.6em;
  }}
  .rb-monitor-arm {{
    margin-top: 0.45em;
    padding-top: 0.45em;
    border-top: 1px solid rgba(15, 23, 42, 0.1);
  }}
  .rb-monitor-arm:first-of-type {{
    border-top: 0;
    margin-top: 0;
    padding-top: 0;
  }}
  .rb-monitor-arm-title {{
    font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    font-weight: 600;
    margin-bottom: 0.25em;
  }}
  .rb-monitor-row {{
    display: grid;
    grid-template-columns: minmax(7.5em, 1fr) auto;
    column-gap: 0.8em;
    line-height: 1.55;
    white-space: nowrap;
  }}
  .rb-rad {{ display: none; }}
  body:has(#rb-joint-unit-rad:checked) .rb-monitor-joint-card .rb-deg {{ display: none; }}
  body:has(#rb-joint-unit-rad:checked) .rb-monitor-joint-card .rb-rad {{ display: inline; }}
  body:has(#rb-stand-unit-rad:checked) .rb-monitor-stand-card .rb-deg {{ display: none; }}
  body:has(#rb-stand-unit-rad:checked) .rb-monitor-stand-card .rb-rad {{ display: inline; }}
  @media (max-width: 960px) {{
    .rb-monitor-card {{ font-size: 11px; }}
    .rb-monitor-header-card {{
      min-height: 4.0em;
      padding: 0.55em 0.65em 0.45em;
    }}
    .rb-monitor-body-card {{
      top: 5.0em;
      max-height: calc(100vh - 6.0em);
      padding: 0.5em 0.65em 0.65em;
    }}
    .rb-monitor-title {{ font-size: 12px; }}
    .rb-monitor-row {{
      grid-template-columns: minmax(5.6em, 1fr) auto;
      column-gap: 0.45em;
    }}
  }}
</style>
<div class="rb-monitor-card rb-monitor-header-card rb-monitor-joint-card">
  <div class="rb-monitor-title">Joint Monitor</div>
  <div class="rb-monitor-units">
    <label><input id="rb-joint-unit-deg" name="rb-joint-unit" type="radio" checked> deg</label>
    <label><input id="rb-joint-unit-rad" name="rb-joint-unit" type="radio"> rad</label>
  </div>
</div>
<div class="rb-monitor-card rb-monitor-header-card rb-monitor-stand-card">
  <div class="rb-monitor-title">Pose Monitor</div>
  <div class="rb-monitor-units">
    <label><input id="rb-stand-unit-deg" name="rb-stand-unit" type="radio" checked> deg</label>
    <label><input id="rb-stand-unit-rad" name="rb-stand-unit" type="radio"> rad</label>
  </div>
</div>
"""


def _operator_monitor_value_pair(deg_value: str, rad_value: str) -> str:
    return f'<span class="rb-deg">{escape(deg_value)}</span><span class="rb-rad">{escape(rad_value)}</span>'


def _operator_monitor_invalid_pair() -> str:
    return _operator_monitor_value_pair("invalid", "invalid")


def _operator_monitor_row(label: str, value_html: str) -> str:
    return f'<div class="rb-monitor-row"><span>{escape(label)}</span><span>{value_html}</span></div>'


def _render_joint_monitor_rows(latest: StateSnapshot | None, *, stale: bool) -> str:
    if latest is None:
        status = "No state stream"
        arms = (("left", None), ("right", None))
    else:
        status = f"{'stale' if stale else 'live'}, tick={latest.tick}"
        arms = (("left", latest.left), ("right", latest.right))
    parts = [f'<div class="rb-monitor-status">{escape(status)}</div>']
    for arm, arm_state in arms:
        parts.append(f'<div class="rb-monitor-arm"><div class="rb-monitor-arm-title">{escape(arm)}</div>')
        for index, joint_name in enumerate(_ROBOT_JOINT_NAMES):
            if arm_state is None:
                value_html = _operator_monitor_invalid_pair()
            else:
                value_html = _operator_monitor_value_pair(
                    _format_joint_monitor_value(
                        arm_state.q_actual_deg,
                        index,
                        valid=arm_state.has_valid_joint_state,
                        unit="deg",
                    ),
                    _format_joint_monitor_value(
                        arm_state.q_actual_deg,
                        index,
                        valid=arm_state.has_valid_joint_state,
                        unit="rad",
                    ),
                )
            parts.append(_operator_monitor_row(f"J{index + 1} {joint_name}", value_html))
        parts.append("</div>")
    return "".join(parts)


def _render_stand_world_monitor_rows(latest: StateSnapshot | None, *, stale: bool) -> str:
    if latest is None:
        status = "No state stream, xyz=mm"
        arms = (("left", None), ("right", None))
    else:
        status = f"{'stale' if stale else 'live'}, xyz=mm, tick={latest.tick}"
        arms = (("left", latest.left), ("right", latest.right))
    parts = [f'<div class="rb-monitor-status">{escape(status)}</div>']
    for arm, arm_state in arms:
        parts.append(f'<div class="rb-monitor-arm"><div class="rb-monitor-arm-title">{escape(arm)}</div>')
        valid = bool(
            arm_state is not None
            and not stale
            and arm_state.has_valid_tcp_pose
            and arm_state.tcp_stand is not None
            and not arm_state.tcp_deferred
        )
        pose = arm_state.tcp_stand if arm_state is not None else None
        for field in _STAND_WORLD_POSE_FIELDS:
            if field in ("x", "y", "z"):
                value_html = escape(_format_stand_world_pose_value(pose, field, valid=valid, unit="deg"))
            else:
                value_html = _operator_monitor_value_pair(
                    _format_stand_world_pose_value(pose, field, valid=valid, unit="deg"),
                    _format_stand_world_pose_value(pose, field, valid=valid, unit="rad"),
                )
            parts.append(_operator_monitor_row(field, value_html))
        parts.append("</div>")
    return "".join(parts)


def _operator_monitor_dynamic_html(latest: StateSnapshot | None, *, stale: bool) -> str:
    return (
        '<div class="rb-monitor-card rb-monitor-body-card rb-monitor-joint-card">'
        + _render_joint_monitor_rows(latest, stale=stale)
        + "</div>"
        + '<div class="rb-monitor-card rb-monitor-body-card rb-monitor-stand-card">'
        + _render_stand_world_monitor_rows(latest, stale=stale)
        + "</div>"
    )


def _build_operator_monitors(server: Any, handles: dict[str, Any]) -> None:
    add_html = getattr(server.gui, "add_html", None)
    if callable(add_html):
        monitor_width_em, gap_em = _operator_monitor_layout()
        handles["operator_monitor_panel_mode"] = "fixed_html_overlay"
        handles["operator_monitor_style"] = add_html(
            _operator_monitor_static_html(monitor_width_em, gap_em),
            order=0.0,
        )
        handles["operator_monitor_content"] = add_html(_operator_monitor_dynamic_html(None, stale=True), order=0.1)
        return
    handles["operator_monitor_panel_mode"] = "root_gui_fallback"
    with server.gui.add_folder("Operator Monitors", expand_by_default=True, order=0.0):
        _build_joint_monitor(server, handles, order=0.0)
        _build_stand_world_monitor(server, handles, order=0.1)


def _update_operator_monitors(handles: dict[str, Any], latest: StateSnapshot | None, *, stale: bool) -> None:
    content = handles.get("operator_monitor_content")
    if content is not None:
        try:
            content.content = _operator_monitor_dynamic_html(latest, stale=stale)
            return
        except Exception:
            pass
    _update_joint_monitor(handles, latest, stale=stale)
    _update_stand_world_monitor(handles, latest, stale=stale)


def build_gui(
    server: Any,
    safety: OperatorSafety,
    store: StateStore,
    overlay_store: CircleOverlayStore | None = None,
) -> dict[str, Any]:
    handles: dict[str, Any] = {}
    handles["circle_overlay_enabled"] = overlay_store is not None
    handles["scene"] = _add_scene_fallback(server)

    _build_operator_monitors(server, handles)

    tabs = server.gui.add_tab_group(order=1.0)
    with tabs.add_tab("Status"):
        handles["connection"] = server.gui.add_text("Connection", initial_value="disconnected", disabled=True)
        handles["mode"] = server.gui.add_text("Observed mode/backend", initial_value=f"{safety.observed_server_mode}/{safety.observed_backend}", disabled=True)
        handles["readiness"] = server.gui.add_text("Readiness", initial_value="No-Go: no state", disabled=True)
        handles["motion"] = server.gui.add_text("Motion state", initial_value="unknown", disabled=True)
        handles["fault"] = server.gui.add_text("Fault", initial_value="none", disabled=True)
        handles["fk_status"] = server.gui.add_text("FK/TCP", initial_value="FK: no state", disabled=True)
        handles["tcp_tracking"] = server.gui.add_text("TCP tracking", initial_value="TCP tracking: no state", disabled=True)
        handles["pgmode_status"] = server.gui.add_text("pgmode simulation", initial_value="pgmode_sim: no state", disabled=True)
        handles["circle_overlay"] = server.gui.add_text(
            "Circle overlay",
            initial_value=_format_circle_overlay_status(None, stale=True, enabled=overlay_store is not None),
            disabled=True,
        )
        handles["cartesian_solve"] = server.gui.add_text("IK solve", initial_value="IK: no state", disabled=True)
        handles["tcp_display_mode"] = "auto"
        handles["tcp_display_buttons"] = {}
        for display_mode in _TCP_DISPLAY_MODES:
            display_button = server.gui.add_button(
                f"TCP display: {display_mode}",
                color=_mode_button_color(display_mode, _tcp_display_mode(handles)),
            )
            handles["tcp_display_buttons"][display_mode] = display_button

            @display_button.on_click
            def _(_: Any, display_mode: str = display_mode) -> None:
                handles["tcp_display_mode"] = display_mode
                _update_tcp_display_buttons(handles)
                handles["tcp_tracking"].value = f"TCP display: {display_mode}"

        handles["ops"] = server.gui.add_text(
            "Container ops",
            initial_value="manual compose commands only; no Docker socket in GUI",
            disabled=True,
        )
        handles["mode_buttons"] = {}
        for mode in _DESIRED_MODES:
            mode_button = server.gui.add_button(mode, color=_mode_button_color(mode, safety.desired_mode))
            handles["mode_buttons"][mode] = mode_button

            @mode_button.on_click
            def _(_: Any, mode: str = mode) -> None:
                safety.set_desired_mode(mode)
                _update_desired_mode_buttons(handles, safety.desired_mode)

    with tabs.add_tab("Lifecycle"):
        handles["lifecycle_buttons"] = {}
        for mode in ("ArmMotion", "DisarmMotion", "Hold", "EmergencyStop", "ResetFault"):
            button = server.gui.add_button(mode)
            handles[f"button_{mode}"] = button
            handles["lifecycle_buttons"][mode] = button

            @button.on_click
            def _(_: Any, mode: str = mode) -> None:
                ok, message = safety.send_lifecycle(mode)
                handles["last_action"].value = ("OK: " if ok else "BLOCKED: ") + message

        init_button = server.gui.add_button("InitMotion")
        handles["init_motion_button"] = init_button

        @init_button.on_click
        def _(_: Any) -> None:
            ok, message = _send_init_motion_and_reset_targets(safety, handles["scene"])
            handles["last_action"].value = ("OK: " if ok else "BLOCKED: ") + message

        handles["last_action"] = server.gui.add_text("Last action", initial_value="none", disabled=True)

    with tabs.add_tab("Joint jog"):
        arm_group = server.gui.add_button_group("Arm", ("left", "right"))
        joint_slider = server.gui.add_slider("Joint index", min=1, max=6, step=1, initial_value=1)
        delta_slider = server.gui.add_slider("Step deg", min=-2.0, max=2.0, step=0.1, initial_value=0.5)
        jog_button = server.gui.add_button("Send bounded joint jog")
        handles["jog_button"] = jog_button
        handles["jog_status"] = server.gui.add_text("Jog status", initial_value="idle", disabled=True)

        @jog_button.on_click
        def _(_: Any) -> None:
            ok, message = safety.jog_joint(arm_group.value, int(joint_slider.value) - 1, float(delta_slider.value))
            handles["jog_status"].value = ("OK: " if ok else "BLOCKED: ") + message

    with tabs.add_tab("Velocity jog"):
        vel_arm = server.gui.add_button_group("Arm", ("left", "right"))
        vj_index = server.gui.add_slider("Joint index", min=1, max=6, step=1, initial_value=1)
        vj_vel = server.gui.add_slider("Joint vel deg/s", min=-10.0, max=10.0, step=0.5, initial_value=3.0)
        vj_button = server.gui.add_button("Send joint velocity")
        v_frame = server.gui.add_button_group("Twist frame", ("stand", "local"))
        v_axis = server.gui.add_button_group("Twist axis", ("X", "Y", "Z", "Rx", "Ry", "Rz"))
        v_lin = server.gui.add_slider("Linear vel m/s", min=-0.05, max=0.05, step=0.005, initial_value=0.02)
        v_ang = server.gui.add_slider("Angular vel rad/s", min=-0.2, max=0.2, step=0.02, initial_value=0.1)
        vt_button = server.gui.add_button("Send TCP twist")
        handles["velocity_status"] = server.gui.add_text("Velocity status", initial_value="idle", disabled=True)

        @vj_button.on_click
        def _(_: Any) -> None:
            vel = [0.0] * 6
            vel[int(vj_index.value) - 1] = float(vj_vel.value)
            ok, message = safety.send_joint_velocity(vel_arm.value, tuple(vel))
            handles["velocity_status"].value = ("OK: " if ok else "BLOCKED: ") + message

        @vt_button.on_click
        def _(_: Any) -> None:
            axis_index = {"X": 0, "Y": 1, "Z": 2, "Rx": 3, "Ry": 4, "Rz": 5}[v_axis.value]
            twist = [0.0] * 6
            twist[axis_index] = float(v_lin.value) if axis_index < 3 else float(v_ang.value)
            if v_frame.value == "local":
                ok, message = safety.send_tcp_twist_local(vel_arm.value, tuple(twist))
            else:
                ok, message = safety.send_tcp_twist_stand(vel_arm.value, tuple(twist))
            handles["velocity_status"].value = ("OK: " if ok else "BLOCKED: ") + message

    with tabs.add_tab("Circle"):
        c_diameter = server.gui.add_slider("Diameter m", min=0.02, max=0.20, step=0.01, initial_value=0.15)
        c_period = server.gui.add_slider("Period s", min=3.0, max=16.0, step=0.5, initial_value=4.0)
        c_start = server.gui.add_button("Start circle (both arms)")
        c_stop = server.gui.add_button("Stop circle")
        handles["circle_status"] = server.gui.add_text("Circle status", initial_value="idle", disabled=True)
        circle_stop_event = threading.Event()
        circle_state: dict[str, Any] = {"thread": None}

        def _circle_loop(diameter: float, period: float) -> None:
            safety.send_lifecycle("ArmMotion")
            while not circle_stop_event.is_set():
                ok, message = safety.send_tcp_circle_move(diameter, period, arm="both")
                if not ok:
                    handles["circle_status"].value = "BLOCKED: " + message
                    break
                circle_stop_event.wait(0.02)
            safety.command_client.send_lifecycle("Hold")

        @c_start.on_click
        def _(_: Any) -> None:
            existing = circle_state["thread"]
            if existing is not None and existing.is_alive():
                handles["circle_status"].value = "already running; press Stop first"
                return
            reason = safety.tcp_command_disabled_reason()
            if reason:
                handles["circle_status"].value = "BLOCKED: " + reason
                return
            circle_stop_event.clear()
            diameter = float(c_diameter.value)
            period = float(c_period.value)
            thread = threading.Thread(target=_circle_loop, args=(diameter, period), daemon=True)
            circle_state["thread"] = thread
            thread.start()
            handles["circle_status"].value = f"running: d={diameter:.2f}m p={period:.1f}s (both arms)"

        @c_stop.on_click
        def _(_: Any) -> None:
            circle_stop_event.set()
            handles["circle_status"].value = "stopped"

    with tabs.add_tab("TCP PTP"):
        handles["tcp_ptp_note"] = server.gui.add_text(
            "TCP PTP",
            initial_value="Move to TCP target, point-to-point. Cartesian path is not guaranteed.",
            disabled=True,
        )
        handles["tcp_status"] = server.gui.add_text(
            "TCP status",
            initial_value="TCP PTP target disabled until RB_GUI_ENABLE_TCP_POSE_COMMANDS=1",
            disabled=True,
        )
        _install_tcp_target_callbacks(handles["scene"], handles["tcp_status"])
        tcp_arm_group = server.gui.add_button_group("TCP arm", ("left", "right"))
        handles["tcp_frame_mode"] = _TCP_FRAME_LOCAL
        handles["tcp_frame_buttons"] = {}
        for frame_mode in _TCP_FRAME_OPTIONS:
            frame_button = server.gui.add_button(frame_mode, color=_mode_button_color(frame_mode, _tcp_frame_mode(handles)))
            handles["tcp_frame_buttons"][frame_mode] = frame_button

            @frame_button.on_click
            def _(_: Any, frame_mode: str = frame_mode) -> None:
                handles["tcp_frame_mode"] = frame_mode
                _update_tcp_frame_buttons(handles)
                handles["tcp_status"].value = f"TCP frame: {frame_mode}"

        linear_step = server.gui.add_slider("Linear step mm", min=0.1, max=10.0, step=0.1, initial_value=5.0)
        angular_step = server.gui.add_slider("Angular step deg", min=0.1, max=10.0, step=0.1, initial_value=1.0)
        handles["tcp_arm_group"] = tcp_arm_group
        handles["tcp_linear_step"] = linear_step
        handles["tcp_angular_step"] = angular_step
        handles["tcp_pose_buttons"] = []
        send_target_button = server.gui.add_button("Send TCP target")
        handles["tcp_pose_buttons"].append(send_target_button)

        @send_target_button.on_click
        def _(_: Any) -> None:
            arm = tcp_arm_group.value
            ok, message = _send_tcp_pose_target_from_marker(safety, handles["scene"], arm)
            handles["tcp_status"].value = ("OK: " if ok else "BLOCKED: ") + message

        def _send_ptp_delta(delta: tuple[float, float, float, float, float, float]) -> None:
            arm = tcp_arm_group.value
            frame_mode = _tcp_frame_mode(handles)
            ok, message = _apply_tcp_delta_and_send_pose_target(safety, handles["scene"], arm, delta, frame_mode)
            handles["tcp_status"].value = ("OK: " if ok else "BLOCKED: ") + message

        def _add_tcp_ptp_delta_button(label: str, axis_index: int, sign: float, angular: bool = False) -> None:
            button = server.gui.add_button(label)
            handles["tcp_pose_buttons"].append(button)

            @button.on_click
            def _(_: Any, axis_index: int = axis_index, sign: float = sign, angular: bool = angular) -> None:
                step = _angular_step_radians(float(angular_step.value)) if angular else _linear_step_meters(float(linear_step.value))
                delta = [0.0] * 6
                delta[axis_index] = sign * step
                _send_ptp_delta(tuple(delta))  # type: ignore[arg-type]

        for label, index, sign in (
            ("+X", 0, 1.0),
            ("-X", 0, -1.0),
            ("+Y", 1, 1.0),
            ("-Y", 1, -1.0),
            ("+Z", 2, 1.0),
            ("-Z", 2, -1.0),
        ):
            _add_tcp_ptp_delta_button(label, index, sign)
        for label, index, sign in (
            ("+roll", 3, 1.0),
            ("-roll", 3, -1.0),
            ("+pitch", 4, 1.0),
            ("-pitch", 4, -1.0),
            ("+yaw", 5, 1.0),
            ("-yaw", 5, -1.0),
        ):
            _add_tcp_ptp_delta_button(label, index, sign, angular=True)

    with tabs.add_tab("TCP Linear"):
        handles["tcp_linear_note"] = server.gui.add_text(
            "TCP Linear",
            initial_value="Move linearly in Cartesian TCP space. Constant orientation keeps the start orientation.",
            disabled=True,
        )
        handles["tcp_linear_status"] = server.gui.add_text(
            "TCP Linear status",
            initial_value="TCP Linear disabled until RB_GUI_ENABLE_TCP_POSE_COMMANDS=1",
            disabled=True,
        )
        handles["tcp_linear_source"] = server.gui.add_text(
            "Target marker source",
            initial_value="current TCP target marker",
            disabled=True,
        )
        handles["tcp_linear_arm"] = "both"
        handles["tcp_linear_arm_buttons"] = {}
        for arm in _TCP_LINEAR_ARM_OPTIONS:
            arm_button = server.gui.add_button(arm, color=_mode_button_color(arm, _tcp_linear_arm(handles)))
            handles["tcp_linear_arm_buttons"][arm] = arm_button

            @arm_button.on_click
            def _(_: Any, arm: str = arm) -> None:
                handles["tcp_linear_arm"] = arm
                _update_tcp_linear_selection_buttons(handles)
                handles["tcp_linear_status"].value = f"TCP Linear arm: {arm}"

        linear_duration = server.gui.add_slider("duration_sec", min=0.05, max=10.0, step=0.05, initial_value=2.0)
        linear_speed = server.gui.add_slider("linear_speed_m_s", min=0.001, max=0.05, step=0.001, initial_value=0.03)
        angular_speed = server.gui.add_slider("angular_speed_rad_s", min=0.01, max=0.3, step=0.01, initial_value=0.2)
        handles["tcp_linear_orientation_mode"] = "slerp"
        handles["tcp_linear_orientation_buttons"] = {}
        for orientation_mode in _TCP_LINEAR_ORIENTATION_MODES:
            orientation_button = server.gui.add_button(
                orientation_mode,
                color=_mode_button_color(orientation_mode, _tcp_linear_orientation_mode(handles)),
            )
            handles["tcp_linear_orientation_buttons"][orientation_mode] = orientation_button

            @orientation_button.on_click
            def _(_: Any, orientation_mode: str = orientation_mode) -> None:
                handles["tcp_linear_orientation_mode"] = orientation_mode
                _update_tcp_linear_selection_buttons(handles)
                handles["tcp_linear_status"].value = f"TCP Linear orientation_mode: {orientation_mode}"

        handles["tcp_linear_buttons"] = []
        send_linear_button = server.gui.add_button("Send TCP Linear Move")
        handles["tcp_linear_buttons"].append(send_linear_button)

        @send_linear_button.on_click
        def _(_: Any) -> None:
            ok, message = _send_tcp_linear_move_from_marker(
                safety,
                handles["scene"],
                _tcp_linear_arm(handles),
                duration_sec=float(linear_duration.value),
                linear_speed_m_s=float(linear_speed.value),
                angular_speed_rad_s=float(angular_speed.value),
                orientation_mode=_tcp_linear_orientation_mode(handles),
            )
            handles["tcp_linear_status"].value = ("OK: " if ok else "BLOCKED: ") + message

    with tabs.add_tab("Low-level Delta Debug"):
        handles["tcp_debug_note"] = server.gui.add_text(
            "Low-level Delta Debug",
            initial_value="Raw TcpDeltaLocal/Stand. Usually not used for normal GUI target moves.",
            disabled=True,
        )
        handles["tcp_debug_status"] = server.gui.add_text(
            "Delta debug status",
            initial_value="Raw TcpDeltaLocal/TcpDeltaStand debug controls",
            disabled=True,
        )
        tcp_debug_arm_group = server.gui.add_button_group("Arm", ("left", "right"))

        def _send_low_level_delta(delta: tuple[float, float, float, float, float, float]) -> None:
            arm = tcp_debug_arm_group.value
            frame_mode = _tcp_frame_mode(handles)
            if frame_mode == _TCP_FRAME_LOCAL:
                ok, message = safety.send_tcp_delta_local(arm, delta)
            else:
                ok, message = safety.send_tcp_delta_stand(arm, delta)
            if ok:
                _apply_tcp_delta_to_target(handles["scene"], arm, delta, frame_mode)
            status = ("OK: " if ok else "BLOCKED: ") + message
            handles["tcp_debug_status"].value = status
            handles["tcp_status"].value = status

        def _add_low_level_delta_button(label: str, axis_index: int, sign: float, angular: bool = False) -> None:
            button = server.gui.add_button(label)
            handles["tcp_pose_buttons"].append(button)

            @button.on_click
            def _(_: Any, axis_index: int = axis_index, sign: float = sign, angular: bool = angular) -> None:
                step = _angular_step_radians(float(angular_step.value)) if angular else _linear_step_meters(float(linear_step.value))
                delta = [0.0] * 6
                delta[axis_index] = sign * step
                _send_low_level_delta(tuple(delta))  # type: ignore[arg-type]

        for label, index, sign in (
            ("+X", 0, 1.0),
            ("-X", 0, -1.0),
            ("+Y", 1, 1.0),
            ("-Y", 1, -1.0),
            ("+Z", 2, 1.0),
            ("-Z", 2, -1.0),
        ):
            _add_low_level_delta_button(label, index, sign)
        for label, index, sign in (
            ("+roll", 3, 1.0),
            ("-roll", 3, -1.0),
            ("+pitch", 4, 1.0),
            ("-pitch", 4, -1.0),
            ("+yaw", 5, 1.0),
            ("-yaw", 5, -1.0),
        ):
            _add_low_level_delta_button(label, index, sign, angular=True)

    with tabs.add_tab("Debug"):
        handles["tick"] = server.gui.add_number("tick", initial_value=0, disabled=True)
        handles["scene_assets"] = server.gui.add_text(
            "scene assets",
            initial_value=_format_scene_asset_status(handles.get("scene", {})),
            disabled=True,
        )
        handles["left_q"] = server.gui.add_text("left q_actual", initial_value="[]", disabled=True)
        handles["right_q"] = server.gui.add_text("right q_actual", initial_value="[]", disabled=True)
        handles["left_sent"] = server.gui.add_text("left q_sent", initial_value="[]", disabled=True)
        handles["right_sent"] = server.gui.add_text("right q_sent", initial_value="[]", disabled=True)
        handles["left_prev_sent"] = server.gui.add_text("left previous sent", initial_value="[]", disabled=True)
        handles["right_prev_sent"] = server.gui.add_text("right previous sent", initial_value="[]", disabled=True)
        handles["timestamps"] = server.gui.add_text("timestamps", initial_value="n/a", disabled=True)
        handles["timing"] = server.gui.add_text("period/jitter/filter", initial_value="n/a", disabled=True)
        handles["send_durations"] = server.gui.add_text("send durations", initial_value="n/a", disabled=True)
        handles["arm_modes"] = server.gui.add_text("arm modes/connections", initial_value="unknown", disabled=True)
        handles["logger"] = server.gui.add_text("logger health", initial_value="unknown", disabled=True)
        handles["packets"] = server.gui.add_text("state packets", initial_value="0 received / 0 invalid", disabled=True)

    return handles


def _latest_circle_overlay(
    overlay_store: CircleOverlayStore | None,
) -> tuple[CircleOverlaySnapshot | None, bool]:
    if overlay_store is None:
        return None, True
    return overlay_store.latest(), overlay_store.is_stale()


def _update_circle_overlay_gui(handles: dict[str, Any], overlay_store: CircleOverlayStore | None) -> None:
    overlay, stale = _latest_circle_overlay(overlay_store)
    if "circle_overlay" in handles:
        handles["circle_overlay"].value = _format_circle_overlay_status(
            overlay,
            stale=stale,
            enabled=overlay_store is not None,
        )
    update_circle_overlay(handles.get("scene", {}), overlay, stale=stale)


def update_gui(
    handles: dict[str, Any],
    safety: OperatorSafety,
    store: StateStore,
    overlay_store: CircleOverlayStore | None = None,
) -> None:
    disabled_states = safety.control_disabled_states()
    for mode, button in handles.get("lifecycle_buttons", {}).items():
        _set_disabled(button, disabled_states.get(f"lifecycle:{mode}", True))
    if "init_motion_button" in handles:
        _set_disabled(handles["init_motion_button"], disabled_states.get("init_motion", True))
    if "jog_button" in handles:
        _set_disabled(handles["jog_button"], disabled_states.get("jog", True))
    for button in handles.get("tcp_pose_buttons", ()):
        _set_disabled(button, disabled_states.get("tcp_pose", True))
    for button in handles.get("tcp_linear_buttons", ()):
        _set_disabled(button, disabled_states.get("tcp_linear", True))

    if "mode_buttons" in handles:
        _update_desired_mode_buttons(handles, safety.desired_mode)
    if "tcp_frame_buttons" in handles:
        _update_tcp_frame_buttons(handles)
    if "tcp_display_buttons" in handles:
        _update_tcp_display_buttons(handles)
    if "tcp_linear_arm_buttons" in handles or "tcp_linear_orientation_buttons" in handles:
        _update_tcp_linear_selection_buttons(handles)
    latest = store.latest()
    stale = store.is_stale()
    readiness = safety.readiness()
    _update_circle_overlay_gui(handles, overlay_store)
    if latest is None:
        _update_operator_monitors(handles, None, stale=True)
        handles["connection"].value = "disconnected/stale"
        handles["readiness"].value = readiness.no_go_reason or "No-Go: no state stream"
        if "fk_status" in handles:
            handles["fk_status"].value = _format_fk_status(None, stale=True)
        if "tcp_tracking" in handles:
            handles["tcp_tracking"].value = _format_tcp_tracking_status(
                None,
                stale=True,
                display_mode=_tcp_display_mode(handles),
            )
        if "pgmode_status" in handles:
            handles["pgmode_status"].value = _format_pgmode_status(
                None,
                stale=True,
                display_mode=_tcp_display_mode(handles),
            )
        if "cartesian_solve" in handles:
            handles["cartesian_solve"].value = _format_cartesian_solve_status(None, stale=True)
        if "tcp_status" in handles:
            handles["tcp_status"].value = _format_tcp_command_status(safety, None, stale=True)
        if "tcp_linear_status" in handles:
            handles["tcp_linear_status"].value = _format_tcp_command_status(safety, None, stale=True)
        handles["packets"].value = f"{store.received_packets} received / {store.invalid_packets} invalid"
        return

    handles["connection"].value = "stale" if stale else "live"
    mode_parts = [
        f"desired={safety.desired_mode}",
        f"observed={safety.observed_server_mode}",
        f"backend={safety.observed_backend}",
    ]
    if safety.config_warnings:
        mode_parts.append("warning=" + "; ".join(safety.config_warnings))
    handles["mode"].value = ", ".join(mode_parts)
    readiness_parts = [
        f"configured={readiness.configured}",
        f"running={readiness.running}",
        f"connected={readiness.connected}",
        f"ready={readiness.ready}",
        f"fault={readiness.fault}",
    ]
    if readiness.no_go_reason:
        readiness_parts.append("No-Go: " + readiness.no_go_reason)
    handles["readiness"].value = ", ".join(readiness_parts)
    handles["motion"].value = latest.motion_state
    handles["fault"].value = latest.fault_reason if latest.fault_latched else "none"
    if "fk_status" in handles:
        handles["fk_status"].value = _format_fk_status(latest, stale=stale)
    if "tcp_tracking" in handles:
        handles["tcp_tracking"].value = _format_tcp_tracking_status(
            latest,
            stale=stale,
            display_mode=_tcp_display_mode(handles),
        )
    if "pgmode_status" in handles:
        handles["pgmode_status"].value = _format_pgmode_status(
            latest,
            stale=stale,
            display_mode=_tcp_display_mode(handles),
        )
    if "cartesian_solve" in handles:
        handles["cartesian_solve"].value = _format_cartesian_solve_status(latest, stale=stale)
    if "tcp_status" in handles:
        handles["tcp_status"].value = _format_tcp_command_status(safety, latest, stale=stale)
    if "tcp_linear_status" in handles:
        handles["tcp_linear_status"].value = _format_tcp_command_status(safety, latest, stale=stale)
    _update_operator_monitors(handles, latest, stale=stale)
    update_scene_markers(handles.get("scene", {}), latest, tcp_display_mode=_tcp_display_mode(handles))
    if "scene_assets" in handles:
        handles["scene_assets"].value = _format_scene_asset_status(handles.get("scene", {}))
    handles["tick"].value = latest.tick
    handles["left_q"].value = _format_joints(latest.left.q_actual_deg)
    handles["right_q"].value = _format_joints(latest.right.q_actual_deg)
    handles["left_sent"].value = _format_joints(latest.left.q_sent_deg)
    handles["right_sent"].value = _format_joints(latest.right.q_sent_deg)
    handles["left_prev_sent"].value = _format_joints(latest.left.q_previous_sent_deg)
    handles["right_prev_sent"].value = _format_joints(latest.right.q_previous_sent_deg)
    raw = latest.raw
    handles["timestamps"].value = f"host={raw.get('host_time_ns')} loop_start={raw.get('loop_start_time_ns')} loop_end={raw.get('loop_end_time_ns')} left_host={raw.get('left', {}).get('host_time_ns')} right_host={raw.get('right', {}).get('host_time_ns')}"
    handles["timing"].value = f"period={raw.get('period_ms')} ms jitter={raw.get('jitter_ms')} ms filter={raw.get('filter_dt_ms')} ms skew={raw.get('send_skew_us')} us"
    handles["send_durations"].value = f"left={raw.get('left', {}).get('send_duration_us')} us right={raw.get('right', {}).get('send_duration_us')} us"
    handles["arm_modes"].value = (
        f"left={latest.left.mode}/{latest.left.connection_state}/valid_joints={latest.left.has_valid_joint_state} "
        f"right={latest.right.mode}/{latest.right.connection_state}/valid_joints={latest.right.has_valid_joint_state}"
    )
    handles["logger"].value = str(latest.logger_health)
    handles["packets"].value = f"{store.received_packets} received / {store.invalid_packets} invalid"


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    print(format_scene_asset_startup_status(), flush=True)
    if args.check_assets:
        return
    try:
        import viser
    except ImportError as exc:
        raise SystemExit("viser is required for the browser GUI. Install with `pip install -e rb_gui`.") from exc

    host = os.environ.get("RB_GUI_HOST", "0.0.0.0")
    port = _env_int("RB_GUI_PORT", 8080)
    state_host = os.environ.get("RB_GUI_STATE_BIND", "0.0.0.0")
    state_port = _env_int("RB_GUI_STATE_PORT", 50110)
    command_host = os.environ.get("RB_GUI_COMMAND_HOST", "127.0.0.1")
    command_port = _env_int("RB_GUI_COMMAND_PORT", 50010)
    observed_mode_raw = os.environ.get("RB_GUI_OBSERVED_MODE", "mock")
    observed_backend_raw = os.environ.get("RB_GUI_OBSERVED_BACKEND", "")
    observed = normalize_observed_mode_backend(observed_mode_raw, observed_backend_raw)
    ops_available = os.environ.get("RB_GUI_OPS_AVAILABLE", "0") == "1"
    enable_tcp_pose_commands = os.environ.get("RB_GUI_ENABLE_TCP_POSE_COMMANDS", "0") == "1"
    enable_controller_sim_cartesian = os.environ.get("RB_GUI_ENABLE_CONTROLLER_SIM_CARTESIAN", "0") == "1"
    init_left_joints = _env_joint6("RB_GUI_INIT_LEFT_JOINTS", _DEFAULT_INIT_LEFT_JOINTS_DEG)
    init_right_joints = _env_joint6("RB_GUI_INIT_RIGHT_JOINTS", _DEFAULT_INIT_RIGHT_JOINTS_DEG)
    init_motion_timeout_sec = _env_float("RB_GUI_INIT_MOTION_TIMEOUT_SEC", 10.0)

    store = StateStore()
    receiver = StateReceiver(store, host=state_host, port=state_port)
    receiver.start()
    circle_overlay_bind = _circle_overlay_bind_from_args_env(args)
    overlay_store: CircleOverlayStore | None = None
    overlay_receiver: CircleOverlayReceiver | None = None
    if circle_overlay_bind is not None:
        overlay_host, overlay_port = circle_overlay_bind
        overlay_store = CircleOverlayStore()
        overlay_receiver = CircleOverlayReceiver(overlay_store, host=overlay_host, port=overlay_port)
        overlay_receiver.start()
    safety = OperatorSafety(
        store,
        CommandClient(command_host, command_port),
        desired_mode=observed.mode,
        observed_server_mode=observed_mode_raw,
        observed_backend=observed_backend_raw,
        sim_readiness=_sim_readiness_from_env(observed),
        ops_available=ops_available,
        enable_tcp_pose_commands=enable_tcp_pose_commands,
        enable_controller_sim_cartesian=enable_controller_sim_cartesian,
        init_left_joint_deg=init_left_joints,
        init_right_joint_deg=init_right_joints,
        init_motion_timeout_sec=init_motion_timeout_sec,
    )
    server = viser.ViserServer(host=host, port=port)
    handles = build_gui(server, safety, store, overlay_store=overlay_store)
    overlay_status = (
        f", circle overlay UDP {circle_overlay_bind[0]}:{circle_overlay_bind[1]}"
        if circle_overlay_bind is not None
        else ", circle overlay disabled"
    )
    print(
        f"rb_servo_gui listening on http://{host}:{port}, UDP state {state_host}:{state_port}{overlay_status}",
        flush=True,
    )

    try:
        while True:
            update_gui(handles, safety, store, overlay_store=overlay_store)
            time.sleep(0.1)
    finally:
        receiver.stop()
        if overlay_receiver is not None:
            overlay_receiver.stop()


if __name__ == "__main__":
    main()
