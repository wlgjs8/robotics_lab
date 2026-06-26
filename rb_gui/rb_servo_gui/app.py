from __future__ import annotations

import argparse
import json
import math
import os
import socket
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
    _wxyz_to_xyzw,
    _xyzw_to_wxyz,
)
from .models import ArmSnapshot, CircleOverlaySnapshot, Pose6D, StateSnapshot
from .overlay_receiver import CircleOverlayReceiver, CircleOverlayStore, parse_udp_bind
from .plane_fit import fit_plane, tilt_deg
from .safety import OperatorSafety, normalize_observed_mode_backend
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
    set_ik_infeasible_region_visible,
    set_reach_envelope_visible,
    update_circle_overlay,
    update_floor_check_points,
    update_floor_plane,
    update_floor_plane_preview,
    update_roi_box,
    update_roi_box_preview,
    update_user_floor_plane,
    update_user_floor_capture_points,
    update_self_collision_check_geom,
    update_self_collision_near_pairs,
    update_self_collision_overlay,
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
    _format_floor_constraint_status,
    _format_roi_box_status,
    _format_user_floor_constraint_status,
    _format_pgmode_status,
    _format_self_collision_status,
    _format_stand_world_pose_value,
    _format_tcp_command_status,
    _format_tcp_tracking_status,
    _joint_monitor_unit,
    _mode_button_color,
    _optional_finite,
    _reflect_gate_reason,
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
_TCP_LINEAR_ARM_OPTIONS = ("both", "left", "right")
_TCP_PTP_ARM_OPTIONS = ("both", "left", "right")
_TCP_PTP_AXES = (
    ("x", 0, False),
    ("y", 1, False),
    ("z", 2, False),
    ("roll", 3, True),
    ("pitch", 4, True),
    ("yaw", 5, True),
)
def _nudge_label(text: str, width: int = 6) -> str:
    """Center a button-group middle label with non-breaking spaces so the −/+
    segments line up vertically across rows of different label lengths
    (e.g. 'X' vs 'Pitch'). NBSP survives viser's whitespace trimming."""
    text = str(text)
    pad = max(0, width - len(text))
    left = pad // 2
    return " " * left + text + " " * (pad - left)


_TCP_LINEAR_ORIENTATION_MODES = ("constant", "slerp")
# Default viewer camera for newly connecting clients, in stand-frame meters
# (Z-up). Looks at the dual-arm robot from the front (-Y side, looking toward
# +Y) at a near-level angle, so the horizon stays level. Captured live from the
# desired zoomed-in framing on the grippers/TCP (2026-06-15). Override per launch
# with RB_GUI_CAMERA_POSITION / RB_GUI_CAMERA_LOOK_AT / RB_GUI_CAMERA_UP
# ("x,y,z" in meters).
_DEFAULT_CAMERA_POSITION = (-0.0128, -1.3823, 0.3985)
_DEFAULT_CAMERA_LOOK_AT = (-0.0128, -0.1381, 0.2257)
_DEFAULT_CAMERA_UP = (0.0, 0.0, 1.0)
# TcpTargetPose replay/profiling InitMotion anchor (2026-06-21): GRIPPER-DOWN (tool z ~ stand
# -z), max single-rest coverage. Left = controller-consistent rest; right = rest + 10deg about
# y (within +/-20deg tool budget; recovers episode_0096). Server-CALIBRATED offline model
# (single-arm rb3_730e.urdf + stack_sim mount, exact ee_local incl r_align=pika_rz180;
# reproduces live REST=12/12, FWD=0/12). Coverage 383/387; the 4 unrecoverable left-wrist
# singularities (0043,0220,0291,0373) were removed from data_tcp. Mirrors
# scripts/batch_replay_episodes.py PROFILE_ANCHOR_*. Override with RB_GUI_INIT_*_JOINTS.
_DEFAULT_INIT_LEFT_JOINTS_DEG = (259.0, 75.6, 129.5, -55.6, -131.2, -161.7)
_DEFAULT_INIT_RIGHT_JOINTS_DEG = (-253.7, -76.9, -127.6, 65.7, 143.7, 166.9)
_OPERATOR_MONITOR_WIDTH_EM = 18.0
_OPERATOR_MONITOR_GAP_EM = 1.0
# Vertical anchor (em, in monitor-card font size) where the Pose Monitor stacks
# below the Joint Monitor. Sized to clear the Joint card's full content (12 joint
# rows + status) so it never needs an inner scrollbar, plus a small gap so the two
# panels sit slightly apart instead of touching. Override with RB_GUI_MONITOR_SPLIT_EM.
_OPERATOR_MONITOR_SPLIT_EM = 35.5


def _env_int(name: str, fallback: int) -> int:
    try:
        return int(os.environ.get(name, str(fallback)))
    except ValueError:
        return fallback


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


def _env_vec3(name: str, fallback: tuple[float, float, float]) -> tuple[float, float, float]:
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return fallback
    try:
        values = tuple(float(part.strip()) for part in raw.split(","))
    except ValueError:
        return fallback
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        return fallback
    return values  # type: ignore[return-value]


def _install_default_camera(server: Any) -> None:
    # Stand-frame default view for new clients (Z-up, level horizon). The world
    # up direction keeps orbiting level; each connecting client is framed on the
    # robot from the front. Guarded so non-viser test servers stay unaffected.
    position = _env_vec3("RB_GUI_CAMERA_POSITION", _DEFAULT_CAMERA_POSITION)
    look_at = _env_vec3("RB_GUI_CAMERA_LOOK_AT", _DEFAULT_CAMERA_LOOK_AT)
    up = _env_vec3("RB_GUI_CAMERA_UP", _DEFAULT_CAMERA_UP)
    scene = getattr(server, "scene", None)
    if scene is not None and hasattr(scene, "set_up_direction"):
        try:
            scene.set_up_direction(up)
        except Exception:
            pass
    if not hasattr(server, "on_client_connect"):
        return

    @server.on_client_connect
    def _(client: Any) -> None:
        camera = getattr(client, "camera", None)
        if camera is None:
            return
        try:
            camera.position = position
            camera.look_at = look_at
            camera.up_direction = up
        except Exception:
            pass


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


def _update_desired_mode_buttons(handles: dict[str, Any], desired_mode: str) -> None:
    for mode, button in handles.get("mode_buttons", {}).items():
        try:
            button.color = _mode_button_color(mode, desired_mode)
        except Exception:
            pass


def _tcp_frame_mode(handles: dict[str, Any]) -> str:
    mode = handles.get("tcp_frame_mode", _TCP_FRAME_STAND)
    return mode if mode in _TCP_FRAME_OPTIONS else _TCP_FRAME_STAND


def _update_tcp_frame_buttons(handles: dict[str, Any]) -> None:
    selected = _tcp_frame_mode(handles)
    for mode, button in handles.get("tcp_frame_buttons", {}).items():
        try:
            button.color = _mode_button_color(mode, selected)
        except Exception:
            pass


def _tcp_ptp_arm(handles: dict[str, Any]) -> str:
    selected = handles.get("tcp_ptp_arm", "both")
    return selected if selected in _TCP_PTP_ARM_OPTIONS else "both"


def _update_tcp_ptp_arm_buttons(handles: dict[str, Any]) -> None:
    selected_arm = _tcp_ptp_arm(handles)
    for arm, button in handles.get("tcp_ptp_arm_buttons", {}).items():
        try:
            button.color = _mode_button_color(arm, selected_arm)
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


def _set_tcp_pose_absolute_marker(
    scene_handles: dict[str, Any],
    arm: str,
    values_mm_deg: list[float] | tuple[float, ...],
) -> bool:
    """Set one arm's TCP target marker to the absolute stand-frame pose held in its
    six PTP fields (x/y/z mm, roll/pitch/yaw deg). Does NOT send — the caller sends
    once, covering all the arms it moved, so a single-arm packet never resets the
    other arm to Hold.

    The orientation is rebuilt with `_rpy_to_wxyz` (Rz·Ry·Rx) — the SAME
    composition the server uses in `rotationFromPose` — so feeding back the
    server's published `rx/ry/rz` reproduces the exact current orientation (no
    drift on an unedited commit), and the commanded pose always matches the
    numbers the operator sees."""
    if arm not in {"left", "right"}:
        return False
    if len(values_mm_deg) != 6 or not all(math.isfinite(float(v)) for v in values_mm_deg):
        return False
    position = (
        float(values_mm_deg[0]) * 0.001,
        float(values_mm_deg[1]) * 0.001,
        float(values_mm_deg[2]) * 0.001,
    )
    wxyz = _normalize_wxyz(
        _rpy_to_wxyz(
            math.radians(float(values_mm_deg[3])),
            math.radians(float(values_mm_deg[4])),
            math.radians(float(values_mm_deg[5])),
        )
    )
    pose6 = _pose6_from_transform(position, wxyz)
    scene_handles[f"{arm}_tcp_target_pose"] = pose6
    scene_handles[f"{arm}_tcp_target_wxyz"] = wxyz
    scene_handles[f"{arm}_tcp_target_user_moved"] = True
    handle = scene_handles.get(f"{arm}_tcp_target")
    if handle is not None:
        try:
            handle.position = pose6[:3]
        except Exception:
            pass
        try:
            handle.wxyz = wxyz
        except Exception:
            pass
    return True


def _send_tcp_poses_absolute(
    safety: OperatorSafety,
    scene_handles: dict[str, Any],
    arm_values: dict[str, list[float] | tuple[float, ...]],
) -> tuple[bool, str]:
    """Set the absolute stand-frame target(s) for one or BOTH arms, then send a
    SINGLE TcpPoseTarget packet covering exactly those arms. Sending both arms in
    one packet is required: a single-arm packet resets the other arm to Hold on the
    server, so two back-to-back single-arm packets make the second clobber the
    first (only one arm ends up moving)."""
    if not arm_values:
        return False, "no TCP target selected"
    for arm, values in arm_values.items():
        if not _set_tcp_pose_absolute_marker(scene_handles, arm, values):
            return False, f"{arm} TCP target unavailable"
    scope = "both" if set(arm_values) == {"left", "right"} else next(iter(arm_values))
    return _send_tcp_pose_target_from_marker(safety, scene_handles, scope)


def _set_tcp_pose_absolute_and_send(
    safety: OperatorSafety,
    scene_handles: dict[str, Any],
    arm: str,
    values_mm_deg: list[float] | tuple[float, ...],
) -> tuple[bool, str]:
    """Single-arm convenience wrapper around `_send_tcp_poses_absolute`."""
    return _send_tcp_poses_absolute(safety, scene_handles, {arm: list(values_mm_deg)})


def _ptp_arm_pose_values(handles: dict[str, Any], arm: str) -> list[float] | None:
    """Six stand-frame values (x/y/z mm, roll/pitch/yaw deg) the PTP fields should
    show for one arm: the live current pose (same selection as the Pose Monitor),
    or all-zero relative deltas in TCP-local frame. None if no fresh/valid pose."""
    if _tcp_frame_mode(handles) == _TCP_FRAME_LOCAL:
        return [0.0] * 6
    latest = handles.get("_latest_state")
    stale = bool(handles.get("_state_stale"))
    arm_state = getattr(latest, arm, None) if latest is not None else None
    pose, valid, _is_sim = _stand_world_monitor_pose(arm_state, stale=stale)
    if pose is None or not valid:
        return None
    return [
        pose.x * 1000.0,
        pose.y * 1000.0,
        pose.z * 1000.0,
        math.degrees(pose.rx),
        math.degrees(pose.ry),
        math.degrees(pose.rz),
    ]


def _read_ptp_arm_fields(handles: dict[str, Any], arm: str) -> list[float] | None:
    """Current values (mm / deg) of one arm's six PTP slots, read from the per-axis
    vector2 widgets (slot 0 = left, slot 1 = right)."""
    vecs = handles.get("tcp_ptp_axis_vec")
    if not vecs:
        return None
    slot = 0 if arm == "left" else 1
    values: list[float] = []
    for _axis_label, axis_index, _angular in _TCP_PTP_AXES:
        vec = vecs.get(axis_index)
        if vec is None:
            return None
        try:
            values.append(float(vec.value[slot]))
        except Exception:
            return None
    return values


def _refresh_tcp_ptp_axis_fields(handles: dict[str, Any]) -> bool:
    """Mirror the live current pose into the per-axis [left | right] vector2 fields.

    In stand/world frame each axis shows the same stand-frame pose as the Pose
    Monitor (mm / deg, server `rx/ry/rz`) for left (slot 0) and right (slot 1). In
    TCP-local frame the fields are relative deltas, so they read 0. The patched
    viser VectorInput ignores server updates for a slot while it is focused, so
    this can repaint every tick without fighting a value the operator is typing;
    the `_tcp_ptp_field_updating` guard additionally keeps these writes from
    re-triggering the commit callback. `_tcp_ptp_shown` records the last value
    written to each slot — the commit handler diffs against it to find which arm
    the operator actually edited."""
    vecs = handles.get("tcp_ptp_axis_vec")
    if not vecs:
        return False
    shown = handles.setdefault("_tcp_ptp_shown", {})
    left_values = _ptp_arm_pose_values(handles, "left")
    right_values = _ptp_arm_pose_values(handles, "right")
    did_set = False
    handles["_tcp_ptp_field_updating"] = True
    try:
        for _axis_label, axis_index, _angular in _TCP_PTP_AXES:
            vec = vecs.get(axis_index)
            if vec is None:
                continue
            prev = shown.get(axis_index)
            left_val = (
                float(left_values[axis_index]) if left_values is not None
                else (prev[0] if prev is not None else 0.0)
            )
            right_val = (
                float(right_values[axis_index]) if right_values is not None
                else (prev[1] if prev is not None else 0.0)
            )
            new_value = (left_val, right_val)
            # Skip no-op writes (sub-0.001 mm/deg) to keep websocket churn low.
            if prev is not None and abs(prev[0] - left_val) < 1e-3 and abs(prev[1] - right_val) < 1e-3:
                continue
            try:
                vec.value = new_value
                shown[axis_index] = new_value
                did_set = True
            except Exception:
                pass
    finally:
        handles["_tcp_ptp_field_updating"] = False
    return did_set


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
    left_pose, right_pose, left_quaternion_xyzw, right_quaternion_xyzw, error = _tcp_marker_payloads(scene_handles, arm)
    if error:
        return False, error
    return safety.send_tcp_pose_target(
        left_pose=left_pose,
        right_pose=right_pose,
        left_quaternion_xyzw=left_quaternion_xyzw,
        right_quaternion_xyzw=right_quaternion_xyzw,
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
    arms = ("left", "right") if arm == "both" else (arm,)
    for single_arm in arms:
        if not _apply_tcp_delta_to_target(scene_handles, single_arm, delta, frame_mode):
            return False, f"{single_arm} TCP target unavailable"
    return _send_tcp_pose_target_from_marker(safety, scene_handles, arm)


def _waypoints_path() -> str:
    raw = os.environ.get("RB_GUI_WAYPOINTS_PATH", "").strip()
    if raw:
        return raw
    return os.path.join(os.path.expanduser("~"), ".rb_servo_gui", "waypoints.json")


def _waypoint_seq(value: Any, length: int) -> tuple[float, ...] | None:
    if not isinstance(value, list | tuple) or len(value) != length:
        return None
    try:
        out = tuple(float(item) for item in value)
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(item) for item in out):
        return None
    return out


def _normalize_loaded_waypoint(wp: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "left_q": _waypoint_seq(wp.get("left_q"), 6),
        "left_pose": _waypoint_seq(wp.get("left_pose"), 6),
        "left_quat": _waypoint_seq(wp.get("left_quat"), 4),
        "right_q": _waypoint_seq(wp.get("right_q"), 6),
        "right_pose": _waypoint_seq(wp.get("right_pose"), 6),
        "right_quat": _waypoint_seq(wp.get("right_quat"), 4),
    }


def _load_waypoints() -> dict[str, Any]:
    path = _waypoints_path()
    try:
        with open(path, encoding="utf-8") as handle:
            data = json.load(handle)
    except FileNotFoundError:
        return {}
    except (OSError, json.JSONDecodeError):
        return {}
    if not isinstance(data, dict):
        return {}
    out: dict[str, Any] = {}
    for name, wp in data.items():
        if isinstance(name, str) and name.strip() and isinstance(wp, Mapping):
            out[name] = _normalize_loaded_waypoint(wp)
    return out


def _save_waypoints(waypoints: Mapping[str, Any]) -> tuple[bool, str]:
    path = _waypoints_path()
    try:
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(waypoints, handle, indent=2, sort_keys=True)
    except OSError as exc:
        return False, str(exc)
    return True, path


def _persist_waypoints(handles: dict[str, Any]) -> str:
    ok, info = _save_waypoints(handles.get("waypoints", {}))
    return f"saved to {info}" if ok else f"save failed: {info}"


def _delete_waypoint(handles: dict[str, Any]) -> tuple[bool, str]:
    name, _ = _selected_waypoint(handles)
    waypoints = handles.get("waypoints", {})
    if not name or name not in waypoints:
        return False, "no waypoint selected to delete"
    del waypoints[name]
    return True, name


# ---- User Safety Floor: captured floor-contact points + fitted plane ----


def _user_floor_path() -> str:
    raw = os.environ.get("RB_GUI_USER_FLOOR_PATH", "").strip()
    if raw:
        return raw
    return os.path.join(os.path.expanduser("~"), ".rb_servo_gui", "user_floor.json")


def _user_floor_state(handles: dict[str, Any]) -> dict[str, Any]:
    """Mutable user-floor GUI state, lazily initialised. points is a list of
    {"arm": "left"|"right", "p": [x, y, z]} captured floor-contact samples."""
    return handles.setdefault(
        "user_floor",
        {"points": [], "plane": None, "margin_mm": 0.0, "enabled": False},
    )


def _load_user_floor() -> dict[str, Any]:
    path = _user_floor_path()
    state: dict[str, Any] = {"points": [], "plane": None, "margin_mm": 0.0, "enabled": False}
    try:
        with open(path, encoding="utf-8") as handle:
            data = json.load(handle)
    except FileNotFoundError:
        return state
    except (OSError, json.JSONDecodeError):
        return state
    if not isinstance(data, Mapping):
        return state
    points: list[dict[str, Any]] = []
    for entry in data.get("points", []) or []:
        if not isinstance(entry, Mapping):
            continue
        p = _waypoint_seq(entry.get("p"), 3)
        arm = entry.get("arm")
        if p is not None and arm in ("left", "right"):
            points.append({"arm": arm, "p": list(p)})
    state["points"] = points
    plane = data.get("plane")
    if isinstance(plane, Mapping):
        point = _waypoint_seq(plane.get("point"), 3)
        normal = _waypoint_seq(plane.get("normal"), 3)
        if point is not None and normal is not None:
            state["plane"] = {"point": list(point), "normal": list(normal)}
    margin = data.get("margin_mm")
    if isinstance(margin, (int, float)) and math.isfinite(margin):
        state["margin_mm"] = float(margin)
    state["enabled"] = bool(data.get("enabled", False))
    return state


def _save_user_floor(state: Mapping[str, Any]) -> tuple[bool, str]:
    path = _user_floor_path()
    payload = {
        "points": [{"arm": e["arm"], "p": list(e["p"])} for e in state.get("points", [])],
        "plane": state.get("plane"),
        "margin_mm": float(state.get("margin_mm", 0.0)),
        "enabled": bool(state.get("enabled", False)),
    }
    try:
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2)
    except OSError as exc:
        return False, str(exc)
    return True, path


# ---- Persisted GUI preferences (operator panel toggles that should survive a
# restart, e.g. the stand-floor enforce checkbox). One small JSON dict keyed by
# preference name; mirrors the waypoints/user_floor persistence pattern. ----


def _gui_settings_path() -> str:
    raw = os.environ.get("RB_GUI_SETTINGS_PATH", "").strip()
    if raw:
        return raw
    return os.path.join(os.path.expanduser("~"), ".rb_servo_gui", "settings.json")


def _load_gui_settings() -> dict[str, Any]:
    path = _gui_settings_path()
    try:
        with open(path, encoding="utf-8") as handle:
            data = json.load(handle)
    except FileNotFoundError:
        return {}
    except (OSError, json.JSONDecodeError):
        return {}
    return dict(data) if isinstance(data, Mapping) else {}


def _save_gui_settings(settings: Mapping[str, Any]) -> tuple[bool, str]:
    path = _gui_settings_path()
    try:
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(dict(settings), handle, indent=2, sort_keys=True)
    except OSError as exc:
        return False, str(exc)
    return True, path


def _update_gui_setting(key: str, value: Any) -> None:
    """Read-modify-write a single GUI preference. Best-effort: persistence
    failures are swallowed so a read-only home dir never breaks the toggle."""
    settings = _load_gui_settings()
    settings[key] = value
    _save_gui_settings(settings)


def _user_floor_contact_point(latest: StateSnapshot | None, arm: str) -> tuple[float, float, float] | None:
    """The arm's lowest floor-contact point (xyz, stand frame) from floor telemetry.

    Uses the MEASURED actual TCP (tcp_actual_stand), which tracks hand-guided
    (direct-teaching / freedrive) motion. The floor's lowest_point_m is intentionally
    NOT used: it is derived from the held servo target and goes stale during freedrive
    (the servo loop stops commanding), so every capture would read the same point.
    Because this captures the TCP ORIGIN, the user floor enforces on the TCP origin
    (no gripper-tip offsets) so capture and enforcement live in the same space: the
    fitted plane is the TCP-origin locus when the tip touches the floor, and keeping
    the TCP origin above it keeps the tip above the real floor."""
    if latest is None:
        return None
    arm_snap = getattr(latest, arm, None)
    pose = getattr(arm_snap, "tcp_actual_stand", None) or getattr(arm_snap, "tcp_stand", None)
    if pose is not None and all(math.isfinite(v) for v in (pose.x, pose.y, pose.z)):
        return (float(pose.x), float(pose.y), float(pose.z))
    return None


def _capture_user_floor_point(
    handles: dict[str, Any], store: StateStore, arm: str
) -> tuple[bool, str]:
    latest = store.latest()
    if latest is None or store.is_stale():
        return False, "state stream missing or stale; cannot capture"
    point = _user_floor_contact_point(latest, arm)
    if point is None:
        return False, f"no {arm} contact point available (FK/floor telemetry missing)"
    state = _user_floor_state(handles)
    state["points"].append({"arm": arm, "p": [float(point[0]), float(point[1]), float(point[2])]})
    counts = _user_floor_point_counts(state)
    # Show the running (x, y) bounding-box span so the operator immediately sees
    # whether the captures are actually spreading out (a fit needs spatial spread;
    # near-zero span here means the captured points are duplicates).
    span = _user_floor_xy_span(state)
    return True, (
        f"captured {arm} pt @[{point[0] * 1000:.0f},{point[1] * 1000:.0f},{point[2] * 1000:.0f}]mm "
        f"({counts}, xy-span {span[0] * 1000:.0f}x{span[1] * 1000:.0f}mm)"
    )


def _user_floor_point_counts(state: Mapping[str, Any]) -> str:
    pts = state.get("points", [])
    left = sum(1 for e in pts if e.get("arm") == "left")
    right = sum(1 for e in pts if e.get("arm") == "right")
    return f"{len(pts)} pts: L{left} R{right}"


def _user_floor_xy_span(state: Mapping[str, Any]) -> tuple[float, float]:
    """(x_extent, y_extent) in meters of the captured points' bounding box."""
    pts = [e["p"] for e in state.get("points", []) if isinstance(e.get("p"), (list, tuple))]
    if not pts:
        return (0.0, 0.0)
    xs = [float(p[0]) for p in pts]
    ys = [float(p[1]) for p in pts]
    return (max(xs) - min(xs), max(ys) - min(ys))


def _fit_user_floor_plane(handles: dict[str, Any]) -> tuple[bool, str]:
    """Fit a plane through the captured points and store it on the GUI state.
    Does not send anything to the server — that is the Fit & Apply / Enforce step."""
    state = _user_floor_state(handles)
    pts = [e["p"] for e in state.get("points", [])]
    if len(pts) < 3:
        return False, f"need >= 3 points to fit ({_user_floor_point_counts(state)})"
    try:
        point, normal = fit_plane(pts)
    except ValueError as exc:
        return False, f"fit failed: {exc}"
    state["plane"] = {"point": list(point), "normal": list(normal)}
    return True, f"fit plane tilt={tilt_deg(normal):.1f}° from {_user_floor_point_counts(state)}"


def _init_motion_path() -> str:
    raw = os.environ.get("RB_GUI_INIT_MOTION_PATH", "").strip()
    if raw:
        return raw
    return os.path.join(os.path.expanduser("~"), ".rb_servo_gui", "init_motion.json")


def _load_init_joints() -> tuple[tuple[float, ...] | None, tuple[float, ...] | None]:
    path = _init_motion_path()
    try:
        with open(path, encoding="utf-8") as handle:
            data = json.load(handle)
    except FileNotFoundError:
        return None, None
    except (OSError, json.JSONDecodeError):
        return None, None
    if not isinstance(data, Mapping):
        return None, None
    return _waypoint_seq(data.get("left"), 6), _waypoint_seq(data.get("right"), 6)


def _save_init_joints(left_q: tuple[float, ...], right_q: tuple[float, ...]) -> tuple[bool, str]:
    path = _init_motion_path()
    try:
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(path, "w", encoding="utf-8") as handle:
            json.dump({"left": list(left_q), "right": list(right_q)}, handle, indent=2)
    except OSError as exc:
        return False, str(exc)
    return True, path


def _format_joint6(values: tuple[float, ...] | None) -> str:
    if values is None:
        return ""
    return ", ".join(f"{float(v):.3f}" for v in values)


def _parse_joint6(text: str) -> tuple[float, ...] | None:
    parts = [p for p in text.replace(",", " ").split() if p]
    if len(parts) != 6:
        return None
    try:
        parsed = tuple(float(p) for p in parts)
    except ValueError:
        return None
    if not all(math.isfinite(v) for v in parsed):
        return None
    return parsed


def _apply_init_joints_live(
    safety: OperatorSafety, left_text: str, right_text: str
) -> tuple[bool, str]:
    """Set the InitMotion target from edited text and apply it at runtime.

    Updates the live OperatorSafety target (used immediately by the next
    InitMotion press) AND persists to init_motion.json — no restart needed.
    """
    left = _parse_joint6(left_text)
    right = _parse_joint6(right_text)
    if left is None or right is None:
        return False, "each arm needs 6 finite joint values (deg), comma/space separated"
    ok, message = safety.set_init_joints(left, right)
    if not ok:
        return False, message
    saved_ok, info = _save_init_joints(left, right)
    suffix = f"saved to {info}" if saved_ok else f"save FAILED: {info}"
    return True, f"InitMotion updated live; {suffix}"


def _current_joints_text(store: StateStore) -> tuple[str | None, str | None, str]:
    """Read current q_actual for both arms as editable text. Returns (left, right, msg)."""
    latest = store.latest()
    if latest is None or store.is_stale():
        return None, None, "state stream missing or stale; cannot read current pose"
    left_q = getattr(latest.left, "q_actual_deg", None)
    right_q = getattr(latest.right, "q_actual_deg", None)
    if not left_q or not right_q:
        return None, None, "current joints unavailable (arm not connected / no valid state)"
    return _format_joint6(tuple(left_q)), _format_joint6(tuple(right_q)), "loaded current pose"


def _set_waypoint_as_init(handles: dict[str, Any], safety: OperatorSafety) -> tuple[bool, str]:
    name, waypoint = _selected_waypoint(handles)
    if waypoint is None:
        return False, "select a captured waypoint first"
    left_q = waypoint.get("left_q")
    right_q = waypoint.get("right_q")
    if left_q is None or right_q is None:
        return False, f"'{name}' missing joint capture for one arm"
    ok, message = safety.set_init_joints(left_q, right_q)
    if not ok:
        return False, message
    saved_ok, info = _save_init_joints(left_q, right_q)
    suffix = f"saved to {info}" if saved_ok else f"save failed: {info}"
    return True, f"init motion set to '{name}'; {suffix}"


def _capture_waypoint(handles: dict[str, Any], store: StateStore, name: str) -> tuple[bool, str]:
    name = (name or "").strip()
    if not name:
        return False, "enter a waypoint name first"
    latest = store.latest()
    if latest is None or store.is_stale():
        return False, "state stream missing or stale; cannot capture"

    def _arm_capture(arm_snap: Any) -> tuple[Any, Any, Any]:
        q = arm_snap.q_actual_deg
        pose = arm_snap.tcp_actual_stand or arm_snap.tcp_stand
        pose6 = None
        quat = None
        if pose is not None:
            pose6 = (pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz)
            quat = pose.quaternion_xyzw
        return q, pose6, quat

    left_q, left_pose, left_quat = _arm_capture(latest.left)
    right_q, right_pose, right_quat = _arm_capture(latest.right)
    handles.setdefault("waypoints", {})[name] = {
        "left_q": left_q,
        "left_pose": left_pose,
        "left_quat": left_quat,
        "right_q": right_q,
        "right_pose": right_pose,
        "right_quat": right_quat,
    }
    return True, (
        f"captured '{name}' "
        f"(L joints {'ok' if left_q else 'none'}, pose {'ok' if left_pose else 'none'}; "
        f"R joints {'ok' if right_q else 'none'}, pose {'ok' if right_pose else 'none'})"
    )


def _refresh_waypoint_dropdown(handles: dict[str, Any]) -> None:
    dropdown = handles.get("waypoint_dropdown")
    if dropdown is None:
        return
    names = tuple(handles.get("waypoints", {}).keys()) or ("(none)",)
    current = dropdown.value
    try:
        dropdown.options = names
        if current not in names:
            dropdown.value = names[0]
    except Exception:
        pass


def _selected_waypoint(handles: dict[str, Any]) -> tuple[str | None, dict[str, Any] | None]:
    dropdown = handles.get("waypoint_dropdown")
    name = dropdown.value if dropdown is not None else None
    waypoint = handles.get("waypoints", {}).get(name)
    return name, waypoint


def _drive_waypoint_tcp(handles: dict[str, Any], safety: OperatorSafety) -> tuple[bool, str]:
    name, waypoint = _selected_waypoint(handles)
    if waypoint is None:
        return False, "select a captured waypoint first"
    left_pose = waypoint.get("left_pose")
    right_pose = waypoint.get("right_pose")
    if left_pose is None and right_pose is None:
        return False, f"'{name}' has no stand pose"
    ok, message = safety.send_tcp_pose_target(
        left_pose=left_pose,
        right_pose=right_pose,
        left_quaternion_xyzw=waypoint.get("left_quat"),
        right_quaternion_xyzw=waypoint.get("right_quat"),
    )
    return ok, f"moveL->{name}: " + ("OK: " if ok else "BLOCKED: ") + message


def _drive_waypoint_joint(handles: dict[str, Any], safety: OperatorSafety) -> tuple[bool, str]:
    name, waypoint = _selected_waypoint(handles)
    if waypoint is None:
        return False, "select a captured waypoint first"
    left_q = waypoint.get("left_q")
    right_q = waypoint.get("right_q")
    if left_q is None or right_q is None:
        return False, f"'{name}' missing joint capture for one arm"
    ok, message = safety.send_joint_target(left_q_deg=left_q, right_q_deg=right_q)
    return ok, f"moveJ->{name}: " + ("OK: " if ok else "BLOCKED: ") + message


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


def _operator_monitor_layout() -> tuple[float, float, float]:
    return (
        _env_positive_float("RB_GUI_MONITOR_WIDTH_EM", _OPERATOR_MONITOR_WIDTH_EM),
        _env_positive_float("RB_GUI_MONITOR_GAP_EM", _OPERATOR_MONITOR_GAP_EM),
        _env_positive_float("RB_GUI_MONITOR_SPLIT_EM", _OPERATOR_MONITOR_SPLIT_EM),
    )


def _operator_monitor_static_html(monitor_width_em: float, gap_em: float, split_em: float) -> str:
    return f"""
<style>
  :root {{
    --rb-monitor-gap: {gap_em:.3f}em;
    --rb-monitor-target-width: {monitor_width_em:.3f}em;
    --rb-monitor-width: min(
      var(--rb-monitor-target-width),
      max(13.5em, calc((100vw - (3 * var(--rb-monitor-gap))) / 2))
    );
    /* Pose Monitor stacks just below the Joint Monitor's natural bottom (em, in
       card font size) — clamped so it never runs off a short viewport. */
    --rb-monitor-split: min({split_em:.3f}em, 60vh);
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
    /* Extra room below the deg/rad radios comes from min-height only — NOT from
       padding-bottom. Padding counts toward content height, and if content grows
       past min-height the header card overflows and overlaps the body card,
       stacking two 0.96-alpha backgrounds into a brighter seam in the gap. */
    min-height: 4.95em;
    padding: 0.65em 0.8em 0.55em;
    border-radius: 0.45em 0.45em 0 0;
    border-bottom: 0;
  }}
  .rb-monitor-body-card {{
    top: 5.95em;
    max-height: calc(100vh - 6.95em);
    overflow: auto;
    padding: 0.6em 0.8em 0.75em;
    border-radius: 0 0 0.45em 0.45em;
  }}
  /* Joint Monitor (top half) and Pose Monitor (bottom half) stack in the same
     left column. Viewport-half split keeps the deg/rad radios (which live in the
     static header cards) stable across dynamic body refreshes. */
  .rb-monitor-joint-card {{ left: var(--rb-monitor-gap); }}
  .rb-monitor-stand-card {{ left: var(--rb-monitor-gap); }}
  .rb-monitor-joint-card.rb-monitor-body-card {{ max-height: calc(var(--rb-monitor-split) - 5.95em); }}
  .rb-monitor-stand-card.rb-monitor-header-card {{ top: var(--rb-monitor-split); }}
  .rb-monitor-stand-card.rb-monitor-body-card {{ top: calc(var(--rb-monitor-split) + 4.95em); max-height: calc(100vh - var(--rb-monitor-split) - 5.95em); }}
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
    grid-template-columns: minmax(6em, 1fr) auto;
    column-gap: 0.8em;
    line-height: 1.55;
    white-space: nowrap;
  }}
  /* Label cell clips itself (ellipsis) instead of overflowing onto the value, and the
     value is right-aligned, so the angle (incl. a leading "-" sign) is always visible. */
  .rb-monitor-row > span:first-child {{ overflow: hidden; text-overflow: ellipsis; min-width: 0; }}
  .rb-monitor-row > span:last-child {{ text-align: right; font-variant-numeric: tabular-nums; }}
  .rb-rad {{ display: none; }}
  body:has(#rb-joint-unit-rad:checked) .rb-monitor-joint-card .rb-deg {{ display: none; }}
  body:has(#rb-joint-unit-rad:checked) .rb-monitor-joint-card .rb-rad {{ display: inline; }}
  body:has(#rb-stand-unit-rad:checked) .rb-monitor-stand-card .rb-deg {{ display: none; }}
  body:has(#rb-stand-unit-rad:checked) .rb-monitor-stand-card .rb-rad {{ display: inline; }}
  @media (max-width: 960px) {{
    .rb-monitor-card {{ font-size: 11px; }}
    .rb-monitor-header-card {{
      min-height: 4.5em;
      padding: 0.55em 0.65em 0.45em;
    }}
    .rb-monitor-body-card {{
      top: 5.5em;
      max-height: calc(100vh - 6.5em);
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


def _arm_is_controller_sim(arm_state: Any) -> bool:
    """True for rbpodo controller (pgmode) simulation, where q_actual does not track
    the streamed servo_j so the reference is the meaningful signal. False for
    software simulation, physical real, and unknown."""
    if arm_state is None:
        return False
    csm = getattr(arm_state, "controller_simulation_mode", None)
    if not csm:
        return False
    try:
        operation_mode = str(csm.get("operation_mode", "")).lower()
    except AttributeError:
        return False
    return operation_mode in ("simulation", "sim")


def _stand_world_monitor_pose(arm_state: Any, *, stale: bool) -> tuple[Pose6D | None, bool, bool]:
    """Pose, validity, and is-controller-sim shown in the Stand/World (Pose) Monitor
    for one arm. Controller (pgmode) simulation shows the commanded TCP (FK of
    q_sent) because q_actual does not track the command and jnt_ref-derived
    tcp_ref is noisy at rest; otherwise the actual TCP (`tcp_stand`).

    The TCP PTP "current pose" mirror reuses this SAME selection so its numbers
    are identical to the Pose Monitor (including the server's `rx/ry/rz` euler,
    which differs from a quaternion round-trip — see se3.cpp eulerAngles(2,1,0))."""
    if arm_state is None:
        return None, False, False
    if _arm_is_controller_sim(arm_state) and arm_state.tcp_command_stand is not None:
        return arm_state.tcp_command_stand, (not stale), True
    valid = bool(
        not stale
        and arm_state.has_valid_tcp_pose
        and arm_state.tcp_stand is not None
        and not arm_state.tcp_deferred
    )
    return arm_state.tcp_stand, valid, False


def _render_joint_monitor_rows(
    latest: StateSnapshot | None, *, stale: bool, uptime: str | None = None
) -> str:
    if latest is None:
        status = "No state stream"
        arms = (("left", None), ("right", None))
    else:
        status = f"{'stale' if stale else 'live'}, tick={latest.tick}"
        if uptime:
            status += f", up={uptime}"
        arms = (("left", latest.left), ("right", latest.right))
    parts = [f'<div class="rb-monitor-status">{escape(status)}</div>']
    for arm, arm_state in arms:
        # In controller (pgmode) simulation q_actual does not track the command, so
        # show the commanded joints (q_sent). q_sent is the clean signal we stream;
        # the controller's jnt_ref readback (q_ref) is noisy at rest, so it is not used.
        use_ref = arm_state is not None and _arm_is_controller_sim(arm_state) and arm_state.q_sent_deg is not None
        title = f"{arm} · q_sent (controller-sim)" if use_ref else arm
        q_values = None
        valid = False
        if arm_state is not None:
            q_values = arm_state.q_sent_deg if use_ref else arm_state.q_actual_deg
            valid = (arm_state.q_sent_deg is not None) if use_ref else arm_state.has_valid_joint_state
        parts.append(f'<div class="rb-monitor-arm"><div class="rb-monitor-arm-title">{escape(title)}</div>')
        for index, joint_name in enumerate(_ROBOT_JOINT_NAMES):
            if arm_state is None:
                value_html = _operator_monitor_invalid_pair()
            else:
                value_html = _operator_monitor_value_pair(
                    _format_joint_monitor_value(q_values, index, valid=valid, unit="deg"),
                    _format_joint_monitor_value(q_values, index, valid=valid, unit="rad"),
                )
            short_name = joint_name[:-6] if joint_name.endswith("_joint") else joint_name
            parts.append(_operator_monitor_row(f"J{index + 1} {short_name}", value_html))
        parts.append("</div>")
    return "".join(parts)


def _render_stand_world_monitor_rows(
    latest: StateSnapshot | None, *, stale: bool, uptime: str | None = None
) -> str:
    if latest is None:
        status = "No state stream"
        arms = (("left", None), ("right", None))
    else:
        status = f"{'stale' if stale else 'live'}, tick={latest.tick}"
        if uptime:
            status += f", up={uptime}"
        arms = (("left", latest.left), ("right", latest.right))
    parts = [f'<div class="rb-monitor-status">{escape(status)}</div>']
    for arm, arm_state in arms:
        # Same pose selection the TCP PTP "current pose" mirror uses, so the two
        # displays never disagree (controller-sim shows the commanded TCP).
        pose, valid, use_ref = _stand_world_monitor_pose(arm_state, stale=stale)
        title = f"{arm} · tcp_command_stand (controller-sim)" if use_ref else arm
        parts.append(f'<div class="rb-monitor-arm"><div class="rb-monitor-arm-title">{escape(title)}</div>')
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


def _operator_monitor_dynamic_html(
    latest: StateSnapshot | None, *, stale: bool, uptime: str | None = None
) -> str:
    return (
        '<div class="rb-monitor-card rb-monitor-body-card rb-monitor-joint-card">'
        + _render_joint_monitor_rows(latest, stale=stale, uptime=uptime)
        + "</div>"
        + '<div class="rb-monitor-card rb-monitor-body-card rb-monitor-stand-card">'
        + _render_stand_world_monitor_rows(latest, stale=stale, uptime=uptime)
        + "</div>"
    )


def _format_hms(seconds: float) -> str:
    """seconds -> 'hh:mm:ss' (hours grow past 99 for multi-day uptimes)."""
    total = int(seconds)
    hours, rem = divmod(total, 3600)
    minutes, secs = divmod(rem, 60)
    return f"{hours:02d}:{minutes:02d}:{secs:02d}"


def _server_uptime_hms(handles: dict[str, Any], latest: StateSnapshot | None) -> str | None:
    """rb_servo_server run-time as 'hh:mm:ss', or None if it can't be derived.

    The server has no uptime field, but `tick` starts at 0 at server start and
    `loop_start_time_ns` is a steady clock, so uptime == tick * loop_period. We
    estimate the loop period from the (loop_start, tick) delta since the first
    sample this GUI saw (self-correcting as it runs); before two distinct ticks
    are seen we fall back to the published per-loop `period_ms`, then 500 Hz."""
    if latest is None:
        return None
    raw = latest.raw if isinstance(latest.raw, Mapping) else {}
    try:
        loop_ns = int(raw.get("loop_start_time_ns"))
    except (TypeError, ValueError):
        return None
    if loop_ns <= 0:
        return None
    tick = latest.tick
    anchor = handles.get("monitor_uptime_anchor")
    if not (isinstance(anchor, tuple) and len(anchor) == 2):
        anchor = (tick, loop_ns)
        handles["monitor_uptime_anchor"] = anchor
    t0, l0 = anchor
    # Server restart (tick rewound or clock reset): re-anchor on the new run.
    if tick < t0 or loop_ns < l0:
        anchor = (tick, loop_ns)
        handles["monitor_uptime_anchor"] = anchor
        t0, l0 = anchor
    if tick > t0 and loop_ns > l0:
        period_ns = (loop_ns - l0) / (tick - t0)  # measured average loop period
    else:
        try:
            period_ms = float(raw.get("period_ms"))
        except (TypeError, ValueError):
            period_ms = 0.0
        period_ns = period_ms * 1e6 if period_ms > 0.0 else 2.0e6  # else nominal 500 Hz
    uptime_sec = tick * period_ns / 1e9
    if not math.isfinite(uptime_sec) or uptime_sec < 0:
        return None
    return _format_hms(uptime_sec)


def _build_operator_monitors(server: Any, handles: dict[str, Any]) -> None:
    add_html = getattr(server.gui, "add_html", None)
    if callable(add_html):
        monitor_width_em, gap_em, split_em = _operator_monitor_layout()
        handles["operator_monitor_panel_mode"] = "fixed_html_overlay"
        handles["operator_monitor_style"] = add_html(
            _operator_monitor_static_html(monitor_width_em, gap_em, split_em),
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
    uptime = _server_uptime_hms(handles, latest)
    if content is not None:
        try:
            content.content = _operator_monitor_dynamic_html(latest, stale=stale, uptime=uptime)
            return
        except Exception:
            pass
    _update_joint_monitor(handles, latest, stale=stale)
    _update_stand_world_monitor(handles, latest, stale=stale)


# Colored at-a-glance status chips (replaces scanning the wall of Status text rows).
# (bg, text, dot) per tone.
_STATUS_TONES = {
    "ok": ("#e0f5e9", "#148a4e", "#22aa63"),
    "warn": ("#fcf3de", "#8a6410", "#e1a01e"),
    "bad": ("#fae4e4", "#b34646", "#dc4646"),
    "info": ("#dbe7fe", "#2563eb", "#2563eb"),
    "muted": ("#eef0f4", "#52606d", "#9aa4b2"),
}


def _status_chip(label: str, value: str, tone: str) -> str:
    bg, fg, dot = _STATUS_TONES.get(tone, _STATUS_TONES["muted"])
    return (
        f'<span style="display:inline-flex;align-items:center;gap:0.4em;background:{bg};'
        f'color:{fg};border-radius:0.9em;padding:0.18em 0.7em;margin:0.15em 0.3em 0.15em 0;'
        f'font-size:12px;font-weight:600;font-family:system-ui,-apple-system,sans-serif;'
        f'white-space:nowrap;"><span style="width:0.5em;height:0.5em;border-radius:50%;'
        f'background:{dot};"></span><span style="opacity:0.65;font-weight:500;">'
        f'{escape(label)}</span>{escape(value)}</span>'
    )


def _status_summary_html(
    *, connection: str, mode: str, readiness_go: bool, motion: str, fault_active: bool
) -> str:
    conn_tone = "ok" if connection == "live" else "warn" if connection == "stale" else "bad"
    chips = [
        _status_chip("연결", connection, conn_tone),
        _status_chip("모드", mode or "—", "info"),
        _status_chip("준비", "Go" if readiness_go else "No-Go", "ok" if readiness_go else "bad"),
        _status_chip("상태", motion or "unknown", "muted"),
        _status_chip("결함", "없음" if not fault_active else "FAULT", "ok" if not fault_active else "bad"),
    ]
    return '<div style="display:flex;flex-wrap:wrap;padding:0.25em 0 0.1em;">' + "".join(chips) + "</div>"


def _tab_theme_html(dark: bool = True) -> str:
    """Color-code the tab levels so the main tab bar (상태/조작/이동/고급) and the
    nested sub-tab bars (관절/속도/… inside 이동, etc.) are visually distinct.
    Main = blue underline; sub = a purple tinted band (sub bars live inside a
    `.mantine-Tabs-panel`, which is how we target only the nested level).

    Two palettes: viser's built-in ``dark_mode`` only restyles the panel chrome,
    so the custom tab accents need a dark variant to stay legible on the dark
    surface (brighter blue/purple, translucent band) vs. the original light one."""
    if dark:
        main_border, main_active = "#2a313c", "#5b8cff"
        sub_bg, sub_border = "rgba(160,123,255,0.12)", "rgba(160,123,255,0.30)"
        sub_text, sub_active = "#9aa3b2", "#a07bff"
    else:
        main_border, main_active = "#d4def0", "#2563eb"
        sub_bg, sub_border = "#f4f1fb", "#e2d9f6"
        sub_text, sub_active = "#6f6788", "#7c3aed"
    return f"""
<style>
  /* Main (top-level) tab bar — blue accent */
  .mantine-Tabs-list {{ border-bottom: 2px solid {main_border} !important; }}
  .mantine-Tabs-tab[data-active] {{ color: {main_active} !important; border-color: {main_active} !important; font-weight: 700 !important; }}
  /* Sub (nested) tab bar — purple band; .mantine-Tabs-panel scopes it to nested groups only */
  .mantine-Tabs-panel .mantine-Tabs-list {{ background: {sub_bg} !important; border-radius: 7px !important; padding: 3px !important; border-bottom: 2px solid {sub_border} !important; }}
  .mantine-Tabs-panel .mantine-Tabs-tab {{ color: {sub_text} !important; }}
  .mantine-Tabs-panel .mantine-Tabs-tab[data-active] {{ color: {sub_active} !important; border-color: {sub_active} !important; font-weight: 700 !important; }}
</style>
"""


_GUI_BRAND_COLOR = (96, 200, 140)


def _apply_gui_theme(server: Any, *, dark: bool) -> None:
    """Push viser's built-in panel theme (panel chrome + 3D canvas background)."""
    configure = getattr(server.gui, "configure_theme", None)
    if not callable(configure):
        return
    try:
        configure(dark_mode=dark, brand_color=_GUI_BRAND_COLOR)
    except Exception:
        pass


def _set_dark_mode(server: Any, handles: dict[str, Any], dark: bool) -> None:
    """Apply dark/light across both viser's theme and the custom tab CSS."""
    _apply_gui_theme(server, dark=dark)
    tab_theme = handles.get("tab_theme")
    if tab_theme is not None:
        try:
            tab_theme.content = _tab_theme_html(dark=dark)
        except Exception:
            pass


def build_gui(
    server: Any,
    safety: OperatorSafety,
    store: StateStore,
    overlay_store: CircleOverlayStore | None = None,
) -> dict[str, Any]:
    handles: dict[str, Any] = {}
    handles["circle_overlay_enabled"] = overlay_store is not None
    # Dark mode is the default; set RB_GUI_DARK_MODE=0 to launch in light mode.
    dark_default = os.environ.get("RB_GUI_DARK_MODE", "1") != "0"
    handles["dark_mode"] = dark_default
    _apply_gui_theme(server, dark=dark_default)
    handles["scene"] = _add_scene_fallback(server)
    _install_default_camera(server)

    _build_operator_monitors(server, handles)

    _add_tab_theme = getattr(server.gui, "add_html", None)
    if callable(_add_tab_theme):
        handles["tab_theme"] = _add_tab_theme(_tab_theme_html(dark=dark_default))

    tabs = server.gui.add_tab_group(order=1.0)
    with tabs.add_tab("상태"):
        # Dark/light theme toggle (default dark). Re-applies viser's panel theme
        # and the custom tab CSS live so the operator can flip it without restart.
        if hasattr(server.gui, "add_checkbox"):
            handles["dark_mode_toggle"] = server.gui.add_checkbox(
                "🌙 다크 모드", initial_value=dark_default
            )

            @handles["dark_mode_toggle"].on_update
            def _(_: Any) -> None:
                dark = bool(handles["dark_mode_toggle"].value)
                handles["dark_mode"] = dark
                _set_dark_mode(server, handles, dark)

        # Colored at-a-glance summary above the detailed rows (no scanning needed).
        _add_status_html = getattr(server.gui, "add_html", None)
        if callable(_add_status_html):
            handles["status_summary"] = _add_status_html(
                _status_summary_html(
                    connection="disconnected",
                    mode=safety.observed_server_mode,
                    readiness_go=False,
                    motion="unknown",
                    fault_active=False,
                )
            )
        handles["connection"] = server.gui.add_text("Connection", initial_value="disconnected", disabled=True)
        handles["mode"] = server.gui.add_text("Observed mode/backend", initial_value=f"{safety.observed_server_mode}/{safety.observed_backend}", disabled=True)
        handles["readiness"] = server.gui.add_text("Readiness", initial_value="No-Go: no state", disabled=True)
        handles["motion"] = server.gui.add_text("Motion state", initial_value="unknown", disabled=True)
        handles["fault"] = server.gui.add_text("Fault", initial_value="none", disabled=True)
        handles["self_collision"] = server.gui.add_text(
            "Self-collision", initial_value="self-collision: no state", disabled=True
        )
        # Self-collision "check view": the translucent collision-hull overlay (the
        # EXACT unified-URDF collision geometry the async monitor checks, built from
        # the server's geometry manifest) + the close-call witness segments from
        # self_collision.near_pairs. Off by default; turn on to debug clearance.
        if hasattr(server.gui, "add_checkbox"):
            capsules_default = os.environ.get("RB_GUI_SELF_COLLISION_CAPSULES_DEFAULT", "0") == "1"
            handles["self_collision_capsules_toggle"] = server.gui.add_checkbox(
                "자기충돌 검사 표시 (반투명)", initial_value=capsules_default
            )
        # Floor/ROI collision check points: the TCP point + 4 gripper-tip offset
        # points (safety.floor_constraint.tcp_offset_points) the server samples
        # against the floor plane, shown as orange dots riding the live TCP pose.
        # Off by default; turn on to see exactly what the floor guard checks.
        if hasattr(server.gui, "add_checkbox"):
            floor_points_default = os.environ.get("RB_GUI_FLOOR_CHECK_POINTS_DEFAULT", "0") == "1"
            handles["floor_check_points_toggle"] = server.gui.add_checkbox(
                "바닥 충돌 검사점 표시 (TCP+팁 4점, 주황)", initial_value=floor_points_default
            )
        handles["floor_constraint"] = server.gui.add_text(
            "Stand Safety floor", initial_value="floor: no state", disabled=True
        )
        handles["roi_box"] = server.gui.add_text(
            "Safety ROI", initial_value="roi: no state", disabled=True
        )
        handles["user_floor_constraint"] = server.gui.add_text(
            "User Safety floor", initial_value="user floor: no state", disabled=True
        )
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

    with tabs.add_tab("조작"):
        _op_tabs = server.gui.add_tab_group()
        with _op_tabs.add_tab("Lifecycle"):
            handles["lifecycle_buttons"] = {}
            for mode in ("ArmMotion", "DisarmMotion", "Hold", "EmergencyStop", "ResetFault"):
                button = server.gui.add_button(mode)
                handles[f"button_{mode}"] = button
                handles["lifecycle_buttons"][mode] = button

                @button.on_click
                def _(_: Any, mode: str = mode) -> None:
                    ok, message = safety.send_lifecycle(mode)
                    handles["last_action"].value = ("OK: " if ok else "BLOCKED: ") + message

            # Explicit lease ownership. One-shot GUI commands bracket the lease per
            # click; streaming controls (twist/circle/joint-vel keep-alive) need it
            # held so the server lease is not torn down between packets. The server
            # rejects an Acquire while another source (e.g. policy_runner teleop)
            # owns it — the lease-owner line below makes that visible.
            # Collapsed by default, like 직접교시 below: holding the lease takes
            # control authority away from other command sources, so it stays
            # folded to avoid accidental toggles.
            with server.gui.add_folder("제어권 (Lease)", expand_by_default=False):
                handles["lease_owner_status"] = server.gui.add_text(
                    "Lease owner", initial_value="unknown", disabled=True
                )
                take_control = server.gui.add_checkbox("Take control (hold lease)", initial_value=False)
                handles["take_control_toggle"] = take_control

                @take_control.on_update
                def _(_: Any) -> None:
                    if take_control.value:
                        safety.command_client.acquire_lease()
                        handles["last_action"].value = "OK: lease held (Take control ON)"
                    else:
                        safety.command_client.release_lease()
                        handles["last_action"].value = "OK: lease released (Take control OFF)"

            # Per-arm direct teaching (free-drive). Releases servo_j authority on
            # the chosen arm's controller so it can be hand-guided, then re-acquires
            # it with a target resync — without tearing down `make run`. Requires
            # servo.allow_freedrive on the server (fail-closed). The other arm holds
            # at its last controller reference while one arm is hand-guided.
            # Collapsed by default: direct teaching hands control of an arm to a
            # human guide, so it stays folded to avoid accidental toggles.
            with server.gui.add_folder("직접교시 (Direct Teaching)", expand_by_default=False):
                handles["freedrive_status"] = server.gui.add_text(
                    "Freedrive", initial_value="off (no state)", disabled=True
                )

                def _freedrive(left: bool | None, right: bool | None) -> None:
                    ok, message = safety.send_freedrive(left=left, right=right)
                    handles["last_action"].value = ("OK: " if ok else "BLOCKED: ") + message

                fd_left_on = server.gui.add_button("왼팔 교시 ON")
                fd_left_off = server.gui.add_button("왼팔 교시 OFF (재동기화)")
                fd_right_on = server.gui.add_button("오른팔 교시 ON")
                fd_right_off = server.gui.add_button("오른팔 교시 OFF (재동기화)")
                fd_both_on = server.gui.add_button("양팔 교시 ON")
                fd_both_off = server.gui.add_button("양팔 교시 OFF (재동기화)")
                handles["freedrive_buttons"] = {
                    "left_on": fd_left_on,
                    "left_off": fd_left_off,
                    "right_on": fd_right_on,
                    "right_off": fd_right_off,
                    "both_on": fd_both_on,
                    "both_off": fd_both_off,
                }

                @fd_left_on.on_click
                def _(_: Any) -> None:
                    _freedrive(left=True, right=None)

                @fd_left_off.on_click
                def _(_: Any) -> None:
                    _freedrive(left=False, right=None)

                @fd_right_on.on_click
                def _(_: Any) -> None:
                    _freedrive(left=None, right=True)

                @fd_right_off.on_click
                def _(_: Any) -> None:
                    _freedrive(left=None, right=False)

                @fd_both_on.on_click
                def _(_: Any) -> None:
                    _freedrive(left=True, right=True)

                @fd_both_off.on_click
                def _(_: Any) -> None:
                    _freedrive(left=False, right=False)

            init_button = server.gui.add_button("InitMotion")
            handles["init_motion_button"] = init_button

            @init_button.on_click
            def _(_: Any) -> None:
                ok, message = _send_init_motion_and_reset_targets(safety, handles["scene"])
                handles["last_action"].value = ("OK: " if ok else "BLOCKED: ") + message

            # Edit the InitMotion target directly in viser and apply it live (no
            # restart): set_init_joints updates the runtime target used by the next
            # InitMotion press, and _save_init_joints persists it to init_motion.json.
            with server.gui.add_folder("InitMotion 편집 (즉시 적용)"):
                init_left_input = server.gui.add_text(
                    "left J1..J6 (deg)", initial_value=_format_joint6(safety.init_left_joint_deg)
                )
                init_right_input = server.gui.add_text(
                    "right J1..J6 (deg)", initial_value=_format_joint6(safety.init_right_joint_deg)
                )
                # Exposed in handles so other tabs (e.g. the WayPoint "set as init"
                # button) can mirror the live InitMotion target back into these
                # editor boxes — otherwise they keep showing the build-time value
                # and the update looks like it never happened.
                handles["init_left_input"] = init_left_input
                handles["init_right_input"] = init_right_input
                init_edit_status = server.gui.add_text(
                    "InitMotion edit status", initial_value="edit + Apply, or load current pose", disabled=True
                )
                load_current_button = server.gui.add_button("현재 자세 불러오기")
                apply_init_button = server.gui.add_button("InitMotion 적용 (즉시)")

                @load_current_button.on_click
                def _(_: Any) -> None:
                    left_text, right_text, message = _current_joints_text(store)
                    if left_text is not None and right_text is not None:
                        init_left_input.value = left_text
                        init_right_input.value = right_text
                    init_edit_status.value = message

                @apply_init_button.on_click
                def _(_: Any) -> None:
                    ok, message = _apply_init_joints_live(
                        safety, init_left_input.value, init_right_input.value
                    )
                    init_edit_status.value = ("OK: " if ok else "BLOCKED: ") + message
                    handles["last_action"].value = ("OK: " if ok else "BLOCKED: ") + message

            handles["last_action"] = server.gui.add_text("Last action", initial_value="none", disabled=True)

        with _op_tabs.add_tab("웨이포인트"):
            # Waypoints persist to JSON (RB_GUI_WAYPOINTS_PATH) and auto-load here.
            handles["waypoints"] = _load_waypoints()
            handles["waypoint_note"] = server.gui.add_text(
                "WayPoint",
                initial_value=(
                    "Move the robot (TCP PTP/Linear, sim or real), then capture both arms' "
                    "joints + stand-frame pose as a teaching point. Hold moveL/moveJ to drive there."
                ),
                disabled=True,
            )
            handles["waypoint_status"] = server.gui.add_text(
                "WayPoint status",
                initial_value=f"loaded {len(handles['waypoints'])} waypoint(s) from {_waypoints_path()}",
                disabled=True,
            )
            waypoint_name_input = server.gui.add_text("WayPoint name", initial_value="wp1")
            capture_button = server.gui.add_button("현재 값 가져오기")
            waypoint_dropdown = server.gui.add_dropdown("Saved waypoints", ("(none)",), initial_value="(none)")
            handles["waypoint_dropdown"] = waypoint_dropdown
            _refresh_waypoint_dropdown(handles)

            @capture_button.on_click
            def _(_: Any) -> None:
                ok, message = _capture_waypoint(handles, store, waypoint_name_input.value)
                if ok:
                    _refresh_waypoint_dropdown(handles)
                    message = f"{message}; {_persist_waypoints(handles)}"
                handles["waypoint_status"].value = message

            # moveL / moveJ are press-and-hold (deadman): on_hold re-sends the target
            # while held, and releasing stops re-sending so the command goes stale
            # (command_timeout_sec) and the server holds in place within a fraction
            # of a second. moveL -> TcpPoseTarget, moveJ -> JointTarget, both arms.
            movel_button = server.gui.add_button("moveL (hold)")
            movej_button = server.gui.add_button("moveJ (hold)")
            handles["waypoint_move_buttons"] = [movel_button, movej_button]

            @movel_button.on_hold(callback_hz=10.0)
            def _(_: Any) -> None:
                _, message = _drive_waypoint_tcp(handles, safety)
                handles["waypoint_status"].value = message

            @movej_button.on_hold(callback_hz=10.0)
            def _(_: Any) -> None:
                _, message = _drive_waypoint_joint(handles, safety)
                handles["waypoint_status"].value = message

            # Saved-waypoints operations on the selected entry: set as the InitMotion
            # pose (persisted to JSON) and delete (kept at the bottom).
            set_init_button = server.gui.add_button("Init Motion 으로 설정하기")
            delete_button = server.gui.add_button("WayPoint 삭제", color="red")

            @set_init_button.on_click
            def _(_: Any) -> None:
                ok, message = _set_waypoint_as_init(handles, safety)
                if ok:
                    # Mirror the new target into the InitMotion editor boxes so the
                    # change is visible there (they are populated once at build time
                    # and would otherwise keep showing the previous pose).
                    left_input = handles.get("init_left_input")
                    right_input = handles.get("init_right_input")
                    if left_input is not None:
                        left_input.value = _format_joint6(safety.init_left_joint_deg)
                    if right_input is not None:
                        right_input.value = _format_joint6(safety.init_right_joint_deg)
                handles["waypoint_status"].value = message

            @delete_button.on_click
            def _(_: Any) -> None:
                ok, info = _delete_waypoint(handles)
                if ok:
                    _refresh_waypoint_dropdown(handles)
                    handles["waypoint_status"].value = f"deleted '{info}'; {_persist_waypoints(handles)}"
                else:
                    handles["waypoint_status"].value = info

        with _op_tabs.add_tab("안전"):
            with server.gui.add_folder("Stand Safety Floor"):
                # Runtime enforce on/off. The operator's last choice is persisted to
                # the GUI settings file and restored here as the checkbox default; once
                # the state stream is live it is re-applied to the server (see the
                # one-shot block in update_gui) so the saved choice wins over
                # the config default. With no saved preference we fall back to syncing
                # from telemetry on the first state (see _update_floor_panel). Only
                # effective when floor_constraint.enable=true in the server config.
                _floor_pref = _load_gui_settings().get("stand_floor_enforce")
                _floor_pref = _floor_pref if isinstance(_floor_pref, bool) else None
                handles["stand_floor_enforce_pref"] = _floor_pref
                if hasattr(server.gui, "add_checkbox"):
                    floor_enforce = server.gui.add_checkbox(
                        "Enforce stand floor",
                        initial_value=_floor_pref if _floor_pref is not None else True,
                    )
                    handles["floor_enforce_toggle"] = floor_enforce

                    @floor_enforce.on_update
                    def _(_: Any) -> None:
                        enabled = bool(floor_enforce.value)
                        ok, message = safety.send_set_floor_enabled(enabled)
                        # Remember the operator's choice for the next launch (and so the
                        # restore block stops fighting telemetry from here on).
                        handles["stand_floor_enforce_pref"] = enabled
                        _update_gui_setting("stand_floor_enforce", enabled)
                        handles["floor_set_status"].value = ("OK: " if ok else "BLOCKED: ") + message
                handles["floor_applied"] = server.gui.add_text(
                    "Applied z", initial_value="no state", disabled=True
                )
                # Draggable slider + compact integrated number box (viser slider
                # renders both side by side).
                # min spans below the stand origin (down to -200 mm) so the floor
                # can be lowered under z=0; the actual allowed range is governed by
                # the server's [runtime_min_z_m, runtime_max_z_m] and re-synced onto
                # this slider in _update_floor_panel once state arrives.
                floor_slider = server.gui.add_slider(
                    "Floor z mm", min=-200.0, max=500.0, step=0.1, initial_value=10.0
                )
                handles["floor_slider"] = floor_slider
                floor_send = server.gui.add_button("Send floor z")
                handles["floor_send_button"] = floor_send
                handles["floor_set_status"] = server.gui.add_text(
                    "Floor set status", initial_value="idle", disabled=True
                )

                @floor_send.on_click
                def _(_: Any) -> None:
                    ok, message = safety.send_set_floor_z(float(floor_slider.value) / 1000.0)
                    handles["floor_set_status"].value = ("OK: " if ok else "BLOCKED: ") + message

                @floor_slider.on_update
                def _(_: Any) -> None:
                    # Live preview while dragging: the yellow plane follows the
                    # pending slider value immediately; _update_floor_panel hides
                    # it again once the slider matches the server-applied z.
                    update_floor_plane_preview(
                        handles.get("scene", {}), float(floor_slider.value) / 1000.0
                    )

            with server.gui.add_folder("Safety ROI box"):
                # Show/hide the ROI box in the 3D scene (default ON). Independent of
                # whether the server is enforcing it — the configured region is
                # always drawable as a reference.
                if hasattr(server.gui, "add_checkbox"):
                    handles["roi_box_visible_toggle"] = server.gui.add_checkbox(
                        "ROI 영역 표시", initial_value=True
                    )
                handles["roi_applied"] = server.gui.add_text(
                    "Applied box", initial_value="no state", disabled=True
                )
                # Six per-axis bound sliders (stand frame, mm): a draggable slider
                # plus viser's compact integrated number box. _update_roi_panel
                # syncs their range to the server's runtime envelope and brings them
                # up at the applied bounds on the first state.
                _roi_axis_defaults_mm = {
                    "x": (-500.0, 500.0),
                    "y": (-1000.0, 0.0),
                    "z": (0.0, 1000.0),
                }
                for _axis in ("x", "y", "z"):
                    _lo_default, _hi_default = _roi_axis_defaults_mm[_axis]
                    handles[f"roi_{_axis}_min"] = server.gui.add_slider(
                        f"{_axis.upper()} min mm", min=-1500.0, max=1500.0, step=5.0,
                        initial_value=_lo_default,
                    )
                    handles[f"roi_{_axis}_max"] = server.gui.add_slider(
                        f"{_axis.upper()} max mm", min=-1500.0, max=1500.0, step=5.0,
                        initial_value=_hi_default,
                    )
                roi_send = server.gui.add_button("Send ROI box")
                handles["roi_send_button"] = roi_send
                handles["roi_set_status"] = server.gui.add_text(
                    "ROI set status", initial_value="idle", disabled=True
                )

                def _roi_slider_bounds() -> tuple[
                    tuple[float, float, float], tuple[float, float, float]
                ]:
                    lo = tuple(
                        float(handles[f"roi_{a}_min"].value) / 1000.0 for a in ("x", "y", "z")
                    )
                    hi = tuple(
                        float(handles[f"roi_{a}_max"].value) / 1000.0 for a in ("x", "y", "z")
                    )
                    return lo, hi  # type: ignore[return-value]

                handles["roi_slider_bounds_fn"] = _roi_slider_bounds

                @roi_send.on_click
                def _(_: Any) -> None:
                    lo, hi = _roi_slider_bounds()
                    ok, message = safety.send_set_roi_bounds(lo, hi)
                    handles["roi_set_status"].value = ("OK: " if ok else "BLOCKED: ") + message

                def _roi_preview(_: Any) -> None:
                    # Live yellow preview box while dragging any bound slider.
                    lo, hi = _roi_slider_bounds()
                    update_roi_box_preview(handles.get("scene", {}), lo, hi)

                for _axis in ("x", "y", "z"):
                    handles[f"roi_{_axis}_min"].on_update(_roi_preview)
                    handles[f"roi_{_axis}_max"].on_update(_roi_preview)

            with server.gui.add_folder("User Safety Floor"):
                # A user-defined TILTED floor plane fit from >= 3 captured floor-contact
                # points (both arms, alternating). Unlike the (horizontal) Stand Safety
                # Floor it can match a physical floor that is not level. Capture -> Fit ->
                # Enforce; both floors are independent and additive (the stricter wins).
                # State is restored from ~/.rb_servo_gui/user_floor.json on startup;
                # if it was enabled, the plane is re-sent to the server once the state
                # stream is live (see the one-shot in update_gui).
                handles["user_floor"] = _load_user_floor()
                uf_state = handles["user_floor"]
                handles["user_floor_points_text"] = server.gui.add_text(
                    "Captured", initial_value=_user_floor_point_counts(uf_state), disabled=True
                )
                handles["user_floor_plane_text"] = server.gui.add_text(
                    "Fitted plane", initial_value="none", disabled=True
                )
                handles["user_floor_set_status"] = server.gui.add_text(
                    "User floor status", initial_value="idle", disabled=True
                )
                capture_left = server.gui.add_button("Capture LEFT contact")
                capture_right = server.gui.add_button("Capture RIGHT contact")
                remove_last = server.gui.add_button("Remove last point")
                clear_points = server.gui.add_button("Clear points")
                handles["user_floor_margin_slider"] = server.gui.add_slider(
                    "Lift margin mm", min=0.0, max=50.0, step=0.5,
                    initial_value=float(uf_state.get("margin_mm", 0.0)),
                )
                fit_apply = server.gui.add_button("Fit & Apply plane")
                if hasattr(server.gui, "add_checkbox"):
                    handles["user_floor_enforce_toggle"] = server.gui.add_checkbox(
                        "Enforce user floor", initial_value=bool(uf_state.get("enabled", False))
                    )

                def _refresh_user_floor_texts() -> None:
                    state = _user_floor_state(handles)
                    handles["user_floor_points_text"].value = _user_floor_point_counts(state)
                    plane = state.get("plane")
                    if isinstance(plane, Mapping):
                        n = plane.get("normal", [0.0, 0.0, 1.0])
                        try:
                            handles["user_floor_plane_text"].value = (
                                f"tilt={tilt_deg(n):.1f}° n=["
                                + ",".join(f"{float(v):.2f}" for v in n) + "]"
                            )
                        except (ValueError, TypeError):
                            handles["user_floor_plane_text"].value = "invalid"
                    else:
                        handles["user_floor_plane_text"].value = "none"
                    update_user_floor_capture_points(handles.get("scene", {}), state.get("points", []))

                handles["user_floor_refresh_fn"] = _refresh_user_floor_texts

                @capture_left.on_click
                def _(_: Any) -> None:
                    ok, message = _capture_user_floor_point(handles, store, "left")
                    handles["user_floor_set_status"].value = ("OK: " if ok else "BLOCKED: ") + message
                    _refresh_user_floor_texts()
                    _save_user_floor(_user_floor_state(handles))

                @capture_right.on_click
                def _(_: Any) -> None:
                    ok, message = _capture_user_floor_point(handles, store, "right")
                    handles["user_floor_set_status"].value = ("OK: " if ok else "BLOCKED: ") + message
                    _refresh_user_floor_texts()
                    _save_user_floor(_user_floor_state(handles))

                @remove_last.on_click
                def _(_: Any) -> None:
                    state = _user_floor_state(handles)
                    if state["points"]:
                        state["points"].pop()
                        handles["user_floor_set_status"].value = f"removed last ({_user_floor_point_counts(state)})"
                    else:
                        handles["user_floor_set_status"].value = "no points to remove"
                    _refresh_user_floor_texts()
                    _save_user_floor(state)

                @clear_points.on_click
                def _(_: Any) -> None:
                    state = _user_floor_state(handles)
                    state["points"] = []
                    state["plane"] = None
                    handles["user_floor_set_status"].value = "cleared captured points"
                    _refresh_user_floor_texts()
                    _save_user_floor(state)

                @fit_apply.on_click
                def _(_: Any) -> None:
                    ok, message = _fit_user_floor_plane(handles)
                    if not ok:
                        handles["user_floor_set_status"].value = "BLOCKED: " + message
                        _refresh_user_floor_texts()
                        return
                    state = _user_floor_state(handles)
                    plane = state["plane"]
                    margin_m = float(handles["user_floor_margin_slider"].value) / 1000.0
                    state["margin_mm"] = float(handles["user_floor_margin_slider"].value)
                    enforce = bool(handles.get("user_floor_enforce_toggle").value) \
                        if "user_floor_enforce_toggle" in handles else True
                    ok2, send_msg = safety.send_set_user_floor_plane(
                        tuple(plane["point"]), tuple(plane["normal"]),
                        margin_m=margin_m, enable=enforce,
                    )
                    state["enabled"] = enforce and ok2
                    handles["user_floor_set_status"].value = (
                        ("OK: " if ok2 else "BLOCKED: ") + f"{message}; {send_msg}"
                    )
                    _refresh_user_floor_texts()
                    _save_user_floor(state)

                if "user_floor_enforce_toggle" in handles:
                    @handles["user_floor_enforce_toggle"].on_update
                    def _(_: Any) -> None:
                        state = _user_floor_state(handles)
                        enforce = bool(handles["user_floor_enforce_toggle"].value)
                        plane = state.get("plane")
                        if enforce and not isinstance(plane, Mapping):
                            handles["user_floor_set_status"].value = "BLOCKED: fit a plane first"
                            handles["user_floor_enforce_toggle"].value = False
                            return
                        if not enforce:
                            ok2, send_msg = safety.send_set_user_floor_plane(
                                (0.0, 0.0, 0.0), (0.0, 0.0, 1.0), margin_m=0.0, enable=False,
                            )
                        else:
                            margin_m = float(handles["user_floor_margin_slider"].value) / 1000.0
                            ok2, send_msg = safety.send_set_user_floor_plane(
                                tuple(plane["point"]), tuple(plane["normal"]),
                                margin_m=margin_m, enable=True,
                            )
                        state["enabled"] = enforce and ok2
                        handles["user_floor_set_status"].value = ("OK: " if ok2 else "BLOCKED: ") + send_msg
                        _save_user_floor(state)

                _refresh_user_floor_texts()

            with server.gui.add_folder("도달영역(reach)"):
                # Show/hide the per-arm reachable-workspace cloud (FK envelope from
                # tools/reach_envelope.py). Static geometry — purely a viewer aid for
                # seeing where each arm can actually go (the outer shell is the reach
                # limit the safety.reach_constraint damper enforces). Default OFF
                # (the cloud is dense); toggling is instant.
                reach_status = "no asset"
                scene = handles.get("scene", {})
                if isinstance(scene, dict) and (
                    "left_reach_envelope" in scene or "right_reach_envelope" in scene
                ):
                    r_max = scene.get("reach_envelope_r_max_m")
                    r_min = scene.get("reach_envelope_r_min_m")
                    reach_status = f"shell r=[{r_min:.3f}, {r_max:.3f}] m"
                elif isinstance(scene, dict) and scene.get("reach_envelope_error"):
                    reach_status = str(scene["reach_envelope_error"])
                if hasattr(server.gui, "add_checkbox"):
                    reach_toggle = server.gui.add_checkbox("도달영역 표시", initial_value=False)
                    handles["reach_envelope_visible_toggle"] = reach_toggle

                    def _reach_toggle(_: Any) -> None:
                        set_reach_envelope_visible(
                            handles.get("scene", {}), bool(reach_toggle.value)
                        )

                    reach_toggle.on_update(_reach_toggle)
                handles["reach_envelope_status"] = server.gui.add_text(
                    "Reach envelope", initial_value=reach_status, disabled=True
                )

            with server.gui.add_folder("IK 불가 영역 (특이점 원통)"):
                # Show/hide the per-arm base-axis singularity cylinder (tools/
                # ik_infeasible_region.py). Vendor "A 영역": the column along each
                # arm's J1 axis where Move J is fine but Cartesian/Move L control
                # forces runaway joint speed (radius R = v_ref/dq_max, grows with
                # commanded speed). Distinct from reach (too far) and from the old
                # top-down shell. Static viewer aid, default OFF.
                ik_status = "no asset"
                scene = handles.get("scene", {})
                if isinstance(scene, dict) and (
                    "left_ik_infeasible" in scene or "right_ik_infeasible" in scene
                ):
                    radius_m = scene.get("ik_infeasible_radius_m")
                    ik_status = (
                        f"cylinder loaded (R={radius_m*1000:.0f} mm)"
                        if isinstance(radius_m, (int, float))
                        else "cylinder loaded"
                    )
                elif isinstance(scene, dict) and scene.get("ik_infeasible_error"):
                    ik_status = str(scene["ik_infeasible_error"])
                if hasattr(server.gui, "add_checkbox"):
                    ik_toggle = server.gui.add_checkbox(
                        "IK 불가 영역(특이점 원통) 표시", initial_value=False
                    )
                    handles["ik_infeasible_visible_toggle"] = ik_toggle

                    def _ik_infeasible_toggle(_: Any) -> None:
                        set_ik_infeasible_region_visible(
                            handles.get("scene", {}), bool(ik_toggle.value)
                        )

                    ik_toggle.on_update(_ik_infeasible_toggle)
                handles["ik_infeasible_status"] = server.gui.add_text(
                    "IK 불가 영역", initial_value=ik_status, disabled=True
                )

        with _op_tabs.add_tab("그리퍼"):
            with server.gui.add_folder("그리퍼 제어"):
                # Operator gripper control. Each slider sends a gripper_cmd.v1
                # setpoint to gripper_server (UDP, default 127.0.0.1:50410), which
                # drives the sim/real gripper; its feedback (state JSON) then moves
                # the articulated-gripper viz. When gripper_server is absent the
                # slider still previews the viz directly (see _push_gripper_percent).
                # 100 = open, 0 = closed. Endpoint: RB_GUI_GRIPPER_CMD_ENDPOINT.
                if hasattr(server.gui, "add_slider"):
                    g_left = server.gui.add_slider(
                        "Left gripper %", min=0.0, max=100.0, step=1.0, initial_value=100.0
                    )
                    g_right = server.gui.add_slider(
                        "Right gripper %", min=0.0, max=100.0, step=1.0, initial_value=100.0
                    )
                    handles["gripper_slider_left"] = g_left
                    handles["gripper_slider_right"] = g_right
                    handles["gripper_cmd_endpoint"] = _gripper_cmd_endpoint()
                    handles["gripper_cmd_sock"] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    handles["gripper_cmd_seq"] = 0

                    # Each slider is BOTH a command setpoint and a live actual-%
                    # readout: _update_gripper_feedback writes the real gripper opening
                    # (server gripper_state.v1) back into the slider every frame so the
                    # slider + its number box track the hardware. _send_gripper_command
                    # suppresses the echo from those programmatic writes and only emits a
                    # gripper_cmd when the OPERATOR moves a slider; a manual move then
                    # holds (pauses auto-sync) briefly so the operator's value is not
                    # instantly overwritten before the gripper starts moving.
                    handles["gripper_manual_hold_sec"] = 1.0
                    def _on_gripper_slider(_: Any = None) -> None:
                        _send_gripper_command(handles)

                    g_left.on_update(_on_gripper_slider)
                    g_right.on_update(_on_gripper_slider)
                    handles["gripper_control_status"] = server.gui.add_text(
                        "Gripper cmd",
                        initial_value=f"-> gripper_server {handles['gripper_cmd_endpoint'][0]}:{handles['gripper_cmd_endpoint'][1]}",
                        disabled=True,
                    )

    with tabs.add_tab("이동"):
        _move_tabs = server.gui.add_tab_group()
        with _move_tabs.add_tab("관절"):
            # Per-joint [−] Jn [+] nudge rows (same direct pattern as TCP PTP):
            # one click jogs that joint by ±Step. Step slider is magnitude only;
            # the −/+ button decides the sign.
            jog_arm = server.gui.add_button_group("Arm", ("left", "right"))
            jog_step = server.gui.add_slider("Step deg", min=0.1, max=5.0, step=0.1, initial_value=0.5)
            handles["jog_status"] = server.gui.add_text("Jog status", initial_value="idle", disabled=True)

            def _jog_joint_nudge(joint_index: int, sign: float) -> None:
                ok, message = safety.jog_joint(jog_arm.value, joint_index, sign * float(jog_step.value))
                handles["jog_status"].value = ("OK: " if ok else "BLOCKED: ") + message

            def _add_joint_jog_row(joint_index: int) -> None:
                group = server.gui.add_button_group("", ("-", _nudge_label(f"J{joint_index + 1}"), "+"))

                @group.on_click
                def _(_: Any, group: Any = group, joint_index: int = joint_index) -> None:
                    if group.value == "-":
                        _jog_joint_nudge(joint_index, -1.0)
                    elif group.value == "+":
                        _jog_joint_nudge(joint_index, 1.0)

            for _joint_index in range(6):
                _add_joint_jog_row(_joint_index)

        with _move_tabs.add_tab("속도"):
            # Two clearly-separated modes (were crammed in one tab). Each DoF has a
            # [−] label [+] row; the slider sets magnitude only. A click STARTS (or
            # switches) a continuous jog: a keep-alive thread re-sends the selected
            # velocity every 0.1 s (< the server's 0.2 s command timeout) while
            # holding the command lease, so the arm moves continuously until 정지 —
            # the same streaming model as the Circle tab, instead of a one-click
            # ~200 ms burst that was choppy for fine manipulation.
            vel_arm = server.gui.add_button_group("Arm", ("left", "right"))
            handles["velocity_status"] = server.gui.add_text("Velocity status", initial_value="idle", disabled=True)

            def _set_vel_status(ok: bool, message: str) -> None:
                handles["velocity_status"].value = ("OK: " if ok else "BLOCKED: ") + message

            # Shared continuous-jog controller for BOTH folders below (joint velocity
            # and TCP twist are mutually exclusive — only one motion at a time). The
            # running loop reads the current command from a single dict key (atomic
            # under the GIL), so switching direction/axis just updates it without
            # tearing down the thread or churning the lease. Per-thread stop event +
            # join-before-restart avoids the start/stop lifecycle race.
            vel_jog: dict[str, Any] = {"thread": None, "stop": None, "cmd": None}
            vel_jog_lock = threading.Lock()

            def _vel_jog_loop(stop_event: threading.Event) -> None:
                client = safety.command_client
                took_lease = not client.hold_lease
                if took_lease:
                    client.acquire_lease()
                last_target: tuple[Any, str] | None = None
                try:
                    while not stop_event.is_set():
                        cmd = vel_jog["cmd"]  # single atomic read
                        if cmd is None:
                            break
                        sender, arm, vec, label = cmd
                        last_target = (sender, arm)
                        ok, message = sender(arm, vec)
                        _set_vel_status(ok, f"{label}: {message}")
                        if not ok:  # gate closed / limit: stop streaming, fail-safe
                            break
                        stop_event.wait(0.1)
                    if last_target is not None:  # best-effort zero on stop
                        sender, arm = last_target
                        try:
                            sender(arm, (0.0,) * 6)
                        except Exception:
                            pass
                finally:
                    if took_lease:
                        client.release_lease()

            def _vel_jog_start(sender: Any, arm: str, vec: tuple[float, ...], label: str) -> None:
                with vel_jog_lock:
                    vel_jog["cmd"] = (sender, arm, vec, label)
                    thread = vel_jog["thread"]
                    if thread is not None and thread.is_alive():
                        _set_vel_status(True, f"jogging {label}")
                        return  # the running loop picks up the new command
                    stop = threading.Event()
                    thread = threading.Thread(target=_vel_jog_loop, args=(stop,), daemon=True)
                    vel_jog["thread"], vel_jog["stop"] = thread, stop
                    thread.start()
                _set_vel_status(True, f"jogging {label} (정지로 멈춤)")

            def _vel_jog_stop() -> None:
                with vel_jog_lock:
                    vel_jog["cmd"] = None
                    stop = vel_jog["stop"]
                    if stop is not None:
                        stop.set()
                    thread = vel_jog["thread"]
                    if thread is not None and thread.is_alive():
                        thread.join(timeout=1.0)
                    vel_jog["thread"], vel_jog["stop"] = None, None
                _set_vel_status(True, "stopped")

            with server.gui.add_folder("관절 속도"):
                jv_speed = server.gui.add_slider("deg/s", min=0.5, max=10.0, step=0.5, initial_value=3.0)

                def _send_joint_vel(joint_index: int, sign: float) -> None:
                    vel = [0.0] * 6
                    vel[joint_index] = sign * float(jv_speed.value)
                    arrow = "+" if sign > 0 else "−"
                    _vel_jog_start(
                        safety.send_joint_velocity, vel_arm.value, tuple(vel),
                        f"{vel_arm.value} J{joint_index + 1}{arrow} {float(jv_speed.value):g}deg/s",
                    )

                def _add_joint_vel_row(joint_index: int) -> None:
                    group = server.gui.add_button_group("", ("-", _nudge_label(f"J{joint_index + 1}"), "+"))

                    @group.on_click
                    def _(_: Any, group: Any = group, joint_index: int = joint_index) -> None:
                        if group.value == "-":
                            _send_joint_vel(joint_index, -1.0)
                        elif group.value == "+":
                            _send_joint_vel(joint_index, 1.0)

                for _vel_joint_index in range(6):
                    _add_joint_vel_row(_vel_joint_index)
                jv_stop = server.gui.add_button("정지 (Stop)", color="red")

                @jv_stop.on_click
                def _(_: Any) -> None:
                    _vel_jog_stop()

            with server.gui.add_folder("TCP 트위스트"):
                tw_frame = server.gui.add_button_group("Frame", ("stand", "local"))
                tw_lin = server.gui.add_slider("Linear m/s", min=0.005, max=0.05, step=0.005, initial_value=0.02)
                tw_ang = server.gui.add_slider("Angular rad/s", min=0.02, max=0.2, step=0.02, initial_value=0.1)
                _twist_axes = (("X", 0, False), ("Y", 1, False), ("Z", 2, False),
                               ("Rx", 3, True), ("Ry", 4, True), ("Rz", 5, True))

                def _send_twist(label: str, axis_index: int, angular: bool, sign: float) -> None:
                    twist = [0.0] * 6
                    twist[axis_index] = sign * (float(tw_ang.value) if angular else float(tw_lin.value))
                    sender = (
                        safety.send_tcp_twist_local if tw_frame.value == "local"
                        else safety.send_tcp_twist_stand
                    )
                    arrow = "+" if sign > 0 else "−"
                    _vel_jog_start(
                        sender, vel_arm.value, tuple(twist),
                        f"{vel_arm.value} {tw_frame.value} {label}{arrow}",
                    )

                def _add_twist_row(label: str, axis_index: int, angular: bool) -> None:
                    group = server.gui.add_button_group("", ("-", _nudge_label(label), "+"))

                    @group.on_click
                    def _(_: Any, group: Any = group, label: str = label, axis_index: int = axis_index, angular: bool = angular) -> None:
                        if group.value == "-":
                            _send_twist(label, axis_index, angular, -1.0)
                        elif group.value == "+":
                            _send_twist(label, axis_index, angular, 1.0)

                for _tw_label, _tw_index, _tw_angular in _twist_axes:
                    _add_twist_row(_tw_label, _tw_index, _tw_angular)
                tw_stop = server.gui.add_button("정지 (Stop)", color="red")

                @tw_stop.on_click
                def _(_: Any) -> None:
                    _vel_jog_stop()

        with _move_tabs.add_tab("TCP PTP"):
            handles["tcp_ptp_note"] = server.gui.add_text(
                "TCP PTP",
                initial_value="Move to TCP target, point-to-point. Cartesian path is not guaranteed.",
                disabled=True,
            )
            handles["tcp_ptp_help"] = server.gui.add_text(
                "Number entry",
                initial_value="Each row: left box = left arm, right box = right arm (mirror the live pose = Pose Monitor). Edit a box and press Enter to move that arm. The single −/+ jogs the arm(s) chosen by 'TCP arm' (both/left/right).",
                disabled=True,
            )
            handles["tcp_status"] = server.gui.add_text(
                "TCP status",
                initial_value="ready",
                disabled=True,
            )
            _install_tcp_target_callbacks(handles["scene"], handles["tcp_status"])
            handles["tcp_ptp_arm"] = "both"
            handles["tcp_ptp_arm_buttons"] = {}
            for arm in _TCP_PTP_ARM_OPTIONS:
                arm_button = server.gui.add_button("TCP arm: " + arm, color=_mode_button_color(arm, _tcp_ptp_arm(handles)))
                handles["tcp_ptp_arm_buttons"][arm] = arm_button

                @arm_button.on_click
                def _(_: Any, arm: str = arm) -> None:
                    handles["tcp_ptp_arm"] = arm
                    _update_tcp_ptp_arm_buttons(handles)
                    _refresh_tcp_ptp_axis_fields(handles)
                    handles["tcp_status"].value = f"TCP arm (−/+ and Send scope): {arm}"

            handles["tcp_frame_mode"] = _TCP_FRAME_STAND
            handles["tcp_frame_buttons"] = {}
            for frame_mode in _TCP_FRAME_OPTIONS:
                frame_button = server.gui.add_button(frame_mode, color=_mode_button_color(frame_mode, _tcp_frame_mode(handles)))
                handles["tcp_frame_buttons"][frame_mode] = frame_button

                @frame_button.on_click
                def _(_: Any, frame_mode: str = frame_mode) -> None:
                    handles["tcp_frame_mode"] = frame_mode
                    _update_tcp_frame_buttons(handles)
                    # Stand/world: fields show the live absolute pose; TCP local:
                    # they reset to 0,0,0,0,0,0 as relative-delta entry boxes.
                    _refresh_tcp_ptp_axis_fields(handles)
                    handles["tcp_status"].value = f"TCP frame: {frame_mode}"

            linear_step = server.gui.add_slider("Linear step mm", min=0.1, max=10.0, step=0.1, initial_value=5.0)
            angular_step = server.gui.add_slider("Angular step deg", min=0.1, max=10.0, step=0.1, initial_value=1.0)
            handles["tcp_linear_step"] = linear_step
            handles["tcp_angular_step"] = angular_step
            handles["tcp_pose_buttons"] = []
            send_target_button = server.gui.add_button("Send TCP target")
            handles["tcp_pose_buttons"].append(send_target_button)

            @send_target_button.on_click
            def _(_: Any) -> None:
                arm = _tcp_ptp_arm(handles)
                ok, message = _send_tcp_pose_target_from_marker(safety, handles["scene"], arm)
                handles["tcp_status"].value = ("OK: " if ok else "BLOCKED: ") + message

            def _nudge_ptp_axis_selected(axis_index: int, angular: bool, sign: float) -> None:
                # ONE [−][+] per axis: nudge the arm(s) the operator selected above —
                # both → left+right TOGETHER (in a single packet), otherwise just
                # that arm. Both arms MUST go in one packet: a single-arm TcpPoseTarget
                # resets the other arm to Hold on the server.
                arm_sel = _tcp_ptp_arm(handles)
                arms = ("left", "right") if arm_sel == "both" else (arm_sel,)
                frame_mode = _tcp_frame_mode(handles)
                if frame_mode == _TCP_FRAME_LOCAL:
                    # Relative jog: same delta to each selected arm; the helper sets
                    # both markers then sends one packet for the scope.
                    step = (
                        _angular_step_radians(float(angular_step.value))
                        if angular
                        else _linear_step_meters(float(linear_step.value))
                    )
                    delta = [0.0] * 6
                    delta[axis_index] = sign * step
                    ok, message = _apply_tcp_delta_and_send_pose_target(
                        safety, handles["scene"], arm_sel, tuple(delta), frame_mode  # type: ignore[arg-type]
                    )
                    handles["tcp_status"].value = ("OK: " if ok else "BLOCKED: ") + message
                    return
                # Stand frame: nudge each selected arm from its mirrored current value,
                # then send ALL of them in a single packet.
                step_disp = float(angular_step.value) if angular else float(linear_step.value)
                arm_values: dict[str, list[float]] = {}
                for arm in arms:
                    values = _read_ptp_arm_fields(handles, arm)
                    if values is None:
                        handles["tcp_status"].value = f"BLOCKED: {arm} TCP fields unavailable"
                        return
                    values[axis_index] += sign * step_disp
                    arm_values[arm] = values
                ok, message = _send_tcp_poses_absolute(safety, handles["scene"], arm_values)
                handles["tcp_status"].value = ("OK: " if ok else "BLOCKED: ") + message

            def _commit_ptp_axis(axis_index: int, angular: bool) -> None:
                # Fires on Enter in either slot of this axis's [left | right] vector2
                # (the patched viser VectorInput only emits on Enter). Programmatic
                # mirror writes set the guard so they never re-trigger this.
                if handles.get("_tcp_ptp_field_updating"):
                    return
                vec = handles.get("tcp_ptp_axis_vec", {}).get(axis_index)
                if vec is None:
                    return
                try:
                    new_left = float(vec.value[0])
                    new_right = float(vec.value[1])
                except Exception:
                    return
                # Diff against the last mirrored values to find which arm the operator
                # actually edited — move only that arm. First commit before any mirror:
                # treat both as edited.
                shown = handles.get("_tcp_ptp_shown", {}).get(axis_index)
                edited: list[tuple[str, float]] = []
                if shown is None:
                    edited = [("left", new_left), ("right", new_right)]
                else:
                    if abs(new_left - float(shown[0])) > 1e-4:
                        edited.append(("left", new_left))
                    if abs(new_right - float(shown[1])) > 1e-4:
                        edited.append(("right", new_right))
                if not edited:
                    return
                frame_mode = _tcp_frame_mode(handles)
                if frame_mode == _TCP_FRAME_LOCAL:
                    # Relative jog per edited arm into its marker, then ONE packet.
                    scope_arms: list[str] = []
                    for arm, raw in edited:
                        delta = [0.0] * 6
                        delta[axis_index] = math.radians(raw) if angular else raw * 0.001
                        if not _apply_tcp_delta_to_target(handles["scene"], arm, tuple(delta), frame_mode):  # type: ignore[arg-type]
                            handles["tcp_status"].value = f"BLOCKED: {arm} TCP target unavailable"
                            return
                        scope_arms.append(arm)
                    scope = "both" if set(scope_arms) == {"left", "right"} else scope_arms[0]
                    ok, message = _send_tcp_pose_target_from_marker(safety, handles["scene"], scope)
                else:
                    # Absolute set: each edited arm's whole pose (only its edited axis
                    # differs from its live pose); send all edited arms in one packet.
                    arm_values: dict[str, list[float]] = {}
                    for arm, _raw in edited:
                        values = _read_ptp_arm_fields(handles, arm)
                        if values is None:
                            handles["tcp_status"].value = f"BLOCKED: {arm} TCP fields unavailable"
                            return
                        arm_values[arm] = values
                    ok, message = _send_tcp_poses_absolute(safety, handles["scene"], arm_values)
                handles["tcp_status"].value = ("OK: " if ok else "BLOCKED: ") + message

            def _add_ptp_axis_row(axis_label: str, axis_index: int, angular: bool) -> None:
                # One row per axis: a single label (X/Roll/…) over a [left | right]
                # pair of half-width number boxes (vector2), then ONE full-width
                # [−][+] nudge group that acts on the selected arm(s). The unit lives
                # in the section header, the arm in the box order (left | right).
                vec = server.gui.add_vector2(axis_label, initial_value=(0.0, 0.0), step=0.1)
                handles["tcp_ptp_axis_vec"][axis_index] = vec

                @vec.on_update
                def _(_: Any, axis_index: int = axis_index, angular: bool = angular) -> None:
                    _commit_ptp_axis(axis_index, angular)

                group = server.gui.add_button_group("", ("−", "+"))

                @group.on_click
                def _(_: Any, group: Any = group, axis_index: int = axis_index, angular: bool = angular) -> None:
                    choice = group.value
                    if choice not in ("−", "+"):
                        return
                    _nudge_ptp_axis_selected(axis_index, angular, 1.0 if choice == "+" else -1.0)

            # Per-axis [left | right] paired layout: one label + two half-width boxes,
            # both arms always visible. Each box mirrors its arm's live pose and, on
            # Enter, moves that arm. The single −/+ per axis acts on the arm(s) the
            # "TCP arm" selector chooses (both / left / right).
            handles["tcp_ptp_axis_vec"] = {}
            with server.gui.add_folder("Position — left | right (mm)", expand_by_default=True):
                for label, index, angular in _TCP_PTP_AXES[:3]:
                    _add_ptp_axis_row(label.upper(), index, angular)
            with server.gui.add_folder("Rotation — left | right (deg)", expand_by_default=True):
                for label, index, angular in _TCP_PTP_AXES[3:]:
                    _add_ptp_axis_row(label.capitalize(), index, angular)
            _refresh_tcp_ptp_axis_fields(handles)

        with _move_tabs.add_tab("TCP Linear"):
            handles["tcp_linear_note"] = server.gui.add_text(
                "TCP Linear",
                initial_value="Move linearly in Cartesian TCP space. Constant orientation keeps the start orientation.",
                disabled=True,
            )
            handles["tcp_linear_status"] = server.gui.add_text(
                "TCP Linear status",
                initial_value="ready",
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

    with tabs.add_tab("고급"):
        _adv_tabs = server.gui.add_tab_group()
        with _adv_tabs.add_tab("Circle"):
            c_diameter = server.gui.add_slider("Diameter m", min=0.02, max=0.20, step=0.01, initial_value=0.15)
            c_period = server.gui.add_slider("Period s", min=3.0, max=16.0, step=0.5, initial_value=4.0)
            c_plane = server.gui.add_button_group("Plane", ("xy", "xz", "yz"))
            c_start = server.gui.add_button("Start circle (both arms)")
            c_stop = server.gui.add_button("Stop circle")
            # Start greys out when the TCP/Cartesian gate is closed; Stop always
            # stays live so the operator can halt a running circle.
            handles["circle_start_button"] = c_start
            handles["circle_status"] = server.gui.add_text("Circle status", initial_value="idle", disabled=True)
            # Per-run stop event stored WITH its thread (not a single shared Event):
            # Start joins any prior thread before launching, so a quick Stop→Start no
            # longer hits "already running" and there is no shared-event clear/recheck
            # race between an outgoing thread and the incoming one.
            circle_state: dict[str, Any] = {"thread": None, "stop": None}

            def _circle_loop(diameter: float, period: float, plane: str, stop_event: threading.Event) -> None:
                # Hold the command-source lease for the whole circle so the keep-
                # alive re-sends carry a FIXED seq. Without a held lease, send()
                # brackets every packet with Acquire/Release and re-issues a fresh
                # seq each time (so one-shot commands clear the Acquire seq); for a
                # streaming circle that fresh seq makes the server treat every
                # 0.3 s keep-alive as a NEW command and re-init the circle to the
                # current TCP — the arm only ever traces the first tangent step and
                # drifts in a straight line. Holding the lease makes bracket=False,
                # so the seq stays fixed: the server accepts ONE circle command and
                # re-sends (same seq) are dropped as harmless keep-alives while the
                # long timeout keeps the single command tracing the full circle.
                #
                # Acquire BEFORE ArmMotion/build so seq order is acquire < ArmMotion
                # < circle (the server drops any command whose seq <= the last
                # accepted seq). Only release a lease we took here, so an operator's
                # explicit "Take control" is left untouched.
                client = safety.command_client
                took_lease = not client.hold_lease
                if took_lease:
                    client.acquire_lease()
                try:
                    safety.send_lifecycle("ArmMotion")
                    time.sleep(0.1)
                    ok, message, packet = safety.build_circle_packet(
                        diameter, period, arm="both", plane=plane, repeat=200
                    )
                    if not ok or packet is None:
                        handles["circle_status"].value = "BLOCKED: " + message
                        return
                    handles["circle_status"].value = "running: " + message
                    while not stop_event.is_set():
                        client.send(packet)
                        stop_event.wait(0.3)
                    client.send_lifecycle("Hold")
                finally:
                    if took_lease:
                        client.release_lease()

            @c_start.on_click
            def _(_: Any) -> None:
                # Cleanly tear down any prior run BEFORE starting: signal its own
                # stop event and join so it has released the lease and sent Hold.
                existing, existing_stop = circle_state["thread"], circle_state["stop"]
                if existing is not None and existing.is_alive():
                    if existing_stop is not None:
                        existing_stop.set()
                    existing.join(timeout=1.5)
                reason = safety.tcp_command_disabled_reason()
                if reason:
                    handles["circle_status"].value = "BLOCKED: " + reason
                    return
                stop_event = threading.Event()
                thread = threading.Thread(
                    target=_circle_loop,
                    args=(float(c_diameter.value), float(c_period.value), str(c_plane.value), stop_event),
                    daemon=True,
                )
                circle_state["thread"], circle_state["stop"] = thread, stop_event
                thread.start()
                handles["circle_status"].value = "starting..."

            @c_stop.on_click
            def _(_: Any) -> None:
                stop_event = circle_state["stop"]
                if stop_event is not None:
                    stop_event.set()
                handles["circle_status"].value = "stopped"

        with _adv_tabs.add_tab("Delta"):
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

        with _adv_tabs.add_tab("Debug"):
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


def _gripper_cmd_endpoint() -> tuple[str, int]:
    """gripper_server command endpoint (gripper_cmd.v1 destination). Override with
    RB_GUI_GRIPPER_CMD_ENDPOINT=udp://host:port (default matches the stack config)."""
    raw = os.environ.get("RB_GUI_GRIPPER_CMD_ENDPOINT", "udp://127.0.0.1:50410")
    text = raw[len("udp://"):] if raw.startswith("udp://") else raw
    host, _, port = text.rpartition(":")
    try:
        return (host or "127.0.0.1", int(port))
    except ValueError:
        return ("127.0.0.1", 50410)


_GRIPPER_SYNC_EPS = 0.5  # % tolerance to tell our own feedback write from an operator move


def _send_gripper_command(handles: dict[str, Any]) -> None:
    """Send the gripper slider values to gripper_server as gripper_cmd.v1.

    Each slider doubles as a live actual-% readout (see _update_gripper_feedback),
    so this on_update fires both when the OPERATOR moves a slider and when we write
    the real gripper feedback back into it. We must only emit a command for the
    former: a side whose current value still matches the last value we synced from
    feedback (within _GRIPPER_SYNC_EPS) is treated as a feedback echo and skipped.
    A genuine operator move also starts a manual-hold window so _update_gripper_feedback
    stops overwriting that slider until the gripper has had time to react.

    gripper_server drives the sim/real gripper; its feedback (state JSON) moves the
    viz. Coexists with teleop/policy gripper commands (latest setpoint wins)."""
    sock = handles.get("gripper_cmd_sock")
    endpoint = handles.get("gripper_cmd_endpoint")
    if sock is None or endpoint is None:
        return
    msg: dict[str, Any] = {"schema": "robotics_lab.gripper_cmd.v1", "deadman": True}
    operator_moved = False
    hold_sec = float(handles.get("gripper_manual_hold_sec", 1.0))
    for side in ("left", "right"):
        slider = handles.get(f"gripper_slider_{side}")
        if slider is None:
            continue
        try:
            value = float(slider.value)
        except (TypeError, ValueError, AttributeError):
            continue
        msg[side] = {"percent": value, "valid": True}
        synced = handles.get(f"gripper_synced_value_{side}")
        # synced is None until the slider has tracked real gripper feedback at
        # least once (startup, before gripper_server publishes; or no gripper_server
        # at all). Do NOT treat that initial default as an operator move: when a
        # browser client connects it echoes the slider's initial value (100 = open),
        # which would otherwise command the gripper OPEN on every startup. Hold the
        # power-on gripper position until we've synced to hardware, then only a
        # genuine move (beyond eps from the synced value) commands it.
        if synced is not None and abs(value - float(synced)) > _GRIPPER_SYNC_EPS:
            operator_moved = True
            handles[f"gripper_manual_hold_until_{side}"] = time.monotonic() + hold_sec
    if not operator_moved:
        return  # only feedback echoes — do not command the gripper to its own position
    seq = int(handles.get("gripper_cmd_seq", 0)) + 1
    handles["gripper_cmd_seq"] = seq
    msg["seq"] = seq
    if "left" not in msg and "right" not in msg:
        return
    try:
        sock.sendto(json.dumps(msg).encode("utf-8"), endpoint)
    except OSError:
        pass


def _push_gripper_percent(handles: dict[str, Any], latest: StateSnapshot | None = None) -> None:
    """Publish the gripper open-percentage (0..100) into scene_handles so
    update_scene_markers can drive the articulated-gripper fingers.

    Source priority: the server's published gripper feedback (gripper_state.v1
    stamped into the state JSON) when valid; otherwise the GUI preview slider.
    So once gripper_server is wired the fingers track the real gripper, and the
    slider stays a manual fallback when there is no feed."""
    scene = handles.get("scene")
    if not isinstance(scene, dict):
        return
    for side in ("left", "right"):
        published = getattr(getattr(latest, side, None), "gripper_percent", None) if latest is not None else None
        if isinstance(published, (int, float)):
            scene[f"gripper_percent_{side}"] = float(published)
            continue
        slider = handles.get(f"gripper_slider_{side}")
        if slider is None:
            continue
        try:
            scene[f"gripper_percent_{side}"] = float(slider.value)
        except (TypeError, ValueError, AttributeError):
            pass


def _update_gripper_feedback(handles: dict[str, Any], latest: StateSnapshot | None, *, stale: bool) -> None:
    """Drive each gripper slider (and its number box) to the live actual opening %.

    The real opening comes from the server gripper_state.v1 feedback
    (latest.<side>.gripper_percent). Writing it into the slider keeps the slider +
    number box in sync with the hardware. We record the synced value so the slider's
    on_update (which fires on this programmatic write too) can tell the echo from a
    real operator move in _send_gripper_command, and we skip a side whose manual-hold
    window is still open so an operator's just-commanded value is not yanked back."""
    if stale:
        return  # don't chase a frozen/last-known reading while the feed is stale
    now = time.monotonic()
    for side in ("left", "right"):
        slider = handles.get(f"gripper_slider_{side}")
        if slider is None:
            continue
        hold_until = handles.get(f"gripper_manual_hold_until_{side}")
        if isinstance(hold_until, (int, float)) and now < float(hold_until):
            continue  # operator just moved this slider; leave their value in place
        arm = getattr(latest, side, None) if latest is not None else None
        pct = getattr(arm, "gripper_percent", None) if arm is not None else None
        if not isinstance(pct, (int, float)) or not math.isfinite(float(pct)):
            continue  # no valid feed -> leave the slider as the operator's last setpoint
        value = float(pct)
        # Record BEFORE writing so the resulting on_update sees value == synced.
        handles[f"gripper_synced_value_{side}"] = value
        try:
            slider.value = value
        except (TypeError, ValueError, AttributeError):
            pass


def _update_floor_panel(handles: dict[str, Any], latest: StateSnapshot | None) -> None:
    floor = latest.floor_constraint if latest is not None else None
    update_floor_plane(handles.get("scene", {}), floor)
    # One-shot: bring the enforce checkbox up at the server-reported state, then leave
    # it operator-controlled (guarded by a flag so we don't fight the user's clicks).
    # Skip this when the operator has a persisted preference: that choice is restored
    # as the checkbox default and pushed to the server in _build_status_updater, so we
    # must not overwrite it from telemetry here (see update_gui).
    if (
        "floor_enforce_toggle" in handles
        and isinstance(floor, Mapping)
        and not handles.get("floor_enforce_synced", False)
    ):
        if handles.get("stand_floor_enforce_pref") is None:
            try:
                handles["floor_enforce_toggle"].value = bool(floor.get("enabled", False))
            except Exception:
                pass
        handles["floor_enforce_synced"] = True
    slider = handles.get("floor_slider")
    if not isinstance(floor, Mapping) or not bool(floor.get("enabled", False)):
        update_floor_plane_preview(handles.get("scene", {}), None)
        if "floor_applied" in handles:
            handles["floor_applied"].value = "disabled"
        return
    z = floor.get("z_min_m")
    # Sync the slider bounds to the server's runtime-allowed range first, so any
    # value we write below lands inside [min, max].
    if slider is not None:
        lo = floor.get("runtime_min_z_m")
        hi = floor.get("runtime_max_z_m")
        if isinstance(lo, (int, float)) and isinstance(hi, (int, float)) and float(hi) > float(lo):
            try:
                slider.min = float(lo) * 1000.0
                slider.max = float(hi) * 1000.0
            except Exception:
                pass
    # First-state init: bring the slider up at the server-applied z instead of the
    # hardcoded default, so the operator edits from the actual current floor. Done
    # once (guarded by a handles flag) to avoid fighting the user's later edits.
    if (
        slider is not None
        and isinstance(z, (int, float))
        and not handles.get("floor_slider_synced", False)
    ):
        applied_mm = float(z) * 1000.0
        try:
            applied_mm = max(float(slider.min), min(float(slider.max), applied_mm))
        except Exception:
            pass
        slider.value = applied_mm
        handles["floor_slider_synced"] = True
    # Pending-value preview reconciliation: show the yellow preview plane only
    # while the slider differs from the server-applied z (>= 0.5 mm); after a
    # successful Send the applied plane catches up and the preview disappears.
    if slider is not None and isinstance(z, (int, float)):
        pending_mm = float(slider.value)
        applied_mm = float(z) * 1000.0
        if abs(pending_mm - applied_mm) >= 0.05:
            update_floor_plane_preview(handles.get("scene", {}), pending_mm / 1000.0)
        else:
            update_floor_plane_preview(handles.get("scene", {}), None)
    if "floor_applied" not in handles:
        return
    z_txt = f"{float(z) * 1000:.1f}mm" if isinstance(z, (int, float)) else "?"
    reject = floor.get("last_set_reject_reason")
    handles["floor_applied"].value = z_txt + (f" (last reject: {reject})" if reject else "")


def _roi_bounds_floats(value: Any) -> list[float] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    try:
        out = [float(v) for v in value]
    except (TypeError, ValueError):
        return None
    if any(not math.isfinite(v) for v in out):
        return None
    return out


def _update_roi_panel(handles: dict[str, Any], latest: StateSnapshot | None) -> None:
    roi = latest.roi_box if latest is not None else None
    # The "ROI 영역 표시" checkbox (default ON) controls scene visibility,
    # independent of whether the server is enforcing the box. The box is drawn
    # from the published bounds whenever the toggle is on and bounds are valid.
    toggle = handles.get("roi_box_visible_toggle")
    visible = bool(getattr(toggle, "value", True))
    update_roi_box(handles.get("scene", {}), roi, visible=visible)
    axes = ("x", "y", "z")
    have_sliders = all(f"roi_{a}_min" in handles and f"roi_{a}_max" in handles for a in axes)
    enabled = isinstance(roi, Mapping) and bool(roi.get("enabled", False))
    applied_min = _roi_bounds_floats(roi.get("min_m")) if isinstance(roi, Mapping) else None
    applied_max = _roi_bounds_floats(roi.get("max_m")) if isinstance(roi, Mapping) else None
    runtime_min = _roi_bounds_floats(roi.get("runtime_min_m")) if isinstance(roi, Mapping) else None
    runtime_max = _roi_bounds_floats(roi.get("runtime_max_m")) if isinstance(roi, Mapping) else None
    # Sync number-input soft range to the server's per-axis runtime envelope first.
    if have_sliders and runtime_min is not None and runtime_max is not None:
        for k, a in enumerate(axes):
            if runtime_max[k] > runtime_min[k]:
                for suffix in ("min", "max"):
                    try:
                        handles[f"roi_{a}_{suffix}"].min = runtime_min[k] * 1000.0
                        handles[f"roi_{a}_{suffix}"].max = runtime_max[k] * 1000.0
                    except Exception:
                        pass
    # First-state init: bring the inputs up at the server-applied bounds (once).
    if (
        have_sliders
        and applied_min is not None
        and applied_max is not None
        and not handles.get("roi_sliders_synced", False)
    ):
        for k, a in enumerate(axes):
            try:
                lo_h = handles[f"roi_{a}_min"]
                hi_h = handles[f"roi_{a}_max"]
                lo_h.value = max(float(lo_h.min), min(float(lo_h.max), applied_min[k] * 1000.0))
                hi_h.value = max(float(hi_h.min), min(float(hi_h.max), applied_max[k] * 1000.0))
            except Exception:
                pass
        handles["roi_sliders_synced"] = True
    # Pending-value preview reconciliation: show the yellow preview box only while
    # visible AND any input differs from the server-applied bound (>= 0.5 mm).
    if visible and have_sliders and applied_min is not None and applied_max is not None:
        fn = handles.get("roi_slider_bounds_fn")
        slider_lo, slider_hi = fn() if callable(fn) else (None, None)
        if slider_lo is not None and slider_hi is not None:
            differs = any(
                abs(slider_lo[k] - applied_min[k]) >= 0.0005
                or abs(slider_hi[k] - applied_max[k]) >= 0.0005
                for k in range(3)
            )
            if differs:
                update_roi_box_preview(handles.get("scene", {}), slider_lo, slider_hi)
            else:
                update_roi_box_preview(handles.get("scene", {}), None, None)
    else:
        update_roi_box_preview(handles.get("scene", {}), None, None)
    if "roi_applied" not in handles:
        return
    if not enabled:
        handles["roi_applied"].value = "disabled"
        return
    if applied_min is not None and applied_max is not None:
        txt = " ".join(
            f"{a}[{applied_min[k] * 1000:.0f},{applied_max[k] * 1000:.0f}]"
            for k, a in enumerate(axes)
        ) + "mm"
    else:
        txt = "?"
    reject = roi.get("last_set_reject_reason")
    handles["roi_applied"].value = txt + (f" (last reject: {reject})" if reject else "")


def _update_circle_overlay_gui(handles: dict[str, Any], overlay_store: CircleOverlayStore | None) -> None:
    overlay, stale = _latest_circle_overlay(overlay_store)
    if "circle_overlay" in handles:
        handles["circle_overlay"].value = _format_circle_overlay_status(
            overlay,
            stale=stale,
            enabled=overlay_store is not None,
        )
    update_circle_overlay(handles.get("scene", {}), overlay, stale=stale)


def _update_lease_owner(
    handles: dict[str, Any],
    latest: StateSnapshot | None,
    my_source_id: str,
    *,
    held: bool,
) -> None:
    """Show who owns the command-source lease, derived from server state.

    The server lease is non-preemptive: while another source (e.g. policy_runner
    teleop) owns it, a GUI Acquire is rejected. Surfacing the owner turns a silent
    failure into an explicit "stop/release the other source first"."""
    handle = handles.get("lease_owner_status")
    if handle is None:
        return
    if latest is None:
        handle.value = "no state stream"
        return
    command_source = latest.command_source
    owner = command_source.display_source_id
    if not command_source.active or owner is None:
        handle.value = "free — you hold it (pending)" if held else "free"
        return
    if owner == my_source_id:
        handle.value = "held by you" + ("; Take control ON" if held else "")
    else:
        handle.value = f"held by {owner} — stop or release it before the GUI can take control"


def update_gui(
    handles: dict[str, Any],
    safety: OperatorSafety,
    store: StateStore,
    overlay_store: CircleOverlayStore | None = None,
) -> None:
    disabled_states = safety.control_disabled_states()
    disabled_reasons = safety.control_disabled_reasons()
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
    # Single buttons that can be greyed (viser button_groups cannot).
    if "circle_start_button" in handles:
        _set_disabled(handles["circle_start_button"], disabled_states.get("circle", True))
    # Button-group tabs (jog/velocity/twist) cannot be greyed in viser, so reflect
    # the live gate reason into their status line proactively — never clobbering a
    # recent click result, only the prior DISABLED note.
    _reflect_gate_reason(handles.get("jog_status"), disabled_reasons.get("jog"))
    _reflect_gate_reason(
        handles.get("velocity_status"),
        disabled_reasons.get("velocity") or disabled_reasons.get("twist"),
    )

    if "mode_buttons" in handles:
        _update_desired_mode_buttons(handles, safety.desired_mode)
    if "tcp_ptp_arm_buttons" in handles:
        _update_tcp_ptp_arm_buttons(handles)
    if "tcp_frame_buttons" in handles:
        _update_tcp_frame_buttons(handles)
    if "tcp_display_buttons" in handles:
        _update_tcp_display_buttons(handles)
    if "tcp_linear_arm_buttons" in handles or "tcp_linear_orientation_buttons" in handles:
        _update_tcp_linear_selection_buttons(handles)
    latest = store.latest()
    stale = store.is_stale()
    # Stash the live snapshot so the TCP PTP fields can mirror the current pose
    # every tick (the patched viser NumberInput ignores server updates while it is
    # focused, so a periodic repaint never clobbers a value the operator is typing).
    handles["_latest_state"] = latest
    handles["_state_stale"] = stale
    if "tcp_ptp_axis_vec" in handles:
        _refresh_tcp_ptp_axis_fields(handles)
    readiness = safety.readiness()
    _update_lease_owner(handles, latest, safety.command_client.source_id, held=safety.command_client.hold_lease)
    _update_circle_overlay_gui(handles, overlay_store)
    if latest is None:
        _update_operator_monitors(handles, None, stale=True)
        if "status_summary" in handles:
            handles["status_summary"].content = _status_summary_html(
                connection="disconnected",
                mode=safety.observed_server_mode,
                readiness_go=False,
                motion="unknown",
                fault_active=False,
            )
        handles["connection"].value = "disconnected/stale"
        handles["readiness"].value = readiness.no_go_reason or "No-Go: no state stream"
        if "self_collision" in handles:
            handles["self_collision"].value = _format_self_collision_status(None, stale=True)
        if "floor_constraint" in handles:
            handles["floor_constraint"].value = _format_floor_constraint_status(None, stale=True)
        _update_floor_panel(handles, None)
        if "roi_box" in handles:
            handles["roi_box"].value = _format_roi_box_status(None, stale=True)
        _update_roi_panel(handles, None)
        if "user_floor_constraint" in handles:
            handles["user_floor_constraint"].value = _format_user_floor_constraint_status(None, stale=True)
        update_user_floor_plane(handles.get("scene", {}), None)
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
    if "freedrive_status" in handles:
        fd = latest.freedrive or {}
        # Per-arm lifecycle stage: off / arming_quiesce / arming_confirm / active / exiting.
        stage_labels = {
            "arming_quiesce": "정지대기",
            "arming_confirm": "확인중",
            "active": "ON",
            "exiting": "해제중",
        }
        left_stage = str(fd.get("left_stage", "off"))
        right_stage = str(fd.get("right_stage", "off"))
        note = str(fd.get("note", "") or "")
        parts = (
            ([f"왼팔 {stage_labels[left_stage]}"] if left_stage in stage_labels else [])
            + ([f"오른팔 {stage_labels[right_stage]}"] if right_stage in stage_labels else [])
        )
        if parts:
            handles["freedrive_status"].value = (
                f"DIRECT TEACHING — {', '.join(parts)} (servo_j 억제됨)"
            )
        elif note:
            handles["freedrive_status"].value = f"off — {note}"
        else:
            handles["freedrive_status"].value = "off"
    if "status_summary" in handles:
        handles["status_summary"].content = _status_summary_html(
            connection="stale" if stale else "live",
            mode=safety.observed_server_mode,
            readiness_go=readiness.ready,
            motion=latest.motion_state,
            fault_active=latest.fault_latched,
        )
    if "self_collision" in handles:
        handles["self_collision"].value = _format_self_collision_status(latest, stale=stale)
    if "floor_constraint" in handles:
        handles["floor_constraint"].value = _format_floor_constraint_status(latest, stale=stale)
    _update_floor_panel(handles, latest)
    if "roi_box" in handles:
        handles["roi_box"].value = _format_roi_box_status(latest, stale=stale)
    _update_roi_panel(handles, latest)
    if "user_floor_constraint" in handles:
        handles["user_floor_constraint"].value = _format_user_floor_constraint_status(latest, stale=stale)
    update_user_floor_plane(handles.get("scene", {}), latest.user_floor_constraint)
    update_user_floor_capture_points(
        handles.get("scene", {}), _user_floor_state(handles).get("points", []))
    # Restore a persisted, previously-enabled user floor plane once the state stream
    # is live (the server starts with it disabled until commanded). RETRY until the
    # server telemetry confirms enabled: the leaseless command can be dropped while the
    # server's command intake is still coming up at startup, even though state already
    # publishes — a one-shot send would then be silently lost and the operator would
    # have to re-press Fit & Apply. Capped (~5 s) so a genuinely-rejecting server (e.g.
    # user_floor disabled in config) does not get spammed forever.
    if not stale and not handles.get("user_floor_resent", False):
        uf_state = _user_floor_state(handles)
        plane = uf_state.get("plane")
        want = bool(uf_state.get("enabled", False)) and isinstance(plane, Mapping)
        uf = latest.user_floor_constraint if latest is not None else None
        confirmed = isinstance(uf, Mapping) and bool(uf.get("enabled", False))
        attempts = handles.get("user_floor_resend_attempts", 0)
        if not want or confirmed or attempts >= 50:
            if want and not confirmed and attempts >= 50 and "user_floor_set_status" in handles:
                handles["user_floor_set_status"].value = "restore gave up (server kept rejecting)"
            handles["user_floor_resent"] = True
        else:
            ok, msg = safety.send_set_user_floor_plane(
                tuple(plane["point"]), tuple(plane["normal"]),
                margin_m=float(uf_state.get("margin_mm", 0.0)) / 1000.0, enable=True,
            )
            handles["user_floor_resend_attempts"] = attempts + 1
            if "user_floor_set_status" in handles:
                handles["user_floor_set_status"].value = ("restoring... " if ok else "restore failed: ") + msg
    # Re-apply the operator's persisted "Enforce stand floor" choice once the state
    # stream is live, so the GUI's last setting wins over the server's config default on
    # every launch. Same retry-until-confirmed pattern (the startup command can be
    # dropped). No-op when there is no saved preference (checkbox just mirrors telemetry).
    if not stale and not handles.get("floor_enforce_resent", False):
        pref = handles.get("stand_floor_enforce_pref")
        floor = latest.floor_constraint if latest is not None else None
        server_enabled = (
            bool(floor.get("enabled", False)) if isinstance(floor, Mapping) else None
        )
        attempts = handles.get("floor_enforce_resend_attempts", 0)
        if not isinstance(pref, bool) or server_enabled == pref or attempts >= 50:
            handles["floor_enforce_resent"] = True
        else:
            ok, msg = safety.send_set_floor_enabled(pref)
            handles["floor_enforce_resend_attempts"] = attempts + 1
            if "floor_set_status" in handles:
                handles["floor_set_status"].value = (
                    "restoring... " if ok else "restore failed: "
                ) + msg
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
    _push_gripper_percent(handles, latest)
    _update_gripper_feedback(handles, latest, stale=stale)
    update_scene_markers(handles.get("scene", {}), latest, tcp_display_mode=_tcp_display_mode(handles))
    # After markers (TCP frames now posed): toggle the orange floor-check points,
    # which are parented under /stand/<arm>_tcp and ride those poses.
    _floor_points_toggle = handles.get("floor_check_points_toggle")
    update_floor_check_points(
        handles.get("scene", {}),
        latest,
        show=bool(getattr(_floor_points_toggle, "value", False)),
    )
    # After markers: the collision overlay may override ghost/solid visibility.
    update_self_collision_overlay(handles.get("scene", {}), latest)
    toggle = handles.get("self_collision_capsules_toggle")
    _self_collision_show = bool(getattr(toggle, "value", False))
    update_self_collision_near_pairs(
        handles.get("scene", {}),
        latest,
        show=_self_collision_show,
    )
    update_self_collision_check_geom(
        handles.get("scene", {}),
        latest,
        show=_self_collision_show,
    )
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
    init_left_joints = _env_joint6("RB_GUI_INIT_LEFT_JOINTS", _DEFAULT_INIT_LEFT_JOINTS_DEG)
    init_right_joints = _env_joint6("RB_GUI_INIT_RIGHT_JOINTS", _DEFAULT_INIT_RIGHT_JOINTS_DEG)
    # Persisted InitMotion pose (set from a waypoint via the GUI) wins over
    # env/default so it survives restarts without editing code.
    saved_left_init, saved_right_init = _load_init_joints()
    if saved_left_init is not None and saved_right_init is not None:
        init_left_joints, init_right_joints = saved_left_init, saved_right_init
        print(f"rb_servo_gui: loaded InitMotion pose from {_init_motion_path()}", flush=True)
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
        ops_available=ops_available,
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
