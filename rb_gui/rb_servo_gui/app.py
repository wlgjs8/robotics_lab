from __future__ import annotations

import argparse
import errno
import glob
import json
import math
import os
import socket
import threading
import time
from html import escape
from pathlib import Path
from typing import Any, Mapping, NamedTuple

import numpy as np

from .box_detect_control import BoxDetectCommandClient, BoxDetectCommandResult
from .camera_quality import (
    ARMS as CAMERA_QUALITY_ARMS,
    CameraQualityReceiver,
    CameraQualityStore,
    RobotMotionTracker,
    camera_quality_html,
)
from .command_client import CommandClient
from .head_preview import (
    DEFAULT_HEAD_STREAM,
    DEFAULT_HEAD_TOPIC,
    HeadPreviewReceiver,
    HeadPreviewStore,
)
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
from .models import (
    EXTERNAL_BOX_LABEL_SLOTS,
    ArmSnapshot,
    ChunkOverlaySnapshot,
    CircleOverlaySnapshot,
    Pose6D,
    StateSnapshot,
    box_lock_status_text,
    external_box_display,
)
from .chunk_overlay_receiver import ChunkOverlayReceiver, ChunkOverlayStore
from .overlay_receiver import CircleOverlayReceiver, CircleOverlayStore, parse_udp_bind
from .pointcloud_receiver import (
    StereoCloudReceiver,
    StereoCloudStore,
    load_T_stand_cam,
    save_T_stand_cam,
    load_T_tcp_cam,
    save_T_tcp_cam,
    mat_to_wxyz,
    wxyz_to_mat,
)
from .plane_fit import fit_plane, tilt_deg
from .recording_control import (
    ArmInitCommandResult,
    RecordingCommandClient,
    RecordingCommandResult,
    RecordingStatusReceiver,
    RecordingStatusStore,
    SpaceMouseCommandResult,
    normalize_arm_init_status,
    normalize_recording_status,
    parse_udp_endpoint,
)
from .realtime_health import RealtimeTimingHistory, realtime_health_html
from .safety import (
    OperatorSafety,
    normalize_observed_mode_backend,
    server_safety_constraint_config_enabled,
)
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
    _pose_position,
    _repo_descriptions_dir,
    _robot_urdf_path,
    _stand_mesh_path,
    _update_urdf_config,
    set_ik_infeasible_region_visible,
    set_reach_envelope_visible,
    update_chunk_overlay,
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
    _format_ft_status,
    _format_force_control_status,
    _format_init_motion_status,
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
# TcpPoseTarget replay/profiling init-motion anchor (2026-06-21): GRIPPER-DOWN (tool z ~ stand
# -z), max single-rest coverage. Left = controller-consistent rest; right = rest + 10deg about
# y (within +/-20deg tool budget; recovers episode_0096). Server-CALIBRATED offline model
# (single-arm rb3_730e.urdf + stack_sim mount, exact ee_local incl r_align=pika_rz180;
# reproduces live REST=12/12, FWD=0/12). Coverage 383/387; the 4 unrecoverable left-wrist
# singularities (0043,0220,0291,0373) were removed from data_tcp. Mirrors
# Override the default init-pose anchors with RB_GUI_INIT_*_JOINTS.
_DEFAULT_INIT_LEFT_JOINTS_DEG = (259.0, 75.6, 129.5, -55.6, -131.2, -161.7)
_DEFAULT_INIT_RIGHT_JOINTS_DEG = (-253.7, -76.9, -127.6, 65.7, 143.7, 166.9)
_OPERATOR_MONITOR_WIDTH_EM = 18.0
_OPERATOR_MONITOR_GAP_EM = 1.0
# Vertical anchor (em, in monitor-card font size) where the lower monitor cards
# begin in both columns. Sized to clear the Joint and FT content above them.
# Override with RB_GUI_MONITOR_SPLIT_EM.
_OPERATOR_MONITOR_SPLIT_EM = 35.5
# The RIGHT column splits LOWER than the left. Camera Quality is seven fixed lines,
# while the FT card is a table that GROWS under a push (lever + the three force-law
# rows appear only then). Sharing the left column's split left the FT card 9-27 px
# short of its own worst case, depending on viewport — measured, and it scrolled.
_OPERATOR_MONITOR_SPLIT_FT_EM = 46.0
_CAMERA_QUALITY_MONITOR_STALE_SEC = 0.5


def _tcp_gizmo_visible(handles: dict[str, Any]) -> bool:
    toggle = handles.get("tcp_gizmo_toggle")
    if toggle is not None:
        try:
            visible = bool(toggle.value)
            handles["tcp_gizmo_visible"] = visible
            return visible
        except Exception:
            pass
    return bool(handles.get("tcp_gizmo_visible", True))


def _hide_tcp_gizmos(handles: dict[str, Any]) -> None:
    scene_handles = handles.get("scene", {})
    if not isinstance(scene_handles, dict):
        return
    for arm in ("left", "right"):
        handle = scene_handles.get(f"{arm}_tcp_target")
        if handle is None:
            continue
        try:
            handle.visible = False
        except Exception:
            pass


def _chunk_overlay_visible(handles: dict[str, Any]) -> bool:
    toggle = handles.get("chunk_overlay_toggle")
    if toggle is not None:
        try:
            visible = bool(toggle.value)
            handles["chunk_overlay_visible"] = visible
            return visible
        except Exception:
            pass
    return bool(handles.get("chunk_overlay_visible", False))


def _hide_chunk_overlays(handles: dict[str, Any]) -> None:
    update_chunk_overlay(handles.get("scene", {}), None, visible=False)
    handle = handles.get("chunk_overlay_error_text")
    if handle is not None:
        try:
            handle.value = "L — / R —"
        except Exception:
            pass


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


def _env_bool(name: str, fallback: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return fallback
    return raw.strip().lower() in ("1", "true", "yes", "on")


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
        "--chunk-overlay-bind",
        default=None,
        help=(
            "UDP endpoint for predicted action-chunk overlay packets, for example "
            "udp://0.0.0.0:50262. Use 'none' or an empty value to disable. "
            "Defaults to RB_GUI_CHUNK_OVERLAY_BIND or udp://0.0.0.0:50262."
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


def _chunk_overlay_bind_from_args_env(args: argparse.Namespace) -> tuple[str, int] | None:
    endpoint = args.chunk_overlay_bind
    if endpoint is None:
        endpoint = os.environ.get("RB_GUI_CHUNK_OVERLAY_BIND", "udp://0.0.0.0:50262")
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


def _apply_tcp_pose_step_to_target(
    scene_handles: dict[str, Any],
    arm: str,
    step: tuple[float, float, float, float, float, float],
    frame_mode: str,
) -> bool:
    target = _tcp_target_position_wxyz(scene_handles, arm)
    if target is None:
        return False
    position, wxyz = target
    target_transform = _pose_transform(position, wxyz)
    delta_transform = _delta_transform(step)
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


def _policy_runner_control_active(
    handles: dict[str, Any],
    latest: StateSnapshot | None = None,
) -> bool:
    store = handles.get("recording_status_store")
    if isinstance(store, RecordingStatusStore) and not store.is_stale(threshold_sec=2.0):
        return True
    latest = latest or handles.get("_latest_state")
    if isinstance(latest, StateSnapshot):
        owner = latest.command_source.display_source_id
        if owner == "policy_runner":
            return True
    return False


def _active_runner_status_packet(
    handles: dict[str, Any],
    latest: StateSnapshot | None = None,
) -> dict[str, Any] | None:
    store = handles.get("recording_status_store")
    if not isinstance(store, RecordingStatusStore):
        return None
    latest = latest or handles.get("_latest_state")
    if isinstance(latest, StateSnapshot):
        source_id = latest.command_source.display_source_id
        session_id = latest.command_source.display_session_id
        packet = store.latest_for_session(source_id, session_id, threshold_sec=2.0)
        if packet is not None:
            return packet
        if source_id == "policy_runner":
            packet = store.latest_for_role("flow_infer", threshold_sec=2.0)
            if packet is not None:
                return packet
    packet = store.latest_packet()
    if packet is None or store.is_stale(threshold_sec=2.0):
        return None
    return packet


def _recording_client_for_endpoint(handles: dict[str, Any], endpoint: str) -> RecordingCommandClient:
    host, port = parse_udp_endpoint(endpoint, default_host="127.0.0.1", default_port=50441)
    key = f"{host}:{port}"
    clients = handles.setdefault("recording_cmd_clients", {})
    if not isinstance(clients, dict):
        clients = {}
        handles["recording_cmd_clients"] = clients
    client = clients.get(key)
    if not isinstance(client, RecordingCommandClient):
        client = RecordingCommandClient(host, port)
        clients[key] = client
    return client


def _arm_init_control_route(
    handles: dict[str, Any],
    latest: StateSnapshot | None,
) -> tuple[str, RecordingCommandClient | None, str]:
    latest = latest or handles.get("_latest_state")
    packet = _active_runner_status_packet(handles, latest)
    if packet is not None:
        endpoint = str(packet.get("control_endpoint", "") or "")
        if endpoint:
            role = str(packet.get("runner_role", "unknown") or "unknown")
            session = str(packet.get("command_session_id", "") or "")
            return (
                f"{role} policy_runner",
                _recording_client_for_endpoint(handles, endpoint),
                f"{endpoint} session={session}",
            )
    if isinstance(latest, StateSnapshot) and latest.command_source.display_source_id == "policy_runner":
        flow_endpoint = os.environ.get("RB_GUI_FLOW_RECORD_CMD_ENDPOINT", "").strip()
        if flow_endpoint:
            return (
                "flow-infer policy_runner (env fallback)",
                _recording_client_for_endpoint(handles, flow_endpoint),
                flow_endpoint,
            )
        default_endpoint = os.environ.get("RB_GUI_RECORD_CMD_ENDPOINT", "udp://127.0.0.1:50441")
        return (
            "stack policy_runner (env/default fallback)",
            _recording_client_for_endpoint(handles, default_endpoint),
            default_endpoint,
        )
    return ("direct server one-shot", None, "")


def _policy_runner_actively_controlling(
    handles: dict[str, Any],
    latest: StateSnapshot | None = None,
) -> bool:
    # Narrower than _policy_runner_control_active (which only means "policy_runner
    # is alive / publishing status"): True only when policy_runner is ACTIVELY
    # driving the robot — a recording/rollout is in progress, an arm_init latch is
    # already engaged (so a press can clear it), or it currently owns the command
    # lease. GUI InitMotion routes to policy_runner's arm_init latch only in those
    # cases; an idle-but-alive policy_runner must let a standalone GUI InitMotion
    # run as a direct one-shot command, otherwise the latch keeps policy_runner
    # streaming init_motion, it never releases the command lease (idle handoff
    # never fires), and later GUI Cartesian commands are rejected with
    # command_source_lease_conflict.
    latest = latest or handles.get("_latest_state")
    store = handles.get("recording_status_store")
    if isinstance(store, RecordingStatusStore) and not store.is_stale(threshold_sec=2.0):
        status = store.latest()
        if isinstance(status, Mapping) and bool(status.get("recording")):
            return True
        arm_init = store.latest_arm_init()
        if isinstance(arm_init, Mapping) and (
            bool(arm_init.get("init_override_left"))
            or bool(arm_init.get("init_override_right"))
        ):
            return True
    if isinstance(latest, StateSnapshot):
        if isinstance(latest.arm_init, Mapping) and (
            bool(latest.arm_init.get("init_override_left"))
            or bool(latest.arm_init.get("init_override_right"))
        ):
            return True
        if latest.command_source.display_source_id == "policy_runner":
            return True
    return False


def _arm_init_status_block(
    handles: dict[str, Any],
    latest: StateSnapshot | None = None,
) -> dict[str, Any]:
    packet = _active_runner_status_packet(handles, latest)
    if isinstance(packet, Mapping) and isinstance(packet.get("arm_init"), Mapping):
        return normalize_arm_init_status(packet["arm_init"])
    if latest is not None and isinstance(latest.arm_init, Mapping):
        return normalize_arm_init_status(latest.arm_init)
    store = handles.get("recording_status_store")
    if isinstance(store, RecordingStatusStore):
        block = store.latest_arm_init()
        if block is not None:
            return normalize_arm_init_status(block)
    local = handles.get("arm_init_local_status")
    if isinstance(local, Mapping):
        return normalize_arm_init_status(local)
    return normalize_arm_init_status(None)


def _optimistic_arm_init_start(handles: dict[str, Any], arms: str) -> dict[str, Any]:
    status = _arm_init_status_block(handles, handles.get("_latest_state"))
    left_on = bool(status.get("init_override_left", False))
    right_on = bool(status.get("init_override_right", False))
    if arms == "both":
        left_on = True
        right_on = True
    elif arms == "left":
        left_on = True
    elif arms == "right":
        right_on = True
    local = normalize_arm_init_status(
        {
            "init_override_left": left_on,
            "init_override_right": right_on,
            "last_command": arms,
        }
    )
    handles["arm_init_local_status"] = local
    return local


def _apply_arm_init_result(
    handles: dict[str, Any],
    result: ArmInitCommandResult | Any,
    *,
    arms: str,
) -> tuple[bool, str]:
    if isinstance(result, ArmInitCommandResult):
        ok = result.ok
        message = result.message
    else:
        ok = bool(result)
        message = f"arm init {arms} request sent" if ok else f"arm init {arms} request failed"
    if "arm_init_status" in handles:
        handles["arm_init_status"].value = ("OK: " if ok else "BLOCKED: ") + message
    return ok, message


def _send_arm_init_override(
    safety: OperatorSafety,
    scene_handles: dict[str, Any],
    handles: dict[str, Any],
    arms: str,
) -> tuple[bool, str]:
    if arms not in {"both", "left", "right"}:
        return False, f"invalid InitMotion arm selector: {arms}"
    if safety.init_left_joint_deg is None or safety.init_right_joint_deg is None:
        return False, "init motion target not configured"

    latest = handles.get("_latest_state")
    route_label, route_client, route_detail = _arm_init_control_route(
        handles,
        latest if isinstance(latest, StateSnapshot) else None,
    )
    client = route_client or handles.get("recording_cmd_client")
    send_arm_init = getattr(client, "send_arm_init", None)
    if callable(send_arm_init) and _policy_runner_actively_controlling(handles, latest):
        try:
            result = send_arm_init(
                arms,
                action="start",
                left_q_deg=safety.init_left_joint_deg,
                right_q_deg=safety.init_right_joint_deg,
            )
        except OSError as exc:
            result = ArmInitCommandResult(False, f"arm init {arms} send failed: {exc}")
        ok, message = _apply_arm_init_result(handles, result, arms=arms)
        if ok:
            handles["arm_init_route_status"] = {
                "route": route_label,
                "detail": route_detail,
            }
            _optimistic_arm_init_start(handles, arms)
            _clear_tcp_target_user_moved(
                scene_handles,
                ("left", "right") if arms == "both" else (arms,),
            )
            _update_arm_init_panel(
                handles,
                latest if isinstance(latest, StateSnapshot) else None,
                stale=bool(handles.get("_state_stale", True)),
            )
        return ok, f"{message}; routed to {route_label} {route_detail}".strip()

    if isinstance(latest, StateSnapshot) and latest.command_source.display_source_id == "policy_runner":
        return False, "policy_runner owns the command lease but no arm_init control endpoint is available"

    if arms == "both":
        handles["arm_init_route_status"] = {"route": "direct server one-shot", "detail": ""}
        return _send_init_motion_and_reset_targets(safety, scene_handles)

    ok, message = safety.send_init_motion_arm(arms)  # type: ignore[arg-type]
    if ok:
        handles["arm_init_route_status"] = {"route": "direct server one-shot", "detail": ""}
        _clear_tcp_target_user_moved(scene_handles, (arms,))
        message = f"{message}; TCP target will follow current TCP"
    return ok, message


def _send_arm_init_cancel_resume(
    handles: dict[str, Any],
    arms: str = "both",
) -> tuple[bool, str]:
    latest = handles.get("_latest_state")
    route_label, route_client, route_detail = _arm_init_control_route(
        handles,
        latest if isinstance(latest, StateSnapshot) else None,
    )
    client = route_client or handles.get("recording_cmd_client")
    send_arm_init = getattr(client, "send_arm_init", None)
    if callable(send_arm_init) and _policy_runner_control_active(handles, latest):
        try:
            result = send_arm_init(arms, action="cancel")
        except OSError as exc:
            result = ArmInitCommandResult(False, f"arm init {arms} cancel failed: {exc}")
        ok, message = _apply_arm_init_result(handles, result, arms=arms)
        if ok:
            handles["arm_init_route_status"] = {"route": route_label, "detail": route_detail}
            handles["arm_init_local_status"] = normalize_arm_init_status(
                {
                    "init_override_left": False,
                    "init_override_right": False,
                    "left_state": "cancelled",
                    "right_state": "cancelled",
                    "last_command": arms,
                }
            )
            _update_arm_init_panel(
                handles,
                latest if isinstance(latest, StateSnapshot) else None,
                stale=bool(handles.get("_state_stale", True)),
            )
        return ok, f"{message}; routed to {route_label} {route_detail}".strip()
    return False, "no active policy_runner InitMotion override to cancel/resume"


def _set_button_color(button: Any, color: str) -> None:
    try:
        button.color = color
    except Exception:
        pass


def _arm_init_display_text(state: str, active: bool) -> str:
    normalized = str(state or "").strip().lower()
    if normalized in {"policy", ""} and not active:
        return "policy 중"
    if "failed" in normalized:
        return "init failed"
    if "cancel" in normalized:
        return "cancelled"
    if "done" in normalized:
        return "init done"
    if "planning" in normalized:
        return "init planning"
    if "executing" in normalized or "requested" in normalized or active:
        return "InitMotion 중"
    return normalized or ("InitMotion 중" if active else "policy 중")


def _arm_init_failure_detail(status: Mapping[str, Any], side: str) -> str:
    message = str(status.get(f"{side}_message", "") or "")
    if message:
        return message
    name_a = str(status.get(f"{side}_goal_nearest_pair_name_a", "") or "")
    name_b = str(status.get(f"{side}_goal_nearest_pair_name_b", "") or "")
    clearance = status.get(f"{side}_goal_clearance_m")
    threshold = status.get(f"{side}_goal_threshold_m")
    if name_a or name_b:
        pair = f"{name_a or 'unknown'} <-> {name_b or 'unknown'}"
        if isinstance(clearance, (int, float)) and isinstance(threshold, (int, float)):
            return (
                f"goal_not_clear: pair {pair}, clearance {float(clearance) * 1000.0:.1f} mm "
                f"< required {float(threshold) * 1000.0:.1f} mm"
            )
        return f"goal_not_clear: pair {pair}"
    return str(status.get(f"{side}_fail_mode", "") or "")


def _update_arm_init_panel(
    handles: dict[str, Any],
    latest: StateSnapshot | None,
    *,
    stale: bool,
) -> None:
    status = _arm_init_status_block(handles, latest)
    left_on = bool(status.get("init_override_left", False))
    right_on = bool(status.get("init_override_right", False))
    policy_active = _policy_runner_control_active(handles, latest)
    store = handles.get("recording_status_store")
    status_stale = isinstance(store, RecordingStatusStore) and store.is_stale(threshold_sec=2.0)
    left_text = _arm_init_display_text(str(status.get("left_state", "") or ""), left_on)
    right_text = _arm_init_display_text(str(status.get("right_state", "") or ""), right_on)
    suffix = ""
    if not policy_active:
        suffix = " / policy_runner 상태 없음"
    elif stale or status_stale:
        suffix = " / stale"
    route = handles.get("arm_init_route_status")
    if isinstance(route, Mapping):
        route_label = str(route.get("route", "") or "")
        route_detail = str(route.get("detail", "") or "")
    else:
        route_label, _client, route_detail = _arm_init_control_route(handles, latest)
    if route_label:
        suffix += f" / route={route_label}"
        if route_detail:
            suffix += f" ({route_detail})"
    error = str(status.get("error", "") or "")
    if error:
        suffix += f" / error={error}"
    left_failed = "failed" in left_text.lower()
    right_failed = "failed" in right_text.lower()
    if left_failed:
        detail = _arm_init_failure_detail(status, "left")
        if detail:
            suffix += f" / left={detail}"
    if right_failed:
        detail = _arm_init_failure_detail(status, "right")
        if detail:
            suffix += f" / right={detail}"
    if "arm_init_status" in handles:
        handles["arm_init_status"].value = f"왼팔: {left_text} / 오른팔: {right_text}{suffix}"
    buttons = handles.get("init_motion_buttons", {})
    if isinstance(buttons, Mapping):
        _set_button_color(buttons.get("both"), "green" if left_on and right_on else "gray")
        _set_button_color(buttons.get("left"), "green" if left_on else "gray")
        _set_button_color(buttons.get("right"), "green" if right_on else "gray")


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
    *,
    arms: tuple[str, ...] | None = None,
) -> tuple[
    tuple[float, float, float, float, float, float] | None,
    tuple[float, float, float, float, float, float] | None,
    tuple[float, float, float, float] | None,
    tuple[float, float, float, float] | None,
    str | None,
]:
    # `arms`, when given, restricts which arms contribute a payload (the rest stay None).
    # Used by the TCP-linear send to forward only the arms the operator actually parked.
    selected_arms = arms if arms is not None else (("left", "right") if arm_group == "both" else (arm_group,))
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
    # Guard: when a TCP target marker has NOT been dragged/set, it follows current TCP
    # every tick (scene.update_scene_markers), so the "destination" equals where the arm
    # already is and the linear move is a near no-op (the symptom: it only creeps a few
    # degrees per click). Require the operator to park the marker at a destination first.
    # The {arm}_tcp_target_user_moved flag is set on drag/set and cleared by
    # the init-motion button.
    # ("TCP targets will follow current TCP").
    selected_arms = ("left", "right") if arm_group == "both" else (arm_group,)
    parked = tuple(arm for arm in selected_arms
                   if f"{arm}_tcp_target_user_moved" in scene_handles)
    # Only block when NO selected arm has been parked at a destination. In "both" mode
    # moving a single gizmo is a valid "move just this arm" request: send only the parked
    # arm(s) and let the other Hold, instead of blocking the whole command.
    if not parked:
        arms_txt = " and ".join(selected_arms)
        return False, (
            f"{arms_txt} TCP target marker is following current TCP "
            "(not parked at a destination) — drag/set the marker to the target first, "
            "otherwise the linear move is ~zero"
        )
    left_pose, right_pose, left_quaternion_xyzw, right_quaternion_xyzw, error = _tcp_marker_payloads(scene_handles, arm_group, arms=parked)
    if error:
        return False, error
    # DIAGNOSTIC: log the target the GUI is about to send + its drag delta from the
    # current measured TCP marker. This disambiguates "arm only goes partway and stops":
    # a large delta here means the GUI captured the full drag (so a short server move is a
    # planner/collision issue), while a tiny delta means the captured target itself was
    # near current TCP (a GUI marker bug). scene_handles[f"{arm}_tcp"] is a viser
    # FrameHandle, so read its .position; wrap everything so logging never blocks the send.
    try:
        for arm, pose in (("left", left_pose), ("right", right_pose)):
            if pose is None:
                continue
            handle = scene_handles.get(f"{arm}_tcp")
            current = getattr(handle, "position", None)
            if current is not None and len(current) >= 3:
                cx, cy, cz = float(current[0]), float(current[1]), float(current[2])
                delta_m = math.sqrt((pose[0] - cx) ** 2 + (pose[1] - cy) ** 2 + (pose[2] - cz) ** 2)
                current_txt = f"({cx:.4f}, {cy:.4f}, {cz:.4f})"
            else:
                delta_m = float("nan")
                current_txt = "unknown"
            print(
                f"rb_servo_gui: TcpLinear send {arm} target_xyz="
                f"({pose[0]:.4f}, {pose[1]:.4f}, {pose[2]:.4f}) "
                f"rpy=({pose[3]:.4f}, {pose[4]:.4f}, {pose[5]:.4f}) "
                f"current_xyz={current_txt} drag_delta={delta_m:.4f} m "
                f"orientation_mode={orientation_mode}",
                flush=True,
            )
    except Exception as exc:  # noqa: BLE001 - diagnostic logging must never block the send
        print(f"rb_servo_gui: TcpLinear send diagnostic log failed ({type(exc).__name__}: {exc})", flush=True)
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


def _apply_tcp_pose_step_and_send_pose_target(
    safety: OperatorSafety,
    scene_handles: dict[str, Any],
    arm: str,
    step: tuple[float, float, float, float, float, float],
    frame_mode: str,
) -> tuple[bool, str]:
    arms = ("left", "right") if arm == "both" else (arm,)
    for single_arm in arms:
        if not _apply_tcp_pose_step_to_target(scene_handles, single_arm, step, frame_mode):
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
        # 원자적 쓰기(temp+replace): camera_server stereo_worker가 1Hz로 이 파일을 읽으므로
        # 부분 기록 상태를 읽지 않게 한다(truncated json → 워커가 기본 ROI로 폴백하는 깜빡임 방지).
        tmp = f"{path}.tmp"
        with open(tmp, "w", encoding="utf-8") as handle:
            json.dump(dict(settings), handle, indent=2, sort_keys=True)
        os.replace(tmp, path)
    except OSError as exc:
        return False, str(exc)
    return True, path


def _update_gui_setting(key: str, value: Any) -> None:
    """Read-modify-write a single GUI preference. Best-effort: persistence
    failures are swallowed so a read-only home dir never breaks the toggle."""
    settings = _load_gui_settings()
    settings[key] = value
    _save_gui_settings(settings)


def _gui_setting_bool(settings: Mapping[str, Any], key: str, default: bool) -> bool:
    try:
        value = settings.get(key, default)
        if isinstance(value, str):
            lowered = value.strip().lower()
            if lowered in {"1", "true", "yes", "on"}:
                return True
            if lowered in {"0", "false", "no", "off", ""}:
                return False
            raise ValueError
        if isinstance(value, (list, dict)):
            raise ValueError
        return bool(value)
    except Exception:
        return bool(default)


def _gui_setting_float(settings: Mapping[str, Any], key: str, default: float) -> float:
    try:
        value = float(settings.get(key, default))
        if not math.isfinite(value):
            raise ValueError
        return value
    except Exception:
        return float(default)


def _roi_initial_slider_specs(
    settings: Mapping[str, Any],
) -> dict[str, tuple[float, float, float, float]]:
    """Return bootstrap slider ranges and values in millimetres.

    The authoritative range arrives shortly afterwards in the server state
    ``roi_box.runtime_min_m/runtime_max_m`` block.  Until that first state is
    received, the slider still has to be constructible: viser rejects an
    initial value outside its declared range.  Include valid persisted values
    in the temporary range so a wider server envelope from a previous session
    cannot terminate the GUI during startup.
    """
    defaults = {
        "x": (-500.0, 500.0),
        "y": (-1000.0, 0.0),
        "z": (0.0, 1000.0),
    }
    saved_lo = _roi_bounds_floats(settings.get("roi_min_m"))
    saved_hi = _roi_bounds_floats(settings.get("roi_max_m"))
    if saved_lo is not None and saved_hi is not None and all(
        saved_lo[k] <= saved_hi[k] for k in range(3)
    ):
        values = {
            axis: (saved_lo[k] * 1000.0, saved_hi[k] * 1000.0)
            for k, axis in enumerate(("x", "y", "z"))
        }
    else:
        values = defaults

    return {
        axis: (
            min(-1500.0, lo_mm),
            max(1500.0, hi_mm),
            lo_mm,
            hi_mm,
        )
        for axis, (lo_mm, hi_mm) in values.items()
    }


def _gui_setting_int(settings: Mapping[str, Any], key: str, default: int) -> int:
    try:
        value = int(settings.get(key, default))
        if value < 0:
            raise ValueError
        return value
    except Exception:
        return int(default)


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


def _user_floor_display_points(handles: dict[str, Any]) -> list:
    """Captured contact points to RENDER, gated by the 'Show capture points'
    toggle (default OFF). Returns [] when the toggle is missing or off, which
    makes update_user_floor_capture_points hide the cyan/magenta point cloud."""
    toggle = handles.get("user_floor_show_points_toggle")
    if toggle is None or not bool(getattr(toggle, "value", False)):
        return []
    points = _user_floor_state(handles).get("points", [])
    return points if isinstance(points, list) else []


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
    """Set the init-motion target from edited text and apply it at runtime.

    Updates the live OperatorSafety target (used immediately by the next
    init-motion press) AND persists to init_motion.json — no restart needed.
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
    return True, f"Init Motion updated live; {suffix}"


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


def _build_camera_quality_monitor(
    server: Any,
    handles: dict[str, Any],
    *,
    order: float | None = None,
) -> None:
    with server.gui.add_folder(
        "Camera Quality Monitor",
        expand_by_default=True,
        order=order,
    ):
        handles["camera_quality_monitor_status"] = server.gui.add_text(
            "Status",
            initial_value="disabled",
            disabled=True,
        )
        handles["camera_quality_monitor_values"] = {"right": {}, "left": {}}
        for arm in ("right", "left"):
            with server.gui.add_folder(arm.upper(), expand_by_default=True):
                handles["camera_quality_monitor_values"][arm]["blur"] = (
                    server.gui.add_text(
                        "Blur",
                        initial_value="N/A",
                        disabled=True,
                    )
                )
                handles["camera_quality_monitor_values"][arm]["shake"] = (
                    server.gui.add_text(
                        "Shake [px/s]",
                        initial_value="N/A",
                        disabled=True,
                    )
                )


def _operator_monitor_layout() -> tuple[float, float, float, float]:
    return (
        _env_positive_float("RB_GUI_MONITOR_WIDTH_EM", _OPERATOR_MONITOR_WIDTH_EM),
        _env_positive_float("RB_GUI_MONITOR_GAP_EM", _OPERATOR_MONITOR_GAP_EM),
        _env_positive_float("RB_GUI_MONITOR_SPLIT_EM", _OPERATOR_MONITOR_SPLIT_EM),
        _env_positive_float("RB_GUI_MONITOR_SPLIT_FT_EM", _OPERATOR_MONITOR_SPLIT_FT_EM),
    )


def _operator_monitor_static_html(
    monitor_width_em: float,
    gap_em: float,
    split_em: float,
    split_ft_em: float,
) -> str:
    return f"""
<style>
  :root {{
    --rb-monitor-gap: {gap_em:.3f}em;
    --rb-monitor-target-width: {monitor_width_em:.3f}em;
    --rb-monitor-width: min(
      var(--rb-monitor-target-width),
      max(13.5em, calc((100vw - (3 * var(--rb-monitor-gap))) / 2))
    );
    /* Lower monitor cards stack below the upper cards at this common vertical
       anchor (em, in card font size), clamped for short viewports. */
    --rb-monitor-split: min({split_em:.3f}em, 60vh);
    /* The FT/Camera column's own anchor — see _OPERATOR_MONITOR_SPLIT_FT_EM. Bounded
       by what Camera Quality actually needs (it is seven fixed lines, ~21em with its
       header) rather than by a fraction of the viewport, so the FT table gets every
       pixel the column can spare before it has to scroll. */
    --rb-monitor-split-ft: min({split_ft_em:.3f}em, max(26em, calc(100vh - 21em)));
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
  /* Joint/Pose share the left column; FT/Camera Quality share the right column.
     The common viewport split keeps every static header stable across dynamic
     body refreshes. */
  .rb-monitor-joint-card {{ left: var(--rb-monitor-gap); }}
  .rb-monitor-stand-card {{ left: var(--rb-monitor-gap); }}
  .rb-monitor-ft-card {{ left: calc(var(--rb-monitor-gap) * 2 + var(--rb-monitor-width)); }}
  .rb-monitor-camera-card {{ left: calc(var(--rb-monitor-gap) * 2 + var(--rb-monitor-width)); }}
  .rb-monitor-joint-card.rb-monitor-body-card {{ max-height: calc(var(--rb-monitor-split) - 5.95em); }}
  .rb-monitor-stand-card.rb-monitor-header-card {{ top: var(--rb-monitor-split); }}
  .rb-monitor-stand-card.rb-monitor-body-card {{ top: calc(var(--rb-monitor-split) + 4.95em); max-height: calc(100vh - var(--rb-monitor-split) - 5.95em); }}
  .rb-monitor-ft-card.rb-monitor-body-card {{ max-height: calc(var(--rb-monitor-split-ft) - 5.95em); }}
  .rb-monitor-camera-card.rb-monitor-header-card {{ top: var(--rb-monitor-split-ft); }}
  .rb-monitor-camera-card.rb-monitor-body-card {{ top: calc(var(--rb-monitor-split-ft) + 4.95em); max-height: calc(100vh - var(--rb-monitor-split-ft) - 5.95em); }}
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
  /* THE FT CARD IS A TABLE, NOT TWO STACKS. The operator's question is "which arm
     is feeling what", and that is a comparison — a per-arm stack forced them to
     scroll between the two halves of one answer. One row per channel, one column
     per arm, and both arms land in the same glance. */
  .rb-monitor-row3 {{
    /* The label takes what it needs and THE TWO ARMS SPLIT THE REST EVENLY. Sizing
       the arm columns to their content instead let a wide cell in one arm eat the
       other's gap, and the two values ran together. */
    grid-template-columns: auto 1fr 1fr;
    column-gap: 0.5em;
    /* Slightly tighter than the two-column cards: this one is a TABLE, it reads
       better dense, and the ~19 px it saves is what keeps the worst-case card
       (push + covering law) inside its box on a remote-desktop-sized viewport. */
    line-height: 1.45;
  }}
  .rb-monitor-row3 > span {{ text-align: right; font-variant-numeric: tabular-nums; }}
  .rb-monitor-row3 > span:first-child {{ text-align: left; }}
  .rb-monitor-row-head {{
    font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    font-weight: 600;
    font-size: 11px;
    border-bottom: 1px solid rgba(15, 23, 42, 0.12);
    margin-bottom: 0.15em;
  }}
  /* Force and torque are different units; the blank line is the unit boundary. */
  .rb-monitor-row-gap {{ margin-top: 0.4em; }}
  .rb-monitor-dim {{ color: #9aa5b1; }}
  .rb-monitor-warn {{ color: #8a6410; font-weight: 600; }}
  .rb-monitor-bad {{ color: #b34646; font-weight: 600; }}
  .rb-monitor-ok {{ color: #148a4e; }}
  /* A failure needs a SENTENCE, and a sentence does not fit an arm column. It gets
     its own full-width line so the columns stay narrow enough to read as a table. */
  .rb-monitor-note {{
    font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    font-size: 11px;
    line-height: 1.35;
    margin-top: 0.15em;
  }}
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
    /* Re-state the 3-column template: the `.rb-monitor-row` rule above resets it to
       two columns at the same specificity and later in the sheet, and the third cell
       would silently wrap onto a second line instead of failing loudly. */
    .rb-monitor-row3 {{
      grid-template-columns: auto 1fr 1fr;
      column-gap: 0.35em;
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
<div class="rb-monitor-card rb-monitor-header-card rb-monitor-ft-card">
  <div class="rb-monitor-title">FT Monitor</div>
</div>
<div class="rb-monitor-card rb-monitor-header-card rb-monitor-camera-card">
  <div class="rb-monitor-title">Camera Quality Monitor</div>
</div>
"""


def _operator_monitor_value_pair(deg_value: str, rad_value: str) -> str:
    return f'<span class="rb-deg">{escape(deg_value)}</span><span class="rb-rad">{escape(rad_value)}</span>'


def _operator_monitor_invalid_pair() -> str:
    return _operator_monitor_value_pair("invalid", "invalid")


def _operator_monitor_row(label: str, value_html: str) -> str:
    return f'<div class="rb-monitor-row"><span>{escape(label)}</span><span>{value_html}</span></div>'


def _operator_monitor_row3(
    label: str,
    left_html: str,
    right_html: str,
    *,
    row_class: str = "",
) -> str:
    """One `label | left | right` line. The value cells are HTML, the label is text."""
    classes = "rb-monitor-row rb-monitor-row3"
    if row_class:
        classes += " " + row_class
    return (
        f'<div class="{classes}"><span>{escape(label)}</span>'
        f"<span>{left_html}</span><span>{right_html}</span></div>"
    )


def _camera_quality_monitor_status(store: CameraQualityStore | None) -> str:
    if store is None:
        return "disabled"
    status = store.status()
    text = f"receiver={status['receiver']}"
    if status["error"]:
        text += f" · {status['error']}"
    return text


def _camera_quality_monitor_values(
    store: CameraQualityStore | None,
    arm: str,
    *,
    now: float,
) -> tuple[str, str]:
    if store is None:
        return "N/A", "N/A"
    sample = store.latest(arm)
    if (
        sample is None
        or now - sample.received_monotonic > _CAMERA_QUALITY_MONITOR_STALE_SEC
    ):
        return "N/A", "N/A"
    blur = (
        f"{sample.blur_effect:.3f}"
        if math.isfinite(sample.blur_effect)
        else "N/A"
    )
    shake = (
        f"{sample.shake_px_s_rms:.1f}"
        if sample.shake_px_s_rms is not None
        and math.isfinite(sample.shake_px_s_rms)
        else "N/A"
    )
    return blur, shake


def _render_camera_quality_monitor_rows(
    store: CameraQualityStore | None,
    *,
    now: float | None = None,
) -> str:
    timestamp = time.monotonic() if now is None else float(now)
    parts = [
        '<div class="rb-monitor-status">'
        + escape(_camera_quality_monitor_status(store))
        + "</div>"
    ]
    for arm in ("right", "left"):
        blur, shake = _camera_quality_monitor_values(
            store,
            arm,
            now=timestamp,
        )
        parts.append(
            '<div class="rb-monitor-arm"><div class="rb-monitor-arm-title">'
            + arm.upper()
            + "</div>"
        )
        parts.append(_operator_monitor_row("Blur", escape(blur)))
        parts.append(_operator_monitor_row("Shake [px/s]", escape(shake)))
        parts.append("</div>")
    return "".join(parts)


def _update_camera_quality_monitor_fallback(
    handles: dict[str, Any],
    *,
    now: float | None = None,
) -> None:
    store = handles.get("camera_quality_store")
    if not isinstance(store, CameraQualityStore):
        store = None
    status_handle = handles.get("camera_quality_monitor_status")
    if status_handle is not None:
        status_handle.value = _camera_quality_monitor_status(store)
    timestamp = time.monotonic() if now is None else float(now)
    values = handles.get("camera_quality_monitor_values")
    if not isinstance(values, Mapping):
        return
    for arm in ("right", "left"):
        blur, shake = _camera_quality_monitor_values(
            store,
            arm,
            now=timestamp,
        )
        arm_values = values.get(arm)
        if not isinstance(arm_values, Mapping):
            continue
        blur_handle = arm_values.get("blur")
        if blur_handle is not None:
            blur_handle.value = blur
        shake_handle = arm_values.get("shake")
        if shake_handle is not None:
            shake_handle.value = shake


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


# The server's per-axis dead-band on the compensated channels
# (`force_torque.<arm>.deadzone_force_n` / `deadzone_torque_nm`). Repeated here ONLY
# to caption the card: a resting arm reads 0.00 and so does the first ~2 N of a push,
# and an operator who does not know that reads a working sensor as a dead one.
_FT_DEADZONE_N = 2.0
_FT_DEADZONE_NM = 0.5
# Below this the deadzoned wrench is mostly the band's own residue, so the
# moment-arm ratio is a ratio of two near-zero numbers.
_FT_LEVER_MIN_FORCE_N = 3.0


def _ft_dim(text: str) -> str:
    return f'<span class="rb-monitor-dim">{escape(text)}</span>'


class _FtArmReading(NamedTuple):
    """One arm's F/T read.

    `token` is what fits an arm COLUMN (two or three characters); `note` is the
    sentence that does not, and gets its own full-width line under the table.
    `values` of None means there is NOTHING TO READ and the card prints "--"; a list
    of zeros means a REAL ZERO — not the same thing, and the whole reason the two are
    kept apart here instead of both collapsing to 0.00.
    """

    token: str
    css_class: str
    note: str
    values: list[float] | None
    ft: Mapping[str, Any] | None


def _ft_arm_reading(arm: Any) -> _FtArmReading:
    ft = getattr(arm, "force_torque", None)
    if not isinstance(ft, Mapping):
        # ABSENT is not OFF. An absent block means the server is not publishing
        # force_torque at all (old binary, or a GUI started before it did) — a config
        # choice and a missing stream must not share a message.
        return _FtArmReading(
            "--", "rb-monitor-bad", "no force_torque in the state stream", None, None)
    if not ft.get("enabled"):
        return _FtArmReading(
            "off", "rb-monitor-dim", "disabled in config (force_torque.<arm>.enable)",
            None, ft)
    if not ft.get("connected"):
        # Every compensated channel is pinned to exact zero upstream, so a printed
        # 0.00 here would read as "no force" when it means "no sensor".
        reason = str(ft.get("connect_reason") or "sensor stream is flat")
        return _FtArmReading("--", "rb-monitor-bad", f"NOT CONNECTED - {reason}", None, ft)

    wrench = ft.get("comp_stand_axes_at_tcp")
    values: list[float] = []
    if isinstance(wrench, (list, tuple)) and len(wrench) >= 6:
        for raw_value in wrench[:6]:
            try:
                parsed = float(raw_value)
            except (TypeError, ValueError):
                values = []
                break
            if not math.isfinite(parsed):
                values = []
                break
            values.append(parsed)
    if len(values) != 6:
        return _FtArmReading(
            "--", "rb-monitor-bad", "the published wrench is not six finite numbers",
            None, ft)

    if not ft.get("bias_valid"):
        # NOT ZEROED READS AS A TRUE ZERO, ON PURPOSE. Without a tare the compensated
        # wrench still carries the sensor's own offset (~20-40 N on this cell), and the
        # server refuses to let any law cover an untared arm ("F/T has no bias yet"),
        # so nothing whatsoever is acting on those numbers. Zero is the honest reading
        # of "no measured change from a zero you have not set yet".
        return _FtArmReading(
            "필요", "rb-monitor-warn", "영점 조절 전 - F/T 영점 버튼을 누르세요",
            [0.0] * 6, ft)
    return _FtArmReading("OK", "rb-monitor-ok", "", values, ft)


def _ft_value_cells(
    left_values: list[float] | None,
    right_values: list[float] | None,
    index: int,
    digits: int,
) -> tuple[str, str]:
    def cell(values: list[float] | None) -> str:
        if values is None:
            return _ft_dim("--")
        text = f"{values[index]:.{digits}f}"
        # A channel that ROUNDS to zero prints "0.00", never "-0.00": the sign of a
        # value the deadzone already flattened is noise, and a minus in front of a
        # zero reads as a direction nobody is pushing in.
        if float(text) == 0.0:
            text = f"{0.0:.{digits}f}"
        return escape(text)

    return cell(left_values), cell(right_values)


def _ft_magnitude_cells(
    left_values: list[float] | None,
    right_values: list[float] | None,
    offset: int,
    digits: int,
) -> tuple[str, str]:
    def cell(values: list[float] | None) -> str:
        if values is None:
            return _ft_dim("--")
        trio = values[offset:offset + 3]
        return escape(f"{math.sqrt(sum(v * v for v in trio)):.{digits}f}")

    return cell(left_values), cell(right_values)


def _ft_force_magnitude(values: list[float] | None) -> float:
    if values is None:
        return 0.0
    return math.sqrt(sum(v * v for v in values[0:3]))


def _ft_lever_cell(values: list[float] | None) -> str:
    f_mag = _ft_force_magnitude(values)
    if values is None or f_mag <= _FT_LEVER_MIN_FORCE_N:
        return _ft_dim("--")
    t_mag = math.sqrt(sum(v * v for v in values[3:6]))
    return escape(f"{t_mag / f_mag * 1e3:.0f}")


def _ft_load_cell(ft: Mapping[str, Any] | None, values: list[float] | None) -> str:
    """The tool-load estimate is the one channel that ESCAPES the deadzone (a heavy
    low-pass on the pre-deadzone force), so it reads the ~200 g the 2 N band flattens
    to zero. It is meaningless without a bias, so it follows the same gate."""
    if ft is None or values is None or not ft.get("bias_valid"):
        return _ft_dim("--")
    try:
        load = float(ft.get("load_mass_kg"))
    except (TypeError, ValueError):
        return _ft_dim("--")
    if not math.isfinite(load):
        return _ft_dim("--")
    text = f"{load:.3f}"
    return escape(text) if ft.get("load_settled") else _ft_dim(text + "~")


def _ft_fc_cell(fc: Mapping[str, Any] | None, key: str, scale: float, digits: int) -> str:
    if not isinstance(fc, Mapping) or not fc.get("covered"):
        return _ft_dim("--")
    try:
        value = float(fc.get(key, 0.0)) * scale
    except (TypeError, ValueError):
        return _ft_dim("--")
    if not math.isfinite(value):
        return _ft_dim("--")
    return escape(f"{value:.{digits}f}")


def _ft_law_cell(fc: Mapping[str, Any] | None) -> str:
    """WHICH LAW. The stream law and the hold law differ by 5x in the ratio that
    decides how much of a push turns the tool rather than moving it, so a deviation
    cannot be judged without knowing which one produced it."""
    if not isinstance(fc, Mapping):
        return _ft_dim("--")
    if not fc.get("enabled"):
        return _ft_dim("off")
    if not fc.get("covered"):
        return _ft_dim("idle")
    # Past the fence the law HOLDS the bound instead of tracking, so the arm feels
    # stiff for no visible reason unless the card says so.
    if fc.get("bounded"):
        return '<span class="rb-monitor-warn">fence</span>'
    law = fc.get("law")
    return escape(str(law)) if law else escape("on")


def _ft_safety_tracking(arm: Any) -> Mapping[str, Any] | None:
    """The per-arm safety_tracking block from the raw state, or None."""
    block = getattr(arm, "safety_tracking", None)
    return block if isinstance(block, Mapping) else None


def _ft_track_cell(track: Mapping[str, Any] | None, key: str, *,
                   need_reference: bool = False) -> str:
    """One tracking-error cell. "--" means there is nothing to read, which is a
    different answer from 0.00 — and for the reference error it is the honest one
    when the controller is not reporting a joint reference at all."""
    if not isinstance(track, Mapping):
        return _ft_dim("--")
    if need_reference and not track.get("reference_valid"):
        return _ft_dim("--")
    try:
        return escape(f"{float(track.get(key)):.2f}")
    except (TypeError, ValueError):
        return _ft_dim("--")


def _render_ft_monitor_rows(latest: StateSnapshot | None, *, stale: bool) -> str:
    """The FT Monitor card: what each arm is feeling RIGHT NOW, relative to its zero.

    ONE TABLE, BOTH ARMS, NO SCROLL. Every row is a channel and every column is an
    arm, because the question this card answers is a comparison and the previous
    per-arm stack put the two halves of that answer a scroll apart.

    THE NUMBERS ARE THE COMPENSATED, DEADZONED WRENCH AT THE TCP IN STAND AXES —
    `force_torque.comp_stand_axes_at_tcp`, the same surface the force law consumes.
    That is what "change from the zero" means here and it is already three
    subtractions deep:

        raw  -  bias(tare)  -  tool gravity  ->  2 N / 0.5 Nm deadzone

    So a resting arm reads 0.00, and the first ~2 N of any push reads 0.00 as well:
    the deadzone is not a display choice, it is what the controller acts on, and a
    monitor that showed the pre-deadzone value would disagree with the arm.

    STAND AXES, NOT TOOL AXES: this card is read by a human standing at the cell,
    and stand X/Y/Z are the directions they can point at. The law integrates in the
    same frame.

    "--" AND "0.00" ARE DIFFERENT ANSWERS. "--" is "there is nothing to read here";
    0.00 is a measurement. Before a tare the channels are a true zero (see
    `_ft_arm_reading`); with no sensor, no stream or a bad wrench they are "--".
    """
    if latest is None:
        status = "No state stream"
    else:
        status = (
            ("stale" if stale else "live")
            + " · stand axes @TCP · deadband "
            + f"{_FT_DEADZONE_N:g} N / {_FT_DEADZONE_NM:g} Nm"
        )
    rows: list[str] = [f'<div class="rb-monitor-status">{escape(status)}</div>']
    if latest is None or stale:
        return "".join(rows)

    left_read = _ft_arm_reading(latest.left)
    right_read = _ft_arm_reading(latest.right)
    left_values, right_values = left_read.values, right_read.values
    left_ft, right_ft = left_read.ft, right_read.ft
    left_fc = getattr(latest.left, "force_control", None)
    right_fc = getattr(latest.right, "force_control", None)

    rows.append(_operator_monitor_row3("", "LEFT", "RIGHT", row_class="rb-monitor-row-head"))
    rows.append(_operator_monitor_row3(
        "영점",
        f'<span class="{left_read.css_class}">{escape(left_read.token)}</span>',
        f'<span class="{right_read.css_class}">{escape(right_read.token)}</span>',
    ))

    for label, index in (("Fx [N]", 0), ("Fy [N]", 1), ("Fz [N]", 2)):
        left_cell, right_cell = _ft_value_cells(left_values, right_values, index, 2)
        rows.append(_operator_monitor_row3(
            label, left_cell, right_cell,
            row_class="rb-monitor-row-gap" if index == 0 else "",
        ))
    left_cell, right_cell = _ft_magnitude_cells(left_values, right_values, 0, 2)
    rows.append(_operator_monitor_row3("|F| [N]", left_cell, right_cell))

    for label, index in (("Tx [Nm]", 3), ("Ty [Nm]", 4), ("Tz [Nm]", 5)):
        left_cell, right_cell = _ft_value_cells(left_values, right_values, index, 3)
        rows.append(_operator_monitor_row3(
            label, left_cell, right_cell,
            row_class="rb-monitor-row-gap" if index == 3 else "",
        ))
    left_cell, right_cell = _ft_magnitude_cells(left_values, right_values, 3, 3)
    rows.append(_operator_monitor_row3("|T| [Nm]", left_cell, right_cell))

    # THE MOMENT ARM, |dT| / |dF|: the perpendicular distance from the reference point
    # to the line of action of the measured force. It is the ONE number that settles
    # "is this wrench really referenced at the fingertip" — push exactly on the
    # fingertip and it reads ~0 mm if the reference is the TCP, ~203 mm if it is still
    # the sensor origin — and it is frame-free, assuming nothing about which axis the
    # tool points along. Below ~3 N the deadzoned wrench is mostly the band's residue
    # and the ratio of two near-zero numbers is noise, so the row only appears while
    # somebody is actually pushing. That is also what keeps this card scroll-free.
    if any(_ft_force_magnitude(v) > _FT_LEVER_MIN_FORCE_N for v in (left_values, right_values)):
        rows.append(_operator_monitor_row3(
            "lever [mm]", _ft_lever_cell(left_values), _ft_lever_cell(right_values)))

    rows.append(_operator_monitor_row3(
        "load [kg]",
        _ft_load_cell(left_ft, left_values),
        _ft_load_cell(right_ft, right_values),
        row_class="rb-monitor-row-gap",
    ))

    # What the law did with it, so the cause and the effect are read together. The
    # deviation rows only appear while a law is actually covering an arm — an idle
    # cell would be three more rows of "--" on a card whose point is to stay short.
    rows.append(_operator_monitor_row3(
        "law", _ft_law_cell(left_fc), _ft_law_cell(right_fc)))
    covering = any(
        isinstance(fc, Mapping) and fc.get("covered") for fc in (left_fc, right_fc)
    )
    if covering:
        rows.append(_operator_monitor_row3(
            "dev [mm]",
            _ft_fc_cell(left_fc, "deviation_norm_m", 1e3, 1),
            _ft_fc_cell(right_fc, "deviation_norm_m", 1e3, 1),
        ))
        rows.append(_operator_monitor_row3(
            "dev [deg]",
            _ft_fc_cell(left_fc, "deviation_norm_rad", 180.0 / math.pi, 1),
            _ft_fc_cell(right_fc, "deviation_norm_rad", 180.0 / math.pi, 1),
        ))
        rows.append(_operator_monitor_row3(
            "gate",
            _ft_fc_cell(left_fc, "gate_translation", 1.0, 2),
            _ft_fc_cell(right_fc, "gate_translation", 1.0, 2),
        ))

    # THE TWO TRACKING ERRORS, and they blame different subsystems. The latch can
    # only report one number, and on 2026-08-26 it reported the wrong one: it said
    # "tracking error" while each arm was following its OWN controller reference to
    # 0.00 deg. The arm was fine; the box had stopped taking our commands.
    #
    #   cmd-act large, ref-act small -> the CONTROLLER is not executing what we send
    #   ref-act large                -> the ARM is in trouble (collision, overload)
    #
    # Shown on this card rather than a safety one because force control is what makes
    # them diverge: a compliant command deliberately leaves the arm behind, so a
    # reader here needs to know how much of the gap is the compliance and how much is
    # a link that has stopped.
    left_track = _ft_safety_tracking(latest.left)
    right_track = _ft_safety_tracking(latest.right)
    if left_track is not None or right_track is not None:
        rows.append(_operator_monitor_row3(
            "cmd-act [deg]",
            _ft_track_cell(left_track, "command_vs_actual_deg"),
            _ft_track_cell(right_track, "command_vs_actual_deg"),
            row_class="rb-monitor-row-gap",
        ))
        rows.append(_operator_monitor_row3(
            "ref-act [deg]",
            _ft_track_cell(left_track, "reference_vs_actual_deg", need_reference=True),
            _ft_track_cell(right_track, "reference_vs_actual_deg", need_reference=True),
        ))

    # A FAILURE NEEDS A SENTENCE, and a sentence does not fit an arm column. "--" in
    # the table says an arm has nothing to read; this says why, once, in full width.
    # Both arms usually fail the same way, so an identical note is printed once.
    seen: list[str] = []
    for arm_name, read in (("left", left_read), ("right", right_read)):
        if not read.note or read.note in seen:
            continue
        seen.append(read.note)
        both = left_read.note == right_read.note
        who = "both arms" if both else arm_name
        rows.append(
            f'<div class="rb-monitor-note {read.css_class}">'
            f"{escape(who)}: {escape(read.note)}</div>"
        )

    return "".join(rows)


def _operator_monitor_dynamic_html(
    latest: StateSnapshot | None,
    *,
    stale: bool,
    uptime: str | None = None,
    camera_quality_store: CameraQualityStore | None = None,
    now: float | None = None,
) -> str:
    return (
        '<div class="rb-monitor-card rb-monitor-body-card rb-monitor-joint-card">'
        + _render_joint_monitor_rows(latest, stale=stale, uptime=uptime)
        + "</div>"
        + '<div class="rb-monitor-card rb-monitor-body-card rb-monitor-stand-card">'
        + _render_stand_world_monitor_rows(latest, stale=stale, uptime=uptime)
        + "</div>"
        + '<div class="rb-monitor-card rb-monitor-body-card rb-monitor-ft-card">'
        + _render_ft_monitor_rows(latest, stale=stale)
        + "</div>"
        + '<div class="rb-monitor-card rb-monitor-body-card rb-monitor-camera-card">'
        + _render_camera_quality_monitor_rows(camera_quality_store, now=now)
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
        monitor_width_em, gap_em, split_em, split_ft_em = _operator_monitor_layout()
        handles["operator_monitor_panel_mode"] = "fixed_html_overlay"
        handles["operator_monitor_style"] = add_html(
            _operator_monitor_static_html(monitor_width_em, gap_em, split_em, split_ft_em),
            order=0.0,
        )
        camera_quality_store = handles.get("camera_quality_store")
        handles["operator_monitor_content"] = add_html(
            _operator_monitor_dynamic_html(
                None,
                stale=True,
                camera_quality_store=(
                    camera_quality_store
                    if isinstance(camera_quality_store, CameraQualityStore)
                    else None
                ),
            ),
            order=0.1,
        )
        return
    handles["operator_monitor_panel_mode"] = "root_gui_fallback"
    folder = server.gui.add_folder("Operator Monitors", expand_by_default=True, order=0.0)
    handles["operator_monitor_folder"] = folder
    with folder:
        _build_joint_monitor(server, handles, order=0.0)
        _build_stand_world_monitor(server, handles, order=0.1)
        _build_camera_quality_monitor(server, handles, order=0.2)


def _update_operator_monitors(handles: dict[str, Any], latest: StateSnapshot | None, *, stale: bool) -> None:
    content = handles.get("operator_monitor_content")
    uptime = _server_uptime_hms(handles, latest)
    camera_quality_store = handles.get("camera_quality_store")
    if not isinstance(camera_quality_store, CameraQualityStore):
        camera_quality_store = None
    now = time.monotonic()
    if content is not None:
        try:
            content.content = _operator_monitor_dynamic_html(
                latest,
                stale=stale,
                uptime=uptime,
                camera_quality_store=camera_quality_store,
                now=now,
            )
            return
        except Exception:
            pass
    _update_joint_monitor(handles, latest, stale=stale)
    _update_stand_world_monitor(handles, latest, stale=stale)
    _update_camera_quality_monitor_fallback(handles, now=now)


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


def _update_realtime_health(
    handles: dict[str, Any],
    latest: StateSnapshot | None,
    chunk_overlay_store: ChunkOverlayStore | None,
    *,
    stale: bool,
) -> None:
    handle = handles.get("realtime_health")
    chunk = chunk_overlay_store.latest() if chunk_overlay_store is not None else None
    chunk_current = (
        chunk is not None
        and chunk_overlay_store is not None
        and not chunk_overlay_store.is_stale()
    )
    html = realtime_health_html(
        latest.raw if latest is not None else None,
        chunk.raw if chunk_current else None,
        stale=stale,
    )
    if handle is not None:
        try:
            handle.content = html
        except Exception:
            try:
                handle.value = "Realtime timing telemetry unavailable"
            except Exception:
                pass
    history = handles.get("realtime_history")
    if isinstance(history, RealtimeTimingHistory) and history.add(
        latest.raw if latest is not None and not stale else None,
        chunk.raw if chunk_current else None,
    ):
        plot = handles.get("realtime_plot")
        if plot is not None:
            try:
                plot.figure = history.figure()
            except Exception:
                pass


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


def _lifecycle_init_motion_layout_html() -> str:
    return """
<style>
  .rb-init-motion-layout-applied button { box-sizing: border-box; }
</style>
<script>
(function(){
  function labelOf(button){ return (button.textContent || button.innerText || '').trim(); }
  function parentBox(button){ return button && (button.parentElement || button); }
  function apply(){
    var buttons = Array.prototype.slice.call(document.querySelectorAll('button'));
    function isInit(b, tok){ var l = labelOf(b); return l.indexOf('InitMotion') >= 0 && l.indexOf(tok) >= 0; }
    var both = buttons.find(function(b){ return isInit(b, '(양팔)'); });
    var left = buttons.find(function(b){ return isInit(b, '(왼팔)'); });
    var right = buttons.find(function(b){ return isInit(b, '(오른팔)'); });
    if(!both || !left || !right) return;
    var bothBox = parentBox(both), leftBox = parentBox(left), rightBox = parentBox(right);
    [both, left, right].forEach(function(b){ b.style.width = '100%'; });
    if(bothBox){ bothBox.style.width = '100%'; bothBox.classList.add('rb-init-motion-layout-applied'); }
    if(leftBox){
      leftBox.style.display = 'inline-block';
      leftBox.style.width = 'calc(50% - 0.25rem)';
      leftBox.style.marginRight = '0.5rem';
      leftBox.style.boxSizing = 'border-box';
      leftBox.style.verticalAlign = 'top';
      leftBox.classList.add('rb-init-motion-layout-applied');
    }
    if(rightBox){
      rightBox.style.display = 'inline-block';
      rightBox.style.width = 'calc(50% - 0.25rem)';
      rightBox.style.boxSizing = 'border-box';
      rightBox.style.verticalAlign = 'top';
      rightBox.classList.add('rb-init-motion-layout-applied');
    }
  }
  if(document.readyState === 'loading'){ document.addEventListener('DOMContentLoaded', apply); }
  apply();
  new MutationObserver(apply).observe(document.documentElement, {childList:true, subtree:true});
})();
</script>
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
    chunk_overlay_store: ChunkOverlayStore | None = None,
    recording_status_store: RecordingStatusStore | None = None,
    camera_quality_store: CameraQualityStore | None = None,
    head_preview_store: HeadPreviewStore | None = None,
) -> dict[str, Any]:
    handles: dict[str, Any] = {}
    handles["circle_overlay_enabled"] = overlay_store is not None
    handles["chunk_overlay_enabled"] = chunk_overlay_store is not None
    handles["recording_status_store"] = recording_status_store
    handles["camera_quality_store"] = camera_quality_store
    handles["head_preview_store"] = head_preview_store
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
    handles["main_tabs"] = tabs
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
            handles["realtime_health"] = _add_status_html(
                realtime_health_html(None, None, stale=True)
            )
        handles["realtime_history"] = RealtimeTimingHistory()
        _add_plotly = getattr(server.gui, "add_plotly", None)
        if callable(_add_plotly):
            try:
                with server.gui.add_folder("Realtime timing · 최근 30초", expand_by_default=True):
                    handles["realtime_plot"] = _add_plotly(
                        handles["realtime_history"].figure(),
                        aspect=1.35,
                    )
            except Exception:
                pass
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
        _ov = _load_gui_settings()
        chunk_overlay_visible = _gui_setting_bool(_ov, "chunk_overlay_visible", True)
        chunk_overlay_axes_visible = _gui_setting_bool(_ov, "chunk_overlay_axes_visible", True)
        chunk_overlay_dot_size = _gui_setting_float(_ov, "chunk_overlay_dot_size", 0.022)
        chunk_overlay_persist_sec = _gui_setting_float(_ov, "chunk_overlay_persist_sec", 30.0)
        chunk_overlay_axes_stride = _gui_setting_int(_ov, "chunk_overlay_axes_stride", 2)
        chunk_overlay_history_count = _gui_setting_int(_ov, "chunk_overlay_history_count", 12)
        tcp_gizmo_visible = _gui_setting_bool(_ov, "tcp_gizmo_visible", True)
        tcp_trail_limit = _gui_setting_int(_ov, "tcp_trail_limit", 600)
        if chunk_overlay_axes_stride <= 0:
            chunk_overlay_axes_stride = 2
        if tcp_trail_limit <= 0:
            tcp_trail_limit = 600
        handles["chunk_overlay_visible"] = chunk_overlay_visible
        handles["chunk_overlay_dot_size"] = chunk_overlay_dot_size
        handles["chunk_overlay_persist_sec"] = chunk_overlay_persist_sec
        handles["chunk_overlay_axes_visible"] = chunk_overlay_axes_visible
        handles["chunk_overlay_axes_stride"] = chunk_overlay_axes_stride
        handles["chunk_overlay_history_count"] = chunk_overlay_history_count
        handles["scene"]["tcp_trail_limit"] = tcp_trail_limit
        if hasattr(server.gui, "add_checkbox"):
            handles["chunk_overlay_toggle"] = server.gui.add_checkbox(
                "예측 chunk 궤적 표시", initial_value=chunk_overlay_visible
            )

            @handles["chunk_overlay_toggle"].on_update
            def _(_: Any) -> None:
                handles["chunk_overlay_visible"] = bool(handles["chunk_overlay_toggle"].value)
                _update_gui_setting("chunk_overlay_visible", handles["chunk_overlay_visible"])
                if not handles["chunk_overlay_visible"]:
                    _hide_chunk_overlays(handles)

            handles["chunk_overlay_axes_toggle"] = server.gui.add_checkbox(
                "웨이포인트 자세(6DOF) 표시", initial_value=chunk_overlay_axes_visible
            )

            @handles["chunk_overlay_axes_toggle"].on_update
            def _(_: Any) -> None:
                handles["chunk_overlay_axes_visible"] = bool(
                    handles["chunk_overlay_axes_toggle"].value
                )
                _update_gui_setting(
                    "chunk_overlay_axes_visible", handles["chunk_overlay_axes_visible"]
                )

        if hasattr(server.gui, "add_slider"):
            handles["chunk_overlay_dot_size_slider"] = server.gui.add_slider(
                "웨이포인트 dot 크기",
                min=0.002,
                max=0.035,
                step=0.001,
                initial_value=chunk_overlay_dot_size,
            )

            @handles["chunk_overlay_dot_size_slider"].on_update
            def _(_: Any) -> None:
                handles["chunk_overlay_dot_size"] = float(
                    handles["chunk_overlay_dot_size_slider"].value
                )
                _update_gui_setting("chunk_overlay_dot_size", handles["chunk_overlay_dot_size"])

            handles["chunk_overlay_persist_sec_slider"] = server.gui.add_slider(
                "예측 궤적 잔류(초)",
                min=2,
                max=120,
                step=2,
                initial_value=chunk_overlay_persist_sec,
            )

            @handles["chunk_overlay_persist_sec_slider"].on_update
            def _(_: Any) -> None:
                value = float(handles["chunk_overlay_persist_sec_slider"].value)
                handles["chunk_overlay_persist_sec"] = value
                _update_gui_setting("chunk_overlay_persist_sec", value)

            handles["chunk_overlay_axes_stride_slider"] = server.gui.add_slider(
                "자세 triad 간격(step)",
                min=1,
                max=6,
                step=1,
                initial_value=chunk_overlay_axes_stride,
            )

            @handles["chunk_overlay_axes_stride_slider"].on_update
            def _(_: Any) -> None:
                handles["chunk_overlay_axes_stride"] = int(
                    handles["chunk_overlay_axes_stride_slider"].value
                )
                _update_gui_setting(
                    "chunk_overlay_axes_stride", handles["chunk_overlay_axes_stride"]
                )

            handles["chunk_overlay_history_count_slider"] = server.gui.add_slider(
                "예측 chunk 이력 개수",
                min=0,
                max=40,
                step=1,
                initial_value=chunk_overlay_history_count,
            )

            @handles["chunk_overlay_history_count_slider"].on_update
            def _(_: Any) -> None:
                value = int(handles["chunk_overlay_history_count_slider"].value)
                handles["chunk_overlay_history_count"] = value
                _update_gui_setting("chunk_overlay_history_count", value)

            handles["tcp_trail_limit_slider"] = server.gui.add_slider(
                "궤적 잔류 길이(점)",
                min=100,
                max=3000,
                step=50,
                initial_value=tcp_trail_limit,
            )

            @handles["tcp_trail_limit_slider"].on_update
            def _(_: Any) -> None:
                value = int(handles["tcp_trail_limit_slider"].value)
                handles["scene"]["tcp_trail_limit"] = value
                _update_gui_setting("tcp_trail_limit", value)

        if hasattr(server.gui, "add_text"):
            handles["chunk_overlay_error_text"] = server.gui.add_text(
                "예측-실제 오차 (mm)", initial_value="L — / R —", disabled=True
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

        handles["tcp_gizmo_visible"] = tcp_gizmo_visible
        if hasattr(server.gui, "add_checkbox"):
            handles["tcp_gizmo_toggle"] = server.gui.add_checkbox(
                "TCP 기즈모 표시", initial_value=tcp_gizmo_visible
            )

            @handles["tcp_gizmo_toggle"].on_update
            def _(_: Any) -> None:
                handles["tcp_gizmo_visible"] = bool(
                    handles["tcp_gizmo_toggle"].value
                )
                _update_gui_setting("tcp_gizmo_visible", handles["tcp_gizmo_visible"])
                if not handles["tcp_gizmo_visible"]:
                    _hide_tcp_gizmos(handles)

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

    with tabs.add_tab("카메라 품질"):
        if camera_quality_store is None:
            server.gui.add_text(
                "Camera quality",
                initial_value="disabled (RB_GUI_CAMERA_QUALITY=0)",
                disabled=True,
            )
        else:
            add_html = getattr(server.gui, "add_html", None)
            if callable(add_html):
                handles["camera_quality_status"] = add_html(
                    camera_quality_html(camera_quality_store)
                )
            handles["camera_quality_preview_toggle"] = server.gui.add_checkbox(
                "좌우 wrist preview 표시 (5 Hz)",
                initial_value=False,
            )
            handles["camera_quality_reset_baseline"] = server.gui.add_button(
                "자동 정지 기준선 초기화"
            )
            handles["camera_quality_csv"] = server.gui.add_text(
                "CSV",
                initial_value=str(camera_quality_store.csv_path or "unavailable"),
                disabled=True,
            )
            initial_image = np.zeros((240, 320, 3), dtype=np.uint8)
            for arm in CAMERA_QUALITY_ARMS:
                handles[f"camera_quality_preview_{arm}"] = server.gui.add_image(
                    initial_image,
                    label=f"{arm} wrist",
                    format="jpeg",
                    jpeg_quality=75,
                    visible=False,
                )
            handles["camera_quality_last_preview_monotonic"] = float("-inf")

            @handles["camera_quality_reset_baseline"].on_click
            def _(_: Any) -> None:
                camera_quality_store.request_baseline_reset()

            @handles["camera_quality_preview_toggle"].on_update
            def _(_: Any) -> None:
                visible = bool(handles["camera_quality_preview_toggle"].value)
                for arm in CAMERA_QUALITY_ARMS:
                    preview_handle = handles.get(f"camera_quality_preview_{arm}")
                    if preview_handle is not None:
                        preview_handle.visible = visible

        _build_head_preview_panel(server, handles, head_preview_store)

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

            # ---- F/T sensor + force control ---------------------------------
            # An operator surface for the ONE precondition force control has: a
            # sensor zero. Without a tare the server refuses to make any arm
            # compliant, and before this panel existed the only way to send one was
            # a second terminal — which is how the first hardware attempt ended up
            # taring a server that had already been stopped.
            with server.gui.add_folder("Force / F/T", expand_by_default=True):
                handles["ft_status"] = server.gui.add_text(
                    "F/T", initial_value="F/T: no state", disabled=True
                )
                handles["fc_status"] = server.gui.add_text(
                    "Force control", initial_value="force: no state", disabled=True
                )

                def _tare(left: bool, right: bool) -> None:
                    ok, message = safety.send_ft_tare(left=left, right=right)
                    handles["last_action"].value = ("OK: " if ok else "BLOCKED: ") + message

                tare_both = server.gui.add_button("F/T 영점 (양팔)")
                tare_left_btn = server.gui.add_button("F/T 영점 (왼팔)")
                tare_right_btn = server.gui.add_button("F/T 영점 (오른팔)")

                @tare_both.on_click
                def _(_: Any) -> None:
                    _tare(True, True)

                @tare_left_btn.on_click
                def _(_: Any) -> None:
                    _tare(True, False)

                @tare_right_btn.on_click
                def _(_: Any) -> None:
                    _tare(False, True)

                # The one thing neither the GUI nor the server can check, stated where
                # the button is rather than in a doc nobody has open.
                server.gui.add_text(
                    "영점 주의",
                    initial_value="정지 + 툴 외 무부하 상태에서. 지금 걸린 하중이 새 0이 됩니다",
                    disabled=True,
                )

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

            add_html = getattr(server.gui, "add_html", None)
            if callable(add_html):
                handles["init_motion_layout"] = add_html(_lifecycle_init_motion_layout_html())
            handles["arm_init_status"] = server.gui.add_text(
                "InitMotion override",
                initial_value="왼팔: policy 중 / 오른팔: policy 중",
                disabled=True,
            )
            handles["init_motion_runtime"] = server.gui.add_text(
                "InitMotion runtime",
                initial_value="InitMotion: no state",
                disabled=True,
            )
            init_both_button = server.gui.add_button("InitMotion (양팔)")
            init_left_button = server.gui.add_button("InitMotion (왼팔) [a]")
            init_right_button = server.gui.add_button("InitMotion (오른팔) [c]")
            init_cancel_button = server.gui.add_button("Cancel Init Override / Resume Flow")
            handles["init_motion_button"] = init_both_button
            handles["init_motion_buttons"] = {
                "both": init_both_button,
                "left": init_left_button,
                "right": init_right_button,
                "cancel": init_cancel_button,
            }

            @init_both_button.on_click
            def _(_: Any) -> None:
                ok, message = _send_arm_init_override(safety, handles["scene"], handles, "both")
                handles["last_action"].value = ("OK: " if ok else "BLOCKED: ") + message

            @init_left_button.on_click
            def _(_: Any) -> None:
                ok, message = _send_arm_init_override(safety, handles["scene"], handles, "left")
                handles["last_action"].value = ("OK: " if ok else "BLOCKED: ") + message

            @init_right_button.on_click
            def _(_: Any) -> None:
                ok, message = _send_arm_init_override(safety, handles["scene"], handles, "right")
                handles["last_action"].value = ("OK: " if ok else "BLOCKED: ") + message

            @init_cancel_button.on_click
            def _(_: Any) -> None:
                ok, message = _send_arm_init_cancel_resume(handles, "both")
                handles["last_action"].value = ("OK: " if ok else "BLOCKED: ") + message

            # Edit the init-motion target directly in viser and apply it live (no
            # restart): set_init_joints updates the runtime target used by the next
            # init-motion press, and _save_init_joints persists it to init_motion.json.
            with server.gui.add_folder("Init Motion 편집 (즉시 적용)"):
                init_left_input = server.gui.add_text(
                    "left J1..J6 (deg)", initial_value=_format_joint6(safety.init_left_joint_deg)
                )
                init_right_input = server.gui.add_text(
                    "right J1..J6 (deg)", initial_value=_format_joint6(safety.init_right_joint_deg)
                )
                # Exposed in handles so other tabs (e.g. the WayPoint "set as init"
                # button) can mirror the live init-motion target back into these
                # editor boxes — otherwise they keep showing the build-time value
                # and the update looks like it never happened.
                handles["init_left_input"] = init_left_input
                handles["init_right_input"] = init_right_input
                init_edit_status = server.gui.add_text(
                    "Init Motion edit status", initial_value="edit + Apply, or load current pose", disabled=True
                )
                load_current_button = server.gui.add_button("현재 자세 불러오기")
                apply_init_button = server.gui.add_button("Init Motion 적용 (즉시)")

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

            # Saved-waypoints operations on the selected entry: set as the init-motion
            # pose (persisted to JSON) and delete (kept at the bottom).
            set_init_button = server.gui.add_button("Init Motion 으로 설정하기")
            delete_button = server.gui.add_button("WayPoint 삭제", color="red")
            handles["waypoint_edit_controls"] = [
                waypoint_name_input,
                capture_button,
                waypoint_dropdown,
                set_init_button,
                delete_button,
            ]

            @set_init_button.on_click
            def _(_: Any) -> None:
                ok, message = _set_waypoint_as_init(handles, safety)
                if ok:
                    # Mirror the new target into the init-motion editor boxes so the
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
            stand_floor_config_enabled = server_safety_constraint_config_enabled(
                "floor_constraint"
            )
            user_floor_config_enabled = server_safety_constraint_config_enabled(
                "user_floor_constraint"
            )
            handles["stand_floor_config_enabled"] = stand_floor_config_enabled
            handles["user_floor_config_enabled"] = user_floor_config_enabled
            with server.gui.add_folder("Stand Safety Floor"):
                # Runtime enforce on/off. Startup authority belongs to the server
                # config, so the first state always initializes this checkbox. A stale
                # GUI preference must never disable a configured safety envelope.
                if hasattr(server.gui, "add_checkbox"):
                    floor_enforce = server.gui.add_checkbox(
                        "Enforce stand floor",
                        initial_value=stand_floor_config_enabled is not False,
                    )
                    handles["floor_enforce_toggle"] = floor_enforce
                    _set_disabled(floor_enforce, stand_floor_config_enabled is False)

                    @floor_enforce.on_update
                    def _(_: Any) -> None:
                        enabled = bool(floor_enforce.value)
                        ok, message = safety.send_set_floor_enabled(enabled)
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
                _set_disabled(floor_send, stand_floor_config_enabled is False)
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
                # Show/hide the captured contact-point markers (left=cyan,
                # right=magenta). Default OFF so the points don't clutter the scene;
                # they are still stored/fit, just not drawn until toggled on.
                if hasattr(server.gui, "add_checkbox"):
                    handles["user_floor_show_points_toggle"] = server.gui.add_checkbox(
                        "Show capture points", initial_value=False
                    )
                handles["user_floor_margin_slider"] = server.gui.add_slider(
                    "Lift margin mm", min=0.0, max=50.0, step=0.5,
                    initial_value=float(uf_state.get("margin_mm", 0.0)),
                )
                fit_apply = server.gui.add_button("Fit & Apply plane")
                handles["user_floor_fit_apply_button"] = fit_apply
                _set_disabled(fit_apply, user_floor_config_enabled is False)
                if hasattr(server.gui, "add_checkbox"):
                    handles["user_floor_enforce_toggle"] = server.gui.add_checkbox(
                        "Enforce user floor",
                        initial_value=(
                            bool(uf_state.get("enabled", False))
                            and user_floor_config_enabled is not False
                        ),
                    )
                    _set_disabled(
                        handles["user_floor_enforce_toggle"],
                        user_floor_config_enabled is False,
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
                    update_user_floor_capture_points(
                        handles.get("scene", {}), _user_floor_display_points(handles))

                handles["user_floor_refresh_fn"] = _refresh_user_floor_texts

                if "user_floor_show_points_toggle" in handles:
                    @handles["user_floor_show_points_toggle"].on_update
                    def _(_: Any) -> None:
                        # Re-render immediately so toggling shows/hides the markers.
                        _refresh_user_floor_texts()

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
                # 저장된 ROI(settings.json roi_min_m/roi_max_m)가 있으면 그 값으로 시작 →
                # 재시작 후에도 perception/viz 클립 영역 유지(검출 워커와 같은 박스).
                # 이 범위는 첫 서버 상태 전까지만 쓰는 bootstrap 값이다. 이후
                # _update_roi_panel이 서버가 publish한 runtime envelope/applied bounds로
                # 범위와 값을 동기화한다.
                _roi_slider_specs = _roi_initial_slider_specs(_load_gui_settings())
                for _axis in ("x", "y", "z"):
                    _slider_min, _slider_max, _lo_default, _hi_default = _roi_slider_specs[_axis]
                    handles[f"roi_{_axis}_min"] = server.gui.add_slider(
                        f"{_axis.upper()} min mm", min=_slider_min, max=_slider_max, step=5.0,
                        initial_value=_lo_default,
                    )
                    handles[f"roi_{_axis}_max"] = server.gui.add_slider(
                        f"{_axis.upper()} max mm", min=_slider_min, max=_slider_max, step=5.0,
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
                    _persist_roi_settings(handles)   # perception/viz 클립 영역도 함께 적용

                def _roi_preview(_: Any) -> None:
                    # Live yellow preview box while dragging any bound slider.
                    lo, hi = _roi_slider_bounds()
                    update_roi_box_preview(handles.get("scene", {}), lo, hi)
                    _persist_roi_settings(handles)   # 드래그 즉시 검출/클라우드 클립에 반영(≤1s)

                for _axis in ("x", "y", "z"):
                    handles[f"roi_{_axis}_min"].on_update(_roi_preview)
                    handles[f"roi_{_axis}_max"].on_update(_roi_preview)

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

        with _op_tabs.add_tab("SpaceMouse"):
            handles["spacemouse_summary"] = server.gui.add_text(
                "감지 장치", initial_value="policy_runner 상태 대기 중", disabled=True
            )
            handles["spacemouse_left_select"] = server.gui.add_dropdown(
                "왼팔 SpaceMouse", ("미배정",), initial_value="미배정"
            )
            handles["spacemouse_right_select"] = server.gui.add_dropdown(
                "오른팔 SpaceMouse", ("미배정",), initial_value="미배정"
            )
            handles["spacemouse_assignment_dirty"] = False
            handles["spacemouse_syncing"] = False
            handles["spacemouse_pending_seq"] = 0

            def _mark_spacemouse_dirty(_: Any = None) -> None:
                if not handles.get("spacemouse_syncing", False):
                    handles["spacemouse_assignment_dirty"] = True

            handles["spacemouse_left_select"].on_update(_mark_spacemouse_dirty)
            handles["spacemouse_right_select"].on_update(_mark_spacemouse_dirty)
            apply_button = server.gui.add_button("좌/우 배정 적용", color="green")
            swap_button = server.gui.add_button("좌우 교환")
            handles["spacemouse_apply_button"] = apply_button
            handles["spacemouse_swap_button"] = swap_button
            handles["spacemouse_command_status"] = server.gui.add_text(
                "배정 상태", initial_value="미배정 장치는 로봇을 움직이지 않습니다", disabled=True
            )

            @apply_button.on_click
            def _(_: Any) -> None:
                _send_spacemouse_assignment(handles)

            @swap_button.on_click
            def _(_: Any) -> None:
                _send_spacemouse_swap(handles)

    with tabs.add_tab("이동"):
        _move_tabs = server.gui.add_tab_group()
        with _move_tabs.add_tab("관절"):
            # Per-joint [−] Jn [+] nudge rows (same direct pattern as TCP PTP):
            # one click jogs that joint by ±Step. Step slider is magnitude only;
            # the −/+ button decides the sign.
            jog_arm = server.gui.add_button_group("Arm", ("left", "right"))
            jog_step = server.gui.add_slider("Step deg", min=0.1, max=5.0, step=0.1, initial_value=0.5)
            handles["joint_jog_controls"] = [jog_arm, jog_step]
            handles["jog_status"] = server.gui.add_text("Jog status", initial_value="idle", disabled=True)

            def _jog_joint_nudge(joint_index: int, sign: float) -> None:
                ok, message = safety.jog_joint(jog_arm.value, joint_index, sign * float(jog_step.value))
                handles["jog_status"].value = ("OK: " if ok else "BLOCKED: ") + message

            def _add_joint_jog_row(joint_index: int) -> None:
                group = server.gui.add_button_group("", ("-", _nudge_label(f"J{joint_index + 1}"), "+"))
                handles["joint_jog_controls"].append(group)

                @group.on_click
                def _(_: Any, group: Any = group, joint_index: int = joint_index) -> None:
                    if group.value == "-":
                        _jog_joint_nudge(joint_index, -1.0)
                    elif group.value == "+":
                        _jog_joint_nudge(joint_index, 1.0)

            for _joint_index in range(6):
                _add_joint_jog_row(_joint_index)

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
                    ok, message = _apply_tcp_pose_step_and_send_pose_target(
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
                        if not _apply_tcp_pose_step_to_target(handles["scene"], arm, tuple(delta), frame_mode):  # type: ignore[arg-type]
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
        with _adv_tabs.add_tab("Pointcloud"):
            _pc = _load_gui_settings()
            handles["pc_enable"] = server.gui.add_checkbox(
                "스테레오 pointcloud 표시", initial_value=bool(_pc.get("pc_enable", False)))
            handles["pc_box_enable"] = server.gui.add_checkbox(
                "매칭 박스 표시",
                initial_value=bool(_pc.get("pc_box_enable", _pc.get("pc_enable", False))),
            )
            handles["pc_size"] = server.gui.add_slider(
                "point size", min=0.001, max=0.02, step=0.001,
                initial_value=float(_pc.get("pc_size", 0.004)))
            handles["pc_max_k"] = server.gui.add_slider(
                "최대 표시 점수(k)", min=10, max=300, step=10,
                initial_value=int(_pc.get("pc_max_k", 80)))
            handles["pc_dmin"] = server.gui.add_slider(
                "depth min (m)", min=0.1, max=4.0, step=0.05,
                initial_value=float(_pc.get("pc_dmin", 0.2)))
            handles["pc_dmax"] = server.gui.add_slider(
                "depth max (m)", min=0.1, max=4.0, step=0.05,
                initial_value=float(_pc.get("pc_dmax", 3.0)))
            # stand-Z 점 상한 (mm, 0=off). Safety ROI(로봇 safety용 Z)와 독립한
            # perception 전용 높이 컷 → ROI Z는 로봇 모션용으로 높게 두고 클라우드/검출만
            # 낮게 자른다(로봇팔·배경 noise 제거). settings.json pc_clip_z_max_m(=값/1000)로
            # 영속화, camera_server stereo_worker가 1Hz로 읽어 viz·검출 양쪽에 적용.
            handles["pc_clip_z_max_mm"] = server.gui.add_slider(
                "stand Z 상한 (mm, 0=off)", min=0, max=500, step=10,
                initial_value=float(_pc.get("pc_clip_z_max_m", 0.150)) * 1000.0)
            handles["pc_status"] = server.gui.add_text(
                "pointcloud status", initial_value="off", disabled=True)
            # 설정 영속화: 변경 시 settings.json에 저장 (재시작 후에도 유지)
            for _k in ("pc_enable", "pc_box_enable", "pc_size", "pc_max_k", "pc_dmin",
                       "pc_dmax", "pc_clip_z_max_mm"):
                handles[_k].on_update(lambda _evt: _persist_pc_settings(handles))

            box_detect_host, box_detect_port = _box_detect_cmd_endpoint()
            handles["box_detect_cmd_client"] = BoxDetectCommandClient(box_detect_host, box_detect_port)
            handles["box_detect_busy"] = False
            handles["box_detect_busy_since"] = float("-inf")
            handles["box_detect_seen_burst"] = False
            handles["box_detect_last_trigger_monotonic"] = float("-inf")
            handles["box_detect_button"] = server.gui.add_button("🎯 박스 재탐지")
            handles["box_detect_status_green"] = server.gui.add_text(
                "green box", initial_value="탐지 전", disabled=True)
            handles["box_detect_status_gray"] = server.gui.add_text(
                "gray box", initial_value="탐지 전", disabled=True)

            @handles["box_detect_button"].on_click
            def _(_evt) -> None:
                _trigger_box_detect(handles)

            with server.gui.add_folder("수동 캘리브레이션 (T_stand_cam)", expand_by_default=False):
                handles["pc_calib_mode"] = server.gui.add_checkbox("캘리브레이션 모드(기즈모)", initial_value=False)
                handles["pc_calib_save"] = server.gui.add_button("💾 캘리브레이션 저장")
                handles["pc_calib_reset"] = server.gui.add_button("↩ 저장값으로 리셋")
                handles["pc_calib_status"] = server.gui.add_text(
                    "calib pose", initial_value="(기즈모로 URDF 로봇팔과 정렬 후 저장)", disabled=True)

            # 클라우드 부모 프레임(=T_stand_cam) + 캘리브용 기즈모 (씬 노드)
            T0 = load_T_stand_cam()
            handles["_T_stand_cam_init"] = T0
            handles["pc_cam_frame"] = server.scene.add_frame(
                "/stereo_cam", show_axes=False,
                wxyz=tuple(mat_to_wxyz(T0[:3, :3])), position=tuple(T0[:3, 3]))
            handles["pc_cam_gizmo"] = server.scene.add_transform_controls(
                "/stereo_cam_gizmo", scale=0.3, line_width=2.5, depth_test=False, visible=False,
                wxyz=tuple(mat_to_wxyz(T0[:3, :3])), position=tuple(T0[:3, 3]))

            @handles["pc_calib_save"].on_click
            def _(_evt) -> None:
                import numpy as np
                g = handles["pc_cam_gizmo"]
                T = np.eye(4); T[:3, :3] = wxyz_to_mat(g.wxyz); T[:3, 3] = np.array(g.position)
                ok, msg = save_T_stand_cam(T)
                handles["pc_calib_status"].value = (f"✅ 저장: {msg}" if ok else f"❌ {msg}")

            @handles["pc_calib_reset"].on_click
            def _(_evt) -> None:
                T = load_T_stand_cam()
                for h in (handles["pc_cam_gizmo"], handles["pc_cam_frame"]):
                    h.wxyz = tuple(mat_to_wxyz(T[:3, :3])); h.position = tuple(T[:3, 3])
                handles["pc_calib_status"].value = "저장값으로 리셋됨"

            # --- 손목(D405) raw 클라우드 오버레이 (모델 추론 X) ---
            # 워커가 stereo.wrist로 카메라 프레임 클라우드를 저속 발행. rb_gui는 이를
            # /stand/{arm}_tcp(실시간 TCP) 자식의 핸드아이(T_tcp_cam) 프레임 아래 렌더한다.
            # 핸드아이는 추정 초기값 + 기즈모 수동 캘리브(저장/로드).
            handles["pc_wrist_enable"] = server.gui.add_checkbox(
                "손목 카메라 raw 클라우드 표시", initial_value=bool(_pc.get("pc_wrist_enable", False)))
            handles["pc_wrist_status"] = server.gui.add_text(
                "wrist cloud status", initial_value="off", disabled=True)
            handles["pc_wrist_enable"].on_update(lambda _evt: _persist_pc_settings(handles))

            # 좌/우 손목 D405는 동일 하드웨어·동일 마운트 → 단일 T_tcp_cam 공유.
            # 기즈모는 양손에 모두 두되(각 팔 클라우드와 정렬 확인용), 한쪽을 조작하면
            # 같은 로컬 pose(=T_tcp_cam)가 양쪽 기즈모+프레임에 동기 전파된다.
            handles["_wrist_gizmo"] = {}
            handles["_wrist_frame"] = {}
            handles["_wrist_sync_last"] = None
            with server.gui.add_folder("손목 핸드아이 (통합 T_tcp_cam · 양손 동기화)", expand_by_default=False):
                handles["pc_wrist_calib"] = server.gui.add_checkbox(
                    "캘리브레이션 모드(양손 기즈모)", initial_value=False)
                handles["pc_wrist_save"] = server.gui.add_button("💾 핸드아이 저장(공유)")
                handles["pc_wrist_reset"] = server.gui.add_button("↩ 저장값으로 리셋")
                handles["pc_wrist_status_calib"] = server.gui.add_text(
                    "calib pose", initial_value="(한쪽 기즈모로 정렬하면 양손이 같이 움직임)", disabled=True)

            Tw = load_T_tcp_cam("")          # 공유 핸드아이(arm 무시)
            for _arm in ("left", "right"):
                # 핸드아이 프레임 + 기즈모: /stand/{arm}_tcp 자식이므로 로컬 pose = T_tcp_cam.
                handles["_wrist_frame"][_arm] = server.scene.add_frame(
                    f"/stand/{_arm}_tcp/wrist_cam", show_axes=False,
                    wxyz=tuple(mat_to_wxyz(Tw[:3, :3])), position=tuple(Tw[:3, 3]))
                handles["_wrist_gizmo"][_arm] = server.scene.add_transform_controls(
                    f"/stand/{_arm}_tcp/wrist_cam_gizmo", scale=0.2, line_width=2.0,
                    depth_test=False, visible=False,
                    wxyz=tuple(mat_to_wxyz(Tw[:3, :3])), position=tuple(Tw[:3, 3]))

            def _wrist_save(_evt) -> None:
                import numpy as np
                g = handles["_wrist_gizmo"]["left"]    # 양손 동기화 → 어느 쪽이든 동일 pose
                T = np.eye(4); T[:3, :3] = wxyz_to_mat(g.wxyz); T[:3, 3] = np.array(g.position)
                ok, msg = save_T_tcp_cam("", T)
                handles["pc_wrist_status_calib"].value = (f"✅ 저장: {msg}" if ok else f"❌ {msg}")

            def _wrist_reset(_evt) -> None:
                T = load_T_tcp_cam("")
                w = tuple(mat_to_wxyz(T[:3, :3])); p = tuple(T[:3, 3])
                for _a in ("left", "right"):
                    for h in (handles["_wrist_gizmo"][_a], handles["_wrist_frame"][_a]):
                        h.wxyz = w; h.position = p
                handles["_wrist_sync_last"] = {"left": (w, p), "right": (w, p)}
                handles["pc_wrist_status_calib"].value = "저장값으로 리셋됨"

            handles["pc_wrist_save"].on_click(_wrist_save)
            handles["pc_wrist_reset"].on_click(_wrist_reset)

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

    with tabs.add_tab("에피소드"):
        record_host, record_port = _recording_cmd_endpoint()
        handles["recording_cmd_client"] = RecordingCommandClient(record_host, record_port)
        handles["recording_last_toggle_monotonic"] = float("-inf")
        handles["recording_task"] = server.gui.add_text("Task", initial_value="", disabled=False)
        handles["recording_operator"] = server.gui.add_text(
            "Operator", initial_value=os.environ.get("USER", ""), disabled=False
        )
        handles["recording_state"] = server.gui.add_text("State", initial_value="idle", disabled=True)
        handles["recording_episode"] = server.gui.add_text("Episode", initial_value="", disabled=True)
        handles["recording_frames"] = server.gui.add_text("Frames", initial_value="0 @ 30.0 Hz", disabled=True)
        handles["recording_command_status"] = server.gui.add_text(
            "Command",
            initial_value=f"policy_runner {record_host}:{record_port}",
            disabled=True,
        )
        # Master On/Off gate for the whole episode-recording feature. Always OFF
        # when `make run` brings the GUI up (initial_value=False, not persisted):
        # only when this is ON does a 수집 시작 / 'b' hotkey / foot-pedal press
        # actually start (and save) an episode. Stopping an in-progress episode is
        # never gated. See _toggle_episode_recording / _update_recording_panel.
        if hasattr(server.gui, "add_checkbox"):
            handles["recording_enabled"] = server.gui.add_checkbox(
                "녹화 활성화 (Enable)", initial_value=False
            )

            @handles["recording_enabled"].on_update
            def _(_: Any) -> None:
                _update_recording_panel(
                    handles,
                    handles.get("_latest_state"),
                    stale=bool(handles.get("_state_stale", True)),
                )

        start_button = server.gui.add_button("수집 시작", color="green")
        stop_button = server.gui.add_button("수집 종료", color="red")
        toggle_button = server.gui.add_button("record-toggle-hotkey-b")
        try:
            toggle_button.visible = False
        except Exception:
            pass
        handles["recording_start_button"] = start_button
        handles["recording_stop_button"] = stop_button
        handles["recording_toggle_hotkey_button"] = toggle_button

        @start_button.on_click
        def _(_: Any) -> None:
            _toggle_episode_recording(handles, target="start")

        @stop_button.on_click
        def _(_: Any) -> None:
            _toggle_episode_recording(handles, target="stop")

        @toggle_button.on_click
        def _(_: Any) -> None:
            _toggle_episode_recording(handles)

    handles["_server"] = server
    return handles


def _persist_pc_settings(handles: dict[str, Any]) -> None:
    """Pointcloud 표시 설정을 settings.json에 병합 저장 (재시작 후 유지)."""
    try:
        s = _load_gui_settings()
        s["pc_enable"] = bool(handles["pc_enable"].value)
        if handles.get("pc_box_enable") is not None:
            s["pc_box_enable"] = bool(handles["pc_box_enable"].value)
        s["pc_size"] = float(handles["pc_size"].value)
        s["pc_max_k"] = int(handles["pc_max_k"].value)
        s["pc_dmin"] = float(handles["pc_dmin"].value)
        s["pc_dmax"] = float(handles["pc_dmax"].value)
        if handles.get("pc_clip_z_max_mm") is not None:
            _zc_mm = float(handles["pc_clip_z_max_mm"].value)
            if _zc_mm > 0:                       # 0 = off → 키 제거(상한 없음)
                s["pc_clip_z_max_m"] = _zc_mm / 1000.0
            else:
                s.pop("pc_clip_z_max_m", None)
        if handles.get("pc_wrist_enable") is not None:
            s["pc_wrist_enable"] = bool(handles["pc_wrist_enable"].value)
        _save_gui_settings(s)
    except Exception:
        pass


_BOX_DETECT_BUSY_TIMEOUT_S = 8.0  # client-side fallback only, independent of the worker's actual
                                   # (server-side, env-configurable STEREO_BOX_BURST_S, default 4.0s)
                                   # burst duration — just generous enough that a real burst always
                                   # finishes first; only fires on a truly unresponsive worker.


def _trigger_box_detect(handles: dict[str, Any], *, monotonic_fn=time.monotonic) -> bool:
    if handles.get("box_detect_busy"):
        return False
    now = monotonic_fn()
    last = handles.get("box_detect_last_trigger_monotonic", float("-inf"))
    try:
        last_value = float(last)
    except (TypeError, ValueError):
        last_value = float("-inf")
    if now - last_value < 1.0:
        return False
    client = handles.get("box_detect_cmd_client")
    send = getattr(client, "send_detect_now", None)
    if not callable(send):
        return False
    try:
        result = send()
    except OSError as exc:
        result = BoxDetectCommandResult(False, f"box detect_now send failed: {exc}")
    if not result.ok:
        for key in ("box_detect_status_green", "box_detect_status_gray"):
            if key in handles:
                handles[key].value = f"BLOCKED: {result.message}"
        return False
    handles["box_detect_last_trigger_monotonic"] = now
    handles["box_detect_busy"] = True
    handles["box_detect_busy_since"] = now
    handles["box_detect_seen_burst"] = False
    if "box_detect_button" in handles:
        _set_disabled(handles["box_detect_button"], True)
    return True


def _update_box_detect_status(handles: dict[str, Any], *, monotonic_fn=time.monotonic) -> None:
    """Called every update_gui() tick. While the OLD locked box is still what's rendered (the
    worker always heartbeats the previous lock during a burst, never the in-progress candidate —
    see worker.py's build_heartbeat_boxes), this refreshes only the green/gray status text.

    `phase`/`locks` come from the worker in real time even mid-burst (it republishes every frame
    while phase=="burst" too, not just once at the end), so once the worker's own burst-start
    sets last_result="pending" that reaches the GUI almost immediately — we don't need to fake a
    local "탐지 중…" text. The local busy/seen_burst/timeout bookkeeping below exists ONLY to
    (a) keep the button disabled for the duration of a burst, and (b) detect a truly unresponsive
    worker (UDP send always reports ok=True even if nothing is listening) so the button doesn't
    stay stuck disabled forever.
    """
    store = handles.get("_stereo_store")
    latest_locks = store.latest_locks() if store is not None else None
    locks = (latest_locks or {}).get("locks") or {}
    phase = (latest_locks or {}).get("phase")

    timed_out = False
    if handles.get("box_detect_busy"):
        if phase == "burst":
            handles["box_detect_seen_burst"] = True
        now = monotonic_fn()
        since = float(handles.get("box_detect_busy_since", now))
        finished = phase == "idle" and handles.get("box_detect_seen_burst")
        timed_out = (not finished) and (now - since > _BOX_DETECT_BUSY_TIMEOUT_S)
        if finished or timed_out:
            handles["box_detect_busy"] = False
            if "box_detect_button" in handles:
                _set_disabled(handles["box_detect_button"], False)

    if timed_out:
        for key in ("box_detect_status_green", "box_detect_status_gray"):
            if key in handles:
                handles[key].value = "응답 없음"
        return

    if "box_detect_status_green" in handles:
        handles["box_detect_status_green"].value = box_lock_status_text(locks.get("green"))
    if "box_detect_status_gray" in handles:
        handles["box_detect_status_gray"].value = box_lock_status_text(locks.get("gray"))


def _persist_roi_settings(handles: dict[str, Any]) -> None:
    """현재 ROI 슬라이더 바운드를 settings.json(roi_min_m/roi_max_m, stand 프레임 m)에
    저장. camera_server stereo_worker(box_detect)가 이 값을 읽어 박스 검출 영역 + publish
    클라우드(head/손목) 클립 영역으로 쓴다 → viser 시각화와 검출이 같은 Safety ROI로 정렬."""
    try:
        fn = handles.get("roi_slider_bounds_fn")
        if not callable(fn):
            return
        lo, hi = fn()
        s = _load_gui_settings()
        s["roi_min_m"] = [float(v) for v in lo]
        s["roi_max_m"] = [float(v) for v in hi]
        _save_gui_settings(s)
    except Exception:
        pass


def _latest_circle_overlay(
    overlay_store: CircleOverlayStore | None,
) -> tuple[CircleOverlaySnapshot | None, bool]:
    if overlay_store is None:
        return None, True
    return overlay_store.latest(), overlay_store.is_stale()


def _latest_chunk_overlay(
    chunk_overlay_store: ChunkOverlayStore | None,
) -> tuple[ChunkOverlaySnapshot | None, bool]:
    if chunk_overlay_store is None:
        return None, True
    return chunk_overlay_store.latest(), chunk_overlay_store.is_stale()


def _recording_cmd_endpoint() -> tuple[str, int]:
    """policy_runner record_cmd.v1 destination."""
    return parse_udp_endpoint(
        os.environ.get("RB_GUI_RECORD_CMD_ENDPOINT", "udp://127.0.0.1:50441"),
        default_host="127.0.0.1",
        default_port=50441,
    )


def _box_detect_cmd_endpoint() -> tuple[str, int]:
    """stereo_worker box_detect_cmd.v1 destination (must match its STEREO_TRIGGER_ENDPOINT)."""
    return parse_udp_endpoint(
        os.environ.get("RB_GUI_BOX_DETECT_CMD_ENDPOINT", "udp://127.0.0.1:50387"),
        default_host="127.0.0.1",
        default_port=50387,
    )


def _recording_status_bind_endpoint() -> tuple[str, int]:
    """policy_runner recording_state.v1 status bind."""
    return parse_udp_endpoint(
        os.environ.get("RB_GUI_RECORD_STATUS_BIND", "udp://0.0.0.0:50442"),
        default_host="0.0.0.0",
        default_port=50442,
    )


def _recording_text_value(handle: Any) -> str:
    try:
        return str(handle.value or "")
    except Exception:
        return ""


def _recording_status_block(
    handles: dict[str, Any],
    latest: StateSnapshot | None = None,
) -> dict[str, Any]:
    if latest is not None and isinstance(latest.recording, Mapping):
        return normalize_recording_status(latest.recording)
    store = handles.get("recording_status_store")
    if isinstance(store, RecordingStatusStore):
        block = store.latest()
        if block is not None:
            return normalize_recording_status(block)
    local = handles.get("recording_local_status")
    if isinstance(local, Mapping):
        return normalize_recording_status(local)
    return normalize_recording_status(None)


def _spacemouse_status_block(handles: dict[str, Any]) -> dict[str, Any] | None:
    store = handles.get("recording_status_store")
    if isinstance(store, RecordingStatusStore):
        return store.latest_spacemouse()
    return None


def _spacemouse_selected_connection(handle: Any) -> str | None:
    value = str(getattr(handle, "value", "") or "").strip()
    return None if value in {"", "미배정"} else value


def _send_spacemouse_assignment(handles: dict[str, Any]) -> bool:
    status = _spacemouse_status_block(handles)
    client = handles.get("recording_cmd_client")
    send = getattr(client, "send_spacemouse_assignments", None)
    if status is None or not callable(send):
        handles["spacemouse_command_status"].value = "BLOCKED: policy_runner 상태/제어 없음"
        return False
    result = send(
        status_generation=int(status.get("generation", -1)),
        left_connection_id=_spacemouse_selected_connection(handles.get("spacemouse_left_select")),
        right_connection_id=_spacemouse_selected_connection(handles.get("spacemouse_right_select")),
    )
    return _apply_spacemouse_command_result(handles, result)


def _send_spacemouse_swap(handles: dict[str, Any]) -> bool:
    status = _spacemouse_status_block(handles)
    client = handles.get("recording_cmd_client")
    send = getattr(client, "send_spacemouse_swap", None)
    if status is None or not callable(send):
        handles["spacemouse_command_status"].value = "BLOCKED: policy_runner 상태/제어 없음"
        return False
    return _apply_spacemouse_command_result(
        handles,
        send(status_generation=int(status.get("generation", -1))),
    )


def _apply_spacemouse_command_result(handles: dict[str, Any], result: SpaceMouseCommandResult) -> bool:
    prefix = "PENDING: " if result.ok else "BLOCKED: "
    handles["spacemouse_command_status"].value = prefix + result.message
    if result.ok and isinstance(result.payload, Mapping):
        handles["spacemouse_pending_seq"] = int(result.payload.get("seq", 0) or 0)
    return result.ok


def _update_spacemouse_panel(handles: dict[str, Any]) -> None:
    if "spacemouse_summary" not in handles:
        return
    status = _spacemouse_status_block(handles)
    store = handles.get("recording_status_store")
    stale = not isinstance(store, RecordingStatusStore) or store.is_stale(threshold_sec=2.0)
    if status is None:
        handles["spacemouse_summary"].value = "policy_runner 상태 없음"
        _set_disabled(handles.get("spacemouse_apply_button"), True)
        _set_disabled(handles.get("spacemouse_swap_button"), True)
        return
    devices = status.get("devices") if isinstance(status.get("devices"), list) else []
    input_policy = status.get("input_policy") if isinstance(status.get("input_policy"), Mapping) else {}
    side_gates = status.get("side_gates") if isinstance(status.get("side_gates"), Mapping) else {}
    connection_ids = [
        str(device.get("connection_id"))
        for device in devices
        if isinstance(device, Mapping) and device.get("connection_id")
    ]
    options = ("미배정", *connection_ids)
    for key in ("spacemouse_left_select", "spacemouse_right_select"):
        handle = handles.get(key)
        if handle is not None:
            try:
                handle.options = options
            except Exception:
                pass
    if not handles.get("spacemouse_assignment_dirty", False):
        left = str(status.get("left_connection_id") or "미배정")
        right = str(status.get("right_connection_id") or "미배정")
        handles["spacemouse_syncing"] = True
        try:
            if handles.get("spacemouse_left_select") is not None:
                handles["spacemouse_left_select"].value = left if left in options else "미배정"
            if handles.get("spacemouse_right_select") is not None:
                handles["spacemouse_right_select"].value = right if right in options else "미배정"
        finally:
            handles["spacemouse_syncing"] = False
    rows = [
        "policy "
        f"deadman={'on' if bool(input_policy.get('require_deadman', False)) else 'off'} "
        f"startup-neutral={'on' if bool(input_policy.get('startup_requires_neutral', False)) else 'off'} "
        f"deadband={float(input_policy.get('deadband', 0.0) or 0.0):.3f} "
        f"activation={float(input_policy.get('activation_deadband', 0.0) or 0.0):.3f}"
    ]
    for index, device in enumerate(devices):
        if not isinstance(device, Mapping):
            continue
        arm = str(device.get("arm", "unassigned") or "unassigned")
        gate = side_gates.get(arm) if arm in {"left", "right"} else None
        gate = gate if isinstance(gate, Mapping) else {}
        raw_axes = device.get("raw_axes")
        raw_text = "no-sample"
        if isinstance(raw_axes, (list, tuple)) and len(raw_axes) == 6:
            raw_text = "(" + ",".join(f"{float(value):+.2f}" for value in raw_axes) + ")"
        sample_age = device.get("sample_age_sec")
        age_text = "n/a" if sample_age is None else f"{float(sample_age):.3f}s"
        gate_text = str(gate.get("gate", "unassigned"))
        if gate_text == "startup_neutral":
            elapsed = float(gate.get("neutral_hold_elapsed_sec", 0.0) or 0.0)
            required = float(input_policy.get("startup_neutral_hold_sec", 0.0) or 0.0)
            gate_text = f"startup-neutral {elapsed:.2f}/{required:.2f}s"
        rows.append(
            f"{chr(ord('A') + index)}={device.get('connection_id')} "
            f"arm={arm} activity={float(device.get('activity', 0.0) or 0.0):.2f} "
            f"neutral={bool(device.get('neutral', True))} age={age_text} gate={gate_text}\n"
            f"  raw={raw_text}"
        )
    handles["spacemouse_summary"].value = "\n".join(rows) if rows else "감지된 SpaceMouse 없음"
    allowed = bool(status.get("assignment_change_allowed", False)) and not stale
    _set_disabled(handles.get("spacemouse_apply_button"), not allowed)
    both_assigned = bool(status.get("left_connection_id")) and bool(status.get("right_connection_id"))
    _set_disabled(handles.get("spacemouse_swap_button"), not allowed or not both_assigned)
    pending = int(handles.get("spacemouse_pending_seq", 0) or 0)
    acknowledged = int(status.get("last_command_seq", 0) or 0)
    if pending and acknowledged >= pending:
        result = str(status.get("last_result", "") or "")
        error = str(status.get("last_error", "") or "")
        handles["spacemouse_command_status"].value = (
            "OK: 배정 적용됨" if result == "accepted" else f"BLOCKED: {error or result or 'unknown'}"
        )
        handles["spacemouse_pending_seq"] = 0
        if result == "accepted":
            handles["spacemouse_assignment_dirty"] = False


def _recording_is_active(handles: dict[str, Any], latest: StateSnapshot | None = None) -> bool:
    return bool(_recording_status_block(handles, latest).get("recording", False))


def _apply_recording_result(
    handles: dict[str, Any],
    result: RecordingCommandResult | Any,
    *,
    command: str,
) -> bool:
    if isinstance(result, RecordingCommandResult):
        ok = result.ok
        message = result.message
    else:
        ok = bool(result)
        message = f"recording {command} sent" if ok else f"recording {command} failed"
    if "recording_command_status" in handles:
        handles["recording_command_status"].value = ("OK: " if ok else "BLOCKED: ") + message
    return ok


def _recording_enabled(handles: dict[str, Any]) -> bool:
    """Master On/Off gate for episode recording.

    Backed by the "녹화 활성화" checkbox in the 에피소드 tab, which is always OFF
    when the GUI starts (default at every `make run`). When the checkbox handle is
    absent (headless/older test fixtures that don't wire it) recording is treated
    as enabled so legacy behavior and existing tests are preserved.
    """
    toggle = handles.get("recording_enabled")
    if toggle is None:
        return True
    try:
        return bool(toggle.value)
    except Exception:
        return True


def _toggle_episode_recording(
    handles: dict[str, Any],
    *,
    target: str | None = None,
    monotonic_fn=time.monotonic,
) -> bool:
    now = monotonic_fn()
    last = handles.get("recording_last_toggle_monotonic", float("-inf"))
    try:
        last_value = float(last)
    except (TypeError, ValueError):
        last_value = float("-inf")
    if now - last_value < 0.5:
        if "recording_command_status" in handles:
            handles["recording_command_status"].value = "ignored: debounce active (<0.5s)"
        return False
    active = _recording_is_active(handles, handles.get("_latest_state"))
    command = target if target in {"start", "stop"} else ("stop" if active else "start")
    if command == "start" and active:
        if "recording_command_status" in handles:
            handles["recording_command_status"].value = "already recording"
        return False
    if command == "stop" and not active:
        if "recording_command_status" in handles:
            handles["recording_command_status"].value = "already idle"
        return False
    # Master gate: a start (수집 시작 / 'b' / foot-pedal from idle) only fires when
    # the 녹화 활성화 toggle is ON. Stopping an in-progress episode is never gated,
    # so 'b' can always end a recording even after the toggle is switched off.
    if command == "start" and not _recording_enabled(handles):
        if "recording_command_status" in handles:
            handles["recording_command_status"].value = "BLOCKED: 녹화 비활성화 (녹화 활성화 토글 OFF)"
        return False
    client = handles.get("recording_cmd_client")
    send = getattr(client, "send", None)
    if not callable(send):
        if "recording_command_status" in handles:
            handles["recording_command_status"].value = "BLOCKED: no recording command client"
        return False
    try:
        result = send(
            command,
            task=_recording_text_value(handles.get("recording_task")),
            operator=_recording_text_value(handles.get("recording_operator")) or None,
        )
    except OSError as exc:
        result = RecordingCommandResult(False, f"recording {command} send failed: {exc}")
    ok = _apply_recording_result(handles, result, command=command)
    if not ok:
        return False
    handles["recording_last_toggle_monotonic"] = now
    handles["recording_local_status"] = normalize_recording_status(
        {
            "recording": command == "start",
            "state": "recording" if command == "start" else "idle",
            "episode_name": "",
            "frame_count": 0,
            "rate_hz": 30.0,
            "last_command": command,
        }
    )
    _update_recording_panel(handles, handles.get("_latest_state"), stale=bool(handles.get("_state_stale", True)))
    return True


def _update_recording_panel(handles: dict[str, Any], latest: StateSnapshot | None, *, stale: bool) -> None:
    status = _recording_status_block(handles, latest)
    recording = bool(status.get("recording", False))
    state = str(status.get("state", "recording" if recording else "idle"))
    episode = str(status.get("episode_name", "") or "")
    frames = int(status.get("frame_count", 0) or 0)
    rate = float(status.get("rate_hz", 30.0) or 30.0)
    error = str(status.get("error", "") or "")
    if "recording_state" in handles:
        suffix = " (stale)" if stale else ""
        handles["recording_state"].value = state + suffix + (f" error={error}" if error else "")
    if "recording_episode" in handles:
        handles["recording_episode"].value = episode
    if "recording_frames" in handles:
        handles["recording_frames"].value = f"{frames} @ {rate:.1f} Hz"
    enabled = _recording_enabled(handles)
    if "recording_start_button" in handles:
        # Start is available only while the master toggle is ON and nothing is
        # already recording. Stop stays live whenever an episode is in progress
        # (even after the toggle is switched off) so it can always be ended.
        _set_disabled(handles["recording_start_button"], recording or not enabled)
    if "recording_stop_button" in handles:
        _set_disabled(handles["recording_stop_button"], not recording)


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
    if (
        "floor_enforce_toggle" in handles
        and isinstance(floor, Mapping)
        and not handles.get("floor_enforce_synced", False)
    ):
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


def _set_slider_server_range(handle: Any, minimum: float, maximum: float) -> None:
    """Apply a server-owned slider range without transiently invalid state.

    Viser validates the current value whenever min/max changes.  Expand first,
    clamp the value into the new server range, and only then shrink to the exact
    envelope.  This also handles stale settings from a wider previous profile.
    """
    if not (math.isfinite(minimum) and math.isfinite(maximum) and maximum > minimum):
        return
    current = float(handle.value)
    old_min = float(handle.min)
    old_max = float(handle.max)
    handle.min = min(old_min, minimum, current)
    handle.max = max(old_max, maximum, current)
    handle.value = max(minimum, min(maximum, current))
    handle.min = minimum
    handle.max = maximum


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
                        _set_slider_server_range(
                            handles[f"roi_{a}_{suffix}"],
                            runtime_min[k] * 1000.0,
                            runtime_max[k] * 1000.0,
                        )
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


def _update_chunk_overlay_gui(
    handles: dict[str, Any],
    chunk_overlay_store: ChunkOverlayStore | None,
    latest: StateSnapshot | None,
) -> None:
    overlay = chunk_overlay_store.latest() if chunk_overlay_store is not None else None
    try:
        history_count = int(handles.get("chunk_overlay_history_count", 12) or 0)
    except Exception:
        history_count = 12
    history_count = max(0, history_count)
    history = chunk_overlay_store.history(history_count) if chunk_overlay_store is not None else []
    try:
        persist_sec = float(handles.get("chunk_overlay_persist_sec", 30.0) or 30.0)
        if not math.isfinite(persist_sec) or persist_sec <= 0.0:
            raise ValueError
    except Exception:
        persist_sec = 30.0
    stale = overlay is None or overlay.stale(threshold_sec=persist_sec)
    now = time.monotonic()
    actual_positions: dict[str, tuple[float, float, float] | None] = {}
    for arm in ("left", "right"):
        arm_state = getattr(latest, arm, None) if latest is not None else None
        if (
            arm_state is not None
            and getattr(arm_state, "tcp_actual_valid", False)
            and arm_state.tcp_actual_stand is not None
        ):
            actual_positions[arm] = _pose_position(arm_state.tcp_actual_stand)
        else:
            actual_positions[arm] = None
    try:
        axes_stride = int(handles.get("chunk_overlay_axes_stride", 2) or 2)
    except Exception:
        axes_stride = 2
    errors = update_chunk_overlay(
        handles.get("scene", {}),
        overlay,
        stale=stale,
        visible=_chunk_overlay_visible(handles),
        now_monotonic=now,
        actual_positions=actual_positions,
        dot_size=handles.get("chunk_overlay_dot_size"),
        show_axes=bool(handles.get("chunk_overlay_axes_visible")),
        axes_stride=axes_stride,
        history_overlays=history,
    )
    handle = handles.get("chunk_overlay_error_text")
    if handle is not None:
        left = f"{errors['left'] * 1000.0:.0f}" if errors.get("left") is not None else "—"
        right = f"{errors['right'] * 1000.0:.0f}" if errors.get("right") is not None else "—"
        try:
            handle.value = f"L {left} / R {right}"
        except Exception:
            pass


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


_BOX_MESH: Any = "uninit"
_EXTERNAL_BOX_COLLISION_RED = (220, 60, 60)
# TODO: tune with operator.
_EXTERNAL_BOX_LABEL_Z_OFFSET_M = 0.06


def _box_mesh():
    """box.stl(open-tray) -> (verts[m, 중심정렬], faces). 1회 로드 캐시."""
    global _BOX_MESH
    if _BOX_MESH != "uninit":
        return _BOX_MESH
    try:
        import numpy as np
        import trimesh
        path = os.environ.get("RB_GUI_BOX_STL", "/home/plaif/workspace/box.stl")
        m = trimesh.load(path, force="mesh")
        v = np.asarray(m.vertices, dtype=np.float64)
        v = (v - (v.min(0) + v.max(0)) / 2.0) * 0.001        # 중심정렬 + mm->m
        _BOX_MESH = (v.astype(np.float32), np.asarray(m.faces, dtype=np.uint32))
    except Exception:
        _BOX_MESH = None
    return _BOX_MESH


def _update_stereo_boxes(handles: dict[str, Any], latest: StateSnapshot | None = None) -> None:
    """검출된 박스(stereo.boxes, T_stand)를 box.stl 메쉬로 각각 렌더."""
    server = handles.get("_server")
    store = handles.get("_stereo_store")
    toggle = handles.get("pc_box_enable")
    if toggle is None:
        toggle = handles.get("pc_enable")
    if server is None or store is None or toggle is None:
        return
    hs = handles.setdefault("_box_handles", {})
    label_hs = handles.setdefault("_box_dist_labels", {})
    if not bool(getattr(toggle, "value", False)):
        for h in hs.values():
            try:
                h.visible = False
            except Exception:
                pass
        for h in label_hs.values():
            try:
                h.visible = False
            except Exception:
                pass
        return
    mesh = _box_mesh()
    boxes, _seq = store.latest_boxes()
    label_color = {"green": (40, 220, 80), "gray": (170, 170, 180)}
    fallback = [(240, 150, 40), (80, 160, 240), (230, 60, 200), (240, 220, 60)]
    sc = latest.self_collision if latest is not None else None
    clearances = sc.get("external_box_clearance_m") if isinstance(sc, Mapping) else None
    if not isinstance(clearances, (list, tuple)):
        clearances = []
    seen = set()
    for i, b in enumerate(boxes[:4]):
        T = b["T"]; pos = tuple(float(v) for v in T[:3, 3])
        wxyz = tuple(float(v) for v in mat_to_wxyz(T[:3, :3]))
        box_label = b.get("label")
        slot = EXTERNAL_BOX_LABEL_SLOTS.get(str(box_label)) if box_label is not None else None
        clearance = clearances[slot] if slot is not None and slot < len(clearances) else None
        display = external_box_display(clearance)
        normal_color = label_color.get(box_label, fallback[i % 4])
        col_i = _EXTERNAL_BOX_COLLISION_RED if display["in_collision"] else normal_color
        name = f"box{i}"; seen.add(name)
        if name in hs:
            h = hs[name]; h.position = pos; h.wxyz = wxyz; h.visible = True
            try:
                h.color = col_i
            except Exception:
                pass
        elif mesh is not None:
            hs[name] = server.scene.add_mesh_simple(
                f"/stereo_box_{i}", mesh[0], mesh[1], color=col_i,
                opacity=0.55, side="double", flat_shading=True, position=pos, wxyz=wxyz)
        else:  # STL 로드 실패 시 박스로 폴백
            hs[name] = server.scene.add_box(
                f"/stereo_box_{i}", color=col_i, dimensions=tuple(float(v) for v in b["dims"]),
                wireframe=True, position=pos, wxyz=wxyz)
        dims = b.get("dims", (0.0, 0.0, 0.0))
        try:
            label_z = pos[2] + max(0.0, float(dims[2]) / 2.0) + _EXTERNAL_BOX_LABEL_Z_OFFSET_M
        except Exception:
            label_z = pos[2] + _EXTERNAL_BOX_LABEL_Z_OFFSET_M
        label_pos = (pos[0], pos[1], label_z)
        if display["show_label"]:
            label_text = display["label"]
        elif b.get("locked"):
            label_text = "🔒"
        else:
            label_text = None

        label_handle = label_hs.get(name)
        if label_text and hasattr(server.scene, "add_label"):
            if label_handle is None:
                label_hs[name] = server.scene.add_label(
                    f"/stereo_box_{i}_clearance",
                    text=label_text,
                    position=label_pos,
                    visible=True,
                )
            else:
                try:
                    label_handle.text = label_text
                    label_handle.position = label_pos
                    label_handle.visible = True
                except Exception:
                    pass
        elif label_handle is not None:
            try:
                label_handle.visible = False
            except Exception:
                pass
    for name, h in hs.items():
        if name not in seen:
            try:
                h.visible = False
            except Exception:
                pass
    for name, h in label_hs.items():
        if name not in seen:
            try:
                h.visible = False
            except Exception:
                pass


def _update_stereo_cloud(handles: dict[str, Any]) -> None:
    """stereo_worker 클라우드를 /stereo_cam(=T_stand_cam) 프레임의 자식으로 렌더.
    클라우드는 카메라 좌표계 점이고, 부모 프레임이 stand 배치를 담당. 캘리브 모드에선
    기즈모로 그 프레임을 옮겨 URDF 로봇팔과 정렬한다."""
    import numpy as np
    toggle = handles.get("pc_enable")
    server = handles.get("_server")
    store = handles.get("_stereo_store")
    status = handles.get("pc_status")
    if toggle is None or server is None or store is None:
        return

    # 캘리브레이션 모드: 기즈모 표시 + 기즈모 pose를 클라우드 부모 프레임에 복사
    gizmo = handles.get("pc_cam_gizmo")
    frame = handles.get("pc_cam_frame")
    calib = handles.get("pc_calib_mode")
    calib_on = bool(getattr(calib, "value", False))
    if gizmo is not None:
        gizmo.visible = calib_on
        if calib_on and frame is not None:
            frame.wxyz = gizmo.wxyz
            frame.position = gizmo.position
            cs = handles.get("pc_calib_status")
            if cs is not None:
                p = np.array(gizmo.position)
                cs.value = f"xyz=({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f}) m"

    if not toggle.value:
        h = handles.get("pc_handle")
        if h is not None:
            h.visible = False
        if status is not None:
            status.value = "off"
        return
    data = store.latest()
    if data is None:
        if status is not None:
            status.value = "waiting for stereo_worker (5601)…"
        return
    xyz, rgb, seq, age_ms = data
    dmin = float(handles["pc_dmin"].value) if handles.get("pc_dmin") else 0.0
    dmax = float(handles["pc_dmax"].value) if handles.get("pc_dmax") else 1e9
    max_pts = int(handles["pc_max_k"].value) * 1000 if handles.get("pc_max_k") else 80000
    psize = float(handles["pc_size"].value) if handles.get("pc_size") else 0.004
    # seq/필터/표시값이 모두 그대로면 재전송 생략 (중복 방지). 슬라이더 변경 시엔 즉시 갱신.
    param_key = (round(dmin, 3), round(dmax, 3), max_pts, round(psize, 4))
    key = (seq,) + param_key
    if key == handles.get("_pc_last_key") and handles.get("pc_handle") is not None:
        handles["pc_handle"].visible = True
        return
    # 슬라이더 변경(param_key)은 즉시 반영하되, 클라우드 seq 갱신(~14fps)만으로 오는 재렌더는
    # throttle해 웹소켓/브라우저 부하를 제한한다(노드 재전송이 비싸므로 ~5Hz로 캡).
    now = time.monotonic()
    params_changed = param_key != handles.get("_pc_last_param_key")
    if (not params_changed and handles.get("pc_handle") is not None
            and (now - handles.get("_pc_last_render_t", 0.0)) < 0.2):
        handles["pc_handle"].visible = True
        return
    # depth(=카메라 z) 범위 밖 점 제거
    z = xyz[:, 2]
    m = (z >= dmin) & (z <= dmax)
    xyz, rgb = xyz[m], rgb[m]
    if xyz.shape[0] == 0:
        h = handles.get("pc_handle")
        if h is not None:
            h.visible = False
        if status is not None:
            status.value = f"depth {dmin:.2f}~{dmax:.2f}m 범위 내 점 없음"
        handles["_pc_last_key"] = key
        return
    # 표시 다운샘플 (웹소켓 부하 제한)
    n_in = xyz.shape[0]
    if n_in > max_pts:
        idx = np.random.default_rng(seq if seq >= 0 else 0).choice(n_in, max_pts, replace=False)
        xyz, rgb = xyz[idx], rgb[idx]
    # /stereo_cam 의 자식 -> 부모 프레임(T_stand_cam)이 stand 배치 적용. 점은 카메라 좌표계.
    pts = xyz.astype(np.float32)
    cols = rgb.astype(np.uint8)
    h = handles.get("pc_handle")
    if h is not None and not params_changed:
        # seq만 바뀐 갱신은 노드 재생성 없이 buffer만 교체(점수 가변도 OK — user_floor와 동일 패턴).
        try:
            h.points = pts
            h.colors = cols
            h.visible = True
        except Exception:
            h = None
    else:
        h = None
    if h is None:  # 최초 생성 또는 슬라이더(point_size/shape) 변경 시에만 재생성
        handles["pc_handle"] = server.scene.add_point_cloud(
            "/stereo_cam/cloud", points=pts, colors=cols,
            point_size=psize, point_shape="rounded")
    handles["_pc_last_key"] = key
    handles["_pc_last_param_key"] = param_key
    handles["_pc_last_render_t"] = now
    if status is not None:
        status.value = f"{xyz.shape[0]}/{n_in} pts, depth {dmin:.2f}~{dmax:.2f}m, age {age_ms:.0f}ms (seq {seq})"


def _update_stereo_wrist(handles: dict[str, Any]) -> None:
    """손목(D405) raw 클라우드를 /stand/{arm}_tcp/wrist_cam(=T_tcp_cam) 자식으로 렌더.
    점은 카메라 광학 좌표계, 부모(실시간 TCP × 핸드아이)가 stand 배치를 담당. 캘리브
    모드에선 기즈모로 핸드아이 프레임을 옮겨 클라우드를 로봇팔과 정렬한다."""
    import numpy as np
    server = handles.get("_server")
    store = handles.get("_stereo_store")
    toggle = handles.get("pc_wrist_enable")
    if server is None or store is None or toggle is None:
        return

    gizmos = handles.get("_wrist_gizmo", {})
    frames = handles.get("_wrist_frame", {})
    hs = handles.setdefault("_wrist_handles", {})
    on = bool(toggle.value)
    dmin = float(handles["pc_dmin"].value) if handles.get("pc_dmin") else 0.0
    dmax = float(handles["pc_dmax"].value) if handles.get("pc_dmax") else 1e9
    max_pts = int(handles["pc_max_k"].value) * 1000 if handles.get("pc_max_k") else 80000
    psize = float(handles["pc_size"].value) if handles.get("pc_size") else 0.004
    # point_size(슬라이더) 변경 시에만 노드 재생성, 그 외엔 buffer만 in-place 교체.
    wrist_psize_changed = round(psize, 4) != handles.get("_wrist_last_psize")
    handles["_wrist_last_psize"] = round(psize, 4)

    # 캘리브 모드: 양손 기즈모 표시 + 동기화. 한쪽 기즈모가 움직이면 같은 로컬
    # pose(=공유 T_tcp_cam)를 양쪽 기즈모와 핸드아이 프레임에 전파한다.
    calib_on = bool(getattr(handles.get("pc_wrist_calib"), "value", False)) and on
    giz_l, giz_r = gizmos.get("left"), gizmos.get("right")
    if giz_l is not None and giz_r is not None:
        giz_l.visible = giz_r.visible = calib_on

        def _pose(g):
            return (tuple(np.round(np.asarray(g.wxyz), 7)), tuple(np.round(np.asarray(g.position), 7)))

        last = handles.get("_wrist_sync_last")
        pl, pr = _pose(giz_l), _pose(giz_r)
        driver = None
        if last is None:
            driver = "left"
        elif pl != last["left"]:
            driver = "left"
        elif pr != last["right"]:
            driver = "right"
        if driver is not None:                       # 구동 기즈모 → 양쪽 동기화
            g = gizmos[driver]
            w, p = tuple(np.asarray(g.wxyz)), tuple(np.asarray(g.position))
            for _a in ("left", "right"):
                gizmos[_a].wxyz = w; gizmos[_a].position = p
                frames[_a].wxyz = w; frames[_a].position = p
            snap = (tuple(np.round(w, 7)), tuple(np.round(p, 7)))
            handles["_wrist_sync_last"] = {"left": snap, "right": snap}
            cs = handles.get("pc_wrist_status_calib")
            if cs is not None:
                ap = np.asarray(p)
                cs.value = f"xyz=({ap[0]:.4f}, {ap[1]:.4f}, {ap[2]:.4f}) m  (양손 동기화)"

    parts_status = []
    for arm in ("left", "right"):
        h = hs.get(arm)
        if not on:
            if h is not None:
                h.visible = False
            continue
        data = store.latest_wrist(arm)
        if data is None:
            parts_status.append(f"{arm}: 대기")
            continue
        xyz, rgb, age_ms = data
        # 발행이 멈춘 쪽(카메라 사망/번들 정지)은 마지막 클라우드를 계속 그리지 말고
        # 숨긴다 — GUI만 봐도 어느 손목 카메라가 죽었는지 드러나야 한다.
        # (wrist 발행 주기는 worker ~3Hz × wrist_every=3 ≈ 1s → 5s면 확실한 사망 신호)
        if age_ms > 5000.0:
            if h is not None:
                h.visible = False
            parts_status.append(f"{arm}: ⚠ 끊김 {age_ms / 1000.0:.0f}s")
            continue
        if xyz.shape[0] == 0:
            if h is not None:
                h.visible = False
            parts_status.append(f"{arm}: 0 pts")
            continue
        z = xyz[:, 2]
        m = (z >= dmin) & (z <= dmax)
        xyz_f, rgb_f = xyz[m], rgb[m]
        n_in = xyz_f.shape[0]
        if n_in == 0:
            if h is not None:
                h.visible = False
            parts_status.append(f"{arm}: 범위밖")
            continue
        if n_in > max_pts:
            idx = np.random.default_rng(0).choice(n_in, max_pts, replace=False)
            xyz_f, rgb_f = xyz_f[idx], rgb_f[idx]
        pts = xyz_f.astype(np.float32)
        cols = rgb_f.astype(np.uint8)
        if h is not None and not wrist_psize_changed:
            try:
                h.points = pts
                h.colors = cols
                h.visible = True
            except Exception:
                h = None
        else:
            h = None
        if h is None:
            hs[arm] = server.scene.add_point_cloud(
                f"/stand/{arm}_tcp/wrist_cam/cloud",
                points=pts, colors=cols, point_size=psize, point_shape="rounded")
        parts_status.append(f"{arm}: {xyz_f.shape[0]} pts ({age_ms:.0f}ms)")

    st = handles.get("pc_wrist_status")
    if st is not None:
        st.value = ("off" if not on else (", ".join(parts_status) if parts_status else "waiting…"))


def _build_head_preview_panel(
    server: Any,
    handles: dict[str, Any],
    head_preview_store: HeadPreviewStore | None,
) -> None:
    """Head (D435 color) viewer next to the wrist previews. Display-only.

    The head rig is optional — `make cam-up-wrists` runs without the D435 — so an
    absent topic stays a `waiting` status instead of an error. Model inference is
    unaffected either way: it reads `camera.bundle.policy`, which is wrist-only.
    """

    if head_preview_store is None:
        server.gui.add_text(
            "Head view",
            initial_value="disabled (RB_GUI_HEAD_PREVIEW=0)",
            disabled=True,
        )
        return
    handles["head_preview_toggle"] = server.gui.add_checkbox(
        "head view 표시 (5 Hz)",
        initial_value=False,
    )
    handles["head_preview_status"] = server.gui.add_text(
        "Head view",
        initial_value=head_preview_store.status_text(),
        disabled=True,
    )
    handles["head_preview_image"] = server.gui.add_image(
        np.zeros((270, 480, 3), dtype=np.uint8),
        label="head",
        format="jpeg",
        jpeg_quality=75,
        visible=False,
    )
    handles["head_preview_last_monotonic"] = float("-inf")

    @handles["head_preview_toggle"].on_update
    def _(_: Any) -> None:
        enabled = bool(handles["head_preview_toggle"].value)
        # Gate the receiver too: while off it skips the 1280x720 shm copy.
        head_preview_store.set_enabled(enabled)
        image_handle = handles.get("head_preview_image")
        if image_handle is not None:
            image_handle.visible = enabled


def _update_head_preview(handles: dict[str, Any]) -> None:
    store = handles.get("head_preview_store")
    if not isinstance(store, HeadPreviewStore):
        return
    now = time.monotonic()
    status_handle = handles.get("head_preview_status")
    if status_handle is not None:
        status_handle.value = store.status_text(now=now)
    toggle = handles.get("head_preview_toggle")
    if not bool(getattr(toggle, "value", False)):
        return
    last = float(handles.get("head_preview_last_monotonic", float("-inf")))
    if now - last < 0.2:
        return
    image = store.preview()
    image_handle = handles.get("head_preview_image")
    if image is not None and image_handle is not None:
        try:
            image_handle.image = image
            image_handle.visible = True
        except Exception:
            pass
    handles["head_preview_last_monotonic"] = now


def _update_camera_quality(handles: dict[str, Any]) -> None:
    store = handles.get("camera_quality_store")
    if not isinstance(store, CameraQualityStore):
        return
    now = time.monotonic()
    status_handle = handles.get("camera_quality_status")
    if status_handle is not None:
        try:
            status_handle.content = camera_quality_html(store, now=now)
        except Exception:
            pass
    csv_handle = handles.get("camera_quality_csv")
    if csv_handle is not None:
        status = store.status()
        csv_text = status["csv_path"] or "unavailable"
        if status["csv_error"]:
            csv_text += " · ERROR " + status["csv_error"]
        csv_handle.value = csv_text

    preview_toggle = handles.get("camera_quality_preview_toggle")
    preview_on = bool(getattr(preview_toggle, "value", False))
    last_preview = float(
        handles.get("camera_quality_last_preview_monotonic", float("-inf"))
    )
    if preview_on and now - last_preview >= 0.2:
        for arm in CAMERA_QUALITY_ARMS:
            image = store.preview(arm)
            preview_handle = handles.get(f"camera_quality_preview_{arm}")
            if image is not None and preview_handle is not None:
                try:
                    preview_handle.image = image
                    preview_handle.visible = True
                except Exception:
                    pass
        handles["camera_quality_last_preview_monotonic"] = now


def update_gui(
    handles: dict[str, Any],
    safety: OperatorSafety,
    store: StateStore,
    overlay_store: CircleOverlayStore | None = None,
    chunk_overlay_store: ChunkOverlayStore | None = None,
) -> None:
    _update_stereo_cloud(handles)  # 로봇 상태와 무관 — 항상 갱신
    latest = store.latest()
    _update_stereo_boxes(handles, latest)  # 검출된 박스 렌더
    _update_box_detect_status(handles)  # 박스 재탐지 버튼 상태(green/gray 잠금 텍스트)
    _update_stereo_wrist(handles)  # 손목 raw 클라우드 오버레이
    disabled_states = safety.control_disabled_states()
    disabled_reasons = safety.control_disabled_reasons()
    for mode, button in handles.get("lifecycle_buttons", {}).items():
        _set_disabled(button, disabled_states.get(f"lifecycle:{mode}", True))
    if "init_motion_buttons" in handles:
        for button in handles["init_motion_buttons"].values():
            _set_disabled(button, disabled_states.get("init_motion", True))
    elif "init_motion_button" in handles:
        _set_disabled(handles["init_motion_button"], disabled_states.get("init_motion", True))
    if "jog_button" in handles:
        _set_disabled(handles["jog_button"], disabled_states.get("jog", True))
    for button in handles.get("tcp_pose_buttons", ()):
        _set_disabled(button, disabled_states.get("tcp_pose", True))
    for button in handles.get("tcp_linear_buttons", ()):
        _set_disabled(button, disabled_states.get("tcp_linear", True))
    # Button-group tabs cannot be greyed in viser, so reflect the live gate
    # reason into their status line proactively.
    _reflect_gate_reason(handles.get("jog_status"), disabled_reasons.get("jog"))

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
    stale = store.is_stale()
    _update_camera_quality(handles)
    _update_head_preview(handles)
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
    _update_chunk_overlay_gui(handles, chunk_overlay_store, latest)
    _update_recording_panel(handles, latest, stale=stale)
    _update_spacemouse_panel(handles)
    _update_arm_init_panel(handles, latest, stale=stale)
    if latest is None:
        _update_realtime_health(
            handles,
            None,
            chunk_overlay_store,
            stale=True,
        )
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
        if "ft_status" in handles:
            handles["ft_status"].value = _format_ft_status(None, stale=True)
        if "fc_status" in handles:
            handles["fc_status"].value = _format_force_control_status(None, stale=True)
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
        if "init_motion_runtime" in handles:
            handles["init_motion_runtime"].value = _format_init_motion_status(None, stale=True)
        if "tcp_status" in handles:
            handles["tcp_status"].value = _format_tcp_command_status(safety, None, stale=True)
        if "tcp_linear_status" in handles:
            handles["tcp_linear_status"].value = _format_tcp_command_status(safety, None, stale=True)
        handles["packets"].value = f"{store.received_packets} received / {store.invalid_packets} invalid"
        return

    handles["connection"].value = "stale" if stale else "live"
    _update_realtime_health(
        handles,
        latest,
        chunk_overlay_store,
        stale=stale,
    )
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
    if "roi_box" in handles:
        handles["roi_box"].value = _format_roi_box_status(latest, stale=stale)
    if "user_floor_constraint" in handles:
        handles["user_floor_constraint"].value = _format_user_floor_constraint_status(latest, stale=stale)
    _update_floor_panel(handles, latest)
    if "roi_box" in handles:
        handles["roi_box"].value = _format_roi_box_status(latest, stale=stale)
    _update_roi_panel(handles, latest)
    if "user_floor_constraint" in handles:
        handles["user_floor_constraint"].value = _format_user_floor_constraint_status(latest, stale=stale)
    update_user_floor_plane(handles.get("scene", {}), latest.user_floor_constraint)
    update_user_floor_capture_points(
        handles.get("scene", {}), _user_floor_display_points(handles))
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
        rejected = uf.get("last_set_reject_reason") if isinstance(uf, Mapping) else None
        config_enabled = handles.get("user_floor_config_enabled")
        if config_enabled is None:
            config_enabled = server_safety_constraint_config_enabled(
                "user_floor_constraint"
            )
        attempts = handles.get("user_floor_resend_attempts", 0)
        disabled_by_config = config_enabled is False or rejected == "user_floor_disabled"
        if disabled_by_config:
            handles["user_floor_resent"] = True
            if "user_floor_set_status" in handles:
                handles["user_floor_set_status"].value = (
                    "restore skipped: user floor disabled by server config"
                )
        elif not want or confirmed or attempts >= 50:
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
    if "fk_status" in handles:
        handles["fk_status"].value = _format_fk_status(latest, stale=stale)
    if "ft_status" in handles:
        handles["ft_status"].value = _format_ft_status(latest, stale=stale)
    if "fc_status" in handles:
        handles["fc_status"].value = _format_force_control_status(latest, stale=stale)
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
    if "init_motion_runtime" in handles:
        handles["init_motion_runtime"].value = _format_init_motion_status(latest, stale=stale)
    if "tcp_status" in handles:
        handles["tcp_status"].value = _format_tcp_command_status(safety, latest, stale=stale)
    if "tcp_linear_status" in handles:
        handles["tcp_linear_status"].value = _format_tcp_command_status(safety, latest, stale=stale)
    _update_operator_monitors(handles, latest, stale=stale)
    _push_gripper_percent(handles, latest)
    _update_gripper_feedback(handles, latest, stale=stale)
    update_scene_markers(
        handles.get("scene", {}),
        latest,
        tcp_display_mode=_tcp_display_mode(handles),
        show_tcp_gizmo=_tcp_gizmo_visible(handles),
    )
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


_VISER_WASD_PATCH_MARKER = "rb-disable-wasd-keys"
_VISER_WASD_BUNDLE_MARKER = "rb-wasd-bundle-patched"
_VISER_KEYBOARD_PATCH_VERSION = "rb-disable-wasd-keys-recording-initmotion-v4"


def _patch_viser_bundle_wasd(html: str) -> str:
    """Source-level kill of the W/A/S/D camera keys inside viser's client bundle.

    viser embeds its JS bundle zstd-compressed in index.html (`data-c`, decompressed
    size `data-cs`) and binds keys with hold-event: `new Gg("KeyW",1e3/60)` etc.,
    listening on `document`. We decompress the bundle, rename ONLY the W/A/S/D key
    codes to bogus codes (so `event.code` never matches -> the hold never fires),
    recompress, and re-embed. Q/E (up/down), arrows, and mouse are untouched; 'a' is
    freed. Same-length renames keep the decompressed size (`data-cs`) valid.

    Best-effort: returns html unchanged if zstandard is unavailable, the binding
    pattern is absent (already patched / a different viser build), or the round-trip
    check fails — so we never write a bundle the browser cannot decode."""
    if _VISER_WASD_BUNDLE_MARKER in html:
        return html  # already patched; skip the (expensive) decompress
    try:
        import base64
        import re

        import zstandard as zstd
    except Exception:
        return html
    m = re.search(r'data-c="([^"]*)"', html)
    mcs = re.search(r'data-cs="(\d+)"', html)
    if not m or not mcs:
        return html
    expected = int(mcs.group(1))
    try:
        js = (
            zstd.ZstdDecompressor()
            .decompress(base64.b64decode(m.group(1)), max_output_size=expected + 64)
            .decode("utf-8")
        )
    except Exception:
        return html
    patched = js
    for code, repl in (("KeyW", "XzzW"), ("KeyA", "XzzA"), ("KeyS", "XzzS"), ("KeyD", "XzzD")):
        patched = patched.replace(f'"{code}",1e3/60', f'"{repl}",1e3/60')
    if patched == js:
        return html  # pattern not found -> nothing to do
    payload = patched.encode("utf-8")
    if len(payload) != expected:
        return html  # size drift guard: never write a bundle that won't fit data-cs
    new_blob = zstd.ZstdCompressor().compress(payload)
    try:  # verify the recompressed frame round-trips before committing
        if zstd.ZstdDecompressor().decompress(new_blob, max_output_size=expected + 64) != payload:
            return html
    except Exception:
        return html
    new_html = html[: m.start(1)] + base64.b64encode(new_blob).decode("ascii") + html[m.end(1):]
    # Cheap marker so subsequent launches skip the decompress.
    return new_html.replace("<head>", "<head><!--" + _VISER_WASD_BUNDLE_MARKER + "-->", 1)


def _viser_keyboard_patch_script() -> str:
    return (
        '<script id="' + _VISER_WASD_PATCH_MARKER + '">'
        "(function(){var V='" + _VISER_KEYBOARD_PATCH_VERSION + "';"
        "var B={KeyW:1,KeyA:1,KeyS:1,KeyD:1};"
        "function t(e){if(!e)return false;var g=(e.tagName||'').toUpperCase();"
        "return g==='INPUT'||g==='TEXTAREA'||g==='SELECT'||e.isContentEditable||"
        "(e.closest&&e.closest('input,textarea,select,[contenteditable=true]'));}"
        "function s(e){e.stopImmediatePropagation();e.preventDefault();}"
        "function mod(e){return e.ctrlKey||e.metaKey||e.altKey;}"
        "function txt(b){return (b.textContent||b.innerText||'').trim();}"
        "function r(){var a=Array.prototype.slice.call(document.querySelectorAll('button'));"
        "var h=a.find(function(b){return !b.disabled&&txt(b).indexOf('record-toggle-hotkey-b')>=0;});"
        "if(h){h.click();return;}"
        "var stop=a.find(function(b){return !b.disabled&&txt(b).indexOf('수집 종료')>=0;});"
        "var start=a.find(function(b){return !b.disabled&&txt(b).indexOf('수집 시작')>=0;});"
        "(stop||start||{}).click&& (stop||start).click();}"
        # InitMotion hotkeys: 'a' = left arm, 'c' = right arm. Click the matching GUI
        # button (by 'InitMotion' + side token) so the press routes through the same
        # arm-init path as the click and works during any motion. Skipped while a button
        # is disabled (init not allowed in the current state) or a modifier is held (so
        # Ctrl/Cmd+A/C are never hijacked).
        "function initArm(tok){var a=Array.prototype.slice.call(document.querySelectorAll('button'));"
        "var b=a.find(function(x){return !x.disabled&&txt(x).indexOf('InitMotion')>=0&&txt(x).indexOf(tok)>=0;});"
        "if(b)b.click();}"
        "function d(e){if(t(e.target))return;"
        "if(e.repeat){if(!mod(e)&&(e.code==='KeyA'||e.code==='KeyC'||e.code==='KeyB'||B[e.code]))s(e);return;}"
        "if(e.code==='KeyA'&&!mod(e)){s(e);initArm('(왼팔)');return;}"
        "if(e.code==='KeyC'&&!mod(e)){s(e);initArm('(오른팔)');return;}"
        "if(B[e.code]){s(e);return;}"
        "if(e.code==='KeyB'){s(e);r();}}"
        "function u(e){if(!t(e.target)&&B[e.code])s(e);}"
        "window.addEventListener('keydown',d,true);"
        "window.addEventListener('keyup',u,true);"
        "window.__rbGuiKeyboardPatch=V;})();</script>"
    )


def _disable_viser_wasd_keys() -> None:
    """Disable viser's WASD camera fly-movement (W/A/S/D) in the served client.

    Q/E (up/down), the arrow keys, and mouse drag/zoom are kept; W/A/S/D camera
    fly-movement is suppressed and the injected handler repurposes 'a'/'c' as the
    InitMotion left/right hotkeys ('b' stays the record toggle). viser bakes these key
    handlers into its frontend bundle with NO Python API to disable them, so we patch
    the served client index.html two ways:
      1. Source-level: rename the W/A/S/D hold-event key codes in the JS bundle so
         the bindings never fire (the robust fix — see _patch_viser_bundle_wasd).
      2. Fallback: inject a capture-phase key blocker (covers builds where the
         bundle pattern is absent or zstandard is unavailable).

    Idempotent + best-effort: skipped when already patched, re-applied every launch
    (survives a viser reinstall / client autobuild), and a missing/read-only client
    is logged and skipped — never fatal to GUI startup. NOTE: the browser must load
    the freshly-patched page (a normal reload; hard-refresh if a tab was left open)."""
    try:
        import viser

        index = os.path.join(
            os.path.dirname(os.path.abspath(viser.__file__)), "client", "build", "index.html"
        )
        with open(index, "r", encoding="utf-8") as fh:
            html = fh.read()
        original = html
        html = _patch_viser_bundle_wasd(html)
        if _VISER_KEYBOARD_PATCH_VERSION not in html:
            import re

            script = _viser_keyboard_patch_script()
            pattern = rf'<script id="{re.escape(_VISER_WASD_PATCH_MARKER)}">.*?</script>'
            html, count = re.subn(pattern, script, html, count=1, flags=re.S)
            if count == 0:
                html = html.replace("<head>", "<head>" + script, 1) if "<head>" in html else script + html
        if html != original:
            with open(index, "w", encoding="utf-8") as fh:
                fh.write(html)
            print(
                "rb_servo_gui: patched viser WASD keys and episode hotkey B "
                "(Q/E, arrows, mouse kept; reload the GUI page)",
                flush=True,
            )
    except Exception as exc:  # noqa: BLE001 - cosmetic patch must never block GUI startup
        print(
            f"rb_servo_gui: could not patch viser WASD keys ({type(exc).__name__}: {exc})",
            flush=True,
        )


_FOOT_PEDAL_DEVICE_ENV = "RB_GUI_FOOT_PEDAL_DEVICE"


def _input_node_by_path(node: str) -> str:
    """/dev/input/eventN -> its /dev/input/by-path symlink, or the node unchanged.

    by-path keys off the physical USB port, so it survives replug/reboot (the eventN
    number does not) and stays unique when several identical pedals are attached."""
    try:
        target = os.path.realpath(node)
        for link in glob.glob("/dev/input/by-path/*-event-kbd"):
            if os.path.realpath(link) == target:
                return link
    except Exception:
        pass
    return node


def _foot_pedal_candidates(evdev, ecodes) -> list[str]:
    """Every foot-pedal KEYBOARD node, as stable by-path paths (sorted, deduped).

    Deliberately does NOT use /dev/input/by-id. Two PCsensor pedals are identical
    down to idVendor:idProduct (3553:b001), bcdDevice, product string, interface
    layout and evdev capability bitmap, and they carry no serial — so udev creates
    `usb-PCsensor_FootSwitch-event-kbd` for whichever enumerates first and NONE for
    the other. That symlink therefore silently points at a different physical pedal
    depending on plug order, and the second pedal is unreachable through it."""
    try:
        nodes = evdev.list_devices()
    except Exception:
        return []
    found: list[str] = []
    for path in nodes:
        try:
            cand = evdev.InputDevice(path)
        except Exception:
            continue
        try:
            name = (cand.name or "").lower()
            keys = cand.capabilities().get(ecodes.EV_KEY, [])
            # Keyboard interface only: the pedal's extra HID node may emit a key-down
            # without the matching key-up, and the mouse node emits no usable keys.
            if (("foot" in name or "pcsensor" in name)
                    and "mouse" not in name
                    and ecodes.KEY_A in keys):
                found.append(_input_node_by_path(path))
        finally:
            try:
                cand.close()
            except Exception:
                pass
    return sorted(set(found))


def _open_foot_pedal_device():
    """Open + exclusively grab the foot-pedal keyboard device via evdev.

    The PCsensor FootSwitch presents as a keyboard emitting a/b/c. Reading it
    directly works regardless of viser/browser focus or which terminal is active,
    and grab() (EVIOCGRAB) keeps the pedal from ALSO typing a/b/c into whatever
    window is focused. Returns a grabbed evdev.InputDevice or None (evdev missing,
    no matching device, no permission, or an ambiguous choice). Never raises.

    Device selection: $RB_GUI_FOOT_PEDAL_DEVICE if set, else auto-detect ONLY when
    exactly one foot-pedal keyboard is attached. With several attached the choice is
    safety-relevant and undecidable in software — this pedal fires InitMotion, and
    grab() would additionally steal a pedal another consumer needs (the pika UMI
    teleop clutch reads its own pedal, see ~/workspace/pika). Identical pedals cannot
    be told apart by any descriptor, so guessing could both move the robot from the
    wrong pedal and kill the teleop clutch. Fail closed and make the operator pin it."""
    try:
        import evdev
        from evdev import ecodes
    except Exception:
        print(
            "rb_servo_gui: foot pedal disabled (python-evdev not installed; `pip install evdev`)",
            flush=True,
        )
        return None
    dev = None
    override = os.environ.get(_FOOT_PEDAL_DEVICE_ENV, "").strip()
    if override:
        try:
            dev = evdev.InputDevice(override)
        except Exception as exc:  # noqa: BLE001
            print(
                f"rb_servo_gui: foot pedal {_FOOT_PEDAL_DEVICE_ENV}={override} unusable "
                f"({type(exc).__name__}: {exc})",
                flush=True,
            )
            return None
    else:
        candidates = _foot_pedal_candidates(evdev, ecodes)
        if len(candidates) > 1:
            print(
                "rb_servo_gui: foot pedal DISABLED — several pedals attached and they are "
                "indistinguishable in software (same 3553:b001, no serial, same capabilities). "
                f"Pin one with {_FOOT_PEDAL_DEVICE_ENV}=<path>:\n  "
                + "\n  ".join(candidates),
                flush=True,
            )
            return None
        if candidates:
            try:
                dev = evdev.InputDevice(candidates[0])
            except Exception as exc:  # noqa: BLE001
                print(
                    f"rb_servo_gui: foot pedal {candidates[0]} unusable "
                    f"({type(exc).__name__}: {exc})",
                    flush=True,
                )
                return None
    if dev is None:
        print(
            f"rb_servo_gui: foot pedal not found (set {_FOOT_PEDAL_DEVICE_ENV} to its /dev/input path)",
            flush=True,
        )
        return None
    try:
        dev.grab()
    except Exception as exc:  # noqa: BLE001
        # EBUSY means another process already grabbed it — not a permission problem.
        # The usual culprit is the pika UMI teleop publisher started BEFORE this GUI:
        # its --mute-other-pedals (on by default in run_umi_teleop_publish.sh) grabs
        # every pedal it is not using, to stop pedal keystrokes leaking into the
        # terminal. Start robotics_lab first, or run that publisher with
        # MUTE_OTHER_PEDALS=0.
        busy = getattr(exc, "errno", None) == errno.EBUSY
        hint = ("another process already holds it (pika teleop publisher with "
                "--mute-other-pedals? start robotics_lab first, or set MUTE_OTHER_PEDALS=0)"
                if busy else
                "add your user to the 'input' group (sudo usermod -aG input $USER) and re-login")
        print(
            f"rb_servo_gui: foot pedal grab failed on {dev.path} "
            f"({type(exc).__name__}: {exc}); {hint}",
            flush=True,
        )
        try:
            dev.close()
        except Exception:
            pass
        return None
    print(
        f"rb_servo_gui: foot pedal active on {dev.path} ({dev.name}); "
        "a=InitMotion left, c=InitMotion right, b=record toggle",
        flush=True,
    )
    return dev


def _foot_pedal_action_map(safety: "OperatorSafety", handles: dict[str, Any]) -> dict:
    """evdev key code -> (label, action) for the foot pedal.

    a -> InitMotion left, c -> InitMotion right, b -> record toggle. Each action
    reuses the exact GUI handler (same routing as a viser button click). Returns
    {} if evdev is unavailable."""
    try:
        from evdev import ecodes as e
    except Exception:
        return {}
    return {
        e.KEY_A: ("InitMotion left", lambda: _send_arm_init_override(
            safety, handles.get("scene", {}), handles, "left")),
        e.KEY_C: ("InitMotion right", lambda: _send_arm_init_override(
            safety, handles.get("scene", {}), handles, "right")),
        e.KEY_B: ("record toggle", lambda: _toggle_episode_recording(handles)),
    }


def _foot_pedal_loop(safety: "OperatorSafety", handles: dict[str, Any]) -> None:
    """Map foot-pedal key-down events to robot actions, independent of the browser.

    Works during any motion and whether or not a viser tab is open/focused (reads
    the device directly). Reconnects on unplug; a handler error never kills the loop."""
    try:
        from evdev import ecodes as e
    except Exception:
        return
    actions = _foot_pedal_action_map(safety, handles)
    while True:
        dev = _open_foot_pedal_device()
        if dev is None:
            return  # evdev missing, no device, or no permission -> stay disabled this run.
        try:
            for event in dev.read_loop():
                if event.type != e.EV_KEY or event.value != 1:  # key-DOWN only (ignore up/repeat)
                    continue
                entry = actions.get(event.code)
                if entry is None:
                    continue
                label, fn = entry
                try:
                    fn()
                except Exception as exc:  # noqa: BLE001
                    print(
                        f"rb_servo_gui: foot pedal {label} failed ({type(exc).__name__}: {exc})",
                        flush=True,
                    )
        except OSError as exc:
            print(
                f"rb_servo_gui: foot pedal disconnected ({type(exc).__name__}: {exc}); reconnecting",
                flush=True,
            )
            try:
                dev.close()
            except Exception:
                pass
            time.sleep(2.0)  # then re-open via the loop (handles replug)
        except Exception as exc:  # noqa: BLE001
            print(
                f"rb_servo_gui: foot pedal loop stopped ({type(exc).__name__}: {exc})",
                flush=True,
            )
            try:
                dev.close()
            except Exception:
                pass
            return


def _start_foot_pedal_listener(safety: "OperatorSafety", handles: dict[str, Any]) -> None:
    """Start the global foot-pedal listener as a daemon thread (best-effort)."""
    try:
        thread = threading.Thread(
            target=_foot_pedal_loop,
            args=(safety, handles),
            name="rb-foot-pedal",
            daemon=True,
        )
        thread.start()
        handles["_foot_pedal_thread"] = thread
    except Exception as exc:  # noqa: BLE001
        print(
            f"rb_servo_gui: foot pedal listener not started ({type(exc).__name__}: {exc})",
            flush=True,
        )


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
    # Persisted init-motion pose (set from a waypoint via the GUI) wins over
    # env/default so it survives restarts without editing code.
    saved_left_init, saved_right_init = _load_init_joints()
    if saved_left_init is not None and saved_right_init is not None:
        init_left_joints, init_right_joints = saved_left_init, saved_right_init
        print(f"rb_servo_gui: loaded init-motion pose from {_init_motion_path()}", flush=True)
    init_motion_timeout_sec = _env_float("RB_GUI_INIT_MOTION_TIMEOUT_SEC", 10.0)

    store = StateStore()
    camera_quality_store: CameraQualityStore | None = None
    camera_quality_receiver: CameraQualityReceiver | None = None
    if os.environ.get("RB_GUI_CAMERA_QUALITY", "1") != "0":
        robot_motion = RobotMotionTracker()
        store.add_update_callback(robot_motion.update)
        camera_quality_store = CameraQualityStore(
            csv_dir=os.environ.get(
                "RB_GUI_CAMERA_QUALITY_CSV_DIR",
                "logs/camera_quality",
            )
        )
        camera_quality_receiver = CameraQualityReceiver(
            camera_quality_store,
            robot_motion,
            endpoint=os.environ.get(
                "RB_GUI_CAMERA_QUALITY_ENDPOINT",
                "tcp://127.0.0.1:5600",
            ),
            topics={
                "left": os.environ.get(
                    "RB_GUI_CAMERA_QUALITY_LEFT_TOPIC",
                    "camera.bundle.wrist_left",
                ),
                "right": os.environ.get(
                    "RB_GUI_CAMERA_QUALITY_RIGHT_TOPIC",
                    "camera.bundle.wrist_right",
                ),
            },
        )
        camera_quality_receiver.start()
    # Optional head (D435 color) viewer. Independent of the wrist quality path so
    # it works with RB_GUI_CAMERA_QUALITY=0, and stays quiet on the wrist-only rig
    # (`make cam-up-wrists` publishes no head bundle group).
    head_preview_store: HeadPreviewStore | None = None
    head_preview_receiver: HeadPreviewReceiver | None = None
    if os.environ.get("RB_GUI_HEAD_PREVIEW", "1") != "0":
        head_preview_store = HeadPreviewStore(
            topic=os.environ.get("RB_GUI_HEAD_PREVIEW_TOPIC", DEFAULT_HEAD_TOPIC),
            stream=os.environ.get("RB_GUI_HEAD_PREVIEW_STREAM", DEFAULT_HEAD_STREAM),
        )
        head_preview_receiver = HeadPreviewReceiver(
            head_preview_store,
            endpoint=os.environ.get(
                "RB_GUI_HEAD_PREVIEW_ENDPOINT",
                "tcp://127.0.0.1:5600",
            ),
        )
        head_preview_receiver.start()
    receiver = StateReceiver(store, host=state_host, port=state_port)
    receiver.start()
    # 스테레오 pointcloud 워커(camera_server 컨테이너) 구독. 고급→Pointcloud 토글로 표시.
    stereo_store = StereoCloudStore()
    stereo_endpoint = os.environ.get("RB_GUI_STEREO_CLOUD_ENDPOINT", "tcp://127.0.0.1:5601")
    stereo_receiver = StereoCloudReceiver(stereo_store, endpoint=stereo_endpoint)
    stereo_receiver.start()
    circle_overlay_bind = _circle_overlay_bind_from_args_env(args)
    overlay_store: CircleOverlayStore | None = None
    overlay_receiver: CircleOverlayReceiver | None = None
    if circle_overlay_bind is not None:
        overlay_host, overlay_port = circle_overlay_bind
        overlay_store = CircleOverlayStore()
        overlay_receiver = CircleOverlayReceiver(overlay_store, host=overlay_host, port=overlay_port)
        overlay_receiver.start()
    chunk_overlay_bind = _chunk_overlay_bind_from_args_env(args)
    chunk_overlay_store: ChunkOverlayStore | None = None
    chunk_overlay_receiver: ChunkOverlayReceiver | None = None
    if chunk_overlay_bind is not None:
        chunk_overlay_host, chunk_overlay_port = chunk_overlay_bind
        chunk_overlay_store = ChunkOverlayStore()
        chunk_overlay_receiver = ChunkOverlayReceiver(
            chunk_overlay_store,
            host=chunk_overlay_host,
            port=chunk_overlay_port,
        )
        chunk_overlay_receiver.start()
    recording_status_store = RecordingStatusStore()
    recording_status_receiver: RecordingStatusReceiver | None = None
    recording_status_host, recording_status_port = _recording_status_bind_endpoint()
    try:
        recording_status_receiver = RecordingStatusReceiver(
            recording_status_store,
            host=recording_status_host,
            port=recording_status_port,
        )
        recording_status_receiver.start()
    except OSError as exc:
        print(
            f"rb_servo_gui: recording status receiver disabled ({type(exc).__name__}: {exc})",
            flush=True,
        )
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
    # viser's WASD camera fly-movement is baked into its client bundle with no
    # Python toggle; patch the served client to disable W/A/S/D (Q/E, arrows, mouse
    # kept). Done after ViserServer init (post client-autobuild) and before clients
    # connect. See _disable_viser_wasd_keys.
    _disable_viser_wasd_keys()
    handles = build_gui(
        server,
        safety,
        store,
        overlay_store=overlay_store,
        chunk_overlay_store=chunk_overlay_store,
        recording_status_store=recording_status_store,
        camera_quality_store=camera_quality_store,
        head_preview_store=head_preview_store,
    )
    handles["_stereo_store"] = stereo_store
    # Global foot-pedal hotkeys (PCsensor FootSwitch -> a/b/c), read straight from the
    # input device so they fire regardless of the viser tab being open/focused or which
    # terminal is active. a=InitMotion left, c=InitMotion right, b=record toggle.
    _start_foot_pedal_listener(safety, handles)
    overlay_status = (
        f", circle overlay UDP {circle_overlay_bind[0]}:{circle_overlay_bind[1]}"
        if circle_overlay_bind is not None
        else ", circle overlay disabled"
    )
    recording_status = (
        f", recording status UDP {recording_status_host}:{recording_status_port}"
        if recording_status_receiver is not None
        else ", recording status disabled"
    )
    chunk_overlay_status = (
        f", chunk overlay UDP {chunk_overlay_bind[0]}:{chunk_overlay_bind[1]}"
        if chunk_overlay_bind is not None
        else ", chunk overlay disabled"
    )
    camera_quality_status = (
        f", camera quality {camera_quality_receiver.endpoint}"
        if camera_quality_receiver is not None
        else ", camera quality disabled"
    )
    head_preview_status = (
        f", head view {head_preview_store.topic}/{head_preview_store.stream}"
        if head_preview_receiver is not None and head_preview_store is not None
        else ", head view disabled"
    )
    print(
        f"rb_servo_gui listening on http://{host}:{port}, UDP state {state_host}:{state_port}"
        f"{overlay_status}{chunk_overlay_status}{recording_status}{camera_quality_status}"
        f"{head_preview_status}",
        flush=True,
    )

    try:
        while True:
            update_gui(
                handles,
                safety,
                store,
                overlay_store=overlay_store,
                chunk_overlay_store=chunk_overlay_store,
            )
            time.sleep(0.1)
    finally:
        receiver.stop()
        if camera_quality_receiver is not None:
            camera_quality_receiver.stop()
        if camera_quality_store is not None:
            camera_quality_store.close()
        if head_preview_receiver is not None:
            head_preview_receiver.stop()
        stereo_receiver.stop()
        if overlay_receiver is not None:
            overlay_receiver.stop()
        if chunk_overlay_receiver is not None:
            chunk_overlay_receiver.stop()
        if recording_status_receiver is not None:
            recording_status_receiver.stop()
        close_recording_cmd = getattr(handles.get("recording_cmd_client"), "close", None)
        if callable(close_recording_cmd):
            close_recording_cmd()
        for client in list(getattr(handles.get("recording_cmd_clients"), "values", lambda: [])()):
            close_client = getattr(client, "close", None)
            if callable(close_client):
                close_client()


if __name__ == "__main__":
    main()
