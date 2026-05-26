from __future__ import annotations

import math
import os
from pathlib import Path
from typing import Any

from .geometry import (
    _mount_pose_from_mounts,
    _normalize_wxyz,
    _pose6_from_transform,
    _pose_orientation_wxyz,
    _pose_position,
    _pose_wxyz,
)

_ROBOT_JOINT_NAMES = (
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist1_joint",
    "wrist2_joint",
    "wrist3_joint",
)
# Rotation values are canonical URDF/ROS RPY converted from MJCF euler xyz.
_DEFAULT_LEFT_POSE = (0.1601, -0.1725, 0.5825, 2.186649, 0.523831, 2.526296)
_DEFAULT_RIGHT_POSE = (-0.1601, -0.1725, 0.5825, 2.186649, -0.523831, -2.526296)
_DEFAULT_STAND_MESH_POSE = (0.0, 0.0, 0.01, 0.0, 0.0, -1.57078)


def _repo_descriptions_dir() -> Path:
    workspace_root = Path(__file__).resolve().parents[2]
    candidates = (
        workspace_root / "rb_servo_server" / "descriptions",
        workspace_root / "descriptions",
    )
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def _descriptions_dir() -> Path:
    return Path(os.environ.get("RB_GUI_DESCRIPTIONS_DIR", str(_repo_descriptions_dir())))


def _asset_path(env_name: str, relative_default: str) -> Path:
    configured = os.environ.get(env_name)
    if configured:
        return Path(configured)
    return _descriptions_dir() / relative_default


def _robot_urdf_path() -> Path:
    return _asset_path("RB_GUI_ROBOT_URDF", "urdf/rb3_730e.urdf")


def _stand_mesh_path() -> Path:
    return _asset_path("RB_GUI_STAND_MESH", "meshes/stands/dual_rb3_730e/dual_rb3_730e_stand_ver3.stl")


def _joint_cfg_radians(q_values: tuple[float, ...] | None) -> tuple[float, ...]:
    if q_values is None:
        return tuple(0.0 for _ in _ROBOT_JOINT_NAMES)
    padded = tuple(float(q_values[index]) if index < len(q_values) else 0.0 for index in range(len(_ROBOT_JOINT_NAMES)))
    return tuple(math.radians(value) for value in padded)


def _joint_marker_position(base: tuple[float, float, float], q_values: tuple[float, ...] | None) -> tuple[float, float, float]:
    # Marker-only fallback: not FK. It gives operators visible left/right state
    # changes without pretending Cartesian kinematics are available.
    if q_values is None:
        return (base[0], base[1], base[2] + 0.04)
    shoulder = q_values[0] / 180.0 if q_values else 0.0
    elbow = q_values[1] / 180.0 if len(q_values) > 1 else 0.0
    wrist = q_values[2] / 180.0 if len(q_values) > 2 else 0.0
    return (base[0] + 0.08 * shoulder, base[1] + 0.08 * elbow, base[2] + 0.04 + 0.06 * wrist)


def _add_stand_mesh(server: Any, handles: dict[str, Any]) -> None:
    stand_mesh_path = _stand_mesh_path()
    if not stand_mesh_path.exists():
        handles["stand_mesh_error"] = f"stand mesh not found: {stand_mesh_path}"
        return
    try:
        import trimesh

        mesh = trimesh.load_mesh(str(stand_mesh_path))
        mesh.apply_scale(0.001)
        handles["stand_mesh"] = server.scene.add_mesh_trimesh(
            "/stand/mesh",
            mesh=mesh,
            position=_pose_position(_DEFAULT_STAND_MESH_POSE),
            wxyz=_pose_wxyz(_DEFAULT_STAND_MESH_POSE),
        )
    except Exception as exc:
        handles["stand_mesh_error"] = f"{type(exc).__name__}: {exc}"


def _add_robot_urdfs(server: Any, handles: dict[str, Any]) -> None:
    urdf_path = _robot_urdf_path()
    if not urdf_path.exists():
        handles["urdf_error"] = f"robot URDF not found: {urdf_path}"
        return
    try:
        from viser.extras import ViserUrdf

        handles["left_urdf"] = ViserUrdf(server, urdf_path, root_node_name="/stand/left_base")
        handles["right_urdf"] = ViserUrdf(server, urdf_path, root_node_name="/stand/right_base")
        handles["urdf_joint_names"] = tuple(handles["left_urdf"].get_actuated_joint_names())
    except Exception as exc:
        handles["urdf_error"] = f"{type(exc).__name__}: {exc}"


def _update_urdf_config(urdf_handle: Any, cfg_radians: tuple[float, ...]) -> None:
    try:
        import numpy as np

        payload: Any = np.array(cfg_radians)
    except Exception:
        payload = cfg_radians
    urdf_handle.update_cfg(payload)


def _add_scene_fallback(server: Any) -> dict[str, Any]:
    """Add stand/base frames, URDF assets, and marker fallback for degraded mode."""
    handles: dict[str, Any] = {}
    try:
        handles["stand"] = server.scene.add_frame("/stand", show_axes=False)
        handles["left_base"] = server.scene.add_frame("/stand/left_base", wxyz=_pose_wxyz(_DEFAULT_LEFT_POSE), position=_pose_position(_DEFAULT_LEFT_POSE), show_axes=False)
        handles["right_base"] = server.scene.add_frame("/stand/right_base", wxyz=_pose_wxyz(_DEFAULT_RIGHT_POSE), position=_pose_position(_DEFAULT_RIGHT_POSE), show_axes=False)
        has_transform_controls = hasattr(server.scene, "add_transform_controls")
        handles["left_tcp"] = server.scene.add_frame("/stand/left_tcp", show_axes=not has_transform_controls, axes_length=0.08, axes_radius=0.003, position=(0.1601, -0.1725, 0.78))
        handles["right_tcp"] = server.scene.add_frame("/stand/right_tcp", show_axes=not has_transform_controls, axes_length=0.08, axes_radius=0.003, position=(-0.1601, -0.1725, 0.78))
        if has_transform_controls:
            handles["left_tcp_target"] = server.scene.add_transform_controls(
                "/stand/left_tcp_target", scale=0.16, line_width=3.0, position=(0.1601, -0.1725, 0.78)
            )
            handles["right_tcp_target"] = server.scene.add_transform_controls(
                "/stand/right_tcp_target", scale=0.16, line_width=3.0, position=(-0.1601, -0.1725, 0.78)
            )
        _add_stand_mesh(server, handles)
        _add_robot_urdfs(server, handles)
        urdf_loaded = "left_urdf" in handles and "right_urdf" in handles
        if urdf_loaded:
            return handles
        if hasattr(server.scene, "add_icosphere"):
            handles["left_marker"] = server.scene.add_icosphere("/stand/left_state_marker", radius=0.025, color=(80, 160, 255), position=(0.1601, -0.1725, 0.68))
            handles["right_marker"] = server.scene.add_icosphere("/stand/right_state_marker", radius=0.025, color=(255, 160, 80), position=(-0.1601, -0.1725, 0.68))
        elif hasattr(server.scene, "add_point_cloud"):
            handles["left_marker"] = server.scene.add_point_cloud("/stand/left_state_marker", points=((0.1601, -0.1725, 0.68),), colors=((80, 160, 255),), point_size=0.04)
            handles["right_marker"] = server.scene.add_point_cloud("/stand/right_state_marker", points=((-0.1601, -0.1725, 0.68),), colors=((255, 160, 80),), point_size=0.04)
    except Exception as exc:
        handles["scene_error"] = str(exc)
    return handles


def update_scene_markers(scene_handles: dict[str, Any], latest: Any) -> None:
    mounts = latest.mounts if isinstance(latest.mounts, dict) else {}
    left_pose = _mount_pose_from_mounts(mounts, "left", _DEFAULT_LEFT_POSE)
    right_pose = _mount_pose_from_mounts(mounts, "right", _DEFAULT_RIGHT_POSE)
    left_base = _pose_position(left_pose)
    right_base = _pose_position(right_pose)

    for key, arm_state in (("left_urdf", latest.left), ("right_urdf", latest.right)):
        urdf_handle = scene_handles.get(key)
        if urdf_handle is None:
            continue
        try:
            _update_urdf_config(urdf_handle, _joint_cfg_radians(arm_state.q_actual_deg))
        except Exception as exc:
            scene_handles["urdf_update_error"] = f"{type(exc).__name__}: {exc}"

    updates = {
        "left_base": left_base,
        "right_base": right_base,
        "left_marker": _joint_marker_position(left_base, latest.left.q_actual_deg),
        "right_marker": _joint_marker_position(right_base, latest.right.q_actual_deg),
    }
    tcp_updates: dict[str, tuple[tuple[float, float, float], tuple[float, float, float, float]]] = {}
    for arm, arm_state in (("left", latest.left), ("right", latest.right)):
        tcp_pose = arm_state.tcp_stand
        if arm_state.has_valid_tcp_pose and tcp_pose is not None:
            tcp_updates[arm] = (_pose_position(tcp_pose), _pose_orientation_wxyz(tcp_pose))
            continue
        for key in (f"{arm}_tcp", f"{arm}_tcp_target"):
            handle = scene_handles.get(key)
            if handle is not None:
                try:
                    handle.visible = False
                except Exception:
                    pass

    for arm, (position, wxyz) in tcp_updates.items():
        updates[f"{arm}_tcp"] = position
        target_key = f"{arm}_tcp_target"
        if target_key not in scene_handles or f"{arm}_tcp_target_user_moved" not in scene_handles:
            updates[target_key] = position
            scene_handles[f"{arm}_tcp_target_pose"] = _pose6_from_transform(position, wxyz)
            scene_handles[f"{arm}_tcp_target_wxyz"] = _normalize_wxyz(wxyz)
        for key in (f"{arm}_tcp", f"{arm}_tcp_target"):
            handle = scene_handles.get(key)
            if handle is not None:
                try:
                    handle.visible = True
                except Exception:
                    pass

    for key, position in updates.items():
        handle = scene_handles.get(key)
        if handle is None:
            continue
        try:
            handle.position = position
        except Exception:
            try:
                handle.points = (position,)
            except Exception:
                pass
    rotations = {
        "left_base": _pose_wxyz(left_pose),
        "right_base": _pose_wxyz(right_pose),
    }
    for arm, (_, wxyz) in tcp_updates.items():
        rotations[f"{arm}_tcp"] = wxyz
        if f"{arm}_tcp_target_user_moved" not in scene_handles:
            rotations[f"{arm}_tcp_target"] = wxyz
    for key, wxyz in rotations.items():
        handle = scene_handles.get(key)
        if handle is None:
            continue
        try:
            handle.wxyz = wxyz
        except Exception:
            pass


def _install_tcp_target_callbacks(scene_handles: dict[str, Any], status_handle: Any | None = None) -> None:
    for arm in ("left", "right"):
        control = scene_handles.get(f"{arm}_tcp_target")
        if control is None or not hasattr(control, "on_update"):
            continue

        @control.on_update
        def _(_: Any, arm: str = arm, control: Any = control) -> None:
            try:
                position = tuple(float(value) for value in control.position)
                wxyz = tuple(float(value) for value in control.wxyz)
                if len(position) != 3 or len(wxyz) != 4:
                    return
                scene_handles[f"{arm}_tcp_target_wxyz"] = _normalize_wxyz(wxyz)  # type: ignore[arg-type]
                scene_handles[f"{arm}_tcp_target_pose"] = _pose6_from_transform(position, scene_handles[f"{arm}_tcp_target_wxyz"])
                scene_handles[f"{arm}_tcp_target_user_moved"] = True
                if status_handle is not None:
                    status_handle.value = f"{arm} TCP target updated"
            except Exception as exc:
                scene_handles["tcp_target_error"] = f"{type(exc).__name__}: {exc}"
