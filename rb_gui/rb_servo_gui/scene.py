from __future__ import annotations

import importlib.util
import math
import os
from pathlib import Path
from typing import Any, Mapping

from .geometry import (
    _mount_pose_from_mounts,
    _normalize_wxyz,
    _pose6_from_transform,
    _pose_orientation_wxyz,
    _pose_position,
    _pose_wxyz,
)
from .models import CircleOverlaySnapshot

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
_TCP_DISPLAY_MODES = ("auto", "actual", "reference", "both")
_TCP_TRAIL_LIMIT = 200
_CIRCLE_OVERLAY_POINT_COUNT = 96
_ASSET_INSTALL_HINT = "Install with python3 -m pip install -e rb_gui"


def _points_array(points: Any = ()) -> Any:
    import numpy as np

    return np.asarray(points, dtype=np.float32).reshape((-1, 3))


def _line_segments_array(points: Any = ()) -> Any:
    import numpy as np

    return np.asarray(points, dtype=np.float32).reshape((-1, 2, 3))


def _colors_array(colors: Any = ()) -> Any:
    import numpy as np

    return np.asarray(colors, dtype=np.uint8).reshape((-1, 3))


def _line_segment_colors_array(colors: Any = ()) -> Any:
    import numpy as np

    return np.asarray(colors, dtype=np.uint8).reshape((-1, 2, 3))


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


def _module_available(module_name: str) -> bool:
    try:
        return importlib.util.find_spec(module_name) is not None
    except (ImportError, ValueError):
        return False


def _asset_error(message: str) -> str:
    return f"{message}; {_ASSET_INSTALL_HINT}"


def scene_asset_status() -> dict[str, Any]:
    urdf_path = _robot_urdf_path()
    stand_mesh_path = _stand_mesh_path()
    viser_urdf_available = False
    viser_urdf_error: str | None = None
    try:
        from viser.extras import ViserUrdf  # noqa: F401

        viser_urdf_available = True
    except Exception as exc:
        viser_urdf_error = f"{type(exc).__name__}: {exc}"
    return {
        "descriptions_dir": str(_descriptions_dir()),
        "descriptions_dir_exists": _descriptions_dir().exists(),
        "robot_urdf_path": str(urdf_path),
        "robot_urdf_exists": urdf_path.exists(),
        "stand_mesh_path": str(stand_mesh_path),
        "stand_mesh_exists": stand_mesh_path.exists(),
        "viser_available": _module_available("viser"),
        "viser_extras_urdf_available": viser_urdf_available,
        "viser_extras_urdf_error": viser_urdf_error,
        "trimesh_available": _module_available("trimesh"),
        "yourdfpy_available": _module_available("yourdfpy"),
        "install_hint": _ASSET_INSTALL_HINT,
    }


def format_scene_asset_startup_status() -> str:
    status = scene_asset_status()
    lines = [
        "rb_servo_gui asset status:",
        f"  descriptions_dir: {status['descriptions_dir']} exists={status['descriptions_dir_exists']}",
        f"  robot_urdf_path: {status['robot_urdf_path']} exists={status['robot_urdf_exists']}",
        f"  stand_mesh_path: {status['stand_mesh_path']} exists={status['stand_mesh_exists']}",
        (
            "  dependencies: "
            f"viser={status['viser_available']} "
            f"viser.extras.ViserUrdf={status['viser_extras_urdf_available']} "
            f"trimesh={status['trimesh_available']} "
            f"yourdfpy={status['yourdfpy_available']}"
        ),
    ]
    if status["viser_extras_urdf_error"]:
        lines.append(f"  viser_extras_urdf_error: {status['viser_extras_urdf_error']}")
    missing = [
        not status["robot_urdf_exists"],
        not status["stand_mesh_exists"],
        not status["viser_available"],
        not status["viser_extras_urdf_available"],
        not status["trimesh_available"],
        not status["yourdfpy_available"],
    ]
    if any(missing):
        lines.append(f"  hint: {status['install_hint']}")
    return "\n".join(lines)


def _joint_cfg_radians(q_values: tuple[float, ...] | None) -> tuple[float, ...]:
    if q_values is None:
        return tuple(0.0 for _ in _ROBOT_JOINT_NAMES)
    padded = tuple(float(q_values[index]) if index < len(q_values) else 0.0 for index in range(len(_ROBOT_JOINT_NAMES)))
    return tuple(math.radians(value) for value in padded)


# Translucent blue for the reference "ghost" robot (R, G, B, alpha in 0..1).
_REFERENCE_GHOST_RGBA = (0.25, 0.6, 1.0, 0.35)

# Opaque red for the self-collision overlay (shown while self_collision.violated,
# monitor_only included).
_SELF_COLLISION_RGBA = (0.85, 0.08, 0.08, 1.0)
_SELF_COLLISION_STAND_RGBA = (217, 20, 20, 255)


def _reference_ghost_enabled() -> bool:
    return os.environ.get("RB_GUI_REFERENCE_GHOST", "1").strip().lower() not in ("0", "false", "no", "off")


def _reference_ghost_active(arm_state: Any) -> bool:
    """Show the commanded ghost in simulation contexts where q_actual need not track
    the command (controller/pgmode simulation). Driven by q_sent (the clean commanded
    joints), not the controller's noisy jnt_ref. Hidden for physical real motion."""
    if not _reference_ghost_enabled():
        return False
    if getattr(arm_state, "q_sent_deg", None) is None:
        return False
    if getattr(arm_state, "physical_motion_expected", None) is True:
        return False
    return True


def _joint_marker_position(base: tuple[float, float, float], q_values: tuple[float, ...] | None) -> tuple[float, float, float]:
    # Marker-only fallback: not FK. It gives operators visible left/right state
    # changes without pretending Cartesian kinematics are available.
    if q_values is None:
        return (base[0], base[1], base[2] + 0.04)
    shoulder = q_values[0] / 180.0 if q_values else 0.0
    elbow = q_values[1] / 180.0 if len(q_values) > 1 else 0.0
    wrist = q_values[2] / 180.0 if len(q_values) > 2 else 0.0
    return (base[0] + 0.08 * shoulder, base[1] + 0.08 * elbow, base[2] + 0.04 + 0.06 * wrist)


def _tcp_label_position(position: tuple[float, float, float]) -> tuple[float, float, float]:
    return (position[0], position[1], position[2] + 0.045)


_FLOOR_PLANE_BLUE = (80, 160, 255)
_FLOOR_PLANE_RED = (220, 60, 60)


def _add_floor_plane(server: Any, handles: dict[str, Any]) -> None:
    """Stand-frame safety floor plane (safety.floor_constraint visual).

    Rendered at the server-reported effective z; red when either arm violates.
    Hidden until the server reports the constraint enabled."""
    try:
        if hasattr(server.scene, "add_box"):
            try:
                handles["floor_plane"] = server.scene.add_box(
                    "/stand/floor_plane",
                    dimensions=(1.2, 1.2, 0.002),
                    color=_FLOOR_PLANE_BLUE,
                    opacity=0.25,
                    position=(0.0, 0.0, 0.0),
                    visible=False,
                )
            except TypeError:  # older viser without opacity support
                handles["floor_plane"] = server.scene.add_box(
                    "/stand/floor_plane",
                    dimensions=(1.2, 1.2, 0.002),
                    color=_FLOOR_PLANE_BLUE,
                    position=(0.0, 0.0, 0.0),
                    visible=False,
                )
        elif hasattr(server.scene, "add_grid"):
            handles["floor_plane"] = server.scene.add_grid(
                "/stand/floor_plane",
                width=1.2,
                height=1.2,
                position=(0.0, 0.0, 0.0),
                visible=False,
            )
    except Exception as exc:
        handles["floor_plane_error"] = f"{type(exc).__name__}: {exc}"


def update_floor_plane(scene_handles: dict[str, Any], floor: Mapping[str, Any] | None) -> None:
    """Move/recolor the safety floor plane from the published floor_constraint block."""
    plane = scene_handles.get("floor_plane") if isinstance(scene_handles, dict) else None
    if plane is None:
        return
    if not isinstance(floor, Mapping) or not bool(floor.get("enabled", False)):
        _set_visible(plane, False)
        return
    z = floor.get("z_min_m")
    if isinstance(z, (int, float)) and math.isfinite(float(z)):
        try:
            plane.position = (0.0, 0.0, float(z))
        except Exception:
            pass
    violated = any(
        isinstance(floor.get(key), Mapping) and bool(floor[key].get("violated", False))
        for key in ("left", "right")
    )
    try:
        plane.color = _FLOOR_PLANE_RED if violated else _FLOOR_PLANE_BLUE
    except Exception:
        pass
    _set_visible(plane, True)


def update_self_collision_overlay(scene_handles: dict[str, Any], latest: Any) -> None:
    """Paint the colliding model opaque red while self_collision.violated.

    Driven by the server's self_collision telemetry, so monitor_only runs show
    the overlay too. pgmode real (physical_motion_expected=True): the ACTUAL
    robot (q_actual) and the stand turn red. pgmode simulation: the commanded
    ghost (q_sent) and the stand turn red while the solid robot keeps showing
    the true (stationary) state."""
    if not isinstance(scene_handles, dict):
        return
    sc = getattr(latest, "self_collision", None) if latest is not None else None
    violated = isinstance(sc, Mapping) and bool(sc.get("violated", False))
    physical_real = latest is not None and (
        getattr(latest.left, "physical_motion_expected", None) is True
        or getattr(latest.right, "physical_motion_expected", None) is True
    )

    if violated:
        for key, arm_state in (
            ("left_urdf_collision", latest.left),
            ("right_urdf_collision", latest.right),
        ):
            handle = scene_handles.get(key)
            if handle is None:
                continue
            q = arm_state.q_actual_deg if physical_real else (
                arm_state.q_sent_deg if arm_state.q_sent_deg is not None else arm_state.q_actual_deg
            )
            try:
                _update_urdf_config(handle, _joint_cfg_radians(q))
            except Exception as exc:
                scene_handles["urdf_collision_update_error"] = f"{type(exc).__name__}: {exc}"

    _set_visible(scene_handles.get("left_base_collision"), violated)
    _set_visible(scene_handles.get("right_base_collision"), violated)
    _set_visible(scene_handles.get("stand_mesh_collision"), violated)
    _set_visible(scene_handles.get("stand_mesh"), not violated)
    # Replace (not overlap) the model the red overlay represents to avoid
    # z-fighting at the identical configuration.
    hide_solid = violated and physical_real
    _set_visible(scene_handles.get("left_base"), not hide_solid)
    _set_visible(scene_handles.get("right_base"), not hide_solid)
    if violated and not physical_real:
        _set_visible(scene_handles.get("left_base_ref"), False)
        _set_visible(scene_handles.get("right_base_ref"), False)


def _add_stand_mesh(server: Any, handles: dict[str, Any]) -> None:
    stand_mesh_path = _stand_mesh_path()
    if not stand_mesh_path.exists():
        handles["stand_mesh_error"] = _asset_error(f"stand mesh not found: {stand_mesh_path}")
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
        # Opaque-red duplicate for the self-collision overlay (hidden until violated).
        try:
            red = mesh.copy()
            red.visual.face_colors = _SELF_COLLISION_STAND_RGBA
            handles["stand_mesh_collision"] = server.scene.add_mesh_trimesh(
                "/stand/mesh_collision",
                mesh=red,
                position=_pose_position(_DEFAULT_STAND_MESH_POSE),
                wxyz=_pose_wxyz(_DEFAULT_STAND_MESH_POSE),
                visible=False,
            )
        except Exception as exc:
            handles["stand_mesh_collision_error"] = _asset_error(f"{type(exc).__name__}: {exc}")
    except Exception as exc:
        handles["stand_mesh_error"] = _asset_error(f"{type(exc).__name__}: {exc}")


def _add_robot_urdfs(server: Any, handles: dict[str, Any]) -> None:
    urdf_path = _robot_urdf_path()
    if not urdf_path.exists():
        handles["urdf_error"] = _asset_error(f"robot URDF not found: {urdf_path}")
        return
    try:
        from viser.extras import ViserUrdf

        handles["left_urdf"] = ViserUrdf(server, urdf_path, root_node_name="/stand/left_base")
        handles["right_urdf"] = ViserUrdf(server, urdf_path, root_node_name="/stand/right_base")
        handles["urdf_joint_names"] = tuple(handles["left_urdf"].get_actuated_joint_names())
        # Translucent reference "ghost" robot following q_ref. In controller (pgmode)
        # simulation the controller does not move q_actual to track streamed servo_j,
        # so this overlay shows the commanded motion while the solid robot truthfully
        # stays at q_actual. RGBA mesh_color_override -> add_mesh_simple(opacity=a).
        if _reference_ghost_enabled():
            try:
                handles["left_urdf_ref"] = ViserUrdf(
                    server, urdf_path, root_node_name="/stand/left_base_ref",
                    mesh_color_override=_REFERENCE_GHOST_RGBA)
                handles["right_urdf_ref"] = ViserUrdf(
                    server, urdf_path, root_node_name="/stand/right_base_ref",
                    mesh_color_override=_REFERENCE_GHOST_RGBA)
            except Exception as exc:
                handles["urdf_ref_error"] = f"{type(exc).__name__}: {exc}"
        # Opaque-red self-collision overlay robots (hidden until violated). In
        # pgmode real they replace the actual robot; in pgmode simulation they
        # replace the commanded ghost.
        try:
            handles["left_urdf_collision"] = ViserUrdf(
                server, urdf_path, root_node_name="/stand/left_base_collision",
                mesh_color_override=_SELF_COLLISION_RGBA)
            handles["right_urdf_collision"] = ViserUrdf(
                server, urdf_path, root_node_name="/stand/right_base_collision",
                mesh_color_override=_SELF_COLLISION_RGBA)
        except Exception as exc:
            handles["urdf_collision_error"] = f"{type(exc).__name__}: {exc}"
    except Exception as exc:
        handles["urdf_error"] = _asset_error(f"{type(exc).__name__}: {exc}")


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
        # Mount frames for the translucent reference "ghost" robot (follows q_ref).
        handles["left_base_ref"] = server.scene.add_frame("/stand/left_base_ref", wxyz=_pose_wxyz(_DEFAULT_LEFT_POSE), position=_pose_position(_DEFAULT_LEFT_POSE), show_axes=False, visible=False)
        handles["right_base_ref"] = server.scene.add_frame("/stand/right_base_ref", wxyz=_pose_wxyz(_DEFAULT_RIGHT_POSE), position=_pose_position(_DEFAULT_RIGHT_POSE), show_axes=False, visible=False)
        # Mount frames for the opaque-red self-collision overlay robots.
        handles["left_base_collision"] = server.scene.add_frame("/stand/left_base_collision", wxyz=_pose_wxyz(_DEFAULT_LEFT_POSE), position=_pose_position(_DEFAULT_LEFT_POSE), show_axes=False, visible=False)
        handles["right_base_collision"] = server.scene.add_frame("/stand/right_base_collision", wxyz=_pose_wxyz(_DEFAULT_RIGHT_POSE), position=_pose_position(_DEFAULT_RIGHT_POSE), show_axes=False, visible=False)
        has_transform_controls = hasattr(server.scene, "add_transform_controls")
        handles["left_tcp"] = server.scene.add_frame("/stand/left_tcp", show_axes=not has_transform_controls, axes_length=0.08, axes_radius=0.003, position=(0.1601, -0.1725, 0.78))
        handles["right_tcp"] = server.scene.add_frame("/stand/right_tcp", show_axes=not has_transform_controls, axes_length=0.08, axes_radius=0.003, position=(-0.1601, -0.1725, 0.78))
        handles["left_tcp_ref"] = server.scene.add_frame("/stand/left_tcp_ref", show_axes=True, axes_length=0.055, axes_radius=0.002, position=(0.1601, -0.1725, 0.78))
        handles["right_tcp_ref"] = server.scene.add_frame("/stand/right_tcp_ref", show_axes=True, axes_length=0.055, axes_radius=0.002, position=(-0.1601, -0.1725, 0.78))
        if hasattr(server.scene, "add_label"):
            for arm, position in (
                ("left", (0.1601, -0.1725, 0.78)),
                ("right", (-0.1601, -0.1725, 0.78)),
            ):
                handles[f"{arm}_tcp_label"] = server.scene.add_label(
                    f"/stand/{arm}_tcp_label",
                    f"{arm} tcp_actual_stand physical-state inspection",
                    position=_tcp_label_position(position),
                    visible=False,
                )
                handles[f"{arm}_tcp_ref_label"] = server.scene.add_label(
                    f"/stand/{arm}_tcp_ref_label",
                    f"{arm} tcp_ref_stand controller-sim reference",
                    position=_tcp_label_position(position),
                    visible=False,
                )
        if has_transform_controls:
            handles["left_tcp_target"] = server.scene.add_transform_controls(
                "/stand/left_tcp_target", scale=0.16, line_width=3.0, position=(0.1601, -0.1725, 0.78)
            )
            handles["right_tcp_target"] = server.scene.add_transform_controls(
                "/stand/right_tcp_target", scale=0.16, line_width=3.0, position=(-0.1601, -0.1725, 0.78)
            )
        if hasattr(server.scene, "add_point_cloud"):
            for arm, actual_color, ref_color in (
                ("left", (80, 160, 255), (60, 210, 110)),
                ("right", (255, 160, 80), (60, 210, 110)),
            ):
                handles[f"{arm}_tcp_trail_points"] = []
                handles[f"{arm}_tcp_ref_trail_points"] = []
                handles[f"{arm}_tcp_trail"] = server.scene.add_point_cloud(
                    f"/stand/{arm}_tcp_trail",
                    points=_points_array(),
                    colors=_colors_array(),
                    point_size=0.012,
                )
                handles[f"{arm}_tcp_ref_trail"] = server.scene.add_point_cloud(
                    f"/stand/{arm}_tcp_ref_trail",
                    points=_points_array(),
                    colors=_colors_array(),
                    point_size=0.012,
                )
                handles[f"{arm}_tcp_trail_color"] = actual_color
                handles[f"{arm}_tcp_ref_trail_color"] = ref_color
        if hasattr(server.scene, "add_line_segments"):
            handles["circle_overlay_line_mode"] = "line_segments"
            handles["circle_overlay_line"] = server.scene.add_line_segments(
                "/stand/circle_overlay",
                points=_line_segments_array(),
                colors=_line_segment_colors_array(),
                line_width=2.0,
            )
        elif hasattr(server.scene, "add_point_cloud"):
            handles["circle_overlay_line_mode"] = "point_cloud"
            handles["circle_overlay_line"] = server.scene.add_point_cloud(
                "/stand/circle_overlay",
                points=_points_array(),
                colors=_colors_array(),
                point_size=0.008,
            )
        if hasattr(server.scene, "add_icosphere"):
            handles["circle_overlay_desired"] = server.scene.add_icosphere(
                "/stand/circle_overlay_desired",
                radius=0.012,
                color=(230, 40, 40),
                position=(0.0, 0.0, 0.0),
            )
        elif hasattr(server.scene, "add_point_cloud"):
            handles["circle_overlay_desired"] = server.scene.add_point_cloud(
                "/stand/circle_overlay_desired",
                points=_points_array(),
                colors=_colors_array(),
                point_size=0.02,
            )
        _set_visible(handles.get("circle_overlay_line"), False)
        _set_visible(handles.get("circle_overlay_desired"), False)
        _add_floor_plane(server, handles)
        _add_stand_mesh(server, handles)
        _add_robot_urdfs(server, handles)
        urdf_loaded = "left_urdf" in handles and "right_urdf" in handles
        if urdf_loaded:
            return handles
        if hasattr(server.scene, "add_icosphere"):
            handles["left_marker"] = server.scene.add_icosphere(
                "/stand/left_state_marker",
                radius=0.025,
                color=(80, 160, 255),
                position=(0.1601, -0.1725, 0.68),
            )
            handles["right_marker"] = server.scene.add_icosphere(
                "/stand/right_state_marker",
                radius=0.025,
                color=(255, 160, 80),
                position=(-0.1601, -0.1725, 0.68),
            )
        elif hasattr(server.scene, "add_point_cloud"):
            handles["left_marker"] = server.scene.add_point_cloud(
                "/stand/left_state_marker",
                points=_points_array(((0.1601, -0.1725, 0.68),)),
                colors=_colors_array(((80, 160, 255),)),
                point_size=0.04,
            )
            handles["right_marker"] = server.scene.add_point_cloud(
                "/stand/right_state_marker",
                points=_points_array(((-0.1601, -0.1725, 0.68),)),
                colors=_colors_array(((255, 160, 80),)),
                point_size=0.04,
            )
    except Exception as exc:
        handles["scene_error"] = str(exc)
    return handles


def _tcp_display_mode(scene_handles: dict[str, Any], requested: str | None) -> str:
    mode = requested if requested is not None else scene_handles.get("tcp_display_mode", "auto")
    return mode if mode in _TCP_DISPLAY_MODES else "auto"


def _set_visible(handle: Any, visible: bool) -> None:
    if handle is None:
        return
    try:
        handle.visible = visible
    except Exception:
        pass


def _hide_tcp_handles(scene_handles: dict[str, Any], arm: str, *, include_target: bool = False) -> None:
    keys = [
        f"{arm}_tcp",
        f"{arm}_tcp_ref",
        f"{arm}_tcp_trail",
        f"{arm}_tcp_ref_trail",
        f"{arm}_tcp_label",
        f"{arm}_tcp_ref_label",
    ]
    if include_target:
        keys.append(f"{arm}_tcp_target")
    for key in keys:
        _set_visible(scene_handles.get(key), False)


def _update_tcp_trail(
    scene_handles: dict[str, Any],
    key_prefix: str,
    position: tuple[float, float, float],
    *,
    visible: bool,
) -> None:
    handle = scene_handles.get(f"{key_prefix}_trail")
    points = scene_handles.get(f"{key_prefix}_trail_points")
    if handle is None or not isinstance(points, list):
        return
    if not points or points[-1] != position:
        points.append(position)
        del points[:-_TCP_TRAIL_LIMIT]
    color = scene_handles.get(f"{key_prefix}_trail_color", (160, 160, 160))
    try:
        handle.points = _points_array(tuple(points))
        handle.colors = _colors_array(tuple(color for _ in points))
        handle.visible = visible
    except Exception:
        pass


def _circle_overlay_axes(overlay: CircleOverlaySnapshot) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    if overlay.axis1_stand is not None and overlay.axis2_stand is not None:
        return overlay.axis1_stand, overlay.axis2_stand
    if overlay.plane == "xz":
        return (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)
    if overlay.plane == "yz":
        return (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)
    return (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)


def _circle_overlay_points(
    overlay: CircleOverlaySnapshot,
    *,
    segments: int = _CIRCLE_OVERLAY_POINT_COUNT,
) -> tuple[tuple[float, float, float], ...]:
    count = max(8, int(segments))
    axis1, axis2 = _circle_overlay_axes(overlay)
    center = overlay.center_stand
    radius = overlay.radius_m
    points: list[tuple[float, float, float]] = []
    for index in range(count + 1):
        theta = 2.0 * math.pi * index / count
        c = math.cos(theta)
        s = math.sin(theta)
        points.append(
            (
                center[0] + radius * (c * axis1[0] + s * axis2[0]),
                center[1] + radius * (c * axis1[1] + s * axis2[1]),
                center[2] + radius * (c * axis1[2] + s * axis2[2]),
            )
        )
    return tuple(points)


def _circle_overlay_line_segments(
    points: tuple[tuple[float, float, float], ...],
) -> tuple[tuple[tuple[float, float, float], tuple[float, float, float]], ...]:
    return tuple((points[index], points[index + 1]) for index in range(len(points) - 1))


def update_circle_overlay(scene_handles: dict[str, Any], overlay: CircleOverlaySnapshot | None, *, stale: bool = False) -> None:
    line = scene_handles.get("circle_overlay_line")
    desired = scene_handles.get("circle_overlay_desired")
    if overlay is None or stale:
        _set_visible(line, False)
        _set_visible(desired, False)
        return
    points = _circle_overlay_points(overlay)
    line_color = (230, 40, 40)
    try:
        if scene_handles.get("circle_overlay_line_mode") == "line_segments":
            segments = _circle_overlay_line_segments(points)
            line.points = _line_segments_array(segments)
            line.colors = _line_segment_colors_array(
                tuple((line_color, line_color) for _ in segments)
            )
        elif line is not None:
            line.points = _points_array(points)
            line.colors = _colors_array(tuple(line_color for _ in points))
        _set_visible(line, True)
    except Exception:
        pass
    position = _pose_position(overlay.desired_pose_stand)
    wxyz = _pose_orientation_wxyz(overlay.desired_pose_stand)
    try:
        desired.position = position
    except Exception:
        try:
            desired.points = _points_array((position,))
            desired.colors = _colors_array(((230, 40, 40),))
        except Exception:
            pass
    try:
        desired.wxyz = wxyz
    except Exception:
        pass
    _set_visible(desired, True)


def _arm_is_controller_sim(arm_state: Any) -> bool:
    """True for rbpodo controller (pgmode) simulation (operation_mode: simulation)."""
    csm = getattr(arm_state, "controller_simulation_mode", None)
    if not csm:
        return False
    try:
        return str(csm.get("operation_mode", "")).lower() in ("simulation", "sim")
    except AttributeError:
        return False


def update_scene_markers(scene_handles: dict[str, Any], latest: Any, *, tcp_display_mode: str | None = None) -> None:
    mounts = latest.mounts if isinstance(latest.mounts, dict) else {}
    left_pose = _mount_pose_from_mounts(mounts, "left", _DEFAULT_LEFT_POSE)
    right_pose = _mount_pose_from_mounts(mounts, "right", _DEFAULT_RIGHT_POSE)
    left_base = _pose_position(left_pose)
    right_base = _pose_position(right_pose)
    display_mode = _tcp_display_mode(scene_handles, tcp_display_mode)

    for key, arm_state in (("left_urdf", latest.left), ("right_urdf", latest.right)):
        urdf_handle = scene_handles.get(key)
        if urdf_handle is None:
            continue
        try:
            _update_urdf_config(urdf_handle, _joint_cfg_radians(arm_state.q_actual_deg))
        except Exception as exc:
            scene_handles["urdf_update_error"] = f"{type(exc).__name__}: {exc}"

    # Reference ghost robot: drive q_ref and toggle visibility per arm.
    for key, base_key, arm_state in (
        ("left_urdf_ref", "left_base_ref", latest.left),
        ("right_urdf_ref", "right_base_ref", latest.right),
    ):
        ghost = scene_handles.get(key)
        if ghost is None:
            continue
        show_ghost = _reference_ghost_active(arm_state)
        if show_ghost:
            try:
                _update_urdf_config(ghost, _joint_cfg_radians(arm_state.q_sent_deg))
            except Exception as exc:
                scene_handles["urdf_ref_update_error"] = f"{type(exc).__name__}: {exc}"
        _set_visible(scene_handles.get(base_key), show_ghost)

    updates = {
        "left_base": left_base,
        "right_base": right_base,
        "left_base_ref": left_base,
        "right_base_ref": right_base,
        "left_base_collision": left_base,
        "right_base_collision": right_base,
        "left_marker": _joint_marker_position(left_base, latest.left.q_actual_deg),
        "right_marker": _joint_marker_position(right_base, latest.right.q_actual_deg),
    }
    actual_updates: dict[str, tuple[tuple[float, float, float], tuple[float, float, float, float]]] = {}
    ref_updates: dict[str, tuple[tuple[float, float, float], tuple[float, float, float, float]]] = {}
    for arm, arm_state in (("left", latest.left), ("right", latest.right)):
        actual_pose = arm_state.tcp_actual_stand or arm_state.tcp_stand
        if arm_state.tcp_actual_valid and actual_pose is not None and not arm_state.tcp_deferred:
            actual_updates[arm] = (_pose_position(actual_pose), _pose_orientation_wxyz(actual_pose))
        # Controller-sim reference gizmo: prefer the commanded TCP (FK of q_sent),
        # which is stable at rest, over the noisy jnt_ref-derived tcp_ref_stand.
        if _arm_is_controller_sim(arm_state) and arm_state.tcp_command_stand is not None:
            ref_pose = arm_state.tcp_command_stand
            ref_valid = True
        else:
            ref_pose = arm_state.tcp_ref_stand
            ref_valid = bool(arm_state.tcp_ref_valid and ref_pose is not None)
        if ref_valid:
            ref_updates[arm] = (_pose_position(ref_pose), _pose_orientation_wxyz(ref_pose))
        if arm not in actual_updates and arm not in ref_updates:
            _hide_tcp_handles(scene_handles, arm, include_target=True)

    for arm, (position, wxyz) in actual_updates.items():
        updates[f"{arm}_tcp"] = position
        updates[f"{arm}_tcp_label"] = _tcp_label_position(position)
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
        _update_tcp_trail(
            scene_handles,
            f"{arm}_tcp",
            position,
            visible=display_mode in {"actual", "both"} or (
                display_mode == "auto" and getattr(latest, arm).selected_tcp_source("auto") != "tcp_ref_stand"
            ),
        )

    for arm, (position, wxyz) in ref_updates.items():
        updates[f"{arm}_tcp_ref"] = position
        updates[f"{arm}_tcp_ref_label"] = _tcp_label_position(position)
        _set_visible(
            scene_handles.get(f"{arm}_tcp_ref"),
            display_mode in {"reference", "both"} or (
                display_mode == "auto" and getattr(latest, arm).selected_tcp_source("auto") == "tcp_ref_stand"
            ),
        )
        _update_tcp_trail(
            scene_handles,
            f"{arm}_tcp_ref",
            position,
            visible=display_mode in {"reference", "both"} or (
                display_mode == "auto" and getattr(latest, arm).selected_tcp_source("auto") == "tcp_ref_stand"
            ),
        )

    for arm, arm_state in (("left", latest.left), ("right", latest.right)):
        selected = arm_state.selected_tcp_source("auto")
        actual_visible = arm in actual_updates and (
            display_mode in {"actual", "both"} or (display_mode == "auto" and selected != "tcp_ref_stand")
        )
        ref_visible = arm in ref_updates and (
            display_mode in {"reference", "both"} or (display_mode == "auto" and selected == "tcp_ref_stand")
        )
        _set_visible(scene_handles.get(f"{arm}_tcp"), actual_visible)
        _set_visible(scene_handles.get(f"{arm}_tcp_ref"), ref_visible)
        _set_visible(scene_handles.get(f"{arm}_tcp_label"), actual_visible)
        _set_visible(scene_handles.get(f"{arm}_tcp_ref_label"), ref_visible)
        if arm not in ref_updates:
            _set_visible(scene_handles.get(f"{arm}_tcp_ref"), False)
            _set_visible(scene_handles.get(f"{arm}_tcp_ref_trail"), False)
            _set_visible(scene_handles.get(f"{arm}_tcp_ref_label"), False)
        if arm not in actual_updates:
            _set_visible(scene_handles.get(f"{arm}_tcp"), False)
            _set_visible(scene_handles.get(f"{arm}_tcp_trail"), False)
            _set_visible(scene_handles.get(f"{arm}_tcp_label"), False)
            _set_visible(scene_handles.get(f"{arm}_tcp_target"), False)

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
        "left_base_ref": _pose_wxyz(left_pose),
        "right_base_ref": _pose_wxyz(right_pose),
    }
    for arm, (_, wxyz) in actual_updates.items():
        rotations[f"{arm}_tcp"] = wxyz
        if f"{arm}_tcp_target_user_moved" not in scene_handles:
            rotations[f"{arm}_tcp_target"] = wxyz
    for arm, (_, wxyz) in ref_updates.items():
        rotations[f"{arm}_tcp_ref"] = wxyz
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
