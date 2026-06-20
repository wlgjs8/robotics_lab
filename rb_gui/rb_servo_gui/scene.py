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

# Translucent red for the self-collision overlay (shown while
# self_collision.violated, monitor_only included). The RGBA 4-tuple routes the
# URDF override through add_mesh_simple(opacity=a) — the same transparent
# pipeline as the reference ghost — so the colliding pair reads as a red
# see-through highlight instead of a solid replacement.
_SELF_COLLISION_RGBA = (0.85, 0.08, 0.08, 0.6)
_SELF_COLLISION_STAND_RGB = (217, 20, 20)
_SELF_COLLISION_STAND_OPACITY = 0.6
# Translucent blue overlay of the checked collision hulls (URDF <collision> meshes).
_SELF_COLLISION_CHECK_RGBA = (0.27, 0.62, 1.0, 0.38)


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
# Pending (not-yet-sent) slider value preview: distinct color so it cannot be
# mistaken for the APPLIED safety plane.
_FLOOR_PLANE_PREVIEW_YELLOW = (235, 200, 60)
# Stand-frame footprint of the floor plane visual: x in [-0.5, 0.5] m,
# y in [-1.0, 0.0] m (the workspace in front of the stand).
_FLOOR_PLANE_DIMENSIONS = (1.0, 1.0, 0.002)
_FLOOR_PLANE_CENTER_XY = (0.0, -0.5)


# Reachable-workspace OUTER-SHELL surface (tools/reach_envelope.py output): only
# the farthest reachable points, triangulated into a closed surface and rendered
# translucent + double-sided so the robot and stand stay visible through it. The
# shell's outer face is the reach boundary the safety.reach_constraint damper
# enforces.
_REACH_ENVELOPE_GREEN = (90, 200, 150)
_REACH_ENVELOPE_OPACITY = 0.22


def _reach_envelope_path() -> Path:
    return _asset_path("RB_GUI_REACH_ENVELOPE", "reach_envelope_rb3_730e.npz")


def _add_reachability_cloud(server: Any, handles: dict[str, Any]) -> None:
    """Per-arm reachable-workspace shell mesh (tools/reach_envelope.py output).

    The saved vertices are in the arm-base frame, so attaching the SAME mesh under
    each arm's /stand/<side>_base node renders it correctly placed and mirrored via
    the mount transform (no manual per-arm transform needed). Drawn translucent and
    double-sided so the robot/stand show through. Skips gracefully if the asset is
    missing (run tools/reach_envelope.py to generate it) or viser has no mesh
    support. Static geometry — visibility is toggled from the GUI."""
    if not hasattr(server.scene, "add_mesh_simple"):
        return
    path = _reach_envelope_path()
    if not path.exists():
        handles["reach_envelope_error"] = (
            f"reach envelope not found: {path} (run tools/reach_envelope.py)"
        )
        return
    try:
        import numpy as np

        data = np.load(path)
        verts = np.asarray(data["shell_vertices_base_m"], dtype=np.float32)
        faces = np.asarray(data["shell_faces"], dtype=np.int32)
        if verts.ndim != 2 or verts.shape[1] != 3 or len(verts) == 0 or len(faces) == 0:
            handles["reach_envelope_error"] = f"reach envelope malformed: {path}"
            return
        handles["reach_envelope_r_max_m"] = float(data["r_max_recommended_m"])
        handles["reach_envelope_r_min_m"] = float(data["r_min_recommended_m"])
        for arm in ("left", "right"):
            try:
                handles[f"{arm}_reach_envelope"] = server.scene.add_mesh_simple(
                    f"/stand/{arm}_base/reach_envelope",
                    vertices=verts,
                    faces=faces,
                    color=_REACH_ENVELOPE_GREEN,
                    opacity=_REACH_ENVELOPE_OPACITY,
                    side="double",      # visible from inside the shell too
                    flat_shading=False,
                    visible=False,      # brought up by the GUI "도달영역 표시" toggle
                )
            except TypeError:  # older viser without opacity/side support
                handles[f"{arm}_reach_envelope"] = server.scene.add_mesh_simple(
                    f"/stand/{arm}_base/reach_envelope",
                    vertices=verts,
                    faces=faces,
                    color=_REACH_ENVELOPE_GREEN,
                    visible=False,
                )
    except Exception as exc:
        handles["reach_envelope_error"] = f"{type(exc).__name__}: {exc}"


def set_reach_envelope_visible(scene_handles: dict[str, Any], visible: bool) -> None:
    """Show/hide both arms' reachable-workspace clouds (GUI toggle)."""
    if not isinstance(scene_handles, dict):
        return
    for arm in ("left", "right"):
        _set_visible(scene_handles.get(f"{arm}_reach_envelope"), visible)


def _add_floor_plane(server: Any, handles: dict[str, Any]) -> None:
    """Stand-frame safety floor plane (safety.floor_constraint visual).

    Rendered at the server-reported effective z; red when either arm violates.
    Hidden until the server reports the constraint enabled."""
    try:
        if hasattr(server.scene, "add_box"):
            try:
                handles["floor_plane"] = server.scene.add_box(
                    "/stand/floor_plane",
                    dimensions=_FLOOR_PLANE_DIMENSIONS,
                    color=_FLOOR_PLANE_BLUE,
                    opacity=0.25,
                    position=(*_FLOOR_PLANE_CENTER_XY, 0.0),
                    visible=False,
                )
            except TypeError:  # older viser without opacity support
                handles["floor_plane"] = server.scene.add_box(
                    "/stand/floor_plane",
                    dimensions=_FLOOR_PLANE_DIMENSIONS,
                    color=_FLOOR_PLANE_BLUE,
                    position=(*_FLOOR_PLANE_CENTER_XY, 0.0),
                    visible=False,
                )
        elif hasattr(server.scene, "add_grid"):
            handles["floor_plane"] = server.scene.add_grid(
                "/stand/floor_plane",
                width=_FLOOR_PLANE_DIMENSIONS[0],
                height=_FLOOR_PLANE_DIMENSIONS[1],
                position=(*_FLOOR_PLANE_CENTER_XY, 0.0),
                visible=False,
            )
    except Exception as exc:
        handles["floor_plane_error"] = f"{type(exc).__name__}: {exc}"
    # Pending-slider preview plane (yellow, more translucent): follows the
    # "Set floor z mm" slider live so the operator can line the plane up
    # BEFORE sending; the applied plane above keeps tracking only the
    # server-reported effective z.
    try:
        if hasattr(server.scene, "add_box"):
            try:
                handles["floor_plane_preview"] = server.scene.add_box(
                    "/stand/floor_plane_preview",
                    dimensions=_FLOOR_PLANE_DIMENSIONS,
                    color=_FLOOR_PLANE_PREVIEW_YELLOW,
                    opacity=0.15,
                    position=(*_FLOOR_PLANE_CENTER_XY, 0.0),
                    visible=False,
                )
            except TypeError:  # older viser without opacity support
                handles["floor_plane_preview"] = server.scene.add_box(
                    "/stand/floor_plane_preview",
                    dimensions=_FLOOR_PLANE_DIMENSIONS,
                    color=_FLOOR_PLANE_PREVIEW_YELLOW,
                    position=(*_FLOOR_PLANE_CENTER_XY, 0.0),
                    visible=False,
                )
    except Exception as exc:
        handles["floor_plane_preview_error"] = f"{type(exc).__name__}: {exc}"


def update_floor_plane_preview(scene_handles: dict[str, Any], z_m: float | None) -> None:
    """Show the pending slider value as a translucent preview plane.

    z_m=None hides the preview (slider matches the applied value, or the
    constraint is disabled)."""
    plane = scene_handles.get("floor_plane_preview") if isinstance(scene_handles, dict) else None
    if plane is None:
        return
    if z_m is None or not isinstance(z_m, (int, float)) or not math.isfinite(float(z_m)):
        _set_visible(plane, False)
        return
    try:
        plane.position = (*_FLOOR_PLANE_CENTER_XY, float(z_m))
    except Exception:
        pass
    _set_visible(plane, True)


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
            # z is the stand-frame plane height (z=0 == stand origin plane).
            plane.position = (*_FLOOR_PLANE_CENTER_XY, float(z))
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


# Stand-frame ROI box (safety.roi_box) visual: a translucent box the TCP must
# stay inside. Applied box = blue (red when an arm is outside); pending-slider
# preview = yellow, like the floor plane preview.
_ROI_BOX_BLUE = (40, 110, 245)
_ROI_BOX_RED = (220, 60, 60)
_ROI_BOX_PREVIEW_YELLOW = (235, 200, 60)
# Boundary emphasis: opaque edges + corner vertices drawn over the translucent
# fill so the ROI outline reads clearly even through other geometry. They recolor
# red together with the fill on violation. Brighter than the fill on purpose.
_ROI_BOX_EDGE_BLUE = (150, 200, 255)
_ROI_BOX_EDGE_RED = (255, 120, 120)


def _roi_box_geometry(
    min_m: Any, max_m: Any
) -> tuple[tuple[float, float, float], tuple[float, float, float]] | None:
    """(dimensions, center) for an AABB, or None if bounds are malformed."""
    if (
        not isinstance(min_m, (list, tuple))
        or not isinstance(max_m, (list, tuple))
        or len(min_m) != 3
        or len(max_m) != 3
    ):
        return None
    try:
        lo = [float(v) for v in min_m]
        hi = [float(v) for v in max_m]
    except (TypeError, ValueError):
        return None
    if any(not math.isfinite(v) for v in lo + hi):
        return None
    # Clamp to a positive minimum so a degenerate (zero-width) face still renders.
    dims = tuple(max(1e-3, hi[k] - lo[k]) for k in range(3))
    center = tuple((hi[k] + lo[k]) * 0.5 for k in range(3))
    return dims, center  # type: ignore[return-value]


def _roi_box_outline(dims: tuple[float, ...], center: tuple[float, ...]) -> Any:
    """(edge segments (12,2,3), corner vertices (8,3)) float32 for an AABB.

    Used to draw the ROI box boundary (12 edges) and its 8 corner vertices as
    opaque emphasis over the translucent fill."""
    import numpy as np

    h = [d * 0.5 for d in dims]
    corners = np.array(
        [
            [center[0] + sx * h[0], center[1] + sy * h[1], center[2] + sz * h[2]]
            for sx in (-1.0, 1.0)
            for sy in (-1.0, 1.0)
            for sz in (-1.0, 1.0)
        ],
        dtype=np.float32,
    )

    def _i(sx: int, sy: int, sz: int) -> int:
        # corners are ordered sx-major, then sy, then sz (see comprehension above)
        return (1 if sx > 0 else 0) * 4 + (1 if sy > 0 else 0) * 2 + (1 if sz > 0 else 0)

    pairs = []
    for sy in (-1, 1):  # 4 edges along x
        for sz in (-1, 1):
            pairs.append((_i(-1, sy, sz), _i(1, sy, sz)))
    for sx in (-1, 1):  # 4 edges along y
        for sz in (-1, 1):
            pairs.append((_i(sx, -1, sz), _i(sx, 1, sz)))
    for sx in (-1, 1):  # 4 edges along z
        for sy in (-1, 1):
            pairs.append((_i(sx, sy, -1), _i(sx, sy, 1)))
    segments = np.array([[corners[a], corners[b]] for a, b in pairs], dtype=np.float32)
    return segments, corners


def _apply_roi_box_outline(
    scene_handles: dict[str, Any],
    dims: tuple[float, ...],
    center: tuple[float, ...],
    edge_color: tuple[int, int, int],
) -> None:
    """Resize/recolor the ROI box edge lines + corner vertices to match the fill."""
    import numpy as np

    seg, corners = _roi_box_outline(dims, center)
    col = np.asarray(edge_color, dtype=np.uint8)
    edges = scene_handles.get("roi_box_edges")
    if edges is not None:
        for attr, value in (("points", seg), ("colors", col)):
            try:
                setattr(edges, attr, value)
            except Exception:
                pass
        _set_visible(edges, True)
    verts = scene_handles.get("roi_box_verts")
    if verts is not None:
        for attr, value in (("points", corners), ("colors", col)):
            try:
                setattr(verts, attr, value)
            except Exception:
                pass
        _set_visible(verts, True)


def _hide_roi_box_outline(scene_handles: dict[str, Any]) -> None:
    for key in ("roi_box_edges", "roi_box_verts"):
        _set_visible(scene_handles.get(key), False)


def _add_roi_box(server: Any, handles: dict[str, Any]) -> None:
    """Stand-frame ROI box (safety.roi_box visual): applied box + pending preview.

    Hidden until the server reports the constraint enabled. Both are created with
    a placeholder geometry; update_roi_box / update_roi_box_preview move and
    resize them."""
    if not hasattr(server.scene, "add_box"):
        return
    dims = (1.0, 1.0, 1.0)
    center = (0.0, -0.5, 0.5)
    for key, name, color, opacity in (
        ("roi_box", "/stand/roi_box", _ROI_BOX_BLUE, 0.12),
        ("roi_box_preview", "/stand/roi_box_preview", _ROI_BOX_PREVIEW_YELLOW, 0.10),
    ):
        try:
            try:
                handles[key] = server.scene.add_box(
                    name, dimensions=dims, color=color, opacity=opacity,
                    position=center, visible=False,
                )
            except TypeError:  # older viser without opacity support
                handles[key] = server.scene.add_box(
                    name, dimensions=dims, color=color, position=center, visible=False,
                )
        except Exception as exc:
            handles[f"{key}_error"] = f"{type(exc).__name__}: {exc}"
    # Emphasised boundary for the APPLIED box: opaque edges (line segments) +
    # corner vertices (point cloud). update_roi_box moves/resizes/recolors them
    # alongside the fill; placeholder geometry until then.
    seg, corners = _roi_box_outline(dims, center)
    if hasattr(server.scene, "add_line_segments"):
        try:
            handles["roi_box_edges"] = server.scene.add_line_segments(
                "/stand/roi_box_edges", points=seg, colors=_ROI_BOX_EDGE_BLUE,
                line_width=4.0, visible=False,
            )
        except Exception as exc:
            handles["roi_box_edges_error"] = f"{type(exc).__name__}: {exc}"
    if hasattr(server.scene, "add_point_cloud"):
        try:
            handles["roi_box_verts"] = server.scene.add_point_cloud(
                "/stand/roi_box_verts", points=corners, colors=_ROI_BOX_EDGE_BLUE,
                point_size=0.018, point_shape="circle", visible=False,
            )
        except Exception as exc:
            handles["roi_box_verts_error"] = f"{type(exc).__name__}: {exc}"


def _apply_roi_box(handle: Any, dims: tuple[float, ...], center: tuple[float, ...],
                   color: tuple[int, int, int]) -> None:
    # Set dimensions when the viser handle supports it (live resize); position and
    # color always. Wrapped per-attribute so a viser without settable dimensions
    # still tracks position/color.
    try:
        handle.dimensions = dims
    except Exception:
        pass
    try:
        handle.position = center
    except Exception:
        pass
    try:
        handle.color = color
    except Exception:
        pass


def update_roi_box_preview(
    scene_handles: dict[str, Any], min_m: Any, max_m: Any
) -> None:
    """Show the pending slider bounds as a translucent yellow preview box.

    min_m/max_m=None (or malformed) hides the preview."""
    box = scene_handles.get("roi_box_preview") if isinstance(scene_handles, dict) else None
    if box is None:
        return
    geom = _roi_box_geometry(min_m, max_m) if min_m is not None and max_m is not None else None
    if geom is None:
        _set_visible(box, False)
        return
    dims, center = geom
    _apply_roi_box(box, dims, center, _ROI_BOX_PREVIEW_YELLOW)
    _set_visible(box, True)


def update_roi_box(
    scene_handles: dict[str, Any], roi: Mapping[str, Any] | None, *, visible: bool = True
) -> None:
    """Move/resize/recolor the ROI box from the published roi_box block.

    Drawn whenever `visible` (the GUI "ROI 영역 표시" toggle) is set and the
    published bounds are valid — independent of server enforcement (enabled), so
    the configured region stays a visible reference by default. Red when an arm
    is reported outside the box, teal otherwise."""
    box = scene_handles.get("roi_box") if isinstance(scene_handles, dict) else None
    if box is None:
        return
    if not visible or not isinstance(roi, Mapping):
        _set_visible(box, False)
        _hide_roi_box_outline(scene_handles)
        return
    geom = _roi_box_geometry(roi.get("min_m"), roi.get("max_m"))
    if geom is None:
        _set_visible(box, False)
        _hide_roi_box_outline(scene_handles)
        return
    dims, center = geom
    violated = any(
        isinstance(roi.get(key), Mapping) and bool(roi[key].get("violated", False))
        for key in ("left", "right")
    )
    _apply_roi_box(box, dims, center, _ROI_BOX_RED if violated else _ROI_BOX_BLUE)
    _apply_roi_box_outline(
        scene_handles, dims, center,
        _ROI_BOX_EDGE_RED if violated else _ROI_BOX_EDGE_BLUE,
    )
    _set_visible(box, True)


def update_self_collision_overlay(scene_handles: dict[str, Any], latest: Any) -> None:
    """Paint the colliding PAIR translucent red while self_collision.violated.

    Driven by the server's self_collision telemetry, so monitor_only runs show
    the overlay too. Only the members of the reported pair turn red
    ("left_right" -> both arms, "left_stand"/"right_stand" -> that arm + the
    stand); a violated state without a recognizable pair falls back to all-red
    (conservative). pgmode real (physical_motion_expected=True): the ACTUAL
    robot (q_actual) turns red. pgmode simulation: the commanded ghost (q_sent)
    turns red while the solid robot keeps showing the true (stationary) state."""
    if not isinstance(scene_handles, dict):
        return
    sc = getattr(latest, "self_collision", None) if latest is not None else None
    violated = isinstance(sc, Mapping) and bool(sc.get("violated", False))
    _update_self_collision_witness_markers(scene_handles, sc, violated)
    pair = sc.get("pair") if isinstance(sc, Mapping) else None
    if violated and pair not in ("left_right", "left_stand", "right_stand"):
        pair = "all"  # unknown/legacy pair info: keep the conservative all-red
    left_red = violated and pair in ("left_right", "left_stand", "all")
    right_red = violated and pair in ("left_right", "right_stand", "all")
    stand_red = violated and pair in ("left_stand", "right_stand", "all")
    physical_real = latest is not None and (
        getattr(latest.left, "physical_motion_expected", None) is True
        or getattr(latest.right, "physical_motion_expected", None) is True
    )

    if violated:
        for key, arm_state, arm_red in (
            ("left_urdf_collision", latest.left, left_red),
            ("right_urdf_collision", latest.right, right_red),
        ):
            handle = scene_handles.get(key)
            if handle is None or not arm_red:
                continue
            q = arm_state.q_actual_deg if physical_real else (
                arm_state.q_sent_deg if arm_state.q_sent_deg is not None else arm_state.q_actual_deg
            )
            try:
                _update_urdf_config(handle, _joint_cfg_radians(q))
            except Exception as exc:
                scene_handles["urdf_collision_update_error"] = f"{type(exc).__name__}: {exc}"

    _set_visible(scene_handles.get("left_base_collision"), left_red)
    _set_visible(scene_handles.get("right_base_collision"), right_red)
    _set_visible(scene_handles.get("stand_mesh_collision"), stand_red)
    _set_visible(scene_handles.get("stand_mesh"), not stand_red)
    # Replace (not overlap) the model the red overlay represents to avoid
    # z-fighting at the identical configuration — per arm, only the red one.
    _set_visible(scene_handles.get("left_base"), not (left_red and physical_real))
    _set_visible(scene_handles.get("right_base"), not (right_red and physical_real))
    if violated and not physical_real:
        if left_red:
            _set_visible(scene_handles.get("left_base_ref"), False)
        if right_red:
            _set_visible(scene_handles.get("right_base_ref"), False)


# Frame that maps the monitor's URDF-WORLD-frame self-collision geometry into the
# scene. The unified collision URDF roots at its `world` link, but the URDF carries
# a +90deg-about-Z fixed joint on `world->stand`, and the scene's /stand frame IS
# the URDF `stand` frame (per-arm mounts are defined stand-relative and place the
# solid robots correctly). The async monitor's witness points (coal returns nearest
# points in the pinocchio WORLD frame, published as-is) live in that same URDF-world
# frame. So the collision-hull overlay AND the witness/near-pair markers must hang
# off a frame rotated -90deg about Z to land correctly on /stand.
_SC_WORLD_FRAME = "/stand/sc_urdf_world"


def _ensure_sc_world_frame(scene_handles: dict[str, Any]) -> str:
    server = scene_handles.get("_server")
    if server is None or scene_handles.get("_sc_world_frame_made"):
        return _SC_WORLD_FRAME
    try:
        server.scene.add_frame(
            _SC_WORLD_FRAME,
            wxyz=_pose_wxyz((0.0, 0.0, 0.0, 0.0, 0.0, -math.pi / 2.0)),
            position=(0.0, 0.0, 0.0),
            show_axes=False,
        )
        scene_handles["_sc_world_frame_made"] = True
    except Exception as exc:
        scene_handles["sc_world_frame_error"] = f"{type(exc).__name__}: {exc}"
    return _SC_WORLD_FRAME


def _add_self_collision_witness_markers(server: Any, handles: dict[str, Any]) -> None:
    """Closest-point (witness) markers for the self-collision guard.

    Two small spheres on the min-clearance pair's bone AXES — i.e. on the pair
    members themselves (yellow = first member, cyan = second) — plus a
    connecting segment and a label showing the judged capsule-surface
    clearance. Hidden unless self_collision.violated. They answer "which spot
    on each member was judged this close?" when the visual mesh gap looks
    larger than the capsule clearance (the capsule radii inflate the links)."""
    try:
        if not hasattr(server.scene, "add_icosphere"):
            return
        handles["_server"] = server
        frame = _ensure_sc_world_frame(handles)  # witness points are in URDF-world frame
        for key, name, color in (
            ("self_collision_point_a", f"{frame}/self_collision_point_a", (255, 220, 0)),
            ("self_collision_point_b", f"{frame}/self_collision_point_b", (0, 229, 255)),
        ):
            handles[key] = server.scene.add_icosphere(
                name,
                radius=0.006,
                color=color,
                position=(0.0, 0.0, 0.0),
                visible=False,
            )
        if hasattr(server.scene, "add_line_segments"):
            import numpy as np

            handles["self_collision_gap_line"] = server.scene.add_line_segments(
                f"{frame}/self_collision_gap_line",
                points=np.zeros((1, 2, 3), dtype=np.float32),
                colors=np.full((1, 2, 3), (255, 220, 0), dtype=np.uint8),
                line_width=3.0,
                visible=False,
            )
        if hasattr(server.scene, "add_label"):
            handles["self_collision_gap_label"] = server.scene.add_label(
                f"{frame}/self_collision_gap_label",
                text="",
                position=(0.0, 0.0, 0.0),
                visible=False,
            )
    except Exception as exc:
        handles["self_collision_witness_error"] = f"{type(exc).__name__}: {exc}"


def _update_self_collision_witness_markers(
    scene_handles: dict[str, Any], sc: Mapping[str, Any] | None, violated: bool
) -> None:
    """Place/hide the witness-point markers from self_collision telemetry."""
    point_a = scene_handles.get("self_collision_point_a")
    point_b = scene_handles.get("self_collision_point_b")
    line = scene_handles.get("self_collision_gap_line")
    label = scene_handles.get("self_collision_gap_label")
    if point_a is None or point_b is None:
        return

    def _xyz(value: Any) -> tuple[float, float, float] | None:
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            return None
        try:
            p = tuple(float(v) for v in value)
        except (TypeError, ValueError):
            return None
        return p if all(math.isfinite(v) for v in p) else None

    a = _xyz(sc.get("closest_point_a_m")) if isinstance(sc, Mapping) else None
    b = _xyz(sc.get("closest_point_b_m")) if isinstance(sc, Mapping) else None
    show = bool(violated and a is not None and b is not None)
    if show:
        try:
            point_a.position = a
            point_b.position = b
            if line is not None:
                import numpy as np

                line.points = np.array([[a, b]], dtype=np.float32)
            if label is not None:
                clearance = sc.get("min_clearance_m") if isinstance(sc, Mapping) else None
                if isinstance(clearance, (int, float)) and math.isfinite(float(clearance)):
                    label.text = f"{float(clearance) * 1000:.1f}mm"
                label.position = (
                    (a[0] + b[0]) / 2.0,
                    (a[1] + b[1]) / 2.0,
                    (a[2] + b[2]) / 2.0 + 0.02,
                )
        except Exception as exc:
            scene_handles["self_collision_witness_error"] = f"{type(exc).__name__}: {exc}"
    for handle in (point_a, point_b, line, label):
        _set_visible(handle, show)


def _xyz3(value: Any) -> tuple[float, float, float] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    try:
        p = tuple(float(v) for v in value)
    except (TypeError, ValueError):
        return None
    return p if all(math.isfinite(v) for v in p) else None  # type: ignore[return-value]


def _capsule_axis_wxyz(direction: tuple[float, float, float]) -> tuple[float, float, float, float]:
    """Quaternion (w,x,y,z) rotating the capsule's +Z axis onto `direction`."""
    import numpy as np

    z = np.array([0.0, 0.0, 1.0])
    v = np.asarray(direction, dtype=float)
    norm = float(np.linalg.norm(v))
    if norm < 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    v = v / norm
    c = float(np.clip(np.dot(z, v), -1.0, 1.0))
    if c > 1.0 - 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    if c < -1.0 + 1e-9:
        return (0.0, 1.0, 0.0, 0.0)  # 180 deg about X
    axis = np.cross(z, v)
    axis = axis / float(np.linalg.norm(axis))
    half = math.acos(c) / 2.0
    s = math.sin(half)
    return (math.cos(half), float(axis[0] * s), float(axis[1] * s), float(axis[2] * s))


def _place_capsule(handle: Any, p0: tuple[float, float, float], p1: tuple[float, float, float]) -> None:
    seg = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
    handle.position = ((p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0, (p0[2] + p1[2]) / 2.0)
    handle.wxyz = _capsule_axis_wxyz(seg)


# URDF mesh self-collision close-call overlay (mesh-mode analogue of the capsule
# overlay above). The server's self_collision.near_pairs telemetry lists the K
# closest checked pairs as witness segments {p_a_m, p_b_m, clearance_m}; we draw a
# thin tube per pair, red when within the hard floor, yellow when merely close.
_SELF_COLLISION_NEAR_HARD_RGB = (235, 40, 40)       # clearance < d_hard
_SELF_COLLISION_NEAR_CAUTION_RGB = (255, 210, 40)   # d_hard <= clearance < d_slow
_SELF_COLLISION_NEAR_OK_RGB = (70, 200, 90)         # clearance >= d_slow (in viz reach)
_SELF_COLLISION_NEAR_RADIUS_M = 0.004
_SELF_COLLISION_NEAR_OPACITY = 0.9


def _ensure_near_pair_handle(
    server: Any, scene_handles: dict[str, Any], key: str, length: float, rgb: tuple[int, int, int]
) -> Any:
    cache = scene_handles.setdefault("_self_collision_near_cache", {})
    entry = cache.get(key)
    if entry is not None and abs(entry["length"] - length) < 1e-4 and entry["rgb"] == rgb:
        return entry["handle"]
    if entry is not None:
        try:
            entry["handle"].remove()
        except Exception:
            pass
    try:
        import trimesh

        mesh = trimesh.creation.capsule(
            height=max(float(length), 1e-4), radius=_SELF_COLLISION_NEAR_RADIUS_M, count=[5, 8]
        )
        frame = _ensure_sc_world_frame(scene_handles)  # near_pairs are in URDF-world frame
        handle = server.scene.add_mesh_simple(
            f"{frame}/self_collision_near/{key}",
            vertices=mesh.vertices,
            faces=mesh.faces,
            color=rgb,
            opacity=_SELF_COLLISION_NEAR_OPACITY,
            visible=False,
        )
    except Exception as exc:
        scene_handles["self_collision_near_error"] = f"{type(exc).__name__}: {exc}"
        return None
    cache[key] = {"handle": handle, "length": float(length), "rgb": rgb}
    return handle


def update_self_collision_near_pairs(scene_handles: dict[str, Any], latest: Any, show: bool) -> None:
    """Show/update the URDF-mesh close-call segments from self_collision.near_pairs.
    A thin tube spans each checked pair's witness points, colored by clearance band:
    red < d_hard, amber in [d_hard, d_slow), green >= d_slow (still within the server's
    viz reach). Driven by the COMMANDED (q_sent) verdict the server publishes, so it
    mirrors what the guard checks even when the displayed q_actual diverges."""
    if not isinstance(scene_handles, dict):
        return
    server = scene_handles.get("_server")
    cache = scene_handles.setdefault("_self_collision_near_cache", {})
    used_keys: set[str] = set()
    if show and server is not None:
        sc = getattr(latest, "self_collision", None) if latest is not None else None
        if isinstance(sc, Mapping):
            hard = _finite_or(sc.get("margin_m"), 0.005)  # margin_m == mesh d_hard_m
            manifest = sc.get("manifest") if isinstance(sc.get("manifest"), Mapping) else {}
            slow = _finite_or(manifest.get("d_slow_m"), max(hard, 0.025))
            pairs = sc.get("near_pairs")
            if isinstance(pairs, (list, tuple)):
                for i, pair in enumerate(pairs):
                    if not isinstance(pair, Mapping):
                        continue
                    a = _xyz3(pair.get("p_a_m"))
                    b = _xyz3(pair.get("p_b_m"))
                    clearance = pair.get("clearance_m")
                    if a is None or b is None or not isinstance(clearance, (int, float)):
                        continue
                    c = float(clearance)
                    if c < hard:
                        band, rgb = "hard", _SELF_COLLISION_NEAR_HARD_RGB
                    elif c < slow:
                        band, rgb = "caution", _SELF_COLLISION_NEAR_CAUTION_RGB
                    else:
                        band, rgb = "ok", _SELF_COLLISION_NEAR_OK_RGB
                    key = f"{i}_{band}"
                    handle = _ensure_near_pair_handle(server, scene_handles, key, math.dist(a, b), rgb)
                    if handle is None:
                        continue
                    try:
                        _place_capsule(handle, a, b)
                    except Exception:
                        pass
                    used_keys.add(key)
    for key, entry in cache.items():
        _set_visible(entry["handle"], show and key in used_keys)


def _finite_or(value: Any, default: float) -> float:
    """Return float(value) if it is a finite number, else the default."""
    if isinstance(value, (int, float)) and math.isfinite(float(value)):
        return float(value)
    return default


def _checkgeom_joint_config(scene_handles: dict[str, Any], latest: Any) -> Any:
    """Build the unified-URDF actuated-joint config (radians) from the live left/right
    joint states. The unified joint names are "<prefix>+<base>" exactly as the server
    builds left_joints/right_joints, so the GUI maps each command-order angle onto the
    matching unified joint, then orders by the URDF's actuated-joint list."""
    info = scene_handles.get("checkgeom_manifest")
    if not info or latest is None:
        return None
    joint_names = info.get("joint_names") or []
    actuated = info.get("actuated") or ()
    if not joint_names or not actuated:
        return None
    angles: dict[str, float] = {}
    for side, prefix in (("left", info.get("left_prefix", "")),
                         ("right", info.get("right_prefix", ""))):
        arm_state = getattr(latest, side, None)
        if arm_state is None:
            continue
        # The async monitor checks the COMMANDED targets (q_sent), so the checked-
        # geometry overlay must pose to q_sent too — otherwise (e.g. pgmode
        # controller-sim, where q_actual need not track the command) the overlay
        # shows a different pose than the guard actually evaluates. Fall back to
        # q_actual only when q_sent is unavailable.
        q = arm_state.q_sent_deg
        if q is None:
            q = arm_state.q_actual_deg
        if q is None:
            continue
        for i, base in enumerate(joint_names):
            if i < len(q):
                angles[f"{prefix}{base}"] = math.radians(float(q[i]))
    return [float(angles.get(name, 0.0)) for name in actuated]


def _build_checkgeom_overlay(scene_handles: dict[str, Any], manifest: Mapping[str, Any]) -> Any:
    """Lazily build the unified collision-hull overlay (stand + both arms' URDF
    <collision> convex hulls) from the manifest, once a manifest is available."""
    server = scene_handles.get("_server")
    urdf_path = manifest.get("unified_urdf")
    if server is None or not isinstance(urdf_path, str) or not urdf_path:
        return None
    if not Path(urdf_path).exists():
        scene_handles["urdf_checkgeom_error"] = _asset_error(f"unified URDF not found: {urdf_path}")
        return None
    frame = _ensure_sc_world_frame(scene_handles)
    try:
        from viser.extras import ViserUrdf

        # Root at the URDF's `world` link under the -90deg frame so the URDF `stand`
        # frame (world->stand is +90deg about Z) lands on the scene /stand and the
        # arms overlay the solid robots.
        overlay = ViserUrdf(
            server, Path(urdf_path), root_node_name=f"{frame}/collision_checkgeom",
            load_meshes=False, load_collision_meshes=True,
            collision_mesh_color_override=_SELF_COLLISION_CHECK_RGBA)
    except Exception as exc:
        scene_handles["urdf_checkgeom_error"] = f"{type(exc).__name__}: {exc}"
        return None
    scene_handles["checkgeom_urdf"] = overlay
    scene_handles["checkgeom_manifest"] = {
        "left_prefix": manifest.get("left_prefix", ""),
        "right_prefix": manifest.get("right_prefix", ""),
        "joint_names": list(manifest.get("joint_names") or []),
        "actuated": tuple(overlay.get_actuated_joint_names()),
    }
    _attach_checkgeom_gripper(scene_handles, overlay, manifest)
    return overlay


def _attach_checkgeom_gripper(scene_handles: dict[str, Any], overlay: Any, manifest: Mapping[str, Any]) -> None:
    """Mirror the monitor's runtime Pika-gripper attach in the viewer WITHOUT touching
    the safety URDF: load the gripper mesh from the manifest and parent it under the
    unified overlay's "<prefix>attachment_site" link frame (Z+90deg about Z, manifest
    scale), so the gripper shows as part of the checked geometry. The link frame is
    located via ViserUrdf's per-joint frames; skips gracefully if unavailable."""
    server = scene_handles.get("_server")
    mesh_path = manifest.get("pika_gripper_mesh")
    attach = manifest.get("gripper_attach") if isinstance(manifest.get("gripper_attach"), Mapping) else {}
    if server is None or not isinstance(mesh_path, str) or not mesh_path or not Path(mesh_path).exists():
        return
    try:
        import trimesh

        scale = float(attach.get("mesh_scale", 0.001))
        rpy = attach.get("rpy") or [0.0, 0.0, math.pi / 2.0]
        suffix = str(attach.get("frame_suffix", "attachment_site"))
        base_mesh = trimesh.load_mesh(mesh_path)
        base_mesh.apply_scale(scale)
        rgb = tuple(int(round(c * 255)) for c in _SELF_COLLISION_CHECK_RGBA[:3])
        opacity = float(_SELF_COLLISION_CHECK_RGBA[3])
        wxyz = _pose_wxyz((0.0, 0.0, 0.0, float(rpy[0]), float(rpy[1]), float(rpy[2])))
        # ViserUrdf names each joint-child frame by its full kinematic path; find the
        # two attachment_site frames (left/right) by link-name suffix.
        frames = list(getattr(overlay, "_joint_frames", []) or [])
        handles: list[Any] = []
        for prefix in (manifest.get("left_prefix", ""), manifest.get("right_prefix", "")):
            link = f"{prefix}{suffix}"
            frame = next(
                (f for f in frames if str(getattr(f, "name", "")).endswith("/" + link)), None)
            if frame is None:
                scene_handles["checkgeom_gripper_error"] = f"attachment frame not found: {link}"
                continue
            handles.append(server.scene.add_mesh_simple(
                f"{frame.name}/pika_gripper",
                vertices=base_mesh.vertices,
                faces=base_mesh.faces,
                color=rgb,
                opacity=opacity,
                wxyz=wxyz,
                position=(0.0, 0.0, 0.0),
                visible=False,
            ))
        scene_handles["checkgeom_gripper"] = handles
    except Exception as exc:
        scene_handles["checkgeom_gripper_error"] = f"{type(exc).__name__}: {exc}"


def update_self_collision_check_geom(scene_handles: dict[str, Any], latest: Any, show: bool) -> None:
    """Translucent collision-HULL overlay: the EXACT unified-URDF collision geometry
    the async monitor checks (stand + both arms' <collision> convex hulls), built
    lazily from the server's self_collision.manifest so the viewer mirrors the server
    config (single source of truth) instead of a hardcoded viewer URDF. Posed by the
    live left/right joint states mapped onto the unified URDF's prefixed joints. The
    Pika gripper (which the monitor attaches at runtime, not in the URDF) is mirrored
    as a child mesh on each attachment_site frame so it shows as checked geometry."""
    if not isinstance(scene_handles, dict):
        return
    sc = getattr(latest, "self_collision", None) if latest is not None else None
    manifest = sc.get("manifest") if isinstance(sc, Mapping) else None
    overlay = scene_handles.get("checkgeom_urdf")
    if overlay is None:
        # Build once a manifest is available and the operator wants it shown.
        if not show or not isinstance(manifest, Mapping):
            return
        overlay = _build_checkgeom_overlay(scene_handles, manifest)
        if overlay is None:
            return
    if show:
        cfg = _checkgeom_joint_config(scene_handles, latest)
        if cfg is not None:
            try:
                _update_urdf_config(overlay, cfg)
            except Exception as exc:
                scene_handles["urdf_checkgeom_update_error"] = f"{type(exc).__name__}: {exc}"
    try:
        overlay.show_collision = show
    except Exception as exc:
        scene_handles["urdf_checkgeom_show_error"] = f"{type(exc).__name__}: {exc}"
    # The gripper meshes are independent child handles (not part of show_collision).
    for handle in scene_handles.get("checkgeom_gripper", []):
        _set_visible(handle, show)
    # Collision view: hide the solid visual robots while the overlay is on so the
    # hulls (which hug the links) are not occluded; restore them when off.
    for side in ("left", "right"):
        solid = scene_handles.get(f"{side}_urdf")
        if solid is not None:
            try:
                solid.show_visual = not show
            except Exception:
                pass


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
        # Translucent-red duplicate for the self-collision overlay (hidden until
        # violated). add_mesh_simple (not add_mesh_trimesh): trimesh face-color
        # alpha is not honored, opacity= is.
        try:
            handles["stand_mesh_collision"] = server.scene.add_mesh_simple(
                "/stand/mesh_collision",
                vertices=mesh.vertices,
                faces=mesh.faces,
                color=_SELF_COLLISION_STAND_RGB,
                opacity=_SELF_COLLISION_STAND_OPACITY,
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
        # Translucent-red self-collision overlay robots (hidden until violated).
        # In pgmode real they replace the actual robot; in pgmode simulation
        # they replace the commanded ghost.
        try:
            handles["left_urdf_collision"] = ViserUrdf(
                server, urdf_path, root_node_name="/stand/left_base_collision",
                mesh_color_override=_SELF_COLLISION_RGBA)
            handles["right_urdf_collision"] = ViserUrdf(
                server, urdf_path, root_node_name="/stand/right_base_collision",
                mesh_color_override=_SELF_COLLISION_RGBA)
        except Exception as exc:
            handles["urdf_collision_error"] = f"{type(exc).__name__}: {exc}"
        # The checked collision-GEOMETRY overlay is no longer two per-arm URDFs:
        # it is a single unified stand+both-arms collision-hull URDF built lazily
        # from the server's self_collision.manifest (so the viewer mirrors exactly
        # what the monitor checks). See update_self_collision_check_geom.
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
        # Mount frames for the translucent-red self-collision overlay robots.
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
            # depth_test=False keeps the TCP gizmos drawn ON TOP of the translucent
            # safety visuals (ROI box / reach envelope). Those meshes write the depth
            # buffer, so without this the gizmo sitting inside them gets occluded by
            # the near mesh face and disappears when the operator turns them on.
            for side in ("left", "right"):
                pos = (0.1601 if side == "left" else -0.1601, -0.1725, 0.78)
                try:
                    handles[f"{side}_tcp_target"] = server.scene.add_transform_controls(
                        f"/stand/{side}_tcp_target", scale=0.16, line_width=3.0,
                        depth_test=False, position=pos,
                    )
                except TypeError:  # older viser without depth_test support
                    handles[f"{side}_tcp_target"] = server.scene.add_transform_controls(
                        f"/stand/{side}_tcp_target", scale=0.16, line_width=3.0, position=pos,
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
        _add_roi_box(server, handles)
        _add_reachability_cloud(server, handles)
        _add_self_collision_witness_markers(server, handles)
        # Keep the server handle so the self-collision checked-geometry overlay can
        # lazily build the unified collision-hull URDF from the runtime manifest.
        handles["_server"] = server
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
        # In pgmode controller-simulation the controller does NOT execute the streamed
        # command, so q_actual is decoupled from q_sent (often tens of degrees off) and
        # the "actual" arm is misleading — it can look like it penetrates the stand
        # while the COMMANDED pose the safety guard checks is clear. Show the command
        # (q_sent) there so the displayed robot matches what is actually checked/sent.
        # Real motion (q_actual tracks the command) keeps showing the true q_actual.
        q = arm_state.q_actual_deg
        if _arm_is_controller_sim(arm_state) and arm_state.q_sent_deg is not None:
            q = arm_state.q_sent_deg
        try:
            _update_urdf_config(urdf_handle, _joint_cfg_radians(q))
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
