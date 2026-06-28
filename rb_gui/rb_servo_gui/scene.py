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
from .models import EXTERNAL_BOX_COLLISION_M, CircleOverlaySnapshot

_ROBOT_JOINT_NAMES = (
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist1_joint",
    "wrist2_joint",
    "wrist3_joint",
)
# Rotation values are canonical URDF/ROS RPY converted from MJCF euler xyz.
_DEFAULT_LEFT_POSE = (0.15707, -0.17036, 0.58036, 2.186649, 0.523831, 2.526296)
_DEFAULT_RIGHT_POSE = (-0.15707, -0.17036, 0.58036, 2.186649, -0.523831, -2.526296)
_DEFAULT_STAND_MESH_POSE = (0.0, 0.0, 0.001, 0.0, 0.0, -1.57078)
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


def _trail_line_arrays(points: Any, base_color: tuple[int, int, int]) -> tuple[Any, Any]:
    """Turn a list of trail positions into a glowing 'comet trail' line.

    Consecutive points become line segments; per-vertex colour fades the base
    arm colour from dim (oldest sample) to full brightness (newest sample), so
    the most recent motion glows and the tail trails off smoothly.
    """
    import numpy as np

    pts = np.asarray(points, dtype=np.float32).reshape((-1, 3))
    n = pts.shape[0]
    if n < 2:
        return (
            np.zeros((0, 2, 3), dtype=np.float32),
            np.zeros((0, 2, 3), dtype=np.uint8),
        )
    segments = np.stack([pts[:-1], pts[1:]], axis=1)  # (n-1, 2, 3)
    base = np.asarray(base_color, dtype=np.float32)
    # Ease-in fade so recent samples stay bright while the tail dims toward black.
    lo, hi = 0.10, 1.0
    t = np.linspace(0.0, 1.0, n, dtype=np.float32) ** 1.6
    fade = lo + (hi - lo) * t
    vert_colors = np.clip(base[None, :] * fade[:, None], 0.0, 255.0)  # (n, 3)
    seg_colors = np.stack([vert_colors[:-1], vert_colors[1:]], axis=1)  # (n-1, 2, 3)
    return segments, seg_colors.astype(np.uint8)


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
    # Prefer the GUI-only articulated-gripper URDF (base + two prismatic fingers) so
    # the viewer can show continuous open/close; fall back to the plain single-mesh
    # gripper if that asset is absent. RB_GUI_ROBOT_URDF still forces a specific
    # file. The C++ Pinocchio FK/IK is unaffected (it loads its own rb3_730e.urdf
    # via kinematics.urdf, never this one).
    configured = os.environ.get("RB_GUI_ROBOT_URDF")
    if configured:
        return Path(configured)
    articulated = _descriptions_dir() / "urdf/rb3_730e_pika_articulated.urdf"
    if articulated.exists():
        return articulated
    return _descriptions_dir() / "urdf/rb3_730e.urdf"


def _stand_mesh_path() -> Path:
    return _asset_path("RB_GUI_STAND_MESH", "meshes/stands/dual_rb3_730e/dual_rb3_730e_stand_ver2_clean.stl")


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


# Articulated-gripper finger joints (only present in rb3_730e_pika_articulated.urdf).
# Each finger travels up to _GRIPPER_FINGER_TRAVEL_M from the open (STL) pose to
# closed; finger_left moves +X, finger_right -X (jaw axis = X in the baked mesh
# frame). gripper percent: 100 = open (travel 0), 0 = closed (full travel).
_GRIPPER_FINGER_TRAVEL_M = 0.047
_GRIPPER_FINGER_JOINT_SIGN = {"finger_left_joint": 1.0, "finger_right_joint": -1.0}


def _finger_position_m(gripper_percent: float | None) -> float:
    """Per-finger prismatic travel (m) for a gripper open percentage (0..100)."""
    if gripper_percent is None:
        return 0.0
    try:
        pct = max(0.0, min(100.0, float(gripper_percent)))
    except (TypeError, ValueError):
        return 0.0
    return (1.0 - pct / 100.0) * _GRIPPER_FINGER_TRAVEL_M


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
# Neutral mid-dark gray for the stand. (The RB3-730E URDF "dark_gray" material
# reads near-black with a green-cyan cast, since its rgba 0.07227/0.09084/0.09759
# is neither neutral nor light; this flat (90, 90, 90) is true gray and lighter.)
_RB3_DARK_GRAY_RGB = (90, 90, 90)
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


# Applied safety floor plane fill: emerald-green so it is clearly distinct from the
# blue ROI box (the operator keeps the ROI colors but wants the floor to stand out).
# Red when an arm violates the plane.
_FLOOR_PLANE_GREEN = (45, 205, 130)
_FLOOR_PLANE_RED = (220, 60, 60)
# Pending (not-yet-sent) slider value preview: distinct color so it cannot be
# mistaken for the APPLIED safety plane.
_FLOOR_PLANE_PREVIEW_YELLOW = (235, 200, 60)
# Boundary emphasis for the applied plane: opaque edges + corner vertices drawn over
# the translucent fill so the outline reads clearly even where the ROI box overlaps
# it. Brighter than the fill; recolor red together with the fill on violation.
# Mirrors the ROI box boundary treatment.
_FLOOR_PLANE_EDGE_GREEN = (170, 255, 205)
_FLOOR_PLANE_EDGE_RED = (255, 120, 120)
# Stand-frame footprint of the floor plane visual: an 800 x 800 mm square whose
# near (+y) edge is aligned to the stand's +y back face so the plane "starts" where
# the stand URDF does. The rendered stand mesh (dual_rb3_730e_stand_ver2_clean.stl,
# placed under /stand/mesh) spans y in [-0.269, +0.120] m in the stand frame; the
# plane's +y edge sits at that +0.120 back face and the square extends 0.8 m forward
# into the workspace (-y). With width 0.8 and the +y edge at +0.120, the center is
# y = 0.120 - 0.4 = -0.28; x stays centered (x in [-0.4, 0.4] m). This is a visual
# footprint only — the actual safety constraint is the server-side z-plane.
_FLOOR_PLANE_DIMENSIONS = (0.8, 0.8, 0.002)
_FLOOR_PLANE_CENTER_XY = (0.0, -0.28)

# User Safety Floor (safety.user_floor_constraint) visual: a TILTED plane oriented
# by the fitted normal. Purple when compliant, red on violation. Captured contact
# points render as a small point cloud (left=cyan, right=magenta) so the operator
# sees the samples the plane was fit through.
_USER_FLOOR_PLANE_PURPLE = (150, 90, 220)
_USER_FLOOR_PLANE_RED = (220, 60, 60)
_USER_FLOOR_PLANE_DIMENSIONS = (0.8, 0.8, 0.002)
_USER_FLOOR_POINT_LEFT = (60, 210, 230)
_USER_FLOOR_POINT_RIGHT = (230, 90, 210)
# Boundary emphasis for the tilted user floor, mirroring the stand floor / ROI box:
# opaque edges + corner vertices over the translucent fill. Brighter than the fill;
# recolor red together with the fill on violation.
_USER_FLOOR_EDGE_PURPLE = (200, 160, 255)
_USER_FLOOR_EDGE_RED = (255, 120, 120)

# Floor/ROI collision check points (safety.floor_constraint.tcp_offset_points):
# the exact samples the server's floor/ROI guards FK-check against the plane —
# the TCP point itself plus the configured TCP-frame offset points (PIKA gripper
# fingertips). Offsets are expressed in the TCP body frame, so the point cloud is
# parented under /stand/<arm>_tcp and inherits the live TCP pose for free (no
# per-tick transform math in the GUI). Mirrors the 4-point set in the real stack
# config (rb_servo_server/config/stack_real.yaml: x +/-0.057, y +/-0.012);
# the leading (0,0,0) is the TCP point. The GUI is a pure network client and the
# server does not publish these offsets, so they live here as a constant — keep
# BOTH the open and closed sets in sync with floor_constraint.tcp_offset_points
# (offset_m / offset_closed_m) if the gripper geometry changes.
#
# The server interpolates each fingertip between its gripper-OPEN (offset_m) and
# gripper-CLOSED (offset_closed_m) position by the live gripper open percent;
# update_floor_check_points() mirrors that here so the orange dots track the jaws.
# CLOSED set: x +/-0.010 = open x +/-0.057 - URDF finger travel 0.047 m (jaw axis
# = TCP x); the TCP point and y/z stay put. Rendered as orange dots, hidden until
# the operator enables the GUI toggle.
_FLOOR_CHECK_POINT_ORANGE = (255, 140, 0)
_FLOOR_CHECK_POINTS_TCP_FRAME = (
    (0.0, 0.0, 0.0),        # tcp (the TCP point checked against the floor)
    (0.057, 0.012, 0.0),    # gripper_tip_a_yp (OPEN)
    (0.057, -0.012, 0.0),   # gripper_tip_a_yn (OPEN)
    (-0.057, 0.012, 0.0),   # gripper_tip_b_yp (OPEN)
    (-0.057, -0.012, 0.0),  # gripper_tip_b_yn (OPEN)
)
_FLOOR_CHECK_POINTS_TCP_FRAME_CLOSED = (
    (0.0, 0.0, 0.0),        # tcp (unchanged)
    (0.010, 0.012, 0.0),    # gripper_tip_a_yp (CLOSED)
    (0.010, -0.012, 0.0),   # gripper_tip_a_yn (CLOSED)
    (-0.010, 0.012, 0.0),   # gripper_tip_b_yp (CLOSED)
    (-0.010, -0.012, 0.0),  # gripper_tip_b_yn (CLOSED)
)


def _interpolated_floor_check_points(gripper_percent: float | None) -> "np.ndarray":
    """Floor/ROI check points in the TCP frame, linearly interpolated between the
    gripper-OPEN and gripper-CLOSED sets by the gripper open percent (0..100),
    matching the server's interpolateOffsetPoints: p = closed + t*(open-closed),
    t = clamp(pct,0,100)/100. None/invalid percent -> OPEN (the conservative
    fallback the server uses for absent gripper feedback)."""
    import numpy as np

    open_pts = np.asarray(_FLOOR_CHECK_POINTS_TCP_FRAME, dtype=np.float32)
    closed_pts = np.asarray(_FLOOR_CHECK_POINTS_TCP_FRAME_CLOSED, dtype=np.float32)
    if gripper_percent is None:
        return open_pts
    try:
        t = max(0.0, min(100.0, float(gripper_percent))) / 100.0
    except (TypeError, ValueError):
        return open_pts
    return closed_pts + t * (open_pts - closed_pts)


# Reachable-workspace OUTER-SHELL surface (tools/reach_envelope.py output): only
# the farthest reachable points, triangulated into a closed surface and rendered
# translucent + double-sided so the robot and stand stay visible through it. The
# shell's outer face is the reach boundary the safety.reach_constraint damper
# enforces.
_REACH_ENVELOPE_GREEN = (90, 200, 150)
_REACH_ENVELOPE_OPACITY = 0.22


def _reach_envelope_path() -> Path:
    return _asset_path("RB_GUI_REACH_ENVELOPE", "reach_envelope_rb3_730e.npz")


def _orient_faces_outward(verts: Any, faces: Any) -> Any:
    """Re-wind triangles so every face normal points away from the mesh centroid.

    The reach-envelope shell (tools/reach_envelope.py) is a lat/lon grid with no
    guaranteed winding, so side="back" can't be trusted to cull the NEAR faces
    until the normals are made consistently outward. The shell is roughly
    star-shaped from its centroid, so "normal·(face_center - centroid) >= 0" is a
    reliable outward test for it. Faces failing it get two indices swapped."""
    import numpy as np

    centroid = verts.mean(axis=0)
    v0, v1, v2 = verts[faces[:, 0]], verts[faces[:, 1]], verts[faces[:, 2]]
    normals = np.cross(v1 - v0, v2 - v0)
    outward = (v0 + v1 + v2) / 3.0 - centroid
    flip = np.einsum("ij,ij->i", normals, outward) < 0.0
    fixed = faces.copy()
    fixed[flip, 1] = faces[flip, 2]
    fixed[flip, 2] = faces[flip, 1]
    return fixed


def _add_reachability_cloud(server: Any, handles: dict[str, Any]) -> None:
    """Per-arm reachable-workspace shell mesh (tools/reach_envelope.py output).

    The saved vertices are in the arm-base frame, so attaching the SAME mesh under
    each arm's /stand/<side>_base node renders it correctly placed and mirrored via
    the mount transform (no manual per-arm transform needed). Drawn translucent and
    back-sided (after orienting normals outward): the near hemisphere is culled so
    it never writes depth over the translucent geometry INSIDE the shell (the
    reference ghost and the red self-collision overlays), while the robot/stand
    still show through and the shell stays visible from inside. Skips gracefully if
    the asset is missing (run tools/reach_envelope.py to generate it) or viser has
    no mesh support. Static geometry — visibility is toggled from the GUI."""
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
        faces = _orient_faces_outward(verts, faces)  # make side="back" reliable
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
                    # back-sided: cull the near hemisphere so it doesn't write depth
                    # over the translucent overlays inside; still visible from inside.
                    side="back",
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


# "A 영역" base-axis singularity cylinder: the column along each arm's J1 axis
# where Move J reaches fine but Cartesian/Move L control forces runaway joint speed
# (vendor "A 영역", rb_cobot_docs). Distinct from the reach envelope (positions too
# FAR to reach): this marks the inner velocity-singularity core. A capped cylinder
# of radius R = v_ref/dq_max, axial extent FK-clipped to the reach envelope, built
# by tools/ik_infeasible_region.py. The same base-frame cylinder serves both arms
# (the singularity is mount-independent in base frame); each /stand/<side>_base node
# applies the mount tilt. Drawn vivid red; back-face culling is fine (convex
# watertight). A cylinder has no real edges, so the rim is emphasised separately with
# bright opaque line segments (top/bottom circles + vertical generators) so the column
# reads clearly even through the translucent fill (see _cylinder_outline_segments).
_IK_INFEASIBLE_RED = (255, 45, 45)
_IK_INFEASIBLE_OPACITY = 0.22
_IK_INFEASIBLE_EDGE_RED = (255, 130, 130)


def _ik_infeasible_path() -> Path:
    return _asset_path("RB_GUI_IK_INFEASIBLE", "ik_infeasible_rb3_730e.npz")


def _cylinder_outline_segments(
    radius: float, z_lo: float, z_hi: float, *, n_around: int = 48, n_verticals: int = 0
) -> Any:
    """Line segments (M,2,3) outlining a z-axis cylinder: top + bottom rim circles
    plus a few vertical generator lines. Coaxial with base z, centered at xy=(0,0)."""
    import numpy as np

    theta = np.linspace(0.0, 2.0 * np.pi, int(n_around), endpoint=False)
    cos = np.cos(theta) * float(radius)
    sin = np.sin(theta) * float(radius)
    segments = []
    for z in (float(z_lo), float(z_hi)):  # two rim circles
        ring = np.stack([cos, sin, np.full_like(cos, z)], axis=1)
        for i in range(len(ring)):
            segments.append([ring[i], ring[(i + 1) % len(ring)]])
    if int(n_verticals) > 0:  # vertical generators (n_verticals=0 -> rim circles only)
        step = max(1, int(n_around) // int(n_verticals))
        for i in range(0, int(n_around), step):
            segments.append([[cos[i], sin[i], float(z_lo)], [cos[i], sin[i], float(z_hi)]])
    return np.asarray(segments, dtype=np.float32)


def _ik_infeasible_outline_geometry(data: Any, arm: str) -> Any:
    """Cylinder rim segments from the asset (radius_m/z_lo_m/z_hi_m), or from the
    arm's vertices as a fallback. Returns None if geometry can't be derived."""
    import numpy as np

    verts = np.asarray(data[f"{arm}_vertices_base_m"], dtype=np.float64)
    if verts.ndim != 2 or verts.shape[1] != 3 or len(verts) == 0:
        return None
    radius = float(data["radius_m"]) if "radius_m" in data else float(
        np.max(np.sqrt(verts[:, 0] ** 2 + verts[:, 1] ** 2))
    )
    z_lo = float(data["z_lo_m"]) if "z_lo_m" in data else float(np.min(verts[:, 2]))
    z_hi = float(data["z_hi_m"]) if "z_hi_m" in data else float(np.max(verts[:, 2]))
    if not (math.isfinite(radius) and radius > 0.0 and z_hi > z_lo):
        return None
    return _cylinder_outline_segments(radius, z_lo, z_hi)


def _add_ik_infeasible_region(server: Any, handles: dict[str, Any]) -> None:
    """Per-arm "A 영역" base-axis singularity cylinder (tools/ik_infeasible_region.py).

    The asset holds a base-frame capped cylinder coaxial with each arm's J1 axis
    (vendor "A 영역": the base-axis velocity singularity where Cartesian/Move L
    control forces runaway joint speed). The cylinder is identical in both arms'
    base frames; each is attached under its /stand/<side>_base node so the mount
    tilt is applied automatically. Skips gracefully if the asset is missing (run
    tools/ik_infeasible_region.py) or viser lacks mesh support. Static geometry —
    visibility is toggled from the GUI 안전 tab."""
    if not hasattr(server.scene, "add_mesh_simple"):
        return
    path = _ik_infeasible_path()
    if not path.exists():
        handles["ik_infeasible_error"] = (
            f"IK-infeasible asset not found: {path} (run tools/ik_infeasible_region.py)"
        )
        return
    try:
        import numpy as np

        data = np.load(path)
        total_cells = 0
        added = False
        for arm in ("left", "right"):
            verts = np.asarray(data[f"{arm}_vertices_base_m"], dtype=np.float32)
            faces = np.asarray(data[f"{arm}_faces"], dtype=np.int32)
            if verts.ndim != 2 or verts.shape[1] != 3 or len(verts) == 0 or len(faces) == 0:
                continue  # this arm has no infeasible region (or empty) — skip it
            if f"{arm}_cells" in data:
                total_cells += int(data[f"{arm}_cells"])
            try:
                handles[f"{arm}_ik_infeasible"] = server.scene.add_mesh_simple(
                    f"/stand/{arm}_base/ik_infeasible",
                    vertices=verts,
                    faces=faces,
                    color=_IK_INFEASIBLE_RED,
                    opacity=_IK_INFEASIBLE_OPACITY,
                    side="back",        # convex watertight cylinder: cull near faces for clean translucency
                    flat_shading=False,
                    visible=False,      # brought up by the GUI "A 영역(특이점 원통) 표시" toggle
                )
            except TypeError:  # older viser without opacity/side support
                handles[f"{arm}_ik_infeasible"] = server.scene.add_mesh_simple(
                    f"/stand/{arm}_base/ik_infeasible",
                    vertices=verts,
                    faces=faces,
                    color=_IK_INFEASIBLE_RED,
                    visible=False,
                )
            # Bright opaque rim outline (top/bottom circles + vertical generators) so
            # the cylinder reads clearly through the translucent fill. Same /stand/
            # <arm>_base parent as the mesh, so it inherits the mount tilt; toggled
            # together with the fill in set_ik_infeasible_region_visible.
            if hasattr(server.scene, "add_line_segments"):
                try:
                    outline = _ik_infeasible_outline_geometry(data, arm)
                    if outline is not None:
                        handles[f"{arm}_ik_infeasible_outline"] = server.scene.add_line_segments(
                            f"/stand/{arm}_base/ik_infeasible_outline",
                            points=outline,
                            colors=_IK_INFEASIBLE_EDGE_RED,
                            line_width=3.0,
                            visible=False,
                        )
                except Exception as exc:
                    handles[f"{arm}_ik_infeasible_outline_error"] = f"{type(exc).__name__}: {exc}"
            added = True
        if not added:
            handles["ik_infeasible_error"] = f"IK-infeasible asset empty: {path}"
            return
        handles["ik_infeasible_cells"] = total_cells
        if "radius_m" in data:
            handles["ik_infeasible_radius_m"] = float(data["radius_m"])
    except Exception as exc:
        handles["ik_infeasible_error"] = f"{type(exc).__name__}: {exc}"


def set_ik_infeasible_region_visible(scene_handles: dict[str, Any], visible: bool) -> None:
    """Show/hide both arms' IK-infeasible region meshes + rim outlines (GUI toggle)."""
    if not isinstance(scene_handles, dict):
        return
    for arm in ("left", "right"):
        _set_visible(scene_handles.get(f"{arm}_ik_infeasible"), visible)
        _set_visible(scene_handles.get(f"{arm}_ik_infeasible_outline"), visible)


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
                    color=_FLOOR_PLANE_GREEN,
                    opacity=0.25,
                    side="back",  # don't let the near face occlude geometry above it
                    position=(*_FLOOR_PLANE_CENTER_XY, 0.0),
                    visible=False,
                )
            except TypeError:  # older viser without opacity/side support
                handles["floor_plane"] = server.scene.add_box(
                    "/stand/floor_plane",
                    dimensions=_FLOOR_PLANE_DIMENSIONS,
                    color=_FLOOR_PLANE_GREEN,
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
                    side="back",  # don't let the near face occlude geometry above it
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
    # Emphasised boundary for the APPLIED plane: opaque edges (line segments) +
    # corner vertices (point cloud), drawn over the translucent fill like the ROI
    # box so the plane outline stays readable where the ROI region overlaps it.
    # update_floor_plane moves/recolors them with the fill; placeholder until then.
    seg, corners = _roi_box_outline(_FLOOR_PLANE_DIMENSIONS, (*_FLOOR_PLANE_CENTER_XY, 0.0))
    if hasattr(server.scene, "add_line_segments"):
        try:
            handles["floor_plane_edges"] = server.scene.add_line_segments(
                "/stand/floor_plane_edges", points=seg, colors=_FLOOR_PLANE_EDGE_GREEN,
                line_width=4.0, visible=False,
            )
        except Exception as exc:
            handles["floor_plane_edges_error"] = f"{type(exc).__name__}: {exc}"
    if hasattr(server.scene, "add_point_cloud"):
        try:
            handles["floor_plane_verts"] = server.scene.add_point_cloud(
                "/stand/floor_plane_verts", points=corners, colors=_FLOOR_PLANE_EDGE_GREEN,
                point_size=0.02, point_shape="circle", visible=False,
            )
        except Exception as exc:
            handles["floor_plane_verts_error"] = f"{type(exc).__name__}: {exc}"


def _apply_floor_plane_outline(
    scene_handles: dict[str, Any], z_m: float, edge_color: tuple[int, int, int]
) -> None:
    """Resize/recolor the floor-plane edge lines + corner vertices at plane height z."""
    import numpy as np

    seg, corners = _roi_box_outline(_FLOOR_PLANE_DIMENSIONS, (*_FLOOR_PLANE_CENTER_XY, float(z_m)))
    col = np.asarray(edge_color, dtype=np.uint8)
    for key, pts in (("floor_plane_edges", seg), ("floor_plane_verts", corners)):
        handle = scene_handles.get(key)
        if handle is None:
            continue
        for attr, value in (("points", pts), ("colors", col)):
            try:
                setattr(handle, attr, value)
            except Exception:
                pass
        _set_visible(handle, True)


def _hide_floor_plane_outline(scene_handles: dict[str, Any]) -> None:
    for key in ("floor_plane_edges", "floor_plane_verts"):
        _set_visible(scene_handles.get(key), False)


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
        _hide_floor_plane_outline(scene_handles)
        return
    z = floor.get("z_min_m")
    z_val = float(z) if isinstance(z, (int, float)) and math.isfinite(float(z)) else None
    if z_val is not None:
        try:
            # z is the stand-frame plane height (z=0 == stand origin plane).
            plane.position = (*_FLOOR_PLANE_CENTER_XY, z_val)
        except Exception:
            pass
    violated = any(
        isinstance(floor.get(key), Mapping) and bool(floor[key].get("violated", False))
        for key in ("left", "right")
    )
    try:
        plane.color = _FLOOR_PLANE_RED if violated else _FLOOR_PLANE_GREEN
    except Exception:
        pass
    _set_visible(plane, True)
    # Emphasised outline (edges + corner vertices) tracks the fill at plane height z;
    # red on violation, green otherwise. Hidden when z is unknown.
    if z_val is not None:
        _apply_floor_plane_outline(
            scene_handles, z_val,
            _FLOOR_PLANE_EDGE_RED if violated else _FLOOR_PLANE_EDGE_GREEN,
        )
    else:
        _hide_floor_plane_outline(scene_handles)


def _add_floor_check_points(server: Any, handles: dict[str, Any]) -> None:
    """Per-arm orange point cloud of the floor/ROI collision check samples
    (TCP point + configured TCP-frame offset points). Parented under each
    /stand/<arm>_tcp frame so the points ride the live TCP pose; hidden until
    the GUI toggle enables them. Static local geometry — only visibility ever
    changes after setup."""
    if not hasattr(server.scene, "add_point_cloud"):
        return
    import numpy as np

    pts = np.asarray(_FLOOR_CHECK_POINTS_TCP_FRAME, dtype=np.float32)
    col = np.tile(np.asarray(_FLOOR_CHECK_POINT_ORANGE, dtype=np.uint8), (pts.shape[0], 1))
    for arm in ("left", "right"):
        try:
            handles[f"{arm}_floor_check_points"] = server.scene.add_point_cloud(
                f"/stand/{arm}_tcp/floor_check_points",
                points=pts,
                colors=col,
                point_size=0.006,
                point_shape="circle",
                visible=False,
            )
        except Exception as exc:
            handles[f"{arm}_floor_check_points_error"] = f"{type(exc).__name__}: {exc}"


def update_floor_check_points(scene_handles: dict[str, Any], latest: Any, show: bool) -> None:
    """Show/hide the per-arm floor-check point clouds (GUI toggle).

    Each arm's points appear only when that arm has a valid actual TCP pose (the
    same gate the TCP frame itself uses), so stale orange dots never linger where
    FK is unavailable. The clouds are children of /stand/<arm>_tcp, so they track
    the TCP pose without any per-tick repositioning here.

    While visible, each arm's fingertip points are interpolated to the live gripper
    open percent (scene_handles["gripper_percent_<arm>"], set by _push_gripper_percent)
    so the orange dots open/close with the jaws — mirroring the server's
    interpolateOffsetPoints. Absent percent -> the OPEN set (conservative fallback)."""
    if not isinstance(scene_handles, dict):
        return
    for arm in ("left", "right"):
        handle = scene_handles.get(f"{arm}_floor_check_points")
        if handle is None:
            continue
        arm_state = getattr(latest, arm, None) if latest is not None else None
        actual_pose = None
        if arm_state is not None:
            actual_pose = getattr(arm_state, "tcp_actual_stand", None) or getattr(arm_state, "tcp_stand", None)
        has_tcp = bool(
            arm_state is not None
            and getattr(arm_state, "tcp_actual_valid", False)
            and actual_pose is not None
            and not getattr(arm_state, "tcp_deferred", True)
        )
        visible = bool(show) and has_tcp
        if visible:
            try:
                handle.points = _interpolated_floor_check_points(
                    scene_handles.get(f"gripper_percent_{arm}")
                )
            except Exception as exc:
                scene_handles[f"{arm}_floor_check_points_update_error"] = f"{type(exc).__name__}: {exc}"
        _set_visible(handle, visible)


def _wxyz_from_normal(normal: Any) -> tuple[float, float, float, float]:
    """Quaternion (w, x, y, z) rotating local +z onto the given (upward) normal,
    so a thin box rendered flat in its local z appears as the tilted plane."""
    import numpy as np

    n = np.asarray(normal, dtype=float)
    nn = float(np.linalg.norm(n))
    if nn < 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    n = n / nn
    c = float(np.clip(n[2], -1.0, 1.0))  # dot(+z, n)
    if c > 1.0 - 1e-9:
        return (1.0, 0.0, 0.0, 0.0)  # already +z
    if c < -1.0 + 1e-9:
        return (0.0, 1.0, 0.0, 0.0)  # 180deg about x (n.z > 0 makes this unreachable)
    axis = np.cross(np.array([0.0, 0.0, 1.0]), n)
    axis = axis / float(np.linalg.norm(axis))
    angle = math.acos(c)
    s = math.sin(angle / 2.0)
    return (math.cos(angle / 2.0), float(axis[0] * s), float(axis[1] * s), float(axis[2] * s))


def _add_user_floor_plane(server: Any, handles: dict[str, Any]) -> None:
    """User Safety Floor visual: a tilted translucent plane + captured-point cloud.
    Hidden until update_user_floor_plane reports the constraint enabled."""
    try:
        if hasattr(server.scene, "add_box"):
            try:
                handles["user_floor_plane"] = server.scene.add_box(
                    "/stand/user_floor_plane",
                    dimensions=_USER_FLOOR_PLANE_DIMENSIONS,
                    color=_USER_FLOOR_PLANE_PURPLE,
                    opacity=0.25,
                    side="back",
                    position=(*_FLOOR_PLANE_CENTER_XY, 0.0),
                    visible=False,
                )
            except TypeError:  # older viser without opacity/side support
                handles["user_floor_plane"] = server.scene.add_box(
                    "/stand/user_floor_plane",
                    dimensions=_USER_FLOOR_PLANE_DIMENSIONS,
                    color=_USER_FLOOR_PLANE_PURPLE,
                    position=(*_FLOOR_PLANE_CENTER_XY, 0.0),
                    visible=False,
                )
    except Exception as exc:
        handles["user_floor_plane_error"] = f"{type(exc).__name__}: {exc}"
    # Captured contact-point markers (point cloud). Starts empty/hidden.
    try:
        import numpy as np

        if hasattr(server.scene, "add_point_cloud"):
            handles["user_floor_points"] = server.scene.add_point_cloud(
                "/stand/user_floor_points",
                points=np.zeros((1, 3), dtype=np.float32),
                colors=np.array([_USER_FLOOR_POINT_LEFT], dtype=np.uint8),
                point_size=0.012,
                visible=False,
            )
    except Exception as exc:
        handles["user_floor_points_error"] = f"{type(exc).__name__}: {exc}"
    # Emphasised boundary (opaque edges + corner vertices), like the stand floor.
    # The outline geometry is LOCAL (box centered at the node origin); the node's
    # position + wxyz (set in update_user_floor_plane, same as the fill box) tilt it
    # onto the fitted plane. Placeholder geometry until then.
    seg, corners = _roi_box_outline(_USER_FLOOR_PLANE_DIMENSIONS, (0.0, 0.0, 0.0))
    if hasattr(server.scene, "add_line_segments"):
        try:
            handles["user_floor_edges"] = server.scene.add_line_segments(
                "/stand/user_floor_edges", points=seg, colors=_USER_FLOOR_EDGE_PURPLE,
                line_width=4.0, visible=False,
            )
        except Exception as exc:
            handles["user_floor_edges_error"] = f"{type(exc).__name__}: {exc}"
    if hasattr(server.scene, "add_point_cloud"):
        try:
            handles["user_floor_verts"] = server.scene.add_point_cloud(
                "/stand/user_floor_verts", points=corners, colors=_USER_FLOOR_EDGE_PURPLE,
                point_size=0.02, point_shape="circle", visible=False,
            )
        except Exception as exc:
            handles["user_floor_verts_error"] = f"{type(exc).__name__}: {exc}"


def _apply_user_floor_outline(
    scene_handles: dict[str, Any],
    pos: tuple[float, ...],
    wxyz: tuple[float, ...],
    edge_color: tuple[int, int, int],
) -> None:
    """Place/orient/recolor the user-floor edge lines + corner vertices.

    The outline nodes carry LOCAL box geometry; setting their position + wxyz (the
    same transform as the tilted fill box) lands them on the fitted plane without
    re-baking the rotated points."""
    import numpy as np

    col = np.asarray(edge_color, dtype=np.uint8)
    for key in ("user_floor_edges", "user_floor_verts"):
        handle = scene_handles.get(key)
        if handle is None:
            continue
        for attr, value in (("position", pos), ("wxyz", wxyz), ("colors", col)):
            try:
                setattr(handle, attr, value)
            except Exception:
                pass
        _set_visible(handle, True)


def _hide_user_floor_outline(scene_handles: dict[str, Any]) -> None:
    for key in ("user_floor_edges", "user_floor_verts"):
        _set_visible(scene_handles.get(key), False)


def update_user_floor_plane(
    scene_handles: dict[str, Any], user_floor: Mapping[str, Any] | None
) -> None:
    """Move/orient/recolor the user floor plane from the published
    user_floor_constraint block (point_m + normal + per-arm violated)."""
    plane = scene_handles.get("user_floor_plane") if isinstance(scene_handles, dict) else None
    if plane is None:
        return
    if not isinstance(user_floor, Mapping) or not bool(user_floor.get("enabled", False)):
        _set_visible(plane, False)
        _hide_user_floor_outline(scene_handles)
        return
    point = user_floor.get("point_m")
    normal = user_floor.get("normal")
    if (
        not isinstance(point, (list, tuple)) or len(point) != 3
        or not isinstance(normal, (list, tuple)) or len(normal) != 3
    ):
        _set_visible(plane, False)
        _hide_user_floor_outline(scene_handles)
        return
    wxyz = _wxyz_from_normal(normal)
    try:
        pos = tuple(float(v) for v in point)
    except (TypeError, ValueError):
        _set_visible(plane, False)
        _hide_user_floor_outline(scene_handles)
        return
    if not all(math.isfinite(v) for v in pos):
        _set_visible(plane, False)
        _hide_user_floor_outline(scene_handles)
        return
    try:
        plane.position = pos
        plane.wxyz = wxyz
    except Exception:
        pass
    violated = any(
        isinstance(user_floor.get(key), Mapping) and bool(user_floor[key].get("violated", False))
        for key in ("left", "right")
    )
    try:
        plane.color = _USER_FLOOR_PLANE_RED if violated else _USER_FLOOR_PLANE_PURPLE
    except Exception:
        pass
    _set_visible(plane, True)
    # Outline tracks the same tilt/position as the fill; recolor red on violation.
    _apply_user_floor_outline(
        scene_handles, pos, wxyz,
        _USER_FLOOR_EDGE_RED if violated else _USER_FLOOR_EDGE_PURPLE,
    )


def update_user_floor_capture_points(
    scene_handles: dict[str, Any], points: Any
) -> None:
    """Refresh the captured floor-contact point cloud (left=cyan, right=magenta)."""
    cloud = scene_handles.get("user_floor_points") if isinstance(scene_handles, dict) else None
    if cloud is None:
        return
    import numpy as np

    coords: list[list[float]] = []
    colors: list[tuple[int, int, int]] = []
    for entry in points or []:
        if not isinstance(entry, Mapping):
            continue
        p = entry.get("p")
        if not isinstance(p, (list, tuple)) or len(p) != 3:
            continue
        try:
            xyz = [float(v) for v in p]
        except (TypeError, ValueError):
            continue
        if not all(math.isfinite(v) for v in xyz):
            continue
        coords.append(xyz)
        colors.append(_USER_FLOOR_POINT_RIGHT if entry.get("arm") == "right" else _USER_FLOOR_POINT_LEFT)
    if not coords:
        _set_visible(cloud, False)
        return
    try:
        cloud.points = np.asarray(coords, dtype=np.float32)
        cloud.colors = np.asarray(colors, dtype=np.uint8)
        _set_visible(cloud, True)
    except Exception:
        pass


# Stand-frame ROI box (safety.roi_box) visual: a translucent box the TCP must
# stay inside. Applied box = blue (red when an arm is outside); pending-slider
# preview = yellow, like the floor plane preview.
_ROI_BOX_RED = (220, 60, 60)
# ROI region edge colors. The region is drawn as an outline ONLY (edges + corner
# vertices) — no filled face: viser keeps depthWrite=true even on transparent
# meshes, so a translucent ROI box depth-occludes the robot URDF (the solid arm
# outside the region, the translucent red self-collision overlay) wherever the
# region overlaps it. The wireframe marks the region without any occluding face.
# Recolors red on violation. Applied region is blue; pending preview is yellow.
_ROI_BOX_EDGE_BLUE = (150, 200, 255)
_ROI_BOX_EDGE_RED = (255, 120, 120)
_ROI_BOX_EDGE_PREVIEW_YELLOW = (235, 200, 60)


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
    *,
    edges_key: str = "roi_box_edges",
    verts_key: str = "roi_box_verts",
) -> None:
    """Resize/recolor a ROI region outline (edge lines + corner vertices)."""
    import numpy as np

    seg, corners = _roi_box_outline(dims, center)
    col = np.asarray(edge_color, dtype=np.uint8)
    edges = scene_handles.get(edges_key)
    if edges is not None:
        for attr, value in (("points", seg), ("colors", col)):
            try:
                setattr(edges, attr, value)
            except Exception:
                pass
        _set_visible(edges, True)
    verts = scene_handles.get(verts_key)
    if verts is not None:
        for attr, value in (("points", corners), ("colors", col)):
            try:
                setattr(verts, attr, value)
            except Exception:
                pass
        _set_visible(verts, True)


def _hide_roi_box_outline(
    scene_handles: dict[str, Any],
    *,
    edges_key: str = "roi_box_edges",
    verts_key: str = "roi_box_verts",
) -> None:
    for key in (edges_key, verts_key):
        _set_visible(scene_handles.get(key), False)


def _add_roi_box(server: Any, handles: dict[str, Any]) -> None:
    """Stand-frame ROI region (safety.roi_box visual): applied outline + preview.

    Rendered as an OUTLINE ONLY — opaque edge lines (line segments) + corner
    vertices (point cloud), no filled face. A translucent fill would depth-occlude
    the robot URDF wherever the region overlaps it (viser keeps depthWrite=true on
    transparent meshes), hiding both the solid arm outside the region and the
    translucent red self-collision overlay. The wireframe marks the region without
    any occluding face. Hidden (placeholder geometry) until update_roi_box /
    update_roi_box_preview move/resize/show it."""
    dims = (1.0, 1.0, 1.0)
    center = (0.0, -0.5, 0.5)
    seg, corners = _roi_box_outline(dims, center)
    # (edges handle, verts handle, scene-node prefix, color) for the applied
    # region (blue) and the pending-slider preview (yellow).
    for edges_key, verts_key, prefix, color in (
        ("roi_box_edges", "roi_box_verts", "/stand/roi_box", _ROI_BOX_EDGE_BLUE),
        ("roi_box_preview_edges", "roi_box_preview_verts", "/stand/roi_box_preview",
         _ROI_BOX_EDGE_PREVIEW_YELLOW),
    ):
        if hasattr(server.scene, "add_line_segments"):
            try:
                handles[edges_key] = server.scene.add_line_segments(
                    f"{prefix}_edges", points=seg, colors=color,
                    line_width=4.0, visible=False,
                )
            except Exception as exc:
                handles[f"{edges_key}_error"] = f"{type(exc).__name__}: {exc}"
        if hasattr(server.scene, "add_point_cloud"):
            try:
                handles[verts_key] = server.scene.add_point_cloud(
                    f"{prefix}_verts", points=corners, colors=color,
                    point_size=0.018, point_shape="circle", visible=False,
                )
            except Exception as exc:
                handles[f"{verts_key}_error"] = f"{type(exc).__name__}: {exc}"


def update_roi_box_preview(
    scene_handles: dict[str, Any], min_m: Any, max_m: Any
) -> None:
    """Show the pending slider bounds as a yellow outline preview.

    min_m/max_m=None (or malformed) hides the preview."""
    if not isinstance(scene_handles, dict):
        return
    geom = _roi_box_geometry(min_m, max_m) if min_m is not None and max_m is not None else None
    if geom is None:
        _hide_roi_box_outline(
            scene_handles, edges_key="roi_box_preview_edges", verts_key="roi_box_preview_verts",
        )
        return
    dims, center = geom
    _apply_roi_box_outline(
        scene_handles, dims, center, _ROI_BOX_EDGE_PREVIEW_YELLOW,
        edges_key="roi_box_preview_edges", verts_key="roi_box_preview_verts",
    )


def update_roi_box(
    scene_handles: dict[str, Any], roi: Mapping[str, Any] | None, *, visible: bool = True
) -> None:
    """Move/resize/recolor the ROI region outline from the published roi_box block.

    Drawn whenever `visible` (the GUI "ROI 영역 표시" toggle) is set and the
    published bounds are valid — independent of server enforcement (enabled), so
    the configured region stays a visible reference by default. Red when an arm
    is reported outside the box, blue otherwise. Outline only (no filled face) so
    it never occludes the robot URDF where the region overlaps it."""
    if not isinstance(scene_handles, dict):
        return
    if not visible or not isinstance(roi, Mapping):
        _hide_roi_box_outline(scene_handles)
        return
    geom = _roi_box_geometry(roi.get("min_m"), roi.get("max_m"))
    if geom is None:
        _hide_roi_box_outline(scene_handles)
        return
    dims, center = geom
    violated = any(
        isinstance(roi.get(key), Mapping) and bool(roi[key].get("violated", False))
        for key in ("left", "right")
    )
    _apply_roi_box_outline(
        scene_handles, dims, center,
        _ROI_BOX_EDGE_RED if violated else _ROI_BOX_EDGE_BLUE,
    )


def update_self_collision_overlay(scene_handles: dict[str, Any], latest: Any) -> None:
    """Paint the colliding PAIR translucent red while self_collision.violated.

    Driven by the server's self_collision telemetry, so monitor_only runs show
    the overlay too. Only the members of the reported pair turn red
    ("left_right" -> both arms, "left_stand"/"right_stand" -> that arm + the
    stand); a violated state without a recognizable pair falls back to all-red
    (conservative). pgmode real (physical_motion_expected=True): the ACTUAL
    robot (q_actual) turns red. pgmode simulation: the commanded ghost (q_sent)
    turns red while the solid robot keeps showing the true (stationary) state.
    External-box collision telemetry is per-box only in this first pass, so any
    box collision turns both arms red."""
    if not isinstance(scene_handles, dict):
        return
    sc = getattr(latest, "self_collision", None) if latest is not None else None
    violated = isinstance(sc, Mapping) and bool(sc.get("violated", False))
    box_collision = _external_box_collision(sc)
    _update_self_collision_witness_markers(scene_handles, sc, violated)
    pair = sc.get("pair") if isinstance(sc, Mapping) else None
    if violated and pair not in ("left_right", "left_stand", "right_stand"):
        pair = "all"  # unknown/legacy pair info: keep the conservative all-red
    left_red = box_collision or (violated and pair in ("left_right", "left_stand", "all"))
    right_red = box_collision or (violated and pair in ("left_right", "right_stand", "all"))
    stand_red = violated and pair in ("left_stand", "right_stand", "all")
    physical_real = latest is not None and (
        getattr(latest.left, "physical_motion_expected", None) is True
        or getattr(latest.right, "physical_motion_expected", None) is True
    )

    if violated or box_collision:
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
    if (violated or box_collision) and not physical_real:
        if left_red:
            _set_visible(scene_handles.get("left_base_ref"), False)
        if right_red:
            _set_visible(scene_handles.get("right_base_ref"), False)


def _external_box_collision(sc: Mapping[str, Any] | None) -> bool:
    clearances = sc.get("external_box_clearance_m") if isinstance(sc, Mapping) else None
    if not isinstance(clearances, (list, tuple)):
        return False
    return any(
        isinstance(c, (int, float))
        and math.isfinite(float(c))
        and float(c) <= EXTERNAL_BOX_COLLISION_M
        for c in clearances
    )


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
# EXTERNAL pairs (arm<->floor/ground_plane) get a distinct blue/cyan palette so the
# operator can tell the floor barrier apart from robot self-collision, and are banded
# against the external d_hard/d_slow (which differ from the self thresholds).
_EXTERNAL_NEAR_HARD_RGB = (190, 60, 235)            # clearance < external d_hard (violet)
_EXTERNAL_NEAR_CAUTION_RGB = (60, 200, 235)         # external [d_hard, d_slow) (cyan)
_EXTERNAL_NEAR_OK_RGB = (60, 110, 235)              # clearance >= external d_slow (blue)
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
    A thin tube spans each checked pair's witness points, colored by clearance band.
    SELF pairs (robot<->robot/stand): red < d_hard, amber in [d_hard, d_slow), green
    >= d_slow. EXTERNAL pairs (arm<->floor/ground_plane, pair.external=true): a distinct
    violet/cyan/blue palette banded against the external d_hard/d_slow. Driven by the
    COMMANDED (q_sent) verdict the server publishes, so it mirrors what the guard checks
    even when the displayed q_actual diverges."""
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
            # EXTERNAL (floor) pairs band against their own d_hard/d_slow.
            ext_hard = _finite_or(manifest.get("external_d_hard_m"), 0.003)
            ext_slow = _finite_or(manifest.get("external_d_slow_m"), max(ext_hard, 0.025))
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
                    external = bool(pair.get("external", False))
                    # Per-category thresholds + palette: floor pairs are violet/cyan/blue
                    # banded against the external d_hard/d_slow; self pairs red/amber/green.
                    d_hard, d_slow = (ext_hard, ext_slow) if external else (hard, slow)
                    if external:
                        palette = (_EXTERNAL_NEAR_HARD_RGB, _EXTERNAL_NEAR_CAUTION_RGB, _EXTERNAL_NEAR_OK_RGB)
                        kind = "ext"
                    else:
                        palette = (_SELF_COLLISION_NEAR_HARD_RGB, _SELF_COLLISION_NEAR_CAUTION_RGB, _SELF_COLLISION_NEAR_OK_RGB)
                        kind = "self"
                    if c < d_hard:
                        band, rgb = "hard", palette[0]
                    elif c < d_slow:
                        band, rgb = "caution", palette[1]
                    else:
                        band, rgb = "ok", palette[2]
                    key = f"{i}_{kind}_{band}"
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
    the safety URDF, parenting the gripper geometry under the unified overlay's
    "<prefix>attachment_site" link frame so it shows as part of the checked geometry.

    Two modes, matching the server (collision_monitor.cpp):
      * ARTICULATED (manifest has base + both finger meshes): a static base mesh +
        two finger meshes at IDENTITY (no Z+90deg), the fingers repositioned along
        local +X by the live jaw percent in update_self_collision_check_geom.
      * SINGLE HULL (pika_gripper_mesh only): the legacy static hull with Z+90deg.
    Frames are located via ViserUrdf's per-joint frames; skips gracefully if absent."""
    server = scene_handles.get("_server")
    if server is None:
        return
    attach = manifest.get("gripper_attach") if isinstance(manifest.get("gripper_attach"), Mapping) else {}
    scale = float(attach.get("mesh_scale", 0.001))
    suffix = str(attach.get("frame_suffix", "attachment_site"))
    rgb = tuple(int(round(c * 255)) for c in _SELF_COLLISION_CHECK_RGBA[:3])
    opacity = float(_SELF_COLLISION_CHECK_RGBA[3])

    def _exists(p: Any) -> bool:
        return isinstance(p, str) and bool(p) and Path(p).exists()

    base_path = manifest.get("pika_gripper_base_mesh")
    fl_path = manifest.get("pika_finger_left_mesh")
    fr_path = manifest.get("pika_finger_right_mesh")
    articulated = _exists(base_path) and _exists(fl_path) and _exists(fr_path)
    hull_path = manifest.get("pika_gripper_mesh")
    if not articulated and not _exists(hull_path):
        return
    try:
        import trimesh

        frames = list(getattr(overlay, "_joint_frames", []) or [])

        def _find_frame(prefix: str) -> Any:
            link = f"{prefix}{suffix}"
            f = next((f for f in frames if str(getattr(f, "name", "")).endswith("/" + link)), None)
            if f is None:
                scene_handles["checkgeom_gripper_error"] = f"attachment frame not found: {link}"
            return f

        def _load(path: str, ident_no_rot: bool) -> Any:
            mesh = trimesh.load_mesh(path)
            mesh.apply_scale(scale)
            return mesh

        arms = (
            (manifest.get("left_prefix", ""), "left"),
            (manifest.get("right_prefix", ""), "right"),
        )
        handles: list[Any] = []
        if articulated:
            base_mesh = _load(base_path, True)
            fl_mesh = _load(fl_path, True)
            fr_mesh = _load(fr_path, True)
            ident = _pose_wxyz((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))  # no Z+90 for base/fingers
            fingers_by_arm: dict[str, dict[str, Any]] = {}
            for prefix, arm_name in arms:
                frame = _find_frame(prefix)
                if frame is None:
                    continue
                base_h = server.scene.add_mesh_simple(
                    f"{frame.name}/pika_gripper_base", vertices=base_mesh.vertices,
                    faces=base_mesh.faces, color=rgb, opacity=opacity, wxyz=ident,
                    position=(0.0, 0.0, 0.0), visible=False)
                lh = server.scene.add_mesh_simple(
                    f"{frame.name}/pika_finger_left", vertices=fl_mesh.vertices,
                    faces=fl_mesh.faces, color=rgb, opacity=opacity, wxyz=ident,
                    position=(0.0, 0.0, 0.0), visible=False)
                rh = server.scene.add_mesh_simple(
                    f"{frame.name}/pika_finger_right", vertices=fr_mesh.vertices,
                    faces=fr_mesh.faces, color=rgb, opacity=opacity, wxyz=ident,
                    position=(0.0, 0.0, 0.0), visible=False)
                handles.extend((base_h, lh, rh))
                fingers_by_arm[arm_name] = {"left": lh, "right": rh}
            scene_handles["checkgeom_gripper_fingers"] = fingers_by_arm
            scene_handles["checkgeom_finger_travel"] = float(
                manifest.get("gripper_finger_travel_m", _GRIPPER_FINGER_TRAVEL_M))
        else:
            rpy = attach.get("rpy") or [0.0, 0.0, math.pi / 2.0]
            wxyz = _pose_wxyz((0.0, 0.0, 0.0, float(rpy[0]), float(rpy[1]), float(rpy[2])))
            hull = _load(hull_path, False)
            for prefix, _arm_name in arms:
                frame = _find_frame(prefix)
                if frame is None:
                    continue
                handles.append(server.scene.add_mesh_simple(
                    f"{frame.name}/pika_gripper", vertices=hull.vertices, faces=hull.faces,
                    color=rgb, opacity=opacity, wxyz=wxyz, position=(0.0, 0.0, 0.0), visible=False))
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
                # cfg is already built in the unified URDF's actuated-joint order
                # (prefixed names, all 12 arm joints) by _checkgeom_joint_config, so
                # apply it DIRECTLY. Do NOT route it through _update_urdf_config: that
                # helper re-maps via the 6 short _ROBOT_JOINT_NAMES, which never match
                # the unified URDF's prefixed joints -> every joint falls back to 0
                # and the overlay freezes fully extended (it never tracks q_sent/q_actual).
                _apply_urdf_cfg_direct(overlay, cfg)
            except Exception as exc:
                scene_handles["urdf_checkgeom_update_error"] = f"{type(exc).__name__}: {exc}"
    try:
        overlay.show_collision = show
    except Exception as exc:
        scene_handles["urdf_checkgeom_show_error"] = f"{type(exc).__name__}: {exc}"
    # The gripper meshes are independent child handles (not part of show_collision).
    for handle in scene_handles.get("checkgeom_gripper", []):
        _set_visible(handle, show)
    # Articulated gripper: slide each arm's two finger meshes along local +X by the live
    # jaw percent, mirroring the server's setGripperOpenPercent (finger_pos 0 at OPEN ->
    # travel at CLOSED; left +X, right -X). Uses gripper_percent_<arm> from _push_gripper_percent.
    fingers = scene_handles.get("checkgeom_gripper_fingers")
    if show and isinstance(fingers, dict):
        travel = float(scene_handles.get("checkgeom_finger_travel", _GRIPPER_FINGER_TRAVEL_M))
        for arm_name, hd in fingers.items():
            pct = scene_handles.get(f"gripper_percent_{arm_name}")
            try:
                t = max(0.0, min(100.0, float(pct))) / 100.0 if pct is not None else 1.0
            except (TypeError, ValueError):
                t = 1.0
            pos = (1.0 - t) * travel
            try:
                hd["left"].position = (pos, 0.0, 0.0)
                hd["right"].position = (-pos, 0.0, 0.0)
            except Exception:
                pass
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
        # Paint the stand the RB3-730E dark gray (STL carries no color, so trimesh
        # otherwise defaults to a lighter (102, 102, 102) gray).
        try:
            mesh.visual.face_colors = (*_RB3_DARK_GRAY_RGB, 255)
        except Exception as exc:
            handles["stand_mesh_color_error"] = _asset_error(f"{type(exc).__name__}: {exc}")
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


def _apply_urdf_cfg_direct(urdf_handle: Any, cfg_radians: Any) -> None:
    """Push a config that is ALREADY in the handle's actuated-joint order/length
    straight to ViserUrdf.update_cfg (no _ROBOT_JOINT_NAMES re-mapping). Used by the
    unified collision-hull overlay, whose actuated joints are the prefixed
    "<prefix>+<base>" names that _checkgeom_joint_config orders the cfg by."""
    try:
        import numpy as np

        payload: Any = np.array(cfg_radians, dtype=float)
    except Exception:
        payload = list(cfg_radians)
    urdf_handle.update_cfg(payload)


def _update_urdf_config(
    urdf_handle: Any, cfg_radians: tuple[float, ...], gripper_percent: float | None = None
) -> None:
    """Drive a ViserUrdf. cfg_radians is the 6 arm-joint values (in
    _ROBOT_JOINT_NAMES order); gripper_percent (0..100) drives the prismatic finger
    joints IF this URDF is the articulated variant. Works for both the plain
    6-joint URDF and the 8-joint articulated one: the config is rebuilt in the
    handle's actuated-joint order, so unknown joints default to 0 and the finger
    joints are filled only when present."""
    try:
        names = tuple(urdf_handle.get_actuated_joint_names())
    except Exception:
        names = ()
    arm_map = dict(zip(_ROBOT_JOINT_NAMES, cfg_radians))
    if names and (set(names) - set(_ROBOT_JOINT_NAMES)):
        finger_pos = _finger_position_m(gripper_percent)
        values = [
            arm_map.get(name, _GRIPPER_FINGER_JOINT_SIGN.get(name, 0.0) * finger_pos)
            for name in names
        ]
    else:
        values = list(cfg_radians)  # plain URDF: arm joints only, original behavior
    try:
        import numpy as np

        payload: Any = np.array(values)
    except Exception:
        payload = values
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
        _trail_as_line = hasattr(server.scene, "add_line_segments")
        handles["tcp_trail_mode"] = "line" if _trail_as_line else "point"
        if _trail_as_line or hasattr(server.scene, "add_point_cloud"):
            for arm, actual_color, ref_color in (
                ("left", (80, 160, 255), (60, 210, 110)),
                ("right", (255, 160, 80), (60, 210, 110)),
            ):
                handles[f"{arm}_tcp_trail_points"] = []
                handles[f"{arm}_tcp_ref_trail_points"] = []
                for key in (f"{arm}_tcp_trail", f"{arm}_tcp_ref_trail"):
                    if _trail_as_line:
                        handles[key] = server.scene.add_line_segments(
                            f"/stand/{key}",
                            points=_line_segments_array(),
                            colors=_line_segment_colors_array(),
                            line_width=3.5,
                        )
                    else:
                        handles[key] = server.scene.add_point_cloud(
                            f"/stand/{key}",
                            points=_points_array(),
                            colors=_colors_array(),
                            point_size=0.012,
                            point_shape="rounded",
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
        _add_floor_check_points(server, handles)
        _add_roi_box(server, handles)
        _add_user_floor_plane(server, handles)
        _add_reachability_cloud(server, handles)
        _add_ik_infeasible_region(server, handles)
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
        if scene_handles.get("tcp_trail_mode") == "line":
            segments, seg_colors = _trail_line_arrays(points, color)
            handle.points = segments
            handle.colors = seg_colors
            handle.visible = visible and len(points) >= 2
        else:
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
        # Gripper open percentage drives the prismatic fingers on the articulated
        # URDF (no-op on the plain URDF). Source is published gripper state if
        # present, else the GUI "Gripper %" preview slider (written into
        # scene_handles by the app each update); None leaves the fingers open.
        side = "left" if key == "left_urdf" else "right"
        gripper_pct = scene_handles.get(f"gripper_percent_{side}")
        try:
            _update_urdf_config(urdf_handle, _joint_cfg_radians(q), gripper_percent=gripper_pct)
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
