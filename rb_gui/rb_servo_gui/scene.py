from __future__ import annotations

import importlib.util
import math
import os
from pathlib import Path
from typing import Any, Mapping

from .geometry import (
    _matrix_to_wxyz,
    _mount_pose_from_mounts,
    _multiply_transform,
    _normalize_wxyz,
    _pose6_from_transform,
    _pose_orientation_wxyz,
    _pose_position,
    _pose_transform,
    _pose_wxyz,
    _rpy_to_wxyz,
)
from .models import EXTERNAL_BOX_COLLISION_M, ChunkOverlaySnapshot, CircleOverlaySnapshot

_ROBOT_JOINT_NAMES = (
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist1_joint",
    "wrist2_joint",
    "wrist3_joint",
)
# Rotation values are canonical URDF/ROS RPY converted from MJCF euler xyz.
# Startup fallbacks only: the live mounts arrive in the server's state manifest and
# _mount_pose_from_mounts() prefers those. Kept in step with stack_real.yaml so the
# first frames before state arrives are not visibly wrong. RB5-850E as of 2026-09-02
# -- note the arms mirror in Y now, where RB3 mirrored in X.
# 2026-09-06: upstream stand ver2 -> ver1 (mounts 15.00 mm in along the 45 deg plate;
# the cell's stand is ver1, measured -- see tools/make_rb5_850e_urdfs.py).
_DEFAULT_LEFT_POSE = (0.16285534, 0.18646447, 0.56285534, 2.185914, 0.523132, -2.186649)
_DEFAULT_RIGHT_POSE = (0.16285534, -0.18646447, 0.56285534, 2.185914, -0.523132, -0.954944)
# LAST-RESORT fallback only; the live value comes from the unified URDF's stand
# visual origin via _stand_mesh_pose(). This is dual_rb3_730e_ver5's value, kept so a
# missing/unparsable URDF still renders something rather than nothing.
_DEFAULT_STAND_MESH_POSE = (0.0, 0.0, 0.001, 0.0, 0.0, -1.57078)
_TCP_DISPLAY_MODES = ("auto", "actual", "reference", "both")
_TCP_TRAIL_LIMIT = 600
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


def _chunk_overlay_point_colors(count: int, base_color: tuple[int, int, int]) -> Any:
    import numpy as np

    if count <= 0:
        return _colors_array()
    base = np.asarray(base_color, dtype=np.float32)
    lo, hi = 0.10, 1.0
    t = np.linspace(0.0, 1.0, count, dtype=np.float32) ** 1.6
    fade = lo + (hi - lo) * t
    return np.clip(base[None, :] * fade[:, None], 0.0, 255.0).astype(np.uint8)


def _chunk_overlay_history_line_arrays(
    overlays: list[ChunkOverlaySnapshot] | tuple[ChunkOverlaySnapshot, ...] | None,
    arm: str,
) -> tuple[Any, Any]:
    valid_positions: list[tuple[tuple[float, float, float], ...]] = []
    for overlay in overlays or ():
        positions = getattr(overlay, f"{arm}_positions", None)
        if positions and len(positions) >= 2:
            valid_positions.append(tuple(positions))
    if not valid_positions:
        return _line_segments_array(), _line_segment_colors_array()

    base_color = (255, 140, 40)
    lo, hi = 0.22, 0.80
    denom = max(1, len(valid_positions) - 1)
    segments = []
    colors = []
    for index, positions in enumerate(valid_positions):
        brightness = hi if len(valid_positions) == 1 else lo + (hi - lo) * index / denom
        color = tuple(max(0, min(255, int(component * brightness))) for component in base_color)
        for point_index in range(len(positions) - 1):
            segments.append((positions[point_index], positions[point_index + 1]))
            colors.append((color, color))
    return _line_segments_array(segments), _line_segment_colors_array(colors)


def _rotate_vector_by_wxyz(
    wxyz: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    w, x, y, z = _normalize_wxyz(wxyz)
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _pose_triad_segments(
    pose7: tuple[float, float, float, float, float, float, float],
    length: float,
) -> tuple[Any, Any]:
    origin = (float(pose7[0]), float(pose7[1]), float(pose7[2]))
    axis_length = float(length)
    wxyz = (float(pose7[6]), float(pose7[3]), float(pose7[4]), float(pose7[5]))
    axes = (
        ((1.0, 0.0, 0.0), (224, 36, 36)),
        ((0.0, 1.0, 0.0), (31, 157, 58)),
        ((0.0, 0.0, 1.0), (34, 102, 238)),
    )
    segments = []
    colors = []
    for axis, color in axes:
        direction = _rotate_vector_by_wxyz(wxyz, axis)
        end = (
            origin[0] + direction[0] * axis_length,
            origin[1] + direction[1] * axis_length,
            origin[2] + direction[2] * axis_length,
        )
        segments.append((origin, end))
        colors.append((color, color))
    return _line_segments_array(tuple(segments)), _line_segment_colors_array(tuple(colors))


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
    # the viewer can show continuous open/close; fall back to the plain arm model if
    # that asset is absent. RB_GUI_ROBOT_URDF still forces a specific file. The C++
    # Pinocchio FK/IK is unaffected and must stay that way -- it loads rb5_850e.urdf
    # via kinematics.urdf, and the two extra prismatic joints here would change its
    # DOF. Both are generated by rb_servo_server/tools/make_rb5_850e_urdfs.py from the
    # same source, so the arm chains cannot drift apart (asserted at 0.000e+00).
    configured = os.environ.get("RB_GUI_ROBOT_URDF")
    if configured:
        return Path(configured)
    articulated = _descriptions_dir() / "urdf/rb5_850e_pika_articulated.urdf"
    if articulated.exists():
        return articulated
    return _descriptions_dir() / "urdf/rb5_850e.urdf"


def _unified_urdf_path() -> Path:
    """The stand+both-arms URDF the servo server enforces collisions against.

    rb_gui renders the stand from the SAME file so the two cannot disagree. The server
    also publishes this path in its state manifest; this is the startup default, used
    before any state has arrived.
    """
    return _asset_path("RB_GUI_UNIFIED_URDF", "urdf/dual_rb5_850e_ver3.urdf")


def _stand_visual_from_urdf(urdf_path: Path) -> tuple[Path, tuple] | None:
    """Read the stand link's display mesh and its origin out of the unified URDF.

    Both used to be constants in this module, copied from dual_rb3_730e_ver5: the mesh
    filename, and _DEFAULT_STAND_MESH_POSE = (0, 0, 0.001, 0, 0, -1.57078), which is
    that URDF's stand visual origin verbatim. RB3 needed the -90 deg because its own
    base->stand joint is +90 deg; RB5's base->stand is identity and so is its stand
    visual origin. Carrying the RB3 constant across the swap therefore rotated the
    rendered stand 90 deg away from the arms -- reported from hardware 2026-09-02.
    Reading it makes the next swap a no-op here.
    """
    import xml.etree.ElementTree as ET

    try:
        root = ET.parse(urdf_path).getroot()
    except Exception:
        return None
    for link in root.findall("link"):
        if link.get("name") != "stand":
            continue
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None or not mesh.get("filename"):
                continue
            origin = visual.find("origin")
            xyz = [float(v) for v in (origin.get("xyz") if origin is not None else "0 0 0").split()]
            rpy = [float(v) for v in (origin.get("rpy") if origin is not None else "0 0 0").split()]
            return (urdf_path.parent / mesh.get("filename")).resolve(), (*xyz, *rpy)
    return None


def _stand_mesh_path() -> Path:
    configured = os.environ.get("RB_GUI_STAND_MESH")
    if configured:
        return Path(configured)
    found = _stand_visual_from_urdf(_unified_urdf_path())
    if found is not None:
        return found[0]
    return _descriptions_dir() / "meshes/stands/dual_rb5_850e/dual_rb5_850e_stand_ver1.stl"


def _stand_mesh_pose() -> tuple:
    found = _stand_visual_from_urdf(_unified_urdf_path())
    return found[1] if found is not None else _DEFAULT_STAND_MESH_POSE


def _urdf_stand_to_world_pose(urdf_path: Path) -> tuple:
    """Pose of the URDF's ROOT frame expressed in its `stand` frame.

    The async monitor publishes witness points in the pinocchio WORLD frame, while
    every scene node hangs off /stand, so the overlay needs world-in-stand. This was
    the constant -90 deg about Z, which is only correct for dual_rb3_730e_ver5 (whose
    base->stand joint is +90 deg). RB5's is identity, so the constant rotated the whole
    collision overlay away from the arms. Composing the fixed-joint chain from `stand`
    up to the root and inverting it works for either.
    """
    import xml.etree.ElementTree as ET

    import numpy as np

    def rpy_matrix(rx: float, ry: float, rz: float):
        cx, sx, cy, sy, cz, sz = (
            math.cos(rx), math.sin(rx), math.cos(ry),
            math.sin(ry), math.cos(rz), math.sin(rz))
        return (np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
                @ np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
                @ np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]]))

    try:
        root = ET.parse(urdf_path).getroot()
        parent_of: dict[str, tuple] = {}
        for joint in root.findall("joint"):
            child = joint.find("child")
            parent = joint.find("parent")
            if child is None or parent is None:
                continue
            origin = joint.find("origin")
            xyz = [float(v) for v in (origin.get("xyz") if origin is not None else "0 0 0").split()]
            rpy = [float(v) for v in (origin.get("rpy") if origin is not None else "0 0 0").split()]
            parent_of[child.get("link")] = (parent.get("link"), xyz, rpy)

        # Compose root <- stand by walking up, so `t` ends as root_T_stand.
        rot = np.eye(3)
        pos = np.zeros(3)
        link = "stand"
        for _ in range(64):
            step = parent_of.get(link)
            if step is None:
                break
            parent, xyz, rpy = step
            r = rpy_matrix(*rpy)
            pos = np.asarray(xyz) + r @ pos
            rot = r @ rot
            link = parent
        # Invert: stand_T_root
        inv_rot = rot.T
        inv_pos = -inv_rot @ pos
        ry = math.asin(max(-1.0, min(1.0, -inv_rot[2, 0])))
        rz = math.atan2(inv_rot[1, 0], inv_rot[0, 0])
        rx = math.atan2(inv_rot[2, 1], inv_rot[2, 2])
        return (float(inv_pos[0]), float(inv_pos[1]), float(inv_pos[2]), rx, ry, rz)
    except Exception:
        return (0.0, 0.0, 0.0, 0.0, 0.0, -math.pi / 2.0)


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


# Articulated-gripper finger joints (only present in the *_pika_articulated URDF).
# Each finger travels up to _GRIPPER_FINGER_TRAVEL_M from the open (STL) pose to
# closed; finger_left moves +X, finger_right -X (jaw axis = X in the baked mesh
# frame). gripper percent: 100 = open (travel 0), 0 = closed (full travel).
# 0.049 is MEASURED (2026-09-04): the full-open gap between the TPU tip faces is
# 98.0 mm and the jaws close to contact, so each finger runs half of it. The meshes
# are baked AT that open stop, so travel 0 is genuinely 100 % open. The previous
# 0.047 came from assuming the vendor CAD drew the fingers at the open stop; it does
# not (rb_servo_server/tools/make_pika_tool_meshes.py). Keep this equal to
# FINGER_TRAVEL_M in the URDF generator and to safety.self_collision.mesh
# .gripper_finger_travel_m in the stack configs.
_GRIPPER_FINGER_TRAVEL_M = 0.049
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

# THE FIVE BODY GROUPS OF THE COLLISION MODEL. The monitor checks one geometry set
# but an operator reads the scene in parts, so the red violation overlay lights up
# only the groups the violating pair actually names — an arm folding onto the stand
# must not look the same as two grippers touching. The split is the server's own:
# collision_monitor.cpp attaches the Pika hulls by name ("<prefix>pika_gripper_base",
# "<prefix>pika_finger_left/right", legacy single "<prefix>pika_gripper") on top of
# the prefixed arm links, and everything without an arm prefix is stand/world
# geometry (stand hulls, ground_plane, external_box_*).
_SELF_COLLISION_GROUPS = (
    "left_arm", "right_arm", "left_gripper", "right_gripper", "stand", "environment",
)
# Cell structure drawn from the unified URDF's env_* links. The ones that carry a
# <collision> are checked geometry in the server's own `environment` barrier class
# (25/67 mm, not the 40/90 self set) -- today that is the riser under the stand base
# plate. They share ONE group here because the server gives them one class, and only
# geometry that is actually in the collision model can ever be named by a violating
# pair, so an env_* box that is drawn but never checked cannot light up.
_ENVIRONMENT_GEOM_PREFIX = "env_"
# Geometry-name marker for the Pika hulls, mirroring collision_monitor.cpp's
# isGripperGeometry (name contains "pika_").
_GRIPPER_GEOM_MARKER = "pika_"
# The GUI arm URDF's gripper links (rb5_850e_pika_articulated.urdf): the tool body
# plus the two prismatic fingers. viser names each mesh node ".../<link>/<geometry>",
# so a node belongs to the gripper iff one of these is a path segment. Keep in sync
# with rb_servo_server/tools/make_rb5_850e_urdfs.py.
_GRIPPER_URDF_LINKS = frozenset({"tool", "finger_left", "finger_right"})


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
# translucent + double-sided so the robot and stand stay visible through it.
#
# THIS IS THE ARM'S GEOMETRY, NOT THE ENFORCED LIMIT. The comment here used to say
# the shell's outer face "is the reach boundary the safety.reach_constraint damper
# enforces" — it never was. This asset is a static FK measurement baked at
# r_max_recommended 1.2526 m; the damper enforces safety.reach_constraint.r_max_m,
# which is config and changed twice on 2026-09-04 (1.250 -> 0.980 -> 1.050). On the
# evening of 2026-09-04 an operator turned this overlay on while the server was
# enforcing 0.980 and saw a surface 273 mm too generous, which is exactly no help.
# The ENFORCED sphere is drawn separately from published state — see
# update_reach_shell() below — and that is the one to trust.
_REACH_ENVELOPE_GREEN = (90, 200, 150)
_REACH_ENVELOPE_OPACITY = 0.22

# The ENFORCED reach shell: a true sphere of radius safety.reach_constraint.r_max_m
# centred on the arm mount, read from the server's published reach_shell block. Amber
# so it never reads as the green "what the arm can physically reach" envelope, and
# turns red while that arm is reported violated. Back-sided like the envelope so the
# near hemisphere is culled and the robot inside stays visible.
_REACH_SHELL_AMBER = (240, 170, 60)
_REACH_SHELL_RED = (255, 70, 70)
_REACH_SHELL_OPACITY = 0.16


def _reach_envelope_path() -> Path:
    return _asset_path("RB_GUI_REACH_ENVELOPE", "reach_envelope_rb5_850e.npz")


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
    handles["_server"] = server   # update_reach_shell builds its sphere from state
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
    scene_handles["reach_envelope_visible"] = bool(visible)
    for arm in ("left", "right"):
        _set_visible(scene_handles.get(f"{arm}_reach_envelope"), visible)
        _set_visible(scene_handles.get(f"{arm}_reach_shell"), visible)


def update_reach_shell(
    scene_handles: dict[str, Any],
    reach: Mapping[str, Any] | None,
    *,
    visible: bool = True,
) -> None:
    """Draw the ENFORCED reach shell from the server's published radii.

    The radius is server state (safety.reach_constraint.r_max_m), so the sphere is
    (re)built whenever it changes rather than baked at startup like the measured
    envelope asset. Centred on the arm mount, which the server publishes as
    base_stand_m — the sphere is attached under /stand/<side>_base, whose transform
    already carries that mount, so the sphere sits at the node origin.

    Drawn only while the constraint is ENABLED: a disabled shell bounds nothing and
    a surface drawn for it would be the same class of lie this replaced. Red while
    the arm is reported violated, amber otherwise."""
    if not isinstance(scene_handles, dict):
        return
    server = scene_handles.get("_server")
    enabled = isinstance(reach, Mapping) and bool(reach.get("enabled", False))
    r_max = _finite_or(reach.get("r_max_m"), 0.0) if isinstance(reach, Mapping) else 0.0
    if not visible or not enabled or r_max <= 0.0:
        for arm in ("left", "right"):
            _set_visible(scene_handles.get(f"{arm}_reach_shell"), False)
        return
    if server is None or not hasattr(server.scene, "add_icosphere"):
        return
    violated = {
        arm: isinstance(reach.get(arm), Mapping) and bool(reach[arm].get("violated", False))
        for arm in ("left", "right")
    }
    for arm in ("left", "right"):
        key = f"{arm}_reach_shell"
        color = _REACH_SHELL_RED if violated[arm] else _REACH_SHELL_AMBER
        prev = scene_handles.get(f"{key}_spec")
        spec = (round(r_max, 6), color)
        if prev == spec and scene_handles.get(key) is not None:
            _set_visible(scene_handles.get(key), True)
            continue
        handle = scene_handles.pop(key, None)
        if handle is not None:
            try:
                handle.remove()
            except Exception:  # noqa: BLE001 - a stale handle must not kill the frame
                pass
        try:
            scene_handles[key] = server.scene.add_icosphere(
                f"/stand/{arm}_base/reach_shell_enforced",
                radius=float(r_max),
                color=color,
                opacity=_REACH_SHELL_OPACITY,
                position=(0.0, 0.0, 0.0),
                visible=True,
            )
        except TypeError:  # older viser without opacity
            scene_handles[key] = server.scene.add_icosphere(
                f"/stand/{arm}_base/reach_shell_enforced",
                radius=float(r_max),
                color=color,
                position=(0.0, 0.0, 0.0),
                visible=True,
            )
        except Exception:  # noqa: BLE001 - never let the overlay break the viewer
            scene_handles[f"{key}_spec"] = None
            continue
        scene_handles[f"{key}_spec"] = spec


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
    return _asset_path("RB_GUI_IK_INFEASIBLE", "ik_infeasible_rb5_850e.npz")


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
    center = (0.5, 0.0, 0.5)
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


def _self_collision_geom_group(
    name: Any, left_prefix: str, right_prefix: str
) -> str | None:
    """Map ONE monitor geometry name onto its body group (one of
    _SELF_COLLISION_GROUPS), or None when the name is unusable.

    Side comes from the manifest's arm prefixes, which is exact and is why the
    manifest is preferred: the unified collision URDF also has STAND links named
    "stand_left_arm_base" / "stand_right_arm_base", so a name merely CONTAINING
    "left" is not an arm. Once the prefixes are known, matching neither of them is
    conclusive — that geometry is stand/world (stand hulls, ground_plane,
    external_box_*). env_* is taken out FIRST: it is cell structure the server checks
    in its own barrier class, and calling it "stand" would redden the stand mesh while
    leaving the box the arm actually reached untouched.

    Only a manifest-less state falls back to a word search, and then on the EARLIEST
    of "left"/"right" rather than the first one tested: the articulated gripper
    geometry "<right_prefix>pika_finger_left" contains both words, and testing "left"
    first (as the server's own side() lambda does) files the right arm's finger under
    the left arm."""
    if not isinstance(name, str) or not name:
        return None
    if name.startswith(_ENVIRONMENT_GEOM_PREFIX):
        return "environment"   # checked cell structure, not the stand itself
    side: str | None = None
    if left_prefix and name.startswith(left_prefix):
        side = "left"
    elif right_prefix and name.startswith(right_prefix):
        side = "right"
    elif not left_prefix and not right_prefix:
        i_left = name.find("left")
        i_right = name.find("right")
        if i_left >= 0 and (i_right < 0 or i_left < i_right):
            side = "left"
        elif i_right >= 0:
            side = "right"
    if side is None:
        return "stand"
    return f"{side}_gripper" if _GRIPPER_GEOM_MARKER in name else f"{side}_arm"


def _self_collision_red_groups(
    sc: Mapping[str, Any] | None, violated: bool, box_collision: bool
) -> set[str]:
    """Which of _SELF_COLLISION_GROUPS the red overlay must light up.

    Named from the geometry names of the near pairs that are IN HARD VIOLATION —
    `clearance_m < d_hard_m`, each pair against its own published floor. That per-pair
    floor is why the server sends it: the near list is ordered by RAW clearance while
    the floors differ per category (RB5: self/arm-stand 40 mm, gripper<->gripper 25 mm,
    intra-arm 5 mm), so "nearest" is not "violating". Measured 2026-09-06: the
    structural intra-arm link3<->link5 pair sits at ~23 mm — never violating its own
    5 mm floor — yet is the nearest pair on 99.4% of ticks, so keying the highlight off
    near_pairs[0] named that one arm and left an arm<->stand pair breaching its 40 mm
    floor (anywhere in 40..23 mm) completely unmarked.

    Fallbacks, each strictly more conservative than the last: no pair carries a usable
    d_hard_m (older server) -> near_pairs[0], the previous behavior; no near_pairs ->
    the coarse `pair` category, where the arm groups include their gripper; unknown
    `pair` -> all red."""
    if violated and isinstance(sc, Mapping) and sc.get("near_pairs_hard_truncated") is True:
        # Some breaching pairs could not fit in the state datagram. A surviving
        # pair must not make an omitted arm/structure look clear.
        return set(_SELF_COLLISION_GROUPS)
    if box_collision:
        # External-box telemetry is per-box, not per-geometry, so it cannot name a
        # side: keep both arms (and their grippers) red, as before.
        return {"left_arm", "right_arm", "left_gripper", "right_gripper"}
    if not violated:
        return set()
    manifest = sc.get("manifest") if isinstance(sc, Mapping) else None
    if not isinstance(manifest, Mapping):
        manifest = {}
    left_prefix = str(manifest.get("left_prefix") or "")
    right_prefix = str(manifest.get("right_prefix") or "")
    pairs = sc.get("near_pairs") if isinstance(sc, Mapping) else None
    if not isinstance(pairs, (list, tuple)):
        pairs = ()
    pairs = [p for p in pairs if isinstance(p, Mapping)]

    def _groups_of(pair: Mapping[str, Any]) -> set[str]:
        return {
            group
            for group in (
                _self_collision_geom_group(pair.get("name_a"), left_prefix, right_prefix),
                _self_collision_geom_group(pair.get("name_b"), left_prefix, right_prefix),
            )
            if group is not None
        }

    groups: set[str] = set()
    graded = False
    for entry in pairs:
        clearance = entry.get("clearance_m")
        d_hard = entry.get("d_hard_m")
        if not _is_finite(clearance) or not _is_finite(d_hard):
            continue
        graded = True  # this server publishes per-pair floors; trust them alone
        if float(clearance) < float(d_hard):
            groups |= _groups_of(entry)
    if graded:
        # A violated verdict whose breaching pair fell outside the published near list
        # (viz_near_pairs_m truncation) must not silently highlight nothing.
        return groups if groups else set(_SELF_COLLISION_GROUPS)
    if pairs:
        groups = _groups_of(pairs[0])
        if groups:
            return groups
    pair = sc.get("pair") if isinstance(sc, Mapping) else None
    if pair == "left_right":
        return {"left_arm", "left_gripper", "right_arm", "right_gripper"}
    if pair == "left_stand":
        return {"left_arm", "left_gripper", "stand"}
    if pair == "right_stand":
        return {"right_arm", "right_gripper", "stand"}
    return set(_SELF_COLLISION_GROUPS)  # unknown/legacy: conservative all-red


def _urdf_part_meshes(
    scene_handles: dict[str, Any], cache_key: str, urdf_handle: Any
) -> dict[str, list[Any]] | None:
    """Split one ViserUrdf's mesh nodes into {"arm": [...], "gripper": [...]}.

    viser builds each mesh node's path through the URDF link that carries it, so the
    link name is a path segment of the node name and _GRIPPER_URDF_LINKS decides the
    group. Returns None when the handle exposes no usable mesh list or the URDF has
    no gripper links — callers then fall back to whole-arm visibility, which is the
    behavior this split refines, so a viser change degrades instead of breaking.

    Cached per handle identity: the box-DH swap rebuilds the URDFs
    (ensure_calibrated_arm_urdfs), and a stale list would drive removed nodes."""
    cached = scene_handles.get(cache_key)
    if isinstance(cached, dict) and cached.get("handle") is urdf_handle:
        return cached.get("groups")
    meshes = getattr(urdf_handle, "_meshes", None) if urdf_handle is not None else None
    groups: dict[str, list[Any]] | None = None
    if isinstance(meshes, (list, tuple)) and meshes:
        arm: list[Any] = []
        gripper: list[Any] = []
        for handle in meshes:
            name = getattr(handle, "name", None)
            if not isinstance(name, str):
                arm = []
                break  # unrecognizable handles: do not guess, fall back
            segments = set(name.split("/"))
            (gripper if segments & _GRIPPER_URDF_LINKS else arm).append(handle)
        if arm and gripper:
            groups = {"arm": arm, "gripper": gripper}
    scene_handles[cache_key] = {"handle": urdf_handle, "groups": groups}
    return groups


def _set_urdf_parts_visible(
    scene_handles: dict[str, Any], cache_key: str, urdf_handle: Any,
    *, arm_visible: bool, gripper_visible: bool,
) -> bool:
    """Drive an arm URDF's link meshes per body group. False when the URDF could not
    be split, so the caller applies the un-split (whole-arm) visibility instead."""
    groups = _urdf_part_meshes(scene_handles, cache_key, urdf_handle)
    if groups is None:
        return False
    for handle in groups["arm"]:
        _set_visible(handle, arm_visible)
    for handle in groups["gripper"]:
        _set_visible(handle, gripper_visible)
    return True


def update_self_collision_overlay(scene_handles: dict[str, Any], latest: Any) -> None:
    """Paint the colliding PARTS translucent red while self_collision.violated.

    Driven by the server's self_collision telemetry, so monitor_only runs show the
    overlay too. Only the body groups the violating pair names turn red, out of the
    five the collision model has: left arm, right arm, left gripper, right gripper,
    stand (see _self_collision_red_groups / _SELF_COLLISION_GROUPS). So two grippers
    touching lights the two grippers, not two whole arms, and an arm folding onto the
    stand leaves that arm's gripper normal. Where the URDF cannot be split into arm
    and gripper meshes the group pair collapses back to the whole arm, which is the
    behavior this refines.

    pgmode real (physical_motion_expected=True): the ACTUAL robot (q_actual) turns
    red. pgmode simulation: the commanded ghost (q_sent) turns red while the solid
    robot keeps showing the true (stationary) state. External-box collision telemetry
    is per-box only in this first pass, so any box collision turns both arms red."""
    if not isinstance(scene_handles, dict):
        return
    sc = getattr(latest, "self_collision", None) if latest is not None else None
    violated = isinstance(sc, Mapping) and bool(sc.get("violated", False))
    box_collision = _external_box_collision(sc)
    _update_self_collision_witness_markers(scene_handles, sc, violated)
    red = _self_collision_red_groups(sc, violated, box_collision)
    stand_red = "stand" in red
    physical_real = latest is not None and (
        getattr(latest.left, "physical_motion_expected", None) is True
        or getattr(latest.right, "physical_motion_expected", None) is True
    )

    for side, arm_state in (("left", getattr(latest, "left", None)),
                            ("right", getattr(latest, "right", None))):
        arm_red = f"{side}_arm" in red
        gripper_red = f"{side}_gripper" in red
        any_red = arm_red or gripper_red
        overlay = scene_handles.get(f"{side}_urdf_collision")
        if any_red and overlay is not None and arm_state is not None:
            q = arm_state.q_actual_deg if physical_real else (
                arm_state.q_sent_deg if arm_state.q_sent_deg is not None else arm_state.q_actual_deg
            )
            try:
                _update_urdf_config(overlay, _joint_cfg_radians(q))
            except Exception as exc:
                scene_handles["urdf_collision_update_error"] = f"{type(exc).__name__}: {exc}"
        # Red overlay: only the red groups' meshes, under a frame shown when either is.
        _set_urdf_parts_visible(
            scene_handles, f"_{side}_collision_mesh_groups", overlay,
            arm_visible=arm_red, gripper_visible=gripper_red,
        )
        _set_visible(scene_handles.get(f"{side}_base_collision"), any_red)
        # Replace (not overlap) the model the red overlay represents to avoid
        # z-fighting at the identical configuration — per GROUP, only the red ones.
        # In pgmode simulation the overlay replaces the commanded ghost instead, so
        # the solid robot (q_actual) stays fully visible.
        if not _set_urdf_parts_visible(
            scene_handles, f"_{side}_solid_mesh_groups", scene_handles.get(f"{side}_urdf"),
            arm_visible=not (arm_red and physical_real),
            gripper_visible=not (gripper_red and physical_real),
        ):
            _set_visible(scene_handles.get(f"{side}_base"), not (any_red and physical_real))
        else:
            _set_visible(scene_handles.get(f"{side}_base"), True)
        if any_red and not physical_real:
            # The ghost is what the overlay replaces here. update_scene_markers owns
            # the ghost FRAME (it re-shows it every tick from _reference_ghost_active),
            # so hide the replaced meshes and leave the frame to it; only fall back to
            # hiding the whole ghost when the split is unavailable.
            if not _set_urdf_parts_visible(
                scene_handles, f"_{side}_ref_mesh_groups", scene_handles.get(f"{side}_urdf_ref"),
                arm_visible=not arm_red, gripper_visible=not gripper_red,
            ):
                _set_visible(scene_handles.get(f"{side}_base_ref"), False)
        else:
            _set_urdf_parts_visible(
                scene_handles, f"_{side}_ref_mesh_groups", scene_handles.get(f"{side}_urdf_ref"),
                arm_visible=True, gripper_visible=True,
            )

    _set_visible(scene_handles.get("stand_mesh_collision"), stand_red)
    _set_visible(scene_handles.get("stand_mesh"), not stand_red)
    _set_environment_red(scene_handles, "environment" in red)


def _set_environment_red(scene_handles: dict[str, Any], red: bool) -> None:
    """Recolour the env_* cell-structure boxes for the self-collision highlight.

    These are add_box handles, not URDF meshes, so there is no translucent duplicate to
    swap in the way the arms and the stand have — the box IS the drawing, and the
    highlight is its colour. The nominal colour comes back from environment_rgb, so a
    cleared violation restores exactly what the URDF asked for."""
    names = scene_handles.get("environment_names")
    if not isinstance(names, (list, tuple)):
        return
    nominal = scene_handles.get("environment_rgb")
    nominal = nominal if isinstance(nominal, Mapping) else {}
    for key in names:
        handle = scene_handles.get(f"environment_{key}")
        if handle is None:
            continue
        colour = _SELF_COLLISION_STAND_RGB if red else nominal.get(key)
        if colour is None:
            continue
        try:
            handle.color = tuple(colour)
        except (TypeError, ValueError, AttributeError):
            pass


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
        pose = _urdf_stand_to_world_pose(_unified_urdf_path())
        server.scene.add_frame(
            _SC_WORLD_FRAME,
            wxyz=_pose_wxyz(pose),
            position=_pose_position(pose),
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


def _near_pair_band(
    pair: Mapping[str, Any], clearance_m: float,
    hard: float, slow: float, ext_hard: float, ext_slow: float,
) -> tuple[str, str, tuple[int, int, int]]:
    """(kind, band, rgb) for one near pair: which palette, and where in it.

    THE PAIR'S OWN BAND WINS. `d_hard_m`/`d_slow_m` are published per pair because the
    monitor enforces FIVE bands (self, intra-arm, gripper<->gripper, floor, keep-out
    box) whose floors are an order of magnitude apart, while this function's caller can
    only see external-vs-not. Measured 2026-09-06: the structural intra-arm
    link3<->link5 pair sits at ~23 mm against its own 5 mm floor, so banding it against
    the 40 mm self floor painted it HARD RED continuously — a permanent false alarm on
    a pair that is not going to collide. The external/self split below is the fallback
    for a server that does not publish the per-pair band."""
    external = bool(pair.get("external", False))
    d_hard = _finite_or(pair.get("d_hard_m"), ext_hard if external else hard)
    d_slow = _finite_or(pair.get("d_slow_m"), ext_slow if external else slow)
    if external:
        kind = "ext"
        palette = (_EXTERNAL_NEAR_HARD_RGB, _EXTERNAL_NEAR_CAUTION_RGB, _EXTERNAL_NEAR_OK_RGB)
    else:
        kind = "self"
        palette = (_SELF_COLLISION_NEAR_HARD_RGB, _SELF_COLLISION_NEAR_CAUTION_RGB, _SELF_COLLISION_NEAR_OK_RGB)
    if clearance_m < d_hard:
        return kind, "hard", palette[0]
    if clearance_m < d_slow:
        return kind, "caution", palette[1]
    return kind, "ok", palette[2]


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
                    kind, band, rgb = _near_pair_band(
                        pair, float(clearance), hard, slow, ext_hard, ext_slow)
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


def _is_finite(value: Any) -> bool:
    """True for a real, finite number. bool is rejected: JSON `true` must not read as 1."""
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


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
            position=_pose_position(_stand_mesh_pose()),
            wxyz=_pose_wxyz(_stand_mesh_pose()),
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
                position=_pose_position(_stand_mesh_pose()),
                wxyz=_pose_wxyz(_stand_mesh_pose()),
                visible=False,
            )
        except Exception as exc:
            handles["stand_mesh_collision_error"] = _asset_error(f"{type(exc).__name__}: {exc}")
    except Exception as exc:
        handles["stand_mesh_error"] = _asset_error(f"{type(exc).__name__}: {exc}")


# CELL FURNITURE (the work table, and the riser between it and the stand base plate).
# These are things the operator sees in the room but the viewer used to draw nothing
# for, so the rendered scene ended at the stand and the arms appeared to reach into
# empty space.
#
# The rule here is the stand mesh's rule, for the stand mesh's reason: the dimensions
# and the pose live in the UNIFIED URDF and are read from it, never written as
# constants in this module. Hardcoding the stand's visual origin here is exactly how
# the RB3 -> RB5 swap shipped a stand rotated 90 deg (see _stand_visual_from_urdf).
# Anything named env_* under the `stand` link is picked up with no further GUI change,
# so adding a second table later is a URDF edit, not a code edit.
#
# VISUAL ONLY, DELIBERATELY. The CollisionMonitor builds from
# buildGeom(..., pinocchio::COLLISION, ...), so a link carrying only <visual> adds
# zero geoms and the checked pair set is unchanged. Putting the table INTO the
# collision model is a separate, reviewed decision (it would start braking the arms
# on a surface whose measured pose has never been validated against contact).
_ENVIRONMENT_LINK_PREFIX = "env_"
# Only used when a link's visual carries no <material><color>: a neutral mid-gray,
# distinct from the stand's (90, 90, 90) so furniture does not read as structure.
_ENVIRONMENT_DEFAULT_RGB = (140, 136, 128)
# The one env_* link the operator can retune from the GUI (see set_riser_height_m).
_ENVIRONMENT_RISER_LINK = "env_stand_riser"
# Sanity bounds on that live value. Not a safety limit -- nothing reads the furniture
# but the renderer -- just a guard so a fat-fingered entry cannot put the tables in
# orbit and leave the operator wondering where the scene went.
_ENVIRONMENT_RISER_MIN_M = 0.02
_ENVIRONMENT_RISER_MAX_M = 1.50


def _urdf_origin(element: Any) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """A URDF <origin> child's (xyz, rpy), defaulting to identity as URDF specifies."""
    origin = element.find("origin") if element is not None else None
    if origin is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    xyz = tuple(float(v) for v in (origin.get("xyz") or "0 0 0").split())
    rpy = tuple(float(v) for v in (origin.get("rpy") or "0 0 0").split())
    if len(xyz) != 3 or len(rpy) != 3:
        raise ValueError(f"malformed <origin> xyz={origin.get('xyz')} rpy={origin.get('rpy')}")
    return xyz, rpy


def _urdf_transform(xyz: tuple[float, float, float], rpy: tuple[float, float, float]) -> Any:
    return _pose_transform(xyz, _rpy_to_wxyz(*rpy))


def _link_pose_in(root: Any, link_name: str, base_link: str) -> Any | None:
    """Compose fixed-joint origins from `base_link` down to `link_name`.

    Returns None when the link is not reachable from base_link through FIXED joints
    only — a movable joint on the path would make the pose configuration-dependent,
    which is not what cell furniture is."""
    parents: dict[str, tuple[str, Any]] = {}
    for joint in root.findall("joint"):
        child = joint.find("child")
        parent = joint.find("parent")
        if child is None or parent is None:
            continue
        parents[child.get("link", "")] = (parent.get("link", ""), joint)
    transform = _pose_transform((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    name = link_name
    for _ in range(32):  # depth guard: a URDF cycle must not hang the viewer
        if name == base_link:
            return transform
        entry = parents.get(name)
        if entry is None:
            return None
        parent_name, joint = entry
        if joint.get("type") != "fixed":
            return None
        xyz, rpy = _urdf_origin(joint)
        transform = _multiply_transform(_urdf_transform(xyz, rpy), transform)
        name = parent_name
    return None


def _environment_visuals_from_urdf(urdf_path: Path) -> list[dict[str, Any]]:
    """Every env_* link's <visual> in the unified URDF, posed in the stand frame.

    Each entry is {name, shape, position, wxyz, rgb, opacity} plus either
    `dimensions` (box, metres) or `mesh_path` + `mesh_scale`. Unreadable or
    unsupported entries are skipped with `error` set, so one bad link cannot cost
    the operator the rest of the furniture."""
    import xml.etree.ElementTree as ET

    out: list[dict[str, Any]] = []
    try:
        root = ET.parse(urdf_path).getroot()
    except Exception as exc:
        return [{"name": "", "error": f"{type(exc).__name__}: {exc}"}]
    for link in root.findall("link"):
        name = link.get("name", "")
        if not name.startswith(_ENVIRONMENT_LINK_PREFIX):
            continue
        link_pose = _link_pose_in(root, name, "stand")
        if link_pose is None:
            out.append({"name": name, "error": "not fixed to the stand link"})
            continue
        for index, visual in enumerate(link.findall("visual")):
            entry: dict[str, Any] = {"name": name if index == 0 else f"{name}_{index}"}
            try:
                xyz, rpy = _urdf_origin(visual)
                pose = _multiply_transform(link_pose, _urdf_transform(xyz, rpy))
                rotation, position = pose
                entry["position"] = position
                entry["wxyz"] = _matrix_to_wxyz(rotation)
                geometry = visual.find("geometry")
                box = geometry.find("box") if geometry is not None else None
                mesh = geometry.find("mesh") if geometry is not None else None
                if box is not None:
                    dims = tuple(float(v) for v in (box.get("size") or "").split())
                    if len(dims) != 3:
                        raise ValueError(f"<box size=\"{box.get('size')}\">")
                    entry["shape"] = "box"
                    entry["dimensions"] = dims
                elif mesh is not None and mesh.get("filename"):
                    entry["shape"] = "mesh"
                    entry["mesh_path"] = (urdf_path.parent / mesh.get("filename")).resolve()
                    scale = tuple(float(v) for v in (mesh.get("scale") or "1 1 1").split())
                    entry["mesh_scale"] = scale if len(scale) == 3 else (1.0, 1.0, 1.0)
                else:
                    raise ValueError("no <box> or <mesh> geometry")
                color = visual.find("material/color")
                rgba = tuple(float(v) for v in (color.get("rgba") or "").split()) if color is not None else ()
                entry["rgb"] = (tuple(int(round(255 * c)) for c in rgba[:3])
                                if len(rgba) >= 3 else _ENVIRONMENT_DEFAULT_RGB)
                entry["opacity"] = float(rgba[3]) if len(rgba) == 4 else 1.0
            except Exception as exc:
                entry["error"] = f"{type(exc).__name__}: {exc}"
            out.append(entry)
    return out


def _add_environment_visuals(server: Any, handles: dict[str, Any]) -> None:
    """Draw the env_* furniture from the unified URDF under /stand/env_*."""
    entries = _environment_visuals_from_urdf(_unified_urdf_path())
    names: list[str] = []
    nominal: dict[str, tuple] = {}
    rgb: dict[str, tuple] = {}
    for entry in entries:
        key = entry.get("name") or "env"
        if entry.get("error"):
            handles[f"environment_error_{key}"] = _asset_error(f"{key}: {entry['error']}")
            continue
        try:
            if entry["shape"] == "box":
                if not hasattr(server.scene, "add_box"):
                    continue
                handle = server.scene.add_box(
                    f"/stand/{key}",
                    color=entry["rgb"],
                    dimensions=entry["dimensions"],
                    position=entry["position"],
                    wxyz=entry["wxyz"],
                )
            else:
                import trimesh

                mesh = trimesh.load_mesh(str(entry["mesh_path"]))
                mesh.apply_scale(entry["mesh_scale"])
                try:
                    mesh.visual.face_colors = (*entry["rgb"], 255)
                except Exception:  # pragma: no cover - trimesh visual variants
                    pass
                handle = server.scene.add_mesh_trimesh(
                    f"/stand/{key}", mesh=mesh,
                    position=entry["position"], wxyz=entry["wxyz"],
                )
        except Exception as exc:
            handles[f"environment_error_{key}"] = _asset_error(f"{key}: {type(exc).__name__}: {exc}")
            continue
        handles[f"environment_{key}"] = handle
        # The URDF geometry is the BASELINE every later adjustment is computed from.
        # Keeping it means set_riser_height_m() can be called any number of times
        # without the corrections compounding, and "back to the URDF value" is always
        # one call away.
        nominal[key] = (tuple(entry.get("dimensions", ())), tuple(entry["position"]))
        rgb[key] = tuple(entry["rgb"])
        names.append(key)
    handles["environment_names"] = names
    handles["environment_nominal"] = nominal
    # Kept separately from `nominal` (which set_riser_height_m unpacks positionally):
    # the colour is what update_self_collision_overlay swaps to red and back.
    handles["environment_rgb"] = rgb


def set_environment_visible(scene_handles: dict[str, Any], visible: bool) -> None:
    """Show/hide the cell furniture (GUI toggle). Solid geometry occludes the arms
    from below, so the operator needs to be able to take it away."""
    for key in scene_handles.get("environment_names", ()):
        _set_visible(scene_handles.get(f"environment_{key}"), bool(visible))


def environment_riser_nominal_height_m(scene_handles: Mapping[str, Any]) -> float | None:
    """The riser height the URDF ships, i.e. what the operator is adjusting away from."""
    nominal = scene_handles.get("environment_nominal") or {}
    entry = nominal.get(_ENVIRONMENT_RISER_LINK)
    if not entry or len(entry[0]) != 3:
        return None
    return float(entry[0][2])


def environment_riser_height_m(scene_handles: Mapping[str, Any]) -> float | None:
    """The riser height currently drawn (the operator's value, or the URDF's)."""
    current = scene_handles.get("environment_riser_height_m")
    if isinstance(current, (int, float)):
        return float(current)
    return environment_riser_nominal_height_m(scene_handles)


def environment_table_top_z_m(scene_handles: Mapping[str, Any]) -> float | None:
    """Stand-frame z of the drawn table tops, which is what the operator is really
    judging when they adjust the riser: the riser's underside IS the table top."""
    nominal = scene_handles.get("environment_nominal") or {}
    entry = nominal.get(_ENVIRONMENT_RISER_LINK)
    height = environment_riser_height_m(scene_handles)
    if not entry or height is None or len(entry[0]) != 3:
        return None
    top = float(entry[1][2]) + float(entry[0][2]) / 2.0   # riser top = plate underside
    return top - height


def set_riser_height_m(scene_handles: dict[str, Any], height_m: float) -> str:
    """Re-draw the riser at `height_m`, carrying the tables with it.

    THE RISER IS THE ONE FURNITURE DIMENSION THAT IS NOT REALLY KNOWN. Its columns
    measured 280-290 mm and the URDF models 300, so the drawn table top is 10-20 mm
    low; rather than freeze a wrong number, the operator dials it in against what
    they can see and the settled value goes back into the URDF generator.

    The riser hangs from the stand base plate's underside, so its TOP is fixed and it
    grows downward; the tables sit ON its underside, so they travel with it. Every
    position is recomputed from the URDF baseline, never from the current one.
    Returns "" on success, else why it was refused."""
    nominal = scene_handles.get("environment_nominal") or {}
    riser = nominal.get(_ENVIRONMENT_RISER_LINK)
    if not riser or len(riser[0]) != 3:
        return f"no {_ENVIRONMENT_RISER_LINK} box in the unified URDF"
    try:
        height = float(height_m)
    except (TypeError, ValueError):
        return f"riser height {height_m!r} is not a number"
    if not math.isfinite(height) or not (_ENVIRONMENT_RISER_MIN_M <= height <= _ENVIRONMENT_RISER_MAX_M):
        return (f"riser height {height} m is outside "
                f"[{_ENVIRONMENT_RISER_MIN_M}, {_ENVIRONMENT_RISER_MAX_M}] m")
    dims, position = riser
    nominal_height = float(dims[2])
    top_z = float(position[2]) + nominal_height / 2.0
    handle = scene_handles.get(f"environment_{_ENVIRONMENT_RISER_LINK}")
    if handle is None:
        return f"{_ENVIRONMENT_RISER_LINK} is not drawn"
    try:
        handle.dimensions = (float(dims[0]), float(dims[1]), height)
        handle.position = (float(position[0]), float(position[1]), top_z - height / 2.0)
        shift = nominal_height - height       # riser shorter -> everything under it rises
        for key in scene_handles.get("environment_names", ()):
            if key == _ENVIRONMENT_RISER_LINK:
                continue
            other = nominal.get(key)
            other_handle = scene_handles.get(f"environment_{key}")
            if other is None or other_handle is None:
                continue
            x, y, z = other[1]
            other_handle.position = (float(x), float(y), float(z) + shift)
    except Exception as exc:  # pragma: no cover - viser handle already gone
        return f"{type(exc).__name__}: {exc}"
    scene_handles["environment_riser_height_m"] = height
    return ""


_ARM_URDF_HANDLE_KEYS = (
    "left_urdf", "right_urdf", "left_urdf_ref", "right_urdf_ref",
    "left_urdf_collision", "right_urdf_collision",
)


def _manifest_arm_urdf_paths(manifest: Any) -> tuple[Path, Path] | None:
    """The calibrated per-arm GUI URDFs the server published (state manifest
    robot_urdf_left/right, written by its box DH calibration stage), or None when the
    server did not calibrate or either file is not readable from here."""
    if not isinstance(manifest, Mapping):
        return None
    left = manifest.get("robot_urdf_left")
    right = manifest.get("robot_urdf_right")
    if not isinstance(left, str) or not isinstance(right, str) or not left or not right:
        return None
    lp, rp = Path(left), Path(right)
    if not lp.exists() or not rp.exists():
        return None
    return lp, rp


def _remove_arm_urdfs(handles: dict[str, Any]) -> None:
    for key in _ARM_URDF_HANDLE_KEYS:
        handle = handles.pop(key, None)
        if handle is None:
            continue
        try:
            # Remove the URDF's root frames only: viser removes a node's children
            # with it, and ViserUrdf.remove() then re-removes every child and warns
            # once per node (336 warnings per swap, measured 2026-09-06). The root
            # frames are viser-private attributes; fall back to remove() without them.
            roots = [getattr(handle, "_visual_root_frame", None),
                     getattr(handle, "_collision_root_frame", None)]
            roots = [r for r in roots if r is not None]
            if roots:
                for root in roots:
                    root.remove()
            else:
                handle.remove()
        except Exception as exc:  # pragma: no cover - viser handle already gone
            handles["urdf_remove_error"] = f"{type(exc).__name__}: {exc}"


def ensure_calibrated_arm_urdfs(scene_handles: dict[str, Any], latest: Any) -> bool:
    """THE BOX DH CALIBRATION IN THE VIEWER (2026-09-05). The real server reads each
    box's calibrated DH at startup and publishes calibrated copies of the GUI arm
    URDF (one per arm) in its state manifest. The viewer starts on the nominal asset
    (no state yet); on the first manifest carrying both paths it swaps its solid,
    ghost and collision-overlay arms to the calibrated files, so what is drawn is
    the chain the server actually solves and checks. Returns True when it swapped."""
    if not isinstance(scene_handles, dict):
        return False
    sc = getattr(latest, "self_collision", None) if latest is not None else None
    manifest = sc.get("manifest") if isinstance(sc, Mapping) else None
    paths = _manifest_arm_urdf_paths(manifest)
    if paths is None:
        return False
    wanted = (str(paths[0]), str(paths[1]))
    if scene_handles.get("arm_urdf_paths") == wanted:
        return False
    server = scene_handles.get("_server")
    if server is None:
        return False
    _remove_arm_urdfs(scene_handles)
    _add_robot_urdfs(server, scene_handles, left_path=paths[0], right_path=paths[1])
    if "urdf_error" in scene_handles:
        return False
    scene_handles["arm_urdf_source"] = "box DH calibrated"
    return True


def _add_robot_urdfs(server: Any, handles: dict[str, Any],
                     left_path: Path | None = None, right_path: Path | None = None) -> None:
    default_path = _robot_urdf_path() if (left_path is None or right_path is None) else None
    left_urdf = left_path if left_path is not None else default_path
    right_urdf = right_path if right_path is not None else default_path
    for urdf_path in (left_urdf, right_urdf):
        if not urdf_path.exists():
            handles["urdf_error"] = _asset_error(f"robot URDF not found: {urdf_path}")
            return
    handles.pop("urdf_error", None)
    handles["arm_urdf_paths"] = (str(left_urdf), str(right_urdf))
    handles.setdefault("arm_urdf_source", "nominal")
    try:
        from viser.extras import ViserUrdf

        handles["left_urdf"] = ViserUrdf(server, left_urdf, root_node_name="/stand/left_base")
        handles["right_urdf"] = ViserUrdf(server, right_urdf, root_node_name="/stand/right_base")
        handles["urdf_joint_names"] = tuple(handles["left_urdf"].get_actuated_joint_names())
        # Translucent reference "ghost" robot following q_ref. In controller (pgmode)
        # simulation the controller does not move q_actual to track streamed servo_j,
        # so this overlay shows the commanded motion while the solid robot truthfully
        # stays at q_actual. RGBA mesh_color_override -> add_mesh_simple(opacity=a).
        if _reference_ghost_enabled():
            try:
                handles["left_urdf_ref"] = ViserUrdf(
                    server, left_urdf, root_node_name="/stand/left_base_ref",
                    mesh_color_override=_REFERENCE_GHOST_RGBA)
                handles["right_urdf_ref"] = ViserUrdf(
                    server, right_urdf, root_node_name="/stand/right_base_ref",
                    mesh_color_override=_REFERENCE_GHOST_RGBA)
            except Exception as exc:
                handles["urdf_ref_error"] = f"{type(exc).__name__}: {exc}"
        # Translucent-red self-collision overlay robots (hidden until violated).
        # In pgmode real they replace the actual robot; in pgmode simulation
        # they replace the commanded ghost.
        try:
            handles["left_urdf_collision"] = ViserUrdf(
                server, left_urdf, root_node_name="/stand/left_base_collision",
                mesh_color_override=_SELF_COLLISION_RGBA)
            handles["right_urdf_collision"] = ViserUrdf(
                server, right_urdf, root_node_name="/stand/right_base_collision",
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
        # Startup placeholders for the TCP gizmos, overwritten as soon as state
        # arrives. Rotated Rz(+90 deg) with everything else stand-frame on the
        # 2026-09-02 RB3 -> RB5 swap, so the first frames are not visibly wrong.
        has_transform_controls = hasattr(server.scene, "add_transform_controls")
        handles["left_tcp"] = server.scene.add_frame("/stand/left_tcp", show_axes=not has_transform_controls, axes_length=0.08, axes_radius=0.003, position=(0.1725, 0.1601, 0.78))
        handles["right_tcp"] = server.scene.add_frame("/stand/right_tcp", show_axes=not has_transform_controls, axes_length=0.08, axes_radius=0.003, position=(0.1725, -0.1601, 0.78))
        handles["left_tcp_ref"] = server.scene.add_frame("/stand/left_tcp_ref", show_axes=True, axes_length=0.055, axes_radius=0.002, position=(0.1725, 0.1601, 0.78))
        handles["right_tcp_ref"] = server.scene.add_frame("/stand/right_tcp_ref", show_axes=True, axes_length=0.055, axes_radius=0.002, position=(0.1725, -0.1601, 0.78))
        if hasattr(server.scene, "add_label"):
            # No label on the actual-TCP gizmo: it is the frame the operator reads
            # continuously in every run mode, so a permanent floating caption is
            # pure clutter over the arm. The reference gizmo keeps its label — it
            # only appears in the controller-sim reference display, where naming
            # which TCP source is drawn is what disambiguates the two gizmos.
            for arm, position in (
                ("left", (0.1725, 0.1601, 0.78)),
                ("right", (0.1725, -0.1601, 0.78)),
            ):
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
                handles[f"{arm}_tcp_cmd_trail_points"] = []
                for key in (
                    f"{arm}_tcp_trail",
                    f"{arm}_tcp_ref_trail",
                    f"{arm}_tcp_cmd_trail",
                ):
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
                # Amber is shared by both arms so "commanded target" has one identity.
                handles[f"{arm}_tcp_cmd_trail_color"] = (250, 215, 60)
                _set_visible(handles.get(f"{arm}_tcp_cmd_trail"), False)
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
        if hasattr(server.scene, "add_line_segments"):
            handles["chunk_overlay_line_mode"] = "line_segments"
            for arm in ("left", "right"):
                handles[f"{arm}_chunk_overlay"] = server.scene.add_line_segments(
                    f"/stand/{arm}_chunk_overlay",
                    points=_line_segments_array(),
                    colors=_line_segment_colors_array(),
                    line_width=4.5,
                )
                handles[f"{arm}_chunk_overlay_history"] = server.scene.add_line_segments(
                    f"/stand/{arm}_chunk_overlay_history",
                    points=_line_segments_array(),
                    colors=_line_segment_colors_array(),
                    line_width=2.5,
                    visible=False,
                )
        elif hasattr(server.scene, "add_point_cloud"):
            handles["chunk_overlay_line_mode"] = "point_cloud"
            for arm in ("left", "right"):
                handles[f"{arm}_chunk_overlay"] = server.scene.add_point_cloud(
                    f"/stand/{arm}_chunk_overlay",
                    points=_points_array(),
                    colors=_colors_array(),
                    point_size=0.012,
                    point_shape="rounded",
                )
        if hasattr(server.scene, "add_point_cloud"):
            for arm in ("left", "right"):
                handles[f"{arm}_chunk_overlay_points"] = server.scene.add_point_cloud(
                    f"/stand/{arm}_chunk_overlay_points",
                    points=_points_array(),
                    colors=_colors_array(),
                    point_size=0.022,
                    point_shape="rounded",
                )
        for arm in ("left", "right"):
            if hasattr(server.scene, "add_icosphere"):
                handles[f"{arm}_chunk_overlay_cursor"] = server.scene.add_icosphere(
                    f"/stand/{arm}_chunk_overlay_cursor",
                    radius=0.012,
                    color=(255, 255, 90),
                    position=(0.0, 0.0, 0.0),
                )
            elif hasattr(server.scene, "add_point_cloud"):
                handles[f"{arm}_chunk_overlay_cursor"] = server.scene.add_point_cloud(
                    f"/stand/{arm}_chunk_overlay_cursor",
                    points=_points_array(),
                    colors=_colors_array(((255, 255, 90),)),
                    point_size=0.03,
                    point_shape="rounded",
                )
        if hasattr(server.scene, "add_line_segments"):
            for arm in ("left", "right"):
                handles[f"{arm}_chunk_overlay_error"] = server.scene.add_line_segments(
                    f"/stand/{arm}_chunk_overlay_error",
                    points=_line_segments_array(),
                    colors=_line_segment_colors_array(),
                    line_width=3.5,
                )
                handles[f"{arm}_chunk_overlay_axes"] = server.scene.add_line_segments(
                    f"/stand/{arm}_chunk_overlay_axes",
                    points=_line_segments_array(),
                    colors=_line_segment_colors_array(),
                    line_width=2.0,
                    visible=False,
                )
                handles[f"{arm}_chunk_overlay_cursor_axes"] = server.scene.add_line_segments(
                    f"/stand/{arm}_chunk_overlay_cursor_axes",
                    points=_line_segments_array(),
                    colors=_line_segment_colors_array(),
                    line_width=3.0,
                    visible=False,
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
        for arm in ("left", "right"):
            for suffix in ("", "_history", "_points", "_cursor", "_error", "_axes", "_cursor_axes"):
                _set_visible(handles.get(f"{arm}_chunk_overlay{suffix}"), False)
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
        _add_environment_visuals(server, handles)
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
        f"{arm}_tcp_cmd_trail",
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
    limit = _TCP_TRAIL_LIMIT
    try:
        candidate = int(scene_handles.get("tcp_trail_limit") or _TCP_TRAIL_LIMIT)
        if candidate > 0:
            limit = candidate
    except (TypeError, ValueError, OverflowError):
        limit = _TCP_TRAIL_LIMIT
    if not points or points[-1] != position:
        points.append(position)
        del points[:-limit]
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


def _hide_chunk_overlay_handles(scene_handles: dict[str, Any], arm: str) -> None:
    for suffix in ("", "_history", "_points", "_cursor", "_error", "_axes", "_cursor_axes"):
        _set_visible(scene_handles.get(f"{arm}_chunk_overlay{suffix}"), False)


def _set_chunk_cursor_size(cursor_handle: Any, dot_size: float | None) -> None:
    dot_default = 0.022
    try:
        dot_value = float(dot_size) if dot_size else dot_default
        if not math.isfinite(dot_value):
            raise ValueError
    except Exception:
        dot_value = dot_default
    ratio = (dot_value / dot_default) if dot_size else 1.0
    ratio = max(0.3, min(2.0, ratio))
    try:
        cursor_handle.scale = float(ratio)
    except Exception:
        pass
    try:
        cursor_handle.point_size = float(max(0.006, min(0.05, dot_value * 1.8)))
    except Exception:
        pass


def update_chunk_overlay(
    scene_handles: dict[str, Any],
    overlay: ChunkOverlaySnapshot | None,
    *,
    stale: bool = False,
    visible: bool = True,
    now_monotonic: float | None = None,
    actual_positions: Mapping[str, tuple[float, float, float] | None] | None = None,
    dot_size: float | None = None,
    show_axes: bool = False,
    axes_stride: int = 1,
    history_overlays: list[ChunkOverlaySnapshot] | None = None,
) -> dict[str, float | None]:
    line_color = (60, 230, 180)
    errors: dict[str, float | None] = {"left": None, "right": None}
    try:
        stride = max(1, int(axes_stride))
    except Exception:
        stride = 1
    for arm in ("left", "right"):
        try:
            handle = scene_handles.get(f"{arm}_chunk_overlay")
            points_handle = scene_handles.get(f"{arm}_chunk_overlay_points")
            cursor_handle = scene_handles.get(f"{arm}_chunk_overlay_cursor")
            error_handle = scene_handles.get(f"{arm}_chunk_overlay_error")
            axes_handle = scene_handles.get(f"{arm}_chunk_overlay_axes")
            cursor_axes_handle = scene_handles.get(f"{arm}_chunk_overlay_cursor_axes")
            history_handle = scene_handles.get(f"{arm}_chunk_overlay_history")
            positions = None if overlay is None else getattr(overlay, f"{arm}_positions", None)
            poses = None if overlay is None else getattr(overlay, f"{arm}_poses", None)
            if handle is None or overlay is None or stale or not visible or not positions:
                _hide_chunk_overlay_handles(scene_handles, arm)
                errors[arm] = None
                continue

            point_colors = _chunk_overlay_point_colors(len(positions), line_color)
            if scene_handles.get("chunk_overlay_line_mode") == "line_segments":
                segments, seg_colors = _trail_line_arrays(positions, line_color)
                handle.points = segments
                handle.colors = seg_colors
                _set_visible(handle, len(positions) >= 2)
            else:
                handle.points = _points_array(positions)
                handle.colors = point_colors
                _set_visible(handle, True)

            if history_handle is not None and history_overlays:
                history_segments, history_colors = _chunk_overlay_history_line_arrays(history_overlays, arm)
                history_handle.points = history_segments
                history_handle.colors = history_colors
                _set_visible(history_handle, len(history_segments) > 0)
            else:
                _set_visible(history_handle, False)

            if points_handle is not None:
                points_handle.points = _points_array(positions)
                points_handle.colors = point_colors
                if dot_size is not None:
                    try:
                        if hasattr(points_handle, "point_size"):
                            points_handle.point_size = float(dot_size)
                    except Exception:
                        pass
                _set_visible(points_handle, True)

            cursor = overlay.cursor_position(arm, now=now_monotonic)
            if cursor is not None and cursor_handle is not None:
                _set_chunk_cursor_size(cursor_handle, dot_size)
                try:
                    cursor_handle.position = cursor
                except Exception:
                    try:
                        cursor_handle.points = _points_array((cursor,))
                        cursor_handle.colors = _colors_array(((255, 255, 90),))
                        _set_chunk_cursor_size(cursor_handle, dot_size)
                    except Exception:
                        pass
                _set_visible(cursor_handle, True)
            else:
                _set_visible(cursor_handle, False)

            if show_axes and poses and axes_handle is not None:
                segments = []
                colors = []
                for pose in poses[::stride]:
                    pose_segments, pose_colors = _pose_triad_segments(pose, 0.03)
                    segments.extend(pose_segments.tolist())
                    colors.extend(pose_colors.tolist())
                axes_handle.points = _line_segments_array(segments)
                axes_handle.colors = _line_segment_colors_array(colors)
                _set_visible(axes_handle, len(segments) > 0)
            else:
                _set_visible(axes_handle, False)

            cursor_pose = overlay.cursor_pose(arm, now=now_monotonic) if show_axes else None
            if cursor_pose is not None and cursor_axes_handle is not None:
                cursor_segments, cursor_colors = _pose_triad_segments(cursor_pose, 0.045)
                cursor_axes_handle.points = cursor_segments
                cursor_axes_handle.colors = cursor_colors
                _set_visible(cursor_axes_handle, True)
            else:
                _set_visible(cursor_axes_handle, False)

            actual = actual_positions.get(arm) if actual_positions else None
            if cursor is not None and actual is not None:
                actual_pos = tuple(float(value) for value in actual)
                if len(actual_pos) != 3:
                    raise ValueError("actual position must be xyz")
                error_m = math.sqrt(
                    (cursor[0] - actual_pos[0]) ** 2
                    + (cursor[1] - actual_pos[1]) ** 2
                    + (cursor[2] - actual_pos[2]) ** 2
                )
                color = (60, 220, 90) if error_m <= 0.015 else (230, 50, 50)
                if error_handle is not None:
                    segment = ((cursor, actual_pos),)
                    error_handle.points = _line_segments_array(segment)
                    error_handle.colors = _line_segment_colors_array(((color, color),))
                    _set_visible(error_handle, True)
                errors[arm] = error_m
            else:
                _set_visible(error_handle, False)
                errors[arm] = None
        except Exception:
            _hide_chunk_overlay_handles(scene_handles, arm)
            errors[arm] = None
    return errors


def _arm_is_controller_sim(arm_state: Any) -> bool:
    """True for rbpodo controller (pgmode) simulation (operation_mode: simulation)."""
    csm = getattr(arm_state, "controller_simulation_mode", None)
    if not csm:
        return False
    try:
        return str(csm.get("operation_mode", "")).lower() in ("simulation", "sim")
    except AttributeError:
        return False


def update_scene_markers(
    scene_handles: dict[str, Any],
    latest: Any,
    *,
    tcp_display_mode: str | None = None,
    show_tcp_command_trail: bool = False,
    show_tcp_gizmo: bool = True,
) -> None:
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
        target_key = f"{arm}_tcp_target"
        if target_key not in scene_handles or f"{arm}_tcp_target_user_moved" not in scene_handles:
            updates[target_key] = position
            scene_handles[f"{arm}_tcp_target_pose"] = _pose6_from_transform(position, wxyz)
            scene_handles[f"{arm}_tcp_target_wxyz"] = _normalize_wxyz(wxyz)
        for key in (f"{arm}_tcp", f"{arm}_tcp_target"):
            handle = scene_handles.get(key)
            if handle is not None:
                try:
                    handle.visible = show_tcp_gizmo if key == f"{arm}_tcp_target" else True
                except Exception:
                    pass

    for arm, (position, wxyz) in ref_updates.items():
        updates[f"{arm}_tcp_ref"] = position
        updates[f"{arm}_tcp_ref_label"] = _tcp_label_position(position)
        _set_visible(
            scene_handles.get(f"{arm}_tcp_ref"),
            display_mode in {"reference", "both"} or (
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
        _set_visible(scene_handles.get(f"{arm}_tcp_ref_label"), ref_visible)
        _set_visible(scene_handles.get(f"{arm}_tcp_trail"), False)
        _set_visible(scene_handles.get(f"{arm}_tcp_ref_trail"), False)
        _set_visible(scene_handles.get(f"{arm}_tcp_cmd_trail"), False)
        if arm not in ref_updates:
            _set_visible(scene_handles.get(f"{arm}_tcp_ref"), False)
            _set_visible(scene_handles.get(f"{arm}_tcp_ref_trail"), False)
            _set_visible(scene_handles.get(f"{arm}_tcp_ref_label"), False)
        if arm not in actual_updates:
            _set_visible(scene_handles.get(f"{arm}_tcp"), False)
            _set_visible(scene_handles.get(f"{arm}_tcp_trail"), False)
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
    if not show_tcp_gizmo:
        for arm in ("left", "right"):
            _set_visible(scene_handles.get(f"{arm}_tcp_target"), False)


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
