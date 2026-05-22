from __future__ import annotations

import math
import os
import time
from pathlib import Path
from typing import Any

from .command_client import CommandClient
from .safety import OperatorSafety, Readiness
from .state_receiver import StateReceiver, StateStore

_ROBOT_JOINT_NAMES = (
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist1_joint",
    "wrist2_joint",
    "wrist3_joint",
)
_DEFAULT_LEFT_POSE = (0.1601, -0.1725, 0.5825, 0.785, 2.35619, 0.0)
_DEFAULT_RIGHT_POSE = (-0.1601, -0.1725, 0.5825, 0.785, -2.35619, 0.0)
_DEFAULT_STAND_MESH_POSE = (0.0, 0.0, 0.01, 0.0, 0.0, -1.57078)


def _env_int(name: str, fallback: int) -> int:
    try:
        return int(os.environ.get(name, str(fallback)))
    except ValueError:
        return fallback


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


def _pose6_from_mounts(mounts: dict[str, Any], arm: str, fallback: tuple[float, float, float, float, float, float]) -> tuple[float, float, float, float, float, float]:
    try:
        pose = mounts[arm]["base_pose_in_stand"]
        return (
            float(pose.get("x", fallback[0])),
            float(pose.get("y", fallback[1])),
            float(pose.get("z", fallback[2])),
            float(pose.get("rx", fallback[3])),
            float(pose.get("ry", fallback[4])),
            float(pose.get("rz", fallback[5])),
        )
    except Exception:
        return fallback


def _mount_position(mounts: dict[str, Any], arm: str, fallback: tuple[float, float, float]) -> tuple[float, float, float]:
    pose = _pose6_from_mounts(mounts, arm, (fallback[0], fallback[1], fallback[2], 0.0, 0.0, 0.0))
    return (pose[0], pose[1], pose[2])


def _pose_position(pose6: tuple[float, float, float, float, float, float]) -> tuple[float, float, float]:
    return (pose6[0], pose6[1], pose6[2])


def _rpy_to_wxyz(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def _pose_wxyz(pose6: tuple[float, float, float, float, float, float]) -> tuple[float, float, float, float]:
    return _rpy_to_wxyz(pose6[3], pose6[4], pose6[5])


def _wxyz_to_rpy(wxyz: tuple[float, float, float, float]) -> tuple[float, float, float]:
    w, x, y, z = wxyz
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (roll, pitch, yaw)


def _matrix_to_wxyz(matrix: Any) -> tuple[float, float, float, float]:
    m00 = float(matrix[0][0])
    m01 = float(matrix[0][1])
    m02 = float(matrix[0][2])
    m10 = float(matrix[1][0])
    m11 = float(matrix[1][1])
    m12 = float(matrix[1][2])
    m20 = float(matrix[2][0])
    m21 = float(matrix[2][1])
    m22 = float(matrix[2][2])
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (m21 - m12) / scale
        y = (m02 - m20) / scale
        z = (m10 - m01) / scale
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / scale
        x = 0.25 * scale
        y = (m01 + m10) / scale
        z = (m02 + m20) / scale
    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / scale
        x = (m01 + m10) / scale
        y = 0.25 * scale
        z = (m12 + m21) / scale
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / scale
        x = (m02 + m20) / scale
        y = (m12 + m21) / scale
        z = 0.25 * scale
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 0.0 or not math.isfinite(norm):
        return (1.0, 0.0, 0.0, 0.0)
    return (w / norm, x / norm, y / norm, z / norm)


def _quat_to_matrix(wxyz: tuple[float, float, float, float]) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    w, x, y, z = wxyz
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def _matmul3(
    left: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
    right: Any,
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    return tuple(
        tuple(sum(left[row][k] * float(right[k][col]) for k in range(3)) for col in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def _rotate_vec(
    matrix: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    return tuple(sum(matrix[row][k] * vector[k] for k in range(3)) for row in range(3))  # type: ignore[return-value]


def _pose6_from_state_arm(arm_raw: Any, key: str) -> tuple[float, float, float, float, float, float] | None:
    if not isinstance(arm_raw, dict):
        return None
    pose = arm_raw.get(key)
    try:
        if isinstance(pose, dict):
            values = (pose["x"], pose["y"], pose["z"], pose["rx"], pose["ry"], pose["rz"])
        elif isinstance(pose, list | tuple) and len(pose) == 6:
            values = tuple(pose)
        else:
            return None
        parsed = tuple(float(value) for value in values)
    except Exception:
        return None
    if not all(math.isfinite(value) for value in parsed):
        return None
    return parsed  # type: ignore[return-value]


def _tcp_pose_from_urdf(
    urdf_handle: Any,
    base_pose: tuple[float, float, float, float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]] | None:
    try:
        urdf = urdf_handle._urdf
        base_frame = urdf.scene.graph.base_frame
        transform = urdf.get_transform("tcp", base_frame)
        scale = float(getattr(urdf_handle, "_scale", 1.0))
        local_position = (
            float(transform[0][3]) * scale,
            float(transform[1][3]) * scale,
            float(transform[2][3]) * scale,
        )
        base_rotation = _quat_to_matrix(_pose_wxyz(base_pose))
        rotated_position = _rotate_vec(base_rotation, local_position)
        position = (
            base_pose[0] + rotated_position[0],
            base_pose[1] + rotated_position[1],
            base_pose[2] + rotated_position[2],
        )
        rotation = _matrix_to_wxyz(_matmul3(base_rotation, transform[:3, :3]))
        return position, rotation
    except Exception:
        return None


def _pose6_from_transform(
    position: tuple[float, float, float],
    wxyz: tuple[float, float, float, float],
) -> tuple[float, float, float, float, float, float]:
    roll, pitch, yaw = _wxyz_to_rpy(wxyz)
    return (float(position[0]), float(position[1]), float(position[2]), roll, pitch, yaw)


def _format_joints(q_values: tuple[float, ...]) -> str:
    return ", ".join(f"{value:.2f}" for value in q_values)


def _joint_cfg_radians(q_values: tuple[float, ...]) -> tuple[float, ...]:
    padded = tuple(float(q_values[index]) if index < len(q_values) else 0.0 for index in range(len(_ROBOT_JOINT_NAMES)))
    return tuple(math.radians(value) for value in padded)


def _joint_marker_position(base: tuple[float, float, float], q_values: tuple[float, ...]) -> tuple[float, float, float]:
    # Marker-only fallback: not FK. It gives operators visible left/right state
    # changes without pretending Cartesian kinematics are available.
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
    left_pose = _pose6_from_mounts(mounts, "left", _DEFAULT_LEFT_POSE)
    right_pose = _pose6_from_mounts(mounts, "right", _DEFAULT_RIGHT_POSE)
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
    raw = latest.raw if isinstance(latest.raw, dict) else {}
    tcp_updates: dict[str, tuple[tuple[float, float, float], tuple[float, float, float, float]]] = {}
    for arm, arm_state, base_pose in (("left", latest.left, left_pose), ("right", latest.right, right_pose)):
        tcp_pose = _pose6_from_state_arm(raw.get(arm), "tcp_stand")
        if tcp_pose is not None:
            tcp_updates[arm] = (_pose_position(tcp_pose), _pose_wxyz(tcp_pose))
            continue
        urdf_tcp = _tcp_pose_from_urdf(scene_handles.get(f"{arm}_urdf"), base_pose)
        if urdf_tcp is not None:
            tcp_updates[arm] = urdf_tcp
            continue
        tcp_updates[arm] = (_joint_marker_position(_pose_position(base_pose), arm_state.q_actual_deg), _pose_wxyz(base_pose))

    for arm, (position, wxyz) in tcp_updates.items():
        updates[f"{arm}_tcp"] = position
        target_key = f"{arm}_tcp_target"
        if target_key not in scene_handles or f"{arm}_tcp_target_user_moved" not in scene_handles:
            updates[target_key] = position
            scene_handles[f"{arm}_tcp_target_pose"] = _pose6_from_transform(position, wxyz)

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
                scene_handles[f"{arm}_tcp_target_pose"] = _pose6_from_transform(position, wxyz)
                scene_handles[f"{arm}_tcp_target_user_moved"] = True
                if status_handle is not None:
                    status_handle.value = f"{arm} TCP target updated; press a send button to command from the gizmo"
            except Exception as exc:
                scene_handles["tcp_target_error"] = f"{type(exc).__name__}: {exc}"


def build_gui(server: Any, safety: OperatorSafety, store: StateStore) -> dict[str, Any]:
    handles: dict[str, Any] = {}
    handles["scene"] = _add_scene_fallback(server)

    tabs = server.gui.add_tab_group()
    with tabs.add_tab("Status"):
        handles["connection"] = server.gui.add_text("Connection", initial_value="disconnected", disabled=True)
        handles["mode"] = server.gui.add_text("Observed mode", initial_value=safety.observed_server_mode, disabled=True)
        handles["readiness"] = server.gui.add_text("Readiness", initial_value="No-Go: no state", disabled=True)
        handles["motion"] = server.gui.add_text("Motion state", initial_value="unknown", disabled=True)
        handles["fault"] = server.gui.add_text("Fault", initial_value="none", disabled=True)
        handles["ops"] = server.gui.add_text(
            "Container ops",
            initial_value="manual compose commands only; no Docker socket in GUI",
            disabled=True,
        )
        mode_group = server.gui.add_button_group("Desired mode", ("mock", "simulation", "real"))
        handles["mode_group"] = mode_group

        @mode_group.on_click
        def _(_: Any) -> None:
            safety.set_desired_mode(mode_group.value)

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

    with tabs.add_tab("TCP target"):
        handles["tcp_status"] = server.gui.add_text(
            "TCP status",
            initial_value="Move the left/right TCP gizmos, then send a TCP pose target",
            disabled=True,
        )
        _install_tcp_target_callbacks(handles["scene"], handles["tcp_status"])
        send_left_tcp = server.gui.add_button("Send left TCP target")
        send_right_tcp = server.gui.add_button("Send right TCP target")
        send_both_tcp = server.gui.add_button("Send both TCP targets")
        handles["tcp_pose_buttons"] = (send_left_tcp, send_right_tcp, send_both_tcp)

        def _target_pose(arm: str) -> tuple[float, ...] | None:
            pose = handles["scene"].get(f"{arm}_tcp_target_pose")
            if isinstance(pose, tuple) and len(pose) == 6:
                return pose
            return None

        @send_left_tcp.on_click
        def _(_: Any) -> None:
            ok, message = safety.send_tcp_pose_target(left_pose=_target_pose("left"))
            handles["tcp_status"].value = ("OK: " if ok else "BLOCKED: ") + message

        @send_right_tcp.on_click
        def _(_: Any) -> None:
            ok, message = safety.send_tcp_pose_target(right_pose=_target_pose("right"))
            handles["tcp_status"].value = ("OK: " if ok else "BLOCKED: ") + message

        @send_both_tcp.on_click
        def _(_: Any) -> None:
            ok, message = safety.send_tcp_pose_target(left_pose=_target_pose("left"), right_pose=_target_pose("right"))
            handles["tcp_status"].value = ("OK: " if ok else "BLOCKED: ") + message

    with tabs.add_tab("Debug"):
        handles["tick"] = server.gui.add_number("tick", initial_value=0, disabled=True)
        scene_errors = [
            value
            for key, value in handles.get("scene", {}).items()
            if key.endswith("_error") or key == "scene_error"
        ]
        handles["scene_assets"] = server.gui.add_text(
            "scene assets",
            initial_value="; ".join(scene_errors) if scene_errors else "stand/URDF assets loaded or pending",
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


def _set_disabled(handle: Any, disabled: bool) -> None:
    try:
        handle.disabled = disabled
    except Exception:
        pass


def update_gui(handles: dict[str, Any], safety: OperatorSafety, store: StateStore) -> None:
    disabled_states = safety.control_disabled_states()
    for mode, button in handles.get("lifecycle_buttons", {}).items():
        _set_disabled(button, disabled_states.get(f"lifecycle:{mode}", True))
    if "jog_button" in handles:
        _set_disabled(handles["jog_button"], disabled_states.get("jog", True))
    for button in handles.get("tcp_pose_buttons", ()):
        _set_disabled(button, disabled_states.get("tcp_pose", True))

    if "mode_group" in handles:
        current_mode = handles["mode_group"].value
        if current_mode != safety.desired_mode:
            safety.set_desired_mode(current_mode)
    latest = store.latest()
    stale = store.is_stale()
    readiness = safety.readiness()
    if latest is None:
        handles["connection"].value = "disconnected/stale"
        handles["readiness"].value = readiness.no_go_reason or "No-Go: no state stream"
        handles["packets"].value = f"{store.received_packets} received / {store.invalid_packets} invalid"
        return

    handles["connection"].value = "stale" if stale else "live"
    handles["mode"].value = f"desired={safety.desired_mode}, observed={safety.observed_server_mode}"
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
    update_scene_markers(handles.get("scene", {}), latest)
    if "scene_assets" in handles:
        scene_errors = [
            value
            for key, value in handles.get("scene", {}).items()
            if key.endswith("_error") or key == "scene_error"
        ]
        handles["scene_assets"].value = "; ".join(scene_errors) if scene_errors else "stand/URDF assets loaded"
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
    handles["arm_modes"].value = f"left={latest.left.mode}/{latest.left.connection_state} right={latest.right.mode}/{latest.right.connection_state}"
    handles["logger"].value = str(latest.logger_health)
    handles["packets"].value = f"{store.received_packets} received / {store.invalid_packets} invalid"


def main() -> None:
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
    observed_mode = os.environ.get("RB_GUI_OBSERVED_MODE", "mock")
    ops_available = os.environ.get("RB_GUI_OPS_AVAILABLE", "0") == "1"

    store = StateStore()
    receiver = StateReceiver(store, host=state_host, port=state_port)
    receiver.start()
    safety = OperatorSafety(
        store,
        CommandClient(command_host, command_port),
        desired_mode="mock",
        observed_server_mode=observed_mode,  # type: ignore[arg-type]
        sim_readiness=Readiness(no_go_reason="rbpodo/rbsim readiness tests have not passed"),
        ops_available=ops_available,
    )
    server = viser.ViserServer(host=host, port=port)
    handles = build_gui(server, safety, store)
    print(f"rb_servo_gui listening on http://{host}:{port}, UDP state {state_host}:{state_port}", flush=True)

    try:
        while True:
            update_gui(handles, safety, store)
            time.sleep(0.1)
    finally:
        receiver.stop()


if __name__ == "__main__":
    main()
