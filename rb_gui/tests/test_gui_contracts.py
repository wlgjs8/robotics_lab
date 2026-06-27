from __future__ import annotations

import json
import math
import os
import socket
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rb_servo_gui.app import (
    _DEFAULT_LEFT_POSE,
    _DEFAULT_INIT_LEFT_JOINTS_DEG,
    _DEFAULT_INIT_RIGHT_JOINTS_DEG,
    _DEFAULT_RIGHT_POSE,
    _TCP_DISPLAY_MODES,
    _TCP_FRAME_LOCAL,
    _TCP_FRAME_STAND,
    _apply_tcp_pose_step_and_send_pose_target,
    _apply_tcp_pose_step_to_target,
    _gripper_cmd_endpoint,
    _recording_cmd_endpoint,
    _recording_status_bind_endpoint,
    _push_gripper_percent,
    _toggle_episode_recording,
    _update_recording_panel,
    _send_arm_init_override,
    _update_arm_init_panel,
    _lifecycle_init_motion_layout_html,
    _update_gripper_feedback,
    _viser_keyboard_patch_script,
    _send_gripper_command,
    _user_floor_display_points,
    _apply_init_joints_live,
    _nudge_label,
    _status_summary_html,
    _tab_theme_html,
    _angular_step_radians,
    _current_joints_text,
    _delete_waypoint,
    _format_joint6,
    _parse_joint6,
    _load_init_joints,
    _load_waypoints,
    _save_init_joints,
    _save_waypoints,
    _set_waypoint_as_init,
    _build_operator_monitors,
    _circle_overlay_bind_from_args_env,
    _env_joint6,
    _format_cartesian_solve_status,
    _format_circle_overlay_status,
    _format_fk_status,
    _format_init_motion_status,
    _format_scene_asset_status,
    _format_joint_monitor_value,
    _format_joints,
    _format_pgmode_status,
    _format_stand_world_pose_value,
    _format_tcp_tracking_status,
    _joint_cfg_radians,
    _joint_marker_position,
    _linear_step_meters,
    _mode_button_color,
    _mount_position,
    _mount_pose_from_mounts,
    _format_hms,
    _operator_monitor_dynamic_html,
    _operator_monitor_static_html,
    _server_uptime_hms,
    _pose6_from_mounts,
    _pose_orientation_wxyz,
    _pose_wxyz,
    _quat_to_matrix,
    _clear_tcp_target_user_moved,
    _reflect_gate_reason,
    _send_tcp_linear_move_from_marker,
    _send_init_motion_and_reset_targets,
    _update_floor_panel,
    _update_roi_panel,
    _update_lease_owner,
    _tcp_display_mode,
    _tcp_local_delta_from_target,
    _tcp_frame_mode,
    _tcp_linear_arm,
    _tcp_linear_orientation_mode,
    _tcp_target_pose,
    _tcp_target_wxyz,
    _read_ptp_arm_fields,
    _refresh_tcp_ptp_axis_fields,
    _send_tcp_poses_absolute,
    _set_tcp_pose_absolute_and_send,
    _stand_world_monitor_pose,
    _update_joint_monitor,
    _update_joint_monitor_unit_buttons,
    _update_operator_monitors,
    _update_stand_world_monitor,
    _update_stand_world_monitor_unit_buttons,
    _update_tcp_display_buttons,
    _update_tcp_linear_selection_buttons,
    _update_tcp_frame_buttons,
    _wxyz_to_xyzw,
    parse_args,
    update_scene_markers,
)
from rb_servo_gui.command_client import CommandClient
from rb_servo_gui import geometry as gui_geometry
from rb_servo_gui.models import CIRCLE_OVERLAY_SCHEMA_VERSION, CircleOverlaySnapshot, Pose6D, StateSnapshot
from rb_servo_gui.overlay_receiver import CircleOverlayReceiver, CircleOverlayStore, parse_udp_bind
from rb_servo_gui.recording_control import (
    ARM_INIT_COMMAND_SCHEMA,
    ArmInitCommandResult,
    RecordingCommandClient,
    RecordingStatusStore,
)
from rb_servo_gui.safety import OperatorSafety, normalize_observed_mode_backend
from rb_servo_gui.scene import (
    _FLOOR_CHECK_POINTS_TCP_FRAME,
    _FLOOR_CHECK_POINTS_TCP_FRAME_CLOSED,
    _add_ik_infeasible_region,
    _add_robot_urdfs,
    _add_scene_fallback,
    _attach_checkgeom_gripper,
    _circle_overlay_points,
    _ensure_sc_world_frame,
    _finger_position_m,
    _interpolated_floor_check_points,
    _ik_infeasible_path,
    _reference_ghost_active,
    _robot_urdf_path,
    _update_urdf_config,
    set_ik_infeasible_region_visible,
    update_circle_overlay,
    update_floor_check_points,
    update_floor_plane,
    update_floor_plane_preview,
    update_roi_box,
    update_roi_box_preview,
    update_self_collision_check_geom,
    update_self_collision_near_pairs,
    update_self_collision_overlay,
    update_user_floor_plane,
)
from rb_servo_gui.status_panel import (
    _format_floor_constraint_status,
    _format_user_floor_constraint_status,
    _format_roi_box_status,
    _format_self_collision_status,
)
from rb_servo_gui.state_receiver import StateStore


def _local_udp_socket_available() -> bool:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(("127.0.0.1", 0))
    except OSError:
        return False
    return True


def sample_state(**overrides):
    arm = {
        "mode": "Hold",
        "q_actual_deg": [0, -30, 80, 0, 60, 0],
        "q_sent_deg": [0, -30, 80, 0, 60, 0],
        "q_previous_sent_deg": [0, -30, 80, 0, 60, 0],
        "send_ok": True,
        "send_start_ns": 1,
        "send_end_ns": 2,
        "send_duration_us": 1.0,
        "has_valid_joint_state": True,
        "connection_state": "Connected",
        "robot_time_ns": 1,
        "host_time_ns": 2,
        "error_code": 0,
        "tcp_stand": None,
        "tcp_base": None,
        "tcp_actual_stand": None,
        "tcp_actual_base": None,
        "tcp_ref_stand": None,
        "tcp_ref_base": None,
        "has_valid_tcp_pose": False,
        "tcp_actual_valid": None,
        "tcp_ref_valid": None,
        "tcp_tracking_source": None,
        "tcp_tracking_source_recommendation": None,
        "controller_simulation_mode": None,
        "physical_motion_expected": None,
        "controller_simulation_diagnostic_override_active": None,
        "tcp_deferred": True,
    }
    data = {
        "schema_version": 1,
        "tick": 1,
        "loop_start_time_ns": 1,
        "loop_end_time_ns": 2,
        "host_time_ns": 2,
        "period_ms": 5.0,
        "jitter_ms": 0.0,
        "filter_dt_ms": 5.0,
        "command_seq": 1,
        "left": dict(arm),
        "right": dict(arm),
        "send_skew_us": 0.0,
        "safety_verdict": "Ok",
        "motion_state": "ConnectedHold",
        "fault_latched": False,
        "latched_fault_reason": "Ok",
        "fault_reason": "",
        "logger_health": {"ok": True, "dropped_samples": 0},
        "mounts": {},
        "tcp_fields_deferred": True,
    }
    data.update(overrides)
    return data


def sample_circle_overlay(**overrides):
    data = {
        "schema_version": CIRCLE_OVERLAY_SCHEMA_VERSION,
        "host_time_ns": 123456789,
        "run_id": "run-1",
        "arm": "left",
        "profile": "gene_15cm_4s",
        "controller": "twist_stand_feedback",
        "tracking_source": "tcp_ref_stand",
        "plane": "xy",
        "center_stand": [0.1, 0.2, 0.3],
        "axis1_stand": [1.0, 0.0, 0.0],
        "axis2_stand": [0.0, 1.0, 0.0],
        "radius_m": 0.075,
        "period_sec": 4.0,
        "phase_rad": 0.5,
        "desired_pose_stand": {
            "x": 0.175,
            "y": 0.2,
            "z": 0.3,
            "rx": 0.0,
            "ry": 0.0,
            "rz": 0.0,
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
        },
        "current_error_m": 0.001,
        "running_rms_error_m": 0.002,
        "running_p95_error_m": 0.003,
        "estimated_latency_ms": 12.3,
        "sample_count": 4,
        "command_count": 5,
        "physical_motion_expected": False,
        "result_so_far": "running",
    }
    data.update(overrides)
    return data


class RecordingClient(CommandClient):
    def __init__(self):
        super().__init__("127.0.0.1", 9)

    def send(self, packet):
        self.sent_packets.append(dict(packet))


class RecordingCommandFake:
    def __init__(self):
        self.calls = []
        self.arm_init_calls = []

    def send(self, command, *, task="", operator=None):
        self.calls.append({"command": command, "task": task, "operator": operator})
        return True

    def send_arm_init(self, arms, *, left_q_deg=None, right_q_deg=None, action="toggle"):
        self.arm_init_calls.append(
            {
                "arms": arms,
                "action": action,
                "left_q_deg": left_q_deg,
                "right_q_deg": right_q_deg,
            }
        )
        return ArmInitCommandResult(True, f"arm init {arms} toggle sent")


class RecordingSceneHandle:
    def __init__(self):
        self.position = None
        self.wxyz = None
        self.points = None
        self.colors = None
        self.visible = None
        self.text = None


class RecordingButton:
    def __init__(self, color="gray"):
        self.color = color

    def on_click(self, callback):
        self.callback = callback
        return callback


class RecordingText:
    def __init__(self, value=""):
        self.value = value


class RecordingHtml:
    def __init__(self, content=""):
        self.content = content


class RecordingContext:
    def __init__(self, label=None):
        self.label = label

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return None


class RecordingGui:
    def __init__(self, *, has_html=True):
        self.folders = []
        self.buttons = []
        self.html = []
        self.texts = []
        if has_html:
            self.add_html = self._add_html

    def add_folder(self, label, **kwargs):
        self.folders.append((label, kwargs))
        return RecordingContext(label)

    def add_button(self, label, **kwargs):
        button = RecordingButton(color=kwargs.get("color", "gray"))
        self.buttons.append((label, kwargs, button))
        return button

    def add_text(self, label, **kwargs):
        text = RecordingText(kwargs.get("initial_value", ""))
        self.texts.append((label, kwargs, text))
        return text

    def _add_html(self, content, **kwargs):
        html = RecordingHtml(content)
        self.html.append((content, kwargs, html))
        return html


class RecordingScene:
    def __init__(self):
        self.containers = []
        self.frames = []
        self.labels = []

    def add_3d_gui_container(self, name, **kwargs):
        self.containers.append((name, kwargs))
        return RecordingContext(name)

    def add_frame(self, name, **kwargs):
        handle = RecordingSceneHandle()
        handle.position = kwargs.get("position")
        handle.wxyz = kwargs.get("wxyz")
        handle.visible = kwargs.get("visible", True)
        self.frames.append((name, kwargs, handle))
        return handle

    def add_label(self, name, text, **kwargs):
        handle = RecordingSceneHandle()
        handle.text = text
        handle.position = kwargs.get("position")
        handle.visible = kwargs.get("visible", True)
        self.labels.append((name, text, kwargs, handle))
        return handle


class ShapeCheckingScene(RecordingScene):
    def __init__(self):
        super().__init__()
        self.point_clouds = []
        self.line_segments = []
        self.meshes = []
        self.icospheres = []
        self.transform_controls = []

    def add_point_cloud(self, name, points, colors, **kwargs):
        points_array = np.asarray(points)
        colors_array = np.asarray(colors)
        self.assert_point_cloud_arrays(points_array, colors_array)
        handle = RecordingSceneHandle()
        handle.points = points_array
        handle.colors = colors_array
        handle.visible = kwargs.get("visible", True)
        self.point_clouds.append((name, points_array, colors_array, kwargs, handle))
        return handle

    def add_line_segments(self, name, points, colors, **kwargs):
        points_array = np.asarray(points)
        colors_array = np.asarray(colors)
        self.assert_line_segment_arrays(points_array, colors_array)
        handle = RecordingSceneHandle()
        handle.points = points_array
        handle.colors = colors_array
        handle.visible = kwargs.get("visible", True)
        self.line_segments.append((name, points_array, colors_array, kwargs, handle))
        return handle

    def add_transform_controls(self, name, **kwargs):
        handle = RecordingSceneHandle()
        handle.position = kwargs.get("position")
        handle.wxyz = kwargs.get("wxyz")
        handle.visible = kwargs.get("visible", True)
        self.transform_controls.append((name, kwargs, handle))
        return handle

    def add_mesh_trimesh(self, name, **kwargs):
        handle = RecordingSceneHandle()
        handle.position = kwargs.get("position")
        handle.wxyz = kwargs.get("wxyz")
        handle.visible = kwargs.get("visible", True)
        self.meshes.append((name, kwargs, handle))
        return handle

    def add_icosphere(self, name, **kwargs):
        handle = RecordingSceneHandle()
        handle.position = kwargs.get("position")
        handle.visible = kwargs.get("visible", True)
        self.icospheres.append((name, kwargs, handle))
        return handle

    @staticmethod
    def assert_point_cloud_arrays(points, colors):
        assert points.ndim == 2 and points.shape[-1] == 3
        assert colors.shape in {points.shape, (3,)}

    @staticmethod
    def assert_line_segment_arrays(points, colors):
        assert points.ndim == 3 and points.shape[1:] == (2, 3)
        assert colors.shape in {points.shape, (3,)}


class RecordingServer:
    def __init__(self, *, scene=None, has_html=True):
        self.gui = RecordingGui(has_html=has_html)
        self.scene = scene


class RecordingUrdf:
    def __init__(self):
        self.configs = []

    def update_cfg(self, config):
        self.configs.append(tuple(float(value) for value in config))


class GuiContractsTest(unittest.TestCase):
    def make_safety(
        self,
        state=None,
        *,
        desired="mock",
        observed="mock",
        observed_backend=None,
        sim_ready=False,  # retired: readiness is state-derived (accepted for call-site compat)
        cartesian_available=None,
        enable_tcp_pose=False,  # retired: TCP commands are always wired
        enable_controller_sim_cartesian=False,  # retired: server Cartesian gate is authoritative
        stale=False,
        init_left_joint_deg=None,
        init_right_joint_deg=None,
        init_motion_timeout_sec=10.0,
    ):
        store = StateStore(stale_after_sec=0.5)
        if state is not None:
            # Cartesian availability is now derived from server state, not an env
            # flag: inject the requested value onto each arm's gate fields.
            if cartesian_available is not None:
                for arm in ("left", "right"):
                    arm_state = state.setdefault(arm, {})
                    arm_state["cartesian_available"] = cartesian_available
                    arm_state["controller_simulation_streaming_cartesian_available"] = cartesian_available
            received = time.monotonic() - 1.0 if stale else time.monotonic()
            self.assertTrue(store.update_from_json_bytes(json.dumps(state).encode(), received_monotonic=received))
        client = RecordingClient()
        safety = OperatorSafety(
            store,
            client,
            desired_mode=desired,
            observed_server_mode=observed,
            observed_backend=observed_backend,
            init_left_joint_deg=init_left_joint_deg,
            init_right_joint_deg=init_right_joint_deg,
            init_motion_timeout_sec=init_motion_timeout_sec,
        )
        return store, client, safety

    def tcp_available_state(self, **overrides):
        state = sample_state(**overrides)
        for arm in ("left", "right"):
            state[arm]["tcp_stand"] = {"x": 0.31, "y": 0.12, "z": 0.44, "rx": 0.0, "ry": 0.0, "rz": 0.0}
            state[arm]["tcp_base"] = {"x": 0.2, "y": 0.1, "z": 0.4, "rx": 0.0, "ry": 0.0, "rz": 0.0}
            state[arm]["tcp_actual_stand"] = {"x": 0.31, "y": 0.12, "z": 0.44, "rx": 0.0, "ry": 0.0, "rz": 0.0}
            state[arm]["tcp_actual_base"] = {"x": 0.2, "y": 0.1, "z": 0.4, "rx": 0.0, "ry": 0.0, "rz": 0.0}
            state[arm]["has_valid_tcp_pose"] = True
            state[arm]["tcp_actual_valid"] = True
            state[arm]["tcp_deferred"] = False
        state["tcp_fields_deferred"] = False
        return state

    def test_init_motion_runtime_diag_parses_and_formats_failures(self):
        state = sample_state(
            init_motion={
                "status": "failed",
                "fail_mode": "goal_not_clear",
                "message": "goal blocked",
                "start_clear_m": 0.012,
                "goal_clear_m": -0.002,
                "tree_start": 0,
                "tree_goal": 0,
                "iterations": 0,
                "planning_time_s": 0.01,
                "waypoint_index": 0,
                "waypoint_count": 0,
                "dist_to_goal_deg": None,
            }
        )
        latest = StateSnapshot.parse(state)
        self.assertIsNotNone(latest)
        self.assertEqual(latest.init_motion["fail_mode"], "goal_not_clear")
        text = _format_init_motion_status(latest, stale=False)
        self.assertIn("Init 자세가 충돌/바닥 침범", text)
        self.assertIn("goal_clear=-2.0mm", text)

    def test_init_motion_runtime_diag_formats_rrt_and_execution(self):
        rrt = StateSnapshot.parse(sample_state(init_motion={
            "status": "failed",
            "fail_mode": "rrt_budget",
            "tree_start": 123,
            "tree_goal": 98,
            "iterations": 456,
            "planning_time_s": 5.0,
        }))
        self.assertIn("tree 123/98", _format_init_motion_status(rrt, stale=False))
        self.assertIn("iters 456", _format_init_motion_status(rrt, stale=False))

        exec_failed = StateSnapshot.parse(sample_state(init_motion={
            "status": "failed",
            "fail_mode": "exec_timeout",
            "waypoint_index": 7,
            "waypoint_count": 11,
            "dist_to_goal_deg": 2.5,
        }))
        text = _format_init_motion_status(exec_failed, stale=False)
        self.assertIn("실행 중단", text)
        self.assertIn("wp 7/11", text)

    def pgmode_spacemouse_state(self):
        state = self.tcp_available_state(
            observed_mode="real",
            observed_backend="rbpodo",
            command_source={
                "source_id": "policy_runner",
                "session_id": "policy-session",
                "active": True,
                "active_source_id": "policy_runner",
                "active_session_id": "policy-session",
                "lease_timeout_sec": 60.0,
                "expires_time_ns": 123456789,
                "command_requires_lease": True,
                "command_has_lease": True,
            },
        )
        for arm in ("left", "right"):
            state[arm]["mode"] = "TcpPoseTarget"
            state[arm]["tcp_ref_stand"] = {"x": 0.41, "y": 0.21, "z": 0.51, "rx": 0.0, "ry": 0.0, "rz": 0.0}
            state[arm]["tcp_ref_valid"] = True
            state[arm]["tcp_tracking_source"] = "tcp_ref_stand"
            state[arm]["tcp_tracking_source_recommendation"] = "reference_for_controller_simulation"
            state[arm]["physical_motion_expected"] = False
            state[arm]["controller_simulation_physical_motion_detected"] = False
            state[arm]["cartesian_available"] = True
            state[arm]["controller_simulation_cartesian_enabled"] = True
            state[arm]["controller_simulation_cartesian_enabled_for_current_command"] = True
            state[arm]["controller_simulation_streaming_cartesian_available"] = True
            state[arm]["cartesian_gate"] = {
                "run_mode": "real",
                "backend_type": "rbpodo",
                "operation_mode": "simulation",
                "allow_in_controller_simulation": True,
                "allow_in_real": False,
                "physical_motion_expected": False,
                "cartesian_available": True,
                "controller_simulation_cartesian_enabled": True,
                "controller_simulation_cartesian_enabled_for_current_command": True,
                "controller_simulation_streaming_cartesian_available": True,
                "controller_simulation_physical_motion_detected": False,
            }
        return state

    def test_all_actions_reachable_for_mock_simulator_and_vm_pgmode_sim(self):
        # End-to-end gate contract for the hardware-free stacks the GUI is opened
        # against. The server is the sole authority; the GUI mirrors it:
        #   - mock backend (no FK): joint controls open, Cartesian honestly gated
        #     by the server's FK/Cartesian gate (server cannot do Cartesian).
        #   - simulator stack and VM pgmode controller-sim (FK + open Cartesian
        #     gate): every motion primitive is reachable, no env unlock needed.
        joint_actions = ("JointTarget",)
        # Mock backend: valid joints, no TCP pose.
        _, _, mock_safety = self.make_safety(sample_state(), observed="mock", observed_backend="mock")
        for action in joint_actions:
            self.assertIsNone(mock_safety.blocked_reason(action), f"mock {action}")
        mock_states = mock_safety.control_disabled_states()
        self.assertFalse(mock_states["jog"])
        self.assertFalse(mock_states["lifecycle:ArmMotion"])
        self.assertTrue(mock_states["tcp_pose"])  # honest: server has no Cartesian here

        # Simulator stack (pinocchio FK, allow_in_simulation) and VM pgmode
        # controller-sim (rbpodo, operation_mode=simulation, streaming Cartesian).
        simulator_state = self.tcp_available_state(observed_mode="simulation", observed_backend="simulator")
        for arm in ("left", "right"):
            simulator_state[arm]["cartesian_available"] = True
        vm_pgmode_state = self.pgmode_spacemouse_state()
        for label, state, observed, backend in (
            ("simulator", simulator_state, "simulation", "simulator"),
            ("vm_pgmode_sim", vm_pgmode_state, "real", "rbpodo"),
        ):
            _, _, safety = self.make_safety(state, observed=observed, observed_backend=backend)
            for action in joint_actions:
                self.assertIsNone(safety.blocked_reason(action), f"{label} {action}")
            self.assertIsNone(safety.tcp_command_disabled_reason(), f"{label} tcp")
            self.assertIsNone(safety.tcp_command_disabled_reason("left"), f"{label} tcp left")
            self.assertIsNone(safety.tcp_command_disabled_reason("right"), f"{label} tcp right")
            states = safety.control_disabled_states()
            for key in ("jog", "init_motion", "tcp_pose", "tcp_linear"):
                # init_motion needs an armed state + configured target; ignore it here.
                if key == "init_motion":
                    continue
                self.assertFalse(states[key], f"{label} {key} should be enabled")
            self.assertTrue(safety.readiness().ready, f"{label} readiness")
            self.assertTrue(safety.readiness().cartesian_available, f"{label} cartesian")

    def pgmode_real_state(self):
        # pgmode-real: connects to the real boxes in operation_mode=real. The
        # server opens Cartesian (cartesian_available=True) but reports the
        # controller-simulation streaming flag False (that carve-out is closed).
        state = self.tcp_available_state(observed_mode="real", observed_backend="rbpodo")
        for arm in ("left", "right"):
            state[arm]["physical_motion_expected"] = True
            state[arm]["cartesian_available"] = True
            state[arm]["cartesian_unavailable_reason"] = None
            state[arm]["controller_simulation_streaming_cartesian_available"] = False
            state[arm]["cartesian_gate"] = {
                "run_mode": "real",
                "backend_type": "rbpodo",
                "operation_mode": "real",
                "allow_in_controller_simulation": False,
                "allow_in_real": True,
                "physical_motion_expected": True,
                "cartesian_available": True,
                "controller_simulation_streaming_cartesian_available": False,
            }
        return state

    def test_tcp_reachable_in_pgmode_real_despite_controller_sim_flag_false(self):
        # Regression: in real motion the server opens Cartesian via
        # cartesian_available=True while the controller-sim streaming flag is
        # False. The GUI must not let that controller-sim-only flag block real
        # TCP commands (the live lock the operator saw with make run MODE=real).
        _, _, safety = self.make_safety(self.pgmode_real_state(), observed="real", observed_backend="rbpodo")
        self.assertIsNone(safety.tcp_command_disabled_reason(), "pgmode-real tcp")
        self.assertIsNone(safety.tcp_command_disabled_reason("left"), "pgmode-real tcp left")
        self.assertIsNone(safety.tcp_command_disabled_reason("right"), "pgmode-real tcp right")
        states = safety.control_disabled_states()
        for key in ("tcp_pose", "tcp_linear"):
            self.assertFalse(states[key], f"pgmode-real {key} should be enabled")
        self.assertTrue(safety.readiness().cartesian_available, "pgmode-real cartesian")

    def test_valid_state_updates_latest_and_invalid_json_is_counted(self):
        store, _, safety = self.make_safety(sample_state())
        self.assertFalse(store.is_stale())
        self.assertEqual(store.latest().tick, 1)
        self.assertFalse(store.update_from_json_bytes(b"{not-json"))
        self.assertEqual(store.invalid_packets, 1)
        self.assertTrue(safety.readiness().ready)

    def test_invalid_joint_state_keeps_status_and_fault_visible(self):
        bad = sample_state(fault_latched=True, fault_reason="backend error")
        bad["left"]["has_valid_joint_state"] = False
        bad["left"]["q_actual_deg"] = [0, 0, 0]
        store, client, safety = self.make_safety(bad)
        latest = store.latest()
        self.assertIsNotNone(latest)
        self.assertFalse(latest.left.has_valid_joint_state)
        self.assertEqual(latest.fault_reason, "backend error")
        self.assertEqual(latest.left.connection_state, "Connected")
        self.assertTrue(safety.readiness().fault)
        ok, reason = safety.jog_joint("left", 0, 1.0)
        self.assertFalse(ok)
        self.assertIn("joint state invalid", reason)
        self.assertEqual(client.sent_packets, [])

    def test_parser_preserves_valid_tcp_pose_fields(self):
        state = sample_state()
        state["left"]["tcp_stand"] = {
            "x": 0.31,
            "y": 0.12,
            "z": 0.44,
            "rx": 0.0,
            "ry": 0.0,
            "rz": math.pi,
            "quaternion_xyzw": [0.0, 0.0, 2.0, 0.0],
        }
        state["left"]["tcp_base"] = {
            "x": 0.1,
            "y": 0.2,
            "z": 0.3,
            "rx": 0.0,
            "ry": 0.0,
            "rz": 1.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.0,
            "qw": 1.0,
        }
        state["left"]["has_valid_tcp_pose"] = True
        state["left"]["tcp_deferred"] = False
        store, _, _ = self.make_safety(state)
        latest = store.latest()
        self.assertIsInstance(latest.left.tcp_stand, Pose6D)
        self.assertEqual(latest.left.tcp_stand.as_tuple(), (0.31, 0.12, 0.44, 0.0, 0.0, math.pi))
        self.assertEqual(latest.left.tcp_base.as_tuple(), (0.1, 0.2, 0.3, 0.0, 0.0, 1.0))
        self.assertEqual(latest.left.tcp_stand.quaternion_xyzw, (0.0, 0.0, 1.0, 0.0))
        self.assertEqual(latest.left.tcp_base.quaternion_xyzw, (0.0, 0.0, 0.0, 1.0))
        self.assertTrue(latest.left.has_valid_tcp_pose)
        self.assertFalse(latest.left.tcp_deferred)

    def _fake_ptp_handles(self):
        # Per-axis vector2 widget: .value is a (left, right) tuple, like viser's.
        class _Vec:
            def __init__(self):
                self.value = (0.0, 0.0)

        vecs = {axis: _Vec() for axis in range(6)}
        handles = {
            "tcp_ptp_axis_vec": vecs,
            "tcp_frame_mode": _TCP_FRAME_STAND,
            "tcp_ptp_arm": "both",
            "scene": {},
        }
        return handles, vecs

    def test_pose_monitor_and_ptp_mirror_share_pose_selection(self):
        # The TCP PTP "current pose" mirror and the Pose Monitor must read the
        # exact same pose (incl. the server's rx/ry/rz euler) so they never
        # disagree. _stand_world_monitor_pose is the single shared selector.
        state = self.tcp_available_state()
        state["left"]["tcp_stand"] = {"x": 0.31, "y": 0.12, "z": 0.44, "rx": 0.2, "ry": -0.3, "rz": 1.1}
        store, _, _ = self.make_safety(state)
        latest = store.latest()
        pose, valid, is_sim = _stand_world_monitor_pose(latest.left, stale=False)
        self.assertTrue(valid)
        self.assertFalse(is_sim)
        self.assertEqual(pose.as_tuple(), latest.left.tcp_stand.as_tuple())
        # Stale stream invalidates the readout (matches the Monitor).
        _, valid_stale, _ = _stand_world_monitor_pose(latest.left, stale=True)
        self.assertFalse(valid_stale)

    def test_tcp_ptp_fields_mirror_current_pose_per_arm(self):
        # Each axis vector2 mirrors (left, right) live stand-frame pose
        # (mm / deg, server euler), per arm, in one paired row.
        state = self.tcp_available_state()
        state["left"]["tcp_stand"] = {"x": 0.31, "y": 0.12, "z": 0.44, "rx": 0.0, "ry": 0.0, "rz": math.pi}
        state["right"]["tcp_stand"] = {"x": -0.25, "y": 0.10, "z": 0.40, "rx": 0.1, "ry": 0.2, "rz": -0.3}
        store, _, _ = self.make_safety(state)
        latest = store.latest()
        handles, vecs = self._fake_ptp_handles()
        handles["_latest_state"] = latest
        handles["_state_stale"] = False
        self.assertTrue(_refresh_tcp_ptp_axis_fields(handles))
        # axis 0 = X: (left mm, right mm)
        self.assertAlmostEqual(vecs[0].value[0], 310.0, places=3)
        self.assertAlmostEqual(vecs[0].value[1], -250.0, places=3)
        self.assertAlmostEqual(vecs[1].value[0], 120.0, places=3)
        self.assertAlmostEqual(vecs[1].value[1], 100.0, places=3)
        # axis 5 = yaw (deg)
        self.assertAlmostEqual(vecs[5].value[0], math.degrees(math.pi), places=3)
        self.assertAlmostEqual(vecs[5].value[1], math.degrees(-0.3), places=3)
        # axis 3 = roll (deg)
        self.assertAlmostEqual(vecs[3].value[0], 0.0, places=3)
        self.assertAlmostEqual(vecs[3].value[1], math.degrees(0.1), places=3)
        # _read_ptp_arm_fields pulls one arm's six values from the vector2 slots.
        right_vals = _read_ptp_arm_fields(handles, "right")
        self.assertAlmostEqual(right_vals[0], -250.0, places=3)
        self.assertAlmostEqual(right_vals[5], math.degrees(-0.3), places=3)

    def test_tcp_ptp_fields_local_frame_read_zero(self):
        state = self.tcp_available_state()
        store, _, _ = self.make_safety(state)
        handles, vecs = self._fake_ptp_handles()
        handles["tcp_frame_mode"] = _TCP_FRAME_LOCAL
        handles["_latest_state"] = store.latest()
        handles["_state_stale"] = False
        _refresh_tcp_ptp_axis_fields(handles)
        for axis in range(6):
            self.assertEqual(vecs[axis].value, (0.0, 0.0))

    def test_tcp_ptp_absolute_commit_reproduces_orientation_without_drift(self):
        # Committing the mirrored values unchanged must command the EXACT current
        # orientation: the server builds rotation as Rz·Ry·Rx and so does
        # _rpy_to_wxyz, so feeding back the displayed euler causes no drift.
        from rb_servo_gui.geometry import _rpy_to_wxyz, _wxyz_to_xyzw

        class _CaptureSafety:
            def __init__(self):
                self.calls = []

            def send_tcp_pose_target(self, *, left_pose, right_pose, left_quaternion_xyzw, right_quaternion_xyzw):
                self.calls.append(
                    (left_pose, right_pose, left_quaternion_xyzw, right_quaternion_xyzw)
                )
                return True, "ok"

        safety = _CaptureSafety()
        scene = {}
        rx, ry, rz = 0.2, -0.3, 1.1
        values = [310.0, 120.0, 440.0, math.degrees(rx), math.degrees(ry), math.degrees(rz)]
        ok, message = _set_tcp_pose_absolute_and_send(safety, scene, "left", values)
        self.assertTrue(ok, message)
        self.assertEqual(len(safety.calls), 1)
        left_pose, right_pose, left_quat, right_quat = safety.calls[0]
        self.assertIsNone(right_pose)
        self.assertIsNone(right_quat)
        self.assertAlmostEqual(left_pose[0], 0.310, places=6)
        self.assertAlmostEqual(left_pose[1], 0.120, places=6)
        self.assertAlmostEqual(left_pose[2], 0.440, places=6)
        expected = _wxyz_to_xyzw(_rpy_to_wxyz(rx, ry, rz))
        for got, want in zip(left_quat, expected):
            self.assertAlmostEqual(got, want, places=9)

    def test_tcp_ptp_both_arms_sent_in_single_packet(self):
        # Regression: nudging/committing BOTH arms must send ONE packet carrying
        # both poses. Two back-to-back single-arm packets make the server reset the
        # other arm to Hold, so only the second arm moves.
        class _CaptureSafety:
            def __init__(self):
                self.calls = []

            def send_tcp_pose_target(self, *, left_pose, right_pose, left_quaternion_xyzw, right_quaternion_xyzw):
                self.calls.append((left_pose, right_pose, left_quaternion_xyzw, right_quaternion_xyzw))
                return True, "ok"

        safety = _CaptureSafety()
        scene = {}
        left_vals = [310.0, 120.0, 440.0, 0.0, 0.0, 0.0]
        right_vals = [-250.0, 100.0, 400.0, 0.0, 0.0, 0.0]
        ok, message = _send_tcp_poses_absolute(safety, scene, {"left": left_vals, "right": right_vals})
        self.assertTrue(ok, message)
        # Exactly ONE packet, carrying BOTH arms.
        self.assertEqual(len(safety.calls), 1)
        left_pose, right_pose, left_quat, right_quat = safety.calls[0]
        self.assertIsNotNone(left_pose)
        self.assertIsNotNone(right_pose)
        self.assertAlmostEqual(left_pose[0], 0.310, places=6)
        self.assertAlmostEqual(right_pose[0], -0.250, places=6)

    def test_parser_preserves_actual_and_reference_tcp_pose_fields(self):
        state = sample_state()
        state["left"]["tcp_stand"] = {"x": 0.31, "y": 0.12, "z": 0.44, "rx": 0.0, "ry": 0.0, "rz": 0.0}
        state["left"]["tcp_actual_stand"] = {
            "x": 0.32,
            "y": 0.13,
            "z": 0.45,
            "rx": 0.0,
            "ry": 0.0,
            "rz": 0.0,
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
        }
        state["left"]["tcp_actual_base"] = {"x": 0.22, "y": 0.11, "z": 0.41, "rx": 0.0, "ry": 0.0, "rz": 0.1}
        state["left"]["tcp_ref_stand"] = {
            "x": 0.41,
            "y": 0.21,
            "z": 0.51,
            "rx": 0.0,
            "ry": 0.0,
            "rz": 0.2,
            "quaternion_xyzw": [0.0, 0.0, 1.0, 0.0],
        }
        state["left"]["tcp_ref_base"] = {"x": 0.29, "y": 0.19, "z": 0.49, "rx": 0.0, "ry": 0.0, "rz": 0.3}
        state["left"]["has_valid_tcp_pose"] = True
        state["left"]["tcp_actual_valid"] = True
        state["left"]["tcp_ref_valid"] = True
        state["left"]["tcp_tracking_source"] = "tcp_ref_stand"
        state["left"]["tcp_tracking_source_recommendation"] = "tcp_ref_stand"
        state["left"]["controller_simulation_mode"] = {
            "recommended_tracking_pose": "tcp_ref_stand",
            "physical_motion_expected": False,
        }
        state["left"]["physical_motion_expected"] = False
        state["left"]["controller_simulation_diagnostic_override_active"] = True
        state["left"]["tcp_deferred"] = False
        store, _, _ = self.make_safety(state)
        latest = store.latest()
        self.assertIsNotNone(latest)
        self.assertEqual(latest.left.tcp_actual_stand.as_tuple(), (0.32, 0.13, 0.45, 0.0, 0.0, 0.0))
        self.assertEqual(latest.left.tcp_actual_base.as_tuple(), (0.22, 0.11, 0.41, 0.0, 0.0, 0.1))
        self.assertEqual(latest.left.tcp_ref_stand.as_tuple(), (0.41, 0.21, 0.51, 0.0, 0.0, 0.2))
        self.assertEqual(latest.left.tcp_ref_base.as_tuple(), (0.29, 0.19, 0.49, 0.0, 0.0, 0.3))
        self.assertTrue(latest.left.tcp_actual_valid)
        self.assertTrue(latest.left.tcp_ref_valid)
        self.assertFalse(latest.left.physical_motion_expected)
        self.assertTrue(latest.left.controller_simulation_diagnostic_override_active)
        self.assertEqual(latest.left.tcp_tracking_source_recommendation, "tcp_ref_stand")
        self.assertEqual(latest.left.selected_tcp_source("auto"), "tcp_ref_stand")
        self.assertEqual(latest.left.selected_tcp_pose("auto").as_tuple(), (0.41, 0.21, 0.51, 0.0, 0.0, 0.2))
        self.assertEqual(latest.left.selected_tcp_source("actual"), "tcp_actual_stand")
        self.assertEqual(latest.left.selected_tcp_pose("actual").as_tuple(), (0.32, 0.13, 0.45, 0.0, 0.0, 0.0))
        self.assertEqual(latest.left.selected_tcp_source("both"), "tcp_ref_stand")

    def test_auto_tcp_selection_prefers_reference_when_tracking_source_is_tcp_ref_stand(self):
        state = self.tcp_available_state()
        state["left"]["tcp_ref_stand"] = {"x": 0.41, "y": 0.21, "z": 0.51, "rx": 0.0, "ry": 0.0, "rz": 0.0}
        state["left"]["tcp_ref_valid"] = True
        state["left"]["tcp_tracking_source"] = "tcp_ref_stand"
        state["left"]["tcp_tracking_source_recommendation"] = None
        store, _, _ = self.make_safety(state)
        latest = store.latest()
        self.assertEqual(latest.left.selected_tcp_source("auto"), "tcp_ref_stand")
        self.assertEqual(latest.left.selected_tcp_pose("auto").as_tuple(), (0.41, 0.21, 0.51, 0.0, 0.0, 0.0))

    def test_auto_tcp_selection_prefers_reference_for_controller_simulation_recommendation(self):
        state = self.tcp_available_state()
        state["left"]["tcp_actual_stand"] = {
            "x": 0.31,
            "y": 0.12,
            "z": 0.44,
            "rx": 0.0,
            "ry": 0.0,
            "rz": 0.0,
        }
        state["left"]["tcp_ref_stand"] = {"x": 0.42, "y": 0.22, "z": 0.52, "rx": 0.0, "ry": 0.0, "rz": 0.0}
        state["left"]["tcp_ref_valid"] = True
        state["left"]["tcp_tracking_source"] = None
        state["left"]["tcp_tracking_source_recommendation"] = "reference_for_controller_simulation"
        state["left"]["physical_motion_expected"] = False
        state["left"]["cartesian_gate"] = {
            "run_mode": "real",
            "backend_type": "rbpodo",
            "operation_mode": "simulation",
            "allow_in_controller_simulation": True,
            "allow_in_real": False,
            "physical_motion_expected": False,
            "controller_simulation_streaming_cartesian_available": True,
        }
        store, _, _ = self.make_safety(state)
        latest = store.latest()
        self.assertEqual(latest.left.selected_tcp_source("auto"), "tcp_ref_stand")
        self.assertEqual(latest.left.selected_tcp_pose("auto").as_tuple(), (0.42, 0.22, 0.52, 0.0, 0.0, 0.0))
        self.assertFalse(latest.left.physical_motion_expected)

    def test_physical_motion_expected_null_falls_back_to_cartesian_gate(self):
        # Real-motion servers publish per-arm physical_motion_expected=null with the
        # authoritative boolean in cartesian_gate; the ghost overlay must hide then.
        state = self.tcp_available_state(observed_mode="real", observed_backend="rbpodo")
        for arm in ("left", "right"):
            state[arm]["physical_motion_expected"] = None
            state[arm]["cartesian_gate"] = {
                "run_mode": "real",
                "backend_type": "rbpodo",
                "operation_mode": "real",
                "physical_motion_expected": True,
            }
        store, _, _ = self.make_safety(state)
        latest = store.latest()
        self.assertTrue(latest.left.physical_motion_expected)
        self.assertTrue(latest.right.physical_motion_expected)
        self.assertFalse(_reference_ghost_active(latest.left))
        self.assertFalse(_reference_ghost_active(latest.right))

    def test_reference_ghost_stays_active_for_controller_simulation(self):
        store, _, _ = self.make_safety(self.pgmode_spacemouse_state())
        latest = store.latest()
        self.assertFalse(latest.left.physical_motion_expected)
        self.assertTrue(_reference_ghost_active(latest.left))

    def test_pgmode_status_reports_reference_selection_and_policy_lease(self):
        store, _, _ = self.make_safety(self.pgmode_spacemouse_state())
        latest = store.latest()
        self.assertEqual(latest.left.selected_tcp_source("auto"), "tcp_ref_stand")
        self.assertEqual(latest.right.selected_tcp_source("auto"), "tcp_ref_stand")
        self.assertEqual(latest.left.cartesian_gate["operation_mode"], "simulation")
        self.assertTrue(latest.left.controller_simulation_cartesian_available)
        self.assertFalse(latest.left.controller_simulation_physical_motion_detected)
        self.assertTrue(latest.command_source.active)
        self.assertEqual(latest.command_source.display_source_id, "policy_runner")

        status = _format_pgmode_status(latest, stale=False, display_mode="auto")
        self.assertIn("pgmode_sim:", status)
        self.assertIn("backend=rbpodo", status)
        self.assertIn("run_mode=real", status)
        self.assertIn("operation_mode=simulation", status)
        self.assertIn("physical_motion_expected=false", status)
        self.assertIn("cartesian_available=true", status)
        self.assertIn("policy_runner_lease=active", status)
        self.assertIn("source=policy_runner", status)
        self.assertIn("command=TcpPoseTarget", status)
        self.assertIn("selected_tcp=tcp_ref_stand", status)
        self.assertNotIn("degraded", status)
        self.assertNotIn("warning=", status)

    def test_pgmode_status_warns_on_true_physical_motion_expected(self):
        state = self.pgmode_spacemouse_state()
        for arm in ("left", "right"):
            state[arm]["physical_motion_expected"] = True
            state[arm]["cartesian_gate"]["physical_motion_expected"] = True
        store, _, _ = self.make_safety(state)

        status = _format_pgmode_status(store.latest(), stale=False, display_mode="auto")
        self.assertIn("physical_motion_expected=true", status)
        self.assertIn("warning=physical_motion_expected_not_false", status)

    def test_pgmode_status_degrades_and_warns_when_physical_motion_expected_missing(self):
        state = self.pgmode_spacemouse_state()
        for arm in ("left", "right"):
            state[arm].pop("physical_motion_expected")
            state[arm]["cartesian_gate"].pop("physical_motion_expected")
        store, _, _ = self.make_safety(state)

        status = _format_pgmode_status(store.latest(), stale=False, display_mode="auto")
        self.assertIn("physical_motion_expected=missing", status)
        self.assertIn("warning=physical_motion_expected_not_false", status)
        self.assertIn("degraded missing=physical_motion_expected", status)

    def test_auto_tcp_selection_falls_back_to_actual_when_reference_is_invalid(self):
        state = self.tcp_available_state()
        state["left"]["tcp_ref_stand"] = {"x": 0.41, "y": 0.21, "z": 0.51, "rx": 0.0, "ry": 0.0, "rz": 0.0}
        state["left"]["tcp_ref_valid"] = False
        state["left"]["tcp_tracking_source"] = "tcp_ref_stand"
        state["left"]["tcp_tracking_source_recommendation"] = "reference_for_controller_simulation"
        store, _, _ = self.make_safety(state)
        latest = store.latest()
        self.assertEqual(latest.left.selected_tcp_source("auto"), "tcp_actual_stand")
        self.assertEqual(latest.left.selected_tcp_pose("auto").as_tuple(), (0.31, 0.12, 0.44, 0.0, 0.0, 0.0))

    def test_circle_overlay_snapshot_parser_accepts_schema_and_rejects_wrong_schema(self):
        overlay = CircleOverlaySnapshot.parse(sample_circle_overlay(), received_monotonic=100.0)
        self.assertIsNotNone(overlay)
        self.assertEqual(overlay.schema_version, CIRCLE_OVERLAY_SCHEMA_VERSION)
        self.assertEqual(overlay.arm, "left")
        self.assertEqual(overlay.tracking_source, "tcp_ref_stand")
        self.assertEqual(overlay.center_stand, (0.1, 0.2, 0.3))
        self.assertEqual(overlay.desired_pose_stand.as_tuple(), (0.175, 0.2, 0.3, 0.0, 0.0, 0.0))
        self.assertFalse(overlay.physical_motion_expected)
        self.assertFalse(overlay.stale(now=100.5, threshold_sec=1.0))
        self.assertTrue(overlay.stale(now=101.5, threshold_sec=1.0))

        invalid = sample_circle_overlay(schema_version="wrong.schema")
        self.assertIsNone(CircleOverlaySnapshot.parse(invalid))

    @unittest.skipIf(not _local_udp_socket_available(), "local UDP sockets unavailable")
    def test_circle_overlay_store_and_receiver_accept_udp_packet(self):
        store = CircleOverlayStore(stale_after_sec=0.2)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(("127.0.0.1", 0))
            port = probe.getsockname()[1]
        receiver = CircleOverlayReceiver(store, host="127.0.0.1", port=port)
        receiver.start()
        try:
            deadline = time.monotonic() + 1.0
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                payload = json.dumps(sample_circle_overlay()).encode()
                while time.monotonic() < deadline and store.latest() is None:
                    sender.sendto(payload, ("127.0.0.1", port))
                    time.sleep(0.01)
            latest = store.latest()
            self.assertIsNotNone(latest)
            self.assertEqual(latest.run_id, "run-1")
            self.assertGreaterEqual(store.received_packets, 1)
            self.assertEqual(store.invalid_packets, 0)
        finally:
            receiver.stop()

    def test_circle_overlay_store_rejects_invalid_json_and_detects_stale_packets(self):
        store = CircleOverlayStore(stale_after_sec=0.5)
        self.assertFalse(store.update_from_json_bytes(b"{bad-json"))
        self.assertEqual(store.invalid_packets, 1)
        self.assertTrue(store.update_from_json_bytes(json.dumps(sample_circle_overlay()).encode(), received_monotonic=100.0))
        self.assertFalse(store.is_stale(now=100.25))
        self.assertTrue(store.is_stale(now=100.75))

    def test_circle_overlay_bind_cli_env_and_none(self):
        self.assertEqual(parse_udp_bind("udp://127.0.0.1:50261"), ("127.0.0.1", 50261))
        self.assertIsNone(parse_udp_bind("none"))
        self.assertTrue(parse_args(["--check-assets"]).check_assets)
        self.assertEqual(
            _circle_overlay_bind_from_args_env(parse_args(["--circle-overlay-bind", "udp://127.0.0.1:50261"])),
            ("127.0.0.1", 50261),
        )
        old_value = os.environ.get("RB_GUI_CIRCLE_OVERLAY_BIND")
        try:
            os.environ["RB_GUI_CIRCLE_OVERLAY_BIND"] = "udp://0.0.0.0:50262"
            self.assertEqual(_circle_overlay_bind_from_args_env(parse_args([])), ("0.0.0.0", 50262))
            os.environ["RB_GUI_CIRCLE_OVERLAY_BIND"] = "none"
            self.assertIsNone(_circle_overlay_bind_from_args_env(parse_args([])))
        finally:
            if old_value is None:
                os.environ.pop("RB_GUI_CIRCLE_OVERLAY_BIND", None)
            else:
                os.environ["RB_GUI_CIRCLE_OVERLAY_BIND"] = old_value

    def test_finger_position_maps_gripper_percent_to_travel(self):
        self.assertAlmostEqual(_finger_position_m(100.0), 0.0)        # open
        self.assertAlmostEqual(_finger_position_m(0.0), 0.047)        # closed (full travel)
        self.assertAlmostEqual(_finger_position_m(50.0), 0.0235)      # half
        self.assertAlmostEqual(_finger_position_m(None), 0.0)         # unknown -> open
        self.assertAlmostEqual(_finger_position_m(150.0), 0.0)        # clamp high -> open
        self.assertAlmostEqual(_finger_position_m(-10.0), 0.047)      # clamp low -> closed

    def test_update_urdf_config_drives_finger_joints_only_when_present(self):
        class FakeUrdf:
            def __init__(self, names):
                self._names = names
                self.last = None
            def get_actuated_joint_names(self):
                return self._names
            def update_cfg(self, cfg):
                self.last = list(np.asarray(cfg, dtype=float))

        arm = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)  # 6 arm joint values (radians)
        # Articulated URDF (8 joints): fingers filled from gripper percent.
        art = FakeUrdf((
            "base_joint", "shoulder_joint", "elbow_joint", "wrist1_joint",
            "wrist2_joint", "wrist3_joint", "finger_left_joint", "finger_right_joint",
        ))
        _update_urdf_config(art, arm, gripper_percent=0.0)  # closed
        self.assertEqual(len(art.last), 8)
        self.assertEqual(art.last[:6], list(arm))
        self.assertAlmostEqual(art.last[6], +0.047)   # finger_left +X
        self.assertAlmostEqual(art.last[7], -0.047)   # finger_right -X
        _update_urdf_config(art, arm, gripper_percent=100.0)  # open
        self.assertAlmostEqual(art.last[6], 0.0)
        self.assertAlmostEqual(art.last[7], 0.0)
        # Plain 6-joint URDF: gripper percent ignored, arm values passed through.
        plain = FakeUrdf(("base_joint", "shoulder_joint", "elbow_joint",
                          "wrist1_joint", "wrist2_joint", "wrist3_joint"))
        _update_urdf_config(plain, arm, gripper_percent=0.0)
        self.assertEqual(plain.last, list(arm))

    def test_gripper_cmd_endpoint_default_and_env(self):
        old = os.environ.pop("RB_GUI_GRIPPER_CMD_ENDPOINT", None)
        try:
            self.assertEqual(_gripper_cmd_endpoint(), ("127.0.0.1", 50410))
            os.environ["RB_GUI_GRIPPER_CMD_ENDPOINT"] = "udp://10.0.0.5:51000"
            self.assertEqual(_gripper_cmd_endpoint(), ("10.0.0.5", 51000))
        finally:
            os.environ.pop("RB_GUI_GRIPPER_CMD_ENDPOINT", None)
            if old is not None:
                os.environ["RB_GUI_GRIPPER_CMD_ENDPOINT"] = old

    def test_recording_endpoints_default_and_env(self):
        old_cmd = os.environ.pop("RB_GUI_RECORD_CMD_ENDPOINT", None)
        old_status = os.environ.pop("RB_GUI_RECORD_STATUS_BIND", None)
        try:
            self.assertEqual(_recording_cmd_endpoint(), ("127.0.0.1", 50441))
            self.assertEqual(_recording_status_bind_endpoint(), ("0.0.0.0", 50442))
            os.environ["RB_GUI_RECORD_CMD_ENDPOINT"] = "udp://10.0.0.5:51441"
            os.environ["RB_GUI_RECORD_STATUS_BIND"] = "udp://127.0.0.1:51442"
            self.assertEqual(_recording_cmd_endpoint(), ("10.0.0.5", 51441))
            self.assertEqual(_recording_status_bind_endpoint(), ("127.0.0.1", 51442))
        finally:
            os.environ.pop("RB_GUI_RECORD_CMD_ENDPOINT", None)
            os.environ.pop("RB_GUI_RECORD_STATUS_BIND", None)
            if old_cmd is not None:
                os.environ["RB_GUI_RECORD_CMD_ENDPOINT"] = old_cmd
            if old_status is not None:
                os.environ["RB_GUI_RECORD_STATUS_BIND"] = old_status

    def test_recording_toggle_debounce_and_metadata(self):
        client = RecordingCommandFake()
        handles = {
            "recording_cmd_client": client,
            "recording_last_toggle_monotonic": float("-inf"),
            "recording_task": RecordingText("fold towel"),
            "recording_operator": RecordingText("operator-a"),
            "recording_state": RecordingText("idle"),
            "recording_episode": RecordingText(""),
            "recording_frames": RecordingText(""),
            "recording_command_status": RecordingText(""),
            "recording_start_button": RecordingButton(),
            "recording_stop_button": RecordingButton(),
        }

        self.assertTrue(_toggle_episode_recording(handles, monotonic_fn=lambda: 10.0))
        self.assertEqual(client.calls, [{"command": "start", "task": "fold towel", "operator": "operator-a"}])
        self.assertTrue(handles["recording_start_button"].disabled)
        self.assertFalse(handles["recording_stop_button"].disabled)

        self.assertFalse(_toggle_episode_recording(handles, monotonic_fn=lambda: 10.5))
        self.assertEqual(len(client.calls), 1)
        self.assertIn("debounce", handles["recording_command_status"].value)

        self.assertTrue(_toggle_episode_recording(handles, target="stop", monotonic_fn=lambda: 11.1))
        self.assertEqual(client.calls[-1]["command"], "stop")
        self.assertFalse(handles["recording_start_button"].disabled)
        self.assertTrue(handles["recording_stop_button"].disabled)

    def test_recording_status_panel_prefers_state_block_and_store_fallback(self):
        from rb_servo_gui.models import StateSnapshot

        store = RecordingStatusStore()
        store.update(
            {
                "recording": True,
                "episode_name": "episode-store",
                "frame_count": 12,
                "rate_hz": 30.0,
            },
            received_monotonic=1.0,
        )
        handles = {
            "recording_status_store": store,
            "recording_state": RecordingText(""),
            "recording_episode": RecordingText(""),
            "recording_frames": RecordingText(""),
            "recording_start_button": RecordingButton(),
            "recording_stop_button": RecordingButton(),
        }

        _update_recording_panel(handles, None, stale=False)
        self.assertEqual(handles["recording_state"].value, "recording")
        self.assertEqual(handles["recording_episode"].value, "episode-store")
        self.assertEqual(handles["recording_frames"].value, "12 @ 30.0 Hz")
        self.assertTrue(handles["recording_start_button"].disabled)
        self.assertFalse(handles["recording_stop_button"].disabled)

        latest = StateSnapshot.parse(
            sample_state(
                recording={
                    "recording": False,
                    "state": "idle",
                    "episode_name": "episode-state",
                    "frame_count": 99,
                    "rate_hz": 30.0,
                }
            )
        )
        self.assertIsNotNone(latest)
        self.assertEqual(latest.recording["episode_name"], "episode-state")
        _update_recording_panel(handles, latest, stale=True)
        self.assertEqual(handles["recording_state"].value, "idle (stale)")
        self.assertEqual(handles["recording_episode"].value, "episode-state")
        self.assertFalse(handles["recording_start_button"].disabled)
        self.assertTrue(handles["recording_stop_button"].disabled)

    def test_recording_status_store_parses_wire_packet(self):
        store = RecordingStatusStore()
        ok = store.update_from_packet(
            json.dumps(
                {
                    "schema": "robotics_lab.recording_state.v1",
                    "recording": {"recording": True, "episode_name": "episode-1", "frame_count": 3},
                }
            ).encode("utf-8"),
            received_monotonic=5.0,
        )
        self.assertTrue(ok)
        self.assertEqual(store.latest()["episode_name"], "episode-1")
        self.assertFalse(store.is_stale(now=5.5))
        self.assertTrue(store.is_stale(now=7.0))
        self.assertFalse(store.update_from_packet(b"not json"))
        self.assertEqual(store.invalid_packets, 1)

    def test_arm_init_command_client_emits_control_schema(self):
        class _Sock:
            def __init__(self, *_args):
                self.sent = []

            def sendto(self, data, endpoint):
                self.sent.append((json.loads(data.decode()), endpoint))
                return len(data)

            def close(self):
                pass

        sock = _Sock()
        client = RecordingCommandClient(
            "127.0.0.1",
            50441,
            socket_factory=lambda *_args: sock,
        )
        result = client.send_arm_init(
            "left",
            left_q_deg=(1, 2, 3, 4, 5, 6),
            right_q_deg=(-1, -2, -3, -4, -5, -6),
        )
        self.assertTrue(result.ok, result.message)
        payload, endpoint = sock.sent[0]
        self.assertEqual(endpoint, ("127.0.0.1", 50441))
        self.assertEqual(payload["schema"], ARM_INIT_COMMAND_SCHEMA)
        self.assertEqual(payload["arms"], "left")
        self.assertEqual(payload["action"], "toggle")
        self.assertEqual(payload["left_q_deg"], [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])

    def test_recording_status_store_keeps_arm_init_status_block(self):
        store = RecordingStatusStore()
        ok = store.update_from_packet(
            json.dumps(
                {
                    "schema": "robotics_lab.recording_state.v1",
                    "recording": {"recording": False},
                    "arm_init": {
                        "init_override_left": True,
                        "init_override_right": False,
                        "last_command": "left",
                    },
                }
            ).encode("utf-8"),
            received_monotonic=5.0,
        )
        self.assertTrue(ok)
        self.assertTrue(store.latest_arm_init()["init_override_left"])
        self.assertFalse(store.latest_arm_init()["init_override_right"])

    def test_arm_init_override_uses_policy_control_when_recording_active(self):
        # policy_runner ACTIVELY controlling (recording in progress) -> route the
        # InitMotion press to its arm_init latch so it coordinates with the rollout.
        _, client, safety = self.make_safety(
            sample_state(motion_state="Running"),
            init_left_joint_deg=_DEFAULT_INIT_LEFT_JOINTS_DEG,
            init_right_joint_deg=_DEFAULT_INIT_RIGHT_JOINTS_DEG,
        )
        status_store = RecordingStatusStore()
        status_store.update({"recording": True}, received_monotonic=time.monotonic())
        command_client = RecordingCommandFake()
        handles = {
            "recording_status_store": status_store,
            "recording_cmd_client": command_client,
            "arm_init_status": RecordingText(""),
            "_state_stale": False,
        }
        scene = {"left_tcp_target_user_moved": True, "right_tcp_target_user_moved": True}

        ok, message = _send_arm_init_override(safety, scene, handles, "left")

        self.assertTrue(ok, message)
        self.assertEqual(command_client.arm_init_calls[0]["arms"], "left")
        self.assertEqual(command_client.arm_init_calls[0]["left_q_deg"], _DEFAULT_INIT_LEFT_JOINTS_DEG)
        self.assertEqual(client.sent_packets, [])
        self.assertNotIn("left_tcp_target_user_moved", scene)
        self.assertIn("right_tcp_target_user_moved", scene)
        self.assertTrue(handles["arm_init_local_status"]["init_override_left"])

    def test_arm_init_override_routes_direct_when_policy_runner_idle(self):
        # An idle-but-alive policy_runner (status fresh, NOT recording, no arm_init
        # latch, not owning the lease) must NOT capture the InitMotion latch: a
        # standalone GUI InitMotion goes through the direct one-shot path so the
        # command lease is released afterward and later GUI Cartesian commands are
        # not rejected with command_source_lease_conflict.
        _, client, safety = self.make_safety(
            sample_state(motion_state="ArmedHold"),
            init_left_joint_deg=_DEFAULT_INIT_LEFT_JOINTS_DEG,
            init_right_joint_deg=_DEFAULT_INIT_RIGHT_JOINTS_DEG,
        )
        status_store = RecordingStatusStore()
        status_store.update({"recording": False}, received_monotonic=time.monotonic())
        command_client = RecordingCommandFake()
        handles = {
            "recording_status_store": status_store,
            "recording_cmd_client": command_client,
            "arm_init_status": RecordingText(""),
            "_state_stale": False,
        }
        scene = {"left_tcp_target_user_moved": True, "right_tcp_target_user_moved": True}

        ok, message = _send_arm_init_override(safety, scene, handles, "both")

        self.assertTrue(ok, message)
        # Direct GUI one-shot InitMotion, NOT the policy_runner arm_init latch.
        self.assertEqual(command_client.arm_init_calls, [])
        self.assertEqual(client.sent_packets[-1]["mode"], "JointTarget")

    def test_arm_init_fallback_one_arm_works_without_held_lease(self):
        # Per-arm InitMotion fallback (policy_runner not actively controlling) sends
        # WITHOUT an explicitly held lease: command_client.send auto-brackets the
        # Hold-top packet with AcquireLease/ReleaseLease, same as the dual path.
        _, client, safety = self.make_safety(
            sample_state(motion_state="ArmedHold"),
            init_left_joint_deg=_DEFAULT_INIT_LEFT_JOINTS_DEG,
            init_right_joint_deg=_DEFAULT_INIT_RIGHT_JOINTS_DEG,
        )
        handles = {"arm_init_status": RecordingText(""), "_state_stale": False}
        scene = {"left_tcp_target_user_moved": True, "right_tcp_target_user_moved": True}

        ok, message = _send_arm_init_override(safety, scene, handles, "both")
        self.assertTrue(ok, message)
        self.assertEqual(client.sent_packets[-1]["mode"], "JointTarget")
        self.assertNotIn("left_tcp_target_user_moved", scene)
        self.assertNotIn("right_tcp_target_user_moved", scene)

        # One-arm fallback now succeeds without hold_lease ("Take control") — no
        # longer a silent no-op.
        ok, message = _send_arm_init_override(safety, {}, handles, "left")
        self.assertTrue(ok, message)
        packet = client.sent_packets[-1]
        self.assertEqual(packet["mode"], "Hold")
        self.assertEqual(packet["left"]["mode"], "JointTarget")
        self.assertEqual(packet["left"]["joint_target_profile"], "init_motion")
        self.assertEqual(packet["right"]["mode"], "Hold")

        # ...and still works when the lease is explicitly held.
        client.hold_lease = True
        ok, message = _send_arm_init_override(safety, {}, handles, "right")
        self.assertTrue(ok, message)
        packet = client.sent_packets[-1]
        self.assertEqual(packet["mode"], "Hold")
        self.assertEqual(packet["right"]["mode"], "JointTarget")
        self.assertEqual(packet["right"]["joint_target_profile"], "init_motion")
        self.assertEqual(packet["left"]["mode"], "Hold")

    def test_arm_init_status_panel_updates_text_and_button_colors(self):
        handles = {
            "arm_init_local_status": {
                "init_override_left": True,
                "init_override_right": False,
            },
            "arm_init_status": RecordingText(""),
            "init_motion_buttons": {
                "both": RecordingButton(),
                "left": RecordingButton(),
                "right": RecordingButton(),
            },
        }

        _update_arm_init_panel(handles, None, stale=False)

        self.assertIn("왼팔: InitMotion 중", handles["arm_init_status"].value)
        self.assertEqual(handles["init_motion_buttons"]["both"].color, "gray")
        self.assertEqual(handles["init_motion_buttons"]["left"].color, "green")
        self.assertEqual(handles["init_motion_buttons"]["right"].color, "gray")

    def test_lifecycle_init_motion_layout_targets_three_buttons(self):
        html = _lifecycle_init_motion_layout_html()
        self.assertIn("InitMotion (양팔)", html)
        self.assertIn("InitMotion (왼팔)", html)
        self.assertIn("InitMotion (오른팔)", html)
        self.assertIn("calc(50% - 0.25rem)", html)

    def test_viser_keyboard_patch_includes_b_hotkey_and_input_guard(self):
        script = _viser_keyboard_patch_script()
        self.assertIn("KeyB", script)
        self.assertIn("record-toggle-hotkey-b", script)
        self.assertIn("수집 시작", script)
        self.assertIn("수집 종료", script)
        self.assertIn("INPUT", script)
        self.assertIn("TEXTAREA", script)

    def test_send_gripper_command_builds_and_sends_packet(self):
        class _Slider:
            def __init__(self, v):
                self.value = v

        class _Sock:
            def __init__(self):
                self.sent = []
            def sendto(self, data, endpoint):
                self.sent.append((json.loads(data.decode()), endpoint))

        sock = _Sock()
        handles = {
            "gripper_slider_left": _Slider(30.0),
            "gripper_slider_right": _Slider(80.0),
            # Already synced to hardware once (different value) -> the slider's
            # current value is a genuine operator move and commands.
            "gripper_synced_value_left": 0.0,
            "gripper_synced_value_right": 0.0,
            "gripper_cmd_sock": sock,
            "gripper_cmd_endpoint": ("127.0.0.1", 50410),
            "gripper_cmd_seq": 0,
        }
        _send_gripper_command(handles)
        self.assertEqual(len(sock.sent), 1)
        msg, endpoint = sock.sent[0]
        self.assertEqual(endpoint, ("127.0.0.1", 50410))
        self.assertEqual(msg["schema"], "robotics_lab.gripper_cmd.v1")
        self.assertTrue(msg["deadman"])
        self.assertEqual(msg["left"], {"percent": 30.0, "valid": True})
        self.assertEqual(msg["right"], {"percent": 80.0, "valid": True})
        self.assertEqual(handles["gripper_cmd_seq"], 1)  # seq advances

    def test_send_gripper_command_holds_until_synced(self):
        # Startup: no gripper feedback synced yet (gripper_synced_value_* absent),
        # sliders at their initial 100 (=open). A client-connect echo of that
        # initial value must NOT command the gripper open — hold the power-on
        # position until the slider has tracked real hardware at least once.
        class _Slider:
            def __init__(self, v):
                self.value = v

        class _Sock:
            def __init__(self):
                self.sent = []
            def sendto(self, data, endpoint):
                self.sent.append((data, endpoint))

        sock = _Sock()
        handles = {
            "gripper_slider_left": _Slider(100.0),
            "gripper_slider_right": _Slider(100.0),
            "gripper_cmd_sock": sock,
            "gripper_cmd_endpoint": ("127.0.0.1", 50410),
            "gripper_cmd_seq": 0,
        }
        _send_gripper_command(handles)
        self.assertEqual(sock.sent, [])  # pre-sync: no startup-open command

    def test_arm_snapshot_parses_gripper_feedback_block(self):
        from rb_servo_gui.models import StateSnapshot
        state = sample_state()
        state["left"]["gripper"] = {"valid": True, "percent": 42.5, "moving": True}
        state["right"]["gripper"] = {"valid": False, "stale": True}
        snap = StateSnapshot.parse(state)
        self.assertAlmostEqual(snap.left.gripper_percent, 42.5)
        self.assertTrue(snap.left.gripper_moving)
        self.assertIsNone(snap.right.gripper_percent)  # invalid feed -> None

    def test_push_gripper_prefers_published_then_slider(self):
        from rb_servo_gui.models import StateSnapshot
        class _Slider:
            def __init__(self, v):
                self.value = v

        scene: dict = {}
        handles = {"scene": scene, "gripper_slider_left": _Slider(100.0), "gripper_slider_right": _Slider(100.0)}
        state = sample_state()
        state["left"]["gripper"] = {"valid": True, "percent": 30.0}  # published
        # right: no valid gripper feed -> slider fallback
        _push_gripper_percent(handles, StateSnapshot.parse(state))
        self.assertAlmostEqual(scene["gripper_percent_left"], 30.0)
        self.assertAlmostEqual(scene["gripper_percent_right"], 100.0)
        # no state at all -> slider drives
        scene.clear()
        _push_gripper_percent(handles, None)
        self.assertAlmostEqual(scene["gripper_percent_left"], 100.0)

    def test_gripper_feedback_drives_slider_value(self):
        from rb_servo_gui.models import StateSnapshot
        class _Slider:
            def __init__(self, v):
                self.value = v

        handles = {"gripper_slider_left": _Slider(100.0), "gripper_slider_right": _Slider(100.0)}
        state = sample_state()
        state["left"]["gripper"] = {"valid": True, "percent": 30.0, "moving": True}
        state["right"]["gripper"] = {"valid": False}
        _update_gripper_feedback(handles, StateSnapshot.parse(state), stale=False)
        # Valid feed -> slider tracks actual %, and the synced value is recorded.
        self.assertAlmostEqual(handles["gripper_slider_left"].value, 30.0)
        self.assertAlmostEqual(handles["gripper_synced_value_left"], 30.0)
        # No valid feed -> slider left as the operator's setpoint (unchanged).
        self.assertAlmostEqual(handles["gripper_slider_right"].value, 100.0)
        self.assertNotIn("gripper_synced_value_right", handles)
        # Stale feed is ignored (don't chase a frozen reading).
        handles["gripper_slider_left"].value = 55.0
        _update_gripper_feedback(handles, StateSnapshot.parse(state), stale=True)
        self.assertAlmostEqual(handles["gripper_slider_left"].value, 55.0)

    def test_gripper_feedback_echo_does_not_command(self):
        from rb_servo_gui.models import StateSnapshot
        class _Slider:
            def __init__(self, v):
                self.value = v
        class _Sock:
            def __init__(self):
                self.sent = []
            def sendto(self, payload, endpoint):
                self.sent.append((payload, endpoint))

        sock = _Sock()
        handles = {
            "gripper_slider_left": _Slider(100.0),
            "gripper_slider_right": _Slider(100.0),
            "gripper_cmd_sock": sock,
            "gripper_cmd_endpoint": ("127.0.0.1", 50410),
            "gripper_manual_hold_sec": 1.0,
        }
        state = sample_state()
        state["left"]["gripper"] = {"valid": True, "percent": 30.0}
        state["right"]["gripper"] = {"valid": True, "percent": 70.0}
        snap = StateSnapshot.parse(state)
        # Feedback writes the sliders; the resulting on_update echo must NOT command.
        _update_gripper_feedback(handles, snap, stale=False)
        _send_gripper_command(handles)
        self.assertEqual(sock.sent, [])
        # A genuine operator move (beyond the eps) DOES command, and opens a hold.
        handles["gripper_slider_left"].value = 10.0
        _send_gripper_command(handles)
        self.assertEqual(len(sock.sent), 1)
        payload = json.loads(sock.sent[0][0].decode("utf-8"))
        self.assertAlmostEqual(payload["left"]["percent"], 10.0)
        self.assertIn("gripper_manual_hold_until_left", handles)
        # While the hold is open, feedback does not yank the operator's value back.
        _update_gripper_feedback(handles, snap, stale=False)
        self.assertAlmostEqual(handles["gripper_slider_left"].value, 10.0)

    def test_robot_urdf_path_uses_descriptions_dir_env(self):
        descriptions_dir = Path(__file__).resolve().parents[2] / "rb_servo_server" / "descriptions"
        old_value = os.environ.get("RB_GUI_DESCRIPTIONS_DIR")
        try:
            os.environ["RB_GUI_DESCRIPTIONS_DIR"] = str(descriptions_dir)
            # Default prefers the GUI-only articulated-gripper URDF when present
            # (falls back to the plain rb3_730e.urdf otherwise).
            self.assertEqual(
                _robot_urdf_path(), descriptions_dir / "urdf" / "rb3_730e_pika_articulated.urdf"
            )
            self.assertTrue(_robot_urdf_path().exists())
        finally:
            if old_value is None:
                os.environ.pop("RB_GUI_DESCRIPTIONS_DIR", None)
            else:
                os.environ["RB_GUI_DESCRIPTIONS_DIR"] = old_value

    def test_missing_robot_urdf_reports_clear_error_string(self):
        old_value = os.environ.get("RB_GUI_DESCRIPTIONS_DIR")
        try:
            with tempfile.TemporaryDirectory() as tmpdir:
                os.environ["RB_GUI_DESCRIPTIONS_DIR"] = tmpdir
                handles: dict[str, object] = {}
                _add_robot_urdfs(object(), handles)
                self.assertIn("urdf_error", handles)
                self.assertIn("robot URDF not found", str(handles["urdf_error"]))
                self.assertIn("Install with python3 -m pip install -e rb_gui", str(handles["urdf_error"]))
        finally:
            if old_value is None:
                os.environ.pop("RB_GUI_DESCRIPTIONS_DIR", None)
            else:
                os.environ["RB_GUI_DESCRIPTIONS_DIR"] = old_value

    def test_scene_asset_status_formats_error_keys_and_hint(self):
        status = _format_scene_asset_status(
            {
                "urdf_error": "robot URDF not found: /tmp/missing.urdf",
                "stand_mesh_error": "stand mesh not found: /tmp/missing.stl",
                "urdf_update_error": "RuntimeError: bad joint config",
            }
        )
        self.assertIn("urdf_error=robot URDF not found", status)
        self.assertIn("stand_mesh_error=stand mesh not found", status)
        self.assertIn("urdf_update_error=RuntimeError: bad joint config", status)
        self.assertIn("Install with python3 -m pip install -e rb_gui", status)

    def test_parser_accepts_legacy_tcp_pose_without_quaternion(self):
        pose = Pose6D.parse({"x": 0.31, "y": 0.12, "z": 0.44, "rx": 0.0, "ry": 0.0, "rz": 0.0})
        self.assertIsNotNone(pose)
        self.assertIsNone(pose.quaternion_xyzw)

    def test_parser_keeps_arm_snapshot_when_tcp_pose_is_null(self):
        store, _, _ = self.make_safety(sample_state())
        latest = store.latest()
        self.assertIsNotNone(latest)
        self.assertEqual(latest.left.connection_state, "Connected")
        self.assertIsNone(latest.left.tcp_stand)
        self.assertFalse(latest.left.has_valid_tcp_pose)
        self.assertTrue(latest.left.tcp_deferred)

    def test_parser_preserves_status_when_tcp_pose_is_invalid(self):
        state = sample_state(fault_latched=True, fault_reason="tcp invalid test")
        state["left"]["tcp_stand"] = {"x": 0.31, "y": float("nan"), "z": 0.44, "rx": 0.0, "ry": 0.0, "rz": 0.0}
        state["left"]["has_valid_tcp_pose"] = False
        state["left"]["tcp_deferred"] = False
        store, _, _ = self.make_safety(state)
        latest = store.latest()
        self.assertIsNotNone(latest)
        self.assertIsNone(latest.left.tcp_stand)
        self.assertFalse(latest.left.has_valid_tcp_pose)
        self.assertFalse(latest.left.tcp_deferred)
        self.assertEqual(latest.fault_reason, "tcp invalid test")

    def test_rejected_motion_state_remains_operator_visible_hold(self):
        state = sample_state(motion_state="ArmedHold", safety_verdict="CartesianUnavailable")
        store, _, safety = self.make_safety(state)
        latest = store.latest()
        self.assertEqual(latest.motion_state, "ArmedHold")
        self.assertEqual(latest.safety_verdict, "CartesianUnavailable")
        self.assertNotEqual(latest.motion_state, "Running")
        readiness = safety.readiness()
        self.assertTrue(readiness.ready)
        self.assertFalse(readiness.fault)

    def test_stale_or_missing_state_blocks_joint_jog_without_zero_fallback(self):
        _, client, safety = self.make_safety(None)
        ok, reason = safety.jog_joint("left", 0, 1.0)
        self.assertFalse(ok)
        self.assertIn("state stream", reason)
        self.assertEqual(client.sent_packets, [])

    def test_joint_jog_uses_latest_sent_state_and_clamps_step(self):
        _, client, safety = self.make_safety(sample_state())
        ok, reason = safety.jog_joint("left", 0, 99.0)
        self.assertTrue(ok, reason)
        packet = client.sent_packets[-1]
        self.assertEqual(packet["mode"], "JointTarget")
        self.assertEqual(packet["source_id"], "rb_gui")
        self.assertTrue(packet["session_id"])
        self.assertEqual(packet["left"]["q_target_deg"], [2.0, -30.0, 80.0, 0.0, 60.0, 0.0])
        self.assertEqual(packet["right"]["q_target_deg"], [0.0, -30.0, 80.0, 0.0, 60.0, 0.0])
        self.assertGreater(packet["timeout_sec"], 0.0)

    def test_env_joint6_parser_accepts_csv_and_rejects_invalid_values(self):
        old_value = os.environ.get("RB_GUI_INIT_LEFT_JOINTS")
        try:
            os.environ["RB_GUI_INIT_LEFT_JOINTS"] = "1, 2, 3, 4, 5, 6"
            self.assertEqual(_env_joint6("RB_GUI_INIT_LEFT_JOINTS", _DEFAULT_INIT_LEFT_JOINTS_DEG), (1.0, 2.0, 3.0, 4.0, 5.0, 6.0))
            os.environ["RB_GUI_INIT_LEFT_JOINTS"] = "1, 2, 3"
            self.assertIsNone(_env_joint6("RB_GUI_INIT_LEFT_JOINTS", _DEFAULT_INIT_LEFT_JOINTS_DEG))
            os.environ["RB_GUI_INIT_LEFT_JOINTS"] = "1, 2, nan, 4, 5, 6"
            self.assertIsNone(_env_joint6("RB_GUI_INIT_LEFT_JOINTS", _DEFAULT_INIT_LEFT_JOINTS_DEG))
            os.environ.pop("RB_GUI_INIT_LEFT_JOINTS")
            self.assertEqual(_env_joint6("RB_GUI_INIT_LEFT_JOINTS", _DEFAULT_INIT_LEFT_JOINTS_DEG), _DEFAULT_INIT_LEFT_JOINTS_DEG)
        finally:
            if old_value is None:
                os.environ.pop("RB_GUI_INIT_LEFT_JOINTS", None)
            else:
                os.environ["RB_GUI_INIT_LEFT_JOINTS"] = old_value

    def test_init_motion_requires_configured_targets_and_armed_state(self):
        _, client, unconfigured = self.make_safety(sample_state(motion_state="ArmedHold"))
        ok, reason = unconfigured.send_init_motion()
        self.assertFalse(ok)
        self.assertIn("not configured", reason)
        self.assertEqual(client.sent_packets, [])

        _, client, safety = self.make_safety(
            sample_state(motion_state="ConnectedHold"),
            init_left_joint_deg=_DEFAULT_INIT_LEFT_JOINTS_DEG,
            init_right_joint_deg=_DEFAULT_INIT_RIGHT_JOINTS_DEG,
        )
        ok, reason = safety.send_init_motion()
        self.assertFalse(ok)
        self.assertIn("ArmMotion first", reason)
        self.assertEqual(client.sent_packets, [])

    def test_init_motion_sends_in_every_mode_when_state_ready(self):
        # Env/mode gating retired: the init profile sends in real AND simulation
        # mode as long as the live state is ready (armed, valid joints, no fault).
        for mode in ("real", "simulation"):
            _, client, safety = self.make_safety(
                sample_state(motion_state="ArmedHold"),
                desired=mode,
                observed=mode,
                init_left_joint_deg=_DEFAULT_INIT_LEFT_JOINTS_DEG,
                init_right_joint_deg=_DEFAULT_INIT_RIGHT_JOINTS_DEG,
            )
            ok, reason = safety.send_init_motion()
            self.assertTrue(ok, f"{mode}: {reason}")
            self.assertEqual(client.sent_packets[-1]["mode"], "JointTarget")
            self.assertEqual(client.sent_packets[-1]["left"]["joint_target_profile"], "init_motion")

    def test_init_motion_blocked_by_latched_fault(self):
        # State-derived gate: a latched fault blocks the init profile regardless of mode.
        _, client, safety = self.make_safety(
            sample_state(motion_state="ArmedHold", fault_latched=True, fault_reason="self_collision"),
            init_left_joint_deg=_DEFAULT_INIT_LEFT_JOINTS_DEG,
            init_right_joint_deg=_DEFAULT_INIT_RIGHT_JOINTS_DEG,
        )
        ok, reason = safety.send_init_motion()
        self.assertFalse(ok)
        self.assertIn("fault latched", reason)
        self.assertEqual(client.sent_packets, [])

    def test_init_motion_sends_joint_target_profile_with_long_timeout(self):
        _, client, safety = self.make_safety(
            sample_state(motion_state="ArmedHold"),
            init_left_joint_deg=_DEFAULT_INIT_LEFT_JOINTS_DEG,
            init_right_joint_deg=_DEFAULT_INIT_RIGHT_JOINTS_DEG,
            init_motion_timeout_sec=10.0,
        )
        ok, reason = safety.send_init_motion()
        self.assertTrue(ok, reason)
        packet = client.sent_packets[-1]
        # The server plans a collision-free + floor-safe path to the init pose
        # through a JointTarget profile. The q_target/timeout payload mirrors a
        # direct JointTarget.
        self.assertEqual(packet["mode"], "JointTarget")
        self.assertEqual(packet["left"]["q_target_deg"], list(_DEFAULT_INIT_LEFT_JOINTS_DEG))
        self.assertEqual(packet["right"]["q_target_deg"], list(_DEFAULT_INIT_RIGHT_JOINTS_DEG))
        self.assertEqual(packet["left"]["joint_target_profile"], "init_motion")
        self.assertEqual(packet["right"]["joint_target_profile"], "init_motion")
        self.assertEqual(packet["timeout_sec"], 10.0)
        self.assertTrue(packet["coupled_timeout"])

    def test_init_motion_success_resets_tcp_target_follow_flags(self):
        _, client, safety = self.make_safety(
            sample_state(motion_state="ArmedHold"),
            init_left_joint_deg=_DEFAULT_INIT_LEFT_JOINTS_DEG,
            init_right_joint_deg=_DEFAULT_INIT_RIGHT_JOINTS_DEG,
        )
        handles = {
            "left_tcp_target_user_moved": True,
            "right_tcp_target_user_moved": True,
            "left_tcp_target_pose": (1.0, 2.0, 3.0, 0.0, 0.0, 0.0),
        }
        ok, reason = _send_init_motion_and_reset_targets(safety, handles)
        self.assertTrue(ok, reason)
        self.assertNotIn("left_tcp_target_user_moved", handles)
        self.assertNotIn("right_tcp_target_user_moved", handles)
        self.assertEqual(handles["left_tcp_target_pose"], (1.0, 2.0, 3.0, 0.0, 0.0, 0.0))
        self.assertEqual(client.sent_packets[-1]["mode"], "JointTarget")
        self.assertIn("follow current TCP", reason)

    def test_init_motion_blocked_preserves_tcp_target_follow_flags(self):
        _, client, safety = self.make_safety(
            sample_state(motion_state="ConnectedHold"),
            init_left_joint_deg=_DEFAULT_INIT_LEFT_JOINTS_DEG,
            init_right_joint_deg=_DEFAULT_INIT_RIGHT_JOINTS_DEG,
        )
        handles = {
            "left_tcp_target_user_moved": True,
            "right_tcp_target_user_moved": True,
        }
        ok, reason = _send_init_motion_and_reset_targets(safety, handles)
        self.assertFalse(ok)
        self.assertIn("ArmMotion first", reason)
        self.assertIn("left_tcp_target_user_moved", handles)
        self.assertIn("right_tcp_target_user_moved", handles)
        self.assertEqual(client.sent_packets, [])

    def test_real_mode_blocks_lifecycle_and_motion(self):
        # Real/sim gating retired: lifecycle and jog work in real mode.
        _, client, safety = self.make_safety(sample_state(), desired="real", observed="real")
        ok, reason = safety.send_lifecycle("ArmMotion")
        self.assertTrue(ok, reason)
        self.assertEqual(client.sent_packets[-1]["mode"], "ArmMotion")

    def test_simulation_mode_no_longer_gated_by_env_readiness(self):
        # Env readiness retired: a fresh, joint-valid, fault-free state means the
        # server is the authority, so simulation jog sends without an env unlock.
        _, client, safety = self.make_safety(sample_state(), desired="simulation", observed="simulation")
        ok, reason = safety.jog_joint("left", 0, 1.0)
        self.assertTrue(ok, reason)
        self.assertEqual(client.sent_packets[-1]["mode"], "JointTarget")

    def test_deprecated_rbsim_mode_normalizes_to_simulation_backend(self):
        observed = normalize_observed_mode_backend("rbsim_local", None)
        self.assertEqual(observed.mode, "simulation")
        self.assertEqual(observed.backend, "simulator")
        self.assertIn("deprecated observed mode", observed.warnings[0])
        unknown_backend = normalize_observed_mode_backend("simulation", "future_backend")
        self.assertEqual(unknown_backend.mode, "simulation")
        self.assertEqual(unknown_backend.backend, "unknown")

        _, client, safety = self.make_safety(sample_state(), desired="simulation", observed="rbsim_local", sim_ready=True)
        self.assertEqual(safety.observed_server_mode, "simulation")
        self.assertEqual(safety.observed_backend, "simulator")
        ok, reason = safety.jog_joint("left", 0, 1.0)
        self.assertTrue(ok, reason)
        self.assertEqual(client.sent_packets[-1]["mode"], "JointTarget")

    def test_desired_mode_does_not_claim_server_hot_switch(self):
        # Mode is a display-only label now: a desired/observed mismatch never
        # blocks commands, and set_desired_mode does not claim it reconfigured a
        # running server.
        _, client, safety = self.make_safety(sample_state(), desired="simulation", observed="mock")
        ok, reason = safety.jog_joint("left", 0, 1.0)
        self.assertTrue(ok, reason)
        self.assertEqual(client.sent_packets[-1]["mode"], "JointTarget")
        safety.set_desired_mode("real")
        self.assertIn("not reconfigured", safety.status_message)

    def test_desired_mode_button_color_marks_active_mode(self):
        self.assertEqual(_mode_button_color("simulation", "simulation"), "green")
        self.assertEqual(_mode_button_color("mock", "simulation"), "gray")

    def test_tcp_step_display_units_convert_to_command_units(self):
        self.assertAlmostEqual(_linear_step_meters(0.1), 0.0001)
        self.assertAlmostEqual(_linear_step_meters(10.0), 0.01)
        self.assertAlmostEqual(_angular_step_radians(0.1), math.radians(0.1))
        self.assertAlmostEqual(_angular_step_radians(10.0), math.radians(10.0))

    def test_simulator_lifecycle_enabled_from_state_without_env(self):
        # No RB_GUI_SIM_READINESS_* / RB_GUI_CARTESIAN_AVAILABLE env unlock needed:
        # a fresh, joint-valid, fault-free state alone enables lifecycle + jog.
        _, _, safety = self.make_safety(
            sample_state(),
            desired="simulation",
            observed="simulation",
            observed_backend="simulator",
        )
        states = safety.control_disabled_states()
        self.assertFalse(states["lifecycle:ArmMotion"])
        self.assertFalse(states["jog"])
        # And a stale stream re-locks everything (state is the sole authority).
        _, _, stale_safety = self.make_safety(
            sample_state(), observed="simulation", observed_backend="simulator", stale=True
        )
        stale_states = stale_safety.control_disabled_states()
        self.assertTrue(stale_states["lifecycle:ArmMotion"])
        self.assertTrue(stale_states["jog"])

    def test_tcp_jog_never_sends_cartesian_motion(self):
        _, client, safety = self.make_safety(sample_state())
        ok, reason = safety.tcp_jog_unavailable()
        self.assertFalse(ok)
        self.assertIn("no Cartesian motion command", reason)
        self.assertEqual(client.sent_packets, [])

    def test_tcp_pose_target_is_disabled_until_feature_flag(self):
        # RB_GUI_ENABLE_TCP_POSE_COMMANDS lock retired: TCP pose sends without
        # the feature flag whenever the state is valid.
        _, client, safety = self.make_safety(
            self.tcp_available_state(),
            sim_ready=True,
            cartesian_available=True,
        )
        pose = (0.3, 0.1, 0.4, 0.0, 0.0, 0.0)
        ok, reason = safety.send_tcp_pose_target(left_pose=pose)
        self.assertTrue(ok, reason)
        self.assertEqual(client.sent_packets[-1]["left"]["mode"], "TcpPoseTarget")

    def test_tcp_target_pose_helper_returns_finite_marker_pose(self):
        handles = {"left_tcp_target_pose": (0.31, 0.12, 0.44, 0.0, 0.0, 0.0)}
        self.assertEqual(_tcp_target_pose(handles, "left"), (0.31, 0.12, 0.44, 0.0, 0.0, 0.0))
        handles["left_tcp_target_pose"] = (0.31, 0.12, float("nan"), 0.0, 0.0, 0.0)
        self.assertIsNone(_tcp_target_pose(handles, "left"))

    def test_tcp_target_orientation_helper_prefers_marker_wxyz(self):
        handles = {
            "left_tcp_target_pose": (0.31, 0.12, 0.44, 0.0, 0.0, 0.0),
            "left_tcp_target_wxyz": (0.0, 0.0, 0.0, 2.0),
        }
        self.assertEqual(_tcp_target_wxyz(handles, "left"), (0.0, 0.0, 0.0, 1.0))
        handles.pop("left_tcp_target_wxyz")
        self.assertEqual(_tcp_target_wxyz(handles, "left"), (1.0, 0.0, 0.0, 0.0))

    def test_tcp_linear_move_packet_builder_uses_pose_object_with_quaternion(self):
        client = RecordingClient()
        pose = (0.35, 0.10, 0.45, 0.0, 0.0, 0.0)
        quaternion = (0.0, 0.0, 0.0, 1.0)
        packet = client.build_tcp_linear_move(
            left_pose=pose,
            left_quaternion_xyzw=quaternion,
            duration_sec=2.0,
            linear_speed_m_s=0.03,
            angular_speed_rad_s=0.2,
            orientation_mode="constant",
        )
        self.assertEqual(packet["schema_version"], 1)
        self.assertEqual(packet["mode"], "Hold")
        self.assertEqual(packet["left"]["mode"], "TcpLinearMove")
        self.assertEqual(packet["right"], {})
        # The command must stay fresh through the server's async collision-free decision
        # + path handoff (the old fixed 0.2 s expired mid-decision so the MoveL never ran).
        # Generous timeout = duration + 0.5 (floored at 2.0), covering planning handoff.
        self.assertEqual(packet["timeout_sec"], 2.5)
        self.assertEqual(packet["left"]["duration_sec"], 2.0)
        self.assertEqual(packet["left"]["linear_speed_m_s"], 0.03)
        self.assertEqual(packet["left"]["angular_speed_rad_s"], 0.2)
        self.assertEqual(packet["left"]["orientation_mode"], "constant")
        self.assertEqual(
            packet["left"]["target_tcp_stand"],
            {
                "x": 0.35,
                "y": 0.10,
                "z": 0.45,
                "rx": 0.0,
                "ry": 0.0,
                "rz": 0.0,
                "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            },
        )

    def test_tcp_pose_target_enabled_with_valid_fk_and_server_gate(self):
        state = self.tcp_available_state()
        _, client, safety = self.make_safety(
            state,
            desired="simulation",
            observed="simulation",
            observed_backend="simulator",
            sim_ready=True,
            cartesian_available=True,
            enable_tcp_pose=True,
        )
        self.assertIsNone(safety.tcp_command_disabled_reason("left"))
        self.assertFalse(safety.control_disabled_states()["tcp_pose"])

        pose = (0.31, 0.12, 0.44, 0.0, 0.0, 0.0)
        ok, reason = safety.send_tcp_pose_target(left_pose=pose)
        self.assertTrue(ok, reason)
        packet = client.sent_packets[-1]
        self.assertEqual(packet["mode"], "Hold")
        self.assertEqual(packet["left"]["mode"], "TcpPoseTarget")
        self.assertEqual(packet["left"]["tcp_target_stand"], list(pose))
        self.assertEqual(packet["right"], {})

        ok, reason = safety.send_tcp_pose_target(left_pose=pose, left_quaternion_xyzw=(0.0, 0.0, 1.0, 0.0))
        self.assertTrue(ok, reason)
        packet = client.sent_packets[-1]
        self.assertEqual(packet["left"]["mode"], "TcpPoseTarget")
        self.assertEqual(
            packet["left"]["tcp_target_stand"],
            {
                "x": 0.31,
                "y": 0.12,
                "z": 0.44,
                "rx": 0.0,
                "ry": 0.0,
                "rz": 0.0,
                "quaternion_xyzw": [0.0, 0.0, 1.0, 0.0],
            },
        )

        ok, reason = safety.send_tcp_linear_move(
            left_pose=pose,
            left_quaternion_xyzw=(0.0, 0.0, 1.0, 0.0),
            duration_sec=2.0,
            linear_speed_m_s=0.03,
            angular_speed_rad_s=0.2,
            orientation_mode="slerp",
        )
        self.assertTrue(ok, reason)
        packet = client.sent_packets[-1]
        self.assertEqual(packet["left"]["mode"], "TcpLinearMove")
        self.assertEqual(packet["left"]["duration_sec"], 2.0)
        self.assertEqual(packet["left"]["orientation_mode"], "slerp")
        self.assertEqual(packet["left"]["target_tcp_stand"]["quaternion_xyzw"], [0.0, 0.0, 1.0, 0.0])

    def test_tcp_pose_target_real_mode_uses_same_gui_gate(self):
        _, client, safety = self.make_safety(
            self.tcp_available_state(),
            desired="real",
            observed="real",
            observed_backend="rbpodo",
            sim_ready=True,
            cartesian_available=True,
            enable_tcp_pose=True,
        )
        # Real/sim gating retired: real-mode TCP target commands send normally.
        ok, reason = safety.send_tcp_pose_target(left_pose=(0.31, 0.12, 0.44, 0.0, 0.0, 0.0))
        self.assertTrue(ok, reason)
        ok, reason = safety.send_tcp_linear_move(left_pose=(0.31, 0.12, 0.44, 0.0, 0.0, 0.0), duration_sec=1.0)
        self.assertTrue(ok, reason)
        self.assertFalse(safety.control_disabled_states()["tcp_pose"])
        self.assertFalse(safety.control_disabled_states()["tcp_linear"])
        self.assertTrue(client.sent_packets)

    def test_tcp_command_requires_fk_and_server_cartesian_gate(self):
        state = self.tcp_available_state()
        _, _, backend_safety = self.make_safety(
            state,
            desired="simulation",
            observed="simulation",
            observed_backend="mock",
            cartesian_available=True,
        )
        # Backend requirement retired: any backend may issue TCP commands.
        self.assertIsNone(backend_safety.tcp_command_disabled_reason("left"))

        no_fk = sample_state()
        _, _, fk_safety = self.make_safety(
            no_fk,
            desired="simulation",
            observed="simulation",
            observed_backend="simulator",
            cartesian_available=True,
        )
        self.assertIn("FK/TCP pose unavailable", fk_safety.tcp_command_disabled_reason("left"))
        ok, reason = fk_safety.send_tcp_linear_move(left_pose=(0.31, 0.12, 0.44, 0.0, 0.0, 0.0), duration_sec=1.0)
        self.assertFalse(ok)
        self.assertIn("FK/TCP pose unavailable", reason)

        # Server-closed Cartesian gate (state-derived) blocks with the server's
        # own reason when present, else a clear default.
        cart_state = self.tcp_available_state()
        for arm in ("left", "right"):
            cart_state[arm]["cartesian_unavailable_reason"] = "IK not implemented"
        _, _, cart_safety = self.make_safety(
            cart_state,
            desired="simulation",
            observed="simulation",
            observed_backend="simulator",
            cartesian_available=False,
        )
        self.assertIn("IK not implemented", cart_safety.tcp_command_disabled_reason("left"))

    def test_tcp_command_allows_rbpodo_controller_simulation_with_optin(self):
        state = self.tcp_available_state()
        # rbpodo + operation_mode=simulation, both opt-ins on -> TCP allowed.
        _, _, allowed = self.make_safety(
            state,
            desired="simulation",
            observed="simulation",
            observed_backend="rbpodo",
            sim_ready=True,
            cartesian_available=True,
            enable_tcp_pose=True,
            enable_controller_sim_cartesian=True,
        )
        self.assertIsNone(allowed.tcp_command_disabled_reason("left"))
        ok, _ = allowed.send_tcp_pose_target(left_pose=(0.31, 0.12, 0.44, 0.0, 0.0, 0.0))
        self.assertTrue(ok)

        # rbpodo + simulation but controller-sim opt-in OFF -> still blocked.
        _, _, no_optin = self.make_safety(
            state,
            desired="simulation",
            observed="simulation",
            observed_backend="rbpodo",
            sim_ready=True,
            cartesian_available=True,
            enable_tcp_pose=True,
            enable_controller_sim_cartesian=False,
        )
        # Controller-sim opt-in retired: allowed without the env flag.
        self.assertIsNone(no_optin.tcp_command_disabled_reason("left"))

        # Real mode stays status-only even with the controller-sim opt-in set.
        _, _, real_safety = self.make_safety(
            state,
            desired="real",
            observed="real",
            observed_backend="rbpodo",
            sim_ready=True,
            cartesian_available=True,
            enable_tcp_pose=True,
            enable_controller_sim_cartesian=True,
        )
        # Real mode is no longer status-only.
        self.assertIsNone(real_safety.tcp_command_disabled_reason("left"))

        # Simulator backend behaviour is unchanged (no controller-sim opt-in needed).
        _, _, sim_safety = self.make_safety(
            state,
            desired="simulation",
            observed="simulation",
            observed_backend="simulator",
            sim_ready=True,
            cartesian_available=True,
            enable_tcp_pose=True,
        )
        self.assertIsNone(sim_safety.tcp_command_disabled_reason("left"))

    def test_tcp_commands_block_stale_and_faulted_state(self):
        state = self.tcp_available_state()
        _, client, stale_safety = self.make_safety(
            state,
            desired="simulation",
            observed="simulation",
            observed_backend="simulator",
            sim_ready=True,
            cartesian_available=True,
            enable_tcp_pose=True,
            stale=True,
        )
        ok, reason = stale_safety.send_tcp_pose_target(left_pose=(0.31, 0.12, 0.44, 0.0, 0.0, 0.0))
        self.assertFalse(ok)
        self.assertIn("state stream", reason)
        ok, reason = stale_safety.send_tcp_linear_move(left_pose=(0.31, 0.12, 0.44, 0.0, 0.0, 0.0), duration_sec=1.0)
        self.assertFalse(ok)
        self.assertIn("state stream", reason)
        self.assertEqual(client.sent_packets, [])

        faulted = self.tcp_available_state(fault_latched=True, fault_reason="test fault")
        _, client, fault_safety = self.make_safety(
            faulted,
            desired="simulation",
            observed="simulation",
            observed_backend="simulator",
            sim_ready=True,
            cartesian_available=True,
            enable_tcp_pose=True,
        )
        ok, reason = fault_safety.send_tcp_pose_target(left_pose=(0.31, 0.12, 0.44, 0.0, 0.0, 0.0))
        self.assertFalse(ok)
        self.assertIn("fault", reason)
        ok, reason = fault_safety.send_tcp_linear_move(left_pose=(0.31, 0.12, 0.44, 0.0, 0.0, 0.0), duration_sec=1.0)
        self.assertFalse(ok)
        self.assertIn("fault", reason)
        self.assertEqual(client.sent_packets, [])


    def test_scene_marker_helpers_use_mounts_and_joint_state(self):
        mounts = {"left": {"base_pose_in_stand": {"x": 1.0, "y": 2.0, "z": 3.0, "rx": 0.1, "ry": 0.2, "rz": 0.3}}}
        self.assertEqual(_mount_position(mounts, "left", (0.0, 0.0, 0.0)), (1.0, 2.0, 3.0))
        self.assertEqual(_pose6_from_mounts(mounts, "left", (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)), (1.0, 2.0, 3.0, 0.1, 0.2, 0.3))
        marker = _joint_marker_position((1.0, 2.0, 3.0), (180.0, -180.0, 90.0, 0.0, 0.0, 0.0))
        self.assertAlmostEqual(marker[0], 1.08)
        self.assertAlmostEqual(marker[1], 1.92)
        self.assertAlmostEqual(marker[2], 3.07)

    def test_pose_quaternion_and_joint_cfg_for_urdf_visualization(self):
        identity = _pose_wxyz((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertEqual(identity, (1.0, 0.0, 0.0, 0.0))
        yaw_180 = _pose_wxyz((0.0, 0.0, 0.0, 0.0, 0.0, math.pi))
        self.assertAlmostEqual(yaw_180[0], 0.0, places=7)
        self.assertAlmostEqual(yaw_180[3], 1.0, places=7)
        quaternion_pose = Pose6D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, quaternion_xyzw=(0.0, 0.0, 1.0, 0.0))
        self.assertEqual(_pose_orientation_wxyz(quaternion_pose), (0.0, 0.0, 0.0, 1.0))
        self.assertEqual(_wxyz_to_xyzw((0.0, 0.0, 0.0, 2.0)), (0.0, 0.0, 1.0, 0.0))
        cfg = _joint_cfg_radians((0.0, 90.0, -90.0))
        self.assertEqual(len(cfg), 6)
        self.assertAlmostEqual(cfg[1], math.pi / 2)
        self.assertAlmostEqual(cfg[2], -math.pi / 2)
        self.assertEqual(cfg[3:], (0.0, 0.0, 0.0))
        self.assertEqual(_format_joints(None), "invalid")
        self.assertEqual(_joint_cfg_radians(None), (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

    def test_geometry_module_preserves_quaternion_priority(self):
        pose = Pose6D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, quaternion_xyzw=(0.0, 0.0, 1.0, 0.0))
        self.assertEqual(gui_geometry._pose_orientation_wxyz(pose), (0.0, 0.0, 0.0, 1.0))
        self.assertEqual(gui_geometry._wxyz_to_xyzw((0.0, 0.0, 0.0, 2.0)), (0.0, 0.0, 1.0, 0.0))

    def test_geometry_module_applies_stand_and_local_deltas(self):
        yaw_90_pose = (0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2.0)
        target_transform = gui_geometry._pose_transform(yaw_90_pose[:3], gui_geometry._pose_wxyz(yaw_90_pose))
        delta_transform = gui_geometry._delta_transform((1.0, 0.0, 0.0, 0.0, 0.0, 0.0))

        stand_pose, _ = gui_geometry._transform_to_pose6(gui_geometry._multiply_transform(delta_transform, target_transform))
        local_pose, _ = gui_geometry._transform_to_pose6(gui_geometry._multiply_transform(target_transform, delta_transform))

        self.assertAlmostEqual(stand_pose[0], 1.0, places=7)
        self.assertAlmostEqual(stand_pose[1], 0.0, places=7)
        self.assertAlmostEqual(local_pose[0], 0.0, places=7)
        self.assertAlmostEqual(local_pose[1], 1.0, places=7)

    def test_joint_monitor_formats_actual_joints_in_degrees_or_radians(self):
        q_actual_deg = (0.0, -30.0, 90.0, 180.0, -180.0, 45.0)
        self.assertEqual(_format_joint_monitor_value(q_actual_deg, 0, valid=True, unit="deg"), "0.00 deg")
        self.assertEqual(_format_joint_monitor_value(q_actual_deg, 1, valid=True, unit="deg"), "-30.00 deg")
        self.assertEqual(_format_joint_monitor_value(q_actual_deg, 5, valid=True, unit="deg"), "45.00 deg")

        self.assertEqual(_format_joint_monitor_value(q_actual_deg, 1, valid=True, unit="rad"), "-0.5236 rad")
        self.assertEqual(_format_joint_monitor_value(q_actual_deg, 2, valid=True, unit="rad"), "1.5708 rad")
        self.assertEqual(_format_joint_monitor_value(q_actual_deg, 3, valid=True, unit="rad"), "3.1416 rad")

        raw_controller_q_actual_deg = (0.0, -30.0, 270.0, -317.0, 180.0, -180.0)
        self.assertEqual(_format_joint_monitor_value(raw_controller_q_actual_deg, 2, valid=True, unit="deg"), "270.00 deg")
        self.assertEqual(_format_joint_monitor_value(raw_controller_q_actual_deg, 3, valid=True, unit="deg"), "-317.00 deg")
        self.assertEqual(_format_joint_monitor_value(raw_controller_q_actual_deg, 2, valid=True, unit="rad"), "4.7124 rad")
        self.assertEqual(_format_joint_monitor_value(raw_controller_q_actual_deg, 3, valid=True, unit="rad"), "-5.5327 rad")

    def test_joint_monitor_invalid_state_does_not_fallback_to_zero(self):
        self.assertEqual(_format_joint_monitor_value(None, 0, valid=True, unit="deg"), "invalid")
        self.assertEqual(_format_joint_monitor_value((0.0, 1.0, 2.0), 0, valid=True, unit="deg"), "invalid")
        self.assertEqual(_format_joint_monitor_value((0.0, 1.0, 2.0, 3.0, 4.0, 5.0), 0, valid=False, unit="rad"), "invalid")
        self.assertEqual(_format_joint_monitor_value((0.0, 1.0, 2.0, 3.0, 4.0, 5.0), 6, valid=True, unit="deg"), "invalid")

    def test_joint_monitor_unit_buttons_highlight_selected_unit(self):
        handles = {
            "joint_monitor_unit": "deg",
            "joint_monitor_unit_buttons": {
                "deg": RecordingButton(),
                "rad": RecordingButton(),
            },
        }
        _update_joint_monitor_unit_buttons(handles)
        self.assertEqual(handles["joint_monitor_unit_buttons"]["deg"].color, "green")
        self.assertEqual(handles["joint_monitor_unit_buttons"]["rad"].color, "gray")

        handles["joint_monitor_unit"] = "rad"
        _update_joint_monitor_unit_buttons(handles)
        self.assertEqual(handles["joint_monitor_unit_buttons"]["deg"].color, "gray")
        self.assertEqual(handles["joint_monitor_unit_buttons"]["rad"].color, "green")

    def test_joint_monitor_updates_six_rows_per_arm(self):
        store, _, _ = self.make_safety(sample_state())
        handles = {
            "joint_monitor_unit": "deg",
            "joint_monitor_status": RecordingText(),
            "joint_monitor_values": {
                "left": [RecordingText() for _ in range(6)],
                "right": [RecordingText() for _ in range(6)],
            },
        }
        _update_joint_monitor(handles, store.latest(), stale=False)
        self.assertEqual(handles["joint_monitor_status"].value, "live, unit=deg, tick=1")
        self.assertEqual(handles["joint_monitor_values"]["left"][0].value, "0.00 deg")
        self.assertEqual(handles["joint_monitor_values"]["left"][1].value, "-30.00 deg")
        self.assertEqual(handles["joint_monitor_values"]["right"][2].value, "80.00 deg")
        self.assertEqual(len(handles["joint_monitor_values"]["left"]), 6)
        self.assertEqual(len(handles["joint_monitor_values"]["right"]), 6)

        handles["joint_monitor_unit"] = "rad"
        _update_joint_monitor(handles, store.latest(), stale=True)
        self.assertEqual(handles["joint_monitor_status"].value, "stale, unit=rad, tick=1")
        self.assertEqual(handles["joint_monitor_values"]["left"][1].value, "-0.5236 rad")

    def test_stand_world_monitor_formats_pose_in_mm_and_degrees_or_radians(self):
        pose = Pose6D(0.31, -0.12, 0.44, 0.0, -math.pi / 4.0, math.pi / 2.0)
        self.assertEqual(_format_stand_world_pose_value(pose, "x", valid=True, unit="deg"), "310.0 mm")
        self.assertEqual(_format_stand_world_pose_value(pose, "y", valid=True, unit="deg"), "-120.0 mm")
        self.assertEqual(_format_stand_world_pose_value(pose, "z", valid=True, unit="deg"), "440.0 mm")
        self.assertEqual(_format_stand_world_pose_value(pose, "ry", valid=True, unit="deg"), "-45.00 deg")
        self.assertEqual(_format_stand_world_pose_value(pose, "rz", valid=True, unit="rad"), "1.5708 rad")

    def test_stand_world_monitor_invalid_state_does_not_fallback_to_zero(self):
        pose = Pose6D(0.31, -0.12, 0.44, 0.0, -math.pi / 4.0, math.pi / 2.0)
        self.assertEqual(_format_stand_world_pose_value(None, "x", valid=True, unit="deg"), "invalid")
        self.assertEqual(_format_stand_world_pose_value(pose, "x", valid=False, unit="deg"), "invalid")
        self.assertEqual(_format_stand_world_pose_value(pose, "bad", valid=True, unit="deg"), "invalid")

    def test_stand_world_monitor_unit_buttons_highlight_selected_unit(self):
        handles = {
            "stand_world_monitor_unit": "deg",
            "stand_world_monitor_unit_buttons": {
                "deg": RecordingButton(),
                "rad": RecordingButton(),
            },
        }
        _update_stand_world_monitor_unit_buttons(handles)
        self.assertEqual(handles["stand_world_monitor_unit_buttons"]["deg"].color, "green")
        self.assertEqual(handles["stand_world_monitor_unit_buttons"]["rad"].color, "gray")

        handles["stand_world_monitor_unit"] = "rad"
        _update_stand_world_monitor_unit_buttons(handles)
        self.assertEqual(handles["stand_world_monitor_unit_buttons"]["deg"].color, "gray")
        self.assertEqual(handles["stand_world_monitor_unit_buttons"]["rad"].color, "green")

    def test_stand_world_monitor_updates_pose_rows_per_arm(self):
        state = self.tcp_available_state()
        state["left"]["tcp_stand"] = {
            "x": 0.31,
            "y": 0.12,
            "z": 0.44,
            "rx": 0.0,
            "ry": 0.0,
            "rz": math.pi / 2.0,
        }
        state["right"]["tcp_stand"] = {
            "x": -0.2,
            "y": 0.15,
            "z": 0.5,
            "rx": math.pi,
            "ry": 0.0,
            "rz": -math.pi / 2.0,
        }
        store, _, _ = self.make_safety(state)
        handles = {
            "stand_world_monitor_unit": "deg",
            "stand_world_monitor_status": RecordingText(),
            "stand_world_monitor_values": {
                "left": {field: RecordingText() for field in ("x", "y", "z", "rx", "ry", "rz")},
                "right": {field: RecordingText() for field in ("x", "y", "z", "rx", "ry", "rz")},
            },
        }
        _update_stand_world_monitor(handles, store.latest(), stale=False)
        self.assertEqual(handles["stand_world_monitor_status"].value, "live, xyz=mm, rpy=deg, tick=1")
        self.assertEqual(handles["stand_world_monitor_values"]["left"]["x"].value, "310.0 mm")
        self.assertEqual(handles["stand_world_monitor_values"]["left"]["rz"].value, "90.00 deg")
        self.assertEqual(handles["stand_world_monitor_values"]["right"]["x"].value, "-200.0 mm")
        self.assertEqual(handles["stand_world_monitor_values"]["right"]["rx"].value, "180.00 deg")

        handles["stand_world_monitor_unit"] = "rad"
        _update_stand_world_monitor(handles, store.latest(), stale=False)
        self.assertEqual(handles["stand_world_monitor_status"].value, "live, xyz=mm, rpy=rad, tick=1")
        self.assertEqual(handles["stand_world_monitor_values"]["left"]["rz"].value, "1.5708 rad")

    def test_stand_world_monitor_marks_unavailable_pose_invalid(self):
        store, _, _ = self.make_safety(sample_state())
        handles = {
            "stand_world_monitor_unit": "deg",
            "stand_world_monitor_status": RecordingText(),
            "stand_world_monitor_values": {
                "left": {field: RecordingText() for field in ("x", "y", "z", "rx", "ry", "rz")},
                "right": {field: RecordingText() for field in ("x", "y", "z", "rx", "ry", "rz")},
            },
        }
        _update_stand_world_monitor(handles, store.latest(), stale=False)
        self.assertEqual(handles["stand_world_monitor_status"].value, "live, xyz=mm, rpy=deg, tick=1")
        self.assertEqual(handles["stand_world_monitor_values"]["left"]["x"].value, "invalid")

        _update_stand_world_monitor(handles, None, stale=True)
        self.assertEqual(handles["stand_world_monitor_status"].value, "No state stream, xyz=mm, rpy=deg")
        self.assertEqual(handles["stand_world_monitor_values"]["right"]["rz"].value, "invalid")

        state = self.tcp_available_state()
        store, _, _ = self.make_safety(state)
        _update_stand_world_monitor(handles, store.latest(), stale=True)
        self.assertEqual(handles["stand_world_monitor_status"].value, "stale, xyz=mm, rpy=deg, tick=1")
        self.assertEqual(handles["stand_world_monitor_values"]["left"]["x"].value, "invalid")

    def test_tab_theme_html_colors_main_and_sub_levels_differently(self):
        light = _tab_theme_html(dark=False)
        # main tab bar styled at the top level...
        self.assertIn(".mantine-Tabs-list", light)
        self.assertIn("#2563eb", light)  # main accent (blue)
        # ...sub bars scoped to nested panels with a different hue
        self.assertIn(".mantine-Tabs-panel .mantine-Tabs-list", light)
        self.assertIn("#7c3aed", light)  # sub accent (purple)
        self.assertIn("<style>", light)

    def test_tab_theme_html_dark_palette_distinct_from_light(self):
        dark = _tab_theme_html(dark=True)
        # Dark is the default and uses brighter accents for legibility on the
        # dark panel surface (and is what _tab_theme_html() returns by default).
        self.assertEqual(dark, _tab_theme_html())
        self.assertIn("#5b8cff", dark)  # main accent (brighter blue)
        self.assertIn("#a07bff", dark)  # sub accent (brighter purple)
        self.assertNotIn("#2563eb", dark)
        self.assertIn(".mantine-Tabs-panel .mantine-Tabs-list", dark)

    def test_nudge_label_pads_to_equal_width_with_nbsp(self):
        # Different-length axis labels become the same display width so the
        # −/+ button-group segments line up vertically across rows.
        short = _nudge_label("X")
        long = _nudge_label("Pitch")
        self.assertEqual(len(short), len(long))
        self.assertIn("X", short)
        self.assertIn("Pitch", long)
        self.assertIn(" ", short)  # padded with non-breaking space, not plain space
        # the bare value is still recoverable by stripping NBSP
        self.assertEqual(short.replace(" ", ""), "X")

    def test_status_summary_html_colors_chips_by_state(self):
        good = _status_summary_html(
            connection="live", mode="sim", readiness_go=True, motion="ArmedHold", fault_active=False
        )
        # all-good: connection/readiness/fault chips use the ok tone (green dot)...
        self.assertIn("연결", good)
        self.assertIn("Go", good)
        self.assertIn("없음", good)
        self.assertIn("#22aa63", good)  # ok dot
        self.assertNotIn("#dc4646", good)  # no bad dot when healthy

        bad = _status_summary_html(
            connection="disconnected", mode="real", readiness_go=False, motion="FaultLatched", fault_active=True
        )
        self.assertIn("No-Go", bad)
        self.assertIn("FAULT", bad)
        self.assertIn("#dc4646", bad)  # bad dot present

    def test_operator_monitor_static_html_stacks_pose_below_joint(self):
        html = _operator_monitor_static_html(18.0, 1.0, 31.5)
        self.assertIn("--rb-monitor-gap: 1.000em;", html)
        self.assertIn("--rb-monitor-target-width: 18.000em;", html)
        self.assertIn("--rb-monitor-split: min(31.500em, 60vh);", html)
        # Both monitors share the left column...
        self.assertIn(".rb-monitor-joint-card { left: var(--rb-monitor-gap); }", html)
        self.assertIn(".rb-monitor-stand-card { left: var(--rb-monitor-gap); }", html)
        # ...with the Pose Monitor stacked just below the Joint Monitor's content.
        self.assertIn(".rb-monitor-stand-card.rb-monitor-header-card { top: var(--rb-monitor-split); }", html)
        self.assertIn(".rb-monitor-joint-card.rb-monitor-body-card { max-height: calc(var(--rb-monitor-split) - 5.95em); }", html)
        self.assertIn("Pose Monitor", html)
        self.assertIn('id="rb-joint-unit-rad"', html)
        self.assertIn('id="rb-stand-unit-rad"', html)
        self.assertIn("body:has(#rb-joint-unit-rad:checked)", html)
        self.assertNotIn(".rb-monitor-card { display: none", html)

    def test_operator_monitor_dynamic_html_renders_joint_and_stand_values(self):
        state = self.tcp_available_state()
        state["left"]["tcp_stand"] = {
            "x": 0.31,
            "y": 0.12,
            "z": 0.44,
            "rx": 0.0,
            "ry": 0.0,
            "rz": math.pi / 2.0,
        }
        store, _, _ = self.make_safety(state)
        html = _operator_monitor_dynamic_html(store.latest(), stale=False)
        self.assertIn("live, tick=1", html)
        self.assertIn("J1 base", html)  # label shortened (drops "_joint" suffix)
        self.assertIn("-30.00 deg", html)  # negative angle renders with its sign (J2 shoulder)
        self.assertIn("0.00 deg", html)
        self.assertIn("0.0000 rad", html)
        # The Pose Monitor status no longer carries the "xyz=mm" unit hint.
        self.assertNotIn("xyz=mm", html)
        self.assertIn("310.0 mm", html)
        self.assertIn("90.00 deg", html)
        self.assertIn("1.5708 rad", html)

    def test_operator_monitor_dynamic_html_marks_unavailable_pose_invalid(self):
        store, _, _ = self.make_safety(sample_state())
        html = _operator_monitor_dynamic_html(store.latest(), stale=False)
        self.assertIn("live, tick=1", html)
        self.assertNotIn("xyz=mm", html)
        self.assertIn("invalid", html)

        state = self.tcp_available_state()
        store, _, _ = self.make_safety(state)
        stale_html = _operator_monitor_dynamic_html(store.latest(), stale=True)
        self.assertIn("stale, tick=1", stale_html)
        self.assertNotIn("xyz=mm", stale_html)
        self.assertIn("invalid", stale_html)

    def test_operator_monitors_use_fixed_html_overlay_when_available(self):
        server = RecordingServer(scene=RecordingScene())
        handles = {}
        _build_operator_monitors(server, handles)
        self.assertEqual(handles["operator_monitor_panel_mode"], "fixed_html_overlay")
        self.assertEqual(len(server.gui.html), 2)
        self.assertIn("rb-monitor-header-card", server.gui.html[0][0])
        self.assertIn("rb-monitor-body-card", server.gui.html[1][0])
        self.assertEqual(server.scene.containers, [])
        self.assertEqual(server.gui.folders, [])

    def test_operator_monitors_fallback_to_root_gui_without_html(self):
        server = RecordingServer(scene=RecordingScene(), has_html=False)
        handles = {}
        _build_operator_monitors(server, handles)
        self.assertEqual(handles["operator_monitor_panel_mode"], "root_gui_fallback")
        folder_labels = [label for label, _kwargs in server.gui.folders]
        self.assertEqual(folder_labels[0], "Operator Monitors")
        self.assertIn("Joint Monitor", folder_labels)
        self.assertIn("Stand/World Monitor", folder_labels)

    def test_safety_floor_user_floor_roi_folder_order(self):
        source = (Path(__file__).resolve().parents[1] / "rb_servo_gui" / "app.py").read_text(
            encoding="utf-8"
        )
        stand = source.index('add_folder("Stand Safety Floor")')
        user = source.index('add_folder("User Safety Floor")')
        roi = source.index('add_folder("Safety ROI box")')
        self.assertLess(stand, user)
        self.assertLess(user, roi)

    def test_update_operator_monitors_updates_html_content_handle(self):
        server = RecordingServer()
        handles = {}
        _build_operator_monitors(server, handles)
        state = self.tcp_available_state()
        store, _, _ = self.make_safety(state)
        _update_operator_monitors(handles, store.latest(), stale=False)
        self.assertIn("live, tick=1", handles["operator_monitor_content"].content)
        self.assertNotIn("xyz=mm", handles["operator_monitor_content"].content)
        # Server uptime (hh:mm:ss) is shown next to the tick on both monitors.
        self.assertIn("tick=1, up=00:00:00", handles["operator_monitor_content"].content)

    def test_server_uptime_hms_from_tick_and_loop_clock(self):
        # tick starts at 0 at server start; loop_start_time_ns is a steady clock,
        # so uptime == tick * loop_period. First sample uses the published period_ms
        # (5 ms here) as the fallback period; later samples self-calibrate the period
        # from the (loop_start, tick) delta.
        handles = {}
        first = self.make_safety(
            self.tcp_available_state(tick=600_000, loop_start_time_ns=5_000_000_000)
        )[0].latest()
        # 600000 ticks * 5 ms = 3000 s = 00:50:00
        self.assertEqual(_server_uptime_hms(handles, first), "00:50:00")
        # 1 s later (tick +500, loop +1e9 ns) the measured period is 2 ms.
        second = self.make_safety(
            self.tcp_available_state(tick=600_500, loop_start_time_ns=6_000_000_000)
        )[0].latest()
        # 600500 * 2 ms = 1201 s = 00:20:01
        self.assertEqual(_server_uptime_hms(handles, second), "00:20:01")
        self.assertIsNone(_server_uptime_hms({}, None))
        self.assertEqual(_format_hms(90_061), "25:01:01")

    def test_default_mount_normals_match_stand_shoulder_faces(self):
        left_matrix = _quat_to_matrix(_pose_orientation_wxyz(_DEFAULT_LEFT_POSE))
        right_matrix = _quat_to_matrix(_pose_orientation_wxyz(_DEFAULT_RIGHT_POSE))
        left_normal = tuple(-left_matrix[row][2] for row in range(3))
        right_normal = tuple(right_matrix[row][2] for row in range(3))
        for actual, expected in zip(left_normal, (-0.709, -0.500, 0.498)):
            self.assertAlmostEqual(actual, expected, delta=0.01)
        for actual, expected in zip(right_normal, (-0.708, 0.499, -0.500)):
            self.assertAlmostEqual(actual, expected, delta=0.01)

    def test_mount_parsing_preserves_future_quaternion_orientation(self):
        mounts = {
            "left": {
                "base_pose_in_stand": {
                    "x": 1.0,
                    "y": 2.0,
                    "z": 3.0,
                    "rx": 0.0,
                    "ry": 0.0,
                    "rz": 0.0,
                    "quaternion_xyzw": [0.0, 0.0, 1.0, 0.0],
                }
            }
        }
        pose = _mount_pose_from_mounts(mounts, "left", _DEFAULT_LEFT_POSE)
        self.assertEqual(pose.as_tuple(), (1.0, 2.0, 3.0, 0.0, 0.0, 0.0))
        self.assertEqual(_pose_orientation_wxyz(pose), (0.0, 0.0, 0.0, 1.0))

    def test_scene_update_sets_mount_pose_and_urdf_joint_config(self):
        store, _, _ = self.make_safety(
            sample_state(
                mounts={
                    "left": {"base_pose_in_stand": {"x": 1.0, "y": 2.0, "z": 3.0, "rx": 0.0, "ry": 0.0, "rz": math.pi}},
                    "right": {"base_pose_in_stand": {"x": -1.0, "y": -2.0, "z": -3.0, "rx": 0.0, "ry": math.pi, "rz": 0.0}},
                }
            )
        )
        latest = store.latest()
        left_base = RecordingSceneHandle()
        right_base = RecordingSceneHandle()
        left_urdf = RecordingUrdf()
        right_urdf = RecordingUrdf()
        handles = {
            "left_base": left_base,
            "right_base": right_base,
            "left_urdf": left_urdf,
            "right_urdf": right_urdf,
        }
        update_scene_markers(handles, latest)
        self.assertEqual(left_base.position, (1.0, 2.0, 3.0))
        self.assertEqual(right_base.position, (-1.0, -2.0, -3.0))
        self.assertAlmostEqual(left_base.wxyz[3], 1.0, places=7)
        self.assertEqual(len(left_urdf.configs[-1]), 6)
        self.assertAlmostEqual(left_urdf.configs[-1][1], math.radians(-30.0))
        self.assertAlmostEqual(right_urdf.configs[-1][2], math.radians(80.0))

    def test_scene_update_uses_mount_quaternion_over_legacy_rpy(self):
        store, _, _ = self.make_safety(
            sample_state(
                mounts={
                    "left": {
                        "base_pose_in_stand": {
                            "x": 1.0,
                            "y": 2.0,
                            "z": 3.0,
                            "rx": 0.0,
                            "ry": 0.0,
                            "rz": 0.0,
                            "quaternion_xyzw": [0.0, 0.0, 1.0, 0.0],
                        }
                    }
                }
            )
        )
        left_base = RecordingSceneHandle()
        handles = {"left_base": left_base}
        update_scene_markers(handles, store.latest())
        self.assertEqual(left_base.position, (1.0, 2.0, 3.0))
        self.assertEqual(left_base.wxyz, (0.0, 0.0, 0.0, 1.0))

    def test_scene_update_sets_tcp_target_from_tcp_stand_without_base_axes(self):
        state = sample_state()
        state["left"]["tcp_stand"] = {"x": 0.31, "y": 0.12, "z": 0.44, "rx": 0.0, "ry": 0.0, "rz": math.pi}
        state["left"]["has_valid_tcp_pose"] = True
        state["left"]["tcp_deferred"] = False
        store, _, _ = self.make_safety(state)
        latest = store.latest()
        left_tcp = RecordingSceneHandle()
        left_target = RecordingSceneHandle()
        handles = {"left_tcp": left_tcp, "left_tcp_target": left_target}
        update_scene_markers(handles, latest)
        self.assertEqual(left_tcp.position, (0.31, 0.12, 0.44))
        self.assertEqual(left_target.position, (0.31, 0.12, 0.44))
        self.assertAlmostEqual(left_tcp.wxyz[3], 1.0, places=7)
        self.assertEqual(handles["left_tcp_target_pose"][:3], (0.31, 0.12, 0.44))

        left_target.position = (9.0, 9.0, 9.0)
        handles["left_tcp_target_user_moved"] = True
        state["left"]["tcp_stand"] = {"x": 0.5, "y": 0.5, "z": 0.5, "rx": 0.0, "ry": 0.0, "rz": 0.0}
        store, _, _ = self.make_safety(state)
        update_scene_markers(handles, store.latest())
        self.assertEqual(left_tcp.position, (0.5, 0.5, 0.5))
        self.assertEqual(left_target.position, (9.0, 9.0, 9.0))

        _clear_tcp_target_user_moved(handles, ("left",))
        update_scene_markers(handles, store.latest())
        self.assertEqual(left_target.position, (0.5, 0.5, 0.5))
        self.assertEqual(handles["left_tcp_target_pose"][:3], (0.5, 0.5, 0.5))

    def test_scene_update_distinguishes_actual_and_reference_tcp_frames(self):
        state = sample_state()
        state["left"]["tcp_stand"] = {"x": 0.31, "y": 0.12, "z": 0.44, "rx": 0.0, "ry": 0.0, "rz": 0.0}
        state["left"]["tcp_actual_stand"] = {"x": 0.32, "y": 0.13, "z": 0.45, "rx": 0.0, "ry": 0.0, "rz": 0.0}
        state["left"]["tcp_ref_stand"] = {"x": 0.41, "y": 0.21, "z": 0.51, "rx": 0.0, "ry": 0.0, "rz": math.pi}
        state["left"]["has_valid_tcp_pose"] = True
        state["left"]["tcp_actual_valid"] = True
        state["left"]["tcp_ref_valid"] = True
        state["left"]["tcp_tracking_source_recommendation"] = "tcp_ref_stand"
        state["left"]["tcp_deferred"] = False
        store, _, _ = self.make_safety(state)
        left_tcp = RecordingSceneHandle()
        left_ref = RecordingSceneHandle()
        left_target = RecordingSceneHandle()
        left_tcp_label = RecordingSceneHandle()
        left_ref_label = RecordingSceneHandle()
        handles = {
            "left_tcp": left_tcp,
            "left_tcp_ref": left_ref,
            "left_tcp_target": left_target,
            "left_tcp_label": left_tcp_label,
            "left_tcp_ref_label": left_ref_label,
            "left_tcp_trail": RecordingSceneHandle(),
            "left_tcp_ref_trail": RecordingSceneHandle(),
            "left_tcp_trail_points": [],
            "left_tcp_ref_trail_points": [],
        }
        update_scene_markers(handles, store.latest())
        self.assertEqual(left_tcp.position, (0.32, 0.13, 0.45))
        self.assertEqual(left_ref.position, (0.41, 0.21, 0.51))
        self.assertEqual(left_target.position, (0.32, 0.13, 0.45))
        self.assertFalse(left_tcp.visible)
        self.assertTrue(left_ref.visible)
        self.assertFalse(left_tcp_label.visible)
        self.assertTrue(left_ref_label.visible)
        self.assertEqual(left_tcp_label.position, (0.32, 0.13, 0.495))
        self.assertEqual(left_ref_label.position, (0.41, 0.21, 0.555))
        self.assertFalse(handles["left_tcp_trail"].visible)
        self.assertTrue(handles["left_tcp_ref_trail"].visible)
        self.assertAlmostEqual(left_ref.wxyz[3], 1.0, places=7)

        update_scene_markers(handles, store.latest(), tcp_display_mode="both")
        self.assertTrue(left_tcp.visible)
        self.assertTrue(left_ref.visible)
        self.assertTrue(left_tcp_label.visible)
        self.assertTrue(left_ref_label.visible)

        update_scene_markers(handles, store.latest(), tcp_display_mode="actual")
        self.assertTrue(left_tcp.visible)
        self.assertFalse(left_ref.visible)
        self.assertTrue(left_tcp_label.visible)
        self.assertFalse(left_ref_label.visible)

    def test_floor_check_points_toggle_gated_on_actual_tcp(self):
        state = sample_state()
        # Left has a valid actual TCP pose; right does not (deferred FK).
        state["left"]["tcp_actual_stand"] = {"x": 0.3, "y": 0.1, "z": 0.4, "rx": 0.0, "ry": 0.0, "rz": 0.0}
        state["left"]["has_valid_tcp_pose"] = True
        state["left"]["tcp_actual_valid"] = True
        state["left"]["tcp_deferred"] = False
        state["right"]["tcp_actual_valid"] = False
        state["right"]["tcp_deferred"] = True
        store, _, _ = self.make_safety(state)
        left_pts = RecordingSceneHandle()
        right_pts = RecordingSceneHandle()
        handles = {"left_floor_check_points": left_pts, "right_floor_check_points": right_pts}

        # Toggle off: both hidden regardless of TCP validity.
        update_floor_check_points(handles, store.latest(), show=False)
        self.assertFalse(left_pts.visible)
        self.assertFalse(right_pts.visible)

        # Toggle on: only the arm with a valid actual TCP shows its points.
        update_floor_check_points(handles, store.latest(), show=True)
        self.assertTrue(left_pts.visible)
        self.assertFalse(right_pts.visible)

    def test_interpolated_floor_check_points_matches_server_lerp(self):
        open_pts = np.asarray(_FLOOR_CHECK_POINTS_TCP_FRAME, dtype=np.float32)
        closed_pts = np.asarray(_FLOOR_CHECK_POINTS_TCP_FRAME_CLOSED, dtype=np.float32)
        # percent=100 -> open; percent=0 -> closed; midpoint -> mean.
        np.testing.assert_allclose(_interpolated_floor_check_points(100.0), open_pts)
        np.testing.assert_allclose(_interpolated_floor_check_points(0.0), closed_pts)
        np.testing.assert_allclose(
            _interpolated_floor_check_points(50.0), (open_pts + closed_pts) / 2.0, atol=1e-6
        )
        # Out-of-range percent clamps; None/invalid -> conservative OPEN fallback.
        np.testing.assert_allclose(_interpolated_floor_check_points(250.0), open_pts)
        np.testing.assert_allclose(_interpolated_floor_check_points(-10.0), closed_pts)
        np.testing.assert_allclose(_interpolated_floor_check_points(None), open_pts)
        np.testing.assert_allclose(_interpolated_floor_check_points("bad"), open_pts)
        # The TCP point (index 0) never moves with the gripper.
        np.testing.assert_allclose(_interpolated_floor_check_points(40.0)[0], (0.0, 0.0, 0.0))

    def test_floor_check_points_track_gripper_percent(self):
        state = sample_state()
        state["left"]["tcp_actual_stand"] = {"x": 0.3, "y": 0.1, "z": 0.4, "rx": 0.0, "ry": 0.0, "rz": 0.0}
        state["left"]["has_valid_tcp_pose"] = True
        state["left"]["tcp_actual_valid"] = True
        state["left"]["tcp_deferred"] = False
        store, _, _ = self.make_safety(state)
        left_pts = RecordingSceneHandle()
        # gripper_percent_left mirrors what _push_gripper_percent writes into scene.
        handles = {"left_floor_check_points": left_pts, "gripper_percent_left": 0.0}
        update_floor_check_points(handles, store.latest(), show=True)
        self.assertTrue(left_pts.visible)
        np.testing.assert_allclose(
            np.asarray(left_pts.points),
            np.asarray(_FLOOR_CHECK_POINTS_TCP_FRAME_CLOSED, dtype=np.float32),
        )
        # Re-run fully open -> the wider open set.
        handles["gripper_percent_left"] = 100.0
        update_floor_check_points(handles, store.latest(), show=True)
        np.testing.assert_allclose(
            np.asarray(left_pts.points),
            np.asarray(_FLOOR_CHECK_POINTS_TCP_FRAME, dtype=np.float32),
        )

    def test_scene_fallback_labels_actual_and_reference_tcp_markers(self):
        server = RecordingServer(scene=RecordingScene())
        _add_scene_fallback(server)
        label_texts = [text for _name, text, _kwargs, _handle in server.scene.labels]
        self.assertIn("left tcp_actual_stand physical-state inspection", label_texts)
        self.assertIn("left tcp_ref_stand controller-sim reference", label_texts)
        self.assertIn("right tcp_actual_stand physical-state inspection", label_texts)
        self.assertIn("right tcp_ref_stand controller-sim reference", label_texts)

    def test_scene_fallback_uses_viser_compatible_empty_geometry_arrays(self):
        scene = ShapeCheckingScene()
        server = RecordingServer(scene=scene)
        handles = _add_scene_fallback(server)
        self.assertNotIn("scene_error", handles)
        self.assertIn("stand_mesh", handles)
        # The four TCP trails (left/right × actual/reference) render as gradient
        # comet-trail line segments, plus the circle overlay line.
        self.assertGreaterEqual(len(scene.line_segments), 5)

    def test_circle_overlay_points_support_standard_planes(self):
        xy = CircleOverlaySnapshot.parse(sample_circle_overlay(plane="xy", axis1_stand=None, axis2_stand=None))
        xz = CircleOverlaySnapshot.parse(sample_circle_overlay(plane="xz", axis1_stand=None, axis2_stand=None))
        yz = CircleOverlaySnapshot.parse(sample_circle_overlay(plane="yz", axis1_stand=None, axis2_stand=None))
        self.assertIsNotNone(xy)
        self.assertIsNotNone(xz)
        self.assertIsNotNone(yz)
        xy_points = _circle_overlay_points(xy, segments=8)
        xz_points = _circle_overlay_points(xz, segments=8)
        yz_points = _circle_overlay_points(yz, segments=8)
        self.assertTrue(all(abs(point[2] - xy.center_stand[2]) < 1e-9 for point in xy_points))
        self.assertTrue(all(abs(point[1] - xz.center_stand[1]) < 1e-9 for point in xz_points))
        self.assertTrue(all(abs(point[0] - yz.center_stand[0]) < 1e-9 for point in yz_points))

    def test_circle_overlay_scene_updates_and_hides_stale_overlay(self):
        overlay = CircleOverlaySnapshot.parse(sample_circle_overlay())
        self.assertIsNotNone(overlay)
        line = RecordingSceneHandle()
        desired = RecordingSceneHandle()
        handles = {
            "circle_overlay_line_mode": "point_cloud",
            "circle_overlay_line": line,
            "circle_overlay_desired": desired,
        }
        update_circle_overlay(handles, overlay, stale=False)
        self.assertTrue(line.visible)
        self.assertTrue(desired.visible)
        self.assertGreater(len(line.points), 8)
        self.assertEqual(desired.position, (0.175, 0.2, 0.3))
        self.assertEqual(desired.wxyz, (1.0, 0.0, 0.0, 0.0))

        update_circle_overlay(handles, overlay, stale=True)
        self.assertFalse(line.visible)
        self.assertFalse(desired.visible)

    def test_scene_update_uses_tcp_quaternion_over_legacy_rpy(self):
        state = sample_state()
        state["left"]["tcp_stand"] = {
            "x": 0.31,
            "y": 0.12,
            "z": 0.44,
            "rx": 0.0,
            "ry": 0.0,
            "rz": 0.0,
            "quaternion_xyzw": [0.0, 0.0, 1.0, 0.0],
        }
        state["left"]["has_valid_tcp_pose"] = True
        state["left"]["tcp_deferred"] = False
        store, _, _ = self.make_safety(state)
        left_tcp = RecordingSceneHandle()
        left_target = RecordingSceneHandle()
        handles = {"left_tcp": left_tcp, "left_tcp_target": left_target}
        update_scene_markers(handles, store.latest())
        self.assertEqual(left_tcp.wxyz, (0.0, 0.0, 0.0, 1.0))
        self.assertEqual(left_target.wxyz, (0.0, 0.0, 0.0, 1.0))
        self.assertEqual(handles["left_tcp_target_wxyz"], (0.0, 0.0, 0.0, 1.0))

    def test_tcp_local_delta_uses_current_tcp_quaternion_over_legacy_rpy(self):
        yaw_90_xyzw = (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
        current = Pose6D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, quaternion_xyzw=yaw_90_xyzw)
        target_wxyz = _pose_orientation_wxyz(current)
        delta = _tcp_local_delta_from_target(current, (0.0, 1.0, 0.0), target_wxyz)
        self.assertAlmostEqual(delta[0], 1.0, places=7)
        self.assertAlmostEqual(delta[1], 0.0, places=7)
        self.assertAlmostEqual(delta[2], 0.0, places=7)
        self.assertAlmostEqual(delta[5], 0.0, places=7)

    def test_tcp_target_marker_delta_follows_selected_frame(self):
        stand_handle = RecordingSceneHandle()
        stand_handles = {
            "left_tcp_target": stand_handle,
            "left_tcp_target_pose": (0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2.0),
            "left_tcp_target_wxyz": _pose_wxyz((0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2.0)),
        }
        self.assertTrue(_apply_tcp_pose_step_to_target(stand_handles, "left", (1.0, 0.0, 0.0, 0.0, 0.0, 0.0), _TCP_FRAME_STAND))
        self.assertAlmostEqual(stand_handles["left_tcp_target_pose"][0], 1.0, places=7)
        self.assertAlmostEqual(stand_handles["left_tcp_target_pose"][1], 0.0, places=7)

        local_handle = RecordingSceneHandle()
        local_handles = {
            "left_tcp_target": local_handle,
            "left_tcp_target_pose": (0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2.0),
            "left_tcp_target_wxyz": _pose_wxyz((0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2.0)),
        }
        self.assertTrue(_apply_tcp_pose_step_to_target(local_handles, "left", (1.0, 0.0, 0.0, 0.0, 0.0, 0.0), _TCP_FRAME_LOCAL))
        self.assertAlmostEqual(local_handles["left_tcp_target_pose"][0], 0.0, places=7)
        self.assertAlmostEqual(local_handles["left_tcp_target_pose"][1], 1.0, places=7)
        self.assertEqual(local_handle.position, local_handles["left_tcp_target_pose"][:3])

    def test_tcp_ptp_delta_sends_absolute_pose_target_with_quaternion(self):
        state = self.tcp_available_state()
        _, client, safety = self.make_safety(
            state,
            desired="simulation",
            observed="simulation",
            observed_backend="simulator",
            sim_ready=True,
            cartesian_available=True,
            enable_tcp_pose=True,
        )
        yaw_90_pose = (0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2.0)
        handles = {
            "left_tcp_target": RecordingSceneHandle(),
            "left_tcp_target_pose": yaw_90_pose,
            "left_tcp_target_wxyz": _pose_wxyz(yaw_90_pose),
        }

        ok, reason = _apply_tcp_pose_step_and_send_pose_target(
            safety,
            handles,
            "left",
            (0.005, 0.0, 0.0, 0.0, 0.0, 0.0),
            _TCP_FRAME_LOCAL,
        )
        self.assertTrue(ok, reason)
        self.assertAlmostEqual(handles["left_tcp_target_pose"][0], 0.0, places=7)
        self.assertAlmostEqual(handles["left_tcp_target_pose"][1], 0.005, places=7)

        packet = client.sent_packets[-1]
        self.assertEqual(packet["mode"], "Hold")
        self.assertEqual(packet["left"]["mode"], "TcpPoseTarget")
        target = packet["left"]["tcp_target_stand"]
        self.assertIsInstance(target, dict)
        self.assertEqual(target["quaternion_xyzw"], list(_wxyz_to_xyzw(handles["left_tcp_target_wxyz"])))
        self.assertEqual(packet["right"], {})

    def test_tcp_linear_marker_send_uses_absolute_target_with_quaternion(self):
        state = self.tcp_available_state()
        _, client, safety = self.make_safety(
            state,
            desired="simulation",
            observed="simulation",
            observed_backend="simulator",
            sim_ready=True,
            cartesian_available=True,
            enable_tcp_pose=True,
        )
        yaw_90_pose = (0.31, 0.12, 0.44, 0.0, 0.0, math.pi / 2.0)
        handles = {
            "left_tcp_target_pose": yaw_90_pose,
            "left_tcp_target_wxyz": _pose_wxyz(yaw_90_pose),
            # Marker parked at a destination (operator dragged it) -> send is allowed.
            "left_tcp_target_user_moved": True,
        }
        ok, reason = _send_tcp_linear_move_from_marker(
            safety,
            handles,
            "left",
            duration_sec=2.0,
            linear_speed_m_s=0.03,
            angular_speed_rad_s=0.2,
            orientation_mode="constant",
        )
        self.assertTrue(ok, reason)
        packet = client.sent_packets[-1]
        self.assertEqual(packet["mode"], "Hold")
        self.assertEqual(packet["left"]["mode"], "TcpLinearMove")
        self.assertEqual(packet["left"]["target_tcp_stand"]["quaternion_xyzw"], list(_wxyz_to_xyzw(handles["left_tcp_target_wxyz"])))

    def test_tcp_linear_send_blocked_when_marker_follows_current_tcp(self):
        # Regression: with no {arm}_tcp_target_user_moved flag the marker follows current
        # TCP, so the "destination" equals the current pose and the linear move is a near
        # no-op (the arm only creeps a few degrees per click). The send must be blocked
        # with a clear reason, and NO packet may go out.
        state = self.tcp_available_state()
        _, client, safety = self.make_safety(
            state,
            desired="simulation",
            observed="simulation",
            observed_backend="simulator",
            sim_ready=True,
            cartesian_available=True,
            enable_tcp_pose=True,
        )
        pose = (0.31, 0.12, 0.44, 0.0, 0.0, 0.0)
        handles = {
            # Marker pose present but NOT user-moved -> it is following current TCP.
            "left_tcp_target_pose": pose,
            "left_tcp_target_wxyz": _pose_wxyz(pose),
            "right_tcp_target_pose": pose,
            "right_tcp_target_wxyz": _pose_wxyz(pose),
        }
        before = len(client.sent_packets)
        ok, reason = _send_tcp_linear_move_from_marker(
            safety,
            handles,
            "both",
            duration_sec=2.0,
            linear_speed_m_s=0.03,
            angular_speed_rad_s=0.2,
            orientation_mode="constant",
        )
        self.assertFalse(ok)
        self.assertIn("following current TCP", reason)
        self.assertEqual(len(client.sent_packets), before, "no command may be sent when blocked")

        # Once the operator parks the marker (drag sets user_moved), the send goes through.
        handles["left_tcp_target_user_moved"] = True
        handles["right_tcp_target_user_moved"] = True
        ok, reason = _send_tcp_linear_move_from_marker(
            safety,
            handles,
            "both",
            duration_sec=2.0,
            linear_speed_m_s=0.03,
            angular_speed_rad_s=0.2,
            orientation_mode="constant",
        )
        self.assertTrue(ok, reason)
        self.assertEqual(client.sent_packets[-1]["left"]["mode"], "TcpLinearMove")

    def test_tcp_frame_defaults_to_stand_and_updates_button_colors(self):
        handles = {
            "tcp_frame_buttons": {
                _TCP_FRAME_STAND: RecordingButton(),
                _TCP_FRAME_LOCAL: RecordingButton(),
            }
        }
        self.assertEqual(_tcp_frame_mode(handles), _TCP_FRAME_STAND)
        _update_tcp_frame_buttons(handles)
        self.assertEqual(handles["tcp_frame_buttons"][_TCP_FRAME_STAND].color, "green")
        self.assertEqual(handles["tcp_frame_buttons"][_TCP_FRAME_LOCAL].color, "gray")

        handles["tcp_frame_mode"] = _TCP_FRAME_LOCAL
        _update_tcp_frame_buttons(handles)
        self.assertEqual(handles["tcp_frame_buttons"][_TCP_FRAME_STAND].color, "gray")
        self.assertEqual(handles["tcp_frame_buttons"][_TCP_FRAME_LOCAL].color, "green")

    def test_tcp_display_defaults_to_auto_and_updates_button_colors(self):
        handles = {"tcp_display_buttons": {mode: RecordingButton() for mode in _TCP_DISPLAY_MODES}}
        self.assertEqual(_tcp_display_mode(handles), "auto")
        _update_tcp_display_buttons(handles)
        self.assertEqual(handles["tcp_display_buttons"]["auto"].color, "green")
        self.assertEqual(handles["tcp_display_buttons"]["actual"].color, "gray")

        handles["tcp_display_mode"] = "reference"
        _update_tcp_display_buttons(handles)
        self.assertEqual(handles["tcp_display_buttons"]["auto"].color, "gray")
        self.assertEqual(handles["tcp_display_buttons"]["reference"].color, "green")

        handles["tcp_display_mode"] = "invalid"
        self.assertEqual(_tcp_display_mode(handles), "auto")

    def test_tcp_linear_defaults_to_both_and_slerp_with_active_button_colors(self):
        handles = {
            "tcp_linear_arm_buttons": {
                "left": RecordingButton(),
                "right": RecordingButton(),
                "both": RecordingButton(),
            },
            "tcp_linear_orientation_buttons": {
                "constant": RecordingButton(),
                "slerp": RecordingButton(),
            },
        }
        self.assertEqual(_tcp_linear_arm(handles), "both")
        self.assertEqual(_tcp_linear_orientation_mode(handles), "slerp")
        _update_tcp_linear_selection_buttons(handles)
        self.assertEqual(handles["tcp_linear_arm_buttons"]["left"].color, "gray")
        self.assertEqual(handles["tcp_linear_arm_buttons"]["right"].color, "gray")
        self.assertEqual(handles["tcp_linear_arm_buttons"]["both"].color, "green")
        self.assertEqual(handles["tcp_linear_orientation_buttons"]["constant"].color, "gray")
        self.assertEqual(handles["tcp_linear_orientation_buttons"]["slerp"].color, "green")

        handles["tcp_linear_arm"] = "left"
        handles["tcp_linear_orientation_mode"] = "constant"
        _update_tcp_linear_selection_buttons(handles)
        self.assertEqual(handles["tcp_linear_arm_buttons"]["left"].color, "green")
        self.assertEqual(handles["tcp_linear_arm_buttons"]["both"].color, "gray")
        self.assertEqual(handles["tcp_linear_orientation_buttons"]["constant"].color, "green")
        self.assertEqual(handles["tcp_linear_orientation_buttons"]["slerp"].color, "gray")

    def test_scene_update_hides_tcp_marker_when_pose_is_not_valid(self):
        state = sample_state()
        state["left"]["tcp_stand"] = {"x": 0.31, "y": 0.12, "z": 0.44, "rx": 0.0, "ry": 0.0, "rz": 0.0}
        state["left"]["has_valid_tcp_pose"] = False
        state["left"]["tcp_deferred"] = False
        store, _, _ = self.make_safety(state)
        left_tcp = RecordingSceneHandle()
        left_target = RecordingSceneHandle()
        handles = {"left_tcp": left_tcp, "left_tcp_target": left_target}
        update_scene_markers(handles, store.latest())
        self.assertIsNone(left_tcp.position)
        self.assertFalse(left_tcp.visible)
        self.assertFalse(left_target.visible)

    def test_fk_status_distinguishes_available_deferred_invalid_and_stale(self):
        available = sample_state()
        for arm in ("left", "right"):
            available[arm]["tcp_stand"] = {"x": 0.31, "y": 0.12, "z": 0.44, "rx": 0.0, "ry": 0.0, "rz": 0.0}
            available[arm]["has_valid_tcp_pose"] = True
            available[arm]["tcp_deferred"] = False
        store, _, _ = self.make_safety(available)
        self.assertEqual(_format_fk_status(store.latest(), stale=False), "FK: available")
        self.assertEqual(_format_fk_status(store.latest(), stale=True), "State stream stale")

        deferred_store, _, _ = self.make_safety(sample_state())
        self.assertEqual(_format_fk_status(deferred_store.latest(), stale=False), "FK: deferred")

        invalid = sample_state()
        invalid["left"]["tcp_deferred"] = False
        invalid["right"]["tcp_deferred"] = False
        invalid["left"]["has_valid_joint_state"] = False
        invalid["left"]["q_actual_deg"] = [0, 0, 0]
        invalid_store, _, _ = self.make_safety(invalid)
        self.assertIn("left invalid joint state", _format_fk_status(invalid_store.latest(), stale=False))
        self.assertIn("right invalid TCP pose", _format_fk_status(invalid_store.latest(), stale=False))

    def test_tcp_tracking_status_reports_selection_and_controller_simulation_fields(self):
        state = self.tcp_available_state()
        for arm in ("left", "right"):
            state[arm]["tcp_ref_stand"] = {"x": 0.41, "y": 0.21, "z": 0.51, "rx": 0.0, "ry": 0.0, "rz": 0.0}
            state[arm]["tcp_ref_valid"] = True
            state[arm]["tcp_tracking_source"] = "tcp_ref_stand"
            state[arm]["tcp_tracking_source_recommendation"] = "tcp_ref_stand"
            state[arm]["controller_simulation_mode"] = {
                "recommended_tracking_pose": "tcp_ref_stand",
                "physical_motion_expected": False,
            }
            state[arm]["physical_motion_expected"] = False
        state["left"]["controller_simulation_diagnostic_override_active"] = True
        store, _, _ = self.make_safety(state)
        status = _format_tcp_tracking_status(store.latest(), stale=False, display_mode="auto")
        self.assertIn("TCP tracking:", status)
        self.assertIn("left display=auto selected_source=tcp_ref_stand", status)
        self.assertIn("actual_valid=True", status)
        self.assertIn("ref_valid=True", status)
        self.assertIn("tcp_tracking_source=tcp_ref_stand", status)
        self.assertIn("tcp_tracking_source_recommendation=tcp_ref_stand", status)
        self.assertIn("physical_motion_expected=False", status)
        self.assertIn("diagnostics_override=True", status)
        self.assertEqual(_format_tcp_tracking_status(store.latest(), stale=True), "State stream stale")
        self.assertEqual(_format_tcp_tracking_status(None, stale=True), "TCP tracking: no state")

    def test_circle_overlay_status_formats_missing_live_and_stale_fields(self):
        self.assertEqual(
            _format_circle_overlay_status(None, stale=True, enabled=False),
            "Circle overlay: disabled",
        )
        self.assertEqual(
            _format_circle_overlay_status(None, stale=True, enabled=True),
            "Circle overlay: no packets",
        )
        overlay = CircleOverlaySnapshot.parse(sample_circle_overlay(), received_monotonic=100.0)
        self.assertIsNotNone(overlay)
        live = _format_circle_overlay_status(overlay, stale=False, enabled=True)
        self.assertIn("Circle overlay: live", live)
        self.assertIn("run_id=run-1", live)
        self.assertIn("tracking_source=tcp_ref_stand", live)
        self.assertIn("error=1.0 mm", live)
        self.assertIn("rms=2.0 mm", live)
        self.assertIn("p95=3.0 mm", live)
        self.assertIn("latency=12.3 ms", live)
        self.assertIn("physical_motion_expected=False", live)
        stale = _format_circle_overlay_status(overlay, stale=True, enabled=True)
        self.assertIn("Circle overlay: stale", stale)

    def test_cartesian_solve_status_displays_ik_error_and_timing(self):
        state = sample_state()
        state["left"]["cartesian_solve"] = {
            "attempted": True,
            "status": "ok",
            "position_error_m": 0.0012,
            "orientation_error_rad": 0.0025,
            "ik_iterations": 7,
            "ik_duration_us": 125.0,
            "ik_timed_out": False,
            "path_active": True,
            "path_s": 0.25,
            "path_line_deviation_m": 0.0004,
            "path_orientation_error_rad": 0.001,
            "path_done": False,
        }
        state["right"]["cartesian_solve"] = {
            "attempted": True,
            "status": "failed",
            "position_error_m": 0.03,
            "orientation_error_rad": 0.04,
            "ik_iterations": 50,
            "ik_duration_us": 5000.0,
            "ik_timed_out": True,
        }
        store, _, _ = self.make_safety(state)
        self.assertIsNotNone(store.latest().left.cartesian_solve)
        self.assertEqual(store.latest().left.cartesian_solve.orientation_error_rad, 0.0025)
        self.assertEqual(store.latest().left.cartesian_solve.path_line_deviation_m, 0.0004)
        self.assertEqual(store.latest().left.cartesian_solve.path_s, 0.25)
        status = _format_cartesian_solve_status(store.latest(), stale=False)
        self.assertIn("left ok", status)
        self.assertIn("pos_err=0.0012 m", status)
        self.assertIn("ori_err=0.0025 rad", status)
        self.assertIn("iter=7", status)
        self.assertIn("dur=125 us", status)
        self.assertIn("timed_out=False", status)
        self.assertIn("path_active=True", status)
        self.assertIn("path_s=0.250", status)
        self.assertIn("line_dev=0.0004 m", status)
        self.assertIn("path_ori_err=0.001 rad", status)
        self.assertIn("path_done=False", status)
        self.assertIn("right failed", status)
        self.assertIn("timed_out=True", status)
        self.assertEqual(_format_cartesian_solve_status(store.latest(), stale=True), "State stream stale")

    def test_cartesian_solve_parser_accepts_missing_fields(self):
        store, _, _ = self.make_safety(sample_state())
        latest = store.latest()
        self.assertIsNone(latest.left.cartesian_solve)
        self.assertIn("left=unavailable", _format_cartesian_solve_status(latest, stale=False))

    def test_visual_disabled_state_matches_safety_blocks(self):
        # Real/sim gating retired: real mode controls follow the same rules as
        # any other mode (FK-less sample_state still disables TCP controls).
        _, _, real_safety = self.make_safety(sample_state(), desired="real", observed="real")
        real_states = real_safety.control_disabled_states()
        self.assertFalse(real_states["jog"])
        self.assertTrue(real_states["tcp_linear"])
        self.assertFalse(real_states["lifecycle:ArmMotion"])
        self.assertFalse(real_states["lifecycle:Hold"])

        _, _, stale_safety = self.make_safety(None)
        stale_states = stale_safety.control_disabled_states()
        self.assertTrue(stale_states["jog"])
        self.assertTrue(stale_states["lifecycle:ArmMotion"])

        _, _, mock_safety = self.make_safety(sample_state())
        mock_states = mock_safety.control_disabled_states()
        # Joint-space controls follow the joint gate; Cartesian controls follow the
        # FK/Cartesian gate. FK-less sample_state -> TCP controls disabled.
        self.assertFalse(mock_states["jog"])
        self.assertFalse(mock_states["lifecycle:ArmMotion"])
        self.assertNotIn("tcp_jog", mock_states)
        self.assertTrue(mock_states["tcp_pose"])
        self.assertTrue(mock_states["tcp_linear"])

        # Simulation mode is no longer env-gated: a valid state enables joint
        # controls, and FK + an open server Cartesian gate enables TCP controls.
        _, _, sim_safety = self.make_safety(
            self.tcp_available_state(),
            desired="simulation",
            observed="simulation",
            cartesian_available=True,
        )
        sim_states = sim_safety.control_disabled_states()
        self.assertFalse(sim_states["jog"])
        self.assertFalse(sim_states["lifecycle:ArmMotion"])
        self.assertFalse(sim_states["tcp_pose"])
        self.assertFalse(sim_states["tcp_linear"])

        # A latched fault re-disables every motion control (lifecycle reset stays
        # live so the operator can clear it).
        _, _, fault_safety = self.make_safety(sample_state(fault_latched=True, fault_reason="self_collision"))
        fault_states = fault_safety.control_disabled_states()
        self.assertTrue(fault_states["jog"])
        self.assertTrue(fault_states["tcp_pose"])
        self.assertTrue(fault_states["tcp_linear"])
        self.assertTrue(fault_states["lifecycle:ArmMotion"])
        self.assertFalse(fault_states["lifecycle:ResetFault"])

    def test_lifecycle_packets_match_existing_udp_protocol(self):
        _, client, safety = self.make_safety(sample_state())
        ok, reason = safety.send_lifecycle("EmergencyStop")
        self.assertTrue(ok, reason)
        self.assertEqual(client.sent_packets[-1]["mode"], "EmergencyStop")
        self.assertEqual(client.sent_packets[-1]["left"], {})
        self.assertEqual(client.sent_packets[-1]["right"], {})

    def test_control_disabled_reasons_align_with_states(self):
        # Every disabled key carries a matching reason and vice-versa.
        _, _, safety = self.make_safety(sample_state())  # FK-less: TCP gated, joint open
        states = safety.control_disabled_states()
        reasons = safety.control_disabled_reasons()
        for key, reason in reasons.items():
            self.assertEqual(states[key], reason is not None, key)
        self.assertIsNone(reasons["jog"])
        self.assertIn("FK/TCP pose unavailable", reasons["tcp_pose"])
        self.assertIn("FK/TCP pose unavailable", reasons["tcp_linear"])

    def test_reflect_gate_reason_preserves_click_feedback(self):
        class _Text:
            def __init__(self, value):
                self.value = value

        # Blocked -> shows DISABLED note; cleared -> back to idle.
        handle = _Text("idle")
        _reflect_gate_reason(handle, "joint state invalid")
        self.assertEqual(handle.value, "DISABLED: joint state invalid")
        _reflect_gate_reason(handle, None)
        self.assertEqual(handle.value, "idle")
        # A fresh click result is never clobbered when the gate is open.
        handle.value = "OK: sent JointTarget left"
        _reflect_gate_reason(handle, None)
        self.assertEqual(handle.value, "OK: sent JointTarget left")
        _reflect_gate_reason(None, "ignored")  # no handle -> no crash

    def test_lease_owner_status_flags_foreign_and_self_owner(self):
        class _Text:
            def __init__(self):
                self.value = ""

        store = StateStore(stale_after_sec=5.0)
        foreign = self.pgmode_spacemouse_state()  # command_source owned by policy_runner
        self.assertTrue(store.update_from_json_bytes(json.dumps(foreign).encode(), received_monotonic=time.monotonic()))
        handles = {"lease_owner_status": _Text()}
        _update_lease_owner(handles, store.latest(), "rb_gui", held=False)
        self.assertIn("policy_runner", handles["lease_owner_status"].value)
        self.assertIn("stop or release", handles["lease_owner_status"].value)

        owned = self.pgmode_spacemouse_state()
        owned["command_source"]["source_id"] = "rb_gui"
        owned["command_source"]["active_source_id"] = "rb_gui"
        store2 = StateStore(stale_after_sec=5.0)
        self.assertTrue(store2.update_from_json_bytes(json.dumps(owned).encode(), received_monotonic=time.monotonic()))
        _update_lease_owner(handles, store2.latest(), "rb_gui", held=True)
        self.assertIn("held by you", handles["lease_owner_status"].value)
        self.assertIn("Take control ON", handles["lease_owner_status"].value)

        _update_lease_owner(handles, None, "rb_gui", held=False)
        self.assertEqual(handles["lease_owner_status"].value, "no state stream")


class FloorConstraintGuiTest(unittest.TestCase):
    @staticmethod
    def _floor_block(**overrides):
        block = {
            "enabled": True,
            "monitor_only": False,
            "z_min_m": 0.010,
            "config_z_min_m": 0.010,
            "runtime_min_z_m": 0.0,
            "runtime_max_z_m": 0.5,
            "left": {"checked": True, "violated": False, "tcp_z_m": 0.133},
            "right": {"checked": True, "violated": False, "tcp_z_m": 0.108},
            "clamp_count": 0,
            "last_set_reject_reason": None,
        }
        block.update(overrides)
        return block

    def test_state_snapshot_parses_floor_constraint(self):
        from rb_servo_gui.models import StateSnapshot

        snapshot = StateSnapshot.parse(sample_state(floor_constraint=self._floor_block()))
        self.assertIsNotNone(snapshot)
        self.assertTrue(snapshot.floor_constraint["enabled"])
        self.assertAlmostEqual(snapshot.floor_constraint["z_min_m"], 0.010)

        without = StateSnapshot.parse(sample_state())
        self.assertIsNotNone(without)
        self.assertIsNone(without.floor_constraint)

    def test_format_floor_constraint_status(self):
        from rb_servo_gui.models import StateSnapshot

        self.assertEqual(_format_floor_constraint_status(None, stale=False), "floor: no state")

        ok_state = StateSnapshot.parse(sample_state(floor_constraint=self._floor_block()))
        text = _format_floor_constraint_status(ok_state, stale=False)
        self.assertIn("floor: ON z=10.0mm", text)
        self.assertIn("L:123mm", text)
        self.assertIn("R:98mm", text)

        violated_state = StateSnapshot.parse(sample_state(floor_constraint=self._floor_block(
            left={"checked": True, "violated": True, "tcp_z_m": 0.004},
        )))
        self.assertIn("VIOLATED(left)", _format_floor_constraint_status(violated_state, stale=False))

        disabled_state = StateSnapshot.parse(sample_state(floor_constraint=self._floor_block(enabled=False)))
        self.assertEqual(_format_floor_constraint_status(disabled_state, stale=False), "floor: disabled")

        no_block = StateSnapshot.parse(sample_state())
        self.assertEqual(_format_floor_constraint_status(no_block, stale=False), "floor: disabled")

    def test_build_set_safety_floor_z_packet(self):
        client = CommandClient(host="127.0.0.1", port=0, source_id="rb_gui_test")
        packet = client.build_set_safety_floor_z(0.012)
        self.assertEqual(packet["mode"], "SetSafetyFloorZ")
        self.assertAlmostEqual(packet["floor_z_m"], 0.012)
        self.assertEqual(packet["left"], {})
        self.assertEqual(packet["right"], {})
        self.assertEqual(packet["source_id"], "rb_gui_test")
        with self.assertRaises(ValueError):
            client.build_set_safety_floor_z(float("nan"))

    def test_build_set_safety_floor_enabled_packet(self):
        client = CommandClient(host="127.0.0.1", port=0, source_id="rb_gui_test")
        on = client.build_set_safety_floor_enabled(True)
        self.assertEqual(on["mode"], "SetSafetyFloorEnabled")
        self.assertIs(on["floor_enabled"], True)
        self.assertEqual(on["left"], {})
        self.assertEqual(on["right"], {})
        off = client.build_set_safety_floor_enabled(False)
        self.assertIs(off["floor_enabled"], False)
        # Leaseless: not bracketed with Acquire/Release.
        self.assertNotIn("SetSafetyFloorEnabled", CommandClient._LEASED_MODES)

    def test_build_set_user_safety_floor_plane_packet(self):
        client = CommandClient(host="127.0.0.1", port=0, source_id="rb_gui_test")
        # A non-unit normal is normalized client-side; enable defaults True.
        packet = client.build_set_user_safety_floor_plane(
            (0.0, -0.28, 0.01), (0.0, 0.0, 2.0), margin_m=0.002)
        self.assertEqual(packet["mode"], "SetUserSafetyFloorPlane")
        self.assertEqual(packet["user_floor_point_m"], [0.0, -0.28, 0.01])
        # Normalized to a unit normal.
        self.assertAlmostEqual(
            sum(v * v for v in packet["user_floor_normal"]) ** 0.5, 1.0, places=9)
        self.assertAlmostEqual(packet["user_floor_normal"][2], 1.0)
        self.assertAlmostEqual(packet["user_floor_margin_m"], 0.002)
        self.assertTrue(packet["user_floor_enable"])
        self.assertEqual(packet["left"], {})
        self.assertEqual(packet["right"], {})
        # Disable carries enable=False.
        disable = client.build_set_user_safety_floor_plane(
            (0.0, 0.0, 0.0), (0.0, 0.0, 1.0), enable=False)
        self.assertFalse(disable["user_floor_enable"])
        # Degenerate / non-finite inputs are rejected.
        with self.assertRaises(ValueError):
            client.build_set_user_safety_floor_plane((0, 0, 0), (0, 0, 0))
        with self.assertRaises(ValueError):
            client.build_set_user_safety_floor_plane((0, 0, float("nan")), (0, 0, 1))

    def test_fit_plane_horizontal_and_tilted(self):
        from rb_servo_gui.plane_fit import fit_plane, tilt_deg

        # Horizontal plane at z=0.1 -> normal +z, tilt ~0.
        point, normal = fit_plane([[0, 0, 0.1], [1, 0, 0.1], [0, 1, 0.1], [1, 1, 0.1]])
        self.assertAlmostEqual(normal[2], 1.0, places=6)
        self.assertLess(tilt_deg(normal), 1e-3)
        self.assertAlmostEqual(point[2], 0.1, places=6)
        # A plane tilted 30deg about x: points satisfy z = tan(30)*y.
        import math as _m
        t = _m.tan(_m.radians(30.0))
        pts = [[0, 0, 0], [1, 0, 0], [0, 1, t], [1, 1, t]]
        _p, n = fit_plane(pts)
        self.assertGreater(n[2], 0.0)  # oriented upward
        self.assertAlmostEqual(tilt_deg(n), 30.0, places=4)
        # Colinear-in-XY points CONVERGE (min-norm): tilt only along the captured
        # direction, zero perpendicular. Points along +x with rising z -> tilt about y.
        s = _m.tan(_m.radians(10.0))
        _pc, nc = fit_plane([[0, 0, 0], [1, 0, s], [2, 0, 2 * s]])
        self.assertGreater(nc[2], 0.0)
        self.assertAlmostEqual(tilt_deg(nc), 10.0, places=4)
        # Flat colinear points -> horizontal plane (no raise).
        _ph, nh = fit_plane([[0, 0, 0], [1, 0, 0], [2, 0, 0]])
        self.assertLess(tilt_deg(nh), 1e-6)
        # All points at the same (x, y) is rejected (no spatial spread).
        with self.assertRaises(ValueError):
            fit_plane([[0.1, -0.2, 0.0], [0.1, -0.2, 0.01], [0.1, -0.2, 0.02]])
        # Fewer than 3 points is rejected.
        with self.assertRaises(ValueError):
            fit_plane([[0, 0, 0], [1, 0, 0]])

    def test_user_floor_json_round_trip(self):
        import os
        import tempfile
        from rb_servo_gui import app as gui_app

        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "user_floor.json")
            os.environ["RB_GUI_USER_FLOOR_PATH"] = path
            try:
                state = {
                    "points": [
                        {"arm": "left", "p": [0.1, -0.2, 0.0]},
                        {"arm": "right", "p": [-0.1, -0.3, 0.005]},
                        {"arm": "left", "p": [0.0, -0.25, 0.002]},
                    ],
                    "plane": {"point": [0.0, -0.25, 0.002], "normal": [0.0, 0.1, 0.995]},
                    "margin_mm": 1.5,
                    "enabled": True,
                }
                ok, _ = gui_app._save_user_floor(state)
                self.assertTrue(ok)
                loaded = gui_app._load_user_floor()
                self.assertEqual(len(loaded["points"]), 3)
                self.assertEqual(loaded["points"][0]["arm"], "left")
                self.assertTrue(loaded["enabled"])
                self.assertAlmostEqual(loaded["margin_mm"], 1.5)
                self.assertIsNotNone(loaded["plane"])
            finally:
                os.environ.pop("RB_GUI_USER_FLOOR_PATH", None)

    def test_state_snapshot_parses_user_floor_constraint(self):
        from rb_servo_gui.models import StateSnapshot

        uf = {
            "enabled": True,
            "monitor_only": False,
            "point_m": [0.0, -0.25, 0.0],
            "normal": [0.0, 0.0, 1.0],
            "margin_m": 0.0,
            "left": {"checked": True, "violated": False, "signed_dist_m": 0.12,
                     "lowest_point": "tcp", "lowest_point_m": [0.1, -0.2, 0.12]},
            "right": {"checked": True, "violated": True, "signed_dist_m": -0.01,
                      "lowest_point": "gripper_tip_a_yp", "lowest_point_m": [-0.1, -0.2, -0.01]},
            "clamp_count": 2,
            "last_set_reject_reason": None,
        }
        snap = StateSnapshot.parse(sample_state(user_floor_constraint=uf))
        self.assertIsNotNone(snap)
        self.assertTrue(snap.user_floor_constraint["enabled"])
        text = _format_user_floor_constraint_status(snap, stale=False)
        self.assertIn("VIOLATED(right)", text)
        # Disabled / absent -> "off" / "no state".
        off = StateSnapshot.parse(sample_state(user_floor_constraint={"enabled": False}))
        self.assertEqual(_format_user_floor_constraint_status(off, stale=False), "user floor: off")
        absent = StateSnapshot.parse(sample_state())
        self.assertIsNone(absent.user_floor_constraint)
        self.assertEqual(
            _format_user_floor_constraint_status(absent, stale=False), "user floor: off")

    def test_user_floor_display_points_gated_by_toggle_default_off(self):
        class _Toggle:
            def __init__(self, v):
                self.value = v

        pts = [{"p": [0.0, 0.0, 0.0], "arm": "left"}]
        handles = {"user_floor": {"points": pts}}
        # No toggle handle -> hidden (default off): nothing rendered.
        self.assertEqual(_user_floor_display_points(handles), [])
        # Toggle present but OFF -> still hidden.
        handles["user_floor_show_points_toggle"] = _Toggle(False)
        self.assertEqual(_user_floor_display_points(handles), [])
        # Toggle ON -> the captured points are returned for rendering.
        handles["user_floor_show_points_toggle"].value = True
        self.assertEqual(_user_floor_display_points(handles), pts)

    def test_build_freedrive_per_arm_packet(self):
        client = CommandClient(host="127.0.0.1", port=0, source_id="rb_gui_test")
        # Left only: right is an untouched Hold, left carries the Freedrive payload.
        left_on = client.build_freedrive(left=True)
        self.assertEqual(left_on["mode"], "Freedrive")
        self.assertEqual(left_on["left"], {"mode": "Freedrive", "freedrive_on": True})
        self.assertEqual(left_on["right"], {"mode": "Hold"})
        # Right off carries the explicit off payload.
        right_off = client.build_freedrive(right=False)
        self.assertEqual(right_off["right"], {"mode": "Freedrive", "freedrive_on": False})
        self.assertEqual(right_off["left"], {"mode": "Hold"})
        # Both arms in one packet.
        both = client.build_freedrive(left=False, right=False)
        self.assertEqual(both["left"], {"mode": "Freedrive", "freedrive_on": False})
        self.assertEqual(both["right"], {"mode": "Freedrive", "freedrive_on": False})
        # Both arms ON in one packet (양팔 교시 ON).
        both_on = client.build_freedrive(left=True, right=True)
        self.assertEqual(both_on["left"], {"mode": "Freedrive", "freedrive_on": True})
        self.assertEqual(both_on["right"], {"mode": "Freedrive", "freedrive_on": True})
        # Freedrive is a leased mode so send() brackets it with Acquire/Release.
        self.assertIn("Freedrive", CommandClient._LEASED_MODES)
        with self.assertRaises(ValueError):
            client.build_freedrive()

    def test_persist_floor_z_rewrites_only_z_min_m_and_keeps_comments(self):
        import tempfile
        from pathlib import Path

        from rb_servo_gui.safety import persist_floor_z_to_config

        content = (
            "safety:\n"
            "  max_tracking_error_deg: 30.0\n"
            "  # 안전 평면: z=0은 stand 원점\n"
            "  floor_constraint:\n"
            "    enable: true\n"
            "    z_min_m: 0.010   # startup default\n"
            "    runtime_min_z_m: 0.0\n"
            "    runtime_max_z_m: 0.5\n"
            "network:\n"
            "  z_min_m: 99.0   # decoy outside the floor block\n"
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "stack.yaml"
            path.write_text(content, encoding="utf-8")
            ok, message = persist_floor_z_to_config(path, 0.05)
            self.assertTrue(ok, message)
            updated = path.read_text(encoding="utf-8")
            self.assertIn("    z_min_m: 0.0500   # startup default\n", updated)
            self.assertIn("# 안전 평면: z=0은 stand 원점", updated)
            self.assertIn("z_min_m: 99.0   # decoy outside the floor block", updated)
            self.assertIn("runtime_max_z_m: 0.5", updated)

    def test_persist_floor_z_reports_missing_file_and_missing_key(self):
        import tempfile
        from pathlib import Path

        from rb_servo_gui.safety import persist_floor_z_to_config

        ok, message = persist_floor_z_to_config("/nonexistent/stack.yaml", 0.05)
        self.assertFalse(ok)
        self.assertIn("yaml unchanged", message)

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "stack.yaml"
            path.write_text("safety:\n  floor_constraint:\n    enable: true\n", encoding="utf-8")
            ok, message = persist_floor_z_to_config(path, 0.05)
            self.assertFalse(ok)
            self.assertIn("z_min_m not found", message)

    def test_update_floor_plane_no_crash_and_state(self):
        class FakePlane:
            def __init__(self):
                self.position = (0.0, 0.0, 0.0)
                self.color = None
                self.visible = True

        plane = FakePlane()
        handles = {"floor_plane": plane}
        # Disabled / missing block hides the plane.
        update_floor_plane(handles, None)
        self.assertFalse(plane.visible)
        update_floor_plane(handles, {"enabled": False})
        self.assertFalse(plane.visible)
        # Enabled moves it to z and shows it.
        update_floor_plane(handles, self._floor_block(z_min_m=0.05))
        self.assertTrue(plane.visible)
        self.assertAlmostEqual(plane.position[2], 0.05)
        # Violation recolors.
        blue = plane.color
        update_floor_plane(handles, self._floor_block(
            right={"checked": True, "violated": True, "tcp_z_m": 0.001},
        ))
        self.assertNotEqual(plane.color, blue)
        # Missing handle is a no-op.
        update_floor_plane({}, self._floor_block())

    def test_update_floor_plane_outline_tracks_plane(self):
        # Emphasised edges (12 segments) + corner vertices (8 points) follow the
        # applied floor plane at height z and hide/recolor with it.
        class FakePlane:
            def __init__(self):
                self.position = (0.0, 0.0, 0.0)
                self.color = None
                self.visible = True

        edges = RecordingSceneHandle()
        verts = RecordingSceneHandle()
        handles = {"floor_plane": FakePlane(), "floor_plane_edges": edges, "floor_plane_verts": verts}
        update_floor_plane(handles, self._floor_block(z_min_m=0.05))
        self.assertTrue(edges.visible and verts.visible)
        self.assertEqual(np.asarray(edges.points).shape, (12, 2, 3))
        self.assertEqual(np.asarray(verts.points).shape, (8, 3))
        # Outline sits at the plane height z (corners within the 2 mm slab thickness).
        self.assertTrue(np.allclose(np.asarray(verts.points)[:, 2], 0.05, atol=2e-3))
        normal_edge_color = np.asarray(edges.colors).copy()
        # Violation recolors the outline.
        update_floor_plane(handles, self._floor_block(
            z_min_m=0.05, right={"checked": True, "violated": True, "tcp_z_m": 0.001},
        ))
        self.assertFalse(np.array_equal(np.asarray(edges.colors), normal_edge_color))
        # Disabled hides the outline too.
        update_floor_plane(handles, {"enabled": False})
        self.assertFalse(edges.visible or verts.visible)

    def test_update_user_floor_outline_tracks_tilted_plane(self):
        # The user floor edges + corner vertices mirror the stand floor, but the
        # plane is TILTED: the outline nodes carry local box geometry and take the
        # same position + wxyz as the fill, recolor red on violation, and hide when
        # the constraint is disabled.
        class FakePlane:
            def __init__(self):
                self.position = None
                self.wxyz = None
                self.color = None
                self.visible = True

        def _uf(*, violated=False, enabled=True):
            return {
                "enabled": enabled,
                "point_m": [0.1, -0.25, 0.2],
                "normal": [0.0, 0.0, 1.0],
                "left": {"checked": True, "violated": False},
                "right": {"checked": True, "violated": violated},
            }

        plane = FakePlane()
        edges = RecordingSceneHandle()
        verts = RecordingSceneHandle()
        handles = {
            "user_floor_plane": plane,
            "user_floor_edges": edges,
            "user_floor_verts": verts,
        }
        update_user_floor_plane(handles, _uf())
        self.assertTrue(edges.visible and verts.visible)
        # Outline takes the fill's tilt transform (position + wxyz). Geometry is
        # local box outline fixed at creation, so update only moves/orients it.
        self.assertEqual(tuple(edges.position), (0.1, -0.25, 0.2))
        self.assertEqual(tuple(verts.position), (0.1, -0.25, 0.2))
        self.assertEqual(tuple(edges.wxyz), tuple(plane.wxyz))
        compliant_color = np.asarray(edges.colors).copy()
        # Violation recolors the outline red.
        update_user_floor_plane(handles, _uf(violated=True))
        self.assertFalse(np.array_equal(np.asarray(edges.colors), compliant_color))
        # Disabled hides plane + outline.
        update_user_floor_plane(handles, _uf(enabled=False))
        self.assertFalse(plane.visible)
        self.assertFalse(edges.visible or verts.visible)
        # None (stale/disconnect) also hides the outline.
        update_user_floor_plane(handles, None)
        self.assertFalse(edges.visible or verts.visible)

    def test_update_floor_plane_preview(self):
        class FakePlane:
            def __init__(self):
                self.position = (0.0, 0.0, 0.0)
                self.visible = True

        plane = FakePlane()
        handles = {"floor_plane_preview": plane}
        # Pending value shows the preview at the slider z.
        update_floor_plane_preview(handles, 0.123)
        self.assertTrue(plane.visible)
        self.assertAlmostEqual(plane.position[2], 0.123)
        # None (slider matches applied / constraint disabled) hides it.
        update_floor_plane_preview(handles, None)
        self.assertFalse(plane.visible)
        # Non-finite values hide instead of moving the plane.
        update_floor_plane_preview(handles, float("nan"))
        self.assertFalse(plane.visible)
        # Missing handle is a no-op.
        update_floor_plane_preview({}, 0.05)

    def test_update_floor_panel_syncs_slider_to_applied_z_once(self):
        from rb_servo_gui.models import StateSnapshot

        class FakeSlider:
            def __init__(self):
                self.min = 0.0
                self.max = 500.0
                self.value = 10.0

        class FakeText:
            def __init__(self):
                self.value = ""

        slider = FakeSlider()
        handles = {"floor_slider": slider, "floor_applied": FakeText()}
        # Current applied floor is 80 mm (z_min_m=0.080), not the hardcoded 10.
        state = StateSnapshot.parse(sample_state(floor_constraint=self._floor_block(z_min_m=0.080)))
        _update_floor_panel(handles, state)
        # Slider comes up at the server-applied value, and the one-time guard is set.
        self.assertAlmostEqual(slider.value, 80.0)
        self.assertTrue(handles.get("floor_slider_synced"))
        # A later operator edit must not be clobbered by subsequent state updates.
        slider.value = 123.0
        next_state = StateSnapshot.parse(sample_state(floor_constraint=self._floor_block(z_min_m=0.080)))
        _update_floor_panel(handles, next_state)
        self.assertAlmostEqual(slider.value, 123.0)

    def test_update_floor_panel_clamps_sync_to_slider_bounds(self):
        from rb_servo_gui.models import StateSnapshot

        class FakeSlider:
            def __init__(self):
                self.min = 0.0
                self.max = 500.0
                self.value = 10.0

        slider = FakeSlider()
        handles = {"floor_slider": slider}
        # Applied z above the runtime max -> bounds widen to 300mm and value clamps to it.
        state = StateSnapshot.parse(sample_state(floor_constraint=self._floor_block(
            z_min_m=0.450, runtime_min_z_m=0.0, runtime_max_z_m=0.300,
        )))
        _update_floor_panel(handles, state)
        self.assertAlmostEqual(slider.max, 300.0)
        self.assertAlmostEqual(slider.value, 300.0)

    def test_gui_settings_round_trip(self):
        import os
        import tempfile
        from rb_servo_gui import app as gui_app

        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "settings.json")
            os.environ["RB_GUI_SETTINGS_PATH"] = path
            try:
                self.assertEqual(gui_app._load_gui_settings(), {})
                gui_app._update_gui_setting("stand_floor_enforce", False)
                self.assertEqual(
                    gui_app._load_gui_settings().get("stand_floor_enforce"), False
                )
                # Read-modify-write preserves other keys.
                gui_app._update_gui_setting("other", 1)
                loaded = gui_app._load_gui_settings()
                self.assertEqual(loaded.get("stand_floor_enforce"), False)
                self.assertEqual(loaded.get("other"), 1)
            finally:
                os.environ.pop("RB_GUI_SETTINGS_PATH", None)

    def test_update_floor_panel_keeps_saved_enforce_preference(self):
        from rb_servo_gui.models import StateSnapshot

        class FakeCheckbox:
            def __init__(self, value):
                self.value = value

        # Operator previously turned enforce OFF; server config still reports it ON.
        toggle = FakeCheckbox(False)
        handles = {
            "floor_enforce_toggle": toggle,
            "stand_floor_enforce_pref": False,
        }
        state = StateSnapshot.parse(
            sample_state(floor_constraint=self._floor_block(enabled=True))
        )
        _update_floor_panel(handles, state)
        # Saved preference wins: the checkbox is NOT overwritten from telemetry.
        self.assertFalse(toggle.value)
        self.assertTrue(handles.get("floor_enforce_synced"))

    def test_update_floor_panel_syncs_enforce_when_no_preference(self):
        from rb_servo_gui.models import StateSnapshot

        class FakeCheckbox:
            def __init__(self, value):
                self.value = value

        # No saved preference -> fall back to mirroring the server's reported state.
        toggle = FakeCheckbox(True)
        handles = {
            "floor_enforce_toggle": toggle,
            "stand_floor_enforce_pref": None,
        }
        state = StateSnapshot.parse(
            sample_state(floor_constraint=self._floor_block(enabled=False))
        )
        _update_floor_panel(handles, state)
        self.assertFalse(toggle.value)
        self.assertTrue(handles.get("floor_enforce_synced"))


class RoiBoxGuiTest(unittest.TestCase):
    @staticmethod
    def _roi_block(**overrides):
        block = {
            "enabled": True,
            "monitor_only": False,
            "min_m": [-0.5, -1.0, 0.0],
            "max_m": [0.5, 0.0, 1.0],
            "runtime_min_m": [-1.0, -1.5, -0.2],
            "runtime_max_m": [1.0, 0.5, 1.5],
            "left": {"checked": True, "violated": False, "min_margin_m": 0.12,
                     "closest_face": "x_max"},
            "right": {"checked": True, "violated": False, "min_margin_m": 0.08,
                      "closest_face": "z_min"},
            "clamp_count": 0,
            "last_set_reject_reason": None,
        }
        block.update(overrides)
        return block

    def test_state_snapshot_parses_roi_box(self):
        from rb_servo_gui.models import StateSnapshot

        snapshot = StateSnapshot.parse(sample_state(roi_box=self._roi_block()))
        self.assertIsNotNone(snapshot)
        self.assertTrue(snapshot.roi_box["enabled"])
        self.assertEqual(snapshot.roi_box["max_m"], [0.5, 0.0, 1.0])

        without = StateSnapshot.parse(sample_state())
        self.assertIsNotNone(without)
        self.assertIsNone(without.roi_box)

    def test_format_roi_box_status(self):
        from rb_servo_gui.models import StateSnapshot

        self.assertEqual(_format_roi_box_status(None, stale=False), "roi: no state")

        ok_state = StateSnapshot.parse(sample_state(roi_box=self._roi_block()))
        text = _format_roi_box_status(ok_state, stale=False)
        self.assertIn("roi: ON", text)
        self.assertIn("L:120mm@x_max", text)
        self.assertIn("R:80mm@z_min", text)

        violated = StateSnapshot.parse(sample_state(roi_box=self._roi_block(
            left={"checked": True, "violated": True, "min_margin_m": -0.01,
                  "closest_face": "x_max"},
        )))
        self.assertIn("OUTSIDE(left)", _format_roi_box_status(violated, stale=False))

        disabled = StateSnapshot.parse(sample_state(roi_box=self._roi_block(enabled=False)))
        self.assertEqual(_format_roi_box_status(disabled, stale=False), "roi: disabled")

        no_block = StateSnapshot.parse(sample_state())
        self.assertEqual(_format_roi_box_status(no_block, stale=False), "roi: disabled")

    def test_build_set_safety_roi_bounds_packet(self):
        client = CommandClient(host="127.0.0.1", port=0, source_id="rb_gui_test")
        packet = client.build_set_safety_roi_bounds((-0.5, -1.0, 0.0), (0.5, 0.0, 1.0))
        self.assertEqual(packet["mode"], "SetSafetyRoiBounds")
        self.assertEqual(packet["roi_min_m"], [-0.5, -1.0, 0.0])
        self.assertEqual(packet["roi_max_m"], [0.5, 0.0, 1.0])
        self.assertEqual(packet["left"], {})
        self.assertEqual(packet["right"], {})
        self.assertEqual(packet["source_id"], "rb_gui_test")
        # Leaseless (not a leased mode) -> send() must not bracket it.
        self.assertNotIn("SetSafetyRoiBounds", CommandClient._LEASED_MODES)
        with self.assertRaises(ValueError):
            client.build_set_safety_roi_bounds((0.0, 0.0, float("nan")), (1.0, 1.0, 1.0))
        with self.assertRaises(ValueError):  # min > max on an axis
            client.build_set_safety_roi_bounds((0.6, 0.0, 0.0), (0.5, 1.0, 1.0))

    def test_update_roi_box_resizes_recolors_and_hides(self):
        # The ROI region is an OUTLINE only (no filled face): 12 edge segments +
        # 8 corner vertices that track the bounds, recolor red on violation, and
        # hide when bounds are absent/invalid.
        edges = RecordingSceneHandle()
        verts = RecordingSceneHandle()
        handles = {"roi_box_edges": edges, "roi_box_verts": verts}
        update_roi_box(handles, None)
        self.assertFalse(edges.visible or verts.visible)
        update_roi_box(handles, {"enabled": False})
        self.assertFalse(edges.visible or verts.visible)
        # Enabled: outline drawn, tracking extent/center.
        update_roi_box(handles, self._roi_block())
        self.assertTrue(edges.visible and verts.visible)
        self.assertEqual(np.asarray(edges.points).shape, (12, 2, 3))
        self.assertEqual(np.asarray(verts.points).shape, (8, 3))
        # Corner verts span the bounds: x in [-0.5, 0.5], y in [-1.0, 0.0].
        corners = np.asarray(verts.points)
        self.assertAlmostEqual(corners[:, 0].min(), -0.5)
        self.assertAlmostEqual(corners[:, 0].max(), 0.5)
        self.assertAlmostEqual(corners[:, 1].min(), -1.0)
        blue = np.asarray(edges.colors).copy()
        # Violation recolors the outline red.
        update_roi_box(handles, self._roi_block(
            right={"checked": True, "violated": True, "min_margin_m": -0.02,
                   "closest_face": "z_min"},
        ))
        self.assertFalse(np.array_equal(np.asarray(edges.colors), blue))
        # Toggle off hides the outline.
        update_roi_box(handles, self._roi_block(), visible=False)
        self.assertFalse(edges.visible or verts.visible)
        update_roi_box({}, self._roi_block())  # missing handles is a no-op

    def test_update_roi_box_preview(self):
        # Preview is its own yellow outline (edges + corner verts), no fill.
        edges = RecordingSceneHandle()
        verts = RecordingSceneHandle()
        handles = {"roi_box_preview_edges": edges, "roi_box_preview_verts": verts}
        update_roi_box_preview(handles, (-0.2, -0.2, 0.1), (0.2, 0.2, 0.5))
        self.assertTrue(edges.visible and verts.visible)
        self.assertEqual(np.asarray(edges.points).shape, (12, 2, 3))
        corners = np.asarray(verts.points)
        self.assertAlmostEqual(corners[:, 2].min(), 0.1)   # z min
        self.assertAlmostEqual(corners[:, 2].max(), 0.5)   # z max
        update_roi_box_preview(handles, None, None)
        self.assertFalse(edges.visible or verts.visible)
        update_roi_box_preview({}, (-0.2, -0.2, 0.1), (0.2, 0.2, 0.5))  # no-op

    def test_update_roi_panel_syncs_sliders_once(self):
        from rb_servo_gui.models import StateSnapshot

        class FakeSlider:
            def __init__(self):
                self.min = -1500.0
                self.max = 1500.0
                self.value = 0.0

        sliders = {f"roi_{a}_{s}": FakeSlider() for a in ("x", "y", "z") for s in ("min", "max")}
        handles = dict(sliders)
        state = StateSnapshot.parse(sample_state(roi_box=self._roi_block()))
        _update_roi_panel(handles, state)
        # Sliders come up at the applied bounds (mm) once.
        self.assertAlmostEqual(handles["roi_x_min"].value, -500.0)
        self.assertAlmostEqual(handles["roi_z_max"].value, 1000.0)
        self.assertTrue(handles.get("roi_sliders_synced"))
        # A later operator edit is not clobbered by subsequent state.
        handles["roi_x_min"].value = -250.0
        _update_roi_panel(handles, state)
        self.assertAlmostEqual(handles["roi_x_min"].value, -250.0)

    def test_persist_roi_bounds_rewrites_min_max_and_keeps_comments(self):
        import tempfile
        from pathlib import Path

        from rb_servo_gui.safety import persist_roi_bounds_to_config

        content = (
            "safety:\n"
            "  roi_box:\n"
            "    enable: true\n"
            "    min_m: [-0.5, -1.0, 0.0]   # startup box min\n"
            "    max_m: [0.5, 0.0, 1.0]\n"
            "    runtime_min_m: [-1.0, -1.5, -0.2]\n"
            "network:\n"
            "  min_m: [9, 9, 9]   # decoy outside the block\n"
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "stack.yaml"
            path.write_text(content, encoding="utf-8")
            ok, message = persist_roi_bounds_to_config(path, (-0.3, -0.8, 0.05), (0.3, 0.1, 0.9))
            self.assertTrue(ok, message)
            updated = path.read_text(encoding="utf-8")
            self.assertIn("    min_m: [-0.300, -0.800, 0.050]   # startup box min\n", updated)
            self.assertIn("    max_m: [0.300, 0.100, 0.900]\n", updated)
            self.assertIn("runtime_min_m: [-1.0, -1.5, -0.2]", updated)
            self.assertIn("min_m: [9, 9, 9]   # decoy outside the block", updated)

    def test_update_roi_box_visible_flag(self):
        edges = RecordingSceneHandle()
        verts = RecordingSceneHandle()
        handles = {"roi_box_edges": edges, "roi_box_verts": verts}
        # visible=False hides even with valid bounds.
        update_roi_box(handles, self._roi_block(), visible=False)
        self.assertFalse(edges.visible or verts.visible)
        # Disabled enforcement but valid bounds still draws when visible (the
        # region is a reference independent of server enforcement).
        update_roi_box(handles, self._roi_block(enabled=False), visible=True)
        self.assertTrue(edges.visible and verts.visible)

    def test_update_roi_panel_visibility_toggle(self):
        from rb_servo_gui.models import StateSnapshot

        class FakeToggle:
            def __init__(self, v):
                self.value = v

        edges = RecordingSceneHandle()
        verts = RecordingSceneHandle()
        scene = {"roi_box_edges": edges, "roi_box_verts": verts}
        handles = {"scene": scene, "roi_box_visible_toggle": FakeToggle(False)}
        state = StateSnapshot.parse(sample_state(roi_box=self._roi_block()))
        # Toggle OFF -> region hidden even though the server reports it enabled.
        _update_roi_panel(handles, state)
        self.assertFalse(edges.visible or verts.visible)
        # Toggle ON -> region drawn.
        handles["roi_box_visible_toggle"].value = True
        _update_roi_panel(handles, state)
        self.assertTrue(edges.visible and verts.visible)
        # Disabled enforcement but toggle ON -> still drawn (reference region).
        disabled = StateSnapshot.parse(sample_state(roi_box=self._roi_block(enabled=False)))
        _update_roi_panel(handles, disabled)
        self.assertTrue(edges.visible and verts.visible)


class LeaseBracketTest(unittest.TestCase):
    """One-shot GUI commands must be wrapped Acquire -> command -> Release so
    the GUI never camps on the lease (blocking teleop for lease_timeout_sec),
    while leaseless modes (EmergencyStop, SetSafetyFloorZ) stay bare."""

    def _recv_packets(self, sock, count):
        packets = []
        sock.settimeout(1.0)
        for _ in range(count):
            data, _ = sock.recvfrom(65536)
            packets.append(json.loads(data))
        return packets

    def _client_and_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
        return CommandClient(host="127.0.0.1", port=port, source_id="rb_gui_test"), sock

    @unittest.skipUnless(_local_udp_socket_available(), "local UDP unavailable")
    def test_leased_one_shot_is_bracketed(self):
        for mode in ("ArmMotion", "ResetFault"):
            client, sock = self._client_and_socket()
            try:
                client.send_lifecycle(mode)
                packets = self._recv_packets(sock, 3)
            finally:
                sock.close()
            self.assertEqual([p["mode"] for p in packets], ["AcquireLease", mode, "ReleaseLease"])
            seqs = [p["seq"] for p in packets]
            self.assertEqual(seqs, sorted(seqs))
            self.assertTrue(all(p["source_id"] == "rb_gui_test" for p in packets))

    @unittest.skipUnless(_local_udp_socket_available(), "local UDP unavailable")
    def test_leaseless_modes_stay_bare(self):
        client, sock = self._client_and_socket()
        try:
            client.send_lifecycle("EmergencyStop")
            client.send_set_safety_floor_z(0.012)
            packets = self._recv_packets(sock, 2)
        finally:
            sock.close()
        self.assertEqual([p["mode"] for p in packets], ["EmergencyStop", "SetSafetyFloorZ"])

    @unittest.skipUnless(_local_udp_socket_available(), "local UDP unavailable")
    def test_held_lease_rides_without_per_command_bracket(self):
        # Take control ON: acquire once, then streaming motion commands ride the
        # held lease (no Acquire/Release per packet); release once on OFF.
        client, sock = self._client_and_socket()
        try:
            acquire = client.acquire_lease()
            self.assertTrue(client.hold_lease)
            client.send(client.build_joint_target((0,) * 6, (0,) * 6))
            client.send(client.build_joint_target((1,) * 6, (1,) * 6))
            release = client.release_lease()
            self.assertFalse(client.hold_lease)
            packets = self._recv_packets(sock, 4)
        finally:
            sock.close()
        self.assertEqual(
            [p["mode"] for p in packets],
            ["AcquireLease", "JointTarget", "JointTarget", "ReleaseLease"],
        )
        self.assertEqual(acquire["mode"], "AcquireLease")
        self.assertEqual(release["mode"], "ReleaseLease")
        seqs = [p["seq"] for p in packets]
        self.assertEqual(seqs, sorted(seqs))

    @unittest.skipUnless(_local_udp_socket_available(), "local UDP unavailable")
    def test_held_lease_keepalive_resends_keep_fixed_seq(self):
        # Server-side path primitives re-send ONE prebuilt packet as keep-alives.
        # Under a held lease, send() must NOT re-issue the seq.
        client, sock = self._client_and_socket()
        try:
            client.acquire_lease()
            packet = client.build_tcp_linear_move(
                left_pose=(0.35, 0.1, 0.45, 0.0, 0.0, 0.0),
                right_pose=(0.35, -0.1, 0.45, 0.0, 0.0, 0.0),
                duration_sec=2.0,
            )
            fixed_seq = packet["seq"]
            client.send(packet)
            client.send(packet)
            client.send(packet)
            client.release_lease()
            packets = self._recv_packets(sock, 5)
        finally:
            sock.close()
        self.assertEqual(
            [p["mode"] for p in packets],
            ["AcquireLease", "TcpLinearMove", "TcpLinearMove", "TcpLinearMove", "ReleaseLease"],
        )
        path_seqs = [p["seq"] for p in packets[1:4]]
        self.assertEqual(path_seqs, [fixed_seq, fixed_seq, fixed_seq])

    @unittest.skipUnless(_local_udp_socket_available(), "local UDP unavailable")
    def test_leaseless_keepalive_resends_bump_seq(self):
        # Without a held lease, send() brackets each packet and re-issues a fresh
        # seq (one-shot commands need this to clear the Acquire seq).
        client, sock = self._client_and_socket()
        try:
            packet = client.build_tcp_linear_move(
                left_pose=(0.35, 0.1, 0.45, 0.0, 0.0, 0.0),
                right_pose=(0.35, -0.1, 0.45, 0.0, 0.0, 0.0),
                duration_sec=2.0,
            )
            client.send(packet)
            client.send(packet)
            packets = self._recv_packets(sock, 6)
        finally:
            sock.close()
        self.assertEqual(
            [p["mode"] for p in packets],
            [
                "AcquireLease", "TcpLinearMove", "ReleaseLease",
                "AcquireLease", "TcpLinearMove", "ReleaseLease",
            ],
        )
        self.assertNotEqual(packets[1]["seq"], packets[4]["seq"])


class SelfCollisionOverlayTest(unittest.TestCase):
    @staticmethod
    def _handles():
        return {
            "left_base": RecordingSceneHandle(),
            "right_base": RecordingSceneHandle(),
            "left_base_ref": RecordingSceneHandle(),
            "right_base_ref": RecordingSceneHandle(),
            "left_base_collision": RecordingSceneHandle(),
            "right_base_collision": RecordingSceneHandle(),
            "stand_mesh": RecordingSceneHandle(),
            "stand_mesh_collision": RecordingSceneHandle(),
            "left_urdf_collision": RecordingUrdf(),
            "right_urdf_collision": RecordingUrdf(),
        }

    @staticmethod
    def _latest(*, violated, physical_real, q_actual, q_sent, pair="left_stand"):
        store = StateStore(stale_after_sec=5.0)
        arm = {
            "has_valid_joint_state": True,
            "q_actual_deg": q_actual,
            "q_sent_deg": q_sent,
            "physical_motion_expected": physical_real,
        }
        payload = sample_state(
            left=dict(arm),
            right=dict(arm),
            self_collision={"enabled": True, "checked": True, "violated": violated,
                            "pair": pair,
                            "stand_capsule": "lower_column" if pair and "stand" in pair else None},
        )
        assert store.update_from_json_bytes(json.dumps(payload).encode(), received_monotonic=time.monotonic())
        return store.latest()

    def test_real_violation_paints_only_the_pair_red(self):
        handles = self._handles()
        latest = self._latest(
            violated=True, physical_real=True, pair="left_stand",
            q_actual=[1, 2, 3, 4, 5, 6], q_sent=[9, 9, 9, 9, 9, 9])
        update_self_collision_overlay(handles, latest)
        # left_stand pair: red overlay at q_actual for the LEFT arm + stand only;
        # the right arm stays normal.
        self.assertTrue(handles["left_base_collision"].visible)
        self.assertFalse(handles["right_base_collision"].visible)
        self.assertTrue(handles["stand_mesh_collision"].visible)
        self.assertFalse(handles["stand_mesh"].visible)
        self.assertFalse(handles["left_base"].visible)
        self.assertTrue(handles["right_base"].visible)
        self.assertAlmostEqual(handles["left_urdf_collision"].configs[-1][0], math.radians(1.0))
        self.assertEqual(handles["right_urdf_collision"].configs, [])

    def test_sim_violation_paints_ghost_pair_red_and_keeps_solid(self):
        handles = self._handles()
        latest = self._latest(
            violated=True, physical_real=False, pair="left_stand",
            q_actual=[1, 2, 3, 4, 5, 6], q_sent=[9, 8, 7, 6, 5, 4])
        update_self_collision_overlay(handles, latest)
        # Red overlay shown at q_sent (the commanded ghost pose) for the LEFT arm;
        # only that ghost is replaced; the right ghost and the solid robots stay.
        self.assertTrue(handles["left_base_collision"].visible)
        self.assertFalse(handles["right_base_collision"].visible)
        self.assertFalse(handles["left_base_ref"].visible)
        self.assertIsNone(handles["right_base_ref"].visible)  # untouched
        self.assertTrue(handles["left_base"].visible)
        self.assertTrue(handles["stand_mesh_collision"].visible)
        self.assertAlmostEqual(handles["left_urdf_collision"].configs[-1][0], math.radians(9.0))

    def test_left_right_pair_keeps_stand_normal(self):
        handles = self._handles()
        latest = self._latest(
            violated=True, physical_real=True, pair="left_right",
            q_actual=[1, 2, 3, 4, 5, 6], q_sent=None)
        update_self_collision_overlay(handles, latest)
        self.assertTrue(handles["left_base_collision"].visible)
        self.assertTrue(handles["right_base_collision"].visible)
        self.assertFalse(handles["stand_mesh_collision"].visible)
        self.assertTrue(handles["stand_mesh"].visible)
        self.assertFalse(handles["left_base"].visible)
        self.assertFalse(handles["right_base"].visible)

    def test_unknown_pair_falls_back_to_all_red(self):
        handles = self._handles()
        latest = self._latest(
            violated=True, physical_real=True, pair=None,
            q_actual=[1, 2, 3, 4, 5, 6], q_sent=None)
        update_self_collision_overlay(handles, latest)
        self.assertTrue(handles["left_base_collision"].visible)
        self.assertTrue(handles["right_base_collision"].visible)
        self.assertTrue(handles["stand_mesh_collision"].visible)
        self.assertFalse(handles["stand_mesh"].visible)

    def test_clear_state_restores_normal_models(self):
        handles = self._handles()
        latest = self._latest(
            violated=False, physical_real=True,
            q_actual=[1, 2, 3, 4, 5, 6], q_sent=None)
        update_self_collision_overlay(handles, latest)
        self.assertFalse(handles["left_base_collision"].visible)
        self.assertFalse(handles["right_base_collision"].visible)
        self.assertFalse(handles["stand_mesh_collision"].visible)
        self.assertTrue(handles["stand_mesh"].visible)
        self.assertTrue(handles["left_base"].visible)
        self.assertTrue(handles["right_base"].visible)
        self.assertEqual(handles["left_urdf_collision"].configs, [])


class _CheckGeomUrdf:
    # Mirror the real viser ViserUrdf: it exposes get_actuated_joint_names()
    # returning the unified URDF's PREFIXED joint names. Without this the overlay
    # posing path could silently re-map via the 6 short _ROBOT_JOINT_NAMES and
    # freeze the hull fully extended (the bug this overlay must not regress to).
    def __init__(self, actuated=("R_j2", "L_j1", "L_j0")):
        self.configs = []
        self.show_collision = None
        self._actuated = tuple(actuated)

    def get_actuated_joint_names(self):
        return self._actuated

    def update_cfg(self, config):
        self.configs.append(tuple(float(v) for v in config))


class _SolidUrdf:
    def __init__(self):
        self.show_visual = None


class SelfCollisionCheckGeomOverlayTest(unittest.TestCase):
    @staticmethod
    def _latest(*, manifest):
        store = StateStore(stale_after_sec=5.0)
        sc = {"enabled": True, "checked": True, "violated": False, "mesh": True}
        if manifest is not None:
            sc["manifest"] = manifest
        payload = sample_state(self_collision=sc)
        # Distinct left/right joints so the unified mapping order is observable.
        payload["left"]["q_actual_deg"] = [1, 2, 3, 4, 5, 6]
        payload["left"]["q_sent_deg"] = [1, 2, 3, 4, 5, 6]
        payload["right"]["q_actual_deg"] = [7, 8, 9, 10, 11, 12]
        payload["right"]["q_sent_deg"] = [7, 8, 9, 10, 11, 12]
        assert store.update_from_json_bytes(
            json.dumps(payload).encode(), received_monotonic=time.monotonic()
        )
        return store.latest()

    _MANIFEST = {
        "unified_urdf": "/nonexistent/dual.urdf",
        "left_prefix": "L_",
        "right_prefix": "R_",
        "joint_names": ["j0", "j1", "j2", "j3", "j4", "j5"],
        "d_hard_m": 0.005,
        "d_slow_m": 0.025,
    }

    def _primed_handles(self, urdf):
        # Pre-inject the lazily-built overlay so the test does not need real viser.
        return {
            "checkgeom_urdf": urdf,
            "checkgeom_manifest": {
                "left_prefix": "L_",
                "right_prefix": "R_",
                "joint_names": ["j0", "j1", "j2", "j3", "j4", "j5"],
                # Interleaved + subset: exercises lookup-by-name, not positional.
                "actuated": ("R_j2", "L_j1", "L_j0"),
            },
            "left_urdf": _SolidUrdf(),
            "right_urdf": _SolidUrdf(),
        }

    def test_joints_mapped_to_prefixed_unified_joints(self):
        urdf = _CheckGeomUrdf()
        handles = self._primed_handles(urdf)
        update_self_collision_check_geom(handles, self._latest(manifest=self._MANIFEST), show=True)
        cfg = urdf.configs[-1]
        # actuated order (R_j2, L_j1, L_j0) -> right[2]=9, left[1]=2, left[0]=1.
        self.assertAlmostEqual(cfg[0], math.radians(9.0))
        self.assertAlmostEqual(cfg[1], math.radians(2.0))
        self.assertAlmostEqual(cfg[2], math.radians(1.0))
        self.assertTrue(urdf.show_collision)
        # Collision view hides the solid robots while the overlay is on.
        self.assertFalse(handles["left_urdf"].show_visual)
        self.assertFalse(handles["right_urdf"].show_visual)

    def test_toggle_off_hides_and_restores_solids(self):
        urdf = _CheckGeomUrdf()
        handles = self._primed_handles(urdf)
        latest = self._latest(manifest=self._MANIFEST)
        update_self_collision_check_geom(handles, latest, show=True)
        update_self_collision_check_geom(handles, latest, show=False)
        self.assertFalse(urdf.show_collision)
        self.assertTrue(handles["left_urdf"].show_visual)
        self.assertTrue(handles["right_urdf"].show_visual)

    def test_no_manifest_builds_no_overlay(self):
        handles = {"_server": object(), "left_urdf": _SolidUrdf(), "right_urdf": _SolidUrdf()}
        update_self_collision_check_geom(handles, self._latest(manifest=None), show=True)
        self.assertNotIn("checkgeom_urdf", handles)

    def test_gripper_attached_at_attachment_site_frames(self):
        import trimesh

        class _Frame:
            def __init__(self, name):
                self.name = name

        class _RecScene:
            def __init__(self):
                self.meshes = {}

            def add_mesh_simple(self, name, **kwargs):
                handle = type("_H", (), {"visible": kwargs.get("visible")})()
                self.meshes[name] = handle
                return handle

        class _RecServer:
            def __init__(self):
                self.scene = _RecScene()

        tmp = tempfile.NamedTemporaryFile(suffix=".stl", delete=False)
        tmp.close()
        trimesh.creation.box((0.05, 0.05, 0.1)).export(tmp.name)
        self.addCleanup(lambda: os.unlink(tmp.name))

        root = "/stand/sc_urdf_world/collision_checkgeom"
        overlay = type("_Overlay", (), {})()
        overlay._joint_frames = [
            _Frame(f"{root}/world/base/stand/sl/dual_rb3_730e_left_link6"),
            _Frame(f"{root}/world/base/stand/sl/dual_rb3_730e_left_attachment_site"),
            _Frame(f"{root}/world/base/stand/sr/dual_rb3_730e_right_attachment_site"),
        ]
        server = _RecServer()
        handles = {"_server": server}
        manifest = {
            "pika_gripper_mesh": tmp.name,
            "left_prefix": "dual_rb3_730e_left_",
            "right_prefix": "dual_rb3_730e_right_",
            "gripper_attach": {
                "frame_suffix": "attachment_site",
                "rpy": [0.0, 0.0, math.pi / 2.0],
                "mesh_scale": 0.001,
            },
        }
        _attach_checkgeom_gripper(handles, overlay, manifest)
        names = list(server.scene.meshes)
        self.assertTrue(any(n.endswith("dual_rb3_730e_left_attachment_site/pika_gripper") for n in names))
        self.assertTrue(any(n.endswith("dual_rb3_730e_right_attachment_site/pika_gripper") for n in names))
        self.assertEqual(len(handles["checkgeom_gripper"]), 2)

    def test_sc_world_frame_rotates_minus_90_about_z(self):
        # The unified URDF's world->stand fixed joint is +90deg about Z and the scene
        # /stand frame is the URDF `stand` frame, so the overlay/witness frame must be
        # -90deg about Z. Quaternion (w,x,y,z) for Rz(-90deg) = (cos45, 0, 0, -sin45).
        class _RecFrameScene:
            def __init__(self):
                self.frames = {}

            def add_frame(self, name, *, wxyz, position, show_axes):
                self.frames[name] = (wxyz, position)
                return object()

        class _RecFrameServer:
            def __init__(self):
                self.scene = _RecFrameScene()

        server = _RecFrameServer()
        handles = {"_server": server}
        path = _ensure_sc_world_frame(handles)
        self.assertEqual(path, "/stand/sc_urdf_world")
        wxyz, position = server.scene.frames[path]
        self.assertAlmostEqual(wxyz[0], math.cos(math.pi / 4), places=5)
        self.assertAlmostEqual(wxyz[3], -math.sin(math.pi / 4), places=5)
        self.assertEqual(tuple(position), (0.0, 0.0, 0.0))
        # Idempotent: a second call does not recreate the frame.
        _ensure_sc_world_frame(handles)
        self.assertEqual(len(server.scene.frames), 1)


class SolidRobotPoseSourceTest(unittest.TestCase):
    """In pgmode controller-sim q_actual is decoupled from the command, so the solid
    robot must follow q_sent (what the guard checks); on real motion it follows q_actual."""

    @staticmethod
    def _latest(*, controller_sim):
        store = StateStore(stale_after_sec=5.0)
        payload = sample_state()
        for side in ("left", "right"):
            payload[side]["q_actual_deg"] = [9, 9, 9, 9, 9, 9]
            payload[side]["q_sent_deg"] = [1, 2, 3, 4, 5, 6]
            if controller_sim:
                payload[side]["controller_simulation_mode"] = {"operation_mode": "simulation"}
        assert store.update_from_json_bytes(
            json.dumps(payload).encode(), received_monotonic=time.monotonic())
        return store.latest()

    def test_controller_sim_uses_q_sent(self):
        left_urdf = RecordingUrdf()
        handles = {"left_urdf": left_urdf, "right_urdf": RecordingUrdf()}
        update_scene_markers(handles, self._latest(controller_sim=True))
        self.assertAlmostEqual(left_urdf.configs[-1][1], math.radians(2.0))   # q_sent[1]
        self.assertNotAlmostEqual(left_urdf.configs[-1][1], math.radians(9.0))

    def test_real_motion_uses_q_actual(self):
        left_urdf = RecordingUrdf()
        handles = {"left_urdf": left_urdf, "right_urdf": RecordingUrdf()}
        update_scene_markers(handles, self._latest(controller_sim=False))
        self.assertAlmostEqual(left_urdf.configs[-1][1], math.radians(9.0))   # q_actual[1]


class SelfCollisionNearPairVizTest(unittest.TestCase):
    """#2 viz: near-pair tubes colored by clearance band; #3: closest-pair readout."""

    @staticmethod
    def _latest(near_pairs, *, d_slow=0.025):
        store = StateStore(stale_after_sec=5.0)
        payload = sample_state(self_collision={
            "enabled": True, "checked": True, "violated": False,
            "margin_m": 0.005,  # == d_hard
            "manifest": {"d_hard_m": 0.005, "d_slow_m": d_slow},
            "near_pairs": near_pairs,
            "min_clearance_m": min((p["clearance_m"] for p in near_pairs), default=1.0),
        })
        assert store.update_from_json_bytes(json.dumps(payload).encode(), received_monotonic=time.monotonic())
        return store.latest()

    @staticmethod
    def _pair(name_a, name_b, clearance):
        return {"name_a": name_a, "name_b": name_b,
                "p_a_m": [0.0, 0.0, 0.0], "p_b_m": [0.0, 0.0, clearance], "clearance_m": clearance}

    def test_near_pair_color_bands(self):
        from rb_servo_gui import scene

        class _Scene:
            def __init__(self):
                self.colors = {}

            def add_frame(self, name, **kw):
                return object()

            def add_mesh_simple(self, name, **kw):
                self.colors[name] = kw.get("color")
                return type("_H", (), {"visible": kw.get("visible")})()

        class _Server:
            def __init__(self):
                self.scene = _Scene()

        server = _Server()
        handles = {"_server": server}
        latest = self._latest([
            self._pair("a", "b", 0.003),   # < d_hard -> red
            self._pair("c", "d", 0.012),   # [d_hard, d_slow) -> amber
            self._pair("e", "f", 0.040),   # >= d_slow -> green
        ], d_slow=0.025)
        update_self_collision_near_pairs(handles, latest, show=True)
        colors = list(server.scene.colors.values())
        self.assertIn(scene._SELF_COLLISION_NEAR_HARD_RGB, colors)
        self.assertIn(scene._SELF_COLLISION_NEAR_CAUTION_RGB, colors)
        self.assertIn(scene._SELF_COLLISION_NEAR_OK_RGB, colors)

    def test_status_names_closest_pair(self):
        latest = self._latest([
            self._pair("dual_rb3_730e_left_link4_2", "stand_body_shoulder_0", 0.008),
            self._pair("dual_rb3_730e_left_link2_2", "dual_rb3_730e_left_link4_1", 0.030),
        ])
        txt = _format_self_collision_status(latest, stale=False)
        # closest pair (8mm) named, with the verbose URDF prefixes stripped.
        self.assertIn("left_link4_2", txt)
        self.assertIn("shoulder_0", txt)
        self.assertNotIn("dual_rb3_730e_", txt)


class WaypointPersistenceTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self._prev = os.environ.get("RB_GUI_WAYPOINTS_PATH")
        os.environ["RB_GUI_WAYPOINTS_PATH"] = os.path.join(self._tmp.name, "nested", "waypoints.json")

    def tearDown(self):
        if self._prev is None:
            os.environ.pop("RB_GUI_WAYPOINTS_PATH", None)
        else:
            os.environ["RB_GUI_WAYPOINTS_PATH"] = self._prev
        self._tmp.cleanup()

    def test_missing_file_loads_empty(self):
        self.assertEqual(_load_waypoints(), {})

    def test_save_load_roundtrip_normalizes_types(self):
        waypoints = {
            "home": {
                "left_q": (0.0, -30.0, 80.0, 0.0, 60.0, 0.0),
                "left_pose": (0.4, 0.1, 0.3, 0.0, 1.57, 0.0),
                "left_quat": (0.0, 0.707, 0.0, 0.707),
                "right_q": (1.0, 2.0, 3.0, 4.0, 5.0, 6.0),
                "right_pose": (-0.2, -0.4, 0.25, 0.1, 0.2, 0.3),
                "right_quat": None,
            }
        }
        ok, path = _save_waypoints(waypoints)
        self.assertTrue(ok, path)
        self.assertTrue(os.path.exists(path))  # nested dir created
        loaded = _load_waypoints()
        self.assertEqual(list(loaded.keys()), ["home"])
        self.assertEqual(loaded["home"]["left_q"], (0.0, -30.0, 80.0, 0.0, 60.0, 0.0))
        self.assertIsInstance(loaded["home"]["left_q"], tuple)  # lists -> tuples
        self.assertIsNone(loaded["home"]["right_quat"])

    def test_corrupt_file_loads_empty(self):
        path = os.environ["RB_GUI_WAYPOINTS_PATH"]
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as handle:
            handle.write("{not valid json")
        self.assertEqual(_load_waypoints(), {})

    def test_delete_selected_waypoint(self):
        class _Dropdown:
            value = "home"

        handles = {
            "waypoints": {"home": {"left_q": None}, "other": {"left_q": None}},
            "waypoint_dropdown": _Dropdown(),
        }
        ok, name = _delete_waypoint(handles)
        self.assertTrue(ok)
        self.assertEqual(name, "home")
        self.assertEqual(list(handles["waypoints"].keys()), ["other"])

        handles["waypoint_dropdown"].value = "(none)"
        ok, message = _delete_waypoint(handles)
        self.assertFalse(ok)
        self.assertIn("no waypoint selected", message)


class InitMotionPersistenceTest(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self._prev = os.environ.get("RB_GUI_INIT_MOTION_PATH")
        os.environ["RB_GUI_INIT_MOTION_PATH"] = os.path.join(self._tmp.name, "nested", "init_motion.json")

    def tearDown(self):
        if self._prev is None:
            os.environ.pop("RB_GUI_INIT_MOTION_PATH", None)
        else:
            os.environ["RB_GUI_INIT_MOTION_PATH"] = self._prev
        self._tmp.cleanup()

    def test_missing_file_loads_none(self):
        self.assertEqual(_load_init_joints(), (None, None))

    def test_save_load_roundtrip(self):
        left = (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
        right = (-1.0, -2.0, -3.0, -4.0, -5.0, -6.0)
        ok, path = _save_init_joints(left, right)
        self.assertTrue(ok, path)
        self.assertTrue(os.path.exists(path))  # nested dir created
        self.assertEqual(_load_init_joints(), (left, right))

    def test_set_waypoint_as_init_updates_safety_and_persists(self):
        store = StateStore(stale_after_sec=0.5)
        safety = OperatorSafety(
            store,
            CommandClient(host="127.0.0.1", port=0, source_id="test"),
            init_left_joint_deg=(0.0,) * 6,
            init_right_joint_deg=(0.0,) * 6,
        )
        left = (10.0, 20.0, 30.0, 40.0, 50.0, 60.0)
        right = (-10.0, -20.0, -30.0, -40.0, -50.0, -60.0)

        class _Dropdown:
            value = "home"

        handles = {
            "waypoints": {"home": {"left_q": left, "right_q": right}},
            "waypoint_dropdown": _Dropdown(),
        }
        ok, message = _set_waypoint_as_init(handles, safety)
        self.assertTrue(ok, message)
        self.assertEqual(safety.init_left_joint_deg, left)
        self.assertEqual(safety.init_right_joint_deg, right)
        self.assertEqual(_load_init_joints(), (left, right))

    def test_parse_joint6_accepts_comma_or_space_and_rejects_bad(self):
        self.assertEqual(_parse_joint6("1, 2, 3, 4, 5, 6"), (1.0, 2.0, 3.0, 4.0, 5.0, 6.0))
        self.assertEqual(_parse_joint6("1 2 3 4 5 6"), (1.0, 2.0, 3.0, 4.0, 5.0, 6.0))
        self.assertIsNone(_parse_joint6("1 2 3 4 5"))      # too few
        self.assertIsNone(_parse_joint6("1 2 3 4 5 6 7"))  # too many
        self.assertIsNone(_parse_joint6("1 2 3 4 5 x"))    # non-numeric
        self.assertIsNone(_parse_joint6("1 2 3 4 5 nan"))  # non-finite

    def test_format_joint6_roundtrips_through_parse(self):
        self.assertEqual(_format_joint6(None), "")
        values = (-131.663, 73.0, 113.4, -80.88, -107.064, -145.949)
        self.assertEqual(_parse_joint6(_format_joint6(values)), values)

    def test_apply_init_joints_live_updates_safety_and_persists(self):
        store = StateStore(stale_after_sec=0.5)
        safety = OperatorSafety(
            store,
            CommandClient(host="127.0.0.1", port=0, source_id="test"),
            init_left_joint_deg=(0.0,) * 6,
            init_right_joint_deg=(0.0,) * 6,
        )
        ok, message = _apply_init_joints_live(
            safety, "10, 20, 30, 40, 50, 60", "-10 -20 -30 -40 -50 -60"
        )
        self.assertTrue(ok, message)
        # live runtime target updated (the next InitMotion press uses this)...
        self.assertEqual(safety.init_left_joint_deg, (10.0, 20.0, 30.0, 40.0, 50.0, 60.0))
        self.assertEqual(safety.init_right_joint_deg, (-10.0, -20.0, -30.0, -40.0, -50.0, -60.0))
        # ...and persisted so it survives a restart.
        self.assertEqual(
            _load_init_joints(),
            ((10.0, 20.0, 30.0, 40.0, 50.0, 60.0), (-10.0, -20.0, -30.0, -40.0, -50.0, -60.0)),
        )

    def test_apply_init_joints_live_rejects_bad_input_without_touching_target(self):
        store = StateStore(stale_after_sec=0.5)
        safety = OperatorSafety(
            store,
            CommandClient(host="127.0.0.1", port=0, source_id="test"),
            init_left_joint_deg=(1.0,) * 6,
            init_right_joint_deg=(2.0,) * 6,
        )
        ok, message = _apply_init_joints_live(safety, "1 2 3", "1 2 3 4 5 6")
        self.assertFalse(ok)
        self.assertIn("6 finite joint values", message)
        self.assertEqual(safety.init_left_joint_deg, (1.0,) * 6)   # unchanged
        self.assertEqual(safety.init_right_joint_deg, (2.0,) * 6)

    def test_current_joints_text_stale_stream_returns_message(self):
        store = StateStore(stale_after_sec=0.5)  # no state pushed -> latest() is None
        left_text, right_text, message = _current_joints_text(store)
        self.assertIsNone(left_text)
        self.assertIsNone(right_text)
        self.assertIn("stale", message)

    def test_set_waypoint_as_init_rejects_missing_joints(self):
        store = StateStore(stale_after_sec=0.5)
        safety = OperatorSafety(store, CommandClient(host="127.0.0.1", port=0, source_id="test"))

        class _Dropdown:
            value = "partial"

        handles = {
            "waypoints": {"partial": {"left_q": None, "right_q": (1.0,) * 6}},
            "waypoint_dropdown": _Dropdown(),
        }
        ok, message = _set_waypoint_as_init(handles, safety)
        self.assertFalse(ok)
        self.assertIn("missing joint capture", message)


class IkInfeasibleRegionTest(unittest.TestCase):
    """The IK-infeasible region overlay (scene loader + visibility toggle)."""

    @staticmethod
    def _mesh_scene_server():
        class _Scene:
            def __init__(self):
                self.meshes = {}

            def add_mesh_simple(self, name, **kwargs):
                handle = RecordingSceneHandle()
                handle.visible = kwargs.get("visible", True)
                handle.kwargs = kwargs
                self.meshes[name] = handle
                return handle

            def add_line_segments(self, name, **kwargs):
                handle = RecordingSceneHandle()
                handle.visible = kwargs.get("visible", True)
                handle.kwargs = kwargs
                self.meshes[name] = handle
                return handle

        class _Server:
            def __init__(self):
                self.scene = _Scene()

        return _Server()

    def _write_asset(self, **extra):
        verts = np.array([[0, 0, 0], [0.1, 0, 0], [0.1, 0.1, 0], [0, 0.1, 0.1]],
                         dtype=np.float32)
        faces = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int32)
        path = tempfile.NamedTemporaryFile(suffix=".npz", delete=False)
        path.close()
        self.addCleanup(lambda: os.path.exists(path.name) and os.unlink(path.name))
        np.savez_compressed(path.name,
                            left_vertices_base_m=verts, left_faces=faces,
                            right_vertices_base_m=verts, right_faces=faces, **extra)
        return path.name

    def test_toggle_shows_and_hides_both_arms(self):
        handles = {
            "left_ik_infeasible": RecordingSceneHandle(),
            "right_ik_infeasible": RecordingSceneHandle(),
            "left_ik_infeasible_outline": RecordingSceneHandle(),
            "right_ik_infeasible_outline": RecordingSceneHandle(),
        }
        set_ik_infeasible_region_visible(handles, True)
        self.assertTrue(handles["left_ik_infeasible"].visible)
        self.assertTrue(handles["right_ik_infeasible"].visible)
        # The bright rim outline toggles together with the fill.
        self.assertTrue(handles["left_ik_infeasible_outline"].visible)
        self.assertTrue(handles["right_ik_infeasible_outline"].visible)
        set_ik_infeasible_region_visible(handles, False)
        self.assertFalse(handles["left_ik_infeasible"].visible)
        self.assertFalse(handles["right_ik_infeasible"].visible)
        self.assertFalse(handles["left_ik_infeasible_outline"].visible)
        self.assertFalse(handles["right_ik_infeasible_outline"].visible)

    def test_loads_rim_outline_with_cylinder_geometry(self):
        asset = self._write_asset(
            left_cells=96, right_cells=96, radius_m=0.2, z_lo_m=-0.3, z_hi_m=0.5,
        )
        server = self._mesh_scene_server()
        handles: dict = {}
        with mock.patch.dict(os.environ, {"RB_GUI_IK_INFEASIBLE": asset}):
            _add_ik_infeasible_region(server, handles)
        self.assertIn("left_ik_infeasible_outline", handles)
        node = server.scene.meshes["/stand/left_base/ik_infeasible_outline"]
        seg = np.asarray(node.kwargs["points"])
        self.assertEqual(seg.ndim, 3)
        self.assertEqual(seg.shape[1:], (2, 3))
        # Rim circles sit at z_lo / z_hi; the radius matches the asset.
        zs = np.unique(seg[:, :, 2])
        self.assertTrue(np.any(np.isclose(zs, -0.3, atol=1e-5)))
        self.assertTrue(np.any(np.isclose(zs, 0.5, atol=1e-5)))
        radii = np.sqrt(seg[:, :, 0] ** 2 + seg[:, :, 1] ** 2)
        self.assertAlmostEqual(float(radii.max()), 0.2, places=5)
        self.assertEqual(tuple(node.kwargs.get("colors")), (255, 130, 130))
        self.assertFalse(node.kwargs.get("visible", True))

    def test_toggle_is_safe_without_handles(self):
        set_ik_infeasible_region_visible({}, True)  # no-op, must not raise
        set_ik_infeasible_region_visible(None, True)

    def test_loads_asset_to_both_arm_bases(self):
        asset = self._write_asset(left_cells=2000, right_cells=2321)
        server = self._mesh_scene_server()
        handles: dict = {}
        with mock.patch.dict(os.environ, {"RB_GUI_IK_INFEASIBLE": asset}):
            _add_ik_infeasible_region(server, handles)
        self.assertIn("left_ik_infeasible", handles)
        self.assertIn("right_ik_infeasible", handles)
        self.assertNotIn("ik_infeasible_error", handles)
        self.assertEqual(handles["ik_infeasible_cells"], 4321)
        # both attached under the per-arm base node, hidden by default, red+back-cull
        self.assertIn("/stand/left_base/ik_infeasible", server.scene.meshes)
        self.assertIn("/stand/right_base/ik_infeasible", server.scene.meshes)
        left = server.scene.meshes["/stand/left_base/ik_infeasible"]
        self.assertFalse(left.visible)
        # convex watertight cylinder -> back-cull (was "double" for the old concave shell)
        self.assertEqual(left.kwargs.get("side"), "back")
        self.assertEqual(tuple(left.kwargs.get("color")), (255, 45, 45))

    def test_loads_cylinder_radius_for_status(self):
        asset = self._write_asset(left_cells=96, right_cells=96, radius_m=0.191)
        server = self._mesh_scene_server()
        handles: dict = {}
        with mock.patch.dict(os.environ, {"RB_GUI_IK_INFEASIBLE": asset}):
            _add_ik_infeasible_region(server, handles)
        self.assertAlmostEqual(handles["ik_infeasible_radius_m"], 0.191, places=6)

    def test_missing_asset_records_error(self):
        server = self._mesh_scene_server()
        handles: dict = {}
        missing = os.path.join(tempfile.gettempdir(), "no_such_ik_asset.npz")
        with mock.patch.dict(os.environ, {"RB_GUI_IK_INFEASIBLE": missing}):
            _add_ik_infeasible_region(server, handles)
        self.assertNotIn("left_ik_infeasible", handles)
        self.assertIn("ik_infeasible_error", handles)

    def test_asset_path_honors_env_override(self):
        with mock.patch.dict(os.environ, {"RB_GUI_IK_INFEASIBLE": "/tmp/custom_ik.npz"}):
            self.assertEqual(str(_ik_infeasible_path()), "/tmp/custom_ik.npz")


if __name__ == "__main__":
    unittest.main()
