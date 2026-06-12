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
    _apply_tcp_delta_and_send_pose_target,
    _apply_tcp_delta_to_target,
    _angular_step_radians,
    _build_operator_monitors,
    _circle_overlay_bind_from_args_env,
    _env_joint6,
    _format_cartesian_solve_status,
    _format_circle_overlay_status,
    _format_fk_status,
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
    _operator_monitor_dynamic_html,
    _operator_monitor_static_html,
    _pose6_from_mounts,
    _pose_orientation_wxyz,
    _pose_wxyz,
    _quat_to_matrix,
    _clear_tcp_target_user_moved,
    _send_tcp_linear_move_from_marker,
    _send_init_motion_and_reset_targets,
    _sim_readiness_from_env,
    _tcp_display_mode,
    _tcp_local_delta_from_target,
    _tcp_frame_mode,
    _tcp_linear_arm,
    _tcp_linear_orientation_mode,
    _tcp_target_pose,
    _tcp_target_wxyz,
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
from rb_servo_gui.models import CIRCLE_OVERLAY_SCHEMA_VERSION, CircleOverlaySnapshot, Pose6D
from rb_servo_gui.overlay_receiver import CircleOverlayReceiver, CircleOverlayStore, parse_udp_bind
from rb_servo_gui.safety import OperatorSafety, Readiness, normalize_observed_mode_backend
from rb_servo_gui.scene import (
    _add_robot_urdfs,
    _add_scene_fallback,
    _circle_overlay_points,
    _reference_ghost_active,
    _robot_urdf_path,
    update_circle_overlay,
    update_floor_plane,
    update_floor_plane_preview,
    update_self_collision_overlay,
)
from rb_servo_gui.status_panel import _format_floor_constraint_status
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
        sim_ready=False,
        cartesian_available=None,
        enable_tcp_pose=False,
        enable_controller_sim_cartesian=False,
        stale=False,
        init_left_joint_deg=None,
        init_right_joint_deg=None,
        init_motion_timeout_sec=10.0,
    ):
        store = StateStore(stale_after_sec=0.5)
        if state is not None:
            received = time.monotonic() - 1.0 if stale else time.monotonic()
            self.assertTrue(store.update_from_json_bytes(json.dumps(state).encode(), received_monotonic=received))
        client = RecordingClient()
        safety = OperatorSafety(
            store,
            client,
            desired_mode=desired,
            observed_server_mode=observed,
            observed_backend=observed_backend,
            sim_readiness=Readiness(
                running=True,
                connected=True,
                ready=sim_ready,
                no_go_reason="sim readiness not proven",
                cartesian_available=cartesian_available,
                cartesian_no_go_reason="cartesian readiness not proven",
            ),
            enable_tcp_pose_commands=enable_tcp_pose,
            enable_controller_sim_cartesian=enable_controller_sim_cartesian,
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
            state[arm]["mode"] = "TcpTwistLocal"
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
        self.assertIn("command=TcpTwistLocal", status)
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

    def test_robot_urdf_path_uses_descriptions_dir_env(self):
        descriptions_dir = Path(__file__).resolve().parents[2] / "rb_servo_server" / "descriptions"
        old_value = os.environ.get("RB_GUI_DESCRIPTIONS_DIR")
        try:
            os.environ["RB_GUI_DESCRIPTIONS_DIR"] = str(descriptions_dir)
            self.assertEqual(_robot_urdf_path(), descriptions_dir / "urdf" / "rb3_730e.urdf")
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

    def test_init_motion_blocks_real_mode_and_readiness_no_go(self):
        # Real/sim gating retired: InitMotion sends in real mode too.
        _, client, real_safety = self.make_safety(
            sample_state(motion_state="ArmedHold"),
            desired="real",
            observed="real",
            init_left_joint_deg=_DEFAULT_INIT_LEFT_JOINTS_DEG,
            init_right_joint_deg=_DEFAULT_INIT_RIGHT_JOINTS_DEG,
        )
        ok, reason = real_safety.send_init_motion()
        self.assertTrue(ok, reason)
        self.assertEqual(client.sent_packets[-1]["mode"], "JointTarget")

        _, client, sim_safety = self.make_safety(
            sample_state(motion_state="ArmedHold"),
            desired="simulation",
            observed="simulation",
            sim_ready=False,
            init_left_joint_deg=_DEFAULT_INIT_LEFT_JOINTS_DEG,
            init_right_joint_deg=_DEFAULT_INIT_RIGHT_JOINTS_DEG,
        )
        ok, reason = sim_safety.send_init_motion()
        self.assertFalse(ok)
        self.assertIn("sim readiness", reason)
        self.assertEqual(client.sent_packets, [])

    def test_init_motion_sends_joint_target_with_long_timeout(self):
        _, client, safety = self.make_safety(
            sample_state(motion_state="ArmedHold"),
            init_left_joint_deg=_DEFAULT_INIT_LEFT_JOINTS_DEG,
            init_right_joint_deg=_DEFAULT_INIT_RIGHT_JOINTS_DEG,
            init_motion_timeout_sec=10.0,
        )
        ok, reason = safety.send_init_motion()
        self.assertTrue(ok, reason)
        packet = client.sent_packets[-1]
        self.assertEqual(packet["mode"], "JointTarget")
        self.assertEqual(packet["left"]["q_target_deg"], list(_DEFAULT_INIT_LEFT_JOINTS_DEG))
        self.assertEqual(packet["right"]["q_target_deg"], list(_DEFAULT_INIT_RIGHT_JOINTS_DEG))
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

    def test_simulation_mode_is_no_go_until_readiness_passes(self):
        _, client, safety = self.make_safety(sample_state(), desired="simulation", observed="simulation", sim_ready=False)
        ok, reason = safety.jog_joint("left", 0, 1.0)
        self.assertFalse(ok)
        self.assertIn("sim readiness", reason)
        self.assertEqual(client.sent_packets, [])

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
        # Real/sim gating retired: a desired/observed mode mismatch no longer
        # blocks commands; desired=simulation still gates on sim readiness.
        _, client, safety = self.make_safety(sample_state(), desired="simulation", observed="mock")
        ok, reason = safety.jog_joint("left", 0, 1.0)
        self.assertFalse(ok)
        self.assertIn("sim readiness", reason)
        safety.set_desired_mode("real")
        self.assertIn("not reconfigured", safety.status_message)
        self.assertEqual(client.sent_packets, [])

    def test_desired_mode_button_color_marks_active_mode(self):
        self.assertEqual(_mode_button_color("simulation", "simulation"), "green")
        self.assertEqual(_mode_button_color("mock", "simulation"), "gray")

    def test_tcp_step_display_units_convert_to_command_units(self):
        self.assertAlmostEqual(_linear_step_meters(0.1), 0.0001)
        self.assertAlmostEqual(_linear_step_meters(10.0), 0.01)
        self.assertAlmostEqual(_angular_step_radians(0.1), math.radians(0.1))
        self.assertAlmostEqual(_angular_step_radians(10.0), math.radians(10.0))

    def test_compose_sim_readiness_env_unlocks_simulator_lifecycle(self):
        keys = (
            "RB_GUI_SIM_READINESS_READY",
            "RB_GUI_SIM_READINESS_RUNNING",
            "RB_GUI_SIM_READINESS_CONNECTED",
            "RB_GUI_CARTESIAN_AVAILABLE",
        )
        old_values = {key: os.environ.get(key) for key in keys}
        try:
            os.environ["RB_GUI_SIM_READINESS_READY"] = "1"
            os.environ["RB_GUI_SIM_READINESS_RUNNING"] = "1"
            os.environ["RB_GUI_SIM_READINESS_CONNECTED"] = "1"
            os.environ["RB_GUI_CARTESIAN_AVAILABLE"] = "0"
            observed = normalize_observed_mode_backend("simulation", "simulator")
            readiness = _sim_readiness_from_env(observed)
            self.assertTrue(readiness.ready)
            self.assertTrue(readiness.running)
            self.assertTrue(readiness.connected)
            self.assertFalse(readiness.cartesian_available)

            store = StateStore(stale_after_sec=0.5)
            self.assertTrue(store.update_from_json_bytes(json.dumps(sample_state()).encode(), received_monotonic=time.monotonic()))
            safety = OperatorSafety(
                store,
                RecordingClient(),
                desired_mode="simulation",
                observed_server_mode="simulation",
                observed_backend="simulator",
                sim_readiness=readiness,
            )
            states = safety.control_disabled_states()
            self.assertFalse(states["lifecycle:ArmMotion"])
            self.assertFalse(states["jog"])
        finally:
            for key, value in old_values.items():
                if value is None:
                    os.environ.pop(key, None)
                else:
                    os.environ[key] = value

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

    def test_tcp_delta_stand_packet_builder_holds_other_arm(self):
        client = RecordingClient()
        delta = (0.005, 0.0, 0.0, 0.0, 0.0, 0.0)
        packet = client.build_tcp_delta_stand(left_delta=delta)
        self.assertEqual(packet["schema_version"], 1)
        self.assertEqual(packet["source_id"], "rb_gui")
        self.assertTrue(packet["session_id"])
        self.assertEqual(packet["mode"], "Hold")
        self.assertEqual(packet["left"]["mode"], "TcpDeltaStand")
        self.assertEqual(packet["left"]["tcp_delta_stand"], list(delta))
        self.assertEqual(packet["right"], {})

    def test_tcp_delta_local_packet_builder_holds_other_arm(self):
        client = RecordingClient()
        delta = (0.0, 0.0, 0.005, 0.0, 0.0, 0.01)
        packet = client.build_tcp_delta_local(right_delta=delta)
        self.assertEqual(packet["schema_version"], 1)
        self.assertEqual(packet["source_id"], "rb_gui")
        self.assertTrue(packet["session_id"])
        self.assertEqual(packet["mode"], "Hold")
        self.assertEqual(packet["left"], {})
        self.assertEqual(packet["right"]["mode"], "TcpDeltaLocal")
        self.assertEqual(packet["right"]["tcp_delta_local"], list(delta))

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

    def test_tcp_delta_stand_enabled_only_for_simulator_with_fk_and_feature_flag(self):
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

        ok, reason = safety.send_tcp_delta_stand("left", (0.005, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertTrue(ok, reason)
        packet = client.sent_packets[-1]
        self.assertEqual(packet["mode"], "Hold")
        self.assertEqual(packet["source_id"], "rb_gui")
        self.assertTrue(packet["session_id"])
        self.assertEqual(packet["left"]["mode"], "TcpDeltaStand")
        self.assertEqual(packet["left"]["tcp_delta_stand"], [0.005, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.assertEqual(packet["right"], {})
        self.assertIn("server verdict: Ok", reason)

        ok, reason = safety.send_tcp_delta_local("left", (0.0, 0.005, 0.0, 0.0, 0.0, 0.0))
        self.assertTrue(ok, reason)
        packet = client.sent_packets[-1]
        self.assertEqual(packet["mode"], "Hold")
        self.assertEqual(packet["left"]["mode"], "TcpDeltaLocal")
        self.assertEqual(packet["left"]["tcp_delta_local"], [0.0, 0.005, 0.0, 0.0, 0.0, 0.0])
        self.assertEqual(packet["right"], {})
        self.assertIn("TcpDeltaLocal left", reason)

        ok, reason = safety.send_tcp_delta_local("left", (0.006, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertFalse(ok)
        self.assertIn("linear delta exceeds", reason)

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

    def test_tcp_delta_stand_real_mode_disabled_even_with_feature_flag(self):
        _, client, safety = self.make_safety(
            self.tcp_available_state(),
            desired="real",
            observed="real",
            observed_backend="rbpodo",
            sim_ready=True,
            cartesian_available=True,
            enable_tcp_pose=True,
        )
        # Real/sim gating retired: real-mode TCP commands send normally.
        ok, reason = safety.send_tcp_delta_stand("left", (0.005, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertTrue(ok, reason)
        ok, reason = safety.send_tcp_delta_local("left", (0.005, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertTrue(ok, reason)
        ok, reason = safety.send_tcp_linear_move(left_pose=(0.31, 0.12, 0.44, 0.0, 0.0, 0.0), duration_sec=1.0)
        self.assertTrue(ok, reason)
        self.assertFalse(safety.control_disabled_states()["tcp_pose"])
        self.assertFalse(safety.control_disabled_states()["tcp_linear"])
        self.assertTrue(client.sent_packets)

    def test_tcp_delta_stand_requires_simulator_backend_fk_and_readiness(self):
        state = self.tcp_available_state()
        _, _, backend_safety = self.make_safety(
            state,
            desired="simulation",
            observed="simulation",
            observed_backend="mock",
            sim_ready=True,
            cartesian_available=True,
            enable_tcp_pose=True,
        )
        # Backend requirement retired: any backend may issue TCP commands.
        self.assertIsNone(backend_safety.tcp_command_disabled_reason("left"))

        no_fk = sample_state()
        _, _, fk_safety = self.make_safety(
            no_fk,
            desired="simulation",
            observed="simulation",
            observed_backend="simulator",
            sim_ready=True,
            cartesian_available=True,
            enable_tcp_pose=True,
        )
        self.assertIn("FK/TCP pose unavailable", fk_safety.tcp_command_disabled_reason("left"))
        ok, reason = fk_safety.send_tcp_linear_move(left_pose=(0.31, 0.12, 0.44, 0.0, 0.0, 0.0), duration_sec=1.0)
        self.assertFalse(ok)
        self.assertIn("FK/TCP pose unavailable", reason)

        _, _, cart_safety = self.make_safety(
            state,
            desired="simulation",
            observed="simulation",
            observed_backend="simulator",
            sim_ready=True,
            cartesian_available=False,
            enable_tcp_pose=True,
        )
        self.assertIn("cartesian readiness", cart_safety.tcp_command_disabled_reason("left"))

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
        ok, _ = allowed.send_tcp_delta_stand("left", (0.005, 0.0, 0.0, 0.0, 0.0, 0.0))
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

    def test_tcp_twist_and_joint_velocity_in_controller_sim(self):
        state = self.tcp_available_state()
        _, client, safety = self.make_safety(
            state,
            desired="simulation",
            observed="simulation",
            observed_backend="rbpodo",
            sim_ready=True,
            cartesian_available=True,
            enable_tcp_pose=True,
            enable_controller_sim_cartesian=True,
        )
        # TcpTwistStand within velocity limit -> sent.
        ok, msg = safety.send_tcp_twist_stand("left", (0.02, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertTrue(ok, msg)
        self.assertEqual(client.sent_packets[-1]["left"]["mode"], "TcpTwistStand")
        self.assertIn("tcp_twist_stand", client.sent_packets[-1]["left"])
        # TcpTwistLocal too -> sent.
        ok, msg = safety.send_tcp_twist_local("left", (0.0, 0.0, 0.01, 0.0, 0.0, 0.0))
        self.assertTrue(ok, msg)
        self.assertEqual(client.sent_packets[-1]["left"]["mode"], "TcpTwistLocal")
        # Over the linear velocity limit -> rejected, nothing sent.
        before = len(client.sent_packets)
        ok, msg = safety.send_tcp_twist_stand("left", (99.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertFalse(ok)
        self.assertIn("velocity exceeds", msg)
        self.assertEqual(len(client.sent_packets), before)
        # JointVelocity within limit -> sent.
        ok, msg = safety.send_joint_velocity("left", (5.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertTrue(ok, msg)
        self.assertEqual(client.sent_packets[-1]["left"]["mode"], "JointVelocity")
        self.assertIn("dq_target_deg_s", client.sent_packets[-1]["left"])
        # JointVelocity over limit -> rejected.
        ok, msg = safety.send_joint_velocity("left", (999.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertFalse(ok)
        self.assertIn("velocity exceeds", msg)

    def test_tcp_twist_blocked_in_real_mode(self):
        # Real/sim gating retired: twist and joint-velocity send in real mode.
        state = self.tcp_available_state()
        _, client, safety = self.make_safety(
            state,
            desired="real",
            observed="real",
            observed_backend="rbpodo",
            sim_ready=True,
            cartesian_available=True,
            enable_tcp_pose=True,
            enable_controller_sim_cartesian=True,
        )
        ok, msg = safety.send_tcp_twist_stand("left", (0.02, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertTrue(ok, msg)
        ok, msg = safety.send_joint_velocity("left", (5.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertTrue(ok, msg)

    def test_tcp_circle_move_in_controller_sim(self):
        state = self.tcp_available_state()
        _, client, safety = self.make_safety(
            state,
            desired="simulation",
            observed="simulation",
            observed_backend="rbpodo",
            sim_ready=True,
            cartesian_available=True,
            enable_tcp_pose=True,
            enable_controller_sim_cartesian=True,
        )
        ok, msg = safety.send_tcp_circle_move(0.15, 4.0, arm="both")
        self.assertTrue(ok, msg)
        pkt = client.sent_packets[-1]
        self.assertEqual(pkt["mode"], "TcpCircleMove")
        self.assertEqual(pkt["left"]["mode"], "TcpCircleMove")
        self.assertEqual(pkt["left"]["diameter_m"], 0.15)
        self.assertEqual(pkt["left"]["period_sec"], 4.0)
        self.assertEqual(pkt["right"]["diameter_m"], 0.15)
        ok, msg = safety.send_tcp_circle_move(0.5, 4.0)
        self.assertFalse(ok)
        self.assertIn("diameter", msg)
        ok, msg = safety.send_tcp_circle_move(0.15, 1.0)
        self.assertFalse(ok)
        self.assertIn("period", msg)
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
        ok, msg = real_safety.send_tcp_circle_move(0.15, 4.0)
        self.assertTrue(ok, msg)

    def test_tcp_delta_stand_blocks_stale_and_faulted_state(self):
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
        ok, reason = stale_safety.send_tcp_delta_stand("left", (0.005, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertFalse(ok)
        self.assertIn("state stream", reason)
        ok, reason = stale_safety.send_tcp_delta_local("left", (0.005, 0.0, 0.0, 0.0, 0.0, 0.0))
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
        ok, reason = fault_safety.send_tcp_delta_stand("left", (0.005, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertFalse(ok)
        self.assertIn("fault", reason)
        ok, reason = fault_safety.send_tcp_delta_local("left", (0.005, 0.0, 0.0, 0.0, 0.0, 0.0))
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

    def test_operator_monitor_static_html_places_cards_on_left(self):
        html = _operator_monitor_static_html(18.0, 1.0)
        self.assertIn("--rb-monitor-gap: 1.000em;", html)
        self.assertIn("--rb-monitor-target-width: 18.000em;", html)
        self.assertIn("calc((100vw - (3 * var(--rb-monitor-gap))) / 2)", html)
        self.assertIn(".rb-monitor-joint-card { left: var(--rb-monitor-gap); }", html)
        self.assertIn(".rb-monitor-stand-card { left: calc((2 * var(--rb-monitor-gap)) + var(--rb-monitor-width)); }", html)
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
        self.assertIn("J1 base_joint", html)
        self.assertIn("0.00 deg", html)
        self.assertIn("0.0000 rad", html)
        self.assertIn("live, xyz=mm, tick=1", html)
        self.assertIn("310.0 mm", html)
        self.assertIn("90.00 deg", html)
        self.assertIn("1.5708 rad", html)

    def test_operator_monitor_dynamic_html_marks_unavailable_pose_invalid(self):
        store, _, _ = self.make_safety(sample_state())
        html = _operator_monitor_dynamic_html(store.latest(), stale=False)
        self.assertIn("live, xyz=mm, tick=1", html)
        self.assertIn("invalid", html)

        state = self.tcp_available_state()
        store, _, _ = self.make_safety(state)
        stale_html = _operator_monitor_dynamic_html(store.latest(), stale=True)
        self.assertIn("stale, xyz=mm, tick=1", stale_html)
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

    def test_update_operator_monitors_updates_html_content_handle(self):
        server = RecordingServer()
        handles = {}
        _build_operator_monitors(server, handles)
        state = self.tcp_available_state()
        store, _, _ = self.make_safety(state)
        _update_operator_monitors(handles, store.latest(), stale=False)
        self.assertIn("live, tick=1", handles["operator_monitor_content"].content)
        self.assertIn("live, xyz=mm, tick=1", handles["operator_monitor_content"].content)

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
        self.assertGreaterEqual(len(scene.point_clouds), 4)
        self.assertGreaterEqual(len(scene.line_segments), 1)

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
        self.assertTrue(_apply_tcp_delta_to_target(stand_handles, "left", (1.0, 0.0, 0.0, 0.0, 0.0, 0.0), _TCP_FRAME_STAND))
        self.assertAlmostEqual(stand_handles["left_tcp_target_pose"][0], 1.0, places=7)
        self.assertAlmostEqual(stand_handles["left_tcp_target_pose"][1], 0.0, places=7)

        local_handle = RecordingSceneHandle()
        local_handles = {
            "left_tcp_target": local_handle,
            "left_tcp_target_pose": (0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2.0),
            "left_tcp_target_wxyz": _pose_wxyz((0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2.0)),
        }
        self.assertTrue(_apply_tcp_delta_to_target(local_handles, "left", (1.0, 0.0, 0.0, 0.0, 0.0, 0.0), _TCP_FRAME_LOCAL))
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

        ok, reason = _apply_tcp_delta_and_send_pose_target(
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
        self.assertNotIn("tcp_delta_local", packet["left"])
        self.assertNotIn("tcp_delta_stand", packet["left"])
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
        self.assertNotIn("tcp_delta_local", packet["left"])
        self.assertEqual(packet["left"]["target_tcp_stand"]["quaternion_xyzw"], list(_wxyz_to_xyzw(handles["left_tcp_target_wxyz"])))

    def test_tcp_frame_defaults_to_local_and_updates_button_colors(self):
        handles = {
            "tcp_frame_buttons": {
                _TCP_FRAME_STAND: RecordingButton(),
                _TCP_FRAME_LOCAL: RecordingButton(),
            }
        }
        self.assertEqual(_tcp_frame_mode(handles), _TCP_FRAME_LOCAL)
        _update_tcp_frame_buttons(handles)
        self.assertEqual(handles["tcp_frame_buttons"][_TCP_FRAME_STAND].color, "gray")
        self.assertEqual(handles["tcp_frame_buttons"][_TCP_FRAME_LOCAL].color, "green")

        handles["tcp_frame_mode"] = _TCP_FRAME_STAND
        _update_tcp_frame_buttons(handles)
        self.assertEqual(handles["tcp_frame_buttons"][_TCP_FRAME_STAND].color, "green")
        self.assertEqual(handles["tcp_frame_buttons"][_TCP_FRAME_LOCAL].color, "gray")

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
        self.assertFalse(mock_states["jog"])
        self.assertFalse(mock_states["lifecycle:ArmMotion"])
        self.assertTrue(mock_states["tcp_jog"])
        self.assertTrue(mock_states["tcp_pose"])
        self.assertTrue(mock_states["tcp_linear"])

        _, _, sim_safety = self.make_safety(sample_state(), desired="simulation", observed="simulation", sim_ready=False)
        sim_states = sim_safety.control_disabled_states()
        self.assertTrue(sim_states["jog"])
        self.assertTrue(sim_states["lifecycle:ArmMotion"])

    def test_lifecycle_packets_match_existing_udp_protocol(self):
        _, client, safety = self.make_safety(sample_state())
        ok, reason = safety.send_lifecycle("EmergencyStop")
        self.assertTrue(ok, reason)
        self.assertEqual(client.sent_packets[-1]["mode"], "EmergencyStop")
        self.assertEqual(client.sent_packets[-1]["left"], {})
        self.assertEqual(client.sent_packets[-1]["right"], {})

    def test_compose_gui_env_uses_servo_server_and_canonical_simulation_terms(self):
        compose = Path(__file__).resolve().parents[2] / "docker-compose.yml"
        text = compose.read_text(encoding="utf-8")
        self.assertIn('RB_GUI_COMMAND_HOST: "rb_servo_server"', text)
        self.assertIn('RB_GUI_OBSERVED_MODE: "simulation"', text)
        self.assertIn('RB_GUI_OBSERVED_BACKEND: "simulator"', text)
        self.assertIn('RB_GUI_SIM_READINESS_READY: "1"', text)
        self.assertIn('RB_GUI_SIM_READINESS_CONNECTED: "1"', text)
        self.assertIn('RB_GUI_CARTESIAN_AVAILABLE: "1"', text)
        self.assertIn('RB_GUI_ENABLE_TCP_POSE_COMMANDS: "1"', text)
        self.assertIn('RB_GUI_INIT_LEFT_JOINTS: "-124.660, 32.485, 119.074, -96.294, -81.798, -30.615"', text)
        self.assertIn('RB_GUI_INIT_RIGHT_JOINTS: "111.949, -49.304, -120.057, 75.305, 87.436, 49.983"', text)
        self.assertIn('RB_GUI_INIT_MOTION_TIMEOUT_SEC: "10.0"', text)
        self.assertNotIn('RB_GUI_OBSERVED_MODE: "rbsim_local"', text)

        config = Path(__file__).resolve().parents[2] / "rb_servo_server" / "config" / "dual_simulator_compose.yaml"
        config_text = config.read_text(encoding="utf-8")
        self.assertIn("provider: pinocchio", config_text)
        self.assertIn("publish_tcp: true", config_text)
        self.assertIn("orientation_tolerance_rad: 0.005", config_text)
        self.assertIn("allow_in_simulation: true", config_text)
        self.assertIn("allow_in_real: false", config_text)

        dockerfile = Path(__file__).resolve().parents[2] / "scripts" / "docker" / "rb_servo_server.hardware_free.Dockerfile"
        docker_text = dockerfile.read_text(encoding="utf-8")
        self.assertIn("robotpkg-pinocchio", docker_text)
        self.assertIn("-DRB_SERVO_ENABLE_PINOCCHIO=ON", docker_text)

        cmake = Path(__file__).resolve().parents[2] / "rb_servo_server" / "CMakeLists.txt"
        cmake_text = cmake.read_text(encoding="utf-8")
        self.assertIn('option(RB_SERVO_ENABLE_PINOCCHIO "Enable Pinocchio FK/IK support" ON)', cmake_text)


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
        self.assertIn("floor: ON z=10mm", text)
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
            self.assertIn("    z_min_m: 0.050   # startup default\n", updated)
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


if __name__ == "__main__":
    unittest.main()
