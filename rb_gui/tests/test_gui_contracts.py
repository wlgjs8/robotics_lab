from __future__ import annotations

import json
import math
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rb_servo_gui.app import _format_fk_status, _format_joints, _joint_cfg_radians, _joint_marker_position, _mount_position, _pose6_from_mounts, _pose_wxyz, update_scene_markers
from rb_servo_gui.command_client import CommandClient
from rb_servo_gui.models import Pose6D
from rb_servo_gui.safety import OperatorSafety, Readiness, normalize_observed_mode_backend
from rb_servo_gui.state_receiver import StateStore


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
        "has_valid_tcp_pose": False,
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
        self.visible = None


class RecordingUrdf:
    def __init__(self):
        self.configs = []

    def update_cfg(self, config):
        self.configs.append(tuple(float(value) for value in config))


class GuiContractsTest(unittest.TestCase):
    def make_safety(self, state=None, *, desired="mock", observed="mock", observed_backend=None, sim_ready=False, enable_tcp_pose=False):
        store = StateStore(stale_after_sec=0.5)
        if state is not None:
            self.assertTrue(store.update_from_json_bytes(json.dumps(state).encode(), received_monotonic=time.monotonic()))
        client = RecordingClient()
        safety = OperatorSafety(
            store,
            client,
            desired_mode=desired,
            observed_server_mode=observed,
            observed_backend=observed_backend,
            sim_readiness=Readiness(running=True, connected=True, ready=sim_ready, no_go_reason="sim readiness not proven"),
            enable_tcp_pose_commands=enable_tcp_pose,
        )
        return store, client, safety

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
        state["left"]["tcp_stand"] = {"x": 0.31, "y": 0.12, "z": 0.44, "rx": 0.0, "ry": 0.0, "rz": math.pi}
        state["left"]["tcp_base"] = [0.1, 0.2, 0.3, 0.0, 0.0, 1.0]
        state["left"]["has_valid_tcp_pose"] = True
        state["left"]["tcp_deferred"] = False
        store, _, _ = self.make_safety(state)
        latest = store.latest()
        self.assertIsInstance(latest.left.tcp_stand, Pose6D)
        self.assertEqual(latest.left.tcp_stand.as_tuple(), (0.31, 0.12, 0.44, 0.0, 0.0, math.pi))
        self.assertEqual(latest.left.tcp_base.as_tuple(), (0.1, 0.2, 0.3, 0.0, 0.0, 1.0))
        self.assertTrue(latest.left.has_valid_tcp_pose)
        self.assertFalse(latest.left.tcp_deferred)

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
        self.assertEqual(packet["left"]["q_target_deg"], [2.0, -30.0, 80.0, 0.0, 60.0, 0.0])
        self.assertEqual(packet["right"]["q_target_deg"], [0.0, -30.0, 80.0, 0.0, 60.0, 0.0])
        self.assertGreater(packet["timeout_sec"], 0.0)

    def test_real_mode_blocks_lifecycle_and_motion(self):
        _, client, safety = self.make_safety(sample_state(), desired="real", observed="real")
        ok, reason = safety.send_lifecycle("ArmMotion")
        self.assertFalse(ok)
        self.assertIn("connect/status only", reason)
        ok, reason = safety.jog_joint("left", 0, 1.0)
        self.assertFalse(ok)
        self.assertEqual(client.sent_packets, [])

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
        _, client, safety = self.make_safety(sample_state(), desired="simulation", observed="mock")
        ok, reason = safety.jog_joint("left", 0, 1.0)
        self.assertFalse(ok)
        self.assertIn("desired mode differs", reason)
        safety.set_desired_mode("real")
        self.assertIn("not reconfigured", safety.status_message)
        self.assertEqual(client.sent_packets, [])

    def test_tcp_jog_never_sends_cartesian_motion(self):
        _, client, safety = self.make_safety(sample_state())
        ok, reason = safety.tcp_jog_unavailable()
        self.assertFalse(ok)
        self.assertIn("no Cartesian motion command", reason)
        self.assertEqual(client.sent_packets, [])

    def test_tcp_pose_target_is_disabled_until_feature_flag(self):
        _, client, safety = self.make_safety(sample_state())
        pose = (0.3, 0.1, 0.4, 0.0, 0.0, 0.0)
        ok, reason = safety.send_tcp_pose_target(left_pose=pose)
        self.assertFalse(ok)
        self.assertIn("TCP pose command disabled", reason)
        self.assertEqual(client.sent_packets, [])

    def test_tcp_pose_target_packet_builder_uses_gizmo_pose_and_holds_other_arm(self):
        client = RecordingClient()
        pose = (0.3, 0.1, 0.4, 0.0, 0.0, 0.0)
        packet = client.build_tcp_pose_target(left_pose=pose)
        self.assertEqual(packet["mode"], "Hold")
        self.assertEqual(packet["left"]["mode"], "TcpPoseTarget")
        self.assertEqual(packet["left"]["tcp_target_stand"], list(pose))
        self.assertEqual(packet["right"], {})


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
        cfg = _joint_cfg_radians((0.0, 90.0, -90.0))
        self.assertEqual(len(cfg), 6)
        self.assertAlmostEqual(cfg[1], math.pi / 2)
        self.assertAlmostEqual(cfg[2], -math.pi / 2)
        self.assertEqual(cfg[3:], (0.0, 0.0, 0.0))
        self.assertEqual(_format_joints(None), "invalid")
        self.assertEqual(_joint_cfg_radians(None), (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

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

    def test_visual_disabled_state_matches_safety_blocks(self):
        _, _, real_safety = self.make_safety(sample_state(), desired="real", observed="real")
        real_states = real_safety.control_disabled_states()
        self.assertTrue(real_states["jog"])
        self.assertTrue(real_states["tcp_jog"])
        self.assertTrue(real_states["lifecycle:ArmMotion"])
        self.assertTrue(real_states["lifecycle:Hold"])

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
        self.assertIn('RB_GUI_ENABLE_TCP_POSE_COMMANDS: "0"', text)
        self.assertNotIn('RB_GUI_OBSERVED_MODE: "rbsim_local"', text)


if __name__ == "__main__":
    unittest.main()
