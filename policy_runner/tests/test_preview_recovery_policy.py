"""Shared preview recovery exercises the real dispatcher without device I/O."""
from __future__ import annotations

import copy
import threading
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np

from policy_runner.flow_inference import FlowMatchingActionSource
from policy_runner.openpi_remote import OpenpiRemoteActionSource
from policy_runner.robot_state_client import StateSnapshot
from policy_runner.servo_command_client import CommandIntent
from test_chunk_activation_scheduler import OfflineStreamSource
from test_preview_execution_policy import capability, source as gripper_source


def state(now, phase="tracking", epoch=0, minimum=0, *, left=True, right=True):
    cap = capability()
    cap["preview_recovery"] = True
    stamp = round(now * 1e9)
    return StateSnapshot({
        "chunk_execution_profiles": [cap],
        "preview_recovery": {"enabled": True, "state": phase, "epoch": epoch,
                             "min_observation_time_ns": minimum, "sample_time_ns": stamp,
                             "reason": "test", "attempts": epoch},
        "preview_execution": {
            arm: {"enabled": True, "active": active,
                  "status": "active" if active else "waiting", "sample_time_ns": stamp}
            for arm, active in (("left", left), ("right", right))},
    }, now)


class RecoverySource(OfflineStreamSource):
    def __init__(self):
        super().__init__("ready_event")
        self.tcp_target_profile = "flow_infer_preview"
        self.enable_async_chunking = True
        self._last_server_motion_epoch = None
        self.arm_mask = np.ones(2)

    def _emit_step_intent(self, step, payload, gripper_targets):
        self.emitted.append(step.copy())
        return CommandIntent("TcpPoseTarget", left={"tcp_target_stand": [1], "gripper_target": 4},
                             right={"tcp_target_stand": [2], "gripper_target": 8},
                             tcp_target_profile="flow_infer_preview")

    def run(self, now, phase="tracking", epoch=0, minimum=0):
        self.now = now
        return self.next_intent(state(now, phase, epoch, minimum), now)

    def ready(self, *, observation, epoch=1, row=100):
        self.complete_request(self.now, rows=self.rows(row))
        self._stream_next_chunk_metadata.update(
            observation_time_ns=observation, preview_recovery_epoch=epoch)


class PreviewRecoveryPolicyTest(unittest.TestCase):
    def started(self):
        src = RecoverySource()
        src.run(1.0)
        src.ready(observation=1_005_000_000, epoch=0, row=0)
        src.run(1.01)
        return src

    def test_both_arms_freeze_then_one_candidate_then_row_zero_after_shared_resume(self):
        src = self.started()
        old_intent = copy.deepcopy(src._current_step_intent)
        generation = src._stream_generation
        emitted = src._stream_emitted_policy_steps
        published = len(src.published)
        grip_rows = len(src.gripper_rows)
        heartbeat = src.run(1.02, "braking", 1)
        self.assertEqual(heartbeat.mode, "TcpPoseTarget")
        self.assertEqual(heartbeat.left["tcp_target_stand"], old_intent.left["tcp_target_stand"])
        self.assertNotIn("gripper_target", heartbeat.left)
        self.assertNotIn("gripper_target", heartbeat.right)
        self.assertIsNone(src._chunk)
        self.assertIsNone(src._stream_request)
        self.assertGreater(src._stream_generation, generation)
        src.run(1.03, "braking", 1)
        self.assertIsNone(src._stream_request)
        src.run(1.04, "waiting_fresh", 1, 1_030_000_000)
        self.assertEqual(src._stream_request[3]["preview_recovery"]["epoch"], 1)
        src.ready(observation=1_035_000_000)
        src.run(1.05, "waiting_fresh", 1, 1_030_000_000)
        src.run(1.06, "waiting_fresh", 1, 1_030_000_000)
        src.run(1.08, "starting", 1, 1_030_000_000)
        self.assertEqual(len(src.published), published + 1)
        self.assertEqual(src._stream_emitted_policy_steps, emitted)
        self.assertEqual(len(src.gripper_rows), grip_rows)
        self.assertIsNone(src._stream_request)
        src.run(1.10, "tracking", 1, 1_030_000_000)
        self.assertEqual(src._stream_emitted_policy_steps, emitted + 1)
        np.testing.assert_array_equal(src.emitted[-1], np.full(14, 100))

    def test_new_epoch_discards_published_candidate_and_paused_never_advances(self):
        src = self.started()
        src.run(1.02, "braking", 1)
        src.run(1.04, "waiting_fresh", 1, 1_030_000_000)
        src.ready(observation=1_035_000_000)
        src.run(1.05, "waiting_fresh", 1, 1_030_000_000)
        emitted = src._stream_emitted_policy_steps
        src.run(1.06, "paused", 2)
        src.run(1.20, "paused", 2)
        self.assertIsNone(src._chunk)
        self.assertIsNone(src._stream_request)
        self.assertFalse(src._preview_recovery_resume_row_pending)
        self.assertEqual(src._stream_emitted_policy_steps, emitted)

    def test_recovery_never_invents_an_initial_tcp_heartbeat(self):
        src = RecoverySource()
        self.assertIsNone(src.run(1.0, "braking", 1))
        self.assertIsNone(src._stream_request)

    def paused_command_snapshot(self):
        snap = state(1.0, "paused", 1)
        snap.payload.update(loop_start_time_ns=1_000_000_000, tick=8, motion_epoch=2)
        for arm, x in (("left", 0.2), ("right", 0.4)):
            snap.payload[arm] = {
                "tcp_command_stand": {"x": x, "y": 0.0, "z": 0.3,
                                      "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
                "tcp_actual_stand": {"x": 99.0, "y": 99.0, "z": 99.0,
                                     "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
            }
        return snap

    def test_new_policy_session_can_introduce_itself_with_verified_command_fk(self):
        src = RecoverySource()
        snap = self.paused_command_snapshot()
        heartbeat = src.next_intent(snap, 1.0)
        self.assertEqual(heartbeat.mode, "TcpPoseTarget")
        self.assertTrue(heartbeat.is_motion)
        self.assertEqual(heartbeat.tcp_target_profile, "flow_infer_preview")
        for arm, x in (("left", 0.2), ("right", 0.4)):
            target = getattr(heartbeat, arm)
            self.assertEqual(target["tcp_target_stand"]["x"], x)
            self.assertNotIn("gripper_target", target)
        self.assertEqual(heartbeat.metadata["source_conditioning_mode"], "preview_recovery_reference")
        self.assertIsNone(src._stream_request)
        self.assertIsNone(src._current_step_intent)
        self.assertEqual(src._stream_emitted_policy_steps, 0)

    def test_restart_heartbeat_requires_both_command_poses_and_fresh_logical_sample(self):
        cases = []
        for value in (None, {}, {"x": 1, "y": 2, "z": 3, "rx": 0, "ry": 0, "rz": 0},
                      {"x": 1, "y": 2, "z": 3, "quaternion_xyzw": [0, 0, 0, 0]},
                      {"x": float("nan"), "y": 2, "z": 3, "quaternion_xyzw": [0, 0, 0, 1]}):
            snap = self.paused_command_snapshot()
            snap.payload["right"]["tcp_command_stand"] = value
            cases.append(snap)
        for key, value in (("loop_start_time_ns", 1), ("loop_start_time_ns", 1_100_000_000),
                           ("tick", None), ("motion_epoch", True), ("fault_latched", True),
                           ("send_suppressed", True)):
            snap = self.paused_command_snapshot()
            snap.payload[key] = value
            cases.append(snap)
        for snap in cases:
            src = RecoverySource()
            self.assertIsNone(src.next_intent(snap, 1.0))
            self.assertIsNone(src._stream_request)

    def test_old_epoch_old_generation_equal_barrier_missing_and_future_camera_rejected(self):
        for edits in ({"preview_recovery_epoch": 0}, {"generation": -1},
                      {"observation_time_ns": 1_030_000_000}, {"observation_time_ns": 0},
                      {"observation_time_ns": 1_090_000_000}):
            with self.subTest(edits=edits):
                src = self.started()
                src.run(1.02, "braking", 1)
                src.run(1.04, "waiting_fresh", 1, 1_030_000_000)
                src.ready(observation=1_035_000_000)
                src._stream_next_chunk_metadata.update(edits)
                published = len(src.published)
                src.run(1.05, "waiting_fresh", 1, 1_030_000_000)
                self.assertEqual(len(src.published), published)
                self.assertIsNone(src._chunk)
                self.assertIsNotNone(src._stream_request)

    def test_shared_recovery_blocks_both_grippers_and_requires_both_active_after_resume(self):
        src = gripper_source()
        for phase, left, right, expected in (("braking", True, True, False),
                                           ("waiting_fresh", True, True, False),
                                           ("starting", True, True, False),
                                           ("paused", True, True, False),
                                           ("tracking", True, False, False),
                                           ("tracking", True, True, True)):
            snap = state(1.0, phase, 1, 900_000_000, left=left, right=right)
            src._preview_recovery_enabled = True
            src._preview_recovery_state = phase
            src._update_preview_gripper_authority(snap, 1.01)
            self.assertEqual(src._preview_gripper_arm_allowed("left"), expected)
            self.assertEqual(src._preview_gripper_arm_allowed("right"), expected)
            src._dispatch_gripper_step(np.zeros(14))
            if not expected:
                self.assertEqual(src.gripper_runtime.dispatch.call_args.args[0], [])
            src.gripper_runtime.dispatch.reset_mock()

    def test_camera_guard_early_return_cannot_reuse_prior_gripper_authority(self):
        src = OpenpiRemoteActionSource.__new__(OpenpiRemoteActionSource)
        src.tcp_target_profile = "flow_infer_preview"
        src._preview_recovery_enabled = True
        src._preview_recovery_epoch = 1
        src._preview_recovery_state = "tracking"
        src._preview_gripper_allowed = {"left": True, "right": True}
        hold = CommandIntent.gripper_target(left=7.0, right=42.0)
        with mock.patch.object(src, "_invalidate_policy_chunks"), \
                mock.patch.object(src, "_clear_target_pose_state"), \
                mock.patch.object(src, "reset_rtc"), \
                mock.patch.object(src, "_handle_server_motion_epoch"), \
                mock.patch.object(src, "_camera_runtime_gate", return_value=(True, hold)), \
                mock.patch.object(FlowMatchingActionSource, "next_intent") as dispatcher:
            for phase in ("braking", "waiting_fresh", "starting", "paused"):
                result = src.next_intent(state(1.0, phase, 1, 900_000_000), 1.0)
                self.assertEqual(result.mode, hold.mode)
                self.assertNotIn("gripper_target", result.left)
                self.assertNotIn("gripper_target", result.right)
            dispatcher.assert_not_called()

    def test_nonpreview_profile_does_not_inherit_an_old_recovery_gripper_gate(self):
        src = gripper_source()
        src._preview_recovery_enabled = True
        src._preview_recovery_state = "paused"
        src.tcp_target_profile = "flow_infer_fresh"
        for arm in ("left", "right"):
            self.assertTrue(src._preview_gripper_arm_allowed(arm))

    def test_motion_epoch_interrupting_candidate_reopens_inference_and_drops_old_heartbeat(self):
        src = self.started()
        src._last_server_motion_epoch = 2
        src.run(1.02, "braking", 1)
        src.run(1.04, "waiting_fresh", 1, 1_030_000_000)
        src.ready(observation=1_035_000_000)
        src.run(1.05, "waiting_fresh", 1, 1_030_000_000)
        self.assertTrue(src._preview_recovery_candidate_published)
        old_generation = src._stream_generation
        old_emitted = src._stream_emitted_policy_steps
        snap = state(1.06, "waiting_fresh", 1, 1_030_000_000)
        snap.payload["motion_epoch"] = 3
        # Deliberately omit server command FK: an external lifecycle change
        # must discard the old cached heartbeat, without guessing a new one.
        self.assertIsNone(src.next_intent(snap, 1.06))
        self.assertIsNone(src._preview_recovery_heartbeat)
        self.assertFalse(src._preview_recovery_candidate_published)
        self.assertFalse(src._preview_recovery_resume_row_pending)
        self.assertIsNone(src._chunk)
        self.assertIsNotNone(src._stream_request)
        self.assertGreater(src._stream_generation, old_generation)
        self.assertEqual(src._stream_emitted_policy_steps, old_emitted)

    def test_malformed_missing_or_stale_shared_telemetry_fails_before_advancement(self):
        edits = ({"epoch": True}, {"epoch": -1}, {"state": "unknown"},
                 {"state": []}, {"state": {}},
                 {"min_observation_time_ns": None}, {"sample_time_ns": 1},
                 {"sample_time_ns": 2_000_000_000}, {"enabled": False})
        for edit in edits:
            src = RecoverySource()
            snap = state(1.0)
            snap.payload["preview_recovery"].update(edit)
            with self.subTest(edit=edit), self.assertRaisesRegex(ValueError, "preview recovery telemetry"):
                src.next_intent(snap, 1.0)
            self.assertIsNone(src._stream_request)
        src = RecoverySource()
        snap = state(1.0)
        del snap.payload["preview_recovery"]
        with self.assertRaises(ValueError):
            src.next_intent(snap, 1.0)

    def test_recovery_reader_itself_validates_age_bound_and_receive_clock(self):
        for bad in (None, True, "0.05", 0, -1, float("nan"), float("inf")):
            src = RecoverySource()
            snap = state(1.0)
            snap.payload["chunk_execution_profiles"][0]["gripper_state_max_age_sec"] = bad
            with self.subTest(max_age=bad), self.assertRaisesRegex(ValueError, "preview recovery telemetry"):
                src._update_preview_recovery(snap, 1.0)
        for bad in (None, True, "1.0", float("nan"), float("inf")):
            for field in ("received", "now"):
                src = RecoverySource()
                valid = state(1.0)
                snap = StateSnapshot(valid.payload, bad if field == "received" else 1.0)
                with self.subTest(field=field, value=bad), self.assertRaisesRegex(ValueError, "preview recovery telemetry"):
                    src._update_preview_recovery(snap, bad if field == "now" else 1.0)

    def test_old_inflight_result_cannot_clear_new_epoch_pending_work(self):
        src = self.started()
        entered, finish_old, entered_new, finish_new = [threading.Event() for _ in range(4)]
        def sample(payload):
            if payload["preview_recovery"]["epoch"] == 0:
                entered.set()
                self.assertTrue(finish_old.wait(2.0))
            else:
                entered_new.set()
                self.assertTrue(finish_new.wait(2.0))
            return src.rows(999)
        src._sample_and_align_chunk = sample
        worker = threading.Thread(target=src._stream_worker, daemon=True)
        worker.start()
        try:
            self.assertTrue(entered.wait(2.0))
            src.run(1.02, "braking", 1)
            src.run(1.04, "waiting_fresh", 1, 1_030_000_000)
            finish_old.set()
            self.assertTrue(entered_new.wait(2.0))
            self.assertEqual(src.stale_completions, 1)
            self.assertIsNone(src._stream_next_chunk)
            self.assertTrue(src._stream_pending)
            self.assertIsNone(src._chunk)
        finally:
            with src._stream_cv:
                src._stream_shutdown = True
                src._stream_cv.notify_all()
            finish_old.set()
            finish_new.set()
            worker.join(2.0)
            self.assertFalse(worker.is_alive())

    def test_metadata_uses_request_epoch_not_mutable_current_epoch(self):
        src = RecoverySource()
        src._preview_recovery_epoch = 9
        src._last_obs_camera_observation_time_ns = 123
        metadata = src._inference_recovery_metadata(state(1.0, epoch=3).payload)
        self.assertEqual(metadata, {"preview_recovery_epoch": 3, "observation_time_ns": 123})


class RecoveryCameraTimestampTest(unittest.TestCase):
    def source_and_bundle(self):
        src = OpenpiRemoteActionSource.__new__(OpenpiRemoteActionSource)
        src.camera_names = ["left.color", "right.color"]
        bundle = SimpleNamespace(bundle_time_ns=999, received_monotonic=99.0, frames={
            "left.color": SimpleNamespace(host_arrival_time_ns=1_040_000_000),
            "right.color": SimpleNamespace(host_arrival_time_ns=1_020_000_000),
        })
        return src, bundle

    def test_earliest_required_frame_maps_raw_to_monotonic(self):
        src, bundle = self.source_and_bundle()
        with mock.patch("policy_runner.openpi_remote.time.monotonic", return_value=3.05), \
                mock.patch("policy_runner.openpi_remote.time.clock_gettime_ns", return_value=1_050_000_000):
            self.assertEqual(src._camera_recovery_observation_time_ns(bundle), 3_020_000_000)
            src.include_depth = True
            src.depth_camera_names = ["left.depth", "right.depth"]
            self.assertIsNone(src._camera_recovery_observation_time_ns(bundle))

    def test_missing_or_invalid_capture_stamp_never_uses_late_receipt(self):
        src, bundle = self.source_and_bundle()
        for bad in (0, None, True, 1.02):
            bundle.frames["right.color"].host_arrival_time_ns = bad
            self.assertIsNone(src._camera_recovery_observation_time_ns(bundle))
        bundle.frames["right.color"].host_arrival_time_ns = 100_000_000_000
        with mock.patch("policy_runner.openpi_remote.time.monotonic", return_value=3.05), \
                mock.patch("policy_runner.openpi_remote.time.clock_gettime_ns", return_value=1_050_000_000):
            self.assertIsNone(src._camera_recovery_observation_time_ns(bundle))

    def test_openpi_rejects_pre_stop_images_before_proprio_or_model_request(self):
        src = OpenpiRemoteActionSource.__new__(OpenpiRemoteActionSource)
        src._last_inference_camera_diagnostics = {}
        for stamp in (None, 0, 1_020_000_000, 1_030_000_000):
            src._last_obs_camera_observation_time_ns = stamp
            with mock.patch.object(src, "_raw_camera_images", return_value=({"left": [], "right": []}, 2, 0)), \
                    mock.patch.object(src, "_proprio_state") as proprio:
                self.assertIsNone(src._sample_chunk(state(1.04, "waiting_fresh", 1, 1_030_000_000).payload))
                proprio.assert_not_called()
                self.assertEqual(src._last_inference_camera_diagnostics["outcome"],
                                 "preview_recovery_observation_before_stop")


if __name__ == "__main__":
    unittest.main()
