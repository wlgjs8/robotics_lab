"""Preview capability and gripper readiness tests; no device or model I/O."""
from __future__ import annotations

import copy
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np

from policy_runner.flow_inference import FlowMatchingActionSource
from policy_runner.robot_state_client import StateSnapshot
from policy_runner.servo_command_client import CommandIntent


def capability():
    return {
        "name": "flow_infer_preview", "enabled": True, "controller": "delta_preview",
        "fresh_chunk_replan": True, "continuous_hold_resume": True,
        "preview_execution": True, "gripper_state_max_age_sec": 0.05,
    }


def snapshot(*, left=True, right=True, stamp=1_000_000_000, received=1.0):
    def arm(active):
        return {"enabled": True, "active": active, "status": "active" if active else "waiting",
                "sample_time_ns": stamp, "epoch": 2, "plan_id": 3}
    return StateSnapshot({
        "chunk_execution_profiles": [capability()],
        "preview_execution": {"left": arm(left), "right": arm(right)},
    }, received)


def source():
    result = FlowMatchingActionSource.__new__(FlowMatchingActionSource)
    result.tcp_target_profile = "flow_infer_preview"
    result.arm_mask = np.array([1.0, 1.0])
    result.gripper_action_absolute = True
    result.gripper_command_source = "test"
    result.gripper_runtime = SimpleNamespace(dispatch=mock.Mock())
    result._gripper_targets_by_arm = {"left": None, "right": None}
    result._allow_gripper_targets_in_motion_packet = lambda: True
    result._measured_gripper_percent = lambda payload, arm: 60.0
    result._live_gripper_percent = lambda arm: 60.0
    return result


class PreviewExecutionPolicyTest(unittest.TestCase):
    def test_capability_must_be_explicit_and_validated_every_snapshot(self):
        src = source()
        src._require_chunk_execution_profile(snapshot().payload)
        for field, invalid in (
            ("preview_execution", None), ("preview_execution", 1),
            ("preview_execution", False), ("gripper_state_max_age_sec", None),
            ("gripper_state_max_age_sec", True), ("gripper_state_max_age_sec", 0),
            ("gripper_state_max_age_sec", float("nan")),
        ):
            payload = snapshot().payload
            payload["chunk_execution_profiles"][0][field] = invalid
            with self.subTest(field=field, invalid=invalid), self.assertRaisesRegex(ValueError, "refusing policy inference and motion"):
                src._require_chunk_execution_profile(payload)
        with self.assertRaises(ValueError):
            src._require_chunk_execution_profile({})

    def test_preview_handshake_precedes_model_camera_and_epoch_work(self):
        from policy_runner.openpi_remote import OpenpiRemoteActionSource
        src = OpenpiRemoteActionSource.__new__(OpenpiRemoteActionSource)
        src.tcp_target_profile = "flow_infer_preview"
        with mock.patch.object(src, "_camera_runtime_gate") as camera, \
                mock.patch.object(src, "_handle_server_motion_epoch") as epoch:
            with self.assertRaisesRegex(ValueError, "preview_execution=true"):
                src.next_intent(StateSnapshot({}, 1.0), 1.0)
            camera.assert_not_called()
            epoch.assert_not_called()

    def test_per_arm_wait_drops_both_gripper_sinks_without_advancing_target_latch(self):
        src = source()
        state = snapshot(left=False)
        src._update_preview_gripper_authority(state, 1.01)
        step = np.zeros(14)
        step[6], step[13] = 7.0, 42.0
        targets = src._integrate_gripper_targets(step, state.payload)
        self.assertEqual(targets, {"left": None, "right": 42.0})
        self.assertIsNone(src._gripper_targets_by_arm["left"])
        src._dispatch_gripper_step(step)
        commands = src.gripper_runtime.dispatch.call_args.args[0]
        self.assertEqual([command.arm for command in commands], ["right"])
        self.assertEqual(commands[0].value, 42.0)
        np.testing.assert_array_equal(src.arm_mask, [1, 1])

    def test_stale_missing_faulted_or_future_execution_cannot_release_gripper(self):
        src = source()
        src._update_preview_gripper_authority(snapshot(), 1.01)
        self.assertTrue(src._preview_gripper_arm_allowed("left"))
        cases = [snapshot(stamp=900_000_000), snapshot(received=0.9),
                 snapshot(stamp=1_100_000_000), snapshot(received=1.1)]
        for field, value in (("preview_execution", None), ("fault_latched", True),
                             ("send_suppressed", True)):
            state = snapshot()
            state.payload[field] = value
            cases.append(state)
        for field, value in (("active", 1), ("enabled", "true"), ("status", "waiting"),
                             ("sample_time_ns", True), ("sample_time_ns", 1e9)):
            state = snapshot()
            for arm in ("left", "right"):
                state.payload["preview_execution"][arm][field] = value
            cases.append(state)
        for state in cases:
            with self.subTest(payload=state.payload):
                src._update_preview_gripper_authority(state, 1.01)
                self.assertFalse(src._preview_gripper_arm_allowed("left"))
                self.assertFalse(src._preview_gripper_arm_allowed("right"))

    def test_between_row_cached_intent_loses_grip_but_keeps_tcp_and_profile(self):
        src = source()
        original = CommandIntent("TcpPoseTarget", left={"mode": "TcpPoseTarget", "tcp_target_stand": [1, 2, 3], "gripper_target": 7},
                                 right={"mode": "TcpPoseTarget", "gripper_target": 42}, tcp_target_profile="flow_infer_preview")
        before = copy.deepcopy(original)
        src._update_preview_gripper_authority(snapshot(left=False), 1.01)
        guarded = src._guard_preview_gripper_intent(original)
        self.assertNotIn("gripper_target", guarded.left)
        self.assertEqual(guarded.left["tcp_target_stand"], [1, 2, 3])
        self.assertEqual(guarded.right["gripper_target"], 42)
        self.assertEqual(guarded.tcp_target_profile, "flow_infer_preview")
        self.assertEqual(original, before)

    def test_actual_next_intent_keeps_tcp_during_first_plan_wait(self):
        src = source()
        src.enable_async_chunking = True
        intent = CommandIntent("TcpPoseTarget", left={"mode": "TcpPoseTarget", "gripper_target": 7},
                               right={"mode": "TcpPoseTarget", "gripper_target": 42}, tcp_target_profile="flow_infer_preview")
        with mock.patch.object(src, "_handle_server_motion_epoch"), \
                mock.patch.object(src, "_before_policy_intent"), \
                mock.patch.object(src, "_next_intent_streamed", return_value=intent) as emit:
            result = src.next_intent(snapshot(left=False, right=False), 1.01)
            emit.assert_called_once()
            self.assertEqual(result.mode, "TcpPoseTarget")
            self.assertNotIn("gripper_target", result.left)
            self.assertNotIn("gripper_target", result.right)
            self.assertEqual(result.tcp_target_profile, "flow_infer_preview")

    def test_nonpreview_profiles_preserve_existing_gripper_dispatch(self):
        src = source()
        src.tcp_target_profile = "flow_infer_fresh"
        for arm in ("left", "right"):
            self.assertTrue(src._preview_gripper_arm_allowed(arm))
        step = np.zeros(14)
        src._dispatch_gripper_step(step)
        self.assertEqual(len(src.gripper_runtime.dispatch.call_args.args[0]), 2)


if __name__ == "__main__":
    unittest.main()
