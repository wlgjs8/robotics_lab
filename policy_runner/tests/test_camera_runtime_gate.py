from __future__ import annotations

import io
import unittest
from types import SimpleNamespace
from unittest import mock

from policy_runner.config import CameraConfig
from policy_runner.openpi_remote import OpenpiRemoteActionSource
from policy_runner.rollout_modes import RolloutModePolicy, RolloutSummaryRecorder
from policy_runner.servo_command_client import CommandIntent


def _bundle(seq: int, *, fresh: bool = True, missing: tuple[str, ...] = ()):
    frames = {
        name: SimpleNamespace(pixels=object())
        for name in ("left_realsense_color", "right_realsense_color")
        if name not in missing
    }
    return SimpleNamespace(
        bundle_seq=seq,
        bundle_time_ns=1,
        complete=True,
        frames=frames,
        fresh=fresh,
    )


class _CameraClient:
    def __init__(self, bundles):
        self._bundles = list(bundles)
        self._latest = None
        self._last_poll = {"outcome": "never_polled"}

    def poll(self, timeout_ms=0):
        del timeout_ms
        item = self._bundles.pop(0) if self._bundles else None
        if item is not None:
            self._latest = item
            self._last_poll = {"outcome": "ok", "bundle_seq": item.bundle_seq}
        else:
            self._last_poll = {"outcome": "no_message"}
        return item

    def latest(self):
        return self._latest

    @staticmethod
    def is_fresh(bundle):
        return bool(bundle is not None and bundle.fresh)

    def diagnostics_snapshot(self):
        return {"last_poll": dict(self._last_poll)}


def _source(client: _CameraClient) -> OpenpiRemoteActionSource:
    source = OpenpiRemoteActionSource.__new__(OpenpiRemoteActionSource)
    source.camera_client = client
    source.camera_names = ["left_realsense_color", "right_realsense_color"]
    source._fake_images = False
    source.timeout_sec = 0.2
    source.stderr = io.StringIO()
    source._current_gripper_targets = {"left": 31.0, "right": 47.0}
    source._gripper_targets_by_arm = {"left": None, "right": None}
    return source


class CameraRuntimeGateTest(unittest.TestCase):
    def test_five_distinct_fresh_complete_bundles_are_required_before_motion(self):
        source = _source(_CameraClient([_bundle(seq) for seq in range(1, 6)]))
        source.configure_camera_runtime(
            CameraConfig(
                enable=True,
                readiness_bundle_count=5,
                readiness_timeout_sec=1.0,
                stale_timeout_sec=0.5,
            )
        )

        for index in range(4):
            blocked, intent = source._camera_runtime_gate(index * 0.02)
            self.assertTrue(blocked)
            self.assertEqual(intent.mode, "Hold")
            self.assertEqual(intent.left["gripper_target"], 31.0)
            self.assertEqual(intent.right["gripper_target"], 47.0)

        blocked, intent = source._camera_runtime_gate(0.08)
        self.assertFalse(blocked)
        self.assertIsNone(intent)
        self.assertEqual(source.camera_runtime_status()["state"], "running")
        self.assertEqual(
            source.camera_runtime_status()["consecutive_fresh_complete_bundles"], 5
        )

    def test_duplicate_bundle_sequence_does_not_satisfy_preflight(self):
        source = _source(_CameraClient([_bundle(7) for _ in range(5)]))
        source.configure_camera_runtime(
            CameraConfig(enable=True, readiness_bundle_count=5, readiness_timeout_sec=1.0)
        )

        for index in range(5):
            blocked, _ = source._camera_runtime_gate(index * 0.02)
            self.assertTrue(blocked)
        self.assertEqual(source.camera_runtime_status()["consecutive_fresh_complete_bundles"], 1)

    def test_startup_timeout_is_camera_specific_and_remains_hold(self):
        source = _source(_CameraClient([]))
        source.configure_camera_runtime(
            CameraConfig(enable=True, readiness_bundle_count=5, readiness_timeout_sec=1.0)
        )

        source._camera_runtime_gate(10.0)
        blocked, intent = source._camera_runtime_gate(11.0)

        self.assertTrue(blocked)
        self.assertEqual(intent.mode, "Hold")
        self.assertEqual(source.camera_terminal_abort_reason, "camera_stale_timeout")
        self.assertEqual(source.terminal_abort_reason, "camera_stale_timeout")

    def test_runtime_stale_window_holds_then_terminates(self):
        client = _CameraClient([_bundle(seq) for seq in range(1, 6)])
        source = _source(client)
        source.configure_camera_runtime(
            CameraConfig(
                enable=True,
                readiness_bundle_count=5,
                readiness_timeout_sec=1.0,
                stale_timeout_sec=0.5,
            )
        )
        for index in range(5):
            source._camera_runtime_gate(index * 0.02)

        client._latest.fresh = False
        blocked, intent = source._camera_runtime_gate(0.10)
        self.assertTrue(blocked)
        self.assertEqual(intent.mode, "Hold")
        self.assertIsNone(source.camera_terminal_abort_reason)

        blocked, intent = source._camera_runtime_gate(0.60)
        self.assertTrue(blocked)
        self.assertEqual(intent.mode, "Hold")
        self.assertEqual(source.camera_terminal_abort_reason, "camera_stale_timeout")

    def test_missing_wrist_stream_cannot_count_as_ready(self):
        source = _source(
            _CameraClient([_bundle(seq, missing=("left_realsense_color",)) for seq in range(1, 6)])
        )
        source.configure_camera_runtime(
            CameraConfig(enable=True, readiness_bundle_count=5, readiness_timeout_sec=1.0)
        )

        for index in range(5):
            blocked, _ = source._camera_runtime_gate(index * 0.02)
            self.assertTrue(blocked)
        status = source.camera_runtime_status()
        self.assertEqual(status["consecutive_fresh_complete_bundles"], 0)
        self.assertEqual(status["missing_streams"], ["left_realsense_color"])

    def test_force_contact_precedes_blocked_camera_guard(self):
        client = _CameraClient([])
        source = _source(client)
        source.configure_camera_runtime(
            CameraConfig(enable=True, readiness_bundle_count=5, readiness_timeout_sec=1.0)
        )
        force_hold = CommandIntent.gripper_target(left=22.0, right=44.0)
        snapshot = SimpleNamespace(payload={"left": {}, "right": {}})
        source._force_recovery_gate = mock.Mock(return_value=(True, force_hold))
        source._handle_server_motion_epoch = mock.Mock()

        intent = source.next_intent(snapshot, 0.0)

        self.assertIs(intent, force_hold)
        source._force_recovery_gate.assert_called_once_with(snapshot, 0.0)
        source._handle_server_motion_epoch.assert_not_called()
        self.assertEqual(len(client._bundles), 0)

    def test_camera_guard_status_is_written_to_rollout_summary(self):
        source = _source(_CameraClient([]))
        source.configure_camera_runtime(
            CameraConfig(enable=True, readiness_bundle_count=5, readiness_timeout_sec=1.0)
        )
        source._camera_runtime_gate(0.0)
        recorder = RolloutSummaryRecorder(
            policy=RolloutModePolicy.from_value("sim_dryrun"),
            checkpoint_path="openpi://test",
            config_path="test.yaml",
        )

        document = recorder.to_dict(source)

        self.assertEqual(document["camera_runtime"]["state"], "startup")
        self.assertEqual(document["camera_runtime"]["required_consecutive_bundles"], 5)


if __name__ == "__main__":
    unittest.main()
