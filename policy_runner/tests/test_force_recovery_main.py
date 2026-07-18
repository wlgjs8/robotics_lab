from __future__ import annotations

import io
import time
import unittest

from policy_runner.config import config_from_mapping
from policy_runner.main import FORCE_RECOVERY_TIMEOUT_EXIT_CODE, run
from policy_runner.robot_state_client import StateSnapshot
from policy_runner.rollout_modes import RolloutModePolicy, RolloutSummaryRecorder
from policy_runner.servo_command_client import CommandIntent


class _StateClient:
    def __init__(self, snapshot: StateSnapshot):
        self.snapshot = snapshot
        self.closed = False

    @property
    def latest(self):
        return self.snapshot

    def start(self):
        pass

    def close(self):
        self.closed = True


class _CommandClient:
    source_id = "policy_runner"
    session_id = "force-test"

    def __init__(self):
        self.sent = []
        self.closed = False

    def send(self, intent):
        self.sent.append(intent)
        return len(self.sent)

    def close(self):
        self.closed = True


class _Source:
    name = "flow_infer"
    requirements = None
    force_recovery_terminal_abort_reason = "force_contact_timeout"

    def next_intent(self, snapshot, now_monotonic):
        _ = (snapshot, now_monotonic)
        return CommandIntent.gripper_target(left=20.0, right=30.0, timeout_sec=0.05)

    def force_recovery_status(self):
        return {
            "enabled": True,
            "state": "timed_out",
            "blocked_on": "contact_active",
            "terminal_abort_reason": self.force_recovery_terminal_abort_reason,
        }

    def close(self):
        pass


class _CameraAbortSource(_Source):
    terminal_abort_reason = "camera_stale_timeout"
    force_recovery_terminal_abort_reason = None


class _Recording:
    def drain_control_payloads(self):
        return []

    def dispatch_control_payloads(self, payloads, snapshot, *, action_source):
        _ = (payloads, snapshot, action_source)

    def stamp_snapshot(self, snapshot):
        _ = snapshot

    def record_frame(self, *args, **kwargs):
        _ = (args, kwargs)

    def publish_status(self, **kwargs):
        self.last_status = kwargs

    def close(self):
        pass


class ForceRecoveryMainTest(unittest.TestCase):
    def test_timeout_aborts_only_after_hold_send_attempt(self) -> None:
        config = config_from_mapping(
            {
                "geometry": {"path": ""},
                "recording": {"control_enabled": False, "status_endpoint": None},
                "safety": {"require_valid_joint_state": False},
            }
        )
        snapshot = StateSnapshot(
            payload={
                "left": {"has_valid_joint_state": True},
                "right": {"has_valid_joint_state": True},
            },
            received_monotonic=time.monotonic(),
        )
        state = _StateClient(snapshot)
        command = _CommandClient()
        recording = _Recording()
        stderr = io.StringIO()

        result = run(
            config,
            state_client=state,
            command_client=command,
            source=_Source(),
            recording_supervisor=recording,
            stderr=stderr,
            sleep_fn=lambda _period: self.fail("timeout exit must not sleep another tick"),
        )

        self.assertEqual(result, FORCE_RECOVERY_TIMEOUT_EXIT_CODE)
        self.assertEqual(len(command.sent), 1)
        self.assertEqual(command.sent[0].mode, "Hold")
        self.assertEqual(command.sent[0].left["gripper_target"], 20.0)
        self.assertEqual(recording.last_status["force_recovery"]["state"], "timed_out")
        self.assertEqual(
            recording.last_status["force_recovery"]["terminal_abort_reason"],
            "force_contact_timeout",
        )
        self.assertIn("force_recovery_abort", stderr.getvalue())

    def test_generic_camera_abort_uses_distinct_log_after_hold_send(self) -> None:
        config = config_from_mapping(
            {
                "geometry": {"path": ""},
                "recording": {"control_enabled": False, "status_endpoint": None},
                "safety": {"require_valid_joint_state": False},
            }
        )
        snapshot = StateSnapshot(
            payload={
                "left": {"has_valid_joint_state": True},
                "right": {"has_valid_joint_state": True},
            },
            received_monotonic=time.monotonic(),
        )
        command = _CommandClient()
        stderr = io.StringIO()
        result = run(
            config,
            state_client=_StateClient(snapshot),
            command_client=command,
            source=_CameraAbortSource(),
            recording_supervisor=_Recording(),
            stderr=stderr,
            sleep_fn=lambda _period: self.fail("camera abort must exit after Hold"),
        )
        self.assertEqual(result, FORCE_RECOVERY_TIMEOUT_EXIT_CODE)
        self.assertEqual(len(command.sent), 1)
        self.assertEqual(command.sent[0].mode, "Hold")
        self.assertIn("policy_runner camera_abort: camera_stale_timeout", stderr.getvalue())

    def test_rollout_summary_contains_recovery_telemetry(self) -> None:
        recorder = RolloutSummaryRecorder(
            policy=RolloutModePolicy.from_value("sim_dryrun"),
            checkpoint_path="checkpoint",
            config_path="config",
        )
        data = recorder.to_dict(_Source())
        self.assertEqual(data["force_recovery"]["state"], "timed_out")


if __name__ == "__main__":
    unittest.main()
