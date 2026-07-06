from __future__ import annotations

import sys
import unittest
from io import StringIO
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from policy_runner.config import config_from_mapping
from policy_runner.main import FAULT_LATCH_EXIT_CODE, run
from policy_runner.robot_state_client import StateSnapshot, fault_latch_from_snapshot


def snapshot(payload: dict[str, object]) -> StateSnapshot:
    return StateSnapshot(payload=dict(payload), received_monotonic=0.0)


def fault_context_snapshot() -> StateSnapshot:
    return snapshot(
        {
            "motion_state": "ConnectedHold",
            "fault_latched": False,
            "fault_context": {
                "latched": True,
                "motion_state": "FaultLatched",
                "latched_fault_reason": "ChunkFollowerFault",
                "reason": "ruckig chunk follower degraded",
            },
        }
    )


def runner_config():
    return config_from_mapping(
        {
            "schema": "robotics_lab.policy_runner.v1",
            "runtime": {"startup_timeout_sec": 0.1},
            "geometry": {"path": ""},
            "recording": {"control_enabled": False, "status_endpoint": None},
            "command_rate_hz": 10,
        }
    )


class FakeStateClient:
    def __init__(self, state: StateSnapshot):
        self._state = state
        self.started = False
        self.closed = False

    @property
    def latest(self) -> StateSnapshot:
        return self._state

    def start(self) -> None:
        self.started = True

    def close(self) -> None:
        self.closed = True


class FakeCommandClient:
    def __init__(self):
        self.closed = False

    def close(self) -> None:
        self.closed = True


class NoopRecordingSupervisor:
    def drain_commands(self, _snapshot: StateSnapshot, *, action_source: str) -> None:
        return None

    def record_frame(
        self,
        _snapshot: StateSnapshot,
        *,
        action_packet: dict[str, object] | None,
        action_host_time_ns: int | None,
        action_seq: int | None,
    ) -> None:
        return None

    def stamp_snapshot(self, _snapshot: StateSnapshot) -> None:
        return None

    def publish_status(self, **_kwargs: object) -> None:
        return None

    def close(self) -> None:
        return None


class MinimalSource:
    name = "minimal"

    def __init__(self, runner_role: str):
        self.runner_role = runner_role
        self.next_calls = 0
        self.closed = False

    def next_intent(self, _snapshot: StateSnapshot, _now_monotonic: float):
        self.next_calls += 1
        return None

    def close(self) -> None:
        self.closed = True


class FaultLatchReadbackTest(unittest.TestCase):
    def test_fault_context_is_primary_source(self) -> None:
        readback = fault_latch_from_snapshot(fault_context_snapshot())

        self.assertTrue(readback.latched)
        self.assertEqual(readback.motion_state, "FaultLatched")
        self.assertEqual(readback.latched_fault_reason, "ChunkFollowerFault")
        self.assertEqual(readback.reason, "ruckig chunk follower degraded")

    def test_top_level_fault_latched_fallback(self) -> None:
        readback = fault_latch_from_snapshot(
            snapshot({"fault_latched": True, "motion_state": "ConnectedHold"})
        )

        self.assertTrue(readback.latched)
        self.assertEqual(readback.motion_state, "ConnectedHold")
        self.assertIsNone(readback.latched_fault_reason)
        self.assertIsNone(readback.reason)

    def test_motion_state_only_latches(self) -> None:
        readback = fault_latch_from_snapshot(
            snapshot({"fault_latched": False, "motion_state": "EmergencyLatched"})
        )

        self.assertTrue(readback.latched)
        self.assertEqual(readback.motion_state, "EmergencyLatched")

    def test_absent_or_malformed_fields_do_not_latch(self) -> None:
        cases = (
            {},
            {"fault_context": None, "fault_latched": None, "motion_state": None},
            {"fault_context": "bad", "fault_latched": "true", "motion_state": 3},
            {"fault_context": {"latched": "true", "motion_state": 3}},
        )
        for payload in cases:
            with self.subTest(payload=payload):
                readback = fault_latch_from_snapshot(snapshot(payload))
                self.assertFalse(readback.latched)


class FlowInferFaultLatchAbortTest(unittest.TestCase):
    def test_flow_infer_exits_before_next_intent_on_latched_fault(self) -> None:
        state_client = FakeStateClient(fault_context_snapshot())
        command_client = FakeCommandClient()
        source = MinimalSource("flow_infer")
        stderr = StringIO()

        result = run(
            runner_config(),
            state_client=state_client,
            command_client=command_client,
            source=source,
            monotonic_fn=lambda: 0.0,
            sleep_fn=lambda _period: self.fail("flow-infer fault abort should not sleep"),
            stderr=stderr,
            recording_supervisor=NoopRecordingSupervisor(),
        )

        self.assertEqual(result, FAULT_LATCH_EXIT_CODE)
        self.assertEqual(source.next_calls, 0)
        self.assertTrue(state_client.started)
        self.assertTrue(state_client.closed)
        self.assertTrue(command_client.closed)
        self.assertTrue(source.closed)
        self.assertIn("policy_runner fault_latch_abort:", stderr.getvalue())
        self.assertIn("motion_state=FaultLatched", stderr.getvalue())
        self.assertIn("verdict=ChunkFollowerFault", stderr.getvalue())
        self.assertIn("reason=ruckig chunk follower degraded", stderr.getvalue())

    def test_non_flow_source_keeps_block_and_wait_behavior(self) -> None:
        state_client = FakeStateClient(fault_context_snapshot())
        command_client = FakeCommandClient()
        source = MinimalSource("stack")
        stderr = StringIO()
        sleeps = 0

        def stop_after_one_sleep(_period: float) -> None:
            nonlocal sleeps
            sleeps += 1
            raise KeyboardInterrupt

        result = run(
            runner_config(),
            state_client=state_client,
            command_client=command_client,
            source=source,
            monotonic_fn=lambda: 0.0,
            sleep_fn=stop_after_one_sleep,
            stderr=stderr,
            recording_supervisor=NoopRecordingSupervisor(),
        )

        self.assertEqual(result, 0)
        self.assertNotEqual(result, FAULT_LATCH_EXIT_CODE)
        self.assertEqual(sleeps, 1)
        self.assertEqual(source.next_calls, 1)
        self.assertNotIn("fault_latch_abort", stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
