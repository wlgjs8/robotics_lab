from __future__ import annotations

import threading
import time
import unittest
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from policy_runner.config import ForceRecoveryConfig, config_from_mapping, load_config
from policy_runner.robot_state_client import StateSnapshot

try:
    from policy_runner.flow_inference import FlowMatchingActionSource, _gripper_value_from_payload
except ModuleNotFoundError as exc:
    if exc.name != "torch":
        raise
    FlowMatchingActionSource = None  # type: ignore[assignment]
    _gripper_value_from_payload = None  # type: ignore[assignment]

_TORCH_AVAILABLE = FlowMatchingActionSource is not None


def _payload(
    *, contact: bool, x: float = 0.0, epoch: int = 1, measured_force_n: float = 0.0
) -> dict:
    def arm(gripper: float) -> dict:
        return {
            "tcp_stand": {
                "x": x,
                "y": 0.0,
                "z": 0.5,
                "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            },
            "force_control": {
                "contact_active": contact,
                "measured_force_n": measured_force_n,
            },
            "gripper": {"percent": gripper, "target_percent": gripper + 1.0},
        }

    return {"motion_epoch": epoch, "left": arm(31.0), "right": arm(42.0)}


def _snapshot(*, contact: bool, x: float = 0.0, epoch: int = 1) -> StateSnapshot:
    return StateSnapshot(payload=_payload(contact=contact, x=x, epoch=epoch), received_monotonic=0.0)


def _source(
    *, contact_timeout: float = 5.0, settling_timeout: float = 2.0
) -> FlowMatchingActionSource:
    source = FlowMatchingActionSource.__new__(FlowMatchingActionSource)
    source.timeout_sec = 0.05
    source.policy_dt_sec = 0.1
    source.camera_names = []
    source.camera_client = None
    source._warned_missing_camera_client = False
    source._last_server_motion_epoch = None
    source._reset_left_pose = None
    source._reset_right_pose = None
    source._target_pose_by_arm = {"left": np.ones(7), "right": np.ones(7)}
    source._gripper_targets_by_arm = {"left": 25.0, "right": 35.0}
    source._current_gripper_targets = {"left": 26.0, "right": 36.0}
    source._chunk = np.ones((2, 14), dtype=np.float32)
    source._chunk_index = 1
    source._steps_since_boundary = 1
    source._current_step_intent = object()
    source._tcp_tp_conditioners = None
    source.configure_force_recovery(
        ForceRecoveryConfig(
            enable=True,
            contact_timeout_sec=contact_timeout,
            settling_timeout_sec=settling_timeout,
        )
    )
    return source


class ForceRecoveryConfigTest(unittest.TestCase):
    def test_config_loads_and_validates(self) -> None:
        config = config_from_mapping(
            {
                "force_recovery": {
                    "enable": True,
                    "settle_time_sec": 0.2,
                    "max_linear_velocity_m_s": 0.003,
                    "max_angular_velocity_rad_s": 0.06,
                    "contact_timeout_sec": 5,
                    "settling_timeout_sec": 3,
                }
            }
        )
        self.assertTrue(config.force_recovery.enable)
        self.assertEqual(config.force_recovery.contact_timeout_sec, 5.0)
        self.assertEqual(config.force_recovery.settling_timeout_sec, 3.0)
        with self.assertRaisesRegex(ValueError, "contact_timeout_sec"):
            ForceRecoveryConfig(enable=True, contact_timeout_sec=0.0)
        with self.assertRaisesRegex(ValueError, "settling_timeout_sec"):
            ForceRecoveryConfig(enable=True, settling_timeout_sec=0.0)

    def test_deprecated_timeout_is_per_phase_fallback_and_cannot_be_mixed(self) -> None:
        config = config_from_mapping({"force_recovery": {"timeout_sec": 4.0}})
        self.assertEqual(config.force_recovery.contact_timeout_sec, 4.0)
        self.assertEqual(config.force_recovery.settling_timeout_sec, 4.0)
        with self.assertRaisesRegex(ValueError, "must not set deprecated timeout_sec"):
            config_from_mapping(
                {
                    "force_recovery": {
                        "timeout_sec": 4.0,
                        "contact_timeout_sec": 5.0,
                    }
                }
            )

    def test_real_flow_profile_enables_recovery_without_enabling_teleop_stack(self) -> None:
        try:
            import yaml  # noqa: F401
        except ModuleNotFoundError:
            self.skipTest("PyYAML is required for tracked configs with list entries")
        root = Path(__file__).resolve().parents[2]
        flow = load_config(root / "policy_runner/config/flow_real_realsense.yaml")
        stack = load_config(root / "policy_runner/config/stack_real.yaml")
        self.assertTrue(flow.force_recovery.enable)
        self.assertEqual(flow.force_recovery.contact_timeout_sec, 5.0)
        self.assertEqual(flow.force_recovery.settling_timeout_sec, 2.0)
        self.assertFalse(stack.force_recovery.enable)


@unittest.skipUnless(_TORCH_AVAILABLE, "flow-infer runtime dependency torch is not installed")
class ForceRecoveryGateTest(unittest.TestCase):
    def test_contact_invalidates_once_and_holds_both_grippers(self) -> None:
        source = _source()
        blocked, intent = source._force_recovery_gate(_snapshot(contact=True), 0.0)
        self.assertTrue(blocked)
        self.assertEqual(intent.mode, "Hold")
        self.assertEqual(intent.left, {"mode": "Hold", "gripper_target": 26.0})
        self.assertEqual(intent.right, {"mode": "Hold", "gripper_target": 36.0})
        self.assertIsNone(source._chunk)

        source._force_recovery_gate(_snapshot(contact=True), 0.2)
        status = source.force_recovery_status()
        self.assertEqual(status["counters"]["contact_events"], 1)
        self.assertEqual(status["counters"]["chunk_invalidations"], 1)

    def test_release_resets_then_reanchors_after_measured_quiet_window(self) -> None:
        source = _source()
        reset_calls: list[str] = []
        source.reset_rtc = lambda: reset_calls.append("rtc")

        source._force_recovery_gate(_snapshot(contact=True), 0.0)
        blocked, _ = source._force_recovery_gate(_snapshot(contact=False), 0.01)
        self.assertTrue(blocked)
        self.assertEqual(reset_calls, ["rtc"])
        self.assertIsNone(source._target_pose_by_arm["left"])
        # The delta-mode accumulator is kept at the last committed absolute target.
        self.assertEqual(source._gripper_targets_by_arm, {"left": 25.0, "right": 35.0})

        blocked, intent = source._force_recovery_gate(_snapshot(contact=False), 0.14)
        self.assertFalse(blocked)
        self.assertIsNone(intent)
        self.assertEqual(source._force_recovery_state, "running")
        self.assertEqual(source._gripper_targets_by_arm, {"left": 26.0, "right": 36.0})
        self.assertEqual(source._force_recovery_counters["measured_reanchors"], 1)
        self.assertEqual(source._force_recovery_counters["cold_inference_restarts"], 1)

    def test_recontact_restarts_gate_and_contact_timeout_is_terminal_hold(self) -> None:
        source = _source(contact_timeout=0.2)
        source._force_recovery_gate(_snapshot(contact=True), 0.0)
        source._force_recovery_gate(_snapshot(contact=False), 0.01)
        source._force_recovery_gate(_snapshot(contact=True), 0.05)
        self.assertEqual(source._force_recovery_counters["recontacts"], 1)
        self.assertEqual(source._force_recovery_counters["chunk_invalidations"], 2)

        blocked, intent = source._force_recovery_gate(_snapshot(contact=True), 0.26)
        self.assertTrue(blocked)
        self.assertEqual(intent.mode, "Hold")
        self.assertEqual(source.force_recovery_terminal_abort_reason, "force_contact_timeout")
        self.assertEqual(source.force_recovery_status()["blocked_on"], "contact_active")
        self.assertEqual(source._force_recovery_counters["contact_timeouts"], 1)

    def test_three_second_contact_does_not_consume_settling_deadline(self) -> None:
        source = _source(contact_timeout=5.0, settling_timeout=0.2)
        source._force_recovery_gate(_snapshot(contact=True), 0.0)
        blocked, _ = source._force_recovery_gate(_snapshot(contact=True), 3.0)
        self.assertTrue(blocked)
        self.assertIsNone(source.force_recovery_terminal_abort_reason)

        source._force_recovery_gate(_snapshot(contact=False), 3.01)
        blocked, _ = source._force_recovery_gate(_snapshot(contact=False), 3.15)
        self.assertFalse(blocked)
        self.assertIsNone(source.force_recovery_terminal_abort_reason)

    def test_settling_timeout_reports_motion_blocker(self) -> None:
        source = _source(settling_timeout=0.2)
        source._force_recovery_gate(_snapshot(contact=True), 0.0)
        source._force_recovery_gate(_snapshot(contact=False), 3.0)
        source._force_recovery_gate(_snapshot(contact=False, x=0.01), 3.1)
        blocked, intent = source._force_recovery_gate(
            _snapshot(contact=False, x=0.02), 3.21
        )
        self.assertTrue(blocked)
        self.assertEqual(intent.mode, "Hold")
        self.assertEqual(source.force_recovery_terminal_abort_reason, "force_settling_timeout")
        self.assertEqual(source.force_recovery_status()["blocked_on"], "tcp_motion")

    def test_settling_timeout_reports_camera_blocker(self) -> None:
        source = _source(settling_timeout=0.2)
        source.camera_names = ["left_camera"]
        source._poll_camera_bundle = lambda: SimpleNamespace(
            bundle_seq=10, received_monotonic=3.0
        )
        source._count_missing_camera_frames = lambda bundle: 0
        source._force_recovery_gate(_snapshot(contact=True), 0.0)
        source._force_recovery_gate(_snapshot(contact=False), 3.0)
        source._force_recovery_gate(_snapshot(contact=False), 3.14)
        blocked, _ = source._force_recovery_gate(_snapshot(contact=False), 3.21)
        self.assertTrue(blocked)
        self.assertEqual(source.force_recovery_terminal_abort_reason, "camera_stale_timeout")
        status = source.force_recovery_status()
        self.assertEqual(status["blocked_on"], "stale_camera")
        self.assertEqual(status["camera"]["barrier_seq"], 10)
        self.assertEqual(status["camera"]["latest_seq"], 10)
        self.assertFalse(status["camera"]["fresh"])

    def test_fresh_post_reset_camera_is_required(self) -> None:
        source = _source()
        source.camera_names = ["left_camera"]
        bundles = iter(
            [
                SimpleNamespace(bundle_seq=10, received_monotonic=0.01),
                SimpleNamespace(bundle_seq=10, received_monotonic=0.02),
                SimpleNamespace(bundle_seq=11, received_monotonic=0.15),
            ]
        )
        source._poll_camera_bundle = lambda: next(bundles)
        source._count_missing_camera_frames = lambda bundle: 0

        source._force_recovery_gate(_snapshot(contact=True), 0.0)
        source._force_recovery_gate(_snapshot(contact=False), 0.01)
        blocked, _ = source._force_recovery_gate(_snapshot(contact=False), 0.14)
        self.assertTrue(blocked)
        blocked, _ = source._force_recovery_gate(_snapshot(contact=False), 0.15)
        self.assertFalse(blocked)

    def test_gripper_feedback_reads_server_percent_fields(self) -> None:
        payload = _payload(contact=False)
        self.assertEqual(_gripper_value_from_payload(payload, "left"), 32.0)
        del payload["left"]["gripper"]["target_percent"]
        self.assertEqual(_gripper_value_from_payload(payload, "left"), 31.0)

    def test_status_exposes_force_velocity_camera_and_worker_predicates(self) -> None:
        source = _source()
        source._force_recovery_gate(
            StateSnapshot(
                payload=_payload(contact=True, measured_force_n=6.75),
                received_monotonic=0.0,
            ),
            1.0,
        )
        status = source.force_recovery_status()
        self.assertEqual(status["state"], "contact")
        self.assertEqual(status["blocked_on"], "contact_active")
        self.assertEqual(status["contact_elapsed_sec"], 0.0)
        self.assertEqual(status["arms"]["right"]["measured_normal_force_n"], 6.75)
        self.assertIn("measured_tcp_velocity", status)
        self.assertIn("inflight_worker_generation", status)

    def test_openpi_generic_abort_does_not_mask_force_recovery_reason(self) -> None:
        from policy_runner.openpi_remote import OpenpiRemoteActionSource

        source = OpenpiRemoteActionSource.__new__(OpenpiRemoteActionSource)
        source._camera_runtime_terminal_abort_reason = None
        source._force_recovery_terminal_abort_reason = "force_contact_timeout"
        self.assertEqual(source.terminal_abort_reason, "force_contact_timeout")

        source._camera_runtime_terminal_abort_reason = "camera_stale_timeout"
        self.assertEqual(source.terminal_abort_reason, "camera_stale_timeout")

    def test_stale_openpi_side_effect_reset_restarts_settle_window(self) -> None:
        source = _source()
        source._force_recovery_state = "settling"
        source._force_recovery_quiet_since = 1.0
        source._force_recovery_prev_pose = {
            "left": (1.0, np.ones(7)),
            "right": (1.0, np.ones(7)),
        }
        source._last_obs_camera_bundle = SimpleNamespace(
            bundle_seq=44, received_monotonic=2.0
        )
        reset_calls = []
        source.reset_rtc = lambda: reset_calls.append("reset")

        source._on_stale_inference_completion()

        self.assertEqual(reset_calls, ["reset"])
        self.assertEqual(source._force_recovery_camera_barrier_seq, 44)
        self.assertEqual(source._force_recovery_camera_barrier_received, 2.0)
        self.assertEqual(source._force_recovery_prev_pose, {"left": None, "right": None})
        self.assertGreater(source._force_recovery_quiet_since, 1.0)

    def test_stale_completion_clears_actual_openpi_rtc_history_and_camera_cache(self) -> None:
        from policy_runner.openpi_remote import OpenpiRemoteActionSource

        source = OpenpiRemoteActionSource.__new__(OpenpiRemoteActionSource)
        source._rtc_prev_raw_chunk = np.ones((2, 14), dtype=np.float32)
        source._rtc_warned_no_raw = True
        source._vel_prev_pose_by_arm = {"left": np.ones(7), "right": np.ones(7)}
        source._vel_prev_sample_t = 1.0
        source._pose_history = {
            "left": deque([(1.0, np.ones(7))]),
            "right": deque([(1.0, np.ones(7))]),
        }
        source._command_pose_history = {
            "left": deque([(1.0, np.ones(7))]),
            "right": deque([(1.0, np.ones(7))]),
        }
        source._last_command_pose_by_arm = {"left": np.ones(7), "right": np.ones(7)}
        source._last_now_monotonic = 1.0
        source._last_obs_camera_bundle = SimpleNamespace(
            bundle_seq=55, received_monotonic=2.0
        )
        source._last_obs_camera_seq = 55
        source._last_obs_camera_time_sec = 2.0
        source._force_recovery_state = "settling"
        source._force_recovery_prev_pose = {
            "left": (1.0, np.ones(7)),
            "right": (1.0, np.ones(7)),
        }

        source._on_stale_inference_completion()

        self.assertIsNone(source._rtc_prev_raw_chunk)
        self.assertEqual(len(source._pose_history["left"]), 0)
        self.assertEqual(len(source._command_pose_history["right"]), 0)
        self.assertIsNone(source._last_obs_camera_bundle)
        self.assertEqual(source._force_recovery_camera_barrier_seq, 55)
        self.assertEqual(source._force_recovery_prev_pose, {"left": None, "right": None})


@unittest.skipUnless(_TORCH_AVAILABLE, "flow-infer runtime dependency torch is not installed")
class StreamGenerationRaceTest(unittest.TestCase):
    def test_stale_completion_cannot_clear_new_generation_request(self) -> None:
        source = _source()
        source.policy_label = "test"
        source.stderr = SimpleNamespace(write=lambda *_args, **_kwargs: None, flush=lambda: None)
        first_started = threading.Event()
        first_release = threading.Event()
        second_started = threading.Event()
        second_release = threading.Event()
        calls = 0

        def sample(_payload):
            nonlocal calls
            calls += 1
            if calls == 1:
                first_started.set()
                first_release.wait(1.0)
                return np.full((1, 14), 1.0, dtype=np.float32)
            second_started.set()
            second_release.wait(1.0)
            return np.full((1, 14), 2.0, dtype=np.float32)

        source._sample_and_align_chunk = sample
        source._ensure_stream_state()
        try:
            source._request_prefetch({"request": "A"})
            self.assertTrue(first_started.wait(1.0))
            source._invalidate_policy_chunks(reason="force_contact")
            source._request_prefetch({"request": "B"})
            first_release.set()
            self.assertTrue(second_started.wait(1.0))
            self.assertTrue(source._stream_pending)
            self.assertIsNone(source._stream_next_chunk)
            second_release.set()
            deadline = time.monotonic() + 1.0
            while source._stream_pending and time.monotonic() < deadline:
                time.sleep(0.005)
            chunk = source._take_prefetched()
            self.assertIsNotNone(chunk)
            self.assertTrue(np.all(chunk == 2.0))
        finally:
            source._stream_shutdown = True
            with source._stream_cv:
                source._stream_cv.notify_all()
            source._stream_thread.join(timeout=1.0)


if __name__ == "__main__":
    unittest.main()
