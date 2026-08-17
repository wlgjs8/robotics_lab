from __future__ import annotations

import threading
import time
import unittest
from types import SimpleNamespace

import numpy as np

from policy_runner.flow_inference import FlowMatchingActionSource


def _source() -> FlowMatchingActionSource:
    """Bare action source with just enough state for the prefetch stream.

    Re-homed from tests/test_force_recovery.py when force recovery was removed
    (2026-08-17): the generation race and the stale-completion RTC reset are not
    force-specific — any invalidation (arm-init override, motion_epoch, an
    explicit `_invalidate_policy_chunks`) can strand an in-flight worker.
    """
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
    return source


class StreamGenerationRaceTest(unittest.TestCase):
    def test_stale_completion_cannot_clear_new_generation_request(self) -> None:
        source = _source()
        source.policy_label = "test"
        source.stderr = SimpleNamespace(write=lambda *_a, **_k: None, flush=lambda: None)
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
            source._invalidate_policy_chunks(reason="test_invalidation")
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

    def test_stale_completion_resets_subclass_rtc_guidance(self) -> None:
        """`_on_stale_inference_completion` must undo a discarded worker's RTC write."""
        source = _source()
        reset_calls = []
        source.reset_rtc = lambda: reset_calls.append(True)

        source._on_stale_inference_completion()

        self.assertEqual(len(reset_calls), 1)

    def test_stale_completion_is_a_noop_without_rtc(self) -> None:
        source = _source()

        source._on_stale_inference_completion()  # must not raise


if __name__ == "__main__":
    unittest.main()
