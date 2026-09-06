"""Exercise the real dispatcher/alignment/worker with in-memory I/O only."""
from __future__ import annotations

import copy
import io
import threading
import unittest
from types import SimpleNamespace
from unittest import mock

import numpy as np

from policy_runner.flow_inference import FlowMatchingActionSource, validate_chunk_activation_mode


class OfflineStreamSource(FlowMatchingActionSource):
    """No model, camera, robot or socket; scheduling methods are production code."""

    def __init__(self, mode="fixed_steps", *, dt=0.0334, execute_steps=4):
        self.configure_chunk_activation(mode)
        self.policy_dt_sec = dt
        self.chunk_execute_steps = execute_steps
        self._reset_left_pose = self._reset_right_pose = np.array([0, 0, 0, 0, 0, 0, 1.0])
        self._chunk = None
        self._chunk_index = 0
        self._stream_emitted_policy_steps = 0
        self._stream_generation = 0
        self._stream_pending = False
        self._stream_next_chunk = None
        self._stream_next_chunk_metadata = None
        self._stream_request = None
        self._stream_inflight_generation = None
        self._stream_shutdown = False
        self._stream_stall_count = 0
        self._stream_inited = True
        self._stream_lock = threading.Lock()
        self._stream_cv = threading.Condition(self._stream_lock)
        self._step_deadline = 0.0
        self._current_step_intent = None
        self._last_overlay_payload = None
        self._overlay_chain_pending = None
        self._steps_since_boundary = 0
        self.nonblocking_stream_inference = True
        self.timeout_sec = 0.25
        self._print_chunk_enabled = self._print_tracking_enabled = False
        self._init_inference_timing_state()
        self.now = 1.0
        self._inference_clock_ns = lambda: int(self.now * 1e9)
        self.published = []
        self.activation_timings = []
        self.emitted = []
        self.gripper_rows = []
        self.stale_completions = 0
        self.stderr = io.StringIO()
        self.policy_label = "offline scheduler"

    @staticmethod
    def rows(start=0, horizon=24):
        # Distinct source rows on both arms and both grippers expose off-by-one
        # errors without simulating either model inference or robot dynamics.
        return np.repeat(np.arange(start, start + horizon)[:, None], 14, axis=1).astype(float)

    def _publish_chunk_overlay(self, now_monotonic):
        self.published.append((now_monotonic, self._chunk.copy(), copy.deepcopy(self._active_chunk_metadata)))
        self.activation_timings.append(copy.deepcopy(self._inference_timing_snapshot()))

    def _integrate_gripper_targets(self, step, payload):
        return {"left": float(step[6]), "right": float(step[13])}

    def _dispatch_gripper_step(self, step):
        self.gripper_rows.append((float(step[6]), float(step[13])))

    def _tcp_tp_foh_active(self):
        return False

    def _emit_step_intent(self, step, payload, gripper_targets):
        self.emitted.append((self.now, step.copy(), gripper_targets.copy()))
        return tuple(step)

    def _log_rollout_policy_step(self, **kwargs):
        pass

    def _on_stale_inference_completion(self):
        self.stale_completions += 1

    def tick(self, now):
        self.now = now
        return self._next_intent_streamed(SimpleNamespace(payload={"captured_at": now}), now)

    def complete_request(self, now, *, rows=None):
        """Inject a measured/specified completion time at the real worker output."""
        self.now = now
        with self._stream_lock:
            generation, seq, requested, payload, observed = self._stream_request
            self._stream_request = None
            timing = self._record_inference_completion(
                inference_seq=seq, request_ns=requested, worker_start_ns=requested,
                worker_end_ns=int(now * 1e9), ready_ns=int(now * 1e9), succeeded=True,
            )
            self._stream_next_chunk = self.rows(seq * 100) if rows is None else rows
            self._stream_next_chunk_metadata = {"generation": generation, "observation_step_seq": observed}
            self._stream_ready_timing = timing
            self._stream_pending = False

    def start_ready(self):
        self.tick(1.0)
        self.complete_request(1.01, rows=self.rows())
        self.tick(1.01)


class ChunkActivationSchedulerTest(unittest.TestCase):
    def test_openpi_fresh_profile_handshake_precedes_camera_gate_and_remote_inference(self):
        from policy_runner.openpi_remote import OpenpiRemoteActionSource

        source = OpenpiRemoteActionSource.__new__(OpenpiRemoteActionSource)
        source.tcp_target_profile = "flow_infer_fresh"
        with (
            mock.patch.object(source, "_camera_runtime_gate") as camera,
            mock.patch.object(source, "_handle_server_motion_epoch") as epoch,
            mock.patch.object(source, "_sample_and_align_chunk") as sample,
        ):
            with self.assertRaisesRegex(ValueError, "chunk_execution_profiles"):
                source.next_intent(SimpleNamespace(payload={}), 1.0)
            camera.assert_not_called()
            epoch.assert_not_called()
            sample.assert_not_called()

    def test_fresh_profile_handshake_rejects_missing_disabled_or_ambiguous_server_before_any_work(self):
        valid = {
            "name": "flow_infer_fresh", "enabled": True, "controller": "delta_preview",
            "fresh_chunk_replan": True, "continuous_hold_resume": True,
        }
        invalid_payloads = [
            {}, {"chunk_execution_profiles": []}, {"chunk_execution_profiles": valid},
            {"chunk_execution_profiles": [valid, valid]},
        ]
        for key, value in (
            ("name", "other"), ("enabled", False), ("enabled", 1),
            ("controller", "pose_track_smd"), ("fresh_chunk_replan", False),
            ("continuous_hold_resume", "true"),
        ):
            invalid_payloads.append({"chunk_execution_profiles": [{**valid, key: value}]})
        for payload in invalid_payloads:
            with self.subTest(payload=payload):
                source = OfflineStreamSource("ready_event")
                source.tcp_target_profile = "flow_infer_fresh"
                with (
                    mock.patch.object(source, "_before_policy_intent") as before,
                    mock.patch.object(source, "_sample_and_align_chunk") as sample,
                    mock.patch.object(source, "_request_prefetch") as request,
                ):
                    with self.assertRaisesRegex(ValueError, "refusing policy inference and motion"):
                        source.next_intent(SimpleNamespace(payload=payload), 1.0)
                    before.assert_not_called()
                    sample.assert_not_called()
                    request.assert_not_called()
                self.assertEqual(source.published, [])
                self.assertEqual(source.emitted, [])
                self.assertEqual(source.gripper_rows, [])
        source = OfflineStreamSource("ready_event")
        source.tcp_target_profile = "flow_infer_fresh"
        source._require_chunk_execution_profile({"chunk_execution_profiles": [valid]})
        # A good earlier snapshot is not cached after a restart/downgrade.
        with self.assertRaises(ValueError):
            source._require_chunk_execution_profile({})
        source.tcp_target_profile = "flow_infer_smooth"
        source._require_chunk_execution_profile({})

    def test_fixed_steps_keeps_four_rows_even_when_next_chunk_is_ready(self):
        source = OfflineStreamSource()
        source.start_ready()
        self.assertIsNone(source._stream_request)
        source.tick(1.044)
        source.complete_request(1.05)
        source.tick(1.077)
        source.tick(1.111)
        self.assertEqual(len(source.published), 1)
        source.tick(1.144)
        np.testing.assert_array_equal([row[1][0] for row in source.emitted], [0, 1, 2, 3, 203])
        self.assertEqual(source.published[-1][2]["source_start_index"], 3)
        self.assertEqual(source.published[-1][2]["replaced_chunk_steps"], 4)

    def test_ready_event_waits_for_row_deadline_then_aligns_both_arms_and_grippers(self):
        source = OfflineStreamSource("ready_event")
        source.start_ready()
        self.assertEqual(source._stream_request[-1], 0)  # pre-commit observation
        source.complete_request(1.03)
        held = source.tick(1.04)
        self.assertEqual(held[0], 0)
        self.assertEqual(len(source.published), 1)
        source.tick(1.044)
        metadata = source.published[-1][2]
        self.assertEqual(metadata["source_start_index"], 1)
        self.assertEqual(metadata["replaced_chunk_steps"], 1)
        self.assertEqual(metadata["activation_reason"], "ready_event")
        np.testing.assert_array_equal(source.emitted[-1][1], np.repeat(201, 14))
        self.assertEqual(source.gripper_rows, [(0, 0), (201, 201)])
        self.assertEqual(source._stream_request[-1], 1)
        # No per-chunk accumulation of the 0.6 ms tick lateness.
        self.assertAlmostEqual(source._step_deadline, 1.0768)
        self.assertAlmostEqual(source._inference_timing_snapshot()["ready_wait_ms"], 14.0)

    def test_late_result_holds_at_execute_limit_then_resumes_without_catchup(self):
        source = OfflineStreamSource("ready_event")
        source.start_ready()
        for now in (1.044, 1.077, 1.111):
            source.tick(now)
        last = source._current_step_intent
        self.assertEqual(source.tick(1.144), last)
        self.assertIsNone(source._chunk)
        self.assertEqual(source._stream_stall_count, 1)
        source.tick(1.3)
        self.assertEqual(len(source.emitted), 4)
        source.complete_request(1.31)
        source.tick(1.312)
        self.assertEqual(len(source.emitted), 5)
        self.assertEqual(source.published[-1][2]["source_start_index"], 4)
        self.assertAlmostEqual(source._step_deadline, 1.3454)

    def test_long_command_loop_pause_never_bursts_catchup_rows(self):
        source = OfflineStreamSource("ready_event")
        source.start_ready()
        source.complete_request(1.03)
        source.tick(2.0)
        source.tick(2.001)
        self.assertEqual(len(source.emitted), 2)
        self.assertAlmostEqual(source._step_deadline, 2.0334)

    def test_expired_or_wrong_generation_ready_candidate_keeps_live_old_chunk(self):
        for expired in (True, False):
            with self.subTest(expired=expired):
                source = OfflineStreamSource("ready_event")
                source.start_ready()
                source.complete_request(1.03, rows=source.rows(100, horizon=1 if expired else 24))
                if not expired:
                    source._stream_next_chunk_metadata["generation"] = -1
                source.tick(1.044)
                self.assertEqual(len(source.published), 1)
                self.assertEqual(source.emitted[-1][1][0], 1)
                self.assertEqual(source._stream_ready_discard_count, 1)
                self.assertTrue(source._stream_pending)

    def test_idle_reset_clears_ready_work_and_cold_resume_keeps_row_zero(self):
        source = OfflineStreamSource("ready_event")
        source.start_ready()
        source.complete_request(1.03)
        source._invalidate_policy_chunks(reason="arm_init_done")
        self.assertIsNone(source._stream_next_chunk)
        self.assertIsNone(source.tick(1.2))
        self.assertEqual(len(source.emitted), 1)
        source.complete_request(1.26)
        source.tick(1.262)
        self.assertEqual(source.published[-1][2]["generation"], 1)
        self.assertEqual(source.published[-1][2]["source_start_index"], 0)

    def test_one_inflight_or_ready_request_and_shutdown_does_not_queue(self):
        source = OfflineStreamSource("ready_event")
        source.tick(1.0)
        first = source._stream_request
        source.tick(1.002)
        self.assertIs(source._stream_request, first)
        source.complete_request(1.01)
        source._request_prefetch({})
        self.assertIsNone(source._stream_request)
        source._stream_next_chunk = None
        source._stream_shutdown = True
        source._request_prefetch({})
        self.assertIsNone(source._stream_request)

    def test_failed_model_result_leaves_dispatcher_idle_and_allows_retry(self):
        for raises in (False, True):
            with self.subTest(raises=raises):
                source = OfflineStreamSource("ready_event")
                source.tick(1.0)

                def sample(payload):
                    source._stream_shutdown = True
                    if raises:
                        raise RuntimeError("deliberate offline inference failure")
                    return None

                source._sample_and_align_chunk = sample
                worker = threading.Thread(target=source._stream_worker)
                worker.start()
                worker.join(1)
                self.assertFalse(worker.is_alive())
                self.assertIsNone(source._stream_next_chunk)
                self.assertFalse(source._stream_pending)
                source._stream_shutdown = False
                self.assertIsNone(source.tick(1.1))
                self.assertTrue(source._stream_pending)
                self.assertEqual(source.emitted, [])

    def test_request_freezes_payload_and_row_before_delayed_worker_start(self):
        source = OfflineStreamSource("ready_event")
        payload = {"observation": [10]}
        source._stream_emitted_policy_steps = 5
        source._request_prefetch(payload)
        payload["observation"][0] = 99
        source._stream_emitted_policy_steps = 9
        finished = threading.Event()

        def sample(observed):
            self.assertEqual(observed, {"observation": [10]})
            source._stream_shutdown = True
            finished.set()
            return source.rows()

        source._sample_and_align_chunk = sample
        worker = threading.Thread(target=source._stream_worker)
        worker.start()
        self.assertTrue(finished.wait(1))
        worker.join(1)
        self.assertFalse(worker.is_alive())
        self.assertEqual(source._stream_next_chunk_metadata["observation_step_seq"], 5)

    def test_subclass_inputs_freeze_once_before_worker_and_inline_inference(self):
        for inline in (False, True):
            with self.subTest(inline=inline):
                source = OfflineStreamSource("ready_event")
                mutable_command = [50.0]
                freezes = []

                def freeze(payload):
                    freezes.append(mutable_command[0])
                    payload["frozen_grip"] = mutable_command[0]
                    return payload

                source._freeze_inference_payload = freeze
                observed = []

                def sample(payload):
                    observed.append(payload.copy())
                    mutable_command[0] = 7.0
                    return source.rows()

                source._sample_and_align_chunk = sample
                payload = {"state": [1]}
                if inline:
                    source._sample_and_align_chunk_timed(payload)
                else:
                    source._request_prefetch(payload)
                    mutable_command[0] = 7.0
                    source._sample_and_align_chunk(source._stream_request[3])
                self.assertEqual(freezes, [50.0])
                self.assertEqual(observed, [{"state": [1], "frozen_grip": 50.0}])
                self.assertEqual(payload, {"state": [1]})

    def test_freeze_failure_does_not_leave_an_unserviceable_pending_request(self):
        source = OfflineStreamSource("ready_event")
        with mock.patch.object(source, "_freeze_inference_payload", side_effect=ValueError("invalid gripper")):
            with self.assertRaisesRegex(ValueError, "invalid gripper"):
                source._request_prefetch({})
        self.assertFalse(source._stream_pending)
        self.assertIsNone(source._stream_request)

    def test_reset_during_worker_discards_old_result_without_erasing_new_request(self):
        source = OfflineStreamSource("ready_event")
        entered = threading.Event()
        release = threading.Event()
        calls = []

        def sample(payload):
            calls.append(payload)
            if len(calls) == 1:
                entered.set()
                if not release.wait(1):
                    raise RuntimeError("test did not release blocked worker")
            else:
                source._stream_shutdown = True
            return source.rows(100 * len(calls))

        source._sample_and_align_chunk = sample
        source._request_prefetch({"epoch": 0})
        worker = threading.Thread(target=source._stream_worker)
        worker.start()
        try:
            self.assertTrue(entered.wait(1))
            source._invalidate_policy_chunks(reason="server_motion_epoch")
            source._request_prefetch({"epoch": 1})
            release.set()
            worker.join(1)
            self.assertFalse(worker.is_alive())
            self.assertEqual(source.stale_completions, 1)
            self.assertEqual(calls, [{"epoch": 0}, {"epoch": 1}])
            self.assertEqual(source._stream_next_chunk_metadata["generation"], 1)
            self.assertEqual(source._stream_next_chunk[0, 0], 200)
        finally:
            release.set()
            with source._stream_cv:
                source._stream_shutdown = True
                source._stream_cv.notify_all()
            worker.join(1)

    def test_ready_mode_rejects_incompatible_fixed_contracts_before_initialization(self):
        for kwargs in (
            {"rtc_enabled": True}, {"stitch_mode": "ensemble"}, {"sequential": True},
            {"anchor_source": "chain"}, {"prefetch_at": 0}, {"training_replay": True},
        ):
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(ValueError):
                    validate_chunk_activation_mode("ready_event", **kwargs)
                self.assertEqual(validate_chunk_activation_mode("fixed_steps", **kwargs), "fixed_steps")
        with self.assertRaises(ValueError):
            validate_chunk_activation_mode("unknown")
        source = OfflineStreamSource()
        with self.assertRaises(ValueError):
            source.configure_chunk_activation("ready_event")


if __name__ == "__main__":
    unittest.main()
