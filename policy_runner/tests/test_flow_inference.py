from __future__ import annotations

import os
import subprocess
import sys
import unittest
from pathlib import Path
from unittest import mock

from policy_runner.action_sources.tcp_pose_target import cartesian_action_requirements


class FlowInferenceCliTest(unittest.TestCase):
    def test_flow_infer_help_lists_rollout_mode_and_controller_sim(self) -> None:
        env = dict(os.environ)
        env["PYTHONPATH"] = "policy_runner"

        result = subprocess.run(
            [sys.executable, "-m", "policy_runner", "flow-infer", "--help"],
            cwd=Path(__file__).resolve().parents[2],
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--rollout-mode", result.stdout)
        self.assertIn("controller_sim", result.stdout)
        self.assertNotIn("--command-family", result.stdout)
        self.assertIn("--policy-dt-sec", result.stdout)
        self.assertIn("--speed-scale", result.stdout)
        # flow-infer always emits absolute TcpPoseTarget setpoints; the old
        # --allow-tcp-target-pose no-op flag was removed, not retained.
        self.assertNotIn("--allow-tcp-target-pose", result.stdout)
        self.assertIn("--sequential-chunk-inference", result.stdout)
        self.assertIn("--rollout-step-log", result.stdout)

    def test_sequential_stream_inference_disables_midchunk_prefetch_kick(self) -> None:
        # Unbound-method call: exercise the kick threshold without constructing
        # the (heavy) action source. sequential -> kick point == execute limit,
        # which chunk_index (< limit) never reaches; default -> early kick (<=2).
        from types import SimpleNamespace

        from policy_runner.flow_inference import FlowMatchingActionSource

        dummy = SimpleNamespace(
            _current_chunk_execute_limit=lambda: 24,
            sequential_stream_inference=True,
        )
        self.assertEqual(FlowMatchingActionSource._stream_prefetch_at(dummy), 24)
        dummy.sequential_stream_inference = False
        self.assertLessEqual(FlowMatchingActionSource._stream_prefetch_at(dummy), 2)
        # A SHORT execute window kicks at 1, not 2: with limit=4 a kick at 2 leaves
        # 2*33.4 = 66.8 ms for an inference measured at p90 67.2 ms, and 20.3% of
        # boundaries stalled. Kicking at 1 leaves 3 steps / 100 ms.
        for limit, expected in ((4, 1), (3, 1), (2, 1), (1, 0)):
            dummy._current_chunk_execute_limit = (lambda n: (lambda: n))(limit)
            self.assertEqual(
                FlowMatchingActionSource._stream_prefetch_at(dummy), expected, f"limit={limit}"
            )
        dummy._current_chunk_execute_limit = lambda: 24
        # explicit kick point (RTC pairing): kicks at the requested index, clamped
        dummy.stream_prefetch_at = 8
        self.assertEqual(FlowMatchingActionSource._stream_prefetch_at(dummy), 8)
        dummy.stream_prefetch_at = 99
        self.assertEqual(FlowMatchingActionSource._stream_prefetch_at(dummy), 23)
        # sequential still wins over the explicit kick point
        dummy.sequential_stream_inference = True
        self.assertEqual(FlowMatchingActionSource._stream_prefetch_at(dummy), 24)

    def test_sequential_stream_stall_uses_explicit_hold(self) -> None:
        from types import SimpleNamespace

        from policy_runner.flow_inference import FlowMatchingActionSource

        previous = object()
        dummy = SimpleNamespace(
            sequential_stream_inference=True,
            timeout_sec=0.37,
            _current_step_intent=previous,
        )

        intent = FlowMatchingActionSource._stream_hold_intent(dummy)

        self.assertEqual(intent.mode, "Hold")
        self.assertEqual(intent.timeout_sec, 0.37)
        self.assertEqual(intent.left, {})
        self.assertEqual(intent.right, {})

        dummy.sequential_stream_inference = False
        self.assertIs(FlowMatchingActionSource._stream_hold_intent(dummy), previous)

    def test_pose_from_state_payload_command_source_and_fallback(self) -> None:
        from policy_runner.flow_dataset import pose_from_state_payload

        payload = {
            "left": {
                "tcp_stand": {"x": 1.0, "y": 0.0, "z": 0.0, "quaternion_xyzw": [0, 0, 0, 1]},
                "tcp_command_stand": {"x": 2.0, "y": 0.0, "z": 0.0, "quaternion_xyzw": [0, 0, 0, 1]},
            },
            "right": {
                "tcp_stand": {"x": 3.0, "y": 0.0, "z": 0.0, "quaternion_xyzw": [0, 0, 0, 1]},
            },
        }
        self.assertEqual(pose_from_state_payload(payload, "left")[0], 1.0)
        self.assertEqual(pose_from_state_payload(payload, "left", source="command")[0], 2.0)
        # command pose absent -> falls back to measured (no zeros)
        self.assertEqual(pose_from_state_payload(payload, "right", source="command")[0], 3.0)
        with self.assertRaises(ValueError):
            pose_from_state_payload(payload, "left", source="nope")

    def test_pose_from_state_payload_uses_reference_in_controller_simulation(self) -> None:
        from policy_runner.flow_dataset import pose_from_state_payload

        gate = {
            "operation_mode": "simulation",
            "physical_motion_expected": False,
            "controller_simulation_servo_state_source": "reference",
        }
        payload = {
            "left": {
                "tcp_stand": {
                    "x": 1.0, "y": 0.0, "z": 0.0,
                    "quaternion_xyzw": [0, 0, 0, 1],
                },
                "tcp_ref_valid": True,
                "tcp_ref_stand": {
                    "x": 2.0, "y": 0.0, "z": 0.0,
                    "quaternion_xyzw": [0, 0, 0, 1],
                },
                "physical_motion_expected": False,
                "cartesian_gate": gate,
            },
            "right": {
                "tcp_stand": {
                    "x": 3.0, "y": 0.0, "z": 0.0,
                    "quaternion_xyzw": [0, 0, 0, 1],
                },
                "tcp_ref_valid": False,
                "physical_motion_expected": False,
                "cartesian_gate": gate,
            },
        }

        self.assertEqual(pose_from_state_payload(payload, "left")[0], 2.0)
        self.assertEqual(pose_from_state_payload(payload, "left", source="actual")[0], 1.0)
        with self.assertRaisesRegex(ValueError, "reference pose"):
            pose_from_state_payload(payload, "right")

    def test_pose_from_state_payload_keeps_actual_source_for_physical_real(self) -> None:
        from policy_runner.flow_dataset import pose_from_state_payload

        payload = {
            "left": {
                "tcp_stand": {
                    "x": 1.0, "y": 0.0, "z": 0.0,
                    "quaternion_xyzw": [0, 0, 0, 1],
                },
                "tcp_ref_valid": True,
                "tcp_ref_stand": {
                    "x": 2.0, "y": 0.0, "z": 0.0,
                    "quaternion_xyzw": [0, 0, 0, 1],
                },
                "physical_motion_expected": True,
                "controller_simulation_mode": None,
                "cartesian_gate": {
                    "operation_mode": "real",
                    "physical_motion_expected": True,
                    "controller_simulation_servo_state_source": "reference",
                },
            }
        }

        self.assertEqual(pose_from_state_payload(payload, "left")[0], 1.0)
        self.assertEqual(
            pose_from_state_payload(payload, "left", source="reference")[0], 2.0
        )

    def test_rtc_shift_prev_chunk_advances_by_executed_window(self) -> None:
        import numpy as np

        from policy_runner.openpi_remote import rtc_shift_prev_chunk

        raw = np.arange(24 * 2, dtype=np.float32).reshape(24, 2)  # H=24
        shifted = rtc_shift_prev_chunk(raw, 16)
        # new[0:d] must pin to the UNEXECUTED tail raw[16:24]
        self.assertTrue(np.array_equal(shifted[:8], raw[16:24]))
        # padded region (index >= H - execute) is zeros (guidance weight = 0 there)
        self.assertTrue(np.all(shifted[8:] == 0.0))
        self.assertEqual(shifted.shape, raw.shape)
        # degenerate cases
        self.assertTrue(np.array_equal(rtc_shift_prev_chunk(raw, 0), raw))
        self.assertTrue(np.all(rtc_shift_prev_chunk(raw, 99) == 0.0))

    def test_external_init_motion_surfaces_done_transition(self) -> None:
        # rb_gui InitMotion button commands the server DIRECTLY (bypassing the
        # arm_init_cmd channel): the controller must still surface the completion
        # as a per-arm DONE transition so the source re-anchors its plan chain.
        from types import SimpleNamespace

        from policy_runner.arm_init_control import ArmInitOverrideController

        ctl = ArmInitOverrideController()

        def snap(status_left: str) -> SimpleNamespace:
            return SimpleNamespace(payload={"init_motion": {
                "left": {"status": status_left},
                "right": {"status": "idle"},
            }})

        ctl.update_from_snapshot(snap("executing"))   # external init in flight
        self.assertEqual(ctl.consume_transitions().done, ())
        ctl.update_from_snapshot(snap("done"))        # completion edge
        transitions = ctl.consume_transitions()
        self.assertEqual(transitions.done, ("left",))
        # steady 'done' afterwards must NOT re-fire
        ctl.update_from_snapshot(snap("done"))
        self.assertEqual(ctl.consume_transitions().done, ())

    def test_controller_sim_requirement_helper_allows_rbpodo_carveout(self) -> None:
        requirements = cartesian_action_requirements(allow_rbpodo_controller_simulation=True)

        self.assertTrue(requirements.allow_rbpodo_controller_simulation_cartesian)
        self.assertTrue(requirements.cartesian_motion)

    def test_server_motion_epoch_invalidates_cached_chunk_and_reanchors(self) -> None:
        import numpy as np

        try:
            from policy_runner.flow_inference import FlowMatchingActionSource
        except ModuleNotFoundError as exc:
            if exc.name == "torch":
                self.skipTest("flow-infer runtime dependency torch is not installed")
            raise
        from policy_runner.robot_state_client import StateSnapshot

        source = FlowMatchingActionSource.__new__(FlowMatchingActionSource)
        source._last_server_motion_epoch = 4
        source._target_pose_by_arm = {
            "left": np.ones(7, dtype=np.float64),
            "right": np.ones(7, dtype=np.float64),
        }
        source._gripper_targets_by_arm = {"left": 0.2, "right": 0.8}
        source._chunk = np.ones((2, 14), dtype=np.float32)
        source._chunk_index = 1
        source._steps_since_boundary = 1
        source._current_step_intent = object()
        source._current_gripper_targets = {"left": 0.2, "right": 0.8}

        snapshot = StateSnapshot(
            payload={
                "motion_epoch": 5,
                "left": {
                    "tcp_stand": {
                        "x": 0.1,
                        "y": 0.2,
                        "z": 0.3,
                        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                    },
                },
                "right": {
                    "tcp_stand": {
                        "x": 0.4,
                        "y": 0.5,
                        "z": 0.6,
                        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                    },
                },
            },
            received_monotonic=0.0,
        )

        source._handle_server_motion_epoch(snapshot)

        self.assertEqual(source._last_server_motion_epoch, 5)
        self.assertIsNone(source._chunk)
        self.assertEqual(source._chunk_index, 0)
        self.assertEqual(source._steps_since_boundary, 0)
        self.assertIsNone(source._target_pose_by_arm["left"])
        self.assertIsNone(source._target_pose_by_arm["right"])
        self.assertAlmostEqual(float(source._reset_left_pose[0]), 0.1)
        self.assertAlmostEqual(float(source._reset_right_pose[0]), 0.4)

    def test_openpi_remote_initializes_server_motion_epoch_state(self) -> None:
        try:
            from policy_runner.openpi_remote import OpenpiRemoteActionSource
        except ModuleNotFoundError as exc:
            if exc.name == "torch":
                self.skipTest("flow-infer runtime dependency torch is not installed")
            raise

        with (
            mock.patch("policy_runner.openpi_remote._OpenpiWebsocketClient") as client_type,
            mock.patch.dict(os.environ, {"OPENPI_REMOTE_SKIP_WARMUP": "1"}),
        ):
            client_type.return_value.fetch_metadata.return_value = {"action_horizon": 24}
            source = OpenpiRemoteActionSource(
                "127.0.0.1:8000",
                action_horizon=24,
                camera_names=(),
            )

        self.assertIsNone(source._last_server_motion_epoch)

    def test_async_inference_timing_tracks_request_completion_activation_and_rolling_stats(self) -> None:
        import threading

        import numpy as np

        from policy_runner.flow_inference import FlowMatchingActionSource

        source = FlowMatchingActionSource.__new__(FlowMatchingActionSource)
        source._init_inference_timing_state()
        source._stream_pending = False
        source._stream_next_chunk = None
        source._stream_request = None
        source._stream_generation = 7
        source._stream_stall_count = 3
        source._stream_lock = threading.Lock()
        source._stream_cv = threading.Condition(source._stream_lock)
        source._inference_clock_ns = iter([1_000_000_000]).__next__

        source._request_prefetch({"observation": [1, 2, 3]})

        generation, inference_seq, request_ns, payload = source._stream_request
        self.assertEqual(generation, 7)
        self.assertEqual(inference_seq, 1)
        self.assertEqual(request_ns, 1_000_000_000)
        self.assertEqual(payload, {"observation": [1, 2, 3]})

        timing = source._record_inference_completion(
            inference_seq=inference_seq,
            request_ns=request_ns,
            worker_start_ns=1_003_000_000,
            worker_end_ns=1_013_000_000,
            ready_ns=1_014_000_000,
        )
        source._stream_next_chunk = np.zeros((2, 14), dtype=np.float32)
        source._stream_ready_timing = timing

        chunk = source._take_prefetched()
        source._record_inference_activation(1.020)
        snapshot = source._inference_timing_snapshot()

        self.assertIsNotNone(chunk)
        self.assertIsNotNone(snapshot)
        assert snapshot is not None
        self.assertEqual(snapshot["seq"], 1)
        self.assertEqual(snapshot["request_monotonic_ns"], 1_000_000_000)
        self.assertEqual(snapshot["worker_start_monotonic_ns"], 1_003_000_000)
        self.assertEqual(snapshot["worker_end_monotonic_ns"], 1_013_000_000)
        self.assertEqual(snapshot["chunk_ready_monotonic_ns"], 1_014_000_000)
        self.assertEqual(snapshot["activation_monotonic_ns"], 1_020_000_000)
        self.assertEqual(snapshot["queue_wait_ms"], 3.0)
        self.assertEqual(snapshot["inference_latency_ms"], 10.0)
        self.assertEqual(snapshot["ready_wait_ms"], 6.0)
        self.assertIsNone(snapshot["inference_period_ms"])
        self.assertEqual(snapshot["stall_count"], 3)
        self.assertEqual(snapshot["rolling"]["inference_latency_ms"]["p95_ms"], 10.0)
        self.assertEqual(snapshot["rolling"]["ready_wait_ms"]["max_ms"], 6.0)

        source._record_inference_completion(
            inference_seq=2,
            request_ns=1_025_000_000,
            worker_start_ns=1_033_000_000,
            worker_end_ns=1_045_000_000,
            ready_ns=1_046_000_000,
        )
        snapshot = source._inference_timing_snapshot()
        assert snapshot is not None
        self.assertEqual(snapshot["inference_period_ms"], 30.0)
        self.assertEqual(snapshot["inference_period_nominal_ms"], 30.0)
        self.assertEqual(snapshot["inference_period_jitter_ms"], 0.0)
        self.assertEqual(snapshot["rolling"]["inference_latency_ms"]["max_ms"], 12.0)

        source._record_inference_completion(
            inference_seq=3,
            request_ns=1_060_000_000,
            worker_start_ns=1_073_000_000,
            worker_end_ns=1_084_000_000,
            ready_ns=1_085_000_000,
        )
        snapshot = source._inference_timing_snapshot()
        assert snapshot is not None
        self.assertEqual(snapshot["inference_period_ms"], 40.0)
        self.assertEqual(snapshot["inference_period_nominal_ms"], 35.0)
        self.assertEqual(snapshot["inference_period_jitter_ms"], 5.0)
        self.assertEqual(snapshot["inference_period_jitter"]["last_ms"], 5.0)
        self.assertEqual(snapshot["inference_period_jitter"]["p95_ms"], 5.0)
        self.assertEqual(snapshot["inference_period_jitter"]["max_ms"], 5.0)
        diagnostics = source.inference_diagnostics_snapshot()
        self.assertEqual(diagnostics["total_inferences"], 3)
        self.assertEqual(diagnostics["retained_inferences"], 3)
        self.assertEqual(diagnostics["events"][0]["timing"]["ready_wait_ms"], 6.0)

    def test_inline_inference_uses_the_same_timing_boundary(self) -> None:
        import numpy as np

        from policy_runner.flow_inference import FlowMatchingActionSource

        source = FlowMatchingActionSource.__new__(FlowMatchingActionSource)
        source._init_inference_timing_state()
        source._stream_stall_count = 0
        source._inference_clock_ns = iter(
            [
                2_000_000_000,
                2_001_000_000,
                2_011_000_000,
                2_012_000_000,
                2_013_000_000,
            ]
        ).__next__
        expected = np.zeros((2, 14), dtype=np.float32)
        source._sample_and_align_chunk = lambda payload: expected

        chunk = source._sample_and_align_chunk_timed({"observation": [4, 5, 6]})
        # The loop timestamp predates blocking inline inference; activation must
        # fall back to a fresh monotonic reading instead of reporting time travel.
        source._record_inference_activation(1.999)
        snapshot = source._inference_timing_snapshot()

        self.assertIs(chunk, expected)
        self.assertIsNotNone(snapshot)
        assert snapshot is not None
        self.assertEqual(snapshot["seq"], 1)
        self.assertEqual(snapshot["queue_wait_ms"], 1.0)
        self.assertEqual(snapshot["inference_latency_ms"], 10.0)
        self.assertEqual(snapshot["ready_wait_ms"], 1.0)
        self.assertEqual(snapshot["activation_monotonic_ns"], 2_013_000_000)


    def test_chunk_activation_skips_only_policy_steps_emitted_after_observation(self) -> None:
        import numpy as np

        from policy_runner.flow_inference import FlowMatchingActionSource

        source = FlowMatchingActionSource.__new__(FlowMatchingActionSource)
        source._init_inference_timing_state()
        source._stream_activation_candidate_timing = None
        source._stream_activation_candidate_metadata = {
            "observation_step_seq": 10,
            "observation_bundle_seq": 123,
        }
        source._stream_emitted_policy_steps = 13
        source._last_overlay_payload = None
        source._chunk_crossfade_steps = 0
        source.policy_dt_sec = 1.0 / 30.0
        source._print_chunk_enabled = False
        source._print_tracking_enabled = False
        source._chunk_overlay_publisher = None
        source._overlay_chain_pending = None
        source._inference_clock_ns = lambda: 1_000_000_000

        chunk = np.arange(8 * 14, dtype=np.float32).reshape(8, 14)
        source._activate_chunk(chunk, now_monotonic=1.0)

        self.assertTrue(np.array_equal(source._chunk, chunk[3:]))
        self.assertEqual(source._active_chunk_metadata["source_start_index"], 3)
        self.assertEqual(source._active_chunk_metadata["selected_horizon"], 5)

    def test_cold_start_alignment_keeps_row_zero(self) -> None:
        import numpy as np

        from policy_runner.flow_inference import FlowMatchingActionSource

        source = FlowMatchingActionSource.__new__(FlowMatchingActionSource)
        source._init_inference_timing_state()
        source._stream_activation_candidate_timing = None
        source._stream_activation_candidate_metadata = {"observation_step_seq": 0}
        source._stream_emitted_policy_steps = 0
        source._last_overlay_payload = None
        source._chunk_crossfade_steps = 0
        source.policy_dt_sec = 1.0 / 30.0
        source._print_chunk_enabled = False
        source._print_tracking_enabled = False
        source._chunk_overlay_publisher = None
        source._overlay_chain_pending = None
        source._inference_clock_ns = lambda: 1_000_000_000

        chunk = np.arange(4 * 14, dtype=np.float32).reshape(4, 14)
        source._activate_chunk(chunk, now_monotonic=1.0)

        self.assertTrue(np.array_equal(source._chunk, chunk))
        self.assertEqual(source._active_chunk_metadata["source_start_index"], 0)


if __name__ == "__main__":
    unittest.main()
