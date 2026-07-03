from __future__ import annotations

import os
import subprocess
import sys
import unittest
from pathlib import Path

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
        # explicit kick point (RTC pairing): kicks at the requested index, clamped
        dummy.stream_prefetch_at = 8
        self.assertEqual(FlowMatchingActionSource._stream_prefetch_at(dummy), 8)
        dummy.stream_prefetch_at = 99
        self.assertEqual(FlowMatchingActionSource._stream_prefetch_at(dummy), 23)
        # sequential still wins over the explicit kick point
        dummy.sequential_stream_inference = True
        self.assertEqual(FlowMatchingActionSource._stream_prefetch_at(dummy), 24)

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


if __name__ == "__main__":
    unittest.main()
