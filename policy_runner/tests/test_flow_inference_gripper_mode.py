"""Unit tests for the --gripper-action-mode branch (absolute vs delta).

The action gripper dim is interpreted either as an ABSOLUTE next-step opening
percent (default; the latest openpi `--gripper-mode absolute` checkpoints) or as
a per-step opening DELTA to integrate (legacy checkpoints). Both the motion-packet
target (`_integrate_gripper_targets`) and the serial-backend command
(`_dispatch_gripper_step`) must honour the same `gripper_action_absolute` flag.

These build the source via __new__ (no checkpoint / torch model load) and set only
the attributes the two methods touch, so the gripper logic is exercised in
isolation. Guarded by the same torch-availability skip as the sibling flow tests
(flow_inference imports torch at module load).
"""

from __future__ import annotations

import unittest

try:
    import numpy as np

    from policy_runner.flow_inference import FlowMatchingActionSource
except Exception:  # torch (a flow_inference import) may be absent in this env
    np = None
    FlowMatchingActionSource = None  # type: ignore[assignment]

from policy_runner.gripper import GripperRuntime, NoopGripperBackend


def _step(left_grip: float, right_grip: float):
    values = [0.0] * 14
    values[6] = float(left_grip)
    values[13] = float(right_grip)
    return np.asarray(values, dtype=np.float32)


def _make_source(
    *,
    absolute: bool,
    binary: bool = False,
    open_percent: float = 50.0,
    close_percent: float = 7.0,
    binary_threshold: float = 50.0,
    open_hold_steps: int = 0,
    close_snap_percent: float = 0.0,
    close_bias_left: float | None = None,
    close_bias_right: float | None = None,
    command_deadband_percent: float = 0.0,
) -> "FlowMatchingActionSource":
    assert FlowMatchingActionSource is not None
    source = FlowMatchingActionSource.__new__(FlowMatchingActionSource)
    source.arm_mask = np.asarray([1.0, 1.0], dtype=np.float32)
    source.gripper_command_source = "flow_policy"
    # sim_dryrun lets _integrate_gripper_targets fold the target into the motion
    # packet; dispatch is gated to a logged-noop but still records the command
    # (so its command_type is observable on runtime.results).
    source.gripper_runtime = GripperRuntime(
        rollout_mode="sim_dryrun", backend=NoopGripperBackend()
    )
    source._gripper_targets_by_arm = {"left": None, "right": None}
    source.gripper_action_absolute = bool(absolute)
    source.gripper_close_bias = 0.0
    # Per-arm close-bias overrides: None -> _gripper_close_bias falls back to the
    # shared gripper_close_bias above (so existing single-bias tests are unchanged).
    source.gripper_close_bias_left = close_bias_left
    source.gripper_close_bias_right = close_bias_right
    source.gripper_binary = bool(binary)
    source.gripper_open_percent = float(open_percent)
    source.gripper_close_percent = float(close_percent)
    source.gripper_binary_threshold = float(binary_threshold)
    source.gripper_close_snap_percent = float(close_snap_percent)
    source.gripper_open_hold_steps = int(open_hold_steps)
    source._gripper_integrate_count = 0
    source._gripper_hold_open_now = False
    source.gripper_command_deadband_percent = float(command_deadband_percent)
    source._gripper_last_sent_by_arm = {"left": None, "right": None}
    return source


@unittest.skipIf(FlowMatchingActionSource is None, "torch is not installed")
class GripperActionModeTest(unittest.TestCase):
    def test_absolute_sets_motion_packet_target_directly_and_clamps(self) -> None:
        source = _make_source(absolute=True)
        # Pre-seed a running target to prove ABSOLUTE overwrites (does not integrate).
        source._gripper_targets_by_arm = {"left": 50.0, "right": 50.0}
        targets = source._integrate_gripper_targets(_step(42.0, 120.0), payload={})
        self.assertAlmostEqual(targets["left"], 42.0)
        # 120 -> clamped to the 0..100 opening range.
        self.assertAlmostEqual(targets["right"], 100.0)

    def test_delta_integrates_onto_running_motion_packet_target(self) -> None:
        source = _make_source(absolute=False)
        source._gripper_targets_by_arm = {"left": 50.0, "right": 50.0}
        targets = source._integrate_gripper_targets(_step(10.0, -5.0), payload={})
        self.assertAlmostEqual(targets["left"], 60.0)
        self.assertAlmostEqual(targets["right"], 45.0)

    def test_absolute_dispatches_target_command_type(self) -> None:
        source = _make_source(absolute=True)
        source._dispatch_gripper_step(_step(42.0, 88.0))
        types = [r.command.command_type for r in source.gripper_runtime.results]
        self.assertEqual(types, ["target", "target"])

    def test_delta_dispatches_delta_command_type(self) -> None:
        source = _make_source(absolute=False)
        source._dispatch_gripper_step(_step(42.0, 88.0))
        types = [r.command.command_type for r in source.gripper_runtime.results]
        self.assertEqual(types, ["delta", "delta"])

    def test_close_bias_lowers_absolute_motion_packet_target(self) -> None:
        source = _make_source(absolute=True)
        source.gripper_close_bias = 1.0
        targets = source._integrate_gripper_targets(_step(18.0, 0.5), payload={})
        self.assertAlmostEqual(targets["left"], 17.0)   # 18 - 1
        self.assertAlmostEqual(targets["right"], 0.0)   # 0.5 - 1 -> clamped to 0

    def test_close_bias_lowers_absolute_dispatch_value(self) -> None:
        source = _make_source(absolute=True)
        source.gripper_close_bias = 1.0
        source._dispatch_gripper_step(_step(18.0, 88.0))
        values = [r.command.value for r in source.gripper_runtime.results]
        self.assertEqual(values, [17.0, 87.0])

    def test_close_bias_is_applied_once_per_step_and_never_compounds(self) -> None:
        """The bias must NOT ratchet inside the runner.

        `step` is a VIEW into the live chunk (`self._chunk[self._chunk_index]`),
        and dispatch assigns to `step[6]` / `step[13]`. Without the defensive copy
        in _dispatch_gripper_step, the mapped value would be written back into the
        chunk, and every consumer that re-read that row would subtract the bias
        again -- a per-re-read ratchet toward closed, which is exactly what a
        biased grasp would look like if it drifted shut on its own.
        """
        source = _make_source(absolute=True)
        source.gripper_close_bias = 1.0
        chunk = np.zeros((4, 14), dtype=np.float32)
        chunk[:, 6] = 40.0
        chunk[:, 13] = 60.0
        row = chunk[0]  # exactly what the runner hands to integrate + dispatch
        targets = source._integrate_gripper_targets(row, payload={})
        source._dispatch_gripper_step(row)
        self.assertAlmostEqual(targets["left"], 39.0)
        self.assertAlmostEqual(targets["right"], 59.0)
        # the model's raw action survives untouched in the chunk
        self.assertAlmostEqual(float(chunk[0, 6]), 40.0)
        self.assertAlmostEqual(float(chunk[0, 13]), 60.0)
        # and both sinks agreed on ONE bias application
        values = [r.command.value for r in source.gripper_runtime.results]
        self.assertEqual(values, [39.0, 59.0])

    def test_close_snap_collapses_near_closed_absolute_target(self) -> None:
        # close-snap=10: a mapped opening strictly below 10% snaps to 0 (fully
        # closed); values at/above the threshold pass through unchanged.
        source = _make_source(absolute=True, close_snap_percent=10.0)
        targets = source._integrate_gripper_targets(_step(8.0, 10.0), payload={})
        self.assertAlmostEqual(targets["left"], 0.0)    # 8 < 10 -> snap closed
        self.assertAlmostEqual(targets["right"], 10.0)  # 10 not < 10 -> kept

    def test_close_snap_collapses_near_closed_dispatch_value(self) -> None:
        source = _make_source(absolute=True, close_snap_percent=10.0)
        source._dispatch_gripper_step(_step(8.0, 42.0))
        values = [r.command.value for r in source.gripper_runtime.results]
        self.assertEqual(values, [0.0, 42.0])  # left snaps to closed, right unchanged

    def test_close_snap_applies_after_close_bias(self) -> None:
        # The bias lands the opening just under the snap threshold -> closes fully.
        source = _make_source(absolute=True, close_snap_percent=10.0)
        source.gripper_close_bias = 3.0
        targets = source._integrate_gripper_targets(_step(12.0, 0.0), payload={})
        self.assertAlmostEqual(targets["left"], 0.0)   # 12 - 3 = 9 < 10 -> snap
        self.assertAlmostEqual(targets["right"], 0.0)

    def test_close_snap_off_by_default_keeps_small_openings(self) -> None:
        source = _make_source(absolute=True)  # close_snap_percent defaults 0 (off)
        targets = source._integrate_gripper_targets(_step(8.0, 3.0), payload={})
        self.assertAlmostEqual(targets["left"], 8.0)
        self.assertAlmostEqual(targets["right"], 3.0)

    def test_close_bias_ignored_in_delta_mode(self) -> None:
        source = _make_source(absolute=False)
        source.gripper_close_bias = 1.0  # no effect in delta mode
        source._gripper_targets_by_arm = {"left": 50.0, "right": 50.0}
        targets = source._integrate_gripper_targets(_step(10.0, -5.0), payload={})
        self.assertAlmostEqual(targets["left"], 60.0)   # integrated, not biased
        self.assertAlmostEqual(targets["right"], 45.0)
        source._dispatch_gripper_step(_step(10.0, -5.0))
        values = [r.command.value for r in source.gripper_runtime.results]
        self.assertEqual(values, [10.0, -5.0])          # raw deltas, unbiased

    def test_close_bias_zero_is_noop(self) -> None:
        source = _make_source(absolute=True)  # default bias 0.0
        targets = source._integrate_gripper_targets(_step(42.0, 88.0), payload={})
        self.assertAlmostEqual(targets["left"], 42.0)
        self.assertAlmostEqual(targets["right"], 88.0)

    def test_per_arm_close_bias_overrides_shared_motion_packet(self) -> None:
        # Independent per-arm close-bias: left subtracts 2, right subtracts 6.
        source = _make_source(absolute=True, close_bias_left=2.0, close_bias_right=6.0)
        targets = source._integrate_gripper_targets(_step(50.0, 50.0), payload={})
        self.assertAlmostEqual(targets["left"], 48.0)   # 50 - 2
        self.assertAlmostEqual(targets["right"], 44.0)  # 50 - 6

    def test_per_arm_close_bias_overrides_shared_dispatch(self) -> None:
        # The PHYSICAL serial-backend dispatch must apply the same per-arm bias.
        source = _make_source(absolute=True, close_bias_left=2.0, close_bias_right=6.0)
        source._dispatch_gripper_step(_step(50.0, 50.0))
        values = [r.command.value for r in source.gripper_runtime.results]
        self.assertEqual(values, [48.0, 44.0])  # [left 50-2, right 50-6]

    def test_per_arm_close_bias_falls_back_to_shared_when_unset(self) -> None:
        # Left has an override (1.0); right override is unset (None) -> right uses
        # the shared gripper_close_bias base (3.0).
        source = _make_source(absolute=True, close_bias_left=1.0)
        source.gripper_close_bias = 3.0
        targets = source._integrate_gripper_targets(_step(20.0, 20.0), payload={})
        self.assertAlmostEqual(targets["left"], 19.0)   # 20 - 1 (per-arm override)
        self.assertAlmostEqual(targets["right"], 17.0)  # 20 - 3 (shared fallback)

    def test_per_arm_close_bias_ignored_in_binary_mode(self) -> None:
        # binary snaps to the open/close presets; per-arm bias must not apply.
        source = _make_source(
            absolute=True, binary=True, close_bias_left=2.0, close_bias_right=6.0
        )
        targets = source._integrate_gripper_targets(_step(88.0, 12.0), payload={})
        self.assertAlmostEqual(targets["left"], 50.0)   # open preset, no bias
        self.assertAlmostEqual(targets["right"], 7.0)   # close preset, no bias


@unittest.skipIf(FlowMatchingActionSource is None, "torch is not installed")
class GripperBinaryModeTest(unittest.TestCase):
    def test_binary_snaps_motion_packet_target_to_open_close(self) -> None:
        source = _make_source(absolute=True, binary=True)  # open=50, close=7, thr=50
        # >= threshold -> OPEN preset; < threshold -> CLOSE preset (NOT the raw value).
        targets = source._integrate_gripper_targets(_step(88.0, 12.0), payload={})
        self.assertAlmostEqual(targets["left"], 50.0)   # 88 >= 50 -> open
        self.assertAlmostEqual(targets["right"], 7.0)   # 12 < 50 -> close

    def test_binary_snaps_dispatch_values_to_open_close(self) -> None:
        source = _make_source(absolute=True, binary=True)
        source._integrate_gripper_targets(_step(88.0, 12.0), payload={})  # sets hold flag
        source._dispatch_gripper_step(_step(88.0, 12.0))
        values = [r.command.value for r in source.gripper_runtime.results]
        types = [r.command.command_type for r in source.gripper_runtime.results]
        self.assertEqual(values, [50.0, 7.0])
        self.assertEqual(types, ["target", "target"])   # binary uses the target path

    def test_binary_threshold_boundary_is_open(self) -> None:
        source = _make_source(absolute=True, binary=True, binary_threshold=25.0)
        targets = source._integrate_gripper_targets(_step(25.0, 24.999), payload={})
        self.assertAlmostEqual(targets["left"], 50.0)   # exactly at threshold -> open
        self.assertAlmostEqual(targets["right"], 7.0)   # just below -> close

    def test_binary_custom_presets(self) -> None:
        source = _make_source(
            absolute=True, binary=True, open_percent=70.0, close_percent=3.0
        )
        targets = source._integrate_gripper_targets(_step(99.0, 1.0), payload={})
        self.assertAlmostEqual(targets["left"], 70.0)
        self.assertAlmostEqual(targets["right"], 3.0)


@unittest.skipIf(FlowMatchingActionSource is None, "torch is not installed")
class GripperOpenHoldStepsTest(unittest.TestCase):
    def test_hold_open_forces_dispatch_open_then_releases(self) -> None:
        # The serial-backend dispatch drives the PHYSICAL gripper in real_policy,
        # so the reach-before-grasp hold must force it OPEN there too (not only on
        # the motion-packet target). Hold 2 steps, binary open preset = 50.
        source = _make_source(absolute=True, binary=True, open_hold_steps=2)
        for _ in range(2):
            # The model says CLOSE (12 < threshold) but the hold window overrides to OPEN.
            source._integrate_gripper_targets(_step(12.0, 12.0), payload={})
            source._dispatch_gripper_step(_step(12.0, 12.0))
        # Step 3: hold elapsed -> the policy's CLOSE command is honoured.
        source._integrate_gripper_targets(_step(12.0, 12.0), payload={})
        source._dispatch_gripper_step(_step(12.0, 12.0))
        values = [r.command.value for r in source.gripper_runtime.results]
        # 3 steps x 2 arms = 6 commands: [open,open, open,open, close,close]
        self.assertEqual(values, [50.0, 50.0, 50.0, 50.0, 7.0, 7.0])

    def test_hold_open_absolute_uses_full_open(self) -> None:
        source = _make_source(absolute=True, open_hold_steps=1)
        source._integrate_gripper_targets(_step(12.0, 12.0), payload={})
        source._dispatch_gripper_step(_step(12.0, 12.0))
        values = [r.command.value for r in source.gripper_runtime.results]
        self.assertEqual(values, [100.0, 100.0])   # absolute hold = fully open


if __name__ == "__main__":
    unittest.main()


@unittest.skipIf(FlowMatchingActionSource is None, "torch is not installed")
class GripperCommandDeadbandTest(unittest.TestCase):
    """The jaw is re-targeted every policy step with no rate limit; the deadband
    re-holds the last SENT value so per-step model jitter does not reach it."""

    @staticmethod
    def _drive(source, right_openings):
        out = []
        for value in right_openings:
            step = np.zeros(14, dtype=np.float32)
            step[6] = 80.0          # left parked wide open
            step[13] = float(value)
            out.append(source._integrate_gripper_targets(step, {})["right"])
        return out

    def test_off_by_default_passes_every_jitter_through(self) -> None:
        source = _make_source(absolute=True)
        self.assertEqual(self._drive(source, [40.0, 42.0, 40.5, 43.0]),
                         [40.0, 42.0, 40.5, 43.0])

    def test_sub_deadband_jitter_is_held(self) -> None:
        source = _make_source(absolute=True, command_deadband_percent=5.0)
        # 42.0/40.5 are within 5% of the latched 40.0 -> held; 47.0 exceeds it.
        self.assertEqual(self._drive(source, [40.0, 42.0, 40.5, 47.0, 48.0]),
                         [40.0, 40.0, 40.0, 47.0, 47.0])

    def test_full_close_is_never_suppressed(self) -> None:
        # The failure this exemption exists for: a jaw latched at 4% must still
        # accept a close-snapped 0% under a 5% deadband, or the grasp never happens.
        source = _make_source(absolute=True, command_deadband_percent=5.0,
                              close_snap_percent=15.0)
        self.assertEqual(self._drive(source, [4.0, 3.0]), [0.0, 0.0])

    def test_release_from_full_close_is_never_suppressed(self) -> None:
        source = _make_source(absolute=True, command_deadband_percent=5.0)
        # 0 -> 2 is a 2% move, inside the deadband, but leaving the closed state
        # must not be swallowed either.
        self.assertEqual(self._drive(source, [0.0, 2.0]), [0.0, 2.0])

    def test_dispatch_path_sees_the_same_command(self) -> None:
        source = _make_source(absolute=True, command_deadband_percent=5.0)
        self._drive(source, [40.0])
        step = np.zeros(14, dtype=np.float32)
        step[13] = 42.0     # jitter within the deadband
        source._integrate_gripper_targets(step, {})
        source._dispatch_gripper_step(step)
        values = [r.command.value for r in source.gripper_runtime.results]
        # [left, right]; the right jaw must see the HELD 40.0, not the 42.0 jitter.
        self.assertEqual(len(values), 2)
        self.assertAlmostEqual(values[1], 40.0, places=6)


@unittest.skipIf(FlowMatchingActionSource is None, "torch is not installed")
class ChunkKnotFilterTest(unittest.TestCase):
    """Zero-phase FIR over the chunk's pose deltas (not the gripper)."""

    @staticmethod
    def _source(taps=None):
        s = FlowMatchingActionSource.__new__(FlowMatchingActionSource)
        s._chunk_knot_filter_taps = taps
        s._chunk_knot_filter_context = None
        return s

    def test_off_by_default_returns_the_chunk_unchanged(self) -> None:
        s = self._source(None)
        c = np.random.RandomState(0).randn(19, 14).astype(np.float32)
        self.assertIs(s._band_limit_chunk(c), c)

    def test_gripper_channel_is_never_filtered(self) -> None:
        # The gripper is near-binary; low-passing it would blunt the close.
        s = self._source(np.ones(5) / 5.0)
        c = np.zeros((19, 14), dtype=np.float32)
        c[:, 6] = np.tile([0.0, 100.0], 10)[:19]     # left gripper square wave
        c[:, 13] = np.tile([100.0, 0.0], 10)[:19]    # right gripper square wave
        out = s._band_limit_chunk(c)
        np.testing.assert_allclose(out[:, 6], c[:, 6])
        np.testing.assert_allclose(out[:, 13], c[:, 13])

    def test_constant_motion_is_preserved(self) -> None:
        # A normalised kernel must pass DC untouched: steady travel keeps its rate.
        s = self._source(np.ones(5) / 5.0)
        c = np.zeros((19, 14), dtype=np.float32)
        c[:, 0] = 2.0
        out = s._band_limit_chunk(c)
        np.testing.assert_allclose(out[:, 0], 2.0, atol=1e-5)

    def test_alternating_ripple_is_attenuated(self) -> None:
        s = self._source(np.ones(5) / 5.0)
        c = np.zeros((19, 14), dtype=np.float32)
        c[:, 0] = np.where(np.arange(19) % 2 == 0, 1.0, -1.0)   # Nyquist ripple
        out = s._band_limit_chunk(c)
        self.assertLess(float(np.std(out[2:-2, 0])), 0.25 * float(np.std(c[:, 0])))

    def test_context_carries_across_chunks_instead_of_edge_padding(self) -> None:
        # The backward half must come from the PREVIOUS chunk, or the rows about
        # to execute get distorted by replicated edge samples.
        s = self._source(np.ones(5) / 5.0)
        first = np.zeros((19, 14), dtype=np.float32); first[:, 0] = 1.0
        s._band_limit_chunk(first)
        self.assertIsNotNone(s._chunk_knot_filter_context)
        self.assertEqual(s._chunk_knot_filter_context.shape[0], 2)
        second = np.zeros((19, 14), dtype=np.float32); second[:, 0] = 1.0
        out = s._band_limit_chunk(second)
        # Continuous 1.0 stream across the seam -> row 0 must stay 1.0, not sag.
        self.assertAlmostEqual(float(out[0, 0]), 1.0, places=5)

    def test_short_chunk_is_passed_through(self) -> None:
        s = self._source(np.ones(7) / 7.0)
        c = np.ones((2, 14), dtype=np.float32)
        np.testing.assert_allclose(s._band_limit_chunk(c), c)
