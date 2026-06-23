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
    source.gripper_binary = bool(binary)
    source.gripper_open_percent = float(open_percent)
    source.gripper_close_percent = float(close_percent)
    source.gripper_binary_threshold = float(binary_threshold)
    source.gripper_open_hold_steps = int(open_hold_steps)
    source._gripper_integrate_count = 0
    source._gripper_hold_open_now = False
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
