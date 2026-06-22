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


def _make_source(*, absolute: bool) -> "FlowMatchingActionSource":
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


if __name__ == "__main__":
    unittest.main()
