"""Unit tests for --gripper-proprio-source (which gripper signal the model sees).

The proprio gripper channel is an ABSOLUTE opening percent. Three sources:
  actual  -- the MEASURED jaw (default; matches the pika UMI converter, which
             takes gripper col0 = measured, not col1 = commanded)
  command -- the opening this runner last SENT (no sample staleness, no contact
             signal)
  hybrid  -- command while the jaw travels, measured once it stalls on something

Sources are built via __new__ (no checkpoint / torch model load), so only the
attributes the resolver touches are set.
"""

from __future__ import annotations

import unittest

try:
    import numpy as np

    from policy_runner.flow_inference import FlowMatchingActionSource
except Exception:  # torch (a flow_inference import) may be absent in this env
    np = None
    FlowMatchingActionSource = None  # type: ignore[assignment]


def _payload(*, left_pct: float | None = None, right_pct: float | None = None):
    """A state payload shaped like rb_servo_server's armStateJson gripper block.

    Carries BOTH `percent` and `target_percent`: the measured resolver must pick
    `percent` even though the legacy _gripper_value_from_payload search order
    puts `target_percent` first.
    """
    payload = {}
    for arm, pct in (("left", left_pct), ("right", right_pct)):
        if pct is None:
            continue
        payload[arm] = {
            "gripper": {
                "valid": True,
                "stale": False,
                "percent": float(pct),
                "target_percent": 99.0,
                "moving": False,
                "ok": True,
            }
        }
    return payload


def _make_source(
    *,
    source_mode: str = "actual",
    last_sent: dict | None = None,
    jam_gap_percent: float = 3.0,
    jam_stall_steps: int = 3,
    jam_move_eps_percent: float = 0.5,
) -> "FlowMatchingActionSource":
    assert FlowMatchingActionSource is not None
    src = FlowMatchingActionSource.__new__(FlowMatchingActionSource)
    src.gripper_proprio_source = source_mode
    src.gripper_proprio_jam_gap_percent = float(jam_gap_percent)
    src.gripper_proprio_jam_stall_steps = int(jam_stall_steps)
    src.gripper_proprio_jam_move_eps_percent = float(jam_move_eps_percent)
    src._gripper_targets_by_arm = {"left": None, "right": None}
    src._gripper_last_sent_by_arm = dict(last_sent or {"left": None, "right": None})
    src.gripper_runtime = None
    return src


@unittest.skipIf(FlowMatchingActionSource is None, "torch is not installed")
class GripperProprioSourceTest(unittest.TestCase):
    def test_actual_reads_measured_percent_not_the_target(self):
        # target_percent is 99.0 in the payload; the measured channel must win.
        src = _make_source(source_mode="actual", last_sent={"right": 0.0})
        value = src._proprio_gripper_percent(_payload(right_pct=17.5), "right")
        self.assertAlmostEqual(value, 17.5)
        self.assertEqual(src._gripper_proprio_last_source["right"], "actual")

    def test_command_reports_the_last_sent_opening(self):
        src = _make_source(source_mode="command", last_sent={"right": 0.0})
        value = src._proprio_gripper_percent(_payload(right_pct=17.5), "right")
        self.assertAlmostEqual(value, 0.0)
        self.assertEqual(src._gripper_proprio_last_source["right"], "command")

    def test_command_falls_back_to_measured_before_anything_is_sent(self):
        # Rollout start: nothing commanded yet. Reporting "command" here would
        # hide that the mode never engaged, so the source is labelled distinctly.
        src = _make_source(source_mode="command", last_sent={"right": None})
        value = src._proprio_gripper_percent(_payload(right_pct=42.0), "right")
        self.assertAlmostEqual(value, 42.0)
        self.assertEqual(src._gripper_proprio_last_source["right"], "actual_no_command")

    def test_hybrid_prefers_command_while_the_jaw_is_travelling(self):
        src = _make_source(source_mode="hybrid", last_sent={"right": 0.0})
        # Measured is far from the command but still moving -> travel, not a jam.
        for pct in (24.0, 18.0, 12.0, 6.0):
            src._note_gripper_actual("right", pct)
        value = src._proprio_gripper_percent(_payload(right_pct=6.0), "right")
        self.assertAlmostEqual(value, 0.0)
        self.assertEqual(src._gripper_proprio_last_source["right"], "hybrid_free")

    def test_hybrid_switches_to_measured_once_the_jaw_stalls_on_an_object(self):
        src = _make_source(source_mode="hybrid", last_sent={"right": 0.0})
        # Jaw parked at the bolt width while commanded fully closed.
        for pct in (8.2, 8.1, 8.2, 8.1):
            src._note_gripper_actual("right", pct)
        value = src._proprio_gripper_percent(_payload(right_pct=8.1), "right")
        self.assertAlmostEqual(value, 8.1)
        self.assertEqual(src._gripper_proprio_last_source["right"], "hybrid_jam")

    def test_hybrid_needs_a_full_stall_window_before_calling_a_jam(self):
        src = _make_source(source_mode="hybrid", last_sent={"right": 0.0})
        src._note_gripper_actual("right", 8.2)
        src._note_gripper_actual("right", 8.1)  # only 2 samples, window is 3
        value = src._proprio_gripper_percent(_payload(right_pct=8.1), "right")
        self.assertAlmostEqual(value, 0.0)
        self.assertEqual(src._gripper_proprio_last_source["right"], "hybrid_free")

    def test_hybrid_ignores_a_gap_inside_the_tolerance(self):
        src = _make_source(source_mode="hybrid", last_sent={"right": 10.0})
        for pct in (12.0, 12.0, 12.0, 12.0):
            src._note_gripper_actual("right", pct)
        # 2 pp gap <= 3 pp tolerance -> tracking, not jammed.
        value = src._proprio_gripper_percent(_payload(right_pct=12.0), "right")
        self.assertAlmostEqual(value, 10.0)
        self.assertEqual(src._gripper_proprio_last_source["right"], "hybrid_free")

    def test_history_is_bounded_by_the_stall_window(self):
        src = _make_source(source_mode="hybrid", jam_stall_steps=3)
        for pct in range(40):
            src._note_gripper_actual("right", float(pct))
        self.assertLessEqual(len(src._gripper_actual_history["right"]), 4)

    def test_unknown_source_is_rejected_rather_than_silently_defaulted(self):
        src = _make_source(source_mode="measured", last_sent={"right": 0.0})
        with self.assertRaises(ValueError):
            src._proprio_gripper_percent(_payload(right_pct=17.5), "right")

    def test_stale_or_invalid_feedback_is_not_read_as_measured(self):
        src = _make_source(source_mode="actual")
        payload = {"right": {"gripper": {"valid": False, "stale": True, "percent": 5.0}}}
        src.gripper_runtime = None
        # No usable measurement and no integrated target -> None, never a fake 0.
        self.assertIsNone(src._proprio_gripper_percent(payload, "right"))

    def test_telemetry_reports_the_value_handed_to_the_policy(self):
        src = _make_source(source_mode="command", last_sent={"left": 55.0, "right": 0.0})
        src._proprio_gripper_percent(_payload(left_pct=60.0, right_pct=17.5), "left")
        src._proprio_gripper_percent(_payload(left_pct=60.0, right_pct=17.5), "right")
        telemetry = src._gripper_proprio_telemetry()
        self.assertAlmostEqual(telemetry["left"]["pct"], 55.0)
        self.assertEqual(telemetry["left"]["source"], "command")
        self.assertAlmostEqual(telemetry["right"]["pct"], 0.0)
        self.assertEqual(telemetry["right"]["source"], "command")


if __name__ == "__main__":
    unittest.main()
