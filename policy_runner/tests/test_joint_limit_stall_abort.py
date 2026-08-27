"""The joint-limit standoff that no controller can break.

A policy can ask for a pose whose blocking joint is already at its bound. The
arm then cannot get closer, the scene barely changes, and the policy asks for
the same pose again. Measured 2026-08-27 (servo_log_20260827_141433.csv): both
elbows sat on their bound for the last 6.4 s of a rollout with the Cartesian
error stuck at 32 mm / 0.081 rad, while the IK asked to CLOSE on the bound 2912
times and to RETREAT zero times. The controller is correct there -- the barrier
refuses only the closing direction and retreat stays free -- so the standoff has
to be ended one level up.
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from policy_runner.robot_state_client import (
    JointLimitStallTracker,
    StateSnapshot,
    joint_limit_stall_from_snapshot,
)


def snapshot(
    *,
    left_clamped: bool = False,
    right_clamped: bool = False,
    joint_index: int = 2,
    q_actual: float = 149.9,
    pos_err_m: float = 0.032,
    ori_err_rad: float = 0.081,
) -> StateSnapshot:
    def arm(clamped: bool) -> dict[str, object]:
        joints = [0.0] * 6
        joints[joint_index] = q_actual
        return {
            "q_actual_deg": joints,
            "cartesian_solve": {
                "safety_joint_limit_clamped": clamped,
                "safety_joint_limit_limited_joint": joint_index,
                "pos_err_m": pos_err_m,
                "ori_err_rad": ori_err_rad,
            },
        }

    return StateSnapshot(
        payload={"left": arm(left_clamped), "right": arm(right_clamped)},
        received_monotonic=0.0,
    )


class ReadbackTest(unittest.TestCase):
    def test_reads_the_incident_shape(self):
        rb = joint_limit_stall_from_snapshot(snapshot(left_clamped=True))
        self.assertTrue(rb.blocked)
        self.assertEqual(rb.arm, "left")
        self.assertEqual(rb.joint_index, 2)          # elbow, 0-indexed
        self.assertAlmostEqual(rb.q_actual_deg, 149.9)
        self.assertAlmostEqual(rb.pos_err_m, 0.032)
        self.assertAlmostEqual(rb.ori_err_rad, 0.081)

    def test_not_blocked_when_no_clamp(self):
        self.assertFalse(joint_limit_stall_from_snapshot(snapshot()).blocked)

    def test_reports_right_arm_when_only_right_is_blocked(self):
        rb = joint_limit_stall_from_snapshot(snapshot(right_clamped=True))
        self.assertTrue(rb.blocked)
        self.assertEqual(rb.arm, "right")

    def test_missing_or_malformed_payload_is_not_a_stall(self):
        for payload in (
            {},
            {"left": None},
            {"left": {}},
            {"left": {"cartesian_solve": None}},
            {"left": {"cartesian_solve": {}}},
            {"left": {"cartesian_solve": {"safety_joint_limit_clamped": "yes"}}},
        ):
            rb = joint_limit_stall_from_snapshot(
                StateSnapshot(payload=payload, received_monotonic=0.0)
            )
            self.assertFalse(rb.blocked, payload)

    def test_out_of_range_joint_index_does_not_crash(self):
        snap = StateSnapshot(
            payload={
                "left": {
                    "q_actual_deg": [0.0] * 6,
                    "cartesian_solve": {
                        "safety_joint_limit_clamped": True,
                        "safety_joint_limit_limited_joint": 99,
                    },
                }
            },
            received_monotonic=0.0,
        )
        rb = joint_limit_stall_from_snapshot(snap)
        self.assertTrue(rb.blocked)
        self.assertIsNone(rb.q_actual_deg)


class TrackerTest(unittest.TestCase):
    def test_fires_only_after_the_hold_elapses(self):
        tracker = JointLimitStallTracker(4.0)
        blocked = joint_limit_stall_from_snapshot(snapshot(left_clamped=True))
        self.assertIsNone(tracker.update(blocked, 0.0))     # first sighting arms it
        self.assertIsNone(tracker.update(blocked, 3.9))
        held = tracker.update(blocked, 4.1)
        self.assertIsNotNone(held)
        self.assertAlmostEqual(held, 4.1, places=6)

    def test_incident_duration_would_have_fired(self):
        """The measured standoff ran 6.4 s; the shipped 4 s hold ends it."""
        tracker = JointLimitStallTracker(4.0)
        blocked = joint_limit_stall_from_snapshot(snapshot(left_clamped=True))
        tracker.update(blocked, 0.0)
        self.assertIsNotNone(tracker.update(blocked, 6.4))

    def test_brushing_a_bound_is_not_a_stall(self):
        """Touching a bound in passing is ordinary -- the barrier exists to make
        it smooth -- so any clear tick restarts the clock."""
        tracker = JointLimitStallTracker(4.0)
        blocked = joint_limit_stall_from_snapshot(snapshot(left_clamped=True))
        clear = joint_limit_stall_from_snapshot(snapshot())
        t = 0.0
        for _ in range(20):
            tracker.update(blocked, t)
            t += 3.0
            self.assertIsNone(tracker.update(clear, t))   # cleared: clock resets
            t += 0.1
        self.assertIsNone(tracker.update(blocked, t))

    def test_moving_to_a_different_joint_restarts(self):
        tracker = JointLimitStallTracker(4.0)
        elbow = joint_limit_stall_from_snapshot(snapshot(left_clamped=True, joint_index=2))
        wrist = joint_limit_stall_from_snapshot(snapshot(left_clamped=True, joint_index=4))
        tracker.update(elbow, 0.0)
        self.assertIsNone(tracker.update(wrist, 3.9))      # new joint, new clock
        self.assertIsNone(tracker.update(wrist, 7.0))      # 3.1 s on the wrist
        self.assertIsNotNone(tracker.update(wrist, 8.1))

    def test_reports_once_per_episode(self):
        tracker = JointLimitStallTracker(4.0)
        blocked = joint_limit_stall_from_snapshot(snapshot(left_clamped=True))
        tracker.update(blocked, 0.0)
        self.assertIsNotNone(tracker.update(blocked, 5.0))
        self.assertIsNone(tracker.update(blocked, 5.1))    # already reported
        self.assertIsNone(tracker.update(blocked, 8.0))
        self.assertIsNotNone(tracker.update(blocked, 12.0))  # a new episode does fire

    def test_disabled_by_zero(self):
        tracker = JointLimitStallTracker(0.0)
        self.assertFalse(tracker.enabled)
        blocked = joint_limit_stall_from_snapshot(snapshot(left_clamped=True))
        tracker.update(blocked, 0.0)
        self.assertIsNone(tracker.update(blocked, 1000.0))


if __name__ == "__main__":
    unittest.main()
