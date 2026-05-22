from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from policy_runner.action_sources.spacemouse_joint_velocity import SpaceMouseJointVelocitySource
from policy_runner.robot_state_client import StateSnapshot
from policy_runner.safety import SafetyConfig, SafetyGate
from policy_runner.spacemouse import FakeSpaceMouseReader, SpaceMouseSample


def snapshot():
    return StateSnapshot(
        payload={
            "motion_state": "ConnectedHold",
            "fault_latched": False,
            "left": {"has_valid_joint_state": True, "q_actual_deg": [0, 0, 0, 0, 0, 0]},
            "right": {"has_valid_joint_state": True, "q_actual_deg": [0, 0, 0, 0, 0, 0]},
        },
        received_monotonic=time.monotonic(),
    )


class SpaceMouseMappingTest(unittest.TestCase):
    def test_fake_sample_maps_to_deterministic_clamped_joint_velocity(self):
        reader = FakeSpaceMouseReader([
            SpaceMouseSample(
                tx=2.0,
                ty=0.03,
                tz=-1.5,
                rx=0.5,
                ry=-0.5,
                rz=1.0,
                buttons=(True,),
                timestamp_monotonic=1.0,
            )
        ])
        source = SpaceMouseJointVelocitySource(
            reader=reader,
            max_joint_velocity_deg_s=(5, 5, 5, 8, 8, 10),
            selected_arm="left",
            deadband=0.08,
            smoothing_alpha=1.0,
        )
        intent = source.next_intent(snapshot(), time.monotonic())
        self.assertEqual(intent.left["mode"], "JointVelocity")
        self.assertEqual(intent.left["dq_target_deg_s"], [5.0, 0.0, -5.0, 4.0, -4.0, 10.0])
        self.assertEqual(intent.right["mode"], "Hold")

    def test_deadman_false_sends_no_motion_command(self):
        reader = FakeSpaceMouseReader([
            SpaceMouseSample(1, 1, 1, 1, 1, 1, buttons=(False,), timestamp_monotonic=1.0)
        ])
        source = SpaceMouseJointVelocitySource(
            reader=reader,
            max_joint_velocity_deg_s=(5, 5, 5, 8, 8, 10),
            require_deadman=True,
        )
        self.assertIsNone(source.next_intent(snapshot(), time.monotonic()))

    def test_real_mode_blocks_spacemouse_motion_without_allow(self):
        reader = FakeSpaceMouseReader([
            SpaceMouseSample(1, 0, 0, 0, 0, 0, buttons=(True,), timestamp_monotonic=1.0)
        ])
        source = SpaceMouseJointVelocitySource(
            reader=reader,
            max_joint_velocity_deg_s=(5, 5, 5, 8, 8, 10),
        )
        intent = source.next_intent(snapshot(), time.monotonic())
        gate = SafetyGate("real", SafetyConfig(allow_real_motion=False), stale_timeout_sec=0.5)
        decision = gate.evaluate(snapshot(), intent, source.requirements)
        self.assertFalse(decision.allowed)
        self.assertEqual(decision.reason, "real_motion_not_allowed")


if __name__ == "__main__":
    unittest.main()
