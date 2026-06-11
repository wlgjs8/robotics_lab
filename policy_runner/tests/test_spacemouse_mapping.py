from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from policy_runner.action_sources.spacemouse_joint_velocity import SpaceMouseJointVelocityActionSource
from policy_runner.config import config_from_mapping
from policy_runner.robot_state_client import StateSnapshot
from policy_runner.safety import SafetyConfig, SafetyGate
from policy_runner.spacemouse import FakeSpaceMouseReader, SpaceMouseSample


def sample_state(**overrides):
    payload = {
        "schema_version": 1,
        "host_time_ns": 123,
        "motion_state": "ConnectedHold",
        "fault_latched": False,
        "left": {"has_valid_joint_state": True, "q_actual_deg": [0, -30, 80, 0, 60, 0]},
        "right": {"has_valid_joint_state": True, "q_actual_deg": [0, -30, 80, 0, 60, 0]},
    }
    payload.update(overrides)
    return StateSnapshot(payload=payload, received_monotonic=time.monotonic())


def spacemouse_sample(
    tx=0.0,
    ty=0.0,
    tz=0.0,
    rx=0.0,
    ry=0.0,
    rz=0.0,
    buttons=(True,),
):
    return SpaceMouseSample(
        tx=tx,
        ty=ty,
        tz=tz,
        rx=rx,
        ry=ry,
        rz=rz,
        buttons=tuple(buttons),
        timestamp_monotonic=time.monotonic(),
    )


class SpaceMouseMappingTest(unittest.TestCase):
    def test_fake_spacemouse_sample_maps_to_left_joint_velocity_command(self):
        reader = FakeSpaceMouseReader(
            [
                spacemouse_sample(
                    tx=0.5,
                    ty=-1.0,
                    tz=2.0,
                    rx=-2.0,
                    ry=0.1,
                    rz=0.0,
                )
            ]
        )
        source = SpaceMouseJointVelocityActionSource(
            reader=reader,
            selected_arm="left",
            max_joint_velocity_deg_s=(5, 5, 5, 8, 8, 10),
            smoothing_alpha=1.0,
        )

        intent = source.next_intent(sample_state(), time.monotonic())

        self.assertIsNotNone(intent)
        assert intent is not None
        self.assertEqual(intent.left["mode"], "JointVelocity")
        self.assertEqual(intent.left["dq_target_deg_s"], [2.5, -5.0, 5.0, -8.0, 0.8, 0.0])
        self.assertEqual(intent.right["mode"], "Hold")

    def test_selected_arm_both_sends_same_joint_velocity_to_both_arms(self):
        reader = FakeSpaceMouseReader([spacemouse_sample(tx=1.0)])
        source = SpaceMouseJointVelocityActionSource(
            reader=reader,
            selected_arm="both",
            max_joint_velocity_deg_s=(5, 5, 5, 8, 8, 10),
            smoothing_alpha=1.0,
        )

        intent = source.next_intent(sample_state(), time.monotonic())

        self.assertIsNotNone(intent)
        assert intent is not None
        self.assertEqual(intent.left["dq_target_deg_s"], [5.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.assertEqual(intent.right["dq_target_deg_s"], [5.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def test_deadman_false_produces_no_motion_command(self):
        reader = FakeSpaceMouseReader([spacemouse_sample(tx=1.0, buttons=(False,))])
        source = SpaceMouseJointVelocityActionSource(reader=reader, require_deadman=True)

        self.assertIsNone(source.next_intent(sample_state(), time.monotonic()))

    def test_deadband_zeroes_small_axis_values(self):
        reader = FakeSpaceMouseReader([spacemouse_sample(tx=0.07, ty=-0.08, tz=0.09)])
        source = SpaceMouseJointVelocityActionSource(
            reader=reader,
            max_joint_velocity_deg_s=(5, 5, 5, 8, 8, 10),
            deadband=0.08,
            smoothing_alpha=1.0,
        )

        intent = source.next_intent(sample_state(), time.monotonic())

        self.assertIsNotNone(intent)
        assert intent is not None
        self.assertEqual(intent.left["dq_target_deg_s"][:2], [0.0, 0.0])
        self.assertAlmostEqual(intent.left["dq_target_deg_s"][2], 0.45)
        self.assertEqual(intent.left["dq_target_deg_s"][3:], [0.0, 0.0, 0.0])

    def test_smoothing_is_deterministic_after_first_sample(self):
        reader = FakeSpaceMouseReader(
            [
                spacemouse_sample(tx=1.0),
                spacemouse_sample(tx=0.0),
            ]
        )
        source = SpaceMouseJointVelocityActionSource(
            reader=reader,
            max_joint_velocity_deg_s=(10, 10, 10, 10, 10, 10),
            smoothing_alpha=0.5,
        )

        first = source.next_intent(sample_state(), 1.0)
        second = source.next_intent(sample_state(), 2.0)

        self.assertIsNotNone(first)
        self.assertIsNotNone(second)
        assert first is not None
        assert second is not None
        self.assertEqual(first.left["dq_target_deg_s"], [10.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.assertEqual(second.left["dq_target_deg_s"], [5.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def test_real_mode_blocks_spacemouse_motion_without_explicit_allow(self):
        reader = FakeSpaceMouseReader([spacemouse_sample(tx=1.0)])
        source = SpaceMouseJointVelocityActionSource(reader=reader, smoothing_alpha=1.0)
        intent = source.next_intent(sample_state(), time.monotonic())
        gate = SafetyGate("real", SafetyConfig(allow_real_motion=False), stale_timeout_sec=0.5)

        decision = gate.evaluate(sample_state(), intent, source.requirements, time.monotonic())

        # Real/sim gating retired: allowed.
        self.assertTrue(decision.allowed)

    def test_spacemouse_config_fields_parse_without_yaml_dependency(self):
        cfg = config_from_mapping(
            {
                "schema": "robotics_lab.policy_runner.v1",
                "spacemouse": {
                    "selected_arm": "both",
                    "max_joint_velocity_deg_s": [5, 5, 5, 8, 8, 10],
                    "deadband": 0.08,
                    "smoothing_alpha": 0.2,
                    "require_deadman": True,
                    "deadman_button": 0,
                },
            }
        )

        self.assertEqual(cfg.spacemouse.selected_arm, "both")
        self.assertEqual(cfg.spacemouse.max_joint_velocity_deg_s, (5.0, 5.0, 5.0, 8.0, 8.0, 10.0))
        self.assertTrue(cfg.spacemouse.require_deadman)

    def test_dual_axis_remap_flips_linear_and_swaps_angular(self):
        from policy_runner.action_sources.dual_spacemouse_cartesian import (
            DualSpaceMouseCartesianActionSource,
        )

        left = FakeSpaceMouseReader(
            [spacemouse_sample(tx=1.0, ty=1.0, tz=1.0, rx=1.0, ry=0.0, rz=1.0)]
        )
        right = FakeSpaceMouseReader([spacemouse_sample()])
        source = DualSpaceMouseCartesianActionSource(
            left_reader=left,
            right_reader=right,
            max_linear_velocity_m_s=0.1,
            max_angular_velocity_rad_s=0.2,
            deadband=0.0,
            response_curve_gamma=1.0,
            linear_axis_signs=(-1.0, 1.0, -1.0),
            angular_axis_order=("ry", "rx", "rz"),
        )
        intent = source.next_intent(sample_state(), time.monotonic())
        self.assertIsNotNone(intent)
        twist = intent.left["tcp_twist_local"]
        # Linear: x/z mirrored, y kept.
        self.assertAlmostEqual(twist[0], -0.1)
        self.assertAlmostEqual(twist[1], 0.1)
        self.assertAlmostEqual(twist[2], -0.1)
        # Angular: output rx reads cap ry (0.0), output ry reads cap rx (1.0).
        self.assertAlmostEqual(twist[3], 0.0)
        self.assertAlmostEqual(twist[4], 0.2)
        self.assertAlmostEqual(twist[5], 0.2)

    def test_dual_axis_remap_rejects_invalid_configuration(self):
        from policy_runner.action_sources.dual_spacemouse_cartesian import (
            DualSpaceMouseCartesianActionSource,
        )

        with self.assertRaises(ValueError):
            DualSpaceMouseCartesianActionSource(
                left_reader=FakeSpaceMouseReader([]),
                right_reader=FakeSpaceMouseReader([]),
                linear_axis_signs=(-2.0, 1.0, 1.0),
            )
        with self.assertRaises(ValueError):
            DualSpaceMouseCartesianActionSource(
                left_reader=FakeSpaceMouseReader([]),
                right_reader=FakeSpaceMouseReader([]),
                angular_axis_order=("ry", "ry", "rz"),
            )


if __name__ == "__main__":
    unittest.main()
