from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from policy_runner.action_sources.dual_spacemouse_cartesian import (
    DualSpaceMouseCartesianActionSource,
    _apply_soft_deadband as apply_dual_soft_deadband,
)
from policy_runner.action_sources.spacemouse_cartesian import (
    SpaceMouseCartesianActionSource,
    _apply_soft_deadband,
)
from policy_runner.config import config_from_mapping
from policy_runner.robot_state_client import StateSnapshot
from policy_runner.spacemouse import FakeSpaceMouseReader, SpaceMouseSample


def sample_state() -> StateSnapshot:
    return StateSnapshot(payload={}, received_monotonic=time.monotonic())


def spacemouse_sample(
    tx: float = 0.0,
    ty: float = 0.0,
    tz: float = 0.0,
    rx: float = 0.0,
    ry: float = 0.0,
    rz: float = 0.0,
    buttons: tuple[bool, ...] = (True,),
) -> SpaceMouseSample:
    return SpaceMouseSample(
        tx=tx,
        ty=ty,
        tz=tz,
        rx=rx,
        ry=ry,
        rz=rz,
        buttons=buttons,
        timestamp_monotonic=time.monotonic(),
    )


class SoftDeadbandTest(unittest.TestCase):
    def test_inside_deadband_returns_zero(self):
        self.assertEqual(_apply_soft_deadband(0.05, 0.10, 3.0), 0.0)
        self.assertEqual(_apply_soft_deadband(-0.10, 0.10, 3.0), 0.0)
        self.assertEqual(apply_dual_soft_deadband(0.05, 0.10, 3.0), 0.0)
        self.assertEqual(apply_dual_soft_deadband(-0.10, 0.10, 3.0), 0.0)

    def test_at_full_deflection_returns_one_minus_deadband(self):
        self.assertAlmostEqual(_apply_soft_deadband(1.0, 0.10, 3.0), 0.9)
        self.assertAlmostEqual(_apply_soft_deadband(-1.0, 0.10, 3.0), -0.9)

    def test_small_deflection_is_cubically_attenuated(self):
        magnitude = (0.20 - 0.10) / (1.0 - 0.10)
        expected = magnitude**3.0 * (1.0 - 0.10)

        self.assertAlmostEqual(_apply_soft_deadband(0.20, 0.10, 3.0), expected, places=5)

    def test_gamma_one_is_linear(self):
        self.assertAlmostEqual(_apply_soft_deadband(0.5, 0.10, 1.0), 0.4, places=5)

    def test_gamma_below_one_rejected(self):
        with self.assertRaisesRegex(ValueError, "response_curve_gamma"):
            SpaceMouseCartesianActionSource(
                reader=FakeSpaceMouseReader(),
                response_curve_gamma=0.5,
            )
        with self.assertRaisesRegex(ValueError, "response_curve_gamma"):
            DualSpaceMouseCartesianActionSource(
                left_reader=FakeSpaceMouseReader(),
                right_reader=FakeSpaceMouseReader(),
                response_curve_gamma=0.5,
            )

    def test_config_default_gamma_is_three(self):
        cfg = config_from_mapping({"schema": "robotics_lab.policy_runner.v1"})

        self.assertEqual(cfg.spacemouse_cartesian.response_curve_gamma, 3.0)
        self.assertEqual(cfg.spacemouse_cartesian_dual.response_curve_gamma, 3.0)

    def test_spring_back_tail_is_attenuated_below_threshold(self):
        source = SpaceMouseCartesianActionSource(
            reader=FakeSpaceMouseReader(
                [
                    spacemouse_sample(tx=0.5),
                    spacemouse_sample(tx=0.15),
                ]
            ),
            deadband=0.10,
            response_curve_gamma=3.0,
            max_linear_velocity_m_s=0.03,
        )

        large_intent = source.next_intent(sample_state(), time.monotonic())
        tail_intent = source.next_intent(sample_state(), time.monotonic())

        self.assertIsNotNone(large_intent)
        self.assertIsNotNone(tail_intent)
        assert tail_intent is not None
        self.assertLess(
            abs(tail_intent.left["tcp_twist_local"][0]),
            0.001 * source.max_linear_velocity_m_s,
        )


if __name__ == "__main__":
    unittest.main()
