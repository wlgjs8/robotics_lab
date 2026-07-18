#!/usr/bin/env python3
"""Synthetic stdlib tests for fit_ft_inertial_mass.py."""
from __future__ import annotations

import math
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.dirname(__file__))
import fit_ft_inertial_mass as fitter  # noqa: E402


class FitFtInertialMassTest(unittest.TestCase):
    def test_known_mass_and_lag_are_recovered(self):
        rng = random.Random(20260718)
        count = 900
        dt = 0.002
        mass = 3.4
        lag = 7
        accelerations = [
            tuple(rng.uniform(-2.0, 2.0) for _ in range(3))
            for _ in range(count)
        ]
        positions = [[0.0, 0.0, 0.0] for _ in range(count)]
        for i in range(2, count):
            for axis in range(3):
                positions[i][axis] = (
                    2.0 * positions[i - 1][axis] - positions[i - 2][axis] +
                    accelerations[i][axis] * dt * dt
                )
        forces = [[0.0, 0.0, 0.0] for _ in range(count)]
        for i in range(2, count - lag):
            for axis in range(3):
                forces[i + lag][axis] = (
                    mass * accelerations[i][axis] + rng.uniform(-0.02, 0.02)
                )
        rows = [
            {
                "row_index": i,
                "time_s": i * dt,
                "position": tuple(positions[i]),
                "quaternion": (0.0, 0.0, 0.0, 1.0),
                "force": tuple(forces[i]),
            }
            for i in range(count)
        ]

        result = fitter.fit_inertial_mass(rows, force_threshold_n=20.0)

        self.assertEqual(result["best_lag_ticks"], lag)
        self.assertTrue(math.isclose(
            result["recommended_inertial_effective_mass_kg"],
            mass,
            rel_tol=0.01,
            abs_tol=0.02,
        ))
        self.assertLess(result["residual_rms_n"], 0.03)


if __name__ == "__main__":
    unittest.main()
