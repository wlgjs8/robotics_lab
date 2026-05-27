#!/usr/bin/env python3
"""Unit tests for circle_tracking_benchmark helper semantics."""

from __future__ import annotations

import argparse
import tempfile
import unittest
from pathlib import Path

import circle_tracking_benchmark as bench


def args_with_thresholds(**overrides: float | None) -> argparse.Namespace:
    values: dict[str, float | None] = {
        "max_allowed_rms_error_m": None,
        "max_allowed_p95_error_m": None,
        "max_allowed_orientation_drift_rad": None,
        "max_allowed_latency_ms": None,
    }
    values.update(overrides)
    return argparse.Namespace(**values)


class CircleTrackingBenchmarkHelpersTest(unittest.TestCase):
    def test_result_semantics_without_thresholds_completed(self) -> None:
        args = args_with_thresholds()
        result, reason = bench.benchmark_result(
            bench.thresholds_requested(args),
            bench.threshold_failures(args, {"rms_error_m": 1.0}),
        )
        self.assertEqual(result, "completed")
        self.assertIn("without thresholds", reason)

    def test_result_semantics_thresholds_pass(self) -> None:
        args = args_with_thresholds(max_allowed_rms_error_m=0.01)
        failures = bench.threshold_failures(args, {"rms_error_m": 0.001})
        result, reason = bench.benchmark_result(bench.thresholds_requested(args), failures)
        self.assertEqual(result, "pass")
        self.assertEqual(reason, "thresholds applied and satisfied")

    def test_result_semantics_thresholds_fail(self) -> None:
        args = args_with_thresholds(max_allowed_rms_error_m=0.01)
        failures = bench.threshold_failures(args, {"rms_error_m": 0.1})
        result, reason = bench.benchmark_result(bench.thresholds_requested(args), failures)
        self.assertEqual(result, "fail")
        self.assertEqual(reason, "thresholds applied and failed")
        self.assertTrue(failures)

    def test_radius_gain_warning_for_attenuated_circle(self) -> None:
        reference_radius = 0.075
        fit_radius = 0.016
        radius_gain = fit_radius / reference_radius
        self.assertAlmostEqual(radius_gain, 0.21333333333333335)
        warnings = bench.performance_warnings(
            {
                "reference_radius_m": reference_radius,
                "radius_gain": radius_gain,
                "rms_error_m": 0.02,
                "p95_error_m": 0.04,
                "max_orientation_drift_rad": 0.001,
            }
        )
        self.assertTrue(any("radius_gain" in warning for warning in warnings))

    def test_simulator_dynamics_context_parsing(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            server_config = root / "server.yaml"
            left_config = root / "left.yaml"
            right_config = root / "right.yaml"
            server_config.write_text(
                """
servo:
  rate_hz: 100
cartesian_control:
  max_twist_linear_m_s: 0.15
  max_linear_move_speed_m_s: 0.15
""",
                encoding="utf-8",
            )
            left_config.write_text(
                """
simulator:
  motion_time_constant_sec: 0.04
""",
                encoding="utf-8",
            )
            right_config.write_text(
                """
simulator:
  motion_time_constant_sec: 0.05
""",
                encoding="utf-8",
            )
            args = argparse.Namespace(
                server_config=server_config,
                left_config=left_config,
                right_config=right_config,
                arm="left",
                profile="gene_15cm_4s",
            )
            context = bench.benchmark_context(
                args,
                {"max_twist_linear_m_s": 0.15, "max_linear_move_speed_m_s": 0.15},
            )
        self.assertEqual(context["servo_rate_hz"], 100.0)
        self.assertEqual(context["servo_dt_sec"], 0.01)
        self.assertEqual(context["simulator_motion_time_constant_sec"], 0.04)
        self.assertAlmostEqual(context["simulator_dt_over_tau"], 0.25)
        self.assertTrue(context["stress_profile"])


if __name__ == "__main__":
    unittest.main()
