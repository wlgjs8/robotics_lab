#!/usr/bin/env python3
"""Unit tests for circle_tracking_benchmark helper semantics."""

from __future__ import annotations

import argparse
import subprocess
import sys
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
    def test_feedback_controller_appears_in_help(self) -> None:
        script = Path(__file__).with_name("circle_tracking_benchmark.py")
        completed = subprocess.run(
            [sys.executable, str(script), "--help"],
            check=True,
            text=True,
            stdout=subprocess.PIPE,
        )
        self.assertIn("twist_stand_feedback", completed.stdout)
        self.assertIn("--feedback-kp-pos", completed.stdout)

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

    def test_feedback_formula_adds_position_error(self) -> None:
        result = bench.compute_feedback_twist_stand(
            feedforward_linear_stand=[0.1, 0.0, 0.0],
            position_error_stand=[0.01, -0.02, 0.0],
            orientation_error_stand=[0.0, 0.0, 0.1],
            kp_pos=2.0,
            kp_ori=3.0,
            max_linear_m_s=1.0,
            max_angular_rad_s=1.0,
        )
        self.assertEqual(result["feedback_twist_stand"][:3], [0.02, -0.04, 0.0])
        self.assertEqual(result["applied_twist_stand"][:3], [0.12000000000000001, -0.04, 0.0])
        self.assertEqual(result["applied_twist_stand"][3:], [0.0, 0.0, 0.30000000000000004])
        self.assertFalse(result["saturated"])

    def test_feedback_clamps_total_command(self) -> None:
        result = bench.compute_feedback_twist_stand(
            feedforward_linear_stand=[1.0, 0.0, 0.0],
            position_error_stand=[1.0, 0.0, 0.0],
            orientation_error_stand=[0.0, 0.0, 1.0],
            kp_pos=1.0,
            kp_ori=1.0,
            max_linear_m_s=0.5,
            max_angular_rad_s=0.2,
        )
        self.assertAlmostEqual(bench.norm(result["applied_twist_stand"][:3]), 0.5)
        self.assertAlmostEqual(bench.norm(result["applied_twist_stand"][3:]), 0.2)
        self.assertTrue(result["saturated"])

    def test_stale_feedback_record_uses_zero_twist(self) -> None:
        row = bench.zero_feedback_record(0.1, "stale feedback state", "stand")
        self.assertEqual(row["applied_twist"], [0.0] * 6)
        self.assertTrue(row["stale_or_invalid_state"])
        metrics = bench.feedback_metrics([row])
        self.assertEqual(metrics["stale_state_feedback_skips"], 1)

    def test_preflight_rejects_real_config_text(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            server_config = root / "server.yaml"
            left_config = root / "left.yaml"
            right_config = root / "right.yaml"
            server_config.write_text(
                """
left_robot:
  backend_type: simulator
  run_mode: real
cartesian_control:
  allow_in_simulation: true
  allow_in_real: false
  max_twist_linear_m_s: 0.03
  max_twist_angular_rad_s: 0.2
  max_linear_move_speed_m_s: 0.05
""",
                encoding="utf-8",
            )
            left_config.write_text("simulator:\n  motion_time_constant_sec: 0.04\n", encoding="utf-8")
            right_config.write_text("simulator:\n  motion_time_constant_sec: 0.04\n", encoding="utf-8")
            args = argparse.Namespace(
                root=root,
                server_config=server_config,
                left_config=left_config,
                right_config=right_config,
                arm="left",
                controller="twist_stand_feedback",
                profile="safe_5cm_10s",
                diameter_m=None,
                period_sec=None,
                repeat=1,
                command_rate_hz=100.0,
                warmup_sec=0.0,
                settle_sec=0.0,
                allow_fast_stress=False,
                feedback_kp_pos=2.0,
                feedback_kp_ori=2.0,
                feedback_max_linear_m_s=None,
                feedback_max_angular_rad_s=None,
            )
            with self.assertRaisesRegex(bench.AcceptanceError, "run_mode: real"):
                bench.preflight(args)


if __name__ == "__main__":
    unittest.main()
