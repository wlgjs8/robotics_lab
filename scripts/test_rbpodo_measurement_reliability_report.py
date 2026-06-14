#!/usr/bin/env python3
"""Tests for rbpodo measurement reliability physical-tracking blockers."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import generate_rbpodo_measurement_reliability_report as report


def pgmode_row(physical_tracking_result: dict[str, object]) -> dict[str, object]:
    return {
        "run_name": "physical_fixture",
        "benchmark_category": "rbpodo_controller_simulation",
        "backend": "rbpodo",
        "controller_mode": "pgmode_simulation",
        "profile": "circle_15cm_16s",
        "tracking_source": "tcp_ref_stand",
        "result": "completed",
        "tcp_ref_valid_ratio": 1.0,
        "q_ref_valid_ratio": 1.0,
        "q_ref_update_rate_hz": 100.0,
        "physical_motion_detected": False,
        "fault_latched": False,
        "cartesian_unavailable_count": 0,
        "timing_classification": "clean_timing",
        "physical_tracking_result": physical_tracking_result,
    }


class RbpodoMeasurementReliabilityReportTest(unittest.TestCase):
    def test_tcp_actual_stand_finite_rms_clears_unmeasured_blocker(self) -> None:
        row = pgmode_row(
            {
                "status": "fail",
                "tracking_source": "tcp_actual_stand",
                "rms_error_m": 0.25,
                "p95_error_m": 0.3,
                "max_error_m": 0.4,
            }
        )

        grade = report.grade_row(row)

        self.assertNotIn(
            report.UNMEASURED_PHYSICAL_BLOCKER,
            grade["physical_real_blockers"],
        )
        self.assertNotIn(
            report.UNMEASURED_PHYSICAL_BLOCKER,
            grade["physical_readiness"]["blockers"],
        )
        self.assertEqual(grade["physical_tracking_result"]["status"], "fail")
        self.assertEqual(grade["physical_tracking_result"]["tracking_source"], "tcp_actual_stand")
        self.assertEqual(grade["physical_tracking_result"]["rms_error_m"], 0.25)

    def test_tcp_ref_stand_finite_rms_does_not_clear_unmeasured_blocker(self) -> None:
        row = pgmode_row(
            {
                "status": "pass",
                "tracking_source": "tcp_ref_stand",
                "rms_error_m": 0.001,
            }
        )

        grade = report.grade_row(row)

        self.assertIn(
            report.UNMEASURED_PHYSICAL_BLOCKER,
            grade["physical_real_blockers"],
        )
        self.assertIn(
            report.UNMEASURED_PHYSICAL_BLOCKER,
            grade["physical_readiness"]["blockers"],
        )
        self.assertEqual(grade["physical_tracking_result"]["status"], "not_measured")

    def test_tcp_actual_stand_without_finite_rms_keeps_unmeasured_blocker(self) -> None:
        row = pgmode_row(
            {
                "status": "fail",
                "tracking_source": "tcp_actual_stand",
                "rms_error_m": None,
            }
        )

        grade = report.grade_row(row)

        self.assertIn(
            report.UNMEASURED_PHYSICAL_BLOCKER,
            grade["physical_real_blockers"],
        )
        self.assertEqual(grade["physical_tracking_result"]["status"], "not_measured")

    def test_single_row_json_summary_mirrors_measured_physical_tracking(self) -> None:
        row = pgmode_row(
            {
                "status": "pass",
                "tracking_source": "tcp_actual_stand",
                "rms_error_m": 0.004,
            }
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "summary.json"
            report.write_json(path, [row])
            payload = json.loads(path.read_text(encoding="utf-8"))

        self.assertNotIn(
            report.UNMEASURED_PHYSICAL_BLOCKER,
            payload["physical_readiness"]["blockers"],
        )
        self.assertEqual(payload["physical_tracking_result"]["status"], "pass")


if __name__ == "__main__":
    unittest.main()
