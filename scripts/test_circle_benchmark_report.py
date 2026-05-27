#!/usr/bin/env python3
"""Unit tests for circle benchmark report helpers."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import compare_circle_benchmarks as compare
import generate_circle_benchmark_report as report


class CircleBenchmarkReportTest(unittest.TestCase):
    def test_compare_handles_missing_optional_fields(self) -> None:
        row = compare.comparison_row(
            {
                "artifact_dir": "/tmp/circle_run",
                "controller": "twist_stand",
                "diameter_m": 0.15,
                "period_sec": 16.0,
                "fit_radius_m": 0.075,
                "radius_m": 0.075,
            }
        )
        self.assertEqual(row["profile"], "circle_15cm_16s")
        self.assertAlmostEqual(row["radius_gain"], 1.0)
        self.assertIsNone(row["integrator_clamps_total"])

    def test_good_15cm16s_classifies_as_baseline_candidate(self) -> None:
        rows = report.classify_rows(
            [
                {
                    "run_name": "baseline",
                    "controller": "twist_stand",
                    "arm": "left",
                    "profile": "circle_15cm_16s",
                    "diameter_m": 0.15,
                    "period_sec": 16.0,
                    "repeat": 3,
                    "radius_gain": 0.99,
                    "rms_error_mm": 2.0,
                    "p95_error_mm": 3.0,
                    "max_orientation_drift_mrad": 1.0,
                    "worker_command_drops_total": 0,
                    "send_command_deadline_missed_count": 0,
                    "fault_latched": False,
                }
            ],
            min_repeats=3,
        )
        self.assertEqual(rows[0]["classification"], "stable_simulator_baseline_candidate")
        self.assertEqual(rows[0]["real_candidate_policy"], "simulator_seed_only_after_real_acceptance")

    def test_gene_15cm4s_classifies_as_stress_only(self) -> None:
        rows = report.classify_rows(
            [
                {
                    "run_name": "stress",
                    "controller": "twist_stand",
                    "arm": "left",
                    "profile": "gene_15cm_4s",
                    "diameter_m": 0.15,
                    "period_sec": 4.0,
                    "repeat": 5,
                    "radius_gain": 0.95,
                    "rms_error_mm": 30.0,
                    "p95_error_mm": 45.0,
                    "worker_command_drops_total": 0,
                    "integrator_divergence_total": 0,
                    "fault_latched": False,
                }
            ],
            min_repeats=3,
        )
        self.assertEqual(rows[0]["classification"], "stress_benchmark_candidate")
        self.assertEqual(rows[0]["real_candidate_policy"], "stress_only_not_real_ready")

    def test_report_cli_writes_markdown_and_csv(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            summary = root / "summary.json"
            summary.write_text(
                json.dumps(
                    {
                        "artifact_dir": str(root),
                        "controller": "twist_stand",
                        "arm": "left",
                        "profile": "circle_15cm_16s",
                        "diameter_m": 0.15,
                        "period_sec": 16.0,
                        "repeat": 3,
                        "radius_gain": 0.99,
                        "rms_error_m": 0.002,
                        "p95_error_m": 0.003,
                        "max_orientation_drift_rad": 0.001,
                        "worker_command_drops_total": 0,
                        "send_command_deadline_missed_count": 0,
                        "fault_latched": False,
                        "result": "completed",
                    }
                ),
                encoding="utf-8",
            )
            md = root / "report.md"
            csv_path = root / "report.csv"
            script = Path(__file__).with_name("generate_circle_benchmark_report.py")
            subprocess.run(
                [sys.executable, str(script), str(summary), "--output-md", str(md), "--csv", str(csv_path)],
                check=True,
            )
            self.assertIn("stable_simulator_baseline_candidate", md.read_text(encoding="utf-8"))
            self.assertIn("stress evidence is not real-ready", md.read_text(encoding="utf-8").lower())
            self.assertIn("classification", csv_path.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
