#!/usr/bin/env python3
"""Unit tests for rbpodo 500 Hz comparison reporting."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import generate_rbpodo_500hz_report as report


def circle_row(
    *,
    rate_hz: float,
    profile: str = "safe_5cm_10s",
    rms_error_mm: float = 4.0,
    p95_error_mm: float = 6.0,
    servo_jitter_p99_ms: float = 0.2,
    result: str = "completed",
) -> dict[str, object]:
    return report.normalize_row(
        {
            "run_name": f"{profile}_{int(rate_hz)}hz",
            "profile": profile,
            "rate_hz": rate_hz,
            "controller": "twist_stand_feedback",
            "tracking_source": "tcp_ref_stand",
            "result": result,
            "send_success_rate": 1.0,
            "controller_acceptance_observed_rate": 1.0,
            "send_duration_p99_us": 250.0,
            "send_duration_max_us": 500.0,
            "servo_jitter_p99_ms": servo_jitter_p99_ms,
            "deadline_miss_count": 0,
            "command_interval_max_ms": 10.0 if rate_hz == 100 else 2.0,
            "state_pub_rate_hz": 100.0,
            "q_ref_update_rate_hz": 100.0,
            "rms_error_mm": rms_error_mm,
            "p95_error_mm": p95_error_mm,
            "tail_ratio": 1.5,
            "orientation_p95_deg": 1.0,
            "feedback_saturation_count": 0,
            "measurement_reliability_level": "controller_reference_valid",
            "physical_motion_detected": False,
            "fault_latched": False,
            "cartesian_unavailable_count": 0,
        },
        source_kind="circle",
    )


class Rbpodo500HzReportTest(unittest.TestCase):
    def test_noop_evidence_is_acceptance_stage_not_production_proof(self) -> None:
        rows = report.noop_rows_from_rate_probe(
            {
                "artifact_dir": "/tmp/rbpodo_servo_j_rate_probe_left",
                "backend": "rbpodo",
                "result": "completed",
                "rate_results": [
                    {
                        "requested_rate_hz": 100.0,
                        "feedback_rate_hz": 500.0,
                        "send_count": 1000,
                        "send_success_count": 1000,
                        "send_success_rate": 1.0,
                        "send_failure_count": 0,
                        "send_duration_p99_us": 214.9,
                        "send_duration_max_us": 600.0,
                        "loop_interval_p99_ms": 2.04,
                        "loop_interval_max_ms": 40.0,
                        "q_actual_drift_max_deg": 0.0,
                        "m561_count": 0,
                        "m568_count": 0,
                        "m569_count": 0,
                        "m570_count": 0,
                        "result": "completed",
                    },
                    {
                        "requested_rate_hz": 500.0,
                        "feedback_rate_hz": 500.0,
                        "send_count": 5000,
                        "send_success_count": 5000,
                        "send_success_rate": 1.0,
                        "send_failure_count": 0,
                        "send_duration_p99_us": 334.9,
                        "send_duration_max_us": 501.1,
                        "loop_interval_p99_ms": 2.006,
                        "loop_interval_max_ms": 2.097,
                        "q_actual_drift_max_deg": 0.0,
                        "m561_count": 0,
                        "m568_count": 0,
                        "m569_count": 0,
                        "m570_count": 0,
                        "result": "completed",
                    },
                ],
            }
        )
        selected, comparisons = report.build_report(rows)
        noop = next(row for row in comparisons if row["profile"] == "noop_acceptance")
        selected_500 = next(row for row in selected if row["profile"] == "noop_acceptance" and row["rate_hz"] == 500.0)
        self.assertEqual(noop["classification"], "500hz_noop_pass")
        self.assertEqual(selected_500["measurement_reliability_level"], "acceptance_stage_noop")
        markdown = report.report_markdown(selected, comparisons, "test")
        self.assertIn("no-op success does not prove physical real motion", markdown)
        self.assertIn("Do not change the default rate automatically", markdown)

    def test_lower_jitter_lower_rms_classifies_improved(self) -> None:
        _selected, comparisons = report.build_report(
            [
                circle_row(rate_hz=100.0, rms_error_mm=5.0, p95_error_mm=8.0, servo_jitter_p99_ms=0.5),
                circle_row(rate_hz=500.0, rms_error_mm=3.0, p95_error_mm=6.0, servo_jitter_p99_ms=0.2),
            ]
        )
        safe = next(row for row in comparisons if row["profile"] == "safe_5cm_10s")
        self.assertEqual(safe["classification"], "500hz_circle_improved")

    def test_higher_500hz_error_classifies_no_improvement(self) -> None:
        _selected, comparisons = report.build_report(
            [
                circle_row(rate_hz=100.0, rms_error_mm=3.0, p95_error_mm=5.0, servo_jitter_p99_ms=0.2),
                circle_row(rate_hz=500.0, rms_error_mm=6.0, p95_error_mm=9.0, servo_jitter_p99_ms=0.1),
            ]
        )
        safe = next(row for row in comparisons if row["profile"] == "safe_5cm_10s")
        self.assertEqual(safe["classification"], "500hz_circle_no_improvement")

    def test_faulted_500hz_run_classifies_unstable(self) -> None:
        _selected, comparisons = report.build_report(
            [
                circle_row(rate_hz=100.0, profile="circle_15cm_16s", rms_error_mm=3.0),
                circle_row(rate_hz=500.0, profile="circle_15cm_16s", rms_error_mm=2.0, result="faulted"),
            ]
        )
        stable = next(row for row in comparisons if row["profile"] == "circle_15cm_16s")
        self.assertEqual(stable["classification"], "500hz_unstable")

    def test_cli_help_and_artifact_writes(self) -> None:
        script = Path(__file__).with_name("generate_rbpodo_500hz_report.py")
        completed = subprocess.run(
            [sys.executable, str(script), "--help"],
            check=True,
            stdout=subprocess.PIPE,
            text=True,
        )
        self.assertIn("100 Hz vs 500 Hz", completed.stdout)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            summary = root / "noop_summary.json"
            summary.write_text(
                json.dumps(
                    {
                        "artifact_dir": str(root),
                        "backend": "rbpodo",
                        "result": "completed",
                        "rate_results": [
                            {
                                "requested_rate_hz": 500.0,
                                "feedback_rate_hz": 500.0,
                                "send_count": 5000,
                                "send_success_count": 5000,
                                "send_success_rate": 1.0,
                                "send_failure_count": 0,
                                "send_duration_max_us": 501.0,
                                "loop_interval_p99_ms": 2.006,
                                "q_actual_drift_max_deg": 0.0,
                                "m561_count": 0,
                                "m568_count": 0,
                                "m569_count": 0,
                                "m570_count": 0,
                                "result": "completed",
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )
            md = root / "report.md"
            csv_path = root / "report.csv"
            json_path = root / "report.json"
            subprocess.run(
                [
                    sys.executable,
                    str(script),
                    "--noop-summary",
                    str(summary),
                    "--output-md",
                    str(md),
                    "--csv",
                    str(csv_path),
                    "--json",
                    str(json_path),
                ],
                check=True,
            )
            self.assertIn("500hz_noop_pass", md.read_text(encoding="utf-8"))
            self.assertIn("classification", csv_path.read_text(encoding="utf-8"))
            self.assertEqual(json.loads(json_path.read_text(encoding="utf-8"))["schema"], report.SCHEMA)


if __name__ == "__main__":
    unittest.main()
