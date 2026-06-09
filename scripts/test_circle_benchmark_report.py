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
        self.assertEqual(row["benchmark_category"], "unknown")

    def test_legacy_threshold_fail_is_not_execution_failure(self) -> None:
        row = compare.comparison_row(
            {
                "artifact_dir": "/tmp/ackon500_candidate",
                "schema": "robotics_lab.rbpodo_circle_tracking_benchmark.v1",
                "controller": "twist_stand_feedback",
                "profile": "gene_15cm_4s",
                "tracking_source_used": "tcp_ref_stand",
                "result": "fail",
                "result_reason": "thresholds applied and failed",
                "threshold_failures": ["max_orientation_drift_rad 0.032 exceeds threshold 0.02"],
                "max_orientation_drift_rad": 0.032,
                "p95_orientation_drift_rad": 0.01,
                "safety_preflight": {
                    "backend": "rbpodo",
                    "controller_simulation_only": True,
                    "physical_motion_expected": False,
                    "pgmode_simulation_confirmed": True,
                },
                "physical_motion_detected": False,
                "fault_latched": False,
                "cartesian_unavailable_count": 0,
            }
        )
        self.assertEqual(row["run_result_status"], "completed")
        self.assertEqual(row["benchmark_threshold_status"], "fail")
        self.assertIn("max_orientation_drift_rad", row["threshold_failures"])

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
        self.assertEqual(rows[0]["benchmark_category"], "unknown")

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

    def test_report_distinguishes_simulator_and_rbpodo_controller_simulation(self) -> None:
        simulator = compare.comparison_row(
            {
                "artifact_dir": "/tmp/sim_circle",
                "left_simulator_log": "/tmp/left.log",
                "controller": "twist_stand",
                "arm": "left",
                "profile": "circle_15cm_16s",
                "diameter_m": 0.15,
                "period_sec": 16.0,
            }
        )
        rbpodo = compare.comparison_row(
            {
                "schema": "robotics_lab.rbpodo_circle_tracking_benchmark.v1",
                "artifact_dir": "/tmp/rbpodo_circle",
                "controller": "twist_stand_feedback",
                "arm": "left",
                "profile": "circle_15cm_16s",
                "tracking_source_used": "tcp_ref_stand",
                "safety_preflight": {
                    "backend": "rbpodo",
                    "controller_simulation_only": True,
                    "physical_motion_expected": False,
                    "pgmode_simulation_confirmed": True,
                    "disable_waiting_ack": False,
                },
                "physical_motion_detected": False,
                "fault_latched": False,
            }
        )
        rows = report.classify_rows([simulator, rbpodo])
        self.assertEqual(rows[0]["benchmark_category"], "rb_simulator")
        self.assertEqual(rows[0]["benchmark_lane"], "simulator_python_streaming_open_loop")
        self.assertEqual(rows[1]["benchmark_category"], "rbpodo_controller_simulation")
        self.assertEqual(rows[1]["benchmark_lane"], "rbpodo_python_streaming_feedback")
        self.assertEqual(rows[1]["controller_mode"], "pgmode_simulation")
        self.assertEqual(rows[1]["backend"], "rbpodo")
        markdown = report.report_markdown(rows, "test", 1)
        self.assertIn("## Canonical Benchmark Lanes", markdown)
        self.assertIn("benchmark_lane", markdown)

    def test_rbpodo_stable_baseline_uses_tcp_ref_tracking_source(self) -> None:
        rows = report.classify_rows(
            [
                {
                    "run_name": "rbpodo_stable",
                    "benchmark_category": "rbpodo_controller_simulation",
                    "backend": "rbpodo",
                    "controller_mode": "pgmode_simulation",
                    "controller": "twist_stand_feedback",
                    "arm": "left",
                    "profile": "circle_15cm_16s",
                    "tracking_source": "tcp_ref_stand",
                    "radius_gain": 1.0,
                    "rms_error_mm": 4.0,
                    "p95_error_mm": 6.0,
                    "physical_motion_expected": False,
                    "physical_motion_detected": False,
                    "fault_latched": False,
                    "ack_policy": "ack_on",
                    "controller_acceptance_observed_count": 100,
                    "command_count": 100,
                    "feedback_saturation_count": 0,
                    "command_timeout_count": 0,
                    "controller_rejected_count": 0,
                }
            ],
            min_repeats=1,
        )
        self.assertEqual(rows[0]["classification"], "closed_loop_candidate")
        self.assertEqual(rows[0]["real_candidate_policy"], "future_low_speed_seed_only_not_real_ready")
        self.assertEqual(rows[0]["saturation_ratio"], 0.0)

    def test_rbpodo_stress_is_not_real_ready(self) -> None:
        rows = report.classify_rows(
            [
                {
                    "run_name": "rbpodo_gene",
                    "benchmark_category": "rbpodo_controller_simulation",
                    "backend": "rbpodo",
                    "controller_mode": "pgmode_simulation",
                    "controller": "twist_stand_feedback",
                    "arm": "left",
                    "profile": "gene_15cm_4s",
                    "tracking_source": "tcp_ref_stand",
                    "radius_gain": 0.97,
                    "rms_error_mm": 8.0,
                    "p95_error_mm": 14.0,
                    "feedback_saturation_count": 3,
                    "command_count": 100,
                    "physical_motion_expected": False,
                    "physical_motion_detected": False,
                    "fault_latched": False,
                }
            ],
            min_repeats=1,
        )
        self.assertEqual(rows[0]["classification"], "closed_loop_candidate")
        self.assertEqual(rows[0]["real_candidate_policy"], "controller_sim_stress_not_real_ready")
        self.assertIn("not physical real evidence", rows[0]["promotion_notes"].lower())

    def test_rbpodo_open_loop_center_drift_is_baseline_only(self) -> None:
        rows = report.classify_rows(
            [
                {
                    "run_name": "open_loop_radius_good_center_bad",
                    "benchmark_category": "rbpodo_controller_simulation",
                    "backend": "rbpodo",
                    "controller_mode": "pgmode_simulation",
                    "controller": "twist_stand",
                    "profile": "gene_15cm_4s",
                    "tracking_source": "tcp_ref_stand",
                    "radius_gain": 1.01,
                    "rms_error_mm": 126.0,
                    "p95_error_mm": 143.0,
                    "center_error_mm": 121.0,
                    "orientation_p95_deg": 2.5,
                    "physical_motion_detected": False,
                    "fault_latched": False,
                }
            ],
            min_repeats=1,
        )
        self.assertEqual(rows[0]["classification"], "open_loop_baseline")
        self.assertIn("open-loop radius can be good while center drift is bad", rows[0]["promotion_notes"])

    def test_rbpodo_saturation_limited_case(self) -> None:
        rows = report.classify_rows(
            [
                {
                    "run_name": "sat_limited",
                    "benchmark_category": "rbpodo_controller_simulation",
                    "backend": "rbpodo",
                    "controller_mode": "pgmode_simulation",
                    "controller": "twist_stand_feedback",
                    "profile": "gene_15cm_4s",
                    "tracking_source": "tcp_ref_stand",
                    "kp_pos": 1.2,
                    "kp_ori": 1.0,
                    "radius_gain": 1.05,
                    "rms_error_mm": 40.0,
                    "p95_error_mm": 60.0,
                    "feedback_saturation_count": 30,
                    "command_count": 100,
                    "center_error_mm": 10.0,
                    "orientation_p95_deg": 6.0,
                    "state_pub_rate_hz": 50,
                    "speed_bar_left": 0.1,
                    "speed_bar_right": 0.1,
                    "physical_motion_detected": False,
                    "fault_latched": False,
                }
            ],
            min_repeats=1,
        )
        self.assertEqual(rows[0]["classification"], "saturation_limited")
        self.assertGreater(rows[0]["score"], 50000.0)

    def test_rbpodo_good_low_saturation_candidate_scores(self) -> None:
        rows = report.classify_rows(
            [
                {
                    "run_name": "good_low_sat",
                    "benchmark_category": "rbpodo_controller_simulation",
                    "backend": "rbpodo",
                    "controller_mode": "pgmode_simulation",
                    "controller": "twist_stand_feedback",
                    "profile": "gene_15cm_4s",
                    "tracking_source": "tcp_ref_stand",
                    "kp_pos": 0.5,
                    "kp_ori": 0.2,
                    "radius_gain": 0.99,
                    "rms_error_mm": 20.0,
                    "p95_error_mm": 45.0,
                    "feedback_saturation_count": 0,
                    "command_count": 100,
                    "center_error_mm": 4.0,
                    "orientation_p95_deg": 8.0,
                    "physical_motion_detected": False,
                    "fault_latched": False,
                }
            ],
            min_repeats=1,
        )
        self.assertEqual(rows[0]["classification"], "closed_loop_candidate")
        self.assertEqual(rows[0]["saturation_ratio"], 0.0)
        self.assertLess(rows[0]["score"], 1000.0)

    def test_rbpodo_report_handles_missing_tuning_fields(self) -> None:
        rows = report.classify_rows(
            [
                {
                    "run_name": "missing_fields",
                    "benchmark_category": "rbpodo_controller_simulation",
                    "backend": "rbpodo",
                    "controller_mode": "pgmode_simulation",
                    "controller": "twist_stand_feedback",
                    "profile": "gene_15cm_4s",
                    "tracking_source": "tcp_ref_stand",
                    "physical_motion_detected": False,
                    "fault_latched": False,
                }
            ],
            min_repeats=1,
        )
        self.assertEqual(rows[0]["classification"], "stress_only")
        self.assertIsNone(rows[0]["score"])
        self.assertIn("missing candidate metrics", rows[0]["promotion_notes"])

    def test_completed_tcp_ref_with_diagnostics_is_suspect_not_physical_ready(self) -> None:
        rows = report.classify_rows(
            [
                {
                    "run_name": "diag_suspect_lower_bound",
                    "benchmark_category": "rbpodo_controller_simulation",
                    "backend": "rbpodo",
                    "controller_mode": "pgmode_simulation",
                    "controller": "twist_stand_feedback",
                    "profile": "circle_15cm_16s",
                    "tracking_source": "tcp_ref_stand",
                    "result": "completed",
                    "tcp_ref_valid_ratio": 1.0,
                    "q_ref_valid_ratio": 1.0,
                    "q_ref_update_rate_hz": 50.0,
                    "physical_motion_detected": False,
                    "fault_latched": False,
                    "diagnostics_suspect_count": 10,
                }
            ],
            min_repeats=1,
        )
        self.assertEqual(rows[0]["measurement_reliability_level"], "suspect")
        self.assertIn("diagnostics_suspect_unresolved", rows[0]["reliability_reasons"])
        self.assertIn("diagnostics_suspect_override_active", rows[0]["reliability_caveats"])
        self.assertIn("tcp_ref_lower_bound_only", rows[0]["reliability_caveats"])
        self.assertFalse(rows[0]["physical_ready_candidate"])
        self.assertEqual(rows[0]["physical_readiness"]["status"], "blocked")
        self.assertEqual(rows[0]["physical_tracking_result"]["status"], "not_measured")
        self.assertNotEqual(rows[0]["physical_tracking_result"]["status"], "pass")

    def test_clean_controller_reference_run_is_controller_reference_valid(self) -> None:
        rows = report.classify_rows(
            [
                {
                    "run_name": "clean_controller_reference",
                    "benchmark_category": "rbpodo_controller_simulation",
                    "backend": "rbpodo",
                    "controller_mode": "pgmode_simulation",
                    "controller": "twist_stand_feedback",
                    "profile": "circle_15cm_16s",
                    "tracking_source": "tcp_ref_stand",
                    "result": "completed",
                    "tcp_ref_valid_ratio": 1.0,
                    "q_ref_valid_ratio": 1.0,
                    "q_ref_update_rate_hz": 100.0,
                    "physical_motion_detected": False,
                    "fault_latched": False,
                    "diagnostics_suspect_count": 0,
                    "cartesian_unavailable_count": 0,
                    "timing_classification": "clean_timing",
                }
            ],
            min_repeats=1,
        )
        self.assertEqual(rows[0]["measurement_reliability_level"], "controller_reference_valid")
        self.assertIn("controller_reference_lower_bound", rows[0]["benchmark_interpretation"])
        self.assertIn("physical_reference_to_actual_error_unmeasured", rows[0]["physical_real_blockers"])
        self.assertFalse(rows[0]["physical_ready_candidate"])
        self.assertEqual(rows[0]["controller_reference_result"]["status"], "pass")
        self.assertEqual(rows[0]["controller_reference_result"]["explanation"], "tcp_ref_stand lower-bound evidence")
        self.assertEqual(rows[0]["physical_tracking_result"]["status"], "not_measured")

    def test_measured_tcp_actual_artifact_clears_only_unmeasured_blocker(self) -> None:
        rows = report.classify_rows(
            [
                {
                    "run_name": "measured_physical_tracking",
                    "benchmark_category": "rbpodo_controller_simulation",
                    "backend": "rbpodo",
                    "controller_mode": "pgmode_simulation",
                    "controller": "twist_stand_feedback",
                    "profile": "circle_15cm_16s",
                    "tracking_source": "tcp_ref_stand",
                    "result": "completed",
                    "tcp_ref_valid_ratio": 1.0,
                    "q_ref_valid_ratio": 1.0,
                    "q_ref_update_rate_hz": 100.0,
                    "physical_motion_detected": False,
                    "fault_latched": False,
                    "diagnostics_suspect_count": 7,
                    "cartesian_unavailable_count": 0,
                    "timing_classification": "clean_timing",
                    "physical_tracking_result": {
                        "status": "fail",
                        "tracking_source": "tcp_actual_stand",
                        "rms_error_m": 0.08,
                        "p95_error_m": 0.09,
                        "max_error_m": 0.1,
                    },
                }
            ],
            min_repeats=1,
        )
        self.assertNotIn("physical_reference_to_actual_error_unmeasured", rows[0]["physical_real_blockers"])
        self.assertNotIn(
            "physical_reference_to_actual_error_unmeasured",
            rows[0]["physical_readiness"]["blockers"],
        )
        self.assertIn("diagnostics_suspect_unresolved", rows[0]["physical_real_blockers"])
        self.assertEqual(rows[0]["physical_tracking_result"]["status"], "fail")
        self.assertEqual(rows[0]["physical_tracking_status"], "fail")

    def test_tcp_ref_physical_tracking_result_does_not_clear_unmeasured_blocker(self) -> None:
        rows = report.classify_rows(
            [
                {
                    "run_name": "ref_tracking_only",
                    "benchmark_category": "rbpodo_controller_simulation",
                    "backend": "rbpodo",
                    "controller_mode": "pgmode_simulation",
                    "controller": "twist_stand_feedback",
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
                    "physical_tracking_result": {
                        "status": "pass",
                        "tracking_source": "tcp_ref_stand",
                        "rms_error_m": 0.001,
                    },
                }
            ],
            min_repeats=1,
        )
        self.assertIn("physical_reference_to_actual_error_unmeasured", rows[0]["physical_real_blockers"])
        self.assertEqual(rows[0]["physical_tracking_result"]["status"], "not_measured")

    def test_faulted_run_is_unreliable(self) -> None:
        rows = report.classify_rows(
            [
                {
                    "run_name": "faulted",
                    "benchmark_category": "rbpodo_controller_simulation",
                    "backend": "rbpodo",
                    "controller_mode": "pgmode_simulation",
                    "profile": "circle_15cm_16s",
                    "tracking_source": "tcp_ref_stand",
                    "result": "faulted",
                    "fault_latched": True,
                    "tcp_ref_valid_ratio": 1.0,
                    "q_ref_valid_ratio": 1.0,
                    "physical_motion_detected": False,
                }
            ],
            min_repeats=1,
        )
        self.assertEqual(rows[0]["measurement_reliability_level"], "unreliable")

    def test_physical_motion_detected_is_unreliable(self) -> None:
        rows = report.classify_rows(
            [
                {
                    "run_name": "physical_motion",
                    "benchmark_category": "rbpodo_controller_simulation",
                    "backend": "rbpodo",
                    "controller_mode": "pgmode_simulation",
                    "profile": "circle_15cm_16s",
                    "tracking_source": "tcp_ref_stand",
                    "result": "completed",
                    "tcp_ref_valid_ratio": 1.0,
                    "q_ref_valid_ratio": 1.0,
                    "physical_motion_detected": True,
                    "fault_latched": False,
                }
            ],
            min_repeats=1,
        )
        self.assertEqual(rows[0]["measurement_reliability_level"], "unreliable")
        self.assertIn("physical_motion_detected", rows[0]["reliability_reasons"])

    def test_missing_q_ref_is_suspect(self) -> None:
        rows = report.classify_rows(
            [
                {
                    "run_name": "missing_q_ref",
                    "benchmark_category": "rbpodo_controller_simulation",
                    "backend": "rbpodo",
                    "controller_mode": "pgmode_simulation",
                    "profile": "circle_15cm_16s",
                    "tracking_source": "tcp_ref_stand",
                    "result": "completed",
                    "tcp_ref_valid_ratio": 1.0,
                    "q_ref_reason": "q_ref_deg not published",
                    "physical_motion_detected": False,
                    "fault_latched": False,
                }
            ],
            min_repeats=1,
        )
        self.assertEqual(rows[0]["measurement_reliability_level"], "suspect")
        self.assertIn("q_ref_not_directly_validated", rows[0]["reliability_caveats"])

    def test_state_parity_failed_caps_reliability_at_suspect(self) -> None:
        rows = report.classify_rows(
            [
                {
                    "run_name": "parity_failed",
                    "benchmark_category": "rbpodo_controller_simulation",
                    "backend": "rbpodo",
                    "controller_mode": "pgmode_simulation",
                    "profile": "circle_15cm_16s",
                    "tracking_source": "tcp_ref_stand",
                    "result": "completed",
                    "tcp_ref_valid_ratio": 1.0,
                    "q_ref_valid_ratio": 1.0,
                    "q_ref_update_rate_hz": 100.0,
                    "state_parity_result": "failed_parity_mismatch",
                    "physical_motion_detected": False,
                    "fault_latched": False,
                }
            ],
            min_repeats=1,
        )
        self.assertEqual(rows[0]["measurement_reliability_level"], "suspect")
        self.assertIn("state_parity_failed", rows[0]["reliability_reasons"])
        self.assertIn("state_parity_failed", rows[0]["physical_real_blockers"])

    def test_stress_profile_marks_il_data_not_recommended(self) -> None:
        rows = report.classify_rows(
            [
                {
                    "run_name": "stress",
                    "benchmark_category": "rbpodo_controller_simulation",
                    "backend": "rbpodo",
                    "controller_mode": "pgmode_simulation",
                    "controller": "twist_stand_feedback",
                    "profile": "gene_15cm_4s",
                    "tracking_source": "tcp_ref_stand",
                    "result": "completed",
                    "tcp_ref_valid_ratio": 1.0,
                    "q_ref_valid_ratio": 1.0,
                    "q_ref_update_rate_hz": 50.0,
                    "physical_motion_detected": False,
                    "fault_latched": False,
                }
            ],
            min_repeats=1,
        )
        self.assertIn("stress_profile", rows[0]["benchmark_interpretation"])
        self.assertIn("IL_data_not_recommended", rows[0]["benchmark_interpretation"])

    def test_rbpodo_compare_handles_missing_physical_actual_path(self) -> None:
        row = compare.comparison_row(
            {
                "schema": "robotics_lab.rbpodo_circle_tracking_benchmark.v1",
                "artifact_dir": "/tmp/rbpodo_circle",
                "controller": "twist_stand_feedback",
                "arm": "left",
                "profile": "circle_15cm_16s",
                "tracking_source_used": "tcp_ref_stand",
                "physical_actual_csv": None,
                "physical_motion_detected": False,
                "q_ref_update_rate_hz": 100.0,
                "q_actual_update_rate_hz": 0.0,
                "safety_preflight": {
                    "backend": "rbpodo",
                    "controller_simulation_only": True,
                    "physical_motion_expected": False,
                    "pgmode_simulation_confirmed": True,
                    "disable_waiting_ack": False,
                },
            }
        )
        self.assertEqual(row["benchmark_category"], "rbpodo_controller_simulation")
        self.assertEqual(row["tracking_source"], "tcp_ref_stand")
        self.assertIsNone(row["physical_actual_csv"])
        self.assertFalse(row["physical_motion_detected"])

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
            self.assertIn("closed-loop is structurally needed for rbpodo controller simulation", md.read_text(encoding="utf-8"))
            self.assertIn("classification", csv_path.read_text(encoding="utf-8"))

    def test_rbpodo_report_cli_help_works(self) -> None:
        script = Path(__file__).with_name("generate_rbpodo_circle_report.py")
        completed = subprocess.run(
            [sys.executable, str(script), "--help"],
            check=True,
            stdout=subprocess.PIPE,
            text=True,
        )
        self.assertIn("pgmode-simulation", completed.stdout)


if __name__ == "__main__":
    unittest.main()
