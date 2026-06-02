#!/usr/bin/env python3
"""Unit tests for ACKON500-GENE-GOAL-01 reporting."""

from __future__ import annotations

import csv
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import generate_ackon500_gene_goal_report as report


def write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_jsonl(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, sort_keys=True) + "\n")


def write_ablation_csv(root: Path, artifact_dir: Path, *, acceptance_semantics: str = "controller_ack_observed") -> None:
    with (root / "ablation_summary.csv").open("w", newline="", encoding="utf-8") as handle:
        fieldnames = [
            "name",
            "artifact_dir",
            "profile",
            "tracking_source",
            "servo_rate_hz",
            "servo_t1_sec",
            "async_mode",
            "acceptance_semantics",
        ]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerow(
            {
                "name": "ackon500_gene_sdk_pass",
                "artifact_dir": str(artifact_dir.resolve()),
                "profile": "gene_15cm_4s",
                "tracking_source": "tcp_ref_stand",
                "servo_rate_hz": "500",
                "servo_t1_sec": "0.002",
                "async_mode": "sdk_ack_worker",
                "acceptance_semantics": acceptance_semantics,
            }
        )


def write_candidate(
    root: Path,
    *,
    socket_send_only: bool = False,
    sent: int = 10000,
    acked: int | None = None,
    goal_sent: int | None = 10000,
    goal_acked: int | None = None,
    include_tick_window: bool = True,
    rms_error_m: float = 0.0025,
    result: str = "pass",
    result_reason: str = "thresholds applied and satisfied",
    threshold_failures: list[str] | None = None,
    max_orientation_drift_rad: float = 0.01,
    p95_orientation_drift_rad: float = 0.01,
    fault_latched: bool = False,
) -> Path:
    artifact_dir = root / "01_ackon500_gene_sdk_pass"
    write_ablation_csv(root, artifact_dir, acceptance_semantics="socket_send_only" if socket_send_only else "controller_ack_observed")
    benchmark_start_ns = 1_000_000_000
    official_window_sec = 20.0
    benchmark_end_ns = benchmark_start_ns + int(official_window_sec * 1e9)
    acked = sent if acked is None else acked
    goal_acked = goal_sent if goal_acked is None and goal_sent is not None else goal_acked
    summary = {
        "artifact_dir": str(artifact_dir.resolve()),
        "profile": "gene_15cm_4s",
        "repeat": 5,
        "period_sec": 4.0,
        "duration_sec": official_window_sec,
        "benchmark_start_ns": benchmark_start_ns,
        "benchmark_end_ns": benchmark_end_ns,
        "tracking_source_used": "tcp_ref_stand",
        "servo_rate_hz": 500.0,
        "command_rate_hz": 500.0,
        "async_mode": "sdk_ack_worker",
        "rms_error_m": rms_error_m,
        "p95_error_m": 0.0055,
        "fit_center_error_m": 0.002,
        "radius_gain": 1.0,
        "p95_orientation_drift_rad": p95_orientation_drift_rad,
        "max_orientation_drift_rad": max_orientation_drift_rad,
        "estimated_latency_ms": 2.5,
        "commanded_phase_advance_ms": 40.0,
        "state_age_us": {"p95": 900.0},
        "feedback_saturation_count": 10,
        "command_count": 2,
        "fault_latched": fault_latched,
        "physical_motion_detected": False,
        "physical_motion_expected": False,
        "cartesian_unavailable_count": 0,
        "measurement_reliability_level": "controller_reference_valid",
        "result": result,
        "result_reason": result_reason,
        "threshold_failures": threshold_failures or [],
    }
    write_json(artifact_dir / "summary.json", summary)
    socket_sent = sent if socket_send_only else 0
    first_async = {
        "enabled": True,
        "mode": "sdk_ack_worker",
        "commands_sent_total": 1,
        "commands_acked_total": 1,
        "commands_socket_sent_total": 0,
        "first_worker_send_ns": benchmark_start_ns,
        "last_worker_send_ns": benchmark_start_ns,
    }
    last_async = {
        "enabled": True,
        "mode": "sdk_ack_worker",
        "commands_enqueued_total": sent,
        "commands_sent_total": sent,
        "commands_acked_total": acked,
        "commands_socket_sent_total": socket_sent,
        "commands_dropped_total": 0,
        "commands_overwritten_total": 0,
        "first_worker_send_ns": benchmark_start_ns,
        "last_worker_send_ns": benchmark_end_ns,
    }
    if goal_sent is not None:
        last_async.update(
            {
                "goal_window_commands_sent": goal_sent,
                "goal_window_commands_acked": goal_acked,
                "first_goal_command_send_ns": benchmark_start_ns,
                "last_goal_command_send_ns": benchmark_end_ns,
            }
        )
    first_state = {"host_time_ns": benchmark_start_ns, "loop_start_time_ns": benchmark_start_ns, "left": {"async_streaming": first_async}}
    last_state = {"host_time_ns": benchmark_end_ns, "loop_start_time_ns": benchmark_end_ns - 2_000_000, "left": {"async_streaming": last_async}}
    if include_tick_window:
        first_state["tick"] = 1
        last_state["tick"] = 10000
    write_jsonl(
        artifact_dir / "state_stream.jsonl",
        [first_state, last_state],
    )
    write_jsonl(
        artifact_dir / "command_packets.jsonl",
        [
            {"host_time_ns": benchmark_start_ns, "left": {"mode": "TcpCircleMove"}},
            {"host_time_ns": benchmark_end_ns, "left": {"mode": "Hold"}},
        ],
    )
    write_json(artifact_dir / "error_decomposition.json", {"error_classification": "goal_test"})
    return artifact_dir


def write_repeatability_candidate(
    root: Path,
    *,
    name: str,
    arm: str,
    rms_error_m: float = 0.0024,
    p95_error_m: float = 0.0048,
    estimated_latency_ms: float = 2.0,
    goal_sent: int = 10000,
    goal_acked: int | None = None,
    result: str = "pass",
    result_reason: str = "thresholds applied and satisfied",
    threshold_failures: list[str] | None = None,
    fault_latched: bool = False,
) -> None:
    artifact_dir = root / name
    benchmark_start_ns = 1_000_000_000
    official_window_sec = 20.0
    benchmark_end_ns = benchmark_start_ns + int(official_window_sec * 1e9)
    goal_acked = goal_sent if goal_acked is None else goal_acked
    summary = {
        "name": name,
        "artifact_dir": str(artifact_dir.resolve()),
        "profile": "gene_15cm_4s",
        "controller": "server_circle",
        "command_family": "server_side_circle",
        "arm": arm,
        "repeat": 5,
        "period_sec": 4.0,
        "duration_sec": official_window_sec,
        "benchmark_start_ns": benchmark_start_ns,
        "benchmark_end_ns": benchmark_end_ns,
        "tracking_source_used": "tcp_ref_stand",
        "servo_rate_hz": 500.0,
        "servo_t1_sec": 0.002,
        "command_rate_hz": 500.0,
        "async_mode": "sdk_ack_worker",
        "rms_error_m": rms_error_m,
        "p95_error_m": p95_error_m,
        "fit_center_error_m": 0.001,
        "radius_gain": 1.0,
        "p95_orientation_drift_rad": 0.01,
        "estimated_latency_ms": estimated_latency_ms,
        "commanded_phase_advance_ms": 5.0,
        "state_age_us": {"p95": 900.0},
        "feedback_saturation_count": 0,
        "command_count": 2,
        "fault_latched": fault_latched,
        "physical_motion_detected": False,
        "physical_motion_expected": False,
        "cartesian_unavailable_count": 0,
        "measurement_reliability_level": "suspect",
        "diagnostics_suspect_count": 3,
        "result": result,
        "result_reason": result_reason,
        "threshold_failures": threshold_failures or [],
    }
    write_json(artifact_dir / "summary.json", summary)
    write_jsonl(
        artifact_dir / "state_stream.jsonl",
        [
            {
                "host_time_ns": benchmark_start_ns,
                "loop_start_time_ns": benchmark_start_ns,
                "tick": 1,
                arm: {
                    "async_streaming": {
                        "enabled": True,
                        "mode": "sdk_ack_worker",
                        "commands_sent_total": 1,
                        "commands_acked_total": 1,
                        "commands_socket_sent_total": 0,
                        "first_worker_send_ns": benchmark_start_ns,
                        "last_worker_send_ns": benchmark_start_ns,
                    }
                },
            },
            {
                "host_time_ns": benchmark_end_ns,
                "loop_start_time_ns": benchmark_end_ns - 2_000_000,
                "tick": 10000,
                arm: {
                    "async_streaming": {
                        "enabled": True,
                        "mode": "sdk_ack_worker",
                        "commands_enqueued_total": goal_sent,
                        "commands_sent_total": goal_sent,
                        "commands_acked_total": goal_acked,
                        "commands_socket_sent_total": 0,
                        "commands_dropped_total": 0,
                        "commands_overwritten_total": 0,
                        "goal_window_commands_sent": goal_sent,
                        "goal_window_commands_acked": goal_acked,
                        "first_goal_command_send_ns": benchmark_start_ns,
                        "last_goal_command_send_ns": benchmark_end_ns,
                        "first_worker_send_ns": benchmark_start_ns,
                        "last_worker_send_ns": benchmark_end_ns,
                    }
                },
            },
        ],
    )
    write_jsonl(
        artifact_dir / "command_packets.jsonl",
        [
            {"host_time_ns": benchmark_start_ns, arm: {"mode": "TcpCircleMove"}},
            {"host_time_ns": benchmark_end_ns, arm: {"mode": "Hold"}},
        ],
    )
    write_json(artifact_dir / "error_decomposition.json", {"error_classification": "repeatability_test"})


def write_repeatability_set(root: Path) -> None:
    for arm in ("left", "right"):
        for index in range(1, 4):
            write_repeatability_candidate(root, name=f"best_{arm}_run{index:02d}", arm=arm)


class Ackon500GeneGoalReportTest(unittest.TestCase):
    def test_sdk_ack_worker_candidate_passes_goal_contract(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            artifact_dir = write_candidate(root)
            summary = report.build_summary(root)
            self.assertEqual(summary["result"], "pass")
            best = summary["best_candidate"]
            self.assertEqual(best["benchmark_lane"], "rbpodo_server_side_circle_ackon500_sdk_worker")
            self.assertEqual(best["low_level_send_mode"], "sdk_ack_worker")
            self.assertEqual(best["acceptance_semantics"], "sdk_worker_ack_observed")
            self.assertGreaterEqual(best["ack_coverage_ratio"], 0.98)
            self.assertEqual(best["udp_command_count"], 2)
            self.assertEqual(best["async_commands_sent_total"], 10000)
            self.assertNotEqual(best["udp_command_count"], best["async_commands_sent_total"])
            self.assertAlmostEqual(best["official_tracking_window_sec"], 20.0)
            self.assertAlmostEqual(best["effective_goal_command_rate_hz"], 500.0)
            self.assertTrue((artifact_dir / "async_ack_telemetry.jsonl").is_file())

    def test_extra_hold_commands_do_not_change_goal_rate(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_candidate(root, sent=10800, acked=10800, goal_sent=10000, goal_acked=10000)
            summary = report.build_summary(root)
            best = summary["best_candidate"]
            self.assertEqual(summary["result"], "pass")
            self.assertEqual(best["commands_sent_total"], 10800)
            self.assertEqual(best["goal_window_commands_sent"], 10000)
            self.assertEqual(best["worker_sends_outside_official_window"], 800)
            self.assertAlmostEqual(best["effective_goal_command_rate_hz"], 500.0)

    def test_total_rate_539_without_goal_window_evidence_fails(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_candidate(
                root,
                sent=10788,
                acked=10788,
                goal_sent=None,
                include_tick_window=False,
            )
            summary = report.build_summary(root)
            best = summary["best_candidate"]
            self.assertEqual(summary["result"], "fail")
            self.assertIsNone(best["effective_goal_command_rate_hz"])
            self.assertIn("goal_window_commands_sent unavailable", "\n".join(best["failures"]))

    def test_socket_send_only_candidate_fails_even_with_good_tracking(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_candidate(root, socket_send_only=True)
            summary = report.build_summary(root)
            self.assertEqual(summary["result"], "fail")
            failures = "\n".join(summary["best_candidate"]["failures"])
            self.assertEqual(
                summary["best_candidate"]["benchmark_lane"],
                "rbpodo_server_side_circle_500hz_socket_send_supervised",
            )
            self.assertIn("benchmark_lane", failures)
            self.assertIn("socket_send_only_count", failures)
            self.assertEqual(summary["best_candidate"]["ackon500_goal_status"], "fail")

    def test_goal_pass_can_carry_generic_max_orientation_threshold_failure(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_candidate(
                root,
                result="fail",
                result_reason="thresholds applied and failed",
                threshold_failures=["max_orientation_drift_rad 0.032 exceeds threshold 0.02"],
                max_orientation_drift_rad=0.032,
                p95_orientation_drift_rad=0.01,
            )
            summary = report.build_summary(root)
            best = summary["best_candidate"]
            self.assertEqual(summary["result"], "pass")
            self.assertTrue(summary["goal_pass"])
            self.assertEqual(best["run_result_status"], "completed")
            self.assertEqual(best["ackon500_goal_status"], "pass")
            self.assertEqual(best["benchmark_threshold_status"], "fail")
            self.assertIn("max_orientation_drift_spike", best["diagnostic_warnings"])
            markdown = report.report_markdown(summary)
            self.assertIn("Official goal result", markdown)
            self.assertIn("A candidate can be goal PASS", markdown)
            self.assertIn("max_orientation_drift_rad", markdown)

    def test_faulted_candidate_fails_goal_and_run_result(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_candidate(
                root,
                result="faulted",
                result_reason="server fault latched",
                fault_latched=True,
            )
            summary = report.build_summary(root)
            best = summary["best_candidate"]
            self.assertEqual(summary["result"], "fail")
            self.assertEqual(best["run_result_status"], "faulted")
            self.assertEqual(best["safety_result_status"], "fail")
            self.assertEqual(best["ackon500_goal_status"], "fail")
            self.assertIn("fault_latched", "\n".join(best["failures"]))

    def test_three_pass_repeats_classify_repeatable_pass(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_repeatability_set(root)
            summary = report.build_summary(root)
            repeatability = summary["repeatability"]
            self.assertEqual(repeatability["classification"], "repeatable_pass")
            self.assertEqual(repeatability["required_run_count"], 6)
            self.assertEqual(repeatability["required_pass_count"], 6)
            self.assertLessEqual(
                repeatability["aggregate"]["rms_median"],
                report.REPEATABILITY_THRESHOLDS["max_median_rms_error_mm"],
            )
            self.assertIn("diagnostics_suspect_caveat_remains", repeatability["caveats"])

    def test_one_performance_failure_classifies_outlier(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_repeatability_set(root)
            write_repeatability_candidate(
                root,
                name="best_right_run03",
                arm="right",
                rms_error_m=0.0042,
                p95_error_m=0.0072,
                result="fail",
                result_reason="thresholds applied and failed",
                threshold_failures=["rms_error_m 0.0042 exceeds threshold 0.003"],
            )
            repeatability = report.build_summary(root)["repeatability"]
            self.assertEqual(repeatability["classification"], "pass_with_outlier")
            self.assertEqual(repeatability["required_pass_count"], 5)
            self.assertEqual(repeatability["failed_runs"][0]["name"], "best_right_run03")

    def test_missing_right_arm_classifies_insufficient_evidence(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for index in range(1, 4):
                write_repeatability_candidate(root, name=f"best_left_run{index:02d}", arm="left")
            repeatability = report.build_summary(root)["repeatability"]
            self.assertEqual(repeatability["classification"], "insufficient_evidence")
            self.assertIn("best_right_run01", repeatability["missing_required_runs"])

    def test_cli_help_works(self) -> None:
        script = Path(__file__).with_name("generate_ackon500_gene_goal_report.py")
        completed = subprocess.run(
            [sys.executable, str(script), "--help"],
            check=True,
            stdout=subprocess.PIPE,
            text=True,
        )
        self.assertIn("ACKON500-GENE-GOAL-01", completed.stdout)


if __name__ == "__main__":
    unittest.main()
