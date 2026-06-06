#!/usr/bin/env python3
"""Unit tests for ACKON500-GENE-GOAL-01 reporting."""

from __future__ import annotations

import csv
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import generate_ackon500_gene_goal_report as report
import run_rbpodo_circle_ablation as ablation


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
    tracking_source: str = "tcp_ref_stand",
    cartesian_allow_in_real: bool = False,
    cartesian_allow_in_controller_simulation: bool = True,
    operation_mode: str = "simulation",
    async_mode: str = "sdk_ack_worker",
    command_rate_hz: float = 500.0,
    phase_advance_sec: float = 0.005,
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
    diagnostics_suspect_count: int = 0,
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
        "tracking_source_used": tracking_source,
        "servo_rate_hz": 500.0,
        "command_rate_hz": command_rate_hz,
        "phase_advance_sec": phase_advance_sec,
        "async_mode": async_mode,
        "rms_error_m": rms_error_m,
        "p95_error_m": 0.0055,
        "fit_center_error_m": 0.002,
        "radius_gain": 1.0,
        "p95_orientation_drift_rad": p95_orientation_drift_rad,
        "max_orientation_drift_rad": max_orientation_drift_rad,
        "estimated_latency_ms": 2.5,
        "commanded_phase_advance_ms": phase_advance_sec * 1000.0,
        "state_age_us": {"p95": 900.0},
        "feedback_saturation_count": 10,
        "command_count": 2,
        "fault_latched": fault_latched,
        "physical_motion_detected": False,
        "physical_motion_expected": False,
        "disable_waiting_ack": socket_send_only,
        "left_disable_waiting_ack": socket_send_only,
        "right_disable_waiting_ack": socket_send_only,
        "left_operation_mode": operation_mode,
        "right_operation_mode": operation_mode,
        "cartesian_allow_in_controller_simulation": cartesian_allow_in_controller_simulation,
        "cartesian_allow_in_real": cartesian_allow_in_real,
        "cartesian_unavailable_count": 0,
        "measurement_reliability_level": "controller_reference_valid",
        "diagnostics_suspect_count": diagnostics_suspect_count,
        "result": result,
        "result_reason": result_reason,
        "threshold_failures": threshold_failures or [],
    }
    write_json(artifact_dir / "summary.json", summary)
    socket_sent = sent if socket_send_only else 0
    first_async = {
        "enabled": True,
        "mode": async_mode,
        "commands_sent_total": 1,
        "commands_acked_total": 1,
        "commands_socket_sent_total": 0,
        "first_worker_send_ns": benchmark_start_ns,
        "last_worker_send_ns": benchmark_start_ns,
    }
    last_async = {
        "enabled": True,
        "mode": async_mode,
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
    profile: str = "gene_15cm_4s",
    controller: str = "server_circle",
    tracking_source: str = "tcp_ref_stand",
    command_rate_hz: float = 500.0,
    phase_advance_sec: float = 0.005,
    async_mode: str = "sdk_ack_worker",
    socket_send_only: bool = False,
    cartesian_allow_in_real: bool = False,
    cartesian_allow_in_controller_simulation: bool = True,
    operation_mode: str = "simulation",
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
        "profile": profile,
        "controller": controller,
        "command_family": "server_side_circle" if controller == "server_circle" else "python_streaming",
        "arm": arm,
        "repeat": 5,
        "period_sec": 4.0,
        "duration_sec": official_window_sec,
        "benchmark_start_ns": benchmark_start_ns,
        "benchmark_end_ns": benchmark_end_ns,
        "tracking_source_used": tracking_source,
        "servo_rate_hz": 500.0,
        "servo_t1_sec": 0.002,
        "command_rate_hz": command_rate_hz,
        "phase_advance_sec": phase_advance_sec,
        "async_mode": async_mode,
        "acceptance_semantics": "socket_send_only" if socket_send_only else "controller_ack_observed",
        "rms_error_m": rms_error_m,
        "p95_error_m": p95_error_m,
        "fit_center_error_m": 0.001,
        "radius_gain": 1.0,
        "p95_orientation_drift_rad": 0.01,
        "estimated_latency_ms": estimated_latency_ms,
        "commanded_phase_advance_ms": phase_advance_sec * 1000.0,
        "state_age_us": {"p95": 900.0},
        "feedback_saturation_count": 0,
        "command_count": 2,
        "fault_latched": fault_latched,
        "physical_motion_detected": False,
        "physical_motion_expected": False,
        "disable_waiting_ack": socket_send_only,
        "left_disable_waiting_ack": socket_send_only,
        "right_disable_waiting_ack": socket_send_only,
        "left_operation_mode": operation_mode,
        "right_operation_mode": operation_mode,
        "cartesian_allow_in_controller_simulation": cartesian_allow_in_controller_simulation,
        "cartesian_allow_in_real": cartesian_allow_in_real,
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
                        "mode": async_mode,
                        "commands_sent_total": 1,
                        "commands_acked_total": 1,
                        "commands_socket_sent_total": 1 if socket_send_only else 0,
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
                        "mode": async_mode,
                        "commands_enqueued_total": goal_sent,
                        "commands_sent_total": goal_sent,
                        "commands_acked_total": goal_acked,
                        "commands_socket_sent_total": goal_sent if socket_send_only else 0,
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
            {"host_time_ns": benchmark_start_ns, arm: {"mode": "TcpCircleMove" if controller == "server_circle" else "TcpTwistStand"}},
            {"host_time_ns": benchmark_end_ns, arm: {"mode": "Hold"}},
        ],
    )
    write_json(artifact_dir / "error_decomposition.json", {"error_classification": "repeatability_test"})


def write_repeatability_set(root: Path) -> None:
    for arm in ("left", "right"):
        for index in range(1, 4):
            write_repeatability_candidate(root, name=f"best_{arm}_run{index:02d}", arm=arm)


def wrapper_env(*, include_required: bool = False) -> dict[str, str]:
    env = os.environ.copy()
    for key in (
        "RB_ALLOW_REAL_ROBOT",
        "RB_ALLOW_REAL_MOTION",
        "RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION",
        "RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN",
        "RB_ALLOW_RBPODO_ASYNC_STREAMING",
        "RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM",
        "RB_RBPODO_PGMODE_SIMULATION_CONFIRMED",
        "RB_ALLOW_REAL_CARTESIAN",
    ):
        env.pop(key, None)
    if include_required:
        env.update(
            {
                "RB_ALLOW_REAL_ROBOT": "1",
                "RB_ALLOW_REAL_MOTION": "1",
                "RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION": "1",
                "RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN": "1",
                "RB_ALLOW_RBPODO_ASYNC_STREAMING": "1",
                "RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM": "1",
                "RB_RBPODO_PGMODE_SIMULATION_CONFIRMED": "1",
            }
        )
    return env


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
            self.assertEqual(summary["controller_reference_result"]["status"], "pass")
            self.assertEqual(summary["physical_readiness"]["status"], "blocked")
            self.assertIn("diagnostics_suspect_unresolved", summary["physical_readiness"]["blockers"])
            self.assertEqual(summary["physical_tracking_result"]["status"], "not_measured")
            self.assertNotEqual(summary["physical_tracking_result"]["status"], "pass")
            self.assertTrue((artifact_dir / "async_ack_telemetry.jsonl").is_file())

    def test_goal_pass_with_diagnostics_suspect_still_blocks_physical_readiness(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_candidate(root, diagnostics_suspect_count=2)
            summary = report.build_summary(root)
            self.assertEqual(summary["result"], "pass")
            self.assertEqual(summary["controller_reference_result"]["status"], "pass")
            self.assertEqual(summary["physical_readiness"]["status"], "blocked")
            self.assertEqual(
                summary["physical_readiness"]["blockers"],
                [
                    "diagnostics_suspect_unresolved",
                    "physical_reference_to_actual_error_unmeasured",
                    "stop_resetFault_unverified",
                    "camera_tcp_calibration_unresolved",
                    "no_tiny_physical_acceptance",
                ],
            )
            self.assertEqual(summary["physical_tracking_result"]["status"], "not_measured")

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
            self.assertTrue(repeatability["global_repeatability_pass"])
            self.assertEqual(repeatability["left_arm_aggregate"]["status"], "pass")
            self.assertEqual(repeatability["right_arm_aggregate"]["status"], "pass")
            self.assertEqual(repeatability["required_run_count"], 6)
            self.assertEqual(repeatability["required_pass_count"], 6)
            self.assertLessEqual(
                repeatability["aggregate"]["rms_median"],
                report.REPEATABILITY_THRESHOLDS["max_median_rms_error_mm"],
            )
            self.assertIn("diagnostics_suspect_caveat_remains", repeatability["caveats"])

    def test_one_performance_failure_blocks_global_repeatability(self) -> None:
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
            self.assertEqual(repeatability["classification"], "not_repeatable")
            self.assertFalse(repeatability["global_repeatability_pass"])
            self.assertEqual(repeatability["left_arm_aggregate"]["status"], "pass")
            self.assertEqual(repeatability["right_arm_aggregate"]["status"], "fail")
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

    def test_repeatability_rejects_socket_send_only_required_row(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_repeatability_set(root)
            write_repeatability_candidate(root, name="best_left_run01", arm="left", socket_send_only=True)
            repeatability = report.build_summary(root)["repeatability"]
            self.assertEqual(repeatability["classification"], "not_repeatable")
            failures = "\n".join(repeatability["left_arm_aggregate"]["reasons"])
            self.assertIn("socket_send_only_count", failures)
            self.assertEqual(repeatability["left_arm_aggregate"]["status"], "fail")

    def test_repeatability_rejects_wrong_benchmark_lane(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_repeatability_set(root)
            write_repeatability_candidate(root, name="best_left_run02", arm="left", controller="twist_stand")
            repeatability = report.build_summary(root)["repeatability"]
            self.assertEqual(repeatability["classification"], "not_repeatable")
            failures = "\n".join(repeatability["left_arm_aggregate"]["reasons"])
            self.assertIn("benchmark_lane", failures)
            self.assertIn("controller twist_stand != server_circle", failures)

    def test_repeatability_rejects_wrong_tracking_source(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_repeatability_set(root)
            write_repeatability_candidate(
                root,
                name="best_right_run01",
                arm="right",
                tracking_source="tcp_actual_stand",
            )
            repeatability = report.build_summary(root)["repeatability"]
            self.assertEqual(repeatability["classification"], "not_repeatable")
            failures = "\n".join(repeatability["right_arm_aggregate"]["reasons"])
            self.assertIn("tracking_source tcp_actual_stand != tcp_ref_stand", failures)

    def test_repeatability_rejects_allow_in_real_true(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_repeatability_set(root)
            write_repeatability_candidate(
                root,
                name="best_right_run02",
                arm="right",
                cartesian_allow_in_real=True,
            )
            repeatability = report.build_summary(root)["repeatability"]
            self.assertEqual(repeatability["classification"], "not_repeatable")
            failures = "\n".join(repeatability["right_arm_aggregate"]["reasons"])
            self.assertIn("cartesian_allow_in_real True is not false", failures)

    def test_repeatability_rejects_operation_mode_real(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_repeatability_set(root)
            write_repeatability_candidate(
                root,
                name="best_left_run03",
                arm="left",
                operation_mode="real",
            )
            repeatability = report.build_summary(root)["repeatability"]
            self.assertEqual(repeatability["classification"], "not_repeatable")
            failures = "\n".join(repeatability["left_arm_aggregate"]["reasons"])
            self.assertIn("left_operation_mode real is not simulation", failures)

    def test_repeatability_report_prints_caveat_and_groupings(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_repeatability_set(root)
            repeatability = report.build_summary(root)["repeatability"]
            markdown = report.repeatability_markdown(repeatability)
            self.assertIn("The diagnostics_suspect caveat remains", markdown)
            self.assertIn("physical_readiness.status: `blocked`", markdown)
            self.assertIn("Rows By Arm", markdown)
            self.assertIn("Rows By Benchmark Lane", markdown)
            self.assertIn("Rows By Acceptance Semantics", markdown)
            self.assertIn("Rows By Tracking Source", markdown)

    def test_repeatability_run_name_layout_uses_required_artifact_prefix(self) -> None:
        path = ablation.experiment_dir(
            Path("/tmp/artifacts"),
            1,
            {"name": "best_left_run01"},
            "run-name",
        )
        self.assertEqual(path.name, "run_best_left_run01")

    def test_wrapper_repeatability_dry_run_without_env_prints_commands(self) -> None:
        repo = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as tmp:
            completed = subprocess.run(
                [
                    "bash",
                    "tools/rbpodo_ackon500_gene_goal.sh",
                    "--profile",
                    "repeatability",
                    "--dry-run",
                    "--artifact-root",
                    str(Path(tmp) / "repeatability"),
                    "--i-understand-this-connects-to-real-controller",
                    "--i-confirm-controller-is-in-pgmode-simulation",
                ],
                cwd=repo,
                env=wrapper_env(),
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            self.assertIn("dry-run: required controller-simulation env gates were not checked", completed.stdout)
            self.assertIn("dry-run: server realtime capabilities were not checked", completed.stdout)
            self.assertIn("run_rbpodo_circle_ablation.py", completed.stdout)
            self.assertIn("--run-dir-layout", completed.stdout)
            self.assertIn("run-name", completed.stdout)
            self.assertIn("--require-repeatable", completed.stdout)
            self.assertIn("repeatability_report.md", completed.stdout)

    def test_wrapper_rejects_missing_pgmode_confirmation(self) -> None:
        repo = Path(__file__).resolve().parents[1]
        completed = subprocess.run(
            [
                "bash",
                "tools/rbpodo_ackon500_gene_goal.sh",
                "--profile",
                "repeatability",
                "--dry-run",
                "--i-understand-this-connects-to-real-controller",
            ],
            cwd=repo,
            env=wrapper_env(),
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("missing --i-confirm-controller-is-in-pgmode-simulation", completed.stderr)

    def test_wrapper_rejects_real_cartesian_env_even_in_dry_run(self) -> None:
        repo = Path(__file__).resolve().parents[1]
        env = wrapper_env()
        env["RB_ALLOW_REAL_CARTESIAN"] = "1"
        completed = subprocess.run(
            [
                "bash",
                "tools/rbpodo_ackon500_gene_goal.sh",
                "--profile",
                "repeatability",
                "--dry-run",
                "--i-understand-this-connects-to-real-controller",
                "--i-confirm-controller-is-in-pgmode-simulation",
            ],
            cwd=repo,
            env=env,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("RB_ALLOW_REAL_CARTESIAN must not be set", completed.stderr)

    def test_wrapper_rejects_missing_realtime_caps_before_running(self) -> None:
        repo = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as tmp:
            server = Path(tmp) / "rb_servo_server"
            server.write_text("#!/bin/sh\n", encoding="utf-8")
            server.chmod(0o755)
            completed = subprocess.run(
                [
                    "bash",
                    "tools/rbpodo_ackon500_gene_goal.sh",
                    "--profile",
                    "repeatability",
                    "--server",
                    str(server),
                    "--artifact-root",
                    str(Path(tmp) / "repeatability"),
                    "--skip-noop",
                    "--i-understand-this-connects-to-real-controller",
                    "--i-confirm-controller-is-in-pgmode-simulation",
                ],
                cwd=repo,
                env=wrapper_env(include_required=True),
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("--allow-no-realtime", completed.stderr)

    def test_wrapper_dry_run_rejects_explicit_server_without_realtime_caps(self) -> None:
        repo = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as tmp:
            server = Path(tmp) / "rb_servo_server"
            server.write_text("#!/bin/sh\n", encoding="utf-8")
            server.chmod(0o755)
            completed = subprocess.run(
                [
                    "bash",
                    "tools/rbpodo_ackon500_gene_goal.sh",
                    "--profile",
                    "repeatability",
                    "--server",
                    str(server),
                    "--artifact-root",
                    str(Path(tmp) / "repeatability"),
                    "--dry-run",
                    "--i-understand-this-connects-to-real-controller",
                    "--i-confirm-controller-is-in-pgmode-simulation",
                ],
                cwd=repo,
                env=wrapper_env(),
                check=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("--allow-no-realtime", completed.stderr)

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
