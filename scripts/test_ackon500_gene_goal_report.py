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


def write_candidate(root: Path, *, socket_send_only: bool = False, rms_error_m: float = 0.0025) -> Path:
    artifact_dir = root / "01_ackon500_gene_sdk_pass"
    write_ablation_csv(root, artifact_dir, acceptance_semantics="socket_send_only" if socket_send_only else "controller_ack_observed")
    summary = {
        "artifact_dir": str(artifact_dir.resolve()),
        "profile": "gene_15cm_4s",
        "repeat": 5,
        "period_sec": 4.0,
        "duration_sec": 20.0,
        "tracking_source_used": "tcp_ref_stand",
        "servo_rate_hz": 500.0,
        "command_rate_hz": 500.0,
        "async_mode": "sdk_ack_worker",
        "rms_error_m": rms_error_m,
        "p95_error_m": 0.0055,
        "fit_center_error_m": 0.002,
        "radius_gain": 1.0,
        "p95_orientation_drift_rad": 0.01,
        "estimated_latency_ms": 2.5,
        "commanded_phase_advance_ms": 40.0,
        "state_age_us": {"p95": 900.0},
        "feedback_saturation_count": 10,
        "command_count": 10001,
        "fault_latched": False,
        "physical_motion_detected": False,
        "physical_motion_expected": False,
        "cartesian_unavailable_count": 0,
        "measurement_reliability_level": "controller_reference_valid",
        "result": "pass",
    }
    write_json(artifact_dir / "summary.json", summary)
    sent = 10000
    acked = 9900
    socket_sent = sent if socket_send_only else 0
    write_jsonl(
        artifact_dir / "state_stream.jsonl",
        [
            {
                "host_time_ns": 1,
                "left": {
                    "async_streaming": {
                        "enabled": True,
                        "mode": "sdk_ack_worker",
                        "commands_sent_total": 1,
                        "commands_acked_total": 1,
                        "commands_socket_sent_total": 0,
                    }
                },
            },
            {
                "host_time_ns": 20_000_000_000,
                "left": {
                    "async_streaming": {
                        "enabled": True,
                        "mode": "sdk_ack_worker",
                        "commands_sent_total": sent,
                        "commands_acked_total": acked,
                        "commands_socket_sent_total": socket_sent,
                    }
                },
            },
        ],
    )
    write_jsonl(
        artifact_dir / "command_packets.jsonl",
        [
            {"host_time_ns": 1, "left": {"mode": "TcpTwistStand"}},
            {"host_time_ns": 20_000_000_000, "left": {"mode": "TcpTwistStand"}},
        ],
    )
    write_json(artifact_dir / "error_decomposition.json", {"error_classification": "goal_test"})
    return artifact_dir


class Ackon500GeneGoalReportTest(unittest.TestCase):
    def test_sdk_ack_worker_candidate_passes_goal_contract(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            artifact_dir = write_candidate(root)
            summary = report.build_summary(root)
            self.assertEqual(summary["result"], "pass")
            best = summary["best_candidate"]
            self.assertEqual(best["acceptance_semantics"], "sdk_worker_ack_observed")
            self.assertGreaterEqual(best["ack_observed_ratio"], 0.98)
            self.assertGreaterEqual(best["effective_command_rate_hz"], 490.0)
            self.assertTrue((artifact_dir / "async_ack_telemetry.jsonl").is_file())

    def test_socket_send_only_candidate_fails_even_with_good_tracking(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_candidate(root, socket_send_only=True)
            summary = report.build_summary(root)
            self.assertEqual(summary["result"], "fail")
            failures = "\n".join(summary["best_candidate"]["failures"])
            self.assertIn("socket_send_only_count", failures)

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
