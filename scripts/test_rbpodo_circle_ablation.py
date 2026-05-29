#!/usr/bin/env python3
"""Unit tests for rbpodo controller-simulation circle ablation runner."""

from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import run_rbpodo_circle_ablation as ablation


ENV_NAMES = (
    "RB_ALLOW_REAL_ROBOT",
    "RB_ALLOW_REAL_MOTION",
    "RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION",
    "RB_ALLOW_RBPODO_ACK_DISABLED_MOTION",
    "RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM",
    "RB_ALLOW_REAL_CARTESIAN",
)


class EnvGuard:
    def __enter__(self) -> "EnvGuard":
        self.old = {name: os.environ.get(name) for name in ENV_NAMES}
        return self

    def __exit__(self, *_exc: object) -> None:
        for name, value in self.old.items():
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value


def write_config(path: Path, *, operation_mode: str = "simulation", ack_off: bool = False) -> None:
    ack = "true" if ack_off else "false"
    path.write_text(
        f"""schema: robotics_lab.rb_servo_server.v1
left_robot:
  backend_type: rbpodo
  run_mode: real
  ip: "172.28.60.200"
  operation_mode: {operation_mode}
  servo_t1_sec: 0.01
  servo_t2_sec: 0.05
  servo_gain: 1.0
  servo_alpha: 0.5
  disable_waiting_ack: {ack}
right_robot:
  backend_type: rbpodo
  run_mode: real
  ip: "172.28.60.201"
  operation_mode: {operation_mode}
  servo_t1_sec: 0.01
  servo_t2_sec: 0.05
  servo_gain: 1.0
  servo_alpha: 0.5
  disable_waiting_ack: {ack}
servo:
  rate_hz: 100
  send_servo_commands: true
  allow_controller_simulation_motion: true
  allow_controller_simulation_diagnostics_suspect: false
network:
  command_bind: "udp://127.0.0.1:50051"
  state_pub_endpoint: "udp://127.0.0.1:50151"
cartesian_control:
  enable: true
  allow_in_simulation: true
  allow_in_real: false
  max_twist_linear_m_s: 0.15
  max_twist_angular_rad_s: 0.4
""",
        encoding="utf-8",
    )


def write_matrix(path: Path, config_name: str = "config.yaml") -> None:
    path.write_text(
        f"""experiments:
  - name: rbpodo_15cm16s_twist_stand
    config: {config_name}
    profile: circle_15cm_16s
    controller: twist_stand
    arm: left
    command_rate_hz: 100
    repeat: 1
    tracking_source: tcp_ref_stand
""",
        encoding="utf-8",
    )


def write_pgmode_summary(path: Path) -> None:
    path.write_text(
        json.dumps(
            {
                "schema": "robotics_lab.rainbow_pgmode.v1",
                "overall_result": "ok",
                "ips": ["172.28.60.200", "172.28.60.201"],
                "results": [
                    {"ip": "172.28.60.200", "ok": True, "controller_mode": "simulation"},
                    {"ip": "172.28.60.201", "ok": True, "controller_mode": "simulation"},
                ],
            }
        ),
        encoding="utf-8",
    )


class RbpodoCircleAblationTest(unittest.TestCase):
    def test_help_works(self) -> None:
        script = Path(__file__).with_name("run_rbpodo_circle_ablation.py")
        completed = subprocess.run(
            [sys.executable, str(script), "--help"],
            check=True,
            stdout=subprocess.PIPE,
            text=True,
        )
        self.assertIn("--dry-run", completed.stdout)
        self.assertIn("--i-confirm-controller-is-in-pgmode-simulation", completed.stdout)

    def test_matrix_parser_rejects_operation_mode_real_config(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text:
            root = Path(tmp_text)
            config = root / "config.yaml"
            matrix = root / "matrix.yaml"
            write_config(config, operation_mode="real")
            write_matrix(matrix)
            exp = ablation.load_matrix(matrix)[0]
            ablation.validate_experiment(exp, 1)
            with self.assertRaisesRegex(ablation.AblationError, "operation_mode is real"):
                ablation.validate_config(root, exp)

    def test_dry_run_prints_command_and_writes_artifacts(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text, EnvGuard():
            root = Path(tmp_text)
            config = root / "config.yaml"
            matrix = root / "matrix.yaml"
            pgmode = root / "pgmode.json"
            artifact_root = root / "artifacts"
            write_config(config)
            write_matrix(matrix)
            write_pgmode_summary(pgmode)
            env = os.environ.copy()
            env["RB_ALLOW_REAL_ROBOT"] = "1"
            env["RB_ALLOW_REAL_MOTION"] = "1"
            env["RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION"] = "1"
            env.pop("RB_ALLOW_REAL_CARTESIAN", None)
            script = Path(__file__).with_name("run_rbpodo_circle_ablation.py")
            completed = subprocess.run(
                [
                    sys.executable,
                    str(script),
                    "--root",
                    str(root),
                    "--matrix",
                    str(matrix),
                    "--artifact-root",
                    str(artifact_root),
                    "--server",
                    "missing_server",
                    "--dry-run",
                    "--pgmode-summary-json",
                    str(pgmode),
                    "--i-understand-this-connects-to-real-controller",
                    "--i-confirm-controller-is-in-pgmode-simulation",
                ],
                env=env,
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            self.assertIn("rbpodo_circle_tracking_benchmark.py", completed.stdout)
            self.assertTrue((artifact_root / "ablation_summary.csv").is_file())
            self.assertTrue((artifact_root / "01_rbpodo_15cm16s_twist_stand" / "experiment_command.txt").is_file())

    def test_summary_aggregation_combines_fake_summaries(self) -> None:
        exp = {
            "name": "rbpodo_gene4s_feedback_kp2",
            "profile": "gene_15cm_4s",
            "controller": "twist_stand_feedback",
            "command_rate_hz": 100,
            "tracking_source": "tcp_ref_stand",
        }
        meta = {"ack_policy": "ack_on", "servo_rate_hz": 100, "alignment_warning": ""}
        summary = {
            "controller": "twist_stand_feedback",
            "profile": "gene_15cm_4s",
            "command_rate_hz": 100,
            "tracking_source_used": "tcp_ref_stand",
            "radius_gain": 0.98,
            "rms_error_m": 0.004,
            "p95_error_m": 0.006,
            "max_error_m": 0.008,
            "p95_orientation_drift_rad": 0.001,
            "estimated_latency_ms": 12.0,
            "q_ref_update_rate_hz": 99.5,
            "send_duration_us": {"p95": 900.0},
            "ack_observed_count": 100,
            "controller_acceptance_observed_count": 100,
            "diagnostics_suspect_count": 0,
            "physical_motion_detected": False,
            "result": "completed",
        }
        rows = ablation.rows_from_summaries([summary], [exp], [meta])
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["rms_error_mm"], 4.0)
        self.assertEqual(rows[0]["send_duration_p95_us"], 900.0)
        self.assertEqual(rows[0]["ack_policy"], "ack_on")


if __name__ == "__main__":
    unittest.main()
