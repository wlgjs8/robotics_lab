#!/usr/bin/env python3
"""Unit tests for circle ablation runner helpers."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import run_circle_ablation as ablation


VALID_MATRIX = """
experiments:
  - name: baseline_15cm_16s_twist_stand
    profile: circle_15cm_16s
    controller: twist_stand
    arm: left
    command_rate_hz: 500
    server_config: rb_servo_server/config/dual_simulator_tcp_acceptance.yaml
    server_config_overrides:
      servo.rate_hz: 500
      network.state_pub_rate_hz: 20
      cartesian_control.velocity_target_integration: previous_command
"""


class CircleAblationHelpersTest(unittest.TestCase):
    def test_matrix_parser_accepts_valid_matrix(self) -> None:
        experiments = ablation.parse_matrix_text(VALID_MATRIX)
        self.assertEqual(len(experiments), 1)
        self.assertEqual(experiments[0]["name"], "baseline_15cm_16s_twist_stand")
        self.assertEqual(experiments[0]["profile"], "circle_15cm_16s")
        self.assertEqual(experiments[0]["server_config_overrides"]["servo.rate_hz"], 500)
        ablation.validate_experiment(experiments[0], 1)

    def test_unknown_factor_is_rejected(self) -> None:
        matrix = """
experiments:
  - name: bad
    profile: circle_15cm_16s
    controller: twist_stand
    arm: left
    made_up_factor: 1
"""
        experiment = ablation.parse_matrix_text(matrix)[0]
        with self.assertRaisesRegex(ablation.AblationError, "unknown keys"):
            ablation.validate_experiment(experiment, 1)

    def test_gene_profile_requires_explicit_fast_stress(self) -> None:
        matrix = """
experiments:
  - name: gene_without_flag
    profile: gene_15cm_4s
    controller: twist_stand
    arm: left
"""
        experiment = ablation.parse_matrix_text(matrix)[0]
        with self.assertRaisesRegex(ablation.AblationError, "allow_fast_stress"):
            ablation.validate_experiment(experiment, 1)

    def test_invalid_real_overlay_is_rejected(self) -> None:
        with self.assertRaisesRegex(ablation.AblationError, "real run mode"):
            ablation.validate_overrides({"left_robot.run_mode": "real"}, ablation.SERVER_OVERRIDE_KEYS, "server config")

    def test_real_config_text_is_rejected(self) -> None:
        with self.assertRaisesRegex(ablation.AblationError, "run_mode: real"):
            ablation.reject_unsafe_text("left_robot:\n  run_mode : real\n", "server config")

    def test_config_overlay_creates_temp_config_without_changing_source(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / "server.yaml"
            target = root / "generated.yaml"
            original = """
left_robot:
  backend_type: simulator
  run_mode: simulation
cartesian_control:
  allow_in_simulation: true
  allow_in_real: false
  velocity_target_integration: measured_actual
  path_kp_pos: 6.0
servo:
  rate_hz: 500
network:
  state_pub_rate_hz: 20
"""
            source.write_text(original, encoding="utf-8")
            ablation.prepare_config(
                source=source,
                target=target,
                overrides={
                    "servo.rate_hz": 500,
                    "cartesian_control.velocity_target_integration": "previous_command",
                    "servo.worker_read_rate_hz": 500,
                },
                allowed_overrides=ablation.SERVER_OVERRIDE_KEYS,
                label="server config",
            )
            self.assertEqual(source.read_text(encoding="utf-8"), original)
            generated = target.read_text(encoding="utf-8")
            self.assertIn("rate_hz: 500", generated)
            self.assertIn("velocity_target_integration: previous_command", generated)
            self.assertIn("worker_read_rate_hz: 500", generated)

    def test_summary_aggregation_combines_fake_summaries(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            config = root / "generated_server_config.yaml"
            config.write_text(
                """
servo:
  rate_hz: 500
cartesian_control:
  velocity_target_integration: previous_command
  path_kp_pos: 6.0
  path_kp_ori: 6.0
  velocity_damping: 0.01
  max_twist_linear_m_s: 0.15
""",
                encoding="utf-8",
            )
            exp1 = root / "exp1"
            exp2 = root / "exp2"
            exp1.mkdir()
            exp2.mkdir()
            for exp_dir, start in ((exp1, 1_000_000_000), (exp2, 2_000_000_000)):
                (exp_dir / "command_packets.jsonl").write_text(
                    "\n".join(
                        json.dumps({"host_time_ns": start + offset, "left": {"mode": "TcpTwistStand"}})
                        for offset in (0, 10_000_000, 25_000_000)
                    )
                    + "\n",
                    encoding="utf-8",
                )
            summaries = [
                {
                    "_experiment": {"name": "a", "controller": "twist_stand", "profile": "circle_15cm_16s"},
                    "_experiment_dir": str(exp1),
                    "_generated_server_config": str(config),
                    "diameter_m": 0.15,
                    "period_sec": 16.0,
                    "command_rate_hz": 500,
                    "radius_gain": 0.98,
                    "rms_error_m": 0.002,
                    "p95_error_m": 0.003,
                    "max_error_m": 0.004,
                    "result": "completed",
                },
                {
                    "_experiment": {"name": "b", "controller": "twist_stand", "profile": "gene_15cm_4s"},
                    "_experiment_dir": str(exp2),
                    "_generated_server_config": str(config),
                    "diameter_m": 0.15,
                    "period_sec": 4.0,
                    "command_rate_hz": 500,
                    "radius_gain": 0.9,
                    "rms_error_m": 0.011,
                    "p95_error_m": 0.021,
                    "max_error_m": 0.03,
                    "result": "completed",
                },
            ]
            rows = ablation.rows_from_summaries(summaries)
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0]["rms_error_mm"], 2.0)
            self.assertEqual(rows[0]["command_interval_max_ms"], 15.0)
            self.assertIn("stress rms_error_mm", rows[1]["warnings"])

    def test_markdown_table_handles_missing_optional_fields(self) -> None:
        table = ablation.markdown_table([{"name": "partial", "result": "completed"}])
        self.assertIn("partial", table)
        self.assertIn("completed", table)


if __name__ == "__main__":
    unittest.main()
