#!/usr/bin/env python3
"""Unit tests for rbpodo controller-simulation circle ablation runner."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import run_rbpodo_circle_ablation as ablation


ENV_NAMES = (
    "RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM",
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
  speed_bar: 0.1
  servo_t1_sec: 0.002
  servo_t2_sec: 0.05
  servo_gain: 1.0
  servo_alpha: 0.5
  disable_waiting_ack: {ack}
right_robot:
  backend_type: rbpodo
  run_mode: real
  ip: "172.28.60.201"
  operation_mode: {operation_mode}
  speed_bar: 0.1
  servo_t1_sec: 0.002
  servo_t2_sec: 0.05
  servo_gain: 1.0
  servo_alpha: 0.5
  disable_waiting_ack: {ack}
servo:
  rate_hz: 500
  send_servo_commands: true
  allow_servo_t1_rate_mismatch: false
  allow_controller_simulation_motion: true
  allow_controller_simulation_diagnostics_suspect: false
network:
  command_bind: "udp://127.0.0.1:50051"
  state_pub_endpoint: "udp://127.0.0.1:50151"
  state_pub_rate_hz: 50
cartesian_control:
  enable: true
  allow_in_simulation: true
  allow_in_real: false
  allow_in_controller_simulation: true
  max_twist_linear_m_s: 0.15
  max_twist_angular_rad_s: 0.4
  max_linear_move_speed_m_s: 0.15
  path_kp_pos: 6.0
  path_kp_ori: 6.0
  twist_angular_deadband_rad_s: 0.0001
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
    command_rate_hz: 500
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
                ],
                env=env,
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            self.assertIn("rbpodo_circle_tracking_benchmark.py", completed.stdout)
            self.assertIn("resolved_config:", completed.stdout)
            self.assertIn("resolved_server_config.yaml", completed.stdout)
            self.assertTrue((artifact_root / "ablation_summary.csv").is_file())
            self.assertTrue((artifact_root / "matrix_resolved.yaml").is_file())
            exp_dir = artifact_root / "01_rbpodo_15cm16s_twist_stand"
            self.assertTrue((exp_dir / "experiment_command.txt").is_file())
            self.assertTrue((exp_dir / "resolved_server_config.yaml").is_file())

    def test_config_overrides_apply_to_temporary_config_without_mutating_source(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text:
            root = Path(tmp_text)
            config = root / "config.yaml"
            write_config(config)
            source_text = config.read_text(encoding="utf-8")
            exp = {
                "name": "gene_fb_kp10_pub100_speed02",
                "config": "config.yaml",
                "profile": "gene_15cm_4s",
                "controller": "twist_stand_feedback",
                "arm": "left",
                "config_overrides": {
                    "network.state_pub_rate_hz": 100,
                    "left_robot.speed_bar": 0.2,
                    "right_robot.speed_bar": 0.2,
                    "cartesian_control.max_twist_linear_m_s": 0.2,
                    "cartesian_control.max_linear_move_speed_m_s": 0.12,
                    "cartesian_control.twist_angular_deadband_rad_s": 0.0002,
                },
            }
            ablation.validate_experiment(exp, 1)
            meta = ablation.prepare_experiment_config(root, exp, root / "artifacts" / "01_gene")
            resolved = Path(meta["resolved_config_path"]).read_text(encoding="utf-8")
            self.assertIn("state_pub_rate_hz: 100", resolved)
            self.assertIn("speed_bar: 0.2", resolved)
            self.assertIn("max_twist_linear_m_s: 0.2", resolved)
            self.assertIn("max_linear_move_speed_m_s: 0.12", resolved)
            self.assertIn("twist_angular_deadband_rad_s: 0.0002", resolved)
            self.assertEqual(config.read_text(encoding="utf-8"), source_text)
            self.assertEqual(meta["state_pub_rate_hz"], 100.0)
            self.assertEqual(meta["speed_bar_left"], 0.2)
            self.assertTrue(meta["source_config_unchanged"])

    def test_servo_t2_alpha_overrides_apply_to_generated_config_only(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text:
            root = Path(tmp_text)
            config = root / "config.yaml"
            write_config(config)
            source_text = config.read_text(encoding="utf-8")
            exp = {
                "name": "gene_fb_t2_alpha",
                "config": "config.yaml",
                "profile": "gene_15cm_4s",
                "controller": "twist_stand_feedback",
                "arm": "left",
                "config_overrides": {
                    "left_robot.servo_t2_sec": 0.03,
                    "right_robot.servo_t2_sec": 0.08,
                    "left_robot.servo_alpha": 0.3,
                    "right_robot.servo_alpha": 0.8,
                },
            }
            ablation.validate_experiment(exp, 1)
            meta = ablation.prepare_experiment_config(root, exp, root / "artifacts" / "01_gene")
            resolved_path = Path(meta["resolved_config_path"])
            resolved = resolved_path.read_text(encoding="utf-8")
            self.assertTrue(resolved_path.is_file())
            self.assertIn("servo_t2_sec: 0.03", resolved)
            self.assertIn("servo_t2_sec: 0.08", resolved)
            self.assertIn("servo_alpha: 0.3", resolved)
            self.assertIn("servo_alpha: 0.8", resolved)
            self.assertEqual(config.read_text(encoding="utf-8"), source_text)
            self.assertTrue(meta["source_config_unchanged"])
            self.assertEqual(meta["servo_t2_sec"], "0.03/0.08")
            self.assertEqual(meta["servo_t2_sec_left"], 0.03)
            self.assertEqual(meta["servo_t2_sec_right"], 0.08)
            self.assertEqual(meta["servo_alpha"], "0.3/0.8")
            self.assertEqual(meta["servo_alpha_left"], 0.3)
            self.assertEqual(meta["servo_alpha_right"], 0.8)

    def test_servo_t2_alpha_overrides_reject_out_of_range_values(self) -> None:
        base = {
            "name": "bad_servo_param",
            "config": "config.yaml",
            "profile": "gene_15cm_4s",
            "controller": "twist_stand_feedback",
            "arm": "left",
        }
        invalid_cases = (
            ("left_robot.servo_t2_sec", 0.02, r"> 0\.02"),
            ("left_robot.servo_t2_sec", 0.2, r"< 0\.2"),
            ("right_robot.servo_alpha", 0.0, r"> 0"),
            ("right_robot.servo_alpha", 1.0, r"< 1\.0"),
        )
        for key, value, pattern in invalid_cases:
            with self.subTest(key=key, value=value):
                exp = dict(base)
                exp["config_overrides"] = {key: value}
                with self.assertRaisesRegex(ablation.AblationError, pattern):
                    ablation.validate_experiment(exp, 1)

    def test_resolved_config_rejects_out_of_range_servo_t2_alpha(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text:
            root = Path(tmp_text)
            config = root / "config.yaml"
            write_config(config)
            exp = {
                "name": "bad_source_servo_param",
                "config": "config.yaml",
                "profile": "gene_15cm_4s",
                "controller": "twist_stand_feedback",
                "arm": "left",
            }
            text = config.read_text(encoding="utf-8")
            config.write_text(text.replace("servo_t2_sec: 0.05", "servo_t2_sec: 0.2", 1), encoding="utf-8")
            with self.assertRaisesRegex(ablation.AblationError, r"left_robot\.servo_t2_sec"):
                ablation.prepare_experiment_config(root, exp, root / "artifacts" / "01_bad_t2")

    def test_relative_urdf_path_resolves_from_source_config_not_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text:
            root = Path(tmp_text)
            config_dir = root / "rb_servo_server" / "config" / "local"
            description_dir = root / "rb_servo_server" / "descriptions" / "urdf"
            config_dir.mkdir(parents=True)
            description_dir.mkdir(parents=True)
            urdf = description_dir / "rb3_730e.urdf"
            urdf.write_text("<robot name=\"test\" />\n", encoding="utf-8")
            config = config_dir / "config.yaml"
            write_config(config)
            config.write_text(
                config.read_text(encoding="utf-8")
                + "\nkinematics:\n  urdf: \"../descriptions/urdf/rb3_730e.urdf\"\n",
                encoding="utf-8",
            )
            source_text = config.read_text(encoding="utf-8")
            exp = {
                "name": "gene",
                "config": "rb_servo_server/config/local/config.yaml",
                "profile": "gene_15cm_4s",
                "controller": "twist_stand",
                "arm": "left",
                "config_overrides": {"network.state_pub_rate_hz": 100},
            }
            meta = ablation.prepare_experiment_config(root, exp, root / "artifacts" / "01_gene")
            resolved = Path(meta["resolved_config_path"]).read_text(encoding="utf-8")
            self.assertIn(f'urdf: "{urdf.resolve()}"', resolved)
            self.assertEqual(config.read_text(encoding="utf-8"), source_text)

    def test_matrix_yaml_config_overrides_parse(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text:
            root = Path(tmp_text)
            config = root / "config.yaml"
            matrix = root / "matrix.yaml"
            write_config(config)
            matrix.write_text(
                """experiments:
  - name: gene_fb_kp10_pub100_speed02
    config: config.yaml
    profile: gene_15cm_4s
    controller: twist_stand_feedback
    arm: left
    config_overrides:
      network.state_pub_rate_hz: 100
      left_robot.speed_bar: 0.2
      right_robot.speed_bar: 0.2
""",
                encoding="utf-8",
            )
            exp = ablation.load_matrix(matrix)[0]
            ablation.validate_experiment(exp, 1)
            meta = ablation.prepare_experiment_config(root, exp, root / "artifacts" / "01_gene")
            self.assertEqual(meta["config_overrides"]["network.state_pub_rate_hz"], 100)
            self.assertEqual(meta["speed_bar_right"], 0.2)

    def test_gene_profile_command_includes_fast_stress_opt_in(self) -> None:
        args = argparse.Namespace(
            root=Path("/repo"),
            server=Path("missing_server"),
            skip_plots=False,
            set_pgmode_simulation=False,
            verify_pgmode_simulation=False,
            pgmode_summary_json=None,
            pgmode_timeout_sec=1.0,
            pgmode_command_port=5000,
        )
        exp = {
            "name": "fb_pos05_ori02_pub50_speed01",
            "profile": "gene_15cm_4s",
            "controller": "twist_stand_feedback",
            "arm": "left",
        }
        meta = {"config_path": "/tmp/resolved_server_config.yaml"}
        command = ablation.benchmark_command(args, exp, meta, Path("/tmp/artifacts"))
        self.assertIn("--allow-fast-stress", command)

    def test_feedback_orientation_gain_zero_is_valid_for_gain_split(self) -> None:
        exp = {
            "name": "fb_pos05_ori00_pub50_speed01",
            "config": "config.yaml",
            "profile": "gene_15cm_4s",
            "controller": "twist_stand_feedback",
            "arm": "left",
            "feedback_kp_pos": 0.5,
            "feedback_kp_ori": 0.0,
        }
        ablation.validate_experiment(exp, 1)
        exp["feedback_kp_ori"] = -0.1
        with self.assertRaisesRegex(ablation.AblationError, "feedback_kp_ori"):
            ablation.validate_experiment(exp, 1)

    def test_config_override_rejects_unsafe_allow_in_real(self) -> None:
        exp = {
            "name": "unsafe",
            "config": "config.yaml",
            "profile": "circle_15cm_16s",
            "controller": "twist_stand",
            "arm": "left",
            "config_overrides": {"cartesian_control.allow_in_real": True},
        }
        with self.assertRaisesRegex(ablation.AblationError, "allow_in_real"):
            ablation.validate_experiment(exp, 1)

    def test_config_override_allows_only_safe_cartesian_gate_values(self) -> None:
        exp = {
            "name": "safe_cartesian_gates",
            "config": "config.yaml",
            "profile": "circle_15cm_16s",
            "controller": "twist_stand",
            "arm": "left",
            "config_overrides": {
                "cartesian_control.allow_in_controller_simulation": True,
                "cartesian_control.allow_in_real": False,
            },
        }
        ablation.validate_experiment(exp, 1)
        exp["config_overrides"] = {"cartesian_control.allow_in_controller_simulation": False}
        with self.assertRaisesRegex(ablation.AblationError, "allow_in_controller_simulation"):
            ablation.validate_experiment(exp, 1)

    def test_config_override_rejects_operation_mode_real(self) -> None:
        exp = {
            "name": "unsafe",
            "config": "config.yaml",
            "profile": "circle_15cm_16s",
            "controller": "twist_stand",
            "arm": "left",
            "config_overrides": {"left_robot.operation_mode": "real"},
        }
        with self.assertRaisesRegex(ablation.AblationError, "operation_mode"):
            ablation.validate_experiment(exp, 1)

    def test_config_override_rejects_unknown_key(self) -> None:
        exp = {
            "name": "unknown",
            "config": "config.yaml",
            "profile": "circle_15cm_16s",
            "controller": "twist_stand",
            "arm": "left",
            "config_overrides": {"network.nope": 1},
        }
        with self.assertRaisesRegex(ablation.AblationError, "unknown config override"):
            ablation.validate_experiment(exp, 1)

    def test_config_override_rejects_controller_sim_motion_disable(self) -> None:
        exp = {
            "name": "unsafe",
            "config": "config.yaml",
            "profile": "circle_15cm_16s",
            "controller": "twist_stand",
            "arm": "left",
            "config_overrides": {"servo.allow_controller_simulation_motion": False},
        }
        with self.assertRaisesRegex(ablation.AblationError, "allow_controller_simulation_motion"):
            ablation.validate_experiment(exp, 1)

    def test_config_override_allows_state_pub_rate_500(self) -> None:
        exp = {
            "name": "pub_rate_500",
            "config": "config.yaml",
            "profile": "circle_15cm_16s",
            "controller": "twist_stand",
            "arm": "left",
            "config_overrides": {"network.state_pub_rate_hz": 500},
        }
        ablation.validate_experiment(exp, 1)

    def test_config_override_rejects_state_pub_rate_over_500(self) -> None:
        exp = {
            "name": "bad_pub_rate",
            "config": "config.yaml",
            "profile": "circle_15cm_16s",
            "controller": "twist_stand",
            "arm": "left",
            "config_overrides": {"network.state_pub_rate_hz": 501},
        }
        with self.assertRaisesRegex(ablation.AblationError, "<= 500"):
            ablation.validate_experiment(exp, 1)

    def test_config_override_rejects_rate_t1_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text:
            root = Path(tmp_text)
            config = root / "config.yaml"
            write_config(config)
            exp = {
                "name": "bad_rate",
                "config": "config.yaml",
                "profile": "circle_15cm_16s",
                "controller": "twist_stand",
                "arm": "left",
                "config_overrides": {"servo.rate_hz": 250},
            }
            ablation.validate_experiment(exp, 1)
            with self.assertRaisesRegex(ablation.AblationError, "servo rate/t1 mismatch"):
                ablation.prepare_experiment_config(root, exp, root / "artifacts" / "01_bad_rate")

    def test_config_override_accepts_rate_when_both_t1_values_match(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text:
            root = Path(tmp_text)
            config = root / "config.yaml"
            write_config(config)
            exp = {
                "name": "good_rate",
                "config": "config.yaml",
                "profile": "circle_15cm_16s",
                "controller": "twist_stand",
                "arm": "left",
                "config_overrides": {
                    "servo.rate_hz": 500,
                    "left_robot.servo_t1_sec": 0.002,
                    "right_robot.servo_t1_sec": 0.002,
                },
            }
            ablation.validate_experiment(exp, 1)
            meta = ablation.prepare_experiment_config(root, exp, root / "artifacts" / "01_good_rate")
            resolved = Path(meta["resolved_config_path"]).read_text(encoding="utf-8")
            self.assertIn("rate_hz: 500", resolved)
            self.assertEqual(meta["servo_rate_hz"], 500.0)
            self.assertTrue(meta["servo_t1_rate_aligned"])

    def test_summary_aggregation_combines_fake_summaries(self) -> None:
        exp = {
            "name": "rbpodo_gene4s_feedback_kp2",
            "profile": "gene_15cm_4s",
            "controller": "twist_stand_feedback",
            "command_rate_hz": 500,
            "tracking_source": "tcp_ref_stand",
            "feedback_max_linear_m_s": 0.15,
            "feedback_max_angular_rad_s": 0.4,
        }
        meta = {
            "ack_policy": "ack_on",
            "servo_rate_hz": 500,
            "servo_t2_sec": "0.03/0.08",
            "servo_t2_sec_left": 0.03,
            "servo_t2_sec_right": 0.08,
            "servo_alpha": "0.3/0.8",
            "servo_alpha_left": 0.3,
            "servo_alpha_right": 0.8,
            "alignment_warning": "",
        }
        summary = {
            "controller": "twist_stand_feedback",
            "profile": "gene_15cm_4s",
            "command_rate_hz": 500,
            "tracking_source_used": "tcp_ref_stand",
            "feedback_kp_pos": 2.0,
            "feedback_kp_ori": 2.0,
            "radius_gain": 0.98,
            "rms_error_m": 0.004,
            "p95_error_m": 0.006,
            "max_error_m": 0.008,
            "p95_orientation_drift_rad": 0.001,
            "fit_center_error_m": 0.002,
            "estimated_latency_ms": 12.0,
            "q_ref_update_rate_hz": 499.5,
            "send_duration_us": {"p95": 900.0},
            "feedback_saturation_count": 3,
            "ack_observed_count": 100,
            "controller_acceptance_observed_count": 100,
            "diagnostics_suspect_count": 0,
            "physical_motion_detected": False,
            "fault_latched": False,
            "result": "completed",
        }
        rows = ablation.rows_from_summaries([summary], [exp], [meta])
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["rms_error_mm"], 4.0)
        self.assertEqual(rows[0]["send_duration_p95_us"], 900.0)
        self.assertEqual(rows[0]["ack_policy"], "ack_on")
        self.assertEqual(rows[0]["feedback_kp_pos"], 2.0)
        self.assertEqual(rows[0]["feedback_max_linear_m_s"], 0.15)
        self.assertEqual(rows[0]["feedback_saturation_count"], 3)
        self.assertIn("benchmark_lane", ablation.SUMMARY_COLUMNS)
        self.assertEqual(rows[0]["benchmark_lane"], "rbpodo_python_streaming_feedback")
        self.assertEqual(rows[0]["control_loop_location"], "python_benchmark")
        self.assertEqual(rows[0]["p95_orientation_drift_rad"], 0.001)
        self.assertEqual(rows[0]["fit_center_error_m"], 0.002)
        self.assertFalse(rows[0]["fault_latched"])
        self.assertEqual(rows[0]["servo_t2_sec"], "0.03/0.08")
        self.assertEqual(rows[0]["servo_t2_sec_left"], 0.03)
        self.assertEqual(rows[0]["servo_t2_sec_right"], 0.08)
        self.assertEqual(rows[0]["servo_alpha"], "0.3/0.8")
        self.assertEqual(rows[0]["servo_alpha_left"], 0.3)
        self.assertEqual(rows[0]["servo_alpha_right"], 0.8)


if __name__ == "__main__":
    unittest.main()
