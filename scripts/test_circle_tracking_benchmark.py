#!/usr/bin/env python3
"""Unit tests for circle_tracking_benchmark helper semantics."""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import circle_tracking_benchmark as bench
import compare_circle_benchmarks as compare_bench
import generate_circle_benchmark_report as report_bench
import rbpodo_circle_tracking_benchmark as rbpodo_bench


def args_with_thresholds(**overrides: float | None) -> argparse.Namespace:
    values: dict[str, float | None] = {
        "max_allowed_rms_error_m": None,
        "max_allowed_p95_error_m": None,
        "max_allowed_orientation_drift_rad": None,
        "max_allowed_latency_ms": None,
    }
    values.update(overrides)
    return argparse.Namespace(**values)


RBPODO_ENV_NAMES = (
    "RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM",
)


class EnvGuard:
    def __enter__(self) -> "EnvGuard":
        self.old = {name: os.environ.get(name) for name in RBPODO_ENV_NAMES}
        return self

    def __exit__(self, *_exc: object) -> None:
        for name, value in self.old.items():
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value


def write_rbpodo_config(path: Path, *, allow_controller_sim_cartesian: bool = True) -> None:
    path.write_text(
        f"""schema: robotics_lab.rb_servo_server.v1
left_robot:
  backend_type: rbpodo
  run_mode: real
  ip: "172.28.60.200"
  operation_mode: simulation
  servo_t1_sec: 0.002
  servo_t2_sec: 0.05
  servo_gain: 1.0
  servo_alpha: 0.5
  disable_waiting_ack: false
right_robot:
  backend_type: rbpodo
  run_mode: real
  ip: "172.28.60.201"
  operation_mode: simulation
  servo_t1_sec: 0.002
  servo_t2_sec: 0.05
  servo_gain: 1.0
  servo_alpha: 0.5
  disable_waiting_ack: false
servo:
  rate_hz: 500
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
  allow_in_controller_simulation: {str(allow_controller_sim_cartesian).lower()}
  enable_benchmark_primitives: false
  max_twist_linear_m_s: 0.15
  max_twist_angular_rad_s: 0.4
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
            }
        ),
        encoding="utf-8",
    )


def rbpodo_args(tmp: Path, config: Path, pgmode: Path, **overrides: object) -> argparse.Namespace:
    values: dict[str, object] = {
        "root": Path("."),
        "server": Path("missing"),
        "server_config": config,
        "arm": "left",
        "controller": "twist_stand",
        "plane": "xy",
        "profile": "circle_15cm_16s",
        "allow_fast_stress": False,
        "diameter_m": None,
        "period_sec": None,
        "repeat": 1,
        "command_rate_hz": 500.0,
        "warmup_sec": 0.0,
        "settle_sec": 0.0,
        "startup_timeout_sec": 1.0,
        "tracking_source": "auto",
        "feedback_kp_pos": 2.0,
        "feedback_kp_ori": 2.0,
        "feedback_max_linear_m_s": None,
        "feedback_max_angular_rad_s": None,
        "feedback_use_current_state_time": False,
        "physical_motion_warning_deg": 0.05,
        "max_state_age_us": 250_000.0,
        "max_allowed_rms_error_m": None,
        "max_allowed_p95_error_m": None,
        "max_allowed_orientation_drift_rad": None,
        "max_allowed_latency_ms": None,
        "pgmode_summary_json": pgmode,
        "set_pgmode_simulation": False,
        "verify_pgmode_simulation": False,
        "pgmode_timeout_sec": 1.0,
        "pgmode_command_port": 5000,
        "artifact_dir": tmp / "artifacts",
        "preflight_only": False,
        "skip_plots": True,
    }
    values.update(overrides)
    return argparse.Namespace(**values)


def rbpodo_pose(x: float, y: float, z: float = 0.0) -> dict[str, object]:
    return {
        "x": x,
        "y": y,
        "z": z,
        "rx": 0.0,
        "ry": 0.0,
        "rz": 0.0,
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
    }


def unavailable_state(host_time_ns: int) -> dict[str, object]:
    pose = rbpodo_pose(0.075, 0.0)
    solve = {
        "status": "unavailable",
        "ik_status": "unavailable",
        "reason": "cartesian_control_unavailable",
        "ik_reason": "cartesian_control_unavailable",
        "attempted": True,
        "success": False,
        "max_command_actual_error_deg_observed": 0.0,
    }
    return {
        "schema_version": 1,
        "host_time_ns": host_time_ns,
        "fault_latched": False,
        "motion_state": "ArmedHold",
        "left": {
            "has_valid_joint_state": True,
            "q_actual_deg": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "q_ref_deg": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "tcp_ref_valid": True,
            "tcp_ref_stand": pose,
            "tcp_actual_valid": True,
            "tcp_actual_stand": pose,
            "cartesian_solve": solve,
            "last_send": {
                "ack_observed": True,
                "controller_acceptance_observed": True,
                "ack_policy": "wait",
                "send_acceptance_semantics": "controller_ack_observed",
            },
        },
    }


class CircleTrackingBenchmarkHelpersTest(unittest.TestCase):
    def test_profile_catalog_includes_middle_speed_metadata(self) -> None:
        self.assertIn("circle_15cm_8s", bench.PROFILE_DEFAULTS)
        middle = bench.benchmark_profile_metadata("circle_15cm_8s")
        self.assertEqual(middle["diameter_m"], 0.15)
        self.assertEqual(middle["period_sec"], 8.0)
        self.assertEqual(middle["purpose"], "middle speed ablation")
        self.assertEqual(middle["stress_level"], "middle")
        self.assertEqual(middle["recommended_controller"], ["twist_stand", "twist_stand_feedback"])
        self.assertEqual(middle["recommended_controllers"], ["twist_stand", "twist_stand_feedback"])
        self.assertAlmostEqual(middle["required_tangential_speed_m_s"], math.pi * 0.15 / 8.0)
        self.assertAlmostEqual(middle["angular_frequency_rad_s"], 2.0 * math.pi / 8.0)

    def test_profile_required_tangential_speeds(self) -> None:
        expected = {
            "safe_5cm_10s": math.pi * 0.05 / 10.0,
            "circle_15cm_16s": math.pi * 0.15 / 16.0,
            "circle_15cm_8s": math.pi * 0.15 / 8.0,
            "gene_15cm_4s": math.pi * 0.15 / 4.0,
        }
        for profile, speed in expected.items():
            with self.subTest(profile=profile):
                self.assertAlmostEqual(
                    bench.benchmark_profile_metadata(profile)["required_tangential_speed_m_s"],
                    speed,
                )

    def test_python_feedback_lane_metadata_stays_separate_from_server_circle(self) -> None:
        row = {
            "schema": rbpodo_bench.SCHEMA,
            "benchmark_category": "rbpodo_controller_simulation",
            "backend": "rbpodo",
            "controller_mode": "pgmode_simulation",
            "controller": "twist_stand_feedback",
            "tracking_source_used": "tcp_ref_stand",
            "physical_motion_expected": False,
        }
        bench.apply_canonical_lane_metadata(row)
        self.assertEqual(row["benchmark_lane"], "rbpodo_python_streaming_feedback")
        self.assertEqual(row["control_loop_location"], "python_benchmark")
        self.assertEqual(row["trajectory_generation_location"], "python_benchmark")
        self.assertEqual(row["feedback_loop_location"], "python_benchmark")
        self.assertEqual(row["tracking_source"], "tcp_ref_stand")

    def test_compare_summary_serializes_profile_speed_and_stress(self) -> None:
        row = compare_bench.comparison_row(
            {
                "artifact_dir": "/tmp/middle",
                "controller": "twist_stand",
                "profile": "circle_15cm_8s",
                "diameter_m": 0.15,
                "period_sec": 8.0,
            }
        )
        self.assertAlmostEqual(row["required_tangential_speed_m_s"], math.pi * 0.15 / 8.0)
        self.assertEqual(row["stress_level"], "middle")
        self.assertIn("required_tangential_speed_m_s", report_bench.REPORT_COLUMNS)
        self.assertIn("stress_level", report_bench.REPORT_COLUMNS)

    def test_feedback_controller_appears_in_help(self) -> None:
        script = Path(__file__).with_name("circle_tracking_benchmark.py")
        completed = subprocess.run(
            [sys.executable, str(script), "--help"],
            check=True,
            text=True,
            stdout=subprocess.PIPE,
        )
        self.assertIn("twist_stand_feedback", completed.stdout)
        self.assertIn("--feedback-kp-pos", completed.stdout)

    def test_result_semantics_without_thresholds_completed(self) -> None:
        args = args_with_thresholds()
        result, reason = bench.benchmark_result(
            bench.thresholds_requested(args),
            bench.threshold_failures(args, {"rms_error_m": 1.0}),
        )
        self.assertEqual(result, "completed")
        self.assertIn("without thresholds", reason)

    def test_result_semantics_thresholds_pass(self) -> None:
        args = args_with_thresholds(max_allowed_rms_error_m=0.01)
        failures = bench.threshold_failures(args, {"rms_error_m": 0.001})
        result, reason = bench.benchmark_result(bench.thresholds_requested(args), failures)
        self.assertEqual(result, "pass")
        self.assertEqual(reason, "thresholds applied and satisfied")

    def test_result_semantics_thresholds_fail(self) -> None:
        args = args_with_thresholds(max_allowed_rms_error_m=0.01)
        failures = bench.threshold_failures(args, {"rms_error_m": 0.1})
        result, reason = bench.benchmark_result(bench.thresholds_requested(args), failures)
        self.assertEqual(result, "fail")
        self.assertEqual(reason, "thresholds applied and failed")
        self.assertTrue(failures)

    def test_radius_gain_warning_for_attenuated_circle(self) -> None:
        reference_radius = 0.075
        fit_radius = 0.016
        radius_gain = fit_radius / reference_radius
        self.assertAlmostEqual(radius_gain, 0.21333333333333335)
        warnings = bench.performance_warnings(
            {
                "reference_radius_m": reference_radius,
                "radius_gain": radius_gain,
                "rms_error_m": 0.02,
                "p95_error_m": 0.04,
                "max_orientation_drift_rad": 0.001,
            }
        )
        self.assertTrue(any("radius_gain" in warning for warning in warnings))

    def test_simulator_dynamics_context_parsing(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            server_config = root / "server.yaml"
            left_config = root / "left.yaml"
            right_config = root / "right.yaml"
            server_config.write_text(
                """
servo:
  rate_hz: 100
cartesian_control:
  max_twist_linear_m_s: 0.15
  max_linear_move_speed_m_s: 0.15
""",
                encoding="utf-8",
            )
            left_config.write_text(
                """
simulator:
  motion_time_constant_sec: 0.04
""",
                encoding="utf-8",
            )
            right_config.write_text(
                """
simulator:
  motion_time_constant_sec: 0.05
""",
                encoding="utf-8",
            )
            args = argparse.Namespace(
                server_config=server_config,
                left_config=left_config,
                right_config=right_config,
                arm="left",
                profile="gene_15cm_4s",
            )
            context = bench.benchmark_context(
                args,
                {"max_twist_linear_m_s": 0.15, "max_linear_move_speed_m_s": 0.15},
            )
        self.assertEqual(context["servo_rate_hz"], 100.0)
        self.assertEqual(context["servo_dt_sec"], 0.01)
        self.assertEqual(context["simulator_motion_time_constant_sec"], 0.04)
        self.assertAlmostEqual(context["simulator_dt_over_tau"], 0.25)
        self.assertTrue(context["stress_profile"])

    def test_feedback_formula_adds_position_error(self) -> None:
        result = bench.compute_feedback_twist_stand(
            feedforward_linear_stand=[0.1, 0.0, 0.0],
            position_error_stand=[0.01, -0.02, 0.0],
            orientation_error_stand=[0.0, 0.0, 0.1],
            kp_pos=2.0,
            kp_ori=3.0,
            max_linear_m_s=1.0,
            max_angular_rad_s=1.0,
        )
        self.assertEqual(result["feedback_twist_stand"][:3], [0.02, -0.04, 0.0])
        self.assertEqual(result["applied_twist_stand"][:3], [0.12000000000000001, -0.04, 0.0])
        self.assertEqual(result["applied_twist_stand"][3:], [0.0, 0.0, 0.30000000000000004])
        self.assertFalse(result["saturated"])

    def test_feedback_clamps_total_command(self) -> None:
        result = bench.compute_feedback_twist_stand(
            feedforward_linear_stand=[1.0, 0.0, 0.0],
            position_error_stand=[1.0, 0.0, 0.0],
            orientation_error_stand=[0.0, 0.0, 1.0],
            kp_pos=1.0,
            kp_ori=1.0,
            max_linear_m_s=0.5,
            max_angular_rad_s=0.2,
        )
        self.assertAlmostEqual(bench.norm(result["applied_twist_stand"][:3]), 0.5)
        self.assertAlmostEqual(bench.norm(result["applied_twist_stand"][3:]), 0.2)
        self.assertTrue(result["saturated"])

    def test_stale_feedback_record_uses_zero_twist(self) -> None:
        row = bench.zero_feedback_record(0.1, "stale feedback state", "stand")
        self.assertEqual(row["applied_twist"], [0.0] * 6)
        self.assertTrue(row["stale_or_invalid_state"])
        metrics = bench.feedback_metrics([row])
        self.assertEqual(metrics["stale_state_feedback_skips"], 1)

    def test_preflight_rejects_real_config_text(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            server_config = root / "server.yaml"
            left_config = root / "left.yaml"
            right_config = root / "right.yaml"
            server_config.write_text(
                """
left_robot:
  backend_type: simulator
  run_mode: real
cartesian_control:
  allow_in_simulation: true
  allow_in_real: false
  max_twist_linear_m_s: 0.03
  max_twist_angular_rad_s: 0.2
  max_linear_move_speed_m_s: 0.05
""",
                encoding="utf-8",
            )
            left_config.write_text("simulator:\n  motion_time_constant_sec: 0.04\n", encoding="utf-8")
            right_config.write_text("simulator:\n  motion_time_constant_sec: 0.04\n", encoding="utf-8")
            args = argparse.Namespace(
                root=root,
                server_config=server_config,
                left_config=left_config,
                right_config=right_config,
                arm="left",
                controller="twist_stand_feedback",
                profile="safe_5cm_10s",
                diameter_m=None,
                period_sec=None,
                repeat=1,
                command_rate_hz=500.0,
                warmup_sec=0.0,
                settle_sec=0.0,
                allow_fast_stress=False,
                feedback_kp_pos=2.0,
                feedback_kp_ori=2.0,
                feedback_max_linear_m_s=None,
                feedback_max_angular_rad_s=None,
            )
            with self.assertRaisesRegex(bench.AcceptanceError, "run_mode: real"):
                bench.preflight(args)

    def test_rbpodo_preflight_requires_allow_in_controller_simulation(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text, EnvGuard():
            tmp = Path(tmp_text)
            config = tmp / "config.yaml"
            pgmode = tmp / "pgmode.json"
            write_rbpodo_config(config, allow_controller_sim_cartesian=False)
            write_pgmode_summary(pgmode)
            with self.assertRaisesRegex(rbpodo_bench.BenchmarkError, "allow_in_controller_simulation"):
                rbpodo_bench.preflight(rbpodo_args(tmp, config, pgmode))

    def test_rbpodo_cartesian_unavailable_blocks_tracking_result(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text:
            tmp = Path(tmp_text)
            artifact_dir = tmp / "artifacts"
            args = rbpodo_args(tmp, tmp / "config.yaml", tmp / "pgmode.json", artifact_dir=artifact_dir)
            args.diameter_m = 0.15
            args.period_sec = 4.0
            traj = bench.Trajectory(
                start=[0.075, 0.0, 0.0],
                axis1=[1.0, 0.0, 0.0],
                axis2=[0.0, 1.0, 0.0],
                radius=0.075,
                period_sec=4.0,
            )
            states = [unavailable_state(1_000_000_000 + index * 10_000_000) for index in range(8)]
            summary = rbpodo_bench.summarize_run(
                args=args,
                config=object(),  # summarize_run does not inspect config.
                preflight_result={
                    "required_tangential_speed_m_s": 0.1178,
                    "pgmode_summary": {"overall_result": "ok"},
                },
                states=states,
                traj=traj,
                q0=[0.0, 0.0, 0.0, 1.0],
                tracking_source="tcp_ref_stand",
                tracking_source_warning=None,
                benchmark_start_ns=1_000_000_000,
                benchmark_end_ns=1_070_000_000,
                command_count=8,
                feedback_rows=[],
                artifact_dir=artifact_dir,
                server_returncode=0,
            )
        self.assertEqual(summary["result"], "blocked")
        self.assertEqual(summary["result_reason"], "cartesian_commands_rejected_by_server")
        self.assertTrue(summary["server_rejected_cartesian"])
        self.assertTrue(summary["command_accepted_but_target_static"])
        self.assertFalse(summary["q_ref_moved"])
        self.assertFalse(summary["tcp_ref_moved"])
        self.assertEqual(summary["cartesian_unavailable_count"], 8)
        self.assertEqual(summary["armed_hold_count"], 8)
        self.assertEqual(
            summary["cartesian_unavailable_reason_counts"],
            {"cartesian_control_unavailable": 8},
        )
        self.assertIn("ServoJ ACKs only show hold-target sends", summary["threshold_failures"][0])

    def test_rbpodo_blocked_result_classifies_as_not_performance(self) -> None:
        row = compare_bench.comparison_row(
            {
                "schema": "robotics_lab.rbpodo_circle_tracking_benchmark.v1",
                "safety_preflight": {
                    "backend": "rbpodo",
                    "controller_simulation_only": True,
                    "pgmode_simulation_confirmed": True,
                    "physical_motion_expected": False,
                },
                "controller": "twist_stand",
                "profile": "circle_15cm_16s",
                "tracking_source_used": "tcp_ref_stand",
                "physical_motion_expected": False,
                "physical_motion_detected": False,
                "result": "blocked",
                "result_reason": "cartesian_commands_rejected_by_server",
                "server_rejected_cartesian": True,
                "cartesian_unavailable_count": 8,
            }
        )
        report_bench.classify_row(row, min_repeats=1)
        self.assertEqual(row["classification"], "rbpodo_controller_sim_cartesian_blocked")
        self.assertEqual(row["real_candidate_policy"], "not_real_ready")
        self.assertIn("allow_in_controller_simulation", row["promotion_notes"])


if __name__ == "__main__":
    unittest.main()
