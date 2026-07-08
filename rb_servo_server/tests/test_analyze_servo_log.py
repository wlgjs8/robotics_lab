import csv
import tempfile
import unittest
from pathlib import Path
import sys
from typing import Sequence


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import analyze_servo_log


class AnalyzeServoLogTest(unittest.TestCase):
    def make_log(
        self,
        path: Path,
        *,
        rows: int = 200,
        period_ms: float = 10.0,
        extra_fieldnames: Sequence[str] | None = None,
        row_overrides: Sequence[dict[str, object]] | None = None,
        **overrides: object,
    ) -> None:
        fieldnames = (
            analyze_servo_log.BASE_REQUIRED_COLUMNS
            + analyze_servo_log.Q_REQUIRED_COLUMNS
            + analyze_servo_log.TIMESTAMP_COLUMNS
            + list(extra_fieldnames or [])
        )
        start_ns = 1_000_000_000
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            for index in range(rows):
                loop_start_ns = start_ns + int(index * period_ms * 1_000_000)
                row: dict[str, object] = {
                    "period_ms": period_ms,
                    "jitter_ms": 0.1,
                    "send_skew_us": 10.0,
                    "left_send_start_ns": loop_start_ns + 100_000,
                    "left_send_end_ns": loop_start_ns + 150_000,
                    "right_send_start_ns": loop_start_ns + 110_000,
                    "right_send_end_ns": loop_start_ns + 165_000,
                    "left_send_duration_us": 50.0,
                    "right_send_duration_us": 55.0,
                    "logger_dropped_samples": 0,
                    "left_send_ok": "true",
                    "right_send_ok": "true",
                    "loop_start_time_ns": loop_start_ns,
                    "loop_end_time_ns": loop_start_ns + int(period_ms * 1_000_000),
                }
                for arm in ("left", "right"):
                    for joint in range(6):
                        row[f"{arm}_q_actual_{joint}"] = float(joint)
                        row[f"{arm}_q_sent_{joint}"] = float(joint)
                row.update(overrides)
                if row_overrides is not None:
                    row.update(row_overrides[index])
                writer.writerow(row)

    def profile_failures(self, path: Path, profile: str = "rbsim-local100") -> list[str]:
        metrics = analyze_servo_log.analyze_csv(path)
        return analyze_servo_log.check_budget(metrics, analyze_servo_log.BUDGETS[profile])

    def test_rbsim_profiles_pass_clean_simulator_logs(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            local_log = tmp_path / "local.csv"
            long_log = tmp_path / "long.csv"
            self.make_log(local_log, rows=200)
            self.make_log(long_log, rows=3000)

            self.assertEqual(self.profile_failures(local_log, "rbsim-local100"), [])
            self.assertEqual(self.profile_failures(long_log, "rbsim100"), [])

    def test_missing_send_columns_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / "source.csv"
            broken = Path(tmp) / "missing-send-column.csv"
            self.make_log(source)

            with source.open(newline="", encoding="utf-8") as input_handle:
                reader = csv.DictReader(input_handle)
                fieldnames = [name for name in (reader.fieldnames or []) if name != "left_send_start_ns"]
                with broken.open("w", newline="", encoding="utf-8") as output_handle:
                    writer = csv.DictWriter(output_handle, fieldnames=fieldnames)
                    writer.writeheader()
                    for row in reader:
                        row.pop("left_send_start_ns", None)
                        writer.writerow(row)

            with self.assertRaisesRegex(analyze_servo_log.AnalysisError, "missing required CSV columns: left_send_start_ns"):
                analyze_servo_log.analyze_csv(broken)

    def test_malformed_send_timestamps_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "bad-send-timestamp.csv"
            self.make_log(path, left_send_start_ns=200, left_send_end_ns=100)

            with self.assertRaisesRegex(analyze_servo_log.AnalysisError, "left_send_end_ns is before left_send_start_ns"):
                analyze_servo_log.analyze_csv(path)

    def test_rbsim_local_profile_rejects_bad_timing_and_log_health(self) -> None:
        cases = (
            ({"logger_dropped_samples": 1}, "logger_dropped_samples_max"),
            ({"period_ms": 20.0}, "period_ms.mean"),
            ({"jitter_ms": 20.0}, "jitter_ms.p95"),
            ({"send_skew_us": 4000.0}, "send_skew_us.p95"),
            ({"left_send_ok": "false"}, "send_failures.total_arm_failures"),
            ({"left_q_actual_0": 3.0, "left_q_sent_0": 0.0}, "tracking_error_deg.max"),
        )
        with tempfile.TemporaryDirectory() as tmp:
            for index, (overrides, expected_fragment) in enumerate(cases):
                path = Path(tmp) / f"failure-{index}.csv"
                self.make_log(path, **overrides)

                failures = self.profile_failures(path)

                self.assertTrue(
                    any(expected_fragment in failure for failure in failures),
                    f"{expected_fragment!r} not found in {failures!r}",
                )

    def test_delta_twist_summary_tolerates_missing_optional_columns(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "old-log.csv"
            self.make_log(path, rows=4)

            metrics = analyze_servo_log.analyze_csv(path)

            self.assertEqual(metrics["safety_verdict_counts"], {})
            self.assertEqual(metrics["delta_twist"]["left"]["rows"], 0)
            self.assertEqual(metrics["delta_twist"]["right"]["rows"], 0)
            self.assertEqual(metrics["near_floor_pick_analysis"]["left"]["near_floor_ticks"], 0)
            self.assertEqual(metrics["near_floor_pick_analysis"]["right"]["near_floor_ticks"], 0)

    def test_delta_twist_summary_uses_optional_columns(self) -> None:
        extra = [
            "safety_verdict",
            "left_follower_controller",
            "left_follower_active",
            "left_follower_stall",
            "left_follower_step",
            "left_delta_twist_step_kind",
            "left_delta_twist_pending_linear_norm_m",
            "left_delta_twist_pending_angular_norm_rad",
            "left_delta_twist_xi_ref_linear_norm_m_s",
            "left_delta_twist_xi_ref_angular_norm_rad_s",
            "left_delta_twist_xi_cmd_linear_norm_m_s",
            "left_delta_twist_xi_cmd_angular_norm_rad_s",
            "left_delta_twist_step_yaw_rad",
            "left_delta_twist_realized_yaw_rad",
            "left_delta_twist_realized_linear_ratio",
            "left_delta_twist_realized_angular_ratio",
            "left_delta_twist_realized_yaw_ratio",
            "left_delta_twist_saturated",
            "left_delta_twist_feedback_source",
            "left_delta_twist_pending_clamped",
            "left_delta_twist_min_time_to_go_used",
            "left_delta_twist_xi_ref_clamped_norm",
            "left_delta_twist_xi_cmd_clamped_norm",
            "left_surface_mode",
            "left_surface_active",
            "left_surface_close_soon",
            "left_surface_hull_scaled",
            "left_surface_min_tip_dist_m",
            "left_surface_down_scale",
            "left_surface_tangent_scale",
            "left_surface_hull_alpha",
            "left_surface_raw_linear_norm_m",
            "left_surface_projected_linear_norm_m",
            "left_surface_discarded_linear_norm_m",
            "left_surface_raw_angular_norm_rad",
            "left_surface_projected_angular_norm_rad",
            "left_surface_discarded_angular_norm_rad",
            "left_grasp_phase",
            "left_grasp_commit_active",
            "left_grasp_close_soon",
            "left_grasp_ready",
            "left_grasp_sync_wait_sec",
            "left_grasp_closing_hold_elapsed_sec",
            "left_grasp_lift_elapsed_sec",
            "left_grasp_lift_progress",
            "left_grasp_gripper_override_active",
            "left_grasp_policy_delta_dropped",
            "left_grasp_resume_wait_fresh_chunk",
            "left_stage_tcp_target_stand_x_m",
            "left_stage_tcp_target_stand_y_m",
            "left_stage_tcp_target_stand_z_m",
            "left_safety_accel_clamped",
        ]
        rows = [
            {
                "safety_verdict": "Ok",
                "left_follower_controller": "delta_twist",
                "left_follower_active": "1",
                "left_follower_stall": "0",
                "left_follower_step": "0",
                "left_delta_twist_step_kind": "1",
                "left_delta_twist_pending_linear_norm_m": "0.1",
                "left_delta_twist_pending_angular_norm_rad": "0.01",
                "left_delta_twist_xi_ref_linear_norm_m_s": "1.5",
                "left_delta_twist_xi_ref_angular_norm_rad_s": "1.1",
                "left_delta_twist_xi_cmd_linear_norm_m_s": "0.5",
                "left_delta_twist_xi_cmd_angular_norm_rad_s": "1.0",
                "left_delta_twist_step_yaw_rad": "0.10",
                "left_delta_twist_realized_yaw_rad": "0.05",
                "left_delta_twist_realized_linear_ratio": "0.4",
                "left_delta_twist_realized_angular_ratio": "0.45",
                "left_delta_twist_realized_yaw_ratio": "0.5",
                "left_delta_twist_saturated": "0",
                "left_delta_twist_feedback_source": "2",
                "left_delta_twist_pending_clamped": "0",
                "left_delta_twist_min_time_to_go_used": "0",
                "left_delta_twist_xi_ref_clamped_norm": "0",
                "left_delta_twist_xi_cmd_clamped_norm": "0",
                "left_surface_mode": "0",
                "left_surface_active": "0",
                "left_surface_close_soon": "0",
                "left_surface_hull_scaled": "0",
                "left_surface_min_tip_dist_m": "0.030",
                "left_surface_down_scale": "1.0",
                "left_surface_tangent_scale": "1.0",
                "left_surface_hull_alpha": "1.0",
                "left_surface_raw_linear_norm_m": "0.002",
                "left_surface_projected_linear_norm_m": "0.002",
                "left_surface_discarded_linear_norm_m": "0.0",
                "left_surface_raw_angular_norm_rad": "0.010",
                "left_surface_projected_angular_norm_rad": "0.010",
                "left_surface_discarded_angular_norm_rad": "0.0",
                "left_grasp_phase": "0",
                "left_grasp_commit_active": "0",
                "left_grasp_close_soon": "0",
                "left_grasp_ready": "0",
                "left_grasp_sync_wait_sec": "0.0",
                "left_grasp_closing_hold_elapsed_sec": "0.0",
                "left_grasp_lift_elapsed_sec": "0.0",
                "left_grasp_lift_progress": "0.0",
                "left_grasp_gripper_override_active": "0",
                "left_grasp_policy_delta_dropped": "0",
                "left_grasp_resume_wait_fresh_chunk": "0",
                "left_stage_tcp_target_stand_x_m": "0.0",
                "left_stage_tcp_target_stand_y_m": "0.0",
                "left_stage_tcp_target_stand_z_m": "0.0",
                "left_safety_accel_clamped": "0",
            },
            {
                "safety_verdict": "Ok",
                "left_follower_controller": "delta_twist",
                "left_follower_active": "1",
                "left_follower_stall": "0",
                "left_follower_step": "1",
                "left_delta_twist_step_kind": "1",
                "left_delta_twist_pending_linear_norm_m": "0.2",
                "left_delta_twist_pending_angular_norm_rad": "0.02",
                "left_delta_twist_xi_ref_linear_norm_m_s": "1.6",
                "left_delta_twist_xi_ref_angular_norm_rad_s": "2.1",
                "left_delta_twist_xi_cmd_linear_norm_m_s": "0.6",
                "left_delta_twist_xi_cmd_angular_norm_rad_s": "2.0",
                "left_delta_twist_step_yaw_rad": "0.20",
                "left_delta_twist_realized_yaw_rad": "-0.12",
                "left_delta_twist_realized_linear_ratio": "0.5",
                "left_delta_twist_realized_angular_ratio": "0.55",
                "left_delta_twist_realized_yaw_ratio": "0.6",
                "left_delta_twist_saturated": "0",
                "left_delta_twist_feedback_source": "2",
                "left_delta_twist_pending_clamped": "1",
                "left_delta_twist_min_time_to_go_used": "1",
                "left_delta_twist_xi_ref_clamped_norm": "1",
                "left_delta_twist_xi_cmd_clamped_norm": "0",
                "left_surface_mode": "1",
                "left_surface_active": "1",
                "left_surface_close_soon": "0",
                "left_surface_hull_scaled": "0",
                "left_surface_min_tip_dist_m": "0.010",
                "left_surface_down_scale": "0.5",
                "left_surface_tangent_scale": "0.6",
                "left_surface_hull_alpha": "1.0",
                "left_surface_raw_linear_norm_m": "0.003",
                "left_surface_projected_linear_norm_m": "0.0015",
                "left_surface_discarded_linear_norm_m": "0.0015",
                "left_surface_raw_angular_norm_rad": "0.020",
                "left_surface_projected_angular_norm_rad": "0.015",
                "left_surface_discarded_angular_norm_rad": "0.005",
                "left_grasp_phase": "2",
                "left_grasp_commit_active": "1",
                "left_grasp_close_soon": "1",
                "left_grasp_ready": "1",
                "left_grasp_sync_wait_sec": "0.02",
                "left_grasp_closing_hold_elapsed_sec": "0.0",
                "left_grasp_lift_elapsed_sec": "0.0",
                "left_grasp_lift_progress": "0.0",
                "left_grasp_gripper_override_active": "0",
                "left_grasp_policy_delta_dropped": "1",
                "left_grasp_resume_wait_fresh_chunk": "0",
                "left_stage_tcp_target_stand_x_m": "1.0",
                "left_stage_tcp_target_stand_y_m": "0.0",
                "left_stage_tcp_target_stand_z_m": "0.0",
                "left_safety_accel_clamped": "0",
            },
            {
                "safety_verdict": "JointLimitClamped",
                "left_follower_controller": "delta_twist",
                "left_follower_active": "1",
                "left_follower_stall": "1",
                "left_follower_step": "1",
                "left_delta_twist_step_kind": "2",
                "left_delta_twist_pending_linear_norm_m": "0.3",
                "left_delta_twist_pending_angular_norm_rad": "0.03",
                "left_delta_twist_xi_ref_linear_norm_m_s": "1.7",
                "left_delta_twist_xi_ref_angular_norm_rad_s": "3.1",
                "left_delta_twist_xi_cmd_linear_norm_m_s": "0.7",
                "left_delta_twist_xi_cmd_angular_norm_rad_s": "3.0",
                "left_delta_twist_step_yaw_rad": "0.30",
                "left_delta_twist_realized_yaw_rad": "-0.21",
                "left_delta_twist_realized_linear_ratio": "0.6",
                "left_delta_twist_realized_angular_ratio": "0.65",
                "left_delta_twist_realized_yaw_ratio": "0.7",
                "left_delta_twist_saturated": "1",
                "left_delta_twist_feedback_source": "2",
                "left_delta_twist_pending_clamped": "1",
                "left_delta_twist_min_time_to_go_used": "1",
                "left_delta_twist_xi_ref_clamped_norm": "1",
                "left_delta_twist_xi_cmd_clamped_norm": "1",
                "left_surface_mode": "2",
                "left_surface_active": "1",
                "left_surface_close_soon": "1",
                "left_surface_hull_scaled": "0",
                "left_surface_min_tip_dist_m": "0.006",
                "left_surface_down_scale": "0.0",
                "left_surface_tangent_scale": "0.0",
                "left_surface_hull_alpha": "1.0",
                "left_surface_raw_linear_norm_m": "0.004",
                "left_surface_projected_linear_norm_m": "0.000",
                "left_surface_discarded_linear_norm_m": "0.004",
                "left_surface_raw_angular_norm_rad": "0.030",
                "left_surface_projected_angular_norm_rad": "0.010",
                "left_surface_discarded_angular_norm_rad": "0.020",
                "left_grasp_phase": "3",
                "left_grasp_commit_active": "1",
                "left_grasp_close_soon": "1",
                "left_grasp_ready": "1",
                "left_grasp_sync_wait_sec": "0.04",
                "left_grasp_closing_hold_elapsed_sec": "0.10",
                "left_grasp_lift_elapsed_sec": "0.0",
                "left_grasp_lift_progress": "0.0",
                "left_grasp_gripper_override_active": "1",
                "left_grasp_policy_delta_dropped": "1",
                "left_grasp_resume_wait_fresh_chunk": "0",
                "left_stage_tcp_target_stand_x_m": "0.0",
                "left_stage_tcp_target_stand_y_m": "0.0",
                "left_stage_tcp_target_stand_z_m": "0.0",
                "left_safety_accel_clamped": "1",
            },
            {
                "safety_verdict": "Ok",
                "left_follower_controller": "delta_twist",
                "left_follower_active": "0",
                "left_follower_stall": "0",
                "left_follower_step": "2",
                "left_delta_twist_step_kind": "4",
                "left_delta_twist_pending_linear_norm_m": "0.4",
                "left_delta_twist_pending_angular_norm_rad": "0.04",
                "left_delta_twist_xi_ref_linear_norm_m_s": "1.8",
                "left_delta_twist_xi_ref_angular_norm_rad_s": "4.1",
                "left_delta_twist_xi_cmd_linear_norm_m_s": "0.8",
                "left_delta_twist_xi_cmd_angular_norm_rad_s": "4.0",
                "left_delta_twist_step_yaw_rad": "0.40",
                "left_delta_twist_realized_yaw_rad": "0.40",
                "left_delta_twist_realized_linear_ratio": "1.0",
                "left_delta_twist_realized_angular_ratio": "1.0",
                "left_delta_twist_realized_yaw_ratio": "1.0",
                "left_delta_twist_saturated": "0",
                "left_delta_twist_feedback_source": "2",
                "left_delta_twist_pending_clamped": "0",
                "left_delta_twist_min_time_to_go_used": "0",
                "left_delta_twist_xi_ref_clamped_norm": "0",
                "left_delta_twist_xi_cmd_clamped_norm": "0",
                "left_surface_mode": "3",
                "left_surface_active": "1",
                "left_surface_close_soon": "0",
                "left_surface_hull_scaled": "1",
                "left_surface_min_tip_dist_m": "0.004",
                "left_surface_down_scale": "1.0",
                "left_surface_tangent_scale": "1.0",
                "left_surface_hull_alpha": "0.5",
                "left_surface_raw_linear_norm_m": "0.005",
                "left_surface_projected_linear_norm_m": "0.0025",
                "left_surface_discarded_linear_norm_m": "0.0025",
                "left_surface_raw_angular_norm_rad": "0.040",
                "left_surface_projected_angular_norm_rad": "0.020",
                "left_surface_discarded_angular_norm_rad": "0.020",
                "left_grasp_phase": "4",
                "left_grasp_commit_active": "1",
                "left_grasp_close_soon": "0",
                "left_grasp_ready": "1",
                "left_grasp_sync_wait_sec": "0.04",
                "left_grasp_closing_hold_elapsed_sec": "0.20",
                "left_grasp_lift_elapsed_sec": "0.10",
                "left_grasp_lift_progress": "0.30",
                "left_grasp_gripper_override_active": "1",
                "left_grasp_policy_delta_dropped": "1",
                "left_grasp_resume_wait_fresh_chunk": "0",
                "left_stage_tcp_target_stand_x_m": "0.01",
                "left_stage_tcp_target_stand_y_m": "0.0",
                "left_stage_tcp_target_stand_z_m": "0.0",
                "left_safety_accel_clamped": "0",
            },
        ]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "delta-twist-log.csv"
            self.make_log(path, rows=len(rows), extra_fieldnames=extra, row_overrides=rows)

            metrics = analyze_servo_log.analyze_csv(path)
            report = analyze_servo_log.format_report(
                metrics,
                analyze_servo_log.BUDGETS["rbsim-local100"],
                failures=[],
            )

            left = metrics["delta_twist"]["left"]
            self.assertEqual(metrics["safety_verdict_counts"], {"JointLimitClamped": 1, "Ok": 3})
            self.assertEqual(left["rows"], 4)
            self.assertEqual(left["active_ticks"], 3)
            self.assertEqual(left["stall_ticks"], 1)
            self.assertEqual(left["saturated_ticks"], 1)
            self.assertEqual(left["pending_clamped_ticks"], 2)
            self.assertEqual(left["xi_ref_clamped_ticks"], 2)
            self.assertEqual(left["xi_cmd_clamped_ticks"], 1)
            self.assertEqual(left["min_time_to_go_ticks"], 2)
            self.assertEqual(left["feedback_source_counts"], {"2": 4})
            self.assertEqual(left["follower_step_distribution"], {"0": 1, "1": 2, "2": 1})
            self.assertEqual(left["step_kind_counts"], {"normal": 2, "reserve": 1, "ringdown": 1})
            self.assertEqual(left["accel_clamp_counts"], {"safety_accel_clamped": 1})
            self.assertEqual(left["surface_active_ticks"], 3)
            self.assertEqual(left["surface_close_soon_ticks"], 1)
            self.assertEqual(left["surface_hull_scaled_ticks"], 1)
            self.assertEqual(left["surface_mode_counts"], {"0": 1, "1": 1, "2": 1, "3": 1})
            self.assertEqual(left["grasp_commit_active_ticks"], 3)
            self.assertEqual(left["grasp_close_soon_ticks"], 2)
            self.assertEqual(left["grasp_ready_ticks"], 3)
            self.assertEqual(left["grasp_gripper_override_ticks"], 2)
            self.assertEqual(left["grasp_policy_delta_dropped_ticks"], 3)
            self.assertEqual(left["grasp_resume_wait_ticks"], 0)
            self.assertEqual(
                left["grasp_phase_counts"],
                {"closing_hold": 1, "lift_out": 1, "normal": 1, "pregrasp_commit": 1},
            )
            self.assertEqual(left["pending_residual_linear_norm_m"]["p50"], 0.2)
            self.assertEqual(left["pending_residual_linear_norm_m"]["p90"], 0.4)
            self.assertEqual(left["xi_ref_angular_norm_rad_s"]["p99"], 4.1)
            self.assertEqual(left["commanded_angular_velocity_rad_s"]["p99"], 4.0)
            self.assertEqual(left["requested_yaw_delta_rad"]["p90"], 0.4)
            self.assertEqual(left["realized_yaw_delta_rad"]["p50"], -0.12)
            self.assertEqual(left["yaw_realized_ratio"]["p90"], 1.0)
            self.assertEqual(left["linear_realized_ratio"]["p50"], 0.5)
            self.assertEqual(left["surface_discarded_linear_norm_m"]["p90"], 0.004)
            self.assertEqual(left["surface_tangent_scale"]["p50"], 0.6)
            self.assertEqual(left["grasp_sync_wait_sec"]["p90"], 0.04)
            self.assertEqual(left["grasp_closing_hold_elapsed_sec"]["p99"], 0.2)
            self.assertEqual(left["grasp_lift_progress"]["p99"], 0.3)
            self.assertEqual(left["yaw_sign_match_percent"], 50.0)
            self.assertGreater(left["stage_path_to_net_ratio"], 100.0)
            self.assertTrue(any("path/net" in warning for warning in left["warnings"]))
            self.assertTrue(any("yaw sign" in warning for warning in left["warnings"]))
            self.assertIn("safety_verdict_counts: JointLimitClamped=1, Ok=3", report)
            self.assertIn("step_kind_counts={'normal': 2, 'reserve': 1, 'ringdown': 1}", report)
            self.assertIn("feedback_source_counts: {'2': 4}", report)
            self.assertIn("surface: active_ticks=3 close_soon_ticks=1 hull_scaled_ticks=1", report)
            self.assertIn("grasp: commit_active_ticks=3 close_soon_ticks=2", report)
            self.assertIn("phase_counts={'closing_hold': 1, 'lift_out': 1, 'normal': 1, 'pregrasp_commit': 1}", report)
            self.assertIn("yaw_sign_match_percent: 50.000", report)
            self.assertIn("left xi_cmd_angular_norm_rad_s: count=4", report)

    def test_near_floor_analysis_warns_on_sliding_during_close(self) -> None:
        extra = [
            "safety_verdict",
            "left_follower_controller",
            "left_surface_min_tip_dist_m",
            "left_surface_active",
            "left_surface_close_soon",
            "left_surface_raw_dx_m",
            "left_surface_raw_dy_m",
            "left_surface_raw_dz_m",
            "left_surface_projected_dx_m",
            "left_surface_projected_dy_m",
            "left_surface_projected_dz_m",
            "left_surface_discarded_dx_m",
            "left_surface_discarded_dy_m",
            "left_surface_discarded_dz_m",
            "left_surface_raw_linear_norm_m",
            "left_surface_projected_linear_norm_m",
            "left_surface_discarded_linear_norm_m",
            "left_surface_down_scale",
            "left_surface_tangent_scale",
            "left_grasp_phase",
            "left_grasp_close_soon",
            "left_gripper_close_soon",
            "left_gripper_closing_hold_active",
        ]
        rows = [
            {
                "safety_verdict": "Ok",
                "left_follower_controller": "delta_twist",
                "left_surface_min_tip_dist_m": "0.006",
                "left_surface_active": "1",
                "left_surface_close_soon": "1",
                "left_surface_raw_dx_m": "0.0020",
                "left_surface_raw_dy_m": "0.0000",
                "left_surface_raw_dz_m": "-0.0010",
                "left_surface_projected_dx_m": "0.0010",
                "left_surface_projected_dy_m": "0.0000",
                "left_surface_projected_dz_m": "0.0000",
                "left_surface_discarded_dx_m": "0.0010",
                "left_surface_discarded_dy_m": "0.0000",
                "left_surface_discarded_dz_m": "-0.0010",
                "left_surface_raw_linear_norm_m": "0.002236",
                "left_surface_projected_linear_norm_m": "0.0010",
                "left_surface_discarded_linear_norm_m": "0.001414",
                "left_surface_down_scale": "0.0",
                "left_surface_tangent_scale": "0.5",
                "left_grasp_phase": "3",
                "left_grasp_close_soon": "1",
                "left_gripper_close_soon": "1",
                "left_gripper_closing_hold_active": "1",
            },
        ]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "near-floor-sliding.csv"
            self.make_log(path, rows=len(rows), extra_fieldnames=extra, row_overrides=rows)

            metrics = analyze_servo_log.analyze_csv(path)
            report = analyze_servo_log.format_report(
                metrics,
                analyze_servo_log.BUDGETS["rbsim-local100"],
                failures=[],
            )

            left = metrics["near_floor_pick_analysis"]["left"]
            self.assertEqual(left["near_floor_ticks"], 1)
            self.assertEqual(left["sliding_risk_ticks"], 1)
            self.assertEqual(left["closing_translation_ticks"], 1)
            self.assertIn("Sliding risk: arm translation during close", left["warnings"])
            self.assertIn("Near-floor pick analysis:", report)
            self.assertIn("Sliding risk: arm translation during close", report)

    def test_near_floor_analysis_warns_on_step_consumed_while_blocked(self) -> None:
        extra = [
            "safety_verdict",
            "left_follower_controller",
            "left_follower_step",
            "left_delta_twist_blocked",
            "left_delta_twist_step_consumed_this_tick",
            "left_surface_min_tip_dist_m",
            "left_surface_active",
            "left_surface_close_soon",
            "left_surface_projected_linear_norm_m",
        ]
        rows = [
            {
                "safety_verdict": "FloorViolation",
                "left_follower_controller": "delta_twist",
                "left_follower_step": "4",
                "left_delta_twist_blocked": "1",
                "left_delta_twist_step_consumed_this_tick": "1",
                "left_surface_min_tip_dist_m": "0.004",
                "left_surface_active": "1",
                "left_surface_close_soon": "1",
                "left_surface_projected_linear_norm_m": "0.0",
            },
        ]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "near-floor-blocked.csv"
            self.make_log(path, rows=len(rows), extra_fieldnames=extra, row_overrides=rows)

            metrics = analyze_servo_log.analyze_csv(path)
            report = analyze_servo_log.format_report(
                metrics,
                analyze_servo_log.BUDGETS["rbsim-local100"],
                failures=[],
            )

            left = metrics["near_floor_pick_analysis"]["left"]
            self.assertEqual(left["blocked_ticks"], 1)
            self.assertEqual(left["blocked_step_consumed_ticks"], 1)
            self.assertEqual(left["safety_verdict_counts"], {"FloorViolation": 1})
            self.assertIn("Safety phase mismatch: step consumed while blocked", left["warnings"])
            self.assertIn("Safety phase mismatch: step consumed while blocked", report)


if __name__ == "__main__":
    unittest.main()
