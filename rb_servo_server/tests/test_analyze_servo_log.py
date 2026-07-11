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
            self.assertEqual(metrics["chunk_diagnostics"]["rows"], 0)
            self.assertEqual(metrics["chunk_diagnostics"]["unique_wire_sequences"], 0)
            self.assertEqual(metrics["delta_twist"]["left"]["rows"], 0)
            self.assertEqual(metrics["delta_twist"]["right"]["rows"], 0)

    def test_optional_chunk_and_delta_twist_diagnostics_are_summarized(self) -> None:
        shared_fields = list(analyze_servo_log.CHUNK_DIAGNOSTIC_INTEGER_COLUMNS) + list(
            analyze_servo_log.CHUNK_DIAGNOSTIC_FLOAT_COLUMNS
        )
        arm_fields = [
            "left_follower_controller",
            "left_delta_twist_frame_rows",
            "left_delta_twist_normal_budget",
            "left_delta_twist_total_budget",
            "left_delta_twist_steps_remaining",
            "left_delta_twist_clamp_mask",
            *[
                f"left_delta_twist_accel_cmd_{suffix}"
                for suffix in analyze_servo_log.DELTA_TWIST_ACCEL_COMMAND_SUFFIXES
            ],
        ]
        rows: list[dict[str, object]] = []
        for index in range(3):
            row: dict[str, object] = {
                "chunk_frame_wire_seq": 10 + min(index, 1),
                "chunk_frame_recv_seq": 20 + min(index, 1),
                "chunk_frame_horizon": 16,
                "chunk_frame_execute_steps": 12,
                "chunk_frame_runway_steps": 4,
                "chunk_inference_seq": 30 + min(index, 1),
                "chunk_inference_stall_count": index,
                "chunk_camera_bundle_seq": 40 + index,
                "chunk_camera_left_frame_number": 50 + index,
                "chunk_camera_right_frame_number": 60 + index,
                "chunk_frame_policy_dt_sec": 0.0668,
                "chunk_frame_age_ms": 100.0 + 100.0 * index,
                "chunk_frame_interarrival_ms": 800.0 + 10.0 * index,
                "chunk_inference_queue_wait_ms": 1.0 + index,
                "chunk_inference_latency_ms": 700.0 + 10.0 * index,
                "chunk_inference_ready_wait_ms": 2.0 + index,
                "chunk_inference_period_ms": 810.0 + 10.0 * index,
                "chunk_inference_period_jitter_ms": 5.0 + index,
                "chunk_camera_bundle_age_ms": 10.0 + index,
                "chunk_camera_max_skew_ms": 3.0 + index,
                "chunk_camera_left_frame_age_ms": 11.0 + index,
                "chunk_camera_right_frame_age_ms": 12.0 + index,
                "chunk_camera_left_focus_score": 100.0 + index,
                "chunk_camera_right_focus_score": 200.0 + index,
                "left_follower_controller": "delta_twist",
                "left_delta_twist_frame_rows": 16,
                "left_delta_twist_normal_budget": 6,
                "left_delta_twist_total_budget": 8,
                "left_delta_twist_steps_remaining": 8 - index,
                "left_delta_twist_clamp_mask": (1 << 0) | (1 << 2) if index == 0 else (
                    (1 << 2) | (1 << 13) if index == 1 else 0
                ),
            }
            for axis, suffix in enumerate(analyze_servo_log.DELTA_TWIST_ACCEL_COMMAND_SUFFIXES):
                row[f"left_delta_twist_accel_cmd_{suffix}"] = float(index - axis)
            rows.append(row)

        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "chunk-diagnostics.csv"
            self.make_log(
                path,
                rows=len(rows),
                extra_fieldnames=shared_fields + arm_fields,
                row_overrides=rows,
            )

            metrics = analyze_servo_log.analyze_csv(path)
            report = analyze_servo_log.format_report(
                metrics,
                analyze_servo_log.BUDGETS["rbsim-local100"],
                failures=[],
            )

            chunk = metrics["chunk_diagnostics"]
            self.assertEqual(chunk["rows"], 3)
            self.assertEqual(chunk["unique_wire_sequences"], 2)
            self.assertEqual(chunk["series"]["chunk_frame_age_ms"]["p50"], 200.0)
            self.assertEqual(chunk["series"]["chunk_inference_latency_ms"]["p99"], 720.0)
            self.assertEqual(chunk["series"]["chunk_camera_max_skew_ms"]["p90"], 5.0)

            left = metrics["delta_twist"]["left"]
            self.assertEqual(left["frame_rows"]["p50"], 16)
            self.assertEqual(left["normal_budget"]["p50"], 6)
            self.assertEqual(left["total_budget"]["p50"], 8)
            self.assertEqual(left["steps_remaining"]["p50"], 7)
            self.assertEqual(
                left["clamp_mask_counts"],
                {"lead_angular": 1, "pending_linear": 1, "xi_ref_velocity_linear": 2},
            )
            self.assertEqual(left["accel_command"]["x_m_s2"]["p50"], 1.0)
            self.assertEqual(left["accel_command"]["rz_rad_s2"]["p50"], -4.0)
            self.assertIn("chunk_diagnostics: rows=3 unique_wire_sequences=2", report)
            self.assertIn("chunk_inference_latency_ms: count=3", report)
            self.assertIn("left clamp_mask_counts:", report)
            self.assertIn("left steps_remaining: count=3", report)
            self.assertIn("left accel_cmd_rz_rad_s2: count=3", report)

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
            self.assertEqual(left["pending_residual_linear_norm_m"]["p50"], 0.2)
            self.assertEqual(left["pending_residual_linear_norm_m"]["p90"], 0.4)
            self.assertEqual(left["xi_ref_angular_norm_rad_s"]["p99"], 4.1)
            self.assertEqual(left["commanded_angular_velocity_rad_s"]["p99"], 4.0)
            self.assertEqual(left["requested_yaw_delta_rad"]["p90"], 0.4)
            self.assertEqual(left["realized_yaw_delta_rad"]["p50"], -0.12)
            self.assertEqual(left["yaw_realized_ratio"]["p90"], 1.0)
            self.assertEqual(left["linear_realized_ratio"]["p50"], 0.5)
            self.assertEqual(left["yaw_sign_match_percent"], 50.0)
            self.assertGreater(left["stage_path_to_net_ratio"], 100.0)
            self.assertTrue(any("path/net" in warning for warning in left["warnings"]))
            self.assertTrue(any("yaw sign" in warning for warning in left["warnings"]))
            self.assertIn("safety_verdict_counts: JointLimitClamped=1, Ok=3", report)
            self.assertIn("step_kind_counts={'normal': 2, 'reserve': 1, 'ringdown': 1}", report)
            self.assertIn("feedback_source_counts: {'2': 4}", report)
            self.assertIn("yaw_sign_match_percent: 50.000", report)
            self.assertIn("left xi_cmd_angular_norm_rad_s: count=4", report)


if __name__ == "__main__":
    unittest.main()
