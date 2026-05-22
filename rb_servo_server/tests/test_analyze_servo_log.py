import csv
import tempfile
import unittest
from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import analyze_servo_log


class AnalyzeServoLogTest(unittest.TestCase):
    def make_log(self, path: Path, *, rows: int = 200, period_ms: float = 10.0, **overrides: object) -> None:
        fieldnames = (
            analyze_servo_log.BASE_REQUIRED_COLUMNS
            + analyze_servo_log.Q_REQUIRED_COLUMNS
            + analyze_servo_log.TIMESTAMP_COLUMNS
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


if __name__ == "__main__":
    unittest.main()
