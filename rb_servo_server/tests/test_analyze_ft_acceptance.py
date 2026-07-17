import csv
import sys
import tempfile
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import analyze_ft_acceptance


class AnalyzeFtAcceptanceTest(unittest.TestCase):
    def write_log(
        self,
        path: Path,
        *,
        measured: float | None = None,
        force: float = 1.0,
    ) -> None:
        if measured is None:
            measured = -force
        fields = ["loop_start_time_ns"]
        for arm in ("left", "right"):
            fields += analyze_ft_acceptance._required_columns(arm)[1:]
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            for index in range(20):
                row = {"loop_start_time_ns": 1_000_000_000 + index * 100_000_000}
                for arm in ("left", "right"):
                    row.update({
                        f"{arm}_ft_healthy": 1,
                        f"{arm}_ft_stale": 0,
                        f"{arm}_ft_freshness_value": index + 1,
                        f"{arm}_ft_freshness_advanced": 1,
                        f"{arm}_ft_reason": "ok",
                        f"{arm}_ft_auto_tare_enabled": 1,
                        f"{arm}_ft_tare_valid": 1,
                        f"{arm}_ft_tare_state": "accepted",
                        f"{arm}_force_control_measured_normal_force_n": measured,
                        f"{arm}_force_control_fast_normal_force_n": -force,
                        f"{arm}_force_control_fast_force_norm_n": abs(force),
                        f"{arm}_force_control_fast_torque_norm_nm": 0.0,
                        f"{arm}_force_control_normal_stand_x": 0,
                        f"{arm}_force_control_normal_stand_y": 0,
                        f"{arm}_force_control_normal_stand_z": 1,
                        f"{arm}_tcp_actual_stand_qx": 0,
                        f"{arm}_tcp_actual_stand_qy": 0,
                        f"{arm}_tcp_actual_stand_qz": 0,
                        f"{arm}_tcp_actual_stand_qw": 1,
                    })
                    for axis in "xyz":
                        value = force if axis == "z" else 0.0
                        row[f"{arm}_ft_fast_external_f{axis}_n"] = value
                        row[f"{arm}_ft_fast_external_t{axis}_nm"] = 0.0
                        row[f"{arm}_ft_control_external_f{axis}_n"] = value
                writer.writerow(row)

    def test_clean_static_log_passes_and_reports_tare_candidate(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "clean.csv"
            self.write_log(path)
            result = analyze_ft_acceptance.analyze_csv(path, 0.0, 1.9)
            self.assertTrue(result["promotion_ready"])
            self.assertEqual(result["arms"][0]["incremental_residual_tare_tcp_candidate"], [0, 0, 1, 0, 0, 0])

    def test_positive_compression_is_opposite_the_outward_surface_normal(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "compression.csv"
            self.write_log(path, measured=6.0, force=-6.0)
            result = analyze_ft_acceptance.analyze_csv(path, 0.0, 1.9)
            self.assertTrue(result["promotion_ready"])
            self.assertEqual(result["arms"][0]["normal_projection_error_max_n"], 0.0)

    def test_projection_disagreement_and_hard_limit_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "bad.csv"
            self.write_log(path, measured=0.0, force=25.0)
            result = analyze_ft_acceptance.analyze_csv(path, 0.0, 1.9)
            self.assertFalse(result["promotion_ready"])
            blockers = " ".join(result["arms"][0]["blockers"])
            self.assertIn("hard limit", blockers)
            self.assertIn("projection", blockers)

    def test_unaccepted_software_zero_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "not-zeroed.csv"
            self.write_log(path)
            with path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            fields = list(rows[0])
            rows[0]["left_ft_tare_valid"] = "0"
            rows[0]["left_ft_tare_state"] = "collecting"
            with path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=fields)
                writer.writeheader()
                writer.writerows(rows)
            result = analyze_ft_acceptance.analyze_csv(path, 0.0, 1.9)
            self.assertFalse(result["promotion_ready"])
            self.assertIn("software zero", " ".join(result["arms"][0]["blockers"]))


if __name__ == "__main__":
    unittest.main()
