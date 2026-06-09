import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path

import generate_rbpodo_physical_transition_report as report
import rbpodo_physical_stage_measure as measure


def pose(x: float, y: float, z: float) -> dict[str, object]:
    return {
        "x": x,
        "y": y,
        "z": z,
        "rx": 0.0,
        "ry": 0.0,
        "rz": 0.0,
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
    }


def state_row(
    index: int,
    *,
    actual_valid: bool = True,
    actual_xyz: tuple[float, float, float] | None = None,
    ref_xyz: tuple[float, float, float] | None = None,
    q_offset: float = 0.0,
    state_age_us: float = 1000.0,
) -> dict[str, object]:
    actual_xyz = actual_xyz or (float(index) * 0.001, 0.0, 0.0)
    ref_xyz = ref_xyz or actual_xyz
    host_time_ns = 1_000_000_000 + index * 10_000_000
    q_actual = [q_offset, 0.0, 0.0, 0.0, 0.0, 0.0]
    q_ref = [float(index) * 0.01, 0.0, 0.0, 0.0, 0.0, 0.0]
    return {
        "schema": "robotics_lab.servo_state.v1",
        "schema_version": 1,
        "host_time_ns": host_time_ns,
        "fault_latched": False,
        "left": {
            "host_time_ns": host_time_ns,
            "has_valid_joint_state": True,
            "q_actual_deg": q_actual,
            "q_ref_deg": q_ref,
            "state_age_us": state_age_us,
            "tcp_actual_valid": actual_valid,
            "tcp_actual_stand": pose(*actual_xyz),
            "tcp_ref_valid": True,
            "tcp_ref_stand": pose(*ref_xyz),
            "cartesian_available": True,
        },
        "right": {
            "host_time_ns": host_time_ns,
            "has_valid_joint_state": True,
            "q_actual_deg": q_actual,
            "q_ref_deg": q_ref,
            "state_age_us": state_age_us,
            "tcp_actual_valid": actual_valid,
            "tcp_actual_stand": pose(*actual_xyz),
            "tcp_ref_valid": True,
            "tcp_ref_stand": pose(*ref_xyz),
            "cartesian_available": True,
        },
    }


def write_jsonl(path: Path, rows: list[dict[str, object]]) -> None:
    path.write_text("\n".join(json.dumps(row) for row in rows) + "\n", encoding="utf-8")


def write_desired(path: Path, rows: list[tuple[float, tuple[float, float, float]]]) -> None:
    data = {
        "samples": [
            {"t_sec": t_sec, "desired_pose_stand": pose(x, y, z)}
            for t_sec, (x, y, z) in rows
        ]
    }
    path.write_text(json.dumps(data), encoding="utf-8")


def run_measure(argv: list[str]) -> tuple[int, str, str]:
    stdout = io.StringIO()
    stderr = io.StringIO()
    with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
        code = measure.main(argv)
    return code, stdout.getvalue(), stderr.getvalue()


class RbpodoPhysicalStageMeasureTest(unittest.TestCase):
    def test_clean_circle_uses_tcp_actual_stand_for_finite_metrics(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            capture = root / "capture.jsonl"
            desired = root / "desired.json"
            rows = [
                state_row(index, actual_xyz=(index * 0.01 + 0.001, 0.0, 0.0), ref_xyz=(index * 0.01 + 0.010, 0.0, 0.0), q_offset=index * 0.002)
                for index in range(5)
            ]
            write_jsonl(capture, rows)
            write_desired(desired, [(index * 0.01, (index * 0.01, 0.0, 0.0)) for index in range(5)])
            artifact_dir = root / "artifact"
            code, stdout, stderr = run_measure(
                [
                    "--stage",
                    "P6",
                    "--capture",
                    str(capture),
                    "--desired-trajectory",
                    str(desired),
                    "--arm",
                    "left",
                    "--artifact-dir",
                    str(artifact_dir),
                ]
            )
            summary = json.loads((artifact_dir / "summary.json").read_text(encoding="utf-8"))
        self.assertEqual(code, 0, stderr)
        self.assertIn("tcp_actual_stand", stdout)
        physical = summary["physical_tracking_result"]
        controller = summary["controller_reference_result"]
        self.assertEqual(physical["status"], "pass")
        self.assertEqual(physical["tracking_source"], "tcp_actual_stand")
        self.assertAlmostEqual(physical["rms_error_m"], 0.001)
        self.assertAlmostEqual(physical["p95_error_m"], 0.001)
        self.assertAlmostEqual(physical["max_error_m"], 0.001)
        self.assertEqual(controller["status"], "informational_only")
        self.assertEqual(controller["tracking_source"], "tcp_ref_stand")
        self.assertGreater(controller["rms_error_m"], physical["rms_error_m"])

    def test_all_invalid_actual_fails_closed_without_ref_fallback(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            capture = root / "capture.jsonl"
            desired = root / "desired.json"
            rows = [state_row(index, actual_valid=False, ref_xyz=(index * 0.01, 0.0, 0.0)) for index in range(3)]
            write_jsonl(capture, rows)
            write_desired(desired, [(index * 0.01, (index * 0.01, 0.0, 0.0)) for index in range(3)])
            artifact_dir = root / "artifact"
            code, _, _ = run_measure(
                [
                    "--stage",
                    "P6",
                    "--capture",
                    str(capture),
                    "--desired-trajectory",
                    str(desired),
                    "--arm",
                    "left",
                    "--artifact-dir",
                    str(artifact_dir),
                ]
            )
            summary = json.loads((artifact_dir / "summary.json").read_text(encoding="utf-8"))
        self.assertEqual(code, 2)
        self.assertEqual(summary["result"]["status"], "fail")
        self.assertEqual(summary["physical_tracking_result"]["status"], "fail")
        self.assertEqual(summary["physical_tracking_result"]["tracking_source"], "tcp_actual_stand")
        self.assertIsNone(summary["physical_tracking_result"]["rms_error_m"])
        self.assertIn("tcp_actual_stand is invalid", summary["physical_tracking_result"]["reason"])
        self.assertEqual(summary["controller_reference_result"]["status"], "informational_only")

    def test_update_rate_state_age_and_jitter_known_cadence(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            capture = root / "capture.jsonl"
            rows = [
                state_row(index, q_offset=index * 0.002, state_age_us=1000.0 + index * 100.0)
                for index in range(5)
            ]
            write_jsonl(capture, rows)
            artifact_dir = root / "artifact"
            code, _, stderr = run_measure(
                [
                    "--stage",
                    "P1",
                    "--capture",
                    str(capture),
                    "--arm",
                    "left",
                    "--artifact-dir",
                    str(artifact_dir),
                ]
            )
            summary = json.loads((artifact_dir / "summary.json").read_text(encoding="utf-8"))
        self.assertEqual(code, 0, stderr)
        telemetry = summary["telemetry_requirements"]
        self.assertAlmostEqual(telemetry["q_actual_update_rate_hz"], 100.0)
        self.assertAlmostEqual(telemetry["q_ref_update_rate_hz"], 100.0)
        self.assertAlmostEqual(telemetry["state_age_us"]["p95"], 1380.0)
        self.assertAlmostEqual(telemetry["state_age_us"]["max"], 1400.0)
        self.assertAlmostEqual(telemetry["state_jitter_us"]["p95"], 0.0)
        self.assertAlmostEqual(telemetry["state_jitter_us"]["max"], 0.0)
        self.assertTrue(telemetry["physical_motion_detected"])
        self.assertFalse(telemetry["physical_motion_expected"])
        self.assertEqual(summary["physical_tracking_result"]["status"], "not_run")

    def test_emitted_external_summary_passes_report_generator_validation(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            capture = root / "capture.jsonl"
            desired = root / "desired.json"
            rows = [state_row(index, actual_xyz=(index * 0.01, 0.0, 0.0), q_offset=index * 0.002) for index in range(4)]
            write_jsonl(capture, rows)
            write_desired(desired, [(index * 0.01, (index * 0.01, 0.0, 0.0)) for index in range(4)])
            artifact_dir = root / "artifact"
            code, _, stderr = run_measure(
                [
                    "--stage",
                    "P6",
                    "--capture",
                    str(capture),
                    "--desired-trajectory",
                    str(desired),
                    "--arm",
                    "left",
                    "--artifact-dir",
                    str(artifact_dir),
                ]
            )
            self.assertEqual(code, 0, stderr)
            out_json = root / "report.json"
            stdout = io.StringIO()
            with contextlib.redirect_stdout(stdout):
                report_code = report.main(["--artifact-dir", str(artifact_dir), "--json", str(out_json)])
            report_data = json.loads(out_json.read_text(encoding="utf-8"))
        self.assertEqual(report_code, 0)
        self.assertEqual(report_data["validation_errors"], [])
        row = next(row for row in report_data["stage_rows"] if row["stage_id"] == "P6")
        self.assertEqual(row["physical_tracking_source"], "tcp_actual_stand")
        self.assertEqual(row["controller_reference_source"], "tcp_ref_stand")
        self.assertEqual(row["status"], "pass")


if __name__ == "__main__":
    unittest.main()
