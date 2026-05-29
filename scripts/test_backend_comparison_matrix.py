import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import run_backend_comparison_matrix as matrix


ROOT = Path(__file__).resolve().parents[1]


def write_matrix(text: str) -> Path:
    root = Path(tempfile.mkdtemp())
    path = root / "matrix.yaml"
    path.write_text(text, encoding="utf-8")
    return path


class BackendComparisonMatrixTests(unittest.TestCase):
    def test_help_works(self):
        completed = subprocess.run(
            [sys.executable, "scripts/run_backend_comparison_matrix.py", "--help"],
            cwd=ROOT,
            check=True,
            text=True,
            capture_output=True,
        )
        self.assertIn("--matrix", completed.stdout)
        self.assertIn("--dry-run", completed.stdout)
        self.assertIn("--allow-servo-j-noop-simulation", completed.stdout)

    def test_dry_run_produces_commands(self):
        matrix_path = write_matrix(
            """
controllers:
  left_ip: 127.0.0.1
  right_ip: 127.0.0.1
  arm: left

experiments:
  - name: rbscript_read
    backend: rbscript_tcp
    script: rainbow_rate_probe
    mode: read_state
    rates: [10, 20]
    duration_sec: 1
"""
        )
        artifact_root = Path(tempfile.mkdtemp()) / "artifacts"
        completed = subprocess.run(
            [
                sys.executable,
                "scripts/run_backend_comparison_matrix.py",
                "--matrix",
                str(matrix_path),
                "--artifact-root",
                str(artifact_root),
                "--dry-run",
                "--max-workers",
                "1",
            ],
            cwd=ROOT,
            check=True,
            text=True,
            capture_output=True,
        )
        self.assertIn("rainbow_rate_probe.py", completed.stdout)
        command_path = artifact_root / "rbscript_read" / "experiment_command.txt"
        self.assertIn("--mode read_state", command_path.read_text(encoding="utf-8"))
        summary = json.loads((artifact_root / "backend_comparison_summary.json").read_text(encoding="utf-8"))
        self.assertEqual(summary["experiments"][0]["status"], "dry_run")

    def test_matrix_parser_rejects_unknown_script_backend_mode(self):
        for body, expected in (
            ("backend: nope\n    script: rainbow_rate_probe\n    mode: read_state", "unsupported backend"),
            ("backend: rbpodo\n    script: nope\n    mode: read_state", "unsupported script"),
            ("backend: rbpodo\n    script: rainbow_rate_probe\n    mode: fly", "unsupported mode"),
        ):
            path = write_matrix(
                f"""
controllers:
  left_ip: 127.0.0.1
  right_ip: 127.0.0.1
  arm: left

experiments:
  - name: bad
    {body}
"""
            )
            parsed = matrix.load_matrix(path)
            with self.assertRaisesRegex(matrix.MatrixError, expected):
                matrix.validate_matrix(parsed)

    def test_disabled_experiments_skipped(self):
        path = write_matrix(
            """
controllers:
  left_ip: 127.0.0.1
  right_ip: 127.0.0.1
  arm: left

experiments:
  - name: disabled_read
    backend: rbpodo
    script: rainbow_rate_probe
    mode: read_state
    enabled: false
"""
        )
        artifact_root = Path(tempfile.mkdtemp()) / "artifacts"
        namespace = type("Args", (), {
            "matrix": path,
            "artifact_root": artifact_root,
            "max_workers": 1,
            "dry_run": True,
            "include_disabled": False,
            "allow_servo_j_noop_simulation": False,
            "i_understand_this_connects_to_real_controller": False,
            "skip_plots": True,
        })()
        result = matrix.run_matrix(namespace)
        self.assertEqual(result["experiments"][0]["status"], "skipped")
        status = json.loads((artifact_root / "disabled_read" / "experiment_status.json").read_text(encoding="utf-8"))
        self.assertEqual(status["status"], "skipped")

    def test_unsupported_experiments_summarized(self):
        path = write_matrix(
            """
controllers:
  left_ip: 127.0.0.1
  right_ip: 127.0.0.1
  arm: left

experiments:
  - name: rbpodo_ack_no_motion
    backend: rbpodo
    script: rainbow_rate_probe
    mode: ack_no_motion
"""
        )
        artifact_root = Path(tempfile.mkdtemp()) / "artifacts"
        namespace = type("Args", (), {
            "matrix": path,
            "artifact_root": artifact_root,
            "max_workers": 1,
            "dry_run": True,
            "include_disabled": False,
            "allow_servo_j_noop_simulation": False,
            "i_understand_this_connects_to_real_controller": False,
            "skip_plots": True,
        })()
        result = matrix.run_matrix(namespace)
        self.assertEqual(result["experiments"][0]["status"], "unsupported")
        self.assertIn("ack_no_motion", result["experiments"][0]["reason"])

    def test_fake_summary_aggregation(self):
        status = {
            "name": "rbscript_ack",
            "status": "completed",
            "reason": "",
            "backend": "rbscript_tcp",
            "script": "rainbow_rate_probe",
            "mode": "command_ack_no_motion",
            "profile": "",
            "artifact_dir": "/tmp/rbscript_ack",
        }
        summary = {
            "backend": "rbscript_tcp",
            "mode": "ack_no_motion",
            "rate_results": [
                {
                    "requested_rate_hz": 50.0,
                    "achieved_rate_hz": 49.9,
                    "send_count": 10,
                    "success_rate": 0.9,
                    "p95_ack_us": 200.0,
                    "ack_timeout_count": 1,
                    "persistent_socket": True,
                    "reconnect_count": 0,
                }
            ],
        }
        row = matrix.status_summary_row(status, summary)
        self.assertEqual(row["experiment"], "rbscript_ack")
        self.assertEqual(row["requested_rate_hz"], 50.0)
        self.assertEqual(row["success_rate"], 0.9)
        self.assertEqual(row["p95_ack_us"], 200.0)
        self.assertTrue(row["persistent_socket"])


if __name__ == "__main__":
    unittest.main()
