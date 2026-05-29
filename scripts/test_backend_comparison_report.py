import csv
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import generate_backend_comparison_report as report


ROOT = Path(__file__).resolve().parents[1]


def write_summary(root: Path, name: str, data: dict) -> Path:
    path = root / name / "summary.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data), encoding="utf-8")
    return path


class BackendComparisonReportTests(unittest.TestCase):
    def test_help_works(self):
        completed = subprocess.run(
            [sys.executable, "scripts/generate_backend_comparison_report.py", "--help"],
            cwd=ROOT,
            check=True,
            text=True,
            capture_output=True,
        )
        self.assertIn("--output-md", completed.stdout)
        self.assertIn("--output-csv", completed.stdout)

    def test_report_classifies_rbpodo_read_state_success(self):
        root = Path(tempfile.mkdtemp())
        read_state = write_summary(root, "rbpodo_read_state", {
            "backend": "rbpodo",
            "mode": "read_state",
            "result": "completed",
            "rate_results": [
                {"requested_rate_hz": 50.0, "achieved_rate_hz": 50.0, "success_rate": 1.0, "send_count": 10},
                {"requested_rate_hz": 100.0, "achieved_rate_hz": 99.2, "success_rate": 1.0, "send_count": 10},
            ],
        })
        read_only = write_summary(root, "rbpodo_read_only", {
            "observed_backend": "rbpodo",
            "mode": "read_only",
            "result": "completed",
            "result_reason": "state stream captured",
            "state_valid_ratio": 1.0,
        })

        rows = report.collect_rows([read_state, read_only])
        read_rows = [row for row in rows if row["backend"] == "rbpodo" and row["metric_group"] == "read_state"]
        self.assertTrue(all(row["classification"] == "measured_and_comparable" for row in read_rows))
        self.assertEqual(report.recommendation(rows)["primary_backend_recommendation"], "rbpodo")

    def test_report_classifies_rbscript_read_state_unsupported(self):
        root = Path(tempfile.mkdtemp())
        path = write_summary(root, "rbscript_read_state", {
            "backend": "rbscript_tcp",
            "mode": "read_state",
            "result": "completed",
            "read_state_capability": "unsupported",
            "rbscript_tcp_data_port_mode": "real_controller_unsupported",
            "comparable": False,
            "not_comparable_reason": "rbscript_tcp real data port unsupported",
        })

        rows = report.collect_rows([path])
        self.assertEqual(rows[0]["classification"], "unsupported")
        markdown = report.generate_markdown(rows, report.recommendation(rows))
        self.assertIn("rbscript_tcp read_state unsupported", markdown)

    def test_ack_no_motion_is_not_compared_against_servoj(self):
        root = Path(tempfile.mkdtemp())
        path = write_summary(root, "rbscript_ack", {
            "backend": "rbscript_tcp",
            "mode": "command_ack_no_motion",
            "result": "completed",
            "success_rate": 1.0,
            "sample_count": 5,
            "success_count": 5,
        })

        rows = report.collect_rows([path])
        self.assertEqual(rows[0]["classification"], "measured_not_comparable")
        self.assertIn("not ServoJ test", rows[0]["note"])
        markdown = report.generate_markdown(rows, report.recommendation(rows))
        self.assertIn("rbscript_tcp command ACK test is not ServoJ test", markdown)

    def test_missing_summary_is_not_yet_run(self):
        missing = Path(tempfile.mkdtemp()) / "rbscript_servo_j_noop" / "summary.json"
        rows = report.collect_rows([missing])
        self.assertEqual(rows[0]["classification"], "not_yet_run")
        self.assertEqual(rows[0]["status"], "not_yet_run")
        self.assertIn("summary missing", rows[0]["reason"])

    def test_cli_writes_markdown_csv_and_json(self):
        root = Path(tempfile.mkdtemp())
        path = write_summary(root, "rbscript_read_state", {
            "backend": "rbscript_tcp",
            "mode": "read_state",
            "result": "completed",
            "read_state_capability": "unsupported",
            "rbscript_tcp_data_port_mode": "real_controller_unsupported",
        })
        out_md = root / "report.md"
        out_csv = root / "report.csv"
        out_json = root / "report.json"

        subprocess.run(
            [
                sys.executable,
                "scripts/generate_backend_comparison_report.py",
                str(path),
                "--output-md",
                str(out_md),
                "--output-csv",
                str(out_csv),
                "--output-json",
                str(out_json),
            ],
            cwd=ROOT,
            check=True,
            text=True,
            capture_output=True,
        )

        self.assertIn("# Backend Comparison Report", out_md.read_text(encoding="utf-8"))
        with out_csv.open(encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
        self.assertEqual(rows[0]["classification"], "unsupported")
        payload = json.loads(out_json.read_text(encoding="utf-8"))
        self.assertIn("primary_backend_recommendation", payload["recommendation"])


if __name__ == "__main__":
    unittest.main()
