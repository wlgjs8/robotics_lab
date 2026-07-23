from __future__ import annotations

import importlib.util
import contextlib
import json
import tempfile
import unittest
from io import StringIO
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "analyze_rollout_step_log.py"
SPEC = importlib.util.spec_from_file_location("analyze_rollout_step_log", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
ANALYZER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ANALYZER)


class AnalyzeRolloutStepLogTest(unittest.TestCase):
    def test_descent_segments_and_gripper_toggles(self) -> None:
        samples = [
            {"cmd_z_mm": 300.0, "gripper_cmd_pct": 80.0},
            {"cmd_z_mm": 299.0, "gripper_cmd_pct": 75.0},
            {"cmd_z_mm": 297.0, "gripper_cmd_pct": 10.0},
            {"cmd_z_mm": 297.0, "gripper_cmd_pct": 12.0},
            {"cmd_z_mm": 296.0, "gripper_cmd_pct": 90.0},
        ]

        self.assertEqual(
            ANALYZER.descent_segments(samples, epsilon_mm=0.01),
            [[0, 1, 2], [3, 4]],
        )
        self.assertEqual(
            ANALYZER.gripper_toggles(samples, threshold_pct=50.0),
            [(2, "CLOSE", 10.0), (4, "OPEN", 90.0)],
        )

    def test_text_summary_reads_schema_and_reports_diagnostic_hints(self) -> None:
        records = []
        for index, (cmd_z, meas_z, offset_z, gripper) in enumerate(
            (
                (0.300, 0.300, 0.000, 80.0),
                (0.298, 0.300, 0.001, 10.0),
                (0.296, 0.299, 0.002, 80.0),
            )
        ):
            records.append(
                {
                    "schema": ANALYZER.SCHEMA,
                    "t_mono": float(index),
                    "t_wall": 1_700_000_000.0 + index,
                    "chunk_id": 1,
                    "chunk_step_index": index,
                    "stall": False,
                    "hold": False,
                    "arms": {
                        arm: {
                            "cmd_pose": [0.4, 0.0, cmd_z, 0.0, 0.0, 0.0, 1.0],
                            "meas_pose": [0.4, 0.0, meas_z, 0.0, 0.0, 0.0, 1.0],
                            "cmd_minus_meas_z_mm": (cmd_z - meas_z) * 1000.0,
                            "compliance_offset_surface": [
                                0.0,
                                0.0,
                                offset_z,
                                0.0,
                                0.0,
                                0.0,
                            ],
                            "correction_m": abs(offset_z),
                            "wrench_tcp_fz": 3.0,
                            "control_external_wrench_fz": 2.0,
                            "gripper_cmd_pct": gripper,
                            "gripper_meas_pct": gripper,
                        }
                        for arm in ("left", "right")
                    },
                }
            )
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "steps.jsonl"
            path.write_text(
                "\n".join(json.dumps(record) for record in records) + "\n",
                encoding="utf-8",
            )
            loaded, rejected = ANALYZER.load_records(path)
            stdout = StringIO()
            with contextlib.redirect_stdout(stdout):
                ANALYZER.print_summary(
                    loaded,
                    rejected=rejected,
                    descent_epsilon_mm=0.01,
                    gripper_threshold_pct=50.0,
                    max_points=80,
                    blocked_follow_ratio=0.25,
                    offset_growth_mm=0.5,
                )

        output = stdout.getvalue()
        self.assertIn("descent_segments=1", output)
        self.assertIn("gripper toggles", output)
        self.assertIn("force-control 차단 가능성", output)
        self.assertIn("모델/인지 문제 가능성", output)

    @unittest.skipUnless(
        importlib.util.find_spec("matplotlib") is not None,
        "matplotlib is not installed",
    )
    def test_optional_png_plot(self) -> None:
        samples = [
            {
                "t_mono": float(index),
                "cmd_z_mm": 300.0 - index,
                "meas_z_mm": 300.0 - 0.5 * index,
                "gap_z_mm": -0.5 * index,
                "offset_z_mm": float(index),
                "correction_mm": float(index),
                "gripper_cmd_pct": 80.0 if index == 0 else 10.0,
                "gripper_meas_pct": 75.0 if index == 0 else 15.0,
            }
            for index in range(3)
        ]
        analyzed = {
            arm: (samples, [[0, 1, 2]])
            for arm in ("left", "right")
        }
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "rollout.png"
            self.assertTrue(ANALYZER.save_plot(path, analyzed))
            self.assertGreater(path.stat().st_size, 0)


if __name__ == "__main__":
    unittest.main()
