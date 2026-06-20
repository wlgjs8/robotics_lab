from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from analyze_tcp_replay_logs import main as analyze_main
from tcp_tuning.config import MetricsConfig
from tcp_tuning.metrics import smoothness_metrics, tracking_metrics


Q = np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64)


def pose_series(t: np.ndarray, x: np.ndarray) -> np.ndarray:
    out = np.zeros((t.size, 7), dtype=np.float64)
    out[:, 0] = x
    out[:, 3:7] = Q
    return out


class MetricsAnalysisTest(unittest.TestCase):
    def test_missing_actual_tcp_returns_null_note(self) -> None:
        t = np.arange(5, dtype=np.float64) * 0.002
        goal = pose_series(t, np.linspace(0.0, 0.01, t.size))
        metrics = tracking_metrics(t, actual_tcp=None, reference_after_B=None, conditioned_goal=goal)
        actual_vs_goal = metrics["actual_tcp_vs_conditioned_goal"]
        self.assertEqual(actual_vs_goal["status"], "null")
        self.assertIsNone(actual_vs_goal["value"])
        self.assertIn("missing", actual_vs_goal["reason"])

    def test_frozen_actual_tcp_marked_not_measured(self) -> None:
        t = np.arange(50, dtype=np.float64) * 0.002
        moving = pose_series(t, np.linspace(0.0, 0.05, t.size))  # 50 mm of motion
        frozen = pose_series(t, np.full(t.size, 0.0))  # actual never moves
        metrics = tracking_metrics(
            t,
            actual_tcp=frozen,
            reference_after_B=moving,
            conditioned_goal=moving,
        )
        self.assertEqual(metrics["physical_tracking"]["status"], "not_measured")
        actual_vs_goal = metrics["actual_tcp_vs_conditioned_goal"]
        self.assertEqual(actual_vs_goal["status"], "not_measured")
        self.assertEqual(actual_vs_goal["tracking_source"], "tcp_actual_stand")
        self.assertEqual(metrics["actual_tcp_vs_reference_after_B"]["status"], "not_measured")
        # The pgmode-meaningful metric stays a real comparison, not relabeled.
        self.assertEqual(metrics["reference_after_B_vs_conditioned_goal"]["status"], "ok")

    def test_moving_actual_tcp_stays_measured(self) -> None:
        t = np.arange(50, dtype=np.float64) * 0.002
        goal = pose_series(t, np.linspace(0.0, 0.05, t.size))
        actual = pose_series(t, np.linspace(0.0, 0.049, t.size))  # tracks the goal
        metrics = tracking_metrics(t, actual_tcp=actual, reference_after_B=goal, conditioned_goal=goal)
        self.assertEqual(metrics["physical_tracking"]["status"], "measured")
        self.assertEqual(metrics["actual_tcp_vs_conditioned_goal"]["status"], "ok")

    def test_sinusoid_smoothness_recovers_dominant_frequency(self) -> None:
        sample_rate_hz = 200.0
        frequency_hz = 3.0
        t = np.arange(0.0, 4.0, 1.0 / sample_rate_hz, dtype=np.float64)
        x = 0.01 * np.sin(2.0 * np.pi * frequency_hz * t)
        metrics = smoothness_metrics(t, pose_series(t, x), cfg=MetricsConfig(high_frequency_cutoff_hz=5.0))
        peak = metrics["linear_velocity_spectrum"]["dominant_frequency_hz"]
        self.assertIsNotNone(peak)
        self.assertAlmostEqual(float(peak), frequency_hz, delta=0.30)

    def test_raw_zoh_has_more_high_frequency_power_and_reversals_than_raw_foh_on_episode(self) -> None:
        base = Path("outputs/tcp_tuning/data_20260619_115712__episode_012")
        zoh = base / "raw_zoh_500hz.npz"
        foh = base / "raw_foh_se3_500hz.npz"
        if not (zoh.exists() and foh.exists()):
            self.skipTest("generated episode_012 npz artifacts are absent")
        with np.load(zoh, allow_pickle=True) as zoh_data, np.load(foh, allow_pickle=True) as foh_data:
            zoh_metrics = smoothness_metrics(zoh_data["t_servo"], zoh_data["left_conditioned_goal"], cfg=MetricsConfig())
            foh_metrics = smoothness_metrics(foh_data["t_servo"], foh_data["left_conditioned_goal"], cfg=MetricsConfig())
        self.assertGreater(
            zoh_metrics["linear_velocity_spectrum"]["power_above_cutoff"],
            foh_metrics["linear_velocity_spectrum"]["power_above_cutoff"],
        )
        self.assertGreater(
            zoh_metrics["linear_velocity_sign_reversals_per_sec"]["per_sec"],
            foh_metrics["linear_velocity_sign_reversals_per_sec"]["per_sec"],
        )

    def test_analyze_cli_runs_on_generated_npz(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            npz_path = Path(tmp) / "raw_zoh_10hz.npz"
            t = np.arange(0.0, 0.5, 0.1, dtype=np.float64)
            goal = pose_series(t, 0.01 * t)
            arrays = {
                "t_servo": t,
                "servo_rate_hz": np.asarray(10.0),
                "mode": np.asarray("raw_zoh"),
                "episode": np.asarray("data/example_episode/episode_000.hdf5"),
                "seed": np.asarray(-1),
                "segments": np.asarray([[0, t.size]], dtype=np.int64),
                "gaps": np.empty((0, 2), dtype=np.float64),
                "meta_json": np.asarray(json.dumps({"episode_id": "example_episode__episode_000"})),
            }
            for arm in ("left", "right"):
                arrays[f"{arm}_source_raw_target"] = goal
                arrays[f"{arm}_conditioned_goal"] = goal
                arrays[f"{arm}_conditioned_twist"] = np.full((t.size, 6), np.nan)
                arrays[f"{arm}_reference_after_B"] = np.full((t.size, 7), np.nan)
                arrays[f"{arm}_actual_tcp"] = np.full((t.size, 7), np.nan)
                arrays[f"{arm}_q_target"] = np.full((t.size, 6), np.nan)
                arrays[f"{arm}_q_actual"] = np.full((t.size, 6), np.nan)
                for flag in ("valid", "hold", "dropout", "gap", "reanchor"):
                    arrays[f"{arm}_{flag}"] = np.zeros(t.size, dtype=bool)
                arrays[f"{arm}_src_id_lo"] = np.arange(t.size)
                arrays[f"{arm}_src_id_hi"] = np.arange(t.size)
            np.savez(npz_path, **arrays)

            out_dir = Path(tmp) / "out"
            rc = analyze_main(["--npz", str(npz_path), "--out-dir", str(out_dir)])
            self.assertEqual(rc, 0)
            analysis_dir = out_dir / "example_episode__episode_000" / "analysis" / "raw_zoh_10hz"
            self.assertTrue((analysis_dir / "metrics.json").exists())
            self.assertTrue((analysis_dir / "summary.md").exists())
            payload = json.loads((analysis_dir / "metrics.json").read_text(encoding="utf-8"))
            self.assertEqual(payload["schema_id"], "robotics_lab.tcp_tuning.metrics.v1")


if __name__ == "__main__":
    unittest.main()
