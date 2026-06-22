from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np

from scripts import analyze_pgprofile_run as ana
from tcp_tuning.config import Config
from tcp_tuning.trajectory_log import TrajectoryLogWriter


Q = np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64)


def _pose(x: float) -> np.ndarray:
    return np.asarray([x, 0.0, 0.2, *Q], dtype=np.float64)


def write_log(path: Path, *, with_smd_ref: bool, n: int = 40) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    writer = TrajectoryLogWriter(path, metadata={"episode_id": "ep"})
    t = np.arange(n) / 100.0
    for i in range(n):
        x = 0.001 * i
        for arm in ("left", "right"):
            row = {
                "t": float(t[i]),
                "t_source": float(t[i]),
                "src_idx": i,
                "arm": arm,
                "source_raw_target": _pose(x),
                "conditioned_goal_after_A": _pose(x),
                "reference_after_B": _pose(x - 0.0005),  # small lag
                "actual_tcp": np.full(7, np.nan),
                "ik_status": "ok",
                "ik_branch_jump_clamped": False,
                "branch_jump_flag": False,
                "singular_damping_flag": False,
                "safety_proj_flag": False,
                "self_collision_flag": False,
                "floor_flag": False,
                "roi_flag": False,
                "smd_goal_clamped_flag": False,
                "fault_latched": False,
                "ik_solve_us": 15.0,
                "ik_min_singular_value": 0.2,
                # Patch 4 separated SMD clip telemetry (fire on a few ticks).
                "smd_linear_velocity_clipped": i < 3,
                "smd_linear_accel_clipped": i < 2,
                "smd_angular_velocity_clipped": False,
                "smd_angular_accel_clipped": False,
                "smd_goal_linear_velocity_ff_clipped": i == 0,
                "smd_goal_angular_velocity_ff_clipped": False,
                "smd_velocity_feedforward_used": True,
                "smd_velocity_feedforward_source": "command_twist",
            }
            if with_smd_ref:
                row["smd_ref_stand"] = _pose(x - 0.0002)
            writer.append(row)
    writer.write()
    return path


class AnalyzeABCTest(unittest.TestCase):
    def test_b_falls_back_to_tcp_ref_with_warning_when_no_smd(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            log = write_log(Path(tmp) / "run" / "log.csv", with_smd_ref=False)
            result = ana.analyze_run(log, Config().metrics, validity_class="VALID_FULL_NO_GAP")
        b = result["B_reference_generation"]["left"]
        self.assertEqual(b["reference_source"], "tcp_ref_stand")
        self.assertIsNotNone(b["warning"])
        self.assertTrue(any("smd_ref_stand unavailable" in w for w in result["warnings"]))

    def test_b_uses_smd_ref_when_present_no_warning(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            log = write_log(Path(tmp) / "run" / "log.csv", with_smd_ref=True)
            result = ana.analyze_run(log, Config().metrics, validity_class="VALID_FULL_NO_GAP")
        b = result["B_reference_generation"]["left"]
        self.assertEqual(b["reference_source"], "smd_ref_stand")
        self.assertIsNone(b["warning"])
        self.assertFalse(any("smd_ref_stand unavailable" in w for w in result["warnings"]))

    def test_c_final_polish_unavailable_without_ma_fields(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            log = write_log(Path(tmp) / "run" / "log.csv", with_smd_ref=False)
            result = ana.analyze_run(log, Config().metrics, validity_class="VALID_FULL_NO_GAP")
        self.assertEqual(result["C_final_polish"]["left"]["status"], "unavailable")

    def test_smd_clip_breakdown_counts_separated_reasons(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            log = write_log(Path(tmp) / "run" / "log.csv", with_smd_ref=True)
            result = ana.analyze_run(log, Config().metrics, validity_class="VALID_FULL_NO_GAP")
        b = result["B_reference_generation"]["left"]
        clip = b["smd_clip"]
        self.assertEqual(clip["linear_velocity_clipped_count"], 3)
        self.assertEqual(clip["linear_accel_clipped_count"], 2)
        self.assertEqual(clip["angular_velocity_clipped_count"], 0)
        self.assertEqual(clip["goal_linear_velocity_ff_clipped_count"], 1)
        self.assertEqual(b["smd_velocity_feedforward_source"], "command_twist")
        self.assertAlmostEqual(b["smd_velocity_feedforward_used_ratio"], 1.0)

    def test_abcd_groups_present_and_backcompat_keys_kept(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            log = write_log(Path(tmp) / "run" / "log.csv", with_smd_ref=False)
            result = ana.analyze_run(log, Config().metrics, validity_class="VALID_FULL_NO_GAP")
        for key in ("A_conditioning", "B_reference_generation", "C_final_polish", "D_physical_tracking"):
            self.assertIn(key, result)
        for key in ("goal_conditioning_quality_A", "controller_reference_goal_following_B",
                    "physical_goal_tracking_C", "ik_safety_feasibility"):
            self.assertIn(key, result)

    def test_real_ready_label_is_time_scale_aware(self) -> None:
        self.assertEqual(ana.real_ready_label(1.0), "REAL_READY_TS_1P0")
        self.assertEqual(ana.real_ready_label(2.0), "REAL_READY_TS_2P0")
        self.assertEqual(ana.real_ready_label(1.5), "REAL_READY_TS_1P5")
        self.assertEqual(ana.real_ready_label(1.25), "REAL_READY_TS_1P25")
        with tempfile.TemporaryDirectory() as tmp:
            log = write_log(Path(tmp) / "run" / "log.csv", with_smd_ref=True)
            result = ana.analyze_run(log, Config().metrics, speed_precheck_pass=True,
                                     validity_class="VALID_FULL_NO_GAP", time_scale=2.0)
        self.assertEqual(result["classification"]["primary_class"], "REAL_READY_TS_2P0")
        self.assertEqual(result["time_scale"], 2.0)

    def test_bool_flag_parsing_and_classification_intact(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            log = write_log(Path(tmp) / "run" / "log.csv", with_smd_ref=False)
            result = ana.analyze_run(log, Config().metrics, speed_precheck_pass=True,
                                     validity_class="VALID_FULL_NO_GAP")
        # clean synthetic data -> no hard risks
        agg = result["classification"]["aggregate"]
        self.assertTrue(agg["hard_counts_zero"])
        self.assertEqual(result["classification"]["aggregate"]["self_collision_count"], 0)
        # summary renders without error and mentions reference_source + D section
        text = ana.render_summary(result)
        self.assertIn("B reference_source: tcp_ref_stand", text)
        self.assertIn("## D — physical tracking", text)


if __name__ == "__main__":
    unittest.main()
