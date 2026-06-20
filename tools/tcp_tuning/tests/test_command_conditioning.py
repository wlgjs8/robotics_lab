from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
import sys

import h5py
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
TOOLS = ROOT / "tools"
for _path in (str(ROOT), str(TOOLS)):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from tcp_tuning.command_conditioner import CommandConditioner
from tcp_tuning.config import Config
from tcp_tuning.se3 import quat_canonical

from generate_replay_target import main as generate_main


Q = np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64)


def pose(x: float, quat=Q) -> np.ndarray:
    return np.asarray([x, 0.0, 0.0, *quat], dtype=np.float64)


class CommandConditioningTest(unittest.TestCase):
    def test_raw_zoh_holds_between_source_frames(self) -> None:
        c = CommandConditioner("raw_zoh", Config())
        c.update_source_sample(0.0, pose(0.0), pose(1.0))
        c.update_source_sample(1.0, pose(0.1), pose(1.1))
        cmd = c.sample(0.5)
        np.testing.assert_allclose(cmd.left_pose, pose(0.0))
        self.assertEqual(cmd.src_ids, (0, 0))
        self.assertTrue(cmd.hold)

    def test_raw_foh_hits_endpoints_and_has_no_uniform_ramp_spike(self) -> None:
        c = CommandConditioner("raw_foh_se3", Config())
        for index in range(4):
            c.update_source_sample(float(index), pose(0.1 * index), pose(1.0 + 0.1 * index))
        for index in range(4):
            cmd = c.sample(float(index))
            np.testing.assert_allclose(cmd.left_pose, pose(0.1 * index), atol=1e-12)
        t = np.arange(0.0, 3.0 + 0.002, 0.002)
        x = np.asarray([c.sample(float(item)).left_pose[0] for item in t])
        speed = np.diff(x) / 0.002
        self.assertLessEqual(float(np.max(np.abs(speed - 0.1))), 1e-9)

    def test_quaternion_interp_handles_sign_flip(self) -> None:
        c = CommandConditioner("raw_foh_se3", Config())
        c.update_source_sample(0.0, pose(0.0, Q), pose(1.0, Q))
        c.update_source_sample(1.0, pose(0.1, -Q), pose(1.1, -Q))
        cmd = c.sample(0.5)
        self.assertGreaterEqual(float(np.dot(Q, quat_canonical(cmd.left_pose[3:7], ref=Q))), 0.0)
        np.testing.assert_allclose(cmd.left_pose[3:7], Q, atol=1e-12)

    def test_clean_foh_does_not_smooth_across_gap_boundary(self) -> None:
        c = CommandConditioner("clean_foh_se3", Config())
        samples = [
            (0.000, 0.0),
            (0.033, 0.01),
            (0.066, 0.02),
            (0.300, 1.00),
            (0.333, 1.01),
            (0.366, 1.02),
        ]
        for t, x in samples:
            c.update_source_sample(t, pose(x), pose(x + 1.0))
        clean = c.clean_source["left_pose"]
        self.assertEqual(c.segments, [(0, 3), (3, 6)])
        self.assertLess(float(np.max(clean[:3, 0])), 0.05)
        self.assertGreater(float(np.min(clean[3:, 0])), 0.95)

    def test_generated_npz_contains_contract_keys(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            episode = Path(tmp) / "episode_test.hdf5"
            with h5py.File(episode, "w") as handle:
                handle.attrs["pose_format"] = "x,y,z,qx,qy,qz,qw"
                handle.create_dataset("timestamps", data=np.asarray([0.0, 0.1, 0.2], dtype=np.float64))
                left = np.stack([pose(0.0), pose(0.1), pose(0.2)], axis=0)
                right = np.stack([pose(1.0), pose(1.1), pose(1.2)], axis=0)
                handle.create_dataset("left_target_pose", data=left)
                handle.create_dataset("right_target_pose", data=right)
            out_dir = Path(tmp) / "out"
            rc = generate_main(["--episode", str(episode), "--mode", "raw_foh_se3", "--out-dir", str(out_dir), "--servo-rate-hz", "10"])
            self.assertEqual(rc, 0)
            npz_path = out_dir / f"{episode.parent.name}__{episode.stem}" / "raw_foh_se3_10hz.npz"
            self.assertTrue(npz_path.exists())
            with np.load(npz_path, allow_pickle=True) as data:
                base_keys = {"t_servo", "servo_rate_hz", "mode", "episode", "seed", "segments", "gaps", "meta_json"}
                for key in base_keys:
                    self.assertIn(key, data.files)
                for arm in ("left", "right"):
                    for suffix in (
                        "source_raw_target",
                        "conditioned_goal",
                        "conditioned_twist",
                        "gripper",
                        "valid",
                        "hold",
                        "dropout",
                        "gap",
                        "reanchor",
                        "src_id_lo",
                        "src_id_hi",
                        "reference_after_B",
                        "q_target",
                        "q_actual",
                        "actual_tcp",
                    ):
                        self.assertIn(f"{arm}_{suffix}", data.files)

    def test_generator_auto_largest_segment_is_gap_free_and_metadata_encoded(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            episode = write_gap_episode(Path(tmp) / "episode_gap.hdf5")
            out_dir = Path(tmp) / "out"
            rc = generate_main(
                [
                    "--episode",
                    str(episode),
                    "--mode",
                    "clean_foh_se3",
                    "--segment",
                    "auto-largest",
                    "--out-dir",
                    str(out_dir),
                    "--servo-rate-hz",
                    "100",
                ]
            )
            self.assertEqual(rc, 0)
            npz_path = out_dir / f"{episode.parent.name}__{episode.stem}" / "clean_foh_se3_segment_1_3_7_100hz.npz"
            self.assertTrue(npz_path.exists())
            with np.load(npz_path, allow_pickle=True) as data:
                meta = json.loads(str(np.asarray(data["meta_json"]).item()))
                self.assertFalse(meta["real_replay_unsafe_full_clean"])
                self.assertIsNone(meta["real_replay_unsafe_full_clean_reason"])
                self.assertEqual(meta["segment_selection"]["segment_index"], 1)
                self.assertEqual(meta["segment_selection"]["source_start"], 3)
                self.assertEqual(meta["segment_selection"]["source_stop_exclusive"], 7)
                self.assertLess(max_linear_tick_speed(data), 1.0)

    def test_generator_full_multisegment_clean_marks_real_replay_unsafe(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            episode = write_gap_episode(Path(tmp) / "episode_gap.hdf5")
            out_dir = Path(tmp) / "out"
            rc = generate_main(
                [
                    "--episode",
                    str(episode),
                    "--mode",
                    "clean_foh_se3",
                    "--out-dir",
                    str(out_dir),
                    "--servo-rate-hz",
                    "100",
                ]
            )
            self.assertEqual(rc, 0)
            npz_path = out_dir / f"{episode.parent.name}__{episode.stem}" / "clean_foh_se3_100hz.npz"
            self.assertTrue(npz_path.exists())
            with np.load(npz_path, allow_pickle=True) as data:
                meta = json.loads(str(np.asarray(data["meta_json"]).item()))
                self.assertTrue(meta["real_replay_unsafe_full_clean"])
                self.assertIn("gap-boundary one-tick velocity spike", meta["real_replay_unsafe_full_clean_reason"])
                self.assertEqual(meta["segment_selection"]["mode"], "all")
                self.assertGreater(max_linear_tick_speed(data), 10.0)

    def test_generator_start_stop_segment_encodes_range(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            episode = write_gap_episode(Path(tmp) / "episode_gap.hdf5")
            out_dir = Path(tmp) / "out"
            rc = generate_main(
                [
                    "--episode",
                    str(episode),
                    "--mode",
                    "clean_foh_se3",
                    "--segment",
                    "4:7",
                    "--out-dir",
                    str(out_dir),
                    "--servo-rate-hz",
                    "100",
                ]
            )
            self.assertEqual(rc, 0)
            npz_path = out_dir / f"{episode.parent.name}__{episode.stem}" / "clean_foh_se3_segment_1_4_7_100hz.npz"
            self.assertTrue(npz_path.exists())
            with np.load(npz_path, allow_pickle=True) as data:
                meta = json.loads(str(np.asarray(data["meta_json"]).item()))
                self.assertFalse(meta["real_replay_unsafe_full_clean"])
                self.assertEqual(meta["segment_selection"]["source_frame_range"], [4, 6])


def write_gap_episode(path: Path) -> Path:
    with h5py.File(path, "w") as handle:
        handle.attrs["pose_format"] = "x,y,z,qx,qy,qz,qw"
        handle.create_dataset(
            "timestamps",
            data=np.asarray([0.000, 0.033, 0.066, 0.300, 0.333, 0.366, 0.399], dtype=np.float64),
        )
        left = np.stack(
            [
                pose(0.00),
                pose(0.01),
                pose(0.02),
                pose(1.00),
                pose(1.01),
                pose(1.02),
                pose(1.03),
            ],
            axis=0,
        )
        right = np.stack([pose(float(row[0] + 1.0)) for row in left], axis=0)
        handle.create_dataset("left_target_pose", data=left)
        handle.create_dataset("right_target_pose", data=right)
    return path


def max_linear_tick_speed(data) -> float:
    t = np.asarray(data["t_servo"], dtype=np.float64)
    goal = np.asarray(data["left_conditioned_goal"], dtype=np.float64)
    if t.size < 2:
        return 0.0
    dt = np.diff(t)
    dp = np.linalg.norm(np.diff(goal[:, :3], axis=0), axis=1)
    valid = np.isfinite(dt) & (dt > 0.0) & np.isfinite(dp)
    return float(np.max(dp[valid] / dt[valid])) if np.any(valid) else 0.0


if __name__ == "__main__":
    unittest.main()
