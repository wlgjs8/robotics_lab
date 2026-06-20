from __future__ import annotations

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


if __name__ == "__main__":
    unittest.main()
