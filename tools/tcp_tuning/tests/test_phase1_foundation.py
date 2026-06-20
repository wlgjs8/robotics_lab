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

from audit_episode_hdf5 import detect_timestamp_gaps, main as audit_main
from tcp_tuning.hdf5_io import load_episode
from tcp_tuning.se3 import foh_pose, quat_canonical, twist_from_poses


class Phase1FoundationTest(unittest.TestCase):
    def test_audit_runs_on_episode_012_when_present(self) -> None:
        episode = Path("data/data_20260619_115712/episode_012.hdf5")
        if not episode.exists():
            self.skipTest(f"{episode} is absent")
        with tempfile.TemporaryDirectory() as tmp:
            rc = audit_main(["--episode", str(episode), "--out-dir", tmp])
            self.assertEqual(rc, 0)
            out_dir = Path(tmp) / "data_20260619_115712__episode_012"
            self.assertTrue((out_dir / "audit.json").exists())
            self.assertTrue((out_dir / "audit_summary.md").exists())

    def test_gap_detection_uses_multiplier_and_absolute_thresholds(self) -> None:
        t = np.asarray([0.0, 0.033, 0.066, 0.099, 0.250, 0.283], dtype=np.float64)
        gaps = detect_timestamp_gaps(t, median_multiplier=3.0, absolute_threshold_sec=0.100)
        self.assertEqual(len(gaps), 1)
        self.assertEqual(gaps[0]["before_index"], 3)
        self.assertIn("dt_gt_median_multiplier", gaps[0]["reasons"])
        self.assertIn("dt_gt_absolute_threshold", gaps[0]["reasons"])

    def test_quat_canonical_preserves_sign_continuity(self) -> None:
        q0 = np.asarray([0.0, 0.0, 0.0, 1.0])
        q1 = -q0
        q = quat_canonical(q1, ref=q0)
        self.assertGreaterEqual(float(np.dot(q0, q)), 0.0)
        np.testing.assert_allclose(q, q0)

    def test_foh_pose_endpoints_and_uniform_twist_no_spike(self) -> None:
        q = np.asarray([0.0, 0.0, 0.0, 1.0])
        p0 = np.asarray([0.0, 0.0, 0.0])
        p1 = np.asarray([0.1, 0.0, 0.0])
        p2 = np.asarray([0.2, 0.0, 0.0])

        p_start, q_start = foh_pose(0.0, 0.0, p0, q, 1.0, p1, q)
        p_stop, q_stop = foh_pose(1.0, 0.0, p0, q, 1.0, p1, q)
        np.testing.assert_allclose(p_start, p0)
        np.testing.assert_allclose(q_start, q)
        np.testing.assert_allclose(p_stop, p1)
        np.testing.assert_allclose(q_stop, q)

        v01, w01 = twist_from_poses(p0, q, p1, q, 1.0)
        v12, w12 = twist_from_poses(p1, q, p2, -q, 1.0)
        np.testing.assert_allclose(v01, v12, atol=1e-12)
        np.testing.assert_allclose(w01, w12, atol=1e-12)

    def test_load_episode_tolerates_renamed_and_missing_keys(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "renamed_episode.hdf5"
            with h5py.File(path, "w") as handle:
                handle.attrs["pose_format"] = "x,y,z,qx,qy,qz,qw"
                handle.create_dataset("custom_timestamp_seconds", data=np.asarray([0.0, 0.1], dtype=np.float64))
                pose = np.asarray(
                    [
                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                        [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
                    ],
                    dtype=np.float64,
                )
                handle.create_dataset("robot_left_target", data=pose)

            episode = load_episode(str(path))
            self.assertIsNotNone(episode.t_source)
            self.assertIsNotNone(episode.left_pose)
            self.assertIsNone(episode.right_pose)
            self.assertIsNone(episode.left_gripper)
            self.assertEqual(episode.detected["selected"]["left_pose"], "robot_left_target")
            np.testing.assert_allclose(episode.left_pose[:, 3:7], np.asarray([[0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 0.0, 1.0]]))


if __name__ == "__main__":
    unittest.main()
