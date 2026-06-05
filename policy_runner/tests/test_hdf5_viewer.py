from __future__ import annotations

import io
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

try:
    import h5py
    import numpy as np

    from policy_runner.flow_dataset import pose_delta
    from policy_runner.hdf5_viewer import (
        frame_summary,
        load_viewer_episode,
        render_viewer_frame,
        run_hdf5_viewer_cli,
    )
except ModuleNotFoundError:
    h5py = None
    np = None


@unittest.skipIf(h5py is None or np is None, "HDF5 viewer extras not installed")
class Hdf5ViewerTest(unittest.TestCase):
    def test_uploaded_episode_loads_left_active_when_present(self) -> None:
        episode = Path("episode_002.hdf5")
        if not episode.exists():
            self.skipTest("uploaded episode_002.hdf5 is not present")

        viewer = load_viewer_episode(episode)
        summary = frame_summary(viewer, 0)

        self.assertEqual(viewer.episode.format_name, "pika_umi_single_arm")
        self.assertEqual(viewer.episode.arm_mask.tolist(), [1.0, 0.0])
        self.assertTrue(summary["arms"]["left"]["active"])
        self.assertFalse(summary["arms"]["right"]["active"])
        self.assertIn("realsense_color", viewer.camera_names)

    def test_synthetic_bimanual_episode_renders_both_arms_active(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "episode_001.hdf5"
            _write_bimanual_episode(path)

            viewer = load_viewer_episode(path)
            summary = frame_summary(viewer, 1)
            canvas = render_viewer_frame(viewer, 1, image_size=24, trail_length=3)

            self.assertTrue(summary["arms"]["left"]["active"])
            self.assertTrue(summary["arms"]["right"]["active"])
            self.assertEqual(canvas.dtype, np.uint8)
            self.assertEqual(canvas.ndim, 3)
            self.assertEqual(canvas.shape[2], 3)
            self.assertGreater(int(canvas.sum()), 0)

    def test_action_delta_matches_pose_delta(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "episode_001.hdf5"
            _write_single_arm_episode(path)

            viewer = load_viewer_episode(path)
            summary = frame_summary(viewer, 0)
            expected = pose_delta(
                viewer.episode.left_pose[0],
                viewer.episode.action_left_pose[0],
            )

            self.assertTrue(np.allclose(summary["arms"]["left"]["action"][:6], expected))
            self.assertAlmostEqual(float(summary["arms"]["left"]["action"][6]), 0.2)

    def test_cv2_missing_returns_clear_error(self) -> None:
        args = SimpleNamespace(
            episode="episode_002.hdf5",
            single_arm_side="left",
            camera_names=None,
            start_frame=0,
            fps=None,
            image_size=16,
            trail_length=4,
        )
        stderr = io.StringIO()
        with mock.patch.dict(sys.modules, {"cv2": None}):
            rc = run_hdf5_viewer_cli(args, stderr=stderr)

        self.assertEqual(rc, 1)
        self.assertIn("OpenCV cv2 is required", stderr.getvalue())


def _write_single_arm_episode(path: Path, *, length: int = 4) -> None:
    assert h5py is not None and np is not None
    pose = np.zeros((length, 7), dtype=np.float32)
    pose[:, 0] = np.linspace(0.0, 0.03, length)
    pose[:, 6] = 1.0
    action = np.zeros((length, 8), dtype=np.float32)
    action[:, :7] = pose
    action[:, 0] += 0.01
    action[:, 7] = 0.3
    gripper = np.zeros((length, 2), dtype=np.float32)
    gripper[:, 0] = 0.1

    with h5py.File(path, "w") as handle:
        handle.create_dataset("action", data=action)
        handle.create_dataset("timestamp", data=np.arange(length, dtype=np.float64) / 30.0)
        obs = handle.create_group("observations")
        obs.create_dataset("pose", data=pose)
        obs.create_dataset("gripper", data=gripper)
        images = obs.create_group("images")
        images.create_dataset("fisheye", data=_image_stack(length))


def _write_bimanual_episode(path: Path, *, length: int = 4) -> None:
    assert h5py is not None and np is not None
    with h5py.File(path, "w") as handle:
        handle.attrs["arm_names"] = "left,right"
        handle.create_dataset("timestamp", data=np.arange(length, dtype=np.float64) / 30.0)
        obs = handle.create_group("observations")
        for side, sign in (("left", 1.0), ("right", -1.0)):
            group = obs.create_group(side)
            pose = np.zeros((length, 7), dtype=np.float32)
            pose[:, 0] = sign * np.linspace(0.0, 0.03, length)
            pose[:, 1] = sign * 0.01
            pose[:, 6] = 1.0
            action = np.zeros((length, 8), dtype=np.float32)
            action[:, :7] = pose
            action[:, 0] += sign * 0.01
            action[:, 7] = 0.4
            gripper = np.zeros((length, 2), dtype=np.float32)
            gripper[:, 0] = 0.2
            group.create_dataset("pose", data=pose)
            group.create_dataset("action", data=action)
            group.create_dataset("gripper", data=gripper)
            images = group.create_group("images")
            images.create_dataset("realsense_color", data=_image_stack(length))


def _image_stack(length: int) -> np.ndarray:
    assert np is not None
    data = np.zeros((length, 10, 12, 3), dtype=np.uint8)
    for index in range(length):
        data[index, :, :, 0] = 20 + index
        data[index, :, :, 1] = 50
        data[index, :, :, 2] = 100
    return data


if __name__ == "__main__":
    unittest.main()
