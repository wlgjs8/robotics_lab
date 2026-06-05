from __future__ import annotations

import io
import tempfile
import unittest
from pathlib import Path

try:
    import h5py
    import numpy as np
    from PIL import Image

    from policy_runner.flow_dataset import (
        FLOW_ACTION_DIM,
        FlowHdf5Dataset,
        compute_dataset_statistics,
        decode_hdf5_image_value,
        load_flow_episode_index,
    )
except ModuleNotFoundError:
    h5py = None
    np = None
    Image = None


@unittest.skipIf(h5py is None or np is None or Image is None, "flow dataset extras not installed")
class FlowHdf5DatasetTest(unittest.TestCase):
    def test_uploaded_pika_episode_loads_when_present(self) -> None:
        episode = Path("episode_002.hdf5")
        if not episode.exists():
            self.skipTest("uploaded episode_002.hdf5 is not present")

        index = load_flow_episode_index(episode)
        self.assertEqual(index.format_name, "pika_umi_single_arm")
        self.assertEqual(index.arm_mask.tolist(), [1.0, 0.0])
        self.assertGreater(index.length, 0)
        self.assertIn("realsense_color", index.camera_paths)

        dataset = FlowHdf5Dataset(episode, action_horizon=4, image_size=16, normalize=False)
        sample = dataset.raw_sample(0)
        self.assertEqual(sample["action_chunk"].shape, (4, FLOW_ACTION_DIM))
        self.assertEqual(sample["action_mask"].shape, (4, FLOW_ACTION_DIM))
        self.assertTrue(np.allclose(sample["action_mask"][:, 7:14], 0.0))
        self.assertTrue(np.allclose(sample["action_chunk"][:, 7:14], 0.0))
        self.assertEqual(sample["images"].shape[1:], (3, 16, 16))
        self.assertGreaterEqual(int(sample["image_decode_count"]), 1)

    def test_image_decode_works_for_jpeg_and_png_hdf5_datasets(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "episode_001.hdf5"
            self._write_pika_episode(path, image_count=2)

            dataset = FlowHdf5Dataset(path, action_horizon=2, image_size=8, normalize=False)
            sample = dataset.raw_sample(0)

            self.assertEqual(sample["images"].shape, (2, 3, 8, 8))
            self.assertEqual(int(sample["image_decode_count"]), 2)
            self.assertEqual(int(sample["missing_camera_count"]), 0)
            self.assertTrue(np.isfinite(sample["images"]).all())

            with h5py.File(path, "r") as handle:
                jpeg = decode_hdf5_image_value(handle["observations/images/jpeg_cam"][0], image_size=8)
                png = decode_hdf5_image_value(handle["observations/images/png_cam"][0], image_size=8)
            self.assertEqual(jpeg.shape, (3, 8, 8))
            self.assertEqual(png.shape, (3, 8, 8))

    def test_single_arm_maps_to_left_and_zero_masks_right_arm(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "episode_001.hdf5"
            self._write_pika_episode(path, image_count=1)

            dataset = FlowHdf5Dataset(path, action_horizon=3, image_size=8, normalize=False)
            sample = dataset.raw_sample(0)

            self.assertEqual(sample["arm_mask"].tolist(), [1.0, 0.0])
            self.assertEqual(sample["action_chunk"].shape, (3, 14))
            self.assertTrue(np.all(sample["action_mask"][:, :7] == 1.0))
            self.assertTrue(np.all(sample["action_mask"][:, 7:] == 0.0))
            self.assertTrue(np.allclose(sample["action_chunk"][:, 7:], 0.0))

    def test_bimanual_pika_episode_maps_both_arms(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "episode_001.hdf5"
            self._write_bimanual_pika_episode(path)

            index = load_flow_episode_index(path)
            self.assertEqual(index.format_name, "pika_umi_bimanual")
            self.assertEqual(index.arm_mask.tolist(), [1.0, 1.0])
            self.assertIn("left_realsense_color", index.camera_paths)
            self.assertIn("right_realsense_color", index.camera_paths)

            dataset = FlowHdf5Dataset(path, action_horizon=3, image_size=8, normalize=False)
            sample = dataset.raw_sample(0)

            self.assertEqual(sample["action_chunk"].shape, (3, 14))
            self.assertTrue(np.all(sample["action_mask"] == 1.0))
            self.assertEqual(sample["images"].shape, (2, 3, 8, 8))
            self.assertEqual(int(sample["image_decode_count"]), 2)
            self.assertGreater(float(np.abs(sample["action_chunk"][:, :7]).sum()), 0.0)
            self.assertGreater(float(np.abs(sample["action_chunk"][:, 7:]).sum()), 0.0)

    def test_dataset_statistics_normalize_action_chunks(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "episode_001.hdf5"
            self._write_pika_episode(path, image_count=1)

            raw = FlowHdf5Dataset(path, action_horizon=2, image_size=8, normalize=False)
            stats = compute_dataset_statistics(raw)
            normalized = FlowHdf5Dataset(
                path,
                action_horizon=2,
                image_size=8,
                camera_names=list(stats["camera_names"]),
                stats=stats,
                normalize=True,
            )
            sample = normalized[0]

            self.assertEqual(stats["schema"], "robotics_lab.policy_runner.flow_matching.v1.dataset_stats")
            self.assertEqual(stats["action_dim"], 14)
            self.assertTrue(np.isfinite(sample["proprio"]).all())
            self.assertTrue(np.isfinite(sample["action_chunk"]).all())

    def _write_pika_episode(self, path: Path, *, image_count: int) -> None:
        assert h5py is not None and np is not None and Image is not None
        length = 4
        pose = np.zeros((length, 7), dtype=np.float32)
        pose[:, 0] = np.linspace(0.0, 0.03, length)
        pose[:, 6] = 1.0
        action = np.zeros((length, 8), dtype=np.float32)
        action[:, :7] = pose
        action[:, 0] += 0.01
        action[:, 7] = np.linspace(0.0, 1.0, length)
        gripper = np.zeros((length, 2), dtype=np.float32)
        gripper[:, 0] = 0.1

        with h5py.File(path, "w") as handle:
            handle.create_dataset("action", data=action)
            handle.create_dataset("timestamp", data=np.arange(length, dtype=np.float64) / 30.0)
            obs = handle.create_group("observations")
            obs.create_dataset("pose", data=pose)
            obs.create_dataset("gripper", data=gripper)
            images = obs.create_group("images")
            if image_count >= 1:
                self._write_vlen_image_dataset(images, "jpeg_cam", length, suffix="jpeg")
            if image_count >= 2:
                self._write_vlen_image_dataset(images, "png_cam", length, suffix="png")

    def _write_bimanual_pika_episode(self, path: Path) -> None:
        assert h5py is not None and np is not None
        length = 4
        timestamps = np.arange(length, dtype=np.float64) / 30.0
        with h5py.File(path, "w") as handle:
            handle.attrs["n_arms"] = 2
            handle.attrs["arm_names"] = "left,right"
            handle.create_dataset("timestamp", data=timestamps)
            obs = handle.create_group("observations")
            for side, sign in (("left", 1.0), ("right", -1.0)):
                group = obs.create_group(side)
                pose = np.zeros((length, 7), dtype=np.float32)
                pose[:, 0] = sign * np.linspace(0.0, 0.03, length)
                pose[:, 1] = sign * 0.01
                pose[:, 6] = 1.0
                gripper = np.zeros((length, 2), dtype=np.float32)
                gripper[:, 0] = np.linspace(0.1, 0.4, length)
                gripper[:, 1] = np.linspace(0.2, 0.5, length)
                action = np.zeros((length, 8), dtype=np.float32)
                action[:, :7] = pose
                action[:, 0] += sign * 0.01
                action[:, 7] = gripper[:, 1] + 0.1
                group.create_dataset("pose", data=pose)
                group.create_dataset("gripper", data=gripper)
                group.create_dataset("command", data=np.zeros(length, dtype=np.int8))
                group.create_dataset("action", data=action)
                images = group.create_group("images")
                self._write_vlen_image_dataset(images, "realsense_color", length, suffix="jpeg")

    def _write_vlen_image_dataset(self, group: h5py.Group, name: str, length: int, *, suffix: str) -> None:
        assert h5py is not None and np is not None
        dtype = h5py.vlen_dtype(np.dtype("uint8"))
        dataset = group.create_dataset(name, shape=(length,), dtype=dtype)
        for index in range(length):
            dataset[index] = np.frombuffer(_image_bytes(suffix, index=index), dtype=np.uint8)


def _image_bytes(suffix: str, *, index: int) -> bytes:
    assert Image is not None and np is not None
    arr = np.zeros((12, 10, 3), dtype=np.uint8)
    arr[:, :, 0] = 20 + index
    arr[:, :, 1] = 40
    arr[:, :, 2] = 80
    image = Image.fromarray(arr, mode="RGB")
    buffer = io.BytesIO()
    image.save(buffer, format="JPEG" if suffix == "jpeg" else "PNG")
    return buffer.getvalue()


if __name__ == "__main__":
    unittest.main()
