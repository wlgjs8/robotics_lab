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
        DEFAULT_ACTION_FRAME,
        FLOW_ACTION_DIM,
        FlowHdf5Dataset,
        compute_dataset_statistics,
        decode_hdf5_image_value,
        load_flow_episode_index,
        pose_compose_local,
        pose_delta_local,
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

    def test_center_square_crop_changes_nonsquare_image_before_resize(self) -> None:
        raw = _image_bytes("png", index=0)

        full = decode_hdf5_image_value(raw, image_size=8, image_crop="none")
        cropped = decode_hdf5_image_value(raw, image_size=8, image_crop="center_square")

        self.assertEqual(full.shape, (3, 8, 8))
        self.assertEqual(cropped.shape, (3, 8, 8))
        self.assertGreater(float(np.abs(full - cropped).sum()), 0.0)

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

    def test_single_arm_missing_gripper_loads_with_zero_gripper(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "episode_001.hdf5"
            self._write_pika_episode(path, image_count=1, with_gripper=False)

            index = load_flow_episode_index(path)
            self.assertEqual(index.format_name, "pika_umi_single_arm")
            self.assertTrue(np.allclose(index.left_gripper, 0.0))
            self.assertEqual(index.action_left_gripper.shape, (index.length,))

            dataset = FlowHdf5Dataset(path, action_horizon=2, image_size=8, normalize=False)
            sample = dataset.raw_sample(0)
            self.assertEqual(sample["action_chunk"].shape, (2, 14))
            self.assertTrue(np.allclose(sample["proprio"][6], 0.0))

    def test_single_arm_gripper_shapes_use_first_vector_without_zeroing(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            expected = np.asarray([0.1, 0.2, 0.3, 0.4], dtype=np.float32)
            cases = {
                "gripper_1d.hdf5": expected,
                "gripper_n1.hdf5": expected.reshape(-1, 1),
                "gripper_n2.hdf5": np.stack([expected, expected + 1.0], axis=1),
            }

            for filename, gripper_data in cases.items():
                path = root / filename
                self._write_pika_episode(path, image_count=1, gripper_data=gripper_data)

                index = load_flow_episode_index(path)

                self.assertEqual(index.length, len(expected))
                np.testing.assert_allclose(index.left_gripper, expected)

    def test_single_arm_empty_gripper_uses_zero_vector_without_shortening_episode(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "episode_001.hdf5"
            self._write_pika_episode(path, image_count=1, gripper_data=np.asarray([], dtype=np.float32))

            index = load_flow_episode_index(path)

            self.assertEqual(index.length, 4)
            self.assertTrue(np.allclose(index.left_gripper, 0.0))

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
            self.assertEqual(stats["proprio_action_frame"], DEFAULT_ACTION_FRAME)
            self.assertEqual(stats["image_crop"], "none")
            self.assertAlmostEqual(float(stats["dt_mean_sec"]), 1.0 / 30.0)
            self.assertAlmostEqual(float(stats["dt_p50_sec"]), 1.0 / 30.0)
            self.assertTrue(np.isfinite(sample["proprio"]).all())
            self.assertTrue(np.isfinite(sample["action_chunk"]).all())

    def test_stand_action_frame_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "episode_001.hdf5"
            self._write_pika_episode(path, image_count=1)
            # The world-frame "stand" representation was removed; only ee_local remains.
            with self.assertRaises(ValueError):
                FlowHdf5Dataset(
                    path,
                    action_horizon=2,
                    image_size=8,
                    normalize=False,
                    action_frame="stand",
                )

    def test_action_chunk_ee_local_uses_body_frame_translation(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "episode_001.hdf5"
            self._write_pika_episode(path, image_count=1)
            q_yaw_90 = _quat_from_axis_angle([0.0, 0.0, 1.0], np.pi / 2.0)
            with h5py.File(path, "r+") as handle:
                handle["observations/pose"][:, 3:7] = q_yaw_90
                handle["action"][:, 3:7] = q_yaw_90

            dataset = FlowHdf5Dataset(
                path,
                action_horizon=2,
                image_size=8,
                normalize=False,
                action_frame="ee_local",
            )
            sample = dataset.raw_sample(0)

            np.testing.assert_allclose(sample["action_chunk"][0, 0:3], [0.0, -0.01, 0.0], atol=1e-6)
            np.testing.assert_allclose(sample["action_chunk"][1, 0:3], [0.0, -0.01, 0.0], atol=1e-6)

    def test_pose_delta_local_rotates_translation_into_reference_body(self) -> None:
        q_ref = _quat_from_axis_angle([0.0, 0.0, 1.0], np.pi / 2.0)
        reference = np.asarray([0.0, 0.0, 0.0, *q_ref], dtype=np.float32)
        target = np.asarray([1.0, 0.0, 0.0, *q_ref], dtype=np.float32)

        local_delta = pose_delta_local(reference, target)

        # A +x world displacement viewed from a +90deg-yaw body frame is -y.
        np.testing.assert_allclose(local_delta[:3], [0.0, -1.0, 0.0], atol=1e-6)
        np.testing.assert_allclose(local_delta[3:6], [0.0, 0.0, 0.0], atol=1e-6)

    def test_pose_compose_local_round_trips_pose_delta_local(self) -> None:
        rng = np.random.default_rng(42)
        for _ in range(25):
            q_ref = _quat_from_axis_angle(rng.normal(size=3).tolist(), float(rng.uniform(-2.5, 2.5)))
            reference = np.asarray(
                [
                    float(rng.normal()),
                    float(rng.normal()),
                    float(rng.normal()),
                    *q_ref,
                ],
                dtype=np.float32,
            )
            delta = np.asarray(
                [
                    float(rng.uniform(-0.05, 0.05)),
                    float(rng.uniform(-0.05, 0.05)),
                    float(rng.uniform(-0.05, 0.05)),
                    float(rng.uniform(-0.2, 0.2)),
                    float(rng.uniform(-0.2, 0.2)),
                    float(rng.uniform(-0.2, 0.2)),
                ],
                dtype=np.float32,
            )

            target = pose_compose_local(reference, delta)
            round_trip = pose_delta_local(reference, target)

            np.testing.assert_allclose(round_trip, delta, atol=1e-6)

    def test_flow_dataset_ee_local_frame_invariance(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            original = root / "episode_original.hdf5"
            transformed = root / "episode_transformed.hdf5"
            left_pose, right_pose = _synthetic_bimanual_pose_tracks()
            _write_bimanual_pose_episode(original, left_pose=left_pose, right_pose=right_pose)

            q_transform = _quat_from_axis_angle([0.3, -0.4, 0.8], 1.1)
            t_transform = np.asarray([0.7, -0.2, 0.4], dtype=np.float64)
            _write_bimanual_pose_episode(
                transformed,
                left_pose=_transform_pose_track(left_pose, q_transform, t_transform),
                right_pose=_transform_pose_track(right_pose, q_transform, t_transform),
            )

            original_dataset = FlowHdf5Dataset(
                original,
                action_horizon=3,
                image_size=8,
                normalize=False,
                action_frame="ee_local",
            )
            transformed_dataset = FlowHdf5Dataset(
                transformed,
                action_horizon=3,
                image_size=8,
                normalize=False,
                action_frame="ee_local",
            )
            original_sample = original_dataset.raw_sample(0)
            transformed_sample = transformed_dataset.raw_sample(0)

            np.testing.assert_allclose(
                transformed_sample["proprio"],
                original_sample["proprio"],
                atol=1e-5,
            )
            np.testing.assert_allclose(
                transformed_sample["action_chunk"],
                original_sample["action_chunk"],
                atol=1e-5,
            )

    def _write_pika_episode(
        self,
        path: Path,
        *,
        image_count: int,
        with_gripper: bool = True,
        gripper_data: np.ndarray | None = None,
    ) -> None:
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
            if with_gripper:
                obs.create_dataset("gripper", data=gripper if gripper_data is None else gripper_data)
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
    arr[:, :, 0] = 20 + index + np.arange(10, dtype=np.uint8)[None, :]
    arr[:, :, 1] = 40 + np.arange(12, dtype=np.uint8)[:, None]
    arr[:, :, 2] = 80
    image = Image.fromarray(arr, mode="RGB")
    buffer = io.BytesIO()
    image.save(buffer, format="JPEG" if suffix == "jpeg" else "PNG")
    return buffer.getvalue()


def _quat_from_axis_angle(axis: list[float], angle: float) -> np.ndarray:
    axis_array = np.asarray(axis, dtype=np.float64)
    axis_array = axis_array / np.linalg.norm(axis_array)
    half = angle * 0.5
    return np.asarray([*(axis_array * np.sin(half)), np.cos(half)], dtype=np.float64)


def _synthetic_bimanual_pose_tracks() -> tuple[np.ndarray, np.ndarray]:
    length = 5
    left = np.zeros((length, 7), dtype=np.float32)
    right = np.zeros((length, 7), dtype=np.float32)
    for index in range(length):
        left[index, :3] = [0.10 + 0.015 * index, -0.04 + 0.005 * index, 0.30 + 0.002 * index]
        right[index, :3] = [-0.08 - 0.010 * index, 0.05 + 0.004 * index, 0.28 - 0.003 * index]
        left[index, 3:7] = _quat_from_axis_angle([0.2, 0.0, 1.0], 0.15 + 0.05 * index)
        right[index, 3:7] = _quat_from_axis_angle([0.0, 0.3, 1.0], -0.10 + 0.04 * index)
    return left, right


def _write_bimanual_pose_episode(path: Path, *, left_pose: np.ndarray, right_pose: np.ndarray) -> None:
    assert h5py is not None and np is not None
    length = int(min(len(left_pose), len(right_pose)))
    timestamps = np.arange(length, dtype=np.float64) / 30.0
    with h5py.File(path, "w") as handle:
        handle.attrs["n_arms"] = 2
        handle.attrs["arm_names"] = "left,right"
        handle.create_dataset("timestamp", data=timestamps)
        obs = handle.create_group("observations")
        for side, pose, offset in (
            ("left", left_pose[:length], 0.0),
            ("right", right_pose[:length], 0.1),
        ):
            group = obs.create_group(side)
            gripper = np.linspace(0.1 + offset, 0.4 + offset, length, dtype=np.float32)
            action = np.zeros((length, 8), dtype=np.float32)
            action[:, :7] = pose
            action[:, 7] = gripper + 0.05
            group.create_dataset("pose", data=pose.astype(np.float32))
            group.create_dataset("gripper", data=np.stack([gripper, gripper + 0.02], axis=1))
            group.create_dataset("action", data=action)


def _transform_pose_track(
    poses: np.ndarray,
    q_transform: np.ndarray,
    t_transform: np.ndarray,
) -> np.ndarray:
    transformed = np.zeros_like(poses, dtype=np.float32)
    q_transform = _normalize_quat(q_transform)
    for index, pose in enumerate(np.asarray(poses, dtype=np.float64)):
        transformed[index, :3] = _quat_rotate_vector(q_transform, pose[:3]) + t_transform
        transformed[index, 3:7] = _normalize_quat(_quat_multiply(q_transform, pose[3:7]))
    return transformed


def _normalize_quat(q: np.ndarray) -> np.ndarray:
    quat = np.asarray(q, dtype=np.float64)
    return quat / np.linalg.norm(quat)


def _quat_inverse(q: np.ndarray) -> np.ndarray:
    return np.asarray([-q[0], -q[1], -q[2], q[3]], dtype=np.float64)


def _quat_rotate_vector(q: np.ndarray, vector: np.ndarray) -> np.ndarray:
    quat = _normalize_quat(q)
    value = np.asarray(vector, dtype=np.float64).reshape(3)
    pure = np.asarray([value[0], value[1], value[2], 0.0], dtype=np.float64)
    rotated = _quat_multiply(_quat_multiply(quat, pure), _quat_inverse(quat))
    return rotated[:3]


def _quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.asarray(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ],
        dtype=np.float64,
    )


if __name__ == "__main__":
    unittest.main()
