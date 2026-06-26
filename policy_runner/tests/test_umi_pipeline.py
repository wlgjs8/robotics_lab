from __future__ import annotations

import builtins
import io
import tempfile
import unittest
from pathlib import Path

try:
    import h5py
    import numpy as np
    from PIL import Image

    from policy_runner.flow_dataset import FlowHdf5Dataset, load_flow_episode_index
    from policy_runner.umi_pipeline import (
        UMI_IMPORT_MANIFEST_SCHEMA,
        convert_umi_episode,
        import_umi_session,
        load_umi_retarget_config,
    )
except ModuleNotFoundError:
    h5py = None
    np = None
    Image = None


@unittest.skipIf(h5py is None or np is None or Image is None, "UMI HDF5 test extras not installed")
class UmiPipelineTest(unittest.TestCase):
    def test_import_writes_manifest_report_and_training_links(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            raw_dir = root / "raw"
            output_dir = root / "imported"
            raw_dir.mkdir()
            episode = raw_dir / "episode_001.hdf5"
            retarget_path = root / "umi_retarget.yaml"
            _write_bimanual_umi_episode(episode)
            _write_retarget_config(retarget_path)

            manifest = import_umi_session(
                raw_dir,
                output_dir,
                task="synthetic bimanual task",
                left_device="LEFT_SERIAL",
                right_device="RIGHT_SERIAL",
                retarget_config=retarget_path,
            )

            self.assertEqual(manifest["schema"], UMI_IMPORT_MANIFEST_SCHEMA)
            self.assertTrue((output_dir / "manifest.json").exists())
            self.assertTrue((output_dir / "conversion_report.md").exists())
            self.assertTrue((output_dir / "episode_001.hdf5").exists())
            self.assertEqual(manifest["aggregate"]["episode_count"], 1)
            self.assertEqual(manifest["retarget"]["status"], "configured_estimate")
            self.assertEqual(manifest["retarget"]["source_pose_frame"], "steamvr_world")
            self.assertNotIn("target_pose_frame", manifest["retarget"])
            self.assertEqual(manifest["episodes"][0]["arm_mask"], [1.0, 1.0])
            self.assertIn("left_wrist_rgb", manifest["episodes"][0]["camera_names"])
            self.assertEqual(
                manifest["episodes"][0]["quality_gates"]["camera_decode_success"]["failed_frame_count"],
                0,
            )

            # "stand" action_frame is gone; ee_local is the only representation.
            with self.assertRaises(ValueError):
                FlowHdf5Dataset(
                    output_dir,
                    action_horizon=2,
                    image_size=8,
                    normalize=False,
                    action_frame="stand",
                )
            ee_local_dataset = FlowHdf5Dataset(
                output_dir,
                action_horizon=2,
                image_size=8,
                normalize=False,
                action_frame="ee_local",
            )
            ee_local_sample = ee_local_dataset.raw_sample(0)
            self.assertEqual(ee_local_sample["images"].shape[0], 4)
            self.assertEqual(int(ee_local_sample["missing_camera_count"]), 0)
            self.assertEqual(ee_local_dataset.action_frame, "ee_local")

    def test_missing_right_arm_maps_arm_mask(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            raw_dir = root / "raw"
            output_dir = root / "imported"
            raw_dir.mkdir()
            episode = raw_dir / "episode_left_only.hdf5"
            retarget_path = root / "umi_retarget.yaml"
            _write_bimanual_umi_episode(episode, include_right=False)
            _write_retarget_config(retarget_path)

            import_umi_session(
                raw_dir,
                output_dir,
                task="left only synthetic task",
                retarget_config=retarget_path,
            )

            index = load_flow_episode_index(output_dir / "episode_left_only.hdf5")
            self.assertEqual(index.format_name, "pika_umi_bimanual")
            self.assertEqual(index.arm_mask.tolist(), [1.0, 0.0])

    def test_optional_action_dataset_is_accepted(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            raw_dir = root / "raw"
            output_dir = root / "imported"
            raw_dir.mkdir()
            episode = raw_dir / "episode_no_action.hdf5"
            retarget_path = root / "umi_retarget.yaml"
            _write_bimanual_umi_episode(episode, include_action=False)
            _write_retarget_config(retarget_path)

            import_umi_session(
                raw_dir,
                output_dir,
                task="optional action synthetic task",
                retarget_config=retarget_path,
            )

            index = load_flow_episode_index(output_dir / "episode_no_action.hdf5")
            self.assertEqual(index.format_name, "pika_umi_bimanual")
            self.assertEqual(index.arm_mask.tolist(), [1.0, 1.0])
            self.assertIsNotNone(index.action_left_pose)
            self.assertIsNotNone(index.action_right_pose)

    def test_convert_robotics_lab_preserves_timestamps_cameras_and_retarget_hash(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / "episode_001.hdf5"
            output = root / "episode_robotics_lab.hdf5"
            retarget_path = root / "umi_retarget.yaml"
            _write_bimanual_umi_episode(source)
            _write_retarget_config(retarget_path)
            retarget = load_umi_retarget_config(retarget_path)

            manifest = convert_umi_episode(
                source,
                output,
                output_format="robotics_lab_dual_arm",
                retarget_config=retarget_path,
            )

            self.assertTrue((root / "manifest.json").exists())
            self.assertTrue((root / "conversion_report.md").exists())
            self.assertEqual(manifest["retarget"]["sha256"], retarget.sha256)
            with h5py.File(source, "r") as src, h5py.File(output, "r") as dst:
                self.assertEqual(dst.attrs["schema"], "robotics_lab.episode.v1")
                self.assertEqual(dst.attrs["retarget_config_hash"], retarget.sha256)
                np.testing.assert_array_equal(dst["observations/timestamp"][:], src["timestamp"][:])
                self.assertEqual(
                    sorted(dst["observations/images"].keys()),
                    ["left_overhead_rgb", "left_wrist_rgb", "right_overhead_rgb", "right_wrist_rgb"],
                )
                # Action is the absolute tool-offset target pose; ee_local deltas are
                # derived at load.
                self.assertIn("target_pose_left", dst["action"])
                self.assertEqual(dst["action/target_pose_left"].shape, (5, 7))

            index = load_flow_episode_index(output)
            self.assertEqual(index.format_name, "robotics_lab_dual_arm")
            self.assertIn("left_wrist_rgb", index.camera_paths)
            self.assertIn("right_wrist_rgb", index.camera_paths)
            self.assertTrue(
                any(
                    "retarget_status_not_physical_rollout_ready" in item
                    for item in manifest["aggregate"]["deployment_blockers"]
                )
            )

    def test_convert_robotics_lab_without_action_writes_target_pose(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / "episode_no_action.hdf5"
            output = root / "episode_robotics_lab.hdf5"
            retarget_path = root / "umi_retarget.yaml"
            _write_bimanual_umi_episode(source, include_action=False)
            _write_retarget_config(retarget_path)

            convert_umi_episode(
                source,
                output,
                output_format="robotics_lab_dual_arm",
                retarget_config=retarget_path,
            )

            with h5py.File(output, "r") as dst:
                # Action is the absolute tool-offset target pose.
                left_target = dst["action/target_pose_left"][:]
                right_target = dst["action/target_pose_right"][:]
                self.assertEqual(left_target.shape, (5, 7))
                self.assertEqual(right_target.shape, (5, 7))
                # With no action group the target pose falls back to the observed pose.
                np.testing.assert_allclose(
                    left_target, dst["observations/tcp_stand_left"][:], atol=1e-7
                )

    def test_configured_estimate_retarget_status_blocks_require_measured(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / "episode_001.hdf5"
            output = root / "episode_robotics_lab.hdf5"
            retarget_path = root / "umi_retarget.yaml"
            _write_bimanual_umi_episode(source)
            _write_retarget_config(retarget_path, status="configured_estimate")

            with self.assertRaisesRegex(ValueError, "requires retarget config status=measured or accepted"):
                convert_umi_episode(
                    source,
                    output,
                    output_format="robotics_lab_dual_arm",
                    retarget_config=retarget_path,
                    require_measured_retarget=True,
                )

    def test_accepted_retarget_status_passes_require_measured_and_clears_blocker(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = root / "episode_001.hdf5"
            output = root / "episode_robotics_lab.hdf5"
            retarget_path = root / "umi_retarget.yaml"
            _write_bimanual_umi_episode(source)
            _write_retarget_config(retarget_path, status="accepted")

            manifest = convert_umi_episode(
                source,
                output,
                output_format="robotics_lab_dual_arm",
                retarget_config=retarget_path,
                require_measured_retarget=True,
            )

            self.assertEqual(manifest["retarget"]["status"], "accepted")
            self.assertEqual(manifest["retarget"]["source_pose_frame"], "steamvr_world")
            self.assertNotIn("target_pose_frame", manifest["retarget"])
            self.assertFalse(
                any(
                    "retarget_status_not_physical_rollout_ready" in item
                    for item in manifest["aggregate"]["deployment_blockers"]
                )
            )

    def test_unknown_retarget_status_is_rejected_by_config_loader(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            retarget_path = Path(tmp) / "umi_retarget.yaml"
            _write_retarget_config(retarget_path, status="unknown")

            with self.assertRaisesRegex(ValueError, "status must be one of"):
                load_umi_retarget_config(retarget_path)

    def test_import_does_not_import_live_hardware_sdk_modules(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            raw_dir = root / "raw"
            output_dir = root / "imported"
            raw_dir.mkdir()
            _write_bimanual_umi_episode(raw_dir / "episode_001.hdf5")
            retarget_path = root / "umi_retarget.yaml"
            _write_retarget_config(retarget_path)

            original_import = builtins.__import__
            blocked: list[str] = []

            def guarded_import(name, globals=None, locals=None, fromlist=(), level=0):
                if name in {"umi", "umi_sdk", "pika"} or name.startswith(("umi.", "umi_sdk.", "pika.")):
                    blocked.append(name)
                    raise AssertionError(f"unexpected hardware SDK import: {name}")
                return original_import(name, globals, locals, fromlist, level)

            builtins.__import__ = guarded_import
            try:
                import_umi_session(
                    raw_dir,
                    output_dir,
                    task="no hardware import",
                    retarget_config=retarget_path,
                )
            finally:
                builtins.__import__ = original_import

            self.assertEqual(blocked, [])


def _write_bimanual_umi_episode(
    path: Path,
    *,
    include_right: bool = True,
    include_action: bool = True,
) -> None:
    assert h5py is not None and np is not None
    length = 5
    with h5py.File(path, "w") as handle:
        handle.attrs["schema"] = "robotics_lab.umi_episode.v1"
        handle.attrs["arm_names"] = "left,right"
        handle.attrs["pose_format"] = "x,y,z,qx,qy,qz,qw"
        handle.attrs["pose_frame"] = "steamvr_world"
        handle.attrs["capture_hz"] = 30
        handle.attrs["retarget_status"] = "configured_estimate"
        handle.attrs["umi_device_serials"] = '{"left":"LEFT_SERIAL","right":"RIGHT_SERIAL"}'
        handle.create_dataset("timestamp", data=np.arange(length, dtype=np.float64) / 30.0)
        obs = handle.create_group("observations")
        sides = [("left", 1.0)]
        if include_right:
            sides.append(("right", -1.0))
        for side, sign in sides:
            group = obs.create_group(side)
            pose = np.zeros((length, 7), dtype=np.float32)
            pose[:, 0] = sign * np.linspace(0.1, 0.14, length)
            pose[:, 1] = sign * 0.02
            pose[:, 2] = 0.3
            pose[:, 6] = 1.0
            gripper = np.zeros((length, 2), dtype=np.float32)
            gripper[:, 0] = np.linspace(0.1, 0.5, length)
            gripper[:, 1] = np.linspace(0.2, 0.6, length)
            group.create_dataset("pose", data=pose)
            group.create_dataset("gripper", data=gripper)
            if include_action:
                action = np.zeros((length, 8), dtype=np.float32)
                action[:, :7] = pose
                action[:, 0] += sign * 0.01
                action[:, 7] = gripper[:, 1]
                group.create_dataset("action", data=action)
            images = group.create_group("images")
            _write_vlen_image_dataset(images, "wrist_rgb", length)
            _write_vlen_image_dataset(images, "overhead_rgb", length)


def _write_retarget_config(path: Path, *, status: str = "configured_estimate") -> None:
    path.write_text(
        "\n".join(
            [
                "schema: robotics_lab.umi_retarget.v1",
                f"status: {status}",
                "source_pose_frame: steamvr_world",
                "left:",
                "  T_tcp_umi_gripper: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]",
                "  gripper_open_close_units: percent",
                "right:",
                "  T_tcp_umi_gripper: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]",
                "  gripper_open_close_units: percent",
                "quality:",
                "  measured_date: null",
                "  max_translation_error_m: null",
                "  max_rotation_error_rad: null",
            ]
        )
        + "\n",
        encoding="utf-8",
    )


def _write_vlen_image_dataset(group: h5py.Group, name: str, length: int) -> None:
    assert h5py is not None and np is not None
    dtype = h5py.vlen_dtype(np.dtype("uint8"))
    dataset = group.create_dataset(name, shape=(length,), dtype=dtype)
    dataset.attrs["encoding"] = "jpeg"
    for index in range(length):
        dataset[index] = np.frombuffer(_image_bytes(index=index), dtype=np.uint8)


def _image_bytes(*, index: int) -> bytes:
    assert Image is not None and np is not None
    arr = np.zeros((10, 12, 3), dtype=np.uint8)
    arr[:, :, 0] = 20 + index
    arr[:, :, 1] = 80
    arr[:, :, 2] = 120
    buffer = io.BytesIO()
    Image.fromarray(arr, mode="RGB").save(buffer, format="JPEG")
    return buffer.getvalue()


if __name__ == "__main__":
    unittest.main()
