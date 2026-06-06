from __future__ import annotations

import io
import tempfile
import unittest
from pathlib import Path

try:
    import h5py
    import numpy as np
    from PIL import Image

    from policy_runner.dataset_manifest import DatasetManifest
    from policy_runner.flow_dataset import FlowHdf5Dataset
    from policy_runner.hdf5_audit import audit_hdf5_episodes
except ModuleNotFoundError:
    h5py = None
    np = None
    Image = None


@unittest.skipIf(h5py is None or np is None or Image is None, "HDF5 audit extras not installed")
class Hdf5AuditTest(unittest.TestCase):
    def test_uploaded_episode_audits_as_pika_single_arm_when_present(self) -> None:
        episode = Path("episode_002.hdf5")
        if not episode.exists():
            self.skipTest("uploaded episode_002.hdf5 is not present")

        report = audit_hdf5_episodes(episode)
        audited = report["episodes"][0]

        self.assertEqual(audited["format_name"], "pika_umi_single_arm")
        self.assertEqual(audited["length"], 140)
        self.assertEqual(audited["arm_mask"], [1.0, 0.0])
        self.assertEqual(audited["pose_frame"], "steamvr_world")
        self.assertEqual(audited["pose_format"], "x,y,z,qx,qy,qz,qw")
        self.assertEqual(
            audited["camera_names"],
            ["fisheye", "realsense_color", "realsense_depth"],
        )
        self.assertEqual(audited["camera_encodings"]["realsense_depth"], "png16")
        self.assertTrue(_contains(audited["warnings"], "single_arm_default_left"))
        self.assertTrue(_contains(audited["warnings"], "action_matches_observation_pose"))
        self.assertTrue(_contains(audited["deployment_blockers"], "retarget_required"))

    def test_jpeg_and_png16_varlen_image_decode_audits_cleanly(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "episode_001.hdf5"
            _write_pika_episode(
                path,
                image_specs={
                    "fisheye": "jpeg",
                    "realsense_depth": "png16",
                },
            )

            report = audit_hdf5_episodes(path)
            audited = report["episodes"][0]

            self.assertEqual(audited["format_name"], "pika_umi_single_arm")
            self.assertEqual(audited["camera_encodings"]["fisheye"], "jpeg")
            self.assertEqual(audited["camera_encodings"]["realsense_depth"], "png16")
            self.assertFalse(_contains(audited["warnings"], "corrupt_images"))
            self.assertFalse(_contains(audited["warnings"], "unsupported_image_encoding"))

    def test_png16_varlen_encoding_is_inferred_without_attr(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "episode_001.hdf5"
            _write_pika_episode(
                path,
                image_specs={"realsense_depth": "png16"},
                image_encoding_attrs=False,
            )

            report = audit_hdf5_episodes(path)
            audited = report["episodes"][0]

            self.assertEqual(audited["camera_encodings"]["realsense_depth"], "png16")
            self.assertFalse(_contains(audited["warnings"], "unsupported_image_encoding"))

    def test_missing_gripper_field_is_audited_not_misclassified(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "episode_001.hdf5"
            _write_pika_episode(path, with_gripper=False)

            report = audit_hdf5_episodes(path)
            audited = report["episodes"][0]

            self.assertEqual(audited["format_name"], "pika_umi_single_arm")
            self.assertTrue(_contains(audited["warnings"], "missing_gripper_field"))

    def test_bimanual_missing_gripper_field_is_audited(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "episode_001.hdf5"
            _write_bimanual_pika_episode(path, with_gripper=False)

            report = audit_hdf5_episodes(path)
            audited = report["episodes"][0]

            self.assertEqual(audited["format_name"], "pika_umi_bimanual")
            self.assertEqual(audited["arm_mask"], [1.0, 1.0])
            self.assertTrue(_contains(audited["warnings"], "missing_gripper_field"))

    def test_bimanual_pika_layout_maps_both_arms(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "episode_001.hdf5"
            _write_bimanual_pika_episode(path)

            report = audit_hdf5_episodes(path)
            audited = report["episodes"][0]

            self.assertEqual(audited["format_name"], "pika_umi_bimanual")
            self.assertEqual(audited["arm_mask"], [1.0, 1.0])
            self.assertIn("left_realsense_color", audited["camera_names"])
            self.assertIn("right_realsense_color", audited["camera_names"])

    def test_manifest_camera_filtering_works_for_flow_dataset(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            path = root / "episode_001.hdf5"
            _write_pika_episode(
                path,
                image_specs={
                    "fisheye": "jpeg",
                    "realsense_color": "jpeg",
                    "realsense_depth": "png16",
                },
            )
            manifest_path = root / "manifest.yaml"
            manifest_path.write_text(
                "\n".join(
                    [
                        "schema: robotics_lab.policy_runner.dataset_manifest.v1",
                        f"episodes_dir: {root}",
                        "include_formats:",
                        "  - pika_umi_single_arm",
                        "single_arm_side: left",
                        "camera_names:",
                        "  - fisheye",
                        "  - realsense_color",
                        "  - realsense_depth",
                        "exclude_camera_names:",
                        "  - realsense_depth",
                        "required_attrs:",
                        "  pose_format: x,y,z,qx,qy,qz,qw",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            manifest = DatasetManifest.load(manifest_path)
            dataset = FlowHdf5Dataset(
                manifest.resolved_episodes_dir(),
                action_horizon=2,
                image_size=8,
                normalize=False,
                **manifest.training_dataset_kwargs(),
            )
            sample = dataset.raw_sample(0)

            self.assertEqual(dataset.camera_names, ["fisheye", "realsense_color"])
            self.assertEqual(sample["images"].shape, (2, 3, 8, 8))
            self.assertEqual(int(sample["missing_camera_count"]), 0)

    def test_audit_warns_on_unknown_pose_frame_without_retarget(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "episode_001.hdf5"
            _write_pika_episode(path, pose_frame="mystery_world")

            report = audit_hdf5_episodes(path)
            audited = report["episodes"][0]

            self.assertTrue(_contains(audited["warnings"], "unknown_pose_frame"))
            self.assertTrue(_contains(audited["deployment_blockers"], "retarget_required"))

    def test_measured_retarget_manifest_clears_frame_deployment_blocker(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "episode_001.hdf5"
            _write_pika_episode(path, pose_frame="steamvr_world")
            manifest = DatasetManifest.from_mapping(
                {
                    "schema": "robotics_lab.policy_runner.dataset_manifest.v1",
                    "retarget": {
                        "source_pose_frame": "steamvr_world",
                        "target_pose_frame": "stand",
                        "status": "measured",
                    },
                }
            )

            report = audit_hdf5_episodes(path, dataset_manifest=manifest)
            audited = report["episodes"][0]

            self.assertFalse(_contains(audited["deployment_blockers"], "retarget_required"))

    def test_accepted_retarget_manifest_clears_frame_deployment_blocker(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "episode_001.hdf5"
            _write_pika_episode(path, pose_frame="steamvr_world")
            manifest = DatasetManifest.from_mapping(
                {
                    "schema": "robotics_lab.policy_runner.dataset_manifest.v1",
                    "retarget": {
                        "source_pose_frame": "steamvr_world",
                        "target_pose_frame": "stand",
                        "status": "accepted",
                    },
                }
            )

            report = audit_hdf5_episodes(path, dataset_manifest=manifest)
            audited = report["episodes"][0]

            self.assertFalse(_contains(audited["deployment_blockers"], "retarget_required"))

    def test_manifest_episode_list_limits_audit_to_listed_hdf5_files(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            keep = root / "episode_keep.hdf5"
            extra = root / "episode_extra.hdf5"
            _write_pika_episode(keep)
            _write_pika_episode(extra)
            (root / "checkpoints").mkdir()
            with h5py.File(root / "checkpoints" / "checkpoint_001.hdf5", "w") as handle:
                handle.attrs["not_episode"] = "checkpoint"
            manifest = DatasetManifest.from_mapping(
                {
                    "schema": "robotics_lab.policy_runner.dataset_manifest.v1",
                    "episodes": ["episode_keep.hdf5"],
                    "include_patterns": ["episode_*.hdf5"],
                }
            )

            report = audit_hdf5_episodes(root, dataset_manifest=manifest)

            self.assertEqual(report["aggregate"]["episode_count"], 1)
            self.assertEqual(report["episodes"][0]["path"], "episode_keep.hdf5")

    def test_manifest_include_patterns_limit_audit_to_matching_hdf5_files(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            _write_pika_episode(root / "episode_keep.hdf5")
            _write_pika_episode(root / "other_episode.hdf5")
            with h5py.File(root / "audit.hdf5", "w") as handle:
                handle.attrs["not_episode"] = "audit_output"
            manifest = DatasetManifest.from_mapping(
                {
                    "schema": "robotics_lab.policy_runner.dataset_manifest.v1",
                    "include_patterns": ["episode_*.hdf5"],
                }
            )

            report = audit_hdf5_episodes(root, dataset_manifest=manifest)

            self.assertEqual(report["aggregate"]["episode_count"], 1)
            self.assertEqual(report["episodes"][0]["path"], "episode_keep.hdf5")

    def test_obvious_non_episode_hdf5_files_are_skipped_in_directory_audit(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            _write_pika_episode(root / "episode_keep.hdf5")
            (root / "checkpoints").mkdir()
            with h5py.File(root / "checkpoints" / "checkpoint_001.hdf5", "w") as handle:
                handle.attrs["not_episode"] = "checkpoint"
            with h5py.File(root / "audit.hdf5", "w") as handle:
                handle.attrs["not_episode"] = "audit_output"
            with h5py.File(root / "episode_tmp.tmp.hdf5", "w") as handle:
                handle.attrs["not_episode"] = "temporary_conversion"

            report = audit_hdf5_episodes(root)

            self.assertEqual(report["aggregate"]["episode_count"], 1)
            self.assertEqual(report["episodes"][0]["path"], "episode_keep.hdf5")

    def test_audit_fails_closed_on_empty_dataset(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaisesRegex(ValueError, "no HDF5 episodes"):
                audit_hdf5_episodes(tmp)

    def test_quaternion_normalization_warning_works(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "episode_001.hdf5"
            _write_pika_episode(path, quaternion_scale=2.0)

            report = audit_hdf5_episodes(path)
            audited = report["episodes"][0]

            self.assertTrue(_contains(audited["warnings"], "non_normalized_quaternion"))


def _write_pika_episode(
    path: Path,
    *,
    length: int = 4,
    pose_frame: str = "steamvr_world",
    quaternion_scale: float = 1.0,
    image_specs: dict[str, str] | None = None,
    with_gripper: bool = True,
    image_encoding_attrs: bool = True,
) -> None:
    assert h5py is not None and np is not None
    image_specs = image_specs or {"fisheye": "jpeg"}
    pose = np.zeros((length, 7), dtype=np.float32)
    pose[:, 0] = np.linspace(0.0, 0.03, length)
    pose[:, 6] = quaternion_scale
    action = np.zeros((length, 8), dtype=np.float32)
    action[:, :7] = pose
    action[:, 7] = np.linspace(0.0, 1.0, length)
    gripper = np.zeros((length, 2), dtype=np.float32)
    gripper[:, 0] = 0.1

    with h5py.File(path, "w") as handle:
        handle.attrs["pose_frame"] = pose_frame
        handle.attrs["pose_format"] = "x,y,z,qx,qy,qz,qw"
        handle.create_dataset("action", data=action)
        handle.create_dataset("timestamp", data=np.arange(length, dtype=np.float64) / 30.0)
        obs = handle.create_group("observations")
        obs.create_dataset("pose", data=pose)
        if with_gripper:
            obs.create_dataset("gripper", data=gripper)
        images = obs.create_group("images")
        for name, encoding in image_specs.items():
            _write_vlen_image_dataset(
                images,
                name,
                length,
                encoding=encoding,
                with_encoding_attr=image_encoding_attrs,
            )


def _write_bimanual_pika_episode(path: Path, *, length: int = 4, with_gripper: bool = True) -> None:
    assert h5py is not None and np is not None
    with h5py.File(path, "w") as handle:
        handle.attrs["arm_names"] = "left,right"
        handle.attrs["pose_frame"] = "steamvr_world"
        handle.attrs["pose_format"] = "x,y,z,qx,qy,qz,qw"
        handle.create_dataset("timestamp", data=np.arange(length, dtype=np.float64) / 30.0)
        obs = handle.create_group("observations")
        for side, sign in (("left", 1.0), ("right", -1.0)):
            group = obs.create_group(side)
            pose = np.zeros((length, 7), dtype=np.float32)
            pose[:, 0] = sign * np.linspace(0.0, 0.03, length)
            pose[:, 6] = 1.0
            gripper = np.zeros((length, 2), dtype=np.float32)
            action = np.zeros((length, 8), dtype=np.float32)
            action[:, :7] = pose
            action[:, 0] += sign * 0.01
            group.create_dataset("pose", data=pose)
            if with_gripper:
                group.create_dataset("gripper", data=gripper)
            group.create_dataset("action", data=action)
            images = group.create_group("images")
            _write_vlen_image_dataset(images, "realsense_color", length, encoding="jpeg")


def _write_vlen_image_dataset(
    group: h5py.Group,
    name: str,
    length: int,
    *,
    encoding: str,
    with_encoding_attr: bool = True,
) -> None:
    assert h5py is not None and np is not None
    dtype = h5py.vlen_dtype(np.dtype("uint8"))
    dataset = group.create_dataset(name, shape=(length,), dtype=dtype)
    if with_encoding_attr:
        dataset.attrs["encoding"] = encoding
    for index in range(length):
        dataset[index] = np.frombuffer(_image_bytes(encoding, index=index), dtype=np.uint8)


def _image_bytes(encoding: str, *, index: int) -> bytes:
    assert Image is not None and np is not None
    buffer = io.BytesIO()
    if encoding == "png16":
        arr16 = np.full((12, 10), 1000 + index, dtype=np.uint16)
        Image.fromarray(arr16).save(buffer, format="PNG")
    else:
        arr = np.zeros((12, 10, 3), dtype=np.uint8)
        arr[:, :, 0] = 20 + index
        arr[:, :, 1] = 40
        arr[:, :, 2] = 80
        Image.fromarray(arr, mode="RGB").save(buffer, format="JPEG")
    return buffer.getvalue()


def _contains(values: list[str], needle: str) -> bool:
    return any(needle in value for value in values)


if __name__ == "__main__":
    unittest.main()
