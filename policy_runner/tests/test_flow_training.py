from __future__ import annotations

import importlib
import io
import json
import tempfile
import unittest
from pathlib import Path

try:
    from policy_runner.ml_preflight import run_ml_preflight
except Exception:
    run_ml_preflight = None

try:
    import h5py
    import numpy as np
    import torch
    from PIL import Image

    from policy_runner.flow_dataset import FLOW_CHECKPOINT_SCHEMA
    from policy_runner.flow_training import train_flow_matching
except Exception:
    h5py = None
    np = None
    torch = None
    Image = None
    train_flow_matching = None


class MlPreflightTest(unittest.TestCase):
    @unittest.skipIf(run_ml_preflight is None or torch is None, "torch is not installed")
    def test_ml_preflight_tiny_cnn_passes(self) -> None:
        stdout = io.StringIO()

        rc = run_ml_preflight(vision_backbone="tiny_cnn", stdout=stdout)

        self.assertEqual(rc, 0, stdout.getvalue())
        self.assertIn("torch: OK", stdout.getvalue())
        self.assertIn("backbone_forward: OK backbone=tiny_cnn", stdout.getvalue())

    @unittest.skipIf(run_ml_preflight is None or torch is None, "torch is not installed")
    def test_ml_preflight_resnet_reports_torchvision_failure_when_broken(self) -> None:
        def broken_torchvision_import(module_name: str):
            if module_name == "torchvision":
                raise RuntimeError("operator torchvision::nms does not exist")
            return importlib.import_module(module_name)

        stdout = io.StringIO()

        rc = run_ml_preflight(
            vision_backbone="resnet18",
            stdout=stdout,
            import_module=broken_torchvision_import,
        )

        output = stdout.getvalue()
        self.assertEqual(rc, 1, output)
        self.assertIn("torchvision: ERROR RuntimeError: operator torchvision::nms does not exist", output)
        self.assertIn("requested_backbone_error: torchvision import failed", output)


@unittest.skipIf(
    h5py is None or np is None or torch is None or Image is None or train_flow_matching is None,
    "flow training ml extras not installed",
)
class FlowTrainingTest(unittest.TestCase):
    def test_one_tiny_training_step_runs_on_cpu_without_torchvision(self) -> None:
        assert torch is not None and train_flow_matching is not None
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            uploaded_episode = Path("episode_002.hdf5")
            if uploaded_episode.exists():
                episodes_dir = uploaded_episode
            else:
                episodes_dir = root / "episode_002.hdf5"
                _write_pika_episode(
                    episodes_dir,
                    image_specs={"fisheye": "jpeg", "realsense_color": "jpeg"},
                    length=5,
                )

            result = train_flow_matching(
                episodes_dir=episodes_dir,
                checkpoint_path=root / "flow_policy_smoke.pt",
                vision_backbone="tiny_cnn",
                action_horizon=4,
                batch_size=2,
                epochs=1,
                image_size=32,
                hidden_dim=32,
                condition_encoder="mlp",
                val_split=0.0,
                sample_steps=1,
                device="cpu",
                camera_names=["fisheye", "realsense_color"],
                write_eval_report=root / "flow_eval_report.md",
            )

            self.assertTrue(result.checkpoint_path.exists())
            self.assertTrue(result.dataset_stats_path.exists())
            self.assertTrue(result.curves_path.exists())
            self.assertTrue(result.eval_report_path.exists())
            self.assertTrue(result.eval_summary_path.exists())
            saved = torch.load(result.checkpoint_path, map_location="cpu", weights_only=False)
            self.assertEqual(saved["schema"], FLOW_CHECKPOINT_SCHEMA)
            self.assertEqual(saved["model_config"]["vision_backbone"], "tiny_cnn")
            self.assertIn("action_mse", saved["validation_metrics"])

    def test_flow_train_camera_name_filter(self) -> None:
        assert torch is not None and train_flow_matching is not None
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            _write_pika_episode(
                root / "episode_001.hdf5",
                image_specs={
                    "fisheye": "jpeg",
                    "realsense_color": "jpeg",
                    "realsense_depth": "png16",
                },
            )

            result = train_flow_matching(
                episodes_dir=root,
                checkpoint_path=root / "flow_policy.pt",
                vision_backbone="tiny_cnn",
                action_horizon=2,
                batch_size=2,
                epochs=1,
                image_size=32,
                hidden_dim=32,
                condition_encoder="mlp",
                val_split=0.0,
                sample_steps=1,
                device="cpu",
                camera_names=["fisheye", "realsense_color", "realsense_depth"],
                exclude_camera_names=["realsense_depth"],
            )

            saved = torch.load(result.checkpoint_path, map_location="cpu", weights_only=False)
            self.assertEqual(saved["camera_names"], ["fisheye", "realsense_color"])
            stats = json.loads(result.dataset_stats_path.read_text(encoding="utf-8"))
            self.assertEqual(stats["camera_names"], ["fisheye", "realsense_color"])
            self.assertEqual(stats["missing_camera_count"], 0)

    def test_eval_report_written(self) -> None:
        assert torch is not None and train_flow_matching is not None
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            _write_pika_episode(
                root / "episode_001.hdf5",
                image_specs={"fisheye": "jpeg", "realsense_color": "jpeg"},
            )

            result = train_flow_matching(
                episodes_dir=root,
                checkpoint_path=root / "flow_policy.pt",
                vision_backbone="tiny_cnn",
                action_horizon=2,
                batch_size=2,
                epochs=1,
                image_size=32,
                hidden_dim=32,
                condition_encoder="mlp",
                val_split=0.0,
                sample_steps=1,
                device="cpu",
                camera_names=["fisheye", "realsense_color"],
                write_eval_report=root / "custom_flow_eval_report.md",
            )

            summary = json.loads(result.eval_summary_path.read_text(encoding="utf-8"))
            report = result.eval_report_path.read_text(encoding="utf-8")
            self.assertEqual(summary["dataset"]["camera_names"], ["fisheye", "realsense_color"])
            self.assertIn("action_mse", summary["validation"])
            self.assertEqual(len(summary["checkpoint"]["sha256"]), 64)
            self.assertIn("# Flow Evaluation Report", report)
            self.assertIn("chunk_endpoint_error", report)


def _write_pika_episode(
    path: Path,
    *,
    image_specs: dict[str, str],
    length: int = 4,
) -> None:
    assert h5py is not None and np is not None
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
        handle.attrs["pose_format"] = "x,y,z,qx,qy,qz,qw"
        handle.create_dataset("action", data=action)
        handle.create_dataset("timestamp", data=np.arange(length, dtype=np.float64) / 30.0)
        obs = handle.create_group("observations")
        obs.create_dataset("pose", data=pose)
        obs.create_dataset("gripper", data=gripper)
        images = obs.create_group("images")
        for name, encoding in image_specs.items():
            _write_vlen_image_dataset(images, name, length, encoding=encoding)


def _write_vlen_image_dataset(
    group: h5py.Group,
    name: str,
    length: int,
    *,
    encoding: str,
) -> None:
    assert h5py is not None and np is not None
    dtype = h5py.vlen_dtype(np.dtype("uint8"))
    dataset = group.create_dataset(name, shape=(length,), dtype=dtype)
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


if __name__ == "__main__":
    unittest.main()
