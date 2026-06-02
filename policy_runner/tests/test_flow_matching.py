from __future__ import annotations

import io
import tempfile
import unittest
from pathlib import Path

try:
    import h5py
    import numpy as np
    import torch
    import torchvision  # noqa: F401
    from PIL import Image

    from policy_runner.flow_dataset import FLOW_CHECKPOINT_SCHEMA
    from policy_runner.flow_model import FlowMatchingPolicy, FlowModelConfig, flow_matching_loss
    from policy_runner.flow_training import train_flow_matching
except Exception:
    h5py = None
    np = None
    torch = None
    Image = None


@unittest.skipIf(
    h5py is None or np is None or torch is None or Image is None,
    "flow matching ml extras not installed",
)
class FlowMatchingModelTest(unittest.TestCase):
    def test_flow_objective_produces_finite_loss(self) -> None:
        assert torch is not None
        model = FlowMatchingPolicy(
            FlowModelConfig(
                action_horizon=2,
                camera_names=("cam",),
                vision_backbone="resnet18",
                hidden_dim=32,
                condition_encoder="mlp",
                frozen_vision=True,
            )
        )
        batch = {
            "images": torch.zeros(2, 1, 3, 32, 32),
            "proprio": torch.zeros(2, 16),
            "action_chunk": torch.zeros(2, 2, 14),
            "action_mask": torch.ones(2, 2, 14),
        }

        loss = flow_matching_loss(model, batch)

        self.assertTrue(torch.isfinite(loss).item())

    def test_one_tiny_training_step_runs_on_cpu(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self._write_pika_episode(root / "episode_001.hdf5")
            result = train_flow_matching(
                episodes_dir=root,
                checkpoint_path=root / "flow_policy.pt",
                vision_backbone="resnet18",
                action_horizon=2,
                batch_size=1,
                epochs=1,
                lr=1e-4,
                image_size=32,
                hidden_dim=32,
                condition_encoder="mlp",
                val_split=0.0,
                sample_steps=1,
                device="cpu",
            )

            self.assertTrue(result.checkpoint_path.exists())
            self.assertTrue(result.dataset_stats_path.exists())
            self.assertTrue(result.curves_path.exists())
            saved = torch.load(result.checkpoint_path, map_location="cpu", weights_only=False)
            self.assertEqual(saved["schema"], FLOW_CHECKPOINT_SCHEMA)
            self.assertEqual(saved["action_horizon"], 2)
            self.assertIn("action_mse", saved["validation_metrics"])

    def _write_pika_episode(self, path: Path) -> None:
        assert h5py is not None and np is not None
        length = 3
        pose = np.zeros((length, 7), dtype=np.float32)
        pose[:, 0] = np.linspace(0.0, 0.02, length)
        pose[:, 6] = 1.0
        action = np.zeros((length, 8), dtype=np.float32)
        action[:, :7] = pose
        action[:, 0] += 0.01
        action[:, 7] = 0.5
        gripper = np.zeros((length, 2), dtype=np.float32)
        with h5py.File(path, "w") as handle:
            handle.create_dataset("action", data=action)
            handle.create_dataset("timestamp", data=np.arange(length, dtype=np.float64))
            obs = handle.create_group("observations")
            obs.create_dataset("pose", data=pose)
            obs.create_dataset("gripper", data=gripper)
            images = obs.create_group("images")
            dtype = h5py.vlen_dtype(np.dtype("uint8"))
            ds = images.create_dataset("cam", shape=(length,), dtype=dtype)
            for index in range(length):
                ds[index] = np.frombuffer(_jpeg_bytes(index), dtype=np.uint8)


def _jpeg_bytes(index: int) -> bytes:
    assert Image is not None and np is not None
    arr = np.full((32, 32, 3), 40 + index, dtype=np.uint8)
    image = Image.fromarray(arr, mode="RGB")
    buffer = io.BytesIO()
    image.save(buffer, format="JPEG")
    return buffer.getvalue()


if __name__ == "__main__":
    unittest.main()
