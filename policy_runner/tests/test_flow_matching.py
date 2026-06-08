from __future__ import annotations

import unittest

try:
    import torch

    from policy_runner.flow_model import (
        SUPPORTED_VISION_BACKBONES,
        FlowMatchingPolicy,
        FlowModelConfig,
        _convert_dinov3_convnext_state_dict,
        flow_matching_loss,
    )
except Exception:
    torch = None


@unittest.skipIf(
    torch is None,
    "torch is not installed",
)
class FlowMatchingModelTest(unittest.TestCase):
    def test_tiny_cnn_flow_objective_produces_finite_loss(self) -> None:
        assert torch is not None
        model = FlowMatchingPolicy(
            FlowModelConfig(
                action_horizon=2,
                camera_names=("cam",),
                vision_backbone="tiny_cnn",
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

    def test_dinov3_convnext_state_dict_mapping(self) -> None:
        assert torch is not None
        converted = _convert_dinov3_convnext_state_dict(
            {
                "downsample_layers.0.0.weight": torch.ones(96, 3, 4, 4),
                "downsample_layers.1.1.bias": torch.ones(192),
                "stages.2.7.gamma": torch.ones(384),
                "stages.3.1.pwconv2.bias": torch.ones(768),
                "norm.weight": torch.ones(768),
                "norms.3.weight": torch.ones(768),
            }
        )

        self.assertIn("dinov3_convnext_tiny", SUPPORTED_VISION_BACKBONES)
        self.assertEqual(
            converted["features.0.0.weight"].shape,
            torch.Size((96, 3, 4, 4)),
        )
        self.assertEqual(
            converted["features.2.1.bias"].shape,
            torch.Size((192,)),
        )
        self.assertEqual(
            converted["features.5.7.layer_scale"].shape,
            torch.Size((384, 1, 1)),
        )
        self.assertEqual(
            converted["features.7.1.block.5.bias"].shape,
            torch.Size((768,)),
        )
        self.assertEqual(
            converted["classifier.0.weight"].shape,
            torch.Size((768,)),
        )
        self.assertEqual(
            converted["norms.3.weight"].shape,
            torch.Size((768,)),
        )


if __name__ == "__main__":
    unittest.main()
