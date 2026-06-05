from __future__ import annotations

import unittest

try:
    import torch

    from policy_runner.flow_model import FlowMatchingPolicy, FlowModelConfig, flow_matching_loss
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


if __name__ == "__main__":
    unittest.main()
