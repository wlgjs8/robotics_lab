from __future__ import annotations

import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from policy_runner.action_sources.tcp_delta import cartesian_action_requirements

try:
    import torch

    from policy_runner.flow_dataset import FLOW_ACTION_DIM, FLOW_CHECKPOINT_SCHEMA, FLOW_PROPRIO_DIM
    from policy_runner.flow_inference import FlowMatchingActionSource
    from policy_runner.flow_model import FlowMatchingPolicy, FlowModelConfig
except Exception:
    torch = None


class FlowInferenceCliTest(unittest.TestCase):
    def test_flow_infer_help_lists_rollout_mode_and_controller_sim(self) -> None:
        env = dict(os.environ)
        env["PYTHONPATH"] = "policy_runner"

        result = subprocess.run(
            [sys.executable, "-m", "policy_runner", "flow-infer", "--help"],
            cwd=Path(__file__).resolve().parents[2],
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--rollout-mode", result.stdout)
        self.assertIn("controller_sim", result.stdout)

    def test_controller_sim_requirement_helper_allows_rbpodo_carveout(self) -> None:
        requirements = cartesian_action_requirements(allow_rbpodo_controller_simulation=True)

        self.assertTrue(requirements.allow_rbpodo_controller_simulation_cartesian)
        self.assertTrue(requirements.cartesian_motion)


@unittest.skipIf(torch is None, "torch is not installed")
class FlowInferenceRequirementsTest(unittest.TestCase):
    def test_controller_sim_mode_sets_rbpodo_controller_simulation_requirement(self) -> None:
        assert torch is not None
        with tempfile.TemporaryDirectory() as tmp:
            checkpoint = Path(tmp) / "flow_policy.pt"
            _write_flow_checkpoint(checkpoint)

            source = FlowMatchingActionSource(
                checkpoint,
                device="cpu",
                allow_rbpodo_controller_simulation_cartesian=True,
            )
            try:
                self.assertTrue(source.requirements.allow_rbpodo_controller_simulation_cartesian)
                self.assertTrue(source.requirements.cartesian_motion)
                self.assertEqual(source.command_family, "TcpDeltaStand")
            finally:
                source.close()


def _write_flow_checkpoint(path: Path) -> None:
    assert torch is not None
    config = FlowModelConfig(
        action_horizon=2,
        action_dim=FLOW_ACTION_DIM,
        proprio_dim=FLOW_PROPRIO_DIM,
        camera_names=(),
        vision_backbone="tiny_cnn",
        hidden_dim=32,
        condition_encoder="mlp",
        frozen_vision=True,
    )
    model = FlowMatchingPolicy(config)
    torch.save(
        {
            "schema": FLOW_CHECKPOINT_SCHEMA,
            "dataset_stats": {
                "action_mean": [0.0] * FLOW_ACTION_DIM,
                "action_std": [1.0] * FLOW_ACTION_DIM,
                "proprio_mean": [0.0] * FLOW_PROPRIO_DIM,
                "proprio_std": [1.0] * FLOW_PROPRIO_DIM,
                "image_mean": [0.0, 0.0, 0.0],
                "image_std": [1.0, 1.0, 1.0],
                "arm_mask_counts": {"left": 1, "right": 0},
            },
            "camera_names": [],
            "image_size": 32,
            "model_config": config.to_dict(),
            "model_state": model.state_dict(),
        },
        path,
    )


if __name__ == "__main__":
    unittest.main()
