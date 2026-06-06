from __future__ import annotations

import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

from policy_runner.rollout_modes import RolloutMode, RolloutModeValidationError

try:
    import numpy as np
    import torch

    from policy_runner.flow_dataset import FLOW_ACTION_DIM, FLOW_CHECKPOINT_SCHEMA, FLOW_PROPRIO_DIM
    from policy_runner.flow_inference import (
        FlowMatchingActionSource,
        canonical_flow_command_family,
        resolve_flow_command_family,
        validate_flow_command_family,
    )
    from policy_runner.flow_model import FlowMatchingPolicy, FlowModelConfig
except Exception:
    np = None
    torch = None


@unittest.skipIf(torch is None, "torch is not installed")
class FlowInferenceTcpTwistStandTest(unittest.TestCase):
    def test_controller_sim_mode_sets_rbpodo_controller_simulation_requirement(self) -> None:
        assert torch is not None
        with tempfile.TemporaryDirectory() as tmp:
            checkpoint = Path(tmp) / "flow_policy.pt"
            _write_flow_checkpoint(checkpoint)

            source = FlowMatchingActionSource(
                checkpoint,
                device="cpu",
                policy_dt_sec=0.01,
                allow_rbpodo_controller_simulation_cartesian=True,
            )
            try:
                self.assertTrue(source.requirements.allow_rbpodo_controller_simulation_cartesian)
                self.assertTrue(source.requirements.cartesian_motion)
                self.assertEqual(source.command_family, "TcpTwistStand")
            finally:
                source.close()

    def test_flow_delta_converts_to_bounded_tcp_twist_stand(self) -> None:
        assert torch is not None
        with tempfile.TemporaryDirectory() as tmp:
            checkpoint = Path(tmp) / "flow_policy.pt"
            _write_flow_checkpoint(checkpoint)
            source = FlowMatchingActionSource(
                checkpoint,
                device="cpu",
                policy_dt_sec=0.01,
                max_linear_velocity_m_s=0.2,
                max_angular_velocity_rad_s=0.5,
            )
            action = _action_chunk([0.002, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

            try:
                with mock.patch("policy_runner.flow_inference.sample_action_chunks", return_value=action):
                    intent = source.next_intent(_sample_state(), 0.0)
            finally:
                source.close()

        self.assertIsNotNone(intent)
        assert intent is not None
        self.assertEqual(intent.mode, "TcpTwistStand")
        self.assertEqual(intent.left["mode"], "TcpTwistStand")
        self.assertEqual(intent.left["tcp_twist_stand"], [0.2, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.assertEqual(intent.right["mode"], "Hold")

    def test_flow_twist_clamps_to_configured_velocity_limits(self) -> None:
        assert torch is not None
        with tempfile.TemporaryDirectory() as tmp:
            checkpoint = Path(tmp) / "flow_policy.pt"
            _write_flow_checkpoint(checkpoint)
            source = FlowMatchingActionSource(
                checkpoint,
                device="cpu",
                policy_dt_sec=0.01,
                max_linear_velocity_m_s=0.2,
                max_angular_velocity_rad_s=0.25,
            )
            action = _action_chunk([0.01, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0])

            try:
                with mock.patch("policy_runner.flow_inference.sample_action_chunks", return_value=action):
                    intent = source.next_intent(_sample_state(), 0.0)
            finally:
                source.close()

        self.assertIsNotNone(intent)
        assert intent is not None
        self.assertEqual(intent.left["tcp_twist_stand"], [0.2, 0.0, 0.0, 0.0, 0.0, 0.25])

    def test_right_arm_flow_action_uses_indices_7_to_13(self) -> None:
        assert torch is not None
        with tempfile.TemporaryDirectory() as tmp:
            checkpoint = Path(tmp) / "flow_policy.pt"
            _write_flow_checkpoint(checkpoint, arm_mask_counts={"left": 0, "right": 1})
            source = FlowMatchingActionSource(
                checkpoint,
                device="cpu",
                policy_dt_sec=0.01,
                max_linear_velocity_m_s=0.5,
                max_angular_velocity_rad_s=0.5,
            )
            action = _action_chunk([0.0] * 7 + [0.0, 0.003, 0.0, 0.0, 0.0, 0.0, 0.0])

            try:
                with mock.patch("policy_runner.flow_inference.sample_action_chunks", return_value=action):
                    intent = source.next_intent(_sample_state(), 0.0)
            finally:
                source.close()

        self.assertIsNotNone(intent)
        assert intent is not None
        self.assertEqual(intent.left["mode"], "Hold")
        self.assertEqual(intent.right["mode"], "TcpTwistStand")
        self.assertEqual(intent.right["tcp_twist_stand"], [0.0, 0.3, 0.0, 0.0, 0.0, 0.0])

    def test_zero_action_sends_only_stop_zero_after_nonzero(self) -> None:
        assert torch is not None
        with tempfile.TemporaryDirectory() as tmp:
            checkpoint = Path(tmp) / "flow_policy.pt"
            _write_flow_checkpoint(checkpoint, action_horizon=3)
            source = FlowMatchingActionSource(
                checkpoint,
                device="cpu",
                policy_dt_sec=0.01,
                max_linear_velocity_m_s=0.2,
                max_angular_velocity_rad_s=0.5,
                chunk_execute_steps=3,
            )
            action = _action_chunk(
                [0.002, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0] * 14,
                [0.0] * 14,
            )

            try:
                with mock.patch("policy_runner.flow_inference.sample_action_chunks", return_value=action):
                    moving = source.next_intent(_sample_state(), 0.0)
                    stopped = source.next_intent(_sample_state(), 0.01)
                    idle = source.next_intent(_sample_state(), 0.02)
            finally:
                source.close()

        self.assertIsNotNone(moving)
        self.assertIsNotNone(stopped)
        assert stopped is not None
        self.assertEqual(stopped.left["tcp_twist_stand"], [0.0] * 6)
        self.assertIsNone(idle)

    def test_missing_camera_does_not_replay_cached_nonzero_twist(self) -> None:
        assert torch is not None
        with tempfile.TemporaryDirectory() as tmp:
            checkpoint = Path(tmp) / "flow_policy.pt"
            _write_flow_checkpoint(checkpoint, camera_names=("head",))
            camera_client = FakeCameraClient([_camera_bundle("head"), None])
            source = FlowMatchingActionSource(
                checkpoint,
                device="cpu",
                camera_client=camera_client,
                policy_dt_sec=0.01,
                max_linear_velocity_m_s=0.2,
                max_angular_velocity_rad_s=0.5,
            )
            action = _action_chunk(
                [0.002, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.002, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            )

            try:
                with mock.patch("policy_runner.flow_inference.sample_action_chunks", return_value=action):
                    moving = source.next_intent(_sample_state(), 0.0)
                    missing_camera_stop = source.next_intent(_sample_state(), 0.01)
            finally:
                source.close()

        self.assertIsNotNone(moving)
        self.assertIsNotNone(missing_camera_stop)
        assert missing_camera_stop is not None
        self.assertEqual(missing_camera_stop.left["tcp_twist_stand"], [0.0] * 6)

    def test_command_family_defaults_and_controller_sim_delta_guard(self) -> None:
        self.assertEqual(
            resolve_flow_command_family(RolloutMode.CONTROLLER_SIM, None),
            "tcp_twist_stand",
        )
        self.assertEqual(canonical_flow_command_family("tcp_twist_stand"), "TcpTwistStand")
        self.assertEqual(canonical_flow_command_family("tcp_twist_local"), "TcpTwistLocal")
        with self.assertRaisesRegex(RolloutModeValidationError, "tcp_twist_local"):
            validate_flow_command_family(RolloutMode.CONTROLLER_SIM, "tcp_twist_local")
        validate_flow_command_family(RolloutMode.SIM_DRYRUN, "tcp_twist_local")
        validate_flow_command_family(RolloutMode.SIM_DRYRUN, "tcp_delta_stand")
        validate_flow_command_family(RolloutMode.OFFLINE_EVAL, "tcp_delta_stand")
        with self.assertRaisesRegex(RolloutModeValidationError, "tcp_delta_stand"):
            validate_flow_command_family(RolloutMode.CONTROLLER_SIM, "tcp_delta_stand")
        validate_flow_command_family(
            RolloutMode.CONTROLLER_SIM,
            "tcp_delta_stand",
            allow_experimental_tcp_delta_stand=True,
        )

    def test_default_receding_horizon_resamples_after_half_chunk(self) -> None:
        assert torch is not None
        with tempfile.TemporaryDirectory() as tmp:
            checkpoint = Path(tmp) / "flow_policy.pt"
            _write_flow_checkpoint(checkpoint, action_horizon=4)
            source = FlowMatchingActionSource(
                checkpoint,
                device="cpu",
                policy_dt_sec=0.01,
                max_linear_velocity_m_s=1.0,
                max_angular_velocity_rad_s=1.0,
            )
            first = _action_chunk(
                [0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.002, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.003, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.004, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            )
            second = _action_chunk(
                [0.009, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.010, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.011, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.012, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            )

            try:
                with mock.patch(
                    "policy_runner.flow_inference.sample_action_chunks",
                    side_effect=[first, second],
                ):
                    first_intent = source.next_intent(_sample_state(), 0.0)
                    second_intent = source.next_intent(_sample_state(), 0.01)
                    resampled_intent = source.next_intent(_sample_state(), 0.02)
            finally:
                source.close()

        assert first_intent is not None
        assert second_intent is not None
        assert resampled_intent is not None
        self.assertEqual(first_intent.left["tcp_twist_stand"], [0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.assertEqual(second_intent.left["tcp_twist_stand"], [0.2, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.assertEqual(resampled_intent.left["tcp_twist_stand"], [0.9, 0.0, 0.0, 0.0, 0.0, 0.0])


def _write_flow_checkpoint(
    path: Path,
    *,
    arm_mask_counts: dict[str, int] | None = None,
    camera_names: tuple[str, ...] = (),
    action_horizon: int = 2,
) -> None:
    assert torch is not None
    config = FlowModelConfig(
        action_horizon=action_horizon,
        action_dim=FLOW_ACTION_DIM,
        proprio_dim=FLOW_PROPRIO_DIM,
        camera_names=camera_names,
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
                "arm_mask_counts": arm_mask_counts or {"left": 1, "right": 0},
                "dt_mean_sec": 0.01,
                "dt_p50_sec": 0.01,
            },
            "camera_names": list(camera_names),
            "image_size": 32,
            "model_config": config.to_dict(),
            "model_state": model.state_dict(),
        },
        path,
    )


def _sample_state() -> object:
    from policy_runner.robot_state_client import StateSnapshot

    arm = {
        "has_valid_joint_state": True,
        "has_valid_tcp_pose": True,
        "q_actual_deg": [0, -30, 80, 0, 60, 0],
        "tcp_stand": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
        },
    }
    return StateSnapshot(
        payload={
            "observed_mode": "simulation",
            "observed_backend": "simulator",
            "motion_state": "ConnectedHold",
            "fault_latched": False,
            "left": dict(arm),
            "right": dict(arm),
        },
        received_monotonic=time.monotonic(),
    )


def _action_chunk(*steps: list[float]) -> object:
    assert torch is not None
    if not steps:
        steps = ([0.0] * FLOW_ACTION_DIM,)
    out = torch.zeros((1, len(steps), FLOW_ACTION_DIM), dtype=torch.float32)
    for index, values in enumerate(steps):
        out[0, index, : len(values)] = torch.as_tensor(values, dtype=torch.float32)
    return out


def _camera_bundle(camera_name: str) -> object:
    assert np is not None
    pixels = np.zeros((8, 8, 3), dtype=np.uint8)
    return SimpleNamespace(frames={camera_name: SimpleNamespace(pixels=pixels)})


class FakeCameraClient:
    def __init__(self, bundles: list[object | None]):
        self._bundles = list(bundles)
        self.closed = False

    def poll(self, timeout_ms: int = 0) -> object | None:
        _ = timeout_ms
        if not self._bundles:
            return None
        return self._bundles.pop(0)

    def is_fresh(self, bundle: object | None) -> bool:
        return bundle is not None

    def close(self) -> None:
        self.closed = True


if __name__ == "__main__":
    unittest.main()
