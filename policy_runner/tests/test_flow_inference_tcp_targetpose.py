from __future__ import annotations

import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

from policy_runner.rollout_modes import RolloutMode, RolloutModeValidationError

try:
    import numpy as np
    import torch

    from policy_runner.flow_dataset import pose_delta_local
    from policy_runner.flow_inference import (
        FLOW_ACTION_DIM,
        FLOW_CHECKPOINT_SCHEMA,
        FLOW_PROPRIO_DIM,
        FlowMatchingActionSource,
        canonical_flow_command_family,
        resolve_flow_command_family,
        resolve_flow_policy_dt_sec,
        validate_flow_command_family,
    )
    from policy_runner.flow_model import FlowMatchingPolicy, FlowModelConfig
    from policy_runner.robot_state_client import StateSnapshot
except Exception:
    np = None
    torch = None


@unittest.skipIf(torch is None or np is None, "torch/numpy flow inference extras are not installed")
class FlowInferenceTcpTargetPoseTest(unittest.TestCase):
    def test_tcp_target_pose_family_resolves_and_requires_live_optin(self) -> None:
        self.assertEqual(
            resolve_flow_command_family(RolloutMode.SIM_DRYRUN, "tcp_pose_target"),
            "tcp_target_pose",
        )
        self.assertEqual(canonical_flow_command_family("tcpposetarget"), "TcpPoseTarget")
        self.assertEqual(
            resolve_flow_policy_dt_sec(
                RolloutMode.SIM_DRYRUN,
                "tcp_target_pose",
                policy_dt_sec=None,
                command_rate_hz=100.0,
                dataset_stats={"dt_mean_sec": 0.02},
            ),
            0.02,
        )

        with self.assertRaisesRegex(RolloutModeValidationError, "allow-tcp-target-pose"):
            validate_flow_command_family(RolloutMode.REAL_POLICY, "tcp_target_pose")
        validate_flow_command_family(
            RolloutMode.REAL_POLICY,
            "tcp_target_pose",
            allow_tcp_target_pose=True,
        )
        validate_flow_command_family(RolloutMode.SIM_DRYRUN, "tcp_target_pose")
        validate_flow_command_family(RolloutMode.REAL_READONLY, "tcp_target_pose")

    def test_flow_delta_composes_into_clamped_tcp_pose_target(self) -> None:
        assert torch is not None and np is not None
        measured = _pose7([0.4, -0.2, 0.3], _quat_from_axis_angle([0.0, 0.0, 1.0], np.pi / 2.0))
        with tempfile.TemporaryDirectory() as tmp:
            checkpoint = Path(tmp) / "flow_policy.pt"
            _write_flow_checkpoint(checkpoint)
            source = FlowMatchingActionSource(
                checkpoint,
                device="cpu",
                command_family="tcp_target_pose",
                policy_dt_sec=0.01,
                max_linear_velocity_m_s=0.2,
                max_angular_velocity_rad_s=0.25,
            )
            action = _action_chunk([0.01, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0])

            try:
                with mock.patch("policy_runner.flow_inference.sample_action_chunks", return_value=action):
                    intent = source.next_intent(_sample_state(left_pose=measured), 0.0)
            finally:
                source.close()

        self.assertIsNotNone(intent)
        assert intent is not None
        self.assertEqual(intent.mode, "TcpPoseTarget")
        self.assertEqual(intent.left["mode"], "TcpPoseTarget")
        target_payload = intent.left["tcp_target_stand"]
        self.assertIsInstance(target_payload, dict)
        target_pose = _pose_from_target_payload(target_payload)
        recovered = pose_delta_local(measured, target_pose)

        np.testing.assert_allclose(
            recovered,
            [0.002, 0.0, 0.0, 0.0, 0.0, 0.0025],
            atol=1e-6,
        )
        self.assertEqual(intent.right["mode"], "Hold")

    def test_chunk_boundary_reanchors_target_pose_to_measured_tcp(self) -> None:
        assert torch is not None and np is not None
        first_pose = _pose7([1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
        second_pose = _pose7([2.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
        with tempfile.TemporaryDirectory() as tmp:
            checkpoint = Path(tmp) / "flow_policy.pt"
            _write_flow_checkpoint(checkpoint, action_horizon=2)
            source = FlowMatchingActionSource(
                checkpoint,
                device="cpu",
                command_family="tcp_target_pose",
                policy_dt_sec=0.01,
                max_linear_velocity_m_s=1.0,
                max_angular_velocity_rad_s=1.0,
                chunk_execute_steps=1,
            )
            first = _action_chunk([0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            second = _action_chunk([0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

            try:
                with mock.patch(
                    "policy_runner.flow_inference.sample_action_chunks",
                    side_effect=[first, second],
                ):
                    first_intent = source.next_intent(_sample_state(left_pose=first_pose), 0.0)
                    second_intent = source.next_intent(_sample_state(left_pose=second_pose), 0.01)
            finally:
                source.close()

        assert first_intent is not None and second_intent is not None
        first_target = _pose_from_target_payload(first_intent.left["tcp_target_stand"])
        second_target = _pose_from_target_payload(second_intent.left["tcp_target_stand"])
        np.testing.assert_allclose(first_target[:3], [1.001, 0.0, 0.0], atol=1e-7)
        np.testing.assert_allclose(second_target[:3], [2.001, 0.0, 0.0], atol=1e-7)


def _sample_state(*, left_pose: np.ndarray) -> StateSnapshot:
    right_pose = _pose7([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
    return StateSnapshot(
        payload={
            "observed_mode": "simulation",
            "observed_backend": "simulator",
            "motion_state": "ConnectedHold",
            "fault_latched": False,
            "left": _arm_payload(left_pose),
            "right": _arm_payload(right_pose),
        },
        received_monotonic=time.monotonic(),
    )


def _arm_payload(pose: np.ndarray) -> dict:
    return {
        "has_valid_joint_state": True,
        "has_valid_tcp_pose": True,
        "q_actual_deg": [0, -30, 80, 0, 60, 0],
        "tcp_stand": {
            "x": float(pose[0]),
            "y": float(pose[1]),
            "z": float(pose[2]),
            "quaternion_xyzw": [float(value) for value in pose[3:7]],
        },
    }


def _pose7(position: list[float], quaternion_xyzw: list[float] | np.ndarray) -> np.ndarray:
    quat = np.asarray(quaternion_xyzw, dtype=np.float64)
    quat = quat / np.linalg.norm(quat)
    return np.asarray([*position, *quat.tolist()], dtype=np.float32)


def _pose_from_target_payload(payload: dict) -> np.ndarray:
    return _pose7(
        [float(payload["x"]), float(payload["y"]), float(payload["z"])],
        payload["quaternion_xyzw"],
    )


def _quat_from_axis_angle(axis: list[float], angle: float) -> np.ndarray:
    axis_array = np.asarray(axis, dtype=np.float64)
    axis_array = axis_array / np.linalg.norm(axis_array)
    half = angle * 0.5
    return np.asarray([*(axis_array * np.sin(half)), np.cos(half)], dtype=np.float64)


def _write_flow_checkpoint(
    path: Path,
    *,
    action_horizon: int = 2,
) -> None:
    assert torch is not None
    config = FlowModelConfig(
        action_horizon=action_horizon,
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
                "dt_mean_sec": 0.01,
                "dt_p50_sec": 0.01,
            },
            "camera_names": [],
            "image_size": 32,
            "model_config": config.to_dict(),
            "model_state": model.state_dict(),
        },
        path,
    )


def _action_chunk(*steps: list[float]) -> object:
    assert torch is not None
    out = torch.zeros((1, len(steps), FLOW_ACTION_DIM), dtype=torch.float32)
    for index, values in enumerate(steps):
        out[0, index, : len(values)] = torch.as_tensor(values, dtype=torch.float32)
    return out


if __name__ == "__main__":
    unittest.main()
