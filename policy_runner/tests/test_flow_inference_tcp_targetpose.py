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
        resolve_flow_policy_dt_sec,
    )
    from policy_runner.flow_model import FlowMatchingPolicy, FlowModelConfig
    from policy_runner.robot_state_client import StateSnapshot
except Exception:
    np = None
    torch = None


@unittest.skipIf(torch is None or np is None, "torch/numpy flow inference extras are not installed")
class FlowInferenceTcpPoseTargetTest(unittest.TestCase):
    def test_tcp_target_pose_policy_dt_resolves_without_family_optin(self) -> None:
        self.assertEqual(
            resolve_flow_policy_dt_sec(
                RolloutMode.SIM_DRYRUN,
                policy_dt_sec=None,
                command_rate_hz=100.0,
                dataset_stats={"dt_mean_sec": 0.02},
            ),
            0.02,
        )

    def test_flow_delta_composes_into_clamped_tcp_pose_target(self) -> None:
        assert torch is not None and np is not None
        measured = _pose7([0.4, -0.2, 0.3], _quat_from_axis_angle([0.0, 0.0, 1.0], np.pi / 2.0))
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

    # ------------------------------------------------ Patch 3: foh_se3 conditioning --
    def _streamed_source(self, tmp: str, conditioning: str, reanchor: str = "measured_legacy"):
        checkpoint = Path(tmp) / "flow_policy.pt"
        _write_flow_checkpoint(checkpoint, action_horizon=8)
        source = FlowMatchingActionSource(
            checkpoint,
            device="cpu",
            policy_dt_sec=0.02,
            max_linear_velocity_m_s=1.0,
            max_angular_velocity_rad_s=1.0,
            chunk_execute_steps=8,
            tcp_target_pose_conditioning=conditioning,
            tcp_target_pose_reanchor_mode=reanchor,
        )
        source.enable_async_chunking = True
        return source

    def test_foh_se3_emits_distinct_interpolated_targets_within_a_policy_step(self) -> None:
        assert torch is not None and np is not None
        measured = _pose7([0.4, 0.0, 0.3], [0.0, 0.0, 0.0, 1.0])
        chunk = _action_chunk(*([[0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]] * 8))
        with tempfile.TemporaryDirectory() as tmp:
            source = self._streamed_source(tmp, "foh_se3")
            try:
                with mock.patch(
                    "policy_runner.flow_inference.sample_action_chunks", return_value=chunk
                ):
                    state = _sample_state(left_pose=measured)
                    # t=0 activates the chunk and emits the anchor (alpha 0)
                    a = source.next_intent(state, 0.0)
                    # ticks WITHIN the first policy step [0, 0.02) -> interpolated, not held
                    b = source.next_intent(state, 0.004)
                    c = source.next_intent(state, 0.008)
            finally:
                source.close()
        xs = [
            _pose_from_target_payload(i.left["tcp_target_stand"])[0]
            for i in (a, b, c)
        ]
        # strictly increasing interpolation between the anchor and the first step target
        self.assertLess(xs[0], xs[1])
        self.assertLess(xs[1], xs[2])

    def test_legacy_step_hold_holds_target_between_policy_steps(self) -> None:
        assert torch is not None and np is not None
        measured = _pose7([0.4, 0.0, 0.3], [0.0, 0.0, 0.0, 1.0])
        chunk = _action_chunk(*([[0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]] * 8))
        with tempfile.TemporaryDirectory() as tmp:
            source = self._streamed_source(tmp, "legacy_step_hold")
            try:
                with mock.patch(
                    "policy_runner.flow_inference.sample_action_chunks", return_value=chunk
                ):
                    state = _sample_state(left_pose=measured)
                    a = source.next_intent(state, 0.0)
                    b = source.next_intent(state, 0.004)
                    c = source.next_intent(state, 0.008)
            finally:
                source.close()
        xs = [
            _pose_from_target_payload(i.left["tcp_target_stand"])[0]
            for i in (a, b, c)
        ]
        # legacy holds the same per-step target between policy ticks (ZOH)
        np.testing.assert_allclose(xs, [xs[0]] * 3, atol=1e-9)

    def test_foh_se3_hits_first_step_target_at_step_boundary_time(self) -> None:
        assert torch is not None and np is not None
        measured = _pose7([0.4, 0.0, 0.3], [0.0, 0.0, 0.0, 1.0])
        chunk = _action_chunk(*([[0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]] * 8))
        with tempfile.TemporaryDirectory() as tmp:
            source = self._streamed_source(tmp, "foh_se3")
            try:
                with mock.patch(
                    "policy_runner.flow_inference.sample_action_chunks", return_value=chunk
                ):
                    state = _sample_state(left_pose=measured)
                    source.next_intent(state, 0.0)  # anchor at measured
                    at_boundary = source.next_intent(state, 0.02)  # t = 1*policy_dt -> S_0
            finally:
                source.close()
        x = _pose_from_target_payload(at_boundary.left["tcp_target_stand"])[0]
        # S_0 = measured.x + one clamped step (0.01 m, within the 1.0 m/s * 0.02 s cap)
        self.assertAlmostEqual(x, 0.4 + 0.01, places=4)

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
