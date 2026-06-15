from __future__ import annotations

import hashlib
import json
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
        DirectBcCheckpointEnsembleActionSource,
        DirectBcImageActionSource,
        IMITATION_ENSEMBLE_REPORT_SCHEMA,
        FlowMatchingActionSource,
        IMITATION_CHECKPOINT_SCHEMA,
        action_chunk_checkpoint_kind,
        canonical_flow_command_family,
        load_action_chunk_checkpoint_dataset_stats,
        resolve_flow_command_family,
        validate_flow_command_family,
    )
    from policy_runner.flow_model import FlowMatchingPolicy, FlowModelConfig
    from policy_runner.imitation_experiments import _build_direct_bc_policy
except Exception:
    np = None
    torch = None


@unittest.skipIf(torch is None, "torch is not installed")
class FlowInferenceTcpTwistLocalTest(unittest.TestCase):
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
                self.assertEqual(source.command_family, "TcpTwistLocal")
            finally:
                source.close()

    def test_flow_delta_converts_to_bounded_tcp_twist_local(self) -> None:
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
        self.assertEqual(intent.mode, "TcpTwistLocal")
        self.assertEqual(intent.left["mode"], "TcpTwistLocal")
        self.assertEqual(intent.left["tcp_twist_local"], [0.2, 0.0, 0.0, 0.0, 0.0, 0.0])
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
        self.assertEqual(intent.left["tcp_twist_local"], [0.2, 0.0, 0.0, 0.0, 0.0, 0.25])

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
        self.assertEqual(intent.right["mode"], "TcpTwistLocal")
        _assert_sequence_almost_equal(
            self,
            intent.right["tcp_twist_local"],
            [0.0, 0.3, 0.0, 0.0, 0.0, 0.0],
        )

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
        self.assertEqual(stopped.left["tcp_twist_local"], [0.0] * 6)
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
        self.assertEqual(missing_camera_stop.left["tcp_twist_local"], [0.0] * 6)

    def test_command_family_defaults_and_controller_sim_local_guard(self) -> None:
        # Only tcp_twist_local (ee_local body frame) exists; world-frame "stand"
        # families were removed.
        self.assertEqual(
            resolve_flow_command_family(RolloutMode.CONTROLLER_SIM, None),
            "tcp_twist_local",
        )
        self.assertEqual(
            resolve_flow_command_family(
                RolloutMode.SIM_DRYRUN,
                None,
                dataset_stats={"proprio_action_frame": "ee_local"},
            ),
            "tcp_twist_local",
        )
        self.assertEqual(canonical_flow_command_family("tcp_twist_local"), "TcpTwistLocal")
        # Removed stand families are no longer valid command-family names.
        with self.assertRaisesRegex(RolloutModeValidationError, "invalid command-family"):
            canonical_flow_command_family("tcp_twist_stand")
        with self.assertRaisesRegex(RolloutModeValidationError, "invalid command-family"):
            validate_flow_command_family(RolloutMode.SIM_DRYRUN, "tcp_delta_stand")
        # Live rollout of body-frame twist still requires the explicit opt-in.
        with self.assertRaisesRegex(RolloutModeValidationError, "tcp_twist_local"):
            validate_flow_command_family(RolloutMode.CONTROLLER_SIM, "tcp_twist_local")
        with self.assertRaisesRegex(RolloutModeValidationError, "tcp_twist_local"):
            validate_flow_command_family(
                RolloutMode.CONTROLLER_SIM,
                "tcp_twist_local",
                dataset_stats={"proprio_action_frame": "ee_local"},
            )
        # Non-command-sending modes accept it without the opt-in.
        validate_flow_command_family(
            RolloutMode.SIM_DRYRUN,
            "tcp_twist_local",
            dataset_stats={"proprio_action_frame": "ee_local"},
        )
        validate_flow_command_family(RolloutMode.SIM_DRYRUN, "tcp_twist_local")
        validate_flow_command_family(RolloutMode.REAL_READONLY, "tcp_twist_local")
        # Explicit opt-in unlocks live rollout.
        validate_flow_command_family(
            RolloutMode.CONTROLLER_SIM,
            "tcp_twist_local",
            allow_tcp_twist_local=True,
        )

    def test_ee_local_r_align_resolution_and_rotation_semantics(self) -> None:
        from policy_runner.flow_inference import (
            resolve_ee_local_r_align,
            rotate_flow_arm_vectors,
        )

        self.assertIsNone(resolve_ee_local_r_align(None))
        self.assertIsNone(resolve_ee_local_r_align("none"))
        with self.assertRaisesRegex(ValueError, "preset"):
            resolve_ee_local_r_align("bogus_preset")
        with self.assertRaisesRegex(ValueError, "orthonormal"):
            resolve_ee_local_r_align([1.0] * 9)

        r_align = resolve_ee_local_r_align("pika_tip")
        assert r_align is not None
        # Same matrix accepted as 9 floats text.
        np.testing.assert_allclose(
            resolve_ee_local_r_align("0,0,1,-1,0,0,0,-1,0"), r_align
        )

        # Action direction (v_tcp = R_alignT . v_tip): tip +x (approach) -> TCP +z,
        # tip +z (up) -> TCP -y (TCP y points down).
        step = np.zeros(14, dtype=np.float32)
        step[0:3] = (1.0, 0.0, 0.0)   # left linear: approach
        step[3:6] = (0.0, 0.0, 1.0)   # left angular: about tip up axis
        step[6] = 0.5                  # left gripper untouched
        step[7:10] = (0.0, 0.0, 1.0)  # right linear: up
        step[13] = -0.25               # right gripper untouched
        out = rotate_flow_arm_vectors(step, r_align.T)
        np.testing.assert_allclose(out[0:3], (0.0, 0.0, 1.0), atol=1e-7)
        np.testing.assert_allclose(out[3:6], (0.0, -1.0, 0.0), atol=1e-7)
        self.assertAlmostEqual(float(out[6]), 0.5)
        np.testing.assert_allclose(out[7:10], (0.0, -1.0, 0.0), atol=1e-7)
        self.assertAlmostEqual(float(out[13]), -0.25)

        # Proprio direction (v_tip = R_align . v_tcp): TCP +z (approach) -> tip +x.
        proprio = np.zeros(16, dtype=np.float32)
        proprio[0:3] = (0.0, 0.0, 1.0)
        proprio[14:16] = (1.0, 1.0)   # arm_mask tail untouched
        out = rotate_flow_arm_vectors(proprio, r_align)
        np.testing.assert_allclose(out[0:3], (1.0, 0.0, 0.0), atol=1e-7)
        np.testing.assert_allclose(out[14:16], (1.0, 1.0))

        # Round trip and chunk shape.
        chunk = np.random.default_rng(0).normal(size=(4, 14)).astype(np.float32)
        round_trip = rotate_flow_arm_vectors(
            rotate_flow_arm_vectors(chunk, r_align), r_align.T
        )
        np.testing.assert_allclose(round_trip, chunk, atol=1e-6)

    def test_tcp_twist_local_readonly_and_optin_paths(self) -> None:
        # real_readonly sends no commands -> family always accepted.
        validate_flow_command_family(
            RolloutMode.REAL_READONLY,
            "tcp_twist_local",
            dataset_stats={"proprio_action_frame": "ee_local"},
        )
        # Live modes stay rejected by default but open with the explicit opt-in.
        for mode in (RolloutMode.CONTROLLER_SIM, RolloutMode.REAL_POLICY):
            with self.assertRaisesRegex(RolloutModeValidationError, "allow-tcp-twist-local"):
                validate_flow_command_family(
                    mode,
                    "tcp_twist_local",
                    dataset_stats={"proprio_action_frame": "ee_local"},
                )
            validate_flow_command_family(
                mode,
                "tcp_twist_local",
                allow_tcp_twist_local=True,
                dataset_stats={"proprio_action_frame": "ee_local"},
            )

    def test_ee_local_checkpoint_defaults_to_tcp_twist_local(self) -> None:
        assert torch is not None
        with tempfile.TemporaryDirectory() as tmp:
            checkpoint = Path(tmp) / "flow_policy.pt"
            _write_flow_checkpoint(checkpoint, proprio_action_frame="ee_local")
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

        self.assertEqual(source.command_family, "TcpTwistLocal")
        self.assertIsNotNone(intent)
        assert intent is not None
        self.assertEqual(intent.mode, "TcpTwistLocal")
        self.assertEqual(intent.left["tcp_twist_local"], [0.2, 0.0, 0.0, 0.0, 0.0, 0.0])

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
        _assert_sequence_almost_equal(
            self,
            first_intent.left["tcp_twist_local"],
            [0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        _assert_sequence_almost_equal(
            self,
            second_intent.left["tcp_twist_local"],
            [0.2, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        _assert_sequence_almost_equal(
            self,
            resampled_intent.left["tcp_twist_local"],
            [0.9, 0.0, 0.0, 0.0, 0.0, 0.0],
        )

    def test_direct_bc_checkpoint_converts_chunk_to_bounded_tcp_twist_local(self) -> None:
        assert torch is not None
        with tempfile.TemporaryDirectory() as tmp:
            checkpoint = Path(tmp) / "direct_bc.pt"
            _write_direct_bc_checkpoint(checkpoint)
            source = DirectBcImageActionSource(
                checkpoint,
                device="cpu",
                policy_dt_sec=0.01,
                max_linear_velocity_m_s=0.2,
                max_angular_velocity_rad_s=0.5,
            )
            try:
                intent = source.next_intent(_sample_state(), 0.0)
            finally:
                source.close()
            checkpoint_kind = action_chunk_checkpoint_kind(checkpoint)

        self.assertEqual(checkpoint_kind, "direct_bc")
        self.assertIsNotNone(intent)
        assert intent is not None
        self.assertEqual(intent.mode, "TcpTwistLocal")
        self.assertEqual(intent.left["mode"], "TcpTwistLocal")
        _assert_sequence_almost_equal(
            self,
            intent.left["tcp_twist_local"],
            [0.1, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        self.assertEqual(intent.right["mode"], "Hold")

    def test_direct_bc_checkpoint_without_image_size_fails_closed(self) -> None:
        assert torch is not None
        with tempfile.TemporaryDirectory() as tmp:
            checkpoint = Path(tmp) / "direct_bc.pt"
            _write_direct_bc_checkpoint(checkpoint, include_image_size=False)

            with self.assertRaisesRegex(ValueError, "missing image_size"):
                DirectBcImageActionSource(checkpoint, device="cpu", policy_dt_sec=0.01)

    def test_direct_bc_ensemble_report_averages_member_predictions(self) -> None:
        assert torch is not None
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            first = root / "first.pt"
            second = root / "second.pt"
            _write_direct_bc_checkpoint(first, bias=0.001)
            _write_direct_bc_checkpoint(second, bias=0.003)
            report = root / "ensemble_report.json"
            _write_ensemble_report(report, [first, second])
            source = DirectBcCheckpointEnsembleActionSource(
                report,
                ensemble_name="top2",
                device="cpu",
                policy_dt_sec=0.01,
                max_linear_velocity_m_s=1.0,
                max_angular_velocity_rad_s=1.0,
            )
            try:
                intent = source.next_intent(_sample_state(), 0.0)
            finally:
                source.close()
            checkpoint_kind = action_chunk_checkpoint_kind(report)

        self.assertEqual(checkpoint_kind, "direct_bc_ensemble")
        self.assertIsNotNone(intent)
        assert intent is not None
        _assert_sequence_almost_equal(
            self,
            intent.left["tcp_twist_local"],
            [0.2, 0.0, 0.0, 0.0, 0.0, 0.0],
        )

    def test_direct_bc_ensemble_stats_use_selected_ensemble_member(self) -> None:
        assert torch is not None
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            default = root / "default.pt"
            alternate = root / "alternate.pt"
            _write_direct_bc_checkpoint(default, dt_mean_sec=0.01)
            _write_direct_bc_checkpoint(alternate, dt_mean_sec=0.02)
            report = root / "ensemble_report.json"
            payload = {
                "schema": IMITATION_ENSEMBLE_REPORT_SCHEMA,
                "args": {"image_size": 32},
                "ensembles": [
                    {
                        "name": "top5",
                        "members": [str(default)],
                        "member_checkpoint_sha256": [_sha256_file(default)],
                    },
                    {
                        "name": "alternate",
                        "members": [str(alternate)],
                        "member_checkpoint_sha256": [_sha256_file(alternate)],
                    },
                ],
            }
            report.write_text(json.dumps(payload, sort_keys=True), encoding="utf-8")

            stats = load_action_chunk_checkpoint_dataset_stats(
                report,
                device="cpu",
                ensemble_name="alternate",
            )

        self.assertEqual(stats["dt_mean_sec"], 0.02)


def _write_flow_checkpoint(
    path: Path,
    *,
    arm_mask_counts: dict[str, int] | None = None,
    camera_names: tuple[str, ...] = (),
    action_horizon: int = 2,
    proprio_action_frame: str | None = None,
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
                **({"proprio_action_frame": proprio_action_frame} if proprio_action_frame is not None else {}),
            },
            "camera_names": list(camera_names),
            "image_size": 32,
            "model_config": config.to_dict(),
            "model_state": model.state_dict(),
        },
        path,
    )


def _assert_sequence_almost_equal(
    test_case: unittest.TestCase,
    observed: list[float],
    expected: list[float],
) -> None:
    test_case.assertEqual(len(observed), len(expected))
    for actual, desired in zip(observed, expected):
        test_case.assertAlmostEqual(actual, desired, places=7)


def _write_direct_bc_checkpoint(
    path: Path,
    *,
    include_image_size: bool = True,
    bias: float = 0.001,
    dt_mean_sec: float = 0.01,
) -> None:
    assert torch is not None
    model = _build_direct_bc_policy(
        backbone="tiny_cnn",
        action_horizon=2,
        camera_count=0,
        hidden_dim=32,
    )
    with torch.no_grad():
        for param in model.parameters():
            param.zero_()
        model.head[2].bias[0] = float(bias)
    payload = {
        "schema": IMITATION_CHECKPOINT_SCHEMA,
        "model_family": "direct_bc_chunk",
        "backbone": "tiny_cnn",
        "action_horizon": 2,
        "action_dim": FLOW_ACTION_DIM,
        "proprio_dim": FLOW_PROPRIO_DIM,
        "camera_names": [],
        "dataset_stats": {
            "action_mean": [0.0] * FLOW_ACTION_DIM,
            "action_std": [1.0] * FLOW_ACTION_DIM,
            "proprio_mean": [0.0] * FLOW_PROPRIO_DIM,
            "proprio_std": [1.0] * FLOW_PROPRIO_DIM,
            "image_mean": [0.0, 0.0, 0.0],
            "image_std": [1.0, 1.0, 1.0],
            "arm_mask_counts": {"left": 1, "right": 0},
            "dt_mean_sec": float(dt_mean_sec),
        },
        "model_state": model.state_dict(),
    }
    if include_image_size:
        payload["image_size"] = 32
    torch.save(payload, path)


def _write_ensemble_report(path: Path, members: list[Path]) -> None:
    payload = {
        "schema": IMITATION_ENSEMBLE_REPORT_SCHEMA,
        "args": {"image_size": 32},
        "ensembles": [
            {
                "name": "top2",
                "members": [str(member) for member in members],
                "member_checkpoint_sha256": [_sha256_file(member) for member in members],
            }
        ],
    }
    path.write_text(json.dumps(payload, sort_keys=True), encoding="utf-8")


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


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
