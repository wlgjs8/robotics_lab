from __future__ import annotations

import contextlib
import json
import tempfile
import unittest
import warnings
from io import StringIO
from pathlib import Path

from policy_runner.recording import (
    DATASET_METADATA_SCHEMA,
    EpisodeRecorder,
    _hash_canonical_json,
    build_dataset_metadata,
)
from policy_runner.robot_state_client import StateSnapshot
from policy_runner.training import action_vector, load_dataset, state_vector, train_behavior_cloning

try:
    import h5py
except ModuleNotFoundError:
    h5py = None

try:
    import torch
except ModuleNotFoundError:
    torch = None


def state_payload(tick: int = 1) -> dict:
    arm = {
        "q_actual_deg": [1, 2, 3, 4, 5, 6],
        "q_sent_deg": [1, 2, 3, 4, 5, 6],
        "tcp_stand": {"x": 0.1, "y": 0.2, "z": 0.3, "rx": 0, "ry": 0, "rz": 0, "qw": 1},
    }
    return {
        "tick": tick,
        "host_time_ns": 123,
        "period_ms": 50,
        "jitter_ms": 0,
        "motion_state": "Running",
        "fault_latched": False,
        "left": dict(arm),
        "right": dict(arm),
    }


def config_snapshots(path_kp_pos: float = 6.0, damping: float = 0.001) -> tuple[dict, dict]:
    cartesian = {
        "schema": "robotics_lab.cartesian_control_snapshot.v1",
        "enable": True,
        "path_kp_pos": path_kp_pos,
    }
    kinematics = {
        "schema": "robotics_lab.kinematics_snapshot.v1",
        "provider": "pinocchio",
        "ik": {"damping": damping},
    }
    return cartesian, kinematics


class RecordingAndTrainingTest(unittest.TestCase):
    def test_dataset_metadata_builder_keeps_required_provenance_keys(self) -> None:
        metadata = build_dataset_metadata(
            git_commit="abc123",
            config_hash="config-sha",
            backend_type="rbpodo",
            run_mode="real",
            operation_mode="simulation",
            physical_motion_expected=False,
            controller_pgmode="simulation",
            calibration_status="configured_estimate",
            camera_status="disabled",
            command_source_id="policy_runner",
            benchmark_linkage={"overlay_run_id": "overlay-1"},
        )

        self.assertEqual(metadata["schema"], DATASET_METADATA_SCHEMA)
        self.assertEqual(metadata["backend_type"], "rbpodo")
        self.assertEqual(metadata["run_mode"], "real")
        self.assertEqual(metadata["operation_mode"], "simulation")
        self.assertFalse(metadata["physical_motion_expected"])
        self.assertEqual(metadata["benchmark_linkage"]["overlay_run_id"], "overlay-1")

    def test_episode_recorder_writes_state_and_action_jsonl(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            recorder = EpisodeRecorder(tmp, episode_name="episode_test")
            try:
                recorder.record_state(StateSnapshot(state_payload(), 1.0))
                recorder.record_action(
                    {
                        "seq": 1,
                        "mode": "TcpPoseTarget",
                        "left": {"mode": "TcpPoseTarget", "tcp_target_stand": [0.31, 0.2, 0.3, 0, 0, 0]},
                        "right": {"mode": "Hold"},
                    }
                )
            finally:
                recorder.close()

            root = Path(tmp) / "episode_test"
            self.assertTrue((root / "episode_metadata.json").exists())
            state_rows = [json.loads(line) for line in (root / "robot_state.jsonl").read_text().splitlines()]
            action_rows = [json.loads(line) for line in (root / "actions.jsonl").read_text().splitlines()]
            self.assertEqual(state_rows[0]["payload"]["tick"], 1)
            self.assertEqual(action_rows[0]["nearest_state_tick"], 1)

    def test_training_extracts_fixed_width_state_and_pose_action(self) -> None:
        packet = {
            "left": {"mode": "TcpPoseTarget", "tcp_target_stand": [0.31, 0.22, 0.3, 0, 0, 0]},
            "right": {"mode": "TcpPoseTarget", "tcp_target_stand": [0.1, 0.2, 0.3, 0.1, 0, 0]},
        }
        self.assertEqual(len(state_vector(state_payload())), 40)
        self.assertEqual(len(action_vector(packet) or []), 12)

        with tempfile.TemporaryDirectory() as tmp:
            recorder = EpisodeRecorder(tmp, episode_name="episode_train")
            try:
                recorder.record_state(StateSnapshot(state_payload(), 1.0))
                recorder.record_action({"seq": 1, "mode": "TcpPoseTarget", **packet})
            finally:
                recorder.close()
            obs, actions = load_dataset(tmp)
            self.assertEqual(len(obs), 1)
            self.assertEqual(len(actions), 1)

    @unittest.skipIf(h5py is None or torch is None, "training extras not installed")
    def test_training_records_config_hashes_in_checkpoint(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            self._write_jsonl_episode(tmp)
            cartesian, kinematics = config_snapshots()
            self._write_hdf5_hashes(Path(tmp) / "ep_001.hdf5", cartesian, kinematics)

            checkpoint = Path(tmp) / "bc.pt"
            with warnings.catch_warnings(record=True) as caught:
                warnings.simplefilter("always")
                train_behavior_cloning(
                    episodes_dir=tmp,
                    checkpoint_path=checkpoint,
                    epochs=1,
                    batch_size=1,
                )

            saved = torch.load(checkpoint, map_location="cpu", weights_only=False)
            warning_messages = [str(warning.message) for warning in caught]
            self.assertFalse(
                any("std()" in message or "degrees of freedom" in message for message in warning_messages),
                warning_messages,
            )
            self.assertEqual(saved["schema"], "robotics_lab.policy_runner.bc_checkpoint.v2")
            self.assertEqual(saved["cartesian_control_hash"], _hash_canonical_json(cartesian))
            self.assertEqual(saved["kinematics_hash"], _hash_canonical_json(kinematics))
            obs_std = torch.tensor(saved["obs_std"])
            self.assertTrue(bool(torch.isfinite(obs_std).all()))
            self.assertTrue(bool(torch.allclose(obs_std, torch.full_like(obs_std, 1e-6))))

    @unittest.skipIf(h5py is None or torch is None, "training extras not installed")
    def test_training_warns_on_hdf5_config_hash_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            self._write_jsonl_episode(tmp)
            cartesian, kinematics = config_snapshots(path_kp_pos=6.0)
            changed, _ = config_snapshots(path_kp_pos=9.0)
            self._write_hdf5_hashes(Path(tmp) / "ep_001.hdf5", cartesian, kinematics)
            self._write_hdf5_hashes(Path(tmp) / "ep_002.hdf5", changed, kinematics)

            stderr = StringIO()
            with contextlib.redirect_stderr(stderr):
                train_behavior_cloning(
                    episodes_dir=tmp,
                    checkpoint_path=Path(tmp) / "bc.pt",
                    epochs=1,
                    batch_size=1,
                    strict_config_check=False,
                )

            self.assertIn("config hash mismatch", stderr.getvalue())
            self.assertIn("cartesian_control_hash", stderr.getvalue())

    @unittest.skipIf(h5py is None or torch is None, "training extras not installed")
    def test_training_aborts_on_hdf5_config_hash_mismatch_in_strict_mode(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            self._write_jsonl_episode(tmp)
            cartesian, kinematics = config_snapshots(path_kp_pos=6.0)
            changed, _ = config_snapshots(path_kp_pos=9.0)
            self._write_hdf5_hashes(Path(tmp) / "ep_001.hdf5", cartesian, kinematics)
            self._write_hdf5_hashes(Path(tmp) / "ep_002.hdf5", changed, kinematics)

            with self.assertRaises(RuntimeError):
                train_behavior_cloning(
                    episodes_dir=tmp,
                    checkpoint_path=Path(tmp) / "bc.pt",
                    epochs=1,
                    batch_size=1,
                    strict_config_check=True,
                )

    def _write_jsonl_episode(self, root: str | Path) -> None:
        packet = {
            "left": {"mode": "TcpPoseTarget", "tcp_target_stand": [0.31, 0.22, 0.3, 0, 0, 0]},
            "right": {"mode": "TcpPoseTarget", "tcp_target_stand": [0.1, 0.2, 0.3, 0.1, 0, 0]},
        }
        recorder = EpisodeRecorder(root, episode_name="episode_train")
        try:
            recorder.record_state(StateSnapshot(state_payload(), 1.0))
            recorder.record_action({"seq": 1, "mode": "TcpPoseTarget", **packet})
        finally:
            recorder.close()

    def _write_hdf5_hashes(self, path: Path, cartesian: dict, kinematics: dict) -> None:
        assert h5py is not None
        with h5py.File(path, "w") as handle:
            handle.attrs["cartesian_control_hash"] = _hash_canonical_json(cartesian)
            handle.attrs["kinematics_hash"] = _hash_canonical_json(kinematics)


if __name__ == "__main__":
    unittest.main()
