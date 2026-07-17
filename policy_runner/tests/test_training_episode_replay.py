from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import cv2
import h5py
import numpy as np
import yaml

try:
    from policy_runner.training_episode_replay import TrainingEpisodeReplay
    from policy_runner.openpi_remote import OpenpiRemoteActionSource
    from policy_runner.flow_inference import FlowMatchingActionSource
except Exception:  # optional OpenPI/ML dependencies may be absent in the base test env
    TrainingEpisodeReplay = None  # type: ignore[assignment]
    OpenpiRemoteActionSource = None  # type: ignore[assignment]
    FlowMatchingActionSource = None  # type: ignore[assignment]


def _encoded_color(value: int) -> np.ndarray:
    image = np.full((12, 16, 3), value, dtype=np.uint8)
    ok, encoded = cv2.imencode(".jpg", image)
    if not ok:
        raise RuntimeError("test JPEG encode failed")
    return encoded.reshape(-1)


def _encoded_depth(value: int) -> np.ndarray:
    image = np.full((12, 16), value, dtype=np.uint16)
    ok, encoded = cv2.imencode(".png", image)
    if not ok:
        raise RuntimeError("test depth encode failed")
    return encoded.reshape(-1)


def _write_episode(path: Path, frames: int = 30) -> None:
    variable = h5py.vlen_dtype(np.dtype("uint8"))
    with h5py.File(path, "w") as handle:
        handle.create_dataset("timestamp", data=np.arange(frames, dtype=np.float64) / 30.0)
        for arm_index, arm in enumerate(("left", "right")):
            group = handle.create_group(f"observations/{arm}")
            pose = np.zeros((frames, 7), dtype=np.float64)
            pose[:, 0] = np.arange(frames) * (0.001 + arm_index * 0.0005)
            pose[:, 6] = 1.0
            group.create_dataset("pose", data=pose)
            group.create_dataset("gripper", data=np.full((frames, 1), 60.0 + arm_index))
            images = group.create_group("images")
            color = images.create_dataset("realsense_color", shape=(frames,), dtype=variable)
            depth = images.create_dataset("realsense_depth", shape=(frames,), dtype=variable)
            for index in range(frames):
                color[index] = _encoded_color(20 + index)
                depth[index] = _encoded_depth(1000 + index)


@unittest.skipIf(TrainingEpisodeReplay is None, "training replay dependencies unavailable")
class TrainingEpisodeReplayTest(unittest.TestCase):
    def test_tracked_pgmode_profile_preserves_short_delta_window_endpoints(self) -> None:
        root = Path(__file__).resolve().parents[2]
        config = yaml.safe_load(
            (root / "rb_servo_server/config/stack_sim.yaml").read_text()
        )
        follower = config["cartesian_control"]["tcp_pose_target_profiles"][
            "flow_infer_smooth"
        ]["ruckig_follower"]
        self.assertEqual(follower["controller"], "delta_preview")
        self.assertEqual(follower["smoothing_window"], 1)

    def test_teacher_forced_samples_and_completion_artifacts(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            episode = root / "episode.hdf5"
            retarget = root / "retarget.yaml"
            output = root / "output"
            _write_episode(episode)
            retarget.write_text(
                "schema: robotics_lab.umi_retarget.v1\n"
                "left:\n  T_tcp_umi_gripper: [0, 0, 0, 0, 0, 0, 1]\n"
                "right:\n  T_tcp_umi_gripper: [0, 0, 0, 0, 0, 0, 1]\n"
            )
            replay = TrainingEpisodeReplay(
                episode,
                retarget_config=retarget,
                output_dir=output,
                depth_z_near_mm=50.0,
                depth_z_far_mm=700.0,
                depth_units_m=1e-4,
            )
            replay.configure(action_horizon=8, execute_steps=4)
            self.assertEqual(replay.total_execute_steps, 24)
            first = replay.next_sample()
            assert first is not None
            self.assertEqual(first.frame_index, 0)
            self.assertEqual(first.observation["observation/state"].shape, (12,))
            self.assertEqual(first.observation["observation/left_wrist_0_rgb"].shape, (12, 16, 3))
            self.assertEqual(first.observation["observation/left_wrist_0_depth"].shape, (12, 16, 3))
            self.assertEqual(first.ground_truth_chunk.shape, (8, 14))
            self.assertAlmostEqual(float(first.ground_truth_chunk[0, 0]), 0.001, places=6)
            self.assertAlmostEqual(float(first.ground_truth_chunk[0, 7]), 0.0015, places=6)

            sample = first
            while sample is not None:
                replay.record_prediction(sample, sample.ground_truth_chunk)
                sample = replay.next_sample()
            self.assertIsNone(replay.completion_reason(23))
            self.assertEqual(replay.completion_reason(24), "training_episode_replay_complete")
            replay.close()

            summary = json.loads((output / "teacher_forced_summary.json").read_text())
            self.assertEqual(summary["inferences_completed"], 6)
            self.assertEqual(summary["prediction_error"]["pose_mse"], 0.0)
            self.assertEqual(summary["image_source"], "raw_hdf5_jpeg_depth")
            with np.load(output / "teacher_forced_predictions.npz") as artifact:
                self.assertEqual(artifact["predictions"].shape, (6, 8, 14))
                self.assertTrue(np.array_equal(artifact["predictions"], artifact["ground_truth"]))

    def test_rejects_timestamp_gaps(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            episode = root / "episode.hdf5"
            retarget = root / "retarget.yaml"
            _write_episode(episode)
            with h5py.File(episode, "r+") as handle:
                handle["timestamp"][10:] += 1.0
            retarget.write_text(
                "schema: robotics_lab.umi_retarget.v1\n"
                "left:\n  T_tcp_umi_gripper: [0, 0, 0, 0, 0, 0, 1]\n"
                "right:\n  T_tcp_umi_gripper: [0, 0, 0, 0, 0, 0, 1]\n"
            )
            with self.assertRaisesRegex(ValueError, "timestamp gaps"):
                TrainingEpisodeReplay(
                    episode,
                    retarget_config=retarget,
                    output_dir=root / "output",
                )

    @unittest.skipIf(OpenpiRemoteActionSource is None, "OpenPI runtime dependencies unavailable")
    def test_openpi_source_marks_recorded_velocity_proprio_valid(self) -> None:
        class Provider:
            def __init__(self):
                self.sample = type(
                    "Sample",
                    (),
                    {
                        "frame_index": 7,
                        "observation": {
                            "observation/left_wrist_0_rgb": np.zeros((2, 2, 3), dtype=np.uint8),
                            "observation/right_wrist_0_rgb": np.zeros((2, 2, 3), dtype=np.uint8),
                            "observation/left_wrist_0_depth": np.zeros((2, 2, 3), dtype=np.uint8),
                            "observation/right_wrist_0_depth": np.zeros((2, 2, 3), dtype=np.uint8),
                            "observation/state": np.arange(12, dtype=np.float32),
                        },
                    },
                )()
                self.recorded = None

            def next_sample(self):
                return self.sample

            def record_prediction(self, sample, prediction):
                self.recorded = (sample, prediction.copy())

        class Client:
            def infer(self, observation):
                self.observation = observation
                return {"actions": np.zeros((4, 14), dtype=np.float32)}

        source = OpenpiRemoteActionSource.__new__(OpenpiRemoteActionSource)
        source.episode_observation_provider = Provider()
        source._client = Client()
        source.prompt = "task"
        source.action_horizon = 4
        source.stderr = __import__("sys").stderr
        source.last_image_decode_count = 0
        source.last_missing_camera_count = 0
        source.image_decode_count = 0
        source.missing_camera_count = 0
        source.rtc_enabled = False
        chunk = source._sample_chunk({})
        self.assertIsNotNone(chunk)
        self.assertTrue(source._last_velproprio_diagnostics["valid"])
        self.assertEqual(source._last_velproprio_diagnostics["frame_index"], 7)
        self.assertEqual(
            source._last_velproprio_diagnostics["arms"]["right"]["delta"],
            [float(value) for value in range(6, 12)],
        )
        self.assertIsNotNone(source.episode_observation_provider.recorded)

    def test_openpi_source_emits_hold_before_training_replay_completion(self) -> None:
        class Provider:
            @staticmethod
            def completion_reason(emitted_policy_steps):
                return "training_episode_replay_complete" if emitted_policy_steps >= 3 else None

        source = OpenpiRemoteActionSource.__new__(OpenpiRemoteActionSource)
        source.episode_observation_provider = Provider()
        source._stream_emitted_policy_steps = 3
        source._training_episode_completion_hold_emitted = False
        source._last_chunk_overlay_publish_monotonic = 1.0
        source._last_chunk_overlay_publish_execute_steps = 3
        source._last_chunk_overlay_publish_policy_dt_sec = 0.1
        source.timeout_sec = 0.2
        source._force_recovery_gate = lambda snapshot, now: (False, None)
        source._handle_server_motion_epoch = lambda snapshot: None
        source._camera_runtime_gate = lambda now: (False, None)

        self.assertIsNone(source.completion_reason())
        sentinel = object()
        assert FlowMatchingActionSource is not None
        with mock.patch.object(FlowMatchingActionSource, "next_intent", return_value=sentinel):
            self.assertIs(source.next_intent(None, 1.299), sentinel)
        self.assertFalse(source._training_episode_completion_hold_emitted)
        intent = source.next_intent(None, 1.3)
        self.assertEqual(intent.mode, "Hold")
        self.assertEqual(source.completion_reason(), "training_episode_replay_complete")

    def test_openpi_source_final_hold_fails_closed_without_overlay_metadata(self) -> None:
        source = OpenpiRemoteActionSource.__new__(OpenpiRemoteActionSource)
        source.episode_observation_provider = object()
        self.assertFalse(source._training_episode_final_overlay_elapsed(1.0))


if __name__ == "__main__":
    unittest.main()
