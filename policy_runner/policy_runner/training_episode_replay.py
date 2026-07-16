"""Teacher-forced OpenPI replay inputs from one recorded training episode.

This module deliberately owns only the *policy observation* side of replay.  The
predicted action chunk is still executed by :class:`OpenpiRemoteActionSource`, so
all normal controller-simulation conditioning, leases, kinematics and safety
filters remain in the path.
"""

from __future__ import annotations

import hashlib
import json
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import h5py
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from .openpi_remote import _depth_to_image


_ARMS = ("left", "right")
_STREAMS = (
    "left_wrist_0_rgb",
    "right_wrist_0_rgb",
    "left_wrist_0_depth",
    "right_wrist_0_depth",
)


@dataclass(frozen=True)
class TrainingEpisodeSample:
    frame_index: int
    observation: dict[str, np.ndarray]
    ground_truth_chunk: np.ndarray


class _VideoFrameReader:
    def __init__(self, path: Path):
        self.path = path
        self.capture = cv2.VideoCapture(str(path))
        if not self.capture.isOpened():
            raise ValueError(f"failed to open training video: {path}")
        self.frame_count = int(self.capture.get(cv2.CAP_PROP_FRAME_COUNT))
        self._next_index = 0

    def read_rgb(self, index: int) -> np.ndarray:
        if index < 0 or index >= self.frame_count:
            raise IndexError(f"video frame {index} outside [0,{self.frame_count}) for {self.path}")
        if self._next_index != index:
            if not self.capture.set(cv2.CAP_PROP_POS_FRAMES, float(index)):
                raise ValueError(f"failed to seek training video {self.path} to frame {index}")
        ok, bgr = self.capture.read()
        if not ok or bgr is None:
            raise ValueError(f"failed to decode training video {self.path} frame {index}")
        self._next_index = index + 1
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    def close(self) -> None:
        self.capture.release()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _bad_pose_mask(pose: np.ndarray) -> np.ndarray:
    finite = np.isfinite(pose).all(axis=1)
    qnorm = np.linalg.norm(pose[:, 3:7], axis=1)
    return ~(finite & (qnorm > 1e-6))


def _sanitize_pose(pose: np.ndarray) -> np.ndarray:
    """Match the OpenPI storage converter's forward-fill behavior exactly."""

    result = np.asarray(pose, dtype=np.float64).copy()
    bad = _bad_pose_mask(result)
    if not bad.any():
        return result
    identity = np.asarray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    last: np.ndarray | None = None
    for index in range(len(result)):
        if bad[index]:
            result[index] = identity if last is None else last
        else:
            last = result[index].copy()
    return result


def _load_tool_inverse(config_path: Path) -> dict[str, tuple[Rotation, np.ndarray]]:
    config = yaml.safe_load(config_path.read_text())
    schema = str(config.get("schema", "")) if isinstance(config, dict) else ""
    if not schema.startswith("robotics_lab.umi_retarget"):
        raise ValueError(f"{config_path}: unexpected retarget schema {schema!r}")
    result: dict[str, tuple[Rotation, np.ndarray]] = {}
    for arm in _ARMS:
        try:
            pose = np.asarray(config[arm]["T_tcp_umi_gripper"], dtype=np.float64)
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"{config_path}: missing {arm}.T_tcp_umi_gripper") from exc
        if pose.shape != (7,) or not np.all(np.isfinite(pose)):
            raise ValueError(f"{config_path}: invalid {arm}.T_tcp_umi_gripper")
        rotation_inverse = Rotation.from_quat(pose[3:7]).inv()
        result[arm] = (rotation_inverse, rotation_inverse.apply(-pose[:3]))
    return result


def _apply_tool_inverse(
    pose: np.ndarray, transform: tuple[Rotation, np.ndarray]
) -> np.ndarray:
    rotation_inverse, translation_inverse = transform
    rotation = Rotation.from_quat(pose[:, 3:7])
    position = pose[:, :3] + rotation.apply(translation_inverse)
    quaternion = (rotation * rotation_inverse).as_quat()
    return np.concatenate([position, quaternion], axis=1)


def _arm_velocity(pose: np.ndarray) -> np.ndarray:
    current = Rotation.from_quat(pose[:-1, 3:7])
    following = Rotation.from_quat(pose[1:, 3:7])
    translation = current.inv().apply(pose[1:, :3] - pose[:-1, :3])
    rotation = (current.inv() * following).as_rotvec()
    return np.vstack([np.zeros((1, 6)), np.concatenate([translation, rotation], axis=1)]).astype(
        np.float32
    )


def _arm_actions(pose: np.ndarray, gripper: np.ndarray) -> np.ndarray:
    current = Rotation.from_quat(pose[:-1, 3:7])
    following = Rotation.from_quat(pose[1:, 3:7])
    translation = current.inv().apply(pose[1:, :3] - pose[:-1, :3])
    rotation = (current.inv() * following).as_rotvec()
    return np.concatenate(
        [translation, rotation, (gripper[1:] / 100.0).reshape(-1, 1)], axis=1
    ).astype(np.float32)


def _decode_hdf5_color(value: Any) -> np.ndarray:
    bgr = cv2.imdecode(np.asarray(value, dtype=np.uint8), cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError("failed to decode HDF5 training RGB frame")
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def _decode_hdf5_depth(value: Any) -> np.ndarray:
    raw = cv2.imdecode(np.asarray(value, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    if raw is None:
        raise ValueError("failed to decode HDF5 training depth frame")
    return raw.astype(np.uint16, copy=False)


def _trajectory_metrics(actions: np.ndarray, arm_offset: int) -> dict[str, float]:
    steps = np.asarray(actions[:, arm_offset : arm_offset + 6], dtype=np.float64)
    rotation = Rotation.identity()
    position = np.zeros(3, dtype=np.float64)
    path_length = 0.0
    angular_path = 0.0
    max_translation = 0.0
    max_rotation = 0.0
    for step in steps:
        linear_norm = float(np.linalg.norm(step[:3]))
        angular_norm = float(np.linalg.norm(step[3:6]))
        path_length += linear_norm
        angular_path += angular_norm
        max_translation = max(max_translation, linear_norm)
        max_rotation = max(max_rotation, angular_norm)
        position += rotation.apply(step[:3])
        rotation = rotation * Rotation.from_rotvec(step[3:6])
    return {
        "translation_path_m": path_length,
        "translation_net_m": float(np.linalg.norm(position)),
        "rotation_path_rad": angular_path,
        "rotation_net_rad": float(rotation.magnitude()),
        "max_translation_step_m": max_translation,
        "max_rotation_step_rad": max_rotation,
    }


class TrainingEpisodeReplay:
    """Sequential teacher-forced observations and reference actions for one episode."""

    def __init__(
        self,
        hdf5_path: str | Path,
        *,
        retarget_config: str | Path,
        output_dir: str | Path,
        training_video_dir: str | Path | None = None,
        training_parquet: str | Path | None = None,
        depth_z_near_mm: float = 50.0,
        depth_z_far_mm: float = 700.0,
        depth_units_m: float = 1e-4,
        start_frame: int = 0,
    ):
        self.hdf5_path = Path(hdf5_path).resolve()
        self.retarget_config = Path(retarget_config).resolve()
        self.output_dir = Path(output_dir).resolve()
        self.training_video_dir = (
            Path(training_video_dir).resolve() if training_video_dir is not None else None
        )
        self.training_parquet = (
            Path(training_parquet).resolve() if training_parquet is not None else None
        )
        self.depth_z_near_mm = float(depth_z_near_mm)
        self.depth_z_far_mm = float(depth_z_far_mm)
        self.depth_units_m = float(depth_units_m)
        self.start_frame = int(start_frame)
        if not self.hdf5_path.is_file():
            raise ValueError(f"training episode HDF5 not found: {self.hdf5_path}")
        if not self.retarget_config.is_file():
            raise ValueError(f"retarget config not found: {self.retarget_config}")
        if not (0.0 <= self.depth_z_near_mm < self.depth_z_far_mm):
            raise ValueError("depth clip requires 0 <= near < far")
        if self.depth_units_m <= 0.0:
            raise ValueError("depth_units_m must be positive")
        self._video_readers: dict[str, _VideoFrameReader] = {}
        self._pending: TrainingEpisodeSample | None = None
        self._predictions: list[np.ndarray] = []
        self._inference_sec: list[float] = []
        self._prediction_started: float | None = None
        self._anchors: list[int] = []
        self._anchor_cursor = 0
        self.action_horizon: int | None = None
        self.execute_steps: int | None = None
        self._load_numeric_episode()
        self._open_training_videos()
        self._validate_training_parquet()
        self._hashes: dict[str, Any] = {
            "hdf5": _sha256(self.hdf5_path),
            "retarget": _sha256(self.retarget_config),
            "parquet": None if self.training_parquet is None else _sha256(self.training_parquet),
            "videos": {
                stream: _sha256(reader.path) for stream, reader in self._video_readers.items()
            },
        }

    def _load_numeric_episode(self) -> None:
        tool_inverse = _load_tool_inverse(self.retarget_config)
        with h5py.File(self.hdf5_path, "r") as handle:
            timestamps = np.asarray(handle["timestamp"], dtype=np.float64)
            pose: dict[str, np.ndarray] = {}
            gripper: dict[str, np.ndarray] = {}
            self._hdf5_color: dict[str, Any] = {}
            self._hdf5_depth: dict[str, Any] = {}
            for arm in _ARMS:
                raw_pose = np.asarray(handle[f"observations/{arm}/pose"], dtype=np.float64)
                bad_ratio = float(_bad_pose_mask(raw_pose).mean())
                if bad_ratio > 0.10:
                    raise ValueError(f"{arm} tracking dropout {bad_ratio:.1%} exceeds converter limit")
                pose[arm] = _apply_tool_inverse(_sanitize_pose(raw_pose), tool_inverse[arm])
                gripper[arm] = np.nan_to_num(
                    np.asarray(handle[f"observations/{arm}/gripper"], dtype=np.float64)[:, 0]
                )
                self._hdf5_color[arm] = f"observations/{arm}/images/realsense_color"
                self._hdf5_depth[arm] = f"observations/{arm}/images/realsense_depth"
            lengths = [len(timestamps), *(len(pose[arm]) for arm in _ARMS)]
            if len(set(lengths)) != 1 or lengths[0] < 2:
                raise ValueError(f"inconsistent or empty training episode lengths: {lengths}")
            gaps = np.diff(timestamps) > 0.1
            if gaps.any():
                raise ValueError(
                    "teacher-forced replay currently requires one clean segment; "
                    f"found {int(gaps.sum())} timestamp gaps >100 ms"
                )
            for arm in _ARMS:
                for path in (self._hdf5_color[arm], self._hdf5_depth[arm]):
                    if path not in handle or len(handle[path]) < lengths[0]:
                        raise ValueError(f"missing/incomplete training image stream: {path}")
        self.states = np.concatenate(
            [_arm_velocity(pose["left"]), _arm_velocity(pose["right"])], axis=1
        ).astype(np.float32)
        self.actions = np.concatenate(
            [
                _arm_actions(pose["left"], gripper["left"]),
                _arm_actions(pose["right"], gripper["right"]),
            ],
            axis=1,
        ).astype(np.float32)
        self.timestamps = timestamps
        self.frame_count = len(timestamps)

    def _open_training_videos(self) -> None:
        if self.training_video_dir is None:
            return
        for stream in _STREAMS:
            candidates = (self.training_video_dir / f"{stream}.mp4",)
            matches = [path for path in candidates if path.is_file()]
            if not matches:
                matches = sorted((self.training_video_dir / stream).glob("episode_*.mp4"))
            if len(matches) != 1:
                raise ValueError(
                    f"expected exactly one training MP4 for {stream} under {self.training_video_dir}, "
                    f"found {len(matches)}"
                )
            reader = _VideoFrameReader(matches[0])
            if reader.frame_count < self.frame_count - 1:
                raise ValueError(
                    f"training video {matches[0]} has {reader.frame_count} frames, "
                    f"need at least {self.frame_count - 1}"
                )
            self._video_readers[stream] = reader

    def _validate_training_parquet(self) -> None:
        if self.training_parquet is None:
            return
        if not self.training_parquet.is_file():
            raise ValueError(f"training parquet not found: {self.training_parquet}")
        try:
            import pyarrow.parquet as parquet
        except ImportError as exc:  # pragma: no cover - runtime environment guard
            raise ValueError("pyarrow is required to validate --training-episode-parquet") from exc
        table = parquet.read_table(self.training_parquet, columns=["state", "actions"])
        reference_states = np.asarray(table["state"].to_pylist(), dtype=np.float32)
        reference_actions = np.asarray(table["actions"].to_pylist(), dtype=np.float32)
        count = len(self.actions)
        if reference_states.shape[0] < count or reference_actions.shape[0] < count:
            raise ValueError("training parquet is shorter than the source episode")
        if not np.allclose(reference_states[:count], self.states[:count], atol=1e-6, rtol=0.0):
            error = float(np.max(np.abs(reference_states[:count] - self.states[:count])))
            raise ValueError(f"training parquet state does not match HDF5 conversion (max error {error})")
        if not np.allclose(reference_actions[:count], self.actions, atol=1e-6, rtol=0.0):
            error = float(np.max(np.abs(reference_actions[:count] - self.actions)))
            raise ValueError(f"training parquet action does not match HDF5 conversion (max error {error})")

    def configure(self, action_horizon: int, execute_steps: int) -> None:
        horizon = int(action_horizon)
        window = int(execute_steps)
        if horizon <= 0 or window <= 0 or window > horizon:
            raise ValueError("invalid teacher-forced replay horizon/window")
        last_anchor = len(self.actions) - horizon
        if self.start_frame < 0 or self.start_frame > last_anchor:
            raise ValueError(
                f"start_frame {self.start_frame} leaves no full H={horizon} action chunk "
                f"in {len(self.actions)} actions"
            )
        self.action_horizon = horizon
        self.execute_steps = window
        self._anchors = list(range(self.start_frame, last_anchor + 1, window))
        if not self._anchors:
            raise ValueError("training episode produced no replay anchors")
        self._write_artifacts()

    @property
    def total_execute_steps(self) -> int:
        return len(self._anchors) * int(self.execute_steps or 0)

    @property
    def replay_status(self) -> dict[str, Any]:
        return {
            "frame_index": None if self._pending is None else self._pending.frame_index,
            "inferences_completed": len(self._predictions),
            "inferences_total": len(self._anchors),
            "execute_steps_total": self.total_execute_steps,
            "image_source": "lerobot_training_h264" if self._video_readers else "raw_hdf5_jpeg_depth",
        }

    def _observation_images(self, frame_index: int) -> dict[str, np.ndarray]:
        if self._video_readers:
            return {stream: reader.read_rgb(frame_index) for stream, reader in self._video_readers.items()}
        result: dict[str, np.ndarray] = {}
        with h5py.File(self.hdf5_path, "r") as handle:
            for arm in _ARMS:
                result[f"{arm}_wrist_0_rgb"] = _decode_hdf5_color(
                    handle[self._hdf5_color[arm]][frame_index]
                )
                raw_depth = _decode_hdf5_depth(handle[self._hdf5_depth[arm]][frame_index])
                result[f"{arm}_wrist_0_depth"] = _depth_to_image(
                    raw_depth,
                    self.depth_z_near_mm,
                    self.depth_z_far_mm,
                    self.depth_units_m,
                )
        return result

    def next_sample(self) -> TrainingEpisodeSample | None:
        if self._pending is not None:
            return self._pending
        if self._anchor_cursor >= len(self._anchors):
            return None
        if self.action_horizon is None:
            raise RuntimeError("TrainingEpisodeReplay.configure must be called before sampling")
        frame_index = self._anchors[self._anchor_cursor]
        images = self._observation_images(frame_index)
        observation = {
            "observation/left_wrist_0_rgb": images["left_wrist_0_rgb"],
            "observation/right_wrist_0_rgb": images["right_wrist_0_rgb"],
            "observation/left_wrist_0_depth": images["left_wrist_0_depth"],
            "observation/right_wrist_0_depth": images["right_wrist_0_depth"],
            "observation/state": self.states[frame_index].copy(),
        }
        self._pending = TrainingEpisodeSample(
            frame_index=frame_index,
            observation=observation,
            ground_truth_chunk=self.actions[frame_index : frame_index + self.action_horizon].copy(),
        )
        self._prediction_started = time.perf_counter()
        return self._pending

    def record_prediction(self, sample: TrainingEpisodeSample, prediction: np.ndarray) -> None:
        if sample is not self._pending:
            raise ValueError("teacher-forced prediction does not match the pending observation")
        value = np.asarray(prediction, dtype=np.float32)
        expected = sample.ground_truth_chunk.shape
        if value.shape != expected or not np.all(np.isfinite(value)):
            raise ValueError(f"invalid teacher-forced prediction shape/value {value.shape}, expected {expected}")
        self._predictions.append(value.copy())
        started = self._prediction_started
        self._inference_sec.append(0.0 if started is None else time.perf_counter() - started)
        self._anchor_cursor += 1
        self._pending = None
        self._prediction_started = None
        self._write_artifacts()

    def completion_reason(self, emitted_policy_steps: int) -> str | None:
        if (
            self._anchors
            and len(self._predictions) == len(self._anchors)
            and int(emitted_policy_steps) >= self.total_execute_steps
        ):
            return "training_episode_replay_complete"
        return None

    def _summary(self) -> dict[str, Any]:
        predictions = np.asarray(self._predictions, dtype=np.float32)
        completed_anchors = self._anchors[: len(predictions)]
        ground_truth = np.asarray(
            [self.actions[index : index + int(self.action_horizon or 0)] for index in completed_anchors],
            dtype=np.float32,
        )
        summary: dict[str, Any] = {
            "schema": "robotics_lab.training_episode_teacher_forced_replay.v1",
            "source_hdf5": str(self.hdf5_path),
            "source_hdf5_sha256": self._hashes["hdf5"],
            "retarget_config": str(self.retarget_config),
            "retarget_config_sha256": self._hashes["retarget"],
            "training_video_dir": None if self.training_video_dir is None else str(self.training_video_dir),
            "training_parquet": None if self.training_parquet is None else str(self.training_parquet),
            "training_parquet_sha256": self._hashes["parquet"],
            "image_source": "lerobot_training_h264" if self._video_readers else "raw_hdf5_jpeg_depth",
            "frame_count": self.frame_count,
            "action_count": len(self.actions),
            "action_horizon": self.action_horizon,
            "execute_steps_per_chunk": self.execute_steps,
            "anchors": self._anchors,
            "inferences_completed": len(predictions),
            "inferences_total": len(self._anchors),
            "execute_steps_total": self.total_execute_steps,
            "inference_sec": self._inference_sec,
        }
        if self._video_readers:
            summary["training_video_sha256"] = self._hashes["videos"]
        if len(predictions):
            error = predictions - ground_truth
            summary["prediction_error"] = {
                "pose_mse": float(np.mean(np.square(error[:, :, [*range(6), *range(7, 13)]]))),
                "gripper_mse": float(np.mean(np.square(error[:, :, [6, 13]]))),
                "pose_mae": float(np.mean(np.abs(error[:, :, [*range(6), *range(7, 13)]]))),
                "gripper_mae": float(np.mean(np.abs(error[:, :, [6, 13]]))),
            }
            window = int(self.execute_steps or 0)
            predicted_executed = np.concatenate([chunk[:window] for chunk in predictions], axis=0)
            truth_executed = np.concatenate([chunk[:window] for chunk in ground_truth], axis=0)
            summary["executed_window_trajectory"] = {
                "predicted": {
                    "left": _trajectory_metrics(predicted_executed, 0),
                    "right": _trajectory_metrics(predicted_executed, 7),
                },
                "ground_truth": {
                    "left": _trajectory_metrics(truth_executed, 0),
                    "right": _trajectory_metrics(truth_executed, 7),
                },
            }
        return summary

    def _write_artifacts(self) -> None:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        summary_path = self.output_dir / "teacher_forced_summary.json"
        temporary_summary = summary_path.with_suffix(".json.tmp")
        temporary_summary.write_text(json.dumps(self._summary(), indent=2, sort_keys=True) + "\n")
        os.replace(temporary_summary, summary_path)
        predictions = np.asarray(self._predictions, dtype=np.float32)
        anchors = np.asarray(self._anchors[: len(predictions)], dtype=np.int64)
        ground_truth = np.asarray(
            [self.actions[index : index + int(self.action_horizon or 0)] for index in anchors],
            dtype=np.float32,
        )
        npz_path = self.output_dir / "teacher_forced_predictions.npz"
        temporary_npz = self.output_dir / "teacher_forced_predictions.tmp.npz"
        np.savez_compressed(
            temporary_npz,
            anchors=anchors,
            predictions=predictions,
            ground_truth=ground_truth,
            states=self.states[anchors] if len(anchors) else np.zeros((0, 12), dtype=np.float32),
        )
        os.replace(temporary_npz, npz_path)

    def close(self) -> None:
        self._write_artifacts()
        for reader in self._video_readers.values():
            reader.close()
