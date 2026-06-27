from __future__ import annotations

import glob
import io
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import h5py
import numpy as np
from PIL import Image


FLOW_CHECKPOINT_SCHEMA = "robotics_lab.policy_runner.flow_matching.v1"
FLOW_ACTION_DIM = 14
FLOW_ARM_DIM = 7
FLOW_PROPRIO_DIM = 16
DEFAULT_IMAGE_SIZE = 224
# Action/proprio are expressed only in the end-effector body frame ("ee_local").
# The old world-frame "stand" representation was removed: with steamvr->stand
# unmeasured (T_stand_source == I) a "stand" delta carried the unmeasured world
# rotation and was not a valid robot-TCP action. ee_local is invariant to that
# rotation (see wiki umi-tcp-delta-frame).
DEFAULT_ACTION_FRAME = "ee_local"
ACTION_FRAMES = ("ee_local",)
IMAGE_CROP_MODES = ("none", "center_square")
HDF5_EPISODE_SUFFIXES = {".hdf5", ".h5"}
HDF5_SKIP_PARENT_NAMES = {
    ".tmp",
    "audit",
    "audits",
    "checkpoint",
    "checkpoints",
    "report",
    "reports",
    "temp",
    "temporary",
    "tmp",
}
HDF5_SKIP_EXACT_NAMES = {
    "audit.h5",
    "audit.hdf5",
    "conversion_report.h5",
    "conversion_report.hdf5",
    "hdf5_audit.h5",
    "hdf5_audit.hdf5",
    "manifest.h5",
    "manifest.hdf5",
}
HDF5_SKIP_STEM_PREFIXES = (
    "audit",
    "checkpoint",
    "ckpt",
    "conversion_report",
    "hdf5_audit",
    "model_checkpoint",
    "tmp",
    "temp",
)
HDF5_SKIP_STEM_SUFFIXES = (
    ".partial",
    ".tmp",
    "-partial",
    "-tmp",
    "_partial",
    "_tmp",
)
FLOW_ACTION_DIM_NAMES = (
    "left_dx",
    "left_dy",
    "left_dz",
    "left_drx",
    "left_dry",
    "left_drz",
    "left_grip",
    "right_dx",
    "right_dy",
    "right_dz",
    "right_drx",
    "right_dry",
    "right_drz",
    "right_grip",
)


@dataclass(frozen=True)
class FlowSampleRef:
    episode_index: int
    start: int


@dataclass
class FlowEpisodeIndex:
    path: Path
    format_name: str
    length: int
    timestamps: np.ndarray
    camera_paths: dict[str, str]
    left_pose: np.ndarray
    right_pose: np.ndarray
    left_gripper: np.ndarray
    right_gripper: np.ndarray
    arm_mask: np.ndarray
    reset_left_pose: np.ndarray
    reset_right_pose: np.ndarray
    action_kind: str
    action_left_pose: np.ndarray | None = None
    action_right_pose: np.ndarray | None = None
    action_left_delta: np.ndarray | None = None
    action_right_delta: np.ndarray | None = None
    action_left_gripper: np.ndarray | None = None
    action_right_gripper: np.ndarray | None = None


class FlowHdf5Dataset:
    """Image-conditioned action-chunk dataset for policy_runner flow matching."""

    def __init__(
        self,
        episodes_dir: str | Path,
        *,
        action_horizon: int = 16,
        image_size: int = DEFAULT_IMAGE_SIZE,
        image_crop: str = "none",
        camera_names: list[str] | None = None,
        single_arm_side: str = "left",
        include_formats: list[str] | tuple[str, ...] | None = None,
        exclude_camera_names: list[str] | tuple[str, ...] | None = None,
        required_attrs: dict[str, Any] | None = None,
        episode_paths: list[str] | tuple[str, ...] | None = None,
        include_patterns: list[str] | tuple[str, ...] | None = None,
        max_episodes: int | None = None,
        stats: dict[str, Any] | None = None,
        normalize: bool = True,
        action_frame: str = DEFAULT_ACTION_FRAME,
    ):
        if action_horizon <= 0:
            raise ValueError("action_horizon must be positive")
        if image_size <= 0:
            raise ValueError("image_size must be positive")
        if single_arm_side not in {"left", "right"}:
            raise ValueError("single_arm_side must be left or right")
        if max_episodes is not None and max_episodes <= 0:
            raise ValueError("max_episodes must be positive")
        self.root = Path(episodes_dir)
        self.action_horizon = int(action_horizon)
        self.image_size = int(image_size)
        self.action_frame = normalize_action_frame(action_frame)
        self._phase_boundary_cache: dict[str, Any] = {}
        if image_crop not in IMAGE_CROP_MODES:
            raise ValueError(f"image_crop must be one of: {', '.join(IMAGE_CROP_MODES)}")
        self.image_crop = image_crop
        self.single_arm_side = single_arm_side
        self.normalize = bool(normalize)
        self.stats = stats
        self.required_attrs = dict(required_attrs or {})
        self.episodes = [
            load_flow_episode_index(path, single_arm_side=single_arm_side)
            for path in discover_hdf5_episodes(
                self.root,
                episode_paths=episode_paths,
                include_patterns=include_patterns,
            )
        ]
        if include_formats:
            allowed_formats = set(include_formats)
            self.episodes = [
                episode for episode in self.episodes if episode.format_name in allowed_formats
            ]
        if max_episodes is not None:
            self.episodes = self.episodes[: int(max_episodes)]
        if self.required_attrs:
            for episode in self.episodes:
                _validate_required_attrs(episode.path, self.required_attrs)
        if not self.episodes:
            raise ValueError(f"no HDF5 episodes found under {self.root}")

        discovered_cameras = sorted(
            {name for episode in self.episodes for name in episode.camera_paths}
        )
        selected_cameras = list(camera_names) if camera_names is not None else discovered_cameras
        excluded_cameras = set(exclude_camera_names or [])
        self.camera_names = [
            name for name in selected_cameras if name not in excluded_cameras
        ]
        self.sample_refs: list[FlowSampleRef] = []
        for episode_index, episode in enumerate(self.episodes):
            sample_count = max(0, episode.length - self.action_horizon)
            self.sample_refs.extend(
                FlowSampleRef(episode_index, start) for start in range(sample_count)
            )
        if not self.sample_refs:
            raise ValueError(
                f"no samples with action_horizon={self.action_horizon} under {self.root}"
            )

    @property
    def proprio_dim(self) -> int:
        return FLOW_PROPRIO_DIM

    def __len__(self) -> int:
        return len(self.sample_refs)

    def __getitem__(self, index: int) -> dict[str, np.ndarray]:
        sample = self.raw_sample(index)
        if self.normalize and self.stats:
            sample = normalize_flow_sample(sample, self.stats)
        return sample

    def raw_sample(self, index: int) -> dict[str, np.ndarray]:
        ref = self.sample_refs[index]
        episode = self.episodes[ref.episode_index]
        images, decode_count, missing_count = _load_images(
            episode.path,
            episode.camera_paths,
            self.camera_names,
            ref.start,
            self.image_size,
            self.image_crop,
        )
        proprio = _proprio_vector(episode, ref.start, action_frame=self.action_frame)
        action_chunk = _action_chunk(
            episode,
            ref.start,
            self.action_horizon,
            action_frame=self.action_frame,
        )
        action_mask = action_mask_from_arm_mask(episode.arm_mask, self.action_horizon)
        return {
            "images": images.astype(np.float32, copy=False),
            "proprio": proprio.astype(np.float32, copy=False),
            "action_chunk": action_chunk.astype(np.float32, copy=False),
            "action_mask": action_mask.astype(np.float32, copy=False),
            "arm_mask": episode.arm_mask.astype(np.float32, copy=False),
            "image_decode_count": np.asarray(decode_count, dtype=np.int64),
            "missing_camera_count": np.asarray(missing_count, dtype=np.int64),
        }

    def phase_boundaries_for_episode(self, episode_index: int):
        from .phase_segmentation import extract_phase_boundaries

        episode = self.episodes[int(episode_index)]
        key = str(episode.path)
        boundaries = self._phase_boundary_cache.get(key)
        if boundaries is None:
            boundaries = extract_phase_boundaries(
                episode.left_gripper,
                episode.right_gripper,
                episode.length,
            )
            self._phase_boundary_cache[key] = boundaries
        return boundaries


def discover_hdf5_episodes(
    root: str | Path,
    *,
    episode_paths: list[str] | tuple[str, ...] | None = None,
    include_patterns: list[str] | tuple[str, ...] | None = None,
    skip_non_episode_files: bool = True,
) -> list[Path]:
    path = Path(root)
    if path.is_file():
        return [path] if path.suffix.lower() in HDF5_EPISODE_SUFFIXES else []

    if episode_paths:
        return _manifest_listed_hdf5_episodes(path, episode_paths)

    if include_patterns:
        candidates = _manifest_pattern_hdf5_candidates(path, include_patterns)
    else:
        candidates = path.rglob("*")

    return sorted(
        dict.fromkeys(
            candidate
            for candidate in candidates
            if candidate.is_file()
            and candidate.suffix.lower() in HDF5_EPISODE_SUFFIXES
            and (not skip_non_episode_files or not _is_obvious_non_episode_hdf5(candidate))
        )
    )


def normalize_action_frame(action_frame: str) -> str:
    frame = str(action_frame or "").strip().lower()
    if frame not in ACTION_FRAMES:
        raise ValueError(f"action_frame must be one of: {', '.join(ACTION_FRAMES)}")
    return frame


def _manifest_listed_hdf5_episodes(root: Path, episode_paths: list[str] | tuple[str, ...]) -> list[Path]:
    paths: list[Path] = []
    missing: list[str] = []
    for entry in episode_paths:
        candidate = Path(entry)
        if not candidate.is_absolute():
            candidate = root / candidate
        if not candidate.is_file() or candidate.suffix.lower() not in HDF5_EPISODE_SUFFIXES:
            missing.append(str(candidate))
            continue
        paths.append(candidate)
    if missing:
        raise ValueError("manifest-listed HDF5 episode is missing or not an HDF5 file: " + ", ".join(missing))
    return sorted(dict.fromkeys(paths))


def _manifest_pattern_hdf5_candidates(root: Path, include_patterns: list[str] | tuple[str, ...]):
    candidates: list[Path] = []
    for pattern in include_patterns:
        pattern_path = Path(pattern)
        if pattern_path.is_absolute():
            candidates.extend(Path(item) for item in glob.glob(str(pattern_path), recursive=True))
        else:
            candidates.extend(root.glob(str(pattern)))
    return candidates


def _is_obvious_non_episode_hdf5(path: Path) -> bool:
    name = path.name.lower()
    stem = path.stem.lower()
    if name in HDF5_SKIP_EXACT_NAMES:
        return True
    if path.parent.name.lower() in HDF5_SKIP_PARENT_NAMES:
        return True
    return stem.startswith(HDF5_SKIP_STEM_PREFIXES) or stem.endswith(HDF5_SKIP_STEM_SUFFIXES)


def load_flow_episode_index(
    path: str | Path,
    *,
    single_arm_side: str = "left",
) -> FlowEpisodeIndex:
    episode_path = Path(path)
    with h5py.File(episode_path, "r") as handle:
        if _is_pika_umi_bimanual(handle):
            return _load_pika_bimanual_episode(episode_path, handle)
        if _is_pika_umi(handle):
            return _load_pika_episode(episode_path, handle, single_arm_side)
        if _is_robotics_lab(handle):
            return _load_robotics_lab_episode(episode_path, handle)
    raise ValueError(f"unsupported HDF5 episode format: {episode_path}")


def _validate_required_attrs(path: Path, required_attrs: dict[str, Any]) -> None:
    with h5py.File(path, "r") as handle:
        for key, expected in required_attrs.items():
            if key not in handle.attrs:
                raise ValueError(f"{path}: missing required HDF5 attr {key!r}")
            actual = _decode_hdf5_attr(handle.attrs.get(key))
            if actual != str(expected):
                raise ValueError(
                    f"{path}: required HDF5 attr {key!r}={expected!r}, got {actual!r}"
                )


def compute_dataset_statistics(
    dataset: FlowHdf5Dataset,
    *,
    max_samples: int | None = None,
) -> dict[str, Any]:
    sample_count = len(dataset) if max_samples is None else min(len(dataset), max_samples)
    if sample_count <= 0:
        raise ValueError("cannot compute statistics for an empty dataset")

    proprio_values: list[np.ndarray] = []
    action_sum = np.zeros(FLOW_ACTION_DIM, dtype=np.float64)
    action_sq_sum = np.zeros(FLOW_ACTION_DIM, dtype=np.float64)
    action_count = np.zeros(FLOW_ACTION_DIM, dtype=np.float64)
    image_sum = np.zeros(3, dtype=np.float64)
    image_sq_sum = np.zeros(3, dtype=np.float64)
    image_count = np.zeros(3, dtype=np.float64)
    image_decode_count = 0
    missing_camera_count = 0
    arm_counts = np.zeros(2, dtype=np.int64)
    action_values_by_dim: list[list[np.ndarray]] = [[] for _ in range(FLOW_ACTION_DIM)]

    for index in range(sample_count):
        sample = dataset.raw_sample(index)
        proprio_values.append(sample["proprio"].astype(np.float64))
        actions = sample["action_chunk"].astype(np.float64)
        mask = sample["action_mask"].astype(np.float64)
        action_sum += (actions * mask).sum(axis=0)
        action_sq_sum += ((actions * actions) * mask).sum(axis=0)
        action_count += mask.sum(axis=0)
        for dim in range(FLOW_ACTION_DIM):
            active_values = actions[:, dim][mask[:, dim] > 0.0]
            if active_values.size:
                action_values_by_dim[dim].append(active_values)
        images = sample["images"].astype(np.float64)
        if images.size:
            # Images are V,C,H,W in [0,1].
            image_sum += images.sum(axis=(0, 2, 3))
            image_sq_sum += (images * images).sum(axis=(0, 2, 3))
            image_count += images.shape[0] * images.shape[2] * images.shape[3]
        image_decode_count += int(sample["image_decode_count"])
        missing_camera_count += int(sample["missing_camera_count"])
        arm_counts += sample["arm_mask"].astype(np.int64)

    proprio_stack = np.stack(proprio_values, axis=0)
    proprio_mean = proprio_stack.mean(axis=0)
    proprio_std = np.maximum(proprio_stack.std(axis=0), 1e-6)

    action_mean = np.divide(
        action_sum,
        action_count,
        out=np.zeros_like(action_sum),
        where=action_count > 0,
    )
    action_var = np.divide(
        action_sq_sum,
        action_count,
        out=np.zeros_like(action_sq_sum),
        where=action_count > 0,
    ) - action_mean * action_mean
    action_std = np.sqrt(np.maximum(action_var, 1e-12))
    action_std[action_count == 0] = 1.0

    image_mean = np.divide(
        image_sum,
        image_count,
        out=np.zeros_like(image_sum),
        where=image_count > 0,
    )
    image_var = np.divide(
        image_sq_sum,
        image_count,
        out=np.zeros_like(image_sq_sum),
        where=image_count > 0,
    ) - image_mean * image_mean
    image_std = np.sqrt(np.maximum(image_var, 1e-12))
    image_std[image_count == 0] = 1.0

    formats = sorted({episode.format_name for episode in dataset.episodes})
    dt_values = _dataset_dt_values_sec(dataset.episodes)
    action_percentiles: dict[str, dict[str, float]] = {}
    for dim, name in enumerate(FLOW_ACTION_DIM_NAMES):
        if action_values_by_dim[dim]:
            values = np.concatenate(action_values_by_dim[dim]).astype(np.float64)
            percentiles = np.percentile(values, [1, 5, 50, 95, 99])
        else:
            percentiles = np.zeros(5, dtype=np.float64)
        action_percentiles[name] = {
            "p01": float(percentiles[0]),
            "p05": float(percentiles[1]),
            "p50": float(percentiles[2]),
            "p95": float(percentiles[3]),
            "p99": float(percentiles[4]),
        }
    return {
        "schema": FLOW_CHECKPOINT_SCHEMA + ".dataset_stats",
        "episode_count": len(dataset.episodes),
        "frame_count": int(sum(episode.length for episode in dataset.episodes)),
        "total_sample_count": len(dataset),
        "sample_count": sample_count,
        "action_horizon": dataset.action_horizon,
        "image_size": dataset.image_size,
        "image_crop": dataset.image_crop,
        "camera_names": dataset.camera_names,
        "formats": formats,
        "proprio_action_frame": dataset.action_frame,
        "proprio_dim": FLOW_PROPRIO_DIM,
        "action_dim": FLOW_ACTION_DIM,
        "proprio_mean": proprio_mean.astype(float).tolist(),
        "proprio_std": proprio_std.astype(float).tolist(),
        "action_mean": action_mean.astype(float).tolist(),
        "action_std": action_std.astype(float).tolist(),
        "image_mean": image_mean.astype(float).tolist(),
        "image_std": image_std.astype(float).tolist(),
        "image_decode_count": int(image_decode_count),
        "missing_camera_count": int(missing_camera_count),
        "arm_mask_counts": {"left": int(arm_counts[0]), "right": int(arm_counts[1])},
        "action_distribution_percentiles": action_percentiles,
        "dt_mean_sec": float(np.mean(dt_values)) if dt_values.size else None,
        "dt_p50_sec": float(np.percentile(dt_values, 50)) if dt_values.size else None,
    }


def write_dataset_statistics(path: str | Path, stats: dict[str, Any]) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(stats, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def normalize_flow_sample(sample: dict[str, np.ndarray], stats: dict[str, Any]) -> dict[str, np.ndarray]:
    out = dict(sample)
    out["proprio"] = _normalize_array(
        sample["proprio"],
        stats.get("proprio_mean", []),
        stats.get("proprio_std", []),
    )
    out["action_chunk"] = _normalize_array(
        sample["action_chunk"],
        stats.get("action_mean", []),
        stats.get("action_std", []),
    )
    images = sample["images"]
    if images.size:
        mean = np.asarray(stats.get("image_mean", [0.0, 0.0, 0.0]), dtype=np.float32)
        std = np.asarray(stats.get("image_std", [1.0, 1.0, 1.0]), dtype=np.float32)
        out["images"] = (images - mean.reshape(1, 3, 1, 1)) / np.maximum(
            std.reshape(1, 3, 1, 1),
            1e-6,
        )
    return out


def denormalize_actions(actions: Any, stats: dict[str, Any]):
    import torch

    mean = torch.as_tensor(stats["action_mean"], dtype=actions.dtype, device=actions.device)
    std = torch.as_tensor(stats["action_std"], dtype=actions.dtype, device=actions.device)
    return actions * std.view(1, 1, -1) + mean.view(1, 1, -1)


def action_mask_from_arm_mask(arm_mask: np.ndarray, action_horizon: int) -> np.ndarray:
    mask = np.zeros((action_horizon, FLOW_ACTION_DIM), dtype=np.float32)
    if float(arm_mask[0]) > 0.0:
        mask[:, 0:FLOW_ARM_DIM] = 1.0
    if float(arm_mask[1]) > 0.0:
        mask[:, FLOW_ARM_DIM : 2 * FLOW_ARM_DIM] = 1.0
    return mask


def runtime_proprio_from_state(
    payload: dict[str, Any],
    *,
    reset_left_pose: np.ndarray,
    reset_right_pose: np.ndarray,
    arm_mask: np.ndarray,
    action_frame: str = DEFAULT_ACTION_FRAME,
) -> np.ndarray:
    frame = normalize_action_frame(action_frame)
    delta_fn = pose_delta_local
    left_pose = _pose_from_state_arm(payload.get("left", {}))
    right_pose = _pose_from_state_arm(payload.get("right", {}))
    left_gripper = _float_or_zero(_nested_first(payload.get("left", {}), "gripper", "gripper_position"))
    right_gripper = _float_or_zero(_nested_first(payload.get("right", {}), "gripper", "gripper_position"))
    left_features = np.concatenate([delta_fn(reset_left_pose, left_pose), [left_gripper]])
    right_features = np.concatenate([delta_fn(reset_right_pose, right_pose), [right_gripper]])
    return np.concatenate([left_features, right_features, arm_mask.astype(np.float32)]).astype(np.float32)


def pose_from_state_payload(payload: dict[str, Any], side: str) -> np.ndarray:
    if side not in {"left", "right"}:
        raise ValueError("side must be left or right")
    return _pose_from_state_arm(payload.get(side, {}))


def normalize_runtime_proprio(proprio: np.ndarray, stats: dict[str, Any]) -> np.ndarray:
    return _normalize_array(
        proprio.astype(np.float32),
        stats.get("proprio_mean", []),
        stats.get("proprio_std", []),
    )


def decode_hdf5_image_value(
    value: Any,
    *,
    image_size: int = DEFAULT_IMAGE_SIZE,
    image_crop: str = "none",
) -> np.ndarray:
    if isinstance(value, np.ndarray) and value.ndim == 2:
        image = Image.fromarray(value)
    elif isinstance(value, np.ndarray) and value.dtype == np.uint8 and value.ndim == 3:
        if value.shape[2] == 1:
            image = Image.fromarray(value[:, :, 0], mode="L")
        elif value.shape[2] == 4:
            image = Image.fromarray(value, mode="RGBA")
        else:
            image = Image.fromarray(value[:, :, :3], mode="RGB")
    elif isinstance(value, np.ndarray) and value.dtype == np.uint8 and value.ndim == 1:
        image = Image.open(io.BytesIO(value.tobytes()))
    elif isinstance(value, (bytes, bytearray)):
        image = Image.open(io.BytesIO(bytes(value)))
    else:
        image = Image.open(io.BytesIO(bytes(value)))

    if image.mode in {"I;16", "I", "F", "L"}:
        arr = np.asarray(image)
        if arr.dtype == np.uint16:
            arr8 = np.clip(arr.astype(np.uint32) >> 8, 0, 255).astype(np.uint8)
        else:
            arrf = arr.astype(np.float32)
            min_value = float(np.nanmin(arrf)) if arrf.size else 0.0
            max_value = float(np.nanmax(arrf)) if arrf.size else 1.0
            arr8 = np.clip((arrf - min_value) / max(max_value - min_value, 1.0) * 255.0, 0, 255).astype(np.uint8)
        image = Image.fromarray(arr8, mode="L").convert("RGB")
    else:
        image = image.convert("RGB")
    if image_crop == "center_square":
        width, height = image.size
        side = min(width, height)
        left = max(0, (width - side) // 2)
        top = max(0, (height - side) // 2)
        image = image.crop((left, top, left + side, top + side))
    elif image_crop != "none":
        raise ValueError(f"image_crop must be one of: {', '.join(IMAGE_CROP_MODES)}")
    image = image.resize((image_size, image_size), Image.BILINEAR)
    arr = np.asarray(image, dtype=np.float32) / 255.0
    return np.transpose(arr, (2, 0, 1))


def _is_pika_umi(handle: h5py.File) -> bool:
    return (
        "action" in handle
        and "timestamp" in handle
        and "observations" in handle
        and "pose" in handle["observations"]
        and "images" in handle["observations"]
    )


def _is_pika_umi_bimanual(handle: h5py.File) -> bool:
    if "timestamp" not in handle or "observations" not in handle:
        return False
    observations = handle["observations"]
    if not isinstance(observations, h5py.Group):
        return False
    arm_groups = _pika_bimanual_arm_groups(handle)
    return any(
        group_path in handle
        and "pose" in handle[group_path]
        for group_path in arm_groups.values()
    )


def _is_robotics_lab(handle: h5py.File) -> bool:
    schema = str(handle.attrs.get("schema", "") or "")
    return schema == "robotics_lab.episode.v1" and "observations" in handle


def _load_pika_episode(
    path: Path,
    handle: h5py.File,
    single_arm_side: str,
) -> FlowEpisodeIndex:
    action = np.asarray(handle["action"], dtype=np.float32)
    pose = np.asarray(handle["observations/pose"], dtype=np.float32)
    gripper = np.asarray(handle["observations/gripper"], dtype=np.float32) if "observations/gripper" in handle else None
    timestamps = np.asarray(handle["timestamp"], dtype=np.float64)
    camera_paths = _camera_paths(handle, "observations/images")
    length_candidates = [len(action), len(pose), len(timestamps)] + _camera_lengths(handle, camera_paths)
    if gripper is not None and gripper.size:
        length_candidates.append(int(gripper.shape[0]))
    length = min(length_candidates)
    identity_pose = _identity_poses(length)
    zero_gripper = np.zeros(length, dtype=np.float32)
    arm_mask = np.asarray([1.0, 0.0] if single_arm_side == "left" else [0.0, 1.0], dtype=np.float32)
    left_pose = pose[:length] if single_arm_side == "left" else identity_pose.copy()
    right_pose = pose[:length] if single_arm_side == "right" else identity_pose.copy()
    left_action_pose = action[:length, :7] if single_arm_side == "left" else None
    right_action_pose = action[:length, :7] if single_arm_side == "right" else None
    current_gripper = _gripper_vector_or_zero(gripper, length)
    target_gripper = action[:length, 7] if action.shape[1] > 7 else current_gripper
    return FlowEpisodeIndex(
        path=path,
        format_name="pika_umi_single_arm",
        length=length,
        timestamps=timestamps[:length],
        camera_paths=camera_paths,
        left_pose=left_pose,
        right_pose=right_pose,
        left_gripper=current_gripper if single_arm_side == "left" else zero_gripper,
        right_gripper=current_gripper if single_arm_side == "right" else zero_gripper,
        arm_mask=arm_mask,
        reset_left_pose=left_pose[0],
        reset_right_pose=right_pose[0],
        action_kind="target_pose",
        action_left_pose=left_action_pose,
        action_right_pose=right_action_pose,
        action_left_gripper=target_gripper if single_arm_side == "left" else zero_gripper,
        action_right_gripper=target_gripper if single_arm_side == "right" else zero_gripper,
    )


def _load_pika_bimanual_episode(path: Path, handle: h5py.File) -> FlowEpisodeIndex:
    arm_groups = _pika_bimanual_arm_groups(handle)
    timestamps = np.asarray(handle["timestamp"], dtype=np.float64)
    length_candidates = [len(timestamps)]
    camera_paths: dict[str, str] = {}
    for side, group_path in arm_groups.items():
        group = handle[group_path]
        for name in ("pose", "gripper", "action"):
            if name in group:
                length_candidates.append(int(group[name].shape[0]))
        side_camera_paths = _camera_paths(handle, f"{group_path}/images") if "images" in group else {}
        for camera_name, camera_path in side_camera_paths.items():
            camera_paths[f"{side}_{camera_name}"] = camera_path
    length_candidates.extend(_camera_lengths(handle, camera_paths))
    length = min(length_candidates) if length_candidates else 0

    left_pose, left_gripper, left_action_pose, left_action_gripper = _pika_bimanual_arm_arrays(
        handle,
        arm_groups.get("left"),
        length,
    )
    right_pose, right_gripper, right_action_pose, right_action_gripper = _pika_bimanual_arm_arrays(
        handle,
        arm_groups.get("right"),
        length,
    )
    arm_mask = np.asarray(
        [
            1.0 if arm_groups.get("left") is not None else 0.0,
            1.0 if arm_groups.get("right") is not None else 0.0,
        ],
        dtype=np.float32,
    )
    if not arm_mask.any():
        arm_mask[:] = 1.0
    return FlowEpisodeIndex(
        path=path,
        format_name="pika_umi_bimanual",
        length=length,
        timestamps=timestamps[:length],
        camera_paths=camera_paths,
        left_pose=left_pose,
        right_pose=right_pose,
        left_gripper=left_gripper,
        right_gripper=right_gripper,
        arm_mask=arm_mask,
        reset_left_pose=left_pose[0] if length else _identity_poses(1)[0],
        reset_right_pose=right_pose[0] if length else _identity_poses(1)[0],
        action_kind="target_pose",
        action_left_pose=left_action_pose,
        action_right_pose=right_action_pose,
        action_left_gripper=left_action_gripper,
        action_right_gripper=right_action_gripper,
    )


def _load_robotics_lab_episode(path: Path, handle: h5py.File) -> FlowEpisodeIndex:
    obs = handle["observations"]
    left_pose = _dataset_or_identity(obs, "tcp_stand_left", "tcp_actual_stand_left")
    right_pose = _dataset_or_identity(obs, "tcp_stand_right", "tcp_actual_stand_right", length=len(left_pose))
    length = min(len(left_pose), len(right_pose))
    timestamps = _robotics_timestamps(obs, length)
    left_gripper = _optional_vector(obs, length, "gripper_left", "left_gripper")
    right_gripper = _optional_vector(obs, length, "gripper_right", "right_gripper")
    camera_paths = _camera_paths(handle, "observations/images") if "images" in obs else {}
    length = min([length] + _camera_lengths(handle, camera_paths)) if camera_paths else length
    action_group = handle["action"] if "action" in handle else None
    left_target_pose = right_target_pose = None
    left_target_grip = right_target_grip = None
    action_kind = "target_pose"
    if action_group is not None:
        left_target_pose = _first_dataset(action_group, length, "tcp_target_stand_left")
        right_target_pose = _first_dataset(action_group, length, "tcp_target_stand_right")
        left_target_grip = _optional_vector(action_group, length, "gripper_left", "left_gripper")
        right_target_grip = _optional_vector(action_group, length, "gripper_right", "right_gripper")
    if left_target_pose is None and action_kind == "target_pose":
        left_target_pose = left_pose
    if right_target_pose is None and action_kind == "target_pose":
        right_target_pose = right_pose
    arm_mask = np.asarray([
        1.0 if _has_nonzero_pose(left_pose[:length]) else 0.0,
        1.0 if _has_nonzero_pose(right_pose[:length]) else 0.0,
    ], dtype=np.float32)
    if not arm_mask.any():
        arm_mask[:] = 1.0
    return FlowEpisodeIndex(
        path=path,
        format_name="robotics_lab_dual_arm",
        length=length,
        timestamps=timestamps[:length],
        camera_paths=camera_paths,
        left_pose=left_pose[:length],
        right_pose=right_pose[:length],
        left_gripper=left_gripper[:length],
        right_gripper=right_gripper[:length],
        arm_mask=arm_mask,
        reset_left_pose=np.asarray(handle.attrs.get("reset_tcp_stand_left", left_pose[0]), dtype=np.float32),
        reset_right_pose=np.asarray(handle.attrs.get("reset_tcp_stand_right", right_pose[0]), dtype=np.float32),
        action_kind=action_kind,
        action_left_pose=None if left_target_pose is None else left_target_pose[:length],
        action_right_pose=None if right_target_pose is None else right_target_pose[:length],
        action_left_delta=None,
        action_right_delta=None,
        action_left_gripper=left_target_grip[:length] if left_target_grip is not None else left_gripper[:length],
        action_right_gripper=right_target_grip[:length] if right_target_grip is not None else right_gripper[:length],
    )


def _pika_bimanual_arm_groups(handle: h5py.File) -> dict[str, str]:
    if "observations" not in handle:
        return {}
    observations = handle["observations"]
    if not isinstance(observations, h5py.Group):
        return {}
    group_names = [
        str(name)
        for name, value in observations.items()
        if isinstance(value, h5py.Group) and "pose" in value
    ]
    if not group_names:
        return {}

    out: dict[str, str] = {}
    for name in group_names:
        canonical = name.strip().lower()
        if canonical in {"left", "right"}:
            out[canonical] = f"observations/{name}"

    if {"left", "right"}.issubset(out):
        return out

    arm_names = _decode_hdf5_attr(handle.attrs.get("arm_names", ""))
    ordered_names = [name.strip() for name in arm_names.split(",") if name.strip()]
    for name in ordered_names:
        canonical = name.lower()
        if canonical in {"left", "right"} and name in observations:
            group = observations[name]
            if isinstance(group, h5py.Group) and "pose" in group:
                out[canonical] = f"observations/{name}"

    remaining = [name for name in group_names if f"observations/{name}" not in out.values()]
    for side, name in zip(("left", "right"), remaining):
        out.setdefault(side, f"observations/{name}")
    return out


def _pika_bimanual_arm_arrays(
    handle: h5py.File,
    group_path: str | None,
    length: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray | None, np.ndarray]:
    identity_pose = _identity_poses(length)
    zero_gripper = np.zeros(length, dtype=np.float32)
    if group_path is None or group_path not in handle:
        return identity_pose, zero_gripper, None, zero_gripper

    group = handle[group_path]
    pose = np.asarray(group["pose"], dtype=np.float32)[:length, :7] if "pose" in group else identity_pose
    gripper_raw = np.asarray(group["gripper"], dtype=np.float32) if "gripper" in group else None
    gripper = _gripper_vector_or_zero(gripper_raw, length)
    action = np.asarray(group["action"], dtype=np.float32)[:length] if "action" in group else None
    action_pose = action[:, :7] if action is not None and action.ndim == 2 and action.shape[1] >= 7 else pose
    action_gripper = action[:, 7] if action is not None and action.ndim == 2 and action.shape[1] > 7 else gripper
    return pose, gripper.reshape(-1), action_pose, action_gripper.reshape(-1)


def _camera_paths(handle: h5py.File, group_path: str) -> dict[str, str]:
    if group_path not in handle:
        return {}
    group = handle[group_path]
    return {
        str(name): f"{group_path}/{name}"
        for name, value in group.items()
        if isinstance(value, h5py.Dataset)
    }


def _camera_lengths(handle: h5py.File, camera_paths: dict[str, str]) -> list[int]:
    return [int(handle[path].shape[0]) for path in camera_paths.values()]


def _load_images(
    episode_path: Path,
    camera_paths: dict[str, str],
    camera_names: list[str],
    index: int,
    image_size: int,
    image_crop: str,
) -> tuple[np.ndarray, int, int]:
    if not camera_names:
        return np.zeros((0, 3, image_size, image_size), dtype=np.float32), 0, 0
    frames: list[np.ndarray] = []
    decode_count = 0
    missing_count = 0
    with h5py.File(episode_path, "r") as handle:
        for camera_name in camera_names:
            path = camera_paths.get(camera_name)
            if path is None:
                frames.append(np.zeros((3, image_size, image_size), dtype=np.float32))
                missing_count += 1
                continue
            dataset = handle[path]
            if index >= dataset.shape[0]:
                frames.append(np.zeros((3, image_size, image_size), dtype=np.float32))
                missing_count += 1
                continue
            frames.append(
                decode_hdf5_image_value(
                    dataset[index],
                    image_size=image_size,
                    image_crop=image_crop,
                )
            )
            decode_count += 1
    return np.stack(frames, axis=0), decode_count, missing_count


def _proprio_vector(
    episode: FlowEpisodeIndex,
    index: int,
    *,
    action_frame: str = DEFAULT_ACTION_FRAME,
) -> np.ndarray:
    frame = normalize_action_frame(action_frame)
    delta_fn = pose_delta_local
    left_features = np.concatenate([
        delta_fn(episode.reset_left_pose, episode.left_pose[index]),
        [episode.left_gripper[index]],
    ])
    right_features = np.concatenate([
        delta_fn(episode.reset_right_pose, episode.right_pose[index]),
        [episode.right_gripper[index]],
    ])
    return np.concatenate([left_features, right_features, episode.arm_mask])


def _action_chunk(
    episode: FlowEpisodeIndex,
    start: int,
    horizon: int,
    *,
    action_frame: str = DEFAULT_ACTION_FRAME,
) -> np.ndarray:
    frame = normalize_action_frame(action_frame)
    delta_fn = pose_delta_local
    chunk = np.zeros((horizon, FLOW_ACTION_DIM), dtype=np.float32)
    for offset in range(horizon):
        index = start + offset
        next_index = index + 1
        if episode.arm_mask[0] > 0.0:
            left_current = _action_pose_at(episode.action_left_pose, episode.left_pose, index)
            left_next = _action_pose_at(episode.action_left_pose, episode.left_pose, next_index)
            left_delta = delta_fn(left_current, left_next)
            left_grip = _per_step_gripper_delta(
                episode.action_left_gripper,
                episode.left_gripper,
                index,
                next_index,
            )
            chunk[offset, 0:FLOW_ARM_DIM] = np.concatenate([left_delta, [left_grip]])
        if episode.arm_mask[1] > 0.0:
            right_current = _action_pose_at(episode.action_right_pose, episode.right_pose, index)
            right_next = _action_pose_at(episode.action_right_pose, episode.right_pose, next_index)
            right_delta = delta_fn(right_current, right_next)
            right_grip = _per_step_gripper_delta(
                episode.action_right_gripper,
                episode.right_gripper,
                index,
                next_index,
            )
            chunk[offset, FLOW_ARM_DIM : 2 * FLOW_ARM_DIM] = np.concatenate([right_delta, [right_grip]])
    return chunk


def _action_pose_at(values: np.ndarray | None, fallback: np.ndarray, index: int) -> np.ndarray:
    return values[index] if values is not None else fallback[index]


def _per_step_gripper_delta(
    values: np.ndarray | None,
    fallback: np.ndarray,
    index: int,
    next_index: int,
) -> float:
    if values is None:
        values = fallback
    return float(values[next_index]) - float(values[index])


def pose_delta_local(reference_pose: np.ndarray, target_pose: np.ndarray) -> np.ndarray:
    reference = _valid_pose(reference_pose)
    target = _valid_pose(target_pose)
    q_ref = _normalize_quat(reference[3:7])
    q_target = _normalize_quat(target[3:7])
    q_ref_inverse = _quat_inverse(q_ref)
    translation = _quat_rotate_vector(q_ref_inverse, target[:3] - reference[:3])
    q_delta = _quat_multiply(q_ref_inverse, q_target)
    rotvec = _quat_to_rotvec(q_delta)
    return np.concatenate([translation, rotvec]).astype(np.float32)


def pose_compose_local(reference_pose: np.ndarray, delta6: np.ndarray) -> np.ndarray:
    reference = _valid_pose(reference_pose)
    delta = np.asarray(delta6, dtype=np.float64).reshape(-1)
    if delta.shape[0] != 6:
        raise ValueError("local pose delta must contain 6 values")
    q_ref = _normalize_quat(reference[3:7])
    target = np.zeros(7, dtype=np.float64)
    target[:3] = reference[:3].astype(np.float64) + _quat_rotate_vector(q_ref, delta[:3])
    target[3:7] = _normalize_quat(_quat_multiply(q_ref, _rotvec_to_quat(delta[3:6])))
    return target.astype(np.float32)


def _dataset_dt_values_sec(episodes: list[FlowEpisodeIndex]) -> np.ndarray:
    values: list[np.ndarray] = []
    for episode in episodes:
        timestamps = np.asarray(episode.timestamps, dtype=np.float64).reshape(-1)
        if timestamps.shape[0] < 2:
            continue
        deltas = np.diff(timestamps)
        deltas = deltas[np.isfinite(deltas) & (deltas > 0.0)]
        if not deltas.size:
            continue
        # Some robotics logs carry raw nanoseconds despite a generic timestamp key.
        if float(np.median(deltas)) > 1000.0:
            deltas = deltas / 1e9
        values.append(deltas.astype(np.float64, copy=False))
    if not values:
        return np.zeros(0, dtype=np.float64)
    return np.concatenate(values).astype(np.float64, copy=False)


def _valid_pose(value: np.ndarray) -> np.ndarray:
    pose = np.asarray(value, dtype=np.float32).reshape(-1)
    if pose.shape[0] < 7:
        out = np.zeros(7, dtype=np.float32)
        out[6] = 1.0
        out[: pose.shape[0]] = pose
        return out
    return pose[:7]


def _normalize_quat(q: np.ndarray) -> np.ndarray:
    quat = np.asarray(q, dtype=np.float64)
    norm = float(np.linalg.norm(quat))
    if not math.isfinite(norm) or norm < 1e-12:
        return np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return quat / norm


def _quat_inverse(q: np.ndarray) -> np.ndarray:
    return np.asarray([-q[0], -q[1], -q[2], q[3]], dtype=np.float64)


def _quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.asarray(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ],
        dtype=np.float64,
    )


def _rotvec_to_quat(rotvec: np.ndarray) -> np.ndarray:
    vector = np.asarray(rotvec, dtype=np.float64).reshape(3)
    angle = float(np.linalg.norm(vector))
    if angle < 1e-8:
        return np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    axis = vector / angle
    half = 0.5 * angle
    return _normalize_quat(np.asarray([*(axis * math.sin(half)), math.cos(half)], dtype=np.float64))


def _quat_rotate_vector(q: np.ndarray, vector: np.ndarray) -> np.ndarray:
    quat = _normalize_quat(q)
    value = np.asarray(vector, dtype=np.float64).reshape(3)
    pure = np.asarray([value[0], value[1], value[2], 0.0], dtype=np.float64)
    rotated = _quat_multiply(_quat_multiply(quat, pure), _quat_inverse(quat))
    return rotated[:3]


def _quat_to_rotvec(q: np.ndarray) -> np.ndarray:
    quat = _normalize_quat(q)
    if quat[3] < 0.0:
        quat = -quat
    xyz = quat[:3]
    sin_half = float(np.linalg.norm(xyz))
    if sin_half < 1e-9:
        return (2.0 * xyz).astype(np.float32)
    angle = 2.0 * math.atan2(sin_half, float(quat[3]))
    return (xyz / sin_half * angle).astype(np.float32)


def _identity_poses(length: int) -> np.ndarray:
    poses = np.zeros((length, 7), dtype=np.float32)
    poses[:, 6] = 1.0
    return poses


def _dataset_or_identity(group: h5py.Group, *names: str, length: int | None = None) -> np.ndarray:
    for name in names:
        if name in group:
            arr = np.asarray(group[name], dtype=np.float32)
            if arr.ndim == 2 and arr.shape[1] >= 7:
                return arr[:, :7]
    if length is None:
        length = _infer_group_length(group)
    return _identity_poses(int(length))


def _infer_group_length(group: h5py.Group) -> int:
    for value in group.values():
        if isinstance(value, h5py.Dataset) and value.shape:
            return int(value.shape[0])
    return 0


def _robotics_timestamps(group: h5py.Group, length: int) -> np.ndarray:
    for name in ("timestamp", "timestamps", "state_host_time_ns", "bundle_host_time_ns"):
        if name not in group:
            continue
        values = np.asarray(group[name])
        if len(values) >= length:
            values = values[:length].astype(np.float64)
            if name.endswith("_ns"):
                values = values / 1e9
            return values
    return np.arange(length, dtype=np.float64)


def _optional_vector(group: h5py.Group, length: int, *names: str) -> np.ndarray:
    for name in names:
        if name in group:
            values = np.asarray(group[name], dtype=np.float32)
            return _gripper_vector_or_zero(values, length)
    return np.zeros(length, dtype=np.float32)


def _gripper_vector_or_zero(values: np.ndarray | None, length: int) -> np.ndarray:
    if values is None:
        return np.zeros(length, dtype=np.float32)
    array = np.asarray(values, dtype=np.float32)
    if array.size == 0:
        return np.zeros(length, dtype=np.float32)
    if array.ndim == 1:
        vector = array
    elif array.ndim == 2 and array.shape[1] > 0:
        vector = array[:, 0]
    else:
        return np.zeros(length, dtype=np.float32)
    out = np.zeros(length, dtype=np.float32)
    count = min(length, int(vector.shape[0]))
    if count > 0:
        out[:count] = vector[:count]
    return out


def _first_dataset(group: h5py.Group, length: int, *names: str) -> np.ndarray | None:
    for name in names:
        if name in group:
            values = np.asarray(group[name], dtype=np.float32)
            return values[:length]
    return None


def _ensure_delta(value: np.ndarray | None, length: int) -> np.ndarray:
    if value is None:
        return np.zeros((length, 6), dtype=np.float32)
    if value.ndim != 2:
        return np.zeros((length, 6), dtype=np.float32)
    out = np.zeros((length, 6), dtype=np.float32)
    columns = min(6, value.shape[1])
    out[:, :columns] = value[:length, :columns]
    return out


def _has_nonzero_pose(values: np.ndarray) -> bool:
    return bool(np.isfinite(values).all() and np.any(np.abs(values[:, :3]) > 1e-9))


def _pose_from_state_arm(value: Any) -> np.ndarray:
    arm = value if isinstance(value, dict) else {}
    pose = arm.get("tcp_stand") or arm.get("tcp_actual_stand") or {}
    if not isinstance(pose, dict):
        out = np.zeros(7, dtype=np.float32)
        out[6] = 1.0
        return out
    q = pose.get("quaternion_xyzw")
    if isinstance(q, (list, tuple)) and len(q) >= 4:
        qx, qy, qz, qw = q[:4]
    else:
        qx = pose.get("qx", 0.0)
        qy = pose.get("qy", 0.0)
        qz = pose.get("qz", 0.0)
        qw = pose.get("qw", 1.0)
    return np.asarray(
        [
            _float_or_zero(pose.get("x")),
            _float_or_zero(pose.get("y")),
            _float_or_zero(pose.get("z")),
            _float_or_zero(qx),
            _float_or_zero(qy),
            _float_or_zero(qz),
            _float_or_zero(qw, default=1.0),
        ],
        dtype=np.float32,
    )


def _decode_hdf5_attr(value: Any) -> str:
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return str(value or "")


def _nested_first(value: Any, *names: str) -> Any:
    if not isinstance(value, dict):
        return None
    for name in names:
        if name in value:
            return value[name]
    return None


def _float_or_zero(value: Any, *, default: float = 0.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def _normalize_array(value: np.ndarray, mean: Any, std: Any) -> np.ndarray:
    mean_array = np.asarray(mean, dtype=np.float32)
    std_array = np.asarray(std, dtype=np.float32)
    if mean_array.size == 0 or std_array.size == 0:
        return value.astype(np.float32, copy=False)
    return (value - mean_array) / np.maximum(std_array, 1e-6)
