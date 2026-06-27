"""Colored point-cloud dataset for the flow-matching policy (Variant A).

This is an egocentric RGB-D variant of :mod:`flow_dataset`. Instead of feeding
RGB frames through an image encoder, it back-projects the wrist RealSense depth
into a per-arm XYZRGB point cloud expressed in the (moving) camera frame.

Why this is calibration-free / UMI-safe:
  * The wrist-camera point cloud lives in the end-effector body frame (it moves
    with the EE). The action is ``ee_local`` (also body-frame). Both are
    invariant to the unmeasured ``steamvr_world -> stand`` rotation, exactly like
    the existing ee_local policy (see wiki umi-tcp-delta-frame / iDP3).

Differences vs FlowHdf5Dataset:
  * observation = ``pointcloud`` (V_arms, N, 6) instead of ``images``.
  * the gripper action channel is ABSOLUTE (commanded percent), not a delta.
  * only the RealSense color+depth pair is used (fisheye is ignored).

The pose / proprio / episode-loading machinery is reused verbatim from
flow_dataset so the action frame and reset-relative conventions stay identical.
"""

from __future__ import annotations

import io
import time
from pathlib import Path
from typing import Any

import h5py
import numpy as np
from PIL import Image, ImageFilter

from .flow_dataset import (
    DEFAULT_ACTION_FRAME,
    FLOW_ACTION_DIM,
    FLOW_ACTION_DIM_NAMES,
    FLOW_ARM_DIM,
    FLOW_PROPRIO_DIM,
    FlowEpisodeIndex,
    _action_pose_at,
    _proprio_vector,
    action_mask_from_arm_mask,
    discover_hdf5_episodes,
    load_flow_episode_index,
    normalize_action_frame,
    pose_delta_local,
)

def _load_index_with_retry(path, attempts: int = 6) -> FlowEpisodeIndex:
    """Episode-index load with retry — guards __init__ against transient NFS EIO."""
    last: Exception | None = None
    for attempt in range(attempts):
        try:
            return load_flow_episode_index(path)
        except (OSError, KeyError, RuntimeError) as exc:
            last = exc
            time.sleep(0.1 * (attempt + 1))
    raise RuntimeError(f"failed to load episode index after {attempts} attempts: {path}") from last


PC_CHECKPOINT_SCHEMA = "robotics_lab.policy_runner.flow_matching.pc_v1"
DEFAULT_NUM_POINTS = 4096
DEFAULT_DEPTH_MIN_M = 0.05
DEFAULT_DEPTH_MAX_M = 0.60
ARM_SIDES = ("left", "right")


class PointCloudFlowDataset:
    """Egocentric XYZRGB point-cloud action-chunk dataset (Variant A)."""

    def __init__(
        self,
        episodes_dir: str | Path,
        *,
        action_horizon: int = 16,
        num_points: int = DEFAULT_NUM_POINTS,
        depth_min_m: float = DEFAULT_DEPTH_MIN_M,
        depth_max_m: float = DEFAULT_DEPTH_MAX_M,
        color_stream: str = "realsense_color",
        depth_stream: str = "realsense_depth",
        episode_paths: list[str] | tuple[str, ...] | None = None,
        include_patterns: list[str] | tuple[str, ...] | None = None,
        max_episodes: int | None = None,
        stats: dict[str, Any] | None = None,
        normalize: bool = True,
        action_frame: str = DEFAULT_ACTION_FRAME,
        rng_seed: int = 0,
    ):
        if action_horizon <= 0:
            raise ValueError("action_horizon must be positive")
        if num_points <= 0:
            raise ValueError("num_points must be positive")
        self.root = Path(episodes_dir)
        self.action_horizon = int(action_horizon)
        self.num_points = int(num_points)
        self.depth_min_m = float(depth_min_m)
        self.depth_max_m = float(depth_max_m)
        self.color_stream = str(color_stream)
        self.depth_stream = str(depth_stream)
        self.action_frame = normalize_action_frame(action_frame)
        self.normalize = bool(normalize)
        self.stats = stats
        self._rng = np.random.default_rng(rng_seed)
        self.read_retries = 0
        self.read_failures = 0
        # per-episode-path intrinsics cache: {path: {side: (fx,fy,ppx,ppy,scale)}}
        self._intrinsics_cache: dict[str, dict[str, tuple[float, float, float, float, float]]] = {}

        self.episodes: list[FlowEpisodeIndex] = [
            _load_index_with_retry(path)
            for path in discover_hdf5_episodes(
                self.root,
                episode_paths=episode_paths,
                include_patterns=include_patterns,
            )
        ]
        if max_episodes is not None:
            self.episodes = self.episodes[: int(max_episodes)]
        if not self.episodes:
            raise ValueError(f"no HDF5 episodes found under {self.root}")

        self.sample_refs: list[tuple[int, int]] = []
        for episode_index, episode in enumerate(self.episodes):
            sample_count = max(0, episode.length - self.action_horizon)
            self.sample_refs.extend((episode_index, start) for start in range(sample_count))
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
            sample = normalize_pc_sample(sample, self.stats)
        return sample

    # ------------------------------------------------------------------ intrinsics
    def _episode_intrinsics(
        self, episode: FlowEpisodeIndex, handle: h5py.File
    ) -> dict[str, tuple[float, float, float, float, float]]:
        key = str(episode.path)
        cached = self._intrinsics_cache.get(key)
        if cached is not None:
            return cached
        out = _read_intrinsics(handle)
        self._intrinsics_cache[key] = out
        return out

    # ------------------------------------------------------------------ point cloud
    def _build_arm_cloud(
        self,
        handle: h5py.File,
        episode: FlowEpisodeIndex,
        side: str,
        index: int,
        intrinsics: tuple[float, float, float, float, float],
    ) -> np.ndarray:
        color_path = episode.camera_paths.get(f"{side}_{self.color_stream}")
        depth_path = episode.camera_paths.get(f"{side}_{self.depth_stream}")
        if color_path is None or depth_path is None:
            return np.zeros((self.num_points, 6), dtype=np.float32)
        return _build_arm_cloud_static(
            handle[color_path][index],
            handle[depth_path][index],
            intrinsics,
            self.num_points,
            self.depth_min_m,
            self.depth_max_m,
            self._rng,
        )

    def _read_clouds(self, episode: FlowEpisodeIndex, start: int) -> list[np.ndarray]:
        clouds: list[np.ndarray] = []
        with h5py.File(episode.path, "r") as handle:
            intrinsics = self._episode_intrinsics(episode, handle)
            for arm_idx, side in enumerate(ARM_SIDES):
                if float(episode.arm_mask[arm_idx]) <= 0.0 or side not in intrinsics:
                    cloud = np.zeros((self.num_points, 6), dtype=np.float32)
                else:
                    cloud = self._build_arm_cloud(handle, episode, side, start, intrinsics[side])
                clouds.append(cloud)
        return clouds

    def raw_sample(self, index: int) -> dict[str, np.ndarray]:
        episode_index, start = self.sample_refs[index]
        episode = self.episodes[episode_index]
        # NFS reads can hit transient EIO under many parallel workers; retry, then
        # fall back to zero clouds so a multi-hour training run never crashes.
        clouds: list[np.ndarray] | None = None
        for attempt in range(6):
            try:
                clouds = self._read_clouds(episode, start)
                break
            except (OSError, KeyError, RuntimeError):
                # h5py surfaces transient NFS EIO as OSError/KeyError/RuntimeError
                self._intrinsics_cache.pop(str(episode.path), None)
                self.read_retries += 1
                time.sleep(0.05 * (attempt + 1))
        if clouds is None:
            self.read_failures += 1
            clouds = [np.zeros((self.num_points, 6), dtype=np.float32) for _ in ARM_SIDES]
        valid_counts = [int((c[:, 2] > 0.0).sum()) for c in clouds]
        pointcloud = np.stack(clouds, axis=0)  # (V,N,6)
        proprio = _proprio_vector(episode, start, action_frame=self.action_frame)
        action_chunk = _pc_action_chunk(
            episode, start, self.action_horizon, action_frame=self.action_frame
        )
        action_mask = action_mask_from_arm_mask(episode.arm_mask, self.action_horizon)
        return {
            "pointcloud": pointcloud.astype(np.float32, copy=False),
            "proprio": proprio.astype(np.float32, copy=False),
            "action_chunk": action_chunk.astype(np.float32, copy=False),
            "action_mask": action_mask.astype(np.float32, copy=False),
            "arm_mask": episode.arm_mask.astype(np.float32, copy=False),
            "pc_valid_count": np.asarray(valid_counts, dtype=np.int64),
        }


def cache_key(relpath: str) -> str:
    return str(relpath).replace("/", "__").replace(".hdf5", "").replace(".h5", "")


POSE_DELTA_DIMS = [0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12]  # translation+rotation, exclude gripper(6,13)


def _smooth_pose_deltas(step_action: np.ndarray, window: int) -> np.ndarray:
    """Centered moving-average on the per-step pose deltas (not gripper).

    Raw 30Hz teleop deltas carry ~1.3mm high-frequency jitter that is
    unpredictable from observations and causes rollout tremble. Low-passing the
    target makes per-step motion predictable (const-vel floor 0.48mm @w=5) and
    yields smoother rollouts. Gripper channels are left as raw absolute values.
    """
    if window <= 1:
        return step_action
    out = step_action.copy()
    k = np.ones(window, dtype=np.float32) / float(window)
    for d in POSE_DELTA_DIMS:
        out[:, d] = np.convolve(step_action[:, d], k, mode="same")
    return out


class _CachedEpisode:
    __slots__ = ("cloud", "rgb", "step_action", "step_action_raw", "proprio", "arm_mask", "length")

    def __init__(self, base: Path, with_rgb: bool = False, smooth_window: int = 1):
        self.cloud = np.load(str(base) + ".cloud.npy", mmap_mode="r")  # (T,V,N,6) f16
        meta = np.load(str(base) + ".meta.npz")
        raw = meta["step_action"].astype(np.float32)
        self.step_action_raw = raw  # unsmoothed (for fair vs-raw eval across smooth levels)
        self.step_action = _smooth_pose_deltas(raw, smooth_window)
        self.proprio = meta["proprio"].astype(np.float32)
        self.arm_mask = meta["arm_mask"].astype(np.float32)
        self.length = int(self.cloud.shape[0])
        self.rgb = np.load(str(base) + ".rgb.npy", mmap_mode="r") if with_rgb else None  # (T,V,S,S,3) u8


class CachedPointCloudDataset:
    """Reads the local point-cloud cache written by pc_preprocess (fast path)."""

    def __init__(
        self,
        cache_dir: str | Path,
        *,
        action_horizon: int = 16,
        num_points: int = DEFAULT_NUM_POINTS,
        episode_relpaths: list[str] | tuple[str, ...] | None = None,
        stats: dict[str, Any] | None = None,
        normalize: bool = True,
        action_frame: str = DEFAULT_ACTION_FRAME,
        with_rgb: bool = False,
        augment: bool = False,
        smooth_window: int = 1,
        velocity_steps: int = 0,
        proprio_mode: str = "full",   # full | velocity | none (ego-centric ablation)
        proprio_noise_std: float = 0.0,
        dart_sigma_m: float = 0.0,
        dart_recover_steps: int = 5,
        dart_prob: float = 0.5,
        image_aug: bool = False,
        rng_seed: int = 0,
    ):
        self.cache_dir = Path(cache_dir)
        self.action_horizon = int(action_horizon)
        self.num_points = int(num_points)
        self.action_frame = normalize_action_frame(action_frame)
        self.normalize = bool(normalize)
        self.with_rgb = bool(with_rgb)
        self.augment = bool(augment)
        self.smooth_window = int(smooth_window)
        # append the previous N steps' (smoothed) pose-deltas to proprio as velocity
        # context — lets the policy exploit motion continuity (const-vel floor 0.48mm
        # @ smooth w=5). Available at deploy from the last executed deltas.
        self.velocity_steps = int(velocity_steps)
        self.proprio_mode = str(proprio_mode)
        # DART-style state noise: perturb normalized proprio so the policy learns to
        # recover from off-distribution states (covariate shift = the closed-loop gap).
        self.proprio_noise_std = float(proprio_noise_std)
        # recovery-DART (proper): displace observed proprio pose by eps~N(0,sigma) in
        # body frame AND relabel the first K action steps to recover (-eps/K each), so the
        # policy learns to drive back to the demo from a drifted state. translation-only.
        self.dart_sigma_m = float(dart_sigma_m)
        self.dart_recover_steps = int(dart_recover_steps)
        self.dart_prob = float(dart_prob)
        self.image_aug = bool(image_aug)
        self.stats = stats
        self._rng = np.random.default_rng(rng_seed)
        # kept for compatibility with compute_pc_dataset_statistics
        self.depth_min_m = DEFAULT_DEPTH_MIN_M
        self.depth_max_m = DEFAULT_DEPTH_MAX_M
        self.color_stream = "realsense_color"
        self.depth_stream = "realsense_depth"

        if episode_relpaths is not None:
            bases = [self.cache_dir / cache_key(r) for r in episode_relpaths]
        else:
            bases = sorted({Path(str(p)[: -len(".cloud.npy")]) for p in self.cache_dir.glob("*.cloud.npy")})
        self.episodes: list[_CachedEpisode] = []
        for base in bases:
            ok = Path(str(base) + ".cloud.npy").exists() and Path(str(base) + ".meta.npz").exists()
            if self.with_rgb:
                ok = ok and Path(str(base) + ".rgb.npy").exists()
            if ok:
                self.episodes.append(_CachedEpisode(base, with_rgb=self.with_rgb, smooth_window=self.smooth_window))
        if not self.episodes:
            raise ValueError(f"no cached episodes found under {self.cache_dir}")
        self.cache_num_points = int(self.episodes[0].cloud.shape[2])

        self.sample_refs: list[tuple[int, int]] = []
        for ep_idx, ep in enumerate(self.episodes):
            for start in range(max(0, ep.length - self.action_horizon)):
                self.sample_refs.append((ep_idx, start))
        if not self.sample_refs:
            raise ValueError("no samples; action_horizon too large for cached episodes")

    @property
    def proprio_dim(self) -> int:
        v = self.velocity_steps * len(POSE_DELTA_DIMS)
        if self.proprio_mode == "none":
            return 1                       # placeholder; model drops the proprio token
        if self.proprio_mode == "velocity":
            return max(1, v)
        return FLOW_PROPRIO_DIM + v        # full

    def __len__(self) -> int:
        return len(self.sample_refs)

    def _velocity_context(self, ep: "_CachedEpisode", start: int) -> np.ndarray:
        # previous `velocity_steps` pose-deltas (12 dims each), oldest->newest;
        # zero-padded before episode start (matches deploy cold-start).
        feats = []
        for k in range(self.velocity_steps, 0, -1):
            idx = start - k
            if idx >= 0:
                feats.append(ep.step_action[idx][POSE_DELTA_DIMS])
            else:
                feats.append(np.zeros(len(POSE_DELTA_DIMS), dtype=np.float32))
        return np.concatenate(feats).astype(np.float32) if feats else np.zeros(0, dtype=np.float32)

    def _augment_cloud(self, pc: np.ndarray) -> np.ndarray:
        # pc: (V, N, 6) in raw units (xyz metres, rgb [0,1]). Per-arm geometric +
        # photometric jitter to fight overfitting (343-episode set). Only touch
        # arms that actually have points (z>0 somewhere).
        pc = pc.copy()
        for v in range(pc.shape[0]):
            if not (pc[v, :, 2] > 0.0).any():
                continue
            scale = float(self._rng.uniform(0.97, 1.03))
            shift = self._rng.uniform(-0.01, 0.01, size=3).astype(np.float32)
            jitter = (self._rng.normal(0.0, 0.004, size=(pc.shape[1], 3))).astype(np.float32)
            pc[v, :, :3] = pc[v, :, :3] * scale + shift + jitter
            cjit = float(self._rng.uniform(-0.04, 0.04))
            pc[v, :, 3:6] = np.clip(pc[v, :, 3:6] + cjit, 0.0, 1.0)
        return pc

    def _subsample(self, frame: np.ndarray) -> np.ndarray:
        # frame: (V, N_cache, 6); resample axis=1 to num_points
        n_cache = frame.shape[1]
        if self.num_points == n_cache:
            return frame.astype(np.float32)
        sel = self._rng.choice(n_cache, self.num_points, replace=self.num_points > n_cache)
        return frame[:, sel, :].astype(np.float32)

    def raw_sample(self, index: int) -> dict[str, np.ndarray]:
        ep_idx, start = self.sample_refs[index]
        ep = self.episodes[ep_idx]
        frame = np.asarray(ep.cloud[start])  # (V,N_cache,6) f16 -> read one frame
        pointcloud = self._subsample(frame)
        if self.augment:
            pointcloud = self._augment_cloud(pointcloud)
        end = start + self.action_horizon
        action_chunk = ep.step_action[start:end].copy()
        action_mask = action_mask_from_arm_mask(ep.arm_mask, self.action_horizon)
        base = ep.proprio[start].copy()  # reset-relative pose(12)+grip(2)+arm_mask(2) = 16
        action_chunk = action_chunk.copy()
        # recovery-DART (full proprio only; needs the base pose state to drift)
        if self.proprio_mode == "full" and self.augment and self.dart_sigma_m > 0.0 and self._rng.random() < self.dart_prob:
            K = max(1, min(self.dart_recover_steps, self.action_horizon))
            for arm_idx, tdims in ((0, [0, 1, 2]), (1, [7, 8, 9])):
                if float(ep.arm_mask[arm_idx]) <= 0.0:
                    continue
                eps = self._rng.normal(0.0, self.dart_sigma_m, size=3).astype(np.float32)
                base[tdims] = base[tdims] + eps
                action_chunk[:K, tdims] = action_chunk[:K, tdims] - (eps / K)
        vel = self._velocity_context(ep, start) if self.velocity_steps > 0 else np.zeros(0, dtype=np.float32)
        # proprio_mode: full = reset-relative pose state (+velocity); velocity = ego velocity ONLY
        # (drops the non-ego reset-relative pose, the init-rotation-confounded signal); none = no proprio.
        if self.proprio_mode == "none":
            proprio = np.zeros(1, dtype=np.float32)
        elif self.proprio_mode == "velocity":
            proprio = vel if vel.size else np.zeros(1, dtype=np.float32)
        else:  # full
            proprio = np.concatenate([base, vel]) if vel.size else base
        sample = {
            "pointcloud": pointcloud,
            "proprio": proprio.copy(),
            "action_chunk": action_chunk.astype(np.float32, copy=False),
            "action_mask": action_mask.astype(np.float32, copy=False),
            "arm_mask": ep.arm_mask.copy(),
            "raw_action_chunk": ep.step_action_raw[start:end].astype(np.float32, copy=True),  # unsmoothed (eval only, NOT normalized)
            "pc_valid_count": np.asarray([int((pointcloud[v, :, 2] != 0).sum()) for v in range(pointcloud.shape[0])], dtype=np.int64),
        }
        if self.with_rgb and ep.rgb is not None:
            img = np.asarray(ep.rgb[start]).astype(np.float32) / 255.0  # (V,S,S,3)
            if self.augment and self.image_aug:
                img = self._augment_image(img)
            sample["image"] = np.transpose(img, (0, 3, 1, 2)).copy()  # (V,3,S,S)
        return sample

    def _augment_image(self, img: np.ndarray) -> np.ndarray:
        # img (V,S,S,3) in [0,1]. Domain randomization for the human->robot appearance
        # gap on the 13: photometric (brightness/contrast/tint/noise) + GAUSSIAN BLUR
        # (robot-moved-camera motion blur / focus) + CUTOUT/RANDOM-ERASING (occlusion,
        # missing-region robustness).
        out = img.copy()
        S = out.shape[1]
        for v in range(out.shape[0]):
            x = out[v]
            # --- photometric ---
            b = float(self._rng.uniform(0.8, 1.2))            # brightness
            c = float(self._rng.uniform(0.8, 1.2))            # contrast
            shift = self._rng.uniform(-0.05, 0.05, size=3).astype(np.float32)  # per-channel tint
            x = x * b
            x = (x - x.mean()) * c + x.mean() + shift
            x = np.clip(x + self._rng.normal(0.0, 0.015, size=x.shape).astype(np.float32), 0.0, 1.0)
            # --- gaussian blur (p=0.3) ---
            if self._rng.random() < 0.3:
                radius = float(self._rng.uniform(0.4, 2.0))
                pim = Image.fromarray((x * 255).astype(np.uint8)).filter(ImageFilter.GaussianBlur(radius))
                x = np.asarray(pim, dtype=np.float32) / 255.0
            # --- cutout / random erasing (p=0.5, 1-2 boxes) ---
            if self._rng.random() < 0.5:
                for _ in range(int(self._rng.integers(1, 3))):
                    area = float(self._rng.uniform(0.02, 0.12)) * S * S
                    ar = float(self._rng.uniform(0.5, 2.0))
                    h = int(min(S, max(1, (area * ar) ** 0.5))); w = int(min(S, max(1, (area / ar) ** 0.5)))
                    top = int(self._rng.integers(0, max(1, S - h))); left = int(self._rng.integers(0, max(1, S - w)))
                    fill = self._rng.uniform(0.0, 1.0, size=3).astype(np.float32) if self._rng.random() < 0.5 else 0.0
                    x[top:top + h, left:left + w, :] = fill
            out[v] = x
        return out

    def __getitem__(self, index: int) -> dict[str, np.ndarray]:
        sample = self.raw_sample(index)
        if self.normalize and self.stats:
            sample = normalize_pc_sample(sample, self.stats)
        if self.proprio_mode == "full" and self.augment and self.proprio_noise_std > 0.0:  # input-noise (full proprio only)
            # perturb only the base state (pose+gripper, first FLOW_PROPRIO_DIM dims);
            # leave the appended velocity context clean (it's the commanded recent motion,
            # known exactly at deploy — noising it destroyed the key signal in Batch 1).
            p = sample["proprio"].copy()
            noise = self._rng.normal(0.0, self.proprio_noise_std, size=FLOW_PROPRIO_DIM).astype(np.float32)
            p[:FLOW_PROPRIO_DIM] = p[:FLOW_PROPRIO_DIM] + noise
            sample["proprio"] = p
        return sample


# ------------------------------------------------------------ intrinsics / cloud
def _read_intrinsics(handle: h5py.File) -> dict[str, tuple[float, float, float, float, float]]:
    """Per-arm (fx, fy, ppx, ppy, depth_scale). Aligned depth -> use color K."""
    out: dict[str, tuple[float, float, float, float, float]] = {}
    for side in ARM_SIDES:
        calib_path = f"observations/{side}/camera_calib"
        if calib_path not in handle:
            continue
        calib = handle[calib_path]
        color_path = f"{calib_path}/color_intrinsics"
        if color_path not in handle:
            continue
        intr = handle[color_path]
        fx = float(intr.attrs.get("fx", 0.0))
        fy = float(intr.attrs.get("fy", 0.0))
        ppx = float(intr.attrs.get("ppx", 0.0))
        ppy = float(intr.attrs.get("ppy", 0.0))
        scale = float(calib.attrs.get("depth_scale", 1e-3))
        if fx > 0.0 and fy > 0.0:
            out[side] = (fx, fy, ppx, ppy, scale)
    return out


def _build_arm_cloud_static(
    color_value: Any,
    depth_value: Any,
    intrinsics: tuple[float, float, float, float, float],
    num_points: int,
    depth_min_m: float,
    depth_max_m: float,
    rng: np.random.Generator,
) -> np.ndarray:
    """Back-project one aligned RGB-D frame into a fixed-size XYZRGB cloud (N,6)."""
    zeros = np.zeros((num_points, 6), dtype=np.float32)
    color = _decode_color(color_value)
    depth = _decode_depth(depth_value)
    if color is None or depth is None:
        return zeros
    fx, fy, ppx, ppy, scale = intrinsics
    z = depth.astype(np.float32) * float(scale)
    valid = (depth > 0) & (z >= depth_min_m) & (z <= depth_max_m)
    if color.shape[:2] != depth.shape:
        color = np.asarray(
            Image.fromarray(color).resize((depth.shape[1], depth.shape[0]), Image.NEAREST)
        )
    if not valid.any():
        return zeros
    vs, us = np.nonzero(valid)
    zv = z[vs, us]
    xs = (us.astype(np.float32) - ppx) / fx * zv
    ys = (vs.astype(np.float32) - ppy) / fy * zv
    rgb = color[vs, us, :3].astype(np.float32) / 255.0
    cloud = np.concatenate([np.stack([xs, ys, zv], axis=1), rgb], axis=1)  # (M,6)
    m = cloud.shape[0]
    sel = rng.choice(m, num_points, replace=m < num_points)
    return cloud[sel].astype(np.float32)


# ---------------------------------------------------------------------- decoders
def _decode_color(value: Any) -> np.ndarray | None:
    image = _open_image(value)
    if image is None:
        return None
    return np.asarray(image.convert("RGB"), dtype=np.uint8)


def _decode_depth(value: Any) -> np.ndarray | None:
    image = _open_image(value)
    if image is None:
        return None
    arr = np.asarray(image)
    if arr.ndim != 2:
        arr = arr[..., 0]
    return arr.astype(np.uint16)


def _open_image(value: Any) -> Image.Image | None:
    try:
        if isinstance(value, np.ndarray) and value.ndim >= 2 and value.dtype != np.uint8:
            return Image.fromarray(value)
        if isinstance(value, np.ndarray) and value.dtype == np.uint8 and value.ndim == 1:
            return Image.open(io.BytesIO(value.tobytes()))
        if isinstance(value, np.ndarray) and value.ndim >= 2:
            return Image.fromarray(value)
        return Image.open(io.BytesIO(bytes(value)))
    except Exception:
        return None


# ----------------------------------------------------------------- action chunk
def _pc_action_chunk(
    episode: FlowEpisodeIndex,
    start: int,
    horizon: int,
    *,
    action_frame: str = DEFAULT_ACTION_FRAME,
) -> np.ndarray:
    """Same ee_local pose deltas as flow_dataset, but ABSOLUTE gripper target."""
    normalize_action_frame(action_frame)
    chunk = np.zeros((horizon, FLOW_ACTION_DIM), dtype=np.float32)
    length = episode.length
    for offset in range(horizon):
        index = start + offset
        next_index = min(index + 1, length - 1)
        if episode.arm_mask[0] > 0.0:
            cur = _action_pose_at(episode.action_left_pose, episode.left_pose, index)
            nxt = _action_pose_at(episode.action_left_pose, episode.left_pose, next_index)
            grip = _absolute_gripper(episode.action_left_gripper, episode.left_gripper, next_index)
            chunk[offset, 0:FLOW_ARM_DIM] = np.concatenate([pose_delta_local(cur, nxt), [grip]])
        if episode.arm_mask[1] > 0.0:
            cur = _action_pose_at(episode.action_right_pose, episode.right_pose, index)
            nxt = _action_pose_at(episode.action_right_pose, episode.right_pose, next_index)
            grip = _absolute_gripper(episode.action_right_gripper, episode.right_gripper, next_index)
            chunk[offset, FLOW_ARM_DIM : 2 * FLOW_ARM_DIM] = np.concatenate(
                [pose_delta_local(cur, nxt), [grip]]
            )
    return chunk


def _absolute_gripper(values: np.ndarray | None, fallback: np.ndarray, index: int) -> float:
    source = values if values is not None else fallback
    index = min(int(index), int(source.shape[0]) - 1)
    return float(source[index])


# ------------------------------------------------------------------- statistics
def compute_pc_dataset_statistics(
    dataset: PointCloudFlowDataset,
    *,
    max_samples: int | None = None,
) -> dict[str, Any]:
    sample_count = len(dataset) if max_samples is None else min(len(dataset), max_samples)
    if sample_count <= 0:
        raise ValueError("cannot compute statistics for an empty dataset")
    # uniform stride so a capped stats pass still spans the whole dataset
    stride = max(1, len(dataset) // sample_count)
    indices = list(range(0, len(dataset), stride))[:sample_count]

    proprio_values: list[np.ndarray] = []
    action_sum = np.zeros(FLOW_ACTION_DIM, dtype=np.float64)
    action_sq = np.zeros(FLOW_ACTION_DIM, dtype=np.float64)
    action_cnt = np.zeros(FLOW_ACTION_DIM, dtype=np.float64)
    action_by_dim: list[list[np.ndarray]] = [[] for _ in range(FLOW_ACTION_DIM)]
    xyz_sum = np.zeros(3, dtype=np.float64)
    xyz_sq = np.zeros(3, dtype=np.float64)
    xyz_cnt = 0.0
    arm_counts = np.zeros(2, dtype=np.int64)

    for index in indices:
        sample = dataset.raw_sample(index)
        proprio_values.append(sample["proprio"].astype(np.float64))
        actions = sample["action_chunk"].astype(np.float64)
        mask = sample["action_mask"].astype(np.float64)
        action_sum += (actions * mask).sum(axis=0)
        action_sq += ((actions * actions) * mask).sum(axis=0)
        action_cnt += mask.sum(axis=0)
        for dim in range(FLOW_ACTION_DIM):
            vals = actions[:, dim][mask[:, dim] > 0.0]
            if vals.size:
                action_by_dim[dim].append(vals)
        # only count xyz of non-empty points (z>0) for normalization
        pc = sample["pointcloud"].astype(np.float64).reshape(-1, 6)
        nz = pc[pc[:, 2] > 0.0]
        if nz.size:
            xyz_sum += nz[:, :3].sum(axis=0)
            xyz_sq += (nz[:, :3] ** 2).sum(axis=0)
            xyz_cnt += nz.shape[0]
        arm_counts += sample["arm_mask"].astype(np.int64)

    proprio_stack = np.stack(proprio_values, axis=0)
    proprio_mean = proprio_stack.mean(axis=0)
    proprio_std = np.maximum(proprio_stack.std(axis=0), 1e-6)

    action_mean = np.divide(action_sum, action_cnt, out=np.zeros_like(action_sum), where=action_cnt > 0)
    action_var = (
        np.divide(action_sq, action_cnt, out=np.zeros_like(action_sq), where=action_cnt > 0)
        - action_mean * action_mean
    )
    action_std = np.sqrt(np.maximum(action_var, 1e-12))
    action_std[action_cnt == 0] = 1.0

    xyz_mean = (xyz_sum / xyz_cnt) if xyz_cnt > 0 else np.zeros(3)
    xyz_var = (xyz_sq / xyz_cnt - xyz_mean**2) if xyz_cnt > 0 else np.ones(3)
    xyz_std = np.sqrt(np.maximum(xyz_var, 1e-6))

    action_percentiles: dict[str, dict[str, float]] = {}
    for dim, name in enumerate(FLOW_ACTION_DIM_NAMES):
        if action_by_dim[dim]:
            values = np.concatenate(action_by_dim[dim]).astype(np.float64)
            pct = np.percentile(values, [1, 5, 50, 95, 99])
        else:
            pct = np.zeros(5)
        action_percentiles[name] = {
            "p01": float(pct[0]), "p05": float(pct[1]), "p50": float(pct[2]),
            "p95": float(pct[3]), "p99": float(pct[4]),
        }

    return {
        "schema": PC_CHECKPOINT_SCHEMA + ".dataset_stats",
        "episode_count": len(dataset.episodes),
        "sample_count": sample_count,
        "total_sample_count": len(dataset),
        "action_horizon": dataset.action_horizon,
        "num_points": dataset.num_points,
        "depth_min_m": dataset.depth_min_m,
        "depth_max_m": dataset.depth_max_m,
        "color_stream": dataset.color_stream,
        "depth_stream": dataset.depth_stream,
        "proprio_action_frame": dataset.action_frame,
        "proprio_dim": FLOW_PROPRIO_DIM,
        "action_dim": FLOW_ACTION_DIM,
        "gripper_action_mode": "absolute",
        "proprio_mean": proprio_mean.astype(float).tolist(),
        "proprio_std": proprio_std.astype(float).tolist(),
        "action_mean": action_mean.astype(float).tolist(),
        "action_std": action_std.astype(float).tolist(),
        "pc_xyz_mean": xyz_mean.astype(float).tolist(),
        "pc_xyz_std": xyz_std.astype(float).tolist(),
        "arm_mask_counts": {"left": int(arm_counts[0]), "right": int(arm_counts[1])},
        "action_distribution_percentiles": action_percentiles,
    }


def normalize_pc_sample(sample: dict[str, np.ndarray], stats: dict[str, Any]) -> dict[str, np.ndarray]:
    out = dict(sample)
    out["proprio"] = _standardize(sample["proprio"], stats.get("proprio_mean"), stats.get("proprio_std"))
    out["action_chunk"] = _standardize(
        sample["action_chunk"], stats.get("action_mean"), stats.get("action_std")
    )
    pc = sample["pointcloud"].copy()
    xyz_mean = np.asarray(stats.get("pc_xyz_mean", [0, 0, 0]), dtype=np.float32)
    xyz_std = np.maximum(np.asarray(stats.get("pc_xyz_std", [1, 1, 1]), dtype=np.float32), 1e-6)
    pc[..., :3] = (pc[..., :3] - xyz_mean) / xyz_std
    pc[..., 3:6] = (pc[..., 3:6] - 0.5) / 0.5  # rgb [0,1] -> [-1,1]
    out["pointcloud"] = pc
    if "image" in sample:  # ImageNet normalization for the (DINOv3) image branch
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32).reshape(1, 3, 1, 1)
        std = np.array([0.229, 0.224, 0.225], dtype=np.float32).reshape(1, 3, 1, 1)
        out["image"] = (sample["image"] - mean) / std
    return out


def _standardize(arr: np.ndarray, mean, std) -> np.ndarray:
    if mean is None or std is None:
        return arr
    mean = np.asarray(mean, dtype=np.float32)
    std = np.maximum(np.asarray(std, dtype=np.float32), 1e-6)
    return ((arr - mean) / std).astype(np.float32)


def denormalize_actions(actions, stats: dict[str, Any]):
    import torch

    mean = torch.as_tensor(stats["action_mean"], dtype=actions.dtype, device=actions.device)
    std = torch.as_tensor(stats["action_std"], dtype=actions.dtype, device=actions.device)
    return actions * std.view(1, 1, -1) + mean.view(1, 1, -1)


def load_split_episode_paths(split_json: str | Path, key: str) -> list[str]:
    """Read a train/val episode list (relative HDF5 paths) from a split JSON."""
    import json

    data = json.loads(Path(split_json).read_text())
    entries = data[key]
    return [str(entry) for entry in entries]
