"""Runtime inference for the point-cloud flow policy (pc_v1) — deploy with TcpPoseTarget.

The stock flow-infer path is RGB-image based and rejects pc_v1. This module supplies the
pieces a point-cloud rollout needs, all reusing the training-time math so train/deploy match:

  * build_runtime_cloud  — live aligned color+depth -> egocentric XYZRGB cloud (V,N,6)
  * RuntimeVelocityBuffer — track the last K executed ee_local pose-deltas (the velocity
    proprio context; W-invariant, NOT init-pose dependent)
  * load_pc_policy / sample_pc_chunk — load PointCloudFlowPolicy + sample a (H,14) chunk
  * runtime intrinsics are NOT in the camera stream -> supplied via config (per-arm
    fx,fy,ppx,ppy,depth_scale). Defaults = the D405 calib baked in the training data.

The action chunk is ee_local per-step deltas + ABSOLUTE gripper; the live source composes
the deltas into absolute TcpPoseTarget setpoints (pose_compose_local), exactly like the
image flow path's tcp_target_pose command family.
"""

from __future__ import annotations

from collections import deque
from pathlib import Path
from typing import Any

import numpy as np

from .pc_dataset import (
    ARM_SIDES,
    PC_CHECKPOINT_SCHEMA,
    POSE_DELTA_DIMS,
    _build_arm_cloud_static,
    _decode_color,
    _decode_depth,
    denormalize_actions,
    normalize_pc_sample,
)

# D405 color intrinsics + depth_scale baked in the training HDF5 (640x480, aligned depth).
# Override per-rig at deploy if the live camera differs.
DEFAULT_INTRINSICS = (393.3166, 392.1858, 319.0753, 229.5226, 1e-4)


def parse_intrinsics(spec: str | None) -> tuple[float, float, float, float, float]:
    if not spec:
        return DEFAULT_INTRINSICS
    parts = tuple(float(x) for x in spec.split(","))
    if len(parts) != 5:
        raise ValueError("intrinsics must be fx,fy,ppx,ppy,depth_scale")
    return parts


def build_runtime_cloud(
    color_by_arm: dict[str, Any],
    depth_by_arm: dict[str, Any],
    intrinsics: tuple,
    arm_mask: np.ndarray,
    num_points: int,
    depth_min_m: float,
    depth_max_m: float,
    rng: np.random.Generator,
) -> np.ndarray:
    """color/depth_by_arm: {'left':frame,'right':frame} (uint8 HxWx3 / uint16 HxW or raw).
    Returns (V=2, num_points, 6) XYZRGB in metres / [0,1], zeros for inactive/missing arms."""
    clouds = []
    for idx, side in enumerate(ARM_SIDES):
        c, d = color_by_arm.get(side), depth_by_arm.get(side)
        if float(arm_mask[idx]) <= 0.0 or c is None or d is None:
            clouds.append(np.zeros((num_points, 6), dtype=np.float32))
            continue
        clouds.append(_build_arm_cloud_static(c, d, intrinsics, num_points, depth_min_m, depth_max_m, rng))
    return np.stack(clouds, axis=0)


class RuntimeVelocityBuffer:
    """Ring buffer of the last K executed per-step ee_local pose-deltas (12-dim each:
    L/R translation+rotation, gripper excluded). Push the delta actually commanded each
    step; query() returns the velocity-proprio context (oldest->newest), zero-padded at
    cold start — matching pc_dataset._velocity_context."""

    def __init__(self, steps: int):
        self.steps = int(steps)
        self._buf: deque[np.ndarray] = deque(maxlen=max(1, self.steps))

    def reset(self) -> None:
        self._buf.clear()

    def push(self, action_step_14: np.ndarray) -> None:
        if self.steps <= 0:
            return
        self._buf.append(np.asarray(action_step_14, dtype=np.float32)[POSE_DELTA_DIMS].copy())

    def query(self) -> np.ndarray:
        if self.steps <= 0:
            return np.zeros(0, dtype=np.float32)
        d = len(POSE_DELTA_DIMS)
        feats = [np.zeros(d, dtype=np.float32)] * (self.steps - len(self._buf)) + list(self._buf)
        return np.concatenate(feats[-self.steps:]).astype(np.float32)


def load_pc_policy(checkpoint: dict, device: str):
    """Build PointCloudFlowPolicy from a pc_v1 checkpoint dict. Returns (model, stats, cfg_raw)."""
    from .pc_model import PointCloudFlowConfig, PointCloudFlowPolicy

    if str(checkpoint.get("schema", "")) != PC_CHECKPOINT_SCHEMA:
        raise ValueError(f"not a pc_v1 checkpoint: {checkpoint.get('schema')}")
    cfg_raw = dict(checkpoint["model_config"])
    model = PointCloudFlowPolicy(PointCloudFlowConfig.from_mapping(cfg_raw)).to(device)
    model.load_state_dict(checkpoint["model_state"])
    model.eval()
    return model, dict(checkpoint["dataset_stats"]), cfg_raw


def sample_pc_chunk(model, pointcloud_norm, proprio_norm, stats, *, steps, device, stochastic=True):
    """pointcloud_norm (V,N,6) + proprio_norm (D,) already normalized -> (H,14) DENORMALIZED chunk."""
    import torch
    from .pc_model import pc_sample_action_chunks

    pc = torch.as_tensor(pointcloud_norm, dtype=torch.float32, device=device).unsqueeze(0)
    pr = torch.as_tensor(proprio_norm, dtype=torch.float32, device=device).unsqueeze(0)
    noise = None
    if stochastic:
        noise = torch.randn(1, model.config.action_horizon, model.config.action_dim, device=device)
    with torch.no_grad():
        out = pc_sample_action_chunks(model, pc, pr, steps=steps, initial_noise=noise)
        out = denormalize_actions(out, stats)
    return out.squeeze(0).cpu().numpy()


def assemble_proprio(base16: np.ndarray, vel_buffer: RuntimeVelocityBuffer, proprio_mode: str) -> np.ndarray:
    """Match pc_dataset proprio assembly for the trained proprio_mode."""
    if proprio_mode == "none":
        return np.zeros(1, dtype=np.float32)
    vel = vel_buffer.query()
    if proprio_mode == "velocity":
        return vel if vel.size else np.zeros(1, dtype=np.float32)
    return np.concatenate([base16, vel]) if vel.size else base16  # full
