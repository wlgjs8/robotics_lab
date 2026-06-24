"""Runtime action source for the point-cloud flow policy (pc_v1).

This subclasses :class:`FlowMatchingActionSource` so it reuses ALL of the deploy
machinery the image flow source already has — ee_local delta -> absolute
``TcpPoseTarget`` composition (``pose_compose_local``), per-step velocity clamps,
the absolute-percent gripper dispatch, chunk execution / crossfade, action
logging, and the rollout-mode/SafetyGate gating. Only the model-specific pieces
are overridden:

  * the checkpoint schema gate + model construction (point-cloud flow policy);
  * observation building: a live egocentric XYZRGB cloud back-projected from the
    wrist RealSense color+depth (instead of RGB image tensors), plus the
    velocity-proprio context (last K executed ee_local pose-deltas, W-invariant).

Everything downstream of ``_sample_chunk`` (the EE-frame action chunk) is shared
with the parent, so train/deploy stay consistent and TcpPoseTarget is the deploy
command family.
"""

from __future__ import annotations

from dataclasses import replace
from typing import Any

import numpy as np

from .action_sources.tcp_delta import cartesian_action_requirements
from .camera_bundle_client import resolve_frame
from .flow_dataset import FLOW_PROPRIO_DIM
from .flow_inference import FlowMatchingActionSource
from .pc_dataset import (
    ARM_SIDES,
    DEFAULT_DEPTH_MAX_M,
    DEFAULT_DEPTH_MIN_M,
    DEFAULT_NUM_POINTS,
    PC_CHECKPOINT_SCHEMA,
    POSE_DELTA_DIMS,
    _build_arm_cloud_static,
)
from .pc_infer import DEFAULT_INTRINSICS, RuntimeVelocityBuffer, sample_pc_chunk


def _infer_proprio_layout(total_dim: int) -> tuple[str, int]:
    """Recover (proprio_mode, velocity_steps) from the trained proprio width.

    full     = FLOW_PROPRIO_DIM (16 reset-rel base) + velocity_steps * 12
    velocity = velocity_steps * 12 (ego velocity only)
    none     = 1 (placeholder token)
    """
    d = len(POSE_DELTA_DIMS)  # 12
    if total_dim <= 1:
        return "none", 0
    if total_dim >= FLOW_PROPRIO_DIM and (total_dim - FLOW_PROPRIO_DIM) % d == 0:
        return "full", (total_dim - FLOW_PROPRIO_DIM) // d
    if total_dim % d == 0:
        return "velocity", total_dim // d
    raise ValueError(f"cannot infer pc proprio layout from proprio_dim={total_dim}")


class PointCloudFlowActionSource(FlowMatchingActionSource):
    """Flow source whose observation is an egocentric point cloud (pc_v1)."""

    _default_command_family = "tcp_target_pose"

    def __init__(
        self,
        checkpoint_path: str,
        *,
        sample_steps: int = 16,
        stochastic_sampling: bool = True,
        intrinsics_by_arm: dict[str, tuple[float, float, float, float, float]] | None = None,
        num_points: int | None = None,
        depth_min_m: float | None = None,
        depth_max_m: float | None = None,
        **kwargs: Any,
    ):
        # The parent __init__ loads the checkpoint, builds the model (via our
        # overridden _build_policy_model), and wires the deploy machinery using
        # the dataset_stats (arm mask, velocity limits, command family, gripper).
        super().__init__(
            checkpoint_path,
            sample_steps=sample_steps,
            stochastic_sampling=stochastic_sampling,
            **kwargs,
        )
        self.policy_label = "pc flow policy"
        self.gripper_command_source = "pc_flow_policy"
        # pc_v1 always trains an ABSOLUTE gripper opening in percent.
        self.gripper_action_absolute = True

        # --- point-cloud observation config (from dataset_stats, CLI overridable)
        self.num_points = int(num_points if num_points is not None else self.stats.get("num_points", DEFAULT_NUM_POINTS))
        self.depth_min_m = float(depth_min_m if depth_min_m is not None else self.stats.get("depth_min_m", DEFAULT_DEPTH_MIN_M))
        self.depth_max_m = float(depth_max_m if depth_max_m is not None else self.stats.get("depth_max_m", DEFAULT_DEPTH_MAX_M))
        self._color_stream = str(self.stats.get("color_stream", "realsense_color"))
        self._depth_stream = str(self.stats.get("depth_stream", "realsense_depth"))

        intrinsics_by_arm = intrinsics_by_arm or {}
        self._intrinsics_by_arm = {
            side: tuple(intrinsics_by_arm.get(side, DEFAULT_INTRINSICS)) for side in ARM_SIDES
        }

        # --- proprio layout (full = base16 + velocity; velocity-only; none)
        total_proprio = len(self.stats.get("proprio_mean", []) or [])
        if total_proprio <= 0:
            total_proprio = int(getattr(self.model.config, "proprio_dim", FLOW_PROPRIO_DIM))
        self._proprio_mode, self._velocity_steps = _infer_proprio_layout(total_proprio)
        self._proprio_mean = np.asarray(self.stats.get("proprio_mean", []), dtype=np.float32)
        self._proprio_std = np.asarray(self.stats.get("proprio_std", []), dtype=np.float32)
        self._vel = RuntimeVelocityBuffer(self._velocity_steps)
        # EE-frame chunk we produced last (pre r_align / rotation-mask), used to
        # feed the velocity buffer the deltas actually executed (W-invariant).
        self._chunk_ee: np.ndarray | None = None
        self._rng = np.random.default_rng()

        # The freshness/SafetyGate camera gate keys off self.camera_names. pc_v1
        # model_config carries no camera names (it is cloud-conditioned), so set
        # them from the configured streams for the arms the checkpoint uses, and
        # mark the source as camera-requiring. The parent's _camera_inputs_ready /
        # _count_missing_camera_frames then check both color+depth per arm.
        self.camera_names = []
        for idx, side in enumerate(ARM_SIDES):
            if float(self.arm_mask[idx]) > 0.0:
                self.camera_names.append(f"{side}_{self._color_stream}")
                self.camera_names.append(f"{side}_{self._depth_stream}")
        self.requirements = replace(self.requirements, requires_camera=True)

        print(
            f"[flow-infer] pc_v1: num_points={self.num_points} depth=[{self.depth_min_m:.3f},"
            f"{self.depth_max_m:.3f}]m proprio_mode={self._proprio_mode} "
            f"velocity_steps={self._velocity_steps} streams=({self._color_stream},{self._depth_stream}) "
            f"intrinsics={self._intrinsics_by_arm}",
            flush=True,
        )

    # --------------------------------------------------------- override hooks --
    def _validate_checkpoint_schema(self, schema: str) -> None:
        if schema != PC_CHECKPOINT_SCHEMA:
            raise ValueError(f"unsupported point-cloud checkpoint schema: {schema}")

    def _build_policy_model(self, model_config: dict[str, Any], checkpoint: dict[str, Any]) -> Any:
        from .pc_model import PointCloudFlowConfig, PointCloudFlowPolicy

        model = PointCloudFlowPolicy(PointCloudFlowConfig.from_mapping(dict(model_config))).to(self.device)
        model.load_state_dict(checkpoint["model_state"])
        model.eval()
        return model

    # --------------------------------------------------------- observation ----
    def _build_runtime_proprio(self, payload: dict[str, Any]) -> np.ndarray:
        """Raw (un-normalized) proprio matching the trained proprio_mode.

        base16 is the parent's reset-relative ee_local proprio (r_align applied);
        velocity is the last K executed ee_local pose-deltas (EE frame), which the
        parent's r_align does NOT touch because the buffer already stores deltas in
        the training EE frame (see _sample_chunk)."""
        if self._proprio_mode == "none":
            return np.zeros(1, dtype=np.float32)
        vel = self._vel.query()
        if self._proprio_mode == "velocity":
            return vel if vel.size else np.zeros(1, dtype=np.float32)
        base = np.asarray(super()._runtime_proprio(payload), dtype=np.float32)  # 16-dim
        return np.concatenate([base, vel]).astype(np.float32) if vel.size else base

    def _build_runtime_cloud(self, payload: dict[str, Any]) -> np.ndarray | None:
        bundle = self._poll_camera_bundle()
        bundle_frames = getattr(bundle, "frames", {}) if bundle is not None else {}
        color_by_arm: dict[str, np.ndarray] = {}
        depth_by_arm: dict[str, np.ndarray] = {}
        for idx, side in enumerate(ARM_SIDES):
            if float(self.arm_mask[idx]) <= 0.0:
                continue
            cframe = resolve_frame(bundle_frames, f"{side}_{self._color_stream}")
            dframe = resolve_frame(bundle_frames, f"{side}_{self._depth_stream}")
            cpix = getattr(cframe, "pixels", None)
            dpix = getattr(dframe, "pixels", None)
            if cpix is None or dpix is None:
                return None  # missing required frame -> emit no motion this step
            color_by_arm[side] = np.asarray(cpix)
            depth_by_arm[side] = np.asarray(dpix)

        clouds = []
        for idx, side in enumerate(ARM_SIDES):
            if float(self.arm_mask[idx]) <= 0.0 or side not in color_by_arm:
                clouds.append(np.zeros((self.num_points, 6), dtype=np.float32))
                continue
            clouds.append(
                _build_arm_cloud_static(
                    color_by_arm[side],
                    depth_by_arm[side],
                    self._intrinsics_by_arm[side],
                    self.num_points,
                    self.depth_min_m,
                    self.depth_max_m,
                    self._rng,
                )
            )
        return np.stack(clouds, axis=0)  # (V=2, N, 6)

    def _sample_chunk(self, payload: dict[str, Any]) -> np.ndarray | None:
        # Feed the velocity buffer the EE-frame deltas executed from the PREVIOUS
        # chunk (matches training velocity = previous K pose-deltas, W-invariant).
        if self._chunk_ee is not None:
            executed = min(int(self._chunk_index), int(self._chunk_ee.shape[0]))
            for i in range(executed):
                self._vel.push(self._chunk_ee[i])
            self._chunk_ee = None

        cloud = self._build_runtime_cloud(payload)
        if cloud is None:
            return None

        proprio_raw = self._build_runtime_proprio(payload)
        if self._proprio_mean.size == proprio_raw.size and self._proprio_std.size == proprio_raw.size:
            proprio_norm = (proprio_raw - self._proprio_mean) / np.maximum(self._proprio_std, 1e-6)
        else:
            proprio_norm = proprio_raw

        chunk = sample_pc_chunk(
            self.model,
            cloud.astype(np.float32),
            proprio_norm.astype(np.float32),
            self.stats,
            steps=self.sample_steps,
            device=self.device,
            stochastic=self.stochastic_sampling,
        )  # (H, 14) EE-frame, denormalized
        chunk = np.asarray(chunk, dtype=np.float32)
        self._chunk_ee = chunk.copy()
        return chunk
