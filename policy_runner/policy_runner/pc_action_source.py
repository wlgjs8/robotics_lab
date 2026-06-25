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
from .pc_dataset import denormalize_actions
from .pc_infer import DEFAULT_INTRINSICS, RuntimeVelocityBuffer, sample_pc_chunk

# ImageNet normalization (must match pc_dataset.normalize_pc_sample image branch).
_IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32).reshape(3, 1, 1)
_IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32).reshape(3, 1, 1)
_DEFAULT_IMAGE_SIZE = 224


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
        # Cloud normalization stats (MUST match pc_dataset.normalize_pc_sample, or
        # the PointNet encoder sees an out-of-distribution scale, saturates, and
        # the policy effectively ignores the scene).
        self._pc_xyz_mean = np.asarray(self.stats.get("pc_xyz_mean", [0.0, 0.0, 0.0]), dtype=np.float32)
        self._pc_xyz_std = np.maximum(np.asarray(self.stats.get("pc_xyz_std", [1.0, 1.0, 1.0]), dtype=np.float32), 1e-6)
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
        # Normalize the cloud EXACTLY like training (pc_dataset.normalize_pc_sample):
        # xyz standardized by pc_xyz stats, rgb [0,1] -> [-1,1]. Applied to the whole
        # (V,N,6) array including zero-padded inactive arms (training does the same).
        cloud = cloud.copy()
        cloud[..., :3] = (cloud[..., :3] - self._pc_xyz_mean) / self._pc_xyz_std
        cloud[..., 3:6] = (cloud[..., 3:6] - 0.5) / 0.5

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


class PointCloudRGBFlowActionSource(PointCloudFlowActionSource):
    """Deploy source for the variant-B RGB policy (pc_v1, model_config.variant=="b").

    Same pc_v1 schema as the cloud-only source, but the network is a
    PointCloudRGBFlowPolicy whose condition comes from a finetuned DINOv3 ConvNeXt
    over the LIVE wrist RGB (image_head=spatial -> 7x7 feature-map tokens for
    localization). RGB-only (use_pointcloud False) -> NO depth needed; the cloud
    arg is a zero placeholder the model ignores. Everything downstream (ee_local
    delta -> TcpPoseTarget, gripper, gating) is inherited unchanged.
    """

    def __init__(self, checkpoint_path: str, **kwargs: Any):
        super().__init__(checkpoint_path, **kwargs)
        self.policy_label = "pc-rgb flow policy"
        self.gripper_command_source = "pc_rgb_flow_policy"
        self._image_size = int(_DEFAULT_IMAGE_SIZE)
        self._use_pointcloud = bool(getattr(self.model.config, "use_pointcloud", False))
        # RGB-only: the camera gate only needs the COLOR streams (no depth).
        self.camera_names = [
            f"{side}_{self._color_stream}"
            for idx, side in enumerate(ARM_SIDES)
            if float(self.arm_mask[idx]) > 0.0
        ]
        self.requirements = replace(self.requirements, requires_camera=True)
        print(
            f"[flow-infer] pc_v1 variant=b (RGB): image_head spatial, use_pointcloud={self._use_pointcloud}, "
            f"image_size={self._image_size}, color stream={self._color_stream}",
            flush=True,
        )

    def _build_policy_model(self, model_config: dict[str, Any], checkpoint: dict[str, Any]) -> Any:
        from .pc_model_b import PointCloudRGBFlowConfig, PointCloudRGBFlowPolicy

        model = PointCloudRGBFlowPolicy(PointCloudRGBFlowConfig.from_mapping(dict(model_config))).to(self.device)
        model.load_state_dict(checkpoint["model_state"])
        model.eval()
        return model

    def _runtime_image(self, payload: dict[str, Any]) -> np.ndarray | None:
        """Live wrist RGB -> (V, 3, S, S) ImageNet-normalized, matching training
        (pc_preprocess._decode_resize_rgb: RGB, BILINEAR resize to S, no crop)."""
        from PIL import Image

        bundle = self._poll_camera_bundle()
        bundle_frames = getattr(bundle, "frames", {}) if bundle is not None else {}
        s = self._image_size
        views = []
        for idx, side in enumerate(ARM_SIDES):
            if float(self.arm_mask[idx]) <= 0.0:
                views.append(np.zeros((3, s, s), dtype=np.float32))
                continue
            frame = resolve_frame(bundle_frames, f"{side}_{self._color_stream}")
            pixels = getattr(frame, "pixels", None)
            if pixels is None:
                return None
            im = Image.fromarray(np.asarray(pixels)).convert("RGB").resize((s, s), Image.BILINEAR)
            arr = np.asarray(im, dtype=np.float32).transpose(2, 0, 1) / 255.0  # (3,S,S)
            views.append((arr - _IMAGENET_MEAN) / _IMAGENET_STD)
        return np.stack(views, axis=0)  # (V,3,S,S)

    def _sample_chunk(self, payload: dict[str, Any]) -> np.ndarray | None:
        import torch
        from .pc_model_b import pc_b_sample_action_chunks

        image = self._runtime_image(payload)
        if image is None:
            return None
        v = image.shape[0]
        # Zero cloud placeholder (use_pointcloud=False -> ignored by the model).
        cloud = np.zeros((v, self.num_points, 6), dtype=np.float32)
        # proprio_mode is "none" for the vision-only variant-B runs -> 1-dim placeholder.
        proprio = self._build_runtime_proprio(payload)

        pc = torch.as_tensor(cloud[None], dtype=torch.float32, device=self.device)
        im = torch.as_tensor(image[None], dtype=torch.float32, device=self.device)
        pr = torch.as_tensor(proprio[None], dtype=torch.float32, device=self.device)
        with torch.no_grad():
            chunk = pc_b_sample_action_chunks(self.model, pc, im, pr, steps=self.sample_steps)
            chunk = denormalize_actions(chunk, self.stats)
        return np.asarray(chunk.squeeze(0).cpu().numpy(), dtype=np.float32)
