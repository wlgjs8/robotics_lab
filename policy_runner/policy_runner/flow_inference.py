from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path
from typing import Any, TextIO

import numpy as np
import torch

from .action_sources.tcp_delta import (
    CARTESIAN_ACTION_REQUIREMENTS,
    clamp_tcp_delta,
    tcp_delta_stand_intent,
)
from .flow_dataset import (
    FLOW_CHECKPOINT_SCHEMA,
    decode_hdf5_image_value,
    normalize_runtime_proprio,
    pose_from_state_payload,
    runtime_proprio_from_state,
)
from .flow_model import FlowMatchingPolicy, sample_action_chunks
from .robot_state_client import StateSnapshot
from .servo_command_client import CommandIntent


class FlowMatchingActionSource:
    """Runtime source for high-level flow-policy action chunks.

    The source emits one bounded TcpDeltaStand step per policy_runner tick.
    SafetyGate still decides whether the intent may be sent.
    """

    def __init__(
        self,
        checkpoint_path: str | Path,
        *,
        timeout_sec: float = 0.2,
        camera_client: Any | None = None,
        sample_steps: int = 16,
        max_linear_step_m: float = 0.002,
        max_angular_step_rad: float = 0.01,
        device: str = "auto",
        stderr: TextIO = sys.stderr,
    ):
        if sample_steps <= 0:
            raise ValueError("sample_steps must be positive")
        self.timeout_sec = float(timeout_sec)
        self.camera_client = camera_client
        self.sample_steps = int(sample_steps)
        self.max_linear_step_m = float(max_linear_step_m)
        self.max_angular_step_rad = float(max_angular_step_rad)
        self.stderr = stderr
        self.device = _resolve_device(device)

        checkpoint = torch.load(checkpoint_path, map_location=self.device, weights_only=False)
        schema = str(checkpoint.get("schema", "") or "")
        if schema != FLOW_CHECKPOINT_SCHEMA:
            raise ValueError(f"unsupported flow checkpoint schema: {schema}")
        self.stats = dict(checkpoint["dataset_stats"])
        self.camera_names = [str(name) for name in checkpoint.get("camera_names", [])]
        self.image_size = int(checkpoint.get("image_size", 224))
        model_config = dict(checkpoint.get("model_config", {}))
        if not model_config:
            model_config = {
                "action_horizon": int(checkpoint["action_horizon"]),
                "action_dim": int(checkpoint["action_dim"]),
                "proprio_dim": int(checkpoint["proprio_dim"]),
                "camera_names": self.camera_names,
            }
        self.model = FlowMatchingPolicy.from_checkpoint_config(model_config).to(self.device)
        self.model.load_state_dict(checkpoint["model_state"])
        self.model.eval()

        self.arm_mask = _arm_mask_from_stats(self.stats)
        self.requirements = replace(
            CARTESIAN_ACTION_REQUIREMENTS,
            requires_camera=bool(self.camera_names),
        )
        self._reset_left_pose: np.ndarray | None = None
        self._reset_right_pose: np.ndarray | None = None
        self._chunk: np.ndarray | None = None
        self._chunk_index = 0
        self._warned_missing_camera_client = False
        self.last_image_decode_count = 0
        self.last_missing_camera_count = 0

    def next_intent(self, snapshot: StateSnapshot, now_monotonic: float) -> CommandIntent | None:
        _ = now_monotonic
        payload = snapshot.payload
        if self._reset_left_pose is None or self._reset_right_pose is None:
            self._reset_left_pose = pose_from_state_payload(payload, "left")
            self._reset_right_pose = pose_from_state_payload(payload, "right")

        if self._chunk is None or self._chunk_index >= self._chunk.shape[0]:
            self._chunk = self._sample_chunk(payload)
            if self._chunk is None:
                return None
            self._chunk_index = 0
        step = self._chunk[self._chunk_index]
        self._chunk_index += 1

        left = None
        right = None
        if self.arm_mask[0] > 0.0:
            left = clamp_tcp_delta(
                step[0:6].tolist(),
                self.max_linear_step_m,
                self.max_angular_step_rad,
            )
        if self.arm_mask[1] > 0.0:
            right = clamp_tcp_delta(
                step[7:13].tolist(),
                self.max_linear_step_m,
                self.max_angular_step_rad,
            )
        if left is not None and all(value == 0.0 for value in left):
            left = None
        if right is not None and all(value == 0.0 for value in right):
            right = None
        if left is None and right is None:
            return None
        return tcp_delta_stand_intent(left=left, right=right, timeout_sec=self.timeout_sec)

    def close(self) -> None:
        close = getattr(self.camera_client, "close", None)
        if callable(close):
            close()

    def _sample_chunk(self, payload: dict[str, Any]) -> np.ndarray | None:
        assert self._reset_left_pose is not None
        assert self._reset_right_pose is not None
        proprio = runtime_proprio_from_state(
            payload,
            reset_left_pose=self._reset_left_pose,
            reset_right_pose=self._reset_right_pose,
            arm_mask=self.arm_mask,
        )
        proprio = normalize_runtime_proprio(proprio, self.stats)
        images, decode_count, missing_count = self._runtime_images()
        self.last_image_decode_count = decode_count
        self.last_missing_camera_count = missing_count
        if self.camera_names and missing_count > 0:
            return None
        images = _normalize_images(images, self.stats)
        with torch.no_grad():
            chunk = sample_action_chunks(
                self.model,
                torch.as_tensor(images[None, ...], dtype=torch.float32, device=self.device),
                torch.as_tensor(proprio[None, ...], dtype=torch.float32, device=self.device),
                steps=self.sample_steps,
            )
            chunk = _denormalize_action_numpy(chunk, self.stats)
        return chunk[0]

    def _runtime_images(self) -> tuple[np.ndarray, int, int]:
        if not self.camera_names:
            return np.zeros((0, 3, self.image_size, self.image_size), dtype=np.float32), 0, 0
        bundle = None
        if self.camera_client is not None:
            poll = getattr(self.camera_client, "poll", None)
            if callable(poll):
                try:
                    bundle = poll(timeout_ms=0)
                except TypeError:
                    bundle = poll(0)
            if bundle is None:
                latest = getattr(self.camera_client, "latest", None)
                if callable(latest):
                    bundle = latest()
            is_fresh = getattr(self.camera_client, "is_fresh", None)
            if callable(is_fresh) and not is_fresh(bundle):
                bundle = None
        elif not self._warned_missing_camera_client:
            self._warned_missing_camera_client = True
            print(
                "WARNING: flow policy checkpoint expects cameras, but camera.enable is false; "
                "the flow action source will emit no motion intents until required frames are available.",
                file=self.stderr,
            )

        frames: list[np.ndarray] = []
        decode_count = 0
        missing_count = 0
        bundle_frames = getattr(bundle, "frames", {}) if bundle is not None else {}
        for camera_name in self.camera_names:
            frame = bundle_frames.get(camera_name) if isinstance(bundle_frames, dict) else None
            pixels = getattr(frame, "pixels", None)
            if pixels is None:
                frames.append(np.zeros((3, self.image_size, self.image_size), dtype=np.float32))
                missing_count += 1
                continue
            try:
                frames.append(
                    decode_hdf5_image_value(
                        np.asarray(pixels),
                        image_size=self.image_size,
                    )
                )
                decode_count += 1
            except Exception:
                frames.append(np.zeros((3, self.image_size, self.image_size), dtype=np.float32))
                missing_count += 1
        return np.stack(frames, axis=0), decode_count, missing_count


def _arm_mask_from_stats(stats: dict[str, Any]) -> np.ndarray:
    counts = stats.get("arm_mask_counts", {})
    if isinstance(counts, dict):
        mask = np.asarray(
            [
                1.0 if int(counts.get("left", 0) or 0) > 0 else 0.0,
                1.0 if int(counts.get("right", 0) or 0) > 0 else 0.0,
            ],
            dtype=np.float32,
        )
    else:
        mask = np.ones(2, dtype=np.float32)
    if not mask.any():
        mask[:] = 1.0
    return mask


def _normalize_images(images: np.ndarray, stats: dict[str, Any]) -> np.ndarray:
    if not images.size:
        return images.astype(np.float32, copy=False)
    mean = np.asarray(stats.get("image_mean", [0.0, 0.0, 0.0]), dtype=np.float32)
    std = np.asarray(stats.get("image_std", [1.0, 1.0, 1.0]), dtype=np.float32)
    return ((images - mean.reshape(1, 3, 1, 1)) / np.maximum(std.reshape(1, 3, 1, 1), 1e-6)).astype(
        np.float32,
        copy=False,
    )


def _denormalize_action_numpy(actions: torch.Tensor, stats: dict[str, Any]) -> np.ndarray:
    mean = torch.as_tensor(stats["action_mean"], dtype=actions.dtype, device=actions.device)
    std = torch.as_tensor(stats["action_std"], dtype=actions.dtype, device=actions.device)
    return (actions * std.view(1, 1, -1) + mean.view(1, 1, -1)).detach().cpu().numpy()


def _resolve_device(device: str) -> torch.device:
    if device == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    return torch.device(device)
