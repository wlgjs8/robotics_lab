from __future__ import annotations

import sys
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, TextIO

import numpy as np
import torch

from .action_sources.tcp_delta import (
    clamp_tcp_twist,
    clamp_tcp_delta,
    cartesian_action_requirements,
    tcp_delta_stand_intent,
    tcp_twist_local_intent,
    tcp_twist_stand_intent,
)
from .flow_dataset import (
    FLOW_CHECKPOINT_SCHEMA,
    FLOW_ACTION_DIM_NAMES,
    FlowHdf5Dataset,
    decode_hdf5_image_value,
    normalize_runtime_proprio,
    pose_from_state_payload,
    runtime_proprio_from_state,
)
from .flow_model import FlowMatchingPolicy, sample_action_chunks
from .gripper import REAL_GRIPPER_ENV, GripperRuntime, gripper_commands_from_flow_step
from .robot_state_client import StateSnapshot
from .rollout_modes import RolloutMode, RolloutModeValidationError, parse_rollout_mode
from .servo_command_client import CommandIntent


FLOW_COMMAND_FAMILY_CHOICES = ("tcp_twist_stand", "tcp_twist_local", "tcp_delta_stand")
_FLOW_COMMAND_FAMILY_LABELS = {
    "tcp_twist_stand": "TcpTwistStand",
    "tcp_twist_local": "TcpTwistLocal",
    "tcp_delta_stand": "TcpDeltaStand",
}
_ZERO_TWIST = (0.0,) * 6
DEFAULT_FLOW_MAX_LINEAR_VELOCITY_M_S = 0.30
DEFAULT_FLOW_MAX_ANGULAR_VELOCITY_RAD_S = 2.0
_FLOW_STATS_VELOCITY_LIMIT_SCALE = 1.25


@dataclass(frozen=True)
class FlowOfflineEvalResult:
    sample_count: int
    action_chunk_count: int
    camera_names: list[str]
    image_decode_count: int
    missing_camera_count: int
    command_family: str = "TcpTwistStand"


def resolve_flow_command_family(
    rollout_mode: str | RolloutMode,
    command_family: str | None,
) -> str:
    """Return the normalized flow command family for a rollout mode."""

    _ = parse_rollout_mode(rollout_mode)
    if command_family is None:
        return "tcp_twist_stand"
    return normalize_flow_command_family(command_family)


def normalize_flow_command_family(command_family: str) -> str:
    family = str(command_family or "").strip().lower().replace("-", "_")
    family = {
        "tcptwiststand": "tcp_twist_stand",
        "tcptwistlocal": "tcp_twist_local",
        "tcpdeltastand": "tcp_delta_stand",
    }.get(family, family)
    if family not in FLOW_COMMAND_FAMILY_CHOICES:
        choices = ", ".join(FLOW_COMMAND_FAMILY_CHOICES)
        raise RolloutModeValidationError(
            f"invalid command-family {command_family!r}; expected one of: {choices}"
        )
    return family


def canonical_flow_command_family(command_family: str) -> str:
    return _FLOW_COMMAND_FAMILY_LABELS[normalize_flow_command_family(command_family)]


def validate_flow_command_family(
    rollout_mode: str | RolloutMode,
    command_family: str,
    *,
    allow_experimental_tcp_delta_stand: bool = False,
) -> None:
    mode = parse_rollout_mode(rollout_mode)
    family = normalize_flow_command_family(command_family)
    if family == "tcp_twist_stand":
        return
    if family == "tcp_twist_local":
        if mode in {RolloutMode.OFFLINE_EVAL, RolloutMode.SIM_DRYRUN}:
            return
        raise RolloutModeValidationError(
            "command-family tcp_twist_local is a debug flow rollout path; "
            "use tcp_twist_stand for controller_sim or real_policy"
        )
    if mode in {RolloutMode.OFFLINE_EVAL, RolloutMode.SIM_DRYRUN}:
        return
    if allow_experimental_tcp_delta_stand:
        return
    raise RolloutModeValidationError(
        "command-family tcp_delta_stand is a debug/experimental flow rollout path; "
        "use tcp_twist_stand or pass --allow-experimental-tcp-delta-stand"
    )


def resolve_flow_policy_dt_sec(
    rollout_mode: str | RolloutMode,
    command_family: str,
    *,
    policy_dt_sec: float | None,
    command_rate_hz: float,
    dataset_stats: dict[str, Any] | None = None,
) -> float | None:
    family = normalize_flow_command_family(command_family)
    if family not in {"tcp_twist_stand", "tcp_twist_local"}:
        return None
    if policy_dt_sec is not None:
        resolved = float(policy_dt_sec)
        if resolved <= 0.0:
            raise RolloutModeValidationError("--policy-dt-sec must be positive")
        return resolved
    stats_dt = _positive_float((dataset_stats or {}).get("dt_mean_sec"))
    if stats_dt is not None:
        return stats_dt

    mode = parse_rollout_mode(rollout_mode)
    if mode in {RolloutMode.CONTROLLER_SIM, RolloutMode.REAL_POLICY}:
        raise RolloutModeValidationError(
            f"command-family {family} requires --policy-dt-sec or checkpoint "
            f"dataset_stats.dt_mean_sec for rollout-mode {mode.value}"
        )
    rate = float(command_rate_hz)
    if rate <= 0.0:
        raise RolloutModeValidationError("command_rate_hz must be positive to derive policy dt")
    return 1.0 / rate


class FlowMatchingActionSource:
    """Runtime source for high-level flow-policy action chunks.

    The source emits bounded stand-frame streaming commands by default. TcpDeltaStand
    and TcpTwistLocal remain debug families behind rollout-mode validation. SafetyGate
    still decides whether the intent may be sent.
    """

    def __init__(
        self,
        checkpoint_path: str | Path,
        *,
        timeout_sec: float = 0.2,
        camera_client: Any | None = None,
        sample_steps: int = 16,
        command_family: str = "tcp_twist_stand",
        policy_dt_sec: float | None = None,
        max_linear_velocity_m_s: float | None = None,
        max_angular_velocity_rad_s: float | None = None,
        max_linear_step_m: float = 0.002,
        max_angular_step_rad: float = 0.01,
        chunk_execute_steps: int | None = None,
        allow_rbpodo_controller_simulation_cartesian: bool = False,
        gripper_runtime: GripperRuntime | None = None,
        device: str = "auto",
        stderr: TextIO = sys.stderr,
    ):
        if sample_steps <= 0:
            raise ValueError("sample_steps must be positive")
        self.timeout_sec = float(timeout_sec)
        self.camera_client = camera_client
        self.sample_steps = int(sample_steps)
        self.command_family_option = normalize_flow_command_family(command_family)
        self.command_family = canonical_flow_command_family(self.command_family_option)
        if policy_dt_sec is not None and policy_dt_sec <= 0.0:
            raise ValueError("policy_dt_sec must be positive")
        if max_linear_velocity_m_s is not None and max_linear_velocity_m_s < 0.0:
            raise ValueError("max_linear_velocity_m_s must be non-negative")
        if max_angular_velocity_rad_s is not None and max_angular_velocity_rad_s < 0.0:
            raise ValueError("max_angular_velocity_rad_s must be non-negative")
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
        self.action_horizon = int(model_config.get("action_horizon", checkpoint.get("action_horizon", 0)) or 0)
        self.chunk_execute_steps = _resolve_chunk_execute_steps(chunk_execute_steps, self.action_horizon)
        self.policy_dt_sec = _resolve_runtime_policy_dt_sec(
            self.command_family_option,
            policy_dt_sec,
            self.stats,
        )
        self.max_linear_velocity_m_s = _resolve_velocity_limit_from_stats(
            configured=max_linear_velocity_m_s,
            stats=self.stats,
            policy_dt_sec=self.policy_dt_sec,
            names=("left_dx", "left_dy", "left_dz", "right_dx", "right_dy", "right_dz"),
            fallback=DEFAULT_FLOW_MAX_LINEAR_VELOCITY_M_S,
        )
        self.max_angular_velocity_rad_s = _resolve_velocity_limit_from_stats(
            configured=max_angular_velocity_rad_s,
            stats=self.stats,
            policy_dt_sec=self.policy_dt_sec,
            names=("left_drx", "left_dry", "left_drz", "right_drx", "right_dry", "right_drz"),
            fallback=DEFAULT_FLOW_MAX_ANGULAR_VELOCITY_RAD_S,
        )
        self.model = FlowMatchingPolicy.from_checkpoint_config(model_config).to(self.device)
        self.model.load_state_dict(checkpoint["model_state"])
        self.model.eval()

        self.arm_mask = _arm_mask_from_stats(self.stats)
        self.checkpoint_arm_mask = tuple(float(value) for value in self.arm_mask.tolist())
        self.checkpoint_selected_arms = _arms_from_mask(self.arm_mask)
        self.checkpoint_has_nonzero_gripper_commands = _checkpoint_has_nonzero_gripper_commands(
            self.stats,
            int(model_config.get("action_dim", checkpoint.get("action_dim", 0)) or 0),
        )
        self.gripper_runtime = gripper_runtime or GripperRuntime(rollout_mode="sim_dryrun")
        self.requirements = replace(
            cartesian_action_requirements(
                allow_rbpodo_controller_simulation=allow_rbpodo_controller_simulation_cartesian,
            ),
            requires_camera=bool(self.camera_names),
        )
        self._reset_left_pose: np.ndarray | None = None
        self._reset_right_pose: np.ndarray | None = None
        self._chunk: np.ndarray | None = None
        self._chunk_index = 0
        self._warned_missing_camera_client = False
        self.last_image_decode_count = 0
        self.last_missing_camera_count = 0
        self.image_decode_count = 0
        self.missing_camera_count = 0
        self._last_nonzero_twist_by_arm = {"left": False, "right": False}
        self._gripper_targets_by_arm: dict[str, float | None] = {"left": None, "right": None}

    def next_intent(self, snapshot: StateSnapshot, now_monotonic: float) -> CommandIntent | None:
        _ = now_monotonic
        payload = snapshot.payload
        if self._reset_left_pose is None or self._reset_right_pose is None:
            self._reset_left_pose = pose_from_state_payload(payload, "left")
            self._reset_right_pose = pose_from_state_payload(payload, "right")

        if (
            self.camera_names
            and self._chunk is not None
            and self._chunk_index < self._current_chunk_execute_limit()
            and not self._camera_inputs_ready()
        ):
            self._chunk = None
            self._chunk_index = 0
            return self._no_policy_input_intent()

        if self._chunk is None or self._chunk_index >= self._current_chunk_execute_limit():
            self._chunk = self._sample_chunk(payload)
            if self._chunk is None:
                return self._no_policy_input_intent()
            self._chunk_index = 0
        step = self._chunk[self._chunk_index]
        self._chunk_index += 1
        gripper_targets = self._integrate_gripper_targets(step, payload)
        self._dispatch_gripper_step(step)

        if self.command_family_option == "tcp_twist_local":
            return self._tcp_twist_local_step_intent(step, gripper_targets=gripper_targets)
        if self.command_family_option == "tcp_twist_stand":
            return self._tcp_twist_stand_step_intent(step, gripper_targets=gripper_targets)
        return self._tcp_delta_stand_step_intent(step, gripper_targets=gripper_targets)

    @property
    def gripper_command_count(self) -> int:
        return int(self.gripper_runtime.command_count)

    @property
    def gripper_dropped_count(self) -> int:
        return int(self.gripper_runtime.dropped_count)

    def _dispatch_gripper_step(self, step: np.ndarray) -> None:
        commands = gripper_commands_from_flow_step(
            step.tolist(),
            arm_mask=self.arm_mask.tolist(),
            command_type="delta",
            source="flow_policy",
        )
        self.gripper_runtime.dispatch(commands)

    def _tcp_delta_stand_step_intent(
        self,
        step: np.ndarray,
        *,
        gripper_targets: dict[str, float | None],
    ) -> CommandIntent | None:
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
        return tcp_delta_stand_intent(
            left=left,
            right=right,
            left_gripper=gripper_targets.get("left"),
            right_gripper=gripper_targets.get("right"),
            timeout_sec=self.timeout_sec,
        )

    def _tcp_twist_local_step_intent(
        self,
        step: np.ndarray,
        *,
        gripper_targets: dict[str, float | None],
    ) -> CommandIntent | None:
        left = self._twist_payload_for_arm("left", step[0:6]) if self.arm_mask[0] > 0.0 else None
        right = self._twist_payload_for_arm("right", step[7:13]) if self.arm_mask[1] > 0.0 else None
        if left is None and right is None:
            return None
        return tcp_twist_local_intent(
            left=left,
            right=right,
            left_gripper=gripper_targets.get("left"),
            right_gripper=gripper_targets.get("right"),
            timeout_sec=self.timeout_sec,
        )

    def _tcp_twist_stand_step_intent(
        self,
        step: np.ndarray,
        *,
        gripper_targets: dict[str, float | None],
    ) -> CommandIntent | None:
        left = self._twist_payload_for_arm("left", step[0:6]) if self.arm_mask[0] > 0.0 else None
        right = self._twist_payload_for_arm("right", step[7:13]) if self.arm_mask[1] > 0.0 else None
        if left is None and right is None:
            return None
        return tcp_twist_stand_intent(
            left=left,
            right=right,
            left_gripper=gripper_targets.get("left"),
            right_gripper=gripper_targets.get("right"),
            timeout_sec=self.timeout_sec,
        )

    def _twist_payload_for_arm(self, arm: str, delta: np.ndarray) -> tuple[float, ...] | None:
        twist = self._delta_to_twist(delta)
        if _has_nonzero(twist):
            self._last_nonzero_twist_by_arm[arm] = True
            return twist
        if self._last_nonzero_twist_by_arm[arm]:
            self._last_nonzero_twist_by_arm[arm] = False
            return _ZERO_TWIST
        return None

    def _delta_to_twist(self, delta: np.ndarray) -> tuple[float, ...]:
        assert self.policy_dt_sec is not None
        values = np.asarray(delta, dtype=np.float64).reshape(-1)
        if values.shape[0] != 6:
            raise ValueError("flow Cartesian delta must contain 6 values")
        twist = values / self.policy_dt_sec
        return clamp_tcp_twist(
            twist.tolist(),
            self.max_linear_velocity_m_s,
            self.max_angular_velocity_rad_s,
        )

    def _no_policy_input_intent(self) -> CommandIntent | None:
        if self.command_family_option not in {"tcp_twist_stand", "tcp_twist_local"}:
            return None
        left = _ZERO_TWIST if self._last_nonzero_twist_by_arm["left"] else None
        right = _ZERO_TWIST if self._last_nonzero_twist_by_arm["right"] else None
        if left is None and right is None:
            return None
        self._last_nonzero_twist_by_arm["left"] = False
        self._last_nonzero_twist_by_arm["right"] = False
        if self.command_family_option == "tcp_twist_stand":
            return tcp_twist_stand_intent(left=left, right=right, timeout_sec=self.timeout_sec)
        return tcp_twist_local_intent(left=left, right=right, timeout_sec=self.timeout_sec)

    def _integrate_gripper_targets(
        self,
        step: np.ndarray,
        payload: dict[str, Any],
    ) -> dict[str, float | None]:
        targets: dict[str, float | None] = {"left": None, "right": None}
        if not self._allow_gripper_targets_in_motion_packet():
            return targets
        for arm, mask_index, step_index in (("left", 0, 6), ("right", 1, 13)):
            if len(self.arm_mask) <= mask_index or self.arm_mask[mask_index] <= 0.0:
                continue
            current = _gripper_value_from_payload(payload, arm)
            if self._gripper_targets_by_arm[arm] is None:
                self._gripper_targets_by_arm[arm] = 0.0 if current is None else current
            if step.shape[0] > step_index:
                self._gripper_targets_by_arm[arm] = float(self._gripper_targets_by_arm[arm]) + float(step[step_index])
            targets[arm] = self._gripper_targets_by_arm[arm]
        return targets

    def _allow_gripper_targets_in_motion_packet(self) -> bool:
        mode = str(getattr(self.gripper_runtime, "rollout_mode", "") or "").strip().lower()
        if mode == "real_policy":
            if not bool(getattr(self.gripper_runtime, "allow_real_gripper_motion", False)):
                return False
            env_getter = getattr(self.gripper_runtime, "_env", None)
            values = env_getter() if callable(env_getter) else {}
            return values.get(REAL_GRIPPER_ENV) == "1"
        return mode in {"controller_sim", "sim_dryrun", "offline_eval", "real_readonly"}

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
        self.image_decode_count += decode_count
        self.missing_camera_count += missing_count
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

    def _current_chunk_execute_limit(self) -> int:
        if self._chunk is None:
            return 0
        return min(int(self.chunk_execute_steps), int(self._chunk.shape[0]))

    def _camera_inputs_ready(self) -> bool:
        if not self.camera_names:
            return True
        bundle = self._poll_camera_bundle()
        missing_count = self._count_missing_camera_frames(bundle)
        self.last_image_decode_count = 0
        self.last_missing_camera_count = missing_count
        self.missing_camera_count += missing_count
        return missing_count == 0

    def _runtime_images(self) -> tuple[np.ndarray, int, int]:
        if not self.camera_names:
            return np.zeros((0, 3, self.image_size, self.image_size), dtype=np.float32), 0, 0
        bundle = self._poll_camera_bundle()

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

    def _poll_camera_bundle(self) -> Any | None:
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
        return bundle

    def _count_missing_camera_frames(self, bundle: Any | None) -> int:
        missing_count = 0
        bundle_frames = getattr(bundle, "frames", {}) if bundle is not None else {}
        for camera_name in self.camera_names:
            frame = bundle_frames.get(camera_name) if isinstance(bundle_frames, dict) else None
            pixels = getattr(frame, "pixels", None)
            if pixels is None:
                missing_count += 1
        return missing_count


def run_flow_offline_eval(
    *,
    checkpoint_path: str | Path,
    episodes_dir: str | Path,
    sample_steps: int = 16,
    device: str = "auto",
    max_samples: int = 1,
    command_family: str = "TcpTwistStand",
) -> FlowOfflineEvalResult:
    if sample_steps <= 0:
        raise ValueError("sample_steps must be positive")
    if max_samples <= 0:
        raise ValueError("max_offline_samples must be positive")
    torch_device = _resolve_device(device)
    checkpoint = torch.load(checkpoint_path, map_location=torch_device, weights_only=False)
    schema = str(checkpoint.get("schema", "") or "")
    if schema != FLOW_CHECKPOINT_SCHEMA:
        raise ValueError(f"unsupported flow checkpoint schema: {schema}")

    stats = dict(checkpoint["dataset_stats"])
    camera_names = [str(name) for name in checkpoint.get("camera_names", [])]
    image_size = int(checkpoint.get("image_size", 224))
    model_config = dict(checkpoint.get("model_config", {}))
    if not model_config:
        model_config = {
            "action_horizon": int(checkpoint["action_horizon"]),
            "action_dim": int(checkpoint["action_dim"]),
            "proprio_dim": int(checkpoint["proprio_dim"]),
            "camera_names": camera_names,
        }
    model = FlowMatchingPolicy.from_checkpoint_config(model_config).to(torch_device)
    model.load_state_dict(checkpoint["model_state"])
    model.eval()

    dataset = FlowHdf5Dataset(
        episodes_dir,
        action_horizon=int(model_config["action_horizon"]),
        image_size=image_size,
        camera_names=camera_names,
        stats=stats,
        normalize=True,
    )
    sample_count = min(int(max_samples), len(dataset))
    image_decode_count = 0
    missing_camera_count = 0
    action_chunk_count = 0
    for index in range(sample_count):
        sample = dataset[index]
        image_decode_count += int(sample["image_decode_count"])
        missing_camera_count += int(sample["missing_camera_count"])
        with torch.no_grad():
            chunk = sample_action_chunks(
                model,
                torch.as_tensor(
                    sample["images"][None, ...],
                    dtype=torch.float32,
                    device=torch_device,
                ),
                torch.as_tensor(
                    sample["proprio"][None, ...],
                    dtype=torch.float32,
                    device=torch_device,
                ),
                steps=sample_steps,
            )
        action_chunk_count += int(chunk.shape[1])

    return FlowOfflineEvalResult(
        sample_count=sample_count,
        action_chunk_count=action_chunk_count,
        camera_names=camera_names,
        image_decode_count=image_decode_count,
        missing_camera_count=missing_camera_count,
        command_family=command_family,
    )


def load_flow_checkpoint_dataset_stats(
    checkpoint_path: str | Path,
    *,
    device: str = "cpu",
) -> dict[str, Any]:
    checkpoint = torch.load(checkpoint_path, map_location=_resolve_device(device), weights_only=False)
    schema = str(checkpoint.get("schema", "") or "")
    if schema != FLOW_CHECKPOINT_SCHEMA:
        raise ValueError(f"unsupported flow checkpoint schema: {schema}")
    stats = checkpoint.get("dataset_stats", {})
    return dict(stats) if isinstance(stats, dict) else {}


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


def _arms_from_mask(mask: np.ndarray) -> list[str]:
    arms: list[str] = []
    if len(mask) >= 1 and float(mask[0]) > 0.0:
        arms.append("left")
    if len(mask) >= 2 and float(mask[1]) > 0.0:
        arms.append("right")
    return arms or ["left", "right"]


def _checkpoint_has_nonzero_gripper_commands(stats: dict[str, Any], action_dim: int) -> bool:
    percentiles = stats.get("action_distribution_percentiles", {})
    if isinstance(percentiles, dict):
        found_gripper_percentiles = False
        for name in ("left_grip", "right_grip"):
            values = percentiles.get(name)
            if not isinstance(values, dict):
                continue
            found_gripper_percentiles = True
            for value in values.values():
                if isinstance(value, (int, float)) and not isinstance(value, bool):
                    if abs(float(value)) > 1e-12:
                        return True
        if found_gripper_percentiles:
            return False

    names = list(FLOW_ACTION_DIM_NAMES)
    if action_dim >= len(names):
        return True
    return action_dim > max(names.index("left_grip"), names.index("right_grip"))


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


def _resolve_runtime_policy_dt_sec(
    command_family: str,
    policy_dt_sec: float | None,
    stats: dict[str, Any],
) -> float | None:
    family = normalize_flow_command_family(command_family)
    if family not in {"tcp_twist_stand", "tcp_twist_local"}:
        return None
    resolved = _positive_float(policy_dt_sec)
    if resolved is not None:
        return resolved
    resolved = _positive_float(stats.get("dt_mean_sec"))
    if resolved is not None:
        return resolved
    raise ValueError(
        f"policy_dt_sec or checkpoint dataset_stats.dt_mean_sec is required for {family} flow commands"
    )


def _resolve_chunk_execute_steps(value: int | None, action_horizon: int) -> int:
    if action_horizon <= 0:
        raise ValueError("flow checkpoint action_horizon must be positive")
    if value is None:
        return max(1, int(action_horizon) // 2)
    resolved = int(value)
    if resolved <= 0:
        raise ValueError("chunk_execute_steps must be positive")
    if resolved > int(action_horizon):
        raise ValueError("chunk_execute_steps must not exceed checkpoint action_horizon")
    return resolved


def _resolve_velocity_limit_from_stats(
    *,
    configured: float | None,
    stats: dict[str, Any],
    policy_dt_sec: float | None,
    names: tuple[str, ...],
    fallback: float,
) -> float:
    if configured is not None:
        return float(configured)
    dt = _positive_float(policy_dt_sec)
    percentiles = stats.get("action_distribution_percentiles", {})
    values: list[float] = []
    if dt is not None and isinstance(percentiles, dict):
        for name in names:
            entry = percentiles.get(name)
            if not isinstance(entry, dict):
                continue
            for key in ("p01", "p05", "p95", "p99"):
                value = _positive_abs_float(entry.get(key))
                if value is not None:
                    values.append(value)
    if values and dt is not None:
        return max(values) * _FLOW_STATS_VELOCITY_LIMIT_SCALE / dt
    return float(fallback)


def _positive_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        resolved = float(value)
    except (TypeError, ValueError):
        return None
    if not np.isfinite(resolved) or resolved <= 0.0:
        return None
    return resolved


def _positive_abs_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        resolved = abs(float(value))
    except (TypeError, ValueError):
        return None
    if not np.isfinite(resolved) or resolved <= 1e-12:
        return None
    return resolved


def _gripper_value_from_payload(payload: dict[str, Any], arm: str) -> float | None:
    arm_payload = payload.get(arm, {})
    if not isinstance(arm_payload, dict):
        return None
    for container_name in ("gripper", "gripper_state"):
        container = arm_payload.get(container_name)
        if isinstance(container, dict):
            for key in ("gripper_position", "position", "target", "value"):
                value = _positive_or_zero_float(container.get(key))
                if value is not None:
                    return value
    for key in ("gripper_position", "gripper_target", "gripper", "gripper_value"):
        value = _positive_or_zero_float(arm_payload.get(key))
        if value is not None:
            return value
    return None


def _positive_or_zero_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        resolved = float(value)
    except (TypeError, ValueError):
        return None
    if not np.isfinite(resolved):
        return None
    return resolved


def _resolve_device(device: str) -> torch.device:
    if device == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    return torch.device(device)


def _has_nonzero(values: tuple[float, ...] | list[float]) -> bool:
    return any(abs(float(value)) > 1e-12 for value in values)
