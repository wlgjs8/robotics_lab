from __future__ import annotations

import hashlib
import json
import os
import sys
import threading
import time
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, TextIO

import numpy as np
import torch

from .action_sources.tcp_delta import (
    clamp_tcp_delta,
    clamp_tcp_twist,
    cartesian_action_requirements,
    tcp_pose_target_stand_intent,
    tcp_twist_local_intent,
)
from .flow_dataset import (
    DEFAULT_ACTION_FRAME,
    FLOW_CHECKPOINT_SCHEMA,
    FLOW_ACTION_DIM,
    FLOW_ACTION_DIM_NAMES,
    FLOW_PROPRIO_DIM,
    FlowHdf5Dataset,
    decode_hdf5_image_value,
    normalize_action_frame,
    normalize_runtime_proprio,
    pose_compose_local,
    pose_from_state_payload,
    runtime_proprio_from_state,
)
from .camera_bundle_client import resolve_frame
from .flow_model import FlowMatchingPolicy, sample_action_chunks
from .tcp_target_pose_conditioner import (
    CONDITIONING_MODES,
    REANCHOR_MODES,
    OnlineTcpTargetPoseConditioner,
)
from .gripper import REAL_GRIPPER_ENV, GripperRuntime, gripper_commands_from_flow_step
from .robot_state_client import StateSnapshot
from .rollout_modes import RolloutMode, RolloutModeValidationError, parse_rollout_mode
from .servo_command_client import CommandIntent


def default_action_log_path() -> str:
    """Resolve the per-step action-log path.

    Honors ``POLICY_RUNNER_ACTION_LOG`` when set; otherwise auto-accumulates one
    timestamped file per run under the repo ``logs/`` dir, so a plain
    ``flow-infer`` (no env var on the command line) still captures actions.
    """
    configured = os.environ.get("POLICY_RUNNER_ACTION_LOG")
    if configured:
        return configured
    logs_dir = Path(__file__).resolve().parents[2] / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    return str(logs_dir / f"actions_{time.strftime('%Y%m%d_%H%M%S')}.jsonl")


# Body-frame (ee_local) policy deltas can be consumed either as velocity
# commands or as integrated absolute TCP pose targets.
FLOW_COMMAND_FAMILY_CHOICES = ("tcp_twist_local", "tcp_target_pose")
_FLOW_COMMAND_FAMILY_LABELS = {
    "tcp_twist_local": "TcpTwistLocal",
    "tcp_target_pose": "TcpPoseTarget",
}
_ZERO_TWIST = (0.0,) * 6
DEFAULT_FLOW_MAX_LINEAR_VELOCITY_M_S = 0.30
DEFAULT_FLOW_MAX_ANGULAR_VELOCITY_RAD_S = 2.0
_FLOW_STATS_VELOCITY_LIMIT_SCALE = 1.25
IMITATION_CHECKPOINT_SCHEMA = "robotics_lab.policy_runner.imitation_checkpoint.v1"
IMITATION_ENSEMBLE_REPORT_SCHEMA = "robotics_lab.policy_runner.imitation_ensemble_report.v1"
DIRECT_BC_RUNTIME_FAMILIES = {
    "direct_bc_chunk",
    "direct_bc_distilled_cached_ensemble",
    "arm_structured_direct",
}


@dataclass(frozen=True)
class FlowOfflineEvalResult:
    sample_count: int
    action_chunk_count: int
    camera_names: list[str]
    image_decode_count: int
    missing_camera_count: int
    command_family: str = "TcpTwistLocal"
    selected_arms: list[str] = field(default_factory=list)
    checkpoint_arm_mask: tuple[float, float] | None = None
    checkpoint_has_nonzero_gripper_commands: bool = False


@dataclass(frozen=True)
class _DirectBcEnsembleBundle:
    name: str
    member_paths: list[str]
    models: list[Any]
    stats: dict[str, Any]
    camera_names: list[str]
    image_size: int
    action_horizon: int
    arm_mask: np.ndarray


def resolve_flow_command_family(
    rollout_mode: str | RolloutMode,
    command_family: str | None,
    *,
    dataset_stats: dict[str, Any] | None = None,
) -> str:
    """Return the normalized flow command family for a rollout mode."""

    _ = parse_rollout_mode(rollout_mode)
    _ = dataset_stats
    if command_family is None:
        return "tcp_target_pose"
    return normalize_flow_command_family(command_family)


def normalize_flow_command_family(command_family: str) -> str:
    family = str(command_family or "").strip().lower().replace("-", "_")
    family = {
        "tcptwistlocal": "tcp_twist_local",
        "tcpposetarget": "tcp_target_pose",
        "tcp_pose_target": "tcp_target_pose",
    }.get(family, family)
    if family not in FLOW_COMMAND_FAMILY_CHOICES:
        choices = ", ".join(FLOW_COMMAND_FAMILY_CHOICES)
        raise RolloutModeValidationError(
            f"invalid command-family {command_family!r}; expected one of: {choices}"
        )
    return family


def canonical_flow_command_family(command_family: str) -> str:
    return _FLOW_COMMAND_FAMILY_LABELS[normalize_flow_command_family(command_family)]


# ---- ee_local body-frame alignment (training EE frame vs runtime TCP frame) ----
# Fixed rotation R between the training EE body frame (in the data) and the RB TCP
# frame the server interprets TcpTwistLocal in. Applied symmetrically:
#   v_train = R · v_tcp            (runtime proprio -> training frame)
#   v_tcp   = Rᵀ · v_train         (policy action  -> robot TCP frame)
#
# MEASURED for pika UMI (axis-probe 2026-06-15, see wiki umi-axis-probe): the data's
# baked retarget (R_corr·Trans·R_align) is correct about z=approach but carries a
# 180° YAW about approach (x,y flipped) — the config's unconfirmed R_corr. The 6
# single-axis ground-truth replays solve (Kabsch, 5/6 residual 0) to:
#   R = diag(-1,-1,+1) = 180° about approach(z)  ->  preset "pika_rz180"  <-- ALWAYS USE THIS
# `none` gets z right but is yaw-flipped. Permanent fix = compose pika_rz180 into
# R_corr in calibration/umi_retarget_eelocal.yaml + reconvert + retrain; until then
# pass `--ee-local-r-align pika_rz180` (symmetric -> corrects the existing
# checkpoint, no retrain; the wrist image is unaffected by this coordinate relabel).
# (The old "pika_tip" 90°-permutation guess was removed — pika UMI always uses pika_rz180.)
EE_LOCAL_R_ALIGN_PRESETS: dict[str, tuple[float, ...]] = {
    # measured pika-UMI correction: 180° about approach(z) (axis-probe 2026-06-15)
    "pika_rz180": (-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0),
}


@dataclass(frozen=True, eq=False)
class EeLocalRAlign:
    """Body-frame relabel for ee_local checkpoints, applied per channel.

    `linear` rotates the translation 3-vectors, `angular` the rotation
    3-vectors of every flow proprio/action row. For a true rigid frame change
    the two are identical (a single rotation), which is the normal case. They
    differ only for diagnostic sign-convention presets such as
    ``pika_rz180_trans_only`` (flip x/y translation but leave rotation as-is),
    which is NOT a valid rigid transform and exists purely for axis ablation.
    """

    linear: np.ndarray
    angular: np.ndarray

    @property
    def T(self) -> "EeLocalRAlign":
        return EeLocalRAlign(np.asarray(self.linear).T, np.asarray(self.angular).T)


def _validate_r_align_matrix(value: Any) -> np.ndarray:
    matrix = np.asarray(list(value), dtype=np.float64)
    if matrix.size != 9:
        raise ValueError("ee-local-r-align must have exactly 9 elements (row-major 3x3)")
    matrix = matrix.reshape(3, 3)
    if not np.allclose(matrix @ matrix.T, np.eye(3), atol=1e-6):
        raise ValueError("ee-local-r-align must be a rotation matrix (orthonormal)")
    return matrix


# Split presets: linear != angular, so they cannot be expressed as a single
# orthonormal matrix. `pika_rz180_trans_only` flips x/y translation (the
# pika_rz180 linear correction) but keeps the rotation channel untouched — an
# ablation to test whether rx/ry actually need the same 180° flip the
# translation does (axis-probe 2026-06-15 says they do; this lets you A/B it).
_PIKA_RZ180 = np.array([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]])
EE_LOCAL_R_ALIGN_SPLIT_PRESETS: dict[str, EeLocalRAlign] = {
    "pika_rz180_trans_only": EeLocalRAlign(linear=_PIKA_RZ180, angular=np.eye(3)),
}


def resolve_ee_local_r_align(value: Any) -> EeLocalRAlign | None:
    """None/'none' -> None; preset name or 9 row-major floats -> EeLocalRAlign.

    A plain rotation (preset or 9 floats) yields linear == angular; split
    presets (e.g. ``pika_rz180_trans_only``) yield differing channels.
    """
    if value is None or isinstance(value, EeLocalRAlign):
        return value
    if isinstance(value, str):
        key = value.strip().lower().replace("-", "_")
        if key in {"", "none", "identity"}:
            return None
        if key in EE_LOCAL_R_ALIGN_SPLIT_PRESETS:
            return EE_LOCAL_R_ALIGN_SPLIT_PRESETS[key]
        if key in EE_LOCAL_R_ALIGN_PRESETS:
            value = EE_LOCAL_R_ALIGN_PRESETS[key]
        else:
            try:
                value = [float(item) for item in value.replace(",", " ").split()]
            except ValueError as exc:
                presets = ", ".join(
                    sorted({*EE_LOCAL_R_ALIGN_PRESETS, *EE_LOCAL_R_ALIGN_SPLIT_PRESETS})
                )
                raise ValueError(
                    f"invalid ee-local-r-align {value!r}; expected a preset ({presets}) "
                    "or 9 row-major floats"
                ) from exc
    matrix = _validate_r_align_matrix(value)
    return EeLocalRAlign(linear=matrix, angular=matrix)


def rotate_flow_arm_vectors(
    array: np.ndarray, rotation: "np.ndarray | EeLocalRAlign"
) -> np.ndarray:
    """Rotate the per-arm linear+angular 3-vectors of flow proprio/action rows.

    Works for proprio vectors (..., >=14) and action chunks (steps, 14): the
    last dim holds [left dx dy dz drx dry drz grip | right ...]; gripper and
    any trailing entries (proprio arm_mask) are untouched.

    `rotation` may be a single 3x3 matrix (applied to both the translation and
    rotation 3-vectors) or an :class:`EeLocalRAlign` carrying separate `linear`
    (translation) and `angular` (rotation) matrices.
    """
    if isinstance(rotation, EeLocalRAlign):
        linear = np.asarray(rotation.linear, dtype=np.float32)
        angular = np.asarray(rotation.angular, dtype=np.float32)
    else:
        linear = angular = np.asarray(rotation, dtype=np.float32)
    rotated = np.array(array, dtype=np.float32, copy=True)
    for offset in (0, 7):
        rotated[..., offset : offset + 3] = rotated[..., offset : offset + 3] @ linear.T
        rotated[..., offset + 3 : offset + 6] = (
            rotated[..., offset + 3 : offset + 6] @ angular.T
        )
    return rotated


def _proprio_action_frame_from_stats(stats: dict[str, Any] | None) -> str:
    if not isinstance(stats, dict):
        return DEFAULT_ACTION_FRAME
    return normalize_action_frame(stats.get("proprio_action_frame", DEFAULT_ACTION_FRAME))


def _resolve_runtime_command_family(
    command_family: str | None,
    stats: dict[str, Any],
    *,
    default_command_family: str = "tcp_target_pose",
) -> str:
    # When command_family is unset, fall back to the source-specific default:
    # tcp_target_pose for the flow / openpi rollout sources (position-control deploy
    # lane), tcp_twist_local for the DirectBC sources (their trained/streamed default).
    family = (
        normalize_flow_command_family(command_family)
        if command_family
        else normalize_flow_command_family(default_command_family)
    )
    if _proprio_action_frame_from_stats(stats) == "ee_local" and family not in {
        "tcp_twist_local",
        "tcp_target_pose",
    }:
        raise ValueError(
            "ee_local checkpoints require command-family tcp_twist_local or tcp_target_pose"
        )
    return family


def validate_flow_command_family(
    rollout_mode: str | RolloutMode,
    command_family: str,
    *,
    allow_tcp_twist_local: bool = False,
    allow_tcp_target_pose: bool = False,
    dataset_stats: dict[str, Any] | None = None,
) -> None:
    mode = parse_rollout_mode(rollout_mode)
    family = normalize_flow_command_family(command_family)
    _ = dataset_stats
    # real_readonly never sends commands, so the family is irrelevant there.
    if mode in {RolloutMode.OFFLINE_EVAL, RolloutMode.SIM_DRYRUN, RolloutMode.REAL_READONLY}:
        return
    # Live rollout of body-frame twist is an explicit operator opt-in.
    if family == "tcp_twist_local" and allow_tcp_twist_local:
        return
    if family == "tcp_target_pose" and allow_tcp_target_pose:
        return
    if family == "tcp_target_pose":
        raise RolloutModeValidationError(
            "command-family tcp_target_pose requires --allow-tcp-target-pose for "
            "controller_sim or real_policy"
        )
    raise RolloutModeValidationError(
        "command-family tcp_twist_local requires --allow-tcp-twist-local for "
        "controller_sim or real_policy"
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
    if family not in {"tcp_twist_local", "tcp_target_pose"}:
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

    The source consumes ee_local body-frame policy deltas and emits either bounded
    TcpTwistLocal velocity commands or integrated absolute TcpPoseTarget setpoints.
    Live rollout is behind rollout-mode validation and SafetyGate still decides
    whether the intent may be sent.
    """

    # Default command family when --command-family is unset. The flow / openpi
    # rollout sources deploy on position control (tcp_target_pose); the DirectBC
    # subclasses override this to their streamed tcp_twist_local default.
    _default_command_family = "tcp_target_pose"

    def __init__(
        self,
        checkpoint_path: str | Path,
        *,
        timeout_sec: float = 0.2,
        camera_client: Any | None = None,
        sample_steps: int = 16,
        command_family: str | None = None,
        policy_dt_sec: float | None = None,
        max_linear_velocity_m_s: float | None = None,
        max_angular_velocity_rad_s: float | None = None,
        max_linear_step_m: float = 0.002,
        max_angular_step_rad: float = 0.01,
        chunk_execute_steps: int | None = None,
        chunk_crossfade_steps: int = 0,
        tcp_target_pose_conditioning: str = "legacy_step_hold",
        tcp_target_pose_reanchor_mode: str = "measured_blend",
        tcp_target_pose_blend_steps: int = 2,
        tcp_target_pose_send_twist: bool = False,
        allow_rbpodo_controller_simulation_cartesian: bool = False,
        gripper_runtime: GripperRuntime | None = None,
        ee_local_r_align: Any = None,
        device: str = "auto",
        stochastic_sampling: bool = True,
        stderr: TextIO = sys.stderr,
    ):
        if sample_steps <= 0:
            raise ValueError("sample_steps must be positive")
        # Stochastic sampling integrates the flow ODE from random initial noise
        # (x_T ~ N(0, I)) so the multimodal action distribution is actually sampled.
        # Deterministic (zero init) collapses toward the mean of the modes -- wrong
        # for multimodal tasks (e.g. horizontal vs vertical grasp). Default stochastic.
        self.stochastic_sampling = bool(stochastic_sampling)
        # Patch 3: online A-stage conditioning config for streamed tcp_target_pose.
        # legacy_step_hold (default) preserves per-step ZOH; foh_se3 interpolates the
        # absolute target at every servo tick. Other state is lazy-initialised (so the
        # DirectBc subclasses, which do not forward these params, default to legacy).
        if str(tcp_target_pose_conditioning) not in CONDITIONING_MODES:
            raise ValueError(f"tcp_target_pose_conditioning must be one of {CONDITIONING_MODES}")
        if str(tcp_target_pose_reanchor_mode) not in REANCHOR_MODES:
            raise ValueError(f"tcp_target_pose_reanchor_mode must be one of {REANCHOR_MODES}")
        self._tcp_tp_mode = str(tcp_target_pose_conditioning)
        self._tcp_tp_reanchor_mode = str(tcp_target_pose_reanchor_mode)
        self._tcp_tp_blend_steps = int(tcp_target_pose_blend_steps)
        self._tcp_tp_send_twist = bool(tcp_target_pose_send_twist)
        self.timeout_sec = float(timeout_sec)
        self.camera_client = camera_client
        self.sample_steps = int(sample_steps)
        if policy_dt_sec is not None and policy_dt_sec <= 0.0:
            raise ValueError("policy_dt_sec must be positive")
        if max_linear_velocity_m_s is not None and max_linear_velocity_m_s < 0.0:
            raise ValueError("max_linear_velocity_m_s must be non-negative")
        if max_angular_velocity_rad_s is not None and max_angular_velocity_rad_s < 0.0:
            raise ValueError("max_angular_velocity_rad_s must be non-negative")
        self.max_linear_step_m = float(max_linear_step_m)
        self.max_angular_step_rad = float(max_angular_step_rad)
        self.policy_label = "flow policy"
        self.gripper_command_source = "flow_policy"
        # How to interpret the action gripper dim (already scaled to percent units
        # by the source, e.g. openpi_remote's *100). True (default) = ABSOLUTE
        # next-step opening percent -> command it directly (latest openpi
        # `--gripper-mode absolute` checkpoints). False = per-step percent DELTA
        # to integrate (legacy `(target-current)/100` checkpoints). Set from the
        # `--gripper-action-mode` CLI flag in main.py.
        self.gripper_action_absolute = True
        self.stderr = stderr
        self.device = _resolve_device(device)

        checkpoint = torch.load(checkpoint_path, map_location=self.device, weights_only=False)
        schema = str(checkpoint.get("schema", "") or "")
        if schema != FLOW_CHECKPOINT_SCHEMA:
            raise ValueError(f"unsupported flow checkpoint schema: {schema}")
        self.stats = dict(checkpoint["dataset_stats"])
        self.action_frame = _proprio_action_frame_from_stats(self.stats)
        self.ee_local_r_align = (
            resolve_ee_local_r_align(ee_local_r_align)
            if self.action_frame == "ee_local"
            else None
        )
        if self.action_frame == "ee_local" and self.ee_local_r_align is None:
            print(
                f"WARNING: {self.policy_label}: ee_local checkpoint without --ee-local-r-align; "
                "assuming the runtime TCP body frame matches the training EE frame "
                "(pika UMI data needs --ee-local-r-align pika_rz180; measured 2026-06-15)",
                file=stderr,
            )
        self.command_family_option = _resolve_runtime_command_family(
            command_family, self.stats, default_command_family=self._default_command_family)
        self.command_family = canonical_flow_command_family(self.command_family_option)
        model_config = dict(checkpoint.get("model_config", {}))
        # camera_names may live only inside model_config (flow checkpoints) rather
        # than at the top level. Falling through to model_config is REQUIRED: an
        # empty camera_names makes the source treat a vision-conditioned policy as
        # camera-free (requires_camera=False) and run it BLIND on zero images, so
        # its output is identical across scenes -> wrong, image-independent motion.
        _camera_names = checkpoint.get("camera_names") or model_config.get("camera_names") or []
        self.camera_names = [str(name) for name in _camera_names]
        self.image_size = int(checkpoint.get("image_size") or model_config.get("image_size") or 224)
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
        # Chunk-boundary twist crossfade: blend the first `chunk_crossfade_steps`
        # twists of a freshly activated chunk from the previously emitted twist
        # (alpha ramps 0->1) so the velocity is continuous across the resample
        # boundary, removing the boundary jerk without steady-state lag. 0 = off.
        self._chunk_crossfade_steps = int(chunk_crossfade_steps)
        self._steps_since_boundary = 0
        self._prev_emitted_twist_by_arm: dict[str, tuple[float, ...] | None] = {
            "left": None,
            "right": None,
        }
        self._target_pose_by_arm: dict[str, np.ndarray | None] = {"left": None, "right": None}
        self._gripper_targets_by_arm: dict[str, float | None] = {"left": None, "right": None}
        # Per-policy-step action logger (env-gated, debug only). Set
        # POLICY_RUNNER_ACTION_LOG=/path/to/actions.jsonl to capture one JSON
        # line per executed policy step: raw flow delta, converted/clamped twist
        # actually sent, chunk index and chunk-boundary marker. Used to diagnose
        # trembling (pulsed chunk boundaries vs. delta->twist noise amplification).
        self._action_log: TextIO | None = None
        self._action_log_seq = 0
        _action_log_path = default_action_log_path()
        self._action_log = open(_action_log_path, "w", buffering=1)
        print(
            f"[flow-infer] logging per-step actions to {_action_log_path}",
            file=sys.stderr,
        )

    def next_intent(self, snapshot: StateSnapshot, now_monotonic: float) -> CommandIntent | None:
        if getattr(self, "enable_async_chunking", False):
            return self._next_intent_streamed(snapshot, now_monotonic)
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
            chunk = self._sample_chunk(payload)
            if chunk is None:
                self._chunk = None
                return self._no_policy_input_intent()
            if self.ee_local_r_align is not None:
                # Policy steps are in the training EE frame (e.g. pika tip);
                # convert to the RB TCP body frame: v_tcp = R_alignT . v_tip.
                chunk = rotate_flow_arm_vectors(chunk, self.ee_local_r_align.T)
            self._chunk = chunk
            self._chunk_index = 0
            self._steps_since_boundary = 0  # restart the crossfade ramp at the boundary
        step = self._chunk[self._chunk_index]
        self._chunk_index += 1
        gripper_targets = self._integrate_gripper_targets(step, payload)
        self._dispatch_gripper_step(step)
        return self._emit_step_intent(step, payload, gripper_targets)

    # ----------------------------------------------------------- streamed --
    def _next_intent_streamed(
        self, snapshot: StateSnapshot, now_monotonic: float
    ) -> CommandIntent | None:
        """500 Hz-safe dispatch: background chunk inference + per-step hold.

        Each chunk step is a policy_dt (~30 Hz) delta. The same twist is
        re-emitted every servo tick and the step advances only after policy_dt of
        wall-clock, so the arm moves at the intended cadence; meanwhile the next
        chunk is inferred in a worker thread. The control loop never blocks on
        inference, removing the pulsed start/stop motion (vibration) of the
        synchronous path."""
        payload = snapshot.payload
        if self._reset_left_pose is None or self._reset_right_pose is None:
            self._reset_left_pose = pose_from_state_payload(payload, "left")
            self._reset_right_pose = pose_from_state_payload(payload, "right")
        self._ensure_stream_state()

        advanced = False
        if self._chunk is None:
            # Need a chunk: take a ready prefetch, else sample once inline (only
            # safe while the worker is idle, guaranteed by the _stream_pending
            # guard, so the camera client is never touched by two threads).
            chunk = self._take_prefetched()
            if chunk is None and not self._stream_pending:
                chunk = self._sample_and_align_chunk(payload)
            if chunk is None:
                self._request_prefetch(payload)
                return self._stream_hold_intent()
            self._activate_chunk(chunk, now_monotonic)
            advanced = True
        elif now_monotonic >= self._step_deadline:
            next_index = self._chunk_index + 1
            if next_index < self._current_chunk_execute_limit():
                self._chunk_index = next_index
                self._step_deadline = now_monotonic + float(self.policy_dt_sec)
                advanced = True
            else:
                swapped = self._take_prefetched()
                if swapped is not None:
                    self._activate_chunk(swapped, now_monotonic)
                    advanced = True
                else:
                    # Next chunk not ready at the boundary: hold (zero twist) and
                    # drop the stale chunk so the next tick re-acquires cleanly.
                    self._stream_stall_count += 1
                    self._chunk = None
                    self._request_prefetch(payload)
                    if self._tcp_tp_foh_active():
                        # Re-emit the last absolute target (no abrupt jump); flag stall.
                        self._current_step_intent = self._foh_tick_intent(now_monotonic, stall=True)
                        self._log_foh_action(None, self._current_step_intent, now_monotonic)
                        return self._current_step_intent
                    return self._stream_hold_intent()

        # Kick the next inference once far enough into the chunk that it should
        # finish before the executable window drains (no boundary stall).
        if (
            self._chunk is not None
            and not self._stream_pending
            and self._stream_next_chunk is None
            and self._chunk_index >= self._stream_prefetch_at()
        ):
            self._request_prefetch(payload)

        foh = self._tcp_tp_foh_active()
        if advanced and self._chunk is not None:
            step = self._chunk[self._chunk_index]
            gripper_targets = self._integrate_gripper_targets(step, payload)
            self._dispatch_gripper_step(step)
            self._current_gripper_targets = gripper_targets
            if foh:
                # On a fresh chunk activation (index 0) install its FOH knots, then emit
                # the per-tick interpolated target. Gripper stays step-based.
                if self._chunk_index == 0:
                    self._foh_begin_chunk(payload, now_monotonic)
                self._steps_since_boundary += 1
                self._current_step_intent = self._foh_tick_intent(now_monotonic, stall=False)
                self._log_foh_action(step, self._current_step_intent, now_monotonic)
            else:
                self._current_step_intent = self._emit_step_intent(step, payload, gripper_targets)
        elif foh and self._chunk is not None:
            # Between policy steps: re-interpolate the absolute target every servo tick.
            self._current_step_intent = self._foh_tick_intent(now_monotonic, stall=False)
        return self._current_step_intent

    def _emit_step_intent(
        self,
        step: np.ndarray,
        payload: dict[str, Any],
        gripper_targets: dict[str, float | None],
    ) -> CommandIntent | None:
        if self.command_family_option == "tcp_twist_local":
            intent = self._tcp_twist_local_step_intent(step, gripper_targets=gripper_targets)
        elif self.command_family_option == "tcp_target_pose":
            intent = self._tcp_target_pose_step_intent(
                step,
                payload=payload,
                gripper_targets=gripper_targets,
            )
        else:
            intent = None
        # Advance the crossfade ramp once per executed policy step (not per servo
        # tick): the streamed path re-emits the same held intent between steps.
        self._steps_since_boundary += 1
        self._log_action_step(step, intent)
        return intent

    def _log_action_step(self, step: np.ndarray, intent: CommandIntent | None) -> None:
        """Append one JSONL line per executed policy step (env-gated debug)."""
        log = self._action_log
        if log is None:
            return
        raw = np.asarray(step, dtype=np.float64).reshape(-1)
        record = {
            "seq": self._action_log_seq,
            "t_mono": time.monotonic(),
            "chunk_index": int(self._chunk_index),
            # True on the first step of a freshly sampled chunk -> lets you see
            # whether trembling lines up with chunk boundaries (pulsed resample).
            "chunk_boundary": int(self._chunk_index) <= 1,
            "command_family": self.command_family_option,
            # Raw flow output before delta->twist conversion / clamping.
            "raw_delta": raw.tolist(),
            # Actual per-arm payload sent downstream (twist after /policy_dt and
            # clamp, or None if the arm was idle / no intent emitted).
            "left": None if intent is None else intent.left,
            "right": None if intent is None else intent.right,
        }
        self._action_log_seq += 1
        log.write(json.dumps(record) + "\n")

    def _activate_chunk(self, chunk: np.ndarray, now_monotonic: float) -> None:
        self._chunk = chunk
        self._chunk_index = 0
        self._steps_since_boundary = 0  # restart the crossfade ramp at the boundary
        self._step_deadline = now_monotonic + float(self.policy_dt_sec)

    def _stream_prefetch_at(self) -> int:
        # Kick the next inference EARLY (after ~1/4 of the chunk) so it has the most wall-clock
        # to finish before the executable window drains -> fewer boundary stalls (= fewer
        # pauses in the motion). Was limit//2 (half-chunk lead); a slow medoid/remote inference
        # could not finish in that window and stalled every chunk. Trade-off: the next chunk is
        # inferred from a slightly older frame (~quarter-chunk), but it re-anchors to the current
        # pose at its boundary, so only the IMAGE is marginally staler.
        limit = self._current_chunk_execute_limit()
        return max(0, min(2, limit - 1))

    def _stream_hold_intent(self) -> CommandIntent | None:
        if self.command_family_option == "tcp_target_pose":
            # Stall (next chunk not ready): re-emit the LAST absolute TcpPoseTarget so the
            # command stays fresh and the arm holds steady at the target. Returning None
            # here let the command go stale (real timeout 0.05s) -> the servo jerks/stops,
            # which is the pulsed "뚝뚝" motion. We do NOT clear the target accumulator: the
            # next chunk re-anchors at its own boundary (_steps_since_boundary==0) anyway, so
            # holding the last target until then gives a smooth pause instead of a stale jerk.
            return getattr(self, "_current_step_intent", None)
        left = _ZERO_TWIST if (len(self.arm_mask) > 0 and self.arm_mask[0] > 0.0) else None
        right = _ZERO_TWIST if (len(self.arm_mask) > 1 and self.arm_mask[1] > 0.0) else None
        self._last_nonzero_twist_by_arm["left"] = False
        self._last_nonzero_twist_by_arm["right"] = False
        # The arm is held at zero velocity; anchor the crossfade reference at zero
        # so the next chunk ramps in from rest (smooth restart after a stall).
        self._prev_emitted_twist_by_arm["left"] = _ZERO_TWIST
        self._prev_emitted_twist_by_arm["right"] = _ZERO_TWIST
        if self.command_family_option == "tcp_twist_local":
            return tcp_twist_local_intent(left=left, right=right, timeout_sec=self.timeout_sec)
        return None

    def _sample_and_align_chunk(self, payload: dict[str, Any]) -> np.ndarray | None:
        chunk = self._sample_chunk(payload)
        if chunk is None:
            return None
        if self.ee_local_r_align is not None:
            chunk = rotate_flow_arm_vectors(chunk, self.ee_local_r_align.T)
        return chunk

    def _ensure_stream_state(self) -> None:
        if getattr(self, "_stream_inited", False):
            return
        self._stream_inited = True
        self._step_deadline = 0.0
        self._current_step_intent = None
        self._stream_next_chunk = None
        self._stream_pending = False
        self._stream_shutdown = False
        self._stream_request = None
        self._stream_stall_count = 0
        self._stream_lock = threading.Lock()
        self._stream_cv = threading.Condition(self._stream_lock)
        self._stream_thread = threading.Thread(
            target=self._stream_worker, name="flow-prefetch", daemon=True
        )
        self._stream_thread.start()

    def _stream_worker(self) -> None:
        while True:
            with self._stream_cv:
                while self._stream_request is None and not self._stream_shutdown:
                    self._stream_cv.wait()
                if self._stream_shutdown:
                    return
                payload = self._stream_request
                self._stream_request = None
            try:
                chunk = self._sample_and_align_chunk(payload)
            except Exception as exc:  # noqa: BLE001 - inference must not kill the worker
                print(
                    f"{self.policy_label} prefetch failed: {type(exc).__name__}: {exc}",
                    file=self.stderr,
                    flush=True,
                )
                chunk = None
            with self._stream_lock:
                self._stream_next_chunk = chunk
                self._stream_pending = False

    def _request_prefetch(self, payload: dict[str, Any]) -> None:
        with self._stream_cv:
            if self._stream_pending or self._stream_next_chunk is not None:
                return
            self._stream_pending = True
            self._stream_request = payload
            self._stream_cv.notify()

    def _take_prefetched(self) -> np.ndarray | None:
        with self._stream_lock:
            chunk = self._stream_next_chunk
            self._stream_next_chunk = None
        return chunk

    @property
    def gripper_command_count(self) -> int:
        return int(self.gripper_runtime.command_count)

    @property
    def gripper_dropped_count(self) -> int:
        return int(self.gripper_runtime.dropped_count)

    def _dispatch_gripper_step(self, step: np.ndarray) -> None:
        # ABSOLUTE checkpoints emit the next-step opening percent -> drive the
        # gripper backend as a "target" (set the opening directly); legacy DELTA
        # checkpoints emit a per-step change -> "delta" (accumulate on the motor).
        command_type = "target" if getattr(self, "gripper_action_absolute", True) else "delta"
        commands = gripper_commands_from_flow_step(
            step.tolist(),
            arm_mask=self.arm_mask.tolist(),
            command_type=command_type,
            source=self.gripper_command_source,
        )
        self.gripper_runtime.dispatch(commands)

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

    def _tcp_target_pose_step_intent(
        self,
        step: np.ndarray,
        *,
        payload: dict[str, Any],
        gripper_targets: dict[str, float | None],
    ) -> CommandIntent | None:
        left = (
            self._target_payload_for_arm("left", step[0:6], payload)
            if self.arm_mask[0] > 0.0
            else None
        )
        right = (
            self._target_payload_for_arm("right", step[7:13], payload)
            if self.arm_mask[1] > 0.0
            else None
        )
        if left is None and right is None:
            return None
        return tcp_pose_target_stand_intent(
            left=left,
            right=right,
            left_gripper=gripper_targets.get("left"),
            right_gripper=gripper_targets.get("right"),
            timeout_sec=self.timeout_sec,
        )

    def _target_payload_for_arm(
        self,
        arm: str,
        delta: np.ndarray,
        payload: dict[str, Any],
    ) -> tuple[float, ...]:
        targets = getattr(self, "_target_pose_by_arm", None)
        if targets is None:
            self._target_pose_by_arm = {"left": None, "right": None}
            targets = self._target_pose_by_arm
        if self._steps_since_boundary == 0 or targets.get(arm) is None:
            targets[arm] = pose_from_state_payload(payload, arm)
        clamped_delta = self._clamp_delta_step(delta)
        targets[arm] = pose_compose_local(targets[arm], np.asarray(clamped_delta, dtype=np.float32))
        return tuple(float(value) for value in targets[arm].tolist())

    def _clear_target_pose_state(self) -> None:
        targets = getattr(self, "_target_pose_by_arm", None)
        if targets is not None:
            targets["left"] = None
            targets["right"] = None

    # ----------------------------------------------- foh_se3 conditioning (Patch 3) --
    def _tcp_tp_foh_active(self) -> bool:
        """True only for the streamed tcp_target_pose path with foh_se3 selected.

        Uses getattr defaults so the DirectBc subclasses (which never set _tcp_tp_mode)
        always evaluate False -> unchanged legacy behavior."""
        return (
            self.command_family_option == "tcp_target_pose"
            and getattr(self, "_tcp_tp_mode", "legacy_step_hold") == "foh_se3"
        )

    def _ensure_tcp_tp_conditioners(self) -> dict[str, "OnlineTcpTargetPoseConditioner"]:
        conds = getattr(self, "_tcp_tp_conditioners", None)
        if conds is None:
            assert self.policy_dt_sec is not None
            conds = {
                arm: OnlineTcpTargetPoseConditioner(
                    mode="foh_se3",
                    reanchor_mode=getattr(self, "_tcp_tp_reanchor_mode", "measured_blend"),
                    policy_dt_sec=float(self.policy_dt_sec),
                    blend_steps=int(getattr(self, "_tcp_tp_blend_steps", 2)),
                )
                for arm in ("left", "right")
            }
            self._tcp_tp_conditioners = conds
        return conds

    def _foh_arm_indices(self) -> tuple[tuple[str, int, slice], ...]:
        return (("left", 0, slice(0, 6)), ("right", 1, slice(7, 13)))

    def _foh_begin_chunk(self, payload: dict[str, Any], now_monotonic: float) -> None:
        """Install the freshly activated chunk's per-step absolute targets as FOH knots."""
        conds = self._ensure_tcp_tp_conditioners()
        limit = self._current_chunk_execute_limit()
        chunk = self._chunk
        if chunk is None or limit <= 0:
            return
        self._tcp_tp_chunk_seq = int(getattr(self, "_tcp_tp_chunk_seq", 0)) + 1
        for arm, idx, sl in self._foh_arm_indices():
            if self.arm_mask[idx] <= 0.0:
                continue
            cond = conds[arm]
            measured_anchor = np.asarray(pose_from_state_payload(payload, arm), dtype=np.float64)
            last_emitted = cond.last_emitted
            measured: list[np.ndarray] = []
            continuous: list[np.ndarray] | None = [] if last_emitted is not None else None
            cur_m = measured_anchor
            cur_c = None if last_emitted is None else np.asarray(last_emitted, dtype=np.float64)
            for i in range(limit):
                delta = np.asarray(self._clamp_delta_step(chunk[i][sl]), dtype=np.float64)
                cur_m = np.asarray(pose_compose_local(cur_m, delta), dtype=np.float64)
                measured.append(cur_m.copy())
                if continuous is not None:
                    cur_c = np.asarray(pose_compose_local(cur_c, delta), dtype=np.float64)
                    continuous.append(cur_c.copy())
            cond.begin_chunk(
                measured_anchor=measured_anchor,
                measured_targets=np.asarray(measured, dtype=np.float64),
                continuous_targets=(np.asarray(continuous, dtype=np.float64) if continuous else None),
                t_wall_start=float(now_monotonic),
                chunk_id=self._tcp_tp_chunk_seq,
            )

    def _foh_tick_intent(self, now_monotonic: float, *, stall: bool) -> CommandIntent | None:
        """Emit a fresh SE(3)-interpolated TcpPoseTarget for the current servo tick."""
        conds = self._ensure_tcp_tp_conditioners()
        gripper = getattr(self, "_current_gripper_targets", {"left": None, "right": None})
        send_twist = bool(getattr(self, "_tcp_tp_send_twist", False))
        results: dict[str, Any] = {}
        payload_arms: dict[str, tuple[float, ...] | None] = {"left": None, "right": None}
        twist_arms: dict[str, tuple[float, ...] | None] = {"left": None, "right": None}
        for arm, idx, _sl in self._foh_arm_indices():
            if self.arm_mask[idx] <= 0.0:
                continue
            cond = conds[arm]
            if cond.last_emitted is None and stall:
                continue  # no chunk has produced a target yet -> emit nothing (no jump to origin)
            ct = cond.hold() if stall else cond.sample(now_monotonic)
            results[arm] = ct
            payload_arms[arm] = tuple(float(v) for v in ct.pose.tolist())
            if send_twist and ct.twist is not None and np.all(np.isfinite(ct.twist)):
                twist_arms[arm] = tuple(float(v) for v in np.asarray(ct.twist).tolist())
        self._foh_last_targets = results
        if payload_arms["left"] is None and payload_arms["right"] is None:
            return None
        return tcp_pose_target_stand_intent(
            left=payload_arms["left"],
            right=payload_arms["right"],
            left_gripper=gripper.get("left"),
            right_gripper=gripper.get("right"),
            left_twist=twist_arms["left"],
            right_twist=twist_arms["right"],
            timeout_sec=self.timeout_sec,
        )

    def _log_foh_action(
        self,
        step: np.ndarray | None,
        intent: CommandIntent | None,
        now_monotonic: float,
    ) -> None:
        """One JSONL line per executed policy step for the foh_se3 path (Patch 3)."""
        log = self._action_log
        if log is None:
            return
        raw = None if step is None else np.asarray(step, dtype=np.float64).reshape(-1).tolist()
        record: dict[str, Any] = {
            "seq": self._action_log_seq,
            "t_mono": time.monotonic(),
            "t_wall": float(now_monotonic),
            "command_family": self.command_family_option,
            "tcp_target_pose_conditioning": getattr(self, "_tcp_tp_mode", "legacy_step_hold"),
            "reanchor_mode": getattr(self, "_tcp_tp_reanchor_mode", "measured_blend"),
            "chunk_index": int(self._chunk_index),
            "raw_delta": raw,
            "left": None if intent is None else intent.left,
            "right": None if intent is None else intent.right,
        }
        last = getattr(self, "_foh_last_targets", {}) or {}
        for arm, _idx, sl in self._foh_arm_indices():
            arm_rec: dict[str, Any] = {}
            if step is not None:
                arm_rec["clamped_delta"] = list(self._clamp_delta_step(np.asarray(step)[sl]))
            ct = last.get(arm)
            if ct is not None:
                arm_rec.update({
                    "t_policy_source": float(now_monotonic),
                    "chunk_id": int(ct.chunk_id),
                    "chunk_index_lo": int(ct.chunk_index_lo),
                    "chunk_index_hi": int(ct.chunk_index_hi),
                    "interpolation_alpha": float(ct.interpolation_alpha),
                    "reanchor": bool(ct.reanchor),
                    "hold": bool(ct.hold),
                    "stall": bool(ct.stall),
                    "dropout": bool(ct.dropout),
                    "emitted_target": [float(v) for v in ct.pose.tolist()],
                    "emitted_delta_from_prev": [float(v) for v in ct.emitted_delta_from_prev.tolist()],
                    "conditioned_twist": (None if ct.twist is None else [float(v) for v in ct.twist.tolist()]),
                })
            record[f"{arm}_conditioner"] = arm_rec
        self._action_log_seq += 1
        log.write(json.dumps(record) + "\n")

    def _clamp_delta_step(self, delta: np.ndarray) -> tuple[float, ...]:
        assert self.policy_dt_sec is not None
        return clamp_tcp_delta(
            np.asarray(delta, dtype=np.float64).reshape(-1).tolist(),
            self.max_linear_velocity_m_s * self.policy_dt_sec,
            self.max_angular_velocity_rad_s * self.policy_dt_sec,
        )

    def _twist_payload_for_arm(self, arm: str, delta: np.ndarray) -> tuple[float, ...] | None:
        twist = self._delta_to_twist(delta)
        if _has_nonzero(twist):
            twist = self._apply_chunk_crossfade(arm, twist)
            self._last_nonzero_twist_by_arm[arm] = True
            self._prev_emitted_twist_by_arm[arm] = twist
            return twist
        if self._last_nonzero_twist_by_arm[arm]:
            self._last_nonzero_twist_by_arm[arm] = False
            self._prev_emitted_twist_by_arm[arm] = _ZERO_TWIST
            return _ZERO_TWIST
        return None

    def _apply_chunk_crossfade(
        self, arm: str, twist: tuple[float, ...]
    ) -> tuple[float, ...]:
        """Blend the first few twists after a chunk boundary from the previously
        emitted twist so velocity is continuous across the resample. alpha ramps
        from 1/(K+1) at the boundary step to 1.0 after K steps; outside the window
        (or with no prior twist) the new twist passes through unchanged."""
        k = self._chunk_crossfade_steps
        if k <= 0 or self._steps_since_boundary >= k:
            return twist
        prev = self._prev_emitted_twist_by_arm.get(arm)
        if prev is None:
            return twist
        alpha = (self._steps_since_boundary + 1) / (k + 1)
        return tuple((1.0 - alpha) * p + alpha * t for p, t in zip(prev, twist))

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
        if self.command_family_option == "tcp_target_pose":
            self._clear_target_pose_state()
            return None
        if self.command_family_option != "tcp_twist_local":
            return None
        left = _ZERO_TWIST if self._last_nonzero_twist_by_arm["left"] else None
        right = _ZERO_TWIST if self._last_nonzero_twist_by_arm["right"] else None
        if left is None and right is None:
            return None
        self._last_nonzero_twist_by_arm["left"] = False
        self._last_nonzero_twist_by_arm["right"] = False
        return tcp_twist_local_intent(left=left, right=right, timeout_sec=self.timeout_sec)

    def _integrate_gripper_targets(
        self,
        step: np.ndarray,
        payload: dict[str, Any],
    ) -> dict[str, float | None]:
        targets: dict[str, float | None] = {"left": None, "right": None}
        if not self._allow_gripper_targets_in_motion_packet():
            return targets
        # Optionally hold the gripper fully OPEN for the first N executed policy steps so the
        # arm can reach toward the object before the policy is allowed to close it (avoids a
        # premature grasp at rollout start). Set via `source.gripper_open_hold_steps`.
        open_hold_steps = int(getattr(self, "gripper_open_hold_steps", 0) or 0)
        step_idx = int(getattr(self, "_gripper_integrate_count", 0))
        self._gripper_integrate_count = step_idx + 1
        hold_open = step_idx < open_hold_steps
        for arm, mask_index, step_index in (("left", 0, 6), ("right", 1, 13)):
            if len(self.arm_mask) <= mask_index or self.arm_mask[mask_index] <= 0.0:
                continue
            current = _gripper_value_from_payload(payload, arm)
            if current is None:
                current = self._live_gripper_percent(arm)
            if self._gripper_targets_by_arm[arm] is None:
                self._gripper_targets_by_arm[arm] = 0.0 if current is None else current
            if step.shape[0] > step_index:
                if getattr(self, "gripper_action_absolute", True):
                    # ABSOLUTE: the action dim IS the next-step opening percent;
                    # command it directly (clamped), never integrate.
                    self._gripper_targets_by_arm[arm] = float(
                        np.clip(float(step[step_index]), 0.0, 100.0)
                    )
                else:
                    # DELTA (legacy): accumulate the per-step opening change.
                    self._gripper_targets_by_arm[arm] = float(
                        self._gripper_targets_by_arm[arm]
                    ) + float(step[step_index])
            if hold_open:
                # Pin to fully open AND reset the integrator there, so when the hold ends the
                # policy resumes closing from the open state (not from a drifted accumulator).
                self._gripper_targets_by_arm[arm] = 100.0
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
        self._stream_shutdown = True
        cv = getattr(self, "_stream_cv", None)
        if cv is not None:
            with cv:
                cv.notify_all()
        thread = getattr(self, "_stream_thread", None)
        if thread is not None:
            thread.join(timeout=2.0)
        close = getattr(self.camera_client, "close", None)
        if callable(close):
            close()

    def _live_gripper_percent(self, arm: str) -> float | None:
        """Best-known gripper percent for proprio: physical motor first, then
        the integrated policy target. The servo state carries no gripper, so
        without this the proprio gripper channel reads 0 (= fully closed) and
        the policy behaves as if the grasp already happened."""
        backend = getattr(self.gripper_runtime, "backend", None)
        reader = getattr(backend, "current_percent", None)
        if callable(reader):
            try:
                value = reader(arm)
            except Exception:
                value = None
            if value is not None:
                return float(value)
        integrated = self._gripper_targets_by_arm.get(arm)
        return None if integrated is None else float(integrated)

    def _runtime_proprio(self, payload: dict[str, Any]) -> np.ndarray:
        assert self._reset_left_pose is not None
        assert self._reset_right_pose is not None
        proprio = runtime_proprio_from_state(
            payload,
            reset_left_pose=self._reset_left_pose,
            reset_right_pose=self._reset_right_pose,
            arm_mask=self.arm_mask,
            action_frame=self.action_frame,
        )
        for arm, index in (("left", 6), ("right", 13)):
            if proprio.shape[0] > index and _gripper_value_from_payload(payload, arm) is None:
                value = self._live_gripper_percent(arm)
                if value is not None:
                    proprio[index] = float(value)
        if self.ee_local_r_align is not None:
            # Runtime body deltas are in the RB TCP frame; the checkpoint was
            # trained in the EE (pika tip) frame: v_tip = R_align . v_tcp.
            proprio = rotate_flow_arm_vectors(proprio, self.ee_local_r_align)
        return proprio

    def _initial_noise(self, batch_size: int) -> "torch.Tensor | None":
        """Random x_T ~ N(0, I) for stochastic flow sampling, or None (zero init)."""
        if not self.stochastic_sampling:
            return None
        return torch.randn(
            int(batch_size),
            int(self.model.config.action_horizon),
            int(self.model.config.action_dim),
            dtype=torch.float32,
            device=self.device,
        )

    def _sample_chunk(self, payload: dict[str, Any]) -> np.ndarray | None:
        assert self._reset_left_pose is not None
        assert self._reset_right_pose is not None
        proprio = self._runtime_proprio(payload)
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
                initial_noise=self._initial_noise(1),
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
            frame = resolve_frame(bundle_frames, camera_name)
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
                        # Mirror the training preprocessing exactly (stats carry
                        # the dataset's crop mode; default 'none').
                        image_crop=str(self.stats.get("image_crop", "none") or "none"),
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
                f"WARNING: {self.policy_label} checkpoint expects cameras, but camera.enable is false; "
                "the action source will emit no motion intents until required frames are available.",
                file=self.stderr,
            )
        return bundle

    def _count_missing_camera_frames(self, bundle: Any | None) -> int:
        missing_count = 0
        bundle_frames = getattr(bundle, "frames", {}) if bundle is not None else {}
        for camera_name in self.camera_names:
            frame = resolve_frame(bundle_frames, camera_name)
            pixels = getattr(frame, "pixels", None)
            if pixels is None:
                missing_count += 1
        return missing_count


class DirectBcImageActionSource(FlowMatchingActionSource):
    """Runtime source for supervised image action-chunk imitation checkpoints."""

    _default_command_family = "tcp_twist_local"

    def __init__(
        self,
        checkpoint_path: str | Path,
        *,
        timeout_sec: float = 0.2,
        camera_client: Any | None = None,
        command_family: str | None = None,
        policy_dt_sec: float | None = None,
        image_size: int | None = None,
        max_linear_velocity_m_s: float | None = None,
        max_angular_velocity_rad_s: float | None = None,
        max_linear_step_m: float = 0.002,
        max_angular_step_rad: float = 0.01,
        chunk_execute_steps: int | None = None,
        chunk_crossfade_steps: int = 0,
        allow_rbpodo_controller_simulation_cartesian: bool = False,
        gripper_runtime: GripperRuntime | None = None,
        ee_local_r_align: Any = None,
        device: str = "auto",
        stderr: TextIO = sys.stderr,
    ):
        self.timeout_sec = float(timeout_sec)
        self.camera_client = camera_client
        self.sample_steps = 1
        if policy_dt_sec is not None and policy_dt_sec <= 0.0:
            raise ValueError("policy_dt_sec must be positive")
        if image_size is not None and int(image_size) <= 0:
            raise ValueError("image_size must be positive")
        if max_linear_velocity_m_s is not None and max_linear_velocity_m_s < 0.0:
            raise ValueError("max_linear_velocity_m_s must be non-negative")
        if max_angular_velocity_rad_s is not None and max_angular_velocity_rad_s < 0.0:
            raise ValueError("max_angular_velocity_rad_s must be non-negative")
        self.max_linear_step_m = float(max_linear_step_m)
        self.max_angular_step_rad = float(max_angular_step_rad)
        self.policy_label = "direct BC image policy"
        self.gripper_command_source = "direct_bc_policy"
        self.stderr = stderr
        self.device = _resolve_device(device)

        checkpoint = torch.load(checkpoint_path, map_location=self.device, weights_only=False)
        schema = str(checkpoint.get("schema", "") or "")
        if schema != IMITATION_CHECKPOINT_SCHEMA:
            raise ValueError(f"unsupported imitation checkpoint schema: {schema}")
        family = str(checkpoint.get("model_family", "") or "")
        if family not in DIRECT_BC_RUNTIME_FAMILIES:
            choices = ", ".join(sorted(DIRECT_BC_RUNTIME_FAMILIES))
            raise ValueError(f"unsupported imitation checkpoint family {family!r}; expected one of: {choices}")
        action_dim = int(checkpoint.get("action_dim", 0) or 0)
        proprio_dim = int(checkpoint.get("proprio_dim", 0) or 0)
        if action_dim != FLOW_ACTION_DIM:
            raise ValueError(f"unsupported imitation action_dim {action_dim}; expected {FLOW_ACTION_DIM}")
        if proprio_dim != FLOW_PROPRIO_DIM:
            raise ValueError(f"unsupported imitation proprio_dim {proprio_dim}; expected {FLOW_PROPRIO_DIM}")

        self.stats = dict(checkpoint["dataset_stats"])
        self.action_frame = _proprio_action_frame_from_stats(self.stats)
        self.ee_local_r_align = (
            resolve_ee_local_r_align(ee_local_r_align)
            if self.action_frame == "ee_local"
            else None
        )
        if self.action_frame == "ee_local" and self.ee_local_r_align is None:
            print(
                f"WARNING: {self.policy_label}: ee_local checkpoint without --ee-local-r-align; "
                "assuming the runtime TCP body frame matches the training EE frame "
                "(pika UMI data needs --ee-local-r-align pika_rz180; measured 2026-06-15)",
                file=stderr,
            )
        self.command_family_option = _resolve_runtime_command_family(
            command_family, self.stats, default_command_family=self._default_command_family)
        self.command_family = canonical_flow_command_family(self.command_family_option)
        self.camera_names = [str(name) for name in checkpoint.get("camera_names", [])]
        checkpoint_image_size = _positive_int(checkpoint.get("image_size"))
        stats_image_size = _positive_int(self.stats.get("image_size"))
        resolved_image_size = int(image_size or checkpoint_image_size or stats_image_size or 0)
        if resolved_image_size <= 0:
            raise ValueError(
                "direct BC image checkpoint is missing image_size; pass --image-size matching training"
            )
        self.image_size = resolved_image_size
        self.action_horizon = int(checkpoint.get("action_horizon", 0) or 0)
        if self.action_horizon <= 0:
            raise ValueError("imitation checkpoint action_horizon must be positive")
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

        hidden_dim = _infer_direct_bc_hidden_dim(checkpoint["model_state"], family)
        backbone = str(checkpoint.get("backbone", "") or "")
        if not backbone:
            raise ValueError("imitation checkpoint is missing backbone")
        if family == "arm_structured_direct":
            from .imitation_experiments import _build_structured_direct_policy

            self.model = _build_structured_direct_policy(
                backbone=backbone,
                action_horizon=self.action_horizon,
                camera_count=len(self.camera_names),
                hidden_dim=hidden_dim,
            ).to(self.device)
        else:
            from .imitation_experiments import _build_direct_bc_policy

            self.model = _build_direct_bc_policy(
                backbone=backbone,
                action_horizon=self.action_horizon,
                camera_count=len(self.camera_names),
                hidden_dim=hidden_dim,
            ).to(self.device)
        self.model.load_state_dict(checkpoint["model_state"])
        self.model.eval()

        self.arm_mask = _arm_mask_from_stats(self.stats)
        self.checkpoint_arm_mask = tuple(float(value) for value in self.arm_mask.tolist())
        self.checkpoint_selected_arms = _arms_from_mask(self.arm_mask)
        self.checkpoint_has_nonzero_gripper_commands = _checkpoint_has_nonzero_gripper_commands(
            self.stats,
            action_dim,
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
        # Chunk-boundary twist crossfade: blend the first `chunk_crossfade_steps`
        # twists of a freshly activated chunk from the previously emitted twist
        # (alpha ramps 0->1) so the velocity is continuous across the resample
        # boundary, removing the boundary jerk without steady-state lag. 0 = off.
        self._chunk_crossfade_steps = int(chunk_crossfade_steps)
        self._steps_since_boundary = 0
        self._prev_emitted_twist_by_arm: dict[str, tuple[float, ...] | None] = {
            "left": None,
            "right": None,
        }
        self._target_pose_by_arm: dict[str, np.ndarray | None] = {"left": None, "right": None}
        self._gripper_targets_by_arm: dict[str, float | None] = {"left": None, "right": None}
        # Per-policy-step action logger (env-gated, debug only). Set
        # POLICY_RUNNER_ACTION_LOG=/path/to/actions.jsonl to capture one JSON
        # line per executed policy step: raw flow delta, converted/clamped twist
        # actually sent, chunk index and chunk-boundary marker. Used to diagnose
        # trembling (pulsed chunk boundaries vs. delta->twist noise amplification).
        self._action_log: TextIO | None = None
        self._action_log_seq = 0
        _action_log_path = default_action_log_path()
        self._action_log = open(_action_log_path, "w", buffering=1)
        print(
            f"[flow-infer] logging per-step actions to {_action_log_path}",
            file=sys.stderr,
        )

    def _sample_chunk(self, payload: dict[str, Any]) -> np.ndarray | None:
        assert self._reset_left_pose is not None
        assert self._reset_right_pose is not None
        proprio = self._runtime_proprio(payload)
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
            chunk = self.model(
                torch.as_tensor(images[None, ...], dtype=torch.float32, device=self.device),
                torch.as_tensor(proprio[None, ...], dtype=torch.float32, device=self.device),
            )
            chunk = _denormalize_action_numpy(chunk, self.stats)
        return chunk[0]


class DirectBcCheckpointEnsembleActionSource(FlowMatchingActionSource):
    """Runtime source for prediction-averaged direct-BC checkpoint ensembles."""

    _default_command_family = "tcp_twist_local"

    def __init__(
        self,
        report_path: str | Path,
        *,
        ensemble_name: str | None = "top5",
        timeout_sec: float = 0.2,
        camera_client: Any | None = None,
        command_family: str | None = None,
        policy_dt_sec: float | None = None,
        image_size: int | None = None,
        max_linear_velocity_m_s: float | None = None,
        max_angular_velocity_rad_s: float | None = None,
        max_linear_step_m: float = 0.002,
        max_angular_step_rad: float = 0.01,
        chunk_execute_steps: int | None = None,
        chunk_crossfade_steps: int = 0,
        allow_rbpodo_controller_simulation_cartesian: bool = False,
        gripper_runtime: GripperRuntime | None = None,
        ee_local_r_align: Any = None,
        device: str = "auto",
        stderr: TextIO = sys.stderr,
    ):
        self.timeout_sec = float(timeout_sec)
        self.camera_client = camera_client
        self.sample_steps = 1
        if policy_dt_sec is not None and policy_dt_sec <= 0.0:
            raise ValueError("policy_dt_sec must be positive")
        if image_size is not None and int(image_size) <= 0:
            raise ValueError("image_size must be positive")
        if max_linear_velocity_m_s is not None and max_linear_velocity_m_s < 0.0:
            raise ValueError("max_linear_velocity_m_s must be non-negative")
        if max_angular_velocity_rad_s is not None and max_angular_velocity_rad_s < 0.0:
            raise ValueError("max_angular_velocity_rad_s must be non-negative")
        self.max_linear_step_m = float(max_linear_step_m)
        self.max_angular_step_rad = float(max_angular_step_rad)
        self.policy_label = "direct BC checkpoint ensemble"
        self.gripper_command_source = "direct_bc_ensemble_policy"
        self.stderr = stderr
        self.device = _resolve_device(device)

        bundle = _load_direct_bc_ensemble_bundle(
            report_path,
            device=self.device,
            image_size=image_size,
            ensemble_name=ensemble_name,
        )
        self.ensemble_name = bundle.name
        self.member_checkpoint_paths = list(bundle.member_paths)
        self.models = list(bundle.models)
        self.stats = dict(bundle.stats)
        self.action_frame = _proprio_action_frame_from_stats(self.stats)
        self.ee_local_r_align = (
            resolve_ee_local_r_align(ee_local_r_align)
            if self.action_frame == "ee_local"
            else None
        )
        if self.action_frame == "ee_local" and self.ee_local_r_align is None:
            print(
                f"WARNING: {self.policy_label}: ee_local checkpoint without --ee-local-r-align; "
                "assuming the runtime TCP body frame matches the training EE frame "
                "(pika UMI data needs --ee-local-r-align pika_rz180; measured 2026-06-15)",
                file=stderr,
            )
        self.command_family_option = _resolve_runtime_command_family(
            command_family, self.stats, default_command_family=self._default_command_family)
        self.command_family = canonical_flow_command_family(self.command_family_option)
        self.camera_names = list(bundle.camera_names)
        self.image_size = int(bundle.image_size)
        self.action_horizon = int(bundle.action_horizon)
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

        self.arm_mask = bundle.arm_mask
        self.checkpoint_arm_mask = tuple(float(value) for value in self.arm_mask.tolist())
        self.checkpoint_selected_arms = _arms_from_mask(self.arm_mask)
        self.checkpoint_has_nonzero_gripper_commands = _checkpoint_has_nonzero_gripper_commands(
            self.stats,
            FLOW_ACTION_DIM,
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
        # Chunk-boundary twist crossfade: blend the first `chunk_crossfade_steps`
        # twists of a freshly activated chunk from the previously emitted twist
        # (alpha ramps 0->1) so the velocity is continuous across the resample
        # boundary, removing the boundary jerk without steady-state lag. 0 = off.
        self._chunk_crossfade_steps = int(chunk_crossfade_steps)
        self._steps_since_boundary = 0
        self._prev_emitted_twist_by_arm: dict[str, tuple[float, ...] | None] = {
            "left": None,
            "right": None,
        }
        self._target_pose_by_arm: dict[str, np.ndarray | None] = {"left": None, "right": None}
        self._gripper_targets_by_arm: dict[str, float | None] = {"left": None, "right": None}
        # Per-policy-step action logger (env-gated, debug only). Set
        # POLICY_RUNNER_ACTION_LOG=/path/to/actions.jsonl to capture one JSON
        # line per executed policy step: raw flow delta, converted/clamped twist
        # actually sent, chunk index and chunk-boundary marker. Used to diagnose
        # trembling (pulsed chunk boundaries vs. delta->twist noise amplification).
        self._action_log: TextIO | None = None
        self._action_log_seq = 0
        _action_log_path = default_action_log_path()
        self._action_log = open(_action_log_path, "w", buffering=1)
        print(
            f"[flow-infer] logging per-step actions to {_action_log_path}",
            file=sys.stderr,
        )

    def _sample_chunk(self, payload: dict[str, Any]) -> np.ndarray | None:
        assert self._reset_left_pose is not None
        assert self._reset_right_pose is not None
        proprio = self._runtime_proprio(payload)
        proprio = normalize_runtime_proprio(proprio, self.stats)
        images, decode_count, missing_count = self._runtime_images()
        self.last_image_decode_count = decode_count
        self.last_missing_camera_count = missing_count
        self.image_decode_count += decode_count
        self.missing_camera_count += missing_count
        if self.camera_names and missing_count > 0:
            return None
        images = _normalize_images(images, self.stats)
        image_tensor = torch.as_tensor(images[None, ...], dtype=torch.float32, device=self.device)
        proprio_tensor = torch.as_tensor(proprio[None, ...], dtype=torch.float32, device=self.device)
        with torch.no_grad():
            predictions = [model(image_tensor, proprio_tensor) for model in self.models]
            chunk = torch.stack(predictions, dim=0).mean(dim=0)
            chunk = _denormalize_action_numpy(chunk, self.stats)
        return chunk[0]


def run_flow_offline_eval(
    *,
    checkpoint_path: str | Path,
    episodes_dir: str | Path,
    sample_steps: int = 16,
    device: str = "auto",
    max_samples: int = 1,
    command_family: str = "TcpTwistLocal",
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
    action_frame = _proprio_action_frame_from_stats(stats)
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
    arm_mask = _arm_mask_from_stats(stats)
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
        action_frame=action_frame,
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
        selected_arms=_arms_from_mask(arm_mask),
        checkpoint_arm_mask=tuple(float(value) for value in arm_mask.tolist()),
        checkpoint_has_nonzero_gripper_commands=_checkpoint_has_nonzero_gripper_commands(
            stats,
            int(model_config.get("action_dim", checkpoint.get("action_dim", 0)) or 0),
        ),
    )


def run_direct_bc_offline_eval(
    *,
    checkpoint_path: str | Path,
    episodes_dir: str | Path,
    device: str = "auto",
    max_samples: int = 1,
    command_family: str = "TcpTwistLocal",
    image_size: int | None = None,
) -> FlowOfflineEvalResult:
    if max_samples <= 0:
        raise ValueError("max_offline_samples must be positive")
    torch_device = _resolve_device(device)
    checkpoint = torch.load(checkpoint_path, map_location=torch_device, weights_only=False)
    schema = str(checkpoint.get("schema", "") or "")
    if schema != IMITATION_CHECKPOINT_SCHEMA:
        raise ValueError(f"unsupported imitation checkpoint schema: {schema}")
    family = str(checkpoint.get("model_family", "") or "")
    if family not in DIRECT_BC_RUNTIME_FAMILIES:
        raise ValueError(f"unsupported imitation checkpoint family: {family}")

    stats = dict(checkpoint["dataset_stats"])
    action_frame = _proprio_action_frame_from_stats(stats)
    camera_names = [str(name) for name in checkpoint.get("camera_names", [])]
    arm_mask = _arm_mask_from_stats(stats)
    resolved_image_size = int(image_size or _positive_int(checkpoint.get("image_size")) or _positive_int(stats.get("image_size")) or 0)
    if resolved_image_size <= 0:
        raise ValueError("direct BC image checkpoint is missing image_size; pass --image-size matching training")
    hidden_dim = _infer_direct_bc_hidden_dim(checkpoint["model_state"], family)
    backbone = str(checkpoint.get("backbone", "") or "")
    if family == "arm_structured_direct":
        from .imitation_experiments import _build_structured_direct_policy

        model = _build_structured_direct_policy(
            backbone=backbone,
            action_horizon=int(checkpoint["action_horizon"]),
            camera_count=len(camera_names),
            hidden_dim=hidden_dim,
        ).to(torch_device)
    else:
        from .imitation_experiments import _build_direct_bc_policy

        model = _build_direct_bc_policy(
            backbone=backbone,
            action_horizon=int(checkpoint["action_horizon"]),
            camera_count=len(camera_names),
            hidden_dim=hidden_dim,
        ).to(torch_device)
    model.load_state_dict(checkpoint["model_state"])
    model.eval()

    dataset = FlowHdf5Dataset(
        episodes_dir,
        action_horizon=int(checkpoint["action_horizon"]),
        image_size=resolved_image_size,
        camera_names=camera_names,
        stats=stats,
        normalize=True,
        action_frame=action_frame,
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
            chunk = model(
                torch.as_tensor(sample["images"][None, ...], dtype=torch.float32, device=torch_device),
                torch.as_tensor(sample["proprio"][None, ...], dtype=torch.float32, device=torch_device),
            )
        action_chunk_count += int(chunk.shape[1])

    return FlowOfflineEvalResult(
        sample_count=sample_count,
        action_chunk_count=action_chunk_count,
        camera_names=camera_names,
        image_decode_count=image_decode_count,
        missing_camera_count=missing_camera_count,
        command_family=command_family,
        selected_arms=_arms_from_mask(arm_mask),
        checkpoint_arm_mask=tuple(float(value) for value in arm_mask.tolist()),
        checkpoint_has_nonzero_gripper_commands=_checkpoint_has_nonzero_gripper_commands(
            stats,
            int(checkpoint.get("action_dim", 0) or 0),
        ),
    )


def run_direct_bc_ensemble_offline_eval(
    *,
    report_path: str | Path,
    episodes_dir: str | Path,
    device: str = "auto",
    max_samples: int = 1,
    command_family: str = "TcpTwistLocal",
    image_size: int | None = None,
    ensemble_name: str | None = "top5",
) -> FlowOfflineEvalResult:
    if max_samples <= 0:
        raise ValueError("max_offline_samples must be positive")
    torch_device = _resolve_device(device)
    bundle = _load_direct_bc_ensemble_bundle(
        report_path,
        device=torch_device,
        image_size=image_size,
        ensemble_name=ensemble_name,
    )
    dataset = FlowHdf5Dataset(
        episodes_dir,
        action_horizon=int(bundle.action_horizon),
        image_size=int(bundle.image_size),
        camera_names=list(bundle.camera_names),
        stats=bundle.stats,
        normalize=True,
        action_frame=_proprio_action_frame_from_stats(bundle.stats),
    )
    sample_count = min(int(max_samples), len(dataset))
    image_decode_count = 0
    missing_camera_count = 0
    action_chunk_count = 0
    for index in range(sample_count):
        sample = dataset[index]
        image_decode_count += int(sample["image_decode_count"])
        missing_camera_count += int(sample["missing_camera_count"])
        images = torch.as_tensor(sample["images"][None, ...], dtype=torch.float32, device=torch_device)
        proprio = torch.as_tensor(sample["proprio"][None, ...], dtype=torch.float32, device=torch_device)
        with torch.no_grad():
            predictions = [model(images, proprio) for model in bundle.models]
            chunk = torch.stack(predictions, dim=0).mean(dim=0)
        action_chunk_count += int(chunk.shape[1])

    return FlowOfflineEvalResult(
        sample_count=sample_count,
        action_chunk_count=action_chunk_count,
        camera_names=list(bundle.camera_names),
        image_decode_count=image_decode_count,
        missing_camera_count=missing_camera_count,
        command_family=command_family,
        selected_arms=_arms_from_mask(bundle.arm_mask),
        checkpoint_arm_mask=tuple(float(value) for value in bundle.arm_mask.tolist()),
        checkpoint_has_nonzero_gripper_commands=_checkpoint_has_nonzero_gripper_commands(
            bundle.stats,
            FLOW_ACTION_DIM,
        ),
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


def action_chunk_checkpoint_kind(
    checkpoint_path: str | Path,
    *,
    device: str = "cpu",
) -> str:
    path = Path(checkpoint_path)
    if path.suffix.lower() == ".json":
        payload = _load_json(path)
        schema = str(payload.get("schema", "") or "")
        if schema == IMITATION_ENSEMBLE_REPORT_SCHEMA:
            return "direct_bc_ensemble"
        raise ValueError(f"unsupported action-chunk JSON schema: {schema}")
    checkpoint = torch.load(checkpoint_path, map_location=_resolve_device(device), weights_only=False)
    schema = str(checkpoint.get("schema", "") or "")
    if schema == FLOW_CHECKPOINT_SCHEMA:
        return "flow"
    if schema == IMITATION_CHECKPOINT_SCHEMA:
        family = str(checkpoint.get("model_family", "") or "")
        if family in DIRECT_BC_RUNTIME_FAMILIES:
            return "direct_bc"
        raise ValueError(f"unsupported imitation checkpoint family: {family}")
    raise ValueError(f"unsupported action-chunk checkpoint schema: {schema}")


def load_action_chunk_checkpoint_dataset_stats(
    checkpoint_path: str | Path,
    *,
    device: str = "cpu",
    ensemble_name: str | None = "top5",
) -> dict[str, Any]:
    path = Path(checkpoint_path)
    if path.suffix.lower() == ".json":
        payload = _load_json(path)
        schema = str(payload.get("schema", "") or "")
        if schema != IMITATION_ENSEMBLE_REPORT_SCHEMA:
            raise ValueError(f"unsupported action-chunk JSON schema: {schema}")
        member_path = _first_ensemble_member_path(path, payload, ensemble_name=ensemble_name)
        checkpoint = torch.load(member_path, map_location=_resolve_device(device), weights_only=False)
        stats = checkpoint.get("dataset_stats", {})
        return dict(stats) if isinstance(stats, dict) else {}
    checkpoint = torch.load(checkpoint_path, map_location=_resolve_device(device), weights_only=False)
    schema = str(checkpoint.get("schema", "") or "")
    if schema not in {FLOW_CHECKPOINT_SCHEMA, IMITATION_CHECKPOINT_SCHEMA}:
        raise ValueError(f"unsupported action-chunk checkpoint schema: {schema}")
    if schema == IMITATION_CHECKPOINT_SCHEMA:
        family = str(checkpoint.get("model_family", "") or "")
        if family not in DIRECT_BC_RUNTIME_FAMILIES:
            raise ValueError(f"unsupported imitation checkpoint family: {family}")
    stats = checkpoint.get("dataset_stats", {})
    return dict(stats) if isinstance(stats, dict) else {}


def _load_json(path: str | Path) -> dict[str, Any]:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object in {path}")
    return payload


def _sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _first_ensemble_member_path(
    report_path: Path,
    payload: dict[str, Any],
    *,
    ensemble_name: str | None,
) -> Path:
    ensemble = _select_ensemble(payload, ensemble_name=ensemble_name)
    members = ensemble.get("members", [])
    if not isinstance(members, list) or not members:
        raise ValueError("ensemble report selected entry has no members")
    return _resolve_report_member_path(report_path, members[0])


def _load_direct_bc_ensemble_bundle(
    report_path: str | Path,
    *,
    device: torch.device,
    image_size: int | None,
    ensemble_name: str | None,
) -> _DirectBcEnsembleBundle:
    report = Path(report_path)
    payload = _load_json(report)
    schema = str(payload.get("schema", "") or "")
    if schema != IMITATION_ENSEMBLE_REPORT_SCHEMA:
        raise ValueError(f"unsupported imitation ensemble report schema: {schema}")
    ensemble = _select_ensemble(payload, ensemble_name=ensemble_name)
    name = str(ensemble.get("name", "") or "")
    members = ensemble.get("members", [])
    if not isinstance(members, list) or not members:
        raise ValueError("ensemble report selected entry has no members")
    expected_sha = ensemble.get("member_checkpoint_sha256", [])
    if expected_sha and (not isinstance(expected_sha, list) or len(expected_sha) != len(members)):
        raise ValueError("ensemble report member_checkpoint_sha256 length must match members")
    report_image_size = _positive_int((payload.get("args") or {}).get("image_size")) if isinstance(payload.get("args"), dict) else None
    resolved_image_size = int(image_size or report_image_size or 0)
    if resolved_image_size <= 0:
        raise ValueError("direct BC ensemble report is missing image_size; pass --image-size matching training")

    models: list[Any] = []
    member_paths: list[str] = []
    reference_stats: dict[str, Any] | None = None
    reference_cameras: list[str] | None = None
    reference_horizon: int | None = None
    reference_arm_mask: np.ndarray | None = None
    for index, member in enumerate(members):
        member_path = _resolve_report_member_path(report, member)
        if not member_path.exists():
            raise ValueError(f"ensemble member checkpoint does not exist: {member_path}")
        if expected_sha:
            actual_sha = _sha256_file(member_path)
            wanted_sha = str(expected_sha[index])
            if actual_sha != wanted_sha:
                raise ValueError(f"ensemble member SHA mismatch for {member_path}: {actual_sha} != {wanted_sha}")
        checkpoint = torch.load(member_path, map_location=device, weights_only=False)
        model = _build_direct_bc_model_from_checkpoint(checkpoint, device=device)
        stats = dict(checkpoint["dataset_stats"])
        cameras = [str(camera) for camera in checkpoint.get("camera_names", [])]
        horizon = int(checkpoint.get("action_horizon", 0) or 0)
        arm_mask = _arm_mask_from_stats(stats)
        if reference_stats is None:
            reference_stats = stats
            reference_cameras = cameras
            reference_horizon = horizon
            reference_arm_mask = arm_mask
        else:
            _validate_ensemble_member_compatible(
                member_path=member_path,
                stats=stats,
                camera_names=cameras,
                action_horizon=horizon,
                arm_mask=arm_mask,
                reference_stats=reference_stats,
                reference_camera_names=reference_cameras or [],
                reference_action_horizon=int(reference_horizon or 0),
                reference_arm_mask=reference_arm_mask if reference_arm_mask is not None else np.ones(2, dtype=np.float32),
            )
        models.append(model)
        member_paths.append(str(member_path))
    assert reference_stats is not None
    assert reference_cameras is not None
    assert reference_horizon is not None
    assert reference_arm_mask is not None
    return _DirectBcEnsembleBundle(
        name=name,
        member_paths=member_paths,
        models=models,
        stats=reference_stats,
        camera_names=reference_cameras,
        image_size=resolved_image_size,
        action_horizon=reference_horizon,
        arm_mask=reference_arm_mask,
    )


def _select_ensemble(payload: dict[str, Any], *, ensemble_name: str | None) -> dict[str, Any]:
    ensembles = payload.get("ensembles", payload.get("results", []))
    if not isinstance(ensembles, list) or not ensembles:
        raise ValueError("ensemble report has no ensembles")
    desired = str(ensemble_name or "").strip()
    if desired:
        for ensemble in ensembles:
            if isinstance(ensemble, dict) and str(ensemble.get("name", "") or "") == desired:
                return ensemble
        raise ValueError(f"ensemble {desired!r} not found in report")
    first = ensembles[0]
    if not isinstance(first, dict):
        raise ValueError("ensemble report first entry is not an object")
    return first


def _resolve_report_member_path(report_path: Path, member: Any) -> Path:
    path = Path(str(member))
    if path.is_absolute():
        return path
    return report_path.parent / path


def _build_direct_bc_model_from_checkpoint(checkpoint: dict[str, Any], *, device: torch.device) -> Any:
    schema = str(checkpoint.get("schema", "") or "")
    if schema != IMITATION_CHECKPOINT_SCHEMA:
        raise ValueError(f"unsupported imitation checkpoint schema: {schema}")
    family = str(checkpoint.get("model_family", "") or "")
    if family not in DIRECT_BC_RUNTIME_FAMILIES:
        raise ValueError(f"unsupported imitation checkpoint family: {family}")
    action_dim = int(checkpoint.get("action_dim", 0) or 0)
    proprio_dim = int(checkpoint.get("proprio_dim", 0) or 0)
    if action_dim != FLOW_ACTION_DIM:
        raise ValueError(f"unsupported imitation action_dim {action_dim}; expected {FLOW_ACTION_DIM}")
    if proprio_dim != FLOW_PROPRIO_DIM:
        raise ValueError(f"unsupported imitation proprio_dim {proprio_dim}; expected {FLOW_PROPRIO_DIM}")
    hidden_dim = _infer_direct_bc_hidden_dim(checkpoint["model_state"], family)
    backbone = str(checkpoint.get("backbone", "") or "")
    if not backbone:
        raise ValueError("imitation checkpoint is missing backbone")
    if family == "arm_structured_direct":
        from .imitation_experiments import _build_structured_direct_policy

        model = _build_structured_direct_policy(
            backbone=backbone,
            action_horizon=int(checkpoint["action_horizon"]),
            camera_count=len(checkpoint.get("camera_names", [])),
            hidden_dim=hidden_dim,
        ).to(device)
    else:
        from .imitation_experiments import _build_direct_bc_policy

        model = _build_direct_bc_policy(
            backbone=backbone,
            action_horizon=int(checkpoint["action_horizon"]),
            camera_count=len(checkpoint.get("camera_names", [])),
            hidden_dim=hidden_dim,
        ).to(device)
    model.load_state_dict(checkpoint["model_state"])
    model.eval()
    return model


def _validate_ensemble_member_compatible(
    *,
    member_path: Path,
    stats: dict[str, Any],
    camera_names: list[str],
    action_horizon: int,
    arm_mask: np.ndarray,
    reference_stats: dict[str, Any],
    reference_camera_names: list[str],
    reference_action_horizon: int,
    reference_arm_mask: np.ndarray,
) -> None:
    if camera_names != reference_camera_names:
        raise ValueError(f"ensemble member camera_names mismatch: {member_path}")
    if int(action_horizon) != int(reference_action_horizon):
        raise ValueError(f"ensemble member action_horizon mismatch: {member_path}")
    if not np.allclose(arm_mask, reference_arm_mask):
        raise ValueError(f"ensemble member arm_mask mismatch: {member_path}")
    for key in ("action_mean", "action_std", "proprio_mean", "proprio_std", "image_mean", "image_std"):
        if key not in stats or key not in reference_stats:
            raise ValueError(f"ensemble member missing dataset_stats.{key}: {member_path}")
        if not np.allclose(np.asarray(stats[key], dtype=np.float64), np.asarray(reference_stats[key], dtype=np.float64)):
            raise ValueError(f"ensemble member dataset_stats.{key} mismatch: {member_path}")


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


def _positive_int(value: Any) -> int | None:
    if isinstance(value, bool):
        return None
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return None
    return parsed if parsed > 0 else None


def _infer_direct_bc_hidden_dim(model_state: dict[str, Any], family: str) -> int:
    if family == "arm_structured_direct":
        weight = model_state.get("left_head.weight")
        if weight is not None and len(getattr(weight, "shape", ())) == 2:
            return int(weight.shape[1])
    weight = model_state.get("head.0.weight")
    if weight is not None and len(getattr(weight, "shape", ())) == 2:
        return int(weight.shape[0])
    raise ValueError("could not infer direct BC hidden_dim from checkpoint model_state")


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
    if family not in {"tcp_twist_local", "tcp_target_pose"}:
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
