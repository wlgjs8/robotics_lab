from __future__ import annotations

import copy
import hashlib
import json
import os
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, TextIO

import numpy as np
import torch

from .action_sources.tcp_pose_target import (
    clamp_pose_delta,
    cartesian_action_requirements,
    tcp_pose_target_stand_intent,
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
from .chunk_overlay_publisher import ChunkOverlayPublisher
from .flow_model import FlowMatchingPolicy, sample_action_chunks
from .pc_dataset import PC_CHECKPOINT_SCHEMA
from .tcp_target_pose_conditioner import (
    CONDITIONING_MODES,
    REANCHOR_MODES,
    OnlineTcpPoseTargetConditioner,
)
from .gripper import REAL_GRIPPER_ENV, GripperRuntime, gripper_commands_from_flow_step
from .robot_state_client import StateSnapshot
from .rollout_modes import RolloutMode, RolloutModeValidationError, parse_rollout_mode
from .servo_command_client import CommandIntent


def default_action_log_path() -> str | None:
    """Resolve the per-step action-log path.

    Action logging is intentionally opt-in for live flow-infer runs: disk I/O and
    terminal spam at every policy step add latency jitter. Set
    ``POLICY_RUNNER_ACTION_LOG=/path/to/actions.jsonl`` to capture a specific
    file, or ``POLICY_RUNNER_ACTION_LOG=auto``/``1`` for a timestamped file under
    ``logs/``. Empty/false/off disables logging.
    """
    configured = os.environ.get("POLICY_RUNNER_ACTION_LOG")
    if configured is None:
        return None
    configured = configured.strip()
    if not configured or configured.lower() in {"0", "false", "off", "no", "none"}:
        return None
    if configured.lower() in {"1", "true", "yes", "auto"}:
        logs_dir = Path(__file__).resolve().parents[2] / "logs"
        logs_dir.mkdir(parents=True, exist_ok=True)
        return str(logs_dir / f"actions_{time.strftime('%Y%m%d_%H%M%S')}.jsonl")
    return configured


def _env_truthy(value: str | None) -> bool:
    if value is None:
        return False
    return value.strip().lower() not in {"", "0", "false", "no", "off"}


def _quat_angle_deg(q1: np.ndarray, q2: np.ndarray) -> float:
    """Geodesic angle (deg) between two xyzw quaternions; 0 if either is null."""
    a = np.asarray(q1, dtype=np.float64)
    b = np.asarray(q2, dtype=np.float64)
    na = float(np.linalg.norm(a))
    nb = float(np.linalg.norm(b))
    if na == 0.0 or nb == 0.0:
        return 0.0
    dot = abs(float(np.dot(a / na, b / nb)))
    dot = min(1.0, max(-1.0, dot))
    return float(np.degrees(2.0 * np.arccos(dot)))


def _open_action_log(stderr: TextIO = sys.stderr) -> TextIO | None:
    path = default_action_log_path()
    if path is None:
        return None
    log_path = Path(path)
    if log_path.parent != Path(""):
        log_path.parent.mkdir(parents=True, exist_ok=True)
    handle = open(log_path, "w", buffering=1)
    print(f"[flow-infer] logging per-step actions to {log_path}", file=stderr, flush=True)
    return handle


FLOW_COMMAND_LABEL = "TcpPoseTarget"
_ZERO_TWIST: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
DEFAULT_FLOW_MAX_LINEAR_VELOCITY_M_S = 0.30
DEFAULT_FLOW_MAX_ANGULAR_VELOCITY_RAD_S = 2.0
DEFAULT_CHUNK_OVERLAY_RUNWAY_STEPS = 4
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
    command_family: str = FLOW_COMMAND_LABEL
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


# ---- ee_local body-frame alignment (training EE frame vs runtime TCP frame) ----
# Fixed rotation R between the training EE body frame (in the data) and the RB TCP
# frame the server interprets TcpPoseTarget deltas in. Applied symmetrically:
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


def resolve_flow_policy_dt_sec(
    rollout_mode: str | RolloutMode,
    *,
    policy_dt_sec: float | None,
    command_rate_hz: float,
    dataset_stats: dict[str, Any] | None = None,
) -> float | None:
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
            "TcpPoseTarget flow rollout requires --policy-dt-sec or checkpoint "
            f"dataset_stats.dt_mean_sec for rollout-mode {mode.value}"
        )
    rate = float(command_rate_hz)
    if rate <= 0.0:
        raise RolloutModeValidationError("command_rate_hz must be positive to derive policy dt")
    return 1.0 / rate


class FlowMatchingActionSource:
    """Runtime source for high-level flow-policy action chunks.

    The source consumes ee_local body-frame policy deltas and emits integrated
    absolute TcpPoseTarget setpoints. Live rollout is behind rollout-mode
    validation and SafetyGate still decides whether the intent may be sent.
    """

    def __init__(
        self,
        checkpoint_path: str | Path,
        *,
        timeout_sec: float = 0.2,
        camera_client: Any | None = None,
        sample_steps: int = 16,
        policy_dt_sec: float | None = None,
        max_linear_velocity_m_s: float | None = None,
        max_angular_velocity_rad_s: float | None = None,
        max_linear_step_m: float = 0.002,
        max_angular_step_rad: float = 0.01,
        chunk_execute_steps: int | None = None,
        chunk_overlay_runway_steps: int = DEFAULT_CHUNK_OVERLAY_RUNWAY_STEPS,
        chunk_crossfade_steps: int = 0,
        tcp_target_pose_conditioning: str = "legacy_step_hold",
        tcp_target_pose_reanchor_mode: str = "measured_blend",
        tcp_target_pose_blend_steps: int = 2,
        allow_rbpodo_controller_simulation_cartesian: bool = False,
        gripper_runtime: GripperRuntime | None = None,
        ee_local_r_align: Any = None,
        device: str = "auto",
        stochastic_sampling: bool = True,
        chunk_overlay_endpoint: str | None = None,
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
        # Percent subtracted from the ABSOLUTE gripper opening target so grasps
        # close more firmly (lower opening = more closed): e.g. 1.0 turns an 18%
        # command into 17%. 0 = off. No effect in delta mode. Set from the
        # `--gripper-close-bias` CLI flag in main.py. This is the SHARED base; the
        # per-arm overrides below win over it when set.
        self.gripper_close_bias = 0.0
        # Per-arm ABSOLUTE close-bias overrides (opening percent). None -> fall back
        # to the shared `gripper_close_bias` above. The two pika grippers clamp
        # differently, so main.py resolves independent left/right values from the
        # `--gripper-close-bias-left` / `--gripper-close-bias-right` CLI flags
        # (defaults left 2.0 / right 6.0).
        self.gripper_close_bias_left = None
        self.gripper_close_bias_right = None
        # BINARY gripper (for checkpoints trained with a binarized open/close
        # gripper, e.g. openpi `binary 25`): the model's gripper-action output is
        # bimodal, so threshold it and snap to the physical open/close presets
        # instead of commanding the raw opening. A flavour of the absolute path
        # (gripper_action_absolute stays True). Set from `--gripper-action-mode
        # binary` + `--gripper-open/close-percent` / `--gripper-binary-threshold`.
        self.gripper_binary = False
        self.gripper_open_percent = 50.0
        self.gripper_close_percent = 7.0
        self.gripper_binary_threshold = 50.0
        # ABSOLUTE close-snap deadzone: a mapped opening percent below this snaps
        # to 0 (fully closed), so small near-closed policy jitter does not leave
        # the jaw cracked open. 0 = off. Absolute (non-binary) mode only. Set from
        # the `--gripper-close-snap-percent` CLI flag in main.py.
        self.gripper_close_snap_percent = 0.0
        # ROTATION-AXIS MASK: per-axis gate over the policy's per-arm rotation
        # action (rx, ry, rz at action indices 3/4/5 and 10/11/12). Each entry
        # True keeps that axis, False zeros it before the action is applied so the
        # arm holds that orientation component. Default keeps all three (no
        # masking); (False, False, False) is translation-only. Translation (dxyz)
        # and gripper dims are always untouched. Set from `--rotation-axes` /
        # `--translation-only` in main.py.
        self.rotation_axes_enabled = (True, True, True)
        self.stderr = stderr
        self.device = _resolve_device(device)

        checkpoint = torch.load(checkpoint_path, map_location=self.device, weights_only=False)
        schema = str(checkpoint.get("schema", "") or "")
        # Subclasses (e.g. the point-cloud source) accept their own schema here.
        self._validate_checkpoint_schema(schema)
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
        self.command_family = FLOW_COMMAND_LABEL
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
        self.chunk_overlay_runway_steps = _resolve_chunk_overlay_runway_steps(chunk_overlay_runway_steps)
        self.policy_dt_sec = _resolve_runtime_policy_dt_sec(
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
        # Subclasses override _build_policy_model to load a different policy
        # (e.g. the point-cloud flow policy) from the same checkpoint dict.
        self.model = self._build_policy_model(model_config, checkpoint)

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
        self._last_server_motion_epoch: int | None = None
        self._force_recovery_config: Any | None = None
        self._init_force_recovery_state()
        self._init_inference_timing_state()
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
        self._last_overlay_payload: dict[str, Any] | None = None
        self._chunk_overlay_seq = 0
        self._stream_emitted_policy_steps = 0
        self._active_chunk_metadata: dict[str, object] | None = None
        self._stream_next_chunk_metadata: dict[str, object] | None = None
        self._stream_activation_candidate_metadata: dict[str, object] | None = None
        self.checkpoint_id = str(Path(checkpoint_path).expanduser())
        # Terminal chunk dump (debug): when FLOW_INFER_PRINT_CHUNK is truthy, print
        # every freshly-inferred action chunk step-by-step for both arms — position
        # deltas in meters, rotation deltas converted rad->deg, plus gripper target.
        self._print_chunk_enabled = _env_truthy(os.environ.get("FLOW_INFER_PRINT_CHUNK"))
        self._print_chunk_seq = 0
        self._print_delta_overlay_enabled = _env_truthy(os.environ.get("FLOW_INFER_PRINT_DELTA_OVERLAY"))
        # Per-step chunk-vs-actual tracking dump (FLOW_INFER_PRINT_TRACKING): as each
        # step executes, compare the model's predicted absolute pose (chunk deltas
        # composed onto the boundary anchor) against the live measured pose — position
        # error in mm, rotation error in deg — plus per-step predicted vs actual
        # displacement, all time-stamped from chunk activation for controller tuning.
        self._print_tracking_enabled = _env_truthy(os.environ.get("FLOW_INFER_PRINT_TRACKING"))
        self._trk_predicted: dict[str, list[np.ndarray] | None] = {"left": None, "right": None}
        self._trk_prev_measured: dict[str, np.ndarray | None] = {"left": None, "right": None}
        self._trk_start_monotonic = 0.0
        self._chunk_overlay_publisher: ChunkOverlayPublisher | None = None
        resolved_chunk_overlay_endpoint = (
            os.environ.get("RB_GUI_CHUNK_OVERLAY_ENDPOINT")
            if chunk_overlay_endpoint is None
            else chunk_overlay_endpoint
        )
        if resolved_chunk_overlay_endpoint and resolved_chunk_overlay_endpoint.strip():
            try:
                self._chunk_overlay_publisher = ChunkOverlayPublisher(resolved_chunk_overlay_endpoint.strip())
            except Exception as exc:
                print(
                    f"WARNING: {self.policy_label}: chunk overlay publisher disabled "
                    f"for {resolved_chunk_overlay_endpoint!r}: {type(exc).__name__}: {exc}",
                    file=self.stderr,
                )
        # Per-policy-step action logger (env-gated, debug only). Set
        # POLICY_RUNNER_ACTION_LOG=/path/to/actions.jsonl to capture one JSON
        # line per executed policy step: raw flow delta, converted/clamped twist
        # actually sent, chunk index and chunk-boundary marker. Used to diagnose
        # trembling (pulsed chunk boundaries vs. delta->twist noise amplification).
        self._action_log: TextIO | None = None
        self._action_log_seq = 0
        self._action_log = _open_action_log(self.stderr)

    # --------------------------------------------------------- override hooks --
    def on_arm_init_override_start(self, arms: tuple[str, ...], snapshot: StateSnapshot) -> None:
        before_mask = self.arm_mask.tolist() if hasattr(self.arm_mask, "tolist") else list(self.arm_mask)
        after_mask = list(before_mask)
        for arm in arms:
            idx = 0 if arm == "left" else 1
            if idx < len(after_mask):
                after_mask[idx] = 0.0
        self._clear_target_pose_state(arms)
        self._invalidate_policy_chunks(reason="arm_init_start")
        self._log_arm_init_event(
            "arm_init_override_start",
            arms,
            snapshot,
            before_mask=before_mask,
            after_mask=after_mask,
            chunk_invalidated=True,
        )

    def on_arm_init_override_done(self, arms: tuple[str, ...], snapshot: StateSnapshot) -> None:
        before = self._pose_snapshot_for_arms(arms)
        after = self._reanchor_arms_to_snapshot(arms, snapshot)
        self._invalidate_policy_chunks(reason="arm_init_done")
        self._log_arm_init_event(
            "arm_init_override_done",
            arms,
            snapshot,
            chunk_invalidated=True,
            reanchor_before=before,
            reanchor_after=after,
        )
        self._log_arm_init_event(
            "arm_init_override_resume",
            arms,
            snapshot,
            chunk_invalidated=True,
            reanchor_after=after,
        )

    def on_arm_init_override_cancel(self, arms: tuple[str, ...], snapshot: StateSnapshot) -> None:
        before = self._pose_snapshot_for_arms(arms)
        after = self._reanchor_arms_to_snapshot(arms, snapshot)
        self._invalidate_policy_chunks(reason="arm_init_cancel")
        self._log_arm_init_event(
            "arm_init_override_cancel",
            arms,
            snapshot,
            chunk_invalidated=True,
            reanchor_before=before,
            reanchor_after=after,
        )

    def on_arm_init_override_failed(self, arms: tuple[str, ...], snapshot: StateSnapshot) -> None:
        self._invalidate_policy_chunks(reason="arm_init_failed")
        self._log_arm_init_event(
            "arm_init_override_failed",
            arms,
            snapshot,
            chunk_invalidated=True,
        )

    def _validate_checkpoint_schema(self, schema: str) -> None:
        """Reject checkpoints whose schema this source cannot run. The base flow
        source only accepts RGB-image flow checkpoints; subclasses override to
        accept their own schema (e.g. the point-cloud flow policy)."""
        if schema != FLOW_CHECKPOINT_SCHEMA:
            raise ValueError(f"unsupported flow checkpoint schema: {schema}")

    def _build_policy_model(self, model_config: dict[str, Any], checkpoint: dict[str, Any]) -> Any:
        """Construct and load the policy network from the checkpoint dict.
        Subclasses override to build a different architecture."""
        model = FlowMatchingPolicy.from_checkpoint_config(model_config).to(self.device)
        model.load_state_dict(checkpoint["model_state"])
        model.eval()
        return model

    def next_intent(self, snapshot: StateSnapshot, now_monotonic: float) -> CommandIntent | None:
        self._last_overlay_payload = snapshot.payload
        blocked, recovery_intent = self._force_recovery_gate(snapshot, now_monotonic)
        if blocked:
            return recovery_intent
        self._handle_server_motion_epoch(snapshot)
        self._before_policy_intent(snapshot, now_monotonic)
        if getattr(self, "enable_async_chunking", False):
            return self._next_intent_streamed(snapshot, now_monotonic)
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
            chunk = self._sample_and_align_chunk_timed(payload)
            if chunk is None:
                self._chunk = None
                return self._no_policy_input_intent()
            self._chunk = chunk
            self._chunk_index = 0
            self._steps_since_boundary = 0  # restart the crossfade ramp at the boundary
            self._record_inference_activation()
            self._print_chunk_steps(chunk)
            self._begin_chunk_tracking(chunk, payload, now_monotonic)
            self._publish_chunk_overlay(now_monotonic)
        self._log_chunk_tracking(payload, now_monotonic, int(self._chunk_index))
        step = self._chunk[self._chunk_index]
        self._chunk_index += 1
        gripper_targets = self._integrate_gripper_targets(step, payload)
        self._dispatch_gripper_step(step)
        return self._emit_step_intent(step, payload, gripper_targets)

    def configure_force_recovery(self, config: Any) -> None:
        """Enable the bimanual policy gate without changing server force ownership."""
        self._force_recovery_config = config
        self._init_force_recovery_state()

    def _init_force_recovery_state(self) -> None:
        self._force_recovery_state = "running"
        self._force_recovery_contact_started: float | None = None
        self._force_recovery_release_time: float | None = None
        self._force_recovery_quiet_since: float | None = None
        self._force_recovery_prev_pose: dict[str, tuple[float, np.ndarray] | None] = {
            "left": None,
            "right": None,
        }
        self._force_recovery_frozen_gripper: dict[str, float | None] = {
            "left": None,
            "right": None,
        }
        self._force_recovery_camera_barrier_seq: int | None = None
        self._force_recovery_camera_barrier_received: float | None = None
        self._force_recovery_camera_latest_seq: int | None = None
        self._force_recovery_camera_latest_received: float | None = None
        self._force_recovery_camera_fresh = not bool(getattr(self, "camera_names", ()))
        self._force_recovery_blocked_on = "none"
        self._force_recovery_last_now: float | None = None
        self._force_recovery_arm_status = {
            arm: {"contact_active": False, "measured_normal_force_n": None}
            for arm in ("left", "right")
        }
        self._force_recovery_tcp_velocity = {
            arm: {"linear_m_s": None, "angular_rad_s": None}
            for arm in ("left", "right")
        }
        self._force_recovery_terminal_abort_reason: str | None = None
        self._force_recovery_counters = {
            "contact_events": 0,
            "recontacts": 0,
            "chunk_invalidations": 0,
            "hold_intents": 0,
            "measured_reanchors": 0,
            "cold_inference_restarts": 0,
            "timeouts": 0,
            "contact_timeouts": 0,
            "settling_timeouts": 0,
            "camera_stale_timeouts": 0,
        }

    def _before_policy_intent(
        self, snapshot: StateSnapshot, now_monotonic: float
    ) -> None:
        _ = (snapshot, now_monotonic)

    @property
    def force_recovery_terminal_abort_reason(self) -> str | None:
        return getattr(self, "_force_recovery_terminal_abort_reason", None)

    @property
    def terminal_abort_reason(self) -> str | None:
        camera_reason = getattr(self, "_camera_terminal_abort_reason", None)
        if camera_reason is not None:
            resolved = str(camera_reason).strip()
            if resolved:
                return resolved
        return self.force_recovery_terminal_abort_reason

    def force_recovery_status(self) -> dict[str, Any]:
        config = getattr(self, "_force_recovery_config", None)
        counters = dict(getattr(self, "_force_recovery_counters", {}))
        now = getattr(self, "_force_recovery_last_now", None)
        contact_started = getattr(self, "_force_recovery_contact_started", None)
        release_time = getattr(self, "_force_recovery_release_time", None)
        contact_elapsed = (
            None
            if now is None or contact_started is None
            else max(0.0, float(now) - float(contact_started))
        )
        settling_elapsed = (
            None
            if now is None or release_time is None
            else max(0.0, float(now) - float(release_time))
        )
        camera_received = getattr(self, "_force_recovery_camera_latest_received", None)
        camera_age = (
            None
            if now is None or camera_received is None
            else max(0.0, float(now) - float(camera_received))
        )
        return {
            "enabled": bool(getattr(config, "enable", False)),
            "contact_behavior": str(getattr(config, "contact_behavior", "recover")),
            "state": str(getattr(self, "_force_recovery_state", "running")),
            "blocked_on": str(getattr(self, "_force_recovery_blocked_on", "none")),
            "terminal_abort_reason": self.force_recovery_terminal_abort_reason,
            "contact_elapsed_sec": contact_elapsed,
            "settling_elapsed_sec": settling_elapsed,
            "settle_time_sec": float(getattr(config, "settle_time_sec", 0.12)),
            "max_linear_velocity_m_s": float(
                getattr(config, "max_linear_velocity_m_s", 0.002)
            ),
            "max_angular_velocity_rad_s": float(
                getattr(config, "max_angular_velocity_rad_s", 0.05)
            ),
            "contact_timeout_sec": float(
                getattr(config, "contact_timeout_sec", 5.0)
            ),
            "settling_timeout_sec": float(
                getattr(config, "settling_timeout_sec", 2.0)
            ),
            "arms": {
                arm: dict(values)
                for arm, values in getattr(self, "_force_recovery_arm_status", {}).items()
            },
            "measured_tcp_velocity": {
                arm: dict(values)
                for arm, values in getattr(self, "_force_recovery_tcp_velocity", {}).items()
            },
            "camera": {
                "barrier_seq": getattr(self, "_force_recovery_camera_barrier_seq", None),
                "latest_seq": getattr(self, "_force_recovery_camera_latest_seq", None),
                "latest_age_sec": camera_age,
                "fresh": bool(getattr(self, "_force_recovery_camera_fresh", False)),
            },
            "inflight_worker_generation": self._force_recovery_inflight_generation(),
            "frozen_gripper_targets": dict(
                getattr(self, "_force_recovery_frozen_gripper", {})
            ),
            "counters": counters,
            "velproprio_sample_mode": getattr(self, "velproprio_sample_mode", None),
            "velproprio_source": getattr(self, "velproprio_source", None),
        }

    @staticmethod
    def _force_contact_active(payload: dict[str, Any]) -> bool:
        for arm in ("left", "right"):
            arm_payload = payload.get(arm, {})
            force_control = arm_payload.get("force_control", {}) if isinstance(arm_payload, dict) else {}
            if isinstance(force_control, dict) and force_control.get("contact_active") is True:
                return True
        return False

    def _force_recovery_gate(
        self, snapshot: StateSnapshot, now_monotonic: float
    ) -> tuple[bool, CommandIntent | None]:
        config = getattr(self, "_force_recovery_config", None)
        if not bool(getattr(config, "enable", False)):
            return False, None

        now = float(now_monotonic)
        payload = snapshot.payload
        self._force_recovery_last_now = now
        self._update_force_recovery_arm_status(payload)
        contact = self._force_contact_active(payload)
        if str(getattr(config, "contact_behavior", "recover")) == "continue":
            self._force_recovery_state = "running"
            self._force_recovery_blocked_on = "none"
            return False, None
        state = str(getattr(self, "_force_recovery_state", "running"))

        if contact and state != "contact":
            if state == "settling":
                self._force_recovery_counters["recontacts"] += 1
            self._enter_force_contact(payload, now)
            return True, self._force_recovery_hold_intent()

        if state == "contact":
            self._sync_force_recovery_motion_epoch(payload)
            if contact:
                self._force_recovery_blocked_on = "contact_active"
                if self._force_recovery_phase_timed_out(now, "contact"):
                    return True, self._force_recovery_hold_intent()
                return True, self._force_recovery_hold_intent()
            self._begin_force_recovery_settling(snapshot, now)
            return True, self._force_recovery_hold_intent()

        if state == "settling":
            self._sync_force_recovery_motion_epoch(payload)
            self._record_force_recovery_pose_history(snapshot, now)
            quiet = self._update_force_recovery_quiet_window(payload, now)
            fresh_camera = self._force_recovery_has_fresh_camera(now)
            stream_quiescent = self._force_recovery_stream_quiescent()
            if not quiet:
                self._force_recovery_blocked_on = "tcp_motion"
            elif not fresh_camera:
                self._force_recovery_blocked_on = "stale_camera"
            elif not stream_quiescent:
                self._force_recovery_blocked_on = "stale_worker"
            else:
                self._force_recovery_blocked_on = "none"
            if self._force_recovery_phase_timed_out(now, "settling"):
                return True, self._force_recovery_hold_intent()
            if quiet and fresh_camera and stream_quiescent:
                self._complete_force_recovery(snapshot)
                return False, None
            return True, self._force_recovery_hold_intent()

        if state == "timed_out":
            self._sync_force_recovery_motion_epoch(payload)
            return True, self._force_recovery_hold_intent()

        return False, None

    def _enter_force_contact(self, payload: dict[str, Any], now: float) -> None:
        self._force_recovery_frozen_gripper = {
            arm: self._force_recovery_gripper_target(payload, arm)
            for arm in ("left", "right")
        }
        self._invalidate_policy_chunks(reason="force_contact")
        self._force_recovery_counters["chunk_invalidations"] += 1
        self._force_recovery_counters["contact_events"] += 1
        self._force_recovery_state = "contact"
        self._force_recovery_blocked_on = "contact_active"
        self._force_recovery_contact_started = now
        self._force_recovery_release_time = None
        self._force_recovery_quiet_since = None
        self._force_recovery_prev_pose = {"left": None, "right": None}
        self._force_recovery_terminal_abort_reason = None
        self._sync_force_recovery_motion_epoch(payload)

    def _begin_force_recovery_settling(
        self, snapshot: StateSnapshot, now: float
    ) -> None:
        bundle = self._poll_camera_bundle() if getattr(self, "camera_names", ()) else None
        self._force_recovery_camera_barrier_seq = self._camera_bundle_seq(bundle)
        self._force_recovery_camera_barrier_received = self._camera_bundle_received(bundle)
        reset_rtc = getattr(self, "reset_rtc", None)
        if callable(reset_rtc):
            reset_rtc()
        self._clear_target_pose_state(preserve_gripper_targets=True)
        self._force_recovery_state = "settling"
        self._force_recovery_blocked_on = "tcp_motion"
        self._force_recovery_release_time = now
        self._force_recovery_quiet_since = now
        self._force_recovery_prev_pose = {"left": None, "right": None}
        self._record_force_recovery_pose_history(snapshot, now)
        self._update_force_recovery_quiet_window(snapshot.payload, now)
        self._sync_force_recovery_motion_epoch(snapshot.payload)

    def _complete_force_recovery(self, snapshot: StateSnapshot) -> None:
        self._clear_target_pose_state(preserve_gripper_targets=True)
        for arm, value in self._force_recovery_frozen_gripper.items():
            if value is not None:
                self._gripper_targets_by_arm[arm] = float(value)
        self._reset_left_pose = pose_from_state_payload(snapshot.payload, "left")
        self._reset_right_pose = pose_from_state_payload(snapshot.payload, "right")
        self._force_recovery_state = "running"
        self._force_recovery_blocked_on = "none"
        self._force_recovery_contact_started = None
        self._force_recovery_release_time = None
        self._force_recovery_counters["measured_reanchors"] += 1
        self._force_recovery_counters["cold_inference_restarts"] += 1
        self._sync_force_recovery_motion_epoch(snapshot.payload)

    def _force_recovery_phase_timed_out(self, now: float, phase: str) -> bool:
        config = getattr(self, "_force_recovery_config", None)
        if phase == "contact":
            started = getattr(self, "_force_recovery_contact_started", None)
            timeout = float(getattr(config, "contact_timeout_sec", 5.0))
            reason = "force_contact_timeout"
            counter = "contact_timeouts"
        elif phase == "settling":
            started = getattr(self, "_force_recovery_release_time", None)
            timeout = float(getattr(config, "settling_timeout_sec", 2.0))
            camera_blocked = getattr(self, "_force_recovery_blocked_on", "none") == "stale_camera"
            reason = "camera_stale_timeout" if camera_blocked else "force_settling_timeout"
            counter = "camera_stale_timeouts" if camera_blocked else "settling_timeouts"
        else:
            raise ValueError(f"unsupported force recovery phase: {phase}")
        if started is None or now - float(started) < timeout:
            return False
        if self._force_recovery_state != "timed_out":
            self._force_recovery_state = "timed_out"
            self._force_recovery_terminal_abort_reason = reason
            self._force_recovery_counters["timeouts"] += 1
            self._force_recovery_counters[counter] += 1
        return True

    def _force_recovery_hold_intent(self) -> CommandIntent:
        self._force_recovery_counters["hold_intents"] += 1
        frozen = self._force_recovery_frozen_gripper
        return CommandIntent.gripper_target(
            left=frozen.get("left"),
            right=frozen.get("right"),
            timeout_sec=self.timeout_sec,
        )

    def _force_recovery_gripper_target(
        self, payload: dict[str, Any], arm: str
    ) -> float | None:
        for mapping_name in ("_current_gripper_targets", "_gripper_targets_by_arm"):
            mapping = getattr(self, mapping_name, None)
            if isinstance(mapping, dict) and mapping.get(arm) is not None:
                return float(mapping[arm])
        value = _gripper_value_from_payload(payload, arm)
        if value is None:
            value = self._live_gripper_percent(arm)
        return None if value is None else float(value)

    def _record_force_recovery_pose_history(
        self, snapshot: StateSnapshot, now: float
    ) -> None:
        self._before_policy_intent(snapshot, now)

    def _update_force_recovery_quiet_window(
        self, payload: dict[str, Any], now: float
    ) -> bool:
        config = self._force_recovery_config
        moving = False
        valid_arms = 0
        velocities: dict[str, dict[str, float | None]] = {
            arm: {"linear_m_s": None, "angular_rad_s": None}
            for arm in ("left", "right")
        }
        for arm in ("left", "right"):
            try:
                pose = np.asarray(pose_from_state_payload(payload, arm), dtype=np.float64)
            except Exception:
                moving = True
                continue
            valid_arms += 1
            previous = self._force_recovery_prev_pose.get(arm)
            self._force_recovery_prev_pose[arm] = (now, pose.copy())
            if previous is None:
                continue
            previous_time, previous_pose = previous
            dt = now - float(previous_time)
            if dt <= 0.0:
                moving = True
                continue
            linear_velocity = float(np.linalg.norm(pose[:3] - previous_pose[:3])) / dt
            angular_velocity = np.radians(_quat_angle_deg(previous_pose[3:7], pose[3:7])) / dt
            velocities[arm] = {
                "linear_m_s": linear_velocity,
                "angular_rad_s": float(angular_velocity),
            }
            if (
                linear_velocity > float(config.max_linear_velocity_m_s)
                or angular_velocity > float(config.max_angular_velocity_rad_s)
            ):
                moving = True
        self._force_recovery_tcp_velocity = velocities
        if moving or valid_arms != 2:
            self._force_recovery_quiet_since = now
            return False
        quiet_since = self._force_recovery_quiet_since
        if quiet_since is None:
            self._force_recovery_quiet_since = now
            return False
        required = max(0.12, float(config.settle_time_sec), float(self.policy_dt_sec))
        return now - float(quiet_since) >= required

    def _force_recovery_has_fresh_camera(self, now: float | None = None) -> bool:
        if not getattr(self, "camera_names", ()):
            self._force_recovery_camera_fresh = True
            return True
        bundle = self._poll_camera_bundle()
        seq = self._camera_bundle_seq(bundle)
        received = self._camera_bundle_received(bundle)
        self._force_recovery_camera_latest_seq = seq
        self._force_recovery_camera_latest_received = received
        if self._count_missing_camera_frames(bundle) != 0:
            self._force_recovery_camera_fresh = False
            return False
        barrier_seq = self._force_recovery_camera_barrier_seq
        barrier_received = self._force_recovery_camera_barrier_received
        if seq is not None and barrier_seq is not None:
            fresh = seq > barrier_seq
        elif received is not None and barrier_received is not None:
            fresh = received > barrier_received
        else:
            release_time = self._force_recovery_release_time
            fresh = received is not None and release_time is not None and received > release_time
        self._force_recovery_camera_fresh = bool(fresh)
        return bool(fresh)

    def _update_force_recovery_arm_status(self, payload: dict[str, Any]) -> None:
        status: dict[str, dict[str, Any]] = {}
        for arm in ("left", "right"):
            arm_payload = payload.get(arm, {})
            force = arm_payload.get("force_control", {}) if isinstance(arm_payload, dict) else {}
            measured: float | None = None
            if isinstance(force, dict):
                raw_measured = force.get("measured_force_n")
                if isinstance(raw_measured, (int, float)) and not isinstance(raw_measured, bool):
                    measured = float(raw_measured)
            status[arm] = {
                "contact_active": bool(
                    isinstance(force, dict) and force.get("contact_active") is True
                ),
                "measured_normal_force_n": measured,
            }
        self._force_recovery_arm_status = status

    @staticmethod
    def _camera_bundle_seq(bundle: Any | None) -> int | None:
        try:
            return int(getattr(bundle, "bundle_seq")) if bundle is not None else None
        except (AttributeError, TypeError, ValueError):
            return None

    @staticmethod
    def _camera_bundle_received(bundle: Any | None) -> float | None:
        try:
            return float(getattr(bundle, "received_monotonic")) if bundle is not None else None
        except (AttributeError, TypeError, ValueError):
            return None

    def _sync_force_recovery_motion_epoch(self, payload: dict[str, Any]) -> None:
        raw_epoch = payload.get("motion_epoch")
        if isinstance(raw_epoch, bool) or not isinstance(raw_epoch, (int, float)):
            return
        try:
            self._last_server_motion_epoch = int(raw_epoch)
        except (OverflowError, ValueError):
            pass

    def _force_recovery_stream_quiescent(self) -> bool:
        lock = getattr(self, "_stream_lock", None)
        if lock is None:
            return True
        with lock:
            inflight = getattr(self, "_stream_inflight_generation", None)
            return inflight is None

    def _force_recovery_inflight_generation(self) -> int | None:
        lock = getattr(self, "_stream_lock", None)
        if lock is None:
            return None
        with lock:
            value = getattr(self, "_stream_inflight_generation", None)
            return None if value is None else int(value)

    def _on_stale_inference_completion(self) -> None:
        """Remove subclass inference side effects before recovery can replan.

        OpenPI sampling updates RTC guidance, pose history, and the camera
        watermark before returning its chunk. A generation check can discard the
        chunk, but it cannot undo those writes, so reset them here and restart the
        post-release observation window after the stale worker has exited.
        """
        bundle = getattr(self, "_last_obs_camera_bundle", None)
        barrier_seq = self._camera_bundle_seq(bundle)
        if barrier_seq is None:
            raw_seq = getattr(self, "_last_obs_camera_seq", None)
            try:
                barrier_seq = None if raw_seq is None else int(raw_seq)
            except (TypeError, ValueError):
                barrier_seq = None
        barrier_received = self._camera_bundle_received(bundle)
        reset_rtc = getattr(self, "reset_rtc", None)
        if callable(reset_rtc):
            reset_rtc()
        if getattr(self, "_force_recovery_state", "running") == "settling":
            now = time.monotonic()
            self._force_recovery_last_now = now
            self._force_recovery_camera_barrier_seq = barrier_seq
            self._force_recovery_camera_barrier_received = barrier_received
            self._force_recovery_quiet_since = now
            self._force_recovery_prev_pose = {"left": None, "right": None}

    def _handle_server_motion_epoch(self, snapshot: StateSnapshot) -> None:
        """Invalidate every cached/in-flight action after a server contact event.

        The server increments motion_epoch when force ownership enters/exits or
        an external-force fault is raised. An older policy chunk must never
        resume its accumulated normal delta after that discontinuity.
        """
        raw_epoch = snapshot.payload.get("motion_epoch")
        if isinstance(raw_epoch, bool) or not isinstance(raw_epoch, (int, float)):
            return
        try:
            epoch = int(raw_epoch)
        except (OverflowError, ValueError):
            return
        if self._last_server_motion_epoch is None:
            self._last_server_motion_epoch = epoch
            return
        if epoch == self._last_server_motion_epoch:
            return
        self._last_server_motion_epoch = epoch
        self._clear_target_pose_state()
        self._invalidate_policy_chunks(reason="server_motion_epoch")
        try:
            self._reset_left_pose = pose_from_state_payload(snapshot.payload, "left")
            self._reset_right_pose = pose_from_state_payload(snapshot.payload, "right")
        except Exception:
            self._reset_left_pose = None
            self._reset_right_pose = None

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
        self._last_overlay_payload = snapshot.payload
        payload = snapshot.payload
        if self._reset_left_pose is None or self._reset_right_pose is None:
            self._reset_left_pose = pose_from_state_payload(payload, "left")
            self._reset_right_pose = pose_from_state_payload(payload, "right")
        self._ensure_stream_state()

        advanced = False
        if self._chunk is None:
            # Need a chunk: live real/controller rollouts should never block the
            # command loop on inference. Unit tests and offline/synchronous callers
            # keep the legacy one-shot inline sample unless the runner sets
            # nonblocking_stream_inference=True.
            chunk = self._take_prefetched()
            if (
                chunk is None
                and not self._stream_pending
                and not bool(getattr(self, "nonblocking_stream_inference", False))
            ):
                chunk = self._sample_and_align_chunk_timed(payload)
            if chunk is None:
                self._request_prefetch(payload)
                sched = self._ensure_chunk_ensemble()
                if sched is not None:
                    sched.note_kick(int(sched.window_start))
                return self._stream_hold_intent()
            sched = self._ensure_chunk_ensemble()
            if sched is not None:
                anchor_fn = self._ensemble_anchor_fn(payload)
                if not sched.first_window_built:
                    chunk = sched.begin(chunk, anchor_fn)
                else:
                    # late resume after a stall: the raw chunk joins the schedule
                    chunk = sched.advance(chunk, anchor_fn)
                    if chunk is None:
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
                sched = self._ensure_chunk_ensemble()
                if sched is not None:
                    window = sched.advance(swapped, self._ensemble_anchor_fn(payload))
                    if window is not None:
                        self._activate_chunk(window, now_monotonic)
                        advanced = True
                        swapped = window
                    else:
                        swapped = None  # starved -> fall into the stall path below
                if sched is None and swapped is not None:
                    self._activate_chunk(swapped, now_monotonic)
                    advanced = True
                elif swapped is None:
                    # Next chunk not ready at the boundary: hold (zero twist) and
                    # drop the stale chunk so the next tick re-acquires cleanly.
                    self._stream_stall_count += 1
                    self._chunk = None
                    self._request_prefetch(payload)
                    if sched is not None:
                        sched.note_kick(int(sched.window_start))
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
            sched = getattr(self, "_chunk_ensemble", None)
            if sched is not None:
                sched.note_kick(self._ensemble_wall_now())

        foh = self._tcp_tp_foh_active()
        if advanced and self._chunk is not None:
            self._log_chunk_tracking(payload, now_monotonic, int(self._chunk_index))
            step = self._chunk[self._chunk_index]
            gripper_targets = self._integrate_gripper_targets(step, payload)
            self._dispatch_gripper_step(step)
            self._current_gripper_targets = gripper_targets
            if foh:
                # On a fresh chunk activation (index 0) install its FOH knots, then emit
                # the per-tick interpolated target. Gripper stays step-based.
                if self._chunk_index == 0:
                    self._foh_begin_chunk(payload, now_monotonic)
                self._remember_emitted_deltas_for_step(step, step_index=self._chunk_index)
                self._steps_since_boundary += 1
                self._current_step_intent = self._foh_tick_intent(now_monotonic, stall=False)
                self._log_foh_action(step, self._current_step_intent, now_monotonic)
                self._print_step_debug(step, payload)  # per-policy-step (foh_se3), not per servo tick
            else:
                self._current_step_intent = self._emit_step_intent(step, payload, gripper_targets)
            self._stream_emitted_policy_steps = int(
                getattr(self, "_stream_emitted_policy_steps", 0)
            ) + 1
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
        intent = self._tcp_target_pose_step_intent(
            step,
            payload=payload,
            gripper_targets=gripper_targets,
        )
        # Advance the crossfade ramp once per executed policy step (not per servo
        # tick): the streamed path re-emits the same held intent between steps.
        self._steps_since_boundary += 1
        self._log_action_step(step, intent)
        self._print_step_debug(step, payload)
        return intent

    def _print_step_debug(self, step: np.ndarray, payload: dict[str, Any]) -> None:
        """Per-step ACTION (post r_align ee_local delta + gripper) and PROPRIO
        (live measured pose pos/quat + gripper) to the terminal. Gated by
        POLICY_RUNNER_PRINT_STEPS=1 so it never affects normal runs."""
        if os.environ.get("POLICY_RUNNER_PRINT_STEPS", "") != "1":
            return
        s = np.asarray(step, dtype=np.float64).reshape(-1)

        def f(v, n):
            return " ".join(f"{x:+.4f}" for x in np.asarray(v).reshape(-1)[:n])

        aL, aR = s[0:7], s[7:14]
        lines = [
            f"[step idx={self._chunk_index}] ACT R(dxyz|rxyz|grip)=[{f(aR[:3],3)} | {f(aR[3:6],3)} | {aR[6]:+.2f}]",
            f"            ACT L=[{f(aL[:3],3)} | {f(aL[3:6],3)} | {aL[6]:+.2f}]",
        ]
        try:
            pR = np.asarray(pose_from_state_payload(payload, "right"))
            pL = np.asarray(pose_from_state_payload(payload, "left"))
            gR = _gripper_value_from_payload(payload, "right")
            gL = _gripper_value_from_payload(payload, "left")
            lines.append(f"            PROP R pos=[{f(pR[:3],3)}] quat=[{f(pR[3:7],4)}] grip={gR}")
            lines.append(f"            PROP L pos=[{f(pL[:3],3)}] quat=[{f(pL[3:7],4)}] grip={gL}")
        except Exception as exc:  # noqa: BLE001 - debug print must never break the loop
            lines.append(f"            PROP n/a ({exc})")
        print("\n".join(lines), flush=True)

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
            "command_family": self.command_family,
            # Raw flow output before delta->twist conversion / clamping.
            "raw_delta": raw.tolist(),
            # Actual per-arm payload sent downstream (twist after /policy_dt and
            # clamp, or None if the arm was idle / no intent emitted).
            "left": None if intent is None else intent.left,
            "right": None if intent is None else intent.right,
        }
        self._action_log_seq += 1
        log.write(json.dumps(record) + "\n")

    def _overlay_chain_advance(self) -> None:
        # Promote the outgoing chunk's chain tail to the anchor for the chunk
        # being activated (chain anchor mode). No-op when nothing is pending.
        pending = getattr(self, "_overlay_chain_pending", None)
        if pending is None:
            return
        prev = getattr(self, "_overlay_chain_prev", None)
        if prev is None:
            prev = {"left": None, "right": None}
            self._overlay_chain_prev = prev
        for arm, tail in pending.items():
            if tail is not None:
                prev[arm] = tail
        self._overlay_chain_pending = {"left": None, "right": None}

    def _activate_chunk(self, chunk: np.ndarray, now_monotonic: float) -> None:
        self._record_inference_activation(now_monotonic)
        metadata = dict(getattr(self, "_stream_activation_candidate_metadata", None) or {})
        observation_step_seq = int(
            metadata.get(
                "observation_step_seq",
                getattr(self, "_stream_emitted_policy_steps", 0),
            )
        )
        activation_step_seq = int(getattr(self, "_stream_emitted_policy_steps", 0))
        source_start_index = max(0, activation_step_seq - observation_step_seq)
        original_horizon = int(chunk.shape[0])
        if source_start_index >= original_horizon:
            self._active_chunk_metadata = {
                **metadata,
                "observation_step_seq": observation_step_seq,
                "activation_step_seq": activation_step_seq,
                "source_start_index": source_start_index,
                "original_horizon": original_horizon,
                "selected_horizon": 0,
                "alignment_outcome": "exhausted",
            }
            self._stream_activation_candidate_metadata = None
            self._chunk = None
            return
        if source_start_index > 0:
            chunk = np.asarray(chunk[source_start_index:], dtype=chunk.dtype)
        self._active_chunk_metadata = {
            **metadata,
            "observation_step_seq": observation_step_seq,
            "activation_step_seq": activation_step_seq,
            "source_start_index": source_start_index,
            "original_horizon": original_horizon,
            "selected_horizon": int(chunk.shape[0]),
            "alignment_outcome": "aligned",
        }
        self._stream_activation_candidate_metadata = None
        self._overlay_chain_advance()
        self._chunk = chunk
        self._chunk_index = 0
        self._steps_since_boundary = 0  # restart the crossfade ramp at the boundary
        self._step_deadline = now_monotonic + float(self.policy_dt_sec)
        self._print_chunk_steps(chunk)
        self._begin_chunk_tracking(chunk, getattr(self, "_last_overlay_payload", None), now_monotonic)
        self._publish_chunk_overlay(now_monotonic)

    def _begin_chunk_tracking(
        self,
        chunk: np.ndarray | None,
        payload: dict[str, Any] | None,
        now_monotonic: float,
    ) -> None:
        """Snapshot the predicted absolute pose trajectory for a freshly activated
        chunk so subsequent steps can be compared against the measured pose. Mirrors
        the overlay projection: anchor at the current measured pose, then compose the
        per-step conditioned deltas. Stored poses[0]=anchor, poses[i+1]=after step i."""
        self._trk_predicted = {"left": None, "right": None}
        self._trk_prev_measured = {"left": None, "right": None}
        self._trk_start_monotonic = now_monotonic
        if not getattr(self, "_print_tracking_enabled", False) or chunk is None or payload is None:
            return
        try:
            limit = self._current_chunk_execute_limit()
            if limit <= 0:
                return
            for arm, idx, sl in self._foh_arm_indices():
                if len(self.arm_mask) <= idx or self.arm_mask[idx] <= 0.0:
                    continue
                cur = np.asarray(pose_from_state_payload(payload, arm), dtype=np.float64)
                poses = [cur.copy()]
                for i in range(limit):
                    delta = np.asarray(
                        self._condition_arm_delta(arm, chunk[i][sl], step_index=i, update_prev=False),
                        dtype=np.float64,
                    )
                    cur = np.asarray(pose_compose_local(cur, delta), dtype=np.float64)
                    poses.append(cur.copy())
                self._trk_predicted[arm] = poses
        except Exception:
            self._trk_predicted = {"left": None, "right": None}

    def _log_chunk_tracking(
        self, payload: dict[str, Any], now_monotonic: float, index: int
    ) -> None:
        """Per-step chunk-vs-actual line: for each arm, tracking error (predicted
        absolute pose for this step vs measured), plus predicted and actual per-step
        displacement. Time-stamped from chunk activation. Gated on
        FLOW_INFER_PRINT_TRACKING (see __init__)."""
        if not getattr(self, "_print_tracking_enabled", False):
            return
        predicted = getattr(self, "_trk_predicted", None)
        if not predicted:
            return
        try:
            t = float(now_monotonic) - float(getattr(self, "_trk_start_monotonic", now_monotonic))
            prev = getattr(self, "_trk_prev_measured", None) or {"left": None, "right": None}
            parts: list[str] = []
            for arm, tag in (("left", "L"), ("right", "R")):
                poses = predicted.get(arm)
                if poses is None or index >= len(poses):
                    continue
                meas = np.asarray(pose_from_state_payload(payload, arm), dtype=np.float64)
                tgt = np.asarray(poses[index], dtype=np.float64)
                trk_pos = float(np.linalg.norm(meas[:3] - tgt[:3]) * 1000.0)
                trk_rot = _quat_angle_deg(meas[3:7], tgt[3:7])
                nxt = poses[index + 1] if index + 1 < len(poses) else None
                pred_step = (
                    float(np.linalg.norm(np.asarray(nxt, dtype=np.float64)[:3] - tgt[:3]) * 1000.0)
                    if nxt is not None
                    else 0.0
                )
                pm = prev.get(arm)
                act_step = (
                    float(np.linalg.norm(meas[:3] - np.asarray(pm, dtype=np.float64)[:3]) * 1000.0)
                    if pm is not None
                    else 0.0
                )
                prev[arm] = meas
                parts.append(
                    f"{tag} trk(p={trk_pos:6.1f}mm r={trk_rot:5.1f}°) "
                    f"pred_step={pred_step:6.1f}mm act_step={act_step:6.1f}mm"
                )
            self._trk_prev_measured = prev
            if parts:
                print(
                    f"[trk #{getattr(self, '_print_chunk_seq', 0)} i={index:02d} t={t:+.3f}s] "
                    + "  |  ".join(parts),
                    file=self.stderr,
                    flush=True,
                )
        except Exception:
            return

    def _print_chunk_steps(self, chunk: np.ndarray | None) -> None:
        """Debug: dump a freshly-inferred chunk to the terminal, one line per
        step, for both arms. Position deltas stay in meters; rotation deltas are
        converted rad->deg. Gated on FLOW_INFER_PRINT_CHUNK (see __init__)."""
        if not getattr(self, "_print_chunk_enabled", False) or chunk is None:
            return
        try:
            arr = np.asarray(chunk, dtype=np.float64)
            limit = self._current_chunk_execute_limit()
            n = min(int(limit), arr.shape[0]) if limit > 0 else arr.shape[0]
            self._print_chunk_seq += 1
            print(
                f"[flow-infer] chunk #{self._print_chunk_seq} steps={n} "
                f"(dpos=m, drot=deg, per-step deltas)",
                file=self.stderr,
                flush=True,
            )
            for i in range(n):
                row = arr[i]
                lp, lr, lg = row[0:3], np.degrees(row[3:6]), row[6]
                rp, rr, rg = row[7:10], np.degrees(row[10:13]), row[13]
                print(
                    f"  [{i:02d}] "
                    f"L dpos=[{lp[0]:+.4f} {lp[1]:+.4f} {lp[2]:+.4f}] "
                    f"drot=[{lr[0]:+6.2f} {lr[1]:+6.2f} {lr[2]:+6.2f}] grip={lg:+.3f}  |  "
                    f"R dpos=[{rp[0]:+.4f} {rp[1]:+.4f} {rp[2]:+.4f}] "
                    f"drot=[{rr[0]:+6.2f} {rr[1]:+6.2f} {rr[2]:+6.2f}] grip={rg:+.3f}",
                    file=self.stderr,
                    flush=True,
                )
        except Exception:
            return

    def _print_delta_overlay_summary(
        self,
        seq: int,
        execute_limit: int,
        projected_delta: dict[str, list[list[float]] | None],
    ) -> None:
        """Debug: summarize conditioned local action deltas sent in the chunk overlay.

        The rows here are the same conditioned/clamped deltas used to integrate
        the absolute overlay path, not raw model outputs. Deltas are per policy
        frame, so the print intentionally reports displacement units, not rates.
        """
        if not getattr(self, "_print_delta_overlay_enabled", False):
            return
        try:
            parts: list[str] = []
            for arm, tag in (("left", "L"), ("right", "R")):
                rows = projected_delta.get(arm)
                if not rows:
                    parts.append(f"{tag} delta_overlay=none")
                    continue
                arr = np.asarray(rows, dtype=np.float64)
                n = min(max(int(execute_limit), 0), arr.shape[0], 3)
                if n <= 0:
                    parts.append(f"{tag} delta_overlay=none")
                    continue
                window = arr[:n]
                lin_norm_mm = np.linalg.norm(window[:, :3], axis=1) * 1000.0
                ang_norm_deg = np.degrees(np.linalg.norm(window[:, 3:6], axis=1))
                yaw_deg = np.degrees(window[:, 5])

                def fmt(values: np.ndarray) -> str:
                    return " ".join(f"{float(value):+.2f}" for value in values)

                parts.append(
                    f"{tag} rows={n}/{arr.shape[0]} "
                    f"lin_norm_mm=[{fmt(lin_norm_mm)}] "
                    f"ang_norm_deg=[{fmt(ang_norm_deg)}] "
                    f"yaw_deg=[{fmt(yaw_deg)}]"
                )
            print(
                f"[flow-infer] delta_overlay seq={seq} execute_limit={execute_limit} "
                + "  |  ".join(parts),
                file=self.stderr,
                flush=True,
            )
        except Exception:
            return

    def _publish_chunk_overlay(self, now_monotonic: float) -> None:
        _ = now_monotonic
        publisher = getattr(self, "_chunk_overlay_publisher", None)
        chunk = getattr(self, "_chunk", None)
        payload = getattr(self, "_last_overlay_payload", None)
        if publisher is None or chunk is None or payload is None:
            return
        try:
            execute_limit = self._current_chunk_execute_limit()
            if execute_limit <= 0:
                return
            runway_steps = self._current_chunk_overlay_runway_steps()
            overlay_rows = np.asarray(chunk[:execute_limit], dtype=np.float64)
            if str(getattr(self, "chunk_stitch_mode", "boundary")) == "ensemble":
                sched = getattr(self, "_chunk_ensemble", None)
                runway = (
                    np.asarray(sched.runway_segment(length=runway_steps), dtype=np.float64)
                    if sched is not None and runway_steps > 0
                    else np.zeros((0, 14), dtype=np.float64)
                )
                if runway.size > 0:
                    overlay_rows = np.concatenate([overlay_rows, runway], axis=0)
            else:
                # Boundary runway: publish rows past the Python execution limit so
                # the servo-side follower can use reserve_steps when the next
                # chunk is late. Execution, gripper dispatch, and chain anchoring
                # still use execute_limit; the extra rows are follower-feed only.
                extra = min(int(execute_limit) + int(runway_steps), int(len(chunk)))
                if extra > execute_limit:
                    overlay_rows = np.asarray(chunk[:extra], dtype=np.float64)
            execute_tail_index = max(0, min(int(execute_limit), int(overlay_rows.shape[0])) - 1)
            projected: dict[str, list[list[float]] | None] = {"left": None, "right": None}
            projected_delta: dict[str, list[list[float]] | None] = {"left": None, "right": None}
            for arm, idx, sl in self._foh_arm_indices():
                if len(self.arm_mask) <= idx or self.arm_mask[idx] <= 0.0:
                    continue
                grip_index = 6 if arm == "left" else 13
                anchor_mode = self._chunk_anchor_source()
                if anchor_mode == "chain":
                    # Pure plan-chain integration: anchor on the PREVIOUS chunk's
                    # last executed-window target (its own integrated value), not
                    # on any robot state. Boundary shortfall (unconverged robot)
                    # carries over instead of being discarded each chunk — fixes
                    # the asymptotic-undershoot (Zeno) pattern of state anchoring.
                    # First chunk (no chain yet) seeds from the command pose.
                    chain_prev = getattr(self, "_overlay_chain_prev", None) or {}
                    prev_tail = chain_prev.get(arm)
                    if prev_tail is not None:
                        measured_anchor = np.asarray(prev_tail, dtype=np.float64)
                    else:
                        measured_anchor = np.asarray(
                            pose_from_state_payload(payload, arm, source="command"),
                            dtype=np.float64,
                        )
                else:
                    measured_anchor = np.asarray(
                        pose_from_state_payload(
                            payload,
                            arm,
                            source="auto" if anchor_mode == "actual" else anchor_mode,
                        ),
                        dtype=np.float64,
                    )
                cur = measured_anchor
                arm_points: list[list[float]] = []
                arm_delta_points: list[list[float]] = []
                for i in range(int(overlay_rows.shape[0])):
                    delta = np.asarray(
                        self._condition_arm_delta(
                            arm,
                            overlay_rows[i][sl],
                            step_index=i,
                            update_prev=False,
                        ),
                        dtype=np.float64,
                    )
                    arm_delta_points.append(
                        [float(value) for value in delta[:6].tolist()]
                        + [float(overlay_rows[i][grip_index])]
                    )
                    cur = np.asarray(pose_compose_local(cur, delta), dtype=np.float64)
                    arm_points.append(
                        [float(value) for value in cur[:7].tolist()]
                        + [float(overlay_rows[i][grip_index])]
                    )
                projected[arm] = arm_points
                projected_delta[arm] = arm_delta_points
                if anchor_mode == "chain":
                    # Record this chunk's chain tail; promoted to the anchor when
                    # the NEXT chunk activates (re-publishing the same chunk is
                    # idempotent). Warn once if the plan chain drifts far from
                    # the measured pose (safety layers may be blocking it).
                    pending = getattr(self, "_overlay_chain_pending", None)
                    if pending is None:
                        pending = {"left": None, "right": None}
                        self._overlay_chain_pending = pending
                    # Keep plan-chain anchoring tied to the executable window,
                    # not the runway tail that is published only for the servo
                    # follower's late-frame reserve budget.
                    pending[arm] = np.asarray(arm_points[execute_tail_index][:7], dtype=np.float64)
                    try:
                        meas = np.asarray(pose_from_state_payload(payload, arm), dtype=np.float64)
                        drift = float(np.linalg.norm(measured_anchor[:3] - meas[:3]))
                        if drift > 0.15 and not getattr(self, "_overlay_chain_drift_warned", False):
                            self._overlay_chain_drift_warned = True
                            print(
                                f"[flow-infer] WARNING chain anchor drifted {drift * 1000:.0f}mm from the "
                                "measured pose — a safety layer may be blocking the plan",
                                flush=True,
                            )
                    except Exception:
                        pass
            if projected["left"] is None and projected["right"] is None:
                return
            self._chunk_overlay_seq = int(getattr(self, "_chunk_overlay_seq", 0)) + 1
            self._print_delta_overlay_summary(self._chunk_overlay_seq, execute_limit, projected_delta)
            publisher.publish(
                seq=self._chunk_overlay_seq,
                policy_dt_sec=float(self.policy_dt_sec or 0.0),
                left=projected["left"],
                right=projected["right"],
                left_delta=projected_delta["left"],
                right_delta=projected_delta["right"],
                host_time_ns=time.time_ns(),
                inference_timing=self._inference_timing_snapshot(),
                camera_diagnostics=self._camera_diagnostics_snapshot(),
                execute_steps=int(execute_limit),
                runway_steps=int(self._current_chunk_overlay_runway_steps()),
                chunk_metadata=dict(getattr(self, "_active_chunk_metadata", None) or {}),
            )
        except Exception:
            return

    def _ensure_chunk_ensemble(self):
        """Lazy ChunkEnsembleScheduler when --chunk-stitch-mode ensemble.
        None in the default boundary mode."""
        if str(getattr(self, "chunk_stitch_mode", "boundary")) != "ensemble":
            return None
        sched = getattr(self, "_chunk_ensemble", None)
        if sched is None:
            from .chunk_ensemble import ChunkEnsembleScheduler

            sched = ChunkEnsembleScheduler(
                int(getattr(self, "ensemble_period", 6)),
                int(self.action_horizon),
                blend_mode=str(getattr(self, "ensemble_blend_mode", "linear")),
            )
            self._chunk_ensemble = sched
            if not bool(getattr(self, "_chunk_ensemble_runway_echoed", False)):
                self._chunk_ensemble_runway_echoed = True
                print(
                    f"[flow-infer] chunk-stitch-mode=ensemble runway=+{sched.period} rows",
                    flush=True,
                )
        return sched

    def _ensemble_anchor_fn(self, payload: dict[str, Any]):
        # First-chunk / post-reset plan seed: the overlay chain tail (kept per
        # arm across InitMotion of the OTHER arm) -> command pose fallback.
        def anchor(arm: str) -> np.ndarray:
            chain_prev = getattr(self, "_overlay_chain_prev", None) or {}
            tail = chain_prev.get(arm)
            if tail is not None:
                return np.asarray(tail, dtype=np.float64)
            return np.asarray(
                pose_from_state_payload(payload, arm, source="command"), dtype=np.float64
            )

        return anchor

    def _ensemble_wall_now(self) -> int:
        sched = getattr(self, "_chunk_ensemble", None)
        if sched is None:
            return 0
        active_len = int(self._chunk.shape[0]) if self._chunk is not None else 0
        return int(sched.window_start) - active_len + int(self._chunk_index)

    def _state_anchor_source(self) -> str:
        # pose_from_state_payload-safe variant: in chain mode the per-tick
        # command path is expected to run reanchor_mode=last_emitted_continuous
        # (its own plan chain); its residual measured_anchor uses (first chunk /
        # legacy fallbacks) map to the command pose.
        src = self._chunk_anchor_source()
        if src == "chain":
            return "command"
        # "actual" means the robot's effective tracking state. In physical-real
        # this resolves to tcp_actual. In rbpodo pgmode simulation it resolves to
        # tcp_ref, matching the server-side controller-simulation feedback lane.
        return "auto" if src == "actual" else src

    def _chunk_anchor_source(self) -> str:
        # Where chunk-delta integration anchors: "actual" (measured pose,
        # FK(q_actual) — legacy default) or "command" (FK(q_sent), the pose the
        # server actually commanded). Set by the runner (--chunk-anchor-source).
        # Scope: chunk integration + overlay/follower frames ONLY — proprio,
        # reset poses, tracking logs, and external-move re-anchor stay measured.
        return str(getattr(self, "chunk_anchor_source", "actual"))

    def _stream_prefetch_at(self) -> int:
        limit = self._current_chunk_execute_limit()
        # Sequential (blocking) chunk mode, set by the runner for step-by-step
        # verification: NEVER kick inference mid-chunk. The chunk is consumed to
        # the end, the boundary stall path then requests inference with the
        # freshest observation (the robot deliberately holds still for the
        # inference latency), and the new chunk re-anchors to the measured pose
        # at activation. chunk_index < limit always, so returning limit disables
        # the early kick entirely.
        if bool(getattr(self, "sequential_stream_inference", False)):
            return limit
        sched = getattr(self, "_chunk_ensemble", None)
        if sched is not None:
            # Ensemble: first (2R) window kicks at R; steady windows at start.
            return int(sched.kick_index_for_active_window())
        # Explicit kick point (RTC pairing): kick the next inference when this
        # many steps of the window are consumed. With RTC, inference_delay should
        # equal (execute_steps - stream_prefetch_at) — the steps that will run on
        # the OLD plan while the new chunk is inpainted (its frozen prefix).
        explicit = getattr(self, "stream_prefetch_at", None)
        if explicit is not None:
            return int(max(0, min(int(explicit), limit - 1)))
        # Kick the next inference EARLY (after ~1/4 of the chunk) so it has the most wall-clock
        # to finish before the executable window drains -> fewer boundary stalls (= fewer
        # pauses in the motion). Was limit//2 (half-chunk lead); a slow medoid/remote inference
        # could not finish in that window and stalled every chunk. Trade-off: the next chunk is
        # inferred from a slightly older frame (~quarter-chunk), but it re-anchors to the current
        # pose at its boundary, so only the IMAGE is marginally staler.
        return max(0, min(2, limit - 1))

    def _stream_hold_intent(self) -> CommandIntent | None:
        # Stall (next chunk not ready): re-emit the LAST absolute TcpPoseTarget so the
        # command stays fresh and the arm holds steady at the target. Returning None
        # here lets the command go stale; holding the last target until the next
        # chunk boundary gives a smooth pause instead of a stale-command jerk.
        return getattr(self, "_current_step_intent", None)

    def _apply_rotation_axis_mask(self, chunk: np.ndarray) -> np.ndarray:
        """Zero the per-arm rotation-action axes disabled in
        ``self.rotation_axes_enabled`` (rx, ry, rz at action indices 3/4/5 and
        10/11/12) so the arms hold those orientation components. Translation
        (dxyz) and gripper dims are left untouched. No-op when all three axes
        are enabled."""
        keep = getattr(self, "rotation_axes_enabled", (True, True, True))
        if all(keep):
            return chunk
        masked = np.array(chunk, copy=True)
        for offset in (0, 7):  # left arm at 0, right arm at 7
            for axis, enabled in enumerate(keep):  # axis 0=rx, 1=ry, 2=rz
                if not enabled:
                    masked[..., offset + 3 + axis] = 0.0
        return masked

    def _sample_and_align_chunk(self, payload: dict[str, Any]) -> np.ndarray | None:
        chunk = self._sample_chunk(payload)
        if chunk is None:
            return None
        if self.ee_local_r_align is not None:
            chunk = rotate_flow_arm_vectors(chunk, self.ee_local_r_align.T)
        return self._apply_rotation_axis_mask(chunk)

    def _sample_and_align_chunk_timed(self, payload: dict[str, Any]) -> np.ndarray | None:
        """Measure one inline local/remote inference without changing its behavior."""
        inference_seq, request_ns = self._begin_inference_request()
        worker_start_ns = self._inference_now_ns()
        observation_step_seq = int(getattr(self, "_stream_emitted_policy_steps", 0))
        chunk: np.ndarray | None = None
        try:
            chunk = self._sample_and_align_chunk(payload)
            return chunk
        finally:
            worker_end_ns = self._inference_now_ns()
            ready_ns = self._inference_now_ns()
            timing = self._record_inference_completion(
                inference_seq=inference_seq,
                request_ns=request_ns,
                worker_start_ns=worker_start_ns,
                worker_end_ns=worker_end_ns,
                ready_ns=ready_ns,
                succeeded=chunk is not None,
            )
            self._stream_activation_candidate_timing = timing if chunk is not None else None
            self._stream_activation_candidate_metadata = (
                {
                    "checkpoint_id": str(getattr(self, "checkpoint_id", "unknown")),
                    "inference_seq": int(inference_seq),
                    "observation_step_seq": observation_step_seq,
                    "observation_bundle_seq": getattr(self, "_last_obs_camera_seq", None),
                    "proprio": copy.deepcopy(
                        getattr(self, "_last_velproprio_diagnostics", None)
                    ),
                }
                if chunk is not None
                else None
            )

    def _ensure_stream_state(self) -> None:
        if getattr(self, "_stream_inited", False):
            return
        self._stream_inited = True
        self._step_deadline = 0.0
        self._current_step_intent = None
        self._stream_next_chunk = None
        self._stream_next_chunk_metadata = None
        self._stream_activation_candidate_metadata = None
        self._stream_pending = False
        self._stream_shutdown = False
        self._stream_request = None
        self._stream_inflight_generation = None
        self._stream_stall_count = 0
        self._stream_generation = 0
        if not hasattr(self, "_inference_timing_lock"):
            self._init_inference_timing_state()
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
                request = self._stream_request
                self._stream_request = None
                if isinstance(request, tuple) and len(request) >= 1:
                    self._stream_inflight_generation = int(request[0])
                else:
                    self._stream_inflight_generation = int(
                        getattr(self, "_stream_generation", 0)
                    )
            if isinstance(request, tuple) and len(request) == 4:
                generation, inference_seq, request_ns, payload = request
            elif isinstance(request, tuple) and len(request) == 2:
                generation, payload = request
                inference_seq = 0
                request_ns = self._inference_now_ns()
            else:
                generation, payload = int(getattr(self, "_stream_generation", 0)), request
                inference_seq = 0
                request_ns = self._inference_now_ns()
            worker_start_ns = self._inference_now_ns()
            observation_step_seq = int(getattr(self, "_stream_emitted_policy_steps", 0))
            try:
                chunk = self._sample_and_align_chunk(payload)
            except Exception as exc:  # noqa: BLE001 - inference must not kill the worker
                print(
                    f"{self.policy_label} prefetch failed: {type(exc).__name__}: {exc}",
                    file=self.stderr,
                    flush=True,
                )
                chunk = None
            worker_end_ns = self._inference_now_ns()
            ready_ns = self._inference_now_ns()
            with self._stream_lock:
                if int(generation) != int(getattr(self, "_stream_generation", 0)):
                    # Contact/reset invalidated this request while it was in flight.
                    # Do not let its late completion clear or overwrite a newer
                    # generation's pending request, chunk, or timing telemetry.
                    self._stream_inflight_generation = None
                    self._on_stale_inference_completion()
                    continue
                timing = self._record_inference_completion(
                    inference_seq=int(inference_seq),
                    request_ns=int(request_ns),
                    worker_start_ns=worker_start_ns,
                    worker_end_ns=worker_end_ns,
                    ready_ns=ready_ns,
                    succeeded=chunk is not None,
                )
                self._stream_next_chunk = chunk
                self._stream_next_chunk_metadata = (
                    {
                        "checkpoint_id": str(getattr(self, "checkpoint_id", "unknown")),
                        "inference_seq": int(inference_seq),
                        "observation_step_seq": observation_step_seq,
                        "observation_bundle_seq": getattr(self, "_last_obs_camera_seq", None),
                        "proprio": copy.deepcopy(
                            getattr(self, "_last_velproprio_diagnostics", None)
                        ),
                    }
                    if chunk is not None
                    else None
                )
                self._stream_ready_timing = timing if chunk is not None else None
                self._stream_pending = False
                self._stream_inflight_generation = None

    def _request_prefetch(self, payload: dict[str, Any]) -> None:
        with self._stream_cv:
            if self._stream_pending or self._stream_next_chunk is not None:
                return
            self._stream_pending = True
            # Snapshot the state payload for the worker so a mutable camera/state
            # object cannot be changed by the command loop while inference is reading it.
            payload_snapshot = copy.deepcopy(payload)
            inference_seq, request_ns = self._begin_inference_request()
            self._stream_request = (
                int(getattr(self, "_stream_generation", 0)),
                inference_seq,
                request_ns,
                payload_snapshot,
            )
            self._stream_cv.notify()

    def _take_prefetched(self) -> np.ndarray | None:
        with self._stream_lock:
            chunk = self._stream_next_chunk
            self._stream_next_chunk = None
            if chunk is not None:
                self._stream_activation_candidate_timing = self._stream_ready_timing
                self._stream_activation_candidate_metadata = getattr(
                    self, "_stream_next_chunk_metadata", None
                )
            self._stream_next_chunk_metadata = None
            self._stream_ready_timing = None
        return chunk

    def _init_inference_timing_state(self) -> None:
        self._inference_seq = 0
        self._inference_last_worker_start_ns = 0
        self._inference_timing_lock = threading.Lock()
        self._inference_timing_latest: dict[str, object] | None = None
        self._inference_timing_history = {
            name: deque(maxlen=64)
            for name in (
                "queue_wait_ms",
                "inference_latency_ms",
                "ready_wait_ms",
                "inference_period_ms",
                "inference_period_jitter_ms",
            )
        }
        self._stream_ready_timing: dict[str, object] | None = None
        self._stream_activation_candidate_timing: dict[str, object] | None = None
        self._inference_diagnostics_events: deque[dict[str, object]] = deque(maxlen=64)
        self._inference_diagnostics_total = 0

    def _inference_now_ns(self) -> int:
        clock = getattr(self, "_inference_clock_ns", None)
        return int(clock()) if callable(clock) else time.monotonic_ns()

    def _begin_inference_request(self) -> tuple[int, int]:
        if not hasattr(self, "_inference_timing_lock"):
            self._init_inference_timing_state()
        with self._inference_timing_lock:
            self._inference_seq = int(getattr(self, "_inference_seq", 0)) + 1
            return self._inference_seq, self._inference_now_ns()

    def _record_inference_completion(
        self,
        *,
        inference_seq: int,
        request_ns: int,
        worker_start_ns: int,
        worker_end_ns: int,
        ready_ns: int,
        succeeded: bool | None = None,
    ) -> dict[str, object]:
        queue_wait_ms = max(0.0, (worker_start_ns - request_ns) / 1_000_000.0)
        inference_latency_ms = max(0.0, (worker_end_ns - worker_start_ns) / 1_000_000.0)
        previous_start_ns = int(getattr(self, "_inference_last_worker_start_ns", 0))
        inference_period_ms = (
            max(0.0, (worker_start_ns - previous_start_ns) / 1_000_000.0)
            if previous_start_ns > 0
            else None
        )
        timing: dict[str, object] = {
            "seq": int(inference_seq),
            "request_monotonic_ns": int(request_ns),
            "worker_start_monotonic_ns": int(worker_start_ns),
            "worker_end_monotonic_ns": int(worker_end_ns),
            "chunk_ready_monotonic_ns": int(ready_ns),
            "activation_monotonic_ns": None,
            "queue_wait_ms": queue_wait_ms,
            "inference_latency_ms": inference_latency_ms,
            "ready_wait_ms": None,
            "inference_period_ms": inference_period_ms,
            "inference_period_nominal_ms": None,
            "inference_period_jitter_ms": None,
        }
        event: dict[str, object] = {
            "seq": int(inference_seq),
            "succeeded": succeeded,
            "timing": dict(timing),
            "camera": self._camera_diagnostics_snapshot(),
        }
        lock = getattr(self, "_inference_timing_lock", None)
        if lock is None:
            return timing
        with lock:
            self._inference_last_worker_start_ns = int(worker_start_ns)
            self._inference_timing_history["queue_wait_ms"].append(queue_wait_ms)
            self._inference_timing_history["inference_latency_ms"].append(inference_latency_ms)
            if inference_period_ms is not None:
                self._inference_timing_history["inference_period_ms"].append(inference_period_ms)
                periods = sorted(self._inference_timing_history["inference_period_ms"])
                middle = len(periods) // 2
                nominal_ms = (
                    periods[middle]
                    if len(periods) % 2 == 1
                    else 0.5 * (periods[middle - 1] + periods[middle])
                )
                jitter_ms = abs(inference_period_ms - nominal_ms)
                jitter_history = self._inference_timing_history["inference_period_jitter_ms"]
                jitter_history.clear()
                jitter_history.extend(abs(period_ms - nominal_ms) for period_ms in periods)
                timing["inference_period_nominal_ms"] = nominal_ms
                timing["inference_period_jitter_ms"] = jitter_ms
            self._inference_timing_latest = dict(timing)
            self._inference_diagnostics_total += 1
            event["timing"] = dict(timing)
            self._inference_diagnostics_events.append(event)
        return timing

    def _record_inference_activation(self, now_monotonic: float | None = None) -> None:
        timing = getattr(self, "_stream_activation_candidate_timing", None)
        if not isinstance(timing, dict):
            return
        activation_ns = (
            self._inference_now_ns()
            if now_monotonic is None
            else int(max(0.0, float(now_monotonic)) * 1_000_000_000.0)
        )
        ready_ns = int(timing.get("chunk_ready_monotonic_ns", activation_ns) or activation_ns)
        # Inline fallback inference starts after the command loop captured
        # now_monotonic, so that supplied value can predate chunk readiness.
        if activation_ns < ready_ns:
            activation_ns = self._inference_now_ns()
        ready_wait_ms = max(0.0, (activation_ns - ready_ns) / 1_000_000.0)
        activated = dict(timing)
        activated["activation_monotonic_ns"] = activation_ns
        activated["ready_wait_ms"] = ready_wait_ms
        lock = getattr(self, "_inference_timing_lock", None)
        if lock is not None:
            with lock:
                self._inference_timing_history["ready_wait_ms"].append(ready_wait_ms)
                self._inference_timing_latest = activated
                for event in reversed(self._inference_diagnostics_events):
                    if int(event.get("seq", -1)) == int(activated.get("seq", -2)):
                        event["timing"] = dict(activated)
                        break
        self._stream_activation_candidate_timing = None

    def _camera_diagnostics_snapshot(self) -> dict[str, object] | None:
        details = getattr(self, "_last_inference_camera_diagnostics", None)
        snapshot: dict[str, object] = dict(details) if isinstance(details, dict) else {}
        velproprio = getattr(self, "_last_velproprio_diagnostics", None)
        if isinstance(velproprio, dict):
            snapshot["velocity_proprio"] = copy.deepcopy(velproprio)
        camera_client = getattr(self, "camera_client", None)
        client_snapshot = getattr(camera_client, "diagnostics_snapshot", None)
        if callable(client_snapshot):
            try:
                snapshot["client"] = client_snapshot()
            except Exception:  # noqa: BLE001 - diagnostics are best effort.
                pass
        writer_snapshot = getattr(getattr(self, "_diagnostic_image_writer", None), "snapshot", None)
        if callable(writer_snapshot):
            try:
                snapshot["image_snapshots"] = writer_snapshot()
            except Exception:  # noqa: BLE001 - diagnostics are best effort.
                pass
        return snapshot or None

    def inference_diagnostics_snapshot(self) -> dict[str, object]:
        if not hasattr(self, "_inference_timing_lock"):
            self._init_inference_timing_state()
        with self._inference_timing_lock:
            events = [copy.deepcopy(event) for event in self._inference_diagnostics_events]
            return {
                "total_inferences": int(self._inference_diagnostics_total),
                "retained_inferences": len(events),
                "rolling_window": 64,
                "events": events,
                "latest_camera": self._camera_diagnostics_snapshot(),
            }

    @staticmethod
    def _inference_metric_summary(values: object) -> dict[str, float | int] | None:
        samples = [float(value) for value in values] if values is not None else []
        if not samples:
            return None
        ordered = sorted(samples)
        p95_index = max(0, min(len(ordered) - 1, int(np.ceil(0.95 * len(ordered))) - 1))
        return {
            "samples": len(ordered),
            "p95_ms": ordered[p95_index],
            "max_ms": ordered[-1],
        }

    def _inference_timing_snapshot(self) -> dict[str, object] | None:
        lock = getattr(self, "_inference_timing_lock", None)
        if lock is None:
            return None
        with lock:
            latest = getattr(self, "_inference_timing_latest", None)
            if not isinstance(latest, dict):
                return None
            history = {
                name: self._inference_metric_summary(values)
                for name, values in self._inference_timing_history.items()
            }
            jitter_summary = history.get("inference_period_jitter_ms")
            return {
                **latest,
                "stall_count": int(getattr(self, "_stream_stall_count", 0)),
                "rolling_window": 64,
                "rolling": {name: summary for name, summary in history.items() if summary is not None},
                "inference_period_jitter": {
                    "definition": "abs(start_to_start_period_ms - rolling_median_period_ms)",
                    "nominal_ms": latest.get("inference_period_nominal_ms"),
                    "last_ms": latest.get("inference_period_jitter_ms"),
                    "p95_ms": None if jitter_summary is None else jitter_summary["p95_ms"],
                    "max_ms": None if jitter_summary is None else jitter_summary["max_ms"],
                    "samples": 0 if jitter_summary is None else jitter_summary["samples"],
                },
            }

    @property
    def gripper_command_count(self) -> int:
        return int(self.gripper_runtime.command_count)

    @property
    def gripper_dropped_count(self) -> int:
        return int(self.gripper_runtime.dropped_count)

    def _gripper_close_bias(self, arm: str | None = None) -> float:
        """Percent subtracted from the ABSOLUTE gripper opening target so grasps
        close more firmly (e.g. 1.0 turns an 18% command into 17%). 0 in delta
        mode (the action is a relative change, biasing it would compound), and 0
        in binary mode (the value snaps to the open/close presets).

        Resolved PER ARM: an `arm` of "left"/"right" uses that arm's override
        (`gripper_close_bias_left` / `_right`) when set, else the shared
        `gripper_close_bias`. arm=None (or an unset override) uses the shared base."""
        if not getattr(self, "gripper_action_absolute", True):
            return 0.0
        if getattr(self, "gripper_binary", False):
            return 0.0
        if arm is not None:
            override = getattr(self, f"gripper_close_bias_{arm}", None)
            if override is not None:
                return float(override or 0.0)
        return float(getattr(self, "gripper_close_bias", 0.0) or 0.0)

    def _map_gripper_opening(self, raw_percent: float, arm: str | None = None) -> float:
        """Map the model's raw gripper-action opening percent (already in percent
        units) to the opening percent to command. Used by both gripper sinks
        (motion-packet target and serial-backend dispatch) in the target-command
        modes (absolute / binary); DELTA never calls this.

        binary: the checkpoint was trained with a binarized open/close gripper
        (e.g. openpi `binary 25`), so the model output is bimodal -- threshold it
        and snap to the physical open/close presets (`--gripper-open-percent` /
        `--gripper-close-percent`). absolute: pass the opening through, minus the
        per-arm close-bias (`arm`-resolved). Both clamp to the [0, 100] opening range."""
        if getattr(self, "gripper_binary", False):
            threshold = float(getattr(self, "gripper_binary_threshold", 50.0))
            target = (
                float(getattr(self, "gripper_open_percent", 50.0))
                if float(raw_percent) >= threshold
                else float(getattr(self, "gripper_close_percent", 7.0))
            )
            return float(np.clip(target, 0.0, 100.0))
        mapped = float(np.clip(float(raw_percent) - self._gripper_close_bias(arm), 0.0, 100.0))
        # Close-snap deadzone: collapse a near-closed opening to fully closed so
        # small policy jitter near the closed end doesn't leave the jaw cracked
        # open. 0 = off; absolute (non-binary) mode only.
        snap = float(getattr(self, "gripper_close_snap_percent", 0.0) or 0.0)
        if snap > 0.0 and mapped < snap:
            return 0.0
        return mapped

    def _gripper_hold_open_value(self) -> float:
        """Opening percent commanded while holding the gripper open during the
        reach-before-grasp window: the binary OPEN preset (so binary stays on its
        two physical levels), otherwise fully open."""
        if getattr(self, "gripper_binary", False):
            return float(np.clip(getattr(self, "gripper_open_percent", 50.0), 0.0, 100.0))
        return 100.0

    def _dispatch_gripper_step(self, step: np.ndarray) -> None:
        # RAW action debug is terminal-heavy; keep it opt-in so live flow-infer
        # motion is not paced by stdout. Enable with POLICY_RUNNER_PRINT_RAW_ACTIONS=1.
        _raw = np.asarray(step, dtype=np.float64).reshape(-1)
        if os.environ.get("POLICY_RUNNER_PRINT_RAW_ACTIONS", "") == "1" and _raw.shape[0] > 13:
            # Post-threshold commanded gripper value: replicate exactly what the
            # dispatch below sends for each gripper -- the reach-before-grasp
            # hold-open value if active, else the binary threshold / absolute
            # clip mapping (_map_gripper_opening).
            def _grip_cmd_str(raw_grip: float, arm: str) -> str:
                if not getattr(self, "gripper_action_absolute", True):
                    return " -> delta (no threshold)"
                if bool(getattr(self, "_gripper_hold_open_now", False)):
                    grip_cmd = self._gripper_hold_open_value()
                else:
                    grip_cmd = self._map_gripper_opening(float(raw_grip), arm)
                if getattr(self, "gripper_binary", False):
                    thr = float(getattr(self, "gripper_binary_threshold", 50.0))
                    decision = "OPEN" if float(raw_grip) >= thr else "CLOSE"
                    return f" -> {decision} {grip_cmd:.1f}% (thr={thr:g})"
                return f" -> cmd={grip_cmd:.1f}%"

            for _arm, _v in (("R", _raw[7:14]), ("L", _raw[0:7])):
                _arm_name = "right" if _arm == "R" else "left"
                print(
                    f"[flow-infer] RAW {_arm} action (pre-threshold) "
                    f"dxyz=[{_v[0]:+.4f} {_v[1]:+.4f} {_v[2]:+.4f}] "
                    f"rxyz=[{_v[3]:+.4f} {_v[4]:+.4f} {_v[5]:+.4f}] "
                    f"grip={_v[6]:+.2f}{_grip_cmd_str(float(_v[6]), _arm_name)}",
                    flush=True,
                )
        # ABSOLUTE/BINARY checkpoints emit an opening target -> drive the gripper
        # backend as a "target" (set the opening directly); legacy DELTA
        # checkpoints emit a per-step change -> "delta" (accumulate on the motor).
        # This serial-backend dispatch drives the PHYSICAL gripper in real_policy
        # (PikaSerialGripperBackend), so the binary open/close mapping and the
        # reach-before-grasp hold-open must be applied here too (not only on the
        # motion-packet target in _integrate_gripper_targets).
        command_type = "target" if getattr(self, "gripper_action_absolute", True) else "delta"
        if command_type == "target":
            # _gripper_hold_open_now is set for THIS step by _integrate_gripper_targets,
            # which always runs immediately before dispatch.
            hold_open = bool(getattr(self, "_gripper_hold_open_now", False))
            hold_value = self._gripper_hold_open_value()
            step = step.copy()
            for idx in (6, 13):  # left, right gripper dims in the 14-D action step
                if step.shape[0] > idx:
                    arm = "left" if idx == 6 else "right"
                    step[idx] = (
                        hold_value
                        if hold_open
                        else self._map_gripper_opening(float(step[idx]), arm)
                    )
        commands = gripper_commands_from_flow_step(
            step.tolist(),
            arm_mask=self.arm_mask.tolist(),
            command_type=command_type,
            source=self.gripper_command_source,
        )
        self.gripper_runtime.dispatch(commands)

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
            tcp_target_profile="flow_infer_smooth",
            metadata={
                "action_source": "flow_infer",
                "source_conditioning_mode": getattr(self, "_tcp_tp_mode", "legacy_step_hold"),
            },
        )

    def _apply_chunk_crossfade(
        self,
        arm: str,
        twist: tuple[float, ...] | list[float] | np.ndarray,
        *,
        step_index: int | None = None,
    ) -> tuple[float, ...]:
        """Blend the first N action deltas of a new chunk from the prior chunk's
        last emitted delta. This removes velocity discontinuities at chunk
        boundaries without adding steady-state lag after the short ramp window.
        """
        current = tuple(float(v) for v in np.asarray(twist, dtype=np.float64).reshape(-1)[:6].tolist())
        k = int(getattr(self, "_chunk_crossfade_steps", 0) or 0)
        prev_by_arm = getattr(self, "_prev_emitted_twist_by_arm", {}) or {}
        prev = prev_by_arm.get(arm) if isinstance(prev_by_arm, dict) else None
        idx = int(getattr(self, "_steps_since_boundary", 0) if step_index is None else step_index)
        if k <= 0 or prev is None or idx >= k:
            return current
        prev_tuple = tuple(float(v) for v in np.asarray(prev, dtype=np.float64).reshape(-1)[:6].tolist())
        alpha = float(idx + 1) / float(k + 1)
        return tuple((1.0 - alpha) * prev_tuple[i] + alpha * current[i] for i in range(6))

    def _condition_arm_delta(
        self,
        arm: str,
        delta: np.ndarray,
        *,
        step_index: int | None = None,
        update_prev: bool = False,
    ) -> tuple[float, ...]:
        clamped = self._clamp_delta_step(delta)
        emitted = self._apply_chunk_crossfade(arm, clamped, step_index=step_index)
        if update_prev:
            prev_by_arm = getattr(self, "_prev_emitted_twist_by_arm", None)
            if isinstance(prev_by_arm, dict):
                k = int(getattr(self, "_chunk_crossfade_steps", 0) or 0)
                prev = prev_by_arm.get(arm)
                idx = int(getattr(self, "_steps_since_boundary", 0) if step_index is None else step_index)
                # Keep the old chunk's final delta as the fixed crossfade anchor
                # during the ramp; once past the ramp, track the current emitted delta
                # so the next chunk boundary starts from the true last velocity.
                if prev is None or k <= 0 or idx >= k:
                    prev_by_arm[arm] = emitted
        return emitted

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
            targets[arm] = pose_from_state_payload(payload, arm, source=self._state_anchor_source())
        conditioned_delta = self._condition_arm_delta(arm, delta, update_prev=True)
        targets[arm] = pose_compose_local(targets[arm], np.asarray(conditioned_delta, dtype=np.float32))
        return tuple(float(value) for value in targets[arm].tolist())

    def _clear_target_pose_state(
        self,
        arms: tuple[str, ...] | list[str] | None = None,
        *,
        preserve_gripper_targets: bool = False,
    ) -> None:
        selected = tuple(arms or ("left", "right"))
        targets = getattr(self, "_target_pose_by_arm", None)
        if targets is not None:
            for arm in selected:
                if arm in targets:
                    targets[arm] = None
        gripper_targets = getattr(self, "_gripper_targets_by_arm", None)
        if gripper_targets is not None and not preserve_gripper_targets:
            for arm in selected:
                if arm in gripper_targets:
                    gripper_targets[arm] = None
        conds = getattr(self, "_tcp_tp_conditioners", None)
        if isinstance(conds, dict):
            for arm in selected:
                cond = conds.get(arm)
                if cond is not None:
                    cond.reset()

    def _pose_snapshot_for_arms(self, arms: tuple[str, ...]) -> dict[str, list[float] | None]:
        targets = getattr(self, "_target_pose_by_arm", None)
        out: dict[str, list[float] | None] = {}
        for arm in arms:
            pose = targets.get(arm) if isinstance(targets, dict) else None
            out[arm] = None if pose is None else [float(v) for v in np.asarray(pose).reshape(-1).tolist()]
        return out

    def _reanchor_arms_to_snapshot(
        self,
        arms: tuple[str, ...],
        snapshot: StateSnapshot,
    ) -> dict[str, list[float] | None]:
        payload = snapshot.payload
        targets = getattr(self, "_target_pose_by_arm", None)
        if targets is None:
            self._target_pose_by_arm = {"left": None, "right": None}
            targets = self._target_pose_by_arm
        gripper_targets = getattr(self, "_gripper_targets_by_arm", None)
        conds = getattr(self, "_tcp_tp_conditioners", None)
        after: dict[str, list[float] | None] = {}
        chain_prev = getattr(self, "_overlay_chain_prev", None)
        chain_pending = getattr(self, "_overlay_chain_pending", None)
        for arm in arms:
            try:
                pose = np.asarray(pose_from_state_payload(payload, arm), dtype=np.float64)
            except Exception:
                after[arm] = None
                continue
            targets[arm] = pose.copy()
            if isinstance(gripper_targets, dict):
                gripper_targets[arm] = None
            if isinstance(conds, dict) and arm in conds:
                cond = conds[arm]
                cond.reset()
                try:
                    cond._prev_emitted = pose.copy()
                except Exception:
                    pass
            # Chain anchor (--chunk-anchor-source chain): the arm was moved
            # EXTERNALLY (InitMotion etc.), so the accumulated plan chain is
            # meaningless for it — drop it so the next chunk re-seeds from the
            # arm's new pose. Per-arm: the other arm's chain keeps accumulating.
            if isinstance(chain_prev, dict):
                chain_prev[arm] = None
            if isinstance(chain_pending, dict):
                chain_pending[arm] = None
            after[arm] = [float(v) for v in pose.reshape(-1).tolist()]
        # RTC guidance references the previous BOTH-ARM raw chunk; after an
        # external move it must not pull the new plan toward the pre-init plan.
        # Cold-start the next infer (vanilla). Also drops the velocity-proprio
        # previous-pose memory so the first post-init sample reads rest, not a
        # jump across the init teleport.
        reset_rtc = getattr(self, "reset_rtc", None)
        if callable(reset_rtc):
            try:
                reset_rtc()
            except Exception:
                pass
        return after

    def _invalidate_policy_chunks(self, *, reason: str) -> None:
        _ = reason
        sched = getattr(self, "_chunk_ensemble", None)
        if sched is not None:
            sched.reset()
        self._chunk = None
        self._chunk_index = 0
        self._steps_since_boundary = 0
        self._current_step_intent = None
        self._current_gripper_targets = {"left": None, "right": None}
        if hasattr(self, "_stream_lock"):
            try:
                with self._stream_lock:
                    self._stream_generation = int(getattr(self, "_stream_generation", 0)) + 1
                    self._stream_next_chunk = None
                    self._stream_next_chunk_metadata = None
                    self._stream_activation_candidate_metadata = None
                    self._stream_pending = False
                    self._stream_request = None
                    self._stream_ready_timing = None
                    self._stream_activation_candidate_timing = None
            except Exception:
                pass
        if hasattr(self, "_stream_cv"):
            try:
                with self._stream_cv:
                    self._stream_cv.notify_all()
            except Exception:
                pass

    @staticmethod
    def _command_source_ids(snapshot: StateSnapshot) -> tuple[str | None, str | None]:
        command_source = snapshot.payload.get("command_source", {})
        if not isinstance(command_source, dict):
            return None, None
        source_id = command_source.get("source_id") or command_source.get("active_source_id")
        session_id = command_source.get("session_id") or command_source.get("active_session_id")
        return (
            str(source_id) if source_id is not None else None,
            str(session_id) if session_id is not None else None,
        )

    def _log_arm_init_event(
        self,
        event: str,
        arms: tuple[str, ...],
        snapshot: StateSnapshot,
        *,
        before_mask: list[float] | None = None,
        after_mask: list[float] | None = None,
        chunk_invalidated: bool = False,
        reanchor_before: dict[str, list[float] | None] | None = None,
        reanchor_after: dict[str, list[float] | None] | None = None,
    ) -> None:
        log = self._action_log
        if log is None:
            return
        command_source_id, command_session_id = self._command_source_ids(snapshot)
        record = {
            "seq": self._action_log_seq,
            "t_mono": time.monotonic(),
            "event": event,
            "arms": list(arms),
            "runner_role": "flow_infer",
            "command_source_id": command_source_id,
            "command_session_id": command_session_id,
            "flow_chunk_id": int(getattr(self, "_tcp_tp_chunk_seq", -1)),
            "chunk_invalidated": bool(chunk_invalidated),
            "source_arm_mask_before": before_mask,
            "source_arm_mask_after": after_mask,
            "reanchor_pose_before": reanchor_before,
            "reanchor_pose_after": reanchor_after,
        }
        self._action_log_seq += 1
        log.write(json.dumps(record) + "\n")

    def log_final_intent(
        self,
        intent: CommandIntent | None,
        snapshot: StateSnapshot,
        *,
        arm_init_override: Any | None = None,
        decision_allowed: bool,
        sent: bool,
        command_seq: int = 0,
        drop_reason: str | None = None,
    ) -> None:
        """Append the post-arm-init, post-safety command view to the action JSONL."""
        log = self._action_log
        if log is None or intent is None:
            return

        def arm_mode(payload: dict[str, Any] | None) -> str | None:
            if not isinstance(payload, dict):
                return None
            mode = payload.get("mode")
            return None if mode is None else str(mode)

        def joint_profile(payload: dict[str, Any] | None) -> str | None:
            if not isinstance(payload, dict):
                return None
            value = payload.get("joint_target_profile")
            return None if value is None else str(value)

        command_source_id, command_session_id = self._command_source_ids(snapshot)
        record = {
            "seq": self._action_log_seq,
            "t_mono": time.monotonic(),
            "event": "final_intent",
            "command_seq": int(command_seq),
            "command_family": self.command_family,
            "left_mode": arm_mode(intent.left),
            "right_mode": arm_mode(intent.right),
            "left_joint_target_profile": joint_profile(intent.left),
            "right_joint_target_profile": joint_profile(intent.right),
            "arm_init_override_left": bool(getattr(arm_init_override, "left_on", False)),
            "arm_init_override_right": bool(getattr(arm_init_override, "right_on", False)),
            "decision_allowed": bool(decision_allowed),
            "sent": bool(sent),
            "drop_reason": drop_reason,
            "runner_role": "flow_infer",
            "command_source_id": command_source_id,
            "command_session_id": command_session_id,
        }
        self._action_log_seq += 1
        log.write(json.dumps(record) + "\n")

    # ----------------------------------------------- foh_se3 conditioning (Patch 3) --
    def _tcp_tp_foh_active(self) -> bool:
        """True only when the foh_se3 pose-target conditioner is selected."""
        return (
            getattr(self, "_tcp_tp_mode", "legacy_step_hold") == "foh_se3"
        )

    def _ensure_tcp_tp_conditioners(self) -> dict[str, "OnlineTcpPoseTargetConditioner"]:
        conds = getattr(self, "_tcp_tp_conditioners", None)
        if conds is None:
            assert self.policy_dt_sec is not None
            conds = {
                arm: OnlineTcpPoseTargetConditioner(
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
            measured_anchor = np.asarray(
                pose_from_state_payload(payload, arm, source=self._state_anchor_source()),
                dtype=np.float64,
            )
            last_emitted = cond.last_emitted
            measured: list[np.ndarray] = []
            continuous: list[np.ndarray] | None = [] if last_emitted is not None else None
            cur_m = measured_anchor
            cur_c = None if last_emitted is None else np.asarray(last_emitted, dtype=np.float64)
            for i in range(limit):
                delta = np.asarray(
                    self._condition_arm_delta(arm, chunk[i][sl], step_index=i, update_prev=False),
                    dtype=np.float64,
                )
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

    def _remember_emitted_deltas_for_step(self, step: np.ndarray, *, step_index: int | None = None) -> None:
        for arm, idx, sl in self._foh_arm_indices():
            if self.arm_mask[idx] > 0.0:
                self._condition_arm_delta(arm, step[sl], step_index=step_index, update_prev=True)

    def _foh_tick_intent(self, now_monotonic: float, *, stall: bool) -> CommandIntent | None:
        """Emit a fresh SE(3)-interpolated TcpPoseTarget for the current servo tick."""
        conds = self._ensure_tcp_tp_conditioners()
        gripper = getattr(self, "_current_gripper_targets", {"left": None, "right": None})
        results: dict[str, Any] = {}
        payload_arms: dict[str, tuple[float, ...] | None] = {"left": None, "right": None}
        for arm, idx, _sl in self._foh_arm_indices():
            if self.arm_mask[idx] <= 0.0:
                continue
            cond = conds[arm]
            if cond.last_emitted is None and stall:
                continue  # no chunk has produced a target yet -> emit nothing (no jump to origin)
            ct = cond.hold() if stall else cond.sample(now_monotonic)
            results[arm] = ct
            payload_arms[arm] = tuple(float(v) for v in ct.pose.tolist())
        self._foh_last_targets = results
        if payload_arms["left"] is None and payload_arms["right"] is None:
            return None
        return tcp_pose_target_stand_intent(
            left=payload_arms["left"],
            right=payload_arms["right"],
            left_gripper=gripper.get("left"),
            right_gripper=gripper.get("right"),
            timeout_sec=self.timeout_sec,
            tcp_target_profile="flow_infer_smooth",
            metadata={
                "action_source": "flow_infer",
                "source_conditioning_mode": getattr(self, "_tcp_tp_mode", "legacy_step_hold"),
            },
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
            "command_family": self.command_family,
            "tcp_target_profile": "flow_infer_smooth",
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
                })
            record[f"{arm}_conditioner"] = arm_rec
        self._action_log_seq += 1
        log.write(json.dumps(record) + "\n")

    def _clamp_delta_step(self, delta: np.ndarray) -> tuple[float, ...]:
        assert self.policy_dt_sec is not None
        return clamp_pose_delta(
            np.asarray(delta, dtype=np.float64).reshape(-1).tolist(),
            self.max_linear_velocity_m_s * self.policy_dt_sec,
            self.max_angular_velocity_rad_s * self.policy_dt_sec,
        )

    def _no_policy_input_intent(self) -> CommandIntent | None:
        self._clear_target_pose_state()
        return None

    def _integrate_gripper_targets(
        self,
        step: np.ndarray,
        payload: dict[str, Any],
    ) -> dict[str, float | None]:
        targets: dict[str, float | None] = {"left": None, "right": None}
        # Reach-before-grasp: hold the gripper OPEN for the first N executed policy
        # steps so the arm can reach toward the object before the policy is allowed
        # to close it (avoids a premature grasp at rollout start). Set via
        # `source.gripper_open_hold_steps`. The step counter and the per-step
        # hold-open flag are computed here (this method always runs once per
        # executed step, immediately before _dispatch_gripper_step) and the flag is
        # shared with dispatch so the PHYSICAL serial backend honours the hold too.
        open_hold_steps = int(getattr(self, "gripper_open_hold_steps", 0) or 0)
        step_idx = int(getattr(self, "_gripper_integrate_count", 0))
        self._gripper_integrate_count = step_idx + 1
        hold_open = step_idx < open_hold_steps
        self._gripper_hold_open_now = hold_open
        if not self._allow_gripper_targets_in_motion_packet():
            return targets
        hold_value = self._gripper_hold_open_value()
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
                    # ABSOLUTE/BINARY: the action dim IS the next-step opening;
                    # command it directly (never integrate). absolute applies the
                    # close-bias; binary snaps to the open/close presets. Clamped.
                    self._gripper_targets_by_arm[arm] = self._map_gripper_opening(
                        float(step[step_index]), arm
                    )
                else:
                    # DELTA (legacy): accumulate the per-step opening change.
                    self._gripper_targets_by_arm[arm] = float(
                        self._gripper_targets_by_arm[arm]
                    ) + float(step[step_index])
            if hold_open:
                # Pin to OPEN (binary preset, else fully open) AND reset the integrator
                # there, so when the hold ends the policy resumes closing from the open
                # state (not from a drifted accumulator).
                self._gripper_targets_by_arm[arm] = hold_value
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
        close_overlay = getattr(getattr(self, "_chunk_overlay_publisher", None), "close", None)
        if callable(close_overlay):
            close_overlay()
        close_diagnostics = getattr(getattr(self, "_diagnostic_image_writer", None), "close", None)
        if callable(close_diagnostics):
            close_diagnostics()
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
        if getattr(self, "_chunk_ensemble", None) is not None:
            # Ensemble windows are exactly the execute window (R or 2R rows).
            return int(self._chunk.shape[0])
        return min(int(self.chunk_execute_steps), int(self._chunk.shape[0]))

    def _current_chunk_overlay_runway_steps(self) -> int:
        return _resolve_chunk_overlay_runway_steps(
            getattr(self, "chunk_overlay_runway_steps", DEFAULT_CHUNK_OVERLAY_RUNWAY_STEPS)
        )

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

    def __init__(
        self,
        checkpoint_path: str | Path,
        *,
        timeout_sec: float = 0.2,
        camera_client: Any | None = None,
        policy_dt_sec: float | None = None,
        image_size: int | None = None,
        max_linear_velocity_m_s: float | None = None,
        max_angular_velocity_rad_s: float | None = None,
        max_linear_step_m: float = 0.002,
        max_angular_step_rad: float = 0.01,
        chunk_execute_steps: int | None = None,
        chunk_overlay_runway_steps: int = DEFAULT_CHUNK_OVERLAY_RUNWAY_STEPS,
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
        self.command_family = FLOW_COMMAND_LABEL
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
        self.chunk_overlay_runway_steps = _resolve_chunk_overlay_runway_steps(chunk_overlay_runway_steps)
        self.policy_dt_sec = _resolve_runtime_policy_dt_sec(
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
        self._action_log = _open_action_log(self.stderr)

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

    def __init__(
        self,
        report_path: str | Path,
        *,
        ensemble_name: str | None = "top5",
        timeout_sec: float = 0.2,
        camera_client: Any | None = None,
        policy_dt_sec: float | None = None,
        image_size: int | None = None,
        max_linear_velocity_m_s: float | None = None,
        max_angular_velocity_rad_s: float | None = None,
        max_linear_step_m: float = 0.002,
        max_angular_step_rad: float = 0.01,
        chunk_execute_steps: int | None = None,
        chunk_overlay_runway_steps: int = DEFAULT_CHUNK_OVERLAY_RUNWAY_STEPS,
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
        self.command_family = FLOW_COMMAND_LABEL
        self.camera_names = list(bundle.camera_names)
        self.image_size = int(bundle.image_size)
        self.action_horizon = int(bundle.action_horizon)
        self.chunk_execute_steps = _resolve_chunk_execute_steps(chunk_execute_steps, self.action_horizon)
        self.chunk_overlay_runway_steps = _resolve_chunk_overlay_runway_steps(chunk_overlay_runway_steps)
        self.policy_dt_sec = _resolve_runtime_policy_dt_sec(
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
        self._action_log = _open_action_log(self.stderr)

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
        command_family=FLOW_COMMAND_LABEL,
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
        command_family=FLOW_COMMAND_LABEL,
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
        command_family=FLOW_COMMAND_LABEL,
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
    if schema == PC_CHECKPOINT_SCHEMA:
        return "pc"
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
    if schema not in {FLOW_CHECKPOINT_SCHEMA, PC_CHECKPOINT_SCHEMA, IMITATION_CHECKPOINT_SCHEMA}:
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
    policy_dt_sec: float | None,
    stats: dict[str, Any],
) -> float | None:
    resolved = _positive_float(policy_dt_sec)
    if resolved is not None:
        return resolved
    resolved = _positive_float(stats.get("dt_mean_sec"))
    if resolved is not None:
        return resolved
    raise ValueError(
        "policy_dt_sec or checkpoint dataset_stats.dt_mean_sec is required for TcpPoseTarget flow commands"
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


def _resolve_chunk_overlay_runway_steps(value: int | None) -> int:
    resolved = int(DEFAULT_CHUNK_OVERLAY_RUNWAY_STEPS if value is None else value)
    if resolved < 0:
        raise ValueError("chunk_overlay_runway_steps must be non-negative")
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
            for key in (
                "target_percent",
                "percent",
                "gripper_position",
                "position",
                "target",
                "value",
            ):
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
