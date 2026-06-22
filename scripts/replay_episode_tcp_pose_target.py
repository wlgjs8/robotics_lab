#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np
import h5py

ROOT = Path(__file__).resolve().parent.parent
TOOLS = ROOT / "tools"
POLICY_RUNNER = ROOT / "policy_runner"
for _path in (str(ROOT), str(TOOLS), str(POLICY_RUNNER)):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from generate_replay_target import episode_id as hdf5_episode_id
from generate_replay_target import generate as generate_replay_npz
from generate_replay_target import git_commit as phase1_git_commit
from generate_replay_target import servo_times as phase1_servo_times
from generate_replay_target import build_npz_arrays as phase1_build_npz_arrays
from tcp_tuning.config import Config as Phase1Config
from tcp_tuning.config import apply_cli_overrides as apply_conditioning_cli_overrides
from tcp_tuning.config import load_config as load_tcp_tuning_config
from tcp_tuning.command_conditioner import CommandConditioner
from tcp_tuning.hdf5_io import EpisodeData
from tcp_tuning.hdf5_io import load_episode
from tcp_tuning.se3 import foh_pose, quat_canonical, twist_from_poses
from tcp_tuning.smoothing import split_segments
from tcp_tuning.trajectory_log import TrajectoryLogWriter

from policy_runner.action_sources.tcp_delta import tcp_pose_target_stand_intent
from policy_runner.flow_dataset import pose_compose_local, pose_delta_local
from policy_runner.umi_pipeline import convert_umi_episode
from policy_runner.robot_state_client import RobotStateClient, StateSnapshot, StateStreamLeaseReadback
from policy_runner.servo_command_client import CommandIntent, ServoCommandClient


ARMS = ("left", "right")
CONTROLLER_SIM_NOT_ACTIVATED_REASONS = frozenset(("robot_fault", "servo_disabled"))
_TOLERATED_CONTROLLER_SIM_ARM_ERRORS: set[str] = set()
DEFAULT_SERVER_CONFIG_CANDIDATES = (
    ROOT / "rb_servo_server" / "config" / "local" / "stack_real.yaml",
    ROOT / "rb_servo_server" / "config" / "local" / "stack_real_replay.yaml",
    ROOT / "rb_servo_server" / "config" / "dual_real.example.yaml",
)
DEFAULT_MODE = "clean_foh_se3"
FALLBACK_MAX_LINEAR_SPEED_M_S = 0.03
FALLBACK_MAX_ANGULAR_SPEED_RAD_S = 0.25
DEFAULT_RETARGET_CONFIG = ROOT / "calibration" / "umi_retarget_eelocal.yaml"
# MEASURED pika-UMI correction (axis-probe 2026-06-15, Kabsch 5/6 residual 0):
# 180° about approach(z), preset "pika_rz180" = diag(-1,-1,+1). pika UMI replay
# ALWAYS uses this (it drives toward -y, the box). See wiki umi-axis-probe /
# flow_inference.EE_LOCAL_R_ALIGN_PRESETS.
DEFAULT_R_ALIGN = "pika_rz180"
DEFAULT_MOCK_CURRENT_POSES = {
    "left": np.asarray([-0.18, -0.42, 0.24, 0.0, 0.0, 0.0, 1.0], dtype=np.float64),
    "right": np.asarray([0.18, -0.42, 0.24, 0.0, 0.0, 0.0, 1.0], dtype=np.float64),
}


@dataclass(frozen=True)
class ServerRuntimeConfig:
    path: Path
    command_endpoint: str
    state_bind: str | None
    servo_rate_hz: float
    command_timeout_sec: float
    smd_max_linear_velocity_m_s: float | None
    smd_max_angular_velocity_rad_s: float | None
    floor_z_min_m: float | None
    roi_min_m: tuple[float, float, float] | None
    roi_max_m: tuple[float, float, float] | None
    raw: dict[str, Any]


@dataclass(frozen=True)
class InitDelta:
    arm: str
    linear_m: float
    angular_deg: float


@dataclass(frozen=True)
class ReplayPaths:
    episode_id: str
    run_name: str
    run_dir: Path
    npz_path: Path
    log_path: Path


@dataclass
class ReplayPlan:
    paths: ReplayPaths
    selected_arms: tuple[str, ...]
    t_servo: np.ndarray
    goals: dict[str, np.ndarray]
    raw_targets: dict[str, np.ndarray]
    twists: dict[str, np.ndarray]
    grippers: dict[str, np.ndarray]
    src_lo: np.ndarray
    src_hi: np.ndarray
    hold: np.ndarray
    gap: np.ndarray
    dropout: np.ndarray
    current_poses: dict[str, np.ndarray]
    init_deltas: dict[str, InitDelta]
    init_poses: dict[str, np.ndarray]
    stream_speed_stats: dict[str, Any]
    bounds_notes: list[str]
    segment_selection: dict[str, Any]
    would_abort_large_init: bool
    dry_run: bool


@dataclass(frozen=True)
class EeLocalRAlignValue:
    linear: np.ndarray
    angular: np.ndarray


class ReplayRefusal(RuntimeError):
    pass


class WatchdogStop(RuntimeError):
    pass


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    selected_arms = parse_arms(args.arms)
    validate_source_args(args)
    execute_requested = bool(args.execute)
    dry_run = not execute_requested
    if execute_requested and not all_execute_gates_present(args):
        print("DRY RUN — execute gates incomplete; no motion sent")
        dry_run = True
    if dry_run:
        print("DRY RUN — no motion sent")

    try:
        server = load_server_config(Path(args.server_config) if args.server_config else default_server_config())
        rate_hz = float(args.rate_hz) if args.rate_hz is not None else float(server.servo_rate_hz)
        if rate_hz <= 0.0:
            raise ReplayRefusal("--rate-hz/server servo.rate_hz must be positive")
        if args.time_scale < 1.0:
            raise ReplayRefusal("--time-scale must be >= 1.0")
        configure_client_speed_limits(args, server)

        if args.source == "ee_local":
            data_tcp_path = prepare_data_tcp_episode(args)
            current_poses = resolve_ee_local_anchor(args, server, selected_arms, dry_run=dry_run)
            npz_path = prepare_ee_local_npz(args, data_tcp_path, selected_arms, current_poses, rate_hz)
        else:
            npz_path = prepare_npz(args, selected_arms, rate_hz)
            current_poses = resolve_current_poses(args, server, npz_path, selected_arms, dry_run=dry_run)
        plan = build_replay_plan(args, server, npz_path, selected_arms, current_poses, rate_hz, dry_run=dry_run)
        if dry_run:
            write_would_be_log(plan, args, server)
        print_plan_summary(plan, server, args)

        if dry_run:
            return 0
        if plan_exceeds_stream_limits(plan):
            raise ReplayRefusal("conditioned stream exceeds client speed clamp; regenerate slower targets or raise limits deliberately")
        if plan.would_abort_large_init and not args.allow_large_init_move:
            raise ReplayRefusal("initial pose delta exceeds threshold; jog closer or pass --allow-large-init-move")
        confirm_start(selected_arms, args)
        run_execute(plan, args, server)
        return 0
    except ReplayRefusal as exc:
        print(f"REFUSED: {exc}", file=sys.stderr)
        return 2
    except WatchdogStop as exc:
        print(f"WATCHDOG STOP: {exc}", file=sys.stderr)
        return 3
    except KeyboardInterrupt:
        print("Interrupted before execute cleanup completed.", file=sys.stderr)
        return 130


def parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fail-closed real-robot TcpPoseTarget replay driver for Phase-1 conditioned goals."
    )
    parser.add_argument("--source", default="absolute", choices=["absolute", "ee_local"])
    src = parser.add_mutually_exclusive_group(required=False)
    src.add_argument("--episode", help="Raw HDF5 episode; read-only, converted through CommandConditioner.")
    src.add_argument("--npz", help="Pre-generated conditioned Phase-1 npz.")
    parser.add_argument("--data-tcp", help="Converted data_tcp HDF5 episode for --source ee_local.")
    parser.add_argument("--retarget", default=str(DEFAULT_RETARGET_CONFIG), help="UMI retarget config for inline --source ee_local conversion.")
    parser.add_argument("--r-align", default=DEFAULT_R_ALIGN, help="ee_local r_align preset name or 9 row-major floats.")
    parser.add_argument("--action-scale", type=float, default=1.0, help="Uniform scale for per-step ee_local body deltas.")
    parser.add_argument("--segment", default="all", help="ee_local source segment: all, auto-largest, or a zero-based segment index.")
    parser.add_argument("--anchor", choices=["live", "mock"], default=None, help="Anchor ee_local replay at live server TCP or mock stand poses.")
    parser.add_argument(
        "--anchor-pose-source",
        choices=["actual", "reference", "auto"],
        default="auto",
        help="Live-anchor pose source: 'reference' (tcp_ref_stand), 'actual' (tcp_actual_stand), "
        "or 'auto' (reference-preferred). pgmode freezes actual, so auto/reference avoid the "
        "inter-episode catch-up jump.",
    )
    parser.add_argument("--mode", default=DEFAULT_MODE, choices=["clean_foh_se3"])
    parser.add_argument("--arms", default="left,right")
    parser.add_argument("--server-config", default=None)
    parser.add_argument("--time-scale", type=float, default=1.0)
    parser.add_argument(
        "--time-scale-mode",
        choices=["wall_clock_resample", "legacy_sleep"],
        default="wall_clock_resample",
        help="wall_clock_resample (default): dispatch at the full wall-clock rate_hz and "
        "advance episode time by 1/(rate_hz*time_scale) per tick (no server-side ZOH on "
        "intervening ticks). legacy_sleep: send stored ticks at period=time_scale/rate_hz "
        "(reproduces pre-2026-06 logs).",
    )
    parser.add_argument("--rate-hz", type=float, default=None)
    # --- Patch 2: A-stage command-conditioning config (tunable, logged) -----------
    parser.add_argument(
        "--conditioning-config",
        default=None,
        help="Optional tcp_tuning YAML (conditioning/smoothing sections) merged over defaults.",
    )
    parser.add_argument("--smoothing-method", choices=["none", "savgol", "lowpass", "cubic"], default=None)
    parser.add_argument("--smoothing-window-samples", type=int, default=None)
    parser.add_argument("--smoothing-polyorder", type=int, default=None)
    parser.add_argument("--lowpass-cutoff-hz", type=float, default=None)
    parser.add_argument("--cubic-smoothing", type=float, default=None)
    parser.add_argument("--gap-median-multiplier", type=float, default=None)
    parser.add_argument("--gap-absolute-threshold-sec", type=float, default=None)
    parser.add_argument(
        "--send-conditioned-twist",
        action="store_true",
        help="Attach the A-stage conditioned twist (scaled by 1/time_scale to wall-clock) "
        "to each TcpPoseTarget as tcp_target_twist_stand (Patch 5). The server SMD uses it "
        "only when velocity_feedforward_source is command_twist/auto; ignored otherwise.",
    )
    parser.add_argument("--max-linear-speed-m-s", type=float, default=None)
    parser.add_argument("--max-angular-speed-rad-s", type=float, default=None)
    parser.add_argument("--init-move-sec", type=float, default=5.0)
    parser.add_argument("--max-init-delta-m", type=float, default=0.15)
    parser.add_argument("--max-init-delta-deg", type=float, default=30.0)
    parser.add_argument("--allow-large-init-move", action="store_true")
    parser.add_argument("--out-dir", default="outputs/tcp_tuning")
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--mock-current-pose", default=None, help="'first' or JSON/object pose map for offline dry-run.")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--dry-run", action="store_true", help="Explicit no-motion mode; this is also the default.")
    parser.add_argument("--i-am-at-the-estop", action="store_true")
    parser.add_argument("--source-id", default="tcp_pose_replay")
    parser.add_argument("--state-timeout-sec", type=float, default=2.0)
    parser.add_argument(
        "--allow-controller-sim-arm-error",
        action="store_true",
        help="In rbpodo pgmode controller-sim only, tolerate a per-arm not-activated has_error so replay can collect IK telemetry.",
    )
    parser.add_argument("--non-interactive", action="store_true", help="Skip only the two typed physical-motion confirmations.")
    parser.add_argument("--run-name", default=None, help="Override the run directory name under <out-dir>/<episode_id>/runs/.")
    return parser.parse_args(argv)


def parse_arms(text: str) -> tuple[str, ...]:
    arms = tuple(item.strip() for item in text.split(",") if item.strip())
    invalid = sorted(set(arms) - set(ARMS))
    if invalid:
        raise ReplayRefusal(f"invalid --arms value(s): {', '.join(invalid)}")
    return arms or ARMS


def all_execute_gates_present(args: argparse.Namespace) -> bool:
    return bool(args.execute and not args.dry_run and args.i_am_at_the_estop)


def validate_source_args(args: argparse.Namespace) -> None:
    if args.source == "absolute":
        if not args.episode and not args.npz:
            raise ReplayRefusal("--source absolute requires --episode or --npz")
        if args.data_tcp:
            raise ReplayRefusal("--data-tcp is only valid with --source ee_local")
        if str(args.segment) != "all":
            raise ReplayRefusal("--segment is only valid with --source ee_local")
        return
    if args.npz:
        raise ReplayRefusal("--source ee_local composes its own anchored npz; pass --data-tcp or --episode")
    if bool(args.data_tcp) == bool(args.episode):
        raise ReplayRefusal("--source ee_local requires exactly one of --data-tcp or --episode")
    if float(args.action_scale) < 0.0:
        raise ReplayRefusal("--action-scale must be >= 0")


def default_server_config() -> Path:
    for path in DEFAULT_SERVER_CONFIG_CANDIDATES:
        if path.exists():
            return path
    raise ReplayRefusal("no default server config found; pass --server-config")


def load_server_config(path: Path) -> ServerRuntimeConfig:
    try:
        import yaml
    except ModuleNotFoundError as exc:
        raise ReplayRefusal("PyYAML is required to read rb_servo_server config") from exc
    if not path.exists():
        raise ReplayRefusal(f"server config does not exist: {path}")
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise ReplayRefusal(f"server config root must be a mapping: {path}")
    network = data.get("network") or {}
    servo = data.get("servo") or {}
    safety = data.get("safety") or {}
    cartesian = data.get("cartesian_control") if isinstance(data.get("cartesian_control"), dict) else {}
    pose_track_smd = cartesian.get("pose_track_smd") if isinstance(cartesian.get("pose_track_smd"), dict) else {}
    command_endpoint = str(network.get("command_bind") or "")
    if not command_endpoint:
        raise ReplayRefusal("server config network.command_bind is required")
    endpoints = network.get("state_pub_endpoints")
    if isinstance(endpoints, list) and endpoints:
        state_bind = str(endpoints[-1])
    else:
        state_bind = network.get("state_pub_endpoint")
        state_bind = str(state_bind) if state_bind else None
    floor = safety.get("floor_constraint") if isinstance(safety.get("floor_constraint"), dict) else {}
    floor_z = None
    if floor.get("enable") is True and floor.get("monitor_only") is not True:
        floor_z = _float_or_none(floor.get("z_min_m"))
    roi = safety.get("roi_box") if isinstance(safety.get("roi_box"), dict) else {}
    roi_min = roi_max = None
    if roi.get("enable") is True and roi.get("monitor_only") is not True:
        roi_min = _vec3_or_none(roi.get("min_m"))
        roi_max = _vec3_or_none(roi.get("max_m"))
    return ServerRuntimeConfig(
        path=path,
        command_endpoint=command_endpoint,
        state_bind=state_bind,
        servo_rate_hz=float(servo.get("rate_hz", 500.0)),
        command_timeout_sec=float(servo.get("command_timeout_sec", 0.2)),
        smd_max_linear_velocity_m_s=_float_or_none(pose_track_smd.get("max_linear_velocity_m_s")),
        smd_max_angular_velocity_rad_s=_float_or_none(pose_track_smd.get("max_angular_velocity_rad_s")),
        floor_z_min_m=floor_z,
        roi_min_m=roi_min,
        roi_max_m=roi_max,
        raw=data,
    )


def configure_client_speed_limits(args: argparse.Namespace, server: ServerRuntimeConfig) -> None:
    if args.max_linear_speed_m_s is None:
        value = server.smd_max_linear_velocity_m_s
        if value is None:
            value = FALLBACK_MAX_LINEAR_SPEED_M_S
            source = "fallback_default"
        else:
            source = "server_config:cartesian_control.pose_track_smd.max_linear_velocity_m_s"
        args.max_linear_speed_m_s = float(value)
        args._max_linear_speed_source = source
    else:
        args.max_linear_speed_m_s = float(args.max_linear_speed_m_s)
        args._max_linear_speed_source = "cli"
    if args.max_angular_speed_rad_s is None:
        value = server.smd_max_angular_velocity_rad_s
        if value is None:
            value = FALLBACK_MAX_ANGULAR_SPEED_RAD_S
            source = "fallback_default"
        else:
            source = "server_config:cartesian_control.pose_track_smd.max_angular_velocity_rad_s"
        args.max_angular_speed_rad_s = float(value)
        args._max_angular_speed_source = source
    else:
        args.max_angular_speed_rad_s = float(args.max_angular_speed_rad_s)
        args._max_angular_speed_source = "cli"
    if args.max_linear_speed_m_s <= 0.0 or not math.isfinite(args.max_linear_speed_m_s):
        raise ReplayRefusal("--max-linear-speed-m-s must be positive and finite")
    if args.max_angular_speed_rad_s <= 0.0 or not math.isfinite(args.max_angular_speed_rad_s):
        raise ReplayRefusal("--max-angular-speed-rad-s must be positive and finite")


def prepare_npz(args: argparse.Namespace, selected_arms: tuple[str, ...], rate_hz: float) -> Path:
    if args.npz:
        path = Path(args.npz)
        if not path.exists():
            raise ReplayRefusal(f"npz does not exist: {path}")
        return path
    episode_path = Path(args.episode)
    if not episode_path.exists():
        raise ReplayRefusal(f"episode does not exist: {episode_path}")
    episode = load_episode(str(episode_path), nominal_rate_hz=Phase1Config().conditioning.nominal_source_rate_hz)
    cfg = _phase1_config(args, rate_hz)
    out_dir = Path(args.out_dir) / hdf5_episode_id(episode_path)
    out_dir.mkdir(parents=True, exist_ok=True)
    return generate_replay_npz(episode, args.mode, out_dir, cfg, selected_arms=selected_arms, seed=args.seed)


def prepare_data_tcp_episode(args: argparse.Namespace) -> Path:
    if args.data_tcp:
        path = Path(args.data_tcp)
        if not path.exists():
            raise ReplayRefusal(f"data_tcp episode does not exist: {path}")
        verify_data_tcp_targets(path)
        return path
    source = Path(args.episode)
    if not source.exists():
        raise ReplayRefusal(f"episode does not exist: {source}")
    retarget = Path(args.retarget)
    if not retarget.exists():
        raise ReplayRefusal(f"retarget config does not exist: {retarget}")
    output_dir = Path(args.out_dir) / "inline_data_tcp" / hdf5_episode_id(source)
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / source.name
    convert_umi_episode(
        source,
        output_path,
        output_format="robotics_lab_dual_arm",
        retarget_config=retarget,
        require_measured_retarget=False,
    )
    verify_data_tcp_targets(output_path)
    return output_path


def verify_data_tcp_targets(path: Path) -> None:
    with h5py.File(path, "r") as handle:
        missing = [key for key in ("action/target_pose_left", "action/target_pose_right") if key not in handle]
        if missing:
            raise ReplayRefusal(f"data_tcp episode missing required target pose dataset(s): {', '.join(missing)}")
        for key in ("action/target_pose_left", "action/target_pose_right"):
            arr = np.asarray(handle[key], dtype=np.float64)
            if arr.ndim != 2 or arr.shape[1] != 7 or arr.shape[0] < 1:
                raise ReplayRefusal(f"{path}:{key} must have shape (N,7), got {arr.shape}")
            if not np.isfinite(arr).all():
                raise ReplayRefusal(f"{path}:{key} contains non-finite values")


def prepare_ee_local_npz(
    args: argparse.Namespace,
    data_tcp_path: Path,
    selected_arms: tuple[str, ...],
    current_poses: dict[str, np.ndarray],
    rate_hz: float,
) -> Path:
    cfg = _phase1_config(args, rate_hz)
    episode, source_meta = anchored_ee_local_episode(
        data_tcp_path,
        selected_arms,
        current_poses,
        r_align_spec=args.r_align,
        action_scale=float(args.action_scale),
        nominal_rate_hz=cfg.conditioning.nominal_source_rate_hz,
        segment=str(args.segment),
        conditioning_cfg=cfg.conditioning,
    )
    t_source, nominal_rate_used = effective_timestamps_for_episode(episode, cfg.conditioning.nominal_source_rate_hz)
    conditioner = CommandConditioner(args.mode, cfg)
    for index, t_value in enumerate(t_source):
        conditioner.update_source_sample(
            t_value,
            _row_or_none(episode.left_pose, index) if "left" in selected_arms else None,
            _row_or_none(episode.right_pose, index) if "right" in selected_arms else None,
            _value_or_none(episode.left_gripper, index) if "left" in selected_arms else None,
            _value_or_none(episode.right_gripper, index) if "right" in selected_arms else None,
            metadata={"source_index": index, "source": "ee_local_anchored", "nominal_rate_used": nominal_rate_used},
        )
    t_servo = phase1_servo_times(t_source, cfg.conditioning.servo_rate_hz)
    commands = [conditioner.sample(t_value) for t_value in t_servo]
    meta = {
        "git_commit": git_commit(),
        "phase1_git_commit": phase1_git_commit(),
        "config": cfg.to_dict(),
        "replay_mode": args.mode,
        "source_mode": "ee_local",
        "episode_id": hdf5_episode_id(data_tcp_path),
        "episode_path": str(data_tcp_path),
        "segment_range": conditioner.segments,
        "seed": int(args.seed) if args.seed is not None else -1,
        "detected_schema": episode.detected,
        "nominal_rate_used": bool(nominal_rate_used or conditioner.meta.get("nominal_rate_used", False)),
        **source_meta,
    }
    arrays = phase1_build_npz_arrays(episode, t_source, t_servo, commands, conditioner, args.mode, cfg, meta)
    output_dir = Path(args.out_dir) / hdf5_episode_id(data_tcp_path)
    output_dir.mkdir(parents=True, exist_ok=True)
    scale_token = scale_filename_token(float(args.action_scale))
    segment_token = segment_filename_token(source_meta.get("segment_selection", {}))
    output_path = output_dir / f"ee_local_scale_{scale_token}_{segment_token}_{args.mode}_{int(round(cfg.conditioning.servo_rate_hz))}hz.npz"
    np.savez(output_path, **arrays)
    return output_path


def anchored_ee_local_episode(
    data_tcp_path: Path,
    selected_arms: tuple[str, ...],
    current_poses: dict[str, np.ndarray],
    *,
    r_align_spec: Any,
    action_scale: float,
    nominal_rate_hz: float,
    segment: str,
    conditioning_cfg: Any,
) -> tuple[EpisodeData, dict[str, Any]]:
    with h5py.File(data_tcp_path, "r") as handle:
        left_target = np.asarray(handle["action/target_pose_left"], dtype=np.float64)
        right_target = np.asarray(handle["action/target_pose_right"], dtype=np.float64)
        t_source = _data_tcp_timestamps(handle, max(left_target.shape[0], right_target.shape[0]), nominal_rate_hz)
        left_gripper = _optional_dataset(handle, "action/gripper_left")
        right_gripper = _optional_dataset(handle, "action/gripper_right")
        attrs = {str(key): _jsonable(value) for key, value in handle.attrs.items()}
    if left_target.shape[0] != right_target.shape[0]:
        raise ReplayRefusal("left/right action target pose datasets must have the same length")
    segment_selection = select_source_segment(
        t_source,
        segment,
        median_multiplier=float(conditioning_cfg.gap_median_multiplier),
        absolute_threshold_sec=float(conditioning_cfg.gap_absolute_threshold_sec),
    )
    start = int(segment_selection["source_start"])
    stop = int(segment_selection["source_stop_exclusive"])
    if segment_selection["mode"] == "single":
        left_target = left_target[start:stop]
        right_target = right_target[start:stop]
        t_source = t_source[start:stop]
        left_gripper = _slice_optional(left_gripper, start, stop)
        right_gripper = _slice_optional(right_gripper, start, stop)
    r_align = resolve_r_align_spec(r_align_spec)
    source_targets = {"left": left_target, "right": right_target}
    anchored: dict[str, np.ndarray | None] = {"left": None, "right": None}
    delta_stats: dict[str, Any] = {}
    for arm in ARMS:
        if arm not in selected_arms:
            continue
        target = source_targets[arm]
        anchored[arm], delta_stats[arm] = compose_ee_local_from_anchor(
            target,
            current_poses[arm],
            r_align=r_align,
            action_scale=action_scale,
        )
    detected = {
        "path": str(data_tcp_path),
        "format_name": "robotics_lab_data_tcp_ee_local_anchored",
        "pose_frame": "stand",
        "pose_format": "x,y,z,qx,qy,qz,qw",
        "source_pose_frame": attrs.get("source_pose_frame"),
        "retarget_status": attrs.get("retarget_status"),
        "selected": {
            "left_pose": "action/target_pose_left",
            "right_pose": "action/target_pose_right",
            "left_gripper": "action/gripper_left",
            "right_gripper": "action/gripper_right",
        },
        "segment_selection": segment_selection,
        "notes": [
            "source target poses are used only for pose_delta_local body-frame deltas",
            "absolute output is composed from live/mock stand-frame anchor poses",
        ],
        "root_attrs": attrs,
    }
    episode = EpisodeData(
        path=str(data_tcp_path),
        t_source=t_source,
        left_pose=anchored["left"],
        right_pose=anchored["right"],
        left_gripper=left_gripper,
        right_gripper=right_gripper,
        detected=detected,
    )
    meta = {
        "ee_local": {
            "r_align": str(r_align_spec),
            "action_scale": float(action_scale),
            "anchors": {arm: current_poses[arm].tolist() for arm in selected_arms},
            "delta_stats": delta_stats,
        }
    }
    meta["segment_selection"] = segment_selection
    return episode, meta


def select_source_segment(
    t_source: np.ndarray,
    requested: str,
    *,
    median_multiplier: float,
    absolute_threshold_sec: float,
) -> dict[str, Any]:
    times = np.asarray(t_source, dtype=np.float64).reshape(-1)
    segments, gaps = split_segments(
        times,
        median_multiplier=median_multiplier,
        absolute_threshold_sec=absolute_threshold_sec,
    )
    if not segments and times.size:
        segments = [(0, int(times.size))]
    spec = str(requested).strip()
    if spec == "":
        spec = "all"
    source_count = int(times.size)
    base = {
        "requested": spec,
        "source_frame_count": source_count,
        "all_segments": [[int(start), int(stop)] for start, stop in segments],
        "gap_count": int(gaps.shape[0]),
        "gaps": gaps.tolist(),
    }
    if spec == "all":
        return {
            **base,
            "mode": "all",
            "segment_index": None,
            "source_start": 0,
            "source_stop_exclusive": source_count,
            "source_frame_range": [0, source_count - 1] if source_count else [],
            "selected_frame_count": source_count,
            "dropped_frame_count": 0,
            "dropped_before": 0,
            "dropped_after": 0,
        }
    if not segments:
        raise ReplayRefusal("cannot select a segment from an empty source episode")
    if spec == "auto-largest":
        segment_index, (start, stop) = max(
            enumerate(segments),
            key=lambda item: (int(item[1][1]) - int(item[1][0]), -int(item[0])),
        )
    else:
        try:
            segment_index = int(spec)
        except ValueError as exc:
            raise ReplayRefusal("--segment must be all, auto-largest, or a zero-based integer index") from exc
        if segment_index < 0 or segment_index >= len(segments):
            raise ReplayRefusal(f"--segment {segment_index} out of range; available segments: 0..{len(segments) - 1}")
        start, stop = segments[segment_index]
    start = int(start)
    stop = int(stop)
    selected_count = max(0, stop - start)
    return {
        **base,
        "mode": "single",
        "segment_index": int(segment_index),
        "source_start": start,
        "source_stop_exclusive": stop,
        "source_frame_range": [start, stop - 1] if selected_count else [],
        "selected_frame_count": int(selected_count),
        "dropped_frame_count": int(source_count - selected_count),
        "dropped_before": int(start),
        "dropped_after": int(source_count - stop),
    }


def compose_ee_local_from_anchor(
    target_poses: np.ndarray,
    anchor_pose: np.ndarray,
    *,
    r_align: Any,
    action_scale: float,
) -> tuple[np.ndarray, dict[str, Any]]:
    target = np.asarray(target_poses, dtype=np.float64)
    if target.ndim != 2 or target.shape[1] != 7 or target.shape[0] < 1:
        raise ReplayRefusal(f"target poses must have shape (N,7), got {target.shape}")
    deltas = np.asarray(
        [pose_delta_local(target[i], target[i + 1]) for i in range(target.shape[0] - 1)],
        dtype=np.float64,
    )
    if r_align is not None and deltas.size:
        r_lin = np.asarray(r_align.linear, dtype=np.float64)
        r_ang = np.asarray(r_align.angular, dtype=np.float64)
        deltas[:, 0:3] = deltas[:, 0:3] @ r_lin.T
        deltas[:, 3:6] = deltas[:, 3:6] @ r_ang.T
    if action_scale != 1.0:
        deltas *= float(action_scale)
    composed = np.empty_like(target, dtype=np.float64)
    composed[0] = canonical_pose7(anchor_pose)
    for i, delta in enumerate(deltas):
        composed[i + 1] = canonical_pose7(pose_compose_local(composed[i], delta[:6]))
    lin_norm = np.linalg.norm(deltas[:, 0:3], axis=1) if deltas.size else np.asarray([], dtype=np.float64)
    ang_norm = np.linalg.norm(deltas[:, 3:6], axis=1) if deltas.size else np.asarray([], dtype=np.float64)
    return composed, {
        "sample_count": int(target.shape[0]),
        "delta_count": int(deltas.shape[0]),
        "max_step_linear_m": float(np.max(lin_norm)) if lin_norm.size else 0.0,
        "max_step_angular_rad": float(np.max(ang_norm)) if ang_norm.size else 0.0,
    }


def resolve_r_align_spec(value: Any) -> Any:
    try:
        from policy_runner.flow_inference import resolve_ee_local_r_align

        return resolve_ee_local_r_align(value)
    except ModuleNotFoundError as exc:
        if exc.name != "torch":
            raise
        return _resolve_r_align_without_torch(value)


def _resolve_r_align_without_torch(value: Any) -> EeLocalRAlignValue | None:
    if value is None or isinstance(value, EeLocalRAlignValue):
        return value
    if isinstance(value, str):
        key = value.strip().lower().replace("-", "_")
        if key in {"", "none", "identity"}:
            return None
        # Mirror flow_inference.EE_LOCAL_R_ALIGN_PRESETS so canonical preset names
        # resolve even without torch (e.g. the measured pika_rz180 correction).
        _PRESETS = {
            "pika_rz180": (-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0),
        }
        if key in _PRESETS:
            value = list(_PRESETS[key])
        else:
            try:
                value = [float(item) for item in value.replace(",", " ").split()]
            except ValueError as exc:
                raise ReplayRefusal(
                    "policy_runner.flow_inference requires torch for r-align presets in this environment; "
                    "pass a known preset (pika_rz180), 'identity', or 9 row-major floats"
                ) from exc
    matrix = np.asarray(list(value), dtype=np.float64)
    if matrix.size != 9:
        raise ReplayRefusal("--r-align must be a preset or exactly 9 row-major floats")
    matrix = matrix.reshape(3, 3)
    if not np.allclose(matrix @ matrix.T, np.eye(3), atol=1e-6):
        raise ReplayRefusal("--r-align matrix must be orthonormal")
    return EeLocalRAlignValue(linear=matrix, angular=matrix)


def _data_tcp_timestamps(handle: h5py.File, count: int, nominal_rate_hz: float) -> np.ndarray:
    for key in ("observations/timestamp", "timestamp", "timestamps"):
        if key in handle:
            values = np.asarray(handle[key], dtype=np.float64).reshape(-1)
            if values.size >= count:
                return values[:count]
    return np.arange(count, dtype=np.float64) / float(nominal_rate_hz)


def scale_filename_token(value: float) -> str:
    text = f"{float(value):.6g}"
    return text.replace("-", "neg").replace(".", "p")


def segment_filename_token(selection: dict[str, Any]) -> str:
    if selection.get("mode") != "single":
        return "segment_all"
    index = int(selection.get("segment_index", -1))
    start = int(selection.get("source_start", 0))
    stop = int(selection.get("source_stop_exclusive", 0))
    return f"segment_{index}_{start}_{stop}"


def _optional_dataset(handle: h5py.File, key: str) -> np.ndarray | None:
    if key not in handle:
        return None
    return np.asarray(handle[key], dtype=np.float64)


def _slice_optional(array: np.ndarray | None, start: int, stop: int) -> np.ndarray | None:
    if array is None:
        return None
    return np.asarray(array, dtype=np.float64)[int(start):int(stop)]


def effective_timestamps_for_episode(episode: EpisodeData, nominal_rate_hz: float) -> tuple[np.ndarray, bool]:
    if episode.t_source is not None and episode.t_source.size:
        return np.asarray(episode.t_source, dtype=np.float64).reshape(-1), False
    lengths = [
        arr.shape[0]
        for arr in (episode.left_pose, episode.right_pose, episode.left_gripper, episode.right_gripper)
        if arr is not None and arr.ndim >= 1
    ]
    count = max(lengths) if lengths else 0
    return np.arange(count, dtype=np.float64) / float(nominal_rate_hz), True


def _smoothing_tag(args: argparse.Namespace, rate_hz: float) -> str:
    """Short filename-safe tag describing the effective A-stage smoothing (Patch 2)."""

    cfg = _phase1_config(args, rate_hz).smoothing
    method = str(cfg.method).lower()
    if method in {"none", "off", "identity"}:
        return "_smnone"
    if method.startswith("savgol") or method in {"savitzky_golay", "savitzky-golay"}:
        return f"_savgol_w{int(cfg.window_samples)}p{int(cfg.polyorder)}"
    if method.startswith("low") or method in {"butter", "butterworth"}:
        return f"_lowpass{_num_tag(cfg.lowpass_cutoff_hz)}hz"
    if method.startswith("cubic") or method in {"spline", "smoothing_spline"}:
        return f"_cubic{_num_tag(cfg.cubic_smoothing)}"
    return ""


def _num_tag(value: float) -> str:
    return f"{float(value):g}".replace(".", "p")


def _phase1_config(args: argparse.Namespace, rate_hz: float) -> Phase1Config:
    """Build the effective Phase-1 conditioning/smoothing config.

    Precedence (low -> high): dataclass defaults -> ``--conditioning-config`` YAML ->
    CLI smoothing/gap overrides. The servo rate is always forced to the wall-clock
    dispatch rate, and the synthetic seed comes from ``--seed`` when given.
    """

    from dataclasses import replace

    cfg = load_tcp_tuning_config(getattr(args, "conditioning_config", None))
    cfg = apply_conditioning_cli_overrides(
        cfg,
        {
            "smoothing_method": getattr(args, "smoothing_method", None),
            "smoothing_window_samples": getattr(args, "smoothing_window_samples", None),
            "smoothing_polyorder": getattr(args, "smoothing_polyorder", None),
            "lowpass_cutoff_hz": getattr(args, "lowpass_cutoff_hz", None),
            "cubic_smoothing": getattr(args, "cubic_smoothing", None),
            "gap_median_multiplier": getattr(args, "gap_median_multiplier", None),
            "gap_absolute_threshold_sec": getattr(args, "gap_absolute_threshold_sec", None),
        },
    )
    seed = getattr(args, "seed", None)
    synthetic = cfg.synthetic if seed is None else replace(cfg.synthetic, seed=int(seed))
    conditioning = replace(cfg.conditioning, servo_rate_hz=float(rate_hz))
    return replace(cfg, conditioning=conditioning, synthetic=synthetic)


def resolve_current_poses(
    args: argparse.Namespace,
    server: ServerRuntimeConfig,
    npz_path: Path,
    selected_arms: tuple[str, ...],
    *,
    dry_run: bool,
) -> dict[str, np.ndarray]:
    if args.mock_current_pose:
        return mock_current_poses(args.mock_current_pose, npz_path, selected_arms)
    if server.state_bind is None:
        if dry_run:
            return first_goal_poses(npz_path, selected_arms)
        raise ReplayRefusal("server config has no state endpoint; cannot execute safely")
    snapshot = read_state_snapshot(server.state_bind, timeout_sec=float(args.state_timeout_sec))
    if snapshot is None:
        if dry_run:
            print("State snapshot unreachable; using first conditioned pose for dry-run init delta.")
            return first_goal_poses(npz_path, selected_arms)
        raise ReplayRefusal(f"no fresh state snapshot received on {server.state_bind}")
    poses = {}
    for arm in selected_arms:
        poses[arm] = actual_tcp_pose_from_state(snapshot.payload, arm)
    return poses


def resolve_ee_local_anchor(
    args: argparse.Namespace,
    server: ServerRuntimeConfig,
    selected_arms: tuple[str, ...],
    *,
    dry_run: bool,
) -> dict[str, np.ndarray]:
    anchor = args.anchor or ("mock" if dry_run else "live")
    if not dry_run and anchor != "live":
        raise ReplayRefusal("--source ee_local execute mode must use --anchor live")
    if anchor == "mock":
        if not dry_run:
            raise ReplayRefusal("--anchor mock is dry-run only")
        return mock_anchor_poses(args.mock_current_pose, selected_arms)
    if server.state_bind is None:
        raise ReplayRefusal("server config has no state endpoint; cannot read live anchor")
    snapshot = read_state_snapshot(server.state_bind, timeout_sec=float(args.state_timeout_sec))
    if snapshot is None:
        raise ReplayRefusal(f"no fresh state snapshot received on {server.state_bind}")
    source = getattr(args, "anchor_pose_source", "auto")
    return {arm: live_anchor_pose_from_state(snapshot.payload, arm, source) for arm in selected_arms}


def read_state_snapshot(bind: str, *, timeout_sec: float) -> StateSnapshot | None:
    client = RobotStateClient(bind=bind, stale_timeout_sec=max(timeout_sec, 0.1))
    try:
        return client.poll_once(timeout_sec=max(timeout_sec, 0.0))
    except OSError:
        return None
    finally:
        client.close()


def mock_current_poses(value: str, npz_path: Path, selected_arms: tuple[str, ...]) -> dict[str, np.ndarray]:
    if value == "first":
        return first_goal_poses(npz_path, selected_arms)
    return parse_pose_map_json(value, selected_arms, error_context="--mock-current-pose must be 'first' or JSON")


def mock_anchor_poses(value: str | None, selected_arms: tuple[str, ...]) -> dict[str, np.ndarray]:
    if value is None or value == "" or value == "default":
        return {arm: DEFAULT_MOCK_CURRENT_POSES[arm].copy() for arm in selected_arms}
    if value == "first":
        raise ReplayRefusal("--mock-current-pose first is not valid for --source ee_local; use a stand-frame pose or 'default'")
    return parse_pose_map_json(value, selected_arms, error_context="--mock-current-pose must be 'default' or JSON")


def parse_pose_map_json(value: str, selected_arms: tuple[str, ...], *, error_context: str) -> dict[str, np.ndarray]:
    try:
        raw = json.loads(value)
    except json.JSONDecodeError as exc:
        raise ReplayRefusal(error_context) from exc
    poses = {}
    if isinstance(raw, list):
        pose_value = canonical_pose7(raw)
        return {arm: pose_value.copy() for arm in selected_arms}
    if not isinstance(raw, dict):
        raise ReplayRefusal(error_context)
    for arm in selected_arms:
        item = raw.get(arm)
        if item is None:
            raise ReplayRefusal(f"--mock-current-pose JSON missing {arm}")
        poses[arm] = canonical_pose7(item)
    return poses


def first_goal_poses(npz_path: Path, selected_arms: tuple[str, ...]) -> dict[str, np.ndarray]:
    with np.load(npz_path, allow_pickle=True) as data:
        return {arm: canonical_pose7(data[f"{arm}_conditioned_goal"][0]) for arm in selected_arms}


def build_replay_plan(
    args: argparse.Namespace,
    server: ServerRuntimeConfig,
    npz_path: Path,
    selected_arms: tuple[str, ...],
    current_poses: dict[str, np.ndarray],
    rate_hz: float,
    *,
    dry_run: bool,
) -> ReplayPlan:
    with np.load(npz_path, allow_pickle=True) as data:
        t_servo = np.asarray(data["t_servo"], dtype=np.float64)
        goals = {arm: np.asarray(data[f"{arm}_conditioned_goal"], dtype=np.float64) for arm in selected_arms}
        raw_targets = {arm: np.asarray(data[f"{arm}_source_raw_target"], dtype=np.float64) for arm in selected_arms}
        twists = {arm: np.asarray(data[f"{arm}_conditioned_twist"], dtype=np.float64) for arm in selected_arms}
        grippers = {
            arm: np.asarray(data[f"{arm}_gripper"], dtype=np.float64) if f"{arm}_gripper" in data.files else np.full(t_servo.shape, np.nan)
            for arm in selected_arms
        }
        src_lo = np.asarray(data[f"{selected_arms[0]}_src_id_lo"], dtype=np.int64)
        src_hi = np.asarray(data[f"{selected_arms[0]}_src_id_hi"], dtype=np.int64)
        # Per-tick conditioner flags (genuine source hold / gap / dropout). Combine
        # across selected arms so a hold on either arm marks the tick. Older npz files
        # without these arrays fall back to "no genuine hold".
        def _flag_array(name: str) -> np.ndarray:
            combined = np.zeros(t_servo.shape, dtype=bool)
            for arm in selected_arms:
                key = f"{arm}_{name}"
                if key in data.files:
                    combined = combined | np.asarray(data[key], dtype=bool)
            return combined
        hold = _flag_array("hold")
        gap = _flag_array("gap")
        dropout = _flag_array("dropout")
        meta = _meta_from_npz(data)
        episode_name = _episode_id_from_meta_or_path(npz_path, meta)
        segment_selection = dict(meta.get("segment_selection") or {})

    if t_servo.size == 0:
        raise ReplayRefusal("conditioned npz contains no servo ticks")
    for arm, arr in goals.items():
        bad = np.argwhere(~np.isfinite(arr))
        if bad.size:
            first = bad[0].tolist()
            raise ReplayRefusal(f"{arm} conditioned_goal contains non-finite value at index {first[0]}, column {first[1]}")
    bounds_notes = validate_bounds(goals, server, refuse=not dry_run)
    speed_stats = validate_stream_speeds(
        goals,
        t_servo,
        time_scale=float(args.time_scale),
        max_linear=float(args.max_linear_speed_m_s),
        max_angular=float(args.max_angular_speed_rad_s),
    )
    init_deltas = {
        arm: compute_init_delta(current_poses[arm], goals[arm][0], arm)
        for arm in selected_arms
    }
    would_abort = any(
        item.linear_m > float(args.max_init_delta_m) or item.angular_deg > float(args.max_init_delta_deg)
        for item in init_deltas.values()
    )
    init_poses = {
        arm: make_init_premove(
            current_poses[arm],
            goals[arm][0],
            requested_sec=float(args.init_move_sec),
            rate_hz=rate_hz,
            max_linear=float(args.max_linear_speed_m_s),
            max_angular=float(args.max_angular_speed_rad_s),
        )
        for arm in selected_arms
    }
    run_name = str(args.run_name) if args.run_name else (
        time.strftime("real_replay_%Y%m%dT%H%M%S")
        + _smoothing_tag(args, rate_hz)
        + ("_dryrun" if dry_run else "_execute")
    )
    run_dir = Path(args.out_dir) / episode_name / "runs" / run_name
    paths = ReplayPaths(
        episode_id=episode_name,
        run_name=run_name,
        run_dir=run_dir,
        npz_path=npz_path,
        log_path=run_dir / "log.csv",
    )
    return ReplayPlan(
        paths=paths,
        selected_arms=selected_arms,
        t_servo=t_servo,
        goals=goals,
        raw_targets=raw_targets,
        twists=twists,
        grippers=grippers,
        src_lo=src_lo,
        src_hi=src_hi,
        hold=hold,
        gap=gap,
        dropout=dropout,
        current_poses=current_poses,
        init_deltas=init_deltas,
        init_poses=init_poses,
        stream_speed_stats=speed_stats,
        bounds_notes=bounds_notes,
        segment_selection=segment_selection,
        would_abort_large_init=would_abort,
        dry_run=dry_run,
    )


def validate_bounds(goals: dict[str, np.ndarray], server: ServerRuntimeConfig, *, refuse: bool = True) -> list[str]:
    notes: list[str] = []
    if server.floor_z_min_m is None:
        notes.append("floor_constraint not enabled in config; client floor precheck skipped")
    else:
        floor_violation = False
        for arm, arr in goals.items():
            below = np.where(arr[:, 2] < float(server.floor_z_min_m))[0]
            if below.size:
                floor_violation = True
                i = int(below[0])
                message = f"{arm} setpoint {i} violates floor z_min_m={server.floor_z_min_m}: z={arr[i, 2]}"
                if refuse:
                    raise ReplayRefusal(message)
                notes.append("VIOLATION: " + message)
        if not floor_violation:
            notes.append(f"floor precheck ok: z >= {server.floor_z_min_m:.4f} m")
    if server.roi_min_m is None or server.roi_max_m is None:
        notes.append("roi_box not enabled in config; client ROI precheck skipped")
    else:
        mn = np.asarray(server.roi_min_m, dtype=np.float64)
        mx = np.asarray(server.roi_max_m, dtype=np.float64)
        roi_violation = False
        for arm, arr in goals.items():
            p = arr[:, :3]
            bad_mask = np.any((p < mn) | (p > mx), axis=1)
            bad = np.where(bad_mask)[0]
            if bad.size:
                roi_violation = True
                i = int(bad[0])
                message = f"{arm} setpoint {i} violates ROI min={mn.tolist()} max={mx.tolist()}: p={p[i].tolist()}"
                if refuse:
                    raise ReplayRefusal(message)
                notes.append("VIOLATION: " + message)
        if not roi_violation:
            notes.append(f"ROI precheck ok: min={mn.tolist()} max={mx.tolist()}")
    return notes


def validate_stream_speeds(
    goals: dict[str, np.ndarray],
    t_servo: np.ndarray,
    *,
    time_scale: float,
    max_linear: float,
    max_angular: float,
) -> dict[str, Any]:
    stats: dict[str, Any] = {}
    for arm, poses in goals.items():
        max_lin = 0.0
        max_ang = 0.0
        max_i = 0
        for i in range(1, poses.shape[0]):
            dt = float(t_servo[i] - t_servo[i - 1]) * float(time_scale)
            if dt <= 0:
                continue
            v, w = twist_from_poses(poses[i - 1, :3], poses[i - 1, 3:7], poses[i, :3], poses[i, 3:7], dt)
            lin = float(np.linalg.norm(v))
            ang = float(np.linalg.norm(w))
            if lin > max_lin or ang > max_ang:
                max_i = i
            max_lin = max(max_lin, lin)
            max_ang = max(max_ang, ang)
        stats[arm] = {
            "max_linear_m_s": max_lin,
            "max_angular_rad_s": max_ang,
            "max_index": max_i,
            "linear_limit_m_s": float(max_linear),
            "angular_limit_rad_s": float(max_angular),
            "exceeds_linear_limit": bool(max_lin > max_linear + 1e-12),
            "exceeds_angular_limit": bool(max_ang > max_angular + 1e-12),
        }
    return stats


def plan_exceeds_stream_limits(plan: ReplayPlan) -> bool:
    return any(
        bool(stats.get("exceeds_linear_limit") or stats.get("exceeds_angular_limit"))
        for stats in plan.stream_speed_stats.values()
    )


def compute_init_delta(current: np.ndarray, first: np.ndarray, arm: str) -> InitDelta:
    linear = float(np.linalg.norm(first[:3] - current[:3]))
    angular = math.degrees(quat_angle_rad(current[3:7], first[3:7]))
    return InitDelta(arm=arm, linear_m=linear, angular_deg=angular)


def make_init_premove(
    current: np.ndarray,
    target: np.ndarray,
    *,
    requested_sec: float,
    rate_hz: float,
    max_linear: float,
    max_angular: float,
) -> np.ndarray:
    delta = compute_init_delta(current, target, "arm")
    min_sec = max(
        delta.linear_m / max(max_linear, 1e-12),
        math.radians(delta.angular_deg) / max(max_angular, 1e-12),
        float(requested_sec),
    )
    count = max(2, int(math.ceil(min_sec * float(rate_hz))) + 1)
    times = np.linspace(0.0, min_sec, count)
    poses = []
    for t in times:
        p, q = foh_pose(t, 0.0, current[:3], current[3:7], min_sec, target[:3], target[3:7])
        poses.append(np.concatenate([p, q]))
    return np.asarray(poses, dtype=np.float64)


@dataclass
class WallClockStream:
    """Conditioned trajectory resampled onto a wall-clock 500 Hz dispatch grid.

    ``time_scale`` slows only episode-time sampling, not the wall-clock dispatch
    frequency: episode time advances by ``1/(rate_hz*time_scale)`` per wall tick, so the
    server never sees a held/ZOH target on intervening ticks (it does at genuine source
    hold/gap/dropout, which ``hold`` marks).
    """

    t_wall: np.ndarray
    t_episode: np.ndarray
    src_idx_lo: np.ndarray
    src_idx_hi: np.ndarray
    interp_alpha: np.ndarray
    hold: np.ndarray
    goals: dict[str, np.ndarray]
    twists: dict[str, np.ndarray]
    grippers: dict[str, np.ndarray]
    raw_targets: dict[str, np.ndarray]

    @property
    def size(self) -> int:
        return int(self.t_wall.size)


def build_wall_clock_replay_stream(plan: ReplayPlan, time_scale: float, rate_hz: float) -> WallClockStream:
    """Resample ``plan``'s episode-time conditioned trajectory to a wall-clock grid.

    The stored trajectory lives on the episode-time 500 Hz grid (``plan.t_servo``).
    Wall ticks fire at ``rate_hz``; episode time at wall tick ``k`` is
    ``t_servo[0] + (k/rate_hz)/time_scale``. Poses use SE(3) FOH (position lerp +
    quaternion slerp), conditioned twist is linearly resampled (episode-time units),
    grippers and source-raw targets hold the nearest previous source sample.
    """

    ts = float(time_scale)
    rate = float(rate_hz)
    if ts <= 0.0 or rate <= 0.0:
        raise ReplayRefusal("time_scale and rate_hz must be positive for wall-clock resampling")
    t_servo = np.asarray(plan.t_servo, dtype=np.float64)
    n = int(t_servo.size)
    if n == 0:
        raise ReplayRefusal("conditioned plan has no servo ticks")
    t0 = float(t_servo[0])
    t_end = float(t_servo[-1])
    span = t_end - t0
    dt_ep = (1.0 / rate) / ts
    if n == 1 or span <= 0.0:
        t_episode = np.asarray([t0], dtype=np.float64)
    else:
        n_steps = int(math.floor(span / dt_ep + 1e-9))
        t_episode = t0 + dt_ep * np.arange(n_steps + 1, dtype=np.float64)
        if t_episode[-1] < t_end - 1e-9:
            t_episode = np.append(t_episode, t_end)
        t_episode = np.minimum(t_episode, t_end)
    m = int(t_episode.size)
    t_wall = (1.0 / rate) * np.arange(m, dtype=np.float64)
    lo = np.clip(np.searchsorted(t_servo, t_episode, side="right") - 1, 0, n - 1)
    hi = np.clip(lo + 1, 0, n - 1)
    denom = t_servo[hi] - t_servo[lo]
    safe_denom = np.where(denom > 0.0, denom, 1.0)
    alpha = np.clip(np.where(denom > 0.0, (t_episode - t_servo[lo]) / safe_denom, 0.0), 0.0, 1.0)
    src_lo_arr = np.asarray(plan.src_lo, dtype=np.int64)
    src_hi_arr = np.asarray(plan.src_hi, dtype=np.int64)
    src_idx_lo = src_lo_arr[lo] if src_lo_arr.size == n else np.full(m, -1, dtype=np.int64)
    src_idx_hi = src_hi_arr[hi] if src_hi_arr.size == n else np.full(m, -1, dtype=np.int64)
    hold = (np.asarray(plan.hold, dtype=bool)[lo] if plan.hold.size == n else np.zeros(m, dtype=bool)) | (lo == hi)
    goals: dict[str, np.ndarray] = {}
    twists: dict[str, np.ndarray] = {}
    grippers: dict[str, np.ndarray] = {}
    raw_targets: dict[str, np.ndarray] = {}
    for arm in plan.selected_arms:
        g = plan.goals[arm]
        tw = plan.twists[arm]
        out_g = np.empty((m, 7), dtype=np.float64)
        out_tw = np.full((m, 6), np.nan, dtype=np.float64)
        for k in range(m):
            a = float(alpha[k])
            i0 = int(lo[k])
            i1 = int(hi[k])
            if i0 == i1 or a <= 0.0:
                out_g[k] = g[i0]
            elif a >= 1.0:
                out_g[k] = g[i1]
            else:
                p, q = foh_pose(a, 0.0, g[i0, :3], g[i0, 3:7], 1.0, g[i1, :3], g[i1, 3:7])
                out_g[k, :3] = p
                out_g[k, 3:7] = q
            out_tw[k] = tw[i0] if i0 == i1 else (1.0 - a) * tw[i0] + a * tw[i1]
        goals[arm] = out_g
        twists[arm] = out_tw
        grippers[arm] = np.asarray(plan.grippers[arm], dtype=np.float64)[lo]
        raw_targets[arm] = np.asarray(plan.raw_targets[arm], dtype=np.float64)[lo]
    return WallClockStream(
        t_wall=t_wall,
        t_episode=t_episode,
        src_idx_lo=src_idx_lo,
        src_idx_hi=src_idx_hi,
        interp_alpha=alpha,
        hold=hold,
        goals=goals,
        twists=twists,
        grippers=grippers,
        raw_targets=raw_targets,
    )


def _conditioned_twist_payload(twist: np.ndarray, time_scale: float) -> list[float] | None:
    """A-stage conditioned twist (episode-time) scaled to wall-clock for command_twist
    feedforward: wall goal velocity = episode twist / time_scale. None if non-finite."""
    arr = np.asarray(twist, dtype=np.float64).reshape(-1)
    if arr.size != 6 or not np.all(np.isfinite(arr)):
        return None
    ts = float(time_scale) if float(time_scale) > 0.0 else 1.0
    return [float(v) / ts for v in arr]


def _stale_repeated_count(stream: WallClockStream, selected_arms: tuple[str, ...]) -> int:
    """Ticks whose emitted goal equals the previous tick on every arm but are NOT a
    genuine source hold/gap/dropout — i.e. unexpected ZOH repeats (should be ~0)."""

    count = 0
    for k in range(1, stream.size):
        if bool(stream.hold[k]):
            continue
        same = all(
            float(np.max(np.abs(stream.goals[arm][k] - stream.goals[arm][k - 1]))) < 1e-12
            for arm in selected_arms
        )
        if same:
            count += 1
    return count


def write_would_be_log(plan: ReplayPlan, args: argparse.Namespace, server: ServerRuntimeConfig) -> None:
    rate_hz = float(args.rate_hz or server.servo_rate_hz)
    mode = str(getattr(args, "time_scale_mode", "wall_clock_resample"))
    metadata = run_metadata(plan, args, server)
    writer = TrajectoryLogWriter(plan.paths.log_path, metadata=metadata)
    if mode == "wall_clock_resample":
        stream = build_wall_clock_replay_stream(plan, float(args.time_scale), rate_hz)
        for k in range(stream.size):
            for arm in plan.selected_arms:
                writer.append(log_row(
                    t=float(stream.t_wall[k]),
                    t_source=float(stream.t_episode[k]),
                    src_idx=int(stream.src_idx_lo[k]),
                    arm=arm,
                    source_raw_target=stream.raw_targets[arm][k],
                    conditioned_goal=stream.goals[arm][k],
                    conditioned_twist=stream.twists[arm][k],
                    snapshot=None,
                    t_episode=float(stream.t_episode[k]),
                    src_idx_lo=int(stream.src_idx_lo[k]),
                    src_idx_hi=int(stream.src_idx_hi[k]),
                    interpolation_alpha=float(stream.interp_alpha[k]),
                    time_scale=float(args.time_scale),
                    time_scale_mode=mode,
                    effective_command_rate_hz=rate_hz,
                    hold=bool(stream.hold[k]),
                ))
        writer.metadata.update(_wall_clock_meta(stream, plan, rate_hz, mode, effective_rates=None))
    else:
        for i, t_value in enumerate(plan.t_servo):
            for arm in plan.selected_arms:
                writer.append(log_row(
                    t=float(t_value) * float(args.time_scale),
                    t_source=float(t_value),
                    src_idx=int(plan.src_lo[i]) if i < plan.src_lo.size else -1,
                    arm=arm,
                    source_raw_target=plan.raw_targets[arm][i],
                    conditioned_goal=plan.goals[arm][i],
                    conditioned_twist=plan.twists[arm][i],
                    snapshot=None,
                    t_episode=float(t_value),
                    src_idx_lo=int(plan.src_lo[i]) if i < plan.src_lo.size else -1,
                    src_idx_hi=int(plan.src_hi[i]) if i < plan.src_hi.size else -1,
                    interpolation_alpha=0.0,
                    time_scale=float(args.time_scale),
                    time_scale_mode=mode,
                    effective_command_rate_hz=rate_hz / float(args.time_scale),
                    hold=bool(plan.hold[i]) if i < plan.hold.size else False,
                ))
        writer.metadata.update({
            "time_scale_mode": mode,
            "wall_clock_dispatch_rate_hz": rate_hz / float(args.time_scale),
            "effective_command_rate_hz": {
                "p50": rate_hz / float(args.time_scale),
                "p95": rate_hz / float(args.time_scale),
                "min": rate_hz / float(args.time_scale),
            },
            "held_tick_count": int(np.count_nonzero(plan.hold)),
            "stale_or_repeated_target_count": 0,
            "tick_count": int(plan.t_servo.size),
        })
    writer.write()


def _wall_clock_meta(
    stream: WallClockStream,
    plan: ReplayPlan,
    rate_hz: float,
    mode: str,
    *,
    effective_rates: list[float] | None,
) -> dict[str, Any]:
    if effective_rates:
        arr = np.asarray(effective_rates, dtype=np.float64)
        arr = arr[np.isfinite(arr)]
        eff = {
            "p50": float(np.percentile(arr, 50)) if arr.size else None,
            "p95": float(np.percentile(arr, 95)) if arr.size else None,
            "min": float(arr.min()) if arr.size else None,
        }
    else:
        eff = {"p50": rate_hz, "p95": rate_hz, "min": rate_hz}
    return {
        "time_scale_mode": mode,
        "wall_clock_dispatch_rate_hz": rate_hz,
        "effective_command_rate_hz": eff,
        "held_tick_count": int(np.count_nonzero(stream.hold)),
        "stale_or_repeated_target_count": _stale_repeated_count(stream, plan.selected_arms),
        "tick_count": int(stream.size),
    }


def run_execute(plan: ReplayPlan, args: argparse.Namespace, server: ServerRuntimeConfig) -> None:
    _TOLERATED_CONTROLLER_SIM_ARM_ERRORS.clear()
    state_client = RobotStateClient(bind=server.state_bind or "", stale_timeout_sec=float(args.state_timeout_sec))
    command_client = ServoCommandClient(
        server.command_endpoint,
        timeout_sec=float(server.command_timeout_sec),
        source_id=str(args.source_id),
    )
    writer = TrajectoryLogWriter(plan.paths.log_path, metadata=run_metadata(plan, args, server))
    try:
        state_client.start()
        wait_for_fresh_state(state_client, timeout_sec=float(args.state_timeout_sec))
        command_client.acquire_lease(StateStreamLeaseReadback(state_client), timeout_sec=4.0)
        command_client.send(CommandIntent.arm_motion(timeout_sec=server.command_timeout_sec))
        stream_init_premove(plan, args, server, command_client, state_client)
        command_client.send(CommandIntent.hold(timeout_sec=server.command_timeout_sec))
        confirm_after_init(plan.selected_arms, args)
        stream_conditioned_goals(plan, args, server, command_client, state_client, writer)
        command_client.send(CommandIntent.hold(timeout_sec=server.command_timeout_sec))
    except KeyboardInterrupt:
        command_client.send(CommandIntent.hold(timeout_sec=server.command_timeout_sec))
        raise
    except WatchdogStop:
        command_client.send(CommandIntent.hold(timeout_sec=server.command_timeout_sec))
        raise
    finally:
        try:
            command_client.release_lease()
        except Exception:
            pass
        command_client.close()
        state_client.close()
        writer.write()


def stream_init_premove(
    plan: ReplayPlan,
    args: argparse.Namespace,
    server: ServerRuntimeConfig,
    client: ServoCommandClient,
    state_client: RobotStateClient,
) -> None:
    count = max(len(poses) for poses in plan.init_poses.values())
    period = 1.0 / float(args.rate_hz or server.servo_rate_hz)
    for i in range(count):
        check_watchdog(state_client, client, allow_controller_sim_arm_error=bool(args.allow_controller_sim_arm_error))
        left = _pose_at_fractional_index(plan.init_poses.get("left"), i, count) if "left" in plan.selected_arms else None
        right = _pose_at_fractional_index(plan.init_poses.get("right"), i, count) if "right" in plan.selected_arms else None
        client.send(tcp_pose_target_stand_intent(left=_pose_list(left), right=_pose_list(right), timeout_sec=server.command_timeout_sec))
        time.sleep(period)


def stream_conditioned_goals(
    plan: ReplayPlan,
    args: argparse.Namespace,
    server: ServerRuntimeConfig,
    client: ServoCommandClient,
    state_client: RobotStateClient,
    writer: TrajectoryLogWriter,
) -> None:
    rate_hz = float(args.rate_hz or server.servo_rate_hz)
    mode = str(getattr(args, "time_scale_mode", "wall_clock_resample"))
    if mode == "wall_clock_resample":
        stream = build_wall_clock_replay_stream(plan, float(args.time_scale), rate_hz)
        _stream_wall_clock(stream, plan, args, server, client, state_client, writer, rate_hz, mode)
        return

    # legacy_sleep: send the stored episode-grid ticks, slowing the sleep period.
    period = float(args.time_scale) / rate_hz
    effective_rates: list[float] = []
    last_wall: float | None = None
    for i in range(plan.t_servo.size):
        check_watchdog(state_client, client, allow_controller_sim_arm_error=bool(args.allow_controller_sim_arm_error))
        snapshot = state_client.latest.payload if state_client.latest is not None else None
        left = plan.goals["left"][i] if "left" in plan.selected_arms else None
        right = plan.goals["right"][i] if "right" in plan.selected_arms else None
        left_g = _finite_float_or_none(plan.grippers["left"][i]) if "left" in plan.selected_arms else None
        right_g = _finite_float_or_none(plan.grippers["right"][i]) if "right" in plan.selected_arms else None
        now = time.perf_counter()
        if last_wall is not None and now > last_wall:
            effective_rates.append(1.0 / (now - last_wall))
        last_wall = now
        client.send(
            tcp_pose_target_stand_intent(
                left=_pose_list(left),
                right=_pose_list(right),
                left_gripper=left_g,
                right_gripper=right_g,
                timeout_sec=server.command_timeout_sec,
            )
        )
        for arm in plan.selected_arms:
            writer.append(log_row(
                t=float(plan.t_servo[i]) * float(args.time_scale),
                t_source=float(plan.t_servo[i]),
                src_idx=int(plan.src_lo[i]) if i < plan.src_lo.size else -1,
                arm=arm,
                source_raw_target=plan.raw_targets[arm][i],
                conditioned_goal=plan.goals[arm][i],
                conditioned_twist=plan.twists[arm][i],
                snapshot=snapshot,
                t_episode=float(plan.t_servo[i]),
                src_idx_lo=int(plan.src_lo[i]) if i < plan.src_lo.size else -1,
                src_idx_hi=int(plan.src_hi[i]) if i < plan.src_hi.size else -1,
                interpolation_alpha=0.0,
                time_scale=float(args.time_scale),
                time_scale_mode=mode,
                effective_command_rate_hz=(effective_rates[-1] if effective_rates else rate_hz / float(args.time_scale)),
                hold=bool(plan.hold[i]) if i < plan.hold.size else False,
            ))
        time.sleep(period)
    arr = np.asarray(effective_rates, dtype=np.float64)
    writer.metadata.update({
        "time_scale_mode": mode,
        "wall_clock_dispatch_rate_hz": rate_hz / float(args.time_scale),
        "effective_command_rate_hz": {
            "p50": (float(np.percentile(arr, 50)) if arr.size else None),
            "p95": (float(np.percentile(arr, 95)) if arr.size else None),
            "min": (float(arr.min()) if arr.size else None),
        },
        "held_tick_count": int(np.count_nonzero(plan.hold)),
        "stale_or_repeated_target_count": 0,
        "tick_count": int(plan.t_servo.size),
    })


def _stream_wall_clock(
    stream: WallClockStream,
    plan: ReplayPlan,
    args: argparse.Namespace,
    server: ServerRuntimeConfig,
    client: ServoCommandClient,
    state_client: RobotStateClient,
    writer: TrajectoryLogWriter,
    rate_hz: float,
    mode: str,
) -> None:
    period = 1.0 / rate_hz
    effective_rates: list[float] = []
    last_wall: float | None = None
    start = time.perf_counter()
    for k in range(stream.size):
        check_watchdog(state_client, client, allow_controller_sim_arm_error=bool(args.allow_controller_sim_arm_error))
        snapshot = state_client.latest.payload if state_client.latest is not None else None
        left = stream.goals["left"][k] if "left" in plan.selected_arms else None
        right = stream.goals["right"][k] if "right" in plan.selected_arms else None
        left_g = _finite_float_or_none(stream.grippers["left"][k]) if "left" in plan.selected_arms else None
        right_g = _finite_float_or_none(stream.grippers["right"][k]) if "right" in plan.selected_arms else None
        send_twist = bool(getattr(args, "send_conditioned_twist", False))
        left_tw = _conditioned_twist_payload(stream.twists["left"][k], args.time_scale) if (send_twist and "left" in plan.selected_arms) else None
        right_tw = _conditioned_twist_payload(stream.twists["right"][k], args.time_scale) if (send_twist and "right" in plan.selected_arms) else None
        now = time.perf_counter()
        if last_wall is not None and now > last_wall:
            effective_rates.append(1.0 / (now - last_wall))
        last_wall = now
        client.send(
            tcp_pose_target_stand_intent(
                left=_pose_list(left),
                right=_pose_list(right),
                left_gripper=left_g,
                right_gripper=right_g,
                left_twist=left_tw,
                right_twist=right_tw,
                timeout_sec=server.command_timeout_sec,
            )
        )
        eff_rate = effective_rates[-1] if effective_rates else rate_hz
        for arm in plan.selected_arms:
            writer.append(log_row(
                t=float(stream.t_wall[k]),
                t_source=float(stream.t_episode[k]),
                src_idx=int(stream.src_idx_lo[k]),
                arm=arm,
                source_raw_target=stream.raw_targets[arm][k],
                conditioned_goal=stream.goals[arm][k],
                conditioned_twist=stream.twists[arm][k],
                snapshot=snapshot,
                t_episode=float(stream.t_episode[k]),
                src_idx_lo=int(stream.src_idx_lo[k]),
                src_idx_hi=int(stream.src_idx_hi[k]),
                interpolation_alpha=float(stream.interp_alpha[k]),
                time_scale=float(args.time_scale),
                time_scale_mode=mode,
                effective_command_rate_hz=eff_rate,
                hold=bool(stream.hold[k]),
            ))
        # Pace to wall-clock rate using absolute target times to avoid drift.
        target = start + (k + 1) * period
        sleep_for = target - time.perf_counter()
        if sleep_for > 0:
            time.sleep(sleep_for)
    writer.metadata.update(_wall_clock_meta(stream, plan, rate_hz, mode, effective_rates=effective_rates))


def check_watchdog(
    state_client: RobotStateClient,
    client: ServoCommandClient,
    *,
    allow_controller_sim_arm_error: bool = False,
) -> None:
    snapshot = state_client.latest
    if snapshot is None:
        raise WatchdogStop("no state snapshot")
    if state_client.is_latest_stale():
        raise WatchdogStop("stale state")
    cause = watchdog_cause(snapshot.payload, client, allow_controller_sim_arm_error=allow_controller_sim_arm_error)
    if cause:
        raise WatchdogStop(cause)


def watchdog_cause(
    payload: dict[str, Any],
    client: ServoCommandClient | None = None,
    *,
    allow_controller_sim_arm_error: bool = False,
) -> str | None:
    if payload.get("fault_latched") is True:
        return "fault_latched"
    verdict = payload.get("safety_verdict")
    if verdict in {"EmergencyStop", "IkFailed"}:
        return f"safety_verdict={verdict}"
    if payload.get("tracking_error_latched") is True or payload.get("tracking_error_degraded") is True:
        return "tracking_error"
    lease = payload.get("command_source") if isinstance(payload.get("command_source"), dict) else {}
    if client is not None and lease.get("enforce_lease") is True:
        if lease.get("active_source_id") != client.source_id or lease.get("active_session_id") != client.session_id:
            return "command_source_lease_lost"
    for arm in ARMS:
        arm_state = payload.get(arm)
        if not isinstance(arm_state, dict):
            continue
        if arm_state.get("has_error") is True:
            if allow_controller_sim_arm_error and arm_error_is_controller_sim_not_activated(payload, arm):
                _log_tolerated_controller_sim_arm_error(arm)
                continue
            return f"{arm}.has_error"
        solve = arm_state.get("cartesian_solve")
        if isinstance(solve, dict):
            for key in ("ik_failed", "ik_timed_out", "ik_branch_jump_clamped", "branch_jump_flag"):
                if solve.get(key) is True:
                    return f"{arm}.cartesian_solve.{key}"
    return None


def arm_error_is_controller_sim_not_activated(payload: dict[str, Any], arm: str) -> bool:
    if arm not in ARMS or not isinstance(payload, dict):
        return False
    if not _payload_is_controller_simulation(payload, arm):
        return False
    arm_state = payload.get(arm)
    if not isinstance(arm_state, dict):
        return False
    reasons = arm_state.get("startup_invalid_reasons")
    if not isinstance(reasons, (list, tuple, set)) or not reasons:
        return False
    reason_set = {str(item) for item in reasons}
    if not reason_set.issubset(CONTROLLER_SIM_NOT_ACTIVATED_REASONS):
        return False
    diagnostics = arm_state.get("rbpodo_diagnostics")
    raw = diagnostics.get("raw") if isinstance(diagnostics, dict) else None
    if not isinstance(raw, dict):
        return False
    for key in ("op_stat_ems_flag", "op_stat_sos_flag", "op_stat_soft_estop_occur", "op_stat_collision_occur"):
        if _int_value(raw.get(key)) != 0:
            return False
    self_collision = _int_value(raw.get("op_stat_self_collision"))
    if self_collision is None or self_collision == 1:
        return False
    error_code = _optional_int_value(arm_state.get("error_code"))
    init_error = _optional_int_value(raw.get("init_error"))
    if error_code is not None and init_error is not None and error_code not in {0, init_error}:
        return False
    diagnostic_source = arm_state.get("diagnostic_error_source")
    if diagnostic_source not in (None, "", "rbpodo_init_error"):
        return False
    return True


def _payload_is_controller_simulation(payload: dict[str, Any], arm: str) -> bool:
    for container in (payload, payload.get(arm)):
        if not isinstance(container, dict):
            continue
        for key in ("operation_mode", "observed_mode"):
            if str(container.get(key, "")).lower() == "simulation":
                return True
        for key in (
            "controller_simulation_cartesian_enabled",
            "controller_simulation_cartesian_enabled_for_current_command",
            "controller_simulation_streaming_cartesian_available",
            "allow_in_controller_simulation",
            "allow_controller_simulation_motion",
        ):
            if container.get(key) is True:
                return True
    physical_motion_expected = payload.get("physical_motion_expected")
    if physical_motion_expected is False and str(payload.get("operation_mode", "")).lower() == "simulation":
        return True
    return False


def _int_value(value: Any) -> int | None:
    try:
        if isinstance(value, bool):
            return int(value)
        if isinstance(value, int):
            return value
        if isinstance(value, float) and value.is_integer():
            return int(value)
        if isinstance(value, str) and value.strip():
            return int(value.strip())
    except ValueError:
        return None
    return None


def _optional_int_value(value: Any) -> int | None:
    if value in (None, ""):
        return None
    return _int_value(value)


def _log_tolerated_controller_sim_arm_error(arm: str) -> None:
    if arm in _TOLERATED_CONTROLLER_SIM_ARM_ERRORS:
        return
    _TOLERATED_CONTROLLER_SIM_ARM_ERRORS.add(arm)
    print(
        f"[controller-sim] tolerating {arm} not-activated has_error "
        "(servo_disabled/init_error); server-side IK still captured"
    )


def actual_tcp_pose_from_state(payload: dict[str, Any], arm: str) -> np.ndarray:
    arm_state = payload.get(arm)
    if not isinstance(arm_state, dict):
        raise ReplayRefusal(f"state missing {arm} arm")
    for key in ("tcp_actual_stand", "tcp_stand", "tcp_ref_stand"):
        raw = arm_state.get(key)
        try:
            return pose_dict_to_pose7(raw)
        except (TypeError, ValueError):
            continue
    raise ReplayRefusal(f"state missing finite {arm}.tcp_actual_stand/tcp_stand")


def actual_tcp_pose_from_state_strict(payload: dict[str, Any], arm: str) -> np.ndarray:
    arm_state = payload.get(arm)
    if not isinstance(arm_state, dict):
        raise ReplayRefusal(f"state missing {arm} arm")
    try:
        return pose_dict_to_pose7(arm_state.get("tcp_actual_stand"))
    except (TypeError, ValueError) as exc:
        raise ReplayRefusal(f"state missing finite live {arm}.tcp_actual_stand") from exc


def live_anchor_pose_from_state(payload: dict[str, Any], arm: str, source: str = "auto") -> np.ndarray:
    """Live anchor pose for ee_local replay.

    In rbpodo pgmode controller-simulation the physical arm is stationary, so the
    published ``tcp_actual_stand`` is FROZEN while the controller *reference*
    (``tcp_ref_stand``) is the pose the servo loop actually continues from. Anchoring
    a new episode on the frozen actual yanks the reference back to a stale pose at
    each replay start (the inter-episode "fast catch-up"). ``auto`` therefore prefers
    the reference and falls back to the actual; ``reference``/``actual`` force one.
    """
    arm_state = payload.get(arm)
    if not isinstance(arm_state, dict):
        raise ReplayRefusal(f"state missing {arm} arm")
    if source == "actual":
        order = ("tcp_actual_stand",)
    elif source == "reference":
        order = ("tcp_ref_stand",)
    else:  # auto
        order = ("tcp_ref_stand", "tcp_actual_stand")
    for key in order:
        try:
            return pose_dict_to_pose7(arm_state.get(key))
        except (TypeError, ValueError):
            continue
    raise ReplayRefusal(f"state missing finite live {arm} anchor (source={source}, tried {order})")


def pose_dict_to_pose7(raw: Any) -> np.ndarray:
    if not isinstance(raw, dict):
        raise TypeError("pose is not a mapping")
    q = raw.get("quaternion_xyzw")
    if not isinstance(q, (list, tuple)) or len(q) != 4:
        raise ValueError("pose quaternion_xyzw missing")
    return canonical_pose7([raw["x"], raw["y"], raw["z"], *q])


def canonical_pose7(values: Any) -> np.ndarray:
    arr = np.asarray(values, dtype=np.float64).reshape(7)
    if not np.isfinite(arr).all():
        raise ReplayRefusal("pose contains non-finite values")
    arr[3:7] = quat_canonical(arr[3:7])
    return arr


def quat_angle_rad(a: Any, b: Any) -> float:
    qa = quat_canonical(a)
    qb = quat_canonical(b, ref=qa)
    dot = abs(float(np.dot(qa, qb)))
    return 2.0 * math.acos(min(1.0, max(-1.0, dot)))


def log_row(
    *,
    t: float,
    t_source: float,
    src_idx: int,
    arm: str,
    source_raw_target: np.ndarray,
    conditioned_goal: np.ndarray,
    conditioned_twist: np.ndarray,
    snapshot: dict[str, Any] | None,
    t_episode: float | None = None,
    src_idx_lo: int | None = None,
    src_idx_hi: int | None = None,
    interpolation_alpha: float | None = None,
    time_scale: float | None = None,
    time_scale_mode: str | None = None,
    effective_command_rate_hz: float | None = None,
    hold: bool | None = None,
) -> dict[str, Any]:
    arm_state = snapshot.get(arm, {}) if isinstance(snapshot, dict) else {}
    actual = _state_pose_or_nan(arm_state, "tcp_actual_stand")
    # Controller reference pose (tcp_ref_stand). In rbpodo pgmode controller-sim the
    # physical arm is stationary so tcp_actual_stand is FROZEN, but tcp_ref_stand is
    # what the servo loop actually tracks — it is the meaningful tracking series there.
    reference = _state_pose_or_nan(arm_state, "tcp_ref_stand")
    q_actual = arm_state.get("q_actual_deg") if isinstance(arm_state, dict) else None
    q_target = arm_state.get("q_target_deg") if isinstance(arm_state, dict) else None
    solve = arm_state.get("cartesian_solve") if isinstance(arm_state, dict) and isinstance(arm_state.get("cartesian_solve"), dict) else {}
    # Safety telemetry lives in top-level objects (per-arm sub-dicts) plus the
    # per-arm cartesian_solve clamp flags. The legacy *_active snapshot keys do
    # not exist in the published state schema (robotics_lab.servo_state.v1).
    snap = snapshot if isinstance(snapshot, dict) else {}
    roi_box = snap.get("roi_box") if isinstance(snap.get("roi_box"), dict) else {}
    floor_obj = snap.get("floor_constraint") if isinstance(snap.get("floor_constraint"), dict) else {}
    self_coll = snap.get("self_collision") if isinstance(snap.get("self_collision"), dict) else {}
    roi_arm = roi_box.get(arm) if isinstance(roi_box.get(arm), dict) else {}
    floor_arm = floor_obj.get(arm) if isinstance(floor_obj.get(arm), dict) else {}
    verdict = str(snap.get("safety_verdict", "")).strip().lower()
    row = {
        "t": t,
        "t_source": t_source,
        "src_idx": src_idx,
        "arm": arm,
        "source_raw_target": np.asarray(source_raw_target, dtype=np.float64),
        "conditioned_goal_after_A": np.asarray(conditioned_goal, dtype=np.float64),
        "conditioned_twist_after_A": np.asarray(conditioned_twist, dtype=np.float64),
        "reference_after_B": reference,
        "q_target": _vec_or_nan(q_target, 6),
        "q_actual": _vec_or_nan(q_actual, 6),
        "actual_tcp": actual,
        "ik_solve_us": solve.get("ik_duration_us", np.nan),
        "ik_pos_err": solve.get("position_error_m", np.nan),
        "ik_ori_err": solve.get("orientation_error_rad", np.nan),
        "ik_solution_jump_deg": solve.get("ik_solution_jump_deg", np.nan),
        "ik_min_singular_value": solve.get("ik_min_singular_value", np.nan),
        "ik_applied_damping": solve.get("ik_applied_damping", np.nan),
        "ik_status": solve.get("ik_status", ""),
        "ik_reason": solve.get("ik_reason", ""),
        "ik_branch_jump_clamped": _boolish(solve.get("ik_branch_jump_clamped")),
        "branch_jump_flag": _boolish(solve.get("ik_branch_jump_suspected")),
        "singular_damping_flag": _boolish(solve.get("singular_damping_flag")),
        "safety_proj_flag": _boolish(bool(verdict) and verdict != "ok"),
        "self_collision_flag": _boolish(bool(self_coll.get("violated"))),
        "floor_flag": _boolish(bool(floor_arm.get("violated")) or bool(solve.get("floor_goal_clamped"))),
        "roi_flag": _boolish(bool(roi_arm.get("violated"))),
        # SMD / velocity clamp (the controller-side speed-limit signal): when the
        # streamed goal exceeds the SMD max velocity the reference is rate-limited.
        "smd_goal_clamped_flag": _boolish(bool(solve.get("twist_smd_goal_clamped")) or bool(solve.get("twist_clamped"))),
        "self_collision_min_clearance_m": _finite_or_nan(self_coll.get("min_clearance_m")),
        "roi_min_margin_m": _finite_or_nan(roi_arm.get("min_margin_m")),
        "floor_tcp_z_m": _finite_or_nan(floor_arm.get("tcp_z_m")),
        # command-vs-reference tracking error (deg) — the latch-relevant series in
        # pgmode (physical q_actual is frozen so physical_command_actual is N/A).
        "command_reference_tracking_error_deg": _finite_or_nan(
            arm_state.get("command_reference_tracking_error_deg") if isinstance(arm_state, dict) else None
        ),
        # Patch 4: A/B/C separation telemetry (from cartesian_solve when the server
        # publishes it; NaN/absent on older servers -> analyzer falls back + warns).
        "smd_ref_stand": _state_pose_or_nan(solve, "smd_ref_stand"),
        "smd_goal_stand": _state_pose_or_nan(solve, "smd_goal_stand"),
        "q_target_before_output_ma_deg": _vec_or_nan(solve.get("q_target_before_output_ma_deg"), 6),
        "q_target_after_output_ma_deg": _vec_or_nan(solve.get("q_target_after_output_ma_deg"), 6),
        "smd_velocity_feedforward_used": _boolish(solve.get("smd_velocity_feedforward_used")),
        "smd_velocity_feedforward_source": solve.get("smd_velocity_feedforward_source", ""),
        "smd_velocity_feedforward_fallback": _boolish(solve.get("smd_velocity_feedforward_fallback")),
        "smd_linear_velocity_clipped": _boolish(solve.get("smd_linear_velocity_clipped")),
        "smd_linear_accel_clipped": _boolish(solve.get("smd_linear_accel_clipped")),
        "smd_angular_velocity_clipped": _boolish(solve.get("smd_angular_velocity_clipped")),
        "smd_angular_accel_clipped": _boolish(solve.get("smd_angular_accel_clipped")),
        "smd_goal_linear_velocity_ff_clipped": _boolish(solve.get("smd_goal_linear_velocity_ff_clipped")),
        "smd_goal_angular_velocity_ff_clipped": _boolish(solve.get("smd_goal_angular_velocity_ff_clipped")),
        "smd_goal_linear_velocity_norm_m_s": _finite_or_nan(solve.get("smd_goal_linear_velocity_norm_m_s")),
        "smd_goal_angular_velocity_norm_rad_s": _finite_or_nan(solve.get("smd_goal_angular_velocity_norm_rad_s")),
        "smd_reanchor_count": _finite_or_nan(solve.get("smd_reanchor_count")),
        "output_ma_window": _finite_or_nan(solve.get("output_ma_window")),
    }
    # Patch 1: wall-clock dispatch / resampling telemetry (only set when provided).
    for key, value in (
        ("t_episode", t_episode),
        ("src_idx_lo", src_idx_lo),
        ("src_idx_hi", src_idx_hi),
        ("interpolation_alpha", interpolation_alpha),
        ("time_scale", time_scale),
        ("time_scale_mode", time_scale_mode),
        ("effective_command_rate_hz", effective_command_rate_hz),
        ("hold", hold),
    ):
        if value is not None:
            row[key] = value
    if isinstance(snapshot, dict):
        row.update(
            {
                "fault_latched": snapshot.get("fault_latched"),
                "safety_verdict": snapshot.get("safety_verdict"),
                "motion_state": snapshot.get("motion_state"),
                "tracking_error_degraded": snapshot.get("tracking_error_degraded"),
            }
        )
    return row


def run_metadata(plan: ReplayPlan, args: argparse.Namespace, server: ServerRuntimeConfig) -> dict[str, Any]:
    return {
        "git_commit": git_commit(),
        "phase1_git_commit": phase1_git_commit(),
        "server_config_path": str(server.path),
        "npz_path": str(plan.paths.npz_path),
        "episode_id": plan.paths.episode_id,
        "seed": args.seed if args.seed is not None else -1,
        "dry_run": plan.dry_run,
        "params": {k: _jsonable(v) for k, v in vars(args).items()},
        "selected_arms": list(plan.selected_arms),
        "init_deltas": {arm: asdict(delta) for arm, delta in plan.init_deltas.items()},
        "stream_speed_stats": plan.stream_speed_stats,
        "bounds_notes": plan.bounds_notes,
        "segment_selection": plan.segment_selection,
        "time_scale": float(args.time_scale),
        "time_scale_mode": str(getattr(args, "time_scale_mode", "wall_clock_resample")),
        "wall_clock_dispatch_rate_hz": float(args.rate_hz or server.servo_rate_hz),
        "conditioning_config": _conditioning_config_dump(args, server),
    }


def _conditioning_config_dump(args: argparse.Namespace, server: ServerRuntimeConfig) -> dict[str, Any]:
    """Effective A-stage conditioning/smoothing config (Patch 2), for reproducibility."""

    rate_hz = float(args.rate_hz or server.servo_rate_hz)
    cfg = _phase1_config(args, rate_hz)
    return {
        "conditioning": _jsonable(cfg.conditioning.__dict__),
        "smoothing": _jsonable(cfg.smoothing.__dict__),
        "conditioning_config_path": getattr(args, "conditioning_config", None),
    }


def git_commit() -> str | None:
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True, stderr=subprocess.DEVNULL).strip()
    except Exception:
        return None


def print_plan_summary(plan: ReplayPlan, server: ServerRuntimeConfig, args: argparse.Namespace) -> None:
    print(f"server_config={server.path}")
    print(f"npz={plan.paths.npz_path}")
    print(f"run_dir={plan.paths.run_dir}")
    print(f"ticks={plan.t_servo.size} arms={','.join(plan.selected_arms)} rate_hz={args.rate_hz or server.servo_rate_hz} time_scale={args.time_scale}")
    print(
        f"client_speed_clamp: linear={float(args.max_linear_speed_m_s):.6f} m/s "
        f"({getattr(args, '_max_linear_speed_source', 'unknown')}) "
        f"angular={float(args.max_angular_speed_rad_s):.6f} rad/s "
        f"({getattr(args, '_max_angular_speed_source', 'unknown')})"
    )
    print_segment_summary(plan.segment_selection)
    for note in plan.bounds_notes:
        print(f"bounds: {note}")
    for arm in plan.selected_arms:
        arr = plan.goals[arm]
        mn = np.min(arr[:, :3], axis=0)
        mx = np.max(arr[:, :3], axis=0)
        first = arr[0]
        last = arr[-1]
        delta = plan.init_deltas[arm]
        speeds = plan.stream_speed_stats[arm]
        print(f"{arm} init_delta: linear={delta.linear_m:.6f} m angular={delta.angular_deg:.3f} deg")
        print(f"{arm} first_pose: {format_pose(first)}")
        print(f"{arm} last_pose: {format_pose(last)}")
        print(f"{arm} setpoint_min_xyz: {mn.tolist()}")
        print(f"{arm} setpoint_max_xyz: {mx.tolist()}")
        print(
            f"{arm} stream_speed_max: linear={speeds['max_linear_m_s']:.6f} m/s "
            f"angular={speeds['max_angular_rad_s']:.6f} rad/s"
        )
        if speeds.get("exceeds_linear_limit") or speeds.get("exceeds_angular_limit"):
            print(
                f"{arm} execute would abort: stream exceeds client clamp "
                f"linear_limit={speeds['linear_limit_m_s']:.6f} m/s "
                f"angular_limit={speeds['angular_limit_rad_s']:.6f} rad/s"
            )
        init = plan.init_poses[arm]
        print(f"{arm} planned init pre-move: {len(init)} setpoints, first={format_pose(init[0])}, final={format_pose(init[-1])}")
    if not plan_exceeds_stream_limits(plan):
        print("execute stream clamp precheck ok: max speeds within client clamp")
    if plan.would_abort_large_init and not args.allow_large_init_move:
        print("execute would abort: initial pose delta exceeds threshold")
    if plan.dry_run:
        print(f"would-be log written: {plan.paths.log_path}")
    else:
        print(f"runtime log path: {plan.paths.log_path}")


def confirm_start(selected_arms: tuple[str, ...], args: argparse.Namespace | None = None) -> None:
    # Interactive typed gate removed by operator request. Motion still requires the
    # explicit --execute --i-am-at-the-estop flags (dry-run remains the default), and
    # the operator launches this command with an E-stop in hand. The typed prompt also
    # created a command gap during typing that dropped the command-source lease at
    # stream start (false command_source_lease_lost), so removing it fixes that too.
    print(f"Physical motion gate 1: confirmed for {','.join(selected_arms)} (interactive prompt removed)")


def confirm_after_init(selected_arms: tuple[str, ...], args: argparse.Namespace | None = None) -> None:
    # See confirm_start: no typed prompt; streaming proceeds immediately after the init
    # pre-move so the command-source lease stays continuously held.
    print(f"Physical motion gate 2 (after init pre-move): streaming {','.join(selected_arms)} (interactive prompt removed)")


def wait_for_fresh_state(client: RobotStateClient, *, timeout_sec: float) -> None:
    deadline = time.monotonic() + max(timeout_sec, 0.0)
    while time.monotonic() < deadline:
        if client.latest is not None and not client.is_latest_stale():
            return
        time.sleep(0.01)
    raise ReplayRefusal("no fresh state before execute")


def print_segment_summary(selection: dict[str, Any]) -> None:
    if not selection:
        print("segment_selection: unavailable")
        return
    segments = selection.get("all_segments") or []
    if selection.get("mode") == "single":
        frame_range = selection.get("source_frame_range") or []
        if len(frame_range) == 2:
            range_text = f"{int(frame_range[0])}-{int(frame_range[1])}"
        else:
            range_text = "empty"
        print(
            "segment_selection: "
            f"selected segment={selection.get('segment_index')} source_frames={range_text} "
            f"frames={selection.get('selected_frame_count')} "
            f"dropped={selection.get('dropped_frame_count')} "
            f"(before={selection.get('dropped_before')}, after={selection.get('dropped_after')}) "
            f"total_segments={len(segments)} gaps={selection.get('gap_count')}"
        )
        return
    print(
        "segment_selection: "
        f"all source_frames={selection.get('source_frame_count')} "
        f"total_segments={len(segments)} gaps={selection.get('gap_count')} dropped=0"
    )


def _meta_from_npz(data: Any) -> dict[str, Any]:
    if "meta_json" in data.files:
        try:
            meta = json.loads(str(np.asarray(data["meta_json"]).item()))
            return meta if isinstance(meta, dict) else {}
        except Exception:
            pass
    return {}


def _episode_id_from_meta_or_path(path: Path, meta: dict[str, Any]) -> str:
    if meta.get("episode_id"):
        return str(meta["episode_id"])
    return path.parent.name if path.parent.name else path.stem


def _pose_at_fractional_index(poses: np.ndarray | None, i: int, count: int) -> np.ndarray | None:
    if poses is None:
        return None
    if poses.shape[0] == count:
        return poses[min(i, poses.shape[0] - 1)]
    u = i / max(count - 1, 1)
    idx = int(round(u * (poses.shape[0] - 1)))
    return poses[min(idx, poses.shape[0] - 1)]


def _pose_list(pose: np.ndarray | None) -> list[float] | None:
    if pose is None:
        return None
    return [float(v) for v in np.asarray(pose, dtype=np.float64).reshape(7)]


def _state_pose_or_nan(arm_state: Any, key: str) -> np.ndarray:
    if isinstance(arm_state, dict):
        try:
            return pose_dict_to_pose7(arm_state.get(key))
        except Exception:
            pass
    return np.full(7, np.nan, dtype=np.float64)


def _vec_or_nan(value: Any, size: int) -> np.ndarray:
    try:
        arr = np.asarray(value, dtype=np.float64).reshape(size)
        if np.isfinite(arr).all():
            return arr
    except Exception:
        pass
    return np.full(size, np.nan, dtype=np.float64)


def _vec3_or_none(value: Any) -> tuple[float, float, float] | None:
    try:
        arr = np.asarray(value, dtype=np.float64).reshape(3)
    except Exception:
        return None
    if not np.isfinite(arr).all():
        return None
    return tuple(float(v) for v in arr)


def _float_or_none(value: Any) -> float | None:
    try:
        v = float(value)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def _finite_float_or_none(value: Any) -> float | None:
    v = _float_or_none(value)
    return v


def _row_or_none(array: np.ndarray | None, index: int) -> np.ndarray | None:
    if array is None or index >= array.shape[0]:
        return None
    return np.asarray(array[index], dtype=np.float64)


def _value_or_none(array: np.ndarray | None, index: int) -> float | None:
    if array is None or index >= array.shape[0]:
        return None
    return float(array[index])


def _boolish(value: Any) -> bool | float:
    if isinstance(value, bool):
        return value
    return np.nan


def _finite_or_nan(value: Any) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return float("nan")
    return out if np.isfinite(out) else float("nan")


def _jsonable(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    return value


def format_pose(pose: np.ndarray) -> str:
    values = [float(v) for v in np.asarray(pose, dtype=np.float64).reshape(7)]
    return "[" + ", ".join(f"{v:.6f}" for v in values) + "]"


if __name__ == "__main__":
    raise SystemExit(main())
