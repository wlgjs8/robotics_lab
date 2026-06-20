#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
TOOLS = ROOT / "tools"
POLICY_RUNNER = ROOT / "policy_runner"
for _path in (str(ROOT), str(TOOLS), str(POLICY_RUNNER)):
    if _path not in sys.path:
        sys.path.insert(0, _path)

from analyze_tcp_replay_logs import analyze_inputs, load_input, render_summary
from scripts import replay_episode_tcp_pose_target as driver
from tcp_tuning.config import Config as MetricsRootConfig
from tcp_tuning.trajectory_log import TrajectoryLogReader

from policy_runner.robot_state_client import RobotStateClient, StateStreamLeaseReadback
from policy_runner.servo_command_client import CommandIntent, ServoCommandClient


ARMS = ("left", "right")
DEFAULT_EPISODES_DIR = ROOT / "data_tcp" / "data_20260619_115712"
DEFAULT_OUT_DIR = ROOT / "outputs" / "tcp_tuning"
# Validated folded stow JointTarget from scripts/replay_episode_rollout.py.
REST_LEFT_DEG = (-131.663, 72.989, 113.400, -80.880, -107.064, -145.949)
REST_RIGHT_DEG = (135.099, -64.017, -114.457, 84.379, 112.485, 129.893)


@dataclass(frozen=True)
class JointTargets:
    left: tuple[float, ...]
    right: tuple[float, ...]


@dataclass
class InitReturnResult:
    start_delta_deg: dict[str, float]
    final_delta_deg: dict[str, float] | None
    arrived: bool
    fault: str | None = None
    timeout: bool = False


@dataclass
class EpisodeBatchResult:
    episode: str
    episode_id: str
    status: str
    run_dir: str
    log_path: str | None
    metrics_path: str | None
    segment: str | None
    bounds: dict[str, Any]
    max_stream_speed: dict[str, Any]
    init_return: InitReturnResult
    floor_roi: str
    actual_vs_goal_p95_mm: dict[str, float | None]
    branch_jump_count: dict[str, int | None]
    ik_solve_us_p95: dict[str, float | None]
    refusal: str | None = None


class BatchHalt(RuntimeError):
    pass


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    return run_batch(args)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Fail-closed batch replay harness for ee_local TcpPoseTarget episodes.")
    parser.add_argument("--episodes-dir", default=str(DEFAULT_EPISODES_DIR))
    parser.add_argument("--episodes", default=None, help="Comma-separated episode numbers/stems, for example 000,001.")
    parser.add_argument("--source", default="ee_local", choices=["ee_local"])
    parser.add_argument("--mode", default="clean_foh_se3", choices=["clean_foh_se3"])
    parser.add_argument("--segment", default="auto-largest")
    parser.add_argument("--action-scale", type=float, default=1.0)
    parser.add_argument("--time-scale", type=float, default=2.0)
    parser.add_argument("--server-config", default=None)
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT_DIR))
    parser.add_argument("--arms", default="left,right")
    parser.add_argument("--anchor", choices=["live", "mock"], default=None)
    parser.add_argument("--mock-current-pose", default="default")
    parser.add_argument("--mock-q-actual", default=None, help="'rest_stow', six joint degrees, or JSON/list/dict for dry-run init delta.")
    parser.add_argument("--max-linear-speed-m-s", type=float, default=None)
    parser.add_argument("--max-angular-speed-rad-s", type=float, default=None)
    parser.add_argument("--init-mode", choices=["capture_current", "rest_stow", "joints"], default="capture_current")
    parser.add_argument("--init-left-joints", default=None)
    parser.add_argument("--init-right-joints", default=None)
    parser.add_argument("--init-tol-deg", type=float, default=1.0)
    parser.add_argument("--init-timeout-sec", type=float, default=20.0)
    parser.add_argument("--init-lease-grace-sec", type=float, default=0.4)
    parser.add_argument("--dwell-sec", type=float, default=0.5)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--i-am-at-the-estop", action="store_true")
    parser.add_argument(
        "--skip-failed-episodes",
        action="store_true",
        help="On a per-episode failure (pre-flight REFUSED or mid-motion fault), record it and CONTINUE to the next episode instead of halting the whole batch. Intended for pgmode-sim data collection (no physical motion). Default off keeps the fail-closed halt for real motion.",
    )
    parser.add_argument("--source-id", default="tcp_pose_batch_replay")
    parser.add_argument("--state-timeout-sec", type=float, default=2.0)
    parser.add_argument(
        "--allow-controller-sim-arm-error",
        action="store_true",
        help="Pass through replay-only rbpodo pgmode controller-sim not-activated arm-error tolerance.",
    )
    return parser.parse_args(argv)


def run_batch(
    args: argparse.Namespace,
    *,
    driver_runner: Callable[[list[str]], subprocess.CompletedProcess[str]] | None = None,
) -> int:
    episodes = discover_episodes(Path(args.episodes_dir), args.episodes)
    dry_run = not (bool(args.execute) and not bool(args.dry_run) and bool(args.i_am_at_the_estop))
    if dry_run:
        print("BATCH DRY RUN — no motion sent")
    if not episodes:
        raise BatchHalt("no episodes selected")
    if args.init_tol_deg <= 0.0:
        raise BatchHalt("--init-tol-deg must be positive")
    if args.init_timeout_sec <= 0.0:
        raise BatchHalt("--init-timeout-sec must be positive")
    if args.init_lease_grace_sec < 0.0:
        raise BatchHalt("--init-lease-grace-sec must be non-negative")

    server = None if dry_run and not args.server_config else driver.load_server_config(_server_config_path(args))
    init_target = resolve_init_target(args, dry_run=dry_run, server=server)
    if args.init_mode == "rest_stow":
        print("WARNING: init-mode rest_stow uses the folded stow pose; replay from folded stow may be out-of-distribution.")
    if not dry_run:
        confirm_batch_execute(len(episodes), args.init_mode)

    batch_name = "batch_" + datetime.now().strftime("%Y%m%dT%H%M%S")
    out_dir = Path(args.out_dir)
    batch_dir = out_dir / batch_name
    batch_dir.mkdir(parents=True, exist_ok=True)
    runner = driver_runner or run_episode_driver
    results: list[EpisodeBatchResult] = []
    halted = False
    refusal: str | None = None

    for episode_path in episodes:
        episode_id = driver.hdf5_episode_id(episode_path)
        run_dir = out_dir / episode_id / "runs" / batch_name
        current_q = resolve_current_q_for_delta(args, dry_run=dry_run, server=server, fallback=init_target)
        init_delta = compute_joint_delta(current_q, init_target)
        init_result = InitReturnResult(start_delta_deg=init_delta, final_delta_deg=init_delta, arrived=True)
        try:
            if dry_run:
                print(f"[dry-run] {episode_path.name}: init-return max_delta_deg={max(init_delta.values()):.3f}")
            else:
                if server is None:
                    raise BatchHalt("server config is required for execute")
                init_result = return_to_init_pose(args, server, init_target)
                time.sleep(float(args.dwell_sec))

            completed = runner(build_driver_command(args, episode_path, batch_name, dry_run=dry_run))
            if completed.stdout:
                print(completed.stdout, end="" if completed.stdout.endswith("\n") else "\n")
            if completed.stderr:
                print(completed.stderr, file=sys.stderr, end="" if completed.stderr.endswith("\n") else "\n")
            if completed.returncode != 0:
                if not dry_run and server is not None:
                    send_hold_best_effort(args, server)
                raise BatchHalt(f"driver exited {completed.returncode} for {episode_path.name}")

            metrics_path, tracking, branch, ik = write_episode_metrics(run_dir)
            results.append(build_episode_result(episode_path, run_dir, init_result, metrics_path, tracking, branch, ik))
            time.sleep(float(args.dwell_sec))
        except BatchHalt as exc:
            cause = str(exc)
            results.append(build_failed_result(episode_path, run_dir, init_result, cause))
            if getattr(args, "skip_failed_episodes", False):
                print(f"SKIP (continue): {episode_path.name}: {cause}", file=sys.stderr)
                continue
            halted = True
            refusal = cause
            print(f"BATCH HALT: {refusal}", file=sys.stderr)
            break

    write_batch_summary(batch_dir, args, episodes, results, halted=halted, refusal=refusal, dry_run=dry_run)
    print(f"batch_summary_json={batch_dir / 'batch_summary.json'}")
    print(f"batch_summary_md={batch_dir / 'batch_summary.md'}")
    return 1 if halted else 0


def discover_episodes(episodes_dir: Path, subset: str | None) -> list[Path]:
    if not episodes_dir.exists():
        raise BatchHalt(f"episodes directory does not exist: {episodes_dir}")
    all_paths = sorted(episodes_dir.glob("episode_*.hdf5"))
    if not subset:
        return all_paths
    wanted = {normalize_episode_token(token) for token in subset.split(",") if token.strip()}
    return [path for path in all_paths if path.stem in wanted]


def normalize_episode_token(token: str) -> str:
    text = token.strip()
    if text.startswith("episode_"):
        return text
    return f"episode_{int(text):03d}"


def _server_config_path(args: argparse.Namespace) -> Path:
    return Path(args.server_config) if args.server_config else driver.default_server_config()


def resolve_init_target(args: argparse.Namespace, *, dry_run: bool, server: driver.ServerRuntimeConfig | None) -> JointTargets:
    if args.init_mode == "rest_stow":
        return JointTargets(tuple(REST_LEFT_DEG), tuple(REST_RIGHT_DEG))
    if args.init_mode == "joints":
        if not args.init_left_joints or not args.init_right_joints:
            raise BatchHalt("--init-mode joints requires --init-left-joints and --init-right-joints")
        return JointTargets(parse_joint_list(args.init_left_joints), parse_joint_list(args.init_right_joints))
    if dry_run:
        if not args.mock_q_actual:
            raise BatchHalt("--init-mode capture_current dry-run requires --mock-q-actual")
        return parse_mock_q_actual(args.mock_q_actual)
    if server is None:
        raise BatchHalt("server config is required for capture_current execute")
    q = read_live_joints(server, timeout_sec=float(args.state_timeout_sec))
    return JointTargets(tuple(q["left"]), tuple(q["right"]))


def resolve_current_q_for_delta(
    args: argparse.Namespace,
    *,
    dry_run: bool,
    server: driver.ServerRuntimeConfig | None,
    fallback: JointTargets,
) -> JointTargets:
    if dry_run:
        return parse_mock_q_actual(args.mock_q_actual) if args.mock_q_actual else fallback
    if server is None:
        raise BatchHalt("server config is required for execute")
    q = read_live_joints(server, timeout_sec=float(args.state_timeout_sec))
    return JointTargets(tuple(q["left"]), tuple(q["right"]))


def parse_mock_q_actual(value: str) -> JointTargets:
    if value == "rest_stow":
        return JointTargets(tuple(REST_LEFT_DEG), tuple(REST_RIGHT_DEG))
    try:
        raw = json.loads(value)
    except json.JSONDecodeError:
        joints = parse_joint_list(value)
        return JointTargets(joints, joints)
    if isinstance(raw, list):
        joints = tuple(float(item) for item in raw)
        if len(joints) != 6:
            raise BatchHalt("--mock-q-actual list must contain six values")
        return JointTargets(joints, joints)
    if isinstance(raw, dict):
        return JointTargets(parse_joint_values(raw.get("left"), "--mock-q-actual.left"), parse_joint_values(raw.get("right"), "--mock-q-actual.right"))
    raise BatchHalt("--mock-q-actual must be rest_stow, six joints, or JSON")


def parse_joint_list(text: str) -> tuple[float, ...]:
    return parse_joint_values([item.strip() for item in str(text).split(",") if item.strip()], "joint list")


def parse_joint_values(value: Any, name: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)):
        raise BatchHalt(f"{name} must contain six joint degrees")
    joints = tuple(float(item) for item in value)
    if len(joints) != 6 or not np.isfinite(np.asarray(joints, dtype=np.float64)).all():
        raise BatchHalt(f"{name} must contain six finite joint degrees")
    return joints


def read_live_joints(server: driver.ServerRuntimeConfig, *, timeout_sec: float) -> dict[str, list[float]]:
    state_client = RobotStateClient(bind=server.state_bind or "", stale_timeout_sec=timeout_sec)
    try:
        state_client.start()
        driver.wait_for_fresh_state(state_client, timeout_sec=timeout_sec)
        payload = state_client.latest.payload
        return extract_q_actual(payload)
    finally:
        state_client.close()


def extract_q_actual(payload: dict[str, Any]) -> dict[str, list[float]]:
    out: dict[str, list[float]] = {}
    for arm in ARMS:
        arm_state = payload.get(arm) if isinstance(payload, dict) else None
        q = arm_state.get("q_actual_deg") if isinstance(arm_state, dict) else None
        if not isinstance(q, (list, tuple)) or len(q) != 6:
            raise BatchHalt(f"state missing {arm}.q_actual_deg")
        values = [float(item) for item in q]
        if not np.isfinite(np.asarray(values, dtype=np.float64)).all():
            raise BatchHalt(f"state {arm}.q_actual_deg contains non-finite values")
        out[arm] = values
    return out


def compute_joint_delta(current: JointTargets, target: JointTargets) -> dict[str, float]:
    return {
        "left": max_abs_joint_error(current.left, target.left),
        "right": max_abs_joint_error(current.right, target.right),
    }


def max_abs_joint_error(actual: Any, target: Any) -> float:
    a = np.asarray(actual, dtype=np.float64).reshape(6)
    b = np.asarray(target, dtype=np.float64).reshape(6)
    return float(np.max(np.abs(a - b)))


def return_to_init_pose(args: argparse.Namespace, server: driver.ServerRuntimeConfig, target: JointTargets) -> InitReturnResult:
    state_client = RobotStateClient(bind=server.state_bind or "", stale_timeout_sec=float(args.state_timeout_sec))
    command_client = ServoCommandClient(server.command_endpoint, timeout_sec=float(server.command_timeout_sec), source_id=str(args.source_id))
    try:
        state_client.start()
        driver.wait_for_fresh_state(state_client, timeout_sec=float(args.state_timeout_sec))
        start_q = _targets_from_payload(state_client.latest.payload)
        command_client.acquire_lease(StateStreamLeaseReadback(state_client), timeout_sec=4.0)
        # Clear any fault latched by a PREVIOUS episode (e.g. a singularity IkFailed),
        # otherwise this episode's init-return motion is blocked by fault_latched and
        # every subsequent episode skips. reset_fault is a no-op when nothing is latched.
        snap = state_client.latest
        if snap is not None and isinstance(snap.payload, dict) and snap.payload.get("fault_latched"):
            command_client.send(CommandIntent.reset_fault(timeout_sec=server.command_timeout_sec))
            time.sleep(0.5)
            driver.wait_for_fresh_state(state_client, timeout_sec=float(args.state_timeout_sec))
        command_client.send(CommandIntent.arm_motion(timeout_sec=server.command_timeout_sec))
        command_client.send(
            CommandIntent.joint_target(
                left=target.left,
                right=target.right,
                timeout_sec=max(float(args.init_timeout_sec), server.command_timeout_sec),
            )
        )
        time.sleep(0.3)
        result = drive_joint_target_until_arrived(
            state_client,
            command_client,
            server,
            target,
            tol_deg=float(args.init_tol_deg),
            timeout_sec=float(args.init_timeout_sec),
            init_lease_grace_sec=float(args.init_lease_grace_sec),
            allow_controller_sim_arm_error=bool(args.allow_controller_sim_arm_error),
        )
        result.start_delta_deg = compute_joint_delta(start_q, target)
        if not result.arrived:
            if result.fault:
                raise BatchHalt(f"init-return fault: {result.fault}")
            if result.timeout:
                raise BatchHalt(f"init-return timeout after {float(args.init_timeout_sec):.1f}s")
            raise BatchHalt("init-return failed")
        return result
    finally:
        try:
            command_client.release_lease()
        except Exception:
            pass
        command_client.close()
        state_client.close()
        time.sleep(float(args.dwell_sec))


def send_hold_best_effort(args: argparse.Namespace, server: driver.ServerRuntimeConfig) -> None:
    command_client = ServoCommandClient(server.command_endpoint, timeout_sec=float(server.command_timeout_sec), source_id=str(args.source_id))
    try:
        command_client.send(CommandIntent.hold(timeout_sec=server.command_timeout_sec))
    except Exception:
        pass
    finally:
        command_client.close()


def drive_joint_target_until_arrived(
    state_client: Any,
    command_client: Any,
    server: driver.ServerRuntimeConfig,
    target: JointTargets,
    *,
    tol_deg: float,
    timeout_sec: float,
    period_sec: float = 0.05,
    init_lease_grace_sec: float = 0.4,
    allow_controller_sim_arm_error: bool = False,
) -> InitReturnResult:
    start = time.monotonic()
    deadline = start + timeout_sec
    lease_grace_deadline = start + init_lease_grace_sec
    jt = CommandIntent.joint_target(left=target.left, right=target.right, timeout_sec=max(timeout_sec, server.command_timeout_sec))
    last_delta: dict[str, float] | None = None
    while time.monotonic() < deadline:
        command_client.send(jt)
        snapshot = getattr(state_client, "latest", None)
        if snapshot is None:
            cause = "no robot state during init return"
        elif state_client.is_latest_stale():
            cause = "stale robot state during init return"
        else:
            cause = driver.watchdog_cause(
                snapshot.payload,
                command_client,
                allow_controller_sim_arm_error=allow_controller_sim_arm_error,
            )
        if cause:
            now = time.monotonic()
            if (
                cause == "command_source_lease_lost"
                and now < lease_grace_deadline
                and not _lease_source_observed(snapshot.payload, command_client)
            ):
                q = _targets_from_payload(snapshot.payload)
                last_delta = compute_joint_delta(q, target)
                time.sleep(period_sec)
                continue
            command_client.send(CommandIntent.hold(timeout_sec=server.command_timeout_sec))
            return InitReturnResult(start_delta_deg={}, final_delta_deg=last_delta, arrived=False, fault=cause)
        q = _targets_from_payload(snapshot.payload)
        last_delta = compute_joint_delta(q, target)
        if max(last_delta.values()) <= tol_deg:
            return InitReturnResult(start_delta_deg={}, final_delta_deg=last_delta, arrived=True)
        time.sleep(period_sec)
    command_client.send(CommandIntent.hold(timeout_sec=server.command_timeout_sec))
    return InitReturnResult(start_delta_deg={}, final_delta_deg=last_delta, arrived=False, timeout=True)


def _lease_source_observed(payload: dict[str, Any], command_client: Any) -> bool:
    lease = payload.get("command_source") if isinstance(payload.get("command_source"), dict) else {}
    return lease.get("active_source_id") == getattr(command_client, "source_id", None)


def _targets_from_payload(payload: dict[str, Any]) -> JointTargets:
    q = extract_q_actual(payload)
    return JointTargets(tuple(q["left"]), tuple(q["right"]))


def confirm_batch_execute(count: int, init_mode: str) -> None:
    token = f"RUN {count} {init_mode}"
    answer = input(f"Batch physical motion gate: type {token} to confirm all episodes: ").strip()
    if answer != token:
        raise BatchHalt("batch confirmation token mismatch")


def build_driver_command(args: argparse.Namespace, episode_path: Path, batch_name: str, *, dry_run: bool) -> list[str]:
    anchor = args.anchor or ("mock" if dry_run else "live")
    cmd = [
        sys.executable,
        str(ROOT / "scripts" / "replay_episode_tcp_pose_target.py"),
        "--source",
        args.source,
        "--data-tcp",
        str(episode_path),
        "--mode",
        args.mode,
        "--arms",
        args.arms,
        "--anchor",
        anchor,
        "--segment",
        args.segment,
        "--action-scale",
        str(args.action_scale),
        "--time-scale",
        str(args.time_scale),
        "--out-dir",
        str(args.out_dir),
        "--run-name",
        batch_name,
    ]
    if args.server_config:
        cmd.extend(["--server-config", str(args.server_config)])
    if args.max_linear_speed_m_s is not None:
        cmd.extend(["--max-linear-speed-m-s", str(args.max_linear_speed_m_s)])
    if args.max_angular_speed_rad_s is not None:
        cmd.extend(["--max-angular-speed-rad-s", str(args.max_angular_speed_rad_s)])
    if args.allow_controller_sim_arm_error:
        cmd.append("--allow-controller-sim-arm-error")
    if dry_run:
        cmd.extend(["--mock-current-pose", str(args.mock_current_pose)])
    else:
        cmd.extend(["--execute", "--i-am-at-the-estop", "--non-interactive"])
    return cmd


def run_episode_driver(cmd: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, cwd=ROOT, text=True, capture_output=True)


def write_episode_metrics(run_dir: Path) -> tuple[Path | None, dict[str, float | None], dict[str, int | None], dict[str, float | None]]:
    log_path = run_dir / "log.csv"
    if not log_path.exists():
        return None, _none_by_arm(), _none_by_arm_int(), _none_by_arm()
    cfg = MetricsRootConfig()
    payload = analyze_inputs([load_input(log_path)], cfg, cfg.metrics, primary_path=log_path)
    metrics_path = run_dir / "metrics.json"
    metrics_path.write_text(json.dumps(_jsonable(payload), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (run_dir / "summary.md").write_text(render_summary(payload), encoding="utf-8")
    return metrics_path, extract_tracking_p95_mm(payload), extract_branch_count(payload), extract_ik_p95(payload)


def build_episode_result(
    episode_path: Path,
    run_dir: Path,
    init_result: InitReturnResult,
    metrics_path: Path | None,
    tracking: dict[str, float | None],
    branch: dict[str, int | None],
    ik: dict[str, float | None],
) -> EpisodeBatchResult:
    meta = read_run_meta(run_dir)
    rows = read_log_rows(run_dir / "log.csv")
    return EpisodeBatchResult(
        episode=episode_path.name,
        episode_id=driver.hdf5_episode_id(episode_path),
        status="ok",
        run_dir=str(run_dir),
        log_path=str(run_dir / "log.csv"),
        metrics_path=str(metrics_path) if metrics_path else None,
        segment=format_segment(meta.get("segment_selection")),
        bounds=compute_log_bounds(rows),
        max_stream_speed=meta.get("stream_speed_stats", {}),
        init_return=init_result,
        floor_roi=format_floor_roi(meta.get("bounds_notes", [])),
        actual_vs_goal_p95_mm=tracking,
        branch_jump_count=branch,
        ik_solve_us_p95=ik,
    )


def build_failed_result(episode_path: Path, run_dir: Path, init_result: InitReturnResult, refusal: str) -> EpisodeBatchResult:
    return EpisodeBatchResult(
        episode=episode_path.name,
        episode_id=driver.hdf5_episode_id(episode_path),
        status="halted",
        run_dir=str(run_dir),
        log_path=str(run_dir / "log.csv") if (run_dir / "log.csv").exists() else None,
        metrics_path=None,
        segment=None,
        bounds={},
        max_stream_speed={},
        init_return=init_result,
        floor_roi="not attempted",
        actual_vs_goal_p95_mm=_none_by_arm(),
        branch_jump_count=_none_by_arm_int(),
        ik_solve_us_p95=_none_by_arm(),
        refusal=refusal,
    )


def read_run_meta(run_dir: Path) -> dict[str, Any]:
    path = run_dir / "run_meta.json"
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def read_log_rows(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    return TrajectoryLogReader(path).read()


def compute_log_bounds(rows: list[dict[str, Any]]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for arm in ARMS:
        poses = [np.asarray(row["conditioned_goal_after_A"], dtype=np.float64) for row in rows if row.get("arm") == arm and "conditioned_goal_after_A" in row]
        if not poses:
            out[arm] = None
            continue
        arr = np.vstack(poses)
        out[arm] = {"min_xyz": np.min(arr[:, :3], axis=0).tolist(), "max_xyz": np.max(arr[:, :3], axis=0).tolist()}
    return out


def format_segment(selection: Any) -> str | None:
    if not isinstance(selection, dict):
        return None
    if selection.get("mode") == "single":
        rng = selection.get("source_frame_range") or []
        if len(rng) == 2:
            return f"{int(rng[0])}-{int(rng[1])}"
    if selection.get("mode") == "all":
        return f"all/{selection.get('source_frame_count')}"
    return str(selection.get("requested") or selection.get("mode") or "unknown")


def format_floor_roi(notes: list[Any]) -> str:
    text = " | ".join(str(item) for item in notes)
    if "VIOLATION:" in text:
        return "VIOLATION"
    floor = "floor ok" if "floor precheck ok" in text else "floor skipped"
    roi = "ROI ok" if "ROI precheck ok" in text else "ROI skipped"
    return f"{floor}; {roi}"


def extract_tracking_p95_mm(payload: dict[str, Any]) -> dict[str, float | None]:
    out: dict[str, float | None] = _none_by_arm()
    item = (payload.get("inputs") or [{}])[0]
    for arm in ARMS:
        metric = item.get("arms", {}).get(arm, {}).get("tracking", {}).get("actual_tcp_vs_conditioned_goal", {})
        if metric.get("status") == "ok":
            out[arm] = float(metric.get("position_m", {}).get("p95")) * 1000.0
    return out


def extract_branch_count(payload: dict[str, Any]) -> dict[str, int | None]:
    out: dict[str, int | None] = _none_by_arm_int()
    item = (payload.get("inputs") or [{}])[0]
    for arm in ARMS:
        metric = item.get("arms", {}).get(arm, {}).get("health", {}).get("branch_jump_count", {})
        if metric.get("status") == "ok":
            out[arm] = int(metric.get("count", 0))
    return out


def extract_ik_p95(payload: dict[str, Any]) -> dict[str, float | None]:
    out: dict[str, float | None] = _none_by_arm()
    item = (payload.get("inputs") or [{}])[0]
    for arm in ARMS:
        metric = item.get("arms", {}).get(arm, {}).get("health", {}).get("ik_solve_us", {})
        if metric.get("status") == "ok":
            out[arm] = float(metric.get("p95"))
    return out


def write_batch_summary(
    batch_dir: Path,
    args: argparse.Namespace,
    episodes: list[Path],
    results: list[EpisodeBatchResult],
    *,
    halted: bool,
    refusal: str | None,
    dry_run: bool,
) -> None:
    payload = {
        "schema_id": "robotics_lab.tcp_tuning.batch_replay.v1",
        "generated_at": datetime.now().isoformat(),
        "dry_run": dry_run,
        "halted": halted,
        "refusal": refusal,
        "episode_count_requested": len(episodes),
        "episode_count_attempted": len(results),
        "args": {k: _jsonable(v) for k, v in vars(args).items()},
        "results": [_jsonable(asdict(result)) for result in results],
    }
    (batch_dir / "batch_summary.json").write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (batch_dir / "batch_summary.md").write_text(render_batch_markdown(payload), encoding="utf-8")
    print(render_batch_table(results))


def render_batch_markdown(payload: dict[str, Any]) -> str:
    lines = [
        "# TCP Batch Replay Summary",
        "",
        f"- Dry run: `{payload['dry_run']}`",
        f"- Halted: `{payload['halted']}`",
        f"- Refusal: `{payload.get('refusal')}`",
        "",
        render_batch_table_dicts(payload["results"]),
        "",
    ]
    return "\n".join(lines)


def render_batch_table(results: list[EpisodeBatchResult]) -> str:
    lines = [
        "| episode | status | segment | L xyz min..max | R xyz min..max | max speed L/R | init delta deg L/R | floor/ROI | track p95 mm L/R | branch jumps L/R | ik p95 us L/R |",
        "|---|---|---:|---|---|---|---:|---|---:|---:|---:|",
    ]
    for result in results:
        lines.append(
            "| {episode} | {status} | {segment} | {lb} | {rb} | {speed} | {init} | {floor_roi} | {track} | {branch} | {ik} |".format(
                episode=result.episode,
                status=result.status,
                segment=result.segment or "-",
                lb=format_bounds(result.bounds.get("left") if isinstance(result.bounds, dict) else None),
                rb=format_bounds(result.bounds.get("right") if isinstance(result.bounds, dict) else None),
                speed=format_speed(result.max_stream_speed),
                init=format_pair(result.init_return.start_delta_deg),
                floor_roi=result.floor_roi,
                track=format_pair(result.actual_vs_goal_p95_mm),
                branch=format_pair(result.branch_jump_count),
                ik=format_pair(result.ik_solve_us_p95),
            )
        )
    return "\n".join(lines)


def render_batch_table_dicts(results: list[dict[str, Any]]) -> str:
    lines = [
        "| episode | status | segment | L xyz min..max | R xyz min..max | max speed L/R | init delta deg L/R | floor/ROI | track p95 mm L/R | branch jumps L/R | ik p95 us L/R |",
        "|---|---|---:|---|---|---|---:|---|---:|---:|---:|",
    ]
    for result in results:
        init_return = result.get("init_return") if isinstance(result.get("init_return"), dict) else {}
        lines.append(
            "| {episode} | {status} | {segment} | {lb} | {rb} | {speed} | {init} | {floor_roi} | {track} | {branch} | {ik} |".format(
                episode=result.get("episode"),
                status=result.get("status"),
                segment=result.get("segment") or "-",
                lb=format_bounds((result.get("bounds") or {}).get("left") if isinstance(result.get("bounds"), dict) else None),
                rb=format_bounds((result.get("bounds") or {}).get("right") if isinstance(result.get("bounds"), dict) else None),
                speed=format_speed(result.get("max_stream_speed")),
                init=format_pair(init_return.get("start_delta_deg")),
                floor_roi=result.get("floor_roi"),
                track=format_pair(result.get("actual_vs_goal_p95_mm")),
                branch=format_pair(result.get("branch_jump_count")),
                ik=format_pair(result.get("ik_solve_us_p95")),
            )
        )
    return "\n".join(lines)


def format_bounds(bounds: Any) -> str:
    if not isinstance(bounds, dict):
        return "-"
    return f"{_fmt_vec(bounds.get('min_xyz'))}..{_fmt_vec(bounds.get('max_xyz'))}"


def format_speed(stats: Any) -> str:
    if not isinstance(stats, dict):
        return "-"
    values = []
    for arm in ARMS:
        item = stats.get(arm)
        if isinstance(item, dict):
            values.append(f"{item.get('max_linear_m_s', 0.0):.4f}/{item.get('max_angular_rad_s', 0.0):.4f}")
        else:
            values.append("-")
    return " / ".join(values)


def format_pair(values: Any) -> str:
    if not isinstance(values, dict):
        return "-"
    return f"{_fmt_scalar(values.get('left'))}/{_fmt_scalar(values.get('right'))}"


def _fmt_vec(values: Any) -> str:
    if not isinstance(values, (list, tuple)):
        return "-"
    return "[" + ",".join(f"{float(item):.3f}" for item in values[:3]) + "]"


def _fmt_scalar(value: Any) -> str:
    if value is None:
        return "-"
    try:
        return f"{float(value):.3f}"
    except (TypeError, ValueError):
        return str(value)


def _none_by_arm() -> dict[str, float | None]:
    return {"left": None, "right": None}


def _none_by_arm_int() -> dict[str, int | None]:
    return {"left": None, "right": None}


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


if __name__ == "__main__":
    raise SystemExit(main())
