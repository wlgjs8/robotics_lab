#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path
from typing import Any

import h5py
import numpy as np
from scipy.spatial.transform import Rotation

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
for path in (str(REPO_ROOT), str(SCRIPT_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

from tcp_tuning.config import Config, load_config
from tcp_tuning.hdf5_io import EpisodeData, load_episode
from tcp_tuning.se3 import quat_canonical, twist_from_poses


AUDIT_SCHEMA = "robotics_lab.tcp_tuning.audit.v1"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Audit a UMI/TCP HDF5 episode without modifying it.")
    parser.add_argument("--episode", required=True, help="HDF5 episode path")
    parser.add_argument("--out-dir", required=True, help="Base output directory, e.g. outputs/tcp_tuning")
    parser.add_argument("--plots", action="store_true", help="Write audit plots under plots/")
    parser.add_argument("--config", help="Optional YAML config override")
    args = parser.parse_args(argv)

    cfg = load_config(args.config)
    episode_path = Path(args.episode)
    output_dir = Path(args.out_dir) / episode_id(episode_path)
    output_dir.mkdir(parents=True, exist_ok=True)

    tree = hdf5_tree(episode_path)
    print("\n".join(tree))

    episode = load_episode(str(episode_path), nominal_rate_hz=cfg.audit.nominal_source_rate_hz)
    report = build_audit_report(episode, cfg, tree)
    write_outputs(report, output_dir)
    if args.plots:
        write_plots(episode, report, output_dir / "plots", cfg)
    print(f"wrote {output_dir / 'audit.json'}")
    print(f"wrote {output_dir / 'audit_summary.md'}")
    if args.plots:
        print(f"wrote plots under {output_dir / 'plots'}")
    return 0


def episode_id(path: Path) -> str:
    return f"{path.parent.name}__{path.stem}"


def hdf5_tree(path: Path) -> list[str]:
    lines = [f"{path}"]
    with h5py.File(path, "r") as handle:
        if handle.attrs:
            lines.append(f"  attrs: {_jsonable_attrs(handle.attrs)}")

        def visit(name: str, obj: h5py.Dataset | h5py.Group) -> None:
            depth = name.count("/") + 1
            indent = "  " * depth
            if isinstance(obj, h5py.Dataset):
                lines.append(f"{indent}{name} {tuple(obj.shape)} {obj.dtype} attrs={_jsonable_attrs(obj.attrs)}")
            else:
                lines.append(f"{indent}{name}/ attrs={_jsonable_attrs(obj.attrs)}")

        handle.visititems(visit)
    return lines


def build_audit_report(episode: EpisodeData, cfg: Config, tree: list[str]) -> dict[str, Any]:
    t, nominal_rate_used = effective_timestamps(episode, cfg.audit.nominal_source_rate_hz)
    timing = timing_summary(t, cfg)
    gaps = detect_timestamp_gaps(
        t,
        median_multiplier=cfg.audit.gap_median_multiplier,
        absolute_threshold_sec=cfg.audit.gap_absolute_threshold_sec,
    )
    segments = segment_boundaries(len(t), gaps)
    arm_reports = {
        "left": arm_motion_report(t, episode.left_pose, episode.left_gripper),
        "right": arm_motion_report(t, episode.right_pose, episode.right_gripper),
    }
    report = {
        "schema": AUDIT_SCHEMA,
        "git_commit": git_commit(),
        "episode_path": episode.path,
        "episode_id": episode_id(Path(episode.path)),
        "detected": episode.detected,
        "hdf5_tree": tree,
        "nominal_rate_hz": cfg.audit.nominal_source_rate_hz,
        "nominal_rate_used": nominal_rate_used,
        "timing": timing,
        "gaps": gaps,
        "segments": segments,
        "arms": arm_reports,
    }
    return report


def effective_timestamps(episode: EpisodeData, nominal_rate_hz: float) -> tuple[np.ndarray, bool]:
    if episode.t_source is not None and episode.t_source.size:
        return np.asarray(episode.t_source, dtype=np.float64).reshape(-1), False
    lengths = [
        arr.shape[0]
        for arr in (episode.left_pose, episode.right_pose, episode.left_gripper, episode.right_gripper)
        if arr is not None and arr.ndim >= 1
    ]
    count = max(lengths) if lengths else 0
    if count <= 0:
        return np.asarray([], dtype=np.float64), True
    return np.arange(count, dtype=np.float64) / float(nominal_rate_hz), True


def timing_summary(t: np.ndarray, cfg: Config) -> dict[str, Any]:
    dt = np.diff(np.asarray(t, dtype=np.float64))
    finite = dt[np.isfinite(dt)]
    positive = finite[finite > 0.0]
    stats = numeric_stats(positive)
    median = stats["p50"]
    return {
        "sample_count": int(t.size),
        "duration_sec": _finite_float(float(t[-1] - t[0])) if t.size >= 2 else 0.0,
        "nominal_frequency_hz": _finite_float(1.0 / median) if median > 0.0 else 0.0,
        "configured_nominal_source_rate_hz": cfg.audit.nominal_source_rate_hz,
        "dt_sec": {
            "mean": _finite_float(float(np.mean(positive))) if positive.size else 0.0,
            "p50": stats["p50"],
            "p95": stats["p95"],
            "max": stats["max"],
        },
    }


def detect_timestamp_gaps(
    t: np.ndarray,
    *,
    median_multiplier: float,
    absolute_threshold_sec: float,
) -> list[dict[str, Any]]:
    times = np.asarray(t, dtype=np.float64).reshape(-1)
    if times.size < 2:
        return []
    dt = np.diff(times)
    positive = dt[np.isfinite(dt) & (dt > 0.0)]
    if positive.size == 0:
        return []
    median = float(np.median(positive))
    out: list[dict[str, Any]] = []
    for index, value in enumerate(dt):
        reasons = []
        if not np.isfinite(value) or value <= 0.0:
            reasons.append("non_positive_or_non_finite")
        if np.isfinite(value) and median > 0.0 and value > float(median_multiplier) * median:
            reasons.append("dt_gt_median_multiplier")
        if np.isfinite(value) and value > float(absolute_threshold_sec):
            reasons.append("dt_gt_absolute_threshold")
        if reasons:
            out.append(
                {
                    "before_index": int(index),
                    "after_index": int(index + 1),
                    "t_before": _finite_float(float(times[index])),
                    "t_after": _finite_float(float(times[index + 1])),
                    "dt_sec": _finite_float(float(value)),
                    "median_dt_sec": _finite_float(median),
                    "reasons": reasons,
                }
            )
    return out


def segment_boundaries(length: int, gaps: list[dict[str, Any]]) -> list[dict[str, int]]:
    if length <= 0:
        return []
    starts = [0] + [int(gap["after_index"]) for gap in gaps if 0 < int(gap["after_index"]) < length]
    stops = [int(gap["after_index"]) for gap in gaps if 0 < int(gap["after_index"]) < length] + [length]
    return [
        {"start_index": int(start), "stop_index_exclusive": int(stop), "sample_count": int(stop - start)}
        for start, stop in zip(starts, stops)
        if stop > start
    ]


def arm_motion_report(t: np.ndarray, pose: np.ndarray | None, gripper: np.ndarray | None) -> dict[str, Any]:
    if pose is None or pose.size == 0:
        return {
            "pose_present": False,
            "path_length_m": 0.0,
            "linear_speed_m_s": empty_stats(),
            "angular_speed_rad_s": empty_stats(),
            "linear_jerk_m_s3": empty_stats(),
            "angular_jerk_rad_s3": empty_stats(),
            "gripper": gripper_report(t, gripper),
        }
    count = min(t.size, pose.shape[0])
    times = np.asarray(t[:count], dtype=np.float64)
    poses = np.asarray(pose[:count], dtype=np.float64)
    positions = poses[:, :3]
    path_steps = np.linalg.norm(np.diff(positions, axis=0), axis=1) if count >= 2 else np.asarray([], dtype=np.float64)
    linear_speeds: list[float] = []
    angular_speeds: list[float] = []
    velocity_vectors: list[np.ndarray] = []
    angular_vectors: list[np.ndarray] = []
    velocity_times: list[float] = []
    for index in range(count - 1):
        dt = float(times[index + 1] - times[index])
        if dt <= 0.0 or not np.isfinite(dt):
            continue
        try:
            v, w = twist_from_poses(
                poses[index, :3],
                poses[index, 3:7],
                poses[index + 1, :3],
                poses[index + 1, 3:7],
                dt,
            )
        except ValueError:
            continue
        linear_speeds.append(float(np.linalg.norm(v)))
        angular_speeds.append(float(np.linalg.norm(w)))
        velocity_vectors.append(v)
        angular_vectors.append(w)
        velocity_times.append(float(0.5 * (times[index] + times[index + 1])))
    lin_jerk = jerk_norms(np.asarray(velocity_times), np.asarray(velocity_vectors))
    ang_jerk = jerk_norms(np.asarray(velocity_times), np.asarray(angular_vectors))
    return {
        "pose_present": True,
        "sample_count": int(count),
        "path_length_m": _finite_float(float(np.sum(path_steps))) if path_steps.size else 0.0,
        "linear_speed_m_s": numeric_stats(np.asarray(linear_speeds, dtype=np.float64)),
        "angular_speed_rad_s": numeric_stats(np.asarray(angular_speeds, dtype=np.float64)),
        "linear_jerk_m_s3": numeric_stats(lin_jerk),
        "angular_jerk_rad_s3": numeric_stats(ang_jerk),
        "gripper": gripper_report(times, gripper),
    }


def jerk_norms(times: np.ndarray, velocities: np.ndarray) -> np.ndarray:
    if times.size < 3 or velocities.ndim != 2 or velocities.shape[0] < 3:
        return np.asarray([], dtype=np.float64)
    dt_acc = np.diff(times)
    valid = np.isfinite(dt_acc) & (dt_acc > 0.0)
    if np.count_nonzero(valid) < 2:
        return np.asarray([], dtype=np.float64)
    accel = np.diff(velocities, axis=0)
    accel = accel[valid] / dt_acc[valid, None]
    accel_times = 0.5 * (times[:-1][valid] + times[1:][valid])
    if accel.shape[0] < 2:
        return np.asarray([], dtype=np.float64)
    dt_jerk = np.diff(accel_times)
    valid_jerk = np.isfinite(dt_jerk) & (dt_jerk > 0.0)
    if not np.any(valid_jerk):
        return np.asarray([], dtype=np.float64)
    jerk = np.diff(accel, axis=0)[valid_jerk] / dt_jerk[valid_jerk, None]
    return np.linalg.norm(jerk, axis=1)


def gripper_report(t: np.ndarray, gripper: np.ndarray | None) -> dict[str, Any]:
    if gripper is None or gripper.size == 0:
        return {"present": False, "events": []}
    count = min(t.size, gripper.shape[0])
    values = np.asarray(gripper[:count], dtype=np.float64).reshape(-1)
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return {"present": True, "events": []}
    low = float(np.min(finite))
    high = float(np.max(finite))
    if not math.isfinite(low) or not math.isfinite(high) or abs(high - low) <= 1e-12:
        return {"present": True, "min": _finite_float(low), "max": _finite_float(high), "events": []}
    threshold = 0.5 * (low + high)
    state = values >= threshold
    events = []
    for index in np.flatnonzero(state[1:] != state[:-1]) + 1:
        events.append(
            {
                "index": int(index),
                "time_sec": _finite_float(float(t[index])) if index < t.size else 0.0,
                "event": "open" if bool(state[index]) else "close",
                "value": _finite_float(float(values[index])),
            }
        )
    return {
        "present": True,
        "min": _finite_float(low),
        "max": _finite_float(high),
        "threshold": _finite_float(threshold),
        "events": events,
    }


def numeric_stats(values: np.ndarray) -> dict[str, float]:
    finite = np.asarray(values, dtype=np.float64)
    finite = finite[np.isfinite(finite)]
    if finite.size == 0:
        return empty_stats()
    return {
        "p50": _finite_float(float(np.percentile(finite, 50))),
        "p95": _finite_float(float(np.percentile(finite, 95))),
        "max": _finite_float(float(np.max(finite))),
    }


def empty_stats() -> dict[str, float]:
    return {"p50": 0.0, "p95": 0.0, "max": 0.0}


def write_outputs(report: dict[str, Any], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "audit.json").write_text(json.dumps(_jsonable(report), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output_dir / "audit_summary.md").write_text(render_summary(report), encoding="utf-8")


def render_summary(report: dict[str, Any]) -> str:
    timing = report["timing"]
    lines = [
        "# TCP Tuning Episode Audit",
        "",
        f"- Schema: `{report['schema']}`",
        f"- Episode: `{report['episode_path']}`",
        f"- Git commit: `{report.get('git_commit') or 'unknown'}`",
        f"- Nominal rate used: `{report['nominal_rate_used']}`",
        f"- Samples: {timing['sample_count']}",
        f"- Nominal frequency: {timing['nominal_frequency_hz']:.6g} Hz",
        f"- dt median/mean/p95/max: {timing['dt_sec']['p50']:.6g} / {timing['dt_sec']['mean']:.6g} / {timing['dt_sec']['p95']:.6g} / {timing['dt_sec']['max']:.6g} s",
        f"- Gaps: {len(report['gaps'])}",
        f"- Segments: {len(report['segments'])}",
        "",
        "## Arms",
        "",
        "| Arm | Pose | Path m | Lin speed p50/p95/max m/s | Ang speed p50/p95/max rad/s | Gripper events |",
        "| --- | --- | ---: | --- | --- | ---: |",
    ]
    for arm, data in report["arms"].items():
        lin = data["linear_speed_m_s"]
        ang = data["angular_speed_rad_s"]
        lines.append(
            f"| {arm} | {data['pose_present']} | {data['path_length_m']:.6g} | "
            f"{lin['p50']:.6g}/{lin['p95']:.6g}/{lin['max']:.6g} | "
            f"{ang['p50']:.6g}/{ang['p95']:.6g}/{ang['max']:.6g} | "
            f"{len(data['gripper'].get('events', []))} |"
        )
    lines.extend(["", "## Detected Keys", "", "```json", json.dumps(report["detected"].get("selected", {}), indent=2, sort_keys=True), "```", ""])
    return "\n".join(lines)


def write_plots(episode: EpisodeData, report: dict[str, Any], plot_dir: Path, cfg: Config) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plot_dir.mkdir(parents=True, exist_ok=True)
    t, _ = effective_timestamps(episode, cfg.audit.nominal_source_rate_hz)
    t_rel = t - t[0] if t.size else t
    gaps = report["gaps"]
    if t.size >= 2:
        fig, ax = plt.subplots()
        ax.plot(np.arange(t.size - 1), np.diff(t), label="dt")
        for gap in gaps:
            ax.axvline(int(gap["before_index"]), color="r", alpha=0.35)
        ax.set_xlabel("frame index")
        ax.set_ylabel("dt (s)")
        ax.legend()
        fig.tight_layout()
        fig.savefig(plot_dir / "audit_dt.png", dpi=cfg.audit.plot_dpi)
        plt.close(fig)
    for arm, pose in (("left", episode.left_pose), ("right", episode.right_pose)):
        if pose is None or pose.size == 0 or t.size == 0:
            continue
        count = min(t_rel.size, pose.shape[0])
        tt = t_rel[:count]
        pp = pose[:count, :3]
        qq = _continuous_quat_series(pose[:count, 3:7])
        _plot_series(plot_dir / f"audit_{arm}_position.png", tt, pp, "position (m)", cfg)
        rotvec = Rotation.from_quat(qq).as_rotvec()
        angle = np.linalg.norm(rotvec, axis=1)
        _plot_series(plot_dir / f"audit_{arm}_orientation_log.png", tt, np.column_stack([rotvec, angle]), "orientation log / angle (rad)", cfg)
        lin_speed, ang_speed, speed_t = _speed_series(t[:count], pose[:count])
        if speed_t.size:
            _plot_series(plot_dir / f"audit_{arm}_linear_speed.png", speed_t - t[0], lin_speed[:, None], "linear speed (m/s)", cfg)
            _plot_series(plot_dir / f"audit_{arm}_angular_speed.png", speed_t - t[0], ang_speed[:, None], "angular speed (rad/s)", cfg)


def _plot_series(path: Path, t: np.ndarray, values: np.ndarray, ylabel: str, cfg: Config) -> None:
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots()
    ax.plot(t, values)
    ax.set_xlabel("time (s)")
    ax.set_ylabel(ylabel)
    fig.tight_layout()
    fig.savefig(path, dpi=cfg.audit.plot_dpi)
    plt.close(fig)


def _speed_series(t: np.ndarray, pose: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    lin: list[float] = []
    ang: list[float] = []
    times: list[float] = []
    for index in range(min(t.size, pose.shape[0]) - 1):
        dt = float(t[index + 1] - t[index])
        if dt <= 0.0 or not np.isfinite(dt):
            continue
        try:
            v, w = twist_from_poses(pose[index, :3], pose[index, 3:7], pose[index + 1, :3], pose[index + 1, 3:7], dt)
        except ValueError:
            continue
        lin.append(float(np.linalg.norm(v)))
        ang.append(float(np.linalg.norm(w)))
        times.append(0.5 * float(t[index] + t[index + 1]))
    return np.asarray(lin), np.asarray(ang), np.asarray(times)


def _continuous_quat_series(quat: np.ndarray) -> np.ndarray:
    out = np.zeros_like(quat, dtype=np.float64)
    ref = None
    for index, q in enumerate(quat):
        out[index] = quat_canonical(q, ref=ref)
        ref = out[index]
    return out


def git_commit() -> str | None:
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=REPO_ROOT,
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
    except Exception:
        return None
    return result.stdout.strip() or None


def _jsonable_attrs(attrs: h5py.AttributeManager) -> dict[str, Any]:
    return {key: _jsonable(value) for key, value in attrs.items()}


def _jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _jsonable(val) for key, val in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if isinstance(value, np.ndarray):
        return _jsonable(value.tolist())
    if isinstance(value, np.generic):
        return _jsonable(value.item())
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    if isinstance(value, float):
        return _finite_float(value)
    return value


def _finite_float(value: float) -> float:
    return float(value) if math.isfinite(float(value)) else 0.0


if __name__ == "__main__":
    raise SystemExit(main())
