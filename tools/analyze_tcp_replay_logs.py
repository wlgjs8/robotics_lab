#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
for path in (str(REPO_ROOT), str(SCRIPT_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

from tcp_tuning.config import Config, MetricsConfig, load_config
from tcp_tuning.metrics import (
    ARMS,
    collect_null_metrics,
    health_metrics,
    pose_derivatives,
    smoothness_metrics,
    tracking_metrics,
)
from tcp_tuning.trajectory_log import TrajectoryLogReader


SCHEMA_ID = "robotics_lab.tcp_tuning.metrics.v1"
POSE_KEYS = ("source_raw_target", "conditioned_goal", "reference_after_B", "actual_tcp")
JOINT_KEYS = ("q_target", "q_actual")
HEALTH_KEYS = (
    "ik_solve_us",
    "ik_pos_err",
    "ik_ori_err",
    "ik_failure_flag",
    "ik_failed",
    "ik_fail_flag",
    "branch_jump_flag",
    "safety_proj_flag",
    "self_collision_flag",
    "floor_flag",
    "roi_flag",
)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Analyze offline TCP replay NPZ/log artifacts.")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--npz", help="Generated trajectory NPZ path")
    source.add_argument("--log", help="Trajectory log path: csv, jsonl, or npz")
    parser.add_argument("--out-dir", required=True, help="Base output directory")
    parser.add_argument("--compare", action="append", default=[], help="Additional NPZ/log paths to include in comparison plots")
    parser.add_argument("--policy-rate-hz", type=float, help="Policy/chunk rate for spectral peak-near-rate metrics")
    parser.add_argument("--plots", action="store_true", help="Write plots under analysis/<name>/plots")
    parser.add_argument("--config", help="Optional YAML config override")
    args = parser.parse_args(argv)

    cfg = load_config(args.config)
    metrics_cfg = _metrics_with_policy_rate(cfg.metrics, args.policy_rate_hz)
    primary_path = Path(args.npz or args.log)
    inputs = [load_input(primary_path)]
    inputs.extend(load_input(Path(item)) for item in args.compare)

    analysis_name = _analysis_name(primary_path, [Path(item) for item in args.compare])
    episode = inputs[0]["episode_id"]
    output_dir = _analysis_output_dir(Path(args.out_dir), episode, analysis_name)
    output_dir.mkdir(parents=True, exist_ok=True)

    payload = analyze_inputs(inputs, cfg, metrics_cfg, primary_path=primary_path)
    _write_json(output_dir / "metrics.json", payload)
    (output_dir / "summary.md").write_text(render_summary(payload), encoding="utf-8")
    if args.plots:
        write_plots(output_dir / "plots", inputs, payload, metrics_cfg)
    print(f"wrote {output_dir / 'metrics.json'}")
    print(f"wrote {output_dir / 'summary.md'}")
    if args.plots:
        print(f"wrote {output_dir / 'plots'}")
    return 0


def analyze_inputs(
    inputs: list[dict[str, Any]],
    cfg: Config,
    metrics_cfg: MetricsConfig,
    *,
    primary_path: Path,
) -> dict[str, Any]:
    analyzed = []
    for item in inputs:
        analyzed.append(analyze_single_input(item, metrics_cfg))
    payload: dict[str, Any] = {
        "schema_id": SCHEMA_ID,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "git_commit": git_commit(),
        "primary_input": str(primary_path),
        "config": cfg.to_dict(),
        "inputs": analyzed,
    }
    payload["null_metrics"] = collect_null_metrics({"inputs": analyzed})
    return payload


def analyze_single_input(item: dict[str, Any], cfg: MetricsConfig) -> dict[str, Any]:
    arms: dict[str, Any] = {}
    for arm in ARMS:
        arm_data = item["arms"].get(arm)
        if arm_data is None:
            arms[arm] = {
                "status": "null",
                "reason": f"{arm} arm data is absent",
                "notes": [f"{arm} arm data is absent"],
            }
            continue
        t = arm_data.get("t")
        conditioned = arm_data.get("conditioned_goal")
        smd_ref = arm_data.get("smd_ref_stand")
        has_smd = smd_ref is not None and np.isfinite(np.asarray(smd_ref, dtype=np.float64)).any()
        arms[arm] = {
            "reference_generation": {
                "reference_source": "smd_ref_stand" if has_smd else "tcp_ref_stand",
                "warning": None if has_smd else (
                    "smd_ref_stand unavailable; reference_after_B is tcp_ref_stand "
                    "(post-IK/safety/MA), not pure SMD output"
                ),
            },
            "tracking": tracking_metrics(
                t,
                actual_tcp=arm_data.get("actual_tcp"),
                reference_after_B=arm_data.get("reference_after_B"),
                conditioned_goal=conditioned,
                source_raw_target=arm_data.get("source_raw_target"),
                cfg=cfg,
            ),
            "smoothness": smoothness_metrics(
                t,
                conditioned,
                cfg=cfg,
                policy_rate_hz=cfg.chunk_rate_hz,
                metric_name=f"{item['name']}.{arm}.conditioned_goal_smoothness",
            ),
            "source_raw_target_smoothness": smoothness_metrics(
                t,
                arm_data.get("source_raw_target"),
                cfg=cfg,
                policy_rate_hz=cfg.chunk_rate_hz,
                metric_name=f"{item['name']}.{arm}.source_raw_target_smoothness",
            ),
            "health": health_metrics(t, arm_data, cfg=cfg),
        }
    return {
        "name": item["name"],
        "kind": item["kind"],
        "path": item["path"],
        "mode": item.get("mode"),
        "episode_id": item["episode_id"],
        "sample_count": int(item["sample_count"]),
        "metadata": item.get("metadata", {}),
        "arms": arms,
        "null_metrics": collect_null_metrics({"arms": arms}),
    }


def load_input(path: Path) -> dict[str, Any]:
    suffix = path.suffix.lower()
    if suffix == ".npz":
        if _looks_like_generated_npz(path):
            return load_generated_npz(path)
        return load_trajectory_log(path)
    if suffix in {".csv", ".jsonl"}:
        return load_trajectory_log(path)
    raise ValueError(f"unsupported input suffix: {path.suffix}")


def load_generated_npz(path: Path) -> dict[str, Any]:
    with np.load(path, allow_pickle=True) as data:
        files = set(data.files)
        t = np.asarray(data["t_servo"], dtype=np.float64).reshape(-1)
        metadata = _npz_metadata(data)
        mode = _scalar_str(data["mode"]) if "mode" in files else path.stem
        episode_id = str(metadata.get("episode_id") or _episode_id_from_path(path, data))
        arms: dict[str, Any] = {}
        for arm in ARMS:
            prefix = f"{arm}_"
            arms[arm] = {"t": t.copy()}
            for key in POSE_KEYS:
                arms[arm][key] = _array_or_none(data, prefix + key, width=7)
            for key in JOINT_KEYS:
                arms[arm][key] = _array_or_none(data, prefix + key, width=6)
            arms[arm]["conditioned_twist"] = _array_or_none(data, prefix + "conditioned_twist", width=6)
            for flag in ("valid", "hold", "dropout", "gap", "reanchor"):
                if prefix + flag in files:
                    arms[arm][flag] = np.asarray(data[prefix + flag])
        return {
            "kind": "generated_npz",
            "path": str(path),
            "name": path.stem,
            "mode": mode,
            "episode_id": episode_id,
            "sample_count": int(t.size),
            "metadata": metadata,
            "arms": arms,
        }


def load_trajectory_log(path: Path) -> dict[str, Any]:
    reader = TrajectoryLogReader(path)
    rows = reader.read()
    metadata = reader.read_metadata()
    by_arm: dict[str, list[dict[str, Any]]] = {arm: [] for arm in ARMS}
    for row in rows:
        arm = str(row.get("arm", "")).lower()
        if arm in by_arm:
            by_arm[arm].append(row)
    arms: dict[str, Any] = {}
    sample_count = 0
    for arm, arm_rows in by_arm.items():
        if not arm_rows:
            continue
        sample_count = max(sample_count, len(arm_rows))
        arms[arm] = _columns_from_rows(arm_rows)
    episode_id = str(metadata.get("episode_id") or _episode_id_from_path(path, None))
    return {
        "kind": "trajectory_log",
        "path": str(path),
        "name": path.stem,
        "mode": metadata.get("replay_mode"),
        "episode_id": episode_id,
        "sample_count": int(sample_count),
        "metadata": metadata,
        "arms": arms,
    }


def render_summary(payload: dict[str, Any]) -> str:
    lines = [
        "# TCP Replay Analysis Summary",
        "",
        f"- Schema: `{payload['schema_id']}`",
        f"- Git commit: `{payload.get('git_commit')}`",
        f"- Primary input: `{payload['primary_input']}`",
        "",
        "## Smoothness Comparison",
        "",
        "| input | arm | >5 Hz linear velocity power | sign reversals/sec | linear jerk RMS | dominant velocity Hz |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for item in payload["inputs"]:
        for arm in ARMS:
            smooth = item.get("arms", {}).get(arm, {}).get("smoothness", {})
            spectrum = smooth.get("linear_velocity_spectrum", {})
            reversals = smooth.get("linear_velocity_sign_reversals_per_sec", {})
            jerk = smooth.get("linear_jerk_m_s3", {})
            lines.append(
                "| {name} | {arm} | {hf} | {rev} | {jerk} | {peak} |".format(
                    name=item.get("name"),
                    arm=arm,
                    hf=_fmt(spectrum.get("power_above_cutoff")),
                    rev=_fmt(reversals.get("per_sec")),
                    jerk=_fmt(jerk.get("rms")),
                    peak=_fmt(spectrum.get("dominant_frequency_hz")),
                )
            )
    lines.extend(["", "## Null Metrics", ""])
    nulls = payload.get("null_metrics", [])
    if not nulls:
        lines.append("- None")
    else:
        for item in nulls:
            lines.append(f"- `{item['path']}`: {item['reason']}")
    lines.append("")
    return "\n".join(lines)


def write_plots(output_dir: Path, inputs: list[dict[str, Any]], payload: dict[str, Any], cfg: MetricsConfig) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output_dir.mkdir(parents=True, exist_ok=True)
    for item in inputs:
        for arm in ARMS:
            arm_data = item["arms"].get(arm)
            if not arm_data:
                continue
            _plot_tracking(output_dir, item, arm, arm_data, plt)
            _plot_derivatives(output_dir, item, arm, arm_data, plt)
            _plot_spectrum(output_dir, item, arm, arm_data, cfg, plt)
    _plot_comparison(output_dir, payload, plt)


def _plot_tracking(output_dir: Path, item: dict[str, Any], arm: str, arm_data: dict[str, Any], plt) -> None:
    pairs = [
        ("actual_vs_reference", arm_data.get("actual_tcp"), arm_data.get("reference_after_B")),
        ("actual_vs_conditioned", arm_data.get("actual_tcp"), arm_data.get("conditioned_goal")),
        ("raw_vs_conditioned", arm_data.get("source_raw_target"), arm_data.get("conditioned_goal")),
    ]
    t = _relative_time(arm_data.get("t"))
    for label, lhs, rhs in pairs:
        lhs_arr = _finite_pose(lhs)
        rhs_arr = _finite_pose(rhs)
        if lhs_arr is None or rhs_arr is None:
            continue
        count = min(lhs_arr.shape[0], rhs_arr.shape[0], t.size)
        valid = np.isfinite(lhs_arr[:count]).all(axis=1) & np.isfinite(rhs_arr[:count]).all(axis=1)
        if not np.any(valid):
            continue
        pos = np.linalg.norm(lhs_arr[:count, :3] - rhs_arr[:count, :3], axis=1)
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.plot(t[:count][valid], pos[valid])
        ax.set_title(f"{item['name']} {arm} {label} position error")
        ax.set_xlabel("time (s)")
        ax.set_ylabel("position error (m)")
        fig.tight_layout()
        fig.savefig(output_dir / f"{item['name']}_{arm}_{label}_position_error.png", dpi=150)
        plt.close(fig)


def _plot_derivatives(output_dir: Path, item: dict[str, Any], arm: str, arm_data: dict[str, Any], plt) -> None:
    derivatives = _derivatives_for_arm(arm_data)
    if derivatives is None:
        return
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=False)
    series = [
        ("velocity_time", "linear_velocity_m_s", "linear velocity (m/s)"),
        ("acceleration_time", "linear_acceleration_m_s2", "linear acceleration (m/s^2)"),
        ("jerk_time", "linear_jerk_m_s3", "linear jerk (m/s^3)"),
    ]
    for ax, (time_key, value_key, ylabel) in zip(axes, series):
        t = _relative_time(derivatives[time_key])
        values = derivatives[value_key]
        if values.size:
            ax.plot(t, np.linalg.norm(values, axis=1))
        ax.set_ylabel(ylabel)
    axes[-1].set_xlabel("time (s)")
    fig.suptitle(f"{item['name']} {arm} conditioned-goal derivatives")
    fig.tight_layout()
    fig.savefig(output_dir / f"{item['name']}_{arm}_velocity_acceleration_jerk.png", dpi=150)
    plt.close(fig)


def _plot_spectrum(output_dir: Path, item: dict[str, Any], arm: str, arm_data: dict[str, Any], cfg: MetricsConfig, plt) -> None:
    derivatives = _derivatives_for_arm(arm_data)
    if derivatives is None:
        return
    fig, ax = plt.subplots(figsize=(8, 4))
    for key, label in (("linear_velocity_m_s", "TCP linear velocity"), ("angular_velocity_rad_s", "wrist angular velocity")):
        freq, power = _spectrum_curve(derivatives["velocity_time"], derivatives[key])
        if freq is not None:
            ax.plot(freq, power, label=label)
    ax.axvline(float(cfg.high_frequency_cutoff_hz), color="k", linestyle="--", linewidth=1, label="high-frequency cutoff")
    ax.set_title(f"{item['name']} {arm} velocity PSD")
    ax.set_xlabel("frequency (Hz)")
    ax.set_ylabel("summed PSD")
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_dir / f"{item['name']}_{arm}_velocity_psd.png", dpi=150)
    plt.close(fig)


def _plot_comparison(output_dir: Path, payload: dict[str, Any], plt) -> None:
    for arm in ARMS:
        names = []
        hf = []
        reversals = []
        jerk = []
        for item in payload["inputs"]:
            smooth = item.get("arms", {}).get(arm, {}).get("smoothness", {})
            if smooth.get("status") != "ok":
                continue
            names.append(str(item["name"]))
            hf.append(_float_or_zero(smooth.get("linear_velocity_spectrum", {}).get("power_above_cutoff")))
            reversals.append(_float_or_zero(smooth.get("linear_velocity_sign_reversals_per_sec", {}).get("per_sec")))
            jerk.append(_float_or_zero(smooth.get("linear_jerk_m_s3", {}).get("rms")))
        if not names:
            continue
        x = np.arange(len(names), dtype=np.float64)
        width = 0.25
        fig, ax = plt.subplots(figsize=(10, 5))
        ax.bar(x - width, hf, width, label=">5 Hz power")
        ax.bar(x, reversals, width, label="sign reversals/sec")
        ax.bar(x + width, jerk, width, label="jerk RMS")
        ax.set_xticks(x)
        ax.set_xticklabels(names, rotation=20, ha="right")
        ax.set_title(f"{arm} conditioned-goal smoothness comparison")
        ax.legend()
        fig.tight_layout()
        fig.savefig(output_dir / f"comparison_{arm}_conditioned_goal_smoothness.png", dpi=150)
        plt.close(fig)


def _derivatives_for_arm(arm_data: dict[str, Any]) -> dict[str, Any] | None:
    pose = _finite_pose(arm_data.get("conditioned_goal"))
    t = np.asarray(arm_data.get("t"), dtype=np.float64).reshape(-1)
    if pose is None or t.size != pose.shape[0]:
        return None
    valid = np.isfinite(t) & np.isfinite(pose).all(axis=1)
    if int(np.count_nonzero(valid)) < 4:
        return None
    result = pose_derivatives(t[valid], pose[valid])
    return result if result.get("status") == "ok" else None


def _spectrum_curve(t: np.ndarray, values: np.ndarray) -> tuple[np.ndarray | None, np.ndarray | None]:
    from scipy.signal import periodogram

    times = np.asarray(t, dtype=np.float64).reshape(-1)
    arr = np.asarray(values, dtype=np.float64)
    valid = np.isfinite(times) & np.isfinite(arr).all(axis=1)
    times = times[valid]
    arr = arr[valid]
    if times.size < 4:
        return None, None
    dt = np.diff(times)
    dt = dt[np.isfinite(dt) & (dt > 0.0)]
    if dt.size == 0:
        return None, None
    fs = 1.0 / float(np.median(dt))
    freq, psd = periodogram(arr - np.mean(arr, axis=0, keepdims=True), fs=fs, axis=0, scaling="density")
    return freq, np.sum(psd, axis=1)


def _columns_from_rows(rows: list[dict[str, Any]]) -> dict[str, Any]:
    out: dict[str, Any] = {"t": np.asarray([_value(row.get("t")) for row in rows], dtype=np.float64)}
    aliases = {
        "conditioned_goal": "conditioned_goal_after_A",
        "conditioned_twist": "conditioned_twist_after_A",
    }
    for key in POSE_KEYS:
        source_key = aliases.get(key, key)
        out[key] = _stack_vector(rows, source_key, 7)
    for key in JOINT_KEYS:
        out[key] = _stack_vector(rows, key, 6)
    out["conditioned_twist"] = _stack_vector(rows, aliases["conditioned_twist"], 6)
    # Optional pure-SMD reference (Patch 4/5); used to attribute B-tier correctly.
    out["smd_ref_stand"] = _stack_vector(rows, "smd_ref_stand", 7)
    for key in HEALTH_KEYS:
        if any(key in row for row in rows):
            out[key] = np.asarray([_value(row.get(key)) for row in rows], dtype=np.float64)
    return out


def _stack_vector(rows: list[dict[str, Any]], key: str, width: int) -> np.ndarray | None:
    if not any(key in row for row in rows):
        return None
    out = np.full((len(rows), width), np.nan, dtype=np.float64)
    for index, row in enumerate(rows):
        if key not in row:
            continue
        arr = np.asarray(row[key], dtype=np.float64).reshape(-1)
        out[index, : min(width, arr.size)] = arr[:width]
    return out


def _looks_like_generated_npz(path: Path) -> bool:
    try:
        with np.load(path, allow_pickle=True) as data:
            files = set(data.files)
            return "t_servo" in files and any(f"{arm}_conditioned_goal" in files for arm in ARMS)
    except Exception:
        return False


def _npz_metadata(data) -> dict[str, Any]:
    if "meta_json" not in data.files:
        return {}
    try:
        return json.loads(str(np.asarray(data["meta_json"]).item()))
    except Exception:
        return {"meta_json_parse_error": True}


def _array_or_none(data, key: str, *, width: int) -> np.ndarray | None:
    if key not in data.files:
        return None
    arr = np.asarray(data[key], dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] != width:
        return None
    return arr


def _analysis_output_dir(base: Path, episode_id: str, analysis_name: str) -> Path:
    if base.name == episode_id:
        return base / "analysis" / analysis_name
    return base / episode_id / "analysis" / analysis_name


def _analysis_name(primary: Path, compare: list[Path]) -> str:
    if not compare:
        return primary.stem
    parts = [primary.stem] + [item.stem for item in compare]
    return "compare_" + "_".join(parts)


def _episode_id_from_path(path: Path, data) -> str:
    if data is not None and "episode" in data.files:
        episode = Path(_scalar_str(data["episode"]))
        if episode.suffix:
            return f"{episode.parent.name}__{episode.stem}"
    if path.parent.name and path.parent.name != "runs":
        return path.parent.name
    return path.stem


def _metrics_with_policy_rate(metrics: MetricsConfig, policy_rate_hz: float | None) -> MetricsConfig:
    if policy_rate_hz is None:
        return metrics
    return replace(metrics, chunk_rate_hz=float(policy_rate_hz))


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.write_text(json.dumps(_jsonable(payload), indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")


def git_commit() -> str | None:
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPO_ROOT, text=True, stderr=subprocess.DEVNULL).strip()
    except Exception:
        return None


def _relative_time(value: Any) -> np.ndarray:
    t = np.asarray(value, dtype=np.float64).reshape(-1)
    if t.size:
        return t - float(t[0])
    return t


def _finite_pose(value: Any) -> np.ndarray | None:
    if value is None:
        return None
    arr = np.asarray(value, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] != 7:
        return None
    if not np.any(np.isfinite(arr).all(axis=1)):
        return None
    return arr


def _scalar_str(value: Any) -> str:
    arr = np.asarray(value)
    if arr.shape == ():
        return str(arr.item())
    return str(arr.reshape(-1)[0])


def _value(value: Any) -> float:
    if value is None or value == "":
        return np.nan
    if isinstance(value, str):
        low = value.strip().lower()
        if low in ("true", "false"):
            return 1.0 if low == "true" else 0.0
    try:
        return float(value)
    except (TypeError, ValueError):
        return np.nan


def _float_or_zero(value: Any) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return 0.0
    return out if np.isfinite(out) else 0.0


def _fmt(value: Any) -> str:
    if value is None:
        return "null"
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "null"
    if not np.isfinite(number):
        return "null"
    return f"{number:.6g}"


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return _jsonable(value.tolist())
    if isinstance(value, np.generic):
        return _jsonable(value.item())
    if isinstance(value, float):
        return value if np.isfinite(value) else None
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


if __name__ == "__main__":
    raise SystemExit(main())
