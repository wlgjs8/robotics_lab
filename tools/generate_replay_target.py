#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
for path in (str(REPO_ROOT), str(SCRIPT_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

from tcp_tuning.command_conditioner import CommandConditioner
from tcp_tuning.config import Config, apply_cli_overrides, load_config
from tcp_tuning.hdf5_io import EpisodeData, load_episode
from tcp_tuning.smoothing import split_segments


REQUIRED_MODES = {"raw_zoh", "raw_foh_se3", "clean_foh_se3", "synthetic_policy_surrogate"}
ARMS = ("left", "right")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Generate offline TCP replay targets from an HDF5 episode.")
    parser.add_argument("--episode", help="HDF5 episode path")
    parser.add_argument("--mode", choices=sorted(REQUIRED_MODES), help="Command conditioning mode")
    parser.add_argument("--out-dir", default="outputs/tcp_tuning", help="Base output directory")
    parser.add_argument("--servo-rate-hz", type=float, default=500.0)
    parser.add_argument("--config", help="Optional YAML config override")
    # CLI conditioning/smoothing overrides (consistent with replay/batch; win over --config).
    parser.add_argument("--smoothing-method", choices=["none", "savgol", "lowpass", "cubic"], default=None)
    parser.add_argument("--smoothing-window-samples", type=int, default=None)
    parser.add_argument("--smoothing-polyorder", type=int, default=None)
    parser.add_argument("--lowpass-cutoff-hz", type=float, default=None)
    parser.add_argument("--cubic-smoothing", type=float, default=None)
    parser.add_argument("--gap-median-multiplier", type=float, default=None)
    parser.add_argument("--gap-absolute-threshold-sec", type=float, default=None)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--arms", default="left,right", help="Comma-separated arms to emit: left,right")
    parser.add_argument(
        "--segment",
        default=None,
        help="Source segment to emit: auto-largest, zero-based segment index, or start:stop source-frame range. Omit for full episode.",
    )
    parser.add_argument("--emit-sweep-matrix", help="Write the Phase-1 sweep key matrix to this path and exit")
    args = parser.parse_args(argv)

    cfg = apply_cli_overrides(
        load_config(args.config),
        {
            "smoothing_method": args.smoothing_method,
            "smoothing_window_samples": args.smoothing_window_samples,
            "smoothing_polyorder": args.smoothing_polyorder,
            "lowpass_cutoff_hz": args.lowpass_cutoff_hz,
            "cubic_smoothing": args.cubic_smoothing,
            "gap_median_multiplier": args.gap_median_multiplier,
            "gap_absolute_threshold_sec": args.gap_absolute_threshold_sec,
        },
    )
    cfg = _with_cli_overrides(cfg, args.servo_rate_hz, args.seed)
    if args.emit_sweep_matrix:
        return emit_sweep_matrix(Path(args.emit_sweep_matrix), cfg)
    if not args.episode or not args.mode:
        parser.error("--episode and --mode are required unless --emit-sweep-matrix is used")
    selected_arms = _parse_arms(args.arms)
    episode = load_episode(args.episode, nominal_rate_hz=cfg.conditioning.nominal_source_rate_hz)
    output_dir = Path(args.out_dir) / episode_id(Path(args.episode))
    output_dir.mkdir(parents=True, exist_ok=True)
    path = generate(episode, args.mode, output_dir, cfg, selected_arms=selected_arms, seed=args.seed, segment=args.segment)
    print(f"wrote {path}")
    return 0


def generate(
    episode: EpisodeData,
    mode: str,
    output_dir: Path,
    cfg: Config,
    *,
    selected_arms: tuple[str, ...] = ARMS,
    seed: int | None = None,
    segment: str | None = None,
) -> Path:
    t_source, nominal_rate_used = effective_timestamps(episode, cfg.conditioning.nominal_source_rate_hz)
    segment_selection = select_source_segment(
        t_source,
        segment,
        median_multiplier=float(cfg.conditioning.gap_median_multiplier),
        absolute_threshold_sec=float(cfg.conditioning.gap_absolute_threshold_sec),
    )
    if segment_selection["mode"] == "single":
        start = int(segment_selection["source_start"])
        stop = int(segment_selection["source_stop_exclusive"])
        episode = slice_episode(episode, start, stop, segment_selection)
        t_source = t_source[start:stop]
    conditioner = CommandConditioner(mode, cfg)
    for index, t_value in enumerate(t_source):
        conditioner.update_source_sample(
            t_value,
            _row_or_none(episode.left_pose, index) if "left" in selected_arms else None,
            _row_or_none(episode.right_pose, index) if "right" in selected_arms else None,
            _value_or_none(episode.left_gripper, index) if "left" in selected_arms else None,
            _value_or_none(episode.right_gripper, index) if "right" in selected_arms else None,
            metadata={"source_index": index, "nominal_rate_used": nominal_rate_used},
        )

    t_servo = servo_times(t_source, cfg.conditioning.servo_rate_hz)
    commands = [conditioner.sample(t_value) for t_value in t_servo]
    unsafe_flag, unsafe_reason = unsafe_full_clean_reason(mode, segment_selection, conditioner.segments)
    meta = {
        "git_commit": git_commit(),
        "config": cfg.to_dict(),
        "replay_mode": mode,
        "episode_id": episode_id(Path(episode.path)),
        "episode_path": episode.path,
        "segment_range": conditioner.segments,
        "segment_selection": segment_selection,
        "real_replay_unsafe_full_clean": bool(unsafe_flag),
        "real_replay_unsafe_full_clean_reason": unsafe_reason,
        "seed": int(seed) if seed is not None else -1,
        "detected_schema": episode.detected,
        "nominal_rate_used": bool(nominal_rate_used or conditioner.meta.get("nominal_rate_used", False)),
    }
    arrays = build_npz_arrays(episode, t_source, t_servo, commands, conditioner, mode, cfg, meta)
    if unsafe_flag:
        print(f"WARNING: {unsafe_reason}", file=sys.stderr)
    segment_token = segment_filename_token(segment_selection)
    output_path = output_dir / f"{mode}_{segment_token}{int(round(cfg.conditioning.servo_rate_hz))}hz.npz"
    np.savez(output_path, **arrays)
    if mode == "clean_foh_se3":
        write_clean_source(output_dir / f"clean_trajectory_{segment_token}{int(round(cfg.conditioning.servo_rate_hz))}hz.npz", conditioner, meta)
    return output_path


def select_source_segment(
    t_source: np.ndarray,
    requested: str | None,
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
    source_count = int(times.size)
    spec = "" if requested is None else str(requested).strip()
    base = {
        "requested": None if requested is None else spec,
        "source_frame_count": source_count,
        "all_segments": [[int(start), int(stop)] for start, stop in segments],
        "gap_count": int(gaps.shape[0]),
        "gaps": gaps.tolist(),
    }
    if spec == "":
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
        raise ValueError("cannot select a segment from an empty source episode")
    if spec == "auto-largest":
        segment_index, (start, stop) = max(
            enumerate(segments),
            key=lambda item: (int(item[1][1]) - int(item[1][0]), -int(item[0])),
        )
    elif ":" in spec:
        start, stop = parse_source_range(spec, source_count)
        segment_index = segment_index_containing_range(segments, start, stop)
        if segment_index is None:
            raise ValueError(
                f"--segment {spec!r} must be contained within one gap-free source segment; "
                f"available segments: {[[int(a), int(b)] for a, b in segments]}"
            )
    else:
        try:
            segment_index = int(spec)
        except ValueError as exc:
            raise ValueError("--segment must be auto-largest, a zero-based integer index, or start:stop") from exc
        if segment_index < 0 or segment_index >= len(segments):
            raise ValueError(f"--segment {segment_index} out of range; available segments: 0..{len(segments) - 1}")
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


def parse_source_range(spec: str, source_count: int) -> tuple[int, int]:
    start_text, sep, stop_text = spec.partition(":")
    if not sep or not start_text.strip() or not stop_text.strip():
        raise ValueError("--segment start:stop requires both start and stop")
    try:
        start = int(start_text)
        stop = int(stop_text)
    except ValueError as exc:
        raise ValueError("--segment start:stop values must be integers") from exc
    if start < 0 or stop < 0 or start >= stop or stop > int(source_count):
        raise ValueError(f"--segment {spec!r} out of range for {source_count} source frames")
    return start, stop


def segment_index_containing_range(segments: list[tuple[int, int]], start: int, stop: int) -> int | None:
    for index, (seg_start, seg_stop) in enumerate(segments):
        if int(seg_start) <= int(start) and int(stop) <= int(seg_stop):
            return int(index)
    return None


def slice_episode(episode: EpisodeData, start: int, stop: int, segment_selection: dict[str, Any]) -> EpisodeData:
    detected = dict(episode.detected or {})
    detected["segment_selection"] = segment_selection
    return EpisodeData(
        path=episode.path,
        t_source=_slice_optional_array(episode.t_source, start, stop),
        left_pose=_slice_optional_array(episode.left_pose, start, stop),
        right_pose=_slice_optional_array(episode.right_pose, start, stop),
        left_gripper=_slice_optional_array(episode.left_gripper, start, stop),
        right_gripper=_slice_optional_array(episode.right_gripper, start, stop),
        detected=detected,
    )


def segment_filename_token(selection: dict[str, Any]) -> str:
    if selection.get("mode") != "single":
        return ""
    index = int(selection.get("segment_index", -1))
    start = int(selection.get("source_start", 0))
    stop = int(selection.get("source_stop_exclusive", 0))
    return f"segment_{index}_{start}_{stop}_"


def unsafe_full_clean_reason(mode: str, selection: dict[str, Any], segments: list[tuple[int, int]]) -> tuple[bool, str | None]:
    if selection.get("mode") == "single" or len(segments) <= 1:
        return False, None
    if mode not in {"clean_foh_se3", "synthetic_policy_surrogate"}:
        return False, None
    reason = (
        "contains gap-boundary one-tick velocity spike; not for real replay -- "
        "regenerate with --segment or use the driver --segment path"
    )
    return True, reason


def build_npz_arrays(
    episode: EpisodeData,
    t_source: np.ndarray,
    t_servo: np.ndarray,
    commands,
    conditioner: CommandConditioner,
    mode: str,
    cfg: Config,
    meta: dict[str, Any],
) -> dict[str, Any]:
    arrays: dict[str, Any] = {
        "t_servo": t_servo.astype(np.float64),
        "servo_rate_hz": np.asarray(float(cfg.conditioning.servo_rate_hz), dtype=np.float64),
        "mode": np.asarray(mode),
        "episode": np.asarray(episode.path),
        "seed": np.asarray(int(meta["seed"]), dtype=np.int64),
        "segments": np.asarray(conditioner.segments, dtype=np.int64).reshape(-1, 2),
        "gaps": conditioner.gaps.astype(np.float64).reshape(-1, 2),
        "meta_json": np.asarray(json.dumps(_jsonable(meta), sort_keys=True)),
    }
    raw_by_arm = {"left": episode.left_pose, "right": episode.right_pose}
    for arm in ARMS:
        prefix = f"{arm}_"
        conditioned = np.stack([getattr(command, f"{arm}_pose") for command in commands], axis=0)
        twists = np.stack([_twist_or_nan(getattr(command, f"{arm}_twist")) for command in commands], axis=0)
        src_lo = np.asarray([int(command.src_ids[0]) for command in commands], dtype=np.int64)
        src_hi = np.asarray([int(command.src_ids[1]) for command in commands], dtype=np.int64)
        arrays[prefix + "source_raw_target"] = source_raw_at_servo(raw_by_arm[arm], src_lo)
        arrays[prefix + "conditioned_goal"] = conditioned.astype(np.float64)
        arrays[prefix + "conditioned_twist"] = twists.astype(np.float64)
        arrays[prefix + "gripper"] = np.asarray([_nan_if_none(getattr(command, f"{arm}_gripper")) for command in commands], dtype=np.float64)
        for flag in ("valid", "hold", "dropout", "gap", "reanchor"):
            arrays[prefix + flag] = np.asarray([bool(getattr(command, flag)) for command in commands], dtype=bool)
        arrays[prefix + "src_id_lo"] = src_lo
        arrays[prefix + "src_id_hi"] = src_hi
        arrays[prefix + "reference_after_B"] = np.full((t_servo.size, 7), np.nan, dtype=np.float64)
        arrays[prefix + "q_target"] = np.full((t_servo.size, 6), np.nan, dtype=np.float64)
        arrays[prefix + "q_actual"] = np.full((t_servo.size, 6), np.nan, dtype=np.float64)
        arrays[prefix + "actual_tcp"] = np.full((t_servo.size, 7), np.nan, dtype=np.float64)
    return arrays


def write_clean_source(path: Path, conditioner: CommandConditioner, meta: dict[str, Any]) -> None:
    clean = conditioner.clean_source
    arrays = {
        "t_source": clean["t_source"],
        "left_clean_pose": _nan_source(clean["left_pose"], 7),
        "right_clean_pose": _nan_source(clean["right_pose"], 7),
        "segments": np.asarray(conditioner.segments, dtype=np.int64).reshape(-1, 2),
        "gaps": conditioner.gaps.astype(np.float64).reshape(-1, 2),
        "meta_json": np.asarray(json.dumps(_jsonable({**meta, "artifact": "clean_source_pre_foh"}), sort_keys=True)),
    }
    np.savez(path, **arrays)
    print(f"wrote {path}")


def effective_timestamps(episode: EpisodeData, nominal_rate_hz: float) -> tuple[np.ndarray, bool]:
    if episode.t_source is not None and episode.t_source.size:
        return np.asarray(episode.t_source, dtype=np.float64).reshape(-1), False
    lengths = [
        arr.shape[0]
        for arr in (episode.left_pose, episode.right_pose, episode.left_gripper, episode.right_gripper)
        if arr is not None and arr.ndim >= 1
    ]
    count = max(lengths) if lengths else 0
    return np.arange(count, dtype=np.float64) / float(nominal_rate_hz), True


def servo_times(t_source: np.ndarray, servo_rate_hz: float) -> np.ndarray:
    if t_source.size == 0:
        return np.asarray([], dtype=np.float64)
    start = float(t_source[0])
    stop = float(t_source[-1])
    dt = 1.0 / float(servo_rate_hz)
    count = int(np.floor((stop - start) / dt + 1e-9)) + 1
    times = start + np.arange(max(1, count), dtype=np.float64) * dt
    if times[-1] < stop - 0.5 * dt:
        times = np.append(times, stop)
    elif abs(times[-1] - stop) <= 0.5 * dt:
        times[-1] = stop
    return times.astype(np.float64)


def source_raw_at_servo(source_pose: np.ndarray | None, src_lo: np.ndarray) -> np.ndarray:
    out = np.full((src_lo.size, 7), np.nan, dtype=np.float64)
    if source_pose is None:
        return out
    poses = np.asarray(source_pose, dtype=np.float64)
    valid = (src_lo >= 0) & (src_lo < poses.shape[0])
    out[valid] = poses[src_lo[valid]]
    return out


def emit_sweep_matrix(path: Path, cfg: Config) -> int:
    rows = cfg.metrics.sweep_config_matrix()
    payload = {"sweep_matrix": rows}
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.suffix.lower() in {".yaml", ".yml"}:
        try:
            import yaml
        except ModuleNotFoundError as exc:
            raise RuntimeError("PyYAML is required to emit YAML sweep matrices") from exc
        path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    else:
        path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"wrote {path}")
    return 0


def episode_id(path: Path) -> str:
    return f"{path.parent.name}__{path.stem}"


def git_commit() -> str | None:
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPO_ROOT, text=True, stderr=subprocess.DEVNULL).strip()
    except Exception:
        return None


def _with_cli_overrides(cfg: Config, servo_rate_hz: float, seed: int | None) -> Config:
    from dataclasses import replace

    synthetic = cfg.synthetic if seed is None else replace(cfg.synthetic, seed=int(seed))
    conditioning = replace(cfg.conditioning, servo_rate_hz=float(servo_rate_hz))
    return replace(cfg, conditioning=conditioning, synthetic=synthetic)


def _parse_arms(text: str) -> tuple[str, ...]:
    arms = tuple(item.strip() for item in text.split(",") if item.strip())
    invalid = sorted(set(arms) - set(ARMS))
    if invalid:
        raise ValueError(f"invalid arm(s): {', '.join(invalid)}")
    return arms or ARMS


def _row_or_none(array: np.ndarray | None, index: int) -> np.ndarray | None:
    if array is None or index >= array.shape[0]:
        return None
    return np.asarray(array[index], dtype=np.float64)


def _value_or_none(array: np.ndarray | None, index: int) -> float | None:
    if array is None or index >= array.shape[0]:
        return None
    return float(array[index])


def _twist_or_nan(value) -> np.ndarray:
    if value is None:
        return np.full(6, np.nan, dtype=np.float64)
    return np.asarray(value, dtype=np.float64).reshape(6)


def _nan_if_none(value) -> float:
    return np.nan if value is None else float(value)


def _nan_source(value: np.ndarray | None, width: int) -> np.ndarray:
    if value is None:
        return np.empty((0, width), dtype=np.float64)
    return np.asarray(value, dtype=np.float64)


def _slice_optional_array(value: np.ndarray | None, start: int, stop: int) -> np.ndarray | None:
    if value is None:
        return None
    return np.asarray(value)[int(start) : int(stop)]


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


if __name__ == "__main__":
    raise SystemExit(main())
