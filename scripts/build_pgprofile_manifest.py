#!/usr/bin/env python3
"""Build the offline episode manifest for the TcpTargetPose pgmode profiling campaign.

For every converted ``data_tcp`` episode this:
  1. loads it read-only and runs the richer HDF5 audit (``tools/audit_episode_hdf5``),
  2. writes ``audit.json`` + ``audit_summary.md`` under
     ``outputs/tcp_pgprofile/<episode_id>/``,
  3. assigns a data-quality ``validity_class`` and a time_scale=1.0 speed precheck
     (``required_time_scale_estimate``) against the server SMD velocity limits,
  4. aggregates ``episode_manifest.json`` + ``episode_manifest.csv``.

This is offline only (no robot). Live IK/safety classification is layered on later
by the replay driver. Raw HDF5 episodes are opened read-only; all artifacts go
under ``outputs/``.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np
import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
for path in (str(REPO_ROOT), str(REPO_ROOT / "tools")):
    if path not in sys.path:
        sys.path.insert(0, path)

import audit_episode_hdf5 as audit_mod  # noqa: E402
from tcp_tuning.config import load_config  # noqa: E402
from tcp_tuning.hdf5_io import load_episode  # noqa: E402

MANIFEST_SCHEMA = "robotics_lab.tcp_pgprofile.manifest.v1"

# Data-quality validity classes (spec phase 2).
VALID_FULL_NO_GAP = "VALID_FULL_NO_GAP"
VALID_SEGMENTED_GAPS = "VALID_SEGMENTED_GAPS"
TOO_SHORT = "TOO_SHORT"
BAD_TIMESTAMP = "BAD_TIMESTAMP"
MISSING_POSE = "MISSING_POSE"
BAD_QUATERNION = "BAD_QUATERNION"
NEEDS_MANUAL_REVIEW = "NEEDS_MANUAL_REVIEW"

MANIFEST_COLUMNS = [
    "episode_id",
    "path",
    "n_frames",
    "duration_sec",
    "nominal_frequency_hz",
    "gap_count",
    "segment_count",
    "largest_segment_id",
    "largest_segment_start",
    "largest_segment_end",
    "largest_segment_duration_sec",
    "left_path_length_m",
    "right_path_length_m",
    "left_linear_speed_p95",
    "left_linear_speed_max",
    "right_linear_speed_p95",
    "right_linear_speed_max",
    "left_angular_speed_p95",
    "left_angular_speed_max",
    "right_angular_speed_p95",
    "right_angular_speed_max",
    "max_stream_linear_speed_m_s",
    "max_stream_angular_speed_rad_s",
    "configured_max_linear_m_s",
    "configured_max_angular_rad_s",
    "linear_speed_margin",
    "angular_speed_margin",
    "required_time_scale_estimate",
    "speed_precheck_pass",
    "validity_class",
    "notes",
]


def server_speed_limits(server_config: Path | None) -> tuple[float, float]:
    """Return (max_linear_velocity_m_s, max_angular_velocity_rad_s) from the SMD config."""
    default = (0.25, 1.745329252)
    if server_config is None or not server_config.exists():
        return default
    data = yaml.safe_load(server_config.read_text()) or {}
    smd = (data.get("cartesian_control", {}) or {}).get("pose_track_smd", {}) or {}
    lin = float(smd.get("max_linear_velocity_m_s", default[0]))
    ang = float(smd.get("max_angular_velocity_rad_s", default[1]))
    return lin, ang


def classify_validity(report: dict[str, Any], *, min_frames: int) -> tuple[str, list[str]]:
    notes: list[str] = []
    detected = report.get("detected", {})
    timing = report.get("timing", {})
    arms = report.get("arms", {})

    left_present = arms.get("left", {}).get("pose_present", False)
    right_present = arms.get("right", {}).get("pose_present", False)
    if not (left_present or right_present):
        return MISSING_POSE, ["no finite pose for either arm"]

    bad_q = 0
    for side in ("left_pose_conversion", "right_pose_conversion"):
        conv = detected.get(side, {})
        bad_q += int(conv.get("bad_quaternion_rows", 0) or 0)
    if bad_q > 0:
        return BAD_QUATERNION, [f"bad_quaternion_rows={bad_q}"]

    n = int(timing.get("sample_count", 0) or 0)
    if n < min_frames:
        return TOO_SHORT, [f"sample_count={n} < min_frames={min_frames}"]

    # Timestamp sanity: nominal_rate_used means timestamps were absent/unusable.
    if bool(report.get("nominal_rate_used", False)) or bool(detected.get("nominal_rate_used", False)):
        notes.append("timestamps absent -> nominal rate assumed")
    dt = timing.get("dt_sec", {})
    if dt and (float(dt.get("max", 0.0)) <= 0.0):
        return BAD_TIMESTAMP, ["non-positive dt detected"]

    gaps = report.get("gaps", []) or []
    if gaps:
        return VALID_SEGMENTED_GAPS, notes + [f"gap_count={len(gaps)}"]
    return VALID_FULL_NO_GAP, notes


def largest_segment(report: dict[str, Any], nominal_hz: float) -> dict[str, Any]:
    segments = report.get("segments", []) or []
    if not segments:
        return {"id": -1, "start": -1, "end": -1, "duration_sec": 0.0}
    best_i, best = 0, segments[0]
    for i, seg in enumerate(segments):
        if int(seg.get("sample_count", 0)) > int(best.get("sample_count", 0)):
            best_i, best = i, seg
    start = int(best.get("start_index", 0))
    stop = int(best.get("stop_index_exclusive", 0))
    count = int(best.get("sample_count", max(0, stop - start)))
    dur = (count / nominal_hz) if nominal_hz > 0 else 0.0
    return {"id": best_i, "start": start, "end": stop, "duration_sec": dur}


def speed_precheck(
    report: dict[str, Any], *, max_lin: float, max_ang: float, time_scale: float
) -> dict[str, Any]:
    """At time_scale=1.0 the streamed goal speed equals the source speed.

    A faster replay (time_scale<1) scales speed up by 1/time_scale; the canonical
    campaign runs time_scale=1.0 so the effective stream speed == source speed.
    required_time_scale_estimate is the time_scale that would bring the peak
    stream speed down to the configured limit (>1 means must slow down).
    """
    arms = report.get("arms", {})
    lin_peaks, ang_peaks = [], []
    for side in ("left", "right"):
        a = arms.get(side, {})
        if not a.get("pose_present", False):
            continue
        lin_peaks.append(float(a.get("linear_speed_m_s", {}).get("max", 0.0)))
        ang_peaks.append(float(a.get("angular_speed_rad_s", {}).get("max", 0.0)))
    max_stream_lin = (max(lin_peaks) if lin_peaks else 0.0) / max(time_scale, 1e-9)
    max_stream_ang = (max(ang_peaks) if ang_peaks else 0.0) / max(time_scale, 1e-9)
    lin_margin = (max_stream_lin / max_lin) if max_lin > 0 else float("inf")
    ang_margin = (max_stream_ang / max_ang) if max_ang > 0 else float("inf")
    req_ts = max(lin_margin, ang_margin) * time_scale
    return {
        "max_stream_linear_speed_m_s": max_stream_lin,
        "max_stream_angular_speed_rad_s": max_stream_ang,
        "linear_speed_margin": lin_margin,
        "angular_speed_margin": ang_margin,
        "required_time_scale_estimate": req_ts,
        "speed_precheck_pass": bool(lin_margin <= 1.0 and ang_margin <= 1.0),
    }


def process_episode(
    ep_path: Path,
    out_root: Path,
    cfg,
    *,
    max_lin: float,
    max_ang: float,
    time_scale: float,
    min_frames: int,
    write_plots: bool,
) -> dict[str, Any]:
    eid = audit_mod.episode_id(ep_path)
    ep_out = out_root / eid
    ep_out.mkdir(parents=True, exist_ok=True)

    episode = load_episode(str(ep_path), nominal_rate_hz=cfg.audit.nominal_source_rate_hz)
    tree = audit_mod.hdf5_tree(ep_path)
    report = audit_mod.build_audit_report(episode, cfg, tree)

    (ep_out / "audit.json").write_text(
        json.dumps(report, indent=2, allow_nan=False, default=float) + "\n", encoding="utf-8"
    )
    _write_audit_summary(ep_out / "audit_summary.md", report)

    timing = report.get("timing", {})
    nominal_hz = float(timing.get("nominal_frequency_hz", cfg.audit.nominal_source_rate_hz) or cfg.audit.nominal_source_rate_hz)
    seg = largest_segment(report, nominal_hz)
    validity, notes = classify_validity(report, min_frames=min_frames)
    speed = speed_precheck(report, max_lin=max_lin, max_ang=max_ang, time_scale=time_scale)

    arms = report.get("arms", {})

    def arm_stat(side: str, group: str, stat: str) -> float:
        return float(arms.get(side, {}).get(group, {}).get(stat, 0.0) or 0.0)

    row = {
        "episode_id": eid,
        "path": str(ep_path),
        "n_frames": int(timing.get("sample_count", 0) or 0),
        "duration_sec": float(timing.get("duration_sec", 0.0) or 0.0),
        "nominal_frequency_hz": nominal_hz,
        "gap_count": len(report.get("gaps", []) or []),
        "segment_count": len(report.get("segments", []) or []),
        "largest_segment_id": seg["id"],
        "largest_segment_start": seg["start"],
        "largest_segment_end": seg["end"],
        "largest_segment_duration_sec": seg["duration_sec"],
        "left_path_length_m": float(arms.get("left", {}).get("path_length_m", 0.0) or 0.0),
        "right_path_length_m": float(arms.get("right", {}).get("path_length_m", 0.0) or 0.0),
        "left_linear_speed_p95": arm_stat("left", "linear_speed_m_s", "p95"),
        "left_linear_speed_max": arm_stat("left", "linear_speed_m_s", "max"),
        "right_linear_speed_p95": arm_stat("right", "linear_speed_m_s", "p95"),
        "right_linear_speed_max": arm_stat("right", "linear_speed_m_s", "max"),
        "left_angular_speed_p95": arm_stat("left", "angular_speed_rad_s", "p95"),
        "left_angular_speed_max": arm_stat("left", "angular_speed_rad_s", "max"),
        "right_angular_speed_p95": arm_stat("right", "angular_speed_rad_s", "p95"),
        "right_angular_speed_max": arm_stat("right", "angular_speed_rad_s", "max"),
        "configured_max_linear_m_s": max_lin,
        "configured_max_angular_rad_s": max_ang,
        "validity_class": validity,
        "notes": "; ".join(notes),
        **speed,
    }
    return row


def _write_audit_summary(path: Path, report: dict[str, Any]) -> None:
    timing = report.get("timing", {})
    arms = report.get("arms", {})
    lines = [
        f"# Audit — {report.get('episode_id', '?')}",
        "",
        f"- path: `{report.get('episode_path', '?')}`",
        f"- frames: {timing.get('sample_count')}  duration: {timing.get('duration_sec', 0):.3f} s"
        f"  nominal: {timing.get('nominal_frequency_hz', 0):.3f} Hz",
        f"- gaps: {len(report.get('gaps', []) or [])}  segments: {len(report.get('segments', []) or [])}",
        "",
        "| arm | path_len_m | lin_p95 | lin_max | ang_p95 | ang_max |",
        "|---|---|---|---|---|---|",
    ]
    for side in ("left", "right"):
        a = arms.get(side, {})
        if not a:
            continue
        lin = a.get("linear_speed_m_s", {})
        ang = a.get("angular_speed_rad_s", {})
        lines.append(
            f"| {side} | {a.get('path_length_m', 0):.3f} | {lin.get('p95', 0):.3f} | "
            f"{lin.get('max', 0):.3f} | {ang.get('p95', 0):.3f} | {ang.get('max', 0):.3f} |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def discover(episodes_dir: Path, only: str | None) -> list[Path]:
    paths = sorted(episodes_dir.glob("*.hdf5"))
    if only:
        wanted = {tok.strip() for tok in only.split(",") if tok.strip()}
        paths = [p for p in paths if p.stem in wanted or p.stem.split("_")[-1] in wanted]
    return paths


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes-dir", default="data_tcp/replay_profiling_20260620")
    parser.add_argument("--episodes", default=None, help="comma-separated stems/numbers to restrict to")
    parser.add_argument("--out-dir", default="outputs/tcp_pgprofile")
    parser.add_argument("--server-config", default="rb_servo_server/config/local/stack_sim.yaml")
    parser.add_argument("--config", default=None, help="optional tcp_tuning YAML override")
    parser.add_argument("--time-scale", type=float, default=1.0)
    parser.add_argument("--min-frames", type=int, default=15)
    parser.add_argument("--plots", action="store_true")
    parser.add_argument("--limit", type=int, default=0, help="process only first N (0=all)")
    args = parser.parse_args(argv)

    out_root = Path(args.out_dir)
    out_root.mkdir(parents=True, exist_ok=True)
    cfg = load_config(args.config)
    max_lin, max_ang = server_speed_limits(Path(args.server_config) if args.server_config else None)

    episodes = discover(Path(args.episodes_dir), args.episodes)
    if args.limit:
        episodes = episodes[: args.limit]
    if not episodes:
        print("no episodes found", file=sys.stderr)
        return 2

    print(f"manifest: {len(episodes)} episodes  limits lin={max_lin} m/s ang={max_ang} rad/s  time_scale={args.time_scale}")
    rows: list[dict[str, Any]] = []
    errors: list[dict[str, str]] = []
    for i, ep in enumerate(episodes):
        try:
            row = process_episode(
                ep, out_root, cfg,
                max_lin=max_lin, max_ang=max_ang, time_scale=args.time_scale,
                min_frames=args.min_frames, write_plots=args.plots,
            )
            rows.append(row)
        except Exception as exc:  # noqa: BLE001 - record and continue
            eid = audit_mod.episode_id(ep)
            rows.append({
                "episode_id": eid, "path": str(ep), "validity_class": NEEDS_MANUAL_REVIEW,
                "notes": f"audit_error: {type(exc).__name__}: {exc}",
            })
            errors.append({"episode": eid, "error": f"{type(exc).__name__}: {exc}"})
        if (i + 1) % 25 == 0 or (i + 1) == len(episodes):
            print(f"  [{i+1}/{len(episodes)}] {ep.stem}")

    # Aggregate
    manifest = {
        "schema": MANIFEST_SCHEMA,
        "episodes_dir": str(args.episodes_dir),
        "server_config": str(args.server_config),
        "configured_max_linear_m_s": max_lin,
        "configured_max_angular_rad_s": max_ang,
        "time_scale": args.time_scale,
        "n_episodes": len(rows),
        "validity_histogram": _histogram(rows, "validity_class"),
        "speed_precheck_pass_count": sum(1 for r in rows if r.get("speed_precheck_pass")),
        "errors": errors,
        "episodes": rows,
    }
    (out_root / "episode_manifest.json").write_text(
        json.dumps(manifest, indent=2, allow_nan=False, default=float) + "\n", encoding="utf-8"
    )
    with (out_root / "episode_manifest.csv").open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=MANIFEST_COLUMNS, extrasaction="ignore")
        writer.writeheader()
        for r in rows:
            writer.writerow({k: r.get(k, "") for k in MANIFEST_COLUMNS})

    print(f"\nwrote {out_root/'episode_manifest.json'} and .csv")
    print("validity histogram:", json.dumps(manifest["validity_histogram"]))
    print(f"speed precheck pass @ time_scale={args.time_scale}: "
          f"{manifest['speed_precheck_pass_count']}/{len(rows)}")
    if errors:
        print(f"errors: {len(errors)} (see manifest.errors)")
    return 0


def _histogram(rows: list[dict[str, Any]], key: str) -> dict[str, int]:
    out: dict[str, int] = {}
    for r in rows:
        out[r.get(key, "?")] = out.get(r.get(key, "?"), 0) + 1
    return dict(sorted(out.items()))


if __name__ == "__main__":
    raise SystemExit(main())
