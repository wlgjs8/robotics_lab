#!/usr/bin/env python3
"""Select raw UMI episodes suitable for TcpTargetPose replay profiling.

Read-only. Scans recorded episodes (nominally 30 Hz) and flags any episode whose
recording rate broke up into discontinuous segments because of the collection-code
issue: if any single inter-frame step exceeds the gap threshold (default 50 ms),
the episode is excluded from the profiling set.

Only the ``timestamp`` dataset is read from each HDF5 file, so this stays fast even
on multi-hundred-MB episodes, and it can be re-run while the copy from HDD is still
in progress: files that are still being copied open as truncated and are reported as
INCOMPLETE (re-check after the copy finishes) rather than mislabeled as bad.

Classification per episode:
  INCLUDE     max inter-frame dt <= gap-ms                  -> profiling candidate
  EXCLUDE     max dt > gap-ms (and not borderline)          -> dropped
  BORDERLINE  max dt > gap-ms but within review window      -> dropped-by-rule, listed
              (max dt <= borderline-hi-ms and few gaps)        for manual promotion
  INCOMPLETE  file unreadable / truncated / no timestamps   -> re-run after copy
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import h5py
import numpy as np

SCHEMA = "robotics_lab.select_replay_episodes.v1"

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent

DEFAULT_DATA_DIR = REPO_ROOT / "data"
DEFAULT_OUT_DIR = REPO_ROOT / "outputs" / "episode_selection"

NOMINAL_RATE_HZ = 30.0
GAP_MS = 50.0
# Borderline window: excluded by the rule, but close enough to flag for a human
# call (e.g. a single ~60 ms hiccup vs. a multi-hundred-ms collapse).
BORDERLINE_HI_MS = 80.0
BORDERLINE_MAX_GAPS = 2


@dataclass
class EpisodeRecord:
    path: str
    session: str
    episode: str
    status: str
    frame_count: int = 0
    duration_sec: float = 0.0
    effective_hz: float = 0.0
    nominal_record_hz: float | None = None
    median_dt_ms: float = 0.0
    max_dt_ms: float = 0.0
    gap_count: int = 0
    gap_indices: list[int] = field(default_factory=list)
    gap_dt_ms: list[float] = field(default_factory=list)
    non_monotonic_count: int = 0
    note: str = ""


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA_DIR, help="Root holding data_*/episode_*.hdf5")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR, help="Where to write the selection report")
    parser.add_argument("--gap-ms", type=float, default=GAP_MS, help="Exclude episode if any step exceeds this (ms)")
    parser.add_argument("--borderline-hi-ms", type=float, default=BORDERLINE_HI_MS, help="Upper bound of the review window (ms)")
    parser.add_argument("--borderline-max-gaps", type=int, default=BORDERLINE_MAX_GAPS, help="Max gaps to still count as borderline")
    parser.add_argument(
        "--include-borderline",
        action="store_true",
        help="Fold BORDERLINE episodes into the INCLUDE list (default: keep them out)",
    )
    parser.add_argument("--glob", default="data_*/episode_*.hdf5", help="Episode glob under --data-dir")
    return parser.parse_args(argv)


def read_timestamps_seconds(path: Path) -> np.ndarray:
    """Read the episode timestamp vector and normalize to seconds.

    Mirrors policy_runner.umi_pipeline._timestamps_to_seconds: epoch-second floats
    are passed through; nanosecond integers (huge span / large median dt) are scaled.
    """
    with h5py.File(path, "r") as handle:
        if "timestamp" in handle:
            raw = np.asarray(handle["timestamp"])
        elif "observations" in handle and "timestamp" in handle["observations"]:
            raw = np.asarray(handle["observations"]["timestamp"])
        else:
            raise KeyError("no 'timestamp' dataset")
    values = np.asarray(raw, dtype=np.float64)
    if values.size <= 1:
        return values
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return values
    span = float(np.nanmax(finite) - np.nanmin(finite))
    positive_dt = np.diff(values)
    positive_dt = positive_dt[np.isfinite(positive_dt) & (positive_dt > 0.0)]
    median_dt = float(np.median(positive_dt)) if positive_dt.size else 0.0
    if span > 1e6 or median_dt > 1000.0:
        return values / 1e9
    return values


def read_nominal_record_hz(path: Path) -> float | None:
    try:
        with h5py.File(path, "r") as handle:
            val = handle.attrs.get("record_hz")
        return float(val) if val is not None else None
    except Exception:
        return None


def classify(record: EpisodeRecord, args: argparse.Namespace) -> str:
    if record.max_dt_ms <= args.gap_ms:
        return "INCLUDE"
    if record.max_dt_ms <= args.borderline_hi_ms and record.gap_count <= args.borderline_max_gaps:
        return "BORDERLINE"
    return "EXCLUDE"


def evaluate_episode(path: Path, data_dir: Path, args: argparse.Namespace) -> EpisodeRecord:
    rel = path.relative_to(data_dir)
    session = rel.parts[0] if len(rel.parts) > 1 else ""
    rec = EpisodeRecord(path=str(path), session=session, episode=path.stem, status="INCOMPLETE")
    try:
        ts = read_timestamps_seconds(path)
    except (OSError, KeyError) as exc:
        rec.note = f"unreadable/truncated: {exc}"
        return rec
    rec.nominal_record_hz = read_nominal_record_hz(path)
    rec.frame_count = int(ts.size)
    if ts.size <= 1:
        rec.note = "too few frames"
        return rec

    dt = np.diff(ts)
    finite = dt[np.isfinite(dt)]
    positive = finite[finite > 0.0]
    rec.non_monotonic_count = int(np.count_nonzero((~np.isfinite(dt)) | (dt <= 0.0)))
    rec.duration_sec = round(float(ts[-1] - ts[0]), 6) if math.isfinite(float(ts[-1] - ts[0])) else 0.0
    rec.effective_hz = round(ts.size / rec.duration_sec, 3) if rec.duration_sec > 0 else 0.0
    rec.median_dt_ms = round(float(np.median(positive)) * 1000.0, 3) if positive.size else 0.0
    rec.max_dt_ms = round(float(positive.max()) * 1000.0, 3) if positive.size else 0.0

    gap_thr_s = args.gap_ms / 1000.0
    gap_idx = np.nonzero(dt > gap_thr_s)[0]
    rec.gap_count = int(gap_idx.size)
    rec.gap_indices = [int(i) for i in gap_idx]
    rec.gap_dt_ms = [round(float(dt[i]) * 1000.0, 1) for i in gap_idx]

    rec.status = classify(rec, args)
    return rec


def build_summary(records: list[EpisodeRecord], args: argparse.Namespace) -> dict[str, Any]:
    by_status: dict[str, int] = {}
    for rec in records:
        by_status[rec.status] = by_status.get(rec.status, 0) + 1
    return {
        "schema": SCHEMA,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "data_dir": str(args.data_dir),
        "gap_ms": args.gap_ms,
        "borderline_hi_ms": args.borderline_hi_ms,
        "borderline_max_gaps": args.borderline_max_gaps,
        "include_borderline": args.include_borderline,
        "episode_count": len(records),
        "by_status": by_status,
    }


def included_paths(records: list[EpisodeRecord], include_borderline: bool) -> list[str]:
    keep = {"INCLUDE"} | ({"BORDERLINE"} if include_borderline else set())
    return sorted(rec.path for rec in records if rec.status in keep)


def render_markdown(records: list[EpisodeRecord], summary: dict[str, Any], include_list: list[str]) -> str:
    lines = ["# Replay Episode Selection", ""]
    lines.append(f"- Schema: `{summary['schema']}`")
    lines.append(f"- Created: {summary['created_utc']}")
    lines.append(f"- Data dir: `{summary['data_dir']}`")
    lines.append(
        f"- Gap threshold: {summary['gap_ms']} ms "
        f"(borderline window <= {summary['borderline_hi_ms']} ms and <= {summary['borderline_max_gaps']} gaps)"
    )
    lines.append("")
    lines.append("## Summary")
    for status in ("INCLUDE", "BORDERLINE", "EXCLUDE", "INCOMPLETE"):
        lines.append(f"- {status}: {summary['by_status'].get(status, 0)}")
    lines.append(f"- INCLUDE list size (include_borderline={summary['include_borderline']}): {len(include_list)}")
    if summary["by_status"].get("INCOMPLETE"):
        lines.append("")
        lines.append("> INCOMPLETE episodes are likely still being copied. Re-run after the copy finishes.")
    lines.append("")

    def table(title: str, rows: list[EpisodeRecord], with_gaps: bool) -> None:
        if not rows:
            return
        lines.append(f"## {title} ({len(rows)})")
        header = "| episode | frames | eff_hz | median_dt | max_dt | gaps |"
        sep = "| --- | ---: | ---: | ---: | ---: | ---: |"
        if not with_gaps:
            header = "| episode | frames | eff_hz | note |"
            sep = "| --- | ---: | ---: | --- |"
        lines.append(header)
        lines.append(sep)
        for rec in rows:
            name = f"{rec.session}/{rec.episode}"
            if with_gaps:
                gap_str = f"{rec.gap_count}"
                if rec.gap_dt_ms:
                    gap_str += " @" + ",".join(f"{i}:{d:.0f}ms" for i, d in zip(rec.gap_indices, rec.gap_dt_ms))
                lines.append(
                    f"| {name} | {rec.frame_count} | {rec.effective_hz} | "
                    f"{rec.median_dt_ms:.1f}ms | {rec.max_dt_ms:.1f}ms | {gap_str} |"
                )
            else:
                lines.append(f"| {name} | {rec.frame_count} | {rec.effective_hz} | {rec.note} |")
        lines.append("")

    table("BORDERLINE (review — promote to INCLUDE manually if acceptable)", [r for r in records if r.status == "BORDERLINE"], True)
    table("EXCLUDE (broken segments)", [r for r in records if r.status == "EXCLUDE"], True)
    table("INCOMPLETE (re-check after copy)", [r for r in records if r.status == "INCOMPLETE"], False)
    # INCLUDE table last and terse: usually the largest group.
    table("INCLUDE (profiling candidates)", [r for r in records if r.status == "INCLUDE"], True)
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    data_dir = args.data_dir
    episodes = sorted(data_dir.glob(args.glob))
    if not episodes:
        print(f"no episodes matched {data_dir / args.glob}", file=sys.stderr)
        return 1

    records = [evaluate_episode(p, data_dir, args) for p in episodes]
    summary = build_summary(records, args)
    include_list = included_paths(records, args.include_borderline)

    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "selection.json").write_text(
        json.dumps({"summary": summary, "episodes": [asdict(r) for r in records]}, indent=2),
        encoding="utf-8",
    )
    (out_dir / "selection.md").write_text(render_markdown(records, summary, include_list), encoding="utf-8")
    (out_dir / "include_list.txt").write_text("\n".join(include_list) + ("\n" if include_list else ""), encoding="utf-8")

    bs = summary["by_status"]
    print(
        f"episodes={len(records)} "
        f"INCLUDE={bs.get('INCLUDE', 0)} BORDERLINE={bs.get('BORDERLINE', 0)} "
        f"EXCLUDE={bs.get('EXCLUDE', 0)} INCOMPLETE={bs.get('INCOMPLETE', 0)}"
    )
    print(f"include_list={len(include_list)} -> {out_dir / 'include_list.txt'}")
    print(f"wrote {out_dir / 'selection.json'}")
    print(f"wrote {out_dir / 'selection.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
