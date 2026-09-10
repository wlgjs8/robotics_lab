#!/usr/bin/env python3
"""Aggregate `camera_light_recorder` sessions by local hour and compare AM vs PM.

The recorder logs one photometry row per camera per sample; this reads any number
of session directories (or `metrics.csv` paths) and prints:

  1. a per-hour median table per arm (illumination, colour cast, clipping, focus),
  2. an AM-vs-PM delta, using the hour windows given by `--am` / `--pm`.

Usage:
    .venv/bin/python tools/analyze_camera_light.py logs/light/am_good_*/ \
        --am 9-12 --pm 13-18
"""
from __future__ import annotations

import argparse
import csv
import statistics
from pathlib import Path
from typing import Any

_COLUMNS = [
    ("lum_mean", "lum"),
    ("lum_p05", "p05"),
    ("lum_p95", "p95"),
    ("lum_mean_center", "center"),
    ("r_mean", "R"),
    ("g_mean", "G"),
    ("b_mean", "B"),
    ("clip_hi_frac", "clip_hi"),
    ("clip_lo_frac", "clip_lo"),
    ("focus_gradient_energy", "focus"),
    ("actual_exposure_us", "exp_us"),
    ("gain_level", "gain"),
    ("depth_gain_level", "d_gain"),
]


def _metrics_files(targets: list[str]) -> list[Path]:
    files: list[Path] = []
    for target in targets:
        path = Path(target)
        if path.is_dir():
            found = sorted(path.glob("**/metrics.csv"))
            if not found:
                raise SystemExit(f"no metrics.csv under {path}")
            files.extend(found)
        elif path.is_file():
            files.append(path)
        else:
            raise SystemExit(f"not found: {target}")
    return files


def _load(files: list[Path]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for path in files:
        with path.open() as handle:
            for row in csv.DictReader(handle):
                try:
                    row["_hour"] = float(row["hour_local"])
                except (KeyError, TypeError, ValueError):
                    continue
                row["_session"] = path.parent.name
                rows.append(row)
    return rows


def _median(rows: list[dict[str, Any]], key: str) -> float | None:
    values = []
    for row in rows:
        raw = row.get(key)
        if raw in (None, ""):
            continue
        try:
            values.append(float(raw))
        except ValueError:
            continue
    return statistics.median(values) if values else None


def _fmt(value: float | None, digits: int = 1) -> str:
    return "  n/a" if value is None else f"{value:.{digits}f}"


def _window(spec: str) -> tuple[float, float]:
    lo, _, hi = spec.partition("-")
    return float(lo), float(hi)


def _table(rows: list[dict[str, Any]], arm: str) -> None:
    arm_rows = [row for row in rows if row.get("arm") == arm]
    if not arm_rows:
        return
    hours = sorted({int(row["_hour"]) for row in arm_rows})
    header = f"{'hour':>5}{'n':>7}" + "".join(f"{label:>9}" for _, label in _COLUMNS)
    print(f"\n--- {arm} arm ---")
    print(header)
    for hour in hours:
        bucket = [row for row in arm_rows if int(row["_hour"]) == hour]
        digits = {"clip_hi_frac": 4, "clip_lo_frac": 4, "focus_gradient_energy": 0, "actual_exposure_us": 0}
        line = f"{hour:>5}{len(bucket):>7}"
        for key, _ in _COLUMNS:
            line += f"{_fmt(_median(bucket, key), digits.get(key, 1)):>9}"
        print(line)


def _compare(rows: list[dict[str, Any]], arm: str, am: tuple[float, float], pm: tuple[float, float]) -> None:
    arm_rows = [row for row in rows if row.get("arm") == arm]
    am_rows = [row for row in arm_rows if am[0] <= row["_hour"] < am[1]]
    pm_rows = [row for row in arm_rows if pm[0] <= row["_hour"] < pm[1]]
    if not am_rows or not pm_rows:
        print(f"\n{arm}: need samples in both windows (am={len(am_rows)} pm={len(pm_rows)})")
        return
    print(f"\n--- {arm}: AM {am[0]:g}-{am[1]:g}h (n={len(am_rows)}) vs PM {pm[0]:g}-{pm[1]:g}h (n={len(pm_rows)}) ---")
    print(f"{'metric':>24}{'AM':>10}{'PM':>10}{'delta':>10}{'delta_%':>9}")
    for key, label in _COLUMNS:
        am_value, pm_value = _median(am_rows, key), _median(pm_rows, key)
        if am_value is None or pm_value is None:
            continue
        delta = pm_value - am_value
        pct = (delta / am_value * 100.0) if am_value else float("nan")
        print(f"{label:>24}{am_value:>10.3f}{pm_value:>10.3f}{delta:>10.3f}{pct:>9.1f}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("sessions", nargs="+", help="session directories or metrics.csv paths")
    parser.add_argument("--am", default="8-12", help="AM window as start-end local hours")
    parser.add_argument("--pm", default="13-19", help="PM window as start-end local hours")
    args = parser.parse_args(argv)

    files = _metrics_files(args.sessions)
    rows = _load(files)
    if not rows:
        raise SystemExit("no rows loaded")
    sessions = sorted({row["_session"] for row in rows})
    print(f"loaded {len(rows)} rows from {len(files)} file(s): {', '.join(sessions)}")
    for arm in ("left", "right"):
        _table(rows, arm)
    am, pm = _window(args.am), _window(args.pm)
    for arm in ("left", "right"):
        _compare(rows, arm, am, pm)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
