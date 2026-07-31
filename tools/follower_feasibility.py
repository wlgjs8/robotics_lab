#!/usr/bin/env python3
"""Chunk-follower feasibility readout from a servo CSV.

The follower solves one Ruckig segment per policy step with
``minimum_duration = dt`` (chunk_follower_core.hpp::buildInput), consumes only
the first ``dt`` of that solution, then re-solves toward the NEXT knot. So

    duration / dt

is the honest measure of how much of each planned motion actually executes:
1.0 = the segment completes exactly on schedule, 3.0 = only the first third runs
before the plan is thrown away and re-aimed. A chronically high ratio produces a
sawtooth velocity profile -- accelerate toward a target, get cut off, re-aim,
accelerate again -- which is what reads as trembling.

Measured 2026-07-31 (right arm, real hardware):
    speed 0.75, a=5.0 j=300   -> p50 1.07-1.20   no trembling
    speed 1.00, a=9.0 j=700   -> p50 1.715       trembling
    speed 1.00, a=5.0 j=300   -> p50 1.98-3.19   trembling
so the practical target is p50 <~ 1.3.

Unlike anything derived from differencing tcp_command_stand, this reads a value
the follower itself reports, so it is immune to the 1 um CSV quantization that
makes 500 Hz jerk estimates meaningless (1 LSB = 125 m/s^3 at dt=2 ms).

Usage:
  tools/follower_feasibility.py logs/servo_log_*.csv [--arm right] [--dt-ms 33.4]
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np


def _columns(arm: str) -> tuple[str, ...]:
    return (
        f"{arm}_follower_active",
        f"{arm}_follower_duration_sec",
        f"{arm}_follower_converged",
        f"{arm}_follower_corner",
        f"{arm}_follower_projection_error_m",
        f"{arm}_follower_actual_lead_m",
    )


def _read(path: Path, arm: str) -> dict[str, np.ndarray]:
    wanted = _columns(arm)
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        reader = csv.reader(handle)
        header = next(reader)
        idx = {name: header.index(name) for name in wanted if name in header}
        missing = [name for name in wanted if name not in idx]
        if missing:
            raise KeyError(f"missing columns: {missing}")
        cols: dict[str, list[str]] = {name: [] for name in idx}
        for row in reader:
            if len(row) < len(header):
                continue
            for name, i in idx.items():
                cols[name].append(row[i])
    out = {}
    for name, values in cols.items():
        out[name] = np.asarray(
            [float(v) if v not in ("", "nan") else np.nan for v in values],
            dtype=np.float64,
        )
    return out


def report(path: Path, arm: str, dt: float | None) -> None:
    data = _read(path, arm)
    active = data[f"{arm}_follower_active"] == 1.0
    if not active.any():
        print(f"{path.name}: follower never active")
        return
    dur = data[f"{arm}_follower_duration_sec"][active]
    dur = dur[np.isfinite(dur) & (dur > 0)]
    if dur.size == 0:
        print(f"{path.name}: no segment solves logged")
        return
    # minimum_duration == dt, so the smallest observed duration IS the step budget
    # unless every segment overran; prefer the explicit --dt-ms when given.
    step = dt if dt is not None else float(np.percentile(dur, 1))
    ratio = dur / step
    conv = data[f"{arm}_follower_converged"][active]
    corner = data[f"{arm}_follower_corner"][active]
    proj = data[f"{arm}_follower_projection_error_m"][active]
    lead = data[f"{arm}_follower_actual_lead_m"][active]
    print(f"{path.name}  arm={arm}  dt={step * 1000:.1f}ms  active_ticks={int(active.sum())}")
    print(
        f"    duration/dt : p50={np.median(ratio):.3f}  p90={np.percentile(ratio, 90):.3f}  "
        f"p99={np.percentile(ratio, 99):.3f}  max={ratio.max():.2f}"
        f"   >1.5:{100 * np.mean(ratio > 1.5):.1f}%  >2:{100 * np.mean(ratio > 2):.1f}%"
    )
    print(
        f"    follower    : converged={100 * np.nanmean(conv):.1f}%  corner={100 * np.nanmean(corner):.1f}%  "
        f"projP95={np.nanpercentile(proj, 95) * 1000:.2f}mm  leadP95={np.nanpercentile(lead, 95) * 1000:.2f}mm"
    )
    verdict = "OK" if np.median(ratio) <= 1.3 else "INFEASIBLE (expect trembling)"
    print(f"    verdict     : {verdict}   [target p50 <= 1.3]")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+", type=Path)
    ap.add_argument("--arm", default="right", choices=["left", "right"])
    ap.add_argument("--dt-ms", type=float, default=None,
                    help="policy step budget in ms (default: infer from the 1st percentile duration)")
    args = ap.parse_args()
    for path in args.logs:
        try:
            report(path, args.arm, None if args.dt_ms is None else args.dt_ms / 1000.0)
        except Exception as exc:  # noqa: BLE001 - one bad log must not kill the report
            print(f"{path.name}: {type(exc).__name__}: {exc}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
