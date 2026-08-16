#!/usr/bin/env python3
"""Attribute chunk-follower trembling to the corner guard across a config sweep.

Reads the manifest written by tools/corner_guard_sweep.sh and, per config, reports

  corner%      fraction of ACTIVE ticks with the follower corner flag set
  conv%        fraction of ACTIVE ticks whose segment solve converged within dt
  projP95      p95 of the per-segment projection error (requested knot vs reached)
  proj>1mm     fraction of active ticks over the sim profile's 1 mm bound
  leadP95      p95 commanded-vs-measured lead (servo-chain tracking, NOT plan fidelity)
  tremble%     share of 500 Hz commanded Cartesian-VELOCITY power in 3-20 Hz

Why velocity and not jerk: the servo CSV logs tcp_command_stand to 1 um, and
triple-differencing at 2 ms multiplies one quantization LSB into 125 m/s^3, which
swamps the real signal. The first derivative has 0.5 mm/s resolution against a
~50 mm/s signal, so its spectrum is trustworthy; 3-20 Hz is the band a human sees
and feels as trembling (the 30 Hz segment rate aliases into its top end).
"""
from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

import numpy as np

ARM = "right"
TREMBLE_BAND = (3.0, 20.0)
SIM_PROJECTION_BOUND_M = 0.001

COLUMNS = (
    "period_ms safety_verdict "
    f"{ARM}_tcp_command_stand_x_m {ARM}_tcp_command_stand_y_m {ARM}_tcp_command_stand_z_m "
    f"{ARM}_follower_active {ARM}_follower_converged {ARM}_follower_corner "
    f"{ARM}_follower_projection_error_m {ARM}_follower_actual_lead_m"
).split()


def _load(path: Path) -> dict[str, np.ndarray]:
    """Stream the servo CSV, keeping only the columns this analysis needs."""
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        reader = csv.reader(handle)
        header = next(reader)
        idx = {name: header.index(name) for name in COLUMNS if name in header}
        missing = [name for name in COLUMNS if name not in idx]
        if missing:
            raise KeyError(f"{path.name}: missing columns {missing}")
        cols: dict[str, list] = {name: [] for name in idx}
        for row in reader:
            if len(row) < len(header):
                continue
            for name, i in idx.items():
                cols[name].append(row[i])
    out: dict[str, np.ndarray] = {}
    for name, values in cols.items():
        arr = np.asarray(values, dtype=object)
        if name == "safety_verdict":
            out[name] = arr.astype(str)
        else:
            out[name] = np.asarray(
                [float(v) if v not in ("", "nan") else np.nan for v in values],
                dtype=np.float64,
            )
    return out


def _tremble_share(vel: np.ndarray, dt: float) -> float:
    """Share of commanded-speed spectral power inside the visible trembling band."""
    speed = np.linalg.norm(vel, axis=1)
    speed = speed[np.isfinite(speed)]
    if speed.size < 1024:
        return float("nan")
    speed = speed - speed.mean()
    window = np.hanning(speed.size)
    power = np.abs(np.fft.rfft(speed * window)) ** 2
    freq = np.fft.rfftfreq(speed.size, dt)
    # Ignore DC/drift below 1 Hz: that is task motion, not trembling.
    total = power[(freq >= 1.0) & (freq <= 1.0 / (2 * dt))].sum()
    if total <= 0:
        return float("nan")
    band = power[(freq >= TREMBLE_BAND[0]) & (freq <= TREMBLE_BAND[1])].sum()
    return 100.0 * band / total


def summarize(path: Path) -> dict[str, float]:
    data = _load(path)
    dt = float(np.nanmedian(data["period_ms"])) / 1000.0
    active = data[f"{ARM}_follower_active"] == 1.0
    n_active = int(active.sum())
    pos = np.column_stack(
        [data[f"{ARM}_tcp_command_stand_{a}_m"] for a in "xyz"]
    )
    vel = np.diff(pos, axis=0) / dt
    # Restrict the spectrum to the stretch where the follower actually drove.
    vel_active = vel[active[1:]] if n_active > 512 else vel
    proj = data[f"{ARM}_follower_projection_error_m"][active]
    lead = data[f"{ARM}_follower_actual_lead_m"][active]
    verdict = data["safety_verdict"]
    return {
        "ticks": float(len(verdict)),
        "active_pct": 100.0 * n_active / max(1, len(verdict)),
        "corner_pct": 100.0 * float(np.nanmean(data[f"{ARM}_follower_corner"][active])) if n_active else float("nan"),
        "conv_pct": 100.0 * float(np.nanmean(data[f"{ARM}_follower_converged"][active])) if n_active else float("nan"),
        "proj_p95_mm": float(np.nanpercentile(proj, 95)) * 1000.0 if proj.size else float("nan"),
        "proj_over_pct": 100.0 * float(np.nanmean(proj > SIM_PROJECTION_BOUND_M)) if proj.size else float("nan"),
        "lead_p95_mm": float(np.nanpercentile(lead, 95)) * 1000.0 if lead.size else float("nan"),
        "tremble_pct": _tremble_share(vel_active, dt),
        "roi": float((verdict == "RoiViolation").sum()),
        "faults": float((~np.isin(verdict, ["Ok", "RoiViolation"])).sum()),
    }


def main() -> int:
    manifest = Path(sys.argv[1] if len(sys.argv) > 1 else "/tmp/corner_sweep/manifest.tsv")
    rows = list(csv.DictReader(manifest.open(encoding="utf-8"), delimiter="\t"))
    if not rows:
        print(f"no runs in {manifest}")
        return 1
    print(
        f"{'tag':<9} {'db_ang(deg)':>11} {'vscale':>7} {'ticks':>7} {'act%':>6} "
        f"{'corner%':>8} {'conv%':>7} {'projP95':>9} {'proj>1mm':>9} {'leadP95':>9} "
        f"{'tremble%':>9} {'ROI':>5}"
    )
    for row in rows:
        servo = Path(row["servo_log"])
        if not servo.exists():
            print(f"{row['tag']:<9} (servo log missing: {servo})")
            continue
        try:
            s = summarize(servo)
        except Exception as exc:  # noqa: BLE001 - one bad run must not kill the report
            print(f"{row['tag']:<9} (analysis failed: {type(exc).__name__}: {exc})")
            continue
        deg = math.degrees(float(row["deadband_ang_rad"]))
        print(
            f"{row['tag']:<9} {deg:11.3f} {float(row['vel_scale']):7.2f} {s['ticks']:7.0f} "
            f"{s['active_pct']:6.1f} {s['corner_pct']:8.1f} {s['conv_pct']:7.1f} "
            f"{s['proj_p95_mm']:8.2f}mm {s['proj_over_pct']:8.1f}% {s['lead_p95_mm']:8.2f}mm "
            f"{s['tremble_pct']:9.2f} {s['roi']:5.0f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
