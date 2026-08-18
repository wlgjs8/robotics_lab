#!/usr/bin/env python3
"""Compare servo smoothness between runs — the LPF-off (servo_alpha 10.0) verdict tool.

Why this exists
---------------
"Does alpha 10 cause jerk?" was answered by eye twice before, on a stack that
had four other confounders live (pre-26071103 firmware, 6-significant-digit
wire precision, no high_speed_comm, uncontrolled box queue). Those are cleared
as of 2026-08-18, so the flip is finally a single variable — but only if the
verdict is a number comparable across runs.

The measurement trap this tool exists to avoid
----------------------------------------------
Naive derivatives of ``q_actual`` are useless here. Measured on the alpha=1.0
baseline (``logs/servo_log_20260818_183810.csv``), the joint-velocity spectrum
of ``q_actual`` puts **49.6% of its energy in 100-250 Hz** — 153x the level
``q_sent`` has in the same band. That is not robot motion. It is our own state
read: the box updates joint feedback at ~429 Hz asynchronously against the
500 Hz sample, so ~1% of ticks hold and then double-step, which lands as an
alternating component at Nyquist. Feed that to a third difference and you get
jerk_p99 = 3.4e7 deg/s^3, which is exactly the quantization+aliasing floor
(2e-4 deg grid -> sigma_jerk ~ 3.2e7) and says nothing about the robot.

So: everything below is computed on a zero-phase low-passed signal, and the
headline band metric is an ABSOLUTE level in 10-50 Hz, not a fraction of total
energy. A fraction would be dominated by the Nyquist artifact in its
denominator and would move for reasons that have nothing to do with alpha.

What it reports, per arm, on MOVING samples only
------------------------------------------------
* ``band_10_50``  - absolute velocity energy in 10-50 Hz. THE headline. This is
  where a human feels jerk and where the box reference LPF acts. On the alpha=1.0
  baseline this band holds 0.05-0.10% of total energy, i.e. there is enormous
  headroom to detect a rise. LPF-off should raise it; how much is the question.
* ``accel_rms``, ``jerk_p99`` - magnitudes after the low-pass, so they track real
  motion. Compare ratios between runs, not absolute values.
* ``track_rms`` - |q_actual - q_sent| after shifting q_sent by the measured
  transport lag, in the POSITION domain where the 2e-4 deg grid is negligible.
  LPF-off should IMPROVE this (less reference attenuation).
* ``nyq_frac`` - share of energy above 100 Hz, printed as a HEALTH CHECK on the
  measurement, not as a result. It sits near 50% for sampling reasons; if it
  moves a lot between runs, the state-read path changed and the comparison is
  not clean.

Verdict logic
-------------
    band_10_50 flat (x~1)      -> the box ignored the alpha change. Check that
                                  servo_alpha actually reached the wire.
    band_10_50 up, track down  -> LPF-off working as intended: the arm now
                                  follows the command instead of a smoothed
                                  version of it. Not jerk.
    band_10_50 up, track up,
    accel/jerk up              -> real jerk. Roll alpha back to 1.0.

Usage
-----
    tools/analyze_servo_jerk.py logs/servo_log_BASELINE.csv logs/servo_log_NEW.csv

Give the alpha=1.0 log FIRST; later logs are shown as ratios against it.
"""

from __future__ import annotations

import argparse
import csv
import numpy as np

ARMS = ("left", "right")
JOINTS = range(6)


def load(path: str) -> dict[str, np.ndarray]:
    """Read only the joint columns; servo logs run to hundreds of MB."""
    wanted = ["loop_start_time_ns"]
    for arm in ARMS:
        wanted += [f"{arm}_q_sent_{j}" for j in JOINTS]
        wanted += [f"{arm}_q_actual_{j}" for j in JOINTS]

    with open(path, newline="") as handle:
        reader = csv.reader(handle)
        header = next(reader)
        index = {name: i for i, name in enumerate(header)}
        missing = [n for n in wanted if n not in index]
        if missing:
            raise SystemExit(f"{path}: missing columns {missing}")
        cols = [index[n] for n in wanted]
        rows = []
        for row in reader:
            try:
                rows.append([float(row[c]) for c in cols])
            except (ValueError, IndexError):
                continue  # partially written final row
    if not rows:
        raise SystemExit(f"{path}: no complete rows")
    data = np.array(rows)
    return {name: data[:, i] for i, name in enumerate(wanted)}


def tick_dt(data: dict[str, np.ndarray]) -> float:
    """Median loop period in seconds. Median, not mean: one scheduling outlier
    would drag the mean and rescale every derivative below."""
    dt = np.diff(data["loop_start_time_ns"]) * 1e-9
    dt = dt[(dt > 0) & (dt < 0.05)]
    return float(np.median(dt)) if dt.size else 0.002


def lowpass(x: np.ndarray, dt: float, cutoff_hz: float, taps: int = 129) -> np.ndarray:
    """Zero-phase windowed-sinc low-pass, applied column-wise.

    Symmetric odd-length kernel convolved in 'same' mode, so the group delay is
    exactly zero — important because these signals are later cross-correlated
    against q_sent for the transport lag, and a filter delay would be read as
    robot latency. Edges are handled by reflection so the ramp in and out of the
    record does not appear as a transient the derivative would amplify.
    """
    nyq = 0.5 / dt
    fc = min(cutoff_hz, 0.95 * nyq) / nyq  # normalized to Nyquist
    n = np.arange(taps) - (taps - 1) / 2
    kernel = np.sinc(fc * n) * np.hamming(taps)
    kernel /= kernel.sum()
    pad = taps // 2
    out = np.empty_like(x)
    for j in range(x.shape[1]):
        padded = np.concatenate([x[pad:0:-1, j], x[:, j], x[-2:-pad - 2:-1, j]])
        out[:, j] = np.convolve(padded, kernel, mode="same")[pad:pad + x.shape[0]]
    return out


def lag_samples(sent: np.ndarray, actual: np.ndarray, max_lag: int) -> int:
    """Transport lag in samples, from velocity cross-correlation.

    Velocity rather than position so a constant joint offset cannot bias the
    peak — the same choice analyze_box_queue_lag.py makes, for the same reason.
    """
    a = np.diff(sent)
    b = np.diff(actual)
    a = a - a.mean()
    b = b - b.mean()
    if a.std() < 1e-12 or b.std() < 1e-12:
        return 0
    best, best_r = 0, -np.inf
    for lag in range(max_lag + 1):
        x = a[: len(a) - lag] if lag else a
        y = b[lag:]
        n = min(len(x), len(y))
        if n < 100:
            break
        r = float(np.corrcoef(x[:n], y[:n])[0, 1])
        if r > best_r:
            best_r, best = r, lag
    return best


def longest_run(mask: np.ndarray) -> slice:
    """Longest contiguous True run — the segment the FFT is safe to run on.

    Splicing the masked samples together instead would inject step
    discontinuities at every gap, and steps are broadband, so they would land
    as exactly the high-frequency energy this tool is trying to measure.
    """
    best_len = best_start = cur_start = 0
    for i, flag in enumerate(mask):
        if flag:
            if i == 0 or not mask[i - 1]:
                cur_start = i
            if i - cur_start + 1 > best_len:
                best_len, best_start = i - cur_start + 1, cur_start
    return slice(best_start, best_start + best_len)


def band_energy(vel: np.ndarray, dt: float, lo: float, hi: float) -> float:
    """Summed velocity energy in [lo, hi) Hz across joints, ABSOLUTE.

    Absolute rather than normalized: the total is dominated by a near-Nyquist
    sampling artifact (see module docstring), so any fraction would carry that
    artifact in its denominator.
    """
    n = vel.shape[0]
    if n < 512:
        return float("nan")
    window = np.hanning(n)
    freqs = np.fft.rfftfreq(n, dt)
    sel = (freqs >= lo) & (freqs < hi)
    total = 0.0
    for j in range(vel.shape[1]):
        v = vel[:, j] - vel[:, j].mean()
        total += float((np.abs(np.fft.rfft(v * window)) ** 2)[sel].sum())
    # Normalize by length so records of different duration compare directly.
    return total / n


def analyze(path: str, cutoff: float, floor: float, band: tuple[float, float]) -> dict:
    data = load(path)
    dt = tick_dt(data)
    out = {"dt": dt, "rate_hz": 1.0 / dt, "arms": {}}
    for arm in ARMS:
        sent = np.column_stack([data[f"{arm}_q_sent_{j}"] for j in JOINTS])
        actual_raw = np.column_stack([data[f"{arm}_q_actual_{j}"] for j in JOINTS])
        actual = lowpass(actual_raw, dt, cutoff)

        vel_raw = np.diff(actual_raw, axis=0) / dt   # for the nyq health check only
        vel = np.diff(actual, axis=0) / dt
        acc = np.diff(vel, axis=0) / dt
        jerk = np.diff(acc, axis=0) / dt

        mask = np.any(np.abs(vel) > floor, axis=1)
        if mask.sum() < 512:
            out["arms"][arm] = None
            continue
        seg = longest_run(mask)

        # Lag-align on the widest-moving joint, then residual on moving samples.
        pivot = int(np.argmax(np.std(vel[mask], axis=0)))
        lag = lag_samples(sent[:, pivot], actual_raw[:, pivot], max_lag=int(0.15 / dt))
        n = len(sent) - lag
        resid = actual_raw[lag:, :] - sent[:n, :]
        rmask = mask[: len(resid)]
        if len(rmask) < len(resid):
            rmask = np.pad(rmask, (0, len(resid) - len(rmask)), constant_values=False)

        nyq_total = band_energy(vel_raw[seg], dt, 0.0, 0.5 / dt)
        out["arms"][arm] = {
            "moving_frac": float(mask.mean()),
            "seg_sec": (seg.stop - seg.start) * dt,
            "lag_ms": lag * dt * 1e3,
            "band": band_energy(vel[seg], dt, band[0], band[1]),
            "accel_rms": float(np.sqrt(np.mean(acc[mask[: len(acc)]] ** 2))),
            "jerk_p99": float(np.percentile(np.abs(jerk[mask[: len(jerk)]]), 99)),
            "track_rms": float(np.sqrt(np.mean(resid[rmask] ** 2))),
            "nyq_frac": band_energy(vel_raw[seg], dt, 100.0, 0.5 / dt) / nyq_total
            if nyq_total > 0 else float("nan"),
        }
    return out


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--band", default="10,50",
                    help="headline band in Hz, 'lo,hi' (default 10,50)")
    ap.add_argument("--cutoff-hz", type=float, default=60.0,
                    help="zero-phase low-pass before differentiating (default 60)")
    ap.add_argument("--move-floor-deg-s", type=float, default=0.5,
                    help="a joint counts as moving above this speed (default 0.5)")
    args = ap.parse_args()

    lo, hi = (float(v) for v in args.band.split(","))
    label = f"band{int(lo)}-{int(hi)}"
    results = [(p, analyze(p, args.cutoff_hz, args.move_floor_deg_s, (lo, hi)))
               for p in args.logs]
    base = results[0][1]

    for path, res in results:
        print(f"\n=== {path} ===")
        print(f"  tick {res['rate_hz']:.1f} Hz | low-pass {args.cutoff_hz:.0f} Hz "
              f"| headline band {lo:.0f}-{hi:.0f} Hz")
        header = (f"  {'arm':<6} {'moving':>7} {'seg':>7} {'lag':>7} {label:>11} "
                  f"{'accel_rms':>11} {'jerk_p99':>11} {'track_rms':>10} {'nyq':>6}")
        print(header)
        print("  " + "-" * (len(header) - 2))
        for arm in ARMS:
            m = res["arms"][arm]
            if m is None:
                print(f"  {arm:<6}      (never moved)")
                continue
            print(f"  {arm:<6} {m['moving_frac']*100:6.1f}% {m['seg_sec']:6.1f}s "
                  f"{m['lag_ms']:6.0f}ms {m['band']:10.3e} {m['accel_rms']:9.1f}d/s2 "
                  f"{m['jerk_p99']:9.0f}d/s3 {m['track_rms']:8.4f}d {m['nyq_frac']*100:5.1f}%")
            if res is not base and base["arms"][arm]:
                b = base["arms"][arm]
                print(f"  {'':<6} vs base:  {label} x{m['band']/b['band']:.2f}"
                      f"   accel x{m['accel_rms']/b['accel_rms']:.2f}"
                      f"   jerk x{m['jerk_p99']/b['jerk_p99']:.2f}"
                      f"   track x{m['track_rms']/b['track_rms']:.2f}")

    if len(results) > 1:
        print(f"\n  {label} x~1.00        -> box ignored the alpha change; check the wire.")
        print(f"  {label} up, track DOWN -> LPF-off working (arm follows command). Not jerk.")
        print(f"  {label} up, track UP   -> real jerk. Roll servo_alpha back to 1.0.")
        print("  nyq should stay put between runs; if it moves, the state-read path")
        print("  changed and the comparison is not clean.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
