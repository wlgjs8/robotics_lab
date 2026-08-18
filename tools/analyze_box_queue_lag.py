#!/usr/bin/env python3
"""Measure command-to-robot latency drift in a servo_log, and the box queue fill behind it.

Context
-------
rb_servo_server streams servo_j at exactly ``servo.rate_hz`` off the host clock
while the rbpodo control box consumes at its own slightly slower rate. The
difference accumulates in the box's command queue, so the plan-to-robot delay
GROWS with session uptime instead of sitting at a constant value.

Measured on 2026-08-18 (``logs/servo_log_20260818_104415.csv``), before any fix::

    t =  6- 12 s   left  66 ms   right  54 ms
    t = 48- 54 s   left 126 ms   right 114 ms
    t = 66- 72 s   left 152 ms   right 138 ms      <- ChunkFollowerFault latched here

That is +1.3-1.4 ms/s. It matters because ``preview_max_actual_lead_rad`` (4 deg)
and ``preview_max_actual_lead_m`` (35 mm) in ``stack_real.yaml`` are pure
speed x delay budgets sized for the ~65 ms a FRESH session shows -- so the same
rollout trips or does not trip depending only on how long the stack has been up.

Two independent readings
------------------------
* ``lag``  - sliding-window normalized cross-correlation of joint VELOCITY
  between ``*_q_sent_*`` and ``*_q_actual_*``. Velocity (not position) so a
  constant joint offset cannot bias the peak. This is the observable that
  actually hurts.
* ``fill`` - ``*_box_queue_fill``, the box's own ``RBACK[<n>]`` queue occupancy.
  Present only in logs written after the RBACK observer landed. One fill tick is
  one control period, so ``fill x period`` should track ``lag`` up to the fixed
  servo_t2 lookahead and the arm's mechanical response.

Usage::

    tools/analyze_box_queue_lag.py logs/servo_log.csv
    tools/analyze_box_queue_lag.py logs/before.csv logs/after.csv   # compare runs
"""

from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass

import numpy as np

# servo_log ticks are one control period apart; 500 Hz is the only supported rate
# (stack_real.yaml servo.rate_hz), and servo_t1_sec: 0.002 is pinned to it.
# Nominal 500 Hz. Only a fallback: measure() below derives the real period from the
# log, because stack_real.yaml's servo.rate_hz is not a constant (it was moved to
# 250 Hz on 2026-08-18) and every number this tool prints -- lag, window edges, the
# inter-arm gap in ms -- is a tick count multiplied by it. Reading 500 Hz off a
# 250 Hz log halves every latency it reports.
TICK_MS = 2.0
MAX_LAG_TICKS = 150  # 300 ms -- well past any latency we expect to see
VELOCITY_WINDOW_TICKS = 10  # 20 ms finite-difference window


@dataclass
class WindowResult:
    t_start_s: float
    t_end_s: float
    lag_ms: dict[str, float | None]
    corr: dict[str, float | None]
    fill: dict[str, float | None]


def load(path: str) -> dict[str, np.ndarray]:
    """Read only the columns we need; servo logs run to hundreds of MB."""
    wanted = [
        "tick",
        "box_queue_trim_us",
        "box_queue_integral_us",
        "box_queue_controlled_fill",
        "left_send_skip_count",
        "right_send_skip_count",
        "loop_start_time_ns",
    ]
    for arm in ("left", "right"):
        wanted += [f"{arm}_q_sent_{j}" for j in range(6)]
        wanted += [f"{arm}_q_actual_{j}" for j in range(6)]
        wanted += [f"{arm}_box_queue_fill", f"{arm}_box_queue_fill_unparsed"]

    with open(path, newline="") as handle:
        reader = csv.reader(handle)
        header = next(reader)
        index = {name: i for i, name in enumerate(header)}
        present = [name for name in wanted if name in index]
        missing = [name for name in wanted if name not in index]
        cols = [index[name] for name in present]

        rows = []
        for row in reader:
            try:
                rows.append([float(row[c]) for c in cols])
            except (ValueError, IndexError):
                # Partially written final row, or a column the writer left blank
                # while a subsystem was inactive. Skipping is correct: these
                # estimators need complete samples, not imputed ones.
                continue

    if not rows:
        raise SystemExit(f"{path}: no complete rows")
    data = np.array(rows)
    out = {name: data[:, i] for i, name in enumerate(present)}
    for name in missing:
        out[name] = None  # type: ignore[assignment]
    return out


def velocity_lag(q_sent: np.ndarray, q_actual: np.ndarray) -> tuple[float | None, float | None]:
    """Peak-correlation lag in ms between commanded and measured joint velocity."""
    w = VELOCITY_WINDOW_TICKS
    if len(q_sent) < w + MAX_LAG_TICKS + 10:
        return None, None
    v_sent = q_sent[w:] - q_sent[:-w]
    v_actual = q_actual[w:] - q_actual[:-w]
    # A window where nothing moved carries no phase information at all; reporting
    # a "best lag" from noise would be worse than reporting nothing.
    if np.std(v_sent) < 1e-7 or np.std(v_actual) < 1e-7:
        return None, None
    v_sent = v_sent - v_sent.mean(axis=0)
    v_actual = v_actual - v_actual.mean(axis=0)

    best_corr, best_lag = -2.0, 0
    for lag in range(MAX_LAG_TICKS + 1):
        n = len(v_actual) - lag
        num = float(np.sum(v_sent[:n] * v_actual[lag : lag + n]))
        den = float(
            np.sqrt(np.sum(v_sent[:n] ** 2) * np.sum(v_actual[lag : lag + n] ** 2))
        )
        if den <= 0.0:
            continue
        corr = num / den
        if corr > best_corr:
            best_corr, best_lag = corr, lag
    return best_lag * TICK_MS, best_corr


def analyze(path: str, window_s: float) -> list[WindowResult]:
    data = load(path)
    tick = data["tick"]
    n = len(tick)
    window = int(window_s * 1000.0 / TICK_MS)

    results: list[WindowResult] = []
    for start in range(0, n - window + 1, window):
        end = start + window
        lag: dict[str, float | None] = {}
        corr: dict[str, float | None] = {}
        fill: dict[str, float | None] = {}
        for arm in ("left", "right"):
            q_sent = np.column_stack([data[f"{arm}_q_sent_{j}"][start:end] for j in range(6)])
            q_actual = np.column_stack([data[f"{arm}_q_actual_{j}"][start:end] for j in range(6)])
            lag[arm], corr[arm] = velocity_lag(q_sent, q_actual)

            fill_col = data.get(f"{arm}_box_queue_fill")
            if fill_col is None:
                fill[arm] = None
            else:
                # -1 is the "not observed this cycle" sentinel, not an occupancy.
                observed = fill_col[start:end]
                observed = observed[observed >= 0]
                fill[arm] = float(observed.mean()) if len(observed) else None
        results.append(
            WindowResult(
                t_start_s=(tick[start] - tick[0]) * TICK_MS / 1000.0,
                t_end_s=(tick[end - 1] - tick[0]) * TICK_MS / 1000.0,
                lag_ms=lag,
                corr=corr,
                fill=fill,
            )
        )
    return results


# A cross-correlation peak is only a lag if the correlation is actually high.
# Startup windows -- where the queue is still draining a 40-tick backlog and the
# arm may not be moving yet -- produce a peak at some arbitrary offset with r near
# zero. Regressing through those is what made the 18:38 run report
# "left DRIFTING -0.71 ms/s" when the lag was flat at 34 ms from t=6 s onward: one
# junk window at 182 ms with r=-0.00 dragged the whole fit. Excluding it gives
# -0.013 ms/s [stationary], which is the true answer.
MIN_CORR_FOR_SLOPE = 0.5


def slope_per_s(results: list[WindowResult], arm: str) -> float | None:
    """Least-squares ms-of-lag per second of uptime. This is the defect metric."""
    pts = [
        (r.t_start_s, r.lag_ms[arm])
        for r in results
        if r.lag_ms[arm] is not None
        and r.corr[arm] is not None
        and r.corr[arm] >= MIN_CORR_FOR_SLOPE
    ]
    if len(pts) < 3:
        return None
    xs = np.array([p[0] for p in pts])
    ys = np.array([p[1] for p in pts])
    return float(np.polyfit(xs, ys, 1)[0])


def tick_ms_from_log(data: dict) -> float:
    """Median loop period in ms, straight from the log timestamps."""
    stamps = data.get("loop_start_time_ns")
    if stamps is None or len(stamps) < 100:
        return TICK_MS
    d = np.diff(stamps) * 1e-6
    d = d[(d > 0) & (d < 50.0)]
    return float(np.median(d)) if d.size else TICK_MS


def report(path: str, window_s: float) -> None:
    global TICK_MS
    TICK_MS = tick_ms_from_log(load(path))
    results = analyze(path, window_s)
    data = load(path)

    print(f"\n=== {path} ===")
    print(f"  tick {TICK_MS:.3f} ms ({1000.0 / TICK_MS:.1f} Hz measured from the log)")
    have_fill = data.get("left_box_queue_fill") is not None
    if not have_fill:
        print("  (no *_box_queue_fill columns -- log predates the RBACK observer)")
    else:
        for arm in ("left", "right"):
            col = data[f"{arm}_box_queue_fill"]
            unparsed = data.get(f"{arm}_box_queue_fill_unparsed")
            observed = col[col >= 0]
            pct = 100.0 * len(observed) / len(col) if len(col) else 0.0
            if len(observed) == 0:
                print(
                    f"  {arm}: box_queue_fill NEVER observed "
                    f"(no RBACK[n] parsed in {len(col)} ticks)"
                )
            else:
                print(
                    f"  {arm}: fill observed on {pct:5.1f}% of ticks, "
                    f"min={observed.min():.0f} p50={np.percentile(observed, 50):.0f} "
                    f"max={observed.max():.0f}"
                    + (
                        f", unparsed responses={int(unparsed.sum())}"
                        if unparsed is not None
                        else ""
                    )
                )

    trim = data.get("box_queue_trim_us")
    if trim is None:
        print("  cadence controller: not present in this log")
    else:
        integral = data.get("box_queue_integral_us")
        active = np.count_nonzero(trim != 0.0)
        if active == 0:
            print("  cadence controller: present but never trimmed (disabled or warming up)")
        else:
            print(
                f"  cadence controller: trimming on {100.0 * active / len(trim):5.1f}% of ticks, "
                f"trim p50={np.percentile(trim, 50):+.1f}us "
                f"p05={np.percentile(trim, 5):+.1f} p95={np.percentile(trim, 95):+.1f}"
                + (
                    f", integral settled at {integral[-1]:+.2f}us/cycle"
                    if integral is not None
                    else ""
                )
            )

    header = f"  {'window':>16} | {'left lag':>9} {'r':>5} {'fill':>6} | {'right lag':>9} {'r':>5} {'fill':>6}"
    print(header)
    print("  " + "-" * (len(header) - 2))
    for r in results:
        cells = []
        for arm in ("left", "right"):
            lag = r.lag_ms[arm]
            corr = r.corr[arm]
            fill = r.fill[arm]
            cells.append(
                f"{lag:7.0f}ms" if lag is not None else "      n/a"
            )
            cells.append(f"{corr:5.2f}" if corr is not None else "  n/a")
            cells.append(f"{fill:6.1f}" if fill is not None else "   n/a")
        print(
            f"  {r.t_start_s:6.1f}-{r.t_end_s:6.1f}s | "
            f"{cells[0]} {cells[1]} {cells[2]} | {cells[3]} {cells[4]} {cells[5]}"
        )

    # Per-arm level trim (servo.box_queue_cadence.level). The shared period trim is
    # a rate lever and holds only one arm; this is the check that the OTHER arm was
    # levelled too, which is the difference between "10 ms of queue" and "10 ms on
    # whichever arm happened to warm up shallower".
    if have_fill:
        left_fill = data["left_box_queue_fill"]
        right_fill = data["right_box_queue_fill"]
        both = (left_fill >= 0) & (right_fill >= 0)
        if both.sum() > 100:
            gap = np.abs(left_fill[both] - right_fill[both])
            print()
            print(
                f"  inter-arm fill gap: p50={np.percentile(gap, 50):.1f} "
                f"p99={np.percentile(gap, 99):.1f} ticks "
                f"= {np.percentile(gap, 50) * TICK_MS:.0f} ms of latency the deeper "
                f"arm carries for nothing"
            )
        skips = []
        for arm in ("left", "right"):
            col = data.get(f"{arm}_send_skip_count")
            if col is not None and len(col):
                skips.append(f"{arm}={int(col.max())}")
        if skips:
            # Expect a burst at entry (the 40-50 tick startup backlog) then roughly
            # one skip every two minutes. A count that keeps climbing means the
            # speed gate or skip_margin_ticks is wrong, not that the arms drift.
            print(f"  level-trim send skips: {', '.join(skips)}")

    print()
    for arm in ("left", "right"):
        slope = slope_per_s(results, arm)
        if slope is None:
            print(f"  {arm}: not enough moving windows to fit a drift slope")
            continue
        verdict = "DRIFTING" if abs(slope) > 0.2 else "stationary"
        print(f"  {arm}: lag drift = {slope:+.2f} ms per second of uptime  [{verdict}]")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("logs", nargs="+", help="servo_log_*.csv path(s)")
    parser.add_argument(
        "--window-sec",
        type=float,
        default=6.0,
        help="sliding window length in seconds (default: 6)",
    )
    args = parser.parse_args()
    for path in args.logs:
        report(path, args.window_sec)
    return 0


if __name__ == "__main__":
    sys.exit(main())
