#!/usr/bin/env python3
"""analyze_follow_fidelity — model-intent vs realized-command fidelity of a
flow-infer rollout step log (outputs/sweep/*.jsonl).

Per arm:
  intent[t]   = |raw_delta_ee_local| mm      (the model's action this tick)
  realized[t] = |cmd_pose[t]-cmd_pose[t-1]| mm  (controller command stream,
                sampled at the runner's tick from tcp_command_stand)
  corr        = best Pearson corr of intent vs realized over lags 0..7 ticks
  bursts      = realized ticks > burst_mm (default 5 mm/tick = the follow
                vmax 150 mm/s * 33.4 ms budget)
  track_err   = mean |cmd_pose - meas_pose| mm

Usage: analyze_follow_fidelity.py <run.jsonl> [run2.jsonl ...] [--burst-mm 5]
"""
import argparse
import json
import math
import statistics as st
import sys


def pearson(x, y):
    n = min(len(x), len(y))
    x, y = x[:n], y[:n]
    mx, my = st.mean(x), st.mean(y)
    num = sum((a - mx) * (b - my) for a, b in zip(x, y))
    den = math.sqrt(sum((a - mx) ** 2 for a in x) * sum((b - my) ** 2 for b in y))
    return num / den if den else 0.0


def metrics(rows, side, burst_mm):
    intents, realized, errs = [], [], []
    prev = None
    for r in rows:
        a = r["arms"][side]
        d, c, m = a.get("raw_delta_ee_local"), a.get("cmd_pose"), a.get("meas_pose")
        intents.append(0.0 if not d else math.dist((0, 0, 0), d[:3]) * 1000)
        realized.append(math.dist(c[:3], prev[:3]) * 1000 if (c and prev) else 0.0)
        prev = c
        if c and m:
            errs.append(math.dist(c[:3], m[:3]) * 1000)
    best_lag, best = 0, -1.0
    for lag in range(0, 8):
        c = pearson(intents[: -lag or None] if lag else intents, realized[lag:])
        if c > best:
            best_lag, best = lag, c
    return dict(
        ticks=len(intents),
        intent_total_mm=round(sum(intents)),
        realized_total_mm=round(sum(realized)),
        transfer_ratio=round(sum(realized) / max(sum(intents), 1e-9), 2),
        corr=round(best, 3),
        lag_ticks=best_lag,
        bursts=sum(1 for v in realized if v > burst_mm),
        burst_pct=round(100 * sum(1 for v in realized if v > burst_mm) / max(len(realized), 1), 1),
        mean_intent_mm=round(st.mean(intents), 2),
        track_err_mm=round(st.mean(errs), 1) if errs else None,
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--burst-mm", type=float, default=5.0)
    args = ap.parse_args()
    for path in args.logs:
        rows = [json.loads(l) for l in open(path)]
        print(f"== {path}")
        for side in ("left", "right"):
            print(f"  {side:5s}", metrics(rows, side, args.burst_mm))
    return 0


if __name__ == "__main__":
    sys.exit(main())
