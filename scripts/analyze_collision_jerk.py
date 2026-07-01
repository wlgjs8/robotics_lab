#!/usr/bin/env python3
"""Attribute per-tick command JERK in a servo_log CSV to its likely cause.

Goal: answer "which path is causing the jerky motion?" — collision/floor
velocity-damper projection, stale-hold snap, hard-violation latch, IK branch
jump, or SMD reanchor — using ONLY columns the servo logger already emits
(no server code change required).

Signals per tick (both arms, L2 over the 6 joints, then max of L/R):
  jerk        = *_q_sent_jerk_deg_s3_*                 (the thing we minimize)
  proj_corr   = |q_target_before_output_ma - q_after_accel_limit|  / dt   (deg/s)
                => magnitude of ALL final safety projections combined
                   (floor + roi + reach + user_floor + self-collision).
  ma_smooth   = |q_target_after_output_ma - q_target_before_output_ma|
                => how much the output moving-average is smoothing (0 if window<=1)
  verdict     = safety_verdict (Ok / SelfCollision / FloorViolation / ...)
  cart_jump   = *_cart_sol_jump_deg                    (IK branch-jump jerk source)
  smd_reanchor delta                                   (SMD reanchor jerk source)
  fault edge  = fault_latched 0->1

For the top-jerk ticks it reports which signal co-occurs, so you can see whether
the jerk lives on the collision path or somewhere else entirely.

Usage:
  python3 scripts/analyze_collision_jerk.py logs/servo_log_YYYYMMDD_HHMMSS.csv
  python3 scripts/analyze_collision_jerk.py logs/servo_log.csv --top 40 --window 3
"""
import argparse
import sys

import numpy as np
import pandas as pd

JOINTS = range(6)


def arm_cols(prefix, arm):
    return [f"{arm}_{prefix}_{i}" for i in JOINTS]


def load(path):
    # Only pull the columns we need — the CSV has ~600 columns.
    base = ["tick", "period_ms", "safety_verdict", "motion_state",
            "fault_latched", "command_seq",
            "self_collision_min_clearance_m", "self_collision_pair"]
    per_arm = []
    for arm in ("left", "right"):
        per_arm += arm_cols("q_sent_jerk_deg_s3", arm)
        per_arm += arm_cols("q_target_before_output_ma", arm)
        per_arm += arm_cols("q_target_after_output_ma", arm)
        per_arm += arm_cols("q_after_accel_limit_deg", arm)
        per_arm += [f"{arm}_cart_sol_jump_deg", f"{arm}_smd_reanchor_count"]
    header = pd.read_csv(path, nrows=0).columns.tolist()
    want = [c for c in base + per_arm if c in header]
    missing = [c for c in base + per_arm if c not in header]
    if missing:
        print(f"[warn] {len(missing)} expected columns missing (older log?): "
              f"{missing[:6]}{'...' if len(missing) > 6 else ''}", file=sys.stderr)
    df = pd.read_csv(path, usecols=want, low_memory=False)
    return df


def l2(df, prefix, arm):
    cols = [c for c in arm_cols(prefix, arm) if c in df.columns]
    return np.sqrt((df[cols].astype(float) ** 2).sum(axis=1)) if cols else pd.Series(0.0, index=df.index)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--top", type=int, default=30, help="how many worst-jerk ticks to list")
    ap.add_argument("--pct", type=float, default=99.5, help="jerk percentile = 'spike' threshold")
    args = ap.parse_args()

    df = load(args.csv)
    n = len(df)
    dt = (df["period_ms"].astype(float) / 1000.0).clip(lower=1e-4) if "period_ms" in df else pd.Series(0.002, index=df.index)

    # --- Sent-command jerk magnitude (max over the two arms) ---
    jerk_l = l2(df, "q_sent_jerk_deg_s3", "left")
    jerk_r = l2(df, "q_sent_jerk_deg_s3", "right")
    jerk = np.maximum(jerk_l, jerk_r)

    # --- Combined safety-projection correction (deg/s) per arm ---
    def proj_corr(arm):
        b = [f"{arm}_q_target_before_output_ma_{i}" for i in JOINTS]
        a = [f"{arm}_q_after_accel_limit_deg_{i}" for i in JOINTS]
        if not all(c in df.columns for c in b + a):
            return pd.Series(0.0, index=df.index)
        d = df[b].astype(float).values - df[a].astype(float).values
        return pd.Series(np.linalg.norm(d, axis=1), index=df.index) / dt
    proj = np.maximum(proj_corr("left"), proj_corr("right"))

    # --- Output moving-average smoothing actually applied (deg) ---
    def ma_delta(arm):
        a = [f"{arm}_q_target_after_output_ma_{i}" for i in JOINTS]
        b = [f"{arm}_q_target_before_output_ma_{i}" for i in JOINTS]
        if not all(c in df.columns for c in a + b):
            return pd.Series(0.0, index=df.index)
        d = df[a].astype(float).values - df[b].astype(float).values
        return pd.Series(np.linalg.norm(d, axis=1), index=df.index)
    ma_sm = np.maximum(ma_delta("left"), ma_delta("right"))

    # --- Other known jerk sources ---
    cart_jump = pd.concat([df.get(f"{a}_cart_sol_jump_deg", pd.Series(0.0, index=df.index)).astype(float).abs()
                           for a in ("left", "right")], axis=1).max(axis=1)
    reanchor = pd.Series(0, index=df.index)
    for a in ("left", "right"):
        c = f"{a}_smd_reanchor_count"
        if c in df.columns:
            reanchor = reanchor + df[c].astype(float).diff().fillna(0).clip(lower=0)
    verdict = df.get("safety_verdict", pd.Series("", index=df.index)).astype(str)
    faulted = df.get("fault_latched", pd.Series(False, index=df.index)).astype(str).str.lower().isin(["1", "true"])
    fault_edge = faulted & ~faulted.shift(1, fill_value=False)

    # --- Timing-artifact guard ---------------------------------------------
    # The logged jerk is a 3rd finite difference; at a loop hitch (period != 2ms)
    # or a Hold->Running transition it explodes numerically even though q_sent is
    # perfectly smooth. Flag those ticks so they don't masquerade as real jerk.
    nominal_dt = dt.median()
    period_ms = df["period_ms"].astype(float) if "period_ms" in df else dt * 1000
    timing_bad = (period_ms - nominal_dt * 1000).abs() > 0.5   # >0.5ms off nominal
    timing_bad = timing_bad | timing_bad.shift(1, fill_value=False) | timing_bad.shift(2, fill_value=False)
    state = df.get("motion_state", pd.Series("", index=df.index)).astype(str)
    hold_to_run = (state == "Running") & (state.shift(1, fill_value="") != "Running")
    for k in range(1, 4):
        hold_to_run = hold_to_run | ((state == "Running") & (state.shift(k, fill_value="") != "Running"))
    artifact = timing_bad | hold_to_run
    loop_stall = period_ms > (nominal_dt * 1000 + 1.0)

    jerk_clean = jerk.copy()
    jerk_clean[artifact] = 0.0    # ignore artifact ticks when ranking REAL jerk

    thr = np.percentile(jerk_clean, args.pct)
    spike = (jerk_clean >= thr) & (~artifact)

    print(f"=== {args.csv} ===")
    print(f"rows={n}  running={int((df.get('motion_state','')=='Running').sum())}  "
          f"dt~{dt.median()*1000:.2f}ms")
    print(f"jerk(sent) RAW deg/s^3:  p99={np.percentile(jerk,99):.1f}  max={jerk.max():.1f}  "
          f"(incl. finite-diff artifacts)")
    print(f"timing artifacts excluded:  loop_stall(period>{nominal_dt*1000+1:.0f}ms)={int(loop_stall.sum())}  "
          f"hold->run+hitch guarded={int(artifact.sum())} ticks")
    print(f"jerk(sent) CLEAN deg/s^3:  median={np.median(jerk_clean):.1f}  p99={np.percentile(jerk_clean,99):.1f}  "
          f"p{args.pct}={thr:.1f}  max={jerk_clean.max():.1f}")
    print(f"proj-correction deg/s:  active(>0.5)={int((proj>0.5).sum())} ticks  "
          f"max={proj.max():.2f}  p99={np.percentile(proj,99):.2f}")
    print(f"output-MA smoothing:  active(>1e-4)={int((ma_sm>1e-4).sum())} ticks (0 => MA window<=1, OFF)")
    print(f"safety_verdict != Ok:  {int((verdict!='Ok').sum())} ticks  "
          f"[{', '.join(sorted(set(verdict[verdict!='Ok'])))[:80]}]")
    print(f"fault_latched edges:  {int(fault_edge.sum())}   SMD reanchors:  {int(reanchor.sum())}   "
          f"cart_sol_jump>2deg:  {int((cart_jump>2).sum())} ticks")

    # --- Attribution over spike ticks ---
    print(f"\n--- of {int(spike.sum())} spike ticks (jerk>=p{args.pct}), how many co-occur with: ---")
    def frac(mask):
        s = int((spike & mask).sum())
        return f"{s:5d}  ({100.0*s/max(1,int(spike.sum())):.0f}%)"
    print(f"  safety projection active (>0.5 deg/s) : {frac(proj>0.5)}")
    print(f"  verdict != Ok                          : {frac(verdict!='Ok')}")
    print(f"  cart_sol_jump > 2 deg (IK branch jump) : {frac(cart_jump>2)}")
    print(f"  SMD reanchor this tick                 : {frac(reanchor>0)}")
    print(f"  fault_latched edge                     : {frac(fault_edge)}")
    print(f"  loop stall (period hitch)              : {frac(loop_stall)}")
    print(f"  none of the above (unexplained)        : "
          f"{frac(~((proj>0.5)|(verdict!='Ok')|(cart_jump>2)|(reanchor>0)|fault_edge|loop_stall))}")

    # --- Correlation of jerk with each candidate driver ---
    print("\n--- Pearson corr(jerk, signal) over all ticks ---")
    for name, sig in [("proj_correction", proj), ("cart_sol_jump", cart_jump),
                      ("smd_reanchor", reanchor.astype(float)), ("ma_smoothing", ma_sm)]:
        s = sig.astype(float)
        c = np.corrcoef(jerk, s)[0, 1] if s.std() > 0 else float("nan")
        print(f"  {name:18s}: {c:+.3f}")

    # --- Worst-jerk tick table ---
    order = np.argsort(-jerk_clean.values)[:args.top]
    print(f"\n--- top {args.top} jerk ticks (artifacts excluded) ---")
    print(f"{'tick':>7} {'jerk':>9} {'proj/s':>8} {'verdict':>14} {'cartjmp':>8} "
          f"{'reanch':>6} {'clr_mm':>7} {'pair'}")
    clr = df.get("self_collision_min_clearance_m", pd.Series(np.nan, index=df.index)).astype(float)
    pair = df.get("self_collision_pair", pd.Series("", index=df.index)).astype(str)
    tick = df.get("tick", pd.Series(df.index, index=df.index))
    for i in order:
        print(f"{int(tick.iloc[i]):>7} {jerk.iloc[i]:>9.1f} {proj.iloc[i]:>8.2f} "
              f"{verdict.iloc[i]:>14} {cart_jump.iloc[i]:>8.2f} {int(reanchor.iloc[i]):>6} "
              f"{clr.iloc[i]*1000 if np.isfinite(clr.iloc[i]) else float('nan'):>7.1f} {pair.iloc[i][:40]}")


if __name__ == "__main__":
    main()
