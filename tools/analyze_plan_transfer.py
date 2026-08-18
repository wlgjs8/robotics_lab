#!/usr/bin/env python3
"""Plan-transfer + gripper readout from flow-infer step logs (outputs/sweep/*.jsonl).

Per arm, per run:
  net       1-s-window net displacement of the runner command target projected on the
            model plan (raw_delta_ee_local summed in world frame). 1.0 = the command
            realises the plan; <1 = the plan is being given back / truncated.
  meas      same projection for the MEASURED pose (what the robot actually did).
  jump      chunk-boundary re-anchor jump of the command target along the plan, mm
            (p50). rb_servo runs: ~-0.1 mm. CM + measured_blend: -1..-2 mm (p25 -3..-6).
  open%     fraction of ticks with grip cmd > 45 %  (open),  half% = 8..45 %.
  desc      number of descents (>=40 mm drop to a local z minimum in 1.5 s).
  appr>45   fraction of descents whose approach (1.5 s before bottom) opened past 45 %.
  close@    p50 height above the bottom (mm) when the gripper physically closed (<8 %).

Usage: analyze_plan_transfer.py outputs/sweep/2026*_P0_e4.jsonl [...]
Needs scipy (use the openpi venv python).
"""
from __future__ import annotations

import json
import sys
from collections import defaultdict

import numpy as np
from scipy.spatial.transform import Rotation as R


def load(path):
    rows = [json.loads(l) for l in open(path) if l.strip()]
    return [
        r for r in rows
        if not r.get("hold") and r.get("chunk_step_index") is not None
        and all(r["arms"][s].get("cmd_pose") and r["arms"][s].get("meas_pose")
                and r["arms"][s].get("raw_delta_ee_local") for s in ("left", "right"))
    ]


def transfer(rows, side):
    raw = np.array([r["arms"][side]["raw_delta_ee_local"][:3] for r in rows])
    q = [r["arms"][side]["cmd_pose"][3:7] for r in rows]
    raw_w = np.array([R.from_quat(q[i]).apply(raw[i]) for i in range(len(rows))])
    cmd = np.array([r["arms"][side]["cmd_pose"][:3] for r in rows])
    meas = np.array([r["arms"][side]["meas_pose"][:3] for r in rows])
    W = 30
    rc, rm = [], []
    for i in range(0, len(rows) - W, W):
        p = raw_w[i:i + W].sum(0)
        if np.linalg.norm(p) > 5e-3:
            rc.append(np.dot(cmd[i + W] - cmd[i], p) / np.dot(p, p))
            rm.append(np.dot(meas[i + W] - meas[i], p) / np.dot(p, p))
    chunks = defaultdict(list)
    for i, r in enumerate(rows):
        chunks[r["chunk_id"]].append(i)
    ids = sorted(chunks)
    J = []
    for a, b in zip(ids[:-1], ids[1:]):
        ia, ib = chunks[a], chunks[b]
        if len(ia) < 2 or len(ib) < 2:
            continue
        pe = np.array(rows[ia[-1]]["arms"][side]["cmd_pose"][:3])
        qq = R.from_quat(rows[ia[-1]]["arms"][side]["cmd_pose"][3:7])
        s1 = np.array(rows[ib[1]]["arms"][side]["cmd_pose"][:3])
        plan = qq.apply(np.array(rows[ib[0]]["arms"][side]["raw_delta_ee_local"][:3])
                        + np.array(rows[ib[1]]["arms"][side]["raw_delta_ee_local"][:3]))
        n = np.linalg.norm(plan)
        if n > 1e-3:
            J.append(np.dot((s1 - pe) - plan, plan) / n * 1e3)
    f = lambda v: float(np.median(v)) if len(v) else float("nan")
    return f(rc), f(rm), f(J)


def grip(rows, side):
    t = np.array([r["t_mono"] for r in rows])
    z = np.array([r["arms"][side]["meas_pose"][2] for r in rows])
    gc = np.array([r["arms"][side]["gripper_cmd_pct"] for r in rows], dtype=float)
    gm = np.array([r["arms"][side]["gripper_meas_pct"] if r["arms"][side]["gripper_meas_pct"] is not None
                   else np.nan for r in rows], dtype=float)
    open_frac = float((gc > 45).mean())
    half_frac = float(((gc >= 8) & (gc <= 45)).mean())
    w = 45
    appr, close_h = [], []
    for i in range(w, len(z) - 45):
        if z[i] == z[i - w:i + 15].min() and z[i - w:i].max() - z[i] >= 0.040:
            appr.append(gc[i - w:i].max() > 45)
            ks = [m for m in range(i - 45, i + 45) if gc[m] < 8 and gc[m - 1] >= 8]
            if ks and gc[i - 45] >= 8:
                k = ks[0]
                mk = next((m for m in range(k, min(k + 60, len(gm))) if np.isfinite(gm[m]) and gm[m] < 8), None)
                if mk is not None:
                    close_h.append((z[mk] - z[i]) * 1e3)
    return open_frac, half_frac, len(appr), (float(np.mean(appr)) if appr else float("nan")), \
        (float(np.median(close_h)) if close_h else float("nan"))


def main(paths):
    print(f"{'run':<24}{'arm':<6}{'ticks':>6} {'net':>5} {'meas':>5} {'jump':>6} | {'open%':>5} {'half%':>5} {'desc':>4} {'appr>45':>7} {'close@mm':>8}")
    for p in paths:
        rows = load(p)
        if len(rows) < 300:
            print(f"{p.split('/')[-1][:24]:<24} (too short: {len(rows)} ticks)")
            continue
        for side in ("left", "right"):
            net, meas, jump = transfer(rows, side)
            o, h, nd, ap, ch = grip(rows, side)
            print(f"{p.split('/')[-1][:24]:<24}{side:<6}{len(rows):>6} {net:5.2f} {meas:5.2f} {jump:+6.2f} | "
                  f"{o*100:5.0f} {h*100:5.0f} {nd:4d} {ap*100:7.0f} {ch:8.1f}")


if __name__ == "__main__":
    main(sys.argv[1:])
