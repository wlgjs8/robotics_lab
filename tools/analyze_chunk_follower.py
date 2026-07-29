#!/usr/bin/env python3
"""Offline analysis of the Ruckig chunk-follower from a servo CSV.

Produces the two views agreed for debugging "motion feels off vs training":

  1. Time-synced error plot (x = time, y = position / rotation error), per arm:
       - pf  vs tcp_command   : follower convergence error (how well the generated
                                setpoint reaches each chunk-step target)
       - pf  vs command_tcp   : producer-vs-chunk consistency (the raw per-tick
                                TcpPoseTarget flow-infer sent vs the chunk step)
       - pf  vs tcp_actual    : real tracking error (robot vs chunk step)
     Chunk boundaries (follower_seq changes) are marked; stall/corner ticks shaded.

  2. 3D trajectory plot: chunk-step targets (pf waypoints, colored per chunk seq),
     the follower's emitted setpoint path (tcp_command), and the measured path
     (tcp_actual).

Only rows where <side>_follower_active == 1 are analyzed (the SMD fallback rows
carry no pf). All poses are stand-frame; quaternion columns are used for the
rotation error: ang = 2*acos(|<q1,q2>|).

By default the measured tcp_actual traces are EXCLUDED (opt back in with
--with-actual), and the 3D trajectory opens as an INTERACTIVE window (rotate =
drag, zoom = scroll) in addition to the saved PNGs; use --no-show for headless.

Usage:
  python3 tools/analyze_chunk_follower.py [--csv logs/servo_log.csv]
      [--arm left|right|both] [--out-dir outputs/chunk_follower]
      [--t0-sec S --t1-sec S]   # optional time window (relative seconds)
      [--with-actual] [--no-show]
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from dataclasses import dataclass, field


@dataclass
class ArmSeries:
    t: list[float] = field(default_factory=list)
    pf: list[tuple[float, float, float]] = field(default_factory=list)
    pf_q: list[tuple[float, float, float, float]] = field(default_factory=list)
    cmd: list[tuple[float, float, float]] = field(default_factory=list)
    cmd_q: list[tuple[float, float, float, float]] = field(default_factory=list)
    raw: list[tuple[float, float, float]] = field(default_factory=list)
    raw_q: list[tuple[float, float, float, float]] = field(default_factory=list)
    act: list[tuple[float, float, float]] = field(default_factory=list)
    act_q: list[tuple[float, float, float, float]] = field(default_factory=list)
    seq: list[int] = field(default_factory=list)
    step: list[int] = field(default_factory=list)
    stall: list[bool] = field(default_factory=list)
    corner: list[bool] = field(default_factory=list)
    duration: list[float] = field(default_factory=list)
    alpha: list[float] = field(default_factory=list)
    # Wire policy_dt of the chunk frame driving this tick. T-slip is "the solver needed longer than
    # ONE ROW", so the reference is the row period on the wire, NOT a hardcoded 30 Hz -- a K=3
    # decimated checkpoint runs 100 ms rows and a 0.0334 reference reports 100% T-slip on a healthy run.
    policy_dt: list[float] = field(default_factory=list)


def fnum(row: dict, key: str, default: float = float("nan")) -> float:
    v = row.get(key, "")
    if v is None or v == "":
        return default
    try:
        return float(v)
    except ValueError:
        return default


def pose(row: dict, prefix: str) -> tuple[float, float, float] | None:
    x = fnum(row, f"{prefix}_x_m")
    if math.isnan(x):
        return None
    return (x, fnum(row, f"{prefix}_y_m"), fnum(row, f"{prefix}_z_m"))


def quat(row: dict, prefix: str) -> tuple[float, float, float, float] | None:
    qw = fnum(row, f"{prefix}_qw")
    if math.isnan(qw):
        return None
    return (fnum(row, f"{prefix}_qx"), fnum(row, f"{prefix}_qy"), fnum(row, f"{prefix}_qz"), qw)


def ang_err_rad(q1, q2) -> float:
    if q1 is None or q2 is None:
        return float("nan")
    dot = abs(sum(a * b for a, b in zip(q1, q2)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def dist(p1, p2) -> float:
    if p1 is None or p2 is None:
        return float("nan")
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(p1, p2)))


def load(csv_path: str, arms: list[str], t0: float | None, t1: float | None):
    series = {arm: ArmSeries() for arm in arms}
    base_ns = None
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                ns = float(row.get("loop_start_time_ns", "") or "nan")
            except ValueError:
                continue
            if math.isnan(ns):
                continue
            if base_ns is None:
                base_ns = ns
            t = (ns - base_ns) * 1e-9
            if t0 is not None and t < t0:
                continue
            if t1 is not None and t > t1:
                continue
            for arm in arms:
                if row.get(f"{arm}_follower_active", "0") not in ("1", "true", "True"):
                    continue
                s = series[arm]
                s.t.append(t)
                s.pf.append(pose(row, f"{arm}_follower_pf_stand"))
                s.pf_q.append(quat(row, f"{arm}_follower_pf_stand"))
                s.cmd.append(pose(row, f"{arm}_tcp_command_stand"))
                s.cmd_q.append(quat(row, f"{arm}_tcp_command_stand"))
                s.raw.append(pose(row, f"{arm}_command_tcp_target_stand"))
                s.raw_q.append(quat(row, f"{arm}_command_tcp_target_stand"))
                s.act.append(pose(row, f"{arm}_tcp_actual_stand"))
                s.act_q.append(quat(row, f"{arm}_tcp_actual_stand"))
                # recv_seq (receiver-local accepted-frame count) marks window
                # boundaries; fall back to the pre-rename follower_seq column
                # for CSVs recorded before the wire/recv seq split.
                s.seq.append(int(fnum(row, f"{arm}_follower_recv_seq",
                                      fnum(row, f"{arm}_follower_seq", 0))))
                s.step.append(int(fnum(row, f"{arm}_follower_step", -1)))
                s.stall.append(row.get(f"{arm}_follower_stall", "0") in ("1", "true", "True"))
                s.corner.append(row.get(f"{arm}_follower_corner", "0") in ("1", "true", "True"))
                s.duration.append(fnum(row, f"{arm}_follower_duration_sec"))
                s.policy_dt.append(fnum(row, "chunk_frame_policy_dt_sec"))
                s.alpha.append(fnum(row, f"{arm}_follower_alpha", 1.0))
    return series


def plot_errors(arm: str, s: ArmSeries, out_dir: str, with_actual: bool) -> str:
    import matplotlib.pyplot as plt

    pos_cmd = [dist(a, b) for a, b in zip(s.pf, s.cmd)]
    pos_raw = [dist(a, b) for a, b in zip(s.pf, s.raw)]
    ang_cmd = [math.degrees(ang_err_rad(a, b)) for a, b in zip(s.pf_q, s.cmd_q)]
    ang_raw = [math.degrees(ang_err_rad(a, b)) for a, b in zip(s.pf_q, s.raw_q)]

    fig, axes = plt.subplots(3, 1, figsize=(14, 11), sharex=True)
    ax_p, ax_a, ax_d = axes

    ax_p.plot(s.t, [v * 1000 for v in pos_cmd], label="pf vs cmd (follower convergence)", lw=0.8)
    ax_p.plot(s.t, [v * 1000 for v in pos_raw], label="pf vs raw command (producer consistency)", lw=0.8, alpha=0.8)
    if with_actual:
        pos_act = [dist(a, b) for a, b in zip(s.pf, s.act)]
        ax_p.plot(s.t, [v * 1000 for v in pos_act], label="pf vs actual (real tracking)", lw=0.8, alpha=0.8)
    ax_p.set_ylabel("position error [mm]")
    ax_p.legend(loc="upper right", fontsize=8)

    ax_a.plot(s.t, ang_cmd, label="pf vs cmd", lw=0.8)
    ax_a.plot(s.t, ang_raw, label="pf vs raw command", lw=0.8, alpha=0.8)
    if with_actual:
        ang_act = [math.degrees(ang_err_rad(a, b)) for a, b in zip(s.pf_q, s.act_q)]
        ax_a.plot(s.t, ang_act, label="pf vs actual", lw=0.8, alpha=0.8)
    ax_a.set_ylabel("rotation error [deg]")
    ax_a.legend(loc="upper right", fontsize=8)

    ax_d.plot(s.t, [d * 1000 for d in s.duration], label="segment T_opt [ms]", lw=0.8)
    ax_d.plot(s.t, [a * 33.3 for a in s.alpha], label="alpha x 33.3 (1.0 = no dilation)", lw=0.8, alpha=0.7)
    ax_d.axhline(33.3, color="gray", ls=":", lw=0.8, label="33.3 ms (converged)")
    ax_d.set_ylabel("T_opt [ms] / alpha")
    ax_d.set_xlabel("time [s]")
    ax_d.legend(loc="upper right", fontsize=8)

    # chunk boundaries + stall/corner shading on all axes
    for ax in axes:
        prev = None
        for i, q in enumerate(s.seq):
            if prev is not None and q != prev:
                ax.axvline(s.t[i], color="k", alpha=0.15, lw=0.6)
            prev = q
        for i, (st, co) in enumerate(zip(s.stall, s.corner)):
            if st:
                ax.axvspan(s.t[i] - 0.001, s.t[i] + 0.001, color="red", alpha=0.06, lw=0)
            elif co:
                ax.axvspan(s.t[i] - 0.001, s.t[i] + 0.001, color="orange", alpha=0.05, lw=0)

    stall_n = sum(s.stall)
    fig.suptitle(
        f"{arm} chunk-follower errors  (ticks={len(s.t)}, chunk-boundaries=gray, "
        f"stall-ticks={stall_n} red, corner=orange)"
    )
    out = os.path.join(out_dir, f"follower_errors_{arm}.png")
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out


def plot_3d(arm: str, s: ArmSeries, out_dir: str, with_actual: bool, show: bool) -> str:
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(11, 9))
    ax = fig.add_subplot(111, projection="3d")

    # chunk-step targets: one dot per unique (seq, step), colored per chunk seq
    seen: set[tuple[int, int]] = set()
    wx, wy, wz, wc = [], [], [], []
    for pf, q, k in zip(s.pf, s.seq, s.step):
        if pf is None or k < 0 or (q, k) in seen:
            continue
        seen.add((q, k))
        wx.append(pf[0]); wy.append(pf[1]); wz.append(pf[2]); wc.append(q)
    sc = ax.scatter(wx, wy, wz, c=wc, cmap="viridis", s=18, depthshade=False,
                    label="chunk-step targets (pf, color=chunk seq)")
    fig.colorbar(sc, ax=ax, shrink=0.6, label="chunk seq")

    cmd = [p for p in s.cmd if p is not None]
    if cmd:
        ax.plot([p[0] for p in cmd], [p[1] for p in cmd], [p[2] for p in cmd],
                lw=1.0, color="tab:blue", label="follower cmd path")
    pts = list(cmd) + list(zip(wx, wy, wz))
    if with_actual:
        act = [p for p in s.act if p is not None]
        if act:
            ax.plot([p[0] for p in act], [p[1] for p in act], [p[2] for p in act],
                    lw=1.0, color="tab:red", alpha=0.7, label="actual TCP path")
            pts += act

    # Equal aspect so interactive rotation doesn't distort the geometry.
    if pts:
        xs = [p[0] for p in pts]; ys = [p[1] for p in pts]; zs = [p[2] for p in pts]
        cx, cy, cz = (max(xs) + min(xs)) / 2, (max(ys) + min(ys)) / 2, (max(zs) + min(zs)) / 2
        r = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs), 1e-3) / 2
        ax.set_xlim(cx - r, cx + r); ax.set_ylim(cy - r, cy + r); ax.set_zlim(cz - r, cz + r)
        ax.set_box_aspect((1, 1, 1))

    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
    ax.legend(loc="upper left", fontsize=8)
    ax.set_title(f"{arm} chunk trajectory vs follower cmd" + (" vs actual" if with_actual else ""))
    out = os.path.join(out_dir, f"follower_traj3d_{arm}.png")
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    if not show:
        plt.close(fig)
    return out


def summarize(arm: str, s: ArmSeries) -> None:
    if not s.t:
        print(f"[{arm}] no follower-active rows in the selected window")
        return
    import statistics

    pos_cmd = [dist(a, b) for a, b in zip(s.pf, s.cmd) if a and b]
    pos_act = [dist(a, b) for a, b in zip(s.pf, s.act) if a and b]
    # Per-tick reference = that tick's wire row period; fall back to the median observed dt.
    _dts = [d for d in s.policy_dt if d == d and d > 1e-4]
    _fallback = statistics.median(_dts) if _dts else float("nan")
    _ref = [(p if (p == p and p > 1e-4) else _fallback) for p in s.policy_dt]
    over = [d for d, r in zip(s.duration, _ref) if r == r and d > r + 1e-4]
    _row_ms = f"{_fallback * 1000:.1f}ms" if _fallback == _fallback else "unknown"
    print(
        f"[{arm}] ticks={len(s.t)} span={s.t[-1] - s.t[0]:.1f}s chunks={len(set(s.seq))} "
        f"stall={sum(s.stall)} corner={sum(s.corner)}\n"
        f"      pf-vs-cmd  median={statistics.median(pos_cmd) * 1000:.2f}mm p95={sorted(pos_cmd)[int(0.95 * len(pos_cmd)) - 1] * 1000:.2f}mm\n"
        f"      pf-vs-act  median={statistics.median(pos_act) * 1000:.2f}mm p95={sorted(pos_act)[int(0.95 * len(pos_act)) - 1] * 1000:.2f}mm\n"
        f"      T-slip ticks={len(over)}/{len(s.duration)} "
        f"({100.0 * len(over) / max(1, len(s.duration)):.1f}%, vs wire row={_row_ms})  "
        f"alpha<1 ticks={sum(1 for a in s.alpha if a < 0.999)}"
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", default="logs/servo_log.csv")
    ap.add_argument("--arm", default="both", choices=["left", "right", "both"])
    ap.add_argument("--out-dir", default="outputs/chunk_follower")
    ap.add_argument("--t0-sec", type=float, default=None)
    ap.add_argument("--t1-sec", type=float, default=None)
    ap.add_argument("--with-actual", action="store_true",
                    help="include the measured tcp_actual traces (off by default)")
    ap.add_argument("--no-show", action="store_true",
                    help="headless: save PNGs only, no interactive 3D window")
    args = ap.parse_args()

    show = not args.no_show
    if not show:
        import matplotlib
        matplotlib.use("Agg")  # must precede any pyplot import

    arms = ["left", "right"] if args.arm == "both" else [args.arm]
    os.makedirs(args.out_dir, exist_ok=True)
    series = load(args.csv, arms, args.t0_sec, args.t1_sec)

    wrote = []
    for arm in arms:
        s = series[arm]
        summarize(arm, s)
        if not s.t:
            continue
        wrote.append(plot_errors(arm, s, args.out_dir, args.with_actual))
        wrote.append(plot_3d(arm, s, args.out_dir, args.with_actual, show))
    for path in wrote:
        print(f"wrote {path}")
    if wrote and show:
        import matplotlib.pyplot as plt
        print("[interactive] drag to rotate / scroll to zoom the 3D window(s); close to exit")
        plt.show()
    return 0 if wrote else 1


if __name__ == "__main__":
    sys.exit(main())
