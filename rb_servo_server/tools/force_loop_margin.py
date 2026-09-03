#!/usr/bin/env python3
"""Gain margin of the admittance loop against a rigid contact, per damping row.

The model is controller-manager's (wiki/findings/follow-contact-ring-is-delay-not-
the-gate.md), reproduced in docs/reference/force_control_stability_margin.md:

    L(s) = k_env * 1/(m s^2 + b s + k) * 1/(1 + s/w_f) * exp(-T_d s)

Solve angle(L(jw)) = -180 deg for w by bisection on the UNWRAPPED phase (np.angle
wraps and the root-find silently misses), then read |L| there. GM = -20 log10 |L|.

    python3 tools/force_loop_margin.py                # translation table, k = 0
    python3 tools/force_loop_margin.py --rotation     # rotation on the pika lever
    python3 tools/force_loop_margin.py --k 400 --rows 15:434.7   # the old spring row

Every number in stack_real.yaml's force_control comment tables came out of this file.
"""
import argparse
import math

import numpy as np


def gain_margin(m, b, k, td, f_filter, k_env):
    """Return (ring_hz, gm_db) or None if the phase never reaches -180 deg."""
    wf = 2.0 * math.pi * f_filter if f_filter > 0.0 else None

    def phase(w):
        p = -math.atan2(w * b, k - m * w * w) - w * td
        if wf is not None:
            p -= math.atan2(w, wf)
        return p + math.pi

    ws = np.logspace(-2, 3.5, 8000)
    ph = np.array([phase(w) for w in ws])
    idx = np.where(np.diff(np.sign(ph)) != 0)[0]
    if len(idx) == 0:
        return None
    lo, hi = float(ws[idx[0]]), float(ws[idx[0] + 1])
    for _ in range(100):
        mid = 0.5 * (lo + hi)
        if phase(lo) * phase(mid) <= 0.0:
            hi = mid
        else:
            lo = mid
    w = 0.5 * (lo + hi)
    g = k_env / abs(k - m * w * w + 1j * w * b)
    if wf is not None:
        g /= abs(1.0 + 1j * w / wf)
    return w / (2.0 * math.pi), -20.0 * math.log10(g)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rotation", action="store_true",
                    help="rotation channel: k_env becomes k_env * L^2 on the pika lever")
    ap.add_argument("--lever-m", type=float, default=0.202642, help="SRO->TCP lever [m]")
    ap.add_argument("--k", type=float, default=0.0, help="stiffness [N/m or Nm/rad]")
    ap.add_argument("--filter-hz", type=float, default=25.0, help="wrench_filter_hz (0 = off)")
    ap.add_argument("--delays-ms", default="12,18,26", help="transport delays to tabulate")
    ap.add_argument("--kenv", default="30663,44400", help="contact stiffnesses [N/m]")
    ap.add_argument("--rows", default=None,
                    help="m:b rows, comma separated (default: the shipped candidates)")
    args = ap.parse_args()

    if args.rows:
        rows = [tuple(float(x) for x in r.split(":")) for r in args.rows.split(",")]
    elif args.rotation:
        rows = [(1.0, 15.72), (0.3, 10.0), (0.3, 20.0), (0.3, 30.0), (0.3, 40.0), (0.5, 30.0)]
    else:
        rows = [(15.0, 434.7), (12.0, 500.0), (12.0, 700.0), (12.0, 850.0), (12.0, 1000.0),
                (12.0, 1200.0), (12.0, 1500.0)]
    delays = [float(x) * 1e-3 for x in args.delays_ms.split(",")]
    kenvs = [float(x) for x in args.kenv.split(",")]
    scale = args.lever_m ** 2 if args.rotation else 1.0

    print(f"{'rotation' if args.rotation else 'translation'} channel, k = {args.k}, "
          f"filter {args.filter_hz} Hz; cells: ring Hz then GM [dB] per k_env "
          f"{'/'.join(str(int(k)) for k in kenvs)}")
    head = f"{'m':>6} {'b':>8} |"
    for td in delays:
        head += f"  T_d {td*1e3:4.0f} ms: f_ring " + " ".join(f"GM{int(k/1000)}k" for k in kenvs) + " |"
    print(head)
    for m, b in rows:
        line = f"{m:>6} {b:>8} |"
        for td in delays:
            cells = []
            f_ring = None
            for k_env in kenvs:
                r = gain_margin(m, b, args.k, td, args.filter_hz, k_env * scale)
                if r is None:
                    cells.append("  n/a")
                    continue
                f_ring = r[0]
                cells.append(f"{r[1]:+6.1f}")
            line += f"   {f_ring if f_ring is not None else float('nan'):11.2f}  " + " ".join(cells) + " |"
        print(line)
    if args.rotation:
        print("\nhand cost M = b*w at 0.2 rad/s (~11 deg/s): " +
              ", ".join(f"b {b:g} -> {b*0.2:.1f} Nm" for _, b in rows))
    else:
        print("\nhand cost F = b*v at 30 mm/s: " +
              ", ".join(f"b {b:g} -> {b*0.03:.0f} N" for _, b in rows))


if __name__ == "__main__":
    main()
