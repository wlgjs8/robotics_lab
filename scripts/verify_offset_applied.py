#!/usr/bin/env python3
"""Re-test TCP error AFTER applying the tool offset via umi-convert (option a).

Compares, against the true gripper-TCP path P_G = P_F·C (P_F = recorded tracker pose,
C = pure-translation Pika offset):
  - IGNORE (baseline): execute tracker-frame deltas on the TCP   (the ~cm error we measured)
  - OPTION (a): execute the CONVERTED (gripper-TCP) deltas on the TCP  (should be ~0)
Also reports conversion fidelity ||P_conv - P_G||. Pure numpy + h5py.
"""
from __future__ import annotations

import argparse
import glob
import math
import os

import h5py
import numpy as np

OFFSET = np.array([0.172, 0.0, -0.076])  # Pika tracker->gripper, tracker frame


def qR(q):
    q = np.asarray(q, float); n = np.linalg.norm(q)
    if n < 1e-12: return np.eye(3)
    x, y, z, w = q / n
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def T_of(p):
    T = np.eye(4); T[:3, :3] = qR(p[3:7]); T[:3, 3] = p[:3]; return T


def T_inv(T):
    Ti = np.eye(4); Ti[:3, :3] = T[:3, :3].T; Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]; return Ti


def chunk_err(P_policy, P_G, H):
    """Execute P_policy-frame deltas from the anchored true TCP; error vs P_G. Returns endpoint+max."""
    N = len(P_G)
    TF = [T_of(P_policy[t]) for t in range(N)]
    TG = [T_of(P_G[t]) for t in range(N)]
    endpoint, mx = [], []
    for s in range(N - 1):
        base = TG[s] @ T_inv(TF[s])
        jmax = min(H, N - 1 - s)
        if jmax < 1: continue
        errs = [np.linalg.norm((base @ TF[s + j])[:3, 3] - TG[s + j][:3, 3]) for j in range(1, jmax + 1)]
        mx.append(max(errs))
        if jmax == H: endpoint.append(errs[-1])
    return endpoint, mx


def pct(a, p): return float(np.percentile(a, p)) if len(a) else float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--orig-dir", default="data")
    ap.add_argument("--converted-dir", default="data_tcp")
    ap.add_argument("--horizon", type=int, default=16)
    args = ap.parse_args()
    H = args.horizon
    fid, ig_e, ig_m, a_e, a_m = [], [], [], [], []
    n = 0
    for ep in sorted(glob.glob(os.path.join(args.orig_dir, "**", "*.hdf5"), recursive=True)):
        conv = os.path.join(args.converted_dir, os.path.relpath(ep, args.orig_dir))
        if not os.path.exists(conv): continue
        with h5py.File(ep, "r") as o, h5py.File(conv, "r") as c:
            obs_o, obs_c = o["observations"], c["observations"]
            for arm in ("left", "right"):
                if f"{arm}/pose" not in obs_o: continue
                PF = np.asarray(obs_o[f"{arm}/pose"], float)
                ckey = f"tcp_stand_{arm}"
                if ckey not in obs_c: continue
                PC = np.asarray(obs_c[ckey], float)
                m = min(len(PF), len(PC))
                if m < 3: continue
                PF, PC = PF[:m], PC[:m]
                PG = np.array([[*(PF[t, :3] + qR(PF[t, 3:7]) @ OFFSET), *PF[t, 3:7]] for t in range(m)])
                fid.append(np.linalg.norm(PC[:, :3] - PG[:, :3], axis=1).max() * 1000)
                e, mm = chunk_err(PF, PG, H); ig_e += e; ig_m += mm          # IGNORE: tracker deltas
                e, mm = chunk_err(PC, PG, H); a_e += e; a_m += mm            # OPTION a: converted deltas
        n += 1
    fid = np.asarray(fid)
    print(f"episodes={n}  horizon H={H}  |offset|={np.linalg.norm(OFFSET)*1000:.0f}mm\n")
    print(f"conversion fidelity ||P_conv - P_G|| (mm):  mean={fid.mean():.5f}  max={fid.max():.5f}\n")
    ig_e = np.asarray(ig_e) * 1000.0; a_e = np.asarray(a_e) * 1000.0  # m -> mm
    print("in-chunk TCP endpoint error (mm), re-anchored (deployment scenario):")
    print(f"  IGNORE (tracker deltas on TCP):   mean={ig_e.mean():.2f}  p95={pct(ig_e,95):.2f}  max={ig_e.max():.2f}")
    print(f"  OPTION (a) (converted deltas):    mean={a_e.mean():.5f}  p95={pct(a_e,95):.5f}  max={a_e.max():.5f}")


if __name__ == "__main__":
    main()
