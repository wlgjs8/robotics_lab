#!/usr/bin/env python3
"""Re-test TCP error AFTER applying the tool offset via umi-convert (option a).

Compares, against the true device-TCP path P_G = P_F·X (P_F = recorded tracker pose,
X = official pika_sdk tracker->tip transform composed with the tip->RB-TCP axis
alignment; see calibration/umi_retarget_eelocal.yaml):
  - IGNORE (baseline): execute tracker-frame deltas on the TCP   (the ~cm error we measured)
  - OPTION (a): execute the CONVERTED (device-TCP) deltas on the TCP  (should be ~0)
Also reports conversion fidelity ||P_conv - P_G||. Pure numpy + h5py.

CORRECTION (2026-06-12): the old pure-translation OFFSET (0.172, 0, -0.076) was defined
in the wrong frame (the translation lives in the R_corr-rotated gripper frame, not the
raw tracker frame). X now matches umi_retarget_eelocal.yaml:
  X = R_corr · Trans(0.172, 0, -0.076) · R_align
  R_corr  = Rx(-20°)·[Ry(-90°)·Rx(-90°)]      (pika_sdk vive_tracker.py, hardcoded)
  R_align = [0 0 1; -1 0 0; 0 -1 0]           (stack_real.yaml umi r_align)
"""
from __future__ import annotations

import argparse
import glob
import math
import os

import h5py
import numpy as np


def _rpy_R(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                     [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                     [-sp, cp*sr, cp*cr]])


_D = math.pi / 180.0
R_CORR = _rpy_R(-20.0 * _D, 0.0, 0.0) @ _rpy_R(-90.0 * _D, -90.0 * _D, 0.0)
R_ALIGN = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
T0 = np.array([0.172, 0.0, -0.076])  # tip translation, rotation-corrected gripper frame
X = np.eye(4)
X[:3, :3] = R_CORR @ R_ALIGN
X[:3, 3] = R_CORR @ T0  # raw tracker-frame lever arm [0, -0.0126, +0.1876] m


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


def R_to_q(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 0.5 / math.sqrt(tr + 1.0)
        return np.array([(R[2, 1] - R[1, 2]) * s, (R[0, 2] - R[2, 0]) * s, (R[1, 0] - R[0, 1]) * s, 0.25 / s])
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s])


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
                PG = np.array([
                    [*(TG := T_of(PF[t]) @ X)[:3, 3], *R_to_q(TG[:3, :3])] for t in range(m)
                ])
                fid.append(np.linalg.norm(PC[:, :3] - PG[:, :3], axis=1).max() * 1000)
                e, mm = chunk_err(PF, PG, H); ig_e += e; ig_m += mm          # IGNORE: tracker deltas
                e, mm = chunk_err(PC, PG, H); a_e += e; a_m += mm            # OPTION a: converted deltas
        n += 1
    fid = np.asarray(fid)
    print(f"episodes={n}  horizon H={H}  |lever|={np.linalg.norm(X[:3, 3])*1000:.0f}mm\n")
    print(f"conversion fidelity ||P_conv - P_G|| (mm):  mean={fid.mean():.5f}  max={fid.max():.5f}\n")
    ig_e = np.asarray(ig_e) * 1000.0; a_e = np.asarray(a_e) * 1000.0  # m -> mm
    print("in-chunk TCP endpoint error (mm), re-anchored (deployment scenario):")
    print(f"  IGNORE (tracker deltas on TCP):   mean={ig_e.mean():.2f}  p95={pct(ig_e,95):.2f}  max={ig_e.max():.2f}")
    print(f"  OPTION (a) (converted deltas):    mean={a_e.mean():.5f}  p95={pct(a_e,95):.5f}  max={a_e.max():.5f}")


if __name__ == "__main__":
    main()
