#!/usr/bin/env python3
"""Quantify the TCP position error from IGNORING the tracker->gripper tool offset
when executing tracker-frame EE-relative deltas on the gripper TCP.

Model (option c, "ignore offset"):
  - recorded tracker poses P_F(t) (steamvr_world); gripper TCP pose P_G(t) = P_F(t)·C,
    with C a pure translation d = tracker->gripper-TCP offset (Pika SDK constant).
  - at deployment the policy emits tracker-frame body-relative deltas a_F[k] = P_F(k)^-1 P_F(k+1)
    and the robot applies them to its TCP frame, RE-ANCHORED to the measured TCP at each chunk
    start (closed-loop: chunk horizon H, then re-predict). Within a chunk starting at s:
        P_exec(s+j) = P_G(s) · P_F(s)^-1 · P_F(s+j)
    while the intended (demo) TCP path is P_G(s+j). Error_j = ||pos(P_exec) - pos(P_G)||.

Reports the distribution of in-chunk TCP error (the deployment-relevant number), plus per-step
rotation magnitude and a pessimistic full-episode open-loop bound (no re-anchoring).
Pure numpy + h5py; no torch/GPU.
"""
from __future__ import annotations

import argparse
import glob
import math
import os

import h5py
import numpy as np

DEFAULT_OFFSET = (0.172, 0.0, -0.076)  # Pika GRIPPER_OFFSET, tracker-frame (m)


def quat_to_R(q):
    q = np.asarray(q, float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = q / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def T_of(pose):
    T = np.eye(4)
    T[:3, :3] = quat_to_R(pose[3:7])
    T[:3, 3] = pose[:3]
    return T


def T_inv(T):
    R = T[:3, :3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ T[:3, 3]
    return Ti


def rot_angle_deg(R):
    c = (np.trace(R) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def iter_pose_tracks(path):
    """Yield (arm_name, pose[N,7]) for bimanual or single pika episodes."""
    with h5py.File(path, "r") as h:
        obs = h["observations"]
        if "pose" in obs:  # single-arm flat
            yield "single", np.asarray(obs["pose"], float)
            return
        for arm in obs:  # bimanual: observations/<arm>/pose
            g = obs[arm]
            if isinstance(g, h5py.Group) and "pose" in g:
                yield arm, np.asarray(g["pose"], float)


def analyze(paths, d, H):
    C = np.eye(4)
    C[:3, 3] = np.asarray(d, float)
    chunk_endpoint = []   # error at j=H within a re-anchored chunk
    chunk_max = []        # max error over j in [1,H]
    perstep_rot = []      # per-step rotation magnitude (deg)
    open_loop_max = []    # full-episode open-loop max error (no re-anchor)
    n_tracks = 0
    for path in paths:
        for _arm, P in iter_pose_tracks(path):
            N = len(P)
            if N < 3:
                continue
            n_tracks += 1
            TF = [T_of(P[t]) for t in range(N)]
            TG = [TF[t] @ C for t in range(N)]
            for t in range(N - 1):
                perstep_rot.append(rot_angle_deg((T_inv(TF[t]) @ TF[t + 1])[:3, :3]))
            # re-anchored chunks
            for s in range(0, N - 1):
                base = TG[s] @ T_inv(TF[s])
                jmax = min(H, N - 1 - s)
                if jmax < 1:
                    continue
                errs = []
                for j in range(1, jmax + 1):
                    pe = (base @ TF[s + j])[:3, 3]
                    pg = TG[s + j][:3, 3]
                    errs.append(np.linalg.norm(pe - pg))
                chunk_max.append(max(errs))
                if jmax == H:
                    chunk_endpoint.append(errs[-1])
            # full open-loop
            base0 = TG[0] @ T_inv(TF[0])
            ol = [np.linalg.norm((base0 @ TF[k])[:3, 3] - TG[k][:3, 3]) for k in range(N)]
            open_loop_max.append(max(ol))
    return {
        "n_episodes": len(paths), "n_tracks": n_tracks,
        "offset_m": float(np.linalg.norm(d)),
        "perstep_rot_deg": np.asarray(perstep_rot),
        "chunk_endpoint_mm": np.asarray(chunk_endpoint) * 1000.0,
        "chunk_max_mm": np.asarray(chunk_max) * 1000.0,
        "open_loop_max_mm": np.asarray(open_loop_max) * 1000.0,
    }


def pct(a, p):
    return float(np.percentile(a, p)) if a.size else float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", default="data")
    ap.add_argument("--horizon", type=int, default=16)
    ap.add_argument("--offset", default=",".join(str(v) for v in DEFAULT_OFFSET))
    args = ap.parse_args()
    d = [float(v) for v in args.offset.split(",")]
    paths = sorted(glob.glob(os.path.join(args.data_dir, "**", "*.hdf5"), recursive=True))
    if not paths:
        raise SystemExit(f"no hdf5 under {args.data_dir}")
    r = analyze(paths, d, args.horizon)
    print(f"episodes={r['n_episodes']} pose_tracks={r['n_tracks']} "
          f"|offset|={r['offset_m']*1000:.1f}mm  horizon H={args.horizon}")
    print()
    rot = r["perstep_rot_deg"]
    print(f"per-step rotation (deg):   mean={rot.mean():.3f}  p50={pct(rot,50):.3f}  "
          f"p95={pct(rot,95):.3f}  max={rot.max():.3f}")
    print()
    print("TCP error from IGNORING tool offset (mm):")
    for name, a in (("in-chunk endpoint (j=H, RE-ANCHORED ⇐ deployment)", r["chunk_endpoint_mm"]),
                    ("in-chunk max over j (RE-ANCHORED)", r["chunk_max_mm"]),
                    ("full-episode open-loop max (pessimistic, no re-anchor)", r["open_loop_max_mm"])):
        print(f"  {name}:")
        print(f"      mean={a.mean():.2f}  p50={pct(a,50):.2f}  p95={pct(a,95):.2f}  "
              f"p99={pct(a,99):.2f}  max={a.max():.2f}")


if __name__ == "__main__":
    main()
