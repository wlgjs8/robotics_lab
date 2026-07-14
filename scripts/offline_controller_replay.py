#!/usr/bin/env python3
"""Offline replay of the flow-infer ee_local delta-twist controller against
recorded episodes — no robot, no VM, no model.

It asks a single question: *can the delta-twist controller reproduce the
input/output of the episodes the policy was trained on?* It answers by pushing
each recorded absolute-pose trajectory through the exact controller math the
repo ships:

  training encode : pose_delta_local(pose[t], pose[t+1])          (consecutive ee_local twists)
  controller path : clamp (velocity*dt) -> chunk crossfade -> pose_compose_local
                    re-anchored to the "measured" pose at every chunk boundary

The SE(3) primitives, the per-axis clamp, and the crossfade ramp are imported
from / mirrored verbatim from policy_runner (see refs in the code) so the replay
tracks the deployed controller, not a re-derivation.

FOH SE(3) conditioning is intentionally not simulated: a first-order hold is
knot-exact at the 30 Hz recorded timestamps, so it adds no error at the points
we can compare against ground truth (it only defines the 500 Hz in-between
samples, which have no recorded reference).

Anchor modes bound the real closed-loop behaviour, which needs the robot:
  actual  : re-anchor each chunk boundary to the recorded pose  (optimistic;
            isolates within-chunk tracking gap, matches chunk_anchor_source=actual
            with a perfectly-tracking robot)
  chain   : never re-anchor, integrate through the clamps        (pessimistic;
            full compounded undershoot, matches chunk_anchor_source=chain)
The physical robot (re-anchor to actual, lagged FK) lies between the two.

Data source: either a directory of pose npz files (--npz-dir; keys
{left,right}_pose = (T,7) x,y,z,qx,qy,qz,qw) or raw HDF5 episodes (--hdf5-dir;
reads observations/{arm}/pose, the pika UMI bimanual layout).
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "policy_runner"))

from policy_runner.flow_dataset import pose_delta_local, pose_compose_local  # noqa: E402
from policy_runner.action_sources.tcp_pose_target import clamp_pose_delta  # noqa: E402


# --- production defaults (tools/flow_infer_real_policy.sh omits these flags) ---
DEFAULT_POLICY_DT = 0.0334      # main.py --policy-dt-sec default
DEFAULT_MAX_LIN_VEL = 0.15      # main.py --max-linear-velocity-m-s default
DEFAULT_MAX_ANG_VEL = 2.0       # main.py --max-angular-velocity-rad-s default
DEFAULT_CHUNK = 24              # --chunk-execute-steps default
DEFAULT_CROSSFADE = 2           # --chunk-crossfade-steps default


def _rot_err_deg(qa: np.ndarray, qb: np.ndarray) -> float:
    qa = qa / (np.linalg.norm(qa) + 1e-12)
    qb = qb / (np.linalg.norm(qb) + 1e-12)
    return 2.0 * math.degrees(math.acos(min(1.0, abs(float(np.dot(qa, qb))))))


def _crossfade(current, prev, idx, k):
    """Mirror of FlowMatchingActionSource._apply_chunk_crossfade (flow_inference.py:2356)."""
    current = tuple(float(v) for v in current[:6])
    if k <= 0 or prev is None or idx >= k:
        return current
    prev = tuple(float(v) for v in prev[:6])
    alpha = float(idx + 1) / float(k + 1)
    return tuple((1.0 - alpha) * prev[i] + alpha * current[i] for i in range(6))


def replay(poses: np.ndarray, cfg: dict, anchor_mode: str):
    """Return (reconstructed poses, per-step diagnostics)."""
    T = len(poses)
    lin_step = cfg["max_lin_vel"] * cfg["policy_dt"]
    ang_step = cfg["max_ang_vel"] * cfg["policy_dt"]
    chunk = cfg["chunk"]
    k = cfg["crossfade"]

    out = np.zeros_like(poses)
    out[0] = poses[0]
    running = poses[0].astype(np.float64).copy()
    prev_emitted = None
    n_clamped = 0
    raw_path = 0.0        # commanded translation path length (unclamped)
    emit_path = 0.0       # emitted translation path length (post-clamp)

    for t in range(T - 1):
        steps_since_boundary = t % chunk
        if steps_since_boundary == 0:
            prev_emitted = None  # crossfade anchor resets at boundary
            if anchor_mode == "actual":
                running = poses[t].astype(np.float64).copy()  # re-anchor to "measured"
            # anchor_mode == "chain": keep integrating, no re-anchor

        raw = np.asarray(pose_delta_local(poses[t], poses[t + 1]), dtype=np.float64)
        clamped = np.asarray(clamp_pose_delta(raw.tolist(), lin_step, ang_step), dtype=np.float64)
        if np.any(np.abs(clamped - raw) > 1e-9):
            n_clamped += 1
        emitted = np.asarray(_crossfade(clamped, prev_emitted, steps_since_boundary, k), dtype=np.float64)
        prev_emitted = emitted

        raw_path += float(np.linalg.norm(raw[:3]))
        emit_path += float(np.linalg.norm(emitted[:3]))

        running = np.asarray(pose_compose_local(running, emitted), dtype=np.float64)
        out[t + 1] = running

    pe = np.linalg.norm(poses[:, :3] - out[:, :3], axis=1) * 1000.0  # mm
    re = np.array([_rot_err_deg(poses[i, 3:7], out[i, 3:7]) for i in range(T)])
    diag = {
        "clamp_pct": 100.0 * n_clamped / max(1, T - 1),
        "path_deficit_pct": 100.0 * (raw_path - emit_path) / max(1e-9, raw_path),
        "pos_err_mm_max": float(pe.max()),
        "pos_err_mm_mean": float(pe.mean()),
        "rot_err_deg_max": float(re.max()),
    }
    return out, diag


def load_episodes(args):
    eps = []
    if args.npz_dir:
        for f in sorted(glob.glob(os.path.join(args.npz_dir, "*.npz"))):
            z = np.load(f, allow_pickle=True)
            for arm in ("left", "right"):
                key = f"{arm}_pose"
                if key in z and len(z[key]) >= args.chunk + 2:
                    eps.append((os.path.basename(f) + ":" + arm, z[key].astype(np.float64)))
    elif args.hdf5_dir:
        import h5py
        pat = os.path.join(args.hdf5_dir, "**", "episode_*.hdf5")
        for f in sorted(glob.glob(pat, recursive=True)):
            try:
                with h5py.File(f, "r") as h:
                    for arm in ("left", "right"):
                        ds = f"observations/{arm}/pose"
                        if ds in h and h[ds].shape[0] >= args.chunk + 2:
                            eps.append((os.path.relpath(f, args.hdf5_dir) + ":" + arm, h[ds][:].astype(np.float64)))
            except Exception:
                continue
    else:
        raise SystemExit("provide --npz-dir or --hdf5-dir")
    return eps


def summarize(vals):
    a = np.asarray(vals, dtype=np.float64)
    return {"mean": float(a.mean()), "p95": float(np.percentile(a, 95)), "worst": float(a.max())}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--npz-dir", help="dir of pose npz ({left,right}_pose keys)")
    src.add_argument("--hdf5-dir", help="dir tree of episode_*.hdf5 (pika UMI layout)")
    ap.add_argument("--policy-dt", type=float, default=DEFAULT_POLICY_DT)
    ap.add_argument("--max-lin-vel", type=float, default=DEFAULT_MAX_LIN_VEL, help="m/s (production default 0.15)")
    ap.add_argument("--max-ang-vel", type=float, default=DEFAULT_MAX_ANG_VEL, help="rad/s (production default 2.0)")
    ap.add_argument("--chunk", type=int, default=DEFAULT_CHUNK)
    ap.add_argument("--crossfade", type=int, default=DEFAULT_CROSSFADE)
    ap.add_argument("--anchor", choices=["actual", "chain", "both"], default="both")
    ap.add_argument("--json-out", help="write full per-episode report json")
    ap.add_argument("--limit", type=int, default=0, help="cap episodes (0=all)")
    args = ap.parse_args()

    eps = load_episodes(args)
    if args.limit:
        eps = eps[: args.limit]
    cfg = {
        "policy_dt": args.policy_dt, "max_lin_vel": args.max_lin_vel, "max_ang_vel": args.max_ang_vel,
        "chunk": args.chunk, "crossfade": args.crossfade,
    }
    lin_mm = args.max_lin_vel * args.policy_dt * 1000.0
    ang_deg = math.degrees(args.max_ang_vel * args.policy_dt)
    print(f"episodes(arm-series)={len(eps)}  policy_dt={args.policy_dt}s  chunk={args.chunk}  crossfade={args.crossfade}")
    print(f"per-step clamp: lin={lin_mm:.2f}mm/axis  ang={ang_deg:.2f}deg/axis  "
          f"(from {args.max_lin_vel} m/s, {args.max_ang_vel} rad/s)")
    print("=" * 96)

    modes = ["actual", "chain"] if args.anchor == "both" else [args.anchor]
    per_mode = {m: {"pos": [], "clamp": [], "deficit": []} for m in modes}
    records = []
    for name, poses in eps:
        rec = {"episode": name}
        for m in modes:
            _, d = replay(poses, cfg, m)
            per_mode[m]["pos"].append(d["pos_err_mm_max"])
            per_mode[m]["clamp"].append(d["clamp_pct"])
            per_mode[m]["deficit"].append(d["path_deficit_pct"])
            rec[m] = d
        records.append(rec)

    for m in modes:
        pos = summarize(per_mode[m]["pos"])
        clamp = summarize(per_mode[m]["clamp"])
        deficit = summarize(per_mode[m]["deficit"])
        print(f"[anchor={m:6s}] pos-err mm : mean {pos['mean']:6.1f}  p95 {pos['p95']:6.1f}  worst {pos['worst']:7.1f}")
        print(f"{'':17s} clamp %    : mean {clamp['mean']:6.1f}  p95 {clamp['p95']:6.1f}  worst {clamp['worst']:7.1f}")
        print(f"{'':17s} path lost %: mean {deficit['mean']:6.2f}  p95 {deficit['p95']:6.2f}  worst {deficit['worst']:7.2f}")
        print("-" * 96)

    if args.json_out:
        with open(args.json_out, "w") as fh:
            json.dump({"config": cfg, "records": records}, fh, indent=2)
        print(f"wrote {args.json_out}")


if __name__ == "__main__":
    main()
