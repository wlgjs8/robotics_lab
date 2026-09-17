#!/usr/bin/env python3
"""Lighthouse DRIFT audit: use the wrist camera as an independent witness of motion.

The 2026-09-15 data audit (`~/dq_audit`) catches tracker JUMPS -- a sample with a physically
impossible acceleration and a 6-16 mm position step. Those are easy because they are absurd. It
catches nothing slower. A lighthouse solution that creeps a few mm over a few seconds is physically
plausible, passes every acceleration test, and silently biases exactly the label this task is short
on: the approach depth.

This finds it without ground truth, by cross-checking the tracker against a sensor that shares none
of its failure modes. If two consecutive wrist frames are effectively identical, the camera did not
move; any position change the tracker reports over that interval is the tracker's own error. Summing
those signed errors over an episode separates the two regimes:

  * pure NOISE cancels -- the cumulative sum stays near zero and grows like sqrt(n)
  * DRIFT accumulates -- the cumulative sum walks away linearly

Reported per episode and per arm:

  still_frac        share of frames the camera calls motionless
  noise_mm_per_s    RMS tracker speed while the camera says nothing moved (the noise floor)
  drift_mm          net tracker displacement accumulated over ONLY the motionless frames
  drift_vs_noise    that net displacement in units of the random walk the noise alone would give;
                    >3 means the tracker moved somewhere the camera never went
  worst_axis        which of x/y/z carries it -- z matters most, it is the dz label

    tools/audit_tracker_drift.py ~/Downloads/data_*/episode_*.hdf5

The camera is the right witness precisely because it is what the POLICY sees: a label the camera
cannot corroborate is a label the policy cannot learn.
"""

from __future__ import annotations

import argparse
import glob
import json
import pathlib
import sys

import numpy as np

ARMS = ("left", "right")


def _decode_small(enc, size=(80, 60)):
    import cv2

    img = cv2.imdecode(np.asarray(enc, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
    if img is None:
        return None
    return cv2.resize(img, size, interpolation=cv2.INTER_AREA).astype(np.float32)


def audit_episode(path: str, arm: str, stride: int, still_pct: float) -> dict | None:
    import h5py

    with h5py.File(path, "r") as f:
        g = f[f"observations/{arm}"]
        pose = np.asarray(g["pose_synced" if "pose_synced" in g else "pose"], dtype=np.float64)
        ts = np.asarray(f["timestamp"], dtype=np.float64)
        imgs = g["images/realsense_color"]
        n = min(len(imgs), len(pose), len(ts))
        idx = np.arange(0, n, stride)
        frames, keep = [], []
        for t in idx:
            try:
                im = _decode_small(imgs[int(t)])
            except OSError:
                im = None
            if im is None:
                continue
            frames.append(im)
            keep.append(int(t))
    if len(frames) < 30:
        return None
    keep = np.array(keep)
    F = np.stack(frames)
    # camera motion between consecutive kept frames
    cam = np.mean(np.abs(np.diff(F, axis=0)), axis=(1, 2))
    # tracker motion over the SAME intervals
    p = pose[keep, :3] * 1000.0
    dp = np.diff(p, axis=0)
    dt = np.diff(ts[keep])
    ok = dt > 0
    cam, dp, dt = cam[ok], dp[ok], dt[ok]

    # A percentile threshold does not find motionless frames -- in a demonstration the operator is
    # moving almost the whole time, so the lowest 20% is just "slower". Two sounder readings:
    #
    #  1. NOISE FLOOR by extrapolation. Tracker speed against inter-frame image change is monotone
    #     and close to linear (verified on 09-02 ep003: 23 mm/s in the quietest bin rising to
    #     188 mm/s in the busiest). The intercept at ZERO image change is what the tracker reports
    #     when the camera sees nothing at all -- the noise floor -- without needing a still frame.
    #  2. DRIFT over the quietest frames, selected against the episode's OWN minimum image change
    #     rather than a percentile, so the set is genuinely near-motionless or empty.
    slope, floor = np.polyfit(cam, np.linalg.norm(dp, axis=1) / dt, 1)
    thr = float(cam.min()) * still_pct
    still = cam <= thr
    if still.sum() < 10:
        order = np.argsort(cam)[: max(10, len(cam) // 50)]
        still = np.zeros(len(cam), bool)
        still[order] = True
    step = dp[still]
    secs = float(dt[still].sum())
    speed = np.linalg.norm(step, axis=1) / dt[still]
    net = step.sum(axis=0)
    rw = np.sqrt((step.var(axis=0) * len(step)))
    ratio = np.abs(net) / np.maximum(rw, 1e-9)
    ax = int(np.argmax(ratio))
    return {
        "episode": pathlib.Path(path).name,
        "arm": arm,
        "still_frac": float(still.mean()),
        "still_seconds": secs,
        "noise_floor_mm_per_s": float(floor),
        "quiet_bin_dz_mm_per_frame": float(np.median(np.abs(step[:, 2]))),
        "noise_mm_per_s": float(np.sqrt(np.mean(speed**2))),
        "drift_mm": [float(v) for v in net],
        "drift_norm_mm": float(np.linalg.norm(net)),
        "drift_vs_noise": [float(v) for v in ratio],
        "worst_axis": "xyz"[ax],
        "worst_ratio": float(ratio[ax]),
        "z_drift_mm": float(net[2]),
        "z_vs_noise": float(ratio[2]),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("episodes", nargs="+")
    ap.add_argument("--arms", nargs="+", default=list(ARMS), choices=list(ARMS))
    ap.add_argument("--stride", type=int, default=3, help="90 Hz -> 30 Hz, the training rate")
    ap.add_argument("--still-pct", type=float, default=1.25,
                    help="quiet = inter-frame image change within this MULTIPLE of the episode minimum")
    ap.add_argument("--json", default="")
    args = ap.parse_args()

    paths = [p for pat in args.episodes for p in sorted(glob.glob(pat))] or args.episodes
    rows = []
    for path in paths:
        for arm in args.arms:
            try:
                r = audit_episode(path, arm, args.stride, args.still_pct)
            except Exception as exc:  # noqa: BLE001
                print(f"# skip {path} {arm}: {exc}", file=sys.stderr)
                continue
            if r:
                rows.append(r)
    if not rows:
        print("no usable episodes", file=sys.stderr)
        return 1

    print(f"{'episode':<26}{'arm':<7}{'quiet':>7}{'floor':>9}{'quiet dz':>10}"
          f"{'z drift':>9}{'z/noise':>9}{'worst':>7}")
    print(f"{'':<26}{'':<7}{'frac':>7}{'mm/s':>9}{'mm/frame':>10}{'mm':>9}{'x sigma':>9}{'axis':>7}")
    print("-" * 84)
    for r in rows:
        flag = "  <<" if abs(r["z_vs_noise"]) > 3 else ""
        print(f"{r['episode']:<26}{r['arm']:<7}{r['still_frac']:>7.3f}"
              f"{r['noise_floor_mm_per_s']:>9.1f}{r['quiet_bin_dz_mm_per_frame']:>10.3f}"
              f"{r['z_drift_mm']:>+9.2f}{r['z_vs_noise']:>+9.1f}{r['worst_axis']:>7}{flag}")

    z = np.array([r["z_drift_mm"] for r in rows])
    zr = np.array([r["z_vs_noise"] for r in rows])
    nf = np.array([r["noise_floor_mm_per_s"] for r in rows])
    qz = np.array([r["quiet_bin_dz_mm_per_frame"] for r in rows])
    print(f"\n# tracker noise floor extrapolated to zero image change: p50 {np.median(nf):.1f} mm/s"
          f"  (= {np.median(nf)/30:.2f} mm per 30 Hz training sample)")
    print(f"# |dz| per frame in the quietest bin: p50 {np.median(qz):.3f} mm -- this is the floor on"
          f" how well the dz LABEL can be known")
    print(f"# net z displacement over motionless frames: p50 {np.median(z):+.2f} mm, "
          f"|max| {np.abs(z).max():.2f} mm")
    print(f"# beyond 3 sigma of its own noise on z: {(np.abs(zr) > 3).sum()}/{len(zr)} episode-arms")
    print("#   (>3 sigma = the tracker walked somewhere the camera never went -- drift, not noise)")

    if args.json:
        pathlib.Path(args.json).write_text(json.dumps(rows, indent=1))
        print(f"\n# wrote {args.json}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
