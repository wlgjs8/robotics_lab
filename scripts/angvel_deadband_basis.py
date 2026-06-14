#!/usr/bin/env python3
"""Judgment basis for cartesian_control.twist_angular_deadband_rad_s.

The server holds orientation whenever the commanded angular speed ||omega|| is
below the deadband D (cartesian_servo_controller.cpp computeTwistTarget). Raising
D is only safe if little *intended* rotation falls below it -- otherwise the hold
fights the policy on slow reorientations.

The flow policy reproduces the per-step rotation present in the demos: the chunk
step delta is rotvec(q_t^-1 . q_{t+1}) (flow_dataset.pose_delta_local), and the
controller maps it to twist = delta / policy_dt. So the angular speed the deadband
sees == (geodesic angle between consecutive demo orientations) / policy_dt. The
geodesic angle equals the rotvec norm and is frame-invariant (ee_local vs stand
only rotate the vector, not its length), so we read it straight off the recorded
pose quaternions.

Usage:
    python3 scripts/angvel_deadband_basis.py 'data/data_*/episode_*.hdf5'
    python3 scripts/angvel_deadband_basis.py <files...> --policy-dt 0.0333
"""
from __future__ import annotations

import argparse
import glob

import h5py
import numpy as np

CANDIDATES = [0.0001, 0.005, 0.01, 0.02, 0.03, 0.05]


def episode_angular_speed(handle: h5py.File, policy_dt: float | None) -> list[np.ndarray]:
    """Per-arm angular-speed samples (rad/s) from consecutive recorded orientations."""
    # dt: prefer the recording cadence unless an inference policy_dt is forced.
    if policy_dt is not None:
        dt = policy_dt
    else:
        hz = float(handle.attrs.get("effective_hz", handle.attrs.get("record_hz", 0.0)))
        dt = 1.0 / hz if hz > 0.0 else None
    out: list[np.ndarray] = []
    for arm in ("left", "right"):
        key = f"observations/{arm}/pose"
        if key not in handle:
            continue
        pose = np.asarray(handle[key], dtype=np.float64)  # (T,7) x,y,z,qx,qy,qz,qw
        if pose.shape[0] < 2 or pose.shape[1] < 7:
            continue
        q = pose[:, 3:7]
        q /= np.linalg.norm(q, axis=1, keepdims=True) + 1e-12
        dot = np.abs(np.sum(q[:-1] * q[1:], axis=1)).clip(0.0, 1.0)
        angle = 2.0 * np.arccos(dot)  # geodesic rotation angle between frames (rad)
        step_dt = dt
        if step_dt is None:  # no attr; fall back to per-file timestamp spacing
            ts = np.asarray(handle["timestamp"], dtype=np.float64)
            step_dt = float(np.median(np.diff(ts))) if ts.size > 1 else None
        if not step_dt or step_dt <= 0.0:
            continue
        out.append(angle / step_dt)
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("hdf5", nargs="+", help="demo .hdf5 files or globs")
    ap.add_argument("--policy-dt", type=float, default=None,
                    help="force inference policy_dt_sec (default: each file's effective_hz)")
    args = ap.parse_args()

    paths = sorted({p for g in args.hdf5 for p in glob.glob(g)})
    if not paths:
        raise SystemExit("no files matched")

    samples: list[np.ndarray] = []
    hz_seen: list[float] = []
    for p in paths:
        with h5py.File(p, "r") as h:
            hz_seen.append(float(h.attrs.get("effective_hz", 0.0)))
            samples += episode_angular_speed(h, args.policy_dt)
    if not samples:
        raise SystemExit("no pose data found")

    v = np.concatenate(samples)
    v = v[np.isfinite(v)]
    dt_note = (f"forced policy_dt={args.policy_dt}s"
               if args.policy_dt else
               f"recording cadence ~{np.median(hz_seen):.1f}Hz")
    print(f"episodes={len(paths)}  arm-streams={len(samples)}  samples={v.size}  ({dt_note})")
    print("\n  angular-speed percentiles (the quantity the deadband compares against):")
    for pct in (50, 75, 90, 95, 99):
        q = float(np.percentile(v, pct))
        print(f"    p{pct:>2} = {q:7.4f} rad/s  ({np.rad2deg(q):6.2f} deg/s)")
    print(f"    max  = {v.max():7.4f} rad/s  ({np.rad2deg(v.max()):6.2f} deg/s)")
    print("\n  candidate deadband D  ->  fraction of commands FROZEN (held):")
    for d in CANDIDATES:
        frac = float((v < d).mean())
        bar = "#" * int(round(frac * 40))
        print(f"    D={d:<7g} ->  {frac * 100:5.1f}%  {bar}")
    print("\n  read: if a candidate D freezes a large fraction, the policy rotates "
          "slowly most of the time and raising the deadband to D would suppress real "
          "motion. If ~0%, that D is below the command floor and safe to adopt.")


if __name__ == "__main__":
    main()
