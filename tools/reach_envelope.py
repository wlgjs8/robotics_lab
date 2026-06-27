#!/usr/bin/env python3
"""Compute the reachable-workspace envelope of one RB3-730E arm by FK Monte-Carlo.

Samples the 6 joint angles uniformly inside the URDF limits, runs forward
kinematics to the ``tcp`` frame (in the URDF root / arm-base frame), and records
every reachable TCP point and its radial distance from the arm base. The output
feeds two consumers:

  1. rb_gui viser overlay (``_add_reachability_cloud`` in rb_servo_gui/scene.py):
     the saved point cloud is attached under each arm's ``/stand/<side>_base``
     node, so the same base-frame cloud renders correctly (and mirrored) for both
     arms via the mount transform.

  2. safety.reach_constraint (rb_servo_server): the measured r_min / r_max define
     the radial shell the servo-loop velocity damper enforces so a Cartesian
     command never drives the TCP past the arm's reach (where IK fails and the arm
     used to silently stop). The recommended config values back the raw envelope
     off by --margin-m so the damper engages just inside the true boundary.

The radius origin is the URDF root (arm base) origin — the same point the servo
loop uses as the shell center (left_mount/right_mount.base_pose_in_stand). FK
sampling (not IK) is deliberate: it is the robust, solver-independent way to map
the reachable set; the geometric outer shell also bounds the full-extension
singularity where IK degrades.

Usage:
    python3 tools/reach_envelope.py [--samples N] [--out PATH] [--margin-m M]
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

import numpy as np


def shell_mesh(pts: "np.ndarray", n_lat: int, n_lon: int, min_count: int) -> tuple:
    """Outer reachable-surface mesh: per spherical-direction-bin max radius.

    The reachable set is star-shaped enough from the arm base that its OUTER
    boundary is well captured by r(direction) = max sample radius in that
    direction bin. A face is emitted ONLY between bins that each have >= min_count
    samples, so directions the arm cannot reach (e.g. toward the stand) become
    open holes instead of a filled skin bridging to the base — the surface stays a
    genuine outer shell with no spurious connections. Returns (vertices Nx3
    float32 in the base frame, faces Mx3 int32); not watertight by design."""
    r = np.linalg.norm(pts, axis=1)
    keep = r > 1e-6
    pts = pts[keep]
    r = r[keep]
    d = pts / r[:, None]
    polar = np.arccos(np.clip(d[:, 2], -1.0, 1.0))         # [0, pi]
    azim = np.arctan2(d[:, 1], d[:, 0])                     # [-pi, pi]
    i = np.clip((polar / np.pi * n_lat).astype(int), 0, n_lat - 1)
    j = np.clip(((azim + np.pi) / (2.0 * np.pi) * n_lon).astype(int), 0, n_lon - 1)
    flat = i * n_lon + j
    rmax = np.zeros(n_lat * n_lon, dtype=float)
    np.maximum.at(rmax, flat, r)
    count = np.bincount(flat, minlength=n_lat * n_lon)
    rmax = rmax.reshape(n_lat, n_lon)
    valid = (count.reshape(n_lat, n_lon) >= max(1, min_count))

    polar_c = (np.arange(n_lat) + 0.5) / n_lat * np.pi
    azim_c = (np.arange(n_lon) + 0.5) / n_lon * 2.0 * np.pi - np.pi
    sp = np.sin(polar_c)[:, None]
    cp = np.cos(polar_c)[:, None]
    sa = np.sin(azim_c)[None, :]
    ca = np.cos(azim_c)[None, :]
    verts = np.stack([(rmax * sp * ca).ravel(), (rmax * sp * sa).ravel(),
                      (rmax * cp).ravel()], axis=1)

    def vid(a: int, b: int) -> int:
        return a * n_lon + (b % n_lon)

    faces = []
    # Quad faces only where all four corner bins are reachable (valid).
    for a in range(n_lat - 1):
        for b in range(n_lon):
            bn = (b + 1) % n_lon
            if not (valid[a, b] and valid[a, bn] and valid[a + 1, b] and valid[a + 1, bn]):
                continue
            v00, v01 = vid(a, b), vid(a, bn)
            v10, v11 = vid(a + 1, b), vid(a + 1, bn)
            faces.append((v00, v10, v11))
            faces.append((v00, v11, v01))
    # Pole caps: fan from an apex only across ring segments whose both cells are
    # valid (skip entirely if the polar ring is unreachable, leaving an open cap).
    top_apex = len(verts)
    bot_apex = top_apex + 1
    top_r = float(rmax[0][valid[0]].mean()) if valid[0].any() else 0.0
    bot_r = float(rmax[-1][valid[-1]].mean()) if valid[-1].any() else 0.0
    verts = np.concatenate([verts, np.array([[0.0, 0.0, top_r], [0.0, 0.0, -bot_r]])], axis=0)
    for b in range(n_lon):
        bn = (b + 1) % n_lon
        if top_r > 0.0 and valid[0, b] and valid[0, bn]:
            faces.append((top_apex, vid(0, bn), vid(0, b)))
        if bot_r > 0.0 and valid[-1, b] and valid[-1, bn]:
            faces.append((bot_apex, vid(n_lat - 1, b), vid(n_lat - 1, bn)))
    return verts.astype(np.float32), np.array(faces, dtype=np.int32)


def _repo_root() -> Path:
    return Path(__file__).resolve().parent.parent


def _default_urdf() -> Path:
    env = os.environ.get("RB_GUI_ROBOT_URDF")
    if env:
        return Path(env)
    return _repo_root() / "rb_servo_server" / "descriptions" / "urdf" / "rb3_730e.urdf"


def _default_out() -> Path:
    return _repo_root() / "rb_servo_server" / "descriptions" / "reach_envelope_rb3_730e.npz"


def compute_envelope(
    urdf_path: Path,
    samples: int,
    tcp_frame: str,
    seed: int,
    n_lat: int,
    n_lon: int,
    min_count: int,
) -> dict:
    import yourdfpy

    urdf = yourdfpy.URDF.load(str(urdf_path))
    joint_names = list(urdf.actuated_joint_names)
    lowers = np.array([urdf.joint_map[n].limit.lower for n in joint_names], dtype=float)
    uppers = np.array([urdf.joint_map[n].limit.upper for n in joint_names], dtype=float)
    # Guard against unbounded continuous joints (±2π is plenty for reach coverage).
    lowers = np.where(np.isfinite(lowers), lowers, -np.pi)
    uppers = np.where(np.isfinite(uppers), uppers, np.pi)

    rng = np.random.default_rng(seed)
    qs = rng.uniform(lowers, uppers, size=(samples, len(joint_names)))

    pts = np.empty((samples, 3), dtype=float)
    for i in range(samples):
        urdf.update_cfg(qs[i])
        T = urdf.get_transform(tcp_frame)  # tcp pose in the URDF root (arm base) frame
        pts[i] = T[:3, 3]

    radii = np.linalg.norm(pts, axis=1)
    # Percentile envelope (robust to the handful of extreme single-orientation
    # configs at the very tip / fold).
    r_max_raw = float(np.percentile(radii, 99.5))
    r_min_raw = float(np.percentile(radii, 0.5))

    # Outer reachable-surface mesh (what the viewer renders as a translucent shell).
    verts, faces = shell_mesh(pts, n_lat, n_lon, min_count)

    return {
        "shell_vertices_base_m": verts,
        "shell_faces": faces,
        "joint_names": joint_names,
        "lowers_rad": lowers,
        "uppers_rad": uppers,
        "r_min_raw_m": r_min_raw,
        "r_max_raw_m": r_max_raw,
        "r_min_abs_m": float(radii.min()),
        "r_max_abs_m": float(radii.max()),
        "samples": int(samples),
        "tcp_frame": tcp_frame,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", type=Path, default=_default_urdf())
    ap.add_argument("--out", type=Path, default=_default_out())
    ap.add_argument("--samples", type=int, default=300_000)
    ap.add_argument("--n-lat", type=int, default=48,
                    help="latitude bins of the outer-shell surface mesh")
    ap.add_argument("--n-lon", type=int, default=96,
                    help="longitude bins of the outer-shell surface mesh")
    ap.add_argument("--min-count", type=int, default=8,
                    help="min FK samples per direction bin to treat it as reachable; "
                         "directions below this become open holes (no skin to the stand)")
    ap.add_argument("--tcp-frame", default="tcp")
    ap.add_argument("--margin-m", type=float, default=0.03,
                    help="safety back-off applied to the recommended config shell")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    if not args.urdf.exists():
        raise SystemExit(f"URDF not found: {args.urdf}")

    env = compute_envelope(args.urdf, args.samples, args.tcp_frame, args.seed,
                           args.n_lat, args.n_lon, args.min_count)

    # Recommended safety.reach_constraint shell: back the raw envelope inward so the
    # damper brakes just BEFORE the true reach boundary / inner fold.
    r_max_cfg = round(env["r_max_raw_m"] - args.margin_m, 4)
    r_min_cfg = round(env["r_min_raw_m"] + args.margin_m, 4)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        args.out,
        shell_vertices_base_m=env["shell_vertices_base_m"],
        shell_faces=env["shell_faces"],
        r_min_raw_m=env["r_min_raw_m"],
        r_max_raw_m=env["r_max_raw_m"],
        r_min_recommended_m=r_min_cfg,
        r_max_recommended_m=r_max_cfg,
        margin_m=args.margin_m,
        samples=env["samples"],
        tcp_frame=env["tcp_frame"],
    )
    # A small sidecar JSON for humans / config copy-paste.
    sidecar = args.out.with_suffix(".json")
    sidecar.write_text(json.dumps({
        "joint_names": env["joint_names"],
        "r_min_abs_m": env["r_min_abs_m"],
        "r_max_abs_m": env["r_max_abs_m"],
        "r_min_raw_m": env["r_min_raw_m"],
        "r_max_raw_m": env["r_max_raw_m"],
        "r_min_recommended_m": r_min_cfg,
        "r_max_recommended_m": r_max_cfg,
        "margin_m": args.margin_m,
        "samples": env["samples"],
    }, indent=2))

    print(f"RB3-730E reach envelope ({env['samples']} FK samples)")
    print(f"  raw radius      : [{env['r_min_raw_m']:.4f}, {env['r_max_raw_m']:.4f}] m "
          f"(abs [{env['r_min_abs_m']:.4f}, {env['r_max_abs_m']:.4f}])")
    print(f"  recommended cfg : r_min_m={r_min_cfg}  r_max_m={r_max_cfg}  "
          f"(margin {args.margin_m} m)")
    print(f"  shell mesh      : {len(env['shell_vertices_base_m'])} verts, "
          f"{len(env['shell_faces'])} faces ({args.n_lat}x{args.n_lon} bins)")
    print(f"  saved           : {args.out}")
    print(f"                    {sidecar}")
    print("\nPaste into safety.reach_constraint (rb_servo_server config):")
    print(f"  reach_constraint:\n    enable: true\n    r_min_m: {r_min_cfg}\n    r_max_m: {r_max_cfg}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
