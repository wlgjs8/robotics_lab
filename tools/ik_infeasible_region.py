#!/usr/bin/env python3
"""Build the viser "A 영역 (특이점 원통)" overlay — per-arm base-axis singularity cylinder.

This REPLACES the former top-down IK-infeasible *shell*. Within the reach
envelope, the genuine no-go core for Cartesian (Move L / streaming twist) control
is the base-axis (J1) **velocity singularity column** — Rainbow's documented
"A 영역" (rb_cobot_docs product_introduction/robot_workarea): the region directly
above/below the base where Move J is fine but Cartesian motion forces runaway
joint speed. We render it as a capped cylinder coaxial with each arm's J1 axis.

Radius (option 1 — velocity singularity, the operator-meaningful definition):

    R = v_ref / dq_max_base          # tangential Cartesian speed that saturates J1

i.e. to translate the TCP at v_ref tangentially at radius r from the J1 axis the
base joint must spin v_ref / r; when r < R that exceeds the joint speed limit and
Cartesian tracking breaks down (the source of the dq_max-saturation tremble). The
defaults mirror the deployed config:

    v_ref     = cartesian_control.max_linear_move_speed_m_s   (0.20 m/s)
    dq_max    = safety.dq_max_deg_s[0]  (J1)                  (60 deg/s)
    => R ~= 0.20 / (60*pi/180) ~= 0.191 m

The cylinder is NOT a fixed object: it GROWS with commanded speed (R proportional
to v_ref). Regenerate if the speed cap changes (pass --speed-mps / --dqmax-deg or
--radius-m).

Geometry note — why one cylinder serves both arms: in each arm's
``/stand/<side>_base`` frame the J1 axis is exactly +Z through the origin
(link0 is a pure Z-rotation of the base node), and the base-axis singularity is
mount-independent in that frame. So we compute ONE cylinder in the base frame and
write it for both arms; scene.py applies each arm's mount tilt via its
``/stand/<side>_base`` node. The axial extent is clipped to the reachable z-range
(measured by FK) so only the in-reach segment of the column is drawn.

Output mirrors the prior npz schema (left/right ``_vertices_base_m`` / ``_faces``)
so rb_gui/scene.py renders it unchanged.

Usage:
    python3 tools/ik_infeasible_region.py                 # default R from 0.2 m/s
    python3 tools/ik_infeasible_region.py --speed-mps 0.3 # fatter cylinder
    python3 tools/ik_infeasible_region.py --radius-m 0.22 # explicit radius override
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np


def _repo_root() -> Path:
    return Path(__file__).resolve().parent.parent


def _default_urdf() -> Path:
    return _repo_root() / "rb_servo_server" / "descriptions" / "urdf" / "rb3_730e.urdf"


def _default_out() -> Path:
    return _repo_root() / "rb_servo_server" / "descriptions" / "ik_infeasible_rb3_730e.npz"


def cylinder_mesh(radius: float, z_lo: float, z_hi: float, sections: int) -> tuple:
    """Capped cylinder coaxial with the base-frame +Z axis (x=y=0), z in [z_lo, z_hi].

    Returns (vertices Nx3 float32, faces Mx3 int32) in the ARM-BASE frame."""
    import trimesh

    height = float(z_hi - z_lo)
    if height <= 0:
        raise SystemExit(f"empty axial extent: z_lo={z_lo} >= z_hi={z_hi}")
    mesh = trimesh.creation.cylinder(radius=float(radius), height=height, sections=int(sections))
    # trimesh centers the cylinder at the origin along +Z; lift it to span [z_lo, z_hi].
    mesh.apply_translation([0.0, 0.0, 0.5 * (z_lo + z_hi)])
    return mesh.vertices.astype(np.float32), mesh.faces.astype(np.int32)


def measure_axial_extent(urdf_path: Path, radius: float, samples: int,
                         tcp_frame: str, seed: int) -> tuple:
    """FK-measure the reachable z-range (base frame) of TCP points inside the cylinder.

    Mirrors tools/reach_envelope.py's sampler (same yourdfpy URDF + tcp frame) so the
    'within reach' clip matches the reach overlay. Returns (z_lo, z_hi) in metres."""
    import yourdfpy

    urdf = yourdfpy.URDF.load(str(urdf_path))
    joint_names = list(urdf.actuated_joint_names)
    lowers = np.array([urdf.joint_map[n].limit.lower for n in joint_names], dtype=float)
    uppers = np.array([urdf.joint_map[n].limit.upper for n in joint_names], dtype=float)
    rng = np.random.default_rng(seed)
    qs = rng.uniform(lowers, uppers, size=(samples, len(joint_names)))

    zin: list[float] = []
    r2 = radius * radius
    for i in range(samples):
        urdf.update_cfg(qs[i])
        p = urdf.get_transform(tcp_frame)[:3, 3]  # tcp in URDF root (arm-base) frame
        if p[0] * p[0] + p[1] * p[1] <= r2:       # inside the J1-axis cylinder
            zin.append(float(p[2]))
    if not zin:
        raise SystemExit(
            f"no reachable TCP samples inside radius {radius:.3f} m — increase --samples "
            f"or --radius-m")
    z = np.asarray(zin)
    return float(z.min()), float(z.max())


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", type=Path, default=_default_urdf())
    ap.add_argument("--out", type=Path, default=_default_out())
    ap.add_argument("--tcp-frame", default="tcp")
    # Radius (option 1): R = v_ref / dq_max, unless --radius-m overrides.
    ap.add_argument("--radius-m", type=float, default=None,
                    help="explicit cylinder radius [m]; overrides --speed-mps/--dqmax-deg")
    ap.add_argument("--speed-mps", type=float, default=0.20,
                    help="reference Cartesian speed [m/s] (config max_linear_move_speed_m_s)")
    ap.add_argument("--dqmax-deg", type=float, default=60.0,
                    help="base-joint (J1) speed limit [deg/s] (config dq_max_deg_s[0])")
    # Axial extent: measured from FK unless both overrides are given.
    ap.add_argument("--z-lo", type=float, default=None, help="override lower z bound [m]")
    ap.add_argument("--z-hi", type=float, default=None, help="override upper z bound [m]")
    ap.add_argument("--samples", type=int, default=200_000,
                    help="FK samples for axial-extent measurement")
    ap.add_argument("--sections", type=int, default=48, help="radial segments of the cylinder")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    if not args.urdf.exists():
        raise SystemExit(f"URDF not found: {args.urdf}")

    if args.radius_m is not None:
        radius = float(args.radius_m)
        radius_src = "explicit --radius-m"
    else:
        radius = float(args.speed_mps) / math.radians(float(args.dqmax_deg))
        radius_src = f"v_ref/dq_max = {args.speed_mps} / {args.dqmax_deg}deg_s"

    if args.z_lo is not None and args.z_hi is not None:
        z_lo, z_hi = float(args.z_lo), float(args.z_hi)
        extent_src = "explicit --z-lo/--z-hi"
    else:
        z_lo, z_hi = measure_axial_extent(args.urdf, radius, args.samples,
                                          args.tcp_frame, args.seed)
        extent_src = f"FK-measured ({args.samples} samples, in-reach)"

    verts, faces = cylinder_mesh(radius, z_lo, z_hi, args.sections)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        args.out,
        # both arms share the identical base-frame cylinder (mount-independent here)
        left_vertices_base_m=verts, left_faces=faces,
        right_vertices_base_m=verts, right_faces=faces,
        radius_m=float(radius),
        z_lo_m=float(z_lo), z_hi_m=float(z_hi),
        v_ref_mps=float(args.speed_mps), dqmax_deg=float(args.dqmax_deg),
        sections=int(args.sections),
        region="base_axis_singularity_cylinder",
        # kept for scene/app back-compat (the status text counts these)
        left_cells=int(len(verts)), right_cells=int(len(verts)),
    )
    sidecar = args.out.with_suffix(".json")
    sidecar.write_text(json.dumps({
        "region": "base_axis_singularity_cylinder",
        "radius_m": float(radius),
        "radius_source": radius_src,
        "z_lo_m": float(z_lo), "z_hi_m": float(z_hi),
        "extent_source": extent_src,
        "v_ref_mps": float(args.speed_mps),
        "dqmax_deg": float(args.dqmax_deg),
        "sections": int(args.sections),
        "note": "R = v_ref/dq_max (vendor A-region velocity singularity); grows with speed",
    }, indent=2))

    print("RB3-730E A 영역 (base-axis singularity cylinder) — arm-base frame, both arms identical")
    print(f"  radius          : {radius*1000:.0f} mm  ({radius_src})")
    print(f"  axial extent z  : [{z_lo:.3f}, {z_hi:.3f}] m  ({extent_src})")
    print(f"  mesh            : {len(verts)} verts, {len(faces)} faces (sections={args.sections})")
    print(f"  saved           : {args.out}")
    print(f"                    {sidecar}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
