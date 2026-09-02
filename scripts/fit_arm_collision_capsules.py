#!/usr/bin/env python3
"""Fit one collision capsule per RB3-730e collision hull, in each link's local
frame, for the dual-arm self-collision guard.

The current self-collision model approximates each arm as 7 straight capsules on
the joint-origin skeleton with large uniform radii. That misses the RB3 link
"dogleg" shape (link2/link4 ship multiple convex hulls) and over-inflates the
wrist/gripper. This script reads the URDF collision meshes, PCA-fits a tight
capsule (principal-axis segment + max perpendicular radius) per hull, expresses
the endpoints in the parent LINK frame (applying each <collision><origin>), and
emits an `arm_capsules` template: {parent_frame, p0_m, p1_m, radius_m}.

The same template applies to both arms; the server FK-transforms each capsule by
its parent frame placement (per arm joints + mount) at runtime.

Usage:
  python3 scripts/fit_arm_collision_capsules.py [--radius-pct 100] [--render OUT.png]
"""
from __future__ import annotations

import argparse
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

URDF = "rb_servo_server/descriptions/urdf/rb3_730e.urdf"
# Gripper visual mesh (no collision mesh exists); attached under attachment_site
# via tool_joint (rpy z=+90deg), STL in millimeters.
GRIPPER_MESH_REL = "../meshes/robots/rb5_850e/visual/tool/pika_gripper.STL"
GRIPPER_PARENT = "attachment_site"
GRIPPER_SCALE = 0.001
GRIPPER_RPY = (0.0, 0.0, math.pi / 2.0)


def rpy_to_R(rpy):
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def parse_vec(s, default=(0.0, 0.0, 0.0)):
    if not s:
        return np.array(default, float)
    return np.array([float(x) for x in s.split()], float)


def fit_capsule(points: np.ndarray, radius_pct: float):
    """PCA-fit a capsule to a point cloud. Returns (p0, p1, radius)."""
    mean = points.mean(axis=0)
    centered = points - mean
    # Principal axis = eigenvector of largest eigenvalue.
    cov = centered.T @ centered
    evals, evecs = np.linalg.eigh(cov)
    axis = evecs[:, int(np.argmax(evals))]
    axis = axis / np.linalg.norm(axis)
    t = centered @ axis
    tmin, tmax = float(t.min()), float(t.max())
    perp = centered - np.outer(t, axis)
    perp_dist = np.linalg.norm(perp, axis=1)
    radius = float(np.percentile(perp_dist, radius_pct))
    p0 = mean + tmin * axis
    p1 = mean + tmax * axis
    return p0, p1, radius


def collision_entries(urdf_path):
    """Yield (link_name, mesh_abs_path, origin_xyz, origin_rpy) per collision."""
    base = os.path.dirname(urdf_path)
    tree = ET.parse(urdf_path)
    for link in tree.getroot().findall("link"):
        name = link.get("name")
        for col in link.findall("collision"):
            geom = col.find("geometry")
            mesh = geom.find("mesh") if geom is not None else None
            if mesh is None:
                continue
            o = col.find("origin")
            xyz = parse_vec(o.get("xyz") if o is not None else None)
            rpy = parse_vec(o.get("rpy") if o is not None else None)
            scale = parse_vec(mesh.get("scale"), (1.0, 1.0, 1.0))
            path = os.path.normpath(os.path.join(base, mesh.get("filename")))
            yield name, path, xyz, rpy, scale


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--radius-pct", type=float, default=100.0,
                    help="percentile of perpendicular distance used as radius (100=max, conservative)")
    ap.add_argument("--render", default="")
    args = ap.parse_args()

    capsules = []  # (parent_frame, p0, p1, radius, source)
    meshes_for_render = []  # (verts_local, parent_frame)

    for link, path, xyz, rpy, scale in collision_entries(URDF):
        m = trimesh.load(path, force="mesh")
        v = np.asarray(m.vertices, float) * scale
        # collision <origin>: mesh coords -> link frame.
        v = (rpy_to_R(rpy) @ v.T).T + xyz
        p0, p1, r = fit_capsule(v, args.radius_pct)
        capsules.append((link, p0, p1, r, os.path.basename(path)))
        meshes_for_render.append((v, link))

    # Gripper, expressed in attachment_site frame (z = approach axis). The Pika
    # gripper has distinct parts: wrist/body base (z 0..0.05), a CAMERA that juts
    # out in +y (z 0.05..0.10, y up to ~0.104), a wide jaw HOUSING cross-bar
    # (z 0.10..0.16, x +/-0.107), and the FINGERS/TIP (z 0.16..0.248, spread in x).
    # PCA on the whole mesh tilts badly, so model each part as a clean axis-aligned
    # capsule fit from the mesh points in that region. Radii from a high percentile
    # of the local extent (covers the part without the single-vertex bloat). The
    # TIP capsule is placed so its far cap ends only ~3 mm past the fingertip plane
    # (not radius-far past it). Tunable in config.
    gpath = os.path.normpath(os.path.join(os.path.dirname(URDF), GRIPPER_MESH_REL))
    gm = trimesh.load(gpath, force="mesh")
    gv = np.asarray(gm.vertices, float) * GRIPPER_SCALE
    gv = (rpy_to_R(GRIPPER_RPY) @ gv.T).T
    z = gv[:, 2]
    ztip = float(z.max())  # fingertip plane ~0.2476 (== tcp offset)
    pct = min(args.radius_pct, 95.0)

    def band(zlo, zhi, mask=None):
        m = (z >= zlo) & (z < zhi)
        if mask is not None:
            m = m & mask
        return gv[m]

    grip = []
    not_cam = gv[:, 1] <= 0.035  # exclude the +y camera protrusion from body/housing fits
    # 1) Body/wrist base: central column near the axis, z 0..0.10 (camera excluded).
    b = band(0.0, 0.10, mask=not_cam)
    rb = float(np.percentile(np.linalg.norm(b[:, :2], axis=1), pct))
    grip.append((np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.10]), rb, "pika[body]"))
    # 2) Camera: the +y protrusion at z 0.05..0.10. Capsule along +y.
    cam = band(0.05, 0.105, mask=gv[:, 1] > 0.035)
    if len(cam) > 20:
        zc = float(cam[:, 2].mean())
        y0, y1 = float(np.percentile(cam[:, 1], 5)), float(np.percentile(cam[:, 1], 95))
        rcam = float(np.percentile(np.abs(cam[:, 0]), pct))
        grip.append((np.array([0.0, y0, zc]), np.array([0.0, y1, zc]), max(rcam, 0.028), "pika[camera]"))
    # 3) Jaw housing cross-bar: wide in x at z 0.10..0.16 (camera excluded). Along x.
    h = band(0.10, 0.16, mask=not_cam)
    zh = float(h[:, 2].mean())
    xh = float(np.percentile(np.abs(h[:, 0]), 98))
    rh = float(np.percentile(np.abs(h[:, 1]), pct))
    grip.append((np.array([-xh, 0.0, zh]), np.array([xh, 0.0, zh]), max(rh, 0.028), "pika[housing]"))
    # 4) Fingers/tip: spread in x near the tip. Capsule along x, placed so the
    #    far (z) cap ends ~3 mm past the fingertip plane.
    t = band(0.20, ztip + 1e-6)
    xt = float(np.percentile(np.abs(t[:, 0]), 98))
    rt = float(np.percentile(np.abs(t[:, 1]), pct))
    rt = max(rt, 0.022)
    zt = ztip + 0.003 - rt  # far cap (zt + rt) == ztip + 3mm
    grip.append((np.array([-xt, 0.0, zt]), np.array([xt, 0.0, zt]), rt, "pika[tip]"))

    for p0, p1, r, label in grip:
        capsules.append((GRIPPER_PARENT, p0, p1, float(r), label))
        meshes_for_render.append((gv, GRIPPER_PARENT))

    # --- report ---
    print(f"# radius_pct={args.radius_pct}; {len(capsules)} capsules")
    print("# parent_frame      p0_m                          p1_m                          radius_m   len   source")
    for frame, p0, p1, r, src in capsules:
        ln = np.linalg.norm(p1 - p0)
        print(f"  {frame:16s} [{p0[0]:+.4f},{p0[1]:+.4f},{p0[2]:+.4f}] "
              f"[{p1[0]:+.4f},{p1[1]:+.4f},{p1[2]:+.4f}] r={r:.4f} L={ln:.4f}  {src}")

    print("\n# --- YAML arm_capsules block ---")
    print("    arm_capsules:")
    for frame, p0, p1, r, src in capsules:
        print(f"      - {{ frame: {frame:16s}, "
              f"p0_m: [{p0[0]:+.4f}, {p0[1]:+.4f}, {p0[2]:+.4f}], "
              f"p1_m: [{p1[0]:+.4f}, {p1[1]:+.4f}, {p1[2]:+.4f}], "
              f"radius_m: {r:.4f} }}  # {src}")

    if args.render:
        render(meshes_for_render, capsules, args.render)


def render(meshes_for_render, capsules, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    # Render in each capsule's LOCAL frame stacked (no FK) just to eyeball
    # capsule-vs-hull fit per link; FK comparison happens later in viser.
    fig = plt.figure(figsize=(14, 8))
    for idx, (frame, p0, p1, r, src) in enumerate(capsules):
        ax = fig.add_subplot(3, 5, idx + 1, projection="3d")
        verts = next(v for v, f in meshes_for_render if f == frame and True) if False else None
        # match by index order (capsules and meshes_for_render aligned)
        verts = meshes_for_render[idx][0]
        ax.scatter(verts[:, 0], verts[:, 1], verts[:, 2], s=1, c="gray", alpha=0.3)
        # capsule core line
        ax.plot([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]], c="red", lw=2)
        ax.set_title(f"{frame}\nr={r:.3f} L={np.linalg.norm(p1-p0):.3f}", fontsize=7)
        ax.set_box_aspect([1, 1, 1])
    fig.tight_layout()
    fig.savefig(out, dpi=90)
    print(f"\nrendered {out}")


if __name__ == "__main__":
    main()
