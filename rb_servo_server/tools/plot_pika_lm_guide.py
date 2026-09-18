#!/usr/bin/env python3
"""Section the pika gripper base at the LM guide, and say what the collision model checks.

Companion to plot_pika_gripper_sections.py, which answered "what is 247.642 mm measured
FROM". This one answers "what is the WIDEST thing on the tool, and is the collision model
telling the truth about it".

The answer, from pika_gripper_base.STL alone (no robot involved):

  * the widest part of the whole tool is the LINEAR GUIDE RAIL -- one connected component,
    215.00 x 7.00 x 4.80 mm, at Z 137.30..142.10 from the flange. An MGN7 profile.
  * the two carriages ride it at |X| 73.12..103.92, Y +/-8.50, Z 138.80..145.30; 145.30 is
    the finger seat plane the rest of the tool geometry is datumed on.
  * with the jaw CLOSED the rail ends stick out 48.66 mm per side beyond every other part
    of the tool, fingers included. It is the first thing that meets the other arm.
  * the monitor USED to check not this shape but ONE CONVEX HULL of it
    (pika_gripper_base_hull.STL), which sweeps the 7 mm-thin rail down to the 70 mm
    flange: 4.16x the volume, median 11.8 mm and up to 44.5 mm of material that is not
    there, worst on the flanks at Z ~ 89 -- which is exactly where the other gripper's
    rail tip lands. Since 2026-09-18 it checks three convex pieces instead
    (pika_gripper_base_hull_{flange,housing,guide}.STL, 1062.2 cm3 total), so the red
    outline below is the OLD shape, kept as the thing the split removed.

  rb_servo_server/tools/plot_pika_lm_guide.py [--out PATH]
"""
from __future__ import annotations

import argparse
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
MESH = REPO / "rb_servo_server/descriptions/meshes/robots/rb5_850e/visual/tool"
OUT = REPO / "docs/reference/pika_lm_guide_sections.png"
TRAVEL_MM = 49.0  # gripper_finger_travel_m, measured open stop


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=OUT)
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    import trimesh

    base = trimesh.load(MESH / "pika_gripper_base.STL", process=False)
    hull = trimesh.load(MESH / "pika_gripper_base_hull.STL", process=False)
    fl = trimesh.load(MESH / "pika_finger_left_hull.STL", process=False)
    fr = trimesh.load(MESH / "pika_finger_right_hull.STL", process=False)

    def cut(mesh, normal, origin):
        s = mesh.section(plane_origin=origin, plane_normal=normal)
        return [] if s is None else [np.asarray(p) for p in s.discrete]

    def draw(ax, mesh, normal, origin, ia, ib, **kw):
        for p in cut(mesh, normal, origin):
            ax.plot(p[:, ia], p[:, ib], **kw)

    fig = plt.figure(figsize=(21, 9))
    gs = fig.add_gridspec(1, 3, width_ratios=[0.85, 1.35, 1.1])

    # --- (1) full XZ section: true base vs the hull the monitor checks -----------------
    ax = fig.add_subplot(gs[0, 0])
    draw(ax, hull, [0, 1, 0], [0, 0, 0], 0, 2, lw=1.6, c="tab:red", ls="--")
    draw(ax, base, [0, 1, 0], [0, 0, 0], 0, 2, lw=1.2, c="0.15")
    ax.plot([], [], c="0.15", lw=1.2, label="pika_gripper_base.STL (true)")
    ax.plot([], [], c="tab:red", lw=1.6, ls="--", label="…_base_hull.STL (retired)")
    ax.set_title("XZ cut, y=0 — the ONE hull this replaced\n"
                 "4.16x the volume; up to 44.5 mm of phantom", fontsize=12)
    ax.legend(loc="lower right", fontsize=9)
    ax.annotate("LM guide rail\n215.00 mm tip to tip", xy=(107.5, 139.7),
                xytext=(-10, 205), fontsize=10, color="tab:blue", ha="center",
                arrowprops=dict(arrowstyle="->", color="tab:blue"))
    ax.annotate("worst phantom\n44.5 mm (z≈89)", xy=(80, 89), xytext=(120, 40),
                fontsize=9, color="tab:red", ha="center",
                arrowprops=dict(arrowstyle="->", color="tab:red"))

    # --- (2) zoom on the guide ---------------------------------------------------------
    ax2 = fig.add_subplot(gs[0, 1])
    draw(ax2, base, [0, 1, 0], [0, 0, 0], 0, 2, lw=1.3, c="0.15")
    for m, nm in ((fl, "finger"), (fr, None)):
        draw(ax2, m, [0, 1, 0], [0, 0, 0], 0, 2, lw=1.1, c="tab:green")
        pts = np.asarray(m.vertices).copy()
        pts[:, 0] += TRAVEL_MM * (1 if pts[:, 0].mean() < 0 else -1)
        shifted = trimesh.Trimesh(pts, m.faces, process=False)
        draw(ax2, shifted, [0, 1, 0], [0, 0, 0], 0, 2, lw=1.1, c="tab:green", ls=":")
    ax2.plot([], [], c="tab:green", lw=1.1, label="finger hull, jaw OPEN")
    ax2.plot([], [], c="tab:green", lw=1.1, ls=":", label="finger hull, jaw CLOSED")
    for z, t, c in (
        (137.3, "rail bottom Z 137.30", "tab:blue"),
        (142.1, "rail top Z 142.10", "tab:blue"),
        (145.3, "carriage top Z 145.30 = finger seat plane", "tab:orange"),
        (123.8, "carriage plates from Z 123.80", "0.45"),
    ):
        ax2.axhline(z, color=c, lw=1.0, ls="--")
        ax2.text(132, z, t, color=c, fontsize=9, va="center")
    ax2.annotate("", xy=(-107.5, 118), xytext=(107.5, 118),
                 arrowprops=dict(arrowstyle="<->", color="tab:blue", lw=1.6))
    ax2.text(0, 114.5, "rail 215.00 mm", color="tab:blue", fontsize=11, ha="center")
    ax2.annotate("", xy=(58.84, 168), xytext=(107.5, 168),
                 arrowprops=dict(arrowstyle="<->", color="tab:red", lw=1.8))
    ax2.text(83, 170, "48.66 mm outboard\nof the CLOSED jaw",
             color="tab:red", fontsize=9.5, ha="center")
    ax2.set_xlim(-130, 130); ax2.set_ylim(108, 182)
    ax2.legend(loc="upper left", fontsize=9)
    ax2.set_title("XZ zoom on the LM guide (vertical scale exaggerated)\nrail 215.00 x 7.00 x 4.80 — an MGN7 profile", fontsize=12)

    # --- (3) plan view through the rail ------------------------------------------------
    ax3 = fig.add_subplot(gs[0, 2])
    for z0, c in ((139.5, "tab:blue"), (144.0, "tab:orange"), (130.0, "0.5")):
        for p in cut(base, [0, 0, 1], [0, 0, z0]):
            ax3.plot(p[:, 0], p[:, 1], lw=1.4, color=c)
        ax3.plot([], [], color=c, lw=1.4, label=f"z = {z0} mm")
    for p in cut(hull, [0, 0, 1], [0, 0, 130.0]):
        ax3.plot(p[:, 0], p[:, 1], lw=1.4, color="tab:red", ls="--")
    ax3.plot([], [], color="tab:red", lw=1.4, ls="--", label="hull at z = 130")
    ax3.set_title("XY cuts — rail (7 mm wide) and the two MGN7H carriages", fontsize=12)
    ax3.legend(loc="upper right", fontsize=9)

    for a, (xl, yl) in ((ax, ("x (mm)", "z (mm)")), (ax2, ("x (mm)", "z (mm)")),
                        (ax3, ("x (mm)", "y (mm)"))):
        a.grid(alpha=0.3); a.set_xlabel(xl); a.set_ylabel(yl)
    ax.set_aspect("equal"); ax3.set_aspect("equal")
    fig.suptitle("pika gripper LM guide — x is the jaw axis, z is from the FLANGE face "
                 "(attachment_site)", fontsize=14)
    fig.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=110)
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
