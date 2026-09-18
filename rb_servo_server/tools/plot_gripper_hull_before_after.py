#!/usr/bin/env python3
"""Before/after sections of the gripper base collision shell (2026-09-18 split).

BEFORE: one convex hull of pika_gripper_base.STL. It has to span the 215.00 mm LM guide
rail at Z 137..142 and the Ø70 flange at Z 0, so it claims the cone between them --
1653.2 cm3 for a 397.7 cm3 part.

AFTER: three convex pieces split by connected component at Z 63 and Z 123.8
(pika_gripper_base_hull_{flange,housing,guide}.STL), 1062.2 cm3, union asserted to
contain the source mesh.

The last panel is the one that matters: a real cut through the actual contact, at a
RECORDED pose (servo_log_20260917_154112.csv, one of the 681 ticks where the left<->right
pika_gripper_base barrier was braking). The cut plane passes through both true witness
points and contains the gap direction, so the distances in it are the real ones.

  rb_servo_server/tools/plot_gripper_hull_before_after.py [--out PATH]
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
MESH = REPO / "rb_servo_server/descriptions/meshes/robots/rb5_850e/visual/tool"
URDF = REPO / "rb_servo_server/descriptions/urdf/dual_rb5_850e_ver3.urdf"
OUT = REPO / "docs/reference/pika_base_hull_before_after.png"
SHELL = ("pika_gripper_base_hull_flange.STL", "pika_gripper_base_hull_housing.STL",
         "pika_gripper_base_hull_guide.STL")
SHELL_LABEL = ("flange (_0)", "housing (_1)", "guide (_2)")
SHELL_COLOR = ("tab:blue", "tab:green", "tab:purple")
# The recorded braking pose (deg), left 6 then right 6.
Q_LEFT = [-91.5603, 47.4223, 102.6035, -4.3772, -150.4285, 11.8912]
Q_RIGHT = [54.3387, -73.8441, -107.3952, 34.2658, 82.6443, -123.3438]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=OUT)
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import trimesh
    import coal
    import pinocchio as pin

    base = trimesh.load(MESH / "pika_gripper_base.STL", process=False)
    hull = trimesh.load(MESH / "pika_gripper_base_hull.STL", process=False)
    shell = [trimesh.load(MESH / n, process=False) for n in SHELL]

    def draw(ax, mesh, normal, origin, ia, ib, **kw):
        s = mesh.section(plane_origin=origin, plane_normal=normal)
        if s is None:
            return
        for p in s.discrete:
            p = np.asarray(p)
            ax.plot(p[:, ia], p[:, ib], **kw)

    fig = plt.figure(figsize=(22, 9.5))
    gs = fig.add_gridspec(1, 4, width_ratios=[1, 1, 1, 1.45])

    # --- 1/2: the tool's own XZ section, before and after --------------------------
    for col, (title, pieces, colors, labels) in enumerate((
        ("BEFORE — one convex hull\n1653.2 cm³  (4.16× the part)",
         [hull], ["tab:red"], ["pika_gripper_base_hull.STL"]),
        ("AFTER — three convex pieces\n1062.2 cm³  (2.67×)",
         shell, SHELL_COLOR, SHELL_LABEL),
    )):
        ax = fig.add_subplot(gs[0, col])
        for m, c, lab in zip(pieces, colors, labels):
            draw(ax, m, [0, 1, 0], [0, 0, 0], 0, 2, lw=1.7, c=c, ls="--")
            ax.plot([], [], c=c, lw=1.7, ls="--", label=lab)
        draw(ax, base, [0, 1, 0], [0, 0, 0], 0, 2, lw=1.1, c="0.15")
        ax.plot([], [], c="0.15", lw=1.1, label="the actual part")
        ax.axhline(89.0, color="0.6", lw=0.9, ls=":")
        ax.text(-120, 92, "z = 89  (panel 3)", fontsize=8.5, color="0.35")
        if col == 0:
            ax.annotate("44.5 mm of phantom:\nthe cone from the rail\nends down to the flange",
                        xy=(80, 89), xytext=(55, 200), fontsize=9.5, color="tab:red",
                        ha="center", arrowprops=dict(arrowstyle="->", color="tab:red"))
        else:
            ax.annotate("the cone is gone", xy=(45, 95), xytext=(55, 200),
                        fontsize=9.5, color="tab:green", ha="center",
                        arrowprops=dict(arrowstyle="->", color="tab:green"))
        ax.set_title(title, fontsize=12)
        ax.set_xlabel("x — jaw axis (mm)"); ax.set_ylabel("z from the flange (mm)")
        ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.set_ylim(-15, 230)
        ax.legend(loc="upper left", fontsize=8.5, framealpha=0.95)

    # --- 3: plan cut at the worst flank -------------------------------------------
    ax3 = fig.add_subplot(gs[0, 2])
    draw(ax3, hull, [0, 0, 1], [0, 0, 89.0], 0, 1, lw=1.8, c="tab:red", ls="--")
    ax3.plot([], [], c="tab:red", lw=1.8, ls="--", label="before: one hull")
    for m, c, lab in zip(shell, SHELL_COLOR, SHELL_LABEL):
        draw(ax3, m, [0, 0, 1], [0, 0, 89.0], 0, 1, lw=1.8, c=c)
        ax3.plot([], [], c=c, lw=1.8, label=f"after: {lab}")
    draw(ax3, base, [0, 0, 1], [0, 0, 89.0], 0, 1, lw=1.2, c="0.15")
    ax3.plot([], [], c="0.15", lw=1.2, label="the actual part")
    ax3.annotate("", xy=(38.9, -19.4), xytext=(80.5, -19.4),
                 arrowprops=dict(arrowstyle="<->", color="tab:red", lw=1.8))
    ax3.text(60, -32, "44.5 mm", color="tab:red", fontsize=10, ha="center")
    ax3.set_title("XY cut at z = 89 — the flank\nthe other gripper's rail tip lands here",
                  fontsize=12)
    ax3.set_xlabel("x (mm)"); ax3.set_ylabel("y (mm)")
    ax3.set_aspect("equal"); ax3.grid(alpha=0.3)
    ax3.legend(loc="upper right", fontsize=8.5)

    # --- 4: a real cut through the recorded contact --------------------------------
    model = pin.buildModelFromUrdf(str(URDF))
    data = model.createData()
    fl = model.getFrameId("dual_rb5_850e_left_attachment_site")
    fr = model.getFrameId("dual_rb5_850e_right_attachment_site")
    pin.forwardKinematics(model, data, np.deg2rad(Q_LEFT + Q_RIGHT))
    pin.updateFramePlacements(model, data)
    TL, TR = data.oMf[fl], data.oMf[fr]

    def bvh(m):
        v = np.asarray(m.vertices, float) * 0.001
        f = np.asarray(m.faces, np.int32)
        vv = coal.StdVec_Vec3s(); vv.extend(list(v))
        tri = coal.Triangle32 if hasattr(coal, "Triangle32") else coal.Triangle
        tt = coal.StdVec_Triangle32() if hasattr(coal, "StdVec_Triangle32") else coal.StdVec_Triangle()
        tt.extend([tri(int(a), int(b), int(c)) for a, b, c in f])
        g = coal.BVHModelOBBRSS(); g.beginModel(len(f), len(v)); g.addSubModel(vv, tt); g.endModel()
        return g

    def dist(g1, T1, g2, T2):
        rq, rs = coal.DistanceRequest(), coal.DistanceResult()
        coal.distance(g1, coal.Transform3s(T1.rotation, T1.translation),
                      g2, coal.Transform3s(T2.rotation, T2.translation), rq, rs)
        return rs.min_distance, np.array(rs.getNearestPoint1()), np.array(rs.getNearestPoint2())

    g_true, g_hull = bvh(base), bvh(hull)
    g_shell = [bvh(m) for m in shell]
    d_true, w1, w2 = dist(g_true, TL, g_true, TR)
    d_hull = dist(g_hull, TL, g_hull, TR)[0]
    d_shell = min(dist(a, TL, b, TR)[0] for a in g_shell for b in g_shell)

    # Cut plane: through the witness midpoint, CONTAINING the gap direction, so the gap
    # in the picture is the gap the solver measured.
    n = w2 - w1
    n /= np.linalg.norm(n)
    up = np.array([0.0, 0.0, 1.0])
    pn = np.cross(n, up)
    pn /= np.linalg.norm(pn)
    e1, e2 = n, np.cross(pn, n)
    origin = 0.5 * (w1 + w2)

    def placed(mesh, T):
        m = mesh.copy()
        m.apply_scale(0.001)
        M = np.eye(4); M[:3, :3] = T.rotation; M[:3, 3] = T.translation
        m.apply_transform(M)
        return m

    sub = gs[0, 3].subgridspec(2, 1, height_ratios=[1.0, 1.15], hspace=0.28)
    ax4 = fig.add_subplot(sub[0, 0])
    ax5 = fig.add_subplot(sub[1, 0])

    def cut2d(ax, mesh, **kw):
        s = mesh.section(plane_origin=origin, plane_normal=pn)
        if s is None:
            return
        for p in s.discrete:
            p = np.asarray(p) - origin
            ax.plot(p @ e1 * 1000.0, p @ e2 * 1000.0, **kw)

    for ax in (ax4, ax5):
        for T in (TL, TR):
            cut2d(ax, placed(hull, T), lw=1.7, c="tab:red", ls="--")
            for m, c in zip(shell, SHELL_COLOR):
                cut2d(ax, placed(m, T), lw=1.7, c=c)
            cut2d(ax, placed(base, T), lw=1.1, c="0.15")
        for w in (w1, w2):
            q = w - origin
            ax.plot(q @ e1 * 1000.0, q @ e2 * 1000.0, "o", ms=7, mfc="none", mec="k", mew=1.6)
        ax.set_aspect("equal"); ax.grid(alpha=0.3)
    ax4.plot([], [], c="tab:red", lw=1.7, ls="--", label="before: one hull")
    ax4.plot([], [], c=SHELL_COLOR[1], lw=1.7, label="after: the three pieces")
    ax4.plot([], [], c="0.15", lw=1.1, label="the actual part")
    ax4.plot([], [], "o", ms=7, mfc="none", mec="k", mew=1.6, label="true witness points")
    ax4.set_title("A REAL cut through the recorded contact — both grippers\n"
                  "servo_log_20260917_154112, barrier braking on this pair", fontsize=12)
    ax4.set_xlim(-175, 175); ax4.set_ylim(-125, 125)
    ax4.set_ylabel("mm")
    ax4.legend(loc="lower left", fontsize=8)
    # the gap itself
    ax5.annotate("", xy=((w1 - origin) @ e1 * 1000.0, (w1 - origin) @ e2 * 1000.0),
                 xytext=((w2 - origin) @ e1 * 1000.0, (w2 - origin) @ e2 * 1000.0),
                 arrowprops=dict(arrowstyle="<->", color="k", lw=1.4))
    ax5.text(0.0, 3.4, f"{d_true*1000:.1f} mm of real gap", fontsize=10, ha="center",
             bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.9))
    ax5.set_xlim(-38, 38); ax5.set_ylim(-26, 26)
    ax5.set_title("…zoomed on the gap", fontsize=11)
    ax5.set_xlabel("mm along the gap direction"); ax5.set_ylabel("mm")
    ax5.text(0.015, 0.975,
             f"one hull reported   {d_hull*1000:5.2f} mm\n"
             f"three pieces        {d_shell*1000:5.2f} mm\n"
             f"the truth           {d_true*1000:5.2f} mm\n"
             f"force-covered floor  5.00 mm",
             transform=ax5.transAxes, va="top", ha="left", fontsize=9.5,
             family="monospace", bbox=dict(boxstyle="round", fc="white", ec="0.6"))

    fig.suptitle("pika gripper base collision shell — before / after the 2026-09-18 split "
                 "(no barrier margin changed)", fontsize=15)
    fig.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=105)
    print(f"wrote {args.out}")
    print(f"  recorded pose: one hull {d_hull*1000:.2f} mm, three pieces {d_shell*1000:.2f} mm, "
          f"truth {d_true*1000:.2f} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
