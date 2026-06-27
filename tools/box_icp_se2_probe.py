#!/usr/bin/env python3
"""아이디어 1 검증: yaw-고정(SE(2)) ICP vs full-6DOF.

박스는 User Floor(수평) 위 → box up = stand (0,0,1) 고정. ICP는 (x,y,yaw)만 풀어
부분관측/노이즈에 더 robust한지, full-6DOF(box_icp_v3) 대비 가림 robust성을 비교한다.
호스트에서 scipy만으로 동작(open3d/torch 불필요).

실행: python3 tools/box_icp_se2_probe.py
"""
from __future__ import annotations
import os, sys, time
import numpy as np
from scipy.spatial import cKDTree
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import box_detect_v2 as v2
from box_icp_v3 import open_tray_model, fps, occlude  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))


def init_pose_z(center_xy, yaw_deg, floor_z):
    """up=stand z 고정 init (yaw만). 바닥 z=floor_z, center z=floor_z+box_h/2."""
    yaw = np.radians(yaw_deg)
    x = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    y = np.array([-np.sin(yaw), np.cos(yaw), 0.0])
    z = np.array([0.0, 0.0, 1.0])
    T = np.eye(4); T[:3, :3] = np.column_stack([x, y, z])
    T[:3, 3] = [center_xy[0], center_xy[1], floor_z + v2.BOX_H / 2]
    return T


def icp_se2(scene, T0, model, iters=40, trim=0.8):
    """source=scene -> target=model@T0. SE(2)만(z·up 고정). refined=inv(T_icp)@T0."""
    Mw = model @ T0[:3, :3].T + T0[:3, 3]
    tree = cKDTree(Mw)
    S = scene.copy()
    Tacc = np.eye(4)
    for _ in range(iters):
        d, j = tree.query(S)
        thr = np.quantile(d, trim); m = d <= thr
        P = S[m, :2] - S[m, :2].mean(0); Q = Mw[j[m], :2] - Mw[j[m], :2].mean(0)
        H = P.T @ Q
        phi = np.arctan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
        Rz = np.array([[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]])
        t2 = Mw[j[m], :2].mean(0) - Rz @ S[m, :2].mean(0)
        S[:, :2] = S[:, :2] @ Rz.T + t2                    # z 불변
        Tn = np.eye(4); Tn[:2, :2] = Rz; Tn[:2, 3] = t2
        Tacc = Tn @ Tacc
        if abs(phi) < 1e-5 and np.linalg.norm(t2) < 1e-5:
            break
    return np.linalg.inv(Tacc) @ T0


def main():
    import cv2
    xyz, rgb = v2.grab_cloud()
    if xyz is None:
        print("[se2] NO CLOUD"); return 1
    T = np.load(v2.T_STAND_CAM); cam_xy = T[:2, 3]
    Ps = xyz @ T[:3, :3].T + T[:3, 3]
    roi = ((Ps[:, 0] > v2.ROI_X[0]) & (Ps[:, 0] < v2.ROI_X[1]) &
           (Ps[:, 1] > v2.ROI_Y[0]) & (Ps[:, 1] < v2.ROI_Y[1]))
    P, C = Ps[roi], rgb[roi]
    a, b, c = v2.fit_floor_plane(P)
    h = P[:, 2] - (a * P[:, 0] + b * P[:, 1] + c)
    band = (h > v2.Z_LO) & (h < v2.Z_HI)
    Pb, Cb = P[band], C[band]
    hsv = cv2.cvtColor(np.ascontiguousarray(Cb).reshape(-1, 1, 3), cv2.COLOR_RGB2HSV).reshape(-1, 3)
    is_green = np.all((hsv >= np.array([35, 60, 40])) & (hsv <= np.array([88, 255, 255])), axis=1)
    model = open_tray_model()

    def candidate(mask, label):
        pl, comps = v2.cluster_xy(Pb[mask][:, :2])
        if not comps:
            return None
        sc = Pb[mask][pl == comps[0]]
        ctr, foot, yaw, _ = v2.fit_box(sc[:, :2], cam_xy)
        floor_z = a * ctr[0] + b * ctr[1] + c
        return sc, init_pose_z(ctr, yaw, floor_z)

    cands = {}
    cg = candidate(is_green, "green")
    if cg:
        cands["green"] = cg
    keep = ~is_green
    if cg:
        Tg = cg[1]; d = Pb[:, :2] - Tg[:2, 3]
        xl = d @ Tg[:2, 0]; yl = d @ Tg[:2, 1]
        keep &= ~((np.abs(xl) < v2.BOX_LONG/2 + 0.04) & (np.abs(yl) < v2.BOX_SHORT/2 + 0.04))
    ck = candidate(keep, "gray")
    if ck:
        cands["gray"] = ck

    print("[se2] === yaw-고정 ICP (full vs 50% 가림) ===  up=stand(0,0,1)")
    for name, (scene, T0) in cands.items():
        s = scene[fps(scene, 1024)]
        Tf = icp_se2(s, T0, model)
        so = occlude(scene, 0.5); To = icp_se2(so[fps(so, 1024)], T0, model)
        cf, co = Tf[:3, 3], To[:3, 3]
        yf = np.degrees(np.arctan2(Tf[1, 0], Tf[0, 0])); yo = np.degrees(np.arctan2(To[1, 0], To[0, 0]))
        # up축이 정확히 (0,0,1)인지(틀어지면 안 됨)
        up_dev = np.degrees(np.arccos(np.clip(Tf[2, 2], -1, 1)))
        dpos = np.linalg.norm((cf - co)[:2]) * 1000
        dyaw = abs((yf - yo + 180) % 360 - 180)
        print(f"  [{name:5s}] n={len(scene):6d}")
        print(f"     full   center=({cf[0]:+.3f},{cf[1]:+.3f},{cf[2]:+.3f}) yaw={yf:+.1f} up_dev={up_dev:.2f}deg")
        print(f"     occ50% center=({co[0]:+.3f},{co[1]:+.3f},{co[2]:+.3f}) yaw={yo:+.1f}")
        print(f"     >> Δ(full vs occ): pos={dpos:.1f}mm  yaw={dyaw:.2f}deg")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
