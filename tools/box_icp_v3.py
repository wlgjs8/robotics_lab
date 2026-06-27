#!/usr/bin/env python3
"""박스 탐지 v3 — stand-frame 후보 + known-model ICP (가려짐/노이즈 robust) 검증 프로토타입.

흐름:
  v2 후보 추출(평면-상대 band + 색 분할/제외 + 클러스터) -> 박스별 관측점(scene, stand)
  -> FPS 다운샘플(여기선 numpy; 프로덕션은 torch-GPU)
  -> ICP: source=scene(관측, 부분) -> target=open-tray 전체 모델@init
     (모든 관측점은 모델 대응이 존재 -> 가려짐에 robust). init pose는 band rect fit.
  -> 가려짐 검증: 후보의 한쪽 절반을 제거하고 ICP -> 전체-data pose와 비교.

open3d 없이 호스트(scipy cKDTree)로 검증. 결과 PNG + pose/Δ 콘솔.
실행: python3 tools/box_icp_v3.py
"""
from __future__ import annotations
import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from scipy.spatial import cKDTree  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import box_detect_v2 as v2  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
BOX_LONG, BOX_SHORT, BOX_H = v2.BOX_LONG, v2.BOX_SHORT, v2.BOX_H


def open_tray_model(wall=0.020, n=4000, seed=0):
    """open-top 트레이 표면 점 (박스 로컬: 중심 원점, z 위, 윗면 열림). box_detect와 동일."""
    rng = np.random.default_rng(seed)
    hx, hy, hz = BOX_LONG / 2, BOX_SHORT / 2, BOX_H / 2
    ix, iy = hx - wall, hy - wall
    floor_z = -hz + wall
    pts, k = [], n // 8
    def rect(xlo, xhi, ylo, yhi, z):
        pts.append(np.stack([rng.uniform(xlo, xhi, k), rng.uniform(ylo, yhi, k), np.full(k, z)], 1))
    def wx(x):
        pts.append(np.stack([np.full(k, x), rng.uniform(-hy, hy, k), rng.uniform(-hz, hz, k)], 1))
    def wy(y):
        pts.append(np.stack([rng.uniform(-hx, hx, k), np.full(k, y), rng.uniform(-hz, hz, k)], 1))
    wx(-hx); wx(hx); wy(-hy); wy(hy)              # 바깥 4벽
    rect(-hx, hx, -hy, hy, hz)                    # 윗 림
    rect(-ix, ix, -iy, iy, floor_z)               # 내부 바닥
    return np.concatenate(pts, 0)


def fps(pts, k, seed=0):
    """farthest point sampling (numpy 프로토타입; 프로덕션은 torch GPU)."""
    n = len(pts)
    if n <= k:
        return np.arange(n)
    rng = np.random.default_rng(seed)
    idx = np.empty(k, np.int64)
    idx[0] = rng.integers(n)
    d = np.full(n, np.inf)
    for i in range(1, k):
        d = np.minimum(d, np.sum((pts - pts[idx[i - 1]]) ** 2, 1))
        idx[i] = np.argmax(d)
    return idx


def init_pose(center_xy, yaw_deg, plane):
    """band fit -> 4x4 box->stand. up=평면normal, 바닥을 평면에 안착(z=plane+box_h/2)."""
    a, b, c = plane
    n = np.array([-a, -b, 1.0]); n /= np.linalg.norm(n)
    yaw = np.radians(yaw_deg)
    # 평면상에서 x축(긴변) 방향: stand XY의 yaw를 평면에 투영
    x0 = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    x = x0 - n * (x0 @ n); x /= np.linalg.norm(x)
    y = np.cross(n, x)
    cz = a * center_xy[0] + b * center_xy[1] + c   # 평면 z at center
    center = np.array([center_xy[0], center_xy[1], cz]) + n * (BOX_H / 2)
    T = np.eye(4); T[:3, :3] = np.column_stack([x, y, n]); T[:3, 3] = center
    return T


def icp(scene, model_world, iters=40, trim=0.8):
    """source=scene(부분 관측) -> target=model_world(전체). trimmed point-to-point Kabsch.
    반환 T_icp: T_icp@scene ≈ model_world."""
    tree = cKDTree(model_world)
    S = scene.copy()
    T = np.eye(4)
    for _ in range(iters):
        d, j = tree.query(S)
        thr = np.quantile(d, trim)
        m = d <= thr
        src, dst = S[m], model_world[j[m]]
        cs, cd = src.mean(0), dst.mean(0)
        H = (src - cs).T @ (dst - cd)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1] *= -1; R = Vt.T @ U.T
        t = cd - R @ cs
        S = (R @ S.T).T + t
        Tn = np.eye(4); Tn[:3, :3] = R; Tn[:3, 3] = t
        T = Tn @ T
        if np.linalg.norm(t) < 1e-5:
            break
    return T


def pose_from(T):
    c = T[:3, 3]
    yaw = np.degrees(np.arctan2(T[1, 0], T[0, 0]))
    return c, yaw


def refine(scene, T0, model_local, fps_k=1024):
    """scene(관측, stand) + init T0 -> ICP refined box pose (4x4 box->stand)."""
    idx = fps(scene, fps_k)
    S = scene[idx]
    Mw = (model_local @ T0[:3, :3].T) + T0[:3, 3]      # 모델@init (target)
    T_icp = icp(S, Mw)
    return np.linalg.inv(T_icp) @ T0, S                 # refined = inv(T_icp)@T0


def extract_candidates():
    """v2 파이프라인으로 green/gray 관측점(stand) + plane + init(center,yaw) 추출."""
    import cv2
    xyz, rgb = v2.grab_cloud()
    if xyz is None:
        return None
    T = np.load(v2.T_STAND_CAM); cam_xy = T[:2, 3]
    Ps = xyz @ T[:3, :3].T + T[:3, 3]
    roi = ((Ps[:, 0] > v2.ROI_X[0]) & (Ps[:, 0] < v2.ROI_X[1]) &
           (Ps[:, 1] > v2.ROI_Y[0]) & (Ps[:, 1] < v2.ROI_Y[1]))
    P, C = Ps[roi], rgb[roi]
    plane = v2.fit_floor_plane(P)
    a, b, c = plane
    h = P[:, 2] - (a * P[:, 0] + b * P[:, 1] + c)
    band = (h > v2.Z_LO) & (h < v2.Z_HI)
    Pb, Cb = P[band], C[band]
    hsv = cv2.cvtColor(Cb.reshape(-1, 1, 3).astype(np.uint8), cv2.COLOR_RGB2HSV).reshape(-1, 3)
    is_green = (hsv[:, 0] >= 35) & (hsv[:, 0] <= 88) & (hsv[:, 1] >= 60) & (hsv[:, 2] >= 40)
    out = {}
    # green
    if is_green.sum() > 300:
        Pg = Pb[is_green]; pl, comps = v2.cluster_xy(Pg[:, :2])
        if comps:
            mg = pl == comps[0]; sc = Pg[mg]
            ctr, foot, yaw, _ = v2.fit_box(sc[:, :2], cam_xy)
            out["green"] = (sc, init_pose(ctr, yaw, plane))
    # gray (green footprint 제외)
    keep = ~is_green
    if "green" in out:
        gc = out["green"][1][:2, 3]
        keep &= np.linalg.norm(Pb[:, :2] - gc, axis=1) > 0.5 * np.hypot(BOX_LONG, BOX_SHORT) * 0.92
    if keep.sum() > 300:
        Pk = Pb[keep]; pl, comps = v2.cluster_xy(Pk[:, :2])
        best = None
        for cid in comps[:6]:
            m = pl == cid
            if m.sum() < 300:
                continue
            if best is None or m.sum() > best[0]:
                best = (int(m.sum()), Pk[m])
        if best:
            sc = best[1]; ctr, foot, yaw, _ = v2.fit_box(sc[:, :2], cam_xy)
            out["gray"] = (sc, init_pose(ctr, yaw, plane))
    return out, cam_xy


def occlude(scene, frac=0.5):
    """카메라에서 먼 쪽(작은 Y) frac 만큼 제거해 가려짐 시뮬레이션."""
    thr = np.quantile(scene[:, 1], frac)
    return scene[scene[:, 1] > thr]


def main():
    res = extract_candidates()
    if res is None:
        print("[v3] NO CLOUD"); return 1
    cands, cam_xy = res
    model = open_tray_model()

    fig, ax = plt.subplots(1, 2, figsize=(15, 7))
    colors = {"green": "green", "gray": "dimgray"}
    print("[v3] === ICP refine (full vs 50% occluded) ===  known 0.380x0.240x0.110")
    for name, (scene, T0) in cands.items():
        T_full, Sf = refine(scene, T0, model)
        scene_occ = occlude(scene, 0.5)
        T_occ, _ = refine(scene_occ, T0, model)
        c0, y0 = pose_from(T0)
        cf, yf = pose_from(T_full)
        co, yo = pose_from(T_occ)
        dpos = np.linalg.norm((cf - co)[:2]) * 1000
        dyaw = abs((yf - yo + 180) % 360 - 180)
        print(f"  [{name:5s}] n={len(scene):6d}")
        print(f"     init  center=({c0[0]:+.3f},{c0[1]:+.3f}) yaw={y0:+.1f}")
        print(f"     ICP full   center=({cf[0]:+.3f},{cf[1]:+.3f},{cf[2]:+.3f}) yaw={yf:+.1f}")
        print(f"     ICP occ50% center=({co[0]:+.3f},{co[1]:+.3f},{co[2]:+.3f}) yaw={yo:+.1f}")
        print(f"     >> Δ(full vs occluded): pos={dpos:.1f}mm  yaw={dyaw:.2f}deg")
        # render top-down: scene + model@refined(full) + model@refined(occ)
        sub = scene[np.random.default_rng(0).choice(len(scene), min(8000, len(scene)), replace=False)]
        ax[0].scatter(sub[:, 0], sub[:, 1], s=2, c=colors[name], alpha=0.3)
        for T, ls, lbl in ((T_full, "-", "full"), (T_occ, "--", "occ50%")):
            Mw = (model[::8] @ T[:3, :3].T) + T[:3, 3]
            ax[0].plot(Mw[:, 0], Mw[:, 1], ls, lw=0.6, alpha=0.0)  # placeholder
            # draw box rectangle (top rim) for clarity
            hw, hh = BOX_LONG / 2, BOX_SHORT / 2
            corn = np.array([[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh], [-hw, -hh]])
            w = (corn @ T[:2, :2].T) + T[:2, 3]
            ax[0].plot(w[:, 0], w[:, 1], ls, color=colors[name], lw=2,
                       label=f"{name} {lbl}")
        # side view (occlusion illustration)
        ax[1].scatter(scene[:, 1], scene[:, 2], s=2, c=colors[name], alpha=0.2)
        ax[1].axvline(np.quantile(scene[:, 1], 0.5), color=colors[name], ls=":", lw=1)
    ax[0].plot(*cam_xy, "r*", ms=14); ax[0].set_title("top-down: scene + ICP box (full vs occ50%)")
    ax[0].legend(fontsize=8); ax[0].set_aspect("equal"); ax[0].grid(alpha=0.3)
    ax[0].set_xlabel("stand X"); ax[0].set_ylabel("stand Y")
    ax[1].set_title("side view (dotted = occlusion cut at median Y)")
    ax[1].set_xlabel("stand Y"); ax[1].set_ylabel("stand Z"); ax[1].grid(alpha=0.3)
    out = os.path.join(ROOT, "head_box_icp_v3.png")
    fig.tight_layout(); fig.savefig(out, dpi=110)
    print(f"\n[v3] saved {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
