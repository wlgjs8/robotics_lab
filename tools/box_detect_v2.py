#!/usr/bin/env python3
"""Stand-frame 박스 탐지 v2 — 개선된 클라우드(emitter ON, 1280x720) 기준 프로토타입.

파이프라인(색을 "찾기"가 아니라 "나누기/제외"에만 사용):
  cloud(cam) --T_stand_cam--> stand
  --> 테이블 평면 lstsq(저높이 점) -> 평면-상대 높이 h  (tilt 무관, 깨끗한 band)
  --> band [Z_LO, BOX_H+pad] & XY ROI  -> 후보
  --> GREEN 박스: 색(HSV green) 점만 클러스터 -> 최대 성분 -> 알려진치수 rect fit
  --> GRAY  박스: 후보에서 GREEN footprint 영역을 제외 -> 클러스터 -> 0.38x0.24에
       가장 맞는 성분 -> fit  (초록 박스 내부 비초록점/그리퍼 잡음 제거)
  --> 두 박스 모두 known-dims + 가림 보정(관측<치수면 카메라 반대로 중심 이동)

출력: head_box_detect_v2.png (top-down: 후보 RGB + fit 박스/중심/라벨) + 콘솔 pose.
실행: python3 tools/box_detect_v2.py
"""
from __future__ import annotations
import json
import os
import time
import numpy as np
import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
CLOUD_EP = os.environ.get("CLOUD_SUB_ENDPOINT", "tcp://127.0.0.1:5601")
T_STAND_CAM = os.environ.get("RB_GUI_T_STAND_CAM", "/home/plaif/workspace/T_stand_cam.npy")

BOX_LONG, BOX_SHORT, BOX_H = 0.380, 0.240, 0.110
Z_LO, Z_HI = 0.012, 0.125            # 평면-상대 높이 band (m)
ROI_X, ROI_Y = (-0.55, 0.55), (-1.05, 0.05)
GRID = 0.006                         # XY occupancy 해상도 (m/cell)
MIN_CLUSTER_CELL = 200


def grab_cloud(n_accum=2, timeout_s=10.0):
    import zmq
    s = zmq.Context.instance().socket(zmq.SUB)
    s.setsockopt_string(zmq.SUBSCRIBE, "stereo.cloud")
    s.setsockopt(zmq.RCVHWM, 4); s.setsockopt(zmq.CONFLATE, 0); s.connect(CLOUD_EP)
    xs, cs, t0 = [], [], time.time()
    while len(xs) < n_accum and time.time() - t0 < timeout_s:
        if not s.poll(500):
            continue
        p = s.recv_multipart()
        if p[0] != b"stereo.cloud":
            continue
        xs.append(np.frombuffer(p[2], np.float32).reshape(-1, 3).astype(np.float64))
        cs.append(np.frombuffer(p[3], np.uint8).reshape(-1, 3))
    s.close()
    return (np.concatenate(xs), np.concatenate(cs)) if xs else (None, None)


def fit_floor_plane(P):
    """저높이 점에 z=ax+by+c lstsq -> (a,b,c). 평면-상대 높이 = z-(ax+by+c)."""
    z0 = np.percentile(P[:, 2], 5)
    tab = P[P[:, 2] < z0 + 0.02]
    A = np.c_[tab[:, 0], tab[:, 1], np.ones(len(tab))]
    coef, *_ = np.linalg.lstsq(A, tab[:, 2], rcond=None)
    return coef


def cluster_xy(P_xy):
    """XY occupancy grid -> 연결성분. (라벨배열, 성분수). 면적 내림차순."""
    gx = ((P_xy[:, 0] - ROI_X[0]) / GRID).astype(np.int32)
    gy = ((P_xy[:, 1] - ROI_Y[0]) / GRID).astype(np.int32)
    W = int((ROI_X[1] - ROI_X[0]) / GRID) + 2
    H = int((ROI_Y[1] - ROI_Y[0]) / GRID) + 2
    inb = (gx >= 0) & (gx < W) & (gy >= 0) & (gy < H)
    occ = np.zeros((H, W), np.uint8)
    occ[gy[inb], gx[inb]] = 255
    occ = cv2.morphologyEx(occ, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    occ = cv2.morphologyEx(occ, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    ncc, lab, stats, _ = cv2.connectedComponentsWithStats(occ, 8)
    order = sorted(range(1, ncc), key=lambda c: -stats[c, cv2.CC_STAT_AREA])
    pl = np.where(inb, lab[np.clip(gy, 0, H - 1), np.clip(gx, 0, W - 1)], 0)
    return pl, [c for c in order if stats[c, cv2.CC_STAT_AREA] >= MIN_CLUSTER_CELL]


def fit_box(P_xy, cam_xy):
    """평면상 minAreaRect -> 알려진치수 + 가림 보정. (center, (obs_long,obs_short), yaw, box4)."""
    rect = cv2.minAreaRect((P_xy * 1000).astype(np.float32))
    (cx, cy), (w, h), ang = rect
    cx, cy, w, h = cx / 1000, cy / 1000, w / 1000, h / 1000
    if w >= h:
        obs_long, obs_short, yaw = w, h, np.radians(ang)
    else:
        obs_long, obs_short, yaw = h, w, np.radians(ang + 90)
    xax = np.array([np.cos(yaw), np.sin(yaw)])      # 긴 변(0.38) 방향
    yax = np.array([-np.sin(yaw), np.cos(yaw)])     # 짧은 변(0.24)
    center = np.array([cx, cy])
    # 가림 보정: 관측 길이가 치수보다 짧으면 카메라 반대(가려진) 쪽으로 중심 이동
    for ax, obs, full in ((xax, obs_long, BOX_LONG), (yax, obs_short, BOX_SHORT)):
        push = max(0.0, (full - obs) / 2.0)
        if push > 1e-4:
            away = ax if ax @ (center - cam_xy) > 0 else -ax
            center = center + away * push
    # 보정된 known-dims 사각형 4점
    hw, hh = BOX_LONG / 2, BOX_SHORT / 2
    corners = np.array([[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]])
    box4 = center + corners @ np.stack([xax, yax])
    return center, (obs_long, obs_short), float(np.degrees(yaw)), box4


def main():
    xyz, rgb = grab_cloud()
    if xyz is None:
        print("[v2] NO CLOUD on 5601"); return 1
    T = np.load(T_STAND_CAM)
    cam_xy = T[:2, 3]
    Ps = xyz @ T[:3, :3].T + T[:3, 3]
    roi = ((Ps[:, 0] > ROI_X[0]) & (Ps[:, 0] < ROI_X[1]) &
           (Ps[:, 1] > ROI_Y[0]) & (Ps[:, 1] < ROI_Y[1]))
    P, C = Ps[roi], rgb[roi]
    a, b, c = fit_floor_plane(P)
    h = P[:, 2] - (a * P[:, 0] + b * P[:, 1] + c)
    band = (h > Z_LO) & (h < Z_HI)
    Pb, Cb, hb = P[band], C[band], h[band]
    print(f"[v2] cloud={len(xyz)} band_candidates={int(band.sum())} "
          f"floor_tilt={np.degrees(np.arccos(1/np.sqrt(1+a*a+b*b))):.2f}deg")

    hsv = cv2.cvtColor(Cb.reshape(-1, 1, 3).astype(np.uint8), cv2.COLOR_RGB2HSV).reshape(-1, 3)
    is_green = (hsv[:, 0] >= 35) & (hsv[:, 0] <= 88) & (hsv[:, 1] >= 60) & (hsv[:, 2] >= 40)

    boxes = []
    # --- GREEN: 색 점 클러스터 -> 최대 성분 ---
    green_center = None
    if is_green.sum() > 300:
        Pg = Pb[is_green]
        pl, comps = cluster_xy(Pg[:, :2])
        if comps:
            m = pl == comps[0]
            center, foot, yaw, box4 = fit_box(Pg[m][:, :2], cam_xy)
            green_center = center
            boxes.append(dict(label="green", n=int(m.sum()), center=center, foot=foot,
                              yaw=yaw, box4=box4,
                              z=float(np.percentile(hb[is_green][m], 95))))

    # --- GRAY: GREEN footprint 제외 후 클러스터 -> 0.38x0.24에 가장 맞는 성분 ---
    keep = ~is_green
    if green_center is not None:
        d = np.linalg.norm(Pb[:, :2] - green_center, axis=1)
        keep &= d > (0.5 * np.hypot(BOX_LONG, BOX_SHORT) * 0.92)   # 초록 박스 반경 밖
    if keep.sum() > 300:
        Pk = Pb[keep]
        pl, comps = cluster_xy(Pk[:, :2])
        best = None
        for cid in comps[:6]:
            m = pl == cid
            if m.sum() < 300:
                continue
            center, foot, yaw, box4 = fit_box(Pk[m][:, :2], cam_xy)
            # 0.38x0.24에 얼마나 맞는지(관측 long/short가 치수 이하·근접일수록 good)
            err = abs(min(foot[0], BOX_LONG) - foot[0]) + abs(min(foot[1], BOX_SHORT) - foot[1])
            score = (m.sum(), -err)
            if best is None or score > best[0]:
                best = (score, dict(label="gray", n=int(m.sum()), center=center, foot=foot,
                                    yaw=yaw, box4=box4, z=float(np.percentile(hb[keep][m], 95))))
        if best:
            boxes.append(best[1])

    print("\n[v2] === 박스 pose (stand frame) ===  known 0.380x0.240x0.110")
    for bx in boxes:
        print(f"  [{bx['label']:5s}] center=({bx['center'][0]:+.3f},{bx['center'][1]:+.3f}) "
              f"yaw={bx['yaw']:+.1f}deg  obs_footprint={bx['foot'][0]:.3f}x{bx['foot'][1]:.3f}m "
              f"top_h={bx['z']:.3f}m  n={bx['n']}")

    # --- render ---
    rng = np.random.default_rng(0)
    def sub(n, cap=45000):
        return rng.choice(n, cap, replace=False) if n > cap else np.arange(n)
    fig, ax = plt.subplots(1, 2, figsize=(16, 7))
    si = sub(len(P))
    ax[0].scatter(P[si, 0], P[si, 1], c=h[si], s=1, cmap="turbo", vmin=-0.02, vmax=0.20)
    ax[0].set_title("ROI top-down (color=plane-relative height)")
    bi = sub(len(Pb))
    ax[1].scatter(Pb[bi, 0], Pb[bi, 1], c=Cb[bi] / 255.0, s=2)
    for bx in boxes:
        col = "lime" if bx["label"] == "green" else "deepskyblue"
        poly = np.vstack([bx["box4"], bx["box4"][0]])
        ax[1].plot(poly[:, 0], poly[:, 1], color=col, lw=2.2)
        ax[1].plot(*bx["center"], "+", color=col, ms=16, mew=3)
        ax[1].text(bx["center"][0], bx["center"][1] + 0.03, bx["label"],
                   color=col, fontsize=12, ha="center", weight="bold")
    ax[1].plot(*cam_xy, "r*", ms=14); ax[1].text(cam_xy[0], cam_xy[1], " cam", color="r")
    ax[1].set_title(f"band [{Z_LO*1000:.0f}..{Z_HI*1000:.0f}mm] + known-dims fit (가림보정)")
    for a_ in ax:
        a_.set_xlabel("stand X (m)"); a_.set_ylabel("stand Y (m)")
        a_.set_aspect("equal"); a_.grid(alpha=0.3)
    out = os.path.join(ROOT, "head_box_detect_v2.png")
    fig.tight_layout(); fig.savefig(out, dpi=110)
    print(f"\n[v2] saved {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
