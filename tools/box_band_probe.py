#!/usr/bin/env python3
"""Stand-frame height-band 박스 후보 추출 검증 (라이브 stereo.cloud).

가설(사용자 제안): 픽셀 색 대신 **기하**로 후보를 좁힌다.
  camera-frame cloud --T_stand_cam--> stand-frame
  --> stand floor(테이블 평면) 추정
  --> band [floor+Z_LO, floor+box_height] 크롭   (테이블 아래 / 로봇 하드웨어 위 제거)
  --> XY 평면 클러스터링                          (회색/초록 박스 = 2 클러스터)
  --> 클러스터별 minAreaRect footprint + 색은 라벨링(green/gray)에만 사용

색 임계 튜닝 없이 동작 → 그림자/부분 가림에 robust 한지 평가.
출력: head_box_band_topdown.png (full top-down / band 후보 / 측면도 3패널) + 콘솔 리포트.

실행(repo 루트): python3 tools/box_band_probe.py
"""
from __future__ import annotations
import json
import os
import sys
import time
import numpy as np
import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.realpath(__file__))
ROOT = os.path.dirname(HERE)

CLOUD_EP = os.environ.get("CLOUD_SUB_ENDPOINT", "tcp://127.0.0.1:5601")
CLOUD_TOPIC = "stereo.cloud"
T_STAND_CAM = os.environ.get("RB_GUI_T_STAND_CAM", "/home/plaif/workspace/T_stand_cam.npy")

BOX_H = 0.110            # 박스 높이 (box_detect.BOX_DIMS[2])
BOX_DIMS_XY = (0.380, 0.240)
Z_LO = 0.005            # floor 위 5mm 부터 (테이블 노이즈 제외)
Z_HI_PAD = 0.015        # floor+box_height 위로 살짝 여유(부분점/캘리브 오차)
# stand-frame XY ROI (box_detect.ROI 기반)
ROI_X = (-0.55, 0.55)
ROI_Y = (-1.05, 0.05)
GRID = 0.006           # 클러스터용 occupancy grid 해상도 (m/cell)
MIN_CLUSTER_PX = 120   # grid cell 수 게이트


def grab_cloud(n_accum=1, timeout_s=10.0):
    import zmq
    ctx = zmq.Context.instance()
    s = ctx.socket(zmq.SUB)
    s.setsockopt_string(zmq.SUBSCRIBE, CLOUD_TOPIC)
    s.setsockopt(zmq.RCVHWM, 4)
    s.setsockopt(zmq.CONFLATE, 0)
    s.connect(CLOUD_EP)
    xs, cs = [], []
    t0 = time.time()
    while len(xs) < n_accum and time.time() - t0 < timeout_s:
        if not s.poll(500):
            continue
        parts = s.recv_multipart()
        if parts[0] != CLOUD_TOPIC.encode():
            continue
        hdr = json.loads(parts[1])
        xyz = np.frombuffer(parts[2], np.float32).reshape(-1, 3)
        rgb = np.frombuffer(parts[3], np.uint8).reshape(-1, 3)
        xs.append(xyz.astype(np.float64))
        cs.append(rgb)
    s.close()
    if not xs:
        return None, None
    return np.concatenate(xs, 0), np.concatenate(cs, 0)


def estimate_floor_z(Ps, roi):
    """ROI 안에서 z 히스토그램 최빈값 = 테이블 평면 z (수평 가정)."""
    z = Ps[roi, 2]
    h, edges = np.histogram(z, bins=120, range=(float(z.min()), float(np.percentile(z, 99))))
    k = int(np.argmax(h))
    return 0.5 * (edges[k] + edges[k + 1])


def cluster_xy(P_band):
    """band 점을 stand XY occupancy grid로 래스터 -> 연결성분. 점→클러스터 라벨 반환."""
    x, y = P_band[:, 0], P_band[:, 1]
    x0, y0 = ROI_X[0], ROI_Y[0]
    gx = ((x - x0) / GRID).astype(np.int32)
    gy = ((y - y0) / GRID).astype(np.int32)
    W = int((ROI_X[1] - ROI_X[0]) / GRID) + 2
    H = int((ROI_Y[1] - ROI_Y[0]) / GRID) + 2
    occ = np.zeros((H, W), np.uint8)
    inb = (gx >= 0) & (gx < W) & (gy >= 0) & (gy < H)
    occ[gy[inb], gx[inb]] = 255
    occ = cv2.morphologyEx(occ, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    occ = cv2.morphologyEx(occ, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    ncc, lab, stats, _ = cv2.connectedComponentsWithStats(occ, 8)
    keep = [cid for cid in range(1, ncc) if stats[cid, cv2.CC_STAT_AREA] >= MIN_CLUSTER_PX]
    keep.sort(key=lambda c: -stats[c, cv2.CC_STAT_AREA])
    pt_lab = np.full(len(P_band), -1, np.int32)
    pl = np.where(inb, lab[np.clip(gy, 0, H - 1), np.clip(gx, 0, W - 1)], 0)
    for new_id, cid in enumerate(keep):
        pt_lab[pl == cid] = new_id
    return pt_lab, len(keep)


def fit_rect(P_xy):
    rect = cv2.minAreaRect((P_xy * 1000).astype(np.float32))
    (cx, cy), (w, h), ang = rect
    cx, cy, w, h = cx / 1000, cy / 1000, w / 1000, h / 1000
    long_, short_ = max(w, h), min(w, h)
    box = cv2.boxPoints(((cx * 1000, cy * 1000), (w * 1000, h * 1000), ang)) / 1000
    return (cx, cy), (long_, short_), ang, box


def label_color(rgb_cluster):
    """클러스터 평균색으로 green/gray 라벨 (색은 라벨링에만)."""
    hsv = cv2.cvtColor(rgb_cluster.reshape(-1, 1, 3).astype(np.uint8), cv2.COLOR_RGB2HSV).reshape(-1, 3)
    green_frac = float(((hsv[:, 0] >= 35) & (hsv[:, 0] <= 88) & (hsv[:, 1] >= 60)).mean())
    return ("green" if green_frac > 0.15 else "gray"), green_frac


def main():
    print(f"[band] subscribing {CLOUD_EP} ({CLOUD_TOPIC}) ...", flush=True)
    xyz, rgb = grab_cloud(n_accum=int(os.environ.get("PROBE_ACCUM", "2")))
    if xyz is None:
        print("[band] NO CLOUD — stereo_worker publish 중인지 확인", flush=True)
        return 1
    T = np.load(T_STAND_CAM)
    Ps = (xyz @ T[:3, :3].T) + T[:3, 3]
    roi = ((Ps[:, 0] > ROI_X[0]) & (Ps[:, 0] < ROI_X[1]) &
           (Ps[:, 1] > ROI_Y[0]) & (Ps[:, 1] < ROI_Y[1]))
    floor = estimate_floor_z(Ps, roi)
    z_lo, z_hi = floor + Z_LO, floor + BOX_H + Z_HI_PAD
    band = roi & (Ps[:, 2] > z_lo) & (Ps[:, 2] < z_hi)
    Pb, Cb = Ps[band], rgb[band]
    print(f"[band] cloud n={len(xyz)}  floor_z={floor:.3f}  band=[{z_lo:.3f},{z_hi:.3f}]  "
          f"candidate_pts={int(band.sum())} ({100*band.mean():.1f}% of cloud)", flush=True)

    # 색은 band 안에서 초록/회색을 "나누는" 라벨로만 사용 (박스를 찾는 게 아님).
    hsv = cv2.cvtColor(Cb.reshape(-1, 1, 3).astype(np.uint8), cv2.COLOR_RGB2HSV).reshape(-1, 3)
    is_green = (hsv[:, 0] >= 35) & (hsv[:, 0] <= 88) & (hsv[:, 1] >= 60) & (hsv[:, 2] >= 40)
    print(f"[band] green pts in band: {int(is_green.sum())} / {len(Pb)} "
          f"({100*is_green.mean():.1f}%)", flush=True)

    # ---- 색 분할 후 각 그룹을 따로 클러스터+fit ----
    clusters = []
    for label, sel, col in (("green", is_green, "lime"), ("gray", ~is_green, "deepskyblue")):
        if sel.sum() < 200:
            continue
        Pg, Cg = Pb[sel], Cb[sel]
        sub_lab, nsub = cluster_xy(Pg)
        for cid in range(nsub):
            m = sub_lab == cid
            if m.sum() < 200:
                continue
            Pc, Cc = Pg[m], Cg[m]
            (cx, cy), (lng, sht), ang, box = fit_rect(Pc[:, :2])
            _, gf = label_color(Cc)
            clusters.append(dict(n=int(m.sum()), center=(cx, cy), foot=(lng, sht),
                                 ang=ang, box=box, label=label, gf=gf, plotcol=col,
                                 z_top=float(np.percentile(Pc[:, 2], 95)) - floor))
    clusters.sort(key=lambda c: -c["n"])

    print("\n[band] === 클러스터 (stand frame) ===")
    print(f"  known box footprint = {BOX_DIMS_XY[0]:.3f} x {BOX_DIMS_XY[1]:.3f} m, height {BOX_H:.3f} m")
    for c in clusters:
        lng, sht = c["foot"]
        print(f"  [{c['label']:5s}] n={c['n']:6d}  center=({c['center'][0]:+.3f},{c['center'][1]:+.3f})  "
              f"footprint={lng:.3f}x{sht:.3f}m  height~{c['z_top']:.3f}m  green_frac={c['gf']:.2f}")

    # ---- 시각화 ----
    fig, ax = plt.subplots(1, 3, figsize=(21, 7))
    rng = np.random.default_rng(0)
    def _sub(n, cap=40000):
        return rng.choice(n, cap, replace=False) if n > cap else np.arange(n)
    # (1) full cloud top-down, height-colored
    Pr = Ps[roi]
    si = _sub(len(Pr))
    sc = ax[0].scatter(Pr[si, 0], Pr[si, 1], c=Pr[si, 2] - floor, s=1,
                       cmap="turbo", vmin=-0.02, vmax=0.35)
    ax[0].set_title("full cloud top-down (color=height above floor)")
    plt.colorbar(sc, ax=ax[0], fraction=0.046)
    # (2) band candidates, true RGB + cluster rects
    bi = _sub(len(Pb))
    ax[1].scatter(Pb[bi, 0], Pb[bi, 1], c=Cb[bi] / 255.0, s=2)
    for c in clusters:
        box = np.vstack([c["box"], c["box"][0]])
        col = c["plotcol"]
        ax[1].plot(box[:, 0], box[:, 1], color=col, lw=2)
        ax[1].plot(*c["center"], "+", color=col, ms=14, mew=3)
        ax[1].text(c["center"][0], c["center"][1], f" {c['label']}\n {c['foot'][0]:.2f}x{c['foot'][1]:.2f}",
                   color=col, fontsize=10, va="center")
    ax[1].set_title(f"band [floor+{Z_LO*1000:.0f}mm .. +{(BOX_H+Z_HI_PAD)*1000:.0f}mm]  candidates (true RGB)")
    # (3) side view Y-Z with band lines
    Cr = rgb[roi]
    ax[2].scatter(Pr[si, 1], Pr[si, 2] - floor, c=Cr[si] / 255.0, s=1)
    ax[2].axhline(Z_LO, color="r", ls="--", lw=1); ax[2].axhline(BOX_H + Z_HI_PAD, color="r", ls="--", lw=1)
    ax[2].axhline(0.0, color="k", ls=":", lw=1)
    ax[2].set_title("side view (Y vs height); red dashed = band, dotted = floor")
    for a in (ax[0], ax[1]):
        a.set_xlabel("stand X (m)"); a.set_ylabel("stand Y (m)"); a.set_aspect("equal"); a.grid(alpha=0.3)
    ax[2].set_xlabel("stand Y (m)"); ax[2].set_ylabel("height above floor (m)"); ax[2].grid(alpha=0.3)
    out = os.path.join(ROOT, "head_box_band_topdown.png")
    fig.tight_layout(); fig.savefig(out, dpi=110)
    print(f"\n[band] saved {out}", flush=True)
    return 0


def Cb_color(P, rgb):
    return rgb / 255.0


if __name__ == "__main__":
    sys.exit(main())
