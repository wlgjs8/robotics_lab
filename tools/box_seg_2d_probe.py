#!/usr/bin/env python3
"""Head color 프레임에서 2D 박스 색 세그멘테이션 성능 진단 (라이브 번들).

camera_server 번들(camera.bundle ZMQ + /dev/shm 링)에서 head.color(1280x720 RGB)를
한 프레임 받아 box_detect.BoxDetector 의 HSV 임계값(초록/회색)을 **raw color에 직접** 적용.
depth/평면 게이트 없이 순수 2D 색 분할만 평가 → 다음 PNG 저장:

  head_rgb_raw.png          : raw color (BGR로 저장)
  head_box_segmentation.png : 초록/회색 마스크 오버레이 + 최대 연결성분 박스/minAreaRect

실행: PYTHONPATH 없이 그냥 `python3 tools/box_seg_2d_probe.py` (repo 루트에서).
"""
from __future__ import annotations
import os
import sys
import time
import numpy as np
import cv2

HERE = os.path.dirname(os.path.realpath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "camera_server", "stereo_worker"))

from bundle_reader import BundleReader  # noqa: E402
from box_detect import BoxDetector  # noqa: E402 (임계값/상수 재사용)

OUT_DIR = ROOT
BUNDLE = os.environ.get("CAMERA_BUNDLE_ENDPOINT", "tcp://127.0.0.1:5600")
COLOR_KEY = os.environ.get("PROBE_COLOR_KEY", "head.color")


def grab_color(timeout_s=10.0):
    r = BundleReader(endpoint=BUNDLE)
    want = {COLOR_KEY}
    t0 = time.time()
    try:
        while time.time() - t0 < timeout_s:
            fr = r.poll(want, timeout_ms=500)
            f = fr.get(COLOR_KEY)
            if f is not None:
                return f
    finally:
        r.close()
    return None


def largest_components(mask, min_area):
    """morph-open 후 (box_detect.py와 동일) 면적 내림차순 연결성분 리스트."""
    m = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    ncc, lab, stats, cent = cv2.connectedComponentsWithStats(m, 8)
    comps = []
    for cid in range(1, ncc):
        area = int(stats[cid, cv2.CC_STAT_AREA])
        if area < min_area:
            continue
        x, y, w, h = (int(stats[cid, cv2.CC_STAT_LEFT]), int(stats[cid, cv2.CC_STAT_TOP]),
                      int(stats[cid, cv2.CC_STAT_WIDTH]), int(stats[cid, cv2.CC_STAT_HEIGHT]))
        comps.append(dict(cid=cid, area=area, bbox=(x, y, w, h),
                          centroid=(float(cent[cid][0]), float(cent[cid][1])), lab=lab))
    comps.sort(key=lambda c: -c["area"])
    return m, comps


def main():
    f = grab_color()
    if f is None:
        print(f"[probe] {COLOR_KEY} 프레임 수신 실패 — camera_server publish 중인지 확인", flush=True)
        return 1
    rgb = f.pixels  # HxWx3 RGB uint8
    H, W = rgb.shape[:2]
    print(f"[probe] got {COLOR_KEY} {W}x{H} fn={f.frame_number}", flush=True)

    # 1) raw 저장 (cv2는 BGR 기대)
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    raw_path = os.path.join(OUT_DIR, "head_rgb_raw.png")
    cv2.imwrite(raw_path, bgr)
    print(f"[probe] saved {raw_path}", flush=True)

    # 2) 2D 색 세그멘테이션 (box_detect.py와 동일 임계값)
    hsv = cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2HSV)
    gmask = cv2.inRange(hsv, BoxDetector.GREEN_LO, BoxDetector.GREEN_HI) > 0
    ymask = cv2.inRange(hsv, BoxDetector.GRAY_LO, BoxDetector.GRAY_HI) > 0
    # 1280x720 color는 640x480 IR보다 면적 큼 -> MIN_AREA_PX를 해상도 비로 스케일
    scale = (W * H) / (640.0 * 480.0)
    min_area = int(BoxDetector.MIN_AREA_PX * scale)

    overlay = (bgr.astype(np.float32) * 0.45).astype(np.uint8)  # 어둡게 깔고 마스크 강조
    overlay[gmask] = (overlay[gmask] * 0.3 + np.array([0, 200, 0]) * 0.7).astype(np.uint8)
    overlay[ymask] = (overlay[ymask] * 0.3 + np.array([220, 220, 220]) * 0.7).astype(np.uint8)

    report = []
    for label, mask, col in (("green", gmask, (0, 255, 0)), ("gray", ymask, (255, 160, 0))):
        m, comps = largest_components(mask, min_area)
        tot = int(mask.sum())
        report.append((label, tot, len(comps), comps[0]["area"] if comps else 0))
        for rank, c in enumerate(comps[:3]):  # 상위 3개만 표기
            x, y, w, h = c["bbox"]
            cv2.rectangle(overlay, (x, y), (x + w, y + h), col, 2)
            pts = (c["lab"] == c["cid"]).astype(np.uint8)
            cnts, _ = cv2.findContours(pts, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if cnts:
                big = max(cnts, key=cv2.contourArea)
                rect = cv2.minAreaRect(big)
                box = cv2.boxPoints(rect).astype(np.int32)
                cv2.drawContours(overlay, [box], 0, col, 2)
                (rw, rh) = rect[1]
                long_px, short_px = max(rw, rh), min(rw, rh)
                tag = f"{label}#{rank} A={c['area']} rect={long_px:.0f}x{short_px:.0f}px"
            else:
                tag = f"{label}#{rank} A={c['area']}"
            cv2.putText(overlay, tag, (x, max(18, y - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)

    cv2.putText(overlay, f"min_area={min_area}px (2D color only, no depth/plane gate)",
                (10, H - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
    seg_path = os.path.join(OUT_DIR, "head_box_segmentation.png")
    cv2.imwrite(seg_path, overlay)
    print(f"[probe] saved {seg_path}", flush=True)

    print("\n[probe] === 2D color segmentation report ===")
    print(f"  image {W}x{H}  min_area={min_area}px")
    for label, tot, ncomp, top in report:
        pct = 100.0 * tot / (W * H)
        print(f"  {label:5s}: mask_px={tot:8d} ({pct:5.2f}%)  comps>=min={ncomp}  "
              f"largest={top}px")
    print("\n  해석: depth/평면 게이트 없는 순수 2D 색 분할 결과. mask%가 크고 comps가 많을수록\n"
          "        색만으로는 박스 분리가 어렵다는 뜻(특히 gray는 저채도라 배경/테이블 흡수).", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
