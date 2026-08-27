#!/usr/bin/env python3
"""손목(D405) raw 포인트 클라우드 publisher — wrist-only, 모델/GPU 없음.

camera_server 번들(`camera.bundle.wrist_left` / `.wrist_right`)에서 D405의
color+depth를 읽어 카메라 광학 프레임의 XYZRGB 점을 만들고 `stereo.wrist`
토픽으로 PUB한다. rb_gui(`pointcloud_receiver.py`)가 그대로 구독해
TCP x hand-eye로 stand 프레임에 배치한다.

*** 이 파일이 왜 따로 존재하는가 ***
원래 이 기능은 `stereo_worker/worker.py`의 한 갈래였고, 그 worker는 head D435
스테레오를 Fast-FoundationStereo로 추론하는 것이 본체였다. 2026-08-26 커밋
3287da8이 FFS submodule을 제거하면서 worker 전체를 함께 삭제했는데, 손목
클라우드 경로는 FFS와 무관했다 — D405가 depth를 직접 주므로 역투영 한 번이
전부다(numpy만 필요, torch/TensorRT/CUDA 불필요). 그래서 삭제된 worker에서
그 갈래만 복원한 것이 이 파일이다. head 스테레오/박스 검출/융합/ROI 클립은
FFS와 함께 사라졌으므로 여기에 없다.

publish 계약(rb_gui와 합의된 것, 원본과 동일):
    multipart [b"stereo.wrist", header_json{arm,seq,n}, xyz_f32(Nx3), rgb_u8(Nx3)]

실행:
    python3 camera_server/stereo_worker/wrist_cloud_worker.py
환경변수:
    CAMERA_BUNDLE_ENDPOINT  기본 tcp://127.0.0.1:5600  (camera_server 메타데이터)
    CLOUD_PUB_BIND          기본 tcp://127.0.0.1:5601  (rb_gui가 구독하는 곳)
    WRIST_EVERY             기본 3   (N 프레임마다 발행 — 30fps -> 10Hz)
    WRIST_VIZ_MAX_PTS       기본 30000 (발행 직전 랜덤 서브샘플, 0=무제한)
    WRIST_ZMIN / WRIST_ZMAX 기본 0.05 / 1.2 [m]
    WRIST_STRIDE            기본 2   (픽셀 솎음)
"""
from __future__ import annotations

import json
import os
import sys
import time

import numpy as np

BUNDLE_ENDPOINT = os.environ.get("CAMERA_BUNDLE_ENDPOINT", "tcp://127.0.0.1:5600")
CLOUD_PUB_BIND = os.environ.get("CLOUD_PUB_BIND", "tcp://127.0.0.1:5601")
WRIST_BUNDLE_TOPICS = ["camera.bundle.wrist_left", "camera.bundle.wrist_right"]

# 손목 D405 (640x480) intrinsics + depth scale. 원본 worker.py에서 그대로 가져온
# 값(pc_infer DEFAULT_INTRINSICS)이라 클라우드의 스케일/형상이 이전과 동일하다.
WRIST_FX, WRIST_FY, WRIST_CX, WRIST_CY = 393.3166, 392.1858, 319.0753, 229.5226
WRIST_DSCALE = 1e-4

_MESHGRID_CACHE: dict = {}


def wrist_cloud(depth, color, zmin=0.05, zmax=1.2, stride=2):
    """D405 depth(z16) -> 카메라 프레임 3D점 + 같은 픽셀의 RGB(근사). 모델 없음."""
    h, w = depth.shape
    z = depth[::stride, ::stride].astype(np.float32) * WRIST_DSCALE
    col = color[::stride, ::stride]
    key = (h, w, stride)
    grid = _MESHGRID_CACHE.get(key)
    if grid is None:
        grid = np.meshgrid(np.arange(0, w, stride, dtype=np.float32),
                           np.arange(0, h, stride, dtype=np.float32))
        _MESHGRID_CACHE[key] = grid
    uu, vv = grid
    valid = (z > zmin) & (z < zmax)
    x = (uu - WRIST_CX) / WRIST_FX * z
    y = (vv - WRIST_CY) / WRIST_FY * z
    pts = np.stack([x, y, z], -1)[valid]
    return pts.astype(np.float32), col[valid].astype(np.uint8)


class WristCloudPublisher:
    """ZMQ PUB: [b"stereo.wrist", header_json, xyz_f32, rgb_u8]."""

    def __init__(self, bind: str, viz_max_pts: int = 0) -> None:
        import zmq
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.PUB)
        self._sock.setsockopt(zmq.SNDHWM, 4)
        self._sock.bind(bind)
        self._viz_max_pts = int(viz_max_pts)

    def _cap(self, seq, xyz, rgb):
        """발행 직전 viz 전용 랜덤 서브샘플. 16만 점을 그대로 흘리면 rb_gui의
        수신 스레드와 viser 웹소켓이 포화된다."""
        n = int(xyz.shape[0])
        if self._viz_max_pts <= 0 or n <= self._viz_max_pts:
            return xyz, rgb
        idx = np.random.default_rng(seq if seq >= 0 else 0).choice(
            n, self._viz_max_pts, replace=False)
        return xyz[idx], rgb[idx]

    def publish_wrist(self, arm: str, seq: int, xyz, rgb) -> None:
        xyz, rgb = self._cap(seq, xyz, rgb)
        header = json.dumps({"arm": arm, "seq": int(seq),
                             "n": int(xyz.shape[0])}).encode()
        self._sock.send_multipart([b"stereo.wrist", header,
                                   np.ascontiguousarray(xyz, np.float32).tobytes(),
                                   np.ascontiguousarray(rgb, np.uint8).tobytes()])

    def close(self) -> None:
        self._sock.close(linger=0)


def main() -> int:
    sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
    from bundle_reader import BundleReader

    every = max(1, int(os.environ.get("WRIST_EVERY", "3")))
    viz_max_pts = int(os.environ.get("WRIST_VIZ_MAX_PTS", "30000"))
    zmin = float(os.environ.get("WRIST_ZMIN", "0.05"))
    zmax = float(os.environ.get("WRIST_ZMAX", "1.2"))
    stride = max(1, int(os.environ.get("WRIST_STRIDE", "2")))

    cams = [("left", "left_realsense.color", "left_realsense.depth"),
            ("right", "right_realsense.color", "right_realsense.depth")]
    want = {key for _arm, kc, kd in cams for key in (kc, kd)}

    readers = [BundleReader(endpoint=BUNDLE_ENDPOINT, topic=t)
               for t in WRIST_BUNDLE_TOPICS]
    pub = WristCloudPublisher(CLOUD_PUB_BIND, viz_max_pts=viz_max_pts)
    print(f"[wrist] bundle={BUNDLE_ENDPOINT} topics={WRIST_BUNDLE_TOPICS} "
          f"pub={CLOUD_PUB_BIND} every={every} cap={viz_max_pts or 'off'} "
          f"z[{zmin},{zmax}]m stride={stride}", flush=True)

    seq = 0
    fps_t = time.time()
    fps_n = 0
    pts_last = {}
    try:
        while True:
            frames = {}
            # 첫 리더가 페이싱(최대 200ms 대기), 나머지는 non-blocking으로 합류.
            for i, reader in enumerate(readers):
                frames.update(reader.poll(want, timeout_ms=200 if i == 0 else 0))
            if not frames:
                continue   # 이번 틱 수신 없음 — poll timeout이 페이싱

            if seq % every == 0:
                for arm, kc, kd in cams:
                    c = frames.get(kc)
                    d = frames.get(kd)
                    if c is None or d is None:
                        continue
                    try:
                        xyz, rgb = wrist_cloud(d.pixels, c.pixels,
                                               zmin=zmin, zmax=zmax, stride=stride)
                    except Exception as exc:  # noqa: BLE001
                        print(f"[wrist] {arm} cloud failed: {exc}", flush=True)
                        continue
                    if xyz.shape[0] == 0:
                        continue
                    pts_last[arm] = int(xyz.shape[0])
                    try:
                        pub.publish_wrist(arm, seq, xyz, rgb)
                    except Exception as exc:  # noqa: BLE001
                        print(f"[wrist] {arm} publish failed: {exc}", flush=True)

            seq += 1
            fps_n += 1
            now = time.time()
            if now - fps_t > 1.0:
                fps = fps_n / (now - fps_t)
                pts = " ".join(f"{a}={n}" for a, n in sorted(pts_last.items())) or "none"
                print(f"[wrist] fps={fps:.1f} pub_every={every} pts[{pts}]", flush=True)
                fps_t = now
                fps_n = 0
                pts_last = {}
    except KeyboardInterrupt:
        pass
    finally:
        for reader in readers:
            reader.close()
        pub.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
