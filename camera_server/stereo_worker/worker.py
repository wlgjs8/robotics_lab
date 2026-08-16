#!/usr/bin/env python3
"""stereo_worker 엔트리포인트 (wrist-only).

--run : camera_server 손목 번들(D405 color+depth) 구독 -> raw 3D 점 -> ZMQ publish
        (rb_gui가 구독해서 TCP x hand-eye로 배치·렌더).

head D435 스테레오 추론(Fast-FoundationStereo), 박스 검출, external-box 송신,
손목-head 융합은 2026-08-16에 제거되었다. 이 워커에는 모델이 없다: D405가 하드웨어로
만든 depth를 그대로 deproject할 뿐이다. 배경은 docs/archive/head_stereo/README.md.
"""
from __future__ import annotations
import argparse
import os
import time
import numpy as np

BUNDLE_ENDPOINT = os.environ.get("CAMERA_BUNDLE_ENDPOINT", "tcp://127.0.0.1:5600")
# 손목 번들: 좌/우 독립 그룹을 기본 구독해 한쪽 손목 카메라가 죽어도 건강한 쪽
# RGB-D는 계속 흐른다(예전 camera.bundle.policy 단일 구독은 4스트림 complete 게이트라
# 한쪽 사망 시 양쪽이 함께 끊겼다). STEREO_WRIST_BUNDLE_TOPIC 명시 시 단일 토픽 구독.
_wrist_topic_env = os.environ.get("STEREO_WRIST_BUNDLE_TOPIC")
STEREO_WRIST_BUNDLE_TOPICS = ([_wrist_topic_env] if _wrist_topic_env
                              else ["camera.bundle.wrist_left", "camera.bundle.wrist_right"])
CLOUD_PUB_BIND = os.environ.get("CLOUD_PUB_BIND", "tcp://127.0.0.1:5601")

# 손목 D405 (640x480) intrinsics + depth scale (pc_infer DEFAULT_INTRINSICS). raw cloud용.
WRIST_FX, WRIST_FY, WRIST_CX, WRIST_CY = 393.3166, 392.1858, 319.0753, 229.5226
WRIST_DSCALE = 1e-4


def wrist_cloud(depth, color, zmin=0.05, zmax=1.2, stride=2):
    """D405 depth(z16) -> raw 3D점(카메라 프레임) + RGB(같은 픽셀 샘플, 근사). 모델 X."""
    H, W = depth.shape
    z = depth[::stride, ::stride].astype(np.float32) * WRIST_DSCALE
    col = color[::stride, ::stride]
    uu, vv = np.meshgrid(np.arange(0, W, stride), np.arange(0, H, stride))
    valid = (z > zmin) & (z < zmax)
    X = (uu.astype(np.float32) - WRIST_CX) / WRIST_FX * z
    Y = (vv.astype(np.float32) - WRIST_CY) / WRIST_FY * z
    P = np.stack([X, Y, z], -1)[valid]
    return P.astype(np.float32), col[valid].astype(np.uint8)


def _ck(prof, name, t0):
    """파이프라인 단계 wall-time 누적(STEREO_PROFILE). 다음 단계 시작 시각 반환."""
    t1 = time.perf_counter()
    prof[name] = prof.get(name, 0.0) + (t1 - t0)
    return t1


class CloudPublisher:
    """ZMQ PUB: [topic, header_json, xyz_f32, rgb_u8]."""
    def __init__(self, bind, viz_max_pts=0):
        import zmq, json
        self._json = json
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.PUB)
        self._sock.setsockopt(zmq.SNDHWM, 4)
        self._sock.bind(bind)
        # publish(=시각화) 전용 점수 상한.
        self._viz_max_pts = int(viz_max_pts)

    def _cap(self, seq, xyz, rgb):
        """발행 직전 viz 전용 random subsample (seq 시드 → 프레임마다 균일 분포)."""
        n = int(xyz.shape[0])
        if self._viz_max_pts <= 0 or n <= self._viz_max_pts:
            return xyz, rgb
        idx = np.random.default_rng(seq if seq >= 0 else 0).choice(
            n, self._viz_max_pts, replace=False)
        return xyz[idx], rgb[idx]

    def publish_wrist(self, arm, seq, xyz, rgb):
        """손목 raw 클라우드(카메라 프레임). rb_gui가 TCP×hand-eye로 배치."""
        xyz, rgb = self._cap(seq, xyz, rgb)
        header = self._json.dumps({"arm": arm, "seq": int(seq), "n": int(xyz.shape[0])}).encode()
        self._sock.send_multipart([b"stereo.wrist", header,
                                   np.ascontiguousarray(xyz, np.float32).tobytes(),
                                   np.ascontiguousarray(rgb, np.uint8).tobytes()])


def cmd_run(args):
    import sys
    sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
    from bundle_reader import BundleReader

    # 손목 발행이 이 워커의 유일한 일이다. 꺼두면 할 일이 없으므로 조용히 도는 대신
    # 즉시 종료해 오설정을 드러낸다(worker가 살아있는데 아무것도 안 나오는 상태 방지).
    if os.environ.get("STEREO_WRIST", "1") == "0":
        print("[run] FATAL: STEREO_WRIST=0 인데 이 워커는 손목 클라우드 발행 전용이다.\n"
              "       발행을 끄려면 STEREO_WORKER_AUTOSTART=0 으로 워커 자체를 띄우지 마라.",
              flush=True)
        raise SystemExit(2)

    wrist_every = int(os.environ.get("STEREO_WRIST_EVERY", "3"))
    wrist_cams = [("left", "left_realsense.color", "left_realsense.depth"),
                  ("right", "right_realsense.color", "right_realsense.depth")]
    wrist_want = {key for _arm, kc, kd in wrist_cams for key in (kc, kd)}
    wrist_readers = [BundleReader(endpoint=BUNDLE_ENDPOINT, topic=t)
                     for t in STEREO_WRIST_BUNDLE_TOPICS]
    viz_max_pts = int(os.environ.get("STEREO_VIZ_MAX_PTS", "30000"))
    pub = CloudPublisher(CLOUD_PUB_BIND, viz_max_pts=viz_max_pts)
    print(f"[run] wrist-only: bundle={BUNDLE_ENDPOINT} "
          f"topics[{','.join(STEREO_WRIST_BUNDLE_TOPICS)}] pub={CLOUD_PUB_BIND} "
          f"every={wrist_every} viz_cap={viz_max_pts if viz_max_pts > 0 else 'off'}", flush=True)

    seq = 0
    fps_t, fps_n, fps = time.time(), 0, 0.0
    n_pts = 0
    prof_on = os.environ.get("STEREO_PROFILE", "1") != "0"   # 단계별 ms 브레이크다운
    prof = {}
    while True:
        _t = time.perf_counter()
        # 첫 reader의 poll이 루프 페이싱(최대 200ms 대기), 나머지는 non-blocking drain.
        frames = {}
        for _wi, _wr in enumerate(wrist_readers):
            frames.update(_wr.poll(wrist_want, timeout_ms=(200 if _wi == 0 else 0)))
        _t = _ck(prof, "poll", _t)
        if not frames:
            continue   # 이번 틱 수신 없음(양쪽 카메라 유휴/장애) — poll timeout이 페이싱

        # 발행하는 프레임에서만 클라우드를 만든다(계산 낭비 제거).
        if seq % max(1, wrist_every) == 0:
            n_pts = 0
            for arm, kc, kd in wrist_cams:
                c = frames.get(kc); d = frames.get(kd)
                if c is None or d is None:
                    continue
                try:
                    wp, wc = wrist_cloud(d.pixels, c.pixels)
                except Exception:
                    continue
                if wp.shape[0] == 0:
                    continue
                n_pts += int(wp.shape[0])
                try:
                    pub.publish_wrist(arm, seq, wp, wc)
                except Exception:
                    pass
        _t = _ck(prof, "wrist", _t)

        seq += 1
        fps_n += 1
        if time.time() - fps_t > 1.0:
            nwin = max(fps_n, 1)
            fps = fps_n / (time.time() - fps_t); fps_t = time.time(); fps_n = 0
            prof_str = ""
            if prof_on and prof:
                prof_str = "  ms/frame[" + " ".join(
                    f"{k}={1000.0*v/nwin:.0f}" for k, v in prof.items()) + "]"
            prof = {}
            print(f"[run] fps={fps:.1f} wrist_pts={n_pts}{prof_str}", flush=True)
        if args.max_frames and seq >= args.max_frames:
            print(f"[run] reached max_frames={args.max_frames}, exit", flush=True)
            break
    for _wr in wrist_readers:
        _wr.close()


def main():
    ap = argparse.ArgumentParser()
    # --run 은 run_all.sh 및 기존 운영 절차와의 호환을 위해 유지(유일한 모드).
    ap.add_argument("--run", action="store_true", help="손목 클라우드 발행 루프(기본 동작)")
    ap.add_argument("--max-frames", type=int, default=0, help="0=forever (테스트용 N프레임 후 종료)")
    cmd_run(ap.parse_args())


if __name__ == "__main__":
    main()
