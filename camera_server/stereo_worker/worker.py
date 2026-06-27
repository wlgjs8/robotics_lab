#!/usr/bin/env python3
"""stereo_worker 엔트리포인트.

--smoke : 데모 스테레오 페어로 모델 구동 검증.
--run   : camera_server 번들(IR-L/R [+color]) 구독 -> Fast-FoundationStereo disparity
          -> depth -> pointcloud -> ZMQ publish (rb_gui가 구독해서 렌더).
"""
from __future__ import annotations
import argparse
import json
import os
import time
import numpy as np

FFS_DIR = os.environ.get("FFS_DIR", "/app/Fast-FoundationStereo")
WEIGHTS = os.environ.get("STEREO_WEIGHTS",
                         f"{FFS_DIR}/weights/23-36-37/model_best_bp2_serialize.pth")
BUNDLE_ENDPOINT = os.environ.get("CAMERA_BUNDLE_ENDPOINT", "tcp://127.0.0.1:5600")
CLOUD_PUB_BIND = os.environ.get("CLOUD_PUB_BIND", "tcp://127.0.0.1:5601")
CLOUD_TOPIC = os.environ.get("CLOUD_TOPIC", "stereo.cloud")
STEREO_CAMERA = os.environ.get("STEREO_CAMERA", "head")
STEREO_INTRINSICS = os.environ.get("STEREO_INTRINSICS",
                                   "/app/config/d435_ir_640x480_K.txt")
COLOR_CALIB_PATH = os.environ.get("STEREO_COLOR_CALIB", "/app/config/d435_color_calib.json")


def load_color_calib(path):
    """color intrinsics + IR-left->color 외부보정 (RGB 매핑용)."""
    try:
        with open(path) as f:
            c = json.load(f)
        c["R"] = np.array(c["R_ir_to_color"], float)
        c["t"] = np.array(c["t_ir_to_color"], float)
        return c
    except Exception:
        return None


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


def color_cloud(disp, color_img, K, baseline, calib, dmin=0.1, dmax=3.0):
    """disp(IR-left) -> 3D점(IR 프레임) + 각 점을 color 카메라에 재투영해 RGB 샘플링."""
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    H, W = disp.shape
    uu, vv = np.meshgrid(np.arange(W), np.arange(H))
    with np.errstate(divide="ignore", invalid="ignore"):
        z = fx * baseline / disp
    valid = np.isfinite(z) & (z >= dmin) & (z <= dmax)
    X = (uu - cx) / fx * z; Y = (vv - cy) / fy * z
    P = np.stack([X, Y, z], -1)                       # IR-left 프레임
    Pc = P @ calib["R"].T + calib["t"]                # color 프레임
    Zc = Pc[..., 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        uc = calib["color_fx"] * Pc[..., 0] / Zc + calib["color_cx"]
        vc = calib["color_fy"] * Pc[..., 1] / Zc + calib["color_cy"]
    Hc, Wc = color_img.shape[:2]
    inb = valid & (Zc > 0) & (uc >= 0) & (uc < Wc - 1) & (vc >= 0) & (vc < Hc - 1)
    ui = np.clip(uc, 0, Wc - 1).astype(np.int32)
    vi = np.clip(vc, 0, Hc - 1).astype(np.int32)
    return P[inb].astype(np.float32), color_img[vi[inb], ui[inb]].astype(np.uint8)
# TRT fp16 엔진이 있으면 우선 사용(~14ms), 없으면 PyTorch(.pth) 폴백.
STEREO_ENGINE = os.environ.get("STEREO_ENGINE",
                               "/app/stereo_worker/engines/fast_foundationstereo.engine")


def load_ktxt(path):
    """FoundationStereo K.txt 포맷: 3x3 flat + baseline(m)."""
    with open(path) as f:
        lines = f.readlines()
    K = np.array(list(map(float, lines[0].split()))).reshape(3, 3)
    baseline = float(lines[1])
    return K, baseline


class CloudPublisher:
    """ZMQ PUB: [topic, header_json, xyz_f32, rgb_u8]."""
    def __init__(self, bind, topic):
        import zmq, json
        self._json = json
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.PUB)
        self._sock.setsockopt(zmq.SNDHWM, 4)
        self._sock.bind(bind)
        self._topic = topic.encode()

    def publish(self, seq, ts_ns, xyz, rgb, frame="camera"):
        header = self._json.dumps({"seq": int(seq), "ts_ns": int(ts_ns),
                                   "n": int(xyz.shape[0]), "frame": frame}).encode()
        self._sock.send_multipart([self._topic, header,
                                   np.ascontiguousarray(xyz, np.float32).tobytes(),
                                   np.ascontiguousarray(rgb, np.uint8).tobytes()])

    def publish_boxes(self, seq, boxes, frame="stand"):
        """박스 pose(T_stand_box)를 stereo.boxes 토픽으로 publish (같은 PUB 소켓)."""
        payload = {"seq": int(seq), "frame": frame, "boxes": [
            {"T": [float(v) for v in b["T"].flatten()],
             "dims": [float(v) for v in b["dims"]],
             "footprint": [float(v) for v in b["footprint"]],
             "n": int(b["n"]), "label": b.get("label")} for b in boxes]}
        self._sock.send_multipart([b"stereo.boxes", self._json.dumps(payload).encode()])

    def publish_wrist(self, arm, seq, xyz, rgb):
        """손목 raw 클라우드(카메라 프레임). rb_gui가 TCP×hand-eye로 배치."""
        header = self._json.dumps({"arm": arm, "seq": int(seq), "n": int(xyz.shape[0])}).encode()
        self._sock.send_multipart([b"stereo.wrist", header,
                                   np.ascontiguousarray(xyz, np.float32).tobytes(),
                                   np.ascontiguousarray(rgb, np.uint8).tobytes()])


def cmd_run(args):
    import sys
    sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
    from bundle_reader import BundleReader

    K, baseline = load_ktxt(STEREO_INTRINSICS)
    print(f"[run] intrinsics fx={K[0,0]:.1f} baseline={baseline*1000:.1f}mm  "
          f"bundle={BUNDLE_ENDPOINT}  pub={CLOUD_PUB_BIND}", flush=True)
    use_trt = os.path.exists(STEREO_ENGINE) and not args.force_torch
    if use_trt:
        from stereo_model import TrtStereoModel
        model = TrtStereoModel(FFS_DIR, STEREO_ENGINE)
        print(f"[run] backend=TensorRT  engine={STEREO_ENGINE}", flush=True)
    else:
        from stereo_model import FoundationStereoModel
        model = FoundationStereoModel(FFS_DIR, WEIGHTS, valid_iters=args.valid_iters)
        print(f"[run] backend=PyTorch  weights={WEIGHTS}", flush=True)
    print("[run] model loaded", flush=True)
    calib = load_color_calib(COLOR_CALIB_PATH)
    print(f"[run] color calib: {'loaded ('+COLOR_CALIB_PATH+')' if calib else 'none -> IR 명암'}", flush=True)

    cam = STEREO_CAMERA
    k_irl, k_irr, k_col = f"{cam}.ir_left", f"{cam}.ir_right", f"{cam}.color"
    want = {k_irl, k_irr, k_col}
    # 손목 raw 클라우드 (모델 X). 저rate로 가볍게 publish (head 파이프라인 영향 최소).
    wrist_on = os.environ.get("STEREO_WRIST", "1") != "0"
    wrist_every = int(os.environ.get("STEREO_WRIST_EVERY", "3"))
    wrist_cams = [("left", "left_realsense.color", "left_realsense.depth"),
                  ("right", "right_realsense.color", "right_realsense.depth")]
    if wrist_on:
        for _a, kc, kd in wrist_cams:
            want.add(kc); want.add(kd)
    reader = BundleReader(endpoint=BUNDLE_ENDPOINT)
    pub = CloudPublisher(CLOUD_PUB_BIND, CLOUD_TOPIC)

    detector = None; tracker = None
    if os.environ.get("STEREO_DETECT", "1") != "0":
        try:
            from box_detect import BoxDetector, BoxTracker
            detector = BoxDetector(K, baseline, use_icp=os.environ.get("STEREO_DETECT_ICP", "1") != "0")
            if os.environ.get("STEREO_TRACK", "1") != "0":
                tracker = BoxTracker()
            print(f"[run] box detect: ON (icp={detector.use_icp}, track={tracker is not None})", flush=True)
        except Exception as e:  # noqa: BLE001
            print(f"[run] box detect OFF: {e}", flush=True)

    seq = 0
    fps_t, fps_n, fps = time.time(), 0, 0.0
    n_boxes = 0
    warned_rgb = False
    while True:
        frames = reader.poll(want, timeout_ms=500)
        irl = frames.get(k_irl); irr = frames.get(k_irr)
        if irl is None or irr is None:
            continue
        disp = model.infer_disparity(irl.pixels, irr.pixels)
        col = frames.get(k_col)
        if calib is not None and col is not None:
            xyz, rgb = color_cloud(disp, col.pixels, K, baseline, calib, dmin=0.1, dmax=args.zfar)
            if not warned_rgb:
                print("[run] RGB 매핑 ON (color->IR 재투영)", flush=True); warned_rgb = True
        else:
            xyz, rgb = model.disparity_to_cloud(disp, irl.pixels, K, baseline, zfar=args.zfar)
            if not warned_rgb:
                print(f"[run] IR 명암 사용 (calib={calib is not None}, color={col is not None})", flush=True)
                warned_rgb = True
        if xyz.shape[0] == 0:
            continue  # 유효 점 없음(예: fp16 NaN/범위밖) -> publish/통계 건너뜀
        pub.publish(seq, time.time_ns(), xyz, rgb)
        if detector is not None:
            try:
                raw = detector.detect(disp, color_img=(col.pixels if col is not None else None))
                boxes = tracker.update(raw) if tracker is not None else raw
                n_boxes = len(boxes)
                pub.publish_boxes(seq, boxes)
            except Exception as e:  # noqa: BLE001
                if seq % 60 == 0:
                    print(f"[run] detect err: {e}", flush=True)
        # 손목 raw 클라우드 (저rate, 가볍게)
        if wrist_on and seq % max(1, wrist_every) == 0:
            for arm, kc, kd in wrist_cams:
                c = frames.get(kc); d = frames.get(kd)
                if c is None or d is None:
                    continue
                try:
                    wp, wc = wrist_cloud(d.pixels, c.pixels)
                    if wp.shape[0] > 0:
                        pub.publish_wrist(arm, seq, wp, wc)
                except Exception:
                    pass
        seq += 1
        fps_n += 1
        if time.time() - fps_t > 1.0:
            fps = fps_n / (time.time() - fps_t); fps_t = time.time(); fps_n = 0
            print(f"[run] fps={fps:.1f} cloud_pts={xyz.shape[0]} boxes={n_boxes} "
                  f"z[{xyz[:,2].min():.2f},{xyz[:,2].max():.2f}]m", flush=True)
        if args.max_frames and seq >= args.max_frames:
            print(f"[run] reached max_frames={args.max_frames}, exit", flush=True)
            break
    reader.close()


def cmd_smoke(args):
    import imageio.v2 as imageio
    import cv2
    from stereo_model import FoundationStereoModel

    print(f"[smoke] FFS_DIR={FFS_DIR}\n[smoke] weights={WEIGHTS}", flush=True)
    m = FoundationStereoModel(FFS_DIR, WEIGHTS, valid_iters=args.valid_iters)
    print("[smoke] model loaded", flush=True)
    left = imageio.imread(f"{FFS_DIR}/demo_data/left.png")
    right = imageio.imread(f"{FFS_DIR}/demo_data/right.png")
    disp = m.infer_disparity(left, right)
    t0 = time.time(); disp = m.infer_disparity(left, right); dt = (time.time() - t0) * 1000
    print(f"[smoke] disparity {disp.shape} min={disp.min():.2f} max={disp.max():.2f} "
          f"infer={dt:.0f}ms", flush=True)
    os.makedirs(args.out_dir, exist_ok=True)
    vis = cv2.applyColorMap(cv2.convertScaleAbs(disp, alpha=255.0 / max(disp.max(), 1e-6)),
                            cv2.COLORMAP_TURBO)
    cv2.imwrite(f"{args.out_dir}/disp_vis.png", vis)
    print(f"[smoke] saved {args.out_dir}/disp_vis.png\n[smoke] OK", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.set_defaults(cmd="smoke")
    ap.add_argument("--smoke", action="store_const", dest="cmd", const="smoke")
    ap.add_argument("--run", action="store_const", dest="cmd", const="run")
    ap.add_argument("--valid-iters", type=int, default=8)
    ap.add_argument("--zfar", type=float, default=3.0)
    ap.add_argument("--force-torch", action="store_true", help="TRT 엔진 무시하고 PyTorch 백엔드 사용")
    ap.add_argument("--max-frames", type=int, default=0, help="0=forever (테스트용 N프레임 후 종료)")
    ap.add_argument("--out-dir", default="/app/stereo_worker/out")
    args = ap.parse_args()
    {"smoke": cmd_smoke, "run": cmd_run}[args.cmd](args)


if __name__ == "__main__":
    main()
