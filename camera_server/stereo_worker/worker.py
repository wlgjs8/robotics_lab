#!/usr/bin/env python3
"""stereo_worker 엔트리포인트.

--smoke : 데모 스테레오 페어로 모델 구동 검증.
--run   : camera_server 번들(IR-L/R [+color]) 구독 -> Fast-FoundationStereo disparity
          -> depth -> pointcloud -> ZMQ publish (rb_gui가 구독해서 렌더).
"""
from __future__ import annotations
import argparse
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


def cmd_run(args):
    import sys
    sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
    from stereo_model import FoundationStereoModel
    from bundle_reader import BundleReader

    K, baseline = load_ktxt(STEREO_INTRINSICS)
    print(f"[run] intrinsics fx={K[0,0]:.1f} baseline={baseline*1000:.1f}mm  "
          f"bundle={BUNDLE_ENDPOINT}  pub={CLOUD_PUB_BIND}", flush=True)
    model = FoundationStereoModel(FFS_DIR, WEIGHTS, valid_iters=args.valid_iters)
    print("[run] model loaded", flush=True)

    cam = STEREO_CAMERA
    k_irl, k_irr, k_col = f"{cam}.ir_left", f"{cam}.ir_right", f"{cam}.color"
    want = {k_irl, k_irr, k_col}
    reader = BundleReader(endpoint=BUNDLE_ENDPOINT)
    pub = CloudPublisher(CLOUD_PUB_BIND, CLOUD_TOPIC)

    seq = 0
    fps_t, fps_n, fps = time.time(), 0, 0.0
    warned_rgb = False
    while True:
        frames = reader.poll(want, timeout_ms=500)
        irl = frames.get(k_irl); irr = frames.get(k_irr)
        if irl is None or irr is None:
            continue
        disp = model.infer_disparity(irl.pixels, irr.pixels)
        # 색: color 스트림이 IR과 정합되지 않으므로(서로 다른 센서/FOV) 현재는 IR-left 명암으로 색칠.
        # 정확한 RGB 매핑은 color->IR 외부보정 + 재투영 필요(USB3에서 color 활성 후 TODO).
        if k_col in frames and not warned_rgb:
            print("[run] color 수신됨 — RGB->IR 정합 미구현, 현재는 IR 명암 사용", flush=True)
            warned_rgb = True
        xyz, rgb = model.disparity_to_cloud(disp, irl.pixels, K, baseline, zfar=args.zfar)
        pub.publish(seq, time.time_ns(), xyz, rgb)
        seq += 1
        fps_n += 1
        if time.time() - fps_t > 1.0:
            fps = fps_n / (time.time() - fps_t); fps_t = time.time(); fps_n = 0
            print(f"[run] fps={fps:.1f} cloud_pts={xyz.shape[0]} "
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
    ap.add_argument("--max-frames", type=int, default=0, help="0=forever (테스트용 N프레임 후 종료)")
    ap.add_argument("--out-dir", default="/app/stereo_worker/out")
    args = ap.parse_args()
    {"smoke": cmd_smoke, "run": cmd_run}[args.cmd](args)


if __name__ == "__main__":
    main()
