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
# 아이디어2: 가림 시 head 박스 검출에 손목(D405) 클라우드를 stand 프레임으로 병합.
STEREO_FUSE_WRIST = os.environ.get("STEREO_FUSE_WRIST", "1") != "0"
STATE_ENDPOINT = os.environ.get("STEREO_STATE_ENDPOINT", "udp://127.0.0.1:50386")
T_TCP_CAM_PATH = os.environ.get("STEREO_T_TCP_CAM", "/calibration/T_tcp_cam.npy")
STEREO_SEND_EXTERNAL_BOXES = os.environ.get("STEREO_SEND_EXTERNAL_BOXES", "0") == "1"
STEREO_COMMAND_ENDPOINT = os.environ.get("STEREO_COMMAND_ENDPOINT", "127.0.0.1:50010")
STEREO_BOX_SOURCE_ID = os.environ.get("STEREO_BOX_SOURCE_ID", "stereo_worker")


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


def color_cloud(disp, color_img, K, baseline, calib, dmin=0.1, dmax=3.0, T_sc=None, roi=None):
    """disp(IR-left) -> 3D점(IR 프레임) + color 재투영 RGB. T_sc+roi를 주면 stand-ROI로 먼저
    추려 그 subset만 재투영한다(전체 organized 921k 재투영 회피 + ROI-clip 일체화)."""
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    H, W = disp.shape
    uu, vv = np.meshgrid(np.arange(W), np.arange(H))
    with np.errstate(divide="ignore", invalid="ignore"):
        z = fx * baseline / disp
    valid = np.isfinite(z) & (z >= dmin) & (z <= dmax)
    X = (uu - cx) / fx * z; Y = (vv - cy) / fy * z
    P = np.stack([X, Y, z], -1)                       # IR-left 프레임 (organized)
    sel = valid
    if T_sc is not None and roi is not None:          # stand-ROI로 먼저 추림 → 재투영 점 수↓
        Ps = P @ T_sc[:3, :3].T + T_sc[:3, 3]
        (rx, ry, rz) = roi
        sel = (valid & (Ps[..., 0] >= rx[0]) & (Ps[..., 0] <= rx[1]) &
               (Ps[..., 1] >= ry[0]) & (Ps[..., 1] <= ry[1]) &
               (Ps[..., 2] >= rz[0]) & (Ps[..., 2] <= rz[1]))
    Pf = P[sel]                                       # (K,3) IR — 이 subset만 color 재투영
    if len(Pf) == 0:
        return Pf.astype(np.float32), np.empty((0, 3), np.uint8)
    Pc = Pf @ calib["R"].T + calib["t"]               # color 프레임
    Zc = Pc[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        uc = calib["color_fx"] * Pc[:, 0] / Zc + calib["color_cx"]
        vc = calib["color_fy"] * Pc[:, 1] / Zc + calib["color_cy"]
    Hc, Wc = color_img.shape[:2]
    ok = (Zc > 0) & (uc >= 0) & (uc < Wc - 1) & (vc >= 0) & (vc < Hc - 1)
    ui = np.clip(uc, 0, Wc - 1).astype(np.int32)
    vi = np.clip(vc, 0, Hc - 1).astype(np.int32)
    return Pf[ok].astype(np.float32), color_img[vi[ok], ui[ok]].astype(np.uint8)
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


def load_T_tcp_cam(path):
    """손목 핸드아이 T_tcp_cam(4x4). 없거나 형식 불일치 시 None(→ 병합 비활성)."""
    try:
        T = np.load(path)
        if T.shape == (4, 4):
            return T.astype(np.float64)
    except Exception:
        pass
    return None


def roi_clip_mask(xyz_cam, T_sx, roi):
    """카메라 프레임 점을 stand로 옮겨 ROI((x0,x1),(y0,y1),(z0,z1)) 안인 점만 True.
    publish 전 head/손목 클라우드를 Safety ROI로 잘라 viz·검출 영역을 일치시킨다."""
    ps = xyz_cam.astype(np.float64) @ T_sx[:3, :3].T + T_sx[:3, 3]
    (x0, x1), (y0, y1), (z0, z1) = roi
    return ((ps[:, 0] >= x0) & (ps[:, 0] <= x1) &
            (ps[:, 1] >= y0) & (ps[:, 1] <= y1) &
            (ps[:, 2] >= z0) & (ps[:, 2] <= z1))


def effective_clip_roi(detector):
    """detector의 Safety ROI에 perception Z 캡(pc_clip_z_max_m)을 합친 publish-클립 RoI.
    ROI Z(로봇 safety용)는 그대로 두고, 그보다 낮은 Z 캡이 있으면 상한만 더 낮춘다."""
    rz = detector._roi_z
    if detector._z_cap is not None:
        rz = (rz[0], min(rz[1], detector._z_cap))
    return (detector._roi_x, detector._roi_y, rz)


def _ck(prof, name, t0):
    """파이프라인 단계 wall-time 누적(STEREO_PROFILE). 다음 단계 시작 시각 반환."""
    t1 = time.perf_counter()
    prof[name] = prof.get(name, 0.0) + (t1 - t0)
    return t1


def wait_for_file(path, timeout_s=30.0, poll_s=0.5):
    """camera_server가 기동 시 덤프하는 intrinsics K.txt를 기다린다(아직 없을 수 있음)."""
    t0 = time.time()
    while not os.path.exists(path):
        if time.time() - t0 > timeout_s:
            print(f"[run] WARN: intrinsics {path} 미존재({timeout_s:.0f}s 초과) -> 그대로 load 시도", flush=True)
            return
        time.sleep(poll_s)


class CloudPublisher:
    """ZMQ PUB: [topic, header_json, xyz_f32, rgb_u8]."""
    def __init__(self, bind, topic, viz_max_pts=0):
        import zmq, json
        self._json = json
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.PUB)
        self._sock.setsockopt(zmq.SNDHWM, 4)
        self._sock.bind(bind)
        self._topic = topic.encode()
        # publish(=시각화) 전용 점수 상한. detect는 disp에서 자기 클라우드를 따로 만들므로
        # (box_detect.BoxDetector._cam_points) 여기서 줄여도 박스 위치 추정은 무손실.
        self._viz_max_pts = int(viz_max_pts)

    def _cap(self, seq, xyz, rgb):
        """발행 직전 viz 전용 random subsample (seq 시드 → 프레임마다 균일 분포)."""
        n = int(xyz.shape[0])
        if self._viz_max_pts <= 0 or n <= self._viz_max_pts:
            return xyz, rgb
        idx = np.random.default_rng(seq if seq >= 0 else 0).choice(
            n, self._viz_max_pts, replace=False)
        return xyz[idx], rgb[idx]

    def publish(self, seq, ts_ns, xyz, rgb, frame="camera"):
        xyz, rgb = self._cap(seq, xyz, rgb)
        header = self._json.dumps({"seq": int(seq), "ts_ns": int(ts_ns),
                                   "n": int(xyz.shape[0]), "frame": frame}).encode()
        self._sock.send_multipart([self._topic, header,
                                   np.ascontiguousarray(xyz, np.float32).tobytes(),
                                   np.ascontiguousarray(rgb, np.uint8).tobytes()])

    def publish_boxes(self, seq, boxes, frame="stand"):
        """박스 pose(T_stand_box)를 stereo.boxes 토픽으로 publish (같은 PUB 소켓)."""
        def encode_box(b):
            out = {
                "T": [float(v) for v in b["T"].flatten()],
                "dims": [float(v) for v in b["dims"]],
                "footprint": [float(v) for v in b["footprint"]],
                "n": int(b["n"]),
                "label": b.get("label"),
            }
            for key in ("fitness", "rmse", "track_id", "icp_method", "source_n",
                        "icp_sample_n", "coasting"):
                if key not in b:
                    continue
                v = b[key]
                if v is None:
                    out[key] = None
                elif isinstance(v, (bool, np.bool_)):
                    out[key] = bool(v)
                elif isinstance(v, (int, np.integer)):
                    out[key] = int(v)
                elif isinstance(v, (float, np.floating)):
                    out[key] = float(v)
                else:
                    out[key] = v
            return out
        payload = {"seq": int(seq), "frame": frame, "boxes": [encode_box(b) for b in boxes]}
        self._sock.send_multipart([b"stereo.boxes", self._json.dumps(payload).encode()])

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
    from external_boxes_sender import ExternalBoxesSender

    wait_for_file(STEREO_INTRINSICS)
    if not os.path.exists(STEREO_INTRINSICS):
        print(f"[run] FATAL: intrinsics {STEREO_INTRINSICS} 가 없음.\n"
              f"       1280x720 헤드는 camera_server(C++)가 기동 시 이 파일을 덤프해야 한다.\n"
              f"       덤프가 없으면 보통 Docker 이미지가 새 C++로 재빌드되지 않은 것:\n"
              f"       `make cam-up`(--build 포함) 으로 이미지를 재빌드하라.", flush=True)
        raise SystemExit(2)
    K, baseline = load_ktxt(STEREO_INTRINSICS)
    print(f"[run] intrinsics fx={K[0,0]:.1f} cx={K[0,2]:.1f} cy={K[1,2]:.1f} "
          f"baseline={baseline*1000:.1f}mm  src={STEREO_INTRINSICS}  "
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
    # viz 전용 점수 상한(0=무제한). 16만 head 클라우드를 그대로 흘리면 rb_gui 수신 스레드
    # + viser 웹소켓을 포화시킨다 → 발행 직전 캡. detect는 disp 경로라 위치 추정 무손실.
    viz_max_pts = int(os.environ.get("STEREO_VIZ_MAX_PTS", "30000"))
    pub = CloudPublisher(CLOUD_PUB_BIND, CLOUD_TOPIC, viz_max_pts=viz_max_pts)
    print(f"[run] viz cloud cap: {viz_max_pts if viz_max_pts > 0 else 'off'} pts/frame", flush=True)
    external_boxes_sender = ExternalBoxesSender(
        endpoint=STEREO_COMMAND_ENDPOINT,
        source_id=STEREO_BOX_SOURCE_ID,
        enabled=STEREO_SEND_EXTERNAL_BOXES,
    )
    if STEREO_SEND_EXTERNAL_BOXES:
        print(f"[run] external boxes sender: ON endpoint={STEREO_COMMAND_ENDPOINT} "
              f"source_id={STEREO_BOX_SOURCE_ID}", flush=True)

    detector = None; tracker = None
    if os.environ.get("STEREO_DETECT", "1") != "0":
        try:
            from box_detect import BoxDetector, BoxTracker
            detector = BoxDetector(
                K, baseline,
                use_icp=os.environ.get("STEREO_DETECT_ICP", "1") != "0",
                icp_method=os.environ.get("STEREO_DETECT_ICP_METHOD", "yaw_se2"),
            )
            if os.environ.get("STEREO_TRACK", "1") != "0":
                tracker = BoxTracker()
            print(f"[run] box detect: ON (icp={detector.use_icp}, track={tracker is not None})", flush=True)
        except Exception as e:  # noqa: BLE001
            print(f"[run] box detect OFF: {e}", flush=True)

    # 아이디어2: 손목 클라우드 병합 (state로 TCP pose 구독 + 핸드아이). state 미수신 시 자동 head-only.
    pose_listener = None
    T_tcp_cam = None
    fuse_on = STEREO_FUSE_WRIST and detector is not None and wrist_on
    # 동기화: 팔 이동 중 융합하면 pose↔프레임 비동기로 손목 점이 어긋난다 → 정지 시에만 융합.
    fuse_motion_gate = os.environ.get("STEREO_FUSE_MOTION_GATE", "1") != "0"
    fuse_lin_thr = float(os.environ.get("STEREO_FUSE_MAX_LIN_MPS", "0.03"))
    fuse_ang_thr = float(os.environ.get("STEREO_FUSE_MAX_ANG_RPS", "0.15"))
    # publish하는 head/손목 클라우드를 Safety ROI(box_detect가 settings.json에서 읽는 값)로
    # 잘라 viser 시각화와 검출 영역을 일치시킨다(ROI 밖 noise 제거). detector가 ROI 소유.
    roi_clip_on = os.environ.get("STEREO_ROI_CLIP", "1") != "0" and detector is not None
    if roi_clip_on:
        print(f"[run] cloud ROI clip: ON (x={detector._roi_x} y={detector._roi_y} "
              f"z={detector._roi_z} z_cap={detector._z_cap}; rb_gui Safety ROI + "
              f"pc_clip_z_max_m via settings.json, 1Hz)", flush=True)
    if fuse_on:
        try:
            from state_listener import TcpPoseListener
            pose_listener = TcpPoseListener(STATE_ENDPOINT)
            gate_str = (f"motion-gate ON (<= {fuse_lin_thr} m/s, {fuse_ang_thr} rad/s)"
                        if fuse_motion_gate else "motion-gate OFF")
            print(f"[run] wrist fusion: ON (state={STATE_ENDPOINT}, T_tcp_cam={T_TCP_CAM_PATH}); "
                  f"{gate_str}. state 미수신 시 자동 head-only.", flush=True)
        except Exception as e:  # noqa: BLE001
            print(f"[run] wrist fusion OFF (state listener): {e}", flush=True)
            fuse_on = False

    seq = 0
    fps_t, fps_n, fps = time.time(), 0, 0.0
    n_boxes = 0
    warned_rgb = False
    last_clip_log = None      # ROI/Z캡 변경 시 1회 로그(operator 피드백)용
    prof_on = os.environ.get("STEREO_PROFILE", "1") != "0"   # 단계별 ms 브레이크다운
    prof = {}
    while True:
        _t = time.perf_counter()
        frames = reader.poll(want, timeout_ms=500)
        irl = frames.get(k_irl); irr = frames.get(k_irr)
        if irl is None or irr is None:
            continue
        _t = _ck(prof, "poll", _t)
        disp = model.infer_disparity(irl.pixels, irr.pixels)
        _t = _ck(prof, "infer", _t)
        col = frames.get(k_col)
        roi_cloud = effective_clip_roi(detector) if roi_clip_on else None
        cloud_clipped = False
        if calib is not None and col is not None:
            # color_cloud에 T_sc+roi를 주면 ROI subset만 재투영(+ROI clip 일체화)
            xyz, rgb = color_cloud(disp, col.pixels, K, baseline, calib, dmin=0.1, dmax=args.zfar,
                                   T_sc=(detector._T_sc if roi_cloud is not None else None),
                                   roi=roi_cloud)
            cloud_clipped = roi_cloud is not None
            if not warned_rgb:
                print("[run] RGB 매핑 ON (color->IR 재투영)", flush=True); warned_rgb = True
        else:
            xyz, rgb = model.disparity_to_cloud(disp, irl.pixels, K, baseline, zfar=args.zfar)
            if not warned_rgb:
                print(f"[run] IR 명암 사용 (calib={calib is not None}, color={col is not None})", flush=True)
                warned_rgb = True
        if xyz.shape[0] == 0:
            continue  # 유효 점 없음(예: fp16 NaN/범위밖) -> publish/통계 건너뜀
        if roi_clip_on and not cloud_clipped:            # 아직 안 잘린 경로(IR 등)만 클립
            hm = roi_clip_mask(xyz, detector._T_sc, effective_clip_roi(detector))
            pub.publish(seq, time.time_ns(), xyz[hm], rgb[hm])
        else:
            pub.publish(seq, time.time_ns(), xyz, rgb)
        _t = _ck(prof, "cloud+pub", _t)

        # 손목 raw 클라우드 계산(병합+발행 공용, 카메라 프레임)
        wrist_raw = {}
        if wrist_on or fuse_on:
            for arm, kc, kd in wrist_cams:
                c = frames.get(kc); d = frames.get(kd)
                if c is None or d is None:
                    continue
                try:
                    wp, wc = wrist_cloud(d.pixels, c.pixels)
                    if wp.shape[0] > 0:
                        wrist_raw[arm] = (wp, wc)
                except Exception:
                    pass
        _t = _ck(prof, "wrist", _t)

        # 병합용 stand 점: P_stand = T_stand_tcp @ T_tcp_cam @ P_wrist_cam
        extra_xyz = extra_rgb = None
        fused_arms = gated_arms = 0
        if fuse_on and pose_listener is not None and wrist_raw:
            if T_tcp_cam is None or seq % 30 == 0:
                T_tcp_cam = load_T_tcp_cam(T_TCP_CAM_PATH)
            if T_tcp_cam is not None:
                exs, ecs = [], []
                for arm, (wp, wc) in wrist_raw.items():
                    T_st = pose_listener.get(arm)
                    if T_st is None:
                        continue
                    # 팔 이동 중이면 융합 보류(head-only) — 비동기 misregistration 방지.
                    if fuse_motion_gate and not pose_listener.is_settled(arm, fuse_lin_thr, fuse_ang_thr):
                        gated_arms += 1
                        continue
                    T_sw = T_st @ T_tcp_cam
                    exs.append(wp.astype(np.float64) @ T_sw[:3, :3].T + T_sw[:3, 3]); ecs.append(wc)
                    fused_arms += 1
                if exs:
                    extra_xyz = np.concatenate(exs, 0); extra_rgb = np.concatenate(ecs, 0)
        _t = _ck(prof, "fuse", _t)

        if detector is not None:
            try:
                raw = detector.detect(disp, color_img=(col.pixels if col is not None else None),
                                      extra_xyz=extra_xyz, extra_rgb=extra_rgb)
                boxes = tracker.update(raw) if tracker is not None else raw
                n_boxes = len(boxes)
                pub.publish_boxes(seq, boxes)
                external_boxes_sender.send(boxes)
            except Exception as e:  # noqa: BLE001
                if seq % 60 == 0:
                    print(f"[run] detect err: {e}", flush=True)
            # settings.json에서 갱신된 ROI/Z캡이 바뀌면 1회 로그(GUI 수정이 반영됐는지 확인)
            clip_now = (detector._roi_x, detector._roi_y, detector._roi_z, detector._z_cap)
            if clip_now != last_clip_log:
                print(f"[run] clip RoI update: x={detector._roi_x} y={detector._roi_y} "
                      f"z={detector._roi_z} z_cap={detector._z_cap}", flush=True)
                last_clip_log = clip_now
        _t = _ck(prof, "detect", _t)

        # 손목 raw 클라우드 발행 (저rate, 카메라 프레임 — rb_gui가 TCP×hand-eye로 배치).
        # ROI 클립: state(T_st)+핸드아이가 있을 때만 stand 변환 가능 → 그때만 자른다.
        if wrist_on and seq % max(1, wrist_every) == 0:
            roi = effective_clip_roi(detector) if roi_clip_on else None
            for arm, (wp, wc) in wrist_raw.items():
                wpub, wcub = wp, wc
                if roi is not None and pose_listener is not None and T_tcp_cam is not None:
                    T_st = pose_listener.get(arm)
                    if T_st is not None:
                        wm = roi_clip_mask(wp, T_st @ T_tcp_cam, roi)
                        wpub, wcub = wp[wm], wc[wm]
                try:
                    pub.publish_wrist(arm, seq, wpub, wcub)
                except Exception:
                    pass
        _t = _ck(prof, "pubwrist", _t)
        seq += 1
        fps_n += 1
        if time.time() - fps_t > 1.0:
            nwin = max(fps_n, 1)
            fps = fps_n / (time.time() - fps_t); fps_t = time.time(); fps_n = 0
            fuse_str = ""
            if fuse_on and pose_listener is not None:
                fuse_str = (f" fuse[rx={pose_listener.rx_count} fused={fused_arms} "
                            f"gated={gated_arms}]")
            prof_str = ""
            if prof_on and prof:
                prof_str = "  ms/frame[" + " ".join(
                    f"{k}={1000.0*v/nwin:.0f}" for k, v in prof.items()) + "]"
            prof = {}
            dprof_str = ""
            if prof_on and detector is not None and detector._prof:
                dprof_str = "  detect[" + " ".join(
                    f"{k.strip()}={1000.0*v/nwin:.0f}" for k, v in detector._prof.items()) + "]"
                detector._prof = {}
            print(f"[run] fps={fps:.1f} cloud_pts={xyz.shape[0]} boxes={n_boxes} "
                  f"z[{xyz[:,2].min():.2f},{xyz[:,2].max():.2f}]m{fuse_str}{prof_str}{dprof_str}", flush=True)
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
