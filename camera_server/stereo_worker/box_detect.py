"""박스 검출 (워커) — stand-frame 후보 추출 + known-model ICP.

장면: 카메라 ~45° 비스듬 → 박스 옆면이 주로 보이고 윗면은 앞모서리만, 바닥 거의 안 보임.
로봇이 head view를 가릴 수 있어 부분 관측에도 robust해야 한다. 그래서:

  카메라점(disp) → stand 변환 → 테이블 평면 lstsq(평면-상대 높이, tilt 무관)
  → band[floor+Z_LO, floor+box_h] 후보 추출(테이블/로봇 제거)
  → 색: GREEN은 색으로, GRAY는 GREEN footprint 제외로 분리(인접 두 박스 분리)
  → 클러스터 → known-dims rect init(가림 보정) → FPS(torch GPU) 다운샘플
  → ICP(관측점 → 알려진 open-tray 전체 모델@init): 가려진 부분은 대응 없이도 OK
  → T_stand_box. (시간 안정화는 worker의 BoxTracker가 담당.)

검출은 stand 프레임에서 수행 → 수동 캘리브 T_stand_cam 품질에 직접 의존(현재 ~mm/1.7°).
"""
from __future__ import annotations
import json
import os
import time
import numpy as np

try:
    import cv2
except Exception:  # noqa: BLE001
    cv2 = None

try:
    import open3d as o3d
except Exception:  # noqa: BLE001
    o3d = None

BOX_DIMS = (0.380, 0.240, 0.110)
T_STAND_CAM_PATH = os.environ.get("STEREO_T_STAND_CAM", "/app/state/T_stand_cam.npy")
GUI_SETTINGS_PATH = os.environ.get("STEREO_GUI_SETTINGS", "/app/rb_gui_state/settings.json")
COLOR_CALIB_PATH = os.environ.get("STEREO_COLOR_CALIB", "/app/config/d435_color_calib.json")


def _load_T(path):
    try:
        T = np.load(path)
        if T.shape == (4, 4):
            return T.astype(np.float64)
    except Exception:
        pass
    return np.eye(4)


def _load_depth_range(path, default=(0.2, 3.0)):
    try:
        with open(path) as f:
            s = json.load(f)
        return float(s.get("pc_dmin", default[0])), float(s.get("pc_dmax", default[1]))
    except Exception:
        return default


def _load_color_calib(path):
    try:
        with open(path) as f:
            c = json.load(f)
        c["R"] = np.array(c["R_ir_to_color"], float)
        c["t"] = np.array(c["t_ir_to_color"], float)
        return c
    except Exception:
        return None


def _open_tray_model(dims=BOX_DIMS, wall=0.020, n=4000, rng=None):
    """open-top 트레이 표면 점 (박스 로컬: 중심 원점, z 위, 윗면 열림). ICP target."""
    rng = rng or np.random.default_rng(0)
    hx, hy, hz = dims[0]/2, dims[1]/2, dims[2]/2
    ix, iy = hx-wall, hy-wall
    floor_z = -hz + wall
    pts = []
    def rect(k, xlo, xhi, ylo, yhi, z):
        pts.append(np.stack([rng.uniform(xlo, xhi, k), rng.uniform(ylo, yhi, k), np.full(k, z)], 1))
    def wx(k, x, zlo, zhi):
        pts.append(np.stack([np.full(k, x), rng.uniform(-hy, hy, k), rng.uniform(zlo, zhi, k)], 1))
    def wy(k, y, zlo, zhi):
        pts.append(np.stack([rng.uniform(-hx, hx, k), np.full(k, y), rng.uniform(zlo, zhi, k)], 1))
    k = n // 8
    wx(k, -hx, -hz, hz); wx(k, hx, -hz, hz); wy(k, -hy, -hz, hz); wy(k, hy, -hz, hz)  # 바깥 4벽
    rect(k, -hx, hx, -hy, hy, hz)                       # 윗 림
    rect(k, -ix, ix, -iy, iy, floor_z)                  # 내부 바닥
    return np.concatenate(pts, 0)


class BoxDetector:
    DEPTH_PAD = 0.05               # depth max 위로 약간 여유(박스 뒷부분)
    Z_LO, Z_HI = 0.012, 0.125      # 평면-상대 높이 band (m): 테이블(아래)·로봇(위) 제거
    # stand 프레임 RoI (rb_gui Safety ROI 기본값) — 박스는 이 안에만 있음
    ROI_X = (-0.55, 0.55); ROI_Y = (-1.05, 0.05); ROI_Z = (-0.05, 1.0)
    GREEN_LO = np.array([35, 60, 40]); GREEN_HI = np.array([88, 255, 255])  # HSV(H:0-179)
    GRID = 0.006                   # XY occupancy 클러스터 해상도 (m/cell)
    MIN_CLUSTER_CELL = 150         # 클러스터 최소 cell 수
    MIN_PTS = 500                  # 박스 후보 최소 점 수
    SIZE_LONG = (0.18, 0.55)       # 관측 긴변 게이트 (m)
    FPS_K = 1024                   # ICP 전 FPS 다운샘플 점 수
    FPS_CAP = 8000                 # FPS 전 랜덤 상한(속도)
    ICP_MAX_CORR = 0.03            # ICP 대응 최대거리 (m) = 암묵 trimming
    ICP_ITERS = 40
    MAX_COMPONENTS = 8             # candidate ranking cost bound

    def __init__(self, K, baseline, use_icp=True, icp_method=None):
        self.fx, self.fy = K[0, 0], K[1, 1]
        self.cx, self.cy = K[0, 2], K[1, 2]
        self.baseline = baseline
        self.use_icp = use_icp and (o3d is not None)
        self.icp_method = self._normalize_icp_method(
            icp_method or os.environ.get("STEREO_DETECT_ICP_METHOD", "point_to_point")
        )
        self.model = _open_tray_model()
        self._uu = self._vv = None
        self._T_sc = _load_T(T_STAND_CAM_PATH)
        self._dmin, self._dmax = _load_depth_range(GUI_SETTINGS_PATH)
        self._calib = _load_color_calib(COLOR_CALIB_PATH)
        self._cfg_t = 0.0
        self._rng = np.random.default_rng(0)
        self._warned_torch = False
        self._warned_cuda = False
        self._warned_icp_method = False
        self._torch = None
        self._torch_cuda_ok = None

    @staticmethod
    def _normalize_icp_method(method):
        m = str(method or "point_to_point").strip().lower().replace("-", "_")
        aliases = {
            "p2p": "point_to_point",
            "point": "point_to_point",
            "point_to_point": "point_to_point",
            "point2point": "point_to_point",
            "p2l": "point_to_plane",
            "plane": "point_to_plane",
            "point_to_plane": "point_to_plane",
            "point2plane": "point_to_plane",
        }
        return aliases.get(m, "point_to_point")

    def _reload_cfg(self, now):
        if now - self._cfg_t > 1.0:
            self._T_sc = _load_T(T_STAND_CAM_PATH)
            self._dmin, self._dmax = _load_depth_range(GUI_SETTINGS_PATH)
            self._cfg_t = now

    # ---- 카메라 프레임 점/색 (기존 로직 유지) ----
    def _cam_points(self, disp):
        H, W = disp.shape
        if self._uu is None or self._uu.shape != disp.shape:
            uu, vv = np.meshgrid(np.arange(W), np.arange(H))
            self._uu, self._vv = uu.astype(np.float32), vv.astype(np.float32)
        with np.errstate(divide="ignore", invalid="ignore"):
            z = self.fx * self.baseline / disp
        valid = np.isfinite(z) & (z >= self._dmin) & (z <= self._dmax + self.DEPTH_PAD)
        x = (self._uu - self.cx) / self.fx * z
        y = (self._vv - self.cy) / self.fy * z
        return np.stack([x, y, z], -1), valid

    def _organized_rgb(self, P, color_img, calib):
        """각 점(IR 프레임)을 color 카메라에 재투영해 RGB 샘플 (organized HxWx3) + 유효 마스크."""
        Pc = P @ calib["R"].T + calib["t"]
        Zc = Pc[..., 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            uc = calib["color_fx"] * Pc[..., 0] / Zc + calib["color_cx"]
            vc = calib["color_fy"] * Pc[..., 1] / Zc + calib["color_cy"]
        Hc, Wc = color_img.shape[:2]
        ok = (Zc > 0) & (uc >= 0) & (uc < Wc - 1) & (vc >= 0) & (vc < Hc - 1)
        ui = np.clip(uc, 0, Wc - 1).astype(np.int32)
        vi = np.clip(vc, 0, Hc - 1).astype(np.int32)
        return color_img[vi, ui], ok

    # ---- stand 프레임 기하 ----
    @staticmethod
    def _fit_floor_plane(P):
        """저높이 점에 z=ax+by+c lstsq → (a,b,c). 평면-상대 높이 = z-(ax+by+c)."""
        z0 = np.percentile(P[:, 2], 5)
        tab = P[P[:, 2] < z0 + 0.02]
        if len(tab) < 500:
            return None
        A = np.c_[tab[:, 0], tab[:, 1], np.ones(len(tab))]
        coef, *_ = np.linalg.lstsq(A, tab[:, 2], rcond=None)
        return coef

    def _cluster_xy(self, P_xy):
        """XY occupancy grid → 연결성분. (점별 라벨, 면적내림차순 성분 id 목록)."""
        gx = ((P_xy[:, 0] - self.ROI_X[0]) / self.GRID).astype(np.int32)
        gy = ((P_xy[:, 1] - self.ROI_Y[0]) / self.GRID).astype(np.int32)
        W = int((self.ROI_X[1] - self.ROI_X[0]) / self.GRID) + 2
        H = int((self.ROI_Y[1] - self.ROI_Y[0]) / self.GRID) + 2
        inb = (gx >= 0) & (gx < W) & (gy >= 0) & (gy < H)
        occ = np.zeros((H, W), np.uint8)
        occ[gy[inb], gx[inb]] = 255
        occ = cv2.morphologyEx(occ, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        occ = cv2.morphologyEx(occ, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        ncc, lab, stats, _ = cv2.connectedComponentsWithStats(occ, 8)
        order = sorted(range(1, ncc), key=lambda c: -stats[c, cv2.CC_STAT_AREA])
        keep = [c for c in order if stats[c, cv2.CC_STAT_AREA] >= self.MIN_CLUSTER_CELL]
        pl = np.where(inb, lab[np.clip(gy, 0, H - 1), np.clip(gx, 0, W - 1)], 0)
        return pl, keep

    def _init_fit(self, P_xy, cam_xy, plane):
        """평면상 minAreaRect → known-dims + 가림 보정 → init pose(4x4 box→stand) + obs footprint."""
        rect = cv2.minAreaRect((P_xy * 1000).astype(np.float32))
        (cx, cy), (w, hgt), ang = rect
        cx, cy, w, hgt = cx/1000, cy/1000, w/1000, hgt/1000
        if w >= hgt:
            obs_long, obs_short, yaw = w, hgt, np.radians(ang)
        else:
            obs_long, obs_short, yaw = hgt, w, np.radians(ang + 90.0)
        if not (self.SIZE_LONG[0] <= obs_long <= self.SIZE_LONG[1]) or obs_short > 0.40:
            return None, None
        a, b, c = plane
        n = np.array([-a, -b, 1.0]); n /= np.linalg.norm(n)
        x0 = np.array([np.cos(yaw), np.sin(yaw), 0.0])
        x = x0 - n * (x0 @ n); x /= np.linalg.norm(x)      # 긴변(0.38) 방향(평면 투영)
        y = np.cross(n, x)                                  # 짧은변(0.24)
        center2 = np.array([cx, cy])
        xy = lambda v: v[:2] / (np.linalg.norm(v[:2]) + 1e-9)
        for ax2, obs, full in ((xy(x), obs_long, BOX_DIMS[0]), (xy(y), obs_short, BOX_DIMS[1])):
            push = max(0.0, (full - obs) / 2.0)             # 관측<치수면 카메라 반대(가림)쪽으로
            if push > 1e-4:
                away = ax2 if ax2 @ (center2 - cam_xy) > 0 else -ax2
                center2 = center2 + away * push
        cz = a * center2[0] + b * center2[1] + c
        center = np.array([center2[0], center2[1], cz]) + n * (BOX_DIMS[2] / 2.0)
        T = np.eye(4); T[:3, :3] = np.column_stack([x, y, n]); T[:3, 3] = center
        return T, (round(obs_long, 3), round(obs_short, 3))

    def _fps(self, pts, k):
        """farthest point sampling. torch(GPU) 우선, 실패 시 numpy 폴백."""
        if len(pts) <= k:
            return pts.astype(np.float64)
        if len(pts) > self.FPS_CAP:
            pts = pts[self._rng.choice(len(pts), self.FPS_CAP, replace=False)]
        if self._torch_cuda_ok is None:
            try:
                import torch
                self._torch = torch
                self._torch_cuda_ok = bool(torch.cuda.is_available())
            except Exception as e:  # noqa: BLE001
                self._torch = None
                self._torch_cuda_ok = False
                if not self._warned_torch:
                    print(f"[box_detect] torch FPS 불가 → numpy 폴백: {e}", flush=True)
                    self._warned_torch = True
        if not self._torch_cuda_ok:
            if self._torch is not None and not self._warned_cuda:
                print("[box_detect] CUDA torch FPS 비활성 → numpy 폴백", flush=True)
                self._warned_cuda = True
            return self._fps_numpy(pts, k)
        try:
            t = self._torch.as_tensor(pts, device="cuda", dtype=self._torch.float32)
            n = t.shape[0]
            idx = self._torch.empty(k, dtype=self._torch.long, device=t.device)
            d = self._torch.full((n,), 1e10, device=t.device)
            far = self._torch.zeros((), dtype=self._torch.long, device=t.device)
            for i in range(k):
                idx[i] = far
                d = self._torch.minimum(d, ((t - t[far]) ** 2).sum(1))
                far = self._torch.argmax(d)
            return t[idx].double().cpu().numpy()
        except Exception as e:  # noqa: BLE001
            if not self._warned_torch:
                print(f"[box_detect] torch FPS 불가 → numpy 폴백: {e}", flush=True)
                self._warned_torch = True
            return self._fps_numpy(pts, k)

    @staticmethod
    def _fps_numpy(pts, k):
        n = len(pts)
        idx = np.empty(k, np.int64); idx[0] = 0
        d = np.full(n, np.inf)
        for i in range(1, k):
            d = np.minimum(d, np.sum((pts - pts[idx[i - 1]]) ** 2, 1))
            idx[i] = int(np.argmax(d))
        return pts[idx].astype(np.float64)

    def _icp(self, scene, T0):
        """관측점(scene) → 알려진 open-tray 모델@init ICP. refined = inv(T_icp)@T0."""
        s = self._fps(scene, self.FPS_K)
        Mw = self.model @ T0[:3, :3].T + T0[:3, 3]
        src = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(s))
        tgt = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(Mw))
        method = self.icp_method
        if method == "point_to_plane":
            try:
                tgt.estimate_normals(
                    o3d.geometry.KDTreeSearchParamHybrid(radius=0.06, max_nn=30)
                )
                estimator = o3d.pipelines.registration.TransformationEstimationPointToPlane()
            except Exception as e:  # noqa: BLE001
                if not self._warned_icp_method:
                    print(f"[box_detect] point-to-plane ICP 불가 → point-to-point 폴백: {e}", flush=True)
                    self._warned_icp_method = True
                method = "point_to_point_fallback"
                estimator = o3d.pipelines.registration.TransformationEstimationPointToPoint()
        else:
            estimator = o3d.pipelines.registration.TransformationEstimationPointToPoint()
        try:
            reg = o3d.pipelines.registration.registration_icp(
                src, tgt, self.ICP_MAX_CORR, np.eye(4),
                estimator,
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=self.ICP_ITERS))
        except Exception as e:  # noqa: BLE001
            if method != "point_to_plane":
                raise
            if not self._warned_icp_method:
                print(f"[box_detect] point-to-plane ICP 실패 → point-to-point 폴백: {e}", flush=True)
                self._warned_icp_method = True
            method = "point_to_point_fallback"
            reg = o3d.pipelines.registration.registration_icp(
                src, tgt, self.ICP_MAX_CORR, np.eye(4),
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=self.ICP_ITERS))
        return (np.linalg.inv(reg.transformation) @ T0,
                float(reg.fitness),
                float(getattr(reg, "inlier_rmse", np.nan)),
                method,
                int(len(s)))

    def _component_candidates(self, pts, cam_xy, plane, label):
        """후보 점을 cluster별 init-fit 후보로 변환한다. ICP는 최종 선택 후 한 번만 수행."""
        if len(pts) < self.MIN_PTS:
            return []
        pl, comps = self._cluster_xy(pts[:, :2])
        out = []
        for cid in comps[:self.MAX_COMPONENTS]:
            m = pl == cid
            n = int(m.sum())
            if n < self.MIN_PTS:
                continue
            scene = pts[m]
            T0, foot = self._init_fit(scene[:, :2], cam_xy, plane)
            if T0 is None:
                continue
            out.append(dict(scene=scene, T0=T0, footprint=foot, n=n, label=label))
        return out

    @staticmethod
    def _candidate_fit_score(cand):
        """known dims에 맞는 cluster를 선호하되, partial observation은 허용한다."""
        obs_long, obs_short = cand["footprint"]
        long_cov = min(obs_long / BOX_DIMS[0], 1.0)
        short_cov = min(obs_short / BOX_DIMS[1], 1.0)
        long_over = max(0.0, obs_long - BOX_DIMS[0]) / BOX_DIMS[0]
        short_over = max(0.0, obs_short - BOX_DIMS[1]) / BOX_DIMS[1]
        n_bonus = min(cand["n"] / 6000.0, 1.0)
        oversize_penalty = 8.0 * long_over * long_over + 12.0 * short_over * short_over
        return 0.55 * long_cov + 0.35 * short_cov + 0.10 * n_bonus - oversize_penalty

    def _select_candidate(self, pts, cam_xy, plane, label, prefer_fit=False):
        cands = self._component_candidates(pts, cam_xy, plane, label)
        if not cands:
            return None
        if prefer_fit:
            return max(cands, key=self._candidate_fit_score)
        return max(cands, key=lambda c: (c["n"], self._candidate_fit_score(c)))

    def _detect_one(self, pts, cam_xy, plane, label, prefer_fit=False):
        """후보 점 → cluster ranking → init fit → ICP → box dict (RoI 게이트)."""
        cand = self._select_candidate(pts, cam_xy, plane, label, prefer_fit=prefer_fit)
        if cand is None:
            return None
        scene = cand["scene"]
        T0 = cand["T0"]
        foot = cand["footprint"]
        if T0 is None:
            return None
        if self.use_icp:
            T, fit, rmse, method, sample_n = self._icp(scene, T0)
        else:
            T, fit, rmse, method, sample_n = T0, 1.0, None, "disabled", 0
        c = T[:3, 3]
        if not (self.ROI_X[0] < c[0] < self.ROI_X[1] and self.ROI_Y[0] < c[1] < self.ROI_Y[1]
                and self.ROI_Z[0] < c[2] < self.ROI_Z[1]):
            return None
        return dict(T=T, dims=BOX_DIMS, footprint=foot, n=int(len(scene)),
                    source_n=int(len(scene)), icp_sample_n=int(sample_n),
                    fitness=round(fit, 3),
                    rmse=(None if rmse is None or not np.isfinite(rmse) else round(float(rmse), 5)),
                    icp_method=method, label=label)

    def detect(self, disp, color_img=None):
        if cv2 is None:
            return []
        now = time.time(); self._reload_cfg(now)
        Pc, valid = self._cam_points(disp)                       # 카메라 프레임 organized
        Ps = Pc @ self._T_sc[:3, :3].T + self._T_sc[:3, 3]       # stand 프레임
        inroi = ((Ps[..., 0] > self.ROI_X[0]) & (Ps[..., 0] < self.ROI_X[1]) &
                 (Ps[..., 1] > self.ROI_Y[0]) & (Ps[..., 1] < self.ROI_Y[1]))

        have_color = color_img is not None and self._calib is not None
        if have_color:
            rgb, cok = self._organized_rgb(Pc, color_img, self._calib)
            m = valid & cok & inroi
        else:
            m = valid & inroi
        if int(m.sum()) < 2000:
            return []
        P = Ps[m]
        plane = self._fit_floor_plane(P)
        if plane is None:
            return []
        a, b, c = plane
        h = P[:, 2] - (a * P[:, 0] + b * P[:, 1] + c)
        band = (h > self.Z_LO) & (h < self.Z_HI)
        if int(band.sum()) < self.MIN_PTS:
            return []
        Pb = P[band]
        cam_xy = self._T_sc[:2, 3]

        if not have_color:
            # 색 없음 → 기하만: band 전체 클러스터별 ICP (라벨 None)
            out = []
            pl, comps = self._cluster_xy(Pb[:, :2])
            for cid in comps:
                mm = pl == cid
                b_ = self._detect_one(Pb[mm], cam_xy, plane, None)
                if b_:
                    out.append(b_)
            return out

        C = rgb[m][band]
        hsv = cv2.cvtColor(np.ascontiguousarray(C).reshape(-1, 1, 3), cv2.COLOR_RGB2HSV).reshape(-1, 3)
        is_green = cv2.inRange(hsv, self.GREEN_LO, self.GREEN_HI).reshape(-1) > 0

        out = []
        green_center = None
        if int(is_green.sum()) >= self.MIN_PTS:
            g = self._detect_one(Pb[is_green], cam_xy, plane, "green")
            if g:
                out.append(g); green_center = g["T"][:2, 3]
        keep = ~is_green
        if green_center is not None:                              # 초록 박스 영역 제외(회색 분리)
            r = 0.5 * float(np.hypot(BOX_DIMS[0], BOX_DIMS[1])) * 0.92
            keep &= np.linalg.norm(Pb[:, :2] - green_center, axis=1) > r
        if int(keep.sum()) >= self.MIN_PTS:
            gr = self._detect_one(Pb[keep], cam_xy, plane, "gray", prefer_fit=True)
            if gr:
                out.append(gr)
        return out


class BoxTracker:
    """검출 깜빡임 안정화: 지속 트랙(최대 N개) + 근접 연관 + EMA 스무딩 + coasting.

    update(dets) -> 안정화된 박스 리스트(confirmed). 한 프레임 미검출(가림)이어도 트랙은
    last pose를 유지(coast)하므로 rb_gui에 깜빡임 없이 표시된다.
    """
    def __init__(self, ema=0.6, gate=0.25, min_hits=3, max_miss=12, max_tracks=2):
        self.ema = ema; self.gate = gate
        self.min_hits = min_hits; self.max_miss = max_miss; self.max_tracks = max_tracks
        self.tracks = []
        self._id = 0

    @staticmethod
    def _decomp(T):
        c = T[:3, 3].astype(float)
        x = T[:3, 0].astype(float); up = T[:3, 2].astype(float)
        return c, x/np.linalg.norm(x), up/np.linalg.norm(up)

    def update(self, dets):
        obs = []
        for d in dets:
            c, x, up = self._decomp(d["T"])
            obs.append([c, x, up, d])
        used = set()
        for tr in self.tracks:
            best, bd = -1, self.gate
            for i, (c, x, up, d) in enumerate(obs):
                if i in used:
                    continue
                dist = np.linalg.norm(c - tr["center"])
                if dist < bd:
                    bd, best = dist, i
            if best >= 0:
                used.add(best); c, x, up, d = obs[best]
                if x @ tr["x_ax"] < 0:               # 180° 뒤집힘 정렬
                    x = -x
                a = self.ema
                tr["center"] = a*tr["center"] + (1-a)*c
                xx = a*tr["x_ax"] + (1-a)*x; tr["x_ax"] = xx/np.linalg.norm(xx)
                uu = a*tr["up"] + (1-a)*up; tr["up"] = uu/np.linalg.norm(uu)
                tr["hits"] += 1; tr["miss"] = 0
                tr["foot"] = d["footprint"]; tr["n"] = d["n"]; tr["label"] = d.get("label")
                tr["fitness"] = d.get("fitness")
                tr["rmse"] = d.get("rmse")
                tr["icp_method"] = d.get("icp_method")
                tr["source_n"] = d.get("source_n", d.get("n"))
                tr["icp_sample_n"] = d.get("icp_sample_n")
            else:
                tr["miss"] += 1
        for i, (c, x, up, d) in enumerate(obs):       # 새 트랙
            if i in used or len(self.tracks) >= self.max_tracks:
                continue
            self._id += 1
            self.tracks.append(dict(center=c, x_ax=x, up=up, hits=1, miss=0,
                                    foot=d["footprint"], n=d["n"], id=self._id,
                                    label=d.get("label"), fitness=d.get("fitness"),
                                    rmse=d.get("rmse"), icp_method=d.get("icp_method"),
                                    source_n=d.get("source_n", d.get("n")),
                                    icp_sample_n=d.get("icp_sample_n")))
        self.tracks = [t for t in self.tracks if t["miss"] <= self.max_miss]
        out = []
        for t in self.tracks:
            if t["hits"] >= self.min_hits:
                up = t["up"]; x = t["x_ax"]
                x = x - up*(x @ up); x /= np.linalg.norm(x)   # up에 직교화
                y = np.cross(up, x)
                T = np.eye(4); T[:3, :3] = np.column_stack([x, y, up]); T[:3, 3] = t["center"]
                out.append(dict(T=T, dims=BOX_DIMS, footprint=t["foot"], n=t["n"],
                                fitness=t.get("fitness"), rmse=t.get("rmse"),
                                icp_method=t.get("icp_method"),
                                source_n=t.get("source_n", t.get("n")),
                                icp_sample_n=t.get("icp_sample_n"),
                                track_id=t["id"], coasting=t["miss"] > 0,
                                label=t.get("label")))
        return out
