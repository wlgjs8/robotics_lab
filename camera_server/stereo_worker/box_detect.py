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

BOX_DIMS = (0.380, 0.240, 0.105)   # NPC NTC-321 외부 실측(380×240×105mm). 측벽 20mm, 바닥 6.5mm.
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


def _load_roi(path, default):
    """settings.json의 roi_min_m/roi_max_m(stand 프레임 m, [x,y,z]) → ((x0,x1),(y0,y1),(z0,z1)).
    rb_gui Safety ROI 박스를 검출 영역으로 공유한다. 없거나 형식 불일치 시 default."""
    try:
        with open(path) as f:
            s = json.load(f)
        lo, hi = s.get("roi_min_m"), s.get("roi_max_m")
        if (isinstance(lo, (list, tuple)) and isinstance(hi, (list, tuple))
                and len(lo) == 3 and len(hi) == 3):
            lo = [float(v) for v in lo]
            hi = [float(v) for v in hi]
            if all(lo[i] <= hi[i] for i in range(3)):
                return ((lo[0], hi[0]), (lo[1], hi[1]), (lo[2], hi[2]))
    except Exception:
        pass
    return default


def _load_z_cap(path, default=None):
    """settings.json의 pc_clip_z_max_m(stand 프레임 m) — 점 단위 stand-Z 상한.
    Safety ROI(로봇 safety envelope)와 독립한 perception 전용 높이 컷(로봇팔/배경 noise
    제거). env STEREO_CLIP_Z_MAX로도 지정 가능. 없으면 default(None=상한 없음)."""
    env = os.environ.get("STEREO_CLIP_Z_MAX")
    try:
        with open(path) as f:
            v = json.load(f).get("pc_clip_z_max_m")
        if v is not None:
            return float(v)
    except Exception:
        pass
    if env not in (None, ""):
        try:
            return float(env)
        except (TypeError, ValueError):
            pass
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


def _open_tray_model(dims=BOX_DIMS, wall=0.020, floor=0.0065, n=6500, rng=None):
    """open-top 트레이 **쉘** 표면 점 (박스 로컬: 중심 원점, z 위, 윗면 열림). ICP target.

    카메라가 ~45°로 트레이를 보면 가까운 벽은 **바깥면**, 먼 벽은 (열린 위로 들여다봐)
    **안쪽면**이 관측된다. 모델이 벽을 바깥면만 두면 먼-안쪽 관측이 바깥 모델로 끌려가
    포즈가 카메라쪽으로 ~벽두께만큼 편향되고 yaw가 틀어진다. 그래서 내벽·외벽·림 링·
    내부 바닥을 모두 둔 쉘로 만들어 가까운-바깥/먼-안쪽 관측이 각자 올바른 면에 대응되게
    한다(검증: 벽두께 편향 교정 + ICP 잔차 절반). 바닥(외부)은 테이블에 가려 제외."""
    rng = rng or np.random.default_rng(0)
    hx, hy, hz = dims[0]/2, dims[1]/2, dims[2]/2
    ix, iy = hx-wall, hy-wall                           # 측벽 두께(NTC-321 20mm)
    fz = -hz + floor                                    # 내부 바닥 높이(바닥 두께 6.5mm)
    pts = []
    def wx(k, x, ylo, yhi, zlo, zhi):
        pts.append(np.stack([np.full(k, x), rng.uniform(ylo, yhi, k), rng.uniform(zlo, zhi, k)], 1))
    def wy(k, y, xlo, xhi, zlo, zhi):
        pts.append(np.stack([rng.uniform(xlo, xhi, k), np.full(k, y), rng.uniform(zlo, zhi, k)], 1))
    def rz(k, xlo, xhi, ylo, yhi, z):
        pts.append(np.stack([rng.uniform(xlo, xhi, k), rng.uniform(ylo, yhi, k), np.full(k, z)], 1))
    k = n // 13
    # 외벽 4 (전체 높이)
    wx(k, -hx, -hy, hy, -hz, hz); wx(k, hx, -hy, hy, -hz, hz)
    wy(k, -hy, -hx, hx, -hz, hz); wy(k, hy, -hx, hx, -hz, hz)
    # 내벽 4 (내부 바닥~윗면)
    wx(k, -ix, -iy, iy, fz, hz); wx(k, ix, -iy, iy, fz, hz)
    wy(k, -iy, -ix, ix, fz, hz); wy(k, iy, -ix, ix, fz, hz)
    # 윗 림 링 (z=hz, 내외 사이 4 strip — 솔리드 top 아님)
    rz(k, -hx, hx, iy, hy, hz); rz(k, -hx, hx, -hy, -iy, hz)
    rz(k, ix, hx, -iy, iy, hz); rz(k, -hx, -ix, -iy, iy, hz)
    # 내부 바닥
    rz(k, -ix, ix, -iy, iy, fz)
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
    SIZE_LONG = (0.16, 0.60)       # 관측 긴변 게이트 (m): 붙은 2박스(~0.8)는 거르되 부분관측 허용
    SIZE_SHORT_MAX = 0.42          # 관측 짧은변 상한 (m)
    FPS_K = 1024                   # ICP 전 FPS 다운샘플 점 수
    FPS_CAP = 8000                 # FPS 전 랜덤 상한(속도)
    ICP_MAX_CORR = 0.03            # ICP 대응 최대거리 (m) = 암묵 trimming
    ICP_ITERS = 40
    ICP_TRIM = 0.7                 # SE(2) ICP 대응 trim 비율(가까운 70%만 사용)
    ICP_CLIP_MARGIN = 0.04         # ICP 입력을 박스 근방 3D RoI로 클립(박스 바깥 점 배제)
    ICP_PASSES = 2                 # 클립→ICP→재클립→ICP 반복 횟수
    MAX_COMPONENTS = 8             # candidate ranking cost bound
    # 회색(colorless 조밀 박스) de-bridge OPEN 커널 (px). CLOSE 제거만으로 분리는
    # 되지만, OPEN으로 얇은 연결부를 한 번 더 끊는다. 클수록 강하게 끊지만 희박/가림
    # 회색 박스를 침식한다(3≈거의 보존, 5≈치수 깔끔하나 ~20% 침식, 7≈과침식). env로 튜닝.
    GRAY_DEBRIDGE_OPEN_K = 3
    TEMPORAL_GATE_M = 0.10          # 직전 포즈를 ICP init으로 재사용할 최대 거리(이내=추적, 초과=재획득)

    def __init__(self, K, baseline, use_icp=True, icp_method=None):
        self.fx, self.fy = K[0, 0], K[1, 1]
        self.cx, self.cy = K[0, 2], K[1, 2]
        self.baseline = baseline
        self.use_icp = use_icp and (o3d is not None)
        self.icp_method = self._normalize_icp_method(
            icp_method or os.environ.get("STEREO_DETECT_ICP_METHOD", "yaw_se2")
        )
        self.model = _open_tray_model()
        try:
            self._gray_open_k = max(0, int(os.environ.get("STEREO_GRAY_OPEN_K",
                                                          self.GRAY_DEBRIDGE_OPEN_K)))
        except (TypeError, ValueError):
            self._gray_open_k = self.GRAY_DEBRIDGE_OPEN_K
        self._uu = self._vv = None
        self._T_sc = _load_T(T_STAND_CAM_PATH)
        self._dmin, self._dmax = _load_depth_range(GUI_SETTINGS_PATH)
        # 검출 RoI: rb_gui Safety ROI 박스(settings.json)를 공유. 없으면 클래스 기본값.
        self._roi_x, self._roi_y, self._roi_z = _load_roi(
            GUI_SETTINGS_PATH, (self.ROI_X, self.ROI_Y, self.ROI_Z))
        self._z_cap = _load_z_cap(GUI_SETTINGS_PATH)   # stand-Z 점 상한(None=off)
        self._temporal_icp = os.environ.get("STEREO_TEMPORAL_ICP", "1") != "0"
        self._prev_pose = {}                           # label -> 직전 ICP 포즈(temporal init)
        self._last_plane = None                        # 직전 양호 테이블 평면(가림 시 fallback)
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
        m = str(method or "yaw_se2").strip().lower().replace("-", "_")
        aliases = {
            "yaw": "yaw_se2", "se2": "yaw_se2", "yaw_se2": "yaw_se2",
            "yaw_constrained": "yaw_se2", "planar": "yaw_se2",
            "p2p": "point_to_point", "point": "point_to_point",
            "point_to_point": "point_to_point", "point2point": "point_to_point",
            "p2l": "point_to_plane", "plane": "point_to_plane",
            "point_to_plane": "point_to_plane", "point2plane": "point_to_plane",
        }
        return aliases.get(m, "yaw_se2")

    def _reload_cfg(self, now):
        if now - self._cfg_t > 1.0:
            self._T_sc = _load_T(T_STAND_CAM_PATH)
            self._dmin, self._dmax = _load_depth_range(GUI_SETTINGS_PATH)
            self._roi_x, self._roi_y, self._roi_z = _load_roi(
                GUI_SETTINGS_PATH, (self.ROI_X, self.ROI_Y, self.ROI_Z))
            self._z_cap = _load_z_cap(GUI_SETTINGS_PATH)
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

    def _cluster_xy(self, P_xy, bridge=True):
        """XY occupancy grid → 연결성분. (점별 라벨, 면적내림차순 성분 id 목록).

        bridge=True (초록 등 색-세그 희박 점): MORPH_CLOSE로 마스크의 구멍/틈을 메워
          한 박스의 흩어진 점을 한 성분으로 잇는다.
        bridge=False (회색=colorless 조밀 박스): CLOSE는 박스를 테이블/잡음과 이어
          oversize 덩어리로 만들어 size gate에 걸리게 한다. CLOSE를 빼고 OPEN을 키워
          얇은 연결부를 끊고 박스를 독립 성분으로 분리한다.
        """
        gx = ((P_xy[:, 0] - self._roi_x[0]) / self.GRID).astype(np.int32)
        gy = ((P_xy[:, 1] - self._roi_y[0]) / self.GRID).astype(np.int32)
        W = int((self._roi_x[1] - self._roi_x[0]) / self.GRID) + 2
        H = int((self._roi_y[1] - self._roi_y[0]) / self.GRID) + 2
        inb = (gx >= 0) & (gx < W) & (gy >= 0) & (gy < H)
        occ = np.zeros((H, W), np.uint8)
        occ[gy[inb], gx[inb]] = 255
        if bridge:
            occ = cv2.morphologyEx(occ, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
            occ = cv2.morphologyEx(occ, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        elif self._gray_open_k >= 1:
            k = self._gray_open_k
            occ = cv2.morphologyEx(occ, cv2.MORPH_OPEN, np.ones((k, k), np.uint8))
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
        if not (self.SIZE_LONG[0] <= obs_long <= self.SIZE_LONG[1]) or obs_short > self.SIZE_SHORT_MAX:
            return None, None
        a, b, c = plane
        # 박스는 User Floor(수평) 위 → up축 = stand z (0,0,1) 고정(평면 tilt를 자세에 안 넣음).
        n = np.array([0.0, 0.0, 1.0])
        x = np.array([np.cos(yaw), np.sin(yaw), 0.0])      # 긴변(0.38) 방향(수평)
        y = np.array([-np.sin(yaw), np.cos(yaw), 0.0])     # 짧은변(0.24)
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

    def _icp_se2(self, s, T0):
        """yaw-고정 ICP: box up = stand z 고정, (x,y,yaw)만 정합(SE(2)). User Floor 위 박스용.
        DOF가 적어 부분관측/노이즈에 robust. NN은 scipy cKDTree, 갱신은 2D Kabsch."""
        try:
            from scipy.spatial import cKDTree
        except Exception as e:  # noqa: BLE001
            if not getattr(self, "_warned_se2", False):
                print(f"[box_detect] scipy 없음 → yaw_se2 ICP 불가, point-to-point 폴백: {e}", flush=True)
                self._warned_se2 = True
            return self._icp_o3d(s, T0, "point_to_point")
        Mw = self.model @ T0[:3, :3].T + T0[:3, 3]
        tree = cKDTree(Mw)
        S = s.copy(); Tacc = np.eye(4); rmse = float("nan"); inl = 1.0
        for _ in range(self.ICP_ITERS):
            d, j = tree.query(S)
            thr = np.quantile(d, self.ICP_TRIM); m = d <= thr
            inl = float(m.mean()); rmse = float(np.sqrt(float((d[m] ** 2).mean())))
            sp, dp = S[m, :2], Mw[j[m], :2]
            cs, cd = sp.mean(0), dp.mean(0)
            H = (sp - cs).T @ (dp - cd)
            phi = np.arctan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
            Rz = np.array([[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]])
            t2 = cd - Rz @ cs
            S[:, :2] = S[:, :2] @ Rz.T + t2                 # z·up 불변
            Tn = np.eye(4); Tn[:2, :2] = Rz; Tn[:2, 3] = t2; Tacc = Tn @ Tacc
            if abs(phi) < 1e-5 and float(np.linalg.norm(t2)) < 1e-5:
                break
        return np.linalg.inv(Tacc) @ T0, inl, rmse, "yaw_se2", int(len(s))

    def _clip_to_box(self, scene, T, margin):
        """scene 중 박스(@T) 로컬 근방 3D RoI 안의 점만 남긴다(박스 바깥 점 배제)."""
        loc = np.abs((scene - T[:3, 3]) @ T[:3, :3])
        inb = ((loc[:, 0] < BOX_DIMS[0] / 2 + margin) &
               (loc[:, 1] < BOX_DIMS[1] / 2 + margin) &
               (loc[:, 2] < BOX_DIMS[2] / 2 + margin))
        return scene[inb]

    def _icp(self, scene, T0):
        """관측점(scene) → 모델 ICP. 박스 근방 3D RoI 클립으로 바깥 점 배제 + 다패스 정제.
        refined = inv(T_icp)@T (패스마다 현 pose 기준 재클립)."""
        T, fit, rmse, meth, sample_n = T0, 1.0, float("nan"), self.icp_method, 0
        for _ in range(max(1, self.ICP_PASSES)):
            sc = self._clip_to_box(scene, T, self.ICP_CLIP_MARGIN)
            if len(sc) < self.MIN_PTS:                 # 클립이 과하면 원본 사용
                sc = scene
            s = self._fps(sc, self.FPS_K)
            if self.icp_method == "yaw_se2":
                T, fit, rmse, meth, sample_n = self._icp_se2(s, T)
            else:
                T, fit, rmse, meth, sample_n = self._icp_o3d(s, T, self.icp_method)
        return T, fit, rmse, meth, sample_n

    def _icp_o3d(self, s, T0, method):
        Mw = self.model @ T0[:3, :3].T + T0[:3, 3]
        src = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(s))
        tgt = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(Mw))
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

    def _component_candidates(self, pts, cam_xy, plane, label, bridge=True):
        """후보 점을 cluster별 init-fit 후보로 변환한다. ICP는 최종 선택 후 한 번만 수행."""
        if len(pts) < self.MIN_PTS:
            return []
        pl, comps = self._cluster_xy(pts[:, :2], bridge=bridge)
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

    @staticmethod
    def _center_in_box(T, box_T, margin=0.04):
        """T의 중심이 box_T(known-dims) footprint(+margin) 안에 있는가 (회색이 초록
        잔여물을 무는 것을 막는 reject 게이트용)."""
        dl = T[:2, 3] - box_T[:2, 3]
        return (abs(dl @ box_T[:2, 0]) < BOX_DIMS[0] / 2 + margin and
                abs(dl @ box_T[:2, 1]) < BOX_DIMS[1] / 2 + margin)

    def _select_candidate(self, pts, cam_xy, plane, label, prefer_fit=False,
                          bridge=True, reject_box=None):
        cands = self._component_candidates(pts, cam_xy, plane, label, bridge=bridge)
        if reject_box is not None:
            cands = [c for c in cands if not self._center_in_box(c["T0"], reject_box)]
        if not cands:
            return None
        if prefer_fit:
            return max(cands, key=self._candidate_fit_score)
        return max(cands, key=lambda c: (c["n"], self._candidate_fit_score(c)))

    def _detect_one(self, pts, cam_xy, plane, label, prefer_fit=False,
                    bridge=True, reject_box=None):
        """후보 점 → cluster ranking → init fit → ICP → box dict (RoI 게이트)."""
        cand = self._select_candidate(pts, cam_xy, plane, label, prefer_fit=prefer_fit,
                                      bridge=bridge, reject_box=reject_box)
        if cand is None:
            return None
        scene = cand["scene"]
        T0 = cand["T0"]
        foot = cand["footprint"]
        if T0 is None:
            return None
        if self.use_icp:
            # tracking-by-registration: 직전 포즈가 minAreaRect init 근방이면 그걸 ICP init으로
            # 써서, 매 프레임 노이즈 낀 init을 처음부터 fit하던 jitter를 없앤다(정지 박스 ~안정).
            # 멀면(박스 점프/신규/가림복귀) minAreaRect로 재획득. label별로 트랙 유지.
            init = T0
            prev = self._prev_pose.get(label) if (self._temporal_icp and label is not None) else None
            if prev is not None and float(np.linalg.norm(prev[:2, 3] - T0[:2, 3])) < self.TEMPORAL_GATE_M:
                init = prev
            T, fit, rmse, method, sample_n = self._icp(scene, init)
            if self._temporal_icp and label is not None:
                self._prev_pose[label] = T          # 다음 프레임 ICP init
        else:
            T, fit, rmse, method, sample_n = T0, 1.0, None, "disabled", 0
        c = T[:3, 3]
        if not (self._roi_x[0] < c[0] < self._roi_x[1] and self._roi_y[0] < c[1] < self._roi_y[1]
                and self._roi_z[0] < c[2] < self._roi_z[1]):
            return None
        return dict(T=T, dims=BOX_DIMS, footprint=foot, n=int(len(scene)),
                    source_n=int(len(scene)), icp_sample_n=int(sample_n),
                    fitness=round(fit, 3),
                    rmse=(None if rmse is None or not np.isfinite(rmse) else round(float(rmse), 5)),
                    icp_method=method, label=label)

    def detect(self, disp, color_img=None, extra_xyz=None, extra_rgb=None):
        """결합 클라우드(head disp + 손목 stand 점) 박스 검출. head·손목을 **먼저 합쳐서**
        게이트/평면/band/색분할/검출을 결합 기준으로 수행 → 로봇이 head를 가려도 손목 점으로
        검출이 이어진다. 손목 점(extra_xyz/rgb)은 worker가 T_stand_tcp×hand-eye로 stand에
        놓아 전달하며, head와 같은 bundle 캡처라 시점이 동기화돼 있다."""
        if cv2 is None:
            return []
        now = time.time(); self._reload_cfg(now)
        Pc, valid = self._cam_points(disp)                       # 카메라 프레임 organized
        Ps = Pc @ self._T_sc[:3, :3].T + self._T_sc[:3, 3]       # stand 프레임
        inroi = ((Ps[..., 0] > self._roi_x[0]) & (Ps[..., 0] < self._roi_x[1]) &
                 (Ps[..., 1] > self._roi_y[0]) & (Ps[..., 1] < self._roi_y[1]))
        if self._z_cap is not None:                  # perception 전용 stand-Z 점 상한
            inroi = inroi & (Ps[..., 2] < self._z_cap)

        have_color = color_img is not None and self._calib is not None
        if have_color:
            rgb, cok = self._organized_rgb(Pc, color_img, self._calib)
            mh = valid & cok & inroi
        else:
            mh = valid & inroi
        # --- head + 손목 결합 클라우드 (RoI/zcap 필터된 stand 점) ---
        P = Ps[mh]
        C = rgb[mh] if have_color else None
        if extra_xyz is not None and len(extra_xyz) > 0:
            ex = np.asarray(extra_xyz, dtype=np.float64)
            inb = ((ex[:, 0] > self._roi_x[0]) & (ex[:, 0] < self._roi_x[1]) &
                   (ex[:, 1] > self._roi_y[0]) & (ex[:, 1] < self._roi_y[1]))
            if self._z_cap is not None:
                inb = inb & (ex[:, 2] < self._z_cap)
            if inb.any():
                P = np.concatenate([P, ex[inb]], 0)
                if have_color:
                    C = np.concatenate([C, np.asarray(extra_rgb, dtype=np.uint8)[inb]], 0)
        if int(len(P)) < 2000:                       # 결합 기준 게이트(head 단독 아님)
            return []
        plane = self._fit_floor_plane(P)
        if plane is None:
            plane = self._last_plane                 # 테이블 미관측(가림) 시 직전 양호 평면 재사용
        if plane is None:
            return []
        self._last_plane = plane
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

        Cb = C[band]
        hsv = cv2.cvtColor(np.ascontiguousarray(Cb).reshape(-1, 1, 3), cv2.COLOR_RGB2HSV).reshape(-1, 3)
        # cv2.inRange는 3채널 이미지를 기대 → 평탄화된 (N,3)엔 못 씀(예외).
        # GREEN_LO/HI (3,) 에 대한 numpy 브로드캐스트 비교로 동일 결과.
        is_green = np.all((hsv >= self.GREEN_LO) & (hsv <= self.GREEN_HI), axis=1)

        out = []
        green_T = None
        if int(is_green.sum()) >= self.MIN_PTS:
            g = self._detect_one(Pb[is_green], cam_xy, plane, "green")
            if g:
                out.append(g); green_T = g["T"]
        keep = ~is_green
        if green_T is not None:
            # 두 박스가 붙어 있어 공간분리 불가 → 초록 박스의 known-dims 사각형(ICP pose)을
            # 제외해 회색 박스만 남긴다(원형 제외는 초록 내부 회색점이 남아 oversized).
            d = Pb[:, :2] - green_T[:2, 3]
            xl = d @ green_T[:2, 0]; yl = d @ green_T[:2, 1]
            mgn = 0.04
            keep &= ~((np.abs(xl) < BOX_DIMS[0] / 2 + mgn) & (np.abs(yl) < BOX_DIMS[1] / 2 + mgn))
        if int(keep.sum()) >= self.MIN_PTS:
            # 회색=colorless 조밀 박스. bridge=False(de-bridge)로 박스를 테이블/잡음과
            # 떼어 독립 성분으로 잡고, 초록 footprint 안에 중심이 놓인 후보(초록 뒷벽/
            # 모서리 잔여물)는 reject_box로 거부한 뒤 점 수 최대(n-first) 후보를 고른다.
            # prefer_fit은 초록 잔여물의 known-dims 점수가 높아 오선택되므로 쓰지 않는다.
            gr = self._detect_one(Pb[keep], cam_xy, plane, "gray",
                                  bridge=False, reject_box=green_T)
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
