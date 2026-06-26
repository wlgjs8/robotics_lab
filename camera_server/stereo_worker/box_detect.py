"""박스 검출 (워커, **카메라 프레임**에서 검출 → 결과만 T_stand_cam으로 변환).

장면: 카메라 ~45° 비스듬 → 박스 옆면이 주로 보이고 윗면은 앞모서리만, 바닥 거의 안 보임.
따라서 "윗면 footprint" 대신 **보이는 면(옆면+부분 윗면)에 알려진 open-tray 모델을 ICP 정합**.
카메라 프레임에서 동작 → rough T_stand_cam 캘리브와 무관(표시 위치만 T_stand_cam 사용).

레시피: depth 필터 → 카메라 프레임 점 → 테이블 평면 RANSAC(캐시) → 평면 위 점 2D-CC 분할
→ 클러스터별 모델 ICP(평면normal=up, PCA yaw, 중심 init) → fitness 게이트 → T_cam → T_stand.
"""
from __future__ import annotations
import json
import os
import time
import numpy as np

try:
    import cv2
    import open3d as o3d
except Exception:  # noqa: BLE001
    cv2 = None
    o3d = None

BOX_DIMS = (0.380, 0.240, 0.110)
T_STAND_CAM_PATH = os.environ.get("STEREO_T_STAND_CAM", "/app/state/T_stand_cam.npy")
GUI_SETTINGS_PATH = os.environ.get("STEREO_GUI_SETTINGS", "/app/rb_gui_state/settings.json")


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
    DEPTH_PAD = 0.05          # depth max 위로 약간 여유(박스 뒷부분)
    H_LO, H_HI = 0.02, 0.25   # 테이블 위 높이 밴드 (m)
    MIN_AREA_PX = 800
    MAX_DIM = 0.55            # 클러스터 평면상 최대 변(이보다 크면 박스 아님)
    FIT_MIN = 0.50            # ICP fitness 게이트
    PLANE_REFRESH = 5.0
    # stand 프레임 RoI (rb_gui Safety ROI 기본값) — 박스는 이 안에만 있음
    ROI_X = (-0.55, 0.55); ROI_Y = (-1.05, 0.05); ROI_Z = (-0.05, 1.0)

    def __init__(self, K, baseline, use_icp=True):
        self.fx, self.fy = K[0, 0], K[1, 1]
        self.cx, self.cy = K[0, 2], K[1, 2]
        self.baseline = baseline
        self.use_icp = use_icp and (o3d is not None)
        self.model = _open_tray_model()
        self.model_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(self.model)) if o3d else None
        self._uu = self._vv = None
        self._plane = None; self._plane_t = 0.0
        self._T_sc = _load_T(T_STAND_CAM_PATH)
        self._dmin, self._dmax = _load_depth_range(GUI_SETTINGS_PATH)
        self._cfg_t = 0.0

    def _reload_cfg(self, now):
        if now - self._cfg_t > 1.0:
            self._T_sc = _load_T(T_STAND_CAM_PATH)
            self._dmin, self._dmax = _load_depth_range(GUI_SETTINGS_PATH)
            self._cfg_t = now

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

    PLANE_THR = 0.012
    N_PLANES = 2                 # 테이블 2개(가까운/먼, 비수평) 제거

    def _fit_planes(self, P, valid, now):
        """장면의 큰 평면 N개(테이블들)를 RANSAC 반복 제거. 캐시. normal은 up(n_z<0)."""
        if self._plane is not None and (now - self._plane_t) < self.PLANE_REFRESH:
            return self._plane
        pts = P[valid]
        if len(pts) < 3000 or o3d is None:
            return self._plane
        rem = pts[::3].astype(np.float64)
        planes = []
        for _ in range(self.N_PLANES):
            if len(rem) < 2000:
                break
            pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(rem))
            model, inl = pc.segment_plane(self.PLANE_THR, 3, 250)
            n = np.array(model[:3]); nn = np.linalg.norm(n)
            if nn < 1e-6:
                break
            model = np.array(model) / nn
            if model[2] > 0:        # up = 카메라쪽(-z)
                model = -model
            planes.append(model)
            dist = np.abs(rem @ model[:3] + model[3])
            rem = rem[dist > self.PLANE_THR]   # 이 평면 제거 후 다음 평면
        if planes:
            self._plane, self._plane_t = planes, now
        return self._plane

    def detect(self, disp):
        if cv2 is None or o3d is None:
            return []
        now = time.time(); self._reload_cfg(now)
        P, valid = self._cam_points(disp)
        planes = self._fit_planes(P, valid, now)
        if not planes:
            return []
        # 평면별 높이 (organized). on_plane = 어느 평면에든 가까움. 박스 = 평면 위 밴드.
        Hs = np.stack([P @ pl[:3] + pl[3] for pl in planes], 0)   # (K,H,W)
        on_plane = (np.abs(Hs).min(0) < self.PLANE_THR)
        above_band = ((Hs > self.H_LO) & (Hs < self.H_HI)).any(0)
        obj = (valid & ~on_plane & above_band).astype(np.uint8)
        ncc, lab, stats, _ = cv2.connectedComponentsWithStats(obj, 8)
        out = []
        for cid in range(1, ncc):
            if stats[cid, cv2.CC_STAT_AREA] < self.MIN_AREA_PX:
                continue
            sel = (lab == cid)
            pts = P[sel]; pts = pts[np.isfinite(pts).all(1)]
            if len(pts) < self.MIN_AREA_PX:
                continue
            # 이 클러스터를 받치는 평면 = [H_LO,H_HI] 밴드에 점이 가장 많이 드는 평면
            hp = [pts @ p[:3] + p[3] for p in planes]
            score = [int(((hh > self.H_LO) & (hh < self.H_HI)).sum()) for hh in hp]
            pl = planes[int(np.argmax(score))]
            n, d = pl[:3], pl[3]
            ref = np.array([1.0, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1.0, 0])
            t1 = np.cross(n, ref); t1 /= np.linalg.norm(t1); t2 = np.cross(n, t1)
            r = self._fit(pts, n, d, t1, t2)
            if r is not None:
                T_stand = self._T_sc @ r["T_cam"]
                c = T_stand[:3, 3]
                if not (self.ROI_X[0] < c[0] < self.ROI_X[1] and
                        self.ROI_Y[0] < c[1] < self.ROI_Y[1] and
                        self.ROI_Z[0] < c[2] < self.ROI_Z[1]):
                    continue                       # RoI 밖 -> 박스 아님
                out.append(dict(T=T_stand, dims=BOX_DIMS, footprint=r["footprint"],
                                n=int(len(pts)), fitness=round(r["fitness"], 3)))
        return out

    def _fit(self, pts, n, d, t1, t2):
        """평면 구속 pose. up=평면normal, 바닥은 평면에. yaw=평면상 minAreaRect.
        부분 시점(옆면 위주)이라 중심은 '안 보이는(가려진) 쪽'으로 알려진 치수만큼 밀어 보정."""
        u = pts @ t1; v = pts @ t2
        uv = np.stack([u, v], 1).astype(np.float32)
        (cu, cv), (w, hgt), ang = cv2.minAreaRect((uv * 1000).reshape(-1, 1, 2))
        cu, cv, w, hgt = cu/1000, cv/1000, w/1000, hgt/1000
        if w >= hgt:
            obs_long, obs_short, yaw = w, hgt, np.radians(ang)
        else:
            obs_long, obs_short, yaw = hgt, w, np.radians(ang + 90.0)
        # 크기 게이트: 보이는 긴 변 ≈ 0.38
        if not (0.22 <= obs_long <= 0.52) or obs_short > 0.36:
            return None
        x_ax = np.cos(yaw) * t1 + np.sin(yaw) * t2       # 박스 긴 변(0.38) 방향
        x_ax /= np.linalg.norm(x_ax)
        y_ax = np.cross(n, x_ax)                          # 짧은 변(0.24)
        # 관측 rect 중심(평면상)
        center = cu * t1 + cv * t2 - d * n
        # 가림 보정: 각 축에서 관측 길이가 박스 치수보다 짧으면, 카메라 반대(가려진) 쪽으로 중심 이동
        for ax, full in ((x_ax, BOX_DIMS[0]), (y_ax, BOX_DIMS[1])):
            obs = (uv @ np.array([ax @ t1, ax @ t2]))
            obs_ext = obs.max() - obs.min()
            push = max(0.0, full/2.0 - obs_ext/2.0)
            if push > 1e-4:
                away = ax if (ax @ center) > 0 else -ax   # 카메라(원점)에서 먼 쪽
                center = center + away * push
        center = center + (BOX_DIMS[2]/2.0) * n           # 바닥을 평면에 안착
        R0 = np.column_stack([x_ax, y_ax, n])             # box x,y,z(up) -> cam
        T_cam = np.eye(4); T_cam[:3, :3] = R0; T_cam[:3, 3] = center
        return dict(T_cam=T_cam, footprint=(round(obs_long, 3), round(obs_short, 3)),
                    fitness=min(obs_long / BOX_DIMS[0], 1.0))


class BoxTracker:
    """검출 깜빡임 안정화: 지속 트랙(최대 N개) + 근접 연관 + EMA 스무딩 + coasting.

    update(dets) -> 안정화된 박스 리스트(confirmed). 한 프레임 미검출이어도 트랙은
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
                tr["foot"] = d["footprint"]; tr["n"] = d["n"]
            else:
                tr["miss"] += 1
        for i, (c, x, up, d) in enumerate(obs):       # 새 트랙
            if i in used or len(self.tracks) >= self.max_tracks:
                continue
            self._id += 1
            self.tracks.append(dict(center=c, x_ax=x, up=up, hits=1, miss=0,
                                    foot=d["footprint"], n=d["n"], id=self._id))
        self.tracks = [t for t in self.tracks if t["miss"] <= self.max_miss]
        out = []
        for t in self.tracks:
            if t["hits"] >= self.min_hits:
                up = t["up"]; x = t["x_ax"]
                x = x - up*(x @ up); x /= np.linalg.norm(x)   # up에 직교화
                y = np.cross(up, x)
                T = np.eye(4); T[:3, :3] = np.column_stack([x, y, up]); T[:3, 3] = t["center"]
                out.append(dict(T=T, dims=BOX_DIMS, footprint=t["foot"], n=t["n"],
                                fitness=1.0, track_id=t["id"]))
        return out
