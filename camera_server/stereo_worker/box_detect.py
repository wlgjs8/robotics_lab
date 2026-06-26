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
    FIT_MIN = 0.45            # ICP fitness 게이트
    PLANE_REFRESH = 5.0

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

    def _table_plane(self, P, valid, now):
        """카메라 프레임 테이블 평면 (a,b,c,d), normal을 카메라쪽(up, n_z<0)으로."""
        if self._plane is not None and (now - self._plane_t) < self.PLANE_REFRESH:
            return self._plane
        pts = P[valid]
        if len(pts) < 2000 or o3d is None:
            return self._plane
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts[::3].astype(np.float64)))
        model, _ = pc.segment_plane(0.008, 3, 250)
        n = np.array(model[:3]); nn = np.linalg.norm(n)
        if nn < 1e-6:
            return self._plane
        model = np.array(model) / nn
        if model[2] > 0:            # normal이 카메라쪽(-z, up)을 향하도록
            model = -model
        self._plane, self._plane_t = model, now
        return model

    def detect(self, disp):
        if cv2 is None or o3d is None:
            return []
        now = time.time(); self._reload_cfg(now)
        P, valid = self._cam_points(disp)
        plane = self._table_plane(P, valid, now)
        if plane is None:
            return []
        n, d = plane[:3], plane[3]
        h = P @ n + d                       # 테이블 위 높이 (up = n)
        obj = (valid & (h > self.H_LO) & (h < self.H_HI)).astype(np.uint8)
        ncc, lab, stats, _ = cv2.connectedComponentsWithStats(obj, 8)
        # 평면 접선 기저
        ref = np.array([1.0, 0, 0]) if abs(n[0]) < 0.9 else np.array([0, 1.0, 0])
        t1 = np.cross(n, ref); t1 /= np.linalg.norm(t1); t2 = np.cross(n, t1)
        out = []
        for cid in range(1, ncc):
            if stats[cid, cv2.CC_STAT_AREA] < self.MIN_AREA_PX:
                continue
            pts = P[lab == cid]; pts = pts[np.isfinite(pts).all(1)]
            if len(pts) < self.MIN_AREA_PX:
                continue
            r = self._fit(pts, n, d, t1, t2)
            if r is not None:
                T_stand = self._T_sc @ r["T_cam"]
                out.append(dict(T=T_stand, dims=BOX_DIMS, footprint=r["footprint"],
                                n=int(len(pts)), fitness=round(r["fitness"], 3)))
        return out

    def _fit(self, pts, n, d, t1, t2):
        # 평면 접선 좌표 (u,v), 높이 h
        u = pts @ t1; v = pts @ t2; h = pts @ n + d
        # PCA로 평면상 주축(yaw) — 옆면 점들의 수평 분포 기준
        uv = np.stack([u, v], 1)
        uv0 = uv - uv.mean(0)
        C = uv0.T @ uv0
        evals, evecs = np.linalg.eigh(C)
        major = evecs[:, np.argmax(evals)]              # 평면상 주축
        x_ax = major[0] * t1 + major[1] * t2
        x_ax /= np.linalg.norm(x_ax)
        y_ax = np.cross(n, x_ax)
        # 평면상 사각형 크기 (느슨한 sanity)
        ext_major = uv0 @ major; ext_minor = uv0 @ evecs[:, np.argmin(evals)]
        dim_major = ext_major.max() - ext_major.min()
        if dim_major > self.MAX_DIM:
            return None
        # 중심 init: 평면상 점 중심을 평면에 투영 + height/2 만큼 up
        p_plane = uv.mean(0)[0]*t1 + uv.mean(0)[1]*t2 - d*n   # 평면 위 점
        center = p_plane + (BOX_DIMS[2]/2.0) * n
        R0 = np.column_stack([x_ax, y_ax, n])           # box x,y,z(up) -> cam
        T0 = np.eye(4); T0[:3, :3] = R0; T0[:3, 3] = center
        fitness = 1.0
        T_cam = T0
        if self.use_icp:
            obs = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts.astype(np.float64)))
            reg = o3d.pipelines.registration
            try:
                res = reg.registration_icp(
                    obs, self.model_pcd, 0.03, np.linalg.inv(T0),
                    reg.TransformationEstimationPointToPoint(),
                    reg.ICPConvergenceCriteria(max_iteration=20))
                T_cam = np.linalg.inv(res.transformation)
                fitness = res.fitness
            except Exception:
                pass
        if fitness < self.FIT_MIN:
            return None
        return dict(T_cam=T_cam, footprint=(round(dim_major, 3), round(ext_minor.max()-ext_minor.min(), 3)),
                    fitness=fitness)
