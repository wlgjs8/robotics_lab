#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import json
import pathlib
import threading
import time
import unittest
from collections import deque

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[1]


def _load_module(name: str, rel: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / rel)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


box_detect = _load_module("box_detect", "stereo_worker/box_detect.py")
worker = _load_module("stereo_worker_worker", "stereo_worker/worker.py")
state_listener = _load_module("stereo_worker_state_listener", "stereo_worker/state_listener.py")


def _grid_rect(center, size, nx, ny):
    x = np.linspace(center[0] - size[0] / 2, center[0] + size[0] / 2, nx)
    y = np.linspace(center[1] - size[1] / 2, center[1] + size[1] / 2, ny)
    xx, yy = np.meshgrid(x, y)
    zz = np.full_like(xx, 0.055)
    return np.stack([xx.ravel(), yy.ravel(), zz.ravel()], axis=1)


def _detector(use_icp=False):
    K = np.array([[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]])
    return box_detect.BoxDetector(K, baseline=0.05, use_icp=use_icp)


class BoxDetectCandidateTest(unittest.TestCase):
    @unittest.skipIf(box_detect.cv2 is None, "opencv is required for cluster tests")
    def test_gray_rejects_green_residue_and_picks_real_box(self):
        # 회색 선택: 초록 박스 잔여물(초록 footprint 안에 중심이 놓인 후보)은 reject_box로
        # 걸러내고, 초록 밖의 실제 박스를 고른다. (예전엔 prefer_fit이 known-dims 점수가
        # 높은 잔여물을 골라 회색이 초록 옆에 찍히던 버그.)
        det = _detector(use_icp=False)
        green_T = np.eye(4)
        green_T[:3, 3] = (-0.22, -0.70, 0.055)
        residue = _grid_rect(center=(-0.22, -0.72), size=(0.34, 0.20), nx=80, ny=48)
        real = _grid_rect(center=(0.22, -0.70), size=(0.38, 0.24), nx=86, ny=54)
        pts = np.concatenate([residue, real], axis=0)

        cand = det._select_candidate(pts, cam_xy=np.array([0.0, -1.4]),
                                     plane=(0.0, 0.0, 0.0), label="gray",
                                     bridge=False, reject_box=green_T)

        self.assertIsNotNone(cand)
        self.assertLess(abs(float(cand["T0"][0, 3]) - 0.22), 0.08)
        self.assertLess(abs(float(cand["T0"][1, 3]) - (-0.70)), 0.08)

    def test_center_in_box_footprint_gate(self):
        box_T = np.eye(4)
        box_T[:3, 3] = (0.10, -0.50, 0.05)
        inside = np.eye(4)
        inside[:3, 3] = (0.10, -0.56, 0.05)         # 같은 x, y는 footprint 안
        outside = np.eye(4)
        outside[:3, 3] = (0.60, -0.50, 0.05)        # x로 멀리 → 밖
        self.assertTrue(box_detect.BoxDetector._center_in_box(inside, box_T))
        self.assertFalse(box_detect.BoxDetector._center_in_box(outside, box_T))

    @unittest.skipIf(box_detect.cv2 is None, "opencv is required for cluster tests")
    def test_cluster_xy_debridge_separates_adjacent_blobs(self):
        # colorless 회색 박스가 잡음과 0.02m 틈으로 인접: bridge=True(MORPH_CLOSE)는 한
        # 성분으로 이어 oversize가 되지만, bridge=False는 틈을 살려 둘로 분리한다.
        det = _detector(use_icp=False)
        left = _grid_rect(center=(-0.12, -0.70), size=(0.22, 0.22), nx=60, ny=60)
        right = _grid_rect(center=(0.12, -0.70), size=(0.22, 0.22), nx=60, ny=60)
        xy = np.concatenate([left, right], axis=0)[:, :2]
        self.assertEqual(len(det._cluster_xy(xy, bridge=True)[1]), 1)
        self.assertEqual(len(det._cluster_xy(xy, bridge=False)[1]), 2)

    @unittest.skipIf(box_detect.cv2 is None, "opencv is required for cluster tests")
    def test_detect_one_without_open3d_returns_disabled_icp_metadata(self):
        det = _detector(use_icp=False)
        scene = _grid_rect(center=(0.15, -0.74), size=(0.34, 0.20), nx=34, ny=26)

        box = det._detect_one(scene, cam_xy=np.array([0.0, -1.4]), plane=(0.0, 0.0, 0.0),
                              label="gray", prefer_fit=True)

        self.assertIsNotNone(box)
        self.assertEqual(box["icp_method"], "disabled")
        self.assertEqual(box["source_n"], box["n"])
        self.assertEqual(box["icp_sample_n"], 0)
        self.assertEqual(box["fitness"], 1.0)
        self.assertIsNone(box["rmse"])

    def test_icp_method_normalization_supports_three_modes(self):
        norm = box_detect.BoxDetector._normalize_icp_method
        self.assertEqual(norm("point-to-plane"), "point_to_plane")
        self.assertEqual(norm("p2p"), "point_to_point")
        self.assertEqual(norm("yaw"), "yaw_se2")
        self.assertEqual(norm("se2"), "yaw_se2")
        # unknown/empty → 기본 yaw_se2 (worker.py 및 BoxDetector 기본값과 일치)
        self.assertEqual(norm("unknown"), "yaw_se2")
        self.assertEqual(norm(None), "yaw_se2")


class BoxTrackerTelemetryTest(unittest.TestCase):
    def test_tracker_preserves_raw_icp_telemetry_and_marks_coasting(self):
        tracker = box_detect.BoxTracker(min_hits=1)
        T = np.eye(4)
        T[:3, 3] = [0.1, -0.7, 0.055]
        det = {
            "T": T,
            "dims": box_detect.BOX_DIMS,
            "footprint": (0.34, 0.20),
            "n": 900,
            "source_n": 900,
            "icp_sample_n": 512,
            "fitness": 0.42,
            "rmse": 0.0123,
            "icp_method": "point_to_plane",
            "label": "gray",
        }

        live = tracker.update([det])[0]
        coast = tracker.update([])[0]

        self.assertEqual(live["fitness"], 0.42)
        self.assertEqual(live["rmse"], 0.0123)
        self.assertEqual(live["icp_method"], "point_to_plane")
        self.assertEqual(live["source_n"], 900)
        self.assertEqual(live["icp_sample_n"], 512)
        self.assertFalse(live["coasting"])
        self.assertTrue(coast["coasting"])
        self.assertEqual(coast["fitness"], 0.42)


class BoxPublishContractTest(unittest.TestCase):
    def test_publish_boxes_includes_additive_telemetry(self):
        class FakeSocket:
            def __init__(self):
                self.parts = None

            def send_multipart(self, parts):
                self.parts = parts

        pub = worker.CloudPublisher.__new__(worker.CloudPublisher)
        pub._json = json
        pub._sock = FakeSocket()
        T = np.eye(4)

        pub.publish_boxes(7, [{
            "T": T,
            "dims": box_detect.BOX_DIMS,
            "footprint": (0.34, 0.20),
            "n": 900,
            "source_n": 900,
            "icp_sample_n": 512,
            "fitness": 0.42,
            "rmse": 0.0123,
            "icp_method": "point_to_point",
            "track_id": 3,
            "coasting": False,
            "label": "gray",
        }])

        self.assertEqual(pub._sock.parts[0], b"stereo.boxes")
        payload = json.loads(pub._sock.parts[1].decode())
        box = payload["boxes"][0]
        self.assertEqual(payload["seq"], 7)
        self.assertEqual(box["fitness"], 0.42)
        self.assertEqual(box["rmse"], 0.0123)
        self.assertEqual(box["track_id"], 3)
        self.assertEqual(box["icp_method"], "point_to_point")
        self.assertEqual(box["source_n"], 900)
        self.assertEqual(box["icp_sample_n"], 512)
        self.assertFalse(box["coasting"])


class TcpPoseMotionGateTest(unittest.TestCase):
    """손목 융합 동기화: 팔 이동 중에는 융합을 보류(is_settled=False)해야 한다.
    소켓/스레드 없이(object.__new__) 모션 추정 로직만 검증."""

    def _listener(self):
        sl = object.__new__(state_listener.TcpPoseListener)
        sl._lock = threading.Lock()
        sl._hist = {}
        sl._stale_s = 0.3
        sl._hist_window_s = 0.2
        return sl

    def _push(self, sl, arm, positions, R=None, dt=0.05):
        now = time.monotonic()
        R = np.eye(3) if R is None else R
        h = sl._hist.setdefault(arm, deque(maxlen=32))
        n = len(positions)
        for i, p in enumerate(positions):           # 최신 샘플 t==now 가 되도록
            h.append((now - (n - 1 - i) * dt, np.asarray(p, float), R))

    def test_stationary_arm_is_settled(self):
        sl = self._listener()
        self._push(sl, "left", [(0.1, -0.5, 0.3)] * 5)
        mo = sl.motion("left")
        self.assertIsNotNone(mo)
        self.assertLess(mo[0], 1e-6)
        self.assertTrue(sl.is_settled("left"))

    def test_moving_arm_not_settled(self):
        sl = self._listener()
        # 0.05s 간격 1cm 이동 → 0.2 m/s, 기본 임계 0.03 m/s 초과
        self._push(sl, "left", [(0.1 + 0.01 * i, -0.5, 0.3) for i in range(5)])
        self.assertGreater(sl.motion("left")[0], 0.1)
        self.assertFalse(sl.is_settled("left"))

    def test_insufficient_history_is_conservative(self):
        sl = self._listener()
        self._push(sl, "left", [(0.1, -0.5, 0.3)])
        self.assertIsNone(sl.motion("left"))
        self.assertFalse(sl.is_settled("left"))     # 추정 불가 → 융합 보류


class TemporalIcpTest(unittest.TestCase):
    """tracking-by-registration: _detect_one이 label별 직전 포즈를 저장하고 ICP init으로
    재사용(정지 박스 jitter 제거). yaw_se2 ICP는 scipy만 쓰므로 o3d 없이도 검증 가능."""

    @unittest.skipIf(box_detect.cv2 is None, "opencv required")
    def test_prev_pose_stored_and_reused(self):
        try:
            import scipy.spatial  # noqa: F401
        except Exception:
            self.skipTest("scipy required for yaw_se2 ICP")
        det = _detector(use_icp=False)
        det.use_icp = True                 # o3d 게이트 우회(yaw_se2=scipy)
        det.icp_method = "yaw_se2"
        det._temporal_icp = True
        scene = _grid_rect(center=(0.10, -0.60), size=(0.38, 0.24), nx=80, ny=50)
        cam = np.array([0.0, -1.4])
        b1 = det._detect_one(scene, cam, (0.0, 0.0, 0.0), label="green", bridge=True)
        self.assertIsNotNone(b1)
        self.assertIn("green", det._prev_pose)            # 트랙 저장
        b2 = det._detect_one(scene, cam, (0.0, 0.0, 0.0), label="green", bridge=True)
        self.assertIsNotNone(b2)
        # 같은 관측이면 2번째 출력이 저장된 prev와 일치
        np.testing.assert_allclose(det._prev_pose["green"][:3, 3], b2["T"][:3, 3], atol=1e-6)

    @unittest.skipIf(box_detect.cv2 is None, "opencv required")
    def test_temporal_disabled_keeps_no_track(self):
        det = _detector(use_icp=False)
        det.use_icp = True
        det.icp_method = "yaw_se2"
        det._temporal_icp = False          # off → 트랙 저장 안 함
        scene = _grid_rect(center=(0.10, -0.60), size=(0.38, 0.24), nx=80, ny=50)
        try:
            det._detect_one(scene, np.array([0.0, -1.4]), (0.0, 0.0, 0.0), label="green", bridge=True)
        except Exception:
            self.skipTest("scipy required")
        self.assertEqual(det._prev_pose, {})


class CombinedCloudOcclusionTest(unittest.TestCase):
    """결합 클라우드: head disp가 비어도(완전 가림) 손목 점(extra_xyz)만으로 검출이 이어진다."""

    @unittest.skipIf(box_detect.cv2 is None, "opencv required")
    def test_detects_from_wrist_when_head_empty(self):
        det = _detector(use_icp=False)
        disp = np.zeros((60, 80), dtype=np.float32)        # 전부 무효 → head 점 0 (가림 모사)
        rng = np.random.default_rng(0)
        table = np.column_stack([rng.uniform(-0.3, 0.3, 4000),
                                 rng.uniform(-0.9, -0.5, 4000),
                                 np.zeros(4000)])          # 테이블(z=0, 평면 fit용)
        box = _grid_rect(center=(0.10, -0.70), size=(0.38, 0.24), nx=80, ny=50)  # z=0.055 (band)
        extra = np.concatenate([table, box], 0)
        extra_rgb = np.zeros((len(extra), 3), np.uint8)
        out = det.detect(disp, color_img=None, extra_xyz=extra, extra_rgb=extra_rgb)
        self.assertGreaterEqual(len(out), 1)               # head 비어도 손목으로 검출됨
        c = out[0]["T"][:3, 3]
        self.assertLess(abs(float(c[0]) - 0.10), 0.08)
        self.assertLess(abs(float(c[1]) - (-0.70)), 0.08)

    @unittest.skipIf(box_detect.cv2 is None, "opencv required")
    def test_empty_combined_returns_empty(self):
        det = _detector(use_icp=False)
        disp = np.zeros((60, 80), dtype=np.float32)
        self.assertEqual(det.detect(disp, color_img=None, extra_xyz=None, extra_rgb=None), [])


class OpenTrayModelTest(unittest.TestCase):
    """ICP 모델이 쉘(내벽+외벽+열린 윗면)인지 — 오픈 트레이를 비스듬히 볼 때 가까운-바깥/
    먼-안쪽 관측이 각자 올바른 면에 대응돼 벽두께 편향이 안 생기게."""

    def test_model_has_both_inner_and_outer_walls(self):
        m = box_detect._open_tray_model()
        hx = box_detect.BOX_DIMS[0] / 2
        ix = hx - 0.020
        n_outer = int((np.abs(np.abs(m[:, 0]) - hx) < 0.004).sum())
        n_inner = int((np.abs(np.abs(m[:, 0]) - ix) < 0.004).sum())
        self.assertGreater(n_outer, 50)
        self.assertGreater(n_inner, 50)        # 내벽 존재(예전 모델엔 없었음)

    def test_model_top_is_open_rim_not_solid(self):
        m = box_detect._open_tray_model()
        hz = box_detect.BOX_DIMS[2] / 2
        ix = box_detect.BOX_DIMS[0] / 2 - 0.020
        iy = box_detect.BOX_DIMS[1] / 2 - 0.020
        top = m[np.abs(m[:, 2] - hz) < 0.004]
        self.assertGreater(len(top), 50)
        # 윗면 안쪽(개구부)엔 점이 거의 없어야 함(림 링만, 솔리드 top 아님)
        opening = top[(np.abs(top[:, 0]) < ix - 0.01) & (np.abs(top[:, 1]) < iy - 0.01)]
        self.assertLess(len(opening), len(top) * 0.2)


class CloudRoiClipTest(unittest.TestCase):
    """publish 전 head/손목 클라우드를 Safety ROI(stand)로 자르는 worker.roi_clip_mask."""

    def test_roi_clip_mask_keeps_only_in_roi(self):
        pts = np.array([[0.5, 0.5, 0.5],    # in
                        [1.5, 0.5, 0.5],    # out x
                        [0.5, -0.1, 0.5],   # out y
                        [0.5, 0.5, 1.2]],   # out z
                       dtype=float)
        roi = ((0.0, 1.0), (0.0, 1.0), (0.0, 1.0))
        m = worker.roi_clip_mask(pts, np.eye(4), roi)
        self.assertEqual(m.tolist(), [True, False, False, False])

    def test_roi_clip_mask_applies_stand_transform(self):
        T = np.eye(4); T[0, 3] = 1.0          # cam→stand: +1 in x
        pts = np.array([[-0.5, 0.5, 0.5],     # stand x=0.5 → in
                        [-1.5, 0.5, 0.5]],    # stand x=-0.5 → out
                       dtype=float)
        roi = ((0.0, 1.0), (0.0, 1.0), (0.0, 1.0))
        self.assertEqual(worker.roi_clip_mask(pts, T, roi).tolist(), [True, False])


class BoxDetectRoiSettingsTest(unittest.TestCase):
    """box_detect가 rb_gui Safety ROI(settings.json roi_min_m/roi_max_m)를 읽는다."""

    def _write(self, obj):
        import tempfile
        f = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False)
        json.dump(obj, f); f.close()
        return f.name

    def test_load_roi_parses_min_max(self):
        import os
        p = self._write({"roi_min_m": [-0.5, -1.0, 0.0], "roi_max_m": [0.5, 0.0, 1.0]})
        try:
            self.assertEqual(box_detect._load_roi(p, "DEF"),
                             ((-0.5, 0.5), (-1.0, 0.0), (0.0, 1.0)))
        finally:
            os.unlink(p)

    def test_load_roi_falls_back_when_missing(self):
        self.assertEqual(box_detect._load_roi("/nonexistent/x.json", "DEF"), "DEF")

    def test_load_roi_rejects_inverted_bounds(self):
        import os
        p = self._write({"roi_min_m": [0.5, -1.0, 0.0], "roi_max_m": [-0.5, 0.0, 1.0]})
        try:
            self.assertEqual(box_detect._load_roi(p, "DEF"), "DEF")
        finally:
            os.unlink(p)

    def test_load_z_cap_from_settings(self):
        import os
        p = self._write({"pc_clip_z_max_m": 0.15})
        try:
            self.assertAlmostEqual(box_detect._load_z_cap(p), 0.15)
        finally:
            os.unlink(p)

    def test_load_z_cap_absent_is_none(self):
        p = self._write({"pc_dmin": 0.1})       # no pc_clip_z_max_m, no env
        import os
        try:
            self.assertIsNone(box_detect._load_z_cap(p))
        finally:
            os.unlink(p)

    def test_effective_clip_roi_folds_z_cap(self):
        det = _detector(use_icp=False)
        det._roi_x, det._roi_y, det._roi_z = (-0.5, 0.5), (-1.0, 0.0), (0.0, 0.8)
        det._z_cap = 0.15
        self.assertEqual(worker.effective_clip_roi(det),
                         ((-0.5, 0.5), (-1.0, 0.0), (0.0, 0.15)))
        det._z_cap = None                         # off → ROI Z unchanged
        self.assertEqual(worker.effective_clip_roi(det)[2], (0.0, 0.8))


if __name__ == "__main__":
    raise SystemExit(unittest.main())
