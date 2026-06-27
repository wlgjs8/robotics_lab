#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import json
import pathlib
import unittest

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
    def test_gray_candidate_prefers_known_dims_over_larger_blob(self):
        det = _detector(use_icp=False)
        good = _grid_rect(center=(-0.24, -0.72), size=(0.34, 0.20), nx=34, ny=26)
        oversized = _grid_rect(center=(0.28, -0.72), size=(0.50, 0.35), nx=58, ny=42)
        pts = np.concatenate([good, oversized], axis=0)

        cand = det._select_candidate(pts, cam_xy=np.array([0.0, -1.4]), plane=(0.0, 0.0, 0.0),
                                     label="gray", prefer_fit=True)

        self.assertIsNotNone(cand)
        self.assertLess(abs(float(cand["T0"][0, 3]) - good[:, 0].mean()), 0.08)
        self.assertLess(abs(float(cand["T0"][1, 3]) - good[:, 1].mean()), 0.08)

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

    def test_icp_method_normalization_supports_dual_mode(self):
        self.assertEqual(box_detect.BoxDetector._normalize_icp_method("point-to-plane"), "point_to_plane")
        self.assertEqual(box_detect.BoxDetector._normalize_icp_method("p2p"), "point_to_point")
        self.assertEqual(box_detect.BoxDetector._normalize_icp_method("unknown"), "point_to_point")


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


if __name__ == "__main__":
    raise SystemExit(unittest.main())
