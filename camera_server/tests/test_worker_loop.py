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


worker = _load_module("stereo_worker_worker_loop", "stereo_worker/worker.py")


class WristCloudTest(unittest.TestCase):
    """D405 z16 depth -> 카메라 프레임 3D점. 모델 없는 순수 deprojection."""

    def _frames(self, depth_raw):
        depth = np.full((8, 8), depth_raw, np.uint16)
        color = np.zeros((8, 8, 3), np.uint8)
        color[..., 0] = 200
        return depth, color

    def test_deprojects_with_d405_intrinsics_and_depth_scale(self):
        # 0.30 m -> z16 raw = 0.30 / 1e-4 = 3000.
        depth, color = self._frames(3000)

        xyz, rgb = worker.wrist_cloud(depth, color, stride=1)

        self.assertEqual(xyz.shape[0], 64)
        self.assertEqual(rgb.shape, (64, 3))
        np.testing.assert_allclose(xyz[:, 2], 0.30, atol=1e-6)
        # 픽셀 (0,0): X = (0 - cx)/fx * z
        np.testing.assert_allclose(
            xyz[0, 0], (0.0 - worker.WRIST_CX) / worker.WRIST_FX * 0.30, atol=1e-6)
        np.testing.assert_allclose(
            xyz[0, 1], (0.0 - worker.WRIST_CY) / worker.WRIST_FY * 0.30, atol=1e-6)
        np.testing.assert_array_equal(rgb[0], (200, 0, 0))

    def test_drops_points_outside_the_z_window(self):
        # 0.02 m (zmin=0.05 미만) 과 2.0 m (zmax=1.2 초과) 는 모두 버려진다.
        for raw in (200, 20000):
            with self.subTest(raw=raw):
                depth, color = self._frames(raw)
                xyz, rgb = worker.wrist_cloud(depth, color, stride=1)
                self.assertEqual(xyz.shape[0], 0)
                self.assertEqual(rgb.shape[0], 0)

    def test_zero_depth_is_dropped(self):
        """D405가 매칭 실패한 픽셀은 0을 낸다 — 원점 점으로 새어나가면 안 된다."""
        depth, color = self._frames(0)

        xyz, _ = worker.wrist_cloud(depth, color, stride=1)

        self.assertEqual(xyz.shape[0], 0)

    def test_stride_subsamples_the_pixel_grid(self):
        depth, color = self._frames(3000)

        xyz, _ = worker.wrist_cloud(depth, color, stride=2)

        self.assertEqual(xyz.shape[0], 16)   # 8x8 -> 4x4


class PublishWristTest(unittest.TestCase):
    class FakeSocket:
        def __init__(self):
            self.parts = None

        def send_multipart(self, parts):
            self.parts = parts

    def _publisher(self, viz_max_pts=0):
        pub = worker.CloudPublisher.__new__(worker.CloudPublisher)
        pub._json = json
        pub._sock = self.FakeSocket()
        pub._viz_max_pts = viz_max_pts
        return pub

    def test_publishes_stereo_wrist_topic_with_arm_and_count_header(self):
        pub = self._publisher()
        xyz = np.arange(9, dtype=np.float32).reshape(3, 3)
        rgb = np.full((3, 3), 7, np.uint8)

        pub.publish_wrist("left", 11, xyz, rgb)

        topic, header, xyz_buf, rgb_buf = pub._sock.parts
        self.assertEqual(topic, b"stereo.wrist")
        self.assertEqual(json.loads(header.decode()), {"arm": "left", "seq": 11, "n": 3})
        np.testing.assert_array_equal(
            np.frombuffer(xyz_buf, np.float32).reshape(-1, 3), xyz)
        np.testing.assert_array_equal(
            np.frombuffer(rgb_buf, np.uint8).reshape(-1, 3), rgb)

    def test_viz_cap_subsamples_and_reports_the_capped_count(self):
        pub = self._publisher(viz_max_pts=2)
        xyz = np.arange(15, dtype=np.float32).reshape(5, 3)
        rgb = np.zeros((5, 3), np.uint8)

        pub.publish_wrist("right", 4, xyz, rgb)

        header = json.loads(pub._sock.parts[1].decode())
        self.assertEqual(header["n"], 2)
        self.assertEqual(np.frombuffer(pub._sock.parts[2], np.float32).size, 6)

    def test_cap_is_a_noop_when_under_the_limit(self):
        pub = self._publisher(viz_max_pts=100)
        xyz = np.zeros((5, 3), np.float32)

        capped_xyz, capped_rgb = pub._cap(0, xyz, np.zeros((5, 3), np.uint8))

        self.assertIs(capped_xyz, xyz)
        self.assertEqual(capped_rgb.shape[0], 5)


if __name__ == "__main__":
    unittest.main()
