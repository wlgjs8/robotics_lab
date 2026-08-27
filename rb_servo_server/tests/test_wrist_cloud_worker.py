"""wrist_cloud_worker: the D405 depth -> camera-frame cloud math and the
publish contract rb_gui subscribes to.

The worker itself needs a live camera_server, but the two things that can
silently break for a viewer -- the projection and the multipart wire format --
are pure and testable here. Restored 2026-08-27 after commit 3287da8 deleted
stereo_worker wholesale along with Fast-FoundationStereo; the wrist path never
depended on FFS.
"""
import json
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(
    0,
    str(Path(__file__).resolve().parents[2] / "camera_server" / "stereo_worker"),
)

import wrist_cloud_worker as wcw


class WristCloudMathTest(unittest.TestCase):
    def test_projection_matches_the_pinhole_model(self):
        """A depth pixel must land at the ray its intrinsics define."""
        depth = np.zeros((480, 640), dtype=np.uint16)
        # 0.50 m at the principal point and at a known off-axis pixel.
        z_m = 0.50
        raw = int(round(z_m / wcw.WRIST_DSCALE))
        cx, cy = int(round(wcw.WRIST_CX)), int(round(wcw.WRIST_CY))
        depth[cy, cx] = raw
        depth[cy, cx + 100] = raw
        color = np.full((480, 640, 3), 200, dtype=np.uint8)

        xyz, rgb = wcw.wrist_cloud(depth, color, stride=1)
        self.assertEqual(xyz.shape[0], 2)
        self.assertEqual(rgb.shape, (2, 3))

        # Nearest-to-principal-point pixel: essentially along +z. It is not
        # EXACTLY zero because the principal point is fractional (cx 319.0753)
        # and a pixel index is not, so compare against the model's own answer.
        centre = xyz[np.argmin(np.abs(xyz[:, 0]))]
        expected_centre_x = (cx - wcw.WRIST_CX) / wcw.WRIST_FX * z_m
        self.assertAlmostEqual(float(centre[0]), expected_centre_x, places=6)
        self.assertLess(abs(float(centre[0])), 1e-3)   # sub-mm: on the axis
        self.assertAlmostEqual(float(centre[2]), z_m, places=4)
        # Off-axis pixel: x = (u - cx)/fx * z, sign follows optical +x = right.
        off = xyz[np.argmax(np.abs(xyz[:, 0]))]
        expected_x = ((cx + 100) - wcw.WRIST_CX) / wcw.WRIST_FX * z_m
        self.assertAlmostEqual(float(off[0]), expected_x, places=4)
        self.assertAlmostEqual(float(off[2]), z_m, places=4)

    def test_z_range_gate_drops_out_of_band_depth(self):
        """zmin/zmax exist so the near-field noise and the far wall do not
        swamp the viewer; a 0 (no return) pixel must never become a point."""
        depth = np.zeros((480, 640), dtype=np.uint16)
        depth[100, 100] = int(round(0.02 / wcw.WRIST_DSCALE))   # below zmin
        depth[100, 102] = int(round(0.30 / wcw.WRIST_DSCALE))   # in band
        depth[100, 104] = int(round(3.00 / wcw.WRIST_DSCALE))   # above zmax
        # depth[200, 200] stays 0 -> no return
        color = np.zeros((480, 640, 3), dtype=np.uint8)
        xyz, _rgb = wcw.wrist_cloud(depth, color, zmin=0.05, zmax=1.2, stride=1)
        self.assertEqual(xyz.shape[0], 1)
        self.assertAlmostEqual(float(xyz[0, 2]), 0.30, places=4)

    def test_stride_subsamples_without_shifting_geometry(self):
        """Striding is a viewer cost control, not a calibration change: the
        points it keeps must sit exactly where the full-resolution cloud put
        them."""
        rng = np.random.default_rng(0)
        depth = (rng.integers(2000, 8000, size=(480, 640))).astype(np.uint16)
        color = rng.integers(0, 255, size=(480, 640, 3), dtype=np.uint8)
        full, _ = wcw.wrist_cloud(depth, color, stride=1)
        strided, _ = wcw.wrist_cloud(depth, color, stride=4)
        self.assertLess(strided.shape[0], full.shape[0])
        # Every strided point exists in the full cloud (same ray, same z).
        sample = strided[:: max(1, strided.shape[0] // 50)]
        for p in sample:
            d = np.abs(full - p).max(axis=1)
            self.assertLess(float(d.min()), 1e-6)

    def test_rgb_is_paired_with_the_kept_pixels(self):
        depth = np.zeros((480, 640), dtype=np.uint16)
        color = np.zeros((480, 640, 3), dtype=np.uint8)
        raw = int(round(0.40 / wcw.WRIST_DSCALE))
        depth[10, 10] = raw
        color[10, 10] = (7, 8, 9)
        depth[20, 20] = raw
        color[20, 20] = (70, 80, 90)
        xyz, rgb = wcw.wrist_cloud(depth, color, stride=1)
        self.assertEqual(xyz.shape[0], 2)
        self.assertEqual({tuple(c) for c in rgb}, {(7, 8, 9), (70, 80, 90)})


class _FakeSocket:
    def __init__(self):
        self.sent = []
        self.opts = {}

    def setsockopt(self, opt, val):
        self.opts[opt] = val

    def bind(self, addr):
        self.bound = addr

    def send_multipart(self, parts):
        self.sent.append(parts)

    def close(self, linger=0):
        pass


class PublishContractTest(unittest.TestCase):
    """rb_gui/pointcloud_receiver.py unpacks exactly 4 parts and reshapes the
    payloads by the header's n. A change here silently blanks the viewer."""

    def _publisher(self, viz_max_pts=0):
        pub = wcw.WristCloudPublisher.__new__(wcw.WristCloudPublisher)
        pub._sock = _FakeSocket()
        pub._viz_max_pts = int(viz_max_pts)
        return pub

    def test_multipart_shape_and_header(self):
        pub = self._publisher()
        xyz = np.arange(9, dtype=np.float32).reshape(3, 3)
        rgb = np.arange(9, dtype=np.uint8).reshape(3, 3)
        pub.publish_wrist("left", 42, xyz, rgb)

        (topic, header, xyz_b, rgb_b), = pub._sock.sent
        self.assertEqual(topic, b"stereo.wrist")
        meta = json.loads(header)
        self.assertEqual(meta["arm"], "left")
        self.assertEqual(meta["seq"], 42)
        self.assertEqual(meta["n"], 3)
        # Payloads must round-trip at the receiver's dtypes.
        self.assertTrue(np.array_equal(
            np.frombuffer(xyz_b, np.float32).reshape(-1, 3), xyz))
        self.assertTrue(np.array_equal(
            np.frombuffer(rgb_b, np.uint8).reshape(-1, 3), rgb))

    def test_cap_subsamples_and_keeps_header_consistent(self):
        """The cap exists because 59k points/arm at 10 Hz saturates the viser
        websocket. n must describe what was actually sent, not the input."""
        pub = self._publisher(viz_max_pts=100)
        xyz = np.zeros((5000, 3), dtype=np.float32)
        rgb = np.zeros((5000, 3), dtype=np.uint8)
        pub.publish_wrist("right", 1, xyz, rgb)

        (_topic, header, xyz_b, rgb_b), = pub._sock.sent
        meta = json.loads(header)
        self.assertEqual(meta["n"], 100)
        self.assertEqual(np.frombuffer(xyz_b, np.float32).reshape(-1, 3).shape[0], 100)
        self.assertEqual(np.frombuffer(rgb_b, np.uint8).reshape(-1, 3).shape[0], 100)

    def test_cap_off_passes_everything(self):
        pub = self._publisher(viz_max_pts=0)
        xyz = np.zeros((1234, 3), dtype=np.float32)
        rgb = np.zeros((1234, 3), dtype=np.uint8)
        pub.publish_wrist("left", 0, xyz, rgb)
        meta = json.loads(pub._sock.sent[0][1])
        self.assertEqual(meta["n"], 1234)


if __name__ == "__main__":
    unittest.main()
