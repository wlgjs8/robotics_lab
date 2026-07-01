"""--blank-depth ablation contract for OpenpiRemoteActionSource.

Locks the deploy-side behaviour: with blank_depth set, the source still emits the
``*_wrist_0_depth`` image keys (so an RGB-D openpi server's input transform is
satisfied and the 5-camera token structure stays training-matched) but fills them
with a constant all-far frame and does NOT read live depth. The control case keeps
the existing fail-closed behaviour when real depth is requested but missing.

Guarded by torch/cv2 availability (OpenpiRemoteActionSource imports flow_inference
-> torch, and _raw_camera_images imports cv2).
"""
from __future__ import annotations

import io
import unittest

try:
    import cv2  # noqa: F401
    import numpy as np

    from policy_runner.openpi_remote import OpenpiRemoteActionSource
except Exception:  # torch / cv2 (transitive imports) may be absent
    np = None
    OpenpiRemoteActionSource = None  # type: ignore[assignment]


class _Frame:
    def __init__(self, pixels):
        self.pixels = pixels


class _Bundle:
    def __init__(self, frames):
        self.frames = frames


@unittest.skipIf(OpenpiRemoteActionSource is None, "torch/cv2 not installed")
class BlankDepthAblationTest(unittest.TestCase):
    def _source(self, *, blank_depth: bool):
        src = OpenpiRemoteActionSource.__new__(OpenpiRemoteActionSource)
        src._fake_images = False
        src.camera_names = ["left_realsense_color", "right_realsense_color"]
        src.depth_camera_names = ["left_realsense_depth", "right_realsense_depth"]
        src.include_depth = True
        src.blank_depth = blank_depth
        src.wrist_crop_frac = 0.0
        src.depth_z_near_mm = 120.0
        src.depth_z_far_mm = 700.0
        src.depth_units_m = 1e-4
        src.stderr = io.StringIO()
        src._logged_wrist_shape = True  # skip the one-time wrist-shape print
        return src

    def test_blank_depth_synthesizes_far_frame_without_live_depth(self):
        # RGB-only bundle (NO depth frames) — blank mode must still produce depth.
        rgb_l = np.full((48, 64, 3), 30, dtype=np.uint8)
        rgb_r = np.full((40, 50, 3), 60, dtype=np.uint8)
        src = self._source(blank_depth=True)
        src._poll_camera_bundle = lambda: _Bundle(
            {
                "left_realsense_color": _Frame(rgb_l),
                "right_realsense_color": _Frame(rgb_r),
            }
        )

        images, _decode, missing = src._raw_camera_images()

        self.assertIsNotNone(images)  # no fail-closed despite missing live depth
        self.assertEqual(missing, 0)
        self.assertIn("left_depth", images)
        self.assertIn("right_depth", images)
        # All-far (255), shaped from the matching RGB frame, 3-channel uint8.
        self.assertEqual(images["left_depth"].shape, (48, 64, 3))
        self.assertEqual(images["right_depth"].shape, (40, 50, 3))
        self.assertEqual(images["left_depth"].dtype, np.uint8)
        self.assertTrue((images["left_depth"] == 255).all())
        self.assertTrue((images["right_depth"] == 255).all())

    def test_live_depth_fail_closed_when_missing(self):
        # Control: with blank_depth off, a bundle lacking depth fails closed (None),
        # i.e. the policy emits no motion rather than silently dropping depth.
        rgb = np.zeros((48, 64, 3), dtype=np.uint8)
        src = self._source(blank_depth=False)
        src._poll_camera_bundle = lambda: _Bundle(
            {
                "left_realsense_color": _Frame(rgb),
                "right_realsense_color": _Frame(rgb.copy()),
            }
        )

        images, _decode, _missing = src._raw_camera_images()

        self.assertIsNone(images)


if __name__ == "__main__":
    unittest.main()
