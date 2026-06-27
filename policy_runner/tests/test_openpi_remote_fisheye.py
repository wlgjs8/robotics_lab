"""Fisheye wrist-camera crop wiring in OpenpiRemoteActionSource.

Locks the training-time 0.65 center-crop (640x480 fisheye -> 416x312) that
_raw_camera_images must apply before sending left/right_wrist_0_rgb to the openpi
server (the server only resize_with_pads). Guarded by torch availability
(OpenpiRemoteActionSource imports flow_inference, which imports torch).
"""

from __future__ import annotations

import sys
import unittest

try:
    import numpy as np

    from policy_runner.openpi_remote import OpenpiRemoteActionSource, _center_crop
except Exception:  # torch (a transitive import) may be absent
    np = None
    OpenpiRemoteActionSource = None  # type: ignore[assignment]
    _center_crop = None  # type: ignore[assignment]


class _Frame:
    def __init__(self, pixels):
        self.pixels = pixels


class _Bundle:
    def __init__(self, frames):
        self.frames = frames


@unittest.skipIf(_center_crop is None, "torch is not installed")
class CenterCropTest(unittest.TestCase):
    def test_065_crop_of_fisheye_is_312x416(self) -> None:
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        out = _center_crop(img, 0.65)
        self.assertEqual(out.shape, (312, 416, 3))

    def test_crop_is_centered(self) -> None:
        # Mark the exact expected crop window [84:396, 112:528] with a sentinel and
        # confirm the crop returns precisely that region.
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        img[84:396, 112:528, :] = 7
        out = _center_crop(img, 0.65)
        self.assertTrue(np.all(out == 7))


@unittest.skipIf(OpenpiRemoteActionSource is None, "torch is not installed")
class RawCameraImagesCropTest(unittest.TestCase):
    def _make_source(self, *, wrist_crop_frac: float):
        src = OpenpiRemoteActionSource.__new__(OpenpiRemoteActionSource)
        src._fake_images = False
        src.stderr = sys.stderr
        src.camera_names = ["left_fisheye_color", "right_fisheye_color"]
        src.wrist_crop_frac = wrist_crop_frac
        # Bundle keyed '<camera>.color' (resolve_frame bridges to '<camera>_color').
        frames = {
            "left_fisheye.color": _Frame(np.zeros((480, 640, 3), dtype=np.uint8)),
            "right_fisheye.color": _Frame(np.zeros((480, 640, 3), dtype=np.uint8)),
        }
        src._poll_camera_bundle = lambda: _Bundle(frames)
        return src

    def test_crop_applied_when_frac_set(self) -> None:
        src = self._make_source(wrist_crop_frac=0.65)
        images, decoded, missing = src._raw_camera_images()
        self.assertIsNotNone(images)
        self.assertEqual(missing, 0)
        self.assertEqual(decoded, 2)
        self.assertEqual(images["left"].shape, (312, 416, 3))
        self.assertEqual(images["right"].shape, (312, 416, 3))

    def test_no_crop_when_frac_zero(self) -> None:
        src = self._make_source(wrist_crop_frac=0.0)
        images, _, _ = src._raw_camera_images()
        self.assertEqual(images["left"].shape, (480, 640, 3))


if __name__ == "__main__":
    unittest.main()
