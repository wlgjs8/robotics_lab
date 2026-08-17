import os
import unittest
from unittest import mock

from policy_runner.camera_preview import (
    _image_window_size,
    _opencv_gui_backend,
    _repair_qt_font_path,
    _resize_window_to_image,
)


class _Image:
    def __init__(self, shape):
        self.shape = shape


class _Cv2:
    def __init__(self):
        self.resize_calls = []

    def resizeWindow(self, title, width, height):  # noqa: N802 - mirrors cv2 API
        self.resize_calls.append((title, width, height))


class _Cv2Build:
    def __init__(self, gui):
        self.gui = gui

    def getBuildInformation(self):  # noqa: N802 - mirrors cv2 API
        return f"General configuration\n  GUI: {self.gui}\n"


class CameraPreviewBackendTest(unittest.TestCase):
    def test_detects_gui_backend(self):
        self.assertEqual(_opencv_gui_backend(_Cv2Build("QT5")), "QT5")

    def test_rejects_headless_or_missing_backend(self):
        self.assertIsNone(_opencv_gui_backend(_Cv2Build("NONE")))

        class _NoGuiLine:
            @staticmethod
            def getBuildInformation():  # noqa: N802 - mirrors cv2 API
                return "General configuration only"

        self.assertIsNone(_opencv_gui_backend(_NoGuiLine()))

    def test_repairs_missing_qt_wheel_font_directory(self):
        dejavu = "/usr/share/fonts/truetype/dejavu"
        with mock.patch.dict(os.environ, {"QT_QPA_FONTDIR": "/missing"}):
            with mock.patch(
                "policy_runner.camera_preview.os.path.isdir",
                side_effect=lambda path: path == dejavu,
            ):
                _repair_qt_font_path()
                self.assertEqual(os.environ["QT_QPA_FONTDIR"], dejavu)


class CameraPreviewSizingTest(unittest.TestCase):
    def test_window_size_uses_image_width_height(self):
        image = _Image((480, 1280, 3))

        self.assertEqual(_image_window_size(image), (1280, 480))

    def test_resize_window_tracks_mosaic_size_once_per_size(self):
        cv2 = _Cv2()
        title = "preview"
        image = _Image((480, 1280, 3))

        size = _resize_window_to_image(cv2, title, image, None)
        same_size = _resize_window_to_image(cv2, title, image, size)
        new_size = _resize_window_to_image(cv2, title, _Image((720, 1280, 3)), same_size)

        self.assertEqual(size, (1280, 480))
        self.assertEqual(same_size, (1280, 480))
        self.assertEqual(new_size, (1280, 720))
        self.assertEqual(
            cv2.resize_calls,
            [
                ("preview", 1280, 480),
                ("preview", 1280, 720),
            ],
        )


class CameraPreviewArgsTest(unittest.TestCase):
    def test_include_depth_flag_and_params(self):
        from policy_runner.camera_preview import _parse_args

        args = _parse_args(
            [
                "--include-depth",
                "--depth-z-near-mm", "100",
                "--depth-z-far-mm", "800",
                "--depth-units-m", "1e-3",
            ]
        )
        self.assertTrue(args.include_depth)
        self.assertEqual(
            (args.depth_z_near_mm, args.depth_z_far_mm, args.depth_units_m),
            (100.0, 800.0, 1e-3),
        )

    def test_include_depth_defaults_off(self):
        from policy_runner.camera_preview import _parse_args

        self.assertFalse(_parse_args([]).include_depth)


class CameraPreviewDepthRenderTest(unittest.TestCase):
    def _np(self):
        try:
            import numpy as np

            return np
        except ImportError:
            self.skipTest("numpy not available")

    def test_depth_to_image_is_model_depth_channel(self):
        np = self._np()
        from policy_runner.camera_preview import _depth_to_image

        # units 1e-4 m/count -> mm = raw * 0.1; values are 0/120/410/700 mm.
        raw = np.array([[0, 1200, 4100, 7000]], dtype=np.uint16)
        img = _depth_to_image(raw, 120.0, 700.0, 1e-4)
        self.assertEqual(img.shape, (1, 4, 3))
        self.assertEqual(img.dtype, np.uint8)
        self.assertEqual(img[0, 0].tolist(), [255, 255, 255])  # hole(0) -> far
        self.assertEqual(int(img[0, 1, 0]), 0)                 # z_near -> 0
        self.assertEqual(int(img[0, 3, 0]), 255)               # z_far -> 255

    def test_bit_identical_to_openpi_remote(self):
        # The preview MUST render exactly the policy's depth channel, so its
        # _depth_to_image must stay identical to the inference one. Skips when
        # openpi_remote's heavy deps (torch/openpi_client) are unavailable.
        np = self._np()
        from policy_runner.camera_preview import _depth_to_image as preview_d2i

        try:
            from policy_runner.openpi_remote import _depth_to_image as model_d2i
        except Exception as exc:  # noqa: BLE001 - optional heavy deps
            self.skipTest(f"openpi_remote unavailable: {type(exc).__name__}")
        rng = np.arange(0, 8000, 137, dtype=np.uint16).reshape(1, -1)
        np.testing.assert_array_equal(
            preview_d2i(rng, 120.0, 700.0, 1e-4),
            model_d2i(rng, 120.0, 700.0, 1e-4),
        )


if __name__ == "__main__":
    unittest.main()
