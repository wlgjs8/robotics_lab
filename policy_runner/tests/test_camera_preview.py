import contextlib
import io
import os
import unittest
from unittest import mock

from policy_runner.camera_preview import (
    _image_window_size,
    _opencv_gui_backend,
    _opencv_gui_error,
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


class _Cv2WithCurrentUi:
    def __init__(self, backend, build_gui="QT5"):
        self.backend = backend
        self.build_gui = build_gui

    def currentUIFramework(self):  # noqa: N802 - mirrors cv2 API
        return self.backend

    def getBuildInformation(self):  # noqa: N802 - mirrors cv2 API
        return f"General configuration\n  GUI: {self.build_gui}\n"


class _Cv2NamedWindowFailure(_Cv2WithCurrentUi):
    WINDOW_NORMAL = 0

    def namedWindow(self, _title, _flags):  # noqa: N802 - mirrors cv2 API
        raise RuntimeError("window backend initialization failed")


class _LegacyCv2:
    def __init__(self, build_gui):
        self.build_gui = build_gui

    def getBuildInformation(self):  # noqa: N802 - mirrors cv2 API
        return f"General configuration\n  GUI: {self.build_gui}\n"


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

    def test_check_gui_flag(self):
        from policy_runner.camera_preview import _parse_args

        self.assertTrue(_parse_args(["--check-gui"]).check_gui)


class CameraPreviewGuiPreflightTest(unittest.TestCase):
    def test_opencv5_empty_runtime_backend_is_authoritative(self):
        cv2 = _Cv2WithCurrentUi("", build_gui="QT5")

        self.assertIsNone(_opencv_gui_backend(cv2))

    def test_legacy_build_info_rejects_space_padded_none(self):
        cv2 = _LegacyCv2("                           NONE")

        self.assertIsNone(_opencv_gui_backend(cv2))

    def test_legacy_build_info_accepts_gui_backend(self):
        self.assertEqual(_opencv_gui_backend(_LegacyCv2("GTK3")), "GTK3")

    def test_linux_gui_backend_requires_display(self):
        backend, error = _opencv_gui_error(
            _Cv2WithCurrentUi("QT"), environ={}, platform="linux"
        )

        self.assertEqual(backend, "QT")
        self.assertIn("DISPLAY", error)

    def test_linux_gui_backend_accepts_x11_display(self):
        backend, error = _opencv_gui_error(
            _Cv2WithCurrentUi("QT"), environ={"DISPLAY": ":1"}, platform="linux"
        )

        self.assertEqual(backend, "QT")
        self.assertIsNone(error)

    def test_check_gui_cli_succeeds_without_opening_window(self):
        from policy_runner.camera_preview import main

        fake_cv2 = _Cv2WithCurrentUi("QT")
        stdout = io.StringIO()
        with (
            mock.patch.dict(os.environ, {"DISPLAY": ":1"}),
            mock.patch.dict("sys.modules", {"cv2": fake_cv2}),
            contextlib.redirect_stdout(stdout),
        ):
            result = main(["--check-gui"])

        self.assertEqual(result, 0)
        self.assertIn("backend=QT", stdout.getvalue())

    def test_check_gui_cli_rejects_headless_wheel(self):
        from policy_runner.camera_preview import main

        fake_cv2 = _Cv2WithCurrentUi("")
        stderr = io.StringIO()
        with (
            mock.patch.dict(os.environ, {"DISPLAY": ":1"}),
            mock.patch.dict("sys.modules", {"cv2": fake_cv2}),
            contextlib.redirect_stderr(stderr),
        ):
            result = main(["--check-gui"])

        self.assertEqual(result, 2)
        self.assertIn("opencv-python-headless", stderr.getvalue())
        self.assertIn("HighGUI backend: none", stderr.getvalue())

    def test_named_window_failure_returns_actionable_error(self):
        from policy_runner.camera_preview import main

        fake_cv2 = _Cv2NamedWindowFailure("QT")
        stderr = io.StringIO()
        with (
            mock.patch.dict(os.environ, {"DISPLAY": ":1"}),
            mock.patch.dict("sys.modules", {"cv2": fake_cv2}),
            contextlib.redirect_stderr(stderr),
        ):
            result = main([])

        self.assertEqual(result, 2)
        self.assertIn("namedWindow failed", stderr.getvalue())
        self.assertIn("window backend initialization failed", stderr.getvalue())


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
