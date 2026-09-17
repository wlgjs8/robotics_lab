import contextlib
import io
from types import SimpleNamespace
import unittest
from unittest import mock

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = np = None

from policy_runner import camera_crop_preview as preview


@unittest.skipIf(cv2 is None or np is None, "OpenCV and numpy required")
class CameraCropTransformsTest(unittest.TestCase):
    def test_landscape_letterbox_has_28_zero_rows_and_preserves_input(self):
        source = np.full((480, 640, 3), [20, 80, 160], dtype=np.uint8)
        before = source.copy()
        result = preview.letterbox_224(source)
        self.assertEqual(result.shape, (224, 224, 3))
        self.assertEqual(result.dtype, np.uint8)
        self.assertFalse(result[:28].any())
        self.assertFalse(result[196:].any())
        np.testing.assert_array_equal(result[28:196], np.broadcast_to([20, 80, 160], (168, 224, 3)))
        np.testing.assert_array_equal(source, before)

    def test_portrait_letterbox_has_28_zero_columns(self):
        source = np.full((640, 480, 3), 70, dtype=np.uint8)
        result = preview.letterbox_224(source)
        self.assertFalse(result[:, :28].any())
        self.assertFalse(result[:, 196:].any())
        self.assertTrue((result[:, 28:196] == 70).all())

    def test_exact_center_crop_discards_both_side_strips(self):
        source = np.full((480, 640, 3), 240, dtype=np.uint8)
        source[:, 80:560] = [11, 40, 99]
        result = preview.center_crop_224(source)
        np.testing.assert_array_equal(result, np.broadcast_to([11, 40, 99], (224, 224, 3)))

    def test_odd_crop_is_centered_on_both_axes(self):
        source = np.full((601, 801, 3), 255, dtype=np.uint8)
        source[60:540, 160:640] = 60
        self.assertTrue((preview.center_crop_224(source) == 60).all())

    def test_small_input_does_not_silently_change_crop(self):
        for shape in ((479, 640, 3), (640, 479, 3)):
            with self.subTest(shape=shape), self.assertRaisesRegex(ValueError, "source >=480x480"):
                preview.center_crop_224(np.zeros(shape, dtype=np.uint8))

    def test_square_480_outputs_match_and_have_no_red_diff(self):
        source = np.random.default_rng(7).integers(0, 256, (480, 480, 3), dtype=np.uint8)
        padded = preview.letterbox_224(source)
        cropped = preview.center_crop_224(source)
        np.testing.assert_array_equal(padded, cropped)
        aligned, footprint = preview.align_letterbox_center(padded, source.shape)
        np.testing.assert_array_equal(aligned, padded)
        self.assertEqual(footprint, (224, 224))
        overlay, difference = preview.pixel_diff_overlay(aligned, cropped)
        self.assertFalse(difference.any())
        np.testing.assert_array_equal(overlay[..., 0], overlay[..., 2])

    def test_diff_is_absolute_max_channel_without_uint8_wrap(self):
        a = np.array([[[0, 0, 0], [250, 2, 8], [20, 20, 20]]], dtype=np.uint8)
        b = np.array([[[255, 255, 255], [2, 100, 5], [20, 20, 20]]], dtype=np.uint8)
        overlay, difference = preview.pixel_diff_overlay(a, b)
        np.testing.assert_array_equal(difference, [[255, 248, 0]])
        np.testing.assert_array_equal(overlay[0, 0], [0, 0, 255])
        np.testing.assert_array_equal(overlay[0, 2], [20, 20, 20])
        reverse, reverse_diff = preview.pixel_diff_overlay(b, a)
        np.testing.assert_array_equal(reverse, overlay)
        np.testing.assert_array_equal(reverse_diff, difference)

    def test_alignment_excludes_padding_and_reports_center_sample_gain(self):
        for shape in ((480, 640, 3), (640, 480, 3)):
            with self.subTest(shape=shape):
                source = np.full(shape, [80, 120, 150], dtype=np.uint8)
                padded = preview.letterbox_224(source)
                cropped = preview.center_crop_224(source)
                aligned, footprint = preview.align_letterbox_center(padded, source.shape)
                self.assertEqual(footprint, (168, 168))
                np.testing.assert_array_equal(aligned, cropped)
                _, diff = preview.pixel_diff_overlay(aligned, cropped)
                self.assertFalse(diff.any())
                self.assertEqual(224 ** 2 - footprint[0] * footprint[1], 21952)
                self.assertAlmostEqual(224 ** 2 / (footprint[0] * footprint[1]), 16 / 9)

    def test_alignment_matches_source_coordinates_in_odd_landscape_and_portrait(self):
        for height, width in ((601, 801), (801, 601), (480, 641)):
            with self.subTest(height=height, width=width):
                yy, xx = np.indices((height, width))
                source = np.stack((xx * 255 / width, yy * 255 / height, np.zeros_like(xx)), axis=2).astype(np.uint8)
                aligned, _ = preview.align_letterbox_center(preview.letterbox_224(source), source.shape)
                cropped = preview.center_crop_224(source)
                # A smooth coordinate ramp should agree everywhere after alignment,
                # to within uint8 interpolation rounding (not scale-displaced edges).
                self.assertLessEqual(int(cv2.absdiff(aligned, cropped).max()), 1)

    def test_fine_texture_still_has_detail_diff_after_alignment(self):
        yy, xx = np.indices((480, 640))
        source = np.repeat((((xx // 3 + yy // 3) % 2) * 255).astype(np.uint8)[..., None], 3, axis=2)
        aligned, _ = preview.align_letterbox_center(preview.letterbox_224(source), source.shape)
        cropped = preview.center_crop_224(source)
        overlay, diff = preview.pixel_diff_overlay(aligned, cropped)
        self.assertGreater(float(diff.mean()), 10)
        self.assertTrue((overlay[..., 2] > overlay[..., 0]).any())

    def test_display_gain_does_not_change_raw_diff(self):
        a = np.full((224, 224, 3), 100, dtype=np.uint8)
        b = np.full_like(a, 110)
        normal, normal_diff = preview.pixel_diff_overlay(a, b, 1)
        amplified, amplified_diff = preview.pixel_diff_overlay(a, b, 4)
        np.testing.assert_array_equal(normal_diff, amplified_diff)
        self.assertTrue((normal_diff == 10).all())
        self.assertTrue((amplified[..., 2] > normal[..., 2]).all())
        self.assertTrue((amplified[..., 0] < normal[..., 0]).all())

    def test_render_preserves_rgb_source_and_keeps_labels_outside_outputs(self):
        rgb = np.full((480, 640, 3), [200, 90, 10], dtype=np.uint8)
        original = rgb.copy()
        bundle = SimpleNamespace(frames={"left_realsense.color": SimpleNamespace(pixels=rgb)})
        comparison, overlay = preview.render_views(bundle, ["left_realsense_color"], True)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        a, b = preview.letterbox_224(bgr), preview.center_crop_224(bgr)
        np.testing.assert_array_equal(comparison[76:300, 12:236], a)
        np.testing.assert_array_equal(comparison[76:300, 260:484], b)
        aligned, _ = preview.align_letterbox_center(a, bgr.shape)
        np.testing.assert_array_equal(overlay[76:300, 12:236], aligned)
        np.testing.assert_array_equal(overlay[76:300, 260:484], b)
        np.testing.assert_array_equal(overlay[76:300, 508:732], preview.pixel_diff_overlay(aligned, b, 4)[0])
        np.testing.assert_array_equal(rgb, original)

    def test_missing_small_invalid_and_stale_frames_are_labeled(self):
        frames = {
            "small": SimpleNamespace(pixels=np.full((200, 640, 3), 90, dtype=np.uint8)),
            "invalid": SimpleNamespace(pixels=np.zeros((480, 640), dtype=np.uint16)),
            "valid": SimpleNamespace(pixels=np.full((480, 640, 3), 100, dtype=np.uint8)),
        }
        with mock.patch.object(preview, "_card", wraps=preview._card) as card:
            comparison, overlay = preview.render_views(
                SimpleNamespace(frames=frames), ["absent", "small", "invalid", "valid"], False
            )
        self.assertEqual(comparison.shape, (4 * preview.CARD_HEIGHT, 496, 3))
        self.assertEqual(overlay.shape, (4 * preview.CARD_HEIGHT, 744, 3))
        self.assertEqual([call.args[3] for call in card.call_args_list],
                         ["MISSING"] * 5 + ["STALE"] + ["TOO SMALL"] * 4 +
                         ["INVALID"] * 5 + ["STALE"] * 5)
        self.assertFalse(overlay[preview.CARD_HEIGHT + 76:preview.CARD_HEIGHT + 300, 12:236].any())


class CameraCropArgsTest(unittest.TestCase):
    def test_defaults_and_camera_override(self):
        args = preview._parse_args([])
        self.assertEqual(args.topic, "camera.bundle.policy")
        self.assertEqual(args.diff_gain, 4.0)
        self.assertEqual(preview._parse_args(["--diff-gain", "1"]).diff_gain, 1.0)
        self.assertEqual(args.cameras, ["left_realsense_color", "right_realsense_color"])
        self.assertEqual(preview._parse_args(["--cameras", " left.color, "]).cameras, ["left.color"])

    def test_rejects_empty_cameras_and_invalid_numeric_values(self):
        for argv in (["--cameras", ","], ["--scale", "0"], ["--scale", "nan"],
                     ["--refresh-hz", "inf"], ["--max-age-ms", "-1"], ["--diff-gain", "0"]):
            with self.subTest(argv=argv), contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                preview._parse_args(argv)


@unittest.skipIf(cv2 is None or np is None, "OpenCV and numpy required")
class CameraCropLifecycleTest(unittest.TestCase):
    def test_gui_preflight_does_not_subscribe_or_open_windows(self):
        with (
            mock.patch.object(preview, "_opencv_gui_error", return_value=("QT", None)),
            mock.patch.object(preview, "CameraBundleClient") as client,
            mock.patch.object(cv2, "namedWindow") as named,
            contextlib.redirect_stdout(io.StringIO()),
        ):
            self.assertEqual(preview.main(["--check-gui"]), 0)
        named.assert_not_called()
        client.assert_not_called()

    def test_quit_escape_and_window_close_clean_up_both_windows_and_subscription(self):
        for key, visible in ((ord("q"), 1), (27, 1), (-1, 0)):
            with self.subTest(key=key, visible=visible), contextlib.ExitStack() as stack:
                stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
                stack.enter_context(mock.patch.object(preview, "_opencv_gui_error", return_value=("QT", None)))
                client = stack.enter_context(mock.patch.object(preview, "CameraBundleClient"))
                client.return_value.poll.return_value = None
                client.return_value.latest.return_value = None
                gui = {name: stack.enter_context(mock.patch.object(cv2, name)) for name in
                       ("namedWindow", "imshow", "resizeWindow", "moveWindow", "destroyWindow")}
                stack.enter_context(mock.patch.object(cv2, "waitKey", return_value=key))
                stack.enter_context(mock.patch.object(cv2, "getWindowProperty", return_value=visible))
                self.assertEqual(preview.main([]), 0)
                self.assertEqual(gui["imshow"].call_count, 2)
                client.return_value.close.assert_called_once()
                self.assertEqual(gui["destroyWindow"].call_args_list,
                                 [mock.call(preview.COMPARE_TITLE), mock.call(preview.DIFF_TITLE)])


if __name__ == "__main__":
    unittest.main()
