import unittest

from policy_runner.camera_preview import _image_window_size, _resize_window_to_image


class _Image:
    def __init__(self, shape):
        self.shape = shape


class _Cv2:
    def __init__(self):
        self.resize_calls = []

    def resizeWindow(self, title, width, height):  # noqa: N802 - mirrors cv2 API
        self.resize_calls.append((title, width, height))


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


if __name__ == "__main__":
    unittest.main()
