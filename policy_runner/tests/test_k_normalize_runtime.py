"""Runtime K-normalisation (FLOW_INFER_K_NORMALIZE) — the deploy half of the training-side
`convert_pika_umi_storage_video.py --k-normalize` remap.

The contract these tests pin down:
  * OFF by default, so an ordinary checkpoint keeps seeing raw frames.
  * ON, the inference unit's principal point lands on the virtual camera's centre -- that is what
    "one virtual camera" means, and it is what a `boltv2r6knorm*` checkpoint was trained on.
  * The inference K is fail-closed. A guessed fy would silently rescale the image the policy aims
    with, so a missing/!malformed value must raise rather than fall back to the collection unit's.
  * PP_ALIGN and K_NORMALIZE cannot both run: they correct the same collect->infer mismatch.
  * The crop matches the training crop (cut every edge, resize back), so the served frame keeps the
    dataset's shape.
"""

import os
import unittest

try:
    import numpy as np

    from policy_runner import openpi_remote as om
except Exception:  # torch/h5py (transitive imports) may be absent, as in test_openpi_remote_rtc
    np = None
    om = None  # type: ignore[assignment]

K_LEFT = "390.21,390.0,321.91,237.25"   # measured inference left unit (fy stands in for the test)
K_RIGHT = "395.71,395.0,323.47,229.66"


def _frame(h: int = 480, w: int = 640):
    rng = np.random.default_rng(0)
    return rng.integers(0, 255, (h, w, 3), dtype=np.uint8)


@unittest.skipIf(om is None, "openpi_remote unavailable (torch/h5py missing)")
class KNormalizeRuntimeTest(unittest.TestCase):
    def setUp(self) -> None:
        self._saved = {
            k: os.environ.get(k)
            for k in (
                "FLOW_INFER_K_NORMALIZE",
                "FLOW_INFER_K_VIRTUAL",
                "FLOW_INFER_K_LEFT",
                "FLOW_INFER_K_RIGHT",
                "FLOW_INFER_CROP_PX",
                "FLOW_INFER_PP_ALIGN",
            )
        }
        for k in self._saved:
            os.environ.pop(k, None)

    def tearDown(self) -> None:
        for k, v in self._saved.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v

    def test_disabled_by_default_is_identity(self) -> None:
        rgb = _frame()
        self.assertIs(om._k_normalize(rgb, "left"), rgb)

    def test_principal_point_lands_on_the_virtual_centre(self) -> None:
        os.environ["FLOW_INFER_K_NORMALIZE"] = "1"
        os.environ["FLOW_INFER_K_LEFT"] = K_LEFT
        # A single bright pixel at the unit's principal point must come out at the virtual centre.
        rgb = np.zeros((480, 640, 3), np.uint8)
        rgb[round(237.25), round(321.91)] = 255
        out = om._k_normalize(rgb, "left")
        vfx, vfy, vcx, vcy = om._k_virtual()
        ys, xs = np.nonzero(out.max(axis=2))
        self.assertTrue(len(xs) > 0, "the marked pixel disappeared")
        self.assertLess(abs(xs.mean() - vcx), 1.0)
        self.assertLess(abs(ys.mean() - vcy), 1.0)

    def test_black_border_not_edge_replicated(self) -> None:
        # The left unit's pp sits 2.75 px below the virtual centre, so the warp pulls in rows the
        # real camera never saw. Training frames carry those BLACK (the converter warps with
        # BORDER_CONSTANT), so the deploy path must not fill them by replicating the edge --
        # a replicated edge would hand the policy invented image content in the border band.
        os.environ["FLOW_INFER_K_NORMALIZE"] = "1"
        os.environ["FLOW_INFER_K_LEFT"] = K_LEFT
        flat = np.full((480, 640, 3), 200, np.uint8)
        out = om._k_normalize(flat, "left")
        self.assertEqual(out.shape, (480, 640, 3))
        self.assertEqual(int(out[240, 320, 0]), 200, "the interior must be untouched")
        # Edge replication would keep the top row at the flat value; the black border darkens it.
        self.assertLess(int(out[0, :, :].max()), 100, "top row should be dominated by black border")

    def test_crop_px_restores_shape_and_removes_the_border(self) -> None:
        os.environ["FLOW_INFER_K_NORMALIZE"] = "1"
        os.environ["FLOW_INFER_K_LEFT"] = K_LEFT
        os.environ["FLOW_INFER_CROP_PX"] = "24,18"
        out = om._k_normalize(np.full((480, 640, 3), 200, np.uint8), "left")
        self.assertEqual(out.shape, (480, 640, 3))
        self.assertGreater(int(out[0, :, :].min()), 0, "24/18 crop should cut the black wedge away")

    def test_matches_the_training_transform(self) -> None:
        import cv2

        os.environ["FLOW_INFER_K_NORMALIZE"] = "1"
        os.environ["FLOW_INFER_K_RIGHT"] = K_RIGHT
        os.environ["FLOW_INFER_CROP_PX"] = "24,18"
        rgb = _frame()
        fx, fy, ppx, ppy = (float(v) for v in K_RIGHT.split(","))
        k = np.array([[fx, 0.0, ppx], [0.0, fy, ppy], [0.0, 0.0, 1.0]])
        kv = np.array([[393.0, 0.0, 320.0], [0.0, 393.0, 240.0], [0.0, 0.0, 1.0]])
        want = cv2.warpPerspective(
            rgb, kv @ np.linalg.inv(k), (640, 480), flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0),
        )
        want = cv2.resize(want[18:462, 24:616], (640, 480), interpolation=cv2.INTER_LINEAR)
        np.testing.assert_array_equal(om._k_normalize(rgb, "right"), want)

    def test_missing_inference_k_fails_closed(self) -> None:
        os.environ["FLOW_INFER_K_NORMALIZE"] = "1"
        with self.assertRaises(ValueError) as ctx:
            om._k_normalize(_frame(), "left")
        self.assertIn("FLOW_INFER_K_LEFT", str(ctx.exception))

    def test_malformed_values_raise(self) -> None:
        os.environ["FLOW_INFER_K_NORMALIZE"] = "1"
        os.environ["FLOW_INFER_K_LEFT"] = "390.21,321.91,237.25"  # three, not four
        with self.assertRaises(ValueError):
            om._k_normalize(_frame(), "left")
        os.environ["FLOW_INFER_K_LEFT"] = "0,390,320,240"  # zero focal length
        with self.assertRaises(ValueError):
            om._k_normalize(_frame(), "left")

    def test_crop_cannot_eat_the_frame(self) -> None:
        os.environ["FLOW_INFER_K_NORMALIZE"] = "1"
        os.environ["FLOW_INFER_K_LEFT"] = K_LEFT
        os.environ["FLOW_INFER_CROP_PX"] = "400,18"
        with self.assertRaises(ValueError):
            om._k_normalize(_frame(), "left")

    def test_pp_align_and_k_normalize_are_mutually_exclusive(self) -> None:
        os.environ["FLOW_INFER_K_NORMALIZE"] = "1"
        os.environ["FLOW_INFER_K_LEFT"] = K_LEFT
        os.environ["FLOW_INFER_PP_ALIGN"] = "1"
        with self.assertRaises(ValueError):
            om._align_principal_point(_frame(), "left")

    def test_virtual_camera_default_matches_the_converter(self) -> None:
        self.assertEqual(om._k_virtual(), (393.0, 393.0, 320.0, 240.0))
        os.environ["FLOW_INFER_K_VIRTUAL"] = "400,400,320,240"
        self.assertEqual(om._k_virtual(), (400.0, 400.0, 320.0, 240.0))


if __name__ == "__main__":
    unittest.main()
