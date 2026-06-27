"""CameraConfig wrist-source / fisheye-crop parsing + validation (no torch needed)."""
import unittest

from policy_runner.config import CameraConfig, config_from_mapping


class CameraWristSourceConfigTest(unittest.TestCase):
    def test_defaults_are_realsense_no_crop(self) -> None:
        cfg = CameraConfig()
        self.assertEqual(cfg.wrist_source, "realsense")
        self.assertEqual(cfg.wrist_crop_frac, 0.0)

    def test_fisheye_source_with_crop_parses(self) -> None:
        cfg = config_from_mapping(
            {
                "mode": "real",
                "camera": {
                    "enable": True,
                    "wrist_source": "fisheye",
                    "wrist_crop_frac": 0.65,
                    "expected_cameras": ["left_fisheye_color", "right_fisheye_color"],
                },
            }
        )
        self.assertEqual(cfg.camera.wrist_source, "fisheye")
        self.assertAlmostEqual(cfg.camera.wrist_crop_frac, 0.65)

    def test_rejects_bad_wrist_source(self) -> None:
        with self.assertRaises(ValueError):
            CameraConfig(wrist_source="zed")

    def test_rejects_out_of_range_crop(self) -> None:
        with self.assertRaises(ValueError):
            CameraConfig(wrist_crop_frac=1.5)
        with self.assertRaises(ValueError):
            CameraConfig(wrist_crop_frac=-0.1)


if __name__ == "__main__":
    unittest.main()
