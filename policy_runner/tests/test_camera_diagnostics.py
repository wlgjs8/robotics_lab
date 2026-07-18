from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from policy_runner.camera_diagnostics import BackgroundRgbSnapshotWriter, rgb_image_metrics


class CameraDiagnosticsTest(unittest.TestCase):
    def test_rgb_metrics_distinguish_flat_from_sharp_image(self) -> None:
        flat = np.full((32, 32, 3), 100, dtype=np.uint8)
        sharp = flat.copy()
        sharp[:, ::2] = 255
        flat_metrics = rgb_image_metrics(flat)
        sharp_metrics = rgb_image_metrics(sharp)
        self.assertEqual(flat_metrics["luminance_mean"], 100.0)
        self.assertGreater(
            sharp_metrics["focus_gradient_energy"],
            flat_metrics["focus_gradient_energy"],
        )

    def test_snapshot_writer_is_off_by_default(self) -> None:
        with mock.patch.dict("os.environ", {}, clear=True):
            writer = BackgroundRgbSnapshotWriter()
        writer.submit(1, {"left": np.zeros((2, 2, 3)), "right": np.zeros((2, 2, 3))})
        self.assertFalse(writer.snapshot()["enabled"])
        writer.close()

    def test_invalid_bundle_cap_falls_back_without_breaking_inference_setup(self) -> None:
        with mock.patch.dict(
            "os.environ",
            {
                "FLOW_INFER_DIAGNOSTIC_IMAGES": "off",
                "FLOW_INFER_DIAGNOSTIC_IMAGE_MAX_BUNDLES": "invalid",
            },
            clear=True,
        ):
            writer = BackgroundRgbSnapshotWriter()
        self.assertEqual(writer.snapshot()["max_bundles"], 120)
        writer.close()

    def test_snapshot_writer_saves_post_crop_pair_and_counts_cap_drop(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            writer = BackgroundRgbSnapshotWriter(tmp, max_bundles=1)
            images = {
                "left": np.full((8, 10, 3), 20, dtype=np.uint8),
                "right": np.full((8, 10, 3), 220, dtype=np.uint8),
            }
            writer.submit(17, images)
            writer.submit(18, images)
            writer.close()
            snapshot = writer.snapshot()
            self.assertEqual(snapshot["written_bundles"], 1)
            self.assertEqual(snapshot["cap_drops"], 1)
            self.assertTrue((Path(tmp) / "bundle_0000000017_left.jpg").is_file())
            self.assertTrue((Path(tmp) / "bundle_0000000017_right.jpg").is_file())


if __name__ == "__main__":
    unittest.main()
