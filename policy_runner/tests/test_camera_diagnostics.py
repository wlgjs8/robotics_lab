from __future__ import annotations

import tempfile
import time
import unittest
from datetime import datetime
from pathlib import Path
from unittest import mock

import numpy as np

from policy_runner.camera_diagnostics import (
    BackgroundRgbSnapshotWriter,
    rgb_image_metrics,
    run_directory,
)


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
        snapshot = writer.snapshot()
        self.assertIsNone(snapshot["max_bundles"])
        self.assertEqual(snapshot["max_seconds"], 60.0)
        writer.close()

    def test_default_budget_is_sixty_seconds_not_a_bundle_count(self) -> None:
        with mock.patch.dict("os.environ", {"FLOW_INFER_DIAGNOSTIC_IMAGES": "off"}, clear=True):
            writer = BackgroundRgbSnapshotWriter()
        self.assertIsNone(writer.max_bundles)
        self.assertEqual(writer.max_seconds, 60.0)
        writer.close()

    def test_an_explicit_bundle_cap_replaces_the_duration_default(self) -> None:
        with mock.patch.dict(
            "os.environ",
            {
                "FLOW_INFER_DIAGNOSTIC_IMAGES": "off",
                "FLOW_INFER_DIAGNOSTIC_IMAGE_MAX_BUNDLES": "20000",
            },
            clear=True,
        ):
            writer = BackgroundRgbSnapshotWriter()
        self.assertEqual(writer.max_bundles, 20000)
        self.assertIsNone(writer.max_seconds)
        writer.close()

    def test_duration_budget_stops_submissions_after_the_window(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            writer = BackgroundRgbSnapshotWriter(tmp, max_seconds=0.05)
            images = {
                "left": np.full((8, 10, 3), 20, dtype=np.uint8),
                "right": np.full((8, 10, 3), 220, dtype=np.uint8),
            }
            writer.submit(1, images)
            time.sleep(0.08)
            writer.submit(2, images)
            writer.close()
            snapshot = writer.snapshot()
            self.assertEqual(snapshot["written_bundles"], 1)
            self.assertEqual(snapshot["cap_drops"], 1)
            run_dir = Path(str(snapshot["directory"]))
            self.assertTrue((run_dir / "bundle_0000000001_left.jpg").is_file())
            self.assertFalse((run_dir / "bundle_0000000002_left.jpg").is_file())

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
            run_dir = Path(str(snapshot["directory"]))
            self.assertEqual(run_dir.parent, Path(tmp))
            self.assertTrue(run_dir.name.startswith("run_"))
            self.assertTrue((run_dir / "bundle_0000000017_left.jpg").is_file())
            self.assertTrue((run_dir / "bundle_0000000017_right.jpg").is_file())

    def test_explicit_directory_is_a_parent_so_runs_never_overwrite_each_other(self) -> None:
        # bundle_seq restarts when camera_server restarts, so two runs sharing one
        # directory would overwrite frames under identical names.
        with tempfile.TemporaryDirectory() as tmp:
            images = {
                "left": np.full((8, 10, 3), 20, dtype=np.uint8),
                "right": np.full((8, 10, 3), 220, dtype=np.uint8),
            }
            directories = []
            for _ in range(2):
                writer = BackgroundRgbSnapshotWriter(tmp, max_bundles=1)
                writer.submit(17, images)
                writer.close()
                directories.append(Path(str(writer.snapshot()["directory"])))
            self.assertNotEqual(directories[0], directories[1])
            for directory in directories:
                self.assertEqual(directory.parent, Path(tmp))
                self.assertTrue((directory / "bundle_0000000017_left.jpg").is_file())

    def test_auto_keeps_naming_the_run_under_logs(self) -> None:
        stamp = datetime(2026, 9, 9, 12, 0, 57)
        self.assertEqual(run_directory("auto", now=stamp), Path("logs") / "flow_obs_20260909_120057")
        self.assertEqual(
            run_directory("/tmp/flow_obs_am", now=stamp),
            Path("/tmp/flow_obs_am") / "run_20260909_120057",
        )


if __name__ == "__main__":
    unittest.main()
