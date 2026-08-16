from __future__ import annotations

import unittest

from policy_runner.recording import (
    DATASET_METADATA_SCHEMA,
    build_dataset_metadata,
)


class RecordingMetadataTest(unittest.TestCase):
    def test_dataset_metadata_builder_keeps_required_provenance_keys(self) -> None:
        metadata = build_dataset_metadata(
            git_commit="abc123",
            config_hash="config-sha",
            backend_type="rbpodo",
            run_mode="real",
            operation_mode="simulation",
            physical_motion_expected=False,
            controller_pgmode="simulation",
            calibration_status="configured_estimate",
            camera_status="disabled",
            command_source_id="policy_runner",
            benchmark_linkage={"overlay_run_id": "overlay-1"},
        )

        self.assertEqual(metadata["schema"], DATASET_METADATA_SCHEMA)
        self.assertEqual(metadata["backend_type"], "rbpodo")
        self.assertEqual(metadata["run_mode"], "real")
        self.assertEqual(metadata["operation_mode"], "simulation")
        self.assertFalse(metadata["physical_motion_expected"])
        self.assertEqual(metadata["benchmark_linkage"]["overlay_run_id"], "overlay-1")


if __name__ == "__main__":
    unittest.main()
