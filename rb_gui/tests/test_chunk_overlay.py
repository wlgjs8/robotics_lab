from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rb_servo_gui.chunk_overlay_receiver import ChunkOverlayStore
from rb_servo_gui.models import CHUNK_OVERLAY_SCHEMA_VERSION, ChunkOverlaySnapshot
from rb_servo_gui.scene import update_chunk_overlay


def sample_chunk_overlay(**overrides):
    data = {
        "schema_version": CHUNK_OVERLAY_SCHEMA_VERSION,
        "host_time_ns": 123456789,
        "seq": 7,
        "policy_dt_sec": 0.05,
        "horizon": 2,
        "left": [
            [0.10, 0.20, 0.30, 0.0, 0.0, 0.0, 100.0],
            [0.11, 0.21, 0.31, 0.0, 0.0, 0.0, 90.0],
        ],
        "right": [
            [-0.10, 0.20, 0.30, 0.0, 0.0, 0.0, 100.0],
            [-0.11, 0.21, 0.31, 0.0, 0.0, 0.0, 80.0],
        ],
    }
    data.update(overrides)
    return data


class RecordingSceneHandle:
    def __init__(self) -> None:
        self.points = None
        self.colors = None
        self.visible = False


class ChunkOverlayTest(unittest.TestCase):
    def test_snapshot_parse_accepts_valid_packet_and_positions(self) -> None:
        overlay = ChunkOverlaySnapshot.parse(sample_chunk_overlay(), received_monotonic=100.0)
        self.assertIsNotNone(overlay)
        assert overlay is not None
        self.assertEqual(overlay.schema_version, CHUNK_OVERLAY_SCHEMA_VERSION)
        self.assertEqual(overlay.seq, 7)
        self.assertEqual(overlay.policy_dt_sec, 0.05)
        self.assertEqual(overlay.horizon, 2)
        self.assertEqual(overlay.left_positions, ((0.10, 0.20, 0.30), (0.11, 0.21, 0.31)))
        self.assertEqual(overlay.right_positions, ((-0.10, 0.20, 0.30), (-0.11, 0.21, 0.31)))
        self.assertFalse(overlay.stale(now=104.9, threshold_sec=5.0))
        self.assertTrue(overlay.stale(now=105.1, threshold_sec=5.0))

    def test_snapshot_parse_rejects_wrong_schema_empty_and_nonfinite(self) -> None:
        self.assertIsNone(ChunkOverlaySnapshot.parse(sample_chunk_overlay(schema_version="wrong.schema")))
        self.assertIsNone(ChunkOverlaySnapshot.parse(sample_chunk_overlay(left=None, right=None)))
        self.assertIsNone(ChunkOverlaySnapshot.parse(sample_chunk_overlay(left=[], right=[])))
        bad = sample_chunk_overlay()
        bad["left"][0][2] = float("nan")
        self.assertIsNone(ChunkOverlaySnapshot.parse(bad))

    def test_store_counts_invalid_and_stale_packets(self) -> None:
        store = ChunkOverlayStore(stale_after_sec=5.0)
        self.assertFalse(store.update_from_json_bytes(b"{bad-json"))
        self.assertEqual(store.invalid_packets, 1)
        self.assertTrue(store.update_from_json_bytes(json.dumps(sample_chunk_overlay()).encode(), received_monotonic=10.0))
        self.assertEqual(store.received_packets, 1)
        self.assertFalse(store.is_stale(now=14.0))
        self.assertTrue(store.is_stale(now=16.0))

    def test_scene_update_sets_visible_for_valid_snapshot_and_hides_stale(self) -> None:
        overlay = ChunkOverlaySnapshot.parse(sample_chunk_overlay())
        self.assertIsNotNone(overlay)
        left = RecordingSceneHandle()
        right = RecordingSceneHandle()
        handles = {
            "chunk_overlay_line_mode": "point_cloud",
            "left_chunk_overlay": left,
            "right_chunk_overlay": right,
        }
        update_chunk_overlay(handles, overlay, stale=False, visible=True)
        self.assertTrue(left.visible)
        self.assertTrue(right.visible)
        self.assertEqual(tuple(left.points[0]), (0.10, 0.20, 0.30))
        self.assertEqual(tuple(right.points[1]), (-0.11, 0.21, 0.31))

        update_chunk_overlay(handles, overlay, stale=True, visible=True)
        self.assertFalse(left.visible)
        self.assertFalse(right.visible)

        update_chunk_overlay(handles, overlay, stale=False, visible=False)
        self.assertFalse(left.visible)
        self.assertFalse(right.visible)


if __name__ == "__main__":
    unittest.main()
