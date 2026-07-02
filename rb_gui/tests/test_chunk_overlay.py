from __future__ import annotations

import json
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rb_servo_gui.chunk_overlay_receiver import ChunkOverlayStore
from rb_servo_gui.models import CHUNK_OVERLAY_SCHEMA_VERSION, ChunkOverlaySnapshot
from rb_servo_gui.scene import _pose_triad_segments, update_chunk_overlay


def sample_chunk_overlay(**overrides):
    data = {
        "schema_version": CHUNK_OVERLAY_SCHEMA_VERSION,
        "host_time_ns": 123456789,
        "seq": 7,
        "policy_dt_sec": 0.05,
        "horizon": 2,
        "left": [
            [0.10, 0.20, 0.30, 0.0, 0.0, 0.0, 1.0, 100.0],
            [0.11, 0.21, 0.31, 0.0, 0.0, 0.0, 1.0, 90.0],
        ],
        "right": [
            [-0.10, 0.20, 0.30, 0.0, 0.0, 0.0, 1.0, 100.0],
            [-0.11, 0.21, 0.31, 0.0, 0.0, 0.0, 1.0, 80.0],
        ],
    }
    data.update(overrides)
    return data


class RecordingSceneHandle:
    def __init__(self) -> None:
        self.points = None
        self.colors = None
        self.position = None
        self.point_size = None
        self.scale = None
        self.visible = False


def recording_chunk_overlay_handles(*, mode: str = "line_segments") -> dict[str, object]:
    handles: dict[str, object] = {"chunk_overlay_line_mode": mode}
    for arm in ("left", "right"):
        handles[f"{arm}_chunk_overlay"] = RecordingSceneHandle()
        handles[f"{arm}_chunk_overlay_history"] = RecordingSceneHandle()
        handles[f"{arm}_chunk_overlay_points"] = RecordingSceneHandle()
        handles[f"{arm}_chunk_overlay_cursor"] = RecordingSceneHandle()
        handles[f"{arm}_chunk_overlay_error"] = RecordingSceneHandle()
        handles[f"{arm}_chunk_overlay_axes"] = RecordingSceneHandle()
        handles[f"{arm}_chunk_overlay_cursor_axes"] = RecordingSceneHandle()
    return handles


class ChunkOverlayTest(unittest.TestCase):
    def test_snapshot_parse_accepts_valid_packet_and_positions(self) -> None:
        overlay = ChunkOverlaySnapshot.parse(sample_chunk_overlay(), received_monotonic=100.0)
        self.assertIsNotNone(overlay)
        assert overlay is not None
        self.assertEqual(overlay.schema_version, CHUNK_OVERLAY_SCHEMA_VERSION)
        self.assertEqual(overlay.seq, 7)
        self.assertEqual(overlay.policy_dt_sec, 0.05)
        self.assertEqual(overlay.horizon, 2)
        self.assertEqual(
            overlay.left_poses,
            (
                (0.10, 0.20, 0.30, 0.0, 0.0, 0.0, 1.0),
                (0.11, 0.21, 0.31, 0.0, 0.0, 0.0, 1.0),
            ),
        )
        self.assertEqual(overlay.left_positions, ((0.10, 0.20, 0.30), (0.11, 0.21, 0.31)))
        self.assertEqual(overlay.left_positions[0], overlay.left_poses[0][:3])
        self.assertEqual(overlay.right_positions, ((-0.10, 0.20, 0.30), (-0.11, 0.21, 0.31)))
        self.assertFalse(overlay.stale(now=104.9, threshold_sec=5.0))
        self.assertTrue(overlay.stale(now=105.1, threshold_sec=5.0))

    def test_snapshot_parse_retains_nonzero_waypoint_orientation(self) -> None:
        left = [
            [0.10, 0.20, 0.30, 0.01, -0.02, 0.03, 0.99, 100.0],
            [0.11, 0.21, 0.31, 0.04, -0.05, 0.06, 0.98, 90.0],
        ]
        overlay = ChunkOverlaySnapshot.parse(sample_chunk_overlay(left=left, right=None), received_monotonic=100.0)
        self.assertIsNotNone(overlay)
        assert overlay is not None
        self.assertEqual(overlay.left_poses[0], tuple(left[0][:7]))
        self.assertEqual(overlay.left_poses[1], tuple(left[1][:7]))
        self.assertEqual(overlay.left_positions[0], overlay.left_poses[0][:3])
        self.assertEqual(overlay.left_positions[1], overlay.left_poses[1][:3])

    def test_cursor_position_interpolates_by_gui_local_elapsed_time(self) -> None:
        overlay = ChunkOverlaySnapshot.parse(
            sample_chunk_overlay(
                horizon=3,
                left=[
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 100.0],
                    [0.10, 0.20, 0.30, 0.0, 0.0, 0.0, 1.0, 90.0],
                    [0.20, 0.40, 0.60, 0.0, 0.0, 0.0, 1.0, 80.0],
                ],
                right=None,
                policy_dt_sec=0.10,
            ),
            received_monotonic=100.0,
        )
        self.assertIsNotNone(overlay)
        assert overlay is not None
        self.assertEqual(overlay.cursor_position("left", now=100.0), (0.0, 0.0, 0.0))
        self.assertEqual(overlay.cursor_position("left", now=100.0 + overlay.policy_dt_sec * overlay.horizon), (0.20, 0.40, 0.60))
        midpoint = overlay.cursor_position("left", now=100.05)
        self.assertIsNotNone(midpoint)
        assert midpoint is not None
        self.assertAlmostEqual(midpoint[0], 0.05)
        self.assertAlmostEqual(midpoint[1], 0.10)
        self.assertAlmostEqual(midpoint[2], 0.15)

    def test_cursor_pose_uses_interpolated_position_and_floor_step_orientation(self) -> None:
        overlay = ChunkOverlaySnapshot.parse(
            sample_chunk_overlay(
                horizon=3,
                left=[
                    [0.0, 0.0, 0.0, 0.10, 0.20, 0.30, 0.90, 100.0],
                    [0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 90.0],
                    [0.20, 0.40, 0.60, 0.70, 0.80, 0.90, 0.10, 80.0],
                ],
                right=None,
                policy_dt_sec=0.10,
            ),
            received_monotonic=100.0,
        )
        self.assertIsNotNone(overlay)
        assert overlay is not None
        cursor_position = overlay.cursor_position("left", now=100.15)
        cursor_pose = overlay.cursor_pose("left", now=100.15)
        self.assertIsNotNone(cursor_position)
        self.assertIsNotNone(cursor_pose)
        assert cursor_position is not None
        assert cursor_pose is not None
        self.assertEqual(cursor_pose[:3], cursor_position)
        self.assertEqual(cursor_pose[3:], (0.40, 0.50, 0.60, 0.70))

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

    def test_store_history_returns_past_chunks_oldest_to_newest_without_duplicates(self) -> None:
        store = ChunkOverlayStore(stale_after_sec=5.0)
        for seq in range(1, 5):
            self.assertTrue(
                store.update_from_json_bytes(
                    json.dumps(sample_chunk_overlay(seq=seq)).encode(),
                    received_monotonic=float(seq),
                )
            )
        latest = store.latest()
        self.assertIsNotNone(latest)
        assert latest is not None
        self.assertEqual(latest.seq, 4)
        self.assertEqual([snapshot.seq for snapshot in store.history(2)], [2, 3])
        self.assertEqual(store.history(0), [])

        self.assertTrue(
            store.update_from_json_bytes(
                json.dumps(sample_chunk_overlay(seq=4)).encode(),
                received_monotonic=4.5,
            )
        )
        self.assertEqual([snapshot.seq for snapshot in store.history(10)], [1, 2, 3])

    def test_gui_chunk_overlay_retention_uses_operator_persist_seconds(self) -> None:
        from rb_servo_gui import app as gui_app

        overlay = ChunkOverlaySnapshot.parse(sample_chunk_overlay(), received_monotonic=90.0)
        self.assertIsNotNone(overlay)
        assert overlay is not None
        self.assertFalse(overlay.stale(now=100.0, threshold_sec=30.0))
        self.assertTrue(overlay.stale(now=100.0, threshold_sec=5.0))

        store = ChunkOverlayStore(stale_after_sec=5.0)
        self.assertTrue(
            store.update_from_json_bytes(
                json.dumps(sample_chunk_overlay()).encode(),
                received_monotonic=90.0,
            )
        )
        scene_handles = recording_chunk_overlay_handles()
        handles = {
            "scene": scene_handles,
            "chunk_overlay_visible": True,
            "chunk_overlay_dot_size": 0.022,
            "chunk_overlay_axes_visible": False,
            "chunk_overlay_axes_stride": 1,
            "chunk_overlay_persist_sec": 30.0,
        }
        with mock.patch("rb_servo_gui.app.time.monotonic", return_value=100.0):
            gui_app._update_chunk_overlay_gui(handles, store, latest=None)
        self.assertTrue(scene_handles["left_chunk_overlay"].visible)
        self.assertTrue(scene_handles["right_chunk_overlay"].visible)

        handles["chunk_overlay_persist_sec"] = 5.0
        with mock.patch("rb_servo_gui.app.time.monotonic", return_value=100.0):
            gui_app._update_chunk_overlay_gui(handles, store, latest=None)
        self.assertFalse(scene_handles["left_chunk_overlay"].visible)
        self.assertFalse(scene_handles["right_chunk_overlay"].visible)

    def test_chunk_overlay_persist_seconds_setting_round_trips(self) -> None:
        from rb_servo_gui import app as gui_app

        with tempfile.TemporaryDirectory() as tmp:
            os.environ["RB_GUI_SETTINGS_PATH"] = os.path.join(tmp, "settings.json")
            try:
                gui_app._update_gui_setting("chunk_overlay_persist_sec", 74.0)
                settings = gui_app._load_gui_settings()
                self.assertEqual(settings.get("chunk_overlay_persist_sec"), 74.0)
                self.assertEqual(
                    gui_app._gui_setting_float(settings, "chunk_overlay_persist_sec", 30.0),
                    74.0,
                )
            finally:
                os.environ.pop("RB_GUI_SETTINGS_PATH", None)

    def test_chunk_overlay_history_count_setting_round_trips(self) -> None:
        from rb_servo_gui import app as gui_app

        with tempfile.TemporaryDirectory() as tmp:
            os.environ["RB_GUI_SETTINGS_PATH"] = os.path.join(tmp, "settings.json")
            try:
                gui_app._update_gui_setting("chunk_overlay_history_count", 0)
                settings = gui_app._load_gui_settings()
                self.assertEqual(settings.get("chunk_overlay_history_count"), 0)
                self.assertEqual(
                    gui_app._gui_setting_int(settings, "chunk_overlay_history_count", 12),
                    0,
                )
            finally:
                os.environ.pop("RB_GUI_SETTINGS_PATH", None)

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

    def test_scene_update_returns_tracking_error_and_hides_chunk_handles(self) -> None:
        overlay = ChunkOverlaySnapshot.parse(sample_chunk_overlay(), received_monotonic=10.0)
        self.assertIsNotNone(overlay)
        assert overlay is not None
        handles = recording_chunk_overlay_handles()
        cursor = overlay.cursor_position("left", now=10.025)
        self.assertIsNotNone(cursor)
        assert cursor is not None
        actual_left = (cursor[0], cursor[1], cursor[2] + 0.020)

        errors = update_chunk_overlay(
            handles,
            overlay,
            stale=False,
            visible=True,
            now_monotonic=10.025,
            actual_positions={"left": actual_left, "right": None},
        )

        expected_error = math.sqrt(sum((cursor[index] - actual_left[index]) ** 2 for index in range(3)))
        self.assertAlmostEqual(errors["left"], expected_error)
        self.assertIsNone(errors["right"])
        self.assertTrue(handles["left_chunk_overlay_error"].visible)
        self.assertTrue(handles["left_chunk_overlay_points"].visible)

        hidden_errors = update_chunk_overlay(
            handles,
            overlay,
            stale=True,
            visible=True,
            now_monotonic=10.025,
            actual_positions={"left": actual_left, "right": None},
        )
        self.assertEqual(hidden_errors, {"left": None, "right": None})
        for key, handle in handles.items():
            if key == "chunk_overlay_line_mode":
                continue
            self.assertFalse(handle.visible, key)

    def test_scene_update_applies_live_dot_size_to_point_handles(self) -> None:
        overlay = ChunkOverlaySnapshot.parse(sample_chunk_overlay(), received_monotonic=10.0)
        self.assertIsNotNone(overlay)
        assert overlay is not None
        handles = recording_chunk_overlay_handles()

        update_chunk_overlay(handles, overlay, stale=False, visible=True, dot_size=0.003)

        self.assertEqual(handles["left_chunk_overlay_points"].point_size, 0.003)
        self.assertEqual(handles["right_chunk_overlay_points"].point_size, 0.003)

    def test_scene_update_renders_and_hides_past_chunk_history_lines(self) -> None:
        past = [
            ChunkOverlaySnapshot.parse(sample_chunk_overlay(seq=1, left=[
                [0.00, 0.00, 0.10, 0.0, 0.0, 0.0, 1.0, 100.0],
                [0.01, 0.00, 0.10, 0.0, 0.0, 0.0, 1.0, 90.0],
            ], right=None)),
            ChunkOverlaySnapshot.parse(sample_chunk_overlay(seq=2, left=[
                [0.02, 0.00, 0.10, 0.0, 0.0, 0.0, 1.0, 100.0],
                [0.03, 0.00, 0.10, 0.0, 0.0, 0.0, 1.0, 90.0],
            ], right=None)),
        ]
        self.assertTrue(all(snapshot is not None for snapshot in past))
        history = [snapshot for snapshot in past if snapshot is not None]
        overlay = ChunkOverlaySnapshot.parse(sample_chunk_overlay(seq=3), received_monotonic=10.0)
        self.assertIsNotNone(overlay)
        assert overlay is not None
        handles = recording_chunk_overlay_handles()

        update_chunk_overlay(
            handles,
            overlay,
            stale=False,
            visible=True,
            history_overlays=history,
        )

        history_handle = handles["left_chunk_overlay_history"]
        self.assertTrue(history_handle.visible)
        self.assertEqual(history_handle.points.shape, (2, 2, 3))
        self.assertEqual(history_handle.colors.shape, (2, 2, 3))
        self.assertLess(int(history_handle.colors[0][0][0]), int(history_handle.colors[1][0][0]))
        self.assertTrue(handles["left_chunk_overlay"].visible)
        self.assertTrue(handles["left_chunk_overlay_points"].visible)

        update_chunk_overlay(
            handles,
            overlay,
            stale=False,
            visible=True,
            history_overlays=[],
        )
        self.assertFalse(history_handle.visible)
        self.assertTrue(handles["left_chunk_overlay"].visible)
        self.assertTrue(handles["left_chunk_overlay_points"].visible)

    def test_scene_update_scales_cursor_from_dot_size(self) -> None:
        overlay = ChunkOverlaySnapshot.parse(sample_chunk_overlay(), received_monotonic=10.0)
        self.assertIsNotNone(overlay)
        assert overlay is not None
        handles = recording_chunk_overlay_handles()

        update_chunk_overlay(
            handles,
            overlay,
            stale=False,
            visible=True,
            now_monotonic=10.0,
            dot_size=0.002,
        )
        self.assertEqual(handles["left_chunk_overlay_cursor"].scale, 0.3)
        self.assertEqual(handles["right_chunk_overlay_cursor"].scale, 0.3)
        self.assertEqual(handles["left_chunk_overlay_cursor"].point_size, 0.006)

        update_chunk_overlay(
            handles,
            overlay,
            stale=False,
            visible=True,
            now_monotonic=10.0,
            dot_size=0.022,
        )
        self.assertAlmostEqual(handles["left_chunk_overlay_cursor"].scale, 1.0)
        self.assertAlmostEqual(handles["right_chunk_overlay_cursor"].scale, 1.0)
        self.assertAlmostEqual(handles["left_chunk_overlay_cursor"].point_size, 0.0396)

        update_chunk_overlay(
            handles,
            overlay,
            stale=False,
            visible=True,
            now_monotonic=10.0,
            dot_size=0.08,
        )
        self.assertEqual(handles["left_chunk_overlay_cursor"].scale, 2.0)
        self.assertEqual(handles["left_chunk_overlay_cursor"].point_size, 0.05)

    def test_pose_triad_segments_zero_rotation_aligns_with_world_axes(self) -> None:
        segments, colors = _pose_triad_segments((1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0), 0.05)
        self.assertEqual(segments.shape, (3, 2, 3))
        self.assertEqual(colors.shape, (3, 2, 3))
        expected_directions = ((0.05, 0.0, 0.0), (0.0, 0.05, 0.0), (0.0, 0.0, 0.05))
        for index, expected in enumerate(expected_directions):
            direction = segments[index][1] - segments[index][0]
            for actual_value, expected_value in zip(direction, expected, strict=True):
                self.assertAlmostEqual(float(actual_value), expected_value)

    def test_pose_triad_segments_quaternion_z90_and_sign_flip_match(self) -> None:
        s = math.sqrt(0.5)
        segments, _colors = _pose_triad_segments((0.0, 0.0, 0.0, 0.0, 0.0, s, s), 0.05)
        flipped, _flipped_colors = _pose_triad_segments((0.0, 0.0, 0.0, -0.0, -0.0, -s, -s), 0.05)
        expected_directions = ((0.0, 0.05, 0.0), (-0.05, 0.0, 0.0), (0.0, 0.0, 0.05))
        for index, expected in enumerate(expected_directions):
            direction = segments[index][1] - segments[index][0]
            flipped_direction = flipped[index][1] - flipped[index][0]
            for actual_value, expected_value in zip(direction, expected, strict=True):
                self.assertAlmostEqual(float(actual_value), expected_value, places=6)
            for actual_value, expected_value in zip(flipped_direction, expected, strict=True):
                self.assertAlmostEqual(float(actual_value), expected_value, places=6)

    def test_scene_update_renders_strided_pose_triads_and_hides_when_disabled(self) -> None:
        overlay = ChunkOverlaySnapshot.parse(
            sample_chunk_overlay(
                horizon=5,
                left=[
                    [0.00, 0.00, 0.00, 0.0, 0.0, 0.0, 1.0, 100.0],
                    [0.01, 0.00, 0.00, 0.1, 0.0, 0.0, 0.99, 90.0],
                    [0.02, 0.00, 0.00, 0.2, 0.0, 0.0, 0.98, 80.0],
                    [0.03, 0.00, 0.00, 0.3, 0.0, 0.0, 0.95, 70.0],
                    [0.04, 0.00, 0.00, 0.4, 0.0, 0.0, 0.90, 60.0],
                ],
                right=None,
                policy_dt_sec=0.10,
            ),
            received_monotonic=10.0,
        )
        self.assertIsNotNone(overlay)
        assert overlay is not None
        handles = recording_chunk_overlay_handles()

        update_chunk_overlay(
            handles,
            overlay,
            stale=False,
            visible=True,
            now_monotonic=10.05,
            show_axes=True,
            axes_stride=2,
        )

        axes = handles["left_chunk_overlay_axes"]
        cursor_axes = handles["left_chunk_overlay_cursor_axes"]
        self.assertTrue(axes.visible)
        self.assertEqual(axes.points.shape, (9, 2, 3))
        self.assertEqual(axes.colors.shape, (9, 2, 3))
        self.assertTrue(cursor_axes.visible)
        self.assertEqual(cursor_axes.points.shape, (3, 2, 3))

        update_chunk_overlay(
            handles,
            overlay,
            stale=False,
            visible=True,
            now_monotonic=10.05,
            show_axes=False,
            axes_stride=2,
        )

        self.assertFalse(handles["left_chunk_overlay_axes"].visible)
        self.assertFalse(handles["left_chunk_overlay_cursor_axes"].visible)


if __name__ == "__main__":
    unittest.main()
