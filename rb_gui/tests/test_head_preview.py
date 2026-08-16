from __future__ import annotations

import unittest

import numpy as np

from rb_servo_gui.app import _update_head_preview
from rb_servo_gui.head_preview import (
    DEFAULT_HEAD_STREAM,
    DEFAULT_HEAD_TOPIC,
    HeadPreviewStore,
    _decimate,
    _head_frame_meta,
)


def _bundle(
    *,
    stream: str = DEFAULT_HEAD_STREAM,
    complete: bool = True,
    valid: bool = True,
    schema: str = "camera_server.bundle.v1",
) -> dict:
    return {
        "schema": schema,
        "group_name": "stereo",
        "bundle_seq": 12,
        "complete": complete,
        "frames": {
            stream: {
                "camera_name": "head",
                "stream": "color",
                "width": 1280,
                "height": 720,
                "stride_bytes": 3840,
                "format": "rgb8",
                "shm_name": "/camera_server_frames",
                "shm_offset": 4096,
                "size_bytes": 1280 * 720 * 3,
                "valid": valid,
            }
        },
    }


class Handle:
    def __init__(self, *, value=None, visible=False) -> None:
        self.value = value
        self.visible = visible
        self.image = None


class HeadPreviewTest(unittest.TestCase):
    def test_decimate_shrinks_head_frame_towards_preview_width(self) -> None:
        pixels = np.zeros((720, 1280, 3), dtype=np.uint8)
        preview = _decimate(pixels, target_width=480)
        self.assertEqual(preview.shape, (240, 427, 3))
        self.assertTrue(preview.flags["C_CONTIGUOUS"])

    def test_decimate_keeps_small_frames_unscaled(self) -> None:
        pixels = np.zeros((240, 320, 3), dtype=np.uint8)
        self.assertEqual(_decimate(pixels, target_width=480).shape, (240, 320, 3))

    def test_frame_meta_requires_complete_valid_bundle_of_the_head_stream(self) -> None:
        self.assertIsNotNone(_head_frame_meta(_bundle(), DEFAULT_HEAD_STREAM))
        self.assertIsNone(_head_frame_meta(_bundle(complete=False), DEFAULT_HEAD_STREAM))
        self.assertIsNone(_head_frame_meta(_bundle(valid=False), DEFAULT_HEAD_STREAM))
        self.assertIsNone(_head_frame_meta(_bundle(schema="other.v1"), DEFAULT_HEAD_STREAM))
        # Wrist-only rig: the head stream is simply absent from the bundle.
        self.assertIsNone(
            _head_frame_meta(_bundle(stream="left_realsense.color"), DEFAULT_HEAD_STREAM)
        )

    def test_status_reports_waiting_before_any_frame(self) -> None:
        store = HeadPreviewStore()
        text = store.status_text(now=1.0)
        self.assertIn("waiting", text)
        self.assertIn(DEFAULT_HEAD_TOPIC, text)
        self.assertIn(DEFAULT_HEAD_STREAM, text)
        self.assertIsNone(store.preview())

    def test_status_reports_live_then_stale(self) -> None:
        store = HeadPreviewStore()
        store.update(
            np.zeros((240, 427, 3), dtype=np.uint8),
            source_size=(1280, 720),
            camera_name="head",
            bundle_seq=12,
            frame_age_ms=8.5,
            received_monotonic=100.0,
        )
        live = store.status_text(now=100.1)
        self.assertIn("live", live)
        self.assertIn("1280x720→427x240", live)
        self.assertIn("8.5 ms", live)
        self.assertIn("stale", store.status_text(now=102.0))

    def test_receiver_gate_follows_the_operator_toggle(self) -> None:
        store = HeadPreviewStore()
        self.assertFalse(store.enabled)
        store.set_enabled(True)
        self.assertTrue(store.enabled)

    def test_gui_update_keeps_head_view_off_until_operator_enables_it(self) -> None:
        store = HeadPreviewStore()
        store.update(
            np.zeros((240, 427, 3), dtype=np.uint8),
            source_size=(1280, 720),
            camera_name="head",
            bundle_seq=12,
            frame_age_ms=8.5,
            received_monotonic=100.0,
        )
        toggle = Handle(value=False)
        image_handle = Handle(visible=False)
        status_handle = Handle(value="")
        handles = {
            "head_preview_store": store,
            "head_preview_toggle": toggle,
            "head_preview_image": image_handle,
            "head_preview_status": status_handle,
            "head_preview_last_monotonic": float("-inf"),
        }

        _update_head_preview(handles)
        self.assertFalse(image_handle.visible)
        self.assertIsNone(image_handle.image)
        self.assertNotEqual(status_handle.value, "")

        toggle.value = True
        _update_head_preview(handles)
        self.assertTrue(image_handle.visible)
        self.assertIsNotNone(image_handle.image)

    def test_gui_update_is_a_noop_without_a_store(self) -> None:
        handles: dict = {"head_preview_image": Handle(visible=False)}
        _update_head_preview(handles)
        self.assertFalse(handles["head_preview_image"].visible)


if __name__ == "__main__":
    unittest.main()
