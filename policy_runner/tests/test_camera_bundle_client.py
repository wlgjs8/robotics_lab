from __future__ import annotations

import json
import os
import socket
import tempfile
import threading
import time
import unittest
import uuid
from pathlib import Path
from unittest import mock

from policy_runner.camera_bundle_client import (
    _SLOT_HEADER,
    CameraBundle,
    CameraBundleClient,
    CameraFrame,
    bundle_clock_ns,
    resolve_frame,
)

try:
    import numpy as np
    import zmq
except ModuleNotFoundError:
    np = None
    zmq = None


def _free_tcp_endpoint() -> str:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return f"tcp://127.0.0.1:{sock.getsockname()[1]}"


def _local_tcp_socket_available() -> bool:
    try:
        _free_tcp_endpoint()
    except OSError:
        return False
    return True


def _write_shm_image(
    *,
    name: str,
    width: int,
    height: int,
    channels: int,
    payload: bytes,
    stride_bytes: int | None = None,
    start_tag: int = 2,
    end_tag: int = 2,
    valid_flag: int = 1,
) -> dict:
    stride = stride_bytes if stride_bytes is not None else width * channels
    path = Path("/dev/shm") / name.lstrip("/")
    size = _SLOT_HEADER.size + len(payload)
    with path.open("wb") as handle:
        handle.truncate(size)
        header = _SLOT_HEADER.pack(
            start_tag,
            end_tag,
            11,
            1234,
            1200,
            0,
            width,
            height,
            stride,
            len(payload),
            valid_flag,
            0,
        )
        handle.seek(0)
        handle.write(header)
        handle.write(payload)
    return {
        "camera_name": "head",
        "stream": "color",
        "frame_number": 11,
        "host_arrival_time_ns": 1234,
        "sensor_timestamp_ns": 1200,
        "width": width,
        "height": height,
        "stride_bytes": stride,
        "format": "rgb8",
        "shm_name": name,
        "ring_name": "head.color",
        "slot_index": 0,
        "shm_offset": _SLOT_HEADER.size,
        "size_bytes": len(payload),
        "seq": 11,
        "valid": True,
    }


@unittest.skipIf(zmq is None or np is None, "camera extras not installed")
@unittest.skipIf(not _local_tcp_socket_available(), "local TCP sockets unavailable")
class CameraBundleClientTest(unittest.TestCase):
    def setUp(self) -> None:
        self.ctx = zmq.Context.instance()
        self.pub = self.ctx.socket(zmq.PUB)
        self.endpoint = _free_tcp_endpoint()
        self.pub.bind(self.endpoint)
        self.client = CameraBundleClient(self.endpoint, max_age_ms=100.0)
        self.shm_names: list[str] = []
        time.sleep(0.08)

    def tearDown(self) -> None:
        self.client.close()
        self.pub.close(linger=0)
        for name in self.shm_names:
            try:
                os.unlink("/dev/shm/" + name.lstrip("/"))
            except FileNotFoundError:
                pass

    def _shm_name(self) -> str:
        name = f"policy_runner_test_{uuid.uuid4().hex}"
        self.shm_names.append(name)
        return name

    def _bundle(self, frames: dict, *, complete: bool = True, bundle_time_ns: int | None = None) -> dict:
        return {
            "schema": "camera_server.bundle.v1",
            "bundle_seq": 42,
            "bundle_time_ns": time.time_ns() if bundle_time_ns is None else bundle_time_ns,
            "hardware_synced": False,
            "sync_policy": "nearest_timestamp",
            "max_time_diff_ms": 1.0,
            "complete": complete,
            "frames": frames,
            "drop_counters": {},
        }

    def _publish(self, meta: dict) -> CameraBundle | None:
        payload = json.dumps(meta).encode("utf-8")
        for _ in range(5):
            self.pub.send_multipart([b"camera.bundle", payload])
            deadline = time.monotonic() + 0.2
            while time.monotonic() < deadline:
                bundle = self.client.poll(timeout_ms=20)
                if bundle is not None:
                    return bundle
        return None

    def test_poll_returns_none_when_no_bundle_published(self) -> None:
        self.assertIsNone(self.client.poll(timeout_ms=10))

    def test_poll_preserves_latest_camera_health_document(self) -> None:
        health = {"schema": "camera_server.health.v1", "status": "ok", "fps": 30.0}
        payload = json.dumps(health).encode("utf-8")
        for _ in range(5):
            self.pub.send_multipart([b"camera.health", payload])
            time.sleep(0.01)
            self.client.poll(timeout_ms=1)
        self.assertEqual(self.client.diagnostics_snapshot()["camera_health"], health)

    def test_poll_decodes_complete_bundle_with_one_camera(self) -> None:
        pixels = bytes(range(12))
        frame = _write_shm_image(name=self._shm_name(), width=2, height=2, channels=3, payload=pixels)
        bundle = self._publish(self._bundle({"head": frame}))
        self.assertIsNotNone(bundle)
        assert bundle is not None
        self.assertEqual(sorted(bundle.frames), ["head"])
        self.assertEqual(bundle.frames["head"].pixels.shape, (2, 2, 3))
        self.assertEqual(bundle.sync_policy, "nearest_timestamp")
        self.assertEqual(bundle.max_time_diff_ms, 1.0)
        self.assertEqual(bundle.drop_counters, {})
        diagnostics = self.client.diagnostics_snapshot()
        self.assertEqual(diagnostics["last_poll"]["outcome"], "ok")
        self.assertEqual(diagnostics["latest_bundle"]["bundle_seq"], 42)
        np.testing.assert_array_equal(bundle.frames["head"].pixels.reshape(-1), np.frombuffer(pixels, dtype=np.uint8))

    def test_legacy_topic_ignores_prefixed_group_messages_when_draining(self) -> None:
        frame = _write_shm_image(
            name=self._shm_name(), width=1, height=1, channels=3, payload=b"\x01\x02\x03"
        )
        payload = json.dumps(self._bundle({"head": frame})).encode("utf-8")
        for _ in range(5):
            self.pub.send_multipart([b"camera.bundle", payload])
            self.pub.send_multipart([b"camera.bundle.policy", b"{}"])
            bundle = self.client.poll(timeout_ms=50)
            if bundle is not None:
                break
        self.assertIsNotNone(bundle)
        assert bundle is not None
        self.assertEqual(bundle.bundle_seq, 42)

    def test_poll_skips_incomplete_bundles(self) -> None:
        frame = _write_shm_image(name=self._shm_name(), width=1, height=1, channels=3, payload=b"\x01\x02\x03")
        self.assertIsNone(self._publish(self._bundle({"head": frame}, complete=False)))
        self.assertGreater(
            self.client.diagnostics_snapshot()["poll_outcome_counts"].get("incomplete_bundle", 0),
            0,
        )

    def test_poll_skips_invalid_frames(self) -> None:
        frame = _write_shm_image(name=self._shm_name(), width=1, height=1, channels=3, payload=b"\x01\x02\x03")
        frame["valid"] = False
        self.assertIsNone(self._publish(self._bundle({"head": frame})))

    def test_bgr8_format_is_converted_to_rgb(self) -> None:
        frame = _write_shm_image(name=self._shm_name(), width=1, height=1, channels=3, payload=bytes([1, 2, 3]))
        frame["format"] = "bgr8"
        bundle = self._publish(self._bundle({"head": frame}))
        self.assertIsNotNone(bundle)
        assert bundle is not None
        np.testing.assert_array_equal(bundle.frames["head"].pixels[0, 0], [3, 2, 1])

    def test_seqlock_retry_succeeds_after_concurrent_write(self) -> None:
        name = self._shm_name()
        frame = _write_shm_image(
            name=name,
            width=1,
            height=1,
            channels=3,
            payload=b"\x04\x05\x06",
            start_tag=1,
            end_tag=1,
            valid_flag=0,
        )
        path = Path("/dev/shm") / name

        real_unpack = struct_unpack = __import__("struct").unpack_from
        calls = {"count": 0}

        def unpack_and_complete(fmt, buffer, offset=0):
            if fmt == "<Q" and offset == 0:
                calls["count"] += 1
                if calls["count"] == 1:
                    with path.open("r+b") as handle:
                        handle.seek(0)
                        handle.write(
                            _SLOT_HEADER.pack(2, 2, 11, 1234, 1200, 0, 1, 1, 3, 3, 1, 0)
                        )
                    return (1,)
            return real_unpack(fmt, buffer, offset)

        with mock.patch("policy_runner.camera_bundle_client.struct.unpack_from", side_effect=unpack_and_complete):
            frame_obj = self.client._decode_frame("head", frame)
        np.testing.assert_array_equal(frame_obj.pixels[0, 0], [4, 5, 6])
        self.assertGreaterEqual(calls["count"], 2)

    def _freshness_bundle(self, bundle_time_ns: int) -> CameraBundle:
        return CameraBundle(
            bundle_seq=1,
            bundle_time_ns=bundle_time_ns,
            hardware_synced=False,
            complete=True,
            received_monotonic=time.monotonic(),
            frames={
                "head": CameraFrame(
                    "head",
                    1,
                    1,
                    np.zeros((1, 1, 3), dtype=np.uint8),
                    "rgb8",
                    1,
                    1,
                    1,
                )
            },
        )

    def test_is_fresh_returns_false_for_old_bundle(self) -> None:
        # bundle_time_ns is stamped on the camera_server clock (monotonic_raw).
        self.client._latest = self._freshness_bundle(bundle_clock_ns() - 1_000_000_000)
        self.assertFalse(self.client.is_fresh())

    def test_is_fresh_returns_true_for_recent_bundle(self) -> None:
        self.client._latest = self._freshness_bundle(bundle_clock_ns())
        self.assertTrue(self.client.is_fresh())

    def test_is_fresh_rejects_wall_clock_stamps(self) -> None:
        # Epoch timestamps are not comparable to the monotonic_raw bundle clock.
        self.client._latest = self._freshness_bundle(time.time_ns())
        self.assertFalse(self.client.is_fresh())

    def test_close_releases_zmq_and_shm(self) -> None:
        self.client.close()
        self.assertIsNone(self.client.poll(timeout_ms=1))
        self.client.close()


class ResolveFrameTest(unittest.TestCase):
    def test_exact_key_match(self) -> None:
        frames = {"left_realsense.color": "frame"}
        self.assertEqual(resolve_frame(frames, "left_realsense.color"), "frame")

    def test_dataset_name_maps_to_bundle_key(self) -> None:
        frames = {"left_realsense.color": "frame"}
        self.assertEqual(resolve_frame(frames, "left_realsense_color"), "frame")

    def test_exact_key_wins_over_mapped_key(self) -> None:
        frames = {"left_realsense_color": "exact", "left_realsense.color": "mapped"}
        self.assertEqual(resolve_frame(frames, "left_realsense_color"), "exact")

    def test_dotted_name_is_not_remapped(self) -> None:
        frames = {"left.realsense.color": "frame"}
        self.assertIsNone(resolve_frame(frames, "left.realsense_color"))

    def test_missing_and_empty(self) -> None:
        self.assertIsNone(resolve_frame({}, "left_realsense_color"))
        self.assertIsNone(resolve_frame(None, "left_realsense_color"))
        self.assertIsNone(resolve_frame({"right_realsense.color": "x"}, "left_realsense_color"))
        self.assertIsNone(resolve_frame({"head.color": "x"}, "color"))


if __name__ == "__main__":
    unittest.main()
