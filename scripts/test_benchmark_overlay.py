#!/usr/bin/env python3
"""Unit tests for telemetry-only benchmark overlay publishing."""

from __future__ import annotations

import json
import socket
import tempfile
import unittest
from pathlib import Path
from typing import Any

import benchmark_overlay as overlay


def loopback_udp_available() -> bool:
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    except OSError:
        return False
    sock.close()
    return True


def sample_message(**overrides: Any) -> dict[str, Any]:
    metrics = overlay.CircleOverlayMetrics(
        center=[0.0, 0.0, 0.0],
        axis1=[1.0, 0.0, 0.0],
        axis2=[0.0, 1.0, 0.0],
        radius_m=0.075,
        omega_rad_s=1.0,
    )
    metrics.observe(
        t_sec=0.0,
        desired_position=[0.075, 0.0, 0.0],
        actual_position=[0.074, 0.0, 0.0],
        sample_id=1,
    )
    values: dict[str, Any] = {
        "run_id": "unit-run",
        "arm": "left",
        "profile": "gene_15cm_4s",
        "controller": "twist_stand_feedback",
        "tracking_source": "tcp_ref_stand",
        "plane": "xy",
        "center_stand": [0.0, 0.0, 0.0],
        "axis1_stand": [1.0, 0.0, 0.0],
        "axis2_stand": [0.0, 1.0, 0.0],
        "radius_m": 0.075,
        "period_sec": 4.0,
        "repeat": 5,
        "phase_rad": 0.0,
        "desired_pose_stand": overlay.pose_payload([0.075, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        "metrics": metrics.snapshot(),
        "command_count": 10,
        "physical_motion_expected": False,
        "host_time_ns": 123,
    }
    values.update(overrides)
    return overlay.build_circle_overlay_message(**values)


def nested_keys(value: Any) -> set[str]:
    keys: set[str] = set()
    if isinstance(value, dict):
        for key, item in value.items():
            keys.add(key)
            keys.update(nested_keys(item))
    elif isinstance(value, list):
        for item in value:
            keys.update(nested_keys(item))
    return keys


class BenchmarkOverlayTest(unittest.TestCase):
    def test_schema_generation_contains_live_metrics(self) -> None:
        message = sample_message()
        self.assertEqual(message["schema_version"], overlay.SCHEMA_VERSION)
        self.assertEqual(message["tracking_source"], "tcp_ref_stand")
        self.assertEqual(message["radius_m"], 0.075)
        self.assertEqual(message["diameter_m"], 0.15)
        self.assertEqual(message["desired_pose_stand"]["quaternion_xyzw"], [0.0, 0.0, 0.0, 1.0])
        self.assertEqual(message["sample_count"], 1)
        self.assertAlmostEqual(message["current_error_m"], 0.001)
        self.assertFalse(message["physical_motion_expected"])

    @unittest.skipUnless(loopback_udp_available(), "loopback UDP sockets unavailable")
    def test_udp_receiver_gets_overlay_and_artifact_line(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text:
            tmp = Path(tmp_text)
            receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                receiver.bind(("127.0.0.1", 0))
                receiver.settimeout(1.0)
                port = receiver.getsockname()[1]
                publisher = overlay.BenchmarkOverlayPublisher(
                    endpoint=f"udp://127.0.0.1:{port}",
                    rate_hz=20.0,
                    run_id="unit-run",
                    artifact_path=tmp / "overlay_stream.jsonl",
                )
                try:
                    self.assertTrue(publisher.publish(sample_message(), force=True))
                    data, _addr = receiver.recvfrom(65535)
                finally:
                    publisher.close()
            finally:
                receiver.close()
            received = json.loads(data.decode("utf-8"))
            self.assertEqual(received["schema_version"], overlay.SCHEMA_VERSION)
            self.assertEqual(received["run_id"], "unit-run")
            lines = (tmp / "overlay_stream.jsonl").read_text(encoding="utf-8").splitlines()
            self.assertEqual(len(lines), 1)
            self.assertEqual(json.loads(lines[0])["run_id"], "unit-run")

    def test_overlay_disabled_does_not_send_or_write(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text:
            tmp = Path(tmp_text)
            publisher = overlay.BenchmarkOverlayPublisher(
                endpoint="udp://127.0.0.1:1",
                rate_hz=20.0,
                run_id="disabled-run",
                artifact_path=tmp / "overlay_stream.jsonl",
                enabled=False,
            )
            try:
                self.assertFalse(publisher.publish(sample_message(), force=True))
                summary = publisher.summary()
            finally:
                publisher.close()
            self.assertFalse((tmp / "overlay_stream.jsonl").exists())
            self.assertEqual(summary["overlay_messages_sent"], 0)
            self.assertIsNone(summary["overlay_pub_endpoint"])

    @unittest.skipUnless(loopback_udp_available(), "loopback UDP sockets unavailable")
    def test_rate_limiter_bounds_publish_rate(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text:
            tmp = Path(tmp_text)
            receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                receiver.bind(("127.0.0.1", 0))
                port = receiver.getsockname()[1]
                publisher = overlay.BenchmarkOverlayPublisher(
                    endpoint=f"udp://127.0.0.1:{port}",
                    rate_hz=10.0,
                    run_id="rate-run",
                    artifact_path=tmp / "overlay_stream.jsonl",
                )
                try:
                    self.assertTrue(publisher.publish(sample_message(host_time_ns=1), now_monotonic=100.0))
                    self.assertFalse(publisher.publish(sample_message(host_time_ns=2), now_monotonic=100.05))
                    self.assertTrue(publisher.publish(sample_message(host_time_ns=3), now_monotonic=100.11))
                    summary = publisher.summary()
                finally:
                    publisher.close()
            finally:
                receiver.close()
            self.assertEqual(summary["overlay_messages_sent"], 2)
            self.assertEqual(summary["overlay_messages_recorded"], 2)

    def test_no_command_fields_are_included(self) -> None:
        message = sample_message()
        self.assertFalse(nested_keys(message) & overlay.COMMAND_FIELD_NAMES)
        overlay.validate_no_command_fields(message)
        with self.assertRaisesRegex(ValueError, "command field"):
            overlay.validate_no_command_fields({"left": {"tcp_twist_stand": [0.0] * 6}})


if __name__ == "__main__":
    unittest.main()
