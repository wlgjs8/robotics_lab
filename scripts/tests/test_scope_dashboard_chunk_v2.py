from __future__ import annotations

import json
import math
import sys
import unittest
from pathlib import Path
from unittest import mock

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_DIR = REPO_ROOT / "scripts"
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(SCRIPTS_DIR))

import servo_scope_dashboard as dashboard
from policy_runner.chunk_overlay_publisher import CHUNK_OVERLAY_SCHEMA_VERSION, ChunkOverlayPublisher
from rb_gui.rb_servo_gui.models import ChunkOverlaySnapshot


class RecordingSocket:
    def __init__(self) -> None:
        self.sent: list[tuple[bytes, tuple[str, int]]] = []

    def setblocking(self, _value: bool) -> None:
        return

    def sendto(self, data: bytes, address: tuple[str, int]) -> int:
        self.sent.append((data, address))
        return len(data)

    def close(self) -> None:
        return


def build_v2_packet_bytes(
    *,
    seq: int = 9,
    left: list[list[float]] | None = None,
    right: list[list[float]] | None = None,
    policy_dt_sec: float = 0.05,
) -> tuple[bytes, list[list[float]]]:
    if left is None:
        left = [
            [0.10, 0.20, 0.30, 0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5), 87.0],
            [0.11, 0.21, 0.31, 0.1, 0.2, 0.3, 0.9, 86.0],
        ]
    sock = RecordingSocket()
    publisher = ChunkOverlayPublisher("udp://127.0.0.1:50263", socket_factory=lambda *_args: sock)
    try:
        publisher.publish(
            seq=seq,
            policy_dt_sec=policy_dt_sec,
            left=left,
            right=right,
            host_time_ns=123456789,
        )
    finally:
        publisher.close()
    if not sock.sent:
        raise AssertionError("publisher did not send a packet")
    return sock.sent[0][0], left


class ScopeDashboardChunkV2Test(unittest.TestCase):
    def test_publisher_packet_round_trips_through_dashboard_and_gui_models(self) -> None:
        data, left = build_v2_packet_bytes()
        decoded = json.loads(data.decode("utf-8"))
        self.assertEqual(decoded["schema_version"], CHUNK_OVERLAY_SCHEMA_VERSION)
        self.assertEqual(decoded["schema_version"], "robotics_lab.chunk_overlay.v2")

        store = dashboard.TrajectoryStore()
        with mock.patch("servo_scope_dashboard.time.monotonic", return_value=10.0):
            self.assertTrue(store.update_predicted_from_json_bytes(data))
        payload = store.snapshot_payload()
        self.assertEqual(payload["schema"], "robotics_lab.scope_traj.v2")
        self.assertEqual(len(payload["chunks"]), 1)
        predicted = payload["chunks"][0]["arms"]["left"]["predicted"]
        self.assertEqual(predicted[0], left[0][:7])
        self.assertEqual(predicted[1], left[1][:7])

        snapshot = ChunkOverlaySnapshot.parse(decoded, received_monotonic=10.0)
        self.assertIsNotNone(snapshot)
        assert snapshot is not None
        self.assertEqual(snapshot.left_poses[0], tuple(left[0][:7]))
        self.assertEqual(snapshot.left_poses[1], tuple(left[1][:7]))

        state = {
            "left": {
                "tcp_actual_stand": {
                    "x": 0.10,
                    "y": 0.20,
                    "z": 0.30,
                    "quaternion_xyzw": left[0][3:7],
                }
            }
        }
        with mock.patch("servo_scope_dashboard.time.monotonic", return_value=10.0):
            store.update_actual_from_state_payload(state)
        actual = store.snapshot_payload()["chunks"][0]["arms"]["left"]["actual"]
        self.assertEqual(actual[0], left[0][:7])

        flat_state = {
            "left": {
                "tcp_actual_stand": {
                    "x": left[1][0],
                    "y": left[1][1],
                    "z": left[1][2],
                    "qx": left[1][3],
                    "qy": left[1][4],
                    "qz": left[1][5],
                    "qw": left[1][6],
                }
            }
        }
        with mock.patch("servo_scope_dashboard.time.monotonic", return_value=10.05):
            store.update_actual_from_state_payload(flat_state)
        actual = store.snapshot_payload()["chunks"][0]["arms"]["left"]["actual"]
        self.assertEqual(actual[1], left[1][:7])

    def test_distinct_seq_packets_are_buffered_oldest_to_newest(self) -> None:
        first_left = [
            [0.10, 0.20, 0.30, 0.0, 0.0, 0.0, 1.0, 80.0],
            [0.11, 0.21, 0.31, 0.0, 0.0, 0.1, 0.995, 81.0],
        ]
        second_left = [
            [0.40, 0.50, 0.60, 0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5), 82.0],
            [0.41, 0.51, 0.61, 0.1, 0.0, 0.2, 0.97, 83.0],
            [0.42, 0.52, 0.62, 0.2, 0.0, 0.3, 0.93, 84.0],
        ]
        first, _ = build_v2_packet_bytes(seq=41, left=first_left)
        second, _ = build_v2_packet_bytes(seq=42, left=second_left)

        store = dashboard.TrajectoryStore()
        with mock.patch("servo_scope_dashboard.time.monotonic", side_effect=[10.0, 11.0]):
            self.assertTrue(store.update_predicted_from_json_bytes(first))
            self.assertTrue(store.update_predicted_from_json_bytes(second))

        chunks = store.snapshot_payload()["chunks"]
        self.assertEqual([chunk["seq"] for chunk in chunks], [41, 42])
        self.assertEqual(chunks[0]["horizon"], 2)
        self.assertEqual(chunks[1]["horizon"], 3)
        self.assertEqual(chunks[0]["arms"]["left"]["predicted"][0], first_left[0][:7])
        self.assertEqual(chunks[1]["arms"]["left"]["predicted"][2], second_left[2][:7])

    def test_actual_samples_after_new_chunk_route_to_newest_chunk_only(self) -> None:
        first_left = [
            [0.10, 0.20, 0.30, 0.0, 0.0, 0.0, 1.0, 80.0],
            [0.11, 0.21, 0.31, 0.0, 0.0, 0.1, 0.995, 81.0],
        ]
        second_left = [
            [0.40, 0.50, 0.60, 0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5), 82.0],
            [0.41, 0.51, 0.61, 0.1, 0.0, 0.2, 0.97, 83.0],
        ]
        first, _ = build_v2_packet_bytes(seq=50, left=first_left)
        second, _ = build_v2_packet_bytes(seq=51, left=second_left)
        store = dashboard.TrajectoryStore()
        with mock.patch("servo_scope_dashboard.time.monotonic", side_effect=[10.0, 20.0]):
            self.assertTrue(store.update_predicted_from_json_bytes(first))
            self.assertTrue(store.update_predicted_from_json_bytes(second))

        state = {
            "left": {
                "tcp_actual_stand": {
                    "x": second_left[0][0],
                    "y": second_left[0][1],
                    "z": second_left[0][2],
                    "quaternion_xyzw": second_left[0][3:7],
                }
            }
        }
        with mock.patch("servo_scope_dashboard.time.monotonic", return_value=20.0):
            store.update_actual_from_state_payload(state)

        chunks = store.snapshot_payload()["chunks"]
        self.assertEqual(chunks[0]["arms"]["left"]["actual"], [None, None])
        self.assertEqual(chunks[1]["arms"]["left"]["actual"][0], second_left[0][:7])
        self.assertIsNone(chunks[1]["arms"]["left"]["actual"][1])

    def test_chunk_history_is_capped_at_newest_64_records(self) -> None:
        store = dashboard.TrajectoryStore()
        for seq in range(70):
            data, _ = build_v2_packet_bytes(seq=seq)
            with mock.patch("servo_scope_dashboard.time.monotonic", return_value=float(seq)):
                self.assertTrue(store.update_predicted_from_json_bytes(data))

        chunks = store.snapshot_payload()["chunks"]
        self.assertEqual(len(chunks), 64)
        self.assertEqual(chunks[0]["seq"], 6)
        self.assertEqual(chunks[-1]["seq"], 69)

    def test_quaternion_geodesic_math_is_sign_robust(self) -> None:
        identity = [0.0, 0.0, 0.0, 1.0]
        z90 = [0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)]
        self.assertAlmostEqual(dashboard._quat_geodesic_deg(identity, z90), 90.0)
        self.assertAlmostEqual(dashboard._quat_geodesic_deg(z90, [-item for item in z90]), 0.0)

    def test_v1_schema_packet_is_rejected_by_both_consumers(self) -> None:
        data, _left = build_v2_packet_bytes()
        decoded = json.loads(data.decode("utf-8"))
        legacy_schema = CHUNK_OVERLAY_SCHEMA_VERSION.replace(".v2", ".v1")
        decoded["schema"] = legacy_schema
        decoded["schema_version"] = legacy_schema
        v1_data = json.dumps(decoded).encode("utf-8")

        store = dashboard.TrajectoryStore()
        self.assertFalse(store.update_predicted_from_json_bytes(v1_data))
        self.assertIsNone(ChunkOverlaySnapshot.parse(decoded, received_monotonic=10.0))


if __name__ == "__main__":
    unittest.main()
