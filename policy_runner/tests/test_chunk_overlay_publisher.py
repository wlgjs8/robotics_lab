from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from policy_runner.chunk_overlay_publisher import CHUNK_OVERLAY_SCHEMA_VERSION, ChunkOverlayPublisher


class FakeSendSocket:
    def __init__(self) -> None:
        self.sent = []
        self.blocking_values = []
        self.closed = False

    def setblocking(self, value: bool) -> None:
        self.blocking_values.append(value)

    def sendto(self, data, address):
        self.sent.append((data, address))
        return len(data)

    def close(self) -> None:
        self.closed = True


class ChunkOverlayPublisherTest(unittest.TestCase):
    def test_publish_serializes_documented_schema(self) -> None:
        sock = FakeSendSocket()
        publisher = ChunkOverlayPublisher(
            "udp://127.0.0.1:50262",
            socket_factory=lambda *_args: sock,
        )
        try:
            publisher.publish(
                seq=3,
                policy_dt_sec=0.5,
                left=[[1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.9, 99.0]],
                right=[[4.0, 5.0, 6.0, 0.4, 0.5, 0.6, 0.7, 88.0]],
                host_time_ns=123456789,
            )
        finally:
            publisher.close()

        self.assertEqual(sock.blocking_values, [False])
        self.assertTrue(sock.closed)
        self.assertEqual(len(sock.sent), 1)
        data, address = sock.sent[0]
        self.assertEqual(address, ("127.0.0.1", 50262))
        packet = json.loads(data.decode("utf-8"))
        self.assertEqual(packet["schema_version"], CHUNK_OVERLAY_SCHEMA_VERSION)
        self.assertEqual(packet["schema_version"], "robotics_lab.chunk_overlay.v2")
        self.assertEqual(packet["host_time_ns"], 123456789)
        self.assertEqual(packet["seq"], 3)
        self.assertEqual(packet["policy_dt_sec"], 0.5)
        self.assertEqual(packet["horizon"], 1)
        self.assertEqual(packet["left"], [[1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.9, 99.0]])
        self.assertEqual(packet["right"], [[4.0, 5.0, 6.0, 0.4, 0.5, 0.6, 0.7, 88.0]])
        self.assertNotIn("left_delta", packet)
        self.assertNotIn("right_delta", packet)

    def test_publish_serializes_optional_delta_rows(self) -> None:
        sock = FakeSendSocket()
        publisher = ChunkOverlayPublisher(
            "udp://127.0.0.1:50262",
            socket_factory=lambda *_args: sock,
        )
        try:
            publisher.publish(
                seq=4,
                policy_dt_sec=0.1,
                left=[[1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0, 50.0]],
                right=None,
                left_delta=[[0.01, 0.02, 0.03, 0.1, 0.2, 0.3, 50.0]],
                right_delta=None,
                host_time_ns=123,
            )
        finally:
            publisher.close()

        self.assertEqual(len(sock.sent), 1)
        packet = json.loads(sock.sent[0][0].decode("utf-8"))
        self.assertEqual(packet["schema_version"], "robotics_lab.chunk_overlay.v2")
        self.assertEqual(packet["left_delta"], [[0.01, 0.02, 0.03, 0.1, 0.2, 0.3, 50.0]])
        self.assertNotIn("right_delta", packet)


if __name__ == "__main__":
    unittest.main()
