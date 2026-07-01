from __future__ import annotations

import importlib.util
import json
import pathlib
import unittest

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[1]


def _load_module(name: str, rel: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / rel)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


external_boxes_sender = _load_module(
    "external_boxes_sender", "stereo_worker/external_boxes_sender.py"
)
ExternalBoxesSender = external_boxes_sender.ExternalBoxesSender


class FakeSocket:
    def __init__(self):
        self.calls = []

    def sendto(self, data, address):
        self.calls.append((data, address))
        return len(data)

    def close(self):
        pass


class ExternalBoxesSenderTest(unittest.TestCase):
    def _sender(self, **kwargs):
        sender = ExternalBoxesSender(**kwargs)
        self.addCleanup(sender.close)
        return sender

    def test_build_payload_includes_detected_and_coasting_boxes(self):
        sender = self._sender(enabled=False)
        green_T = np.eye(4)
        green_T[:3, 3] = [0.12, -0.34, 0.56]
        gray_T = np.eye(4)
        gray_T[:3, 3] = [-0.20, -0.70, 0.055]

        payload = sender.build_payload([
            {"label": "green", "T": green_T},
            {"label": "gray", "T": gray_T, "coasting": True},
        ])

        self.assertEqual(payload["mode"], "SetExternalBoxes")
        boxes = {box["label"]: box for box in payload["boxes"]}
        self.assertEqual(set(boxes), {"green", "gray"})
        self.assertTrue(boxes["green"]["enable"])
        self.assertEqual(len(boxes["green"]["T"]), 16)
        self.assertEqual(boxes["green"]["T"][3], 0.12)
        self.assertEqual(boxes["green"]["T"][7], -0.34)
        self.assertEqual(boxes["green"]["T"][11], 0.56)
        self.assertTrue(boxes["gray"]["enable"])

    def test_absent_label_is_parked(self):
        sender = self._sender(enabled=False)
        green_T = np.eye(4)

        payload = sender.build_payload([{"label": "green", "T": green_T}])

        boxes = {box["label"]: box for box in payload["boxes"]}
        self.assertTrue(boxes["green"]["enable"])
        self.assertFalse(boxes["gray"]["enable"])
        self.assertEqual(boxes["gray"]["T"], np.eye(4).reshape(-1).astype(float).tolist())

    def test_send_uses_reusable_socket_and_disabled_sender_is_noop(self):
        sender = self._sender(endpoint="127.0.0.1:50123", source_id="test_worker")
        sender._sock.close()
        fake = FakeSocket()
        sender._sock = fake
        green_T = np.eye(4)
        gray_T = np.eye(4)

        self.assertTrue(sender.send([
            {"label": "green", "T": green_T},
            {"label": "gray", "T": gray_T},
        ]))

        self.assertEqual(len(fake.calls), 1)
        data, address = fake.calls[0]
        self.assertEqual(address, ("127.0.0.1", 50123))
        payload = json.loads(data.decode("utf-8"))
        self.assertEqual(payload["seq"], 1)
        self.assertEqual(payload["source_id"], "test_worker")
        self.assertEqual(payload["session_id"], sender.session_id)
        self.assertEqual(payload["mode"], "SetExternalBoxes")
        self.assertEqual({box["label"] for box in payload["boxes"]}, {"green", "gray"})

        disabled = self._sender(enabled=False)
        disabled._sock.close()
        disabled_fake = FakeSocket()
        disabled._sock = disabled_fake
        self.assertFalse(disabled.send([{"label": "green", "T": green_T}]))
        self.assertEqual(disabled_fake.calls, [])


if __name__ == "__main__":
    unittest.main()
