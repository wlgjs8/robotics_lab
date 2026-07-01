#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import json
import pathlib
import threading
import unittest
from collections import deque


ROOT = pathlib.Path(__file__).resolve().parents[1]


def _load_module(name: str, rel: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / rel)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


box_trigger_listener = _load_module(
    "stereo_worker_box_trigger_listener", "stereo_worker/box_trigger_listener.py"
)


def _json_bytes(value):
    return json.dumps(value).encode("utf-8")


class BoxTriggerParseTest(unittest.TestCase):
    def test_valid_payload_with_explicit_labels(self):
        payload = {
            "schema": box_trigger_listener.TRIGGER_SCHEMA,
            "seq": 42,
            "command": "detect_now",
            "labels": ["green"],
        }

        parsed = box_trigger_listener._parse_trigger(_json_bytes(payload))

        self.assertEqual(
            parsed,
            {"seq": 42, "command": "detect_now", "labels": ["green"]},
        )

    def test_valid_payload_without_labels_defaults_to_all_known_labels(self):
        payload = {
            "schema": box_trigger_listener.TRIGGER_SCHEMA,
            "seq": "abc",
            "command": "detect_now",
        }

        parsed = box_trigger_listener._parse_trigger(_json_bytes(payload))

        self.assertEqual(
            parsed,
            {"seq": "abc", "command": "detect_now", "labels": ["green", "gray"]},
        )

    def test_rejects_invalid_payloads_fail_closed(self):
        base = {
            "schema": box_trigger_listener.TRIGGER_SCHEMA,
            "command": "detect_now",
        }
        cases = [
            ("malformed_json", b"{"),
            ("non_dict_list", _json_bytes(["detect_now"])),
            ("non_dict_number", _json_bytes(7)),
            ("missing_schema", _json_bytes({"command": "detect_now"})),
            ("wrong_schema", _json_bytes({**base, "schema": "wrong.schema"})),
            ("missing_command", _json_bytes({"schema": box_trigger_listener.TRIGGER_SCHEMA})),
            ("wrong_command", _json_bytes({**base, "command": "detect_later"})),
            ("labels_not_list", _json_bytes({**base, "labels": "green"})),
            ("labels_empty", _json_bytes({**base, "labels": []})),
            ("labels_unknown_rejects_whole_payload", _json_bytes({**base, "labels": ["green", "blue"]})),
        ]

        for name, data in cases:
            with self.subTest(name=name):
                self.assertIsNone(box_trigger_listener._parse_trigger(data))


class BoxTriggerDrainTest(unittest.TestCase):
    """소켓/스레드 없이(object.__new__) 큐 drain 동작만 검증."""

    def _listener(self, maxlen=8):
        listener = object.__new__(box_trigger_listener.BoxTriggerListener)
        listener._known_labels = box_trigger_listener.KNOWN_LABELS
        listener._lock = threading.Lock()
        listener._queue = deque(maxlen=maxlen)
        listener._rx = 0
        return listener

    def test_drain_returns_all_queued_commands_fifo_and_clears(self):
        listener = self._listener()
        commands = [
            {"seq": 1, "command": "detect_now", "labels": ["green"]},
            {"seq": 2, "command": "detect_now", "labels": ["gray"]},
            {"seq": 3, "command": "detect_now", "labels": ["green", "gray"]},
        ]
        for command in commands:
            listener._queue.append(command)

        self.assertEqual(listener.drain(), commands)
        self.assertEqual(listener.drain(), [])

    def test_queue_respects_maxlen(self):
        listener = self._listener(maxlen=3)
        for seq in range(5):
            listener._queue.append({"seq": seq, "command": "detect_now", "labels": ["green"]})

        drained = listener.drain()

        self.assertEqual([command["seq"] for command in drained], [2, 3, 4])


if __name__ == "__main__":
    unittest.main()
