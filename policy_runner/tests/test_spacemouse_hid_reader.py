from __future__ import annotations

import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from policy_runner.spacemouse import HidSpaceMouseReader


class _FakeDevice:
    def __init__(self, states=()):
        self._states = list(states)
        self.closed = False

    def read(self):
        if not self._states:
            return None
        return self._states.pop(0)

    def close(self) -> None:
        self.closed = True


def _fake_state(tx=0.0):
    return SimpleNamespace(
        tx=tx,
        ty=0.0,
        tz=0.0,
        rx=0.0,
        ry=0.0,
        rz=0.0,
        buttons=(True,),
    )


class HidSpaceMouseReaderTests(unittest.TestCase):
    def test_uses_open_by_path_when_hid_path_is_configured(self):
        calls = []
        device = _FakeDevice()
        fake_pyspacemouse = SimpleNamespace(
            open_by_path=lambda path: calls.append(("open_by_path", path)) or device,
            open=lambda **kwargs: calls.append(("open", kwargs)) or device,
        )

        with patch.dict(sys.modules, {"pyspacemouse": fake_pyspacemouse}):
            reader = HidSpaceMouseReader(path="/dev/hidraw1", device_number=1)
            reader.close()

        self.assertEqual(calls, [("open_by_path", "/dev/hidraw1")])
        self.assertTrue(device.closed)

    def test_uses_current_pyspacemouse_device_index_argument(self):
        calls = []
        device = _FakeDevice()
        fake_pyspacemouse = SimpleNamespace(
            open=lambda **kwargs: calls.append(kwargs) or device,
        )

        with patch.dict(sys.modules, {"pyspacemouse": fake_pyspacemouse}):
            reader = HidSpaceMouseReader(device="SpaceMouse", device_number=2)
            reader.close()

        self.assertEqual(calls, [{"device": "SpaceMouse", "device_index": 2}])

    def test_falls_back_to_legacy_device_number_argument(self):
        calls = []
        device = _FakeDevice()

        def open_device(**kwargs):
            calls.append(kwargs)
            if "device_index" in kwargs:
                raise TypeError("open() got an unexpected keyword argument 'device_index'")
            return device

        fake_pyspacemouse = SimpleNamespace(open=open_device)

        with patch.dict(sys.modules, {"pyspacemouse": fake_pyspacemouse}):
            reader = HidSpaceMouseReader(device_number=3)
            reader.close()

        self.assertEqual(
            calls,
            [
                {"device": None, "device_index": 3},
                {"device": None, "DeviceNumber": 3},
            ],
        )

    def _make_reader_with_states(self, states):
        device = _FakeDevice(states)
        fake_pyspacemouse = SimpleNamespace(open=lambda **_kwargs: device)

        with patch.dict(sys.modules, {"pyspacemouse": fake_pyspacemouse}):
            reader = HidSpaceMouseReader()
        return reader

    def test_read_drains_buffered_events_and_returns_latest(self):
        reader = self._make_reader_with_states(
            [
                _fake_state(tx=0.1),
                _fake_state(tx=0.5),
                _fake_state(tx=0.9),
                None,
            ]
        )

        sample = reader.read(timeout_sec=0.0)

        self.assertIsNotNone(sample)
        assert sample is not None
        self.assertAlmostEqual(sample.tx, 0.9)

    def test_read_returns_none_immediately_when_buffer_empty(self):
        reader = self._make_reader_with_states([None])

        sample = reader.read(timeout_sec=0.0)

        self.assertIsNone(sample)

    def test_read_with_positive_timeout_waits_then_returns_latest_of_burst(self):
        reader = self._make_reader_with_states(
            [
                None,
                _fake_state(tx=0.2),
                _fake_state(tx=0.8),
                None,
            ]
        )

        sample = reader.read(timeout_sec=0.05)

        self.assertIsNotNone(sample)
        assert sample is not None
        self.assertAlmostEqual(sample.tx, 0.8)

    def test_drain_terminates_when_device_returns_none(self):
        reader = self._make_reader_with_states([None, None, None])

        sample = reader.read(timeout_sec=0.0)

        self.assertIsNone(sample)

    def test_drain_is_bounded_when_device_keeps_returning_states(self):
        states = [_fake_state(tx=float(index)) for index in range(100)]
        reader = self._make_reader_with_states(states)

        sample = reader.read(timeout_sec=0.0)

        self.assertIsNotNone(sample)
        assert sample is not None
        self.assertEqual(sample.tx, 63.0)


if __name__ == "__main__":
    unittest.main()
