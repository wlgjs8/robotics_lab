from __future__ import annotations

import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from policy_runner.spacemouse import HidSpaceMouseReader


class _FakeDevice:
    def __init__(self):
        self.closed = False

    def close(self) -> None:
        self.closed = True


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


if __name__ == "__main__":
    unittest.main()
