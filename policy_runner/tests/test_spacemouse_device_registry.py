from __future__ import annotations

import unittest

from policy_runner.spacemouse import SpaceMouseSample
from policy_runner.spacemouse_registry import (
    SPACEMOUSE_ASSIGNMENT_COMMAND_SCHEMA,
    RegistrySpaceMouseReader,
    SpaceMouseDeviceRegistry,
    SpaceMouseHidDescriptor,
)
from policy_runner.action_sources.dual_spacemouse_pose_target import DualSpaceMousePoseTargetActionSource


class _Reader:
    def __init__(self, samples=()):
        self.samples = list(samples)
        self.closed = False

    def read(self, timeout_sec=None):
        _ = timeout_sec
        if not self.samples:
            return None
        value = self.samples.pop(0)
        if isinstance(value, Exception):
            raise value
        return value

    def close(self):
        self.closed = True


def _descriptor(path: str, *, interface: int = 0, vendor: int = 0x256F):
    return SpaceMouseHidDescriptor(
        path=path,
        vendor_id=vendor,
        product_id=0xC652,
        interface_number=interface,
        product="3Dconnexion Universal Receiver",
        serial="",
    )


def _sample(tx: float):
    return SpaceMouseSample(tx, 0, 0, 0, 0, 0, (), 1.0)


class SpaceMouseDeviceRegistryTest(unittest.TestCase):
    def make_registry(self, descriptors, readers):
        return SpaceMouseDeviceRegistry(
            enumerate_fn=lambda: list(descriptors),
            reader_factory=lambda path: readers[path],
            autostart=False,
        )

    def test_filters_vendor_product_and_motion_interface(self):
        readers = {"a:1.0": _Reader(), "b:1.0": _Reader()}
        registry = self.make_registry(
            [
                _descriptor("a:1.0"),
                _descriptor("a:1.1", interface=1),
                _descriptor("b:1.0"),
                _descriptor("other:1.0", vendor=0x046D),
            ],
            readers,
        )
        registry.scan_once()
        status = registry.status_block(assignment_change_allowed=True)
        self.assertEqual(len(status["devices"]), 2)
        self.assertEqual({d["path"] for d in status["devices"]}, {"a:1.0", "b:1.0"})
        self.assertTrue(all(d["arm"] == "unassigned" for d in status["devices"]))
        registry.close()

    def test_atomic_assignment_requires_current_generation_and_unique_devices(self):
        readers = {"a": _Reader(), "b": _Reader()}
        registry = self.make_registry([_descriptor("a"), _descriptor("b")], readers)
        registry.scan_once()
        status = registry.status_block(assignment_change_allowed=True)
        ids = [device["connection_id"] for device in status["devices"]]
        command = {
            "schema": SPACEMOUSE_ASSIGNMENT_COMMAND_SCHEMA,
            "seq": 7,
            "status_generation": status["generation"],
            "command": "set",
            "left_connection_id": ids[0],
            "right_connection_id": ids[1],
        }
        self.assertTrue(registry.handle_control(command, assignment_change_allowed=True))
        accepted = registry.status_block(assignment_change_allowed=True)
        self.assertEqual(accepted["last_result"], "accepted")
        self.assertEqual(accepted["left_connection_id"], ids[0])
        self.assertEqual(accepted["right_connection_id"], ids[1])

        command["seq"] = 8
        self.assertTrue(registry.handle_control(command, assignment_change_allowed=True))
        self.assertEqual(
            registry.status_block(assignment_change_allowed=True)["last_error"],
            "stale_generation",
        )
        registry.close()

    def test_poll_exposes_activity_and_read_failure_unassigns(self):
        reader = _Reader([_sample(0.5), OSError("unplugged")])
        registry = self.make_registry([_descriptor("a")], {"a": reader})
        registry.scan_once()
        status = registry.status_block(assignment_change_allowed=True)
        connection_id = status["devices"][0]["connection_id"]
        registry.handle_control(
            {
                "schema": SPACEMOUSE_ASSIGNMENT_COMMAND_SCHEMA,
                "seq": 1,
                "status_generation": status["generation"],
                "command": "set",
                "left_connection_id": connection_id,
                "right_connection_id": None,
            },
            assignment_change_allowed=True,
        )
        registry.poll_once()
        device_status = registry.status_block(assignment_change_allowed=True)["devices"][0]
        self.assertAlmostEqual(device_status["activity"], 0.5)
        self.assertEqual(device_status["raw_axes"], [0.5, 0, 0, 0, 0, 0])
        self.assertTrue(device_status["has_sample"])
        self.assertIsInstance(device_status["sample_age_sec"], float)
        self.assertAlmostEqual(RegistrySpaceMouseReader(registry, "left").read().tx, 0.5)
        registry.poll_once()
        disconnected = registry.status_block(assignment_change_allowed=True)
        self.assertEqual(disconnected["devices"], [])
        self.assertIsNone(disconnected["left_connection_id"])
        self.assertTrue(reader.closed)
        registry.close()

    def test_active_owner_rejects_assignment(self):
        registry = self.make_registry([_descriptor("a")], {"a": _Reader()})
        registry.scan_once()
        status = registry.status_block(assignment_change_allowed=False)
        registry.handle_control(
            {
                "schema": SPACEMOUSE_ASSIGNMENT_COMMAND_SCHEMA,
                "seq": 2,
                "status_generation": status["generation"],
                "command": "set",
            },
            assignment_change_allowed=False,
        )
        self.assertEqual(
            registry.status_block(assignment_change_allowed=False)["last_error"],
            "active_owner",
        )
        registry.close()

    def test_accepted_assignment_emits_one_hold_before_motion_processing(self):
        registry = self.make_registry([_descriptor("a")], {"a": _Reader()})
        registry.scan_once()
        status = registry.status_block(assignment_change_allowed=True)
        source = DualSpaceMousePoseTargetActionSource(
            left_reader=RegistrySpaceMouseReader(registry, "left"),
            right_reader=RegistrySpaceMouseReader(registry, "right"),
            device_registry=registry,
        )
        handled = source.handle_spacemouse_control(
            {
                "schema": SPACEMOUSE_ASSIGNMENT_COMMAND_SCHEMA,
                "seq": 3,
                "status_generation": status["generation"],
                "command": "set",
                "left_connection_id": status["devices"][0]["connection_id"],
                "right_connection_id": None,
            }
        )
        self.assertTrue(handled)
        intent = source.next_intent(None, 0.0)  # Hold is emitted before snapshot access.
        self.assertEqual(intent.mode, "Hold")
        source.close()

    def test_source_status_exposes_input_policy_and_startup_gate(self):
        registry = self.make_registry([_descriptor("a")], {"a": _Reader([_sample(0.0)])})
        registry.scan_once()
        registry.poll_once()
        status = registry.status_block(assignment_change_allowed=True)
        source = DualSpaceMousePoseTargetActionSource(
            left_reader=RegistrySpaceMouseReader(registry, "left"),
            right_reader=RegistrySpaceMouseReader(registry, "right"),
            device_registry=registry,
            require_deadman=False,
            startup_requires_neutral=True,
            startup_neutral_hold_sec=0.3,
        )
        source.handle_spacemouse_control(
            {
                "schema": SPACEMOUSE_ASSIGNMENT_COMMAND_SCHEMA,
                "seq": 4,
                "status_generation": status["generation"],
                "command": "set",
                "left_connection_id": status["devices"][0]["connection_id"],
                "right_connection_id": None,
            }
        )
        diagnostics = source.spacemouse_status()
        self.assertFalse(diagnostics["input_policy"]["require_deadman"])
        self.assertTrue(diagnostics["input_policy"]["startup_requires_neutral"])
        self.assertEqual(diagnostics["side_gates"]["left"]["gate"], "startup_neutral")
        source.close()


if __name__ == "__main__":
    unittest.main()
