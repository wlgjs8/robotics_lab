from __future__ import annotations

import socket
import tempfile
import unittest
from pathlib import Path

from policy_runner.pika_usb_pairing import (
    PikaUsbPairingError,
    load_expected_camera_serials,
    resolve_pika_usb_pairing,
    wait_for_camera_health,
)


LEFT_SERIAL = "412622272078"
RIGHT_SERIAL = "260322278348"


class _UsbFixture:
    def __init__(self, root: Path) -> None:
        self.root = root
        self.sysfs_usb = root / "sys/bus/usb/devices"
        self.sysfs_tty = root / "sys/class/tty"
        self.dev = root / "dev"
        self.by_path = self.dev / "serial/by-path"
        for path in (self.sysfs_usb, self.sysfs_tty, self.by_path):
            path.mkdir(parents=True)
        self._controllers: dict[str, Path] = {}

    def _controller(self, name: str) -> Path:
        if name not in self._controllers:
            path = self.root / "sys/devices" / name
            path.mkdir(parents=True)
            self._controllers[name] = path
        return self._controllers[name]

    def add_root(self, bus: int, controller: str = "controller0") -> Path:
        root = self._controller(controller) / f"usb{bus}"
        root.mkdir(parents=True, exist_ok=True)
        link = self.sysfs_usb / f"usb{bus}"
        if not link.exists():
            link.symlink_to(root)
        return root

    def _device_path(self, bus: int, node: str, controller: str) -> Path:
        root = self.add_root(bus, controller)
        route = node.split("-", 1)[1].split(".")
        current = root
        prefix = f"{bus}-{route[0]}"
        current = current / prefix
        current.mkdir(exist_ok=True)
        for part in route[1:]:
            prefix += f".{part}"
            current = current / prefix
            current.mkdir(exist_ok=True)
        link = self.sysfs_usb / node
        if not link.exists():
            link.symlink_to(current)
        return current

    def add_camera(
        self,
        *,
        bus: int,
        node: str,
        video: str,
        controller: str = "controller0",
    ) -> str:
        device = self._device_path(bus, node, controller)
        (device / "devpath").write_text(node.split("-", 1)[1])
        (device / "idVendor").write_text("8086")
        (device / "idProduct").write_text("0b5b")
        physical = device / f"{node}:1.0/video4linux/{video}"
        physical.mkdir(parents=True)
        return str(physical)

    def add_gripper(
        self,
        *,
        bus: int,
        node: str,
        tty: str,
        by_path_name: str,
        controller: str = "controller0",
        create_by_path: bool = True,
    ) -> None:
        device = self._device_path(bus, node, controller)
        (device / "devpath").write_text(node.split("-", 1)[1])
        (device / "idVendor").write_text("1a86")
        (device / "idProduct").write_text("7522")
        tty_device = device / f"{node}:1.0" / tty
        tty_device.mkdir(parents=True)
        tty_class = self.sysfs_tty / tty
        tty_class.mkdir()
        (tty_class / "device").symlink_to(tty_device)
        dev_tty = self.dev / tty
        dev_tty.touch()
        if create_by_path:
            (self.by_path / by_path_name).symlink_to(dev_tty)

    def kwargs(self) -> dict[str, Path]:
        return {
            "sysfs_usb_root": self.sysfs_usb,
            "sysfs_tty_root": self.sysfs_tty,
            "dev_root": self.dev,
            "dev_serial_by_path": self.by_path,
        }


def _health(left_port: str, right_port: str, *, status: str = "ok") -> dict:
    return {
        "schema": "camera_server.health.v1",
        "status": status,
        "cameras": {
            "left_realsense": {
                "serial": LEFT_SERIAL,
                "connected": True,
                "physical_port": left_port,
                "product_id": "0B5B",
                "usb_type": "3.2",
            },
            "right_realsense": {
                "serial": RIGHT_SERIAL,
                "connected": True,
                "physical_port": right_port,
                "product_id": "0B5B",
                "usb_type": "3.2",
            },
        },
    }


class PikaUsbPairingTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.fixture = _UsbFixture(Path(self.tmp.name))
        self.left_camera = self.fixture.add_camera(bus=4, node="4-1.2", video="video14")
        self.right_camera = self.fixture.add_camera(bus=4, node="4-2.2", video="video0")
        # Deliberately reverse tty numbering: identity must come from topology.
        self.fixture.add_gripper(
            bus=3,
            node="3-1.1.4",
            tty="ttyUSB9",
            by_path_name="pci-controller0-usb-0:1.1.4:1.0-port0",
        )
        self.fixture.add_gripper(
            bus=3,
            node="3-2.1.4",
            tty="ttyUSB0",
            by_path_name="pci-controller0-usb-0:2.1.4:1.0-port0",
        )
        self.expected = {"left": LEFT_SERIAL, "right": RIGHT_SERIAL}

    def resolve(self, health: dict | None = None):
        return resolve_pika_usb_pairing(
            health or _health(self.left_camera, self.right_camera),
            self.expected,
            **self.fixture.kwargs(),
        )

    def test_resolves_by_companion_root_not_tty_number(self) -> None:
        pairing = self.resolve()
        self.assertEqual(pairing.arms["left"].gripper_tty, "ttyUSB9")
        self.assertEqual(pairing.arms["right"].gripper_tty, "ttyUSB0")
        self.assertIn("1.1.4", pairing.ports["left"])
        self.assertIn("2.1.4", pairing.ports["right"])
        self.assertEqual(pairing.to_dict()["schema"], "robotics_lab.pika_usb_pairing.v1")

    def test_camera_degraded_status_does_not_hide_valid_identity(self) -> None:
        pairing = self.resolve(_health(self.left_camera, self.right_camera, status="degraded"))
        self.assertEqual(pairing.arms["left"].camera_serial, LEFT_SERIAL)

    def test_stale_video_leaf_requires_fresh_camera_health(self) -> None:
        old_port = Path(self.left_camera)
        old_port.rmdir()
        new_port = old_port.with_name("video32")
        new_port.mkdir()
        with self.assertRaisesRegex(PikaUsbPairingError, "stale path after USB reconnect"):
            self.resolve()
        pairing = self.resolve(_health(str(new_port), self.right_camera))
        self.assertEqual(pairing.arms["left"].gripper_tty, "ttyUSB9")
        self.assertEqual(pairing.arms["left"].camera_physical_port, str(new_port))

    def test_missing_camera_usb_node_is_refused(self) -> None:
        (self.fixture.sysfs_usb / "4-1.2").unlink()
        with self.assertRaisesRegex(PikaUsbPairingError, "USB device 4-1.2 is not present"):
            self.resolve()

    def test_physical_path_on_wrong_controller_is_refused(self) -> None:
        device = self.fixture.sysfs_usb / "4-1.2"
        wrong_device = self.fixture.root / "sys/devices/controller1/usb4/4-1/4-1.2"
        wrong_device.mkdir(parents=True)
        device.unlink()
        device.symlink_to(wrong_device)
        with self.assertRaisesRegex(PikaUsbPairingError, "does not belong to sysfs device"):
            self.resolve()

    def test_unrelated_ch340_on_another_controller_is_ignored(self) -> None:
        self.fixture.add_gripper(
            bus=7,
            node="7-1.4",
            tty="ttyUSB5",
            by_path_name="pci-controller1-usb-0:1.4:1.0-port0",
            controller="controller1",
        )
        pairing = self.resolve()
        self.assertNotEqual(pairing.arms["left"].gripper_tty, "ttyUSB5")

    def test_multiple_ch340_on_same_root_is_refused(self) -> None:
        self.fixture.add_gripper(
            bus=3,
            node="3-1.2",
            tty="ttyUSB7",
            by_path_name="pci-controller0-usb-0:1.2:1.0-port0",
        )
        with self.assertRaisesRegex(PikaUsbPairingError, "exactly one CH340"):
            self.resolve()

    def test_missing_stable_by_path_is_refused(self) -> None:
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        fixture = _UsbFixture(Path(tmp.name))
        left_camera = fixture.add_camera(bus=4, node="4-1.2", video="video14")
        right_camera = fixture.add_camera(bus=4, node="4-2.2", video="video0")
        fixture.add_gripper(
            bus=3,
            node="3-1.1.4",
            tty="ttyUSB1",
            by_path_name="unused-left",
            create_by_path=False,
        )
        fixture.add_gripper(
            bus=3,
            node="3-2.1.4",
            tty="ttyUSB0",
            by_path_name="right",
        )
        with self.assertRaisesRegex(PikaUsbPairingError, "exactly one /dev/serial/by-path"):
            resolve_pika_usb_pairing(
                _health(left_camera, right_camera),
                self.expected,
                **fixture.kwargs(),
            )

    def test_camera_identity_failures_are_refused(self) -> None:
        cases = []
        disconnected = _health(self.left_camera, self.right_camera)
        disconnected["cameras"]["left_realsense"]["connected"] = False
        cases.append(("not connected", disconnected))
        wrong_serial = _health(self.left_camera, self.right_camera)
        wrong_serial["cameras"]["right_realsense"]["serial"] = "WRONG"
        cases.append(("serial mismatch", wrong_serial))
        wrong_product = _health(self.left_camera, self.right_camera)
        wrong_product["cameras"]["left_realsense"]["product_id"] = "FFFF"
        cases.append(("product_id", wrong_product))
        wrong_schema = _health(self.left_camera, self.right_camera)
        wrong_schema["schema"] = "camera_server.health.v0"
        cases.append(("schema", wrong_schema))
        for reason, health in cases:
            with self.subTest(reason=reason):
                with self.assertRaises(PikaUsbPairingError):
                    self.resolve(health)


class PikaCameraConfigTest(unittest.TestCase):
    def test_loads_tracked_shape_and_rejects_placeholder(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / "cameras.yaml"
            config.write_text(
                "cameras:\n"
                "  left_realsense:\n"
                f'    serial: "{LEFT_SERIAL}"\n'
                "  right_realsense:\n"
                f'    serial: "{RIGHT_SERIAL}"\n'
            )
            self.assertEqual(
                load_expected_camera_serials(config),
                {"left": LEFT_SERIAL, "right": RIGHT_SERIAL},
            )
            config.write_text(
                "cameras:\n"
                "  left_realsense: {serial: REPLACE_LEFT_SERIAL}\n"
                f'  right_realsense: {{serial: "{RIGHT_SERIAL}"}}\n'
            )
            with self.assertRaisesRegex(PikaUsbPairingError, "accepted physical serial"):
                load_expected_camera_serials(config)


class CameraHealthWaitTest(unittest.TestCase):
    def test_missing_camera_server_times_out_fail_closed(self) -> None:
        try:
            import zmq  # noqa: F401
        except ModuleNotFoundError:
            self.skipTest("pyzmq not installed")
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.bind(("127.0.0.1", 0))
            port = sock.getsockname()[1]
        with self.assertRaisesRegex(PikaUsbPairingError, "start camera_server first"):
            wait_for_camera_health(
                f"tcp://127.0.0.1:{port}",
                timeout_sec=0.02,
            )


if __name__ == "__main__":
    unittest.main()
