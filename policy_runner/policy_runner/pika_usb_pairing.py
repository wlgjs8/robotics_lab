"""Fail-closed Pika gripper pairing from RealSense identity and USB topology.

Each robot-side hardware module exposes a D405 on the SuperSpeed half of one
host port and a CH340 serial adapter on that port's USB2 companion tree.  The
CH340 adapters have no unique serial number, so tty numbering and ``by-id`` are
not arm identities.  The camera server does know the arm identity from the
configured librealsense serial and publishes the matching physical USB path.

This module joins those two facts at process startup:

``camera name/serial -> xHCI controller + root port -> exactly one CH340 tty``.

There is deliberately no fallback to ttyUSB enumeration order, a fixed
``KERNELS`` value, or an existing ``/dev/pika-*`` symlink.
"""
from __future__ import annotations

import json
import re
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

ARMS = ("left", "right")
CAMERA_NAMES = {"left": "left_realsense", "right": "right_realsense"}
CAMERA_HEALTH_SCHEMA = "camera_server.health.v1"
PAIRING_SCHEMA = "robotics_lab.pika_usb_pairing.v1"
D405_PRODUCT_ID = "0b5b"
CH340_VENDOR_ID = "1a86"
CH340_PRODUCT_ID = "7522"

_USB_DEVICE_NODE_RE = re.compile(r"(?P<bus>[0-9]+)-[0-9]+(?:\.[0-9]+)*$")


class PikaUsbPairingError(RuntimeError):
    """Raised when an arm-to-gripper mapping cannot be proven."""


@dataclass(frozen=True)
class CameraUsbRoute:
    arm: str
    camera_name: str
    serial: str
    physical_port: str
    usb_device_node: str
    controller_path: str
    root_port: str


@dataclass(frozen=True)
class GripperUsbCandidate:
    tty_name: str
    port: str
    usb_device_node: str
    controller_path: str
    root_port: str


@dataclass(frozen=True)
class PikaArmPairing:
    arm: str
    camera_name: str
    camera_serial: str
    camera_physical_port: str
    camera_usb_device_node: str
    controller_path: str
    root_port: str
    gripper_usb_device_node: str
    gripper_tty: str
    gripper_port: str

    def to_dict(self) -> dict[str, str]:
        return {
            "camera_name": self.camera_name,
            "camera_serial": self.camera_serial,
            "camera_physical_port": self.camera_physical_port,
            "camera_usb_device_node": self.camera_usb_device_node,
            "controller_path": self.controller_path,
            "root_port": self.root_port,
            "gripper_usb_device_node": self.gripper_usb_device_node,
            "gripper_tty": self.gripper_tty,
            "gripper_port": self.gripper_port,
        }


@dataclass(frozen=True)
class PikaUsbPairing:
    arms: Mapping[str, PikaArmPairing]

    @property
    def ports(self) -> dict[str, str]:
        return {arm: self.arms[arm].gripper_port for arm in ARMS}

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema": PAIRING_SCHEMA,
            "arms": {arm: self.arms[arm].to_dict() for arm in ARMS},
        }


def _read_text(path: Path) -> str:
    try:
        return path.read_text().strip()
    except OSError as exc:
        raise PikaUsbPairingError(f"required sysfs attribute unreadable: {path} ({exc})") from exc


def load_expected_camera_serials(config_path: str | Path) -> dict[str, str]:
    """Load the authoritative left/right D405 librealsense serials."""
    try:
        import yaml  # type: ignore
    except ModuleNotFoundError as exc:
        raise PikaUsbPairingError(
            "PyYAML is required for Pika camera pairing; install policy_runner[gripper]"
        ) from exc

    path = Path(config_path)
    try:
        loaded = yaml.safe_load(path.read_text()) or {}
    except OSError as exc:
        raise PikaUsbPairingError(f"camera pairing config unreadable: {path} ({exc})") from exc
    except Exception as exc:  # noqa: BLE001 - surface parser detail without guessing
        raise PikaUsbPairingError(f"camera pairing config invalid: {path} ({exc})") from exc

    cameras = loaded.get("cameras") if isinstance(loaded, Mapping) else None
    if not isinstance(cameras, Mapping):
        raise PikaUsbPairingError(f"camera pairing config has no cameras map: {path}")

    serials: dict[str, str] = {}
    for arm in ARMS:
        camera_name = CAMERA_NAMES[arm]
        spec = cameras.get(camera_name)
        if not isinstance(spec, Mapping):
            raise PikaUsbPairingError(
                f"camera pairing config missing cameras.{camera_name}: {path}"
            )
        backend = str(spec.get("backend", "realsense"))
        if backend != "realsense":
            raise PikaUsbPairingError(
                f"cameras.{camera_name}.backend must be realsense, got {backend!r}"
            )
        serial = str(spec.get("serial", "")).strip()
        if not serial or serial.startswith("REPLACE_") or serial.startswith("MOCK_"):
            raise PikaUsbPairingError(
                f"cameras.{camera_name}.serial is not an accepted physical serial"
            )
        serials[arm] = serial

    if serials["left"] == serials["right"]:
        raise PikaUsbPairingError("left/right camera serials must be distinct")
    return serials


def wait_for_camera_health(
    endpoint: str,
    topic: str = "camera.health",
    timeout_sec: float = 5.0,
) -> dict[str, Any]:
    """Wait for one newly published camera health document."""
    if timeout_sec <= 0.0:
        raise PikaUsbPairingError("pairing timeout must be positive")
    try:
        import zmq  # type: ignore
    except ModuleNotFoundError as exc:
        raise PikaUsbPairingError(
            "pyzmq is required for Pika camera pairing; install policy_runner[gripper]"
        ) from exc

    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.RCVHWM, 1)
    sock.setsockopt_string(zmq.SUBSCRIBE, topic)
    sock.connect(endpoint)
    deadline = time.monotonic() + float(timeout_sec)
    last_reject = "no message received"
    try:
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            if not sock.poll(timeout=max(1, int(remaining * 1000)), flags=zmq.POLLIN):
                break
            parts = sock.recv_multipart()
            if len(parts) != 2:
                last_reject = f"expected 2 message parts, got {len(parts)}"
                continue
            try:
                message_topic = parts[0].decode("utf-8")
            except UnicodeDecodeError:
                last_reject = "topic is not UTF-8"
                continue
            if message_topic != topic:
                last_reject = f"unexpected topic {message_topic!r}"
                continue
            try:
                document = json.loads(parts[1].decode("utf-8"))
            except (UnicodeDecodeError, ValueError) as exc:
                last_reject = f"invalid JSON ({exc})"
                continue
            if not isinstance(document, dict):
                last_reject = "health payload is not a JSON object"
                continue
            return document
    finally:
        sock.close(linger=0)
    raise PikaUsbPairingError(
        f"camera health unavailable at {endpoint} topic={topic!r} "
        f"within {timeout_sec:g}s ({last_reject}); start camera_server first"
    )


def _camera_blocks(
    health: Mapping[str, Any],
    expected_serials: Mapping[str, str],
) -> dict[str, Mapping[str, Any]]:
    if health.get("schema") != CAMERA_HEALTH_SCHEMA:
        raise PikaUsbPairingError(
            f"camera health schema must be {CAMERA_HEALTH_SCHEMA!r}, "
            f"got {health.get('schema')!r}"
        )
    cameras = health.get("cameras")
    if not isinstance(cameras, Mapping):
        raise PikaUsbPairingError("camera health has no cameras map")

    blocks: dict[str, Mapping[str, Any]] = {}
    physical_ports: set[str] = set()
    for arm in ARMS:
        camera_name = CAMERA_NAMES[arm]
        block = cameras.get(camera_name)
        if not isinstance(block, Mapping):
            raise PikaUsbPairingError(f"camera health missing {camera_name}")
        if block.get("connected") is not True:
            raise PikaUsbPairingError(f"camera {camera_name} is not connected")
        actual_serial = str(block.get("serial", "")).strip()
        expected_serial = str(expected_serials.get(arm, "")).strip()
        if actual_serial != expected_serial:
            raise PikaUsbPairingError(
                f"camera {camera_name} serial mismatch: "
                f"expected={expected_serial!r} actual={actual_serial!r}"
            )
        product_id = str(block.get("product_id", "")).strip().lower()
        if product_id != D405_PRODUCT_ID:
            raise PikaUsbPairingError(
                f"camera {camera_name} product_id must be D405 {D405_PRODUCT_ID}, "
                f"got {product_id!r}"
            )
        physical_port = str(block.get("physical_port", "")).strip()
        if not physical_port or not Path(physical_port).is_absolute():
            raise PikaUsbPairingError(
                f"camera {camera_name} physical_port is missing or not absolute"
            )
        if physical_port in physical_ports:
            raise PikaUsbPairingError("left/right cameras report the same physical_port")
        physical_ports.add(physical_port)
        blocks[arm] = block
    return blocks


def _usb_device_node_from_physical_port(
    physical_port: str,
    sysfs_usb_root: Path,
) -> str:
    nodes = [
        segment
        for segment in Path(physical_port).parts
        if ":" not in segment and _USB_DEVICE_NODE_RE.fullmatch(segment)
    ]
    if not nodes:
        raise PikaUsbPairingError(
            f"camera physical_port contains no USB device node: {physical_port}"
        )
    node = nodes[-1]
    device_link = sysfs_usb_root / node
    try:
        device_path = device_link.resolve(strict=True)
    except OSError as exc:
        raise PikaUsbPairingError(
            f"camera USB device {node} is not present in sysfs: {physical_port} ({exc}); "
            "check the camera connection and refresh camera_server health before retrying"
        ) from exc
    try:
        physical_path = Path(physical_port).resolve(strict=True)
    except OSError as exc:
        # A reconnected D405 may have new videoN leaves. The producer must
        # refresh its serial-to-port association; a surviving USB ancestor
        # alone cannot prove that the same camera still occupies that port.
        raise PikaUsbPairingError(
            f"camera physical_port is not present in sysfs: {physical_port} ({exc}); "
            f"USB device {node} still exists, but camera.health may contain a stale "
            "path after USB reconnect. Restart camera_server and retry pairing"
        ) from exc
    if device_path != physical_path and device_path not in physical_path.parents:
        raise PikaUsbPairingError(
            f"camera physical_port does not belong to sysfs device {node}: {physical_port}"
        )
    return node


def _usb_controller_and_root_port(
    usb_device_node: str,
    sysfs_usb_root: Path,
) -> tuple[str, str]:
    match = _USB_DEVICE_NODE_RE.fullmatch(usb_device_node)
    if match is None:
        raise PikaUsbPairingError(f"invalid USB device node: {usb_device_node}")
    bus = match.group("bus")
    root_hub = sysfs_usb_root / f"usb{bus}"
    device = sysfs_usb_root / usb_device_node
    try:
        controller = root_hub.resolve(strict=True).parent
    except OSError as exc:
        raise PikaUsbPairingError(
            f"USB root hub unavailable for {usb_device_node}: {root_hub} ({exc})"
        ) from exc
    devpath = _read_text(device / "devpath")
    root_port = devpath.split(".", 1)[0]
    if not root_port.isdigit() or int(root_port) <= 0:
        raise PikaUsbPairingError(
            f"invalid USB devpath for {usb_device_node}: {devpath!r}"
        )
    return str(controller), root_port


def _camera_routes(
    blocks: Mapping[str, Mapping[str, Any]],
    expected_serials: Mapping[str, str],
    sysfs_usb_root: Path,
) -> dict[str, CameraUsbRoute]:
    routes: dict[str, CameraUsbRoute] = {}
    for arm in ARMS:
        camera_name = CAMERA_NAMES[arm]
        physical_port = str(blocks[arm]["physical_port"])
        node = _usb_device_node_from_physical_port(physical_port, sysfs_usb_root)
        controller, root_port = _usb_controller_and_root_port(node, sysfs_usb_root)
        routes[arm] = CameraUsbRoute(
            arm=arm,
            camera_name=camera_name,
            serial=str(expected_serials[arm]),
            physical_port=physical_port,
            usb_device_node=node,
            controller_path=controller,
            root_port=root_port,
        )
    return routes


def _find_usb_device_ancestor(path: Path) -> Path | None:
    for candidate in (path, *path.parents):
        if (candidate / "idVendor").is_file() and (candidate / "idProduct").is_file():
            return candidate
    return None


def _stable_by_path_for_tty(
    tty_name: str,
    dev_root: Path,
    dev_serial_by_path: Path,
) -> str:
    tty_device = dev_root / tty_name
    try:
        tty_target = tty_device.resolve(strict=True)
        links = [
            link
            for link in sorted(dev_serial_by_path.iterdir())
            if link.is_symlink() and link.resolve(strict=True) == tty_target
        ]
    except OSError as exc:
        raise PikaUsbPairingError(
            f"cannot resolve stable by-path link for {tty_name}: {exc}"
        ) from exc
    if len(links) != 1:
        raise PikaUsbPairingError(
            f"expected exactly one /dev/serial/by-path link for {tty_name}, "
            f"found {len(links)}"
        )
    return str(links[0])


def scan_ch340_grippers(
    *,
    sysfs_usb_root: str | Path = "/sys/bus/usb/devices",
    sysfs_tty_root: str | Path = "/sys/class/tty",
    dev_root: str | Path = "/dev",
    dev_serial_by_path: str | Path = "/dev/serial/by-path",
) -> list[GripperUsbCandidate]:
    """Enumerate CH340 tty devices with their companion-root identity."""
    usb_root = Path(sysfs_usb_root)
    tty_root = Path(sysfs_tty_root)
    devices_root = Path(dev_root)
    by_path_root = Path(dev_serial_by_path)
    candidates: list[GripperUsbCandidate] = []
    for tty_entry in sorted(tty_root.glob("ttyUSB*")):
        try:
            tty_sysfs = (tty_entry / "device").resolve(strict=True)
        except OSError:
            continue
        usb_device = _find_usb_device_ancestor(tty_sysfs)
        if usb_device is None:
            continue
        vendor = _read_text(usb_device / "idVendor").lower()
        product = _read_text(usb_device / "idProduct").lower()
        if vendor != CH340_VENDOR_ID or product != CH340_PRODUCT_ID:
            continue
        node = usb_device.name
        controller, root_port = _usb_controller_and_root_port(node, usb_root)
        stable_port = _stable_by_path_for_tty(
            tty_entry.name,
            devices_root,
            by_path_root,
        )
        candidates.append(
            GripperUsbCandidate(
                tty_name=tty_entry.name,
                port=stable_port,
                usb_device_node=node,
                controller_path=controller,
                root_port=root_port,
            )
        )
    return candidates


def resolve_pika_usb_pairing(
    health: Mapping[str, Any],
    expected_serials: Mapping[str, str],
    *,
    sysfs_usb_root: str | Path = "/sys/bus/usb/devices",
    sysfs_tty_root: str | Path = "/sys/class/tty",
    dev_root: str | Path = "/dev",
    dev_serial_by_path: str | Path = "/dev/serial/by-path",
) -> PikaUsbPairing:
    """Resolve both arms, refusing every ambiguous or incomplete topology."""
    usb_root = Path(sysfs_usb_root)
    blocks = _camera_blocks(health, expected_serials)
    cameras = _camera_routes(blocks, expected_serials, usb_root)
    grippers = scan_ch340_grippers(
        sysfs_usb_root=usb_root,
        sysfs_tty_root=sysfs_tty_root,
        dev_root=dev_root,
        dev_serial_by_path=dev_serial_by_path,
    )

    resolved: dict[str, PikaArmPairing] = {}
    used_ttys: set[str] = set()
    for arm in ARMS:
        camera = cameras[arm]
        matches = [
            candidate
            for candidate in grippers
            if candidate.controller_path == camera.controller_path
            and candidate.root_port == camera.root_port
        ]
        if len(matches) != 1:
            detail = [
                {
                    "tty": candidate.tty_name,
                    "usb": candidate.usb_device_node,
                    "controller": candidate.controller_path,
                    "root_port": candidate.root_port,
                }
                for candidate in matches
            ]
            raise PikaUsbPairingError(
                f"{arm} camera serial={camera.serial} controller={camera.controller_path} "
                f"root_port={camera.root_port} must match exactly one CH340, "
                f"found {len(matches)}: {detail}"
            )
        gripper = matches[0]
        if gripper.tty_name in used_ttys:
            raise PikaUsbPairingError(
                f"left/right pairing resolved to the same tty: {gripper.tty_name}"
            )
        used_ttys.add(gripper.tty_name)
        resolved[arm] = PikaArmPairing(
            arm=arm,
            camera_name=camera.camera_name,
            camera_serial=camera.serial,
            camera_physical_port=camera.physical_port,
            camera_usb_device_node=camera.usb_device_node,
            controller_path=camera.controller_path,
            root_port=camera.root_port,
            gripper_usb_device_node=gripper.usb_device_node,
            gripper_tty=gripper.tty_name,
            gripper_port=gripper.port,
        )
    return PikaUsbPairing(arms=resolved)


def resolve_pika_usb_pairing_from_camera_health(
    camera_config: str | Path,
    *,
    endpoint: str = "tcp://127.0.0.1:5600",
    topic: str = "camera.health",
    timeout_sec: float = 5.0,
    sysfs_usb_root: str | Path = "/sys/bus/usb/devices",
    sysfs_tty_root: str | Path = "/sys/class/tty",
    dev_root: str | Path = "/dev",
    dev_serial_by_path: str | Path = "/dev/serial/by-path",
) -> PikaUsbPairing:
    expected_serials = load_expected_camera_serials(camera_config)
    health = wait_for_camera_health(endpoint, topic, timeout_sec)
    return resolve_pika_usb_pairing(
        health,
        expected_serials,
        sysfs_usb_root=sysfs_usb_root,
        sysfs_tty_root=sysfs_tty_root,
        dev_root=dev_root,
        dev_serial_by_path=dev_serial_by_path,
    )
