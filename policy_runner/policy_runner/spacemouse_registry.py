from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Mapping

from .spacemouse import HidSpaceMouseReader, SpaceMouseReader, SpaceMouseSample


SPACEMOUSE_ASSIGNMENT_COMMAND_SCHEMA = "robotics_lab.spacemouse_assignment_cmd.v1"


@dataclass(frozen=True)
class SpaceMouseHidDescriptor:
    path: str
    vendor_id: int
    product_id: int
    interface_number: int
    product: str = ""
    serial: str = ""


@dataclass
class _ConnectedDevice:
    descriptor: SpaceMouseHidDescriptor
    connection_id: str
    reader: SpaceMouseReader
    sample: SpaceMouseSample | None = None
    activity: float = 0.0
    neutral: bool = True
    error: str = ""


def enumerate_spacemouse_hid() -> list[SpaceMouseHidDescriptor]:
    try:
        from easyhid import Enumeration  # type: ignore
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "SpaceMouse discovery requires easyhid/pyspacemouse; install policy_runner with the spacemouse extra"
        ) from exc

    descriptors: dict[tuple[str, int, int, int], SpaceMouseHidDescriptor] = {}
    for device in Enumeration().find() or []:
        path = getattr(device, "path", "")
        if isinstance(path, bytes):
            path = path.decode("utf-8", errors="replace")
        descriptor = SpaceMouseHidDescriptor(
            path=str(path),
            vendor_id=int(getattr(device, "vendor_id", 0) or 0),
            product_id=int(getattr(device, "product_id", 0) or 0),
            interface_number=int(getattr(device, "interface_number", -1)),
            product=str(getattr(device, "product_string", "") or ""),
            serial=str(getattr(device, "serial_number", "") or ""),
        )
        key = (
            descriptor.path,
            descriptor.vendor_id,
            descriptor.product_id,
            descriptor.interface_number,
        )
        descriptors[key] = descriptor
    return list(descriptors.values())


class SpaceMouseDeviceRegistry:
    """Own all matching HID handles and expose explicit logical arm assignments."""

    def __init__(
        self,
        *,
        vendor_id: int = 0x256F,
        product_id: int = 0xC652,
        interface_number: int = 0,
        scan_period_sec: float = 0.5,
        poll_period_sec: float = 0.002,
        neutral_threshold: float = 0.06,
        enumerate_fn: Callable[[], list[SpaceMouseHidDescriptor]] = enumerate_spacemouse_hid,
        reader_factory: Callable[[str], SpaceMouseReader] | None = None,
        autostart: bool = True,
    ) -> None:
        if scan_period_sec <= 0.0 or poll_period_sec <= 0.0:
            raise ValueError("SpaceMouse discovery periods must be positive")
        self.vendor_id = int(vendor_id)
        self.product_id = int(product_id)
        self.interface_number = int(interface_number)
        self.scan_period_sec = float(scan_period_sec)
        self.poll_period_sec = float(poll_period_sec)
        self.neutral_threshold = float(neutral_threshold)
        self._enumerate = enumerate_fn
        self._reader_factory = reader_factory or (
            lambda path: HidSpaceMouseReader(path=path, device_number=0)
        )
        self._devices: dict[str, _ConnectedDevice] = {}
        self._assignments: dict[str, str | None] = {"left": None, "right": None}
        self._generation = 0
        self._connection_counter = 0
        self._last_command_seq = 0
        self._last_result = ""
        self._last_error = ""
        self._scan_error = ""
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        if autostart:
            self.start()

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._run,
            name="spacemouse-device-registry",
            daemon=True,
        )
        self._thread.start()

    def _run(self) -> None:
        next_scan = float("-inf")
        while not self._stop.is_set():
            now = time.monotonic()
            if now >= next_scan:
                self.scan_once()
                next_scan = now + self.scan_period_sec
            self.poll_once()
            self._stop.wait(self.poll_period_sec)

    def scan_once(self) -> None:
        try:
            found = self._enumerate()
            self._scan_error = ""
        except Exception as exc:  # noqa: BLE001 - discovery must recover in background
            with self._lock:
                self._scan_error = f"{type(exc).__name__}: {exc}"
            return
        candidates = {
            descriptor.path: descriptor
            for descriptor in found
            if descriptor.path
            and descriptor.vendor_id == self.vendor_id
            and descriptor.product_id == self.product_id
            and descriptor.interface_number == self.interface_number
        }
        with self._lock:
            for path in sorted(set(self._devices) - set(candidates)):
                self._disconnect_locked(path, "device_removed")
            for path, descriptor in sorted(candidates.items()):
                if path in self._devices:
                    continue
                try:
                    reader = self._reader_factory(path)
                except Exception as exc:  # noqa: BLE001 - rescan retries this candidate
                    self._scan_error = f"open_failed:{path}:{type(exc).__name__}:{exc}"
                    continue
                self._connection_counter += 1
                self._generation += 1
                self._devices[path] = _ConnectedDevice(
                    descriptor=descriptor,
                    connection_id=f"sm-{self._generation}-{self._connection_counter}",
                    reader=reader,
                )

    def poll_once(self) -> None:
        with self._lock:
            paths = list(self._devices)
        for path in paths:
            with self._lock:
                device = self._devices.get(path)
            if device is None:
                continue
            try:
                sample = device.reader.read(timeout_sec=0.0)
            except Exception as exc:  # noqa: BLE001 - unplug is a recoverable transition
                with self._lock:
                    self._disconnect_locked(path, f"read_failed:{type(exc).__name__}:{exc}")
                continue
            if sample is None:
                continue
            activity = max(
                abs(sample.tx), abs(sample.ty), abs(sample.tz),
                abs(sample.rx), abs(sample.ry), abs(sample.rz),
            )
            with self._lock:
                current = self._devices.get(path)
                if current is not None:
                    current.sample = sample
                    current.activity = float(activity)
                    current.neutral = activity <= self.neutral_threshold
                    current.error = ""

    def _disconnect_locked(self, path: str, reason: str) -> None:
        device = self._devices.pop(path, None)
        if device is None:
            return
        try:
            device.reader.close()
        except Exception:
            pass
        for arm, assigned in tuple(self._assignments.items()):
            if assigned == device.connection_id:
                self._assignments[arm] = None
        self._generation += 1
        self._last_error = reason

    def sample_for_arm(self, arm: str) -> SpaceMouseSample | None:
        if arm not in self._assignments:
            raise ValueError("SpaceMouse arm must be left or right")
        with self._lock:
            connection_id = self._assignments[arm]
            for device in self._devices.values():
                if device.connection_id == connection_id:
                    return device.sample
        return None

    def handle_control(self, payload: Mapping[str, Any], *, assignment_change_allowed: bool) -> bool:
        if payload.get("schema") != SPACEMOUSE_ASSIGNMENT_COMMAND_SCHEMA:
            return False
        seq = int(payload.get("seq", 0) or 0)
        expected_generation = int(payload.get("status_generation", -1) or -1)
        command = str(payload.get("command", "") or "")
        with self._lock:
            self._last_command_seq = seq
            if not assignment_change_allowed:
                self._reject_locked("active_owner")
                return True
            if expected_generation != self._generation:
                self._reject_locked("stale_generation")
                return True
            if command == "set":
                left = _optional_connection_id(payload.get("left_connection_id"))
                right = _optional_connection_id(payload.get("right_connection_id"))
                if left is not None and left == right:
                    self._reject_locked("duplicate_assignment")
                    return True
                connected = {device.connection_id for device in self._devices.values()}
                if any(value is not None and value not in connected for value in (left, right)):
                    self._reject_locked("unknown_connection")
                    return True
                self._assignments = {"left": left, "right": right}
            elif command == "swap":
                self._assignments["left"], self._assignments["right"] = (
                    self._assignments["right"], self._assignments["left"]
                )
            else:
                self._reject_locked("unsupported_command")
                return True
            self._generation += 1
            self._last_result = "accepted"
            self._last_error = ""
            return True

    def _reject_locked(self, reason: str) -> None:
        self._last_result = "rejected"
        self._last_error = reason

    def status_block(self, *, assignment_change_allowed: bool, block_reason: str = "") -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            arm_by_connection = {
                connection_id: arm
                for arm, connection_id in self._assignments.items()
                if connection_id is not None
            }
            devices = [
                {
                    "connection_id": device.connection_id,
                    "state": "connected",
                    "arm": arm_by_connection.get(device.connection_id, "unassigned"),
                    "product": device.descriptor.product,
                    "serial": device.descriptor.serial,
                    "path": device.descriptor.path,
                    "interface_number": device.descriptor.interface_number,
                    "activity": device.activity,
                    "neutral": device.neutral,
                    "neutral_threshold": self.neutral_threshold,
                    "has_sample": device.sample is not None,
                    "sample_age_sec": (
                        max(0.0, now - device.sample.timestamp_monotonic)
                        if device.sample is not None
                        else None
                    ),
                    "raw_axes": (
                        [
                            device.sample.tx,
                            device.sample.ty,
                            device.sample.tz,
                            device.sample.rx,
                            device.sample.ry,
                            device.sample.rz,
                        ]
                        if device.sample is not None
                        else None
                    ),
                    "error": device.error,
                }
                for device in sorted(self._devices.values(), key=lambda item: item.connection_id)
            ]
            return {
                "generation": self._generation,
                "scan_state": "error" if self._scan_error else "ready",
                "scan_error": self._scan_error,
                "assignment_change_allowed": bool(assignment_change_allowed),
                "block_reason": block_reason,
                "left_connection_id": self._assignments["left"],
                "right_connection_id": self._assignments["right"],
                "devices": devices,
                "last_command_seq": self._last_command_seq,
                "last_result": self._last_result,
                "last_error": self._last_error,
            }

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        with self._lock:
            for path in list(self._devices):
                self._disconnect_locked(path, "registry_closed")


class RegistrySpaceMouseReader:
    def __init__(self, registry: SpaceMouseDeviceRegistry, arm: str) -> None:
        self.registry = registry
        self.arm = arm

    def read(self, timeout_sec: float | None = None) -> SpaceMouseSample | None:
        _ = timeout_sec
        return self.registry.sample_for_arm(self.arm)

    def close(self) -> None:
        pass


def _optional_connection_id(value: Any) -> str | None:
    text = str(value or "").strip()
    return text or None
