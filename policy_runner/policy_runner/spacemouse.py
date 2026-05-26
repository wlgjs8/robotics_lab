from __future__ import annotations

import time
from collections.abc import Iterable
from dataclasses import dataclass
from typing import Any, Protocol


@dataclass(frozen=True)
class SpaceMouseSample:
    tx: float
    ty: float
    tz: float
    rx: float
    ry: float
    rz: float
    buttons: tuple[bool, ...]
    timestamp_monotonic: float


class SpaceMouseReader(Protocol):
    def read(self, timeout_sec: float | None = None) -> SpaceMouseSample | None:
        ...

    def close(self) -> None:
        ...


class FakeSpaceMouseReader:
    def __init__(self, samples: Iterable[SpaceMouseSample] = ()):
        self._samples = list(samples)
        self.closed = False

    def push(self, sample: SpaceMouseSample) -> None:
        self._samples.append(sample)

    def read(self, timeout_sec: float | None = None) -> SpaceMouseSample | None:
        _ = timeout_sec
        if not self._samples:
            return None
        return self._samples.pop(0)

    def close(self) -> None:
        self.closed = True


class HidSpaceMouseReader:
    """Optional pyspacemouse-backed reader kept out of the test dependency path."""

    _MAX_DRAIN_STATES = 64

    def __init__(
        self,
        *,
        device: str | None = None,
        path: str | None = None,
        device_number: int = 0,
    ):
        if device_number < 0:
            raise ValueError("device_number must be non-negative")
        try:
            import pyspacemouse  # type: ignore
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "SpaceMouse HID support requires optional dependency: "
                "install policy_runner with the spacemouse extra"
            ) from exc
        self._pyspacemouse = pyspacemouse
        self._device = _open_pyspacemouse_device(
            pyspacemouse,
            device=device,
            path=path,
            device_number=device_number,
        )
        if self._device is None or self._device is False:
            raise RuntimeError("failed to open SpaceMouse HID device")

    def read(self, timeout_sec: float | None = None) -> SpaceMouseSample | None:
        if self._device is None:
            return None

        latest_state = self._drain_buffered_states()
        if latest_state is not None:
            return _sample_from_pyspacemouse_state(latest_state)
        if timeout_sec is None or timeout_sec <= 0.0:
            return None

        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            read = getattr(self._device, "read", None)
            state = read() if callable(read) else self._pyspacemouse.read()
            if state is not None:
                tail = self._drain_buffered_states()
                return _sample_from_pyspacemouse_state(tail if tail is not None else state)
            time.sleep(0.001)
        return None

    def _drain_buffered_states(self) -> Any:
        latest = None
        for _ in range(self._MAX_DRAIN_STATES):
            read = getattr(self._device, "read", None)
            state = read() if callable(read) else self._pyspacemouse.read()
            if state is None:
                break
            latest = state
        return latest

    def close(self) -> None:
        close = getattr(self._device, "close", None)
        if callable(close):
            close()
        self._device = None


def _sample_from_pyspacemouse_state(state: Any) -> SpaceMouseSample:
    return SpaceMouseSample(
        tx=_float_attr(state, "tx", "x", "translation_x"),
        ty=_float_attr(state, "ty", "y", "translation_y"),
        tz=_float_attr(state, "tz", "z", "translation_z"),
        rx=_float_attr(state, "rx", "roll", "rotation_x"),
        ry=_float_attr(state, "ry", "pitch", "rotation_y"),
        rz=_float_attr(state, "rz", "yaw", "rotation_z"),
        buttons=_buttons_attr(state),
        timestamp_monotonic=time.monotonic(),
    )


def _open_pyspacemouse_device(
    pyspacemouse: Any,
    *,
    device: str | None,
    path: str | None,
    device_number: int,
) -> Any:
    if path:
        open_by_path = getattr(pyspacemouse, "open_by_path", None)
        if callable(open_by_path):
            return open_by_path(path)
        return pyspacemouse.open(device=device, path=path, DeviceNumber=device_number)
    try:
        return pyspacemouse.open(device=device, device_index=device_number)
    except TypeError as exc:
        if "device_index" not in str(exc):
            raise
        return pyspacemouse.open(device=device, DeviceNumber=device_number)


def _float_attr(state: Any, *names: str) -> float:
    for name in names:
        value = getattr(state, name, None)
        if value is not None:
            return float(value)
    return 0.0


def _buttons_attr(state: Any) -> tuple[bool, ...]:
    buttons = getattr(state, "buttons", ())
    if buttons is None:
        return ()
    return tuple(bool(button) for button in buttons)
