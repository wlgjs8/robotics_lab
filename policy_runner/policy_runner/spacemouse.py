from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Protocol


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
    def read(self) -> SpaceMouseSample | None:
        ...


class FakeSpaceMouseReader:
    def __init__(self, samples: list[SpaceMouseSample], repeat_last: bool = False):
        self._samples = list(samples)
        self._repeat_last = repeat_last
        self._index = 0

    def read(self) -> SpaceMouseSample | None:
        if not self._samples:
            return None
        if self._index >= len(self._samples):
            return self._samples[-1] if self._repeat_last else None
        sample = self._samples[self._index]
        self._index += 1
        return sample


class HidSpaceMouseReader:
    """Optional pyspacemouse-backed reader.

    The dependency is intentionally imported only when this class is
    constructed so normal policy_runner tests remain HID-free.
    """

    def __init__(self, device: str | None = None, path: str | None = None):
        try:
            import pyspacemouse  # type: ignore
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "real SpaceMouse input requires optional dependency "
                "`policy_runner[spacemouse]`"
            ) from exc
        self._pyspacemouse = pyspacemouse
        self._device_handle = None
        if path:
            self._device_handle = pyspacemouse.open_by_path(path)
        else:
            try:
                self._device_handle = pyspacemouse.open(device=device) if device else pyspacemouse.open()
            except TypeError:
                self._device_handle = pyspacemouse.open()
        if self._device_handle is False or self._device_handle is None:
            raise RuntimeError("failed to open SpaceMouse device")

    def read(self) -> SpaceMouseSample | None:
        if hasattr(self._device_handle, "read"):
            raw = self._device_handle.read()
        else:
            raw = self._pyspacemouse.read()
        if raw is None:
            return None
        buttons = getattr(raw, "buttons", getattr(raw, "button", ()))
        return SpaceMouseSample(
            tx=float(getattr(raw, "x")),
            ty=float(getattr(raw, "y")),
            tz=float(getattr(raw, "z")),
            rx=float(getattr(raw, "roll")),
            ry=float(getattr(raw, "pitch")),
            rz=float(getattr(raw, "yaw")),
            buttons=tuple(bool(v) for v in buttons),
            timestamp_monotonic=time.monotonic(),
        )
