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

    def __init__(self):
        try:
            import pyspacemouse  # type: ignore
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "SpaceMouse HID support requires optional dependency: "
                "install policy_runner with the spacemouse extra"
            ) from exc
        self._pyspacemouse = pyspacemouse
        opened = pyspacemouse.open()
        if opened is False:
            raise RuntimeError("failed to open SpaceMouse HID device")

    def read(self, timeout_sec: float | None = None) -> SpaceMouseSample | None:
        deadline = None if timeout_sec is None else time.monotonic() + max(timeout_sec, 0.0)
        while True:
            state = self._pyspacemouse.read()
            if state is not None:
                return _sample_from_pyspacemouse_state(state)
            if deadline is not None and time.monotonic() >= deadline:
                return None
            time.sleep(0.001)

    def close(self) -> None:
        close = getattr(self._pyspacemouse, "close", None)
        if callable(close):
            close()


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
