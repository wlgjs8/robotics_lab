from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

import numpy as np


PHASE_NAMES = ("right_pick", "right_place", "left_pick", "left_place")
EVENT_NAMES = ("right_close", "right_open", "left_close", "left_open")

# Relative hysteresis band inside each arm's own [min, max] gripper-open range.
# HIGH means open; LOW means closed.
CLOSE_FRACTION = 0.30
OPEN_FRACTION = 0.55
MIN_RANGE = 15.0


@dataclass(frozen=True)
class GripperThresholds:
    low: float
    high: float
    close: float
    open: float


@dataclass(frozen=True)
class PhaseBoundaries:
    """Frame indices for right close/open and left close/open events."""

    length: int
    b1: int
    b2: int
    b3: int
    b4: int
    clean: bool
    method: str

    def phase_for_frame(self, frame: int) -> str:
        if frame < self.b1:
            return "right_pick"
        if frame < self.b2:
            return "right_place"
        if frame < self.b3:
            return "left_pick"
        return "left_place"

    def event_frames(self) -> dict[str, int]:
        return {
            "right_close": int(self.b1),
            "right_open": int(self.b2),
            "left_close": int(self.b3),
            "left_open": int(self.b4),
        }


def gripper_thresholds(signal: np.ndarray) -> GripperThresholds | None:
    x = np.asarray(signal, dtype=np.float64).reshape(-1)
    if x.size < 2:
        return None
    lo = float(np.nanmin(x))
    hi = float(np.nanmax(x))
    if not np.isfinite(lo) or not np.isfinite(hi):
        return None
    rng = hi - lo
    if rng < MIN_RANGE:
        return None
    return GripperThresholds(
        low=lo,
        high=hi,
        close=lo + CLOSE_FRACTION * rng,
        open=lo + OPEN_FRACTION * rng,
    )


def gripper_events(
    signal: np.ndarray,
    *,
    thresholds: GripperThresholds | None = None,
) -> dict[str, list[int]]:
    """Return close/open transition frames via hysteresis."""
    x = np.asarray(signal, dtype=np.float64).reshape(-1)
    closes: list[int] = []
    opens: list[int] = []
    if x.size < 2:
        return {"close": closes, "open": opens}
    th = thresholds if thresholds is not None else gripper_thresholds(x)
    if th is None:
        return {"close": closes, "open": opens}
    state = 1 if x[0] > th.close else 0
    for i in range(1, x.size):
        value = float(x[i])
        if not np.isfinite(value):
            continue
        if state == 1 and value < th.close:
            state = 0
            closes.append(i)
        elif state == 0 and value > th.open:
            state = 1
            opens.append(i)
    return {"close": closes, "open": opens}


def quarter_boundaries(length: int) -> PhaseBoundaries:
    length = int(length)
    q = max(1, length) / 4.0
    b1 = int(round(q))
    b2 = int(round(2 * q))
    b3 = int(round(3 * q))
    return PhaseBoundaries(
        length=length,
        b1=b1,
        b2=b2,
        b3=b3,
        b4=length,
        clean=False,
        method="quarter_fallback",
    )


def extract_phase_boundaries(
    left_gripper: np.ndarray,
    right_gripper: np.ndarray,
    length: int,
) -> PhaseBoundaries:
    """Recover b1<b2<b3<b4 from gripper-open signals, else fallback."""
    length = int(length)
    if length <= 4:
        return quarter_boundaries(length)
    right_events = gripper_events(np.asarray(right_gripper)[:length])
    left_events = gripper_events(np.asarray(left_gripper)[:length])
    try:
        b1 = right_events["close"][0]
        b2 = next(frame for frame in right_events["open"] if frame > b1)
        b3 = next(frame for frame in left_events["close"] if frame > b2)
        b4 = next(frame for frame in left_events["open"] if frame > b3)
    except (IndexError, StopIteration):
        return quarter_boundaries(length)
    if not (0 < b1 < b2 < b3 < b4 <= length):
        return quarter_boundaries(length)
    return PhaseBoundaries(
        length=length,
        b1=int(b1),
        b2=int(b2),
        b3=int(b3),
        b4=int(b4),
        clean=True,
        method="gripper_event",
    )


def ordered_event_frames_from_signals(
    *,
    left_signal: np.ndarray,
    right_signal: np.ndarray,
    left_thresholds: GripperThresholds | None,
    right_thresholds: GripperThresholds | None,
) -> dict[str, int | None]:
    """Detect predicted events in task order using supplied hysteresis thresholds."""
    right_events = gripper_events(right_signal, thresholds=right_thresholds)
    left_events = gripper_events(left_signal, thresholds=left_thresholds)
    out: dict[str, int | None] = {
        "right_close": None,
        "right_open": None,
        "left_close": None,
        "left_open": None,
    }
    try:
        b1 = right_events["close"][0]
        b2 = next(frame for frame in right_events["open"] if frame > b1)
        b3 = next(frame for frame in left_events["close"] if frame > b2)
        b4 = next(frame for frame in left_events["open"] if frame > b3)
    except (IndexError, StopIteration):
        if right_events["close"]:
            out["right_close"] = int(right_events["close"][0])
            try:
                out["right_open"] = int(next(frame for frame in right_events["open"] if frame > out["right_close"]))
            except StopIteration:
                pass
        if out["right_open"] is not None:
            try:
                out["left_close"] = int(next(frame for frame in left_events["close"] if frame > out["right_open"]))
                out["left_open"] = int(next(frame for frame in left_events["open"] if frame > out["left_close"]))
            except StopIteration:
                pass
        return out
    return {
        "right_close": int(b1),
        "right_open": int(b2),
        "left_close": int(b3),
        "left_open": int(b4),
    }


def build_boundary_cache(dataset: Any) -> dict[str, PhaseBoundaries]:
    cache: dict[str, PhaseBoundaries] = {}
    for episode in dataset.episodes:
        cache[str(episode.path)] = extract_phase_boundaries(
            episode.left_gripper,
            episode.right_gripper,
            episode.length,
        )
    return cache


def make_sample_phase(cache: dict[str, PhaseBoundaries]):
    def _sample_phase(dataset: Any, sample_index: int) -> str:
        ref = dataset.sample_refs[sample_index]
        episode = dataset.episodes[ref.episode_index]
        bounds = cache.get(str(episode.path))
        if bounds is None:
            bounds = extract_phase_boundaries(
                episode.left_gripper,
                episode.right_gripper,
                episode.length,
            )
            cache[str(episode.path)] = bounds
        return bounds.phase_for_frame(int(ref.start))

    return _sample_phase


def analyze_dataset(dataset: Any) -> dict[str, Any]:
    per_phase_frames: dict[str, list[int]] = {name: [] for name in PHASE_NAMES}
    clean = 0
    fallback_paths: list[str] = []
    rows = []
    for episode in dataset.episodes:
        b = extract_phase_boundaries(episode.left_gripper, episode.right_gripper, episode.length)
        lengths = {
            "right_pick": b.b1,
            "right_place": b.b2 - b.b1,
            "left_pick": b.b3 - b.b2,
            "left_place": b.length - b.b3,
        }
        if b.clean:
            clean += 1
            for name in PHASE_NAMES:
                per_phase_frames[name].append(lengths[name])
        else:
            fallback_paths.append(str(episode.path))
        rows.append({"path": str(episode.path), **asdict(b)})
    summary = {
        "episodes": len(dataset.episodes),
        "clean": clean,
        "fallback": len(fallback_paths),
        "clean_rate": round(clean / max(1, len(dataset.episodes)), 4),
        "phase_frame_stats": {
            name: {
                "mean": round(float(np.mean(values)), 2),
                "std": round(float(np.std(values)), 2),
                "min": int(np.min(values)),
                "max": int(np.max(values)),
            }
            for name, values in per_phase_frames.items()
            if values
        },
        "fallback_paths": fallback_paths,
    }
    return {"summary": summary, "episodes": rows}
