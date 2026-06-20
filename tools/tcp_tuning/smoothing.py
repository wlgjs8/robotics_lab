from __future__ import annotations

from collections.abc import Iterable
from typing import Any

import numpy as np
from scipy.interpolate import UnivariateSpline
from scipy.signal import butter, savgol_filter, sosfiltfilt
from scipy.spatial.transform import Rotation

from .config import SmoothingConfig
from .se3 import quat_canonical


Segment = tuple[int, int]


def split_segments(
    t: np.ndarray,
    *,
    median_multiplier: float,
    absolute_threshold_sec: float,
) -> tuple[list[Segment], np.ndarray]:
    """Split source samples at timestamp gaps.

    Segments are half-open source-index ranges. Gaps are `(t_before, t_after)`.
    A split is inserted when `dt > median_multiplier * median_positive_dt` or
    `dt > absolute_threshold_sec`.
    """

    times = np.asarray(t, dtype=np.float64).reshape(-1)
    if times.size == 0:
        return [], np.empty((0, 2), dtype=np.float64)
    if times.size == 1:
        return [(0, 1)], np.empty((0, 2), dtype=np.float64)
    dt = np.diff(times)
    positive = dt[np.isfinite(dt) & (dt > 0.0)]
    median = float(np.median(positive)) if positive.size else 0.0
    split_after: list[int] = []
    gaps: list[tuple[float, float]] = []
    for index, value in enumerate(dt):
        is_gap = not np.isfinite(value) or value <= 0.0
        if np.isfinite(value) and median > 0.0 and value > float(median_multiplier) * median:
            is_gap = True
        if np.isfinite(value) and value > float(absolute_threshold_sec):
            is_gap = True
        if is_gap:
            split_after.append(index)
            gaps.append((float(times[index]), float(times[index + 1])))

    starts = [0] + [item + 1 for item in split_after]
    stops = [item + 1 for item in split_after] + [int(times.size)]
    segments = [(int(start), int(stop)) for start, stop in zip(starts, stops) if stop > start]
    return segments, np.asarray(gaps, dtype=np.float64).reshape(-1, 2)


def smooth_pose_segments(
    t: np.ndarray,
    pose: np.ndarray,
    segments: Iterable[Segment],
    cfg: SmoothingConfig | None = None,
    *,
    endpoint_mode: str = "preserve",
) -> np.ndarray:
    """Smooth `(N,7)` poses inside each segment without crossing gaps."""

    times = np.asarray(t, dtype=np.float64).reshape(-1)
    poses = np.asarray(pose, dtype=np.float64)
    if poses.ndim != 2 or poses.shape[1] != 7:
        raise ValueError("pose must have shape (N, 7)")
    if times.shape[0] != poses.shape[0]:
        raise ValueError("t and pose must have matching length")
    config = cfg or SmoothingConfig()
    out = poses.copy()
    for start, stop in _normalize_segments(segments, len(times)):
        if stop - start <= 1:
            continue
        local_t = times[start:stop]
        local_pose = poses[start:stop]
        out[start:stop, :3] = smooth_position(local_t, local_pose[:, :3], config, endpoint_mode=endpoint_mode)
        out[start:stop, 3:7] = smooth_orientation(local_t, local_pose[:, 3:7], config, endpoint_mode=endpoint_mode)
    return out.astype(np.float64)


def smooth_position(
    t: np.ndarray,
    position: np.ndarray,
    cfg: SmoothingConfig | None = None,
    *,
    endpoint_mode: str = "preserve",
) -> np.ndarray:
    """Smooth positions with the configured method inside a single segment."""

    times = np.asarray(t, dtype=np.float64).reshape(-1)
    values = np.asarray(position, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 3:
        raise ValueError("position must have shape (N, 3)")
    if values.shape[0] != times.size:
        raise ValueError("t and position must have matching length")
    config = cfg or SmoothingConfig()
    smoothed = _smooth_columns(times, values, config)
    return _apply_endpoint_mode(values, smoothed, endpoint_mode)


def smooth_orientation(
    t: np.ndarray,
    quat_xyzw: np.ndarray,
    cfg: SmoothingConfig | None = None,
    *,
    endpoint_mode: str = "preserve",
) -> np.ndarray:
    """Smooth SO(3) by filtering quaternion logs in a segment tangent space."""

    times = np.asarray(t, dtype=np.float64).reshape(-1)
    raw = np.asarray(quat_xyzw, dtype=np.float64)
    if raw.ndim != 2 or raw.shape[1] != 4:
        raise ValueError("quat_xyzw must have shape (N, 4)")
    if raw.shape[0] != times.size:
        raise ValueError("t and quat_xyzw must have matching length")
    if raw.shape[0] <= 1:
        return raw.copy()

    continuous = np.zeros_like(raw, dtype=np.float64)
    ref = None
    for index, quat in enumerate(raw):
        continuous[index] = quat_canonical(quat, ref=ref)
        ref = continuous[index]

    base = Rotation.from_quat(continuous[0])
    rotations = Rotation.from_quat(continuous)
    rotvec = (rotations * base.inv()).as_rotvec()
    config = cfg or SmoothingConfig()
    smooth_rotvec = _smooth_columns(times, rotvec, config)
    out = (Rotation.from_rotvec(smooth_rotvec) * base).as_quat()
    for index in range(out.shape[0]):
        out[index] = quat_canonical(out[index], ref=out[index - 1] if index else continuous[0])
    return _apply_endpoint_mode(continuous, out, endpoint_mode)


def _smooth_columns(t: np.ndarray, values: np.ndarray, cfg: SmoothingConfig) -> np.ndarray:
    if values.shape[0] < int(cfg.segment_min_samples):
        return values.copy()
    method = str(cfg.method).lower()
    if method in {"none", "off", "identity"}:
        return values.copy()
    if method in {"savgol", "savitzky_golay", "savitzky-golay"}:
        return _savgol(values, cfg)
    if method in {"cubic", "cubic_spline", "spline", "smoothing_spline"}:
        return _cubic_spline(t, values, cfg)
    if method in {"lowpass", "low_pass", "butter", "butterworth"}:
        return _lowpass(t, values, cfg)
    raise ValueError(f"unknown smoothing method: {cfg.method}")


def _savgol(values: np.ndarray, cfg: SmoothingConfig) -> np.ndarray:
    count = values.shape[0]
    window = min(int(cfg.window_samples), count)
    if window % 2 == 0:
        window -= 1
    polyorder = min(int(cfg.polyorder), window - 1)
    if window < 3 or polyorder < 1:
        return values.copy()
    return savgol_filter(values, window_length=window, polyorder=polyorder, axis=0, mode="interp").astype(np.float64)


def _cubic_spline(t: np.ndarray, values: np.ndarray, cfg: SmoothingConfig) -> np.ndarray:
    times = _strictly_increasing_time(t)
    out = np.empty_like(values, dtype=np.float64)
    smooth = max(0.0, float(cfg.cubic_smoothing))
    for col in range(values.shape[1]):
        spline = UnivariateSpline(times, values[:, col], k=min(3, values.shape[0] - 1), s=smooth)
        out[:, col] = spline(times)
    return out


def _lowpass(t: np.ndarray, values: np.ndarray, cfg: SmoothingConfig) -> np.ndarray:
    times = _strictly_increasing_time(t)
    dt = np.diff(times)
    median_dt = float(np.median(dt)) if dt.size else 0.0
    if median_dt <= 0.0:
        return values.copy()
    nyquist = 0.5 / median_dt
    cutoff = float(cfg.lowpass_cutoff_hz)
    if cutoff <= 0.0 or cutoff >= nyquist:
        return values.copy()
    if values.shape[0] < 9:
        return values.copy()
    sos = butter(2, cutoff / nyquist, btype="lowpass", output="sos")
    try:
        return sosfiltfilt(sos, values, axis=0).astype(np.float64)
    except ValueError:
        return values.copy()


def _strictly_increasing_time(t: np.ndarray) -> np.ndarray:
    times = np.asarray(t, dtype=np.float64).reshape(-1).copy()
    if times.size <= 1:
        return times
    for index in range(1, times.size):
        if not np.isfinite(times[index]) or times[index] <= times[index - 1]:
            times[index] = times[index - 1] + np.finfo(np.float64).eps
    return times


def _apply_endpoint_mode(raw: np.ndarray, smoothed: np.ndarray, endpoint_mode: str) -> np.ndarray:
    mode = str(endpoint_mode).lower()
    out = np.asarray(smoothed, dtype=np.float64).copy()
    if mode in {"preserve", "preserve_segment_ends"} and out.shape[0] >= 1:
        out[0] = raw[0]
        out[-1] = raw[-1]
    elif mode in {"interp", "smooth", "natural"}:
        pass
    else:
        raise ValueError(f"unknown endpoint_mode: {endpoint_mode}")
    return out


def _normalize_segments(segments: Iterable[Segment], length: int) -> list[Segment]:
    out: list[Segment] = []
    for item in segments:
        start = int(item[0])
        stop = int(item[1])
        start = max(0, min(length, start))
        stop = max(0, min(length, stop))
        if stop > start:
            out.append((start, stop))
    if not out and length:
        out.append((0, length))
    return out

