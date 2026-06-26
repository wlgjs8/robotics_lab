from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Mapping


ARMS = ("left", "right")
JOINT_LABELS = ("J1", "J2", "J3", "J4", "J5", "J6")
TRACE_KEYS = ("sent", "ref", "actual")


@dataclass(frozen=True)
class ArmScopeSample:
    time_s: float
    q_sent_deg: tuple[float, ...]
    q_ref_deg: tuple[float, ...]
    q_actual_deg: tuple[float, ...]


@dataclass(frozen=True)
class SignalTraces:
    time_s: tuple[float, ...]
    sent: tuple[float, ...]
    ref: tuple[float, ...]
    actual: tuple[float, ...]


@dataclass(frozen=True)
class ArmDebugSeries:
    position: SignalTraces
    velocity: SignalTraces
    acceleration: SignalTraces
    jerk: SignalTraces
    rms_sent_actual_deg: float | None
    max_abs_jerk_deg_s3: float | None
    max_abs_jerk_by_trace_deg_s3: Mapping[str, float | None]
    sample_rate_hz: float | None
    sample_count: int


@dataclass(frozen=True)
class DebugPlotSnapshot:
    arms: Mapping[str, ArmDebugSeries]


def finite_difference(values: Iterable[float], times_s: Iterable[float]) -> tuple[float, ...]:
    """Backward finite difference using measured sample intervals, never fixed dt."""

    value_list = tuple(float(value) for value in values)
    time_list = tuple(float(time_s) for time_s in times_s)
    if len(value_list) != len(time_list):
        raise ValueError("values and times_s must have the same length")
    if not value_list:
        return ()
    out = [math.nan] * len(value_list)
    for index in range(1, len(value_list)):
        prev = value_list[index - 1]
        cur = value_list[index]
        dt = time_list[index] - time_list[index - 1]
        if dt > 0.0 and math.isfinite(prev) and math.isfinite(cur):
            out[index] = (cur - prev) / dt
    return tuple(out)


def moving_average(values: Iterable[float], window: int) -> tuple[float, ...]:
    """Trailing display smoothing; NaN gaps are ignored rather than filled."""

    value_list = tuple(float(value) for value in values)
    width = max(1, int(window))
    if width <= 1 or not value_list:
        return value_list
    out: list[float] = []
    for index in range(len(value_list)):
        start = max(0, index - width + 1)
        finite = [
            value for value in value_list[start : index + 1] if math.isfinite(value)
        ]
        out.append(sum(finite) / len(finite) if finite else math.nan)
    return tuple(out)


def _windowed(
    samples: Iterable[ArmScopeSample], window_sec: float
) -> tuple[ArmScopeSample, ...]:
    items = tuple(samples)
    if not items:
        return ()
    end_time = items[-1].time_s
    start_time = end_time - max(0.0, float(window_sec))
    return tuple(sample for sample in items if sample.time_s >= start_time)


def _selected(values: Iterable[tuple[float, ...]], joint_index: int) -> tuple[float, ...]:
    out: list[float] = []
    for item in values:
        if 0 <= joint_index < len(item):
            out.append(item[joint_index])
        else:
            out.append(math.nan)
    return tuple(out)


def _rms_error(sent: tuple[float, ...], actual: tuple[float, ...]) -> float | None:
    errors = [
        sent_value - actual_value
        for sent_value, actual_value in zip(sent, actual)
        if math.isfinite(sent_value) and math.isfinite(actual_value)
    ]
    if not errors:
        return None
    return math.sqrt(sum(error * error for error in errors) / len(errors))


def _max_abs(values: Iterable[float]) -> float | None:
    finite = [abs(value) for value in values if math.isfinite(value)]
    return max(finite) if finite else None


def _sample_rate(times_s: tuple[float, ...]) -> float | None:
    if len(times_s) < 2:
        return None
    duration = times_s[-1] - times_s[0]
    if duration <= 0.0:
        return None
    return (len(times_s) - 1) / duration


def _trace_values(
    samples: tuple[ArmScopeSample, ...], joint_index: int
) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
    sent_q = _selected((sample.q_sent_deg for sample in samples), joint_index)
    ref_q = _selected((sample.q_ref_deg for sample in samples), joint_index)
    actual_q = _selected((sample.q_actual_deg for sample in samples), joint_index)
    return sent_q, ref_q, actual_q


def _make_traces(
    x: tuple[float, ...],
    sent: tuple[float, ...],
    ref: tuple[float, ...],
    actual: tuple[float, ...],
    *,
    smooth: bool,
    smoothing_window: int,
) -> SignalTraces:
    if smooth:
        sent = moving_average(sent, smoothing_window)
        ref = moving_average(ref, smoothing_window)
        actual = moving_average(actual, smoothing_window)
    return SignalTraces(time_s=x, sent=sent, ref=ref, actual=actual)


def build_arm_series(
    samples: Iterable[ArmScopeSample],
    *,
    joint_index: int,
    window_sec: float,
    smooth: bool,
    smoothing_window: int = 5,
) -> ArmDebugSeries:
    """Build q/v/a/jerk for one arm and joint from the 500 Hz scope stream.

    The scope channel carries dense 500 Hz q_sent/q_ref/q_actual samples in
    100 Hz UDP batches. Derivatives still use the measured t_host_ns interval:
    packet jitter or drops must not be hidden behind a fixed-dt assumption.
    Third derivative is display-only and noisy; the smoothing toggle is meant
    for visualization, while server servo_log_*.csv remains the high-fidelity
    source for offline analysis.
    """

    windowed = _windowed(samples, window_sec)
    if not windowed:
        empty = SignalTraces((), (), (), ())
        return ArmDebugSeries(
            position=empty,
            velocity=empty,
            acceleration=empty,
            jerk=empty,
            rms_sent_actual_deg=None,
            max_abs_jerk_deg_s3=None,
            max_abs_jerk_by_trace_deg_s3={key: None for key in TRACE_KEYS},
            sample_rate_hz=None,
            sample_count=0,
        )

    times = tuple(sample.time_s for sample in windowed)
    end_time = times[-1]
    x = tuple(time_s - end_time for time_s in times)
    sent_q, ref_q, actual_q = _trace_values(windowed, joint_index)

    sent_v = finite_difference(sent_q, times)
    ref_v = finite_difference(ref_q, times)
    actual_v = finite_difference(actual_q, times)
    sent_a = finite_difference(sent_v, times)
    ref_a = finite_difference(ref_v, times)
    actual_a = finite_difference(actual_v, times)
    sent_j = finite_difference(sent_a, times)
    ref_j = finite_difference(ref_a, times)
    actual_j = finite_difference(actual_a, times)

    position = _make_traces(
        x, sent_q, ref_q, actual_q, smooth=smooth, smoothing_window=smoothing_window
    )
    velocity = _make_traces(
        x, sent_v, ref_v, actual_v, smooth=smooth, smoothing_window=smoothing_window
    )
    acceleration = _make_traces(
        x, sent_a, ref_a, actual_a, smooth=smooth, smoothing_window=smoothing_window
    )
    jerk = _make_traces(
        x, sent_j, ref_j, actual_j, smooth=smooth, smoothing_window=smoothing_window
    )

    jerk_by_trace = {
        "sent": _max_abs(jerk.sent),
        "ref": _max_abs(jerk.ref),
        "actual": _max_abs(jerk.actual),
    }
    max_values = [value for value in jerk_by_trace.values() if value is not None]
    return ArmDebugSeries(
        position=position,
        velocity=velocity,
        acceleration=acceleration,
        jerk=jerk,
        rms_sent_actual_deg=_rms_error(position.sent, position.actual),
        max_abs_jerk_deg_s3=max(max_values) if max_values else None,
        max_abs_jerk_by_trace_deg_s3=jerk_by_trace,
        sample_rate_hz=_sample_rate(times),
        sample_count=len(times),
    )


def build_debug_snapshot(
    samples_by_arm: Mapping[str, Iterable[ArmScopeSample]],
    *,
    joint_index: int,
    window_sec: float,
    smooth: bool,
    smoothing_window: int = 5,
) -> DebugPlotSnapshot:
    return DebugPlotSnapshot(
        arms={
            arm: build_arm_series(
                samples_by_arm.get(arm, ()),
                joint_index=joint_index,
                window_sec=window_sec,
                smooth=smooth,
                smoothing_window=smoothing_window,
            )
            for arm in ARMS
        }
    )
