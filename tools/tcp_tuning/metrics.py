from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import numpy as np
from scipy.signal import correlate, correlation_lags, periodogram
from scipy.spatial.transform import Rotation

from .config import MetricsConfig
from . import se3


ARMS = ("left", "right")
POSE_WIDTH = 7


def tracking_metrics(
    t: np.ndarray,
    *,
    actual_tcp: np.ndarray | None = None,
    reference_after_B: np.ndarray | None = None,
    conditioned_goal: np.ndarray | None = None,
    source_raw_target: np.ndarray | None = None,
    cfg: MetricsConfig | None = None,
) -> dict[str, Any]:
    """Compute pose tracking/conditioning errors for the series that exist."""

    config = cfg or MetricsConfig()
    out: dict[str, Any] = {
        "actual_tcp_vs_reference_after_B": pose_error_metrics(
            actual_tcp,
            reference_after_B,
            metric_name="actual_tcp_vs_reference_after_B",
        ),
        "actual_tcp_vs_conditioned_goal": pose_error_metrics(
            actual_tcp,
            conditioned_goal,
            metric_name="actual_tcp_vs_conditioned_goal",
        ),
        "reference_after_B_vs_conditioned_goal": pose_error_metrics(
            reference_after_B,
            conditioned_goal,
            metric_name="reference_after_B_vs_conditioned_goal",
        ),
        "source_raw_target_vs_conditioned_goal": pose_error_metrics(
            source_raw_target,
            conditioned_goal,
            metric_name="source_raw_target_vs_conditioned_goal",
        ),
    }
    out["actual_tcp_lag_vs_reference_after_B"] = cross_correlation_lag(
        t,
        reference_after_B,
        actual_tcp,
        max_lag_sec=config.lag_max_sec,
        metric_name="actual_tcp_lag_vs_reference_after_B",
    )
    out["actual_tcp_lag_vs_conditioned_goal"] = cross_correlation_lag(
        t,
        conditioned_goal,
        actual_tcp,
        max_lag_sec=config.lag_max_sec,
        metric_name="actual_tcp_lag_vs_conditioned_goal",
    )
    return out


def pose_error_metrics(lhs: np.ndarray | None, rhs: np.ndarray | None, *, metric_name: str = "pose_error") -> dict[str, Any]:
    """Return position/orientation error stats between two pose series.

    Orientation error is the norm of ``so3_log(R_lhs * R_rhs.inv())`` and never
    raw quaternion subtraction.
    """

    prepared = _paired_pose_series(lhs, rhs, metric_name)
    if prepared["status"] != "ok":
        return prepared
    lhs_arr = prepared["lhs"]
    rhs_arr = prepared["rhs"]
    pos_error = np.linalg.norm(lhs_arr[:, :3] - rhs_arr[:, :3], axis=1)
    ori_error = _orientation_error(lhs_arr[:, 3:7], rhs_arr[:, 3:7])
    return {
        "status": "ok",
        "sample_count": int(pos_error.size),
        "position_m": _stats(pos_error),
        "orientation_rad": _stats(ori_error),
        "notes": [],
    }


def cross_correlation_lag(
    t: np.ndarray,
    reference_pose: np.ndarray | None,
    actual_pose: np.ndarray | None,
    *,
    max_lag_sec: float,
    metric_name: str = "cross_correlation_lag",
) -> dict[str, Any]:
    """Estimate actual-vs-reference lag from TCP position-speed correlation."""

    prepared = _paired_pose_series(reference_pose, actual_pose, metric_name)
    if prepared["status"] != "ok":
        return prepared
    times = _as_time(t)
    ref = prepared["lhs"]
    actual = prepared["rhs"]
    if times.size != ref.shape[0]:
        return _null(metric_name, "time length does not match paired pose length")
    if times.size < 4:
        return _null(metric_name, "at least four samples are required for lag estimation")
    dt = _median_dt(times)
    if dt is None:
        return _null(metric_name, "time must contain positive finite intervals")
    ref_speed = np.linalg.norm(np.diff(ref[:, :3], axis=0), axis=1) / dt
    actual_speed = np.linalg.norm(np.diff(actual[:, :3], axis=0), axis=1) / dt
    valid = np.isfinite(ref_speed) & np.isfinite(actual_speed)
    if int(np.count_nonzero(valid)) < 3:
        return _null(metric_name, "not enough finite speed samples for lag estimation")
    ref_signal = ref_speed[valid] - float(np.mean(ref_speed[valid]))
    actual_signal = actual_speed[valid] - float(np.mean(actual_speed[valid]))
    if _is_degenerate(ref_signal) or _is_degenerate(actual_signal):
        return _null(metric_name, "speed signal is constant or near-zero")
    corr = correlate(actual_signal, ref_signal, mode="full")
    lags = correlation_lags(actual_signal.size, ref_signal.size, mode="full")
    max_lag_samples = max(1, int(round(float(max_lag_sec) / dt)))
    allowed = np.abs(lags) <= max_lag_samples
    if not np.any(allowed):
        return _null(metric_name, "no lags inside configured lag window")
    local = np.argmax(corr[allowed])
    best_lag_samples = int(lags[allowed][local])
    denom = float(np.linalg.norm(ref_signal) * np.linalg.norm(actual_signal))
    corr_value = float(corr[allowed][local] / denom) if denom > 0.0 else np.nan
    return {
        "status": "ok",
        "lag_sec": float(best_lag_samples * dt),
        "lag_samples": best_lag_samples,
        "correlation": corr_value,
        "notes": [],
    }


def smoothness_metrics(
    t: np.ndarray,
    pose: np.ndarray | None,
    *,
    cfg: MetricsConfig | None = None,
    policy_rate_hz: float | None = None,
    metric_name: str = "smoothness",
) -> dict[str, Any]:
    """Compute TCP linear/angular smoothness from a pose series and timestamps."""

    config = cfg or MetricsConfig()
    poses = _pose_series(pose)
    if poses is None:
        return _null(metric_name, "pose series is missing or contains no fully finite rows")
    times = _as_time(t)
    if times.size != poses.shape[0]:
        return _null(metric_name, "time length does not match pose length")
    valid = np.isfinite(times) & np.isfinite(poses).all(axis=1)
    times = times[valid]
    poses = poses[valid]
    if poses.shape[0] < 4:
        return _null(metric_name, "at least four finite pose samples are required")
    derivatives = pose_derivatives(times, poses)
    if derivatives["status"] != "ok":
        return derivatives
    linear_vel = derivatives["linear_velocity_m_s"]
    angular_vel = derivatives["angular_velocity_rad_s"]
    linear_acc = derivatives["linear_acceleration_m_s2"]
    angular_acc = derivatives["angular_acceleration_rad_s2"]
    linear_jerk = derivatives["linear_jerk_m_s3"]
    angular_jerk = derivatives["angular_jerk_rad_s3"]
    rate = float(policy_rate_hz if policy_rate_hz is not None else config.chunk_rate_hz)
    return {
        "status": "ok",
        "sample_count": int(poses.shape[0]),
        "linear_velocity_m_s": _vector_norm_stats(linear_vel),
        "linear_acceleration_m_s2": _vector_norm_stats(linear_acc),
        "linear_jerk_m_s3": _vector_norm_stats(linear_jerk),
        "angular_velocity_rad_s": _vector_norm_stats(angular_vel),
        "angular_acceleration_rad_s2": _vector_norm_stats(angular_acc),
        "angular_jerk_rad_s3": _vector_norm_stats(angular_jerk),
        "linear_velocity_spectrum": spectral_metrics(
            derivatives["velocity_time"],
            linear_vel,
            high_frequency_cutoff_hz=config.high_frequency_cutoff_hz,
            policy_rate_hz=rate,
            near_rate_half_width_hz=config.spectral_peak_near_rate_half_width_hz,
            metric_name=f"{metric_name}.linear_velocity_spectrum",
        ),
        "angular_velocity_spectrum": spectral_metrics(
            derivatives["velocity_time"],
            angular_vel,
            high_frequency_cutoff_hz=config.high_frequency_cutoff_hz,
            policy_rate_hz=rate,
            near_rate_half_width_hz=config.spectral_peak_near_rate_half_width_hz,
            metric_name=f"{metric_name}.angular_velocity_spectrum",
        ),
        "linear_velocity_sign_reversals_per_sec": sign_reversals_per_second(
            derivatives["velocity_time"],
            linear_vel,
            deadband=config.velocity_sign_deadband,
        ),
        "angular_velocity_sign_reversals_per_sec": sign_reversals_per_second(
            derivatives["velocity_time"],
            angular_vel,
            deadband=config.velocity_sign_deadband,
        ),
        "notes": [],
    }


def pose_derivatives(t: np.ndarray, pose: np.ndarray) -> dict[str, Any]:
    """Return linear/angular velocity, acceleration, and jerk arrays."""

    times = _as_time(t)
    poses = np.asarray(pose, dtype=np.float64)
    if poses.ndim != 2 or poses.shape[1] != POSE_WIDTH:
        return _null("pose_derivatives", "pose must have shape (N, 7)")
    dt = np.diff(times)
    if dt.size == 0 or not np.all(np.isfinite(dt)) or np.any(dt <= 0.0):
        return _null("pose_derivatives", "time must be strictly increasing and finite")
    linear_vel = np.empty((poses.shape[0] - 1, 3), dtype=np.float64)
    angular_vel = np.empty((poses.shape[0] - 1, 3), dtype=np.float64)
    for index, step in enumerate(dt):
        try:
            v, w = se3.twist_from_poses(
                poses[index, :3],
                poses[index, 3:7],
                poses[index + 1, :3],
                poses[index + 1, 3:7],
                step,
            )
        except ValueError:
            v = np.full(3, np.nan, dtype=np.float64)
            w = np.full(3, np.nan, dtype=np.float64)
        linear_vel[index] = v
        angular_vel[index] = w
    velocity_time = 0.5 * (times[:-1] + times[1:])
    linear_acc, acc_time = _differentiate(velocity_time, linear_vel)
    angular_acc, _ = _differentiate(velocity_time, angular_vel)
    linear_jerk, jerk_time = _differentiate(acc_time, linear_acc)
    angular_jerk, _ = _differentiate(acc_time, angular_acc)
    return {
        "status": "ok",
        "velocity_time": velocity_time,
        "acceleration_time": acc_time,
        "jerk_time": jerk_time,
        "linear_velocity_m_s": linear_vel,
        "angular_velocity_rad_s": angular_vel,
        "linear_acceleration_m_s2": linear_acc,
        "angular_acceleration_rad_s2": angular_acc,
        "linear_jerk_m_s3": linear_jerk,
        "angular_jerk_rad_s3": angular_jerk,
    }


def spectral_metrics(
    t: np.ndarray,
    values: np.ndarray,
    *,
    high_frequency_cutoff_hz: float,
    policy_rate_hz: float | None = None,
    near_rate_half_width_hz: float = 2.0,
    metric_name: str = "spectrum",
) -> dict[str, Any]:
    """Return one-sided PSD metrics for vector time series."""

    times = _as_time(t)
    arr = np.asarray(values, dtype=np.float64)
    if arr.ndim == 1:
        arr = arr.reshape(-1, 1)
    if arr.ndim != 2 or times.size != arr.shape[0]:
        return _null(metric_name, "time and value lengths must match")
    valid = np.isfinite(times) & np.isfinite(arr).all(axis=1)
    times = times[valid]
    arr = arr[valid]
    if arr.shape[0] < 4:
        return _null(metric_name, "at least four finite samples are required")
    dt = _median_dt(times)
    if dt is None:
        return _null(metric_name, "time must contain positive finite intervals")
    fs = 1.0 / dt
    demeaned = arr - np.mean(arr, axis=0, keepdims=True)
    if _is_degenerate(demeaned.reshape(-1)):
        return {
            "status": "ok",
            "sample_count": int(arr.shape[0]),
            "sample_rate_hz": float(fs),
            "dominant_frequency_hz": None,
            "dominant_power": 0.0,
            "power_above_cutoff": 0.0,
            "power_above_cutoff_ratio": 0.0,
            "peak_near_policy_rate_hz": None,
            "peak_near_policy_power": 0.0,
            "notes": ["signal is constant or near-zero"],
        }
    freqs, psd = periodogram(demeaned, fs=fs, axis=0, scaling="density")
    summed = np.sum(psd, axis=1)
    total_power = _integrate_power(freqs, summed, np.ones_like(freqs, dtype=bool))
    high_mask = freqs > float(high_frequency_cutoff_hz)
    high_power = _integrate_power(freqs, summed, high_mask)
    positive = freqs > 0.0
    if np.any(positive):
        peak_index = int(np.argmax(summed[positive]))
        peak_freq = float(freqs[positive][peak_index])
        peak_power = float(summed[positive][peak_index])
    else:
        peak_freq = None
        peak_power = 0.0
    near_freq = None
    near_power = 0.0
    if policy_rate_hz is not None and float(policy_rate_hz) > 0.0:
        near = np.abs(freqs - float(policy_rate_hz)) <= float(near_rate_half_width_hz)
        near &= freqs > 0.0
        if np.any(near):
            local = int(np.argmax(summed[near]))
            near_freq = float(freqs[near][local])
            near_power = float(summed[near][local])
    return {
        "status": "ok",
        "sample_count": int(arr.shape[0]),
        "sample_rate_hz": float(fs),
        "dominant_frequency_hz": peak_freq,
        "dominant_power": peak_power,
        "power_above_cutoff": float(high_power),
        "power_above_cutoff_ratio": float(high_power / total_power) if total_power > 0.0 else 0.0,
        "peak_near_policy_rate_hz": near_freq,
        "peak_near_policy_power": near_power,
        "notes": [],
    }


def sign_reversals_per_second(t: np.ndarray, values: np.ndarray, *, deadband: float) -> dict[str, Any]:
    """Count per-axis velocity sign transitions per second.

    A zero-velocity plateau between nonzero samples is counted as a transition.
    That makes ZOH start/stop chatter visible, while the configured deadband
    prevents numerical noise from dominating the count.
    """

    times = _as_time(t)
    arr = np.asarray(values, dtype=np.float64)
    if arr.ndim == 1:
        arr = arr.reshape(-1, 1)
    if arr.ndim != 2 or times.size != arr.shape[0]:
        return _null("sign_reversals_per_second", "time and value lengths must match")
    valid = np.isfinite(times) & np.isfinite(arr).all(axis=1)
    times = times[valid]
    arr = arr[valid]
    if arr.shape[0] < 2:
        return _null("sign_reversals_per_second", "at least two finite samples are required")
    count = 0
    by_axis: list[int] = []
    for axis in range(arr.shape[1]):
        signs = np.sign(arr[:, axis])
        signs[np.abs(arr[:, axis]) <= float(deadband)] = 0.0
        axis_count = int(np.count_nonzero(signs[1:] != signs[:-1])) if signs.size >= 2 else 0
        by_axis.append(axis_count)
        count += axis_count
    duration = float(times[-1] - times[0])
    if duration <= 0.0 or not np.isfinite(duration):
        return _null("sign_reversals_per_second", "duration must be positive")
    return {
        "status": "ok",
        "count": int(count),
        "per_sec": float(count / duration),
        "by_axis": by_axis,
        "duration_sec": duration,
        "notes": [],
    }


def health_metrics(t: np.ndarray | None, columns: Mapping[str, Any], *, cfg: MetricsConfig | None = None) -> dict[str, Any]:
    """Compute log-health metrics from whichever scalar/vector columns exist."""

    config = cfg or MetricsConfig()
    out: dict[str, Any] = {}
    ik = _optional_array(columns.get("ik_solve_us"))
    if ik is None:
        out["ik_solve_us"] = _null("ik_solve_us", "ik_solve_us column is absent")
    else:
        out["ik_solve_us"] = {
            "status": "ok",
            "p50": _percentile(ik, 50.0),
            "p95": _percentile(ik, 95.0),
            "max": _max(ik),
            "notes": [],
        }
    out["ik_failures"] = _count_failures(columns)
    for key in ("branch_jump_flag", "safety_proj_flag", "roi_flag", "floor_flag", "self_collision_flag"):
        values = _optional_array(columns.get(key))
        out[key.replace("_flag", "") + "_count"] = (
            _null(key, f"{key} column is absent")
            if values is None
            else {"status": "ok", "count": int(np.count_nonzero(values.astype(bool))), "notes": []}
        )
    out["q_target_vs_q_actual_lag"] = joint_lag(
        t,
        columns.get("q_target"),
        columns.get("q_actual"),
        max_lag_sec=config.lag_max_sec,
    )
    return out


def joint_lag(t: np.ndarray | None, q_target: Any, q_actual: Any, *, max_lag_sec: float) -> dict[str, Any]:
    if t is None:
        return _null("q_target_vs_q_actual_lag", "time column is absent")
    target = _matrix_or_none(q_target, 6)
    actual = _matrix_or_none(q_actual, 6)
    if target is None:
        return _null("q_target_vs_q_actual_lag", "q_target column is absent or non-finite")
    if actual is None:
        return _null("q_target_vs_q_actual_lag", "q_actual column is absent or non-finite")
    times = _as_time(t)
    if times.size != target.shape[0] or times.size != actual.shape[0]:
        return _null("q_target_vs_q_actual_lag", "time and joint series lengths must match")
    dt = _median_dt(times)
    if dt is None:
        return _null("q_target_vs_q_actual_lag", "time must contain positive finite intervals")
    target_signal = np.linalg.norm(target - np.nanmean(target, axis=0), axis=1)
    actual_signal = np.linalg.norm(actual - np.nanmean(actual, axis=0), axis=1)
    valid = np.isfinite(target_signal) & np.isfinite(actual_signal)
    if int(np.count_nonzero(valid)) < 3:
        return _null("q_target_vs_q_actual_lag", "not enough finite joint samples")
    target_signal = target_signal[valid] - float(np.mean(target_signal[valid]))
    actual_signal = actual_signal[valid] - float(np.mean(actual_signal[valid]))
    if _is_degenerate(target_signal) or _is_degenerate(actual_signal):
        return _null("q_target_vs_q_actual_lag", "joint signal is constant or near-zero")
    corr = correlate(actual_signal, target_signal, mode="full")
    lags = correlation_lags(actual_signal.size, target_signal.size, mode="full")
    max_lag_samples = max(1, int(round(float(max_lag_sec) / dt)))
    allowed = np.abs(lags) <= max_lag_samples
    local = int(np.argmax(corr[allowed]))
    lag_samples = int(lags[allowed][local])
    denom = float(np.linalg.norm(target_signal) * np.linalg.norm(actual_signal))
    return {
        "status": "ok",
        "lag_sec": float(lag_samples * dt),
        "lag_samples": lag_samples,
        "correlation": float(corr[allowed][local] / denom) if denom > 0.0 else np.nan,
        "notes": [],
    }


def collect_null_metrics(metrics: Any, prefix: str = "") -> list[dict[str, str]]:
    """Return all null metric paths and reasons from a nested metrics mapping."""

    out: list[dict[str, str]] = []
    if isinstance(metrics, Mapping):
        for key, value in metrics.items():
            path = f"{prefix}.{key}" if prefix else str(key)
            if isinstance(value, Mapping) and value.get("status") == "null":
                out.append({"path": path, "reason": str(value.get("reason", "unknown"))})
            out.extend(collect_null_metrics(value, path))
    elif isinstance(metrics, list):
        for index, value in enumerate(metrics):
            path = f"{prefix}[{index}]" if prefix else f"[{index}]"
            out.extend(collect_null_metrics(value, path))
    return out


def _paired_pose_series(lhs: np.ndarray | None, rhs: np.ndarray | None, metric_name: str) -> dict[str, Any]:
    lhs_arr = _pose_series(lhs)
    rhs_arr = _pose_series(rhs)
    if lhs_arr is None:
        return _null(metric_name, "left-hand pose series is missing or contains no fully finite rows")
    if rhs_arr is None:
        return _null(metric_name, "right-hand pose series is missing or contains no fully finite rows")
    count = min(lhs_arr.shape[0], rhs_arr.shape[0])
    lhs_arr = lhs_arr[:count]
    rhs_arr = rhs_arr[:count]
    valid = np.isfinite(lhs_arr).all(axis=1) & np.isfinite(rhs_arr).all(axis=1)
    if not np.any(valid):
        return _null(metric_name, "paired pose series have no overlapping finite rows")
    return {"status": "ok", "lhs": lhs_arr[valid], "rhs": rhs_arr[valid]}


def _pose_series(value: np.ndarray | None) -> np.ndarray | None:
    if value is None:
        return None
    arr = np.asarray(value, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] != POSE_WIDTH:
        return None
    if not np.any(np.isfinite(arr).all(axis=1)):
        return None
    return arr


def _orientation_error(lhs_quat: np.ndarray, rhs_quat: np.ndarray) -> np.ndarray:
    out = np.empty(lhs_quat.shape[0], dtype=np.float64)
    ref_lhs = None
    ref_rhs = None
    for index, (lhs, rhs) in enumerate(zip(lhs_quat, rhs_quat)):
        q_lhs = se3.quat_canonical(lhs, ref=ref_lhs)
        q_rhs = se3.quat_canonical(rhs, ref=ref_rhs)
        ref_lhs = q_lhs
        ref_rhs = q_rhs
        relative = Rotation.from_quat(q_lhs) * Rotation.from_quat(q_rhs).inv()
        out[index] = float(np.linalg.norm(se3.so3_log(relative)))
    return out


def _differentiate(t: np.ndarray, values: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    if values.shape[0] < 2:
        return np.empty((0, values.shape[1]), dtype=np.float64), np.empty(0, dtype=np.float64)
    dt = np.diff(t).reshape(-1, 1)
    out = np.divide(np.diff(values, axis=0), dt, out=np.full((values.shape[0] - 1, values.shape[1]), np.nan), where=dt > 0.0)
    return out.astype(np.float64), (0.5 * (t[:-1] + t[1:])).astype(np.float64)


def _stats(values: np.ndarray) -> dict[str, float | None]:
    arr = np.asarray(values, dtype=np.float64)
    arr = arr[np.isfinite(arr)]
    if arr.size == 0:
        return {"rms": None, "p95": None, "max": None}
    return {
        "rms": float(np.sqrt(np.mean(np.square(arr)))),
        "p95": _percentile(arr, 95.0),
        "max": _max(arr),
    }


def _vector_norm_stats(values: np.ndarray) -> dict[str, float | None]:
    arr = np.asarray(values, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[0] == 0:
        return {"rms": None, "p95": None, "max": None}
    return _stats(np.linalg.norm(arr, axis=1))


def _integrate_power(freqs: np.ndarray, power: np.ndarray, mask: np.ndarray) -> float:
    if int(np.count_nonzero(mask)) < 2:
        return 0.0
    return float(np.trapezoid(power[mask], freqs[mask]))


def _count_failures(columns: Mapping[str, Any]) -> dict[str, Any]:
    failure = np.zeros(0, dtype=bool)
    for key in ("ik_failure_flag", "ik_failed", "ik_fail_flag"):
        values = _optional_array(columns.get(key))
        if values is not None:
            failure = values.astype(bool)
            break
    if failure.size == 0:
        pos = _optional_array(columns.get("ik_pos_err"))
        ori = _optional_array(columns.get("ik_ori_err"))
        if pos is None and ori is None:
            return _null("ik_failures", "IK failure/error columns are absent")
        pieces = []
        if pos is not None:
            pieces.append(~np.isfinite(pos))
        if ori is not None:
            pieces.append(~np.isfinite(ori))
        failure = np.logical_or.reduce(pieces)
    return {"status": "ok", "count": int(np.count_nonzero(failure)), "notes": []}


def _optional_array(value: Any) -> np.ndarray | None:
    if value is None:
        return None
    arr = np.asarray(value, dtype=np.float64).reshape(-1)
    finite_or_bool = np.isfinite(arr)
    if arr.size == 0 or not np.any(finite_or_bool):
        return None
    return arr


def _matrix_or_none(value: Any, width: int) -> np.ndarray | None:
    if value is None:
        return None
    arr = np.asarray(value, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] != width:
        return None
    if not np.any(np.isfinite(arr).all(axis=1)):
        return None
    return arr


def _as_time(value: np.ndarray | None) -> np.ndarray:
    if value is None:
        return np.empty(0, dtype=np.float64)
    return np.asarray(value, dtype=np.float64).reshape(-1)


def _median_dt(t: np.ndarray) -> float | None:
    dt = np.diff(np.asarray(t, dtype=np.float64).reshape(-1))
    dt = dt[np.isfinite(dt) & (dt > 0.0)]
    if dt.size == 0:
        return None
    return float(np.median(dt))


def _is_degenerate(values: np.ndarray) -> bool:
    arr = np.asarray(values, dtype=np.float64)
    arr = arr[np.isfinite(arr)]
    return arr.size == 0 or float(np.max(np.abs(arr))) <= np.finfo(np.float64).eps


def _percentile(values: np.ndarray, percentile: float) -> float | None:
    arr = np.asarray(values, dtype=np.float64)
    arr = arr[np.isfinite(arr)]
    if arr.size == 0:
        return None
    return float(np.percentile(arr, percentile))


def _max(values: np.ndarray) -> float | None:
    arr = np.asarray(values, dtype=np.float64)
    arr = arr[np.isfinite(arr)]
    if arr.size == 0:
        return None
    return float(np.max(arr))


def _null(metric_name: str, reason: str) -> dict[str, Any]:
    return {"status": "null", "value": None, "reason": reason, "notes": [reason], "metric": metric_name}
