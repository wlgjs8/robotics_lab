"""Pure payload-identification math and report helpers for the CoG GUI.

The server supplies both inputs in the TCP frame: the pre-payload/pre-tare
``wrench_tcp`` and the gravity vector resolved from the measured TCP
orientation.  This module deliberately owns no robot commands and never
applies an estimate to either server configuration or a controller.
"""

from __future__ import annotations

import csv
import io
import json
import math
import os
import shutil
import tempfile
import threading
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


_NUMERIC_EPS = np.finfo(float).eps


class CogCalibrationError(ValueError):
    """An estimate cannot be produced from the supplied measurements."""

    def __init__(self, code: str, detail: str) -> None:
        super().__init__(detail)
        self.code = code
        self.detail = detail


def _finite_tuple(values: Sequence[float], length: int, field: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != length:
        raise ValueError(f"{field} must contain exactly {length} values")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{field} must contain only finite values")
    return result


@dataclass(frozen=True)
class CogSample:
    """One unique F/T acquisition in the TCP frame."""

    freshness_value: int
    wrench_tcp: tuple[float, ...]
    gravity_tcp: tuple[float, ...]
    received_monotonic: float

    def __post_init__(self) -> None:
        if isinstance(self.freshness_value, bool) or int(self.freshness_value) < 0:
            raise ValueError("freshness_value must be a non-negative integer")
        if int(self.freshness_value) != self.freshness_value:
            raise ValueError("freshness_value must be a non-negative integer")
        object.__setattr__(self, "freshness_value", int(self.freshness_value))
        object.__setattr__(self, "wrench_tcp", _finite_tuple(self.wrench_tcp, 6, "wrench_tcp"))
        object.__setattr__(self, "gravity_tcp", _finite_tuple(self.gravity_tcp, 3, "gravity_tcp"))
        received = float(self.received_monotonic)
        if not math.isfinite(received) or received < 0.0:
            raise ValueError("received_monotonic must be finite and non-negative")
        object.__setattr__(self, "received_monotonic", received)


@dataclass(frozen=True)
class CogPoseSummary:
    name: str
    sample_count: int
    first_freshness_value: int
    last_freshness_value: int
    gravity_tcp_mean: tuple[float, float, float]
    force_tcp_mean_n: tuple[float, float, float]
    torque_tcp_mean_nm: tuple[float, float, float]
    force_stddev_n: tuple[float, float, float]
    torque_stddev_nm: tuple[float, float, float]
    max_force_stddev_n: float
    max_torque_stddev_nm: float


@dataclass(frozen=True)
class CogPoseMeasurement:
    """Immutable samples collected after one waypoint has settled."""

    name: str
    q_target_deg: tuple[float, ...]
    samples: tuple[CogSample, ...]

    def __post_init__(self) -> None:
        name = str(self.name).strip()
        if not name:
            raise ValueError("pose name must not be empty")
        object.__setattr__(self, "name", name)
        object.__setattr__(self, "q_target_deg", _finite_tuple(self.q_target_deg, 6, "q_target_deg"))
        samples = tuple(self.samples)
        if not samples:
            raise ValueError("pose must contain at least one sample")
        if any(not isinstance(sample, CogSample) for sample in samples):
            raise ValueError("samples must contain only CogSample values")
        freshness = [sample.freshness_value for sample in samples]
        if any(current <= previous for previous, current in zip(freshness, freshness[1:])):
            raise ValueError("pose sample freshness values must be strictly increasing")
        object.__setattr__(self, "samples", samples)

    def summary(self) -> CogPoseSummary:
        wrench = np.asarray([sample.wrench_tcp for sample in self.samples], dtype=float)
        gravity = np.asarray([sample.gravity_tcp for sample in self.samples], dtype=float)
        ddof = 1 if len(self.samples) > 1 else 0
        force_stddev = np.std(wrench[:, :3], axis=0, ddof=ddof)
        torque_stddev = np.std(wrench[:, 3:], axis=0, ddof=ddof)
        return CogPoseSummary(
            name=self.name,
            sample_count=len(self.samples),
            first_freshness_value=self.samples[0].freshness_value,
            last_freshness_value=self.samples[-1].freshness_value,
            gravity_tcp_mean=tuple(float(value) for value in np.mean(gravity, axis=0)),
            force_tcp_mean_n=tuple(float(value) for value in np.mean(wrench[:, :3], axis=0)),
            torque_tcp_mean_nm=tuple(float(value) for value in np.mean(wrench[:, 3:], axis=0)),
            force_stddev_n=tuple(float(value) for value in force_stddev),
            torque_stddev_nm=tuple(float(value) for value in torque_stddev),
            max_force_stddev_n=float(np.max(force_stddev)),
            max_torque_stddev_nm=float(np.max(torque_stddev)),
        )


class CogSampleAccumulator:
    """Lock-protected collector that admits each fresh acquisition once."""

    def __init__(self, name: str, q_target_deg: Sequence[float]) -> None:
        name = str(name).strip()
        if not name:
            raise ValueError("pose name must not be empty")
        self._name = name
        self._q_target_deg = _finite_tuple(q_target_deg, 6, "q_target_deg")
        self._samples: list[CogSample] = []
        self._last_freshness_value: int | None = None
        self._lock = threading.Lock()

    def add(self, sample: CogSample) -> bool:
        """Append a strictly newer sample; return false for duplicate/old input."""
        if not isinstance(sample, CogSample):
            raise TypeError("sample must be a CogSample")
        with self._lock:
            if (
                self._last_freshness_value is not None
                and sample.freshness_value <= self._last_freshness_value
            ):
                return False
            self._samples.append(sample)
            self._last_freshness_value = sample.freshness_value
            return True

    @property
    def sample_count(self) -> int:
        with self._lock:
            return len(self._samples)

    @property
    def last_freshness_value(self) -> int | None:
        with self._lock:
            return self._last_freshness_value

    def reset(self) -> None:
        with self._lock:
            self._samples.clear()
            self._last_freshness_value = None

    def freeze(self) -> CogPoseMeasurement:
        with self._lock:
            samples = tuple(self._samples)
        return CogPoseMeasurement(
            name=self._name,
            q_target_deg=self._q_target_deg,
            samples=samples,
        )


@dataclass(frozen=True)
class CogPoseResidual:
    name: str
    force_residual_n: tuple[float, float, float]
    torque_residual_nm: tuple[float, float, float]
    force_residual_norm_n: float
    torque_residual_norm_nm: float


@dataclass(frozen=True)
class CogLeaveOneOutDiagnostic:
    omitted_pose: str
    valid: bool
    error: str | None = None
    mass_kg: float | None = None
    mass_delta_kg: float | None = None
    cog_tcp_m: tuple[float, float, float] | None = None
    cog_delta_norm_m: float | None = None
    force_fit_rms_n: float | None = None
    torque_fit_rms_nm: float | None = None


@dataclass(frozen=True)
class CogEstimate:
    mass_kg: float
    mass_std_error_kg: float
    cog_tcp_m: tuple[float, float, float]
    force_bias_n: tuple[float, float, float]
    torque_bias_nm: tuple[float, float, float]
    force_fit_rms_n: float
    torque_fit_rms_nm: float
    force_design_rank: int
    force_design_condition: float
    torque_design_rank: int
    torque_design_condition: float
    force_gravity_correlation: float | None
    gravity_correlated_force_rms_n: float
    measured_force_variation_rms_n: float
    gravity_compensation_ambiguous: bool
    ambiguity_reasons: tuple[str, ...]
    pose_residuals: tuple[CogPoseResidual, ...]
    leave_one_out: tuple[CogLeaveOneOutDiagnostic, ...]
    provisional: bool = True
    applied: bool = False

    @property
    def max_leave_one_out_mass_delta_kg(self) -> float | None:
        values = [
            item.mass_delta_kg
            for item in self.leave_one_out
            if item.valid and item.mass_delta_kg is not None
        ]
        return max(values, default=None)

    @property
    def max_leave_one_out_cog_delta_m(self) -> float | None:
        values = [
            item.cog_delta_norm_m
            for item in self.leave_one_out
            if item.valid and item.cog_delta_norm_m is not None
        ]
        return max(values, default=None)


@dataclass(frozen=True)
class CogReportPaths:
    report_json: Path
    samples_csv: Path


@dataclass(frozen=True)
class _SolveResult:
    mass_kg: float
    mass_std_error_kg: float
    cog_tcp_m: np.ndarray
    force_bias_n: np.ndarray
    torque_bias_nm: np.ndarray
    force_fit_rms_n: float
    torque_fit_rms_nm: float
    force_design_rank: int
    force_design_condition: float
    torque_design_rank: int
    torque_design_condition: float
    force_gravity_correlation: float | None
    gravity_correlated_force_rms_n: float
    measured_force_variation_rms_n: float
    gravity_compensation_ambiguous: bool
    ambiguity_reasons: tuple[str, ...]
    pose_residuals: tuple[CogPoseResidual, ...]


def _skew(vector: np.ndarray) -> np.ndarray:
    x_value, y_value, z_value = vector
    return np.asarray(
        [[0.0, -z_value, y_value], [z_value, 0.0, -x_value], [-y_value, x_value, 0.0]],
        dtype=float,
    )


def _design_condition(matrix: np.ndarray) -> tuple[int, float]:
    singular_values = np.linalg.svd(matrix, compute_uv=False)
    rank = int(np.linalg.matrix_rank(matrix))
    if rank < matrix.shape[1] or singular_values[-1] <= 0.0:
        return rank, math.inf
    return rank, float(singular_values[0] / singular_values[-1])


def _mean_weights(variance: np.ndarray, counts: np.ndarray) -> np.ndarray:
    """Inverse standard-error weights with only a numerical zero regularizer."""
    standard_error_squared = variance / counts[:, None]
    positive = standard_error_squared[standard_error_squared > 0.0]
    if positive.size == 0:
        weights = np.sqrt(counts)[:, None] * np.ones((1, variance.shape[1]))
    else:
        numerical_floor = max(float(np.min(positive)) * 1e-6, _NUMERIC_EPS)
        weights = 1.0 / np.sqrt(np.maximum(standard_error_squared, numerical_floor))
    median = float(np.median(weights))
    return weights / median if median > 0.0 else np.ones_like(weights)


def _weighted_lstsq(
    design: np.ndarray,
    observations: np.ndarray,
    weights: np.ndarray,
) -> tuple[np.ndarray, int, np.ndarray, float]:
    weighted_design = design * weights[:, None]
    weighted_observations = observations * weights
    coefficients, _residuals, rank, singular_values = np.linalg.lstsq(
        weighted_design,
        weighted_observations,
        rcond=None,
    )
    residual = weighted_observations - weighted_design @ coefficients
    degrees_of_freedom = max(weighted_design.shape[0] - weighted_design.shape[1], 1)
    residual_variance = float(residual @ residual) / degrees_of_freedom
    covariance = residual_variance * np.linalg.pinv(weighted_design.T @ weighted_design)
    return coefficients, int(rank), singular_values, float(math.sqrt(max(covariance[0, 0], 0.0)))


def _solve(
    summaries: Sequence[CogPoseSummary],
    *,
    max_condition_number: float | None,
) -> _SolveResult:
    gravity = np.asarray([summary.gravity_tcp_mean for summary in summaries], dtype=float)
    force = np.asarray([summary.force_tcp_mean_n for summary in summaries], dtype=float)
    torque = np.asarray([summary.torque_tcp_mean_nm for summary in summaries], dtype=float)
    counts = np.asarray([summary.sample_count for summary in summaries], dtype=float)
    force_variance = np.square(
        np.asarray([summary.force_stddev_n for summary in summaries], dtype=float)
    )
    torque_variance = np.square(
        np.asarray([summary.torque_stddev_nm for summary in summaries], dtype=float)
    )

    force_design = np.zeros((3 * len(summaries), 4), dtype=float)
    torque_design = np.zeros((3 * len(summaries), 6), dtype=float)
    for index, gravity_vector in enumerate(gravity):
        row = slice(3 * index, 3 * index + 3)
        force_design[row, 0] = gravity_vector
        force_design[row, 1:] = np.eye(3)
        torque_design[row, :3] = -_skew(gravity_vector)
        torque_design[row, 3:] = np.eye(3)

    raw_force_rank, _raw_force_condition = _design_condition(force_design)
    raw_torque_rank, _raw_torque_condition = _design_condition(torque_design)
    if raw_force_rank < force_design.shape[1] or raw_torque_rank < torque_design.shape[1]:
        raise CogCalibrationError(
            "rank_deficient",
            "gravity orientations do not provide full-rank force and torque designs "
            f"(force {raw_force_rank}/4, torque {raw_torque_rank}/6)",
        )

    force_weights = _mean_weights(force_variance, counts).reshape(-1)
    torque_weights = _mean_weights(torque_variance, counts).reshape(-1)
    weighted_force_design = force_design * force_weights[:, None]
    weighted_torque_design = torque_design * torque_weights[:, None]
    force_rank, force_condition = _design_condition(weighted_force_design)
    torque_rank, torque_condition = _design_condition(weighted_torque_design)
    if force_rank < force_design.shape[1] or torque_rank < torque_design.shape[1]:
        raise CogCalibrationError("rank_deficient", "weighted design is rank deficient")
    if (
        max_condition_number is not None
        and max(force_condition, torque_condition) > max_condition_number
    ):
        raise CogCalibrationError(
            "ill_conditioned",
            "weighted gravity design exceeds explicit condition-number limit "
            f"({max(force_condition, torque_condition):.6g} > {max_condition_number:.6g})",
        )

    force_coefficients, weighted_force_rank, _force_singular, mass_std_error = _weighted_lstsq(
        force_design,
        force.reshape(-1),
        force_weights,
    )
    torque_coefficients, weighted_torque_rank, _torque_singular, _unused = _weighted_lstsq(
        torque_design,
        torque.reshape(-1),
        torque_weights,
    )
    if weighted_force_rank < 4 or weighted_torque_rank < 6:
        raise CogCalibrationError("rank_deficient", "weighted design is rank deficient")
    if not np.all(np.isfinite(force_coefficients)) or not np.all(np.isfinite(torque_coefficients)):
        raise CogCalibrationError("non_finite", "least-squares output contains non-finite values")

    mass = float(force_coefficients[0])
    centered_gravity = gravity - np.mean(gravity, axis=0)
    centered_force = force - np.mean(force, axis=0)
    gravity_norm = float(np.linalg.norm(centered_gravity))
    force_norm = float(np.linalg.norm(centered_force))
    correlation = None
    if gravity_norm > _NUMERIC_EPS and force_norm > _NUMERIC_EPS:
        correlation = float(np.sum(centered_gravity * centered_force) / (gravity_norm * force_norm))
        correlation = float(np.clip(correlation, -1.0, 1.0))
    measured_force_variation_rms = float(np.sqrt(np.mean(np.square(centered_force))))
    gravity_correlated_force_rms = float(np.sqrt(np.mean(np.square(mass * centered_gravity))))

    ambiguity_reasons: list[str] = []
    if force_norm <= _NUMERIC_EPS:
        ambiguity_reasons.append("no measurable force variation across distinct gravity directions")
    if mass_std_error > _NUMERIC_EPS and abs(mass) < 3.0 * mass_std_error:
        ambiguity_reasons.append("gravity-correlated mass signal is below three standard errors")
    if mass <= _NUMERIC_EPS:
        code = "gravity_compensation_ambiguous" if ambiguity_reasons else "non_positive_mass"
        detail = (
            "payload mass is not identifiable from wrench variation; controller output may already be "
            "gravity compensated"
            if ambiguity_reasons
            else f"estimated payload mass must be positive, got {mass:.9g} kg"
        )
        raise CogCalibrationError(code, detail)

    cog = torque_coefficients[:3] / mass
    if not np.all(np.isfinite(cog)):
        raise CogCalibrationError("non_finite", "recovered CoG contains non-finite values")

    predicted_force = mass * gravity + force_coefficients[1:]
    predicted_torque = np.asarray(
        [np.cross(cog, mass * gravity_vector) for gravity_vector in gravity],
        dtype=float,
    ) + torque_coefficients[3:]
    force_residual = force - predicted_force
    torque_residual = torque - predicted_torque
    force_fit_rms = float(np.sqrt(np.mean(np.square(force_residual))))
    torque_fit_rms = float(np.sqrt(np.mean(np.square(torque_residual))))
    if correlation is None or correlation <= 0.0:
        ambiguity_reasons.append(
            "force variation is not positively correlated with measured gravity"
        )
    if gravity_correlated_force_rms <= force_fit_rms:
        ambiguity_reasons.append(
            "gravity-correlated force signal does not exceed force-fit residual"
        )
    pose_residuals = tuple(
        CogPoseResidual(
            name=summary.name,
            force_residual_n=tuple(float(value) for value in force_residual[index]),
            torque_residual_nm=tuple(float(value) for value in torque_residual[index]),
            force_residual_norm_n=float(np.linalg.norm(force_residual[index])),
            torque_residual_norm_nm=float(np.linalg.norm(torque_residual[index])),
        )
        for index, summary in enumerate(summaries)
    )
    return _SolveResult(
        mass_kg=mass,
        mass_std_error_kg=mass_std_error,
        cog_tcp_m=cog,
        force_bias_n=force_coefficients[1:],
        torque_bias_nm=torque_coefficients[3:],
        force_fit_rms_n=force_fit_rms,
        torque_fit_rms_nm=torque_fit_rms,
        force_design_rank=force_rank,
        force_design_condition=force_condition,
        torque_design_rank=torque_rank,
        torque_design_condition=torque_condition,
        force_gravity_correlation=correlation,
        gravity_correlated_force_rms_n=gravity_correlated_force_rms,
        measured_force_variation_rms_n=measured_force_variation_rms,
        gravity_compensation_ambiguous=bool(ambiguity_reasons),
        ambiguity_reasons=tuple(ambiguity_reasons),
        pose_residuals=pose_residuals,
    )


def estimate_payload(
    poses: Sequence[CogPoseMeasurement],
    *,
    min_poses: int = 5,
    max_condition_number: float | None = None,
) -> CogEstimate:
    """Estimate mass, TCP CoG, and constant TCP wrench bias.

    ``max_condition_number`` has no hidden fallback.  Passing ``None`` records
    the measured condition without declaring a physical acceptance threshold.
    Every successful result remains provisional and unapplied.
    """
    if not isinstance(min_poses, int) or isinstance(min_poses, bool) or min_poses < 3:
        raise ValueError("min_poses must be an integer of at least 3")
    if max_condition_number is not None:
        max_condition_number = float(max_condition_number)
        if not math.isfinite(max_condition_number) or max_condition_number <= 1.0:
            raise ValueError("max_condition_number must be finite and greater than 1")
    frozen_poses = tuple(poses)
    if len(frozen_poses) < min_poses:
        raise CogCalibrationError(
            "insufficient_poses",
            f"need at least {min_poses} accepted poses, got {len(frozen_poses)}",
        )
    if any(not isinstance(pose, CogPoseMeasurement) for pose in frozen_poses):
        raise TypeError("poses must contain only CogPoseMeasurement values")
    names = [pose.name for pose in frozen_poses]
    if len(set(names)) != len(names):
        raise CogCalibrationError("duplicate_pose_name", "pose names must be unique")
    summaries = tuple(pose.summary() for pose in frozen_poses)
    solved = _solve(summaries, max_condition_number=max_condition_number)

    leave_one_out: list[CogLeaveOneOutDiagnostic] = []
    for omitted_index, omitted in enumerate(summaries):
        remaining = summaries[:omitted_index] + summaries[omitted_index + 1 :]
        try:
            candidate = _solve(remaining, max_condition_number=max_condition_number)
        except CogCalibrationError as exc:
            leave_one_out.append(
                CogLeaveOneOutDiagnostic(
                    omitted_pose=omitted.name,
                    valid=False,
                    error=f"{exc.code}: {exc.detail}",
                )
            )
            continue
        leave_one_out.append(
            CogLeaveOneOutDiagnostic(
                omitted_pose=omitted.name,
                valid=True,
                mass_kg=candidate.mass_kg,
                mass_delta_kg=abs(candidate.mass_kg - solved.mass_kg),
                cog_tcp_m=tuple(float(value) for value in candidate.cog_tcp_m),
                cog_delta_norm_m=float(np.linalg.norm(candidate.cog_tcp_m - solved.cog_tcp_m)),
                force_fit_rms_n=candidate.force_fit_rms_n,
                torque_fit_rms_nm=candidate.torque_fit_rms_nm,
            )
        )

    return CogEstimate(
        mass_kg=solved.mass_kg,
        mass_std_error_kg=solved.mass_std_error_kg,
        cog_tcp_m=tuple(float(value) for value in solved.cog_tcp_m),
        force_bias_n=tuple(float(value) for value in solved.force_bias_n),
        torque_bias_nm=tuple(float(value) for value in solved.torque_bias_nm),
        force_fit_rms_n=solved.force_fit_rms_n,
        torque_fit_rms_nm=solved.torque_fit_rms_nm,
        force_design_rank=solved.force_design_rank,
        force_design_condition=solved.force_design_condition,
        torque_design_rank=solved.torque_design_rank,
        torque_design_condition=solved.torque_design_condition,
        force_gravity_correlation=solved.force_gravity_correlation,
        gravity_correlated_force_rms_n=solved.gravity_correlated_force_rms_n,
        measured_force_variation_rms_n=solved.measured_force_variation_rms_n,
        gravity_compensation_ambiguous=solved.gravity_compensation_ambiguous,
        ambiguity_reasons=solved.ambiguity_reasons,
        pose_residuals=solved.pose_residuals,
        leave_one_out=tuple(leave_one_out),
    )


def _atomic_write(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent, text=True)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="") as handle:
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_name, path)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def save_calibration_report(
    output_dir: str | Path,
    *,
    run_id: str,
    arm: str,
    poses: Sequence[CogPoseMeasurement],
    estimate: CogEstimate,
    provenance: Mapping[str, Any],
) -> CogReportPaths:
    """Atomically save a provisional JSON report and raw-sample CSV."""
    run_id = str(run_id).strip()
    if not run_id or Path(run_id).name != run_id or run_id in {".", ".."}:
        raise ValueError("run_id must be one safe path component")
    arm = str(arm).strip().lower()
    if arm not in {"left", "right"}:
        raise ValueError("arm must be 'left' or 'right'")
    frozen_poses = tuple(poses)
    if any(not isinstance(pose, CogPoseMeasurement) for pose in frozen_poses):
        raise TypeError("poses must contain only CogPoseMeasurement values")
    if not isinstance(estimate, CogEstimate):
        raise TypeError("estimate must be a CogEstimate")
    if estimate.provisional is not True or estimate.applied is not False:
        raise ValueError("only a PROVISIONAL / NOT APPLIED estimate may be saved")
    try:
        provenance_copy = json.loads(json.dumps(dict(provenance), allow_nan=False))
    except (TypeError, ValueError) as exc:
        raise ValueError("provenance must be finite JSON-compatible data") from exc

    output_root = Path(output_dir).expanduser()
    run_dir = output_root / run_id
    report_path = run_dir / "calibration_report.json"
    samples_path = run_dir / "pose_samples.csv"
    summaries = [pose.summary() for pose in frozen_poses]
    report = {
        "schema_version": 1,
        "run_id": run_id,
        "arm": arm,
        "status": "PROVISIONAL / NOT APPLIED",
        "provisional": True,
        "applied": False,
        "provenance": provenance_copy,
        "poses": [asdict(summary) for summary in summaries],
        "estimate": asdict(estimate),
    }
    report_text = json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n"

    csv_buffer = io.StringIO(newline="")
    writer = csv.writer(csv_buffer)
    writer.writerow(
        [
            "pose_name",
            "sample_index",
            "freshness_value",
            "received_monotonic",
            "gravity_tcp_x_m_s2",
            "gravity_tcp_y_m_s2",
            "gravity_tcp_z_m_s2",
            "force_tcp_x_n",
            "force_tcp_y_n",
            "force_tcp_z_n",
            "torque_tcp_x_nm",
            "torque_tcp_y_nm",
            "torque_tcp_z_nm",
            "q_target_deg",
        ]
    )
    for pose in frozen_poses:
        q_target_json = json.dumps(pose.q_target_deg, separators=(",", ":"))
        for sample_index, sample in enumerate(pose.samples):
            writer.writerow(
                [
                    pose.name,
                    sample_index,
                    sample.freshness_value,
                    sample.received_monotonic,
                    *sample.gravity_tcp,
                    *sample.wrench_tcp,
                    q_target_json,
                ]
            )

    output_root.mkdir(parents=True, exist_ok=True)
    temporary_dir = Path(tempfile.mkdtemp(prefix=f".{run_id}.", dir=output_root))
    try:
        _atomic_write(temporary_dir / report_path.name, report_text)
        _atomic_write(temporary_dir / samples_path.name, csv_buffer.getvalue())
        os.rename(temporary_dir, run_dir)
    except BaseException:
        shutil.rmtree(temporary_dir, ignore_errors=True)
        raise
    return CogReportPaths(report_json=report_path, samples_csv=samples_path)
