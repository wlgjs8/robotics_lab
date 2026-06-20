from __future__ import annotations

from dataclasses import asdict, dataclass, field, fields, is_dataclass, replace
from pathlib import Path
from typing import Any


SMD_SWEEP_VALUES: dict[str, list[Any]] = {
    "cartesian_control.pose_track_smd.natural_frequency_linear_hz": [0.8, 1.0, 1.3, 1.6],
    "cartesian_control.pose_track_smd.natural_frequency_angular_hz": [0.6, 0.8, 1.0, 1.2],
    "cartesian_control.pose_track_smd.damping_ratio_linear": [1.0, 1.2, 1.5],
    "cartesian_control.pose_track_smd.damping_ratio_angular": [1.0, 1.2, 1.5],
    "cartesian_control.pose_track_smd.velocity_feedforward": [True, False],
    "cartesian_control.pose_track_smd.max_linear_velocity_m_s": [0.32, 0.40],
    "cartesian_control.pose_track_smd.max_angular_velocity_rad_s": [0.8, 1.0],
    "servo.output_moving_average_window": [1, 4, 8, 12, 16],
}


@dataclass(frozen=True)
class AuditConfig:
    nominal_source_rate_hz: float = 30.0
    gap_median_multiplier: float = 3.0
    gap_absolute_threshold_sec: float = 0.100
    plot_dpi: int = 150
    gripper_event_min_span: float = 1e-6


@dataclass(frozen=True)
class ConditioningConfig:
    servo_rate_hz: float = 500.0
    nominal_source_rate_hz: float = 30.0
    gap_median_multiplier: float = 3.0
    gap_absolute_threshold_sec: float = 0.100
    modes: tuple[str, ...] = (
        "raw_zoh",
        "raw_foh_se3",
        "clean_foh_se3",
        "synthetic_policy_surrogate",
    )
    reanchor_after_gap: bool = True
    hold_on_dropout: bool = True
    max_extrapolation_sec: float = 0.100


@dataclass(frozen=True)
class SmoothingConfig:
    method: str = "savgol"
    window_samples: int = 7
    polyorder: int = 3
    lowpass_cutoff_hz: float = 5.0
    cubic_smoothing: float = 0.0
    segment_min_samples: int = 5


@dataclass(frozen=True)
class SyntheticConfig:
    seed: int = 0
    position_noise_rms_mm: float = 1.0
    orientation_noise_rms_deg: float = 0.5
    oscillation_hz_min: float = 5.0
    oscillation_hz_max: float = 10.0
    oscillation_position_rms_mm: float = 0.5
    oscillation_orientation_rms_deg: float = 0.25
    chunk_boundary_position_step_mm: float = 1.0
    chunk_boundary_orientation_step_deg: float = 0.5
    chunk_size_source_frames: int = 8
    dropout_min_frames: int = 1
    dropout_max_frames: int = 3
    dropout_probability: float = 0.02


@dataclass(frozen=True)
class MetricsConfig:
    high_frequency_cutoff_hz: float = 5.0
    chunk_rate_hz: float = 30.0
    spectral_peak_near_rate_half_width_hz: float = 2.0
    velocity_sign_deadband: float = 1e-9
    lag_max_sec: float = 0.250
    sweep_values: dict[str, list[Any]] = field(default_factory=lambda: dict(SMD_SWEEP_VALUES))

    def sweep_config_matrix(self) -> list[dict[str, Any]]:
        """Return one-key sweep configs using real runtime config key names."""

        rows: list[dict[str, Any]] = []
        for key, values in self.sweep_values.items():
            for value in values:
                rows.append({key: value})
        return rows


@dataclass(frozen=True)
class Config:
    audit: AuditConfig = field(default_factory=AuditConfig)
    conditioning: ConditioningConfig = field(default_factory=ConditioningConfig)
    smoothing: SmoothingConfig = field(default_factory=SmoothingConfig)
    synthetic: SyntheticConfig = field(default_factory=SyntheticConfig)
    metrics: MetricsConfig = field(default_factory=MetricsConfig)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def load_config(path: str | Path | None) -> Config:
    """Load the default Phase-1 config, optionally merged with a YAML override."""

    cfg = Config()
    if path is None:
        return cfg
    try:
        import yaml
    except ModuleNotFoundError as exc:
        raise RuntimeError("PyYAML is required to load a tcp_tuning config YAML") from exc
    with Path(path).open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError("tcp_tuning config YAML must contain a mapping at the root")
    return _merge_dataclass(cfg, data)


def _merge_dataclass(instance: Any, overrides: dict[str, Any]) -> Any:
    if not is_dataclass(instance):
        return overrides
    valid = {item.name for item in fields(instance)}
    unknown = sorted(set(overrides) - valid)
    if unknown:
        raise ValueError(f"unknown tcp_tuning config key(s): {', '.join(unknown)}")
    changes: dict[str, Any] = {}
    for key, value in overrides.items():
        current = getattr(instance, key)
        if is_dataclass(current):
            if not isinstance(value, dict):
                raise ValueError(f"config key {key} must be a mapping")
            changes[key] = _merge_dataclass(current, value)
        elif isinstance(current, tuple):
            if not isinstance(value, (list, tuple)):
                raise ValueError(f"config key {key} must be a list")
            changes[key] = tuple(value)
        else:
            changes[key] = value
    return replace(instance, **changes)
