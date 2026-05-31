#!/usr/bin/env python3
"""Telemetry-only UDP overlay publisher for benchmark live visualization."""

from __future__ import annotations

import json
import math
import os
import socket
import time
from pathlib import Path
from typing import Any, Callable
from urllib.parse import urlparse


SCHEMA_VERSION = "robotics_lab.circle_overlay.v1"
COMMAND_FIELD_NAMES = {
    "mode",
    "left",
    "right",
    "timeout_sec",
    "coupled_timeout",
    "source_id",
    "session_id",
    "tcp_twist_stand",
    "tcp_twist_local",
    "tcp_delta_stand",
    "tcp_delta_local",
    "tcp_target_stand",
    "target_tcp_stand",
    "q_target_deg",
}


def parse_udp_endpoint(endpoint: str) -> tuple[str, int]:
    parsed = urlparse(endpoint)
    if parsed.scheme != "udp" or not parsed.hostname or parsed.port is None:
        raise ValueError(f"overlay endpoint must be udp://host:port, got {endpoint!r}")
    if parsed.port < 1 or parsed.port > 65535:
        raise ValueError(f"overlay endpoint port out of range: {parsed.port}")
    return parsed.hostname, parsed.port


def finite_float(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    parsed = float(value)
    return parsed if math.isfinite(parsed) else None


def percentile(values: list[float], pct: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * pct / 100.0
    lo = math.floor(index)
    hi = math.ceil(index)
    if lo == hi:
        return ordered[int(index)]
    return ordered[lo] * (hi - index) + ordered[hi] * (index - lo)


def sub(a: list[float], b: list[float]) -> list[float]:
    return [a[i] - b[i] for i in range(3)]


def dot(a: list[float], b: list[float]) -> float:
    return sum(a[i] * b[i] for i in range(3))


def norm(a: list[float]) -> float:
    return math.sqrt(dot(a, a))


def unwrap(values: list[float]) -> list[float]:
    if not values:
        return []
    out = [values[0]]
    offset = 0.0
    prev = values[0]
    for value in values[1:]:
        delta = value - prev
        if delta > math.pi:
            offset -= 2.0 * math.pi
        elif delta < -math.pi:
            offset += 2.0 * math.pi
        out.append(value + offset)
        prev = value
    return out


def solve_3x3(a: list[list[float]], b: list[float]) -> list[float] | None:
    matrix = [row[:] + [b[index]] for index, row in enumerate(a)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda row: abs(matrix[row][col]))
        if abs(matrix[pivot][col]) < 1e-12:
            return None
        matrix[col], matrix[pivot] = matrix[pivot], matrix[col]
        divisor = matrix[col][col]
        for j in range(col, 4):
            matrix[col][j] /= divisor
        for row in range(3):
            if row == col:
                continue
            factor = matrix[row][col]
            for j in range(col, 4):
                matrix[row][j] -= factor * matrix[col][j]
    return [matrix[index][3] for index in range(3)]


def fit_circle_radius(points: list[tuple[float, float]]) -> float | None:
    if len(points) < 3:
        return None
    ata = [[0.0] * 3 for _ in range(3)]
    atb = [0.0] * 3
    for x, y in points:
        row = [x, y, 1.0]
        rhs = x * x + y * y
        for i in range(3):
            atb[i] += row[i] * rhs
            for j in range(3):
                ata[i][j] += row[i] * row[j]
    solution = solve_3x3(ata, atb)
    if solution is None:
        return None
    a, b, c = solution
    cx = 0.5 * a
    cy = 0.5 * b
    radius_sq = c + cx * cx + cy * cy
    if radius_sq <= 0.0 or not math.isfinite(radius_sq):
        return None
    return math.sqrt(radius_sq)


class CircleOverlayMetrics:
    def __init__(self, *, center: list[float], axis1: list[float], axis2: list[float], radius_m: float, omega_rad_s: float) -> None:
        self.center = list(center)
        self.axis1 = list(axis1)
        self.axis2 = list(axis2)
        self.radius_m = float(radius_m)
        self.omega_rad_s = float(omega_rad_s)
        self.errors: list[float] = []
        self.actual_plane_points: list[tuple[float, float]] = []
        self.actual_phases: list[float] = []
        self.reference_phases: list[float] = []
        self.sample_count = 0
        self.current_error_m: float | None = None
        self._last_sample_id: int | str | None = None

    def observe(
        self,
        *,
        t_sec: float,
        desired_position: list[float],
        actual_position: list[float] | None,
        sample_id: int | str | None = None,
    ) -> None:
        if actual_position is None:
            self.current_error_m = None
            return
        error = norm(sub(desired_position, actual_position))
        self.current_error_m = error
        if sample_id is not None and sample_id == self._last_sample_id:
            return
        self._last_sample_id = sample_id
        self.errors.append(error)
        self.sample_count += 1
        rel = sub(actual_position, self.center)
        c1 = dot(rel, self.axis1)
        c2 = dot(rel, self.axis2)
        self.actual_plane_points.append((c1, c2))
        if self.radius_m > 1e-12:
            self.actual_phases.append(math.atan2(c2, c1))
            self.reference_phases.append(self.omega_rad_s * t_sec)

    def snapshot(self) -> dict[str, float | int | None]:
        radius = fit_circle_radius(self.actual_plane_points)
        radius_gain = radius / self.radius_m if radius is not None and self.radius_m > 0.0 else None
        estimated_latency_ms = None
        if len(self.actual_phases) >= 5 and radius_gain is not None and 0.25 <= radius_gain <= 4.0:
            actual_unwrapped = unwrap(self.actual_phases)
            lags = [ref - actual for ref, actual in zip(self.reference_phases, actual_unwrapped)]
            lag = percentile(lags, 50.0)
            if lag is not None and abs(self.omega_rad_s) > 1e-12:
                estimated_latency_ms = lag / self.omega_rad_s * 1000.0
        return {
            "current_error_m": self.current_error_m,
            "running_mean_error_m": sum(self.errors) / len(self.errors) if self.errors else None,
            "running_rms_error_m": math.sqrt(sum(value * value for value in self.errors) / len(self.errors)) if self.errors else None,
            "running_p95_error_m": percentile(self.errors, 95.0),
            "radius_gain": radius_gain,
            "estimated_latency_ms": estimated_latency_ms,
            "sample_count": self.sample_count,
        }


def pose_payload(position: list[float], quaternion_xyzw: list[float]) -> dict[str, Any]:
    return {
        "x": float(position[0]),
        "y": float(position[1]),
        "z": float(position[2]),
        "rx": 0.0,
        "ry": 0.0,
        "rz": 0.0,
        "quaternion_xyzw": [float(value) for value in quaternion_xyzw],
    }


def validate_no_command_fields(value: Any, *, path: str = "$") -> None:
    if isinstance(value, dict):
        for key, item in value.items():
            if key in COMMAND_FIELD_NAMES:
                raise ValueError(f"overlay message contains command field {path}.{key}")
            validate_no_command_fields(item, path=f"{path}.{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value):
            validate_no_command_fields(item, path=f"{path}[{index}]")


def build_circle_overlay_message(
    *,
    run_id: str,
    arm: str,
    profile: str,
    controller: str,
    tracking_source: str,
    plane: str,
    center_stand: list[float],
    axis1_stand: list[float],
    axis2_stand: list[float],
    radius_m: float,
    period_sec: float,
    repeat: int,
    phase_rad: float,
    desired_pose_stand: dict[str, Any],
    metrics: dict[str, Any],
    command_count: int,
    physical_motion_expected: bool,
    result_so_far: str = "running",
    host_time_ns: int | None = None,
) -> dict[str, Any]:
    message = {
        "schema_version": SCHEMA_VERSION,
        "host_time_ns": int(host_time_ns if host_time_ns is not None else time.monotonic_ns()),
        "run_id": run_id,
        "arm": arm,
        "profile": profile,
        "controller": controller,
        "tracking_source": tracking_source,
        "plane": plane,
        "center_stand": [float(value) for value in center_stand],
        "axis1_stand": [float(value) for value in axis1_stand],
        "axis2_stand": [float(value) for value in axis2_stand],
        "radius_m": float(radius_m),
        "diameter_m": float(radius_m) * 2.0,
        "period_sec": float(period_sec),
        "repeat": int(repeat),
        "phase_rad": float(phase_rad),
        "desired_pose_stand": desired_pose_stand,
        "current_error_m": metrics.get("current_error_m"),
        "running_mean_error_m": metrics.get("running_mean_error_m"),
        "running_rms_error_m": metrics.get("running_rms_error_m"),
        "running_p95_error_m": metrics.get("running_p95_error_m"),
        "radius_gain": metrics.get("radius_gain"),
        "estimated_latency_ms": metrics.get("estimated_latency_ms"),
        "sample_count": int(metrics.get("sample_count") or 0),
        "command_count": int(command_count),
        "physical_motion_expected": bool(physical_motion_expected),
        "result_so_far": result_so_far,
    }
    validate_no_command_fields(message)
    json.dumps(message, allow_nan=False)
    return message


class BenchmarkOverlayPublisher:
    def __init__(
        self,
        *,
        endpoint: str,
        rate_hz: float,
        run_id: str | None,
        artifact_path: Path,
        enabled: bool = True,
        warn: Callable[[str], None] | None = None,
    ) -> None:
        self.endpoint = endpoint
        self.rate_hz = float(rate_hz)
        self.run_id = run_id or f"circle-{time.strftime('%Y%m%dT%H%M%S')}-{os.getpid()}"
        self.artifact_path = artifact_path
        self.enabled = bool(enabled)
        self.warning_count = 0
        self.last_warning: str | None = None
        self.messages_sent = 0
        self.messages_recorded = 0
        self._warn = warn
        self._period_sec = 1.0 / self.rate_hz if self.rate_hz > 0.0 else math.inf
        self._next_send_monotonic: float | None = None
        self._sock: socket.socket | None = None
        self._handle = None
        self._target: tuple[str, int] | None = None
        if self.enabled:
            if self.rate_hz <= 0.0 or not math.isfinite(self.rate_hz):
                raise ValueError("--overlay-pub-rate-hz must be finite and positive")
            self._target = parse_udp_endpoint(endpoint)
            artifact_path.parent.mkdir(parents=True, exist_ok=True)
            self._handle = artifact_path.open("w", encoding="utf-8")
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def close(self) -> None:
        if self._handle is not None:
            self._handle.close()
            self._handle = None
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def _record_warning(self, message: str) -> None:
        self.warning_count += 1
        self.last_warning = message
        if self._warn is not None:
            self._warn(message)

    def record_warning(self, message: str) -> None:
        self._record_warning(message)

    def should_publish(self, now_monotonic: float | None = None, *, force: bool = False) -> bool:
        if not self.enabled:
            return False
        if force:
            return True
        now = time.monotonic() if now_monotonic is None else now_monotonic
        return self._next_send_monotonic is None or now >= self._next_send_monotonic

    def publish(
        self,
        message: dict[str, Any],
        *,
        now_monotonic: float | None = None,
        force: bool = False,
    ) -> bool:
        if not self.should_publish(now_monotonic, force=force):
            return False
        now = time.monotonic() if now_monotonic is None else now_monotonic
        self._next_send_monotonic = now + self._period_sec
        if not self.enabled:
            return False
        try:
            validate_no_command_fields(message)
            payload = json.dumps(message, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
        except Exception as exc:
            self._record_warning(f"overlay message rejected: {exc}")
            return False
        if self._handle is not None:
            self._handle.write(payload.decode("utf-8") + "\n")
            self._handle.flush()
            self.messages_recorded += 1
        try:
            assert self._sock is not None and self._target is not None
            self._sock.sendto(payload, self._target)
            self.messages_sent += 1
            return True
        except OSError as exc:
            self._record_warning(f"overlay UDP send failed: {exc}")
            return False

    def summary(self) -> dict[str, Any]:
        return {
            "overlay_enabled": self.enabled,
            "overlay_pub_endpoint": self.endpoint if self.enabled else None,
            "overlay_pub_rate_hz": self.rate_hz if self.enabled else None,
            "overlay_run_id": self.run_id,
            "overlay_stream": str(self.artifact_path.resolve()) if self.enabled else None,
            "overlay_messages_sent": self.messages_sent,
            "overlay_messages_recorded": self.messages_recorded,
            "overlay_warning_count": self.warning_count,
            "overlay_last_warning": self.last_warning,
        }
