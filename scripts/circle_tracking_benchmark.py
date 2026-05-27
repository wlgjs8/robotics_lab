#!/usr/bin/env python3
"""Simulator-only circular TCP tracking benchmark."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import shlex
import shutil
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

from cartesian_acceptance import (
    AcceptanceError,
    CommandRecorder,
    StateCapture,
    load_simulator_config,
    parse_tcp_endpoint,
    prepare_start_local_server_config,
    start_process,
    terminate_process,
    valid_state,
    wait_for,
    wait_tcp,
)


REAL_ROBOT_IPS = ("172.28.60.200", "172.28.60.201")
REAL_GATE_ENV = ("RB_ALLOW_REAL_ROBOT", "RB_ALLOW_REAL_MOTION", "RB_ALLOW_REAL_CARTESIAN")
PROFILE_DEFAULTS: dict[str, tuple[float, float]] = {
    "safe_5cm_10s": (0.05, 10.0),
    "circle_15cm_16s": (0.15, 16.0),
    "circle_15cm_8s": (0.15, 8.0),
    "gene_15cm_4s": (0.15, 4.0),
}
PROFILE_USE_CASES = {
    "safe_5cm_10s": "conservative simulator smoke/regression baseline",
    "circle_15cm_16s": "15 cm circle within the default 0.03 m/s twist speed limit",
    "circle_15cm_8s": "15 cm simulator stress below GENE-style speed",
    "gene_15cm_4s": "explicit GENE-style 15 cm / 4 s simulator-only stress",
}
DEFAULT_ORIENTATION_DRIFT_WARNING_RAD = 0.1
RADIUS_GAIN_WARNING_MIN = 0.8
CONTROLLER_CHOICES = (
    "twist_stand",
    "twist_local",
    "twist_stand_feedback",
    "twist_local_feedback",
    "server_circle",
    "linear_segments",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run a simulator-only circular TCP tracking benchmark and write "
            "summary, CSV, JSONL, and plot artifacts."
        )
    )
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--mode", choices=("start-local", "assume-running"), default="start-local")
    parser.add_argument("--server", type=Path, required=True)
    parser.add_argument("--server-config", type=Path, required=True)
    parser.add_argument("--left-config", type=Path, required=True)
    parser.add_argument("--right-config", type=Path, required=True)
    parser.add_argument("--rbsim-command", default="python3 -m rbsim")
    parser.add_argument("--command-host", default="127.0.0.1")
    parser.add_argument("--command-port", type=int, default=50010)
    parser.add_argument("--state-host", default="127.0.0.1")
    parser.add_argument("--state-port", type=int, default=50110)
    parser.add_argument("--arm", choices=("left", "right"), default="left")
    parser.add_argument("--controller", choices=CONTROLLER_CHOICES, default="twist_stand")
    parser.add_argument("--plane", choices=("xy", "xz", "yz"), default="xy")
    parser.add_argument("--diameter-m", type=float)
    parser.add_argument("--period-sec", type=float)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--command-rate-hz", type=float, default=100.0)
    parser.add_argument("--warmup-sec", type=float, default=0.5)
    parser.add_argument("--settle-sec", type=float, default=1.0)
    parser.add_argument("--startup-timeout-sec", type=float, default=8.0)
    parser.add_argument("--orientation-mode", choices=("constant", "slerp"), default="constant")
    parser.add_argument("--profile", choices=tuple(PROFILE_DEFAULTS), default="safe_5cm_10s")
    parser.add_argument("--allow-fast-stress", action="store_true")
    parser.add_argument("--artifact-dir", type=Path, required=True)
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--skip-plots", action="store_true")
    parser.add_argument("--feedback-kp-pos", type=float, default=2.0)
    parser.add_argument("--feedback-kp-ori", type=float, default=2.0)
    parser.add_argument("--feedback-max-linear-m-s", type=float)
    parser.add_argument("--feedback-max-angular-rad-s", type=float)
    parser.add_argument("--feedback-use-current-state-time", action="store_true")
    parser.add_argument("--max-allowed-rms-error-m", type=float)
    parser.add_argument("--max-allowed-p95-error-m", type=float)
    parser.add_argument("--max-allowed-orientation-drift-rad", type=float)
    parser.add_argument("--max-allowed-latency-ms", type=float)
    return parser.parse_args()


def read_text(path: Path, label: str) -> str:
    if not path.is_file():
        raise AcceptanceError(f"missing {label}: {path}")
    return path.read_text(encoding="utf-8")


def parse_scalar_float(config_text: str, key: str) -> float | None:
    match = re.search(rf"^\s*{re.escape(key)}:\s*([-+0-9.eE]+)\s*(?:#.*)?$", config_text, flags=re.MULTILINE)
    if not match:
        return None
    try:
        value = float(match.group(1))
    except ValueError:
        return None
    return value if math.isfinite(value) else None


def parse_scalar_bool(config_text: str, key: str) -> bool | None:
    match = re.search(rf"^\s*{re.escape(key)}:\s*(true|false)\s*(?:#.*)?$", config_text, flags=re.MULTILINE | re.IGNORECASE)
    if not match:
        return None
    return match.group(1).lower() == "true"


def benchmark_profile_label(profile: str) -> str:
    if profile == "gene_15cm_4s":
        return "GENE-style 15 cm / 4 s stress"
    if profile == "safe_5cm_10s":
        return "safe baseline"
    return profile


def benchmark_profile_metadata(profile: str) -> dict[str, Any]:
    diameter_m, period_sec = PROFILE_DEFAULTS[profile]
    return {
        "name": profile,
        "diameter_m": diameter_m,
        "period_sec": period_sec,
        "required_tangential_speed_m_s": math.pi * diameter_m / period_sec,
        "expected_use_case": PROFILE_USE_CASES[profile],
    }


def apply_profile(args: argparse.Namespace) -> None:
    default_diameter, default_period = PROFILE_DEFAULTS[args.profile]
    if args.diameter_m is None:
        args.diameter_m = default_diameter
    if args.period_sec is None:
        args.period_sec = default_period


def required_speed(args: argparse.Namespace) -> float:
    assert args.diameter_m is not None and args.period_sec is not None
    return math.pi * args.diameter_m / args.period_sec


def benchmark_context(args: argparse.Namespace, safety: dict[str, Any]) -> dict[str, Any]:
    server_text = read_text(args.server_config, "server config")
    simulator_config = args.left_config if args.arm == "left" else args.right_config
    simulator_text = read_text(simulator_config, f"{args.arm} simulator config")
    servo_rate_hz = parse_scalar_float(server_text, "rate_hz")
    motion_time_constant_sec = parse_scalar_float(simulator_text, "motion_time_constant_sec")
    servo_dt_sec = 1.0 / servo_rate_hz if servo_rate_hz and servo_rate_hz > 0.0 else None
    dt_over_tau = (
        servo_dt_sec / motion_time_constant_sec
        if servo_dt_sec is not None and motion_time_constant_sec and motion_time_constant_sec > 0.0
        else None
    )
    return {
        "profile": args.profile,
        "profile_label": benchmark_profile_label(args.profile),
        "profile_expected_use_case": PROFILE_USE_CASES[args.profile],
        "profile_catalog_entry": benchmark_profile_metadata(args.profile),
        "stress_profile": args.profile == "gene_15cm_4s",
        "configured_max_twist_linear_m_s": safety.get("max_twist_linear_m_s"),
        "configured_max_linear_move_speed_m_s": safety.get("max_linear_move_speed_m_s"),
        "simulator_motion_time_constant_sec": motion_time_constant_sec,
        "servo_rate_hz": servo_rate_hz,
        "servo_dt_sec": servo_dt_sec,
        "simulator_dt_over_tau": dt_over_tau,
    }


def preflight(args: argparse.Namespace) -> dict[str, Any]:
    apply_profile(args)
    assert args.diameter_m is not None and args.period_sec is not None
    if args.repeat < 1:
        raise AcceptanceError("--repeat must be >= 1")
    for name, value in (
        ("--diameter-m", args.diameter_m),
        ("--period-sec", args.period_sec),
        ("--command-rate-hz", args.command_rate_hz),
        ("--warmup-sec", args.warmup_sec),
        ("--settle-sec", args.settle_sec),
        ("--feedback-kp-pos", args.feedback_kp_pos),
        ("--feedback-kp-ori", args.feedback_kp_ori),
    ):
        if not math.isfinite(value) or value <= 0.0 and name not in {"--warmup-sec", "--settle-sec"}:
            raise AcceptanceError(f"{name} must be finite and positive")
    if args.warmup_sec < 0.0 or args.settle_sec < 0.0:
        raise AcceptanceError("--warmup-sec and --settle-sec must be non-negative")

    configured_real_gates = [name for name in REAL_GATE_ENV if os.environ.get(name)]
    if configured_real_gates:
        raise AcceptanceError("real robot environment gates must not be set for this simulator benchmark: " + ", ".join(configured_real_gates))

    server_text = read_text(args.server_config, "server config")
    left_text = read_text(args.left_config, "left simulator config")
    right_text = read_text(args.right_config, "right simulator config")
    combined = "\n".join((server_text, left_text, right_text))
    unsafe: list[str] = []
    for ip in REAL_ROBOT_IPS:
        if ip in combined:
            unsafe.append(f"real robot IP {ip}")
    for marker in ("run_mode: real", "backend_type: rbpodo", "allow_in_real: true"):
        if marker in combined:
            unsafe.append(marker)
    if unsafe:
        raise AcceptanceError("simulator-only safety preflight failed: " + ", ".join(unsafe))

    missing = []
    for required in (
        "backend_type: simulator",
        "run_mode: simulation",
        "cartesian_control:",
        "allow_in_simulation: true",
        "allow_in_real: false",
    ):
        if required not in server_text:
            missing.append(required)
    if missing:
        raise AcceptanceError("benchmark config preflight failed, missing: " + ", ".join(missing))

    speed = required_speed(args)
    max_twist = parse_scalar_float(server_text, "max_twist_linear_m_s")
    max_twist_angular = parse_scalar_float(server_text, "max_twist_angular_rad_s")
    max_linear = parse_scalar_float(server_text, "max_linear_move_speed_m_s")
    benchmark_primitives_enabled = parse_scalar_bool(server_text, "enable_benchmark_primitives")
    profile_is_gene = args.profile == "gene_15cm_4s" or (args.diameter_m >= 0.149 and args.period_sec <= 4.01)
    if profile_is_gene and not args.allow_fast_stress:
        raise AcceptanceError("GENE-style 15 cm / 4 s stress requires --allow-fast-stress")
    if args.controller in {"twist_stand", "twist_local", "twist_stand_feedback", "twist_local_feedback", "server_circle"}:
        if max_twist is None:
            raise AcceptanceError("server config must expose max_twist_linear_m_s for twist benchmark preflight")
        if speed > max_twist + 1e-9:
            raise AcceptanceError(
                f"required tangential speed {speed:.6f} m/s exceeds max_twist_linear_m_s {max_twist:.6f}"
            )
        if args.controller.endswith("_feedback"):
            if max_twist_angular is None:
                raise AcceptanceError("server config must expose max_twist_angular_rad_s for feedback benchmark preflight")
            if args.feedback_max_linear_m_s is None:
                args.feedback_max_linear_m_s = max_twist
            if args.feedback_max_angular_rad_s is None:
                args.feedback_max_angular_rad_s = max_twist_angular
            for name, value in (
                ("--feedback-max-linear-m-s", args.feedback_max_linear_m_s),
                ("--feedback-max-angular-rad-s", args.feedback_max_angular_rad_s),
            ):
                if value is None or not math.isfinite(value) or value <= 0.0:
                    raise AcceptanceError(f"{name} must be finite and positive")
    if args.controller == "linear_segments":
        if max_linear is None:
            raise AcceptanceError("server config must expose max_linear_move_speed_m_s for linear benchmark preflight")
        if speed > max_linear + 1e-9:
            raise AcceptanceError(
                f"required tangential speed {speed:.6f} m/s exceeds max_linear_move_speed_m_s {max_linear:.6f}"
            )
    if args.controller == "server_circle" and benchmark_primitives_enabled is not True:
        raise AcceptanceError("server_circle requires cartesian_control.enable_benchmark_primitives: true")

    return {
        "passed": True,
        "simulator_only": True,
        "refused_markers": ["run_mode: real", "backend_type: rbpodo", "allow_in_real: true", *REAL_ROBOT_IPS],
        "real_gate_env_checked": list(REAL_GATE_ENV),
        "max_twist_linear_m_s": max_twist,
        "max_twist_angular_rad_s": max_twist_angular,
        "max_linear_move_speed_m_s": max_linear,
        "enable_benchmark_primitives": benchmark_primitives_enabled,
        "required_tangential_speed_m_s": speed,
    }


def ensure_rbsim_import_path(root: Path) -> None:
    sim_src = (root / "rb_simulator" / "src").resolve()
    if not sim_src.is_dir():
        raise AcceptanceError(f"missing rb_simulator source path: {sim_src}")
    sim_src_text = str(sim_src)
    if sim_src_text not in sys.path:
        sys.path.insert(0, sim_src_text)


def git_commit(root: Path) -> str | None:
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=str(root), text=True).strip()
    except Exception:
        return None


def seq() -> int:
    return time.monotonic_ns()


def send_udp(host: str, port: int, packet: dict[str, Any], recorder: CommandRecorder) -> None:
    packet.setdefault("schema_version", 1)
    packet.setdefault("host_time_ns", packet.get("seq", seq()))
    packet.setdefault("timeout_sec", 0.2)
    packet.setdefault("coupled_timeout", True)
    recorder.record(packet)
    payload = json.dumps(packet, separators=(",", ":"), allow_nan=False).encode("utf-8")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.sendto(payload, (host, port))


def lifecycle_command(mode: str) -> dict[str, Any]:
    s = seq()
    return {"seq": s, "mode": mode, "host_time_ns": s, "left": {}, "right": {}, "timeout_sec": 0.2}


def hold_command() -> dict[str, Any]:
    s = seq()
    return {"seq": s, "mode": "Hold", "host_time_ns": s, "left": {"mode": "Hold"}, "right": {"mode": "Hold"}}


def twist_command(arm: str, frame: str, twist: list[float], timeout_sec: float) -> dict[str, Any]:
    s = seq()
    mode = "TcpTwistLocal" if frame == "local" else "TcpTwistStand"
    key = "tcp_twist_local" if frame == "local" else "tcp_twist_stand"
    return {
        "seq": s,
        "mode": "Hold",
        "host_time_ns": s,
        "timeout_sec": timeout_sec,
        "left": {"mode": mode, key: twist} if arm == "left" else {"mode": "Hold"},
        "right": {"mode": mode, key: twist} if arm == "right" else {"mode": "Hold"},
    }


def linear_move_command(arm: str, target: dict[str, Any], duration_sec: float, orientation_mode: str) -> dict[str, Any]:
    s = seq()
    payload = {
        "mode": "TcpLinearMove",
        "target_tcp_stand": target,
        "duration_sec": duration_sec,
        "orientation_mode": orientation_mode,
    }


def circle_move_command(args: argparse.Namespace, duration_sec: float) -> dict[str, Any]:
    s = seq()
    payload = {
        "mode": "TcpCircleMove",
        "plane": args.plane,
        "diameter_m": float(args.diameter_m),
        "period_sec": float(args.period_sec),
        "repeat": int(args.repeat),
        "center_mode": "start_on_circle",
        "orientation_mode": "constant",
        "frame": "stand",
    }
    return {
        "seq": s,
        "mode": "Hold",
        "host_time_ns": s,
        "timeout_sec": max(0.2, duration_sec + 0.2),
        "left": payload if args.arm == "left" else {"mode": "Hold"},
        "right": payload if args.arm == "right" else {"mode": "Hold"},
    }
    return {
        "seq": s,
        "mode": "Hold",
        "host_time_ns": s,
        "timeout_sec": max(0.2, duration_sec + 0.2),
        "left": payload if arm == "left" else {"mode": "Hold"},
        "right": payload if arm == "right" else {"mode": "Hold"},
    }


def finite_array(value: Any, length: int, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != length:
        raise AcceptanceError(f"{label} must be a {length}-element list")
    out = [float(item) for item in value]
    if not all(math.isfinite(item) for item in out):
        raise AcceptanceError(f"{label} contains non-finite value")
    return out


def pose(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise AcceptanceError(f"{label} must be an object")
    out = {key: float(value[key]) for key in ("x", "y", "z", "rx", "ry", "rz")}
    if not all(math.isfinite(v) for v in out.values()):
        raise AcceptanceError(f"{label} contains non-finite pose values")
    q = finite_array(value.get("quaternion_xyzw"), 4, f"{label}.quaternion_xyzw")
    n = math.sqrt(sum(v * v for v in q))
    if n <= 0.0:
        raise AcceptanceError(f"{label}.quaternion_xyzw is zero")
    out["quaternion_xyzw"] = [v / n for v in q]
    return out


def pose_payload(position: list[float], quaternion_xyzw: list[float]) -> dict[str, Any]:
    return {
        "x": position[0],
        "y": position[1],
        "z": position[2],
        "rx": 0.0,
        "ry": 0.0,
        "rz": 0.0,
        "quaternion_xyzw": list(quaternion_xyzw),
    }


def arm_pose(snapshot: dict[str, Any], arm: str, label: str) -> dict[str, Any]:
    return pose(snapshot.get(arm, {}).get("tcp_stand"), f"{label}.{arm}.tcp_stand")


def vec(p: dict[str, Any]) -> list[float]:
    return [float(p["x"]), float(p["y"]), float(p["z"])]


def twist_linear(twist: list[float]) -> list[float]:
    return [float(twist[0]), float(twist[1]), float(twist[2])]


def twist_angular(twist: list[float]) -> list[float]:
    return [float(twist[3]), float(twist[4]), float(twist[5])]


def quat_angle(a: list[float], b: list[float]) -> float:
    d = abs(sum(float(x) * float(y) for x, y in zip(a, b)))
    return 2.0 * math.acos(max(-1.0, min(1.0, d)))


def quat_conjugate(q: list[float]) -> list[float]:
    return [-q[0], -q[1], -q[2], q[3]]


def quat_multiply(a: list[float], b: list[float]) -> list[float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


def quat_error_vector(q_ref: list[float], q_actual: list[float]) -> list[float]:
    q_err = quat_multiply(q_ref, quat_conjugate(q_actual))
    if q_err[3] < 0.0:
        q_err = [-q_err[0], -q_err[1], -q_err[2], -q_err[3]]
    vector_norm = norm([q_err[0], q_err[1], q_err[2]])
    if vector_norm < 1e-12:
        return [0.0, 0.0, 0.0]
    angle = 2.0 * math.atan2(vector_norm, max(-1.0, min(1.0, q_err[3])))
    axis = [q_err[0] / vector_norm, q_err[1] / vector_norm, q_err[2] / vector_norm]
    return scale(axis, angle)


def quat_to_matrix(q: list[float]) -> list[list[float]]:
    x, y, z, w = q
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def mat_vec(m: list[list[float]], v: list[float]) -> list[float]:
    return [sum(m[row][col] * v[col] for col in range(3)) for row in range(3)]


def mat_transpose_vec(m: list[list[float]], v: list[float]) -> list[float]:
    return [sum(m[row][col] * v[row] for row in range(3)) for col in range(3)]


def add(a: list[float], b: list[float]) -> list[float]:
    return [a[i] + b[i] for i in range(3)]


def sub(a: list[float], b: list[float]) -> list[float]:
    return [a[i] - b[i] for i in range(3)]


def scale(a: list[float], s: float) -> list[float]:
    return [a[i] * s for i in range(3)]


def dot(a: list[float], b: list[float]) -> float:
    return sum(a[i] * b[i] for i in range(3))


def norm(a: list[float]) -> float:
    return math.sqrt(dot(a, a))


def clamp_vector_norm(v: list[float], max_norm: float) -> tuple[list[float], bool]:
    value_norm = norm(v)
    if value_norm <= max_norm or value_norm <= 1e-12:
        return list(v), False
    return scale(v, max_norm / value_norm), True


def compute_feedback_twist_stand(
    *,
    feedforward_linear_stand: list[float],
    position_error_stand: list[float],
    orientation_error_stand: list[float],
    kp_pos: float,
    kp_ori: float,
    max_linear_m_s: float,
    max_angular_rad_s: float,
) -> dict[str, Any]:
    feedback_linear = scale(position_error_stand, kp_pos)
    feedback_angular = scale(orientation_error_stand, kp_ori)
    raw_linear = add(feedforward_linear_stand, feedback_linear)
    raw_angular = feedback_angular
    applied_linear, linear_saturated = clamp_vector_norm(raw_linear, max_linear_m_s)
    applied_angular, angular_saturated = clamp_vector_norm(raw_angular, max_angular_rad_s)
    return {
        "feedforward_twist_stand": [*feedforward_linear_stand, 0.0, 0.0, 0.0],
        "feedback_twist_stand": [*feedback_linear, *feedback_angular],
        "raw_twist_stand": [*raw_linear, *raw_angular],
        "applied_twist_stand": [*applied_linear, *applied_angular],
        "saturated": linear_saturated or angular_saturated,
    }


def zero_feedback_record(t_sec: float, reason: str, frame: str) -> dict[str, Any]:
    return {
        "t_sec": t_sec,
        "frame": frame,
        "feedback_skip_reason": reason,
        "position_error_x": None,
        "position_error_y": None,
        "position_error_z": None,
        "position_error_vector": [0.0, 0.0, 0.0],
        "orientation_error_x": None,
        "orientation_error_y": None,
        "orientation_error_z": None,
        "orientation_error_vector": [0.0, 0.0, 0.0],
        "feedforward_twist": [0.0] * 6,
        "feedback_twist": [0.0] * 6,
        "applied_twist": [0.0] * 6,
        "saturated": False,
        "stale_or_invalid_state": True,
    }


def axes_for_plane(plane: str) -> tuple[list[float], list[float]]:
    if plane == "xy":
        return [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]
    if plane == "xz":
        return [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]
    return [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]


class Trajectory:
    def __init__(self, *, start: list[float], axis1: list[float], axis2: list[float], radius: float, period_sec: float) -> None:
        self.start = start
        self.axis1 = axis1
        self.axis2 = axis2
        self.radius = radius
        self.period_sec = period_sec
        self.omega = 2.0 * math.pi / period_sec
        self.center = sub(start, scale(axis1, radius))

    def position(self, t: float) -> list[float]:
        theta = self.omega * t
        return add(self.center, add(scale(self.axis1, self.radius * math.cos(theta)), scale(self.axis2, self.radius * math.sin(theta))))

    def velocity_components(self, t: float) -> tuple[float, float]:
        theta = self.omega * t
        return -self.radius * self.omega * math.sin(theta), self.radius * self.omega * math.cos(theta)

    def plane_coords(self, point: list[float]) -> tuple[float, float]:
        rel = sub(point, self.center)
        return dot(rel, self.axis1), dot(rel, self.axis2)


def trajectory_velocity_stand(traj: Trajectory, t: float) -> list[float]:
    v1, v2 = traj.velocity_components(t)
    return add(scale(traj.axis1, v1), scale(traj.axis2, v2))


def feedback_state_latest(capture: StateCapture, arm: str) -> dict[str, Any] | None:
    for snapshot in reversed(capture.snapshots):
        if not valid_state(snapshot):
            continue
        try:
            arm_pose(snapshot, arm, "feedback.latest")
        except Exception:
            continue
        return snapshot
    return None


def wait_for_arm_motion(args: argparse.Namespace, capture: StateCapture, commands: CommandRecorder) -> dict[str, Any]:
    packet = lifecycle_command("ArmMotion")
    send_udp(args.command_host, args.command_port, packet, commands)
    wait_for(
        capture,
        lambda s: int(s.get("host_time_ns", -1)) >= int(packet["host_time_ns"])
        and s.get("motion_state") in {"ArmedHold", "Running"}
        and s.get("safety_verdict") == "Ok"
        and s.get("fault_latched") is False,
        args.startup_timeout_sec,
        "ArmMotion observed",
    )
    return latest_valid_after(capture, int(packet["host_time_ns"]), "ArmMotion")


def latest_valid_after(capture: StateCapture, host_time_ns: int, label: str) -> dict[str, Any]:
    candidates = [
        snapshot
        for snapshot in capture.snapshots
        if int(snapshot.get("host_time_ns", -1)) >= host_time_ns and valid_state(snapshot)
    ]
    if not candidates:
        raise AcceptanceError(f"no valid state snapshot found after {label}")
    return candidates[-1]


def stream_twist(args: argparse.Namespace, commands: CommandRecorder, traj: Trajectory, frame: str, duration_sec: float) -> tuple[int, int, int]:
    period = 1.0 / args.command_rate_hz
    timeout = max(0.2, 3.0 * period)
    command_count = 0
    first_ns = 0
    last_ns = 0
    start_monotonic = time.monotonic()
    next_send = start_monotonic
    while True:
        now = time.monotonic()
        t = now - start_monotonic
        if t >= duration_sec:
            break
        v1, v2 = traj.velocity_components(t)
        twist = [0.0] * 6
        if args.plane == "xy":
            twist[0], twist[1] = v1, v2
        elif args.plane == "xz":
            twist[0], twist[2] = v1, v2
        else:
            twist[1], twist[2] = v1, v2
        packet = twist_command(args.arm, frame, twist, timeout)
        if first_ns == 0:
            first_ns = int(packet["host_time_ns"])
        last_ns = int(packet["host_time_ns"])
        send_udp(args.command_host, args.command_port, packet, commands)
        command_count += 1
        next_send += period
        time.sleep(max(0.0, next_send - time.monotonic()))
    stop = hold_command()
    send_udp(args.command_host, args.command_port, stop, commands)
    return first_ns, last_ns, command_count + 1


def stream_twist_feedback(
    args: argparse.Namespace,
    capture: StateCapture,
    commands: CommandRecorder,
    traj: Trajectory,
    frame: str,
    q0: list[float],
    duration_sec: float,
) -> tuple[int, int, int, list[dict[str, Any]]]:
    period = 1.0 / args.command_rate_hz
    timeout = max(0.2, 3.0 * period)
    stale_limit_ns = int(max(0.2, 3.0 * period) * 1e9)
    command_count = 0
    first_ns = 0
    last_ns = 0
    rows: list[dict[str, Any]] = []
    start_monotonic = time.monotonic()
    next_send = start_monotonic
    while True:
        now = time.monotonic()
        elapsed = now - start_monotonic
        if elapsed >= duration_sec:
            break
        now_ns = time.monotonic_ns()
        t = elapsed
        snapshot = feedback_state_latest(capture, args.arm)
        stale_or_invalid = True
        skip_reason = "missing valid feedback state"
        if snapshot is not None:
            state_ns = int(snapshot.get("host_time_ns", -1))
            stale_or_invalid = state_ns < 0 or now_ns - state_ns > stale_limit_ns
            skip_reason = "stale feedback state" if stale_or_invalid else ""
            if args.feedback_use_current_state_time and first_ns:
                t = max(0.0, (state_ns - first_ns) / 1e9)
        if snapshot is None or stale_or_invalid:
            applied_twist = [0.0] * 6
            row = zero_feedback_record(t, skip_reason, frame)
        else:
            actual_pose = arm_pose(snapshot, args.arm, "feedback.actual")
            p_actual = vec(actual_pose)
            q_actual = actual_pose["quaternion_xyzw"]
            p_ref = traj.position(t)
            position_error_stand = sub(p_ref, p_actual)
            orientation_error_stand = quat_error_vector(q0, q_actual)
            feedback = compute_feedback_twist_stand(
                feedforward_linear_stand=trajectory_velocity_stand(traj, t),
                position_error_stand=position_error_stand,
                orientation_error_stand=orientation_error_stand,
                kp_pos=args.feedback_kp_pos,
                kp_ori=args.feedback_kp_ori,
                max_linear_m_s=float(args.feedback_max_linear_m_s),
                max_angular_rad_s=float(args.feedback_max_angular_rad_s),
            )
            if frame == "local":
                rot_current = quat_to_matrix(q_actual)
                feedforward_linear = mat_transpose_vec(rot_current, twist_linear(feedback["feedforward_twist_stand"]))
                feedback_linear = mat_transpose_vec(rot_current, twist_linear(feedback["feedback_twist_stand"]))
                feedback_angular = mat_transpose_vec(rot_current, twist_angular(feedback["feedback_twist_stand"]))
                applied_linear = mat_transpose_vec(rot_current, twist_linear(feedback["applied_twist_stand"]))
                applied_angular = mat_transpose_vec(rot_current, twist_angular(feedback["applied_twist_stand"]))
                feedforward_twist = [*feedforward_linear, 0.0, 0.0, 0.0]
                feedback_twist = [*feedback_linear, *feedback_angular]
                applied_twist = [*applied_linear, *applied_angular]
            else:
                feedforward_twist = feedback["feedforward_twist_stand"]
                feedback_twist = feedback["feedback_twist_stand"]
                applied_twist = feedback["applied_twist_stand"]
            row = {
                "t_sec": t,
                "frame": frame,
                "feedback_skip_reason": "",
                "actual_x": p_actual[0],
                "actual_y": p_actual[1],
                "actual_z": p_actual[2],
                "reference_x": p_ref[0],
                "reference_y": p_ref[1],
                "reference_z": p_ref[2],
                "position_error_x": position_error_stand[0],
                "position_error_y": position_error_stand[1],
                "position_error_z": position_error_stand[2],
                "position_error_vector": position_error_stand,
                "orientation_error_x": orientation_error_stand[0],
                "orientation_error_y": orientation_error_stand[1],
                "orientation_error_z": orientation_error_stand[2],
                "orientation_error_vector": orientation_error_stand,
                "feedforward_twist": feedforward_twist,
                "feedback_twist": feedback_twist,
                "applied_twist": applied_twist,
                "feedforward_twist_stand": feedback["feedforward_twist_stand"],
                "feedback_twist_stand": feedback["feedback_twist_stand"],
                "applied_twist_stand": feedback["applied_twist_stand"],
                "saturated": bool(feedback["saturated"]),
                "stale_or_invalid_state": False,
            }
        packet = twist_command(args.arm, frame, applied_twist, timeout)
        if first_ns == 0:
            first_ns = int(packet["host_time_ns"])
        last_ns = int(packet["host_time_ns"])
        row["host_time_ns"] = int(packet["host_time_ns"])
        rows.append(row)
        send_udp(args.command_host, args.command_port, packet, commands)
        command_count += 1
        next_send += period
        time.sleep(max(0.0, next_send - time.monotonic()))
    stop = hold_command()
    send_udp(args.command_host, args.command_port, stop, commands)
    return first_ns, last_ns, command_count + 1, rows


def stream_linear_segments(args: argparse.Namespace, commands: CommandRecorder, traj: Trajectory, q0: list[float], duration_sec: float) -> tuple[int, int, int]:
    min_segment_sec = 0.05
    segment_sec = max(min_segment_sec, 1.0 / args.command_rate_hz)
    segment_count = max(12, int(math.ceil(duration_sec / segment_sec)))
    segment_sec = duration_sec / segment_count
    command_count = 0
    first_ns = 0
    last_ns = 0
    start_monotonic = time.monotonic()
    for index in range(1, segment_count + 1):
        t = min(duration_sec, index * segment_sec)
        target = pose_payload(traj.position(t), q0)
        packet = linear_move_command(args.arm, target, segment_sec, args.orientation_mode)
        if first_ns == 0:
            first_ns = int(packet["host_time_ns"])
        last_ns = int(packet["host_time_ns"])
        send_udp(args.command_host, args.command_port, packet, commands)
        command_count += 1
        next_send = start_monotonic + index * segment_sec
        time.sleep(max(0.0, next_send - time.monotonic()))
    stop = hold_command()
    send_udp(args.command_host, args.command_port, stop, commands)
    return first_ns, last_ns, command_count + 1


def run_server_circle(args: argparse.Namespace, commands: CommandRecorder, duration_sec: float) -> tuple[int, int, int]:
    packet = circle_move_command(args, duration_sec)
    first_ns = int(packet["host_time_ns"])
    send_udp(args.command_host, args.command_port, packet, commands)
    time.sleep(duration_sec)
    stop = hold_command()
    send_udp(args.command_host, args.command_port, stop, commands)
    return first_ns, int(stop["host_time_ns"]), 2


def collect_actual_samples(capture: StateCapture, args: argparse.Namespace, start_ns: int, end_ns: int) -> list[dict[str, Any]]:
    samples: list[dict[str, Any]] = []
    for snapshot in capture.snapshots:
        host_time_ns = int(snapshot.get("host_time_ns", -1))
        if host_time_ns < start_ns or host_time_ns > end_ns:
            continue
        if not valid_state(snapshot):
            continue
        try:
            p = arm_pose(snapshot, args.arm, "actual")
        except Exception:
            continue
        arm_state = snapshot.get(args.arm, {})
        tel = arm_state.get("cartesian_solve") if isinstance(arm_state.get("cartesian_solve"), dict) else {}
        worker = arm_state.get("worker") if isinstance(arm_state.get("worker"), dict) else {}
        samples.append({"host_time_ns": host_time_ns, "pose": p, "arm_state": arm_state, "telemetry": tel, "worker": worker, "snapshot": snapshot})
    return samples


def reference_rows(traj: Trajectory, q0: list[float], duration_sec: float, rate_hz: float) -> list[dict[str, Any]]:
    count = max(1, int(math.ceil(duration_sec * rate_hz)))
    rows = []
    for index in range(count + 1):
        t = min(duration_sec, index / rate_hz)
        p = traj.position(t)
        rows.append({"t_sec": t, "x": p[0], "y": p[1], "z": p[2], "qx": q0[0], "qy": q0[1], "qz": q0[2], "qw": q0[3]})
    return rows


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


def solve_3x3(a: list[list[float]], b: list[float]) -> list[float] | None:
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda row: abs(m[row][col]))
        if abs(m[pivot][col]) < 1e-12:
            return None
        m[col], m[pivot] = m[pivot], m[col]
        div = m[col][col]
        for j in range(col, 4):
            m[col][j] /= div
        for row in range(3):
            if row == col:
                continue
            factor = m[row][col]
            for j in range(col, 4):
                m[row][j] -= factor * m[col][j]
    return [m[i][3] for i in range(3)]


def fit_circle(points: list[tuple[float, float]]) -> dict[str, Any]:
    if len(points) < 3:
        return {"fit_radius_m": None, "fit_center": None, "fit_reason": "fewer than 3 samples"}
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
        return {"fit_radius_m": None, "fit_center": None, "fit_reason": "least-squares system was singular"}
    a, b, c = solution
    cx = 0.5 * a
    cy = 0.5 * b
    radius_sq = c + cx * cx + cy * cy
    if radius_sq <= 0.0 or not math.isfinite(radius_sq):
        return {"fit_radius_m": None, "fit_center": None, "fit_reason": "fit radius was invalid"}
    return {"fit_radius_m": math.sqrt(radius_sq), "fit_center": [cx, cy], "fit_reason": None}


def metric_max(samples: list[dict[str, Any]], key: str, location: str) -> float | None:
    values: list[float] = []
    for sample in samples:
        source = sample.get(location)
        if not isinstance(source, dict):
            continue
        value = source.get(key)
        if isinstance(value, (int, float)) and math.isfinite(float(value)):
            values.append(float(value))
    return max(values) if values else None


def feedback_metrics(rows: list[dict[str, Any]]) -> dict[str, Any]:
    feedback_linear_norms: list[float] = []
    total_linear_norms: list[float] = []
    for row in rows:
        feedback_twist = row.get("feedback_twist")
        applied_twist = row.get("applied_twist")
        if isinstance(feedback_twist, list) and len(feedback_twist) >= 3:
            feedback_linear_norms.append(norm([float(feedback_twist[0]), float(feedback_twist[1]), float(feedback_twist[2])]))
        if isinstance(applied_twist, list) and len(applied_twist) >= 3:
            total_linear_norms.append(norm([float(applied_twist[0]), float(applied_twist[1]), float(applied_twist[2])]))
    return {
        "mean_feedback_linear_norm_m_s": (
            sum(feedback_linear_norms) / len(feedback_linear_norms) if feedback_linear_norms else None
        ),
        "max_feedback_linear_norm_m_s": max(feedback_linear_norms) if feedback_linear_norms else None,
        "mean_total_command_linear_norm_m_s": (
            sum(total_linear_norms) / len(total_linear_norms) if total_linear_norms else None
        ),
        "feedback_saturation_count": sum(1 for row in rows if row.get("saturated") is True),
        "stale_state_feedback_skips": sum(1 for row in rows if row.get("stale_or_invalid_state") is True),
    }


def write_feedback_artifacts(artifact_dir: Path, rows: list[dict[str, Any]]) -> dict[str, str] | None:
    if not rows:
        return None
    jsonl_path = artifact_dir / "feedback_terms.jsonl"
    with jsonl_path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n")
    csv_rows = []
    for row in rows:
        csv_row = dict(row)
        for key in (
            "feedforward_twist",
            "feedback_twist",
            "applied_twist",
            "feedforward_twist_stand",
            "feedback_twist_stand",
            "applied_twist_stand",
            "position_error_vector",
            "orientation_error_vector",
        ):
            value = csv_row.get(key)
            if isinstance(value, list):
                for index, item in enumerate(value):
                    csv_row[f"{key}_{index}"] = item
                del csv_row[key]
        csv_rows.append(csv_row)
    write_csv(artifact_dir / "feedback_terms.csv", csv_rows)
    return {"jsonl": str(jsonl_path.resolve()), "csv": str((artifact_dir / "feedback_terms.csv").resolve())}


def compute_metrics(
    *,
    args: argparse.Namespace,
    traj: Trajectory,
    q0: list[float],
    samples: list[dict[str, Any]],
    benchmark_start_ns: int,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    merged: list[dict[str, Any]] = []
    errors: list[float] = []
    orientation_drifts: list[float] = []
    radial_errors: list[float] = []
    actual_phases: list[float] = []
    ref_phases: list[float] = []
    plane_points: list[tuple[float, float]] = []
    for sample in samples:
        t = max(0.0, (int(sample["host_time_ns"]) - benchmark_start_ns) / 1e9)
        p = vec(sample["pose"])
        ref = traj.position(t)
        error = norm(sub(p, ref))
        drift = quat_angle(sample["pose"]["quaternion_xyzw"], q0)
        c1, c2 = traj.plane_coords(p)
        phase = math.atan2(c2, c1)
        radial = math.sqrt(c1 * c1 + c2 * c2) - traj.radius
        ref_phase = traj.omega * t
        errors.append(error)
        orientation_drifts.append(drift)
        radial_errors.append(radial)
        actual_phases.append(phase)
        ref_phases.append(ref_phase)
        plane_points.append((c1, c2))
        merged.append(
            {
                "host_time_ns": int(sample["host_time_ns"]),
                "t_sec": t,
                "actual_x": p[0],
                "actual_y": p[1],
                "actual_z": p[2],
                "actual_qx": sample["pose"]["quaternion_xyzw"][0],
                "actual_qy": sample["pose"]["quaternion_xyzw"][1],
                "actual_qz": sample["pose"]["quaternion_xyzw"][2],
                "actual_qw": sample["pose"]["quaternion_xyzw"][3],
                "reference_x": ref[0],
                "reference_y": ref[1],
                "reference_z": ref[2],
                "position_error_m": error,
                "orientation_drift_rad": drift,
                "radial_error_m": radial,
                "reference_phase_rad": ref_phase,
                "actual_phase_rad": phase,
            }
        )

    fit = fit_circle(plane_points)
    phase_reliability_reason = None
    estimated_phase_lag_rad = None
    estimated_latency_ms = None
    if len(actual_phases) < 5:
        phase_reliability_reason = "fewer than 5 samples"
    elif max(abs(v) for v in radial_errors) > max(0.02, traj.radius * 0.8):
        phase_reliability_reason = "trajectory radial distortion too large"
    elif isinstance(fit["fit_radius_m"], (int, float)) and abs(float(fit["fit_radius_m"]) - traj.radius) > max(0.01, traj.radius * 0.5):
        phase_reliability_reason = "circle fit radius differs too much from reference"
    else:
        actual_unwrapped = unwrap(actual_phases)
        ref_unwrapped = ref_phases
        lags = [ref - actual for ref, actual in zip(ref_unwrapped, actual_unwrapped)]
        steady_lags = lags[max(0, len(lags) // 10):]
        estimated_phase_lag_rad = percentile(steady_lags, 50.0)
        if estimated_phase_lag_rad is not None:
            estimated_latency_ms = estimated_phase_lag_rad / traj.omega * 1000.0
            for row, lag in zip(merged, lags):
                row["phase_lag_rad"] = lag
                row["phase_lag_ms"] = lag / traj.omega * 1000.0

    fit_radius = fit["fit_radius_m"]
    fit_center = fit["fit_center"]
    radius_error = abs(fit_radius - traj.radius) if isinstance(fit_radius, (int, float)) else None
    fit_center_error = math.sqrt(fit_center[0] * fit_center[0] + fit_center[1] * fit_center[1]) if isinstance(fit_center, list) else None
    fault_latched = any(sample["snapshot"].get("fault_latched") is True for sample in samples)
    send_within_period = sum(
        1
        for sample in samples
        if sample["snapshot"].get("send_within_period") is True
        or sample.get("arm_state", {}).get("send_within_period") is True
        or sample["snapshot"].get("send_deadline_hit") is True
        or sample.get("arm_state", {}).get("send_deadline_hit") is True
    )
    send_period_overruns = sum(
        1
        for sample in samples
        if sample["snapshot"].get("send_period_overrun") is True
        or sample.get("arm_state", {}).get("send_period_overrun") is True
    )
    send_command_deadline_missed = sum(
        1
        for sample in samples
        if sample["snapshot"].get("send_command_deadline_missed") is True
        or sample.get("arm_state", {}).get("send_command_deadline_missed") is True
    )
    radius_gain = fit_radius / traj.radius if isinstance(fit_radius, (int, float)) and traj.radius > 0.0 else None
    metrics = {
        "reference_radius_m": traj.radius,
        "mean_error_m": sum(errors) / len(errors) if errors else None,
        "rms_error_m": math.sqrt(sum(v * v for v in errors) / len(errors)) if errors else None,
        "median_error_m": percentile(errors, 50.0),
        "p95_error_m": percentile(errors, 95.0),
        "max_error_m": max(errors) if errors else None,
        "mean_orientation_drift_rad": sum(orientation_drifts) / len(orientation_drifts) if orientation_drifts else None,
        "p95_orientation_drift_rad": percentile(orientation_drifts, 95.0),
        "max_orientation_drift_rad": max(orientation_drifts) if orientation_drifts else None,
        "estimated_phase_lag_rad": estimated_phase_lag_rad,
        "estimated_latency_ms": estimated_latency_ms,
        "phase_lag_reliability_reason": phase_reliability_reason,
        "fit_radius_m": fit_radius,
        "radius_gain": radius_gain,
        "amplitude_gain": radius_gain,
        "radius_error_m": radius_error,
        "fit_center_error_m": fit_center_error,
        "fit_center_plane_m": fit_center,
        "circle_fit_reason": fit["fit_reason"],
        "sample_count": len(samples),
        "fault_latched": fault_latched,
        "worker_command_drops_total": metric_max(samples, "command_drops_total", "worker"),
        "worker_pending_overwrites_total": metric_max(samples, "pending_overwrites_total", "worker"),
        "send_within_period_count": send_within_period,
        "send_period_overrun_count": send_period_overruns,
        "send_command_deadline_missed_count": send_command_deadline_missed,
        "send_deadline_hit_count": send_within_period,
        "send_deadline_hit_count_deprecated_alias_for": "send_within_period_count",
        "max_state_age_us": metric_max(samples, "state_age_us", "arm_state"),
        "max_send_result_age_us": metric_max(samples, "send_result_age_us", "arm_state"),
        "max_cartesian_servo_duration_us": None,
        "max_ik_duration_us": metric_max(samples, "ik_duration_us", "telemetry"),
        "integrator_resets_total": metric_max(samples, "integrator_resets_total", "telemetry"),
        "integrator_clamps_total": metric_max(samples, "integrator_clamps_total", "telemetry"),
        "integrator_divergence_total": metric_max(samples, "integrator_divergence_total", "telemetry"),
        "max_command_actual_error_deg_observed": metric_max(samples, "max_command_actual_error_deg_observed", "telemetry"),
    }
    return metrics, merged


def write_csv(path: Path, rows: list[dict[str, Any]], fieldnames: list[str] | None = None) -> None:
    if fieldnames is None:
        keys: list[str] = []
        for row in rows:
            for key in row:
                if key not in keys:
                    keys.append(key)
        fieldnames = keys
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_actual_csv(path: Path, samples: list[dict[str, Any]], benchmark_start_ns: int) -> None:
    rows = []
    for sample in samples:
        p = sample["pose"]
        rows.append(
            {
                "host_time_ns": sample["host_time_ns"],
                "t_sec": (int(sample["host_time_ns"]) - benchmark_start_ns) / 1e9,
                "x": p["x"],
                "y": p["y"],
                "z": p["z"],
                "qx": p["quaternion_xyzw"][0],
                "qy": p["quaternion_xyzw"][1],
                "qz": p["quaternion_xyzw"][2],
                "qw": p["quaternion_xyzw"][3],
            }
        )
    write_csv(path, rows)


def write_summary_csv(path: Path, summary: dict[str, Any]) -> None:
    fields = [
        "result",
        "result_reason",
        "controller",
        "arm",
        "plane",
        "profile",
        "profile_label",
        "diameter_m",
        "reference_radius_m",
        "period_sec",
        "repeat",
        "command_rate_hz",
        "required_tangential_speed_m_s",
        "configured_max_twist_linear_m_s",
        "configured_max_linear_move_speed_m_s",
        "simulator_motion_time_constant_sec",
        "servo_rate_hz",
        "servo_dt_sec",
        "simulator_dt_over_tau",
        "mean_error_m",
        "rms_error_m",
        "p95_error_m",
        "max_error_m",
        "p95_orientation_drift_rad",
        "estimated_latency_ms",
        "fit_radius_m",
        "radius_gain",
        "amplitude_gain",
        "radius_error_m",
        "fit_center_error_m",
        "sample_count",
        "command_count",
        "worker_command_drops_total",
        "integrator_resets_total",
        "integrator_clamps_total",
        "integrator_divergence_total",
        "mean_feedback_linear_norm_m_s",
        "max_feedback_linear_norm_m_s",
        "mean_total_command_linear_norm_m_s",
        "feedback_saturation_count",
        "stale_state_feedback_skips",
        "fault_latched",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerow({field: summary.get(field) for field in fields})


def thresholds_requested(args: argparse.Namespace) -> bool:
    return any(
        value is not None
        for value in (
            args.max_allowed_rms_error_m,
            args.max_allowed_p95_error_m,
            args.max_allowed_orientation_drift_rad,
            args.max_allowed_latency_ms,
        )
    )


def benchmark_result(thresholds_were_requested: bool, failures: list[str]) -> tuple[str, str]:
    if failures:
        return "fail", "thresholds applied and failed"
    if thresholds_were_requested:
        return "pass", "thresholds applied and satisfied"
    return "completed", "run completed without thresholds; performance pass/fail was not evaluated"


def performance_warnings(summary: dict[str, Any], orientation_warning_rad: float = DEFAULT_ORIENTATION_DRIFT_WARNING_RAD) -> list[str]:
    warnings: list[str] = []
    radius = summary.get("reference_radius_m", summary.get("radius_m"))
    radius_gain = summary.get("radius_gain")
    if isinstance(radius_gain, (int, float)) and radius_gain < RADIUS_GAIN_WARNING_MIN:
        warnings.append(
            f"radius_gain {radius_gain:.3f} below {RADIUS_GAIN_WARNING_MIN:.3f}; actual circle amplitude is attenuated"
        )
    rms_error = summary.get("rms_error_m")
    if isinstance(rms_error, (int, float)) and isinstance(radius, (int, float)) and rms_error > 0.5 * radius:
        warnings.append(f"rms_error_m {rms_error:.6f} exceeds half the reference radius {0.5 * radius:.6f}")
    p95_error = summary.get("p95_error_m")
    if isinstance(p95_error, (int, float)) and isinstance(radius, (int, float)) and p95_error > radius:
        warnings.append(f"p95_error_m {p95_error:.6f} exceeds reference radius {radius:.6f}")
    orientation_drift = summary.get("max_orientation_drift_rad")
    if isinstance(orientation_drift, (int, float)) and orientation_drift > orientation_warning_rad:
        warnings.append(
            f"max_orientation_drift_rad {orientation_drift:.6f} exceeds warning threshold {orientation_warning_rad:.6f}"
        )
    if summary.get("fault_latched") is True:
        warnings.append("fault_latched was true during benchmark")
    stale_skips = summary.get("stale_state_feedback_skips")
    if isinstance(stale_skips, (int, float)) and stale_skips > 0:
        warnings.append(f"stale_state_feedback_skips was {stale_skips}")
    saturation_count = summary.get("feedback_saturation_count")
    if isinstance(saturation_count, (int, float)) and saturation_count > 0:
        warnings.append(f"feedback_saturation_count was {saturation_count}")
    return warnings


def plot_artifacts(artifact_dir: Path, args: argparse.Namespace, traj: Trajectory, merged: list[dict[str, Any]]) -> list[str]:
    skipped: list[str] = []
    if args.skip_plots:
        return ["plots skipped by --skip-plots"]
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        return [f"matplotlib unavailable: {exc}"]

    if not merged:
        return ["no merged samples available for plotting"]
    desired = [(row["reference_x"], row["reference_y"], row["reference_z"]) for row in merged]
    actual = [(row["actual_x"], row["actual_y"], row["actual_z"]) for row in merged]
    a1, a2 = traj.axis1, traj.axis2
    center = traj.center
    desired_plane = [(dot(sub(list(p), center), a1), dot(sub(list(p), center), a2)) for p in desired]
    actual_plane = [(dot(sub(list(p), center), a1), dot(sub(list(p), center), a2)) for p in actual]

    plt.figure()
    plt.plot([p[0] for p in desired_plane], [p[1] for p in desired_plane], label="reference")
    plt.plot([p[0] for p in actual_plane], [p[1] for p in actual_plane], label="actual")
    plt.gca().set_aspect("equal", adjustable="box")
    plt.xlabel(f"{args.plane[0]} axis relative to center (m)")
    plt.ylabel(f"{args.plane[1]} axis relative to center (m)")
    plt.legend()
    plt.tight_layout()
    plt.savefig(artifact_dir / "circle_trajectory.png")
    plt.close()

    def line_plot(filename: str, key: str, ylabel: str) -> None:
        plt.figure()
        plt.plot([row["t_sec"] for row in merged], [row.get(key) for row in merged])
        plt.xlabel("time (s)")
        plt.ylabel(ylabel)
        plt.tight_layout()
        plt.savefig(artifact_dir / filename)
        plt.close()

    line_plot("tracking_error_time.png", "position_error_m", "position error (m)")
    line_plot("orientation_drift_time.png", "orientation_drift_rad", "orientation drift (rad)")
    line_plot("radial_error_time.png", "radial_error_m", "radial error (m)")
    if any("phase_lag_ms" in row for row in merged):
        line_plot("phase_lag_time.png", "phase_lag_ms", "phase lag (ms)")
    else:
        skipped.append("phase_lag_time.png skipped because phase estimate was unreliable")

    plt.figure()
    for axis in ("x", "y", "z"):
        plt.plot([row["t_sec"] for row in merged], [row[f"actual_{axis}"] for row in merged], label=f"actual {axis}")
        plt.plot([row["t_sec"] for row in merged], [row[f"reference_{axis}"] for row in merged], linestyle="--", label=f"ref {axis}")
    plt.xlabel("time (s)")
    plt.ylabel("position (m)")
    plt.legend(ncol=2)
    plt.tight_layout()
    plt.savefig(artifact_dir / "axis_positions_time.png")
    plt.close()
    return skipped


def generated_plot_paths(artifact_dir: Path) -> list[str]:
    plot_names = [
        "circle_trajectory.png",
        "tracking_error_time.png",
        "orientation_drift_time.png",
        "phase_lag_time.png",
        "radial_error_time.png",
        "axis_positions_time.png",
    ]
    return [str((artifact_dir / name).resolve()) for name in plot_names if (artifact_dir / name).is_file()]


def threshold_failures(args: argparse.Namespace, summary: dict[str, Any]) -> list[str]:
    checks = [
        ("rms_error_m", args.max_allowed_rms_error_m),
        ("p95_error_m", args.max_allowed_p95_error_m),
        ("max_orientation_drift_rad", args.max_allowed_orientation_drift_rad),
        ("estimated_latency_ms", args.max_allowed_latency_ms),
    ]
    failures: list[str] = []
    for key, limit in checks:
        if limit is None:
            continue
        value = summary.get(key)
        if value is None:
            failures.append(f"{key} unavailable with threshold {limit}")
        elif abs(float(value)) > limit:
            failures.append(f"{key} {value} exceeds threshold {limit}")
    return failures


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def run_benchmark(args: argparse.Namespace) -> dict[str, Any]:
    safety = preflight(args)
    context = benchmark_context(args, safety)
    artifact_dir = args.artifact_dir.resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)
    radius = float(args.diameter_m) * 0.5
    duration_sec = float(args.period_sec) * int(args.repeat)
    base_summary: dict[str, Any] = {
        "repo_git_commit": git_commit(args.root),
        "server_config": str(args.server_config.resolve()),
        "left_config": str(args.left_config.resolve()),
        "right_config": str(args.right_config.resolve()),
        "controller": args.controller,
        "arm": args.arm,
        "plane": args.plane,
        "diameter_m": args.diameter_m,
        "radius_m": radius,
        "reference_radius_m": radius,
        "period_sec": args.period_sec,
        "repeat": args.repeat,
        "command_rate_hz": args.command_rate_hz,
        "required_tangential_speed_m_s": safety["required_tangential_speed_m_s"],
        "safety_preflight": safety,
        "artifact_dir": str(artifact_dir),
        "state_stream": str(artifact_dir / "state_stream.jsonl"),
        "command_packets": str(artifact_dir / "command_packets.jsonl"),
        "rb_servo_server_log": str(artifact_dir / "rb_servo_server.log"),
        "left_simulator_log": str(artifact_dir / "left_simulator.log"),
        "right_simulator_log": str(artifact_dir / "right_simulator.log"),
        "caveat": "simulator-only benchmark evidence; not real robot readiness",
    }
    if args.controller.endswith("_feedback"):
        base_summary.update(
            {
                "feedback_kp_pos": args.feedback_kp_pos,
                "feedback_kp_ori": args.feedback_kp_ori,
                "feedback_max_linear_m_s": args.feedback_max_linear_m_s,
                "feedback_max_angular_rad_s": args.feedback_max_angular_rad_s,
                "feedback_use_current_state_time": args.feedback_use_current_state_time,
                "feedback_mode_caveat": "closed-loop command-source benchmark compensation; not production policy or real robot readiness",
            }
        )
    base_summary.update(context)
    if args.preflight_only:
        base_summary.update({
            "result": "completed",
            "result_reason": "simulator-only safety preflight completed; no tracking run or performance thresholds were evaluated",
            "preflight_only": True,
            "performance_warnings": [],
            "threshold_failures": [],
            "skipped_plots": ["preflight only"],
        })
        write_json(artifact_dir / "summary.json", base_summary)
        write_summary_csv(artifact_dir / "summary.csv", base_summary)
        return base_summary

    if args.mode == "start-local":
        prepare_start_local_server_config(args, artifact_dir)
        base_summary["server_config"] = str(args.server_config.resolve())

    left_proc: subprocess.Popen[str] | None = None
    right_proc: subprocess.Popen[str] | None = None
    server_proc: subprocess.Popen[str] | None = None
    capture = StateCapture(args.state_host, args.state_port, artifact_dir / "state_stream.jsonl")
    commands = CommandRecorder(artifact_dir / "command_packets.jsonl")
    benchmark_start_ns = 0
    benchmark_end_ns = 0
    command_count = 0
    traj: Trajectory | None = None
    q0: list[float] | None = None
    feedback_rows: list[dict[str, Any]] = []
    try:
        capture.start()
        if args.mode == "start-local":
            if not args.server.is_file() or not os.access(args.server, os.X_OK):
                raise AcceptanceError(f"server binary missing or not executable: {args.server}")
            ensure_rbsim_import_path(args.root)
            left_config = load_simulator_config(args.left_config)
            right_config = load_simulator_config(args.right_config)
            if left_config.arm != "left" or right_config.arm != "right":
                raise AcceptanceError("simulator configs do not declare left/right arms")
            left_host, left_port = parse_tcp_endpoint(left_config.control_bind)
            right_host, right_port = parse_tcp_endpoint(right_config.control_bind)
            rbsim_command = shlex.split(args.rbsim_command)
            env = os.environ.copy()
            for gate in REAL_GATE_ENV:
                env.pop(gate, None)
            sim_src = args.root / "rb_simulator" / "src"
            env["PYTHONPATH"] = str(sim_src) + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
            left_proc = start_process([*rbsim_command, "--config", str(args.left_config.resolve())], args.root, artifact_dir / "left_simulator.log", env)
            wait_tcp(left_host, left_port, args.startup_timeout_sec, "left simulator")
            right_proc = start_process([*rbsim_command, "--config", str(args.right_config.resolve())], args.root, artifact_dir / "right_simulator.log", env)
            wait_tcp(right_host, right_port, args.startup_timeout_sec, "right simulator")
            server_env = os.environ.copy()
            for gate in REAL_GATE_ENV:
                server_env.pop(gate, None)
            server_proc = start_process([str(args.server.resolve()), "--config", str(args.server_config.resolve())], artifact_dir, artifact_dir / "rb_servo_server.log", server_env)
        else:
            for name in ("left_simulator.log", "right_simulator.log", "rb_servo_server.log"):
                (artifact_dir / name).write_text("not captured: --assume-running was used\n", encoding="utf-8")

        wait_for(capture, valid_state, args.startup_timeout_sec, "valid FK TCP state")
        armed = wait_for_arm_motion(args, capture, commands)
        start_pose = arm_pose(armed, args.arm, "benchmark.start")
        p0 = vec(start_pose)
        q0 = start_pose["quaternion_xyzw"]
        local_axis1, local_axis2 = axes_for_plane(args.plane)
        if args.controller in {"twist_local", "twist_local_feedback"}:
            rot = quat_to_matrix(q0)
            axis1 = mat_vec(rot, local_axis1)
            axis2 = mat_vec(rot, local_axis2)
        else:
            axis1, axis2 = local_axis1, local_axis2
        traj = Trajectory(start=p0, axis1=axis1, axis2=axis2, radius=radius, period_sec=float(args.period_sec))
        if args.warmup_sec > 0.0:
            time.sleep(args.warmup_sec)

        if args.controller == "twist_stand":
            benchmark_start_ns, benchmark_end_ns, command_count = stream_twist(args, commands, traj, "stand", duration_sec)
        elif args.controller == "twist_local":
            benchmark_start_ns, benchmark_end_ns, command_count = stream_twist(args, commands, traj, "local", duration_sec)
        elif args.controller == "twist_stand_feedback":
            benchmark_start_ns, benchmark_end_ns, command_count, feedback_rows = stream_twist_feedback(
                args, capture, commands, traj, "stand", q0, duration_sec
            )
        elif args.controller == "twist_local_feedback":
            benchmark_start_ns, benchmark_end_ns, command_count, feedback_rows = stream_twist_feedback(
                args, capture, commands, traj, "local", q0, duration_sec
            )
        elif args.controller == "server_circle":
            benchmark_start_ns, benchmark_end_ns, command_count = run_server_circle(args, commands, duration_sec)
        else:
            benchmark_start_ns, benchmark_end_ns, command_count = stream_linear_segments(args, commands, traj, q0, duration_sec)
        if args.settle_sec > 0.0:
            time.sleep(args.settle_sec)
        benchmark_end_ns = max(benchmark_end_ns, seq())
    finally:
        commands.close()
        terminate_process(server_proc)
        terminate_process(right_proc)
        terminate_process(left_proc)
        capture.stop()

    if benchmark_start_ns == 0:
        raise AcceptanceError("benchmark did not send any command packets")
    actual_samples = collect_actual_samples(capture, args, benchmark_start_ns, benchmark_start_ns + int(duration_sec * 1e9))
    if not actual_samples:
        raise AcceptanceError("no valid actual TCP samples captured during benchmark")
    if traj is None or q0 is None:
        raise AcceptanceError("benchmark reference trajectory was not initialized")
    metrics, merged = compute_metrics(args=args, traj=traj, q0=q0, samples=actual_samples, benchmark_start_ns=benchmark_start_ns)
    feedback_artifacts = write_feedback_artifacts(artifact_dir, feedback_rows)
    if feedback_rows:
        metrics.update(feedback_metrics(feedback_rows))
    skipped_plots = plot_artifacts(artifact_dir, args, traj, merged)

    write_csv(artifact_dir / "reference.csv", reference_rows(traj, q0, duration_sec, args.command_rate_hz))
    write_actual_csv(artifact_dir / "actual.csv", actual_samples, benchmark_start_ns)
    write_csv(artifact_dir / "merged_samples.csv", merged)
    servo_log = copy_servo_log(artifact_dir)

    summary = dict(base_summary)
    summary.update(metrics)
    thresholds_were_requested = thresholds_requested(args)
    failures = threshold_failures(args, summary)
    if thresholds_were_requested and summary.get("fault_latched") is True:
        failures.append("fault_latched was true during benchmark")
    result, result_reason = benchmark_result(thresholds_were_requested, failures)
    summary.update(
        {
            "result": result,
            "result_reason": result_reason,
            "preflight_only": False,
            "command_count": command_count,
            "invalid_state_packets": capture.invalid_packets,
            "state_packet_count": len(capture.snapshots),
            "benchmark_start_ns": benchmark_start_ns,
            "benchmark_end_ns": benchmark_end_ns,
            "duration_sec": duration_sec,
            "generated_plots": generated_plot_paths(artifact_dir),
            "skipped_plots": skipped_plots,
            "servo_log": servo_log,
            "feedback_terms": feedback_artifacts,
            "threshold_failures": failures,
            "performance_warnings": performance_warnings(summary),
            "orientation_drift_warning_threshold_rad": DEFAULT_ORIENTATION_DRIFT_WARNING_RAD,
        }
    )
    write_json(artifact_dir / "summary.json", summary)
    write_summary_csv(artifact_dir / "summary.csv", summary)
    return summary


def copy_servo_log(artifact_dir: Path) -> dict[str, Any] | None:
    log_dir = artifact_dir / "logs"
    candidates = sorted(log_dir.glob("*.csv"), key=lambda path: path.stat().st_mtime)
    if not candidates:
        return None
    target = artifact_dir / "servo_log.csv"
    shutil.copy2(candidates[-1], target)
    rows = 0
    with target.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for _row in reader:
            rows += 1
    return {"path": str(target), "rows": rows}


def main() -> int:
    args = parse_args()
    artifact_dir = args.artifact_dir.resolve()
    try:
        summary = run_benchmark(args)
    except Exception as exc:
        artifact_dir.mkdir(parents=True, exist_ok=True)
        failure = {
            "result": "error",
            "result_reason": "benchmark could not run",
            "error": str(exc),
            "repo_git_commit": git_commit(args.root) if hasattr(args, "root") else None,
            "server_config": str(args.server_config.resolve()) if hasattr(args, "server_config") else None,
            "left_config": str(args.left_config.resolve()) if hasattr(args, "left_config") else None,
            "right_config": str(args.right_config.resolve()) if hasattr(args, "right_config") else None,
            "safety_preflight": {"passed": False, "error": str(exc)},
            "performance_warnings": [],
            "threshold_failures": [str(exc)],
            "caveat": "simulator-only benchmark evidence; not real robot readiness",
        }
        write_json(artifact_dir / "summary.json", failure)
        write_summary_csv(artifact_dir / "summary.csv", failure)
        print(f"circle_tracking_benchmark: FAIL: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary.get("result") in {"pass", "completed"} else 2


if __name__ == "__main__":
    raise SystemExit(main())
