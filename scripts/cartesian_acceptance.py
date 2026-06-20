#!/usr/bin/env python3
"""Simulator-only Cartesian acceptance for PTP, Linear, and Twist commands."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Callable
from urllib.parse import urlparse


REAL_ROBOT_IPS = ("172.28.60.200", "172.28.60.201")


class AcceptanceError(RuntimeError):
    pass


class CommandRecorder:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._handle = self.path.open("w", encoding="utf-8")

    def close(self) -> None:
        self._handle.close()

    def record(self, packet: dict[str, Any]) -> None:
        self._handle.write(json.dumps(packet, sort_keys=True, separators=(",", ":")) + "\n")
        self._handle.flush()


class StateCapture:
    def __init__(self, host: str, port: int, output_path: Path) -> None:
        self.host = host
        self.port = port
        self.output_path = output_path
        self.snapshots: list[dict[str, Any]] = []
        self.invalid_packets = 0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._sock: socket.socket | None = None

    def start(self) -> None:
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((self.host, self.port))
            sock.settimeout(0.1)
        except OSError as exc:
            raise AcceptanceError(f"state capture socket unavailable at {self.host}:{self.port}: {exc}") from exc
        self._sock = sock
        self._thread = threading.Thread(target=self._run, name="cartesian-acceptance-state-capture", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._sock is not None:
            self._sock.close()

    def _run(self) -> None:
        assert self._sock is not None
        with self.output_path.open("w", encoding="utf-8") as out:
            while not self._stop.is_set():
                try:
                    payload, _addr = self._sock.recvfrom(65535)
                except socket.timeout:
                    continue
                except OSError:
                    break
                try:
                    snapshot = json.loads(payload.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    self.invalid_packets += 1
                    continue
                if not isinstance(snapshot, dict):
                    self.invalid_packets += 1
                    continue
                self.snapshots.append(snapshot)
                out.write(json.dumps(snapshot, sort_keys=True, separators=(",", ":")) + "\n")
                out.flush()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Cartesian acceptance runner (against an already-running rbpodo/mock server)"
    )
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--mode", choices=("assume-running",), default="assume-running")
    parser.add_argument("--artifact-dir", type=Path, required=True)
    parser.add_argument("--server", type=Path, required=True)
    parser.add_argument("--server-config", type=Path, required=True)
    parser.add_argument("--left-config", type=Path, required=True)
    parser.add_argument("--right-config", type=Path, required=True)
    parser.add_argument("--command-host", default="127.0.0.1")
    parser.add_argument("--command-port", type=int, default=50010)
    parser.add_argument("--state-host", default="127.0.0.1")
    parser.add_argument("--state-port", type=int, default=50110)
    parser.add_argument("--startup-timeout-sec", type=float, default=8.0)
    parser.add_argument("--capture-sec", type=float, default=1.0)
    parser.add_argument("--linear-duration-sec", type=float, default=1.0)
    parser.add_argument("--position-tolerance-m", type=float, default=0.004)
    parser.add_argument("--orientation-tolerance-rad", type=float, default=0.005)
    parser.add_argument("--line-tolerance-m", type=float, default=0.002)
    parser.add_argument("--run-ptp", action="store_true")
    parser.add_argument("--run-linear", action="store_true")
    parser.add_argument("--run-twist-local", action="store_true")
    parser.add_argument("--run-twist-stand", action="store_true")
    parser.add_argument("--run-near-pi-ptp", action="store_true")
    parser.add_argument("--all", action="store_true")
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--record-run-label", default="")
    parser.add_argument("--require-slerp", action="store_true")
    parser.add_argument("--skip-slerp", action="store_true")
    parser.add_argument("--skip-estop-reset", action="store_true")
    parser.add_argument("--near-pi-math-tests-run", action="store_true")
    parser.add_argument("--preflight-only", action="store_true")
    return parser.parse_args()


def selected_scenarios(args: argparse.Namespace) -> set[str]:
    if args.all or not any((
        args.run_ptp,
        args.run_linear,
        args.run_twist_local,
        args.run_twist_stand,
        args.run_near_pi_ptp,
    )):
        return {"ptp", "linear", "twist_local", "twist_stand"}
    out: set[str] = set()
    if args.run_ptp:
        out.add("ptp")
    if args.run_linear:
        out.add("linear")
    if args.run_twist_local:
        out.add("twist_local")
    if args.run_twist_stand:
        out.add("twist_stand")
    if args.run_near_pi_ptp:
        out.add("near_pi_ptp")
    return out


def read_text(path: Path, label: str) -> str:
    if not path.is_file():
        raise AcceptanceError(f"missing {label}: {path}")
    return path.read_text(encoding="utf-8")


def preflight(args: argparse.Namespace) -> None:
    if args.repeat < 1:
        raise AcceptanceError("--repeat must be >= 1")
    if args.skip_slerp and args.require_slerp:
        raise AcceptanceError("--skip-slerp and --require-slerp are mutually exclusive")
    server_text = read_text(args.server_config, "server config")
    combined = "\n".join(
        text
        for text in (
            server_text,
            read_text(args.left_config, "left config") if args.left_config else "",
            read_text(args.right_config, "right config") if args.right_config else "",
        )
        if text
    )
    # Hardware-free / controller-simulation only: refuse any config that could move
    # the real robot. Mock (operation_mode: simulation) and rbpodo controller
    # `pgmode` simulation (run_mode: real + operation_mode: simulation) are allowed.
    unsafe: list[str] = []
    for ip in REAL_ROBOT_IPS:
        if ip in combined:
            unsafe.append(f"real robot IP {ip}")
    if "operation_mode: real" in server_text:
        unsafe.append("operation_mode: real")
    if "allow_in_real: true" in server_text:
        unsafe.append("cartesian_control.allow_in_real: true")
    required = (
        "provider: pinocchio",
        "publish_tcp: true",
        "cartesian_control:",
        "allow_in_real: false",
        "send_servo_commands: true",
        "orientation_tolerance_rad: 0.005",
    )
    missing = [item for item in required if item not in server_text]
    for tool in ("send_tcp_pose_target.py", "send_tcp_linear_move.py", "send_tcp_twist.py"):
        if not (args.root / "rb_servo_server" / "tools" / tool).is_file():
            missing.append(f"rb_servo_server/tools/{tool}")
    if unsafe:
        raise AcceptanceError("non-real-motion safety preflight failed: " + ", ".join(unsafe))
    if missing:
        raise AcceptanceError("acceptance config/tool preflight failed, missing: " + ", ".join(missing))


def parse_tcp_endpoint(endpoint: str) -> tuple[str, int]:
    parsed = urlparse(endpoint)
    if parsed.scheme != "tcp" or parsed.hostname is None or parsed.port is None:
        raise AcceptanceError(f"expected tcp://host:port endpoint, got {endpoint!r}")
    return parsed.hostname, int(parsed.port)


def wait_tcp(host: str, port: int, timeout_sec: float, label: str) -> None:
    deadline = time.monotonic() + timeout_sec
    last_error: OSError | None = None
    while time.monotonic() < deadline:
        try:
            with socket.create_connection((host, port), timeout=0.25):
                return
        except OSError as exc:
            last_error = exc
            time.sleep(0.05)
    raise AcceptanceError(f"timed out waiting for {label} at {host}:{port}: {last_error}")


def start_process(command: list[str], cwd: Path, output_path: Path, env: dict[str, str] | None = None) -> subprocess.Popen[str]:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output = output_path.open("w", encoding="utf-8")
    try:
        return subprocess.Popen(command, cwd=str(cwd), stdout=output, stderr=subprocess.STDOUT, text=True, env=env)
    except Exception:
        output.close()
        raise


def udp_bind_available(host: str, port: int) -> bool:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((host, port))
            return True
    except OSError:
        return False


def select_udp_port(host: str, requested: int, label: str) -> int:
    if udp_bind_available(host, requested):
        return requested
    for port in range(requested + 1, requested + 100):
        if udp_bind_available(host, port):
            print(
                f"cartesian_acceptance: {label} UDP port {requested} unavailable; using {host}:{port}",
                file=sys.stderr,
            )
            return port
    raise AcceptanceError(f"no available UDP {label} port near {host}:{requested}")


def replace_udp_endpoint(config_text: str, key: str, host: str, port: int) -> str:
    pattern = rf'(^\s*{re.escape(key)}:\s*")udp://[^"]+(".*$)'
    replacement = rf"\1udp://{host}:{port}\2"
    updated, count = re.subn(pattern, replacement, config_text, count=1, flags=re.MULTILINE)
    if count != 1:
        raise AcceptanceError(f"server config does not contain scalar network.{key} endpoint")
    return updated


def replace_scalar_string(config_text: str, key: str, value: str) -> str:
    pattern = rf'(^\s*{re.escape(key)}:\s*")[^"]+(".*$)'
    replacement = rf"\1{value}\2"
    updated, count = re.subn(pattern, replacement, config_text, count=1, flags=re.MULTILINE)
    if count != 1:
        raise AcceptanceError(f"server config does not contain scalar {key} value")
    return updated


def scalar_string_value(config_text: str, key: str) -> str | None:
    match = re.search(rf'^\s*{re.escape(key)}:\s*"([^"]+)"', config_text, flags=re.MULTILINE)
    if match is None:
        return None
    return match.group(1)


def terminate_process(proc: subprocess.Popen[str] | None) -> None:
    if proc is None or proc.poll() is not None:
        return
    proc.send_signal(signal.SIGTERM)
    try:
        proc.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=2.0)


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


def pose_target_command(target: dict[str, Any]) -> dict[str, Any]:
    s = seq()
    return {
        "seq": s,
        "mode": "Hold",
        "host_time_ns": s,
        "left": {"mode": "TcpPoseTarget", "tcp_target_stand": target},
        "right": {"mode": "Hold"},
    }


def linear_move_command(target: dict[str, Any], duration_sec: float, orientation_mode: str) -> dict[str, Any]:
    s = seq()
    return {
        "seq": s,
        "mode": "Hold",
        "host_time_ns": s,
        "timeout_sec": max(0.2, duration_sec + 1.0),
        "left": {
            "mode": "TcpLinearMove",
            "target_tcp_stand": target,
            "duration_sec": duration_sec,
            "orientation_mode": orientation_mode,
        },
        "right": {"mode": "Hold"},
    }


def twist_command(frame: str, twist: list[float], timeout_sec: float = 0.2) -> dict[str, Any]:
    s = seq()
    mode = "TcpTwistLocal" if frame == "local" else "TcpTwistStand"
    key = "tcp_twist_local" if frame == "local" else "tcp_twist_stand"
    return {
        "seq": s,
        "mode": "Hold",
        "host_time_ns": s,
        "timeout_sec": timeout_sec,
        "left": {"mode": mode, key: twist},
        "right": {"mode": "Hold"},
    }


def finite_array(value: Any, length: int, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != length:
        raise AcceptanceError(f"{label} must be a {length}-element list")
    out: list[float] = []
    for item in value:
        number = float(item)
        if not math.isfinite(number):
            raise AcceptanceError(f"{label} contains non-finite value")
        out.append(number)
    return out


def pose(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise AcceptanceError(f"{label} must be an object")
    out: dict[str, Any] = {}
    for key in ("x", "y", "z", "rx", "ry", "rz"):
        number = float(value[key])
        if not math.isfinite(number):
            raise AcceptanceError(f"{label}.{key} is non-finite")
        out[key] = number
    q = finite_array(value.get("quaternion_xyzw"), 4, f"{label}.quaternion_xyzw")
    norm = math.sqrt(sum(v * v for v in q))
    if abs(norm - 1.0) > 1e-5:
        raise AcceptanceError(f"{label}.quaternion_xyzw must be normalized, got {norm}")
    out["quaternion_xyzw"] = [v / norm for v in q]
    return out


def pose_payload(p: dict[str, Any]) -> dict[str, Any]:
    return {
        "x": p["x"],
        "y": p["y"],
        "z": p["z"],
        "rx": p["rx"],
        "ry": p["ry"],
        "rz": p["rz"],
        "quaternion_xyzw": list(p["quaternion_xyzw"]),
    }


def arm_pose(snapshot: dict[str, Any], arm: str, label: str) -> dict[str, Any]:
    return pose(snapshot[arm].get("tcp_stand"), f"{label}.{arm}.tcp_stand")


def vec(p: dict[str, Any]) -> list[float]:
    return [float(p["x"]), float(p["y"]), float(p["z"])]


def sub(a: list[float], b: list[float]) -> list[float]:
    return [a[i] - b[i] for i in range(3)]


def dot(a: list[float], b: list[float]) -> float:
    return sum(a[i] * b[i] for i in range(3))


def norm(a: list[float]) -> float:
    return math.sqrt(dot(a, a))


def quat_angle(a: list[float], b: list[float]) -> float:
    d = abs(sum(float(x) * float(y) for x, y in zip(a, b)))
    return 2.0 * math.acos(max(-1.0, min(1.0, d)))


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


def quat_multiply(a: list[float], b: list[float]) -> list[float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    q = [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]
    n = math.sqrt(sum(v * v for v in q))
    return [v / n for v in q]


def yaw_quat(angle_rad: float) -> list[float]:
    return [0.0, 0.0, math.sin(angle_rad * 0.5), math.cos(angle_rad * 0.5)]


def line_deviation(point: list[float], start: list[float], end: list[float]) -> float:
    line = sub(end, start)
    length = norm(line)
    if length <= 1e-12:
        return norm(sub(point, start))
    rel = sub(point, start)
    s = max(0.0, min(1.0, dot(rel, line) / (length * length)))
    closest = [start[i] + s * line[i] for i in range(3)]
    return norm(sub(point, closest))


def snapshot_after(packet: dict[str, Any]) -> Callable[[dict[str, Any]], bool]:
    threshold = int(packet["host_time_ns"])
    return lambda snapshot: int(snapshot.get("host_time_ns", -1)) >= threshold


def valid_state(snapshot: dict[str, Any]) -> bool:
    try:
        if snapshot.get("schema_version") != 1 or snapshot.get("fault_latched") is not False:
            return False
        if snapshot.get("observed_mode") != "simulation" or snapshot.get("observed_backend") != "simulator":
            return False
        for arm in ("left", "right"):
            arm_state = snapshot[arm]
            if arm_state.get("connection_state") != "Connected":
                return False
            if arm_state.get("has_valid_joint_state") is not True or arm_state.get("has_valid_tcp_pose") is not True:
                return False
            finite_array(arm_state.get("q_actual_deg"), 6, f"{arm}.q_actual_deg")
            finite_array(arm_state.get("q_sent_deg"), 6, f"{arm}.q_sent_deg")
            pose(arm_state.get("tcp_stand"), f"{arm}.tcp_stand")
        return True
    except Exception:
        return False


def wait_for(capture: StateCapture, predicate: Callable[[dict[str, Any]], bool], timeout_sec: float, label: str) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_sec
    index = 0
    while time.monotonic() < deadline:
        for snapshot in capture.snapshots[index:]:
            if predicate(snapshot):
                return snapshot
        index = len(capture.snapshots)
        time.sleep(0.02)
    raise AcceptanceError(f"timed out waiting for {label}")


def latest_valid_after(capture: StateCapture, host_time_ns: int, label: str) -> dict[str, Any]:
    candidates = [
        snapshot
        for snapshot in capture.snapshots
        if int(snapshot.get("host_time_ns", -1)) >= host_time_ns and valid_state(snapshot)
    ]
    if not candidates:
        raise AcceptanceError(f"no valid state snapshot found after {label}")
    return candidates[-1]


def telemetry(snapshot: dict[str, Any], arm: str) -> dict[str, Any]:
    value = snapshot.get(arm, {}).get("cartesian_solve")
    if not isinstance(value, dict):
        raise AcceptanceError(f"{arm}.cartesian_solve is missing")
    return value


def no_fault(snapshot: dict[str, Any], label: str) -> None:
    if snapshot.get("fault_latched") is True:
        raise AcceptanceError(f"{label} latched fault: {snapshot.get('fault_reason')}")


def assert_joint_limits(snapshot: dict[str, Any], label: str) -> None:
    q_min = [-170.0, -120.0, -170.0, -190.0, -120.0, -360.0]
    q_max = [170.0, 120.0, 170.0, 190.0, 120.0, 360.0]
    for arm in ("left", "right"):
        q = finite_array(snapshot[arm].get("q_sent_deg"), 6, f"{label}.{arm}.q_sent_deg")
        for idx, value in enumerate(q):
            if value < q_min[idx] - 1e-6 or value > q_max[idx] + 1e-6:
                raise AcceptanceError(f"{label}.{arm}.q_sent_deg[{idx}]={value} outside joint limits")


def arm_motion(ctx: "Context") -> dict[str, Any]:
    packet = lifecycle_command("ArmMotion")
    send_udp(ctx.args.command_host, ctx.args.command_port, packet, ctx.commands)
    wait_for(
        ctx.capture,
        lambda s: snapshot_after(packet)(s)
        and s.get("motion_state") in {"ArmedHold", "Running"}
        and s.get("safety_verdict") == "Ok"
        and s.get("fault_latched") is False,
        ctx.args.startup_timeout_sec,
        "ArmMotion observed",
    )
    return latest_valid_after(ctx.capture, int(packet["host_time_ns"]), "ArmMotion")


class Context:
    def __init__(self, args: argparse.Namespace, capture: StateCapture, commands: CommandRecorder) -> None:
        self.args = args
        self.capture = capture
        self.commands = commands
        self.scenario_results: dict[str, dict[str, Any]] = {}


def run_ptp(ctx: Context) -> None:
    start = arm_motion(ctx)
    start_pose = arm_pose(start, "left", "ptp.start")
    target = pose_payload(start_pose)
    target["x"] += 0.008
    packet = pose_target_command(target)
    send_udp(ctx.args.command_host, ctx.args.command_port, packet, ctx.commands)
    observed = wait_for(
        ctx.capture,
        lambda s: snapshot_after(packet)(s) and s.get("safety_verdict") == "Ok" and s.get("fault_latched") is False,
        ctx.args.startup_timeout_sec,
        "TcpPoseTarget accepted",
    )
    assert_joint_limits(observed, "ptp")
    deadline = time.monotonic() + ctx.args.capture_sec + ctx.args.startup_timeout_sec
    final = observed
    while time.monotonic() < deadline:
        time.sleep(0.1)
        candidate = latest_valid_after(ctx.capture, int(packet["host_time_ns"]), "PTP command")
        final_pose = arm_pose(candidate, "left", "ptp.final")
        final = candidate
        if norm(sub(vec(final_pose), vec(target))) <= ctx.args.position_tolerance_m:
            break
    final_pose = arm_pose(final, "left", "ptp.final")
    position_error = norm(sub(vec(final_pose), vec(target)))
    orientation_error = quat_angle(final_pose["quaternion_xyzw"], target["quaternion_xyzw"])
    if position_error > ctx.args.position_tolerance_m:
        raise AcceptanceError(f"PTP final position error {position_error} > {ctx.args.position_tolerance_m}")
    if orientation_error > ctx.args.orientation_tolerance_rad:
        raise AcceptanceError(f"PTP final orientation error {orientation_error} > {ctx.args.orientation_tolerance_rad}")
    no_fault(final, "PTP")
    samples = collect_path_samples(ctx.capture, int(packet["host_time_ns"]), int(final.get("host_time_ns", seq())), "left")
    ctx.scenario_results["ptp"] = scenario_result(
        command_seq=int(packet["seq"]),
        max_position_error_m=position_error,
        max_orientation_error_rad=orientation_error,
        max_ik_duration_us=sample_metric_max(samples, "ik_duration_us"),
        sample_count=len(samples),
        final_position_error_m=position_error,
        final_orientation_error_rad=orientation_error,
    )


def run_near_pi_ptp(ctx: Context) -> None:
    start = arm_motion(ctx)
    start_pose = arm_pose(start, "left", "near_pi_ptp.start")
    target = pose_payload(start_pose)
    target["quaternion_xyzw"] = quat_multiply(start_pose["quaternion_xyzw"], yaw_quat(math.pi - 1e-6))
    packet = pose_target_command(target)
    send_udp(ctx.args.command_host, ctx.args.command_port, packet, ctx.commands)
    observed = wait_for(
        ctx.capture,
        lambda s: snapshot_after(packet)(s) and s.get("safety_verdict") == "Ok" and s.get("fault_latched") is False,
        ctx.args.startup_timeout_sec,
        "TcpPoseTarget near-pi accepted",
    )
    assert_joint_limits(observed, "near_pi_ptp")
    final, final_position_error, final_orientation_error = wait_for_pose_tolerance(
        ctx,
        packet,
        vec(target),
        target["quaternion_xyzw"],
        "near_pi_ptp",
    )
    if final_position_error > ctx.args.position_tolerance_m:
        raise AcceptanceError(f"Near-pi PTP final position error {final_position_error}")
    if final_orientation_error > ctx.args.orientation_tolerance_rad:
        raise AcceptanceError(f"Near-pi PTP final orientation error {final_orientation_error}")
    no_fault(final, "Near-pi PTP")
    samples = collect_path_samples(ctx.capture, int(packet["host_time_ns"]), int(final.get("host_time_ns", seq())), "left")
    ctx.scenario_results["near_pi_ptp"] = scenario_result(
        command_seq=int(packet["seq"]),
        max_position_error_m=final_position_error,
        max_orientation_error_rad=final_orientation_error,
        max_ik_duration_us=sample_metric_max(samples, "ik_duration_us"),
        sample_count=len(samples),
        target_rotation_angle_rad=math.pi - 1e-6,
        final_position_error_m=final_position_error,
        final_orientation_error_rad=final_orientation_error,
    )


def collect_path_samples(capture: StateCapture, start_ns: int, end_ns: int, arm: str) -> list[dict[str, Any]]:
    samples: list[dict[str, Any]] = []
    for snapshot in capture.snapshots:
        host_time_ns = int(snapshot.get("host_time_ns", 0))
        if host_time_ns < start_ns or host_time_ns > end_ns:
            continue
        arm_state = snapshot.get(arm)
        if not isinstance(arm_state, dict) or arm_state.get("tcp_stand") is None:
            continue
        tel = arm_state.get("cartesian_solve") if isinstance(arm_state.get("cartesian_solve"), dict) else {}
        p = pose(arm_state.get("tcp_stand"), f"{arm}.path.tcp_stand")
        samples.append({"host_time_ns": host_time_ns, "pose": p, "telemetry": tel})
    return samples


def assert_path_monotonic(samples: list[dict[str, Any]], label: str) -> float:
    values: list[float] = []
    for sample in samples:
        value = sample["telemetry"].get("path_s")
        if isinstance(value, (int, float)) and math.isfinite(float(value)):
            values.append(float(value))
    if len(values) < 2:
        raise AcceptanceError(f"{label} did not publish enough path_s samples")
    max_drop = 0.0
    for prev, current in zip(values, values[1:]):
        max_drop = max(max_drop, prev - current)
    if max_drop > 0.08:
        raise AcceptanceError(f"{label} path_s was not monotonic enough, max drop {max_drop}")
    if max(values) < 0.90:
        raise AcceptanceError(f"{label} path_s did not approach completion, max {max(values)}")
    return max_drop


def wait_for_pose_tolerance(
    ctx: Context,
    packet: dict[str, Any],
    target_position: list[float],
    target_quaternion: list[float],
    label: str,
) -> tuple[dict[str, Any], float, float]:
    deadline = time.monotonic() + ctx.args.capture_sec + ctx.args.startup_timeout_sec
    final = latest_valid_after(ctx.capture, int(packet["host_time_ns"]), label)
    final_position_error = math.inf
    final_orientation_error = math.inf
    while time.monotonic() < deadline:
        time.sleep(0.05)
        candidate = latest_valid_after(ctx.capture, int(packet["host_time_ns"]), label)
        p = arm_pose(candidate, "left", f"{label}.final")
        final = candidate
        final_position_error = norm(sub(vec(p), target_position))
        final_orientation_error = quat_angle(p["quaternion_xyzw"], target_quaternion)
        if (
            candidate.get("fault_latched") is False
            and final_position_error <= ctx.args.position_tolerance_m
            and final_orientation_error <= ctx.args.orientation_tolerance_rad
        ):
            return final, final_position_error, final_orientation_error
    p = arm_pose(final, "left", f"{label}.final")
    return final, norm(sub(vec(p), target_position)), quat_angle(p["quaternion_xyzw"], target_quaternion)


def run_linear_constant(ctx: Context) -> None:
    start = arm_motion(ctx)
    start_pose = arm_pose(start, "left", "linear_constant.start")
    target = pose_payload(start_pose)
    target["y"] += 0.012
    packet = linear_move_command(target, ctx.args.linear_duration_sec, "constant")
    start_ns = int(packet["host_time_ns"])
    send_udp(ctx.args.command_host, ctx.args.command_port, packet, ctx.commands)
    done = wait_for(
        ctx.capture,
        lambda s: snapshot_after(packet)(s)
        and telemetry(s, "left").get("path_done") is True
        and s.get("fault_latched") is False,
        ctx.args.linear_duration_sec + ctx.args.startup_timeout_sec,
        "TcpLinearMove constant path_done",
    )
    done_ns = int(done.get("host_time_ns", seq()))
    samples = collect_path_samples(ctx.capture, start_ns, done_ns, "left")
    max_drop = assert_path_monotonic(samples, "TcpLinearMove constant")
    start_pos = vec(start_pose)
    target_pos = vec(target)
    max_line = 0.0
    max_q = 0.0
    max_path_orientation_error = 0.0
    max_tracking = 0.0
    for sample in samples:
        p = sample["pose"]
        max_line = max(max_line, line_deviation(vec(p), start_pos, target_pos))
        max_q = max(max_q, quat_angle(p["quaternion_xyzw"], start_pose["quaternion_xyzw"]))
        tel = sample["telemetry"]
        if isinstance(tel.get("path_position_error_m"), (int, float)):
            max_tracking = max(max_tracking, float(tel["path_position_error_m"]))
        if isinstance(tel.get("path_orientation_error_rad"), (int, float)):
            max_path_orientation_error = max(max_path_orientation_error, float(tel["path_orientation_error_rad"]))
    final, final_position_error, final_orientation_error = wait_for_pose_tolerance(
        ctx,
        packet,
        target_pos,
        start_pose["quaternion_xyzw"],
        "linear_constant",
    )
    if final_position_error > ctx.args.position_tolerance_m:
        raise AcceptanceError(f"Linear constant final position error {final_position_error}")
    if final_orientation_error > ctx.args.orientation_tolerance_rad:
        raise AcceptanceError(f"Linear constant final orientation error {final_orientation_error}")
    if max_line > ctx.args.line_tolerance_m:
        raise AcceptanceError(f"Linear constant line deviation {max_line} > {ctx.args.line_tolerance_m}")
    if max_q > ctx.args.orientation_tolerance_rad:
        raise AcceptanceError(f"Linear constant changed orientation by {max_q}")
    no_fault(final, "Linear constant")
    ctx.scenario_results["linear_constant"] = scenario_result(
        command_seq=int(packet["seq"]),
        max_position_error_m=final_position_error,
        max_orientation_error_rad=final_orientation_error,
        max_line_deviation_m=max_line,
        max_path_orientation_error_rad=max(max_q, max_path_orientation_error),
        max_ik_duration_us=sample_metric_max(samples, "ik_duration_us"),
        path_done_observed=True,
        path_done_time_ns=done_ns,
        sample_count=len(samples),
        max_path_s_drop=max_drop,
        final_position_error_m=final_position_error,
        final_orientation_error_rad=final_orientation_error,
        max_quaternion_angle_from_start_rad=max_q,
        max_path_tracking_error_m=max_tracking,
    )


def run_linear_slerp(ctx: Context) -> None:
    start = arm_motion(ctx)
    start_pose = arm_pose(start, "left", "linear_slerp.start")
    target = pose_payload(start_pose)
    target["z"] += 0.006
    target["quaternion_xyzw"] = quat_multiply(start_pose["quaternion_xyzw"], yaw_quat(0.03))
    packet = linear_move_command(target, ctx.args.linear_duration_sec, "slerp")
    start_ns = int(packet["host_time_ns"])
    send_udp(ctx.args.command_host, ctx.args.command_port, packet, ctx.commands)
    done = wait_for(
        ctx.capture,
        lambda s: snapshot_after(packet)(s)
        and telemetry(s, "left").get("path_done") is True
        and s.get("fault_latched") is False,
        ctx.args.linear_duration_sec + ctx.args.startup_timeout_sec,
        "TcpLinearMove slerp path_done",
    )
    done_ns = int(done.get("host_time_ns", seq()))
    samples = collect_path_samples(ctx.capture, start_ns, done_ns, "left")
    assert_path_monotonic(samples, "TcpLinearMove slerp")
    orientation_progress = [
        quat_angle(sample["pose"]["quaternion_xyzw"], start_pose["quaternion_xyzw"])
        for sample in samples
    ]
    max_drop = 0.0
    for prev, current in zip(orientation_progress, orientation_progress[1:]):
        max_drop = max(max_drop, prev - current)
    if max_drop > 0.02:
        raise AcceptanceError(f"Linear slerp orientation was not monotonic enough, max drop {max_drop}")
    final, final_position_error, final_orientation_error = wait_for_pose_tolerance(
        ctx,
        packet,
        vec(target),
        target["quaternion_xyzw"],
        "linear_slerp",
    )
    if final_position_error > ctx.args.position_tolerance_m:
        raise AcceptanceError(f"Linear slerp final position error {final_position_error}")
    if final_orientation_error > max(ctx.args.orientation_tolerance_rad, 0.01):
        raise AcceptanceError(f"Linear slerp final orientation error {final_orientation_error}")
    no_fault(final, "Linear slerp")
    ctx.scenario_results["linear_slerp"] = scenario_result(
        command_seq=int(packet["seq"]),
        max_position_error_m=final_position_error,
        max_orientation_error_rad=final_orientation_error,
        max_line_deviation_m=sample_metric_max(samples, "path_line_deviation_m"),
        max_path_orientation_error_rad=sample_metric_max(samples, "path_orientation_error_rad"),
        max_ik_duration_us=sample_metric_max(samples, "ik_duration_us"),
        path_done_observed=True,
        path_done_time_ns=done_ns,
        sample_count=len(samples),
        final_position_error_m=final_position_error,
        final_orientation_error_rad=final_orientation_error,
        max_orientation_progress_drop_rad=max_drop,
    )


def stream_twist(ctx: Context, frame: str, twist: list[float], duration_sec: float, rate_hz: float) -> tuple[int, int]:
    first_ns = 0
    last_ns = 0
    period = 1.0 / rate_hz
    deadline = time.monotonic() + duration_sec
    while time.monotonic() < deadline:
        packet = twist_command(frame, twist, timeout_sec=0.2)
        if first_ns == 0:
            first_ns = int(packet["host_time_ns"])
        last_ns = int(packet["host_time_ns"])
        send_udp(ctx.args.command_host, ctx.args.command_port, packet, ctx.commands)
        time.sleep(period)
    stop = hold_command()
    send_udp(ctx.args.command_host, ctx.args.command_port, stop, ctx.commands)
    return first_ns, last_ns


def run_twist(ctx: Context, frame: str) -> None:
    label = f"twist_{frame}"
    start = arm_motion(ctx)
    start_pose = arm_pose(start, "left", f"{label}.start")
    direction = [1.0, 0.0, 0.0]
    if frame == "local":
        rot = quat_to_matrix(start_pose["quaternion_xyzw"])
        direction = [rot[0][0], rot[1][0], rot[2][0]]
    start_ns, _last_ns = stream_twist(ctx, frame, [0.02, 0.0, 0.0, 0.0, 0.0, 0.0], 1.0, 30.0)
    time.sleep(0.3)
    final = latest_valid_after(ctx.capture, start_ns, f"{label} stream")
    final_pose = arm_pose(final, "left", f"{label}.final")
    final_ns = int(final.get("host_time_ns", seq()))
    samples = collect_path_samples(ctx.capture, start_ns, final_ns, "left")
    delta = sub(vec(final_pose), vec(start_pose))
    along = dot(delta, direction)
    off_axis = norm(sub(delta, [along * direction[i] for i in range(3)]))
    orientation_error = quat_angle(final_pose["quaternion_xyzw"], start_pose["quaternion_xyzw"])
    max_orientation_drift = orientation_error
    for sample in samples:
        max_orientation_drift = max(
            max_orientation_drift,
            quat_angle(sample["pose"]["quaternion_xyzw"], start_pose["quaternion_xyzw"]),
        )
    if along < 0.003:
        raise AcceptanceError(f"{label} did not move primarily forward, projected distance {along}")
    if off_axis > 0.01:
        raise AcceptanceError(f"{label} off-axis movement {off_axis} too large")
    if max_orientation_drift > ctx.args.orientation_tolerance_rad:
        raise AcceptanceError(f"{label} orientation drift {max_orientation_drift} > {ctx.args.orientation_tolerance_rad}")
    no_fault(final, label)
    ctx.scenario_results[label] = scenario_result(
        max_position_error_m=off_axis,
        max_orientation_error_rad=max_orientation_drift,
        max_twist_orientation_drift_rad=max_orientation_drift,
        max_ik_duration_us=sample_metric_max(samples, "ik_duration_us"),
        sample_count=len(samples),
        projected_translation_m=along,
        off_axis_translation_m=off_axis,
        orientation_error_rad=orientation_error,
    )


def run_estop_reset(ctx: Context) -> dict[str, Any]:
    if ctx.args.skip_estop_reset:
        return {"skipped": True}
    estop = lifecycle_command("EmergencyStop")
    send_udp(ctx.args.command_host, ctx.args.command_port, estop, ctx.commands)
    estop_snapshot = wait_for(
        ctx.capture,
        lambda s: snapshot_after(estop)(s)
        and (s.get("fault_latched") is True or s.get("safety_verdict") == "EmergencyStop"),
        ctx.args.startup_timeout_sec,
        "EmergencyStop observed",
    )
    reset = lifecycle_command("ResetFault")
    send_udp(ctx.args.command_host, ctx.args.command_port, reset, ctx.commands)
    reset_snapshot = wait_for(
        ctx.capture,
        lambda s: snapshot_after(reset)(s) and s.get("fault_latched") is False,
        ctx.args.startup_timeout_sec,
        "ResetFault observed",
    )
    return {
        "skipped": False,
        "estop_motion_state": estop_snapshot.get("motion_state"),
        "reset_motion_state": reset_snapshot.get("motion_state"),
    }


def git_commit(root: Path) -> str | None:
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=str(root), text=True).strip()
    except Exception:
        return None


def metric_max(snapshots: list[dict[str, Any]], key: str) -> float:
    values: list[float] = []
    for snapshot in snapshots:
        for arm in ("left", "right"):
            tel = snapshot.get(arm, {}).get("cartesian_solve")
            if isinstance(tel, dict) and isinstance(tel.get(key), (int, float)):
                number = float(tel[key])
                if math.isfinite(number):
                    values.append(number)
    return max(values) if values else 0.0


def scenario_metric_max(results: dict[str, dict[str, Any]], key: str) -> float:
    values: list[float] = []
    for result in results.values():
        value = result.get(key)
        if isinstance(value, (int, float)) and math.isfinite(float(value)):
            values.append(float(value))
    return max(values) if values else 0.0


def sample_metric_max(samples: list[dict[str, Any]], key: str) -> float:
    values: list[float] = []
    for sample in samples:
        telemetry_value = sample.get("telemetry")
        if not isinstance(telemetry_value, dict):
            continue
        value = telemetry_value.get(key)
        if isinstance(value, (int, float)) and math.isfinite(float(value)):
            values.append(float(value))
    return max(values) if values else 0.0


def scenario_result(
    *,
    command_seq: int | None = None,
    max_position_error_m: float = 0.0,
    max_orientation_error_rad: float = 0.0,
    max_line_deviation_m: float = 0.0,
    max_path_orientation_error_rad: float = 0.0,
    max_twist_orientation_drift_rad: float = 0.0,
    max_ik_duration_us: float = 0.0,
    max_cartesian_servo_duration_us: float | None = None,
    path_done_observed: bool = False,
    path_done_time_ns: int | None = None,
    sample_count: int = 0,
    **extra: Any,
) -> dict[str, Any]:
    result: dict[str, Any] = {
        "result": "pass",
        "passed": True,
        "max_position_error_m": max_position_error_m,
        "max_orientation_error_rad": max_orientation_error_rad,
        "max_line_deviation_m": max_line_deviation_m,
        "max_path_orientation_error_rad": max_path_orientation_error_rad,
        "max_twist_orientation_drift_rad": max_twist_orientation_drift_rad,
        "max_ik_duration_us": max_ik_duration_us,
        "max_cartesian_servo_duration_us": max_cartesian_servo_duration_us,
        "path_done_observed": path_done_observed,
        "path_done_time_ns": path_done_time_ns,
        "sample_count": sample_count,
    }
    if command_seq is not None:
        result["command_seq"] = command_seq
    result.update(extra)
    return result


def fault_list(snapshots: list[dict[str, Any]]) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for snapshot in snapshots:
        if snapshot.get("fault_latched") is True or snapshot.get("safety_verdict") not in {None, "Ok", "IkFailed", "EmergencyStop"}:
            out.append(
                {
                    "host_time_ns": snapshot.get("host_time_ns"),
                    "safety_verdict": snapshot.get("safety_verdict"),
                    "motion_state": snapshot.get("motion_state"),
                    "fault_latched": snapshot.get("fault_latched"),
                    "fault_reason": snapshot.get("fault_reason"),
                }
            )
    return out


def write_path_csv(artifact_dir: Path, snapshots: list[dict[str, Any]], arm: str) -> None:
    path = artifact_dir / f"path_samples_{arm}.csv"
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "host_time_ns",
                "x",
                "y",
                "z",
                "qx",
                "qy",
                "qz",
                "qw",
                "path_active",
                "path_s",
                "path_line_deviation_m",
                "path_position_error_m",
                "path_orientation_error_rad",
                "path_done",
            ]
        )
        for snapshot in snapshots:
            arm_state = snapshot.get(arm)
            if not isinstance(arm_state, dict) or arm_state.get("tcp_stand") is None:
                continue
            try:
                p = pose(arm_state.get("tcp_stand"), f"{arm}.csv.tcp_stand")
            except Exception:
                continue
            tel = arm_state.get("cartesian_solve") if isinstance(arm_state.get("cartesian_solve"), dict) else {}
            writer.writerow(
                [
                    snapshot.get("host_time_ns"),
                    p["x"],
                    p["y"],
                    p["z"],
                    *p["quaternion_xyzw"],
                    tel.get("path_active"),
                    tel.get("path_s"),
                    tel.get("path_line_deviation_m"),
                    tel.get("path_position_error_m"),
                    tel.get("path_orientation_error_rad"),
                    tel.get("path_done"),
                ]
            )


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


def aggregate_scenario_results(iterations: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    aggregate: dict[str, dict[str, Any]] = {}
    for iteration in iterations:
        scenarios = iteration.get("scenarios", {})
        if not isinstance(scenarios, dict):
            continue
        for name, result in scenarios.items():
            if not isinstance(result, dict):
                continue
            current = aggregate.setdefault(
                name,
                {
                    "result": "pass",
                    "passed": True,
                    "iterations": 0,
                    "path_done_observed": False,
                    "path_done_time_ns": None,
                    "sample_count": 0,
                },
            )
            current["iterations"] = int(current["iterations"]) + 1
            current["passed"] = bool(current["passed"]) and bool(result.get("passed", result.get("result") == "pass"))
            current["result"] = "pass" if current["passed"] else "fail"
            current["path_done_observed"] = bool(current["path_done_observed"]) or bool(result.get("path_done_observed"))
            if result.get("path_done_time_ns") is not None:
                current["path_done_time_ns"] = result.get("path_done_time_ns")
            current["sample_count"] = int(current["sample_count"]) + int(result.get("sample_count", 0) or 0)
            for key, value in result.items():
                if key in {"result", "passed", "path_done_observed", "path_done_time_ns", "sample_count"}:
                    continue
                if isinstance(value, (int, float)) and math.isfinite(float(value)):
                    existing = current.get(key)
                    current[key] = max(float(existing), float(value)) if isinstance(existing, (int, float)) else value
                elif key not in current:
                    current[key] = value
    return aggregate


def run_acceptance(args: argparse.Namespace) -> dict[str, Any]:
    preflight(args)
    artifact_dir = args.artifact_dir.resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)
    if args.preflight_only:
        summary = {
            "result": "pass",
            "preflight_only": True,
            "record_run_label": args.record_run_label,
            "repeat": args.repeat,
            "config_file": str(args.server_config.resolve()),
            "git_commit": git_commit(args.root),
            "near_pi_math_tests_run": bool(args.near_pi_math_tests_run),
        }
        write_summaries(artifact_dir, summary)
        return summary
    left_proc: subprocess.Popen[str] | None = None
    right_proc: subprocess.Popen[str] | None = None
    server_proc: subprocess.Popen[str] | None = None
    capture = StateCapture(args.state_host, args.state_port, artifact_dir / "state_stream.jsonl")
    commands = CommandRecorder(artifact_dir / "command_packets.jsonl")
    capture.start()
    try:
        for name in ("left_simulator.log", "right_simulator.log", "rb_servo_server.log"):
            (artifact_dir / name).write_text("not captured: runs against an already-running server\n", encoding="utf-8")

        wait_for(capture, valid_state, args.startup_timeout_sec, "valid FK TCP state")
        scenarios = selected_scenarios(args)
        iteration_results: list[dict[str, Any]] = []
        for iteration in range(1, args.repeat + 1):
            ctx = Context(args, capture, commands)
            if "ptp" in scenarios:
                run_ptp(ctx)
            if "near_pi_ptp" in scenarios:
                run_near_pi_ptp(ctx)
            if "linear" in scenarios:
                run_linear_constant(ctx)
                if not args.skip_slerp:
                    run_linear_slerp(ctx)
                elif args.require_slerp:
                    raise AcceptanceError("--require-slerp was set but slerp was skipped")
            if "twist_local" in scenarios:
                run_twist(ctx, "local")
            if "twist_stand" in scenarios:
                run_twist(ctx, "stand")
            iteration_results.append({"iteration": iteration, "scenarios": ctx.scenario_results})
        estop_reset = run_estop_reset(ctx)
    finally:
        commands.close()
        terminate_process(server_proc)
        terminate_process(right_proc)
        terminate_process(left_proc)
        capture.stop()

    write_path_csv(artifact_dir, capture.snapshots, "left")
    write_path_csv(artifact_dir, capture.snapshots, "right")
    servo_log = copy_servo_log(artifact_dir)
    scenario_results = aggregate_scenario_results(iteration_results)
    summary = {
        "result": "pass",
        "record_run_label": args.record_run_label,
        "repeat": args.repeat,
        "config_file": str(args.server_config.resolve()),
        "git_commit": git_commit(args.root),
        "artifacts_dir": str(artifact_dir),
        "state_stream": str(artifact_dir / "state_stream.jsonl"),
        "command_packets": str(artifact_dir / "command_packets.jsonl"),
        "rb_servo_server_log": str(artifact_dir / "rb_servo_server.log"),
        "left_simulator_log": str(artifact_dir / "left_simulator.log"),
        "right_simulator_log": str(artifact_dir / "right_simulator.log"),
        "path_samples_left": str(artifact_dir / "path_samples_left.csv"),
        "path_samples_right": str(artifact_dir / "path_samples_right.csv"),
        "state_packets": len(capture.snapshots),
        "invalid_state_packets": capture.invalid_packets,
        "scenarios": scenario_results,
        "iterations": iteration_results,
        "estop_reset": estop_reset,
        "max_position_error_m": metric_max(capture.snapshots, "position_error_m"),
        "max_orientation_error_rad": metric_max(capture.snapshots, "orientation_error_rad"),
        "max_path_orientation_error_rad": max(
            metric_max(capture.snapshots, "path_orientation_error_rad"),
            scenario_metric_max(scenario_results, "max_path_orientation_error_rad"),
        ),
        "max_line_deviation_m": metric_max(capture.snapshots, "path_line_deviation_m"),
        "max_twist_orientation_drift_rad": scenario_metric_max(
            scenario_results,
            "max_twist_orientation_drift_rad",
        ),
        "near_pi_math_tests_run": bool(args.near_pi_math_tests_run),
        "max_path_tracking_error_m": metric_max(capture.snapshots, "path_position_error_m"),
        "max_ik_duration_us": metric_max(capture.snapshots, "ik_duration_us"),
        "max_cartesian_servo_duration_us": None,
        "faults": fault_list(capture.snapshots),
        "servo_log": servo_log,
        "caveat": "simulator-only evidence; not real robot readiness",
    }
    write_summaries(artifact_dir, summary)
    return summary


def write_summaries(artifact_dir: Path, summary: dict[str, Any]) -> None:
    (artifact_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (artifact_dir / "tcp_pose_acceptance_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    compact = {
        "result": summary.get("result"),
        "record_run_label": summary.get("record_run_label", ""),
        "repeat": summary.get("repeat", 1),
        "git_commit": summary.get("git_commit"),
        "config_file": summary.get("config_file"),
        "near_pi_math_tests_run": summary.get("near_pi_math_tests_run", False),
        "scenarios": summary.get("scenarios", {}),
        "iterations": summary.get("iterations", []),
        "caveat": summary.get("caveat", "simulator-only evidence; not real robot readiness"),
    }
    (artifact_dir / "acceptance_results.json").write_text(
        json.dumps(compact, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    with (artifact_dir / "acceptance_results.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "iteration",
                "scenario",
                "passed",
                "max_position_error_m",
                "max_orientation_error_rad",
                "max_line_deviation_m",
                "max_path_orientation_error_rad",
                "max_twist_orientation_drift_rad",
                "max_ik_duration_us",
                "max_cartesian_servo_duration_us",
                "path_done_observed",
                "path_done_time_ns",
                "sample_count",
            ]
        )
        iterations = summary.get("iterations")
        if isinstance(iterations, list) and iterations:
            for iteration in iterations:
                if not isinstance(iteration, dict):
                    continue
                scenarios = iteration.get("scenarios")
                if not isinstance(scenarios, dict):
                    continue
                for name, result in scenarios.items():
                    if not isinstance(result, dict):
                        continue
                    writer.writerow([
                        iteration.get("iteration"),
                        name,
                        result.get("passed", result.get("result") == "pass"),
                        result.get("max_position_error_m", 0.0),
                        result.get("max_orientation_error_rad", 0.0),
                        result.get("max_line_deviation_m", 0.0),
                        result.get("max_path_orientation_error_rad", 0.0),
                        result.get("max_twist_orientation_drift_rad", 0.0),
                        result.get("max_ik_duration_us", 0.0),
                        result.get("max_cartesian_servo_duration_us"),
                        result.get("path_done_observed", False),
                        result.get("path_done_time_ns"),
                        result.get("sample_count", 0),
                    ])
        else:
            scenarios = summary.get("scenarios")
            if isinstance(scenarios, dict):
                for name, result in scenarios.items():
                    if not isinstance(result, dict):
                        continue
                    writer.writerow([
                        "",
                        name,
                        result.get("passed", result.get("result") == "pass"),
                        result.get("max_position_error_m", 0.0),
                        result.get("max_orientation_error_rad", 0.0),
                        result.get("max_line_deviation_m", 0.0),
                        result.get("max_path_orientation_error_rad", 0.0),
                        result.get("max_twist_orientation_drift_rad", 0.0),
                        result.get("max_ik_duration_us", 0.0),
                        result.get("max_cartesian_servo_duration_us"),
                        result.get("path_done_observed", False),
                        result.get("path_done_time_ns"),
                        result.get("sample_count", 0),
                    ])


def main() -> int:
    args = parse_args()
    artifact_dir = args.artifact_dir.resolve()
    try:
        summary = run_acceptance(args)
    except Exception as exc:
        artifact_dir.mkdir(parents=True, exist_ok=True)
        failure = {
            "result": "fail",
            "error": str(exc),
            "record_run_label": getattr(args, "record_run_label", ""),
            "repeat": getattr(args, "repeat", 1),
            "config_file": str(args.server_config.resolve()) if hasattr(args, "server_config") else None,
            "git_commit": git_commit(args.root) if hasattr(args, "root") else None,
            "artifacts_dir": str(artifact_dir),
            "scenarios": {},
            "iterations": [],
            "state_stream": str(artifact_dir / "state_stream.jsonl"),
            "command_packets": str(artifact_dir / "command_packets.jsonl"),
            "rb_servo_server_log": str(artifact_dir / "rb_servo_server.log"),
            "left_simulator_log": str(artifact_dir / "left_simulator.log"),
            "right_simulator_log": str(artifact_dir / "right_simulator.log"),
            "near_pi_math_tests_run": bool(getattr(args, "near_pi_math_tests_run", False)),
            "caveat": "simulator-only evidence; not real robot readiness",
        }
        write_summaries(artifact_dir, failure)
        print(f"cartesian_acceptance: FAIL: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
