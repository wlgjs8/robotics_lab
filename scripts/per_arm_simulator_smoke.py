#!/usr/bin/env python3
"""Per-arm rb_simulator + rb_servo_server hardware-free smoke."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shutil
import shlex
import signal
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
SIM_SRC = ROOT / "rb_simulator" / "src"


class SmokeError(RuntimeError):
    pass


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
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.host, self.port))
        sock.settimeout(0.1)
        self._sock = sock
        self._thread = threading.Thread(target=self._run, name="state-capture", daemon=True)
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
                if isinstance(snapshot, dict):
                    self.snapshots.append(snapshot)
                    out.write(json.dumps(snapshot, sort_keys=True, separators=(",", ":")) + "\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left-config", type=Path, default=ROOT / "rb_simulator" / "config" / "left_rb3_730e.yaml")
    parser.add_argument("--right-config", type=Path, default=ROOT / "rb_simulator" / "config" / "right_rb3_730e.yaml")
    parser.add_argument("--server", type=Path, default=ROOT / "rb_servo_server" / "build" / "hardware_free_gate" / "rb_servo_server")
    parser.add_argument("--server-config", type=Path, default=None)
    parser.add_argument("--artifacts-dir", type=Path, default=ROOT / "rb_simulator" / "artifacts" / "hardware_free_gate")
    parser.add_argument("--rbsim-command", default=os.environ.get("RBSIM_COMMAND", "python3 -m rbsim"))
    parser.add_argument("--command-host", default="127.0.0.1")
    parser.add_argument("--command-port", type=int, default=50010)
    parser.add_argument("--state-host", default="127.0.0.1")
    parser.add_argument("--state-port", type=int, default=50110)
    parser.add_argument("--startup-timeout-sec", type=float, default=6.0)
    parser.add_argument("--capture-sec", type=float, default=1.5)
    parser.add_argument("--target-delta-deg", type=float, default=1.0)
    return parser.parse_args()


def ensure_file(path: Path, label: str, executable: bool = False) -> None:
    if not path.exists():
        raise SmokeError(f"missing {label}: {path}")
    if executable and not os.access(path, os.X_OK):
        raise SmokeError(f"{label} is not executable: {path}")


def finite_joint_array(value: Any, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != 6:
        raise SmokeError(f"{label} must be a 6-element list")
    out = []
    for item in value:
        number = float(item)
        if not math.isfinite(number):
            raise SmokeError(f"{label} contains non-finite value {item!r}")
        out.append(number)
    return out


def is_connected_valid(snapshot: dict[str, Any]) -> bool:
    try:
        for arm in ("left", "right"):
            arm_state = snapshot[arm]
            if arm_state.get("connection_state") != "Connected":
                return False
            if arm_state.get("has_valid_joint_state") is not True:
                return False
            finite_joint_array(arm_state.get("q_actual_deg"), f"{arm}.q_actual_deg")
            finite_joint_array(arm_state.get("q_sent_deg"), f"{arm}.q_sent_deg")
        return True
    except (KeyError, TypeError, ValueError, SmokeError):
        return False


def close_enough(actual: list[float], expected: list[float], tolerance: float = 0.05) -> bool:
    return all(abs(a - b) <= tolerance for a, b in zip(actual, expected))


def wait_for_snapshot(capture: StateCapture, predicate: Any, timeout_sec: float, label: str) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_sec
    seen = 0
    while time.monotonic() < deadline:
        for snapshot in capture.snapshots[seen:]:
            if predicate(snapshot):
                return snapshot
        seen = len(capture.snapshots)
        time.sleep(0.02)
    raise SmokeError(f"timed out waiting for state snapshot: {label}")


def send_udp_command(host: str, port: int, message: dict[str, Any]) -> None:
    payload = json.dumps(message, separators=(",", ":")).encode("utf-8")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(payload, (host, port))
    finally:
        sock.close()


def lifecycle_command(mode: str) -> dict[str, Any]:
    seq = time.monotonic_ns()
    return {
        "seq": seq,
        "mode": mode,
        "host_time_ns": seq,
        "timeout_sec": 0.2,
        "coupled_timeout": True,
        "left": {},
        "right": {},
    }


def joint_target_command(left: list[float], right: list[float]) -> dict[str, Any]:
    seq = time.monotonic_ns()
    return {
        "seq": seq,
        "mode": "JointTarget",
        "host_time_ns": seq,
        "timeout_sec": 0.2,
        "coupled_timeout": True,
        "left": {"q_target_deg": left},
        "right": {"q_target_deg": right},
    }


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
    raise SmokeError(f"timed out waiting for {label} at {host}:{port}: {last_error}")


def start_process(command: list[str], cwd: Path, output_path: Path, env: dict[str, str] | None = None) -> subprocess.Popen[str]:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output = output_path.open("w", encoding="utf-8")
    try:
        return subprocess.Popen(command, cwd=str(cwd), stdout=output, stderr=subprocess.STDOUT, text=True, env=env)
    except Exception:
        output.close()
        raise


def terminate_process(proc: subprocess.Popen[str] | None) -> None:
    if proc is None or proc.poll() is not None:
        return
    proc.send_signal(signal.SIGTERM)
    try:
        proc.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=2.0)


def latest_servo_log(log_dir: Path) -> Path:
    candidates = sorted(log_dir.glob("*.csv"), key=lambda path: path.stat().st_mtime)
    if not candidates:
        raise SmokeError(f"no servo CSV log found under {log_dir}")
    return candidates[-1]


def validate_servo_log(path: Path, target_seq: int, left_target: list[float], right_target: list[float]) -> dict[str, Any]:
    rows = 0
    target_rows = 0
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fieldnames = set(reader.fieldnames or [])
        required = {
            "command_seq",
            "left_send_ok",
            "right_send_ok",
            "logger_dropped_samples",
            *{f"left_q_sent_{i}" for i in range(6)},
            *{f"right_q_sent_{i}" for i in range(6)},
        }
        missing = sorted(required - fieldnames)
        if missing:
            raise SmokeError("servo log missing required columns: " + ", ".join(missing))
        for row in reader:
            rows += 1
            if row["left_send_ok"].strip().lower() not in {"1", "true", "yes", "ok"}:
                raise SmokeError("servo log recorded left_send_ok failure")
            if row["right_send_ok"].strip().lower() not in {"1", "true", "yes", "ok"}:
                raise SmokeError("servo log recorded right_send_ok failure")
            if int(float(row["logger_dropped_samples"])) != 0:
                raise SmokeError("servo log recorded dropped samples")
            left_sent = finite_joint_array([row[f"left_q_sent_{i}"] for i in range(6)], "log.left_q_sent")
            right_sent = finite_joint_array([row[f"right_q_sent_{i}"] for i in range(6)], "log.right_q_sent")
            if int(float(row["command_seq"])) >= target_seq and close_enough(left_sent, left_target) and close_enough(right_sent, right_target):
                target_rows += 1
    if rows == 0:
        raise SmokeError(f"servo log contains no samples: {path}")
    if target_rows == 0:
        raise SmokeError("servo log never reflected the commanded joint target")
    return {"path": str(path), "rows": rows, "target_rows": target_rows}


def write_server_config(path: Path) -> None:
    path.write_text(
        """schema: robotics_lab.rb_servo_server.v1

left_robot:
  backend_type: simulator
  run_mode: simulation
  name: left_simulator
  simulator_control_endpoint: "tcp://127.0.0.1:50200"
  simulator_request_timeout_sec: 0.2

right_robot:
  backend_type: simulator
  run_mode: simulation
  name: right_simulator
  simulator_control_endpoint: "tcp://127.0.0.1:50210"
  simulator_request_timeout_sec: 0.2

left_mount:
  base_pose_in_stand: [0.1601, -0.1725, 0.5825, 0.785, 2.35619, 0.0]

right_mount:
  base_pose_in_stand: [-0.1601, -0.1725, 0.5825, 0.785, -2.35619, 0.0]

servo:
  rate_hz: 100
  command_timeout_sec: 0.2
  startup_mode: Hold
  enable_realtime_priority: false
  realtime_priority: 80
  cpu_core: -1
  filter_dt_min_ratio: 0.5
  filter_dt_max_ratio: 1.5

safety:
  q_min_deg: [-170, -120, -170, -190, -120, -360]
  q_max_deg: [170, 120, 170, 190, 120, 360]
  dq_max_deg_s: [60, 60, 60, 90, 90, 120]
  ddq_max_deg_s2: [300, 300, 300, 500, 500, 700]
  command_timeout_sec: 0.2
  max_tracking_error_deg: 10.0
  tracking_error_policy: snap_to_actual
  latch_fault_on_robot_state_error: true
  stop_both_arms_on_single_arm_error: true

network:
  command_bind: "udp://127.0.0.1:50010"
  state_pub_endpoint: "udp://127.0.0.1:50110"
  state_pub_rate_hz: 20
  command_source_allowlist: ["127.0.0.1/32"]

logging:
  enable: true
  directory: "./logs"
  flush_period_ms: 100
  queue_capacity: 4096

force_control:
  provider: null
  enable: false
""",
        encoding="utf-8",
    )


def run_smoke(args: argparse.Namespace) -> dict[str, Any]:
    ensure_file(args.left_config, "left simulator config")
    ensure_file(args.right_config, "right simulator config")
    ensure_file(args.server, "rb_servo_server binary", executable=True)

    artifacts_dir = args.artifacts_dir.resolve()
    artifacts_dir.mkdir(parents=True, exist_ok=True)
    server_config = args.server_config.resolve() if args.server_config else artifacts_dir / "dual_simulator_smoke.yaml"
    if args.server_config is None:
        write_server_config(server_config)
    ensure_file(server_config, "rb_servo_server simulator config")

    rbsim_command = shlex.split(args.rbsim_command)
    if not rbsim_command:
        raise SmokeError("RBSIM_COMMAND resolved to an empty command")

    env = os.environ.copy()
    env["PYTHONPATH"] = str(SIM_SRC) + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")

    capture = StateCapture(args.state_host, args.state_port, artifacts_dir / "state_stream.jsonl")
    left_proc: subprocess.Popen[str] | None = None
    right_proc: subprocess.Popen[str] | None = None
    server_proc: subprocess.Popen[str] | None = None

    capture.start()
    try:
        left_proc = start_process(
            [*rbsim_command, "--config", str(args.left_config.resolve())],
            cwd=ROOT,
            output_path=artifacts_dir / "left_simulator.log",
            env=env,
        )
        wait_tcp("127.0.0.1", 50200, args.startup_timeout_sec, "left simulator control")

        right_proc = start_process(
            [*rbsim_command, "--config", str(args.right_config.resolve())],
            cwd=ROOT,
            output_path=artifacts_dir / "right_simulator.log",
            env=env,
        )
        wait_tcp("127.0.0.1", 50210, args.startup_timeout_sec, "right simulator control")

        server_proc = start_process(
            [str(args.server.resolve()), "--config", str(server_config)],
            cwd=artifacts_dir,
            output_path=artifacts_dir / "rb_servo_server.log",
        )

        first = wait_for_snapshot(capture, is_connected_valid, args.startup_timeout_sec, "connected valid startup")
        left_initial = finite_joint_array(first["left"].get("q_actual_deg"), "left.q_actual_deg")
        right_initial = finite_joint_array(first["right"].get("q_actual_deg"), "right.q_actual_deg")

        arm_motion = lifecycle_command("ArmMotion")
        send_udp_command(args.command_host, args.command_port, arm_motion)
        time.sleep(0.1)

        left_target = list(left_initial)
        right_target = list(right_initial)
        left_target[0] += args.target_delta_deg
        right_target[0] -= args.target_delta_deg
        joint_target = joint_target_command(left_target, right_target)
        send_udp_command(args.command_host, args.command_port, joint_target)
        wait_for_snapshot(
            capture,
            lambda snapshot: int(snapshot.get("command_seq", -1)) >= int(joint_target["seq"]),
            args.startup_timeout_sec,
            "JointTarget command_seq",
        )
        time.sleep(args.capture_sec)
    finally:
        terminate_process(server_proc)
        terminate_process(right_proc)
        terminate_process(left_proc)
        capture.stop()

    target_seq = int(joint_target["seq"])
    target_packets = 0
    for snapshot in capture.snapshots:
        if int(snapshot.get("command_seq", -1)) < target_seq:
            continue
        left_sent = finite_joint_array(snapshot["left"].get("q_sent_deg"), "left.q_sent_deg")
        right_sent = finite_joint_array(snapshot["right"].get("q_sent_deg"), "right.q_sent_deg")
        if close_enough(left_sent, left_target) and close_enough(right_sent, right_target):
            target_packets += 1

    if target_packets == 0:
        raise SmokeError("state stream never reflected the commanded joint target")

    servo_log_path = latest_servo_log(artifacts_dir / "logs")
    servo_log_copy = artifacts_dir / "servo_log.csv"
    shutil.copy2(servo_log_path, servo_log_copy)

    summary = {
        "result": "pass",
        "artifacts_dir": str(artifacts_dir),
        "left_simulator_log": str(artifacts_dir / "left_simulator.log"),
        "right_simulator_log": str(artifacts_dir / "right_simulator.log"),
        "server_log": str(artifacts_dir / "rb_servo_server.log"),
        "state_stream": str(artifacts_dir / "state_stream.jsonl"),
        "invalid_state_packets": capture.invalid_packets,
        "state_packets": len(capture.snapshots),
        "target_state_packets": target_packets,
        "servo_log": validate_servo_log(servo_log_copy, target_seq, left_target, right_target),
        "simulator_command": args.rbsim_command,
        "caveat": "simulator-only evidence; does not prove rbpodo or real robot readiness",
    }
    (artifacts_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return summary


def main() -> int:
    args = parse_args()
    try:
        summary = run_smoke(args)
    except SmokeError as exc:
        args.artifacts_dir.mkdir(parents=True, exist_ok=True)
        failure = {
            "result": "fail",
            "error": str(exc),
            "artifacts_dir": str(args.artifacts_dir.resolve()),
            "left_simulator_log": str(args.artifacts_dir.resolve() / "left_simulator.log"),
            "right_simulator_log": str(args.artifacts_dir.resolve() / "right_simulator.log"),
            "server_log": str(args.artifacts_dir.resolve() / "rb_servo_server.log"),
        }
        (args.artifacts_dir / "summary.json").write_text(json.dumps(failure, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(f"per_arm_simulator_smoke: FAIL: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
