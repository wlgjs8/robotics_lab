#!/usr/bin/env python3
"""Per-arm rb_simulator + rb_servo_server hardware-free smoke runner."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path
from typing import Any
from urllib.parse import urlparse


ROOT = Path(__file__).resolve().parents[2]
SIM_ROOT = ROOT / "rb_simulator"
SIM_SRC = SIM_ROOT / "src"
SERVO_ROOT = ROOT / "rb_servo_server"

sys.path.insert(0, str(SIM_SRC))

from rbsim import PROTOCOL_VERSION, load_simulator_config  # noqa: E402


class SmokeError(RuntimeError):
    """A smoke-check failure with an operator-facing message."""


class StateCapture:
    def __init__(self, host: str, port: int, output_path: Path, max_packets: int, max_bytes: int) -> None:
        self.host = host
        self.port = port
        self.output_path = output_path
        self.max_packets = max_packets
        self.max_bytes = max_bytes
        self.snapshots: list[dict[str, Any]] = []
        self.invalid_packets = 0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._sock: socket.socket | None = None
        self._bytes_written = 0
        self._truncated = False

    @property
    def truncated(self) -> bool:
        return self._truncated

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

                if not isinstance(snapshot, dict):
                    self.invalid_packets += 1
                    continue

                self.snapshots.append(snapshot)
                if len(self.snapshots) > self.max_packets:
                    self.snapshots = self.snapshots[-self.max_packets :]
                    self._truncated = True

                encoded = json.dumps(snapshot, sort_keys=True, separators=(",", ":")) + "\n"
                encoded_len = len(encoded.encode("utf-8"))
                if self._bytes_written + encoded_len <= self.max_bytes:
                    out.write(encoded)
                    self._bytes_written += encoded_len
                else:
                    self._truncated = True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--left-simulator-command",
        "--rbsim-command",
        dest="left_simulator_command",
        default=os.environ.get("RBSIM_COMMAND", "python3 -m rbsim"),
        help="Command used to launch the left simulator process",
    )
    parser.add_argument(
        "--right-simulator-command",
        dest="right_simulator_command",
        default=None,
        help="Command used to launch the right simulator process; defaults to the left command",
    )
    parser.add_argument(
        "--left-simulator-config",
        "--left-config",
        dest="left_simulator_config",
        type=Path,
        default=SIM_ROOT / "config" / "left_rb3_730e.yaml",
    )
    parser.add_argument(
        "--right-simulator-config",
        "--right-config",
        dest="right_simulator_config",
        type=Path,
        default=SIM_ROOT / "config" / "right_rb3_730e.yaml",
    )
    parser.add_argument("--server", type=Path, default=SERVO_ROOT / "build" / "hardware_free_gate" / "rb_servo_server")
    parser.add_argument("--server-config", type=Path, default=SERVO_ROOT / "config" / "dual_simulator.yaml")
    parser.add_argument(
        "--artifact-dir",
        "--artifacts-dir",
        dest="artifact_dir",
        type=Path,
        default=None,
    )
    parser.add_argument("--command-host", default="127.0.0.1")
    parser.add_argument("--command-port", type=int, default=50010)
    parser.add_argument("--state-host", default="127.0.0.1")
    parser.add_argument("--state-port", type=int, default=50110)
    parser.add_argument("--startup-timeout-sec", type=float, default=6.0)
    parser.add_argument("--settle-sec", type=float, default=0.15)
    parser.add_argument("--capture-sec", type=float, default=1.5)
    parser.add_argument("--target-delta-deg", type=float, default=1.0)
    parser.add_argument("--max-state-packets", type=int, default=300)
    parser.add_argument("--max-state-bytes", type=int, default=1_000_000)
    parser.add_argument("--self-test", action="store_true", help="Run parser/validator self-test without launching processes")
    return parser.parse_args()


def ensure_file(path: Path, label: str, executable: bool = False) -> None:
    if not path.exists():
        raise SmokeError(f"missing {label}: {path}")
    if executable and not os.access(path, os.X_OK):
        raise SmokeError(f"{label} is not executable: {path}")


def finite_joint_array(value: Any, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != 6:
        raise SmokeError(f"{label} must be a 6-element list")
    out: list[float] = []
    for item in value:
        try:
            number = float(item)
        except (TypeError, ValueError) as exc:
            raise SmokeError(f"{label} contains non-numeric value {item!r}") from exc
        if not math.isfinite(number):
            raise SmokeError(f"{label} contains non-finite value {item!r}")
        out.append(number)
    return out


def parse_tcp_endpoint(endpoint: str) -> tuple[str, int]:
    parsed = urlparse(endpoint)
    if parsed.scheme != "tcp" or parsed.hostname is None or parsed.port is None:
        raise SmokeError(f"expected tcp://host:port endpoint, got {endpoint!r}")
    return (parsed.hostname, int(parsed.port))


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


def tcp_jsonl_request(address: tuple[str, int], request: dict[str, Any]) -> dict[str, Any]:
    payload = json.dumps(request, separators=(",", ":")).encode("utf-8") + b"\n"
    with socket.create_connection(address, timeout=1.0) as sock:
        sock.sendall(payload)
        chunks: list[bytes] = []
        while True:
            chunk = sock.recv(1)
            if not chunk or chunk == b"\n":
                break
            chunks.append(chunk)
    return json.loads(b"".join(chunks).decode("utf-8"))


def wrong_arm_request(address: tuple[str, int], requested_arm: str, request_id: str) -> dict[str, Any]:
    return tcp_jsonl_request(
        address,
        {
            "schema_version": PROTOCOL_VERSION,
            "request_id": request_id,
            "op": "read_state",
            "arm": requested_arm,
            "params": {},
        },
    )


def validate_wrong_arm_response(response: dict[str, Any], label: str) -> dict[str, Any]:
    if response.get("ok") is not False:
        raise SmokeError(f"{label} unexpectedly accepted wrong-arm request: {response}")
    error = response.get("error")
    if not isinstance(error, dict):
        raise SmokeError(f"{label} wrong-arm response has no error object: {response}")
    if error.get("name") != "wrong_arm" or int(error.get("code", 0)) != 1005:
        raise SmokeError(f"{label} wrong-arm response was not explicit wrong_arm: {response}")
    return {"error": error.get("name"), "code": int(error.get("code", 0))}


def is_connected_valid(snapshot: dict[str, Any]) -> bool:
    try:
        if snapshot.get("schema_version") != 1:
            return False
        if snapshot.get("fault_latched") is not False:
            return False
        for arm in ("left", "right"):
            arm_state = snapshot[arm]
            if arm_state.get("connection_state") != "Connected":
                return False
            if arm_state.get("has_valid_joint_state") is not True:
                return False
            finite_joint_array(arm_state.get("q_actual_deg"), f"{arm}.q_actual_deg")
            finite_joint_array(arm_state.get("q_sent_deg"), f"{arm}.q_sent_deg")
        return True
    except (KeyError, SmokeError):
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


def snapshot_after_command(snapshot: dict[str, Any], command: dict[str, Any]) -> bool:
    try:
        return int(snapshot.get("host_time_ns", -1)) >= int(command["host_time_ns"])
    except (KeyError, TypeError, ValueError):
        return False


def arm_motion_observed(snapshot: dict[str, Any], command: dict[str, Any]) -> bool:
    return (
        snapshot_after_command(snapshot, command)
        and snapshot.get("motion_state") in {"ArmedHold", "Running"}
        and snapshot.get("safety_verdict") in {None, "Ok", "JointLimitClamped"}
        and snapshot.get("fault_latched") is False
    )


def send_udp_command(host: str, port: int, message: dict[str, Any]) -> None:
    payload = json.dumps(message, separators=(",", ":")).encode("utf-8")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.sendto(payload, (host, port))


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


def validate_state_stream(
    snapshots: list[dict[str, Any]],
    target_seq: int,
    left_target: list[float],
    right_target: list[float],
) -> dict[str, Any]:
    if not snapshots:
        raise SmokeError("state stream produced no packets")

    invalid_schema = [s.get("schema_version") for s in snapshots if s.get("schema_version") != 1]
    if invalid_schema:
        raise SmokeError("state stream contains snapshots with unsupported schema_version")

    faulted = [s for s in snapshots if s.get("fault_latched") is True]
    if faulted:
        raise SmokeError("normal smoke latched a fault")

    connected_count = sum(1 for snapshot in snapshots if is_connected_valid(snapshot))
    if connected_count == 0:
        raise SmokeError("state stream never reported both arms Connected with valid joint state")

    target_snapshot: dict[str, Any] | None = None
    safe_verdicts = {"Ok", "JointLimitClamped"}
    unsafe_verdicts: set[str] = set()
    for snapshot in snapshots:
        verdict = snapshot.get("safety_verdict")
        if isinstance(verdict, str) and verdict not in safe_verdicts:
            unsafe_verdicts.add(verdict)
        if int(snapshot.get("command_seq", -1)) < target_seq:
            continue
        left_sent = finite_joint_array(snapshot["left"].get("q_sent_deg"), "left.q_sent_deg")
        right_sent = finite_joint_array(snapshot["right"].get("q_sent_deg"), "right.q_sent_deg")
        if close_enough(left_sent, left_target) and close_enough(right_sent, right_target):
            target_snapshot = snapshot
            break

    if unsafe_verdicts:
        raise SmokeError("state stream contains unsafe command verdicts: " + ", ".join(sorted(unsafe_verdicts)))
    if target_snapshot is None:
        raise SmokeError("state stream never reflected the commanded joint target in q_sent_deg")
    if target_snapshot.get("left", {}).get("send_ok") is not True:
        raise SmokeError("target state did not report left send_ok")
    if target_snapshot.get("right", {}).get("send_ok") is not True:
        raise SmokeError("target state did not report right send_ok")

    left_sent = finite_joint_array(target_snapshot["left"].get("q_sent_deg"), "left.q_sent_deg")
    right_sent = finite_joint_array(target_snapshot["right"].get("q_sent_deg"), "right.q_sent_deg")
    if close_enough(left_sent, right_sent):
        raise SmokeError("left/right sent targets unexpectedly match; cross-arm confusion cannot be ruled out")

    return {
        "packets": len(snapshots),
        "connected_valid_packets": connected_count,
        "target_tick": target_snapshot.get("tick"),
        "target_motion_state": target_snapshot.get("motion_state"),
        "target_safety_verdict": target_snapshot.get("safety_verdict"),
    }


def latest_servo_log(log_dir: Path) -> Path | None:
    candidates = sorted(log_dir.glob("*.csv"), key=lambda path: path.stat().st_mtime)
    return candidates[-1] if candidates else None


def validate_servo_log(path: Path, target_seq: int, left_target: list[float], right_target: list[float]) -> dict[str, Any]:
    required = [
        "command_seq",
        "safety_verdict",
        "fault_latched",
        "left_send_ok",
        "right_send_ok",
        "logger_dropped_samples",
        *[f"left_q_sent_{i}" for i in range(6)],
        *[f"right_q_sent_{i}" for i in range(6)],
    ]
    rows = 0
    send_failures = 0
    dropped_max = 0
    fault_rows = 0
    unsafe_verdicts: set[str] = set()
    target_rows = 0
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        missing = [column for column in required if column not in (reader.fieldnames or [])]
        if missing:
            raise SmokeError("servo log missing required columns: " + ", ".join(missing))
        for row in reader:
            rows += 1
            left_sent = finite_joint_array([row[f"left_q_sent_{i}"] for i in range(6)], "log.left_q_sent")
            right_sent = finite_joint_array([row[f"right_q_sent_{i}"] for i in range(6)], "log.right_q_sent")
            if row["left_send_ok"].strip().lower() not in {"1", "true", "yes", "ok"}:
                send_failures += 1
            if row["right_send_ok"].strip().lower() not in {"1", "true", "yes", "ok"}:
                send_failures += 1
            if row["fault_latched"].strip().lower() in {"1", "true", "yes"}:
                fault_rows += 1
            verdict = row["safety_verdict"].strip()
            if verdict not in {"Ok", "JointLimitClamped"}:
                unsafe_verdicts.add(verdict)
            dropped_max = max(dropped_max, int(float(row["logger_dropped_samples"])))
            if int(float(row["command_seq"])) >= target_seq and close_enough(left_sent, left_target) and close_enough(right_sent, right_target):
                target_rows += 1

    if rows == 0:
        raise SmokeError(f"servo log contains no samples: {path}")
    if send_failures:
        raise SmokeError(f"servo log recorded send failures: {send_failures}")
    if fault_rows:
        raise SmokeError(f"servo log recorded fault-latched rows: {fault_rows}")
    if unsafe_verdicts:
        raise SmokeError("servo log contains unsafe verdicts: " + ", ".join(sorted(unsafe_verdicts)))
    if dropped_max:
        raise SmokeError(f"servo log recorded dropped samples: {dropped_max}")
    if target_rows == 0:
        raise SmokeError("servo log never reflected the commanded joint target")

    return {"path": str(path), "rows": rows, "target_rows": target_rows, "dropped_samples_max": dropped_max}


def start_process(command: list[str], cwd: Path, output_path: Path, env: dict[str, str] | None = None) -> subprocess.Popen[str]:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output = output_path.open("w", encoding="utf-8")
    try:
        return subprocess.Popen(
            command,
            cwd=str(cwd),
            stdout=output,
            stderr=subprocess.STDOUT,
            text=True,
            env=env,
        )
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


def process_command(command_text: str, label: str) -> list[str]:
    command = shlex.split(command_text)
    if not command:
        raise SmokeError(f"{label} resolved to an empty command")
    return command


def run_smoke(args: argparse.Namespace) -> dict[str, Any]:
    left_config_path = args.left_simulator_config.resolve()
    right_config_path = args.right_simulator_config.resolve()
    ensure_file(left_config_path, "left simulator config")
    ensure_file(right_config_path, "right simulator config")
    ensure_file(args.server.resolve(), "rb_servo_server binary", executable=True)
    ensure_file(args.server_config.resolve(), "rb_servo_server simulator config")

    left_sim_config = load_simulator_config(left_config_path)
    right_sim_config = load_simulator_config(right_config_path)
    if left_sim_config.arm != "left":
        raise SmokeError(f"left simulator config must declare simulator.arm: left, got {left_sim_config.arm!r}")
    if right_sim_config.arm != "right":
        raise SmokeError(f"right simulator config must declare simulator.arm: right, got {right_sim_config.arm!r}")

    left_control = parse_tcp_endpoint(left_sim_config.control_bind)
    right_control = parse_tcp_endpoint(right_sim_config.control_bind)

    artifact_dir = args.artifact_dir
    if artifact_dir is None:
        stamp = time.strftime("%Y%m%dT%H%M%S")
        artifact_dir = SIM_ROOT / "artifacts" / f"rbsim_servo_smoke_{stamp}"
    artifact_dir = artifact_dir.resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)

    left_command = process_command(args.left_simulator_command, "left simulator command")
    right_command = process_command(args.right_simulator_command or args.left_simulator_command, "right simulator command")

    env = os.environ.copy()
    env["PYTHONPATH"] = str(SIM_SRC) + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")

    capture = StateCapture(
        args.state_host,
        args.state_port,
        artifact_dir / "state_stream.jsonl",
        args.max_state_packets,
        args.max_state_bytes,
    )
    left_proc: subprocess.Popen[str] | None = None
    right_proc: subprocess.Popen[str] | None = None
    server_proc: subprocess.Popen[str] | None = None
    wrong_arm_summary: dict[str, Any] = {}

    capture.start()
    try:
        left_proc = start_process(
            [*left_command, "--config", str(left_config_path)],
            cwd=ROOT,
            output_path=artifact_dir / "left_simulator.log",
            env=env,
        )
        wait_tcp(left_control[0], left_control[1], args.startup_timeout_sec, "left simulator control")

        right_proc = start_process(
            [*right_command, "--config", str(right_config_path)],
            cwd=ROOT,
            output_path=artifact_dir / "right_simulator.log",
            env=env,
        )
        wait_tcp(right_control[0], right_control[1], args.startup_timeout_sec, "right simulator control")

        wrong_arm_summary = {
            "right_request_to_left": validate_wrong_arm_response(
                wrong_arm_request(left_control, "right", "wrong-right-to-left"),
                "right request to left simulator",
            ),
            "left_request_to_right": validate_wrong_arm_response(
                wrong_arm_request(right_control, "left", "wrong-left-to-right"),
                "left request to right simulator",
            ),
        }

        server_proc = start_process(
            [str(args.server.resolve()), "--config", str(args.server_config.resolve())],
            cwd=artifact_dir,
            output_path=artifact_dir / "rb_servo_server.log",
        )

        first = wait_for_snapshot(capture, is_connected_valid, args.startup_timeout_sec, "connected valid startup")
        left_initial = finite_joint_array(first["left"].get("q_actual_deg"), "left.q_actual_deg")
        right_initial = finite_joint_array(first["right"].get("q_actual_deg"), "right.q_actual_deg")

        arm_motion = lifecycle_command("ArmMotion")
        send_udp_command(args.command_host, args.command_port, arm_motion)
        wait_for_snapshot(
            capture,
            lambda snapshot: arm_motion_observed(snapshot, arm_motion),
            args.startup_timeout_sec,
            "ArmMotion armed state",
        )

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
    state_summary = validate_state_stream(capture.snapshots, target_seq, left_target, right_target)

    servo_log_path = latest_servo_log(artifact_dir / "logs")
    servo_summary: dict[str, Any] | None = None
    if servo_log_path is not None:
        servo_log_copy = artifact_dir / "servo_log.csv"
        shutil.copy2(servo_log_path, servo_log_copy)
        servo_summary = validate_servo_log(servo_log_copy, target_seq, left_target, right_target)

    summary = {
        "result": "pass",
        "artifacts_dir": str(artifact_dir),
        "state_stream": str(artifact_dir / "state_stream.jsonl"),
        "state_truncated": capture.truncated,
        "invalid_state_packets": capture.invalid_packets,
        "left_simulator_log": str(artifact_dir / "left_simulator.log"),
        "right_simulator_log": str(artifact_dir / "right_simulator.log"),
        "server_log": str(artifact_dir / "rb_servo_server.log"),
        "servo_log": servo_summary,
        "wrong_arm": wrong_arm_summary,
        "target_delta_deg": args.target_delta_deg,
        "state": state_summary,
        "left_simulator_endpoint": left_sim_config.control_bind,
        "right_simulator_endpoint": right_sim_config.control_bind,
        "server_config": str(args.server_config.resolve()),
        "caveat": "simulator-only evidence; does not prove rbpodo or real robot readiness",
    }
    (artifact_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return summary


def self_test() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)
        left = [1.0, -30.0, 80.0, 0.0, 60.0, 0.0]
        right = [-1.0, -30.0, 80.0, 0.0, 60.0, 0.0]
        snapshots = [
            {
                "schema_version": 1,
                "tick": 1,
                "command_seq": 10,
                "motion_state": "Running",
                "safety_verdict": "Ok",
                "fault_latched": False,
                "left": {
                    "connection_state": "Connected",
                    "has_valid_joint_state": True,
                    "send_ok": True,
                    "q_actual_deg": left,
                    "q_sent_deg": left,
                },
                "right": {
                    "connection_state": "Connected",
                    "has_valid_joint_state": True,
                    "send_ok": True,
                    "q_actual_deg": right,
                    "q_sent_deg": right,
                },
            }
        ]
        validate_state_stream(snapshots, 10, left, right)
        validate_wrong_arm_response(
            {"ok": False, "error": {"name": "wrong_arm", "code": 1005}},
            "self-test wrong arm",
        )

        log_path = tmp_path / "servo.csv"
        fieldnames = [
            "command_seq",
            "safety_verdict",
            "fault_latched",
            "left_send_ok",
            "right_send_ok",
            "logger_dropped_samples",
            *[f"left_q_sent_{i}" for i in range(6)],
            *[f"right_q_sent_{i}" for i in range(6)],
        ]
        with log_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            row: dict[str, Any] = {
                "command_seq": 10,
                "safety_verdict": "Ok",
                "fault_latched": "false",
                "left_send_ok": "1",
                "right_send_ok": "1",
                "logger_dropped_samples": "0",
            }
            row.update({f"left_q_sent_{i}": left[i] for i in range(6)})
            row.update({f"right_q_sent_{i}": right[i] for i in range(6)})
            writer.writerow(row)
        validate_servo_log(log_path, 10, left, right)


def main() -> int:
    args = parse_args()
    if args.self_test:
        self_test()
        print("self-test passed")
        return 0

    artifact_dir = args.artifact_dir or SIM_ROOT / "artifacts" / "rbsim_servo_smoke_failed"
    try:
        summary = run_smoke(args)
    except (SmokeError, PermissionError) as exc:
        artifact_dir.mkdir(parents=True, exist_ok=True)
        failure = {
            "result": "fail",
            "error": str(exc),
            "artifacts_dir": str(artifact_dir.resolve()),
            "left_simulator_log": str(artifact_dir.resolve() / "left_simulator.log"),
            "right_simulator_log": str(artifact_dir.resolve() / "right_simulator.log"),
            "server_log": str(artifact_dir.resolve() / "rb_servo_server.log"),
        }
        (artifact_dir / "summary.json").write_text(json.dumps(failure, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(f"rbsim_servo_smoke: FAIL: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
