#!/usr/bin/env python3
"""Hardware-free rb_simulator + rb_servo_server smoke runner.

The runner starts one dual-arm simulator process and the rb_servo_server rbsim
profile, sends an ArmMotion gate followed by a small joint target, then checks
that the UDP state stream and servo CSV log tell the same bounded story. It
fails closed when the simulator/backend prerequisites have not landed yet.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
SIM_ROOT = ROOT / "rb_simulator"
SERVO_ROOT = ROOT / "rb_servo_server"


class SmokeError(RuntimeError):
    """A smoke-check failure with an operator-facing message."""


@dataclass(frozen=True)
class SmokeConfig:
    simulator: Path
    simulator_config: Path
    server: Path
    server_config: Path
    artifacts_dir: Path
    command_host: str
    command_port: int
    state_host: str
    state_port: int
    startup_timeout_sec: float
    settle_sec: float
    capture_sec: float
    target_delta_deg: float
    max_state_packets: int
    max_state_bytes: int


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

                if isinstance(snapshot, dict):
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
    parser = argparse.ArgumentParser(
        description=(
            "Run one dual-arm rb_simulator process plus rb_servo_server "
            "hardware-free smoke"
        )
    )
    parser.add_argument(
        "--simulator",
        type=Path,
        default=SIM_ROOT / "build" / "rb_simulator",
        help="Dual-arm rb_simulator executable; one process owns left and right arms",
    )
    parser.add_argument(
        "--simulator-config",
        type=Path,
        default=SIM_ROOT / "config" / "dual_rb3_730e.yaml",
        help="Simulator config with one control/admin endpoint pair for both arms",
    )
    parser.add_argument("--server", type=Path, default=SERVO_ROOT / "build" / "rb_servo_server")
    parser.add_argument("--server-config", type=Path, default=SERVO_ROOT / "config" / "dual_rb_simulator.yaml")
    parser.add_argument("--artifacts-dir", type=Path, default=None)
    parser.add_argument("--command-host", default="127.0.0.1")
    parser.add_argument("--command-port", type=int, default=50010)
    parser.add_argument("--state-host", default="127.0.0.1")
    parser.add_argument("--state-port", type=int, default=50110)
    parser.add_argument("--startup-timeout-sec", type=float, default=5.0)
    parser.add_argument("--settle-sec", type=float, default=0.25)
    parser.add_argument("--capture-sec", type=float, default=2.0)
    parser.add_argument("--target-delta-deg", type=float, default=1.0)
    parser.add_argument("--max-state-packets", type=int, default=200)
    parser.add_argument("--max-state-bytes", type=int, default=1_000_000)
    parser.add_argument("--self-test", action="store_true", help="Run parser/validator self-test without launching processes")
    return parser.parse_args()


def ensure_file(path: Path, label: str, executable: bool = False) -> None:
    if not path.exists():
        raise SmokeError(
            f"missing {label}: {path}\n"
            "This smoke requires the earlier rb_simulator executable and rb_servo_server "
            "rbsim-backend/config tasks to be complete."
        )
    if executable and not os.access(path, os.X_OK):
        raise SmokeError(f"{label} is not executable: {path}")


def make_config(args: argparse.Namespace) -> SmokeConfig:
    artifacts_dir = args.artifacts_dir
    if artifacts_dir is None:
        stamp = time.strftime("%Y%m%dT%H%M%S")
        artifacts_dir = SIM_ROOT / "artifacts" / f"rbsim_servo_smoke_{stamp}"
    return SmokeConfig(
        simulator=args.simulator.resolve(),
        simulator_config=args.simulator_config.resolve(),
        server=args.server.resolve(),
        server_config=args.server_config.resolve(),
        artifacts_dir=artifacts_dir.resolve(),
        command_host=args.command_host,
        command_port=args.command_port,
        state_host=args.state_host,
        state_port=args.state_port,
        startup_timeout_sec=args.startup_timeout_sec,
        settle_sec=args.settle_sec,
        capture_sec=args.capture_sec,
        target_delta_deg=args.target_delta_deg,
        max_state_packets=args.max_state_packets,
        max_state_bytes=args.max_state_bytes,
    )


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
    except (KeyError, SmokeError):
        return False


def close_enough(actual: list[float], expected: list[float], tolerance: float = 0.05) -> bool:
    return all(abs(a - b) <= tolerance for a, b in zip(actual, expected))


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

    connected_count = sum(1 for snapshot in snapshots if is_connected_valid(snapshot))
    if connected_count == 0:
        raise SmokeError("state stream never reported both arms Connected with valid joint state")

    target_snapshot: dict[str, Any] | None = None
    for snapshot in snapshots:
        if int(snapshot.get("command_seq", -1)) < target_seq:
            continue
        left_sent = finite_joint_array(snapshot["left"].get("q_sent_deg"), "left.q_sent_deg")
        right_sent = finite_joint_array(snapshot["right"].get("q_sent_deg"), "right.q_sent_deg")
        if close_enough(left_sent, left_target) and close_enough(right_sent, right_target):
            target_snapshot = snapshot
            break

    if target_snapshot is None:
        raise SmokeError("state stream never reflected the commanded joint target in q_sent_deg")

    return {
        "packets": len(snapshots),
        "connected_valid_packets": connected_count,
        "target_tick": target_snapshot.get("tick"),
        "target_motion_state": target_snapshot.get("motion_state"),
    }


def latest_servo_log(log_dir: Path) -> Path:
    candidates = sorted(log_dir.glob("*.csv"), key=lambda path: path.stat().st_mtime)
    if not candidates:
        raise SmokeError(f"no servo CSV log found under {log_dir}")
    return candidates[-1]


def validate_servo_log(path: Path, target_seq: int, left_target: list[float], right_target: list[float]) -> dict[str, Any]:
    required = [
        "command_seq",
        "left_send_ok",
        "right_send_ok",
        "logger_dropped_samples",
        *[f"left_q_sent_{i}" for i in range(6)],
        *[f"right_q_sent_{i}" for i in range(6)],
    ]
    rows = 0
    send_failures = 0
    dropped_max = 0
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
            dropped_max = max(dropped_max, int(float(row["logger_dropped_samples"])))
            if int(float(row["command_seq"])) >= target_seq and close_enough(left_sent, left_target) and close_enough(right_sent, right_target):
                target_rows += 1

    if rows == 0:
        raise SmokeError(f"servo log contains no samples: {path}")
    if send_failures:
        raise SmokeError(f"servo log recorded send failures: {send_failures}")
    if dropped_max:
        raise SmokeError(f"servo log recorded dropped samples: {dropped_max}")
    if target_rows == 0:
        raise SmokeError("servo log never reflected the commanded joint target")

    return {"path": str(path), "rows": rows, "target_rows": target_rows, "dropped_samples_max": dropped_max}


def start_process(command: list[str], cwd: Path, output_path: Path) -> subprocess.Popen[str]:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output = output_path.open("w", encoding="utf-8")
    try:
        return subprocess.Popen(
            command,
            cwd=str(cwd),
            stdout=output,
            stderr=subprocess.STDOUT,
            text=True,
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


def run_smoke(config: SmokeConfig) -> dict[str, Any]:
    ensure_file(config.simulator, "rb_simulator executable", executable=True)
    ensure_file(config.simulator_config, "rb_simulator config")
    ensure_file(config.server, "rb_servo_server binary", executable=True)
    ensure_file(config.server_config, "rb_servo_server rbsim config")

    config.artifacts_dir.mkdir(parents=True, exist_ok=True)
    state_path = config.artifacts_dir / "state_snapshots.jsonl"
    capture = StateCapture(
        config.state_host,
        config.state_port,
        state_path,
        config.max_state_packets,
        config.max_state_bytes,
    )
    simulator_proc: subprocess.Popen[str] | None = None
    server_proc: subprocess.Popen[str] | None = None

    capture.start()
    try:
        simulator_proc = start_process(
            [str(config.simulator), "--config", str(config.simulator_config)],
            cwd=SIM_ROOT,
            output_path=config.artifacts_dir / "simulator.log",
        )
        time.sleep(config.settle_sec)
        if simulator_proc.poll() is not None:
            raise SmokeError(f"rb_simulator exited early with code {simulator_proc.returncode}")

        server_proc = start_process(
            [str(config.server), "--config", str(config.server_config)],
            cwd=config.artifacts_dir,
            output_path=config.artifacts_dir / "rb_servo_server.log",
        )

        first = wait_for_snapshot(capture, is_connected_valid, config.startup_timeout_sec, "connected valid startup")
        left_initial = finite_joint_array(first["left"].get("q_actual_deg"), "left.q_actual_deg")
        right_initial = finite_joint_array(first["right"].get("q_actual_deg"), "right.q_actual_deg")

        arm_motion = lifecycle_command("ArmMotion")
        send_udp_command(config.command_host, config.command_port, arm_motion)
        wait_for_snapshot(
            capture,
            lambda snapshot: int(snapshot.get("command_seq", -1)) >= int(arm_motion["seq"]),
            config.startup_timeout_sec,
            "ArmMotion command_seq",
        )

        left_target = list(left_initial)
        right_target = list(right_initial)
        left_target[0] += config.target_delta_deg
        right_target[0] -= config.target_delta_deg
        joint_target = joint_target_command(left_target, right_target)
        send_udp_command(config.command_host, config.command_port, joint_target)

        wait_for_snapshot(
            capture,
            lambda snapshot: int(snapshot.get("command_seq", -1)) >= int(joint_target["seq"]),
            config.startup_timeout_sec,
            "JointTarget command_seq",
        )
        time.sleep(config.capture_sec)
    finally:
        terminate_process(server_proc)
        terminate_process(simulator_proc)
        capture.stop()

    target_seq = int(joint_target["seq"])
    state_summary = validate_state_stream(capture.snapshots, target_seq, left_target, right_target)
    log_summary = validate_servo_log(latest_servo_log(config.artifacts_dir / "logs"), target_seq, left_target, right_target)
    summary = {
        "result": "pass",
        "artifacts_dir": str(config.artifacts_dir),
        "state_artifact": str(state_path),
        "state_truncated": capture.truncated,
        "invalid_state_packets": capture.invalid_packets,
        "target_delta_deg": config.target_delta_deg,
        "state": state_summary,
        "servo_log": log_summary,
        "caveat": "simulator-only evidence; does not prove Rainbow rbsim or real robot readiness",
    }
    (config.artifacts_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
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
                "left": {
                    "connection_state": "Connected",
                    "has_valid_joint_state": True,
                    "q_actual_deg": left,
                    "q_sent_deg": left,
                },
                "right": {
                    "connection_state": "Connected",
                    "has_valid_joint_state": True,
                    "q_actual_deg": right,
                    "q_sent_deg": right,
                },
            }
        ]
        validate_state_stream(snapshots, 10, left, right)

        log_path = tmp_path / "servo.csv"
        fieldnames = [
            "command_seq",
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

    config = make_config(args)
    try:
        summary = run_smoke(config)
    except SmokeError as exc:
        print(f"rbsim_servo_smoke: FAIL: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
