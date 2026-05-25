#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ARTIFACT_DIR=""
STACK_MODE="start-local"
CHECK_DEPS=1
ALLOW_MISSING_PINOCCHIO=0
RUN_ESTOP_RESET=1
SERVER="${ROOT_DIR}/rb_servo_server/build/pinocchio_gate/rb_servo_server"
SERVER_CONFIG="${ROOT_DIR}/rb_servo_server/config/dual_simulator_tcp_acceptance.yaml"
LEFT_CONFIG="${ROOT_DIR}/rb_simulator/config/left_rb3_730e.yaml"
RIGHT_CONFIG="${ROOT_DIR}/rb_simulator/config/right_rb3_730e.yaml"
RBSIM_COMMAND="${RBSIM_COMMAND:-python3 -m rbsim}"
COMMAND_HOST="127.0.0.1"
COMMAND_PORT="50010"
STATE_HOST="127.0.0.1"
STATE_PORT="50110"
STARTUP_TIMEOUT_SEC="8.0"
CAPTURE_SEC="1.0"
TCP_TOLERANCE_M="0.004"
TCP_ORIENTATION_TOLERANCE_RAD="0.005"

usage() {
  cat <<'USAGE'
Usage: scripts/tcp_pose_simulator_acceptance.sh [options]

Simulator-only TCP Pose/Delta acceptance runner.

Options:
  --artifact-dir DIR          Artifact directory. Default: artifacts/tcp_pose_acceptance/<timestamp>
  --assume-running            Do not start local simulator/server processes; use configured host endpoints.
  --start-local               Start host-loopback simulators/server locally. Default.
  --skip-deps                 Skip scripts/check_deps.sh --profile hardware-free.
  --allow-missing-pinocchio   Allow syntax/config checks to pass without Pinocchio; runtime FK/IK acceptance is skipped if unavailable.
  --skip-estop-reset          Skip the optional EmergencyStop/ResetFault check.
  --server PATH               rb_servo_server binary.
  --server-config PATH        FK/IK-enabled simulator server config.
  --left-config PATH          Left rb_simulator config.
  --right-config PATH         Right rb_simulator config.
  --rbsim-command COMMAND     Simulator launch command. Default: python3 -m rbsim
  --command-host HOST         UDP command host. Default: 127.0.0.1
  --command-port PORT         UDP command port. Default: 50010
  --state-host HOST           UDP state capture bind host. Default: 127.0.0.1
  --state-port PORT           UDP state capture bind port. Default: 50110
  --startup-timeout-sec SEC   Startup/state wait timeout. Default: 8.0
  --capture-sec SEC           Post-command capture duration. Default: 1.0
  --tcp-tolerance-m M         TCP x movement tolerance. Default: 0.004
  --tcp-orientation-tolerance-rad RAD
                              TCP quaternion angle tolerance for pure translations. Default: 0.005
  -h, --help                  Show this help.

The selected server config must enable kinematics.publish_tcp, kinematics.ik,
and cartesian_control for simulation. Real robot gates are intentionally unused.
USAGE
}

fail() {
  echo "tcp_pose_acceptance: FAIL: $*" >&2
  exit 2
}

pinocchio_available() {
  "${ROOT_DIR}/scripts/check_deps.sh" --profile kinematics
}

require_file() {
  local path="$1"
  local label="$2"
  [[ -f "${path}" ]] || fail "missing ${label}: ${path}"
}

require_executable() {
  local path="$1"
  local label="$2"
  require_file "${path}" "${label}"
  [[ -x "${path}" ]] || fail "${label} is not executable: ${path}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --artifact-dir)
      [[ $# -ge 2 ]] || fail "--artifact-dir requires a value"
      ARTIFACT_DIR="$2"
      shift 2
      ;;
    --assume-running)
      STACK_MODE="assume-running"
      shift
      ;;
    --start-local)
      STACK_MODE="start-local"
      shift
      ;;
    --skip-deps)
      CHECK_DEPS=0
      shift
      ;;
    --allow-missing-pinocchio)
      ALLOW_MISSING_PINOCCHIO=1
      shift
      ;;
    --skip-estop-reset)
      RUN_ESTOP_RESET=0
      shift
      ;;
    --server)
      [[ $# -ge 2 ]] || fail "--server requires a value"
      SERVER="$2"
      shift 2
      ;;
    --server-config)
      [[ $# -ge 2 ]] || fail "--server-config requires a value"
      SERVER_CONFIG="$2"
      shift 2
      ;;
    --left-config)
      [[ $# -ge 2 ]] || fail "--left-config requires a value"
      LEFT_CONFIG="$2"
      shift 2
      ;;
    --right-config)
      [[ $# -ge 2 ]] || fail "--right-config requires a value"
      RIGHT_CONFIG="$2"
      shift 2
      ;;
    --rbsim-command)
      [[ $# -ge 2 ]] || fail "--rbsim-command requires a value"
      RBSIM_COMMAND="$2"
      shift 2
      ;;
    --command-host)
      [[ $# -ge 2 ]] || fail "--command-host requires a value"
      COMMAND_HOST="$2"
      shift 2
      ;;
    --command-port)
      [[ $# -ge 2 ]] || fail "--command-port requires a value"
      COMMAND_PORT="$2"
      shift 2
      ;;
    --state-host)
      [[ $# -ge 2 ]] || fail "--state-host requires a value"
      STATE_HOST="$2"
      shift 2
      ;;
    --state-port)
      [[ $# -ge 2 ]] || fail "--state-port requires a value"
      STATE_PORT="$2"
      shift 2
      ;;
    --startup-timeout-sec)
      [[ $# -ge 2 ]] || fail "--startup-timeout-sec requires a value"
      STARTUP_TIMEOUT_SEC="$2"
      shift 2
      ;;
    --capture-sec)
      [[ $# -ge 2 ]] || fail "--capture-sec requires a value"
      CAPTURE_SEC="$2"
      shift 2
      ;;
    --tcp-tolerance-m)
      [[ $# -ge 2 ]] || fail "--tcp-tolerance-m requires a value"
      TCP_TOLERANCE_M="$2"
      shift 2
      ;;
    --tcp-orientation-tolerance-rad)
      [[ $# -ge 2 ]] || fail "--tcp-orientation-tolerance-rad requires a value"
      TCP_ORIENTATION_TOLERANCE_RAD="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      fail "unknown argument: $1"
      ;;
  esac
done

if [[ -z "${ARTIFACT_DIR}" ]]; then
  ARTIFACT_DIR="${ROOT_DIR}/artifacts/tcp_pose_acceptance/$(date -u +%Y%m%dT%H%M%SZ)"
fi

if [[ "${CHECK_DEPS}" -eq 1 ]]; then
  "${ROOT_DIR}/scripts/check_deps.sh" --profile hardware-free
fi

PINOCCHIO_READY=0
if pinocchio_available; then
  PINOCCHIO_READY=1
elif [[ "${ALLOW_MISSING_PINOCCHIO}" -eq 0 ]]; then
  fail "Pinocchio is required for simulator TCP FK/IK acceptance. Install the pinocchio CMake package or pass --allow-missing-pinocchio for syntax/config-only validation."
else
  echo "tcp_pose_acceptance: Pinocchio unavailable; continuing only because --allow-missing-pinocchio was provided." >&2
fi

require_file "${ROOT_DIR}/rb_servo_server/tools/send_arm_motion.py" "ArmMotion send tool"
require_file "${ROOT_DIR}/rb_servo_server/tools/send_tcp_delta.py" "TCP delta send tool from P3-C"
require_file "${ROOT_DIR}/rb_servo_server/tools/send_tcp_pose_target.py" "TCP pose target send tool from P3-C"
require_file "${ROOT_DIR}/rb_servo_server/src/network/state_publisher.cpp" "P2-C state publisher source"
require_file "${ROOT_DIR}/rb_servo_server/src/control/cartesian_controller.cpp" "P3-B CartesianController source"
require_file "${ROOT_DIR}/rb_servo_server/src/control/dual_arm_servo_loop.cpp" "P3-B servo loop source"
require_file "${LEFT_CONFIG}" "left simulator config"
require_file "${RIGHT_CONFIG}" "right simulator config"
require_file "${SERVER_CONFIG}" "rb_servo_server simulator config"

grep -q "has_valid_tcp_pose" "${ROOT_DIR}/rb_servo_server/src/network/state_publisher.cpp" \
  || fail "P2-C FK TCP state publish surface is missing has_valid_tcp_pose serialization"
grep -q "tcp_stand" "${ROOT_DIR}/rb_servo_server/src/network/state_publisher.cpp" \
  || fail "P2-C FK TCP state publish surface is missing tcp_stand serialization"
grep -q "solveIk" "${ROOT_DIR}/rb_servo_server/src/control/cartesian_controller.cpp" \
  || fail "P3-B CartesianController IK path appears missing"
grep -q "isCartesianMode" "${ROOT_DIR}/rb_servo_server/src/control/dual_arm_servo_loop.cpp" \
  || fail "P3-B servo loop Cartesian command path appears missing"

python3 "${ROOT_DIR}/rb_servo_server/tools/send_tcp_delta.py" \
  --dry-run --frame stand \
  --left 0.005 0 0 0 0 0 \
  --right 0 0 0 0 0 0 >/dev/null
python3 "${ROOT_DIR}/rb_servo_server/tools/send_tcp_delta.py" \
  --dry-run --frame local \
  --left 0.005 0 0 0 0 0 \
  --right 0 0 0 0 0 0 >/dev/null
python3 "${ROOT_DIR}/rb_servo_server/tools/send_tcp_pose_target.py" \
  --dry-run --left 10 0 10 0 0 0 >/dev/null

python3 - "${SERVER_CONFIG}" <<'PY'
from pathlib import Path
import re
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")

checks = [
    ("left_robot section", r"(?m)^left_robot:\s*$"),
    ("left_robot backend_type simulator", r"(?ms)^left_robot:.*?^\s+backend_type:\s+simulator\s*$"),
    ("left_robot run_mode simulation", r"(?ms)^left_robot:.*?^\s+run_mode:\s+simulation\s*$"),
    ("right_robot section", r"(?m)^right_robot:\s*$"),
    ("right_robot backend_type simulator", r"(?ms)^right_robot:.*?^\s+backend_type:\s+simulator\s*$"),
    ("right_robot run_mode simulation", r"(?ms)^right_robot:.*?^\s+run_mode:\s+simulation\s*$"),
    ("servo section", r"(?m)^servo:\s*$"),
    ("servo.send_servo_commands true", r"(?ms)^servo:.*?^\s+send_servo_commands:\s+true\s*$"),
    ("kinematics section", r"(?m)^kinematics:\s*$"),
    ("kinematics.enable true", r"(?ms)^kinematics:.*?^\s+enable:\s+true\s*$"),
    ("kinematics.provider pinocchio", r"(?ms)^kinematics:.*?^\s+provider:\s+pinocchio\s*$"),
    ("kinematics.urdf path", r"(?ms)^kinematics:.*?^\s+urdf:\s+\"?[^\n\"]+\"?\s*$"),
    ("kinematics.publish_tcp true", r"(?ms)^kinematics:.*?^\s+publish_tcp:\s+true\s*$"),
    ("kinematics.ik section", r"(?ms)^kinematics:.*?^\s+ik:\s*$"),
    ("kinematics.ik.enable true", r"(?ms)^kinematics:.*?^\s+ik:\s*$.*?^\s+enable:\s+true\s*$"),
    ("cartesian_control section", r"(?m)^cartesian_control:\s*$"),
    ("cartesian_control.enable true", r"(?ms)^cartesian_control:.*?^\s+enable:\s+true\s*$"),
    ("cartesian_control.allow_in_simulation true", r"(?ms)^cartesian_control:.*?^\s+allow_in_simulation:\s+true\s*$"),
    ("cartesian_control.allow_in_real false", r"(?ms)^cartesian_control:.*?^\s+allow_in_real:\s+false\s*$"),
    ("cartesian_control.warn_ik_duration_us", r"(?ms)^cartesian_control:.*?^\s+warn_ik_duration_us:\s+[0-9.]+\s*$"),
    ("cartesian_control.fail_ik_duration_us", r"(?ms)^cartesian_control:.*?^\s+fail_ik_duration_us:\s+[0-9.]+\s*$"),
]

missing = [label for label, pattern in checks if re.search(pattern, text) is None]
unsafe = []
if re.search(r"(?m)^\s+run_mode:\s+real\s*$", text):
    unsafe.append("run_mode: real")
if re.search(r"(?m)^\s+backend_type:\s+rbpodo\s*$", text):
    unsafe.append("backend_type: rbpodo")
if re.search(r"(?ms)^cartesian_control:.*?^\s+allow_in_real:\s+true\s*$", text):
    unsafe.append("cartesian_control.allow_in_real: true")
if "172.28.60.200" in text:
    unsafe.append("real left robot IP 172.28.60.200")
if "172.28.60.201" in text:
    unsafe.append("real right robot IP 172.28.60.201")
orientation_tolerance = re.search(r"(?m)^\s+orientation_tolerance_rad:\s+([0-9.]+)\s*$", text)
if orientation_tolerance is None:
    missing.append("kinematics.ik.orientation_tolerance_rad")
else:
    value = float(orientation_tolerance.group(1))
    if value > 0.005:
        missing.append("kinematics.ik.orientation_tolerance_rad <= 0.005")
if missing:
    print(
        "tcp_pose_acceptance: FAIL: selected server config is not FK/IK TCP acceptance-ready: "
        + str(path),
        file=sys.stderr,
    )
    for label in missing:
        print(f"  - missing {label}", file=sys.stderr)
    print(
        "Provide --server-config with kinematics.enable=true, "
        "kinematics.provider=pinocchio, kinematics.urdf, "
        "kinematics.publish_tcp=true, kinematics.ik.enable=true, "
        "cartesian_control.enable=true, allow_in_simulation=true, "
        "allow_in_real=false, and servo.send_servo_commands=true. "
        "Build the server with RB_SERVO_ENABLE_PINOCCHIO=ON.",
        file=sys.stderr,
    )
    sys.exit(2)
if unsafe:
    print(
        "tcp_pose_acceptance: FAIL: selected server config is not simulator-only safe: "
        + str(path),
        file=sys.stderr,
    )
    for item in unsafe:
        print(f"  - unsafe {item}", file=sys.stderr)
    print("Use only run_mode: simulation and backend_type: simulator for this acceptance.", file=sys.stderr)
    sys.exit(2)
PY

if [[ "${PINOCCHIO_READY}" -eq 0 ]]; then
  echo "tcp_pose_acceptance: syntax/config checks passed; runtime FK/IK acceptance skipped because Pinocchio is unavailable." >&2
  exit 0
fi

if [[ "${STACK_MODE}" == "start-local" ]]; then
  require_executable "${SERVER}" "rb_servo_server binary"
  server_cache="$(dirname "${SERVER}")/CMakeCache.txt"
  if [[ -f "${server_cache}" ]] && ! grep -q '^RB_SERVO_ENABLE_PINOCCHIO:BOOL=ON$' "${server_cache}"; then
    fail "rb_servo_server binary was not built with RB_SERVO_ENABLE_PINOCCHIO=ON: ${SERVER}. Reconfigure a simulator acceptance build with Pinocchio enabled."
  fi
fi

PYTHONPATH="${ROOT_DIR}/rb_simulator/src${PYTHONPATH:+:${PYTHONPATH}}" \
python3 - \
  --root "${ROOT_DIR}" \
  --mode "${STACK_MODE}" \
  --artifact-dir "${ARTIFACT_DIR}" \
  --server "${SERVER}" \
  --server-config "${SERVER_CONFIG}" \
  --left-config "${LEFT_CONFIG}" \
  --right-config "${RIGHT_CONFIG}" \
  --rbsim-command "${RBSIM_COMMAND}" \
  --command-host "${COMMAND_HOST}" \
  --command-port "${COMMAND_PORT}" \
  --state-host "${STATE_HOST}" \
  --state-port "${STATE_PORT}" \
  --startup-timeout-sec "${STARTUP_TIMEOUT_SEC}" \
  --capture-sec "${CAPTURE_SEC}" \
  --tcp-tolerance-m "${TCP_TOLERANCE_M}" \
  --tcp-orientation-tolerance-rad "${TCP_ORIENTATION_TOLERANCE_RAD}" \
  --run-estop-reset "${RUN_ESTOP_RESET}" <<'PY'
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
import threading
import time
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

from rbsim import load_simulator_config


class AcceptanceError(RuntimeError):
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
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((self.host, self.port))
            sock.settimeout(0.1)
        except OSError as exc:
            raise AcceptanceError(
                f"loopback UDP state capture socket unavailable at {self.host}:{self.port}: {exc}"
            ) from exc
        self._sock = sock
        self._thread = threading.Thread(target=self._run, name="tcp-pose-state-capture", daemon=True)
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--mode", choices=("start-local", "assume-running"), required=True)
    parser.add_argument("--artifact-dir", type=Path, required=True)
    parser.add_argument("--server", type=Path, required=True)
    parser.add_argument("--server-config", type=Path, required=True)
    parser.add_argument("--left-config", type=Path, required=True)
    parser.add_argument("--right-config", type=Path, required=True)
    parser.add_argument("--rbsim-command", required=True)
    parser.add_argument("--command-host", required=True)
    parser.add_argument("--command-port", type=int, required=True)
    parser.add_argument("--state-host", required=True)
    parser.add_argument("--state-port", type=int, required=True)
    parser.add_argument("--startup-timeout-sec", type=float, required=True)
    parser.add_argument("--capture-sec", type=float, required=True)
    parser.add_argument("--tcp-tolerance-m", type=float, required=True)
    parser.add_argument("--tcp-orientation-tolerance-rad", type=float, required=True)
    parser.add_argument("--run-estop-reset", choices=("0", "1"), required=True)
    return parser.parse_args()


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


def terminate_process(proc: subprocess.Popen[str] | None) -> None:
    if proc is None or proc.poll() is not None:
        return
    proc.send_signal(signal.SIGTERM)
    try:
        proc.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=2.0)


def finite_joint_array(value: Any, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != 6:
        raise AcceptanceError(f"{label} must be a 6-element list")
    out: list[float] = []
    for item in value:
        try:
            number = float(item)
        except (TypeError, ValueError) as exc:
            raise AcceptanceError(f"{label} contains non-numeric value {item!r}") from exc
        if not math.isfinite(number):
            raise AcceptanceError(f"{label} contains non-finite value {item!r}")
        out.append(number)
    return out


def finite_tcp_pose(value: Any, label: str) -> dict[str, float]:
    if not isinstance(value, dict):
        raise AcceptanceError(f"{label} must be a TCP pose object")
    out: dict[str, float] = {}
    for key in ("x", "y", "z", "rx", "ry", "rz"):
        try:
            number = float(value[key])
        except (KeyError, TypeError, ValueError) as exc:
            raise AcceptanceError(f"{label}.{key} is missing or non-numeric") from exc
        if not math.isfinite(number):
            raise AcceptanceError(f"{label}.{key} is non-finite")
        out[key] = number
    return out


def finite_published_tcp_pose(value: Any, label: str) -> dict[str, float | list[float]]:
    pose = finite_tcp_pose(value, label)
    assert isinstance(value, dict)
    quaternion = value.get("quaternion_xyzw")
    if not isinstance(quaternion, list) or len(quaternion) != 4:
        raise AcceptanceError(f"{label}.quaternion_xyzw must be [qx, qy, qz, qw]")
    q: list[float] = []
    for index, item in enumerate(quaternion):
        try:
            number = float(item)
        except (TypeError, ValueError) as exc:
            raise AcceptanceError(f"{label}.quaternion_xyzw[{index}] is non-numeric") from exc
        if not math.isfinite(number):
            raise AcceptanceError(f"{label}.quaternion_xyzw[{index}] is non-finite")
        q.append(number)
    norm = math.sqrt(sum(component * component for component in q))
    if abs(norm - 1.0) > 1e-6:
        raise AcceptanceError(f"{label}.quaternion_xyzw must be normalized, got norm={norm}")
    for index, key in enumerate(("qx", "qy", "qz", "qw")):
        try:
            alias = float(value[key])
        except (KeyError, TypeError, ValueError) as exc:
            raise AcceptanceError(f"{label}.{key} alias is missing or non-numeric") from exc
        if not math.isfinite(alias) or abs(alias - q[index]) > 1e-9:
            raise AcceptanceError(f"{label}.{key} alias does not match quaternion_xyzw[{index}]")
    pose["quaternion_xyzw"] = q
    return pose


def quaternion_angle_distance(a: list[float], b: list[float]) -> float:
    if len(a) != 4 or len(b) != 4:
        raise AcceptanceError("quaternion angle distance requires two [qx, qy, qz, qw] arrays")
    dot = abs(sum(float(x) * float(y) for x, y in zip(a, b)))
    dot = min(1.0, max(-1.0, dot))
    return 2.0 * math.acos(dot)


def assert_quaternion_preserved(
    before: dict[str, float | list[float]],
    after: dict[str, float | list[float]],
    label: str,
    tolerance_rad: float,
) -> float:
    before_q = before.get("quaternion_xyzw")
    after_q = after.get("quaternion_xyzw")
    if not isinstance(before_q, list) or not isinstance(after_q, list):
        raise AcceptanceError(f"{label} missing published quaternion for orientation preservation check")
    angle = quaternion_angle_distance(before_q, after_q)
    if angle > tolerance_rad:
        raise AcceptanceError(
            f"{label} pure translation changed TCP quaternion by {angle} rad, tolerance {tolerance_rad}"
        )
    return angle


def close_enough(actual: list[float], expected: list[float], tolerance: float = 0.2) -> bool:
    return all(abs(a - b) <= tolerance for a, b in zip(actual, expected))


def within_joint_limits(q: list[float], label: str) -> None:
    q_min = [-170.0, -120.0, -170.0, -190.0, -120.0, -360.0]
    q_max = [170.0, 120.0, 170.0, 190.0, 120.0, 360.0]
    for index, value in enumerate(q):
        if value < q_min[index] - 1e-6 or value > q_max[index] + 1e-6:
            raise AcceptanceError(f"{label}[{index}]={value} outside configured joint limits")


def is_connected_valid(snapshot: dict[str, Any]) -> bool:
    try:
        if snapshot.get("schema_version") != 1:
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
    except (KeyError, AcceptanceError):
        return False


def has_valid_tcp(snapshot: dict[str, Any]) -> bool:
    if not is_connected_valid(snapshot):
        return False
    try:
        for arm in ("left", "right"):
            arm_state = snapshot[arm]
            if arm_state.get("has_valid_tcp_pose") is not True:
                return False
            finite_published_tcp_pose(arm_state.get("tcp_stand"), f"{arm}.tcp_stand")
        return snapshot.get("fault_latched") is False
    except (KeyError, AcceptanceError):
        return False


def wait_for_snapshot(capture: StateCapture, predicate: Any, timeout_sec: float, label: str) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_sec
    seen = 0
    while time.monotonic() < deadline:
        for snapshot in capture.snapshots[seen:]:
            if predicate(snapshot):
                return snapshot
        seen = len(capture.snapshots)
        time.sleep(0.02)
    raise AcceptanceError(f"timed out waiting for state snapshot: {label}")


def snapshot_at_or_after(snapshot: dict[str, Any], host_time_ns: int) -> bool:
    try:
        return int(snapshot.get("host_time_ns", -1)) >= int(host_time_ns)
    except (TypeError, ValueError):
        return False


def snapshot_after_command(snapshot: dict[str, Any], command: dict[str, Any]) -> bool:
    try:
        return snapshot_at_or_after(snapshot, int(command["host_time_ns"]))
    except (KeyError, TypeError, ValueError):
        return False


def arm_motion_observed(snapshot: dict[str, Any], command: dict[str, Any]) -> bool:
    return (
        snapshot_after_command(snapshot, command)
        and snapshot.get("motion_state") in {"ArmedHold", "Running"}
        and snapshot.get("safety_verdict") == "Ok"
        and snapshot.get("fault_latched") is False
    )


def emergency_stop_observed(snapshot: dict[str, Any], command: dict[str, Any]) -> bool:
    return (
        snapshot_after_command(snapshot, command)
        and (
            snapshot.get("fault_latched") is True
            or snapshot.get("safety_verdict") == "EmergencyStop"
            or snapshot.get("motion_state") in {"EmergencyLatched", "FaultLatched"}
        )
    )


def reset_fault_observed(snapshot: dict[str, Any], command: dict[str, Any]) -> bool:
    return (
        snapshot_after_command(snapshot, command)
        and snapshot.get("motion_state") in {"ConnectedHold", "ArmedHold"}
        and snapshot.get("fault_latched") is False
    )


def send_udp_command(host: str, port: int, message: dict[str, Any]) -> None:
    payload = json.dumps(message, separators=(",", ":"), allow_nan=False).encode("utf-8")
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


def tcp_delta_stand_command() -> dict[str, Any]:
    seq = time.monotonic_ns()
    return {
        "schema_version": 1,
        "seq": seq,
        "mode": "TcpDeltaStand",
        "host_time_ns": seq,
        "timeout_sec": 0.2,
        "coupled_timeout": True,
        "left": {"mode": "TcpDeltaStand", "tcp_delta_stand": [0.005, 0, 0, 0, 0, 0]},
        "right": {"mode": "TcpDeltaStand", "tcp_delta_stand": [0, 0, 0, 0, 0, 0]},
    }


def tcp_delta_local_command() -> dict[str, Any]:
    seq = time.monotonic_ns()
    return {
        "schema_version": 1,
        "seq": seq,
        "mode": "TcpDeltaLocal",
        "host_time_ns": seq,
        "timeout_sec": 0.2,
        "coupled_timeout": True,
        "left": {"mode": "TcpDeltaLocal", "tcp_delta_local": [0.005, 0, 0, 0, 0, 0]},
        "right": {"mode": "TcpDeltaLocal", "tcp_delta_local": [0, 0, 0, 0, 0, 0]},
    }


def unreachable_tcp_pose_command(current_left_pose: dict[str, float]) -> dict[str, Any]:
    seq = time.monotonic_ns()
    return {
        "schema_version": 1,
        "seq": seq,
        "mode": "Hold",
        "host_time_ns": seq,
        "timeout_sec": 0.2,
        "coupled_timeout": True,
        "left": {
            "mode": "TcpPoseTarget",
            "tcp_target_stand": [
                current_left_pose["x"] + 10.0,
                current_left_pose["y"],
                current_left_pose["z"] + 10.0,
                current_left_pose["rx"],
                current_left_pose["ry"],
                current_left_pose["rz"],
            ],
        },
        "right": {},
    }


def copy_servo_log(artifact_dir: Path) -> dict[str, Any] | None:
    log_dir = artifact_dir / "logs"
    candidates = sorted(log_dir.glob("*.csv"), key=lambda path: path.stat().st_mtime)
    if not candidates:
        return None
    target = artifact_dir / "servo_log.csv"
    shutil.copy2(candidates[-1], target)
    rows = 0
    unsafe_verdicts: set[str] = set()
    with target.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            rows += 1
            verdict = (row.get("safety_verdict") or "").strip()
            if verdict and verdict not in {"Ok", "IkFailed", "EmergencyStop", "FaultLatched"}:
                unsafe_verdicts.add(verdict)
    return {"path": str(target), "rows": rows, "other_verdicts": sorted(unsafe_verdicts)}


def nonnegative_number(value: Any, label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise AcceptanceError(f"{label} is missing or non-numeric") from exc
    if not math.isfinite(number) or number < 0.0:
        raise AcceptanceError(f"{label} must be finite and non-negative")
    return number


def validate_cartesian_telemetry(
    arm_state: dict[str, Any],
    label: str,
    *,
    expected_attempted: bool | None = None,
    expected_success: bool | None = None,
) -> dict[str, Any]:
    telemetry = arm_state.get("cartesian_solve")
    if not isinstance(telemetry, dict):
        raise AcceptanceError(f"{label}.cartesian_solve must be an object")
    attempted = telemetry.get("attempted")
    success = telemetry.get("success")
    if not isinstance(attempted, bool):
        raise AcceptanceError(f"{label}.cartesian_solve.attempted must be bool")
    if not isinstance(success, bool):
        raise AcceptanceError(f"{label}.cartesian_solve.success must be bool")
    if expected_attempted is not None and attempted is not expected_attempted:
        raise AcceptanceError(f"{label}.cartesian_solve.attempted expected {expected_attempted}, got {attempted}")
    if expected_success is not None and success is not expected_success:
        raise AcceptanceError(f"{label}.cartesian_solve.success expected {expected_success}, got {success}")
    for key in ("status", "reason", "ik_status", "ik_reason"):
        if key not in telemetry or not isinstance(telemetry[key], str):
            raise AcceptanceError(f"{label}.cartesian_solve.{key} must be a string")
    for key in (
        "fk_duration_us",
        "ik_duration_us",
        "position_error_m",
        "orientation_error_rad",
        "warn_ik_duration_us",
        "fail_ik_duration_us",
    ):
        telemetry[key] = nonnegative_number(telemetry.get(key), f"{label}.cartesian_solve.{key}")
    iterations = telemetry.get("ik_iterations")
    if not isinstance(iterations, int) or iterations < 0:
        raise AcceptanceError(f"{label}.cartesian_solve.ik_iterations must be a non-negative integer")
    for key in ("ik_timed_out", "ik_warn_duration_exceeded", "ik_fail_duration_exceeded"):
        if not isinstance(telemetry.get(key), bool):
            raise AcceptanceError(f"{label}.cartesian_solve.{key} must be bool")
    nonnegative_number(arm_state.get("fk_duration_us"), f"{label}.fk_duration_us")
    return telemetry


def percentile(values: list[float], pct: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, math.ceil((pct / 100.0) * len(ordered)) - 1))
    return ordered[index]


def successful_ik_latency_summary(snapshots: list[dict[str, Any]]) -> dict[str, Any]:
    durations: list[float] = []
    iterations: list[int] = []
    timed_out_count = 0
    warn_threshold = 3000.0
    fail_threshold = 5000.0
    for snapshot in snapshots:
        for arm in ("left", "right"):
            arm_state = snapshot.get(arm)
            if not isinstance(arm_state, dict):
                continue
            telemetry = arm_state.get("cartesian_solve")
            if not isinstance(telemetry, dict) or telemetry.get("attempted") is not True:
                continue
            try:
                warn = nonnegative_number(telemetry.get("warn_ik_duration_us"), f"{arm}.warn_ik_duration_us")
                fail = nonnegative_number(telemetry.get("fail_ik_duration_us"), f"{arm}.fail_ik_duration_us")
                duration = nonnegative_number(telemetry.get("ik_duration_us"), f"{arm}.ik_duration_us")
            except AcceptanceError:
                continue
            if warn > 0.0:
                warn_threshold = warn
            if fail > 0.0:
                fail_threshold = fail
            if telemetry.get("ik_timed_out") is True:
                timed_out_count += 1
            if telemetry.get("success") is True:
                durations.append(duration)
                iterations_value = telemetry.get("ik_iterations")
                if isinstance(iterations_value, int):
                    iterations.append(iterations_value)
    return {
        "count": len(durations),
        "min": min(durations) if durations else 0.0,
        "p50": percentile(durations, 50.0),
        "p95": percentile(durations, 95.0),
        "max": max(durations) if durations else 0.0,
        "threshold_p95": warn_threshold,
        "threshold_max": fail_threshold,
        "timed_out_count": timed_out_count,
        "iterations_min": min(iterations) if iterations else 0,
        "iterations_max": max(iterations) if iterations else 0,
    }


def run_acceptance(args: argparse.Namespace) -> dict[str, Any]:
    artifact_dir = args.artifact_dir.resolve()
    artifact_dir.mkdir(parents=True, exist_ok=True)

    left_proc: subprocess.Popen[str] | None = None
    right_proc: subprocess.Popen[str] | None = None
    server_proc: subprocess.Popen[str] | None = None

    capture = StateCapture(args.state_host, args.state_port, artifact_dir / "state_stream.jsonl")
    capture.start()
    try:
        if args.mode == "start-local":
            left_config = load_simulator_config(args.left_config.resolve())
            right_config = load_simulator_config(args.right_config.resolve())
            if left_config.arm != "left":
                raise AcceptanceError(f"left simulator config declares arm={left_config.arm!r}")
            if right_config.arm != "right":
                raise AcceptanceError(f"right simulator config declares arm={right_config.arm!r}")

            left_endpoint = parse_tcp_endpoint(left_config.control_bind)
            right_endpoint = parse_tcp_endpoint(right_config.control_bind)
            rbsim_command = shlex.split(args.rbsim_command)
            if not rbsim_command:
                raise AcceptanceError("RBSIM command is empty")

            env = os.environ.copy()
            sim_src = args.root / "rb_simulator" / "src"
            env["PYTHONPATH"] = str(sim_src) + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")

            left_proc = start_process(
                [*rbsim_command, "--config", str(args.left_config.resolve())],
                cwd=args.root,
                output_path=artifact_dir / "left_simulator.log",
                env=env,
            )
            wait_tcp(left_endpoint[0], left_endpoint[1], args.startup_timeout_sec, "left simulator control")

            right_proc = start_process(
                [*rbsim_command, "--config", str(args.right_config.resolve())],
                cwd=args.root,
                output_path=artifact_dir / "right_simulator.log",
                env=env,
            )
            wait_tcp(right_endpoint[0], right_endpoint[1], args.startup_timeout_sec, "right simulator control")

            server_proc = start_process(
                [str(args.server.resolve()), "--config", str(args.server_config.resolve())],
                cwd=artifact_dir,
                output_path=artifact_dir / "rb_servo_server.log",
            )
        else:
            for name in ("left_simulator.log", "right_simulator.log", "rb_servo_server.log"):
                (artifact_dir / name).write_text(
                    "not captured: --assume-running was used\n",
                    encoding="utf-8",
                )

        initial = wait_for_snapshot(capture, has_valid_tcp, args.startup_timeout_sec, "FK TCP pose on both arms")
        initial_left_tcp = finite_published_tcp_pose(initial["left"].get("tcp_stand"), "left.initial.tcp_stand")
        initial_right_tcp = finite_published_tcp_pose(initial["right"].get("tcp_stand"), "right.initial.tcp_stand")

        arm_motion = lifecycle_command("ArmMotion")
        send_udp_command(args.command_host, args.command_port, arm_motion)
        wait_for_snapshot(
            capture,
            lambda snapshot: arm_motion_observed(snapshot, arm_motion),
            args.startup_timeout_sec,
            "ArmMotion observed without fault latch",
        )

        delta_command = tcp_delta_stand_command()
        send_udp_command(args.command_host, args.command_port, delta_command)
        delta_snapshot = wait_for_snapshot(
            capture,
            lambda snapshot: (
                int(snapshot.get("command_seq", -1)) >= int(delta_command["seq"])
                and snapshot.get("safety_verdict") == "Ok"
                and snapshot.get("fault_latched") is False
            ),
            args.startup_timeout_sec,
            "TcpDeltaStand Ok verdict",
        )
        finite_joint_array(delta_snapshot["left"].get("q_sent_deg"), "left.delta.q_sent_deg")
        finite_joint_array(delta_snapshot["right"].get("q_sent_deg"), "right.delta.q_sent_deg")
        within_joint_limits(delta_snapshot["left"]["q_sent_deg"], "left.delta.q_sent_deg")
        within_joint_limits(delta_snapshot["right"]["q_sent_deg"], "right.delta.q_sent_deg")
        validate_cartesian_telemetry(delta_snapshot["left"], "left.delta", expected_attempted=True, expected_success=True)
        validate_cartesian_telemetry(delta_snapshot["right"], "right.delta", expected_attempted=True, expected_success=True)

        time.sleep(args.capture_sec)
        final_after_ns = time.monotonic_ns()
        final = wait_for_snapshot(
            capture,
            lambda snapshot: snapshot_at_or_after(snapshot, final_after_ns) and has_valid_tcp(snapshot),
            args.startup_timeout_sec,
            "final FK TCP pose",
        )
        final_left_tcp = finite_published_tcp_pose(final["left"].get("tcp_stand"), "left.final.tcp_stand")
        final_right_tcp = finite_published_tcp_pose(final["right"].get("tcp_stand"), "right.final.tcp_stand")
        left_dx = final_left_tcp["x"] - initial_left_tcp["x"]
        right_dx = final_right_tcp["x"] - initial_right_tcp["x"]
        if left_dx <= 0.0:
            raise AcceptanceError(f"left TCP x did not move positive after +0.005 m delta: dx={left_dx}")
        if left_dx > 0.05:
            raise AcceptanceError(f"left TCP x delta {left_dx} exceeds bounded simulator acceptance limit")
        if abs(right_dx) > args.tcp_tolerance_m:
            raise AcceptanceError(f"right TCP x moved despite zero delta: dx={right_dx}")
        stand_left_orientation_delta_rad = assert_quaternion_preserved(
            initial_left_tcp,
            final_left_tcp,
            "left TcpDeltaStand",
            args.tcp_orientation_tolerance_rad,
        )
        stand_right_orientation_delta_rad = assert_quaternion_preserved(
            initial_right_tcp,
            final_right_tcp,
            "right zero TcpDeltaStand",
            args.tcp_orientation_tolerance_rad,
        )

        local_arm_motion = lifecycle_command("ArmMotion")
        send_udp_command(args.command_host, args.command_port, local_arm_motion)
        wait_for_snapshot(
            capture,
            lambda snapshot: arm_motion_observed(snapshot, local_arm_motion),
            args.startup_timeout_sec,
            "ArmMotion refreshed before TcpDeltaLocal",
        )
        local_command = tcp_delta_local_command()
        send_udp_command(args.command_host, args.command_port, local_command)
        local_delta_snapshot = wait_for_snapshot(
            capture,
            lambda snapshot: (
                int(snapshot.get("command_seq", -1)) >= int(local_command["seq"])
                and snapshot.get("safety_verdict") == "Ok"
                and snapshot.get("fault_latched") is False
            ),
            args.startup_timeout_sec,
            "TcpDeltaLocal Ok verdict",
        )
        validate_cartesian_telemetry(local_delta_snapshot["left"], "left.local_delta", expected_attempted=True, expected_success=True)
        validate_cartesian_telemetry(local_delta_snapshot["right"], "right.local_delta", expected_attempted=True, expected_success=True)
        time.sleep(args.capture_sec)
        local_final_after_ns = time.monotonic_ns()
        local_final = wait_for_snapshot(
            capture,
            lambda snapshot: snapshot_at_or_after(snapshot, local_final_after_ns) and has_valid_tcp(snapshot),
            args.startup_timeout_sec,
            "final FK TCP pose after TcpDeltaLocal",
        )
        local_final_left_tcp = finite_published_tcp_pose(local_final["left"].get("tcp_stand"), "left.local_final.tcp_stand")
        local_final_right_tcp = finite_published_tcp_pose(local_final["right"].get("tcp_stand"), "right.local_final.tcp_stand")
        local_left_orientation_delta_rad = assert_quaternion_preserved(
            final_left_tcp,
            local_final_left_tcp,
            "left TcpDeltaLocal",
            args.tcp_orientation_tolerance_rad,
        )
        local_right_orientation_delta_rad = assert_quaternion_preserved(
            final_right_tcp,
            local_final_right_tcp,
            "right zero TcpDeltaLocal",
            args.tcp_orientation_tolerance_rad,
        )

        previous_left_q = finite_joint_array(local_final["left"].get("q_sent_deg"), "left.safe.q_sent_deg")
        previous_right_q = finite_joint_array(local_final["right"].get("q_sent_deg"), "right.safe.q_sent_deg")
        ik_arm_motion = lifecycle_command("ArmMotion")
        send_udp_command(args.command_host, args.command_port, ik_arm_motion)
        wait_for_snapshot(
            capture,
            lambda snapshot: arm_motion_observed(snapshot, ik_arm_motion),
            args.startup_timeout_sec,
            "ArmMotion refreshed before unreachable TcpPoseTarget",
        )
        ik_failure_command = unreachable_tcp_pose_command(local_final_left_tcp)
        send_udp_command(args.command_host, args.command_port, ik_failure_command)
        ik_failure = wait_for_snapshot(
            capture,
            lambda snapshot: (
                int(snapshot.get("command_seq", -1)) >= int(ik_failure_command["seq"])
                and snapshot.get("safety_verdict") == "IkFailed"
            ),
            args.startup_timeout_sec,
            "unreachable TcpPoseTarget IkFailed verdict",
        )
        if ik_failure.get("fault_latched") is True:
            raise AcceptanceError("IK failure latched a fault during simulator acceptance")
        if not close_enough(finite_joint_array(ik_failure["left"].get("q_sent_deg"), "left.ik_failure.q_sent_deg"), previous_left_q):
            raise AcceptanceError("IK failure did not retain previous safe left q target")
        if not close_enough(finite_joint_array(ik_failure["right"].get("q_sent_deg"), "right.ik_failure.q_sent_deg"), previous_right_q):
            raise AcceptanceError("IK failure did not retain previous safe right q target")
        validate_cartesian_telemetry(ik_failure["left"], "left.ik_failure", expected_attempted=True, expected_success=False)
        wait_for_snapshot(capture, is_connected_valid, args.startup_timeout_sec, "state stream continues after IK failure")

        ik_latency_us = successful_ik_latency_summary(capture.snapshots)
        if ik_latency_us["count"] <= 0:
            raise AcceptanceError("no successful IK telemetry samples were captured")
        if ik_latency_us["p95"] > ik_latency_us["threshold_p95"]:
            raise AcceptanceError(
                f"successful IK p95 latency {ik_latency_us['p95']} us exceeds "
                f"{ik_latency_us['threshold_p95']} us"
            )
        if ik_latency_us["max"] > ik_latency_us["threshold_max"]:
            raise AcceptanceError(
                f"successful IK max latency {ik_latency_us['max']} us exceeds "
                f"{ik_latency_us['threshold_max']} us"
            )

        estop_summary: dict[str, Any] = {"skipped": True}
        if args.run_estop_reset == "1":
            estop = lifecycle_command("EmergencyStop")
            send_udp_command(args.command_host, args.command_port, estop)
            estop_snapshot = wait_for_snapshot(
                capture,
                lambda snapshot: emergency_stop_observed(snapshot, estop),
                args.startup_timeout_sec,
                "EmergencyStop latch",
            )
            reset = lifecycle_command("ResetFault")
            send_udp_command(args.command_host, args.command_port, reset)
            reset_snapshot = wait_for_snapshot(
                capture,
                lambda snapshot: reset_fault_observed(snapshot, reset),
                args.startup_timeout_sec,
                "ResetFault safe hold",
            )
            estop_summary = {
                "skipped": False,
                "estop_motion_state": estop_snapshot.get("motion_state"),
                "reset_motion_state": reset_snapshot.get("motion_state"),
            }
    finally:
        terminate_process(server_proc)
        terminate_process(right_proc)
        terminate_process(left_proc)
        capture.stop()

    servo_log = copy_servo_log(artifact_dir)
    if args.mode == "start-local" and servo_log is None:
        raise AcceptanceError("no servo_log.csv was produced under the acceptance artifact logs directory")

    summary = {
        "result": "pass",
        "artifacts_dir": str(artifact_dir),
        "mode": args.mode,
        "server_config": str(args.server_config.resolve()),
        "state_stream": str(artifact_dir / "state_stream.jsonl"),
        "servo_log": servo_log,
        "rb_servo_server_log": str(artifact_dir / "rb_servo_server.log"),
        "left_simulator_log": str(artifact_dir / "left_simulator.log"),
        "right_simulator_log": str(artifact_dir / "right_simulator.log"),
        "invalid_state_packets": capture.invalid_packets,
        "state_packets": len(capture.snapshots),
        "initial_left_tcp_x": initial_left_tcp["x"],
        "final_left_tcp_x": final_left_tcp["x"],
        "left_tcp_dx_m": left_dx,
        "right_tcp_dx_m": right_dx,
        "tcp_orientation_tolerance_rad": args.tcp_orientation_tolerance_rad,
        "stand_left_orientation_delta_rad": stand_left_orientation_delta_rad,
        "stand_right_orientation_delta_rad": stand_right_orientation_delta_rad,
        "local_left_orientation_delta_rad": local_left_orientation_delta_rad,
        "local_right_orientation_delta_rad": local_right_orientation_delta_rad,
        "normal_delta_verdict": delta_snapshot.get("safety_verdict"),
        "local_delta_verdict": local_delta_snapshot.get("safety_verdict"),
        "ik_failure_verdict": ik_failure.get("safety_verdict"),
        "ik_latency_us": ik_latency_us,
        "ik_failure_retained_previous_safe_target": True,
        "estop_reset": estop_summary,
        "caveat": "simulator-only evidence; does not prove rbpodo or real robot readiness",
    }
    (artifact_dir / "tcp_pose_acceptance_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return summary


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
            "artifacts_dir": str(artifact_dir),
            "state_stream": str(artifact_dir / "state_stream.jsonl"),
            "servo_log": str(artifact_dir / "servo_log.csv"),
            "rb_servo_server_log": str(artifact_dir / "rb_servo_server.log"),
            "left_simulator_log": str(artifact_dir / "left_simulator.log"),
            "right_simulator_log": str(artifact_dir / "right_simulator.log"),
            "caveat": "simulator-only evidence; does not prove rbpodo or real robot readiness",
        }
        (artifact_dir / "tcp_pose_acceptance_summary.json").write_text(
            json.dumps(failure, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(f"tcp_pose_acceptance: FAIL: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
PY
