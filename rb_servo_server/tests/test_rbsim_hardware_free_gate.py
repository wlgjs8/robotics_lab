#!/usr/bin/env python3
"""Hardware-free rb_simulator + rb_servo_server integration gate."""

from __future__ import annotations

import csv
import json
import os
import re
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from typing import Any, Callable


SERVO_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = SERVO_ROOT.parent
SIM_ROOT = WORKSPACE_ROOT / "rb_simulator"

sys.path.insert(0, str(SIM_ROOT / "src"))
sys.path.insert(0, str(SERVO_ROOT / "tools"))

from rbsim import PROTOCOL_VERSION, RbsimService, load_simulator_config  # noqa: E402
import analyze_servo_log  # noqa: E402


class RbsimWorkerConfigContractTest(unittest.TestCase):
    def test_worker_profile_keeps_simulator_motion_gates_explicit(self) -> None:
        config_path = SERVO_ROOT / "config" / "dual_simulator_worker.yaml"
        text = config_path.read_text(encoding="utf-8")
        self.assertRegex(text, r"backend_type:\s*simulator")
        self.assertRegex(text, r"run_mode:\s*simulation")
        self.assertRegex(text, r"io_model:\s*worker")
        self.assertRegex(text, r"send_servo_commands:\s*true")
        self.assertRegex(text, r"force_control:\s*\n\s*provider:\s*null\s*\n\s*enable:\s*false")
        self.assertNotRegex(text, r"172\.28\.60\.20[01]")


def reserve_port(kind: socket.SocketKind) -> int:
    with socket.socket(socket.AF_INET, kind) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def tcp_jsonl_request(address: tuple[str, int], request: dict[str, Any]) -> dict[str, Any]:
    payload = json.dumps(request, separators=(",", ":")).encode("utf-8") + b"\n"
    with socket.create_connection(address, timeout=1.0) as sock:
        sock.sendall(payload)
        chunks: list[bytes] = []
        while True:
            chunk = sock.recv(1)
            if not chunk:
                break
            if chunk == b"\n":
                break
            chunks.append(chunk)
    return json.loads(b"".join(chunks).decode("utf-8"))


class StateCapture:
    def __init__(self, port: int) -> None:
        self.port = port
        self.snapshots: list[dict[str, Any]] = []
        self.invalid_packets = 0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._sock: socket.socket | None = None

    def start(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("127.0.0.1", self.port))
        sock.settimeout(0.1)
        self._sock = sock
        self._thread = threading.Thread(target=self._run, name="rbsim-state-capture", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._sock is not None:
            self._sock.close()

    def _run(self) -> None:
        assert self._sock is not None
        while not self._stop.is_set():
            try:
                payload, _addr = self._sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                decoded = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                self.invalid_packets += 1
                continue
            if isinstance(decoded, dict):
                self.snapshots.append(decoded)


class RbsimHardwareFreeGateTest(unittest.TestCase):
    SERVER_IO_MODEL = "direct"

    def setUp(self) -> None:
        if "RB_SERVO_SERVER_BIN" not in os.environ:
            self.skipTest("set RB_SERVO_SERVER_BIN to run the hardware-free integration gate")
        self.tmp = tempfile.TemporaryDirectory(prefix="rb-servo-rbsim-gate-")
        self.tmp_path = Path(self.tmp.name)
        try:
            self.left_control_port = reserve_port(socket.SOCK_STREAM)
            self.left_admin_port = reserve_port(socket.SOCK_STREAM)
            self.right_control_port = reserve_port(socket.SOCK_STREAM)
            self.right_admin_port = reserve_port(socket.SOCK_STREAM)
            self.command_port = reserve_port(socket.SOCK_DGRAM)
            self.state_port = reserve_port(socket.SOCK_DGRAM)
        except PermissionError as exc:
            self.tmp.cleanup()
            self.skipTest(f"loopback sockets are unavailable in this sandbox: {exc}")
        self.capture = StateCapture(self.state_port)
        self.capture.start()

        left_sim_config = load_simulator_config(SIM_ROOT / "config" / "left_rb3_730e.yaml")
        right_sim_config = load_simulator_config(SIM_ROOT / "config" / "right_rb3_730e.yaml")
        self.left_service = RbsimService.with_binds(
            left_sim_config,
            f"tcp://127.0.0.1:{self.left_control_port}",
            f"tcp://127.0.0.1:{self.left_admin_port}",
        )
        self.right_service = RbsimService.with_binds(
            right_sim_config,
            f"tcp://127.0.0.1:{self.right_control_port}",
            f"tcp://127.0.0.1:{self.right_admin_port}",
        )
        self.left_service.start()
        self.right_service.start()

        self.server_bin = Path(os.environ.get("RB_SERVO_SERVER_BIN", SERVO_ROOT / "build" / "rb_servo_server"))
        if not self.server_bin.exists():
            self.skipTest(f"rb_servo_server binary not found: {self.server_bin}")
        self.server_config = self.tmp_path / "dual_simulator_test.yaml"
        self.log_dir = self.tmp_path / "logs"
        self.server_config.write_text(self._server_config_text(), encoding="utf-8")
        self.server_log = (self.tmp_path / "rb_servo_server.log").open("w", encoding="utf-8")
        self.server = subprocess.Popen(
            [str(self.server_bin), "--config", str(self.server_config)],
            cwd=str(self.tmp_path),
            stdout=self.server_log,
            stderr=subprocess.STDOUT,
            text=True,
        )
        self.seq = 100
        self.wait_snapshot(self.connected_valid, "connected valid startup")

    def tearDown(self) -> None:
        server = getattr(self, "server", None)
        if server is not None and server.poll() is None:
            server.send_signal(signal.SIGTERM)
            try:
                server.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                server.kill()
                server.wait(timeout=2.0)
        server_log = getattr(self, "server_log", None)
        if server_log is not None:
            server_log.close()
        right_service = getattr(self, "right_service", None)
        if right_service is not None:
            right_service.stop()
        left_service = getattr(self, "left_service", None)
        if left_service is not None:
            left_service.stop()
        capture = getattr(self, "capture", None)
        if capture is not None:
            capture.stop()
        tmp = getattr(self, "tmp", None)
        if tmp is not None:
            tmp.cleanup()

    def _server_config_text(self) -> str:
        left_endpoint = f"tcp://127.0.0.1:{self.left_control_port}"
        right_endpoint = f"tcp://127.0.0.1:{self.right_control_port}"
        return f"""left_robot:
  backend_type: simulator
  run_mode: simulation
  name: left_rbsim_test
  simulator_control_endpoint: "{left_endpoint}"
  simulator_request_timeout_sec: 0.2
right_robot:
  backend_type: simulator
  run_mode: simulation
  name: right_rbsim_test
  simulator_control_endpoint: "{right_endpoint}"
  simulator_request_timeout_sec: 0.2
servo:
  rate_hz: 100
  command_timeout_sec: 0.5
  io_model: {self.SERVER_IO_MODEL}
  send_servo_commands: true
  enable_realtime_priority: false
safety:
  q_min_deg: [-170, -120, -170, -190, -120, -360]
  q_max_deg: [170, 120, 170, 190, 120, 360]
  dq_max_deg_s: [600, 600, 600, 600, 600, 600]
  ddq_max_deg_s2: [6000, 6000, 6000, 6000, 6000, 6000]
  max_tracking_error_deg: 10.0
  tracking_error_policy: snap_to_actual
  latch_fault_on_robot_state_error: true
  stop_both_arms_on_single_arm_error: true
network:
  command_bind: "udp://127.0.0.1:{self.command_port}"
  state_pub_bind: "udp://127.0.0.1:{self.state_port}"
  command_source_allowlist: ["127.0.0.1/32"]
logging:
  enable: true
  directory: "{self.log_dir}"
  flush_period_ms: 20
  queue_capacity: 4096
force_control:
  provider: null
  enable: false
"""

    def admin(self, op: str, arm: str | None = None, **params: Any) -> dict[str, Any]:
        self.seq += 1
        request: dict[str, Any] = {
            "schema_version": PROTOCOL_VERSION,
            "request_id": f"admin-{self.seq}",
            "op": op,
            "params": params,
        }
        if arm is not None:
            request["arm"] = arm
        if arm is None and op == "admin.reset_hooks":
            left_response = tcp_jsonl_request(("127.0.0.1", self.left_admin_port), {**request, "arm": "left"})
            right_response = tcp_jsonl_request(("127.0.0.1", self.right_admin_port), {**request, "arm": "right"})
            self.assertTrue(left_response.get("ok"), left_response)
            self.assertTrue(right_response.get("ok"), right_response)
            return {"ok": True, "left": left_response, "right": right_response}

        admin_port = self.left_admin_port if arm != "right" else self.right_admin_port
        response = tcp_jsonl_request(("127.0.0.1", admin_port), request)
        self.assertTrue(response.get("ok"), response)
        return response

    def control_request(self, port: int, op: str, arm: str, **params: Any) -> dict[str, Any]:
        self.seq += 1
        request: dict[str, Any] = {
            "schema_version": PROTOCOL_VERSION,
            "request_id": f"control-{self.seq}",
            "op": op,
            "arm": arm,
            "params": params,
        }
        return tcp_jsonl_request(("127.0.0.1", port), request)

    def send_command(self, mode: str, **extra: Any) -> int:
        self.seq += 1
        message: dict[str, Any] = {
            "seq": self.seq,
            "mode": mode,
            "timeout_sec": 0.5,
            "coupled_timeout": True,
        }
        message.update(extra)
        payload = json.dumps(message, separators=(",", ":")).encode("utf-8")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.sendto(payload, ("127.0.0.1", self.command_port))
        return self.seq

    def wait_snapshot(
        self,
        predicate: Callable[[dict[str, Any]], bool],
        label: str,
        timeout: float = 3.0,
    ) -> dict[str, Any]:
        deadline = time.monotonic() + timeout
        seen = 0
        while time.monotonic() < deadline:
            for snapshot in self.capture.snapshots[seen:]:
                if predicate(snapshot):
                    return snapshot
            seen = len(self.capture.snapshots)
            self.assertIsNone(self.server.poll(), self.server_log_path_text())
            time.sleep(0.02)
        self.fail(f"timed out waiting for {label}; latest={self.capture.snapshots[-1:]}")

    def server_log_path_text(self) -> str:
        self.server_log.flush()
        return (self.tmp_path / "rb_servo_server.log").read_text(encoding="utf-8", errors="replace")

    @staticmethod
    def connected_valid(snapshot: dict[str, Any]) -> bool:
        return (
            snapshot.get("schema_version") == 1
            and snapshot.get("fault_latched") is False
            and snapshot.get("motion_state") == "ConnectedHold"
            and snapshot.get("left", {}).get("connection_state") == "Connected"
            and snapshot.get("right", {}).get("connection_state") == "Connected"
            and snapshot.get("left", {}).get("has_valid_joint_state") is True
            and snapshot.get("right", {}).get("has_valid_joint_state") is True
        )

    def latest_log(self) -> Path:
        candidates = sorted(self.log_dir.glob("*.csv"), key=lambda path: path.stat().st_mtime)
        self.assertTrue(candidates, f"no CSV logs under {self.log_dir}")
        return candidates[-1]

    def assert_latency_metrics_present(self, snapshot: dict[str, Any]) -> None:
        self.assertIn("command_seq", snapshot)
        self.assertIn("send_skew_us", snapshot)
        self.assertIn("dispatch_skew_us", snapshot)
        self.assertIn("send_deadline_hit", snapshot)
        for arm in ("left", "right"):
            arm_state = snapshot[arm]
            for key in (
                "state_age_us",
                "send_result_age_us",
                "send_deadline_hit",
                "worker_loop_read_duration_us",
            ):
                self.assertIn(key, arm_state)
            self.assertGreaterEqual(float(arm_state["state_age_us"]), 0.0)
            self.assertGreaterEqual(float(arm_state["send_result_age_us"]), 0.0)
            self.assertIsInstance(arm_state["send_deadline_hit"], bool)
            self.assertGreaterEqual(float(arm_state["worker_loop_read_duration_us"]), 0.0)

    def test_wrong_arm_requests_fail_closed(self) -> None:
        right_to_left = self.control_request(self.left_control_port, "read_state", "right")
        self.assertFalse(right_to_left.get("ok"), right_to_left)
        self.assertEqual(right_to_left.get("error", {}).get("name"), "wrong_arm")
        self.assertEqual(right_to_left.get("error", {}).get("code"), 1005)

        left_to_right = self.control_request(self.right_control_port, "read_state", "left")
        self.assertFalse(left_to_right.get("ok"), left_to_right)
        self.assertEqual(left_to_right.get("error", {}).get("name"), "wrong_arm")
        self.assertEqual(left_to_right.get("error", {}).get("code"), 1005)

    def test_nominal_motion_and_log_budget_gate(self) -> None:
        first = self.wait_snapshot(self.connected_valid, "startup")
        left_initial = list(first["left"]["q_actual_deg"])
        right_initial = list(first["right"]["q_actual_deg"])
        left_target = list(left_initial)
        right_target = list(right_initial)
        left_target[0] += 1.0
        right_target[0] -= 1.0

        self.send_command("ArmMotion")
        target_seq = self.send_command(
            "JointTarget",
            left={"q_target_deg": left_target},
            right={"q_target_deg": right_target},
        )
        running = self.wait_snapshot(
            lambda snap: snap.get("command_seq", 0) >= target_seq
            and snap.get("motion_state") == "Running"
            and abs(float(snap["left"]["q_sent_deg"][0]) - left_target[0]) < 0.05
            and abs(float(snap["right"]["q_sent_deg"][0]) - right_target[0]) < 0.05,
            "running joint target",
        )
        self.assertFalse(running["fault_latched"])
        self.assert_latency_metrics_present(running)
        time.sleep(2.1)

        metrics = analyze_servo_log.analyze_csv(self.latest_log())
        failures = analyze_servo_log.check_budget(metrics, analyze_servo_log.BUDGETS["rbsim-local100"])
        self.assertEqual(failures, [])
        self.assertGreaterEqual(metrics["duration_s"], 2.0)
        with self.latest_log().open(newline="", encoding="utf-8") as handle:
            fieldnames = csv.DictReader(handle).fieldnames or []
        for column in (
            "left_state_age_us",
            "right_state_age_us",
            "left_send_result_age_us",
            "right_send_result_age_us",
            "left_send_deadline_hit",
            "right_send_deadline_hit",
            "dispatch_skew_us",
            "left_worker_loop_read_duration_us",
            "right_worker_loop_read_duration_us",
        ):
            self.assertIn(column, fieldnames)

    def test_faults_are_visible_and_reset_semantics_are_truthful(self) -> None:
        self.send_command("ArmMotion")
        send_fail_seq = self.send_command(
            "JointTarget",
            q_target_deg=[1, -29, 80, 0, 60, 0],
            timeout_sec=2.0,
        )
        self.wait_snapshot(
            lambda snap: snap.get("command_seq", 0) >= send_fail_seq
            and snap.get("motion_state") == "Running",
            "send failure target armed",
        )
        self.admin("admin.inject", "left", hook="send_failure", enabled=True)
        send_fault = self.wait_snapshot(
            # StatePublisher samples the latest loop snapshot at a lower rate than
            # the servo loop. The command tick that first latched the send failure
            # can be missed, so the durable contract here is the truthful latched
            # fault state and per-arm send result, not the transient command seq.
            lambda snap: snap.get("fault_latched") is True
            and snap.get("latched_fault_reason") == "SendFailure"
            and snap.get("left", {}).get("send_ok") is False,
            "send failure latch",
        )
        self.assertEqual(send_fault["motion_state"], "FaultLatched")
        self.admin("admin.reset_hooks")
        self.send_command("ResetFault")
        send_fault_tick = int(send_fault.get("tick", 0))
        self.wait_snapshot(
            lambda snap: int(snap.get("tick", 0)) > send_fault_tick and self.connected_valid(snap),
            "reset after injected send failure",
        )

        self.admin("admin.set_stale_state", "left", enabled=True)
        stale = self.wait_snapshot(
            lambda snap: snap.get("fault_latched") is True
            and snap.get("latched_fault_reason") == "RobotStateError"
            and snap.get("left", {}).get("has_valid_joint_state") is False,
            "stale state fault visibility",
        )
        self.assertEqual(stale["left"]["connection_state"], "Connected")
        self.admin("admin.set_stale_state", "left", enabled=False)
        self.send_command("ResetFault")
        stale_tick = int(stale.get("tick", 0))
        self.wait_snapshot(
            lambda snap: int(snap.get("tick", 0)) > stale_tick and self.connected_valid(snap),
            "reset after stale state clears",
        )

        self.admin("admin.set_fault", "left", error_code=2222, recoverable=True)
        recoverable = self.wait_snapshot(
            lambda snap: snap.get("fault_latched") is True
            and snap.get("latched_fault_reason") == "RobotStateError"
            and snap.get("left", {}).get("error_code") == 2222,
            "recoverable simulator fault",
        )
        self.assertEqual(recoverable["motion_state"], "FaultLatched")
        self.assertEqual(recoverable["send_policy"], "fault_latched")
        self.assertTrue(recoverable["send_suppressed"])
        self.assertEqual(recoverable["left"]["last_read"]["backend_error_kind"], "RobotFault")
        self.assertEqual(recoverable["left"]["last_send"]["backend_error_kind"], "SuppressedByPolicy")
        self.assertFalse(recoverable["left"]["last_send"]["accepted"])
        self.send_command("ResetFault")
        recoverable_tick = int(recoverable.get("tick", 0))
        self.wait_snapshot(
            lambda snap: int(snap.get("tick", 0)) > recoverable_tick and self.connected_valid(snap),
            "recoverable simulator fault reset",
        )

        self.admin("admin.set_fault", "right", error_code=3333, recoverable=False)
        unrecoverable = self.wait_snapshot(
            lambda snap: snap.get("fault_latched") is True
            and snap.get("latched_fault_reason") == "RobotStateError"
            and snap.get("right", {}).get("error_code") == 3333,
            "unrecoverable simulator fault",
        )
        self.assertEqual(unrecoverable["send_policy"], "fault_latched")
        self.assertTrue(unrecoverable["send_suppressed"])
        self.assertEqual(unrecoverable["right"]["last_read"]["backend_error_kind"], "RobotFault")
        self.assertEqual(unrecoverable["right"]["last_send"]["backend_error_kind"], "SuppressedByPolicy")
        self.assertFalse(unrecoverable["right"]["last_send"]["accepted"])
        self.send_command("ResetFault")
        unrecoverable_tick = int(unrecoverable.get("tick", 0))
        still_latched = self.wait_snapshot(
            lambda snap: int(snap.get("tick", 0)) > unrecoverable_tick
            and snap.get("fault_latched") is True
            and snap.get("right", {}).get("error_code") == 3333,
            "unrecoverable fault remains latched",
        )
        self.assertEqual(still_latched["motion_state"], "FaultLatched")

        with self.latest_log().open(newline="", encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
        self.assertTrue(any(row["left_send_ok"] in {"0", "false"} for row in rows))
        self.assertTrue(any(row["fault_latched"] in {"1", "true"} for row in rows))


class RbsimWorkerHardwareFreeGateTest(RbsimHardwareFreeGateTest):
    SERVER_IO_MODEL = "worker"

    def test_faults_are_visible_and_reset_semantics_are_truthful(self) -> None:
        self.send_command("ArmMotion")
        send_fail_seq = self.send_command(
            "JointTarget",
            q_target_deg=[1, -29, 80, 0, 60, 0],
            timeout_sec=2.0,
        )
        self.wait_snapshot(
            lambda snap: snap.get("command_seq", 0) >= send_fail_seq
            and snap.get("motion_state") == "Running",
            "worker send failure target armed",
        )

        self.admin("admin.inject", "left", hook="send_failure", enabled=True)
        send_fault = self.wait_snapshot(
            lambda snap: snap.get("fault_latched") is True
            and snap.get("latched_fault_reason") == "SendFailure"
            and snap.get("left", {}).get("send_ok") is False,
            "worker send failure latch",
        )
        self.assertEqual(send_fault["motion_state"], "FaultLatched")
        self.assertEqual(send_fault["left"]["last_send"]["backend_error_kind"], "TransportWriteFailed")
        self.assertEqual(send_fault["left"]["last_send"]["error_name"], "send_failure_injected")
        self.assert_latency_metrics_present(send_fault)

        self.admin("admin.reset_hooks")
        suppressed_seq = self.send_command(
            "JointTarget",
            q_target_deg=[2, -29, 80, 0, 60, 0],
            timeout_sec=2.0,
        )
        suppressed = self.wait_snapshot(
            lambda snap: snap.get("command_seq", 0) >= suppressed_seq
            and snap.get("send_policy") == "fault_latched"
            and snap.get("send_suppressed") is True,
            "worker fault-latched regular servo_j suppression",
        )
        self.assertEqual(suppressed["left"]["last_send"]["backend_error_kind"], "SuppressedByPolicy")
        self.assertFalse(suppressed["left"]["last_send"]["accepted"])
        self.assertEqual(suppressed["right"]["last_send"]["backend_error_kind"], "SuppressedByPolicy")
        self.assertFalse(suppressed["right"]["last_send"]["accepted"])

    def test_worker_simulator_robot_fault_is_distinguishable(self) -> None:
        self.admin("admin.set_fault", "right", error_code=3333, recoverable=False)
        fault = self.wait_snapshot(
            lambda snap: snap.get("fault_latched") is True
            and snap.get("latched_fault_reason") == "RobotStateError"
            and snap.get("right", {}).get("error_code") == 3333,
            "worker simulator robot fault",
        )
        self.assertEqual(fault["motion_state"], "FaultLatched")
        self.assertEqual(fault["right"]["last_read"]["backend_error_kind"], "RobotFault")
        self.assertEqual(fault["right"]["last_read"]["error_name"], "fault_latched")
        self.assertEqual(fault["right"]["last_send"]["backend_error_kind"], "SuppressedByPolicy")
        self.assertFalse(fault["right"]["last_send"]["accepted"])
        self.assert_latency_metrics_present(fault)


if __name__ == "__main__":
    unittest.main()
