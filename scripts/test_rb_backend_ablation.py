import json
import os
import socket
import subprocess
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import rb_backend_ablation as ablation
import compare_backend_ablation as compare


class FakeLineServer:
    def __init__(self, response: str, hold: float = 0.0):
        self.response = response
        self.hold = hold
        self.received = ""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.listen(1)
        self.port = self.sock.getsockname()[1]
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        try:
            conn, _ = self.sock.accept()
        except OSError:
            return
        with conn:
            while True:
                data = conn.recv(1)
                if not data:
                    break
                self.received += data.decode("utf-8", errors="replace")
                if data == b"\n":
                    break
            if self.hold:
                threading.Event().wait(self.hold)
            if self.response is not None:
                conn.sendall((self.response + "\n").encode("utf-8"))
        self.sock.close()

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass
        self.thread.join(timeout=1.0)


class MultiCommandLineServer:
    def __init__(self, responses, extra_lines=None, close_after_responses=None, drop_without_response=None):
        self.responses = list(responses)
        self.extra_lines = {int(key): list(value) for key, value in (extra_lines or {}).items()}
        self.close_after_responses = set(close_after_responses or [])
        self.drop_without_response = set(drop_without_response or [])
        self.received_commands: list[str] = []
        self.connection_count = 0
        self._stop = threading.Event()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.listen(5)
        self.sock.settimeout(0.05)
        self.port = self.sock.getsockname()[1]
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _recv_line(self, conn):
        chunks = []
        while not self._stop.is_set():
            data = conn.recv(1)
            if not data:
                return None
            chunks.append(data)
            if data == b"\n":
                break
        return b"".join(chunks).decode("utf-8", errors="replace")

    def _send_line(self, conn, line):
        conn.sendall((line + "\n").encode("utf-8"))

    def _run(self):
        while not self._stop.is_set():
            try:
                conn, _ = self.sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            self.connection_count += 1
            with conn:
                while not self._stop.is_set():
                    try:
                        line = self._recv_line(conn)
                    except OSError:
                        break
                    if line is None:
                        break
                    command_index = len(self.received_commands)
                    self.received_commands.append(line)
                    if command_index in self.drop_without_response:
                        break
                    response = self.responses[command_index] if command_index < len(self.responses) else self.responses[-1]
                    try:
                        self._send_line(conn, response)
                        for extra in self.extra_lines.get(command_index, []):
                            self._send_line(conn, extra)
                    except OSError:
                        break
                    if command_index in self.close_after_responses:
                        break
        try:
            self.sock.close()
        except OSError:
            pass

    def close(self):
        self._stop.set()
        try:
            self.sock.close()
        except OSError:
            pass
        self.thread.join(timeout=1.0)


def args(**overrides):
    base = {
        "left_ip": "127.0.0.1",
        "right_ip": "127.0.0.1",
        "arm": "left",
        "backend": "rbscript_tcp",
        "mode": "read_state",
        "duration_sec": 0.01,
        "rate_hz": 10.0,
        "artifact_dir": Path(tempfile.mkdtemp()),
        "command_port": 5000,
        "data_port": 5001,
        "connect_timeout_sec": 0.1,
        "read_timeout_sec": 0.1,
        "command_timeout_sec": 0.1,
        "persistent_socket": False,
        "rbscript_no_motion_command": None,
        "allow_motion": False,
        "max_delta_deg": None,
        "i_understand_this_connects_to_real_controller": False,
        "skip_plots": True,
    }
    base.update(overrides)
    return SimpleNamespace(**base)


class BackendAblationTests(unittest.TestCase):
    def test_help_includes_persistent_socket(self):
        completed = subprocess.run(
            [sys.executable, "scripts/rb_backend_ablation.py", "--help"],
            cwd=Path(__file__).resolve().parents[1],
            check=True,
            text=True,
            capture_output=True,
        )
        self.assertIn("--persistent-socket", completed.stdout)

    def test_preflight_rejects_motion_without_allow(self):
        with self.assertRaisesRegex(ablation.AblationError, "--allow-motion"):
            ablation.preflight(args(mode="servo_j_dry_run"))

    def test_preflight_rejects_real_ip_without_confirmation(self):
        with mock.patch.dict(os.environ, {"RB_ALLOW_REAL_ROBOT": "1", "RB_ALLOW_RBSCRIPT_TCP": "1"}, clear=False):
            with self.assertRaisesRegex(ablation.AblationError, "known real controller IP"):
                ablation.preflight(args(left_ip="172.28.60.200"))

    def test_preflight_requires_rbscript_env_for_real_read(self):
        with mock.patch.dict(os.environ, {}, clear=True):
            with self.assertRaisesRegex(ablation.AblationError, "RB_ALLOW_REAL_ROBOT"):
                ablation.preflight(
                    args(
                        left_ip="172.28.60.200",
                        i_understand_this_connects_to_real_controller=True,
                    )
                )

    def test_preflight_rejects_motion_like_no_motion_command(self):
        with self.assertRaisesRegex(ablation.AblationError, "motion-capable token"):
            ablation.preflight(
                args(
                    mode="command_ack_no_motion",
                    rbscript_no_motion_command="move_servo_j(jnt[0,0,0,0,0,0],0.008,0.05,1,0.5)",
                )
            )

    def test_rbscript_read_state_fake_server(self):
        fixture = json.dumps(
            {
                "schema": "rbscript_tcp_state_v1",
                "q_actual_deg": [1, 2, 3, 4, 5, 6],
                "robot_time_sec": 1.0,
            }
        )
        server = FakeLineServer(fixture)
        try:
            cfg = args(data_port=server.port)
            sample = ablation.RbscriptProbe(cfg, "127.0.0.1").read_state(0)
        finally:
            server.close()
        self.assertTrue(sample.success)
        self.assertTrue(sample.state_valid)
        self.assertTrue(sample.q_actual_finite)
        self.assertEqual(sample.rbscript_tcp_data_port_mode, "json_fixture")
        self.assertEqual(sample.read_state_capability, "experimental")
        self.assertFalse(sample.comparable)
        self.assertEqual(server.received, "reqdata\n")

    def test_rbscript_read_state_unknown_payload_is_unsupported(self):
        server = FakeLineServer("\x01\x02rainbow")
        try:
            cfg = args(data_port=server.port)
            sample = ablation.RbscriptProbe(cfg, "127.0.0.1").read_state(0)
        finally:
            server.close()
        self.assertFalse(sample.success)
        self.assertFalse(sample.state_valid)
        self.assertEqual(sample.error_name, "rbscript_tcp_real_data_port_unsupported")
        self.assertEqual(sample.rbscript_tcp_data_port_mode, "real_controller_unsupported")
        self.assertEqual(sample.read_state_capability, "unsupported")
        self.assertFalse(sample.comparable)
        summary = ablation.summarize(
            cfg,
            {"target_ip": "127.0.0.1", "env": {}, "safety_mode": "no_motion"},
            [sample],
        )
        self.assertEqual(summary["read_state_capability"], "unsupported")
        self.assertEqual(summary["rbscript_tcp_data_port_mode"], "real_controller_unsupported")
        self.assertFalse(summary["comparable"])

    def test_persistent_command_socket_reuses_one_connection(self):
        server = MultiCommandLineServer(["ok", "ok", "ok"])
        try:
            cfg = args(
                mode="command_ack_no_motion",
                command_port=server.port,
                rbscript_no_motion_command="noop()",
                persistent_socket=True,
            )
            probe = ablation.RbscriptProbe(cfg, "127.0.0.1")
            try:
                samples = [probe.command_ack_no_motion(i) for i in range(3)]
            finally:
                probe.close()
        finally:
            server.close()
        self.assertTrue(all(sample.success for sample in samples))
        self.assertEqual(server.connection_count, 1)
        self.assertEqual(server.received_commands, ["noop()\n", "noop()\n", "noop()\n"])
        self.assertTrue(all(sample.persistent_socket for sample in samples))

    def test_extra_lines_are_counted_and_classified(self):
        server = MultiCommandLineServer(["ok"], extra_lines={0: ["extra detail", "another detail"]})
        try:
            cfg = args(
                mode="command_ack_no_motion",
                command_port=server.port,
                rbscript_no_motion_command="noop()",
                persistent_socket=True,
            )
            probe = ablation.RbscriptProbe(cfg, "127.0.0.1")
            try:
                sample = probe.command_ack_no_motion(0)
            finally:
                probe.close()
        finally:
            server.close()
        self.assertTrue(sample.success)
        self.assertEqual(sample.extra_response_count, 2)
        self.assertEqual(sample.response_line_count, 3)
        self.assertEqual(sample.unrecognized_response_count, 2)

    def test_responses_jsonl_preserves_stale_and_unrecognized(self):
        artifact_dir = Path(tempfile.mkdtemp())
        sample = ablation.Sample(
            0,
            "command_ack_no_motion",
            "rbscript_tcp",
            False,
            100.0,
            0,
            100_000,
            "unrecognized_response",
            "",
            "garbled",
            response_error_names=["unrecognized_response"],
        )
        ablation.annotate_response_metrics(
            sample,
            response_lines=["garbled"],
            stale_response_lines=["late previous ack"],
            persistent_socket=True,
            unrecognized_response_count=1,
        )
        cfg = args(mode="command_ack_no_motion", artifact_dir=artifact_dir, persistent_socket=True)
        ablation.write_artifacts(
            cfg,
            {"target_ip": "127.0.0.1", "env": {}, "safety_mode": "no_motion"},
            [sample],
            {"result": "completed"},
        )
        records = [
            json.loads(line)
            for line in (artifact_dir / "responses.jsonl").read_text(encoding="utf-8").splitlines()
        ]
        self.assertEqual(records[0]["response_lines"], ["garbled"])
        self.assertEqual(records[0]["stale_response_lines"], ["late previous ack"])
        self.assertIn("unrecognized_response", records[0]["response_error_names"])

    def test_persistent_reconnect_count_after_transport_drop(self):
        server = MultiCommandLineServer(
            ["ok", "ok"],
            close_after_responses={0},
        )
        try:
            cfg = args(
                mode="command_ack_no_motion",
                command_port=server.port,
                rbscript_no_motion_command="noop()",
                persistent_socket=True,
            )
            probe = ablation.RbscriptProbe(cfg, "127.0.0.1")
            try:
                first = probe.command_ack_no_motion(0)
                second = probe.command_ack_no_motion(1)
            finally:
                probe.close()
        finally:
            server.close()
        self.assertTrue(first.success)
        self.assertTrue(second.success)
        self.assertEqual(second.reconnect_count, 1)
        summary = ablation.summarize(
            cfg,
            {"target_ip": "127.0.0.1", "env": {}, "safety_mode": "no_motion"},
            [first, second],
        )
        self.assertEqual(summary["reconnect_count"], 1)

    def test_summary_metrics(self):
        samples = [
            ablation.Sample(0, "command_ack_no_motion", "rbscript_tcp", True, 100.0, 0, 100_000),
            ablation.Sample(1, "command_ack_no_motion", "rbscript_tcp", False, 300.0, 1_000_000, 1_300_000, "TransportTimeout"),
        ]
        summary = ablation.summarize(
            args(mode="command_ack_no_motion"),
            {"target_ip": "127.0.0.1", "env": {}, "safety_mode": "no_motion"},
            samples,
        )
        self.assertEqual(summary["sample_count"], 2)
        self.assertEqual(summary["command_success_count"], 1)
        self.assertEqual(summary["command_timeout_count"], 1)
        self.assertAlmostEqual(summary["success_rate"], 0.5)
        self.assertEqual(summary["command_ack_latency_us"]["max"], 300.0)
        self.assertFalse(summary["persistent_socket"])

    def test_compare_row_handles_missing_fields(self):
        row = compare.comparison_row({"backend": "rbpodo", "mode": "read_state", "sample_count": 2, "success_count": 1})
        self.assertEqual(row["backend"], "rbpodo")
        self.assertEqual(row["mode"], "read_state")
        self.assertEqual(row["error_count"], 1)
        self.assertIsNone(row["p95_ack_us"])
        self.assertEqual(row["read_state_capability"], "supported")
        self.assertTrue(row["comparable"])

    def test_compare_row_marks_unsupported_rbscript_read_state_not_comparable(self):
        row = compare.comparison_row(
            {
                "backend": "rbscript_tcp",
                "mode": "read_state",
                "sample_count": 2,
                "success_count": 0,
                "read_duration_us": {"p50": 100.0, "p95": 200.0, "p99": 250.0},
                "read_state_capability": "unsupported",
                "rbscript_tcp_data_port_mode": "real_controller_unsupported",
            }
        )
        self.assertEqual(row["read_state_capability"], "unsupported")
        self.assertFalse(row["comparable"])
        self.assertIsNone(row["success_rate"])
        self.assertIsNone(row["p95_ack_us"])
        self.assertIn("parser is unsupported", row["not_comparable_reason"])

    def test_compare_rows_expands_rate_probe_summary(self):
        rows = compare.comparison_rows(
            {
                "backend": "rbscript_tcp",
                "mode": "ack_no_motion",
                "rate_results": [
                    {
                        "requested_rate_hz": 50.0,
                        "send_count": 10,
                        "success_rate": 0.8,
                        "p50_ack_us": 100.0,
                        "p95_ack_us": 200.0,
                        "p99_ack_us": 250.0,
                        "ack_timeout_count": 1,
                        "achieved_rate_hz": 49.5,
                        "persistent_socket": True,
                        "reconnect_count": 0,
                    }
                ],
            }
        )
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["requested_rate_hz"], 50.0)
        self.assertEqual(rows[0]["timeout_count"], 1)
        self.assertEqual(rows[0]["error_count"], 1)
        self.assertTrue(rows[0]["persistent_socket"])
        self.assertEqual(rows[0]["reconnect_count"], 0)


if __name__ == "__main__":
    unittest.main()
