import json
import os
import socket
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
        "rbscript_no_motion_command": None,
        "allow_motion": False,
        "max_delta_deg": None,
        "i_understand_this_connects_to_real_controller": False,
        "skip_plots": True,
    }
    base.update(overrides)
    return SimpleNamespace(**base)


class BackendAblationTests(unittest.TestCase):
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
        self.assertEqual(server.received, "reqdata\n")

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

    def test_compare_row_handles_missing_fields(self):
        row = compare.comparison_row({"backend": "rbpodo", "mode": "read_state", "sample_count": 2, "success_count": 1})
        self.assertEqual(row["backend"], "rbpodo")
        self.assertEqual(row["mode"], "read_state")
        self.assertEqual(row["error_count"], 1)
        self.assertIsNone(row["p95_ack_us"])

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
                    }
                ],
            }
        )
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["requested_rate_hz"], 50.0)
        self.assertEqual(rows[0]["timeout_count"], 1)
        self.assertEqual(rows[0]["error_count"], 1)


if __name__ == "__main__":
    unittest.main()
