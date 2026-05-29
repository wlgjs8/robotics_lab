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

import rainbow_rate_probe as probe
import rb_backend_ablation as ablation


def args(**overrides):
    base = {
        "ip": "127.0.0.1",
        "backend": "rbscript_tcp",
        "mode": "read_state",
        "rates": "50,75,100",
        "duration_sec": 1.0,
        "artifact_dir": Path(tempfile.mkdtemp()),
        "command_port": 5000,
        "data_port": 5001,
        "connect_timeout_sec": 0.1,
        "read_timeout_sec": 0.1,
        "command_timeout_sec": 0.1,
        "persistent_socket": False,
        "capture_raw_data_port": False,
        "rbscript_no_motion_command": None,
        "i_understand_this_connects_to_real_controller": False,
        "allow_simulation_servo_j": False,
        "disable_waiting_ack": False,
        "skip_plots": True,
    }
    base.update(overrides)
    return SimpleNamespace(**base)


class RawDataServer:
    def __init__(self, payload: bytes):
        self.payload = payload
        self.received = b""
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
                self.received += data
                if data == b"\n":
                    break
            conn.sendall(self.payload)
        self.sock.close()

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass
        self.thread.join(timeout=1.0)


class RainbowRateProbeTests(unittest.TestCase):
    def test_help_includes_persistent_socket(self):
        completed = subprocess.run(
            [sys.executable, "scripts/rainbow_rate_probe.py", "--help"],
            cwd=Path(__file__).resolve().parents[1],
            check=True,
            text=True,
            capture_output=True,
        )
        self.assertIn("--persistent-socket", completed.stdout)
        self.assertIn("--capture-raw-data-port", completed.stdout)

    def test_rate_parser(self):
        self.assertEqual(probe.parse_rates("50,75,100.5"), [50.0, 75.0, 100.5])
        with self.assertRaises(probe.RateProbeError):
            probe.parse_rates("50,0")
        with self.assertRaises(probe.RateProbeError):
            probe.parse_rates("")

    def test_help_safe_preflight_rejects_real_ip_without_confirmation(self):
        with mock.patch.dict(os.environ, {"RB_ALLOW_REAL_ROBOT": "1", "RB_ALLOW_RBSCRIPT_TCP": "1"}, clear=False):
            with self.assertRaisesRegex(probe.RateProbeError, "known real controller IP"):
                probe.preflight(args(ip="172.28.60.200"))

    def test_no_motion_refuses_move_command(self):
        with self.assertRaisesRegex(probe.RateProbeError, "motion-capable token"):
            probe.preflight(
                args(
                    mode="ack_no_motion",
                    rbscript_no_motion_command="move_servo_j(jnt[0,0,0,0,0,0],0.008,0.05,1,0.5)",
                )
            )

    def test_ack_no_motion_requires_explicit_command(self):
        with self.assertRaisesRegex(probe.RateProbeError, "requires explicit"):
            probe.preflight(args(mode="ack_no_motion"))

    def test_capture_raw_data_port_requires_rbscript_read_state(self):
        with self.assertRaisesRegex(probe.RateProbeError, "requires --backend rbscript_tcp --mode read_state"):
            probe.preflight(args(mode="ack_no_motion", capture_raw_data_port=True, rbscript_no_motion_command="noop()"))

    def test_capture_raw_data_port_writes_bytes_without_valid_state(self):
        artifact_dir = Path(tempfile.mkdtemp())
        server = RawDataServer(b"\x01\x02rainbow5001\n")
        try:
            cfg = args(
                artifact_dir=artifact_dir,
                data_port=server.port,
                capture_raw_data_port=True,
                read_timeout_sec=0.2,
            )
            metadata = probe.capture_raw_data_port(cfg, artifact_dir)
        finally:
            server.close()
        self.assertEqual(server.received, b"reqdata\n")
        self.assertEqual((artifact_dir / "raw_data_port_capture.bin").read_bytes(), b"\x01\x02rainbow5001\n")
        self.assertFalse(metadata["state_valid"])
        self.assertEqual(metadata["read_state_capability"], "unsupported")
        self.assertEqual(metadata["rbscript_tcp_data_port_mode"], "real_controller_unsupported")

    def test_rate_summary_aggregation(self):
        samples = [
            ablation.Sample(0, "command_ack_no_motion", "rbscript_tcp", True, 100.0, 0, 100_000),
            ablation.Sample(
                1,
                "command_ack_no_motion",
                "rbscript_tcp",
                False,
                200.0,
                10_000_000,
                10_200_000,
                "M568",
                "",
                "M568 previous motion not finished",
                response_error_names=["M568"],
            ),
        ]
        summary = ablation.summarize(
            SimpleNamespace(mode="command_ack_no_motion", backend="rbscript_tcp", arm="left", rate_hz=50.0, duration_sec=1.0),
            {"target_ip": "127.0.0.1", "env": {}, "safety_mode": "no_motion"},
            samples,
        )
        row = probe.rate_summary_row(summary)
        self.assertEqual(row["requested_rate_hz"], 50.0)
        self.assertEqual(row["send_count"], 2)
        self.assertFalse(row["persistent_socket"])
        self.assertEqual(row["ack_success_count"], 1)
        self.assertEqual(row["m568_count"], 1)
        self.assertEqual(row["p95_ack_us"], 195.0)
        self.assertEqual(row["read_state_capability"], "")

    def test_write_artifacts_summary(self):
        artifact_dir = Path(tempfile.mkdtemp())
        cfg = args(artifact_dir=artifact_dir)
        samples = [ablation.Sample(0, "read_state", "rbscript_tcp", True, 50.0, 0, 50_000)]
        row = {
            "requested_rate_hz": 50.0,
            "achieved_rate_hz": 50.0,
            "persistent_socket": False,
            "send_count": 1,
            "ack_success_count": 0,
            "ack_timeout_count": 0,
            "ack_error_count": 0,
            "p50_ack_us": 50.0,
            "p95_ack_us": 50.0,
            "p99_ack_us": 50.0,
            "max_ack_us": 50.0,
            "loop_interval_p50_ms": None,
            "loop_interval_p95_ms": None,
            "loop_interval_max_ms": None,
            "m561_count": 0,
            "m568_count": 0,
            "m569_count": 0,
            "m570_count": 0,
            "other_error_counts": {},
            "reconnect_count": 0,
            "stale_response_count": 0,
            "extra_response_count": 0,
            "unrecognized_response_count": 0,
            "response_lines_per_command_p50": 0.0,
            "response_lines_per_command_p95": 0.0,
            "response_lines_per_command_max": 0.0,
            "command_write_duration_us_p50": None,
            "command_write_duration_us_p95": None,
            "command_write_duration_us_max": None,
            "ack_read_duration_us_p50": None,
            "ack_read_duration_us_p95": None,
            "ack_read_duration_us_max": None,
            "data_success_count": 1,
            "data_timeout_count": 0,
            "success_rate": 1.0,
        }
        summary = probe.write_artifacts(
            cfg,
            {"rates": [50.0], "env": {}, "safety_mode": "no_motion"},
            [{"rate": 50.0, "samples": samples, "summary": {}}],
            [row],
        )
        self.assertEqual(summary["result"], "completed")
        self.assertTrue((artifact_dir / "summary.json").is_file())
        self.assertTrue((artifact_dir / "summary.csv").is_file())
        loaded = json.loads((artifact_dir / "summary.json").read_text(encoding="utf-8"))
        self.assertEqual(loaded["rate_results"][0]["requested_rate_hz"], 50.0)


if __name__ == "__main__":
    unittest.main()
