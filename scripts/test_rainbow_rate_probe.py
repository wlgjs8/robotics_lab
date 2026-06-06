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


def loopback_tcp_available():
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    except OSError:
        return False
    sock.close()
    return True


def args(**overrides):
    base = {
        "ip": "127.0.0.1",
        "backend": "rbscript_tcp",
        "mode": "read_state",
        "rates": probe.DEFAULT_RATES,
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
        "feedback_rate_hz": 500.0,
        "servo_t2_sec": 0.05,
        "servo_gain": 1.0,
        "servo_alpha": 0.5,
        "max_q_actual_drift_deg": 0.05,
        "set_pgmode_simulation": False,
        "verify_pgmode_simulation": False,
        "pgmode_timeout_sec": 1.0,
        "pgmode_command_port": 5000,
        "skip_plots": True,
    }
    base.update(overrides)
    return SimpleNamespace(**base)


def fake_rbpodo_module(q_actual=None, q_ref=None, raw_mode=1):
    q_actual = q_actual or [0.0, -1.0, 2.0, -3.0, 4.0, -5.0]
    q_ref = q_ref or list(q_actual)

    class FakeReturn:
        def is_success(self):
            return True

        def is_timeout(self):
            return False

        def __str__(self):
            return "ok"

    class FakeResponseCollector:
        def has_error(self):
            return False

        def __str__(self):
            return ""

    class FakeSData:
        def __init__(self):
            self.jnt_ang = list(q_actual)
            self.jnt_ref = list(q_ref)
            self.real_vs_simulation_mode = raw_mode
            self.init_state_info = 0
            self.init_error = 0
            self.op_stat_sos_flag = 0
            self.op_stat_ems_flag = 0
            self.op_stat_soft_estop_occur = 0
            self.op_stat_collision_occur = 0
            self.op_stat_self_collision = 0

    class FakeState:
        def __init__(self):
            self.sdata = FakeSData()

    class FakeCobotData:
        instances = []

        def __init__(self, ip):
            self.ip = ip
            self.calls = 0
            self.__class__.instances.append(self)

        def request_data(self, timeout_sec):
            self.calls += 1
            return FakeState()

    class FakeCobot:
        instances = []

        def __init__(self, ip):
            self.ip = ip
            self.waiting_ack_enabled = False
            self.servo_calls = []
            self.__class__.instances.append(self)

        def enable_waiting_ack(self, response_collector):
            self.waiting_ack_enabled = True
            return True

        def move_servo_j(self, response_collector, joint, t1, t2, gain, alpha, timeout, return_on_error):
            self.servo_calls.append({
                "joint": list(joint),
                "t1": t1,
                "t2": t2,
                "gain": gain,
                "alpha": alpha,
                "timeout": timeout,
                "return_on_error": return_on_error,
            })
            return FakeReturn()

    module = SimpleNamespace(
        Cobot=FakeCobot,
        CobotData=FakeCobotData,
        ResponseCollector=FakeResponseCollector,
    )
    return module, FakeCobot, FakeCobotData


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
        self.assertIn("--feedback-rate-hz", completed.stdout)
        self.assertIn("--set-pgmode-simulation", completed.stdout)

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

    def test_servo_j_preflight_is_rbpodo_only(self):
        with mock.patch.dict(os.environ, {
            "RB_ALLOW_REAL_ROBOT": "1",
            "RB_ALLOW_REAL_MOTION": "1",
            "RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION": "1",
        }, clear=False):
            with self.assertRaisesRegex(probe.RateProbeError, "only for --backend rbpodo"):
                probe.preflight(args(
                    backend="rbscript_tcp",
                    mode="servo_j_simulation_only",
                    allow_simulation_servo_j=True,
                    verify_pgmode_simulation=True,
                ))

    def test_servo_j_preflight_requires_explicit_allow(self):
        with mock.patch.dict(os.environ, {
            "RB_ALLOW_REAL_ROBOT": "1",
            "RB_ALLOW_REAL_MOTION": "1",
            "RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION": "1",
        }, clear=False):
            with self.assertRaisesRegex(probe.RateProbeError, "requires --allow-simulation-servo-j"):
                probe.preflight(args(
                    backend="rbpodo",
                    mode="servo_j_simulation_only",
                    verify_pgmode_simulation=True,
                ))

    def test_servo_j_preflight_requires_pgmode_path(self):
        with mock.patch.dict(os.environ, {
            "RB_ALLOW_REAL_ROBOT": "1",
            "RB_ALLOW_REAL_MOTION": "1",
            "RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION": "1",
        }, clear=False):
            with self.assertRaisesRegex(probe.RateProbeError, "requires --set-pgmode-simulation or --verify-pgmode-simulation"):
                probe.preflight(args(
                    backend="rbpodo",
                    mode="servo_j_simulation_only",
                    allow_simulation_servo_j=True,
                ))

    def test_servo_j_preflight_records_pgmode_and_schedule(self):
        pgmode = {"overall_result": "ok", "results": [{"ip": "127.0.0.1", "confirmed_simulation": True}]}
        with mock.patch.dict(os.environ, {
            "RB_ALLOW_REAL_ROBOT": "1",
            "RB_ALLOW_REAL_MOTION": "1",
            "RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION": "1",
        }, clear=False), mock.patch.object(probe, "ensure_controller_simulation_for_servo_j", return_value=pgmode):
            safety = probe.preflight(args(
                backend="rbpodo",
                mode="servo_j_simulation_only",
                allow_simulation_servo_j=True,
                verify_pgmode_simulation=True,
            ))
        self.assertEqual(safety["backend"], "rbpodo")
        self.assertFalse(safety["physical_motion_expected"])
        self.assertEqual(safety["feedback_rate_hz"], 500.0)
        self.assertEqual([item["send_interval_feedback_ticks"] for item in safety["send_rate_schedule"]], [5, 1])
        self.assertEqual([item["servo_t1_sec"] for item in safety["send_rate_schedule"]], [0.01, 0.002])
        self.assertEqual(safety["pgmode_simulation_preflight"], pgmode)

    def test_servo_j_preflight_rejects_rates_outside_100_500_scope(self):
        with mock.patch.dict(os.environ, {
            "RB_ALLOW_REAL_ROBOT": "1",
            "RB_ALLOW_REAL_MOTION": "1",
            "RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION": "1",
        }, clear=False):
            with self.assertRaisesRegex(probe.RateProbeError, "exactly --rates 100,500"):
                probe.preflight(args(
                    backend="rbpodo",
                    mode="servo_j_simulation_only",
                    rates="100",
                    allow_simulation_servo_j=True,
                    verify_pgmode_simulation=True,
                ))

    def test_servo_j_pgmode_failure_reports_details_and_writes_artifact(self):
        artifact_dir = Path(tempfile.mkdtemp())
        cfg = args(
            artifact_dir=artifact_dir,
            backend="rbpodo",
            mode="servo_j_simulation_only",
            allow_simulation_servo_j=True,
            set_pgmode_simulation=True,
            i_understand_this_connects_to_real_controller=True,
        )
        pgmode = {
            "overall_result": "error",
            "results": [{
                "ip": "172.28.60.200",
                "action": "set_simulation",
                "pgmode_command_sent": True,
                "command_ok": True,
                "response_classification": "no_response_after_send",
                "response_raw": "",
                "verification_available": True,
                "real_vs_simulation_mode": 0,
                "controller_mode": "real",
                "error_name": "controller_not_confirmed_in_pgmode_simulation",
                "error_message": "controller not confirmed in pgmode simulation",
            }],
        }
        with mock.patch("rainbow_pgmode.run_pgmode", return_value=pgmode):
            with self.assertRaisesRegex(probe.RateProbeError, "real_vs_simulation_mode=0"):
                probe.ensure_controller_simulation_for_servo_j(cfg)
        written = artifact_dir / "pgmode_preflight.json"
        self.assertTrue(written.is_file())
        self.assertEqual(json.loads(written.read_text(encoding="utf-8"))["overall_result"], "error")

    @unittest.skipUnless(loopback_tcp_available(), "loopback TCP sockets unavailable")
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

    def test_servo_j_probe_uses_500hz_feedback_100hz_send_schedule(self):
        fake_module, fake_cobot, _ = fake_rbpodo_module()
        cfg = args(
            backend="rbpodo",
            mode="servo_j_simulation_only",
            rates="100",
            duration_sec=0.026,
            allow_simulation_servo_j=True,
            verify_pgmode_simulation=True,
            read_timeout_sec=0.01,
            command_timeout_sec=0.01,
        )
        safety = {
            "rates": [100.0],
            "send_rate_schedule": probe.validate_servo_j_rate_schedule([100.0], 500.0),
        }
        with mock.patch.dict(sys.modules, {"rbpodo": fake_module}), mock.patch.object(probe, "joint_payload", lambda q: list(q)):
            samples, summary = probe.run_servo_j_simulation_rate(cfg, safety, 100.0)
        send_indexes = [sample["index"] for sample in samples if sample["send_requested"]]
        self.assertGreaterEqual(len(send_indexes), 2)
        self.assertTrue(all(index % 5 == 0 for index in send_indexes))
        self.assertTrue(fake_cobot.instances[-1].waiting_ack_enabled)
        self.assertEqual(len(fake_cobot.instances[-1].servo_calls), len(send_indexes))
        self.assertTrue(all(abs(call["t1"] - 0.01) < 1e-12 for call in fake_cobot.instances[-1].servo_calls))
        self.assertEqual(summary["result"], "completed")
        self.assertEqual(summary["feedback_rate_hz"], 500.0)

    def test_servo_j_probe_uses_2ms_t1_at_500hz(self):
        fake_module, fake_cobot, _ = fake_rbpodo_module()
        cfg = args(
            backend="rbpodo",
            mode="servo_j_simulation_only",
            rates="500",
            duration_sec=0.014,
            allow_simulation_servo_j=True,
            verify_pgmode_simulation=True,
            read_timeout_sec=0.01,
            command_timeout_sec=0.01,
        )
        safety = {
            "rates": [500.0],
            "send_rate_schedule": probe.validate_servo_j_rate_schedule([500.0], 500.0),
        }
        with mock.patch.dict(sys.modules, {"rbpodo": fake_module}), mock.patch.object(probe, "joint_payload", lambda q: list(q)):
            samples, summary = probe.run_servo_j_simulation_rate(cfg, safety, 500.0)
        self.assertGreaterEqual(len(samples), 2)
        self.assertTrue(all(sample["send_requested"] for sample in samples))
        self.assertEqual(len(fake_cobot.instances[-1].servo_calls), len(samples))
        self.assertTrue(all(abs(call["t1"] - 0.002) < 1e-12 for call in fake_cobot.instances[-1].servo_calls))
        self.assertEqual(summary["result"], "completed")

    def test_write_servo_j_artifacts_summary(self):
        artifact_dir = Path(tempfile.mkdtemp())
        cfg = args(
            artifact_dir=artifact_dir,
            backend="rbpodo",
            mode="servo_j_simulation_only",
            rates="100,500",
            allow_simulation_servo_j=True,
            verify_pgmode_simulation=True,
        )
        samples = [{
            "index": 0,
            "mode": "servo_j_simulation_only",
            "backend": "rbpodo",
            "requested_rate_hz": 100.0,
            "feedback_rate_hz": 500.0,
            "loop_start_ns": 0,
            "loop_end_ns": 1000,
            "loop_interval_ms": None,
            "read_duration_us": 20.0,
            "state_valid": True,
            "controller_mode": "simulation",
            "raw_controller_mode": 1,
            "q_actual_drift_max_deg": 0.0,
            "send_requested": True,
            "send_success": True,
            "send_start_ns": 100,
            "send_end_ns": 500,
            "send_duration_us": 0.4,
            "servo_t1_sec": 0.01,
            "servo_t2_sec": 0.05,
            "servo_gain": 1.0,
            "servo_alpha": 0.5,
            "error_name": "",
            "error_message": "",
            "response": "ok",
            "response_error_names": [],
            "q_actual_0": 0.0,
            "q_actual_1": 0.0,
            "q_actual_2": 0.0,
            "q_actual_3": 0.0,
            "q_actual_4": 0.0,
            "q_actual_5": 0.0,
            "q_ref_0": 0.0,
            "q_ref_1": 0.0,
            "q_ref_2": 0.0,
            "q_ref_3": 0.0,
            "q_ref_4": 0.0,
            "q_ref_5": 0.0,
        }]
        row = {
            "requested_rate_hz": 100.0,
            "feedback_rate_hz": 500.0,
            "achieved_feedback_rate_hz": None,
            "achieved_send_rate_hz": None,
            "feedback_sample_count": 1,
            "send_count": 1,
            "send_success_count": 1,
            "send_failure_count": 0,
            "send_success_rate": 1.0,
            "loop_interval_p50_ms": None,
            "loop_interval_p95_ms": None,
            "loop_interval_p99_ms": None,
            "loop_interval_max_ms": None,
            "read_duration_p50_us": 20.0,
            "read_duration_p95_us": 20.0,
            "read_duration_p99_us": 20.0,
            "read_duration_max_us": 20.0,
            "send_duration_p50_us": 0.4,
            "send_duration_p95_us": 0.4,
            "send_duration_p99_us": 0.4,
            "send_duration_max_us": 0.4,
            "q_actual_drift_max_deg": 0.0,
            "m561_count": 0,
            "m568_count": 0,
            "m569_count": 0,
            "m570_count": 0,
            "other_error_counts": {},
            "result": "completed",
            "result_reason": "",
        }
        summary = probe.write_servo_j_artifacts(
            cfg,
            {"rates": [100.0], "env": {}, "safety_mode": "rbpodo_controller_simulation_servo_j_noop"},
            [{"rate": 100.0, "samples": samples, "summary": {}}],
            [row],
        )
        self.assertEqual(summary["result"], "completed")
        self.assertTrue((artifact_dir / "samples_servo_j_100.csv").is_file())
        self.assertTrue((artifact_dir / "responses_servo_j_100.jsonl").is_file())
        loaded = json.loads((artifact_dir / "summary.json").read_text(encoding="utf-8"))
        self.assertEqual(loaded["backend"], "rbpodo")
        self.assertEqual(loaded["feedback_rate_hz"], 500.0)


if __name__ == "__main__":
    unittest.main()
