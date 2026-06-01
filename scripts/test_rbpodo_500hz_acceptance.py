import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import rbpodo_500hz_acceptance as accept


CONFIG_TEMPLATE = """schema: robotics_lab.rb_servo_server.v1
left_robot:
  backend_type: rbpodo
  run_mode: real
  ip: "172.28.60.200"
  operation_mode: {operation_mode}
  command_timeout_sec: 0.005
  servo_t1_sec: 0.002
  servo_t2_sec: 0.03
  servo_gain: 1.0
  servo_alpha: 0.5
  disable_waiting_ack: false
right_robot:
  backend_type: rbpodo
  run_mode: real
  ip: "172.28.60.201"
  operation_mode: {operation_mode}
  command_timeout_sec: 0.005
  servo_t1_sec: 0.002
  servo_t2_sec: 0.03
  servo_gain: 1.0
  servo_alpha: 0.5
  disable_waiting_ack: false
servo:
  rate_hz: 500
  send_servo_commands: true
  allow_controller_simulation_motion: true
  allow_controller_simulation_diagnostics_suspect: false
  servo_t1_rate_match_tolerance_ratio: 0.2
network:
  command_bind: "udp://127.0.0.1:50251"
  state_pub_endpoints:
    - "udp://127.0.0.1:50351"
  state_pub_rate_hz: 100
logging:
  enable: true
  directory: "./logs"
"""


def make_args(config: Path, artifact_dir: Path, duration_sec: float = 0.01) -> SimpleNamespace:
    return SimpleNamespace(
        root=Path("."),
        server=Path("missing-server"),
        config=config,
        arm="left",
        mode=accept.MODE,
        duration_sec=duration_sec,
        artifact_dir=artifact_dir,
        startup_timeout_sec=1.0,
        settle_sec=0.0,
        max_state_age_us=250_000.0,
        max_physical_motion_deg=0.05,
        max_reference_drift_deg=0.05,
        min_send_count_ratio=0.98,
        min_controller_acceptance_ratio=0.98,
        max_send_duration_p99_us=1000.0,
        max_servo_jitter_p99_ms=2.5,
        max_deadline_miss_count=0,
        max_worker_drop_count=0,
        set_pgmode_simulation=True,
        verify_pgmode_simulation=False,
        pgmode_timeout_sec=1.0,
        pgmode_command_port=5000,
        preflight_only=False,
        skip_plots=True,
        i_understand_this_connects_to_real_controller=True,
        i_confirm_controller_is_in_pgmode_simulation=True,
    )


def write_config(tmp: Path, operation_mode: str = "simulation") -> Path:
    path = tmp / "config.yaml"
    path.write_text(CONFIG_TEMPLATE.format(operation_mode=operation_mode), encoding="utf-8")
    return path


def required_env() -> dict[str, str]:
    return {
        "RB_ALLOW_REAL_ROBOT": "1",
        "RB_ALLOW_REAL_MOTION": "1",
        "RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION": "1",
        "RB_RBPODO_PGMODE_SIMULATION_CONFIRMED": "1",
    }


def fake_state(index: int, *, q_actual_5: float = 0.0, deadline_missed: bool = False) -> dict:
    host_time_ns = 1_000_000_000 + index * 2_000_000
    q_ref = [1.0, -2.0, 3.0, -4.0, 5.0, -6.0]
    return {
        "schema_version": 1,
        "host_time_ns": host_time_ns,
        "loop_start_time_ns": host_time_ns,
        "motion_state": "Running",
        "safety_verdict": "Ok",
        "fault_latched": False,
        "observed_backend": "rbpodo",
        "left": {
            "has_valid_joint_state": True,
            "q_actual_deg": [0.0, 0.0, 0.0, 0.0, 0.0, q_actual_5],
            "q_ref_deg": q_ref,
            "q_target_deg": q_ref,
            "state_age_us": 1000.0,
            "send_command_deadline_missed": deadline_missed,
            "last_send": {
                "accepted": True,
                "duration_us": 800.0,
                "ack_wait_duration_us": 600.0,
                "ack_policy": "wait",
                "ack_observed": True,
                "controller_acceptance_observed": True,
                "send_acceptance_semantics": "controller_ack_observed",
            },
            "worker": {
                "enabled": False,
                "command_drops_total": 0,
                "pending_overwrites_total": 0,
            },
        },
    }


class Rbpodo500HzAcceptanceTests(unittest.TestCase):
    def test_help_works(self) -> None:
        script = Path(__file__).with_name("rbpodo_500hz_acceptance.py")
        completed = subprocess.run(
            [sys.executable, str(script), "--help"],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("servo_j_noop_500hz", completed.stdout)

    def test_preflight_rejects_operation_mode_real(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text:
            tmp = Path(tmp_text)
            config_path = write_config(tmp, operation_mode="real")
            args = make_args(config_path, tmp / "artifacts")
            with mock.patch.dict(os.environ, required_env(), clear=False):
                os.environ.pop("RB_ALLOW_REAL_CARTESIAN", None)
                with self.assertRaisesRegex(accept.Acceptance500HzError, "operation_mode is real"):
                    accept.preflight(args, run_pgmode=False)

    def test_fake_state_stream_noop_pass(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text:
            tmp = Path(tmp_text)
            config_path = write_config(tmp)
            config = accept.load_config(config_path)
            args = make_args(config_path, tmp / "artifacts")
            states = [fake_state(index) for index in range(5)]
            command_run = accept.CommandRunMetrics(
                command_count=5,
                expected_command_count=5,
                start_host_time_ns=states[0]["host_time_ns"],
                end_host_time_ns=states[-1]["host_time_ns"],
                elapsed_sec=0.01,
                sender_deadline_missed_count=0,
                max_sender_lateness_us=0.0,
                hold_sent=True,
            )
            summary = accept.summarize_acceptance(
                args,
                config,
                {"passed": True, "mode": accept.MODE},
                states,
                command_run,
                tmp,
                None,
                None,
            )
            self.assertEqual(summary["result"], "pass", json.dumps(summary["threshold_failures"]))
            self.assertEqual(summary["send_count"], 5)
            self.assertEqual(summary["controller_acceptance_observed_count"], 5)
            self.assertFalse(summary["physical_motion_detected"])

    def test_fake_deadline_misses_fail(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text:
            tmp = Path(tmp_text)
            config_path = write_config(tmp)
            config = accept.load_config(config_path)
            args = make_args(config_path, tmp / "artifacts")
            states = [fake_state(index, deadline_missed=(index == 2)) for index in range(5)]
            command_run = accept.CommandRunMetrics(5, 5, states[0]["host_time_ns"], states[-1]["host_time_ns"], 0.01, 0, 0.0, True)
            summary = accept.summarize_acceptance(args, config, {"passed": True}, states, command_run, tmp, None, None)
            self.assertEqual(summary["result"], "fail")
            self.assertTrue(any("send_deadline_missed_count" in item for item in summary["threshold_failures"]))

    def test_fake_physical_motion_detected_fail(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text:
            tmp = Path(tmp_text)
            config_path = write_config(tmp)
            config = accept.load_config(config_path)
            args = make_args(config_path, tmp / "artifacts")
            states = [fake_state(index, q_actual_5=(0.1 if index == 4 else 0.0)) for index in range(5)]
            command_run = accept.CommandRunMetrics(5, 5, states[0]["host_time_ns"], states[-1]["host_time_ns"], 0.01, 0, 0.0, True)
            summary = accept.summarize_acceptance(args, config, {"passed": True}, states, command_run, tmp, None, None)
            self.assertEqual(summary["result"], "fail")
            self.assertTrue(summary["physical_motion_detected"])
            self.assertTrue(any("physical q_actual motion" in item for item in summary["threshold_failures"]))


if __name__ == "__main__":
    unittest.main()
