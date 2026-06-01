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
  disable_waiting_ack: {disable_waiting_ack}
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
  disable_waiting_ack: {disable_waiting_ack}
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
cartesian_control:
  enable: {cartesian_enable}
  allow_in_controller_simulation: true
  allow_in_real: false
"""


def make_args(config: Path, artifact_dir: Path, duration_sec: float = 0.01) -> SimpleNamespace:
    return SimpleNamespace(
        root=Path("."),
        server=Path("missing-server"),
        config=config,
        arm="left",
        send_arms=None,
        mode=accept.MODE,
        duration_sec=duration_sec,
        artifact_dir=artifact_dir,
        command_timeout_sec=None,
        warmup_duration_sec=0.0,
        warmup_rate_hz=100.0,
        warmup_command_timeout_sec=None,
        ack_timeout_sweep=None,
        disable_waiting_ack_diagnostic=False,
        preserve_cartesian_control=False,
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


def write_config(
    tmp: Path,
    operation_mode: str = "simulation",
    *,
    cartesian_enable: bool = True,
    disable_waiting_ack: bool = False,
) -> Path:
    path = tmp / "config.yaml"
    path.write_text(
        CONFIG_TEMPLATE.format(
            operation_mode=operation_mode,
            cartesian_enable=str(cartesian_enable).lower(),
            disable_waiting_ack=str(disable_waiting_ack).lower(),
        ),
        encoding="utf-8",
    )
    return path


def required_env() -> dict[str, str]:
    return {
        "RB_ALLOW_REAL_ROBOT": "1",
        "RB_ALLOW_REAL_MOTION": "1",
        "RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION": "1",
        "RB_RBPODO_PGMODE_SIMULATION_CONFIRMED": "1",
    }


def fake_arm_state(
    *,
    q_actual_5: float = 0.0,
    deadline_missed: bool = False,
    ack_timeout: bool = False,
    ack_disabled: bool = False,
) -> dict:
    q_ref = [1.0, -2.0, 3.0, -4.0, 5.0, -6.0]
    if ack_timeout:
        last_send = {
            "accepted": False,
            "duration_us": 5100.0,
            "ack_wait_duration_us": 5000.0,
            "ack_policy": "wait",
            "ack_observed": False,
            "controller_acceptance_observed": False,
            "backend_error_kind": "TransportTimeout",
            "error_name": "rbpodo_move_servo_j_timeout",
            "reason": "rbpodo move_servo_j timed out waiting for command acknowledgement",
            "send_acceptance_semantics": "controller_ack_observed",
        }
        send_ok = False
    elif ack_disabled:
        last_send = {
            "accepted": True,
            "duration_us": 150.0,
            "ack_wait_duration_us": 0.0,
            "ack_policy": "disabled",
            "ack_observed": False,
            "controller_acceptance_observed": False,
            "send_acceptance_semantics": "socket_send_only",
        }
        send_ok = True
    else:
        last_send = {
            "accepted": True,
            "duration_us": 800.0,
            "ack_wait_duration_us": 600.0,
            "ack_policy": "wait",
            "ack_observed": True,
            "controller_acceptance_observed": True,
            "send_acceptance_semantics": "controller_ack_observed",
        }
        send_ok = True
    return {
        "has_valid_joint_state": True,
        "q_actual_deg": [0.0, 0.0, 0.0, 0.0, 0.0, q_actual_5],
        "q_ref_deg": q_ref,
        "q_target_deg": q_ref,
        "state_age_us": 1000.0,
        "send_ok": send_ok,
        "send_command_deadline_missed": deadline_missed,
        "last_send": last_send,
        "worker": {
            "enabled": False,
            "command_drops_total": 0,
            "pending_overwrites_total": 0,
        },
    }


def fake_state(
    index: int,
    *,
    q_actual_5: float = 0.0,
    deadline_missed: bool = False,
    right_ack_timeout: bool = False,
    left_ack_timeout: bool = False,
    ack_disabled: bool = False,
) -> dict:
    host_time_ns = 1_000_000_000 + index * 2_000_000
    return {
        "schema_version": 1,
        "host_time_ns": host_time_ns,
        "loop_start_time_ns": host_time_ns,
        "motion_state": "Running",
        "safety_verdict": "Ok",
        "fault_latched": right_ack_timeout or left_ack_timeout,
        "observed_backend": "rbpodo",
        "left": fake_arm_state(
            q_actual_5=q_actual_5,
            deadline_missed=deadline_missed,
            ack_timeout=left_ack_timeout,
            ack_disabled=ack_disabled,
        ),
        "right": fake_arm_state(ack_timeout=right_ack_timeout, ack_disabled=ack_disabled),
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

    def test_missing_cartesian_env_reported_when_config_requires_it(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text:
            tmp = Path(tmp_text)
            config_path = write_config(tmp, cartesian_enable=True)
            args = make_args(config_path, tmp / "artifacts")
            args.preserve_cartesian_control = True
            env = required_env()
            env.pop("RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN", None)
            with mock.patch.dict(os.environ, env, clear=True):
                with self.assertRaises(accept.Acceptance500HzError) as ctx:
                    accept.preflight(args, run_pgmode=False)
            self.assertEqual(ctx.exception.failure_phase, "preflight")
            self.assertEqual(
                ctx.exception.failure_classification,
                "preflight_env_missing_or_config_mismatch",
            )
            self.assertIn("RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN", str(ctx.exception))

    def test_noop_resolved_config_disables_cartesian_without_cartesian_env(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text:
            tmp = Path(tmp_text)
            config_path = write_config(tmp, cartesian_enable=True)
            args = make_args(config_path, tmp / "artifacts")
            env = required_env()
            env.pop("RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN", None)
            with mock.patch.dict(os.environ, env, clear=True):
                _config, preflight = accept.preflight(args, run_pgmode=False)
            self.assertTrue(preflight["cartesian_control_disabled_for_noop"])
            self.assertFalse(preflight["cartesian_control_enable"])
            self.assertFalse(preflight["cartesian_env_required"])
            resolved_text = Path(preflight["resolved_config"]).read_text(encoding="utf-8")
            self.assertIn("enable: false", resolved_text)
            self.assertIn("allow_in_controller_simulation: false", resolved_text)

    def test_command_timeout_override_writes_resolved_config(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text:
            tmp = Path(tmp_text)
            config_path = write_config(tmp, cartesian_enable=False)
            args = make_args(config_path, tmp / "artifacts")
            args.command_timeout_sec = 0.02
            with mock.patch.dict(os.environ, required_env(), clear=True):
                _config, preflight = accept.preflight(args, run_pgmode=False)
            self.assertEqual(preflight["command_timeout_sec_left"], 0.02)
            self.assertEqual(preflight["command_timeout_sec_right"], 0.02)
            resolved_text = Path(preflight["resolved_config"]).read_text(encoding="utf-8")
            self.assertIn("command_timeout_sec: 0.02", resolved_text)

    def test_ack_off_diagnostic_requires_ack_disabled_env(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text:
            tmp = Path(tmp_text)
            config_path = write_config(tmp, cartesian_enable=False)
            args = make_args(config_path, tmp / "artifacts")
            args.disable_waiting_ack_diagnostic = True
            env = required_env()
            env.pop("RB_ALLOW_RBPODO_ACK_DISABLED_MOTION", None)
            with mock.patch.dict(os.environ, env, clear=True):
                with self.assertRaises(accept.Acceptance500HzError) as ctx:
                    accept.preflight(args, run_pgmode=False)
            self.assertEqual(ctx.exception.failure_phase, "preflight")
            self.assertEqual(ctx.exception.failure_classification, "preflight_env_missing")
            self.assertIn("RB_ALLOW_RBPODO_ACK_DISABLED_MOTION", str(ctx.exception))

    def test_ack_off_diagnostic_writes_resolved_config(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text:
            tmp = Path(tmp_text)
            config_path = write_config(tmp, cartesian_enable=False)
            args = make_args(config_path, tmp / "artifacts")
            args.disable_waiting_ack_diagnostic = True
            env = {**required_env(), "RB_ALLOW_RBPODO_ACK_DISABLED_MOTION": "1"}
            with mock.patch.dict(os.environ, env, clear=True):
                _config, preflight = accept.preflight(args, run_pgmode=False)
            self.assertTrue(preflight["disable_waiting_ack"])
            self.assertTrue(preflight["disable_waiting_ack_diagnostic"])
            self.assertEqual(preflight["ack_semantics"], "socket_send_only")
            self.assertFalse(preflight["controller_acceptance_measured"])
            resolved_text = Path(preflight["resolved_config"]).read_text(encoding="utf-8")
            self.assertEqual(resolved_text.count("disable_waiting_ack: true"), 2)

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
            self.assertIsNone(summary["failure_phase"])
            self.assertEqual(summary["failure_classification"], None)

    def test_fake_ack_off_diagnostic_pass_without_controller_acceptance(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text:
            tmp = Path(tmp_text)
            config_path = write_config(tmp)
            config = accept.load_config(config_path)
            args = make_args(config_path, tmp / "artifacts")
            args.disable_waiting_ack_diagnostic = True
            states = [fake_state(index, ack_disabled=True) for index in range(5)]
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
                {
                    "passed": True,
                    "mode": accept.MODE,
                    "disable_waiting_ack": True,
                    "disable_waiting_ack_diagnostic": True,
                    "ack_semantics": "socket_send_only",
                    "controller_acceptance_measured": False,
                },
                states,
                command_run,
                tmp,
                None,
                None,
            )
            self.assertEqual(summary["result"], "pass", json.dumps(summary["threshold_failures"]))
            self.assertEqual(summary["controller_acceptance_observed_count"], 0)
            self.assertFalse(summary["controller_acceptance_measured"])
            self.assertEqual(summary["ack_semantics"], "socket_send_only")
            self.assertIn("controller ACK acceptance not measured", summary["result_reason"])

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
            self.assertEqual(summary["failure_classification"], "deadline_limited")
            self.assertTrue(any("send_deadline_missed_count" in item for item in summary["threshold_failures"]))

    def test_fake_right_ack_timeout_classified_measurement(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text:
            tmp = Path(tmp_text)
            config_path = write_config(tmp)
            config = accept.load_config(config_path)
            args = make_args(config_path, tmp / "artifacts")
            args.send_arms = "both"
            states = [fake_state(index, right_ack_timeout=(index == 2)) for index in range(5)]
            command_run = accept.CommandRunMetrics(5, 5, states[0]["host_time_ns"], states[-1]["host_time_ns"], 0.01, 0, 0.0, True)
            summary = accept.summarize_acceptance(args, config, {"passed": True}, states, command_run, tmp, None, None)
            self.assertEqual(summary["result"], "fail")
            self.assertEqual(summary["failure_phase"], "measurement")
            self.assertEqual(summary["failure_classification"], "measurement_ack_timeout")
            self.assertEqual(summary["first_send_failure_arm"], "right")
            self.assertGreater(summary["measurement_timeout_count"], 0)
            self.assertGreater(summary["ack_timeout_count_by_arm"]["right"], 0)

    def test_fake_warmup_timeout_classified_warmup(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text:
            tmp = Path(tmp_text)
            config_path = write_config(tmp)
            config = accept.load_config(config_path)
            args = make_args(config_path, tmp / "artifacts")
            states = [fake_state(index, left_ack_timeout=(index == 1)) for index in range(5)]
            command_run = accept.CommandRunMetrics(5, 5, states[0]["host_time_ns"], states[-1]["host_time_ns"], 0.01, 0, 0.0, True)
            summary = accept.summarize_acceptance(
                args,
                config,
                {"passed": True},
                states,
                command_run,
                tmp,
                None,
                None,
                phase="warmup",
            )
            self.assertEqual(summary["result"], "fail")
            self.assertEqual(summary["failure_phase"], "warmup")
            self.assertEqual(summary["failure_classification"], "warmup_ack_timeout")
            self.assertGreater(summary["warmup_timeout_count"], 0)
            self.assertEqual(summary["measurement_timeout_count"], 0)

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
