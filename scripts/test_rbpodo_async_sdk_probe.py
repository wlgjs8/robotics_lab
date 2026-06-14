import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import rbpodo_async_sdk_probe as probe


def args(**overrides):
    base = {
        "ip": "127.0.0.1",
        "duration_sec": 0.01,
        "rate_hz": 500.0,
        "mode": "ack_on",
        "artifact_dir": Path(tempfile.mkdtemp()),
        "operation_mode": "simulation",
        "config": None,
        "read_timeout_sec": 0.01,
        "command_timeout_sec": 0.01,
        "servo_t2_sec": 0.03,
        "servo_gain": 1.0,
        "servo_alpha": 0.5,
        "max_noop_target_delta_deg": 0.05,
        "max_q_actual_drift_deg": 0.05,
        "late_ack_poll_sec": 0.0,
        "set_pgmode_simulation": True,
        "verify_pgmode_simulation": False,
        "pgmode_timeout_sec": 0.1,
        "pgmode_command_port": 5000,
        "i_understand_this_connects_to_real_controller": True,
        "allow_simulation_servo_j": True,
    }
    base.update(overrides)
    return SimpleNamespace(**base)


def summary_for(mode: str, metrics: dict, caps: dict | None = None) -> dict:
    return {
        "mode": mode,
        "rate_hz": 500.0,
        "metrics": metrics,
        "sdk_capabilities": caps or {
            "module_available": True,
            "has_cobot": True,
            "has_cobot_data": True,
            "has_response_collector": True,
            "has_move_servo_j": True,
            "has_disable_waiting_ack": True,
        },
    }


class RbpodoAsyncSdkProbeTests(unittest.TestCase):
    def test_help_works(self) -> None:
        script = Path(__file__).with_name("rbpodo_async_sdk_probe.py")
        completed = subprocess.run(
            [sys.executable, str(script), "--help"],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("ack_on", completed.stdout)
        self.assertIn("--allow-simulation-servo-j", completed.stdout)
        self.assertIn("--set-pgmode-simulation", completed.stdout)

    def test_no_confirmation_flag_fails(self) -> None:
        script = Path(__file__).with_name("rbpodo_async_sdk_probe.py")
        completed = subprocess.run(
            [
                sys.executable,
                str(script),
                "--ip",
                "127.0.0.1",
                "--duration-sec",
                "0.01",
                "--rate-hz",
                "500",
                "--mode",
                "ack_on",
                "--artifact-dir",
                tempfile.mkdtemp(),
                "--set-pgmode-simulation",
                "--allow-simulation-servo-j",
            ],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        self.assertEqual(completed.returncode, 2)
        self.assertIn("--i-understand-this-connects-to-real-controller", completed.stderr)

    def test_operation_mode_real_rejected(self) -> None:
        with self.assertRaisesRegex(probe.AsyncSdkProbeError, "operation_mode real is refused"):
            probe.preflight(args(operation_mode="real"), run_pgmode=False)

    def test_config_operation_mode_real_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text:
            tmp = Path(tmp_text)
            config = tmp / "config.yaml"
            config.write_text(
                """
left_robot:
  operation_mode: real
right_robot:
  operation_mode: simulation
""",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(probe.AsyncSdkProbeError, "operation_mode is real"):
                probe.preflight(args(config=config), run_pgmode=False)

    def test_classifies_ack_on_fast_enough(self) -> None:
        classification, reasons = probe.classify_summary(summary_for("ack_on", {
            "send_count": 100,
            "send_success_count": 100,
            "send_success_ratio": 1.0,
            "ack_observed_count": 100,
            "send_duration_us": {"p95": 900.0, "p99": 1500.0, "max": 1800.0},
        }))
        self.assertEqual(classification, "ack_on_fast_enough", reasons)

    def test_classifies_ack_on_outlier_limited(self) -> None:
        classification, reasons = probe.classify_summary(summary_for("ack_on", {
            "send_count": 100,
            "send_success_count": 100,
            "send_success_ratio": 1.0,
            "ack_observed_count": 100,
            "send_duration_us": {"p95": 1500.0, "p99": 3500.0, "max": 6000.0},
        }))
        self.assertEqual(classification, "ack_on_outlier_limited", reasons)

    def test_classifies_ack_off_state_supervision_viable(self) -> None:
        classification, reasons = probe.classify_summary(summary_for("ack_off", {
            "send_count": 100,
            "send_success_count": 100,
            "send_success_ratio": 1.0,
            "socket_send_only_count": 100,
            "state_sample_count": 100,
            "q_ref_finite_sample_count": 100,
            "q_ref_drift_deg": 0.0,
            "send_duration_us": {"p95": 150.0, "p99": 200.0, "max": 250.0},
        }))
        self.assertEqual(classification, "ack_off_state_supervision_viable", reasons)

    def test_classifies_concurrent_read_send_viable(self) -> None:
        classification, reasons = probe.classify_summary(summary_for("concurrent_read_send", {
            "send_count": 50,
            "send_success_count": 50,
            "send_success_ratio": 1.0,
            "state_valid_sample_count": 60,
            "concurrent_read_error_count": 0,
            "concurrent_send_error_count": 0,
            "send_duration_us": {"p95": 900.0, "p99": 1500.0, "max": 1800.0},
        }))
        self.assertEqual(classification, "concurrent_read_send_viable", reasons)

    def test_classifies_sdk_async_ack_not_supported(self) -> None:
        classification, reasons = probe.classify_summary(summary_for(
            "ack_off",
            {
                "send_count": 10,
                "send_success_count": 10,
                "send_success_ratio": 1.0,
                "socket_send_only_count": 10,
            },
            caps={
                "module_available": True,
                "has_cobot": True,
                "has_cobot_data": True,
                "has_response_collector": True,
                "has_move_servo_j": True,
                "has_disable_waiting_ack": False,
            },
        ))
        self.assertEqual(classification, "sdk_async_ack_not_supported", reasons)

    def test_classifies_insufficient_evidence(self) -> None:
        classification, _reasons = probe.classify_summary(summary_for("ack_on", {
            "send_count": 0,
            "send_success_count": 0,
            "send_success_ratio": 0.0,
            "ack_observed_count": 0,
            "send_duration_us": {},
        }))
        self.assertEqual(classification, "insufficient_evidence")


if __name__ == "__main__":
    unittest.main()
