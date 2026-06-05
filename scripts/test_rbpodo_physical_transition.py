import contextlib
import io
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import generate_rbpodo_physical_transition_report as report
import rbpodo_physical_transition_acceptance as accept


CONFIG_TEMPLATE = """schema: robotics_lab.rb_servo_server.v1
left_robot:
  backend_type: rbpodo
  run_mode: real
  ip: "172.28.60.200"
  operation_mode: real
  speed_bar: 0.05
  servo_t1_sec: 0.01
  servo_t2_sec: 0.05
  servo_gain: 1.0
  servo_alpha: 0.5
  disable_waiting_ack: false
right_robot:
  backend_type: rbpodo
  run_mode: real
  ip: "172.28.60.201"
  operation_mode: real
  speed_bar: 0.05
  servo_t1_sec: 0.01
  servo_t2_sec: 0.05
  servo_gain: 1.0
  servo_alpha: 0.5
  disable_waiting_ack: false
servo:
  rate_hz: 100
  send_servo_commands: {send_servo_commands}
force_control:
  provider: null
  enable: false
cartesian_control:
  enable: {cartesian_enable}
  allow_in_real: {allow_in_real}
network:
  command_bind: "udp://127.0.0.1:50451"
  state_pub_endpoint: "udp://127.0.0.1:50551"
"""


ALL_FLAGS = [
    "--i-understand-this-may-move-the-physical-robot",
    "--i-have-clear-workspace",
    "--i-have-estop-in-hand",
    "--i-reviewed-local-config",
    "--i-confirm-operator-supervision",
]


def write_config(
    root: Path,
    relative: str,
    *,
    send_servo_commands: bool,
    cartesian_enable: bool = False,
    allow_in_real: bool = False,
) -> Path:
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        CONFIG_TEMPLATE.format(
            send_servo_commands=str(send_servo_commands).lower(),
            cartesian_enable=str(cartesian_enable).lower(),
            allow_in_real=str(allow_in_real).lower(),
        ),
        encoding="utf-8",
    )
    return path


def run_accept(argv: list[str], env: dict[str, str] | None = None) -> tuple[int, str, str]:
    stdout = io.StringIO()
    stderr = io.StringIO()
    with mock.patch.dict(os.environ, env or {}, clear=True):
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
            code = accept.main(argv)
    return code, stdout.getvalue(), stderr.getvalue()


class PhysicalTransitionAcceptanceTest(unittest.TestCase):
    def test_dry_run_prints_required_gates_and_does_not_call_hardware_path(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            artifact_dir = Path(tmpdir) / "artifact"
            with mock.patch.object(accept, "live_preflight_placeholder") as live:
                code, stdout, stderr = run_accept(
                    [
                        "--stage",
                        "tiny_cartesian",
                        "--dry-run",
                        "--artifact-dir",
                        str(artifact_dir),
                    ]
                )
        self.assertEqual(code, 0, stderr)
        self.assertIn("RB_ALLOW_REAL_ROBOT=1", stdout)
        self.assertIn("RB_ALLOW_REAL_MOTION=1", stdout)
        self.assertIn("RB_ALLOW_REAL_CARTESIAN=1", stdout)
        self.assertIn("Hardware process started: false", stdout)
        self.assertIn("Motion command sent: false", stdout)
        live.assert_not_called()

    def test_readonly_config_refuses_motion_stage_execute(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            config = write_config(
                root,
                "rb_servo_server/config/local/readonly.yaml",
                send_servo_commands=False,
            )
            code, stdout, stderr = run_accept(
                [
                    "--root",
                    str(root),
                    "--stage",
                    "tiny_joint",
                    "--execute",
                    "--config",
                    str(config),
                    *ALL_FLAGS,
                ],
                env={"RB_ALLOW_REAL_ROBOT": "1", "RB_ALLOW_REAL_MOTION": "1"},
            )
        self.assertEqual(code, 2)
        self.assertIn("requires servo.send_servo_commands=true", stdout + stderr)

    def test_tiny_cartesian_refuses_without_real_cartesian_gate(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            config = write_config(
                root,
                "rb_servo_server/config/local/tiny_cartesian.yaml",
                send_servo_commands=True,
                cartesian_enable=True,
                allow_in_real=True,
            )
            code, stdout, stderr = run_accept(
                [
                    "--root",
                    str(root),
                    "--stage",
                    "tiny_cartesian",
                    "--execute",
                    "--config",
                    str(config),
                    *ALL_FLAGS,
                ],
                env={"RB_ALLOW_REAL_ROBOT": "1", "RB_ALLOW_REAL_MOTION": "1"},
            )
        self.assertEqual(code, 2)
        self.assertIn("RB_ALLOW_REAL_CARTESIAN=1", stdout + stderr)

    def test_tracked_example_refuses_send_servo_commands_true(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            config = write_config(
                root,
                "rb_servo_server/config/bad.example.yaml",
                send_servo_commands=True,
            )
            code, stdout, stderr = run_accept(
                [
                    "--root",
                    str(root),
                    "--stage",
                    "read_only",
                    "--dry-run",
                    "--config",
                    str(config),
                ]
            )
        self.assertEqual(code, 2)
        self.assertIn("tracked example config must keep servo.send_servo_commands=false", stdout + stderr)


def write_stage_summary(
    root: Path,
    stage_id: str,
    *,
    status: str = "pass",
    physical_status: str = "not_run",
    physical_source: str = "tcp_actual_stand",
) -> Path:
    path = root / stage_id / "summary.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "schema": "robotics_lab.rbpodo_physical_transition.stage.v1",
        "stage": {"id": stage_id, "ladder_name": stage_id},
        "result": {"status": status},
        "physical_tracking_result": {
            "status": physical_status,
            "tracking_source": physical_source,
            "rms_error_m": 0.001,
            "p95_error_m": 0.002,
            "max_error_m": 0.003,
        },
        "controller_reference_result": {
            "status": "informational_only",
            "tracking_source": "tcp_ref_stand",
        },
        "telemetry_requirements": {
            "state_age_us": {"p95": 1000.0},
            "state_jitter_us": {"p95": 100.0},
            "q_actual_update_rate_hz": 100.0,
            "q_ref_update_rate_hz": 100.0,
            "fault_latch_status": "pass",
            "cartesian_availability": "available",
            "stop_reset_behavior_result": "pass",
            "physical_motion_expected": physical_status == "pass",
            "physical_motion_detected": physical_status == "pass",
        },
    }
    path.write_text(json.dumps(data), encoding="utf-8")
    return path


class PhysicalTransitionReportTest(unittest.TestCase):
    def test_report_rejects_physical_pass_from_tcp_ref_stand(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            write_stage_summary(root, "P5", physical_status="pass", physical_source="tcp_ref_stand")
            stdout = io.StringIO()
            stderr = io.StringIO()
            with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
                code = report.main(["--artifact-dir", str(root)])
        self.assertEqual(code, 2)
        self.assertIn("refusing physical pass", stderr.getvalue())

    def test_report_readiness_blocked_until_prerequisite_artifact_refs_exist(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            write_stage_summary(root, "P0")
            out_json = root / "report.json"
            stdout = io.StringIO()
            with contextlib.redirect_stdout(stdout):
                code = report.main(["--artifact-dir", str(root), "--json", str(out_json)])
            self.assertEqual(code, 0)
            data = json.loads(out_json.read_text(encoding="utf-8"))
        self.assertEqual(data["physical_readiness"]["status"], "blocked")
        self.assertIn("missing_P1_artifact", data["physical_readiness"]["blockers"])


if __name__ == "__main__":
    unittest.main()
