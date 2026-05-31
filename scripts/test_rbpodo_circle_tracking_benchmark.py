#!/usr/bin/env python3
"""Unit tests for rbpodo controller-simulation circle benchmark helpers."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import circle_tracking_benchmark as sim_bench
import rbpodo_circle_tracking_benchmark as bench


ENV_NAMES = (
    "RB_ALLOW_REAL_ROBOT",
    "RB_ALLOW_REAL_MOTION",
    "RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION",
    "RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN",
    "RB_ALLOW_RBPODO_ACK_DISABLED_MOTION",
    "RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM",
    "RB_ALLOW_REAL_CARTESIAN",
)


class EnvGuard:
    def __enter__(self) -> "EnvGuard":
        self.old = {name: os.environ.get(name) for name in ENV_NAMES}
        return self

    def __exit__(self, *_exc: object) -> None:
        for name, value in self.old.items():
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value


def write_config(path: Path, *, operation_mode: str = "simulation", state_fanout: bool = False) -> None:
    state_network = (
        '  state_pub_endpoints:\n'
        '    - "udp://127.0.0.1:50151"\n'
        '    - "udp://127.0.0.1:50161"\n'
        if state_fanout
        else '  state_pub_endpoint: "udp://127.0.0.1:50151"\n'
    )
    path.write_text(
        f"""schema: robotics_lab.rb_servo_server.v1
left_robot:
  backend_type: rbpodo
  run_mode: real
  ip: "172.28.60.200"
  operation_mode: {operation_mode}
  servo_t1_sec: 0.01
  servo_t2_sec: 0.05
  servo_gain: 1.0
  servo_alpha: 0.5
  disable_waiting_ack: false
right_robot:
  backend_type: rbpodo
  run_mode: real
  ip: "172.28.60.201"
  operation_mode: {operation_mode}
  servo_t1_sec: 0.01
  servo_t2_sec: 0.05
  servo_gain: 1.0
  servo_alpha: 0.5
  disable_waiting_ack: false
servo:
  rate_hz: 100
  send_servo_commands: true
  allow_controller_simulation_motion: true
  allow_controller_simulation_diagnostics_suspect: false
network:
  command_bind: "udp://127.0.0.1:50051"
{state_network.rstrip()}
cartesian_control:
  enable: true
  allow_in_simulation: true
  allow_in_real: false
  allow_in_controller_simulation: true
  enable_benchmark_primitives: false
  max_twist_linear_m_s: 0.15
  max_twist_angular_rad_s: 0.4
""",
        encoding="utf-8",
    )


def write_pgmode_summary(path: Path) -> None:
    path.write_text(
        json.dumps(
            {
                "schema": "robotics_lab.rainbow_pgmode.v1",
                "overall_result": "ok",
                "ips": ["172.28.60.200", "172.28.60.201"],
                "results": [
                    {"ip": "172.28.60.200", "ok": True, "controller_mode": "simulation"},
                    {"ip": "172.28.60.201", "ok": True, "controller_mode": "simulation"},
                ],
            }
        ),
        encoding="utf-8",
    )


def make_args(tmp: Path, config: Path, pgmode: Path, **overrides: object) -> argparse.Namespace:
    values: dict[str, object] = {
        "root": Path("."),
        "server": Path("missing"),
        "server_config": config,
        "arm": "left",
        "controller": "twist_stand_feedback",
        "plane": "xy",
        "profile": "circle_15cm_16s",
        "diameter_m": None,
        "period_sec": None,
        "repeat": 1,
        "command_rate_hz": 100.0,
        "warmup_sec": 0.0,
        "settle_sec": 0.0,
        "startup_timeout_sec": 1.0,
        "tracking_source": "auto",
        "feedback_kp_pos": 2.0,
        "feedback_kp_ori": 2.0,
        "feedback_max_linear_m_s": None,
        "feedback_max_angular_rad_s": None,
        "feedback_use_current_state_time": False,
        "physical_motion_warning_deg": 0.05,
        "max_state_age_us": 250_000.0,
        "max_allowed_rms_error_m": None,
        "max_allowed_p95_error_m": None,
        "max_allowed_orientation_drift_rad": None,
        "max_allowed_latency_ms": None,
        "pgmode_summary_json": pgmode,
        "set_pgmode_simulation": False,
        "verify_pgmode_simulation": False,
        "pgmode_timeout_sec": 1.0,
        "pgmode_command_port": 5000,
        "artifact_dir": tmp / "artifacts",
        "preflight_only": False,
        "skip_plots": True,
        "i_understand_this_connects_to_real_controller": True,
        "i_confirm_controller_is_in_pgmode_simulation": True,
    }
    values.update(overrides)
    return argparse.Namespace(**values)


_MISSING = object()


def state(
    host_time_ns: int,
    *,
    ref: dict[str, object] | None = None,
    actual: dict[str, object] | None = None,
    q_actual: list[float] | None = None,
    q_sent: list[float] | None = None,
    q_ref: list[float] | object | None = _MISSING,
    cartesian_solve: dict[str, object] | None = None,
    fault_latched: bool = False,
    latched_fault_reason: str | None = None,
    fault_reason: str | None = None,
) -> dict[str, object]:
    arm_state: dict[str, object] = {
        "has_valid_joint_state": True,
        "q_actual_deg": q_actual or [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "q_sent_deg": q_sent or q_actual or [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "tcp_ref_valid": ref is not None,
        "tcp_actual_valid": actual is not None,
    }
    if q_ref is _MISSING:
        arm_state["q_ref_deg"] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    elif q_ref is not None:
        arm_state["q_ref_deg"] = q_ref
    if ref is not None:
        arm_state["tcp_ref_stand"] = ref
    if actual is not None:
        arm_state["tcp_actual_stand"] = actual
    if cartesian_solve is not None:
        arm_state["cartesian_solve"] = cartesian_solve
    return {
        "schema_version": 1,
        "host_time_ns": host_time_ns,
        "fault_latched": fault_latched,
        "latched_fault_reason": latched_fault_reason,
        "fault_reason": fault_reason,
        "left": arm_state,
    }


def pose(x: float, y: float, z: float = 0.0) -> dict[str, object]:
    return {
        "x": x,
        "y": y,
        "z": z,
        "rx": 0.0,
        "ry": 0.0,
        "rz": 0.0,
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
    }


class RbpodoCircleTrackingBenchmarkTest(unittest.TestCase):
    def test_help_works(self) -> None:
        script = Path(__file__).with_name("rbpodo_circle_tracking_benchmark.py")
        completed = subprocess.run(
            [sys.executable, str(script), "--help"],
            check=True,
            stdout=subprocess.PIPE,
            text=True,
        )
        self.assertIn("--tracking-source", completed.stdout)
        self.assertIn("--overlay-pub-endpoint", completed.stdout)
        self.assertIn("--i-confirm-controller-is-in-pgmode-simulation", completed.stdout)

    def test_preflight_rejects_operation_mode_real(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text, EnvGuard():
            tmp = Path(tmp_text)
            config = tmp / "config.yaml"
            pgmode = tmp / "pgmode.json"
            write_config(config, operation_mode="real")
            write_pgmode_summary(pgmode)
            os.environ["RB_ALLOW_REAL_ROBOT"] = "1"
            os.environ["RB_ALLOW_REAL_MOTION"] = "1"
            os.environ["RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION"] = "1"
            os.environ.pop("RB_ALLOW_REAL_CARTESIAN", None)
            args = make_args(tmp, config, pgmode)
            with self.assertRaisesRegex(bench.BenchmarkError, "operation_mode is real"):
                bench.preflight(args)

    def test_preflight_rejects_missing_pgmode_confirmation(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text, EnvGuard():
            tmp = Path(tmp_text)
            config = tmp / "config.yaml"
            pgmode = tmp / "pgmode.json"
            write_config(config)
            write_pgmode_summary(pgmode)
            os.environ["RB_ALLOW_REAL_ROBOT"] = "1"
            os.environ["RB_ALLOW_REAL_MOTION"] = "1"
            os.environ["RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION"] = "1"
            os.environ.pop("RB_ALLOW_REAL_CARTESIAN", None)
            args = make_args(tmp, config, pgmode, i_confirm_controller_is_in_pgmode_simulation=False)
            with self.assertRaisesRegex(bench.BenchmarkError, "i-confirm-controller"):
                bench.preflight(args)

    def test_preflight_uses_first_state_pub_endpoint_from_fanout(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text, EnvGuard():
            tmp = Path(tmp_text)
            config = tmp / "config.yaml"
            pgmode = tmp / "pgmode.json"
            write_config(config, state_fanout=True)
            write_pgmode_summary(pgmode)
            os.environ["RB_ALLOW_REAL_ROBOT"] = "1"
            os.environ["RB_ALLOW_REAL_MOTION"] = "1"
            os.environ["RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION"] = "1"
            os.environ["RB_ALLOW_RBPODO_CONTROLLER_SIM_CARTESIAN"] = "1"
            os.environ.pop("RB_ALLOW_REAL_CARTESIAN", None)
            args = make_args(tmp, config, pgmode)
            _config, _sections, preflight, endpoints = bench.preflight(args)
            self.assertEqual(preflight["state_endpoint"], "udp://127.0.0.1:50151")
            self.assertEqual(endpoints["state_port"], 50151)

    def test_tracking_source_auto_chooses_tcp_ref(self) -> None:
        source, warning = bench.select_tracking_source(
            "auto",
            state(100, ref=pose(0.1, 0.0), actual=pose(0.0, 0.0)),
            "left",
        )
        self.assertEqual(source, "tcp_ref_stand")
        self.assertIsNone(warning)

    def test_tracking_source_auto_fails_without_tcp_ref(self) -> None:
        with self.assertRaisesRegex(bench.BenchmarkError, "tcp_ref_stand"):
            bench.select_tracking_source("auto", state(100, actual=pose(0.0, 0.0)), "left")

    def test_metrics_use_tcp_ref_stand_samples(self) -> None:
        args = argparse.Namespace(arm="left")
        traj = sim_bench.Trajectory(
            start=[0.075, 0.0, 0.0],
            axis1=[1.0, 0.0, 0.0],
            axis2=[0.0, 1.0, 0.0],
            radius=0.075,
            period_sec=4.0,
        )
        snapshots = [
            state(1_000_000_000, ref=pose(0.075, 0.0), actual=pose(0.0, 0.0)),
            state(2_000_000_000, ref=pose(0.0, 0.075), actual=pose(0.0, 0.0)),
        ]
        samples = bench.collect_samples(snapshots, "left", "tcp_ref_stand", 1_000_000_000, 2_000_000_000)
        metrics, _rows = sim_bench.compute_metrics(
            args=args,
            traj=traj,
            q0=[0.0, 0.0, 0.0, 1.0],
            samples=samples,
            benchmark_start_ns=1_000_000_000,
        )
        self.assertLess(metrics["rms_error_m"], 1e-9)
        self.assertEqual(bench.tcp_valid_ratio(snapshots, "left", "tcp_ref_stand"), 1.0)

    def test_physical_motion_detection_uses_q_actual_drift(self) -> None:
        snapshots = [
            state(1_000_000_000, ref=pose(0.075, 0.0), q_actual=[0.0, 0, 0, 0, 0, 0]),
            state(2_000_000_000, ref=pose(0.0, 0.075), q_actual=[0.08, 0, 0, 0, 0, 0]),
        ]
        drift = bench.max_q_drift(bench.q_series(snapshots, "left", "q_actual_deg"))
        self.assertIsNotNone(drift)
        self.assertGreater(drift, 0.05)

    def test_motion_metrics_distinguish_sent_ref_actual_sources(self) -> None:
        snapshots = [
            state(
                1_000_000_000,
                ref=pose(0.075, 0.0),
                actual=pose(0.0, 0.0),
                q_actual=[0.0, 0, 0, 0, 0, 0],
                q_sent=[0.0, 0, 0, 0, 0, 0],
                q_ref=None,
            ),
            state(
                2_000_000_000,
                ref=pose(0.0, 0.075),
                actual=pose(0.0, 0.0),
                q_actual=[0.0, 0, 0, 0, 0, 0],
                q_sent=[1.0, 0, 0, 0, 0, 0],
                q_ref=None,
            ),
        ]
        q_sent = bench.q_motion_metrics(snapshots, "left", "q_sent_deg")
        q_actual = bench.q_motion_metrics(snapshots, "left", "q_actual_deg")
        q_ref = bench.q_motion_metrics(snapshots, "left", "q_ref_deg")
        runtime = bench.cartesian_runtime_diagnostics(snapshots, "left", 1_000_000_000, 2_000_000_000)
        self.assertTrue(q_sent["q_sent_moved"])
        self.assertGreater(q_sent["q_sent_update_rate_hz"], 0.0)
        self.assertFalse(q_actual["q_actual_moved"])
        self.assertEqual(q_actual["q_actual_update_rate_hz"], 0.0)
        self.assertIsNone(q_ref["q_ref_moved"])
        self.assertIsNone(q_ref["q_ref_update_rate_hz"])
        self.assertEqual(q_ref["q_ref_reason"], "q_ref_deg not published")
        self.assertTrue(bench.pose_moved(runtime["tcp_ref_displacement_m"]))
        self.assertFalse(bench.pose_moved(runtime["tcp_actual_displacement_m"]))

    def test_summary_warns_when_integrator_diverges_and_actual_is_static(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text:
            tmp = Path(tmp_text)
            args = make_args(
                tmp,
                tmp / "config.yaml",
                tmp / "pgmode.json",
                diameter_m=0.15,
                period_sec=4.0,
                artifact_dir=tmp / "artifacts",
                skip_plots=True,
                controller="twist_stand",
            )
            traj = sim_bench.Trajectory(
                start=[0.075, 0.0, 0.0],
                axis1=[1.0, 0.0, 0.0],
                axis2=[0.0, 1.0, 0.0],
                radius=0.075,
                period_sec=4.0,
            )
            samples: list[tuple[int, dict[str, object], list[float]]] = [
                (1_000_000_000, pose(0.075, 0.0), [0.0, 0, 0, 0, 0, 0]),
                (2_000_000_000, pose(0.0, 0.075), [0.5, 0, 0, 0, 0, 0]),
                (3_000_000_000, pose(-0.075, 0.0), [1.0, 0, 0, 0, 0, 0]),
                (4_000_000_000, pose(0.0, -0.075), [1.5, 0, 0, 0, 0, 0]),
                (5_000_000_000, pose(0.075, 0.0), [2.0, 0, 0, 0, 0, 0]),
            ]
            states = [
                state(
                    host_time_ns,
                    ref=ref,
                    actual=pose(0.0, 0.0),
                    q_actual=[0.0, 0, 0, 0, 0, 0],
                    q_sent=q_sent,
                    q_ref=None,
                    cartesian_solve={
                        "integrator_resets_total": 20,
                        "integrator_divergence_total": 20,
                        "integrator_clamps_total": 5,
                        "status": "ok",
                        "attempted": True,
                        "success": True,
                    },
                )
                for host_time_ns, ref, q_sent in samples
            ]
            summary = bench.summarize_run(
                args,
                None,  # type: ignore[arg-type]
                {"required_tangential_speed_m_s": 0.1178},
                states,
                traj,
                [0.0, 0.0, 0.0, 1.0],
                "tcp_ref_stand",
                None,
                1_000_000_000,
                5_000_000_000,
                400,
                [],
                tmp / "artifacts",
                0,
                {},
            )
            self.assertTrue(summary["q_sent_moved"])
            self.assertIsNone(summary["q_ref_moved"])
            self.assertEqual(summary["q_ref_reason"], "q_ref_deg not published")
            self.assertTrue(summary["tcp_ref_moved"])
            self.assertFalse(summary["tcp_actual_moved"])
            self.assertFalse(summary["q_actual_moved"])
            self.assertEqual(summary["integrator_divergence_total"], 20.0)
            self.assertEqual(summary["divergence_rate_hz"], 5.0)
            self.assertIn(
                "controller-simulation q_actual is stationary; Cartesian integration may need reference-state source.",
                summary["performance_warnings"],
            )

    def test_summary_marks_latched_tracking_error_as_faulted(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_text:
            tmp = Path(tmp_text)
            args = make_args(
                tmp,
                tmp / "config.yaml",
                tmp / "pgmode.json",
                diameter_m=0.15,
                period_sec=4.0,
                artifact_dir=tmp / "artifacts",
                skip_plots=True,
                controller="twist_stand",
            )
            traj = sim_bench.Trajectory(
                start=[0.075, 0.0, 0.0],
                axis1=[1.0, 0.0, 0.0],
                axis2=[0.0, 1.0, 0.0],
                radius=0.075,
                period_sec=4.0,
            )
            states = [
                state(1_000_000_000, ref=pose(0.075, 0.0), actual=pose(0.0, 0.0)),
                state(2_000_000_000, ref=pose(0.0, 0.075), actual=pose(0.0, 0.0)),
                state(
                    3_000_000_000,
                    ref=pose(-0.075, 0.0),
                    actual=pose(0.0, 0.0),
                    fault_latched=True,
                    latched_fault_reason="TrackingError",
                    fault_reason="reference tracking error exceeded threshold",
                ),
            ]
            summary = bench.summarize_run(
                args,
                None,  # type: ignore[arg-type]
                {"required_tangential_speed_m_s": 0.1178},
                states,
                traj,
                [0.0, 0.0, 0.0, 1.0],
                "tcp_ref_stand",
                None,
                1_000_000_000,
                5_000_000_000,
                200,
                [],
                tmp / "artifacts",
                1,
                {},
            )
            self.assertEqual(summary["result"], "faulted")
            self.assertTrue(summary["fault_latched"])
            self.assertEqual(summary["latched_fault_reason"], "TrackingError")
            self.assertEqual(summary["first_fault_time_sec"], 2.0)
            self.assertIn("server fault latched", summary["result_reason"])


if __name__ == "__main__":
    unittest.main()
