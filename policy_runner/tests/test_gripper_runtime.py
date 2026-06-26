from __future__ import annotations

import io
import logging
import unittest

from policy_runner.config import config_from_mapping
from policy_runner.action_sources.tcp_pose_target import tcp_pose_target_stand_intent
from policy_runner.gripper import (
    GripperCommand,
    GripperRuntime,
    NoopGripperBackend,
    PikaSerialGripperBackend,
    REAL_GRIPPER_ENV,
    gripper_commands_from_flow_step,
    suppress_pika_sdk_logging,
)
from policy_runner.servo_command_client import CommandIntent


class FakePikaGripper:
    def __init__(self, port: str):
        self.port = port
        self.position = 0.5
        self.sent_angles: list[float] = []
        self.closed_calls: list[str] = []
        self.zero_calls = 0

    def connect(self) -> bool:
        return True

    def enable(self) -> bool:
        return True

    def get_motor_position(self) -> float:
        return self.position

    def set_motor_angle(self, rad: float) -> bool:
        self.sent_angles.append(float(rad))
        return True

    def set_zero(self) -> bool:
        self.zero_calls += 1
        self.position = 0.0  # closed stop becomes the new zero
        return True

    def disable(self) -> None:
        self.closed_calls.append("disable")

    def disconnect(self) -> None:
        self.closed_calls.append("disconnect")


def _pika_backend(**kwargs) -> PikaSerialGripperBackend:
    clock = {"now": 0.0}
    # Default OFF for the send/clamp tests below: homing would prepend a
    # closed-stop write and reset the seeded target. Homing has its own tests.
    kwargs.setdefault("home_on_connect", False)
    backend = PikaSerialGripperBackend(
        ports={"left": "/dev/ttyFAKE0", "right": "/dev/ttyFAKE1"},
        gripper_cls=FakePikaGripper,
        clock=lambda: clock["now"],
        **kwargs,
    ).connect()
    backend._test_clock = clock  # type: ignore[attr-defined]
    return backend


class GripperRuntimeTest(unittest.TestCase):
    def test_real_gripper_env_absent_rejects_command(self) -> None:
        runtime = GripperRuntime(
            rollout_mode="real_policy",
            allow_real_gripper_motion=True,
            env={},
        )

        results = runtime.dispatch([GripperCommand("left", 0.25, command_type="target")])

        self.assertEqual(len(results), 1)
        self.assertTrue(results[0].dropped)
        self.assertFalse(results[0].sent_to_physical)
        self.assertEqual(results[0].reason, "real_gripper_env_missing")
        self.assertEqual(runtime.command_count, 1)
        self.assertEqual(runtime.dropped_count, 1)

    def test_config_allow_real_gripper_motion_false_rejects_even_with_env(self) -> None:
        cfg = config_from_mapping(
            {
                "schema": "robotics_lab.policy_runner.v1",
                "safety": {"allow_real_gripper_motion": False},
            }
        )
        runtime = GripperRuntime(
            rollout_mode="real_policy",
            allow_real_gripper_motion=cfg.safety.allow_real_gripper_motion,
            env={REAL_GRIPPER_ENV: "1"},
        )

        results = runtime.dispatch([GripperCommand("right", -0.1)])

        self.assertEqual(len(results), 1)
        self.assertTrue(results[0].dropped)
        self.assertFalse(results[0].sent_to_physical)
        self.assertEqual(results[0].reason, "real_gripper_config_not_allowed")

    def test_controller_sim_logs_and_drops_gripper_without_physical_command(self) -> None:
        backend = NoopGripperBackend()
        runtime = GripperRuntime(
            rollout_mode="controller_sim",
            allow_real_gripper_motion=False,
            backend=backend,
            env={REAL_GRIPPER_ENV: "1"},
        )

        results = runtime.dispatch([GripperCommand("left", 0.4)])

        self.assertEqual(len(results), 1)
        self.assertTrue(results[0].dropped)
        self.assertFalse(results[0].sent_to_physical)
        self.assertEqual(results[0].reason, "controller_sim_gripper_logged_noop")
        self.assertEqual(backend.commands, [])
        self.assertEqual(runtime.command_count, 1)
        self.assertEqual(runtime.dropped_count, 1)

    def test_noop_backend_records_dry_run_command_without_physical_send(self) -> None:
        backend = NoopGripperBackend()
        command = GripperCommand("right", 1.0, command_type="target")

        result = backend.send(command)

        self.assertTrue(result.dropped)
        self.assertFalse(result.sent_to_physical)
        self.assertEqual(backend.commands, [command])

    def test_flow_step_maps_gripper_channels_separately_from_cartesian(self) -> None:
        step = [0.0] * 14
        step[6] = 0.2
        step[13] = -0.3

        commands = gripper_commands_from_flow_step(step, arm_mask=[1.0, 1.0])

        self.assertEqual([command.arm for command in commands], ["left", "right"])
        self.assertEqual([command.value for command in commands], [0.2, -0.3])
        self.assertTrue(all(command.command_type == "delta" for command in commands))

    def test_zero_delta_skipped_but_zero_target_reaches_backend(self) -> None:
        # Idle-suppression is DELTA-only: a ~zero delta = no change, skip it.
        # An absolute "target" of 0 = fully closed, a real goal, must be sent.
        env = {REAL_GRIPPER_ENV: "1"}
        backend = NoopGripperBackend()
        runtime = GripperRuntime(
            rollout_mode="real_policy",
            allow_real_gripper_motion=True,
            backend=backend,
            env=env,
        )

        runtime.dispatch([GripperCommand("left", 0.0, command_type="delta")])
        self.assertEqual(backend.commands, [])  # zero delta dropped before backend

        target = GripperCommand("right", 0.0, command_type="target")
        runtime.dispatch([target])
        self.assertEqual(backend.commands, [target])  # zero target still sent

    def test_cartesian_intent_can_carry_gripper_targets(self) -> None:
        intent = tcp_pose_target_stand_intent(
            left=[0.1, 0.0, 0.4, 0.0, 0.0, 0.0],
            right=None,
            left_gripper=0.42,
            right_gripper=0.15,
        )

        self.assertEqual(intent.left["mode"], "TcpPoseTarget")
        self.assertEqual(intent.left["gripper_target"], 0.42)
        self.assertEqual(intent.right["mode"], "Hold")
        self.assertEqual(intent.right["gripper_target"], 0.15)

    def test_gripper_only_intent_is_hold_not_motion(self) -> None:
        intent = CommandIntent.gripper_target(left=100.0, right=10.0)

        self.assertEqual(intent.mode, "Hold")
        self.assertFalse(intent.is_motion)
        self.assertEqual(intent.left["mode"], "Hold")
        self.assertEqual(intent.left["gripper_target"], 100.0)
        self.assertEqual(intent.right["mode"], "Hold")
        self.assertEqual(intent.right["gripper_target"], 10.0)


class PikaSerialGripperBackendTest(unittest.TestCase):
    # Command values are dataset PERCENT units over [min_rad, max_rad]:
    # delta_rad = pct/100 * (1.75 - 0.0).
    def test_delta_integrates_from_seeded_motor_position(self) -> None:
        backend = _pika_backend()

        result = backend.send(GripperCommand("left", 20.0))

        self.assertTrue(result.sent_to_physical)
        self.assertFalse(result.dropped)
        self.assertEqual(result.reason, "gripper_position_sent")
        # Seeded at 0.5 rad (FakePikaGripper.position) + 20% of 1.75.
        angles = backend._grippers["left"].sent_angles
        self.assertEqual(len(angles), 1)
        self.assertAlmostEqual(angles[0], 0.5 + 0.35)

    def test_target_command_is_absolute_percent_and_clamped(self) -> None:
        backend = _pika_backend(min_rad=0.0, max_rad=1.75)

        backend.send(GripperCommand("right", 200.0, command_type="target"))
        backend._test_clock["now"] = 1.0
        backend.send(GripperCommand("right", 50.0, command_type="target"))

        angles = backend._grippers["right"].sent_angles
        self.assertEqual(len(angles), 2)
        self.assertAlmostEqual(angles[0], 1.75)   # 200% clamps to max_rad
        self.assertAlmostEqual(angles[1], 0.875)  # 50% of range

    def test_delta_clamps_at_range_and_does_not_wind_up(self) -> None:
        backend = _pika_backend(min_rad=0.0, max_rad=1.75, deadband_rad=0.0)
        backend._test_clock["now"] = 1.0
        backend.send(GripperCommand("left", 1000.0))
        backend._test_clock["now"] = 2.0

        backend.send(GripperCommand("left", -50.0))

        # Without wind-up the second delta acts on the clamped 1.75 rad.
        angles = backend._grippers["left"].sent_angles
        self.assertEqual(len(angles), 2)
        self.assertAlmostEqual(angles[0], 1.75)
        self.assertAlmostEqual(angles[1], 1.75 - 0.875)

    def test_rate_limit_holds_serial_write_but_keeps_integrated_target(self) -> None:
        backend = _pika_backend(max_hz=10.0, deadband_rad=0.0)
        backend.send(GripperCommand("left", 20.0))

        held = backend.send(GripperCommand("left", 20.0))

        self.assertFalse(held.sent_to_physical)
        self.assertFalse(held.dropped)
        self.assertEqual(held.reason, "gripper_rate_limited")
        backend._test_clock["now"] = 0.2
        backend.send(GripperCommand("left", 0.0))
        # Both deltas accumulated into the next write: 0.5 + 0.35 + 0.35.
        angles = backend._grippers["left"].sent_angles
        self.assertEqual(len(angles), 2)
        self.assertAlmostEqual(angles[0], 0.85)
        self.assertAlmostEqual(angles[1], 1.2)

    def test_deadband_skips_small_changes(self) -> None:
        backend = _pika_backend(deadband_rad=0.01, max_hz=0.0)
        backend.send(GripperCommand("left", 20.0))

        held = backend.send(GripperCommand("left", 0.01))

        self.assertFalse(held.sent_to_physical)
        self.assertEqual(held.reason, "gripper_deadband_hold")
        self.assertEqual(len(backend._grippers["left"].sent_angles), 1)

    def test_current_percent_reads_live_motor(self) -> None:
        backend = _pika_backend(min_rad=0.0, max_rad=1.75)
        backend._grippers["left"].position = 0.875

        self.assertAlmostEqual(backend.current_percent("left"), 50.0)
        self.assertIsNone(backend.current_percent("missing"))

    def test_serial_error_reports_dropped_without_raising(self) -> None:
        backend = _pika_backend()

        def boom(_rad: float) -> bool:
            raise OSError("serial gone")

        backend._grippers["left"].set_motor_angle = boom

        result = backend.send(GripperCommand("left", 0.1))

        self.assertTrue(result.dropped)
        self.assertFalse(result.sent_to_physical)
        self.assertIn("gripper_serial_error", result.reason)

    def test_close_disables_and_disconnects(self) -> None:
        backend = _pika_backend()
        left = backend._grippers["left"]

        backend.close()

        self.assertEqual(left.closed_calls, ["disable", "disconnect"])
        self.assertEqual(backend._grippers, {})

    def test_controller_sim_dispatch_honors_actuation_flag(self) -> None:
        allowed = _pika_backend(supports_controller_simulation=True)
        runtime = GripperRuntime(rollout_mode="controller_sim", backend=allowed)
        results = runtime.dispatch([GripperCommand("left", 0.2)])
        self.assertTrue(results[0].sent_to_physical)

        blocked = _pika_backend(supports_controller_simulation=False)
        runtime = GripperRuntime(rollout_mode="controller_sim", backend=blocked)
        results = runtime.dispatch([GripperCommand("left", 0.2)])
        self.assertTrue(results[0].dropped)
        self.assertEqual(results[0].reason, "controller_sim_gripper_logged_noop")
        self.assertEqual(blocked._grippers["left"].sent_angles, [])

    def test_missing_real_serial_port_fails_before_sdk_import(self) -> None:
        backend = PikaSerialGripperBackend(
            ports={"left": "/tmp/robotics_lab_missing_gripper_port"},
            sdk_path="/tmp/robotics_lab_missing_pika_sdk",
            home_on_connect=False,
        )

        with self.assertRaisesRegex(RuntimeError, "serial port not found"):
            backend.connect()


class PikaGripperHomingTest(unittest.TestCase):
    # home_poll_sec=0.0 keeps the settle loop fast; the fake reports a constant
    # position so it settles on the second poll regardless.
    def test_homing_closes_to_stop_and_zeroes_both_arms(self) -> None:
        backend = _pika_backend(
            min_rad=0.0, max_rad=1.75, home_on_connect=True, home_poll_sec=0.0
        )
        for arm in ("left", "right"):
            g = backend._grippers[arm]
            # Drove to the closed stop (min_rad) ...
            self.assertEqual(g.sent_angles, [0.0])
            # ... then re-zeroed there, and the target tracks the closed stop.
            self.assertEqual(g.zero_calls, 1)
            self.assertAlmostEqual(backend._targets[arm], 0.0)

    def test_homing_makes_a_full_open_command_reach_max_rad(self) -> None:
        # After homing, a 100% target maps to true max_rad on each identical arm.
        backend = _pika_backend(
            min_rad=0.0, max_rad=2.3738, home_on_connect=True, home_poll_sec=0.0
        )
        backend.send(GripperCommand("right", 100.0, command_type="target"))
        # Homing write (0.0) then the open write (max_rad).
        self.assertEqual(backend._grippers["right"].sent_angles, [0.0, 2.3738])

    def test_homing_failure_is_nonfatal(self) -> None:
        backend = PikaSerialGripperBackend(
            ports={"left": "/dev/ttyFAKE0"},
            gripper_cls=FakePikaGripper,
            home_on_connect=True,
            home_poll_sec=0.0,
        )

        def boom(_rad: float) -> bool:
            raise OSError("serial gone")

        backend._gripper_cls = type(
            "BoomGripper", (FakePikaGripper,), {"set_motor_angle": lambda self, rad: boom(rad)}
        )
        # connect() must still succeed even if homing raises.
        backend.connect()
        self.assertIn("left", backend._grippers)

    def test_sim_dryrun_never_reaches_backend(self) -> None:
        backend = _pika_backend(supports_controller_simulation=True)
        runtime = GripperRuntime(rollout_mode="sim_dryrun", backend=backend)

        results = runtime.dispatch([GripperCommand("left", 0.3)])

        self.assertTrue(results[0].dropped)
        self.assertEqual(results[0].reason, "sim_dryrun_gripper_logged_noop")
        self.assertEqual(backend._grippers["left"].sent_angles, [])

    def test_pika_sdk_logging_is_suppressed(self) -> None:
        logger = logging.getLogger("pika.serial_comm")
        old_disabled = logger.disabled
        old_level = logger.level
        old_propagate = logger.propagate
        old_handlers = list(logger.handlers)
        stream = io.StringIO()
        handler = logging.StreamHandler(stream)
        logger.handlers.clear()
        logger.addHandler(handler)
        logger.disabled = False
        logger.propagate = False
        logger.setLevel(logging.INFO)

        def restore_logger() -> None:
            logger.handlers.clear()
            logger.handlers.extend(old_handlers)
            logger.disabled = old_disabled
            logger.propagate = old_propagate
            logger.setLevel(old_level)

        self.addCleanup(restore_logger)

        suppress_pika_sdk_logging()
        logger.error("JSON解析错误")

        self.assertEqual(stream.getvalue(), "")
        self.assertTrue(logger.disabled)
        self.assertFalse(logger.propagate)


class GripperConfigTest(unittest.TestCase):
    def test_defaults_to_fail_closed_none_backend(self) -> None:
        cfg = config_from_mapping({"schema": "robotics_lab.policy_runner.v1"})
        self.assertEqual(cfg.gripper.backend, "none")
        self.assertEqual(cfg.gripper.left_port, "/dev/pika-left")
        self.assertEqual(cfg.gripper.right_port, "/dev/pika-right")
        self.assertTrue(cfg.gripper.suppress_sdk_logs)
        self.assertFalse(cfg.gripper.actuate_in_controller_simulation)

    def test_pika_serial_section_parses(self) -> None:
        cfg = config_from_mapping(
            {
                "schema": "robotics_lab.policy_runner.v1",
                "gripper": {
                    "backend": "pika_serial",
                    "left_port": "/dev/serial/by-path/left",
                    "right_port": "/dev/serial/by-path/right",
                    "pika_sdk_path": "/home/plaif/workspace/pika_sdk",
                    "max_rad": 1.75,
                    "suppress_sdk_logs": False,
                    "actuate_in_controller_simulation": True,
                },
            }
        )
        self.assertEqual(cfg.gripper.backend, "pika_serial")
        self.assertEqual(cfg.gripper.left_port, "/dev/serial/by-path/left")
        self.assertFalse(cfg.gripper.suppress_sdk_logs)
        self.assertTrue(cfg.gripper.actuate_in_controller_simulation)

    def test_rejects_unknown_backend(self) -> None:
        with self.assertRaises(ValueError):
            config_from_mapping(
                {
                    "schema": "robotics_lab.policy_runner.v1",
                    "gripper": {"backend": "robotiq"},
                }
            )


if __name__ == "__main__":
    unittest.main()
