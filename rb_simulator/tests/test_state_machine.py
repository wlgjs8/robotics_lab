import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from rbsim import ArmSimulator, SimulatorError, load_simulator_config


CONFIG_LEFT = Path(__file__).resolve().parents[1] / "config" / "left_rb3_730e.yaml"
CONFIG_RIGHT = Path(__file__).resolve().parents[1] / "config" / "right_rb3_730e.yaml"


class SimulatorStateMachineTest(unittest.TestCase):
    def make_simulator(self, config_path: Path = CONFIG_LEFT) -> ArmSimulator:
        return ArmSimulator(load_simulator_config(config_path))

    def test_loads_per_arm_configs(self) -> None:
        config = load_simulator_config(CONFIG_LEFT)

        self.assertEqual(config.arm, "left")
        self.assertEqual(config.control_bind, "tcp://127.0.0.1:50200")
        self.assertEqual(config.admin_bind, "tcp://127.0.0.1:50201")
        self.assertEqual(config.update_rate_hz, 200)
        self.assertEqual(config.max_joint_velocity_deg_s, 360)
        self.assertEqual(config.model, "RB3_730E")
        self.assertEqual(config.arm_config.name, "left_simulator")
        self.assertEqual(len(config.arm_config.initial_q_deg), 6)

        right = load_simulator_config(CONFIG_RIGHT)
        self.assertEqual(right.arm, "right")
        self.assertEqual(right.control_bind, "tcp://127.0.0.1:50210")
        self.assertEqual(right.admin_bind, "tcp://127.0.0.1:50211")
        self.assertEqual(right.arm_config.name, "right_simulator")

    def test_lifecycle_transitions_are_explicit(self) -> None:
        sim = self.make_simulator()

        self.assertEqual(sim.snapshot("left").lifecycle_state, "connected")
        self.assertEqual(sim.initialize("left").lifecycle_state, "initialized")
        self.assertEqual(sim.enable_servo("left").lifecycle_state, "servo_enabled")
        self.assertEqual(sim.stop("left").lifecycle_state, "stopped")
        self.assertEqual(sim.set_fault("left", error_code=4321).lifecycle_state, "faulted")

        reset = sim.reset_fault("left")
        self.assertEqual(reset.lifecycle_state, "stopped")
        self.assertFalse(reset.initialized)
        self.assertFalse(reset.servo_enabled)
        self.assertEqual(reset.error_code, 0)

    def test_motion_advances_toward_target_deterministically(self) -> None:
        sim = self.make_simulator()
        sim.initialize("left")
        sim.enable_servo("left")
        sim.send_servo_j("left", [10, -20, 70, 5, 55, 3])

        after_one = sim.tick(1)["left"]
        self.assertEqual(after_one.robot_time_ns, 5_000_000)
        self.assertAlmostEqual(after_one.q_actual_deg[0], 1.25)
        self.assertAlmostEqual(after_one.q_actual_deg[1], -28.75)
        self.assertAlmostEqual(after_one.dq_actual_deg_s[0], 250.0)

        after_two = sim.tick(1)["left"]
        self.assertAlmostEqual(after_two.q_actual_deg[0], 2.34375)
        self.assertLess(after_two.q_actual_deg[0], after_two.q_target_deg[0])
        self.assertLess(after_two.q_actual_deg[1], after_two.q_target_deg[1])

    def test_motion_velocity_limit_caps_large_target_steps(self) -> None:
        sim = self.make_simulator()
        sim.initialize("left")
        sim.enable_servo("left")
        sim.send_servo_j("left", [100, -30, 80, 0, 60, 0])

        after_one = sim.tick(1)["left"]
        self.assertAlmostEqual(after_one.q_actual_deg[0], 1.8)
        self.assertAlmostEqual(after_one.dq_actual_deg_s[0], 360.0)

    def test_one_process_owns_exactly_one_arm(self) -> None:
        left = self.make_simulator(CONFIG_LEFT)
        right = self.make_simulator(CONFIG_RIGHT)

        left.initialize("left")
        left.enable_servo("left")
        right.initialize("right")
        right.enable_servo("right")

        left.send_servo_j("left", [6, -30, 80, 0, 60, 0])
        right.send_servo_j("right", [-6, -30, 80, 0, 60, 0])
        left_snapshot = left.tick(4)["left"]
        right_snapshot = right.tick(4)["right"]

        self.assertEqual(set(left.snapshots()), {"left"})
        self.assertEqual(set(right.snapshots()), {"right"})
        self.assertGreater(left_snapshot.q_actual_deg[0], 0)
        self.assertLess(right_snapshot.q_actual_deg[0], 0)
        self.assertEqual(len(left_snapshot.q_actual_deg), 6)
        self.assertEqual(len(right_snapshot.q_actual_deg), 6)

    def test_wrong_arm_requests_fail_closed(self) -> None:
        left = self.make_simulator(CONFIG_LEFT)
        right = self.make_simulator(CONFIG_RIGHT)

        with self.assertRaisesRegex(SimulatorError, "wrong arm"):
            left.snapshot("right")
        with self.assertRaisesRegex(SimulatorError, "wrong arm"):
            right.snapshot("left")

    def test_fault_and_stop_do_not_mutate_to_new_targets(self) -> None:
        sim = self.make_simulator()
        sim.initialize("left")
        sim.enable_servo("left")
        sim.send_servo_j("left", [6, -30, 80, 0, 60, 0])
        moving = sim.tick(1)["left"]

        stopped = sim.stop("left")
        self.assertEqual(stopped.q_target_deg, moving.q_actual_deg)
        with self.assertRaises(SimulatorError):
            sim.send_servo_j("left", [20, -30, 80, 0, 60, 0])
        self.assertEqual(sim.snapshot("left").q_target_deg, moving.q_actual_deg)

        sim.set_fault("left")
        with self.assertRaises(SimulatorError):
            sim.enable_servo("left")
        self.assertEqual(sim.snapshot("left").q_target_deg, moving.q_actual_deg)

    def test_stop_and_fault_hold_last_safe_target_when_state_is_invalid(self) -> None:
        sim = self.make_simulator()
        sim.initialize("left")
        sim.enable_servo("left")
        safe_target = (7.0, -30.0, 80.0, 0.0, 60.0, 0.0)
        sim.send_servo_j("left", safe_target)
        sim.set_joint_validity("left", False)

        stopped = sim.stop("left")
        self.assertEqual(stopped.q_target_deg, safe_target)
        self.assertNotEqual(stopped.q_target_deg, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

        faulted = sim.set_fault("left", error_code=4321)
        self.assertEqual(faulted.q_target_deg, safe_target)
        self.assertNotEqual(faulted.q_target_deg, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

    def test_recoverable_fault_resets_but_unrecoverable_fault_stays_latched(self) -> None:
        sim = self.make_simulator()
        unrecoverable = sim.set_fault("left", error_code=9001, recoverable=False)
        self.assertTrue(unrecoverable.faulted)
        self.assertFalse(unrecoverable.fault_recoverable)

        with self.assertRaises(SimulatorError):
            sim.reset_fault("left")
        self.assertTrue(sim.snapshot("left").faulted)
        self.assertEqual(sim.snapshot("left").error_code, 9001)

        sim.set_fault("left", error_code=2001, recoverable=True)
        reset = sim.reset_fault("left")
        self.assertFalse(reset.faulted)
        self.assertTrue(reset.fault_recoverable)
        self.assertEqual(reset.lifecycle_state, "stopped")

    def test_tracking_bias_and_freeze_are_deterministic(self) -> None:
        sim = self.make_simulator()
        sim.initialize("left")
        sim.enable_servo("left")
        sim.send_servo_j("left", [8, -30, 80, 0, 60, 0])

        biased = sim.set_tracking_bias("left", [1, 0, 0, 0, 0, 0])
        self.assertEqual(biased.q_actual_deg[0], 1)

        sim.set_motion_frozen("left", True)
        frozen = sim.tick(3)["left"]
        self.assertEqual(frozen.q_actual_deg[0], 1)
        self.assertEqual(frozen.dq_actual_deg_s, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

        sim.set_motion_frozen("left", False)
        moving = sim.tick(1)["left"]
        self.assertGreater(moving.q_actual_deg[0], frozen.q_actual_deg[0])

    def test_stale_state_freezes_timestamp_and_reported_motion(self) -> None:
        sim = self.make_simulator()
        sim.initialize("left")
        sim.enable_servo("left")
        sim.send_servo_j("left", [8, -30, 80, 0, 60, 0])
        moving = sim.tick(1)["left"]

        stale = sim.set_stale_state("left", True)
        self.assertTrue(stale.stale_state)
        frozen = sim.tick(5)["left"]
        self.assertEqual(frozen.robot_time_ns, moving.robot_time_ns)
        self.assertEqual(frozen.q_actual_deg, moving.q_actual_deg)
        self.assertEqual(frozen.dq_actual_deg_s, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

        sim.set_stale_state("left", False)
        resumed = sim.tick(1)["left"]
        self.assertGreater(resumed.robot_time_ns, frozen.robot_time_ns)
        self.assertGreater(resumed.q_actual_deg[0], frozen.q_actual_deg[0])

    def test_rejects_invalid_inputs_without_partial_state_change(self) -> None:
        sim = self.make_simulator()
        sim.initialize("left")
        sim.enable_servo("left")
        before = sim.snapshot("left")

        with self.assertRaises(ValueError):
            sim.send_servo_j("left", [1, 2, 3])
        self.assertEqual(sim.snapshot("left").q_target_deg, before.q_target_deg)

        sim.disconnect("left")
        with self.assertRaises(SimulatorError):
            sim.initialize("left")
        self.assertEqual(sim.snapshot("left").lifecycle_state, "disconnected")


if __name__ == "__main__":
    unittest.main()
