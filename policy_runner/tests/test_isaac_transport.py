import pathlib
import tempfile
import threading
import unittest
from unittest.mock import patch

from policy_runner.isaac_transport import (
    BridgeStateClient, PipeDatagram, ServoBridge, SimulationClock,
    TimedInferenceClient,
)
from policy_runner.servo_command_client import CommandIntent, ServoCommandClient


ROOT = pathlib.Path(__file__).resolve().parents[2]
EXE = ROOT / "rb_servo_server/build/rb_isaac_bridge"
# Explicit measured-pose fixture for this RB5 cell. No hardware is queried.
MEASURED = {
    "left": {"q_deg": [-85.721, 36.301, 125.914, -9.832, -123.706, 33.556],
             "dq_deg_s": [0]*6, "gripper_percent": 100},
    "right": {"q_deg": [86.320, -28.371, -125.802, -1.274, 123.360, -42.350],
              "dq_deg_s": [0]*6, "gripper_percent": 100},
}


class ClockTests(unittest.TestCase):
    def test_exact_tick_without_rounding_drift(self):
        clock = SimulationClock()
        for _ in range(500):
            clock.advance()
        self.assertEqual(clock.now_ns(), 2_000_000_000)

    def test_service_latency_cannot_disappear_in_slow_rendering(self):
        clock = SimulationClock()
        queried = threading.Event()
        class FixtureClient:
            def infer(self, obs):
                queried.set()
                return {"actions": [1]}
        client = TimedInferenceClient(FixtureClient(), clock)
        results = []
        with patch("policy_runner.isaac_transport.time.perf_counter_ns", side_effect=[0, 10_000_000]):
            worker = threading.Thread(target=lambda: results.append(client.infer({})))
            worker.start()
            self.assertTrue(queried.wait(1))
            for _ in range(4):
                clock.advance()
            worker.join(0.03)
            self.assertTrue(worker.is_alive())
            clock.advance()
            worker.join(1)
        self.assertFalse(worker.is_alive())
        self.assertEqual(results, [{"actions": [1]}])
        self.assertEqual(client.calls[0]["delivered_ns"] - client.calls[0]["request_ns"], 10_000_000)


@unittest.skipUnless(EXE.exists(), "build rb_isaac_bridge for C++ integration tests")
class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.clock = SimulationClock()
        self.bridge = ServoBridge(EXE, ROOT/"rb_servo_server/config/stack_real.yaml",
                                  MEASURED, cwd=ROOT, log_path=pathlib.Path(self.tmp.name)/"servo.log")
        self.client = ServoCommandClient("udp://127.0.0.1:1", source_id="policy_runner",
                                        session_id="isaac-test", socket_factory=lambda *a: PipeDatagram(self.bridge, "command"))

    def tearDown(self):
        self.bridge.close()
        self.tmp.cleanup()

    def tick(self):
        self.clock.advance()
        return self.bridge.step(self.clock, MEASURED)

    def test_only_physics_step_advances_state_and_lease_gate_is_preserved(self):
        state = BridgeStateClient(self.bridge, self.clock)
        self.assertEqual(state.latest.received_monotonic, 1.0)
        with self.assertRaisesRegex(RuntimeError, "C\\+\\+ rejected command"):
            self.client.send(CommandIntent.joint_target(left=MEASURED["left"]["q_deg"],
                                                         right=MEASURED["right"]["q_deg"]))
        self.assertEqual(self.bridge.reply["seq"], 1)
        self.client.acquire_lease()
        self.tick()
        lease = self.bridge.reply["state"]["command_source"]
        self.assertTrue(lease["active"])
        self.client.lease_token = lease["active_lease_token"]
        self.client.send(CommandIntent("ArmMotion"))
        self.tick()
        self.assertFalse(self.bridge.reply["state"]["fault_latched"])
        self.assertEqual(self.bridge.reply["state"]["filter_dt_ms"], 2.0)
        self.assertEqual(self.bridge.reply["time_ns"], 1_004_000_000)
        self.client.release_lease()
        self.tick()
        self.assertFalse(self.bridge.reply["state"]["command_source"]["active"])

    def test_duplicate_or_skipped_physics_step_fails_closed(self):
        with self.assertRaisesRegex(RuntimeError, "exactly one 2 ms"):
            self.bridge.rpc({"op": "step", "seq": 2, "time_ns": 1_004_000_000, **MEASURED})
        self.bridge.process.wait(timeout=5)
        self.assertEqual(self.bridge.process.returncode, 1)

    def test_malformed_measurement_fails_closed(self):
        bad = {**MEASURED, "left": {**MEASURED["left"], "q_deg": [0]*5}}
        with self.assertRaisesRegex(RuntimeError, "six joints"):
            self.clock.advance()
            self.bridge.step(self.clock, bad)
        self.bridge.process.wait(timeout=5)
        self.assertEqual(self.bridge.process.returncode, 1)

    def test_invalid_chunk_rejected_without_consuming_a_tick(self):
        with self.assertRaises(RuntimeError):
            self.bridge.rpc({"op": "chunk", "packet": {"schema_version": "bad"}})
        self.assertEqual(self.bridge.reply["seq"], 1)
        self.assertFalse(self.tick()["state"]["fault_latched"])

    def test_emergency_stop_holds_plant_and_explicit_reset_recovers(self):
        self.client.acquire_lease()
        self.tick()
        self.client.lease_token = self.bridge.reply["state"]["command_source"]["active_lease_token"]
        self.client.send(CommandIntent("EmergencyStop"))
        stopped = self.tick()
        self.assertTrue(stopped["state"]["fault_latched"])
        for side in ("left", "right"):
            self.assertEqual(stopped[side]["q_target_deg"], MEASURED[side]["q_deg"])
        self.client.send(CommandIntent("ResetFault"))
        reset = self.tick()
        self.assertFalse(reset["state"]["fault_latched"])
        self.client.acquire_lease()
        self.tick()
        self.client.lease_token = self.bridge.reply["state"]["command_source"]["active_lease_token"]
        self.client.send(CommandIntent("ArmMotion"))
        self.assertFalse(self.tick()["state"]["fault_latched"])


@unittest.skipUnless(EXE.exists(), "build rb_isaac_bridge for C++ integration tests")
class ForceBridgeTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.clock = SimulationClock()
        self.bridge = ServoBridge(EXE, ROOT/'rb_servo_server/config/stack_real.yaml',
                                  {**self.measurement(), 'force_sensor_enabled': True},
                                  cwd=ROOT, log_path=pathlib.Path(self.tmp.name)/'servo.log')

    def measurement(self):
        return {side: {**value, 'force_sensor': {
            'valid': True, 'seq': 1+(self.clock.now_ns()-1_000_000_000)//2_000_000,
            'time_ns': self.clock.now_ns(), 'frame': 'flange_at_flange',
            'wrench': [1,2,3,0.1,0.2,0.3]}} for side,value in MEASURED.items()}

    def tearDown(self):
        self.bridge.close()
        self.tmp.cleanup()

    def test_full_wrench_shift_and_electrical_axis_map_without_implicit_tare(self):
        ft = self.bridge.reply['state']['left']['force_torque']
        self.assertTrue(ft['connected'])
        self.assertFalse(ft['bias_valid'])
        # 45 mm SRO offset: M_sro = M_flange - r_sro x F.
        for actual,expected in zip(ft['raw_sensor_axes_at_sro'], [1,2,3,0.19,0.155,0.3]):
            self.assertAlmostEqual(actual, expected, places=8)
        self.assertFalse(self.bridge.reply['state']['left']['force_control']['covered'])

    def test_stale_force_sample_is_rejected_even_with_fresh_joint_state(self):
        measurement = self.measurement()
        self.clock.advance()
        with self.assertRaisesRegex(RuntimeError, 'invalid/stale PhysX force'):
            self.bridge.step(self.clock, measurement)

    def test_missing_force_sample_cannot_silently_disable_feedback(self):
        self.clock.advance()
        with self.assertRaises(RuntimeError):
            self.bridge.step(self.clock, MEASURED)

    def test_wrong_frame_is_rejected(self):
        self.clock.advance()
        measurement = self.measurement()
        measurement['left']['force_sensor']['frame'] = 'tcp_at_tcp'
        with self.assertRaisesRegex(RuntimeError, 'invalid/stale PhysX force'):
            self.bridge.step(self.clock, measurement)


if __name__ == "__main__":
    unittest.main()
