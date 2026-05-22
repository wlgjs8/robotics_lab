import json
import sys
import unittest
from pathlib import Path
from typing import Any
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from rbsim import PROTOCOL_VERSION, DualArmSimulator, SimulatorProtocol, load_simulator_config


CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "dual_rb3_730e.yaml"


class SimulatorProtocolContractTest(unittest.TestCase):
    def setUp(self) -> None:
        self.protocol = SimulatorProtocol(DualArmSimulator(load_simulator_config(CONFIG_PATH)))

    def request(self, endpoint_role: str, payload: dict[str, Any]) -> dict[str, Any]:
        response = self.protocol.handle_json_line(json.dumps(payload, separators=(",", ":")), endpoint_role)
        decoded = json.loads(response)
        self.assertEqual(decoded["schema_version"], PROTOCOL_VERSION)
        self.assertIn("server_time_ns", decoded)
        return decoded

    def control(self, op: str, arm: str = "left", params: dict[str, Any] | None = None) -> dict[str, Any]:
        return self.request(
            "control",
            {
                "schema_version": PROTOCOL_VERSION,
                "request_id": f"req-{op}",
                "op": op,
                "arm": arm,
                "params": params or {},
            },
        )

    def admin(self, op: str, arm: str | None = "left", params: dict[str, Any] | None = None) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "schema_version": PROTOCOL_VERSION,
            "request_id": f"req-{op}",
            "op": op,
            "params": params or {},
        }
        if arm is not None:
            payload["arm"] = arm
        return self.request("admin", payload)

    def test_control_operations_return_machine_testable_states(self) -> None:
        self.assertEqual(self.admin("admin.disconnect")["state"]["lifecycle_state"], "disconnected")

        connected = self.control("connect")
        self.assertTrue(connected["ok"])
        self.assertEqual(connected["state"]["lifecycle_state"], "connected")

        initialized = self.control("initialize")
        self.assertTrue(initialized["state"]["servo_enabled"])

        state = self.control("read_state")
        self.assertEqual(state["state"]["connection_state"], "Connected")

        sent = self.control("send_servo_j", params={"q_target_deg": [4, -30, 80, 0, 60, 0]})
        self.assertEqual(sent["state"]["q_target_deg"][0], 4.0)

        stopped = self.control("stop")
        self.assertEqual(stopped["state"]["lifecycle_state"], "stopped")

        reset = self.control("reset_fault")
        self.assertEqual(reset["state"]["lifecycle_state"], "stopped")
        self.assertFalse(reset["state"]["has_error"])

    def test_admin_fault_injection_operations_are_machine_testable(self) -> None:
        self.assertTrue(self.control("initialize")["ok"])

        for hook, op, code, name in (
            ("read_failure", "read_state", 2104, "read_failure_injected"),
            ("send_failure", "send_servo_j", 2101, "send_failure_injected"),
            ("stop_failure", "stop", 2102, "stop_failure_injected"),
            ("reset_failure", "reset_fault", 2103, "reset_failure_injected"),
        ):
            injected = self.admin("admin.inject", params={"hook": hook, "enabled": True})
            self.assertTrue(injected["admin"]["fault_hooks"][hook])

            params = {"q_target_deg": [1, -30, 80, 0, 60, 0]} if op == "send_servo_j" else None
            failed = self.control(op, params=params)
            self.assertFalse(failed["ok"])
            self.assertEqual(failed["error"]["name"], name)
            self.assertEqual(failed["error"]["code"], code)

            self.assertTrue(self.admin("admin.inject", params={"hook": hook, "enabled": False})["ok"])

    def test_injected_read_failure_is_per_arm_isolated(self) -> None:
        self.assertTrue(self.control("initialize", arm="left")["ok"])
        self.assertTrue(self.control("initialize", arm="right")["ok"])

        injected = self.admin("admin.inject", arm="left", params={"hook": "read_failure", "enabled": True})
        self.assertTrue(injected["admin"]["fault_hooks"]["read_failure"])

        failed_left = self.control("read_state", arm="left")
        self.assertFalse(failed_left["ok"])
        self.assertEqual(failed_left["error"]["name"], "read_failure_injected")
        self.assertEqual(failed_left["error"]["code"], 2104)

        right = self.control("read_state", arm="right")
        self.assertTrue(right["ok"])
        self.assertEqual(right["state"]["arm"], "right")

    def test_send_failure_preserves_last_safe_target_without_zero_substitution(self) -> None:
        self.assertTrue(self.control("initialize")["ok"])
        safe_target = [3, -30, 80, 0, 60, 0]
        self.assertTrue(self.control("send_servo_j", params={"q_target_deg": safe_target})["ok"])

        self.assertTrue(self.admin("admin.inject", params={"hook": "send_failure", "enabled": True})["ok"])
        failed = self.control("send_servo_j", params={"q_target_deg": [9, -30, 80, 0, 60, 0]})
        self.assertFalse(failed["ok"])

        state = self.control("read_state")["state"]
        self.assertEqual(state["q_target_deg"], [float(value) for value in safe_target])
        self.assertNotEqual(state["q_target_deg"], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def test_admin_state_faults_bias_freeze_and_latency(self) -> None:
        self.assertEqual(self.admin("admin.disconnect")["state"]["lifecycle_state"], "disconnected")
        disconnected = self.control("initialize")
        self.assertFalse(disconnected["ok"])
        self.assertEqual(disconnected["error"]["code"], 1001)

        self.assertTrue(self.admin("admin.reconnect")["ok"])
        invalid = self.admin("admin.set_joint_validity", params={"valid": False})
        self.assertFalse(invalid["state"]["has_valid_joint_state"])
        self.assertEqual(self.control("initialize")["error"]["code"], 2002)

        self.assertTrue(self.admin("admin.set_joint_validity", params={"valid": True})["ok"])
        faulted = self.admin("admin.set_fault", params={"error_code": 4321})
        self.assertEqual(faulted["state"]["error_code"], 4321)
        self.assertEqual(self.control("initialize")["error"]["code"], 4321)

        self.assertTrue(self.control("reset_fault")["ok"])
        self.assertTrue(self.control("initialize")["ok"])
        biased = self.admin("admin.set_tracking_bias", params={"bias_deg": [1, 0, 0, 0, 0, 0]})
        self.assertEqual(biased["state"]["q_actual_deg"][0], 1.0)

        self.assertTrue(self.control("send_servo_j", params={"q_target_deg": [8, -30, 80, 0, 60, 0]})["ok"])
        self.assertTrue(self.admin("admin.freeze_motion", params={"enabled": True})["ok"])
        frozen = self.admin("admin.tick", arm=None, params={"steps": 3})["states"]["left"]
        self.assertEqual(frozen["q_actual_deg"][0], 1.0)
        self.assertEqual(frozen["dq_actual_deg_s"], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.admin("admin.set_latency", params={"latency_ms": 12})
        with patch("rbsim.protocol.time.sleep") as sleep:
            self.control("read_state")
        sleep.assert_called_once_with(0.012)

    def test_protocol_exposes_stale_state_and_unrecoverable_fault_semantics(self) -> None:
        self.assertTrue(self.control("initialize")["ok"])
        before_stale = self.admin("admin.tick", arm=None, params={"steps": 1})["states"]["left"]

        stale = self.admin("admin.set_stale_state", params={"enabled": True})
        self.assertTrue(stale["state"]["stale_state"])
        frozen = self.admin("admin.tick", arm=None, params={"steps": 5})["states"]["left"]
        self.assertEqual(frozen["robot_time_ns"], before_stale["robot_time_ns"])

        self.assertTrue(self.admin("admin.set_stale_state", params={"enabled": False})["ok"])
        recovered_clock = self.admin("admin.tick", arm=None, params={"steps": 1})["states"]["left"]
        self.assertGreater(recovered_clock["robot_time_ns"], frozen["robot_time_ns"])

        faulted = self.admin("admin.set_fault", params={"error_code": 9001, "recoverable": False})
        self.assertTrue(faulted["state"]["has_error"])
        self.assertFalse(faulted["state"]["fault_recoverable"])

        reset = self.control("reset_fault")
        self.assertFalse(reset["ok"])
        self.assertEqual(reset["error"]["name"], "fault_latched")
        self.assertEqual(reset["error"]["code"], 9001)

        still_faulted = self.control("read_state")
        self.assertTrue(still_faulted["state"]["has_error"])
        self.assertFalse(still_faulted["state"]["fault_recoverable"])

    def test_bad_joint_payload_returns_json_error_response(self) -> None:
        self.assertTrue(self.control("initialize")["ok"])

        missing = self.control("send_servo_j")
        self.assertFalse(missing["ok"])
        self.assertEqual(missing["error"]["name"], "bad_request")
        self.assertEqual(missing["error"]["code"], 4000)


if __name__ == "__main__":
    unittest.main()
