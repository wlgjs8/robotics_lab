import json
import socket
import sys
import unittest
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from rbsim import PROTOCOL_VERSION, ArmSimulator, RbsimService, SimulatorProtocol, load_simulator_config
from rbsim.server import parse_tcp_bind


CONFIG_LEFT = Path(__file__).resolve().parents[1] / "config" / "left_rb3_730e.yaml"
CONFIG_RIGHT = Path(__file__).resolve().parents[1] / "config" / "right_rb3_730e.yaml"


class JsonlProtocolServerTest(unittest.TestCase):
    def setUp(self) -> None:
        config = load_simulator_config(CONFIG_LEFT)
        self.protocol = SimulatorProtocol(ArmSimulator(config))

    def request(self, endpoint_role: str, payload: dict[str, Any]) -> dict[str, Any]:
        request = json.dumps(payload, separators=(",", ":")).encode("utf-8") + b"\n"
        response = self.protocol.handle_json_line(request, endpoint_role)
        decoded = json.loads(response)
        self.assertEqual(decoded["schema_version"], PROTOCOL_VERSION)
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

    def test_control_lifecycle_uses_versioned_response_envelope(self) -> None:
        initialized = self.control("initialize")
        self.assertTrue(initialized["ok"])
        self.assertEqual(initialized["arm"], "left")
        self.assertEqual(initialized["state"]["connection_state"], "Connected")
        self.assertTrue(initialized["state"]["servo_enabled"])

        sent = self.control("send_servo_j", params={"q_target_deg": [5, -30, 80, 0, 60, 0]})
        self.assertTrue(sent["ok"])
        self.assertEqual(sent["state"]["q_target_deg"][0], 5.0)

        ticked = self.admin("admin.tick", arm=None, params={"steps": 2})
        self.assertTrue(ticked["ok"])
        self.assertIn("left", ticked["states"])
        self.assertGreater(ticked["states"]["left"]["q_actual_deg"][0], 0.0)

        state = self.control("read_state")
        self.assertEqual(state["state"]["robot_time_ns"], 10_000_000)
        self.assertGreater(state["state"]["dq_actual_deg_s"][0], 0.0)

    def test_admin_fault_hooks_are_machine_testable_and_resettable(self) -> None:
        self.assertTrue(self.control("initialize")["ok"])
        injected = self.admin("admin.inject", params={"hook": "send_failure", "enabled": True})
        self.assertTrue(injected["admin"]["fault_hooks"]["send_failure"])

        failed = self.control("send_servo_j", params={"q_target_deg": [2, -30, 80, 0, 60, 0]})
        self.assertFalse(failed["ok"])
        self.assertEqual(failed["error"]["kind"], "TransportWriteFailed")
        self.assertEqual(failed["error"]["name"], "send_failure_injected")
        self.assertEqual(failed["error"]["code"], 2101)

        self.assertTrue(self.admin("admin.reset_hooks")["ok"])
        sent = self.control("send_servo_j", params={"q_target_deg": [2, -30, 80, 0, 60, 0]})
        self.assertTrue(sent["ok"])

    def test_invalid_state_and_faults_return_explicit_error_codes(self) -> None:
        invalid = self.admin("admin.set_joint_validity", params={"valid": False})
        self.assertFalse(invalid["state"]["has_valid_joint_state"])

        blocked = self.control("initialize")
        self.assertFalse(blocked["ok"])
        self.assertEqual(blocked["error"]["kind"], "InvalidJointState")
        self.assertEqual(blocked["error"]["name"], "invalid_joint_state")
        self.assertEqual(blocked["error"]["code"], 2002)
        self.assertIn("state", blocked)

        self.assertTrue(self.admin("admin.set_joint_validity", params={"valid": True})["ok"])
        faulted = self.admin("admin.set_fault", params={"error_code": 4321})
        self.assertEqual(faulted["state"]["error_code"], 4321)
        fault_response = self.control("initialize")
        self.assertEqual(fault_response["error"]["kind"], "RobotFault")
        self.assertEqual(fault_response["error"]["name"], "fault_latched")
        self.assertEqual(fault_response["error"]["code"], 4321)
        self.assertIn("state", fault_response)

    def test_control_and_admin_endpoints_are_separated(self) -> None:
        admin_on_control = self.request(
            "control",
            {"schema_version": PROTOCOL_VERSION, "request_id": "wrong", "op": "admin.tick", "params": {}},
        )
        self.assertFalse(admin_on_control["ok"])
        self.assertEqual(admin_on_control["error"]["kind"], "WrongEndpoint")
        self.assertEqual(admin_on_control["error"]["name"], "wrong_endpoint")

        control_on_admin = self.request(
            "admin",
            {"schema_version": PROTOCOL_VERSION, "request_id": "wrong", "op": "read_state", "arm": "left"},
        )
        self.assertFalse(control_on_admin["ok"])
        self.assertEqual(control_on_admin["error"]["kind"], "WrongEndpoint")
        self.assertEqual(control_on_admin["error"]["name"], "wrong_endpoint")

    def test_protocol_rejects_schema_mismatch_and_malformed_requests(self) -> None:
        bad_schema = self.request(
            "control",
            {"schema_version": "rbsim.v999", "request_id": "bad-schema", "op": "read_state", "arm": "left"},
        )
        self.assertFalse(bad_schema["ok"])
        self.assertEqual(bad_schema["request_id"], "bad-schema")
        self.assertEqual(bad_schema["error"]["kind"], "UnsupportedSchema")
        self.assertEqual(bad_schema["error"]["name"], "unsupported_schema_version")
        self.assertEqual(bad_schema["error"]["code"], 4001)

        missing_arm = self.request(
            "control",
            {"schema_version": PROTOCOL_VERSION, "request_id": "missing-arm", "op": "read_state"},
        )
        self.assertFalse(missing_arm["ok"])
        self.assertEqual(missing_arm["error"]["kind"], "ProtocolError")
        self.assertEqual(missing_arm["error"]["name"], "bad_request")

        bad_params = self.request(
            "control",
            {
                "schema_version": PROTOCOL_VERSION,
                "request_id": "bad-params",
                "op": "read_state",
                "arm": "left",
                "params": [],
            },
        )
        self.assertFalse(bad_params["ok"])
        self.assertEqual(bad_params["error"]["name"], "bad_request")

        invalid_json = json.loads(self.protocol.handle_json_line(b"{not-json}\n", "control"))
        self.assertFalse(invalid_json["ok"])
        self.assertEqual(invalid_json["error"]["kind"], "ProtocolError")
        self.assertEqual(invalid_json["error"]["name"], "invalid_json")
        self.assertEqual(invalid_json["error"]["code"], 4002)

    def test_loopback_bind_guard_rejects_exposed_addresses(self) -> None:
        self.assertEqual(parse_tcp_bind("tcp://127.0.0.1:50200"), ("127.0.0.1", 50200))
        with self.assertRaises(ValueError):
            parse_tcp_bind("tcp://0.0.0.0:50200")

    def test_non_loopback_bind_requires_environment_gate(self) -> None:
        from unittest.mock import patch

        with patch.dict("os.environ", {"RB_SIMULATOR_ALLOW_NON_LOOPBACK": "1"}):
            self.assertEqual(parse_tcp_bind("tcp://0.0.0.0:50200"), ("0.0.0.0", 50200))

    def test_tcp_service_accepts_json_lines_on_control_and_admin_loopback_ports(self) -> None:
        config = load_simulator_config(CONFIG_LEFT)
        service = RbsimService.with_binds(config, "tcp://127.0.0.1:0", "tcp://127.0.0.1:0")
        try:
            service.start(start_tick_loop=False)
        except PermissionError as exc:
            self.skipTest(f"loopback sockets are unavailable in this environment: {exc}")
        try:
            initialized = self.tcp_request(
                service.control_address,
                {"schema_version": PROTOCOL_VERSION, "request_id": "tcp-init", "op": "initialize", "arm": "left"},
            )
            self.assertTrue(initialized["ok"])
            self.assertEqual(initialized["request_id"], "tcp-init")
            self.assertTrue(initialized["state"]["servo_enabled"])

            ticked = self.tcp_request(
                service.admin_address,
                {
                    "schema_version": PROTOCOL_VERSION,
                    "request_id": "tcp-tick",
                    "op": "admin.tick",
                    "params": {"steps": 1},
                },
            )
            self.assertTrue(ticked["ok"])
            self.assertIn("left", ticked["states"])
            self.assertEqual(ticked["states"]["left"]["robot_time_ns"], 5_000_000)
        finally:
            service.stop()

    def test_tcp_service_waits_for_jsonl_frame_before_responding(self) -> None:
        config = load_simulator_config(CONFIG_LEFT)
        service = RbsimService.with_binds(config, "tcp://127.0.0.1:0", "tcp://127.0.0.1:0")
        try:
            service.start(start_tick_loop=False)
        except PermissionError as exc:
            self.skipTest(f"loopback sockets are unavailable in this environment: {exc}")
        try:
            payload = json.dumps(
                {"schema_version": PROTOCOL_VERSION, "request_id": "partial-frame", "op": "read_state", "arm": "left"},
                separators=(",", ":"),
            ).encode("utf-8")
            with socket.create_connection(service.control_address, timeout=1.0) as sock:
                sock.settimeout(0.05)
                sock.sendall(payload)
                with self.assertRaises(socket.timeout):
                    sock.recv(4096)

                sock.sendall(b"\n")
                sock.settimeout(1.0)
                response = sock.recv(4096)

            decoded = json.loads(response.decode("utf-8"))
            self.assertTrue(decoded["ok"])
            self.assertEqual(decoded["request_id"], "partial-frame")
        finally:
            service.stop()

    def test_tcp_service_keeps_jsonl_connection_after_protocol_robot_errors(self) -> None:
        config = load_simulator_config(CONFIG_LEFT)
        service = RbsimService.with_binds(config, "tcp://127.0.0.1:0", "tcp://127.0.0.1:0")
        try:
            service.start(start_tick_loop=False)
        except PermissionError as exc:
            self.skipTest(f"loopback sockets are unavailable in this environment: {exc}")
        try:
            with socket.create_connection(service.control_address, timeout=1.0) as control_sock:
                with control_sock.makefile("rwb") as control_file:
                    connected = self.tcp_file_request(
                        control_file,
                        {
                            "schema_version": PROTOCOL_VERSION,
                            "request_id": "persist-connect",
                            "op": "connect",
                            "arm": "left",
                        },
                    )
                    self.assertTrue(connected["ok"])

                    initialized_disabled = self.tcp_file_request(
                        control_file,
                        {
                            "schema_version": PROTOCOL_VERSION,
                            "request_id": "persist-init-disabled",
                            "op": "initialize",
                            "arm": "left",
                            "params": {"enable_servo": False},
                        },
                    )
                    self.assertTrue(initialized_disabled["ok"])

                    servo_disabled = self.tcp_file_request(
                        control_file,
                        {
                            "schema_version": PROTOCOL_VERSION,
                            "request_id": "persist-servo-disabled",
                            "op": "send_servo_j",
                            "arm": "left",
                            "params": {"q_target_deg": [0, -30, 80, 0, 60, 0]},
                        },
                    )
                    self.assertFalse(servo_disabled["ok"])
                    self.assertEqual(servo_disabled["error"]["kind"], "ServoDisabled")

                    wrong_arm = self.tcp_file_request(
                        control_file,
                        {
                            "schema_version": PROTOCOL_VERSION,
                            "request_id": "persist-wrong-arm",
                            "op": "read_state",
                            "arm": "right",
                        },
                    )
                    self.assertFalse(wrong_arm["ok"])
                    self.assertEqual(wrong_arm["error"]["kind"], "WrongArm")

                    reinitialized = self.tcp_file_request(
                        control_file,
                        {
                            "schema_version": PROTOCOL_VERSION,
                            "request_id": "persist-reinit",
                            "op": "initialize",
                            "arm": "left",
                        },
                    )
                    self.assertTrue(reinitialized["ok"])

                    faulted = self.tcp_request(
                        service.admin_address,
                        {
                            "schema_version": PROTOCOL_VERSION,
                            "request_id": "persist-set-fault",
                            "op": "admin.set_fault",
                            "params": {"error_code": 4321},
                        },
                    )
                    self.assertTrue(faulted["ok"])

                    robot_fault = self.tcp_file_request(
                        control_file,
                        {
                            "schema_version": PROTOCOL_VERSION,
                            "request_id": "persist-robot-fault",
                            "op": "send_servo_j",
                            "arm": "left",
                            "params": {"q_target_deg": [1, -30, 80, 0, 60, 0]},
                        },
                    )
                    self.assertFalse(robot_fault["ok"])
                    self.assertEqual(robot_fault["error"]["kind"], "RobotFault")

                    still_open = self.tcp_file_request(
                        control_file,
                        {
                            "schema_version": PROTOCOL_VERSION,
                            "request_id": "persist-read-after-errors",
                            "op": "read_state",
                            "arm": "left",
                        },
                    )
                    self.assertTrue(still_open["ok"])
                    self.assertEqual(still_open["state"]["error_code"], 4321)
        finally:
            service.stop()

    def test_left_and_right_services_can_run_concurrently(self) -> None:
        left = RbsimService.with_binds(
            load_simulator_config(CONFIG_LEFT),
            "tcp://127.0.0.1:0",
            "tcp://127.0.0.1:0",
        )
        right = RbsimService.with_binds(
            load_simulator_config(CONFIG_RIGHT),
            "tcp://127.0.0.1:0",
            "tcp://127.0.0.1:0",
        )
        try:
            left.start(start_tick_loop=False)
            right.start(start_tick_loop=False)
        except PermissionError as exc:
            self.skipTest(f"loopback sockets are unavailable in this environment: {exc}")
        try:
            left_ok = self.tcp_request(
                left.control_address,
                {"schema_version": PROTOCOL_VERSION, "request_id": "left-ok", "op": "read_state", "arm": "left"},
            )
            right_ok = self.tcp_request(
                right.control_address,
                {"schema_version": PROTOCOL_VERSION, "request_id": "right-ok", "op": "read_state", "arm": "right"},
            )
            left_wrong = self.tcp_request(
                left.control_address,
                {"schema_version": PROTOCOL_VERSION, "request_id": "left-wrong", "op": "read_state", "arm": "right"},
            )
            right_wrong = self.tcp_request(
                right.control_address,
                {"schema_version": PROTOCOL_VERSION, "request_id": "right-wrong", "op": "read_state", "arm": "left"},
            )

            self.assertTrue(left_ok["ok"])
            self.assertTrue(right_ok["ok"])
            self.assertFalse(left_wrong["ok"])
            self.assertEqual(left_wrong["error"]["name"], "wrong_arm")
            self.assertFalse(right_wrong["ok"])
            self.assertEqual(right_wrong["error"]["name"], "wrong_arm")
        finally:
            left.stop()
            right.stop()

    def tcp_request(self, address: tuple[str, int], payload: dict[str, Any]) -> dict[str, Any]:
        line = json.dumps(payload, separators=(",", ":")).encode("utf-8") + b"\n"
        with socket.create_connection(address, timeout=1.0) as sock:
            sock.sendall(line)
            response = sock.makefile("rb").readline()
        decoded = json.loads(response.decode("utf-8"))
        self.assertEqual(decoded["schema_version"], PROTOCOL_VERSION)
        return decoded

    def tcp_file_request(self, sock_file: Any, payload: dict[str, Any]) -> dict[str, Any]:
        line = json.dumps(payload, separators=(",", ":")).encode("utf-8") + b"\n"
        sock_file.write(line)
        sock_file.flush()
        response = sock_file.readline()
        self.assertNotEqual(response, b"", f"socket closed before response to {payload.get('request_id')}")
        decoded = json.loads(response.decode("utf-8"))
        self.assertEqual(decoded["schema_version"], PROTOCOL_VERSION)
        self.assertEqual(decoded["request_id"], payload["request_id"])
        return decoded


if __name__ == "__main__":
    unittest.main()
