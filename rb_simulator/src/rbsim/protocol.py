"""Versioned JSON Lines protocol for the hardware-free simulator."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any

from .state_machine import ArmSnapshot, DualArmSimulator, SimulatorError

PROTOCOL_VERSION = "rbsim.v1"

ERROR_BAD_REQUEST = 4000
ERROR_UNSUPPORTED_SCHEMA = 4001
ERROR_INVALID_JSON = 4002
ERROR_UNKNOWN_OPERATION = 4003
ERROR_WRONG_ENDPOINT = 4004
ERROR_DISCONNECTED = 1001
ERROR_NOT_INITIALIZED = 1002
ERROR_SERVO_DISABLED = 1003
ERROR_UNKNOWN_ARM = 1004
ERROR_FAULT_LATCHED = 2001
ERROR_INVALID_STATE = 2002
ERROR_SEND_FAILURE = 2101
ERROR_STOP_FAILURE = 2102
ERROR_RESET_FAILURE = 2103
ERROR_READ_FAILURE = 2104


class ProtocolError(RuntimeError):
    """A machine-testable protocol failure."""

    def __init__(self, name: str, message: str, code: int = ERROR_BAD_REQUEST):
        super().__init__(message)
        self.name = name
        self.code = code


@dataclass
class ArmFaultHooks:
    read_failure: bool = False
    send_failure: bool = False
    stop_failure: bool = False
    reset_failure: bool = False
    latency_ms: float = 0.0


@dataclass
class FaultInjectionState:
    arms: dict[str, ArmFaultHooks] = field(default_factory=dict)

    @classmethod
    def for_arms(cls, arms: list[str] | tuple[str, ...]) -> "FaultInjectionState":
        return cls({arm: ArmFaultHooks() for arm in arms})

    def hook(self, arm: str) -> ArmFaultHooks:
        try:
            return self.arms[arm]
        except KeyError as exc:
            raise ProtocolError("unknown_arm", f"unknown arm: {arm}", ERROR_UNKNOWN_ARM) from exc

    def reset(self, arm: str | None = None) -> None:
        if arm is None:
            for name in self.arms:
                self.arms[name] = ArmFaultHooks()
            return
        self.arms[arm] = ArmFaultHooks()


class SimulatorProtocol:
    """Request dispatcher shared by the control and admin JSONL endpoints."""

    def __init__(self, simulator: DualArmSimulator):
        self.simulator = simulator
        self.faults = FaultInjectionState.for_arms(tuple(simulator.snapshots()))

    def handle_json_line(self, payload: bytes | str, endpoint_role: str) -> str:
        request_id: str | None = None
        arm: str | None = None
        try:
            request = _decode_request(payload)
            request_id = _optional_string(request.get("request_id"))
            arm = _optional_string(request.get("arm"))
            response = self.handle_request(request, endpoint_role)
        except ProtocolError as exc:
            response = _error_response(request_id, arm, exc.name, str(exc), exc.code)
        except SimulatorError as exc:
            response = _error_response(request_id, arm, *_classify_simulator_error(str(exc), arm, self.simulator))
        except ValueError as exc:
            response = _error_response(request_id, arm, "bad_request", str(exc), ERROR_BAD_REQUEST)

        return json.dumps(response, sort_keys=True, separators=(",", ":"))

    def handle_request(self, request: dict[str, Any], endpoint_role: str) -> dict[str, Any]:
        schema_version = request.get("schema_version")
        if schema_version != PROTOCOL_VERSION:
            raise ProtocolError(
                "unsupported_schema_version",
                f"schema_version must be {PROTOCOL_VERSION!r}",
                ERROR_UNSUPPORTED_SCHEMA,
            )

        op = _required_string(request, "op")
        request_id = _optional_string(request.get("request_id"))
        params = request.get("params", {})
        if not isinstance(params, dict):
            raise ProtocolError("bad_request", "params must be an object")

        if op.startswith("admin."):
            if endpoint_role != "admin":
                raise ProtocolError("wrong_endpoint", "admin operation sent to control endpoint", ERROR_WRONG_ENDPOINT)
            return self._handle_admin(op, request_id, params, request)

        if endpoint_role != "control":
            raise ProtocolError("wrong_endpoint", "control operation sent to admin endpoint", ERROR_WRONG_ENDPOINT)
        return self._handle_control(op, request_id, params, request)

    def _handle_control(
        self,
        op: str,
        request_id: str | None,
        params: dict[str, Any],
        request: dict[str, Any],
    ) -> dict[str, Any]:
        arm = _required_string(request, "arm")
        self._apply_latency(arm)

        if op == "connect":
            snapshot = self.simulator.connect(arm)
        elif op == "initialize":
            snapshot = self.simulator.initialize(arm)
            if bool(params.get("enable_servo", True)):
                snapshot = self.simulator.enable_servo(arm)
        elif op == "read_state":
            hooks = self.faults.hook(arm)
            if hooks.read_failure:
                raise ProtocolError("read_failure_injected", f"{arm} read failure injected", ERROR_READ_FAILURE)
            snapshot = self.simulator.snapshot(arm)
        elif op == "send_servo_j":
            hooks = self.faults.hook(arm)
            if hooks.send_failure:
                raise ProtocolError("send_failure_injected", f"{arm} send failure injected", ERROR_SEND_FAILURE)
            snapshot = self.simulator.send_servo_j(arm, params.get("q_target_deg"))
        elif op == "stop":
            hooks = self.faults.hook(arm)
            if hooks.stop_failure:
                raise ProtocolError("stop_failure_injected", f"{arm} stop failure injected", ERROR_STOP_FAILURE)
            snapshot = self.simulator.stop(arm)
        elif op == "reset_fault":
            hooks = self.faults.hook(arm)
            if hooks.reset_failure:
                raise ProtocolError("reset_failure_injected", f"{arm} reset failure injected", ERROR_RESET_FAILURE)
            snapshot = self.simulator.reset_fault(arm)
        else:
            raise ProtocolError("unknown_operation", f"unknown control operation: {op}", ERROR_UNKNOWN_OPERATION)

        return _ok_response(request_id, arm, snapshot)

    def _handle_admin(
        self,
        op: str,
        request_id: str | None,
        params: dict[str, Any],
        request: dict[str, Any],
    ) -> dict[str, Any]:
        arm = _optional_string(request.get("arm") or params.get("arm"))
        snapshot: ArmSnapshot | None = None

        if op == "admin.tick":
            steps = int(params.get("steps", 1))
            states = self.simulator.tick(steps)
            return _ok_response(request_id, arm, states.get(arm) if arm else None, states=states)
        if op == "admin.reset_hooks":
            self.faults.reset(arm)
        elif op == "admin.inject":
            if arm is None:
                raise ProtocolError("bad_request", "admin.inject requires arm")
            hook_name = _required_string(params, "hook")
            enabled = bool(params.get("enabled", True))
            hooks = self.faults.hook(arm)
            if hook_name not in {"read_failure", "send_failure", "stop_failure", "reset_failure"}:
                raise ProtocolError("bad_request", f"unsupported hook: {hook_name}")
            setattr(hooks, hook_name, enabled)
        elif op == "admin.set_latency":
            if arm is None:
                raise ProtocolError("bad_request", "admin.set_latency requires arm")
            self.faults.hook(arm).latency_ms = float(params.get("latency_ms", 0.0))
        elif op == "admin.disconnect":
            if arm is None:
                raise ProtocolError("bad_request", "admin.disconnect requires arm")
            snapshot = self.simulator.disconnect(arm)
        elif op == "admin.reconnect":
            if arm is None:
                raise ProtocolError("bad_request", "admin.reconnect requires arm")
            snapshot = self.simulator.connect(arm)
        elif op == "admin.set_joint_validity":
            if arm is None:
                raise ProtocolError("bad_request", "admin.set_joint_validity requires arm")
            snapshot = self.simulator.set_joint_validity(arm, bool(params.get("valid", True)))
        elif op == "admin.set_stale_state":
            if arm is None:
                raise ProtocolError("bad_request", "admin.set_stale_state requires arm")
            snapshot = self.simulator.set_stale_state(arm, bool(params.get("enabled", True)))
        elif op == "admin.set_fault":
            if arm is None:
                raise ProtocolError("bad_request", "admin.set_fault requires arm")
            snapshot = self.simulator.set_fault(
                arm,
                int(params.get("error_code", ERROR_FAULT_LATCHED)),
                recoverable=bool(params.get("recoverable", True)),
            )
        elif op == "admin.set_tracking_bias":
            if arm is None:
                raise ProtocolError("bad_request", "admin.set_tracking_bias requires arm")
            snapshot = self.simulator.set_tracking_bias(arm, params.get("bias_deg", [0, 0, 0, 0, 0, 0]))
        elif op == "admin.freeze_motion":
            if arm is None:
                raise ProtocolError("bad_request", "admin.freeze_motion requires arm")
            snapshot = self.simulator.set_motion_frozen(arm, bool(params.get("enabled", True)))
        else:
            raise ProtocolError("unknown_operation", f"unknown admin operation: {op}", ERROR_UNKNOWN_OPERATION)

        hooks = self.faults.hook(arm).__dict__ if arm else {name: hook.__dict__ for name, hook in self.faults.arms.items()}
        return _ok_response(request_id, arm, snapshot, admin={"fault_hooks": hooks})

    def _apply_latency(self, arm: str) -> None:
        latency_sec = self.faults.hook(arm).latency_ms / 1000.0
        if latency_sec > 0:
            time.sleep(latency_sec)


def _decode_request(payload: bytes | str) -> dict[str, Any]:
    try:
        text = payload.decode("utf-8") if isinstance(payload, bytes) else payload
        decoded = json.loads(text)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProtocolError("invalid_json", "request line must be valid UTF-8 JSON", ERROR_INVALID_JSON) from exc
    if not isinstance(decoded, dict):
        raise ProtocolError("bad_request", "request must be a JSON object")
    return decoded


def _required_string(mapping: dict[str, Any], key: str) -> str:
    value = mapping.get(key)
    if not isinstance(value, str) or not value:
        raise ProtocolError("bad_request", f"{key} must be a non-empty string")
    return value


def _optional_string(value: Any) -> str | None:
    return value if isinstance(value, str) and value else None


def _ok_response(
    request_id: str | None,
    arm: str | None,
    snapshot: ArmSnapshot | None,
    *,
    states: dict[str, ArmSnapshot] | None = None,
    admin: dict[str, Any] | None = None,
) -> dict[str, Any]:
    response: dict[str, Any] = {
        "schema_version": PROTOCOL_VERSION,
        "request_id": request_id,
        "ok": True,
        "server_time_ns": time.monotonic_ns(),
    }
    if arm is not None:
        response["arm"] = arm
    if snapshot is not None:
        response["state"] = snapshot_to_state(snapshot)
    if states is not None:
        response["states"] = {name: snapshot_to_state(state) for name, state in states.items()}
    if admin is not None:
        response["admin"] = admin
    return response


def _error_response(
    request_id: str | None,
    arm: str | None,
    name: str,
    message: str,
    code: int,
) -> dict[str, Any]:
    response: dict[str, Any] = {
        "schema_version": PROTOCOL_VERSION,
        "request_id": request_id,
        "ok": False,
        "server_time_ns": time.monotonic_ns(),
        "error": {"name": name, "message": message, "code": int(code)},
    }
    if arm is not None:
        response["arm"] = arm
    return response


def _classify_simulator_error(
    message: str,
    arm: str | None,
    simulator: DualArmSimulator,
) -> tuple[str, str, int]:
    if arm is not None:
        try:
            snapshot = simulator.snapshot(arm)
            if snapshot.faulted:
                return ("fault_latched", message, snapshot.error_code or ERROR_FAULT_LATCHED)
        except SimulatorError:
            pass
    if "unknown arm" in message:
        return ("unknown_arm", message, ERROR_UNKNOWN_ARM)
    if "disconnected" in message:
        return ("disconnected", message, ERROR_DISCONNECTED)
    if "not initialized" in message:
        return ("not_initialized", message, ERROR_NOT_INITIALIZED)
    if "servo is not enabled" in message:
        return ("servo_disabled", message, ERROR_SERVO_DISABLED)
    if "joint state is invalid" in message:
        return ("invalid_joint_state", message, ERROR_INVALID_STATE)
    return ("state_rejected", message, ERROR_BAD_REQUEST)


def snapshot_to_state(snapshot: ArmSnapshot) -> dict[str, Any]:
    return {
        "arm": snapshot.arm,
        "q_actual_deg": list(snapshot.q_actual_deg),
        "q_target_deg": list(snapshot.q_target_deg),
        "dq_actual_deg_s": list(snapshot.dq_actual_deg_s),
        "stale_state": snapshot.stale_state,
        "has_valid_joint_state": snapshot.has_valid_joint_state,
        "connection_state": "Connected" if snapshot.connected else "Disconnected",
        "servo_enabled": snapshot.servo_enabled,
        "has_error": snapshot.faulted,
        "fault_recoverable": snapshot.fault_recoverable,
        "error_code": snapshot.error_code,
        "robot_time_ns": snapshot.robot_time_ns,
        "lifecycle_state": snapshot.lifecycle_state,
    }
