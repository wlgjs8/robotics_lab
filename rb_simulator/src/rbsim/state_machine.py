"""Deterministic dual-arm simulator state machine."""

from __future__ import annotations

from dataclasses import dataclass

from .config import ArmConfig, FaultDefaults, SimulatorConfig

JointTuple = tuple[float, float, float, float, float, float]


class SimulatorError(RuntimeError):
    """Raised when a simulator operation violates the state-machine contract."""


@dataclass(frozen=True)
class ArmSnapshot:
    arm: str
    q_actual_deg: JointTuple
    q_target_deg: JointTuple
    dq_actual_deg_s: JointTuple
    stale_state: bool
    has_valid_joint_state: bool
    connected: bool
    initialized: bool
    servo_enabled: bool
    stopped: bool
    faulted: bool
    fault_recoverable: bool
    error_code: int
    robot_time_ns: int
    lifecycle_state: str


@dataclass
class _ArmRuntime:
    config: ArmConfig
    q_actual_deg: JointTuple
    q_target_deg: JointTuple
    last_safe_target_deg: JointTuple
    dq_actual_deg_s: JointTuple
    stale_state: bool
    has_valid_joint_state: bool
    connected: bool
    initialized: bool
    servo_enabled: bool
    stopped: bool
    faulted: bool
    fault_recoverable: bool
    error_code: int
    robot_time_ns: int = 0
    tracking_bias_deg: JointTuple = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    motion_frozen: bool = False

    @classmethod
    def from_config(cls, config: ArmConfig, defaults: FaultDefaults) -> "_ArmRuntime":
        return cls(
            config=config,
            q_actual_deg=config.initial_q_deg,
            q_target_deg=config.initial_q_deg,
            last_safe_target_deg=config.initial_q_deg,
            dq_actual_deg_s=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            stale_state=False,
            has_valid_joint_state=defaults.has_valid_joint_state,
            connected=defaults.connected,
            initialized=defaults.initialized,
            servo_enabled=defaults.servo_enabled,
            stopped=False,
            faulted=defaults.has_error,
            fault_recoverable=True,
            error_code=defaults.error_code,
        )

    def snapshot(self, arm: str) -> ArmSnapshot:
        q_actual_deg = _add_joints(self.q_actual_deg, self.tracking_bias_deg)
        return ArmSnapshot(
            arm=arm,
            q_actual_deg=q_actual_deg,
            q_target_deg=self.q_target_deg,
            dq_actual_deg_s=self.dq_actual_deg_s,
            stale_state=self.stale_state,
            has_valid_joint_state=self.has_valid_joint_state,
            connected=self.connected,
            initialized=self.initialized,
            servo_enabled=self.servo_enabled,
            stopped=self.stopped,
            faulted=self.faulted,
            fault_recoverable=self.fault_recoverable,
            error_code=self.error_code,
            robot_time_ns=self.robot_time_ns,
            lifecycle_state=self.lifecycle_state(),
        )

    def lifecycle_state(self) -> str:
        if self.faulted:
            return "faulted"
        if not self.connected:
            return "disconnected"
        if self.stopped:
            return "stopped"
        if self.servo_enabled:
            return "servo_enabled"
        if self.initialized:
            return "initialized"
        return "connected"


class DualArmSimulator:
    """Hardware-free simulator for two six-joint RB arms."""

    def __init__(self, config: SimulatorConfig):
        if config.update_rate_hz <= 0:
            raise ValueError("update_rate_hz must be positive")
        if config.motion_time_constant_sec <= 0:
            raise ValueError("motion_time_constant_sec must be positive")
        missing = {"left", "right"} - set(config.arms)
        if missing:
            raise ValueError(f"missing arm config: {', '.join(sorted(missing))}")

        self.config = config
        self._arms = {
            name: _ArmRuntime.from_config(arm_config, config.fault_defaults)
            for name, arm_config in config.arms.items()
        }

    def snapshot(self, arm: str) -> ArmSnapshot:
        return self._arm(arm).snapshot(arm)

    def snapshots(self) -> dict[str, ArmSnapshot]:
        return {arm: runtime.snapshot(arm) for arm, runtime in self._arms.items()}

    def connect(self, arm: str) -> ArmSnapshot:
        runtime = self._arm(arm)
        runtime.connected = True
        runtime.stopped = False
        return runtime.snapshot(arm)

    def disconnect(self, arm: str) -> ArmSnapshot:
        runtime = self._arm(arm)
        runtime.connected = False
        runtime.initialized = False
        runtime.servo_enabled = False
        runtime.stopped = True
        runtime.dq_actual_deg_s = _zero_joints()
        return runtime.snapshot(arm)

    def initialize(self, arm: str) -> ArmSnapshot:
        runtime = self._ready_arm(arm, require_initialized=False, require_servo=False)
        runtime.initialized = True
        runtime.servo_enabled = False
        runtime.stopped = False
        return runtime.snapshot(arm)

    def enable_servo(self, arm: str) -> ArmSnapshot:
        runtime = self._ready_arm(arm, require_initialized=True, require_servo=False)
        runtime.servo_enabled = True
        runtime.stopped = False
        return runtime.snapshot(arm)

    def send_servo_j(self, arm: str, q_target_deg: list[float] | tuple[float, ...]) -> ArmSnapshot:
        runtime = self._ready_arm(arm, require_initialized=True, require_servo=True)
        target = _six_joint_tuple(q_target_deg, "q_target_deg")
        runtime.q_target_deg = target
        runtime.last_safe_target_deg = target
        runtime.stopped = False
        return runtime.snapshot(arm)

    def stop(self, arm: str) -> ArmSnapshot:
        runtime = self._arm(arm)
        if not runtime.connected:
            raise SimulatorError(f"{arm} is disconnected")
        runtime.q_target_deg = _hold_target(runtime)
        runtime.last_safe_target_deg = runtime.q_target_deg
        runtime.dq_actual_deg_s = _zero_joints()
        runtime.servo_enabled = False
        runtime.stopped = True
        return runtime.snapshot(arm)

    def set_fault(self, arm: str, error_code: int = 2001, *, recoverable: bool = True) -> ArmSnapshot:
        runtime = self._arm(arm)
        runtime.faulted = True
        runtime.fault_recoverable = bool(recoverable)
        runtime.error_code = int(error_code)
        runtime.servo_enabled = False
        runtime.stopped = True
        runtime.q_target_deg = _hold_target(runtime)
        runtime.dq_actual_deg_s = _zero_joints()
        return runtime.snapshot(arm)

    def reset_fault(self, arm: str) -> ArmSnapshot:
        runtime = self._arm(arm)
        if not runtime.connected:
            raise SimulatorError(f"{arm} is disconnected")
        if runtime.faulted and not runtime.fault_recoverable:
            raise SimulatorError(f"{arm} fault is unrecoverable")
        runtime.faulted = False
        runtime.fault_recoverable = True
        runtime.error_code = 0
        runtime.initialized = False
        runtime.servo_enabled = False
        runtime.stopped = True
        runtime.q_target_deg = _hold_target(runtime)
        runtime.last_safe_target_deg = runtime.q_target_deg
        runtime.dq_actual_deg_s = _zero_joints()
        return runtime.snapshot(arm)

    def set_joint_validity(self, arm: str, valid: bool) -> ArmSnapshot:
        runtime = self._arm(arm)
        runtime.has_valid_joint_state = bool(valid)
        return runtime.snapshot(arm)

    def set_stale_state(self, arm: str, enabled: bool) -> ArmSnapshot:
        runtime = self._arm(arm)
        runtime.stale_state = bool(enabled)
        if runtime.stale_state:
            runtime.dq_actual_deg_s = _zero_joints()
        return runtime.snapshot(arm)

    def set_tracking_bias(self, arm: str, bias_deg: list[float] | tuple[float, ...]) -> ArmSnapshot:
        runtime = self._arm(arm)
        runtime.tracking_bias_deg = _six_joint_tuple(bias_deg, "bias_deg")
        return runtime.snapshot(arm)

    def set_motion_frozen(self, arm: str, enabled: bool) -> ArmSnapshot:
        runtime = self._arm(arm)
        runtime.motion_frozen = bool(enabled)
        if runtime.motion_frozen:
            runtime.dq_actual_deg_s = _zero_joints()
        return runtime.snapshot(arm)

    def tick(self, steps: int = 1) -> dict[str, ArmSnapshot]:
        if steps < 0:
            raise ValueError("steps must be non-negative")
        dt_sec = 1.0 / self.config.update_rate_hz
        for _ in range(steps):
            for runtime in self._arms.values():
                self._advance_arm(runtime, dt_sec)
        return self.snapshots()

    def _advance_arm(self, runtime: _ArmRuntime, dt_sec: float) -> None:
        if runtime.stale_state:
            runtime.dq_actual_deg_s = _zero_joints()
            return

        runtime.robot_time_ns += int(round(dt_sec * 1_000_000_000))
        if not _motion_active(runtime) or runtime.motion_frozen:
            runtime.dq_actual_deg_s = _zero_joints()
            return

        alpha = min(1.0, dt_sec / self.config.motion_time_constant_sec)
        max_delta = self.config.max_joint_velocity_deg_s * dt_sec
        next_actual: list[float] = []
        next_velocity: list[float] = []
        for actual, target in zip(runtime.q_actual_deg, runtime.q_target_deg):
            delta = (target - actual) * alpha
            delta = max(-max_delta, min(max_delta, delta))
            updated = actual + delta
            if (target - actual) > 0:
                updated = min(updated, target)
            else:
                updated = max(updated, target)
            next_actual.append(updated)
            next_velocity.append((updated - actual) / dt_sec)

        runtime.q_actual_deg = _six_joint_tuple(next_actual, "q_actual_deg")
        runtime.dq_actual_deg_s = _six_joint_tuple(next_velocity, "dq_actual_deg_s")

    def _ready_arm(
        self,
        arm: str,
        *,
        require_initialized: bool,
        require_servo: bool,
    ) -> _ArmRuntime:
        runtime = self._arm(arm)
        if not runtime.connected:
            raise SimulatorError(f"{arm} is disconnected")
        if runtime.faulted:
            raise SimulatorError(f"{arm} is faulted")
        if not runtime.has_valid_joint_state:
            raise SimulatorError(f"{arm} joint state is invalid")
        if require_initialized and not runtime.initialized:
            raise SimulatorError(f"{arm} is not initialized")
        if require_servo and not runtime.servo_enabled:
            raise SimulatorError(f"{arm} servo is not enabled")
        return runtime

    def _arm(self, arm: str) -> _ArmRuntime:
        try:
            return self._arms[arm]
        except KeyError as exc:
            raise SimulatorError(f"unknown arm: {arm}") from exc


def _motion_active(runtime: _ArmRuntime) -> bool:
    return (
        runtime.connected
        and runtime.initialized
        and runtime.servo_enabled
        and not runtime.stopped
        and not runtime.faulted
        and runtime.has_valid_joint_state
    )


def _hold_target(runtime: _ArmRuntime) -> JointTuple:
    if runtime.has_valid_joint_state:
        return runtime.q_actual_deg
    return runtime.last_safe_target_deg


def _six_joint_tuple(values: list[float] | tuple[float, ...], field_name: str) -> JointTuple:
    try:
        parsed = tuple(float(value) for value in values)
    except TypeError as exc:
        raise ValueError(f"{field_name} must contain exactly 6 numeric joints") from exc
    except ValueError as exc:
        raise ValueError(f"{field_name} must contain exactly 6 numeric joints") from exc
    if len(parsed) != 6:
        raise ValueError(f"{field_name} must contain exactly 6 joints")
    return parsed  # type: ignore[return-value]


def _zero_joints() -> JointTuple:
    return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def _add_joints(left: JointTuple, right: JointTuple) -> JointTuple:
    return _six_joint_tuple([a + b for a, b in zip(left, right)], "q_actual_deg")
