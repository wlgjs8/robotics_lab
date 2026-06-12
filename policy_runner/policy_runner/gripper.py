from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Mapping, Protocol


REAL_GRIPPER_ENV = "RB_ALLOW_REAL_GRIPPER"
GRIPPER_EPSILON = 1e-12


@dataclass(frozen=True)
class GripperCommand:
    arm: str
    value: float
    command_type: str = "delta"
    source: str = "flow_policy"

    def __post_init__(self) -> None:
        if self.arm not in {"left", "right"}:
            raise ValueError("gripper command arm must be left or right")
        if self.command_type not in {"delta", "target"}:
            raise ValueError("gripper command_type must be delta or target")

    @property
    def is_nonzero(self) -> bool:
        return abs(float(self.value)) > GRIPPER_EPSILON

    def to_dict(self) -> dict[str, Any]:
        return {
            "arm": self.arm,
            "value": float(self.value),
            "command_type": self.command_type,
            "source": self.source,
        }


@dataclass(frozen=True)
class GripperDispatchResult:
    command: GripperCommand
    accepted: bool
    sent_to_physical: bool
    dropped: bool
    reason: str


class GripperBackend(Protocol):
    supports_controller_simulation: bool

    def send(self, command: GripperCommand) -> GripperDispatchResult:
        ...


@dataclass
class NoopGripperBackend:
    """Dry-run gripper backend that records commands and never touches hardware."""

    reason: str = "noop_gripper_backend"
    supports_controller_simulation: bool = False
    commands: list[GripperCommand] = field(default_factory=list)

    def send(self, command: GripperCommand) -> GripperDispatchResult:
        self.commands.append(command)
        return GripperDispatchResult(
            command=command,
            accepted=False,
            sent_to_physical=False,
            dropped=True,
            reason=self.reason,
        )


def _import_pika_gripper_class(sdk_path: str | None) -> type:
    """Import pika.gripper.Gripper, optionally from a configured SDK copy.

    The AgileX Pika SDK is not packaged on this PC's default sys.path; the
    copy from the SteamVR PC's conda env lives at gripper.pika_sdk_path
    (same convention as scripts/umi_gripper_follow.py).
    """
    if sdk_path and sdk_path not in sys.path:
        sys.path.insert(0, sdk_path)
    try:
        from pika.gripper import Gripper  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError(
            f"pika.gripper import failed ({exc}); set gripper.pika_sdk_path to the "
            "directory containing the 'pika' package"
        ) from exc
    return Gripper


class PikaSerialGripperBackend:
    """Drives the robot-mounted Pika grippers over local serial POSITION_CTRL.

    Policy gripper actions are per-step deltas in the dataset's raw Sense
    encoder units (rad; per the Pika manual the Sense and Gripper share motor
    parameters, so the angle is a 1:1 passthrough — same convention as
    scripts/umi_gripper_follow.py). Deltas integrate onto a per-arm target
    seeded from the live motor position at connect(), clamped to
    [min_rad, max_rad]; 'target' commands set the angle absolutely.

    send() never raises into the control loop: serial errors are reported as
    dropped dispatch results.
    """

    def __init__(
        self,
        ports: Mapping[str, str],
        *,
        sdk_path: str | None = None,
        min_rad: float = 0.0,
        max_rad: float = 1.75,
        deadband_rad: float = 0.005,
        max_hz: float = 60.0,
        supports_controller_simulation: bool = False,
        gripper_cls: type | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if max_rad <= min_rad:
            raise ValueError("gripper max_rad must be greater than min_rad")
        if deadband_rad < 0.0:
            raise ValueError("gripper deadband_rad must be non-negative")
        self.ports = {str(arm): str(port) for arm, port in ports.items()}
        for arm in self.ports:
            if arm not in {"left", "right"}:
                raise ValueError("gripper port arms must be left or right")
        self.sdk_path = sdk_path
        self.min_rad = float(min_rad)
        self.max_rad = float(max_rad)
        self.deadband_rad = float(deadband_rad)
        self.min_period_sec = 1.0 / float(max_hz) if max_hz > 0 else 0.0
        self.supports_controller_simulation = bool(supports_controller_simulation)
        self._gripper_cls = gripper_cls
        self._clock = clock
        self._grippers: dict[str, Any] = {}
        self._targets: dict[str, float] = {}
        self._last_sent: dict[str, tuple[float, float]] = {}

    def connect(self) -> "PikaSerialGripperBackend":
        gripper_cls = self._gripper_cls or _import_pika_gripper_class(self.sdk_path)
        for arm, port in self.ports.items():
            gripper = gripper_cls(port=port)
            if not gripper.connect():
                self.close()
                raise RuntimeError(f"pika gripper {arm} connect failed on {port}")
            if not gripper.enable():
                self.close()
                raise RuntimeError(f"pika gripper {arm} enable failed on {port}")
            self._grippers[arm] = gripper
            self._targets[arm] = self._seed_target(gripper)
        return self

    def _seed_target(self, gripper: Any) -> float:
        try:
            position = float(gripper.get_motor_position())
        except Exception:
            position = self.min_rad
        return self._clamp(position)

    def _clamp(self, value: float) -> float:
        return max(self.min_rad, min(self.max_rad, float(value)))

    def send(self, command: GripperCommand) -> GripperDispatchResult:
        gripper = self._grippers.get(command.arm)
        if gripper is None:
            return self._result(command, accepted=False, sent=False, dropped=True, reason="gripper_arm_not_connected")
        if command.command_type == "target":
            target = self._clamp(command.value)
        else:
            target = self._clamp(self._targets.get(command.arm, self.min_rad) + float(command.value))
        # The integrated target always advances; deadband/rate gates only skip
        # the serial write so small deltas accumulate instead of being lost.
        self._targets[command.arm] = target
        now = self._clock()
        last = self._last_sent.get(command.arm)
        if last is not None:
            last_time, last_rad = last
            if self.min_period_sec > 0.0 and now - last_time < self.min_period_sec:
                # Held, not lost: the integrated target carries to the next send.
                return self._result(command, accepted=True, sent=False, dropped=False, reason="gripper_rate_limited")
            if abs(target - last_rad) < self.deadband_rad:
                return self._result(command, accepted=True, sent=False, dropped=False, reason="gripper_deadband_hold")
        try:
            ok = bool(gripper.set_motor_angle(target))
        except Exception as exc:
            return self._result(command, accepted=False, sent=False, dropped=True, reason=f"gripper_serial_error:{exc}")
        if not ok:
            return self._result(command, accepted=False, sent=False, dropped=True, reason="gripper_command_rejected")
        self._last_sent[command.arm] = (now, target)
        return self._result(command, accepted=True, sent=True, dropped=False, reason="gripper_position_sent")

    def close(self) -> None:
        for gripper in self._grippers.values():
            for method_name in ("disable", "disconnect"):
                method = getattr(gripper, method_name, None)
                if method is None:
                    continue
                try:
                    method()
                except Exception:
                    pass
        self._grippers = {}

    @staticmethod
    def _result(
        command: GripperCommand, *, accepted: bool, sent: bool, dropped: bool, reason: str
    ) -> GripperDispatchResult:
        return GripperDispatchResult(
            command=command,
            accepted=accepted,
            sent_to_physical=sent,
            dropped=dropped,
            reason=reason,
        )


@dataclass
class GripperRuntime:
    rollout_mode: str
    allow_real_gripper_motion: bool = False
    backend: GripperBackend = field(default_factory=NoopGripperBackend)
    env: Mapping[str, str] | None = None
    command_count: int = 0
    dropped_count: int = 0
    results: list[GripperDispatchResult] = field(default_factory=list)

    def dispatch(self, commands: list[GripperCommand] | tuple[GripperCommand, ...]) -> list[GripperDispatchResult]:
        results: list[GripperDispatchResult] = []
        for command in commands:
            if not command.is_nonzero:
                continue
            self.command_count += 1
            decision = self._gate(command)
            if decision is not None:
                self.dropped_count += 1
                self.results.append(decision)
                results.append(decision)
                continue
            result = self.backend.send(command)
            if result.dropped:
                self.dropped_count += 1
            self.results.append(result)
            results.append(result)
        return results

    @property
    def latest_reason(self) -> str | None:
        if not self.results:
            return None
        return self.results[-1].reason

    def _gate(self, command: GripperCommand) -> GripperDispatchResult | None:
        mode = str(self.rollout_mode or "").strip().lower()
        if mode == "real_policy":
            if not self.allow_real_gripper_motion:
                return self._drop(command, "real_gripper_config_not_allowed")
            if self._env().get(REAL_GRIPPER_ENV) != "1":
                return self._drop(command, "real_gripper_env_missing")
            return None
        if mode == "controller_sim":
            if bool(getattr(self.backend, "supports_controller_simulation", False)):
                return None
            return self._drop(command, "controller_sim_gripper_logged_noop")
        if mode in {"offline_eval", "sim_dryrun", "real_readonly"}:
            return self._drop(command, f"{mode}_gripper_logged_noop")
        return self._drop(command, "gripper_backend_not_configured")

    def _drop(self, command: GripperCommand, reason: str) -> GripperDispatchResult:
        return GripperDispatchResult(
            command=command,
            accepted=False,
            sent_to_physical=False,
            dropped=True,
            reason=reason,
        )

    def _env(self) -> Mapping[str, str]:
        return os.environ if self.env is None else self.env


def gripper_commands_from_flow_step(
    step: Any,
    *,
    arm_mask: Any,
    command_type: str = "delta",
    source: str = "flow_policy",
) -> list[GripperCommand]:
    values = list(step)
    mask = list(arm_mask)
    commands: list[GripperCommand] = []
    if len(values) >= 7 and len(mask) >= 1 and float(mask[0]) > 0.0:
        commands.append(
            GripperCommand(
                arm="left",
                value=float(values[6]),
                command_type=command_type,
                source=source,
            )
        )
    if len(values) >= 14 and len(mask) >= 2 and float(mask[1]) > 0.0:
        commands.append(
            GripperCommand(
                arm="right",
                value=float(values[13]),
                command_type=command_type,
                source=source,
            )
        )
    return commands
