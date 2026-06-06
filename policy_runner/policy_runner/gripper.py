from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Any, Mapping, Protocol


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
