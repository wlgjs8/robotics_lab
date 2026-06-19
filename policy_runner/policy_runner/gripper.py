from __future__ import annotations

import logging
import os
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Mapping, Protocol


REAL_GRIPPER_ENV = "RB_ALLOW_REAL_GRIPPER"
GRIPPER_EPSILON = 1e-12
_PIKA_SDK_LOGGER_NAMES = ("pika", "pika.serial_comm", "pika.gripper", "pika.sense")


def suppress_pika_sdk_logging() -> None:
    """Silence noisy logs emitted by the vendor Pika SDK.

    The SDK calls logging.basicConfig() at import time and its serial reader
    reports parse noise at ERROR level. policy_runner reports gripper command
    outcomes through GripperDispatchResult instead.
    """
    names = set(_PIKA_SDK_LOGGER_NAMES)
    names.update(
        name
        for name in logging.root.manager.loggerDict
        if name == "pika" or name.startswith("pika.")
    )
    for name in names:
        logger = logging.getLogger(name)
        logger.handlers.clear()
        logger.propagate = False
        logger.disabled = True
        logger.setLevel(logging.CRITICAL + 1)


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

    Policy gripper actions are per-step deltas in the dataset's gripper units:
    PERCENT of the open/close range (0 = closed = min_rad, 100 = open =
    max_rad; pika UMI conversion uses gripper_open_close_units: percent).
    Deltas integrate onto a per-arm target seeded from the live motor position
    at connect(), clamped to [min_rad, max_rad]; 'target' commands set the
    percent absolutely. current_percent() exposes the live motor angle in the
    same percent units for proprio feedback.

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
        suppress_sdk_logs: bool = True,
        gripper_cls: type | None = None,
        clock: Callable[[], float] = time.monotonic,
        home_on_connect: bool = True,
        home_timeout_sec: float = 3.0,
        home_settle_eps_rad: float = 0.01,
        home_poll_sec: float = 0.05,
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
        self.suppress_sdk_logs = bool(suppress_sdk_logs)
        self.home_on_connect = bool(home_on_connect)
        self.home_timeout_sec = float(home_timeout_sec)
        self.home_settle_eps_rad = float(home_settle_eps_rad)
        self.home_poll_sec = float(home_poll_sec)
        self._gripper_cls = gripper_cls
        self._clock = clock
        self._grippers: dict[str, Any] = {}
        self._targets: dict[str, float] = {}
        self._last_sent: dict[str, tuple[float, float]] = {}

    def connect(self) -> "PikaSerialGripperBackend":
        if self.suppress_sdk_logs:
            suppress_pika_sdk_logging()
        if self._gripper_cls is None:
            for arm, port in self.ports.items():
                if not os.path.exists(port):
                    self.close()
                    raise RuntimeError(f"pika gripper {arm} serial port not found: {port}")
        gripper_cls = self._gripper_cls or _import_pika_gripper_class(self.sdk_path)
        if self.suppress_sdk_logs:
            suppress_pika_sdk_logging()
        for arm, port in self.ports.items():
            gripper = gripper_cls(port=port)
            if not gripper.connect():
                self.close()
                detail = port
                if os.path.islink(port):
                    detail = f"{port} -> {os.path.realpath(port)}"
                raise RuntimeError(f"pika gripper {arm} connect failed on {detail}")
            if not gripper.enable():
                self.close()
                raise RuntimeError(f"pika gripper {arm} enable failed on {port}")
            self._grippers[arm] = gripper
            self._targets[arm] = self._seed_target(gripper)
        if self.home_on_connect and self._grippers:
            self._home_all_concurrent()
        return self

    def _home_all_concurrent(self) -> None:
        """Reference every gripper to its CLOSED mechanical stop and re-zero there,
        so absolute-angle commands (set_motor_angle) map to the SAME physical
        opening on both (identical) grippers. Without this the motor zero is the
        arbitrary power-on position, so a 100%-open command lands at a different
        physical opening per arm (observed: right ~39% vs left ~70% open).

        Both arms home in parallel threads (independent serial ports) so the whole
        step takes one gripper's homing time, not the sum. Failures are logged and
        non-fatal: a connected+enabled gripper still works, just uncalibrated."""
        threads = []
        for arm, gripper in self._grippers.items():
            t = threading.Thread(
                target=self._home_one, args=(arm, gripper), name=f"gripper-home-{arm}", daemon=True
            )
            t.start()
            threads.append(t)
        # Join with margin over the per-arm settle timeout so a stuck arm can't
        # hang startup forever.
        for t in threads:
            t.join(timeout=self.home_timeout_sec + 1.0)

    def _home_one(self, arm: str, gripper: Any) -> None:
        try:
            # 1. Drive toward the closed stop (min_rad). set_motor_angle clamps
            #    rad<0 to 0, so commanding min_rad bottoms the jaw on the stop.
            gripper.set_motor_angle(self.min_rad)
            # 2. Wait until the jaw stops moving (settled against the stop) so the
            #    re-zero references the true mechanical closed position.
            self._wait_until_settled(gripper)
            # 3. Define the closed stop as zero. Subsequent set_motor_angle(rad) is
            #    now consistent across both grippers; max_rad == true full open.
            if hasattr(gripper, "set_zero"):
                gripper.set_zero()
            self._targets[arm] = self.min_rad
            self._last_sent.pop(arm, None)
            print(f"[gripper] homed {arm}: closed-stop zeroed", file=sys.stderr, flush=True)
        except Exception as exc:  # noqa: BLE001 - homing must not crash startup
            print(
                f"[gripper] WARN home {arm} failed ({type(exc).__name__}: {exc}); "
                "gripper left uncalibrated (open may be partial)",
                file=sys.stderr,
                flush=True,
            )

    def _wait_until_settled(self, gripper: Any) -> None:
        """Poll the motor position until two consecutive reads agree within
        home_settle_eps_rad (jaw stopped) or home_timeout_sec elapses."""
        deadline = self._clock() + self.home_timeout_sec
        last: float | None = None
        while self._clock() < deadline:
            time.sleep(self.home_poll_sec)
            try:
                pos = float(gripper.get_motor_position())
            except Exception:
                return  # no feedback -> fall back to the time already spent moving
            if last is not None and abs(pos - last) < self.home_settle_eps_rad:
                return
            last = pos

    def _seed_target(self, gripper: Any) -> float:
        try:
            position = float(gripper.get_motor_position())
        except Exception:
            position = self.min_rad
        return self._clamp(position)

    def _clamp(self, value: float) -> float:
        return max(self.min_rad, min(self.max_rad, float(value)))

    def _percent_to_rad(self, percent: float) -> float:
        return self.min_rad + (self.max_rad - self.min_rad) * float(percent) / 100.0

    def _rad_to_percent(self, rad: float) -> float:
        return (float(rad) - self.min_rad) / (self.max_rad - self.min_rad) * 100.0

    def current_percent(self, arm: str) -> float | None:
        """Live motor angle in dataset percent units (proprio feedback)."""
        gripper = self._grippers.get(arm)
        if gripper is None:
            return None
        try:
            return self._rad_to_percent(float(gripper.get_motor_position()))
        except Exception:
            return None

    def send(self, command: GripperCommand) -> GripperDispatchResult:
        gripper = self._grippers.get(command.arm)
        if gripper is None:
            return self._result(command, accepted=False, sent=False, dropped=True, reason="gripper_arm_not_connected")
        # Command values are in dataset percent units; motors take rad.
        if command.command_type == "target":
            target = self._clamp(self._percent_to_rad(command.value))
        else:
            delta_rad = (self.max_rad - self.min_rad) * float(command.value) / 100.0
            target = self._clamp(self._targets.get(command.arm, self.min_rad) + delta_rad)
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

    def dispatch(
        self,
        commands: list[GripperCommand] | tuple[GripperCommand, ...],
        *,
        concurrent: bool = False,
    ) -> list[GripperDispatchResult]:
        # Gate first (cheap, in order). The slow part is backend.send -> the
        # blocking per-arm serial write; with concurrent=True those run in
        # parallel threads so both grippers move at once instead of
        # left-then-right (each arm is an independent serial port, so concurrent
        # writes are safe and touch disjoint backend state). Default stays
        # sequential for the per-tick policy path.
        plan: list[tuple[str, Any]] = []  # ("result", res) | ("send", command)
        for command in commands:
            if not command.is_nonzero:
                continue
            self.command_count += 1
            decision = self._gate(command)
            if decision is not None:
                self.dropped_count += 1
                plan.append(("result", decision))
                continue
            plan.append(("send", command))

        send_cmds = [item for kind, item in plan if kind == "send"]
        sent: dict[int, GripperDispatchResult] = {}
        if concurrent and len(send_cmds) > 1:
            threads = []
            for cmd in send_cmds:
                t = threading.Thread(
                    target=lambda c=cmd: sent.__setitem__(id(c), self.backend.send(c))
                )
                t.start()
                threads.append(t)
            for t in threads:
                t.join()
        else:
            for cmd in send_cmds:
                sent[id(cmd)] = self.backend.send(cmd)

        results: list[GripperDispatchResult] = []
        for kind, item in plan:
            result = item if kind == "result" else sent[id(item)]
            if kind == "send" and result.dropped:
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
