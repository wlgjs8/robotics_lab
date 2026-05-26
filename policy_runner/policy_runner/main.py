from __future__ import annotations

import argparse
import sys
import time
from typing import TextIO

from .action_sources import (
    DualSpaceMouseCartesianActionSource,
    HoldActionSource,
    JointSineActionSource,
    JointVelocityActionSource,
    SpaceMouseCartesianActionSource,
    SpaceMouseJointVelocityActionSource,
    TcpDeltaActionSource,
)
from .config import PolicyRunnerConfig, load_config
from .geometry import GeometryStatus, load_geometry_status
from .robot_state_client import RobotStateClient, StateStreamLeaseReadback
from .safety import SafetyGate
from .servo_command_client import CommandIntent, ServoCommandClient
from .spacemouse import HidSpaceMouseReader, SpaceMouseReader, SpaceMouseSample


STARTUP_TIMEOUT_EXIT_CODE = 2
LEASE_READBACK_TIMEOUT_EXIT_CODE = 3


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="robotics_lab policy_runner")
    parser.add_argument("--config", required=True, help="policy_runner YAML config")
    args = parser.parse_args(argv)
    config = load_config(args.config)
    return run(config)


def run(
    config: PolicyRunnerConfig,
    *,
    state_client: RobotStateClient | None = None,
    command_client: ServoCommandClient | None = None,
    source: object | None = None,
    sleep_fn=time.sleep,
    monotonic_fn=time.monotonic,
    stderr: TextIO = sys.stderr,
) -> int:
    state_client = state_client or RobotStateClient(config.robot_state.bind, config.robot_state.stale_timeout_sec)
    command_client = command_client or ServoCommandClient(
        config.servo_command.endpoint,
        config.servo_command.timeout_sec,
    )
    source = source or make_action_source(config)
    safety_gate = SafetyGate(
        config.mode,
        config.safety,
        config.robot_state.stale_timeout_sec,
        geometry_status=_load_runtime_geometry_status(config),
    )
    state_client.start()
    armed_for_motion = False
    lease_acquired = False
    period = 1.0 / max(config.command_rate_hz, 1.0)
    startup_deadline = monotonic_fn() + max(config.runtime.startup_timeout_sec, 0.0)
    try:
        while True:
            snapshot = state_client.latest
            now = monotonic_fn()
            if snapshot is None:
                if now >= startup_deadline:
                    print(
                        "policy_runner startup_timeout_no_state: "
                        f"no robot state received within {config.runtime.startup_timeout_sec:.3f}s "
                        f"on {config.robot_state.bind}",
                        file=stderr,
                    )
                    return STARTUP_TIMEOUT_EXIT_CODE
                sleep_fn(period)
                continue
            if config.servo_command.acquire_lease and not lease_acquired:
                try:
                    command_client.acquire_lease(
                        StateStreamLeaseReadback(state_client),
                        timeout_sec=config.servo_command.lease_readback_timeout_sec,
                        monotonic_fn=monotonic_fn,
                        sleep_fn=sleep_fn,
                    )
                except TimeoutError as exc:
                    print(f"policy_runner lease_readback_timeout: {exc}", file=stderr)
                    return LEASE_READBACK_TIMEOUT_EXIT_CODE
                lease_acquired = True
            intent = source.next_intent(snapshot, now)
            decision = safety_gate.evaluate(snapshot, intent, getattr(source, "requirements", None), now)
            if decision.allowed and intent is not None:
                if intent.is_motion and not armed_for_motion:
                    arm = CommandIntent.arm_motion(timeout_sec=config.servo_command.timeout_sec)
                    arm_decision = safety_gate.evaluate(snapshot, arm, getattr(source, "requirements", None), now)
                    if arm_decision.allowed:
                        command_client.send(arm)
                        armed_for_motion = True
                command_client.send(intent)
            sleep_fn(period)
    except KeyboardInterrupt:
        return 0
    finally:
        _close_if_supported(source)
        state_client.close()
        command_client.close()


def make_action_source(config: PolicyRunnerConfig):
    if config.action_source == "hold":
        return HoldActionSource(timeout_sec=config.servo_command.timeout_sec)
    if config.action_source == "joint_sine":
        return JointSineActionSource(
            amplitude_deg=config.joint_sine.amplitude_deg,
            frequency_hz=config.joint_sine.frequency_hz,
            selected_arm=config.joint_sine.selected_arm,
            timeout_sec=config.servo_command.timeout_sec,
            simulation_only=config.joint_sine.simulation_only,
        )
    if config.action_source == "joint_velocity":
        return JointVelocityActionSource(
            velocity_deg_s=config.joint_velocity.velocity_deg_s,
            selected_arm=config.joint_velocity.selected_arm,
            timeout_sec=config.servo_command.timeout_sec,
            simulation_only=config.joint_velocity.simulation_only,
        )
    if config.action_source == "spacemouse_joint_velocity":
        return SpaceMouseJointVelocityActionSource(
            reader=_LazyHidSpaceMouseReader(),
            selected_arm=config.spacemouse.selected_arm,
            max_joint_velocity_deg_s=config.spacemouse.max_joint_velocity_deg_s,
            deadband=config.spacemouse.deadband,
            smoothing_alpha=config.spacemouse.smoothing_alpha,
            require_deadman=config.spacemouse.require_deadman,
            deadman_button=config.spacemouse.deadman_button,
            timeout_sec=config.servo_command.timeout_sec,
        )
    if config.action_source == "tcp_delta":
        return TcpDeltaActionSource(
            delta=config.tcp_delta.delta,
            selected_arm=config.tcp_delta.selected_arm,
            frame=config.tcp_delta.frame,
            max_linear_step_m=config.tcp_delta.max_linear_step_m,
            max_angular_step_rad=config.tcp_delta.max_angular_step_rad,
            timeout_sec=config.servo_command.timeout_sec,
            simulation_only=config.tcp_delta.simulation_only,
        )
    if config.action_source == "spacemouse_cartesian":
        return SpaceMouseCartesianActionSource(
            reader=_LazyHidSpaceMouseReader(),
            selected_arm=config.spacemouse_cartesian.selected_arm,
            frame=config.spacemouse_cartesian.frame,
            max_linear_velocity_m_s=config.spacemouse_cartesian.max_linear_velocity_m_s,
            max_angular_velocity_rad_s=config.spacemouse_cartesian.max_angular_velocity_rad_s,
            deadband=config.spacemouse_cartesian.deadband,
            require_deadman=config.spacemouse_cartesian.require_deadman,
            deadman_button=config.spacemouse_cartesian.deadman_button,
            timeout_sec=config.servo_command.timeout_sec,
        )
    if config.action_source == "dual_spacemouse_cartesian":
        left = config.spacemouse_cartesian_dual.left
        right = config.spacemouse_cartesian_dual.right
        return DualSpaceMouseCartesianActionSource(
            left_reader=_LazyHidSpaceMouseReader(
                device=left.device,
                path=left.path,
                device_number=left.device_number,
            ),
            right_reader=_LazyHidSpaceMouseReader(
                device=right.device,
                path=right.path,
                device_number=right.device_number,
            ),
            frame=config.spacemouse_cartesian_dual.frame,
            max_linear_velocity_m_s=config.spacemouse_cartesian_dual.max_linear_velocity_m_s,
            max_angular_velocity_rad_s=config.spacemouse_cartesian_dual.max_angular_velocity_rad_s,
            deadband=config.spacemouse_cartesian_dual.deadband,
            left_deadman_button=left.deadman_button,
            right_deadman_button=right.deadman_button,
            timeout_sec=config.servo_command.timeout_sec,
        )
    raise ValueError(f"unknown action_source: {config.action_source}")


def _load_runtime_geometry_status(config: PolicyRunnerConfig) -> GeometryStatus:
    if not config.geometry.path:
        return GeometryStatus.unavailable("geometry_path_missing")
    return load_geometry_status(config.geometry.path)


def _close_if_supported(value: object) -> None:
    close = getattr(value, "close", None)
    if callable(close):
        close()


class _LazyHidSpaceMouseReader(SpaceMouseReader):
    def __init__(
        self,
        *,
        device: str | None = None,
        path: str | None = None,
        device_number: int = 0,
    ):
        self._device = device
        self._path = path
        self._device_number = device_number
        self._reader: HidSpaceMouseReader | None = None

    def read(self, timeout_sec: float | None = None) -> SpaceMouseSample | None:
        if self._reader is None:
            self._reader = HidSpaceMouseReader(
                device=self._device,
                path=self._path,
                device_number=self._device_number,
            )
        return self._reader.read(timeout_sec=timeout_sec)

    def close(self) -> None:
        if self._reader is not None:
            self._reader.close()
            self._reader = None
