from __future__ import annotations

import argparse
import time

from .action_sources import (
    HoldActionSource,
    JointSineActionSource,
    JointVelocityActionSource,
)
from .config import PolicyRunnerConfig, load_config
from .robot_state_client import RobotStateClient
from .safety import SafetyGate
from .servo_command_client import CommandIntent, ServoCommandClient


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="robotics_lab policy_runner")
    parser.add_argument("--config", required=True, help="policy_runner YAML config")
    args = parser.parse_args(argv)
    config = load_config(args.config)
    return run(config)


def run(config: PolicyRunnerConfig) -> int:
    state_client = RobotStateClient(config.robot_state.bind, config.robot_state.stale_timeout_sec)
    command_client = ServoCommandClient(config.servo_command.endpoint, config.servo_command.timeout_sec)
    source = make_action_source(config)
    safety_gate = SafetyGate(config.mode, config.safety, config.robot_state.stale_timeout_sec)
    state_client.start()
    armed_for_motion = False
    period = 1.0 / max(config.command_rate_hz, 1.0)
    try:
        while True:
            snapshot = state_client.latest
            now = time.monotonic()
            if snapshot is None:
                time.sleep(period)
                continue
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
            time.sleep(period)
    except KeyboardInterrupt:
        return 0
    finally:
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
    raise ValueError(f"unknown action_source: {config.action_source}")
