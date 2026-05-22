from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class RobotStateConfig:
    bind: str = "udp://127.0.0.1:50120"
    stale_timeout_sec: float = 0.5


@dataclass(frozen=True)
class ServoCommandConfig:
    endpoint: str = "udp://127.0.0.1:50010"
    timeout_sec: float = 0.2


@dataclass(frozen=True)
class SafetyConfig:
    allow_real_motion: bool = False
    require_valid_joint_state: bool = True
    kinematics_available: bool = False
    camera_available: bool = False
    camera_stale: bool = False


@dataclass(frozen=True)
class JointSineConfig:
    selected_arm: str = "both"
    amplitude_deg: tuple[float, ...] = (1.0, 1.0, 1.0, 0.5, 0.5, 0.5)
    frequency_hz: float = 0.1
    simulation_only: bool = True


@dataclass(frozen=True)
class JointVelocityConfig:
    selected_arm: str = "both"
    velocity_deg_s: tuple[float, ...] = (1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    simulation_only: bool = True


@dataclass(frozen=True)
class SpaceMouseConfig:
    selected_arm: str = "left"
    max_joint_velocity_deg_s: tuple[float, ...] = (5.0, 5.0, 5.0, 8.0, 8.0, 10.0)
    deadband: float = 0.08
    smoothing_alpha: float = 0.2
    require_deadman: bool = True
    deadman_button: int = 0


@dataclass(frozen=True)
class PolicyRunnerConfig:
    schema: str = "robotics_lab.policy_runner.v1"
    mode: str = "simulation"
    action_source: str = "hold"
    robot_state: RobotStateConfig = field(default_factory=RobotStateConfig)
    servo_command: ServoCommandConfig = field(default_factory=ServoCommandConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    joint_sine: JointSineConfig = field(default_factory=JointSineConfig)
    joint_velocity: JointVelocityConfig = field(default_factory=JointVelocityConfig)
    spacemouse: SpaceMouseConfig = field(default_factory=SpaceMouseConfig)
    command_rate_hz: float = 30.0


def load_config(path: str | Path) -> PolicyRunnerConfig:
    raw = _load_mapping(Path(path))
    return config_from_mapping(raw)


def config_from_mapping(raw: dict[str, Any]) -> PolicyRunnerConfig:
    if raw.get("schema", "robotics_lab.policy_runner.v1") != "robotics_lab.policy_runner.v1":
        raise ValueError("unsupported policy_runner schema")
    return PolicyRunnerConfig(
        schema=str(raw.get("schema", "robotics_lab.policy_runner.v1")),
        mode=str(raw.get("mode", "simulation")),
        action_source=str(raw.get("action_source", "hold")),
        robot_state=RobotStateConfig(**_section(raw, "robot_state")),
        servo_command=ServoCommandConfig(**_section(raw, "servo_command")),
        safety=SafetyConfig(**_section(raw, "safety")),
        joint_sine=_joint_sine_config(_section(raw, "joint_sine")),
        joint_velocity=_joint_velocity_config(_section(raw, "joint_velocity")),
        spacemouse=_spacemouse_config(_section(raw, "spacemouse")),
        command_rate_hz=float(raw.get("command_rate_hz", 30.0)),
    )


def _section(raw: dict[str, Any], key: str) -> dict[str, Any]:
    value = raw.get(key, {})
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError(f"{key} must be a mapping")
    return dict(value)


def _joint_sine_config(raw: dict[str, Any]) -> JointSineConfig:
    if "amplitude_deg" in raw:
        raw["amplitude_deg"] = _tuple6(raw["amplitude_deg"], "joint_sine.amplitude_deg")
    return JointSineConfig(**raw)


def _joint_velocity_config(raw: dict[str, Any]) -> JointVelocityConfig:
    if "velocity_deg_s" in raw:
        raw["velocity_deg_s"] = _tuple6(raw["velocity_deg_s"], "joint_velocity.velocity_deg_s")
    return JointVelocityConfig(**raw)


def _spacemouse_config(raw: dict[str, Any]) -> SpaceMouseConfig:
    if "max_joint_velocity_deg_s" in raw:
        raw["max_joint_velocity_deg_s"] = _tuple6(
            raw["max_joint_velocity_deg_s"],
            "spacemouse.max_joint_velocity_deg_s",
        )
    return SpaceMouseConfig(**raw)


def _tuple6(value: Any, label: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        raise ValueError(f"{label} must contain 6 numbers")
    return tuple(float(v) for v in value)


def _load_mapping(path: Path) -> dict[str, Any]:
    text = path.read_text()
    try:
        import yaml  # type: ignore
    except ModuleNotFoundError:
        return _parse_simple_yaml(text)
    loaded = yaml.safe_load(text)
    if not isinstance(loaded, dict):
        raise ValueError(f"{path} must contain a mapping")
    return loaded


def _parse_simple_yaml(text: str) -> dict[str, Any]:
    root: dict[str, Any] = {}
    stack: list[tuple[int, dict[str, Any]]] = [(-1, root)]
    for raw_line in text.splitlines():
        line = raw_line.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip(" "))
        stripped = line.strip()
        if ":" not in stripped:
            raise ValueError(f"unsupported YAML line: {raw_line}")
        key, value = stripped.split(":", 1)
        key = key.strip()
        value = value.strip()
        while stack and indent <= stack[-1][0]:
            stack.pop()
        parent = stack[-1][1]
        if value == "":
            child: dict[str, Any] = {}
            parent[key] = child
            stack.append((indent, child))
        else:
            parent[key] = _parse_scalar(value)
    return root


def _parse_scalar(value: str) -> Any:
    if len(value) >= 2 and value[0] == value[-1] and value[0] in {'"', "'"}:
        return value[1:-1]
    if value in {"true", "false"}:
        return value == "true"
    if value == "null":
        return None
    if value.startswith("[") and value.endswith("]"):
        inner = value[1:-1].strip()
        if not inner:
            return []
        return [_parse_scalar(part.strip()) for part in inner.split(",")]
    try:
        if any(ch in value for ch in ".eE"):
            return float(value)
        return int(value)
    except ValueError:
        return value
