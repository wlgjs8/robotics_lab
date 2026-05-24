from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class RobotStateConfig:
    bind: str = "udp://127.0.0.1:50120"
    stale_timeout_sec: float = 0.5


@dataclass(frozen=True)
class RuntimeConfig:
    startup_timeout_sec: float = 5.0


@dataclass(frozen=True)
class ServoCommandConfig:
    endpoint: str = "udp://127.0.0.1:50010"
    timeout_sec: float = 0.2
    acquire_lease: bool = False
    lease_readback_timeout_sec: float = 1.0


@dataclass(frozen=True)
class SafetyConfig:
    allow_real_motion: bool = False
    require_valid_joint_state: bool = True
    kinematics_available: bool = False
    camera_available: bool = False
    camera_stale: bool = False
    camera_stale_timeout_sec: float = 0.5
    allow_configured_estimate_geometry_in_simulation: bool = True
    allow_configured_estimate_geometry_in_real: bool = False

    def __post_init__(self) -> None:
        if self.camera_stale_timeout_sec <= 0.0:
            raise ValueError("safety.camera_stale_timeout_sec must be positive")


@dataclass(frozen=True)
class GeometryConfig:
    path: str = "calibration/active_calibration.yaml"


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
class TcpDeltaConfig:
    selected_arm: str = "both"
    frame: str = "stand"
    delta: tuple[float, ...] = (0.001, 0.0, 0.0, 0.0, 0.0, 0.0)
    max_linear_step_m: float = 0.002
    max_angular_step_rad: float = 0.01
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
class SpaceMouseCartesianConfig:
    selected_arm: str = "left"
    frame: str = "stand"
    command_rate_hz: float = 30.0
    max_linear_step_m: float = 0.002
    max_angular_step_rad: float = 0.01
    deadband: float = 0.08
    require_deadman: bool = True
    deadman_button: int = 0


@dataclass(frozen=True)
class PolicyRunnerConfig:
    schema: str = "robotics_lab.policy_runner.v1"
    mode: str = "simulation"
    action_source: str = "hold"
    runtime: RuntimeConfig = field(default_factory=RuntimeConfig)
    geometry: GeometryConfig = field(default_factory=GeometryConfig)
    robot_state: RobotStateConfig = field(default_factory=RobotStateConfig)
    servo_command: ServoCommandConfig = field(default_factory=ServoCommandConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    joint_sine: JointSineConfig = field(default_factory=JointSineConfig)
    joint_velocity: JointVelocityConfig = field(default_factory=JointVelocityConfig)
    tcp_delta: TcpDeltaConfig = field(default_factory=TcpDeltaConfig)
    spacemouse: SpaceMouseConfig = field(default_factory=SpaceMouseConfig)
    spacemouse_cartesian: SpaceMouseCartesianConfig = field(default_factory=SpaceMouseCartesianConfig)
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
        runtime=_runtime_config(_section(raw, "runtime")),
        geometry=GeometryConfig(**_section(raw, "geometry")),
        robot_state=RobotStateConfig(**_section(raw, "robot_state")),
        servo_command=_servo_command_config(_section(raw, "servo_command")),
        safety=_safety_config(_section(raw, "safety")),
        joint_sine=_joint_sine_config(_section(raw, "joint_sine")),
        joint_velocity=_joint_velocity_config(_section(raw, "joint_velocity")),
        tcp_delta=_tcp_delta_config(_section(raw, "tcp_delta")),
        spacemouse=_spacemouse_config(_section(raw, "spacemouse")),
        spacemouse_cartesian=_spacemouse_cartesian_config(_section(raw, "spacemouse_cartesian")),
        command_rate_hz=float(raw.get("command_rate_hz", 30.0)),
    )


def _section(raw: dict[str, Any], key: str) -> dict[str, Any]:
    value = raw.get(key, {})
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError(f"{key} must be a mapping")
    return dict(value)


def _runtime_config(raw: dict[str, Any]) -> RuntimeConfig:
    if "startup_timeout_sec" in raw:
        raw["startup_timeout_sec"] = float(raw["startup_timeout_sec"])
    return RuntimeConfig(**raw)


def _servo_command_config(raw: dict[str, Any]) -> ServoCommandConfig:
    if "timeout_sec" in raw:
        raw["timeout_sec"] = float(raw["timeout_sec"])
    if "lease_readback_timeout_sec" in raw:
        raw["lease_readback_timeout_sec"] = float(raw["lease_readback_timeout_sec"])
    config = ServoCommandConfig(**raw)
    if config.timeout_sec <= 0.0:
        raise ValueError("servo_command.timeout_sec must be positive")
    if config.lease_readback_timeout_sec <= 0.0:
        raise ValueError("servo_command.lease_readback_timeout_sec must be positive")
    return config


def _safety_config(raw: dict[str, Any]) -> SafetyConfig:
    if "camera_stale_timeout_sec" in raw:
        raw["camera_stale_timeout_sec"] = float(raw["camera_stale_timeout_sec"])
    return SafetyConfig(**raw)


def _joint_sine_config(raw: dict[str, Any]) -> JointSineConfig:
    if "amplitude_deg" in raw:
        raw["amplitude_deg"] = _tuple6(raw["amplitude_deg"], "joint_sine.amplitude_deg")
    return JointSineConfig(**raw)


def _joint_velocity_config(raw: dict[str, Any]) -> JointVelocityConfig:
    if "velocity_deg_s" in raw:
        raw["velocity_deg_s"] = _tuple6(raw["velocity_deg_s"], "joint_velocity.velocity_deg_s")
    return JointVelocityConfig(**raw)


def _tcp_delta_config(raw: dict[str, Any]) -> TcpDeltaConfig:
    if "delta" in raw:
        raw["delta"] = _tuple6(raw["delta"], "tcp_delta.delta")
    return TcpDeltaConfig(**raw)


def _spacemouse_config(raw: dict[str, Any]) -> SpaceMouseConfig:
    if "max_joint_velocity_deg_s" in raw:
        raw["max_joint_velocity_deg_s"] = _tuple6(
            raw["max_joint_velocity_deg_s"],
            "spacemouse.max_joint_velocity_deg_s",
        )
    return SpaceMouseConfig(**raw)


def _spacemouse_cartesian_config(raw: dict[str, Any]) -> SpaceMouseCartesianConfig:
    return SpaceMouseCartesianConfig(**raw)


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
