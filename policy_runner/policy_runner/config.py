from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any
import warnings


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
    allow_rbpodo_controller_simulation_cartesian: bool = False
    allow_configured_estimate_geometry_in_controller_simulation: bool = True
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
class RecordingConfig:
    output_dir: str = "/data/episodes"
    rate_hz: float = 30.0
    format: str = "hdf5"
    dataset_metadata: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.rate_hz < 1.0 or self.rate_hz > 100.0:
            raise ValueError("recording.rate_hz must be in [1.0, 100.0]")
        if self.format not in {"hdf5", "jsonl"}:
            raise ValueError(
                f"recording.format must be 'hdf5' or 'jsonl', got: {self.format}"
            )


@dataclass(frozen=True)
class CameraConfig:
    enable: bool = False
    zmq_endpoint: str = "tcp://127.0.0.1:5600"
    bundle_topic: str = "camera.bundle"
    max_age_ms: float = 100.0
    expected_cameras: list[str] = field(default_factory=list)
    record_zero_on_missing: bool = True

    def __post_init__(self) -> None:
        if self.max_age_ms <= 0.0:
            raise ValueError("camera.max_age_ms must be positive")


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
    frame: str = "local"
    command_rate_hz: float = 100.0
    max_linear_velocity_m_s: float = 0.03
    max_angular_velocity_rad_s: float = 0.2
    deadband: float = 0.08
    response_curve_gamma: float = 3.0
    sample_hold_timeout_sec: float = 0.05
    require_deadman: bool = True
    deadman_button: int = 0

    def __post_init__(self) -> None:
        if self.max_linear_velocity_m_s < 0.0:
            raise ValueError("spacemouse_cartesian.max_linear_velocity_m_s must be non-negative")
        if self.max_angular_velocity_rad_s < 0.0:
            raise ValueError("spacemouse_cartesian.max_angular_velocity_rad_s must be non-negative")
        if self.deadband < 0.0:
            raise ValueError("spacemouse_cartesian.deadband must be non-negative")
        if self.response_curve_gamma < 1.0:
            raise ValueError("spacemouse_cartesian.response_curve_gamma must be >= 1.0")
        if self.sample_hold_timeout_sec <= 0.0:
            raise ValueError("spacemouse_cartesian.sample_hold_timeout_sec must be positive")
        _validate_command_rate_hz(float(self.command_rate_hz))

    @property
    def max_linear_step_m(self) -> float:
        return self.max_linear_velocity_m_s

    @property
    def max_angular_step_rad(self) -> float:
        return self.max_angular_velocity_rad_s


@dataclass(frozen=True)
class SpaceMouseDeviceConfig:
    device: str | None = None
    path: str | None = None
    device_number: int = 0
    deadman_button: int = 0
    mock_script: str | tuple[dict[str, Any] | None, ...] | None = None

    def __post_init__(self) -> None:
        if self.device_number < 0:
            raise ValueError("spacemouse device_number must be non-negative")
        if self.deadman_button < 0:
            raise ValueError("spacemouse deadman_button must be non-negative")
        if self.mock_script is not None and not isinstance(self.mock_script, (str, tuple)):
            raise ValueError("spacemouse mock_script must be a script name or a list of samples")


@dataclass(frozen=True)
class DualSpaceMouseCartesianConfig:
    left: SpaceMouseDeviceConfig = field(default_factory=SpaceMouseDeviceConfig)
    right: SpaceMouseDeviceConfig = field(
        default_factory=lambda: SpaceMouseDeviceConfig(device_number=1)
    )
    frame: str = "local"
    max_linear_velocity_m_s: float = 0.03
    max_angular_velocity_rad_s: float = 0.2
    deadband: float = 0.08
    response_curve_gamma: float = 3.0
    sample_hold_timeout_sec: float = 0.05

    def __post_init__(self) -> None:
        if self.max_linear_velocity_m_s < 0.0:
            raise ValueError("spacemouse_cartesian_dual.max_linear_velocity_m_s must be non-negative")
        if self.max_angular_velocity_rad_s < 0.0:
            raise ValueError("spacemouse_cartesian_dual.max_angular_velocity_rad_s must be non-negative")
        if self.deadband < 0.0:
            raise ValueError("spacemouse_cartesian_dual.deadband must be non-negative")
        if self.response_curve_gamma < 1.0:
            raise ValueError("spacemouse_cartesian_dual.response_curve_gamma must be >= 1.0")
        if self.sample_hold_timeout_sec <= 0.0:
            raise ValueError("spacemouse_cartesian_dual.sample_hold_timeout_sec must be positive")

    @property
    def max_linear_step_m(self) -> float:
        return self.max_linear_velocity_m_s

    @property
    def max_angular_step_rad(self) -> float:
        return self.max_angular_velocity_rad_s


@dataclass(frozen=True)
class MasterArmJointConfig:
    config_path: str = ""
    python_module_dir: str = ""
    module_name: str = "_mo_master_arm_py"
    selected_arm: str = "both"
    mapping_mode: str = "delta"
    robot_init_strategy: str = "current"
    deadman_side: str = "either"
    deadman_switch: int = 2
    require_deadman: bool = True
    use_gravity_compensation: bool = True
    hold_master_on_release: bool = True
    enable_gripper_readers: bool = False
    left_scale: tuple[float, ...] = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
    right_scale: tuple[float, ...] = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
    left_sign: tuple[float, ...] = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
    right_sign: tuple[float, ...] = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
    left_offset_deg: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    right_offset_deg: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    left_robot_init_deg: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    right_robot_init_deg: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    left_joint_map: tuple[int, ...] = (0, 1, 2, 3, 4, 5)
    right_joint_map: tuple[int, ...] = (0, 1, 2, 3, 4, 5)
    max_joint_velocity_deg_s: tuple[float, ...] = (30.0, 30.0, 30.0, 45.0, 45.0, 60.0)
    smoothing_alpha: float = 1.0
    wrap_delta: bool = True

    def __post_init__(self) -> None:
        if self.selected_arm not in {"left", "right", "both"}:
            raise ValueError("master_arm_joint.selected_arm must be left, right, or both")
        if self.mapping_mode not in {"delta", "absolute"}:
            raise ValueError("master_arm_joint.mapping_mode must be delta or absolute")
        if self.robot_init_strategy not in {"current", "configured", "master_absolute"}:
            raise ValueError("master_arm_joint.robot_init_strategy must be current, configured, or master_absolute")
        if self.deadman_side not in {"left", "right", "either", "both", "none"}:
            raise ValueError("master_arm_joint.deadman_side must be left, right, either, both, or none")
        if self.deadman_switch < 0:
            raise ValueError("master_arm_joint.deadman_switch must be non-negative")
        if not 0.0 <= self.smoothing_alpha <= 1.0:
            raise ValueError("master_arm_joint.smoothing_alpha must be in [0, 1]")


@dataclass(frozen=True)
class PolicyRunnerConfig:
    schema: str = "robotics_lab.policy_runner.v1"
    mode: str = "simulation"
    action_source: str = "hold"
    runtime: RuntimeConfig = field(default_factory=RuntimeConfig)
    geometry: GeometryConfig = field(default_factory=GeometryConfig)
    recording: RecordingConfig = field(default_factory=RecordingConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    robot_state: RobotStateConfig = field(default_factory=RobotStateConfig)
    servo_command: ServoCommandConfig = field(default_factory=ServoCommandConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    joint_sine: JointSineConfig = field(default_factory=JointSineConfig)
    joint_velocity: JointVelocityConfig = field(default_factory=JointVelocityConfig)
    tcp_delta: TcpDeltaConfig = field(default_factory=TcpDeltaConfig)
    spacemouse: SpaceMouseConfig = field(default_factory=SpaceMouseConfig)
    spacemouse_cartesian: SpaceMouseCartesianConfig = field(default_factory=SpaceMouseCartesianConfig)
    spacemouse_cartesian_dual: DualSpaceMouseCartesianConfig = field(
        default_factory=DualSpaceMouseCartesianConfig
    )
    master_arm_joint: MasterArmJointConfig = field(default_factory=MasterArmJointConfig)
    command_rate_hz: float = 100.0

    def __post_init__(self) -> None:
        _validate_command_rate_hz(float(self.command_rate_hz))

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> "PolicyRunnerConfig":
        return config_from_mapping(raw)


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
        recording=_recording_config(_section(raw, "recording")),
        camera=_camera_config(_section(raw, "camera")),
        robot_state=RobotStateConfig(**_section(raw, "robot_state")),
        servo_command=_servo_command_config(_section(raw, "servo_command")),
        safety=_safety_config(_section(raw, "safety")),
        joint_sine=_joint_sine_config(_section(raw, "joint_sine")),
        joint_velocity=_joint_velocity_config(_section(raw, "joint_velocity")),
        tcp_delta=_tcp_delta_config(_section(raw, "tcp_delta")),
        spacemouse=_spacemouse_config(_section(raw, "spacemouse")),
        spacemouse_cartesian=_spacemouse_cartesian_config(_section(raw, "spacemouse_cartesian")),
        spacemouse_cartesian_dual=_spacemouse_cartesian_dual_config(
            _section(raw, "spacemouse_cartesian_dual")
        ),
        master_arm_joint=_master_arm_joint_config(_section(raw, "master_arm_joint")),
        command_rate_hz=float(raw.get("command_rate_hz", 100.0)),
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


def _recording_config(raw: dict[str, Any]) -> RecordingConfig:
    if "rate_hz" in raw:
        raw["rate_hz"] = float(raw["rate_hz"])
    if "format" in raw:
        raw["format"] = str(raw["format"])
    if "output_dir" in raw:
        raw["output_dir"] = str(raw["output_dir"])
    if "dataset_metadata" in raw:
        value = raw["dataset_metadata"]
        if not isinstance(value, dict):
            raise ValueError("recording.dataset_metadata must be a mapping")
        raw["dataset_metadata"] = dict(value)
    return RecordingConfig(**raw)


def _camera_config(raw: dict[str, Any]) -> CameraConfig:
    if "max_age_ms" in raw:
        raw["max_age_ms"] = float(raw["max_age_ms"])
    if "expected_cameras" in raw:
        value = raw["expected_cameras"]
        if not isinstance(value, list):
            raise ValueError("camera.expected_cameras must be a list")
        raw["expected_cameras"] = [str(item) for item in value]
    return CameraConfig(**raw)


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
    _apply_spacemouse_cartesian_velocity_aliases(raw, "spacemouse_cartesian")
    if "command_rate_hz" in raw:
        raw["command_rate_hz"] = float(raw["command_rate_hz"])
    if "response_curve_gamma" in raw:
        raw["response_curve_gamma"] = float(raw["response_curve_gamma"])
    if "sample_hold_timeout_sec" in raw:
        raw["sample_hold_timeout_sec"] = float(raw["sample_hold_timeout_sec"])
    return SpaceMouseCartesianConfig(**raw)


def _spacemouse_cartesian_dual_config(raw: dict[str, Any]) -> DualSpaceMouseCartesianConfig:
    left = _spacemouse_device_config(_section(raw, "left"))
    right_raw = _section(raw, "right")
    if "device_number" not in right_raw:
        right_raw["device_number"] = 1
    right = _spacemouse_device_config(right_raw)
    top_level = {key: value for key, value in raw.items() if key not in {"left", "right"}}
    _apply_spacemouse_cartesian_velocity_aliases(top_level, "spacemouse_cartesian_dual")
    if "response_curve_gamma" in top_level:
        top_level["response_curve_gamma"] = float(top_level["response_curve_gamma"])
    if "sample_hold_timeout_sec" in top_level:
        top_level["sample_hold_timeout_sec"] = float(top_level["sample_hold_timeout_sec"])
    return DualSpaceMouseCartesianConfig(left=left, right=right, **top_level)


def _spacemouse_device_config(raw: dict[str, Any]) -> SpaceMouseDeviceConfig:
    if "device_number" in raw:
        raw["device_number"] = int(raw["device_number"])
    if "deadman_button" in raw:
        raw["deadman_button"] = int(raw["deadman_button"])
    if "mock_script" in raw:
        value = raw["mock_script"]
        if isinstance(value, list):
            samples: list[dict[str, Any] | None] = []
            for item in value:
                if item is None:
                    samples.append(None)
                elif isinstance(item, dict):
                    samples.append(dict(item))
                else:
                    raise ValueError("spacemouse mock_script entries must be mappings or null")
            raw["mock_script"] = tuple(samples)
        elif value is not None and not isinstance(value, str):
            raise ValueError("spacemouse mock_script must be a script name or a list of samples")
    return SpaceMouseDeviceConfig(**raw)


def _master_arm_joint_config(raw: dict[str, Any]) -> MasterArmJointConfig:
    tuple6_keys = {
        "left_scale",
        "right_scale",
        "left_sign",
        "right_sign",
        "left_offset_deg",
        "right_offset_deg",
        "left_robot_init_deg",
        "right_robot_init_deg",
        "max_joint_velocity_deg_s",
    }
    for key in tuple6_keys:
        if key in raw:
            raw[key] = _tuple6(raw[key], f"master_arm_joint.{key}")
    for key in ("left_joint_map", "right_joint_map"):
        if key in raw:
            raw[key] = _tuple6_int(raw[key], f"master_arm_joint.{key}")
            if sorted(raw[key]) != [0, 1, 2, 3, 4, 5]:
                raise ValueError(f"master_arm_joint.{key} must be a permutation of 0..5")
    for key in ("deadman_switch",):
        if key in raw:
            raw[key] = int(raw[key])
    for key in ("smoothing_alpha",):
        if key in raw:
            raw[key] = float(raw[key])
    return MasterArmJointConfig(**raw)


def _apply_spacemouse_cartesian_velocity_aliases(raw: dict[str, Any], section: str) -> None:
    aliases = (
        ("max_linear_step_m", "max_linear_velocity_m_s"),
        ("max_angular_step_rad", "max_angular_velocity_rad_s"),
    )
    for old_key, new_key in aliases:
        if old_key not in raw:
            continue
        value = raw.pop(old_key)
        warnings.warn(
            f"{section}.{old_key} is deprecated; use {section}.{new_key} "
            "because TcpTwistLocal/Stand fields are velocities, not one-shot deltas",
            DeprecationWarning,
            stacklevel=3,
        )
        raw.setdefault(new_key, value)


def _tuple6(value: Any, label: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        raise ValueError(f"{label} must contain 6 numbers")
    return tuple(float(v) for v in value)


def _tuple6_int(value: Any, label: str) -> tuple[int, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        raise ValueError(f"{label} must contain 6 integers")
    return tuple(int(v) for v in value)


def _validate_command_rate_hz(command_rate_hz: float) -> None:
    if command_rate_hz < 1.0 or command_rate_hz > 500.0:
        raise ValueError("command_rate_hz must be in [1.0, 500.0]")


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
