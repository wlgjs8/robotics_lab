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
    allow_real_gripper_motion: bool = False
    allow_rbpodo_controller_simulation_cartesian: bool = False
    allow_configured_estimate_geometry_in_controller_simulation: bool = True
    selected_arm: str = "both"
    selected_arms: tuple[str, ...] = ()
    retarget_status: str = "missing"
    collision_model_status: str = "missing"
    minimum_inter_arm_distance_m: float | None = None
    workspace_envelope_status: str = "missing"
    measured_retarget_available: bool = False
    measured_collision_model_available: bool = False
    measured_gripper_available: bool = False
    allow_selected_arm_checkpoint_mismatch_readonly: bool = False
    require_valid_joint_state: bool = True
    kinematics_available: bool = False
    camera_available: bool = False
    camera_stale: bool = False
    camera_stale_timeout_sec: float = 0.5
    allow_configured_estimate_geometry_in_simulation: bool = True
    allow_configured_estimate_geometry_in_real: bool = False

    def __post_init__(self) -> None:
        if self.selected_arm not in {"left", "right", "both"}:
            raise ValueError("safety.selected_arm must be left, right, or both")
        invalid_arms = [arm for arm in self.selected_arms if arm not in {"left", "right"}]
        if invalid_arms:
            raise ValueError("safety.selected_arms entries must be left or right")
        if self.retarget_status not in {"missing", "configured_estimate", "measured", "accepted"}:
            raise ValueError(
                "safety.retarget_status must be missing, configured_estimate, measured, or accepted"
            )
        if self.collision_model_status not in {"missing", "configured_estimate", "measured", "validated"}:
            raise ValueError(
                "safety.collision_model_status must be missing, configured_estimate, measured, or validated"
            )
        if self.workspace_envelope_status not in {"missing", "configured_estimate", "measured", "validated"}:
            raise ValueError(
                "safety.workspace_envelope_status must be missing, configured_estimate, measured, or validated"
            )
        if self.minimum_inter_arm_distance_m is not None and self.minimum_inter_arm_distance_m < 0.0:
            raise ValueError("safety.minimum_inter_arm_distance_m must be non-negative")
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
    command_rate_hz: float = 500.0
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
    linear_axis_signs: tuple[float, ...] = (1.0, 1.0, 1.0)
    angular_axis_signs: tuple[float, ...] = (1.0, 1.0, 1.0)
    angular_axis_order: tuple[str, ...] = ("rx", "ry", "rz")
    sample_hold_timeout_sec: float = 0.05
    # Buttonless teleop: require_deadman=False replaces the button gate with an
    # axis-intent gate (cap deflection beyond activation_deadband starts motion;
    # neutral sends one zero twist). Startup/reconnect requires the cap held
    # neutral for startup_neutral_hold_sec first.
    require_deadman: bool = True
    activation_deadband: float | None = None
    startup_requires_neutral: bool = True
    startup_neutral_hold_sec: float = 0.3

    def __post_init__(self) -> None:
        if self.max_linear_velocity_m_s < 0.0:
            raise ValueError("spacemouse_cartesian_dual.max_linear_velocity_m_s must be non-negative")
        if self.max_angular_velocity_rad_s < 0.0:
            raise ValueError("spacemouse_cartesian_dual.max_angular_velocity_rad_s must be non-negative")
        if self.deadband < 0.0:
            raise ValueError("spacemouse_cartesian_dual.deadband must be non-negative")
        if self.response_curve_gamma < 1.0:
            raise ValueError("spacemouse_cartesian_dual.response_curve_gamma must be >= 1.0")
        for name in ("linear_axis_signs", "angular_axis_signs"):
            signs = getattr(self, name)
            if len(signs) != 3 or any(sign not in (-1.0, 1.0) for sign in signs):
                raise ValueError(f"spacemouse_cartesian_dual.{name} must be 3 entries of -1 or 1")
        if sorted(str(axis).lower() for axis in self.angular_axis_order) != ["rx", "ry", "rz"]:
            raise ValueError(
                "spacemouse_cartesian_dual.angular_axis_order must be a permutation of rx/ry/rz"
            )
        if self.activation_deadband is not None and self.activation_deadband < 0.0:
            raise ValueError("spacemouse_cartesian_dual.activation_deadband must be non-negative")
        if self.startup_neutral_hold_sec < 0.0:
            raise ValueError("spacemouse_cartesian_dual.startup_neutral_hold_sec must be non-negative")
        if self.sample_hold_timeout_sec <= 0.0:
            raise ValueError("spacemouse_cartesian_dual.sample_hold_timeout_sec must be positive")

    @property
    def max_linear_step_m(self) -> float:
        return self.max_linear_velocity_m_s

    @property
    def max_angular_step_rad(self) -> float:
        return self.max_angular_velocity_rad_s


@dataclass(frozen=True)
class UmiPoseReaderConfig:
    udp_endpoint: str | None = None
    bind: str | None = None
    mock_script: str | tuple[dict[str, Any] | None, ...] | None = None

    def __post_init__(self) -> None:
        if self.mock_script is not None and not isinstance(self.mock_script, (str, tuple)):
            raise ValueError("umi mock_script must be a script name or a list of samples")
        if self.udp_endpoint is not None and self.bind is not None:
            raise ValueError("umi reader must not set both udp_endpoint and bind")

    @property
    def endpoint(self) -> str | None:
        return self.udp_endpoint or self.bind


@dataclass(frozen=True)
class UmiDualCartesianConfig:
    left: UmiPoseReaderConfig = field(
        default_factory=lambda: UmiPoseReaderConfig(mock_script="pgmode_umi_smoke")
    )
    right: UmiPoseReaderConfig = field(
        default_factory=lambda: UmiPoseReaderConfig(mock_script="pgmode_umi_smoke")
    )
    max_linear_step_m: float = 0.005
    max_angular_step_rad: float = 0.04
    input_moving_average_window: int = 1
    target_lpf_tau_sec: float = 0.0
    deadband_linear_m: float = 0.0
    deadband_angular_rad: float = 0.0
    linear_axis_signs: tuple[float, ...] = (1.0, 1.0, 1.0)
    angular_axis_signs: tuple[float, ...] = (1.0, 1.0, 1.0)
    delta_frame: str = "tool"
    # The pika publisher streams the official gripper-tip pose by default
    # (--pose-frame tip), so the receiver adds no further offset; see
    # stack_real.yaml for the paired r_align/axis-sign geometry.
    gripper_offset: tuple[float, ...] = (0.0, 0.0, 0.0)
    r_align: tuple[float, ...] = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    workspace_bounds: dict[str, tuple[float, float]] | tuple[float, ...] | None = None
    sample_hold_timeout_sec: float = 0.05

    def __post_init__(self) -> None:
        if self.max_linear_step_m < 0.0:
            raise ValueError("umi_dual_cartesian.max_linear_step_m must be non-negative")
        if self.max_angular_step_rad < 0.0:
            raise ValueError("umi_dual_cartesian.max_angular_step_rad must be non-negative")
        if self.input_moving_average_window < 0:
            raise ValueError("umi_dual_cartesian.input_moving_average_window must be non-negative")
        if self.target_lpf_tau_sec < 0.0:
            raise ValueError("umi_dual_cartesian.target_lpf_tau_sec must be non-negative")
        if self.deadband_linear_m < 0.0:
            raise ValueError("umi_dual_cartesian.deadband_linear_m must be non-negative")
        if self.deadband_angular_rad < 0.0:
            raise ValueError("umi_dual_cartesian.deadband_angular_rad must be non-negative")
        for name in ("linear_axis_signs", "angular_axis_signs"):
            signs = getattr(self, name)
            if len(signs) != 3 or any(sign not in (-1.0, 1.0) for sign in signs):
                raise ValueError(f"umi_dual_cartesian.{name} must contain 3 entries of -1 or 1")
        if self.delta_frame not in ("tool", "world"):
            raise ValueError("umi_dual_cartesian.delta_frame must be 'tool' or 'world'")
        if len(self.gripper_offset) != 3:
            raise ValueError("umi_dual_cartesian.gripper_offset must contain 3 values")
        if len(self.r_align) not in {3, 9}:
            raise ValueError("umi_dual_cartesian.r_align must contain 3 RPY values or 9 matrix values")
        if self.sample_hold_timeout_sec <= 0.0:
            raise ValueError("umi_dual_cartesian.sample_hold_timeout_sec must be positive")


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
    umi_dual_cartesian: UmiDualCartesianConfig = field(default_factory=UmiDualCartesianConfig)
    master_arm_joint: MasterArmJointConfig = field(default_factory=MasterArmJointConfig)
    command_rate_hz: float = 500.0

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
        umi_dual_cartesian=_umi_dual_cartesian_config(_section(raw, "umi_dual_cartesian")),
        master_arm_joint=_master_arm_joint_config(_section(raw, "master_arm_joint")),
        command_rate_hz=float(raw.get("command_rate_hz", 500.0)),
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
    if "selected_arms" in raw:
        value = raw["selected_arms"]
        if not isinstance(value, (list, tuple)):
            raise ValueError("safety.selected_arms must be a list")
        raw["selected_arms"] = tuple(str(item) for item in value)
    if "selected_arm" in raw:
        raw["selected_arm"] = str(raw["selected_arm"])
    if "retarget_status" in raw:
        raw["retarget_status"] = str(raw["retarget_status"])
    if "collision_model_status" in raw:
        raw["collision_model_status"] = str(raw["collision_model_status"])
    if "workspace_envelope_status" in raw:
        raw["workspace_envelope_status"] = str(raw["workspace_envelope_status"])
    if "minimum_inter_arm_distance_m" in raw and raw["minimum_inter_arm_distance_m"] is not None:
        raw["minimum_inter_arm_distance_m"] = float(raw["minimum_inter_arm_distance_m"])
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
    for key in ("linear_axis_signs", "angular_axis_signs"):
        if key in top_level:
            top_level[key] = tuple(float(v) for v in top_level[key])
    if "angular_axis_order" in top_level:
        top_level["angular_axis_order"] = tuple(
            str(axis).lower() for axis in top_level["angular_axis_order"]
        )
    if "require_deadman" in top_level:
        top_level["require_deadman"] = bool(top_level["require_deadman"])
    if "activation_deadband" in top_level and top_level["activation_deadband"] is not None:
        top_level["activation_deadband"] = float(top_level["activation_deadband"])
    if "startup_requires_neutral" in top_level:
        top_level["startup_requires_neutral"] = bool(top_level["startup_requires_neutral"])
    if "startup_neutral_hold_sec" in top_level:
        top_level["startup_neutral_hold_sec"] = float(top_level["startup_neutral_hold_sec"])
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


def _umi_dual_cartesian_config(raw: dict[str, Any]) -> UmiDualCartesianConfig:
    left = _umi_reader_config(_section(raw, "left"))
    right = _umi_reader_config(_section(raw, "right"))
    top_level = {key: value for key, value in raw.items() if key not in {"left", "right"}}
    if "max_linear_step_m" in top_level:
        top_level["max_linear_step_m"] = float(top_level["max_linear_step_m"])
    if "max_angular_step_rad" in top_level:
        top_level["max_angular_step_rad"] = float(top_level["max_angular_step_rad"])
    if "input_moving_average_window" in top_level:
        top_level["input_moving_average_window"] = int(top_level["input_moving_average_window"])
    for key in ("target_lpf_tau_sec", "deadband_linear_m", "deadband_angular_rad"):
        if key in top_level:
            top_level[key] = float(top_level[key])
    for key in ("linear_axis_signs", "angular_axis_signs"):
        if key in top_level:
            top_level[key] = _tuple3(top_level[key], f"umi_dual_cartesian.{key}")
    if "delta_frame" in top_level:
        top_level["delta_frame"] = str(top_level["delta_frame"])
    if "sample_hold_timeout_sec" in top_level:
        top_level["sample_hold_timeout_sec"] = float(top_level["sample_hold_timeout_sec"])
    if "gripper_offset" in top_level:
        top_level["gripper_offset"] = _tuple3(top_level["gripper_offset"], "umi_dual_cartesian.gripper_offset")
    if "r_align" in top_level:
        value = top_level["r_align"]
        if not isinstance(value, (list, tuple)) or len(value) not in {3, 9}:
            raise ValueError("umi_dual_cartesian.r_align must contain 3 or 9 numbers")
        top_level["r_align"] = tuple(float(item) for item in value)
    if "workspace_bounds" in top_level:
        top_level["workspace_bounds"] = _umi_workspace_bounds(top_level["workspace_bounds"])
    return UmiDualCartesianConfig(left=left, right=right, **top_level)


def _umi_reader_config(raw: dict[str, Any]) -> UmiPoseReaderConfig:
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
                    raise ValueError("umi mock_script entries must be mappings or null")
            raw["mock_script"] = tuple(samples)
        elif value is not None and not isinstance(value, str):
            raise ValueError("umi mock_script must be a script name or a list of samples")
    if "bind" in raw and raw.get("udp_endpoint") is None:
        raw["bind"] = str(raw["bind"]) if raw["bind"] is not None else None
    if "udp_endpoint" in raw and raw["udp_endpoint"] is not None:
        raw["udp_endpoint"] = str(raw["udp_endpoint"])
    return UmiPoseReaderConfig(**raw)


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


def _tuple3(value: Any, label: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{label} must contain 3 numbers")
    return tuple(float(v) for v in value)


def _umi_workspace_bounds(value: Any) -> dict[str, tuple[float, float]] | tuple[float, ...] | None:
    if value is None:
        return None
    if isinstance(value, dict):
        out: dict[str, tuple[float, float]] = {}
        for axis in ("x", "y", "z"):
            if axis not in value:
                continue
            raw_pair = value[axis]
            if not isinstance(raw_pair, (list, tuple)) or len(raw_pair) != 2:
                raise ValueError(f"umi_dual_cartesian.workspace_bounds.{axis} must contain [min,max]")
            out[axis] = (float(raw_pair[0]), float(raw_pair[1]))
        return out
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        raise ValueError("umi_dual_cartesian.workspace_bounds must be a mapping or 6-number list")
    return tuple(float(item) for item in value)


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
