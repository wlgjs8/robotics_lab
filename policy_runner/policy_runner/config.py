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
    control_enabled: bool = True
    control_bind: str = "udp://127.0.0.1:50441"
    status_endpoint: str | None = "udp://127.0.0.1:50442"
    status_rate_hz: float = 10.0

    def __post_init__(self) -> None:
        if self.rate_hz < 1.0 or self.rate_hz > 100.0:
            raise ValueError("recording.rate_hz must be in [1.0, 100.0]")
        if self.format not in {"hdf5", "jsonl"}:
            raise ValueError(
                f"recording.format must be 'hdf5' or 'jsonl', got: {self.format}"
            )
        if self.status_rate_hz < 1.0 or self.status_rate_hz > 100.0:
            raise ValueError("recording.status_rate_hz must be in [1.0, 100.0]")


@dataclass(frozen=True)
class ArmInitOverrideConfig:
    auto_clear_on_done: bool = True
    auto_clear_on_failed: bool = False
    resume_flow_on_done: bool = True
    resume_flow_on_failed: bool = False
    allow_manual_cancel_after_failed: bool = True
    reset_flow_source_on_start: bool = True
    reset_flow_source_on_resume: bool = True


@dataclass(frozen=True)
class CameraConfig:
    enable: bool = False
    zmq_endpoint: str = "tcp://127.0.0.1:5600"
    bundle_topic: str = "camera.bundle"
    max_age_ms: float = 100.0
    expected_cameras: list[str] = field(default_factory=list)
    record_zero_on_missing: bool = True
    # Which physical camera feeds the checkpoint's left/right_wrist_0_rgb inputs.
    # "realsense" -> left/right_realsense_color; "fisheye" -> left/right_fisheye_color
    # (the fe65 deploy). Only consumed by the openpi remote source.
    wrist_source: str = "realsense"
    # Center-crop fraction applied to each wrist frame before inference (openpi remote
    # only). 0.0 = off; 0.65 reproduces the fisheye fe65 training crop (640x480 -> 416x312).
    wrist_crop_frac: float = 0.0
    # OpenPI physical-motion camera guard. A zero bundle count keeps the guard
    # disabled for non-real/offline profiles; real flow-infer opts in explicitly.
    readiness_bundle_count: int = 0
    readiness_timeout_sec: float = 1.0
    stale_timeout_sec: float = 1.0

    def __post_init__(self) -> None:
        if self.max_age_ms <= 0.0:
            raise ValueError("camera.max_age_ms must be positive")
        if self.wrist_source not in ("realsense", "fisheye"):
            raise ValueError("camera.wrist_source must be 'realsense' or 'fisheye'")
        if not 0.0 <= self.wrist_crop_frac <= 1.0:
            raise ValueError("camera.wrist_crop_frac must be in [0.0, 1.0]")
        if self.readiness_bundle_count < 0:
            raise ValueError("camera.readiness_bundle_count must be non-negative")
        if self.readiness_timeout_sec <= 0.0:
            raise ValueError("camera.readiness_timeout_sec must be positive")
        if self.stale_timeout_sec <= 0.0:
            raise ValueError("camera.stale_timeout_sec must be positive")


@dataclass(frozen=True)
class GripperConfig:
    # Physical gripper actuation backend for flow-infer rollouts. 'none' keeps
    # the fail-closed NoopGripperBackend; 'pika_serial' drives robot-mounted
    # Pika grippers over local serial (POSITION_CTRL rad).
    backend: str = "none"
    left_port: str = "/dev/pika-left"
    right_port: str = "/dev/pika-right"
    # Directory containing the 'pika' package (AgileX SDK copy).
    pika_sdk_path: str = ""
    min_rad: float = 0.0
    max_rad: float = 1.75
    deadband_rad: float = 0.005
    max_hz: float = 60.0
    suppress_sdk_logs: bool = True
    # Whether the physical grippers actuate during controller_sim rollouts
    # (arms are controller-simulated; grippers are separate local hardware).
    # real_policy additionally requires safety.allow_real_gripper_motion and
    # the RB_ALLOW_REAL_GRIPPER=1 env gate.
    actuate_in_controller_simulation: bool = False
    # Whether to drive both grippers fully OPEN once at rollout startup so every
    # rollout begins from a known open pose. DEFAULT OFF: hold each gripper at its
    # current power-on position instead (the backend seeds its target from the live
    # motor position on connect). Left/right Pika motors have opposite power-on
    # directions when not homed, so a blind startup-open lands asymmetrically
    # (observed: right opens, left closes); holding the existing position avoids
    # that. Set true to restore the legacy open-at-startup.
    startup_open: bool = False

    def __post_init__(self) -> None:
        if self.backend not in {"none", "pika_serial"}:
            raise ValueError("gripper.backend must be 'none' or 'pika_serial'")
        if self.max_rad <= self.min_rad:
            raise ValueError("gripper.max_rad must be greater than gripper.min_rad")
        if self.deadband_rad < 0.0:
            raise ValueError("gripper.deadband_rad must be non-negative")
        if self.max_hz <= 0.0:
            raise ValueError("gripper.max_hz must be positive")


@dataclass(frozen=True)
class ForceRecoveryConfig:
    """Policy-side recovery gate driven by server-owned force-contact telemetry."""

    enable: bool = False
    contact_behavior: str = "recover"
    settle_time_sec: float = 0.12
    max_linear_velocity_m_s: float = 0.002
    max_angular_velocity_rad_s: float = 0.05
    contact_timeout_sec: float = 5.0
    settling_timeout_sec: float = 2.0

    def __post_init__(self) -> None:
        if self.contact_behavior not in {"recover", "continue"}:
            raise ValueError("force_recovery.contact_behavior must be recover or continue")
        if self.settle_time_sec < 0.0:
            raise ValueError("force_recovery.settle_time_sec must be non-negative")
        if self.max_linear_velocity_m_s < 0.0:
            raise ValueError("force_recovery.max_linear_velocity_m_s must be non-negative")
        if self.max_angular_velocity_rad_s < 0.0:
            raise ValueError("force_recovery.max_angular_velocity_rad_s must be non-negative")
        if self.contact_timeout_sec <= 0.0:
            raise ValueError("force_recovery.contact_timeout_sec must be positive")
        if self.settling_timeout_sec <= 0.0:
            raise ValueError("force_recovery.settling_timeout_sec must be positive")


@dataclass(frozen=True)
class JointSineConfig:
    selected_arm: str = "both"
    amplitude_deg: tuple[float, ...] = (1.0, 1.0, 1.0, 0.5, 0.5, 0.5)
    frequency_hz: float = 0.1
    simulation_only: bool = True


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
class SpaceMouseDiscoveryConfig:
    enable: bool = False
    vendor_id: int = 0x256F
    product_id: int = 0xC652
    interface_number: int = 0
    scan_period_sec: float = 0.5
    poll_period_sec: float = 0.002

    def __post_init__(self) -> None:
        if not 0 <= self.vendor_id <= 0xFFFF or not 0 <= self.product_id <= 0xFFFF:
            raise ValueError("spacemouse discovery vendor_id/product_id must be USB 16-bit integers")
        if self.interface_number < 0:
            raise ValueError("spacemouse discovery interface_number must be non-negative")
        if self.scan_period_sec <= 0.0 or self.poll_period_sec <= 0.0:
            raise ValueError("spacemouse discovery periods must be positive")


@dataclass(frozen=True)
class SpaceMouseGripperButtonsConfig:
    enable: bool = False
    open_button: int = 0
    close_button: int = 1
    open_percent: float = 100.0
    close_percent: float = 10.0

    def __post_init__(self) -> None:
        if self.open_button < 0 or self.close_button < 0:
            raise ValueError("spacemouse_pose_target_dual.gripper_buttons button indices must be non-negative")
        if self.open_button == self.close_button:
            raise ValueError("spacemouse_pose_target_dual.gripper_buttons open_button and close_button must differ")
        for name in ("open_percent", "close_percent"):
            value = float(getattr(self, name))
            if value < 0.0 or value > 100.0:
                raise ValueError(f"spacemouse_pose_target_dual.gripper_buttons.{name} must be in [0, 100]")


@dataclass(frozen=True)
class DualSpaceMousePoseTargetConfig:
    discovery: SpaceMouseDiscoveryConfig = field(default_factory=SpaceMouseDiscoveryConfig)
    left: SpaceMouseDeviceConfig = field(default_factory=SpaceMouseDeviceConfig)
    right: SpaceMouseDeviceConfig = field(
        default_factory=lambda: SpaceMouseDeviceConfig(device_number=1)
    )
    max_linear_step_m: float = 0.001
    max_angular_step_rad: float = 0.01
    max_target_lead_m: float = 0.05
    max_target_lead_rad: float = 0.35
    deadband: float = 0.08
    activation_deadband: float | None = None
    response_curve_gamma: float = 3.0
    linear_axis_signs: tuple[float, ...] = (1.0, 1.0, 1.0)
    angular_axis_signs: tuple[float, ...] = (1.0, 1.0, 1.0)
    angular_axis_order: tuple[str, ...] = ("rx", "ry", "rz")
    sample_stale_timeout_sec: float = 0.05
    require_deadman: bool = True
    startup_requires_neutral: bool = True
    startup_neutral_hold_sec: float = 0.3
    gripper_buttons: SpaceMouseGripperButtonsConfig = field(
        default_factory=SpaceMouseGripperButtonsConfig
    )

    def __post_init__(self) -> None:
        if self.max_linear_step_m < 0.0:
            raise ValueError("spacemouse_pose_target_dual.max_linear_step_m must be non-negative")
        if self.max_angular_step_rad < 0.0:
            raise ValueError("spacemouse_pose_target_dual.max_angular_step_rad must be non-negative")
        if self.max_target_lead_m < 0.0:
            raise ValueError("spacemouse_pose_target_dual.max_target_lead_m must be non-negative")
        if self.max_target_lead_rad < 0.0:
            raise ValueError("spacemouse_pose_target_dual.max_target_lead_rad must be non-negative")
        if self.deadband < 0.0:
            raise ValueError("spacemouse_pose_target_dual.deadband must be non-negative")
        if self.activation_deadband is not None and self.activation_deadband < 0.0:
            raise ValueError("spacemouse_pose_target_dual.activation_deadband must be non-negative")
        if self.response_curve_gamma < 1.0:
            raise ValueError("spacemouse_pose_target_dual.response_curve_gamma must be >= 1.0")
        for name in ("linear_axis_signs", "angular_axis_signs"):
            signs = getattr(self, name)
            if len(signs) != 3 or any(sign not in (-1.0, 1.0) for sign in signs):
                raise ValueError(f"spacemouse_pose_target_dual.{name} must be 3 entries of -1 or 1")
        if sorted(str(axis).lower() for axis in self.angular_axis_order) != ["rx", "ry", "rz"]:
            raise ValueError(
                "spacemouse_pose_target_dual.angular_axis_order must be a permutation of rx/ry/rz"
            )
        if self.startup_neutral_hold_sec < 0.0:
            raise ValueError("spacemouse_pose_target_dual.startup_neutral_hold_sec must be non-negative")
        if self.sample_stale_timeout_sec <= 0.0:
            raise ValueError("spacemouse_pose_target_dual.sample_stale_timeout_sec must be positive")
        if self.gripper_buttons.enable and self.require_deadman:
            gripper_buttons = {
                self.gripper_buttons.open_button,
                self.gripper_buttons.close_button,
            }
            conflicts = []
            if self.left.deadman_button in gripper_buttons:
                conflicts.append("left.deadman_button")
            if self.right.deadman_button in gripper_buttons:
                conflicts.append("right.deadman_button")
            if conflicts:
                joined = ", ".join(conflicts)
                raise ValueError(
                    "spacemouse_pose_target_dual.gripper_buttons conflicts with "
                    f"{joined}; disable require_deadman or choose different buttons"
                )


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
class UmiTcpPoseTargetConditioningConfig:
    enable: bool = True
    mode: str = "foh_se3"
    emit_rate_hz: float = 500.0
    min_interpolation_horizon_sec: float = 0.004
    max_interpolation_horizon_sec: float = 0.012
    default_interpolation_horizon_sec: float = 0.006
    reset_on_engage: bool = True
    stop_on_release: bool = True
    stop_on_stale: bool = True

    def __post_init__(self) -> None:
        if self.mode not in {"none", "foh_se3"}:
            raise ValueError("umi_dual_cartesian.tcp_pose_target_conditioning.mode must be none or foh_se3")
        if self.emit_rate_hz <= 0.0:
            raise ValueError("umi_dual_cartesian.tcp_pose_target_conditioning.emit_rate_hz must be positive")
        if self.min_interpolation_horizon_sec <= 0.0:
            raise ValueError(
                "umi_dual_cartesian.tcp_pose_target_conditioning.min_interpolation_horizon_sec must be positive"
            )
        if self.max_interpolation_horizon_sec < self.min_interpolation_horizon_sec:
            raise ValueError(
                "umi_dual_cartesian.tcp_pose_target_conditioning.max_interpolation_horizon_sec "
                "must be >= min_interpolation_horizon_sec"
            )
        if not (
            self.min_interpolation_horizon_sec
            <= self.default_interpolation_horizon_sec
            <= self.max_interpolation_horizon_sec
        ):
            raise ValueError(
                "umi_dual_cartesian.tcp_pose_target_conditioning.default_interpolation_horizon_sec "
                "must be within [min_interpolation_horizon_sec, max_interpolation_horizon_sec]"
            )


@dataclass(frozen=True)
class UmiTargetLeadClampConfig:
    enable: bool = True
    max_target_lead_m: float = 0.060
    max_target_lead_rad: float = 0.25
    rebase_conditioner_on_clamp: bool = True

    def __post_init__(self) -> None:
        if self.max_target_lead_m < 0.0:
            raise ValueError("umi_dual_cartesian.target_lead_clamp.max_target_lead_m must be non-negative")
        if self.max_target_lead_rad < 0.0:
            raise ValueError("umi_dual_cartesian.target_lead_clamp.max_target_lead_rad must be non-negative")


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
    deadband_linear_m: float = 0.0
    deadband_angular_rad: float = 0.0
    linear_axis_signs: tuple[float, ...] = (1.0, 1.0, 1.0)
    angular_axis_signs: tuple[float, ...] = (1.0, 1.0, 1.0)
    # The pika publisher streams the official gripper-tip pose by default
    # (--pose-frame tip), so the receiver adds no further offset; see
    # stack_real.yaml for the paired r_align/axis-sign geometry.
    gripper_offset: tuple[float, ...] = (0.0, 0.0, 0.0)
    r_align: tuple[float, ...] = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    workspace_bounds: dict[str, tuple[float, float]] | tuple[float, ...] | None = None
    sample_hold_timeout_sec: float = 0.05
    # Ride out brief clutch (foot-switch) deadman drops for absolute
    # TcpPoseTarget: while the deadman is released for less than this window the
    # last setpoint keeps streaming (arm holds, server stays fresh) instead of
    # tearing down to Hold. 0.0 restores the legacy immediate-release behavior.
    deadman_release_grace_sec: float = 0.2
    tcp_pose_target_conditioning: UmiTcpPoseTargetConditioningConfig = field(
        default_factory=UmiTcpPoseTargetConditioningConfig
    )
    target_lead_clamp: UmiTargetLeadClampConfig = field(default_factory=UmiTargetLeadClampConfig)

    def __post_init__(self) -> None:
        if self.max_linear_step_m < 0.0:
            raise ValueError("umi_dual_cartesian.max_linear_step_m must be non-negative")
        if self.max_angular_step_rad < 0.0:
            raise ValueError("umi_dual_cartesian.max_angular_step_rad must be non-negative")
        if self.input_moving_average_window < 0:
            raise ValueError("umi_dual_cartesian.input_moving_average_window must be non-negative")
        if self.deadband_linear_m < 0.0:
            raise ValueError("umi_dual_cartesian.deadband_linear_m must be non-negative")
        if self.deadband_angular_rad < 0.0:
            raise ValueError("umi_dual_cartesian.deadband_angular_rad must be non-negative")
        for name in ("linear_axis_signs", "angular_axis_signs"):
            signs = getattr(self, name)
            if len(signs) != 3 or any(sign not in (-1.0, 1.0) for sign in signs):
                raise ValueError(f"umi_dual_cartesian.{name} must contain 3 entries of -1 or 1")
        if len(self.gripper_offset) != 3:
            raise ValueError("umi_dual_cartesian.gripper_offset must contain 3 values")
        if len(self.r_align) not in {3, 9}:
            raise ValueError("umi_dual_cartesian.r_align must contain 3 RPY values or 9 matrix values")
        if self.sample_hold_timeout_sec <= 0.0:
            raise ValueError("umi_dual_cartesian.sample_hold_timeout_sec must be positive")
        if self.deadman_release_grace_sec < 0.0:
            raise ValueError("umi_dual_cartesian.deadman_release_grace_sec must be non-negative")


@dataclass(frozen=True)
class TeleopMuxConfig:
    """SpaceMouse + UMI side-by-side teleop (action_source: teleop_mux).

    tie_break picks the owner when both sources engage on the same tick;
    otherwise the first source to engage owns the robot until it idles."""

    tie_break: str = "umi"

    def __post_init__(self) -> None:
        if self.tie_break not in {"spacemouse", "umi"}:
            raise ValueError("teleop_mux.tie_break must be spacemouse or umi")


@dataclass(frozen=True)
class PolicyRunnerConfig:
    schema: str = "robotics_lab.policy_runner.v1"
    mode: str = "simulation"
    action_source: str = "hold"
    runtime: RuntimeConfig = field(default_factory=RuntimeConfig)
    geometry: GeometryConfig = field(default_factory=GeometryConfig)
    recording: RecordingConfig = field(default_factory=RecordingConfig)
    arm_init_override: ArmInitOverrideConfig = field(default_factory=ArmInitOverrideConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    gripper: GripperConfig = field(default_factory=GripperConfig)
    force_recovery: ForceRecoveryConfig = field(default_factory=ForceRecoveryConfig)
    robot_state: RobotStateConfig = field(default_factory=RobotStateConfig)
    servo_command: ServoCommandConfig = field(default_factory=ServoCommandConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    joint_sine: JointSineConfig = field(default_factory=JointSineConfig)
    spacemouse_pose_target_dual: DualSpaceMousePoseTargetConfig = field(
        default_factory=DualSpaceMousePoseTargetConfig
    )
    umi_dual_cartesian: UmiDualCartesianConfig = field(default_factory=UmiDualCartesianConfig)
    teleop_mux: TeleopMuxConfig = field(default_factory=TeleopMuxConfig)
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
    _validate_top_level_keys(raw)
    return PolicyRunnerConfig(
        schema=str(raw.get("schema", "robotics_lab.policy_runner.v1")),
        mode=str(raw.get("mode", "simulation")),
        action_source=str(raw.get("action_source", "hold")),
        runtime=_runtime_config(_section(raw, "runtime")),
        geometry=GeometryConfig(**_section(raw, "geometry")),
        recording=_recording_config(_section(raw, "recording")),
        arm_init_override=_arm_init_override_config(_section(raw, "arm_init_override")),
        camera=_camera_config(_section(raw, "camera")),
        gripper=_gripper_config(_section(raw, "gripper")),
        force_recovery=_force_recovery_config(_section(raw, "force_recovery")),
        robot_state=RobotStateConfig(**_section(raw, "robot_state")),
        servo_command=_servo_command_config(_section(raw, "servo_command")),
        safety=_safety_config(_section(raw, "safety")),
        joint_sine=_joint_sine_config(_section(raw, "joint_sine")),
        spacemouse_pose_target_dual=_spacemouse_pose_target_dual_config(
            _section(raw, "spacemouse_pose_target_dual")
        ),
        umi_dual_cartesian=_umi_dual_cartesian_config(_section(raw, "umi_dual_cartesian")),
        teleop_mux=_teleop_mux_config(_section(raw, "teleop_mux")),
        command_rate_hz=float(raw.get("command_rate_hz", 500.0)),
    )


def _validate_top_level_keys(raw: dict[str, Any]) -> None:
    allowed = {
        "schema",
        "mode",
        "action_source",
        "runtime",
        "geometry",
        "recording",
        "arm_init_override",
        "camera",
        "gripper",
        "force_recovery",
        "robot_state",
        "servo_command",
        "safety",
        "joint_sine",
        "spacemouse_pose_target_dual",
        "umi_dual_cartesian",
        "teleop_mux",
        "command_rate_hz",
    }
    unknown = sorted(set(raw) - allowed)
    if unknown:
        raise ValueError(f"unsupported policy_runner config key(s): {', '.join(unknown)}")


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
    if "control_bind" in raw:
        raw["control_bind"] = str(raw["control_bind"])
    if "control_enabled" in raw:
        raw["control_enabled"] = bool(raw["control_enabled"])
    if "status_endpoint" in raw and raw["status_endpoint"] is not None:
        raw["status_endpoint"] = str(raw["status_endpoint"])
    if "status_rate_hz" in raw:
        raw["status_rate_hz"] = float(raw["status_rate_hz"])
    if "dataset_metadata" in raw:
        value = raw["dataset_metadata"]
        if not isinstance(value, dict):
            raise ValueError("recording.dataset_metadata must be a mapping")
        raw["dataset_metadata"] = dict(value)
    return RecordingConfig(**raw)


def _arm_init_override_config(raw: dict[str, Any]) -> ArmInitOverrideConfig:
    for key in (
        "auto_clear_on_done",
        "auto_clear_on_failed",
        "resume_flow_on_done",
        "resume_flow_on_failed",
        "allow_manual_cancel_after_failed",
        "reset_flow_source_on_start",
        "reset_flow_source_on_resume",
    ):
        if key in raw:
            raw[key] = bool(raw[key])
    return ArmInitOverrideConfig(**raw)


def _camera_config(raw: dict[str, Any]) -> CameraConfig:
    if "max_age_ms" in raw:
        raw["max_age_ms"] = float(raw["max_age_ms"])
    if "expected_cameras" in raw:
        value = raw["expected_cameras"]
        if not isinstance(value, list):
            raise ValueError("camera.expected_cameras must be a list")
        raw["expected_cameras"] = [str(item) for item in value]
    if "wrist_source" in raw:
        raw["wrist_source"] = str(raw["wrist_source"])
    if "wrist_crop_frac" in raw:
        raw["wrist_crop_frac"] = float(raw["wrist_crop_frac"])
    if "readiness_bundle_count" in raw:
        raw["readiness_bundle_count"] = int(raw["readiness_bundle_count"])
    for key in ("readiness_timeout_sec", "stale_timeout_sec"):
        if key in raw:
            raw[key] = float(raw[key])
    return CameraConfig(**raw)


def _gripper_config(raw: dict[str, Any]) -> GripperConfig:
    for key in ("backend", "left_port", "right_port", "pika_sdk_path"):
        if key in raw:
            raw[key] = str(raw[key])
    for key in ("min_rad", "max_rad", "deadband_rad", "max_hz"):
        if key in raw:
            raw[key] = float(raw[key])
    if "suppress_sdk_logs" in raw:
        raw["suppress_sdk_logs"] = bool(raw["suppress_sdk_logs"])
    if "actuate_in_controller_simulation" in raw:
        raw["actuate_in_controller_simulation"] = bool(raw["actuate_in_controller_simulation"])
    if "startup_open" in raw:
        raw["startup_open"] = bool(raw["startup_open"])
    return GripperConfig(**raw)


def _force_recovery_config(raw: dict[str, Any]) -> ForceRecoveryConfig:
    legacy_timeout = raw.get("timeout_sec")
    phase_fields = {"contact_timeout_sec", "settling_timeout_sec"}.intersection(raw)
    if legacy_timeout is not None and phase_fields:
        raise ValueError(
            "force_recovery must not set deprecated timeout_sec together with "
            "contact_timeout_sec or settling_timeout_sec"
        )
    if legacy_timeout is not None:
        # Compatibility for older non-real profiles: the former single deadline
        # is applied independently to each phase instead of spanning both phases.
        timeout = float(raw.pop("timeout_sec"))
        raw["contact_timeout_sec"] = timeout
        raw["settling_timeout_sec"] = timeout
    if "enable" in raw:
        raw["enable"] = bool(raw["enable"])
    if "contact_behavior" in raw:
        raw["contact_behavior"] = str(raw["contact_behavior"])
    for key in (
        "settle_time_sec",
        "max_linear_velocity_m_s",
        "max_angular_velocity_rad_s",
        "contact_timeout_sec",
        "settling_timeout_sec",
    ):
        if key in raw:
            raw[key] = float(raw[key])
    return ForceRecoveryConfig(**raw)


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


def _spacemouse_pose_target_dual_config(raw: dict[str, Any]) -> DualSpaceMousePoseTargetConfig:
    discovery = _spacemouse_discovery_config(_section(raw, "discovery"))
    left = _spacemouse_device_config(_section(raw, "left"))
    right_raw = _section(raw, "right")
    if "device_number" not in right_raw:
        right_raw["device_number"] = 1
    right = _spacemouse_device_config(right_raw)
    gripper_buttons = _spacemouse_gripper_buttons_config(_section(raw, "gripper_buttons"))
    top_level = {
        key: value
        for key, value in raw.items()
        if key not in {"discovery", "left", "right", "gripper_buttons"}
    }
    for key in ("max_linear_step_m", "max_angular_step_rad", "max_target_lead_m", "max_target_lead_rad"):
        if key in top_level:
            top_level[key] = float(top_level[key])
    if "response_curve_gamma" in top_level:
        top_level["response_curve_gamma"] = float(top_level["response_curve_gamma"])
    if "deadband" in top_level:
        top_level["deadband"] = float(top_level["deadband"])
    if "sample_stale_timeout_sec" in top_level:
        top_level["sample_stale_timeout_sec"] = float(top_level["sample_stale_timeout_sec"])
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
    return DualSpaceMousePoseTargetConfig(
        discovery=discovery,
        left=left,
        right=right,
        gripper_buttons=gripper_buttons,
        **top_level,
    )


def _spacemouse_discovery_config(raw: dict[str, Any]) -> SpaceMouseDiscoveryConfig:
    for key in ("vendor_id", "product_id", "interface_number"):
        if key in raw:
            raw[key] = int(raw[key])
    for key in ("scan_period_sec", "poll_period_sec"):
        if key in raw:
            raw[key] = float(raw[key])
    if "enable" in raw:
        raw["enable"] = bool(raw["enable"])
    return SpaceMouseDiscoveryConfig(**raw)


def _spacemouse_gripper_buttons_config(raw: dict[str, Any]) -> SpaceMouseGripperButtonsConfig:
    if "enable" in raw:
        raw["enable"] = bool(raw["enable"])
    for key in ("open_button", "close_button"):
        if key in raw:
            raw[key] = int(raw[key])
    for key in ("open_percent", "close_percent"):
        if key in raw:
            raw[key] = float(raw[key])
    return SpaceMouseGripperButtonsConfig(**raw)


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
    conditioning = _umi_conditioning_config(_section(raw, "tcp_pose_target_conditioning"))
    target_lead_clamp = _umi_target_lead_clamp_config(_section(raw, "target_lead_clamp"))
    top_level = {
        key: value
        for key, value in raw.items()
        if key not in {"left", "right", "tcp_pose_target_conditioning", "target_lead_clamp"}
    }
    if "max_linear_step_m" in top_level:
        top_level["max_linear_step_m"] = float(top_level["max_linear_step_m"])
    if "max_angular_step_rad" in top_level:
        top_level["max_angular_step_rad"] = float(top_level["max_angular_step_rad"])
    if "input_moving_average_window" in top_level:
        top_level["input_moving_average_window"] = int(top_level["input_moving_average_window"])
    for key in ("deadband_linear_m", "deadband_angular_rad"):
        if key in top_level:
            top_level[key] = float(top_level[key])
    for key in ("linear_axis_signs", "angular_axis_signs"):
        if key in top_level:
            top_level[key] = _tuple3(top_level[key], f"umi_dual_cartesian.{key}")
    if "sample_stale_timeout_sec" in top_level:
        if "sample_hold_timeout_sec" in top_level:
            raise ValueError(
                "umi_dual_cartesian must not set both sample_hold_timeout_sec "
                "and deprecated sample_stale_timeout_sec"
            )
        top_level["sample_hold_timeout_sec"] = top_level.pop("sample_stale_timeout_sec")
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
    return UmiDualCartesianConfig(
        left=left,
        right=right,
        tcp_pose_target_conditioning=conditioning,
        target_lead_clamp=target_lead_clamp,
        **top_level,
    )


def _umi_conditioning_config(raw: dict[str, Any]) -> UmiTcpPoseTargetConditioningConfig:
    if "enable" in raw:
        raw["enable"] = bool(raw["enable"])
    if "mode" in raw:
        raw["mode"] = str(raw["mode"])
    for key in (
        "emit_rate_hz",
        "min_interpolation_horizon_sec",
        "max_interpolation_horizon_sec",
        "default_interpolation_horizon_sec",
    ):
        if key in raw:
            raw[key] = float(raw[key])
    for key in ("reset_on_engage", "stop_on_release", "stop_on_stale"):
        if key in raw:
            raw[key] = bool(raw[key])
    return UmiTcpPoseTargetConditioningConfig(**raw)


def _umi_target_lead_clamp_config(raw: dict[str, Any]) -> UmiTargetLeadClampConfig:
    if "enable" in raw:
        raw["enable"] = bool(raw["enable"])
    for key in ("max_target_lead_m", "max_target_lead_rad"):
        if key in raw:
            raw[key] = float(raw[key])
    if "rebase_conditioner_on_clamp" in raw:
        raw["rebase_conditioner_on_clamp"] = bool(raw["rebase_conditioner_on_clamp"])
    return UmiTargetLeadClampConfig(**raw)


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


def _teleop_mux_config(raw: dict[str, Any]) -> TeleopMuxConfig:
    if "tie_break" in raw:
        raw["tie_break"] = str(raw["tie_break"])
    return TeleopMuxConfig(**raw)


def _tuple6(value: Any, label: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        raise ValueError(f"{label} must contain 6 numbers")
    return tuple(float(v) for v in value)


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
