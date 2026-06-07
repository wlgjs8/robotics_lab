#pragma once

#include <optional>
#include <string>
#include <vector>
#include "rb_servo/core/types.hpp"

namespace rb_servo {

enum class ServoIoModel {
    Direct,
    Worker
};

struct BackendConfig {
    BackendType backend_type = BackendType::Rbpodo;
    RunMode run_mode = RunMode::Real;

    std::string name;
    std::string ip;
    std::string operation_mode = "real";
    std::string simulator_control_endpoint = "tcp://127.0.0.1:50200";
    // Deprecated compatibility alias. Keep synchronized with simulator_control_endpoint.
    std::string rbsim_control_endpoint = "tcp://127.0.0.1:50200";
    double rbsim_request_timeout_sec = 0.2;
    double rbsim_connect_timeout_sec = 0.2;
    double rbsim_read_timeout_sec = 0.2;
    double rbsim_send_timeout_sec = 0.2;
    double rbsim_stop_timeout_sec = 0.2;
    double rbsim_reset_timeout_sec = 0.2;

    double command_timeout_sec = 0.2;

    JointArray initial_q_deg{};

    double speed_bar = 0.1;

    double servo_t1_sec = 0.002;
    double servo_t2_sec = 0.05;
    double servo_gain = 1.0;
    double servo_alpha = 0.5;

    // Deprecated rbpodo aliases. Kept synchronized with canonical fields while
    // old configs migrate.
    double servo_time_sec = 0.002;
    double servo_lookahead_sec = 0.05;
    double servo_acc = 0.5;

    // rbpodo-only. When true, Cobot::disable_waiting_ack() makes command calls
    // return after socket send instead of waiting for controller ACK.
    bool disable_waiting_ack = false;
    bool allow_controller_simulation_diagnostics_suspect = false;
    bool allow_controller_simulation_init_error = false;
};

struct ArmMountConfig {
    ArmId arm_id = ArmId::Left;
    Pose6D base_pose_in_stand;
};

struct IkSolverConfig {
    bool enable = true;
    int max_iterations = 50;
    double timeout_ms = 2.0;
    double damping = 0.001;
    double position_tolerance_m = 0.001;
    double orientation_tolerance_rad = 0.02;
    JointArray max_step_deg{2.0, 2.0, 2.0, 3.0, 3.0, 4.0};
};

struct KinematicsConfig {
    bool enable = false;
    std::string provider = "none";
    std::string urdf = "descriptions/urdf/rb3_730e.urdf";
    std::string base_link = "world";
    std::string tip_link = "tcp";
    std::vector<std::string> joint_names{
        "base_joint",
        "shoulder_joint",
        "elbow_joint",
        "wrist1_joint",
        "wrist2_joint",
        "wrist3_joint",
    };
    std::string q_units = "deg";
    bool publish_tcp = false;
    IkSolverConfig ik;
};

enum class ControllerSimulationTrackingErrorSource {
    Actual,
    Reference
};

enum class ControllerSimulationPhysicalMotionPolicy {
    WarnOnly,
    FaultLatch
};

enum class RbpodoAsyncStreamingMode {
    Disabled,
    SdkAckWorker,
    SocketSendSupervised
};

enum class RbpodoAsyncQueuePolicy {
    LatestWins
};

enum class RbpodoAsyncReferenceSupervisionPolicy {
    WarnOnly,
    FaultLatch
};

struct RbpodoAsyncAckSupervisionConfig {
    bool enable = true;
    double expected_ack_timeout_ms = 50.0;
    double missing_ack_fault_after_ms = 100.0;
    int max_consecutive_missing_ack = 10;
};

struct RbpodoAsyncReferenceSupervisionConfig {
    bool enable = true;
    double q_ref_update_timeout_ms = 50.0;
    double q_ref_target_tolerance_deg = 1.0;
    double q_ref_target_fault_after_ms = 100.0;
    double tcp_ref_update_timeout_ms = 50.0;
    double tcp_ref_target_tolerance_m = 0.02;
    double tcp_ref_target_fault_after_ms = 100.0;
    RbpodoAsyncReferenceSupervisionPolicy policy =
        RbpodoAsyncReferenceSupervisionPolicy::FaultLatch;
};

struct RbpodoAsyncDiagnosticsConfig {
    bool publish_per_command_jsonl = false;
};

struct RbpodoAsyncStreamingConfig {
    bool enable = false;
    RbpodoAsyncStreamingMode mode = RbpodoAsyncStreamingMode::Disabled;
    int rate_hz = 500;
    RbpodoAsyncQueuePolicy queue_policy = RbpodoAsyncQueuePolicy::LatestWins;
    double max_pending_age_ms = 10.0;
    RbpodoAsyncAckSupervisionConfig ack_supervision;
    RbpodoAsyncReferenceSupervisionConfig reference_supervision;
    RbpodoAsyncDiagnosticsConfig diagnostics;
};

struct SafetyConfig {
    JointArray q_min_deg{};
    JointArray q_max_deg{};
    JointArray dq_max_deg_s{};
    JointArray ddq_max_deg_s2{};
    JointArray joint_wrap_period_deg{};

    double command_timeout_sec = 0.2;
    double max_tracking_error_deg = 10.0;

    // mock/rbsim can use SnapToActual for fast iteration. real should use FaultLatch.
    TrackingErrorPolicy tracking_error_policy = TrackingErrorPolicy::FaultLatch;

    bool stop_both_arms_on_single_arm_error = true;
    bool latch_fault_on_robot_state_error = true;
    bool joint_wrap_for_startup_validation = false;
    bool joint_wrap_for_motion_safety = false;
    ControllerSimulationTrackingErrorSource controller_simulation_tracking_error_source =
        ControllerSimulationTrackingErrorSource::Actual;
    ControllerSimulationPhysicalMotionPolicy controller_simulation_physical_motion_policy =
        ControllerSimulationPhysicalMotionPolicy::FaultLatch;
    double controller_simulation_physical_motion_threshold_deg = 0.05;
};

inline constexpr JointArray rbpodoDefaultSafetyJointMinDeg() {
    return JointArray{-360.0, -360.0, -360.0, -360.0, -360.0, -360.0};
}

inline constexpr JointArray rbpodoDefaultSafetyJointMaxDeg() {
    return JointArray{360.0, 360.0, 360.0, 360.0, 360.0, 360.0};
}

struct ServoConfig {
    int rate_hz = 500;
    double command_timeout_sec = 0.2;
    ServoIoModel io_model = ServoIoModel::Direct;
    ControlMode startup_mode = ControlMode::Hold;
    bool send_servo_commands = true;
    bool allow_readonly_faulted_startup = false;
    bool allow_readonly_q_range_violation_startup = false;
    bool allow_readonly_wrong_mode_startup = false;
    bool allow_controller_simulation_motion = false;
    bool allow_controller_simulation_diagnostics_suspect = false;
    bool allow_controller_simulation_init_error = false;
    bool allow_controller_simulation_not_activated = false;

    bool enable_realtime_priority = true;
    int realtime_priority = 80;
    int cpu_core = -1;
    double worker_read_period_sec = 0.002;

    // Use actual period for logging, but cap filter dt so one late tick does not
    // create an unexpectedly large joint step.
    double filter_dt_min_ratio = 0.5;
    double filter_dt_max_ratio = 1.5;

    double servo_t1_rate_match_tolerance_ratio = 0.2;
    bool allow_servo_t1_rate_mismatch = false;

    RbpodoAsyncStreamingConfig rbpodo_async_streaming;
};

struct NetworkConfig {
    std::string command_bind = "udp://127.0.0.1:50010";
    std::string state_pub_endpoint = "udp://127.0.0.1:50110";
    // Deprecated compatibility alias. Keep synchronized with the first endpoint.
    std::string state_pub_bind = "udp://127.0.0.1:50110";
    // Canonical UDP state destinations. The first entry mirrors state_pub_endpoint.
    std::vector<std::string> state_pub_endpoints{"udp://127.0.0.1:50110"};
    int state_pub_rate_hz = 20;
    std::vector<std::string> command_source_allowlist{"127.0.0.1/32"};
    double command_timeout_sec = 0.2;
    bool command_source_enforce_lease = false;
    double command_source_lease_timeout_sec = 1.0;
};

struct CommandSourceConfig {
    bool enforce_lease = false;
    double lease_timeout_sec = 1.0;
};

struct LoggingConfig {
    bool enable = true;
    std::string directory = "./logs";
    int flush_period_ms = 100;
    size_t queue_capacity = 4096;
};

struct ForceControlConfig {
    std::string provider = "null";
    bool enable = false;
    int update_rate_hz = 200;

    // Simple admittance fallback used before integrating mo_forcecontroller.
    double admittance_gain_pos = 0.0002;  // m / (N*s) applied as gain * error * dt
    double admittance_gain_rot = 0.0001;  // rad / (Nm*s)
    double force_lpf_alpha = 0.2;

    double max_pos_offset_m = 0.01;
    double max_rot_offset_rad = 0.1;
    double max_pos_step_m = 0.001;
    double max_rot_step_rad = 0.01;
};

struct LinearMoveConfig {
    double min_duration_sec = 0.05;
    double max_duration_sec = 10.0;
    double default_linear_speed_m_s = 0.03;
    double default_angular_speed_rad_s = 0.2;
    double constant_orientation_tolerance_rad = 0.005;
    LinearMoveOrientationMode default_orientation_mode = LinearMoveOrientationMode::Constant;
};

struct CircleMoveConfig {
    bool allow_in_simulation = true;
    bool allow_in_real = false;
    double max_diameter_m = 0.20;
    double min_period_sec = 3.0;
};

enum class CartesianLimitPolicy {
    Clamp,
    Reject
};

enum class CartesianVelocityTargetIntegrationMode {
    MeasuredActual,
    MeasuredActualLookahead,
    PreviousCommand
};

enum class CartesianCommandActualErrorPolicy {
    Reset,
    Fault
};

enum class CartesianControllerSimulationStateSource {
    Actual,
    Reference
};

struct CartesianControlConfig {
    bool enable = true;
    bool allow_in_simulation = true;
    bool allow_in_real = false;
    bool allow_in_controller_simulation = false;
    bool enable_server_side_circle_track = false;
    bool enable_benchmark_primitives = false;
    double warn_ik_duration_us = 3000.0;
    double fail_ik_duration_us = 0.0;
    // Deprecated compatibility field. New control code uses path_kp_pos/path_kp_ori.
    double path_kp = 6.0;
    double path_kp_pos = 6.0;
    double path_kp_ori = 6.0;
    double twist_orientation_hold_kp = 6.0;
    double twist_angular_deadband_rad_s = 0.0001;
    double velocity_damping = 0.01;
    double max_twist_linear_m_s = 0.03;
    double max_twist_angular_rad_s = 0.2;
    double max_linear_move_speed_m_s = 0.05;
    double max_angular_move_speed_rad_s = 0.3;
    std::optional<double> max_cartesian_step_m;
    std::optional<double> max_cartesian_step_rad;
    CartesianLimitPolicy exceed_limit_policy = CartesianLimitPolicy::Clamp;
    CartesianVelocityTargetIntegrationMode velocity_target_integration =
        CartesianVelocityTargetIntegrationMode::PreviousCommand;
    CartesianControllerSimulationStateSource controller_simulation_servo_state_source =
        CartesianControllerSimulationStateSource::Actual;
    CartesianControllerSimulationStateSource controller_simulation_divergence_source =
        CartesianControllerSimulationStateSource::Actual;
    double velocity_target_lookahead_sec = 0.04;
    JointArray max_command_actual_error_deg{5.0, 5.0, 5.0, 8.0, 8.0, 10.0};
    bool reset_velocity_integrator_on_mode_change = true;
    CartesianCommandActualErrorPolicy command_actual_error_policy =
        CartesianCommandActualErrorPolicy::Reset;
    LinearMoveConfig linear_move;
    CircleMoveConfig circle_move;
};

struct DualArmConfig {
    BackendConfig left_robot;
    BackendConfig right_robot;

    ArmMountConfig left_mount;
    ArmMountConfig right_mount;

    ServoConfig servo;
    SafetyConfig safety;
    NetworkConfig network;
    CommandSourceConfig command_source;
    LoggingConfig logging;
    ForceControlConfig force_control;
    CartesianControlConfig cartesian_control;
    KinematicsConfig kinematics;
};

DualArmConfig loadConfigFromYaml(const std::string& path);

}  // namespace rb_servo
