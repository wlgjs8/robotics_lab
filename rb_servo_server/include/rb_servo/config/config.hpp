#pragma once

#include <string>
#include <vector>
#include "rb_servo/core/types.hpp"

namespace rb_servo {

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

    JointArray initial_q_deg{};

    double speed_bar = 0.1;

    double servo_time_sec = 0.01;
    double servo_lookahead_sec = 0.1;
    double servo_gain = 1.0;
    double servo_acc = 1.0;

    bool disable_waiting_ack = true;
};

struct ArmMountConfig {
    ArmId arm_id = ArmId::Left;
    Pose6D base_pose_in_stand;
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
};

struct SafetyConfig {
    JointArray q_min_deg{};
    JointArray q_max_deg{};
    JointArray dq_max_deg_s{};
    JointArray ddq_max_deg_s2{};

    double command_timeout_sec = 0.2;
    double max_tracking_error_deg = 10.0;

    // mock/rbsim can use SnapToActual for fast iteration. real should use FaultLatch.
    TrackingErrorPolicy tracking_error_policy = TrackingErrorPolicy::FaultLatch;

    bool stop_both_arms_on_single_arm_error = true;
    bool latch_fault_on_robot_state_error = true;
};

struct ServoConfig {
    int rate_hz = 200;
    double command_timeout_sec = 0.2;
    ControlMode startup_mode = ControlMode::Hold;
    bool send_servo_commands = true;

    bool enable_realtime_priority = true;
    int realtime_priority = 80;
    int cpu_core = -1;

    // Use actual period for logging, but cap filter dt so one late tick does not
    // create an unexpectedly large joint step.
    double filter_dt_min_ratio = 0.5;
    double filter_dt_max_ratio = 1.5;
};

struct NetworkConfig {
    std::string command_bind = "udp://127.0.0.1:50010";
    std::string state_pub_endpoint = "udp://127.0.0.1:50110";
    // Deprecated compatibility alias. Keep synchronized with state_pub_endpoint.
    std::string state_pub_bind = "udp://127.0.0.1:50110";
    int state_pub_rate_hz = 20;
    std::vector<std::string> command_source_allowlist{"127.0.0.1/32"};
    double command_timeout_sec = 0.2;
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

struct DualArmConfig {
    BackendConfig left_robot;
    BackendConfig right_robot;

    ArmMountConfig left_mount;
    ArmMountConfig right_mount;

    ServoConfig servo;
    SafetyConfig safety;
    NetworkConfig network;
    LoggingConfig logging;
    ForceControlConfig force_control;
    KinematicsConfig kinematics;
};

DualArmConfig loadConfigFromYaml(const std::string& path);

}  // namespace rb_servo
