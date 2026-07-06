#include "rb_servo/logging/servo_logger.hpp"

#include <cmath>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>

namespace rb_servo {
namespace {

// Per-run local-time stamp, matching the policy_runner action-log convention
// (actions_%Y%m%d_%H%M%S.jsonl). Local time = wall-clock (Korea time when the
// host is set to KST), so runs sort and read naturally.
std::string runStamp() {
    std::time_t now = std::time(nullptr);
    std::tm tm_local{};
    localtime_r(&now, &tm_local);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm_local);
    return std::string(buf);
}

// Per-arm Cartesian IK/solve diagnostic column names. Appended to the END of
// each row so existing column indices stay stable. Header field order MUST
// match writeCartesianSolveColumns below.
void writeCartesianSolveHeader(std::ostream& os, const char* side) {
    os << ',' << side << "_cart_attempted"
       << ',' << side << "_cart_success"
       << ',' << side << "_cart_status"
       << ',' << side << "_cart_ik_us"
       << ',' << side << "_cart_ik_iters"
       << ',' << side << "_cart_ik_timed_out"
       << ',' << side << "_cart_ik_warn_exceeded"
       << ',' << side << "_cart_ik_fail_exceeded"
       << ',' << side << "_cart_min_singular"
       << ',' << side << "_cart_applied_damping"
       << ',' << side << "_cart_sol_jump_deg"
       << ',' << side << "_cart_branch_jump_suspected"
       << ',' << side << "_cart_branch_jump_clamped"
       << ',' << side << "_cart_pos_err_m"
       << ',' << side << "_cart_ori_err_rad"
       << ',' << side << "_cart_path_active"
       << ',' << side << "_cart_path_done"
       << ',' << side << "_cart_reason"
       << ',' << side << "_cart_ik_joint_limit_index"
       << ',' << side << "_cart_ik_joint_limit_margin_deg";
}

void writePoseHeader(std::ostream& os, const char* side, const char* name) {
    os << ',' << side << '_' << name << "_x_m"
       << ',' << side << '_' << name << "_y_m"
       << ',' << side << '_' << name << "_z_m"
       << ',' << side << '_' << name << "_rx_rad"
       << ',' << side << '_' << name << "_ry_rad"
       << ',' << side << '_' << name << "_rz_rad"
       << ',' << side << '_' << name << "_qx"
       << ',' << side << '_' << name << "_qy"
       << ',' << side << '_' << name << "_qz"
       << ',' << side << '_' << name << "_qw";
}

void writeJointArrayHeader(std::ostream& os, const char* side, const char* name) {
    for (int i = 0; i < kDof; ++i) {
        os << ',' << side << '_' << name << '_' << i;
    }
}

void writeArmProfilingHeader(std::ostream& os, const char* side) {
    writePoseHeader(os, side, "command_tcp_target_stand");
    writePoseHeader(os, side, "smd_goal_stand");
    writePoseHeader(os, side, "smd_ref_stand");
    writePoseHeader(os, side, "tcp_command_stand");
    writePoseHeader(os, side, "tcp_actual_stand");
    writePoseHeader(os, side, "tcp_ref_stand");
    os << ',' << side << "_tcp_target_profile"
       << ',' << side << "_smd_profile_nf_linear_hz"
       << ',' << side << "_smd_profile_nf_angular_hz"
       << ',' << side << "_smd_profile_velocity_feedforward"
       << ',' << side << "_smd_profile_max_linear_velocity_m_s"
       << ',' << side << "_smd_profile_max_linear_accel_m_s2"
       << ',' << side << "_smd_profile_max_angular_velocity_rad_s"
       << ',' << side << "_smd_profile_max_angular_accel_rad_s2"
       << ',' << side << "_smd_profile_max_goal_lead_m"
       << ',' << side << "_smd_profile_max_goal_lead_rad"
       << ',' << side << "_smd_active"
       << ',' << side << "_smd_velocity_feedforward_used"
       << ',' << side << "_smd_linear_velocity_clipped"
       << ',' << side << "_smd_linear_accel_clipped"
       << ',' << side << "_smd_angular_velocity_clipped"
       << ',' << side << "_smd_angular_accel_clipped"
       << ',' << side << "_smd_goal_linear_velocity_ff_clipped"
       << ',' << side << "_smd_goal_angular_velocity_ff_clipped"
       << ',' << side << "_smd_goal_linear_velocity_norm_m_s"
       << ',' << side << "_smd_goal_angular_velocity_norm_rad_s"
       << ',' << side << "_smd_reanchor_count";
    // Ruckig chunk-follower stage: the active segment's chunk-step target pose
    // (pf), final stage output pose handed to IK, explicit producer/receiver
    // sequence ids, and divergence-guard inputs.
    writePoseHeader(os, side, "follower_pf_stand");
    writePoseHeader(os, side, "stage_tcp_target_stand");
    os << ',' << side << "_follower_active"
       << ',' << side << "_follower_wire_seq"
       << ',' << side << "_follower_recv_seq"
       << ',' << side << "_follower_step"
       << ',' << side << "_follower_t_in_seg_sec"
       << ',' << side << "_follower_duration_sec"
       << ',' << side << "_follower_alpha"
       << ',' << side << "_follower_converged"
       << ',' << side << "_follower_stall"
       << ',' << side << "_follower_corner"
       << ',' << side << "_follower_divergence_pos_m"
       << ',' << side << "_follower_divergence_ang_rad"
       << ',' << side << "_follower_reanchor_count"
       << ',' << side << "_safety_intervention_recent"
       << ',' << side << "_output_ma_present"
       << ',' << side << "_output_ma_window";
    writeJointArrayHeader(os, side, "q_target_before_output_ma");
    writeJointArrayHeader(os, side, "q_target_after_output_ma");
    writeJointArrayHeader(os, side, "q_sent_velocity_deg_s");
    writeJointArrayHeader(os, side, "q_sent_accel_deg_s2");
    writeJointArrayHeader(os, side, "q_sent_jerk_deg_s3");
    writeJointArrayHeader(os, side, "q_actual_velocity_deg_s");
    writeJointArrayHeader(os, side, "q_actual_accel_deg_s2");
    writeJointArrayHeader(os, side, "q_actual_jerk_deg_s3");
}

void writeTcpPoseTargetDebugHeader(std::ostream& os, const char* side) {
    os << ',' << side << "_cart_branch_jump_rate_limited"
       << ',' << side << "_cart_branch_jump_raw_deg"
       << ',' << side << "_cart_branch_jump_limit_deg"
       << ',' << side << "_cart_branch_jump_scale"
       << ',' << side << "_cart_branch_jump_retry_count";
    writeJointArrayHeader(os, side, "q_ik_seed_deg");
    writeJointArrayHeader(os, side, "q_ik_raw_solution_deg");
    writeJointArrayHeader(os, side, "q_ik_solution_deg");
    writeJointArrayHeader(os, side, "q_ik_raw_delta_deg");
    writeJointArrayHeader(os, side, "q_ik_delta_deg");
    os << ',' << side << "_safety_clamp_present";
    writeJointArrayHeader(os, side, "q_before_safety_deg");
    writeJointArrayHeader(os, side, "q_after_joint_limit_deg");
    writeJointArrayHeader(os, side, "q_after_velocity_limit_deg");
    writeJointArrayHeader(os, side, "q_after_accel_limit_deg");
    os << ',' << side << "_safety_joint_limit_clamped"
       << ',' << side << "_safety_velocity_clamped"
       << ',' << side << "_safety_accel_clamped"
       << ',' << side << "_safety_joint_limit_clamp_max_delta_deg"
       << ',' << side << "_safety_velocity_clamp_max_delta_deg"
       << ',' << side << "_safety_accel_clamp_max_delta_deg"
       << ',' << side << "_safety_joint_limit_limited_joint"
       << ',' << side << "_safety_velocity_limited_joint"
       << ',' << side << "_safety_accel_limited_joint";
}

void writeInitMotionHeader(std::ostream& os) {
    os << ",left_mode_before_init_sequencer"
       << ",right_mode_before_init_sequencer"
       << ",left_mode_after_init_sequencer"
       << ",right_mode_after_init_sequencer"
       << ",left_joint_target_profile_before_init_sequencer"
       << ",right_joint_target_profile_before_init_sequencer"
       << ",left_joint_target_profile_after_init_sequencer"
       << ",right_joint_target_profile_after_init_sequencer"
       << ",init_motion_left_status"
       << ",init_motion_right_status"
       << ",init_motion_aggregate_status"
       << ",init_motion_left_fail_mode"
       << ",init_motion_right_fail_mode"
       << ",init_motion_aggregate_fail_mode"
       << ",init_motion_left_message"
       << ",init_motion_right_message"
       << ",init_motion_aggregate_message"
       << ",init_motion_left_waypoint_index"
       << ",init_motion_left_waypoint_count"
       << ",init_motion_right_waypoint_index"
       << ",init_motion_right_waypoint_count"
       << ",init_motion_aggregate_waypoint_index"
       << ",init_motion_aggregate_waypoint_count"
       << ",init_motion_left_dist_to_goal_deg"
       << ",init_motion_right_dist_to_goal_deg"
       << ",init_motion_aggregate_dist_to_goal_deg"
       << ",non_init_arm_preserved_mode"
       << ",single_arm_freeze_other_arm"
       << ",init_motion_left_clear_threshold_m"
       << ",init_motion_right_clear_threshold_m"
       << ",init_motion_aggregate_clear_threshold_m"
       << ",init_motion_left_external_clear_threshold_m"
       << ",init_motion_right_external_clear_threshold_m"
       << ",init_motion_aggregate_external_clear_threshold_m"
       << ",init_motion_left_nearest_pair"
       << ",init_motion_right_nearest_pair"
       << ",init_motion_aggregate_nearest_pair"
       << ",init_motion_left_nearest_pair_distance_m"
       << ",init_motion_right_nearest_pair_distance_m"
       << ",init_motion_aggregate_nearest_pair_distance_m"
       << ",init_motion_left_nearest_pair_external"
       << ",init_motion_right_nearest_pair_external"
       << ",init_motion_aggregate_nearest_pair_external"
       << ",init_motion_left_goal_nearest_pair_a"
       << ",init_motion_left_goal_nearest_pair_b"
       << ",init_motion_left_goal_pair_category"
       << ",init_motion_left_goal_clearance_m"
       << ",init_motion_left_goal_threshold_m"
       << ",init_motion_left_goal_margin_deficit_m"
       << ",init_motion_right_goal_nearest_pair_a"
       << ",init_motion_right_goal_nearest_pair_b"
       << ",init_motion_right_goal_pair_category"
       << ",init_motion_right_goal_clearance_m"
       << ",init_motion_right_goal_threshold_m"
       << ",init_motion_right_goal_margin_deficit_m"
       << ",init_motion_aggregate_goal_nearest_pair_a"
       << ",init_motion_aggregate_goal_nearest_pair_b"
       << ",init_motion_aggregate_goal_pair_category"
       << ",init_motion_aggregate_goal_clearance_m"
       << ",init_motion_aggregate_goal_threshold_m"
       << ",init_motion_aggregate_goal_margin_deficit_m"
       << ",self_collision_min_clearance_m"
       << ",self_collision_pair";
}

}  // namespace

ServoLogger::ServoLogger(const LoggingConfig& config) : config_(config) {}

ServoLogger::~ServoLogger() {
    stop();
}

bool ServoLogger::start() {
    if (!config_.enable) return true;
    if (running_) return true;

    std::filesystem::create_directories(config_.directory);
    // One file per run: servo_log_<YYYYMMDD_HHMMSS>.csv (no longer truncated/
    // overwritten each run). `servo_log.csv` is kept as a symlink to the latest
    // run so existing tooling/acceptance scripts that read the fixed name still
    // resolve to the current run.
    const std::string run_name = "servo_log_" + runStamp() + ".csv";
    file_.open(config_.directory + "/" + run_name, std::ios::out | std::ios::trunc);
    if (!file_) {
        std::cerr << "[ERROR] failed to open servo log file\n";
        return false;
    }
    const std::filesystem::path latest = std::filesystem::path(config_.directory) / "servo_log.csv";
    std::error_code ec;
    std::filesystem::remove(latest, ec);  // clear any prior file/symlink
    std::filesystem::create_symlink(run_name, latest, ec);  // relative target
    if (ec) {
        std::cerr << "[WARN] servo log: could not update servo_log.csv symlink: "
                  << ec.message() << "\n";
    }
    writeHeader();

    running_ = true;
    thread_ = std::thread(&ServoLogger::threadMain, this);
    return true;
}

void ServoLogger::stop() {
    if (!running_) {
        if (file_.is_open()) file_.close();
        return;
    }
    running_ = false;
    cv_.notify_all();
    if (thread_.joinable()) {
        thread_.join();
    }
    if (file_.is_open()) {
        file_.flush();
        file_.close();
    }
}

void ServoLogger::push(const ServoSample& sample) {
    if (!config_.enable || !running_) return;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (config_.queue_capacity == 0) {
            dropped_samples_.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        if (queue_.size() >= config_.queue_capacity) {
            queue_.pop_front();
            dropped_samples_.fetch_add(1, std::memory_order_relaxed);
        }
        queue_.push_back(sample);
    }
    cv_.notify_one();
}

uint64_t ServoLogger::droppedSamples() const {
    return dropped_samples_.load(std::memory_order_relaxed);
}

void ServoLogger::threadMain() {
    while (running_) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait_for(lock, std::chrono::milliseconds(config_.flush_period_ms), [&] {
            return !queue_.empty() || !running_;
        });
        while (!queue_.empty()) {
            ServoSample sample = queue_.front();
            queue_.pop_front();
            lock.unlock();
            writeSample(sample);
            lock.lock();
        }
        if (file_) file_.flush();
    }
}

void ServoLogger::writeHeader() {
    file_ << "tick,loop_start_time_ns,loop_end_time_ns,period_ms,jitter_ms,filter_dt_ms,safety_verdict,motion_state,fault_latched,fault_reason,logger_dropped_samples,command_seq,left_mode,right_mode,left_send_ok,right_send_ok";
    file_ << ",fault_context_verdict,fault_context_domain,fault_context_arm,fault_context_backend_op,fault_context_backend_error_kind,fault_context_backend_error_name,fault_context_backend_error_code,fault_context_retryable,fault_context_recoverable,fault_context_robot_fault,fault_context_transport_fault,fault_context_state_after_source,fault_context_reason";
    file_ << ",left_send_start_ns,left_send_end_ns,right_send_start_ns,right_send_end_ns,send_skew_us,left_send_duration_us,right_send_duration_us";
    file_ << ",left_ack_policy,right_ack_policy,left_ack_observed,right_ack_observed,left_controller_acceptance_observed,right_controller_acceptance_observed,left_ack_wait_duration_us,right_ack_wait_duration_us,left_rbpodo_waiting_ack,right_rbpodo_waiting_ack,left_send_acceptance_semantics,right_send_acceptance_semantics";
    file_ << ",left_state_age_us,right_state_age_us,left_send_result_age_us,right_send_result_age_us";
    file_ << ",left_send_within_period,right_send_within_period,left_send_period_overrun,right_send_period_overrun,left_send_command_deadline_missed,right_send_command_deadline_missed";
    file_ << ",left_send_deadline_hit,right_send_deadline_hit,dispatch_skew_us,left_worker_loop_read_duration_us,right_worker_loop_read_duration_us";
    for (int i = 0; i < kDof; ++i) file_ << ",left_q_actual_" << i;
    for (int i = 0; i < kDof; ++i) file_ << ",right_q_actual_" << i;
    for (int i = 0; i < kDof; ++i) file_ << ",left_q_sent_" << i;
    for (int i = 0; i < kDof; ++i) file_ << ",right_q_sent_" << i;
    file_ << ",left_error_code,right_error_code";
    writeCartesianSolveHeader(file_, "left");
    writeCartesianSolveHeader(file_, "right");
    writeArmProfilingHeader(file_, "left");
    writeArmProfilingHeader(file_, "right");
    writeTcpPoseTargetDebugHeader(file_, "left");
    writeTcpPoseTargetDebugHeader(file_, "right");
    writeInitMotionHeader(file_);
    file_ << ",sched_wake_time_ns,prev_sleep_enter_time_ns"
             ",wake_latency_us,sleep_entry_margin_us"
             ",left_pre_send_us,right_pre_send_us";
    file_ << '\n';
}

namespace {
double ageUs(uint64_t newer_ns, uint64_t older_ns) {
    if (newer_ns == 0 || older_ns == 0 || newer_ns < older_ns) return 0.0;
    return static_cast<double>(newer_ns - older_ns) / 1000.0;
}

double signedDiffUs(uint64_t a_ns, uint64_t b_ns) {
    return static_cast<double>(static_cast<int64_t>(a_ns - b_ns)) / 1000.0;
}

bool sendWithinPeriod(const ServoSample& sample, uint64_t send_end_ns) {
    if (sample.loop_start_time_ns == 0 || send_end_ns == 0 || sample.period_ms <= 0.0) return false;
    const auto period_ns = static_cast<uint64_t>(sample.period_ms * 1'000'000.0);
    return send_end_ns <= sample.loop_start_time_ns + period_ns;
}

bool sendPeriodOverrun(const ServoSample& sample, uint64_t send_end_ns) {
    if (sample.loop_start_time_ns == 0 || send_end_ns == 0 || sample.period_ms <= 0.0) return false;
    return !sendWithinPeriod(sample, send_end_ns);
}

std::string csvEscape(const std::string& value) {
    bool quote = false;
    for (char c : value) {
        if (c == '"' || c == ',' || c == '\n' || c == '\r') {
            quote = true;
            break;
        }
    }
    if (!quote) return value;

    std::string out = "\"";
    for (char c : value) {
        if (c == '"') out += '"';
        out += c;
    }
    out += '"';
    return out;
}

// Per-arm Cartesian IK/solve diagnostics row values. Field order MUST match
// writeCartesianSolveHeader (defined above, near runStamp).
void writeCartesianSolveColumns(std::ostream& os, const CartesianSolveTelemetry& t) {
    os << ',' << t.attempted
       << ',' << t.success
       << ',' << csvEscape(t.status)
       << ',' << t.ik_duration_us
       << ',' << t.ik_iterations
       << ',' << t.ik_timed_out
       << ',' << t.ik_warn_duration_exceeded
       << ',' << t.ik_fail_duration_exceeded
       << ',' << t.ik_min_singular_value
       << ',' << t.ik_applied_damping
       << ',' << t.ik_solution_jump_deg
       << ',' << t.ik_branch_jump_suspected
       << ',' << t.ik_branch_jump_clamped
       << ',' << t.position_error_m
       << ',' << t.orientation_error_rad
       << ',' << t.path_active
       << ',' << t.path_done
       << ',' << csvEscape(t.reason)
       << ',' << t.ik_joint_limit_worst_index
       << ',' << t.ik_joint_limit_worst_margin_deg;
}

struct ArmJointDerivatives {
    JointArray q_sent_velocity_deg_s{};
    JointArray q_sent_accel_deg_s2{};
    JointArray q_sent_jerk_deg_s3{};
    JointArray q_actual_velocity_deg_s{};
    JointArray q_actual_accel_deg_s2{};
    JointArray q_actual_jerk_deg_s3{};
};

void writePoseColumns(std::ostream& os, const std::optional<Pose6D>& pose) {
    if (!pose.has_value()) {
        for (int i = 0; i < 10; ++i) os << ',';
        return;
    }
    os << ',' << pose->x
       << ',' << pose->y
       << ',' << pose->z
       << ',' << pose->rx
       << ',' << pose->ry
       << ',' << pose->rz;
    if (pose->quaternion_xyzw.has_value()) {
        for (double v : *pose->quaternion_xyzw) os << ',' << v;
    } else {
        for (int i = 0; i < 4; ++i) os << ',';
    }
}

void writeJointArrayColumns(std::ostream& os, const JointArray& values) {
    for (double v : values) os << ',' << v;
}

void writeJointArrayBlanks(std::ostream& os) {
    for (int i = 0; i < kDof; ++i) os << ',';
}

std::optional<Pose6D> commandTcpTargetStand(const ArmCommand& command) {
    if (!command.has_tcp_target) return std::nullopt;
    return command.tcp_target_stand;
}

std::optional<Pose6D> tcpActualStand(const RobotState& state) {
    if (state.tcp_actual_stand.has_value()) return state.tcp_actual_stand;
    return state.tcp_stand;
}

void writeArmProfilingColumns(
    std::ostream& os,
    const ArmCommand& command,
    const RobotState& state,
    const CartesianSolveTelemetry& telemetry,
    const ArmJointDerivatives& derivatives) {
    writePoseColumns(os, commandTcpTargetStand(command));
    writePoseColumns(os, telemetry.smd_goal_stand);
    writePoseColumns(os, telemetry.smd_ref_stand);
    writePoseColumns(os, state.tcp_command_stand);
    writePoseColumns(os, tcpActualStand(state));
    writePoseColumns(os, state.tcp_ref_stand);
    os << ',' << csvEscape(telemetry.tcp_target_profile)
       << ',' << telemetry.smd_profile_nf_linear_hz
       << ',' << telemetry.smd_profile_nf_angular_hz
       << ',' << telemetry.smd_profile_velocity_feedforward
       << ',' << telemetry.smd_profile_max_linear_velocity_m_s
       << ',' << telemetry.smd_profile_max_linear_accel_m_s2
       << ',' << telemetry.smd_profile_max_angular_velocity_rad_s
       << ',' << telemetry.smd_profile_max_angular_accel_rad_s2
       << ',' << telemetry.smd_profile_max_goal_lead_m
       << ',' << telemetry.smd_profile_max_goal_lead_rad
       << ',' << telemetry.smd_active
       << ',' << telemetry.smd_velocity_feedforward_used
       << ',' << telemetry.smd_linear_velocity_clipped
       << ',' << telemetry.smd_linear_accel_clipped
       << ',' << telemetry.smd_angular_velocity_clipped
       << ',' << telemetry.smd_angular_accel_clipped
       << ',' << telemetry.smd_goal_linear_velocity_ff_clipped
       << ',' << telemetry.smd_goal_angular_velocity_ff_clipped
       << ',' << telemetry.smd_goal_linear_velocity_norm_m_s
       << ',' << telemetry.smd_goal_angular_velocity_norm_rad_s
       << ',' << telemetry.smd_reanchor_count;
    writePoseColumns(os, telemetry.follower_pf_stand);
    writePoseColumns(os, telemetry.stage_tcp_target_stand);
    os << ',' << telemetry.follower_active
       << ',' << telemetry.follower_wire_seq
       << ',' << telemetry.follower_recv_seq
       << ',' << telemetry.follower_step
       << ',' << telemetry.follower_t_in_seg_sec
       << ',' << telemetry.follower_duration_sec
       << ',' << telemetry.follower_alpha
       << ',' << telemetry.follower_converged
       << ',' << telemetry.follower_stall
       << ',' << telemetry.follower_corner
       << ',' << telemetry.follower_divergence_pos_m
       << ',' << telemetry.follower_divergence_ang_rad
       << ',' << telemetry.follower_reanchor_count
       << ',' << telemetry.safety_intervention_recent
       << ',' << telemetry.output_ma_present
       << ',' << telemetry.output_ma_window;
    if (telemetry.output_ma_present) {
        writeJointArrayColumns(os, telemetry.q_target_before_output_ma_deg);
        writeJointArrayColumns(os, telemetry.q_target_after_output_ma_deg);
    } else {
        writeJointArrayBlanks(os);
        writeJointArrayBlanks(os);
    }
    writeJointArrayColumns(os, derivatives.q_sent_velocity_deg_s);
    writeJointArrayColumns(os, derivatives.q_sent_accel_deg_s2);
    writeJointArrayColumns(os, derivatives.q_sent_jerk_deg_s3);
    writeJointArrayColumns(os, derivatives.q_actual_velocity_deg_s);
    writeJointArrayColumns(os, derivatives.q_actual_accel_deg_s2);
    writeJointArrayColumns(os, derivatives.q_actual_jerk_deg_s3);
}

void writeTcpPoseTargetDebugColumns(std::ostream& os, const CartesianSolveTelemetry& telemetry) {
    const SafetyClampTelemetry& clamp = telemetry.safety_clamp;
    os << ',' << telemetry.ik_branch_jump_rate_limited
       << ',' << telemetry.ik_branch_jump_raw_deg
       << ',' << telemetry.ik_branch_jump_limit_deg
       << ',' << telemetry.ik_branch_jump_scale
       << ',' << telemetry.ik_branch_jump_retry_count;
    writeJointArrayColumns(os, telemetry.q_ik_seed_deg);
    writeJointArrayColumns(os, telemetry.q_ik_raw_solution_deg);
    writeJointArrayColumns(os, telemetry.q_ik_solution_deg);
    writeJointArrayColumns(os, telemetry.q_ik_raw_delta_deg);
    writeJointArrayColumns(os, telemetry.q_ik_delta_deg);
    os << ',' << clamp.present;
    writeJointArrayColumns(os, clamp.q_before_safety_deg);
    writeJointArrayColumns(os, clamp.q_after_joint_limit_deg);
    writeJointArrayColumns(os, clamp.q_after_velocity_limit_deg);
    writeJointArrayColumns(os, clamp.q_after_accel_limit_deg);
    os << ',' << clamp.joint_limit_clamped
       << ',' << clamp.velocity_clamped
       << ',' << clamp.accel_clamped
       << ',' << clamp.joint_limit_clamp_max_delta_deg
       << ',' << clamp.velocity_clamp_max_delta_deg
       << ',' << clamp.accel_clamp_max_delta_deg
       << ',' << clamp.joint_limit_limited_joint
       << ',' << clamp.velocity_limited_joint
       << ',' << clamp.accel_limited_joint;
}

void writeInitMotionColumns(std::ostream& os, const ServoSample& sample) {
    const auto goal_threshold = [](const auto& diag) {
        return diag.goal_nearest_pair_external
            ? diag.goal_clear_threshold_external_m
            : diag.goal_clear_threshold_self_m;
    };
    const auto write_goal_columns = [&](const auto& diag) {
        os << ',' << csvEscape(diag.goal_nearest_pair_name_a)
           << ',' << csvEscape(diag.goal_nearest_pair_name_b)
           << ',' << csvEscape(diag.goal_nearest_pair_category)
           << ',' << diag.goal_nearest_pair_distance_m
           << ',' << goal_threshold(diag)
           << ',' << diag.goal_clear_margin_deficit_m;
    };
    os << ',' << csvEscape(sample.left_mode_before_init_sequencer)
       << ',' << csvEscape(sample.right_mode_before_init_sequencer)
       << ',' << csvEscape(sample.left_mode_after_init_sequencer)
       << ',' << csvEscape(sample.right_mode_after_init_sequencer)
       << ',' << csvEscape(sample.left_joint_target_profile_before_init_sequencer)
       << ',' << csvEscape(sample.right_joint_target_profile_before_init_sequencer)
       << ',' << csvEscape(sample.left_joint_target_profile_after_init_sequencer)
       << ',' << csvEscape(sample.right_joint_target_profile_after_init_sequencer)
       << ',' << csvEscape(sample.init_motion_left.status)
       << ',' << csvEscape(sample.init_motion_right.status)
       << ',' << csvEscape(sample.init_motion.status)
       << ',' << csvEscape(sample.init_motion_left.fail_mode)
       << ',' << csvEscape(sample.init_motion_right.fail_mode)
       << ',' << csvEscape(sample.init_motion.fail_mode)
       << ',' << csvEscape(sample.init_motion_left.message)
       << ',' << csvEscape(sample.init_motion_right.message)
       << ',' << csvEscape(sample.init_motion.message)
       << ',' << sample.init_motion_left.waypoint_index
       << ',' << sample.init_motion_left.waypoint_count
       << ',' << sample.init_motion_right.waypoint_index
       << ',' << sample.init_motion_right.waypoint_count
       << ',' << sample.init_motion.waypoint_index
       << ',' << sample.init_motion.waypoint_count
       << ',' << sample.init_motion_left.dist_to_goal_deg
       << ',' << sample.init_motion_right.dist_to_goal_deg
       << ',' << sample.init_motion.dist_to_goal_deg
       << ',' << csvEscape(sample.non_init_arm_preserved_mode)
       << ',' << sample.single_arm_freeze_other_arm
       << ',' << sample.init_motion_left.clear_threshold_m
       << ',' << sample.init_motion_right.clear_threshold_m
       << ',' << sample.init_motion.clear_threshold_m
       << ',' << sample.init_motion_left.external_clear_threshold_m
       << ',' << sample.init_motion_right.external_clear_threshold_m
       << ',' << sample.init_motion.external_clear_threshold_m
       << ',' << csvEscape(sample.init_motion_left.nearest_pair)
       << ',' << csvEscape(sample.init_motion_right.nearest_pair)
       << ',' << csvEscape(sample.init_motion.nearest_pair)
       << ',' << sample.init_motion_left.nearest_pair_distance_m
       << ',' << sample.init_motion_right.nearest_pair_distance_m
       << ',' << sample.init_motion.nearest_pair_distance_m
       << ',' << sample.init_motion_left.nearest_pair_external
       << ',' << sample.init_motion_right.nearest_pair_external
       << ',' << sample.init_motion.nearest_pair_external;
    write_goal_columns(sample.init_motion_left);
    write_goal_columns(sample.init_motion_right);
    write_goal_columns(sample.init_motion);
    os << ',' << sample.self_collision_min_clearance_m
       << ',' << csvEscape(sample.self_collision_pair);
}
}  // namespace

void ServoLogger::writeSample(const ServoSample& sample) {
    const uint64_t derivative_time_ns =
        sample.loop_end_time_ns > 0 ? sample.loop_end_time_ns : sample.loop_start_time_ns;
    const bool can_derive =
        derivative_state_valid_ &&
        derivative_prev_time_ns_ > 0 &&
        derivative_time_ns > derivative_prev_time_ns_;
    const double derivative_dt_sec = can_derive
        ? static_cast<double>(derivative_time_ns - derivative_prev_time_ns_) / 1'000'000'000.0
        : 0.0;
    ArmJointDerivatives left_derivatives;
    ArmJointDerivatives right_derivatives;
    const auto compute_joint_derivatives = [&](
        const JointArray& q,
        JointArray& prev_q,
        JointArray& prev_velocity,
        JointArray& prev_accel,
        JointArray& out_velocity,
        JointArray& out_accel,
        JointArray& out_jerk) {
        for (int i = 0; i < kDof; ++i) {
            double velocity = 0.0;
            double accel = 0.0;
            double jerk = 0.0;
            if (can_derive &&
                derivative_dt_sec > 0.0 &&
                std::isfinite(q[i]) &&
                std::isfinite(prev_q[i]) &&
                std::isfinite(prev_velocity[i]) &&
                std::isfinite(prev_accel[i])) {
                velocity = (q[i] - prev_q[i]) / derivative_dt_sec;
                accel = (velocity - prev_velocity[i]) / derivative_dt_sec;
                jerk = (accel - prev_accel[i]) / derivative_dt_sec;
            }
            out_velocity[i] = velocity;
            out_accel[i] = accel;
            out_jerk[i] = jerk;
        }
        prev_q = q;
        prev_velocity = out_velocity;
        prev_accel = out_accel;
    };
    compute_joint_derivatives(
        sample.left_sent_q_deg,
        left_derivative_.prev_q_sent_deg,
        left_derivative_.prev_q_sent_velocity_deg_s,
        left_derivative_.prev_q_sent_accel_deg_s2,
        left_derivatives.q_sent_velocity_deg_s,
        left_derivatives.q_sent_accel_deg_s2,
        left_derivatives.q_sent_jerk_deg_s3);
    compute_joint_derivatives(
        sample.left_state.q_actual_deg,
        left_derivative_.prev_q_actual_deg,
        left_derivative_.prev_q_actual_velocity_deg_s,
        left_derivative_.prev_q_actual_accel_deg_s2,
        left_derivatives.q_actual_velocity_deg_s,
        left_derivatives.q_actual_accel_deg_s2,
        left_derivatives.q_actual_jerk_deg_s3);
    compute_joint_derivatives(
        sample.right_sent_q_deg,
        right_derivative_.prev_q_sent_deg,
        right_derivative_.prev_q_sent_velocity_deg_s,
        right_derivative_.prev_q_sent_accel_deg_s2,
        right_derivatives.q_sent_velocity_deg_s,
        right_derivatives.q_sent_accel_deg_s2,
        right_derivatives.q_sent_jerk_deg_s3);
    compute_joint_derivatives(
        sample.right_state.q_actual_deg,
        right_derivative_.prev_q_actual_deg,
        right_derivative_.prev_q_actual_velocity_deg_s,
        right_derivative_.prev_q_actual_accel_deg_s2,
        right_derivatives.q_actual_velocity_deg_s,
        right_derivatives.q_actual_accel_deg_s2,
        right_derivatives.q_actual_jerk_deg_s3);
    derivative_prev_time_ns_ = derivative_time_ns;
    derivative_state_valid_ = derivative_time_ns > 0;

    file_ << sample.tick << ','
          << sample.loop_start_time_ns << ','
          << sample.loop_end_time_ns << ','
          << sample.period_ms << ','
          << sample.jitter_ms << ','
          << sample.filter_dt_ms << ','
          << toString(sample.safety_verdict) << ','
          << toString(sample.motion_state) << ','
          << sample.fault_latched << ','
          << csvEscape(sample.fault_reason) << ','
          << droppedSamples() << ','
          << sample.command.seq << ','
          << toString(sample.command.left.mode) << ','
          << toString(sample.command.right.mode) << ','
          << sample.left_send_ok << ','
          << sample.right_send_ok << ',';
    if (sample.latched_fault_context.has_value()) {
        const LatchedFaultContextSnapshot& context = *sample.latched_fault_context;
        file_ << context.verdict << ','
              << context.domain << ','
              << context.arm << ','
              << context.backend_op << ','
              << context.backend_error_kind << ','
              << csvEscape(context.backend_error_name) << ','
              << csvEscape(context.backend_error_code) << ','
              << context.retryable << ','
              << context.recoverable << ','
              << context.robot_fault << ','
              << context.transport_fault << ','
              << context.state_after_source << ','
              << csvEscape(context.reason) << ',';
    } else {
        file_ << ",,,,,,,,,,,,,";
    }
    file_ << sample.left_send_start_ns << ','
          << sample.left_send_end_ns << ','
          << sample.right_send_start_ns << ','
          << sample.right_send_end_ns << ','
          << sample.send_skew_us << ','
          << sample.left_send_duration_us << ','
          << sample.right_send_duration_us << ','
          << toString(sample.left_last_send.ack_policy) << ','
          << toString(sample.right_last_send.ack_policy) << ','
          << sample.left_last_send.ack_observed << ','
          << sample.right_last_send.ack_observed << ','
          << sample.left_last_send.controller_acceptance_observed << ','
          << sample.right_last_send.controller_acceptance_observed << ','
          << sample.left_last_send.ack_wait_duration_us << ','
          << sample.right_last_send.ack_wait_duration_us << ','
          << sample.left_last_send.rbpodo_waiting_ack << ','
          << sample.right_last_send.rbpodo_waiting_ack << ','
          << csvEscape(sample.left_last_send.acceptance_semantics) << ','
          << csvEscape(sample.right_last_send.acceptance_semantics) << ','
          << ageUs(sample.loop_end_time_ns, sample.left_state.host_time_ns) << ','
          << ageUs(sample.loop_end_time_ns, sample.right_state.host_time_ns) << ','
          << ageUs(sample.loop_end_time_ns, sample.left_send_end_ns) << ','
          << ageUs(sample.loop_end_time_ns, sample.right_send_end_ns) << ','
          << sendWithinPeriod(sample, sample.left_send_end_ns) << ','
          << sendWithinPeriod(sample, sample.right_send_end_ns) << ','
          << sendPeriodOverrun(sample, sample.left_send_end_ns) << ','
          << sendPeriodOverrun(sample, sample.right_send_end_ns) << ','
          << "" << ','
          << "" << ','
          << sendWithinPeriod(sample, sample.left_send_end_ns) << ','
          << sendWithinPeriod(sample, sample.right_send_end_ns) << ','
          << sample.send_skew_us << ','
          << sample.left_last_read.duration_us << ','
          << sample.right_last_read.duration_us;
    for (double v : sample.left_state.q_actual_deg) file_ << ',' << v;
    for (double v : sample.right_state.q_actual_deg) file_ << ',' << v;
    for (double v : sample.left_sent_q_deg) file_ << ',' << v;
    for (double v : sample.right_sent_q_deg) file_ << ',' << v;
    file_ << ',' << sample.left_state.error_code << ',' << sample.right_state.error_code;
    writeCartesianSolveColumns(file_, sample.left_cartesian_solve);
    writeCartesianSolveColumns(file_, sample.right_cartesian_solve);
    writeArmProfilingColumns(
        file_,
        sample.command.left,
        sample.left_state,
        sample.left_cartesian_solve,
        left_derivatives);
    writeArmProfilingColumns(
        file_,
        sample.command.right,
        sample.right_state,
        sample.right_cartesian_solve,
        right_derivatives);
    writeTcpPoseTargetDebugColumns(file_, sample.left_cartesian_solve);
    writeTcpPoseTargetDebugColumns(file_, sample.right_cartesian_solve);
    writeInitMotionColumns(file_, sample);
    // Wake/send jitter decomposition (all stamps share the steady clock epoch):
    //   wake_latency_us       = loop_start - sched_wake  (how late this tick woke)
    //   sleep_entry_margin_us = sched_wake - prev_sleep_enter (signed; negative means
    //                           the PREVIOUS tick overran into this tick's slot, so a
    //                           large wake_latency is overrun, not scheduler latency)
    //   pre_send_us           = send_start - loop_start (compute time before the send)
    const double wake_latency_us = sample.sched_wake_time_ns > 0
        ? signedDiffUs(sample.loop_start_time_ns, sample.sched_wake_time_ns)
        : 0.0;
    const double sleep_entry_margin_us =
        (sample.sched_wake_time_ns > 0 && sample.prev_sleep_enter_time_ns > 0)
            ? signedDiffUs(sample.sched_wake_time_ns, sample.prev_sleep_enter_time_ns)
            : 0.0;
    const double left_pre_send_us = sample.left_send_start_ns > 0
        ? signedDiffUs(sample.left_send_start_ns, sample.loop_start_time_ns)
        : 0.0;
    const double right_pre_send_us = sample.right_send_start_ns > 0
        ? signedDiffUs(sample.right_send_start_ns, sample.loop_start_time_ns)
        : 0.0;
    file_ << ',' << sample.sched_wake_time_ns
          << ',' << sample.prev_sleep_enter_time_ns
          << ',' << wake_latency_us
          << ',' << sleep_entry_margin_us
          << ',' << left_pre_send_us
          << ',' << right_pre_send_us;
    file_ << '\n';
}

}  // namespace rb_servo
