#include "rb_servo/logging/servo_logger.hpp"

#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
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
       << ',' << side << "_cart_ik_joint_limit_margin_deg"
       << ',' << side << "_cart_ik_joint_limit_pinned"
       << ',' << side << "_cart_ik_limit_relief_weight"
       << ',' << side << "_cart_ik_limit_avoidance_step_deg"
       << ',' << side << "_cart_ik_pinned_lowpass_active";
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

void writeDeltaTwistVecHeader(std::ostream& os, const char* side, const char* name) {
    os << ',' << side << '_' << name << "_dx_m"
       << ',' << side << '_' << name << "_dy_m"
       << ',' << side << '_' << name << "_dz_m"
       << ',' << side << '_' << name << "_drx_rad"
       << ',' << side << '_' << name << "_dry_rad"
       << ',' << side << '_' << name << "_drz_rad";
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
       << ',' << side << "_follower_controller"
       << ',' << side << "_follower_wire_seq"
       << ',' << side << "_follower_recv_seq"
       << ',' << side << "_follower_step"
       << ',' << side << "_follower_t_in_seg_sec"
       << ',' << side << "_follower_duration_sec"
       << ',' << side << "_follower_alpha"
       << ',' << side << "_follower_converged"
       << ',' << side << "_follower_stall"
       << ',' << side << "_follower_corner"
       << ',' << side << "_follower_output_smd_active"
       << ',' << side << "_follower_output_smd_lag_m"
       << ',' << side << "_follower_output_smd_lag_rad"
       << ',' << side << "_follower_prefilter_stand_x_m"
       << ',' << side << "_follower_prefilter_stand_y_m"
       << ',' << side << "_follower_prefilter_stand_z_m"
       << ',' << side << "_follower_divergence_pos_m"
       << ',' << side << "_follower_divergence_ang_rad"
       << ',' << side << "_follower_projection_error_m"
       << ',' << side << "_follower_projection_error_rad"
       << ',' << side << "_follower_projection_error_count"
       << ',' << side << "_follower_actual_lead_m"
       << ',' << side << "_follower_actual_lead_rad"
       << ',' << side << "_follower_actual_lead_error_count"
       << ',' << side << "_follower_reanchor_count"
       << ',' << side << "_follower_divergence_reanchor_count"
       << ',' << side << "_follower_lead_reanchor_explained_count"
       << ',' << side << "_follower_lead_reanchor_unexplained_count"
       << ',' << side << "_follower_warm_resume_count"
       << ',' << side << "_safety_intervention_recent"
       << ',' << side << "_cartesian_solve_blocked_recent"
       << ',' << side << "_command_throttled_recent"
       << ',' << side << "_delta_twist_pending_linear_norm_m"
       << ',' << side << "_delta_twist_pending_angular_norm_rad"
       << ',' << side << "_delta_twist_step_linear_norm_m"
       << ',' << side << "_delta_twist_step_angular_norm_rad"
       << ',' << side << "_delta_twist_step_yaw_rad";
    writeDeltaTwistVecHeader(os, side, "delta_twist_step");
    os << ',' << side << "_delta_twist_realized_linear_norm_m"
       << ',' << side << "_delta_twist_realized_angular_norm_rad"
       << ',' << side << "_delta_twist_realized_yaw_rad";
    writeDeltaTwistVecHeader(os, side, "delta_twist_realized");
    os << ',' << side << "_delta_twist_realized_linear_ratio"
       << ',' << side << "_delta_twist_realized_angular_ratio"
       << ',' << side << "_delta_twist_realized_yaw_ratio"
       << ',' << side << "_delta_twist_phase_sec"
       << ',' << side << "_delta_twist_step_kind"
       << ',' << side << "_delta_twist_normal_consumed"
       << ',' << side << "_delta_twist_reserve_consumed"
       << ',' << side << "_delta_twist_xi_ref_linear_norm_m_s"
       << ',' << side << "_delta_twist_xi_ref_angular_norm_rad_s"
       << ',' << side << "_delta_twist_xi_cmd_linear_norm_m_s"
       << ',' << side << "_delta_twist_xi_cmd_angular_norm_rad_s"
       << ',' << side << "_delta_twist_saturated"
       << ',' << side << "_delta_twist_lead_linear_norm_m"
       << ',' << side << "_delta_twist_lead_angular_norm_rad"
       << ',' << side << "_delta_twist_feedback_source"
       << ',' << side << "_delta_twist_pending_clamped"
       << ',' << side << "_delta_twist_residual_cleared_on_frame"
       << ',' << side << "_delta_twist_min_time_to_go_used"
       << ',' << side << "_delta_twist_lin_feedback_cos"
       << ',' << side << "_delta_twist_ang_feedback_cos"
       << ',' << side << "_delta_twist_xi_ref_clamped_norm"
       << ',' << side << "_delta_twist_xi_cmd_clamped_norm"
       << ',' << side << "_delta_twist_frame_rows"
       << ',' << side << "_delta_twist_normal_budget"
       << ',' << side << "_delta_twist_total_budget"
       << ',' << side << "_delta_twist_steps_remaining"
       << ',' << side << "_delta_twist_clamp_mask"
       << ',' << side << "_delta_twist_accel_cmd_x_m_s2"
       << ',' << side << "_delta_twist_accel_cmd_y_m_s2"
       << ',' << side << "_delta_twist_accel_cmd_z_m_s2"
       << ',' << side << "_delta_twist_accel_cmd_rx_rad_s2"
       << ',' << side << "_delta_twist_accel_cmd_ry_rad_s2"
       << ',' << side << "_delta_twist_accel_cmd_rz_rad_s2"
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
       << ",self_collision_pair"
       << ",self_collision_closest_pair";
}


// One arm's F/T + force-control columns. EVERY wrench column names its FRAME and its
// REFERENCE POINT in the column name itself, because a wrench column called
// "<side>_fz" is unreadable six months later: three different surfaces in this
// pipeline all have an fz and they disagree by the tool's weight and a lever arm.
void writeWrenchHeader(std::ostream& os, const char* side, const char* name) {
    os << ',' << side << '_' << name << "_fx_n"
       << ',' << side << '_' << name << "_fy_n"
       << ',' << side << '_' << name << "_fz_n"
       << ',' << side << '_' << name << "_tx_nm"
       << ',' << side << '_' << name << "_ty_nm"
       << ',' << side << '_' << name << "_tz_nm";
}

void writeForceHeader(std::ostream& os, const char* side) {
    os << ',' << side << "_ft_enabled"
       << ',' << side << "_ft_connected"
       << ',' << side << "_ft_connect_reason"
       << ',' << side << "_ft_axes_det";
    // (1) raw, sensor axes @SRO - what the wire carried, mapped only.
    writeWrenchHeader(os, side, "ft_raw_sensor");
    // the tool-gravity term subtracted this tick, sensor frame @SRO.
    writeWrenchHeader(os, side, "ft_gravity_sensor");
    // (2) compensated @SRO, pre- and post-deadzone.
    writeWrenchHeader(os, side, "ft_comp_sensor_nodz");
    writeWrenchHeader(os, side, "ft_comp_sensor");
    // (3) compensated @TCP in TOOL axes - THE SURFACE THE FORCE LAW CONSUMES.
    writeWrenchHeader(os, side, "ft_comp_tcp");
    // (4) the same wrench in STAND axes - the frame the overlay integrates in.
    writeWrenchHeader(os, side, "ft_comp_stand");
    // the bias in force right now, and where it came from.
    writeWrenchHeader(os, side, "ft_bias");
    os << ',' << side << "_ft_bias_valid"
       << ',' << side << "_ft_bias_source"
       << ',' << side << "_ft_bias_generation"
       << ',' << side << "_ft_tare_state"
       << ',' << side << "_ft_tare_samples"
       << ',' << side << "_ft_auto_tare_stage"
       << ',' << side << "_ft_load_force_n"
       << ',' << side << "_ft_load_mass_kg"
       << ',' << side << "_ft_load_settled"
       << ',' << side << "_ft_tool_mass_kg"
       // ---- the force law ----
       << ',' << side << "_fc_enabled"
       << ',' << side << "_fc_covered"
       << ',' << side << "_fc_coverage_reason"
       << ',' << side << "_fc_law"
       << ',' << side << "_fc_compose_applied"
       << ',' << side << "_fc_dev_x_m"
       << ',' << side << "_fc_dev_y_m"
       << ',' << side << "_fc_dev_z_m"
       << ',' << side << "_fc_dev_norm_m"
       << ',' << side << "_fc_dev_rx_rad"
       << ',' << side << "_fc_dev_ry_rad"
       << ',' << side << "_fc_dev_rz_rad"
       << ',' << side << "_fc_dev_norm_rad"
       << ',' << side << "_fc_vel_x_m_s"
       << ',' << side << "_fc_vel_y_m_s"
       << ',' << side << "_fc_vel_z_m_s"
       << ',' << side << "_fc_bounded"
       << ',' << side << "_fc_osc_frozen"
       << ',' << side << "_fc_osc_trips"
       << ',' << side << "_fc_recover_streak"
       << ',' << side << "_fc_recover_needed"
       << ',' << side << "_fc_fence_m"
       << ',' << side << "_fc_fence_rad"
       << ',' << side << "_fc_gate_translation"
       << ',' << side << "_fc_gate_rotation"
       << ',' << side << "_fc_gate_force_n"
       << ',' << side << "_fc_gate_torque_nm"
       << ',' << side << "_fc_gate_closed"
       << ',' << side << "_fc_gate_removed_m"
       // The wrench the LAW consumed, after the contact-shock low-pass, beside
       // the RAW one in the ft_ block above: the pair is what shows how much
       // shock the filter took out (0 Hz => filter off, the two are equal).
       << ',' << side << "_fc_wrench_filter_hz"
       << ',' << side << "_fc_wrench_filt_fx_n"
       << ',' << side << "_fc_wrench_filt_fy_n"
       << ',' << side << "_fc_wrench_filt_fz_n"
       // The two tracking errors, beside the force columns because the force path is
       // what makes them diverge: a compliant command deliberately leaves the arm.
       << ',' << side << "_track_command_vs_actual_deg"
       << ',' << side << "_track_reference_vs_actual_deg"
       << ',' << side << "_track_reference_valid"
       << ',' << side << "_track_latch_cause";
}

}  // namespace

ServoLogger::ServoLogger(const LoggingConfig& config) : config_(config) {}

ServoLogger::~ServoLogger() {
    stop();
}

bool ServoLogger::start() {
    if (!config_.enable) return true;
    if (running_) return true;

    // Preallocate the ring once, off the RT path, so push() never allocates.
    // queue_capacity == 0 leaves the ring empty and push drops every sample
    // (preserves the prior capacity==0 behavior). Sized to queue_capacity, it
    // holds ~queue_capacity/rate seconds of backlog before evicting oldest.
    ring_.assign(config_.queue_capacity, ServoSample{});
    head_ = 0;
    size_ = 0;
    drain_buffer_.clear();
    drain_buffer_.reserve(config_.queue_capacity);

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
    // 9 significant digits for every double column. The default (6) quantizes a
    // joint logged at 226 deg to 0.001 deg -- exactly the floor that made the
    // LPF-off q_actual jerk question undecidable offline and that hides the
    // wire-precision staircase (servo_j_text_precision) in q_ref. ~15-25% larger
    // files, accepted for offline smoothness analysis.
    file_ << std::setprecision(9);
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
        // try_to_lock, never block: if the consumer holds the mutex (it can be
        // preempted while draining on a loaded non-RT core), a plain lock would
        // priority-invert the FIFO-80 servo loop for milliseconds. Dropping a
        // log row (counted below, surfaced as logger_dropped_samples) is the
        // correct RT trade — the servo tick must not stall to log itself.
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock.owns_lock()) {
            dropped_samples_.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        if (ring_.empty()) {  // queue_capacity == 0: logging disabled at the queue
            dropped_samples_.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        if (size_ >= ring_.size()) {  // full: evict oldest, no allocation
            head_ = (head_ + 1) % ring_.size();
            --size_;
            dropped_samples_.fetch_add(1, std::memory_order_relaxed);
        }
        const size_t write_index = (head_ + size_) % ring_.size();
        ring_[write_index] = sample;  // fixed-slot copy; deque node growth is gone
        ++size_;
    }
    cv_.notify_one();
}

uint64_t ServoLogger::droppedSamples() const {
    return dropped_samples_.load(std::memory_order_relaxed);
}

void ServoLogger::drainInto(std::vector<ServoSample>& out) {
    out.clear();
    std::lock_guard<std::mutex> lock(mutex_);
    if (ring_.empty() || size_ == 0) return;
    for (size_t i = 0; i < size_; ++i) {
        out.push_back(ring_[(head_ + i) % ring_.size()]);
    }
    head_ = 0;
    size_ = 0;
}

void ServoLogger::threadMain() {
    while (running_) {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait_for(lock, std::chrono::milliseconds(config_.flush_period_ms), [&] {
                return size_ > 0 || !running_;
            });
        }
        // Copy the pending batch out under the lock, then write it with the lock
        // released so the RT producer's try_lock in push() rarely contends.
        drainInto(drain_buffer_);
        for (const ServoSample& sample : drain_buffer_) {
            writeSample(sample);
        }
        if (file_) file_.flush();
    }
    // FINAL DRAIN. `while (running_)` is evaluated before the wait, so stop() can clear
    // the flag in the window between one iteration finishing and the next condition
    // check, and the thread then exits with whatever was pushed since the last drain
    // still in the ring -- silently dropping the tail of every log, and making
    // test_servo_logger_columns fail about one run in five (measured 2026-09-02).
    // push() already refuses samples once running_ is false, so one pass is enough.
    drainInto(drain_buffer_);
    for (const ServoSample& sample : drain_buffer_) {
        writeSample(sample);
    }
    if (file_) file_.flush();
}

void ServoLogger::writeHeader() {
    file_ << "tick,loop_start_time_ns,loop_end_time_ns,period_ms,jitter_ms,filter_dt_ms,safety_verdict,motion_state,fault_latched,fault_reason,logger_dropped_samples,command_seq,left_mode,right_mode";
    file_ << ",command_buffer_result,command_buffer_pending_lifecycle_count,command_buffer_skipped_lifecycle_count";
    file_ << ",command_buffer_returned_seq,command_buffer_returned_left_mode,command_buffer_returned_right_mode,command_buffer_returned_host_time_ns,command_buffer_returned_age_ms,command_buffer_returned_client_send_age_ms";
    file_ << ",command_buffer_latest_seq,command_buffer_latest_left_mode,command_buffer_latest_right_mode,command_buffer_latest_host_time_ns,command_buffer_latest_age_ms,command_buffer_latest_timeout_ms,command_buffer_latest_timeout_valid,command_buffer_latest_usable,command_buffer_latest_client_send_age_ms";
    file_ << ",command_buffer_lifecycle_seq,command_buffer_lifecycle_left_mode,command_buffer_lifecycle_right_mode,command_buffer_lifecycle_host_time_ns,command_buffer_lifecycle_age_ms,command_buffer_lifecycle_timeout_ms,command_buffer_lifecycle_timeout_valid,command_buffer_lifecycle_usable";
    file_ << ",command_buffer_external_boxes_pending,command_buffer_external_boxes_consumed,command_buffer_external_boxes_applied,command_buffer_external_boxes_seq,command_buffer_external_boxes_left_mode,command_buffer_external_boxes_right_mode,command_buffer_external_boxes_host_time_ns,command_buffer_external_boxes_age_ms,command_buffer_external_boxes_client_send_age_ms";
    file_ << ",chunk_frame_wire_seq,chunk_frame_recv_seq,chunk_frame_policy_dt_sec,chunk_frame_horizon,chunk_frame_execute_steps,chunk_frame_runway_steps,chunk_frame_age_ms,chunk_frame_interarrival_ms";
    file_ << ",chunk_inference_seq,chunk_inference_queue_wait_ms,chunk_inference_latency_ms,chunk_inference_ready_wait_ms,chunk_inference_period_ms,chunk_inference_period_jitter_ms,chunk_inference_stall_count";
    file_ << ",chunk_camera_bundle_seq,chunk_camera_bundle_age_ms,chunk_camera_max_skew_ms,chunk_camera_left_frame_number,chunk_camera_right_frame_number,chunk_camera_left_frame_age_ms,chunk_camera_right_frame_age_ms,chunk_camera_left_focus_score,chunk_camera_right_focus_score";
    file_ << ",left_send_ok,right_send_ok";
    file_ << ",fault_context_verdict,fault_context_domain,fault_context_arm,fault_context_backend_op,fault_context_backend_error_kind,fault_context_backend_error_name,fault_context_backend_error_code,fault_context_retryable,fault_context_recoverable,fault_context_robot_fault,fault_context_transport_fault,fault_context_state_after_source,fault_context_reason";
    file_ << ",left_send_start_ns,left_send_end_ns,right_send_start_ns,right_send_end_ns,send_skew_us,left_send_duration_us,right_send_duration_us";
    file_ << ",left_ack_policy,right_ack_policy,left_ack_observed,right_ack_observed,left_controller_acceptance_observed,right_controller_acceptance_observed,left_ack_wait_duration_us,right_ack_wait_duration_us,left_rbpodo_waiting_ack,right_rbpodo_waiting_ack,left_send_acceptance_semantics,right_send_acceptance_semantics";
    file_ << ",left_state_age_us,right_state_age_us,left_send_result_age_us,right_send_result_age_us";
    file_ << ",left_send_within_period,right_send_within_period,left_send_period_overrun,right_send_period_overrun,left_send_command_deadline_missed,right_send_command_deadline_missed";
    file_ << ",left_send_deadline_hit,right_send_deadline_hit,dispatch_skew_us,left_worker_loop_read_duration_us,right_worker_loop_read_duration_us";
    file_ << ",left_reqdata_timing_available,left_reqdata_exchange_sequence,left_reqdata_timing_source,left_reqdata_call_start_steady_ns,left_reqdata_call_start_system_ns,left_reqdata_call_return_steady_ns,left_reqdata_call_return_system_ns,left_reqdata_call_duration_us";
    file_ << ",right_reqdata_timing_available,right_reqdata_exchange_sequence,right_reqdata_timing_source,right_reqdata_call_start_steady_ns,right_reqdata_call_start_system_ns,right_reqdata_call_return_steady_ns,right_reqdata_call_return_system_ns,right_reqdata_call_duration_us";
    for (int i = 0; i < kDof; ++i) file_ << ",left_q_actual_" << i;
    for (int i = 0; i < kDof; ++i) file_ << ",right_q_actual_" << i;
    for (int i = 0; i < kDof; ++i) file_ << ",left_q_sent_" << i;
    for (int i = 0; i < kDof; ++i) file_ << ",right_q_sent_" << i;
    // Rainbow controller reference readback (rbpodo sdata.jnt_ref, mirrored to
    // the state JSON as q_ref_deg/q_target_deg). This is the only joint-space
    // window into what the control box did with the servo_j stream, so offline
    // box-latency analysis needs it at tick rate alongside q_sent/q_actual.
    for (int i = 0; i < kDof; ++i) file_ << ",left_q_ref_" << i;
    for (int i = 0; i < kDof; ++i) file_ << ",right_q_ref_" << i;
    file_ << ",left_q_ref_valid,right_q_ref_valid,left_q_actual_valid,right_q_actual_valid";
    // WHICH read path produced this tick's state, and whether it was a fresh frame
    // or a held cache. One of:
    //   CobotData.request_data          blocking path, fresh
    //   last_state_cache (read-miss hold)  blocking path, timed out -> held
    //   CobotData.pipelined             pipelined path, fresh frame drained
    //   CobotData.pipelined(held)       pipelined path, nothing arrived -> held
    // The hold fraction is the number that decides whether the pipelined read is
    // usable on real hardware, and `state_age_us` alone cannot show it: a held
    // state keeps its ORIGINAL host_time_ns, so age is honest, but a fresh frame
    // and a held one are otherwise indistinguishable in the log.
    file_ << ",left_state_source,right_state_source";
    // The control box's OWN activation stage, straight off the data frame.
    // `init_state_info == 6` is "activation done / servo on" -- the same field and
    // the same test controller-manager gates its servo_j stream on
    // (RobotLink.cpp: status.init_state = s.init_state_info, servo_on() == 6).
    // `servo_enabled` is rbpodo_backend.cpp's precomputed form of exactly that.
    //
    // Logged to answer one open question: on 2026-08-26 the box accepted ~131
    // commands over 264 ms while reporting RBACK fill 0, then reported the whole
    // backlog at once. If it was below stage 6 for that window, gating the stream
    // on activation removes the backlog at the source. If it was already 6, that
    // gate would change nothing and the cause is elsewhere -- so this column is
    // what decides between those two, before any gate is written.
    // -1 = the backend published no diagnostics this tick, which is NOT stage 0.
    file_ << ",left_init_state_info,right_init_state_info"
          << ",left_servo_enabled,right_servo_enabled";
    // Rainbow control-box command-queue occupancy from the RBACK ACK stream
    // (firmware v8.7.3+ reports a meaningful fill). -1 means no RBACK has been
    // parsed on this connection -- distinct from a genuine empty-queue 0.
    for (const char* side : {"left", "right"}) {
        file_ << ',' << side << "_rback_observed"
              << ',' << side << "_rback_fill"
              << ',' << side << "_rback_fill_min"
              << ',' << side << "_rback_fill_max"
              << ',' << side << "_rback_seq"
              << ',' << side << "_rback_parsed_total"
              << ',' << side << "_rback_drained_total"
              << ',' << side << "_rback_malformed_total"
              << ',' << side << "_rback_drained_this_send"
              << ',' << side << "_rback_parsed_this_send";
    }
    // The queue-sync CONTROLLER beside the plant it regulates. `qsync_trim_us` is
    // the actuator, so a fill that misbehaves can be attributed: trim pinned at
    // adj_clamp_us is saturation, a wound-up integral is a clock mismatch the law
    // cannot absorb, and a phase that is not `track` means it is not regulating at
    // all. The event counters are cumulative, so they are read as steps.
    for (const char* side : {"left", "right"}) {
        file_ << ',' << side << "_qsync_enabled"
              << ',' << side << "_qsync_trim_us"
              << ',' << side << "_qsync_fill_lpf"
              << ',' << side << "_qsync_integral_us"
              << ',' << side << "_qsync_last_fill"
              << ',' << side << "_qsync_fill_valid"
              << ',' << side << "_qsync_phase"
              << ',' << side << "_qsync_locked"
              << ',' << side << "_qsync_stale_cycles"
              << ',' << side << "_qsync_underrun_events"
              << ',' << side << "_qsync_warn_events"
              << ',' << side << "_qsync_dip_events"
              << ',' << side << "_qsync_dip_last_min"
              << ',' << side << "_qsync_dip_last_ms"
              << ',' << side << "_qsync_stall_events"
              << ',' << side << "_qsync_highwater_events"
              << ',' << side << "_qsync_redrain_events"
              << ',' << side << "_qsync_no_consumption_events";
    }
    // Worker mailbox/wire accounting. The loop enqueues at its own 500 Hz clock
    // while the worker dispatches at the box-locked cadence (~499.35 Hz under
    // queue sync), so `pending_overwrites_total` counts setpoints that never
    // reached the wire (skips: the box FIFO plays a 2-tick step) and
    // `repeated_sends_total` counts wire holds on an empty-mailbox cadence
    // tick. `wire_send_start/end_ns` bracket the worker's actual
    // backend->sendServoJ() call -- left/right_send_start_ns above is the
    // LOOP-side enqueue stamp, NOT the wire instant.
    for (const char* side : {"left", "right"}) {
        file_ << ',' << side << "_worker_pending_overwrites_total"
              << ',' << side << "_worker_repeated_sends_total"
              << ',' << side << "_worker_wire_dispatches_total"
              << ',' << side << "_worker_wire_send_start_ns"
              << ',' << side << "_worker_wire_send_end_ns"
              << ',' << side << "_worker_interp_active"
              << ',' << side << "_worker_interp_delay_setpoints"
              << ',' << side << "_worker_interp_rebase_total"
              << ',' << side << "_worker_interp_hold_total";
    }
    // The cached state frame's own stamp: consecutive ticks with the SAME value
    // are a readback SAMPLING REPEAT (measured 2026-08-26: ~5 % of moving ticks;
    // its 0-then-2x catch-up pair in q_ref mimics a wire double-step and
    // contaminated the lag-8 event set ~40 %). Filter q_ref analyses on this.
    file_ << ",left_state_host_time_ns,right_state_host_time_ns";
    // Combined geometric velocity projection (ROI/floor/reach/self-collision
    // rows + the trailing global per-joint ceiling): the actuator side of the
    // geometric safety layers. `projection_ceiling_clamped` marks the 1-tick
    // velocity-ceiling step; `projection_min_margin_m` is the closest engaged
    // row's d_now (-1 = no row engaged); `selfcol_verdict_age_ms` is the age of
    // the collision verdict the rows were extrapolated with (-1 = none).
    //
    // `projection_min_margin_m` alone cannot tell a breach from normal geometry:
    // it minimises a RAW clearance across rows whose floors differ 5x (arm<->arm
    // 25 mm, intra-arm 5 mm), and the pre-existing `self_collision_pair` column
    // only carries a side CATEGORY -- measured 2026-08-28 it read "all" on 115118
    // of 115439 rows, i.e. "the nearest pair was inside one arm", naming nothing.
    // So a run reading "30 s below 25 mm" could be entirely ordinary intra-arm
    // geometry. The headroom columns close that: `projection_min_headroom_m` is
    // d_now MINUS that row's own d_hard, so 0 means "at its floor" for every
    // class, `projection_min_headroom_pair` names the two geoms, and
    // `projection_min_headroom_class` / `_d_hard_m` say which floor applied.
    file_ << ",projection_active,projection_constraint_count"
             ",left_projection_correction_deg_s,right_projection_correction_deg_s"
             ",left_projection_applied_correction_deg_s"
             ",right_projection_applied_correction_deg_s"
             ",projection_ceiling_clamped,projection_min_margin_m,selfcol_verdict_age_ms"
             ",projection_min_headroom_m,projection_min_headroom_d_hard_m"
             ",projection_min_headroom_class,projection_min_headroom_pair"
             ",left_plan_gate,right_plan_gate";
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
    writeForceHeader(file_, "left");
    writeForceHeader(file_, "right");
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

void writeWrenchColumns(std::ostream& os, const Wrench6D& w) {
    os << ',' << w.fx << ',' << w.fy << ',' << w.fz
       << ',' << w.tx << ',' << w.ty << ',' << w.tz;
}

void writeForceColumns(std::ostream& os, const FtTelemetry& ft, const ForceControlTelemetry& fc,
                       const SafetyTrackingTelemetry& track) {
    os << ',' << ft.enabled
       << ',' << ft.connected
       << ',' << csvEscape(ft.connect_reason)
       << ',' << ft.axes_determinant;
    writeWrenchColumns(os, ft.raw_sensor);
    writeWrenchColumns(os, ft.gravity_sensor);
    writeWrenchColumns(os, ft.comp_sensor_nodz);
    writeWrenchColumns(os, ft.comp_sensor);
    writeWrenchColumns(os, ft.comp_tcp);
    writeWrenchColumns(os, ft.comp_stand);
    writeWrenchColumns(os, ft.bias);
    os << ',' << ft.bias_valid
       << ',' << csvEscape(ft.bias_source)
       << ',' << ft.bias_generation
       << ',' << csvEscape(ft.tare_state)
       << ',' << ft.tare_samples
       << ',' << csvEscape(ft.auto_tare_stage)
       << ',' << ft.load_force_n
       << ',' << ft.load_mass_kg
       << ',' << ft.load_settled
       << ',' << ft.tool_mass_kg
       << ',' << fc.enabled
       << ',' << fc.covered
       << ',' << csvEscape(fc.coverage_reason)
       << ',' << csvEscape(fc.law)
       << ',' << fc.compose_applied
       << ',' << fc.deviation_m[0]
       << ',' << fc.deviation_m[1]
       << ',' << fc.deviation_m[2]
       << ',' << fc.deviation_norm_m
       << ',' << fc.deviation_rad[0]
       << ',' << fc.deviation_rad[1]
       << ',' << fc.deviation_rad[2]
       << ',' << fc.deviation_norm_rad
       << ',' << fc.velocity_m_s[0]
       << ',' << fc.velocity_m_s[1]
       << ',' << fc.velocity_m_s[2]
       << ',' << fc.bounded
       << ',' << fc.oscillation_frozen
       << ',' << fc.oscillation_trips
       << ',' << fc.coverage_recover_streak
       << ',' << fc.coverage_recover_needed
       << ',' << fc.fence_m
       << ',' << fc.fence_rad
       << ',' << fc.gate_translation
       << ',' << fc.gate_rotation
       << ',' << fc.gate_force_n
       << ',' << fc.gate_torque_nm
       << ',' << fc.gate_closed
       << ',' << fc.gate_removed_m
       << ',' << fc.wrench_filter_hz
       << ',' << fc.wrench_filtered_stand.fx
       << ',' << fc.wrench_filtered_stand.fy
       << ',' << fc.wrench_filtered_stand.fz
       << ',' << track.command_vs_actual_deg
       << ',' << track.reference_vs_actual_deg
       << ',' << track.reference_valid
       << ',' << csvEscape(track.latch_cause);
}

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
       << ',' << t.ik_joint_limit_worst_margin_deg
       << ',' << (t.ik_joint_limit_pinned ? 1 : 0)
       << ',' << t.ik_limit_relief_weight
       << ',' << t.ik_limit_avoidance_step_deg
       << ',' << (t.ik_pinned_lowpass_active ? 1 : 0);
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

void writeDeltaTwistVecColumns(std::ostream& os, const Vec6& value) {
    os << ',' << value.x
       << ',' << value.y
       << ',' << value.z
       << ',' << value.rx
       << ',' << value.ry
       << ',' << value.rz;
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
       << ',' << csvEscape(telemetry.follower_controller)
       << ',' << telemetry.follower_wire_seq
       << ',' << telemetry.follower_recv_seq
       << ',' << telemetry.follower_step
       << ',' << telemetry.follower_t_in_seg_sec
       << ',' << telemetry.follower_duration_sec
       << ',' << telemetry.follower_alpha
       << ',' << telemetry.follower_converged
       << ',' << telemetry.follower_stall
       << ',' << telemetry.follower_corner
       << ',' << telemetry.follower_output_smd_active
       << ',' << telemetry.follower_output_smd_lag_m
       << ',' << telemetry.follower_output_smd_lag_rad;
    if (telemetry.follower_prefilter_stand.has_value()) {
        os << ',' << telemetry.follower_prefilter_stand->x
           << ',' << telemetry.follower_prefilter_stand->y
           << ',' << telemetry.follower_prefilter_stand->z;
    } else {
        os << ",,,";
    }
    os << ',' << telemetry.follower_divergence_pos_m
       << ',' << telemetry.follower_divergence_ang_rad
       << ',' << telemetry.follower_projection_error_m
       << ',' << telemetry.follower_projection_error_rad
       << ',' << telemetry.follower_projection_error_count
       << ',' << telemetry.follower_actual_lead_m
       << ',' << telemetry.follower_actual_lead_rad
       << ',' << telemetry.follower_actual_lead_error_count
       << ',' << telemetry.follower_reanchor_count
       << ',' << telemetry.follower_divergence_reanchor_count
       << ',' << telemetry.follower_lead_reanchor_explained_count
       << ',' << telemetry.follower_lead_reanchor_unexplained_count
       << ',' << telemetry.follower_warm_resume_count
       << ',' << telemetry.safety_intervention_recent
       << ',' << telemetry.cartesian_solve_blocked_recent
       << ',' << telemetry.command_throttled_recent
       << ',' << telemetry.delta_twist_pending_linear_norm_m
       << ',' << telemetry.delta_twist_pending_angular_norm_rad
       << ',' << telemetry.delta_twist_step_linear_norm_m
       << ',' << telemetry.delta_twist_step_angular_norm_rad
       << ',' << telemetry.delta_twist_step_yaw_rad;
    writeDeltaTwistVecColumns(os, telemetry.delta_twist_step_delta);
    os << ',' << telemetry.delta_twist_realized_linear_norm_m
       << ',' << telemetry.delta_twist_realized_angular_norm_rad
       << ',' << telemetry.delta_twist_realized_yaw_rad;
    writeDeltaTwistVecColumns(os, telemetry.delta_twist_realized_delta);
    os << ',' << telemetry.delta_twist_realized_linear_ratio
       << ',' << telemetry.delta_twist_realized_angular_ratio
       << ',' << telemetry.delta_twist_realized_yaw_ratio
       << ',' << telemetry.delta_twist_phase_sec
       << ',' << telemetry.delta_twist_step_kind
       << ',' << telemetry.delta_twist_normal_consumed
       << ',' << telemetry.delta_twist_reserve_consumed
       << ',' << telemetry.delta_twist_xi_ref_linear_norm_m_s
       << ',' << telemetry.delta_twist_xi_ref_angular_norm_rad_s
       << ',' << telemetry.delta_twist_xi_cmd_linear_norm_m_s
       << ',' << telemetry.delta_twist_xi_cmd_angular_norm_rad_s
       << ',' << telemetry.delta_twist_saturated
       << ',' << telemetry.delta_twist_lead_linear_norm_m
       << ',' << telemetry.delta_twist_lead_angular_norm_rad
       << ',' << telemetry.delta_twist_feedback_source
       << ',' << telemetry.delta_twist_pending_clamped
       << ',' << telemetry.delta_twist_residual_cleared_on_frame
       << ',' << telemetry.delta_twist_min_time_to_go_used
       << ',' << telemetry.delta_twist_lin_feedback_cos
       << ',' << telemetry.delta_twist_ang_feedback_cos
       << ',' << telemetry.delta_twist_xi_ref_clamped_norm
       << ',' << telemetry.delta_twist_xi_cmd_clamped_norm
       << ',' << telemetry.delta_twist_frame_rows
       << ',' << telemetry.delta_twist_normal_budget
       << ',' << telemetry.delta_twist_total_budget
       << ',' << telemetry.delta_twist_steps_remaining
       << ',' << telemetry.delta_twist_clamp_mask
       << ',' << telemetry.delta_twist_accel_cmd.x
       << ',' << telemetry.delta_twist_accel_cmd.y
       << ',' << telemetry.delta_twist_accel_cmd.z
       << ',' << telemetry.delta_twist_accel_cmd.rx
       << ',' << telemetry.delta_twist_accel_cmd.ry
       << ',' << telemetry.delta_twist_accel_cmd.rz
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
       << ',' << csvEscape(sample.self_collision_pair)
       << ',' << csvEscape(sample.self_collision_closest_pair);
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
          << csvEscape(sample.command_buffer_read.result) << ','
          << sample.command_buffer_read.pending_lifecycle_count << ','
          << sample.command_buffer_read.skipped_lifecycle_count << ','
          << sample.command_buffer_read.returned_seq << ','
          << toString(sample.command_buffer_read.returned_left_mode) << ','
          << toString(sample.command_buffer_read.returned_right_mode) << ','
          << sample.command_buffer_read.returned_host_time_ns << ','
          << sample.command_buffer_read.returned_age_ms << ','
          << sample.command_buffer_read.returned_client_send_age_ms << ','
          << sample.command_buffer_read.latest_seq << ','
          << toString(sample.command_buffer_read.latest_left_mode) << ','
          << toString(sample.command_buffer_read.latest_right_mode) << ','
          << sample.command_buffer_read.latest_host_time_ns << ','
          << sample.command_buffer_read.latest_age_ms << ','
          << sample.command_buffer_read.latest_timeout_ms << ','
          << sample.command_buffer_read.latest_timeout_valid << ','
          << sample.command_buffer_read.latest_usable << ','
          << sample.command_buffer_read.latest_client_send_age_ms << ','
          << sample.command_buffer_read.lifecycle_seq << ','
          << toString(sample.command_buffer_read.lifecycle_left_mode) << ','
          << toString(sample.command_buffer_read.lifecycle_right_mode) << ','
          << sample.command_buffer_read.lifecycle_host_time_ns << ','
          << sample.command_buffer_read.lifecycle_age_ms << ','
          << sample.command_buffer_read.lifecycle_timeout_ms << ','
          << sample.command_buffer_read.lifecycle_timeout_valid << ','
          << sample.command_buffer_read.lifecycle_usable << ','
          << sample.command_buffer_read.external_boxes_pending << ','
          << sample.command_buffer_read.external_boxes_consumed << ','
          << sample.command_buffer_read.external_boxes_applied << ','
          << sample.command_buffer_read.external_boxes_seq << ','
          << toString(sample.command_buffer_read.external_boxes_left_mode) << ','
          << toString(sample.command_buffer_read.external_boxes_right_mode) << ','
          << sample.command_buffer_read.external_boxes_host_time_ns << ','
          << sample.command_buffer_read.external_boxes_age_ms << ','
          << sample.command_buffer_read.external_boxes_client_send_age_ms << ','
          << sample.chunk_frame.wire_seq << ','
          << sample.chunk_frame.recv_seq << ','
          << sample.chunk_frame.policy_dt_sec << ','
          << sample.chunk_frame.horizon << ','
          << sample.chunk_frame.execute_steps << ','
          << sample.chunk_frame.runway_steps << ','
          << sample.chunk_frame.age_ms << ','
          << sample.chunk_frame.interarrival_ms << ','
          << sample.chunk_frame.inference_seq << ','
          << sample.chunk_frame.inference_queue_wait_ms << ','
          << sample.chunk_frame.inference_latency_ms << ','
          << sample.chunk_frame.inference_ready_wait_ms << ','
          << sample.chunk_frame.inference_period_ms << ','
          << sample.chunk_frame.inference_period_jitter_ms << ','
          << sample.chunk_frame.inference_stall_count << ','
          << sample.chunk_frame.camera_bundle_seq << ','
          << sample.chunk_frame.camera_bundle_age_ms << ','
          << sample.chunk_frame.camera_max_skew_ms << ','
          << sample.chunk_frame.camera_left_frame_number << ','
          << sample.chunk_frame.camera_right_frame_number << ','
          << sample.chunk_frame.camera_left_frame_age_ms << ','
          << sample.chunk_frame.camera_right_frame_age_ms << ','
          << sample.chunk_frame.camera_left_focus_score << ','
          << sample.chunk_frame.camera_right_focus_score << ','
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
          << sample.right_last_read.duration_us << ','
          << sample.left_last_read.read_exchange_timing.available << ','
          << sample.left_last_read.read_exchange_timing.exchange_sequence << ','
          << csvEscape(sample.left_last_read.read_exchange_timing.source) << ','
          << sample.left_last_read.read_exchange_timing.request_data_call_start_steady_ns << ','
          << sample.left_last_read.read_exchange_timing.request_data_call_start_system_ns << ','
          << sample.left_last_read.read_exchange_timing.request_data_call_return_steady_ns << ','
          << sample.left_last_read.read_exchange_timing.request_data_call_return_system_ns << ','
          << sample.left_last_read.read_exchange_timing.request_data_call_duration_us << ','
          << sample.right_last_read.read_exchange_timing.available << ','
          << sample.right_last_read.read_exchange_timing.exchange_sequence << ','
          << csvEscape(sample.right_last_read.read_exchange_timing.source) << ','
          << sample.right_last_read.read_exchange_timing.request_data_call_start_steady_ns << ','
          << sample.right_last_read.read_exchange_timing.request_data_call_start_system_ns << ','
          << sample.right_last_read.read_exchange_timing.request_data_call_return_steady_ns << ','
          << sample.right_last_read.read_exchange_timing.request_data_call_return_system_ns << ','
          << sample.right_last_read.read_exchange_timing.request_data_call_duration_us;
    for (double v : sample.left_state.q_actual_deg) file_ << ',' << v;
    for (double v : sample.right_state.q_actual_deg) file_ << ',' << v;
    for (double v : sample.left_sent_q_deg) file_ << ',' << v;
    for (double v : sample.right_sent_q_deg) file_ << ',' << v;
    for (double v : sample.left_state.q_target_deg) file_ << ',' << v;
    for (double v : sample.right_state.q_target_deg) file_ << ',' << v;
    file_ << ',' << sample.left_state.q_ref_valid
          << ',' << sample.right_state.q_ref_valid
          << ',' << sample.left_state.q_actual_valid
          << ',' << sample.right_state.q_actual_valid;
    file_ << ',' << csvEscape(sample.left_state.rbpodo_sdk_state_source)
          << ',' << csvEscape(sample.right_state.rbpodo_sdk_state_source);
    const auto init_stage = [](const RobotState& st) {
        return st.rbpodo_diagnostics.has_value()
            ? st.rbpodo_diagnostics->raw.init_state_info
            : -1;
    };
    file_ << ',' << init_stage(sample.left_state)
          << ',' << init_stage(sample.right_state)
          << ',' << sample.left_state.servo_enabled
          << ',' << sample.right_state.servo_enabled;
    for (const RbpodoQueueAckTelemetry* q : {&sample.left_queue_ack, &sample.right_queue_ack}) {
        file_ << ',' << q->observed
              << ',' << q->fill
              << ',' << q->fill_min
              << ',' << q->fill_max
              << ',' << q->sequence
              << ',' << q->parsed_total
              << ',' << q->drained_total
              << ',' << q->malformed_total
              << ',' << q->drained_this_send
              << ',' << q->parsed_this_send;
    }
    for (const QueueSyncTelemetry* qs : {&sample.left_queue_sync, &sample.right_queue_sync}) {
        file_ << ',' << qs->enabled
              << ',' << qs->period_trim_us
              << ',' << qs->fill_lpf
              << ',' << qs->integral_us
              << ',' << qs->last_fill
              << ',' << qs->fill_valid
              << ',' << csvEscape(qs->phase)
              << ',' << qs->locked
              << ',' << qs->stale_cycles
              << ',' << qs->underrun_events
              << ',' << qs->warn_events
              << ',' << qs->dip_events
              << ',' << qs->dip_last_min
              << ',' << qs->dip_last_ms
              << ',' << qs->stall_events
              << ',' << qs->highwater_events
              << ',' << qs->redrain_events
              << ',' << qs->no_consumption_events;
    }
    for (const ArmWorkerTelemetry* wt :
         {&sample.left_worker_telemetry, &sample.right_worker_telemetry}) {
        file_ << ',' << wt->worker_pending_overwrites_total
              << ',' << wt->worker_repeated_sends_total
              << ',' << wt->worker_wire_dispatches_total
              << ',' << wt->worker_last_wire_send_start_ns
              << ',' << wt->worker_last_wire_send_end_ns
              << ',' << wt->worker_interp_active
              << ',' << wt->worker_interp_delay_setpoints
              << ',' << wt->worker_interp_rebase_total
              << ',' << wt->worker_interp_hold_total;
    }
    file_ << ',' << sample.left_state.host_time_ns
          << ',' << sample.right_state.host_time_ns;
    file_ << ',' << sample.safety_projection.active
          << ',' << sample.safety_projection.constraint_count
          << ',' << sample.safety_projection.left_correction_deg_s
          << ',' << sample.safety_projection.right_correction_deg_s
          << ',' << sample.safety_projection.left_applied_correction_deg_s
          << ',' << sample.safety_projection.right_applied_correction_deg_s
          << ',' << sample.safety_projection.ceiling_clamped
          << ',' << sample.safety_projection.min_margin_m
          << ',' << sample.safety_projection.selfcol_verdict_age_ms
          << ',' << sample.safety_projection.min_headroom_m
          << ',' << sample.safety_projection.min_headroom_d_hard_m
          << ',' << csvEscape(sample.safety_projection.min_headroom_class)
          << ',' << csvEscape(sample.safety_projection.min_headroom_pair)
          << ',' << sample.safety_projection.left_plan_gate
          << ',' << sample.safety_projection.right_plan_gate;
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
    writeForceColumns(file_, sample.left_ft, sample.left_force_control, sample.left_safety_tracking);
    writeForceColumns(file_, sample.right_ft, sample.right_force_control, sample.right_safety_tracking);
    file_ << '\n';
}

}  // namespace rb_servo
