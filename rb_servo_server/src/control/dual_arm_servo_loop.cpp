#include "rb_servo/control/dual_arm_servo_loop.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <pinocchio/spatial/motion.hpp>

#include "rb_servo/control/cartesian_controller.hpp"
#include "rb_servo/control/fault_classifier.hpp"
#include "rb_servo/control/servo_dispatcher.hpp"
#include "rb_servo/core/clock.hpp"
#include "rb_servo/core/realtime.hpp"
#include "rb_servo/kinematics/pinocchio_kinematics.hpp"
#include "rb_servo/math/se3.hpp"
#include "rb_servo/network/scope_publisher.hpp"

namespace rb_servo {
namespace {

// How many consecutive ticks a tare averages before committing. At 500 Hz this is
// 0.5 s of samples — long enough to average the sensor's noise, short enough that an
// operator holding the arm still cannot drift far during it.
constexpr std::uint32_t kFtTareSamples = 250;



// Spin-hint for the hybrid sleep-then-spin tail: keeps the core in C0 and yields
// the pipeline without descheduling. No-op on unknown ISAs (spin still works,
// just without the relax hint).
inline void cpuRelax() {
#if defined(__x86_64__) || defined(__i386__)
    __builtin_ia32_pause();
#elif defined(__aarch64__)
    asm volatile("yield" ::: "memory");
#endif
}

// kernel.sched_rt_runtime_us: -1 means RT throttling is OFF (safe for a busy
// spin). Any other value (or an unreadable file) is treated as "throttling on"
// so the spin guard fails closed and falls back to plain sleep_until.
long long readSchedRtRuntimeUs() {
    std::ifstream f("/proc/sys/kernel/sched_rt_runtime_us");
    long long value = 0;
    if (f >> value) return value;
    return 0;  // unreadable -> conservative: not -1, so spin stays disabled
}

bool isCartesianMode(ControlMode mode) {
    return mode == ControlMode::TcpPoseTarget ||
           mode == ControlMode::TcpLinearMove;
}

bool isStreamingCartesianMode(ControlMode mode) {
    return mode == ControlMode::TcpLinearMove;
}

bool isMotionMode(ControlMode mode) {
    return mode == ControlMode::JointTarget ||
           mode == ControlMode::TcpPoseTarget ||
           mode == ControlMode::TcpLinearMove;
}

double vec6LinearNorm(const Vec6& value) {
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

double vec6AngularNorm(const Vec6& value) {
    return std::sqrt(value.rx * value.rx + value.ry * value.ry + value.rz * value.rz);
}

int deltaTwistStepKind(control::DeltaTwistStepPhase phase) {
    switch (phase) {
        case control::DeltaTwistStepPhase::Normal:
            return 1;
        case control::DeltaTwistStepPhase::Reserve:
            return 2;
        case control::DeltaTwistStepPhase::ResidualDrain:
            return 3;
        case control::DeltaTwistStepPhase::Ringdown:
            return 4;
        case control::DeltaTwistStepPhase::Inactive:
        default:
            return 0;
    }
}

bool finiteJointArray(const JointArray& joints) {
    return std::all_of(joints.begin(), joints.end(), [](double value) {
        return std::isfinite(value);
    });
}


bool isSyntheticHoldCommand(const DualArmCommand& command) {
    return command.seq == 0 &&
           command.left.mode == ControlMode::Hold &&
           command.right.mode == ControlMode::Hold;
}

std::string initMotionFailModeString(InitMotionPlanResult::FailMode mode) {
    switch (mode) {
        case InitMotionPlanResult::FailMode::None:
            return "none";
        case InitMotionPlanResult::FailMode::GoalNotClear:
            return "goal_not_clear";
        case InitMotionPlanResult::FailMode::EscapeFailed:
            return "escape_failed";
        case InitMotionPlanResult::FailMode::RrtBudget:
            return "rrt_budget";
        case InitMotionPlanResult::FailMode::NonFinite:
            return "nonfinite";
    }
    return "unknown";
}

bool isExplicitDualHoldCommand(const DualArmCommand& command) {
    return command.seq != 0 &&
           command.left.mode == ControlMode::Hold &&
           command.right.mode == ControlMode::Hold;
}

bool linearPathLeaseExpired(const CartesianServoPathState& path, uint64_t now_ns) {
    return path.active &&
           path.lease_enforced &&
           path.lease_expires_time_ns > 0 &&
           now_ns > path.lease_expires_time_ns;
}

bool retainCompletedPathTelemetry(
    const CartesianServoPathState& path,
    const CartesianSolveTelemetry& telemetry
) {
    return path.active &&
           path.done &&
           telemetry.status == "ok" &&
           telemetry.path_done;
}

bool isReadOnlyBlockedMode(ControlMode mode) {
    return mode == ControlMode::ArmMotion || isMotionMode(mode);
}

bool isCommandModeMissingPayload(const ArmCommand& command) {
    switch (command.mode) {
        case ControlMode::JointTarget:
            return !command.has_joint_target;
        case ControlMode::TcpPoseTarget:
            return !command.has_tcp_target;
        case ControlMode::TcpLinearMove:
            return !command.has_tcp_target ||
                   (!command.has_linear_move_duration && !command.has_linear_move_linear_speed);
        default:
            return false;
    }
}

std::shared_ptr<IKinematics> makeKinematicsProvider(const DualArmConfig& config) {
    if (!config.kinematics.enable || (!config.kinematics.publish_tcp && !config.kinematics.ik.enable)) {
        return nullptr;
    }
    if (config.kinematics.provider != "pinocchio") {
        return nullptr;
    }
    try {
        return std::make_shared<PinocchioKinematics>(config.kinematics);
    } catch (const std::exception& exc) {
        std::cerr << "[WARN] FK TCP publish deferred: failed to initialize kinematics: "
                  << exc.what() << "\n";
        return nullptr;
    }
}


std::string jointArrayDebugString(const JointArray& joints) {
    std::ostringstream out;
    out << "[";
    for (int i = 0; i < kDof; ++i) {
        if (i > 0) out << ",";
        out << joints[i];
    }
    out << "]";
    return out.str();
}

bool containsReason(
    const ArmStartupValidationSnapshot& validation,
    const std::string& reason
) {
    return std::find(
        validation.invalid_reasons.begin(),
        validation.invalid_reasons.end(),
        reason
    ) != validation.invalid_reasons.end();
}

void appendReason(ArmStartupValidationSnapshot* validation, const std::string& reason) {
    if (!validation || containsReason(*validation, reason)) return;
    validation->invalid_reasons.push_back(reason);
}

// Re-anchor the pose-track SMD when the live reference (FK of the last sent joints) has
// drifted this far from the tracker's held pose — i.e., another control path (JointTarget
// init-return, fault hold, command-buffer gap) moved the robot while the tracker stayed
// active with stale state. Generous vs. normal per-tick tracking/IK residual (sub-cm) yet far
// below an inter-episode swing (tens of cm), so it isolates external moves without false trips.
constexpr double kPoseTrackReanchorPosTolM = 0.05;
constexpr double kPoseTrackReanchorAngTolRad = 0.10;
// Strict chunk-follower divergence is explained only by a safety intervention
// stamped by applySafety on a recent prior tick. The follower stage runs before
// this tick's safety evaluation, so the window must cover one or more servo ticks
// without letting stale, unrelated interventions mask real divergence.
constexpr double kFollowerDivergenceExplainWindowSec = 0.1;
constexpr uint64_t kFollowerDivergenceExplainWindowNs =
    static_cast<uint64_t>(kFollowerDivergenceExplainWindowSec * 1'000'000'000.0);
constexpr uint64_t kFollowerDivergenceReanchorLogPeriodNs = 1'000'000'000ULL;
constexpr double kSafetyInterventionCorrectionEpsDegPerSec = 1e-3;

// Streaming TcpPoseTarget smoothing: integrate received target deltas into the
// SMD tracker's goal and replace the commanded pose with this tick's SMD
// solution. Any other mode deactivates the tracker so re-entry re-anchors at
// the currently sent pose (FK of the previous sent joints) with zero velocity.
ArmCommand applyPoseTrackSmd(
    const ArmCommand& command,
    const PoseTrackSmdConfig& config,
    SmdPoseTracker* tracker,
    const std::shared_ptr<IKinematics>& kinematics,
    const ArmMountConfig& mount,
    const JointArray& previous_sent_q_deg,
    double dt_sec
) {
    if (!tracker) return command;
    if (!config.enable || command.mode != ControlMode::TcpPoseTarget || !command.has_tcp_target) {
        tracker->deactivate();
        return command;
    }
    if (!kinematics) {
        // Without FK there is no safe anchor pose; pass the raw target through.
        tracker->deactivate();
        return command;
    }
    if (!tracker->active()) {
        tracker->reset(kinematics->computeTcpStand(command.arm_id, previous_sent_q_deg, mount));
    } else {
        // Re-anchor if the live reference (FK of the last sent joints) has drifted far from
        // the tracker's held pose. The command buffer holds the previous episode's last
        // TcpPoseTarget across the inter-episode gap, so the tracker can stay active while a
        // JointTarget init-return (or fault hold) moves the robot elsewhere; the first stream
        // tick would then snap the output from the stale pose toward the live reference — an
        // out-and-back jerk at each episode transition. During normal streaming the SMD output
        // IS the sent command, so currentPose ~= FK(prev_sent_q) within IK/safety residual and
        // the generous tolerances below never trip. Re-anchoring only ever starts the tracker
        // from where the robot actually is; it cannot inject motion.
        const Pose6D reference =
            kinematics->computeTcpStand(command.arm_id, previous_sent_q_deg, mount);
        if (tracker->driftedFrom(reference, kPoseTrackReanchorPosTolM, kPoseTrackReanchorAngTolRad)) {
            tracker->reset(reference);
        }
    }
    tracker->updateGoalFromCommand(command.tcp_target_stand);
    ArmCommand smoothed = command;
    smoothed.tcp_target_stand = tracker->step(dt_sec);
    return smoothed;
}

// ---- Ruckig chunk-follower stage helpers -----------------------------------

bool ruckigFollowerConfigChanged(const RuckigFollowerConfig& a, const RuckigFollowerConfig& b) {
    return a.enable != b.enable ||
        a.controller != b.controller ||
        a.fallback_policy != b.fallback_policy ||
        a.engage_timeout_sec != b.engage_timeout_sec ||
        a.max_linear_velocity_m_s != b.max_linear_velocity_m_s ||
        a.max_linear_accel_m_s2 != b.max_linear_accel_m_s2 ||
        a.max_linear_jerk_m_s3 != b.max_linear_jerk_m_s3 ||
        a.max_angular_velocity_rad_s != b.max_angular_velocity_rad_s ||
        a.max_angular_accel_rad_s2 != b.max_angular_accel_rad_s2 ||
        a.max_angular_jerk_rad_s3 != b.max_angular_jerk_rad_s3 ||
        a.discard_head_steps != b.discard_head_steps ||
        a.consume_steps != b.consume_steps ||
        a.reserve_steps != b.reserve_steps ||
        a.smoothing_window != b.smoothing_window ||
        a.output_smd.enable != b.output_smd.enable ||
        a.output_smd.nf_linear_hz != b.output_smd.nf_linear_hz ||
        a.output_smd.nf_angular_hz != b.output_smd.nf_angular_hz ||
        a.output_smd.damping_ratio != b.output_smd.damping_ratio ||
        a.output_smd.velocity_ff != b.output_smd.velocity_ff ||
        a.output_smd.velocity_ff_lpf_hz != b.output_smd.velocity_ff_lpf_hz ||
        a.af_damping_beta_lin != b.af_damping_beta_lin ||
        a.af_damping_beta_ang != b.af_damping_beta_ang ||
        a.corner_deadband_lin_m != b.corner_deadband_lin_m ||
        a.corner_deadband_ang_rad != b.corner_deadband_ang_rad ||
        a.corner_velocity_scale != b.corner_velocity_scale ||
        a.delta_twist_tau_sec != b.delta_twist_tau_sec ||
        a.delta_twist_residual_drain_steps != b.delta_twist_residual_drain_steps ||
        a.delta_twist_clear_residual_on_new_frame != b.delta_twist_clear_residual_on_new_frame ||
        a.delta_twist_min_time_to_go_sec != b.delta_twist_min_time_to_go_sec ||
        a.delta_twist_max_residual_m != b.delta_twist_max_residual_m ||
        a.delta_twist_max_residual_rad != b.delta_twist_max_residual_rad ||
        a.delta_twist_max_lead_m != b.delta_twist_max_lead_m ||
        a.delta_twist_max_lead_rad != b.delta_twist_max_lead_rad ||
        a.delta_twist_stale_residual_timeout_sec != b.delta_twist_stale_residual_timeout_sec ||
        a.preview_max_projection_error_m != b.preview_max_projection_error_m ||
        a.preview_max_projection_error_rad != b.preview_max_projection_error_rad ||
        a.preview_max_consecutive_projection_errors != b.preview_max_consecutive_projection_errors ||
        a.preview_max_actual_lead_m != b.preview_max_actual_lead_m ||
        a.preview_max_actual_lead_rad != b.preview_max_actual_lead_rad ||
        a.preview_max_consecutive_actual_lead_errors != b.preview_max_consecutive_actual_lead_errors ||
        a.hold_bounce_resume_sec != b.hold_bounce_resume_sec ||
        a.preview_projection_fault_policy != b.preview_projection_fault_policy ||
        a.chunk_feed_timeout_sec != b.chunk_feed_timeout_sec;
}

control::CartesianChunkFollowerConfig makeChunkFollowerConfig(const RuckigFollowerConfig& rf) {
    control::CartesianChunkFollowerConfig cfg;
    cfg.lin = control::AxisLimit{rf.max_linear_velocity_m_s, rf.max_linear_accel_m_s2, rf.max_linear_jerk_m_s3};
    cfg.ang = control::AxisLimit{rf.max_angular_velocity_rad_s, rf.max_angular_accel_rad_s2, rf.max_angular_jerk_rad_s3};
    cfg.window.discard_head_L = rf.discard_head_steps;
    cfg.window.consume_C = rf.consume_steps;
    cfg.window.reserve_R = rf.reserve_steps;
    cfg.window.smoothing_window = rf.smoothing_window;
    cfg.guard.af_damping_beta_lin = rf.af_damping_beta_lin;
    cfg.guard.af_damping_beta_ang = rf.af_damping_beta_ang;
    cfg.guard.corner_deadband_lin_m = rf.corner_deadband_lin_m;
    cfg.guard.corner_deadband_ang_rad = rf.corner_deadband_ang_rad;
    cfg.guard.corner_velocity_scale = rf.corner_velocity_scale;
    cfg.max_projection_error_m = rf.preview_max_projection_error_m;
    cfg.max_projection_error_rad = rf.preview_max_projection_error_rad;
    cfg.max_consecutive_projection_errors = rf.preview_max_consecutive_projection_errors;
    cfg.max_actual_lead_m = rf.preview_max_actual_lead_m;
    cfg.max_actual_lead_rad = rf.preview_max_actual_lead_rad;
    cfg.max_consecutive_actual_lead_errors = rf.preview_max_consecutive_actual_lead_errors;
    return cfg;
}

control::DeltaTwistFollowerConfig makeDeltaTwistFollowerConfig(const RuckigFollowerConfig& rf) {
    control::DeltaTwistFollowerConfig cfg;
    cfg.lin = control::AxisLimit{rf.max_linear_velocity_m_s, rf.max_linear_accel_m_s2, rf.max_linear_jerk_m_s3};
    cfg.ang = control::AxisLimit{rf.max_angular_velocity_rad_s, rf.max_angular_accel_rad_s2, rf.max_angular_jerk_rad_s3};
    cfg.discard_head_steps = rf.discard_head_steps;
    cfg.consume_steps = rf.consume_steps;
    cfg.reserve_steps = rf.reserve_steps;
    cfg.tau_sec = rf.delta_twist_tau_sec;
    cfg.residual_drain_steps = rf.delta_twist_residual_drain_steps;
    cfg.clear_residual_on_new_frame = rf.delta_twist_clear_residual_on_new_frame;
    cfg.min_time_to_go_sec = rf.delta_twist_min_time_to_go_sec;
    cfg.max_residual_m = rf.delta_twist_max_residual_m;
    cfg.max_residual_rad = rf.delta_twist_max_residual_rad;
    cfg.max_lead_m = rf.delta_twist_max_lead_m;
    cfg.max_lead_rad = rf.delta_twist_max_lead_rad;
    cfg.stale_residual_timeout_sec = rf.delta_twist_stale_residual_timeout_sec;
    return cfg;
}

control::ChunkFrame toControlChunkFrame(
    const ChunkFrameReceiver::Frame& frame,
    ArmId arm_id
) {
    const ChunkFrameReceiver::ArmSteps& steps =
        arm_id == ArmId::Left ? frame.left : frame.right;
    const bool has_delta =
        arm_id == ArmId::Left ? frame.has_left_delta : frame.has_right_delta;
    const ChunkFrameReceiver::ArmDeltaSteps& delta_steps =
        arm_id == ArmId::Left ? frame.left_delta : frame.right_delta;
    control::ChunkFrame out;
    out.policy_dt = frame.policy_dt_sec;
    out.wire_seq = frame.seq;
    out.recv_seq = frame.receiver_seq;
    out.recv_time = frame.recv_steady_sec;
    out.pose.reserve(static_cast<std::size_t>(steps.count));
    out.grip.reserve(static_cast<std::size_t>(steps.count));
    if (has_delta) {
        out.delta.reserve(static_cast<std::size_t>(delta_steps.count));
    }
    for (int i = 0; i < steps.count; ++i) {
        const auto& s = steps.step[static_cast<std::size_t>(i)];
        Pose6D p;
        p.x = s[0];
        p.y = s[1];
        p.z = s[2];
        p.quaternion_xyzw = std::array<double, 4>{s[3], s[4], s[5], s[6]};
        out.pose.push_back(p);
        out.grip.push_back(s[7]);
    }
    if (has_delta) {
        for (int i = 0; i < delta_steps.count; ++i) {
            const auto& s = delta_steps.step[static_cast<std::size_t>(i)];
            out.delta.push_back(Vec6{s[0], s[1], s[2], s[3], s[4], s[5]});
        }
    }
    return out;
}

TcpPoseTargetProfileConfig selectTcpPoseTargetProfile(
    const CartesianControlConfig& config,
    const std::string& requested_profile,
    bool* found
) {
    const std::string name = requested_profile.empty()
        ? config.tcp_pose_target_profile_default
        : requested_profile;
    for (const TcpPoseTargetProfileConfig& profile : config.tcp_pose_target_profiles) {
        if (profile.name == name) {
            if (found) *found = true;
            return profile;
        }
    }
    if (found) *found = false;
    TcpPoseTargetProfileConfig fallback;
    fallback.name = config.tcp_pose_target_profile_default.empty()
        ? "default"
        : config.tcp_pose_target_profile_default;
    fallback.pose_track_smd = config.pose_track_smd;
    return fallback;
}

std::string lowerAscii(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

bool isRbpodoControllerSimulationBackend(const BackendConfig& backend) {
    if (backend.backend_type != BackendType::Rbpodo) return false;
    if (backend.run_mode != RunMode::Real) return false;
    const std::string operation_mode = lowerAscii(backend.operation_mode);
    return operation_mode == "simulation" || operation_mode == "sim";
}

bool anyRbpodoControllerSimulationBackend(const DualArmConfig& config) {
    return isRbpodoControllerSimulationBackend(config.left_robot) ||
        isRbpodoControllerSimulationBackend(config.right_robot);
}

bool bothRbpodoControllerSimulationBackends(const DualArmConfig& config) {
    return isRbpodoControllerSimulationBackend(config.left_robot) &&
        isRbpodoControllerSimulationBackend(config.right_robot);
}

bool controllerSimulationMotionRequired(const DualArmConfig& config) {
    return config.servo.send_servo_commands &&
        anyRbpodoControllerSimulationBackend(config);
}

bool controllerSimulationMotionGateOpen(const DualArmConfig& config) {
    if (!controllerSimulationMotionRequired(config)) return true;
    return config.servo.allow_controller_simulation_motion &&
        bothRbpodoControllerSimulationBackends(config);
}

const BackendConfig& backendConfigForArm(const DualArmConfig& config, ArmId arm_id) {
    return arm_id == ArmId::Left ? config.left_robot : config.right_robot;
}

bool controllerSimulationCartesianGateOpen(
    const DualArmConfig& config,
    const BackendConfig& backend
) {
    return config.cartesian_control.enable &&
        config.cartesian_control.allow_in_controller_simulation &&
        config.servo.allow_controller_simulation_motion &&
        isRbpodoControllerSimulationBackend(backend);
}

struct CartesianAvailability {
    bool available = false;
    std::string reason = "cartesian_control_unavailable_run_mode";
    bool controller_simulation_cartesian_enabled = false;
    bool physical_motion_expected = true;
};

CartesianAvailability cartesianAvailabilityForArm(
    const DualArmConfig& config,
    const ArmCommand& command
) {
    // Real/sim execution gating is retired: every Cartesian mode is available
    // whenever cartesian_control is enabled, regardless of run_mode /
    // operation_mode. run_mode remains a telemetry label only; safety is owned
    // by the mode-independent layers (safety filter clamps, tracking-error
    // latch, self-collision guard, lease arbitration, deadman on the client).
    CartesianAvailability availability;
    const BackendConfig& backend = backendConfigForArm(config, command.arm_id);

    if (!config.cartesian_control.enable) {
        availability.reason = "cartesian_control_unavailable_disabled";
        return availability;
    }

    const bool controller_simulation = isRbpodoControllerSimulationBackend(backend);
    availability.available = true;
    availability.reason = "";
    availability.controller_simulation_cartesian_enabled = controller_simulation;
    availability.physical_motion_expected =
        !controller_simulation && backend.run_mode == RunMode::Real;
    return availability;
}

RunMode cartesianComputationRunModeForArm(
    const DualArmConfig& config,
    const ArmCommand& command
) {
    const RunMode run_mode = backendConfigForArm(config, command.arm_id).run_mode;
    if (run_mode == RunMode::Real &&
        isCartesianMode(command.mode) &&
        controllerSimulationCartesianGateOpen(config, backendConfigForArm(config, command.arm_id))) {
        return RunMode::Simulation;
    }
    return run_mode;
}

std::string controllerSimulationStateSourceName(CartesianControllerSimulationStateSource source) {
    switch (source) {
        case CartesianControllerSimulationStateSource::Actual:
            return "actual";
        case CartesianControllerSimulationStateSource::Reference:
            return "reference";
    }
    return "unknown";
}

std::string controllerSimulationTrackingSourceName(ControllerSimulationTrackingErrorSource source) {
    switch (source) {
        case ControllerSimulationTrackingErrorSource::Actual:
            return "actual";
        case ControllerSimulationTrackingErrorSource::Reference:
            return "reference";
    }
    return "unknown";
}

bool controllerSimulationReferenceSourceActive(
    const DualArmConfig& config,
    const ArmCommand& command,
    CartesianControllerSimulationStateSource source
) {
    if (source != CartesianControllerSimulationStateSource::Reference) return false;
    const BackendConfig& backend = backendConfigForArm(config, command.arm_id);
    return backend.run_mode == RunMode::Real &&
        isCartesianMode(command.mode) &&
        controllerSimulationCartesianGateOpen(config, backend);
}

bool controllerSimulationTrackingReferenceActive(
    const DualArmConfig& config,
    ArmId arm_id
) {
    if (config.safety.controller_simulation_tracking_error_source !=
        ControllerSimulationTrackingErrorSource::Reference) {
        return false;
    }
    const BackendConfig& backend = backendConfigForArm(config, arm_id);
    return isRbpodoControllerSimulationBackend(backend) &&
        controllerSimulationMotionGateOpen(config);
}

double maxAbsJointDelta(const JointArray& a, const JointArray& b) {
    double max_delta = 0.0;
    for (int i = 0; i < kDof; ++i) {
        if (!std::isfinite(a[i]) || !std::isfinite(b[i])) {
            return std::numeric_limits<double>::infinity();
        }
        max_delta = std::max(max_delta, std::abs(a[i] - b[i]));
    }
    return max_delta;
}

bool finiteTcpPosition(const Pose6D& pose) {
    return std::isfinite(pose.x) &&
        std::isfinite(pose.y) &&
        std::isfinite(pose.z);
}

double tcpPositionDistanceM(const Pose6D& a, const Pose6D& b) {
    if (!finiteTcpPosition(a) || !finiteTcpPosition(b)) {
        return std::numeric_limits<double>::infinity();
    }
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool stateSequenceChanged(uint64_t previous, uint64_t current) {
    return current > 0 && current != previous;
}

uint64_t referenceStateSequenceNs(const RobotState& state) {
    return state.robot_time_ns;
}

RbpodoAsyncStreamingSupervisionState moreSevere(
    RbpodoAsyncStreamingSupervisionState a,
    RbpodoAsyncStreamingSupervisionState b
) {
    const auto rank = [](RbpodoAsyncStreamingSupervisionState value) {
        switch (value) {
            case RbpodoAsyncStreamingSupervisionState::Ok:
                return 0;
            case RbpodoAsyncStreamingSupervisionState::Warning:
                return 1;
            case RbpodoAsyncStreamingSupervisionState::Fault:
                return 2;
        }
        return 0;
    };
    return rank(b) > rank(a) ? b : a;
}

struct ReferenceSupervisionRuntime {
    bool have_q_ref_sample = false;
    JointArray last_q_ref_deg{};
    uint64_t last_q_ref_state_sequence_ns = 0;
    uint64_t last_q_ref_update_host_time_ns = 0;

    bool have_tcp_ref_sample = false;
    Pose6D last_tcp_ref_stand;
    uint64_t last_tcp_ref_update_host_time_ns = 0;

    bool has_latest_sent_target = false;
    JointArray latest_sent_q_target_deg{};
    std::optional<Pose6D> latest_sent_tcp_ref_stand;
    uint64_t q_ref_invalid_start_ns = 0;
    uint64_t q_ref_target_error_start_ns = 0;
    uint64_t tcp_ref_target_error_start_ns = 0;
    uint64_t last_q_ref_watchdog_miss_ns = 0;
    uint64_t last_tcp_ref_watchdog_miss_ns = 0;
    uint64_t q_ref_watchdog_miss_count = 0;
    uint64_t tcp_ref_watchdog_miss_count = 0;

    uint64_t fault_count = 0;
    RbpodoAsyncStreamingSupervisionState last_state =
        RbpodoAsyncStreamingSupervisionState::Ok;
    std::string last_reason;
};

struct ReferenceSupervisionState {
    ReferenceSupervisionRuntime left;
    ReferenceSupervisionRuntime right;
};

std::mutex g_reference_supervision_mutex;
std::unordered_map<const DualArmServoLoop*, ReferenceSupervisionState> g_reference_supervision_states;

bool referenceSupervisionActive(
    const DualArmConfig& config,
    ArmId arm_id
) {
    const auto& async = config.servo.rbpodo_async_streaming;
    if (!async.enable ||
        async.mode == RbpodoAsyncStreamingMode::Disabled ||
        !async.reference_supervision.enable) {
        return false;
    }
    const BackendConfig& backend = backendConfigForArm(config, arm_id);
    return isRbpodoControllerSimulationBackend(backend) &&
        controllerSimulationMotionGateOpen(config);
}

ReferenceSupervisionRuntime& runtimeForArm(
    ReferenceSupervisionState& state,
    ArmId arm_id
) {
    return arm_id == ArmId::Left ? state.left : state.right;
}

void eraseReferenceSupervisionState(const DualArmServoLoop* loop) {
    std::lock_guard<std::mutex> lock(g_reference_supervision_mutex);
    g_reference_supervision_states.erase(loop);
}

void resetReferenceSupervisionState(const DualArmServoLoop* loop) {
    std::lock_guard<std::mutex> lock(g_reference_supervision_mutex);
    g_reference_supervision_states[loop] = ReferenceSupervisionState{};
}

void noteReferenceSupervisionSentTarget(
    const DualArmServoLoop* loop,
    const DualArmConfig& config,
    const std::shared_ptr<IKinematics>& kinematics,
    bool kinematics_injected,
    ArmId arm_id,
    const JointArray& q_target_deg
) {
    if (!referenceSupervisionActive(config, arm_id)) {
        return;
    }

    std::lock_guard<std::mutex> lock(g_reference_supervision_mutex);
    ReferenceSupervisionRuntime& runtime =
        runtimeForArm(g_reference_supervision_states[loop], arm_id);
    runtime.has_latest_sent_target = true;
    runtime.latest_sent_q_target_deg = q_target_deg;
    runtime.latest_sent_tcp_ref_stand.reset();

    const bool publish_tcp = config.kinematics.publish_tcp || kinematics_injected;
    if (!kinematics || !publish_tcp || !finiteJointArray(q_target_deg)) {
        return;
    }

    try {
        const ArmMountConfig& mount = arm_id == ArmId::Left
            ? config.left_mount
            : config.right_mount;
        runtime.latest_sent_tcp_ref_stand =
            kinematics->computeTcpStand(arm_id, q_target_deg, mount);
    } catch (const std::exception&) {
        runtime.latest_sent_tcp_ref_stand.reset();
    }
}

void updateReferenceSupervision(
    const DualArmServoLoop* loop,
    const DualArmConfig& config,
    ArmId arm_id,
    const RobotState& state,
    uint64_t observed_ns,
    RbpodoAsyncStreamingTelemetry* telemetry
) {
    if (!telemetry || !referenceSupervisionActive(config, arm_id)) {
        return;
    }

    std::lock_guard<std::mutex> lock(g_reference_supervision_mutex);
    ReferenceSupervisionRuntime& runtime =
        runtimeForArm(g_reference_supervision_states[loop], arm_id);
    const auto& ref_cfg = config.servo.rbpodo_async_streaming.reference_supervision;
    const RbpodoAsyncStreamingMode mode = config.servo.rbpodo_async_streaming.mode;
    const bool socket_send_supervised = mode == RbpodoAsyncStreamingMode::SocketSendSupervised;
    const bool fault_policy =
        ref_cfg.policy == RbpodoAsyncReferenceSupervisionPolicy::FaultLatch;
    const double q_ref_epsilon_deg = 1e-6;
    const double tcp_ref_epsilon_m = 1e-6;

    RbpodoAsyncStreamingSupervisionState reference_state =
        RbpodoAsyncStreamingSupervisionState::Ok;
    std::string reference_reason;
    const auto mark = [&](RbpodoAsyncStreamingSupervisionState candidate, const std::string& reason) {
        const RbpodoAsyncStreamingSupervisionState merged = moreSevere(reference_state, candidate);
        if (merged != reference_state || reference_reason.empty()) {
            reference_state = merged;
            reference_reason = reason;
        }
    };
    const auto markPolicyViolation = [&](const std::string& reason) {
        mark(
            socket_send_supervised && fault_policy
                ? RbpodoAsyncStreamingSupervisionState::Fault
                : RbpodoAsyncStreamingSupervisionState::Warning,
            reason
        );
    };
    const auto markUpdateTimeout = [&](const std::string& reason) {
        mark(
            socket_send_supervised
                ? RbpodoAsyncStreamingSupervisionState::Fault
                : RbpodoAsyncStreamingSupervisionState::Warning,
            reason
        );
    };

    const bool q_ref_valid =
        state.q_ref_valid &&
        state.has_valid_joint_state &&
        finiteJointArray(state.q_target_deg);
    const uint64_t q_ref_sequence_ns = referenceStateSequenceNs(state);
    bool q_ref_updated = false;

    if (q_ref_valid) {
        q_ref_updated =
            !runtime.have_q_ref_sample ||
            maxAbsJointDelta(state.q_target_deg, runtime.last_q_ref_deg) > q_ref_epsilon_deg ||
            stateSequenceChanged(runtime.last_q_ref_state_sequence_ns, q_ref_sequence_ns);
        if (q_ref_updated) {
            runtime.have_q_ref_sample = true;
            runtime.last_q_ref_deg = state.q_target_deg;
            runtime.last_q_ref_state_sequence_ns = q_ref_sequence_ns;
            runtime.last_q_ref_update_host_time_ns = observed_ns;
        }
        runtime.q_ref_invalid_start_ns = 0;
    } else if (runtime.has_latest_sent_target) {
        if (runtime.q_ref_invalid_start_ns == 0) {
            runtime.q_ref_invalid_start_ns = observed_ns;
        }
        mark(RbpodoAsyncStreamingSupervisionState::Warning, "async_reference_q_ref_invalid");
        const double invalid_age_ms = observed_ns >= runtime.q_ref_invalid_start_ns
            ? static_cast<double>(observed_ns - runtime.q_ref_invalid_start_ns) /
                1'000'000.0
            : 0.0;
        if (invalid_age_ms >= ref_cfg.q_ref_target_fault_after_ms) {
            markPolicyViolation("async_reference_q_ref_invalid");
        }
    }

    const bool tcp_ref_valid =
        state.tcp_ref_valid &&
        state.tcp_ref_stand.has_value() &&
        finiteTcpPosition(*state.tcp_ref_stand);
    bool tcp_ref_updated = false;
    if (tcp_ref_valid) {
        tcp_ref_updated =
            !runtime.have_tcp_ref_sample ||
            tcpPositionDistanceM(*state.tcp_ref_stand, runtime.last_tcp_ref_stand) >
                tcp_ref_epsilon_m ||
            q_ref_updated;
        if (tcp_ref_updated) {
            runtime.have_tcp_ref_sample = true;
            runtime.last_tcp_ref_stand = *state.tcp_ref_stand;
            runtime.last_tcp_ref_update_host_time_ns = observed_ns;
        }
    }

    double q_ref_update_age_ms = 0.0;
    if (runtime.last_q_ref_update_host_time_ns > 0 &&
        observed_ns >= runtime.last_q_ref_update_host_time_ns) {
        q_ref_update_age_ms =
            static_cast<double>(observed_ns - runtime.last_q_ref_update_host_time_ns) /
            1'000'000.0;
        if (q_ref_update_age_ms >= ref_cfg.q_ref_update_timeout_ms) {
            if (runtime.last_q_ref_watchdog_miss_ns != observed_ns) {
                runtime.last_q_ref_watchdog_miss_ns = observed_ns;
                ++runtime.q_ref_watchdog_miss_count;
            }
            markUpdateTimeout("async_q_ref_update_timeout");
        }
    }

    double tcp_ref_update_age_ms = 0.0;
    if (runtime.have_tcp_ref_sample &&
        runtime.last_tcp_ref_update_host_time_ns > 0 &&
        observed_ns >= runtime.last_tcp_ref_update_host_time_ns) {
        tcp_ref_update_age_ms =
            static_cast<double>(observed_ns - runtime.last_tcp_ref_update_host_time_ns) /
            1'000'000.0;
        if (tcp_ref_update_age_ms >= ref_cfg.tcp_ref_update_timeout_ms) {
            if (runtime.last_tcp_ref_watchdog_miss_ns != observed_ns) {
                runtime.last_tcp_ref_watchdog_miss_ns = observed_ns;
                ++runtime.tcp_ref_watchdog_miss_count;
            }
            markUpdateTimeout("async_tcp_ref_update_timeout");
        }
    }

    double q_ref_target_error_deg_max = 0.0;
    if (q_ref_valid && runtime.has_latest_sent_target) {
        q_ref_target_error_deg_max =
            maxAbsJointDelta(state.q_target_deg, runtime.latest_sent_q_target_deg);
        if (q_ref_target_error_deg_max > ref_cfg.q_ref_target_tolerance_deg) {
            if (runtime.q_ref_target_error_start_ns == 0) {
                runtime.q_ref_target_error_start_ns = observed_ns;
            }
            mark(RbpodoAsyncStreamingSupervisionState::Warning, "async_q_ref_target_error");
            const double error_age_ms = observed_ns >= runtime.q_ref_target_error_start_ns
                ? static_cast<double>(observed_ns - runtime.q_ref_target_error_start_ns) /
                    1'000'000.0
                : 0.0;
            if (error_age_ms >= ref_cfg.q_ref_target_fault_after_ms) {
                markPolicyViolation("async_q_ref_target_error");
            }
        } else {
            runtime.q_ref_target_error_start_ns = 0;
        }
    } else {
        runtime.q_ref_target_error_start_ns = 0;
    }

    double tcp_ref_target_error_m = 0.0;
    if (tcp_ref_valid && runtime.latest_sent_tcp_ref_stand.has_value()) {
        tcp_ref_target_error_m =
            tcpPositionDistanceM(*state.tcp_ref_stand, *runtime.latest_sent_tcp_ref_stand);
        if (tcp_ref_target_error_m > ref_cfg.tcp_ref_target_tolerance_m) {
            if (runtime.tcp_ref_target_error_start_ns == 0) {
                runtime.tcp_ref_target_error_start_ns = observed_ns;
            }
            mark(RbpodoAsyncStreamingSupervisionState::Warning, "async_tcp_ref_target_error");
            const double error_age_ms = observed_ns >= runtime.tcp_ref_target_error_start_ns
                ? static_cast<double>(observed_ns - runtime.tcp_ref_target_error_start_ns) /
                    1'000'000.0
                : 0.0;
            if (error_age_ms >= ref_cfg.tcp_ref_target_fault_after_ms) {
                markPolicyViolation("async_tcp_ref_target_error");
            }
        } else {
            runtime.tcp_ref_target_error_start_ns = 0;
        }
    } else {
        runtime.tcp_ref_target_error_start_ns = 0;
    }

    if (reference_state == RbpodoAsyncStreamingSupervisionState::Fault &&
        runtime.last_state != RbpodoAsyncStreamingSupervisionState::Fault) {
        ++runtime.fault_count;
    }
    runtime.last_state = reference_state;
    runtime.last_reason = reference_reason;

    telemetry->last_q_ref_update_host_time_ns = runtime.last_q_ref_update_host_time_ns;
    telemetry->last_tcp_ref_update_host_time_ns = runtime.last_tcp_ref_update_host_time_ns;
    telemetry->q_ref_update_age_ms = q_ref_update_age_ms;
    telemetry->tcp_ref_update_age_ms = tcp_ref_update_age_ms;
    telemetry->q_ref_target_error_deg_max = q_ref_target_error_deg_max;
    telemetry->tcp_ref_target_error_m = tcp_ref_target_error_m;
    telemetry->q_ref_watchdog_miss_count += runtime.q_ref_watchdog_miss_count;
    telemetry->tcp_ref_watchdog_miss_count += runtime.tcp_ref_watchdog_miss_count;
    telemetry->reference_supervision_state = reference_state;
    telemetry->reference_supervision_reason = reference_reason;
    telemetry->reference_supervision_fault_count = runtime.fault_count;
    telemetry->supervision_state = moreSevere(telemetry->supervision_state, reference_state);
    if (!reference_reason.empty()) {
        if (reference_state == RbpodoAsyncStreamingSupervisionState::Fault ||
            telemetry->last_failure.empty()) {
            telemetry->last_failure = reference_reason;
        }
    }
}

SafetyTrackingState trackingStateForArm(
    const DualArmConfig& config,
    ArmId arm_id,
    const RobotState& state,
    const JointArray& physical_baseline_q_deg
) {
    SafetyTrackingState tracking;
    tracking.source =
        controllerSimulationTrackingSourceName(config.safety.controller_simulation_tracking_error_source);
    if (!controllerSimulationTrackingReferenceActive(config, arm_id)) {
        tracking.source = "actual";
        return tracking;
    }

    tracking.override_tracking_q = true;
    tracking.tracking_q_deg = state.q_target_deg;
    tracking.source = "reference";
    // In controller-simulation the reference (jnt_ref) does not advance while the
    // sim servo is disabled, so a streaming Cartesian command that runs ahead would
    // be pinned by the tracking-error snap-back. Mark the tracking error advisory so
    // the safety filter reports it without snapping the command back. Same gate as
    // the existing non-latching advisory (controller_simulation_tracking_error_nonlatching).
    tracking.tracking_error_advisory =
        controllerSimulationMotionRequired(config) &&
        controllerSimulationMotionGateOpen(config) &&
        config.safety.controller_simulation_tracking_error_nonlatching;
    if (!state.has_valid_joint_state || !finiteJointArray(state.q_target_deg)) {
        tracking.source_valid = false;
        tracking.reason = "controller_simulation_reference_state_unavailable";
        return tracking;
    }
    if (!finiteJointArray(physical_baseline_q_deg) || !finiteJointArray(state.q_actual_deg)) {
        tracking.source_valid = false;
        tracking.reason = "controller_simulation_physical_baseline_unavailable";
        return tracking;
    }

    const double physical_motion_deg = maxAbsJointDelta(state.q_actual_deg, physical_baseline_q_deg);
    tracking.controller_simulation_physical_motion_detected =
        physical_motion_deg > config.safety.controller_simulation_physical_motion_threshold_deg;
    if (tracking.controller_simulation_physical_motion_detected &&
        config.safety.controller_simulation_physical_motion_policy ==
            ControllerSimulationPhysicalMotionPolicy::FaultLatch) {
        tracking.controller_simulation_physical_motion_fault = true;
        tracking.reason = "controller_simulation_physical_motion_detected";
    }
    return tracking;
}

struct StartupTrackingTargetSelection {
    JointArray q_deg{};
    std::string source = "actual";
    bool ok = true;
    std::string reason;
};

StartupTrackingTargetSelection startupTrackingReferenceForArm(
    const DualArmConfig& config,
    ArmId arm_id,
    const RobotState& state
) {
    StartupTrackingTargetSelection selection;
    if (!controllerSimulationTrackingReferenceActive(config, arm_id)) {
        selection.q_deg = state.q_actual_deg;
        selection.source = "actual";
        return selection;
    }

    selection.source = "reference";
    selection.q_deg = state.q_target_deg;
    if (!state.has_valid_joint_state || !finiteJointArray(state.q_target_deg)) {
        selection.ok = false;
        selection.reason = "controller_simulation_startup_reference_unavailable";
    }
    return selection;
}

void logStartupReferenceUnavailable(
    ArmId arm_id,
    const RobotState& state,
    const StartupTrackingTargetSelection& selection
) {
    std::cerr << "[ERROR] controller-simulation startup reference unavailable: "
              << (arm_id == ArmId::Left ? "left" : "right") << "\n"
              << "  startup_previous_target_source=" << selection.source << "\n"
              << "  reason=" << selection.reason << "\n"
              << "  has_valid_joint_state=" << (state.has_valid_joint_state ? "true" : "false") << "\n"
              << "  q_actual_deg=" << jointArrayDebugString(state.q_actual_deg) << "\n"
              << "  q_target_deg=" << jointArrayDebugString(state.q_target_deg) << "\n";
}

struct CartesianServoStateSelection {
    RobotState state;
    CartesianServoStateContext context;
    bool ok = true;
    std::string reason;
};

CartesianServoStateSelection selectCartesianServoStateForArm(
    const DualArmConfig& config,
    const ArmCommand& command,
    const RobotState& physical_state
) {
    CartesianServoStateSelection selection;
    selection.state = physical_state;
    selection.context.physical_q_actual_deg = physical_state.q_actual_deg;
    selection.context.reference_q_deg = physical_state.q_target_deg;
    selection.context.divergence_q_deg = physical_state.q_actual_deg;
    selection.context.q_reference_for_servo_valid =
        finiteJointArray(physical_state.q_target_deg) &&
        physical_state.tcp_ref_valid &&
        physical_state.tcp_ref_stand.has_value();

    const bool servo_uses_reference = controllerSimulationReferenceSourceActive(
        config,
        command,
        config.cartesian_control.controller_simulation_servo_state_source
    );
    const bool divergence_uses_reference = controllerSimulationReferenceSourceActive(
        config,
        command,
        config.cartesian_control.controller_simulation_divergence_source
    );

    selection.context.servo_state_source = servo_uses_reference
        ? "reference"
        : controllerSimulationStateSourceName(CartesianControllerSimulationStateSource::Actual);
    selection.context.divergence_source = divergence_uses_reference
        ? "reference"
        : controllerSimulationStateSourceName(CartesianControllerSimulationStateSource::Actual);

    if ((servo_uses_reference || divergence_uses_reference) &&
        !selection.context.q_reference_for_servo_valid) {
        selection.ok = false;
        selection.reason = "cartesian_reference_state_unavailable";
        return selection;
    }

    if (servo_uses_reference) {
        selection.state.q_actual_deg = physical_state.q_target_deg;
        selection.state.tcp_base = physical_state.tcp_ref_base;
        selection.state.tcp_stand = physical_state.tcp_ref_stand;
        selection.state.has_valid_tcp_pose = physical_state.tcp_ref_valid;
    }
    if (divergence_uses_reference) {
        selection.context.divergence_q_deg = physical_state.q_target_deg;
    }
    return selection;
}

bool controllerSimulationDiagnosticsSuspectGateOpen(const DualArmConfig& config) {
    return controllerSimulationMotionGateOpen(config) &&
        config.servo.allow_controller_simulation_diagnostics_suspect;
}

bool controllerSimulationInitErrorGateOpen(const DualArmConfig& config) {
    return controllerSimulationMotionRequired(config) &&
        controllerSimulationMotionGateOpen(config) &&
        config.servo.allow_controller_simulation_init_error;
}

bool controllerSimulationNotActivatedGateOpen(const DualArmConfig& config) {
    return controllerSimulationMotionRequired(config) &&
        controllerSimulationMotionGateOpen(config) &&
        config.servo.allow_controller_simulation_not_activated;
}

bool isAllowedControllerSimulationDiagnosticReason(
    const std::string& reason,
    bool tolerate_servo_disabled
) {
    return reason == "robot_fault" ||
        (tolerate_servo_disabled && reason == "servo_disabled");
}

bool isRbpodoDiagnosticsSuspectOnly(
    const ArmStartupValidationSnapshot& arm,
    bool tolerate_servo_disabled
) {
    if (arm.motion_ready) return true;
    if (!arm.acquisition_ok) return false;
    if (arm.diagnostic_error_source != "rbpodo_diagnostics_suspect") return false;
    if (!containsReason(arm, "robot_fault")) return false;
    for (const std::string& reason : arm.invalid_reasons) {
        if (!isAllowedControllerSimulationDiagnosticReason(reason, tolerate_servo_disabled)) {
            return false;
        }
    }
    return true;
}

bool isRbpodoInitErrorOnly(const ArmStartupValidationSnapshot& arm) {
    if (arm.motion_ready) return true;
    if (!arm.acquisition_ok) return false;
    if (arm.diagnostic_error_source != "rbpodo_init_error") return false;
    if (!containsReason(arm, "robot_fault")) return false;
    if (!containsReason(arm, "servo_disabled")) return false;
    for (const std::string& reason : arm.invalid_reasons) {
        if (reason != "robot_fault" && reason != "servo_disabled") return false;
    }
    return true;
}

bool controllerSimulationDiagnosticsSuspectStartupAllowed(
    const DualArmConfig& config,
    const StartupValidationSnapshot& validation
) {
    if (!controllerSimulationDiagnosticsSuspectGateOpen(config)) return false;
    if (!validation.acquisition_ok) return false;
    if (validation.motion_ready) return true;
    const bool tolerate_servo_disabled = controllerSimulationNotActivatedGateOpen(config);
    return isRbpodoDiagnosticsSuspectOnly(validation.left, tolerate_servo_disabled) &&
        isRbpodoDiagnosticsSuspectOnly(validation.right, tolerate_servo_disabled);
}

bool controllerSimulationInitErrorStartupAllowed(
    const DualArmConfig& config,
    const StartupValidationSnapshot& validation
) {
    if (!controllerSimulationInitErrorGateOpen(config)) return false;
    if (!validation.acquisition_ok) return false;
    if (validation.motion_ready) return true;
    return isRbpodoInitErrorOnly(validation.left) &&
        isRbpodoInitErrorOnly(validation.right);
}

bool controllerSimulationDiagnosticsSuspectStateAllowed(
    const DualArmConfig& config,
    const RobotState& state
) {
    return controllerSimulationDiagnosticsSuspectGateOpen(config) &&
        state.has_error &&
        state.diagnostic_error_source == "rbpodo_diagnostics_suspect" &&
        (state.servo_enabled || controllerSimulationNotActivatedGateOpen(config));
}

bool controllerSimulationInitErrorStateAllowed(
    const DualArmConfig& config,
    const RobotState& state
) {
    return controllerSimulationInitErrorGateOpen(config) &&
        state.has_error &&
        !state.servo_enabled &&
        state.diagnostic_error_source == "rbpodo_init_error";
}

bool controllerSimulationDiagnosticStateAllowed(
    const DualArmConfig& config,
    const RobotState& state
) {
    return controllerSimulationDiagnosticsSuspectStateAllowed(config, state) ||
        controllerSimulationInitErrorStateAllowed(config, state);
}

FaultContext clearControllerSimulationDiagnosticReadFault(
    const DualArmConfig& config,
    const RobotState& state,
    const FaultContext& fault,
    ArmId arm
) {
    if (fault.verdict == SafetyVerdict::Ok ||
        !controllerSimulationDiagnosticStateAllowed(config, state)) {
        return fault;
    }
    FaultContext cleared;
    cleared.backend_op = BackendOp::ReadState;
    cleared.arm = arm;
    return cleared;
}

RunMode runModeForArm(const DualArmConfig& config, ArmId arm_id) {
    return arm_id == ArmId::Left
        ? config.left_robot.run_mode
        : config.right_robot.run_mode;
}

CartesianSolveTelemetry cartesianUnavailableTelemetry(
    const RobotState& state,
    const CartesianControlConfig& config,
    const std::string& reason
) {
    CartesianSolveTelemetry telemetry;
    telemetry.attempted = true;
    telemetry.success = false;
    telemetry.status = "unavailable";
    telemetry.reason = reason;
    telemetry.fk_duration_us = state.fk_duration_us;
    telemetry.warn_ik_duration_us = config.warn_ik_duration_us;
    telemetry.fail_ik_duration_us = config.fail_ik_duration_us;
    return telemetry;
}

ArmCommand linearMoveContinuationCommand(
    const ArmCommand& hold_command,
    const CartesianServoPathState& path
) {
    ArmCommand command = hold_command;
    command.mode = ControlMode::TcpLinearMove;
    command.tcp_target_stand = path.target_tcp_stand;
    command.has_tcp_target = true;
    command.linear_move_duration_sec = path.duration_sec;
    command.has_linear_move_duration = true;
    return command;
}

BackendCallSnapshot readCallSnapshot(
    const BackendResult<RobotState>& result,
    const FaultContext& classified
) {
    BackendCallSnapshot snapshot;
    snapshot.ok = classified.verdict == SafetyVerdict::Ok;
    snapshot.accepted = snapshot.ok;
    const BackendError& error = snapshot.ok ? result.error : classified.backend_error;
    snapshot.backend_error_kind = toString(error.kind);
    snapshot.error_name = error.name;
    snapshot.error_code = error.code;
    snapshot.error_message = error.message;
    snapshot.duration_us = result.timing.duration_us;
    snapshot.read_exchange_timing = result.read_exchange_timing;
    return snapshot;
}

BackendCallSnapshot sendCallSnapshot(const SendServoJResult& result) {
    BackendCallSnapshot snapshot;
    snapshot.ok = result.accepted || result.error.kind == BackendErrorKind::SuppressedByPolicy;
    snapshot.accepted = result.accepted;
    snapshot.backend_error_kind = toString(result.error.kind);
    snapshot.error_name = result.error.name;
    snapshot.error_code = result.error.code;
    snapshot.error_message = result.error.message;
    snapshot.duration_us = result.timing.duration_us;
    snapshot.state_after_source = result.state_after_source;
    snapshot.ack_policy = result.ack_policy;
    snapshot.ack_observed = result.ack_observed;
    snapshot.controller_acceptance_observed = result.controller_acceptance_observed;
    snapshot.ack_wait_duration_us = result.ack_wait_duration_us;
    snapshot.rbpodo_waiting_ack = result.rbpodo_waiting_ack;
    snapshot.acceptance_semantics = result.acceptance_semantics;
    return snapshot;
}

BackendError suppressedSendError(const std::string& send_policy) {
    return backendError(
        BackendErrorKind::SuppressedByPolicy,
        "regular servo_j suppressed by send_policy=" + send_policy,
        "",
        send_policy
    );
}

uint64_t timeoutNs(double timeout_sec, uint64_t fallback_ns) {
    if (timeout_sec <= 0.0 || !std::isfinite(timeout_sec)) {
        return fallback_ns;
    }
    return static_cast<uint64_t>(timeout_sec * 1'000'000'000.0);
}

uint64_t addDeadlineNs(uint64_t host_time_ns, uint64_t timeout_ns) {
    if (host_time_ns == 0 || timeout_ns == 0) {
        return 0;
    }
    constexpr uint64_t kMax = ~uint64_t{0};
    if (kMax - host_time_ns < timeout_ns) {
        return kMax;
    }
    return host_time_ns + timeout_ns;
}

uint64_t commandSendDeadlineNs(
    const DualArmCommand& command,
    uint64_t command_host_time_ns,
    uint64_t fallback_timeout_ns
) {
    const uint64_t left_timeout_ns = timeoutNs(command.left.timeout_sec, fallback_timeout_ns);
    const uint64_t right_timeout_ns = timeoutNs(command.right.timeout_sec, fallback_timeout_ns);
    const uint64_t timeout_ns = std::min(left_timeout_ns, right_timeout_ns);
    return addDeadlineNs(command_host_time_ns, timeout_ns);
}

ArmWorkerTelemetry workerTelemetryOrDefault(const ArmWorker* worker) {
    return worker ? worker->telemetry() : ArmWorkerTelemetry{};
}

RbpodoAsyncStreamingTelemetry asyncTelemetryOrDefault(const ArmWorker* worker) {
    return worker ? worker->asyncStreamingTelemetry() : RbpodoAsyncStreamingTelemetry{};
}

std::optional<BackendTransportTelemetry> workerTransportTelemetry(const ArmWorker* worker) {
    return worker ? worker->transportTelemetry() : std::nullopt;
}

std::optional<BackendTransportTelemetry> backendTransportTelemetry(const IRobotBackend* backend) {
    return backend ? backend->transportTelemetry() : std::nullopt;
}

std::string workerStartupSummary(const ArmWorker* worker) {
    if (!worker) {
        return "{worker=null}";
    }
    const ArmWorkerStartupTelemetry telemetry = worker->startupTelemetry();
    const uint64_t now = nowSteadyNs();
    const double phase_age_ms =
        telemetry.phase_time_ns > 0 && now >= telemetry.phase_time_ns
            ? static_cast<double>(now - telemetry.phase_time_ns) / 1'000'000.0
            : 0.0;

    std::ostringstream out;
    out << "{backend=" << telemetry.backend_name
        << ",phase=" << telemetry.phase
        << ",phase_age_ms=" << phase_age_ms
        << ",latest_state_present=" << (telemetry.latest_state_present ? "true" : "false");
    if (!telemetry.last_op.empty()) {
        out << ",last_op=" << telemetry.last_op
            << ",last_ok=" << (telemetry.last_result_ok ? "true" : "false")
            << ",last_error_kind=" << telemetry.last_error_kind
            << ",last_error_name=" << telemetry.last_error_name;
        if (!telemetry.last_error_message.empty()) {
            out << ",last_error_message=" << telemetry.last_error_message;
        }
    }
    out << "}";
    return out.str();
}

uint64_t workerReadPeriodNs(const ServoConfig& config) {
    constexpr double kNsPerSecond = 1'000'000'000.0;
    const double period_ns = config.worker_read_period_sec * kNsPerSecond;
    if (!std::isfinite(period_ns) || period_ns <= 0.0) {
        return 10'000'000;
    }
    constexpr double kMaxUint64AsDouble =
        static_cast<double>(std::numeric_limits<uint64_t>::max());
    if (period_ns >= kMaxUint64AsDouble) {
        return std::numeric_limits<uint64_t>::max();
    }
    return std::max<uint64_t>(1, static_cast<uint64_t>(std::llround(period_ns)));
}

ArmWorkerOptions workerOptions(const DualArmConfig& config) {
    ArmWorkerOptions options;
    options.read_period_ns = workerReadPeriodNs(config.servo);
    options.rbpodo_async_streaming_enabled = config.servo.rbpodo_async_streaming.enable;
    options.rbpodo_async_streaming_mode = config.servo.rbpodo_async_streaming.mode;
    options.rbpodo_async_max_pending_age_ms = config.servo.rbpodo_async_streaming.max_pending_age_ms;
    options.controller_simulation_timing_reject_tolerance_enabled =
        controllerSimulationMotionGateOpen(config);
    options.rbpodo_async_ack_supervision =
        config.servo.rbpodo_async_streaming.ack_supervision;
    options.rbpodo_async_reference_supervision =
        config.servo.rbpodo_async_streaming.reference_supervision;
    // Cadence ownership only makes sense in worker I/O: it is the worker thread
    // that would own the period. In direct I/O the servo loop still owns the
    // rhythm, so leave send_period_ns at 0 (event-driven) rather than handing the
    // worker a cadence it is not running.
    if (config.servo.io_model == ServoIoModel::Worker && config.queue_sync.enable) {
        const int rate_hz = config.servo.rate_hz > 0 ? config.servo.rate_hz : 500;
        options.send_period_ns = static_cast<uint64_t>(1'000'000'000LL / rate_hz);
        options.queue_sync = config.queue_sync;
    }
    return options;
}

uint64_t workerStartupTimeoutNs(const DualArmConfig& config) {
    double timeout_sec = std::max(0.1, config.servo.command_timeout_sec);
    if (config.servo.io_model == ServoIoModel::Worker ||
        config.servo.rbpodo_async_streaming.enable) {
        timeout_sec = std::max(timeout_sec, 1.0);
    }
    return static_cast<uint64_t>(timeout_sec * 1'000'000'000.0);
}

LatchedFaultContextSnapshot faultContextSnapshot(const FaultContext& context) {
    LatchedFaultContextSnapshot snapshot;
    snapshot.verdict = toString(context.verdict);
    snapshot.domain = toString(context.domain);
    snapshot.arm = toString(context.arm);
    snapshot.backend_op = toString(context.backend_op);
    snapshot.backend_error_kind = toString(context.backend_error.kind);
    snapshot.backend_error_name = context.backend_error.name;
    snapshot.backend_error_code = context.backend_error.code;
    snapshot.retryable = context.retryable;
    snapshot.recoverable = context.recoverable;
    snapshot.robot_fault = context.backend_error.robot_fault;
    snapshot.transport_fault = context.backend_error.transport_fault;
    snapshot.state_after_source = context.state_after_source;
    snapshot.reason = context.reason;
    return snapshot;
}

LatchedDualFaultContext dualReadFaultContext(
    const FaultContext& left,
    const FaultContext& right
) {
    LatchedDualFaultContext contexts;
    if (left.verdict != SafetyVerdict::Ok) {
        contexts.left = left;
    }
    if (right.verdict != SafetyVerdict::Ok) {
        contexts.right = right;
    }
    if (contexts.left.has_value()) {
        contexts.top_level = contexts.left;
    } else if (contexts.right.has_value()) {
        contexts.top_level = contexts.right;
    }
    return contexts;
}

std::optional<FaultContext> asyncSupervisionFaultContext(
    const RbpodoAsyncStreamingTelemetry& telemetry,
    ArmId arm
) {
    if (telemetry.supervision_state != RbpodoAsyncStreamingSupervisionState::Fault) {
        return std::nullopt;
    }

    FaultContext context;
    context.verdict = SafetyVerdict::SendFailure;
    context.domain = FaultDomain::Backend;
    context.arm = arm;
    context.backend_op = BackendOp::SendServoJ;
    context.backend_error = backendError(
        BackendErrorKind::CommandTimeout,
        telemetry.last_failure.empty()
            ? "rbpodo async streaming supervision fault"
            : telemetry.last_failure,
        "",
        telemetry.last_failure.empty()
            ? "rbpodo_async_supervision_fault"
            : telemetry.last_failure,
        true,
        true
    );
    context.reason = "rbpodo async streaming supervision fault";
    context.retryable = true;
    context.recoverable = true;
    context.suppress_regular_servo = true;
    return context;
}

LatchedDualFaultContext asyncSupervisionFaultContexts(
    const RbpodoAsyncStreamingTelemetry& left_telemetry,
    const RbpodoAsyncStreamingTelemetry& right_telemetry
) {
    LatchedDualFaultContext contexts;
    const std::optional<FaultContext> left =
        asyncSupervisionFaultContext(left_telemetry, ArmId::Left);
    const std::optional<FaultContext> right =
        asyncSupervisionFaultContext(right_telemetry, ArmId::Right);
    contexts.left = left;
    contexts.right = right;
    if (left.has_value()) {
        contexts.top_level = left;
    } else if (right.has_value()) {
        contexts.top_level = right;
    }
    return contexts;
}

std::string asyncSupervisionFaultReason(const LatchedDualFaultContext& contexts) {
    if (!contexts.top_level.has_value()) {
        return "unknown";
    }
    const FaultContext& context = *contexts.top_level;
    if (context.backend_error.name != "None" && !context.backend_error.name.empty()) {
        return context.backend_error.name;
    }
    if (!context.reason.empty()) {
        return context.reason;
    }
    return "rbpodo_async_supervision_fault";
}

FaultContext contextWithReason(FaultContext context, const std::string& reason) {
    if (context.reason.empty()) {
        context.reason = reason;
    }
    return context;
}
}

namespace {
// Map an async mesh CollisionMonitor verdict onto the SelfCollisionResult the
// telemetry/viser pipeline already understands (so the existing overlay + witness
// markers render mesh-based results unchanged). violated = hard breach (red).
SelfCollisionResult selfCollisionResultFromVerdict(
    const CollisionVerdict& v,
    const CollisionMonitorConfig& cfg
) {
    (void)cfg;
    SelfCollisionResult sc;
    sc.checked = v.valid;
    sc.min_clearance_m = v.min_clearance_m;
    sc.violated = v.valid && v.hard_violation;
    if (!v.near.empty()) {
        const CollisionNearPair& p = v.near.front();
        sc.has_closest_points = true;
        sc.closest_point_a_m = {p.p_a.x(), p.p_a.y(), p.p_a.z()};
        sc.closest_point_b_m = {p.p_b.x(), p.p_b.y(), p.p_b.z()};
        const auto side = [](const std::string& n) {
            if (n.find("left") != std::string::npos) return 0;
            if (n.find("right") != std::string::npos) return 1;
            return 2;  // stand
        };
        const int a = side(p.name_a);
        const int b = side(p.name_b);
        if ((a == 0 && b == 1) || (a == 1 && b == 0)) sc.pair = "left_right";
        else if ((a == 0 && b == 2) || (a == 2 && b == 0)) sc.pair = "left_stand";
        else if ((a == 1 && b == 2) || (a == 2 && b == 1)) sc.pair = "right_stand";
        else sc.pair = "all";
        sc.stand_capsule = (a == 2) ? p.name_a : (b == 2 ? p.name_b : std::string());
    }
    return sc;
}

ScopeSample scopeSampleFromServoSample(const ServoSample& sample) {
    ScopeSample scope;
    scope.t_host_ns = sample.loop_start_time_ns;
    scope.l_robot_ns = sample.left_state.robot_time_ns;
    scope.r_robot_ns = sample.right_state.robot_time_ns;
    scope.l_sent = sample.left_sent_q_deg;
    scope.r_sent = sample.right_sent_q_deg;
    scope.l_ref = sample.left_state.q_target_deg;
    scope.r_ref = sample.right_state.q_target_deg;
    scope.l_actual = sample.left_state.q_actual_deg;
    scope.r_actual = sample.right_state.q_actual_deg;
    return scope;
}
}  // namespace

uint8_t RealtimeTimingAccumulator::Histogram::binFor(uint64_t value_ns) {
    if (value_ns <= 1) return 0;
    const unsigned exponent = 63U - static_cast<unsigned>(__builtin_clzll(value_ns));
    if (exponent >= 31U) return static_cast<uint8_t>(kHistogramBins - 1);
    const uint64_t base = uint64_t{1} << exponent;
    const uint64_t quarter = std::max<uint64_t>(1, base >> 2U);
    const unsigned sub = static_cast<unsigned>(
        std::min<uint64_t>(3, (value_ns - base) / quarter));
    return static_cast<uint8_t>(exponent * 4U + sub);
}

uint64_t RealtimeTimingAccumulator::Histogram::upperBoundNs(uint8_t bin) {
    const unsigned exponent = static_cast<unsigned>(bin) / 4U;
    const unsigned sub = static_cast<unsigned>(bin) % 4U;
    const uint64_t base = uint64_t{1} << exponent;
    const uint64_t quarter = std::max<uint64_t>(1, base >> 2U);
    return base + static_cast<uint64_t>(sub + 1U) * quarter - 1U;
}

void RealtimeTimingAccumulator::Histogram::add(uint8_t bin) {
    ++count[bin];
    ++total;
}

void RealtimeTimingAccumulator::Histogram::remove(uint8_t bin) {
    if (count[bin] > 0) --count[bin];
    if (total > 0) --total;
}

uint64_t RealtimeTimingAccumulator::Histogram::percentileUpperNs(double quantile) const {
    if (total == 0) return 0;
    const uint32_t rank = static_cast<uint32_t>(
        std::ceil(std::clamp(quantile, 0.0, 1.0) * static_cast<double>(total)));
    uint32_t cumulative = 0;
    for (std::size_t i = 0; i < count.size(); ++i) {
        cumulative += count[i];
        if (cumulative >= std::max<uint32_t>(1, rank)) {
            return upperBoundNs(static_cast<uint8_t>(i));
        }
    }
    return maxUpperNs();
}

uint64_t RealtimeTimingAccumulator::Histogram::maxUpperNs() const {
    for (std::size_t i = count.size(); i > 0; --i) {
        if (count[i - 1] > 0) return upperBoundNs(static_cast<uint8_t>(i - 1));
    }
    return 0;
}

uint64_t RealtimeTimingAccumulator::absoluteDifference(uint64_t lhs, uint64_t rhs) {
    return lhs >= rhs ? lhs - rhs : rhs - lhs;
}

uint64_t RealtimeTimingAccumulator::receiptPhaseNs(
    uint64_t host_time_ns,
    uint64_t scheduled_wake_ns,
    uint64_t nominal_period_ns
) {
    if (host_time_ns == 0 || scheduled_wake_ns == 0 || nominal_period_ns == 0) return 0;
    if (host_time_ns >= scheduled_wake_ns) {
        return (host_time_ns - scheduled_wake_ns) % nominal_period_ns;
    }
    const uint64_t before_ns = (scheduled_wake_ns - host_time_ns) % nominal_period_ns;
    return before_ns == 0 ? 0 : nominal_period_ns - before_ns;
}

RealtimeTimingAccumulator::ArmEntry RealtimeTimingAccumulator::makeArmEntry(
    const RealtimeFeedbackTimingTick& feedback,
    uint64_t loop_end_ns,
    uint64_t scheduled_wake_ns,
    uint64_t nominal_period_ns,
    ArmAggregate* aggregate
) {
    ArmEntry entry;
    entry.observed = feedback.host_time_ns > 0;
    if (!entry.observed) return entry;

    const bool regressed = aggregate->previous_host_time_ns > 0 &&
        feedback.host_time_ns < aggregate->previous_host_time_ns;
    entry.frame = !feedback.explicit_cached_hold && !regressed &&
        (aggregate->previous_host_time_ns == 0 ||
         feedback.host_time_ns > aggregate->previous_host_time_ns);
    entry.held = feedback.explicit_cached_hold ||
        (aggregate->previous_host_time_ns > 0 &&
         feedback.host_time_ns == aggregate->previous_host_time_ns);

    if (entry.frame) {
        if (aggregate->previous_host_time_ns > 0) {
            aggregate->last_period_ns = feedback.host_time_ns - aggregate->previous_host_time_ns;
            aggregate->last_jitter_ns = absoluteDifference(
                aggregate->last_period_ns, nominal_period_ns);
            entry.period_valid = true;
            entry.period_bin = Histogram::binFor(aggregate->last_period_ns);
            entry.jitter_bin = Histogram::binFor(aggregate->last_jitter_ns);
        }
        aggregate->previous_host_time_ns = feedback.host_time_ns;
    }

    aggregate->last_age_ns = loop_end_ns >= feedback.host_time_ns
        ? loop_end_ns - feedback.host_time_ns : 0;
    aggregate->last_phase_ns = receiptPhaseNs(
        feedback.host_time_ns, scheduled_wake_ns, nominal_period_ns);
    entry.age_bin = Histogram::binFor(aggregate->last_age_ns);
    entry.phase_bin = Histogram::binFor(aggregate->last_phase_ns);

    if (entry.frame && feedback.robot_time_ns > 0) {
        aggregate->robot_time_available = true;
        if (aggregate->previous_robot_time_ns > 0 &&
            feedback.robot_time_ns < aggregate->previous_robot_time_ns) {
            aggregate->robot_time_monotonic = false;
        }
        entry.fresh = aggregate->previous_robot_time_ns == 0 ||
            feedback.robot_time_ns > aggregate->previous_robot_time_ns;
        aggregate->previous_robot_time_ns = feedback.robot_time_ns;
    }
    return entry;
}

void RealtimeTimingAccumulator::addArmEntry(
    const ArmEntry& entry,
    ArmAggregate* aggregate
) {
    if (!entry.observed) return;
    aggregate->age.add(entry.age_bin);
    aggregate->phase.add(entry.phase_bin);
    if (entry.frame) ++aggregate->frame_count;
    if (entry.fresh) ++aggregate->fresh_count;
    if (entry.held) ++aggregate->held_count;
    if (entry.period_valid) {
        aggregate->period.add(entry.period_bin);
        aggregate->jitter.add(entry.jitter_bin);
    }
}

void RealtimeTimingAccumulator::removeArmEntry(
    const ArmEntry& entry,
    ArmAggregate* aggregate
) {
    if (!entry.observed) return;
    aggregate->age.remove(entry.age_bin);
    aggregate->phase.remove(entry.phase_bin);
    if (entry.frame && aggregate->frame_count > 0) --aggregate->frame_count;
    if (entry.fresh && aggregate->fresh_count > 0) --aggregate->fresh_count;
    if (entry.held && aggregate->held_count > 0) --aggregate->held_count;
    if (entry.period_valid) {
        aggregate->period.remove(entry.period_bin);
        aggregate->jitter.remove(entry.jitter_bin);
    }
}

void RealtimeTimingAccumulator::removeOldest() {
    if (size_ == 0) return;
    const Entry& entry = entries_[head_];
    period_.remove(entry.period_bin);
    jitter_.remove(entry.jitter_bin);
    wake_.remove(entry.wake_bin);
    if (entry.send_cycle) {
        pre_send_.remove(entry.pre_send_bin);
        send_duration_.remove(entry.send_duration_bin);
        if (send_cycle_count_ > 0) --send_cycle_count_;
    }
    if (entry.deadline_miss && deadline_miss_count_ > 0) --deadline_miss_count_;
    if (entry.catch_up && catch_up_count_ > 0) --catch_up_count_;
    removeArmEntry(entry.left, &left_);
    removeArmEntry(entry.right, &right_);
    head_ = (head_ + 1) % entries_.size();
    --size_;
}

void RealtimeTimingAccumulator::reset() {
    *this = RealtimeTimingAccumulator{};
}

void RealtimeTimingAccumulator::add(const RealtimeTimingTick& tick) {
    if (tick.loop_start_ns == 0 || tick.nominal_period_ns == 0) return;
    nominal_period_ns_ = tick.nominal_period_ns;
    while (size_ > 0) {
        const uint64_t oldest_ns = entries_[head_].loop_start_ns;
        if (tick.loop_start_ns >= oldest_ns &&
            tick.loop_start_ns - oldest_ns >= kWindowNs) {
            removeOldest();
        } else {
            break;
        }
    }
    if (size_ == entries_.size()) removeOldest();

    Entry entry;
    entry.loop_start_ns = tick.loop_start_ns;
    last_period_ns_ = previous_loop_start_ns_ > 0 &&
        tick.loop_start_ns >= previous_loop_start_ns_
        ? tick.loop_start_ns - previous_loop_start_ns_
        : tick.nominal_period_ns;
    previous_loop_start_ns_ = tick.loop_start_ns;
    last_jitter_ns_ = absoluteDifference(last_period_ns_, tick.nominal_period_ns);
    last_wake_ns_ = tick.scheduled_wake_ns > 0 &&
        tick.loop_start_ns >= tick.scheduled_wake_ns
        ? tick.loop_start_ns - tick.scheduled_wake_ns : 0;
    last_pre_send_ns_ = tick.pre_send_ns;
    last_send_duration_ns_ = tick.send_duration_ns;
    entry.period_bin = Histogram::binFor(last_period_ns_);
    entry.jitter_bin = Histogram::binFor(last_jitter_ns_);
    entry.wake_bin = Histogram::binFor(last_wake_ns_);
    entry.pre_send_bin = Histogram::binFor(last_pre_send_ns_);
    entry.send_duration_bin = Histogram::binFor(last_send_duration_ns_);
    entry.deadline_miss = tick.scheduled_wake_ns > 0 &&
        tick.loop_end_ns > tick.scheduled_wake_ns + tick.nominal_period_ns;
    entry.catch_up = tick.scheduled_wake_ns > 0 &&
        tick.previous_sleep_enter_ns >= tick.scheduled_wake_ns;
    entry.send_cycle = tick.send_cycle;
    entry.left = makeArmEntry(
        tick.left_feedback, tick.loop_end_ns, tick.scheduled_wake_ns,
        tick.nominal_period_ns, &left_);
    entry.right = makeArmEntry(
        tick.right_feedback, tick.loop_end_ns, tick.scheduled_wake_ns,
        tick.nominal_period_ns, &right_);

    const std::size_t write_index = (head_ + size_) % entries_.size();
    entries_[write_index] = entry;
    ++size_;
    period_.add(entry.period_bin);
    jitter_.add(entry.jitter_bin);
    wake_.add(entry.wake_bin);
    if (entry.send_cycle) {
        ++send_cycle_count_;
        pre_send_.add(entry.pre_send_bin);
        send_duration_.add(entry.send_duration_bin);
    }
    if (entry.deadline_miss) ++deadline_miss_count_;
    if (entry.catch_up) ++catch_up_count_;
    addArmEntry(entry.left, &left_);
    addArmEntry(entry.right, &right_);
}

FeedbackRealtimeTimingTelemetry RealtimeTimingAccumulator::armSnapshot(
    const ArmAggregate& aggregate,
    double window_sec
) {
    FeedbackRealtimeTimingTelemetry out;
    if (window_sec > 0.0) {
        out.frame_rate_hz = static_cast<double>(aggregate.frame_count) / window_sec;
        out.fresh_rate_hz = static_cast<double>(aggregate.fresh_count) / window_sec;
    }
    out.held_count = aggregate.held_count;
    const auto range_ms = [](uint64_t last_ns, const Histogram& histogram) {
        RealtimeTimingRange range;
        range.last = static_cast<double>(last_ns) / 1'000'000.0;
        range.p95 = static_cast<double>(histogram.percentileUpperNs(0.95)) / 1'000'000.0;
        range.max = static_cast<double>(histogram.maxUpperNs()) / 1'000'000.0;
        return range;
    };
    const auto range_us = [](uint64_t last_ns, const Histogram& histogram) {
        RealtimeTimingRange range;
        range.last = static_cast<double>(last_ns) / 1'000.0;
        range.p95 = static_cast<double>(histogram.percentileUpperNs(0.95)) / 1'000.0;
        range.max = static_cast<double>(histogram.maxUpperNs()) / 1'000.0;
        return range;
    };
    out.period_ms = range_ms(aggregate.last_period_ns, aggregate.period);
    out.jitter_ms = range_ms(aggregate.last_jitter_ns, aggregate.jitter);
    out.age_us = range_us(aggregate.last_age_ns, aggregate.age);
    out.phase_us = range_us(aggregate.last_phase_ns, aggregate.phase);
    // robot_time_ns is retained as a diagnostic only: some controller/firmware
    // modes expose an unavailable or suspect field, so monotonicity alone is not
    // enough to promote it to an acquisition-freshness clock.
    out.freshness_reliable = false;
    out.robot_time_available = aggregate.robot_time_available;
    out.robot_time_monotonic = aggregate.robot_time_monotonic;
    return out;
}

RealtimeTimingTelemetry RealtimeTimingAccumulator::snapshot() const {
    RealtimeTimingTelemetry out;
    if (size_ == 0 || nominal_period_ns_ == 0) return out;
    const Entry& oldest = entries_[head_];
    const Entry& newest = entries_[(head_ + size_ - 1) % entries_.size()];
    const uint64_t covered_ns = newest.loop_start_ns >= oldest.loop_start_ns
        ? newest.loop_start_ns - oldest.loop_start_ns + nominal_period_ns_
        : nominal_period_ns_;
    out.window_sec = static_cast<double>(covered_ns) * 1e-9;
    out.servo.target_rate_hz = 1e9 / static_cast<double>(nominal_period_ns_);
    out.servo.observed_rate_hz = static_cast<double>(size_) / out.window_sec;
    out.servo.send_rate_hz = static_cast<double>(send_cycle_count_) / out.window_sec;
    const auto range_ms = [](uint64_t last_ns, const Histogram& histogram) {
        return RealtimeTimingRange{
            static_cast<double>(last_ns) / 1'000'000.0,
            static_cast<double>(histogram.percentileUpperNs(0.95)) / 1'000'000.0,
            static_cast<double>(histogram.maxUpperNs()) / 1'000'000.0,
        };
    };
    const auto range_us = [](uint64_t last_ns, const Histogram& histogram) {
        return RealtimeTimingRange{
            static_cast<double>(last_ns) / 1'000.0,
            static_cast<double>(histogram.percentileUpperNs(0.95)) / 1'000.0,
            static_cast<double>(histogram.maxUpperNs()) / 1'000.0,
        };
    };
    out.servo.period_ms = range_ms(last_period_ns_, period_);
    out.servo.jitter_ms = range_ms(last_jitter_ns_, jitter_);
    out.servo.wake_latency_us = range_us(last_wake_ns_, wake_);
    out.servo.pre_send_us = range_us(last_pre_send_ns_, pre_send_);
    out.servo.send_duration_us = range_us(last_send_duration_ns_, send_duration_);
    out.servo.deadline_miss_count = deadline_miss_count_;
    out.servo.catch_up_count = catch_up_count_;
    out.left_feedback = armSnapshot(left_, out.window_sec);
    out.right_feedback = armSnapshot(right_, out.window_sec);
    return out;
}







DualArmServoLoop::DualArmServoLoop(
    std::unique_ptr<IRobotBackend> left_robot,
    std::unique_ptr<IRobotBackend> right_robot,
    const DualArmConfig& config,
    CommandBuffer* command_buffer,
    ServoLogger* logger,
    std::shared_ptr<IKinematics> kinematics,
    ScopePublisher* scope_publisher
) : left_robot_(std::move(left_robot)),
    right_robot_(std::move(right_robot)),
    config_(config),
    command_buffer_(command_buffer),
    logger_(logger),
    scope_publisher_(scope_publisher),
    kinematics_(nullptr),
    kinematics_injected_(kinematics != nullptr),
    left_traj_filter_(config.servo, config.safety),
    right_traj_filter_(config.servo, config.safety),
    safety_filter_(config.safety) {
    bool profile_found = false;
    const TcpPoseTargetProfileConfig initial_profile =
        selectTcpPoseTargetProfile(config.cartesian_control, config.cartesian_control.tcp_pose_target_profile_default, &profile_found);
    left_pose_track_smd_ = SmdPoseTracker(initial_profile.pose_track_smd);
    right_pose_track_smd_ = SmdPoseTracker(initial_profile.pose_track_smd);
    left_pose_track_profile_name_ = initial_profile.name;
    right_pose_track_profile_name_ = initial_profile.name;

    // ---- force control ------------------------------------------------------
    const double control_period_sec = 1.0 / std::max(1, config.servo.rate_hz);
    left_ft_pipeline_.configure(config.force_torque.left, control_period_sec);
    right_ft_pipeline_.configure(config.force_torque.right, control_period_sec);
    left_overlay_.configure(config.force_control, control_period_sec);
    right_overlay_.configure(config.force_control, control_period_sec);
    left_force_gate_.configure(config.force_control, control_period_sec);
    right_force_gate_.configure(config.force_control, control_period_sec);
    if (config.force_control.enable) {
        // Print the LIVE law at boot. A force law that only exists in a yaml nobody
        // opened is a law nobody can check against what the arm actually did.
        const auto axis_line = [](const char* name, const ForceAxisConfig& a) {
            std::cerr << "[INFO]   " << name << " m=" << a.m << " b=" << a.b << " k=" << a.k
                      << " mode=" << (a.mode == ForceAxisMode::Force      ? "force"
                                      : a.mode == ForceAxisMode::Rigid    ? "rigid"
                                                                          : "compliance");
            if (a.mode == ForceAxisMode::Force) std::cerr << " ref=" << a.ref_force;
            std::cerr << "\n";
        };
        std::cerr << "[INFO] force_control ENABLED\n";
        const char* tnames[3] = {"x ", "y ", "z "};
        const char* rnames[3] = {"rx", "ry", "rz"};
        for (int i = 0; i < 3; ++i) axis_line(tnames[i], config.force_control.translation[i]);
        for (int i = 0; i < 3; ++i) axis_line(rnames[i], config.force_control.rotation[i]);
        std::cerr << "[INFO]   gate " << (config.force_control.gate_enable ? "ON" : "OFF")
                  << " converges to " << config.force_control.gate_max_force_n << " N / "
                  << config.force_control.gate_max_torque_nm << " Nm (close "
                  << config.force_control.gate_close_tau_s << " s, open "
                  << config.force_control.gate_open_tau_s << " s)\n";
        std::cerr << "[INFO]   fence " << config.force_control.max_deviation_m * 1e3 << " mm / "
                  << config.force_control.max_deviation_rad * 180.0 / M_PI << " deg"
                  << (config.force_control.hold_compliance ? ", Hold is COMPLIANT" : "")
                  << "\n";
        // The deviation a converged contact will settle at, printed because it is the
        // number an operator can check against the arm with a ruler.
        const double k = config.force_control.translation[2].k;
        if (k > 0.0 && config.force_control.gate_max_force_n > 0.0) {
            std::cerr << "[INFO]   converged deviation = max_force_n / k = "
                      << config.force_control.gate_max_force_n / k * 1e3 << " mm\n";
        }
    }
    left_output_ma_ = JointMovingAverage(config.servo.output_moving_average_window);
    right_output_ma_ = JointMovingAverage(config.servo.output_moving_average_window);
    kinematics_ = kinematics ? std::move(kinematics) : makeKinematicsProvider(config);
    runtime_floor_z_m_.store(config.safety.floor_constraint.z_min_m);
    runtime_floor_enabled_.store(config.safety.floor_constraint.enable);
    for (int k = 0; k < 3; ++k) {
        runtime_roi_min_m_[k].store(config.safety.roi_box.min_m[k]);
        runtime_roi_max_m_[k].store(config.safety.roi_box.max_m[k]);
    }
    // Seed the user floor plane from config. It is only active once a usable plane
    // is configured (has_initial_plane); otherwise it stays disabled until a
    // SetUserSafetyFloorPlane command arrives, even when enable=true.
    for (int k = 0; k < 3; ++k) {
        runtime_user_floor_point_m_[k].store(config.safety.user_floor_constraint.point_m[k]);
        runtime_user_floor_normal_[k].store(config.safety.user_floor_constraint.normal[k]);
    }
    runtime_user_floor_margin_m_.store(config.safety.user_floor_constraint.margin_m);
    runtime_user_floor_enabled_.store(config.safety.user_floor_constraint.enable &&
                                      config.safety.user_floor_constraint.has_initial_plane);
    // URDF mesh self-collision: spin up the async monitor (off the servo_j path).
    // Throws on a bad URDF/mesh path — fail closed at startup rather than run a
    // real robot with the guard silently disabled. The single self_collision.enable
    // flag is the sole switch; there is no capsule fallback path.
    if (config_.safety.self_collision.enable) {
        const auto& m = config_.safety.self_collision.mesh;
        collision_monitor_cfg_.enable = true;
        collision_monitor_cfg_.unified_urdf = m.unified_urdf;
        collision_monitor_cfg_.package_dirs = m.package_dirs;
        collision_monitor_cfg_.pika_gripper_mesh = m.pika_gripper_mesh;
        collision_monitor_cfg_.pika_gripper_base_mesh = m.pika_gripper_base_mesh;
        collision_monitor_cfg_.pika_finger_left_mesh = m.pika_finger_left_mesh;
        collision_monitor_cfg_.pika_finger_right_mesh = m.pika_finger_right_mesh;
        collision_monitor_cfg_.gripper_finger_travel_m = m.gripper_finger_travel_m;
        collision_monitor_cfg_.stand_frame = m.stand_frame;
        collision_monitor_cfg_.left_prefix = m.left_prefix;
        collision_monitor_cfg_.right_prefix = m.right_prefix;
        collision_monitor_cfg_.stand_ignore_arm_substrings = m.stand_ignore_arm_substrings;
        collision_monitor_cfg_.left_arm_root_frame = m.left_arm_root_frame;
        collision_monitor_cfg_.right_arm_root_frame = m.right_arm_root_frame;
        collision_monitor_cfg_.check_intra_arm = m.check_intra_arm;
        collision_monitor_cfg_.intra_arm_min_chain_separation = m.intra_arm_min_chain_separation;
        collision_monitor_cfg_.disabled_collision_pairs = m.disabled_collision_pairs;
        collision_monitor_cfg_.debug_pair_curation = m.debug_pair_curation;
        collision_monitor_cfg_.swept_samples = m.swept_samples;
        collision_monitor_cfg_.d_hard_m = m.d_hard_m;
        collision_monitor_cfg_.d_slow_m = m.d_slow_m;
        collision_monitor_cfg_.a_brake_m_s2 = m.a_brake_m_s2;
        collision_monitor_cfg_.hyst_m = m.hyst_m;
        collision_monitor_cfg_.projection_iterations = m.projection_iterations;
        collision_monitor_cfg_.recover_speed_m_s = m.recover_speed_m_s;
        collision_monitor_cfg_.latency_s = m.latency_s;
        collision_monitor_cfg_.max_staleness_s = m.max_staleness_s;
        collision_monitor_cfg_.monitor_core = m.monitor_core;
        collision_monitor_cfg_.max_near_pairs = m.max_near_pairs;
        // External-collision (arm<->floor) barrier set: separate d_hard so the floor
        // can be approached closer than the robot approaches itself.
        collision_monitor_cfg_.external_d_hard_m = m.external.d_hard_m;
        collision_monitor_cfg_.external_d_slow_m = m.external.d_slow_m;
        collision_monitor_cfg_.external_a_brake_m_s2 = m.external.a_brake_m_s2;
        collision_monitor_cfg_.external_hyst_m = m.external.hyst_m;
        collision_monitor_cfg_.external_recover_speed_m_s = m.external.recover_speed_m_s;
        collision_monitor_cfg_.external_latency_s = m.external.latency_s;
        const auto inherit = [](double value, double self_value) {
            return value > 0.0 ? value : self_value;
        };
        collision_monitor_cfg_.intra_arm_d_hard_m =
            inherit(m.intra_arm.d_hard_m, m.d_hard_m);
        collision_monitor_cfg_.intra_arm_d_slow_m =
            inherit(m.intra_arm.d_slow_m, m.d_slow_m);
        collision_monitor_cfg_.intra_arm_a_brake_m_s2 =
            inherit(m.intra_arm.a_brake_m_s2, m.a_brake_m_s2);
        collision_monitor_cfg_.intra_arm_hyst_m =
            inherit(m.intra_arm.hyst_m, m.hyst_m);
        collision_monitor_cfg_.intra_arm_recover_speed_m_s =
            inherit(m.intra_arm.recover_speed_m_s, m.recover_speed_m_s);
        collision_monitor_cfg_.intra_arm_latency_s =
            inherit(m.intra_arm.latency_s, m.latency_s);
        collision_monitor_cfg_.external_boxes.enable = m.external_boxes.enable;
        collision_monitor_cfg_.external_boxes.max_count = m.external_boxes.max_count;
        collision_monitor_cfg_.external_boxes.size_m = m.external_boxes.size_m;
        collision_monitor_cfg_.external_boxes.margin_m = m.external_boxes.margin_m;
        collision_monitor_cfg_.external_boxes.monitor_only = m.external_boxes.monitor_only;
        collision_monitor_cfg_.external_boxes.stale_timeout_s = m.external_boxes.stale_timeout_s;
        collision_monitor_cfg_.external_boxes.stale_policy = m.external_boxes.stale_policy;
        // Box-only keep-out barrier set (separate from the floor's external_* above).
        collision_monitor_cfg_.external_box_d_hard_m = m.external_boxes.barrier.d_hard_m;
        collision_monitor_cfg_.external_box_d_slow_m = m.external_boxes.barrier.d_slow_m;
        collision_monitor_cfg_.external_box_a_brake_m_s2 = m.external_boxes.barrier.a_brake_m_s2;
        collision_monitor_cfg_.external_box_hyst_m = m.external_boxes.barrier.hyst_m;
        collision_monitor_cfg_.external_box_recover_speed_m_s = m.external_boxes.barrier.recover_speed_m_s;
        collision_monitor_cfg_.external_box_latency_s = m.external_boxes.barrier.latency_s;
        collision_monitor_cfg_.extra_collision.clear();
        for (const auto& e : m.extra_collision) {
            ExtraCollisionShape s;
            s.name = e.name;
            s.shape = e.shape;
            s.parent_frame = e.parent_frame;
            s.size_m = e.size_m;
            s.radius_m = e.radius_m;
            s.length_m = e.length_m;
            s.xyz_m = e.xyz_m;
            s.rpy = e.rpy;
            collision_monitor_cfg_.extra_collision.push_back(s);
        }
        // Whole-arm floor: inject a large thin static box (world/stand frame) so the
        // SAME mesh barrier that guards arm<->arm / arm<->stand also keeps EVERY arm
        // link above the floor. The floor_constraint plane only checks the TCP + its
        // offset points (an elbow/wrist can dip below it); this box closes that gap
        // and is also what the InitMotion planner's oracle avoids. Top face at z_m.
        if (m.ground_plane.enable) {
            const auto& g = m.ground_plane;
            ExtraCollisionShape s;
            s.name = "ground_plane";
            s.shape = "box";
            s.parent_frame = g.parent_frame;
            s.size_m = {g.size_m[0], g.size_m[1], g.thickness_m};
            s.xyz_m = {0.0, 0.0, g.z_m - g.thickness_m * 0.5};
            s.rpy = {0.0, 0.0, 0.0};
            collision_monitor_cfg_.extra_collision.push_back(s);
        }
        for (int i = 0; i < kDof && i < static_cast<int>(config_.kinematics.joint_names.size()); ++i) {
            collision_monitor_cfg_.left_joints[i] = m.left_prefix + config_.kinematics.joint_names[i];
            collision_monitor_cfg_.right_joints[i] = m.right_prefix + config_.kinematics.joint_names[i];
        }
        collision_monitor_ = std::make_unique<CollisionMonitor>(collision_monitor_cfg_);
        collision_monitor_->start();
    }
    // Post-condition: the guard is requested but the monitor is absent. make_unique
    // throws on load failure, so this is unreachable defensive code — but never let
    // a server come up with self_collision.enable=true and no monitor behind it.
    if (config_.safety.self_collision.enable && !collision_monitor_) {
        throw std::runtime_error(
            "safety.self_collision.enable=true but the CollisionMonitor failed to construct");
    }
    // Collision-free InitMotion planner. Built from the SAME geometry config as the
    // servo monitor (collision_monitor_cfg_ already carries the ground plane) so it
    // plans against exactly what the runtime barrier enforces. Config validation
    // guarantees self_collision.enable when the planner is enabled. The planner owns
    // its own private (unstarted) CollisionMonitor; it never touches collision_monitor_.
    if (config_.safety.init_motion_planner.enable && config_.safety.self_collision.enable) {
        // Private IK/FK instance (own pinocchio Data) for collision-free TcpLinearMove's
        // off-thread straight-path precheck + detour-goal IK — never the RT kinematics_.
        std::shared_ptr<IKinematics> planner_kin =
            config_.kinematics.enable ? makeKinematicsProvider(config_) : nullptr;
        init_motion_planner_ = std::make_unique<InitMotionPlanner>(
            collision_monitor_cfg_,
            config_.safety.init_motion_planner,
            config_.safety.q_min_deg,
            config_.safety.q_max_deg,
            planner_kin,
            config_.left_mount,
            config_.right_mount,
            config_.safety.joint_target_literal_axes);
    }
    resetReferenceSupervisionState(this);
}

DualArmServoLoop::~DualArmServoLoop() {
    stop();
    eraseReferenceSupervisionState(this);
}

void DualArmServoLoop::plannerWorkerMain() {
    for (;;) {
        PlannerJob job;
        {
            std::unique_lock<std::mutex> lk(planner_mtx_);
            planner_cv_.wait(lk, [&]() {
                return planner_stop_ ||
                       planner_pending_[0].has_value() ||
                       planner_pending_[1].has_value() ||
                       planner_pending_[2].has_value();
            });
            if (planner_stop_) {
                return;
            }
            for (int i = 0; i < 3; ++i) {
                if (planner_pending_[i].has_value()) {
                    job = std::move(*planner_pending_[i]);
                    planner_pending_[i].reset();
                    break;
                }
            }
        }

        PlannerResultSlot res;
        res.generation = job.generation;
        res.valid = true;
        res.is_linear = job.is_linear;
        if (job.is_linear) {
            res.linear_result = init_motion_planner_->planLinearMove(
                job.start_left, job.start_right, job.lin_left_active, job.lin_goal_left,
                job.lin_right_active, job.lin_goal_right, job.slerp, job.lin_samples);
        } else {
            res.init_result = init_motion_planner_->plan(
                job.start_left, job.start_right, job.request_left, job.target_left,
                job.request_right, job.target_right);
        }

        {
            std::lock_guard<std::mutex> lk(planner_mtx_);
            planner_result_[static_cast<int>(job.requester)] = std::move(res);
        }
    }
}

uint64_t DualArmServoLoop::postPlannerJob(PlannerJob job) {
    std::lock_guard<std::mutex> lk(planner_mtx_);
    job.generation = ++planner_job_seq_;
    const uint64_t generation = job.generation;
    planner_pending_[static_cast<int>(job.requester)] = std::move(job);
    planner_cv_.notify_one();
    return generation;
}

bool DualArmServoLoop::takePlannerResult(
    PlannerRequester requester,
    uint64_t generation,
    PlannerResultSlot& out
) {
    std::lock_guard<std::mutex> lk(planner_mtx_);
    PlannerResultSlot& slot = planner_result_[static_cast<int>(requester)];
    if (slot.valid && slot.generation == generation) {
        out = slot;
        slot.valid = false;
        return true;
    }
    return false;
}

bool DualArmServoLoop::start() {
    if (running_) return true;
    if (!initializeRobots()) {
        return false;
    }
    if (init_motion_planner_ && !planner_worker_.joinable()) {
        {
            std::lock_guard<std::mutex> lk(planner_mtx_);
            planner_stop_ = false;
            for (auto& pending : planner_pending_) {
                pending.reset();
            }
            for (PlannerResultSlot& result : planner_result_) {
                result = PlannerResultSlot{};
            }
        }
        planner_worker_ = std::thread(&DualArmServoLoop::plannerWorkerMain, this);
    }
    running_ = true;
    startup_complete_ = false;
    startup_ok_ = false;
    thread_ = std::thread(&DualArmServoLoop::loopMain, this);
    for (int i = 0; i < 100; ++i) {
        if (startup_complete_.load()) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!startup_complete_.load() || !startup_ok_.load()) {
        stop();
        return false;
    }
    return true;
}

void DualArmServoLoop::stop() {
    running_ = false;
    if (thread_.joinable()) {
        thread_.join();
    }
    {
        std::lock_guard<std::mutex> lk(planner_mtx_);
        planner_stop_ = true;
    }
    planner_cv_.notify_all();
    if (planner_worker_.joinable()) {
        planner_worker_.join();
    }
    // The loop thread is joined (no race on the freedrive stage atomics) but the
    // workers/backends are still connected — issue freedrive_teach_off now for any
    // arm left in freedrive so a Ctrl-C/shutdown mid-teaching does not strand the
    // controller in a program-latched freedrive_teach_on state.
    teardownFreedriveOnStop();
    if (left_worker_) left_worker_->stop();
    if (right_worker_) right_worker_->stop();
    if (!workerBackedIoMode()) {
        if (left_robot_) left_robot_->stop();
        if (right_robot_) right_robot_->stop();
    }
}

bool DualArmServoLoop::isRunning() const {
    return running_;
}

ServerMotionState DualArmServoLoop::motionState() const {
    return motion_state_.load();
}

bool DualArmServoLoop::faultLatched() const {
    return fault_latched_.load();
}

SafetyVerdict DualArmServoLoop::latchedFaultReason() const {
    return latched_fault_reason_.load();
}

ServoTarget DualArmServoLoop::previousSentTarget() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    ServoTarget target;
    target.left_q_target_deg = left_prev_sent_q_deg_;
    target.right_q_target_deg = right_prev_sent_q_deg_;
    return target;
}

ServoSnapshot DualArmServoLoop::latestSnapshot() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return latest_snapshot_;
}

bool DualArmServoLoop::initializeRobots() {
    if (workerBackedIoMode()) {
        return initializeWorkers();
    }

    const BackendResult<RobotState> left_connect = left_robot_->connect();
    const BackendResult<RobotState> right_connect = right_robot_->connect();
    if (!left_connect.ok || !right_connect.ok) {
        std::cerr << "[ERROR] failed to connect robots"
                  << " left=" << left_connect.error.name << ":" << left_connect.error.message
                  << " right=" << right_connect.error.name << ":" << right_connect.error.message << "\n";
        return false;
    }
    const BackendResult<RobotState> left_init = left_robot_->initialize();
    const BackendResult<RobotState> right_init = right_robot_->initialize();
    if (!left_init.ok || !right_init.ok) {
        std::cerr << "[ERROR] failed to initialize robots"
                  << " left=" << left_init.error.name << ":" << left_init.error.message
                  << " right=" << right_init.error.name << ":" << right_init.error.message << "\n";
        return false;
    }

    RobotState left, right;
    const bool states_read = readRobotStates(left, right);
    const StartupValidationSnapshot startup_validation = validateStartupStates(left, right);
    logStartupValidation(startup_validation, left, right);
    if (!states_read || !startupValidationAllowsStart(startup_validation)) {
        return false;
    }
    if (!initializeStartupTargets(left, right)) {
        return false;
    }
    storeStartupValidation(startup_validation);
    setMotionState(ServerMotionState::ConnectedHold);
    if (readOnlyMode()) {
        std::cerr << "[INFO] servo send policy: read_only; backend sendServoJ calls are suppressed\n";
    }
    return true;
}

bool DualArmServoLoop::initializeWorkers() {
    if (!left_worker_) {
        if (!left_robot_) {
            std::cerr << "[ERROR] worker io requested but left backend is unavailable\n";
            return false;
        }
        left_worker_ = std::make_unique<ArmWorker>(std::move(left_robot_), workerOptions(config_));
    }
    if (!right_worker_) {
        if (!right_robot_) {
            std::cerr << "[ERROR] worker io requested but right backend is unavailable\n";
            return false;
        }
        right_worker_ = std::make_unique<ArmWorker>(std::move(right_robot_), workerOptions(config_));
    }

    const bool left_started = left_worker_->start();
    const bool right_started = right_worker_->start();
    if (!left_started || !right_started) {
        std::cerr << "[ERROR] failed to start arm workers"
                  << " left_started=" << left_started
                  << " right_started=" << right_started << "\n";
        if (left_worker_) left_worker_->stop();
        if (right_worker_) right_worker_->stop();
        return false;
    }

    const uint64_t startup_timeout_ns = workerStartupTimeoutNs(config_);
    const uint64_t deadline_ns = nowSteadyNs() + startup_timeout_ns;
    RobotState left;
    RobotState right;
    while (nowSteadyNs() < deadline_ns) {
        if (readRobotStates(left, right)) {
            const StartupValidationSnapshot startup_validation = validateStartupStates(left, right);
            if (!startupValidationAllowsStart(startup_validation)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            logStartupValidation(startup_validation, left, right);
            if (!initializeStartupTargets(left, right)) {
                left_worker_->stop();
                right_worker_->stop();
                return false;
            }
            storeStartupValidation(startup_validation);
            setMotionState(ServerMotionState::ConnectedHold);
            if (readOnlyMode()) {
                std::cerr << "[INFO] servo send policy: read_only; worker sendServoJ requests are suppressed\n";
            }
            if (rbpodoAsyncIoMode()) {
                std::cerr << "[INFO] servo io_model: rbpodo_async; ServoLoop reads cached "
                          << "ArmWorker state and enqueues non-blocking rbpodo sends\n";
            } else {
                std::cerr << "[INFO] servo io_model: worker; ServoLoop reads cached "
                          << "ArmWorker state and enqueues sends\n";
            }
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    const BackendResult<RobotState> left_state = left_worker_->latestState(startup_timeout_ns);
    const BackendResult<RobotState> right_state = right_worker_->latestState(startup_timeout_ns);
    std::cerr << "[ERROR] invalid worker startup state"
              << " left=" << left_state.error.name << ":" << left_state.error.message
              << " right=" << right_state.error.name << ":" << right_state.error.message
              << " timeout_sec=" << static_cast<double>(startup_timeout_ns) / 1'000'000'000.0
              << " left_worker=" << workerStartupSummary(left_worker_.get())
              << " right_worker=" << workerStartupSummary(right_worker_.get()) << "\n";
    left_worker_->stop();
    right_worker_->stop();
    return false;
}

void DualArmServoLoop::checkExternalBoxFeedOrAbort(uint64_t now_ns) {
    // Only when external-box keep-out is ENFORCED. When monitor_only (the default) or
    // disabled, a missing feed is expected/harmless, so this is a no-op.
    if (!collision_monitor_ || !collision_monitor_cfg_.external_boxes.enable ||
        collision_monitor_cfg_.external_boxes.monitor_only) {
        return;
    }
    if (external_box_enforce_start_ns_ == 0) external_box_enforce_start_ns_ = now_ns;
    // Liveness is measured at RECEIVE time (network thread stamps every accepted
    // SetExternalBoxes packet on the CommandBuffer), NOT at apply time. The payload
    // is consumed from an external-box side slot so it cannot displace the high-rate
    // motion stream; the receive stamp still catches a genuinely dead/stopped producer.
    // Startup grace lets the producer come up; after the first feed, a gap beyond
    // the timeout means it stopped. Decision is in the pure (tested) helper.
    constexpr double kStartupGraceS = 10.0;
    constexpr double kFeedTimeoutS = 3.0;
    const uint64_t last_receive_ns =
        command_buffer_ ? command_buffer_->lastExternalBoxReceiveNs() : 0;
    const bool feed_seen = last_receive_ns != 0;
    const double since_last_feed_s =
        feed_seen ? nsToSec(now_ns - last_receive_ns) : 0.0;
    const char* reason = externalBoxFeedAbortReason(
        feed_seen, nsToSec(now_ns - external_box_enforce_start_ns_),
        since_last_feed_s, kStartupGraceS, kFeedTimeoutS);
    if (!reason) return;
    std::cerr << "\n[FATAL][collision_monitor] external-box keep-out is ENFORCED"
                 " (external_boxes.enable=true, monitor_only=false) but " << reason << ".\n"
                 "Refusing to run on without keep-out (fail-closed). Resolve one of:\n"
                 "  - start the box producer so SetExternalBoxes is sent to the command port"
                 " (camera/stereo with STEREO_SEND_EXTERNAL_BOXES=1), or\n"
                 "  - set safety.self_collision.mesh.external_boxes.monitor_only: true"
                 " (report-only), or\n"
                 "  - set external_boxes.enable: false to disable the feature.\n"
                 "Aborting." << std::endl;
    std::abort();
}

void DualArmServoLoop::loopMain() {
    if (!configureRealtimeForLoop()) {
        startup_ok_ = false;
        startup_complete_ = true;
        running_ = false;
        return;
    }
    startup_ok_ = true;
    startup_complete_ = true;

    const int rate_hz = config_.servo.rate_hz > 0 ? config_.servo.rate_hz : 200;
    const auto period = std::chrono::nanoseconds(static_cast<long long>(1'000'000'000LL / rate_hz));
    auto next_tick = std::chrono::steady_clock::now();
    last_loop_start_ns_ = 0;
    const bool async_supervision_nonlatching =
        controllerSimulationMotionRequired(config_) &&
        controllerSimulationMotionGateOpen(config_) &&
        config_.servo.controller_simulation_async_supervision_nonlatching;
    bool async_supervision_degraded_last_tick = false;
    uint64_t last_async_supervision_degraded_warn_ns = 0;
    std::string last_async_supervision_degraded_reason;
    constexpr uint64_t kAsyncSupervisionDegradedWarnPeriodNs = 5'000'000'000ULL;

    // Jitter instrumentation: when the previous tick entered sleep_until (0 on first tick).
    uint64_t prev_sleep_enter_ns = 0;
    // servo.send_at_tick_start staging: tick N computes and safety-filters a
    // target, tick N+1 dispatches it immediately after wake-up (wire timing
    // then depends only on wake-up latency). Invalid until the first compute.
    struct PendingTopSend {
        ServoTarget target{};
        uint64_t command_seq = 0;
        uint64_t command_host_time_ns = 0;
        uint64_t deadline_ns = 0;
        bool valid = false;
    };
    PendingTopSend pending_top_send;
    const bool send_at_top = config_.servo.send_at_tick_start;
    // In pgmode the safety filter advances a purely commanded/reference robot;
    // it must therefore see the target that was actually dispatched at the top
    // of this tick. Leaving bookkeeping until the end of the tick makes the
    // acceleration limiter operate on a two-send-old history and forms an
    // unstable alternating recurrence. Keep the physical-real timing unchanged
    // until that lane has its own supervised acceptance evidence.
    const bool controller_sim_top_send_bookkeeping =
        send_at_top && controllerSimulationMotionRequired(config_) &&
        controllerSimulationMotionGateOpen(config_);

    // Hybrid sleep-then-spin: sleep_until(next_tick - slack), then busy-spin the
    // final `slack` so the tick lands on phase without the C-state exit +
    // hrtimer/scheduler wake-path jitter. Guarded (fail-closed to plain
    // sleep_until): a ~100% RT spin under RT throttling would be descheduled
    // ~50 ms/s, so spin is only enabled with RT priority AND throttling off.
    long long spin_slack_ns = 0;
    if (config_.servo.spin_slack_us > 0) {
        const long long requested_ns = static_cast<long long>(config_.servo.spin_slack_us) * 1000LL;
        if (!config_.servo.enable_realtime_priority) {
            std::cerr << "[WARN] servo.spin_slack_us=" << config_.servo.spin_slack_us
                      << " ignored: needs enable_realtime_priority=true (a non-RT spin just "
                         "burns a core and is preempted anyway). Using plain sleep_until.\n";
        } else {
            const long long rt_runtime_us = readSchedRtRuntimeUs();
            if (rt_runtime_us != -1) {
                std::cerr << "[WARN] servo.spin_slack_us=" << config_.servo.spin_slack_us
                          << " ignored: kernel.sched_rt_runtime_us=" << rt_runtime_us
                          << " (RT throttling ON) would throttle a busy spin ~50 ms/s. "
                             "Run `sudo sysctl kernel.sched_rt_runtime_us=-1` to enable. "
                             "Using plain sleep_until.\n";
            } else {
                spin_slack_ns = requested_ns;
                std::cerr << "[INFO] servo hybrid sleep-then-spin enabled: slack="
                          << config_.servo.spin_slack_us << " us (RT throttling off).\n";
            }
        }
    }

    while (running_) {
        // Scheduled wake time of THIS tick = the sleep_until target of the previous
        // iteration (next_tick before the increment below). Same steady epoch as
        // nowSteadyNs().
        const uint64_t sched_wake_ns = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                next_tick.time_since_epoch()).count());
        next_tick += period;
        const uint64_t loop_start = nowSteadyNs();
        // Dispatch/result variables shared by both send modes. In
        // send_at_tick_start mode they are filled HERE (dispatching the target
        // staged by the previous tick); in legacy mode they are filled at the
        // post-compute send site below. `sent_target` is the target actually
        // dispatched THIS tick (staged one in top mode, attempted one in
        // legacy) and is what send bookkeeping must record.
        DualSendResult dual_send_result;
        ServoTarget sent_target{};
        bool fault_latched_before_send = false;
        std::string send_policy = "send_servo_j";
        bool send_suppressed = true;
        if (send_at_top) {
            // Re-evaluate the policy at dispatch time: a fault latched (or a
            // freedrive/lease change) since the target was computed must still
            // suppress it. sendTargets() itself suppresses on any policy other
            // than "send_servo_j" without touching the sockets.
            fault_latched_before_send = fault_latched_.load();
            send_policy = currentSendPolicy();
            if (!pending_top_send.valid && send_policy == "send_servo_j") {
                // First tick(s) after start: nothing staged yet. Telemetry
                // shows this as its own suppressed policy.
                send_policy = "no_pending_target";
            }
            send_suppressed = send_policy != "send_servo_j";
            sent_target = pending_top_send.target;
            dual_send_result = sendTargets(
                pending_top_send.target,
                pending_top_send.command_seq,
                pending_top_send.command_host_time_ns,
                send_policy,
                loop_start,
                pending_top_send.deadline_ns
            );
            if (controller_sim_top_send_bookkeeping) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                if (dual_send_result.left.result.accepted &&
                    !fault_latched_before_send && !send_suppressed) {
                    left_prevprev_sent_q_deg_ = left_prev_sent_q_deg_;
                    left_prev_sent_q_deg_ = sent_target.left_q_target_deg;
                }
                if (dual_send_result.right.result.accepted &&
                    !fault_latched_before_send && !send_suppressed) {
                    right_prevprev_sent_q_deg_ = right_prev_sent_q_deg_;
                    right_prev_sent_q_deg_ = sent_target.right_q_target_deg;
                }
            }
            pending_top_send.valid = false;
        }
        // Fail-closed: if external-box keep-out is enforced but the producer feed is
        // missing/stale, abort instead of silently running without keep-out (no-op when
        // monitor_only/disabled — the default config).
        checkExternalBoxFeedOrAbort(loop_start);
        bool async_supervision_degraded_this_tick = false;
        bool async_supervision_degraded_warned_this_tick = false;
        tracking_error_degraded_this_tick_ = false;
        left_abc_telemetry_.safety_clamp_present = false;
        right_abc_telemetry_.safety_clamp_present = false;
        const auto handleAsyncSupervisionFault =
            [&](const LatchedDualFaultContext& async_fault_contexts,
                const RobotState& current_left_state,
                const RobotState& current_right_state) {
                if (!async_fault_contexts.top_level.has_value()) {
                    return false;
                }
                if (async_supervision_nonlatching) {
                    async_supervision_degraded_this_tick = true;
                    const std::string reason =
                        asyncSupervisionFaultReason(async_fault_contexts);
                    const bool state_changed =
                        !async_supervision_degraded_last_tick ||
                        reason != last_async_supervision_degraded_reason;
                    const bool warn_period_elapsed =
                        last_async_supervision_degraded_warn_ns == 0 ||
                        loop_start - last_async_supervision_degraded_warn_ns >=
                            kAsyncSupervisionDegradedWarnPeriodNs;
                    if (!async_supervision_degraded_warned_this_tick &&
                        (state_changed || warn_period_elapsed)) {
                        std::cerr
                            << "[WARN] controller-sim async supervision degraded "
                            << "(suppressed, not latched): " << reason << "\n";
                        last_async_supervision_degraded_warn_ns = loop_start;
                        last_async_supervision_degraded_reason = reason;
                        async_supervision_degraded_warned_this_tick = true;
                    }
                    return false;
                }
                latchFault(
                    SafetyVerdict::SendFailure,
                    "rbpodo async streaming supervision fault",
                    current_left_state,
                    current_right_state,
                    async_fault_contexts
                );
                return true;
            };
        const uint64_t nominal_period_ns = static_cast<uint64_t>(period.count());
        const uint64_t actual_period_ns = last_loop_start_ns_ == 0
            ? nominal_period_ns
            : loop_start - last_loop_start_ns_;
        const double filter_dt_sec = computeFilterDtSec(actual_period_ns, nominal_period_ns);
        last_loop_start_ns_ = loop_start;

        const double state_age_periods = config_.servo.worker_state_max_age_periods > 0.0
            ? config_.servo.worker_state_max_age_periods
            : 2.0;
        uint64_t worker_state_max_age_ns = std::max<uint64_t>(
            static_cast<uint64_t>(state_age_periods * static_cast<double>(nominal_period_ns)),
            1'000'000);
        if (rbpodoAsyncIoMode()) {
            const double async_state_age_ns =
                config_.servo.rbpodo_async_streaming.ack_supervision.expected_ack_timeout_ms *
                1'000'000.0;
            if (std::isfinite(async_state_age_ns) && async_state_age_ns > 0.0) {
                worker_state_max_age_ns = std::max<uint64_t>(
                    worker_state_max_age_ns,
                    static_cast<uint64_t>(async_state_age_ns)
                );
            }
        }
        BackendResult<RobotState> left_state_result = workerBackedIoMode()
            ? (left_worker_
                ? left_worker_->latestState(worker_state_max_age_ns)
                : failedReadState(backendError(BackendErrorKind::RobotDisconnected, "left worker unavailable")))
            : (left_robot_
                ? left_robot_->readState()
                : failedReadState(backendError(BackendErrorKind::RobotDisconnected, "left backend unavailable")));
        BackendResult<RobotState> right_state_result = workerBackedIoMode()
            ? (right_worker_
                ? right_worker_->latestState(worker_state_max_age_ns)
                : failedReadState(backendError(BackendErrorKind::RobotDisconnected, "right worker unavailable")))
            : (right_robot_
                ? right_robot_->readState()
                : failedReadState(backendError(BackendErrorKind::RobotDisconnected, "right backend unavailable")));

        RobotState left_state = left_state_result.value;
        RobotState right_state = right_state_result.value;
        left_state.arm_id = ArmId::Left;
        right_state.arm_id = ArmId::Right;
        populateTcpPose(left_state, config_.left_mount);
        populateTcpPose(right_state, config_.right_mount);

        // THE F/T PIPELINE RUNS ONCE PER TICK, HERE - after the state that carries the
        // raw reading and the FK that the compensation needs, and BEFORE anything
        // reads a wrench. Running it per consumer would let two consumers of one tick
        // see two different wrenches, which is the class of frame slip that makes a
        // force number impossible to argue about.
        stepFtPipeline(ArmId::Left, left_state);
        stepFtPipeline(ArmId::Right, right_state);

        FaultContext left_read_fault = classifyReadStateResult(left_state_result, ArmId::Left);
        FaultContext right_read_fault = classifyReadStateResult(right_state_result, ArmId::Right);
        left_read_fault = clearControllerSimulationDiagnosticReadFault(
            config_,
            left_state,
            left_read_fault,
            ArmId::Left
        );
        right_read_fault = clearControllerSimulationDiagnosticReadFault(
            config_,
            right_state,
            right_read_fault,
            ArmId::Right
        );
        FaultContext read_fault = left_read_fault.verdict != SafetyVerdict::Ok
            ? left_read_fault
            : right_read_fault;
        LatchedDualFaultContext read_fault_contexts =
            dualReadFaultContext(left_read_fault, right_read_fault);
        bool state_ok = read_fault.verdict == SafetyVerdict::Ok;
        RbpodoAsyncStreamingTelemetry left_async_telemetry = workerBackedIoMode()
            ? asyncTelemetryOrDefault(left_worker_.get())
            : RbpodoAsyncStreamingTelemetry{};
        RbpodoAsyncStreamingTelemetry right_async_telemetry = workerBackedIoMode()
            ? asyncTelemetryOrDefault(right_worker_.get())
            : RbpodoAsyncStreamingTelemetry{};
        updateReferenceSupervision(
            this,
            config_,
            ArmId::Left,
            left_state,
            loop_start,
            &left_async_telemetry
        );
        updateReferenceSupervision(
            this,
            config_,
            ArmId::Right,
            right_state,
            loop_start,
            &right_async_telemetry
        );
        if (rbpodoAsyncIoMode() && !fault_latched_.load()) {
            const LatchedDualFaultContext async_fault_contexts =
                asyncSupervisionFaultContexts(left_async_telemetry, right_async_telemetry);
            (void)handleAsyncSupervisionFault(async_fault_contexts, left_state, right_state);
        }


        CommandBufferReadTelemetry command_buffer_read;
        DualArmCommand command = command_buffer_
            ? command_buffer_->latestOrHold(loop_start, &command_buffer_read)
            : makeHoldCommand(left_state, right_state, loop_start);
        if (!command_buffer_) {
            command_buffer_read.result = "no_command_buffer_hold";
            command_buffer_read.returned_seq = command.seq;
            command_buffer_read.returned_left_mode = command.left.mode;
            command_buffer_read.returned_right_mode = command.right.mode;
            command_buffer_read.returned_host_time_ns = command.host_time_ns;
            command_buffer_read.returned_age_ms = 0.0;
        }
        const auto metadata_hold = [&](const DualArmCommand& source_command) {
            DualArmCommand hold = makeHoldCommand(left_state, right_state, loop_start);
            hold.source = source_command.source;
            hold.lease = source_command.lease;
            // Preserve any gripper setpoint riding the command. A button-only
            // gripper press (SpaceMouse cap neutral) arrives as a Hold/Hold with
            // gripper_target, which this hold-rewrite path (e.g. the explicit
            // dual-hold branch) would otherwise drop because makeHoldCommand only
            // sets joint hold targets — so the gripper would never actuate.
            hold.left.has_gripper = source_command.left.has_gripper;
            hold.left.gripper_target = source_command.left.gripper_target;
            hold.right.has_gripper = source_command.right.has_gripper;
            hold.right.gripper_target = source_command.right.gripper_target;
            return hold;
        };
        const auto is_plain_hold_arm = [](const ArmCommand& arm) {
            return arm.mode == ControlMode::Hold &&
                arm.joint_target_profile == JointTargetProfile::Direct &&
                !arm.has_joint_target && !arm.has_arrival_stop &&
                !arm.has_tcp_target && !arm.has_linear_move_duration &&
                !arm.has_linear_move_linear_speed &&
                !arm.has_linear_move_angular_speed &&
                !arm.has_linear_move_orientation_mode &&
                !arm.has_gripper && !arm.has_freedrive;
        };
        if (isSyntheticHoldCommand(command)) {
            command = metadata_hold(command);
        }
        if (command_buffer_) {
            std::optional<DualArmCommand> external_boxes_command =
                command_buffer_->consumeLatestExternalBoxes(loop_start, &command_buffer_read);
            if (external_boxes_command.has_value()) {
                command_buffer_read.external_boxes_applied =
                    applySetExternalBoxesCommand(*external_boxes_command);
            }
        }
        const bool read_only_command_blocked = readOnlyMode() && commandBlockedByReadOnly(command);

        if (read_only_command_blocked) {
            setMotionState(ServerMotionState::ConnectedHold);
        } else if (commandRequestsEmergencyStop(command)) {
            clearLatchedCartesianTargets();
            const FaultContext emergency_context = classifyCommandValidation(
                SafetyVerdict::EmergencyStop,
                command.left.mode == ControlMode::EmergencyStop ? ArmId::Left : ArmId::Right,
                "EmergencyStop command"
            );
            latchFault(
                SafetyVerdict::EmergencyStop,
                "EmergencyStop command",
                left_state,
                right_state,
                emergency_context
            );
            command = metadata_hold(command);
        } else if (commandRequestsResetFault(command)) {
            clearLatchedCartesianTargets();
            if (readOnlyMode()) {
                command = metadata_hold(command);
            } else if (fault_latched_.load()) {
                if (clearFaultLatch(left_state, right_state)) {
                    const BackendTiming reset_read_timing = makeBackendTiming(loop_start, nowSteadyNs());
                    left_state_result = okReadState(left_state, reset_read_timing);
                    right_state_result = okReadState(right_state, reset_read_timing);
                    left_read_fault = classifyReadStateResult(left_state_result, ArmId::Left);
                    right_read_fault = classifyReadStateResult(right_state_result, ArmId::Right);
                    left_read_fault = clearControllerSimulationDiagnosticReadFault(
                        config_,
                        left_state,
                        left_read_fault,
                        ArmId::Left
                    );
                    right_read_fault = clearControllerSimulationDiagnosticReadFault(
                        config_,
                        right_state,
                        right_read_fault,
                        ArmId::Right
                    );
                    read_fault = left_read_fault.verdict != SafetyVerdict::Ok
                        ? left_read_fault
                        : right_read_fault;
                    read_fault_contexts = dualReadFaultContext(left_read_fault, right_read_fault);
                    state_ok = read_fault.verdict == SafetyVerdict::Ok;
                }
            } else {
                setMotionState(ServerMotionState::ConnectedHold);
            }
            command = metadata_hold(command);
        } else if (command.left.mode == ControlMode::TareForceSensor ||
                   command.right.mode == ControlMode::TareForceSensor) {
            // Leaseless, non-motion: deposit the request and hold. The averaging runs
            // on the RT loop (stepFtPipeline) over kFtTareSamples consecutive ticks,
            // because a bias averaged off the RT path would be a bias measured against
            // samples nobody knows the timing of.
            //
            // THE ARM MUST BE STILL AND UNLOADED. Nothing here can check that — a tare
            // records whatever is hanging on the tool at that moment as "zero" — so the
            // refusal that matters is the operator's, and the log line says so.
            const bool want_left = command.tare_left || (!command.tare_left && !command.tare_right);
            const bool want_right = command.tare_right || (!command.tare_left && !command.tare_right);
            if (want_left) requestFtTare(ArmId::Left);
            if (want_right) requestFtTare(ArmId::Right);
            std::cerr << "[INFO] TareForceSensor requested ("
                      << (want_left ? "left " : "") << (want_right ? "right" : "")
                      << ") - averaging " << kFtTareSamples
                      << " ticks. The arm must be STILL and carrying nothing but the tool: "
                         "whatever load stands now becomes the new zero\n";
            command = metadata_hold(command);
        } else if (commandRequestsSetSafetyFloorZ(command)) {
            // Leaseless runtime adjustment of the floor plane height. Accepted only
            // within the configured [runtime_min_z_m, runtime_max_z_m] envelope;
            // works while fault-latched (raising the floor must never be blocked).
            if (!command.has_floor_z) {
                floor_last_set_reject_reason_ = "floor_z_missing";
                std::cerr << "[WARN] SetSafetyFloorZ rejected: missing floor_z_m payload\n";
            } else {
                const std::optional<std::string> reject =
                    validateFloorZRequest(command.floor_z_m, config_.safety.floor_constraint);
                if (reject.has_value()) {
                    floor_last_set_reject_reason_ = *reject;
                    std::cerr << "[WARN] SetSafetyFloorZ rejected (" << *reject << "): "
                              << command.floor_z_m << " m from source_id="
                              << command.source.source_id << "\n";
                } else {
                    runtime_floor_z_m_.store(command.floor_z_m);
                    floor_last_set_reject_reason_.clear();
                    std::cerr << "[INFO] safety floor plane set to " << command.floor_z_m
                              << " m by source_id=" << command.source.source_id << "\n";
                }
            }
            command = metadata_hold(command);
        } else if (commandRequestsSetSafetyFloorEnabled(command)) {
            // Leaseless runtime enforce on/off for the stand floor. config.enable is a
            // hard master (opt-in + startup kinematics validation): a runtime enable is
            // rejected when the floor is not opted in at config. Works while fault-latched.
            if (!command.has_floor_enabled) {
                floor_last_set_reject_reason_ = "floor_enabled_missing";
            } else if (command.floor_enabled && !config_.safety.floor_constraint.enable) {
                floor_last_set_reject_reason_ = "floor_constraint_disabled_in_config";
                std::cerr << "[WARN] SetSafetyFloorEnabled(true) rejected: "
                             "floor_constraint.enable=false in config\n";
            } else {
                runtime_floor_enabled_.store(command.floor_enabled);
                floor_last_set_reject_reason_.clear();
                std::cerr << "[INFO] safety floor enforcement "
                          << (command.floor_enabled ? "ENABLED" : "DISABLED")
                          << " by source_id=" << command.source.source_id << "\n";
            }
            command = metadata_hold(command);
        } else if (commandRequestsSetSafetyRoiBounds(command)) {
            // Leaseless runtime adjustment of the stand-frame ROI box bounds.
            // Accepted only within the configured per-axis runtime envelope; works
            // while fault-latched (shrinking/raising the box must never be blocked).
            if (!command.has_roi_bounds) {
                roi_last_set_reject_reason_ = "roi_bounds_missing";
                std::cerr << "[WARN] SetSafetyRoiBounds rejected: missing roi_min_m/roi_max_m payload\n";
            } else {
                const std::optional<std::string> reject = validateRoiBoundsRequest(
                    command.roi_min_m, command.roi_max_m, config_.safety.roi_box);
                if (reject.has_value()) {
                    roi_last_set_reject_reason_ = *reject;
                    std::cerr << "[WARN] SetSafetyRoiBounds rejected (" << *reject
                              << ") from source_id=" << command.source.source_id << "\n";
                } else {
                    for (int k = 0; k < 3; ++k) {
                        runtime_roi_min_m_[k].store(command.roi_min_m[k]);
                        runtime_roi_max_m_[k].store(command.roi_max_m[k]);
                    }
                    roi_last_set_reject_reason_.clear();
                    std::cerr << "[INFO] safety ROI box set to min["
                              << command.roi_min_m[0] << "," << command.roi_min_m[1] << ","
                              << command.roi_min_m[2] << "] max[" << command.roi_max_m[0] << ","
                              << command.roi_max_m[1] << "," << command.roi_max_m[2]
                              << "] m by source_id=" << command.source.source_id << "\n";
                }
            }
            command = metadata_hold(command);
        } else if (commandRequestsSetExternalBoxes(command)) {
            (void)applySetExternalBoxesCommand(command);
            command = metadata_hold(command);
        } else if (commandRequestsSetUserSafetyFloorPlane(command)) {
            // Leaseless runtime set/enable of the user-defined tilted floor plane.
            // A disable request (enable=false) is accepted unconditionally; an enable
            // request must pass validateUserFloorPlaneRequest. Works while fault-latched
            // (turning the constraint on/off must never be blocked).
            if (!command.has_user_floor_plane) {
                user_floor_last_set_reject_reason_ = "user_floor_plane_missing";
                std::cerr << "[WARN] SetUserSafetyFloorPlane rejected: missing payload\n";
            } else if (!command.user_floor_enable) {
                runtime_user_floor_enabled_.store(false);
                user_floor_last_set_reject_reason_.clear();
                std::cerr << "[INFO] user safety floor plane DISABLED by source_id="
                          << command.source.source_id << "\n";
            } else {
                const std::optional<std::string> reject = validateUserFloorPlaneRequest(
                    command.user_floor_point_m, command.user_floor_normal,
                    command.user_floor_margin_m, config_.safety.user_floor_constraint);
                if (reject.has_value()) {
                    user_floor_last_set_reject_reason_ = *reject;
                    std::cerr << "[WARN] SetUserSafetyFloorPlane rejected (" << *reject
                              << ") from source_id=" << command.source.source_id << "\n";
                } else {
                    for (int k = 0; k < 3; ++k) {
                        runtime_user_floor_point_m_[k].store(command.user_floor_point_m[k]);
                        runtime_user_floor_normal_[k].store(command.user_floor_normal[k]);
                    }
                    runtime_user_floor_margin_m_.store(command.user_floor_margin_m);
                    runtime_user_floor_enabled_.store(true);
                    user_floor_last_set_reject_reason_.clear();
                    std::cerr << "[INFO] user safety floor plane set to point["
                              << command.user_floor_point_m[0] << "," << command.user_floor_point_m[1]
                              << "," << command.user_floor_point_m[2] << "] normal["
                              << command.user_floor_normal[0] << "," << command.user_floor_normal[1]
                              << "," << command.user_floor_normal[2] << "] margin="
                              << command.user_floor_margin_m << " m by source_id="
                              << command.source.source_id << "\n";
                }
            }
            command = metadata_hold(command);
        } else if (commandRequestsDisarmMotion(command)) {
            clearLatchedCartesianTargets();
            setMotionState(ServerMotionState::ConnectedHold);
            command = metadata_hold(command);
        } else if (isExplicitDualHoldCommand(command)) {
            if (!fault_latched_.load() &&
                motion_state_.load() == ServerMotionState::Running) {
                setMotionState(ServerMotionState::ArmedHold);
            }
            DualArmCommand hold = metadata_hold(command);
            hold.seq = command.seq;
            hold.host_time_ns = command.host_time_ns;
            hold.left.timeout_sec = command.left.timeout_sec;
            hold.right.timeout_sec = command.right.timeout_sec;
            command = hold;
        } else if (commandRequestsArmMotion(command)) {
            if (!fault_latched_.load()) {
                setMotionState(ServerMotionState::ArmedHold);
            }
            command = metadata_hold(command);
        } else if (commandRequestsFreedrive(command)) {
            // Per-arm direct teaching. Toggling is leased (operator must hold
            // control). This only records the requested stage transition; the
            // arming state machine (advanceFreedrive, below) quiesces the servo
            // stream and engages/exits free-drive over subsequent ticks.
            requestFreedrive(command, left_state, right_state);
            command = metadata_hold(command);
        } else if (commandRequestsMotion(command) && !motionAllowed()) {
            clearLatchedCartesianTargets();
            command = metadata_hold(command);
        }

        // Advance the per-arm free-drive arming state machine every tick. Once an
        // arm leaves Off, anyFreedriveActive() suppresses servo_j to both
        // controllers (currentSendPolicy()=="freedrive") so the controller settles
        // to idle before freedrive_teach_on is issued — the M151 ("Cannot run this
        // function") fix: real-time servo control must stop before direct teaching.
        advanceFreedrive(left_state, right_state);

        ServoTarget safe_target;
        SafetyVerdict safety_verdict = SafetyVerdict::Ok;
        std::string init_mode_before_left = toString(command.left.mode);
        std::string init_mode_before_right = toString(command.right.mode);
        std::string init_mode_after_left = init_mode_before_left;
        std::string init_mode_after_right = init_mode_before_right;
        std::string init_profile_before_left = toString(command.left.joint_target_profile);
        std::string init_profile_before_right = toString(command.right.joint_target_profile);
        std::string init_profile_after_left = init_profile_before_left;
        std::string init_profile_after_right = init_profile_before_right;
        std::string init_non_init_arm_preserved_mode;
        if (!fault_latched_.load()) {
            left_safety_tracking_ = SafetyTrackingTelemetry{};
            right_safety_tracking_ = SafetyTrackingTelemetry{};
        }

        if (!state_ok || !isValidJointState(left_state) || !isValidJointState(right_state)) {
            safety_verdict = state_ok ? SafetyVerdict::RobotStateError : read_fault.verdict;
            if (isRealMode() || config_.safety.latch_fault_on_robot_state_error) {
                const std::string reason = state_ok || read_fault.reason.empty()
                    ? "robot state read failed or invalid"
                    : read_fault.reason;
                const std::optional<FaultContext> context =
                    read_fault.verdict != SafetyVerdict::Ok
                        ? std::optional<FaultContext>(read_fault)
                        : std::nullopt;
                if (read_fault_contexts.top_level.has_value()) {
                    latchFault(safety_verdict, reason, left_state, right_state, read_fault_contexts);
                } else {
                    latchFault(safety_verdict, reason, left_state, right_state, context);
                }
                safe_target = currentFaultHoldTarget();
                safety_verdict = SafetyVerdict::FaultLatched;
            } else {
                safe_target.left_q_target_deg = left_prev_sent_q_deg_;
                safe_target.right_q_target_deg = right_prev_sent_q_deg_;
            }
        } else if (fault_latched_.load()) {
            clearLatchedCartesianTargets();
            safe_target = currentFaultHoldTarget();
            safety_verdict = SafetyVerdict::FaultLatched;
        } else if (read_only_command_blocked) {
            safe_target.left_q_target_deg = left_prev_sent_q_deg_;
            safe_target.right_q_target_deg = right_prev_sent_q_deg_;
            safety_verdict = SafetyVerdict::InvalidCommand;
        } else if (anyFreedriveActive()) {
            // Direct teaching: one or both controllers are hand-guided. Bypass the
            // motion pipeline entirely so the hand-driven actual divergence cannot
            // latch a tracking error or trip the velocity clamp. Sends are
            // suppressed by send_policy=="freedrive"; the held target is bookkeeping
            // only and is resynced to actual on exit.
            safe_target.left_q_target_deg = left_prev_sent_q_deg_;
            safe_target.right_q_target_deg = right_prev_sent_q_deg_;
            safety_verdict = SafetyVerdict::Ok;
        } else {
            SafetyVerdict command_verdict = SafetyVerdict::Ok;
            // Collision-free TcpLinearMove: decide Straight (pass through to the exact
            // MoveL) vs Detour (rewrite to a streamed JointTarget collision-free path).
            command = applyCollisionFreeLinearMove(command, left_state, right_state);
            const ControlMode left_mode_before_init_sequencer = command.left.mode;
            const ControlMode right_mode_before_init_sequencer = command.right.mode;
            const JointTargetProfile left_profile_before_init_sequencer =
                command.left.joint_target_profile;
            const JointTargetProfile right_profile_before_init_sequencer =
                command.right.joint_target_profile;
            const bool left_init_before_sequencer =
                command.left.mode == ControlMode::JointTarget &&
                command.left.joint_target_profile == JointTargetProfile::InitMotion;
            const bool right_init_before_sequencer =
                command.right.mode == ControlMode::JointTarget &&
                command.right.joint_target_profile == JointTargetProfile::InitMotion;
            // Collision-free JointTarget init_motion profile: rewrite to direct
            // JointTarget waypoints (or hold while planning / on failure), so the
            // trajectory + safety pipeline below is unchanged.
            command = applyInitMotionSequencer(command, left_state, right_state);
            init_mode_before_left = toString(left_mode_before_init_sequencer);
            init_mode_before_right = toString(right_mode_before_init_sequencer);
            init_mode_after_left = toString(command.left.mode);
            init_mode_after_right = toString(command.right.mode);
            init_profile_before_left = toString(left_profile_before_init_sequencer);
            init_profile_before_right = toString(right_profile_before_init_sequencer);
            init_profile_after_left = toString(command.left.joint_target_profile);
            init_profile_after_right = toString(command.right.joint_target_profile);
            init_non_init_arm_preserved_mode.clear();
            if (left_init_before_sequencer != right_init_before_sequencer &&
                !config_.safety.init_motion_planner.single_arm_freeze_other_arm) {
                init_non_init_arm_preserved_mode =
                    left_init_before_sequencer ? toString(command.right.mode) : toString(command.left.mode);
            }
            const bool motion_requested = commandRequestsMotion(command);
            const ServoTarget desired =
                computeServoTarget(left_state, right_state, command, filter_dt_sec, &command_verdict);
            // computeServoTarget already encodes the correct PER-ARM result in `desired`:
            // an arm whose Cartesian/IK solve failed is held at its prev_sent target, while
            // a healthy arm keeps its freshly computed target (and a whole-command failure
            // — missing payload / Cartesian unavailable — holds BOTH at prev_sent). So we
            // run the safety stage on `desired` UNCONDITIONALLY. A single arm's IK/Cartesian
            // failure must NOT blanket-hold the other arm: previously this branch overwrote
            // BOTH targets with prev_sent on any non-Ok command_verdict, so e.g. a left-arm
            // flow IK failure (elbow hitting the URDF joint limit) froze a right-arm
            // InitMotion mid-stream, and vice-versa. Holding only the failed arm is already
            // done inside computeServoTarget; here we just stop discarding the healthy arm.
            safe_target = applySafety(
                desired,
                left_state,
                right_state,
                command.left.mode,
                command.right.mode,
                filter_dt_sec,
                &safety_verdict);
            // A command-generation fault (IkFailed / TrackingError / InvalidCommand /
            // CartesianUnavailable, on one or both arms) outranks a clean safety verdict so
            // the operator still sees the failure (the per-arm *_cart_status columns name the
            // arm); it no longer implies the whole robot is held.
            if (command_verdict != SafetyVerdict::Ok &&
                (safety_verdict == SafetyVerdict::Ok ||
                 safety_verdict == SafetyVerdict::JointLimitClamped)) {
                safety_verdict = command_verdict;
            }
            if (motion_requested) {
                if (safety_verdict == SafetyVerdict::Ok ||
                    safety_verdict == SafetyVerdict::JointLimitClamped) {
                    setMotionState(ServerMotionState::Running);
                } else if (!fault_latched_.load()) {
                    setMotionState(ServerMotionState::ArmedHold);
                }
            }
        }

        // Final output stage: moving average over the last N safety-passed
        // targets (convex combination — joint limits and per-tick velocity
        // bounds preserved). prev_sent bookkeeping, logging, and tracking
        // error all use this filtered value, i.e., what is actually sent.
        ServoTarget output_filtered_target = safe_target;
        output_filtered_target.left_q_target_deg = left_output_ma_.apply(safe_target.left_q_target_deg);
        output_filtered_target.right_q_target_deg = right_output_ma_.apply(safe_target.right_q_target_deg);
        // Patch 4: C-stage telemetry — joint target before/after the output MA.
        left_abc_telemetry_.output_ma_present = true;
        left_abc_telemetry_.q_target_before_output_ma_deg = safe_target.left_q_target_deg;
        left_abc_telemetry_.q_target_after_output_ma_deg = output_filtered_target.left_q_target_deg;
        right_abc_telemetry_.output_ma_present = true;
        right_abc_telemetry_.q_target_before_output_ma_deg = safe_target.right_q_target_deg;
        right_abc_telemetry_.q_target_after_output_ma_deg = output_filtered_target.right_q_target_deg;
        const ServoTarget attempted_target = output_filtered_target;
        const uint64_t command_host_time_ns = command.host_time_ns > 0
            ? command.host_time_ns
            : loop_start;
        const uint64_t fallback_timeout_ns = timeoutNs(
            config_.servo.command_timeout_sec,
            nominal_period_ns
        );
        const uint64_t send_deadline_ns = commandSendDeadlineNs(
            command,
            command_host_time_ns,
            fallback_timeout_ns
        );
        if (send_at_top) {
            // Stage this tick's safety-filtered target for dispatch at the top
            // of the NEXT tick (send policy is re-evaluated there).
            pending_top_send.target = attempted_target;
            pending_top_send.command_seq = command.seq;
            pending_top_send.command_host_time_ns = command_host_time_ns;
            pending_top_send.deadline_ns = send_deadline_ns;
            pending_top_send.valid = true;
        } else {
            fault_latched_before_send = fault_latched_.load();
            send_policy = currentSendPolicy();
            send_suppressed = send_policy != "send_servo_j";
            sent_target = attempted_target;
            dual_send_result = sendTargets(
                attempted_target,
                command.seq,
                command_host_time_ns,
                send_policy,
                loop_start,
                send_deadline_ns
            );
        }
        const SendServoJResult& left_send_result = dual_send_result.left.result;
        const SendServoJResult& right_send_result = dual_send_result.right.result;
        const uint64_t left_send_start_ns = dual_send_result.left.dispatch_timing.start_ns;
        const uint64_t left_send_end_ns = dual_send_result.left.dispatch_timing.end_ns;
        const uint64_t right_send_start_ns = dual_send_result.right.dispatch_timing.start_ns;
        const uint64_t right_send_end_ns = dual_send_result.right.dispatch_timing.end_ns;
        const bool left_ok = left_send_result.accepted;
        const bool right_ok = right_send_result.accepted;
        if (!send_at_top) {
            // Legacy in-tick send: the dispatch's cached state_after is the
            // freshest state available. In send_at_tick_start mode the regular
            // state read above already ran AFTER the dispatch, so it is fresher
            // than the dispatch cache and must not be overwritten.
            if (left_send_result.state_after.has_value()) {
                left_state = *left_send_result.state_after;
                populateTcpPose(left_state, config_.left_mount);
            }
            if (right_send_result.state_after.has_value()) {
                right_state = *right_send_result.state_after;
                populateTcpPose(right_state, config_.right_mount);
            }
        }
        if (left_ok && !fault_latched_before_send && !send_suppressed) {
            noteReferenceSupervisionSentTarget(
                this,
                config_,
                kinematics_,
                kinematics_injected_,
                ArmId::Left,
                sent_target.left_q_target_deg
            );
        }
        if (right_ok && !fault_latched_before_send && !send_suppressed) {
            noteReferenceSupervisionSentTarget(
                this,
                config_,
                kinematics_,
                kinematics_injected_,
                ArmId::Right,
                sent_target.right_q_target_deg
            );
        }
        if (workerBackedIoMode()) {
            left_async_telemetry = asyncTelemetryOrDefault(left_worker_.get());
            right_async_telemetry = asyncTelemetryOrDefault(right_worker_.get());
            updateReferenceSupervision(
                this,
                config_,
                ArmId::Left,
                left_state,
                loop_start,
                &left_async_telemetry
            );
            updateReferenceSupervision(
                this,
                config_,
                ArmId::Right,
                right_state,
                loop_start,
                &right_async_telemetry
            );
        }

        const LatchedDualFaultContext send_fault_contexts =
            classifyDualSendResultContexts(dual_send_result);
        const FaultContext send_fault = send_fault_contexts.top_level.has_value()
            ? *send_fault_contexts.top_level
            : classifyDualSendResult(dual_send_result);
        if (send_fault.verdict != SafetyVerdict::Ok) {
            safety_verdict = send_fault.verdict;
            if (isRealMode() || config_.safety.stop_both_arms_on_single_arm_error) {
                std::string reason = send_fault.reason.empty()
                    ? "sendServoJ failed"
                    : send_fault.reason;
                latchFault(send_fault.verdict, reason, left_state, right_state, send_fault_contexts);
                safe_target = currentFaultHoldTarget();
                safety_verdict = SafetyVerdict::FaultLatched;
            }
        }
        if (rbpodoAsyncIoMode() && !fault_latched_.load()) {
            const LatchedDualFaultContext async_fault_contexts =
                asyncSupervisionFaultContexts(left_async_telemetry, right_async_telemetry);
            if (handleAsyncSupervisionFault(async_fault_contexts, left_state, right_state)) {
                safe_target = currentFaultHoldTarget();
                safety_verdict = SafetyVerdict::FaultLatched;
            }
        }

        const uint64_t loop_end = nowSteadyNs();

        const bool send_cycle = !send_suppressed &&
            (left_send_start_ns > 0 || right_send_start_ns > 0);
        uint64_t first_send_start_ns = 0;
        if (left_send_start_ns > 0 && right_send_start_ns > 0) {
            first_send_start_ns = std::min(left_send_start_ns, right_send_start_ns);
        } else {
            first_send_start_ns = std::max(left_send_start_ns, right_send_start_ns);
        }
        const uint64_t pre_send_ns = first_send_start_ns >= loop_start
            ? first_send_start_ns - loop_start : 0;
        const uint64_t left_send_duration_ns =
            left_send_end_ns >= left_send_start_ns && left_send_start_ns > 0
                ? left_send_end_ns - left_send_start_ns : 0;
        const uint64_t right_send_duration_ns =
            right_send_end_ns >= right_send_start_ns && right_send_start_ns > 0
                ? right_send_end_ns - right_send_start_ns : 0;
        RealtimeTimingTick timing_tick;
        timing_tick.loop_start_ns = loop_start;
        timing_tick.loop_end_ns = loop_end;
        timing_tick.scheduled_wake_ns = sched_wake_ns;
        timing_tick.previous_sleep_enter_ns = prev_sleep_enter_ns;
        timing_tick.nominal_period_ns = nominal_period_ns;
        timing_tick.send_cycle = send_cycle;
        timing_tick.pre_send_ns = pre_send_ns;
        timing_tick.send_duration_ns = std::max(
            left_send_duration_ns, right_send_duration_ns);
        timing_tick.left_feedback.host_time_ns = left_state.host_time_ns;
        timing_tick.left_feedback.robot_time_ns = left_state.robot_time_ns;
        timing_tick.left_feedback.explicit_cached_hold =
            left_state.rbpodo_sdk_state_source.find("last_state_cache") != std::string::npos ||
            left_state.rbpodo_sdk_state_source.find("(held)") != std::string::npos;
        timing_tick.right_feedback.host_time_ns = right_state.host_time_ns;
        timing_tick.right_feedback.robot_time_ns = right_state.robot_time_ns;
        timing_tick.right_feedback.explicit_cached_hold =
            right_state.rbpodo_sdk_state_source.find("last_state_cache") != std::string::npos ||
            right_state.rbpodo_sdk_state_source.find("(held)") != std::string::npos;
        realtime_timing_accumulator_.add(timing_tick);
        const RealtimeTimingTelemetry realtime_timing =
            realtime_timing_accumulator_.snapshot();

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!controller_sim_top_send_bookkeeping && left_ok &&
                !fault_latched_before_send && !send_suppressed) {
                left_prevprev_sent_q_deg_ = left_prev_sent_q_deg_;
                left_prev_sent_q_deg_ = sent_target.left_q_target_deg;
            }
            if (!controller_sim_top_send_bookkeeping && right_ok &&
                !fault_latched_before_send && !send_suppressed) {
                right_prevprev_sent_q_deg_ = right_prev_sent_q_deg_;
                right_prev_sent_q_deg_ = sent_target.right_q_target_deg;
            }
        }

        ServoSample sample;
        sample.tick = tick_++;
        sample.loop_start_time_ns = loop_start;
        sample.loop_end_time_ns = loop_end;
        sample.sched_wake_time_ns = sched_wake_ns;
        // Still holds the PREVIOUS iteration's value here; this tick's sleep-enter
        // stamp happens right before sleep_until below (after this sample is pushed).
        sample.prev_sleep_enter_time_ns = prev_sleep_enter_ns;
        sample.left_state = left_state;
    sample.left_ft = left_ft_telemetry_;
    sample.right_ft = right_ft_telemetry_;
    sample.left_force_control = left_force_control_telemetry_;
    sample.right_force_control = right_force_control_telemetry_;
        sample.right_state = right_state;
        sample.command = command;
        sample.command_buffer_read = command_buffer_read;
        if (chunk_frame_cache_recv_seq_ != 0) {
            const ChunkFrameReceiver::Frame& frame = chunk_frame_cache_;
            ChunkFrameTelemetry& chunk = sample.chunk_frame;
            chunk.wire_seq = frame.seq;
            chunk.recv_seq = frame.receiver_seq;
            chunk.policy_dt_sec = frame.policy_dt_sec;
            chunk.horizon = std::max({
                frame.left.count,
                frame.right.count,
                frame.left_delta.count,
                frame.right_delta.count,
            });
            chunk.execute_steps = frame.diagnostics.execute_steps;
            chunk.runway_steps = frame.diagnostics.runway_steps;
            const double chunk_now_sec = ChunkFrameReceiver::steadyNowSec();
            chunk.age_ms = frame.recv_steady_sec > 0.0 &&
                    chunk_now_sec >= frame.recv_steady_sec
                ? (chunk_now_sec - frame.recv_steady_sec) * 1000.0
                : 0.0;
            chunk.interarrival_ms = frame.interarrival_sec * 1000.0;
            chunk.inference_seq = frame.diagnostics.inference_seq;
            chunk.inference_queue_wait_ms = frame.diagnostics.inference_queue_wait_ms;
            chunk.inference_latency_ms = frame.diagnostics.inference_latency_ms;
            chunk.inference_ready_wait_ms = frame.diagnostics.inference_ready_wait_ms;
            chunk.inference_period_ms = frame.diagnostics.inference_period_ms;
            chunk.inference_period_jitter_ms =
                frame.diagnostics.inference_period_jitter_ms;
            chunk.inference_stall_count = frame.diagnostics.inference_stall_count;
            chunk.camera_bundle_seq = frame.diagnostics.camera_bundle_seq;
            chunk.camera_bundle_age_ms = frame.diagnostics.camera_bundle_age_ms;
            chunk.camera_max_skew_ms = frame.diagnostics.camera_max_skew_ms;
            chunk.camera_left_frame_number = frame.diagnostics.camera_left_frame_number;
            chunk.camera_right_frame_number = frame.diagnostics.camera_right_frame_number;
            chunk.camera_left_frame_age_ms = frame.diagnostics.camera_left_frame_age_ms;
            chunk.camera_right_frame_age_ms = frame.diagnostics.camera_right_frame_age_ms;
            chunk.camera_left_focus_score = frame.diagnostics.camera_left_focus_score;
            chunk.camera_right_focus_score = frame.diagnostics.camera_right_focus_score;
        }
        sample.left_mode_before_init_sequencer = init_mode_before_left;
        sample.right_mode_before_init_sequencer = init_mode_before_right;
        sample.left_mode_after_init_sequencer = init_mode_after_left;
        sample.right_mode_after_init_sequencer = init_mode_after_right;
        sample.left_joint_target_profile_before_init_sequencer = init_profile_before_left;
        sample.right_joint_target_profile_before_init_sequencer = init_profile_before_right;
        sample.left_joint_target_profile_after_init_sequencer = init_profile_after_left;
        sample.right_joint_target_profile_after_init_sequencer = init_profile_after_right;
        sample.non_init_arm_preserved_mode = init_non_init_arm_preserved_mode;
        sample.single_arm_freeze_other_arm =
            config_.safety.init_motion_planner.single_arm_freeze_other_arm;
        sample.left_sent_q_deg = sent_target.left_q_target_deg;
        sample.right_sent_q_deg = sent_target.right_q_target_deg;
        sample.left_send_ok = left_ok;
        sample.right_send_ok = right_ok;
        sample.left_last_read = readCallSnapshot(left_state_result, left_read_fault);
        sample.right_last_read = readCallSnapshot(right_state_result, right_read_fault);
        sample.left_last_send = sendCallSnapshot(left_send_result);
        sample.right_last_send = sendCallSnapshot(right_send_result);
        // When the worker owns the cadence the loop only ENQUEUES; the actual
        // send (and with it the RBACK the box answered) happens later on the
        // worker thread. Taking queue_ack off the enqueue result would log an
        // empty observation forever, so pull the worker's latest SEND result
        // instead. Direct/blocking modes already return the real send here.
        sample.left_queue_ack = left_send_result.queue_ack;
        sample.right_queue_ack = right_send_result.queue_ack;
        if (workerOwnsSendCadence()) {
            // lastSendResult() is NOT usable here: the enqueue path overwrites it
            // every tick, so it reports a default queue_ack almost always
            // (measured: 1 real observation in 7408 ticks). latestQueueAck() is
            // written only on a real send, from the worker thread.
            if (left_worker_) sample.left_queue_ack = left_worker_->latestQueueAck();
            if (right_worker_) sample.right_queue_ack = right_worker_->latestQueueAck();
            // The regulator that produced the trim behind that RBACK. It runs on
            // the worker thread beside the send, so it is read from the same place
            // at the same freshness. Reaching this branch means queue_sync is on
            // (workerOwnsSendCadence() == worker I/O && queue_sync.enable), which
            // is what `enabled` records -- an all-zero row with enabled=0 is
            // "not regulating", not "regulating at zero".
            const auto qsync_snapshot = [](const QueueSyncDecision& d) {
                QueueSyncTelemetry t;
                t.enabled = true;
                t.period_trim_us = d.period_trim_us;
                t.fill_lpf = d.fill_lpf;
                t.integral_us = d.integral_us;
                t.last_fill = d.last_fill;
                t.fill_valid = d.fill_valid;
                t.phase = d.phase;
                t.locked = d.locked;
                t.underrun_events = d.underrun_events;
                t.stall_events = d.stall_events;
                t.highwater_events = d.highwater_events;
                t.redrain_events = d.redrain_events;
                t.no_consumption_events = d.no_consumption_events;
                return t;
            };
            if (left_worker_) {
                sample.left_queue_sync = qsync_snapshot(left_worker_->queueSyncDecision());
            }
            if (right_worker_) {
                sample.right_queue_sync = qsync_snapshot(right_worker_->queueSyncDecision());
            }
        }
        sample.left_cartesian_solve = left_last_cartesian_solve_;
        sample.right_cartesian_solve = right_last_cartesian_solve_;
        mergeAbcTelemetry(sample.left_cartesian_solve, left_abc_telemetry_,
                          config_.servo.output_moving_average_window);
        mergeAbcTelemetry(sample.right_cartesian_solve, right_abc_telemetry_,
                          config_.servo.output_moving_average_window);
        sample.left_safety_tracking = left_safety_tracking_;
        sample.right_safety_tracking = right_safety_tracking_;
        if (!left_ok) {
            sample.left_send_error_kind = toString(left_send_result.error.kind);
            sample.left_send_error_name = left_send_result.error.name;
            sample.left_send_error_code = left_send_result.error.code;
            sample.left_send_error_message = left_send_result.error.message;
        }
        if (!right_ok) {
            sample.right_send_error_kind = toString(right_send_result.error.kind);
            sample.right_send_error_name = right_send_result.error.name;
            sample.right_send_error_code = right_send_result.error.code;
            sample.right_send_error_message = right_send_result.error.message;
        }
        sample.send_suppressed = send_suppressed;
        sample.send_policy = send_policy;
        sample.left_send_start_ns = left_send_start_ns;
        sample.left_send_end_ns = left_send_end_ns;
        sample.right_send_start_ns = right_send_start_ns;
        sample.right_send_end_ns = right_send_end_ns;
        sample.send_skew_us = dual_send_result.left_right_start_skew_us;
        if (left_send_end_ns >= left_send_start_ns && left_send_start_ns > 0) {
            sample.left_send_duration_us = static_cast<double>(left_send_end_ns - left_send_start_ns) / 1000.0;
        }
        if (right_send_end_ns >= right_send_start_ns && right_send_start_ns > 0) {
            sample.right_send_duration_us = static_cast<double>(right_send_end_ns - right_send_start_ns) / 1000.0;
        }
        if (workerBackedIoMode()) {
            sample.left_worker_telemetry = workerTelemetryOrDefault(left_worker_.get());
            sample.right_worker_telemetry = workerTelemetryOrDefault(right_worker_.get());
            sample.left_async_streaming = left_async_telemetry;
            sample.right_async_streaming = right_async_telemetry;
            sample.left_transport_telemetry = workerTransportTelemetry(left_worker_.get());
            sample.right_transport_telemetry = workerTransportTelemetry(right_worker_.get());
        } else {
            sample.left_transport_telemetry = backendTransportTelemetry(left_robot_.get());
            sample.right_transport_telemetry = backendTransportTelemetry(right_robot_.get());
        }
        sample.period_ms = nsToMs(actual_period_ns);
        sample.filter_dt_ms = filter_dt_sec * 1000.0;
        sample.jitter_ms = nsToMs(actual_period_ns > nominal_period_ns
            ? actual_period_ns - nominal_period_ns
            : nominal_period_ns - actual_period_ns);
        sample.safety_verdict = safety_verdict;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            sample.fault_latched = fault_latched_.load();
            sample.motion_state = motion_state_.load();
            sample.async_supervision_degraded = async_supervision_degraded_this_tick;
            sample.tracking_error_degraded = tracking_error_degraded_this_tick_;
            sample.fault_reason = fault_reason_;
            if (latched_fault_context_) {
                sample.latched_fault_context = faultContextSnapshot(*latched_fault_context_);
            } else {
                sample.latched_fault_context.reset();
            }
            if (left_latched_fault_context_) {
                sample.left_latched_fault_context =
                    faultContextSnapshot(*left_latched_fault_context_);
            } else {
                sample.left_latched_fault_context.reset();
            }
            if (right_latched_fault_context_) {
                sample.right_latched_fault_context =
                    faultContextSnapshot(*right_latched_fault_context_);
            } else {
                sample.right_latched_fault_context.reset();
            }

            // Commanded TCP FK from the joints actually sent this cycle. Stable at
            // rest (unlike the controller's noisy jnt_ref-derived tcp_ref), used for
            // clean display in controller simulation.
            if (kinematics_ && (config_.kinematics.publish_tcp || kinematics_injected_)) {
                const auto fk_command_tcp = [&](RobotState& st, const JointArray& q, const ArmMountConfig& mount) {
                    st.tcp_command_stand.reset();
                    if (!finiteJointArray(q)) {
                        return;
                    }
                    try {
                        st.tcp_command_stand = kinematics_->computeTcpStand(st.arm_id, q, mount);
                    } catch (const std::exception&) {
                        st.tcp_command_stand.reset();
                    }
                };
                fk_command_tcp(left_state, attempted_target.left_q_target_deg, config_.left_mount);
                fk_command_tcp(right_state, attempted_target.right_q_target_deg, config_.right_mount);
            }

            sample.left_state = left_state;
            sample.right_state = right_state;
            latest_snapshot_.tick = sample.tick;
            latest_snapshot_.loop_start_time_ns = loop_start;
            latest_snapshot_.loop_end_time_ns = loop_end;
            latest_snapshot_.left_state = left_state;
            latest_snapshot_.left_ft = left_ft_telemetry_;
            latest_snapshot_.right_ft = right_ft_telemetry_;
            latest_snapshot_.left_force_control = left_force_control_telemetry_;
            latest_snapshot_.right_force_control = right_force_control_telemetry_;
            latest_snapshot_.right_state = right_state;
            latest_snapshot_.motion_epoch = motion_epoch_;
            latest_snapshot_.command = command;
            latest_snapshot_.left_sent_q_deg = sent_target.left_q_target_deg;
            latest_snapshot_.right_sent_q_deg = sent_target.right_q_target_deg;
            latest_snapshot_.left_prev_sent_q_deg = left_prev_sent_q_deg_;
            latest_snapshot_.right_prev_sent_q_deg = right_prev_sent_q_deg_;
            latest_snapshot_.period_ms = sample.period_ms;
            latest_snapshot_.jitter_ms = sample.jitter_ms;
            latest_snapshot_.filter_dt_ms = sample.filter_dt_ms;
            latest_snapshot_.realtime_timing = realtime_timing;
            latest_snapshot_.safety_verdict = safety_verdict;
            latest_snapshot_.self_collision_enabled = config_.safety.self_collision.enable;
            latest_snapshot_.self_collision_checked = last_self_collision_.checked;
            latest_snapshot_.self_collision_violated = last_self_collision_.violated;
            latest_snapshot_.self_collision_min_clearance_m = last_self_collision_.min_clearance_m;
            latest_snapshot_.self_collision_external_box_min_clearance_m =
                last_collision_verdict_.external_box_min_clearance_m;
            latest_snapshot_.self_collision_external_box_clearance_m =
                last_collision_verdict_.external_box_clearance_m;
            // Telemetry "margin" is the hard floor the mesh barrier defends.
            latest_snapshot_.self_collision_margin_m = config_.safety.self_collision.mesh.d_hard_m;
            latest_snapshot_.self_collision_left_bone = last_self_collision_.left_bone;
            latest_snapshot_.self_collision_right_bone = last_self_collision_.right_bone;
            latest_snapshot_.self_collision_pair = last_self_collision_.pair;
            // Mirror the mesh-monitor proximity into the CSV sample (see ServoSample) so a
            // controller op_stat_self_collision (1005) latch is cross-checkable post-hoc.
            sample.self_collision_min_clearance_m = last_self_collision_.min_clearance_m;
            sample.self_collision_pair = last_self_collision_.pair;
            latest_snapshot_.self_collision_stand_capsule = last_self_collision_.stand_capsule;
            latest_snapshot_.self_collision_has_closest_points = last_self_collision_.has_closest_points;
            latest_snapshot_.self_collision_closest_point_a_m = last_self_collision_.closest_point_a_m;
            latest_snapshot_.self_collision_closest_point_b_m = last_self_collision_.closest_point_b_m;
            // URDF mesh self-collision: publish the closest near pairs (witness
            // segments) so the viewer can draw the mesh-based close calls.
            latest_snapshot_.self_collision_mesh = config_.safety.self_collision.enable;
            latest_snapshot_.self_collision_near_pairs.clear();
            if (config_.safety.self_collision.enable && last_collision_verdict_.valid) {
                // Publish close-call witness segments for the viewer. The viz reach is
                // decoupled from the barrier band (d_slow) so close calls stay visible
                // even when d_slow is tuned tight; at least d_slow so it never hides a
                // pair the barrier is acting on.
                const double viz_max = std::max(config_.safety.self_collision.mesh.viz_near_pairs_m,
                                                config_.safety.self_collision.mesh.d_slow_m);
                latest_snapshot_.self_collision_near_pairs.reserve(last_collision_verdict_.near.size());
                for (const CollisionNearPair& p : last_collision_verdict_.near) {
                    if (p.d_m > viz_max) continue;
                    latest_snapshot_.self_collision_near_pairs.push_back(SelfCollisionNearPairViz{
                        p.name_a, p.name_b,
                        {p.p_a.x(), p.p_a.y(), p.p_a.z()},
                        {p.p_b.x(), p.p_b.y(), p.p_b.z()},
                        p.d_m, p.external, p.external_box});
                }
            }
            {
                const auto status_string = [](InitMotionStatus status) {
                    switch (status) {
                        case InitMotionStatus::Idle:
                            return std::string("idle");
                        case InitMotionStatus::Planning:
                            return std::string("planning");
                        case InitMotionStatus::Executing:
                            return std::string("executing");
                        case InitMotionStatus::Done:
                            return std::string("done");
                        case InitMotionStatus::Failed:
                            return std::string("failed");
                    }
                    return std::string("unknown");
                };
                InitMotionArmDiag left_diag;
                InitMotionArmDiag right_diag;
                auto fail_mode_string = [&](const InitMotionExec& ex) {
                    if (ex.exec_timeout) return std::string("exec_timeout");
                    if (ex.exec_stalled) return std::string("exec_stalled");
                    return initMotionFailModeString(ex.fail_mode);
                };
                const JointArray left_init_diag_q =
                    (left_state.q_actual_valid && finiteJointArray(left_state.q_actual_deg))
                        ? left_state.q_actual_deg
                        : left_prev_sent_q_deg_;
                const JointArray right_init_diag_q =
                    (right_state.q_actual_valid && finiteJointArray(right_state.q_actual_deg))
                        ? right_state.q_actual_deg
                        : right_prev_sent_q_deg_;
                auto fill_arm = [&](InitMotionArmDiag& arm, const InitMotionExec& ex, bool left_arm) {
                    arm.status = status_string(ex.status);
                    arm.fail_mode = fail_mode_string(ex);
                    arm.message = ex.message;
                    arm.waypoint_index = static_cast<int>(ex.index);
                    arm.waypoint_count = static_cast<int>(ex.waypoints.size());
                    arm.goal_self_min_clearance_m = ex.goal_self_min_clearance_m;
                    arm.goal_external_min_clearance_m = ex.goal_external_min_clearance_m;
                    arm.goal_nearest_pair_name_a = ex.goal_nearest_pair_name_a;
                    arm.goal_nearest_pair_name_b = ex.goal_nearest_pair_name_b;
                    arm.goal_nearest_pair_category = ex.goal_nearest_pair_category;
                    arm.goal_nearest_pair_external = ex.goal_nearest_pair_external;
                    arm.goal_nearest_pair_disabled_by_rule = ex.goal_nearest_pair_disabled_by_rule;
                    arm.goal_nearest_pair_distance_m = ex.goal_nearest_pair_distance_m;
                    arm.goal_clear_threshold_self_m = ex.goal_clear_threshold_self_m;
                    arm.goal_clear_threshold_external_m = ex.goal_clear_threshold_external_m;
                    arm.goal_clear_margin_deficit_m = ex.goal_clear_margin_deficit_m;
                    arm.clear_threshold_m = ex.clear_threshold_m;
                    arm.external_clear_threshold_m = ex.external_clear_threshold_m;
                    arm.nearest_pair = ex.nearest_pair;
                    arm.nearest_pair_distance_m = ex.nearest_pair_distance_m;
                    arm.nearest_pair_external = ex.nearest_pair_external;
                    arm.dist_to_goal_deg = std::numeric_limits<double>::quiet_NaN();
                    if (ex.waypoints.empty() ||
                        !(ex.status == InitMotionStatus::Executing ||
                          ex.status == InitMotionStatus::Done ||
                          ex.status == InitMotionStatus::Failed)) {
                        return;
                    }
                    const auto& goal_wp = ex.waypoints.back();
                    double dist = 0.0;
                    for (int i = 0; i < kDof; ++i) {
                        dist = std::max(dist, std::abs(
                            (left_arm ? left_init_diag_q[i] : right_init_diag_q[i]) -
                            (left_arm ? goal_wp.first[i] : goal_wp.second[i])));
                    }
                    arm.dist_to_goal_deg = dist;
                };
                const InitMotionExec* left_owner = nullptr;
                const InitMotionExec* right_owner = nullptr;
                for (const InitMotionExec* ex : {&left_init_motion_exec_, &right_init_motion_exec_}) {
                    if (ex->status == InitMotionStatus::Idle) continue;
                    if (ex->left_active && left_owner == nullptr) left_owner = ex;
                    if (ex->right_active && right_owner == nullptr) right_owner = ex;
                }
                if (left_owner != nullptr) fill_arm(left_diag, *left_owner, true);
                if (right_owner != nullptr) fill_arm(right_diag, *right_owner, false);

                InitMotionDiag diag;
                const InitMotionExec* aggregate_owner =
                    left_owner != nullptr ? left_owner : right_owner;
                if (left_owner != nullptr && right_owner != nullptr &&
                    left_owner->status == InitMotionStatus::Idle &&
                    right_owner->status != InitMotionStatus::Idle) {
                    aggregate_owner = right_owner;
                }
                if (aggregate_owner != nullptr) {
                    diag.status = status_string(aggregate_owner->status);
                    diag.fail_mode = fail_mode_string(*aggregate_owner);
                    diag.message = aggregate_owner->message;
                    diag.start_clear_m = aggregate_owner->start_clear_m;
                    diag.goal_clear_m = aggregate_owner->goal_clear_m;
                    diag.goal_self_min_clearance_m = aggregate_owner->goal_self_min_clearance_m;
                    diag.goal_external_min_clearance_m = aggregate_owner->goal_external_min_clearance_m;
                    diag.goal_nearest_pair_name_a = aggregate_owner->goal_nearest_pair_name_a;
                    diag.goal_nearest_pair_name_b = aggregate_owner->goal_nearest_pair_name_b;
                    diag.goal_nearest_pair_category = aggregate_owner->goal_nearest_pair_category;
                    diag.goal_nearest_pair_external = aggregate_owner->goal_nearest_pair_external;
                    diag.goal_nearest_pair_disabled_by_rule =
                        aggregate_owner->goal_nearest_pair_disabled_by_rule;
                    diag.goal_nearest_pair_distance_m =
                        aggregate_owner->goal_nearest_pair_distance_m;
                    diag.goal_clear_threshold_self_m = aggregate_owner->goal_clear_threshold_self_m;
                    diag.goal_clear_threshold_external_m = aggregate_owner->goal_clear_threshold_external_m;
                    diag.goal_clear_margin_deficit_m =
                        aggregate_owner->goal_clear_margin_deficit_m;
                    diag.tree_start = aggregate_owner->tree_start_size;
                    diag.tree_goal = aggregate_owner->tree_goal_size;
                    diag.iterations = aggregate_owner->last_iterations;
                    diag.planning_time_s = aggregate_owner->last_planning_time_s;
                    diag.waypoint_index = static_cast<int>(aggregate_owner->index);
                    diag.waypoint_count = static_cast<int>(aggregate_owner->waypoints.size());
                    diag.clear_threshold_m = aggregate_owner->clear_threshold_m;
                    diag.external_clear_threshold_m = aggregate_owner->external_clear_threshold_m;
                    diag.nearest_pair = aggregate_owner->nearest_pair;
                    diag.nearest_pair_distance_m = aggregate_owner->nearest_pair_distance_m;
                    diag.nearest_pair_external = aggregate_owner->nearest_pair_external;
                    diag.dist_to_goal_deg = std::numeric_limits<double>::quiet_NaN();
                    if (std::isfinite(left_diag.dist_to_goal_deg)) {
                        diag.dist_to_goal_deg = left_diag.dist_to_goal_deg;
                    }
                    if (std::isfinite(right_diag.dist_to_goal_deg)) {
                        diag.dist_to_goal_deg = std::isfinite(diag.dist_to_goal_deg)
                            ? std::max(diag.dist_to_goal_deg, right_diag.dist_to_goal_deg)
                            : right_diag.dist_to_goal_deg;
                    }
                }
                sample.init_motion = diag;
                sample.init_motion_left = left_diag;
                sample.init_motion_right = right_diag;
                latest_snapshot_.init_motion = diag;
                latest_snapshot_.init_motion_left = left_diag;
                latest_snapshot_.init_motion_right = right_diag;
            }
            latest_snapshot_.floor_constraint_enabled = floorConstraintActive();
            latest_snapshot_.floor_constraint_monitor_only = config_.safety.floor_constraint.monitor_only;
            latest_snapshot_.floor_constraint_z_min_m = effectiveFloorZ();
            latest_snapshot_.floor_constraint_config_z_min_m = config_.safety.floor_constraint.z_min_m;
            latest_snapshot_.floor_constraint_runtime_min_z_m = config_.safety.floor_constraint.runtime_min_z_m;
            latest_snapshot_.floor_constraint_runtime_max_z_m = config_.safety.floor_constraint.runtime_max_z_m;
            latest_snapshot_.floor_constraint_left_checked = last_floor_left_.checked;
            latest_snapshot_.floor_constraint_left_violated = last_floor_left_.violated;
            latest_snapshot_.floor_constraint_left_tcp_z_m = last_floor_left_.tcp_z_m;
            latest_snapshot_.floor_constraint_left_lowest_point = last_floor_left_.lowest_point;
            latest_snapshot_.floor_constraint_left_lowest_point_m = {
                last_floor_left_.lowest_point_stand.x(), last_floor_left_.lowest_point_stand.y(),
                last_floor_left_.lowest_point_stand.z()};
            latest_snapshot_.floor_constraint_right_checked = last_floor_right_.checked;
            latest_snapshot_.floor_constraint_right_violated = last_floor_right_.violated;
            latest_snapshot_.floor_constraint_right_tcp_z_m = last_floor_right_.tcp_z_m;
            latest_snapshot_.floor_constraint_right_lowest_point = last_floor_right_.lowest_point;
            latest_snapshot_.floor_constraint_right_lowest_point_m = {
                last_floor_right_.lowest_point_stand.x(), last_floor_right_.lowest_point_stand.y(),
                last_floor_right_.lowest_point_stand.z()};
            latest_snapshot_.floor_constraint_clamp_count = floor_clamp_count_;
            latest_snapshot_.floor_constraint_last_set_reject_reason = floor_last_set_reject_reason_;
            latest_snapshot_.roi_box_enabled = config_.safety.roi_box.enable;
            latest_snapshot_.roi_box_monitor_only = config_.safety.roi_box.monitor_only;
            latest_snapshot_.roi_box_min_m = effectiveRoiMin();
            latest_snapshot_.roi_box_max_m = effectiveRoiMax();
            latest_snapshot_.roi_box_runtime_min_m = config_.safety.roi_box.runtime_min_m;
            latest_snapshot_.roi_box_runtime_max_m = config_.safety.roi_box.runtime_max_m;
            latest_snapshot_.roi_box_left_checked = last_roi_left_.checked;
            latest_snapshot_.roi_box_left_violated = last_roi_left_.violated;
            latest_snapshot_.roi_box_left_min_margin_m = last_roi_left_.min_margin_m;
            latest_snapshot_.roi_box_left_closest_face = last_roi_left_.closest_face;
            latest_snapshot_.roi_box_right_checked = last_roi_right_.checked;
            latest_snapshot_.roi_box_right_violated = last_roi_right_.violated;
            latest_snapshot_.roi_box_right_min_margin_m = last_roi_right_.min_margin_m;
            latest_snapshot_.roi_box_right_closest_face = last_roi_right_.closest_face;
            latest_snapshot_.roi_box_clamp_count = roi_clamp_count_;
            latest_snapshot_.roi_box_last_set_reject_reason = roi_last_set_reject_reason_;
            {
                const math::Vector3 uf_point = effectiveUserFloorPoint();
                const math::Vector3 uf_normal = effectiveUserFloorNormal();
                latest_snapshot_.user_floor_constraint_enabled = userFloorActive();
                latest_snapshot_.user_floor_constraint_monitor_only =
                    config_.safety.user_floor_constraint.monitor_only;
                latest_snapshot_.user_floor_constraint_point_m = {uf_point.x(), uf_point.y(), uf_point.z()};
                latest_snapshot_.user_floor_constraint_normal = {uf_normal.x(), uf_normal.y(), uf_normal.z()};
                latest_snapshot_.user_floor_constraint_margin_m = effectiveUserFloorMargin();
                latest_snapshot_.user_floor_constraint_left_checked = last_user_floor_left_.checked;
                latest_snapshot_.user_floor_constraint_left_violated = last_user_floor_left_.violated;
                latest_snapshot_.user_floor_constraint_left_signed_dist_m = last_user_floor_left_.signed_dist_m;
                latest_snapshot_.user_floor_constraint_left_lowest_point = last_user_floor_left_.lowest_point;
                latest_snapshot_.user_floor_constraint_left_lowest_point_m = {
                    last_user_floor_left_.lowest_point_stand.x(),
                    last_user_floor_left_.lowest_point_stand.y(),
                    last_user_floor_left_.lowest_point_stand.z()};
                latest_snapshot_.user_floor_constraint_right_checked = last_user_floor_right_.checked;
                latest_snapshot_.user_floor_constraint_right_violated = last_user_floor_right_.violated;
                latest_snapshot_.user_floor_constraint_right_signed_dist_m = last_user_floor_right_.signed_dist_m;
                latest_snapshot_.user_floor_constraint_right_lowest_point = last_user_floor_right_.lowest_point;
                latest_snapshot_.user_floor_constraint_right_lowest_point_m = {
                    last_user_floor_right_.lowest_point_stand.x(),
                    last_user_floor_right_.lowest_point_stand.y(),
                    last_user_floor_right_.lowest_point_stand.z()};
                latest_snapshot_.user_floor_constraint_clamp_count = user_floor_clamp_count_;
                latest_snapshot_.user_floor_constraint_last_set_reject_reason =
                    user_floor_last_set_reject_reason_;
            }
            latest_snapshot_.motion_state = sample.motion_state;
            latest_snapshot_.fault_latched = sample.fault_latched;
            latest_snapshot_.async_supervision_degraded = sample.async_supervision_degraded;
            latest_snapshot_.tracking_error_degraded = sample.tracking_error_degraded;
            const FreedriveStage left_fd_stage = left_freedrive_stage_.load();
            const FreedriveStage right_fd_stage = right_freedrive_stage_.load();
            latest_snapshot_.left_freedrive_active = left_fd_stage == FreedriveStage::Active;
            latest_snapshot_.right_freedrive_active = right_fd_stage == FreedriveStage::Active;
            latest_snapshot_.left_freedrive_stage = toString(left_fd_stage);
            latest_snapshot_.right_freedrive_stage = toString(right_fd_stage);
            latest_snapshot_.freedrive_note = freedrive_note_;
            latest_snapshot_.latched_fault_reason = latched_fault_reason_.load();
            latest_snapshot_.fault_reason = fault_reason_;
            latest_snapshot_.latched_fault_context = sample.latched_fault_context;
            latest_snapshot_.left_latched_fault_context = sample.left_latched_fault_context;
            latest_snapshot_.right_latched_fault_context = sample.right_latched_fault_context;
            latest_snapshot_.left_send_ok = left_ok;
            latest_snapshot_.right_send_ok = right_ok;
            latest_snapshot_.left_last_read = sample.left_last_read;
            latest_snapshot_.right_last_read = sample.right_last_read;
            latest_snapshot_.left_last_send = sample.left_last_send;
            latest_snapshot_.right_last_send = sample.right_last_send;
            latest_snapshot_.left_cartesian_solve = sample.left_cartesian_solve;
            latest_snapshot_.right_cartesian_solve = sample.right_cartesian_solve;
            latest_snapshot_.left_safety_tracking = sample.left_safety_tracking;
            latest_snapshot_.right_safety_tracking = sample.right_safety_tracking;
            latest_snapshot_.left_send_error_kind = sample.left_send_error_kind;
            latest_snapshot_.left_send_error_name = sample.left_send_error_name;
            latest_snapshot_.left_send_error_code = sample.left_send_error_code;
            latest_snapshot_.left_send_error_message = sample.left_send_error_message;
            latest_snapshot_.right_send_error_kind = sample.right_send_error_kind;
            latest_snapshot_.right_send_error_name = sample.right_send_error_name;
            latest_snapshot_.right_send_error_code = sample.right_send_error_code;
            latest_snapshot_.right_send_error_message = sample.right_send_error_message;
            latest_snapshot_.send_suppressed = send_suppressed;
            latest_snapshot_.send_policy = send_policy;
            latest_snapshot_.left_send_start_ns = left_send_start_ns;
            latest_snapshot_.left_send_end_ns = left_send_end_ns;
            latest_snapshot_.right_send_start_ns = right_send_start_ns;
            latest_snapshot_.right_send_end_ns = right_send_end_ns;
            latest_snapshot_.send_skew_us = sample.send_skew_us;
            latest_snapshot_.left_send_duration_us = sample.left_send_duration_us;
            latest_snapshot_.right_send_duration_us = sample.right_send_duration_us;
            latest_snapshot_.left_worker_telemetry = sample.left_worker_telemetry;
            latest_snapshot_.right_worker_telemetry = sample.right_worker_telemetry;
            latest_snapshot_.left_async_streaming = sample.left_async_streaming;
            latest_snapshot_.right_async_streaming = sample.right_async_streaming;
            latest_snapshot_.left_transport_telemetry = sample.left_transport_telemetry;
            latest_snapshot_.right_transport_telemetry = sample.right_transport_telemetry;
            latest_snapshot_.startup_validation = startup_validation_;
            latest_snapshot_.logger_dropped_samples = logger_ ? logger_->droppedSamples() : 0;
        }

        if (logger_) {
            logger_->push(sample);
        }
        if (scope_publisher_) {
            scope_publisher_->push(scopeSampleFromServoSample(sample));
        }

        async_supervision_degraded_last_tick = async_supervision_degraded_this_tick;
        tracking_error_degraded_prev_tick_ = tracking_error_degraded_this_tick_;
        prev_sleep_enter_ns = nowSteadyNs();
        if (spin_slack_ns > 0) {
            // Sleep until `slack` before the deadline, then busy-spin the rest so
            // the tick wakes exactly on phase (no C-state/scheduler wake jitter).
            std::this_thread::sleep_until(next_tick - std::chrono::nanoseconds(spin_slack_ns));
            while (std::chrono::steady_clock::now() < next_tick) {
                cpuRelax();
            }
        } else {
            std::this_thread::sleep_until(next_tick);
        }
    }
}

bool DualArmServoLoop::configureRealtimeForLoop() {
    bool ok = true;
    if (config_.servo.enable_realtime_priority) {
        ok = lockMemory() && ok;
        ok = setCurrentThreadRealtimePriority(config_.servo.realtime_priority) && ok;
    }
    if (config_.servo.cpu_core >= 0) {
        ok = pinCurrentThreadToCpu(config_.servo.cpu_core) && ok;
    }

    if (!ok && isRealMode()) {
        std::cerr << "[ERROR] realtime setup failed in real mode\n";
        return false;
    }
    return true;
}

bool DualArmServoLoop::readRobotStates(RobotState& left, RobotState& right) {
    const uint64_t max_age_ns =
        static_cast<uint64_t>(std::max(0.1, config_.servo.command_timeout_sec) * 1'000'000'000.0);
    const BackendResult<RobotState> left_result = workerBackedIoMode()
        ? (left_worker_ ? left_worker_->latestState(max_age_ns) : BackendResult<RobotState>{})
        : (left_robot_ ? left_robot_->readState() : BackendResult<RobotState>{});
    const BackendResult<RobotState> right_result = workerBackedIoMode()
        ? (right_worker_ ? right_worker_->latestState(max_age_ns) : BackendResult<RobotState>{})
        : (right_robot_ ? right_robot_->readState() : BackendResult<RobotState>{});
    if (left_result.ok) {
        left = left_result.value;
    } else {
        left = left_result.value;
        left.arm_id = ArmId::Left;
    }
    if (right_result.ok) {
        right = right_result.value;
    } else {
        right = right_result.value;
        right.arm_id = ArmId::Right;
    }
    populateTcpPose(left, config_.left_mount);
    populateTcpPose(right, config_.right_mount);
    return left_result.ok && right_result.ok;
}

void DualArmServoLoop::populateTcpPose(RobotState& state, const ArmMountConfig& mount) const {
    state.tcp_base.reset();
    state.tcp_stand.reset();
    state.tcp_actual_base.reset();
    state.tcp_actual_stand.reset();
    state.tcp_ref_base.reset();
    state.tcp_ref_stand.reset();
    state.has_valid_tcp_pose = false;
    state.tcp_actual_valid = false;
    state.tcp_ref_valid = false;
    state.fk_duration_us = 0.0;
    const bool publish_tcp = config_.kinematics.publish_tcp || kinematics_injected_;
    state.tcp_deferred = kinematics_ == nullptr || !publish_tcp;

    if (!kinematics_ || !publish_tcp) {
        return;
    }
    if (!isValidJointState(state)) {
        return;
    }

    const auto started = std::chrono::steady_clock::now();
    try {
        state.tcp_actual_base = kinematics_->computeTcpBase(state.q_actual_deg);
        state.tcp_actual_stand = kinematics_->computeTcpStand(state.arm_id, state.q_actual_deg, mount);
        state.tcp_base = state.tcp_actual_base;
        state.tcp_stand = state.tcp_actual_stand;
        state.fk_duration_us = std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - started
        ).count();
        state.has_valid_tcp_pose = true;
        state.tcp_actual_valid = true;
        state.tcp_deferred = false;
    } catch (const std::exception& exc) {
        state.fk_duration_us = std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - started
        ).count();
        std::cerr << "[WARN] FK TCP publish invalid for "
                  << (state.arm_id == ArmId::Left ? "left" : "right")
                  << " arm: " << exc.what() << "\n";
        return;
    }

    if (!finiteJointArray(state.q_target_deg)) {
        return;
    }
    try {
        state.tcp_ref_base = kinematics_->computeTcpBase(state.q_target_deg);
        state.tcp_ref_stand = kinematics_->computeTcpStand(state.arm_id, state.q_target_deg, mount);
        state.tcp_ref_valid = true;
        state.fk_duration_us = std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - started
        ).count();
    } catch (const std::exception& exc) {
        state.tcp_ref_base.reset();
        state.tcp_ref_stand.reset();
        state.tcp_ref_valid = false;
        state.fk_duration_us = std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - started
        ).count();
        std::cerr << "[WARN] reference FK TCP publish invalid for "
                  << (state.arm_id == ArmId::Left ? "left" : "right")
                  << " arm: " << exc.what() << "\n";
    }
}

ArmStartupValidationSnapshot DualArmServoLoop::validateStartupArm(const RobotState& state) const {
    ArmStartupValidationSnapshot validation;
    validation.read_only_diagnostic = readOnlyDiagnosticStartupEnabled();

    if (state.connection_state != RobotConnectionState::Connected) {
        appendReason(&validation, "not_connected");
    }
    if (!state.has_valid_joint_state) {
        appendReason(&validation, "invalid_joint_state");
    }

    bool finite_q_actual = true;
    bool any_startup_wrapped = false;
    JointArray normalized_q_actual = state.q_actual_deg;
    for (int i = 0; i < kDof; ++i) {
        const double q = state.q_actual_deg[i];
        if (!std::isfinite(q)) {
            finite_q_actual = false;
            continue;
        }
        double q_for_range = q;
        if (config_.safety.joint_wrap_for_startup_validation) {
            const JointRangeNormalization normalization = normalizeJointForRange(
                q,
                config_.safety.q_min_deg[i],
                config_.safety.q_max_deg[i],
                config_.safety.joint_wrap_period_deg[i]
            );
            q_for_range = normalization.normalized_value_deg;
            normalized_q_actual[i] = q_for_range;
            if (normalization.was_wrapped) {
                any_startup_wrapped = true;
                validation.q_range_wrapped.push_back({
                    i + 1,
                    q,
                    normalization.normalized_value_deg,
                    config_.safety.joint_wrap_period_deg[i],
                });
            }
        }
        if (q_for_range < config_.safety.q_min_deg[i] ||
            q_for_range > config_.safety.q_max_deg[i]) {
            validation.q_range_violations.push_back({
                i + 1,
                q_for_range,
                config_.safety.q_min_deg[i],
                config_.safety.q_max_deg[i],
            });
        }
    }
    if (any_startup_wrapped) {
        validation.q_actual_normalized_for_safety_deg = normalized_q_actual;
    }
    if (!finite_q_actual) {
        appendReason(&validation, "non_finite_q_actual");
    }

    validation.acquisition_ok =
        state.connection_state == RobotConnectionState::Connected &&
        state.has_valid_joint_state &&
        finite_q_actual;

    if (state.has_error) {
        appendReason(&validation, "robot_fault");
        validation.diagnostic_error_source = state.diagnostic_error_source.empty()
            ? "error_code:" + std::to_string(state.error_code)
            : state.diagnostic_error_source;
    }
    if (!validation.q_range_violations.empty()) {
        appendReason(&validation, "q_range_violation");
    }
    if (state.motion_readiness_error_kind == "WrongMode") {
        appendReason(&validation, "wrong_mode");
    } else if (state.motion_readiness_error_kind == "ServoDisabled") {
        appendReason(&validation, "servo_disabled");
    } else if (!state.motion_readiness_error_kind.empty() &&
               state.motion_readiness_error_kind != "None" &&
               state.motion_readiness_error_kind != "RobotFault") {
        appendReason(&validation, "motion_readiness_error");
    }
    if (validation.diagnostic_error_source.empty() && !state.diagnostic_error_source.empty()) {
        validation.diagnostic_error_source = state.diagnostic_error_source;
    }
    if (!state.servo_enabled) {
        appendReason(&validation, "servo_disabled");
    }

    validation.motion_ready =
        validation.acquisition_ok &&
        !state.has_error &&
        validation.q_range_violations.empty() &&
        (state.motion_readiness_error_kind.empty() ||
         state.motion_readiness_error_kind == "None") &&
        state.servo_enabled;
    return validation;
}

StartupValidationSnapshot DualArmServoLoop::validateStartupStates(
    const RobotState& left,
    const RobotState& right
) const {
    StartupValidationSnapshot validation;
    validation.left = validateStartupArm(left);
    validation.right = validateStartupArm(right);
    validation.acquisition_ok = validation.left.acquisition_ok && validation.right.acquisition_ok;
    validation.motion_ready = validation.left.motion_ready && validation.right.motion_ready;
    validation.read_only_diagnostic =
        validation.left.read_only_diagnostic || validation.right.read_only_diagnostic;
    validation.allowed_unsafe_startup =
        validation.acquisition_ok &&
        !validation.motion_ready &&
        startupValidationAllowsStart(validation);
    validation.left.allowed_unsafe_startup =
        validation.left.acquisition_ok &&
        !validation.left.motion_ready &&
        validation.allowed_unsafe_startup;
    validation.right.allowed_unsafe_startup =
        validation.right.acquisition_ok &&
        !validation.right.motion_ready &&
        validation.allowed_unsafe_startup;
    return validation;
}

bool DualArmServoLoop::startupValidationAllowsStart(
    const StartupValidationSnapshot& validation
) const {
    if (!validation.acquisition_ok) return false;
    if (validation.motion_ready) {
        return !controllerSimulationMotionRequired(config_) ||
            controllerSimulationMotionGateOpen(config_);
    }
    if (controllerSimulationDiagnosticsSuspectStartupAllowed(config_, validation)) {
        return true;
    }
    if (controllerSimulationInitErrorStartupAllowed(config_, validation)) {
        return true;
    }
    if (!readOnlyDiagnosticStartupEnabled()) return false;
    if (config_.servo.send_servo_commands) return false;

    const auto arm_allowed = [&](const ArmStartupValidationSnapshot& arm) {
        if (arm.motion_ready) return true;
        if (!arm.acquisition_ok) return false;
        if (containsReason(arm, "robot_fault") &&
            !config_.servo.allow_readonly_faulted_startup) {
            return false;
        }
        if (containsReason(arm, "q_range_violation") &&
            !config_.servo.allow_readonly_q_range_violation_startup) {
            return false;
        }
        if ((containsReason(arm, "servo_disabled") || containsReason(arm, "wrong_mode")) &&
            !config_.servo.allow_readonly_wrong_mode_startup) {
            return false;
        }
        for (const std::string& reason : arm.invalid_reasons) {
            if (reason == "robot_fault" ||
                reason == "q_range_violation" ||
                reason == "servo_disabled" ||
                reason == "wrong_mode") {
                continue;
            }
            return false;
        }
        return true;
    };

    return arm_allowed(validation.left) && arm_allowed(validation.right);
}

bool DualArmServoLoop::readOnlyDiagnosticStartupEnabled() const {
    return readOnlyMode() &&
        (config_.servo.allow_readonly_faulted_startup ||
         config_.servo.allow_readonly_q_range_violation_startup ||
         config_.servo.allow_readonly_wrong_mode_startup);
}

bool DualArmServoLoop::initializeStartupTargets(
    const RobotState& left,
    const RobotState& right
) {
    const StartupTrackingTargetSelection left_target =
        startupTrackingReferenceForArm(config_, ArmId::Left, left);
    const StartupTrackingTargetSelection right_target =
        startupTrackingReferenceForArm(config_, ArmId::Right, right);

    bool ok = true;
    if (!left_target.ok) {
        logStartupReferenceUnavailable(ArmId::Left, left, left_target);
        ok = false;
    }
    if (!right_target.ok) {
        logStartupReferenceUnavailable(ArmId::Right, right, right_target);
        ok = false;
    }
    if (!ok) return false;

    left_prev_sent_q_deg_ = left_target.q_deg;
    left_prevprev_sent_q_deg_ = left_target.q_deg;
    right_prev_sent_q_deg_ = right_target.q_deg;
    right_prevprev_sent_q_deg_ = right_target.q_deg;
    left_controller_sim_physical_baseline_q_deg_ = left.q_actual_deg;
    right_controller_sim_physical_baseline_q_deg_ = right.q_actual_deg;
    left_fault_hold_q_deg_ = left_target.q_deg;
    right_fault_hold_q_deg_ = right_target.q_deg;

    std::cerr << "[INFO] startup_previous_target_source"
              << " left=" << left_target.source
              << " right=" << right_target.source
              << " physical_baseline_source=q_actual\n";
    return true;
}

void DualArmServoLoop::logStartupValidation(
    const StartupValidationSnapshot& validation,
    const RobotState& left,
    const RobotState& right
) const {
    const bool start_allowed = startupValidationAllowsStart(validation);
    const auto log_arm = [&](const char* name,
                             const RobotState& state,
                             const ArmStartupValidationSnapshot& arm) {
        if (arm.motion_ready) return;
        const bool controller_sim_diagnostics_suspect_override_active =
            start_allowed &&
            arm.allowed_unsafe_startup &&
            arm.diagnostic_error_source == "rbpodo_diagnostics_suspect" &&
            config_.servo.send_servo_commands;
        const bool controller_sim_init_error_override_active =
            start_allowed &&
            arm.allowed_unsafe_startup &&
            arm.diagnostic_error_source == "rbpodo_init_error" &&
            config_.servo.send_servo_commands;
        const bool controller_sim_not_activated_override_active =
            start_allowed &&
            arm.allowed_unsafe_startup &&
            containsReason(arm, "servo_disabled") &&
            controllerSimulationNotActivatedGateOpen(config_) &&
            config_.servo.send_servo_commands;
        std::cerr << "[ERROR] invalid robot startup state: " << name << "\n"
                  << "  has_valid_joint_state=" << (state.has_valid_joint_state ? "true" : "false") << "\n"
                  << "  has_error=" << (state.has_error ? "true" : "false") << "\n"
                  << "  error_code=" << state.error_code << "\n"
                  << "  lifecycle_state=" << (state.lifecycle_state.empty() ? "null" : state.lifecycle_state) << "\n"
                  << "  q_actual_deg=" << jointArrayDebugString(state.q_actual_deg) << "\n"
                  << "  q_target_deg=" << jointArrayDebugString(state.q_target_deg) << "\n";
        if (arm.q_actual_normalized_for_safety_deg.has_value()) {
            std::cerr << "  q_actual_normalized_for_safety_deg="
                      << jointArrayDebugString(*arm.q_actual_normalized_for_safety_deg) << "\n";
        }
        std::cerr
                  << "  startup_invalid_reasons=";
        for (std::size_t i = 0; i < arm.invalid_reasons.size(); ++i) {
            if (i > 0) std::cerr << ",";
            std::cerr << arm.invalid_reasons[i];
        }
        std::cerr << "\n  q_range_violations=[";
        for (std::size_t i = 0; i < arm.q_range_violations.size(); ++i) {
            const JointRangeViolation& violation = arm.q_range_violations[i];
            if (i > 0) std::cerr << ",";
            std::cerr << "{joint:" << violation.joint
                      << ",value_deg:" << violation.value_deg
                      << ",min:" << violation.min_deg
                      << ",max:" << violation.max_deg << "}";
        }
        std::cerr << "]\n  q_range_wrapped=[";
        for (std::size_t i = 0; i < arm.q_range_wrapped.size(); ++i) {
            const JointRangeWrapped& wrapped = arm.q_range_wrapped[i];
            if (i > 0) std::cerr << ",";
            std::cerr << "{joint:" << wrapped.joint
                      << ",raw_deg:" << wrapped.raw_deg
                      << ",normalized_deg:" << wrapped.normalized_deg
                      << ",period_deg:" << wrapped.period_deg << "}";
        }
        std::cerr << "]\n"
                  << "  read_only_diagnostic_allowed="
                  << (start_allowed && arm.allowed_unsafe_startup && !config_.servo.send_servo_commands
                          ? "true" : "false") << "\n"
                  << "  controller_simulation_motion_gate_required="
                  << (controllerSimulationMotionRequired(config_) ? "true" : "false") << "\n"
                  << "  controller_simulation_motion_gate_open="
                  << (controllerSimulationMotionGateOpen(config_) ? "true" : "false") << "\n"
                  << "  controller_simulation_diagnostic_override_allowed="
                  << ((controllerSimulationDiagnosticsSuspectGateOpen(config_) ||
                       controllerSimulationInitErrorGateOpen(config_)) ? "true" : "false") << "\n"
                  << "  controller_simulation_diagnostics_suspect_override_allowed="
                  << (controllerSimulationDiagnosticsSuspectGateOpen(config_) ? "true" : "false") << "\n"
                  << "  controller_simulation_init_error_override_allowed="
                  << (controllerSimulationInitErrorGateOpen(config_) ? "true" : "false") << "\n"
                  << "  controller_simulation_not_activated_override_allowed="
                  << (controllerSimulationNotActivatedGateOpen(config_) ? "true" : "false") << "\n"
                  << "  controller_simulation_diagnostic_override_active="
                  << ((controller_sim_diagnostics_suspect_override_active ||
                       controller_sim_init_error_override_active) ? "true" : "false") << "\n"
                  << "  controller_simulation_not_activated_override_active="
                  << (controller_sim_not_activated_override_active ? "true" : "false") << "\n"
                  << "  send_servo_commands="
                  << (config_.servo.send_servo_commands ? "true" : "false") << "\n";
        if (controller_sim_diagnostics_suspect_override_active) {
            std::cerr << "[WARN] rbpodo controller-simulation diagnostics-suspect startup "
                      << "override active for " << name
                      << "; physical_motion_expected=false.\n";
        } else if (controller_sim_init_error_override_active) {
            std::cerr << "[WARN] rbpodo controller-simulation init-error startup "
                      << "override active for " << name
                      << "; physical_motion_expected=false.\n";
        } else if (start_allowed && arm.allowed_unsafe_startup) {
            std::cerr << "[WARN] read-only diagnostic startup allowed unsafe robot state for "
                      << name << "; motion remains suppressed.\n";
        }
    };

    log_arm("left", left, validation.left);
    log_arm("right", right, validation.right);
    if (validation.motion_ready &&
        controllerSimulationMotionRequired(config_) &&
        !controllerSimulationMotionGateOpen(config_)) {
        std::cerr << "[ERROR] controller-simulation motion gate closed; refusing rbpodo "
                     "controller-simulation benchmark. Required: operation_mode=simulation, "
                     "servo.allow_controller_simulation_motion=true.\n";
    }
}

void DualArmServoLoop::storeStartupValidation(const StartupValidationSnapshot& validation) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    startup_validation_ = validation;
    latest_snapshot_.startup_validation = validation;
}

bool DualArmServoLoop::isValidJointState(const RobotState& state) const {
    if (state.connection_state != RobotConnectionState::Connected) return false;
    if (!state.has_valid_joint_state) return false;
    if (state.has_error && !controllerSimulationDiagnosticStateAllowed(config_, state)) {
        return false;
    }
    for (int i = 0; i < kDof; ++i) {
        const double q = state.q_actual_deg[i];
        if (!std::isfinite(q)) return false;
        if (q < config_.safety.q_min_deg[i] || q > config_.safety.q_max_deg[i]) return false;
    }
    return true;
}

bool DualArmServoLoop::isValidRobotStateForStartup(const RobotState& state) const {
    return validateStartupArm(state).motion_ready;
}

void DualArmServoLoop::clearLatchedCartesianTargets() {
    left_latched_cartesian_target_ = LatchedCartesianTarget{};
    right_latched_cartesian_target_ = LatchedCartesianTarget{};
    left_cartesian_servo_path_ = CartesianServoPathState{};
    right_cartesian_servo_path_ = CartesianServoPathState{};
    left_last_cartesian_solve_ = CartesianSolveTelemetry{};
    right_last_cartesian_solve_ = CartesianSolveTelemetry{};
}

void DualArmServoLoop::clearLatchedCartesianTarget(ArmId arm_id) {
    if (arm_id == ArmId::Left) {
        left_latched_cartesian_target_ = LatchedCartesianTarget{};
        left_cartesian_servo_path_ = CartesianServoPathState{};
        left_last_cartesian_solve_ = CartesianSolveTelemetry{};
    } else {
        right_latched_cartesian_target_ = LatchedCartesianTarget{};
        right_cartesian_servo_path_ = CartesianServoPathState{};
        right_last_cartesian_solve_ = CartesianSolveTelemetry{};
    }
}

void DualArmServoLoop::mergeAbcTelemetry(
    CartesianSolveTelemetry& solve, const AbcTelemetry& abc, int output_ma_window) {
    solve.smd_active = abc.smd_active;
    solve.tcp_target_profile = abc.tcp_target_profile;
    solve.tcp_target_profile_found = abc.tcp_target_profile_found;
    solve.smd_profile_nf_linear_hz = abc.smd_profile.natural_frequency_linear_hz;
    solve.smd_profile_nf_angular_hz = abc.smd_profile.natural_frequency_angular_hz;
    solve.smd_profile_velocity_feedforward = abc.smd_profile.velocity_feedforward;
    solve.smd_profile_max_linear_velocity_m_s = abc.smd_profile.max_linear_velocity_m_s;
    solve.smd_profile_max_linear_accel_m_s2 = abc.smd_profile.max_linear_accel_m_s2;
    solve.smd_profile_max_angular_velocity_rad_s = abc.smd_profile.max_angular_velocity_rad_s;
    solve.smd_profile_max_angular_accel_rad_s2 = abc.smd_profile.max_angular_accel_rad_s2;
    solve.smd_profile_max_goal_lead_m = abc.max_smd_goal_lead_m;
    solve.smd_profile_max_goal_lead_rad = abc.max_smd_goal_lead_rad;
    solve.smd_goal_stand = abc.smd_goal_stand;
    solve.smd_ref_stand = abc.smd_ref_stand;
    const SmdStepInfo& info = abc.smd_step_info;
    solve.smd_velocity_feedforward_used = info.velocity_feedforward_used;
    solve.smd_linear_velocity_clipped = info.linear_velocity_clipped;
    solve.smd_linear_accel_clipped = info.linear_accel_clipped;
    solve.smd_angular_velocity_clipped = info.angular_velocity_clipped;
    solve.smd_angular_accel_clipped = info.angular_accel_clipped;
    solve.smd_goal_linear_velocity_ff_clipped = info.goal_linear_velocity_ff_clipped;
    solve.smd_goal_angular_velocity_ff_clipped = info.goal_angular_velocity_ff_clipped;
    solve.smd_goal_linear_velocity_norm_m_s = info.goal_linear_velocity.norm();
    solve.smd_goal_angular_velocity_norm_rad_s = info.goal_angular_velocity.norm();
    solve.smd_reanchor_count = abc.smd_reanchor_count;
    solve.follower_controller = abc.follower_controller;
    solve.follower_active = abc.follower_active;
    solve.follower_wire_seq = abc.follower_wire_seq;
    solve.follower_recv_seq = abc.follower_recv_seq;
    solve.follower_step = abc.follower_step;
    solve.follower_t_in_seg_sec = abc.follower_t_in_seg_sec;
    solve.follower_duration_sec = abc.follower_duration_sec;
    solve.follower_alpha = abc.follower_alpha;
    solve.follower_converged = abc.follower_converged;
    solve.follower_stall = abc.follower_stall;
    solve.follower_corner = abc.follower_corner;
    solve.follower_pf_stand = abc.follower_pf_stand;
    solve.stage_tcp_target_stand = abc.stage_tcp_target_stand;
    solve.follower_output_smd_active = abc.follower_output_smd_active;
    solve.follower_output_smd_lag_m = abc.follower_output_smd_lag_m;
    solve.follower_output_smd_lag_rad = abc.follower_output_smd_lag_rad;
    solve.follower_prefilter_stand = abc.follower_prefilter_stand;
    solve.follower_divergence_pos_m = abc.follower_divergence_pos_m;
    solve.follower_divergence_ang_rad = abc.follower_divergence_ang_rad;
    solve.follower_projection_error_m = abc.follower_projection_error_m;
    solve.follower_projection_error_rad = abc.follower_projection_error_rad;
    solve.follower_projection_error_count = abc.follower_projection_error_count;
    solve.follower_actual_lead_m = abc.follower_actual_lead_m;
    solve.follower_actual_lead_rad = abc.follower_actual_lead_rad;
    solve.follower_actual_lead_error_count = abc.follower_actual_lead_error_count;
    solve.follower_reanchor_count = abc.follower_reanchor_count;
    solve.follower_warm_resume_count = abc.follower_warm_resume_count;
    solve.safety_intervention_recent = abc.safety_intervention_recent;
    solve.cartesian_solve_blocked_recent = abc.cartesian_solve_blocked_recent;
    solve.delta_twist_pending_linear_norm_m = abc.delta_twist_pending_linear_norm_m;
    solve.delta_twist_pending_angular_norm_rad = abc.delta_twist_pending_angular_norm_rad;
    solve.delta_twist_step_delta = abc.delta_twist_step_delta;
    solve.delta_twist_step_linear_norm_m = abc.delta_twist_step_linear_norm_m;
    solve.delta_twist_step_angular_norm_rad = abc.delta_twist_step_angular_norm_rad;
    solve.delta_twist_step_yaw_rad = abc.delta_twist_step_yaw_rad;
    solve.delta_twist_realized_delta = abc.delta_twist_realized_delta;
    solve.delta_twist_realized_linear_norm_m = abc.delta_twist_realized_linear_norm_m;
    solve.delta_twist_realized_angular_norm_rad = abc.delta_twist_realized_angular_norm_rad;
    solve.delta_twist_realized_yaw_rad = abc.delta_twist_realized_yaw_rad;
    solve.delta_twist_realized_linear_ratio = abc.delta_twist_realized_linear_ratio;
    solve.delta_twist_realized_angular_ratio = abc.delta_twist_realized_angular_ratio;
    solve.delta_twist_realized_yaw_ratio = abc.delta_twist_realized_yaw_ratio;
    solve.delta_twist_phase_sec = abc.delta_twist_phase_sec;
    solve.delta_twist_step_kind = abc.delta_twist_step_kind;
    solve.delta_twist_normal_consumed = abc.delta_twist_normal_consumed;
    solve.delta_twist_reserve_consumed = abc.delta_twist_reserve_consumed;
    solve.delta_twist_xi_ref_linear_norm_m_s = abc.delta_twist_xi_ref_linear_norm_m_s;
    solve.delta_twist_xi_ref_angular_norm_rad_s = abc.delta_twist_xi_ref_angular_norm_rad_s;
    solve.delta_twist_xi_cmd_linear_norm_m_s = abc.delta_twist_xi_cmd_linear_norm_m_s;
    solve.delta_twist_xi_cmd_angular_norm_rad_s = abc.delta_twist_xi_cmd_angular_norm_rad_s;
    solve.delta_twist_saturated = abc.delta_twist_saturated;
    solve.delta_twist_lead_linear_norm_m = abc.delta_twist_lead_linear_norm_m;
    solve.delta_twist_lead_angular_norm_rad = abc.delta_twist_lead_angular_norm_rad;
    solve.delta_twist_feedback_source = abc.delta_twist_feedback_source;
    solve.delta_twist_pending_clamped = abc.delta_twist_pending_clamped;
    solve.delta_twist_residual_cleared_on_frame = abc.delta_twist_residual_cleared_on_frame;
    solve.delta_twist_min_time_to_go_used = abc.delta_twist_min_time_to_go_used;
    solve.delta_twist_lin_feedback_cos = abc.delta_twist_lin_feedback_cos;
    solve.delta_twist_ang_feedback_cos = abc.delta_twist_ang_feedback_cos;
    solve.delta_twist_xi_ref_clamped_norm = abc.delta_twist_xi_ref_clamped_norm;
    solve.delta_twist_xi_cmd_clamped_norm = abc.delta_twist_xi_cmd_clamped_norm;
    solve.delta_twist_frame_rows = abc.delta_twist_frame_rows;
    solve.delta_twist_normal_budget = abc.delta_twist_normal_budget;
    solve.delta_twist_total_budget = abc.delta_twist_total_budget;
    solve.delta_twist_steps_remaining = abc.delta_twist_steps_remaining;
    solve.delta_twist_clamp_mask = abc.delta_twist_clamp_mask;
    solve.delta_twist_accel_cmd = abc.delta_twist_accel_cmd;
    solve.output_ma_present = abc.output_ma_present;
    solve.output_ma_window = output_ma_window;
    if (abc.output_ma_present) {
        solve.q_target_before_output_ma_deg = abc.q_target_before_output_ma_deg;
        solve.q_target_after_output_ma_deg = abc.q_target_after_output_ma_deg;
    }
    if (abc.safety_clamp_present) {
        solve.safety_clamp = abc.safety_clamp;
    }
}

void DualArmServoLoop::pollChunkFrames() {
    if (!chunk_frame_receiver_) return;
    const std::uint64_t recv_seq = chunk_frame_receiver_->latestSeq();
    if (recv_seq == 0 || recv_seq == chunk_frame_cache_recv_seq_) return;
    ChunkFrameReceiver::Frame frame;
    if (!chunk_frame_receiver_->copyLatest(&frame)) return;  // writer busy; retry next tick
    chunk_frame_cache_ = frame;
    // receiver_seq is copied UNDER the same lock as the frame; the atomic above
    // is only the cheap "anything new?" gate.
    chunk_frame_cache_recv_seq_ = frame.receiver_seq;
}

void DualArmServoLoop::resetChunkFollowerEngageWait(ArmId arm_id) {
    if (arm_id == ArmId::Left) {
        left_chunk_engage_waiting_ = false;
        left_chunk_engage_wait_start_sec_ = 0.0;
    } else {
        right_chunk_engage_waiting_ = false;
        right_chunk_engage_wait_start_sec_ = 0.0;
    }
}

void DualArmServoLoop::markSafetyIntervention(ArmId arm_id, uint64_t now_ns) {
    uint64_t& stamp = arm_id == ArmId::Left ? left_safety_intervention_last_ns_
                                            : right_safety_intervention_last_ns_;
    stamp = now_ns;
}

bool DualArmServoLoop::safetyInterventionRecent(ArmId arm_id, uint64_t now_ns) const {
    const uint64_t stamp = arm_id == ArmId::Left ? left_safety_intervention_last_ns_
                                                 : right_safety_intervention_last_ns_;
    if (stamp == 0) return false;
    if (now_ns < stamp) return true;
    return now_ns - stamp <= kFollowerDivergenceExplainWindowNs;
}

void DualArmServoLoop::markCartesianSolveBlocked(ArmId arm_id, uint64_t now_ns) {
    uint64_t& stamp = arm_id == ArmId::Left ? left_cartesian_solve_blocked_last_ns_
                                            : right_cartesian_solve_blocked_last_ns_;
    stamp = now_ns;
}

bool DualArmServoLoop::cartesianSolveBlockedRecent(ArmId arm_id, uint64_t now_ns) const {
    const uint64_t stamp = arm_id == ArmId::Left ? left_cartesian_solve_blocked_last_ns_
                                                 : right_cartesian_solve_blocked_last_ns_;
    if (stamp == 0) return false;
    if (now_ns < stamp) return true;
    return now_ns - stamp <= kFollowerDivergenceExplainWindowNs;
}

void DualArmServoLoop::clearChunkFollowerFaultRequests() {
    left_chunk_follower_fault_request_ = ChunkFollowerFaultRequest{};
    left_chunk_follower_fault_request_.arm = ArmId::Left;
    right_chunk_follower_fault_request_ = ChunkFollowerFaultRequest{};
    right_chunk_follower_fault_request_.arm = ArmId::Right;
}

void DualArmServoLoop::recordChunkFollowerFaultRequest(ArmId arm_id, const std::string& reason) {
    ChunkFollowerFaultRequest& request =
        arm_id == ArmId::Left ? left_chunk_follower_fault_request_ : right_chunk_follower_fault_request_;
    request.active = true;
    request.arm = arm_id;
    request.reason = reason;
}

bool DualArmServoLoop::latchChunkFollowerFaultRequests(
    const RobotState& left_state,
    const RobotState& right_state
) {
    if (!left_chunk_follower_fault_request_.active && !right_chunk_follower_fault_request_.active) {
        return false;
    }

    const auto make_context = [](const ChunkFollowerFaultRequest& request) {
        FaultContext context;
        context.verdict = SafetyVerdict::ChunkFollowerFault;
        context.domain = FaultDomain::SafetyPolicy;
        context.arm = request.arm;
        context.reason = request.reason;
        context.suppress_regular_servo = true;
        return context;
    };

    LatchedDualFaultContext contexts;
    if (left_chunk_follower_fault_request_.active) {
        contexts.left = make_context(left_chunk_follower_fault_request_);
    }
    if (right_chunk_follower_fault_request_.active) {
        contexts.right = make_context(right_chunk_follower_fault_request_);
    }
    contexts.top_level = contexts.left.has_value() ? contexts.left : contexts.right;

    std::string reason = contexts.top_level ? contexts.top_level->reason : "chunk follower fault";
    if (contexts.left.has_value() && contexts.right.has_value()) {
        reason = contexts.left->reason + "; " + contexts.right->reason;
    }
    latchFault(SafetyVerdict::ChunkFollowerFault, reason, left_state, right_state, contexts);

    if (left_chunk_follower_fault_request_.active) {
        resetChunkFollowerEngageWait(ArmId::Left);
    }
    if (right_chunk_follower_fault_request_.active) {
        resetChunkFollowerEngageWait(ArmId::Right);
    }
    clearChunkFollowerFaultRequests();
    return true;
}

ArmCommand DualArmServoLoop::applyChunkFollowerStage(
    ArmControlContext& ctx,
    const ArmCommand& command,
    const TcpPoseTargetProfileConfig& profile,
    const Pose6D& actual_feedback_pose,
    double dt_sec
) {
    return applyChunkFollowerStage(
        ctx.arm, command, profile,
        &ctx.chunk_follower, &ctx.chunk_follower_built,
        &ctx.chunk_submitted_wire_seq, &ctx.chunk_submitted_recv_seq,
        &ctx.pose_track_smd, &ctx.follower_output_smd,
        ctx.mount, ctx.prev_sent_q_deg, actual_feedback_pose, dt_sec);
}

ArmCommand DualArmServoLoop::applyDeltaTwistFollowerStage(
    ArmControlContext& ctx,
    const ArmCommand& command,
    const TcpPoseTargetProfileConfig& profile,
    const Pose6D& actual_feedback_pose,
    const Pose6D& execution_feedback_pose,
    double dt_sec
) {
    return applyDeltaTwistFollowerStage(
        ctx.arm, command, profile,
        &ctx.delta_twist_follower, &ctx.chunk_follower_built,
        &ctx.chunk_submitted_wire_seq, &ctx.chunk_submitted_recv_seq,
        &ctx.pose_track_smd,
        ctx.mount, ctx.prev_sent_q_deg,
        actual_feedback_pose, execution_feedback_pose, dt_sec);
}

ArmCommand DualArmServoLoop::applyChunkFollowerStage(
    ArmId arm_id,
    const ArmCommand& command,
    const TcpPoseTargetProfileConfig& profile,
    control::CartesianChunkFollower* follower,
    RuckigFollowerConfig* built_cfg,
    std::uint64_t* submitted_wire_seq,
    std::uint64_t* submitted_recv_seq,
    SmdPoseTracker* smd_tracker,
    control::FollowerOutputSmd* output_smd,
    const ArmMountConfig& mount,
    const JointArray& previous_sent_q_deg,
    const Pose6D& actual_feedback_pose,
    double dt_sec
) {
    const RuckigFollowerConfig& rf = profile.ruckig_follower;
    const bool delta_preview = rf.controller == RuckigFollowerController::DeltaPreview;
    AbcTelemetry& abc = arm_id == ArmId::Left ? left_abc_telemetry_ : right_abc_telemetry_;
    // Reset follower telemetry each tick; re-filled below when the follower drives.
    abc.follower_controller = delta_preview ? "delta_preview" : "ruckig_waypoint";
    abc.follower_active = false;
    abc.follower_wire_seq = 0;
    abc.follower_recv_seq = 0;
    abc.follower_step = -1;
    abc.follower_t_in_seg_sec = 0.0;
    abc.follower_duration_sec = 0.0;
    abc.follower_alpha = 1.0;
    abc.follower_converged = false;
    abc.follower_stall = false;
    abc.follower_corner = false;
    abc.follower_pf_stand.reset();
    abc.stage_tcp_target_stand.reset();
    abc.follower_output_smd_active = false;
    abc.follower_output_smd_lag_m = 0.0;
    abc.follower_output_smd_lag_rad = 0.0;
    abc.follower_prefilter_stand.reset();
    abc.follower_divergence_pos_m = 0.0;
    abc.follower_divergence_ang_rad = 0.0;
    abc.follower_projection_error_m = 0.0;
    abc.follower_projection_error_rad = 0.0;
    abc.follower_projection_error_count = 0;
    abc.follower_actual_lead_m = 0.0;
    abc.follower_actual_lead_rad = 0.0;
    abc.follower_actual_lead_error_count = 0;
    abc.delta_twist_pending_linear_norm_m = 0.0;
    abc.delta_twist_pending_angular_norm_rad = 0.0;
    abc.delta_twist_step_delta = Vec6{};
    abc.delta_twist_step_linear_norm_m = 0.0;
    abc.delta_twist_step_angular_norm_rad = 0.0;
    abc.delta_twist_step_yaw_rad = 0.0;
    abc.delta_twist_realized_delta = Vec6{};
    abc.delta_twist_realized_linear_norm_m = 0.0;
    abc.delta_twist_realized_angular_norm_rad = 0.0;
    abc.delta_twist_realized_yaw_rad = 0.0;
    abc.delta_twist_realized_linear_ratio = 1.0;
    abc.delta_twist_realized_angular_ratio = 1.0;
    abc.delta_twist_realized_yaw_ratio = 1.0;
    abc.delta_twist_phase_sec = 0.0;
    abc.delta_twist_step_kind = 0;
    abc.delta_twist_normal_consumed = 0;
    abc.delta_twist_reserve_consumed = 0;
    abc.delta_twist_xi_ref_linear_norm_m_s = 0.0;
    abc.delta_twist_xi_ref_angular_norm_rad_s = 0.0;
    abc.delta_twist_xi_cmd_linear_norm_m_s = 0.0;
    abc.delta_twist_xi_cmd_angular_norm_rad_s = 0.0;
    abc.delta_twist_saturated = false;
    abc.delta_twist_lead_linear_norm_m = 0.0;
    abc.delta_twist_lead_angular_norm_rad = 0.0;
    abc.delta_twist_feedback_source = 0;
    abc.delta_twist_pending_clamped = false;
    abc.delta_twist_residual_cleared_on_frame = false;
    abc.delta_twist_min_time_to_go_used = false;
    abc.delta_twist_lin_feedback_cos = 1.0;
    abc.delta_twist_ang_feedback_cos = 1.0;
    abc.delta_twist_xi_ref_clamped_norm = false;
    abc.delta_twist_xi_cmd_clamped_norm = false;
    abc.delta_twist_frame_rows = 0;
    abc.delta_twist_normal_budget = 0;
    abc.delta_twist_total_budget = 0;
    abc.delta_twist_steps_remaining = 0;
    abc.delta_twist_clamp_mask = 0;
    abc.delta_twist_accel_cmd = Vec6{};
    const uint64_t now_ns = last_loop_start_ns_ != 0 ? last_loop_start_ns_ : nowSteadyNs();
    const bool intervention_recent = safetyInterventionRecent(arm_id, now_ns);
    // A refused Cartesian solve holds this arm at its previous sent joints exactly as a
    // safety clamp does, but it is stamped by the Cartesian stage, not by applySafety.
    // The follower has to freeze its plan for BOTH, or it keeps integrating deltas
    // against a stationary robot. Measured 2026-08-25 on five pi0.5 rollouts: J3 pinned
    // at its +/-150 deg elbow limit -> IkFailed/ArmedHold on ~85% of ticks (the arm stops
    // dead) -> actual_lead grew 1 mm -> 15-23 mm / 4.1-4.4 deg in 1-5 s and every run
    // ended in delta_preview_actual_lead_fault. safety_intervention_recent was 0
    // throughout, so the ROI/floor plan-freeze below never fired.
    const bool solve_blocked_recent = cartesianSolveBlockedRecent(arm_id, now_ns);
    const bool command_refused_recent = intervention_recent || solve_blocked_recent;
    std::uint64_t& reanchor_count = arm_id == ArmId::Left
        ? left_chunk_follower_reanchor_count_
        : right_chunk_follower_reanchor_count_;
    std::uint64_t& warm_resume_count = arm_id == ArmId::Left
        ? left_chunk_follower_warm_resume_count_
        : right_chunk_follower_warm_resume_count_;
    abc.follower_reanchor_count = reanchor_count;
    abc.follower_warm_resume_count = warm_resume_count;
    abc.safety_intervention_recent = intervention_recent;
    abc.cartesian_solve_blocked_recent = solve_blocked_recent;
    const bool fault_policy = rf.fallback_policy == RuckigFollowerFallbackPolicy::Fault;
    const bool was_active = follower->active();
    std::string transition_reason;
    const auto seq_labels = [](std::uint64_t wire_seq, std::uint64_t recv_seq) {
        std::ostringstream os;
        os << "wire_seq=" << wire_seq << " recv_seq=" << recv_seq;
        return os.str();
    };
    const auto diag_seq_labels = [&seq_labels](const control::FollowerDiag& diag) {
        return seq_labels(diag.seg_wire_seq, diag.seg_recv_seq);
    };
    const auto with_stage_telemetry = [&](ArmCommand out) {
        if (out.mode == ControlMode::TcpPoseTarget && out.has_tcp_target) {
            abc.stage_tcp_target_stand = out.tcp_target_stand;
        }
        return out;
    };
    const auto log_transition = [&]() {
        if (follower->active() == was_active) return;
        // Rare (engage / watchdog / divergence / mode change) — safe to print
        // from the loop, mirrors the existing throttled-WARN practice.
        std::cout << "[chunk_follower] " << (arm_id == ArmId::Left ? "left" : "right")
                  << (follower->active() ? " ENGAGED" : " disengaged")
                  << (!transition_reason.empty() ? " (" : "") << transition_reason
                  << (!transition_reason.empty() ? ")" : "") << "\n";
    };
    const auto smd_fallback = [&]() {
        output_smd->deactivate();
        log_transition();
        return with_stage_telemetry(applyPoseTrackSmd(
            command, profile.pose_track_smd, smd_tracker, kinematics_, mount,
            previous_sent_q_deg, dt_sec));
    };
    if (rf.enable && chunk_frame_receiver_ && command.mode == ControlMode::Hold &&
        follower->active()) {
        // The 2026-07-18 12:44 stream reached this path with 10--360 ms Hold
        // gaps, far shorter than chunk_feed_timeout_sec, and used to call
        // deactivate() on each of them. Freeze segment/window time;
        // the Hold path owns the robot until a bounded warm resume or expiry.
        const double now_sec = ChunkFrameReceiver::steadyNowSec();
        follower->pauseForHold(now_sec);
        output_smd->deactivate();
        follower->expireHoldPause(now_sec, rf.hold_bounce_resume_sec);
        resetChunkFollowerEngageWait(arm_id);
        smd_tracker->deactivate();
        return with_stage_telemetry(command);
    }
    if (!rf.enable || !chunk_frame_receiver_ || command.mode != ControlMode::TcpPoseTarget ||
        !command.has_tcp_target || !kinematics_) {
        // Not in the follower regime: deactivate (drops the stale window) and run
        // the legacy SMD path with identical semantics. Per-arm mode changes
        // that reach this stage cannot leave the follower attached to an old
        // streaming command.
        transition_reason = "mode/enable";
        follower->deactivate();
        output_smd->deactivate();
        resetChunkFollowerEngageWait(arm_id);
        return smd_fallback();
    }
    // Re-point the follower at the active profile's params when they change.
    // In-place (keeps the prewarmed ruckig OTG): cheap enough for the RT tick.
    if (ruckigFollowerConfigChanged(*built_cfg, rf)) {
        follower->reconfigure(makeChunkFollowerConfig(rf));
        *output_smd = control::FollowerOutputSmd(rf.output_smd);
        *built_cfg = rf;
        *submitted_wire_seq = 0;
        *submitted_recv_seq = 0;
    }
    // Live reference = FK of the previously SENT joints (same anchor discipline
    // as the SMD path), so the engage-wait hold is a fixed point: hold target ==
    // last sent pose. Nothing is added to this target downstream, so it needs no
    // correction term.
    Pose6D reference = kinematics_->computeTcpStand(arm_id, previous_sent_q_deg, mount);
    const auto hold_at_reference = [&]() {
        output_smd->deactivate();
        ArmCommand smoothed = command;
        smoothed.tcp_target_stand = reference;
        return smoothed;
    };
    // Safety-layer hold: the floor/ROI/self-collision stage clamps this arm to its previous
    // sent joints, but that stage runs AFTER command generation, so the follower never learned
    // the command was refused and kept integrating deltas against a stationary robot. Measured
    // on servo_log_20260729_165037.csv: eight RoiViolation episodes (right gripper tip crossing
    // roi_box.max_m y=-0.150) drove actual_lead 1.3 mm -> 40.7 mm / 4.5 deg, ending the rollout
    // in delta_preview_actual_lead_fault, and every time the verdict flipped back to Ok the arm
    // lunged to catch up the ran-ahead plan. Freezing the plan here does NOT stop the robot any
    // earlier -- the safety layer already stopped it -- it only stops the plan from running away
    // while it is stopped. Same pause/expire pair the Hold path uses, so the bounded-grace
    // deactivation policy is unchanged (and prevents a deadlock if the block never clears).
    if (command_refused_recent && follower->active()) {
        const double safety_hold_now_sec = ChunkFrameReceiver::steadyNowSec();
        follower->pauseForHold(safety_hold_now_sec);
        output_smd->deactivate();
        follower->expireHoldPause(safety_hold_now_sec, rf.hold_bounce_resume_sec);
        transition_reason = intervention_recent ? "safety intervention hold"
                                                : "cartesian solve blocked hold";
    } else if (follower->holdPaused()) {
        // Warm resume always re-seeds the output stage from the latest sent pose;
        // the preserved p/v/a belongs only to the pre-filter follower.
        output_smd->deactivate();
        const control::HoldResumeResult resume = follower->resumeFromHold(
            reference,
            ChunkFrameReceiver::steadyNowSec(),
            rf.hold_bounce_resume_sec,
            kPoseTrackReanchorPosTolM,
            kPoseTrackReanchorAngTolRad
        );
        if (resume == control::HoldResumeResult::WarmResumed) {
            ++warm_resume_count;
            abc.follower_warm_resume_count = warm_resume_count;
            transition_reason = "warm resume";
        } else if (resume == control::HoldResumeResult::GraceExpired) {
            transition_reason = "Hold grace expired";
        } else if (resume == control::HoldResumeResult::Diverged) {
            transition_reason = "Hold resume divergence";
        }
    }
    if (follower->active()) {
        const control::FollowerDiag& active_diag = follower->diag();
        abc.follower_wire_seq = active_diag.seg_wire_seq;
        abc.follower_recv_seq = active_diag.seg_recv_seq;
        const double pos_err = math::positionDistance(follower->lastPose(), reference);
        const double ang_err = math::orientationDistanceRad(follower->lastPose(), reference);
        abc.follower_divergence_pos_m = pos_err;
        abc.follower_divergence_ang_rad = ang_err;
        const double age = follower->ageSince(ChunkFrameReceiver::steadyNowSec());
        if (age > rf.chunk_feed_timeout_sec) {
            // Feed-liveness watchdog: producer stalled/died. Fall back to the SMD
            // path (which re-anchors at the live reference) until frames resume.
            transition_reason = fault_policy
                ? "feed timeout -> fault " + diag_seq_labels(active_diag)
                : "feed timeout " + diag_seq_labels(active_diag);
            follower->deactivate();
            output_smd->deactivate();
            if (fault_policy) {
                smd_tracker->deactivate();
                std::ostringstream reason;
                reason << toString(arm_id)
                       << " chunk_feed_timeout"
                       << " age_sec=" << age
                       << " timeout_sec=" << rf.chunk_feed_timeout_sec
                       << " " << diag_seq_labels(active_diag);
                recordChunkFollowerFaultRequest(arm_id, reason.str());
                log_transition();
                return with_stage_telemetry(hold_at_reference());
            }
        } else {
            if (pos_err > kPoseTrackReanchorPosTolM || ang_err > kPoseTrackReanchorAngTolRad) {
                // Divergence: the sent target drifted from the follower's plan. In
                // strict fault mode, only an active recent safety intervention can
                // explain it; then reseed the chained state but keep the active window.
                // Otherwise keep the legacy drop/fault or SMD re-anchor behavior.
                if (fault_policy && command_refused_recent) {
                    follower->reanchor(reference);
                    output_smd->deactivate();
                    ++reanchor_count;
                    abc.follower_reanchor_count = reanchor_count;
                    uint64_t& last_log_ns = arm_id == ArmId::Left
                        ? left_chunk_follower_reanchor_log_ns_
                        : right_chunk_follower_reanchor_log_ns_;
                    if (last_log_ns == 0 || now_ns < last_log_ns ||
                        now_ns - last_log_ns >= kFollowerDivergenceReanchorLogPeriodNs) {
                        std::cout << "[chunk_follower] " << toString(arm_id)
                                  << (intervention_recent
                                          ? " divergence re-anchor (safety intervention)"
                                          : " divergence re-anchor (cartesian solve blocked)")
                                  << " pos_err=" << pos_err
                                  << " ang_err=" << ang_err
                                  << " wire_seq=" << active_diag.seg_wire_seq
                                  << " recv_seq=" << active_diag.seg_recv_seq << "\n";
                        last_log_ns = now_ns;
                    }
                } else {
                    transition_reason = fault_policy
                        ? "divergence -> fault " + diag_seq_labels(active_diag)
                        : "divergence re-anchor " + diag_seq_labels(active_diag);
                    follower->deactivate();
                    output_smd->deactivate();
                    if (fault_policy) {
                        smd_tracker->deactivate();
                        std::ostringstream reason;
                        reason << toString(arm_id)
                               << " chunk_follower_divergence"
                               << " pos_err_m=" << pos_err
                               << " pos_tol_m=" << kPoseTrackReanchorPosTolM
                               << " ang_err_rad=" << ang_err
                               << " ang_tol_rad=" << kPoseTrackReanchorAngTolRad
                               << " " << diag_seq_labels(active_diag);
                        recordChunkFollowerFaultRequest(arm_id, reason.str());
                        log_transition();
                        return with_stage_telemetry(hold_at_reference());
                    }
                }
            }
        }
    }
    const bool arm_present =
        arm_id == ArmId::Left ? chunk_frame_cache_.has_left : chunk_frame_cache_.has_right;
    if (chunk_frame_cache_recv_seq_ != 0 &&
        chunk_frame_cache_recv_seq_ != *submitted_recv_seq &&
        arm_present) {
        transition_reason = (delta_preview ? "delta preview frame " : "chunk frame ") +
            seq_labels(chunk_frame_cache_.seq, chunk_frame_cache_.receiver_seq);
        const bool preview_contract_valid =
            chunk_frame_cache_.schema_generation == 3 &&
            chunk_frame_cache_.chunk_metadata_present &&
            chunk_frame_cache_.proprio_valid &&
            (arm_id == ArmId::Left ? chunk_frame_cache_.has_left_delta
                                   : chunk_frame_cache_.has_right_delta);
        if (delta_preview && preview_contract_valid) {
            follower->submitDeltaFrame(toControlChunkFrame(chunk_frame_cache_, arm_id), reference);
        } else if (!delta_preview) {
            follower->submitFrame(toControlChunkFrame(chunk_frame_cache_, arm_id), reference);
        } else {
            follower->deactivate();
            output_smd->deactivate();
            transition_reason = "delta preview rejected v3/proprio contract " +
                seq_labels(chunk_frame_cache_.seq, chunk_frame_cache_.receiver_seq);
        }
        *submitted_wire_seq = chunk_frame_cache_.seq;
        *submitted_recv_seq = chunk_frame_cache_.receiver_seq;
    }
    if (!follower->active()) {
        if (fault_policy) {
            smd_tracker->deactivate();
            const double now_sec = ChunkFrameReceiver::steadyNowSec();
            bool& waiting = arm_id == ArmId::Left ? left_chunk_engage_waiting_ : right_chunk_engage_waiting_;
            double& wait_start_sec =
                arm_id == ArmId::Left ? left_chunk_engage_wait_start_sec_ : right_chunk_engage_wait_start_sec_;
            if (!waiting) {
                waiting = true;
                wait_start_sec = now_sec;
                std::cout << "[chunk_follower] " << toString(arm_id)
                          << " waiting for first chunk frame (fallback_policy=fault "
                          << seq_labels(*submitted_wire_seq, *submitted_recv_seq) << ")\n";
            }
            const double elapsed_sec = now_sec - wait_start_sec;
            if (elapsed_sec > rf.engage_timeout_sec) {
                std::ostringstream reason;
                reason << toString(arm_id)
                       << " chunk_follower_engage_timeout"
                       << " elapsed_sec=" << elapsed_sec
                       << " timeout_sec=" << rf.engage_timeout_sec
                       << " " << seq_labels(*submitted_wire_seq, *submitted_recv_seq);
                recordChunkFollowerFaultRequest(arm_id, reason.str());
            }
            return with_stage_telemetry(hold_at_reference());
        }
        return smd_fallback();  // cold start: no frame yet -> today's behavior
    }
    // The follower drives; keep the SMD inactive so telemetry reads honestly and
    // fallback re-entry re-anchors at the live pose.
    resetChunkFollowerEngageWait(arm_id);
    log_transition();
    smd_tracker->deactivate();
    ArmCommand smoothed = command;
    // Contact-aware following is gone with the F/T stack: no external reaction
    // is supplied, which is the follower's own blind-mode behaviour.
    const Pose6D pre_filter = follower->tick(dt_sec);
    abc.follower_prefilter_stand = pre_filter;
    const Vec6 follower_velocity = follower->currentVelocity().value_or(Vec6{});
    if (rf.output_smd.enable) {
        if (!output_smd->active()) {
            // `reference` is FK of the previous sent target, i.e. the last pose
            // this stage emitted. Never cold-seed from an older filter state.
            output_smd->reset(reference, follower_velocity);
        }
        smoothed.tcp_target_stand =
            output_smd->step(pre_filter, follower_velocity, dt_sec);
        abc.follower_output_smd_active = output_smd->active();
        abc.follower_output_smd_lag_m = output_smd->lagPos();
        abc.follower_output_smd_lag_rad = output_smd->lagAng();
    } else {
        output_smd->deactivate();
        smoothed.tcp_target_stand = pre_filter;
    }
    if (delta_preview) {
        follower->updateActualLead(actual_feedback_pose);
    }
    const control::FollowerDiag& diag = follower->diag();
    abc.follower_projection_error_m = diag.projection_error_m;
    abc.follower_projection_error_rad = diag.projection_error_rad;
    abc.follower_projection_error_count = diag.consecutive_projection_errors;
    abc.follower_actual_lead_m = diag.actual_lead_m;
    abc.follower_actual_lead_rad = diag.actual_lead_rad;
    abc.follower_actual_lead_error_count = diag.consecutive_actual_lead_errors;
    // The projection gate is a plan-FIDELITY alarm (Ruckig cannot reach the
    // requested knots), not a runaway guard: lead, the 50 mm divergence latch,
    // ROI, self-collision, and the FT hard limits own safety. With
    // preview_projection_fault_policy: warn the operator chooses to FOLLOW the
    // model's chunks (smoothed at the configured dynamics) and only log
    // sustained infeasibility instead of latching (2026-07-18 SPEED_SCALE=1.0
    // decision). Lead faults always latch — they measure the ROBOT diverging.
    const bool projection_latches =
        rf.preview_projection_fault_policy != RuckigProjectionFaultPolicy::Warn;
    if (delta_preview && diag.infeasible_fault && !projection_latches) {
        static thread_local double last_projection_warn_sec = 0.0;
        const double now_sec = ChunkFrameReceiver::steadyNowSec();
        if (now_sec - last_projection_warn_sec > 1.0) {
            last_projection_warn_sec = now_sec;
            std::cerr << "[chunk_follower] " << toString(arm_id)
                      << " projection infeasibility (warn policy):"
                      << " err_m=" << diag.projection_error_m
                      << " err_rad=" << diag.projection_error_rad
                      << " count=" << diag.consecutive_projection_errors << "\n";
        }
    }
    if (delta_preview &&
        ((diag.infeasible_fault && projection_latches) || diag.actual_lead_fault)) {
        std::ostringstream reason;
        reason << toString(arm_id)
               << (diag.infeasible_fault && projection_latches
                       ? " delta_preview_projection_fault"
                       : " delta_preview_actual_lead_fault")
               << " projection_error_m=" << diag.projection_error_m
               << " projection_error_rad=" << diag.projection_error_rad
               << " projection_count=" << diag.consecutive_projection_errors
               << " actual_lead_m=" << diag.actual_lead_m
               << " actual_lead_rad=" << diag.actual_lead_rad
               << " actual_lead_count=" << diag.consecutive_actual_lead_errors
               << " " << diag_seq_labels(diag);
        recordChunkFollowerFaultRequest(arm_id, reason.str());
    }
    abc.follower_active = true;
    abc.follower_wire_seq = diag.seg_wire_seq;
    abc.follower_recv_seq = diag.seg_recv_seq;
    abc.follower_step = diag.seg_step_index;
    abc.follower_t_in_seg_sec = follower->tInSegment();
    abc.follower_duration_sec = diag.last_solve.duration;
    abc.follower_alpha = diag.last_solve.alpha;
    abc.follower_converged = diag.last_solve.converged;
    abc.follower_stall = diag.stall;
    abc.follower_corner = diag.last_solve.corner;
    abc.follower_pf_stand = diag.seg_target_stand;
    return with_stage_telemetry(smoothed);
}

ArmCommand DualArmServoLoop::applyDeltaTwistFollowerStage(
    ArmId arm_id,
    const ArmCommand& command,
    const TcpPoseTargetProfileConfig& profile,
    control::DeltaTwistFollower* follower,
    RuckigFollowerConfig* built_cfg,
    std::uint64_t* submitted_wire_seq,
    std::uint64_t* submitted_recv_seq,
    SmdPoseTracker* smd_tracker,
    const ArmMountConfig& mount,
    const JointArray& previous_sent_q_deg,
    const Pose6D& actual_feedback_pose,
    const Pose6D& execution_feedback_pose,
    double dt_sec
) {
    const RuckigFollowerConfig& rf = profile.ruckig_follower;
    AbcTelemetry& abc = arm_id == ArmId::Left ? left_abc_telemetry_ : right_abc_telemetry_;
    abc.follower_controller = "delta_twist";
    abc.follower_active = false;
    abc.follower_wire_seq = 0;
    abc.follower_recv_seq = 0;
    abc.follower_step = -1;
    abc.follower_t_in_seg_sec = 0.0;
    abc.follower_duration_sec = 0.0;
    abc.follower_alpha = 1.0;
    abc.follower_converged = false;
    abc.follower_stall = false;
    abc.follower_corner = false;
    abc.follower_pf_stand.reset();
    abc.stage_tcp_target_stand.reset();
    abc.follower_output_smd_active = false;
    abc.follower_output_smd_lag_m = 0.0;
    abc.follower_output_smd_lag_rad = 0.0;
    abc.follower_prefilter_stand.reset();
    abc.follower_divergence_pos_m = 0.0;
    abc.follower_divergence_ang_rad = 0.0;
    abc.delta_twist_pending_linear_norm_m = 0.0;
    abc.delta_twist_pending_angular_norm_rad = 0.0;
    abc.delta_twist_step_delta = Vec6{};
    abc.delta_twist_step_linear_norm_m = 0.0;
    abc.delta_twist_step_angular_norm_rad = 0.0;
    abc.delta_twist_step_yaw_rad = 0.0;
    abc.delta_twist_realized_delta = Vec6{};
    abc.delta_twist_realized_linear_norm_m = 0.0;
    abc.delta_twist_realized_angular_norm_rad = 0.0;
    abc.delta_twist_realized_yaw_rad = 0.0;
    abc.delta_twist_realized_linear_ratio = 1.0;
    abc.delta_twist_realized_angular_ratio = 1.0;
    abc.delta_twist_realized_yaw_ratio = 1.0;
    abc.delta_twist_phase_sec = 0.0;
    abc.delta_twist_step_kind = 0;
    abc.delta_twist_normal_consumed = 0;
    abc.delta_twist_reserve_consumed = 0;
    abc.delta_twist_xi_ref_linear_norm_m_s = 0.0;
    abc.delta_twist_xi_ref_angular_norm_rad_s = 0.0;
    abc.delta_twist_xi_cmd_linear_norm_m_s = 0.0;
    abc.delta_twist_xi_cmd_angular_norm_rad_s = 0.0;
    abc.delta_twist_saturated = false;
    abc.delta_twist_lead_linear_norm_m = 0.0;
    abc.delta_twist_lead_angular_norm_rad = 0.0;
    abc.delta_twist_feedback_source = 0;
    abc.delta_twist_pending_clamped = false;
    abc.delta_twist_residual_cleared_on_frame = false;
    abc.delta_twist_min_time_to_go_used = false;
    abc.delta_twist_lin_feedback_cos = 1.0;
    abc.delta_twist_ang_feedback_cos = 1.0;
    abc.delta_twist_xi_ref_clamped_norm = false;
    abc.delta_twist_xi_cmd_clamped_norm = false;
    const uint64_t now_ns = last_loop_start_ns_ != 0 ? last_loop_start_ns_ : nowSteadyNs();
    const bool intervention_recent = safetyInterventionRecent(arm_id, now_ns);
    // A refused Cartesian solve holds this arm at its previous sent joints exactly as a
    // safety clamp does, but it is stamped by the Cartesian stage, not by applySafety.
    // The follower has to freeze its plan for BOTH, or it keeps integrating deltas
    // against a stationary robot. Measured 2026-08-25 on five pi0.5 rollouts: J3 pinned
    // at its +/-150 deg elbow limit -> IkFailed/ArmedHold on ~85% of ticks (the arm stops
    // dead) -> actual_lead grew 1 mm -> 15-23 mm / 4.1-4.4 deg in 1-5 s and every run
    // ended in delta_preview_actual_lead_fault. safety_intervention_recent was 0
    // throughout, so the ROI/floor plan-freeze below never fired.
    const bool solve_blocked_recent = cartesianSolveBlockedRecent(arm_id, now_ns);
    const bool command_refused_recent = intervention_recent || solve_blocked_recent;
    std::uint64_t& reanchor_count = arm_id == ArmId::Left
        ? left_chunk_follower_reanchor_count_
        : right_chunk_follower_reanchor_count_;
    abc.follower_reanchor_count = reanchor_count;
    abc.safety_intervention_recent = intervention_recent;
    abc.cartesian_solve_blocked_recent = solve_blocked_recent;
    const bool fault_policy = rf.fallback_policy == RuckigFollowerFallbackPolicy::Fault;
    const bool was_active = follower->active();
    std::string transition_reason;
    const auto seq_labels = [](std::uint64_t wire_seq, std::uint64_t recv_seq) {
        std::ostringstream os;
        os << "wire_seq=" << wire_seq << " recv_seq=" << recv_seq;
        return os.str();
    };
    const auto diag_seq_labels = [&seq_labels](const control::DeltaTwistFollowerDiag& diag) {
        return seq_labels(diag.seg_wire_seq, diag.seg_recv_seq);
    };
    const auto with_stage_telemetry = [&](ArmCommand out) {
        if (out.mode == ControlMode::TcpPoseTarget && out.has_tcp_target) {
            abc.stage_tcp_target_stand = out.tcp_target_stand;
        }
        return out;
    };
    const auto log_transition = [&]() {
        if (follower->active() == was_active) return;
        std::cout << "[chunk_follower] " << (arm_id == ArmId::Left ? "left" : "right")
                  << (follower->active() ? " delta_twist ENGAGED" : " delta_twist disengaged")
                  << (!transition_reason.empty() ? " (" : "") << transition_reason
                  << (!transition_reason.empty() ? ")" : "") << "\n";
    };
    const auto smd_fallback = [&]() {
        log_transition();
        return with_stage_telemetry(applyPoseTrackSmd(
            command, profile.pose_track_smd, smd_tracker, kinematics_, mount,
            previous_sent_q_deg, dt_sec));
    };
    if (!rf.enable || !chunk_frame_receiver_ || command.mode != ControlMode::TcpPoseTarget ||
        !command.has_tcp_target || !kinematics_) {
        transition_reason = "mode/enable";
        follower->deactivate();
        resetChunkFollowerEngageWait(arm_id);
        return smd_fallback();
    }
    if (ruckigFollowerConfigChanged(*built_cfg, rf)) {
        follower->reconfigure(makeDeltaTwistFollowerConfig(rf));
        *built_cfg = rf;
        *submitted_wire_seq = 0;
        *submitted_recv_seq = 0;
    }

    const Pose6D reference = actual_feedback_pose;
    const auto hold_at_reference = [&]() {
        ArmCommand smoothed = command;
        smoothed.tcp_target_stand = reference;
        return smoothed;
    };
    if (follower->active()) {
        const control::DeltaTwistFollowerDiag& active_diag = follower->diag();
        abc.follower_wire_seq = active_diag.seg_wire_seq;
        abc.follower_recv_seq = active_diag.seg_recv_seq;
        const double pos_err = math::positionDistance(follower->lastPose(), reference);
        const double ang_err = math::orientationDistanceRad(follower->lastPose(), reference);
        abc.follower_divergence_pos_m = pos_err;
        abc.follower_divergence_ang_rad = ang_err;
        const double age = follower->ageSince(ChunkFrameReceiver::steadyNowSec());
        if (age > rf.chunk_feed_timeout_sec) {
            transition_reason = fault_policy
                ? "feed timeout -> fault " + diag_seq_labels(active_diag)
                : "feed timeout " + diag_seq_labels(active_diag);
            follower->deactivate();
            if (fault_policy) {
                smd_tracker->deactivate();
                std::ostringstream reason;
                reason << toString(arm_id)
                       << " delta_twist_feed_timeout"
                       << " age_sec=" << age
                       << " timeout_sec=" << rf.chunk_feed_timeout_sec
                       << " " << diag_seq_labels(active_diag);
                recordChunkFollowerFaultRequest(arm_id, reason.str());
                log_transition();
                return with_stage_telemetry(hold_at_reference());
            }
        } else {
            if (pos_err > kPoseTrackReanchorPosTolM || ang_err > kPoseTrackReanchorAngTolRad) {
                if (fault_policy && command_refused_recent) {
                    follower->reanchor(reference);
                    ++reanchor_count;
                    abc.follower_reanchor_count = reanchor_count;
                    uint64_t& last_log_ns = arm_id == ArmId::Left
                        ? left_chunk_follower_reanchor_log_ns_
                        : right_chunk_follower_reanchor_log_ns_;
                    if (last_log_ns == 0 || now_ns < last_log_ns ||
                        now_ns - last_log_ns >= kFollowerDivergenceReanchorLogPeriodNs) {
                        std::cout << "[chunk_follower] " << toString(arm_id)
                                  << (intervention_recent
                                          ? " delta_twist divergence re-anchor (safety intervention)"
                                          : " delta_twist divergence re-anchor (cartesian solve blocked)")
                                  << " pos_err=" << pos_err
                                  << " ang_err=" << ang_err
                                  << " wire_seq=" << active_diag.seg_wire_seq
                                  << " recv_seq=" << active_diag.seg_recv_seq << "\n";
                        last_log_ns = now_ns;
                    }
                } else {
                    transition_reason = fault_policy
                        ? "divergence -> fault " + diag_seq_labels(active_diag)
                        : "divergence re-anchor " + diag_seq_labels(active_diag);
                    follower->deactivate();
                    if (fault_policy) {
                        smd_tracker->deactivate();
                        std::ostringstream reason;
                        reason << toString(arm_id)
                               << " delta_twist_divergence"
                               << " pos_err_m=" << pos_err
                               << " pos_tol_m=" << kPoseTrackReanchorPosTolM
                               << " ang_err_rad=" << ang_err
                               << " ang_tol_rad=" << kPoseTrackReanchorAngTolRad
                               << " " << diag_seq_labels(active_diag);
                        recordChunkFollowerFaultRequest(arm_id, reason.str());
                        log_transition();
                        return with_stage_telemetry(hold_at_reference());
                    }
                }
            }
        }
    }
    const bool arm_present =
        arm_id == ArmId::Left ? chunk_frame_cache_.has_left : chunk_frame_cache_.has_right;
    const bool arm_has_delta =
        arm_id == ArmId::Left ? chunk_frame_cache_.has_left_delta : chunk_frame_cache_.has_right_delta;
    if (chunk_frame_cache_recv_seq_ != 0 &&
        chunk_frame_cache_recv_seq_ != *submitted_recv_seq &&
        arm_present) {
        if (arm_has_delta) {
            transition_reason = "delta chunk frame " +
                seq_labels(chunk_frame_cache_.seq, chunk_frame_cache_.receiver_seq);
            follower->submitFrame(toControlChunkFrame(chunk_frame_cache_, arm_id), execution_feedback_pose);
        } else {
            transition_reason = "chunk frame missing delta " +
                seq_labels(chunk_frame_cache_.seq, chunk_frame_cache_.receiver_seq);
            follower->deactivate();
        }
        *submitted_wire_seq = chunk_frame_cache_.seq;
        *submitted_recv_seq = chunk_frame_cache_.receiver_seq;
    }
    if (!follower->active()) {
        if (fault_policy) {
            smd_tracker->deactivate();
            const double now_sec = ChunkFrameReceiver::steadyNowSec();
            bool& waiting = arm_id == ArmId::Left ? left_chunk_engage_waiting_ : right_chunk_engage_waiting_;
            double& wait_start_sec =
                arm_id == ArmId::Left ? left_chunk_engage_wait_start_sec_ : right_chunk_engage_wait_start_sec_;
            if (!waiting) {
                waiting = true;
                wait_start_sec = now_sec;
                std::cout << "[chunk_follower] " << toString(arm_id)
                          << " waiting for first delta chunk frame (fallback_policy=fault "
                          << seq_labels(*submitted_wire_seq, *submitted_recv_seq) << ")\n";
            }
            const double elapsed_sec = now_sec - wait_start_sec;
            if (elapsed_sec > rf.engage_timeout_sec) {
                std::ostringstream reason;
                reason << toString(arm_id)
                       << " delta_twist_engage_timeout"
                       << " elapsed_sec=" << elapsed_sec
                       << " timeout_sec=" << rf.engage_timeout_sec
                       << " " << seq_labels(*submitted_wire_seq, *submitted_recv_seq);
                recordChunkFollowerFaultRequest(arm_id, reason.str());
            }
            return with_stage_telemetry(hold_at_reference());
        }
        return smd_fallback();
    }

    resetChunkFollowerEngageWait(arm_id);
    log_transition();
    smd_tracker->deactivate();
    ArmCommand smoothed = command;
    follower->setFeedbackPose(
        execution_feedback_pose,
        control::DeltaTwistFeedbackSource::PreviousSentFk
    );
    smoothed.tcp_target_stand = follower->tick(dt_sec);
    const control::DeltaTwistFollowerDiag& diag = follower->diag();
    abc.follower_active = true;
    abc.follower_wire_seq = diag.seg_wire_seq;
    abc.follower_recv_seq = diag.seg_recv_seq;
    abc.follower_step = diag.seg_step_index;
    abc.follower_t_in_seg_sec = follower->tInSegment();
    abc.follower_duration_sec = diag.last_solve.duration;
    abc.follower_alpha = diag.last_solve.alpha;
    abc.follower_converged = diag.last_solve.converged;
    abc.follower_stall = diag.stall;
    abc.follower_corner = diag.last_solve.corner;
    abc.follower_pf_stand = diag.seg_target_stand;
    abc.delta_twist_pending_linear_norm_m = vec6LinearNorm(diag.pending_delta);
    abc.delta_twist_pending_angular_norm_rad = vec6AngularNorm(diag.pending_delta);
    abc.delta_twist_step_delta = diag.step_delta;
    abc.delta_twist_step_linear_norm_m = vec6LinearNorm(diag.step_delta);
    abc.delta_twist_step_angular_norm_rad = vec6AngularNorm(diag.step_delta);
    abc.delta_twist_step_yaw_rad = diag.step_delta.rz;
    abc.delta_twist_realized_delta = diag.realized_delta;
    abc.delta_twist_realized_linear_norm_m = vec6LinearNorm(diag.realized_delta);
    abc.delta_twist_realized_angular_norm_rad = vec6AngularNorm(diag.realized_delta);
    abc.delta_twist_realized_yaw_rad = diag.realized_delta.rz;
    abc.delta_twist_realized_linear_ratio = diag.realized_linear_ratio;
    abc.delta_twist_realized_angular_ratio = diag.realized_angular_ratio;
    abc.delta_twist_realized_yaw_ratio = diag.realized_yaw_ratio;
    abc.delta_twist_phase_sec = follower->tInSegment();
    abc.delta_twist_step_kind = deltaTwistStepKind(diag.step_phase);
    abc.delta_twist_normal_consumed = diag.normal_consumed;
    abc.delta_twist_reserve_consumed = diag.reserve_consumed;
    abc.delta_twist_xi_ref_linear_norm_m_s = vec6LinearNorm(diag.xi_ref);
    abc.delta_twist_xi_ref_angular_norm_rad_s = vec6AngularNorm(diag.xi_ref);
    abc.delta_twist_xi_cmd_linear_norm_m_s = vec6LinearNorm(diag.xi_cmd);
    abc.delta_twist_xi_cmd_angular_norm_rad_s = vec6AngularNorm(diag.xi_cmd);
    abc.delta_twist_saturated = diag.saturated;
    abc.delta_twist_lead_linear_norm_m = vec6LinearNorm(diag.lead_delta);
    abc.delta_twist_lead_angular_norm_rad = vec6AngularNorm(diag.lead_delta);
    abc.delta_twist_feedback_source = diag.feedback_source;
    abc.delta_twist_pending_clamped = diag.pending_clamped;
    abc.delta_twist_residual_cleared_on_frame = diag.residual_cleared_on_frame;
    abc.delta_twist_min_time_to_go_used = diag.min_time_to_go_used;
    abc.delta_twist_lin_feedback_cos = diag.lin_feedback_cos;
    abc.delta_twist_ang_feedback_cos = diag.ang_feedback_cos;
    abc.delta_twist_xi_ref_clamped_norm = diag.xi_ref_clamped_norm;
    abc.delta_twist_xi_cmd_clamped_norm = diag.xi_cmd_clamped_norm;
    abc.delta_twist_frame_rows = diag.frame_rows;
    abc.delta_twist_normal_budget = diag.normal_budget;
    abc.delta_twist_total_budget = diag.total_budget;
    abc.delta_twist_steps_remaining = diag.steps_remaining;
    abc.delta_twist_clamp_mask = diag.clamp_mask;
    abc.delta_twist_accel_cmd = diag.accel_cmd;
    return with_stage_telemetry(smoothed);
}

namespace {
// Stamp one arm's Cartesian solve telemetry and refresh a running TcpLinearMove's
// lease. Per-arm by construction: reads only this arm's result/selection.
template <typename Ctx, typename Result, typename Selection, typename Command>
void stampCartesianSolve(Ctx& ctx, const Result& result, const Selection& selection,
                         const Command& command) {
    ctx.last_cartesian_solve = result.telemetry;
    ctx.last_cartesian_solve.cartesian_servo_state_source =
        selection.context.servo_state_source;
    ctx.last_cartesian_solve.cartesian_divergence_source =
        selection.context.divergence_source;
    ctx.last_cartesian_solve.q_reference_for_servo_valid =
        selection.context.q_reference_for_servo_valid;
    const ArmCommand& arm_cmd = ctx.arm == ArmId::Left ? command.left : command.right;
    if (arm_cmd.mode == ControlMode::TcpLinearMove && ctx.cartesian_servo_path.active) {
        ctx.cartesian_servo_path.lease_enforced = command.lease.enforce_lease;
        ctx.cartesian_servo_path.lease_expires_time_ns = command.lease.expires_time_ns;
    }
}
}  // namespace

ServoTarget DualArmServoLoop::computeServoTarget(
    const RobotState& left_state,
    const RobotState& right_state,
    const DualArmCommand& command,
    double dt_sec,
    SafetyVerdict* command_verdict
) {
    if (command_verdict) *command_verdict = SafetyVerdict::Ok;
    clearChunkFollowerFaultRequests();
    // Per-arm state reached through ArmControlContext for the whole function, so
    // the body no longer names an arm: it is already in the shape a per-arm
    // control thread needs (one context instead of an array of two).
    ArmControlContext arm_ctx[2] = {armContext(ArmId::Left), armContext(ArmId::Right)};
    const RobotState* arm_state[2] = {&left_state, &right_state};
    ++cross_arm_tick_;
    for (ArmControlContext& ctx : arm_ctx) {
        AbcTelemetry& abc = ctx.abc_telemetry;
        abc.follower_output_smd_active = false;
        abc.follower_output_smd_lag_m = 0.0;
        abc.follower_output_smd_lag_rad = 0.0;
        abc.follower_prefilter_stand.reset();
    }
    ServoTarget target;
    const bool synthetic_hold = isSyntheticHoldCommand(command);
    const auto clear_linear_path = [](ArmControlContext& ctx) {
        ctx.cartesian_servo_path = CartesianServoPathState{};
        ctx.last_cartesian_solve = CartesianSolveTelemetry{};
    };
    const auto clear_left_linear_path = [&]() { clear_linear_path(arm_ctx[0]); };
    const auto clear_right_linear_path = [&]() { clear_linear_path(arm_ctx[1]); };
    for (int i = 0; i < 2; ++i) {
        if (arm_ctx[i].cartesian_servo_path.active && !isValidJointState(*arm_state[i])) {
            clear_linear_path(arm_ctx[i]);
        }
    }
    // TcpLinearMove is a FINITE, bounded path (duration_sec <= linear_move.max_duration_sec).
    // Once it is running, let it drive to completion even if the (one-shot) command's
    // lease lapses — a single click must always reach the target. The deadman still
    // applies once the path is DONE (cleared below), and an explicit command-mode change,
    // a fault, or an invalid joint state still abort it immediately above/below; E-stop
    // latches regardless. So only a FINISHED path is torn down on lease expiry.
    for (int i = 0; i < 2; ++i) {
        if (arm_ctx[i].cartesian_servo_path.done &&
            linearPathLeaseExpired(arm_ctx[i].cartesian_servo_path, command.host_time_ns)) {
            clear_linear_path(arm_ctx[i]);
        }
    }
    if (!synthetic_hold) {
        if (command.left.mode != ControlMode::TcpLinearMove) {
            clear_left_linear_path();
        }
        if (command.right.mode != ControlMode::TcpLinearMove) {
            clear_right_linear_path();
        }
    }

    for (ArmControlContext& ctx : arm_ctx) {
        const CartesianSolveTelemetry previous = ctx.last_cartesian_solve;
        ctx.last_cartesian_solve =
            retainCompletedPathTelemetry(ctx.cartesian_servo_path, previous)
                ? previous
                : CartesianSolveTelemetry{};
    }

    if (isCommandModeMissingPayload(command.left) || isCommandModeMissingPayload(command.right)) {
        if (command_verdict) *command_verdict = SafetyVerdict::InvalidCommand;
        for (ArmControlContext& ctx : arm_ctx) ctx.follower_output_smd.deactivate();
        target.left_q_target_deg = arm_ctx[0].prev_sent_q_deg;
        target.right_q_target_deg = arm_ctx[1].prev_sent_q_deg;
        return target;
    }

    const auto continue_linear = [&](const ArmControlContext& ctx) {
        return synthetic_hold && ctx.cartesian_servo_path.active && !ctx.cartesian_servo_path.done;
    };
    const bool continue_left_linear = continue_linear(arm_ctx[0]);
    const bool continue_right_linear = continue_linear(arm_ctx[1]);
    DualArmCommand effective_command = command;
    if (continue_left_linear) {
        effective_command.left =
            linearMoveContinuationCommand(command.left, arm_ctx[0].cartesian_servo_path);
    }
    if (continue_right_linear) {
        effective_command.right =
            linearMoveContinuationCommand(command.right, arm_ctx[1].cartesian_servo_path);
    }

    const ControlMode arm_raw_mode[2] = {command.left.mode, command.right.mode};

    // effective_command is final here.
    const ControlMode arm_effective_mode[2] =
        {effective_command.left.mode, effective_command.right.mode};

    // Already arm-agnostic in shape; taking the context makes it so by type, so
    // a per-arm thread can call it with its own arm and nothing else.
    const auto pause_chunk_follower_for_hold = [this](
        ArmControlContext& ctx,
        ControlMode raw_mode
    ) {
        const ArmId arm_id = ctx.arm;
        control::CartesianChunkFollower& follower = ctx.chunk_follower;
        control::FollowerOutputSmd& output_smd = ctx.follower_output_smd;
        const RuckigFollowerConfig& built_config = ctx.chunk_follower_built;
        // Hold/mode-exit owns the sent target. A later warm resume may preserve
        // follower p/v/a, but the output conditioner must re-seed from that new
        // last-sent target rather than continue from a stale pose.
        output_smd.deactivate();
        if (raw_mode != ControlMode::Hold || !follower.active() ||
            !built_config.enable ||
            built_config.controller == RuckigFollowerController::DeltaTwist) {
            follower.deactivate();
            resetChunkFollowerEngageWait(arm_id);
            return false;
        }
        const double now_sec = ChunkFrameReceiver::steadyNowSec();
        follower.pauseForHold(now_sec);
        if (follower.expireHoldPause(now_sec, built_config.hold_bounce_resume_sec)) {
            resetChunkFollowerEngageWait(arm_id);
            return false;
        }
        return true;
    };

    if (isCartesianMode(effective_command.left.mode) || isCartesianMode(effective_command.right.mode)) {
        bool cartesian_available =
            config_.cartesian_control.enable &&
            config_.kinematics.enable &&
            config_.kinematics.ik.enable &&
            kinematics_ != nullptr;
        std::string left_unavailable_reason = "cartesian_control_unavailable_disabled";
        std::string right_unavailable_reason = "cartesian_control_unavailable_disabled";
        if (cartesian_available) {
            for (const ArmCommand* arm_command : {&effective_command.left, &effective_command.right}) {
                if (!isCartesianMode(arm_command->mode)) continue;
                const CartesianAvailability availability =
                    cartesianAvailabilityForArm(config_, *arm_command);
                if (!availability.available) {
                    cartesian_available = false;
                    if (arm_command->arm_id == ArmId::Left) {
                        left_unavailable_reason = availability.reason;
                    } else {
                        right_unavailable_reason = availability.reason;
                    }
                }
            }
        }
        if (!cartesian_available) {
            // CROSS-ARM: Cartesian being unavailable is a whole-command condition
            // (missing kinematics / config), so it holds BOTH arms. The per-arm
            // telemetry below still names only the arm it describes.
            if (command_verdict) *command_verdict = SafetyVerdict::CartesianUnavailable;
            const std::string* arm_unavailable_reason[2] =
                {&left_unavailable_reason, &right_unavailable_reason};
            for (int i = 0; i < 2; ++i) {
                arm_ctx[i].follower_output_smd.deactivate();
                if (!isCartesianMode(arm_effective_mode[i])) continue;
                arm_ctx[i].last_cartesian_solve = cartesianUnavailableTelemetry(
                    *arm_state[i],
                    config_.cartesian_control,
                    *arm_unavailable_reason[i]
                );
            }
            target.left_q_target_deg = arm_ctx[0].prev_sent_q_deg;
            target.right_q_target_deg = arm_ctx[1].prev_sent_q_deg;
            return target;
        }

        CartesianController cartesian(
            config_.left_mount,
            config_.right_mount,
            config_.cartesian_control,
            kinematics_
        );
        CartesianServoController cartesian_servo(
            config_.left_mount,
            config_.right_mount,
            config_.cartesian_control,
            kinematics_
        );
        // Tier-2 floor clamp for absolute Cartesian targets: TcpPoseTarget aims
        // at the clamped pose every tick; TcpLinearMove latches its path goal from
        // this command at path start, so clamping here keeps the whole path legal
        // (interpolation between legal start and legal goal stays legal).
        const auto clamp_command_pose = [this](ArmCommand& cmd) {
            if ((cmd.mode == ControlMode::TcpPoseTarget || cmd.mode == ControlMode::TcpLinearMove) &&
                cmd.has_tcp_target) {
                cmd.tcp_target_stand = clampPoseToRoi(clampPoseToFloor(cmd.tcp_target_stand));
            }
        };
        clamp_command_pose(effective_command.left);
        clamp_command_pose(effective_command.right);

        bool left_profile_found = false;
        bool right_profile_found = false;
        const TcpPoseTargetProfileConfig left_tcp_profile = selectTcpPoseTargetProfile(
            config_.cartesian_control,
            effective_command.tcp_target_profile,
            &left_profile_found
        );
        const TcpPoseTargetProfileConfig right_tcp_profile = selectTcpPoseTargetProfile(
            config_.cartesian_control,
            effective_command.tcp_target_profile,
            &right_profile_found
        );
        // Resolve the Cartesian feedback state before advancing any stateful
        // tracker. Physical-real uses q_actual/tcp_actual. An rbpodo control box
        // held in pgmode simulation uses q_target/tcp_ref instead, because its
        // physical encoders intentionally remain still. Missing reference state
        // is fail-closed and must not mutate the chunk/SMD followers.
        const CartesianServoStateSelection left_servo_state =
            selectCartesianServoStateForArm(config_, effective_command.left, left_state);
        const CartesianServoStateSelection right_servo_state =
            selectCartesianServoStateForArm(config_, effective_command.right, right_state);
        // CROSS-ARM: either arm losing its Cartesian servo state holds BOTH.
        // Routed through the published cross-arm status rather than the two local
        // values, so the decision keeps its meaning once each arm runs on its own
        // thread and can only see its own selection directly.
        publishCrossArmStatus(ArmId::Left, left_servo_state.ok);
        publishCrossArmStatus(ArmId::Right, right_servo_state.ok);
        const bool cartesian_state_ok[2] = {
            left_servo_state.ok && peerCartesianServoOk(ArmId::Left),
            right_servo_state.ok && peerCartesianServoOk(ArmId::Right),
        };
        if (!cartesian_state_ok[0] || !cartesian_state_ok[1]) {
            if (command_verdict) *command_verdict = SafetyVerdict::CartesianUnavailable;
            const CartesianServoStateSelection* selection[2] =
                {&left_servo_state, &right_servo_state};
            for (int i = 0; i < 2; ++i) {
                arm_ctx[i].follower_output_smd.deactivate();
                if (selection[i]->ok) continue;
                arm_ctx[i].last_cartesian_solve = cartesianUnavailableTelemetry(
                    *arm_state[i],
                    config_.cartesian_control,
                    selection[i]->reason
                );
                arm_ctx[i].last_cartesian_solve.cartesian_servo_state_source =
                    selection[i]->context.servo_state_source;
                arm_ctx[i].last_cartesian_solve.cartesian_divergence_source =
                    selection[i]->context.divergence_source;
                arm_ctx[i].last_cartesian_solve.q_reference_for_servo_valid =
                    selection[i]->context.q_reference_for_servo_valid;
            }
            target.left_q_target_deg = arm_ctx[0].prev_sent_q_deg;
            target.right_q_target_deg = arm_ctx[1].prev_sent_q_deg;
            return target;
        }
        const TcpPoseTargetProfileConfig* arm_tcp_profile[2] =
            {&left_tcp_profile, &right_tcp_profile};
        for (int i = 0; i < 2; ++i) {
            if (arm_effective_mode[i] == ControlMode::TcpPoseTarget &&
                arm_ctx[i].pose_track_profile_name != arm_tcp_profile[i]->name) {
                arm_ctx[i].pose_track_smd = SmdPoseTracker(arm_tcp_profile[i]->pose_track_smd);
                arm_ctx[i].pose_track_profile_name = arm_tcp_profile[i]->name;
            }
        }
        // Manipulability guard: feed the previous tick's IK min singular value into each
        // SMD so step() scales tracking velocity down near a singularity (config
        // singularity_scale_*). Velocity-only — cannot stall the IK.
        for (ArmControlContext& ctx : arm_ctx) {
            ctx.pose_track_smd.setMinSingular(ctx.last_cartesian_solve.ik_min_singular_value);
        }
        // Chunk-follower stage: pull the newest chunk frame once per tick, then
        // run the selected follower (absolute-waypoint Ruckig or local-delta
        // delta_twist) or the legacy SMD path.
        // applyChunkFollowerStage falls back to applyPoseTrackSmd verbatim when
        // the profile leaves ruckig_follower disabled, so existing profiles are
        // byte-identical in behavior. This stage runs before this tick's
        // applySafety(), so strict divergence explanation intentionally reads the
        // previous safety tick's intervention stamp through a short debounce.
        pollChunkFrames();
        const auto servo_feedback_pose_for_arm = [this](
            const RobotState& state,
            const ArmControlContext& ctx
        ) {
            if (state.has_valid_tcp_pose && state.tcp_stand.has_value()) {
                return *state.tcp_stand;
            }
            if (kinematics_) {
                return kinematics_->computeTcpStand(ctx.arm, ctx.prev_sent_q_deg, ctx.mount);
            }
            return Pose6D{};
        };
        const auto execution_feedback_pose_for_arm = [this](const ArmControlContext& ctx) {
            if (kinematics_) {
                return kinematics_->computeTcpStand(ctx.arm, ctx.prev_sent_q_deg, ctx.mount);
            }
            return Pose6D{};
        };
        const Pose6D left_delta_twist_actual_feedback =
            servo_feedback_pose_for_arm(left_servo_state.state, arm_ctx[0]);
        const Pose6D right_delta_twist_actual_feedback =
            servo_feedback_pose_for_arm(right_servo_state.state, arm_ctx[1]);
        const Pose6D left_delta_twist_execution_feedback =
            execution_feedback_pose_for_arm(arm_ctx[0]);
        const Pose6D right_delta_twist_execution_feedback =
            execution_feedback_pose_for_arm(arm_ctx[1]);
        // Symmetric per-arm dispatch, folded into one body: the follower choice
        // depends only on this arm's own profile.
        const ArmCommand* arm_effective_cmd[2] =
            {&effective_command.left, &effective_command.right};
        const Pose6D* arm_actual_fb[2] =
            {&left_delta_twist_actual_feedback, &right_delta_twist_actual_feedback};
        const Pose6D* arm_exec_fb[2] =
            {&left_delta_twist_execution_feedback, &right_delta_twist_execution_feedback};
        ArmCommand arm_pose_track_command[2];
        for (int i = 0; i < 2; ++i) {
            ArmControlContext& ctx = arm_ctx[i];
            if (arm_tcp_profile[i]->ruckig_follower.controller ==
                RuckigFollowerController::DeltaTwist) {
                ctx.chunk_follower.deactivate();
                ctx.follower_output_smd.deactivate();
                arm_pose_track_command[i] = applyDeltaTwistFollowerStage(
                    ctx, *arm_effective_cmd[i], *arm_tcp_profile[i],
                    *arm_actual_fb[i], *arm_exec_fb[i], dt_sec);
            } else {
                ctx.delta_twist_follower.deactivate();
                arm_pose_track_command[i] = applyChunkFollowerStage(
                    ctx, *arm_effective_cmd[i], *arm_tcp_profile[i],
                    *arm_actual_fb[i], dt_sec);
            }
        }
        ArmCommand& left_pose_track_command = arm_pose_track_command[0];
        ArmCommand& right_pose_track_command = arm_pose_track_command[1];

        // ---- THE FORCE OVERLAY ------------------------------------------------
        // Here, and not earlier or later, for one reason: this is the last point at
        // which the target is a CARTESIAN pose and the first at which it is final.
        // Composing before the follower would have the follower plan around a
        // deviation that the contact is still changing; composing after the IK would
        // mean deviating joints, which is not what a wrench in the tool frame asks
        // for. The deviation is composed onto the followed pose and the ORDINARY
        // Cartesian solve + safety stages run on the result, so IK refusal, the
        // joint clamps, ROI, self-collision and the floor all still own their veto.
        for (int i = 0; i < 2; ++i) {
            const ArmId arm_id = i == 0 ? ArmId::Left : ArmId::Right;
            const bool left_arm = i == 0;
            ArmCommand& track = arm_pose_track_command[i];
            const RobotState& arm_state = left_arm ? left_state : right_state;
            ForceControlTelemetry& tel =
                left_arm ? left_force_control_telemetry_ : right_force_control_telemetry_;
            control::AdmittanceOverlay& overlay = left_arm ? left_overlay_ : right_overlay_;
            std::optional<Pose6D>& hold_nominal =
                left_arm ? left_hold_compliance_nominal_ : right_hold_compliance_nominal_;

            const ForceControlTelemetry blank{};
            tel = blank;
            tel.enabled = config_.force_control.enable;

            std::string reason;
            const bool covered = forceControlCovered(arm_id, track, arm_state, &reason);
            tel.coverage_reason = covered ? "covered" : reason;
            if (!covered) {
                // LEAVING SERVICE FREEZES THE DEVIATION AND DROPS THE MOMENTUM - it
                // does NOT walk the deviation back. Under contact the followed pose is
                // INSIDE the workpiece (the deviation is what was holding the command
                // at the surface), so retiring would command the tool the whole
                // deviation DEEPER. A stop must come to rest at the pose the arm
                // ACHIEVED, which is the composition already on the wire.
                overlay.freeze();
                hold_nominal.reset();
                continue;
            }

            // A plain Hold has no Cartesian nominal, so LATCH one on entry. Re-reading
            // the measured pose every tick would make the nominal follow the deviation
            // and leave the spring nothing to pull back to - the arm would simply walk
            // wherever it was pushed and stay there.
            if (track.mode == ControlMode::Hold) {
                if (!hold_nominal.has_value()) {
                    hold_nominal = *arm_state.tcp_actual_stand;
                    std::cerr << "[INFO] force overlay " << toString(arm_id)
                              << ": Hold made compliant, nominal latched at the measured TCP ["
                              << hold_nominal->x << ", " << hold_nominal->y << ", "
                              << hold_nominal->z << "] m\n";
                }
                track.mode = ControlMode::TcpPoseTarget;
                track.tcp_target_stand = *hold_nominal;
                track.has_tcp_target = true;
            } else {
                hold_nominal.reset();
            }
            if (!track.has_tcp_target) {
                tel.coverage_reason = "no Cartesian target on this tick";
                overlay.freeze();
                continue;
            }
            Pose6D target = track.tcp_target_stand;
            if (applyForceOverlay(arm_id, arm_state, &target)) {
                track.tcp_target_stand = target;
            }
            // FEED THE GATE TO THE PLAN, not just to the law. The overlay makes the
            // arm YIELD; only the gate stops the PLAN from walking further into what
            // it is yielding to. Without this the spring's deviation grows for as
            // long as the stream keeps advancing, and the contact force with it.
            control::ForceGate& gate = left_arm ? left_force_gate_ : right_force_gate_;
            const Wrench6D& gw = tel.wrench_stand;
            const math::Vector3 f_stand(gw.fx, gw.fy, gw.fz);
            const double fn = f_stand.norm();
            // The direction pushing INTO the contact is the NEGATION of the measured
            // reaction: the sensor reports what the environment does to the tool.
            arm_ctx[i].chunk_follower.setAdvanceGate(
                gate.translation(),
                fn > 1e-9 ? math::Vector3(f_stand / fn) : math::Vector3::Zero());
        }
        if (latchChunkFollowerFaultRequests(left_state, right_state) && command_verdict) {
            *command_verdict = SafetyVerdict::ChunkFollowerFault;
        }

        // Patch 4: capture SMD (B-stage) telemetry right after the step, before any
        // later cartesian-solve return path can reset the per-arm solve telemetry.
        // Merged into the published cartesian_solve sample in loopMain.
        auto capture_smd_abc = [](
            const SmdPoseTracker& tracker,
            const TcpPoseTargetProfileConfig& profile,
            bool profile_found,
            AbcTelemetry& abc
        ) {
            abc.smd_active = tracker.active();
            abc.tcp_target_profile = profile.name;
            abc.tcp_target_profile_found = profile_found;
            abc.smd_profile = profile.pose_track_smd;
            abc.max_smd_goal_lead_m = profile.max_smd_goal_lead_m;
            abc.max_smd_goal_lead_rad = profile.max_smd_goal_lead_rad;
            if (tracker.active()) {
                abc.smd_ref_stand = tracker.currentPose();
                abc.smd_goal_stand = tracker.goalPose();
                abc.smd_step_info = tracker.lastStepInfo();
                abc.smd_reanchor_count = tracker.reanchorCount();
            } else {
                abc.smd_ref_stand.reset();
                abc.smd_goal_stand.reset();
                abc.smd_step_info = SmdStepInfo{};
            }
        };
        const bool arm_profile_found[2] = {left_profile_found, right_profile_found};
        for (int i = 0; i < 2; ++i) {
            capture_smd_abc(arm_ctx[i].pose_track_smd, *arm_tcp_profile[i],
                            arm_profile_found[i], arm_ctx[i].abc_telemetry);
        }

        const RunMode left_cartesian_compute_run_mode =
            cartesianComputationRunModeForArm(config_, effective_command.left);
        const RunMode right_cartesian_compute_run_mode =
            cartesianComputationRunModeForArm(config_, effective_command.right);

        // The IK seed and the pose used to form the Cartesian error must describe
        // the same controller state. Physical-real deliberately keeps the
        // feed-forward previous-sent seed. In controller pgmode, however, tcp_ref
        // is derived from q_target and is normally one data-frame behind the last
        // sent target. Combining that delayed pose with the newer sent-joint seed
        // creates an alternating feedback pair at 500 Hz. Seed pgmode IK from the
        // selected reference joints instead; this branch is active only when the
        // explicit controller-simulation reference source is selected.
        const JointArray& left_cartesian_ik_seed_q_deg =
            left_servo_state.context.servo_state_source == "reference"
            ? left_servo_state.state.q_actual_deg
            : arm_ctx[0].prev_sent_q_deg;
        const JointArray& right_cartesian_ik_seed_q_deg =
            right_servo_state.context.servo_state_source == "reference"
            ? right_servo_state.state.q_actual_deg
            : arm_ctx[1].prev_sent_q_deg;

        // The Cartesian branches below bypass TrajectoryFilter::computeJointTarget, so the
        // joint-target SMD would otherwise seed its activation velocity from a stale sample
        // the next time a JointTarget takes over (init brake-before-plan, jog handoff).
        // Record this tick's sent target for the arm(s) that go the Cartesian way.
        for (int i = 0; i < 2; ++i) {
            if (isCartesianMode(arm_effective_mode[i])) {
                arm_ctx[i].traj_filter.observeSentTarget(arm_ctx[i].prev_sent_q_deg);
            }
        }

        // One body for both arms: the solve reads only this arm's command, servo
        // state, IK seed and run mode. Nothing crosses.
        const CartesianServoStateSelection* arm_selection[2] =
            {&left_servo_state, &right_servo_state};
        const JointArray* arm_ik_seed[2] =
            {&left_cartesian_ik_seed_q_deg, &right_cartesian_ik_seed_q_deg};
        const RunMode arm_run_mode[2] =
            {left_cartesian_compute_run_mode, right_cartesian_compute_run_mode};
        const bool arm_continue_linear[2] = {continue_left_linear, continue_right_linear};
        JointArray* arm_target_out[2] =
            {&target.left_q_target_deg, &target.right_q_target_deg};
        const ArmCommand* arm_cmd_raw[2] = {&command.left, &command.right};
        CartesianArmTargetResult arm_cartesian_result[2];
        for (int i = 0; i < 2; ++i) {
            ArmControlContext& ctx = arm_ctx[i];
            arm_cartesian_result[i] =
                arm_effective_mode[i] == ControlMode::TcpLinearMove
                ? cartesian_servo.computeLinearMoveTarget(
                    *arm_effective_cmd[i],
                    arm_selection[i]->state,
                    *arm_ik_seed[i],
                    arm_run_mode[i],
                    dt_sec,
                    arm_continue_linear[i] ? 0 : command.seq,
                    &ctx.cartesian_servo_path,
                    &arm_selection[i]->context
                )
                : isCartesianMode(arm_effective_mode[i])
                ? cartesian.computeArmJointTarget(
                    arm_pose_track_command[i],
                    arm_selection[i]->state,
                    *arm_ik_seed[i],
                    arm_run_mode[i]
                )
                : CartesianArmTargetResult{
                    SafetyVerdict::Ok,
                    ctx.traj_filter.computeJointTarget(
                        *arm_cmd_raw[i], *arm_state[i], ctx.prev_sent_q_deg, dt_sec),
                    "",
                    CartesianSolveTelemetry{}
                };
            *arm_target_out[i] = arm_cartesian_result[i].q_target_deg;
            stampCartesianSolve(ctx, arm_cartesian_result[i], *arm_selection[i], command);
        }
        const CartesianArmTargetResult& left_cartesian_result = arm_cartesian_result[0];
        const CartesianArmTargetResult& right_cartesian_result = arm_cartesian_result[1];

        if (left_cartesian_result.verdict != SafetyVerdict::Ok ||
            right_cartesian_result.verdict != SafetyVerdict::Ok) {
            SafetyVerdict verdict = SafetyVerdict::CartesianUnavailable;
            if (left_cartesian_result.verdict == SafetyVerdict::IkFailed ||
                right_cartesian_result.verdict == SafetyVerdict::IkFailed) {
                verdict = SafetyVerdict::IkFailed;
            } else if (left_cartesian_result.verdict == SafetyVerdict::TrackingError ||
                       right_cartesian_result.verdict == SafetyVerdict::TrackingError) {
                verdict = SafetyVerdict::TrackingError;
            } else if (left_cartesian_result.verdict == SafetyVerdict::InvalidCommand ||
                       right_cartesian_result.verdict == SafetyVerdict::InvalidCommand) {
                verdict = SafetyVerdict::InvalidCommand;
            }
            if (command_verdict && *command_verdict != SafetyVerdict::ChunkFollowerFault) {
                *command_verdict = verdict;
            }
            // Per-arm hold: only the arm whose solve failed is held. (The verdict
            // above is a whole-command aggregate, but the target is not.)
            const CartesianArmTargetResult* arm_result[2] =
                {&left_cartesian_result, &right_cartesian_result};
            JointArray* arm_target[2] =
                {&target.left_q_target_deg, &target.right_q_target_deg};
            const uint64_t solve_block_now_ns =
                last_loop_start_ns_ != 0 ? last_loop_start_ns_ : nowSteadyNs();
            for (int i = 0; i < 2; ++i) {
                if (arm_result[i]->verdict == SafetyVerdict::Ok) continue;
                *arm_target[i] = arm_ctx[i].prev_sent_q_deg;
                arm_ctx[i].follower_output_smd.deactivate();
                // The arm is now held at its previous joints, so the plan the follower
                // is streaming will NOT be executed this tick. Stamp it here (not in the
                // solver) so the follower stage freezes its plan on the next tick instead
                // of running away from a stationary robot.
                markCartesianSolveBlocked(arm_ctx[i].arm, solve_block_now_ns);
            }
        }
        return target;
    }

    // No arm is in Cartesian mode, so applyChunkFollowerStage() did not run.
    // A raw dual Hold reaches this dispatch path directly, and before warm-resume
    // it unconditionally deactivated both followers -- one of the unconditional
    // paths seen in the 2026-07-18 12:44 bounce evidence. Preserve a bounded,
    // frozen chunk only for raw Hold; every other mode switch still drops the
    // streaming state immediately.
    for (int i = 0; i < 2; ++i) {
        pause_chunk_follower_for_hold(arm_ctx[i], arm_raw_mode[i]);
    }
    // Joint-mode fallback: each arm's target comes only from its own filter,
    // state and command -- no cross-arm term.
    const ArmCommand* arm_cmd[2] = {&command.left, &command.right};
    JointArray* arm_out[2] = {&target.left_q_target_deg, &target.right_q_target_deg};
    for (int i = 0; i < 2; ++i) {
        arm_ctx[i].delta_twist_follower.deactivate();
        *arm_out[i] = arm_ctx[i].traj_filter.computeJointTarget(
            *arm_cmd[i], *arm_state[i], arm_ctx[i].prev_sent_q_deg, dt_sec);
    }
    return target;
}

ServoTarget DualArmServoLoop::applySafety(
    const ServoTarget& desired,
    const RobotState& left_state,
    const RobotState& right_state,
    ControlMode left_mode,
    ControlMode right_mode,
    double dt_sec,
    SafetyVerdict* verdict
) {
    ServoTarget out;
    // Same per-arm seam as computeServoTarget: the safety stage reads and writes
    // only these five per-arm members, all of which the context already carries.
    ArmControlContext safety_ctx[2] = {armContext(ArmId::Left), armContext(ArmId::Right)};
    RobotState left_filter_state = left_state;
    RobotState right_filter_state = right_state;
    if (controllerSimulationDiagnosticStateAllowed(config_, left_filter_state)) {
        left_filter_state.has_error = false;
        left_filter_state.error_code = 0;
    }
    if (controllerSimulationDiagnosticStateAllowed(config_, right_filter_state)) {
        right_filter_state.has_error = false;
        right_filter_state.error_code = 0;
    }
    const SafetyTrackingState left_tracking_state = trackingStateForArm(
        config_,
        ArmId::Left,
        left_filter_state,
        safety_ctx[0].controller_sim_physical_baseline_q_deg
    );
    const SafetyTrackingState right_tracking_state = trackingStateForArm(
        config_,
        ArmId::Right,
        right_filter_state,
        safety_ctx[1].controller_sim_physical_baseline_q_deg
    );
    // pgmode controller-sim tracking-error advisory gate. Fail-closed in real mode
    // (controllerSimulationMotionGateOpen is false there). The physical-motion guard
    // (controller_simulation_physical_motion_fault) is a genuine safety signal — an
    // unexpected actual move in a no-motion mode — so it is excluded and still latches.
    const bool tracking_error_nonlatching =
        controllerSimulationMotionRequired(config_) &&
        controllerSimulationMotionGateOpen(config_) &&
        config_.safety.controller_simulation_tracking_error_nonlatching;
    const bool tracking_error_physical_motion_fault =
        left_tracking_state.controller_simulation_physical_motion_fault ||
        right_tracking_state.controller_simulation_physical_motion_fault;
    const SafetyCheckResult left_result = safety_filter_.filterJointTarget(
        desired.left_q_target_deg,
        safety_ctx[0].prev_sent_q_deg,
        safety_ctx[0].prevprev_sent_q_deg,
        left_filter_state,
        dt_sec,
        left_tracking_state
    );
    const SafetyCheckResult right_result = safety_filter_.filterJointTarget(
        desired.right_q_target_deg,
        safety_ctx[1].prev_sent_q_deg,
        safety_ctx[1].prevprev_sent_q_deg,
        right_filter_state,
        dt_sec,
        right_tracking_state
    );
    safety_ctx[0].safety_tracking = left_result.tracking;
    safety_ctx[1].safety_tracking = right_result.tracking;
    safety_ctx[0].abc_telemetry.safety_clamp_present = left_result.clamp.present;
    safety_ctx[0].abc_telemetry.safety_clamp = left_result.clamp;
    safety_ctx[1].abc_telemetry.safety_clamp_present = right_result.clamp.present;
    safety_ctx[1].abc_telemetry.safety_clamp = right_result.clamp;

    out.left_q_target_deg = left_result.filtered_q_deg;
    out.right_q_target_deg = right_result.filtered_q_deg;

    SafetyVerdict combined = SafetyVerdict::Ok;
    if (!left_result.ok) combined = left_result.verdict;
    if (!right_result.ok && combined == SafetyVerdict::Ok) combined = right_result.verdict;
    if ((left_result.joint_limit_clamped || right_result.joint_limit_clamped) && combined == SafetyVerdict::Ok) {
        combined = SafetyVerdict::JointLimitClamped;
    }
    const std::string combined_reason = !left_result.ok && !left_result.reason.empty()
        ? left_result.reason
        : (!right_result.ok && !right_result.reason.empty() ? right_result.reason : "");

    if (combined == SafetyVerdict::TrackingError) {
        if (config_.safety.tracking_error_policy == TrackingErrorPolicy::SnapToActual) {
            // 개발/mock용 복구 정책: 현재 실제 자세를 새 안전 기준점으로 삼고 그 자리에서 멈춘다.
            out.left_q_target_deg = left_state.q_actual_deg;
            out.right_q_target_deg = right_state.q_actual_deg;
        } else if (tracking_error_nonlatching && !tracking_error_physical_motion_fault) {
            // pgmode controller-sim advisory: keep following the rate-limited desired
            // target instead of holding/latching, so teleop stays live. The lag is the
            // diagnostics_suspect controller's reference readback, with no physical
            // motion. Surfaced as degraded telemetry + throttled WARN; real mode keeps
            // the latch (gate closed above).
            const SafetyClampTelemetry left_clamp = safety_filter_.clampMotionDetailed(
                desired.left_q_target_deg,
                safety_ctx[0].prev_sent_q_deg,
                safety_ctx[0].prevprev_sent_q_deg,
                dt_sec
            );
            const SafetyClampTelemetry right_clamp = safety_filter_.clampMotionDetailed(
                desired.right_q_target_deg,
                safety_ctx[1].prev_sent_q_deg,
                safety_ctx[1].prevprev_sent_q_deg,
                dt_sec
            );
            out.left_q_target_deg = left_clamp.q_after_accel_limit_deg;
            out.right_q_target_deg = right_clamp.q_after_accel_limit_deg;
            safety_ctx[0].abc_telemetry.safety_clamp_present = left_clamp.present;
            safety_ctx[0].abc_telemetry.safety_clamp = left_clamp;
            safety_ctx[1].abc_telemetry.safety_clamp_present = right_clamp.present;
            safety_ctx[1].abc_telemetry.safety_clamp = right_clamp;
            tracking_error_degraded_this_tick_ = true;
            const std::string reason = combined_reason.empty()
                ? "tracking error exceeded threshold"
                : combined_reason;
            constexpr uint64_t kTrackingErrorDegradedWarnPeriodNs = 5'000'000'000ULL;
            const uint64_t now_ns = nowSteadyNs();
            const bool state_changed =
                !tracking_error_degraded_prev_tick_ ||
                reason != last_tracking_error_degraded_reason_;
            const bool warn_period_elapsed =
                last_tracking_error_degraded_warn_ns_ == 0 ||
                now_ns - last_tracking_error_degraded_warn_ns_ >=
                    kTrackingErrorDegradedWarnPeriodNs;
            if (state_changed || warn_period_elapsed) {
                std::cerr
                    << "[WARN] controller-sim tracking error degraded "
                    << "(suppressed, not latched): " << reason << "\n";
                last_tracking_error_degraded_warn_ns_ = now_ns;
                last_tracking_error_degraded_reason_ = reason;
            }
            combined = SafetyVerdict::Ok;
        } else {
            latchFault(
                SafetyVerdict::TrackingError,
                combined_reason.empty() ? "tracking error exceeded threshold" : combined_reason,
                left_state,
                right_state
            );
            out = currentFaultHoldTarget();
            combined = SafetyVerdict::FaultLatched;
        }
    } else if (combined == SafetyVerdict::RobotStateError) {
        if (config_.safety.latch_fault_on_robot_state_error) {
            latchFault(
                SafetyVerdict::RobotStateError,
                combined_reason.empty() ? "robot state error or disconnected" : combined_reason,
                left_state,
                right_state
            );
            out = currentFaultHoldTarget();
            combined = SafetyVerdict::FaultLatched;
        } else {
            out.left_q_target_deg = safety_ctx[0].prev_sent_q_deg;
            out.right_q_target_deg = safety_ctx[1].prev_sent_q_deg;
        }
    }

    // === Unified safety velocity projection (Stage 3) ===
    // Floor plane and dual-arm self-collision are both expressed as linear velocity
    // constraints (d(constraint)/dt = J . qdot >= -xi) on the commanded joint
    // velocity and solved together in ONE Gauss-Seidel pass, so they cannot fight
    // and each removes ONLY the closing component (tangential/separating motion is
    // free). This replaces the floor binary Hold-revert and the self-collision
    // scalar barrier. Latch / FK-fail-closed / monitor_only semantics are preserved.
    std::vector<VelocityConstraint> safety_cons;
    bool floor_engaged = false;       // a floor row is within its engage band
    bool roi_engaged = false;          // a ROI-box face row is within its engage band
    bool reach_engaged = false;        // a reach-shell row is within its engage band
    bool user_floor_engaged = false;   // a user-floor-plane row is within its engage band
    bool self_collision_hold = false;  // stale verdict -> whole-arm hold, skip solve
    bool collision_constraints_engaged = false;
    bool left_floor_engaged = false;
    bool right_floor_engaged = false;
    bool left_roi_engaged = false;
    bool right_roi_engaged = false;
    bool left_reach_engaged = false;
    bool right_reach_engaged = false;
    bool left_user_floor_engaged = false;
    bool right_user_floor_engaged = false;
    const uint64_t safety_now_ns = last_loop_start_ns_ != 0 ? last_loop_start_ns_ : nowSteadyNs();
    const auto mark_intervention = [&](ArmId arm) {
        markSafetyIntervention(arm, safety_now_ns);
    };

    // ---- Floor plane: synchronous FK + per-point z-velocity Jacobian ----
    if (floorConstraintActive()) {
        const double floor_z = effectiveFloorZ();
        const FloorArmEvaluation left_eval = evaluateFloorArm(ArmId::Left, out.left_q_target_deg);
        const FloorArmEvaluation right_eval = evaluateFloorArm(ArmId::Right, out.right_q_target_deg);
        last_floor_left_ = left_eval;    // telemetry, even when already faulted
        last_floor_right_ = right_eval;
        if (!config_.safety.floor_constraint.monitor_only &&
            combined != SafetyVerdict::FaultLatched && !fault_latched_.load()) {
            const auto& fc = config_.safety.floor_constraint;
            // Returns true if it LATCHED (caller stops processing the other arm).
            const auto build_floor_arm = [&](
                ArmId arm, ControlMode mode, const RobotState& state,
                const FloorArmEvaluation& eval, JointArray& target_q,
                const JointArray& prev_sent_q) -> bool {
                (void)mode;
                (void)state;
                const std::string arm_name = arm == ArmId::Left ? "left" : "right";
                const ArmMountConfig& mount = arm == ArmId::Left ? config_.left_mount
                                                                 : config_.right_mount;
                if (!eval.checked) {
                    // TCP FK unavailable -> fail closed (cannot build a Jacobian).
                    const std::string reason = "floor constraint: " + arm_name + " TCP FK unavailable";
                    if (fc.fail_policy == FloorConstraintFailPolicy::FaultLatch) {
                        mark_intervention(arm);
                        latchFault(SafetyVerdict::FloorViolation, reason, left_state, right_state);
                        out = currentFaultHoldTarget();
                        combined = SafetyVerdict::FaultLatched;
                        return true;
                    }
                    target_q = prev_sent_q;  // hold this arm
                    mark_intervention(arm);
                    if (combined == SafetyVerdict::Ok || combined == SafetyVerdict::JointLimitClamped) {
                        combined = SafetyVerdict::FloorViolation;
                    }
                    return false;
                }
                // Hard latch (preserved): FaultLatch policy + descending below
                // the plane (not escaping vs the previous sent pose).
                if (fc.fail_policy == FloorConstraintFailPolicy::FaultLatch &&
                    eval.tcp_z_m < floor_z) {
                    const FloorArmEvaluation prev_eval = evaluateFloorArm(arm, prev_sent_q);
                    const bool escaping = prev_eval.checked && std::isfinite(prev_eval.tcp_z_m) &&
                        eval.tcp_z_m > prev_eval.tcp_z_m - kFloorEscapeEpsilonM;
                    if (!escaping) {
                        const std::string reason = "floor constraint: " + arm_name + " tcp z " +
                            std::to_string(eval.tcp_z_m) + " m below plane z_min " +
                            std::to_string(floor_z) + " m";
                        mark_intervention(arm);
                        latchFault(SafetyVerdict::FloorViolation, reason, left_state, right_state);
                        out = currentFaultHoldTarget();
                        combined = SafetyVerdict::FaultLatched;
                        return true;
                    }
                }
                // Within the engage band: add a z-velocity damper constraint. zdot of
                // the lowest point >= -sqrt(2 a (z - z_min)); below the plane xi=0
                // (block deeper, slide/lift free).
                const double margin = eval.tcp_z_m - floor_z;
                if (margin < fc.d_slow_m) {
                    JointArray Jz{};
                    const std::array<double, 3> offset{eval.lowest_offset_tcp.x(),
                                                       eval.lowest_offset_tcp.y(),
                                                       eval.lowest_offset_tcp.z()};
                    if (kinematics_ &&
                        kinematics_->computeFloorPointZJacobian(arm, target_q, mount, offset, Jz)) {
                        VelocityConstraint c;
                        const int base = arm == ArmId::Left ? 0 : kDof;
                        for (int i = 0; i < kDof; ++i) c.J[base + i] = Jz[i];
                        if (c.J.squaredNorm() > 1e-18) {
                            c.xi = margin > 0.0 ? std::sqrt(2.0 * fc.a_brake_m_s2 * margin) : 0.0;
                            c.d_now = margin;
                            safety_cons.push_back(std::move(c));
                            floor_engaged = true;
                            if (arm == ArmId::Left) {
                                left_floor_engaged = true;
                            } else {
                                right_floor_engaged = true;
                            }
                        }
                    } else {
                        // Jacobian unavailable -> fail closed (revert this arm).
                        target_q = prev_sent_q;
                        mark_intervention(arm);
                        if (combined == SafetyVerdict::Ok ||
                            combined == SafetyVerdict::JointLimitClamped) {
                            combined = SafetyVerdict::FloorViolation;
                        }
                    }
                }
                return false;
            };
            if (!build_floor_arm(ArmId::Left, left_mode, left_state, left_eval,
                                 out.left_q_target_deg, safety_ctx[0].prev_sent_q_deg)) {
                build_floor_arm(ArmId::Right, right_mode, right_state, right_eval,
                                out.right_q_target_deg, safety_ctx[1].prev_sent_q_deg);
            }
        }
    }

    // ---- ROI box (workspace limit): synchronous FK + per-face stand-axis ----
    // The 3D generalization of the floor plane: the TCP (and each offset point)
    // must stay inside the stand-frame box. Each of the 6 faces within its engage
    // band adds one closing-velocity row to the SAME shared solve, so the box, the
    // floor, and self-collision are all reconciled in one Gauss-Seidel pass.
    if (config_.safety.roi_box.enable) {
        const std::array<double, 3> roi_min = effectiveRoiMin();
        const std::array<double, 3> roi_max = effectiveRoiMax();
        const RoiArmEvaluation left_eval = evaluateRoiArm(ArmId::Left, out.left_q_target_deg);
        const RoiArmEvaluation right_eval = evaluateRoiArm(ArmId::Right, out.right_q_target_deg);
        last_roi_left_ = left_eval;    // telemetry, even when already faulted
        last_roi_right_ = right_eval;
        if (!config_.safety.roi_box.monitor_only &&
            combined != SafetyVerdict::FaultLatched && !fault_latched_.load()) {
            const auto& rb = config_.safety.roi_box;
            // Returns true if it LATCHED (caller stops processing the other arm).
            const auto build_roi_arm = [&](
                ArmId arm, ControlMode mode, const RobotState& state,
                const RoiArmEvaluation& eval, JointArray& target_q,
                const JointArray& prev_sent_q) -> bool {
                (void)mode;
                (void)state;
                const std::string arm_name = arm == ArmId::Left ? "left" : "right";
                const ArmMountConfig& mount = arm == ArmId::Left ? config_.left_mount
                                                                 : config_.right_mount;
                const auto fail_closed_hold = [&](const char* why) {
                    (void)why;
                    target_q = prev_sent_q;  // hold this arm
                    mark_intervention(arm);
                    if (combined == SafetyVerdict::Ok || combined == SafetyVerdict::JointLimitClamped) {
                        combined = SafetyVerdict::RoiViolation;
                    }
                };
                if (!eval.checked) {
                    // TCP FK unavailable -> fail closed (cannot build a Jacobian).
                    const std::string reason = "roi box: " + arm_name + " TCP FK unavailable";
                    if (rb.fail_policy == FloorConstraintFailPolicy::FaultLatch) {
                        mark_intervention(arm);
                        latchFault(SafetyVerdict::RoiViolation, reason, left_state, right_state);
                        out = currentFaultHoldTarget();
                        combined = SafetyVerdict::FaultLatched;
                        return true;
                    }
                    fail_closed_hold("roi_fk_unavailable");
                    return false;
                }
                // Hard latch (mirror of floor): FaultLatch policy + outside the
                // box + not escaping (outside-depth not shrinking
                // vs the previous sent pose).
                if (rb.fail_policy == FloorConstraintFailPolicy::FaultLatch &&
                    eval.violated) {
                    const RoiArmEvaluation prev_eval = evaluateRoiArm(arm, prev_sent_q);
                    const bool escaping = prev_eval.checked &&
                        std::isfinite(prev_eval.outside_depth_m) &&
                        eval.outside_depth_m < prev_eval.outside_depth_m + kRoiEscapeEpsilonM;
                    if (!escaping) {
                        const std::string reason = "roi box: " + arm_name + " outside box (closest face " +
                            eval.closest_face + ", margin " + std::to_string(eval.min_margin_m) + " m)";
                        mark_intervention(arm);
                        latchFault(SafetyVerdict::RoiViolation, reason, left_state, right_state);
                        out = currentFaultHoldTarget();
                        combined = SafetyVerdict::FaultLatched;
                        return true;
                    }
                }
                // Within the engage band of any face: add a closing-velocity damper
                // row for that face's most-exposed point. Lower face (side 0): the
                // point's stand-axis speed >= -sqrt(2 a margin) (block decreasing
                // past min); upper face (side 1): <= +sqrt(...) (block increasing
                // past max). Below the face (margin<0) xi=0 (block deeper, slide/
                // return free).
                const int base = arm == ArmId::Left ? 0 : kDof;
                for (int axis = 0; axis < 3; ++axis) {
                    for (int side = 0; side < 2; ++side) {
                        const double margin = eval.faces[axis][side].margin_m;
                        if (!std::isfinite(margin) || margin >= rb.d_slow_m) continue;
                        const math::Vector3& off = eval.faces[axis][side].offset_tcp;
                        const std::array<double, 3> offset{off.x(), off.y(), off.z()};
                        JointArray Jaxis{};
                        if (kinematics_ &&
                            kinematics_->computeStandAxisJacobian(arm, target_q, mount, offset,
                                                                  axis, Jaxis)) {
                            VelocityConstraint c;
                            const double sign = side == 0 ? 1.0 : -1.0;  // lower=+, upper=-
                            for (int i = 0; i < kDof; ++i) c.J[base + i] = sign * Jaxis[i];
                            if (c.J.squaredNorm() > 1e-18) {
                                c.xi = margin > 0.0 ? std::sqrt(2.0 * rb.a_brake_m_s2 * margin) : 0.0;
                                c.d_now = margin;
                                safety_cons.push_back(std::move(c));
                                roi_engaged = true;
                                if (arm == ArmId::Left) {
                                    left_roi_engaged = true;
                                } else {
                                    right_roi_engaged = true;
                                }
                            }
                        } else {
                            // Jacobian unavailable -> fail closed (revert this arm).
                            fail_closed_hold("roi_jacobian_unavailable");
                            return false;
                        }
                    }
                }
                return false;
            };
            if (!build_roi_arm(ArmId::Left, left_mode, left_state, left_eval,
                               out.left_q_target_deg, safety_ctx[0].prev_sent_q_deg)) {
                build_roi_arm(ArmId::Right, right_mode, right_state, right_eval,
                              out.right_q_target_deg, safety_ctx[1].prev_sent_q_deg);
            }
        }
    }

    // ---- Reach shell (workspace reach limit): synchronous FK + per-shell radial ----
    // The radial generalization of the floor/ROI faces: the TCP (and each offset
    // point) must stay inside the spherical shell [r_min, r_max] centered on the arm
    // base. Each shell within its engage band adds one closing-velocity row (along
    // the binding point's radial direction) to the SAME shared solve, so reach,
    // floor, ROI box, and self-collision are all reconciled in one Gauss-Seidel pass.
    // This brakes the TCP to zero radial speed AT the reach boundary and lets it
    // slide tangentially / return inward, instead of commanding a pose past the
    // arm's reach where IK fails and the legacy behavior was to silently stop.
    if (config_.safety.reach_constraint.enable) {
        const ReachArmEvaluation left_eval = evaluateReachArm(ArmId::Left, out.left_q_target_deg);
        const ReachArmEvaluation right_eval = evaluateReachArm(ArmId::Right, out.right_q_target_deg);
        last_reach_left_ = left_eval;    // telemetry, even when already faulted
        last_reach_right_ = right_eval;
        if (!config_.safety.reach_constraint.monitor_only &&
            combined != SafetyVerdict::FaultLatched && !fault_latched_.load()) {
            const auto& rc = config_.safety.reach_constraint;
            // Returns true if it LATCHED (caller stops processing the other arm).
            const auto build_reach_arm = [&](
                ArmId arm, ControlMode mode, const RobotState& state,
                const ReachArmEvaluation& eval, JointArray& target_q,
                const JointArray& prev_sent_q) -> bool {
                (void)mode;
                (void)state;
                const std::string arm_name = arm == ArmId::Left ? "left" : "right";
                const ArmMountConfig& mount = arm == ArmId::Left ? config_.left_mount
                                                                 : config_.right_mount;
                const auto fail_closed_hold = [&](const char* why) {
                    (void)why;
                    target_q = prev_sent_q;  // hold this arm
                    mark_intervention(arm);
                    if (combined == SafetyVerdict::Ok || combined == SafetyVerdict::JointLimitClamped) {
                        combined = SafetyVerdict::RoiViolation;
                    }
                };
                if (!eval.checked) {
                    // TCP FK unavailable -> fail closed (cannot build a Jacobian).
                    const std::string reason = "reach shell: " + arm_name + " TCP FK unavailable";
                    if (rc.fail_policy == FloorConstraintFailPolicy::FaultLatch) {
                        mark_intervention(arm);
                        latchFault(SafetyVerdict::RoiViolation, reason, left_state, right_state);
                        out = currentFaultHoldTarget();
                        combined = SafetyVerdict::FaultLatched;
                        return true;
                    }
                    fail_closed_hold("reach_fk_unavailable");
                    return false;
                }
                // Hard latch (mirror of floor/ROI): FaultLatch policy + outside
                // a shell + not escaping (outside-depth not shrinking vs the
                // previous sent pose).
                if (rc.fail_policy == FloorConstraintFailPolicy::FaultLatch &&
                    eval.violated) {
                    const ReachArmEvaluation prev_eval = evaluateReachArm(arm, prev_sent_q);
                    const bool escaping = prev_eval.checked &&
                        std::isfinite(prev_eval.outside_depth_m) &&
                        eval.outside_depth_m < prev_eval.outside_depth_m + kReachEscapeEpsilonM;
                    if (!escaping) {
                        const std::string reason = "reach shell: " + arm_name + " outside shell (closest " +
                            eval.closest_shell + ", margin " + std::to_string(eval.min_margin_m) + " m)";
                        mark_intervention(arm);
                        latchFault(SafetyVerdict::RoiViolation, reason, left_state, right_state);
                        out = currentFaultHoldTarget();
                        combined = SafetyVerdict::FaultLatched;
                        return true;
                    }
                }
                // Within the engage band of either shell: add a radial closing-velocity
                // damper row for that shell's binding point. Inner shell (0): radial
                // speed >= -sqrt(2 a margin) (block decreasing past r_min); outer shell
                // (1): <= +sqrt(...) (block increasing past r_max). Outside (margin<0)
                // xi=0 (block deeper, slide/return free).
                const int base = arm == ArmId::Left ? 0 : kDof;
                for (int shell = 0; shell < 2; ++shell) {
                    const double margin = eval.shells[shell].margin_m;
                    if (!std::isfinite(margin) || margin >= rc.d_slow_m) continue;  // inf inner = disabled
                    const math::Vector3& off = eval.shells[shell].offset_tcp;
                    const std::array<double, 3> offset{off.x(), off.y(), off.z()};
                    const std::array<double, 3>& dir = eval.shells[shell].dir_stand;
                    JointArray Jr{};
                    if (kinematics_ &&
                        kinematics_->computeStandDirectionJacobian(arm, target_q, mount, offset,
                                                                   dir, Jr)) {
                        VelocityConstraint c;
                        const double sign = shell == 0 ? 1.0 : -1.0;  // inner=+, outer=-
                        for (int i = 0; i < kDof; ++i) c.J[base + i] = sign * Jr[i];
                        if (c.J.squaredNorm() > 1e-18) {
                            c.xi = margin > 0.0 ? std::sqrt(2.0 * rc.a_brake_m_s2 * margin) : 0.0;
                            c.d_now = margin;
                            safety_cons.push_back(std::move(c));
                            reach_engaged = true;
                            if (arm == ArmId::Left) {
                                left_reach_engaged = true;
                            } else {
                                right_reach_engaged = true;
                            }
                        }
                    } else {
                        // Jacobian unavailable -> fail closed (revert this arm).
                        fail_closed_hold("reach_jacobian_unavailable");
                        return false;
                    }
                }
                return false;
            };
            if (!build_reach_arm(ArmId::Left, left_mode, left_state, left_eval,
                                 out.left_q_target_deg, safety_ctx[0].prev_sent_q_deg)) {
                build_reach_arm(ArmId::Right, right_mode, right_state, right_eval,
                                out.right_q_target_deg, safety_ctx[1].prev_sent_q_deg);
            }
        }
    }

    // ---- User-defined tilted floor plane: synchronous FK + per-point normal-velocity
    // Jacobian. Parallel to the (horizontal) floor_constraint above and ADDITIVE: both
    // apply when enabled, sharing the same Gauss-Seidel safety solve. Uses the plane
    // normal n as the constant stand-frame direction for the velocity damper. ----
    if (userFloorActive()) {
        const UserFloorArmEvaluation left_eval = evaluateUserFloorArm(ArmId::Left, out.left_q_target_deg);
        const UserFloorArmEvaluation right_eval = evaluateUserFloorArm(ArmId::Right, out.right_q_target_deg);
        last_user_floor_left_ = left_eval;    // telemetry, even when already faulted
        last_user_floor_right_ = right_eval;
        const auto& uf = config_.safety.user_floor_constraint;
        if (!uf.monitor_only &&
            combined != SafetyVerdict::FaultLatched && !fault_latched_.load()) {
            const math::Vector3 normal = effectiveUserFloorNormal();
            const std::array<double, 3> dir{normal.x(), normal.y(), normal.z()};
            // Returns true if it LATCHED (caller stops processing the other arm).
            const auto build_user_floor_arm = [&](
                ArmId arm, ControlMode mode, const RobotState& state,
                const UserFloorArmEvaluation& eval, JointArray& target_q,
                const JointArray& prev_sent_q) -> bool {
                (void)mode;
                (void)state;
                const std::string arm_name = arm == ArmId::Left ? "left" : "right";
                const ArmMountConfig& mount = arm == ArmId::Left ? config_.left_mount
                                                                 : config_.right_mount;
                const auto fail_closed_hold = [&](const char* why) {
                    (void)why;
                    target_q = prev_sent_q;  // hold this arm
                    mark_intervention(arm);
                    if (combined == SafetyVerdict::Ok || combined == SafetyVerdict::JointLimitClamped) {
                        combined = SafetyVerdict::FloorViolation;
                    }
                };
                if (!eval.checked) {
                    // TCP FK unavailable -> fail closed (cannot build a Jacobian).
                    const std::string reason = "user floor: " + arm_name + " TCP FK unavailable";
                    if (uf.fail_policy == FloorConstraintFailPolicy::FaultLatch) {
                        mark_intervention(arm);
                        latchFault(SafetyVerdict::FloorViolation, reason, left_state, right_state);
                        out = currentFaultHoldTarget();
                        combined = SafetyVerdict::FaultLatched;
                        return true;
                    }
                    fail_closed_hold("user_floor_fk_unavailable");
                    return false;
                }
                // Hard latch (mirror of floor): FaultLatch policy + below the
                // plane (signed_dist < 0) + not escaping (signed distance not
                // increasing vs the previous sent pose).
                if (uf.fail_policy == FloorConstraintFailPolicy::FaultLatch &&
                    eval.signed_dist_m < 0.0) {
                    const UserFloorArmEvaluation prev_eval = evaluateUserFloorArm(arm, prev_sent_q);
                    const bool escaping = prev_eval.checked && std::isfinite(prev_eval.signed_dist_m) &&
                        eval.signed_dist_m > prev_eval.signed_dist_m - kFloorEscapeEpsilonM;
                    if (!escaping) {
                        const std::string reason = "user floor: " + arm_name + " signed dist " +
                            std::to_string(eval.signed_dist_m) + " m below plane";
                        mark_intervention(arm);
                        latchFault(SafetyVerdict::FloorViolation, reason, left_state, right_state);
                        out = currentFaultHoldTarget();
                        combined = SafetyVerdict::FaultLatched;
                        return true;
                    }
                }
                // Within the engage band: add a normal-velocity damper row for the
                // lowest point. d(signed_dist)/dt >= -sqrt(2 a margin) brakes the closing
                // motion to zero AT the plane; lateral/lifting motion stays free, and
                // below the plane (margin<0) xi=0 (block deeper, slide/lift free).
                const double margin = eval.signed_dist_m;
                if (margin < uf.d_slow_m) {
                    const math::Vector3& off = eval.lowest_offset_tcp;
                    const std::array<double, 3> offset{off.x(), off.y(), off.z()};
                    JointArray Jn{};
                    if (kinematics_ &&
                        kinematics_->computeStandDirectionJacobian(arm, target_q, mount, offset,
                                                                   dir, Jn)) {
                        VelocityConstraint c;
                        const int base = arm == ArmId::Left ? 0 : kDof;
                        for (int i = 0; i < kDof; ++i) c.J[base + i] = Jn[i];
                        if (c.J.squaredNorm() > 1e-18) {
                            c.xi = margin > 0.0 ? std::sqrt(2.0 * uf.a_brake_m_s2 * margin) : 0.0;
                            c.d_now = margin;
                            safety_cons.push_back(std::move(c));
                            user_floor_engaged = true;
                            if (arm == ArmId::Left) {
                                left_user_floor_engaged = true;
                            } else {
                                right_user_floor_engaged = true;
                            }
                        }
                    } else {
                        // Jacobian unavailable -> fail closed (revert this arm).
                        fail_closed_hold("user_floor_jacobian_unavailable");
                        return false;
                    }
                }
                return false;
            };
            if (!build_user_floor_arm(ArmId::Left, left_mode, left_state, left_eval,
                                      out.left_q_target_deg, safety_ctx[0].prev_sent_q_deg)) {
                build_user_floor_arm(ArmId::Right, right_mode, right_state, right_eval,
                                     out.right_q_target_deg, safety_ctx[1].prev_sent_q_deg);
            }
        }
    }

    // Dual-arm self-collision guard: never command a configuration that brings the
    // two arms (or an arm and the stand) within the mesh barrier's hard floor.
    // Evaluated on the final candidate targets (post per-arm filtering). The single
    // self_collision.enable flag is the sole switch; the only implementation is the
    // async URDF-mesh CollisionMonitor (there is no capsule fallback path).
    if (config_.safety.self_collision.enable) {
        if (!collision_monitor_) {
            // LOUD fail-closed: the guard is enabled but the monitor is absent. The
            // constructor throws on load failure, so this is unreachable defensive
            // code — but never advance a real arm with the guard silently gone.
            last_self_collision_ = SelfCollisionResult{};  // checked=false
            if (combined != SafetyVerdict::FaultLatched && !fault_latched_.load()) {
                // Guard availability is a dual-arm aggregate decision with no
                // per-arm attribution; stamp both arms as conservatively intervened.
                mark_intervention(ArmId::Left);
                mark_intervention(ArmId::Right);
                latchFault(SafetyVerdict::SelfCollision,
                    "self-collision guard enabled but CollisionMonitor unavailable",
                    left_state, right_state);
                out = currentFaultHoldTarget();
                combined = SafetyVerdict::FaultLatched;
            }
        } else {
            // URDF mesh self-collision (async monitor + shared velocity barrier). The
            // monitor runs off the servo_j path; here we only feed the candidate and
            // read the latest verdict (atomic). The barrier scales the approach toward
            // the target so the arm can always brake before the hard floor; a stale
            // verdict or a hard breach fails closed.
            // Optional: drive the whole-arm floor box to track the operator's active
            // viser floor (ground_plane.follow_safety_floors) — user floor when active,
            // else stand floor, else off. Cheap mutex store; the monitor applies it
            // before its next eval. Stand floor is a horizontal plane at effectiveFloorZ.
            if (config_.safety.self_collision.mesh.ground_plane.follow_safety_floors &&
                collision_monitor_->hasGroundPlane()) {
                if (userFloorActive()) {
                    collision_monitor_->setGroundPlanePose(
                        true, effectiveUserFloorPoint(), effectiveUserFloorNormal());
                } else if (floorConstraintActive()) {
                    collision_monitor_->setGroundPlanePose(
                        true, math::Vector3(0.0, 0.0, effectiveFloorZ()),
                        math::Vector3::UnitZ());
                } else {
                    collision_monitor_->setGroundPlanePose(
                        false, math::Vector3::Zero(), math::Vector3::UnitZ());
                }
            }
            // Drive the articulated gripper's finger hulls to the live jaw open percent
            // so the checked gripper tracks the real gripper (no-op unless the
            // articulated meshes were configured). Invalid feedback -> open (envelope).
            if (collision_monitor_->hasArticulatedGripper()) {
                collision_monitor_->setGripperOpenPercent(
                    ArmId::Left, effectiveGripperPercent(ArmId::Left));
                collision_monitor_->setGripperOpenPercent(
                    ArmId::Right, effectiveGripperPercent(ArmId::Right));
            }
            // Submitted per arm even while one thread still drives both: this is
            // the call shape per-arm control threads need (neither ever holds both
            // candidates), so the seam is already in place when applySafety is
            // split. Behaviourally identical here -- both arms submit in the same
            // tick, and the monitor only evaluates once both have reported.
            collision_monitor_->submitArmTarget(ArmId::Left, out.left_q_target_deg);
            collision_monitor_->submitArmTarget(ArmId::Right, out.right_q_target_deg);
            const CollisionVerdict v = collision_monitor_->latest();
            last_collision_verdict_ = v;
            // Ground-truth diagnostic (opt-in via RB_SELF_COLLISION_LOG=1): print the
            // EXACT closest checked pair + clearance the async monitor computes for the
            // COMMANDED targets submitted above. Use this to see what the guard really
            // evaluates at an apparent collision — note it checks q_sent (the command),
            // not the displayed q_actual, which can diverge in pgmode controller-sim.
            static const bool kSelfColLog = std::getenv("RB_SELF_COLLISION_LOG") != nullptr;
            if (kSelfColLog && v.valid) {
                const double t_log_s = std::chrono::duration<double>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                static double last_log_s = 0.0;
                if (t_log_s - last_log_s > 0.2) {  // throttle ~5 Hz
                    last_log_s = t_log_s;
                    const std::string pair = v.near.empty()
                        ? std::string("(no near pairs)")
                        : (v.near.front().name_a + " <-> " + v.near.front().name_b);
                    // Max per-joint divergence between the ACTUAL arm (q_actual, what
                    // the GUI solid robots / display show) and the COMMANDED target the
                    // monitor just checked. Large here = display != checked pose (the
                    // pgmode controller-sim artifact); ~0 = display tracks the command.
                    double qdiv = 0.0;
                    for (int i = 0; i < kDof; ++i) {
                        qdiv = std::max(qdiv,
                            std::abs(left_state.q_actual_deg[i] - out.left_q_target_deg[i]));
                        qdiv = std::max(qdiv,
                            std::abs(right_state.q_actual_deg[i] - out.right_q_target_deg[i]));
                    }
                    std::cerr << "[selfcol] min=" << (v.min_clearance_m * 1000.0) << "mm  "
                              << pair << (v.hard_violation ? "  HARD-VIOLATION" : "")
                              << "  | q_actual-vs-checked max=" << qdiv << "deg\n";
                }
            }
            last_self_collision_ = selfCollisionResultFromVerdict(v, collision_monitor_cfg_);
            const double now_s = std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            const bool stale =
                collisionVerdictStale(v, now_s, collision_monitor_cfg_.max_staleness_s);
            if (!config_.safety.self_collision.monitor_only &&
                combined != SafetyVerdict::FaultLatched && !fault_latched_.load()) {
                // Only a real GEOMETRIC breach (hard_violation) escalates to fault_latch.
                // A stale verdict is a transient scheduling issue, not a collision: hold
                // (scale 0) and auto-recover when fresh verdicts resume — never latch on it.
                if (!stale &&
                    config_.safety.self_collision.fail_policy == SelfCollisionFailPolicy::FaultLatch &&
                    v.hard_violation) {
                    const std::string reason = "self-collision mesh breach (" +
                        (last_self_collision_.pair.empty() ? std::string("unknown")
                                                           : last_self_collision_.pair) +
                        "): clearance " + std::to_string(v.min_clearance_m) + " m below floor " +
                        std::to_string(collision_monitor_cfg_.d_hard_m) + " m";
                    // A hard mesh breach is a dual-arm/stand verdict at this level;
                    // stamp both arms because per-arm blame is not reliable here.
                    mark_intervention(ArmId::Left);
                    mark_intervention(ArmId::Right);
                    latchFault(SafetyVerdict::SelfCollision, reason, left_state, right_state);
                    out = currentFaultHoldTarget();
                    combined = SafetyVerdict::FaultLatched;
                } else if (stale) {
                    // Fail-closed: no fresh verdict -> hold at the previous sent pose
                    // (qdot = 0) and reanchor so nothing winds up while we wait. Auto-
                    // recovers once fresh verdicts resume (never latches on staleness).
                    out.left_q_target_deg = safety_ctx[0].prev_sent_q_deg;
                    out.right_q_target_deg = safety_ctx[1].prev_sent_q_deg;
                    // Staleness is a dual-arm aggregate verdict with no reliable
                    // per-arm attribution; stamp both arms as conservatively intervened.
                    mark_intervention(ArmId::Left);
                    mark_intervention(ArmId::Right);
                    self_collision_hold = true;  // skip the combined solve (qdot already 0)
                    if (combined == SafetyVerdict::Ok ||
                        combined == SafetyVerdict::JointLimitClamped) {
                        combined = SafetyVerdict::SelfCollision;
                    }
                } else {
                    // Collect self-collision velocity constraints (per near pair within
                    // d_slow, age-extrapolated) into the shared list; the combined solve
                    // below removes only the closing component of the command.
                    const std::size_t before_collision_cons = safety_cons.size();
                    buildCollisionConstraints(v, collision_monitor_cfg_, now_s - v.stamp_s,
                                              safety_cons);
                    if (safety_cons.size() > before_collision_cons) {
                        collision_constraints_engaged = true;
                    }
                }
            }
        }
    }

    // ---- Combined Gauss-Seidel solve (floor + self-collision together) ----
    if (!safety_cons.empty() && !self_collision_hold &&
        combined != SafetyVerdict::FaultLatched && !fault_latched_.load()) {
        int iters = 3;
        if (config_.safety.self_collision.enable) {
            iters = std::max(iters, collision_monitor_cfg_.projection_iterations);
        }
        const CollisionProjectionResult proj = solveVelocityProjection(
            safety_cons, safety_ctx[0].prev_sent_q_deg, safety_ctx[1].prev_sent_q_deg,
            out.left_q_target_deg, out.right_q_target_deg, dt_sec, iters,
            config_.safety.joint_target_smd.max_velocity_deg_s);
        // "Meaningfully blocked" (vs merely slowed while sliding) gates the windup
        // reanchor AND the safety verdict, so tangential motion stays Running and is
        // never frozen.
        constexpr double kReanchorDegPerSec = 2.0;
        const bool left_blocked = proj.left_correction_deg_s > kReanchorDegPerSec;
        const bool right_blocked = proj.right_correction_deg_s > kReanchorDegPerSec;
        const bool left_corrected =
            proj.left_correction_deg_s > kSafetyInterventionCorrectionEpsDegPerSec;
        const bool right_corrected =
            proj.right_correction_deg_s > kSafetyInterventionCorrectionEpsDegPerSec;
        if (left_corrected &&
            (left_floor_engaged || left_roi_engaged || left_reach_engaged ||
             left_user_floor_engaged || collision_constraints_engaged)) {
            mark_intervention(ArmId::Left);
        }
        if (right_corrected &&
            (right_floor_engaged || right_roi_engaged || right_reach_engaged ||
             right_user_floor_engaged || collision_constraints_engaged)) {
            mark_intervention(ArmId::Right);
        }
        if (left_blocked || right_blocked) {
            if (floor_engaged) ++floor_clamp_count_;
            if (roi_engaged) ++roi_clamp_count_;
            if (reach_engaged) ++reach_clamp_count_;
            if (user_floor_engaged) ++user_floor_clamp_count_;
            if (combined == SafetyVerdict::Ok || combined == SafetyVerdict::JointLimitClamped) {
                combined = floor_engaged      ? SafetyVerdict::FloorViolation
                         : user_floor_engaged ? SafetyVerdict::FloorViolation
                         : roi_engaged         ? SafetyVerdict::RoiViolation
                         : reach_engaged       ? SafetyVerdict::RoiViolation
                                               : SafetyVerdict::SelfCollision;
            }
        }
        static const bool kProjLog = std::getenv("RB_SELF_COLLISION_LOG") != nullptr;
        if (kProjLog && proj.active) {
            static double last_proj_log_s = 0.0;
            const double tt = std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            if (tt - last_proj_log_s > 0.2) {
                last_proj_log_s = tt;
                std::cerr << "[safety-proj] cons=" << proj.active_pairs
                          << (floor_engaged ? " (incl floor)" : "")
                          << (roi_engaged ? " (incl roi)" : "")
                          << (reach_engaged ? " (incl reach)" : "")
                          << (user_floor_engaged ? " (incl user_floor)" : "")
                          << " corr(L/R)=" << proj.left_correction_deg_s << "/"
                          << proj.right_correction_deg_s << " deg/s"
                          << ((left_blocked || right_blocked) ? "  BLOCKED" : "") << "\n";
            }
        }
    }

    if (verdict) *verdict = combined;
    return out;
}

void DualArmServoLoop::setGripperFeedback(ArmId arm, double percent, bool valid) {
    if (arm == ArmId::Left) {
        gripper_percent_left_.store(percent);
        gripper_percent_valid_left_.store(valid);
    } else {
        gripper_percent_right_.store(percent);
        gripper_percent_valid_right_.store(valid);
    }
}

double DualArmServoLoop::effectiveGripperPercent(ArmId arm) const {
    const bool valid = arm == ArmId::Left ? gripper_percent_valid_left_.load()
                                          : gripper_percent_valid_right_.load();
    if (!valid) return 100.0;  // no fresh feedback -> conservative gripper-open envelope
    const double pct = arm == ArmId::Left ? gripper_percent_left_.load()
                                          : gripper_percent_right_.load();
    return std::isfinite(pct) ? pct : 100.0;
}

FloorArmEvaluation DualArmServoLoop::evaluateFloorArm(ArmId arm, const JointArray& q_deg) const {
    FloorArmEvaluation eval;
    if (!kinematics_ || !finiteJointArray(q_deg)) {
        return eval;  // checked=false -> caller fails closed
    }
    const ArmMountConfig& mount = arm == ArmId::Left ? config_.left_mount : config_.right_mount;
    try {
        const Pose6D tcp = kinematics_->computeTcpStand(arm, q_deg, mount);
        if (!std::isfinite(tcp.z)) {
            return eval;  // checked=false -> caller fails closed
        }
        // Lowest point over the TCP and the configured TCP-frame offset points
        // (e.g. gripper fingertips, which dip below the TCP when the tool
        // rotates). The offset points are interpolated to the live gripper open
        // percent so the checked fingertips track the actual jaw geometry.
        // tcp_z_m carries the worst (lowest) z so the decision, escape, and clamp
        // logic all act on the most exposed point.
        std::vector<FloorCheckPointConfig>& offsets =
            arm == ArmId::Left ? floor_offset_scratch_left_ : floor_offset_scratch_right_;
        interpolateOffsetPoints(config_.safety.floor_constraint.tcp_offset_points,
                                effectiveGripperPercent(arm), offsets);
        const double lowest_z = floorLowestZWithOffsets(
            tcp, offsets, &eval.lowest_point,
            &eval.lowest_offset_tcp, &eval.lowest_point_stand);
        if (!std::isfinite(lowest_z)) {
            return FloorArmEvaluation{};  // checked=false -> caller fails closed
        }
        eval.checked = true;
        eval.tcp_z_m = lowest_z;
        eval.violated = lowest_z < effectiveFloorZ();
    } catch (const std::exception&) {
        return FloorArmEvaluation{};  // checked=false -> caller fails closed
    }
    return eval;
}

double DualArmServoLoop::effectiveFloorZ() const {
    return runtime_floor_z_m_.load();
}

bool DualArmServoLoop::floorConstraintActive() const {
    return config_.safety.floor_constraint.enable && runtime_floor_enabled_.load();
}

RoiArmEvaluation DualArmServoLoop::evaluateRoiArm(ArmId arm, const JointArray& q_deg) const {
    RoiArmEvaluation eval;
    if (!kinematics_ || !finiteJointArray(q_deg)) {
        return eval;  // checked=false -> caller fails closed
    }
    const ArmMountConfig& mount = arm == ArmId::Left ? config_.left_mount : config_.right_mount;
    try {
        const Pose6D tcp = kinematics_->computeTcpStand(arm, q_deg, mount);
        // Evaluate the TCP + configured offset points (interpolated to the live
        // gripper open percent) against the effective (runtime) box bounds;
        // checked=false (fail closed) when FK is non-finite.
        std::vector<FloorCheckPointConfig>& offsets =
            arm == ArmId::Left ? roi_offset_scratch_left_ : roi_offset_scratch_right_;
        interpolateOffsetPoints(config_.safety.roi_box.tcp_offset_points,
                                effectiveGripperPercent(arm), offsets);
        if (!roiEvaluateBox(tcp, offsets,
                            effectiveRoiMin(), effectiveRoiMax(), &eval)) {
            return RoiArmEvaluation{};
        }
    } catch (const std::exception&) {
        return RoiArmEvaluation{};  // checked=false -> caller fails closed
    }
    return eval;
}

ReachArmEvaluation DualArmServoLoop::evaluateReachArm(ArmId arm, const JointArray& q_deg) const {
    ReachArmEvaluation eval;
    if (!kinematics_ || !finiteJointArray(q_deg)) {
        return eval;  // checked=false -> caller fails closed
    }
    const ArmMountConfig& mount = arm == ArmId::Left ? config_.left_mount : config_.right_mount;
    // The shell is centered on the arm's mount (shoulder) origin in the stand frame.
    const std::array<double, 3> base_stand{mount.base_pose_in_stand.x,
                                           mount.base_pose_in_stand.y,
                                           mount.base_pose_in_stand.z};
    try {
        const Pose6D tcp = kinematics_->computeTcpStand(arm, q_deg, mount);
        // Offset points interpolated to the live gripper open percent.
        std::vector<FloorCheckPointConfig>& offsets =
            arm == ArmId::Left ? reach_offset_scratch_left_ : reach_offset_scratch_right_;
        interpolateOffsetPoints(config_.safety.reach_constraint.tcp_offset_points,
                                effectiveGripperPercent(arm), offsets);
        if (!reachEvaluateShell(tcp, base_stand, offsets,
                                config_.safety.reach_constraint.r_min_m,
                                config_.safety.reach_constraint.r_max_m, &eval)) {
            return ReachArmEvaluation{};  // checked=false -> caller fails closed
        }
    } catch (const std::exception&) {
        return ReachArmEvaluation{};  // checked=false -> caller fails closed
    }
    return eval;
}

std::array<double, 3> DualArmServoLoop::effectiveRoiMin() const {
    return {runtime_roi_min_m_[0].load(), runtime_roi_min_m_[1].load(),
            runtime_roi_min_m_[2].load()};
}

std::array<double, 3> DualArmServoLoop::effectiveRoiMax() const {
    return {runtime_roi_max_m_[0].load(), runtime_roi_max_m_[1].load(),
            runtime_roi_max_m_[2].load()};
}

bool DualArmServoLoop::userFloorActive() const {
    return runtime_user_floor_enabled_.load();
}

math::Vector3 DualArmServoLoop::effectiveUserFloorPoint() const {
    return math::Vector3(runtime_user_floor_point_m_[0].load(),
                         runtime_user_floor_point_m_[1].load(),
                         runtime_user_floor_point_m_[2].load());
}

math::Vector3 DualArmServoLoop::effectiveUserFloorNormal() const {
    return math::Vector3(runtime_user_floor_normal_[0].load(),
                         runtime_user_floor_normal_[1].load(),
                         runtime_user_floor_normal_[2].load());
}

double DualArmServoLoop::effectiveUserFloorMargin() const {
    return runtime_user_floor_margin_m_.load();
}

UserFloorArmEvaluation DualArmServoLoop::evaluateUserFloorArm(ArmId arm, const JointArray& q_deg) const {
    UserFloorArmEvaluation eval;
    if (!kinematics_ || !finiteJointArray(q_deg)) {
        return eval;  // checked=false -> caller fails closed
    }
    const ArmMountConfig& mount = arm == ArmId::Left ? config_.left_mount : config_.right_mount;
    try {
        const Pose6D tcp = kinematics_->computeTcpStand(arm, q_deg, mount);
        // Signed distance of the most-exposed point (TCP + configured offset points,
        // interpolated to the live gripper open percent) to the runtime plane;
        // checked=false (fail closed) when FK is non-finite.
        std::vector<FloorCheckPointConfig>& offsets =
            arm == ArmId::Left ? user_floor_offset_scratch_left_ : user_floor_offset_scratch_right_;
        interpolateOffsetPoints(config_.safety.user_floor_constraint.tcp_offset_points,
                                effectiveGripperPercent(arm), offsets);
        if (!userFloorEvaluatePlane(tcp, offsets,
                                    effectiveUserFloorPoint(), effectiveUserFloorNormal(),
                                    effectiveUserFloorMargin(), &eval)) {
            return UserFloorArmEvaluation{};
        }
    } catch (const std::exception&) {
        return UserFloorArmEvaluation{};  // checked=false -> caller fails closed
    }
    return eval;
}

Pose6D DualArmServoLoop::clampPoseToRoi(const Pose6D& pose) const {
    if (!config_.safety.roi_box.enable || config_.safety.roi_box.monitor_only) {
        return pose;
    }
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.z)) {
        return pose;
    }
    Pose6D clamped = pose;
    const std::array<double, 3> lo = effectiveRoiMin();
    const std::array<double, 3> hi = effectiveRoiMax();
    // Pull the TCP point into the box per stand axis, shrinking the interval by the
    // tool's offset-point extents so each configured check point (e.g. a gripper
    // fingertip at the target orientation) also stays inside. If a tool spans more
    // than the box on some axis, aim at the box center on that axis.
    std::array<double, 3> delta_lo{0.0, 0.0, 0.0};  // most-negative offset delta per axis
    std::array<double, 3> delta_hi{0.0, 0.0, 0.0};  // most-positive offset delta per axis
    const auto& points = config_.safety.roi_box.tcp_offset_points;
    if (!points.empty()) {
        const math::Matrix3 rotation = math::rotationFromPose(clamped);
        for (const FloorCheckPointConfig& point : points) {
            const math::Vector3 offset(point.offset_m[0], point.offset_m[1], point.offset_m[2]);
            const math::Vector3 w = rotation * offset;
            for (int k = 0; k < 3; ++k) {
                if (!std::isfinite(w[k])) continue;
                delta_lo[k] = std::min(delta_lo[k], w[k]);
                delta_hi[k] = std::max(delta_hi[k], w[k]);
            }
        }
    }
    std::array<double, 3> pos{clamped.x, clamped.y, clamped.z};
    for (int k = 0; k < 3; ++k) {
        const double low = lo[k] - delta_lo[k];   // TCP lower bound so min point >= lo
        const double high = hi[k] - delta_hi[k];  // TCP upper bound so max point <= hi
        if (low <= high) {
            pos[k] = std::min(std::max(pos[k], low), high);
        } else {
            pos[k] = 0.5 * (lo[k] + hi[k]);  // tool wider than box: aim at center
        }
    }
    clamped.x = pos[0];
    clamped.y = pos[1];
    clamped.z = pos[2];
    return clamped;
}

Pose6D DualArmServoLoop::clampPoseToFloor(const Pose6D& pose) const {
    if (!floorConstraintActive() || config_.safety.floor_constraint.monitor_only) {
        return pose;
    }
    Pose6D clamped = pose;
    if (std::isfinite(clamped.z)) {
        // Lift the TCP target so the LOWEST configured check point (e.g. a
        // gripper fingertip at the target orientation) stays on/above the
        // plane, not just the TCP point itself.
        double min_offset_delta_z = 0.0;  // tcp itself
        const auto& points = config_.safety.floor_constraint.tcp_offset_points;
        if (!points.empty()) {
            const math::Matrix3 rotation = math::rotationFromPose(clamped);
            for (const FloorCheckPointConfig& point : points) {
                const math::Vector3 offset(point.offset_m[0], point.offset_m[1], point.offset_m[2]);
                const double delta_z = (rotation * offset).z();
                if (std::isfinite(delta_z)) {
                    min_offset_delta_z = std::min(min_offset_delta_z, delta_z);
                }
            }
        }
        clamped.z = std::max(clamped.z, effectiveFloorZ() - min_offset_delta_z);
    }
    return clamped;
}

DualSendResult DualArmServoLoop::sendTargets(
    const ServoTarget& target,
    uint64_t command_seq,
    uint64_t command_host_time_ns,
    const std::string& send_policy,
    uint64_t dispatch_start_ns,
    uint64_t deadline_ns
) {
    ServoDispatchRequest dispatch_request;
    dispatch_request.left.q_target_deg = target.left_q_target_deg;
    dispatch_request.right.q_target_deg = target.right_q_target_deg;
    dispatch_request.seq = command_seq;
    dispatch_request.dispatch_start_ns = dispatch_start_ns;
    dispatch_request.deadline_ns = deadline_ns;

    if (send_policy != "send_servo_j") {
        const uint64_t suppressed_time_ns = nowSteadyNs();
        const BackendTiming timing = makeBackendTiming(suppressed_time_ns, suppressed_time_ns);
        const BackendError error = suppressedSendError(send_policy);
        dispatch_request.left.command_seq = command_seq;
        dispatch_request.left.host_time_ns = command_host_time_ns > 0
            ? command_host_time_ns
            : suppressed_time_ns;
        dispatch_request.left.deadline_ns = deadline_ns;
        dispatch_request.right.command_seq = command_seq;
        dispatch_request.right.host_time_ns = command_host_time_ns > 0
            ? command_host_time_ns
            : suppressed_time_ns;
        dispatch_request.right.deadline_ns = deadline_ns;

        DualSendResult result;
        result.left.arm_id = ArmId::Left;
        result.left.request = dispatch_request.left;
        result.left.result = rejectedSend(dispatch_request.left, error, timing);
        result.right.arm_id = ArmId::Right;
        result.right.request = dispatch_request.right;
        result.right.result = rejectedSend(dispatch_request.right, error, timing);
        result.dispatch_start_ns = suppressed_time_ns;
        result.dispatch_end_ns = suppressed_time_ns;
        result.timing = timing;
        return result;
    }

    if (rbpodoAsyncIoMode()) {
        dispatch_request.left.host_time_ns = command_host_time_ns;
        dispatch_request.right.host_time_ns = command_host_time_ns;
        return ServoDispatcher::dispatchRbpodoAsync(
            *left_worker_,
            *right_worker_,
            dispatch_request
        );
    }

    if (workerIoMode()) {
        dispatch_request.left.host_time_ns = command_host_time_ns;
        dispatch_request.right.host_time_ns = command_host_time_ns;
        if (workerOwnsSendCadence()) {
            // Non-blocking hand-off into each worker's latest-wins mailbox; the
            // worker sends it on its own cadence. Same enqueue path the rbpodo
            // async mode uses, so the result is "enqueued", not "sent".
            return ServoDispatcher::dispatchRbpodoAsync(
                *left_worker_,
                *right_worker_,
                dispatch_request
            );
        }
        return ServoDispatcher::dispatchWorker(*left_worker_, *right_worker_, dispatch_request);
    }

    dispatch_request.left.host_time_ns = command_host_time_ns;
    dispatch_request.right.host_time_ns = command_host_time_ns;
    return ServoDispatcher::dispatchDirectSequential(*left_robot_, *right_robot_, dispatch_request);
}

DualArmCommand DualArmServoLoop::makeHoldCommand(
    const RobotState& left_state,
    const RobotState& right_state,
    uint64_t now_ns
) const {
    (void)left_state;
    (void)right_state;
    DualArmCommand cmd;
    cmd.host_time_ns = now_ns;
    cmd.left.arm_id = ArmId::Left;
    cmd.right.arm_id = ArmId::Right;
    cmd.left.mode = ControlMode::Hold;
    cmd.right.mode = ControlMode::Hold;
    // Hold freezes each arm at its last commanded reference (previous_sent), NOT
    // the live measured actual. trajectory_filter's Hold branch holds
    // previous_sent only when has_joint_target is unset; setting has_joint_target
    // and commanding the measured q_actual every tick (the prior streaming hold)
    // left the servo with zero commanded error, so it produced no restoring
    // torque and the arm crept ~5-8 deg to its gravity-settled pose at
    // startup/idle (and the q_sent->q_actual feedback added a small tremble).
    // Keep q_target consistent with the Hold contract as well.  Hold currently
    // leaves has_joint_target=false and TrajectoryFilter uses previous_sent, but
    // carrying q_actual here made this command unsafe if a later rewrite retained
    // the payload flag and obscured the real equilibrium in telemetry/debugging.
    cmd.left.q_target_deg = left_prev_sent_q_deg_;
    cmd.right.q_target_deg = right_prev_sent_q_deg_;
    return cmd;
}

bool DualArmServoLoop::commandRequestsResetFault(const DualArmCommand& command) const {
    return command.left.mode == ControlMode::ResetFault || command.right.mode == ControlMode::ResetFault;
}

bool DualArmServoLoop::commandRequestsSetSafetyFloorZ(const DualArmCommand& command) const {
    return command.left.mode == ControlMode::SetSafetyFloorZ ||
           command.right.mode == ControlMode::SetSafetyFloorZ;
}

bool DualArmServoLoop::commandRequestsSetSafetyFloorEnabled(const DualArmCommand& command) const {
    return command.left.mode == ControlMode::SetSafetyFloorEnabled ||
           command.right.mode == ControlMode::SetSafetyFloorEnabled;
}

bool DualArmServoLoop::commandRequestsSetSafetyRoiBounds(const DualArmCommand& command) const {
    return command.left.mode == ControlMode::SetSafetyRoiBounds ||
           command.right.mode == ControlMode::SetSafetyRoiBounds;
}

bool DualArmServoLoop::commandRequestsSetExternalBoxes(const DualArmCommand& command) const {
    return command.left.mode == ControlMode::SetExternalBoxes ||
           command.right.mode == ControlMode::SetExternalBoxes;
}

bool DualArmServoLoop::applySetExternalBoxesCommand(const DualArmCommand& command) {
    // Leaseless runtime update of preallocated external keep-out boxes.
    // No-op unless the mesh CollisionMonitor was built with external boxes.
    if (!command.has_external_boxes) {
        std::cerr << "[WARN] SetExternalBoxes rejected: missing boxes payload\n";
        return false;
    }
    if (!collision_monitor_ || !collision_monitor_cfg_.external_boxes.enable) {
        std::cerr << "[DEBUG] SetExternalBoxes ignored: external boxes disabled or monitor absent\n";
        return false;
    }

    const int max_count = collision_monitor_cfg_.external_boxes.max_count;
    std::vector<ExternalBoxPose> poses(static_cast<std::size_t>(std::max(0, max_count)));
    int accepted = 0;
    int ignored_unknown = 0;
    int ignored_out_of_range = 0;
    int ignored_invalid_rotation = 0;
    for (const auto& box : command.external_boxes) {
        int slot = -1;
        if (box.label == "green") {
            slot = 0;
        } else if (box.label == "gray") {
            slot = 1;
        } else {
            ++ignored_unknown;
            continue;
        }
        if (slot < 0 || slot >= max_count) {
            ++ignored_out_of_range;
            continue;
        }

        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                R(row, col) = box.T_stand_box[static_cast<std::size_t>(row * 4 + col)];
            }
            t(row) = box.T_stand_box[static_cast<std::size_t>(row * 4 + 3)];
        }
        Eigen::Quaterniond q(R);
        const double q_norm = q.norm();
        if (!std::isfinite(q_norm) || q_norm <= 0.0) {
            ++ignored_invalid_rotation;
            continue;
        }
        q.normalize();
        ExternalBoxPose pose;
        pose.enable = box.enable;
        pose.R = q.toRotationMatrix();
        pose.t = t;
        poses[static_cast<std::size_t>(slot)] = pose;
        ++accepted;
    }
    const double stamp_s = nsToSec(nowSteadyNs());
    collision_monitor_->setExternalBoxes(poses, stamp_s);
    std::cerr << "[DEBUG] SetExternalBoxes applied " << accepted
              << " box(es), ignored_unknown=" << ignored_unknown
              << " ignored_out_of_range=" << ignored_out_of_range
              << " ignored_invalid_rotation=" << ignored_invalid_rotation
              << " by source_id=" << command.source.source_id << "\n";
    return true;
}

bool DualArmServoLoop::commandRequestsSetUserSafetyFloorPlane(const DualArmCommand& command) const {
    return command.left.mode == ControlMode::SetUserSafetyFloorPlane ||
           command.right.mode == ControlMode::SetUserSafetyFloorPlane;
}

bool DualArmServoLoop::commandRequestsEmergencyStop(const DualArmCommand& command) const {
    return command.left.mode == ControlMode::EmergencyStop || command.right.mode == ControlMode::EmergencyStop;
}

bool DualArmServoLoop::commandRequestsArmMotion(const DualArmCommand& command) const {
    return command.left.mode == ControlMode::ArmMotion || command.right.mode == ControlMode::ArmMotion;
}

bool DualArmServoLoop::commandRequestsDisarmMotion(const DualArmCommand& command) const {
    return command.left.mode == ControlMode::DisarmMotion || command.right.mode == ControlMode::DisarmMotion;
}

bool DualArmServoLoop::commandRequestsFreedrive(const DualArmCommand& command) const {
    return command.left.mode == ControlMode::Freedrive || command.right.mode == ControlMode::Freedrive;
}

const char* toString(FreedriveStage stage) {
    switch (stage) {
        case FreedriveStage::Off: return "off";
        case FreedriveStage::Quiesce: return "arming_quiesce";
        case FreedriveStage::Confirm: return "arming_confirm";
        case FreedriveStage::Active: return "active";
        case FreedriveStage::Exiting: return "exiting";
    }
    return "unknown";
}

bool DualArmServoLoop::anyFreedriveActive() const {
    return left_freedrive_stage_.load() != FreedriveStage::Off ||
           right_freedrive_stage_.load() != FreedriveStage::Off;
}

bool DualArmServoLoop::anyFreedriveEngaged() const {
    return left_freedrive_stage_.load() == FreedriveStage::Active ||
           right_freedrive_stage_.load() == FreedriveStage::Active;
}

// ---- force control ---------------------------------------------------------

void DualArmServoLoop::requestFtTare(ArmId arm) {
    (arm == ArmId::Left ? left_tare_request_ : right_tare_request_).store(true);
}

bool DualArmServoLoop::stepFtPipeline(ArmId arm, const RobotState& state) {
    const bool left = arm == ArmId::Left;
    sensor::FtPipeline& pipe = left ? left_ft_pipeline_ : right_ft_pipeline_;
    const FtArmConfig& ft_cfg = left ? config_.force_torque.left : config_.force_torque.right;
    FtTelemetry& tel = left ? left_ft_telemetry_ : right_ft_telemetry_;
    bool& decided = left ? left_ft_liveness_decided_ : right_ft_liveness_decided_;
    std::uint32_t& live_ticks = left ? left_ft_liveness_ticks_ : right_ft_liveness_ticks_;

    if (!config_.force_torque.enable || !ft_cfg.enable) {
        pipe.fillTelemetry(&tel);
        return false;
    }

    // ---- the COLD sensor-presence window ------------------------------------
    // A live RFT jitters above its noise floor every tick. Folding samples for a
    // bounded window and then LATCHING the verdict means a sensor that is unplugged
    // mid-run keeps its "connected" verdict — which is deliberate: the run-time
    // answer to a sensor that stops changing is the staleness of the state stream,
    // not a second liveness test that would flag a genuinely motionless arm.
    if (!decided) {
        pipe.livenessSample(state.eft_wrench);
        const auto need = static_cast<std::uint32_t>(
            ft_cfg.liveness_window_sec / (1.0 / std::max(1, config_.servo.rate_hz)));
        if (++live_ticks >= std::max<std::uint32_t>(need, 2u)) {
            decided = true;
            const bool connected = pipe.livenessDecide();
            std::cerr << (connected ? "[INFO] " : "[WARN] ") << "F/T " << toString(arm)
                      << ": " << pipe.connectReason()
                      << " (force p-p " << pipe.livenessForcePpN() << " N, torque p-p "
                      << pipe.livenessTorquePpNm() << " Nm over " << live_ticks << " ticks)\n";
            if (connected) {
                std::cerr << "[INFO] F/T " << toString(arm) << ": axis triad det = "
                          << pipe.axesDeterminant()
                          << (pipe.axesDeterminant() < 0.0
                                  ? " (LEFT-HANDED - correct for this RFT64; do not 'fix' it)"
                                  : " (right-handed)")
                          << ", tool " << ft_cfg.tool_mass_kg << " kg, SRO->TCP "
                          << ft_cfg.tool_xyz_mm[2] << " mm\n";
            }
        }
    }

    // ---- the wrench itself ---------------------------------------------------
    // The FK configuration is the arm's CURRENT COMMAND, not its measurement: the
    // sensor rides the commanded configuration, and the correction this wrench drives
    // is applied to that same command. One configuration per cycle, so the wrench
    // compensation and the correction it feeds cannot disagree about where the arm is.
    sensor::FtPipelineInput in;
    in.raw_sensor_axes = state.eft_wrench;
    in.raw_valid = state.eft_valid;
    const JointArray& q_cmd = left ? left_prev_sent_q_deg_ : right_prev_sent_q_deg_;
    if (kinematics_ != nullptr && state.has_valid_joint_state) {
        const ArmMountConfig& mount = left ? config_.left_mount : config_.right_mount;
        const std::optional<Pose6D> flange = kinematics_->computeFlangeStand(arm, q_cmd, mount);
        if (flange.has_value()) {
            in.r_stand_flange = math::rotationFromPose(*flange);
            in.kinematics_valid = true;
        }
    }
    const bool ok = pipe.step(in);

    // ---- a pending tare ------------------------------------------------------
    std::atomic<bool>& request = left ? left_tare_request_ : right_tare_request_;
    std::uint32_t& tare_ticks = left ? left_tare_ticks_ : right_tare_ticks_;
    if (request.load()) {
        if (!ok) {
            // REFUSE rather than tare against a pinned zero: a bias averaged while
            // the pipeline is not producing a wrench is a bias of exactly zero, which
            // would silently look like a successful tare.
            request.store(false);
            tare_ticks = 0;
            pipe.tareReset();
            tel.tare_state = "rejected";
            tel.tare_reason = "no trustworthy wrench (sensor absent, stale or kinematics unavailable)";
            std::cerr << "[WARN] F/T " << toString(arm) << " tare REFUSED: " << tel.tare_reason << "\n";
        } else {
            pipe.tareSample();
            tel.tare_state = "settling";
            if (++tare_ticks >= kFtTareSamples) {
                std::string reason;
                const bool accepted = pipe.tareCommit(kFtTareSamples, &reason);
                request.store(false);
                tare_ticks = 0;
                tel.tare_state = accepted ? "accepted" : "rejected";
                tel.tare_reason = reason;
                std::cerr << (accepted ? "[INFO] " : "[WARN] ") << "F/T " << toString(arm)
                          << " tare " << tel.tare_state << ": " << reason
                          << " (bias F [" << pipe.bias().fx << ", " << pipe.bias().fy << ", "
                          << pipe.bias().fz << "] N, M [" << pipe.bias().tx << ", "
                          << pipe.bias().ty << ", " << pipe.bias().tz << "] Nm, generation "
                          << pipe.biasGeneration() << ")\n";
            }
        }
    }

    pipe.fillTelemetry(&tel);
    return ok;
}

bool DualArmServoLoop::forceControlCovered(
    ArmId arm,
    const ArmCommand& command,
    const RobotState& state,
    std::string* reason
) {
    const auto no = [&](const char* why) {
        if (reason != nullptr) *reason = why;
        return false;
    };
    if (!config_.force_control.enable) return no("force_control.enable is false");

    const bool left = arm == ArmId::Left;
    const sensor::FtPipeline& pipe = left ? left_ft_pipeline_ : right_ft_pipeline_;
    const FtArmConfig& ft_cfg = left ? config_.force_torque.left : config_.force_torque.right;
    if (!ft_cfg.enable) return no("this arm's force_torque is disabled");
    if (!pipe.connected()) return no("F/T sensor is not connected (compensated channels pinned to zero)");
    // A LAW DRIVEN BY AN UNTARED SENSOR REGULATES AGAINST THE BIAS. The bias is
    // indistinguishable from contact to any law reading this surface, so an untared
    // arm would comply toward a force nobody is applying — and would do it silently.
    if (!pipe.biasValid()) return no("F/T has no bias yet - run a tare before enabling compliance");
    if (fault_latched_.load()) return no("a fault is latched");
    if (!state.has_valid_joint_state) return no("joint state is invalid");
    if (!state.tcp_actual_valid || !state.tcp_actual_stand.has_value()) {
        return no("measured TCP is unavailable");
    }
    // A DEVIATION COMPOSED ONTO A STALE NOMINAL IS A COMMAND NOBODY AUTHORED.
    const uint64_t now = nowSteadyNs();
    if (state.host_time_ns != 0 && now > state.host_time_ns) {
        const double age_sec = static_cast<double>(now - state.host_time_ns) * 1e-9;
        if (age_sec > config_.force_control.max_state_age_sec) {
            return no("robot state is stale");
        }
    }
    // Free-drive hands the arm to a human; composing a force offset on top of that
    // is two controllers driving one arm.
    const FreedriveStage stage =
        (left ? left_freedrive_stage_ : right_freedrive_stage_).load();
    if (stage != FreedriveStage::Off) return no("free-drive owns this arm");

    switch (command.mode) {
        case ControlMode::TcpPoseTarget:
            return true;
        case ControlMode::Hold:
            // A plain Hold has no Cartesian nominal to deviate FROM. With
            // hold_compliance the first covered tick latches the measured TCP and
            // holds it as the nominal, which is what makes a hand-push test possible
            // without a policy running.
            if (config_.force_control.hold_compliance) return true;
            return no("Hold is not compliant (set force_control.hold_compliance)");
        default:
            break;
    }
    if (reason != nullptr) {
        *reason = std::string("mode ") + toString(command.mode) + " is not a compliant path";
    }
    return false;
}

bool DualArmServoLoop::applyForceOverlay(ArmId arm, const RobotState& state, Pose6D* target) {
    if (target == nullptr) return false;
    const bool left = arm == ArmId::Left;
    sensor::FtPipeline& pipe = left ? left_ft_pipeline_ : right_ft_pipeline_;
    control::AdmittanceOverlay& overlay = left ? left_overlay_ : right_overlay_;
    control::ForceGate& gate = left ? left_force_gate_ : right_force_gate_;
    ForceControlTelemetry& tel = left ? left_force_control_telemetry_ : right_force_control_telemetry_;

    // ---- the wrench, carried to the STAND frame -----------------------------
    // The pipeline's TCP surface is in TOOL axes; the overlay integrates in stand.
    // The wrench source and the compose pivot are BOTH the TCP, which is what keeps
    // a straight push from twisting the tool.
    const Wrench6D& w = pipe.compStand();
    const math::Vector3 f_stand(w.fx, w.fy, w.fz);
    const math::Vector3 m_stand(w.tx, w.ty, w.tz);
    tel.wrench_stand = w;

    // ---- the workspace frame, re-aimed every tick ---------------------------
    // The per-axis law is only meaningful in a frame the task reasons in, and for a
    // streamed tool path that is the TOOL's. Latching it at entry would let the axis
    // meanings rotate as the arm moves — "soft along the approach" would stop
    // meaning the approach.
    {
        const ArmMountConfig& mount = left ? config_.left_mount : config_.right_mount;
        const JointArray& q_cmd = left ? left_prev_sent_q_deg_ : right_prev_sent_q_deg_;
        const std::optional<Pose6D> flange =
            kinematics_ != nullptr ? kinematics_->computeFlangeStand(arm, q_cmd, mount)
                                   : std::nullopt;
        if (flange.has_value()) {
            const FtArmConfig& ft_cfg = left ? config_.force_torque.left : config_.force_torque.right;
            Pose6D tool_rot{};
            tool_rot.rx = ft_cfg.tool_rpy_deg[0] * M_PI / 180.0;
            tool_rot.ry = ft_cfg.tool_rpy_deg[1] * M_PI / 180.0;
            tool_rot.rz = ft_cfg.tool_rpy_deg[2] * M_PI / 180.0;
            overlay.setWorkspaceFrame(math::rotationFromPose(*flange) *
                                      math::rotationFromPose(tool_rot));
        }
    }

    gate.update(f_stand, m_stand);
    // The gate throttles the FORCE-mode walk; compliance keeps yielding regardless.
    // A gated axis stops WALKING, it does not stop being soft.
    overlay.step(f_stand, m_stand, gate.translation());

    const Pose6D nominal = *target;
    const Pose6D composed = overlay.compose(nominal);
    *target = composed;

    tel.enabled = config_.force_control.enable;
    tel.covered = true;
    tel.compose_applied = !overlay.quiescent();
    const math::Vector3& d = overlay.deviation();
    const math::Vector3& dr = overlay.deviationRot();
    tel.deviation_m = {d.x(), d.y(), d.z()};
    tel.deviation_rad = {dr.x(), dr.y(), dr.z()};
    tel.deviation_norm_m = d.norm();
    tel.deviation_norm_rad = dr.norm();
    const math::Vector3& v = overlay.velocity();
    const math::Vector3& vr = overlay.velocityRot();
    tel.velocity_m_s = {v.x(), v.y(), v.z()};
    tel.velocity_rad_s = {vr.x(), vr.y(), vr.z()};
    tel.bounded = overlay.bounded();
    tel.fence_m = config_.force_control.max_deviation_m;
    tel.fence_rad = config_.force_control.max_deviation_rad;
    tel.gate_translation = gate.translation();
    tel.gate_rotation = gate.rotation();
    tel.gate_force_n = gate.forceN();
    tel.gate_torque_nm = gate.torqueNm();
    tel.gate_closed = gate.closed();

    // A SATURATION IS NEVER SILENT: while pinned at the fence the overlay holds the
    // bound instead of tracking, and the operator must know which state they are in.
    bool& prev = left ? left_overlay_bounded_prev_ : right_overlay_bounded_prev_;
    if (tel.bounded && !prev) {
        std::cerr << "[WARN] force overlay " << toString(arm)
                  << ": deviation SATURATED at the fence (" << tel.fence_m * 1e3 << " mm / "
                  << tel.fence_rad * 180.0 / M_PI << " deg) - holding the bound, not tracking\n";
    } else if (!tel.bounded && prev) {
        std::cerr << "[INFO] force overlay " << toString(arm)
                  << ": deviation back inside the fence\n";
    }
    prev = tel.bounded;

    bool& gate_prev = left ? left_gate_closed_prev_ : right_gate_closed_prev_;
    if (tel.gate_closed && !gate_prev) {
        std::cerr << "[INFO] force gate " << toString(arm) << " CLOSED (|F| " << tel.gate_force_n
                  << " N vs " << config_.force_control.gate_max_force_n << ", |M| "
                  << tel.gate_torque_nm << " Nm vs " << config_.force_control.gate_max_torque_nm
                  << ") - the plan is held while the contact stands. This is the design: the "
                     "command converges to the declared force\n";
    } else if (!tel.gate_closed && gate_prev) {
        std::cerr << "[INFO] force gate " << toString(arm) << " re-opened\n";
    }
    gate_prev = tel.gate_closed;
    return tel.compose_applied;
}

math::Vector3 DualArmServoLoop::gatePlanAdvance(ArmId arm, const math::Vector3& advance_stand) {
    const bool left = arm == ArmId::Left;
    control::ForceGate& gate = left ? left_force_gate_ : right_force_gate_;
    ForceControlTelemetry& tel = left ? left_force_control_telemetry_ : right_force_control_telemetry_;
    double removed = 0.0;
    const math::Vector3 out = gate.applyTranslation(advance_stand, &removed);
    tel.gate_removed_m = removed;
    return out;
}

void DualArmServoLoop::resyncArmAfterFreedrive(ArmId arm_id, const RobotState& state) {
    // Clear path state outside the state lock (mirroring clearFaultLatch's ordering).
    clearLatchedCartesianTarget(arm_id);

    // The servo stream was globally suppressed while teaching. A policy/follower
    // residual assembled before or during that interval must not execute from the
    // new physical pose. Publish a new epoch and require a fresh source anchor.
    ++motion_epoch_;
    left_delta_twist_follower_.deactivate();
    right_delta_twist_follower_.deactivate();
    left_chunk_follower_.deactivate();
    right_chunk_follower_.deactivate();
    left_follower_output_smd_.deactivate();
    right_follower_output_smd_.deactivate();
    left_pose_track_smd_.deactivate();
    right_pose_track_smd_.deactivate();
    left_chunk_submitted_recv_seq_ = chunk_frame_cache_recv_seq_;
    right_chunk_submitted_recv_seq_ = chunk_frame_cache_recv_seq_;

    std::lock_guard<std::mutex> lock(state_mutex_);
    if (arm_id == ArmId::Left) {
        const JointArray q = chooseSafeHoldTarget(ArmId::Left, state, left_prev_sent_q_deg_);
        left_prev_sent_q_deg_ = q;
        left_prevprev_sent_q_deg_ = q;
        left_fault_hold_q_deg_ = q;
        left_controller_sim_physical_baseline_q_deg_ = state.q_actual_deg;
        left_output_ma_.reset();
    } else {
        const JointArray q = chooseSafeHoldTarget(ArmId::Right, state, right_prev_sent_q_deg_);
        right_prev_sent_q_deg_ = q;
        right_prevprev_sent_q_deg_ = q;
        right_fault_hold_q_deg_ = q;
        right_controller_sim_physical_baseline_q_deg_ = state.q_actual_deg;
        right_output_ma_.reset();
    }
    std::cerr << "[INFO] freedrive exit resync: " << toString(arm_id)
              << " held target snapped to current actual joints"
              << "; motion_epoch=" << motion_epoch_ << "\n";
}

namespace {
// Free-drive arming timings (steady ns).
constexpr uint64_t kFreedriveQuiesceSettleNs = 150'000'000;    // fallback if motion-state unreported
constexpr uint64_t kFreedriveQuiesceDeadlineNs = 1'000'000'000; // abort if never idle
constexpr uint64_t kFreedriveConfirmTrustAckNs = 150'000'000;  // controller-sim no-op: trust ACK after this
constexpr uint64_t kFreedriveConfirmDeadlineNs = 800'000'000;  // abort if is_freedrive_mode never confirms (real)
constexpr uint64_t kFreedriveExitDeadlineNs = 800'000'000;     // resync anyway after teach_off if unconfirmed
}  // namespace

bool DualArmServoLoop::freedriveUsesControllerSignals(ArmId arm_id) const {
    const BackendConfig& cfg = (arm_id == ArmId::Left) ? config_.left_robot : config_.right_robot;
    return cfg.backend_type == BackendType::Rbpodo;
}

BackendResult<RobotState> DualArmServoLoop::sendFreedriveToBackend(ArmId arm_id, bool on) {
    const uint64_t start_ns = nowSteadyNs();
    const uint64_t timeout_ns = timeoutNs(config_.servo.command_timeout_sec, 1'000'000'000);
    const uint64_t deadline_ns = addDeadlineNs(start_ns, timeout_ns);
    ArmWorker* worker = (arm_id == ArmId::Left) ? left_worker_.get() : right_worker_.get();
    IRobotBackend* robot = (arm_id == ArmId::Left) ? left_robot_.get() : right_robot_.get();
    if (workerBackedIoMode()) {
        if (!worker) {
            return BackendResult<RobotState>{
                false, BackendOp::SetFreedrive, RobotState{},
                backendError(BackendErrorKind::RobotDisconnected,
                             "arm worker unavailable for setFreedrive", "", "worker_unavailable"),
                makeBackendTiming(start_ns, nowSteadyNs())};
        }
        return worker->setFreedrive(on, tick_, deadline_ns);
    }
    if (!robot) {
        return BackendResult<RobotState>{};  // ok defaults to false -> caller aborts
    }
    return robot->setFreedrive(on);
}

void DualArmServoLoop::abortFreedrive(ArmId arm_id, const RobotState& state, const std::string& reason) {
    std::atomic<FreedriveStage>& stage_atomic =
        (arm_id == ArmId::Left) ? left_freedrive_stage_ : right_freedrive_stage_;
    const FreedriveStage stage = stage_atomic.load();
    // If teach_on may have already been issued (Confirm) or engaged (Active/Exiting),
    // best-effort teach_off so we never leave the controller silently hand-guidable.
    if (stage == FreedriveStage::Confirm || stage == FreedriveStage::Active ||
        stage == FreedriveStage::Exiting) {
        const BackendResult<RobotState> off = sendFreedriveToBackend(arm_id, false);
        if (!off.ok) {
            std::cerr << "[WARN] freedrive " << toString(arm_id)
                      << " abort: best-effort teach_off failed: " << off.error.name << "\n";
        }
    }
    // Resync the held target to the current actual joints so resuming servo_j does
    // not snap the arm. Safe even from Quiesce (the arm was only holding).
    resyncArmAfterFreedrive(arm_id, state);
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        freedrive_note_ = std::string(toString(arm_id)) + ": freedrive aborted (" + reason + ")";
    }
    stage_atomic.store(FreedriveStage::Off);
    std::cerr << "[WARN] freedrive " << toString(arm_id) << " aborted: " << reason << "\n";
}

void DualArmServoLoop::teardownFreedriveOnStop() {
    // Called from stop() with the loop thread already joined and the
    // workers/backends still alive. No held-target resync is needed (the process
    // is tearing down); the one thing that must not be skipped is teach_off, so
    // the controller does not exit with a latched freedrive_teach_on that the
    // pendant's hardware direct-teaching button cannot clear.
    const auto teardown = [&](ArmId arm_id, std::atomic<FreedriveStage>& stage_atomic) {
        const FreedriveStage stage = stage_atomic.load();
        if (stage == FreedriveStage::Off) return;
        // teach_on may already be live (Confirm/Active/Exiting) or not yet issued
        // (Quiesce). teach_off is harmless if it was never on, so always send it.
        const BackendResult<RobotState> off = sendFreedriveToBackend(arm_id, false);
        if (!off.ok) {
            std::cerr << "[WARN] freedrive " << toString(arm_id)
                      << " teach_off on shutdown failed: " << off.error.name
                      << " — controller may stay latched in freedrive; power-cycle "
                         "the control box if hardware teaching does not return\n";
        } else {
            std::cerr << "[INFO] freedrive " << toString(arm_id)
                      << " teach_off issued on shutdown\n";
        }
        stage_atomic.store(FreedriveStage::Off);
    };
    teardown(ArmId::Left, left_freedrive_stage_);
    teardown(ArmId::Right, right_freedrive_stage_);
}

void DualArmServoLoop::requestFreedrive(
    const DualArmCommand& command,
    const RobotState& left_state,
    const RobotState& right_state
) {
    if (!config_.servo.allow_freedrive) {
        std::cerr << "[WARN] Freedrive command rejected: servo.allow_freedrive=false "
                     "(direct teaching is a fail-closed config opt-in)\n";
        std::lock_guard<std::mutex> lock(state_mutex_);
        freedrive_note_ = "rejected: servo.allow_freedrive=false";
        return;
    }

    const uint64_t now = nowSteadyNs();
    const auto request_arm = [&](ArmId arm_id,
                                 const ArmCommand& arm_command,
                                 std::atomic<FreedriveStage>& stage_atomic,
                                 uint64_t& deadline_ns,
                                 uint64_t& entered_ns,
                                 const RobotState& state) {
        if (arm_command.mode != ControlMode::Freedrive) return;
        if (!arm_command.has_freedrive) {
            std::cerr << "[WARN] Freedrive command for " << toString(arm_id)
                      << " ignored: missing freedrive_on payload\n";
            return;
        }
        const bool want_on = arm_command.freedrive_on;
        const FreedriveStage stage = stage_atomic.load();
        if (want_on) {
            if (stage == FreedriveStage::Off) {
                stage_atomic.store(FreedriveStage::Quiesce);
                entered_ns = now;
                deadline_ns = now + kFreedriveQuiesceDeadlineNs;
                std::cerr << "[INFO] freedrive " << toString(arm_id)
                          << " ON requested: quiescing servo stream before teach_on\n";
            }
            // else already arming/active — idempotent, ignore.
        } else {
            if (stage == FreedriveStage::Active) {
                const BackendResult<RobotState> off = sendFreedriveToBackend(arm_id, false);
                if (!off.ok) {
                    std::cerr << "[WARN] freedrive " << toString(arm_id)
                              << " teach_off failed: " << off.error.name
                              << " — forcing exit + resync\n";
                }
                stage_atomic.store(FreedriveStage::Exiting);
                entered_ns = now;
                deadline_ns = now + kFreedriveExitDeadlineNs;
                std::cerr << "[INFO] freedrive " << toString(arm_id)
                          << " OFF requested: teach_off issued, confirming exit\n";
            } else if (stage == FreedriveStage::Quiesce || stage == FreedriveStage::Confirm) {
                abortFreedrive(arm_id, state, "cancelled before engage");
            }
            // else Off/Exiting — idempotent.
        }
    };

    request_arm(ArmId::Left, command.left, left_freedrive_stage_,
                left_freedrive_deadline_ns_, left_freedrive_stage_entered_ns_, left_state);
    request_arm(ArmId::Right, command.right, right_freedrive_stage_,
                right_freedrive_deadline_ns_, right_freedrive_stage_entered_ns_, right_state);

    // Arming/holding free-drive on any arm leaves the system not motion-ready;
    // require an explicit re-arm afterwards. (No-op if already fault/emergency.)
    if (anyFreedriveActive() && !fault_latched_.load() &&
        motion_state_.load() != ServerMotionState::EmergencyLatched) {
        setMotionState(ServerMotionState::ConnectedHold);
    }
}

void DualArmServoLoop::advanceFreedrive(const RobotState& left_state, const RobotState& right_state) {
    const uint64_t now = nowSteadyNs();
    const auto advance_arm = [&](ArmId arm_id,
                                 std::atomic<FreedriveStage>& stage_atomic,
                                 uint64_t& deadline_ns,
                                 uint64_t& entered_ns,
                                 const std::string& op_mode,
                                 const RobotState& state) {
        const FreedriveStage stage = stage_atomic.load();
        if (stage == FreedriveStage::Off || stage == FreedriveStage::Active) return;

        const bool uses_signals = freedriveUsesControllerSignals(arm_id);
        const bool real_op = op_mode == "real";
        const uint64_t in_stage_ns = now - entered_ns;

        switch (stage) {
            case FreedriveStage::Quiesce: {
                // Direct teaching requires the controller to be idle. Wait until it
                // reports robot_state == 1 (Idle) after the servo stream stopped.
                // The settle-time fallback applies ONLY when the controller never
                // reports a usable motion state (stays 0); a controller actively
                // reporting "moving" (3) is waited out until idle or the deadline.
                const bool reports_motion_state = state.controller_motion_state != 0;
                const bool idle = !uses_signals ||
                    state.controller_motion_state == 1 ||
                    (!reports_motion_state && in_stage_ns >= kFreedriveQuiesceSettleNs);
                if (!idle) {
                    if (now >= deadline_ns) {
                        abortFreedrive(arm_id, state,
                                       "quiesce timeout: controller never reported idle");
                    }
                    return;
                }
                const BackendResult<RobotState> on = sendFreedriveToBackend(arm_id, true);
                if (!on.ok) {
                    abortFreedrive(arm_id, state,
                                   "teach_on failed: " + on.error.name + ":" + on.error.message);
                    return;
                }
                stage_atomic.store(FreedriveStage::Confirm);
                entered_ns = now;
                deadline_ns = now + kFreedriveConfirmDeadlineNs;
                std::cerr << "[INFO] freedrive " << toString(arm_id)
                          << " teach_on issued (controller idle); confirming engagement\n";
                return;
            }
            case FreedriveStage::Confirm: {
                // Ground-truth confirmation via is_freedrive_mode. In controller
                // simulation the flag may never flip (teach_on is a no-op), so trust
                // the ACK after a short settle; in real operation it MUST confirm.
                const bool confirmed = !uses_signals ||
                    state.controller_freedrive_on ||
                    (!real_op && in_stage_ns >= kFreedriveConfirmTrustAckNs);
                if (confirmed) {
                    stage_atomic.store(FreedriveStage::Active);
                    {
                        std::lock_guard<std::mutex> lock(state_mutex_);
                        freedrive_note_.clear();
                    }
                    std::cerr << "[INFO] freedrive " << toString(arm_id)
                              << " ACTIVE (direct teaching engaged"
                              << (state.controller_freedrive_on ? ", controller-confirmed"
                                                                : ", ack-trusted")
                              << "); servo_j suppressed\n";
                    return;
                }
                if (now >= deadline_ns) {
                    abortFreedrive(arm_id, state,
                                   "freedrive not confirmed by controller (is_freedrive_mode stayed off)");
                }
                return;
            }
            case FreedriveStage::Exiting: {
                const bool off_confirmed = !uses_signals ||
                    !state.controller_freedrive_on ||
                    (!real_op && in_stage_ns >= kFreedriveConfirmTrustAckNs);
                if (off_confirmed || now >= deadline_ns) {
                    if (!off_confirmed) {
                        std::cerr << "[WARN] freedrive " << toString(arm_id)
                                  << " exit not controller-confirmed before deadline; resyncing anyway\n";
                    }
                    resyncArmAfterFreedrive(arm_id, state);
                    stage_atomic.store(FreedriveStage::Off);
                    std::cerr << "[INFO] freedrive " << toString(arm_id)
                              << " exited; servo_j re-enabled after resync\n";
                }
                return;
            }
            case FreedriveStage::Off:
            case FreedriveStage::Active:
                return;
        }
    };

    advance_arm(ArmId::Left, left_freedrive_stage_, left_freedrive_deadline_ns_,
                left_freedrive_stage_entered_ns_, config_.left_robot.operation_mode, left_state);
    advance_arm(ArmId::Right, right_freedrive_stage_, right_freedrive_deadline_ns_,
                right_freedrive_stage_entered_ns_, config_.right_robot.operation_mode, right_state);
}

bool DualArmServoLoop::commandRequestsMotion(const DualArmCommand& command) const {
    return isMotionMode(command.left.mode) || isMotionMode(command.right.mode);
}

DualArmCommand DualArmServoLoop::applyInitMotionSequencer(
    DualArmCommand command,
    const RobotState& left_state,
    const RobotState& right_state
) {
    const bool left_init =
        command.left.mode == ControlMode::JointTarget &&
        command.left.joint_target_profile == JointTargetProfile::InitMotion;
    const bool right_init =
        command.right.mode == ControlMode::JointTarget &&
        command.right.joint_target_profile == JointTargetProfile::InitMotion;
    const bool is_init = left_init || right_init;

    const bool freeze_other_arm = config_.safety.init_motion_planner.single_arm_freeze_other_arm;

    // Rewrite only the arm(s) covered by the active init_motion profile. The
    // downstream pipeline never sees the init_motion profile marker, while a
    // non-selected arm can keep its original TcpPoseTarget/flow command.
    const auto set_joint_target = [](ArmCommand& arm, const JointArray& q) {
        arm.mode = ControlMode::JointTarget;
        arm.joint_target_profile = JointTargetProfile::Direct;
        arm.q_target_deg = q;
        arm.has_joint_target = true;
    };
    const auto rewrite_selected = [&](DualArmCommand& c, const InitMotionExec& ex,
                                      const JointArray& l, const JointArray& r) {
        if (ex.left_active || freeze_other_arm) {
            set_joint_target(c.left, l);
        }
        if (ex.right_active || freeze_other_arm) {
            set_joint_target(c.right, r);
        }
    };
    // Arrival-decel taper endpoint: hand the SMD the TRUE final stop (terminal waypoint)
    // for the active arm(s) while rewrite_selected feeds it the pursuit carrot as q_target.
    // The SMD eases into this stop independently of the cruise natural frequency; a command
    // without it (plain PTP / jog) is untouched.
    const auto set_arrival_stop = [&](DualArmCommand& c, const InitMotionExec& ex,
                                      const JointArray& l, const JointArray& r) {
        if (ex.left_active || freeze_other_arm) {
            c.left.arrival_stop_q_deg = l;
            c.left.has_arrival_stop = true;
        }
        if (ex.right_active || freeze_other_arm) {
            c.right.arrival_stop_q_deg = r;
            c.right.has_arrival_stop = true;
        }
    };
    const auto hold_selected = [&](DualArmCommand& c, const InitMotionExec& ex) {
        if (ex.left_active || freeze_other_arm) {
            c.left.mode = ControlMode::Hold;
            c.left.joint_target_profile = JointTargetProfile::Direct;
            // IMPORTANT: a stale InitMotion/JointTarget q_target_deg must not leak through
            // Hold. TrajectoryFilter::computeJointTarget(Hold) uses q_target_deg when
            // has_joint_target=true, so failing to clear this field turns "hold while
            // planning/already-at-goal" into an immediate move toward the raw init target.
            c.left.has_joint_target = false;
        }
        if (ex.right_active || freeze_other_arm) {
            c.right.mode = ControlMode::Hold;
            c.right.joint_target_profile = JointTargetProfile::Direct;
            c.right.has_joint_target = false;
        }
    };
    // Brake-before-plan: the braking arm(s) are streamed as a plain JointTarget toward
    // their monotone stop goal (joint_target_smd continues from the last SENT velocity —
    // see TrajectoryFilter::observeSentTarget); every other selected/frozen arm holds at
    // its last sent target exactly as hold_selected does.
    const auto brake_selected = [&](DualArmCommand& c, const InitMotionExec& ex) {
        hold_selected(c, ex);
        if (ex.brake_left) {
            set_joint_target(c.left, ex.brake_goal_left);
            c.left.has_arrival_stop = false;
        }
        if (ex.brake_right) {
            set_joint_target(c.right, ex.brake_goal_right);
            c.right.has_arrival_stop = false;
        }
    };
    const auto sequence_active = [](const InitMotionExec& e) {
        return e.status == InitMotionStatus::Planning ||
               e.status == InitMotionStatus::Executing;
    };
    const auto non_idle = [](const InitMotionExec& e) {
        return e.status != InitMotionStatus::Idle;
    };
    // A plan that has begun (planning or streaming) is a committed go-to-init move.
    // The deadman synthesises a seq==0 Hold/Hold once the one-shot profile command
    // ages out of its freshness window. That is NOT an operator cancel: keep driving
    // the in-flight move to completion. An EXPLICIT command cancels.
    const bool deadman_hold = isSyntheticHoldCommand(command);

    if (!is_init) {
        const bool any_sequence_active =
            sequence_active(left_init_motion_exec_) || sequence_active(right_init_motion_exec_);
        if (any_sequence_active && deadman_hold && init_motion_planner_) {
            // Continuation across command staleness: fall through to the status machine
            // WITHOUT recomputing the target (command carries no init pose now) and
            // WITHOUT relaunching. exec.target/waypoints already hold the committed move.
        } else {
            // Explicit non-profile command, or no active sequence -> cancel/reset so
            // the next init_motion profile plans fresh. Reset is non-blocking: in-flight
            // planner jobs run on the worker and stale results are ignored by generation.
            for (InitMotionExec* ex : {&left_init_motion_exec_, &right_init_motion_exec_}) {
                if (ex->status == InitMotionStatus::Idle) continue;
                // Diagnostic: a committed go-to-init move was interrupted before it
                // reached the goal. This is the only silent way the arm stops part-way
                // (no execution-timeout, no planning failure). Logs WHAT interrupted it
                // (a real command with seq!=0 from a competing source, or a deadman hold
                // that failed the synthetic-hold test) so the cause is unambiguous.
                if (sequence_active(*ex)) {
                    std::cerr << "[WARN] JointTarget init_motion: committed move CANCELLED mid-"
                              << (ex->status == InitMotionStatus::Planning ? "planning"
                                                                           : "execution")
                              << " by a non-init command (seq=" << command.seq
                              << ", left_mode=" << static_cast<int>(command.left.mode)
                              << ", right_mode=" << static_cast<int>(command.right.mode)
                              << ", deadman_hold=" << (deadman_hold ? 1 : 0)
                              << ", planner=" << (init_motion_planner_ ? 1 : 0)
                              << "); holding (arm stops where it is)\n";
                }
                *ex = InitMotionExec{};
            }
            return command;
        }
    }

    if (is_init) {
        // Launching an init for one arm must NOT cancel an in-flight init on the OTHER,
        // disjoint arm: the left/right execs drive their own arm independently and each
        // rewrite_selected only writes its active arm's command, so they run concurrently.
        // Reset the other exec ONLY when it OVERLAPS the arm(s) we are about to drive (a
        // prior both-arm exec) — otherwise two execs would fight over the same arm.
        if (left_init && right_init) {
            // Both-arm init drives the LEFT exec for both arms; drop any separate right exec.
            if (non_idle(right_init_motion_exec_)) right_init_motion_exec_ = InitMotionExec{};
        } else if (left_init) {
            // Left-only: a right-ONLY exec keeps running; clear the right exec only if it
            // also drives the LEFT arm (a both-arm exec) -> overlap.
            if (right_init_motion_exec_.left_active) right_init_motion_exec_ = InitMotionExec{};
        } else if (right_init) {
            // Right-only: a left-ONLY exec keeps running; clear the left exec only if it
            // also drives the RIGHT arm (a both-arm exec) -> overlap.
            if (left_init_motion_exec_.right_active) left_init_motion_exec_ = InitMotionExec{};
        }
    }

    // Planner disabled -> fall back to a direct JointTarget to the requested pose;
    // the reactive barrier + floor still guard each tick.
    if (is_init && !init_motion_planner_) {
        InitMotionExec direct;
        direct.left_active = left_init;
        direct.right_active = right_init;
        const JointArray direct_left =
            left_init ? command.left.q_target_deg : left_prev_sent_q_deg_;
        const JointArray direct_right =
            right_init ? command.right.q_target_deg : right_prev_sent_q_deg_;
        rewrite_selected(command, direct, direct_left, direct_right);
        // No waypoints in the planner-disabled fallback: the requested pose IS the final
        // stop, so ease into it directly (same arrival taper as the planned path).
        set_arrival_stop(command, direct, direct_left, direct_right);
        return command;
    }

    const auto measured_or_sent = [](const RobotState& state, const JointArray& fallback) -> JointArray {
        return (state.q_actual_valid && finiteJointArray(state.q_actual_deg))
            ? state.q_actual_deg
            : fallback;
    };
    const JointArray current_left_q = measured_or_sent(left_state, left_prev_sent_q_deg_);
    const JointArray current_right_q = measured_or_sent(right_state, right_prev_sent_q_deg_);
    const auto active_goal_dist = [&](const InitMotionExec& ex,
                                      const std::pair<JointArray, JointArray>& goal_wp) {
        double dist = 0.0;
        for (int i = 0; i < kDof; ++i) {
            if (ex.left_active || freeze_other_arm) {
                dist = std::max(dist, std::abs(current_left_q[i] - goal_wp.first[i]));
            }
            if (ex.right_active || freeze_other_arm) {
                dist = std::max(dist, std::abs(current_right_q[i] - goal_wp.second[i]));
            }
        }
        return dist;
    };
    const auto active_goal_reached = [&](const InitMotionExec& ex,
                                         const std::pair<JointArray, JointArray>& goal_wp,
                                         double tol_deg) {
        for (int i = 0; i < kDof; ++i) {
            if ((ex.left_active || freeze_other_arm) &&
                std::abs(current_left_q[i] - goal_wp.first[i]) > tol_deg) {
                return false;
            }
            if ((ex.right_active || freeze_other_arm) &&
                std::abs(current_right_q[i] - goal_wp.second[i]) > tol_deg) {
                return false;
            }
        }
        return true;
    };
    const auto request_goal_reached = [&](bool request_left, bool request_right,
                                          const JointArray& target_left,
                                          const JointArray& target_right,
                                          double tol_deg) {
        for (int i = 0; i < kDof; ++i) {
            if (request_left && std::abs(current_left_q[i] - target_left[i]) > tol_deg) {
                return false;
            }
            if (request_right && std::abs(current_right_q[i] - target_right[i]) > tol_deg) {
                return false;
            }
        }
        return true;
    };
    const auto reanchor_selected_to_measured = [&](bool request_left, bool request_right) {
        // InitMotion plans from measured q_actual. The command stream must be re-anchored to
        // the same measured pose before the first waypoint is streamed; otherwise a flow/servo
        // target that was leading q_actual can become the first InitMotion command and create
        // the observed initial jerk. Re-anchoring only on an explicit InitMotion start/no-op
        // avoids using measured pose as a continuous hold source during normal operation.
        std::lock_guard<std::mutex> lock(state_mutex_);
        if ((request_left || freeze_other_arm) && finiteJointArray(current_left_q)) {
            left_prevprev_sent_q_deg_ = current_left_q;
            left_prev_sent_q_deg_ = current_left_q;
            left_output_ma_.reset();
        }
        if ((request_right || freeze_other_arm) && finiteJointArray(current_right_q)) {
            right_prevprev_sent_q_deg_ = current_right_q;
            right_prev_sent_q_deg_ = current_right_q;
            right_output_ma_.reset();
        }
    };
    const auto mark_done_at_measured = [&](InitMotionExec& ex,
                                           bool request_left,
                                           bool request_right,
                                           const JointArray& target_left,
                                           const JointArray& target_right,
                                           const std::string& message) {
        ex.target_left = target_left;
        ex.target_right = target_right;
        ex.left_active = request_left;
        ex.right_active = request_right;
        ex.has_target = true;
        ex.waypoints.clear();
        ex.waypoints.emplace_back(current_left_q, current_right_q);
        ex.index = 0;
        ex.escape_waypoints = 0;
        ex.status = InitMotionStatus::Done;
        ex.message = message;
        ex.fail_mode = InitMotionPlanResult::FailMode::None;
        ex.exec_timeout = false;
        ex.exec_stalled = false;
        ex.best_dist_deg = 0.0;
        ex.last_progress_ns = nowSteadyNs();
        ex.last_exec_log_ns = 0;
        ex.start_ns = nowSteadyNs();
    };

    // Brake-before-plan (safety.init_motion_planner.brake_before_plan): decided from the
    // last SENT joint velocity of the selected arm(s). The servo period is the sample
    // spacing of prev/prevprev sent, and the joint_target_smd natural frequency shapes
    // the monotone stop goal (see planInitMotionBrake).
    const auto& brake_cfg = config_.safety.init_motion_planner;
    const double brake_dt_sec = 1.0 / static_cast<double>(std::max(1, config_.servo.rate_hz));
    const double brake_smd_fn_hz = config_.safety.joint_target_smd.enable
        ? config_.safety.joint_target_smd.natural_frequency_hz
        : 0.0;
    const auto begin_brake = [&](InitMotionExec& ex, bool request_left, bool request_right,
                                 const JointArray& target_left,
                                 const JointArray& target_right) -> bool {
        if (!brake_cfg.brake_before_plan) return false;
        InitMotionBrakePlan brake_left;
        InitMotionBrakePlan brake_right;
        if (request_left || freeze_other_arm) {
            brake_left = planInitMotionBrake(
                left_prev_sent_q_deg_, left_prevprev_sent_q_deg_, brake_dt_sec,
                brake_smd_fn_hz, brake_cfg.brake_enter_deg_s, brake_cfg.brake_max_travel_deg);
        }
        if (request_right || freeze_other_arm) {
            brake_right = planInitMotionBrake(
                right_prev_sent_q_deg_, right_prevprev_sent_q_deg_, brake_dt_sec,
                brake_smd_fn_hz, brake_cfg.brake_enter_deg_s, brake_cfg.brake_max_travel_deg);
        }
        if (!brake_left.needed && !brake_right.needed) return false;
        // Commit the request (targets/arms) but do NOT re-anchor and do NOT post the planner
        // job yet: the plan is launched from the settled measured pose once the sent
        // velocity has decayed (or the brake timeout elapsed).
        ex.target_left = target_left;
        ex.target_right = target_right;
        ex.left_active = request_left;
        ex.right_active = request_right;
        ex.has_target = true;
        ex.waypoints.clear();
        ex.index = 0;
        ex.escape_waypoints = 0;
        ex.plan_generation = 0;
        ex.status = InitMotionStatus::Planning;
        ex.message = "braking";
        ex.fail_mode = InitMotionPlanResult::FailMode::None;
        ex.exec_timeout = false;
        ex.exec_stalled = false;
        ex.start_ns = nowSteadyNs();
        ex.brake_pending = true;
        ex.brake_left = brake_left.needed;
        ex.brake_right = brake_right.needed;
        ex.brake_start_ns = ex.start_ns;
        ex.brake_goal_left = brake_left.goal;
        ex.brake_goal_right = brake_right.goal;
        std::cerr << "[INFO] JointTarget init_motion: brake-before-plan: selected arm still "
                     "streaming (sent speed left=" << brake_left.max_speed_deg_s
                  << " right=" << brake_right.max_speed_deg_s
                  << " deg/s); decelerating from the last sent target before planning\n";
        return true;
    };
    const auto brake_settled = [&](const InitMotionExec& ex) {
        double speed = 0.0;
        if (ex.brake_left) {
            speed = std::max(speed, sentJointSpeedDegS(
                left_prev_sent_q_deg_, left_prevprev_sent_q_deg_, brake_dt_sec));
        }
        if (ex.brake_right) {
            speed = std::max(speed, sentJointSpeedDegS(
                right_prev_sent_q_deg_, right_prevprev_sent_q_deg_, brake_dt_sec));
        }
        return speed <= std::max(0.0, brake_cfg.brake_exit_deg_s);
    };
    const auto clear_brake = [](InitMotionExec& ex) {
        ex.brake_pending = false;
        ex.brake_left = false;
        ex.brake_right = false;
    };

    const auto process_exec = [&](InitMotionExec& ex, PlannerRequester requester,
                                  bool request_left, bool request_right, bool fresh_request) {
        // Launch a new one-shot request from the MEASURED joint pose
        // when it is available. For single-arm init, freeze the non-selected arm in the
        // planner at its measured pose; do not use last-sent references because SMD/flow
        // can intentionally lead the physical robot by a small amount.
        //
        // No-op-or-launch for a committed request. Called on the request tick when the
        // selected arm(s) are at rest, or after the brake phase settled. Returns true when
        // it completed as a measured no-op (command already rewritten to Hold).
        const auto launch_or_noop = [&](InitMotionExec& ex, bool request_left, bool request_right,
                                        const JointArray& target_left,
                                        const JointArray& target_right) -> bool {
            clear_brake(ex);
            const double noop_tol_deg = std::max(
                0.0,
                std::max(config_.safety.init_motion_planner.noop_tol_deg,
                         config_.safety.init_motion_planner.waypoint_tol_deg));
            if (noop_tol_deg > 0.0 && request_goal_reached(
                    request_left, request_right, target_left, target_right, noop_tol_deg)) {
                // A fresh GUI/key request while the measured joints are already at the
                // requested init pose is a no-op. Do not replay an old Done waypoint and do
                // not stream the raw target for one tick while planning; both show up as
                // "pressing InitMotion changes the pose".
                reanchor_selected_to_measured(request_left, request_right);
                mark_done_at_measured(
                    ex, request_left, request_right, target_left, target_right,
                    "already_at_goal_measured_noop");
                hold_selected(command, ex);
                return true;
            }
            const auto launch_plan = [&]() {
                reanchor_selected_to_measured(request_left, request_right);
                ex.target_left = target_left;
                ex.target_right = target_right;
                ex.left_active = request_left;
                ex.right_active = request_right;
                ex.has_target = true;
                ex.waypoints.clear();
                ex.index = 0;
                ex.status = InitMotionStatus::Planning;
                ex.message = "planning";
                ex.fail_mode = InitMotionPlanResult::FailMode::None;
                ex.start_clear_m = std::numeric_limits<double>::quiet_NaN();
                ex.goal_clear_m = std::numeric_limits<double>::quiet_NaN();
                ex.goal_self_min_clearance_m = std::numeric_limits<double>::quiet_NaN();
                ex.goal_external_min_clearance_m = std::numeric_limits<double>::quiet_NaN();
                ex.goal_nearest_pair_name_a.clear();
                ex.goal_nearest_pair_name_b.clear();
                ex.goal_nearest_pair_category.clear();
                ex.goal_nearest_pair_external = false;
                ex.goal_nearest_pair_disabled_by_rule = false;
                ex.goal_nearest_pair_distance_m = std::numeric_limits<double>::quiet_NaN();
                ex.goal_clear_threshold_self_m = std::numeric_limits<double>::quiet_NaN();
                ex.goal_clear_threshold_external_m = std::numeric_limits<double>::quiet_NaN();
                ex.goal_clear_margin_deficit_m = std::numeric_limits<double>::quiet_NaN();
                ex.clear_threshold_m = std::numeric_limits<double>::quiet_NaN();
                ex.external_clear_threshold_m = std::numeric_limits<double>::quiet_NaN();
                ex.nearest_pair.clear();
                ex.nearest_pair_distance_m = std::numeric_limits<double>::quiet_NaN();
                ex.nearest_pair_external = false;
                ex.tree_start_size = 0;
                ex.tree_goal_size = 0;
                ex.last_iterations = 0;
                ex.last_planning_time_s = 0.0;
                ex.exec_timeout = false;
                ex.exec_stalled = false;
                ex.start_ns = nowSteadyNs();
                PlannerJob job;
                job.requester = requester;
                job.is_linear = false;
                job.start_left = current_left_q;
                job.start_right = current_right_q;
                job.target_left = target_left;
                job.target_right = target_right;
                job.request_left = request_left;
                job.request_right = request_right;
                ex.plan_generation = postPlannerJob(std::move(job));
                std::cerr << "[INFO] JointTarget init_motion: planning collision-free path from measured pose\n";
            };
            // A genuinely new sequence is an explicit operator request.  Re-plan
            // even when the endpoint matches the previous request: the robot may
            // have been moved since completion and the new press must remain usable.
            launch_plan();
            return false;
        };

        if (fresh_request && (request_left || request_right)) {
            ex.request_seen = true;
            ex.request_seq = command.seq;
            const JointArray target_left =
                request_left ? command.left.q_target_deg : current_left_q;
            const JointArray target_right =
                request_right ? command.right.q_target_deg : current_right_q;
            // A selected arm that is still streaming (policy motion, or an earlier init
            // move being re-targeted by a second press) brakes from its last SENT target
            // first; the measured re-anchor + plan follow once it has settled. Arms at rest
            // take the immediate no-op/launch path exactly as before.
            if (!begin_brake(ex, request_left, request_right, target_left, target_right)) {
                if (launch_or_noop(ex, request_left, request_right, target_left, target_right)) {
                    return;
                }
            }
        } else if (ex.brake_pending && ex.status == InitMotionStatus::Planning) {
            const uint64_t now_ns = nowSteadyNs();
            const double brake_age_s = ex.brake_start_ns != 0
                ? static_cast<double>(now_ns - ex.brake_start_ns) * 1e-9
                : 0.0;
            const bool settled = brake_settled(ex);
            const bool timed_out = brake_age_s >= brake_cfg.brake_timeout_sec;
            if (settled || timed_out) {
                std::cerr << "[INFO] JointTarget init_motion: brake-before-plan "
                          << (settled ? "settled" : "TIMED OUT (planning anyway)")
                          << " after " << brake_age_s << " s; planning from measured pose\n";
                if (launch_or_noop(ex, ex.left_active, ex.right_active,
                                   ex.target_left, ex.target_right)) {
                    return;
                }
            }
        }

        // Runaway bound (progress-aware): a committed sequence gives up (Failed -> hold) so a
        // permanently barrier-blocked corner cannot hold motion authority forever. A move that
        // keeps CLOSING on the goal is not killed by the wall-clock budget (a slow escape that
        // crawls out of a near-collision still completes); a genuine STALL (no progress for the
        // stall window) fails closed FAST instead of waiting out the full budget.
        if (sequence_active(ex) && ex.start_ns != 0) {
            const uint64_t now_ns = nowSteadyNs();
            const double age_s = static_cast<double>(now_ns - ex.start_ns) * 1e-9;
            const double stall_window_s =
                std::min(6.0, config_.safety.init_motion_planner.execution_timeout_sec);
            const double since_progress_s = ex.last_progress_ns != 0
                ? static_cast<double>(now_ns - ex.last_progress_ns) * 1e-9 : 0.0;
            const bool absolute_timeout =
                age_s > config_.safety.init_motion_planner.execution_timeout_sec;
            const bool stalled = ex.status == InitMotionStatus::Executing &&
                                 ex.last_progress_ns != 0 && since_progress_s > stall_window_s;
            if (absolute_timeout || stalled) {
                ex.status = InitMotionStatus::Failed;
                ex.message = stalled ? "execution stalled (no progress)" : "execution timeout";
                ex.exec_timeout = absolute_timeout;
                ex.exec_stalled = stalled;
                ex.fail_mode = InitMotionPlanResult::FailMode::None;
                std::cerr << "[WARN] JointTarget init_motion: " << ex.message
                          << " (age=" << age_s << "s, since_progress=" << since_progress_s
                          << "s, wp=" << ex.index << "/" << ex.waypoints.size()
                          << "); holding (fail-closed)\n";
            }
        }

        // Poll the worker plan (non-blocking) and transition Planning -> Executing/Failed.
        PlannerResultSlot pr;
        if (ex.status == InitMotionStatus::Planning && ex.plan_generation != 0 &&
            takePlannerResult(requester, ex.plan_generation, pr)) {
            InitMotionPlanResult result = pr.init_result;
            ex.fail_mode = result.fail_mode;
            ex.start_clear_m = result.start_clear_m;
            ex.goal_clear_m = result.goal_clear_m;
            ex.goal_self_min_clearance_m = result.goal_self_min_clearance_m;
            ex.goal_external_min_clearance_m = result.goal_external_min_clearance_m;
            ex.goal_nearest_pair_name_a = result.goal_nearest_pair_name_a;
            ex.goal_nearest_pair_name_b = result.goal_nearest_pair_name_b;
            ex.goal_nearest_pair_category = result.goal_nearest_pair_category;
            ex.goal_nearest_pair_external = result.goal_nearest_pair_external;
            ex.goal_nearest_pair_disabled_by_rule = result.goal_nearest_pair_disabled_by_rule;
            ex.goal_nearest_pair_distance_m = result.goal_nearest_pair_distance_m;
            ex.goal_clear_threshold_self_m = result.goal_clear_threshold_self_m;
            ex.goal_clear_threshold_external_m = result.goal_clear_threshold_external_m;
            ex.goal_clear_margin_deficit_m = result.goal_clear_margin_deficit_m;
            ex.clear_threshold_m = result.clear_threshold_m;
            ex.external_clear_threshold_m = result.external_clear_threshold_m;
            ex.nearest_pair = result.nearest_pair;
            ex.nearest_pair_distance_m = result.nearest_pair_distance_m;
            ex.nearest_pair_external = result.nearest_pair_external;
            ex.tree_start_size = result.tree_start_size;
            ex.tree_goal_size = result.tree_goal_size;
            ex.last_iterations = result.iterations;
            ex.last_planning_time_s = result.planning_time_s;
            ex.exec_timeout = false;
            ex.exec_stalled = false;
            if (result.success && !result.waypoints.empty()) {
                ex.waypoints = std::move(result.waypoints);
                ex.index = 0;
                ex.escape_waypoints = result.escape_waypoints;
                ex.status = InitMotionStatus::Executing;
                ex.message = "executing";
                // Reset progress tracking for the progress-aware execution timeout.
                ex.best_dist_deg = std::numeric_limits<double>::infinity();
                ex.last_progress_ns = nowSteadyNs();
                ex.last_exec_log_ns = 0;
                std::cerr << "[INFO] JointTarget init_motion: plan found ("
                          << ex.waypoints.size() << " waypoints, "
                          << result.escape_waypoints << " escape, "
                          << result.iterations << " iters, "
                          << result.planning_time_s << " s); streaming\n";
            } else {
                ex.status = InitMotionStatus::Failed;
                ex.message = result.message;
                std::cerr << "[WARN] JointTarget init_motion: planning failed (" << result.message
                          << "); holding (fail-closed)\n";
            }
        }

        switch (ex.status) {
            case InitMotionStatus::Executing: {
                // Follow the gradient-escape head precisely (escape_waypoints), then
                // pure-pursuit. Active arms are tracked against MEASURED joints so init
                // cannot complete just because the sent target reached the goal while
                // the physical robot is still lagging behind the SMD/servo filters.
                JointArray pursue_left = (ex.left_active || freeze_other_arm)
                    ? current_left_q
                    : (!ex.waypoints.empty() ? ex.waypoints.back().first : ex.target_left);
                JointArray pursue_right = (ex.right_active || freeze_other_arm)
                    ? current_right_q
                    : (!ex.waypoints.empty() ? ex.waypoints.back().second : ex.target_right);
                const std::size_t index_before_pursuit = ex.index;
                PursuitStep pursuit =
                    pursueWaypointsStep(
                        ex.waypoints,
                        pursue_left,
                        pursue_right,
                        ex.index,
                        config_.safety.init_motion_planner.waypoint_tol_deg,
                        config_.safety.init_motion_planner.execution_lookahead_deg,
                        ex.escape_waypoints
                    );
                std::pair<JointArray, JointArray> wp = {pursuit.left, pursuit.right};
                bool done = false;
                if (!ex.waypoints.empty()) {
                    const auto& goal_wp = ex.waypoints.back();
                    done = active_goal_reached(
                        ex,
                        goal_wp,
                        config_.safety.init_motion_planner.waypoint_tol_deg
                    );
                    // Progress = max-joint distance from the current MEASURED pose to the final
                    // waypoint. Closing on it (by > a noise floor) refreshes the stall timer.
                    const double dist = active_goal_dist(ex, goal_wp);
                    const uint64_t now_ns = nowSteadyNs();
                    if (ex.index > index_before_pursuit) {
                        ex.last_progress_ns = now_ns;
                    }
                    if (dist < ex.best_dist_deg - 0.05) {
                        ex.best_dist_deg = dist;
                        ex.last_progress_ns = now_ns;
                    }
                    // Throttled (~1 Hz) streaming-progress diagnostic: which waypoint, whether
                    // still in the escape head, and how far from the goal — so a stall location
                    // is visible in the log.
                    if (ex.last_exec_log_ns == 0 || (now_ns - ex.last_exec_log_ns) > 1000000000ULL) {
                        ex.last_exec_log_ns = now_ns;
                        std::cerr << "[INFO] JointTarget init_motion: streaming wp " << ex.index << "/"
                                  << ex.waypoints.size() << " (escape=" << ex.escape_waypoints
                                  << ", actual_dist_to_goal=" << dist << " deg, best=" << ex.best_dist_deg
                                  << " deg)\n";
                    }
                }
                if (done) {
                    ex.status = InitMotionStatus::Done;
                    ex.message = "done";
                    std::cerr << "[INFO] JointTarget init_motion: reached init pose\n";
                }
                rewrite_selected(command, ex, wp.first, wp.second);
                if (!ex.waypoints.empty()) {
                    const auto& stop_wp = ex.waypoints.back();
                    set_arrival_stop(command, ex, stop_wp.first, stop_wp.second);
                }
                break;
            }
            case InitMotionStatus::Done: {
                // Terminal ownership is Hold, not a perpetually replayed
                // JointTarget.  previous_sent is already the realized final
                // waypoint, so this preserves restoring authority without letting
                // the cached Init Motion packet re-enter/re-anchor the sequencer.
                hold_selected(command, ex);
                break;
            }
            case InitMotionStatus::Planning:
            case InitMotionStatus::Failed:
            default:
                if (ex.status == InitMotionStatus::Planning && ex.brake_pending) {
                    // Brake-before-plan: decelerate the streaming arm(s) from the last sent
                    // target; the plan launches once they have settled.
                    brake_selected(command, ex);
                    break;
                }
                // Hold in place: while planning, or fail-closed after a planning failure.
                hold_selected(command, ex);
                break;
        }
    };

    // Drive each arm's exec at most once per tick. A FRESH init request for an arm
    // (re)launches + advances ITS exec; any OTHER in-flight exec is CONTINUED. Crucially
    // the continuation is NOT gated on is_init: a fresh single-arm init for one arm must
    // not pause the OTHER arm's in-flight init while its command is fresh — both run
    // concurrently. (Both-arm init drives the left exec for both arms; the right exec was
    // reset above, so it is idle and not continued here.)
    const bool left_exec_requested = is_init && left_init;
    const bool right_exec_requested = is_init && right_init && !left_init;
    // Freshness is a NEW OPERATOR REQUEST, not a new packet. A one-shot GUI command is
    // re-delivered from the command buffer with a CONSTANT seq, so seq alone used to be a
    // good proxy — but policy_runner's arm_init latch (ArmInitOverrideController::
    // compose_intent) re-emits the SAME logical InitMotion every tick, and every packet
    // carries a fresh seq. That made each tick look like a new press: launch_plan() reset
    // status to Planning and cleared the waypoints, so the async planner's result was
    // discarded before takePlannerResult() could consume it, and the arm sat held in place
    // (rewrite_selected holds while Planning) until the stream stopped. Evidence
    // (2026-08-13, servo_log_20260813_155705): 1722 "planning collision-free path" vs 2
    // "plan found"; the right arm was pinned in `planning` for ~3.7k ticks and reached the
    // init pose 118 ticks after flow-infer was killed. A committed sequence therefore
    // relaunches only on a materially different goal; an idle/done/failed exec still
    // replans on the next distinct seq, so pressing again after completion still works.
    const double init_request_tol_deg = std::max(
        0.0,
        std::max(config_.safety.init_motion_planner.noop_tol_deg,
                 config_.safety.init_motion_planner.waypoint_tol_deg));
    const auto exec_is_fresh = [&](const InitMotionExec& ex, bool requested,
                                   bool request_left, bool request_right) {
        if (!requested) return false;
        InitMotionRequestView view;
        view.request_seen = ex.request_seen;
        view.request_seq = ex.request_seq;
        view.sequence_active = sequence_active(ex);
        view.has_target = ex.has_target;
        view.left_active = ex.left_active;
        view.right_active = ex.right_active;
        view.target_left = ex.target_left;
        view.target_right = ex.target_right;
        return initMotionRequestIsFresh(
            view, command.seq, request_left, request_right,
            command.left.q_target_deg, command.right.q_target_deg, init_request_tol_deg);
    };
    const bool left_exec_fresh =
        exec_is_fresh(left_init_motion_exec_, left_exec_requested, true, right_init);
    const bool right_exec_fresh =
        exec_is_fresh(right_init_motion_exec_, right_exec_requested, false, true);
    if (left_exec_requested) {
        process_exec(
            left_init_motion_exec_, PlannerRequester::LeftInit, true, right_init,
            left_exec_fresh);
    } else if (sequence_active(left_init_motion_exec_)) {
        process_exec(left_init_motion_exec_, PlannerRequester::LeftInit,
                     left_init_motion_exec_.left_active,
                     left_init_motion_exec_.right_active, false);
    }
    if (right_exec_requested) {
        process_exec(
            right_init_motion_exec_, PlannerRequester::RightInit, false, true,
            right_exec_fresh);
    } else if (sequence_active(right_init_motion_exec_)) {
        process_exec(right_init_motion_exec_, PlannerRequester::RightInit,
                     right_init_motion_exec_.left_active,
                     right_init_motion_exec_.right_active, false);
    }
    return command;
}

bool initMotionRequestIsFresh(
    const InitMotionRequestView& ex,
    uint64_t command_seq,
    bool request_left,
    bool request_right,
    const JointArray& command_target_left,
    const JointArray& command_target_right,
    double tol_deg) {
    // First request this exec has ever seen.
    if (!ex.request_seen) return true;
    // Same packet re-served from the command buffer (one-shot GUI command held across
    // ticks) — already handled on the tick it arrived.
    if (ex.request_seq == command_seq) return false;
    // Distinct seq with no committed sequence in flight: a genuine new press. Replan even
    // when the endpoint matches the previous one — the robot may have been moved since
    // that sequence completed, so the new press must remain usable.
    if (!ex.sequence_active) return true;
    // Distinct seq while Planning/Executing. This is the streaming-client case: relaunch
    // ONLY for a materially different goal, otherwise the per-tick re-emission would reset
    // the planner forever and the arm would never leave the hold.
    if (!ex.has_target) return true;
    if (ex.left_active != request_left || ex.right_active != request_right) return true;
    const double tol = std::max(0.0, tol_deg);
    for (int i = 0; i < kDof; ++i) {
        if (request_left && std::abs(command_target_left[i] - ex.target_left[i]) > tol) {
            return true;
        }
        if (request_right && std::abs(command_target_right[i] - ex.target_right[i]) > tol) {
            return true;
        }
    }
    return false;
}

double sentJointSpeedDegS(
    const JointArray& prev_sent_deg,
    const JointArray& prevprev_sent_deg,
    double dt_sec) {
    if (!(dt_sec > 0.0) || !std::isfinite(dt_sec)) return 0.0;
    double speed = 0.0;
    for (int i = 0; i < kDof; ++i) {
        const double d = prev_sent_deg[i] - prevprev_sent_deg[i];
        if (!std::isfinite(d)) continue;
        speed = std::max(speed, std::abs(d) / dt_sec);
    }
    return speed;
}

InitMotionBrakePlan planInitMotionBrake(
    const JointArray& prev_sent_deg,
    const JointArray& prevprev_sent_deg,
    double dt_sec,
    double smd_natural_frequency_hz,
    double enter_deg_s,
    double max_travel_deg) {
    InitMotionBrakePlan plan;
    plan.goal = prev_sent_deg;
    if (!(dt_sec > 0.0) || !std::isfinite(dt_sec)) return plan;
    plan.max_speed_deg_s = sentJointSpeedDegS(prev_sent_deg, prevprev_sent_deg, dt_sec);
    plan.needed = plan.max_speed_deg_s > std::max(0.0, enter_deg_s);
    if (!plan.needed) return plan;
    const double wn = smd_natural_frequency_hz > 0.0 && std::isfinite(smd_natural_frequency_hz)
        ? 2.0 * M_PI * smd_natural_frequency_hz
        : 0.0;
    const double travel_cap = std::isfinite(max_travel_deg) ? std::max(0.0, max_travel_deg) : 0.0;
    for (int i = 0; i < kDof; ++i) {
        const double d = prev_sent_deg[i] - prevprev_sent_deg[i];
        if (!std::isfinite(d) || wn <= 0.0) continue;  // rate-limited stop at prev_sent
        const double dq = d / dt_sec;
        plan.goal[i] = prev_sent_deg[i] + std::clamp(dq / wn, -travel_cap, travel_cap);
    }
    return plan;
}

PursuitStep pursueWaypointsStep(
    const std::vector<std::pair<JointArray, JointArray>>& waypoints,
    const JointArray& cur_left,
    const JointArray& cur_right,
    std::size_t& index,
    double waypoint_tol_deg,
    double lookahead_deg,
    int escape_count
) {
    PursuitStep out;
    out.left = cur_left;
    out.right = cur_right;
    if (waypoints.empty()) {
        return out;
    }
    const std::size_t n = waypoints.size();
    if (index >= n) index = n - 1;

    const auto reached = [&](const JointArray& a, const JointArray& b) {
        for (int i = 0; i < kDof; ++i) {
            if (std::abs(a[i] - b[i]) > waypoint_tol_deg) return false;
        }
        return true;
    };
    // Joint-space chord (max per-joint angle over BOTH arms) from the current sent pose.
    const auto chord = [&](const std::pair<JointArray, JointArray>& w) {
        double m = 0.0;
        for (int i = 0; i < kDof; ++i) {
            m = std::max(m, std::abs(cur_left[i] - w.first[i]));
            m = std::max(m, std::abs(cur_right[i] - w.second[i]));
        }
        return m;
    };
    // Fraction of segment [a,b] that the current pose projects onto, in the combined
    // (both-arm) joint space. >= 1 means the pose has advanced past b. This is a
    // PROJECTION, not a proximity test, so a lookahead chord that cuts a corner still
    // advances progress instead of stalling at an apex node it never passed within tol.
    const auto segFraction = [&](const std::pair<JointArray, JointArray>& a,
                                 const std::pair<JointArray, JointArray>& b) {
        double num = 0.0;
        double den = 0.0;
        for (int i = 0; i < kDof; ++i) {
            const double dl = b.first[i] - a.first[i];
            const double dr = b.second[i] - a.second[i];
            num += (cur_left[i] - a.first[i]) * dl + (cur_right[i] - a.second[i]) * dr;
            den += dl * dl + dr * dr;
        }
        return den > 1e-9 ? num / den : 1.0;  // degenerate segment -> already passed
    };

    // Progress pointer: monotonic projection advance, with endpoint proximity as a
    // fallback. Projection keeps corner-cutting from freezing at apex nodes; proximity
    // keeps an asymptotic tracker from freezing when the carrot is exactly the segment
    // endpoint (gradient-escape head, lookahead 0) and the pose settles just short of
    // projecting past it.
    while (index + 1 < n &&
           (segFraction(waypoints[index], waypoints[index + 1]) >= 1.0 ||
            (reached(cur_left, waypoints[index + 1].first) &&
             reached(cur_right, waypoints[index + 1].second)))) {
        ++index;
    }
    // Inside the gradient-escape head (the leading sub-threshold waypoints), follow the
    // path PRECISELY — lookahead 0 aims at the immediate next waypoint only, so the
    // pure-pursuit chord cannot cut the escape corner back into the obstacle it is
    // climbing out of (which would trip the reactive barrier and stall). Past the escape
    // the normal lookahead resumes for near-max-velocity streaming.
    const bool in_escape = escape_count > 0 &&
                           index < static_cast<std::size_t>(escape_count);
    const double lookahead_eff = in_escape ? 0.0 : lookahead_deg;
    // Pure-pursuit: aim at the farthest forward waypoint still within `lookahead_eff` of
    // the current pose, so the downstream servo runs near max velocity (no stop-and-go at
    // every densified node). Start at index+1 so the command always leads forward and
    // never re-aims at the node behind us. The path stays the planned collision-free
    // polyline; any corner the chord cuts into a keep-out is braked by the reactive
    // barrier (slow only at that corner, never collide).
    std::size_t tgt = std::min(index + 1, n - 1);
    while (tgt + 1 < n && chord(waypoints[tgt + 1]) <= lookahead_eff) {
        ++tgt;
    }
    out.left = waypoints[tgt].first;
    out.right = waypoints[tgt].second;
    // Slide the carrot ALONG the segment past `tgt` so it sits at (about) the
    // lookahead distance, instead of snapping to the node.
    //
    // Snapping makes the carrot distance a sawtooth: it grows toward the
    // lookahead, then drops the instant `tgt` advances. The downstream servo
    // speed follows that distance, so the commanded speed sawtooths at the
    // waypoint advance rate -- measured on hardware as a ~23 % velocity ripple
    // at 9.5 Hz against a 9.19 Hz node rate, identical on all six joints of an
    // arm because it is the scalar distance that modulates. The control-box LPF
    // used to mask it; with servo_alpha at the LPF-off value it reaches the arm
    // as a visible tremor.
    //
    // Interpolating in chord distance keeps that distance ~constant, which is
    // what pure pursuit is supposed to do. Untouched when there is no next
    // segment (converge on the final waypoint) or in the escape head, where
    // lookahead 0 must aim exactly at the node.
    if (lookahead_eff > 0.0 && tgt + 1 < n) {
        const double d0 = chord(waypoints[tgt]);
        const double d1 = chord(waypoints[tgt + 1]);
        if (d1 > d0 + 1e-9 && d0 < lookahead_eff) {
            const double frac = std::clamp((lookahead_eff - d0) / (d1 - d0), 0.0, 1.0);
            for (int i = 0; i < kDof; ++i) {
                out.left[i] = waypoints[tgt].first[i] +
                    frac * (waypoints[tgt + 1].first[i] - waypoints[tgt].first[i]);
                out.right[i] = waypoints[tgt].second[i] +
                    frac * (waypoints[tgt + 1].second[i] - waypoints[tgt].second[i]);
            }
        }
    }
    // Finished only when the current sent pose has actually settled at the final
    // waypoint (within tol on every joint), independent of the progress pointer.
    out.done = reached(cur_left, waypoints.back().first) &&
               reached(cur_right, waypoints.back().second);
    return out;
}

std::pair<JointArray, JointArray> DualArmServoLoop::pursueWaypoints(
    const std::vector<std::pair<JointArray, JointArray>>& waypoints,
    std::size_t& index,
    bool& done,
    int escape_count
) const {
    const PursuitStep step = pursueWaypointsStep(
        waypoints,
        left_prev_sent_q_deg_,
        right_prev_sent_q_deg_,
        index,
        config_.safety.init_motion_planner.waypoint_tol_deg,
        config_.safety.init_motion_planner.execution_lookahead_deg,
        escape_count);
    done = step.done;
    return {step.left, step.right};
}

DualArmCommand DualArmServoLoop::applyCollisionFreeLinearMove(
    DualArmCommand command,
    const RobotState& left_state,
    const RobotState& right_state
) {
    (void)left_state;
    (void)right_state;
    const bool enabled = config_.cartesian_control.linear_move.collision_free &&
                         init_motion_planner_ != nullptr;
    const bool is_lin = command.left.mode == ControlMode::TcpLinearMove ||
                        command.right.mode == ControlMode::TcpLinearMove;
    auto& lx = linear_move_exec_;
    const bool sequence_active = lx.status == LinearMoveStatus::Deciding ||
                                 lx.status == LinearMoveStatus::Detour;
    const bool deadman_hold = isSyntheticHoldCommand(command);

    const auto as_joint_target = [](DualArmCommand& c, const JointArray& l, const JointArray& r) {
        c.left.mode = ControlMode::JointTarget;
        c.right.mode = ControlMode::JointTarget;
        c.left.q_target_deg = l;
        c.right.q_target_deg = r;
        c.left.has_joint_target = true;
        c.right.has_joint_target = true;
    };
    const auto as_hold = [](DualArmCommand& c) {
        c.left.mode = ControlMode::Hold;
        c.right.mode = ControlMode::Hold;
    };

    if (!enabled) {
        if (lx.status != LinearMoveStatus::Idle) lx = LinearMoveExec{};
        return command;  // feature off -> normal straight MoveL / other handling
    }

    if (!is_lin) {
        // Continue an in-flight Deciding/Detour across command staleness (deadman),
        // mirroring InitMotion; any explicit non-linear command cancels. NOTE: a
        // decided Straight is intentionally NOT continued here — its in-flight Cartesian
        // path is carried to completion by the executor's own linear-path continuation.
        if (sequence_active && deadman_hold) {
            // fall through to the status machine without relaunching.
        } else {
            if (lx.status != LinearMoveStatus::Idle) lx = LinearMoveExec{};
            return command;
        }
    }

    // Determine the active arms + target poses for a fresh linear command.
    if (is_lin) {
        const bool left_active = command.left.mode == ControlMode::TcpLinearMove &&
                                 command.left.has_tcp_target;
        const bool right_active = command.right.mode == ControlMode::TcpLinearMove &&
                                  command.right.has_tcp_target;
        const bool slerp = (command.left.has_linear_move_orientation_mode
                                ? command.left.linear_move_orientation_mode
                                : config_.cartesian_control.linear_move.default_orientation_mode) ==
                           LinearMoveOrientationMode::Slerp;
        const auto pose_near = [](const Pose6D& a, const Pose6D& b) {
            return std::abs(a.x - b.x) < 1e-6 && std::abs(a.y - b.y) < 1e-6 &&
                   std::abs(a.z - b.z) < 1e-6 && std::abs(a.rx - b.rx) < 1e-6 &&
                   std::abs(a.ry - b.ry) < 1e-6 && std::abs(a.rz - b.rz) < 1e-6;
        };
        const bool new_target = !lx.has_target ||
            left_active != lx.left_active || right_active != lx.right_active ||
            !pose_near(command.left.tcp_target_stand, lx.target_left) ||
            !pose_near(command.right.tcp_target_stand, lx.target_right);
        const auto launch_decide = [&]() {
            lx.has_target = true;
            lx.left_active = left_active;
            lx.right_active = right_active;
            lx.slerp = slerp;
            lx.target_left = command.left.tcp_target_stand;
            lx.target_right = command.right.tcp_target_stand;
            lx.waypoints.clear();
            lx.index = 0;
            lx.status = LinearMoveStatus::Deciding;
            lx.message = "deciding";
            lx.start_ns = nowSteadyNs();
            const JointArray start_left = left_prev_sent_q_deg_;
            const JointArray start_right = right_prev_sent_q_deg_;
            const Pose6D goal_left = command.left.tcp_target_stand;
            const Pose6D goal_right = command.right.tcp_target_stand;
            const int samples = config_.cartesian_control.linear_move.collision_check_samples;
            PlannerJob job;
            job.requester = PlannerRequester::Linear;
            job.is_linear = true;
            job.start_left = start_left;
            job.start_right = start_right;
            job.lin_left_active = left_active;
            job.lin_right_active = right_active;
            job.lin_goal_left = goal_left;
            job.lin_goal_right = goal_right;
            job.slerp = slerp;
            job.lin_samples = samples;
            lx.plan_generation = postPlannerJob(std::move(job));
            std::cerr << "[INFO] TcpLinearMove(collision-free): checking straight path\n";
        };
        if (lx.status == LinearMoveStatus::Idle ||
            ((lx.status == LinearMoveStatus::Straight ||
              lx.status == LinearMoveStatus::Detour ||
              lx.status == LinearMoveStatus::Done ||
              lx.status == LinearMoveStatus::Failed) && new_target)) {
            launch_decide();
        }
    }

    // Runaway bound (same as InitMotion).
    if (sequence_active && lx.start_ns != 0) {
        const double age_s = static_cast<double>(nowSteadyNs() - lx.start_ns) * 1e-9;
        if (age_s > config_.safety.init_motion_planner.execution_timeout_sec) {
            lx.status = LinearMoveStatus::Failed;
            lx.message = "execution timeout";
            std::cerr << "[WARN] TcpLinearMove(collision-free): timeout; holding\n";
        }
    }

    // Poll the worker decision.
    PlannerResultSlot pr;
    if (lx.status == LinearMoveStatus::Deciding && lx.plan_generation != 0 &&
        takePlannerResult(PlannerRequester::Linear, lx.plan_generation, pr)) {
        InitMotionLinearResult r = pr.linear_result;
        // Diagnostic: goal-vs-current joint delta. ~0 deg means the requested TCP target
        // is essentially the current pose (e.g., the GUI target marker is following
        // current TCP, not parked at a destination) -> the move is a near no-op.
        if (r.goal_vs_start_max_deg < 1.0) {
            std::cerr << "[WARN] TcpLinearMove(collision-free): goal is only "
                      << r.goal_vs_start_max_deg << " deg / " << r.goal_vs_start_cart_m
                      << " m from current pose -> near no-op (is the target marker "
                         "following current TCP? drag it to a destination first)\n";
        }
        if (r.decision == InitMotionLinearResult::Decision::Straight) {
            lx.status = LinearMoveStatus::Straight;
            lx.message = "straight";
            std::cerr << "[INFO] TcpLinearMove(collision-free): straight path clear; running MoveL"
                      << " (goal_vs_current=" << r.goal_vs_start_max_deg << " deg / "
                      << r.goal_vs_start_cart_m << " m)\n";
        } else if (r.decision == InitMotionLinearResult::Decision::Detour) {
            lx.waypoints = std::move(r.waypoints);
            lx.index = 0;
            lx.escape_waypoints = r.escape_waypoints;
            lx.status = LinearMoveStatus::Detour;
            lx.message = "detour";
            std::cerr << "[INFO] TcpLinearMove(collision-free): straight path blocked; "
                      << "streaming collision-free detour (" << lx.waypoints.size()
                      << " waypoints, goal_vs_current=" << r.goal_vs_start_max_deg << " deg / "
                      << r.goal_vs_start_cart_m << " m)\n";
        } else {
            lx.status = LinearMoveStatus::Failed;
            lx.message = r.message;
            std::cerr << "[WARN] TcpLinearMove(collision-free): " << r.message
                      << "; holding (fail-closed)\n";
        }
    }

    switch (lx.status) {
        case LinearMoveStatus::Straight:
            // Pass the original TcpLinearMove through untouched -> the Cartesian executor
            // runs the exact straight MoveL (and its own finite-path continuation carries
            // it to completion across command/lease staleness).
            return command;
        case LinearMoveStatus::Detour: {
            bool done = false;
            const std::pair<JointArray, JointArray> wp =
                pursueWaypoints(lx.waypoints, lx.index, done, lx.escape_waypoints);
            if (done) {
                lx.status = LinearMoveStatus::Done;
                lx.message = "done";
                std::cerr << "[INFO] TcpLinearMove(collision-free): detour reached target\n";
            }
            as_joint_target(command, wp.first, wp.second);
            return command;
        }
        case LinearMoveStatus::Done:
            // Hold at the detour goal.
            as_joint_target(command, lx.waypoints.empty()
                                         ? left_prev_sent_q_deg_
                                         : lx.waypoints.back().first,
                            lx.waypoints.empty() ? right_prev_sent_q_deg_
                                                 : lx.waypoints.back().second);
            return command;
        case LinearMoveStatus::Deciding:
        case LinearMoveStatus::Failed:
        default:
            as_hold(command);
            return command;
    }
}

bool DualArmServoLoop::commandBlockedByReadOnly(const DualArmCommand& command) const {
    return isReadOnlyBlockedMode(command.left.mode) || isReadOnlyBlockedMode(command.right.mode);
}

bool DualArmServoLoop::readOnlyMode() const {
    return !config_.servo.send_servo_commands;
}

bool DualArmServoLoop::workerIoMode() const {
    return config_.servo.io_model == ServoIoModel::Worker;
}

bool DualArmServoLoop::rbpodoAsyncIoMode() const {
    return config_.servo.rbpodo_async_streaming.enable;
}

bool DualArmServoLoop::workerBackedIoMode() const {
    return workerIoMode() || rbpodoAsyncIoMode();
}

DualArmServoLoop::ArmControlContext DualArmServoLoop::armContext(ArmId arm) {
    if (arm == ArmId::Left) {
        return ArmControlContext{
            ArmId::Left,
            config_.left_mount,
            config_.left_robot,
            left_abc_telemetry_,
            left_cartesian_servo_path_,
            left_chunk_engage_waiting_,
            left_chunk_engage_wait_start_sec_,
            left_chunk_follower_,
            left_chunk_follower_built_,
            left_chunk_follower_fault_request_,
            left_chunk_follower_reanchor_count_,
            left_chunk_follower_reanchor_log_ns_,
            left_chunk_follower_warm_resume_count_,
            left_chunk_submitted_recv_seq_,
            left_chunk_submitted_wire_seq_,
            left_controller_sim_physical_baseline_q_deg_,
            left_delta_twist_follower_,
            left_fault_hold_q_deg_,
            left_follower_output_smd_,
            left_freedrive_deadline_ns_,
            left_freedrive_stage_,
            left_freedrive_stage_entered_ns_,
            left_init_motion_exec_,
            left_last_cartesian_solve_,
            left_latched_cartesian_target_,
            left_latched_fault_context_,
            left_output_ma_,
            left_pose_track_profile_name_,
            left_pose_track_smd_,
            left_prevprev_sent_q_deg_,
            left_prev_sent_q_deg_,
            left_robot_,
            left_safety_intervention_last_ns_,
            left_safety_tracking_,
            left_traj_filter_,
            left_worker_
        };
    }
    return ArmControlContext{
        ArmId::Right,
        config_.right_mount,
        config_.right_robot,
            right_abc_telemetry_,
            right_cartesian_servo_path_,
            right_chunk_engage_waiting_,
            right_chunk_engage_wait_start_sec_,
            right_chunk_follower_,
            right_chunk_follower_built_,
            right_chunk_follower_fault_request_,
            right_chunk_follower_reanchor_count_,
            right_chunk_follower_reanchor_log_ns_,
            right_chunk_follower_warm_resume_count_,
            right_chunk_submitted_recv_seq_,
            right_chunk_submitted_wire_seq_,
            right_controller_sim_physical_baseline_q_deg_,
            right_delta_twist_follower_,
            right_fault_hold_q_deg_,
            right_follower_output_smd_,
            right_freedrive_deadline_ns_,
            right_freedrive_stage_,
            right_freedrive_stage_entered_ns_,
            right_init_motion_exec_,
            right_last_cartesian_solve_,
            right_latched_cartesian_target_,
            right_latched_fault_context_,
            right_output_ma_,
            right_pose_track_profile_name_,
            right_pose_track_smd_,
            right_prevprev_sent_q_deg_,
            right_prev_sent_q_deg_,
            right_robot_,
            right_safety_intervention_last_ns_,
            right_safety_tracking_,
            right_traj_filter_,
            right_worker_
    };
}

bool crossArmPeerUsable(std::uint64_t published_tick, std::uint64_t now_tick,
                        std::uint64_t max_age_ticks) {
    if (published_tick == 0) return true;
    if (now_tick <= published_tick) return true;
    return now_tick - published_tick <= max_age_ticks;
}

void DualArmServoLoop::publishCrossArmStatus(ArmId arm, bool cartesian_servo_ok) {
    CrossArmStatus& slot = cross_arm_status_[arm == ArmId::Left ? 0 : 1];
    slot.cartesian_servo_ok.store(cartesian_servo_ok, std::memory_order_relaxed);
    // Store the tick LAST so a reader that sees a fresh tick is guaranteed to see
    // the value that went with it.
    slot.published_tick.store(cross_arm_tick_, std::memory_order_release);
}

bool DualArmServoLoop::peerCartesianServoOk(ArmId arm) const {
    const CrossArmStatus& peer = cross_arm_status_[arm == ArmId::Left ? 1 : 0];
    const std::uint64_t published = peer.published_tick.load(std::memory_order_acquire);
    if (!crossArmPeerUsable(published, cross_arm_tick_, cross_arm_max_age_ticks_)) {
        return false;  // fail-closed: a stale peer is not evidence of health
    }
    if (published == 0) return true;  // nothing published yet (startup)
    return peer.cartesian_servo_ok.load(std::memory_order_relaxed);
}

bool DualArmServoLoop::workerOwnsSendCadence() const {
    // When queue sync is on, each worker sends on its OWN trimmed period. The
    // loop must then hand the setpoint over and move on: blocking on
    // waitForSendResult would re-couple the loop to the worker and, with two
    // workers on two different box clocks, drag the loop to the slower of them
    // every tick -- which is exactly the coupling per-arm cadence exists to
    // remove.
    return workerIoMode() && config_.queue_sync.enable;
}

bool DualArmServoLoop::motionAllowed() const {
    // The explicit ArmMotion arming step is no longer required: motion is accepted
    // whenever the robot is connected and healthy. ConnectedHold (fresh connect / after
    // DisarmMotion) now allows motion just like ArmedHold/Running, so a motion command
    // (incl. InitMotion) executes without a preceding ArmMotion. Disconnected and every
    // latched/emergency state still block (handled here by omission + the fault/E-stop
    // short-circuits upstream). ArmMotion/DisarmMotion remain accepted as no-op state
    // labels for backward compatibility (e.g. policy_runner still emits ArmMotion).
    const ServerMotionState state = motion_state_.load();
    return state == ServerMotionState::ConnectedHold ||
           state == ServerMotionState::ArmedHold ||
           state == ServerMotionState::Running;
}

bool DualArmServoLoop::isRealMode() const {
    return config_.left_robot.run_mode == RunMode::Real || config_.right_robot.run_mode == RunMode::Real;
}

std::string DualArmServoLoop::currentSendPolicy() const {
    const ServerMotionState state = motion_state_.load();
    if (state == ServerMotionState::EmergencyLatched) {
        return "emergency_latched";
    }
    if (readOnlyMode()) {
        return "read_only";
    }
    if (fault_latched_.load() || state == ServerMotionState::FaultLatched) {
        return "fault_latched";
    }
    if (anyFreedriveActive()) {
        // Direct teaching arming or active on at least one arm: send no servo_j to
        // either controller. Suppression starts at Quiesce so the controller can
        // settle to idle before freedrive_teach_on (else M151). The freedrive arm
        // is hand-guided; the other holds at its last reference. Recoverable.
        return "freedrive";
    }
    if (controllerSimulationMotionRequired(config_) &&
        !controllerSimulationMotionGateOpen(config_)) {
        return "controller_simulation_gate_closed";
    }
    return "send_servo_j";
}

bool DualArmServoLoop::clearFaultLatch(RobotState& left_state, RobotState& right_state) {
    clearLatchedCartesianTargets();
    const uint64_t reset_start_ns = nowSteadyNs();
    const uint64_t reset_timeout_ns = timeoutNs(config_.servo.command_timeout_sec, 1'000'000'000);
    const uint64_t reset_deadline_ns = addDeadlineNs(reset_start_ns, reset_timeout_ns);
    if (workerBackedIoMode()) {
        const BackendResult<RobotState> left_reset = left_worker_
            ? left_worker_->resetFault(tick_, reset_deadline_ns)
            : BackendResult<RobotState>{
                false,
                BackendOp::ResetFault,
                RobotState{},
                backendError(
                    BackendErrorKind::RobotDisconnected,
                    "left worker unavailable for resetFault",
                    "",
                    "left_worker_unavailable"
                ),
                makeBackendTiming(reset_start_ns, nowSteadyNs())
            };
        const BackendResult<RobotState> right_reset = right_worker_
            ? right_worker_->resetFault(tick_, reset_deadline_ns)
            : BackendResult<RobotState>{
                false,
                BackendOp::ResetFault,
                RobotState{},
                backendError(
                    BackendErrorKind::RobotDisconnected,
                    "right worker unavailable for resetFault",
                    "",
                    "right_worker_unavailable"
                ),
                makeBackendTiming(reset_start_ns, nowSteadyNs())
            };
        const bool left_reset_ok = left_reset.ok;
        const bool right_reset_ok = right_reset.ok;
        if (!left_reset_ok || !right_reset_ok) {
            std::cerr << "[WARN] fault latch remains: worker resetFault failed"
                      << " left=" << left_reset.error.name << ":" << left_reset.error.message
                      << " right=" << right_reset.error.name << ":" << right_reset.error.message << "\n";
            return false;
        }
    } else {
        const BackendResult<RobotState> left_reset = left_robot_
            ? left_robot_->resetFault()
            : BackendResult<RobotState>{};
        const BackendResult<RobotState> right_reset = right_robot_
            ? right_robot_->resetFault()
            : BackendResult<RobotState>{};
        const bool left_reset_ok = left_reset.ok;
        const bool right_reset_ok = right_reset.ok;
        if (!left_reset_ok || !right_reset_ok) {
            std::cerr << "[WARN] fault latch remains: backend resetFault failed"
                      << " left=" << left_reset.error.name << ":" << left_reset.error.message
                      << " right=" << right_reset.error.name << ":" << right_reset.error.message << "\n";
            return false;
        }
    }

    RobotState fresh_left;
    RobotState fresh_right;
    if (!readRobotStates(fresh_left, fresh_right) ||
        !isValidJointState(fresh_left) ||
        !isValidJointState(fresh_right)) {
        std::cerr << "[WARN] fault latch remains: reset did not produce fresh valid robot state\n";
        return false;
    }

    left_state = fresh_left;
    right_state = fresh_right;

    std::lock_guard<std::mutex> lock(state_mutex_);
    fault_latched_.store(false);
    fault_verdict_.store(SafetyVerdict::Ok);
    latched_fault_reason_.store(SafetyVerdict::Ok);
    fault_reason_.clear();
    latched_fault_context_.reset();
    left_latched_fault_context_.reset();
    right_latched_fault_context_.reset();
    left_prev_sent_q_deg_ = chooseSafeHoldTarget(
        ArmId::Left, left_state, left_prev_sent_q_deg_);
    right_prev_sent_q_deg_ = chooseSafeHoldTarget(
        ArmId::Right, right_state, right_prev_sent_q_deg_);
    left_prevprev_sent_q_deg_ = left_prev_sent_q_deg_;
    right_prevprev_sent_q_deg_ = right_prev_sent_q_deg_;
    left_controller_sim_physical_baseline_q_deg_ = left_state.q_actual_deg;
    right_controller_sim_physical_baseline_q_deg_ = right_state.q_actual_deg;
    left_fault_hold_q_deg_ = left_prev_sent_q_deg_;
    right_fault_hold_q_deg_ = right_prev_sent_q_deg_;
    resetReferenceSupervisionState(this);
    setMotionState(ServerMotionState::ConnectedHold);
    std::cerr << "[INFO] fault latch cleared\n";
    return true;
}

void DualArmServoLoop::latchFault(
    SafetyVerdict verdict,
    const std::string& reason,
    const RobotState& left_state,
    const RobotState& right_state,
    const std::optional<FaultContext>& context
) {
    LatchedDualFaultContext contexts;
    contexts.top_level = context;
    if (context.has_value()) {
        if (context->arm == ArmId::Left) {
            contexts.left = context;
        } else {
            contexts.right = context;
        }
    }
    latchFault(verdict, reason, left_state, right_state, contexts);
}

void DualArmServoLoop::latchFault(
    SafetyVerdict verdict,
    const std::string& reason,
    const RobotState& left_state,
    const RobotState& right_state,
    const LatchedDualFaultContext& contexts
) {
    clearLatchedCartesianTargets();
    left_follower_output_smd_.deactivate();
    right_follower_output_smd_.deactivate();
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (fault_latched_.load()) return;
    fault_latched_.store(true);
    fault_verdict_.store(verdict);
    latched_fault_reason_.store(verdict);
    fault_reason_ = reason;
    if (contexts.top_level.has_value()) {
        latched_fault_context_ = contextWithReason(*contexts.top_level, reason);
    } else {
        FaultContext fallback;
        fallback.verdict = verdict;
        fallback.domain = verdict == SafetyVerdict::EmergencyStop ? FaultDomain::Emergency : FaultDomain::SafetyPolicy;
        fallback.reason = reason;
        fallback.suppress_regular_servo = true;
        latched_fault_context_ = fallback;
    }
    left_latched_fault_context_ = contexts.left.has_value()
        ? std::optional<FaultContext>(contextWithReason(*contexts.left, reason))
        : std::nullopt;
    right_latched_fault_context_ = contexts.right.has_value()
        ? std::optional<FaultContext>(contextWithReason(*contexts.right, reason))
        : std::nullopt;
    left_fault_hold_q_deg_ = chooseSafeHoldTarget(
        ArmId::Left, left_state, left_prev_sent_q_deg_);
    right_fault_hold_q_deg_ = chooseSafeHoldTarget(
        ArmId::Right, right_state, right_prev_sent_q_deg_);
    setMotionState(verdict == SafetyVerdict::EmergencyStop
        ? ServerMotionState::EmergencyLatched
        : ServerMotionState::FaultLatched);
    std::cerr << "[WARN] fault latched: " << toString(verdict) << " - " << reason << "\n";
}

void DualArmServoLoop::setMotionState(ServerMotionState state) {
    motion_state_ = state;
}

ServoTarget DualArmServoLoop::currentFaultHoldTarget() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    ServoTarget target;
    target.left_q_target_deg = left_fault_hold_q_deg_;
    target.right_q_target_deg = right_fault_hold_q_deg_;
    return target;
}

JointArray DualArmServoLoop::chooseSafeHoldTarget(
    ArmId arm_id,
    const RobotState& state,
    const JointArray& previous_sent
) const {
    const BackendConfig& backend = arm_id == ArmId::Left
        ? config_.left_robot : config_.right_robot;
    if (isRbpodoControllerSimulationBackend(backend) &&
        controllerSimulationMotionGateOpen(config_) &&
        finiteJointArray(state.q_target_deg)) {
        // A physical control box held in pgmode simulation intentionally keeps
        // q_actual fixed. Fault/reset and freedrive resync must therefore hold
        // the simulated controller reference instead of snapping the internal
        // target back to the physical encoders.
        return state.q_target_deg;
    }
    if (isValidJointState(state)) {
        return state.q_actual_deg;
    }
    return previous_sent;
}

double DualArmServoLoop::computeFilterDtSec(uint64_t actual_period_ns, uint64_t nominal_period_ns) const {
    const double nominal_dt = nsToSec(nominal_period_ns);
    const double actual_dt = nsToSec(actual_period_ns);
    const double min_ratio = std::max(0.0, config_.servo.filter_dt_min_ratio);
    const double max_ratio = std::max(min_ratio, config_.servo.filter_dt_max_ratio);
    return std::clamp(actual_dt, nominal_dt * min_ratio, nominal_dt * max_ratio);
}

}  // namespace rb_servo
