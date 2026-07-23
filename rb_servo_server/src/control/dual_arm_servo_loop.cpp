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
#include <pinocchio/spatial/force.hpp>
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

math::Matrix3 surfaceFrameFromNormal(const math::Vector3& normal_input) {
    const math::Vector3 normal = normal_input.normalized();
    math::Vector3 tangent_x = math::Vector3::UnitX() - normal * normal.x();
    if (tangent_x.squaredNorm() < 1e-8) {
        tangent_x = math::Vector3::UnitY() - normal * normal.y();
    }
    tangent_x.normalize();
    const math::Vector3 tangent_y = normal.cross(tangent_x).normalized();
    math::Matrix3 frame;
    frame.col(0) = tangent_x;
    frame.col(1) = tangent_y;
    frame.col(2) = normal;
    return frame;
}

Wrench6D transformWrenchFrame(
    const pinocchio::SE3& t_target_source,
    const Wrench6D& source
) {
    pinocchio::Force source_force;
    source_force.linear() = math::Vector3(source.fx, source.fy, source.fz);
    source_force.angular() = math::Vector3(source.tx, source.ty, source.tz);
    const pinocchio::Force target_force = t_target_source.act(source_force);
    return {
        target_force.linear().x(), target_force.linear().y(), target_force.linear().z(),
        target_force.angular().x(), target_force.angular().y(), target_force.angular().z(),
    };
}

Vec6 transformTwistFrame(
    const pinocchio::SE3& t_target_source,
    const Vec6& source
) {
    pinocchio::Motion source_motion;
    source_motion.linear() = math::Vector3(source.x, source.y, source.z);
    source_motion.angular() = math::Vector3(source.rx, source.ry, source.rz);
    const pinocchio::Motion target_motion = t_target_source.act(source_motion);
    return {
        target_motion.linear().x(), target_motion.linear().y(), target_motion.linear().z(),
        target_motion.angular().x(), target_motion.angular().y(), target_motion.angular().z(),
    };
}

pinocchio::SE3 complianceFrameTcp(
    const ForceControlArmConfig& arm_config,
    const FtWrenchPipelineConfig& ft_config,
    const Pose6D& tcp_stand,
    const math::Matrix3& r_stand_surface
) {
    const pinocchio::SE3 t_tcp_sensor = math::se3FromPose(ft_config.t_tcp_sensor);
    if (arm_config.compliance_frame == "sensor_origin") {
        return t_tcp_sensor;
    }
    if (arm_config.compliance_frame == "tcp_origin") {
        return pinocchio::SE3(t_tcp_sensor.rotation(), math::Vector3::Zero());
    }
    const math::Matrix3 r_stand_tcp = math::rotationFromPose(tcp_stand);
    return pinocchio::SE3(
        r_stand_tcp.transpose() * r_stand_surface,
        math::Vector3::Zero()
    );
}

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

NormalForceControllerConfig makeNormalForceControllerConfig(
    const DualArmConfig& config,
    ArmId arm
) {
    const ForceControlArmConfig& arm_config =
        arm == ArmId::Left ? config.force_control.left : config.force_control.right;
    const NormalAdmittanceConfig& normal = config.force_control.normal_admittance;
    NormalForceControllerConfig out;
    out.enable = config.force_control.enable && arm_config.enable &&
        (config.force_control.operating_mode == "guarded_admittance" ||
         config.force_control.operating_mode == "cartesian_admittance");
    out.virtual_mass_kg = normal.virtual_mass_kg;
    out.damping_n_s_per_m = normal.damping_n_s_m;
    out.stiffness_n_per_m = normal.stiffness_n_m;
    out.force_deadband_n = arm_config.force_deadband_n;
    out.max_dt_sec = std::max(
        1.0 / static_cast<double>(std::max(1, config.servo.rate_hz)) * 4.0,
        0.02
    );
    out.max_unload_offset_m = normal.max_unload_offset_m;
    out.max_unload_velocity_m_s = normal.max_normal_velocity_m_s;
    out.max_unload_acceleration_m_s2 = normal.max_normal_acceleration_m_s2;
    out.max_unload_jerk_m_s3 = normal.max_normal_jerk_m_s3;
    out.max_unload_step_m = normal.max_normal_step_m;
    out.max_observed_energy_j = normal.max_energy_j;
    return out;
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
        a.af_damping_beta != b.af_damping_beta ||
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
    cfg.guard.af_damping_beta = rf.af_damping_beta;
    cfg.max_projection_error_m = rf.preview_max_projection_error_m;
    cfg.max_projection_error_rad = rf.preview_max_projection_error_rad;
    cfg.max_consecutive_projection_errors = rf.preview_max_consecutive_projection_errors;
    cfg.max_actual_lead_m = rf.preview_max_actual_lead_m;
    cfg.max_actual_lead_rad = rf.preview_max_actual_lead_rad;
    cfg.max_consecutive_actual_lead_errors = rf.preview_max_consecutive_actual_lead_errors;
    cfg.loading_projection_max_accel_m_s2 = rf.loading_projection_max_accel_m_s2;
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

void DualArmServoLoop::invalidatePostInitTare(ArmId arm_id, uint64_t command_seq) {
    ForceArmRuntime& runtime =
        arm_id == ArmId::Left ? left_force_runtime_ : right_force_runtime_;
    FtWrenchPipeline& pipeline =
        arm_id == ArmId::Left ? left_ft_pipeline_ : right_ft_pipeline_;
    NormalForceController& controller = arm_id == ArmId::Left
        ? left_normal_force_controller_ : right_normal_force_controller_;
    ForceController& cartesian_controller = arm_id == ArmId::Left
        ? left_cartesian_force_controller_ : right_cartesian_force_controller_;
    const FtWrenchPipelineConfig& config = arm_id == ArmId::Left
        ? config_.force_torque.left : config_.force_torque.right;

    if (!config.enable || !config.auto_tare_after_init_motion) return;
    if (command_seq != 0 && command_seq == runtime.last_init_tare_command_seq) return;
    if (command_seq == 0 && runtime.tare_waiting_for_init_completion) return;

    runtime.last_init_tare_command_seq = command_seq;
    runtime.tare_valid = false;
    runtime.tare_waiting_for_init_completion = true;
    runtime.tare_collecting = false;
    runtime.tare_not_before_ns = 0;
    runtime.ft.tare_state = "awaiting_init_completion";
    runtime.ft.tare_sample_count = 0;
    runtime.ft.tare_reason = "init motion requested; previous software zero invalidated";
    pipeline.cancelResidualTare();

    runtime.pending_proposal.reset();
    runtime.pending_proposal_applied = false;
    runtime.pending_cartesian_proposal.reset();
    runtime.pending_cartesian_proposal_applied = false;
    runtime.rolling_compliance_target_valid = false;
    runtime.rolling_compliance_target_source = "unavailable";
    runtime.pending_rolling_compliance_target_valid = false;
    runtime.pending_rolling_compliance_target_source = "unavailable";
    runtime.compliance_hold_target_this_tick = false;
    runtime.control.contact_active = false;
    runtime.control.normal_contact_active = false;
    runtime.control.transverse_contact_active = false;
    runtime.control.rotational_contact_active = false;
    runtime.normal_contact_active = false;
    runtime.contact_force_normal_estimator.reset();
    runtime.contact_cartesian_normal_offset_m = 0.0;
    runtime.follower_contact_normal_owned = false;
    runtime.transverse_contact_active = false;
    runtime.rotational_contact_active = false;
    runtime.contact_anchor_valid = false;
    runtime.enter_count = 0;
    runtime.transverse_enter_count = 0;
    runtime.rotational_enter_count = 0;
    runtime.hard_limit_count = 0;
    runtime.retreat_active = false;
    runtime.retreat_braking = false;
    runtime.retreat_virtual_current_n = 0.0;
    runtime.retreat_started_ns = 0;
    runtime.retreat_attempt_count = 0;
    runtime.retreat_window_start_ns = 0;
    runtime.release_start_ns = 0;
    runtime.release_hold_pending = false;
    runtime.release_hold_applied = false;
    runtime.release_hold_clear_requested = false;
    runtime.control.correction_m = 0.0;
    runtime.control.velocity_m_s = 0.0;
    runtime.control.acceleration_m_s2 = 0.0;
    runtime.control.energy_j = 0.0;
    controller.release();
    cartesian_controller.release();
}

bool DualArmServoLoop::latchPayloadIdentificationInhibit(ArmId arm_id) {
    ForceArmRuntime& runtime =
        arm_id == ArmId::Left ? left_force_runtime_ : right_force_runtime_;
    FtWrenchPipeline& pipeline =
        arm_id == ArmId::Left ? left_ft_pipeline_ : right_ft_pipeline_;
    NormalForceController& controller = arm_id == ArmId::Left
        ? left_normal_force_controller_ : right_normal_force_controller_;
    ForceController& cartesian_controller = arm_id == ArmId::Left
        ? left_cartesian_force_controller_ : right_cartesian_force_controller_;
    const FtWrenchPipelineConfig& ft_config = arm_id == ArmId::Left
        ? config_.force_torque.left : config_.force_torque.right;

    // The profile is unusable unless the server owns an explicit profile and
    // this arm has the post-InitMotion tare path that is the only release event.
    // Holding here is safer than allowing a profile that can never establish or
    // later restore a valid software zero.
    if (!config_.force_torque.payload_identification.enable ||
        !ft_config.enable || !ft_config.auto_tare_after_init_motion ||
        !runtime.ft.healthy || runtime.ft.stale) {
        return false;
    }
    if (runtime.payload_identification_inhibit) {
        return true;
    }

    runtime.payload_identification_inhibit = true;
    runtime.tare_valid = false;
    runtime.tare_waiting_for_init_completion = false;
    runtime.tare_collecting = false;
    runtime.tare_not_before_ns = 0;
    runtime.ft.payload_identification_inhibit = true;
    runtime.ft.tare_valid = false;
    runtime.ft.tare_state = "awaiting_init_motion";
    runtime.ft.tare_sample_count = 0;
    runtime.ft.tare_reason =
        "payload identification invalidated software zero; run Init Motion to re-tare";
    pipeline.cancelResidualTare();

    runtime.pending_proposal.reset();
    runtime.pending_proposal_applied = false;
    runtime.pending_cartesian_proposal.reset();
    runtime.pending_cartesian_proposal_applied = false;
    runtime.rolling_compliance_target_valid = false;
    runtime.rolling_compliance_target_source = "unavailable";
    runtime.pending_rolling_compliance_target_valid = false;
    runtime.pending_rolling_compliance_target_source = "unavailable";
    runtime.compliance_hold_target_this_tick = false;
    runtime.control.contact_active = false;
    runtime.control.normal_contact_active = false;
    runtime.control.transverse_contact_active = false;
    runtime.control.rotational_contact_active = false;
    runtime.control.compliance_active = false;
    runtime.control.normal_regulating = false;
    runtime.control.transverse_regulating = false;
    runtime.control.rotational_regulating = false;
    runtime.control.proposal_valid = false;
    runtime.control.proposal_committed = false;
    runtime.control.state = "payload_identification_inhibited";
    runtime.normal_contact_active = false;
    runtime.contact_force_normal_estimator.reset();
    runtime.contact_cartesian_normal_offset_m = 0.0;
    runtime.follower_contact_normal_owned = false;
    runtime.transverse_contact_active = false;
    runtime.rotational_contact_active = false;
    runtime.contact_anchor_valid = false;
    runtime.enter_count = 0;
    runtime.transverse_enter_count = 0;
    runtime.rotational_enter_count = 0;
    runtime.hard_limit_count = 0;
    runtime.retreat_active = false;
    runtime.retreat_braking = false;
    runtime.retreat_virtual_current_n = 0.0;
    runtime.retreat_started_ns = 0;
    runtime.retreat_attempt_count = 0;
    runtime.retreat_window_start_ns = 0;
    runtime.release_start_ns = 0;
    runtime.release_hold_pending = false;
    runtime.release_hold_applied = false;
    runtime.release_hold_clear_requested = false;
    runtime.control.correction_m = 0.0;
    runtime.control.velocity_m_s = 0.0;
    runtime.control.acceleration_m_s2 = 0.0;
    runtime.control.energy_j = 0.0;
    controller.release();
    cartesian_controller.release();

    std::cerr << "[INFO] FT " << toString(arm_id)
              << " payload-identification force-motion inhibit latched; "
                 "Init Motion re-tare required\n";
    return true;
}

void DualArmServoLoop::beginPostInitTare(ArmId arm_id, uint64_t now_ns) {
    ForceArmRuntime& runtime =
        arm_id == ArmId::Left ? left_force_runtime_ : right_force_runtime_;
    const FtWrenchPipelineConfig& config = arm_id == ArmId::Left
        ? config_.force_torque.left : config_.force_torque.right;
    if (!config.enable || !config.auto_tare_after_init_motion ||
        !runtime.tare_waiting_for_init_completion) {
        return;
    }

    runtime.tare_waiting_for_init_completion = false;
    runtime.tare_collecting = false;
    runtime.tare_not_before_ns = now_ns + static_cast<uint64_t>(
        config.auto_tare_settle_sec * 1'000'000'000.0);
    ++runtime.tare_generation;
    runtime.ft.tare_generation = runtime.tare_generation;
    runtime.ft.tare_state = "settling";
    runtime.ft.tare_sample_count = 0;
    runtime.ft.tare_reason = "init motion complete; waiting for mechanical settling";
    std::cerr << "[INFO] FT " << toString(arm_id)
              << " post-init software zero: settling for "
              << config.auto_tare_settle_sec << " s\n";
}

bool DualArmServoLoop::updateForceRuntime(
    ArmId arm_id,
    const RobotState& state,
    double dt_sec,
    uint64_t now_ns,
    std::string* fault_reason
) {
    ForceArmRuntime& runtime =
        arm_id == ArmId::Left ? left_force_runtime_ : right_force_runtime_;
    FtWrenchPipeline& pipeline =
        arm_id == ArmId::Left ? left_ft_pipeline_ : right_ft_pipeline_;
    NormalForceController& controller = arm_id == ArmId::Left
        ? left_normal_force_controller_ : right_normal_force_controller_;
    ForceController& cartesian_controller = arm_id == ArmId::Left
        ? left_cartesian_force_controller_ : right_cartesian_force_controller_;
    const FtWrenchPipelineConfig& ft_config = arm_id == ArmId::Left
        ? config_.force_torque.left : config_.force_torque.right;
    const ForceControlArmConfig& arm_config = arm_id == ArmId::Left
        ? config_.force_control.left : config_.force_control.right;

    runtime.pending_proposal.reset();
    runtime.pending_proposal_applied = false;
    runtime.pending_cartesian_proposal.reset();
    runtime.pending_cartesian_proposal_applied = false;
    runtime.pending_rolling_compliance_target_valid = false;
    runtime.pending_rolling_compliance_target_source = "unavailable";
    runtime.compliance_hold_target_this_tick = false;
    runtime.release_hold_applied = false;
    runtime.release_hold_clear_requested = false;
    runtime.control.proposal_valid = false;
    runtime.control.proposal_committed = false;
    runtime.control.compliance_active = false;
    runtime.t_tcp_compliance_valid = false;
    runtime.follower_loading_reaction_valid = false;
    runtime.follower_contact_normal_owned = false;
    runtime.control.normal_regulating = false;
    runtime.control.transverse_regulating = false;
    runtime.control.rotational_regulating = false;
    runtime.control.loading_projection_active = false;
    runtime.control.compliance_recenter_active = false;
    runtime.control.compliance_translation_recenter_coupled = false;
    runtime.control.compliance_rotation_recenter_coupled = false;
    runtime.control.compliance_translation_recenter_deferred = false;
    runtime.control.compliance_rotation_recenter_deferred = false;
    runtime.control.compliance_limit_axes = {};
    runtime.control.compliance_limit_reason.clear();
    runtime.control.compliance_equilibrium_stand = runtime.rolling_compliance_target;
    runtime.control.compliance_equilibrium_source =
        runtime.rolling_compliance_target_valid
            ? runtime.rolling_compliance_target_source : "unavailable";
    runtime.control.raw_policy_delta_surface = {};
    runtime.control.accepted_policy_delta_surface = {};
    runtime.control.fault_reason.clear();
    runtime.ft.enabled = ft_config.enable;
    runtime.ft.source = config_.force_torque.source;
    runtime.ft.source_assurance = ft_config.enable
        ? "controller_frame_only" : "unavailable";
    runtime.ft.sensor_health_verified = false;
    runtime.ft.safety_rated = false;
    runtime.ft.raw_sensor_wrench = state.eft_wrench;
    runtime.ft.t_tcp_sensor = ft_config.t_tcp_sensor;
    runtime.ft.gravity_compensation_model = ft_config.gravity_compensation_model;
    runtime.ft.gravity_compensation_calibration_id =
        ft_config.gravity_compensation_calibration_id;
    runtime.ft.payload_identification_inhibit =
        runtime.payload_identification_inhibit;
    runtime.ft.auto_tare_enabled = ft_config.auto_tare_after_init_motion;
    runtime.ft.tare_valid = runtime.tare_valid;
    runtime.ft.tare_generation = runtime.tare_generation;
    runtime.ft.residual_tare_tcp = pipeline.residualTareTcp();
    if (!ft_config.auto_tare_after_init_motion) {
        runtime.ft.tare_state = "disabled";
        runtime.ft.tare_reason.clear();
    } else if (runtime.ft.tare_state == "disabled") {
        runtime.ft.tare_state = "awaiting_init_motion";
        runtime.ft.tare_reason = "run Init Motion to establish the software zero";
    }
    runtime.control.enabled = config_.force_control.enable && arm_config.enable;
    runtime.control.operating_mode = config_.force_control.operating_mode;
    runtime.control.surface_source = arm_config.surface_source;
    runtime.control.compliance_frame = arm_config.compliance_frame;
    runtime.control.compliance_frame_pose_valid = false;
    runtime.control.target_force_n = arm_config.target_force_n;
    runtime.control.motion_epoch = motion_epoch_;

    const bool contact_force_surface = arm_config.surface_source == "contact_force";
    // user_floor_plane tracks the runtime tilted plane; floor_constraint and the
    // floorless "none" posture use nominal stand +Z. contact_force has no frame
    // outside a debounced episode; +Z below is only an internal placeholder
    // until the current F/T sample can enter/capture, never an active Branch-A
    // normal and never published as one.
    math::Vector3 normal = arm_config.surface_source == "user_floor_plane"
        ? effectiveUserFloorNormal()
        : contact_force_surface && runtime.contact_force_normal_estimator.valid()
            ? runtime.contact_force_normal_estimator.normalStand()
            : math::Vector3(0.0, 0.0, 1.0);
    const bool initial_normal_valid =
        !contact_force_surface || runtime.contact_force_normal_estimator.valid();
    runtime.control.normal_stand = initial_normal_valid
        ? std::array<double, 3>{normal.x(), normal.y(), normal.z()}
        : std::array<double, 3>{0.0, 0.0, 0.0};

    if (!ft_config.enable) {
        runtime.ft.healthy = false;
        runtime.ft.reason = "disabled";
        runtime.control.state = runtime.control.enabled ? "unavailable" : "inactive";
        return false;
    }
    if (!state.tcp_actual_valid || !state.tcp_actual_stand.has_value()) {
        runtime.ft.healthy = false;
        runtime.ft.reason = "actual TCP pose unavailable";
        runtime.control.state = "unavailable";
        if (runtime.control.enabled && config_.force_control.operating_mode != "monitor") {
            if (fault_reason) *fault_reason = "force control actual TCP pose unavailable";
            return true;
        }
        return false;
    }

    // Resolve and publish the exact controller frame as soon as the actual TCP
    // is available. This pose is configuration/geometry state, not a claim
    // that the wrench pipeline is healthy or that force motion is active.
    const Pose6D& tcp = *state.tcp_actual_stand;
    math::Matrix3 r_stand_surface = surfaceFrameFromNormal(normal);
    pinocchio::SE3 t_tcp_compliance = complianceFrameTcp(
        arm_config, ft_config, tcp, r_stand_surface
    );
    pinocchio::SE3 t_stand_compliance =
        math::se3FromPose(tcp) * t_tcp_compliance;
    runtime.compliance_frame_actual_stand = math::poseFromSe3(t_stand_compliance);
    runtime.control.compliance_frame_actual_stand =
        runtime.compliance_frame_actual_stand;
    runtime.control.compliance_frame_pose_valid = true;
    runtime.t_tcp_compliance_pose = math::poseFromSe3(t_tcp_compliance);
    runtime.t_tcp_compliance_valid = true;

    FtRawSample raw;
    raw.wrench_sensor = state.eft_wrench;
    raw.host_time_ns = state.host_time_ns;
    raw.source_sequence = state.acquisition_sequence;
    raw.source_time_ns = state.robot_time_ns;
    raw.source_sequence_valid = state.acquisition_sequence > 0;
    raw.source_time_valid = state.robot_time_ns > 0;
    raw.fields_present = state.eft_valid;
    // rbpodo has no independent presence bit. This is deliberately weaker
    // than verified sensor health and is exposed as controller_frame_only.
    raw.sensor_present = state.eft_valid && ft_config.frame_configured;

    math::Vector3 commanded_acceleration_tcp = math::Vector3::Zero();
    try {
        if (kinematics_) {
            const JointArray& sent_q = arm_id == ArmId::Left
                ? left_prev_sent_q_deg_ : right_prev_sent_q_deg_;
            const ArmMountConfig& mount = arm_id == ArmId::Left
                ? config_.left_mount : config_.right_mount;
            const Pose6D sent_tcp =
                kinematics_->computeTcpStand(arm_id, sent_q, mount);
            if (runtime.sent_tcp_sample_count >= 2 &&
                now_ns > runtime.sent_tcp_sample_newer_ns &&
                runtime.sent_tcp_sample_newer_ns > runtime.sent_tcp_sample_older_ns) {
                const double newer_dt_sec = static_cast<double>(
                    now_ns - runtime.sent_tcp_sample_newer_ns) * 1e-9;
                const double older_dt_sec = static_cast<double>(
                    runtime.sent_tcp_sample_newer_ns -
                    runtime.sent_tcp_sample_older_ns) * 1e-9;
                if (newer_dt_sec <= config_.force_control.max_dt_sec &&
                    older_dt_sec <= config_.force_control.max_dt_sec) {
                    const math::Vector3 current_position(
                        sent_tcp.x, sent_tcp.y, sent_tcp.z);
                    const math::Vector3 newer_position(
                        runtime.sent_tcp_sample_newer.x,
                        runtime.sent_tcp_sample_newer.y,
                        runtime.sent_tcp_sample_newer.z);
                    const math::Vector3 older_position(
                        runtime.sent_tcp_sample_older.x,
                        runtime.sent_tcp_sample_older.y,
                        runtime.sent_tcp_sample_older.z);
                    const math::Vector3 newer_velocity =
                        (current_position - newer_position) / newer_dt_sec;
                    const math::Vector3 older_velocity =
                        (newer_position - older_position) / older_dt_sec;
                    const math::Vector3 acceleration_stand =
                        2.0 * (newer_velocity - older_velocity) /
                        (newer_dt_sec + older_dt_sec);
                    commanded_acceleration_tcp =
                        math::rotationFromPose(sent_tcp).transpose() *
                        acceleration_stand;
                } else {
                    runtime.sent_tcp_sample_count = 0;
                }
            }
            runtime.sent_tcp_sample_older = runtime.sent_tcp_sample_newer;
            runtime.sent_tcp_sample_older_ns = runtime.sent_tcp_sample_newer_ns;
            runtime.sent_tcp_sample_newer = sent_tcp;
            runtime.sent_tcp_sample_newer_ns = now_ns;
            runtime.sent_tcp_sample_count =
                std::min(runtime.sent_tcp_sample_count + 1, 2);
        } else {
            runtime.sent_tcp_sample_count = 0;
        }
    } catch (const std::exception&) {
        runtime.sent_tcp_sample_count = 0;
        commanded_acceleration_tcp.setZero();
    }
    // Three valid sent-command TCP samples are required before the double
    // difference is meaningful. Startup/gap ticks explicitly supply zero;
    // the pipeline then phase-matches its own acceleration LPF to fresh EFT
    // acquisitions before subtracting m*a.
    pipeline.setTcpLinearAcceleration(commanded_acceleration_tcp);
    const FtWrenchPipelineOutput output =
        pipeline.process(raw, *state.tcp_actual_stand, now_ns);
    runtime.ft.wrench_tcp = output.wrench_tcp;
    runtime.ft.gravity_tcp = output.gravity_tcp;
    runtime.ft.modeled_gravity_wrench = output.modeled_gravity_wrench_tcp;
    runtime.ft.fast_external_wrench = output.fast_external_wrench_tcp;
    runtime.ft.control_external_wrench = output.control_external_wrench_tcp;
    runtime.ft.healthy = output.healthy;
    runtime.ft.stale = output.stale;
    runtime.ft.freshness_value = output.freshness_value;
    runtime.ft.freshness_advanced = output.freshness_advanced;
    runtime.ft.reason = output.reason;

    // Payload identification deliberately invalidates the software tare so the
    // GUI can collect the pre-payload/pre-tare TCP wrench.  The normal force
    // path returns early while that tare is invalid, so retain a separate raw
    // wrench hard guard here.  Otherwise calibration motion could continue with
    // stale/unhealthy F/T data or never publish/latch the configured hard limit.
    if (runtime.payload_identification_inhibit && !output.healthy) {
        runtime.control.state = "unavailable";
        runtime.control.fault_reason =
            "payload identification F/T pipeline unhealthy: " + output.reason;
        if (runtime.control.enabled &&
            config_.force_control.operating_mode != "monitor") {
            if (fault_reason) *fault_reason = runtime.control.fault_reason;
            return true;
        }
    }
    if (runtime.payload_identification_inhibit && output.healthy &&
        runtime.control.enabled &&
        config_.force_control.operating_mode != "monitor") {
        const math::Matrix3 r_stand_tcp = math::rotationFromPose(tcp);
        const math::Vector3 raw_force_stand = r_stand_tcp * math::Vector3(
            output.wrench_tcp.fx, output.wrench_tcp.fy, output.wrench_tcp.fz
        );
        const math::Vector3 raw_torque_stand = r_stand_tcp * math::Vector3(
            output.wrench_tcp.tx, output.wrench_tcp.ty, output.wrench_tcp.tz
        );
        runtime.control.fast_normal_force_n = -normal.dot(raw_force_stand);
        runtime.control.fast_force_norm_n = raw_force_stand.norm();
        runtime.control.fast_torque_norm_nm = raw_torque_stand.norm();
        const bool hard_limit_threshold_exceeded =
            runtime.control.fast_normal_force_n >= arm_config.hard_normal_force_n ||
            runtime.control.fast_force_norm_n >= arm_config.hard_force_norm_n ||
            runtime.control.fast_torque_norm_nm >= arm_config.hard_torque_norm_nm;
        runtime.control.hard_limit_threshold_exceeded = hard_limit_threshold_exceeded;
        if (output.freshness_advanced) {
            if (!hard_limit_threshold_exceeded) {
                runtime.hard_limit_count = 0;
            } else if (runtime.hard_limit_count < arm_config.hard_limit_debounce_samples) {
                ++runtime.hard_limit_count;
            }
        }
        runtime.control.hard_limit_sample_count = runtime.hard_limit_count;
        runtime.control.hard_limit_exceeded =
            runtime.hard_limit_count >= arm_config.hard_limit_debounce_samples;
        if (runtime.control.hard_limit_exceeded) {
            runtime.control.state = "fault";
            runtime.control.fault_reason =
                "payload identification raw force/torque hard limit exceeded";
            if (fault_reason) *fault_reason = runtime.control.fault_reason;
            return true;
        }
    }

    const auto update_tare = [&]() {
        // Init Motion completion plus the configured settle time is the stationarity
        // gate. The explicit operator action also asserts that the tool is contact-free;
        // the variance limit rejects motion/transients but cannot detect a steady contact.
        const FtTareUpdate update = pipeline.updateResidualTare(output, true, true);
        runtime.ft.tare_sample_count = update.sample_count;
        runtime.ft.tare_reason = update.reason;
        if (update.state == FtTareState::Collecting) {
            runtime.ft.tare_state = "collecting";
        } else if (update.state == FtTareState::Accepted) {
            runtime.tare_collecting = false;
            runtime.tare_valid = true;
            const bool payload_identification_inhibit_was_latched =
                runtime.payload_identification_inhibit;
            runtime.payload_identification_inhibit = false;
            runtime.ft.tare_valid = true;
            runtime.ft.payload_identification_inhibit = false;
            runtime.ft.tare_state = "accepted";
            runtime.ft.residual_tare_tcp = pipeline.residualTareTcp();
            runtime.previous_actual_pose_ns = 0;
            if (config_.force_control.operating_mode == "cartesian_admittance") {
                cartesian_controller.engage();
                runtime.rolling_compliance_target = *state.tcp_actual_stand;
                runtime.previous_raw_compliance_target = *state.tcp_actual_stand;
                runtime.rolling_compliance_target_valid = true;
                runtime.rolling_compliance_target_source = "hold_anchor";
                runtime.control.compliance_equilibrium_stand =
                    runtime.rolling_compliance_target;
                runtime.control.compliance_equilibrium_source = "hold_anchor";
            }
            ++motion_epoch_;
            runtime.control.motion_epoch = motion_epoch_;
            std::cerr << "[INFO] FT " << toString(arm_id)
                      << " post-init software zero accepted (samples="
                      << update.sample_count << ")\n";
            if (payload_identification_inhibit_was_latched) {
                std::cerr << "[INFO] FT " << toString(arm_id)
                          << " payload-identification force-motion inhibit cleared "
                             "after accepted Init Motion tare\n";
            }
        } else if (update.state == FtTareState::Rejected) {
            runtime.tare_collecting = false;
            runtime.tare_valid = false;
            runtime.ft.tare_valid = false;
            runtime.ft.tare_state = "rejected";
            std::cerr << "[WARN] FT " << toString(arm_id)
                      << " post-init software zero rejected: " << update.reason << "\n";
        }
    };

    if (ft_config.auto_tare_after_init_motion) {
        if (runtime.tare_waiting_for_init_completion) {
            runtime.ft.tare_state = "awaiting_init_completion";
            runtime.control.state = "awaiting_init_tare";
            return false;
        }
        if (runtime.tare_not_before_ns != 0) {
            if (now_ns < runtime.tare_not_before_ns) {
                runtime.ft.tare_state = "settling";
                runtime.control.state = "zeroing";
                return false;
            }
            runtime.tare_not_before_ns = 0;
            pipeline.beginResidualTare();
            runtime.tare_collecting = true;
            runtime.ft.tare_state = "collecting";
            runtime.ft.tare_reason = "collecting fresh post-init samples";
        }
        if (runtime.tare_collecting) {
            update_tare();
            runtime.control.state = runtime.tare_valid ? "zeroed" :
                (runtime.ft.tare_state == "rejected" ? "tare_rejected" : "zeroing");
            return false;
        }
        if (!runtime.tare_valid) {
            runtime.control.state = runtime.payload_identification_inhibit
                ? "payload_identification_inhibited"
                : (runtime.ft.tare_state == "rejected"
                    ? "tare_rejected" : "awaiting_init_tare");
            return false;
        }
    }

    // Defensive backstop for configurations without automatic tare. Such a
    // configuration cannot normally admit the profile, but an already-latched
    // runtime must still never re-enter a force-motion path.
    if (runtime.payload_identification_inhibit) {
        runtime.control.state = "payload_identification_inhibited";
        return false;
    }

    if (!output.healthy) {
        runtime.control.state = "unavailable";
        if (runtime.control.enabled && config_.force_control.operating_mode != "monitor") {
            runtime.control.fault_reason = "FT pipeline unhealthy: " + output.reason;
            if (fault_reason) *fault_reason = runtime.control.fault_reason;
            return true;
        }
        return false;
    }

    const math::Matrix3 r_stand_tcp = math::rotationFromPose(tcp);
    const auto forceStand = [&r_stand_tcp](const Wrench6D& wrench) -> math::Vector3 {
        return r_stand_tcp * math::Vector3(wrench.fx, wrench.fy, wrench.fz);
    };
    const auto torqueStand = [&r_stand_tcp](const Wrench6D& wrench) -> math::Vector3 {
        return r_stand_tcp * math::Vector3(wrench.tx, wrench.ty, wrench.tz);
    };
    const math::Vector3 force_control_stand = forceStand(output.control_external_wrench_tcp);
    const math::Vector3 torque_control_stand = torqueStand(output.control_external_wrench_tcp);
    const math::Vector3 force_fast_stand = forceStand(output.fast_external_wrench_tcp);
    const math::Vector3 torque_fast_stand = torqueStand(output.fast_external_wrench_tcp);

    // contact_force enters on resultant control force because no normal exists
    // yet. Capture n=-normalize(F) exactly once at the debounced rising edge,
    // then freeze it until the existing release-brake/dwell state completes.
    bool contact_force_entered = false;
    if (contact_force_surface && runtime.control.enabled &&
        config_.force_control.operating_mode == "cartesian_admittance" &&
        output.freshness_advanced) {
        if (!runtime.normal_contact_active) {
            // Episode ENTRY gates (2026-07-18 15:23 runs): transit grip/
            // inertial residuals reached 3.5-9.5 N with a ROTATING direction
            // and froze bogus lateral normals for seconds, warping the
            // post-pick motion. A real surface contact is (a) approached at
            // low commanded speed and (b) pushes in a stable direction across
            // the debounce window. Either gate failing resets the debounce.
            double commanded_speed_m_s =
                std::numeric_limits<double>::infinity();
            if (runtime.sent_tcp_sample_count >= 2 &&
                runtime.sent_tcp_sample_newer_ns >
                    runtime.sent_tcp_sample_older_ns) {
                const double dt_s = static_cast<double>(
                    runtime.sent_tcp_sample_newer_ns -
                    runtime.sent_tcp_sample_older_ns) * 1e-9;
                const math::Vector3 dp(
                    runtime.sent_tcp_sample_newer.x - runtime.sent_tcp_sample_older.x,
                    runtime.sent_tcp_sample_newer.y - runtime.sent_tcp_sample_older.y,
                    runtime.sent_tcp_sample_newer.z - runtime.sent_tcp_sample_older.z);
                if (dt_s > 0.0) commanded_speed_m_s = dp.norm() / dt_s;
            }
            const bool entry_speed_ok =
                commanded_speed_m_s <= arm_config.contact_entry_max_speed_m_s;
            bool entry_direction_ok = false;
            const double f_norm = force_control_stand.norm();
            if (f_norm > 1e-9) {
                const math::Vector3 dir = force_control_stand / f_norm;
                const math::Vector3 first(
                    runtime.contact_entry_first_dir[0],
                    runtime.contact_entry_first_dir[1],
                    runtime.contact_entry_first_dir[2]);
                constexpr double kEntryDirCosMin = 0.866;  // 30 deg cone
                if (!runtime.contact_entry_first_dir_valid ||
                    dir.dot(first) >= kEntryDirCosMin) {
                    entry_direction_ok = true;
                    if (!runtime.contact_entry_first_dir_valid) {
                        runtime.contact_entry_first_dir = {
                            dir.x(), dir.y(), dir.z()};
                        runtime.contact_entry_first_dir_valid = true;
                    }
                } else {
                    // Rotated out of the cone: restart the window on the new
                    // direction so a genuine contact after a transient still
                    // enters promptly.
                    runtime.contact_entry_first_dir = {dir.x(), dir.y(), dir.z()};
                    runtime.contact_entry_first_dir_valid = true;
                    runtime.enter_count = 0;
                }
            }
            if (force_control_stand.norm() >= arm_config.contact_enter_force_n &&
                entry_speed_ok && entry_direction_ok) {
                if (++runtime.enter_count >= arm_config.debounce_samples) {
                    runtime.normal_contact_active = true;
                    runtime.enter_count = 0;
                    runtime.contact_force_normal_estimator.update(
                        true, force_control_stand);
                    if (!runtime.contact_force_normal_estimator.valid()) {
                        runtime.normal_contact_active = false;
                        runtime.control.state = "fault";
                        runtime.control.fault_reason =
                            "contact-force normal capture failed";
                        if (fault_reason) *fault_reason = runtime.control.fault_reason;
                        return true;
                    }
                    runtime.contact_anchor = tcp;
                    runtime.contact_anchor_valid = true;
                    runtime.release_start_ns = 0;
                    runtime.contact_entry_first_dir_valid = false;
                    controller.engage();
                    contact_force_entered = true;
                }
            } else {
                runtime.enter_count = 0;
                runtime.contact_entry_first_dir_valid = false;
            }
        }
    }
    if (contact_force_surface && runtime.normal_contact_active) {
        runtime.contact_force_normal_estimator.update(true, force_control_stand);
        if (!runtime.contact_force_normal_estimator.valid()) {
            runtime.control.state = "fault";
            runtime.control.fault_reason = "contact-force episode lost its normal";
            if (fault_reason) *fault_reason = runtime.control.fault_reason;
            return true;
        }
        normal = runtime.contact_force_normal_estimator.normalStand();
        runtime.control.normal_stand = {normal.x(), normal.y(), normal.z()};
        r_stand_surface = surfaceFrameFromNormal(normal);
        t_tcp_compliance = complianceFrameTcp(
            arm_config, ft_config, tcp, r_stand_surface);
        t_stand_compliance = math::se3FromPose(tcp) * t_tcp_compliance;
        runtime.compliance_frame_actual_stand = math::poseFromSe3(t_stand_compliance);
        runtime.control.compliance_frame_actual_stand =
            runtime.compliance_frame_actual_stand;
        runtime.control.compliance_frame_pose_valid = true;
        runtime.t_tcp_compliance_pose = math::poseFromSe3(t_tcp_compliance);
        runtime.t_tcp_compliance_valid = true;
        if (contact_force_entered) {
            const Pose6D& committed_offset = cartesian_controller.state().offset_tcp;
            const math::Vector3 committed_translation_compliance(
                committed_offset.x, committed_offset.y, committed_offset.z);
            runtime.contact_cartesian_normal_offset_m = normal.dot(
                t_stand_compliance.rotation() * committed_translation_compliance);
        }
    } else if (contact_force_surface) {
        runtime.control.normal_stand = {0.0, 0.0, 0.0};
        if (arm_config.compliance_frame == "surface") {
            runtime.control.compliance_frame_pose_valid = false;
            runtime.t_tcp_compliance_valid = false;
        }
    }
    const math::Vector3 force_control_surface =
        r_stand_surface.transpose() * force_control_stand;
    const math::Vector3 torque_control_surface =
        r_stand_surface.transpose() * torque_control_stand;
    runtime.control_wrench_surface = {
        force_control_surface.x(), force_control_surface.y(), force_control_surface.z(),
        torque_control_surface.x(), torque_control_surface.y(), torque_control_surface.z(),
    };
    runtime.control.control_wrench_surface = runtime.control_wrench_surface;
    runtime.control_wrench_compliance = transformWrenchFrame(
        t_tcp_compliance.inverse(), output.control_external_wrench_tcp
    );
    runtime.control.control_wrench_compliance = runtime.control_wrench_compliance;
    // The geometric surface normal points outward from the constrained surface
    // (+Z for the stand floor).  The installed F/T sensor reports the reaction
    // wrench on the TCP, so floor compression projects opposite that outward
    // normal.  Keep the geometric normal unchanged for unloading motion and
    // negate only the wrench projection to expose positive compressive force.
    const bool normal_frame_valid =
        !contact_force_surface || runtime.contact_force_normal_estimator.valid();
    const double measured_normal = normal_frame_valid
        ? -normal.dot(force_control_stand) : 0.0;
    const double fast_normal = normal_frame_valid
        ? -normal.dot(force_fast_stand)
        : force_fast_stand.norm();
    runtime.control.measured_force_n = measured_normal;
    runtime.control.fast_normal_force_n = fast_normal;
    runtime.control.fast_force_norm_n = force_fast_stand.norm();
    runtime.control.fast_torque_norm_nm = torque_fast_stand.norm();
    const double transverse_force_n = std::hypot(
        runtime.control_wrench_compliance.fx,
        runtime.control_wrench_compliance.fy
    );
    const double control_torque_norm_nm = std::sqrt(
        runtime.control_wrench_compliance.tx * runtime.control_wrench_compliance.tx +
        runtime.control_wrench_compliance.ty * runtime.control_wrench_compliance.ty +
        runtime.control_wrench_compliance.tz * runtime.control_wrench_compliance.tz
    );
    const bool cartesian_soft_contact =
        transverse_force_n >= arm_config.transverse_contact_enter_force_n ||
        control_torque_norm_nm >= arm_config.torque_contact_enter_nm;
    runtime.control.contact_threshold_exceeded =
        (contact_force_surface
            ? force_control_stand.norm() >= arm_config.contact_enter_force_n
            : measured_normal >= arm_config.contact_enter_force_n) ||
        (config_.force_control.operating_mode == "cartesian_admittance" &&
         cartesian_soft_contact);
    const bool hard_limit_threshold_exceeded =
        fast_normal >= arm_config.hard_normal_force_n ||
        runtime.control.fast_force_norm_n >= arm_config.hard_force_norm_n ||
        runtime.control.fast_torque_norm_nm >= arm_config.hard_torque_norm_nm;
    runtime.control.hard_limit_threshold_exceeded = hard_limit_threshold_exceeded;
    if (output.freshness_advanced) {
        if (!hard_limit_threshold_exceeded) {
            runtime.hard_limit_count = 0;
        } else if (runtime.hard_limit_count < arm_config.hard_limit_debounce_samples) {
            ++runtime.hard_limit_count;
        }
    }
    runtime.control.hard_limit_sample_count = runtime.hard_limit_count;
    runtime.control.hard_limit_exceeded =
        runtime.hard_limit_count >= arm_config.hard_limit_debounce_samples;

    if (config_.force_control.operating_mode == "cartesian_admittance" &&
        output.freshness_advanced) {
        if (!contact_force_surface) {
            if (runtime.normal_contact_active) {
                if (measured_normal <= arm_config.contact_release_force_n) {
                    runtime.normal_contact_active = false;
                }
            } else if (measured_normal >= arm_config.contact_enter_force_n) {
                if (++runtime.enter_count >= arm_config.debounce_samples) {
                    runtime.normal_contact_active = true;
                    runtime.enter_count = 0;
                }
            } else {
                runtime.enter_count = 0;
            }
        }
        if (runtime.transverse_contact_active) {
            if (transverse_force_n <= arm_config.transverse_contact_release_force_n) {
                runtime.transverse_contact_active = false;
            }
        } else if (transverse_force_n >= arm_config.transverse_contact_enter_force_n) {
            if (++runtime.transverse_enter_count >= arm_config.debounce_samples) {
                runtime.transverse_contact_active = true;
                runtime.transverse_enter_count = 0;
            }
        } else {
            runtime.transverse_enter_count = 0;
        }
        if (runtime.rotational_contact_active) {
            if (control_torque_norm_nm <= arm_config.torque_contact_release_nm) {
                runtime.rotational_contact_active = false;
            }
        } else if (control_torque_norm_nm >= arm_config.torque_contact_enter_nm) {
            if (++runtime.rotational_enter_count >= arm_config.debounce_samples) {
                runtime.rotational_contact_active = true;
                runtime.rotational_enter_count = 0;
            }
        } else {
            runtime.rotational_enter_count = 0;
        }
    }
    const auto sync_contact_telemetry = [&] {
        runtime.control.normal_contact_active = runtime.normal_contact_active;
        runtime.control.transverse_contact_active = runtime.transverse_contact_active;
        runtime.control.rotational_contact_active = runtime.rotational_contact_active;
        runtime.control.contact_active =
            runtime.normal_contact_active || runtime.transverse_contact_active ||
            runtime.rotational_contact_active;
    };
    sync_contact_telemetry();

    double actual_normal_velocity = 0.0;
    math::Vector3 actual_linear_velocity_stand = math::Vector3::Zero();
    math::Vector3 actual_angular_velocity_stand = math::Vector3::Zero();
    if (runtime.previous_actual_pose_ns != 0 && now_ns > runtime.previous_actual_pose_ns) {
        const double pose_dt = static_cast<double>(now_ns - runtime.previous_actual_pose_ns) * 1e-9;
        const math::Vector3 delta(
            tcp.x - runtime.previous_actual_pose.x,
            tcp.y - runtime.previous_actual_pose.y,
            tcp.z - runtime.previous_actual_pose.z
        );
        actual_linear_velocity_stand = delta / pose_dt;
        const math::Matrix3 previous_rotation =
            math::rotationFromPose(runtime.previous_actual_pose);
        actual_angular_velocity_stand =
            math::log3(r_stand_tcp * previous_rotation.transpose()) / pose_dt;
        actual_normal_velocity = normal.dot(delta) / pose_dt;
    }
    const math::Vector3 actual_linear_velocity_tcp =
        r_stand_tcp.transpose() * actual_linear_velocity_stand;
    const math::Vector3 actual_angular_velocity_tcp =
        r_stand_tcp.transpose() * actual_angular_velocity_stand;
    runtime.actual_twist_compliance = transformTwistFrame(
        t_tcp_compliance.inverse(),
        {
            actual_linear_velocity_tcp.x(), actual_linear_velocity_tcp.y(),
            actual_linear_velocity_tcp.z(), actual_angular_velocity_tcp.x(),
            actual_angular_velocity_tcp.y(), actual_angular_velocity_tcp.z(),
        }
    );
    runtime.previous_actual_pose = tcp;
    runtime.previous_actual_pose_ns = now_ns;

    if (!runtime.control.enabled) {
        runtime.control.state = "monitoring";
        return false;
    }
    const std::string& mode = config_.force_control.operating_mode;
    if (mode == "monitor") {
        runtime.control.state = "monitoring";
        return false;
    }

    const bool hard_limit = runtime.control.hard_limit_exceeded;
    // hard_limit_policy retreat: the debounced hard limit starts a bounded
    // retreat episode inside the cartesian_admittance path below instead of
    // latching. The scalar contact_force normal episode keeps the latch —
    // its frozen-normal ownership predates the generic escape machinery.
    const bool retreat_capable =
        config_.force_control.hard_limit_policy == "retreat" &&
        mode == "cartesian_admittance" && !contact_force_surface;
    if (hard_limit && !retreat_capable) {
        runtime.control.state = "fault";
        runtime.control.fault_reason = "external force/torque hard limit exceeded";
        if (fault_reason) *fault_reason = runtime.control.fault_reason;
        return true;
    }

    const double controller_dt_sec =
        1.0 / static_cast<double>(config_.force_control.update_rate_hz);
    if (mode == "cartesian_admittance") {
        if (contact_force_surface && runtime.release_hold_pending) {
            runtime.control.state = "release_hold";
            return false;
        }
        if (contact_force_surface && arm_config.compliance_frame == "surface" &&
            !runtime.t_tcp_compliance_valid) {
            // A surface-frame Cartesian controller has no defined axes before
            // measured contact establishes the episode normal.
            runtime.control.state = "armed";
            return false;
        }
        if (!cartesian_controller.engaged()) cartesian_controller.engage();
        ForceControlCommand cartesian_command;
        cartesian_command.mode = ForceControlMode::Admittance;
        cartesian_command.enabled_axis = arm_config.compliance_axes;
        // Bleed only under a live policy equilibrium. A static hold anchor
        // frozen at an in-surface pose (client aborted mid-press) turns the
        // bleed into a perpetual touch-escape-drain bounce: the drain itself
        // re-creates the contact (2026-07-23 14:20 Hold oscillation, ~0.6 s /
        // 7 mm / 5-9 N). On Hold the escaped offset is simply kept and the
        // arm rests contact-free until a policy target moves the equilibrium.
        cartesian_command.allow_release_bleed =
            runtime.rolling_compliance_target_source == "policy_target";
        Wrench6D cartesian_wrench = runtime.control_wrench_compliance;
        Vec6 cartesian_twist = runtime.actual_twist_compliance;
        if (contact_force_surface && runtime.normal_contact_active) {
            const math::Matrix3 r_stand_compliance =
                t_stand_compliance.rotation();
            math::Vector3 tangent_force_stand = r_stand_compliance * math::Vector3(
                cartesian_wrench.fx, cartesian_wrench.fy, cartesian_wrench.fz);
            tangent_force_stand -= normal * normal.dot(tangent_force_stand);
            const math::Vector3 tangent_force_compliance =
                r_stand_compliance.transpose() * tangent_force_stand;
            cartesian_wrench.fx = tangent_force_compliance.x();
            cartesian_wrench.fy = tangent_force_compliance.y();
            cartesian_wrench.fz = tangent_force_compliance.z();

            math::Vector3 tangent_velocity_stand =
                r_stand_compliance * math::Vector3(
                    cartesian_twist.x, cartesian_twist.y, cartesian_twist.z);
            tangent_velocity_stand -= normal * normal.dot(tangent_velocity_stand);
            const math::Vector3 tangent_velocity_compliance =
                r_stand_compliance.transpose() * tangent_velocity_stand;
            cartesian_twist.x = tangent_velocity_compliance.x();
            cartesian_twist.y = tangent_velocity_compliance.y();
            cartesian_twist.z = tangent_velocity_compliance.z();
        }
        // Hard-limit retreat episode management (hard_limit_policy: retreat).
        if (retreat_capable) {
            const auto retreat_latch = [&](const std::string& why) {
                runtime.retreat_active = false;
                runtime.control.state = "fault";
                runtime.control.fault_reason = why;
                if (fault_reason) *fault_reason = why;
                return true;
            };
            if (hard_limit && !runtime.retreat_active &&
                !fault_latched_.load()) {
                if (config_.force_control.retreat_max_attempts > 0) {
                    const uint64_t window_ns = static_cast<uint64_t>(
                        config_.force_control.retreat_attempt_window_sec * 1e9);
                    if (runtime.retreat_window_start_ns == 0 ||
                        now_ns - runtime.retreat_window_start_ns > window_ns) {
                        runtime.retreat_window_start_ns = now_ns;
                        runtime.retreat_attempt_count = 0;
                    }
                    if (runtime.retreat_attempt_count >=
                        config_.force_control.retreat_max_attempts) {
                        return retreat_latch(
                            "external force/torque hard limit exceeded "
                            "(retreat attempts exhausted)");
                    }
                }
                const double press_norm = force_fast_stand.norm();
                if (!(press_norm > 1e-6)) {
                    // Torque-only trigger or degenerate force: no defined
                    // translation escape direction — keep the latch.
                    return retreat_latch(
                        "external force/torque hard limit exceeded "
                        "(no retreat direction)");
                }
                ++runtime.retreat_attempt_count;
                runtime.retreat_active = true;
                runtime.retreat_braking = false;
                runtime.retreat_virtual_current_n =
                    config_.force_control.retreat_virtual_force_n;
                const math::Vector3 press = force_fast_stand / press_norm;
                runtime.retreat_press_stand = {press.x(), press.y(), press.z()};
                runtime.retreat_started_ns = now_ns;
                const Pose6D& off0 = cartesian_controller.state().offset_tcp;
                runtime.retreat_start_offset = {off0.x, off0.y, off0.z};
            } else if (runtime.retreat_active) {
                const math::Vector3 press_stand(
                    runtime.retreat_press_stand[0],
                    runtime.retreat_press_stand[1],
                    runtime.retreat_press_stand[2]);
                const math::Vector3 escape_compliance =
                    t_stand_compliance.rotation().transpose() * (-press_stand);
                const auto& committed = cartesian_controller.state();
                const Pose6D& off = committed.offset_tcp;
                const math::Vector3 offset_delta(
                    off.x - runtime.retreat_start_offset[0],
                    off.y - runtime.retreat_start_offset[1],
                    off.z - runtime.retreat_start_offset[2]);
                const bool displaced = offset_delta.dot(escape_compliance) >=
                    config_.force_control.retreat_distance_m;
                const bool unloaded =
                    !runtime.control.hard_limit_threshold_exceeded;
                // Budget guard: the escape must remain brakeable inside the
                // UNBOOSTED envelope before the per-axis offset cap. The
                // Layer-3 dynamic scale follows the wrench error, so once the
                // real press decays the acceleration limit collapses back to
                // base — a fast state near the cap then fails the jerk
                // governor's brake-before-cap oracle and would fault
                // (2026-07-23 12:55 run). Enter braking with margin instead.
                const double escape_speed =
                    escape_compliance.x() * committed.velocity_tcp.x +
                    escape_compliance.y() * committed.velocity_tcp.y +
                    escape_compliance.z() * committed.velocity_tcp.z;
                const double worst_axis_offset = std::max({
                    std::abs(off.x), std::abs(off.y), std::abs(off.z)});
                const double budget_m = std::max(
                    1e-4,
                    config_.force_control.max_pos_offset_m - worst_axis_offset);
                const double base_acc =
                    config_.force_control.max_linear_acceleration_m_s2;
                const bool budget_low =
                    escape_speed > 0.0 &&
                    escape_speed * escape_speed / (2.0 * budget_m) >=
                        0.5 * base_acc;
                const uint64_t timeout_ns = static_cast<uint64_t>(
                    config_.force_control.retreat_timeout_sec * 1e9);
                if (!runtime.retreat_braking &&
                    (displaced || budget_low ||
                     worst_axis_offset >=
                         0.8 * config_.force_control.max_pos_offset_m)) {
                    runtime.retreat_braking = true;
                }
                if (runtime.retreat_braking) {
                    // Ramp the virtual wrench off (~50 ms) so the Layer-3
                    // envelope shrinks continuously while damping brakes the
                    // escape; a step release strands the state outside the
                    // collapsed envelope.
                    const double release_rate_n_s =
                        config_.force_control.retreat_virtual_force_n / 0.05;
                    runtime.retreat_virtual_current_n = std::max(
                        0.0,
                        runtime.retreat_virtual_current_n -
                            release_rate_n_s * controller_dt_sec);
                    if (runtime.retreat_virtual_current_n <= 0.0) {
                        runtime.retreat_active = false;
                        runtime.retreat_braking = false;
                        if (unloaded) runtime.hard_limit_count = 0;
                    }
                } else if (now_ns - runtime.retreat_started_ns > timeout_ns) {
                    if (unloaded) {
                        // Partial escape but the force is back under the hard
                        // threshold — ramp off and resume; Layer-3 keeps
                        // handling any residual load.
                        runtime.retreat_braking = true;
                    } else {
                        // Retreating did not unload (wedged/jammed): final stop.
                        return retreat_latch(
                            "external force/torque hard limit exceeded "
                            "(retreat timed out without unloading)");
                    }
                }
            }
            if (runtime.retreat_active &&
                runtime.retreat_virtual_current_n > 0.0) {
                // Virtual wrench: amplify the measured press so the existing
                // admittance escape (and its Layer-3 envelope opening) drives
                // the offset away from the contact. Telemetry keeps the real
                // measured wrench; only the controller drive sees the boost.
                const math::Vector3 press_compliance =
                    t_stand_compliance.rotation().transpose() * math::Vector3(
                        runtime.retreat_press_stand[0],
                        runtime.retreat_press_stand[1],
                        runtime.retreat_press_stand[2]);
                const double boost = runtime.retreat_virtual_current_n;
                cartesian_wrench.fx += press_compliance.x() * boost;
                cartesian_wrench.fy += press_compliance.y() * boost;
                cartesian_wrench.fz += press_compliance.z() * boost;
            }
        }
        // During contact_force, the symmetric Cartesian controller receives
        // only tangential translation; the scalar unilateral controller below
        // exclusively owns the frozen measured normal.
        const ForceControllerProposal cartesian_proposal = cartesian_controller.propose(
            cartesian_wrench,
            cartesian_command,
            cartesian_twist,
            controller_dt_sec
        );
        runtime.pending_cartesian_proposal = cartesian_proposal;
        runtime.control.proposal_valid = cartesian_proposal.valid;
        runtime.control.saturated = cartesian_proposal.saturated;
        runtime.control.wrench_error_compliance = cartesian_proposal.wrench_error_tcp;
        // Stand-frame loading direction for the chunk follower's wrench-gated
        // projection: the negation of the deadband-filtered wrench error, the
        // same convention the policy-delta loading projection uses below.
        if (contact_force_surface && runtime.normal_contact_active) {
            runtime.follower_loading_reaction_stand = {
                -normal.x(), -normal.y(), -normal.z()
            };
            runtime.follower_loading_reaction_valid = true;
            runtime.follower_contact_normal_owned = true;
        } else {
            const math::Vector3 reaction_compliance(
                -cartesian_proposal.wrench_error_tcp.fx,
                -cartesian_proposal.wrench_error_tcp.fy,
                -cartesian_proposal.wrench_error_tcp.fz
            );
            const math::Vector3 reaction_stand =
                t_stand_compliance.rotation() * reaction_compliance;
            runtime.follower_loading_reaction_stand = {
                reaction_stand.x(), reaction_stand.y(), reaction_stand.z()
            };
            // Layer-4 gate (generic redesign 2026-07-18): project policy
            // loading only while the deadband-filtered force EXCEEDS the
            // Layer-3 force limit — the same threshold that opens the
            // back-off envelope. Grip/inertial transit residuals measured
            // 2.5-9.5 N stay far below a 15 N limit, so no classifier or
            // frozen-normal state is needed; the direction refreshes every
            // tick from the measured wrench. force_limit_n <= 0 disables the
            // layer entirely.
            runtime.follower_loading_reaction_valid = runtime.control.enabled &&
                config_.force_control.force_limit_n > 0.0 &&
                reaction_compliance.norm() >= config_.force_control.force_limit_n;
        }
        const pinocchio::SE3 t_tcp_surface(
            r_stand_tcp.transpose() * r_stand_surface,
            math::Vector3::Zero()
        );
        runtime.control.wrench_error_surface = transformWrenchFrame(
            t_tcp_surface.inverse() * t_tcp_compliance,
            cartesian_proposal.wrench_error_tcp
        );
        runtime.control.compliance_limit_axes = cartesian_proposal.limit_axes;
        runtime.control.compliance_limit_reason = cartesian_proposal.limit_reason;
        runtime.control.compliance_translation_recenter_coupled =
            cartesian_proposal.translation_recenter_coupled;
        runtime.control.compliance_rotation_recenter_coupled =
            cartesian_proposal.rotation_recenter_coupled;
        runtime.control.compliance_translation_recenter_deferred =
            cartesian_proposal.translation_recenter_deferred;
        runtime.control.compliance_rotation_recenter_deferred =
            cartesian_proposal.rotation_recenter_deferred;
        runtime.control.compliance_offset_surface = cartesian_proposal.state.offset_tcp;
        runtime.control.compliance_velocity_surface = cartesian_proposal.state.velocity_tcp;
        runtime.control.compliance_acceleration_surface =
            cartesian_proposal.state.acceleration_tcp;
        const Pose6D& offset = cartesian_proposal.state.offset_tcp;
        const Vec6& velocity = cartesian_proposal.state.velocity_tcp;
        const Wrench6D& error = cartesian_proposal.wrench_error_tcp;
        const bool linear_residual =
            std::sqrt(offset.x * offset.x + offset.y * offset.y + offset.z * offset.z) > 1e-8 ||
            std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y +
                      velocity.z * velocity.z) > 1e-8;
        const bool angular_residual =
            std::sqrt(offset.rx * offset.rx + offset.ry * offset.ry +
                      offset.rz * offset.rz) > 1e-8 ||
            std::sqrt(velocity.rx * velocity.rx + velocity.ry * velocity.ry +
                      velocity.rz * velocity.rz) > 1e-8;
        const bool wrench_drive =
            std::abs(error.fx) > 1e-12 || std::abs(error.fy) > 1e-12 ||
            std::abs(error.fz) > 1e-12 || std::abs(error.tx) > 1e-12 ||
            std::abs(error.ty) > 1e-12 || std::abs(error.tz) > 1e-12;
        runtime.control.normal_regulating =
            runtime.normal_contact_active || std::abs(offset.z) > 1e-8 ||
            std::abs(velocity.z) > 1e-8 || std::abs(error.fz) > 1e-12;
        runtime.control.transverse_regulating =
            runtime.transverse_contact_active ||
            std::hypot(offset.x, offset.y) > 1e-8 ||
            std::hypot(velocity.x, velocity.y) > 1e-8 ||
            std::hypot(error.fx, error.fy) > 1e-12;
        runtime.control.rotational_regulating =
            runtime.rotational_contact_active ||
            std::sqrt(offset.rx * offset.rx + offset.ry * offset.ry +
                      offset.rz * offset.rz) > 1e-8 ||
            std::sqrt(velocity.rx * velocity.rx + velocity.ry * velocity.ry +
                      velocity.rz * velocity.rz) > 1e-8 ||
            std::sqrt(error.tx * error.tx + error.ty * error.ty +
                      error.tz * error.tz) > 1e-12;
        runtime.control.compliance_active =
            runtime.control.contact_active || linear_residual || angular_residual ||
            wrench_drive;
        runtime.control.compliance_recenter_active =
            !wrench_drive && (linear_residual || angular_residual);
        runtime.control.target_force_n =
            contact_force_surface && runtime.normal_contact_active
                ? arm_config.target_force_n : 0.0;
        runtime.control.energy_j = cartesian_proposal.state.observed_energy_j;
        if (!cartesian_proposal.valid) {
            runtime.control.state = "fault";
            runtime.control.fault_reason = cartesian_proposal.reason;
            if (fault_reason) {
                *fault_reason = "Cartesian force controller: " + cartesian_proposal.reason;
            }
            return true;
        }
        runtime.control.state = runtime.retreat_active
            ? "retreating"
            : (runtime.control.compliance_active ? "regulating" : "armed");
        if (!(contact_force_surface && runtime.normal_contact_active)) {
            return false;
        }
    }

    // A release hold is retried until the measured-pose target reaches the
    // accepted backend path. Do not re-enter contact or revive a stale policy
    // target in the interval between motion_epoch publication and client
    // re-anchoring.
    if (runtime.release_hold_pending) {
        runtime.control.state = "release_hold";
        return false;
    }

    if (!runtime.normal_contact_active) {
        runtime.control.state = "armed";
        if (output.freshness_advanced &&
            measured_normal >= arm_config.contact_enter_force_n) {
            ++runtime.enter_count;
        } else if (output.freshness_advanced) {
            runtime.enter_count = 0;
        }
        if (runtime.enter_count >= arm_config.debounce_samples) {
            runtime.normal_contact_active = true;
            sync_contact_telemetry();
            runtime.contact_anchor = tcp;
            runtime.contact_anchor_valid = true;
            runtime.release_start_ns = 0;
            runtime.enter_count = 0;
            if (mode != "cartesian_admittance") {
                ++motion_epoch_;
                runtime.control.motion_epoch = motion_epoch_;
                if (arm_id == ArmId::Left) left_output_ma_.reset();
                else right_output_ma_.reset();
            }
            if (mode == "guard") {
                runtime.control.state = "fault";
                runtime.control.fault_reason = "force guard contact threshold exceeded";
                if (fault_reason) *fault_reason = runtime.control.fault_reason;
                return true;
            }
            controller.engage();
            runtime.control.state = "regulating";
        }
    }

    if (!runtime.normal_contact_active) {
        sync_contact_telemetry();
        runtime.control.state = runtime.control.compliance_active ? "regulating" : "armed";
        return false;
    }

    runtime.control.normal_regulating = true;
    runtime.control.compliance_active =
        mode == "cartesian_admittance" || runtime.control.normal_regulating;

    // Release semantics differ by surface source. The legacy thresholds
    // (release 2.75 N) detect "contact is ending" for a compliance that never
    // TARGETS a contact force. A contact_force guarded episode REGULATES
    // 2.0 N, i.e. permanently below 2.75 N, so the legacy threshold released
    // the instant regulation succeeded and oscillated regulating <->
    // release_braking (2026-07-18 15:13 run, 3 bounces in 80 ms) before
    // deadlocking in release_hold. A guarded episode ends only when contact
    // truly vanishes: normal force inside the sensor deadband (e.g. the
    // grasped bolt lifted off the floor), still debounced by the existing
    // velocity threshold + release dwell.
    const double release_threshold_n = contact_force_surface
        ? arm_config.force_deadband_n
        : arm_config.contact_release_force_n;
    const bool below_release = measured_normal <= release_threshold_n;
    if (below_release) {
        runtime.control.state = "release_braking";
        if (std::abs(controller.state().unload_velocity_m_s) <=
            arm_config.release_velocity_threshold_m_s) {
            if (runtime.release_start_ns == 0) runtime.release_start_ns = now_ns;
        } else {
            runtime.release_start_ns = 0;
        }
        const double release_sec = runtime.release_start_ns == 0
            ? 0.0
            : static_cast<double>(now_ns - runtime.release_start_ns) * 1e-9;
        if (runtime.release_start_ns != 0 &&
            release_sec >= arm_config.release_dwell_sec) {
            // Latch the fresh measured pose before resetting the correction.
            // Target generation consumes this one-shot hold even though
            // contact_active is cleared, closing the release-tick stale-target
            // race until flow-infer observes motion_epoch and re-anchors.
            runtime.release_hold_pose = tcp;
            runtime.release_hold_pending = true;
            runtime.release_hold_applied = false;
            runtime.release_hold_clear_requested = false;
            runtime.normal_contact_active = false;
            if (contact_force_surface) {
                runtime.contact_force_normal_estimator.update(
                    false, math::Vector3::Zero());
                runtime.control.normal_stand = {0.0, 0.0, 0.0};
                runtime.follower_loading_reaction_valid = false;
                runtime.follower_contact_normal_owned = false;
                runtime.pending_cartesian_proposal.reset();
                runtime.pending_cartesian_proposal_applied = false;
                cartesian_controller.reset();
                cartesian_controller.engage();
                runtime.rolling_compliance_target = tcp;
                runtime.previous_raw_compliance_target = tcp;
                runtime.rolling_compliance_target_valid = true;
                runtime.rolling_compliance_target_source = "release_hold";
                runtime.contact_cartesian_normal_offset_m = 0.0;
            }
            sync_contact_telemetry();
            runtime.contact_anchor = tcp;
            runtime.contact_anchor_valid = true;
            runtime.release_start_ns = 0;
            controller.release();
            runtime.control.correction_m = 0.0;
            runtime.control.velocity_m_s = 0.0;
            runtime.control.acceleration_m_s2 = 0.0;
            runtime.control.energy_j = 0.0;
            runtime.control.saturated = false;
            ++motion_epoch_;
            runtime.control.motion_epoch = motion_epoch_;
            runtime.control.state = "release_hold";
            if (arm_id == ArmId::Left) {
                left_delta_twist_follower_.deactivate();
                left_chunk_follower_.deactivate();
                left_pose_track_smd_.deactivate();
                left_chunk_submitted_recv_seq_ = chunk_frame_cache_recv_seq_;
                left_output_ma_.reset();
            } else {
                right_delta_twist_follower_.deactivate();
                right_chunk_follower_.deactivate();
                right_pose_track_smd_.deactivate();
                right_chunk_submitted_recv_seq_ = chunk_frame_cache_recv_seq_;
                right_output_ma_.reset();
            }
            return false;
        }
    } else {
        runtime.release_start_ns = 0;
        runtime.control.state = "regulating";
    }

    NormalForceControllerCommand command;
    command.target_contact_force_n = arm_config.target_force_n;
    command.brake_to_hold = runtime.control.state == "release_braking";
    // The controller is configured and validated to run at the servo rate.
    // Use that fixed control period for its discrete jerk envelope: feeding
    // sub-percent scheduler jitter made a state accepted on one tick become
    // infeasible on the next (2.005 ms -> 1.995 ms in the physical capture).
    const NormalForceControllerProposal proposal = controller.propose(
        measured_normal,
        command,
        actual_normal_velocity,
        controller_dt_sec
    );
    runtime.pending_proposal = proposal;
    runtime.control.proposal_valid = proposal.valid;
    runtime.control.saturated = proposal.saturated;
    runtime.control.correction_m = proposal.state.unload_offset_m;
    runtime.control.velocity_m_s = proposal.state.unload_velocity_m_s;
    runtime.control.acceleration_m_s2 = proposal.state.unload_acceleration_m_s2;
    runtime.control.energy_j = proposal.state.observed_energy_j;
    if (!proposal.valid) {
        runtime.control.state = "fault";
        runtime.control.fault_reason = proposal.reason;
        if (fault_reason) *fault_reason = "normal force controller: " + proposal.reason;
        return true;
    }
    return false;
}

void DualArmServoLoop::applyForceCorrection(ArmId arm_id, ArmCommand* command) {
    if (!command || command->mode != ControlMode::TcpPoseTarget || !command->has_tcp_target) {
        return;
    }
    ForceArmRuntime& runtime =
        arm_id == ArmId::Left ? left_force_runtime_ : right_force_runtime_;
    const ForceControlArmConfig& arm_config = arm_id == ArmId::Left
        ? config_.force_control.left : config_.force_control.right;
    const FtWrenchPipelineConfig& ft_config = arm_id == ArmId::Left
        ? config_.force_torque.left : config_.force_torque.right;
    const bool cartesian_mode =
        config_.force_control.operating_mode == "cartesian_admittance";
    const bool normal_available =
        runtime.normal_contact_active && runtime.contact_anchor_valid &&
        runtime.pending_proposal.has_value() && runtime.pending_proposal->valid;
    const bool cartesian_available =
        cartesian_mode && runtime.pending_cartesian_proposal.has_value() &&
        runtime.pending_cartesian_proposal->valid;
    if (!normal_available && !cartesian_available) {
        return;
    }
    const bool contact_normal_owned =
        arm_config.surface_source == "contact_force" &&
        runtime.normal_contact_active &&
        runtime.contact_force_normal_estimator.valid();
    math::Vector3 normal(
        runtime.control.normal_stand[0],
        runtime.control.normal_stand[1],
        runtime.control.normal_stand[2]
    );
    if (!contact_normal_owned && !(normal.squaredNorm() > 0.0)) {
        normal = math::Vector3(0.0, 0.0, 1.0);
    }
    const math::Matrix3 r_stand_surface = surfaceFrameFromNormal(normal);
    const math::Matrix3 r_stand_compliance =
        math::rotationFromPose(runtime.compliance_frame_actual_stand);

    if (cartesian_available) {
        const Pose6D raw_target = command->tcp_target_stand;
        Pose6D candidate_previous_raw_target = runtime.previous_raw_compliance_target;
        Pose6D candidate_rolling_target = runtime.rolling_compliance_target;
        std::string candidate_source = runtime.rolling_compliance_target_source;
        if (runtime.compliance_hold_target_this_tick) {
            if (!runtime.rolling_compliance_target_valid) {
                candidate_previous_raw_target = raw_target;
                candidate_rolling_target = raw_target;
            }
            candidate_source = "hold_anchor";
        } else if (!runtime.rolling_compliance_target_valid) {
            candidate_previous_raw_target = raw_target;
            candidate_rolling_target = raw_target;
            candidate_source = "policy_target";
        } else if (runtime.chunk_follower_drove_this_tick) {
            // The chunk follower produced this target and its wrench-gated
            // loading projection already removed contact loading (episode
            // tangent projection included under contact_force ownership).
            // Adopt it verbatim: running the equilibrium's own delta
            // projection ON TOP double-books the contact and lets the
            // equilibrium drift from the plan (2026-07-18 14:16 run: 11.6 mm
            // plan-vs-equilibrium gap consumed the 20 mm actual-lead budget
            // with only ~0.3 deg of real joint tracking error).
            candidate_previous_raw_target = raw_target;
            candidate_rolling_target = raw_target;
            candidate_source = "policy_target";
            runtime.control.loading_projection_active = false;
        } else {
            // On Hold -> policy ownership transfer, project the first policy
            // delta from the fixed Hold equilibrium as well. Resetting directly
            // to raw_target here would let a single contact-loading command
            // jump through the compliance boundary.
            if (runtime.rolling_compliance_target_source != "policy_target") {
                candidate_previous_raw_target = candidate_rolling_target;
                candidate_source = "policy_target";
            }
            math::Vector3 raw_delta_stand(
                raw_target.x - candidate_previous_raw_target.x,
                raw_target.y - candidate_previous_raw_target.y,
                raw_target.z - candidate_previous_raw_target.z
            );
            math::Vector3 raw_delta_surface =
                r_stand_compliance.transpose() * raw_delta_stand;
            const math::Vector3 original_linear_delta_surface = raw_delta_surface;
            if (contact_normal_owned) {
                // The scalar guarded controller owns both signs of motion on
                // the frozen normal for the entire episode. Policy translation
                // is therefore tangent-only, independent of wrench deadband.
                raw_delta_stand -= normal * normal.dot(raw_delta_stand);
                raw_delta_surface =
                    r_stand_compliance.transpose() * raw_delta_stand;
            } else {
                // Use only the wrench outside the configured deadband. This keeps
                // quiet sensor residuals from projecting the policy equilibrium.
                const math::Vector3 effective_reaction(
                    -runtime.control.wrench_error_compliance.fx,
                    -runtime.control.wrench_error_compliance.fy,
                    -runtime.control.wrench_error_compliance.fz
                );
                const double translation_loading =
                    effective_reaction.dot(raw_delta_surface);
                if (translation_loading > 0.0 &&
                    effective_reaction.squaredNorm() > 1e-12) {
                    raw_delta_surface -= effective_reaction *
                        (translation_loading / effective_reaction.squaredNorm());
                }
            }

            const math::Matrix3 raw_rotation = math::rotationFromPose(raw_target);
            const math::Matrix3 previous_raw_rotation =
                math::rotationFromPose(candidate_previous_raw_target);
            math::Vector3 raw_angular_delta_surface =
                r_stand_compliance.transpose() *
                math::log3(raw_rotation * previous_raw_rotation.transpose());
            const math::Vector3 original_angular_delta_surface =
                raw_angular_delta_surface;
            const math::Vector3 torque_reaction(
                -runtime.control.wrench_error_compliance.tx,
                -runtime.control.wrench_error_compliance.ty,
                -runtime.control.wrench_error_compliance.tz
            );
            const double rotational_loading =
                torque_reaction.dot(raw_angular_delta_surface);
            if (rotational_loading > 0.0 && torque_reaction.squaredNorm() > 1e-12) {
                raw_angular_delta_surface -= torque_reaction *
                    (rotational_loading / torque_reaction.squaredNorm());
            }

            runtime.control.raw_policy_delta_surface = {
                original_linear_delta_surface.x(), original_linear_delta_surface.y(),
                original_linear_delta_surface.z(), original_angular_delta_surface.x(),
                original_angular_delta_surface.y(), original_angular_delta_surface.z(),
            };
            runtime.control.accepted_policy_delta_surface = {
                raw_delta_surface.x(), raw_delta_surface.y(), raw_delta_surface.z(),
                raw_angular_delta_surface.x(), raw_angular_delta_surface.y(),
                raw_angular_delta_surface.z(),
            };
            runtime.control.loading_projection_active =
                (original_linear_delta_surface - raw_delta_surface).squaredNorm() > 1e-16 ||
                (original_angular_delta_surface - raw_angular_delta_surface).squaredNorm() > 1e-16;

            const math::Vector3 accepted_delta_stand =
                r_stand_compliance * raw_delta_surface;
            candidate_rolling_target.x += accepted_delta_stand.x();
            candidate_rolling_target.y += accepted_delta_stand.y();
            candidate_rolling_target.z += accepted_delta_stand.z();
            const math::Matrix3 rolling_rotation =
                math::rotationFromPose(candidate_rolling_target);
            const math::Matrix3 accepted_rotation =
                math::exp3(r_stand_compliance * raw_angular_delta_surface) * rolling_rotation;
            const math::Vector3 rolling_translation(
                candidate_rolling_target.x,
                candidate_rolling_target.y,
                candidate_rolling_target.z
            );
            candidate_rolling_target = math::poseFromSe3(
                pinocchio::SE3(accepted_rotation, rolling_translation)
            );
            candidate_previous_raw_target = raw_target;
        }
        runtime.pending_previous_raw_compliance_target = candidate_previous_raw_target;
        runtime.pending_rolling_compliance_target = candidate_rolling_target;
        runtime.pending_rolling_compliance_target_valid = true;
        runtime.pending_rolling_compliance_target_source = candidate_source;
        command->tcp_target_stand = candidate_rolling_target;
    }

    const math::Vector3 anchor(
        runtime.contact_anchor.x,
        runtime.contact_anchor.y,
        runtime.contact_anchor.z
    );
    math::Vector3 target(
        command->tcp_target_stand.x,
        command->tcp_target_stand.y,
        command->tcp_target_stand.z
    );
    // Reject only policy penetration along the accepted contact normal. A
    // policy-requested retreat remains available so contact cannot deadlock at
    // a positive target force. The force controller supplies the minimum
    // outward unload offset; tangential motion and orientation remain intact.
    if (normal_available) {
        const double policy_normal_offset = normal.dot(target - anchor);
        const double accepted_normal_offset = contact_normal_owned
            ? runtime.pending_proposal->state.unload_offset_m
            : std::max(
                policy_normal_offset,
                runtime.pending_proposal->state.unload_offset_m
            );
        target += normal * (accepted_normal_offset - policy_normal_offset);
        runtime.pending_proposal_applied = true;
    }
    command->tcp_target_stand.x = target.x();
    command->tcp_target_stand.y = target.y();
    command->tcp_target_stand.z = target.z();

    if (cartesian_available) {
        Pose6D offset = runtime.pending_cartesian_proposal->state.offset_tcp;
        if (contact_normal_owned) {
            math::Vector3 translation_stand = r_stand_compliance *
                math::Vector3(offset.x, offset.y, offset.z);
            translation_stand += normal * (
                runtime.contact_cartesian_normal_offset_m -
                normal.dot(translation_stand));
            const math::Vector3 translation_compliance =
                r_stand_compliance.transpose() * translation_stand;
            offset.x = translation_compliance.x();
            offset.y = translation_compliance.y();
            offset.z = translation_compliance.z();
        }
        const pinocchio::SE3 t_stand_tcp = math::se3FromPose(
            command->tcp_target_stand
        );
        const pinocchio::SE3 t_tcp_compliance = complianceFrameTcp(
            arm_config, ft_config, command->tcp_target_stand, r_stand_surface
        );
        const pinocchio::SE3 compliance_delta(
            math::exp3(math::Vector3(offset.rx, offset.ry, offset.rz)),
            math::Vector3(offset.x, offset.y, offset.z)
        );
        command->tcp_target_stand = math::poseFromSe3(
            t_stand_tcp * t_tcp_compliance * compliance_delta *
            t_tcp_compliance.inverse()
        );
        runtime.pending_cartesian_proposal_applied = true;
    }
}

void DualArmServoLoop::finishForceProposals(
    bool left_accepted,
    bool right_accepted,
    SafetyVerdict verdict
) {
    const bool downstream_clean = verdict == SafetyVerdict::Ok;
    const auto finish = [&](ForceArmRuntime& runtime,
                            NormalForceController& controller,
                            ForceController& cartesian_controller,
                            bool accepted) {
        runtime.control.proposal_committed = false;
        if (runtime.pending_proposal.has_value()) {
            if (accepted && downstream_clean && runtime.pending_proposal_applied) {
                runtime.control.proposal_committed = controller.commit(*runtime.pending_proposal);
            } else {
                controller.reject();
            }
            runtime.pending_proposal.reset();
            runtime.pending_proposal_applied = false;
            const NormalForceControllerState& state = controller.state();
            runtime.control.correction_m = state.unload_offset_m;
            runtime.control.velocity_m_s = state.unload_velocity_m_s;
            runtime.control.acceleration_m_s2 = state.unload_acceleration_m_s2;
            runtime.control.energy_j = state.observed_energy_j;
        }
        if (runtime.pending_cartesian_proposal.has_value()) {
            if (accepted && downstream_clean &&
                runtime.pending_cartesian_proposal_applied) {
                const bool committed = cartesian_controller.commit(
                    *runtime.pending_cartesian_proposal
                );
                if (committed && runtime.pending_rolling_compliance_target_valid) {
                    runtime.previous_raw_compliance_target =
                        runtime.pending_previous_raw_compliance_target;
                    runtime.rolling_compliance_target =
                        runtime.pending_rolling_compliance_target;
                    runtime.rolling_compliance_target_valid = true;
                    runtime.rolling_compliance_target_source =
                        runtime.pending_rolling_compliance_target_source;
                    runtime.control.compliance_equilibrium_stand =
                        runtime.rolling_compliance_target;
                    runtime.control.compliance_equilibrium_source =
                        runtime.rolling_compliance_target_source;
                }
                runtime.control.proposal_committed =
                    runtime.control.proposal_committed || committed;
            } else {
                cartesian_controller.reject();
            }
            runtime.pending_cartesian_proposal.reset();
            runtime.pending_cartesian_proposal_applied = false;
            runtime.pending_rolling_compliance_target_valid = false;
            runtime.pending_rolling_compliance_target_source = "unavailable";
            const ForceControllerState& state = cartesian_controller.state();
            runtime.control.compliance_offset_surface = state.offset_tcp;
            runtime.control.compliance_velocity_surface = state.velocity_tcp;
            runtime.control.compliance_acceleration_surface = state.acceleration_tcp;
        }
        if (runtime.release_hold_pending && runtime.release_hold_applied) {
            if (accepted && downstream_clean &&
                runtime.release_hold_clear_requested) {
                runtime.release_hold_pending = false;
                runtime.contact_anchor_valid = false;
            }
            runtime.release_hold_applied = false;
            runtime.release_hold_clear_requested = false;
        }
    };
    finish(
        left_force_runtime_, left_normal_force_controller_,
        left_cartesian_force_controller_, left_accepted
    );
    finish(
        right_force_runtime_, right_normal_force_controller_,
        right_cartesian_force_controller_, right_accepted
    );
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
    left_ft_pipeline_(config.force_torque.left),
    right_ft_pipeline_(config.force_torque.right),
    left_normal_force_controller_(makeNormalForceControllerConfig(config, ArmId::Left)),
    right_normal_force_controller_(makeNormalForceControllerConfig(config, ArmId::Right)),
    left_cartesian_force_controller_(config.force_control),
    right_cartesian_force_controller_(config.force_control),
    command_buffer_(command_buffer),
    logger_(logger),
    scope_publisher_(scope_publisher),
    kinematics_(nullptr),
    kinematics_injected_(kinematics != nullptr),
    left_traj_filter_(config.servo, config.safety),
    right_traj_filter_(config.servo, config.safety),
    safety_filter_(config.safety) {
    left_force_runtime_.tare_valid =
        !config_.force_torque.left.auto_tare_after_init_motion;
    right_force_runtime_.tare_valid =
        !config_.force_torque.right.auto_tare_after_init_motion;
    bool profile_found = false;
    const TcpPoseTargetProfileConfig initial_profile =
        selectTcpPoseTargetProfile(config.cartesian_control, config.cartesian_control.tcp_pose_target_profile_default, &profile_found);
    left_pose_track_smd_ = SmdPoseTracker(initial_profile.pose_track_smd);
    right_pose_track_smd_ = SmdPoseTracker(initial_profile.pose_track_smd);
    left_pose_track_profile_name_ = initial_profile.name;
    right_pose_track_profile_name_ = initial_profile.name;
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

        uint64_t worker_state_max_age_ns = std::max<uint64_t>(2 * nominal_period_ns, 1'000'000);
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

        std::string left_force_fault_reason;
        std::string right_force_fault_reason;
        // Backend state acquisition stamps RobotState::host_time_ns while the
        // read is in progress, so it is necessarily newer than loop_start.
        // Use a post-read monotonic timestamp for FT age/freshness checks;
        // passing loop_start makes every freshly acquired sample look like it
        // came from the future and rejects it as an invalid timestamp.
        const uint64_t force_update_now_ns = nowSteadyNs();
        const bool left_force_fault = updateForceRuntime(
            ArmId::Left,
            left_state,
            filter_dt_sec,
            force_update_now_ns,
            &left_force_fault_reason
        );
        const bool right_force_fault = updateForceRuntime(
            ArmId::Right,
            right_state,
            filter_dt_sec,
            force_update_now_ns,
            &right_force_fault_reason
        );
        if ((left_force_fault || right_force_fault) && !fault_latched_.load()) {
            const ArmId fault_arm = left_force_fault ? ArmId::Left : ArmId::Right;
            const std::string reason = left_force_fault
                ? left_force_fault_reason : right_force_fault_reason;
            const FaultContext context = classifyCommandValidation(
                SafetyVerdict::ExternalForceLimit,
                fault_arm,
                reason
            );
            ++motion_epoch_;
            left_force_runtime_.control.motion_epoch = motion_epoch_;
            right_force_runtime_.control.motion_epoch = motion_epoch_;
            latchFault(
                SafetyVerdict::ExternalForceLimit,
                reason,
                left_state,
                right_state,
                context
            );
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
        const auto is_payload_identification_arm = [](const ArmCommand& arm) {
            return arm.joint_target_profile ==
                JointTargetProfile::PayloadIdentification;
        };
        const auto is_plain_hold_arm = [](const ArmCommand& arm) {
            return arm.mode == ControlMode::Hold &&
                arm.joint_target_profile == JointTargetProfile::Direct &&
                !arm.has_joint_target && !arm.has_arrival_stop &&
                !arm.has_tcp_target && !arm.has_linear_move_duration &&
                !arm.has_linear_move_linear_speed &&
                !arm.has_linear_move_angular_speed &&
                !arm.has_linear_move_orientation_mode &&
                !arm.has_gripper && !arm.has_freedrive &&
                arm.force_control.mode == ForceControlMode::Off;
        };
        const auto is_payload_identification_target = [
            &is_payload_identification_arm
        ](const ArmCommand& arm) {
            return is_payload_identification_arm(arm) &&
                arm.mode == ControlMode::JointTarget && arm.has_joint_target &&
                finiteJointArray(arm.q_target_deg) && !arm.has_arrival_stop &&
                !arm.has_tcp_target && !arm.has_linear_move_duration &&
                !arm.has_linear_move_linear_speed &&
                !arm.has_linear_move_angular_speed &&
                !arm.has_linear_move_orientation_mode &&
                !arm.has_gripper && !arm.has_freedrive &&
                arm.force_control.mode == ForceControlMode::Off;
        };
        const bool left_payload_identification =
            is_payload_identification_arm(command.left);
        const bool right_payload_identification =
            is_payload_identification_arm(command.right);
        const bool payload_identification_profile_present =
            left_payload_identification || right_payload_identification;
        const bool payload_identification_shape_valid =
            left_payload_identification != right_payload_identification &&
            ((is_payload_identification_target(command.left) &&
              is_plain_hold_arm(command.right)) ||
             (is_payload_identification_target(command.right) &&
              is_plain_hold_arm(command.left)));
        bool payload_identification_shape_rejected = false;
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
        } else if (payload_identification_profile_present &&
                   !payload_identification_shape_valid) {
            // Payload identification is an exclusive one-arm calibration
            // operation. Reject the entire packet before Freedrive or any
            // motion primitive can consume a malformed peer command. Preserve
            // the profile only for rejection telemetry and to suppress
            // Cartesian force-Hold promotion on this packet.
            DualArmCommand rejected = makeHoldCommand(
                left_state, right_state, loop_start);
            rejected.seq = command.seq;
            rejected.host_time_ns = command.host_time_ns;
            rejected.source = command.source;
            rejected.lease = command.lease;
            rejected.left.timeout_sec = command.left.timeout_sec;
            rejected.right.timeout_sec = command.right.timeout_sec;
            rejected.left.joint_target_profile =
                command.left.joint_target_profile;
            rejected.right.joint_target_profile =
                command.right.joint_target_profile;
            command = rejected;
            payload_identification_shape_rejected = true;
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

        // Preserve the profile received from the authoritative command path.
        // applyInitMotionSequencer rewrites init_motion into direct waypoints,
        // while this field remains the operator-visible pre-rewrite profile.
        left_force_runtime_.ft.joint_target_profile =
            toString(command.left.joint_target_profile);
        right_force_runtime_.ft.joint_target_profile =
            toString(command.right.joint_target_profile);

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
        bool payload_identification_profile_rejected =
            payload_identification_shape_rejected;
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
            const auto apply_payload_identification_profile = [this](
                ArmId arm_id,
                ArmCommand* arm_command
            ) {
                if (!arm_command || arm_command->mode != ControlMode::JointTarget ||
                    arm_command->joint_target_profile !=
                        JointTargetProfile::PayloadIdentification) {
                    return true;
                }
                if (latchPayloadIdentificationInhibit(arm_id)) {
                    return true;
                }

                ForceArmRuntime& runtime = arm_id == ArmId::Left
                    ? left_force_runtime_ : right_force_runtime_;
                runtime.ft.tare_reason =
                    "payload_identification profile unavailable: enable profile, arm F/T, "
                    "and automatic post-InitMotion tare";
                arm_command->mode = ControlMode::Hold;
                arm_command->has_joint_target = false;
                return false;
            };
            const bool left_payload_profile_ok =
                apply_payload_identification_profile(ArmId::Left, &command.left);
            const bool right_payload_profile_ok =
                apply_payload_identification_profile(ArmId::Right, &command.right);
            payload_identification_profile_rejected =
                payload_identification_profile_rejected ||
                !left_payload_profile_ok || !right_payload_profile_ok;
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
            if (payload_identification_profile_rejected) {
                command_verdict = SafetyVerdict::InvalidCommand;
            }

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
        finishForceProposals(
            left_ok && !send_suppressed,
            right_ok && !send_suppressed,
            safety_verdict
        );

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
            sample.left_force_torque = left_force_runtime_.ft;
            sample.right_force_torque = right_force_runtime_.ft;
            sample.left_force_control = left_force_runtime_.control;
            sample.right_force_control = right_force_runtime_.control;
            latest_snapshot_.tick = sample.tick;
            latest_snapshot_.loop_start_time_ns = loop_start;
            latest_snapshot_.loop_end_time_ns = loop_end;
            latest_snapshot_.left_state = left_state;
            latest_snapshot_.right_state = right_state;
            latest_snapshot_.left_force_torque = left_force_runtime_.ft;
            latest_snapshot_.right_force_torque = right_force_runtime_.ft;
            latest_snapshot_.left_force_control = left_force_runtime_.control;
            latest_snapshot_.right_force_control = right_force_runtime_.control;
            const auto& payload_id = config_.force_torque.payload_identification;
            latest_snapshot_.payload_identification_config.enable = payload_id.enable;
            latest_snapshot_.payload_identification_config.observation_model =
                payload_id.observation_model;
            latest_snapshot_.payload_identification_config.wrench_convention =
                payload_id.wrench_convention;
            latest_snapshot_.payload_identification_config.min_poses = payload_id.min_poses;
            latest_snapshot_.payload_identification_config.arrival_tolerance_deg =
                payload_id.arrival_tolerance_deg;
            latest_snapshot_.payload_identification_config.settle_sec = payload_id.settle_sec;
            latest_snapshot_.payload_identification_config.samples_per_pose =
                payload_id.samples_per_pose;
            latest_snapshot_.payload_identification_config.max_force_stddev_n =
                payload_id.max_force_stddev_n;
            latest_snapshot_.payload_identification_config.max_torque_stddev_nm =
                payload_id.max_torque_stddev_nm;
            latest_snapshot_.payload_identification_config.max_force_fit_rms_n =
                payload_id.max_force_fit_rms_n;
            latest_snapshot_.payload_identification_config.max_torque_fit_rms_nm =
                payload_id.max_torque_fit_rms_nm;
            latest_snapshot_.payload_identification_config.max_design_condition_number =
                payload_id.max_design_condition_number;
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
    solve.follower_divergence_pos_m = abc.follower_divergence_pos_m;
    solve.follower_divergence_ang_rad = abc.follower_divergence_ang_rad;
    solve.follower_projection_error_m = abc.follower_projection_error_m;
    solve.follower_projection_error_rad = abc.follower_projection_error_rad;
    solve.follower_projection_error_count = abc.follower_projection_error_count;
    solve.follower_actual_lead_m = abc.follower_actual_lead_m;
    solve.follower_actual_lead_rad = abc.follower_actual_lead_rad;
    solve.follower_actual_lead_error_count = abc.follower_actual_lead_error_count;
    solve.follower_loading_projection_active = abc.follower_loading_projection_active;
    solve.follower_contact_shift_m = abc.follower_contact_shift_m;
    solve.follower_reanchor_count = abc.follower_reanchor_count;
    solve.follower_warm_resume_count = abc.follower_warm_resume_count;
    solve.safety_intervention_recent = abc.safety_intervention_recent;
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
    ArmId arm_id,
    const ArmCommand& command,
    const TcpPoseTargetProfileConfig& profile,
    control::CartesianChunkFollower* follower,
    RuckigFollowerConfig* built_cfg,
    std::uint64_t* submitted_wire_seq,
    std::uint64_t* submitted_recv_seq,
    SmdPoseTracker* smd_tracker,
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
    std::uint64_t& reanchor_count = arm_id == ArmId::Left
        ? left_chunk_follower_reanchor_count_
        : right_chunk_follower_reanchor_count_;
    std::uint64_t& warm_resume_count = arm_id == ArmId::Left
        ? left_chunk_follower_warm_resume_count_
        : right_chunk_follower_warm_resume_count_;
    abc.follower_reanchor_count = reanchor_count;
    abc.follower_warm_resume_count = warm_resume_count;
    abc.safety_intervention_recent = intervention_recent;
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
        log_transition();
        return with_stage_telemetry(applyPoseTrackSmd(
            command, profile.pose_track_smd, smd_tracker, kinematics_, mount,
            previous_sent_q_deg, dt_sec));
    };
    if (rf.enable && chunk_frame_receiver_ && command.mode == ControlMode::Hold &&
        follower->active()) {
        // The 2026-07-18 12:44 stream reached this path through two dispatch
        // shapes: a raw dual Hold bypassed the Cartesian branch entirely, while
        // Cartesian admittance promoted a raw Hold and took the force-hold branch.
        // Both used to call deactivate(), even though their 10--360 ms gaps were
        // far shorter than chunk_feed_timeout_sec. Freeze segment/window time;
        // the Hold path owns the robot until a bounded warm resume or expiry.
        const double now_sec = ChunkFrameReceiver::steadyNowSec();
        follower->pauseForHold(now_sec);
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
        resetChunkFollowerEngageWait(arm_id);
        return smd_fallback();
    }
    // Re-point the follower at the active profile's params when they change.
    // In-place (keeps the prewarmed ruckig OTG): cheap enough for the RT tick.
    if (ruckigFollowerConfigChanged(*built_cfg, rf)) {
        follower->reconfigure(makeChunkFollowerConfig(rf));
        *built_cfg = rf;
        *submitted_wire_seq = 0;
        *submitted_recv_seq = 0;
    }
    // Live reference = FK of the previously SENT joints (same anchor discipline
    // as the SMD path). The sent joints already CONTAIN the committed compliance
    // offset, and applyForceCorrection re-applies that offset downstream every
    // tick — so strip it here. Without this the engage-wait hold is not a fixed
    // point: hold target = FK(sent) + offset ratchets by one offset per tick and
    // crawls at the velocity clamp along the offset direction (2026-07-18 12:28
    // run: a chunk-feed gap deactivated the follower mid-transit with an
    // 10.7 mm persistent K=0 offset and the arm took off at ~0.5 m/s until the
    // inertial spike latched ExternalForceLimit). A cold-start re-engage
    // likewise anchors at the stripped pose so anchor + offset reproduces the
    // previous sent pose exactly (no double-counted offset jump).
    Pose6D reference = kinematics_->computeTcpStand(arm_id, previous_sent_q_deg, mount);
    {
        const ForceArmRuntime& reference_force_runtime =
            arm_id == ArmId::Left ? left_force_runtime_ : right_force_runtime_;
        if (reference_force_runtime.t_tcp_compliance_valid) {
            const ForceController& reference_cartesian_controller =
                arm_id == ArmId::Left ? left_cartesian_force_controller_
                                      : right_cartesian_force_controller_;
            reference = removeComplianceOffsetFromMeasured(
                reference,
                reference_force_runtime.t_tcp_compliance_pose,
                reference_cartesian_controller.state().offset_tcp);
        }
    }
    const auto hold_at_reference = [&]() {
        ArmCommand smoothed = command;
        smoothed.tcp_target_stand = reference;
        return smoothed;
    };
    if (follower->holdPaused()) {
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
                if (fault_policy && intervention_recent) {
                    follower->reanchor(reference);
                    ++reanchor_count;
                    abc.follower_reanchor_count = reanchor_count;
                    uint64_t& last_log_ns = arm_id == ArmId::Left
                        ? left_chunk_follower_reanchor_log_ns_
                        : right_chunk_follower_reanchor_log_ns_;
                    if (last_log_ns == 0 || now_ns < last_log_ns ||
                        now_ns - last_log_ns >= kFollowerDivergenceReanchorLogPeriodNs) {
                        std::cout << "[chunk_follower] " << toString(arm_id)
                                  << " divergence re-anchor (safety intervention)"
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
    const ForceArmRuntime& force_runtime =
        arm_id == ArmId::Left ? left_force_runtime_ : right_force_runtime_;
    // Contact-aware following: hand the follower this tick's deadband-filtered
    // loading direction so segment advances never integrate into a loaded
    // contact (invalid -> strict no-op, identical to the blind follower).
    follower->setExternalReaction(
        Eigen::Vector3d(force_runtime.follower_loading_reaction_stand[0],
                        force_runtime.follower_loading_reaction_stand[1],
                        force_runtime.follower_loading_reaction_stand[2]),
        force_runtime.follower_loading_reaction_valid,
        force_runtime.follower_contact_normal_owned);
    smoothed.tcp_target_stand = follower->tick(dt_sec);
    // Tell the force path this target is follower-produced (already loading-
    // projected) so the rolling equilibrium adopts it verbatim instead of
    // double-projecting and drifting away from the plan.
    (arm_id == ArmId::Left ? left_force_runtime_ : right_force_runtime_)
        .chunk_follower_drove_this_tick = true;
    if (delta_preview) {
        // Compliance-aware actual lead: strip the committed admittance offset
        // from the measured pose so deliberate compliance is not counted as
        // plan-vs-robot divergence (zero offset -> identity).
        Pose6D lead_feedback = actual_feedback_pose;
        if (force_runtime.t_tcp_compliance_valid) {
            const ForceController& cartesian_controller =
                arm_id == ArmId::Left ? left_cartesian_force_controller_
                                      : right_cartesian_force_controller_;
            lead_feedback = removeComplianceOffsetFromMeasured(
                lead_feedback, force_runtime.t_tcp_compliance_pose,
                cartesian_controller.state().offset_tcp);
        }
        follower->updateActualLead(lead_feedback);
    }
    const control::FollowerDiag& diag = follower->diag();
    abc.follower_projection_error_m = diag.projection_error_m;
    abc.follower_projection_error_rad = diag.projection_error_rad;
    abc.follower_projection_error_count = diag.consecutive_projection_errors;
    abc.follower_actual_lead_m = diag.actual_lead_m;
    abc.follower_actual_lead_rad = diag.actual_lead_rad;
    abc.follower_actual_lead_error_count = diag.consecutive_actual_lead_errors;
    abc.follower_loading_projection_active = diag.loading_projection_active;
    abc.follower_contact_shift_m = diag.contact_shift_m;
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
    std::uint64_t& reanchor_count = arm_id == ArmId::Left
        ? left_chunk_follower_reanchor_count_
        : right_chunk_follower_reanchor_count_;
    abc.follower_reanchor_count = reanchor_count;
    abc.safety_intervention_recent = intervention_recent;
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
                if (fault_policy && intervention_recent) {
                    follower->reanchor(reference);
                    ++reanchor_count;
                    abc.follower_reanchor_count = reanchor_count;
                    uint64_t& last_log_ns = arm_id == ArmId::Left
                        ? left_chunk_follower_reanchor_log_ns_
                        : right_chunk_follower_reanchor_log_ns_;
                    if (last_log_ns == 0 || now_ns < last_log_ns ||
                        now_ns - last_log_ns >= kFollowerDivergenceReanchorLogPeriodNs) {
                        std::cout << "[chunk_follower] " << toString(arm_id)
                                  << " delta_twist divergence re-anchor (safety intervention)"
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

ServoTarget DualArmServoLoop::computeServoTarget(
    const RobotState& left_state,
    const RobotState& right_state,
    const DualArmCommand& command,
    double dt_sec,
    SafetyVerdict* command_verdict
) {
    if (command_verdict) *command_verdict = SafetyVerdict::Ok;
    clearChunkFollowerFaultRequests();
    // Per-tick provenance: applyChunkFollowerStage sets these when the chunk
    // follower produces the Cartesian target this tick; every other dispatch
    // shape leaves them false so the equilibrium keeps its own projection.
    left_force_runtime_.chunk_follower_drove_this_tick = false;
    right_force_runtime_.chunk_follower_drove_this_tick = false;
    ServoTarget target;
    const bool synthetic_hold = isSyntheticHoldCommand(command);
    const auto clear_left_linear_path = [&]() {
        left_cartesian_servo_path_ = CartesianServoPathState{};
        left_last_cartesian_solve_ = CartesianSolveTelemetry{};
    };
    const auto clear_right_linear_path = [&]() {
        right_cartesian_servo_path_ = CartesianServoPathState{};
        right_last_cartesian_solve_ = CartesianSolveTelemetry{};
    };
    if (left_cartesian_servo_path_.active && !isValidJointState(left_state)) {
        clear_left_linear_path();
    }
    if (right_cartesian_servo_path_.active && !isValidJointState(right_state)) {
        clear_right_linear_path();
    }
    // TcpLinearMove is a FINITE, bounded path (duration_sec <= linear_move.max_duration_sec).
    // Once it is running, let it drive to completion even if the (one-shot) command's
    // lease lapses — a single click must always reach the target. The deadman still
    // applies once the path is DONE (cleared below), and an explicit command-mode change,
    // a fault, or an invalid joint state still abort it immediately above/below; E-stop
    // latches regardless. So only a FINISHED path is torn down on lease expiry.
    if (left_cartesian_servo_path_.done &&
        linearPathLeaseExpired(left_cartesian_servo_path_, command.host_time_ns)) {
        clear_left_linear_path();
    }
    if (right_cartesian_servo_path_.done &&
        linearPathLeaseExpired(right_cartesian_servo_path_, command.host_time_ns)) {
        clear_right_linear_path();
    }
    if (!synthetic_hold) {
        if (command.left.mode != ControlMode::TcpLinearMove) {
            clear_left_linear_path();
        }
        if (command.right.mode != ControlMode::TcpLinearMove) {
            clear_right_linear_path();
        }
    }

    const CartesianSolveTelemetry previous_left_cartesian_solve = left_last_cartesian_solve_;
    const CartesianSolveTelemetry previous_right_cartesian_solve = right_last_cartesian_solve_;
    left_last_cartesian_solve_ = retainCompletedPathTelemetry(
        left_cartesian_servo_path_,
        previous_left_cartesian_solve
    ) ? previous_left_cartesian_solve : CartesianSolveTelemetry{};
    right_last_cartesian_solve_ = retainCompletedPathTelemetry(
        right_cartesian_servo_path_,
        previous_right_cartesian_solve
    ) ? previous_right_cartesian_solve : CartesianSolveTelemetry{};

    if (isCommandModeMissingPayload(command.left) || isCommandModeMissingPayload(command.right)) {
        if (command_verdict) *command_verdict = SafetyVerdict::InvalidCommand;
        target.left_q_target_deg = left_prev_sent_q_deg_;
        target.right_q_target_deg = right_prev_sent_q_deg_;
        return target;
    }

    const bool continue_left_linear = synthetic_hold &&
        left_cartesian_servo_path_.active &&
        !left_cartesian_servo_path_.done;
    const bool continue_right_linear = synthetic_hold &&
        right_cartesian_servo_path_.active &&
        !right_cartesian_servo_path_.done;
    DualArmCommand effective_command = command;
    if (continue_left_linear) {
        effective_command.left = linearMoveContinuationCommand(command.left, left_cartesian_servo_path_);
    }
    if (continue_right_linear) {
        effective_command.right = linearMoveContinuationCommand(command.right, right_cartesian_servo_path_);
    }

    const auto invalidate_non_cartesian_equilibrium = [this](
        ForceArmRuntime& runtime,
        ControlMode raw_mode
    ) {
        if (config_.force_control.operating_mode == "cartesian_admittance" &&
            raw_mode != ControlMode::Hold && raw_mode != ControlMode::TcpPoseTarget) {
            runtime.rolling_compliance_target_valid = false;
            runtime.rolling_compliance_target_source = "unavailable";
            runtime.control.compliance_equilibrium_source = "unavailable";
        }
    };
    invalidate_non_cartesian_equilibrium(left_force_runtime_, command.left.mode);
    invalidate_non_cartesian_equilibrium(right_force_runtime_, command.right.mode);

    // A payload-identification packet is an exclusive joint-space operation.
    // Suppress Cartesian force-Hold promotion on BOTH arms for this packet: the
    // selected arm must stay on its JointTarget path, and the peer's explicit
    // Hold must remain a stationary joint hold. Checking the raw profile also
    // covers fail-closed admission, where the selected command was rewritten
    // to Hold because the profile was unavailable.
    const bool payload_identification_command =
        command.left.joint_target_profile ==
            JointTargetProfile::PayloadIdentification ||
        command.right.joint_target_profile ==
            JointTargetProfile::PayloadIdentification;

    // A Cartesian-admittance Hold is an active fixed equilibrium, not a fresh
    // measured-pose target each tick. This prevents the compliance offset from
    // ratcheting the command in the direction of motion.
    const auto promote_force_hold = [this, &command, payload_identification_command](
        ArmId arm_id,
        const RobotState& state,
        ControlMode raw_command_mode,
        ArmCommand* arm_command
    ) {
        if (!arm_command) {
            return false;
        }
        if (payload_identification_command) {
            return false;
        }
        ForceArmRuntime& runtime = arm_id == ArmId::Left
            ? left_force_runtime_ : right_force_runtime_;
        if (runtime.payload_identification_inhibit) {
            return false;
        }
        if (runtime.release_hold_pending) {
            arm_command->mode = ControlMode::TcpPoseTarget;
            arm_command->tcp_target_stand = runtime.release_hold_pose;
            arm_command->has_tcp_target = true;
            runtime.release_hold_applied = true;
            // flow-infer acknowledges the release epoch by returning a REAL
            // Hold during its re-anchor/reset cycle (legacy stream client) —
            // OR, in the delta_preview chunk era, by delivering a FRESH chunk
            // frame after the release latched the cache sequence: the
            // re-anchored chunk IS the client's acknowledgment. The chunk
            // client never sends a deliberate Hold, so the legacy-only
            // condition deadlocked release_hold for 34 s while the policy
            // re-inferred in a loop (2026-07-18 15:13 run). A stale
            // TcpPoseTarget stream still cannot clear the latch: the recorded
            // sequence only advances on genuinely new frames.
            const std::uint64_t release_submitted_seq = arm_id == ArmId::Left
                ? left_chunk_submitted_recv_seq_
                : right_chunk_submitted_recv_seq_;
            const bool arm_frame_present = arm_id == ArmId::Left
                ? chunk_frame_cache_.has_left
                : chunk_frame_cache_.has_right;
            const bool fresh_chunk_after_release =
                chunk_frame_cache_recv_seq_ != 0 &&
                chunk_frame_cache_recv_seq_ != release_submitted_seq &&
                arm_frame_present;
            runtime.release_hold_clear_requested =
                (raw_command_mode == ControlMode::Hold &&
                 !isSyntheticHoldCommand(command)) ||
                fresh_chunk_after_release;
            return true;
        }
        if (arm_command->mode != ControlMode::Hold ||
            !state.tcp_actual_valid || !state.tcp_actual_stand.has_value()) {
            return false;
        }
        if (config_.force_control.operating_mode == "cartesian_admittance") {
            if (!runtime.control.enabled ||
                !runtime.pending_cartesian_proposal.has_value() ||
                !runtime.pending_cartesian_proposal->valid) {
                return false;
            }
            arm_command->mode = ControlMode::TcpPoseTarget;
            arm_command->tcp_target_stand = runtime.rolling_compliance_target_valid
                ? runtime.rolling_compliance_target : *state.tcp_actual_stand;
            arm_command->has_tcp_target = true;
            runtime.compliance_hold_target_this_tick = true;
            return true;
        }
        if (!runtime.control.enabled || !runtime.normal_contact_active ||
            !runtime.contact_anchor_valid || !runtime.pending_proposal.has_value() ||
            !runtime.pending_proposal->valid) {
            return false;
        }
        arm_command->mode = ControlMode::TcpPoseTarget;
        arm_command->tcp_target_stand = *state.tcp_actual_stand;
        arm_command->has_tcp_target = true;
        return true;
    };
    const bool left_force_hold = promote_force_hold(
        ArmId::Left, left_state, command.left.mode, &effective_command.left);
    const bool right_force_hold = promote_force_hold(
        ArmId::Right, right_state, command.right.mode, &effective_command.right);

    const auto pause_chunk_follower_for_hold = [this](
        ArmId arm_id,
        ControlMode raw_mode,
        control::CartesianChunkFollower& follower,
        const RuckigFollowerConfig& built_config
    ) {
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
            if (command_verdict) *command_verdict = SafetyVerdict::CartesianUnavailable;
            if (isCartesianMode(effective_command.left.mode)) {
                left_last_cartesian_solve_ = cartesianUnavailableTelemetry(
                    left_state,
                    config_.cartesian_control,
                    left_unavailable_reason
                );
            }
            if (isCartesianMode(effective_command.right.mode)) {
                right_last_cartesian_solve_ = cartesianUnavailableTelemetry(
                    right_state,
                    config_.cartesian_control,
                    right_unavailable_reason
                );
            }
            target.left_q_target_deg = left_prev_sent_q_deg_;
            target.right_q_target_deg = right_prev_sent_q_deg_;
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
        if (!left_servo_state.ok || !right_servo_state.ok) {
            if (command_verdict) *command_verdict = SafetyVerdict::CartesianUnavailable;
            if (!left_servo_state.ok) {
                left_last_cartesian_solve_ = cartesianUnavailableTelemetry(
                    left_state,
                    config_.cartesian_control,
                    left_servo_state.reason
                );
                left_last_cartesian_solve_.cartesian_servo_state_source =
                    left_servo_state.context.servo_state_source;
                left_last_cartesian_solve_.cartesian_divergence_source =
                    left_servo_state.context.divergence_source;
                left_last_cartesian_solve_.q_reference_for_servo_valid =
                    left_servo_state.context.q_reference_for_servo_valid;
            }
            if (!right_servo_state.ok) {
                right_last_cartesian_solve_ = cartesianUnavailableTelemetry(
                    right_state,
                    config_.cartesian_control,
                    right_servo_state.reason
                );
                right_last_cartesian_solve_.cartesian_servo_state_source =
                    right_servo_state.context.servo_state_source;
                right_last_cartesian_solve_.cartesian_divergence_source =
                    right_servo_state.context.divergence_source;
                right_last_cartesian_solve_.q_reference_for_servo_valid =
                    right_servo_state.context.q_reference_for_servo_valid;
            }
            target.left_q_target_deg = left_prev_sent_q_deg_;
            target.right_q_target_deg = right_prev_sent_q_deg_;
            return target;
        }
        if (effective_command.left.mode == ControlMode::TcpPoseTarget &&
            left_pose_track_profile_name_ != left_tcp_profile.name) {
            left_pose_track_smd_ = SmdPoseTracker(left_tcp_profile.pose_track_smd);
            left_pose_track_profile_name_ = left_tcp_profile.name;
        }
        if (effective_command.right.mode == ControlMode::TcpPoseTarget &&
            right_pose_track_profile_name_ != right_tcp_profile.name) {
            right_pose_track_smd_ = SmdPoseTracker(right_tcp_profile.pose_track_smd);
            right_pose_track_profile_name_ = right_tcp_profile.name;
        }
        // Manipulability guard: feed the previous tick's IK min singular value into each
        // SMD so step() scales tracking velocity down near a singularity (config
        // singularity_scale_*). Velocity-only — cannot stall the IK.
        left_pose_track_smd_.setMinSingular(left_last_cartesian_solve_.ik_min_singular_value);
        right_pose_track_smd_.setMinSingular(right_last_cartesian_solve_.ik_min_singular_value);
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
            ArmId arm_id,
            const ArmMountConfig& mount,
            const JointArray& previous_sent_q_deg
        ) {
            if (state.has_valid_tcp_pose && state.tcp_stand.has_value()) {
                return *state.tcp_stand;
            }
            if (kinematics_) {
                return kinematics_->computeTcpStand(arm_id, previous_sent_q_deg, mount);
            }
            return Pose6D{};
        };
        const auto execution_feedback_pose_for_arm = [this](
            ArmId arm_id,
            const ArmMountConfig& mount,
            const JointArray& previous_sent_q_deg
        ) {
            if (kinematics_) {
                return kinematics_->computeTcpStand(arm_id, previous_sent_q_deg, mount);
            }
            return Pose6D{};
        };
        const Pose6D left_delta_twist_actual_feedback = servo_feedback_pose_for_arm(
            left_servo_state.state, ArmId::Left, config_.left_mount, left_prev_sent_q_deg_);
        const Pose6D right_delta_twist_actual_feedback = servo_feedback_pose_for_arm(
            right_servo_state.state, ArmId::Right, config_.right_mount, right_prev_sent_q_deg_);
        const Pose6D left_delta_twist_execution_feedback = execution_feedback_pose_for_arm(
            ArmId::Left, config_.left_mount, left_prev_sent_q_deg_);
        const Pose6D right_delta_twist_execution_feedback = execution_feedback_pose_for_arm(
            ArmId::Right, config_.right_mount, right_prev_sent_q_deg_);
        ArmCommand left_pose_track_command;
        if (left_force_hold) {
            pause_chunk_follower_for_hold(
                ArmId::Left,
                command.left.mode,
                left_chunk_follower_,
                left_chunk_follower_built_);
            left_delta_twist_follower_.deactivate();
            left_pose_track_smd_.deactivate();
            left_pose_track_command = effective_command.left;
        } else if (left_tcp_profile.ruckig_follower.controller == RuckigFollowerController::DeltaTwist) {
            left_chunk_follower_.deactivate();
            left_pose_track_command = applyDeltaTwistFollowerStage(
                ArmId::Left,
                effective_command.left,
                left_tcp_profile,
                &left_delta_twist_follower_,
                &left_chunk_follower_built_,
                &left_chunk_submitted_wire_seq_,
                &left_chunk_submitted_recv_seq_,
                &left_pose_track_smd_,
                config_.left_mount,
                left_prev_sent_q_deg_,
                left_delta_twist_actual_feedback,
                left_delta_twist_execution_feedback,
                dt_sec
            );
        } else {
            left_delta_twist_follower_.deactivate();
            left_pose_track_command = applyChunkFollowerStage(
                ArmId::Left,
                effective_command.left,
                left_tcp_profile,
                &left_chunk_follower_,
                &left_chunk_follower_built_,
                &left_chunk_submitted_wire_seq_,
                &left_chunk_submitted_recv_seq_,
                &left_pose_track_smd_,
                config_.left_mount,
                left_prev_sent_q_deg_,
                left_delta_twist_actual_feedback,
                dt_sec
            );
        }
        ArmCommand right_pose_track_command;
        if (right_force_hold) {
            pause_chunk_follower_for_hold(
                ArmId::Right,
                command.right.mode,
                right_chunk_follower_,
                right_chunk_follower_built_);
            right_delta_twist_follower_.deactivate();
            right_pose_track_smd_.deactivate();
            right_pose_track_command = effective_command.right;
        } else if (right_tcp_profile.ruckig_follower.controller == RuckigFollowerController::DeltaTwist) {
            right_chunk_follower_.deactivate();
            right_pose_track_command = applyDeltaTwistFollowerStage(
                ArmId::Right,
                effective_command.right,
                right_tcp_profile,
                &right_delta_twist_follower_,
                &right_chunk_follower_built_,
                &right_chunk_submitted_wire_seq_,
                &right_chunk_submitted_recv_seq_,
                &right_pose_track_smd_,
                config_.right_mount,
                right_prev_sent_q_deg_,
                right_delta_twist_actual_feedback,
                right_delta_twist_execution_feedback,
                dt_sec
            );
        } else {
            right_delta_twist_follower_.deactivate();
            right_pose_track_command = applyChunkFollowerStage(
                ArmId::Right,
                effective_command.right,
                right_tcp_profile,
                &right_chunk_follower_,
                &right_chunk_follower_built_,
                &right_chunk_submitted_wire_seq_,
                &right_chunk_submitted_recv_seq_,
                &right_pose_track_smd_,
                config_.right_mount,
                right_prev_sent_q_deg_,
                right_delta_twist_actual_feedback,
                dt_sec
            );
        }
        applyForceCorrection(ArmId::Left, &left_pose_track_command);
        applyForceCorrection(ArmId::Right, &right_pose_track_command);
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
        capture_smd_abc(left_pose_track_smd_, left_tcp_profile, left_profile_found, left_abc_telemetry_);
        capture_smd_abc(right_pose_track_smd_, right_tcp_profile, right_profile_found, right_abc_telemetry_);

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
            : left_prev_sent_q_deg_;
        const JointArray& right_cartesian_ik_seed_q_deg =
            right_servo_state.context.servo_state_source == "reference"
            ? right_servo_state.state.q_actual_deg
            : right_prev_sent_q_deg_;

        const CartesianArmTargetResult left_cartesian_result =
            effective_command.left.mode == ControlMode::TcpLinearMove
            ? cartesian_servo.computeLinearMoveTarget(
                effective_command.left,
                left_servo_state.state,
                left_cartesian_ik_seed_q_deg,
                left_cartesian_compute_run_mode,
                dt_sec,
                continue_left_linear ? 0 : command.seq,
                &left_cartesian_servo_path_,
                &left_servo_state.context
            )
            : isCartesianMode(effective_command.left.mode)
            ? cartesian.computeArmJointTarget(
                left_pose_track_command,
                left_servo_state.state,
                left_cartesian_ik_seed_q_deg,
                left_cartesian_compute_run_mode
            )
            : CartesianArmTargetResult{
                SafetyVerdict::Ok,
                left_traj_filter_.computeJointTarget(command.left, left_state, left_prev_sent_q_deg_, dt_sec),
                "",
                CartesianSolveTelemetry{}
            };
        target.left_q_target_deg = left_cartesian_result.q_target_deg;
        left_last_cartesian_solve_ = left_cartesian_result.telemetry;
        left_last_cartesian_solve_.cartesian_servo_state_source =
            left_servo_state.context.servo_state_source;
        left_last_cartesian_solve_.cartesian_divergence_source =
            left_servo_state.context.divergence_source;
        left_last_cartesian_solve_.q_reference_for_servo_valid =
            left_servo_state.context.q_reference_for_servo_valid;
        if (command.left.mode == ControlMode::TcpLinearMove && left_cartesian_servo_path_.active) {
            left_cartesian_servo_path_.lease_enforced = command.lease.enforce_lease;
            left_cartesian_servo_path_.lease_expires_time_ns = command.lease.expires_time_ns;
        }

        const CartesianArmTargetResult right_cartesian_result =
            effective_command.right.mode == ControlMode::TcpLinearMove
            ? cartesian_servo.computeLinearMoveTarget(
                effective_command.right,
                right_servo_state.state,
                right_cartesian_ik_seed_q_deg,
                right_cartesian_compute_run_mode,
                dt_sec,
                continue_right_linear ? 0 : command.seq,
                &right_cartesian_servo_path_,
                &right_servo_state.context
            )
            : isCartesianMode(effective_command.right.mode)
            ? cartesian.computeArmJointTarget(
                right_pose_track_command,
                right_servo_state.state,
                right_cartesian_ik_seed_q_deg,
                right_cartesian_compute_run_mode
            )
            : CartesianArmTargetResult{
                SafetyVerdict::Ok,
                right_traj_filter_.computeJointTarget(command.right, right_state, right_prev_sent_q_deg_, dt_sec),
                "",
                CartesianSolveTelemetry{}
            };
        target.right_q_target_deg = right_cartesian_result.q_target_deg;
        right_last_cartesian_solve_ = right_cartesian_result.telemetry;
        right_last_cartesian_solve_.cartesian_servo_state_source =
            right_servo_state.context.servo_state_source;
        right_last_cartesian_solve_.cartesian_divergence_source =
            right_servo_state.context.divergence_source;
        right_last_cartesian_solve_.q_reference_for_servo_valid =
            right_servo_state.context.q_reference_for_servo_valid;
        if (command.right.mode == ControlMode::TcpLinearMove && right_cartesian_servo_path_.active) {
            right_cartesian_servo_path_.lease_enforced = command.lease.enforce_lease;
            right_cartesian_servo_path_.lease_expires_time_ns = command.lease.expires_time_ns;
        }

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
            if (left_cartesian_result.verdict != SafetyVerdict::Ok) {
                target.left_q_target_deg = left_prev_sent_q_deg_;
            }
            if (right_cartesian_result.verdict != SafetyVerdict::Ok) {
                target.right_q_target_deg = right_prev_sent_q_deg_;
            }
        }
        return target;
    }

    // No arm is in Cartesian mode, so applyChunkFollowerStage() did not run.
    // A raw dual Hold reaches this dispatch path directly. Before warm-resume,
    // it unconditionally deactivated both followers; the force-Hold promotion
    // above was the second unconditional path seen in the 2026-07-18 12:44
    // bounce evidence. Preserve a bounded, frozen chunk only for raw Hold;
    // every other mode switch still drops the streaming state immediately.
    pause_chunk_follower_for_hold(
        ArmId::Left, command.left.mode, left_chunk_follower_, left_chunk_follower_built_);
    pause_chunk_follower_for_hold(
        ArmId::Right, command.right.mode, right_chunk_follower_, right_chunk_follower_built_);
    left_delta_twist_follower_.deactivate();
    right_delta_twist_follower_.deactivate();

    target.left_q_target_deg = left_traj_filter_.computeJointTarget(
        command.left,
        left_state,
        left_prev_sent_q_deg_,
        dt_sec
    );
    target.right_q_target_deg = right_traj_filter_.computeJointTarget(
        command.right,
        right_state,
        right_prev_sent_q_deg_,
        dt_sec
    );
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
        left_controller_sim_physical_baseline_q_deg_
    );
    const SafetyTrackingState right_tracking_state = trackingStateForArm(
        config_,
        ArmId::Right,
        right_filter_state,
        right_controller_sim_physical_baseline_q_deg_
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
        left_prev_sent_q_deg_,
        left_prevprev_sent_q_deg_,
        left_filter_state,
        dt_sec,
        left_tracking_state
    );
    const SafetyCheckResult right_result = safety_filter_.filterJointTarget(
        desired.right_q_target_deg,
        right_prev_sent_q_deg_,
        right_prevprev_sent_q_deg_,
        right_filter_state,
        dt_sec,
        right_tracking_state
    );
    left_safety_tracking_ = left_result.tracking;
    right_safety_tracking_ = right_result.tracking;
    left_abc_telemetry_.safety_clamp_present = left_result.clamp.present;
    left_abc_telemetry_.safety_clamp = left_result.clamp;
    right_abc_telemetry_.safety_clamp_present = right_result.clamp.present;
    right_abc_telemetry_.safety_clamp = right_result.clamp;

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
                left_prev_sent_q_deg_,
                left_prevprev_sent_q_deg_,
                dt_sec
            );
            const SafetyClampTelemetry right_clamp = safety_filter_.clampMotionDetailed(
                desired.right_q_target_deg,
                right_prev_sent_q_deg_,
                right_prevprev_sent_q_deg_,
                dt_sec
            );
            out.left_q_target_deg = left_clamp.q_after_accel_limit_deg;
            out.right_q_target_deg = right_clamp.q_after_accel_limit_deg;
            left_abc_telemetry_.safety_clamp_present = left_clamp.present;
            left_abc_telemetry_.safety_clamp = left_clamp;
            right_abc_telemetry_.safety_clamp_present = right_clamp.present;
            right_abc_telemetry_.safety_clamp = right_clamp;
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
            out.left_q_target_deg = left_prev_sent_q_deg_;
            out.right_q_target_deg = right_prev_sent_q_deg_;
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
                                 out.left_q_target_deg, left_prev_sent_q_deg_)) {
                build_floor_arm(ArmId::Right, right_mode, right_state, right_eval,
                                out.right_q_target_deg, right_prev_sent_q_deg_);
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
                               out.left_q_target_deg, left_prev_sent_q_deg_)) {
                build_roi_arm(ArmId::Right, right_mode, right_state, right_eval,
                              out.right_q_target_deg, right_prev_sent_q_deg_);
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
                                 out.left_q_target_deg, left_prev_sent_q_deg_)) {
                build_reach_arm(ArmId::Right, right_mode, right_state, right_eval,
                                out.right_q_target_deg, right_prev_sent_q_deg_);
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
                                      out.left_q_target_deg, left_prev_sent_q_deg_)) {
                build_user_floor_arm(ArmId::Right, right_mode, right_state, right_eval,
                                     out.right_q_target_deg, right_prev_sent_q_deg_);
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
            collision_monitor_->submitTargets(out.left_q_target_deg, out.right_q_target_deg);
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
                    out.left_q_target_deg = left_prev_sent_q_deg_;
                    out.right_q_target_deg = right_prev_sent_q_deg_;
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
            safety_cons, left_prev_sent_q_deg_, right_prev_sent_q_deg_,
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

void DualArmServoLoop::resyncArmAfterFreedrive(ArmId arm_id, const RobotState& state) {
    // Clear path state outside the state lock (mirroring clearFaultLatch's ordering).
    clearLatchedCartesianTarget(arm_id);

    // Free-drive changes the measured configuration without advancing any of the
    // server-owned Cartesian controllers. Re-anchoring only the joint hold would
    // therefore leave Cartesian admittance pulling toward its pre-teaching
    // equilibrium as soon as servo_j resumes. Drop every proposal computed before
    // teach_off, reset the controller dynamics, and seed the fixed Hold equilibrium
    // from the freshly measured TCP.
    ForceArmRuntime& force_runtime = arm_id == ArmId::Left
        ? left_force_runtime_ : right_force_runtime_;
    NormalForceController& normal_controller = arm_id == ArmId::Left
        ? left_normal_force_controller_ : right_normal_force_controller_;
    ForceController& cartesian_controller = arm_id == ArmId::Left
        ? left_cartesian_force_controller_ : right_cartesian_force_controller_;

    force_runtime.pending_proposal.reset();
    force_runtime.pending_proposal_applied = false;
    force_runtime.pending_cartesian_proposal.reset();
    force_runtime.pending_cartesian_proposal_applied = false;
    force_runtime.pending_rolling_compliance_target_valid = false;
    force_runtime.pending_rolling_compliance_target_source = "unavailable";
    force_runtime.compliance_hold_target_this_tick = false;
    force_runtime.normal_contact_active = false;
    force_runtime.contact_force_normal_estimator.reset();
    force_runtime.contact_cartesian_normal_offset_m = 0.0;
    force_runtime.follower_contact_normal_owned = false;
    force_runtime.transverse_contact_active = false;
    force_runtime.rotational_contact_active = false;
    force_runtime.contact_anchor_valid = false;
    force_runtime.enter_count = 0;
    force_runtime.transverse_enter_count = 0;
    force_runtime.rotational_enter_count = 0;
    force_runtime.release_start_ns = 0;
    force_runtime.release_hold_pending = false;
    force_runtime.release_hold_applied = false;
    force_runtime.release_hold_clear_requested = false;
    force_runtime.previous_actual_pose_ns = 0;
    force_runtime.sent_tcp_sample_count = 0;

    force_runtime.control.contact_active = false;
    force_runtime.control.normal_contact_active = false;
    force_runtime.control.transverse_contact_active = false;
    force_runtime.control.rotational_contact_active = false;
    force_runtime.control.compliance_active = false;
    force_runtime.control.normal_regulating = false;
    force_runtime.control.transverse_regulating = false;
    force_runtime.control.rotational_regulating = false;
    force_runtime.control.loading_projection_active = false;
    force_runtime.control.compliance_recenter_active = false;
    force_runtime.control.compliance_translation_recenter_coupled = false;
    force_runtime.control.compliance_rotation_recenter_coupled = false;
    force_runtime.control.compliance_translation_recenter_deferred = false;
    force_runtime.control.compliance_rotation_recenter_deferred = false;
    force_runtime.control.compliance_offset_surface = {};
    force_runtime.control.compliance_velocity_surface = {};
    force_runtime.control.compliance_acceleration_surface = {};
    force_runtime.control.raw_policy_delta_surface = {};
    force_runtime.control.accepted_policy_delta_surface = {};
    force_runtime.control.compliance_limit_axes = {};
    force_runtime.control.compliance_limit_reason.clear();
    force_runtime.control.correction_m = 0.0;
    force_runtime.control.velocity_m_s = 0.0;
    force_runtime.control.acceleration_m_s2 = 0.0;
    force_runtime.control.energy_j = 0.0;
    force_runtime.control.saturated = false;
    force_runtime.control.proposal_valid = false;
    force_runtime.control.proposal_committed = false;
    normal_controller.release();
    cartesian_controller.release();

    const bool tcp_reanchored =
        state.tcp_actual_valid && state.tcp_actual_stand.has_value();
    if (tcp_reanchored) {
        force_runtime.previous_raw_compliance_target = *state.tcp_actual_stand;
        force_runtime.rolling_compliance_target = *state.tcp_actual_stand;
        force_runtime.rolling_compliance_target_valid = true;
        force_runtime.rolling_compliance_target_source = "hold_anchor";
        force_runtime.control.compliance_equilibrium_stand = *state.tcp_actual_stand;
        force_runtime.control.compliance_equilibrium_source = "hold_anchor";
    } else {
        force_runtime.rolling_compliance_target_valid = false;
        force_runtime.rolling_compliance_target_source = "unavailable";
        force_runtime.control.compliance_equilibrium_source = "unavailable";
    }

    // The servo stream was globally suppressed while teaching. A policy/follower
    // residual assembled before or during that interval must not execute from the
    // new physical pose. Publish a new epoch and require a fresh source anchor.
    ++motion_epoch_;
    left_force_runtime_.control.motion_epoch = motion_epoch_;
    right_force_runtime_.control.motion_epoch = motion_epoch_;
    left_delta_twist_follower_.deactivate();
    right_delta_twist_follower_.deactivate();
    left_chunk_follower_.deactivate();
    right_chunk_follower_.deactivate();
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
              << " held target snapped to current actual joints; Cartesian force "
              << (tcp_reanchored ? "equilibrium reanchored to actual TCP"
                                 : "equilibrium unavailable (actual TCP invalid)")
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

    if (is_init) {
        if (left_init) invalidatePostInitTare(ArmId::Left, command.seq);
        if (right_init) invalidatePostInitTare(ArmId::Right, command.seq);
    }

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

    const auto process_exec = [&](InitMotionExec& ex, PlannerRequester requester,
                                  bool request_left, bool request_right, bool fresh_request) {
        // Launch a new one-shot request from the MEASURED joint pose
        // when it is available. For single-arm init, freeze the non-selected arm in the
        // planner at its measured pose; do not use last-sent references because SMD/flow
        // can intentionally lead the physical robot by a small amount.
        if (fresh_request && (request_left || request_right)) {
            ex.request_seen = true;
            ex.request_seq = command.seq;
            const JointArray target_left =
                request_left ? command.left.q_target_deg : current_left_q;
            const JointArray target_right =
                request_right ? command.right.q_target_deg : current_right_q;
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
                const uint64_t tare_start_ns = nowSteadyNs();
                if (request_left) beginPostInitTare(ArmId::Left, tare_start_ns);
                if (request_right) beginPostInitTare(ArmId::Right, tare_start_ns);
                hold_selected(command, ex);
                return;
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
                    const uint64_t tare_start_ns = nowSteadyNs();
                    if (ex.left_active) beginPostInitTare(ArmId::Left, tare_start_ns);
                    if (ex.right_active) beginPostInitTare(ArmId::Right, tare_start_ns);
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
    const bool left_exec_fresh = left_exec_requested &&
        (!left_init_motion_exec_.request_seen ||
         left_init_motion_exec_.request_seq != command.seq);
    const bool right_exec_fresh = right_exec_requested &&
        (!right_init_motion_exec_.request_seen ||
         right_init_motion_exec_.request_seq != command.seq);
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
