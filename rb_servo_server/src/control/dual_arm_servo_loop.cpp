#include "rb_servo/control/dual_arm_servo_loop.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <unordered_map>
#include <vector>

#include "rb_servo/control/cartesian_controller.hpp"
#include "rb_servo/control/fault_classifier.hpp"
#include "rb_servo/control/servo_dispatcher.hpp"
#include "rb_servo/core/clock.hpp"
#include "rb_servo/core/realtime.hpp"
#include "rb_servo/kinematics/pinocchio_kinematics.hpp"
#include "rb_servo/network/scope_publisher.hpp"

namespace rb_servo {
namespace {
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
        isStreamingCartesianMode(command.mode) &&
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
        isStreamingCartesianMode(command.mode) &&
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
    SelfCollisionResult sc;
    sc.checked = v.valid;
    sc.min_clearance_m = v.min_clearance_m;
    sc.violated = v.valid && v.min_clearance_m < cfg.d_hard_m;
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
            config_.right_mount);
    }
    resetReferenceSupervisionState(this);
}

DualArmServoLoop::~DualArmServoLoop() {
    stop();
    eraseReferenceSupervisionState(this);
}

bool DualArmServoLoop::start() {
    if (running_) return true;
    if (!initializeRobots()) {
        return false;
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

    while (running_) {
        next_tick += period;
        const uint64_t loop_start = nowSteadyNs();
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

        DualArmCommand command = command_buffer_
            ? command_buffer_->latestOrHold(loop_start)
            : makeHoldCommand(left_state, right_state, loop_start);
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
        if (isSyntheticHoldCommand(command)) {
            command = metadata_hold(command);
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
            init_non_init_arm_preserved_mode.clear();
            if (left_init_before_sequencer != right_init_before_sequencer &&
                !config_.safety.init_motion_planner.single_arm_freeze_other_arm) {
                init_non_init_arm_preserved_mode =
                    left_init_before_sequencer ? toString(command.right.mode) : toString(command.left.mode);
            }
            const bool motion_requested = commandRequestsMotion(command);
            const ServoTarget desired =
                computeServoTarget(left_state, right_state, command, filter_dt_sec, &command_verdict);

            if (command_verdict != SafetyVerdict::Ok) {
                // Missing payload, unsupported Cartesian/IK, or other command generation failure.
                // Do not synthesize a new target or report Running for a held/rejected command.
                safe_target.left_q_target_deg = left_prev_sent_q_deg_;
                safe_target.right_q_target_deg = right_prev_sent_q_deg_;
                safety_verdict = command_verdict;
                if (motion_requested) {
                    setMotionState(ServerMotionState::ArmedHold);
                }
            } else {
                safe_target = applySafety(
                    desired,
                    left_state,
                    right_state,
                    command.left.mode,
                    command.right.mode,
                    filter_dt_sec,
                    &safety_verdict);
                if (motion_requested) {
                    if (safety_verdict == SafetyVerdict::Ok ||
                        safety_verdict == SafetyVerdict::JointLimitClamped) {
                        setMotionState(ServerMotionState::Running);
                    } else if (!fault_latched_.load()) {
                        setMotionState(ServerMotionState::ArmedHold);
                    }
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
        const bool fault_latched_before_send = fault_latched_.load();
        const std::string send_policy = currentSendPolicy();
        const bool send_suppressed = send_policy != "send_servo_j";
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
        DualSendResult dual_send_result = sendTargets(
            attempted_target,
            command.seq,
            command_host_time_ns,
            send_policy,
            loop_start,
            send_deadline_ns
        );
        const SendServoJResult& left_send_result = dual_send_result.left.result;
        const SendServoJResult& right_send_result = dual_send_result.right.result;
        const uint64_t left_send_start_ns = dual_send_result.left.dispatch_timing.start_ns;
        const uint64_t left_send_end_ns = dual_send_result.left.dispatch_timing.end_ns;
        const uint64_t right_send_start_ns = dual_send_result.right.dispatch_timing.start_ns;
        const uint64_t right_send_end_ns = dual_send_result.right.dispatch_timing.end_ns;
        const bool left_ok = left_send_result.accepted;
        const bool right_ok = right_send_result.accepted;
        if (left_send_result.state_after.has_value()) {
            left_state = *left_send_result.state_after;
            populateTcpPose(left_state, config_.left_mount);
        }
        if (right_send_result.state_after.has_value()) {
            right_state = *right_send_result.state_after;
            populateTcpPose(right_state, config_.right_mount);
        }
        if (left_ok && !fault_latched_before_send && !send_suppressed) {
            noteReferenceSupervisionSentTarget(
                this,
                config_,
                kinematics_,
                kinematics_injected_,
                ArmId::Left,
                attempted_target.left_q_target_deg
            );
        }
        if (right_ok && !fault_latched_before_send && !send_suppressed) {
            noteReferenceSupervisionSentTarget(
                this,
                config_,
                kinematics_,
                kinematics_injected_,
                ArmId::Right,
                attempted_target.right_q_target_deg
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

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (left_ok && !fault_latched_before_send && !send_suppressed) {
                left_prevprev_sent_q_deg_ = left_prev_sent_q_deg_;
                left_prev_sent_q_deg_ = attempted_target.left_q_target_deg;
            }
            if (right_ok && !fault_latched_before_send && !send_suppressed) {
                right_prevprev_sent_q_deg_ = right_prev_sent_q_deg_;
                right_prev_sent_q_deg_ = attempted_target.right_q_target_deg;
            }
        }

        ServoSample sample;
        sample.tick = tick_++;
        sample.loop_start_time_ns = loop_start;
        sample.loop_end_time_ns = loop_end;
        sample.left_state = left_state;
        sample.right_state = right_state;
        sample.command = command;
        sample.left_mode_before_init_sequencer = init_mode_before_left;
        sample.right_mode_before_init_sequencer = init_mode_before_right;
        sample.left_mode_after_init_sequencer = init_mode_after_left;
        sample.right_mode_after_init_sequencer = init_mode_after_right;
        sample.non_init_arm_preserved_mode = init_non_init_arm_preserved_mode;
        sample.single_arm_freeze_other_arm =
            config_.safety.init_motion_planner.single_arm_freeze_other_arm;
        sample.left_sent_q_deg = attempted_target.left_q_target_deg;
        sample.right_sent_q_deg = attempted_target.right_q_target_deg;
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
            latest_snapshot_.tick = sample.tick;
            latest_snapshot_.loop_start_time_ns = loop_start;
            latest_snapshot_.loop_end_time_ns = loop_end;
            latest_snapshot_.left_state = left_state;
            latest_snapshot_.right_state = right_state;
            latest_snapshot_.command = command;
            latest_snapshot_.left_sent_q_deg = attempted_target.left_q_target_deg;
            latest_snapshot_.right_sent_q_deg = attempted_target.right_q_target_deg;
            latest_snapshot_.left_prev_sent_q_deg = left_prev_sent_q_deg_;
            latest_snapshot_.right_prev_sent_q_deg = right_prev_sent_q_deg_;
            latest_snapshot_.period_ms = sample.period_ms;
            latest_snapshot_.jitter_ms = sample.jitter_ms;
            latest_snapshot_.filter_dt_ms = sample.filter_dt_ms;
            latest_snapshot_.safety_verdict = safety_verdict;
            latest_snapshot_.self_collision_enabled = config_.safety.self_collision.enable;
            latest_snapshot_.self_collision_checked = last_self_collision_.checked;
            latest_snapshot_.self_collision_violated = last_self_collision_.violated;
            latest_snapshot_.self_collision_min_clearance_m = last_self_collision_.min_clearance_m;
            // Telemetry "margin" is the hard floor the mesh barrier defends.
            latest_snapshot_.self_collision_margin_m = config_.safety.self_collision.mesh.d_hard_m;
            latest_snapshot_.self_collision_left_bone = last_self_collision_.left_bone;
            latest_snapshot_.self_collision_right_bone = last_self_collision_.right_bone;
            latest_snapshot_.self_collision_pair = last_self_collision_.pair;
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
                        p.d_m, p.external});
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
                    arm.goal_nearest_pair_external = ex.goal_nearest_pair_external;
                    arm.goal_clear_threshold_self_m = ex.goal_clear_threshold_self_m;
                    arm.goal_clear_threshold_external_m = ex.goal_clear_threshold_external_m;
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
                            (left_arm ? left_prev_sent_q_deg_[i] : right_prev_sent_q_deg_[i]) -
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
                    diag.goal_nearest_pair_external = aggregate_owner->goal_nearest_pair_external;
                    diag.goal_clear_threshold_self_m = aggregate_owner->goal_clear_threshold_self_m;
                    diag.goal_clear_threshold_external_m = aggregate_owner->goal_clear_threshold_external_m;
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
        std::this_thread::sleep_until(next_tick);
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

ServoTarget DualArmServoLoop::computeServoTarget(
    const RobotState& left_state,
    const RobotState& right_state,
    const DualArmCommand& command,
    double dt_sec,
    SafetyVerdict* command_verdict
) {
    if (command_verdict) *command_verdict = SafetyVerdict::Ok;
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
        const ArmCommand left_pose_track_command = applyPoseTrackSmd(
            effective_command.left,
            left_tcp_profile.pose_track_smd,
            &left_pose_track_smd_,
            kinematics_,
            config_.left_mount,
            left_prev_sent_q_deg_,
            dt_sec
        );
        const ArmCommand right_pose_track_command = applyPoseTrackSmd(
            effective_command.right,
            right_tcp_profile.pose_track_smd,
            &right_pose_track_smd_,
            kinematics_,
            config_.right_mount,
            right_prev_sent_q_deg_,
            dt_sec
        );

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

        const CartesianArmTargetResult left_cartesian_result =
            effective_command.left.mode == ControlMode::TcpLinearMove
            ? cartesian_servo.computeLinearMoveTarget(
                effective_command.left,
                left_servo_state.state,
                left_prev_sent_q_deg_,
                left_cartesian_compute_run_mode,
                dt_sec,
                continue_left_linear ? 0 : command.seq,
                &left_cartesian_servo_path_,
                &left_servo_state.context
            )
            : isCartesianMode(effective_command.left.mode)
            ? cartesian.computeArmJointTarget(
                left_pose_track_command,
                left_state,
                left_prev_sent_q_deg_,
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
        if (command.left.mode == ControlMode::TcpLinearMove && left_cartesian_servo_path_.active) {
            left_cartesian_servo_path_.lease_enforced = command.lease.enforce_lease;
            left_cartesian_servo_path_.lease_expires_time_ns = command.lease.expires_time_ns;
        }

        const CartesianArmTargetResult right_cartesian_result =
            effective_command.right.mode == ControlMode::TcpLinearMove
            ? cartesian_servo.computeLinearMoveTarget(
                effective_command.right,
                right_servo_state.state,
                right_prev_sent_q_deg_,
                right_cartesian_compute_run_mode,
                dt_sec,
                continue_right_linear ? 0 : command.seq,
                &right_cartesian_servo_path_,
                &right_servo_state.context
            )
            : isCartesianMode(effective_command.right.mode)
            ? cartesian.computeArmJointTarget(
                right_pose_track_command,
                right_state,
                right_prev_sent_q_deg_,
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
            if (command_verdict) *command_verdict = verdict;
            if (left_cartesian_result.verdict != SafetyVerdict::Ok) {
                target.left_q_target_deg = left_prev_sent_q_deg_;
            }
            if (right_cartesian_result.verdict != SafetyVerdict::Ok) {
                target.right_q_target_deg = right_prev_sent_q_deg_;
            }
        }
        return target;
    }

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
                        latchFault(SafetyVerdict::FloorViolation, reason, left_state, right_state);
                        out = currentFaultHoldTarget();
                        combined = SafetyVerdict::FaultLatched;
                        return true;
                    }
                    target_q = prev_sent_q;  // hold this arm
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
                        }
                    } else {
                        // Jacobian unavailable -> fail closed (revert this arm).
                        target_q = prev_sent_q;
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
                    if (combined == SafetyVerdict::Ok || combined == SafetyVerdict::JointLimitClamped) {
                        combined = SafetyVerdict::RoiViolation;
                    }
                };
                if (!eval.checked) {
                    // TCP FK unavailable -> fail closed (cannot build a Jacobian).
                    const std::string reason = "roi box: " + arm_name + " TCP FK unavailable";
                    if (rb.fail_policy == FloorConstraintFailPolicy::FaultLatch) {
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
                    if (combined == SafetyVerdict::Ok || combined == SafetyVerdict::JointLimitClamped) {
                        combined = SafetyVerdict::RoiViolation;
                    }
                };
                if (!eval.checked) {
                    // TCP FK unavailable -> fail closed (cannot build a Jacobian).
                    const std::string reason = "reach shell: " + arm_name + " TCP FK unavailable";
                    if (rc.fail_policy == FloorConstraintFailPolicy::FaultLatch) {
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
                    if (combined == SafetyVerdict::Ok || combined == SafetyVerdict::JointLimitClamped) {
                        combined = SafetyVerdict::FloorViolation;
                    }
                };
                if (!eval.checked) {
                    // TCP FK unavailable -> fail closed (cannot build a Jacobian).
                    const std::string reason = "user floor: " + arm_name + " TCP FK unavailable";
                    if (uf.fail_policy == FloorConstraintFailPolicy::FaultLatch) {
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
                    latchFault(SafetyVerdict::SelfCollision, reason, left_state, right_state);
                    out = currentFaultHoldTarget();
                    combined = SafetyVerdict::FaultLatched;
                } else if (stale) {
                    // Fail-closed: no fresh verdict -> hold at the previous sent pose
                    // (qdot = 0) and reanchor so nothing winds up while we wait. Auto-
                    // recovers once fresh verdicts resume (never latches on staleness).
                    out.left_q_target_deg = left_prev_sent_q_deg_;
                    out.right_q_target_deg = right_prev_sent_q_deg_;
                    self_collision_hold = true;  // skip the combined solve (qdot already 0)
                    if (combined == SafetyVerdict::Ok ||
                        combined == SafetyVerdict::JointLimitClamped) {
                        combined = SafetyVerdict::SelfCollision;
                    }
                } else {
                    // Collect self-collision velocity constraints (per near pair within
                    // d_slow, age-extrapolated) into the shared list; the combined solve
                    // below removes only the closing component of the command.
                    buildCollisionConstraints(v, collision_monitor_cfg_, now_s - v.stamp_s,
                                              safety_cons);
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
    cmd.left.q_target_deg = chooseSafeHoldTarget(left_state, left_prev_sent_q_deg_);
    cmd.right.q_target_deg = chooseSafeHoldTarget(right_state, right_prev_sent_q_deg_);
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
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (arm_id == ArmId::Left) {
        const JointArray q = chooseSafeHoldTarget(state, left_prev_sent_q_deg_);
        left_prev_sent_q_deg_ = q;
        left_prevprev_sent_q_deg_ = q;
        left_fault_hold_q_deg_ = q;
        left_controller_sim_physical_baseline_q_deg_ = state.q_actual_deg;
        left_output_ma_.reset();
    } else {
        const JointArray q = chooseSafeHoldTarget(state, right_prev_sent_q_deg_);
        right_prev_sent_q_deg_ = q;
        right_prevprev_sent_q_deg_ = q;
        right_fault_hold_q_deg_ = q;
        right_controller_sim_physical_baseline_q_deg_ = state.q_actual_deg;
        right_output_ma_.reset();
    }
    std::cerr << "[INFO] freedrive exit resync: " << toString(arm_id)
              << " held target snapped to current actual joints\n";
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
    (void)left_state;
    (void)right_state;
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
    const auto hold_selected = [&](DualArmCommand& c, const InitMotionExec& ex) {
        if (ex.left_active || freeze_other_arm) {
            c.left.mode = ControlMode::Hold;
            c.left.joint_target_profile = JointTargetProfile::Direct;
        }
        if (ex.right_active || freeze_other_arm) {
            c.right.mode = ControlMode::Hold;
            c.right.joint_target_profile = JointTargetProfile::Direct;
        }
    };
    const auto reached = [](const JointArray& a, const JointArray& b, double tol) {
        for (int i = 0; i < kDof; ++i) {
            if (std::abs(a[i] - b[i]) > tol) return false;
        }
        return true;
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
            // the next init_motion profile plans fresh. A discarded in-flight future detaches.
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
        if (left_init && right_init) {
            if (non_idle(right_init_motion_exec_)) right_init_motion_exec_ = InitMotionExec{};
        } else if (left_init) {
            if (left_init_motion_exec_.right_active) left_init_motion_exec_ = InitMotionExec{};
            if (non_idle(right_init_motion_exec_)) right_init_motion_exec_ = InitMotionExec{};
        } else if (right_init) {
            if (right_init_motion_exec_.left_active) right_init_motion_exec_ = InitMotionExec{};
            if (non_idle(left_init_motion_exec_)) left_init_motion_exec_ = InitMotionExec{};
        }
    }

    // Planner disabled -> fall back to a direct JointTarget to the requested pose;
    // the reactive barrier + floor still guard each tick.
    if (is_init && !init_motion_planner_) {
        InitMotionExec direct;
        direct.left_active = left_init;
        direct.right_active = right_init;
        rewrite_selected(command, direct,
                         left_init ? command.left.q_target_deg : left_prev_sent_q_deg_,
                         right_init ? command.right.q_target_deg : right_prev_sent_q_deg_);
        return command;
    }

    const auto process_exec = [&](InitMotionExec& ex, bool request_left, bool request_right,
                                  bool fresh_profile) {
        // Launch (or relaunch on a changed target) a plan from the CURRENT sent pose.
        // For single-arm init the non-selected arm is frozen only inside the planner's
        // combined collision oracle; it is not a target identity and must not trigger
        // replans as flow moves it on later ticks.
        if (fresh_profile && (request_left || request_right)) {
            const JointArray target_left =
                request_left ? command.left.q_target_deg : left_prev_sent_q_deg_;
            const JointArray target_right =
                request_right ? command.right.q_target_deg : right_prev_sent_q_deg_;
            const bool new_target = !ex.has_target ||
                (request_left && !reached(target_left, ex.target_left, 1e-6)) ||
                (request_right && !reached(target_right, ex.target_right, 1e-6)) ||
                ex.left_active != request_left ||
                ex.right_active != request_right;
            const auto launch_plan = [&]() {
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
                ex.goal_nearest_pair_external = false;
                ex.goal_clear_threshold_self_m = std::numeric_limits<double>::quiet_NaN();
                ex.goal_clear_threshold_external_m = std::numeric_limits<double>::quiet_NaN();
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
                const JointArray start_left = left_prev_sent_q_deg_;
                const JointArray start_right = right_prev_sent_q_deg_;
                InitMotionPlanner* planner = init_motion_planner_.get();
                ex.future = std::async(std::launch::async,
                    [planner, start_left, start_right, request_left, target_left,
                     request_right, target_right]() {
                        return planner->plan(
                            start_left, start_right, request_left, target_left,
                            request_right, target_right);
                    });
                std::cerr << "[INFO] JointTarget init_motion: planning collision-free path from current pose\n";
            };
            if (ex.status == InitMotionStatus::Idle ||
                ((ex.status == InitMotionStatus::Executing ||
                  ex.status == InitMotionStatus::Done ||
                  ex.status == InitMotionStatus::Failed) && new_target)) {
                launch_plan();
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

        // Poll the async plan (non-blocking) and transition Planning -> Executing/Failed.
        if (ex.status == InitMotionStatus::Planning && ex.future.valid() &&
            ex.future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            InitMotionPlanResult result = ex.future.get();
            ex.fail_mode = result.fail_mode;
            ex.start_clear_m = result.start_clear_m;
            ex.goal_clear_m = result.goal_clear_m;
            ex.goal_self_min_clearance_m = result.goal_self_min_clearance_m;
            ex.goal_external_min_clearance_m = result.goal_external_min_clearance_m;
            ex.goal_nearest_pair_name_a = result.goal_nearest_pair_name_a;
            ex.goal_nearest_pair_name_b = result.goal_nearest_pair_name_b;
            ex.goal_nearest_pair_external = result.goal_nearest_pair_external;
            ex.goal_clear_threshold_self_m = result.goal_clear_threshold_self_m;
            ex.goal_clear_threshold_external_m = result.goal_clear_threshold_external_m;
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
                bool done = false;
                // Follow the gradient-escape head precisely (escape_waypoints), then pure-pursuit.
                JointArray pursue_left = left_prev_sent_q_deg_;
                JointArray pursue_right = right_prev_sent_q_deg_;
                if (!ex.left_active && !freeze_other_arm) {
                    pursue_left = ex.target_left;
                }
                if (!ex.right_active && !freeze_other_arm) {
                    pursue_right = ex.target_right;
                }
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
                done = pursuit.done;
                if (!ex.waypoints.empty()) {
                    // Progress = max-joint distance from the current sent pose to the final
                    // waypoint. Closing on it (by > a noise floor) refreshes the stall timer.
                    const auto& goal_wp = ex.waypoints.back();
                    double dist = 0.0;
                    for (int i = 0; i < kDof; ++i) {
                        if (ex.left_active || freeze_other_arm) {
                            dist = std::max(dist, std::abs(left_prev_sent_q_deg_[i] - goal_wp.first[i]));
                        }
                        if (ex.right_active || freeze_other_arm) {
                            dist = std::max(dist, std::abs(right_prev_sent_q_deg_[i] - goal_wp.second[i]));
                        }
                    }
                    const uint64_t now_ns = nowSteadyNs();
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
                                  << ", dist_to_goal=" << dist << " deg, best=" << ex.best_dist_deg
                                  << " deg)\n";
                    }
                }
                if (done) {
                    ex.status = InitMotionStatus::Done;
                    ex.message = "done";
                    std::cerr << "[INFO] JointTarget init_motion: reached init pose\n";
                }
                rewrite_selected(command, ex, wp.first, wp.second);
                break;
            }
            case InitMotionStatus::Done:
                // Hold at the (planned) goal so the arm resists drift until the command
                // expires or the operator issues something else.
                rewrite_selected(command, ex, ex.target_left, ex.target_right);
                break;
            case InitMotionStatus::Planning:
            case InitMotionStatus::Failed:
            default:
                // Hold in place: while planning, or fail-closed after a planning failure.
                hold_selected(command, ex);
                break;
        }
    };

    if (is_init) {
        if (left_init && right_init) {
            process_exec(left_init_motion_exec_, true, true, true);
        } else {
            if (left_init) process_exec(left_init_motion_exec_, true, false, true);
            if (right_init) process_exec(right_init_motion_exec_, false, true, true);
        }
    } else {
        if (sequence_active(left_init_motion_exec_)) {
            process_exec(left_init_motion_exec_, left_init_motion_exec_.left_active,
                         left_init_motion_exec_.right_active, false);
        }
        if (sequence_active(right_init_motion_exec_)) {
            process_exec(right_init_motion_exec_, right_init_motion_exec_.left_active,
                         right_init_motion_exec_.right_active, false);
        }
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

    // Progress pointer: monotonic projection advance. Advances past every segment whose
    // far endpoint the pose has projected beyond. Corner-cutting can never freeze it,
    // because progress is measured along the path direction, not by proximity to a node.
    while (index + 1 < n &&
           segFraction(waypoints[index], waypoints[index + 1]) >= 1.0) {
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
            InitMotionPlanner* planner = init_motion_planner_.get();
            lx.future = std::async(std::launch::async,
                [planner, start_left, start_right, left_active, goal_left,
                 right_active, goal_right, slerp, samples]() {
                    return planner->planLinearMove(start_left, start_right,
                                                   left_active, goal_left,
                                                   right_active, goal_right, slerp, samples);
                });
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

    // Poll the async decision.
    if (lx.status == LinearMoveStatus::Deciding && lx.future.valid() &&
        lx.future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        InitMotionLinearResult r = lx.future.get();
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
    const ServerMotionState state = motion_state_.load();
    return state == ServerMotionState::ArmedHold || state == ServerMotionState::Running;
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
    left_prev_sent_q_deg_ = chooseSafeHoldTarget(left_state, left_prev_sent_q_deg_);
    right_prev_sent_q_deg_ = chooseSafeHoldTarget(right_state, right_prev_sent_q_deg_);
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
    left_fault_hold_q_deg_ = chooseSafeHoldTarget(left_state, left_prev_sent_q_deg_);
    right_fault_hold_q_deg_ = chooseSafeHoldTarget(right_state, right_prev_sent_q_deg_);
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
    const RobotState& state,
    const JointArray& previous_sent
) const {
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
