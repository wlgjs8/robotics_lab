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

namespace rb_servo {
namespace {
bool isCartesianMode(ControlMode mode) {
    return mode == ControlMode::TcpPoseTarget ||
           mode == ControlMode::TcpLinearMove ||
           mode == ControlMode::TcpCircleMove ||
           mode == ControlMode::TcpCircleTrack ||
           mode == ControlMode::TcpDeltaStand ||
           mode == ControlMode::TcpDeltaLocal ||
           mode == ControlMode::TcpTwistStand ||
           mode == ControlMode::TcpTwistLocal;
}

bool isCartesianDeltaMode(ControlMode mode) {
    return mode == ControlMode::TcpDeltaStand ||
           mode == ControlMode::TcpDeltaLocal;
}

bool isCartesianVelocityServoMode(ControlMode mode) {
    return mode == ControlMode::TcpLinearMove ||
           mode == ControlMode::TcpCircleMove ||
           mode == ControlMode::TcpCircleTrack ||
           mode == ControlMode::TcpTwistStand ||
           mode == ControlMode::TcpTwistLocal;
}

bool isStreamingCartesianMode(ControlMode mode) {
    return isCartesianVelocityServoMode(mode);
}

bool isMotionMode(ControlMode mode) {
    return mode == ControlMode::JointTarget ||
           mode == ControlMode::JointVelocity ||
           mode == ControlMode::TcpPoseTarget ||
           mode == ControlMode::TcpLinearMove ||
           mode == ControlMode::TcpCircleMove ||
           mode == ControlMode::TcpCircleTrack ||
           mode == ControlMode::TcpDeltaStand ||
           mode == ControlMode::TcpDeltaLocal ||
           mode == ControlMode::TcpTwistStand ||
           mode == ControlMode::TcpTwistLocal;
}

std::string velocityIntegrationModeName(CartesianVelocityTargetIntegrationMode mode) {
    switch (mode) {
        case CartesianVelocityTargetIntegrationMode::MeasuredActual:
            return "measured_actual";
        case CartesianVelocityTargetIntegrationMode::MeasuredActualLookahead:
            return "measured_actual_lookahead";
        case CartesianVelocityTargetIntegrationMode::PreviousCommand:
            return "previous_command";
    }
    return "unknown";
}

bool jointArraysDiffer(const JointArray& a, const JointArray& b, double tolerance = 1e-9) {
    for (int i = 0; i < kDof; ++i) {
        if (std::abs(a[i] - b[i]) > tolerance) return true;
    }
    return false;
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

bool linearPathLeaseExpired(const CartesianServoPathState& path, uint64_t now_ns) {
    return path.active &&
           path.lease_enforced &&
           path.lease_expires_time_ns > 0 &&
           now_ns > path.lease_expires_time_ns;
}

bool circlePathLeaseExpired(const CartesianCircleMoveState& path, uint64_t now_ns) {
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
        case ControlMode::JointVelocity:
            return !command.has_joint_velocity;
        case ControlMode::TcpPoseTarget:
            return !command.has_tcp_target;
        case ControlMode::TcpLinearMove:
            return !command.has_tcp_target ||
                   (!command.has_linear_move_duration && !command.has_linear_move_linear_speed);
        case ControlMode::TcpCircleMove:
            return !command.has_tcp_circle_move;
        case ControlMode::TcpCircleTrack:
            return !command.has_tcp_circle_track;
        case ControlMode::TcpDeltaStand:
            return !command.has_tcp_delta_stand;
        case ControlMode::TcpDeltaLocal:
            return !command.has_tcp_delta_local;
        case ControlMode::TcpTwistStand:
            return !command.has_tcp_twist_stand;
        case ControlMode::TcpTwistLocal:
            return !command.has_tcp_twist_local;
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

bool envFlagEnabled(const char* name) {
    const char* value = std::getenv(name);
    return value && std::string(value) == "1";
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
    }
    tracker->updateGoalFromCommand(command.tcp_target_stand);
    ArmCommand smoothed = command;
    smoothed.tcp_target_stand = tracker->step(dt_sec);
    return smoothed;
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
        bothRbpodoControllerSimulationBackends(config) &&
        envFlagEnabled("RB_ALLOW_REAL_ROBOT") &&
        envFlagEnabled("RB_ALLOW_REAL_MOTION");
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
        isRbpodoControllerSimulationBackend(backend) &&
        envFlagEnabled("RB_ALLOW_REAL_ROBOT") &&
        envFlagEnabled("RB_ALLOW_REAL_MOTION");
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
    CartesianAvailability availability;
    const BackendConfig& backend = backendConfigForArm(config, command.arm_id);

    if (!config.cartesian_control.enable) {
        availability.reason = "cartesian_control_unavailable_disabled";
        return availability;
    }

    if (command.mode == ControlMode::TcpCircleTrack) {
        if (!config.cartesian_control.enable_server_side_circle_track) {
            availability.reason = "tcp_circle_track_disabled";
            availability.physical_motion_expected = false;
            return availability;
        }
        if (backend.run_mode == RunMode::Simulation) {
            availability.available = config.cartesian_control.allow_in_simulation;
            availability.reason = availability.available
                ? ""
                : "cartesian_control_unavailable_run_mode";
            availability.physical_motion_expected = false;
            return availability;
        }
        if (backend.run_mode != RunMode::Real) {
            availability.reason = "cartesian_control_unavailable_run_mode";
            return availability;
        }
        if (backend.backend_type != BackendType::Rbpodo) {
            availability.reason = "cartesian_control_unavailable_backend";
            return availability;
        }
        const std::string operation_mode = lowerAscii(backend.operation_mode);
        if (!(operation_mode == "simulation" || operation_mode == "sim")) {
            availability.reason = "tcp_circle_track_physical_real_blocked";
            return availability;
        }
        if (!config.cartesian_control.allow_in_controller_simulation ||
            !config.servo.allow_controller_simulation_motion) {
            availability.reason = "cartesian_control_unavailable_controller_sim_config";
            availability.physical_motion_expected = false;
            return availability;
        }
        if (!controllerSimulationCartesianGateOpen(config, backend)) {
            availability.reason = "cartesian_control_unavailable_controller_sim_env";
            availability.physical_motion_expected = false;
            return availability;
        }
        availability.available = true;
        availability.reason = "";
        availability.controller_simulation_cartesian_enabled = true;
        availability.physical_motion_expected = false;
        return availability;
    }

    const bool streaming_cartesian = isStreamingCartesianMode(command.mode);
    const bool controller_simulation_circle_move =
        command.mode == ControlMode::TcpCircleMove &&
        config.cartesian_control.enable_benchmark_primitives &&
        config.cartesian_control.circle_move.allow_in_simulation &&
        !config.cartesian_control.circle_move.allow_in_real;
    if (backend.run_mode == RunMode::Simulation) {
        availability.available = config.cartesian_control.allow_in_simulation;
        availability.reason = availability.available
            ? ""
            : "cartesian_control_unavailable_run_mode";
        availability.physical_motion_expected = false;
        return availability;
    }

    if (backend.run_mode != RunMode::Real) {
        availability.reason = "cartesian_control_unavailable_run_mode";
        return availability;
    }

    const bool rbpodo_controller_simulation_operation =
        isRbpodoControllerSimulationBackend(backend);
    const bool controller_simulation_cartesian_context =
        rbpodo_controller_simulation_operation &&
        controllerSimulationCartesianGateOpen(config, backend);

    if (!streaming_cartesian &&
        !controller_simulation_circle_move &&
        !controller_simulation_cartesian_context) {
        if (rbpodo_controller_simulation_operation) {
            availability.reason = "cartesian_control_unavailable_physical_real_blocked";
            return availability;
        }
        availability.available =
            config.cartesian_control.allow_in_real &&
            envFlagEnabled("RB_ALLOW_REAL_CARTESIAN");
        availability.reason = availability.available
            ? ""
            : "cartesian_control_unavailable_physical_real_blocked";
        return availability;
    }

    if (backend.backend_type != BackendType::Rbpodo) {
        availability.reason = "cartesian_control_unavailable_backend";
        return availability;
    }

    const std::string operation_mode = lowerAscii(backend.operation_mode);
    if (!(operation_mode == "simulation" || operation_mode == "sim")) {
        availability.reason =
            config.cartesian_control.allow_in_real && envFlagEnabled("RB_ALLOW_REAL_CARTESIAN")
            ? "cartesian_control_unavailable_physical_real_blocked"
            : "cartesian_control_unavailable_operation_mode";
        return availability;
    }

    if (!config.cartesian_control.allow_in_controller_simulation ||
        !config.servo.allow_controller_simulation_motion) {
        availability.reason = "cartesian_control_unavailable_controller_sim_config";
        availability.physical_motion_expected = false;
        return availability;
    }

    if (!controllerSimulationCartesianGateOpen(config, backend)) {
        availability.reason = "cartesian_control_unavailable_controller_sim_env";
        availability.physical_motion_expected = false;
        return availability;
    }

    availability.available = true;
    availability.reason = "";
    availability.controller_simulation_cartesian_enabled = true;
    availability.physical_motion_expected = false;
    return availability;
}

RunMode cartesianComputationRunModeForArm(
    const DualArmConfig& config,
    const ArmCommand& command
) {
    const RunMode run_mode = backendConfigForArm(config, command.arm_id).run_mode;
    if (run_mode == RunMode::Real &&
        (isStreamingCartesianMode(command.mode) || command.mode == ControlMode::TcpCircleMove) &&
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
        (isStreamingCartesianMode(command.mode) || command.mode == ControlMode::TcpCircleMove) &&
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

CartesianArmTargetResult rejectedCircleTrackTarget(
    const RobotState& state,
    const CartesianControlConfig& config,
    const JointArray& previous_safe_sent_q_deg
) {
    CartesianArmTargetResult result;
    result.q_target_deg = previous_safe_sent_q_deg;
    result.verdict = SafetyVerdict::CartesianUnavailable;
    result.reason = "tcp_circle_track_not_implemented";
    result.telemetry = cartesianUnavailableTelemetry(state, config, result.reason);
    return result;
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

ArmCommand circleMoveContinuationCommand(
    const ArmCommand& hold_command,
    const CartesianCircleMoveState& circle
) {
    ArmCommand command = hold_command;
    command.mode = ControlMode::TcpCircleMove;
    command.tcp_circle_move = circle.command;
    command.has_tcp_circle_move = true;
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

DualArmServoLoop::DualArmServoLoop(
    std::unique_ptr<IRobotBackend> left_robot,
    std::unique_ptr<IRobotBackend> right_robot,
    const DualArmConfig& config,
    CommandBuffer* command_buffer,
    ServoLogger* logger,
    std::shared_ptr<IKinematics> kinematics
) : left_robot_(std::move(left_robot)),
    right_robot_(std::move(right_robot)),
    config_(config),
    command_buffer_(command_buffer),
    logger_(logger),
    kinematics_(nullptr),
    kinematics_injected_(kinematics != nullptr),
    left_traj_filter_(config.servo, config.safety),
    right_traj_filter_(config.servo, config.safety),
    safety_filter_(config.safety) {
    left_pose_track_smd_ = SmdPoseTracker(config.cartesian_control.pose_track_smd);
    right_pose_track_smd_ = SmdPoseTracker(config.cartesian_control.pose_track_smd);
    left_output_ma_ = JointMovingAverage(config.servo.output_moving_average_window);
    right_output_ma_ = JointMovingAverage(config.servo.output_moving_average_window);
    kinematics_ = kinematics ? std::move(kinematics) : makeKinematicsProvider(config);
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
            return hold;
        };
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
        } else if (commandRequestsDisarmMotion(command)) {
            clearLatchedCartesianTargets();
            setMotionState(ServerMotionState::ConnectedHold);
            command = metadata_hold(command);
        } else if (commandRequestsArmMotion(command)) {
            if (!fault_latched_.load()) {
                setMotionState(ServerMotionState::ArmedHold);
            }
            command = metadata_hold(command);
        } else if (commandRequestsMotion(command) && !motionAllowed()) {
            clearLatchedCartesianTargets();
            command = metadata_hold(command);
        }

        ServoTarget safe_target;
        ServoTarget desired_target;
        bool have_desired_target = false;
        SafetyVerdict safety_verdict = SafetyVerdict::Ok;
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
        } else {
            SafetyVerdict command_verdict = SafetyVerdict::Ok;
            command = resolveCartesianDeltaCommand(command, left_state, right_state);
            const bool motion_requested = commandRequestsMotion(command);
            ServoTarget desired = computeServoTarget(left_state, right_state, command, filter_dt_sec, &command_verdict);
            desired_target = desired;
            have_desired_target = true;

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
                safe_target = applySafety(desired, left_state, right_state, filter_dt_sec, &safety_verdict);
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

        CartesianServoController cartesian_servo_post_safety(
            config_.left_mount,
            config_.right_mount,
            config_.cartesian_control,
            kinematics_
        );
        const bool safety_allows_integrator_update =
            safety_verdict == SafetyVerdict::Ok ||
            safety_verdict == SafetyVerdict::JointLimitClamped;
        const bool left_integrator_update_ok =
            left_ok && safety_allows_integrator_update &&
            !fault_latched_before_send && !send_suppressed && !fault_latched_.load();
        const bool right_integrator_update_ok =
            right_ok && safety_allows_integrator_update &&
            !fault_latched_before_send && !send_suppressed && !fault_latched_.load();
        const bool left_target_clamped = have_desired_target &&
            jointArraysDiffer(desired_target.left_q_target_deg, attempted_target.left_q_target_deg);
        const bool right_target_clamped = have_desired_target &&
            jointArraysDiffer(desired_target.right_q_target_deg, attempted_target.right_q_target_deg);
        cartesian_servo_post_safety.updateVelocityIntegratorAfterSafety(
            &left_cartesian_velocity_integrator_,
            attempted_target.left_q_target_deg,
            left_integrator_update_ok,
            left_target_clamped,
            left_ok ? "send_suppressed_or_fault" : "send_failed"
        );
        cartesian_servo_post_safety.updateVelocityIntegratorAfterSafety(
            &right_cartesian_velocity_integrator_,
            attempted_target.right_q_target_deg,
            right_integrator_update_ok,
            right_target_clamped,
            right_ok ? "send_suppressed_or_fault" : "send_failed"
        );
        if (left_cartesian_servo_path_.active && left_cartesian_servo_path_.done) {
            resetCartesianVelocityIntegrator(ArmId::Left, "linear_path_done");
        }
        if (right_cartesian_servo_path_.active && right_cartesian_servo_path_.done) {
            resetCartesianVelocityIntegrator(ArmId::Right, "linear_path_done");
        }
        if (left_cartesian_circle_move_.active && left_cartesian_circle_move_.done) {
            resetCartesianVelocityIntegrator(ArmId::Left, "circle_move_done");
        }
        if (right_cartesian_circle_move_.active && right_cartesian_circle_move_.done) {
            resetCartesianVelocityIntegrator(ArmId::Right, "circle_move_done");
        }
        refreshCartesianVelocityIntegratorTelemetry(ArmId::Left);
        refreshCartesianVelocityIntegratorTelemetry(ArmId::Right);

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
            latest_snapshot_.self_collision_margin_m = config_.safety.self_collision.margin_m;
            latest_snapshot_.self_collision_left_bone = last_self_collision_.left_bone;
            latest_snapshot_.self_collision_right_bone = last_self_collision_.right_bone;
            latest_snapshot_.motion_state = sample.motion_state;
            latest_snapshot_.fault_latched = sample.fault_latched;
            latest_snapshot_.async_supervision_degraded = sample.async_supervision_degraded;
            latest_snapshot_.tracking_error_degraded = sample.tracking_error_degraded;
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
                     "servo.allow_controller_simulation_motion=true, "
                     "RB_ALLOW_REAL_ROBOT=1, and RB_ALLOW_REAL_MOTION=1.\n";
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
    left_cartesian_circle_move_ = CartesianCircleMoveState{};
    right_cartesian_circle_move_ = CartesianCircleMoveState{};
    left_cartesian_twist_hold_ = CartesianTwistHoldState{};
    right_cartesian_twist_hold_ = CartesianTwistHoldState{};
    resetCartesianVelocityIntegrator(ArmId::Left, "cartesian_target_clear");
    resetCartesianVelocityIntegrator(ArmId::Right, "cartesian_target_clear");
    left_last_cartesian_solve_ = CartesianSolveTelemetry{};
    right_last_cartesian_solve_ = CartesianSolveTelemetry{};
}

void DualArmServoLoop::clearLatchedCartesianTarget(ArmId arm_id) {
    if (arm_id == ArmId::Left) {
        left_latched_cartesian_target_ = LatchedCartesianTarget{};
        left_cartesian_servo_path_ = CartesianServoPathState{};
        left_cartesian_circle_move_ = CartesianCircleMoveState{};
        left_cartesian_twist_hold_ = CartesianTwistHoldState{};
        resetCartesianVelocityIntegrator(ArmId::Left, "cartesian_target_clear");
        left_last_cartesian_solve_ = CartesianSolveTelemetry{};
    } else {
        right_latched_cartesian_target_ = LatchedCartesianTarget{};
        right_cartesian_servo_path_ = CartesianServoPathState{};
        right_cartesian_circle_move_ = CartesianCircleMoveState{};
        right_cartesian_twist_hold_ = CartesianTwistHoldState{};
        resetCartesianVelocityIntegrator(ArmId::Right, "cartesian_target_clear");
        right_last_cartesian_solve_ = CartesianSolveTelemetry{};
    }
}

void DualArmServoLoop::resetCartesianVelocityIntegrator(ArmId arm_id, const std::string& reason) {
    CartesianVelocityIntegratorState& state = arm_id == ArmId::Left
        ? left_cartesian_velocity_integrator_
        : right_cartesian_velocity_integrator_;
    if (state.valid) {
        state.valid = false;
        ++state.resets_total;
    }
    state.reset_reason = reason;
    refreshCartesianVelocityIntegratorTelemetry(arm_id);
}

void DualArmServoLoop::refreshCartesianVelocityIntegratorTelemetry(ArmId arm_id) {
    const CartesianVelocityIntegratorState& state = arm_id == ArmId::Left
        ? left_cartesian_velocity_integrator_
        : right_cartesian_velocity_integrator_;
    CartesianSolveTelemetry& telemetry = arm_id == ArmId::Left
        ? left_last_cartesian_solve_
        : right_last_cartesian_solve_;
    if (!telemetry.attempted) return;
    telemetry.cartesian_velocity_integration_mode =
        velocityIntegrationModeName(config_.cartesian_control.velocity_target_integration);
    telemetry.cartesian_servo_state_source = state.cartesian_servo_state_source;
    telemetry.cartesian_divergence_source = state.cartesian_divergence_source;
    telemetry.q_reference_for_servo_valid = state.q_reference_for_servo_valid;
    telemetry.q_integrator_valid = state.valid;
    telemetry.integrator_reset_reason = state.reset_reason;
    telemetry.integrator_resets_total = state.resets_total;
    telemetry.integrator_clamps_total = state.clamps_total;
    telemetry.integrator_divergence_total = state.divergence_total;
    telemetry.max_command_actual_error_deg_observed =
        state.max_command_actual_error_deg_observed;
    telemetry.command_reference_error_deg_observed =
        state.command_reference_error_deg_observed;
    telemetry.physical_command_actual_error_deg_observed =
        state.physical_command_actual_error_deg_observed;
    telemetry.velocity_target_lookahead_sec =
        config_.cartesian_control.velocity_target_lookahead_sec;
}

DualArmCommand DualArmServoLoop::resolveCartesianDeltaCommand(
    const DualArmCommand& command,
    const RobotState& left_state,
    const RobotState& right_state
) {
    DualArmCommand resolved = command;
    resolved.left = resolveArmCartesianDeltaCommand(command.left, left_state, command.seq, left_latched_cartesian_target_);
    resolved.right = resolveArmCartesianDeltaCommand(command.right, right_state, command.seq, right_latched_cartesian_target_);
    return resolved;
}

ArmCommand DualArmServoLoop::resolveArmCartesianDeltaCommand(
    const ArmCommand& command,
    const RobotState& state,
    uint64_t command_seq,
    LatchedCartesianTarget& latch
) {
    if (!isCartesianDeltaMode(command.mode)) {
        latch = LatchedCartesianTarget{};
        return command;
    }

    ArmCommand resolved = command;
    if (latch.valid && latch.seq == command_seq) {
        resolved.mode = ControlMode::TcpPoseTarget;
        resolved.tcp_target_stand = latch.target_tcp_stand;
        resolved.has_tcp_target = true;
        resolved.has_tcp_delta_stand = false;
        resolved.has_tcp_delta_local = false;
        return resolved;
    }

    latch = LatchedCartesianTarget{};
    if (!state.has_valid_tcp_pose || !state.tcp_stand.has_value()) {
        return command;
    }
    if (command.mode == ControlMode::TcpDeltaStand && !command.has_tcp_delta_stand) {
        return command;
    }
    if (command.mode == ControlMode::TcpDeltaLocal && !command.has_tcp_delta_local) {
        return command;
    }

    CartesianController cartesian(
        config_.left_mount,
        config_.right_mount,
        config_.cartesian_control,
        kinematics_
    );
    const Pose6D target_tcp_stand = command.mode == ControlMode::TcpDeltaStand
        ? cartesian.applyTcpDeltaStand(*state.tcp_stand, command.tcp_delta_stand)
        : cartesian.applyTcpDeltaLocal(*state.tcp_stand, command.tcp_delta_local);

    latch.seq = command_seq;
    latch.target_tcp_stand = target_tcp_stand;
    latch.valid = true;

    resolved.mode = ControlMode::TcpPoseTarget;
    resolved.tcp_target_stand = target_tcp_stand;
    resolved.has_tcp_target = true;
    resolved.has_tcp_delta_stand = false;
    resolved.has_tcp_delta_local = false;
    return resolved;
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
    const auto clear_left_circle_move = [&]() {
        left_cartesian_circle_move_ = CartesianCircleMoveState{};
        left_last_cartesian_solve_ = CartesianSolveTelemetry{};
    };
    const auto clear_right_circle_move = [&]() {
        right_cartesian_circle_move_ = CartesianCircleMoveState{};
        right_last_cartesian_solve_ = CartesianSolveTelemetry{};
    };

    if (left_cartesian_servo_path_.active && !isValidJointState(left_state)) {
        clear_left_linear_path();
    }
    if (right_cartesian_servo_path_.active && !isValidJointState(right_state)) {
        clear_right_linear_path();
    }
    if (left_cartesian_circle_move_.active && !isValidJointState(left_state)) {
        clear_left_circle_move();
    }
    if (right_cartesian_circle_move_.active && !isValidJointState(right_state)) {
        clear_right_circle_move();
    }
    if (linearPathLeaseExpired(left_cartesian_servo_path_, command.host_time_ns)) {
        clear_left_linear_path();
    }
    if (linearPathLeaseExpired(right_cartesian_servo_path_, command.host_time_ns)) {
        clear_right_linear_path();
    }
    if (circlePathLeaseExpired(left_cartesian_circle_move_, command.host_time_ns)) {
        clear_left_circle_move();
    }
    if (circlePathLeaseExpired(right_cartesian_circle_move_, command.host_time_ns)) {
        clear_right_circle_move();
    }
    if (!synthetic_hold) {
        if (command.left.mode != ControlMode::TcpLinearMove) {
            clear_left_linear_path();
        }
        if (command.right.mode != ControlMode::TcpLinearMove) {
            clear_right_linear_path();
        }
        if (command.left.mode != ControlMode::TcpCircleMove) {
            clear_left_circle_move();
        }
        if (command.right.mode != ControlMode::TcpCircleMove) {
            clear_right_circle_move();
        }
        if (!isCartesianVelocityServoMode(command.left.mode)) {
            resetCartesianVelocityIntegrator(ArmId::Left, "velocity_mode_exit");
        }
        if (!isCartesianVelocityServoMode(command.right.mode)) {
            resetCartesianVelocityIntegrator(ArmId::Right, "velocity_mode_exit");
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
    const bool continue_left_circle = synthetic_hold &&
        left_cartesian_circle_move_.active &&
        !left_cartesian_circle_move_.done;
    const bool continue_right_circle = synthetic_hold &&
        right_cartesian_circle_move_.active &&
        !right_cartesian_circle_move_.done;
    if (synthetic_hold && left_cartesian_servo_path_.active && left_cartesian_servo_path_.done) {
        resetCartesianVelocityIntegrator(ArmId::Left, "linear_path_done");
    }
    if (synthetic_hold && right_cartesian_servo_path_.active && right_cartesian_servo_path_.done) {
        resetCartesianVelocityIntegrator(ArmId::Right, "linear_path_done");
    }
    if (synthetic_hold && left_cartesian_circle_move_.active && left_cartesian_circle_move_.done) {
        resetCartesianVelocityIntegrator(ArmId::Left, "circle_move_done");
    }
    if (synthetic_hold && right_cartesian_circle_move_.active && right_cartesian_circle_move_.done) {
        resetCartesianVelocityIntegrator(ArmId::Right, "circle_move_done");
    }

    DualArmCommand effective_command = command;
    if (continue_left_linear) {
        effective_command.left = linearMoveContinuationCommand(command.left, left_cartesian_servo_path_);
    }
    if (continue_right_linear) {
        effective_command.right = linearMoveContinuationCommand(command.right, right_cartesian_servo_path_);
    }
    if (continue_left_circle) {
        effective_command.left = circleMoveContinuationCommand(command.left, left_cartesian_circle_move_);
    }
    if (continue_right_circle) {
        effective_command.right = circleMoveContinuationCommand(command.right, right_cartesian_circle_move_);
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

        if (effective_command.left.mode != ControlMode::TcpTwistStand && effective_command.left.mode != ControlMode::TcpTwistLocal) {
            left_cartesian_twist_hold_ = CartesianTwistHoldState{};
        }
        if (effective_command.right.mode != ControlMode::TcpTwistStand && effective_command.right.mode != ControlMode::TcpTwistLocal) {
            right_cartesian_twist_hold_ = CartesianTwistHoldState{};
        }

        const ArmCommand left_pose_track_command = applyPoseTrackSmd(
            effective_command.left,
            config_.cartesian_control.pose_track_smd,
            &left_pose_track_smd_,
            kinematics_,
            config_.left_mount,
            left_prev_sent_q_deg_,
            dt_sec
        );
        const ArmCommand right_pose_track_command = applyPoseTrackSmd(
            effective_command.right,
            config_.cartesian_control.pose_track_smd,
            &right_pose_track_smd_,
            kinematics_,
            config_.right_mount,
            right_prev_sent_q_deg_,
            dt_sec
        );

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
                left_cartesian_velocity_integrator_.cartesian_servo_state_source =
                    left_servo_state.context.servo_state_source;
                left_cartesian_velocity_integrator_.cartesian_divergence_source =
                    left_servo_state.context.divergence_source;
                left_cartesian_velocity_integrator_.q_reference_for_servo_valid =
                    left_servo_state.context.q_reference_for_servo_valid;
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
                right_cartesian_velocity_integrator_.cartesian_servo_state_source =
                    right_servo_state.context.servo_state_source;
                right_cartesian_velocity_integrator_.cartesian_divergence_source =
                    right_servo_state.context.divergence_source;
                right_cartesian_velocity_integrator_.q_reference_for_servo_valid =
                    right_servo_state.context.q_reference_for_servo_valid;
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

        const CartesianArmTargetResult left_cartesian_result = effective_command.left.mode == ControlMode::TcpLinearMove
            ? cartesian_servo.computeLinearMoveTarget(
                effective_command.left,
                left_servo_state.state,
                left_prev_sent_q_deg_,
                left_cartesian_compute_run_mode,
                dt_sec,
                continue_left_linear ? 0 : command.seq,
                &left_cartesian_servo_path_,
                &left_cartesian_velocity_integrator_,
                &left_servo_state.context
            )
            : effective_command.left.mode == ControlMode::TcpCircleTrack
            ? rejectedCircleTrackTarget(
                left_servo_state.state,
                config_.cartesian_control,
                left_prev_sent_q_deg_
            )
            : effective_command.left.mode == ControlMode::TcpCircleMove
            ? cartesian_servo.computeCircleMoveTarget(
                effective_command.left,
                left_servo_state.state,
                left_prev_sent_q_deg_,
                left_cartesian_compute_run_mode,
                dt_sec,
                continue_left_circle ? 0 : command.seq,
                &left_cartesian_circle_move_,
                &left_cartesian_velocity_integrator_,
                &left_servo_state.context
            )
            : (effective_command.left.mode == ControlMode::TcpTwistStand || effective_command.left.mode == ControlMode::TcpTwistLocal)
            ? cartesian_servo.computeTwistTarget(
                effective_command.left,
                left_servo_state.state,
                left_prev_sent_q_deg_,
                left_cartesian_compute_run_mode,
                dt_sec,
                command.seq,
                &left_cartesian_twist_hold_,
                &left_cartesian_velocity_integrator_,
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
        if (command.left.mode == ControlMode::TcpCircleMove && left_cartesian_circle_move_.active) {
            left_cartesian_circle_move_.lease_enforced = command.lease.enforce_lease;
            left_cartesian_circle_move_.lease_expires_time_ns = command.lease.expires_time_ns;
        }

        const CartesianArmTargetResult right_cartesian_result = effective_command.right.mode == ControlMode::TcpLinearMove
            ? cartesian_servo.computeLinearMoveTarget(
                effective_command.right,
                right_servo_state.state,
                right_prev_sent_q_deg_,
                right_cartesian_compute_run_mode,
                dt_sec,
                continue_right_linear ? 0 : command.seq,
                &right_cartesian_servo_path_,
                &right_cartesian_velocity_integrator_,
                &right_servo_state.context
            )
            : effective_command.right.mode == ControlMode::TcpCircleTrack
            ? rejectedCircleTrackTarget(
                right_servo_state.state,
                config_.cartesian_control,
                right_prev_sent_q_deg_
            )
            : effective_command.right.mode == ControlMode::TcpCircleMove
            ? cartesian_servo.computeCircleMoveTarget(
                effective_command.right,
                right_servo_state.state,
                right_prev_sent_q_deg_,
                right_cartesian_compute_run_mode,
                dt_sec,
                continue_right_circle ? 0 : command.seq,
                &right_cartesian_circle_move_,
                &right_cartesian_velocity_integrator_,
                &right_servo_state.context
            )
            : (effective_command.right.mode == ControlMode::TcpTwistStand || effective_command.right.mode == ControlMode::TcpTwistLocal)
            ? cartesian_servo.computeTwistTarget(
                effective_command.right,
                right_servo_state.state,
                right_prev_sent_q_deg_,
                right_cartesian_compute_run_mode,
                dt_sec,
                command.seq,
                &right_cartesian_twist_hold_,
                &right_cartesian_velocity_integrator_,
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
        if (command.right.mode == ControlMode::TcpCircleMove && right_cartesian_circle_move_.active) {
            right_cartesian_circle_move_.lease_enforced = command.lease.enforce_lease;
            right_cartesian_circle_move_.lease_expires_time_ns = command.lease.expires_time_ns;
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
            // 개발/mock/rbsim용 복구 정책: 현재 실제 자세를 새 안전 기준점으로 삼고 그 자리에서 멈춘다.
            out.left_q_target_deg = left_state.q_actual_deg;
            out.right_q_target_deg = right_state.q_actual_deg;
        } else if (tracking_error_nonlatching && !tracking_error_physical_motion_fault) {
            // pgmode controller-sim advisory: keep following the rate-limited desired
            // target instead of holding/latching, so teleop stays live. The lag is the
            // diagnostics_suspect controller's reference readback, with no physical
            // motion. Surfaced as degraded telemetry + throttled WARN; real mode keeps
            // the latch (gate closed above).
            out.left_q_target_deg = safety_filter_.clampMotion(
                desired.left_q_target_deg,
                left_prev_sent_q_deg_,
                left_prevprev_sent_q_deg_,
                dt_sec
            );
            out.right_q_target_deg = safety_filter_.clampMotion(
                desired.right_q_target_deg,
                right_prev_sent_q_deg_,
                right_prevprev_sent_q_deg_,
                dt_sec
            );
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

    // Dual-arm self-collision guard: never command a configuration that brings the
    // two arms' link capsules within the configured margin. Evaluated on the final
    // candidate targets (post per-arm filtering).
    if (config_.safety.self_collision.enable) {
        const SelfCollisionResult sc =
            evaluateSelfCollision(out.left_q_target_deg, out.right_q_target_deg);
        last_self_collision_ = sc;  // telemetry, even when already faulted
        // Fail closed on a violation, or if geometry is unavailable; skip once
        // latched. monitor_only keeps the telemetry but never clamps/latches
        // (tuning aid only — not a real-motion safety posture).
        if (!config_.safety.self_collision.monitor_only &&
            (sc.violated || !sc.checked) &&
            combined != SafetyVerdict::FaultLatched &&
            !fault_latched_.load()) {
            const std::string reason = sc.checked
                ? ("self-collision: capsule clearance " + std::to_string(sc.min_clearance_m) +
                   " m below margin " + std::to_string(config_.safety.self_collision.margin_m) + " m")
                : "self-collision guard: link geometry unavailable";
            if (config_.safety.self_collision.fail_policy == SelfCollisionFailPolicy::FaultLatch) {
                latchFault(SafetyVerdict::SelfCollision, reason, left_state, right_state);
                out = currentFaultHoldTarget();
                combined = SafetyVerdict::FaultLatched;
            } else {
                // ClampToHold with an escape direction: refuse to advance *toward*
                // collision, but allow a commanded target that strictly increases
                // clearance versus the last sent configuration (retreating out of
                // the keep-out zone). Without the escape exception the arm is
                // permanently frozen once it touches the margin and can never back
                // out. Geometry-unavailable (!checked) always holds (fail closed).
                constexpr double kEscapeEpsM = 1e-4;
                bool allow_escape = false;
                if (sc.checked) {
                    const SelfCollisionResult prev =
                        evaluateSelfCollision(left_prev_sent_q_deg_, right_prev_sent_q_deg_);
                    allow_escape = prev.checked &&
                        sc.min_clearance_m > prev.min_clearance_m + kEscapeEpsM;
                }
                if (!allow_escape) {
                    out.left_q_target_deg = left_prev_sent_q_deg_;
                    out.right_q_target_deg = right_prev_sent_q_deg_;
                    if (combined == SafetyVerdict::Ok || combined == SafetyVerdict::JointLimitClamped) {
                        combined = SafetyVerdict::SelfCollision;
                    }
                }
            }
        }
    }

    if (verdict) *verdict = combined;
    return out;
}

SelfCollisionResult DualArmServoLoop::evaluateSelfCollision(
    const JointArray& left_q_deg,
    const JointArray& right_q_deg
) const {
    if (!kinematics_) {
        return SelfCollisionResult{};  // checked=false -> caller fails closed
    }
    std::vector<std::array<double, 3>> left_points;
    std::vector<std::array<double, 3>> right_points;
    try {
        left_points = kinematics_->linkCollisionPointsInStand(ArmId::Left, left_q_deg, config_.left_mount);
        right_points = kinematics_->linkCollisionPointsInStand(ArmId::Right, right_q_deg, config_.right_mount);
    } catch (const std::exception&) {
        return SelfCollisionResult{};  // checked=false -> caller fails closed
    }
    return dualArmSelfCollisionClearance(
        left_points,
        right_points,
        config_.safety.self_collision.link_radius_m,
        config_.safety.self_collision.margin_m
    );
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
    cmd.left.q_target_deg = chooseSafeHoldTarget(left_state, left_prev_sent_q_deg_);
    cmd.right.q_target_deg = chooseSafeHoldTarget(right_state, right_prev_sent_q_deg_);
    return cmd;
}

bool DualArmServoLoop::commandRequestsResetFault(const DualArmCommand& command) const {
    return command.left.mode == ControlMode::ResetFault || command.right.mode == ControlMode::ResetFault;
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

bool DualArmServoLoop::commandRequestsMotion(const DualArmCommand& command) const {
    return isMotionMode(command.left.mode) || isMotionMode(command.right.mode);
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
