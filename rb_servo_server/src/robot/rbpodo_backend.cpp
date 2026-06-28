#include "rb_servo/robot/rbpodo_backend.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <memory>
#include <optional>
#include <chrono>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include "rb_servo/core/clock.hpp"
#include "rb_servo/core/shutdown.hpp"

#ifdef RB_SERVO_ENABLE_RBPODO
#include <rbpodo/rbpodo.hpp>
#endif

namespace rb_servo {
namespace {


bool finiteJointArray(const JointArray& joints) {
    return std::all_of(joints.begin(), joints.end(), [](double value) {
        return std::isfinite(value);
    });
}

template <typename T>
BackendResult<T> okResult(BackendOp op, const T& value, const BackendTiming& timing = BackendTiming{}) {
    BackendResult<T> result;
    result.ok = true;
    result.op = op;
    result.value = value;
    result.error = noBackendError();
    result.timing = timing;
    return result;
}

template <typename T>
BackendResult<T> failedResult(BackendOp op, const BackendError& error, const BackendTiming& timing = BackendTiming{}) {
    BackendResult<T> result;
    result.ok = false;
    result.op = op;
    result.error = error;
    result.timing = timing;
    return result;
}

template <typename T>
BackendResult<T> failedResultWithValue(
    BackendOp op,
    const BackendError& error,
    const T& value,
    const BackendTiming& timing = BackendTiming{}
) {
    BackendResult<T> result = failedResult<T>(op, error, timing);
    result.value = value;
    return result;
}

RobotState basicState(ArmId arm_id, bool connected) {
    RobotState state;
    state.arm_id = arm_id;
    state.host_time_ns = nowSteadyNs();
    state.connection_state = connected
        ? RobotConnectionState::Connected
        : RobotConnectionState::Disconnected;
    state.has_valid_joint_state = false;
    return state;
}

std::string lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

bool expectedSimulationMode(const BackendConfig& config) {
    const std::string operation_mode = lower(config.operation_mode);
    if (operation_mode == "simulation" || operation_mode == "sim") {
        return true;
    }
    return false;
}

std::string rbpodoModeName(bool simulation_mode) {
    return simulation_mode ? "simulation" : "real";
}

bool rbpodoModeFieldIsKnown(int real_vs_simulation_mode) {
    return real_vs_simulation_mode == 0 || real_vs_simulation_mode == 1;
}

bool operationModeMatchesConfig(const BackendConfig& config, const RbpodoSystemStateSnapshot& snapshot) {
    if (!rbpodoModeFieldIsKnown(snapshot.real_vs_simulation_mode)) return false;
    const bool actual_simulation = snapshot.real_vs_simulation_mode == 1;
    return expectedSimulationMode(config) == actual_simulation;
}

bool rbpodoControllerSimulationMotionGateOpen(const BackendConfig& config) {
    return config.backend_type == BackendType::Rbpodo &&
        config.run_mode == RunMode::Real &&
        expectedSimulationMode(config);
}

// REAL physical-motion opt-in (operation_mode: real, NOT controller-sim): accept the
// same vendor-unreliable status fields (op_stat_self_collision shape, robot_time) as
// UNAVAILABLE instead of latching diagnostics_suspect. Fail-closed: requires the per-arm
// config opt-in (allow_real_motion_with_suspect_diagnostics) AND a real (non-sim)
// operation mode. Every other field (EMS/SOS/soft-estop/collision_occur, unknown
// real_vs_sim mode, init error) keeps faulting exactly as before — this only suppresses
// the two measured field-layout-garbage fields so a physical run is not blocked by the
// vendor -2001 mismatch.
bool rbpodoSuspectDiagnosticsRealMotionGateOpen(const BackendConfig& config) {
    return config.backend_type == BackendType::Rbpodo &&
        config.run_mode == RunMode::Real &&
        !expectedSimulationMode(config) &&
        config.allow_real_motion_with_suspect_diagnostics;
}

RbpodoStateDecodeOptions decodeOptionsForConfig(const BackendConfig& config) {
    RbpodoStateDecodeOptions options;
    const bool controller_sim_unreliable_gate =
        config.controller_simulation_treat_unreliable_status_fields_as_unavailable &&
        rbpodoControllerSimulationMotionGateOpen(config);
    const bool real_motion_suspect_gate =
        rbpodoSuspectDiagnosticsRealMotionGateOpen(config);
    options.controller_simulation_unreliable_status_fields_unavailable =
        controller_sim_unreliable_gate || real_motion_suspect_gate;
    options.real_motion_suspect_diagnostics_accepted = real_motion_suspect_gate;
    // Controller-simulation ONLY: gated by the controller-sim motion gate (never real),
    // so a clean op_stat_self_collision (1005) defers to the server's mesh CollisionMonitor.
    options.demote_self_collision_fault =
        config.controller_simulation_demote_self_collision_fault &&
        rbpodoControllerSimulationMotionGateOpen(config);
    return options;
}

bool diagnosticsSuspectControllerSimulationOverrideAllowed(
    const BackendConfig& config,
    const BackendError& error
) {
    return error.name == "rbpodo_diagnostics_suspect" &&
        rbpodoControllerSimulationMotionGateOpen(config) &&
        config.allow_controller_simulation_diagnostics_suspect;
}

bool initErrorControllerSimulationOverrideAllowed(
    const BackendConfig& config,
    const BackendError& error
) {
    return error.name == "rbpodo_init_error" &&
        rbpodoControllerSimulationMotionGateOpen(config) &&
        config.allow_controller_simulation_init_error;
}

bool controllerSimulationReadinessOverrideAllowed(
    const BackendConfig& config,
    const BackendError& error
) {
    return diagnosticsSuspectControllerSimulationOverrideAllowed(config, error) ||
        initErrorControllerSimulationOverrideAllowed(config, error);
}

void annotateRbpodoAckResult(
    SendServoJResult* result,
    const BackendConfig& config,
    double ack_wait_duration_us,
    bool ack_observed
) {
    result->ack_policy = config.disable_waiting_ack ? BackendAckPolicy::Disabled : BackendAckPolicy::Wait;
    result->rbpodo_waiting_ack = !config.disable_waiting_ack;
    result->ack_wait_duration_us = config.disable_waiting_ack ? 0.0 : ack_wait_duration_us;
    if (config.disable_waiting_ack) {
        result->ack_observed = false;
        result->controller_acceptance_observed = false;
        result->acceptance_semantics = result->accepted ? "socket_send_only" : "not_sent";
        return;
    }

    result->ack_observed = ack_observed;
    result->controller_acceptance_observed = result->accepted && ack_observed;
    result->acceptance_semantics =
        result->controller_acceptance_observed ? "controller_ack_observed" : "controller_ack_not_observed";
}

constexpr int kRbpodoDiagnosticsSuspectCode = -2001;
constexpr int kRbpodoSoftEstopCode = 1003;
constexpr int kRbpodoCollisionCode = 1004;
constexpr int kRbpodoSelfCollisionCode = 1005;
constexpr int kRbpodoSuspiciousRawCodeCutoff = 1'000'000;
constexpr double kRbpodoMinPlausibleNonZeroTimeSec = 1e-6;

// Rainbow controller op_stat_* status fields are BIT-PACKED: only the documented
// low bits carry the status, the upper bits are reserved/undefined and routinely
// observed carrying uninitialised values (e.g. op_stat_self_collision arrives as
// 0x76904aa0 with bit0 = 0). They MUST be masked before validating the boolean/code
// shape or interpreting a fault, otherwise a healthy field reads as "garbage" (false
// diagnostics_suspect) and, worse, a real fault with non-zero upper bits is missed by
// an `== 1` test. Ref: rainbowrobotics.github.io rb_cobot_docs technical_docs/data_structure
//   item 33 op_stat_collision_occur : lower 2 bits  (0/1)
//   item 34 op_stat_sos_flag        : lower 6 bits  (device code 0..12)
//   item 35 op_stat_self_collision  : bits 0-1 self-collision (0/1), bits 2-3 last out-collision source
//   item 37 op_stat_ems_flag        : lower 6 bits  (kinematic e-stop code 0..4)
constexpr int kRbpodoCollisionOccurMask = 0b11;    // item 33: lower 2 bits valid
constexpr int kRbpodoSelfCollisionMask = 0b11;     // item 35: bits 0-1 = self-collision flag
constexpr int kRbpodoStatusCodeMask = 0b111111;    // items 34/37: lower 6 bits valid

struct RbpodoInterpretedFault {
    int code = 0;
    std::string name;
};

void appendDiagnosticReason(std::string* reason, const std::string& item) {
    if (!reason) return;
    if (!reason->empty()) reason->append("; ");
    reason->append(item);
}

bool rbpodoBooleanFlagValue(int value) {
    return value == 0 || value == 1;
}

bool rbpodoSuspiciousRawCode(int value) {
    return value >= kRbpodoSuspiciousRawCodeCutoff ||
        value <= -kRbpodoSuspiciousRawCodeCutoff;
}

RbpodoRawDiagnostics rawDiagnosticsFromSnapshot(const RbpodoSystemStateSnapshot& snapshot) {
    RbpodoRawDiagnostics raw;
    raw.time_sec = snapshot.robot_time_sec;
    raw.real_vs_simulation_mode = snapshot.real_vs_simulation_mode;
    raw.init_state_info = snapshot.init_state_info;
    raw.init_error = snapshot.init_error;
    raw.op_stat_sos_flag = snapshot.op_stat_sos_flag;
    raw.op_stat_ems_flag = snapshot.op_stat_ems_flag;
    raw.op_stat_soft_estop_occur = snapshot.op_stat_soft_estop_occur;
    raw.op_stat_collision_occur = snapshot.op_stat_collision_occur;
    raw.op_stat_self_collision = snapshot.op_stat_self_collision;
    return raw;
}

void markSuspiciousFlag(
    RbpodoDiagnosticsSnapshot* diagnostics,
    const std::string& field_name,
    int value
) {
    if (!diagnostics || rbpodoBooleanFlagValue(value)) return;
    diagnostics->diagnostics_valid = false;
    diagnostics->diagnostics_suspect = true;
    appendDiagnosticReason(
        &diagnostics->reason,
        field_name + " expected 0/1 but was " + std::to_string(value)
    );
}

void markUnavailableField(
    RbpodoDiagnosticsSnapshot* diagnostics,
    const std::string& field_name
) {
    if (!diagnostics) return;
    if (std::find(
            diagnostics->unavailable_fields.begin(),
            diagnostics->unavailable_fields.end(),
            field_name
        ) == diagnostics->unavailable_fields.end()) {
        diagnostics->unavailable_fields.push_back(field_name);
    }
}

void markSuspiciousBoundedCode(
    RbpodoDiagnosticsSnapshot* diagnostics,
    const std::string& field_name,
    int value,
    int min_value,
    int max_value
) {
    if (!diagnostics || (value >= min_value && value <= max_value)) return;
    diagnostics->diagnostics_valid = false;
    diagnostics->diagnostics_suspect = true;
    appendDiagnosticReason(
        &diagnostics->reason,
        field_name + " expected " + std::to_string(min_value) + ".." +
            std::to_string(max_value) + " but was " + std::to_string(value)
    );
}

RbpodoDiagnosticsSnapshot interpretRbpodoDiagnostics(
    const RbpodoSystemStateSnapshot& snapshot,
    const RbpodoStateDecodeOptions& decode_options
) {
    RbpodoDiagnosticsSnapshot diagnostics;
    diagnostics.raw = rawDiagnosticsFromSnapshot(snapshot);
    const bool unreliable_fields_unavailable =
        decode_options.controller_simulation_unreliable_status_fields_unavailable;

    // Mask to the documented valid bits before shape-validating (see kRbpodo*Mask).
    markSuspiciousBoundedCode(
        &diagnostics, "op_stat_sos_flag", snapshot.op_stat_sos_flag & kRbpodoStatusCodeMask, 0, 12);
    markSuspiciousBoundedCode(
        &diagnostics, "op_stat_ems_flag", snapshot.op_stat_ems_flag & kRbpodoStatusCodeMask, 0, 4);
    // op_stat_soft_estop_occur is documented as a plain 0/1 (no reserved upper bits), so it is
    // validated unmasked — a non-boolean value there is a genuine anomaly.
    markSuspiciousFlag(&diagnostics, "op_stat_soft_estop_occur", snapshot.op_stat_soft_estop_occur);
    markSuspiciousFlag(
        &diagnostics, "op_stat_collision_occur",
        snapshot.op_stat_collision_occur & kRbpodoCollisionOccurMask);
    if (unreliable_fields_unavailable) {
        markUnavailableField(&diagnostics, "op_stat_self_collision");
    } else {
        markSuspiciousFlag(
            &diagnostics, "op_stat_self_collision",
            snapshot.op_stat_self_collision & kRbpodoSelfCollisionMask);
    }

    if (unreliable_fields_unavailable &&
        (!std::isfinite(snapshot.robot_time_sec) ||
         snapshot.robot_time_sec <= 0.0 ||
         (snapshot.robot_time_sec > 0.0 &&
          snapshot.robot_time_sec < kRbpodoMinPlausibleNonZeroTimeSec))) {
        markUnavailableField(&diagnostics, "robot_time_sec");
    } else if (!std::isfinite(snapshot.robot_time_sec)) {
        diagnostics.diagnostics_valid = false;
        diagnostics.diagnostics_suspect = true;
        appendDiagnosticReason(&diagnostics.reason, "time was non-finite");
    } else if (snapshot.robot_time_sec < 0.0 ||
               (snapshot.robot_time_sec > 0.0 &&
                snapshot.robot_time_sec < kRbpodoMinPlausibleNonZeroTimeSec)) {
        diagnostics.diagnostics_valid = false;
        diagnostics.diagnostics_suspect = true;
        appendDiagnosticReason(
            &diagnostics.reason,
            "time was implausible: " + std::to_string(snapshot.robot_time_sec)
        );
    }

    if (!rbpodoModeFieldIsKnown(snapshot.real_vs_simulation_mode)) {
        diagnostics.diagnostics_valid = false;
        diagnostics.diagnostics_suspect = true;
        appendDiagnosticReason(
            &diagnostics.reason,
            "real_vs_simulation_mode unknown: " + std::to_string(snapshot.real_vs_simulation_mode)
        );
    }

    if (diagnostics.diagnostics_suspect) {
        diagnostics.error_name = "rbpodo_diagnostics_suspect";
        diagnostics.stable_error_code = kRbpodoDiagnosticsSuspectCode;
    }
    if (!diagnostics.unavailable_fields.empty()) {
        std::ostringstream fields;
        for (std::size_t i = 0; i < diagnostics.unavailable_fields.size(); ++i) {
            if (i > 0) fields << ",";
            fields << diagnostics.unavailable_fields[i];
        }
        appendDiagnosticReason(
            &diagnostics.reason,
            "unavailable fields under controller-simulation decode policy: " + fields.str()
        );
    }
    return diagnostics;
}

std::optional<RbpodoInterpretedFault> firstClearRbpodoFault(
    const RbpodoSystemStateSnapshot& snapshot,
    const RbpodoDiagnosticsSnapshot& diagnostics,
    const RbpodoStateDecodeOptions& decode_options
) {
    // Mask to the documented valid bits before interpreting a fault (see kRbpodo*Mask).
    // Using the raw value here would miss a real fault whenever the reserved upper bits
    // are non-zero (e.g. self-collision arriving as 0x...01 never equals 1).
    const int sos_flag = snapshot.op_stat_sos_flag & kRbpodoStatusCodeMask;
    if (sos_flag >= 1 && sos_flag <= 12) {
        return RbpodoInterpretedFault{sos_flag, "rbpodo_sos_flag"};
    }
    if (snapshot.init_error != 0) {
        return RbpodoInterpretedFault{snapshot.init_error, "rbpodo_init_error"};
    }
    const int ems_flag = snapshot.op_stat_ems_flag & kRbpodoStatusCodeMask;
    if (ems_flag >= 1 && ems_flag <= 4) {
        return RbpodoInterpretedFault{ems_flag, "rbpodo_ems_flag"};
    }
    if (snapshot.op_stat_soft_estop_occur == 1) {
        return RbpodoInterpretedFault{kRbpodoSoftEstopCode, "rbpodo_soft_estop"};
    }
    if ((snapshot.op_stat_collision_occur & kRbpodoCollisionOccurMask) == 1) {
        return RbpodoInterpretedFault{kRbpodoCollisionCode, "rbpodo_collision"};
    }
    // A CLEAN self-collision bit (masked == 1) is the vendor "self-collision asserted"
    // signal and normally latches a hard RobotFault even under the unavailable-field policy
    // (which only demotes garbage/undefined shapes). In controller-simulation (pgmode) the
    // flag false-positives at full-amplitude TcpPoseTarget replay, so the explicit
    // controller_simulation_demote_self_collision_fault opt-in skips it here and defers to
    // the server's trusted async URDF-mesh CollisionMonitor. Gated to controller-sim only
    // (decode_options.demote_self_collision_fault is never set for real motion).
    if (!decode_options.demote_self_collision_fault &&
        (snapshot.op_stat_self_collision & kRbpodoSelfCollisionMask) == 1) {
        return RbpodoInterpretedFault{kRbpodoSelfCollisionCode, "rbpodo_self_collision"};
    }

    if (diagnostics.diagnostics_suspect) {
        if (snapshot.op_stat_soft_estop_occur != 0 &&
            !rbpodoSuspiciousRawCode(snapshot.op_stat_soft_estop_occur)) {
            return RbpodoInterpretedFault{snapshot.op_stat_soft_estop_occur, "rbpodo_soft_estop_suspect"};
        }
        // NO raw-value fallback for op_stat_collision_occur (item 33) or
        // op_stat_self_collision (item 35). Per the Rainbow data-structure spec, ONLY the
        // low bits are valid: item 33 = lower 2 bits (0/1); item 35 = bits0-1 self-collision
        // (0/1), bits2-3 last out-collision SOURCE, bits4-31 control-box TIME ZONE. The
        // masked checks above ((field & kRbpodo*Mask) == 1, lines for collision/self-
        // collision) are the complete truth and run regardless of diagnostics_suspect, so a
        // real collision is already caught. Reading the RAW value here only false-positives
        // on the reserved/source/time-zone bits — e.g. the observed op_stat_self_collision =
        // 0x76904aa0 has its self-collision bit = 0 yet a nonzero raw. A non-1 low-bit value
        // (e.g. 2) is malformed garbage, surfaced by the generic diagnostics_suspect path,
        // not a collision. soft_estop is kept: it is a plain 0/1 with no reserved upper bits.
    }

    return std::nullopt;
}

}  // namespace

RobotState mapRbpodoSystemStateSnapshot(
    ArmId arm_id,
    const RbpodoSystemStateSnapshot& snapshot,
    const RbpodoStateDecodeOptions& decode_options
) {
    RobotState out_state;
    out_state.arm_id = arm_id;
    out_state.host_time_ns = nowSteadyNs();
    out_state.robot_time_ns = std::isfinite(snapshot.robot_time_sec) && snapshot.robot_time_sec >= 0.0
        ? static_cast<uint64_t>(snapshot.robot_time_sec * 1'000'000'000.0)
        : 0;

    out_state.q_actual_deg = snapshot.q_actual_deg;
    out_state.q_target_deg = snapshot.q_target_deg;
    out_state.dq_actual_deg_s.fill(0.0);
    out_state.q_actual_valid = finiteJointArray(out_state.q_actual_deg);
    out_state.q_ref_valid = finiteJointArray(out_state.q_target_deg);
    out_state.q_ref_source = "rbpodo.sdata.jnt_ref";
    out_state.rbpodo_sdk_state_source = "CobotData.request_data";
    out_state.rbpodo_state_decode_policy =
        decode_options.real_motion_suspect_diagnostics_accepted
            ? "real_motion_suspect_diagnostics_accepted"
            : (decode_options.controller_simulation_unreliable_status_fields_unavailable
                ? "controller_sim_unreliable_fields_unavailable"
                : "bounded_status_codes_with_boolean_safety_flags");

    RbpodoDiagnosticsSnapshot diagnostics = interpretRbpodoDiagnostics(snapshot, decode_options);
    const std::optional<RbpodoInterpretedFault> clear_fault =
        firstClearRbpodoFault(snapshot, diagnostics, decode_options);
    out_state.connection_state = RobotConnectionState::Connected;
    out_state.controller_motion_state = snapshot.robot_state;
    out_state.controller_freedrive_on = snapshot.is_freedrive_mode == 1;
    out_state.servo_enabled = snapshot.init_state_info == 6;
    out_state.has_error = clear_fault.has_value() || diagnostics.diagnostics_suspect;
    out_state.error_code = clear_fault.has_value()
        ? clear_fault->code
        : (diagnostics.diagnostics_suspect ? diagnostics.stable_error_code : 0);
    out_state.diagnostic_error_source = clear_fault.has_value()
        ? clear_fault->name
        : diagnostics.error_name;
    out_state.rbpodo_diagnostics = diagnostics;
    out_state.has_valid_joint_state = out_state.q_actual_valid && out_state.q_ref_valid;
    if (!out_state.has_valid_joint_state) {
        out_state.lifecycle_state = "invalid_joint_state";
    } else if (diagnostics.diagnostics_suspect && !clear_fault.has_value()) {
        out_state.lifecycle_state = "diagnostics_suspect";
        out_state.fault_recoverable = true;
    } else if (out_state.has_error) {
        out_state.lifecycle_state = "faulted";
        out_state.fault_recoverable = true;
    } else if (out_state.servo_enabled) {
        out_state.lifecycle_state = "servo_enabled";
    } else {
        out_state.lifecycle_state = "connected_not_motion_ready";
    }
    return out_state;
}

RobotState mapRbpodoSystemStateSnapshot(
    ArmId arm_id,
    const RbpodoSystemStateSnapshot& snapshot
) {
    return mapRbpodoSystemStateSnapshot(arm_id, snapshot, RbpodoStateDecodeOptions{});
}

RobotState mapRbpodoSystemStateSnapshot(
    ArmId arm_id,
    const RbpodoSystemStateSnapshot& snapshot,
    const BackendConfig& config
) {
    return mapRbpodoSystemStateSnapshot(arm_id, snapshot, decodeOptionsForConfig(config));
}

std::optional<BackendError> rbpodoStateAcquisitionError(const RobotState& mapped) {
    if (!mapped.has_valid_joint_state) {
        return backendError(
            BackendErrorKind::InvalidJointState,
            "rbpodo state contained non-finite joint data",
            "",
            "rbpodo_invalid_joint_state"
        );
    }
    return std::nullopt;
}

std::optional<BackendError> rbpodoMotionReadinessError(
    const BackendConfig& config,
    const RbpodoSystemStateSnapshot& snapshot,
    const RobotState& mapped
) {
    if (const auto acquisition_error = rbpodoStateAcquisitionError(mapped)) {
        return acquisition_error;
    }
    if (mapped.rbpodo_diagnostics.has_value() &&
        mapped.rbpodo_diagnostics->diagnostics_suspect) {
        return backendError(
            BackendErrorKind::RobotFault,
            "rbpodo diagnostics suspect: " + mapped.rbpodo_diagnostics->reason,
            std::to_string(mapped.error_code),
            "rbpodo_diagnostics_suspect"
        );
    }
    if (mapped.has_error) {
        return backendError(
            BackendErrorKind::RobotFault,
            "rbpodo SystemState reported a robot/controller fault",
            std::to_string(mapped.error_code),
            mapped.diagnostic_error_source.empty()
                ? "rbpodo_robot_fault"
                : mapped.diagnostic_error_source
        );
    }
    if (!operationModeMatchesConfig(config, snapshot)) {
        const bool actual_simulation = snapshot.real_vs_simulation_mode == 1;
        const bool expected_simulation = expectedSimulationMode(config);
        return backendError(
            BackendErrorKind::WrongMode,
            "rbpodo controller mode is " + rbpodoModeName(actual_simulation) +
                " but config expects " + rbpodoModeName(expected_simulation),
            std::to_string(snapshot.real_vs_simulation_mode),
            "rbpodo_wrong_operation_mode"
        );
    }
    if (!mapped.servo_enabled) {
        return backendError(
            BackendErrorKind::ServoDisabled,
            "rbpodo activation stage is " + std::to_string(snapshot.init_state_info) +
                "; expected activation done stage 6 before servo_j",
            std::to_string(snapshot.init_state_info),
            "rbpodo_servo_disabled"
        );
    }
    return std::nullopt;
}

void attachMotionReadinessDiagnostic(
    RobotState* state,
    const std::optional<BackendError>& error
) {
    if (!state || !error.has_value()) return;
    state->motion_readiness_error_kind = toString(error->kind);
    state->motion_readiness_error_name = error->name;
    if (state->diagnostic_error_source.empty()) {
        state->diagnostic_error_source = error->name.empty()
            ? toString(error->kind)
            : error->name;
    }
}

namespace {

#ifdef RB_SERVO_ENABLE_RBPODO
constexpr double kDefaultStateTimeoutSec = 0.2;
// One-shot initialize-time commands (e.g. set_speed_bar) can afford a longer ack
// wait than the streaming command_timeout_sec, which is tuned for the servo loop.
constexpr double kInitializeCommandAckTimeoutSec = 1.0;
// pgmode switches (especially simulation -> real, which powers up the servo
// stage) can take tens of seconds on the controller; this is initialize-time,
// so wait generously before declaring the switch unconfirmed.
constexpr uint64_t kOperationModeSwitchTimeoutNs = 60'000'000'000ULL;
constexpr uint64_t kRecentStateCacheMaxAgeNs = 1'000'000'000ULL;

RbpodoSystemStateSnapshot snapshotFromSystemState(const rb::podo::SystemState& rb_state) {
    RbpodoSystemStateSnapshot snapshot;
    snapshot.robot_time_sec = static_cast<double>(rb_state.sdata.time);
    snapshot.real_vs_simulation_mode = rb_state.sdata.real_vs_simulation_mode;
    snapshot.init_state_info = rb_state.sdata.init_state_info;
    snapshot.init_error = rb_state.sdata.init_error;
    snapshot.op_stat_sos_flag = rb_state.sdata.op_stat_sos_flag;
    snapshot.op_stat_ems_flag = rb_state.sdata.op_stat_ems_flag;
    snapshot.op_stat_soft_estop_occur = rb_state.sdata.op_stat_soft_estop_occur;
    snapshot.op_stat_collision_occur = rb_state.sdata.op_stat_collision_occur;
    snapshot.op_stat_self_collision = rb_state.sdata.op_stat_self_collision;
    snapshot.robot_state = rb_state.sdata.robot_state;
    snapshot.is_freedrive_mode = rb_state.sdata.is_freedrive_mode;
    for (int i = 0; i < kDof; ++i) {
        snapshot.q_actual_deg[static_cast<std::size_t>(i)] = rb_state.sdata.jnt_ang[i];
        snapshot.q_target_deg[static_cast<std::size_t>(i)] = rb_state.sdata.jnt_ref[i];
    }
    return snapshot;
}

std::optional<RobotState> recentStateCache(
    const std::optional<RobotState>& cached_state,
    uint64_t now_ns
) {
    if (!cached_state.has_value()) return std::nullopt;
    if (cached_state->host_time_ns == 0) return std::nullopt;
    if (now_ns < cached_state->host_time_ns) return std::nullopt;
    if (now_ns - cached_state->host_time_ns > kRecentStateCacheMaxAgeNs) return std::nullopt;
    return cached_state;
}

std::string responseCollectorText(const rb::podo::ResponseCollector& responses) {
    std::ostringstream out;
    out << responses;
    return out.str();
}

std::string normalizedDiagnosticName(const std::string& prefix, const std::string& value) {
    std::string out = prefix;
    for (unsigned char c : value) {
        if (std::isalnum(c)) {
            out.push_back(static_cast<char>(std::tolower(c)));
        } else if (!out.empty() && out.back() != '_') {
            out.push_back('_');
        }
    }
    while (!out.empty() && out.back() == '_') out.pop_back();
    return out == prefix ? prefix + "response_error" : out;
}

BackendError controllerResponseError(
    const std::string& operation_name,
    const rb::podo::ResponseCollector& responses
) {
    for (const auto& response : responses) {
        if (response.type() == rb::podo::Response::Type::Error) {
            const std::string category = response.category();
            return backendError(
                BackendErrorKind::ControllerRejected,
                response.msg().empty()
                    ? "rbpodo " + operation_name + " response contained an error"
                    : response.msg(),
                category,
                normalizedDiagnosticName("rbpodo_", category)
            );
        }
    }
    const std::string text = responseCollectorText(responses);
    return backendError(
        BackendErrorKind::ControllerRejected,
        text.empty()
            ? "rbpodo " + operation_name + " was rejected by the controller"
            : "rbpodo " + operation_name + " was rejected by the controller: " + text,
        "",
        "rbpodo_" + operation_name + "_rejected"
    );
}

BackendError commandReturnError(
    const std::string& operation_name,
    const rb::podo::ReturnType& ret,
    const rb::podo::ResponseCollector& responses
) {
    if (ret.is_timeout()) {
        return backendError(
            BackendErrorKind::TransportTimeout,
            "rbpodo " + operation_name + " timed out waiting for command acknowledgement",
            "",
            "rbpodo_" + operation_name + "_timeout"
        );
    }
    if (responses.has_error()) {
        return controllerResponseError(operation_name, responses);
    }
    return backendError(
        BackendErrorKind::ControllerRejected,
        "rbpodo " + operation_name + " was not accepted by the controller",
        "",
        "rbpodo_" + operation_name + "_rejected"
    );
}

std::optional<BackendError> configureWaitingAck(
    rb::podo::Cobot<>& robot,
    bool disable_waiting_ack
) {
    rb::podo::ResponseCollector responses;
    const bool configured = disable_waiting_ack
        ? robot.disable_waiting_ack(responses)
        : robot.enable_waiting_ack(responses);
    if (configured) {
        return std::nullopt;
    }

    const std::string operation = disable_waiting_ack ? "disable_waiting_ack" : "enable_waiting_ack";
    return backendError(
        BackendErrorKind::ControllerRejected,
        "rbpodo " + operation + " was not accepted by the SDK",
        "",
        "rbpodo_" + operation + "_rejected"
    );
}

// Documented controller-state queries (rbpodo data_type.hpp):
//   SD_PG_MODE         "Indicates the robot's operation mode. 0 = Real / 1 = Simulation"
//   SD_INIT_STATE_INFO activation stage 0..6 (6 = Activation done; mask & 0x3f
//                      as the SDK's own activate() does)
// set_operation_mode() only waits for the ACK of the "pgmode" script command —
// receipt, not completion — so the switch MUST be confirmed by reading these
// back (or their data-channel equivalents real_vs_simulation_mode /
// init_state_info, which carry the same documented semantics).
std::optional<int> querySystemVariableInt(
    rb::podo::Cobot<>& robot,
    rb::podo::SystemVariable variable,
    double timeout_sec
) {
    rb::podo::ResponseCollector responses;
    double out = 0.0;
    const auto ret = robot.get_system_variable(responses, variable, out, timeout_sec, true);
    if (!ret.is_success()) {
        return std::nullopt;
    }
    return static_cast<int>(out);
}

struct RbpodoModeProbe {
    int pg_mode = -1;      // 0=real, 1=simulation (SD_PG_MODE semantics)
    int init_stage = -1;   // 0..6 activation stage, -1 unknown
};

std::string rbpodoPgModeLabel(int pg_mode) {
    if (pg_mode == 0) return "real";
    if (pg_mode == 1) return "simulation";
    return "unknown(" + std::to_string(pg_mode) + ")";
}
#endif

}  // namespace

struct RbpodoBackend::Impl {
    ArmId arm_id;
    BackendConfig config;
    bool connected = false;
    // Direct-teaching (free-drive) latch. While true the controller is in
    // freedrive_teach_on and sendServoJ() refuses to write move_servo_j (a
    // defensive backstop layered under the server's global send suppression).
    bool freedrive_active = false;

    // Soft-entry gain ramp state for move_servo_j RT-servo (re)engagement.
    // ramp_start==0 means "ramp not armed yet" (first send arms it); it also
    // re-arms when the stream resumes after a gap (see sendServoJ()).
    uint64_t servo_engage_ramp_start_ns = 0;
    uint64_t last_servo_j_send_ns = 0;

#ifdef RB_SERVO_ENABLE_RBPODO
    std::unique_ptr<rb::podo::Cobot<>> robot;
    std::unique_ptr<rb::podo::CobotData> data_channel;
    std::optional<RobotState> last_state_cache;
    std::optional<BackendError> last_state_error;
    // Consecutive transient readState misses currently being tolerated (held)
    // under the controller-simulation read-miss carve-out. Reset on any valid read.
    int consecutive_read_misses = 0;
#endif
};

RbpodoBackend::RbpodoBackend(ArmId arm_id, const BackendConfig& config)
    : impl_(std::make_unique<Impl>()) {
    impl_->arm_id = arm_id;
    impl_->config = config;
}

RbpodoBackend::~RbpodoBackend() = default;

BackendResult<RobotState> RbpodoBackend::connect() {
    const uint64_t start = nowSteadyNs();
#ifndef RB_SERVO_ENABLE_RBPODO
    std::cerr << "[ERROR] RbpodoBackend requested, but RB_SERVO_ENABLE_RBPODO=OFF.\n";
    return failedResult<RobotState>(
        BackendOp::Connect,
        backendError(BackendErrorKind::DependencyUnavailable, "RbpodoBackend requested, but RB_SERVO_ENABLE_RBPODO=OFF"),
        makeBackendTiming(start, nowSteadyNs())
    );
#else
    if (impl_->config.ip.empty()) {
        std::cerr << "[ERROR] RbpodoBackend requires a non-empty controller ip for "
                  << impl_->config.name << "\n";
        return failedResult<RobotState>(
            BackendOp::Connect,
            backendError(
                BackendErrorKind::WrongEndpoint,
                "RbpodoBackend requires a non-empty controller ip",
                "",
                "rbpodo_missing_controller_ip"
            ),
            makeBackendTiming(start, nowSteadyNs())
        );
    }

    try {
        impl_->robot = std::make_unique<rb::podo::Cobot<>>(impl_->config.ip);
        if (const auto ack_error = configureWaitingAck(*impl_->robot, impl_->config.disable_waiting_ack)) {
            impl_->robot.reset();
            impl_->data_channel.reset();
            impl_->connected = false;
            return failedResult<RobotState>(
                BackendOp::Connect,
                *ack_error,
                makeBackendTiming(start, nowSteadyNs())
            );
        }
        impl_->data_channel = std::make_unique<rb::podo::CobotData>(impl_->config.ip);
        const auto state = impl_->data_channel->request_data(kDefaultStateTimeoutSec);
        if (!state) {
            std::cerr << "[ERROR] RbpodoBackend connected sockets but did not receive state from "
                      << impl_->config.ip << "\n";
            impl_->robot.reset();
            impl_->data_channel.reset();
            impl_->connected = false;
            return failedResult<RobotState>(
                BackendOp::Connect,
                backendError(
                    BackendErrorKind::TransportReadFailed,
                    "rbpodo connected sockets but did not receive state",
                    "",
                    "rbpodo_connect_state_unavailable"
                ),
                makeBackendTiming(start, nowSteadyNs())
            );
        }
        impl_->connected = true;
        const RbpodoSystemStateSnapshot snapshot = snapshotFromSystemState(*state);
        RobotState mapped = mapRbpodoSystemStateSnapshot(impl_->arm_id, snapshot, impl_->config);
        impl_->last_state_error = rbpodoMotionReadinessError(impl_->config, snapshot, mapped);
        attachMotionReadinessDiagnostic(&mapped, impl_->last_state_error);
        impl_->last_state_cache = mapped.has_valid_joint_state ? std::make_optional(mapped) : std::nullopt;
        if (const auto acquisition_error = rbpodoStateAcquisitionError(mapped)) {
            return failedResultWithValue(
                BackendOp::Connect,
                *acquisition_error,
                mapped,
                makeBackendTiming(start, nowSteadyNs())
            );
        }
        std::cerr << "[INFO] RbpodoBackend connected to " << impl_->config.ip
                  << " for " << impl_->config.name
                  << " initial_state_valid=" << (mapped.has_valid_joint_state ? "true" : "false")
                  << " controller_mode="
                  << (rbpodoModeFieldIsKnown(snapshot.real_vs_simulation_mode)
                      ? rbpodoModeName(snapshot.real_vs_simulation_mode == 1)
                      : "unknown");
        if (impl_->last_state_error.has_value()) {
            std::cerr << " readiness_error=" << impl_->last_state_error->name;
        }
        std::cerr << "\n";
        return okResult(BackendOp::Connect, mapped, makeBackendTiming(start, nowSteadyNs()));
    } catch (const std::exception& exc) {
        std::cerr << "[ERROR] RbpodoBackend connect failed for " << impl_->config.ip
                  << ": " << exc.what() << "\n";
        impl_->robot.reset();
        impl_->data_channel.reset();
        impl_->connected = false;
        return failedResult<RobotState>(
            BackendOp::Connect,
            backendError(
                BackendErrorKind::TransportConnectFailed,
                exc.what(),
                "",
                "rbpodo_connect_exception"
            ),
            makeBackendTiming(start, nowSteadyNs())
        );
    }
#endif
}

BackendResult<RobotState> RbpodoBackend::initialize() {
    const uint64_t start = nowSteadyNs();
#ifndef RB_SERVO_ENABLE_RBPODO
    return failedResult<RobotState>(
        BackendOp::Initialize,
        backendError(BackendErrorKind::DependencyUnavailable, "RbpodoBackend requested, but RB_SERVO_ENABLE_RBPODO=OFF"),
        makeBackendTiming(start, nowSteadyNs())
    );
#else
    if (!impl_->connected || !impl_->data_channel || !impl_->robot) {
        return failedResult<RobotState>(
            BackendOp::Initialize,
            backendError(BackendErrorKind::RobotDisconnected, "rbpodo backend is not connected"),
            makeBackendTiming(start, nowSteadyNs())
        );
    }
    // Arm the move_servo_j soft-entry gain ramp so the first servo_j after this
    // (re)initialize eases into the transparent gain instead of clunking.
    impl_->servo_engage_ramp_start_ns = 0;
    impl_->last_servo_j_send_ns = 0;
    try {
        // Reconcile the controller's pgmode (operation mode) with the config:
        // if the pendant/controller is in the other mode, switch it and VERIFY
        // the switch took effect before continuing. This makes `make run
        // MODE=real|sim` work regardless of what the pendant was left in.
        {
            const bool expected_simulation = expectedSimulationMode(impl_->config);
            const auto mode_state = impl_->data_channel->request_data(kDefaultStateTimeoutSec);
            if (mode_state) {
                const RbpodoSystemStateSnapshot mode_snapshot = snapshotFromSystemState(*mode_state);
                if (rbpodoModeFieldIsKnown(mode_snapshot.real_vs_simulation_mode) &&
                    !operationModeMatchesConfig(impl_->config, mode_snapshot)) {
                    std::cerr << "[INFO] RbpodoBackend switching controller pgmode to "
                              << rbpodoModeName(expected_simulation) << " for "
                              << impl_->config.name << " (controller reported "
                              << rbpodoModeName(mode_snapshot.real_vs_simulation_mode == 1) << ")\n";
                    rb::podo::ResponseCollector responses;
                    const auto ret = impl_->robot->set_operation_mode(
                        responses,
                        expected_simulation ? rb::podo::OperationMode::Simulation
                                            : rb::podo::OperationMode::Real,
                        kInitializeCommandAckTimeoutSec
                    );
                    if (impl_->config.disable_waiting_ack) {
                        rb::podo::ResponseCollector drained;
                        impl_->robot->flush(drained);
                    }
                    if (!ret.is_success()) {
                        return failedResult<RobotState>(
                            BackendOp::Initialize,
                            commandReturnError("set_operation_mode", ret, responses),
                            makeBackendTiming(start, nowSteadyNs())
                        );
                    }
                    // Verify by reading the CONTROLLER STATE back, per the SDK
                    // contract: set_operation_mode only ACKs receipt of the
                    // "pgmode" command (cobot.hpp), completion is reported by
                    // SD_PG_MODE (0=real / 1=simulation, data_type.hpp) and the
                    // activation stage SD_INIT_STATE_INFO (0..6, 6=done).
                    // simulation -> real powers up the servo stage, so this can
                    // legitimately take tens of seconds — we poll the state and
                    // log every observed change so progress is visible and a
                    // stuck controller is distinguishable from a slow one.
                    // Primary read: SD_PG_MODE system variable on the command
                    // channel (request/response). When ACK echoing is disabled
                    // (disable_waiting_ack) that query cannot complete, so we
                    // read the same documented fields from the data channel.
                    const auto probe_mode_and_stage = [this]() -> std::optional<RbpodoModeProbe> {
                        if (!impl_->config.disable_waiting_ack) {
                            const auto pg = querySystemVariableInt(
                                *impl_->robot, rb::podo::SystemVariable::SD_PG_MODE,
                                kInitializeCommandAckTimeoutSec);
                            if (pg) {
                                RbpodoModeProbe probe;
                                probe.pg_mode = *pg;
                                const auto stage = querySystemVariableInt(
                                    *impl_->robot, rb::podo::SystemVariable::SD_INIT_STATE_INFO,
                                    kInitializeCommandAckTimeoutSec);
                                probe.init_stage = stage ? (*stage & 0x3f) : -1;
                                return probe;
                            }
                        }
                        const auto state = impl_->data_channel->request_data(kDefaultStateTimeoutSec);
                        if (!state) return std::nullopt;
                        const RbpodoSystemStateSnapshot snapshot = snapshotFromSystemState(*state);
                        if (!rbpodoModeFieldIsKnown(snapshot.real_vs_simulation_mode)) return std::nullopt;
                        RbpodoModeProbe probe;
                        probe.pg_mode = snapshot.real_vs_simulation_mode;
                        probe.init_stage = snapshot.init_state_info & 0x3f;
                        return probe;
                    };
                    const int expected_pg_mode = expected_simulation ? 1 : 0;
                    bool confirmed = false;
                    RbpodoModeProbe last_logged{-2, -2};
                    RbpodoModeProbe last_seen{-2, -2};
                    const uint64_t verify_start_ns = nowSteadyNs();
                    const uint64_t verify_deadline_ns = verify_start_ns + kOperationModeSwitchTimeoutNs;
                    uint64_t next_heartbeat_ns = verify_start_ns + 10'000'000'000ULL;
                    while (nowSteadyNs() < verify_deadline_ns) {
                        if (shutdownRequested()) {
                            return failedResult<RobotState>(
                                BackendOp::Initialize,
                                backendError(
                                    BackendErrorKind::RobotNotInitialized,
                                    "rbpodo pgmode switch confirmation aborted by shutdown signal",
                                    "",
                                    "rbpodo_initialize_aborted_by_shutdown"
                                ),
                                makeBackendTiming(start, nowSteadyNs())
                            );
                        }
                        const auto probe = probe_mode_and_stage();
                        if (probe) {
                            last_seen = *probe;
                            if (probe->pg_mode == expected_pg_mode) {
                                confirmed = true;
                                break;
                            }
                            const bool changed = probe->pg_mode != last_logged.pg_mode ||
                                                 probe->init_stage != last_logged.init_stage;
                            if (changed || nowSteadyNs() >= next_heartbeat_ns) {
                                std::cerr << "[INFO] RbpodoBackend " << impl_->config.name
                                          << " controller reports pgmode="
                                          << rbpodoPgModeLabel(probe->pg_mode)
                                          << " activation_stage=" << probe->init_stage << "/6"
                                          << " (waiting for pgmode="
                                          << rbpodoModeName(expected_simulation) << ", "
                                          << (nowSteadyNs() - verify_start_ns) / 1'000'000'000.0
                                          << " s)\n";
                                last_logged = *probe;
                                next_heartbeat_ns = nowSteadyNs() + 10'000'000'000ULL;
                            }
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    }
                    if (!confirmed) {
                        return failedResult<RobotState>(
                            BackendOp::Initialize,
                            backendError(
                                BackendErrorKind::WrongMode,
                                "rbpodo pgmode switch to " + rbpodoModeName(expected_simulation) +
                                    " not confirmed within " +
                                    std::to_string(kOperationModeSwitchTimeoutNs / 1'000'000'000ULL) +
                                    " s (controller last reported pgmode=" +
                                    rbpodoPgModeLabel(last_seen.pg_mode) + ", activation_stage=" +
                                    std::to_string(last_seen.init_stage) + "/6)",
                                "",
                                "rbpodo_operation_mode_switch_unconfirmed"
                            ),
                            makeBackendTiming(start, nowSteadyNs())
                        );
                    }
                    std::cerr << "[INFO] RbpodoBackend controller pgmode confirmed "
                              << rbpodoModeName(expected_simulation) << " for "
                              << impl_->config.name << " in "
                              << (nowSteadyNs() - verify_start_ns) / 1'000'000'000.0 << " s\n";
                }
            }
        }
        // Auto-activate the robot (mc jall init) when the controller reports the
        // servo stage not ready — covers fresh control-box (VM) boots and cold
        // bring-ups so `make run` needs no pendant interaction. Verified by
        // polling until the activation stage reaches servo_enabled.
        {
            const auto act_probe = impl_->data_channel->request_data(kDefaultStateTimeoutSec);
            if (act_probe) {
                const RbpodoSystemStateSnapshot probe_snapshot = snapshotFromSystemState(*act_probe);
                const RobotState probe_mapped =
                    mapRbpodoSystemStateSnapshot(impl_->arm_id, probe_snapshot, impl_->config);
                if (!probe_mapped.has_error && !probe_mapped.servo_enabled) {
                    std::cerr << "[INFO] RbpodoBackend activating robot (mc jall init) for "
                              << impl_->config.name << " (activation stage "
                              << probe_snapshot.init_state_info << ")\n";
                    rb::podo::ResponseCollector responses;
                    const auto ret = impl_->robot->activate(
                        responses, kInitializeCommandAckTimeoutSec, true);
                    if (impl_->config.disable_waiting_ack) {
                        rb::podo::ResponseCollector drained;
                        impl_->robot->flush(drained);
                    }
                    (void)ret;  // activation progress is judged by the state poll below
                    bool activated = false;
                    int last_act_stage = -2;
                    const uint64_t act_start_ns = nowSteadyNs();
                    const uint64_t act_deadline_ns = act_start_ns + 60'000'000'000ULL;
                    uint64_t act_progress_ns = act_start_ns + 10'000'000'000ULL;
                    while (nowSteadyNs() < act_deadline_ns) {
                        if (shutdownRequested()) {
                            return failedResult<RobotState>(
                                BackendOp::Initialize,
                                backendError(
                                    BackendErrorKind::RobotNotInitialized,
                                    "rbpodo activation confirmation aborted by shutdown signal",
                                    "",
                                    "rbpodo_initialize_aborted_by_shutdown"
                                ),
                                makeBackendTiming(start, nowSteadyNs())
                            );
                        }
                        const auto verify = impl_->data_channel->request_data(kDefaultStateTimeoutSec);
                        int observed_stage = -1;
                        if (verify) {
                            const RbpodoSystemStateSnapshot vs = snapshotFromSystemState(*verify);
                            const RobotState vm = mapRbpodoSystemStateSnapshot(impl_->arm_id, vs, impl_->config);
                            observed_stage = vs.init_state_info & 0x3f;
                            if (vm.servo_enabled) {
                                activated = true;
                                break;
                            }
                            if (vm.has_error) break;  // activation fault: report below
                        }
                        // Log on activation-stage CHANGE (SD_INIT_STATE_INFO
                        // 0..6, 6=done) plus a periodic heartbeat, so the
                        // operator sees real controller progress.
                        if (observed_stage != last_act_stage || nowSteadyNs() >= act_progress_ns) {
                            std::cerr << "[INFO] RbpodoBackend activation_stage="
                                      << observed_stage << "/6 on "
                                      << impl_->config.name << " ("
                                      << (nowSteadyNs() - act_start_ns) / 1'000'000'000.0
                                      << " s)\n";
                            last_act_stage = observed_stage;
                            act_progress_ns = nowSteadyNs() + 10'000'000'000ULL;
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    }
                    if (activated) {
                        std::cerr << "[INFO] RbpodoBackend robot activated for "
                                  << impl_->config.name << " in "
                                  << (nowSteadyNs() - act_start_ns) / 1'000'000'000.0 << " s\n";
                    } else {
                        std::cerr << "[WARN] RbpodoBackend activation not confirmed for "
                                  << impl_->config.name
                                  << "; continuing — startup validation will report the state\n";
                    }
                }
            }
        }
        // Apply the configured overall speed bar (controller UI bottom bar) so every
        // bring-up starts from a known speed instead of whatever the pendant last had.
        {
            rb::podo::ResponseCollector responses;
            const auto ret = impl_->robot->set_speed_bar(
                responses,
                impl_->config.speed_bar,
                kInitializeCommandAckTimeoutSec
            );
            if (impl_->config.disable_waiting_ack) {
                rb::podo::ResponseCollector drained;
                impl_->robot->flush(drained);
            }
            if (!ret.is_success()) {
                return failedResult<RobotState>(
                    BackendOp::Initialize,
                    commandReturnError("set_speed_bar", ret, responses),
                    makeBackendTiming(start, nowSteadyNs())
                );
            }
            std::cerr << "[INFO] RbpodoBackend applied speed_bar=" << impl_->config.speed_bar
                      << " for " << impl_->config.name << "\n";
        }
        const auto state = impl_->data_channel->request_data(kDefaultStateTimeoutSec);
        if (!state) {
            std::cerr << "[ERROR] RbpodoBackend initialize failed: no state from "
                      << impl_->config.ip << "\n";
            return failedResult<RobotState>(
                BackendOp::Initialize,
                backendError(
                    BackendErrorKind::TransportReadFailed,
                    "rbpodo initialize failed: no state",
                    "",
                    "rbpodo_initialize_state_unavailable"
                ),
                makeBackendTiming(start, nowSteadyNs())
            );
        }

        const RbpodoSystemStateSnapshot snapshot = snapshotFromSystemState(*state);
        RobotState mapped = mapRbpodoSystemStateSnapshot(impl_->arm_id, snapshot, impl_->config);
        impl_->last_state_error = rbpodoMotionReadinessError(impl_->config, snapshot, mapped);
        attachMotionReadinessDiagnostic(&mapped, impl_->last_state_error);
        impl_->last_state_cache = mapped.has_valid_joint_state ? std::make_optional(mapped) : std::nullopt;
        if (const auto acquisition_error = rbpodoStateAcquisitionError(mapped)) {
            return failedResultWithValue(
                BackendOp::Initialize,
                *acquisition_error,
                mapped,
                makeBackendTiming(start, nowSteadyNs())
            );
        }
        return okResult(BackendOp::Initialize, mapped, makeBackendTiming(start, nowSteadyNs()));
    } catch (const std::exception& exc) {
        std::cerr << "[ERROR] RbpodoBackend initialize failed for " << impl_->config.name
                  << ": " << exc.what() << "\n";
        return failedResult<RobotState>(
            BackendOp::Initialize,
            backendError(
                BackendErrorKind::TransportReadFailed,
                exc.what(),
                "",
                "rbpodo_initialize_exception"
            ),
            makeBackendTiming(start, nowSteadyNs())
        );
    }
#endif
}

BackendResult<RobotState> RbpodoBackend::readState() {
    const uint64_t start = nowSteadyNs();
#ifndef RB_SERVO_ENABLE_RBPODO
    return failedResult<RobotState>(
        BackendOp::ReadState,
        backendError(BackendErrorKind::DependencyUnavailable, "RbpodoBackend requested, but RB_SERVO_ENABLE_RBPODO=OFF"),
        makeBackendTiming(start, nowSteadyNs())
    );
#else
    RobotState out_state = basicState(impl_->arm_id, impl_->connected);
    out_state.arm_id = impl_->arm_id;
    out_state.host_time_ns = nowSteadyNs();
    out_state.connection_state = impl_->connected
        ? RobotConnectionState::Connected
        : RobotConnectionState::Disconnected;
    out_state.has_valid_joint_state = false;
    if (!impl_->connected || !impl_->data_channel) {
        return failedResult<RobotState>(
            BackendOp::ReadState,
            backendError(BackendErrorKind::RobotDisconnected, "rbpodo backend is not connected"),
            makeBackendTiming(start, nowSteadyNs())
        );
    }

    try {
        const auto state = impl_->data_channel->request_data(kDefaultStateTimeoutSec);
        if (!state) {
            // Controller-simulation read-miss carve-out: a single missing CobotData
            // frame is treated as a transient gap (hold the last valid state and stay
            // connected) for up to max_consecutive_read_misses consecutive misses. A
            // sustained outage still trips after that many. Physical real operation
            // always fails closed here (carve-out gated on simulation operation_mode).
            if (expectedSimulationMode(impl_->config) &&
                impl_->config.max_consecutive_read_misses > 0 &&
                impl_->consecutive_read_misses < impl_->config.max_consecutive_read_misses &&
                impl_->last_state_cache.has_value()) {
                ++impl_->consecutive_read_misses;
                RobotState held = *impl_->last_state_cache;
                held.arm_id = impl_->arm_id;
                held.host_time_ns = nowSteadyNs();
                held.connection_state = RobotConnectionState::Connected;
                held.rbpodo_sdk_state_source = "last_state_cache (read-miss hold)";
                std::cerr << "[WARN] RbpodoBackend readState transient miss for "
                          << impl_->config.name << "; holding last state ("
                          << impl_->consecutive_read_misses << "/"
                          << impl_->config.max_consecutive_read_misses << ")\n";
                return okResult(BackendOp::ReadState, held, makeBackendTiming(start, nowSteadyNs()));
            }
            out_state.connection_state = RobotConnectionState::Disconnected;
            impl_->connected = false;
            return failedResult<RobotState>(
                BackendOp::ReadState,
                backendError(
                    BackendErrorKind::TransportReadFailed,
                    "rbpodo readState returned no state",
                    "",
                    "rbpodo_read_state_unavailable"
                ),
                makeBackendTiming(start, nowSteadyNs())
            );
        }
        impl_->consecutive_read_misses = 0;
        const RbpodoSystemStateSnapshot snapshot = snapshotFromSystemState(*state);
        out_state = mapRbpodoSystemStateSnapshot(impl_->arm_id, snapshot, impl_->config);
        impl_->last_state_error = rbpodoMotionReadinessError(impl_->config, snapshot, out_state);
        attachMotionReadinessDiagnostic(&out_state, impl_->last_state_error);
        impl_->last_state_cache = out_state.has_valid_joint_state ? std::make_optional(out_state) : std::nullopt;
        if (const auto acquisition_error = rbpodoStateAcquisitionError(out_state)) {
            return failedResultWithValue(
                BackendOp::ReadState,
                *acquisition_error,
                out_state,
                makeBackendTiming(start, nowSteadyNs())
            );
        }
        return okResult(BackendOp::ReadState, out_state, makeBackendTiming(start, nowSteadyNs()));
    } catch (const std::exception& exc) {
        std::cerr << "[WARN] RbpodoBackend readState failed for " << impl_->config.name
                  << ": " << exc.what() << "\n";
        out_state.connection_state = RobotConnectionState::Error;
        out_state.has_error = true;
        out_state.error_code = -1;
        return failedResult<RobotState>(
            BackendOp::ReadState,
            backendError(
                BackendErrorKind::TransportReadFailed,
                exc.what(),
                "",
                "rbpodo_read_exception"
            ),
            makeBackendTiming(start, nowSteadyNs())
        );
    }
#endif
}

SendServoJResult RbpodoBackend::sendServoJ(const SendServoJRequest& request) {
    const uint64_t start = nowSteadyNs();
#ifndef RB_SERVO_ENABLE_RBPODO
    return rejectedSend(
        request,
        backendError(BackendErrorKind::DependencyUnavailable, "RbpodoBackend requested, but RB_SERVO_ENABLE_RBPODO=OFF"),
        makeBackendTiming(start, nowSteadyNs())
    );
#else
    const std::optional<RobotState> cached_state = recentStateCache(impl_->last_state_cache, nowSteadyNs());
    const char* cached_source = cached_state.has_value() ? "cache" : "none";
    const auto with_ack_metadata = [&](SendServoJResult result, bool ack_observed, double ack_wait_duration_us) {
        annotateRbpodoAckResult(&result, impl_->config, ack_wait_duration_us, ack_observed);
        return result;
    };
    if (!impl_->connected || !impl_->robot) {
        return with_ack_metadata(
            rejectedSend(
                request,
                backendError(BackendErrorKind::RobotDisconnected, "rbpodo backend is not connected"),
                makeBackendTiming(start, nowSteadyNs()),
                cached_state,
                cached_source
            ),
            false,
            0.0
        );
    }
    if (impl_->freedrive_active) {
        // Defensive backstop: never stream move_servo_j to a controller that is
        // in freedrive_teach_on. Policy suppression, not a fault (the fault
        // classifier treats SuppressedByPolicy as non-latching).
        return with_ack_metadata(
            rejectedSend(
                request,
                backendError(
                    BackendErrorKind::SuppressedByPolicy,
                    "servo_j suppressed while controller is in freedrive (direct teaching)",
                    "",
                    "rbpodo_freedrive_active"
                ),
                makeBackendTiming(start, nowSteadyNs()),
                cached_state,
                cached_source
            ),
            false,
            0.0
        );
    }
    if (!finiteJointArray(request.q_target_deg)) {
        std::cerr << "[ERROR] RbpodoBackend refused non-finite servo_j target\n";
        return with_ack_metadata(
            rejectedSend(
                request,
                backendError(
                    BackendErrorKind::InvalidTarget,
                    "RbpodoBackend refused non-finite servo_j target",
                    "",
                    "rbpodo_invalid_servo_j_target"
                ),
                makeBackendTiming(start, nowSteadyNs()),
                cached_state,
                cached_source
            ),
            false,
            0.0
        );
    }
    if (cached_state.has_value() &&
        impl_->last_state_error.has_value() &&
        !controllerSimulationReadinessOverrideAllowed(
            impl_->config,
            *impl_->last_state_error
        )) {
        return with_ack_metadata(
            rejectedSend(
                request,
                *impl_->last_state_error,
                makeBackendTiming(start, nowSteadyNs()),
                cached_state,
                "cache"
            ),
            false,
            0.0
        );
    }
    if (!cached_state.has_value() &&
        impl_->last_state_error.has_value() &&
        !controllerSimulationReadinessOverrideAllowed(
            impl_->config,
            *impl_->last_state_error
        )) {
        return with_ack_metadata(
            rejectedSend(
                request,
                *impl_->last_state_error,
                makeBackendTiming(start, nowSteadyNs()),
                std::nullopt,
                "none"
            ),
            false,
            0.0
        );
    }

    try {
        // Soft-entry gain ramp on RT-servo (re)engagement (see BackendConfig
        // docs): spread the one-tick stiffen / backlash take-up over
        // servo_soft_entry_sec so the controller eases into the transparent gain
        // instead of clunking. Transient only — steady state is always
        // impl_->config.servo_gain. Re-arms after any stream gap.
        double servo_gain_effective = impl_->config.servo_gain;
        if (impl_->config.servo_soft_entry_enable && impl_->config.servo_soft_entry_sec > 0.0) {
            const uint64_t now_ramp_ns = nowSteadyNs();
            const uint64_t rearm_gap_ns =
                static_cast<uint64_t>(impl_->config.servo_soft_entry_rearm_gap_sec * 1e9);
            const bool rearm =
                impl_->servo_engage_ramp_start_ns == 0 ||
                (impl_->last_servo_j_send_ns != 0 &&
                 now_ramp_ns - impl_->last_servo_j_send_ns > rearm_gap_ns);
            if (rearm) {
                impl_->servo_engage_ramp_start_ns = now_ramp_ns;
                std::cerr << "[INFO] RbpodoBackend servo_j soft-entry gain ramp armed ("
                          << impl_->config.servo_soft_entry_sec << " s, start_scale="
                          << impl_->config.servo_soft_entry_gain_start_scale << ") for "
                          << impl_->config.name << "\n";
            }
            double progress =
                static_cast<double>(now_ramp_ns - impl_->servo_engage_ramp_start_ns) / 1e9 /
                impl_->config.servo_soft_entry_sec;
            if (progress < 0.0) progress = 0.0;
            if (progress > 1.0) progress = 1.0;
            const double gain_start =
                impl_->config.servo_gain * impl_->config.servo_soft_entry_gain_start_scale;
            servo_gain_effective = gain_start + (impl_->config.servo_gain - gain_start) * progress;
            impl_->last_servo_j_send_ns = now_ramp_ns;
        }
        rb::podo::ResponseCollector responses;
        const uint64_t ack_start = nowSteadyNs();
        const auto ret = impl_->robot->move_servo_j(
            responses,
            request.q_target_deg,
            impl_->config.servo_t1_sec,
            impl_->config.servo_t2_sec,
            servo_gain_effective,
            impl_->config.servo_alpha,
            impl_->config.command_timeout_sec,
            true
        );
        const uint64_t ack_end = nowSteadyNs();
        // ACK-disabled streaming leak fix: the SDK's disable_waiting_ack only
        // flips a client-side flag (waiting_ack=false) — it does NOT tell the
        // controller to stop sending a response per command. move_servo_j then
        // returns without reading the command socket, and state is read on a
        // separate CobotData channel, so the command socket is NEVER drained.
        // Over a long run the controller's per-command responses pile up unread
        // in the command-socket receive buffer until it backs up and the command
        // channel corrupts (~10-20 min: controller "parsing error", teaching
        // pendant link drops, GUI ArmMotion/init_motion-profile requests ignored). Drain it every
        // cycle: flush() is a non-blocking read-until-empty. We discard the
        // drained responses (this mode already ignores ACKs), so accept/reject
        // below is unchanged and the streamed command bytes are identical — safe
        // for the real path too. No-op when ACK waiting is enabled (move_servo_j
        // already drains via wait_until_ack_message).
        if (impl_->config.disable_waiting_ack) {
            rb::podo::ResponseCollector drained;
            impl_->robot->flush(drained);
        }
        const double ack_wait_duration_us =
            impl_->config.disable_waiting_ack ? 0.0 : makeBackendTiming(ack_start, ack_end).duration_us;
        if (!ret.is_success()) {
            std::cerr << "[WARN] RbpodoBackend move_servo_j was not accepted for "
                      << impl_->config.name << "\n";
            return with_ack_metadata(
                rejectedSend(
                    request,
                    commandReturnError("move_servo_j", ret, responses),
                    makeBackendTiming(start, nowSteadyNs()),
                    cached_state,
                    cached_source
                ),
                responses.has_error(),
                ack_wait_duration_us
            );
        }
        if (responses.has_error()) {
            std::cerr << "[WARN] RbpodoBackend move_servo_j response contained an error for "
                      << impl_->config.name << ": " << responses << "\n";
            return with_ack_metadata(
                rejectedSend(
                    request,
                    controllerResponseError("move_servo_j", responses),
                    makeBackendTiming(start, nowSteadyNs()),
                    cached_state,
                    cached_source
                ),
                !impl_->config.disable_waiting_ack,
                ack_wait_duration_us
            );
        }
        return with_ack_metadata(
            acceptedSend(request, makeBackendTiming(start, nowSteadyNs())),
            !impl_->config.disable_waiting_ack,
            ack_wait_duration_us
        );
    } catch (const std::exception& exc) {
        std::cerr << "[WARN] RbpodoBackend move_servo_j failed for " << impl_->config.name
                  << ": " << exc.what() << "\n";
        return with_ack_metadata(
            rejectedSend(
                request,
                backendError(
                    BackendErrorKind::TransportWriteFailed,
                    exc.what(),
                    "",
                    "rbpodo_move_servo_j_exception"
                ),
                makeBackendTiming(start, nowSteadyNs()),
                cached_state,
                cached_source
            ),
            false,
            0.0
        );
    }
#endif
}

BackendResult<RobotState> RbpodoBackend::stop() {
    const uint64_t start = nowSteadyNs();
#ifndef RB_SERVO_ENABLE_RBPODO
    return failedResult<RobotState>(
        BackendOp::Stop,
        backendError(BackendErrorKind::DependencyUnavailable, "RbpodoBackend requested, but RB_SERVO_ENABLE_RBPODO=OFF"),
        makeBackendTiming(start, nowSteadyNs())
    );
#else
    // No verified controller-level hold/stop API is wired for MIG-04.
    // task_stop exists in rbpodo, but it stops task programs and is not treated
    // here as a safe servo hold primitive.
    return failedResult<RobotState>(
        BackendOp::Stop,
        backendError(
            BackendErrorKind::DependencyUnavailable,
            "No verified rbpodo controller-level stop/hold API is wired; operator intervention is required to stop or hold the real robot safely.",
            "",
            "rbpodo_stop_unverified"
        ),
        makeBackendTiming(start, nowSteadyNs())
    );
#endif
}

BackendResult<RobotState> RbpodoBackend::resetFault() {
    const uint64_t start = nowSteadyNs();
#ifndef RB_SERVO_ENABLE_RBPODO
    return failedResult<RobotState>(
        BackendOp::ResetFault,
        backendError(BackendErrorKind::DependencyUnavailable, "RbpodoBackend requested, but RB_SERVO_ENABLE_RBPODO=OFF"),
        makeBackendTiming(start, nowSteadyNs())
    );
#else
    // Controller-simulation carve-out: an rbpodo controller in pgmode simulation
    // (operation_mode=simulation, physical_motion_expected=false) has no physical
    // robot to endanger, so a fault reset is safe and is required to recover from
    // a server-side EmergencyStop/soft-estop latch without restarting the server.
    // Gated by operation_mode=simulation and live controller data.
    // Physical real mode keeps the conservative refusal below: no verified rbpodo
    // fault-reset API exists, so motion must remain disabled after any reset.
    if (expectedSimulationMode(impl_->config) &&
        impl_->connected && impl_->data_channel) {
        try {
            const auto state = impl_->data_channel->request_data(kDefaultStateTimeoutSec);
            if (state) {
                const RbpodoSystemStateSnapshot snapshot = snapshotFromSystemState(*state);
                RobotState mapped = mapRbpodoSystemStateSnapshot(impl_->arm_id, snapshot, impl_->config);
                return okResult(BackendOp::ResetFault, mapped, makeBackendTiming(start, nowSteadyNs()));
            }
        } catch (const std::exception&) {
            // fall through to the conservative refusal below
        }
    }
    // No verified fault-reset API is exposed by the inspected rbpodo headers.
    return failedResult<RobotState>(
        BackendOp::ResetFault,
        backendError(
            BackendErrorKind::DependencyUnavailable,
            "No verified rbpodo fault-reset API is exposed; operator intervention is required and motion must remain disabled after any external reset.",
            "",
            "rbpodo_reset_fault_unverified"
        ),
        makeBackendTiming(start, nowSteadyNs())
    );
#endif
}

BackendResult<RobotState> RbpodoBackend::setFreedrive(bool on) {
    const uint64_t start = nowSteadyNs();
#ifndef RB_SERVO_ENABLE_RBPODO
    return failedResult<RobotState>(
        BackendOp::SetFreedrive,
        backendError(BackendErrorKind::DependencyUnavailable, "RbpodoBackend requested, but RB_SERVO_ENABLE_RBPODO=OFF"),
        makeBackendTiming(start, nowSteadyNs())
    );
#else
    if (!impl_->connected || !impl_->robot) {
        return failedResult<RobotState>(
            BackendOp::SetFreedrive,
            backendError(BackendErrorKind::RobotDisconnected, "rbpodo backend is not connected"),
            makeBackendTiming(start, nowSteadyNs())
        );
    }
    // Maps to the rbpodo SDK wrapper (cobot.hpp set_freedrive_mode →
    // freedrive_teach_on()/freedrive_teach_off()). set_freedrive_mode only ACKs
    // receipt of the script command; the server treats the controller's gravity-
    // compensation hand-guiding as operator-supervised, not server-verified.
    rb::podo::ResponseCollector responses;
    const auto ret = impl_->robot->set_freedrive_mode(responses, on, kInitializeCommandAckTimeoutSec);
    if (impl_->config.disable_waiting_ack) {
        rb::podo::ResponseCollector drained;
        impl_->robot->flush(drained);
    }
    if (!ret.is_success()) {
        return failedResult<RobotState>(
            BackendOp::SetFreedrive,
            commandReturnError(on ? "freedrive_teach_on" : "freedrive_teach_off", ret, responses),
            makeBackendTiming(start, nowSteadyNs())
        );
    }
    impl_->freedrive_active = on;
    std::cerr << "[INFO] RbpodoBackend " << impl_->config.name
              << (on ? " entered freedrive (direct teaching) — servo_j suppressed"
                     : " exited freedrive (direct teaching) — servo_j re-enabled")
              << "\n";
    // Best-effort fresh state read so the caller can resync its held target to
    // the (possibly hand-moved) actual joints on exit.
    RobotState state_after;
    if (impl_->data_channel) {
        try {
            const auto state = impl_->data_channel->request_data(kDefaultStateTimeoutSec);
            if (state) {
                const RbpodoSystemStateSnapshot snapshot = snapshotFromSystemState(*state);
                state_after = mapRbpodoSystemStateSnapshot(impl_->arm_id, snapshot, impl_->config);
                if (state_after.has_valid_joint_state) {
                    impl_->last_state_cache = state_after;
                }
            }
        } catch (const std::exception&) {
            // Non-fatal: the server falls back to its own latest readState() for resync.
        }
    }
    return okResult(BackendOp::SetFreedrive, state_after, makeBackendTiming(start, nowSteadyNs()));
#endif
}

bool RbpodoBackend::isConnected() const {
    return impl_->connected;
}

ArmId RbpodoBackend::armId() const {
    return impl_->arm_id;
}

std::string RbpodoBackend::name() const {
    return impl_->config.name;
}

}  // namespace rb_servo
