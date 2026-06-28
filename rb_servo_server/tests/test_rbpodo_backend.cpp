#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "rb_servo/config/config.hpp"
#include "rb_servo/network/state_publisher.hpp"
#include "rb_servo/robot/backend_result.hpp"
#include "rb_servo/robot/rbpodo_backend.hpp"

namespace {

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

bool contains(const std::string& text, const std::string& needle) {
    return text.find(needle) != std::string::npos;
}

bool containsField(const std::vector<std::string>& fields, const std::string& needle) {
    return std::find(fields.begin(), fields.end(), needle) != fields.end();
}

class EnvVarGuard {
public:
    explicit EnvVarGuard(const char* name)
        : name_(name) {
        const char* value = std::getenv(name_.c_str());
        if (value) {
            had_value_ = true;
            old_value_ = value;
        }
    }

    ~EnvVarGuard() {
        if (had_value_) {
            setenv(name_.c_str(), old_value_.c_str(), 1);
        } else {
            unsetenv(name_.c_str());
        }
    }

    void set(const char* value) const { setenv(name_.c_str(), value, 1); }
    void unset() const { unsetenv(name_.c_str()); }

private:
    std::string name_;
    bool had_value_ = false;
    std::string old_value_;
};

rb_servo::JointArray joints(double value) {
    rb_servo::JointArray out{};
    out.fill(value);
    return out;
}

rb_servo::BackendConfig rbpodoConfig(std::string operation_mode = "real") {
    rb_servo::BackendConfig config;
    config.backend_type = rb_servo::BackendType::Rbpodo;
    config.run_mode = rb_servo::RunMode::Real;
    config.operation_mode = std::move(operation_mode);
    return config;
}

rb_servo::RbpodoSystemStateSnapshot rbpodoSnapshot() {
    rb_servo::RbpodoSystemStateSnapshot snapshot;
    snapshot.q_actual_deg = {1.0, -2.0, 3.0, -4.0, 5.0, -6.0};
    snapshot.q_target_deg = snapshot.q_actual_deg;
    snapshot.robot_time_sec = 12.5;
    snapshot.real_vs_simulation_mode = 0;
    snapshot.init_state_info = 6;
    return snapshot;
}

bool testFreedriveControllerSignalsMapped() {
    rb_servo::RbpodoSystemStateSnapshot snapshot = rbpodoSnapshot();
    // Controller actively executing motion, free-drive off.
    snapshot.robot_state = 3;
    snapshot.is_freedrive_mode = 0;
    rb_servo::RobotState moving =
        rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Left, snapshot);
    RB_CHECK(moving.controller_motion_state == 3);
    RB_CHECK(!moving.controller_freedrive_on);

    // Controller idle, free-drive engaged.
    snapshot.robot_state = 1;
    snapshot.is_freedrive_mode = 1;
    rb_servo::RobotState idle_freedrive =
        rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Right, snapshot);
    RB_CHECK(idle_freedrive.controller_motion_state == 1);
    RB_CHECK(idle_freedrive.controller_freedrive_on);
    return true;
}

bool testClearSelfCollisionIsRobotFault() {
    rb_servo::RbpodoSystemStateSnapshot snapshot = rbpodoSnapshot();
    snapshot.op_stat_self_collision = 1;

    const rb_servo::RobotState state =
        rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Left, snapshot);
    RB_CHECK(state.has_valid_joint_state);
    RB_CHECK(state.has_error);
    RB_CHECK(state.lifecycle_state == "faulted");
    RB_CHECK(state.diagnostic_error_source == "rbpodo_self_collision");
    RB_CHECK(state.rbpodo_diagnostics.has_value());
    RB_CHECK(state.rbpodo_diagnostics->diagnostics_valid);
    RB_CHECK(!state.rbpodo_diagnostics->diagnostics_suspect);
    RB_CHECK(state.rbpodo_diagnostics->raw.op_stat_self_collision == 1);
    RB_CHECK(!rb_servo::rbpodoStateAcquisitionError(state).has_value());

    const std::optional<rb_servo::BackendError> readiness =
        rb_servo::rbpodoMotionReadinessError(rbpodoConfig(), snapshot, state);
    RB_CHECK(readiness.has_value());
    RB_CHECK(readiness->kind == rb_servo::BackendErrorKind::RobotFault);
    RB_CHECK(readiness->name == "rbpodo_self_collision");
    return true;
}

// op_stat_self_collision is bit-packed: bit0-1 are the flag, the upper bits are
// reserved/undefined. 0x76904aa0 (= 1989167776) is a value observed live on real
// hardware: its low bits are 0, so the field is HEALTHY and must NOT be flagged as
// garbage/suspect once masked. (Pre-fix this whole int was checked against 0/1 and
// latched a false -2001 diagnostics_suspect.)
bool testSelfCollisionUpperBitsAreMaskedToHealthy() {
    rb_servo::RbpodoSystemStateSnapshot snapshot = rbpodoSnapshot();
    snapshot.op_stat_self_collision = 1989167776;  // 0x76904aa0, bit0 = 0 -> no self-collision

    const rb_servo::RobotState state =
        rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Right, snapshot);
    RB_CHECK(state.has_valid_joint_state);
    RB_CHECK(!state.has_error);
    RB_CHECK(state.lifecycle_state == "servo_enabled");
    RB_CHECK(state.rbpodo_diagnostics.has_value());
    RB_CHECK(state.rbpodo_diagnostics->diagnostics_valid);
    RB_CHECK(!state.rbpodo_diagnostics->diagnostics_suspect);
    // The raw value is preserved verbatim for telemetry/transparency.
    RB_CHECK(state.rbpodo_diagnostics->raw.op_stat_self_collision == 1989167776);
    RB_CHECK(!rb_servo::rbpodoStateAcquisitionError(state).has_value());
    RB_CHECK(!rb_servo::rbpodoMotionReadinessError(rbpodoConfig(), snapshot, state).has_value());
    return true;
}

// Safety-critical: a REAL self-collision (bit0 = 1) must still latch even when the
// reserved upper bits are non-zero. Pre-fix the `== 1` test silently missed this.
bool testRealSelfCollisionFaultsThroughGarbageUpperBits() {
    rb_servo::RbpodoSystemStateSnapshot snapshot = rbpodoSnapshot();
    snapshot.op_stat_self_collision = 1989167777;  // 0x76904aa1, bit0 = 1 -> self-collision

    const rb_servo::RobotState state =
        rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Left, snapshot);
    RB_CHECK(state.has_valid_joint_state);
    RB_CHECK(state.has_error);
    RB_CHECK(state.lifecycle_state == "faulted");
    RB_CHECK(state.diagnostic_error_source == "rbpodo_self_collision");
    RB_CHECK(state.rbpodo_diagnostics.has_value());
    RB_CHECK(state.rbpodo_diagnostics->diagnostics_valid);
    RB_CHECK(!state.rbpodo_diagnostics->diagnostics_suspect);
    RB_CHECK(state.rbpodo_diagnostics->raw.op_stat_self_collision == 1989167777);

    const std::optional<rb_servo::BackendError> readiness =
        rb_servo::rbpodoMotionReadinessError(rbpodoConfig(), snapshot, state);
    RB_CHECK(readiness.has_value());
    RB_CHECK(readiness->kind == rb_servo::BackendErrorKind::RobotFault);
    RB_CHECK(readiness->name == "rbpodo_self_collision");
    return true;
}

// A device code (sos/ems) carried in the low 6 bits must still be interpreted as the
// real fault code even with reserved upper bits set.
bool testSosCodeMaskedThroughGarbageUpperBits() {
    rb_servo::RbpodoSystemStateSnapshot snapshot = rbpodoSnapshot();
    snapshot.op_stat_sos_flag = static_cast<int>(0x76900000) | 5;  // upper garbage + code 5

    const rb_servo::RobotState state =
        rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Left, snapshot);
    RB_CHECK(state.has_error);
    RB_CHECK(state.error_code == 5);
    RB_CHECK(state.lifecycle_state == "faulted");
    RB_CHECK(state.diagnostic_error_source == "rbpodo_sos_flag");
    RB_CHECK(state.rbpodo_diagnostics.has_value());
    RB_CHECK(!state.rbpodo_diagnostics->diagnostics_suspect);
    return true;
}

bool testValidSosCodeIsDeviceFaultNotBooleanViolation() {
    rb_servo::RbpodoSystemStateSnapshot snapshot = rbpodoSnapshot();
    snapshot.op_stat_sos_flag = 2;

    const rb_servo::RobotState state =
        rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Left, snapshot);
    RB_CHECK(state.has_valid_joint_state);
    RB_CHECK(state.has_error);
    RB_CHECK(state.error_code == 2);
    RB_CHECK(state.lifecycle_state == "faulted");
    RB_CHECK(state.diagnostic_error_source == "rbpodo_sos_flag");
    RB_CHECK(state.rbpodo_diagnostics.has_value());
    RB_CHECK(state.rbpodo_diagnostics->diagnostics_valid);
    RB_CHECK(!state.rbpodo_diagnostics->diagnostics_suspect);
    RB_CHECK(!contains(state.rbpodo_diagnostics->reason, "expected 0/1"));
    RB_CHECK(state.rbpodo_diagnostics->raw.op_stat_sos_flag == 2);

    const std::optional<rb_servo::BackendError> readiness =
        rb_servo::rbpodoMotionReadinessError(rbpodoConfig(), snapshot, state);
    RB_CHECK(readiness.has_value());
    RB_CHECK(readiness->kind == rb_servo::BackendErrorKind::RobotFault);
    RB_CHECK(readiness->name == "rbpodo_sos_flag");
    return true;
}

bool testValidEmsCodeIsDeviceFaultNotBooleanViolation() {
    rb_servo::RbpodoSystemStateSnapshot snapshot = rbpodoSnapshot();
    snapshot.op_stat_ems_flag = 1;

    const rb_servo::RobotState state =
        rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Right, snapshot);
    RB_CHECK(state.has_valid_joint_state);
    RB_CHECK(state.has_error);
    RB_CHECK(state.error_code == 1);
    RB_CHECK(state.lifecycle_state == "faulted");
    RB_CHECK(state.diagnostic_error_source == "rbpodo_ems_flag");
    RB_CHECK(state.rbpodo_diagnostics.has_value());
    RB_CHECK(state.rbpodo_diagnostics->diagnostics_valid);
    RB_CHECK(!state.rbpodo_diagnostics->diagnostics_suspect);
    RB_CHECK(!contains(state.rbpodo_diagnostics->reason, "expected 0/1"));
    RB_CHECK(state.rbpodo_diagnostics->raw.op_stat_ems_flag == 1);

    const std::optional<rb_servo::BackendError> readiness =
        rb_servo::rbpodoMotionReadinessError(rbpodoConfig(), snapshot, state);
    RB_CHECK(readiness.has_value());
    RB_CHECK(readiness->kind == rb_servo::BackendErrorKind::RobotFault);
    RB_CHECK(readiness->name == "rbpodo_ems_flag");
    return true;
}

bool testOutOfRangeSosAndEmsCodesAreSuspect() {
    {
        rb_servo::RbpodoSystemStateSnapshot snapshot = rbpodoSnapshot();
        snapshot.op_stat_sos_flag = 13;

        const rb_servo::RobotState state =
            rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Left, snapshot);
        RB_CHECK(state.has_valid_joint_state);
        RB_CHECK(state.has_error);
        RB_CHECK(state.error_code != snapshot.op_stat_sos_flag);
        RB_CHECK(state.lifecycle_state == "diagnostics_suspect");
        RB_CHECK(state.diagnostic_error_source == "rbpodo_diagnostics_suspect");
        RB_CHECK(state.rbpodo_diagnostics.has_value());
        RB_CHECK(!state.rbpodo_diagnostics->diagnostics_valid);
        RB_CHECK(state.rbpodo_diagnostics->diagnostics_suspect);
        RB_CHECK(contains(state.rbpodo_diagnostics->reason, "op_stat_sos_flag expected 0..12"));
        RB_CHECK(!contains(state.rbpodo_diagnostics->reason, "op_stat_sos_flag expected 0/1"));
    }

    {
        rb_servo::RbpodoSystemStateSnapshot snapshot = rbpodoSnapshot();
        snapshot.op_stat_ems_flag = 5;

        const rb_servo::RobotState state =
            rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Right, snapshot);
        RB_CHECK(state.has_valid_joint_state);
        RB_CHECK(state.has_error);
        RB_CHECK(state.error_code != snapshot.op_stat_ems_flag);
        RB_CHECK(state.lifecycle_state == "diagnostics_suspect");
        RB_CHECK(state.diagnostic_error_source == "rbpodo_diagnostics_suspect");
        RB_CHECK(state.rbpodo_diagnostics.has_value());
        RB_CHECK(!state.rbpodo_diagnostics->diagnostics_valid);
        RB_CHECK(state.rbpodo_diagnostics->diagnostics_suspect);
        RB_CHECK(contains(state.rbpodo_diagnostics->reason, "op_stat_ems_flag expected 0..4"));
        RB_CHECK(!contains(state.rbpodo_diagnostics->reason, "op_stat_ems_flag expected 0/1"));
    }

    return true;
}

bool testBooleanStatusFieldsStillRejectNonBooleanValues() {
    rb_servo::RbpodoSystemStateSnapshot snapshot = rbpodoSnapshot();
    snapshot.op_stat_collision_occur = 2;

    const rb_servo::RobotState state =
        rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Left, snapshot);
    RB_CHECK(state.has_valid_joint_state);
    RB_CHECK(state.has_error);
    // op_stat_collision_occur low 2 bits = 2 is malformed (spec: 0/1 only). It is rejected
    // and flagged suspect, but NOT as a collision: the masked collision check ((& 0b11)==1)
    // is false, and the raw-value collision fallback was removed (it only false-positived on
    // reserved bits), so this surfaces via the generic diagnostics_suspect source.
    RB_CHECK(state.diagnostic_error_source == "rbpodo_diagnostics_suspect");
    RB_CHECK(state.rbpodo_diagnostics.has_value());
    RB_CHECK(!state.rbpodo_diagnostics->diagnostics_valid);
    RB_CHECK(state.rbpodo_diagnostics->diagnostics_suspect);
    RB_CHECK(contains(state.rbpodo_diagnostics->reason, "op_stat_collision_occur expected 0/1"));
    return true;
}

bool testInitErrorSimulationStateIsFaultedButReadable() {
    rb_servo::RbpodoSystemStateSnapshot snapshot = rbpodoSnapshot();
    snapshot.real_vs_simulation_mode = 1;
    snapshot.init_state_info = 5;
    snapshot.init_error = 187;

    const rb_servo::RobotState state =
        rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Left, snapshot);
    RB_CHECK(state.has_valid_joint_state);
    RB_CHECK(state.has_error);
    RB_CHECK(!state.servo_enabled);
    RB_CHECK(state.error_code == 187);
    RB_CHECK(state.lifecycle_state == "faulted");
    RB_CHECK(state.diagnostic_error_source == "rbpodo_init_error");
    RB_CHECK(state.rbpodo_diagnostics.has_value());
    RB_CHECK(state.rbpodo_diagnostics->diagnostics_valid);
    RB_CHECK(!state.rbpodo_diagnostics->diagnostics_suspect);
    RB_CHECK(state.rbpodo_diagnostics->raw.init_error == 187);
    RB_CHECK(!rb_servo::rbpodoStateAcquisitionError(state).has_value());

    const std::optional<rb_servo::BackendError> readiness =
        rb_servo::rbpodoMotionReadinessError(rbpodoConfig("simulation"), snapshot, state);
    RB_CHECK(readiness.has_value());
    RB_CHECK(readiness->kind == rb_servo::BackendErrorKind::RobotFault);
    RB_CHECK(readiness->name == "rbpodo_init_error");
    return true;
}

bool testTinyTimeMarksDiagnosticsSuspectWithoutLosingJoints() {
    rb_servo::RbpodoSystemStateSnapshot snapshot = rbpodoSnapshot();
    snapshot.robot_time_sec = 3.0e-41;

    const rb_servo::RobotState state =
        rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Left, snapshot);
    RB_CHECK(state.has_valid_joint_state);
    RB_CHECK(state.has_error);
    RB_CHECK(state.rbpodo_diagnostics.has_value());
    RB_CHECK(state.rbpodo_diagnostics->diagnostics_suspect);
    RB_CHECK(contains(state.rbpodo_diagnostics->reason, "time"));
    RB_CHECK(!rb_servo::rbpodoStateAcquisitionError(state).has_value());

    const std::optional<rb_servo::BackendError> readiness =
        rb_servo::rbpodoMotionReadinessError(rbpodoConfig(), snapshot, state);
    RB_CHECK(readiness.has_value());
    RB_CHECK(readiness->kind == rb_servo::BackendErrorKind::RobotFault);
    RB_CHECK(readiness->name == "rbpodo_diagnostics_suspect");
    return true;
}

rb_servo::RbpodoSystemStateSnapshot controllerSimGarbageSelfCollisionSnapshot() {
    rb_servo::RbpodoSystemStateSnapshot snapshot = rbpodoSnapshot();
    snapshot.robot_time_sec = 0.0;
    snapshot.real_vs_simulation_mode = 1;
    snapshot.init_state_info = 6;
    snapshot.init_error = 0;
    snapshot.op_stat_sos_flag = 0;
    snapshot.op_stat_ems_flag = 0;
    snapshot.op_stat_soft_estop_occur = 0;
    snapshot.op_stat_collision_occur = 0;
    // Low 2 bits = 2 (an undefined self-collision value) so the field is genuinely
    // suspect after masking; the controller-sim policy then marks it unavailable.
    snapshot.op_stat_self_collision = 1984732818;
    return snapshot;
}

rb_servo::BackendConfig controllerSimUnavailableFieldConfig() {
    rb_servo::BackendConfig config = rbpodoConfig("simulation");
    config.controller_simulation_treat_unreliable_status_fields_as_unavailable = true;
    return config;
}

bool testControllerSimUnavailableFieldPolicyRequiresConfigAndGate() {
    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");
    allow_real.unset();
    allow_motion.unset();

    const rb_servo::RbpodoSystemStateSnapshot snapshot =
        controllerSimGarbageSelfCollisionSnapshot();

    {
        rb_servo::BackendConfig config = rbpodoConfig("simulation");
        const rb_servo::RobotState state =
            rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Left, snapshot, config);
        RB_CHECK(state.has_error);
        RB_CHECK(state.lifecycle_state == "diagnostics_suspect");
        RB_CHECK(state.rbpodo_diagnostics.has_value());
        RB_CHECK(state.rbpodo_diagnostics->diagnostics_suspect);
        RB_CHECK(state.rbpodo_diagnostics->unavailable_fields.empty());
    }

    {
        // Real/sim env gates retired: the unreliable-field policy is config-only,
        // so the opt-in suppresses the captured bad fields without any env.
        rb_servo::BackendConfig config = controllerSimUnavailableFieldConfig();
        const rb_servo::RobotState state =
            rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Left, snapshot, config);
        RB_CHECK(!state.has_error);
        RB_CHECK(state.lifecycle_state == "servo_enabled");
        RB_CHECK(state.rbpodo_diagnostics.has_value());
        RB_CHECK(!state.rbpodo_diagnostics->diagnostics_suspect);
        RB_CHECK(containsField(state.rbpodo_diagnostics->unavailable_fields, "op_stat_self_collision"));
    }

    allow_real.set("1");
    allow_motion.set("1");

    {
        rb_servo::BackendConfig config = controllerSimUnavailableFieldConfig();
        config.operation_mode = "real";
        const rb_servo::RobotState state =
            rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Left, snapshot, config);
        RB_CHECK(state.has_error);
        RB_CHECK(state.lifecycle_state == "diagnostics_suspect");
        RB_CHECK(state.rbpodo_diagnostics.has_value());
        RB_CHECK(state.rbpodo_diagnostics->diagnostics_suspect);
        RB_CHECK(state.rbpodo_diagnostics->unavailable_fields.empty());
    }

    return true;
}

bool testControllerSimUnavailableFieldPolicySuppressesOnlyCapturedBadFields() {
    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");
    allow_real.set("1");
    allow_motion.set("1");

    const rb_servo::RbpodoSystemStateSnapshot snapshot =
        controllerSimGarbageSelfCollisionSnapshot();
    const rb_servo::BackendConfig config = controllerSimUnavailableFieldConfig();
    const rb_servo::RobotState state =
        rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Right, snapshot, config);

    RB_CHECK(state.has_valid_joint_state);
    RB_CHECK(!state.has_error);
    RB_CHECK(state.lifecycle_state == "servo_enabled");
    RB_CHECK(state.rbpodo_state_decode_policy == "controller_sim_unreliable_fields_unavailable");
    RB_CHECK(state.rbpodo_diagnostics.has_value());
    RB_CHECK(state.rbpodo_diagnostics->diagnostics_valid);
    RB_CHECK(!state.rbpodo_diagnostics->diagnostics_suspect);
    RB_CHECK(state.rbpodo_diagnostics->raw.op_stat_self_collision == 1984732818);
    RB_CHECK(containsField(state.rbpodo_diagnostics->unavailable_fields, "op_stat_self_collision"));
    RB_CHECK(containsField(state.rbpodo_diagnostics->unavailable_fields, "robot_time_sec"));
    RB_CHECK(contains(state.rbpodo_diagnostics->reason, "unavailable fields"));
    RB_CHECK(!rb_servo::rbpodoStateAcquisitionError(state).has_value());
    RB_CHECK(!rb_servo::rbpodoMotionReadinessError(config, snapshot, state).has_value());
    return true;
}

bool testControllerSimUnavailableFieldPolicyStillFaultsRealSafetyFields() {
    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");
    allow_real.set("1");
    allow_motion.set("1");

    const rb_servo::BackendConfig config = controllerSimUnavailableFieldConfig();

    {
        rb_servo::RbpodoSystemStateSnapshot snapshot =
            controllerSimGarbageSelfCollisionSnapshot();
        snapshot.op_stat_collision_occur = 1;
        const rb_servo::RobotState state =
            rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Left, snapshot, config);
        RB_CHECK(state.has_error);
        RB_CHECK(state.lifecycle_state == "faulted");
        RB_CHECK(state.diagnostic_error_source == "rbpodo_collision");
        RB_CHECK(state.rbpodo_diagnostics.has_value());
        RB_CHECK(!state.rbpodo_diagnostics->diagnostics_suspect);
        const std::optional<rb_servo::BackendError> readiness =
            rb_servo::rbpodoMotionReadinessError(config, snapshot, state);
        RB_CHECK(readiness.has_value());
        RB_CHECK(readiness->kind == rb_servo::BackendErrorKind::RobotFault);
        RB_CHECK(readiness->name == "rbpodo_collision");
    }

    {
        rb_servo::RbpodoSystemStateSnapshot snapshot =
            controllerSimGarbageSelfCollisionSnapshot();
        snapshot.op_stat_self_collision = 1;
        const rb_servo::RobotState state =
            rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Right, snapshot, config);
        RB_CHECK(state.has_error);
        RB_CHECK(state.lifecycle_state == "faulted");
        RB_CHECK(state.diagnostic_error_source == "rbpodo_self_collision");
        RB_CHECK(state.rbpodo_diagnostics.has_value());
        RB_CHECK(!state.rbpodo_diagnostics->diagnostics_suspect);
        RB_CHECK(containsField(state.rbpodo_diagnostics->unavailable_fields, "op_stat_self_collision"));
        const std::optional<rb_servo::BackendError> readiness =
            rb_servo::rbpodoMotionReadinessError(config, snapshot, state);
        RB_CHECK(readiness.has_value());
        RB_CHECK(readiness->kind == rb_servo::BackendErrorKind::RobotFault);
        RB_CHECK(readiness->name == "rbpodo_self_collision");
    }

    return true;
}

rb_servo::BackendConfig controllerSimDemoteSelfCollisionConfig() {
    rb_servo::BackendConfig config = rbpodoConfig("simulation");
    config.controller_simulation_demote_self_collision_fault = true;
    return config;
}

// Explicit opt-in: in controller-simulation (pgmode) a CLEAN op_stat_self_collision (1005)
// is demoted to non-latching ONLY when controller_simulation_demote_self_collision_fault is
// set, deferring to the server's URDF-mesh CollisionMonitor. Default-off still latches, and
// the gate is controller-sim only so real (operation_mode: real) motion always latches.
bool testControllerSimDemoteSelfCollisionFaultOptIn() {
    rb_servo::RbpodoSystemStateSnapshot snapshot = controllerSimGarbageSelfCollisionSnapshot();
    snapshot.op_stat_self_collision = 1;  // clean self-collision bit (masked == 1)

    // Opt-in OFF (default pgmode): clean self-collision still latches.
    {
        const rb_servo::BackendConfig config = rbpodoConfig("simulation");
        const rb_servo::RobotState state =
            rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Left, snapshot, config);
        RB_CHECK(state.has_error);
        RB_CHECK(state.diagnostic_error_source == "rbpodo_self_collision");
    }

    // Opt-in ON (controller-sim): demoted — no hard fault, streaming allowed.
    {
        const rb_servo::BackendConfig config = controllerSimDemoteSelfCollisionConfig();
        const rb_servo::RobotState state =
            rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Left, snapshot, config);
        RB_CHECK(state.has_valid_joint_state);
        RB_CHECK(!state.has_error);
        RB_CHECK(state.lifecycle_state == "servo_enabled");
        RB_CHECK(state.diagnostic_error_source != "rbpodo_self_collision");
        const std::optional<rb_servo::BackendError> readiness =
            rb_servo::rbpodoMotionReadinessError(config, snapshot, state);
        RB_CHECK(!readiness.has_value() || readiness->name != "rbpodo_self_collision");
    }

    // Real (operation_mode: real) with the flag set must STILL latch: the gate is
    // controller-sim only, so real motion is never affected.
    {
        rb_servo::BackendConfig config = rbpodoConfig("real");
        config.controller_simulation_demote_self_collision_fault = true;
        rb_servo::RbpodoSystemStateSnapshot real_snapshot = snapshot;
        real_snapshot.real_vs_simulation_mode = 0;
        const rb_servo::RobotState state =
            rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Left, real_snapshot, config);
        RB_CHECK(state.has_error);
        RB_CHECK(state.diagnostic_error_source == "rbpodo_self_collision");
    }
    return true;
}

rb_servo::BackendConfig realSuspectDiagnosticsConfig() {
    rb_servo::BackendConfig config = rbpodoConfig("real");
    config.allow_real_motion_with_suspect_diagnostics = true;
    return config;
}

rb_servo::RbpodoSystemStateSnapshot realModeGarbageSelfCollisionSnapshot() {
    rb_servo::RbpodoSystemStateSnapshot snapshot = controllerSimGarbageSelfCollisionSnapshot();
    snapshot.real_vs_simulation_mode = 0;  // a real controller reports real mode
    return snapshot;
}

// operation_mode: real physical opt-in accepts the same vendor-garbage op_stat/time
// fields as unavailable (so the -2001 mismatch does not block a physical run), with a
// distinct operator-visible decode policy.
bool testRealMotionSuspectDiagnosticsAcceptedWhenGateOpen() {
    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");
    EnvVarGuard allow_suspect("RB_ALLOW_RBPODO_SUSPECT_DIAGNOSTICS_REAL_MOTION");
    allow_real.set("1");
    allow_motion.set("1");
    allow_suspect.set("1");

    const rb_servo::RbpodoSystemStateSnapshot snapshot = realModeGarbageSelfCollisionSnapshot();
    const rb_servo::BackendConfig config = realSuspectDiagnosticsConfig();
    const rb_servo::RobotState state =
        rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Left, snapshot, config);

    RB_CHECK(state.has_valid_joint_state);
    RB_CHECK(!state.has_error);
    RB_CHECK(state.rbpodo_state_decode_policy == "real_motion_suspect_diagnostics_accepted");
    RB_CHECK(state.rbpodo_diagnostics.has_value());
    RB_CHECK(state.rbpodo_diagnostics->diagnostics_valid);
    RB_CHECK(!state.rbpodo_diagnostics->diagnostics_suspect);
    RB_CHECK(state.rbpodo_diagnostics->raw.op_stat_self_collision == 1984732818);
    RB_CHECK(containsField(state.rbpodo_diagnostics->unavailable_fields, "op_stat_self_collision"));
    RB_CHECK(containsField(state.rbpodo_diagnostics->unavailable_fields, "robot_time_sec"));
    RB_CHECK(!rb_servo::rbpodoStateAcquisitionError(state).has_value());
    return true;
}

// Fail-closed: missing the dedicated env, the config opt-in, or a real operation mode
// all keep the suspect latch (and the controller-sim carve-out never leaks into real).
bool testRealMotionSuspectDiagnosticsFailClosed() {
    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");
    EnvVarGuard allow_suspect("RB_ALLOW_RBPODO_SUSPECT_DIAGNOSTICS_REAL_MOTION");
    allow_real.set("1");
    allow_motion.set("1");

    const rb_servo::RbpodoSystemStateSnapshot snapshot = realModeGarbageSelfCollisionSnapshot();

    // (a) env gates retired: the dedicated env no longer matters — the config
    //     opt-in alone opens the acceptance.
    allow_suspect.unset();
    {
        const rb_servo::BackendConfig config = realSuspectDiagnosticsConfig();
        const rb_servo::RobotState state =
            rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Left, snapshot, config);
        RB_CHECK(!state.has_error);
        RB_CHECK(state.rbpodo_state_decode_policy == "real_motion_suspect_diagnostics_accepted");
    }

    // (b) config opt-in false -> still suspect.
    allow_suspect.set("1");
    {
        const rb_servo::BackendConfig config = rbpodoConfig("real");
        const rb_servo::RobotState state =
            rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Left, snapshot, config);
        RB_CHECK(state.has_error);
        RB_CHECK(state.rbpodo_diagnostics->diagnostics_suspect);
    }

    // (c) operation_mode simulation with the REAL flag -> real gate needs non-sim, and
    //     the controller-sim carve-out flag is not set -> still suspect.
    {
        rb_servo::BackendConfig config = realSuspectDiagnosticsConfig();
        config.operation_mode = "simulation";
        const rb_servo::RobotState state = rb_servo::mapRbpodoSystemStateSnapshot(
            rb_servo::ArmId::Left, controllerSimGarbageSelfCollisionSnapshot(), config);
        RB_CHECK(state.has_error);
        RB_CHECK(state.rbpodo_diagnostics->diagnostics_suspect);
    }
    return true;
}

// The real suspect gate suppresses ONLY the two field-layout-garbage fields; genuine
// EMS/collision/self-collision faults still latch under physical motion.
bool testRealMotionSuspectGateStillFaultsRealSafetyFields() {
    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");
    EnvVarGuard allow_suspect("RB_ALLOW_RBPODO_SUSPECT_DIAGNOSTICS_REAL_MOTION");
    allow_real.set("1");
    allow_motion.set("1");
    allow_suspect.set("1");

    const rb_servo::BackendConfig config = realSuspectDiagnosticsConfig();

    {
        rb_servo::RbpodoSystemStateSnapshot snapshot = realModeGarbageSelfCollisionSnapshot();
        snapshot.op_stat_collision_occur = 1;
        const rb_servo::RobotState state =
            rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Left, snapshot, config);
        RB_CHECK(state.has_error);
        RB_CHECK(state.diagnostic_error_source == "rbpodo_collision");
        const std::optional<rb_servo::BackendError> readiness =
            rb_servo::rbpodoMotionReadinessError(config, snapshot, state);
        RB_CHECK(readiness.has_value());
        RB_CHECK(readiness->name == "rbpodo_collision");
    }

    {
        rb_servo::RbpodoSystemStateSnapshot snapshot = realModeGarbageSelfCollisionSnapshot();
        snapshot.op_stat_self_collision = 1;  // a genuine self-collision (clean 1, not garbage)
        const rb_servo::RobotState state =
            rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Right, snapshot, config);
        RB_CHECK(state.has_error);
        RB_CHECK(state.diagnostic_error_source == "rbpodo_self_collision");
    }
    return true;
}

bool testNonFiniteJointStateStillFailsAcquisition() {
    rb_servo::RbpodoSystemStateSnapshot snapshot = rbpodoSnapshot();
    snapshot.q_actual_deg[2] = std::numeric_limits<double>::quiet_NaN();

    const rb_servo::RobotState state =
        rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Left, snapshot);
    RB_CHECK(!state.has_valid_joint_state);
    RB_CHECK(!state.q_actual_valid);
    RB_CHECK(state.q_ref_valid);

    const std::optional<rb_servo::BackendError> acquisition =
        rb_servo::rbpodoStateAcquisitionError(state);
    RB_CHECK(acquisition.has_value());
    RB_CHECK(acquisition->kind == rb_servo::BackendErrorKind::InvalidJointState);

    const std::optional<rb_servo::BackendError> readiness =
        rb_servo::rbpodoMotionReadinessError(rbpodoConfig(), snapshot, state);
    RB_CHECK(readiness.has_value());
    RB_CHECK(readiness->kind == rb_servo::BackendErrorKind::InvalidJointState);
    return true;
}

bool testStatePublisherSerializesRawRbpodoDiagnostics() {
    rb_servo::RbpodoSystemStateSnapshot suspect_snapshot = rbpodoSnapshot();
    suspect_snapshot.op_stat_self_collision = 1977953904;
    suspect_snapshot.robot_time_sec = 3.0e-41;

    rb_servo::ServoSnapshot servo_snapshot;
    servo_snapshot.left_state =
        rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Left, suspect_snapshot);
    servo_snapshot.right_state.arm_id = rb_servo::ArmId::Right;
    servo_snapshot.right_state.q_actual_deg = joints(0.0);
    servo_snapshot.right_state.has_valid_joint_state = true;
    servo_snapshot.right_state.connection_state = rb_servo::RobotConnectionState::Connected;
    servo_snapshot.right_state.servo_enabled = true;

    rb_servo::DualArmConfig config;
    config.left_robot.backend_type = rb_servo::BackendType::Rbpodo;
    config.right_robot.backend_type = rb_servo::BackendType::Rbpodo;
    rb_servo::StatePublisher publisher(config);
    const nlohmann::json json = nlohmann::json::parse(publisher.serializeSnapshot(servo_snapshot));
    const nlohmann::json& left = json.at("left");
    const nlohmann::json& diagnostics = json.at("left").at("rbpodo_diagnostics");

    RB_CHECK(left.at("q_actual_deg").at(0).get<double>() == suspect_snapshot.q_actual_deg[0]);
    RB_CHECK(left.at("q_target_deg").at(0).get<double>() == suspect_snapshot.q_target_deg[0]);
    RB_CHECK(left.at("q_ref_deg").at(0).get<double>() == suspect_snapshot.q_target_deg[0]);
    RB_CHECK(left.at("q_actual_valid").get<bool>());
    RB_CHECK(left.at("q_ref_valid").get<bool>());
    RB_CHECK(left.at("q_ref_source").get<std::string>() == "rbpodo.sdata.jnt_ref");
    RB_CHECK(left.at("rbpodo_sdk_state_source").get<std::string>() == "CobotData.request_data");
    RB_CHECK(
        left.at("rbpodo_state_decode_policy").get<std::string>() ==
        "bounded_status_codes_with_boolean_safety_flags"
    );
    RB_CHECK(!diagnostics.at("diagnostics_valid").get<bool>());
    RB_CHECK(diagnostics.at("diagnostics_suspect").get<bool>());
    RB_CHECK(diagnostics.at("error_name").get<std::string>() == "rbpodo_diagnostics_suspect");
    RB_CHECK(diagnostics.at("unavailable_fields").empty());
    RB_CHECK(diagnostics.at("raw").at("op_stat_self_collision").get<int>() == 1977953904);
    RB_CHECK(diagnostics.at("raw").at("real_vs_simulation_mode").get<int>() == 0);
    RB_CHECK(diagnostics.at("raw").at("time").get<double>() == suspect_snapshot.robot_time_sec);
    RB_CHECK(json.at("right").at("rbpodo_diagnostics").is_null());
    return true;
}

bool testStatePublisherSerializesControllerSimUnavailableFields() {
    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");
    allow_real.set("1");
    allow_motion.set("1");

    const rb_servo::RbpodoSystemStateSnapshot snapshot =
        controllerSimGarbageSelfCollisionSnapshot();
    const rb_servo::BackendConfig backend_config = controllerSimUnavailableFieldConfig();

    rb_servo::ServoSnapshot servo_snapshot;
    servo_snapshot.left_state =
        rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Left, snapshot, backend_config);
    servo_snapshot.right_state.arm_id = rb_servo::ArmId::Right;
    servo_snapshot.right_state.q_actual_deg = joints(0.0);
    servo_snapshot.right_state.has_valid_joint_state = true;
    servo_snapshot.right_state.connection_state = rb_servo::RobotConnectionState::Connected;
    servo_snapshot.right_state.servo_enabled = true;

    rb_servo::DualArmConfig config;
    config.left_robot.backend_type = rb_servo::BackendType::Rbpodo;
    config.right_robot.backend_type = rb_servo::BackendType::Rbpodo;
    rb_servo::StatePublisher publisher(config);
    const nlohmann::json json = nlohmann::json::parse(publisher.serializeSnapshot(servo_snapshot));
    const nlohmann::json& left = json.at("left");
    const nlohmann::json& diagnostics = left.at("rbpodo_diagnostics");

    RB_CHECK(
        left.at("rbpodo_state_decode_policy").get<std::string>() ==
        "controller_sim_unreliable_fields_unavailable"
    );
    RB_CHECK(!diagnostics.at("diagnostics_suspect").get<bool>());
    RB_CHECK(diagnostics.at("raw").at("op_stat_self_collision").get<int>() == 1984732818);
    RB_CHECK(diagnostics.at("raw").at("time").get<double>() == 0.0);
    RB_CHECK(diagnostics.at("unavailable_fields").is_array());
    RB_CHECK(diagnostics.at("unavailable_fields").size() == 2);
    RB_CHECK(diagnostics.at("unavailable_fields").at(0).get<std::string>() == "op_stat_self_collision");
    RB_CHECK(diagnostics.at("unavailable_fields").at(1).get<std::string>() == "robot_time_sec");
    RB_CHECK(contains(diagnostics.at("reason").get<std::string>(), "unavailable fields"));
    return true;
}

}  // namespace

int main() {
    if (!testFreedriveControllerSignalsMapped()) return 1;
    if (!testClearSelfCollisionIsRobotFault()) return 1;
    if (!testSelfCollisionUpperBitsAreMaskedToHealthy()) return 1;
    if (!testRealSelfCollisionFaultsThroughGarbageUpperBits()) return 1;
    if (!testSosCodeMaskedThroughGarbageUpperBits()) return 1;
    if (!testValidSosCodeIsDeviceFaultNotBooleanViolation()) return 1;
    if (!testValidEmsCodeIsDeviceFaultNotBooleanViolation()) return 1;
    if (!testOutOfRangeSosAndEmsCodesAreSuspect()) return 1;
    if (!testBooleanStatusFieldsStillRejectNonBooleanValues()) return 1;
    if (!testInitErrorSimulationStateIsFaultedButReadable()) return 1;
    if (!testTinyTimeMarksDiagnosticsSuspectWithoutLosingJoints()) return 1;
    if (!testControllerSimUnavailableFieldPolicyRequiresConfigAndGate()) return 1;
    if (!testControllerSimUnavailableFieldPolicySuppressesOnlyCapturedBadFields()) return 1;
    if (!testControllerSimUnavailableFieldPolicyStillFaultsRealSafetyFields()) return 1;
    if (!testControllerSimDemoteSelfCollisionFaultOptIn()) return 1;
    if (!testRealMotionSuspectDiagnosticsAcceptedWhenGateOpen()) return 1;
    if (!testRealMotionSuspectDiagnosticsFailClosed()) return 1;
    if (!testRealMotionSuspectGateStillFaultsRealSafetyFields()) return 1;
    if (!testNonFiniteJointStateStillFailsAcquisition()) return 1;
    if (!testStatePublisherSerializesRawRbpodoDiagnostics()) return 1;
    if (!testStatePublisherSerializesControllerSimUnavailableFields()) return 1;
    return 0;
}
