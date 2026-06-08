#include <cmath>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <utility>

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

bool testHugeSelfCollisionIsSuspectButReadable() {
    rb_servo::RbpodoSystemStateSnapshot snapshot = rbpodoSnapshot();
    snapshot.op_stat_self_collision = 1977953904;

    const rb_servo::RobotState state =
        rb_servo::mapRbpodoSystemStateSnapshot(rb_servo::ArmId::Right, snapshot);
    RB_CHECK(state.has_valid_joint_state);
    RB_CHECK(state.has_error);
    RB_CHECK(state.error_code != snapshot.op_stat_self_collision);
    RB_CHECK(state.lifecycle_state == "diagnostics_suspect");
    RB_CHECK(state.diagnostic_error_source == "rbpodo_diagnostics_suspect");
    RB_CHECK(state.rbpodo_diagnostics.has_value());
    RB_CHECK(!state.rbpodo_diagnostics->diagnostics_valid);
    RB_CHECK(state.rbpodo_diagnostics->diagnostics_suspect);
    RB_CHECK(state.rbpodo_diagnostics->raw.op_stat_self_collision == 1977953904);
    RB_CHECK(contains(state.rbpodo_diagnostics->reason, "op_stat_self_collision"));
    RB_CHECK(!rb_servo::rbpodoStateAcquisitionError(state).has_value());

    const std::optional<rb_servo::BackendError> readiness =
        rb_servo::rbpodoMotionReadinessError(rbpodoConfig(), snapshot, state);
    RB_CHECK(readiness.has_value());
    RB_CHECK(readiness->kind == rb_servo::BackendErrorKind::RobotFault);
    RB_CHECK(readiness->name == "rbpodo_diagnostics_suspect");
    RB_CHECK(contains(readiness->message, "op_stat_self_collision"));

    rb_servo::SendServoJRequest request;
    request.command_seq = 9;
    request.q_target_deg = joints(0.0);
    const rb_servo::SendServoJResult rejected =
        rb_servo::rejectedSend(request, *readiness, {}, state, "cache");
    RB_CHECK(!rejected.accepted);
    RB_CHECK(rejected.error.kind == rb_servo::BackendErrorKind::RobotFault);
    RB_CHECK(rejected.error.name == "rbpodo_diagnostics_suspect");
    RB_CHECK(rejected.state_after.has_value());
    RB_CHECK(rejected.state_after->rbpodo_diagnostics.has_value());
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
        "strict_boolean_flags_with_suspect_large_values"
    );
    RB_CHECK(!diagnostics.at("diagnostics_valid").get<bool>());
    RB_CHECK(diagnostics.at("diagnostics_suspect").get<bool>());
    RB_CHECK(diagnostics.at("error_name").get<std::string>() == "rbpodo_diagnostics_suspect");
    RB_CHECK(diagnostics.at("raw").at("op_stat_self_collision").get<int>() == 1977953904);
    RB_CHECK(diagnostics.at("raw").at("real_vs_simulation_mode").get<int>() == 0);
    RB_CHECK(diagnostics.at("raw").at("time").get<double>() == suspect_snapshot.robot_time_sec);
    RB_CHECK(json.at("right").at("rbpodo_diagnostics").is_null());
    return true;
}

}  // namespace

int main() {
    if (!testClearSelfCollisionIsRobotFault()) return 1;
    if (!testHugeSelfCollisionIsSuspectButReadable()) return 1;
    if (!testInitErrorSimulationStateIsFaultedButReadable()) return 1;
    if (!testTinyTimeMarksDiagnosticsSuspectWithoutLosingJoints()) return 1;
    if (!testNonFiniteJointStateStillFailsAcquisition()) return 1;
    if (!testStatePublisherSerializesRawRbpodoDiagnostics()) return 1;
    return 0;
}
