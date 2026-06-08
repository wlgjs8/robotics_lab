#include <cstdlib>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>

#include "rb_servo/config/config.hpp"

namespace {

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

std::string writeTempConfig(const std::string& name, const std::string& body) {
    const std::string path = "/tmp/rb-servo-config-" + name + "-" + std::to_string(getpid()) + ".yaml";
    std::ofstream file(path);
    file << body;
    return path;
}

bool loadRejects(const std::string& path) {
    try {
        (void)rb_servo::loadConfigFromYaml(path);
    } catch (const std::exception&) {
        return true;
    }
    return false;
}

bool near(double a, double b) {
    return std::abs(a - b) < 1e-12;
}

std::filesystem::path servoRoot() {
    return std::filesystem::path(__FILE__).parent_path().parent_path();
}

std::string rb3UrdfPath() {
    return (servoRoot() / "descriptions" / "urdf" / "rb3_730e.urdf").string();
}

class EnvGuard {
public:
    EnvGuard(const char* name, const char* value) : name_(name) {
        const char* previous = std::getenv(name);
        if (previous) {
            had_previous_ = true;
            previous_ = previous;
        }
        if (value) {
            ::setenv(name, value, 1);
        } else {
            ::unsetenv(name);
        }
    }

    ~EnvGuard() {
        if (had_previous_) {
            ::setenv(name_.c_str(), previous_.c_str(), 1);
        } else {
            ::unsetenv(name_.c_str());
        }
    }

private:
    std::string name_;
    bool had_previous_ = false;
    std::string previous_;
};

bool assertSimulatorCartesianConfig(const rb_servo::DualArmConfig& cfg) {
    RB_CHECK(cfg.left_robot.backend_type == rb_servo::BackendType::Simulator);
    RB_CHECK(cfg.right_robot.backend_type == rb_servo::BackendType::Simulator);
    RB_CHECK(cfg.left_robot.run_mode == rb_servo::RunMode::Simulation);
    RB_CHECK(cfg.right_robot.run_mode == rb_servo::RunMode::Simulation);
    RB_CHECK(cfg.kinematics.enable);
    RB_CHECK(cfg.kinematics.provider == "pinocchio");
    RB_CHECK(std::filesystem::is_regular_file(cfg.kinematics.urdf));
    RB_CHECK(cfg.kinematics.base_link == "world");
    RB_CHECK(cfg.kinematics.tip_link == "tcp");
    RB_CHECK(cfg.kinematics.joint_names.size() == rb_servo::kDof);
    RB_CHECK(cfg.kinematics.q_units == "deg");
    RB_CHECK(cfg.kinematics.publish_tcp);
    RB_CHECK(cfg.kinematics.ik.enable);
    RB_CHECK(cfg.cartesian_control.enable);
    RB_CHECK(cfg.cartesian_control.allow_in_simulation);
    RB_CHECK(!cfg.cartesian_control.allow_in_real);
    RB_CHECK(!cfg.cartesian_control.allow_in_controller_simulation);
    RB_CHECK(!cfg.cartesian_control.enable_server_side_circle_track);
    RB_CHECK(near(cfg.cartesian_control.linear_move.constant_orientation_tolerance_rad, 0.005));
    RB_CHECK(cfg.force_control.provider == "null");
    RB_CHECK(!cfg.force_control.enable);
    return true;
}

bool testRepositoryConfigsParse() {
    const std::filesystem::path config_dir =
        std::filesystem::path(__FILE__).parent_path().parent_path() / "config";

    const rb_servo::DualArmConfig mock =
        rb_servo::loadConfigFromYaml((config_dir / "dual_mock.yaml").string());
    RB_CHECK(mock.left_robot.backend_type == rb_servo::BackendType::Mock);
    RB_CHECK(mock.right_robot.backend_type == rb_servo::BackendType::Mock);
    RB_CHECK(mock.network.state_pub_endpoint == "udp://127.0.0.1:50110");
    RB_CHECK(mock.network.state_pub_bind == mock.network.state_pub_endpoint);
    RB_CHECK(mock.network.state_pub_rate_hz == 20);
    RB_CHECK(mock.force_control.provider == "null");
    RB_CHECK(!mock.force_control.enable);
    RB_CHECK(mock.servo.io_model == rb_servo::ServoIoModel::Direct);
    RB_CHECK(near(mock.servo.worker_read_period_sec, 0.002));

    const rb_servo::DualArmConfig simulator =
        rb_servo::loadConfigFromYaml((config_dir / "dual_simulator.yaml").string());
    RB_CHECK(assertSimulatorCartesianConfig(simulator));
    RB_CHECK(simulator.left_robot.simulator_control_endpoint == "tcp://127.0.0.1:50200");
    RB_CHECK(simulator.right_robot.simulator_control_endpoint == "tcp://127.0.0.1:50210");
    RB_CHECK(near(simulator.cartesian_control.path_kp_pos, 6.0));
    RB_CHECK(near(simulator.cartesian_control.path_kp_ori, 6.0));
    RB_CHECK(near(simulator.cartesian_control.twist_angular_deadband_rad_s, 0.0001));

    const rb_servo::DualArmConfig remote_simulator =
        rb_servo::loadConfigFromYaml((config_dir / "dual_simulator_remote_172_28_60_36.yaml").string());
    RB_CHECK(assertSimulatorCartesianConfig(remote_simulator));
    RB_CHECK(remote_simulator.left_robot.simulator_control_endpoint == "tcp://172.28.60.36:50200");
    RB_CHECK(remote_simulator.right_robot.simulator_control_endpoint == "tcp://172.28.60.36:50210");
    RB_CHECK(!remote_simulator.cartesian_control.allow_in_real);

    const rb_servo::DualArmConfig simulator_worker =
        rb_servo::loadConfigFromYaml((config_dir / "dual_simulator_worker.yaml").string());
    RB_CHECK(assertSimulatorCartesianConfig(simulator_worker));
    RB_CHECK(simulator_worker.servo.io_model == rb_servo::ServoIoModel::Worker);
    RB_CHECK(near(simulator_worker.servo.worker_read_period_sec, 0.01));

    const rb_servo::DualArmConfig tcp_acceptance =
        rb_servo::loadConfigFromYaml((config_dir / "dual_simulator_tcp_acceptance.yaml").string());
    RB_CHECK(tcp_acceptance.left_robot.backend_type == rb_servo::BackendType::Simulator);
    RB_CHECK(tcp_acceptance.right_robot.backend_type == rb_servo::BackendType::Simulator);
    RB_CHECK(tcp_acceptance.command_source.enforce_lease);
    RB_CHECK(tcp_acceptance.network.command_source_enforce_lease);

    {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        EnvGuard motion_gate("RB_ALLOW_REAL_MOTION", nullptr);
        const rb_servo::DualArmConfig real_default =
            rb_servo::loadConfigFromYaml((config_dir / "dual_real.example.yaml").string());
        RB_CHECK(real_default.left_robot.backend_type == rb_servo::BackendType::Rbpodo);
        RB_CHECK(real_default.servo.rate_hz == 500);
        RB_CHECK(near(real_default.left_robot.servo_t1_sec, 0.002));
        RB_CHECK(near(real_default.left_robot.servo_t2_sec, 0.05));
        RB_CHECK(near(real_default.left_robot.servo_alpha, 0.5));
        RB_CHECK(near(real_default.left_robot.command_timeout_sec, 0.02));
        RB_CHECK(!real_default.servo.send_servo_commands);
        RB_CHECK(near(real_default.safety.q_min_deg[0], -360.0));
        RB_CHECK(near(real_default.safety.q_min_deg[2], -360.0));
        RB_CHECK(near(real_default.safety.q_max_deg[0], 360.0));
        RB_CHECK(near(real_default.safety.q_max_deg[2], 360.0));
    }

    {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        EnvGuard motion_gate("RB_ALLOW_REAL_MOTION", "1");
        EnvGuard physical_cartesian_gate("RB_ALLOW_REAL_CARTESIAN", nullptr);

        const rb_servo::DualArmConfig pgmode =
            rb_servo::loadConfigFromYaml(
                (config_dir / "dual_real_rbpodo_pgmode_spacemouse_500hz_ack.example.yaml").string()
            );
        RB_CHECK(pgmode.left_robot.backend_type == rb_servo::BackendType::Rbpodo);
        RB_CHECK(pgmode.right_robot.backend_type == rb_servo::BackendType::Rbpodo);
        RB_CHECK(pgmode.left_robot.run_mode == rb_servo::RunMode::Real);
        RB_CHECK(pgmode.right_robot.run_mode == rb_servo::RunMode::Real);
        RB_CHECK(pgmode.left_robot.operation_mode == "simulation");
        RB_CHECK(pgmode.right_robot.operation_mode == "simulation");
        RB_CHECK(pgmode.servo.rate_hz == 500);
        RB_CHECK(pgmode.servo.send_servo_commands);
        RB_CHECK(pgmode.servo.allow_controller_simulation_motion);
        RB_CHECK(pgmode.servo.allow_controller_simulation_diagnostics_suspect);
        RB_CHECK(!pgmode.servo.allow_controller_simulation_init_error);
        RB_CHECK(!pgmode.servo.allow_controller_simulation_not_activated);
        RB_CHECK(pgmode.left_robot.allow_controller_simulation_diagnostics_suspect);
        RB_CHECK(pgmode.right_robot.allow_controller_simulation_diagnostics_suspect);
        RB_CHECK(!pgmode.left_robot.allow_controller_simulation_init_error);
        RB_CHECK(!pgmode.right_robot.allow_controller_simulation_init_error);
        RB_CHECK(near(pgmode.left_robot.servo_t1_sec, 0.002));
        RB_CHECK(near(pgmode.right_robot.servo_t1_sec, 0.002));
        RB_CHECK(near(pgmode.left_robot.servo_t2_sec, 0.08));
        RB_CHECK(near(pgmode.left_robot.servo_alpha, 0.8));
        RB_CHECK(!pgmode.left_robot.disable_waiting_ack);
        RB_CHECK(!pgmode.right_robot.disable_waiting_ack);
        RB_CHECK(pgmode.servo.rbpodo_async_streaming.enable);
        RB_CHECK(pgmode.servo.rbpodo_async_streaming.mode ==
                 rb_servo::RbpodoAsyncStreamingMode::SdkAckWorker);
        RB_CHECK(pgmode.servo.rbpodo_async_streaming.rate_hz == 500);
        RB_CHECK(pgmode.servo.rbpodo_async_streaming.reference_supervision.policy ==
                 rb_servo::RbpodoAsyncReferenceSupervisionPolicy::FaultLatch);
        RB_CHECK(pgmode.servo.rbpodo_async_streaming.diagnostics.publish_per_command_jsonl);
        RB_CHECK(near(pgmode.safety.q_min_deg[0], -360.0));
        RB_CHECK(near(pgmode.safety.q_max_deg[2], 360.0));
        RB_CHECK(pgmode.safety.controller_simulation_tracking_error_source ==
                 rb_servo::ControllerSimulationTrackingErrorSource::Reference);
        RB_CHECK(pgmode.safety.controller_simulation_physical_motion_policy ==
                 rb_servo::ControllerSimulationPhysicalMotionPolicy::FaultLatch);
        RB_CHECK(pgmode.network.command_bind == "udp://127.0.0.1:50256");
        RB_CHECK(pgmode.network.state_pub_endpoint == "udp://127.0.0.1:50356");
        RB_CHECK(pgmode.network.state_pub_endpoints.size() == 3);
        RB_CHECK(pgmode.network.state_pub_endpoints[1] == "udp://127.0.0.1:50366");
        RB_CHECK(pgmode.network.state_pub_endpoints[2] == "udp://127.0.0.1:50376");
        RB_CHECK(pgmode.command_source.enforce_lease);
        RB_CHECK(pgmode.network.command_source_enforce_lease);
        RB_CHECK(near(pgmode.command_source.lease_timeout_sec, 60.0));
        RB_CHECK(pgmode.cartesian_control.enable);
        RB_CHECK(pgmode.cartesian_control.allow_in_controller_simulation);
        RB_CHECK(!pgmode.cartesian_control.allow_in_real);
        RB_CHECK(!pgmode.cartesian_control.enable_benchmark_primitives);
        RB_CHECK(!pgmode.cartesian_control.circle_move.allow_in_simulation);
        RB_CHECK(near(pgmode.cartesian_control.max_twist_linear_m_s, 0.2));
        RB_CHECK(near(pgmode.cartesian_control.max_twist_angular_rad_s, 0.4));
        RB_CHECK(pgmode.cartesian_control.controller_simulation_servo_state_source ==
                 rb_servo::CartesianControllerSimulationStateSource::Reference);
        RB_CHECK(pgmode.cartesian_control.controller_simulation_divergence_source ==
                 rb_servo::CartesianControllerSimulationStateSource::Reference);
        RB_CHECK(pgmode.force_control.provider == "null");
        RB_CHECK(!pgmode.force_control.enable);
        RB_CHECK(pgmode.kinematics.enable);
        RB_CHECK(pgmode.kinematics.provider == "pinocchio");
    }

    return true;
}

bool testServoIoModelParsesAndValidates() {
    const std::string worker_path = writeTempConfig(
        "worker-io-model",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  io_model: worker\n"
        "  worker_read_period_sec: 0.025\n"
    );
    const rb_servo::DualArmConfig worker = rb_servo::loadConfigFromYaml(worker_path);
    ::unlink(worker_path.c_str());
    RB_CHECK(worker.servo.io_model == rb_servo::ServoIoModel::Worker);
    RB_CHECK(near(worker.servo.worker_read_period_sec, 0.025));

    const std::string worker_rate_path = writeTempConfig(
        "worker-read-rate",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  io_model: worker\n"
        "  worker_read_rate_hz: 50\n"
    );
    const rb_servo::DualArmConfig worker_rate = rb_servo::loadConfigFromYaml(worker_rate_path);
    ::unlink(worker_rate_path.c_str());
    RB_CHECK(near(worker_rate.servo.worker_read_period_sec, 0.02));

    const std::string invalid_path = writeTempConfig(
        "invalid-io-model",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  io_model: threadpool\n"
    );
    const bool invalid_rejected = loadRejects(invalid_path);
    ::unlink(invalid_path.c_str());
    RB_CHECK(invalid_rejected);

    const std::string invalid_period_path = writeTempConfig(
        "invalid-worker-read-period",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  worker_read_period_sec: 0\n"
    );
    const bool invalid_period_rejected = loadRejects(invalid_period_path);
    ::unlink(invalid_period_path.c_str());
    RB_CHECK(invalid_period_rejected);

    const std::string ambiguous_period_path = writeTempConfig(
        "ambiguous-worker-read-period",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  worker_read_period_sec: 0.002\n"
        "  worker_read_rate_hz: 500\n"
    );
    const bool ambiguous_period_rejected = loadRejects(ambiguous_period_path);
    ::unlink(ambiguous_period_path.c_str());
    RB_CHECK(ambiguous_period_rejected);

    const std::string real_worker_path = writeTempConfig(
        "real-worker-io-model",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "left_robot:\n"
        "  backend_type: rbpodo\n"
        "  run_mode: real\n"
        "  ip: \"172.28.60.200\"\n"
        "right_robot:\n"
        "  backend_type: rbpodo\n"
        "  run_mode: real\n"
        "  ip: \"172.28.60.201\"\n"
        "servo:\n"
        "  io_model: worker\n"
        "  worker_read_period_sec: 0.002\n"
        "  send_servo_commands: false\n"
        "  enable_realtime_priority: true\n"
        "safety:\n"
        "  tracking_error_policy: fault_latch\n"
        "  stop_both_arms_on_single_arm_error: true\n"
        "  latch_fault_on_robot_state_error: true\n"
    );
    const bool real_worker_rejected = loadRejects(real_worker_path);
    ::unlink(real_worker_path.c_str());
    RB_CHECK(real_worker_rejected);
    return true;
}

bool testUnknownKeysAndSchemaFail() {
    const std::string unknown_key_path = writeTempConfig(
        "unknown-key",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "network:\n"
        "  state_pub_endpoint: \"udp://127.0.0.1:50110\"\n"
        "  state_pub_destination_typo: \"udp://127.0.0.1:50111\"\n"
    );
    const bool unknown_key_rejected = loadRejects(unknown_key_path);
    ::unlink(unknown_key_path.c_str());
    RB_CHECK(unknown_key_rejected);

    const std::string unknown_schema_path = writeTempConfig(
        "unknown-schema",
        "schema: robotics_lab.rb_servo_server.v2\n"
    );
    const bool unknown_schema_rejected = loadRejects(unknown_schema_path);
    ::unlink(unknown_schema_path.c_str());
    RB_CHECK(unknown_schema_rejected);
    return true;
}

bool testDeprecatedAliasesWarnAndParse() {
    const std::string path = writeTempConfig(
        "deprecated-alias",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "left_robot:\n"
        "  backend_type: rbsim_local\n"
        "  run_mode: rbsim_local\n"
        "  rbsim_control_endpoint: \"tcp://127.0.0.1:50200\"\n"
        "right_robot:\n"
        "  backend_type: mock\n"
        "  run_mode: mock\n"
        "network:\n"
        "  state_pub_bind: \"udp://127.0.0.1:55110\"\n"
        "  state_pub_rate_hz: 33\n"
    );

    std::ostringstream warnings;
    auto* const old_cerr = std::cerr.rdbuf(warnings.rdbuf());
    const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
    std::cerr.rdbuf(old_cerr);
    ::unlink(path.c_str());

    RB_CHECK(cfg.left_robot.backend_type == rb_servo::BackendType::Simulator);
    RB_CHECK(cfg.left_robot.run_mode == rb_servo::RunMode::Simulation);
    RB_CHECK(cfg.left_robot.simulator_control_endpoint == "tcp://127.0.0.1:50200");
    RB_CHECK(cfg.left_robot.rbsim_control_endpoint == cfg.left_robot.simulator_control_endpoint);
    RB_CHECK(cfg.network.state_pub_endpoint == "udp://127.0.0.1:55110");
    RB_CHECK(cfg.network.state_pub_bind == cfg.network.state_pub_endpoint);
    RB_CHECK(cfg.network.state_pub_endpoints.size() == 1);
    RB_CHECK(cfg.network.state_pub_endpoints.front() == cfg.network.state_pub_endpoint);
    RB_CHECK(cfg.network.state_pub_rate_hz == 33);
    RB_CHECK(warnings.str().find("deprecated") != std::string::npos);
    return true;
}

bool testStatePublisherEndpointsParseAndValidate() {
    const std::string path = writeTempConfig(
        "state-pub-endpoints",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "network:\n"
        "  state_pub_endpoints:\n"
        "    - \"udp://rb_gui:50110\"\n"
        "    - \"udp://policy_runner_state_sink:50120\"\n"
    );
    const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
    ::unlink(path.c_str());
    RB_CHECK(cfg.network.state_pub_endpoint == "udp://rb_gui:50110");
    RB_CHECK(cfg.network.state_pub_bind == cfg.network.state_pub_endpoint);
    RB_CHECK(cfg.network.state_pub_endpoints.size() == 2);
    RB_CHECK(cfg.network.state_pub_endpoints[0] == "udp://rb_gui:50110");
    RB_CHECK(cfg.network.state_pub_endpoints[1] == "udp://policy_runner_state_sink:50120");

    const std::string ambiguous_path = writeTempConfig(
        "state-pub-ambiguous",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "network:\n"
        "  state_pub_endpoint: \"udp://rb_gui:50110\"\n"
        "  state_pub_endpoints:\n"
        "    - \"udp://policy_runner_state_sink:50120\"\n"
    );
    const bool ambiguous_rejected = loadRejects(ambiguous_path);
    ::unlink(ambiguous_path.c_str());
    RB_CHECK(ambiguous_rejected);
    return true;
}

bool testForceControlStaysDisabled() {
    const std::string enabled_path = writeTempConfig(
        "force-enabled",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "force_control:\n"
        "  provider: null\n"
        "  enable: true\n"
    );
    const bool enabled_rejected = loadRejects(enabled_path);
    ::unlink(enabled_path.c_str());
    RB_CHECK(enabled_rejected);

    const std::string provider_path = writeTempConfig(
        "force-provider",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "force_control:\n"
        "  provider: admittance\n"
        "  enable: false\n"
    );
    const bool provider_rejected = loadRejects(provider_path);
    ::unlink(provider_path.c_str());
    RB_CHECK(provider_rejected);
    return true;
}

bool testCommandSourceConfigParsesAndValidates() {
    const std::string path = writeTempConfig(
        "command-source",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "command_source:\n"
        "  enforce_lease: true\n"
        "  lease_timeout_sec: 1.25\n"
    );
    const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
    ::unlink(path.c_str());
    RB_CHECK(cfg.command_source.enforce_lease);
    RB_CHECK(cfg.command_source.lease_timeout_sec == 1.25);
    RB_CHECK(cfg.network.command_source_enforce_lease);
    RB_CHECK(cfg.network.command_source_lease_timeout_sec == 1.25);

    const std::string invalid_path = writeTempConfig(
        "command-source-invalid",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "command_source:\n"
        "  lease_timeout_sec: 0\n"
    );
    const bool invalid_rejected = loadRejects(invalid_path);
    ::unlink(invalid_path.c_str());
    RB_CHECK(invalid_rejected);
    return true;
}

bool testCartesianControlTuningParsesAndValidates() {
    const std::string path = writeTempConfig(
        "cartesian-tuning",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "cartesian_control:\n"
        "  allow_in_controller_simulation: true\n"
        "  enable_server_side_circle_track: true\n"
        "  path_kp_pos: 2.5\n"
        "  path_kp_ori: 7.5\n"
        "  twist_orientation_hold_kp: 8.0\n"
        "  twist_angular_deadband_rad_s: 0.002\n"
        "  velocity_target_integration: measured_actual_lookahead\n"
        "  controller_simulation_servo_state_source: reference\n"
        "  controller_simulation_divergence_source: reference\n"
        "  velocity_target_lookahead_sec: 0.05\n"
        "  max_command_actual_error_deg: [1, 2, 3, 4, 5, 6]\n"
        "  reset_velocity_integrator_on_mode_change: false\n"
        "  command_actual_error_policy: fault\n"
        "  linear_move:\n"
        "    constant_orientation_tolerance_rad: 0.004\n"
    );
    const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
    ::unlink(path.c_str());
    RB_CHECK(cfg.cartesian_control.allow_in_controller_simulation);
    RB_CHECK(cfg.cartesian_control.enable_server_side_circle_track);
    RB_CHECK(near(cfg.cartesian_control.path_kp_pos, 2.5));
    RB_CHECK(near(cfg.cartesian_control.path_kp_ori, 7.5));
    RB_CHECK(near(cfg.cartesian_control.twist_orientation_hold_kp, 8.0));
    RB_CHECK(near(cfg.cartesian_control.twist_angular_deadband_rad_s, 0.002));
    RB_CHECK(cfg.cartesian_control.velocity_target_integration ==
             rb_servo::CartesianVelocityTargetIntegrationMode::MeasuredActualLookahead);
    RB_CHECK(cfg.cartesian_control.controller_simulation_servo_state_source ==
             rb_servo::CartesianControllerSimulationStateSource::Reference);
    RB_CHECK(cfg.cartesian_control.controller_simulation_divergence_source ==
             rb_servo::CartesianControllerSimulationStateSource::Reference);
    RB_CHECK(near(cfg.cartesian_control.velocity_target_lookahead_sec, 0.05));
    RB_CHECK(near(cfg.cartesian_control.max_command_actual_error_deg[0], 1.0));
    RB_CHECK(near(cfg.cartesian_control.max_command_actual_error_deg[5], 6.0));
    RB_CHECK(!cfg.cartesian_control.reset_velocity_integrator_on_mode_change);
    RB_CHECK(cfg.cartesian_control.command_actual_error_policy ==
             rb_servo::CartesianCommandActualErrorPolicy::Fault);
    RB_CHECK(near(cfg.cartesian_control.linear_move.constant_orientation_tolerance_rad, 0.004));

    const std::string legacy_path = writeTempConfig(
        "cartesian-legacy-path-kp",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "cartesian_control:\n"
        "  path_kp: 4.0\n"
    );
    std::ostringstream warnings;
    auto* const old_cerr = std::cerr.rdbuf(warnings.rdbuf());
    const rb_servo::DualArmConfig legacy = rb_servo::loadConfigFromYaml(legacy_path);
    std::cerr.rdbuf(old_cerr);
    ::unlink(legacy_path.c_str());
    RB_CHECK(near(legacy.cartesian_control.path_kp, 4.0));
    RB_CHECK(near(legacy.cartesian_control.path_kp_pos, 4.0));
    RB_CHECK(near(legacy.cartesian_control.path_kp_ori, 4.0));
    RB_CHECK(warnings.str().find("deprecated") != std::string::npos);

    const std::string ambiguous_path = writeTempConfig(
        "cartesian-ambiguous-path-kp",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "cartesian_control:\n"
        "  path_kp: 4.0\n"
        "  path_kp_pos: 5.0\n"
    );
    const bool ambiguous_rejected = loadRejects(ambiguous_path);
    ::unlink(ambiguous_path.c_str());
    RB_CHECK(ambiguous_rejected);

    const std::string bad_gain_path = writeTempConfig(
        "cartesian-bad-gain",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "cartesian_control:\n"
        "  path_kp_ori: 0\n"
    );
    const bool bad_gain_rejected = loadRejects(bad_gain_path);
    ::unlink(bad_gain_path.c_str());
    RB_CHECK(bad_gain_rejected);

    const std::string bad_deadband_path = writeTempConfig(
        "cartesian-bad-deadband",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "cartesian_control:\n"
        "  twist_angular_deadband_rad_s: 0\n"
    );
    const bool bad_deadband_rejected = loadRejects(bad_deadband_path);
    ::unlink(bad_deadband_path.c_str());
    RB_CHECK(bad_deadband_rejected);

    const std::string bad_velocity_integration_path = writeTempConfig(
        "cartesian-bad-velocity-integration",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "cartesian_control:\n"
        "  velocity_target_integration: nope\n"
    );
    const bool bad_velocity_integration_rejected = loadRejects(bad_velocity_integration_path);
    ::unlink(bad_velocity_integration_path.c_str());
    RB_CHECK(bad_velocity_integration_rejected);

    const std::string bad_command_actual_error_path = writeTempConfig(
        "cartesian-bad-command-actual-error",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "cartesian_control:\n"
        "  max_command_actual_error_deg: [1, 1, 1, 1, 1, 0]\n"
    );
    const bool bad_command_actual_error_rejected = loadRejects(bad_command_actual_error_path);
    ::unlink(bad_command_actual_error_path.c_str());
    RB_CHECK(bad_command_actual_error_rejected);

    const std::string bad_constant_orientation_tolerance_path = writeTempConfig(
        "cartesian-bad-constant-orientation-tolerance",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "cartesian_control:\n"
        "  linear_move:\n"
        "    constant_orientation_tolerance_rad: 0\n"
    );
    const bool bad_constant_orientation_tolerance_rejected =
        loadRejects(bad_constant_orientation_tolerance_path);
    ::unlink(bad_constant_orientation_tolerance_path.c_str());
    RB_CHECK(bad_constant_orientation_tolerance_rejected);
    return true;
}

bool testRemovedRawScriptBackendRejected() {
    const std::string removed_backend = std::string("rb") + "script_tcp";
    const std::string removed_backend_body =
        "schema: robotics_lab.rb_servo_server.v1\n"
        "left_robot:\n"
        "  backend_type: " + removed_backend + "\n"
        "  run_mode: real\n"
        "  ip: \"172.28.60.200\"\n"
        "  command_timeout_sec: 0.2\n"
        "  disable_waiting_ack: false\n"
        "right_robot:\n"
        "  backend_type: mock\n"
        "  run_mode: mock\n"
        "servo:\n"
        "  send_servo_commands: false\n"
        "  enable_realtime_priority: true\n"
        "safety:\n"
        "  tracking_error_policy: fault_latch\n"
        "  stop_both_arms_on_single_arm_error: true\n"
        "  latch_fault_on_robot_state_error: true\n";

    const std::string removed_backend_path = writeTempConfig("removed-raw-script-backend", removed_backend_body);
    RB_CHECK(loadRejects(removed_backend_path));
    ::unlink(removed_backend_path.c_str());

    const std::string removed_backend_key_path = writeTempConfig(
        "removed-raw-script-key",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "left_robot:\n"
        "  backend_type: rbpodo\n"
        "  run_mode: real\n"
        "  ip: \"172.28.60.200\"\n"
        "  command_port: 5000\n"
    );
    RB_CHECK(loadRejects(removed_backend_key_path));
    ::unlink(removed_backend_key_path.c_str());
    return true;
}

std::string rbpodoEnvIpConfigBody(const std::string& left_ip, const std::string& right_ip) {
    return std::string(
        "schema: robotics_lab.rb_servo_server.v1\n"
        "left_robot:\n"
        "  backend_type: rbpodo\n"
        "  run_mode: real\n"
        "  ip: \"") + left_ip + "\"\n" +
        "  operation_mode: simulation\n"
        "  servo_t1_sec: 0.002\n"
        "  servo_t2_sec: 0.05\n"
        "  servo_gain: 1.0\n"
        "  servo_alpha: 0.5\n"
        "right_robot:\n"
        "  backend_type: rbpodo\n"
        "  run_mode: real\n"
        "  ip: \"" + right_ip + "\"\n" +
        "  operation_mode: simulation\n"
        "  servo_t1_sec: 0.002\n"
        "  servo_t2_sec: 0.05\n"
        "  servo_gain: 1.0\n"
        "  servo_alpha: 0.5\n"
        "servo:\n"
        "  rate_hz: 500\n"
        "  send_servo_commands: false\n"
        "  enable_realtime_priority: true\n"
        "safety:\n"
        "  tracking_error_policy: fault_latch\n"
        "  stop_both_arms_on_single_arm_error: true\n"
        "  latch_fault_on_robot_state_error: true\n";
}

bool testRobotIpEnvExpansion() {
    {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        EnvGuard left_ip("ROBOT_LEFT_IP", "192.168.56.101");
        EnvGuard right_ip("ROBOT_RIGHT_IP", "192.168.56.102");
        const std::string path = writeTempConfig(
            "rbpodo-env-ip",
            rbpodoEnvIpConfigBody("${ROBOT_LEFT_IP}", "${ROBOT_RIGHT_IP}")
        );
        const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
        ::unlink(path.c_str());
        RB_CHECK(cfg.left_robot.ip == "192.168.56.101");
        RB_CHECK(cfg.right_robot.ip == "192.168.56.102");
        RB_CHECK(cfg.left_robot.operation_mode == "simulation");
        RB_CHECK(!cfg.servo.send_servo_commands);
    }

    {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        EnvGuard ignored("ROBOT_LEFT_IP", "192.168.56.101");
        const std::string path = writeTempConfig(
            "rbpodo-literal-ip",
            rbpodoEnvIpConfigBody("192.168.56.201", "192.168.56.202")
        );
        const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
        ::unlink(path.c_str());
        RB_CHECK(cfg.left_robot.ip == "192.168.56.201");
        RB_CHECK(cfg.right_robot.ip == "192.168.56.202");
    }

    {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        EnvGuard missing("ROBOT_MISSING_IP", nullptr);
        const std::string path = writeTempConfig(
            "rbpodo-missing-env-ip",
            rbpodoEnvIpConfigBody("${ROBOT_MISSING_IP}", "192.168.56.202")
        );
        const bool rejected = loadRejects(path);
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }

    {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        EnvGuard empty("ROBOT_EMPTY_IP", "");
        const std::string path = writeTempConfig(
            "rbpodo-empty-env-ip",
            rbpodoEnvIpConfigBody("${ROBOT_EMPTY_IP}", "192.168.56.202")
        );
        const bool rejected = loadRejects(path);
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }

    {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        const std::string path = writeTempConfig(
            "rbpodo-invalid-env-ip",
            rbpodoEnvIpConfigBody("${1BAD_IP}", "192.168.56.202")
        );
        const bool rejected = loadRejects(path);
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }

    return true;
}

std::string rbpodoConfigBody(
    const std::string& left_fields,
    int rate_hz,
    bool send_servo_commands
) {
    return
        "schema: robotics_lab.rb_servo_server.v1\n"
        "left_robot:\n"
        "  backend_type: rbpodo\n"
        "  run_mode: real\n"
        "  ip: \"172.28.60.200\"\n"
        "  operation_mode: real\n" +
        left_fields +
        "right_robot:\n"
        "  backend_type: mock\n"
        "  run_mode: mock\n"
        "servo:\n"
        "  rate_hz: " + std::to_string(rate_hz) + "\n"
        "  send_servo_commands: " + std::string(send_servo_commands ? "true\n" : "false\n") +
        "  enable_realtime_priority: true\n"
        "  servo_t1_rate_match_tolerance_ratio: 0.2\n"
        "  allow_servo_t1_rate_mismatch: false\n" +
        "safety:\n"
        "  tracking_error_policy: fault_latch\n"
        "  stop_both_arms_on_single_arm_error: true\n"
        "  latch_fault_on_robot_state_error: true\n";
}

bool testRbpodoServoJParametersParseAndValidate() {
    {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        EnvGuard motion_gate("RB_ALLOW_REAL_MOTION", nullptr);
        const std::string path = writeTempConfig(
            "rbpodo-servo-canonical",
            rbpodoConfigBody(
                "  servo_t1_sec: 0.002\n"
                "  servo_t2_sec: 0.05\n"
                "  servo_gain: 1.25\n"
                "  servo_alpha: 0.5\n",
                500,
                false
            )
        );
        const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
        ::unlink(path.c_str());
        RB_CHECK(near(cfg.left_robot.servo_t1_sec, 0.002));
        RB_CHECK(near(cfg.left_robot.servo_t2_sec, 0.05));
        RB_CHECK(near(cfg.left_robot.servo_gain, 1.25));
        RB_CHECK(near(cfg.left_robot.servo_alpha, 0.5));
        RB_CHECK(near(cfg.left_robot.command_timeout_sec, 0.2));
        RB_CHECK(near(cfg.left_robot.servo_time_sec, cfg.left_robot.servo_t1_sec));
        RB_CHECK(near(cfg.left_robot.servo_lookahead_sec, cfg.left_robot.servo_t2_sec));
        RB_CHECK(near(cfg.left_robot.servo_acc, cfg.left_robot.servo_alpha));
    }

    {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        std::ostringstream warnings;
        auto* const old_cerr = std::cerr.rdbuf(warnings.rdbuf());
        const std::string path = writeTempConfig(
            "rbpodo-servo-deprecated",
            rbpodoConfigBody(
                "  servo_time_sec: 0.002\n"
                "  servo_lookahead_sec: 0.05\n"
                "  servo_gain: 1.0\n"
                "  servo_acc: 0.5\n",
                500,
                false
            )
        );
        const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
        std::cerr.rdbuf(old_cerr);
        ::unlink(path.c_str());
        RB_CHECK(near(cfg.left_robot.servo_t1_sec, 0.002));
        RB_CHECK(near(cfg.left_robot.servo_t2_sec, 0.05));
        RB_CHECK(near(cfg.left_robot.servo_alpha, 0.5));
        RB_CHECK(warnings.str().find("deprecated") != std::string::npos);
    }

    {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        const std::string path = writeTempConfig(
            "rbpodo-servo-duplicate",
            rbpodoConfigBody(
                "  servo_t1_sec: 0.002\n"
                "  servo_time_sec: 0.002\n"
                "  servo_t2_sec: 0.05\n"
                "  servo_gain: 1.0\n"
                "  servo_alpha: 0.5\n",
                500,
                false
            )
        );
        const bool rejected = loadRejects(path);
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }

    const std::string invalid_cases[] = {
        "  command_timeout_sec: 0.0\n  servo_t1_sec: 0.002\n  servo_t2_sec: 0.05\n  servo_gain: 1.0\n  servo_alpha: 0.5\n",
        "  servo_t1_sec: 0.001\n  servo_t2_sec: 0.05\n  servo_gain: 1.0\n  servo_alpha: 0.5\n",
        "  servo_t1_sec: 0.002\n  servo_t2_sec: 0.02\n  servo_gain: 1.0\n  servo_alpha: 0.5\n",
        "  servo_t1_sec: 0.002\n  servo_t2_sec: 0.2\n  servo_gain: 1.0\n  servo_alpha: 0.5\n",
        "  servo_t1_sec: 0.002\n  servo_t2_sec: 0.05\n  servo_gain: 0.0\n  servo_alpha: 0.5\n",
        "  servo_t1_sec: 0.002\n  servo_t2_sec: 0.05\n  servo_gain: 1.0\n  servo_alpha: 0.0\n",
        "  servo_t1_sec: 0.002\n  servo_t2_sec: 0.05\n  servo_gain: 1.0\n  servo_alpha: 1.0\n",
    };
    for (std::size_t i = 0; i < sizeof(invalid_cases) / sizeof(invalid_cases[0]); ++i) {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        const std::string path = writeTempConfig(
            "rbpodo-servo-invalid-" + std::to_string(i),
            rbpodoConfigBody(invalid_cases[i], 500, false)
        );
        const bool rejected = loadRejects(path);
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }

    {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        EnvGuard motion_gate("RB_ALLOW_REAL_MOTION", "1");
        const std::string path = writeTempConfig(
            "rbpodo-servo-rate-mismatch",
            rbpodoConfigBody(
                "  servo_t1_sec: 0.004\n"
                "  servo_t2_sec: 0.05\n"
                "  servo_gain: 1.0\n"
                "  servo_alpha: 0.5\n",
                500,
                true
            )
        );
        const bool rejected = loadRejects(path);
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }

    {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        EnvGuard motion_gate("RB_ALLOW_REAL_MOTION", "1");
        const std::string path = writeTempConfig(
            "rbpodo-servo-rate-manual-non500",
            rbpodoConfigBody(
                "  servo_t1_sec: 0.004\n"
                "  servo_t2_sec: 0.05\n"
                "  servo_gain: 1.0\n"
                "  servo_alpha: 0.5\n",
                250,
                true
            )
        );
        const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
        ::unlink(path.c_str());
        RB_CHECK(cfg.servo.rate_hz == 250);
        RB_CHECK(near(cfg.left_robot.servo_t1_sec, 0.004));
    }

    {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        EnvGuard motion_gate("RB_ALLOW_REAL_MOTION", "1");
        EnvGuard ack_disabled_motion_gate("RB_ALLOW_RBPODO_ACK_DISABLED_MOTION", nullptr);
        const std::string path = writeTempConfig(
            "rbpodo-ack-disabled-motion-gate",
            rbpodoConfigBody(
                "  servo_t1_sec: 0.002\n"
                "  servo_t2_sec: 0.05\n"
                "  servo_gain: 1.0\n"
                "  servo_alpha: 0.5\n"
                "  disable_waiting_ack: true\n",
                500,
                true
            )
        );
        const bool rejected = loadRejects(path);
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }

    {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        EnvGuard motion_gate("RB_ALLOW_REAL_MOTION", "1");
        EnvGuard ack_disabled_motion_gate("RB_ALLOW_RBPODO_ACK_DISABLED_MOTION", "1");
        const std::string path = writeTempConfig(
            "rbpodo-ack-disabled-motion-explicit",
            rbpodoConfigBody(
                "  command_timeout_sec: 0.02\n"
                "  servo_t1_sec: 0.002\n"
                "  servo_t2_sec: 0.05\n"
                "  servo_gain: 1.0\n"
                "  servo_alpha: 0.5\n"
                "  disable_waiting_ack: true\n",
                500,
                true
            )
        );
        const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
        ::unlink(path.c_str());
        RB_CHECK(cfg.left_robot.disable_waiting_ack);
        RB_CHECK(near(cfg.left_robot.command_timeout_sec, 0.02));
    }
    return true;
}

bool testKinematicsSafetyLimitMismatchWarnsForRbpodo() {
    EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
    const std::string path = writeTempConfig(
        "rbpodo-kinematics-safety-mismatch",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "left_robot:\n"
        "  backend_type: rbpodo\n"
        "  run_mode: real\n"
        "  ip: \"172.28.60.200\"\n"
        "  operation_mode: simulation\n"
        "  servo_t1_sec: 0.002\n"
        "  servo_t2_sec: 0.05\n"
        "  servo_gain: 1.0\n"
        "  servo_alpha: 0.5\n"
        "right_robot:\n"
        "  backend_type: rbpodo\n"
        "  run_mode: real\n"
        "  ip: \"172.28.60.201\"\n"
        "  operation_mode: simulation\n"
        "  servo_t1_sec: 0.002\n"
        "  servo_t2_sec: 0.05\n"
        "  servo_gain: 1.0\n"
        "  servo_alpha: 0.5\n"
        "servo:\n"
        "  rate_hz: 500\n"
        "  send_servo_commands: false\n"
        "  enable_realtime_priority: true\n"
        "safety:\n"
        "  q_min_deg: [-360, -360, -360, -360, -360, -360]\n"
        "  q_max_deg: [360, 360, 360, 360, 360, 360]\n"
        "  tracking_error_policy: fault_latch\n"
        "  stop_both_arms_on_single_arm_error: true\n"
        "  latch_fault_on_robot_state_error: true\n"
        "kinematics:\n"
        "  enable: true\n"
        "  provider: pinocchio\n"
        "  urdf: \"" + rb3UrdfPath() + "\"\n"
        "  publish_tcp: true\n"
    );

    std::ostringstream warnings;
    auto* const old_cerr = std::cerr.rdbuf(warnings.rdbuf());
    const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
    std::cerr.rdbuf(old_cerr);
    ::unlink(path.c_str());
    RB_CHECK(cfg.kinematics.enable);
    RB_CHECK(near(cfg.safety.q_min_deg[2], -360.0));
    RB_CHECK(warnings.str().find("differs from rb3_730e URDF IK limit") != std::string::npos);
    RB_CHECK(warnings.str().find("elbow_joint") != std::string::npos);
    return true;
}

std::string controllerSimReadMissBody(int left_misses, int right_misses) {
    return
        "schema: robotics_lab.rb_servo_server.v1\n"
        "left_robot:\n"
        "  backend_type: rbpodo\n"
        "  run_mode: real\n"
        "  ip: \"172.28.60.200\"\n"
        "  operation_mode: simulation\n"
        "  max_consecutive_read_misses: " + std::to_string(left_misses) + "\n"
        "right_robot:\n"
        "  backend_type: rbpodo\n"
        "  run_mode: real\n"
        "  ip: \"172.28.60.201\"\n"
        "  operation_mode: simulation\n"
        "  max_consecutive_read_misses: " + std::to_string(right_misses) + "\n"
        "servo:\n"
        "  rate_hz: 500\n"
        "  send_servo_commands: false\n"
        "  enable_realtime_priority: true\n"
        "safety:\n"
        "  tracking_error_policy: fault_latch\n"
        "  stop_both_arms_on_single_arm_error: true\n"
        "  latch_fault_on_robot_state_error: true\n";
}

bool testReadMissToleranceParsesAndIsControllerSimOnly() {
    EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");

    // Controller simulation accepts and parses the tolerance.
    {
        const std::string path = writeTempConfig(
            "read-miss-controller-sim", controllerSimReadMissBody(3, 2));
        const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
        ::unlink(path.c_str());
        RB_CHECK(cfg.left_robot.max_consecutive_read_misses == 3);
        RB_CHECK(cfg.right_robot.max_consecutive_read_misses == 2);
    }

    // Default is 0 (fail-closed) when the key is absent.
    {
        const std::string path = writeTempConfig(
            "read-miss-default", controllerSimReadMissBody(0, 0));
        const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
        ::unlink(path.c_str());
        RB_CHECK(cfg.left_robot.max_consecutive_read_misses == 0);
    }

    // Negative tolerance is rejected.
    {
        const std::string path = writeTempConfig(
            "read-miss-negative", controllerSimReadMissBody(-1, 0));
        const bool rejected = loadRejects(path);
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }

    // Physical real (operation_mode: real) must fail closed: tolerance > 0 rejected.
    {
        const std::string path = writeTempConfig(
            "read-miss-real-rejected",
            rbpodoConfigBody("  max_consecutive_read_misses: 3\n", 500, false));
        const bool rejected = loadRejects(path);
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }

    return true;
}

}  // namespace

int main() {
    if (!testRepositoryConfigsParse()) return 1;
    if (!testServoIoModelParsesAndValidates()) return 1;
    if (!testUnknownKeysAndSchemaFail()) return 1;
    if (!testDeprecatedAliasesWarnAndParse()) return 1;
    if (!testStatePublisherEndpointsParseAndValidate()) return 1;
    if (!testForceControlStaysDisabled()) return 1;
    if (!testCommandSourceConfigParsesAndValidates()) return 1;
    if (!testCartesianControlTuningParsesAndValidates()) return 1;
    if (!testRemovedRawScriptBackendRejected()) return 1;
    if (!testRobotIpEnvExpansion()) return 1;
    if (!testRbpodoServoJParametersParseAndValidate()) return 1;
    if (!testKinematicsSafetyLimitMismatchWarnsForRbpodo()) return 1;
    if (!testReadMissToleranceParsesAndIsControllerSimOnly()) return 1;
    return 0;
}
