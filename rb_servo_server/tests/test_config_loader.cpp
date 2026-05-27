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
    RB_CHECK(near(mock.servo.worker_read_period_sec, 0.01));

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
        EnvGuard rbscript_gate("RB_ALLOW_RBSCRIPT_TCP", "1");
        EnvGuard motion_gate("RB_ALLOW_REAL_MOTION", nullptr);
        EnvGuard rbscript_motion_gate("RB_ALLOW_RBSCRIPT_TCP_MOTION", nullptr);
        const rb_servo::DualArmConfig rbscript =
            rb_servo::loadConfigFromYaml((config_dir / "dual_real_rbscript.example.yaml").string());
        RB_CHECK(rbscript.left_robot.backend_type == rb_servo::BackendType::RbscriptTcp);
        RB_CHECK(rbscript.right_robot.backend_type == rb_servo::BackendType::RbscriptTcp);
        RB_CHECK(rbscript.left_robot.run_mode == rb_servo::RunMode::Real);
        RB_CHECK(rbscript.right_robot.run_mode == rb_servo::RunMode::Real);
        RB_CHECK(rbscript.left_robot.command_port == 5000);
        RB_CHECK(rbscript.left_robot.data_port == 5001);
        RB_CHECK(near(rbscript.left_robot.script_t1_sec, 0.008));
        RB_CHECK(near(rbscript.left_robot.script_t2_sec, 0.05));
        RB_CHECK(near(rbscript.left_robot.script_gain, 1.0));
        RB_CHECK(near(rbscript.left_robot.script_alpha, 0.5));
        RB_CHECK(!rbscript.servo.send_servo_commands);
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
        "  worker_read_period_sec: 0.01\n"
        "  worker_read_rate_hz: 100\n"
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
        "  worker_read_period_sec: 0.01\n"
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
        "  path_kp_pos: 2.5\n"
        "  path_kp_ori: 7.5\n"
        "  twist_orientation_hold_kp: 8.0\n"
        "  twist_angular_deadband_rad_s: 0.002\n"
        "  velocity_target_integration: measured_actual_lookahead\n"
        "  velocity_target_lookahead_sec: 0.05\n"
        "  max_command_actual_error_deg: [1, 2, 3, 4, 5, 6]\n"
        "  reset_velocity_integrator_on_mode_change: false\n"
        "  command_actual_error_policy: fault\n"
        "  linear_move:\n"
        "    constant_orientation_tolerance_rad: 0.004\n"
    );
    const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
    ::unlink(path.c_str());
    RB_CHECK(near(cfg.cartesian_control.path_kp_pos, 2.5));
    RB_CHECK(near(cfg.cartesian_control.path_kp_ori, 7.5));
    RB_CHECK(near(cfg.cartesian_control.twist_orientation_hold_kp, 8.0));
    RB_CHECK(near(cfg.cartesian_control.twist_angular_deadband_rad_s, 0.002));
    RB_CHECK(cfg.cartesian_control.velocity_target_integration ==
             rb_servo::CartesianVelocityTargetIntegrationMode::MeasuredActualLookahead);
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

bool testRbscriptTcpConfigParsesAndValidates() {
    const std::string valid_body =
        "schema: robotics_lab.rb_servo_server.v1\n"
        "left_robot:\n"
        "  backend_type: rbscript_tcp\n"
        "  run_mode: real\n"
        "  ip: \"172.28.60.200\"\n"
        "  command_port: 5000\n"
        "  data_port: 5001\n"
        "  command_timeout_sec: 0.2\n"
        "  read_timeout_sec: 0.2\n"
        "  connect_timeout_sec: 1.0\n"
        "  disable_waiting_ack: false\n"
        "  script_t1_sec: 0.008\n"
        "  script_t2_sec: 0.05\n"
        "  script_gain: 1.0\n"
        "  script_alpha: 0.5\n"
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

    const std::string missing_env_path = writeTempConfig("rbscript-missing-env", valid_body);
    {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        EnvGuard rbscript_gate("RB_ALLOW_RBSCRIPT_TCP", nullptr);
        const bool missing_env_rejected = loadRejects(missing_env_path);
        RB_CHECK(missing_env_rejected);
    }
    ::unlink(missing_env_path.c_str());

    const std::string valid_path = writeTempConfig("rbscript-valid", valid_body);
    {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        EnvGuard rbscript_gate("RB_ALLOW_RBSCRIPT_TCP", "1");
        EnvGuard motion_gate("RB_ALLOW_REAL_MOTION", nullptr);
        EnvGuard rbscript_motion_gate("RB_ALLOW_RBSCRIPT_TCP_MOTION", nullptr);
        const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(valid_path);
        RB_CHECK(cfg.left_robot.backend_type == rb_servo::BackendType::RbscriptTcp);
        RB_CHECK(cfg.left_robot.command_port == 5000);
        RB_CHECK(cfg.left_robot.data_port == 5001);
        RB_CHECK(near(cfg.left_robot.command_timeout_sec, 0.2));
        RB_CHECK(near(cfg.left_robot.read_timeout_sec, 0.2));
        RB_CHECK(near(cfg.left_robot.connect_timeout_sec, 1.0));
        RB_CHECK(!cfg.left_robot.disable_waiting_ack);
    }
    ::unlink(valid_path.c_str());

    const std::string simulation_path = writeTempConfig(
        "rbscript-simulation",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "left_robot:\n"
        "  backend_type: rbscript_tcp\n"
        "  run_mode: simulation\n"
        "  ip: \"127.0.0.1\"\n"
    );
    {
        EnvGuard rbscript_gate("RB_ALLOW_RBSCRIPT_TCP", "1");
        const bool simulation_rejected = loadRejects(simulation_path);
        RB_CHECK(simulation_rejected);
    }
    ::unlink(simulation_path.c_str());

    const std::string bad_t1_path = writeTempConfig(
        "rbscript-bad-t1",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "left_robot:\n"
        "  backend_type: rbscript_tcp\n"
        "  run_mode: real\n"
        "  ip: \"172.28.60.200\"\n"
        "  script_t1_sec: 0.001\n"
        "right_robot:\n"
        "  backend_type: mock\n"
        "  run_mode: mock\n"
        "servo:\n"
        "  send_servo_commands: false\n"
        "  enable_realtime_priority: true\n"
        "safety:\n"
        "  tracking_error_policy: fault_latch\n"
    );
    {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        EnvGuard rbscript_gate("RB_ALLOW_RBSCRIPT_TCP", "1");
        const bool bad_t1_rejected = loadRejects(bad_t1_path);
        RB_CHECK(bad_t1_rejected);
    }
    ::unlink(bad_t1_path.c_str());

    const std::string bad_values_path = writeTempConfig(
        "rbscript-bad-values",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "left_robot:\n"
        "  backend_type: rbscript_tcp\n"
        "  run_mode: real\n"
        "  ip: \"172.28.60.200\"\n"
        "  script_t2_sec: 0.2\n"
        "right_robot:\n"
        "  backend_type: mock\n"
        "  run_mode: mock\n"
        "servo:\n"
        "  send_servo_commands: false\n"
        "  enable_realtime_priority: true\n"
        "safety:\n"
        "  tracking_error_policy: fault_latch\n"
    );
    {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        EnvGuard rbscript_gate("RB_ALLOW_RBSCRIPT_TCP", "1");
        const bool bad_values_rejected = loadRejects(bad_values_path);
        RB_CHECK(bad_values_rejected);
    }
    ::unlink(bad_values_path.c_str());

    const std::string motion_path = writeTempConfig(
        "rbscript-motion-missing-env",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "left_robot:\n"
        "  backend_type: rbscript_tcp\n"
        "  run_mode: real\n"
        "  ip: \"172.28.60.200\"\n"
        "right_robot:\n"
        "  backend_type: mock\n"
        "  run_mode: mock\n"
        "servo:\n"
        "  send_servo_commands: true\n"
        "  enable_realtime_priority: true\n"
        "safety:\n"
        "  tracking_error_policy: fault_latch\n"
        "  stop_both_arms_on_single_arm_error: true\n"
        "  latch_fault_on_robot_state_error: true\n"
    );
    {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        EnvGuard rbscript_gate("RB_ALLOW_RBSCRIPT_TCP", "1");
        EnvGuard motion_gate("RB_ALLOW_REAL_MOTION", "1");
        EnvGuard rbscript_motion_gate("RB_ALLOW_RBSCRIPT_TCP_MOTION", nullptr);
        const bool motion_rejected = loadRejects(motion_path);
        RB_CHECK(motion_rejected);
    }
    ::unlink(motion_path.c_str());
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
    if (!testRbscriptTcpConfigParsesAndValidates()) return 1;
    return 0;
}
