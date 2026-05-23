#include <cstdlib>
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

    const rb_servo::DualArmConfig simulator =
        rb_servo::loadConfigFromYaml((config_dir / "dual_simulator.yaml").string());
    RB_CHECK(simulator.left_robot.backend_type == rb_servo::BackendType::Simulator);
    RB_CHECK(simulator.right_robot.backend_type == rb_servo::BackendType::Simulator);
    RB_CHECK(simulator.left_robot.run_mode == rb_servo::RunMode::Simulation);
    RB_CHECK(simulator.right_robot.run_mode == rb_servo::RunMode::Simulation);
    RB_CHECK(simulator.left_robot.simulator_control_endpoint == "tcp://127.0.0.1:50200");
    RB_CHECK(simulator.right_robot.simulator_control_endpoint == "tcp://127.0.0.1:50210");
    return true;
}

bool testServoIoModelParsesAndValidates() {
    const std::string worker_path = writeTempConfig(
        "worker-io-model",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  io_model: worker\n"
    );
    const rb_servo::DualArmConfig worker = rb_servo::loadConfigFromYaml(worker_path);
    ::unlink(worker_path.c_str());
    RB_CHECK(worker.servo.io_model == rb_servo::ServoIoModel::Worker);

    const std::string invalid_path = writeTempConfig(
        "invalid-io-model",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  io_model: threadpool\n"
    );
    const bool invalid_rejected = loadRejects(invalid_path);
    ::unlink(invalid_path.c_str());
    RB_CHECK(invalid_rejected);

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
    RB_CHECK(cfg.network.state_pub_rate_hz == 33);
    RB_CHECK(warnings.str().find("deprecated") != std::string::npos);
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

}  // namespace

int main() {
    if (!testRepositoryConfigsParse()) return 1;
    if (!testServoIoModelParsesAndValidates()) return 1;
    if (!testUnknownKeysAndSchemaFail()) return 1;
    if (!testDeprecatedAliasesWarnAndParse()) return 1;
    if (!testForceControlStaysDisabled()) return 1;
    return 0;
}
