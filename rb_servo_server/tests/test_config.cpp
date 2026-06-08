#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
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
    const std::string path = "/tmp/rb-servo-wrap-config-" + name + "-" + std::to_string(getpid()) + ".yaml";
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

std::string asyncControllerSimulationBody(
    const std::string& mode,
    const std::string& left_operation_mode = "simulation",
    const std::string& right_operation_mode = "simulation",
    bool disable_waiting_ack = false
) {
    const std::string ack_line = disable_waiting_ack ? "  disable_waiting_ack: true\n" : "";
    return
        "schema: robotics_lab.rb_servo_server.v1\n"
        "left_robot:\n"
        "  backend_type: rbpodo\n"
        "  run_mode: real\n"
        "  ip: 172.28.60.200\n"
        "  operation_mode: " + left_operation_mode + "\n"
        "  servo_t1_sec: 0.002\n"
        "  servo_t2_sec: 0.03\n" +
        ack_line +
        "right_robot:\n"
        "  backend_type: rbpodo\n"
        "  run_mode: real\n"
        "  ip: 172.28.60.201\n"
        "  operation_mode: " + right_operation_mode + "\n"
        "  servo_t1_sec: 0.002\n"
        "  servo_t2_sec: 0.03\n" +
        ack_line +
        "servo:\n"
        "  rate_hz: 500\n"
        "  enable_realtime_priority: true\n"
        "  send_servo_commands: true\n"
        "  allow_controller_simulation_motion: true\n"
        "  rbpodo_async_streaming:\n"
        "    enable: true\n"
        "    mode: " + mode + "\n"
        "    rate_hz: 500\n"
        "    queue_policy: latest_wins\n"
        "    max_pending_age_ms: 10\n"
        "    ack_supervision:\n"
        "      enable: true\n"
        "      expected_ack_timeout_ms: 50\n"
        "      missing_ack_fault_after_ms: 100\n"
        "      max_consecutive_missing_ack: 10\n"
        "    reference_supervision:\n"
        "      enable: true\n"
        "      q_ref_update_timeout_ms: 50\n"
        "      q_ref_target_tolerance_deg: 0.5\n"
        "      tcp_ref_update_timeout_ms: 50\n"
        "    diagnostics:\n"
        "      publish_per_command_jsonl: false\n"
        "safety:\n"
        "  tracking_error_policy: fault_latch\n";
}

bool testJointWrapConfigParses() {
    const std::string path = writeTempConfig(
        "valid",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  send_servo_commands: false\n"
        "safety:\n"
        "  joint_wrap_period_deg: [0, 0, 0, 360, 0, 360]\n"
        "  joint_wrap_for_startup_validation: true\n"
        "  joint_wrap_for_motion_safety: false\n"
    );
    const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
    ::unlink(path.c_str());
    RB_CHECK(near(cfg.safety.joint_wrap_period_deg[3], 360.0));
    RB_CHECK(near(cfg.safety.joint_wrap_period_deg[5], 360.0));
    RB_CHECK(cfg.safety.joint_wrap_for_startup_validation);
    RB_CHECK(!cfg.safety.joint_wrap_for_motion_safety);
    return true;
}

bool testInvalidJointWrapConfigRejects() {
    const std::string negative_path = writeTempConfig(
        "negative-period",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "safety:\n"
        "  joint_wrap_period_deg: [0, 0, 0, -360, 0, 360]\n"
    );
    RB_CHECK(loadRejects(negative_path));
    ::unlink(negative_path.c_str());

    const std::string wrong_length_path = writeTempConfig(
        "wrong-length",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "safety:\n"
        "  joint_wrap_period_deg: [0, 0, 0, 360, 0]\n"
    );
    RB_CHECK(loadRejects(wrong_length_path));
    ::unlink(wrong_length_path.c_str());

    const std::string nonfinite_path = writeTempConfig(
        "nonfinite-period",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "safety:\n"
        "  joint_wrap_period_deg: [0, 0, 0, .nan, 0, 360]\n"
    );
    RB_CHECK(loadRejects(nonfinite_path));
    ::unlink(nonfinite_path.c_str());

    const std::string motion_wrap_path = writeTempConfig(
        "motion-wrap",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  send_servo_commands: false\n"
        "safety:\n"
        "  joint_wrap_period_deg: [0, 0, 0, 360, 0, 360]\n"
        "  joint_wrap_for_motion_safety: true\n"
    );
    RB_CHECK(loadRejects(motion_wrap_path));
    ::unlink(motion_wrap_path.c_str());

    const std::string startup_motion_path = writeTempConfig(
        "startup-motion",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  send_servo_commands: true\n"
        "safety:\n"
        "  joint_wrap_period_deg: [0, 0, 0, 360, 0, 360]\n"
        "  joint_wrap_for_startup_validation: true\n"
    );
    RB_CHECK(loadRejects(startup_motion_path));
    ::unlink(startup_motion_path.c_str());

    return true;
}

bool testControllerSimulationGateConfig() {
    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");

    allow_real.unset();
    allow_motion.unset();

    const std::string valid_body =
        "schema: robotics_lab.rb_servo_server.v1\n"
        "left_robot:\n"
        "  backend_type: rbpodo\n"
        "  run_mode: real\n"
        "  ip: 172.28.60.200\n"
        "  operation_mode: simulation\n"
        "  servo_t1_sec: 0.002\n"
        "  servo_t2_sec: 0.05\n"
        "  servo_alpha: 0.5\n"
        "right_robot:\n"
        "  backend_type: rbpodo\n"
        "  run_mode: real\n"
        "  ip: 172.28.60.201\n"
        "  operation_mode: simulation\n"
        "  servo_t1_sec: 0.002\n"
        "  servo_t2_sec: 0.05\n"
        "  servo_alpha: 0.5\n"
        "servo:\n"
        "  rate_hz: 500\n"
        "  enable_realtime_priority: true\n"
        "  send_servo_commands: true\n"
        "  allow_controller_simulation_motion: true\n"
        "  allow_controller_simulation_diagnostics_suspect: true\n"
        "  allow_controller_simulation_init_error: true\n"
        "  allow_controller_simulation_not_activated: true\n"
        "safety:\n"
        "  tracking_error_policy: fault_latch\n"
        "  controller_simulation_tracking_error_source: reference\n"
        "  controller_simulation_physical_motion_policy: fault_latch\n"
        "  controller_simulation_physical_motion_threshold_deg: 0.05\n"
        "cartesian_control:\n"
        "  allow_in_controller_simulation: true\n";

    const std::string missing_real_env_path = writeTempConfig("controller-sim-missing-real-env", valid_body);
    RB_CHECK(loadRejects(missing_real_env_path));
    ::unlink(missing_real_env_path.c_str());

    allow_real.set("1");
    allow_motion.set("1");

    const std::string valid_path = writeTempConfig("controller-sim-valid", valid_body);
    const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(valid_path);
    ::unlink(valid_path.c_str());
    RB_CHECK(cfg.servo.allow_controller_simulation_motion);
    RB_CHECK(cfg.servo.allow_controller_simulation_diagnostics_suspect);
    RB_CHECK(cfg.servo.allow_controller_simulation_init_error);
    RB_CHECK(cfg.servo.allow_controller_simulation_not_activated);
    RB_CHECK(cfg.cartesian_control.allow_in_controller_simulation);
    RB_CHECK(cfg.left_robot.allow_controller_simulation_diagnostics_suspect);
    RB_CHECK(cfg.right_robot.allow_controller_simulation_diagnostics_suspect);
    RB_CHECK(cfg.left_robot.allow_controller_simulation_init_error);
    RB_CHECK(cfg.right_robot.allow_controller_simulation_init_error);
    RB_CHECK(
        cfg.safety.controller_simulation_tracking_error_source ==
        rb_servo::ControllerSimulationTrackingErrorSource::Reference
    );
    RB_CHECK(
        cfg.safety.controller_simulation_physical_motion_policy ==
        rb_servo::ControllerSimulationPhysicalMotionPolicy::FaultLatch
    );
    RB_CHECK(cfg.safety.controller_simulation_physical_motion_threshold_deg == 0.05);
    RB_CHECK(cfg.left_robot.operation_mode == "simulation");
    RB_CHECK(cfg.right_robot.operation_mode == "simulation");

    std::string real_body = valid_body;
    const std::size_t pos = real_body.find("operation_mode: simulation");
    RB_CHECK(pos != std::string::npos);
    real_body.replace(pos, std::string("operation_mode: simulation").size(), "operation_mode: real");
    const std::string real_path = writeTempConfig("controller-sim-real", real_body);
    RB_CHECK(loadRejects(real_path));
    ::unlink(real_path.c_str());

    std::string diag_without_motion_body = valid_body;
    const std::string motion_key = "  allow_controller_simulation_motion: true\n";
    const std::size_t motion_pos = diag_without_motion_body.find(motion_key);
    RB_CHECK(motion_pos != std::string::npos);
    diag_without_motion_body.erase(motion_pos, motion_key.size());
    const std::string diag_without_motion_path =
        writeTempConfig("controller-sim-diag-without-motion", diag_without_motion_body);
    RB_CHECK(loadRejects(diag_without_motion_path));
    ::unlink(diag_without_motion_path.c_str());

    std::string init_without_motion_body = valid_body;
    const std::size_t init_motion_pos = init_without_motion_body.find(motion_key);
    RB_CHECK(init_motion_pos != std::string::npos);
    init_without_motion_body.erase(init_motion_pos, motion_key.size());
    const std::string diag_key = "  allow_controller_simulation_diagnostics_suspect: true\n";
    const std::size_t diag_pos = init_without_motion_body.find(diag_key);
    RB_CHECK(diag_pos != std::string::npos);
    init_without_motion_body.erase(diag_pos, diag_key.size());
    const std::string init_without_motion_path =
        writeTempConfig("controller-sim-init-without-motion", init_without_motion_body);
    RB_CHECK(loadRejects(init_without_motion_path));
    ::unlink(init_without_motion_path.c_str());

    std::string not_activated_without_motion_body = valid_body;
    const std::size_t not_activated_motion_pos = not_activated_without_motion_body.find(motion_key);
    RB_CHECK(not_activated_motion_pos != std::string::npos);
    not_activated_without_motion_body.erase(not_activated_motion_pos, motion_key.size());
    const std::size_t not_activated_diag_pos = not_activated_without_motion_body.find(diag_key);
    RB_CHECK(not_activated_diag_pos != std::string::npos);
    not_activated_without_motion_body.erase(not_activated_diag_pos, diag_key.size());
    const std::string init_key = "  allow_controller_simulation_init_error: true\n";
    const std::size_t not_activated_init_pos = not_activated_without_motion_body.find(init_key);
    RB_CHECK(not_activated_init_pos != std::string::npos);
    not_activated_without_motion_body.erase(not_activated_init_pos, init_key.size());
    const std::string not_activated_without_motion_path =
        writeTempConfig("controller-sim-not-activated-without-motion", not_activated_without_motion_body);
    RB_CHECK(loadRejects(not_activated_without_motion_path));
    ::unlink(not_activated_without_motion_path.c_str());

    std::string read_only_body = valid_body;
    const std::string send_true = "  send_servo_commands: true\n";
    const std::size_t send_pos = read_only_body.find(send_true);
    RB_CHECK(send_pos != std::string::npos);
    read_only_body.replace(send_pos, send_true.size(), "  send_servo_commands: false\n");
    const std::string read_only_path =
        writeTempConfig("controller-sim-read-only", read_only_body);
    RB_CHECK(loadRejects(read_only_path));
    ::unlink(read_only_path.c_str());

    return true;
}

bool testRbpodoAsyncStreamingConfigContract() {
    const std::string default_path = writeTempConfig(
        "async-default",
        "schema: robotics_lab.rb_servo_server.v1\n"
    );
    const rb_servo::DualArmConfig default_cfg = rb_servo::loadConfigFromYaml(default_path);
    ::unlink(default_path.c_str());
    RB_CHECK(!default_cfg.servo.rbpodo_async_streaming.enable);
    RB_CHECK(
        default_cfg.servo.rbpodo_async_streaming.mode ==
        rb_servo::RbpodoAsyncStreamingMode::Disabled
    );
    RB_CHECK(default_cfg.servo.rbpodo_async_streaming.rate_hz == 500);
    RB_CHECK(
        default_cfg.servo.rbpodo_async_streaming.queue_policy ==
        rb_servo::RbpodoAsyncQueuePolicy::LatestWins
    );

    EnvVarGuard allow_real("RB_ALLOW_REAL_ROBOT");
    EnvVarGuard allow_motion("RB_ALLOW_REAL_MOTION");

    allow_real.set("1");
    allow_motion.set("1");

    const std::string valid_sdk_path = writeTempConfig(
        "async-sdk-valid",
        asyncControllerSimulationBody("sdk_ack_worker")
    );
    const rb_servo::DualArmConfig sdk_cfg = rb_servo::loadConfigFromYaml(valid_sdk_path);
    ::unlink(valid_sdk_path.c_str());
    RB_CHECK(sdk_cfg.servo.rbpodo_async_streaming.enable);
    RB_CHECK(
        sdk_cfg.servo.rbpodo_async_streaming.mode ==
        rb_servo::RbpodoAsyncStreamingMode::SdkAckWorker
    );
    RB_CHECK(sdk_cfg.servo.rbpodo_async_streaming.rate_hz == 500);
    RB_CHECK(sdk_cfg.servo.rbpodo_async_streaming.ack_supervision.enable);
    RB_CHECK(sdk_cfg.servo.rbpodo_async_streaming.reference_supervision.enable);

    const std::string real_mode_path = writeTempConfig(
        "async-real-operation-reject",
        asyncControllerSimulationBody("sdk_ack_worker", "real", "simulation")
    );
    RB_CHECK(loadRejects(real_mode_path));
    ::unlink(real_mode_path.c_str());

    const std::string valid_socket_path = writeTempConfig(
        "async-socket-valid",
        asyncControllerSimulationBody("socket_send_supervised", "simulation", "simulation", true)
    );
    const rb_servo::DualArmConfig socket_cfg = rb_servo::loadConfigFromYaml(valid_socket_path);
    ::unlink(valid_socket_path.c_str());
    RB_CHECK(
        socket_cfg.servo.rbpodo_async_streaming.mode ==
        rb_servo::RbpodoAsyncStreamingMode::SocketSendSupervised
    );
    RB_CHECK(socket_cfg.left_robot.disable_waiting_ack);
    RB_CHECK(socket_cfg.right_robot.disable_waiting_ack);

    return true;
}

bool testStatePublisherEndpointsParseAndValidate() {
    const std::string valid_path = writeTempConfig(
        "state-pub-endpoints-valid",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "network:\n"
        "  state_pub_endpoints:\n"
        "    - \"udp://127.0.0.1:50151\"\n"
        "    - \"udp://rb_gui:50161\"\n"
    );
    const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(valid_path);
    ::unlink(valid_path.c_str());
    RB_CHECK(cfg.network.state_pub_endpoint == "udp://127.0.0.1:50151");
    RB_CHECK(cfg.network.state_pub_bind == cfg.network.state_pub_endpoint);
    RB_CHECK(cfg.network.state_pub_endpoints.size() == 2);
    RB_CHECK(cfg.network.state_pub_endpoints[0] == "udp://127.0.0.1:50151");
    RB_CHECK(cfg.network.state_pub_endpoints[1] == "udp://rb_gui:50161");

    const std::string duplicate_path = writeTempConfig(
        "state-pub-endpoints-duplicate",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "network:\n"
        "  state_pub_endpoints:\n"
        "    - \"udp://127.0.0.1:50151\"\n"
        "    - \"udp://127.0.0.1:50151\"\n"
        "    - \"udp://127.0.0.1:50161\"\n"
    );
    const rb_servo::DualArmConfig duplicate_cfg = rb_servo::loadConfigFromYaml(duplicate_path);
    ::unlink(duplicate_path.c_str());
    RB_CHECK(duplicate_cfg.network.state_pub_endpoints.size() == 2);
    RB_CHECK(duplicate_cfg.network.state_pub_endpoints[0] == "udp://127.0.0.1:50151");
    RB_CHECK(duplicate_cfg.network.state_pub_endpoints[1] == "udp://127.0.0.1:50161");

    const std::string empty_path = writeTempConfig(
        "state-pub-endpoints-empty",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "network:\n"
        "  state_pub_endpoints: []\n"
    );
    RB_CHECK(loadRejects(empty_path));
    ::unlink(empty_path.c_str());

    const std::string mixed_path = writeTempConfig(
        "state-pub-endpoints-mixed",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "network:\n"
        "  state_pub_endpoint: \"udp://127.0.0.1:50151\"\n"
        "  state_pub_endpoints:\n"
        "    - \"udp://127.0.0.1:50161\"\n"
    );
    RB_CHECK(loadRejects(mixed_path));
    ::unlink(mixed_path.c_str());

    const std::string invalid_scheme_path = writeTempConfig(
        "state-pub-endpoints-invalid-scheme",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "network:\n"
        "  state_pub_endpoints:\n"
        "    - \"tcp://127.0.0.1:50151\"\n"
    );
    RB_CHECK(loadRejects(invalid_scheme_path));
    ::unlink(invalid_scheme_path.c_str());

    const std::string invalid_port_path = writeTempConfig(
        "state-pub-endpoints-invalid-port",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "network:\n"
        "  state_pub_endpoint: \"udp://127.0.0.1:0\"\n"
    );
    RB_CHECK(loadRejects(invalid_port_path));
    ::unlink(invalid_port_path.c_str());

    return true;
}

}  // namespace

int main() {
    if (!testJointWrapConfigParses()) return 1;
    if (!testInvalidJointWrapConfigRejects()) return 1;
    if (!testControllerSimulationGateConfig()) return 1;
    if (!testRbpodoAsyncStreamingConfigContract()) return 1;
    if (!testStatePublisherEndpointsParseAndValidate()) return 1;
    return 0;
}
