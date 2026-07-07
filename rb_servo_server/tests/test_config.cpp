#include <cmath>
#include <cstdlib>
#include <filesystem>
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

bool loadRejectsWithMessage(const std::string& path, const std::string& needle) {
    try {
        (void)rb_servo::loadConfigFromYaml(path);
    } catch (const std::exception& exc) {
        return std::string(exc.what()).find(needle) != std::string::npos;
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

bool testJointTargetLiteralAxesConfigParses() {
    const std::string path = writeTempConfig(
        "literal-axes-valid",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "safety:\n"
        "  joint_target_literal_axes: [false, false, false, false, false, true]\n"
    );
    const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
    ::unlink(path.c_str());
    for (int i = 0; i < rb_servo::kDof - 1; ++i) {
        RB_CHECK(!cfg.safety.joint_target_literal_axes[static_cast<std::size_t>(i)]);
    }
    RB_CHECK(cfg.safety.joint_target_literal_axes[5]);
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

bool testInvalidJointTargetLiteralAxesConfigRejects() {
    const std::string wrong_length_path = writeTempConfig(
        "literal-axes-wrong-length",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "safety:\n"
        "  joint_target_literal_axes: [false, false, false, false, false]\n"
    );
    RB_CHECK(loadRejects(wrong_length_path));
    ::unlink(wrong_length_path.c_str());

    const std::string non_bool_path = writeTempConfig(
        "literal-axes-non-bool",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "safety:\n"
        "  joint_target_literal_axes: [false, false, false, false, false, not_bool]\n"
    );
    RB_CHECK(loadRejects(non_bool_path));
    ::unlink(non_bool_path.c_str());

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
        "  controller_simulation_treat_unreliable_status_fields_as_unavailable: true\n"
        "  allow_controller_simulation_init_error: true\n"
        "  allow_controller_simulation_not_activated: true\n"
        "safety:\n"
        "  tracking_error_policy: fault_latch\n"
        "  controller_simulation_tracking_error_source: reference\n"
        "  controller_simulation_physical_motion_policy: fault_latch\n"
        "  controller_simulation_physical_motion_threshold_deg: 0.05\n"
        "cartesian_control:\n"
        "  allow_in_controller_simulation: true\n";

    // Real/sim env gates retired: the config loads without RB_ALLOW_REAL_* envs.
    const std::string missing_real_env_path = writeTempConfig("controller-sim-missing-real-env", valid_body);
    (void)rb_servo::loadConfigFromYaml(missing_real_env_path);
    ::unlink(missing_real_env_path.c_str());

    allow_real.set("1");
    allow_motion.set("1");

    const std::string valid_path = writeTempConfig("controller-sim-valid", valid_body);
    const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(valid_path);
    ::unlink(valid_path.c_str());
    RB_CHECK(cfg.servo.allow_controller_simulation_motion);
    RB_CHECK(cfg.servo.allow_controller_simulation_diagnostics_suspect);
    RB_CHECK(cfg.servo.controller_simulation_treat_unreliable_status_fields_as_unavailable);
    RB_CHECK(cfg.servo.allow_controller_simulation_init_error);
    RB_CHECK(cfg.servo.allow_controller_simulation_not_activated);
    RB_CHECK(cfg.cartesian_control.allow_in_controller_simulation);
    RB_CHECK(cfg.left_robot.allow_controller_simulation_diagnostics_suspect);
    RB_CHECK(cfg.right_robot.allow_controller_simulation_diagnostics_suspect);
    RB_CHECK(cfg.left_robot.controller_simulation_treat_unreliable_status_fields_as_unavailable);
    RB_CHECK(cfg.right_robot.controller_simulation_treat_unreliable_status_fields_as_unavailable);
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
    const std::string unavailable_key =
        "  controller_simulation_treat_unreliable_status_fields_as_unavailable: true\n";
    const std::size_t unavailable_pos = init_without_motion_body.find(unavailable_key);
    RB_CHECK(unavailable_pos != std::string::npos);
    init_without_motion_body.erase(unavailable_pos, unavailable_key.size());
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
    const std::size_t not_activated_unavailable_pos =
        not_activated_without_motion_body.find(unavailable_key);
    RB_CHECK(not_activated_unavailable_pos != std::string::npos);
    not_activated_without_motion_body.erase(not_activated_unavailable_pos, unavailable_key.size());
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

    std::string unavailable_without_motion_body = valid_body;
    const std::size_t unavailable_motion_pos = unavailable_without_motion_body.find(motion_key);
    RB_CHECK(unavailable_motion_pos != std::string::npos);
    unavailable_without_motion_body.erase(unavailable_motion_pos, motion_key.size());
    const std::size_t unavailable_diag_pos = unavailable_without_motion_body.find(diag_key);
    RB_CHECK(unavailable_diag_pos != std::string::npos);
    unavailable_without_motion_body.erase(unavailable_diag_pos, diag_key.size());
    const std::string unavailable_without_motion_path =
        writeTempConfig("controller-sim-unavailable-without-motion", unavailable_without_motion_body);
    RB_CHECK(loadRejects(unavailable_without_motion_path));
    ::unlink(unavailable_without_motion_path.c_str());

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

bool testScopeConfigParsesAndValidates() {
    const std::string default_path = writeTempConfig(
        "scope-defaults",
        "schema: robotics_lab.rb_servo_server.v1\n"
    );
    const rb_servo::DualArmConfig defaults = rb_servo::loadConfigFromYaml(default_path);
    ::unlink(default_path.c_str());
    RB_CHECK(!defaults.scope.enable);
    RB_CHECK(defaults.scope.publish_rate_hz == 100);
    RB_CHECK(defaults.scope.max_samples_per_batch == 64);
    RB_CHECK(defaults.network.scope_pub_endpoints.size() == 1);
    RB_CHECK(defaults.network.scope_pub_endpoints[0] == "udp://127.0.0.1:50357");

    const std::string valid_path = writeTempConfig(
        "scope-valid",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "network:\n"
        "  scope_pub_endpoints:\n"
        "    - \"udp://127.0.0.1:50357\"\n"
        "    - \"udp://rb_gui:50358\"\n"
        "    - \"udp://127.0.0.1:50357\"\n"
        "scope:\n"
        "  enable: true\n"
        "  publish_rate_hz: 125\n"
        "  max_samples_per_batch: 32\n"
    );
    const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(valid_path);
    ::unlink(valid_path.c_str());
    RB_CHECK(cfg.scope.enable);
    RB_CHECK(cfg.scope.publish_rate_hz == 125);
    RB_CHECK(cfg.scope.max_samples_per_batch == 32);
    RB_CHECK(cfg.network.scope_pub_endpoints.size() == 2);
    RB_CHECK(cfg.network.scope_pub_endpoints[0] == "udp://127.0.0.1:50357");
    RB_CHECK(cfg.network.scope_pub_endpoints[1] == "udp://rb_gui:50358");

    const std::string empty_endpoints_path = writeTempConfig(
        "scope-empty-endpoints",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "network:\n"
        "  scope_pub_endpoints: []\n"
    );
    RB_CHECK(loadRejects(empty_endpoints_path));
    ::unlink(empty_endpoints_path.c_str());

    const std::string invalid_endpoint_path = writeTempConfig(
        "scope-invalid-endpoint",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "network:\n"
        "  scope_pub_endpoints:\n"
        "    - \"tcp://127.0.0.1:50357\"\n"
    );
    RB_CHECK(loadRejects(invalid_endpoint_path));
    ::unlink(invalid_endpoint_path.c_str());

    const std::string invalid_rate_path = writeTempConfig(
        "scope-invalid-rate",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "scope:\n"
        "  publish_rate_hz: 0\n"
    );
    RB_CHECK(loadRejects(invalid_rate_path));
    ::unlink(invalid_rate_path.c_str());

    const std::string invalid_batch_path = writeTempConfig(
        "scope-invalid-batch",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "scope:\n"
        "  max_samples_per_batch: 0\n"
    );
    RB_CHECK(loadRejects(invalid_batch_path));
    ::unlink(invalid_batch_path.c_str());

    return true;
}

bool testFloorConstraintConfigParsesAndDefaults() {
    // Values parse (enable=false skips the kinematics requirement).
    const std::string path = writeTempConfig(
        "floor-values",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  send_servo_commands: false\n"
        "safety:\n"
        "  floor_constraint:\n"
        "    enable: false\n"
        "    z_min_m: 0.02\n"
        "    runtime_min_z_m: 0.005\n"
        "    runtime_max_z_m: 0.3\n"
        "    fail_policy: fault_latch\n"
        "    monitor_only: true\n"
    );
    const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
    ::unlink(path.c_str());
    RB_CHECK(!cfg.safety.floor_constraint.enable);
    RB_CHECK(near(cfg.safety.floor_constraint.z_min_m, 0.02));
    RB_CHECK(near(cfg.safety.floor_constraint.runtime_min_z_m, 0.005));
    RB_CHECK(near(cfg.safety.floor_constraint.runtime_max_z_m, 0.3));
    RB_CHECK(cfg.safety.floor_constraint.fail_policy ==
             rb_servo::FloorConstraintFailPolicy::FaultLatch);
    RB_CHECK(cfg.safety.floor_constraint.monitor_only);

    // Defaults when the block is absent.
    const std::string default_path = writeTempConfig(
        "floor-defaults",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  send_servo_commands: false\n"
    );
    const rb_servo::DualArmConfig defaults = rb_servo::loadConfigFromYaml(default_path);
    ::unlink(default_path.c_str());
    RB_CHECK(!defaults.safety.floor_constraint.enable);
    RB_CHECK(near(defaults.safety.floor_constraint.z_min_m, 0.010));
    RB_CHECK(near(defaults.safety.floor_constraint.runtime_min_z_m, -0.2));
    RB_CHECK(near(defaults.safety.floor_constraint.runtime_max_z_m, 0.5));
    RB_CHECK(defaults.safety.floor_constraint.fail_policy ==
             rb_servo::FloorConstraintFailPolicy::ClampToHold);
    RB_CHECK(!defaults.safety.floor_constraint.monitor_only);
    return true;
}

bool testFloorConstraintInvalidConfigRejects() {
    // Unknown key inside the block.
    const std::string unknown_key = writeTempConfig(
        "floor-unknown-key",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  send_servo_commands: false\n"
        "safety:\n"
        "  floor_constraint:\n"
        "    enable: false\n"
        "    z_max_m: 0.1\n"
    );
    RB_CHECK(loadRejects(unknown_key));
    ::unlink(unknown_key.c_str());

    // Unknown fail_policy string.
    const std::string bad_policy = writeTempConfig(
        "floor-bad-policy",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  send_servo_commands: false\n"
        "safety:\n"
        "  floor_constraint:\n"
        "    fail_policy: stop\n"
    );
    RB_CHECK(loadRejects(bad_policy));
    ::unlink(bad_policy.c_str());

    // z_min outside the runtime envelope (enable=true triggers validation).
    const std::string bad_bounds = writeTempConfig(
        "floor-bad-bounds",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  send_servo_commands: false\n"
        "safety:\n"
        "  floor_constraint:\n"
        "    enable: true\n"
        "    z_min_m: 0.6\n"
        "    runtime_max_z_m: 0.5\n"
    );
    RB_CHECK(loadRejects(bad_bounds));
    ::unlink(bad_bounds.c_str());

    // enable=true without kinematics (no FK source) is fail-closed at load time.
    const std::string no_kinematics = writeTempConfig(
        "floor-no-kinematics",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  send_servo_commands: false\n"
        "safety:\n"
        "  floor_constraint:\n"
        "    enable: true\n"
    );
    RB_CHECK(loadRejects(no_kinematics));
    ::unlink(no_kinematics.c_str());
    return true;
}

bool testFloorCheckPointOffsetClosedParses() {
    // offset_closed_m is optional: present -> has_closed + both stored; absent ->
    // mirror offset_m (static point). enable=false skips the kinematics requirement.
    const std::string path = writeTempConfig(
        "floor-offset-closed",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  send_servo_commands: false\n"
        "safety:\n"
        "  floor_constraint:\n"
        "    enable: false\n"
        "    tcp_offset_points:\n"
        "      - { name: tip_a, offset_m: [0.057, 0.012, 0.0], offset_closed_m: [0.010, 0.012, 0.0] }\n"
        "      - { name: tip_b, offset_m: [-0.057, -0.012, 0.0] }\n"
    );
    const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
    ::unlink(path.c_str());
    const auto& pts = cfg.safety.floor_constraint.tcp_offset_points;
    RB_CHECK(pts.size() == 2);
    RB_CHECK(pts[0].name == "tip_a");
    RB_CHECK(pts[0].has_closed);
    RB_CHECK(near(pts[0].offset_m[0], 0.057));
    RB_CHECK(near(pts[0].offset_closed_m[0], 0.010));
    // Absent offset_closed_m mirrors offset_m and is flagged static (identity interp).
    RB_CHECK(!pts[1].has_closed);
    RB_CHECK(near(pts[1].offset_closed_m[0], -0.057));
    RB_CHECK(near(pts[1].offset_closed_m[1], -0.012));

    // Malformed offset_closed_m (wrong length) is rejected at parse time.
    const std::string bad = writeTempConfig(
        "floor-offset-closed-bad",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  send_servo_commands: false\n"
        "safety:\n"
        "  floor_constraint:\n"
        "    enable: false\n"
        "    tcp_offset_points:\n"
        "      - { name: tip_a, offset_m: [0.057, 0.012, 0.0], offset_closed_m: [0.010, 0.012] }\n"
    );
    RB_CHECK(loadRejects(bad));
    ::unlink(bad.c_str());
    return true;
}

bool testRoiBoxConfigParsesAndDefaults() {
    // Values parse (enable=false skips the kinematics requirement).
    const std::string path = writeTempConfig(
        "roi-values",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  send_servo_commands: false\n"
        "safety:\n"
        "  roi_box:\n"
        "    enable: false\n"
        "    min_m: [-0.4, -0.9, 0.05]\n"
        "    max_m: [0.4, 0.1, 0.9]\n"
        "    runtime_min_m: [-1.0, -1.5, -0.2]\n"
        "    runtime_max_m: [1.0, 0.5, 1.5]\n"
        "    fail_policy: fault_latch\n"
        "    monitor_only: true\n"
        "    a_brake_m_s2: 3.0\n"
        "    d_slow_m: 0.04\n"
    );
    const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
    ::unlink(path.c_str());
    RB_CHECK(!cfg.safety.roi_box.enable);
    RB_CHECK(near(cfg.safety.roi_box.min_m[0], -0.4));
    RB_CHECK(near(cfg.safety.roi_box.max_m[2], 0.9));
    RB_CHECK(near(cfg.safety.roi_box.runtime_min_m[1], -1.5));
    RB_CHECK(cfg.safety.roi_box.fail_policy == rb_servo::FloorConstraintFailPolicy::FaultLatch);
    RB_CHECK(cfg.safety.roi_box.monitor_only);
    RB_CHECK(near(cfg.safety.roi_box.a_brake_m_s2, 3.0));
    RB_CHECK(near(cfg.safety.roi_box.d_slow_m, 0.04));

    // Defaults when the block is absent.
    const std::string default_path = writeTempConfig(
        "roi-defaults",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  send_servo_commands: false\n"
    );
    const rb_servo::DualArmConfig defaults = rb_servo::loadConfigFromYaml(default_path);
    ::unlink(default_path.c_str());
    RB_CHECK(!defaults.safety.roi_box.enable);
    RB_CHECK(near(defaults.safety.roi_box.min_m[0], -0.5));
    RB_CHECK(near(defaults.safety.roi_box.max_m[1], 0.0));
    RB_CHECK(defaults.safety.roi_box.fail_policy == rb_servo::FloorConstraintFailPolicy::ClampToHold);
    return true;
}

bool testRoiBoxInvalidConfigRejects() {
    // Unknown key inside the block.
    const std::string unknown_key = writeTempConfig(
        "roi-unknown-key",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  send_servo_commands: false\n"
        "safety:\n"
        "  roi_box:\n"
        "    enable: false\n"
        "    depth_m: 0.1\n"
    );
    RB_CHECK(loadRejects(unknown_key));
    ::unlink(unknown_key.c_str());

    // min above max on an axis (enable=true triggers validation).
    const std::string bad_order = writeTempConfig(
        "roi-bad-order",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  send_servo_commands: false\n"
        "safety:\n"
        "  roi_box:\n"
        "    enable: true\n"
        "    min_m: [0.6, -1.0, 0.0]\n"
        "    max_m: [0.5, 0.0, 1.0]\n"
        "    runtime_min_m: [-1.0, -1.5, -0.2]\n"
        "    runtime_max_m: [1.0, 0.5, 1.5]\n"
    );
    RB_CHECK(loadRejects(bad_order));
    ::unlink(bad_order.c_str());

    // min below the runtime envelope.
    const std::string out_of_envelope = writeTempConfig(
        "roi-out-of-envelope",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  send_servo_commands: false\n"
        "safety:\n"
        "  roi_box:\n"
        "    enable: true\n"
        "    min_m: [-2.0, -1.0, 0.0]\n"
        "    max_m: [0.5, 0.0, 1.0]\n"
        "    runtime_min_m: [-1.0, -1.5, -0.2]\n"
        "    runtime_max_m: [1.0, 0.5, 1.5]\n"
    );
    RB_CHECK(loadRejects(out_of_envelope));
    ::unlink(out_of_envelope.c_str());

    // enable=true without kinematics (no FK source) is fail-closed at load time.
    const std::string no_kinematics = writeTempConfig(
        "roi-no-kinematics",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  send_servo_commands: false\n"
        "safety:\n"
        "  roi_box:\n"
        "    enable: true\n"
    );
    RB_CHECK(loadRejects(no_kinematics));
    ::unlink(no_kinematics.c_str());
    return true;
}

std::string initMotionConfigBody(const std::string& init_block) {
    const std::string rb3_urdf =
        (std::filesystem::path(__FILE__).parent_path().parent_path() /
         "descriptions/urdf/rb3_730e.urdf").string();
    return std::string(
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  send_servo_commands: false\n"
        "kinematics:\n"
        "  enable: true\n"
        "  provider: pinocchio\n"
        "  urdf: \"") + rb3_urdf + "\"\n" +
        "  base_link: world\n"
        "  tip_link: tcp\n"
        "  joint_names: [base_joint, shoulder_joint, elbow_joint, wrist1_joint, wrist2_joint, wrist3_joint]\n"
        "  q_units: deg\n"
        "safety:\n"
        "  self_collision:\n"
        "    enable: true\n"
        "    mesh:\n"
        "      unified_urdf: dummy.urdf\n"
        "      d_hard_m: 0.010\n"
        "      d_slow_m: 0.035\n"
        "      a_brake_m_s2: 4.0\n"
        "      max_staleness_s: 0.050\n"
        "  init_motion_planner:\n" +
        init_block;
}

std::string selfCollisionMeshConfigBody(const std::string& mesh_extra) {
    const std::string rb3_urdf =
        (std::filesystem::path(__FILE__).parent_path().parent_path() /
         "descriptions/urdf/rb3_730e.urdf").string();
    return std::string(
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  send_servo_commands: false\n"
        "kinematics:\n"
        "  enable: true\n"
        "  provider: pinocchio\n"
        "  urdf: \"") + rb3_urdf + "\"\n" +
        "  base_link: world\n"
        "  tip_link: tcp\n"
        "  joint_names: [base_joint, shoulder_joint, elbow_joint, wrist1_joint, wrist2_joint, wrist3_joint]\n"
        "  q_units: deg\n"
        "safety:\n"
        "  self_collision:\n"
        "    enable: true\n"
        "    mesh:\n"
        "      unified_urdf: dummy.urdf\n"
        "      d_hard_m: 0.010\n"
        "      d_slow_m: 0.035\n" +
        mesh_extra;
}

bool testDisabledCollisionPairsConfig() {
    const std::string valid_path = writeTempConfig(
        "disabled-collision-pairs-valid",
        selfCollisionMeshConfigBody(
            "      disabled_collision_pairs:\n"
            "        - [\"*left*link0*\", \"*left*link1*\"]\n"
            "        - [\"*right*link0*\", \"*right*link1*\"]\n"
            "      debug_pair_curation: true\n"
        )
    );
    const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(valid_path);
    ::unlink(valid_path.c_str());
    const auto& mesh = cfg.safety.self_collision.mesh;
    RB_CHECK(mesh.disabled_collision_pairs.size() == 2);
    RB_CHECK(mesh.disabled_collision_pairs[0].pattern_a == "*left*link0*");
    RB_CHECK(mesh.disabled_collision_pairs[0].pattern_b == "*left*link1*");
    RB_CHECK(mesh.disabled_collision_pairs[1].pattern_a == "*right*link0*");
    RB_CHECK(mesh.disabled_collision_pairs[1].pattern_b == "*right*link1*");
    RB_CHECK(mesh.debug_pair_curation);

    const std::string bad_length_path = writeTempConfig(
        "disabled-collision-pairs-bad-length",
        selfCollisionMeshConfigBody(
            "      disabled_collision_pairs:\n"
            "        - [\"*right*link0*\"]\n"
        )
    );
    RB_CHECK(loadRejects(bad_length_path));
    ::unlink(bad_length_path.c_str());

    const std::string bad_shape_path = writeTempConfig(
        "disabled-collision-pairs-bad-shape",
        selfCollisionMeshConfigBody(
            "      disabled_collision_pairs: \"*right*link0*\"\n"
        )
    );
    RB_CHECK(loadRejects(bad_shape_path));
    ::unlink(bad_shape_path.c_str());

    return true;
}

bool testIntraArmSelfCollisionConfig() {
    const std::string valid_path = writeTempConfig(
        "intra-arm-self-collision-valid",
        selfCollisionMeshConfigBody(
            "      intra_arm:\n"
            "        d_hard_m: 0.005\n"
            "        d_slow_m: 0.015\n"
            "        a_brake_m_s2: 3.0\n"
            "        hyst_m: 0.004\n"
            "        recover_speed_m_s: 0.002\n"
            "        latency_s: 0.006\n"
        )
    );
    const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(valid_path);
    ::unlink(valid_path.c_str());
    const auto& intra = cfg.safety.self_collision.mesh.intra_arm;
    RB_CHECK(near(intra.d_hard_m, 0.005));
    RB_CHECK(near(intra.d_slow_m, 0.015));
    RB_CHECK(near(intra.a_brake_m_s2, 3.0));
    RB_CHECK(near(intra.hyst_m, 0.004));
    RB_CHECK(near(intra.recover_speed_m_s, 0.002));
    RB_CHECK(near(intra.latency_s, 0.006));

    const std::string default_path = writeTempConfig(
        "intra-arm-self-collision-default",
        selfCollisionMeshConfigBody("")
    );
    const rb_servo::DualArmConfig default_cfg = rb_servo::loadConfigFromYaml(default_path);
    ::unlink(default_path.c_str());
    RB_CHECK(default_cfg.safety.self_collision.mesh.intra_arm.d_hard_m < 0.0);
    RB_CHECK(default_cfg.safety.self_collision.mesh.intra_arm.d_slow_m < 0.0);
    RB_CHECK(default_cfg.safety.self_collision.mesh.intra_arm.a_brake_m_s2 < 0.0);

    const std::string bad_band_path = writeTempConfig(
        "intra-arm-self-collision-bad-band",
        selfCollisionMeshConfigBody(
            "      intra_arm:\n"
            "        d_hard_m: 0.020\n"
            "        d_slow_m: 0.010\n"
        )
    );
    RB_CHECK(loadRejects(bad_band_path));
    ::unlink(bad_band_path.c_str());

    const std::string unknown_key_path = writeTempConfig(
        "intra-arm-self-collision-unknown-key",
        selfCollisionMeshConfigBody(
            "      intra_arm:\n"
            "        d_hard_m: 0.005\n"
            "        typo: 0.010\n"
        )
    );
    RB_CHECK(loadRejects(unknown_key_path));
    ::unlink(unknown_key_path.c_str());
    return true;
}

bool testInitMotionPlannerConfigExt() {
    const std::string valid_path = writeTempConfig(
        "init-motion-valid",
        initMotionConfigBody(
            "    enable: true\n"
            "    goal_bias: 0.25\n"
            "    sample_margin_deg_per_joint: [45, 75, 75, 45, 75, 30]\n"
            "    global_sample_fraction: 0.15\n"
            "    global_sample_margin_deg: 150.0\n"
            "    escape_max_time_sec: 0.75\n"
            "    escape_max_steps: 40\n"
            "    escape_restart_attempts: 4\n"
            "    escape_perturb_deg: 5.0\n"
            "    lazy_edges: true\n"
        )
    );
    const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(valid_path);
    ::unlink(valid_path.c_str());
    RB_CHECK(near(cfg.safety.init_motion_planner.goal_bias, 0.25));
    RB_CHECK(near(cfg.safety.init_motion_planner.sample_margin_deg_per_joint[2], 75.0));
    RB_CHECK(near(cfg.safety.init_motion_planner.global_sample_fraction, 0.15));
    RB_CHECK(cfg.safety.init_motion_planner.lazy_edges);

    const std::string bad_sum_path = writeTempConfig(
        "init-motion-bad-sum",
        initMotionConfigBody(
            "    enable: true\n"
            "    goal_bias: 0.9\n"
            "    global_sample_fraction: 0.2\n"
        )
    );
    RB_CHECK(loadRejects(bad_sum_path));
    ::unlink(bad_sum_path.c_str());

    const std::string bad_len_path = writeTempConfig(
        "init-motion-bad-margin-len",
        initMotionConfigBody(
            "    enable: true\n"
            "    sample_margin_deg_per_joint: [45, 75, 75, 45, 75]\n"
        )
    );
    RB_CHECK(loadRejects(bad_len_path));
    ::unlink(bad_len_path.c_str());

    return true;
}

bool testRuckigFollowerFallbackPolicyConfig() {
    const std::string default_path = writeTempConfig(
        "ruckig-follower-fallback-default",
        "schema: robotics_lab.rb_servo_server.v1\n"
    );
    const rb_servo::DualArmConfig defaults = rb_servo::loadConfigFromYaml(default_path);
    ::unlink(default_path.c_str());
    RB_CHECK(defaults.cartesian_control.ruckig_follower.fallback_policy ==
             rb_servo::RuckigFollowerFallbackPolicy::Smd);
    RB_CHECK(near(defaults.cartesian_control.ruckig_follower.engage_timeout_sec, 3.0));
    RB_CHECK(!defaults.cartesian_control.tcp_pose_target_profiles.empty());
    RB_CHECK(defaults.cartesian_control.tcp_pose_target_profiles.front().ruckig_follower.fallback_policy ==
             rb_servo::RuckigFollowerFallbackPolicy::Smd);
    RB_CHECK(near(
        defaults.cartesian_control.tcp_pose_target_profiles.front().ruckig_follower.engage_timeout_sec,
        3.0
    ));

    const std::string fault_path = writeTempConfig(
        "ruckig-follower-fallback-fault",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "cartesian_control:\n"
        "  ruckig_follower:\n"
        "    fallback_policy: \"fault\"\n"
        "    engage_timeout_sec: 2.5\n"
        "  tcp_pose_target_profile_default: strict\n"
        "  tcp_pose_target_profiles:\n"
        "    strict:\n"
        "      ruckig_follower:\n"
        "        fallback_policy: \"fault\"\n"
        "        engage_timeout_sec: 1.25\n"
    );
    const rb_servo::DualArmConfig fault_cfg = rb_servo::loadConfigFromYaml(fault_path);
    ::unlink(fault_path.c_str());
    RB_CHECK(fault_cfg.cartesian_control.ruckig_follower.fallback_policy ==
             rb_servo::RuckigFollowerFallbackPolicy::Fault);
    RB_CHECK(near(fault_cfg.cartesian_control.ruckig_follower.engage_timeout_sec, 2.5));
    RB_CHECK(fault_cfg.cartesian_control.tcp_pose_target_profiles.size() == 1);
    RB_CHECK(fault_cfg.cartesian_control.tcp_pose_target_profiles.front().ruckig_follower.fallback_policy ==
             rb_servo::RuckigFollowerFallbackPolicy::Fault);
    RB_CHECK(near(
        fault_cfg.cartesian_control.tcp_pose_target_profiles.front().ruckig_follower.engage_timeout_sec,
        1.25
    ));

    const std::string bad_policy_path = writeTempConfig(
        "ruckig-follower-bad-policy",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "cartesian_control:\n"
        "  ruckig_follower:\n"
        "    fallback_policy: \"hold\"\n"
    );
    RB_CHECK(loadRejectsWithMessage(
        bad_policy_path,
        "cartesian_control.ruckig_follower.fallback_policy"
    ));
    ::unlink(bad_policy_path.c_str());

    const std::string zero_timeout_path = writeTempConfig(
        "ruckig-follower-zero-engage-timeout",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "cartesian_control:\n"
        "  ruckig_follower:\n"
        "    engage_timeout_sec: 0.0\n"
    );
    RB_CHECK(loadRejectsWithMessage(
        zero_timeout_path,
        "cartesian_control.ruckig_follower.engage_timeout_sec"
    ));
    ::unlink(zero_timeout_path.c_str());

    const std::string infinite_timeout_path = writeTempConfig(
        "ruckig-follower-inf-engage-timeout",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "cartesian_control:\n"
        "  ruckig_follower:\n"
        "    engage_timeout_sec: .inf\n"
    );
    RB_CHECK(loadRejectsWithMessage(
        infinite_timeout_path,
        "cartesian_control.ruckig_follower.engage_timeout_sec"
    ));
    ::unlink(infinite_timeout_path.c_str());

    return true;
}

bool testRuckigFollowerControllerConfig() {
    const std::string default_path = writeTempConfig(
        "ruckig-follower-controller-default",
        "schema: robotics_lab.rb_servo_server.v1\n"
    );
    const rb_servo::DualArmConfig defaults = rb_servo::loadConfigFromYaml(default_path);
    ::unlink(default_path.c_str());
    RB_CHECK(defaults.cartesian_control.ruckig_follower.controller ==
             rb_servo::RuckigFollowerController::RuckigWaypoint);
    RB_CHECK(defaults.cartesian_control.tcp_pose_target_profiles.front().ruckig_follower.controller ==
             rb_servo::RuckigFollowerController::RuckigWaypoint);

    const std::string delta_path = writeTempConfig(
        "ruckig-follower-controller-delta",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "cartesian_control:\n"
        "  ruckig_follower:\n"
        "    controller: \"delta_twist\"\n"
        "    delta_twist_tau_sec: 0.031\n"
        "    delta_twist_residual_drain_steps: 2\n"
        "    delta_twist_max_residual_m: 0.041\n"
        "    delta_twist_max_residual_rad: 0.42\n"
        "    delta_twist_max_lead_m: 0.091\n"
        "    delta_twist_max_lead_rad: 0.51\n"
        "  tcp_pose_target_profile_default: strict\n"
        "  tcp_pose_target_profiles:\n"
        "    strict:\n"
        "      ruckig_follower:\n"
        "        controller: \"ruckig_waypoint\"\n"
    );
    const rb_servo::DualArmConfig delta_cfg = rb_servo::loadConfigFromYaml(delta_path);
    ::unlink(delta_path.c_str());
    RB_CHECK(delta_cfg.cartesian_control.ruckig_follower.controller ==
             rb_servo::RuckigFollowerController::DeltaTwist);
    RB_CHECK(near(delta_cfg.cartesian_control.ruckig_follower.delta_twist_tau_sec, 0.031));
    RB_CHECK(delta_cfg.cartesian_control.ruckig_follower.delta_twist_residual_drain_steps == 2);
    RB_CHECK(near(delta_cfg.cartesian_control.ruckig_follower.delta_twist_max_residual_m, 0.041));
    RB_CHECK(near(delta_cfg.cartesian_control.ruckig_follower.delta_twist_max_residual_rad, 0.42));
    RB_CHECK(near(delta_cfg.cartesian_control.ruckig_follower.delta_twist_max_lead_m, 0.091));
    RB_CHECK(near(delta_cfg.cartesian_control.ruckig_follower.delta_twist_max_lead_rad, 0.51));
    RB_CHECK(delta_cfg.cartesian_control.tcp_pose_target_profiles.front().ruckig_follower.controller ==
             rb_servo::RuckigFollowerController::RuckigWaypoint);

    const std::string bad_controller_path = writeTempConfig(
        "ruckig-follower-bad-controller",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "cartesian_control:\n"
        "  ruckig_follower:\n"
        "    controller: \"central_difference\"\n"
    );
    RB_CHECK(loadRejectsWithMessage(
        bad_controller_path,
        "cartesian_control.ruckig_follower.controller"
    ));
    ::unlink(bad_controller_path.c_str());

    const std::string bad_tau_path = writeTempConfig(
        "ruckig-follower-bad-delta-tau",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "cartesian_control:\n"
        "  ruckig_follower:\n"
        "    delta_twist_tau_sec: 0.0\n"
    );
    RB_CHECK(loadRejectsWithMessage(
        bad_tau_path,
        "cartesian_control.ruckig_follower.delta_twist_tau_sec"
    ));
    ::unlink(bad_tau_path.c_str());

    return true;
}

bool testSendAtTickStartAndPipelinedReadConfig() {
    // Defaults: both jitter-decoupling flags stay off (legacy in-tick send +
    // blocking state read) unless a config opts in.
    const std::string default_path = writeTempConfig(
        "jitter-decoupling-default",
        "schema: robotics_lab.rb_servo_server.v1\n"
    );
    const rb_servo::DualArmConfig default_cfg = rb_servo::loadConfigFromYaml(default_path);
    ::unlink(default_path.c_str());
    RB_CHECK(!default_cfg.servo.send_at_tick_start);
    RB_CHECK(!default_cfg.left_robot.state_read_pipelined);
    RB_CHECK(!default_cfg.right_robot.state_read_pipelined);

    const std::string enabled_path = writeTempConfig(
        "jitter-decoupling-enabled",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "left_robot:\n"
        "  state_read_pipelined: true\n"
        "right_robot:\n"
        "  state_read_pipelined: true\n"
        "servo:\n"
        "  send_at_tick_start: true\n"
    );
    const rb_servo::DualArmConfig enabled_cfg = rb_servo::loadConfigFromYaml(enabled_path);
    ::unlink(enabled_path.c_str());
    RB_CHECK(enabled_cfg.servo.send_at_tick_start);
    RB_CHECK(enabled_cfg.left_robot.state_read_pipelined);
    RB_CHECK(enabled_cfg.right_robot.state_read_pipelined);

    return true;
}

}  // namespace

int main() {
    if (!testJointWrapConfigParses()) return 1;
    if (!testJointTargetLiteralAxesConfigParses()) return 1;
    if (!testInvalidJointWrapConfigRejects()) return 1;
    if (!testInvalidJointTargetLiteralAxesConfigRejects()) return 1;
    if (!testControllerSimulationGateConfig()) return 1;
    if (!testRbpodoAsyncStreamingConfigContract()) return 1;
    if (!testStatePublisherEndpointsParseAndValidate()) return 1;
    if (!testScopeConfigParsesAndValidates()) return 1;
    if (!testFloorConstraintConfigParsesAndDefaults()) return 1;
    if (!testFloorConstraintInvalidConfigRejects()) return 1;
    if (!testFloorCheckPointOffsetClosedParses()) return 1;
    if (!testRoiBoxConfigParsesAndDefaults()) return 1;
    if (!testRoiBoxInvalidConfigRejects()) return 1;
    if (!testDisabledCollisionPairsConfig()) return 1;
    if (!testIntraArmSelfCollisionConfig()) return 1;
    if (!testInitMotionPlannerConfigExt()) return 1;
    if (!testRuckigFollowerFallbackPolicyConfig()) return 1;
    if (!testRuckigFollowerControllerConfig()) return 1;
    if (!testSendAtTickStartAndPipelinedReadConfig()) return 1;
    return 0;
}
