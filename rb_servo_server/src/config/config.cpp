#include "rb_servo/config/config.hpp"

#include <arpa/inet.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace rb_servo {
namespace {

constexpr const char* kConfigSchema = "robotics_lab.rb_servo_server.v1";

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return s;
}

bool envIsOne(const char* name) {
    const char* value = std::getenv(name);
    return value && std::string(value) == "1";
}

std::string location(const YAML::Node& node) {
    const YAML::Mark mark = node.Mark();
    if (mark.line < 0) return {};
    return " at line " + std::to_string(mark.line + 1);
}

[[noreturn]] void fail(const std::string& message, const YAML::Node& node = YAML::Node()) {
    throw std::runtime_error(message + location(node));
}

void warn(const std::string& message) {
    std::cerr << "[WARN] " << message << "\n";
}

void warnDeprecatedValue(const std::string& path, const std::string& old_value, const std::string& new_value) {
    warn("deprecated config value " + path + "=" + old_value + "; use " + new_value);
}

void warnDeprecatedKey(const std::string& old_key, const std::string& new_key) {
    warn("deprecated config key " + old_key + "; use " + new_key);
}

void requireMapping(const YAML::Node& node, const std::string& path) {
    if (!node || node.IsNull()) return;
    if (!node.IsMap()) fail(path + " must be a YAML mapping", node);
}

void validateAllowedKeys(const YAML::Node& node, const std::set<std::string>& allowed, const std::string& path) {
    requireMapping(node, path);
    if (!node || node.IsNull()) return;
    for (const auto& item : node) {
        if (!item.first.IsScalar()) {
            fail(path + " contains a non-scalar key", item.first);
        }
        const std::string key = item.first.as<std::string>();
        if (allowed.find(key) == allowed.end()) {
            fail("Unknown config key " + path + "." + key, item.first);
        }
    }
}

bool has(const YAML::Node& node, const std::string& key) {
    return node && node.IsMap() && static_cast<bool>(node[key]);
}

template <typename T>
T asValue(const YAML::Node& node, const std::string& path) {
    try {
        return node.as<T>();
    } catch (const YAML::Exception& exc) {
        fail("Invalid value for " + path + ": " + exc.msg, node);
    }
}

std::string asString(const YAML::Node& node, const std::string& path) {
    if (!node || node.IsNull()) return "null";
    if (!node.IsScalar()) fail(path + " must be a scalar", node);
    return asValue<std::string>(node, path);
}

double asDouble(const YAML::Node& node, const std::string& path) {
    return asValue<double>(node, path);
}

int asInt(const YAML::Node& node, const std::string& path) {
    return asValue<int>(node, path);
}

bool asBool(const YAML::Node& node, const std::string& path) {
    return asValue<bool>(node, path);
}

std::vector<std::string> asStringArray(const YAML::Node& node, const std::string& path) {
    if (!node.IsSequence()) fail(path + " must be a sequence", node);
    std::vector<std::string> values;
    values.reserve(node.size());
    for (std::size_t i = 0; i < node.size(); ++i) {
        values.push_back(asString(node[i], path + "[" + std::to_string(i) + "]"));
    }
    return values;
}

JointArray parseJointArray(const YAML::Node& node, const std::string& path) {
    if (!node.IsSequence()) fail(path + " must be a sequence", node);
    if (node.size() != kDof) {
        fail(path + " must contain exactly 6 values", node);
    }
    JointArray out{};
    for (int i = 0; i < kDof; ++i) {
        out[static_cast<std::size_t>(i)] = asDouble(node[static_cast<std::size_t>(i)], path + "[" + std::to_string(i) + "]");
    }
    return out;
}

Pose6D parsePose6D(const YAML::Node& node, const std::string& path) {
    const JointArray v = parseJointArray(node, path);
    return Pose6D{v[0], v[1], v[2], v[3], v[4], v[5]};
}

BackendType parseBackendType(const YAML::Node& node, const std::string& path) {
    const std::string value = lower(asString(node, path));
    if (value == "mock") return BackendType::Mock;
    if (value == "rbpodo") return BackendType::Rbpodo;
    if (value == "simulator") return BackendType::Simulator;
    if (value == "rbscript_tcp") return BackendType::RbscriptTcp;
    if (value == "rbsim" || value == "rbsim_local") {
        warnDeprecatedValue(path, value, "simulator");
        return BackendType::Simulator;
    }
    fail("Unknown backend_type: " + value, node);
}

RunMode parseRunMode(const YAML::Node& node, const std::string& path) {
    const std::string value = lower(asString(node, path));
    if (value == "mock") return RunMode::Mock;
    if (value == "simulation") return RunMode::Simulation;
    if (value == "real") return RunMode::Real;
    if (value == "sim" || value == "rbsim" || value == "rbsim_local") {
        warnDeprecatedValue(path, value, "simulation");
        return RunMode::Simulation;
    }
    fail("Unknown run_mode: " + value, node);
}

ServoIoModel parseServoIoModel(const YAML::Node& node, const std::string& path) {
    const std::string value = lower(asString(node, path));
    if (value == "direct") return ServoIoModel::Direct;
    if (value == "worker") return ServoIoModel::Worker;
    fail("Unknown servo.io_model: " + value, node);
}

LinearMoveOrientationMode parseLinearMoveOrientationMode(const YAML::Node& node, const std::string& path) {
    const std::string value = lower(asString(node, path));
    if (value == "constant") return LinearMoveOrientationMode::Constant;
    if (value == "slerp") return LinearMoveOrientationMode::Slerp;
    fail("Unknown cartesian_control.linear_move.default_orientation_mode: " + value, node);
}

CartesianLimitPolicy parseCartesianLimitPolicy(const YAML::Node& node, const std::string& path) {
    const std::string value = lower(asString(node, path));
    if (value == "clamp") return CartesianLimitPolicy::Clamp;
    if (value == "reject") return CartesianLimitPolicy::Reject;
    fail("Unknown cartesian_control.exceed_limit_policy: " + value, node);
}

CartesianVelocityTargetIntegrationMode parseCartesianVelocityTargetIntegrationMode(
    const YAML::Node& node,
    const std::string& path
) {
    const std::string value = lower(asString(node, path));
    if (value == "measured_actual") return CartesianVelocityTargetIntegrationMode::MeasuredActual;
    if (value == "measured_actual_lookahead") return CartesianVelocityTargetIntegrationMode::MeasuredActualLookahead;
    if (value == "previous_command") return CartesianVelocityTargetIntegrationMode::PreviousCommand;
    fail("Unknown cartesian_control.velocity_target_integration: " + value, node);
}

CartesianCommandActualErrorPolicy parseCartesianCommandActualErrorPolicy(
    const YAML::Node& node,
    const std::string& path
) {
    const std::string value = lower(asString(node, path));
    if (value == "reset") return CartesianCommandActualErrorPolicy::Reset;
    if (value == "fault") return CartesianCommandActualErrorPolicy::Fault;
    fail("Unknown cartesian_control.command_actual_error_policy: " + value, node);
}

std::string getString(const YAML::Node& sec, const std::string& key, const std::string& fallback, const std::string& path) {
    return has(sec, key) ? asString(sec[key], path + "." + key) : fallback;
}

void applySimulatorTimeoutAlias(
    const YAML::Node& sec,
    const std::string& canonical,
    const std::string& deprecated,
    const std::string& path,
    double* target
) {
    const bool has_canonical = has(sec, canonical);
    const bool has_deprecated = has(sec, deprecated);
    if (has_canonical && has_deprecated) {
        fail(path + " cannot set both " + canonical + " and deprecated " + deprecated, sec[deprecated]);
    }
    if (has_canonical) {
        *target = asDouble(sec[canonical], path + "." + canonical);
    } else if (has_deprecated) {
        warnDeprecatedKey(path + "." + deprecated, path + "." + canonical);
        *target = asDouble(sec[deprecated], path + "." + deprecated);
    }
}

void applyDeprecatedDoubleAlias(
    const YAML::Node& sec,
    const std::string& canonical,
    const std::string& deprecated,
    const std::string& path,
    double* target
) {
    const bool has_canonical = has(sec, canonical);
    const bool has_deprecated = has(sec, deprecated);
    if (has_canonical && has_deprecated) {
        fail(path + " cannot set both " + canonical + " and deprecated " + deprecated, sec[deprecated]);
    }
    if (has_canonical) {
        *target = asDouble(sec[canonical], path + "." + canonical);
    } else if (has_deprecated) {
        warnDeprecatedKey(path + "." + deprecated, path + "." + canonical);
        *target = asDouble(sec[deprecated], path + "." + deprecated);
    }
}

void applyBackendSection(const YAML::Node& sec, BackendConfig* cfg, const std::string& path) {
    validateAllowedKeys(sec, {
        "backend_type",
        "run_mode",
        "name",
        "ip",
        "operation_mode",
        "simulator_control_endpoint",
        "simulator_request_timeout_sec",
        "simulator_connect_timeout_sec",
        "simulator_read_timeout_sec",
        "simulator_send_timeout_sec",
        "simulator_stop_timeout_sec",
        "simulator_reset_timeout_sec",
        "rbsim_control_endpoint",
        "rbsim_request_timeout_sec",
        "rbsim_connect_timeout_sec",
        "rbsim_read_timeout_sec",
        "rbsim_send_timeout_sec",
        "rbsim_stop_timeout_sec",
        "rbsim_reset_timeout_sec",
        "command_port",
        "data_port",
        "command_timeout_sec",
        "read_timeout_sec",
        "connect_timeout_sec",
        "script_t1_sec",
        "script_t2_sec",
        "script_gain",
        "script_alpha",
        "initial_q_deg",
        "speed_bar",
        "servo_t1_sec",
        "servo_t2_sec",
        "servo_time_sec",
        "servo_lookahead_sec",
        "servo_gain",
        "servo_alpha",
        "servo_acc",
        "disable_waiting_ack",
    }, path);

    if (has(sec, "backend_type")) cfg->backend_type = parseBackendType(sec["backend_type"], path + ".backend_type");
    if (has(sec, "run_mode")) cfg->run_mode = parseRunMode(sec["run_mode"], path + ".run_mode");
    cfg->name = getString(sec, "name", cfg->name, path);
    cfg->ip = getString(sec, "ip", cfg->ip, path);
    cfg->operation_mode = getString(sec, "operation_mode", cfg->operation_mode, path);

    if (has(sec, "simulator_control_endpoint") && has(sec, "rbsim_control_endpoint")) {
        fail(path + " cannot set both simulator_control_endpoint and deprecated rbsim_control_endpoint", sec["rbsim_control_endpoint"]);
    }
    if (has(sec, "simulator_control_endpoint")) {
        cfg->simulator_control_endpoint = asString(sec["simulator_control_endpoint"], path + ".simulator_control_endpoint");
    } else if (has(sec, "rbsim_control_endpoint")) {
        warnDeprecatedKey(path + ".rbsim_control_endpoint", path + ".simulator_control_endpoint");
        cfg->simulator_control_endpoint = asString(sec["rbsim_control_endpoint"], path + ".rbsim_control_endpoint");
    }
    cfg->rbsim_control_endpoint = cfg->simulator_control_endpoint;

    double request_timeout = cfg->rbsim_request_timeout_sec;
    applySimulatorTimeoutAlias(
        sec,
        "simulator_request_timeout_sec",
        "rbsim_request_timeout_sec",
        path,
        &request_timeout
    );
    if (request_timeout != cfg->rbsim_request_timeout_sec) {
        cfg->rbsim_request_timeout_sec = request_timeout;
        cfg->rbsim_connect_timeout_sec = request_timeout;
        cfg->rbsim_read_timeout_sec = request_timeout;
        cfg->rbsim_send_timeout_sec = request_timeout;
        cfg->rbsim_stop_timeout_sec = request_timeout;
        cfg->rbsim_reset_timeout_sec = request_timeout;
    }
    applySimulatorTimeoutAlias(sec, "simulator_connect_timeout_sec", "rbsim_connect_timeout_sec", path, &cfg->rbsim_connect_timeout_sec);
    applySimulatorTimeoutAlias(sec, "simulator_read_timeout_sec", "rbsim_read_timeout_sec", path, &cfg->rbsim_read_timeout_sec);
    applySimulatorTimeoutAlias(sec, "simulator_send_timeout_sec", "rbsim_send_timeout_sec", path, &cfg->rbsim_send_timeout_sec);
    applySimulatorTimeoutAlias(sec, "simulator_stop_timeout_sec", "rbsim_stop_timeout_sec", path, &cfg->rbsim_stop_timeout_sec);
    applySimulatorTimeoutAlias(sec, "simulator_reset_timeout_sec", "rbsim_reset_timeout_sec", path, &cfg->rbsim_reset_timeout_sec);

    if (has(sec, "command_port")) cfg->command_port = asInt(sec["command_port"], path + ".command_port");
    if (has(sec, "data_port")) cfg->data_port = asInt(sec["data_port"], path + ".data_port");
    if (has(sec, "command_timeout_sec")) cfg->command_timeout_sec = asDouble(sec["command_timeout_sec"], path + ".command_timeout_sec");
    if (has(sec, "read_timeout_sec")) cfg->read_timeout_sec = asDouble(sec["read_timeout_sec"], path + ".read_timeout_sec");
    if (has(sec, "connect_timeout_sec")) cfg->connect_timeout_sec = asDouble(sec["connect_timeout_sec"], path + ".connect_timeout_sec");
    if (has(sec, "script_t1_sec")) cfg->script_t1_sec = asDouble(sec["script_t1_sec"], path + ".script_t1_sec");
    if (has(sec, "script_t2_sec")) cfg->script_t2_sec = asDouble(sec["script_t2_sec"], path + ".script_t2_sec");
    if (has(sec, "script_gain")) cfg->script_gain = asDouble(sec["script_gain"], path + ".script_gain");
    if (has(sec, "script_alpha")) cfg->script_alpha = asDouble(sec["script_alpha"], path + ".script_alpha");

    if (has(sec, "initial_q_deg")) cfg->initial_q_deg = parseJointArray(sec["initial_q_deg"], path + ".initial_q_deg");
    if (has(sec, "speed_bar")) cfg->speed_bar = asDouble(sec["speed_bar"], path + ".speed_bar");
    applyDeprecatedDoubleAlias(sec, "servo_t1_sec", "servo_time_sec", path, &cfg->servo_t1_sec);
    applyDeprecatedDoubleAlias(sec, "servo_t2_sec", "servo_lookahead_sec", path, &cfg->servo_t2_sec);
    if (has(sec, "servo_gain")) cfg->servo_gain = asDouble(sec["servo_gain"], path + ".servo_gain");
    applyDeprecatedDoubleAlias(sec, "servo_alpha", "servo_acc", path, &cfg->servo_alpha);
    cfg->servo_time_sec = cfg->servo_t1_sec;
    cfg->servo_lookahead_sec = cfg->servo_t2_sec;
    cfg->servo_acc = cfg->servo_alpha;
    if (has(sec, "disable_waiting_ack")) cfg->disable_waiting_ack = asBool(sec["disable_waiting_ack"], path + ".disable_waiting_ack");
}

bool anyReal(const DualArmConfig& cfg) {
    return cfg.left_robot.run_mode == RunMode::Real || cfg.right_robot.run_mode == RunMode::Real;
}

bool isRbpodoControllerSimulationBackend(const BackendConfig& backend) {
    if (backend.backend_type != BackendType::Rbpodo) return false;
    if (backend.run_mode != RunMode::Real) return false;
    const std::string operation_mode = lower(backend.operation_mode);
    return operation_mode == "simulation" || operation_mode == "sim";
}

bool bothBackendsAreRbpodoControllerSimulation(const DualArmConfig& cfg) {
    return isRbpodoControllerSimulationBackend(cfg.left_robot) &&
        isRbpodoControllerSimulationBackend(cfg.right_robot);
}

std::string resolvePathForConfig(const std::string& value, const std::string& config_path) {
    namespace fs = std::filesystem;
    fs::path raw(value);
    if (raw.is_absolute()) return raw.lexically_normal().string();

    const fs::path cwd_candidate = fs::absolute(raw);
    if (fs::exists(cwd_candidate)) return cwd_candidate.lexically_normal().string();

    const fs::path parent = fs::absolute(fs::path(config_path)).parent_path();
    const fs::path sibling_candidate = parent / raw;
    if (fs::exists(sibling_candidate)) return sibling_candidate.lexically_normal().string();

    const fs::path repo_candidate = parent.parent_path() / raw;
    if (fs::exists(repo_candidate)) return repo_candidate.lexically_normal().string();

    return repo_candidate.lexically_normal().string();
}

std::string bindHost(const std::string& bind) {
    const auto scheme = bind.find("://");
    if (scheme == std::string::npos) return {};
    const std::string rest = bind.substr(scheme + 3);
    if (rest.empty()) return {};
    if (rest.front() == '[') {
        const auto close = rest.find(']');
        if (close == std::string::npos) return {};
        return rest.substr(1, close - 1);
    }
    const auto colon = rest.rfind(':');
    return colon == std::string::npos ? rest : rest.substr(0, colon);
}

bool bindRequiresExposureOverride(const std::string& bind) {
    const std::string host = bindHost(bind);
    if (host.empty()) return true;
    return !(host == "127.0.0.1" || host == "localhost" || host == "::1");
}

bool isLoopbackHost(const std::string& host) {
    return host == "127.0.0.1" || host == "localhost" || host == "::1";
}

bool isValidIpv4AllowlistEntry(const std::string& entry) {
    const auto slash = entry.find('/');
    const std::string host = slash == std::string::npos ? entry : entry.substr(0, slash);

    in_addr addr{};
    if (::inet_pton(AF_INET, host.c_str(), &addr) != 1) return false;
    if (slash == std::string::npos) return true;

    const std::string prefix_text = entry.substr(slash + 1);
    if (prefix_text.empty()) return false;
    for (char c : prefix_text) {
        if (!std::isdigit(static_cast<unsigned char>(c))) return false;
    }
    const int prefix = std::stoi(prefix_text);
    return prefix >= 1 && prefix <= 32;
}

void validatePositiveFinite(double value, const std::string& name) {
    if (!(value > 0.0) || !std::isfinite(value)) {
        throw std::runtime_error(name + " must be positive and finite");
    }
}

void validateNonNegativeFinite(double value, const std::string& name) {
    if (!(value >= 0.0) || !std::isfinite(value)) {
        throw std::runtime_error(name + " must be non-negative and finite");
    }
}

void validatePositiveFiniteArray(const JointArray& values, const std::string& name) {
    for (std::size_t i = 0; i < values.size(); ++i) {
        validatePositiveFinite(values[i], name + "[" + std::to_string(i) + "]");
    }
}

void validateNonNegativeFiniteArray(const JointArray& values, const std::string& name) {
    for (std::size_t i = 0; i < values.size(); ++i) {
        validateNonNegativeFinite(values[i], name + "[" + std::to_string(i) + "]");
    }
}

double workerReadPeriodFromRate(double rate_hz, const std::string& name) {
    validatePositiveFinite(rate_hz, name);
    return 1.0 / rate_hz;
}

void validateConfig(const DualArmConfig& cfg) {
    validatePositiveFinite(static_cast<double>(cfg.servo.rate_hz), "servo.rate_hz");
    validatePositiveFinite(cfg.servo.command_timeout_sec, "servo.command_timeout_sec");
    validatePositiveFinite(cfg.safety.command_timeout_sec, "safety.command_timeout_sec");
    validatePositiveFinite(cfg.safety.max_tracking_error_deg, "safety.max_tracking_error_deg");
    validateNonNegativeFiniteArray(cfg.safety.joint_wrap_period_deg, "safety.joint_wrap_period_deg");
    validatePositiveFinite(cfg.servo.filter_dt_min_ratio, "servo.filter_dt_min_ratio");
    validatePositiveFinite(cfg.servo.filter_dt_max_ratio, "servo.filter_dt_max_ratio");
    validatePositiveFinite(cfg.servo.worker_read_period_sec, "servo.worker_read_period_sec");
    validateNonNegativeFinite(
        cfg.servo.servo_t1_rate_match_tolerance_ratio,
        "servo.servo_t1_rate_match_tolerance_ratio"
    );
    validatePositiveFinite(static_cast<double>(cfg.network.state_pub_rate_hz), "network.state_pub_rate_hz");
    validatePositiveFinite(cfg.command_source.lease_timeout_sec, "command_source.lease_timeout_sec");
    validatePositiveFinite(cfg.left_robot.rbsim_request_timeout_sec, "left_robot.simulator_request_timeout_sec");
    validatePositiveFinite(cfg.right_robot.rbsim_request_timeout_sec, "right_robot.simulator_request_timeout_sec");
    validatePositiveFinite(cfg.left_robot.rbsim_connect_timeout_sec, "left_robot.simulator_connect_timeout_sec");
    validatePositiveFinite(cfg.right_robot.rbsim_connect_timeout_sec, "right_robot.simulator_connect_timeout_sec");
    validatePositiveFinite(cfg.left_robot.rbsim_read_timeout_sec, "left_robot.simulator_read_timeout_sec");
    validatePositiveFinite(cfg.right_robot.rbsim_read_timeout_sec, "right_robot.simulator_read_timeout_sec");
    validatePositiveFinite(cfg.left_robot.rbsim_send_timeout_sec, "left_robot.simulator_send_timeout_sec");
    validatePositiveFinite(cfg.right_robot.rbsim_send_timeout_sec, "right_robot.simulator_send_timeout_sec");
    validatePositiveFinite(cfg.left_robot.rbsim_stop_timeout_sec, "left_robot.simulator_stop_timeout_sec");
    validatePositiveFinite(cfg.right_robot.rbsim_stop_timeout_sec, "right_robot.simulator_stop_timeout_sec");
    validatePositiveFinite(cfg.left_robot.rbsim_reset_timeout_sec, "left_robot.simulator_reset_timeout_sec");
    validatePositiveFinite(cfg.right_robot.rbsim_reset_timeout_sec, "right_robot.simulator_reset_timeout_sec");
    if (cfg.servo.filter_dt_max_ratio < cfg.servo.filter_dt_min_ratio) {
        throw std::runtime_error("servo.filter_dt_max_ratio must be >= filter_dt_min_ratio");
    }
    if (cfg.servo.realtime_priority < 1 || cfg.servo.realtime_priority > 99) {
        throw std::runtime_error("servo.realtime_priority must be in [1, 99]");
    }
    if (cfg.network.command_source_allowlist.empty()) {
        throw std::runtime_error("network.command_source_allowlist must not be empty");
    }
    for (const std::string& entry : cfg.network.command_source_allowlist) {
        if (!isValidIpv4AllowlistEntry(entry)) {
            throw std::runtime_error("Invalid network.command_source_allowlist entry: " + entry);
        }
    }
    if (cfg.network.state_pub_endpoint != cfg.network.state_pub_bind) {
        throw std::runtime_error("network.state_pub_endpoint and deprecated state_pub_bind must be synchronized");
    }
    if (cfg.network.state_pub_endpoints.empty()) {
        throw std::runtime_error("network.state_pub_endpoints must not be empty");
    }
    if (cfg.network.state_pub_endpoints.front() != cfg.network.state_pub_endpoint) {
        throw std::runtime_error("network.state_pub_endpoint must match first network.state_pub_endpoints entry");
    }

    const bool readonly_diagnostic_startup_allowed =
        cfg.servo.allow_readonly_faulted_startup ||
        cfg.servo.allow_readonly_q_range_violation_startup ||
        cfg.servo.allow_readonly_wrong_mode_startup;
    if (cfg.servo.send_servo_commands && readonly_diagnostic_startup_allowed) {
        throw std::runtime_error(
            "servo.allow_readonly_*_startup options require servo.send_servo_commands=false"
        );
    }
    if (cfg.servo.allow_controller_simulation_diagnostics_suspect &&
        !cfg.servo.allow_controller_simulation_motion) {
        throw std::runtime_error(
            "servo.allow_controller_simulation_diagnostics_suspect requires "
            "servo.allow_controller_simulation_motion=true"
        );
    }
    if (cfg.servo.allow_controller_simulation_motion ||
        cfg.servo.allow_controller_simulation_diagnostics_suspect) {
        if (!cfg.servo.send_servo_commands) {
            throw std::runtime_error(
                "servo.allow_controller_simulation_* options require servo.send_servo_commands=true"
            );
        }
        if (!bothBackendsAreRbpodoControllerSimulation(cfg)) {
            throw std::runtime_error(
                "servo.allow_controller_simulation_* options require both rbpodo backends "
                "to use run_mode=real and operation_mode=simulation"
            );
        }
        if (!envIsOne("RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION")) {
            throw std::runtime_error(
                "Refusing rbpodo controller-simulation motion without "
                "RB_ALLOW_RBPODO_CONTROLLER_SIM_MOTION=1."
            );
        }
        if (!envIsOne("RB_RBPODO_PGMODE_SIMULATION_CONFIRMED")) {
            throw std::runtime_error(
                "Refusing rbpodo controller-simulation motion before pgmode simulation "
                "is confirmed by the acceptance tool."
            );
        }
        if (cfg.servo.allow_controller_simulation_diagnostics_suspect &&
            !envIsOne("RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM")) {
            throw std::runtime_error(
                "Refusing rbpodo diagnostics-suspect controller-simulation override without "
                "RB_ALLOW_RBPODO_DIAGNOSTICS_SUSPECT_CONTROLLER_SIM=1."
            );
        }
    }
    if (cfg.safety.joint_wrap_for_motion_safety) {
        throw std::runtime_error(
            "safety.joint_wrap_for_motion_safety is not implemented for motion targets"
        );
    }
    if (cfg.servo.send_servo_commands && cfg.safety.joint_wrap_for_startup_validation) {
        throw std::runtime_error(
            "safety.joint_wrap_for_startup_validation requires servo.send_servo_commands=false "
            "until motion-safe joint unwrapping is implemented"
        );
    }

    const std::string force_provider = lower(cfg.force_control.provider);
    if (!(force_provider == "null" || force_provider == "none" || force_provider.empty())) {
        throw std::runtime_error("force_control.provider must be null");
    }
    if (cfg.force_control.enable) {
        throw std::runtime_error("force_control.enable must remain false");
    }

    if (anyReal(cfg) && cfg.cartesian_control.enable && cfg.cartesian_control.allow_in_real) {
        const char* allow_cartesian = std::getenv("RB_ALLOW_REAL_CARTESIAN");
        if (!allow_cartesian || std::string(allow_cartesian) != "1") {
            throw std::runtime_error("Refusing real Cartesian control. Set RB_ALLOW_REAL_CARTESIAN=1.");
        }
    }
    validateNonNegativeFinite(cfg.cartesian_control.warn_ik_duration_us, "cartesian_control.warn_ik_duration_us");
    validateNonNegativeFinite(cfg.cartesian_control.fail_ik_duration_us, "cartesian_control.fail_ik_duration_us");
    validatePositiveFinite(cfg.cartesian_control.path_kp, "cartesian_control.path_kp");
    validatePositiveFinite(cfg.cartesian_control.path_kp_pos, "cartesian_control.path_kp_pos");
    validatePositiveFinite(cfg.cartesian_control.path_kp_ori, "cartesian_control.path_kp_ori");
    validatePositiveFinite(cfg.cartesian_control.twist_orientation_hold_kp, "cartesian_control.twist_orientation_hold_kp");
    validatePositiveFinite(cfg.cartesian_control.twist_angular_deadband_rad_s, "cartesian_control.twist_angular_deadband_rad_s");
    validatePositiveFinite(cfg.cartesian_control.velocity_damping, "cartesian_control.velocity_damping");
    validatePositiveFinite(cfg.cartesian_control.max_twist_linear_m_s, "cartesian_control.max_twist_linear_m_s");
    validatePositiveFinite(cfg.cartesian_control.max_twist_angular_rad_s, "cartesian_control.max_twist_angular_rad_s");
    validatePositiveFinite(cfg.cartesian_control.max_linear_move_speed_m_s, "cartesian_control.max_linear_move_speed_m_s");
    validatePositiveFinite(cfg.cartesian_control.max_angular_move_speed_rad_s, "cartesian_control.max_angular_move_speed_rad_s");
    validatePositiveFinite(cfg.cartesian_control.velocity_target_lookahead_sec, "cartesian_control.velocity_target_lookahead_sec");
    validatePositiveFiniteArray(
        cfg.cartesian_control.max_command_actual_error_deg,
        "cartesian_control.max_command_actual_error_deg"
    );
    if (cfg.cartesian_control.max_cartesian_step_m.has_value()) {
        validatePositiveFinite(*cfg.cartesian_control.max_cartesian_step_m, "cartesian_control.max_cartesian_step_m");
    }
    if (cfg.cartesian_control.max_cartesian_step_rad.has_value()) {
        validatePositiveFinite(*cfg.cartesian_control.max_cartesian_step_rad, "cartesian_control.max_cartesian_step_rad");
    }
    validatePositiveFinite(cfg.cartesian_control.linear_move.min_duration_sec, "cartesian_control.linear_move.min_duration_sec");
    validatePositiveFinite(cfg.cartesian_control.linear_move.max_duration_sec, "cartesian_control.linear_move.max_duration_sec");
    validatePositiveFinite(cfg.cartesian_control.linear_move.default_linear_speed_m_s, "cartesian_control.linear_move.default_linear_speed_m_s");
    validatePositiveFinite(cfg.cartesian_control.linear_move.default_angular_speed_rad_s, "cartesian_control.linear_move.default_angular_speed_rad_s");
    validatePositiveFinite(
        cfg.cartesian_control.linear_move.constant_orientation_tolerance_rad,
        "cartesian_control.linear_move.constant_orientation_tolerance_rad"
    );
    if (cfg.cartesian_control.linear_move.max_duration_sec < cfg.cartesian_control.linear_move.min_duration_sec) {
        throw std::runtime_error("cartesian_control.linear_move.max_duration_sec must be >= min_duration_sec");
    }
    if (cfg.cartesian_control.enable_benchmark_primitives && anyReal(cfg)) {
        throw std::runtime_error("Refusing cartesian_control.enable_benchmark_primitives in real mode");
    }
    if (cfg.cartesian_control.circle_move.allow_in_real) {
        throw std::runtime_error("cartesian_control.circle_move.allow_in_real must remain false");
    }
    validatePositiveFinite(cfg.cartesian_control.circle_move.max_diameter_m, "cartesian_control.circle_move.max_diameter_m");
    validatePositiveFinite(cfg.cartesian_control.circle_move.min_period_sec, "cartesian_control.circle_move.min_period_sec");

    if (cfg.kinematics.enable) {
        const std::string provider = lower(cfg.kinematics.provider);
        if (provider != "pinocchio") {
            throw std::runtime_error("kinematics.provider must be pinocchio when kinematics.enable=true");
        }
        if (lower(cfg.kinematics.q_units) != "deg") {
            throw std::runtime_error("kinematics.q_units must be deg");
        }
        if (cfg.kinematics.base_link.empty()) {
            throw std::runtime_error("kinematics.base_link must not be empty");
        }
        if (cfg.kinematics.tip_link.empty()) {
            throw std::runtime_error("kinematics.tip_link must not be empty");
        }
        if (cfg.kinematics.joint_names.size() != kDof) {
            throw std::runtime_error("kinematics.joint_names must contain exactly 6 names");
        }
        std::set<std::string> joint_names;
        for (const std::string& joint_name : cfg.kinematics.joint_names) {
            if (joint_name.empty()) {
                throw std::runtime_error("kinematics.joint_names must not contain empty names");
            }
            if (!joint_names.insert(joint_name).second) {
                throw std::runtime_error("kinematics.joint_names must be unique");
            }
        }
        if (cfg.kinematics.urdf.empty() || !std::filesystem::is_regular_file(cfg.kinematics.urdf)) {
            throw std::runtime_error("kinematics.urdf must point to an existing URDF file: " + cfg.kinematics.urdf);
        }
    }
    if (cfg.kinematics.ik.max_iterations <= 0) {
        throw std::runtime_error("kinematics.ik.max_iterations must be positive");
    }
    validatePositiveFinite(cfg.kinematics.ik.timeout_ms, "kinematics.ik.timeout_ms");
    validatePositiveFinite(cfg.kinematics.ik.damping, "kinematics.ik.damping");
    validatePositiveFinite(cfg.kinematics.ik.position_tolerance_m, "kinematics.ik.position_tolerance_m");
    validatePositiveFinite(cfg.kinematics.ik.orientation_tolerance_rad, "kinematics.ik.orientation_tolerance_rad");
    validatePositiveFiniteArray(cfg.kinematics.ik.max_step_deg, "kinematics.ik.max_step_deg");

    const auto validate_simulator_backend = [](const BackendConfig& backend, const std::string& label) {
        if (backend.backend_type != BackendType::Simulator) return;
        const std::string host = bindHost(backend.simulator_control_endpoint);
        if (host.empty()) {
            throw std::runtime_error(label + ".simulator_control_endpoint must use tcp://host:port");
        }
        if (host == "0.0.0.0" || host == "::" || host == "*") {
            throw std::runtime_error(label + ".simulator_control_endpoint must name a reachable simulator host, not a wildcard bind");
        }
        if (backend.run_mode == RunMode::Real) {
            throw std::runtime_error(label + " backend_type=simulator cannot use run_mode=real");
        }
    };
    validate_simulator_backend(cfg.left_robot, "left_robot");
    validate_simulator_backend(cfg.right_robot, "right_robot");

    const auto validate_rbpodo_backend = [&cfg](const BackendConfig& backend, const std::string& label) {
        if (backend.backend_type != BackendType::Rbpodo) return;
        if (backend.ip.empty()) {
            throw std::runtime_error(label + ".ip must be set for backend_type=rbpodo");
        }
        const std::string operation_mode = lower(backend.operation_mode);
        if (!(operation_mode == "real" || operation_mode == "simulation" || operation_mode == "sim")) {
            throw std::runtime_error(label + ".operation_mode must be real or simulation for backend_type=rbpodo");
        }
        if (!std::isfinite(backend.speed_bar) || backend.speed_bar < 0.0 || backend.speed_bar > 1.0) {
            throw std::runtime_error(label + ".speed_bar must be finite and in [0, 1] for backend_type=rbpodo");
        }
        validatePositiveFinite(backend.command_timeout_sec, label + ".command_timeout_sec");
        if (!(backend.servo_t1_sec >= 0.002) || !std::isfinite(backend.servo_t1_sec)) {
            throw std::runtime_error(label + ".servo_t1_sec must be finite and >= 0.002 for backend_type=rbpodo");
        }
        if (!(backend.servo_t2_sec > 0.02 && backend.servo_t2_sec < 0.2) || !std::isfinite(backend.servo_t2_sec)) {
            throw std::runtime_error(label + ".servo_t2_sec must be finite and in (0.02, 0.2) for backend_type=rbpodo");
        }
        validatePositiveFinite(backend.servo_gain, label + ".servo_gain");
        if (!(backend.servo_alpha > 0.0 && backend.servo_alpha < 1.0) || !std::isfinite(backend.servo_alpha)) {
            throw std::runtime_error(label + ".servo_alpha must be finite and in (0, 1) for backend_type=rbpodo");
        }
        const double servo_dt_sec = 1.0 / static_cast<double>(cfg.servo.rate_hz);
        const double tolerance_sec = cfg.servo.servo_t1_rate_match_tolerance_ratio * servo_dt_sec;
        const bool t1_matches_rate = std::abs(backend.servo_t1_sec - servo_dt_sec) <= tolerance_sec;
        if (!t1_matches_rate && backend.run_mode == RunMode::Real) {
            const std::string message =
                label + ".servo_t1_sec must match servo.rate_hz period for real rbpodo motion: servo_t1_sec=" +
                std::to_string(backend.servo_t1_sec) + ", period=" + std::to_string(servo_dt_sec);
            if (cfg.servo.send_servo_commands && !cfg.servo.allow_servo_t1_rate_mismatch) {
                throw std::runtime_error(message + ". Set servo.allow_servo_t1_rate_mismatch=true only after explicit acceptance.");
            }
            warn(message + "; send_servo_commands=false so this is a warning");
        }
        if (backend.run_mode == RunMode::Real &&
            backend.disable_waiting_ack &&
            cfg.servo.send_servo_commands &&
            !envIsOne("RB_ALLOW_RBPODO_ACK_DISABLED_MOTION")) {
            throw std::runtime_error(
                "Refusing rbpodo real motion with disable_waiting_ack=true. "
                "Set RB_ALLOW_RBPODO_ACK_DISABLED_MOTION=1 only after explicit ACK-off acceptance."
            );
        }
    };
    validate_rbpodo_backend(cfg.left_robot, "left_robot");
    validate_rbpodo_backend(cfg.right_robot, "right_robot");

    const auto validate_rbscript_tcp_backend = [&cfg](const BackendConfig& backend, const std::string& label) {
        if (backend.backend_type != BackendType::RbscriptTcp) return;
        if (backend.run_mode != RunMode::Real) {
            throw std::runtime_error(label + " backend_type=rbscript_tcp is experimental real-read-only only; mock/simulation configs are refused");
        }
        if (backend.ip.empty()) {
            throw std::runtime_error(label + ".ip must be set for backend_type=rbscript_tcp");
        }
        if (backend.command_port <= 0 || backend.command_port > 65535) {
            throw std::runtime_error(label + ".command_port must be in [1, 65535] for backend_type=rbscript_tcp");
        }
        if (backend.data_port <= 0 || backend.data_port > 65535) {
            throw std::runtime_error(label + ".data_port must be in [1, 65535] for backend_type=rbscript_tcp");
        }
        validatePositiveFinite(backend.command_timeout_sec, label + ".command_timeout_sec");
        validatePositiveFinite(backend.read_timeout_sec, label + ".read_timeout_sec");
        validatePositiveFinite(backend.connect_timeout_sec, label + ".connect_timeout_sec");
        if (!(backend.script_t1_sec >= 0.002) || !std::isfinite(backend.script_t1_sec)) {
            throw std::runtime_error(label + ".script_t1_sec must be finite and >= 0.002 for backend_type=rbscript_tcp");
        }
        if (!(backend.script_t2_sec > 0.02 && backend.script_t2_sec < 0.2) || !std::isfinite(backend.script_t2_sec)) {
            throw std::runtime_error(label + ".script_t2_sec must be finite and in (0.02, 0.2) for backend_type=rbscript_tcp");
        }
        validatePositiveFinite(backend.script_gain, label + ".script_gain");
        if (!(backend.script_alpha > 0.0 && backend.script_alpha < 1.0) || !std::isfinite(backend.script_alpha)) {
            throw std::runtime_error(label + ".script_alpha must be finite and in (0, 1) for backend_type=rbscript_tcp");
        }
        if (!envIsOne("RB_ALLOW_RBSCRIPT_TCP")) {
            throw std::runtime_error("Refusing rbscript_tcp real connection. Set RB_ALLOW_RBSCRIPT_TCP=1.");
        }
        if (backend.disable_waiting_ack && !envIsOne("RB_ALLOW_RBSCRIPT_TCP_MOTION")) {
            throw std::runtime_error("Refusing rbscript_tcp disable_waiting_ack without RB_ALLOW_RBSCRIPT_TCP_MOTION=1.");
        }
        if (cfg.servo.send_servo_commands && !envIsOne("RB_ALLOW_RBSCRIPT_TCP_MOTION")) {
            throw std::runtime_error("Refusing rbscript_tcp servo motion. Set RB_ALLOW_RBSCRIPT_TCP_MOTION=1 and RB_ALLOW_REAL_MOTION=1 or servo.send_servo_commands=false.");
        }
    };
    validate_rbscript_tcp_backend(cfg.left_robot, "left_robot");
    validate_rbscript_tcp_backend(cfg.right_robot, "right_robot");

    if (anyReal(cfg)) {
        if (cfg.servo.io_model == ServoIoModel::Worker) {
            throw std::runtime_error("Refusing servo.io_model=worker in real mode until worker I/O has real-hardware acceptance.");
        }
        if (!envIsOne("RB_ALLOW_REAL_ROBOT")) {
            throw std::runtime_error("Refusing real mode. Set RB_ALLOW_REAL_ROBOT=1.");
        }
        if (cfg.servo.send_servo_commands) {
            if (!envIsOne("RB_ALLOW_REAL_MOTION")) {
                throw std::runtime_error("Refusing real robot motion. Set RB_ALLOW_REAL_MOTION=1 or servo.send_servo_commands=false.");
            }
        }
        if (!cfg.servo.enable_realtime_priority) {
            throw std::runtime_error("Refusing real mode without servo.enable_realtime_priority=true.");
        }
        if (cfg.safety.tracking_error_policy != TrackingErrorPolicy::FaultLatch) {
            throw std::runtime_error("Refusing real mode without safety.tracking_error_policy=fault_latch.");
        }
        if (!cfg.safety.stop_both_arms_on_single_arm_error) {
            throw std::runtime_error("Refusing real mode without stop_both_arms_on_single_arm_error=true.");
        }
        if (!cfg.safety.latch_fault_on_robot_state_error) {
            throw std::runtime_error("Refusing real mode without latch_fault_on_robot_state_error=true.");
        }
        bool exposed_network = bindRequiresExposureOverride(cfg.network.command_bind);
        for (const std::string& endpoint : cfg.network.state_pub_endpoints) {
            exposed_network = exposed_network || bindRequiresExposureOverride(endpoint);
        }
        if (exposed_network) {
            if (!envIsOne("RB_ALLOW_NETWORK_EXPOSURE")) {
                throw std::runtime_error("Refusing exposed network bind in real mode. Set RB_ALLOW_NETWORK_EXPOSURE=1.");
            }
        }
    }
}

void applySchema(const YAML::Node& root) {
    if (!has(root, "schema")) {
        warn("config schema is missing; loading in compatibility mode. Add schema: " + std::string(kConfigSchema));
        return;
    }
    const std::string schema = asString(root["schema"], "schema");
    if (schema != kConfigSchema) {
        fail("Unknown config schema: " + schema, root["schema"]);
    }
}

YAML::Node loadYamlFile(const std::string& path) {
    try {
        YAML::Node root = YAML::LoadFile(path);
        if (!root || root.IsNull()) {
            return YAML::Node(YAML::NodeType::Map);
        }
        return root;
    } catch (const YAML::BadFile&) {
        throw std::runtime_error("Failed to open config file: " + path);
    } catch (const YAML::ParserException& exc) {
        throw std::runtime_error("Failed to parse config file " + path + ": " + exc.msg);
    }
}

void validateRootKeys(const YAML::Node& root) {
    validateAllowedKeys(root, {
        "schema",
        "left_robot",
        "right_robot",
        "left_mount",
        "right_mount",
        "servo",
        "safety",
        "network",
        "command_source",
        "logging",
        "force_control",
        "cartesian_control",
        "kinematics",
    }, "config");
}

}  // namespace

DualArmConfig loadConfigFromYaml(const std::string& path) {
    DualArmConfig cfg;

    // Safe defaults for this project. YAML values override these.
    cfg.left_robot.name = "left_mock";
    cfg.left_robot.backend_type = BackendType::Mock;
    cfg.left_robot.run_mode = RunMode::Mock;
    cfg.left_robot.initial_q_deg = {0.0, -30.0, 80.0, 0.0, 60.0, 0.0};

    cfg.right_robot.name = "right_mock";
    cfg.right_robot.backend_type = BackendType::Mock;
    cfg.right_robot.run_mode = RunMode::Mock;
    cfg.right_robot.initial_q_deg = {0.0, -30.0, 80.0, 0.0, 60.0, 0.0};

    cfg.left_mount.arm_id = ArmId::Left;
    // Rotation is canonical URDF/ROS RPY converted from MJCF euler xyz.
    cfg.left_mount.base_pose_in_stand = {0.1601, -0.1725, 0.5825, 2.186649, 0.523831, 2.526296};
    cfg.right_mount.arm_id = ArmId::Right;
    cfg.right_mount.base_pose_in_stand = {-0.1601, -0.1725, 0.5825, 2.186649, -0.523831, -2.526296};

    cfg.servo.enable_realtime_priority = false;
    cfg.servo.filter_dt_min_ratio = 0.5;
    cfg.servo.filter_dt_max_ratio = 1.5;
    cfg.safety.q_min_deg = {-170, -120, -170, -190, -120, -360};
    cfg.safety.q_max_deg = {170, 120, 170, 190, 120, 360};
    cfg.safety.dq_max_deg_s = {60, 60, 60, 90, 90, 120};
    cfg.safety.ddq_max_deg_s2 = {300, 300, 300, 500, 500, 700};
    cfg.safety.tracking_error_policy = TrackingErrorPolicy::SnapToActual;

    const YAML::Node root = loadYamlFile(path);
    validateRootKeys(root);
    applySchema(root);

    if (has(root, "left_robot")) applyBackendSection(root["left_robot"], &cfg.left_robot, "left_robot");
    if (has(root, "right_robot")) applyBackendSection(root["right_robot"], &cfg.right_robot, "right_robot");

    if (has(root, "left_mount")) {
        const YAML::Node sec = root["left_mount"];
        validateAllowedKeys(sec, {"base_pose_in_stand"}, "left_mount");
        if (has(sec, "base_pose_in_stand")) {
            cfg.left_mount.base_pose_in_stand = parsePose6D(sec["base_pose_in_stand"], "left_mount.base_pose_in_stand");
        }
    }
    if (has(root, "right_mount")) {
        const YAML::Node sec = root["right_mount"];
        validateAllowedKeys(sec, {"base_pose_in_stand"}, "right_mount");
        if (has(sec, "base_pose_in_stand")) {
            cfg.right_mount.base_pose_in_stand = parsePose6D(sec["base_pose_in_stand"], "right_mount.base_pose_in_stand");
        }
    }

    if (has(root, "servo")) {
        const YAML::Node sec = root["servo"];
        validateAllowedKeys(sec, {
            "rate_hz",
            "command_timeout_sec",
            "io_model",
            "startup_mode",
            "send_servo_commands",
            "allow_readonly_faulted_startup",
            "allow_readonly_q_range_violation_startup",
            "allow_readonly_wrong_mode_startup",
            "allow_controller_simulation_motion",
            "allow_controller_simulation_diagnostics_suspect",
            "enable_realtime_priority",
            "realtime_priority",
            "cpu_core",
            "worker_read_period_sec",
            "worker_read_rate_hz",
            "filter_dt_min_ratio",
            "filter_dt_max_ratio",
            "servo_t1_rate_match_tolerance_ratio",
            "allow_servo_t1_rate_mismatch",
        }, "servo");
        if (has(sec, "rate_hz")) cfg.servo.rate_hz = asInt(sec["rate_hz"], "servo.rate_hz");
        if (has(sec, "command_timeout_sec")) cfg.servo.command_timeout_sec = asDouble(sec["command_timeout_sec"], "servo.command_timeout_sec");
        if (has(sec, "io_model")) cfg.servo.io_model = parseServoIoModel(sec["io_model"], "servo.io_model");
        if (has(sec, "startup_mode")) cfg.servo.startup_mode = controlModeFromString(asString(sec["startup_mode"], "servo.startup_mode"));
        if (has(sec, "send_servo_commands")) cfg.servo.send_servo_commands = asBool(sec["send_servo_commands"], "servo.send_servo_commands");
        if (has(sec, "allow_readonly_faulted_startup")) {
            cfg.servo.allow_readonly_faulted_startup =
                asBool(sec["allow_readonly_faulted_startup"], "servo.allow_readonly_faulted_startup");
        }
        if (has(sec, "allow_readonly_q_range_violation_startup")) {
            cfg.servo.allow_readonly_q_range_violation_startup =
                asBool(
                    sec["allow_readonly_q_range_violation_startup"],
                    "servo.allow_readonly_q_range_violation_startup"
                );
        }
        if (has(sec, "allow_readonly_wrong_mode_startup")) {
            cfg.servo.allow_readonly_wrong_mode_startup =
                asBool(sec["allow_readonly_wrong_mode_startup"], "servo.allow_readonly_wrong_mode_startup");
        }
        if (has(sec, "allow_controller_simulation_motion")) {
            cfg.servo.allow_controller_simulation_motion =
                asBool(sec["allow_controller_simulation_motion"], "servo.allow_controller_simulation_motion");
        }
        if (has(sec, "allow_controller_simulation_diagnostics_suspect")) {
            cfg.servo.allow_controller_simulation_diagnostics_suspect =
                asBool(
                    sec["allow_controller_simulation_diagnostics_suspect"],
                    "servo.allow_controller_simulation_diagnostics_suspect"
                );
        }
        if (has(sec, "enable_realtime_priority")) cfg.servo.enable_realtime_priority = asBool(sec["enable_realtime_priority"], "servo.enable_realtime_priority");
        if (has(sec, "realtime_priority")) cfg.servo.realtime_priority = asInt(sec["realtime_priority"], "servo.realtime_priority");
        if (has(sec, "cpu_core")) cfg.servo.cpu_core = asInt(sec["cpu_core"], "servo.cpu_core");
        if (has(sec, "worker_read_period_sec") && has(sec, "worker_read_rate_hz")) {
            fail("servo cannot set both worker_read_period_sec and worker_read_rate_hz", sec["worker_read_rate_hz"]);
        }
        if (has(sec, "worker_read_period_sec")) {
            cfg.servo.worker_read_period_sec = asDouble(sec["worker_read_period_sec"], "servo.worker_read_period_sec");
        } else if (has(sec, "worker_read_rate_hz")) {
            cfg.servo.worker_read_period_sec =
                workerReadPeriodFromRate(asDouble(sec["worker_read_rate_hz"], "servo.worker_read_rate_hz"), "servo.worker_read_rate_hz");
        }
        if (has(sec, "filter_dt_min_ratio")) cfg.servo.filter_dt_min_ratio = asDouble(sec["filter_dt_min_ratio"], "servo.filter_dt_min_ratio");
        if (has(sec, "filter_dt_max_ratio")) cfg.servo.filter_dt_max_ratio = asDouble(sec["filter_dt_max_ratio"], "servo.filter_dt_max_ratio");
        if (has(sec, "servo_t1_rate_match_tolerance_ratio")) {
            cfg.servo.servo_t1_rate_match_tolerance_ratio =
                asDouble(sec["servo_t1_rate_match_tolerance_ratio"], "servo.servo_t1_rate_match_tolerance_ratio");
        }
        if (has(sec, "allow_servo_t1_rate_mismatch")) {
            cfg.servo.allow_servo_t1_rate_mismatch =
                asBool(sec["allow_servo_t1_rate_mismatch"], "servo.allow_servo_t1_rate_mismatch");
        }
    }

    if (has(root, "safety")) {
        const YAML::Node sec = root["safety"];
        validateAllowedKeys(sec, {
            "q_min_deg",
            "q_max_deg",
            "dq_max_deg_s",
            "ddq_max_deg_s2",
            "joint_wrap_period_deg",
            "command_timeout_sec",
            "max_tracking_error_deg",
            "stop_both_arms_on_single_arm_error",
            "tracking_error_policy",
            "latch_fault_on_robot_state_error",
            "joint_wrap_for_startup_validation",
            "joint_wrap_for_motion_safety",
        }, "safety");
        if (has(sec, "q_min_deg")) cfg.safety.q_min_deg = parseJointArray(sec["q_min_deg"], "safety.q_min_deg");
        if (has(sec, "q_max_deg")) cfg.safety.q_max_deg = parseJointArray(sec["q_max_deg"], "safety.q_max_deg");
        if (has(sec, "dq_max_deg_s")) cfg.safety.dq_max_deg_s = parseJointArray(sec["dq_max_deg_s"], "safety.dq_max_deg_s");
        if (has(sec, "ddq_max_deg_s2")) cfg.safety.ddq_max_deg_s2 = parseJointArray(sec["ddq_max_deg_s2"], "safety.ddq_max_deg_s2");
        if (has(sec, "joint_wrap_period_deg")) cfg.safety.joint_wrap_period_deg = parseJointArray(sec["joint_wrap_period_deg"], "safety.joint_wrap_period_deg");
        if (has(sec, "command_timeout_sec")) cfg.safety.command_timeout_sec = asDouble(sec["command_timeout_sec"], "safety.command_timeout_sec");
        if (has(sec, "max_tracking_error_deg")) cfg.safety.max_tracking_error_deg = asDouble(sec["max_tracking_error_deg"], "safety.max_tracking_error_deg");
        if (has(sec, "stop_both_arms_on_single_arm_error")) cfg.safety.stop_both_arms_on_single_arm_error = asBool(sec["stop_both_arms_on_single_arm_error"], "safety.stop_both_arms_on_single_arm_error");
        if (has(sec, "tracking_error_policy")) cfg.safety.tracking_error_policy = trackingErrorPolicyFromString(asString(sec["tracking_error_policy"], "safety.tracking_error_policy"));
        if (has(sec, "latch_fault_on_robot_state_error")) cfg.safety.latch_fault_on_robot_state_error = asBool(sec["latch_fault_on_robot_state_error"], "safety.latch_fault_on_robot_state_error");
        if (has(sec, "joint_wrap_for_startup_validation")) cfg.safety.joint_wrap_for_startup_validation = asBool(sec["joint_wrap_for_startup_validation"], "safety.joint_wrap_for_startup_validation");
        if (has(sec, "joint_wrap_for_motion_safety")) cfg.safety.joint_wrap_for_motion_safety = asBool(sec["joint_wrap_for_motion_safety"], "safety.joint_wrap_for_motion_safety");
    }

    if (has(root, "network")) {
        const YAML::Node sec = root["network"];
        validateAllowedKeys(sec, {
            "command_bind",
            "state_pub_endpoint",
            "state_pub_endpoints",
            "state_pub_bind",
            "state_pub_rate_hz",
            "command_source_allowlist",
        }, "network");
        cfg.network.command_bind = getString(sec, "command_bind", cfg.network.command_bind, "network");
        if (has(sec, "state_pub_endpoints") && (has(sec, "state_pub_endpoint") || has(sec, "state_pub_bind"))) {
            fail("network.state_pub_endpoints cannot be combined with state_pub_endpoint or deprecated state_pub_bind", sec["state_pub_endpoints"]);
        }
        if (has(sec, "state_pub_endpoint") && has(sec, "state_pub_bind")) {
            fail("network cannot set both state_pub_endpoint and deprecated state_pub_bind", sec["state_pub_bind"]);
        }
        if (has(sec, "state_pub_endpoints")) {
            cfg.network.state_pub_endpoints = asStringArray(sec["state_pub_endpoints"], "network.state_pub_endpoints");
            if (cfg.network.state_pub_endpoints.empty()) {
                fail("network.state_pub_endpoints must not be empty", sec["state_pub_endpoints"]);
            }
            cfg.network.state_pub_endpoint = cfg.network.state_pub_endpoints.front();
        } else if (has(sec, "state_pub_endpoint")) {
            cfg.network.state_pub_endpoint = asString(sec["state_pub_endpoint"], "network.state_pub_endpoint");
            cfg.network.state_pub_endpoints = {cfg.network.state_pub_endpoint};
        } else if (has(sec, "state_pub_bind")) {
            warnDeprecatedKey("network.state_pub_bind", "network.state_pub_endpoint");
            cfg.network.state_pub_endpoint = asString(sec["state_pub_bind"], "network.state_pub_bind");
            cfg.network.state_pub_endpoints = {cfg.network.state_pub_endpoint};
        } else {
            cfg.network.state_pub_endpoints = {cfg.network.state_pub_endpoint};
        }
        cfg.network.state_pub_bind = cfg.network.state_pub_endpoint;
        if (has(sec, "state_pub_rate_hz")) cfg.network.state_pub_rate_hz = asInt(sec["state_pub_rate_hz"], "network.state_pub_rate_hz");
        if (has(sec, "command_source_allowlist")) {
            cfg.network.command_source_allowlist = asStringArray(sec["command_source_allowlist"], "network.command_source_allowlist");
        }
    } else {
        cfg.network.state_pub_bind = cfg.network.state_pub_endpoint;
        cfg.network.state_pub_endpoints = {cfg.network.state_pub_endpoint};
    }
    cfg.network.command_timeout_sec = cfg.servo.command_timeout_sec;

    if (has(root, "command_source")) {
        const YAML::Node sec = root["command_source"];
        validateAllowedKeys(sec, {
            "enforce_lease",
            "lease_timeout_sec",
        }, "command_source");
        if (has(sec, "enforce_lease")) {
            cfg.command_source.enforce_lease = asBool(sec["enforce_lease"], "command_source.enforce_lease");
        }
        if (has(sec, "lease_timeout_sec")) {
            cfg.command_source.lease_timeout_sec = asDouble(sec["lease_timeout_sec"], "command_source.lease_timeout_sec");
        }
    }
    cfg.network.command_source_enforce_lease = cfg.command_source.enforce_lease;
    cfg.network.command_source_lease_timeout_sec = cfg.command_source.lease_timeout_sec;

    if (has(root, "logging")) {
        const YAML::Node sec = root["logging"];
        validateAllowedKeys(sec, {
            "enable",
            "directory",
            "flush_period_ms",
            "queue_capacity",
        }, "logging");
        if (has(sec, "enable")) cfg.logging.enable = asBool(sec["enable"], "logging.enable");
        cfg.logging.directory = getString(sec, "directory", cfg.logging.directory, "logging");
        if (has(sec, "flush_period_ms")) cfg.logging.flush_period_ms = asInt(sec["flush_period_ms"], "logging.flush_period_ms");
        if (has(sec, "queue_capacity")) {
            const int capacity = asInt(sec["queue_capacity"], "logging.queue_capacity");
            if (capacity <= 0) {
                throw std::runtime_error("logging.queue_capacity must be positive");
            }
            cfg.logging.queue_capacity = static_cast<size_t>(capacity);
        }
    }

    if (has(root, "force_control")) {
        const YAML::Node sec = root["force_control"];
        validateAllowedKeys(sec, {"provider", "enable"}, "force_control");
        if (has(sec, "provider")) cfg.force_control.provider = lower(asString(sec["provider"], "force_control.provider"));
        if (has(sec, "enable")) cfg.force_control.enable = asBool(sec["enable"], "force_control.enable");
    }

    if (has(root, "cartesian_control")) {
        const YAML::Node sec = root["cartesian_control"];
        validateAllowedKeys(sec, {
            "enable",
            "allow_in_simulation",
            "allow_in_real",
            "enable_benchmark_primitives",
            "warn_ik_duration_us",
            "fail_ik_duration_us",
            "path_kp",
            "path_kp_pos",
            "path_kp_ori",
            "twist_orientation_hold_kp",
            "twist_angular_deadband_rad_s",
            "velocity_damping",
            "max_twist_linear_m_s",
            "max_twist_angular_rad_s",
            "max_linear_move_speed_m_s",
            "max_angular_move_speed_rad_s",
            "max_cartesian_step_m",
            "max_cartesian_step_rad",
            "exceed_limit_policy",
            "velocity_target_integration",
            "velocity_target_lookahead_sec",
            "max_command_actual_error_deg",
            "reset_velocity_integrator_on_mode_change",
            "command_actual_error_policy",
            "linear_move",
            "circle_move",
        }, "cartesian_control");
        if (has(sec, "enable")) cfg.cartesian_control.enable = asBool(sec["enable"], "cartesian_control.enable");
        if (has(sec, "allow_in_simulation")) {
            cfg.cartesian_control.allow_in_simulation =
                asBool(sec["allow_in_simulation"], "cartesian_control.allow_in_simulation");
        }
        if (has(sec, "allow_in_real")) {
            cfg.cartesian_control.allow_in_real =
                asBool(sec["allow_in_real"], "cartesian_control.allow_in_real");
        }
        if (has(sec, "enable_benchmark_primitives")) {
            cfg.cartesian_control.enable_benchmark_primitives =
                asBool(sec["enable_benchmark_primitives"], "cartesian_control.enable_benchmark_primitives");
        }
        if (has(sec, "warn_ik_duration_us")) {
            cfg.cartesian_control.warn_ik_duration_us =
                asDouble(sec["warn_ik_duration_us"], "cartesian_control.warn_ik_duration_us");
        }
        if (has(sec, "fail_ik_duration_us")) {
            cfg.cartesian_control.fail_ik_duration_us =
                asDouble(sec["fail_ik_duration_us"], "cartesian_control.fail_ik_duration_us");
        }
        const bool has_legacy_path_kp = has(sec, "path_kp");
        const bool has_path_kp_pos = has(sec, "path_kp_pos");
        const bool has_path_kp_ori = has(sec, "path_kp_ori");
        if (has_legacy_path_kp && (has_path_kp_pos || has_path_kp_ori)) {
            fail(
                "cartesian_control cannot set deprecated path_kp together with path_kp_pos/path_kp_ori",
                sec["path_kp"]
            );
        }
        if (has(sec, "path_kp")) {
            warnDeprecatedKey("cartesian_control.path_kp", "cartesian_control.path_kp_pos and cartesian_control.path_kp_ori");
            const double path_kp = asDouble(sec["path_kp"], "cartesian_control.path_kp");
            cfg.cartesian_control.path_kp = path_kp;
            cfg.cartesian_control.path_kp_pos = path_kp;
            cfg.cartesian_control.path_kp_ori = path_kp;
        } else {
            if (has(sec, "path_kp_pos")) {
                cfg.cartesian_control.path_kp_pos = asDouble(sec["path_kp_pos"], "cartesian_control.path_kp_pos");
            }
            if (has(sec, "path_kp_ori")) {
                cfg.cartesian_control.path_kp_ori = asDouble(sec["path_kp_ori"], "cartesian_control.path_kp_ori");
            }
        }
        if (has(sec, "twist_orientation_hold_kp")) {
            cfg.cartesian_control.twist_orientation_hold_kp =
                asDouble(sec["twist_orientation_hold_kp"], "cartesian_control.twist_orientation_hold_kp");
        }
        if (has(sec, "twist_angular_deadband_rad_s")) {
            cfg.cartesian_control.twist_angular_deadband_rad_s =
                asDouble(sec["twist_angular_deadband_rad_s"], "cartesian_control.twist_angular_deadband_rad_s");
        }
        if (has(sec, "velocity_damping")) {
            cfg.cartesian_control.velocity_damping = asDouble(sec["velocity_damping"], "cartesian_control.velocity_damping");
        }
        if (has(sec, "max_twist_linear_m_s")) {
            cfg.cartesian_control.max_twist_linear_m_s =
                asDouble(sec["max_twist_linear_m_s"], "cartesian_control.max_twist_linear_m_s");
        }
        if (has(sec, "max_twist_angular_rad_s")) {
            cfg.cartesian_control.max_twist_angular_rad_s =
                asDouble(sec["max_twist_angular_rad_s"], "cartesian_control.max_twist_angular_rad_s");
        }
        if (has(sec, "max_linear_move_speed_m_s")) {
            cfg.cartesian_control.max_linear_move_speed_m_s =
                asDouble(sec["max_linear_move_speed_m_s"], "cartesian_control.max_linear_move_speed_m_s");
        }
        if (has(sec, "max_angular_move_speed_rad_s")) {
            cfg.cartesian_control.max_angular_move_speed_rad_s =
                asDouble(sec["max_angular_move_speed_rad_s"], "cartesian_control.max_angular_move_speed_rad_s");
        }
        if (has(sec, "max_cartesian_step_m")) {
            cfg.cartesian_control.max_cartesian_step_m =
                asDouble(sec["max_cartesian_step_m"], "cartesian_control.max_cartesian_step_m");
        }
        if (has(sec, "max_cartesian_step_rad")) {
            cfg.cartesian_control.max_cartesian_step_rad =
                asDouble(sec["max_cartesian_step_rad"], "cartesian_control.max_cartesian_step_rad");
        }
        if (has(sec, "exceed_limit_policy")) {
            cfg.cartesian_control.exceed_limit_policy =
                parseCartesianLimitPolicy(sec["exceed_limit_policy"], "cartesian_control.exceed_limit_policy");
        }
        if (has(sec, "velocity_target_integration")) {
            cfg.cartesian_control.velocity_target_integration =
                parseCartesianVelocityTargetIntegrationMode(
                    sec["velocity_target_integration"],
                    "cartesian_control.velocity_target_integration"
                );
        }
        if (has(sec, "velocity_target_lookahead_sec")) {
            cfg.cartesian_control.velocity_target_lookahead_sec =
                asDouble(sec["velocity_target_lookahead_sec"], "cartesian_control.velocity_target_lookahead_sec");
        }
        if (has(sec, "max_command_actual_error_deg")) {
            cfg.cartesian_control.max_command_actual_error_deg =
                parseJointArray(sec["max_command_actual_error_deg"], "cartesian_control.max_command_actual_error_deg");
        }
        if (has(sec, "reset_velocity_integrator_on_mode_change")) {
            cfg.cartesian_control.reset_velocity_integrator_on_mode_change =
                asBool(
                    sec["reset_velocity_integrator_on_mode_change"],
                    "cartesian_control.reset_velocity_integrator_on_mode_change"
                );
        }
        if (has(sec, "command_actual_error_policy")) {
            cfg.cartesian_control.command_actual_error_policy =
                parseCartesianCommandActualErrorPolicy(
                    sec["command_actual_error_policy"],
                    "cartesian_control.command_actual_error_policy"
                );
        }
        if (has(sec, "linear_move")) {
            const YAML::Node linear = sec["linear_move"];
            validateAllowedKeys(linear, {
                "min_duration_sec",
                "max_duration_sec",
                "default_linear_speed_m_s",
                "default_angular_speed_rad_s",
                "constant_orientation_tolerance_rad",
                "default_orientation_mode",
            }, "cartesian_control.linear_move");
            if (has(linear, "min_duration_sec")) {
                cfg.cartesian_control.linear_move.min_duration_sec =
                    asDouble(linear["min_duration_sec"], "cartesian_control.linear_move.min_duration_sec");
            }
            if (has(linear, "max_duration_sec")) {
                cfg.cartesian_control.linear_move.max_duration_sec =
                    asDouble(linear["max_duration_sec"], "cartesian_control.linear_move.max_duration_sec");
            }
            if (has(linear, "default_linear_speed_m_s")) {
                cfg.cartesian_control.linear_move.default_linear_speed_m_s =
                    asDouble(linear["default_linear_speed_m_s"], "cartesian_control.linear_move.default_linear_speed_m_s");
            }
            if (has(linear, "default_angular_speed_rad_s")) {
                cfg.cartesian_control.linear_move.default_angular_speed_rad_s =
                    asDouble(linear["default_angular_speed_rad_s"], "cartesian_control.linear_move.default_angular_speed_rad_s");
            }
            if (has(linear, "constant_orientation_tolerance_rad")) {
                cfg.cartesian_control.linear_move.constant_orientation_tolerance_rad =
                    asDouble(
                        linear["constant_orientation_tolerance_rad"],
                        "cartesian_control.linear_move.constant_orientation_tolerance_rad"
                    );
            }
            if (has(linear, "default_orientation_mode")) {
                cfg.cartesian_control.linear_move.default_orientation_mode =
                    parseLinearMoveOrientationMode(linear["default_orientation_mode"], "cartesian_control.linear_move.default_orientation_mode");
            }
        }
        if (has(sec, "circle_move")) {
            const YAML::Node circle = sec["circle_move"];
            validateAllowedKeys(circle, {
                "allow_in_simulation",
                "allow_in_real",
                "max_diameter_m",
                "min_period_sec",
            }, "cartesian_control.circle_move");
            if (has(circle, "allow_in_simulation")) {
                cfg.cartesian_control.circle_move.allow_in_simulation =
                    asBool(circle["allow_in_simulation"], "cartesian_control.circle_move.allow_in_simulation");
            }
            if (has(circle, "allow_in_real")) {
                cfg.cartesian_control.circle_move.allow_in_real =
                    asBool(circle["allow_in_real"], "cartesian_control.circle_move.allow_in_real");
            }
            if (has(circle, "max_diameter_m")) {
                cfg.cartesian_control.circle_move.max_diameter_m =
                    asDouble(circle["max_diameter_m"], "cartesian_control.circle_move.max_diameter_m");
            }
            if (has(circle, "min_period_sec")) {
                cfg.cartesian_control.circle_move.min_period_sec =
                    asDouble(circle["min_period_sec"], "cartesian_control.circle_move.min_period_sec");
            }
        }
    }

    if (has(root, "kinematics")) {
        const YAML::Node sec = root["kinematics"];
        validateAllowedKeys(sec, {
            "enable",
            "provider",
            "urdf",
            "base_link",
            "tip_link",
            "joint_names",
            "q_units",
            "publish_tcp",
            "ik",
        }, "kinematics");
        if (has(sec, "enable")) cfg.kinematics.enable = asBool(sec["enable"], "kinematics.enable");
        if (has(sec, "provider")) cfg.kinematics.provider = lower(asString(sec["provider"], "kinematics.provider"));
        if (has(sec, "urdf")) {
            cfg.kinematics.urdf = resolvePathForConfig(asString(sec["urdf"], "kinematics.urdf"), path);
        } else {
            cfg.kinematics.urdf = resolvePathForConfig(cfg.kinematics.urdf, path);
        }
        if (has(sec, "base_link")) cfg.kinematics.base_link = asString(sec["base_link"], "kinematics.base_link");
        if (has(sec, "tip_link")) cfg.kinematics.tip_link = asString(sec["tip_link"], "kinematics.tip_link");
        if (has(sec, "joint_names")) cfg.kinematics.joint_names = asStringArray(sec["joint_names"], "kinematics.joint_names");
        if (has(sec, "q_units")) cfg.kinematics.q_units = lower(asString(sec["q_units"], "kinematics.q_units"));
        if (has(sec, "publish_tcp")) cfg.kinematics.publish_tcp = asBool(sec["publish_tcp"], "kinematics.publish_tcp");
        if (has(sec, "ik")) {
            const YAML::Node ik = sec["ik"];
            validateAllowedKeys(ik, {
                "enable",
                "max_iterations",
                "timeout_ms",
                "damping",
                "position_tolerance_m",
                "orientation_tolerance_rad",
                "max_step_deg",
            }, "kinematics.ik");
            if (has(ik, "enable")) cfg.kinematics.ik.enable = asBool(ik["enable"], "kinematics.ik.enable");
            if (has(ik, "max_iterations")) cfg.kinematics.ik.max_iterations = asInt(ik["max_iterations"], "kinematics.ik.max_iterations");
            if (has(ik, "timeout_ms")) cfg.kinematics.ik.timeout_ms = asDouble(ik["timeout_ms"], "kinematics.ik.timeout_ms");
            if (has(ik, "damping")) cfg.kinematics.ik.damping = asDouble(ik["damping"], "kinematics.ik.damping");
            if (has(ik, "position_tolerance_m")) cfg.kinematics.ik.position_tolerance_m = asDouble(ik["position_tolerance_m"], "kinematics.ik.position_tolerance_m");
            if (has(ik, "orientation_tolerance_rad")) cfg.kinematics.ik.orientation_tolerance_rad = asDouble(ik["orientation_tolerance_rad"], "kinematics.ik.orientation_tolerance_rad");
            if (has(ik, "max_step_deg")) cfg.kinematics.ik.max_step_deg = parseJointArray(ik["max_step_deg"], "kinematics.ik.max_step_deg");
        }
    }

    if ((cfg.left_robot.run_mode == RunMode::Real || cfg.right_robot.run_mode == RunMode::Real) &&
        (!has(root, "safety") || !has(root["safety"], "tracking_error_policy"))) {
        cfg.safety.tracking_error_policy = TrackingErrorPolicy::FaultLatch;
    }

    validateConfig(cfg);

    std::cerr << "[INFO] loaded config: " << path << "\n";
    return cfg;
}

}  // namespace rb_servo
