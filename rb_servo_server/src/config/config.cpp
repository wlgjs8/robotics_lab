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
        "initial_q_deg",
        "speed_bar",
        "servo_time_sec",
        "servo_lookahead_sec",
        "servo_gain",
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

    if (has(sec, "initial_q_deg")) cfg->initial_q_deg = parseJointArray(sec["initial_q_deg"], path + ".initial_q_deg");
    if (has(sec, "speed_bar")) cfg->speed_bar = asDouble(sec["speed_bar"], path + ".speed_bar");
    if (has(sec, "servo_time_sec")) cfg->servo_time_sec = asDouble(sec["servo_time_sec"], path + ".servo_time_sec");
    if (has(sec, "servo_lookahead_sec")) cfg->servo_lookahead_sec = asDouble(sec["servo_lookahead_sec"], path + ".servo_lookahead_sec");
    if (has(sec, "servo_gain")) cfg->servo_gain = asDouble(sec["servo_gain"], path + ".servo_gain");
    if (has(sec, "servo_acc")) cfg->servo_acc = asDouble(sec["servo_acc"], path + ".servo_acc");
    if (has(sec, "disable_waiting_ack")) cfg->disable_waiting_ack = asBool(sec["disable_waiting_ack"], path + ".disable_waiting_ack");
}

bool anyReal(const DualArmConfig& cfg) {
    return cfg.left_robot.run_mode == RunMode::Real || cfg.right_robot.run_mode == RunMode::Real;
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

void validateConfig(const DualArmConfig& cfg) {
    validatePositiveFinite(static_cast<double>(cfg.servo.rate_hz), "servo.rate_hz");
    validatePositiveFinite(cfg.servo.command_timeout_sec, "servo.command_timeout_sec");
    validatePositiveFinite(cfg.safety.command_timeout_sec, "safety.command_timeout_sec");
    validatePositiveFinite(cfg.safety.max_tracking_error_deg, "safety.max_tracking_error_deg");
    validatePositiveFinite(cfg.servo.filter_dt_min_ratio, "servo.filter_dt_min_ratio");
    validatePositiveFinite(cfg.servo.filter_dt_max_ratio, "servo.filter_dt_max_ratio");
    validatePositiveFinite(static_cast<double>(cfg.network.state_pub_rate_hz), "network.state_pub_rate_hz");
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

#ifdef RB_SERVO_ENABLE_RBPODO
    const auto validate_rbpodo_backend = [](const BackendConfig& backend, const std::string& label) {
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
        validatePositiveFinite(backend.servo_time_sec, label + ".servo_time_sec");
        validatePositiveFinite(backend.servo_lookahead_sec, label + ".servo_lookahead_sec");
        validatePositiveFinite(backend.servo_gain, label + ".servo_gain");
        validatePositiveFinite(backend.servo_acc, label + ".servo_acc");
    };
    validate_rbpodo_backend(cfg.left_robot, "left_robot");
    validate_rbpodo_backend(cfg.right_robot, "right_robot");
#endif

    if (anyReal(cfg)) {
        if (cfg.servo.io_model == ServoIoModel::Worker) {
            throw std::runtime_error("Refusing servo.io_model=worker in real mode until worker I/O has real-hardware acceptance.");
        }
        const char* allow = std::getenv("RB_ALLOW_REAL_ROBOT");
        if (!allow || std::string(allow) != "1") {
            throw std::runtime_error("Refusing real mode. Set RB_ALLOW_REAL_ROBOT=1.");
        }
        if (cfg.servo.send_servo_commands) {
            const char* allow_motion = std::getenv("RB_ALLOW_REAL_MOTION");
            if (!allow_motion || std::string(allow_motion) != "1") {
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
        if (bindRequiresExposureOverride(cfg.network.command_bind) ||
            bindRequiresExposureOverride(cfg.network.state_pub_endpoint)) {
            const char* allow_network = std::getenv("RB_ALLOW_NETWORK_EXPOSURE");
            if (!allow_network || std::string(allow_network) != "1") {
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
    cfg.left_mount.base_pose_in_stand = {0.1601, -0.1725, 0.5825, 0.785, 2.35619, 0.0};
    cfg.right_mount.arm_id = ArmId::Right;
    cfg.right_mount.base_pose_in_stand = {-0.1601, -0.1725, 0.5825, 0.785, -2.35619, 0.0};

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
            "enable_realtime_priority",
            "realtime_priority",
            "cpu_core",
            "filter_dt_min_ratio",
            "filter_dt_max_ratio",
        }, "servo");
        if (has(sec, "rate_hz")) cfg.servo.rate_hz = asInt(sec["rate_hz"], "servo.rate_hz");
        if (has(sec, "command_timeout_sec")) cfg.servo.command_timeout_sec = asDouble(sec["command_timeout_sec"], "servo.command_timeout_sec");
        if (has(sec, "io_model")) cfg.servo.io_model = parseServoIoModel(sec["io_model"], "servo.io_model");
        if (has(sec, "startup_mode")) cfg.servo.startup_mode = controlModeFromString(asString(sec["startup_mode"], "servo.startup_mode"));
        if (has(sec, "send_servo_commands")) cfg.servo.send_servo_commands = asBool(sec["send_servo_commands"], "servo.send_servo_commands");
        if (has(sec, "enable_realtime_priority")) cfg.servo.enable_realtime_priority = asBool(sec["enable_realtime_priority"], "servo.enable_realtime_priority");
        if (has(sec, "realtime_priority")) cfg.servo.realtime_priority = asInt(sec["realtime_priority"], "servo.realtime_priority");
        if (has(sec, "cpu_core")) cfg.servo.cpu_core = asInt(sec["cpu_core"], "servo.cpu_core");
        if (has(sec, "filter_dt_min_ratio")) cfg.servo.filter_dt_min_ratio = asDouble(sec["filter_dt_min_ratio"], "servo.filter_dt_min_ratio");
        if (has(sec, "filter_dt_max_ratio")) cfg.servo.filter_dt_max_ratio = asDouble(sec["filter_dt_max_ratio"], "servo.filter_dt_max_ratio");
    }

    if (has(root, "safety")) {
        const YAML::Node sec = root["safety"];
        validateAllowedKeys(sec, {
            "q_min_deg",
            "q_max_deg",
            "dq_max_deg_s",
            "ddq_max_deg_s2",
            "command_timeout_sec",
            "max_tracking_error_deg",
            "stop_both_arms_on_single_arm_error",
            "tracking_error_policy",
            "latch_fault_on_robot_state_error",
        }, "safety");
        if (has(sec, "q_min_deg")) cfg.safety.q_min_deg = parseJointArray(sec["q_min_deg"], "safety.q_min_deg");
        if (has(sec, "q_max_deg")) cfg.safety.q_max_deg = parseJointArray(sec["q_max_deg"], "safety.q_max_deg");
        if (has(sec, "dq_max_deg_s")) cfg.safety.dq_max_deg_s = parseJointArray(sec["dq_max_deg_s"], "safety.dq_max_deg_s");
        if (has(sec, "ddq_max_deg_s2")) cfg.safety.ddq_max_deg_s2 = parseJointArray(sec["ddq_max_deg_s2"], "safety.ddq_max_deg_s2");
        if (has(sec, "command_timeout_sec")) cfg.safety.command_timeout_sec = asDouble(sec["command_timeout_sec"], "safety.command_timeout_sec");
        if (has(sec, "max_tracking_error_deg")) cfg.safety.max_tracking_error_deg = asDouble(sec["max_tracking_error_deg"], "safety.max_tracking_error_deg");
        if (has(sec, "stop_both_arms_on_single_arm_error")) cfg.safety.stop_both_arms_on_single_arm_error = asBool(sec["stop_both_arms_on_single_arm_error"], "safety.stop_both_arms_on_single_arm_error");
        if (has(sec, "tracking_error_policy")) cfg.safety.tracking_error_policy = trackingErrorPolicyFromString(asString(sec["tracking_error_policy"], "safety.tracking_error_policy"));
        if (has(sec, "latch_fault_on_robot_state_error")) cfg.safety.latch_fault_on_robot_state_error = asBool(sec["latch_fault_on_robot_state_error"], "safety.latch_fault_on_robot_state_error");
    }

    if (has(root, "network")) {
        const YAML::Node sec = root["network"];
        validateAllowedKeys(sec, {
            "command_bind",
            "state_pub_endpoint",
            "state_pub_bind",
            "state_pub_rate_hz",
            "command_source_allowlist",
        }, "network");
        cfg.network.command_bind = getString(sec, "command_bind", cfg.network.command_bind, "network");
        if (has(sec, "state_pub_endpoint") && has(sec, "state_pub_bind")) {
            fail("network cannot set both state_pub_endpoint and deprecated state_pub_bind", sec["state_pub_bind"]);
        }
        if (has(sec, "state_pub_endpoint")) {
            cfg.network.state_pub_endpoint = asString(sec["state_pub_endpoint"], "network.state_pub_endpoint");
        } else if (has(sec, "state_pub_bind")) {
            warnDeprecatedKey("network.state_pub_bind", "network.state_pub_endpoint");
            cfg.network.state_pub_endpoint = asString(sec["state_pub_bind"], "network.state_pub_bind");
        }
        cfg.network.state_pub_bind = cfg.network.state_pub_endpoint;
        if (has(sec, "state_pub_rate_hz")) cfg.network.state_pub_rate_hz = asInt(sec["state_pub_rate_hz"], "network.state_pub_rate_hz");
        if (has(sec, "command_source_allowlist")) {
            cfg.network.command_source_allowlist = asStringArray(sec["command_source_allowlist"], "network.command_source_allowlist");
        }
    } else {
        cfg.network.state_pub_bind = cfg.network.state_pub_endpoint;
    }
    cfg.network.command_timeout_sec = cfg.servo.command_timeout_sec;

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
            "warn_ik_duration_us",
            "fail_ik_duration_us",
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
        if (has(sec, "warn_ik_duration_us")) {
            cfg.cartesian_control.warn_ik_duration_us =
                asDouble(sec["warn_ik_duration_us"], "cartesian_control.warn_ik_duration_us");
        }
        if (has(sec, "fail_ik_duration_us")) {
            cfg.cartesian_control.fail_ik_duration_us =
                asDouble(sec["fail_ik_duration_us"], "cartesian_control.fail_ik_duration_us");
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
