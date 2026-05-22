#include "rb_servo/config/config.hpp"

#include <arpa/inet.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace rb_servo {
namespace {

std::string trim(const std::string& s) {
    const auto begin = s.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) return "";
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(begin, end - begin + 1);
}

std::string stripQuotes(std::string s) {
    s = trim(s);
    if (s.size() >= 2 && ((s.front() == '"' && s.back() == '"') ||
                          (s.front() == '\'' && s.back() == '\''))) {
        return s.substr(1, s.size() - 2);
    }
    return s;
}

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return s;
}

bool parseBool(const std::string& s) {
    const std::string v = lower(stripQuotes(s));
    return v == "true" || v == "1" || v == "yes" || v == "on";
}

double parseDouble(const std::string& s) {
    return std::stod(stripQuotes(s));
}

int parseInt(const std::string& s) {
    return std::stoi(stripQuotes(s));
}

std::vector<double> parseDoubleArray(std::string s) {
    s = trim(s);
    if (!s.empty() && s.front() == '[') s.erase(s.begin());
    if (!s.empty() && s.back() == ']') s.pop_back();
    std::vector<double> values;
    std::stringstream ss(s);
    std::string token;
    while (std::getline(ss, token, ',')) {
        token = trim(token);
        if (!token.empty()) values.push_back(std::stod(token));
    }
    return values;
}

std::vector<std::string> parseStringArray(std::string s) {
    s = trim(s);
    if (!s.empty() && s.front() == '[') s.erase(s.begin());
    if (!s.empty() && s.back() == ']') s.pop_back();
    std::vector<std::string> values;
    std::stringstream ss(s);
    std::string token;
    while (std::getline(ss, token, ',')) {
        token = stripQuotes(trim(token));
        if (!token.empty()) values.push_back(token);
    }
    if (values.empty() && !trim(s).empty()) {
        values.push_back(stripQuotes(s));
    }
    return values;
}

JointArray parseJointArray(const std::string& s) {
    const auto values = parseDoubleArray(s);
    if (values.size() != kDof) {
        throw std::runtime_error("Expected 6 values in JointArray, got " + std::to_string(values.size()));
    }
    JointArray out{};
    for (int i = 0; i < kDof; ++i) out[i] = values[static_cast<size_t>(i)];
    return out;
}

Pose6D parsePose6D(const std::string& s) {
    const auto v = parseDoubleArray(s);
    if (v.size() != 6) {
        throw std::runtime_error("Expected 6 values in Pose6D, got " + std::to_string(v.size()));
    }
    return Pose6D{v[0], v[1], v[2], v[3], v[4], v[5]};
}

BackendType parseBackendType(const std::string& s) {
    const std::string v = lower(stripQuotes(s));
    if (v == "mock") return BackendType::Mock;
    if (v == "rbpodo") return BackendType::Rbpodo;
    if (v == "rbsim" || v == "rbsim_local") return BackendType::Rbsim;
    throw std::runtime_error("Unknown backend_type: " + s);
}

RunMode parseRunMode(const std::string& s) {
    const std::string v = lower(stripQuotes(s));
    if (v == "mock") return RunMode::Mock;
    if (v == "simulation" || v == "sim" || v == "rbsim" || v == "rbsim_local") return RunMode::Simulation;
    if (v == "real") return RunMode::Real;
    throw std::runtime_error("Unknown run_mode: " + s);
}

using Section = std::map<std::string, std::string>;
using Sections = std::map<std::string, Section>;

Sections parseSimpleYaml(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("Failed to open config file: " + path);
    }

    Sections sections;
    std::string current_section;
    std::string line;
    int line_no = 0;
    while (std::getline(in, line)) {
        ++line_no;
        const auto comment_pos = line.find('#');
        if (comment_pos != std::string::npos) line = line.substr(0, comment_pos);
        if (trim(line).empty()) continue;

        const size_t indent = line.find_first_not_of(' ');
        const std::string t = trim(line);
        const auto colon = t.find(':');
        if (colon == std::string::npos) {
            throw std::runtime_error("Invalid YAML line " + std::to_string(line_no) + ": " + line);
        }
        const std::string key = trim(t.substr(0, colon));
        const std::string value = trim(t.substr(colon + 1));

        if (indent == 0 && value.empty()) {
            current_section = key;
            sections[current_section];
        } else {
            if (current_section.empty()) {
                throw std::runtime_error("Key outside section at line " + std::to_string(line_no));
            }
            sections[current_section][key] = value;
        }
    }
    return sections;
}

bool has(const Section& sec, const std::string& key) {
    return sec.find(key) != sec.end();
}

std::string getString(const Section& sec, const std::string& key, const std::string& fallback) {
    auto it = sec.find(key);
    return it == sec.end() ? fallback : stripQuotes(it->second);
}

void applyBackendSection(const Section& sec, BackendConfig* cfg) {
    if (has(sec, "backend_type")) cfg->backend_type = parseBackendType(sec.at("backend_type"));
    if (has(sec, "run_mode")) cfg->run_mode = parseRunMode(sec.at("run_mode"));
    cfg->name = getString(sec, "name", cfg->name);
    cfg->ip = getString(sec, "ip", cfg->ip);
    cfg->operation_mode = getString(sec, "operation_mode", cfg->operation_mode);
    cfg->rbsim_control_endpoint = getString(sec, "rbsim_control_endpoint", cfg->rbsim_control_endpoint);
    if (has(sec, "rbsim_request_timeout_sec")) {
        const double request_timeout = parseDouble(sec.at("rbsim_request_timeout_sec"));
        cfg->rbsim_request_timeout_sec = request_timeout;
        cfg->rbsim_connect_timeout_sec = request_timeout;
        cfg->rbsim_read_timeout_sec = request_timeout;
        cfg->rbsim_send_timeout_sec = request_timeout;
        cfg->rbsim_stop_timeout_sec = request_timeout;
        cfg->rbsim_reset_timeout_sec = request_timeout;
    }
    if (has(sec, "rbsim_connect_timeout_sec")) cfg->rbsim_connect_timeout_sec = parseDouble(sec.at("rbsim_connect_timeout_sec"));
    if (has(sec, "rbsim_read_timeout_sec")) cfg->rbsim_read_timeout_sec = parseDouble(sec.at("rbsim_read_timeout_sec"));
    if (has(sec, "rbsim_send_timeout_sec")) cfg->rbsim_send_timeout_sec = parseDouble(sec.at("rbsim_send_timeout_sec"));
    if (has(sec, "rbsim_stop_timeout_sec")) cfg->rbsim_stop_timeout_sec = parseDouble(sec.at("rbsim_stop_timeout_sec"));
    if (has(sec, "rbsim_reset_timeout_sec")) cfg->rbsim_reset_timeout_sec = parseDouble(sec.at("rbsim_reset_timeout_sec"));
    if (has(sec, "initial_q_deg")) cfg->initial_q_deg = parseJointArray(sec.at("initial_q_deg"));
    if (has(sec, "speed_bar")) cfg->speed_bar = parseDouble(sec.at("speed_bar"));
    if (has(sec, "servo_time_sec")) cfg->servo_time_sec = parseDouble(sec.at("servo_time_sec"));
    if (has(sec, "servo_lookahead_sec")) cfg->servo_lookahead_sec = parseDouble(sec.at("servo_lookahead_sec"));
    if (has(sec, "servo_gain")) cfg->servo_gain = parseDouble(sec.at("servo_gain"));
    if (has(sec, "servo_acc")) cfg->servo_acc = parseDouble(sec.at("servo_acc"));
    if (has(sec, "disable_waiting_ack")) cfg->disable_waiting_ack = parseBool(sec.at("disable_waiting_ack"));
}

bool anyReal(const DualArmConfig& cfg) {
    return cfg.left_robot.run_mode == RunMode::Real || cfg.right_robot.run_mode == RunMode::Real;
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

void validateConfig(const DualArmConfig& cfg) {
    validatePositiveFinite(static_cast<double>(cfg.servo.rate_hz), "servo.rate_hz");
    validatePositiveFinite(cfg.servo.command_timeout_sec, "servo.command_timeout_sec");
    validatePositiveFinite(cfg.safety.command_timeout_sec, "safety.command_timeout_sec");
    validatePositiveFinite(cfg.safety.max_tracking_error_deg, "safety.max_tracking_error_deg");
    validatePositiveFinite(cfg.servo.filter_dt_min_ratio, "servo.filter_dt_min_ratio");
    validatePositiveFinite(cfg.servo.filter_dt_max_ratio, "servo.filter_dt_max_ratio");
    validatePositiveFinite(cfg.left_robot.rbsim_request_timeout_sec, "left_robot.rbsim_request_timeout_sec");
    validatePositiveFinite(cfg.right_robot.rbsim_request_timeout_sec, "right_robot.rbsim_request_timeout_sec");
    validatePositiveFinite(cfg.left_robot.rbsim_connect_timeout_sec, "left_robot.rbsim_connect_timeout_sec");
    validatePositiveFinite(cfg.right_robot.rbsim_connect_timeout_sec, "right_robot.rbsim_connect_timeout_sec");
    validatePositiveFinite(cfg.left_robot.rbsim_read_timeout_sec, "left_robot.rbsim_read_timeout_sec");
    validatePositiveFinite(cfg.right_robot.rbsim_read_timeout_sec, "right_robot.rbsim_read_timeout_sec");
    validatePositiveFinite(cfg.left_robot.rbsim_send_timeout_sec, "left_robot.rbsim_send_timeout_sec");
    validatePositiveFinite(cfg.right_robot.rbsim_send_timeout_sec, "right_robot.rbsim_send_timeout_sec");
    validatePositiveFinite(cfg.left_robot.rbsim_stop_timeout_sec, "left_robot.rbsim_stop_timeout_sec");
    validatePositiveFinite(cfg.right_robot.rbsim_stop_timeout_sec, "right_robot.rbsim_stop_timeout_sec");
    validatePositiveFinite(cfg.left_robot.rbsim_reset_timeout_sec, "left_robot.rbsim_reset_timeout_sec");
    validatePositiveFinite(cfg.right_robot.rbsim_reset_timeout_sec, "right_robot.rbsim_reset_timeout_sec");
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

    const auto validate_rbsim_backend = [](const BackendConfig& backend, const std::string& label) {
        if (backend.backend_type != BackendType::Rbsim) return;
        const std::string host = bindHost(backend.rbsim_control_endpoint);
        if (host.empty() || !isLoopbackHost(host)) {
            throw std::runtime_error(label + ".rbsim_control_endpoint must use loopback");
        }
        if (backend.run_mode == RunMode::Real) {
            throw std::runtime_error(label + " backend_type=rbsim cannot use run_mode=real");
        }
    };
    validate_rbsim_backend(cfg.left_robot, "left_robot");
    validate_rbsim_backend(cfg.right_robot, "right_robot");

    if (anyReal(cfg)) {
        const char* allow = std::getenv("RB_ALLOW_REAL_ROBOT");
        if (!allow || std::string(allow) != "1") {
            throw std::runtime_error("Refusing real mode. Set RB_ALLOW_REAL_ROBOT=1.");
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
            bindRequiresExposureOverride(cfg.network.state_pub_bind)) {
            const char* allow_network = std::getenv("RB_ALLOW_NETWORK_EXPOSURE");
            if (!allow_network || std::string(allow_network) != "1") {
                throw std::runtime_error("Refusing exposed network bind in real mode. Set RB_ALLOW_NETWORK_EXPOSURE=1.");
            }
        }
    }
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

    const Sections sections = parseSimpleYaml(path);

    if (sections.count("left_robot")) applyBackendSection(sections.at("left_robot"), &cfg.left_robot);
    if (sections.count("right_robot")) applyBackendSection(sections.at("right_robot"), &cfg.right_robot);

    if (sections.count("left_mount")) {
        const Section& sec = sections.at("left_mount");
        if (has(sec, "base_pose_in_stand")) cfg.left_mount.base_pose_in_stand = parsePose6D(sec.at("base_pose_in_stand"));
    }
    if (sections.count("right_mount")) {
        const Section& sec = sections.at("right_mount");
        if (has(sec, "base_pose_in_stand")) cfg.right_mount.base_pose_in_stand = parsePose6D(sec.at("base_pose_in_stand"));
    }

    if (sections.count("servo")) {
        const Section& sec = sections.at("servo");
        if (has(sec, "rate_hz")) cfg.servo.rate_hz = parseInt(sec.at("rate_hz"));
        if (has(sec, "command_timeout_sec")) cfg.servo.command_timeout_sec = parseDouble(sec.at("command_timeout_sec"));
        if (has(sec, "startup_mode")) cfg.servo.startup_mode = controlModeFromString(stripQuotes(sec.at("startup_mode")));
        if (has(sec, "enable_realtime_priority")) cfg.servo.enable_realtime_priority = parseBool(sec.at("enable_realtime_priority"));
        if (has(sec, "realtime_priority")) cfg.servo.realtime_priority = parseInt(sec.at("realtime_priority"));
        if (has(sec, "cpu_core")) cfg.servo.cpu_core = parseInt(sec.at("cpu_core"));
        if (has(sec, "filter_dt_min_ratio")) cfg.servo.filter_dt_min_ratio = parseDouble(sec.at("filter_dt_min_ratio"));
        if (has(sec, "filter_dt_max_ratio")) cfg.servo.filter_dt_max_ratio = parseDouble(sec.at("filter_dt_max_ratio"));
    }

    if (sections.count("safety")) {
        const Section& sec = sections.at("safety");
        if (has(sec, "q_min_deg")) cfg.safety.q_min_deg = parseJointArray(sec.at("q_min_deg"));
        if (has(sec, "q_max_deg")) cfg.safety.q_max_deg = parseJointArray(sec.at("q_max_deg"));
        if (has(sec, "dq_max_deg_s")) cfg.safety.dq_max_deg_s = parseJointArray(sec.at("dq_max_deg_s"));
        if (has(sec, "ddq_max_deg_s2")) cfg.safety.ddq_max_deg_s2 = parseJointArray(sec.at("ddq_max_deg_s2"));
        if (has(sec, "command_timeout_sec")) cfg.safety.command_timeout_sec = parseDouble(sec.at("command_timeout_sec"));
        if (has(sec, "max_tracking_error_deg")) cfg.safety.max_tracking_error_deg = parseDouble(sec.at("max_tracking_error_deg"));
        if (has(sec, "stop_both_arms_on_single_arm_error")) cfg.safety.stop_both_arms_on_single_arm_error = parseBool(sec.at("stop_both_arms_on_single_arm_error"));
        if (has(sec, "tracking_error_policy")) cfg.safety.tracking_error_policy = trackingErrorPolicyFromString(stripQuotes(sec.at("tracking_error_policy")));
        if (has(sec, "latch_fault_on_robot_state_error")) cfg.safety.latch_fault_on_robot_state_error = parseBool(sec.at("latch_fault_on_robot_state_error"));
    }

    if (sections.count("network")) {
        const Section& sec = sections.at("network");
        cfg.network.command_bind = getString(sec, "command_bind", cfg.network.command_bind);
        cfg.network.state_pub_bind = getString(sec, "state_pub_bind", cfg.network.state_pub_bind);
        if (has(sec, "command_source_allowlist")) {
            cfg.network.command_source_allowlist = parseStringArray(sec.at("command_source_allowlist"));
        }
    }
    cfg.network.command_timeout_sec = cfg.servo.command_timeout_sec;

    if (sections.count("logging")) {
        const Section& sec = sections.at("logging");
        if (has(sec, "enable")) cfg.logging.enable = parseBool(sec.at("enable"));
        cfg.logging.directory = getString(sec, "directory", cfg.logging.directory);
        if (has(sec, "flush_period_ms")) cfg.logging.flush_period_ms = parseInt(sec.at("flush_period_ms"));
        if (has(sec, "queue_capacity")) {
            const int capacity = parseInt(sec.at("queue_capacity"));
            if (capacity <= 0) {
                throw std::runtime_error("logging.queue_capacity must be positive");
            }
            cfg.logging.queue_capacity = static_cast<size_t>(capacity);
        }
    }

    if (sections.count("force_control")) {
        const Section& sec = sections.at("force_control");
        if (has(sec, "enable")) cfg.force_control.enable = parseBool(sec.at("enable"));
        if (has(sec, "update_rate_hz")) cfg.force_control.update_rate_hz = parseInt(sec.at("update_rate_hz"));
        if (has(sec, "admittance_gain_pos")) cfg.force_control.admittance_gain_pos = parseDouble(sec.at("admittance_gain_pos"));
        if (has(sec, "admittance_gain_rot")) cfg.force_control.admittance_gain_rot = parseDouble(sec.at("admittance_gain_rot"));
        if (has(sec, "force_lpf_alpha")) cfg.force_control.force_lpf_alpha = parseDouble(sec.at("force_lpf_alpha"));
        if (has(sec, "max_pos_offset_m")) cfg.force_control.max_pos_offset_m = parseDouble(sec.at("max_pos_offset_m"));
        if (has(sec, "max_rot_offset_rad")) cfg.force_control.max_rot_offset_rad = parseDouble(sec.at("max_rot_offset_rad"));
        if (has(sec, "max_pos_step_m")) cfg.force_control.max_pos_step_m = parseDouble(sec.at("max_pos_step_m"));
        if (has(sec, "max_rot_step_rad")) cfg.force_control.max_rot_step_rad = parseDouble(sec.at("max_rot_step_rad"));
    }

    if ((cfg.left_robot.run_mode == RunMode::Real || cfg.right_robot.run_mode == RunMode::Real) &&
        (!sections.count("safety") || !has(sections.at("safety"), "tracking_error_policy"))) {
        cfg.safety.tracking_error_policy = TrackingErrorPolicy::FaultLatch;
    }

    validateConfig(cfg);

    std::cerr << "[INFO] loaded config: " << path << "\n";
    return cfg;
}

}  // namespace rb_servo
