#include "rb_servo/config/config.hpp"

#include <arpa/inet.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "rb_servo/control/user_floor_constraint.hpp"  // validateUserFloorPlaneRequest

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

bool isValidUdpEndpointUri(const std::string& endpoint) {
    constexpr const char* prefix = "udp://";
    if (endpoint.rfind(prefix, 0) != 0) return false;

    const std::string rest = endpoint.substr(std::strlen(prefix));
    if (rest.empty()) return false;

    const auto colon = rest.rfind(':');
    if (colon == std::string::npos || colon + 1 >= rest.size()) return false;

    const std::string host = rest.substr(0, colon);
    if (host.empty() || host == "0.0.0.0") return false;

    try {
        size_t consumed = 0;
        const int port = std::stoi(rest.substr(colon + 1), &consumed);
        if (consumed != rest.size() - colon - 1) return false;
        return port > 0 && port <= 65535;
    } catch (const std::exception&) {
        return false;
    }
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

void parsePoseTrackSmdConfig(const YAML::Node& smd, const std::string& path, PoseTrackSmdConfig* out) {
    if (!out) return;
    validateAllowedKeys(smd, {
        "enable",
        "damping_ratio_linear",
        "natural_frequency_linear_hz",
        "damping_ratio_angular",
        "natural_frequency_angular_hz",
        "max_linear_velocity_m_s",
        "max_linear_accel_m_s2",
        "max_angular_velocity_rad_s",
        "max_angular_accel_rad_s2",
        "velocity_feedforward",
        "reengage_relatch_max_step_m",
        "reengage_relatch_max_step_rad",
        "singularity_scale_full_sigma",
        "singularity_scale_floor_sigma",
        "singularity_scale_min",
    }, path);
    if (has(smd, "enable")) {
        out->enable = asBool(smd["enable"], path + ".enable");
    }
    if (has(smd, "damping_ratio_linear")) {
        out->damping_ratio_linear = asDouble(smd["damping_ratio_linear"], path + ".damping_ratio_linear");
    }
    if (has(smd, "natural_frequency_linear_hz")) {
        out->natural_frequency_linear_hz =
            asDouble(smd["natural_frequency_linear_hz"], path + ".natural_frequency_linear_hz");
    }
    if (has(smd, "damping_ratio_angular")) {
        out->damping_ratio_angular = asDouble(smd["damping_ratio_angular"], path + ".damping_ratio_angular");
    }
    if (has(smd, "natural_frequency_angular_hz")) {
        out->natural_frequency_angular_hz =
            asDouble(smd["natural_frequency_angular_hz"], path + ".natural_frequency_angular_hz");
    }
    if (has(smd, "max_linear_velocity_m_s")) {
        out->max_linear_velocity_m_s =
            asDouble(smd["max_linear_velocity_m_s"], path + ".max_linear_velocity_m_s");
    }
    if (has(smd, "max_linear_accel_m_s2")) {
        out->max_linear_accel_m_s2 = asDouble(smd["max_linear_accel_m_s2"], path + ".max_linear_accel_m_s2");
    }
    if (has(smd, "max_angular_velocity_rad_s")) {
        out->max_angular_velocity_rad_s =
            asDouble(smd["max_angular_velocity_rad_s"], path + ".max_angular_velocity_rad_s");
    }
    if (has(smd, "max_angular_accel_rad_s2")) {
        out->max_angular_accel_rad_s2 =
            asDouble(smd["max_angular_accel_rad_s2"], path + ".max_angular_accel_rad_s2");
    }
    if (has(smd, "velocity_feedforward")) {
        out->velocity_feedforward = asBool(smd["velocity_feedforward"], path + ".velocity_feedforward");
    }
    if (has(smd, "reengage_relatch_max_step_m")) {
        out->reengage_relatch_max_step_m =
            asDouble(smd["reengage_relatch_max_step_m"], path + ".reengage_relatch_max_step_m");
    }
    if (has(smd, "reengage_relatch_max_step_rad")) {
        out->reengage_relatch_max_step_rad =
            asDouble(smd["reengage_relatch_max_step_rad"], path + ".reengage_relatch_max_step_rad");
    }
    if (has(smd, "singularity_scale_full_sigma")) {
        out->singularity_scale_full_sigma =
            asDouble(smd["singularity_scale_full_sigma"], path + ".singularity_scale_full_sigma");
    }
    if (has(smd, "singularity_scale_floor_sigma")) {
        out->singularity_scale_floor_sigma =
            asDouble(smd["singularity_scale_floor_sigma"], path + ".singularity_scale_floor_sigma");
    }
    if (has(smd, "singularity_scale_min")) {
        out->singularity_scale_min =
            asDouble(smd["singularity_scale_min"], path + ".singularity_scale_min");
    }
}

void parseRuckigFollowerConfig(const YAML::Node& node, const std::string& path, RuckigFollowerConfig* out) {
    if (!out) return;
    validateAllowedKeys(node, {
        "enable",
        "controller",
        "fallback_policy",
        "engage_timeout_sec",
        "max_linear_velocity_m_s",
        "max_linear_accel_m_s2",
        "max_linear_jerk_m_s3",
        "max_angular_velocity_rad_s",
        "max_angular_accel_rad_s2",
        "max_angular_jerk_rad_s3",
        "discard_head_steps",
        "consume_steps",
        "reserve_steps",
        "smoothing_window",
        "af_damping_beta",
        "af_damping_beta_lin",
        "af_damping_beta_ang",
        "delta_twist_tau_sec",
        "delta_twist_residual_drain_steps",
        "delta_twist_clear_residual_on_new_frame",
        "delta_twist_min_time_to_go_sec",
        "delta_twist_max_residual_m",
        "delta_twist_max_residual_rad",
        "delta_twist_max_lead_m",
        "delta_twist_max_lead_rad",
        "delta_twist_stale_residual_timeout_sec",
        "preview_max_projection_error_m",
        "preview_max_projection_error_rad",
        "preview_max_consecutive_projection_errors",
        "preview_max_actual_lead_m",
        "preview_max_actual_lead_rad",
        "preview_max_consecutive_actual_lead_errors",
        "preview_projection_fault_policy",
        "loading_projection_max_accel_m_s2",
        "corner_deadband_lin_m",
        "corner_deadband_ang_rad",
        "corner_velocity_scale",
        "hold_bounce_resume_sec",
        "chunk_feed_timeout_sec",
    }, path);
    if (has(node, "enable")) out->enable = asBool(node["enable"], path + ".enable");
    if (has(node, "controller")) {
        const std::string value = lower(asString(node["controller"], path + ".controller"));
        if (value == "ruckig_waypoint") {
            out->controller = RuckigFollowerController::RuckigWaypoint;
        } else if (value == "delta_twist") {
            out->controller = RuckigFollowerController::DeltaTwist;
        } else if (value == "delta_preview") {
            out->controller = RuckigFollowerController::DeltaPreview;
        } else {
            fail("Unknown " + path + ".controller: " + value, node["controller"]);
        }
    }
    if (has(node, "fallback_policy")) {
        const std::string value = lower(asString(node["fallback_policy"], path + ".fallback_policy"));
        if (value == "smd") {
            out->fallback_policy = RuckigFollowerFallbackPolicy::Smd;
        } else if (value == "fault") {
            out->fallback_policy = RuckigFollowerFallbackPolicy::Fault;
        } else {
            fail("Unknown " + path + ".fallback_policy: " + value, node["fallback_policy"]);
        }
    }
    if (has(node, "engage_timeout_sec")) {
        out->engage_timeout_sec = asDouble(node["engage_timeout_sec"], path + ".engage_timeout_sec");
    }
    if (has(node, "max_linear_velocity_m_s")) {
        out->max_linear_velocity_m_s = asDouble(node["max_linear_velocity_m_s"], path + ".max_linear_velocity_m_s");
    }
    if (has(node, "max_linear_accel_m_s2")) {
        out->max_linear_accel_m_s2 = asDouble(node["max_linear_accel_m_s2"], path + ".max_linear_accel_m_s2");
    }
    if (has(node, "max_linear_jerk_m_s3")) {
        out->max_linear_jerk_m_s3 = asDouble(node["max_linear_jerk_m_s3"], path + ".max_linear_jerk_m_s3");
    }
    if (has(node, "max_angular_velocity_rad_s")) {
        out->max_angular_velocity_rad_s = asDouble(node["max_angular_velocity_rad_s"], path + ".max_angular_velocity_rad_s");
    }
    if (has(node, "max_angular_accel_rad_s2")) {
        out->max_angular_accel_rad_s2 = asDouble(node["max_angular_accel_rad_s2"], path + ".max_angular_accel_rad_s2");
    }
    if (has(node, "max_angular_jerk_rad_s3")) {
        out->max_angular_jerk_rad_s3 = asDouble(node["max_angular_jerk_rad_s3"], path + ".max_angular_jerk_rad_s3");
    }
    if (has(node, "discard_head_steps")) {
        out->discard_head_steps = asInt(node["discard_head_steps"], path + ".discard_head_steps");
    }
    if (has(node, "consume_steps")) {
        out->consume_steps = asInt(node["consume_steps"], path + ".consume_steps");
    }
    if (has(node, "reserve_steps")) {
        out->reserve_steps = asInt(node["reserve_steps"], path + ".reserve_steps");
    }
    if (has(node, "smoothing_window")) {
        out->smoothing_window = asInt(node["smoothing_window"], path + ".smoothing_window");
    }
    // Legacy scalar first so the per-class keys below can override it. A config that only sets
    // `af_damping_beta` therefore keeps its exact previous behavior on both axis classes.
    if (has(node, "af_damping_beta")) {
        const double beta = asDouble(node["af_damping_beta"], path + ".af_damping_beta");
        out->af_damping_beta_lin = beta;
        out->af_damping_beta_ang = beta;
    }
    if (has(node, "af_damping_beta_lin")) {
        out->af_damping_beta_lin =
            asDouble(node["af_damping_beta_lin"], path + ".af_damping_beta_lin");
    }
    if (has(node, "af_damping_beta_ang")) {
        out->af_damping_beta_ang =
            asDouble(node["af_damping_beta_ang"], path + ".af_damping_beta_ang");
    }
    if (has(node, "corner_deadband_lin_m")) {
        out->corner_deadband_lin_m =
            asDouble(node["corner_deadband_lin_m"], path + ".corner_deadband_lin_m");
    }
    if (has(node, "corner_deadband_ang_rad")) {
        out->corner_deadband_ang_rad =
            asDouble(node["corner_deadband_ang_rad"], path + ".corner_deadband_ang_rad");
    }
    if (has(node, "corner_velocity_scale")) {
        out->corner_velocity_scale =
            asDouble(node["corner_velocity_scale"], path + ".corner_velocity_scale");
    }
    if (has(node, "delta_twist_tau_sec")) {
        out->delta_twist_tau_sec = asDouble(node["delta_twist_tau_sec"], path + ".delta_twist_tau_sec");
    }
    if (has(node, "delta_twist_residual_drain_steps")) {
        out->delta_twist_residual_drain_steps =
            asInt(node["delta_twist_residual_drain_steps"], path + ".delta_twist_residual_drain_steps");
    }
    if (has(node, "delta_twist_clear_residual_on_new_frame")) {
        out->delta_twist_clear_residual_on_new_frame =
            asBool(
                node["delta_twist_clear_residual_on_new_frame"],
                path + ".delta_twist_clear_residual_on_new_frame"
            );
    }
    if (has(node, "delta_twist_min_time_to_go_sec")) {
        out->delta_twist_min_time_to_go_sec =
            asDouble(node["delta_twist_min_time_to_go_sec"], path + ".delta_twist_min_time_to_go_sec");
    }
    if (has(node, "delta_twist_max_residual_m")) {
        out->delta_twist_max_residual_m =
            asDouble(node["delta_twist_max_residual_m"], path + ".delta_twist_max_residual_m");
    }
    if (has(node, "delta_twist_max_residual_rad")) {
        out->delta_twist_max_residual_rad =
            asDouble(node["delta_twist_max_residual_rad"], path + ".delta_twist_max_residual_rad");
    }
    if (has(node, "delta_twist_max_lead_m")) {
        out->delta_twist_max_lead_m =
            asDouble(node["delta_twist_max_lead_m"], path + ".delta_twist_max_lead_m");
    }
    if (has(node, "delta_twist_max_lead_rad")) {
        out->delta_twist_max_lead_rad =
            asDouble(node["delta_twist_max_lead_rad"], path + ".delta_twist_max_lead_rad");
    }
    if (has(node, "delta_twist_stale_residual_timeout_sec")) {
        out->delta_twist_stale_residual_timeout_sec =
            asDouble(
                node["delta_twist_stale_residual_timeout_sec"],
                path + ".delta_twist_stale_residual_timeout_sec"
            );
    }
    if (has(node, "preview_max_projection_error_m")) {
        out->preview_max_projection_error_m = asDouble(
            node["preview_max_projection_error_m"], path + ".preview_max_projection_error_m");
    }
    if (has(node, "preview_max_projection_error_rad")) {
        out->preview_max_projection_error_rad = asDouble(
            node["preview_max_projection_error_rad"], path + ".preview_max_projection_error_rad");
    }
    if (has(node, "preview_max_consecutive_projection_errors")) {
        out->preview_max_consecutive_projection_errors = asInt(
            node["preview_max_consecutive_projection_errors"],
            path + ".preview_max_consecutive_projection_errors");
    }
    if (has(node, "preview_max_actual_lead_m")) {
        out->preview_max_actual_lead_m = asDouble(
            node["preview_max_actual_lead_m"], path + ".preview_max_actual_lead_m");
    }
    if (has(node, "preview_max_actual_lead_rad")) {
        out->preview_max_actual_lead_rad = asDouble(
            node["preview_max_actual_lead_rad"], path + ".preview_max_actual_lead_rad");
    }
    if (has(node, "loading_projection_max_accel_m_s2")) {
        out->loading_projection_max_accel_m_s2 = asDouble(
            node["loading_projection_max_accel_m_s2"],
            path + ".loading_projection_max_accel_m_s2");
    }
    if (has(node, "hold_bounce_resume_sec")) {
        out->hold_bounce_resume_sec = asDouble(
            node["hold_bounce_resume_sec"], path + ".hold_bounce_resume_sec");
    }
    if (has(node, "preview_projection_fault_policy")) {
        const std::string policy = lower(asString(
            node["preview_projection_fault_policy"],
            path + ".preview_projection_fault_policy"));
        if (policy == "fault") {
            out->preview_projection_fault_policy = RuckigProjectionFaultPolicy::Fault;
        } else if (policy == "warn") {
            out->preview_projection_fault_policy = RuckigProjectionFaultPolicy::Warn;
        } else {
            throw std::runtime_error(
                path + ".preview_projection_fault_policy must be fault or warn");
        }
    }
    if (has(node, "preview_max_consecutive_actual_lead_errors")) {
        out->preview_max_consecutive_actual_lead_errors = asInt(
            node["preview_max_consecutive_actual_lead_errors"],
            path + ".preview_max_consecutive_actual_lead_errors");
    }
    if (has(node, "chunk_feed_timeout_sec")) {
        out->chunk_feed_timeout_sec = asDouble(node["chunk_feed_timeout_sec"], path + ".chunk_feed_timeout_sec");
    }
}

void ensureTcpPoseTargetProfiles(DualArmConfig* cfg) {
    if (!cfg) return;
    if (cfg->cartesian_control.tcp_pose_target_profile_default.empty()) {
        cfg->cartesian_control.tcp_pose_target_profile_default = "default";
    }
    if (cfg->cartesian_control.tcp_pose_target_profiles.empty()) {
        TcpPoseTargetProfileConfig profile;
        profile.name = cfg->cartesian_control.tcp_pose_target_profile_default;
        profile.pose_track_smd = cfg->cartesian_control.pose_track_smd;
        profile.ruckig_follower = cfg->cartesian_control.ruckig_follower;
        cfg->cartesian_control.tcp_pose_target_profiles.push_back(profile);
    }
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

std::vector<std::string> dedupeStatePubEndpoints(std::vector<std::string> endpoints) {
    std::vector<std::string> unique;
    unique.reserve(endpoints.size());
    std::set<std::string> seen;
    for (const std::string& endpoint : endpoints) {
        if (seen.insert(endpoint).second) {
            unique.push_back(endpoint);
        } else {
            warn("duplicate network.state_pub_endpoints entry ignored: " + endpoint);
        }
    }
    return unique;
}

std::vector<std::string> dedupeScopePubEndpoints(std::vector<std::string> endpoints) {
    std::vector<std::string> unique;
    unique.reserve(endpoints.size());
    std::set<std::string> seen;
    for (const std::string& endpoint : endpoints) {
        if (seen.insert(endpoint).second) {
            unique.push_back(endpoint);
        } else {
            warn("duplicate network.scope_pub_endpoints entry ignored: " + endpoint);
        }
    }
    return unique;
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

JointBoolArray parseJointBoolArray(const YAML::Node& node, const std::string& path) {
    if (!node.IsSequence()) fail(path + " must be a sequence", node);
    if (node.size() != kDof) {
        fail(path + " must contain exactly 6 values", node);
    }
    JointBoolArray out{};
    for (int i = 0; i < kDof; ++i) {
        out[static_cast<std::size_t>(i)] =
            asBool(node[static_cast<std::size_t>(i)], path + "[" + std::to_string(i) + "]");
    }
    return out;
}

Pose6D parsePose6D(const YAML::Node& node, const std::string& path) {
    const JointArray v = parseJointArray(node, path);
    return Pose6D{v[0], v[1], v[2], v[3], v[4], v[5]};
}

Wrench6D parseWrench6D(const YAML::Node& node, const std::string& path) {
    const JointArray v = parseJointArray(node, path);
    return Wrench6D{v[0], v[1], v[2], v[3], v[4], v[5]};
}

std::array<double, 3> parseVec3(const YAML::Node& node, const std::string& path) {
    if (!node.IsSequence() || node.size() != 3) {
        fail(path + " must contain exactly 3 values", node);
    }
    return {
        asDouble(node[0], path + "[0]"),
        asDouble(node[1], path + "[1]"),
        asDouble(node[2], path + "[2]"),
    };
}

std::array<double, 9> parseMatrix3RowMajor(
    const YAML::Node& node,
    const std::string& path
) {
    if (!node.IsSequence() || node.size() != 9) {
        fail(path + " must contain exactly 9 row-major values", node);
    }
    std::array<double, 9> out{};
    for (std::size_t i = 0; i < out.size(); ++i) {
        out[i] = asDouble(node[i], path + "[" + std::to_string(i) + "]");
    }
    return out;
}

// Parse a safety constraint's tcp_offset_points sequence (floor/roi/reach/user_floor
// all share the FloorCheckPointConfig schema). Each entry has a name, the OPEN
// offset_m: [x, y, z], and an OPTIONAL offset_closed_m: [x, y, z] (the gripper-closed
// position; when omitted it mirrors offset_m so interpolation is the identity and the
// point stays static, the legacy behavior). path is the YAML path for error messages.
std::vector<FloorCheckPointConfig> parseTcpOffsetPoints(const YAML::Node& points,
                                                        const std::string& path) {
    if (!points.IsSequence()) {
        fail(path + " must be a sequence", points);
    }
    const auto parseVec3 = [&](const YAML::Node& node, const std::string& sub,
                               std::array<double, 3>& out) {
        if (!node.IsSequence() || node.size() != 3) {
            fail(path + " entries need " + sub + ": [x, y, z]", node);
        }
        for (std::size_t axis = 0; axis < 3; ++axis) {
            out[axis] = asDouble(node[axis], path + "." + sub);
        }
    };
    std::vector<FloorCheckPointConfig> out;
    out.reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
        const YAML::Node entry = points[i];
        validateAllowedKeys(entry, {"name", "offset_m", "offset_closed_m"}, path);
        FloorCheckPointConfig point;
        if (has(entry, "name")) {
            point.name = asString(entry["name"], path + ".name");
        }
        if (!has(entry, "offset_m")) {
            fail(path + " entries need offset_m: [x, y, z]", entry);
        }
        parseVec3(entry["offset_m"], "offset_m", point.offset_m);
        if (has(entry, "offset_closed_m")) {
            parseVec3(entry["offset_closed_m"], "offset_closed_m", point.offset_closed_m);
            point.has_closed = true;
        } else {
            point.offset_closed_m = point.offset_m;  // static: identity interpolation
            point.has_closed = false;
        }
        out.push_back(std::move(point));
    }
    return out;
}

// Semantic validation shared by all four constraints' offset-point lists (non-empty
// name, finite open AND closed offsets). Runs on the final config regardless of source.
void validateTcpOffsetPoints(const std::vector<FloorCheckPointConfig>& points,
                             const std::string& path) {
    for (const FloorCheckPointConfig& point : points) {
        if (point.name.empty()) {
            throw std::runtime_error(path + " entries need a non-empty name");
        }
        for (double value : point.offset_m) {
            if (!std::isfinite(value)) {
                throw std::runtime_error(path + " offsets must be finite (" + point.name + ")");
            }
        }
        for (double value : point.offset_closed_m) {
            if (!std::isfinite(value)) {
                throw std::runtime_error(
                    path + " offset_closed_m must be finite (" + point.name + ")");
            }
        }
    }
}

BackendType parseBackendType(const YAML::Node& node, const std::string& path) {
    const std::string value = lower(asString(node, path));
    if (value == "mock") return BackendType::Mock;
    if (value == "rbpodo") return BackendType::Rbpodo;
    fail("Unknown backend_type: " + value, node);
}

RunMode parseRunMode(const YAML::Node& node, const std::string& path) {
    const std::string value = lower(asString(node, path));
    if (value == "mock") return RunMode::Mock;
    if (value == "simulation") return RunMode::Simulation;
    if (value == "real") return RunMode::Real;
    if (value == "sim") {
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

RbpodoAsyncStreamingMode parseRbpodoAsyncStreamingMode(const YAML::Node& node, const std::string& path) {
    const std::string value = lower(asString(node, path));
    if (value == "disabled") return RbpodoAsyncStreamingMode::Disabled;
    if (value == "sdk_ack_worker") return RbpodoAsyncStreamingMode::SdkAckWorker;
    if (value == "socket_send_supervised") return RbpodoAsyncStreamingMode::SocketSendSupervised;
    fail("Unknown " + path + ": " + value, node);
}

RbpodoAsyncQueuePolicy parseRbpodoAsyncQueuePolicy(const YAML::Node& node, const std::string& path) {
    const std::string value = lower(asString(node, path));
    if (value == "latest_wins") return RbpodoAsyncQueuePolicy::LatestWins;
    fail("Unknown " + path + ": " + value, node);
}

RbpodoAsyncReferenceSupervisionPolicy parseRbpodoAsyncReferenceSupervisionPolicy(
    const YAML::Node& node,
    const std::string& path
) {
    const std::string value = lower(asString(node, path));
    if (value == "warn_only") return RbpodoAsyncReferenceSupervisionPolicy::WarnOnly;
    if (value == "fault_latch") return RbpodoAsyncReferenceSupervisionPolicy::FaultLatch;
    fail("Unknown " + path + ": " + value, node);
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

CartesianControllerSimulationStateSource parseCartesianControllerSimulationStateSource(
    const YAML::Node& node,
    const std::string& path
) {
    const std::string value = lower(asString(node, path));
    if (value == "actual") return CartesianControllerSimulationStateSource::Actual;
    if (value == "reference") return CartesianControllerSimulationStateSource::Reference;
    fail("Unknown " + path + ": " + value, node);
}

ControllerSimulationTrackingErrorSource parseControllerSimulationTrackingErrorSource(
    const YAML::Node& node,
    const std::string& path
) {
    const std::string value = lower(asString(node, path));
    if (value == "actual") return ControllerSimulationTrackingErrorSource::Actual;
    if (value == "reference") return ControllerSimulationTrackingErrorSource::Reference;
    fail("Unknown " + path + ": " + value, node);
}

ControllerSimulationPhysicalMotionPolicy parseControllerSimulationPhysicalMotionPolicy(
    const YAML::Node& node,
    const std::string& path
) {
    const std::string value = lower(asString(node, path));
    if (value == "warn_only") return ControllerSimulationPhysicalMotionPolicy::WarnOnly;
    if (value == "fault_latch") return ControllerSimulationPhysicalMotionPolicy::FaultLatch;
    fail("Unknown " + path + ": " + value, node);
}

SelfCollisionFailPolicy parseSelfCollisionFailPolicy(
    const YAML::Node& node,
    const std::string& path
) {
    const std::string value = lower(asString(node, path));
    if (value == "clamp_hold" || value == "clamp_to_hold") return SelfCollisionFailPolicy::ClampToHold;
    if (value == "fault_latch") return SelfCollisionFailPolicy::FaultLatch;
    fail("Unknown " + path + ": " + value, node);
}

FloorConstraintFailPolicy parseFloorConstraintFailPolicy(
    const YAML::Node& node,
    const std::string& path
) {
    const std::string value = lower(asString(node, path));
    if (value == "clamp_hold" || value == "clamp_to_hold") return FloorConstraintFailPolicy::ClampToHold;
    if (value == "fault_latch") return FloorConstraintFailPolicy::FaultLatch;
    fail("Unknown " + path + ": " + value, node);
}

std::string getString(const YAML::Node& sec, const std::string& key, const std::string& fallback, const std::string& path) {
    return has(sec, key) ? asString(sec[key], path + "." + key) : fallback;
}

bool isEnvNameStart(char c) {
    const unsigned char value = static_cast<unsigned char>(c);
    return std::isalpha(value) || c == '_';
}

bool isEnvNameChar(char c) {
    const unsigned char value = static_cast<unsigned char>(c);
    return std::isalnum(value) || c == '_';
}

std::string expandEnvReferences(
    const std::string& value,
    const std::string& field_path,
    const YAML::Node& node
) {
    std::string expanded;
    std::size_t cursor = 0;
    while (cursor < value.size()) {
        const std::size_t start = value.find("${", cursor);
        if (start == std::string::npos) {
            expanded.append(value.substr(cursor));
            break;
        }
        expanded.append(value.substr(cursor, start - cursor));
        const std::size_t end = value.find('}', start + 2);
        if (end == std::string::npos) {
            fail(field_path + " contains an unterminated environment reference", node);
        }
        const std::string name = value.substr(start + 2, end - start - 2);
        if (name.empty() || !isEnvNameStart(name.front())) {
            fail(field_path + " contains an invalid environment variable reference: ${" + name + "}", node);
        }
        for (char c : name) {
            if (!isEnvNameChar(c)) {
                fail(field_path + " contains an invalid environment variable reference: ${" + name + "}", node);
            }
        }
        const char* env_value = std::getenv(name.c_str());
        if (!env_value) {
            fail(field_path + " references unset environment variable ${" + name + "}", node);
        }
        if (std::string(env_value).empty()) {
            fail(field_path + " references empty environment variable ${" + name + "}", node);
        }
        expanded.append(env_value);
        cursor = end + 1;
    }
    return expanded;
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

void applyRbpodoAsyncStreamingSection(
    const YAML::Node& sec,
    RbpodoAsyncStreamingConfig* cfg,
    const std::string& path
) {
    validateAllowedKeys(sec, {
        "enable",
        "mode",
        "rate_hz",
        "queue_policy",
        "max_pending_age_ms",
        "ack_supervision",
        "reference_supervision",
        "diagnostics",
    }, path);

    if (has(sec, "enable")) cfg->enable = asBool(sec["enable"], path + ".enable");
    if (has(sec, "mode")) cfg->mode = parseRbpodoAsyncStreamingMode(sec["mode"], path + ".mode");
    if (has(sec, "rate_hz")) cfg->rate_hz = asInt(sec["rate_hz"], path + ".rate_hz");
    if (has(sec, "queue_policy")) {
        cfg->queue_policy = parseRbpodoAsyncQueuePolicy(sec["queue_policy"], path + ".queue_policy");
    }
    if (has(sec, "max_pending_age_ms")) {
        cfg->max_pending_age_ms = asDouble(sec["max_pending_age_ms"], path + ".max_pending_age_ms");
    }

    if (has(sec, "ack_supervision")) {
        const YAML::Node ack = sec["ack_supervision"];
        validateAllowedKeys(ack, {
            "enable",
            "expected_ack_timeout_ms",
            "missing_ack_fault_after_ms",
            "max_consecutive_missing_ack",
        }, path + ".ack_supervision");
        if (has(ack, "enable")) {
            cfg->ack_supervision.enable = asBool(ack["enable"], path + ".ack_supervision.enable");
        }
        if (has(ack, "expected_ack_timeout_ms")) {
            cfg->ack_supervision.expected_ack_timeout_ms =
                asDouble(ack["expected_ack_timeout_ms"], path + ".ack_supervision.expected_ack_timeout_ms");
        }
        if (has(ack, "missing_ack_fault_after_ms")) {
            cfg->ack_supervision.missing_ack_fault_after_ms =
                asDouble(ack["missing_ack_fault_after_ms"], path + ".ack_supervision.missing_ack_fault_after_ms");
        }
        if (has(ack, "max_consecutive_missing_ack")) {
            cfg->ack_supervision.max_consecutive_missing_ack =
                asInt(ack["max_consecutive_missing_ack"], path + ".ack_supervision.max_consecutive_missing_ack");
        }
    }

    if (has(sec, "reference_supervision")) {
        const YAML::Node ref = sec["reference_supervision"];
        validateAllowedKeys(ref, {
            "enable",
            "q_ref_update_timeout_ms",
            "q_ref_target_tolerance_deg",
            "q_ref_target_fault_after_ms",
            "tcp_ref_update_timeout_ms",
            "tcp_ref_target_tolerance_m",
            "tcp_ref_target_fault_after_ms",
            "policy",
        }, path + ".reference_supervision");
        if (has(ref, "enable")) {
            cfg->reference_supervision.enable = asBool(ref["enable"], path + ".reference_supervision.enable");
        }
        if (has(ref, "q_ref_update_timeout_ms")) {
            cfg->reference_supervision.q_ref_update_timeout_ms =
                asDouble(ref["q_ref_update_timeout_ms"], path + ".reference_supervision.q_ref_update_timeout_ms");
        }
        if (has(ref, "q_ref_target_tolerance_deg")) {
            cfg->reference_supervision.q_ref_target_tolerance_deg =
                asDouble(ref["q_ref_target_tolerance_deg"], path + ".reference_supervision.q_ref_target_tolerance_deg");
        }
        if (has(ref, "q_ref_target_fault_after_ms")) {
            cfg->reference_supervision.q_ref_target_fault_after_ms =
                asDouble(ref["q_ref_target_fault_after_ms"], path + ".reference_supervision.q_ref_target_fault_after_ms");
        }
        if (has(ref, "tcp_ref_update_timeout_ms")) {
            cfg->reference_supervision.tcp_ref_update_timeout_ms =
                asDouble(ref["tcp_ref_update_timeout_ms"], path + ".reference_supervision.tcp_ref_update_timeout_ms");
        }
        if (has(ref, "tcp_ref_target_tolerance_m")) {
            cfg->reference_supervision.tcp_ref_target_tolerance_m =
                asDouble(ref["tcp_ref_target_tolerance_m"], path + ".reference_supervision.tcp_ref_target_tolerance_m");
        }
        if (has(ref, "tcp_ref_target_fault_after_ms")) {
            cfg->reference_supervision.tcp_ref_target_fault_after_ms =
                asDouble(ref["tcp_ref_target_fault_after_ms"], path + ".reference_supervision.tcp_ref_target_fault_after_ms");
        }
        if (has(ref, "policy")) {
            cfg->reference_supervision.policy =
                parseRbpodoAsyncReferenceSupervisionPolicy(ref["policy"], path + ".reference_supervision.policy");
        }
    }

    if (has(sec, "diagnostics")) {
        const YAML::Node diagnostics = sec["diagnostics"];
        validateAllowedKeys(diagnostics, {
            "publish_per_command_jsonl",
        }, path + ".diagnostics");
        if (has(diagnostics, "publish_per_command_jsonl")) {
            cfg->diagnostics.publish_per_command_jsonl =
                asBool(diagnostics["publish_per_command_jsonl"], path + ".diagnostics.publish_per_command_jsonl");
        }
    }
}

void applyBackendSection(const YAML::Node& sec, BackendConfig* cfg, const std::string& path) {
    validateAllowedKeys(sec, {
        "backend_type",
        "run_mode",
        "name",
        "ip",
        "operation_mode",
        "command_timeout_sec",
        "initial_q_deg",
        "speed_bar",
        "servo_t1_sec",
        "servo_t2_sec",
        "servo_time_sec",
        "servo_lookahead_sec",
        "servo_gain",
        "servo_alpha",
        "servo_acc",
        "servo_soft_entry_enable",
        "servo_soft_entry_sec",
        "servo_soft_entry_gain_start_scale",
        "servo_soft_entry_rearm_gap_sec",
        "disable_waiting_ack",
        "state_read_pipelined",
        "max_consecutive_read_misses",
    }, path);

    if (has(sec, "backend_type")) cfg->backend_type = parseBackendType(sec["backend_type"], path + ".backend_type");
    if (has(sec, "run_mode")) cfg->run_mode = parseRunMode(sec["run_mode"], path + ".run_mode");
    cfg->name = getString(sec, "name", cfg->name, path);
    cfg->ip = has(sec, "ip")
        ? expandEnvReferences(asString(sec["ip"], path + ".ip"), path + ".ip", sec["ip"])
        : cfg->ip;
    cfg->operation_mode = getString(sec, "operation_mode", cfg->operation_mode, path);

    if (has(sec, "command_timeout_sec")) cfg->command_timeout_sec = asDouble(sec["command_timeout_sec"], path + ".command_timeout_sec");

    if (has(sec, "initial_q_deg")) cfg->initial_q_deg = parseJointArray(sec["initial_q_deg"], path + ".initial_q_deg");
    if (has(sec, "speed_bar")) cfg->speed_bar = asDouble(sec["speed_bar"], path + ".speed_bar");
    applyDeprecatedDoubleAlias(sec, "servo_t1_sec", "servo_time_sec", path, &cfg->servo_t1_sec);
    applyDeprecatedDoubleAlias(sec, "servo_t2_sec", "servo_lookahead_sec", path, &cfg->servo_t2_sec);
    if (has(sec, "servo_gain")) cfg->servo_gain = asDouble(sec["servo_gain"], path + ".servo_gain");
    applyDeprecatedDoubleAlias(sec, "servo_alpha", "servo_acc", path, &cfg->servo_alpha);
    cfg->servo_time_sec = cfg->servo_t1_sec;
    cfg->servo_lookahead_sec = cfg->servo_t2_sec;
    cfg->servo_acc = cfg->servo_alpha;
    if (has(sec, "servo_soft_entry_enable")) cfg->servo_soft_entry_enable = asBool(sec["servo_soft_entry_enable"], path + ".servo_soft_entry_enable");
    if (has(sec, "servo_soft_entry_sec")) cfg->servo_soft_entry_sec = asDouble(sec["servo_soft_entry_sec"], path + ".servo_soft_entry_sec");
    if (has(sec, "servo_soft_entry_gain_start_scale")) cfg->servo_soft_entry_gain_start_scale = asDouble(sec["servo_soft_entry_gain_start_scale"], path + ".servo_soft_entry_gain_start_scale");
    if (has(sec, "servo_soft_entry_rearm_gap_sec")) cfg->servo_soft_entry_rearm_gap_sec = asDouble(sec["servo_soft_entry_rearm_gap_sec"], path + ".servo_soft_entry_rearm_gap_sec");
    if (has(sec, "disable_waiting_ack")) cfg->disable_waiting_ack = asBool(sec["disable_waiting_ack"], path + ".disable_waiting_ack");
    if (has(sec, "state_read_pipelined")) cfg->state_read_pipelined = asBool(sec["state_read_pipelined"], path + ".state_read_pipelined");
    if (has(sec, "max_consecutive_read_misses")) cfg->max_consecutive_read_misses = asInt(sec["max_consecutive_read_misses"], path + ".max_consecutive_read_misses");
}

bool anyReal(const DualArmConfig& cfg) {
    return cfg.left_robot.run_mode == RunMode::Real || cfg.right_robot.run_mode == RunMode::Real;
}

bool anyRbpodo(const DualArmConfig& cfg) {
    return cfg.left_robot.backend_type == BackendType::Rbpodo ||
           cfg.right_robot.backend_type == BackendType::Rbpodo;
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

// Read a single-DOF joint's <limit lower/upper> (radians) directly from a URDF file.
// Deliberately a small, dependency-free text scan (not a full XML/pinocchio parse): it
// is diagnostic-only and must never throw or block startup. Returns {lower_rad, upper_rad}
// in degrees, or nullopt if the joint or its limit cannot be located.
std::optional<std::pair<double, double>> readUrdfJointLimitDeg(
    const std::string& urdf_path,
    const std::string& joint_name
) {
    std::ifstream file(urdf_path);
    if (!file) return std::nullopt;
    std::stringstream buffer;
    buffer << file.rdbuf();
    const std::string xml = buffer.str();

    const std::string needle = "name=\"" + joint_name + "\"";
    const std::size_t joint_pos = xml.find(needle);
    if (joint_pos == std::string::npos) return std::nullopt;
    // The <limit .../> must belong to this joint: stop at the joint's closing tag.
    const std::size_t joint_end = xml.find("</joint>", joint_pos);
    const std::size_t limit_pos = xml.find("<limit", joint_pos);
    if (limit_pos == std::string::npos ||
        (joint_end != std::string::npos && limit_pos > joint_end)) {
        return std::nullopt;
    }
    const std::size_t limit_end = xml.find('>', limit_pos);
    if (limit_end == std::string::npos) return std::nullopt;
    const std::string tag = xml.substr(limit_pos, limit_end - limit_pos);

    const auto readAttr = [&tag](const std::string& attr) -> std::optional<double> {
        const std::string key = attr + "=\"";
        const std::size_t p = tag.find(key);
        if (p == std::string::npos) return std::nullopt;
        const std::size_t v = p + key.size();
        const std::size_t e = tag.find('"', v);
        if (e == std::string::npos) return std::nullopt;
        try {
            return std::stod(tag.substr(v, e - v));
        } catch (...) {
            return std::nullopt;
        }
    };
    const std::optional<double> lower = readAttr("lower");
    const std::optional<double> upper = readAttr("upper");
    if (!lower || !upper) return std::nullopt;
    constexpr double kRadToDeg = 180.0 / 3.141592653589793238462643383279502884;
    return std::make_pair(*lower * kRadToDeg, *upper * kRadToDeg);
}

void warnIfRbpodoSafetyRangeDiffersFromKnownUrdf(const DualArmConfig& cfg) {
    if (!cfg.kinematics.enable || !anyRbpodo(cfg)) return;
    if (std::filesystem::path(cfg.kinematics.urdf).filename().string() != "rb3_730e.urdf") return;

    const JointArray urdf_min_deg{-360.0, -360.0, -150.0, -360.0, -360.0, -360.0};
    const JointArray urdf_max_deg{360.0, 360.0, 150.0, 360.0, 360.0, 360.0};
    constexpr double kToleranceDeg = 0.5;

    // Guard against the actual URDF file silently drifting from the expected physical
    // limits above (the J3/elbow +/-150 -> +/-360 drift that made IK pick an unreachable
    // elbow branch). This reads the real file, so a regenerated/overwritten URDF is
    // caught here instead of only at runtime as a branch-flip. J3 is the load-bearing one.
    const std::optional<std::pair<double, double>> elbow_deg =
        readUrdfJointLimitDeg(cfg.kinematics.urdf, "elbow_joint");
    if (elbow_deg) {
        if (std::abs(elbow_deg->first - urdf_min_deg[2]) > kToleranceDeg ||
            std::abs(elbow_deg->second - urdf_max_deg[2]) > kToleranceDeg) {
            warn(
                "rb3_730e URDF elbow_joint (J3) limit=[" +
                std::to_string(elbow_deg->first) + ", " + std::to_string(elbow_deg->second) +
                "] deg does NOT match the RB3-730E physical range [" +
                std::to_string(urdf_min_deg[2]) + ", " + std::to_string(urdf_max_deg[2]) +
                "] deg. The elbow cannot reach +/-360; a widened URDF makes IK select an "
                "unreachable elbow branch the controller rejects (TCP branch-flip / lurch). "
                "Restore elbow to +/-2.618 rad. See docs/joint_range_policy.md."
            );
        }
    }
    for (int i = 0; i < kDof; ++i) {
        if (std::abs(cfg.safety.q_min_deg[i] - urdf_min_deg[i]) <= kToleranceDeg &&
            std::abs(cfg.safety.q_max_deg[i] - urdf_max_deg[i]) <= kToleranceDeg) {
            continue;
        }
        const std::string joint_name = i < static_cast<int>(cfg.kinematics.joint_names.size())
            ? cfg.kinematics.joint_names[static_cast<std::size_t>(i)]
            : ("joint_" + std::to_string(i + 1));
        warn(
            "rbpodo safety q_min/q_max differs from rb3_730e URDF IK limit for " +
            joint_name + ": safety=[" + std::to_string(cfg.safety.q_min_deg[i]) +
            ", " + std::to_string(cfg.safety.q_max_deg[i]) + "], urdf=[" +
            std::to_string(urdf_min_deg[i]) + ", " + std::to_string(urdf_max_deg[i]) +
            "]. Raw rbpodo state/commands stay in configured safety range; IK may still be model-limited."
        );
    }
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
    validateNonNegativeFinite(
        cfg.safety.controller_simulation_physical_motion_threshold_deg,
        "safety.controller_simulation_physical_motion_threshold_deg"
    );
    validateNonNegativeFiniteArray(cfg.safety.joint_wrap_period_deg, "safety.joint_wrap_period_deg");
    if (cfg.safety.self_collision.enable) {
        // The only implementation is the async URDF-mesh CollisionMonitor, so the
        // mesh geometry + barrier params are mandatory when the guard is enabled.
        if (!cfg.kinematics.enable) {
            throw std::runtime_error(
                "safety.self_collision.enable=true requires kinematics.enable=true (link geometry source)");
        }
        const auto& m = cfg.safety.self_collision.mesh;
        if (m.unified_urdf.empty()) {
            throw std::runtime_error(
                "safety.self_collision.enable=true requires mesh.unified_urdf (stand+both-arms URDF)");
        }
        validatePositiveFinite(m.d_hard_m, "safety.self_collision.mesh.d_hard_m");
        validatePositiveFinite(m.d_slow_m, "safety.self_collision.mesh.d_slow_m");
        validatePositiveFinite(m.a_brake_m_s2, "safety.self_collision.mesh.a_brake_m_s2");
        validatePositiveFinite(m.max_staleness_s, "safety.self_collision.mesh.max_staleness_s");
        const auto validateOptionalPositive = [](double value, const std::string& name) {
            if (!std::isfinite(value)) {
                throw std::runtime_error(name + " must be finite");
            }
            if (value > 0.0) {
                validatePositiveFinite(value, name);
            }
        };
        validateOptionalPositive(
            m.intra_arm.d_hard_m, "safety.self_collision.mesh.intra_arm.d_hard_m");
        validateOptionalPositive(
            m.intra_arm.d_slow_m, "safety.self_collision.mesh.intra_arm.d_slow_m");
        validateOptionalPositive(
            m.intra_arm.a_brake_m_s2, "safety.self_collision.mesh.intra_arm.a_brake_m_s2");
        validateOptionalPositive(
            m.intra_arm.hyst_m, "safety.self_collision.mesh.intra_arm.hyst_m");
        validateOptionalPositive(
            m.intra_arm.recover_speed_m_s,
            "safety.self_collision.mesh.intra_arm.recover_speed_m_s");
        validateOptionalPositive(
            m.intra_arm.latency_s, "safety.self_collision.mesh.intra_arm.latency_s");
        const double intra_hard =
            m.intra_arm.d_hard_m > 0.0 ? m.intra_arm.d_hard_m : m.d_hard_m;
        const double intra_slow =
            m.intra_arm.d_slow_m > 0.0 ? m.intra_arm.d_slow_m : m.d_slow_m;
        if (intra_slow < intra_hard) {
            throw std::runtime_error(
                "safety.self_collision.mesh.intra_arm.d_slow_m must be >= d_hard_m");
        }
        if (m.external_boxes.max_count < 0) {
            throw std::runtime_error(
                "safety.self_collision.mesh.external_boxes.max_count must be >= 0");
        }
        for (std::size_t i = 0; i < m.external_boxes.size_m.size(); ++i) {
            validatePositiveFinite(
                m.external_boxes.size_m[i],
                "safety.self_collision.mesh.external_boxes.size_m[" + std::to_string(i) + "]");
        }
        for (std::size_t i = 0; i < m.external_boxes.margin_m.size(); ++i) {
            validateNonNegativeFinite(
                m.external_boxes.margin_m[i],
                "safety.self_collision.mesh.external_boxes.margin_m[" + std::to_string(i) + "]");
        }
        validatePositiveFinite(
            m.external_boxes.stale_timeout_s,
            "safety.self_collision.mesh.external_boxes.stale_timeout_s");
        if (m.external_boxes.stale_policy != "hold" &&
            m.external_boxes.stale_policy != "disable") {
            throw std::runtime_error(
                "safety.self_collision.mesh.external_boxes.stale_policy must be 'hold' or 'disable'");
        }
        {
            const auto& b = m.external_boxes.barrier;
            validateNonNegativeFinite(b.d_hard_m, "safety.self_collision.mesh.external_boxes.barrier.d_hard_m");
            validatePositiveFinite(b.d_slow_m, "safety.self_collision.mesh.external_boxes.barrier.d_slow_m");
            validatePositiveFinite(b.a_brake_m_s2, "safety.self_collision.mesh.external_boxes.barrier.a_brake_m_s2");
            validateNonNegativeFinite(b.hyst_m, "safety.self_collision.mesh.external_boxes.barrier.hyst_m");
            validateNonNegativeFinite(b.recover_speed_m_s, "safety.self_collision.mesh.external_boxes.barrier.recover_speed_m_s");
            validatePositiveFinite(b.latency_s, "safety.self_collision.mesh.external_boxes.barrier.latency_s");
            if (b.d_slow_m < b.d_hard_m) {
                throw std::runtime_error(
                    "safety.self_collision.mesh.external_boxes.barrier.d_slow_m must be >= d_hard_m");
            }
        }
        if (m.d_slow_m < m.d_hard_m) {
            throw std::runtime_error(
                "safety.self_collision.mesh.d_slow_m must be >= d_hard_m");
        }
        for (const auto& rule : m.disabled_collision_pairs) {
            if (rule.pattern_a.empty() || rule.pattern_b.empty()) {
                throw std::runtime_error(
                    "safety.self_collision.mesh.disabled_collision_pairs patterns must be non-empty");
            }
        }
    }
    if (cfg.safety.joint_target_smd.enable) {
        const auto& js = cfg.safety.joint_target_smd;
        validatePositiveFinite(js.damping_ratio, "safety.joint_target_smd.damping_ratio");
        validatePositiveFinite(js.natural_frequency_hz, "safety.joint_target_smd.natural_frequency_hz");
        validatePositiveFiniteArray(js.max_velocity_deg_s, "safety.joint_target_smd.max_velocity_deg_s");
        validatePositiveFiniteArray(js.max_accel_deg_s2, "safety.joint_target_smd.max_accel_deg_s2");
        if (js.arrival_taper_enable) {
            validatePositiveFinite(js.arrival_decel_deg_s2, "safety.joint_target_smd.arrival_decel_deg_s2");
            validateNonNegativeFinite(js.arrival_min_speed_deg_s, "safety.joint_target_smd.arrival_min_speed_deg_s");
        }
    }
    if (cfg.safety.init_motion_planner.enable) {
        const auto& ip = cfg.safety.init_motion_planner;
        // The planner's collision+floor oracle IS the mesh self-collision model, so
        // the mesh guard must be configured. Fail-closed: refuse to come up enabled
        // without it (otherwise InitMotion would plan against nothing).
        if (!cfg.safety.self_collision.enable) {
            throw std::runtime_error(
                "safety.init_motion_planner.enable requires safety.self_collision.enable "
                "(the mesh model is the planner's collision oracle)");
        }
        validatePositiveFinite(ip.max_planning_time_sec, "safety.init_motion_planner.max_planning_time_sec");
        validatePositiveFinite(ip.step_size_rad, "safety.init_motion_planner.step_size_rad");
        validatePositiveFinite(ip.edge_resolution_rad, "safety.init_motion_planner.edge_resolution_rad");
        validatePositiveFinite(ip.waypoint_tol_deg, "safety.init_motion_planner.waypoint_tol_deg");
        validateNonNegativeFinite(ip.noop_tol_deg, "safety.init_motion_planner.noop_tol_deg");
        validatePositiveFinite(ip.max_segment_deg, "safety.init_motion_planner.max_segment_deg");
        if (ip.max_iterations <= 0) {
            throw std::runtime_error("safety.init_motion_planner.max_iterations must be > 0");
        }
        if (ip.escape_max_steps <= 0) {
            throw std::runtime_error("safety.init_motion_planner.escape_max_steps must be > 0");
        }
        if (ip.escape_restart_attempts < 0) {
            throw std::runtime_error("safety.init_motion_planner.escape_restart_attempts must be >= 0");
        }
        if (!std::isfinite(ip.goal_bias) || ip.goal_bias < 0.0 || ip.goal_bias > 1.0) {
            throw std::runtime_error("safety.init_motion_planner.goal_bias must be in [0, 1]");
        }
        if (!std::isfinite(ip.global_sample_fraction) ||
            ip.global_sample_fraction < 0.0 || ip.global_sample_fraction > 1.0) {
            throw std::runtime_error("safety.init_motion_planner.global_sample_fraction must be in [0, 1]");
        }
        for (double margin : ip.sample_margin_deg_per_joint) {
            if (!std::isfinite(margin)) {
                throw std::runtime_error(
                    "safety.init_motion_planner.sample_margin_deg_per_joint values must be finite");
            }
        }
        if (ip.goal_bias + ip.global_sample_fraction > 1.0) {
            std::ostringstream oss;
            oss << "safety.init_motion_planner.goal_bias + global_sample_fraction must be <= 1.0"
                << " (goal_bias=" << ip.goal_bias
                << ", global_sample_fraction=" << ip.global_sample_fraction << ")";
            throw std::runtime_error(oss.str());
        }
        validateNonNegativeFinite(ip.global_sample_margin_deg,
                                  "safety.init_motion_planner.global_sample_margin_deg");
        validateNonNegativeFinite(ip.escape_max_time_sec,
                                  "safety.init_motion_planner.escape_max_time_sec");
        validateNonNegativeFinite(ip.escape_perturb_deg,
                                  "safety.init_motion_planner.escape_perturb_deg");
    }
    if (cfg.safety.floor_constraint.enable) {
        const auto& fc = cfg.safety.floor_constraint;
        if (!std::isfinite(fc.z_min_m) || !std::isfinite(fc.runtime_min_z_m) ||
            !std::isfinite(fc.runtime_max_z_m)) {
            throw std::runtime_error("safety.floor_constraint values must be finite");
        }
        if (fc.runtime_min_z_m > fc.z_min_m || fc.z_min_m > fc.runtime_max_z_m) {
            throw std::runtime_error(
                "safety.floor_constraint requires runtime_min_z_m <= z_min_m <= runtime_max_z_m");
        }
        if (!cfg.kinematics.enable) {
            throw std::runtime_error(
                "safety.floor_constraint.enable=true requires kinematics.enable=true (TCP FK source)");
        }
        validateTcpOffsetPoints(fc.tcp_offset_points, "safety.floor_constraint.tcp_offset_points");
    }
    if (cfg.safety.roi_box.enable) {
        const auto& rb = cfg.safety.roi_box;
        for (int k = 0; k < 3; ++k) {
            if (!std::isfinite(rb.min_m[k]) || !std::isfinite(rb.max_m[k]) ||
                !std::isfinite(rb.runtime_min_m[k]) || !std::isfinite(rb.runtime_max_m[k])) {
                throw std::runtime_error("safety.roi_box bounds must be finite");
            }
            // runtime_min <= min <= max <= runtime_max on every axis.
            if (rb.runtime_min_m[k] > rb.min_m[k] || rb.min_m[k] > rb.max_m[k] ||
                rb.max_m[k] > rb.runtime_max_m[k]) {
                throw std::runtime_error(
                    "safety.roi_box requires runtime_min_m <= min_m <= max_m <= runtime_max_m on each axis");
            }
        }
        if (!std::isfinite(rb.a_brake_m_s2) || rb.a_brake_m_s2 <= 0.0) {
            throw std::runtime_error("safety.roi_box.a_brake_m_s2 must be finite and positive");
        }
        if (!std::isfinite(rb.d_slow_m) || rb.d_slow_m < 0.0) {
            throw std::runtime_error("safety.roi_box.d_slow_m must be finite and non-negative");
        }
        if (!cfg.kinematics.enable) {
            throw std::runtime_error(
                "safety.roi_box.enable=true requires kinematics.enable=true (TCP FK source)");
        }
        validateTcpOffsetPoints(rb.tcp_offset_points, "safety.roi_box.tcp_offset_points");
    }
    if (cfg.safety.reach_constraint.enable) {
        const auto& rc = cfg.safety.reach_constraint;
        if (!std::isfinite(rc.r_max_m) || rc.r_max_m <= 0.0) {
            throw std::runtime_error("safety.reach_constraint.r_max_m must be finite and positive");
        }
        if (!std::isfinite(rc.r_min_m)) {
            throw std::runtime_error("safety.reach_constraint.r_min_m must be finite");
        }
        // r_min_m <= 0 disables the inner shell; when active it must be below r_max.
        if (rc.r_min_m > 0.0 && rc.r_min_m >= rc.r_max_m) {
            throw std::runtime_error(
                "safety.reach_constraint requires r_min_m < r_max_m when the inner shell is active");
        }
        if (!std::isfinite(rc.a_brake_m_s2) || rc.a_brake_m_s2 <= 0.0) {
            throw std::runtime_error("safety.reach_constraint.a_brake_m_s2 must be finite and positive");
        }
        if (!std::isfinite(rc.d_slow_m) || rc.d_slow_m < 0.0) {
            throw std::runtime_error("safety.reach_constraint.d_slow_m must be finite and non-negative");
        }
        if (!cfg.kinematics.enable) {
            throw std::runtime_error(
                "safety.reach_constraint.enable=true requires kinematics.enable=true (TCP FK source)");
        }
        validateTcpOffsetPoints(rc.tcp_offset_points, "safety.reach_constraint.tcp_offset_points");
    }
    if (cfg.safety.user_floor_constraint.enable) {
        const auto& uf = cfg.safety.user_floor_constraint;
        if (!std::isfinite(uf.a_brake_m_s2) || uf.a_brake_m_s2 <= 0.0) {
            throw std::runtime_error("safety.user_floor_constraint.a_brake_m_s2 must be finite and positive");
        }
        if (!std::isfinite(uf.d_slow_m) || uf.d_slow_m < 0.0) {
            throw std::runtime_error("safety.user_floor_constraint.d_slow_m must be finite and non-negative");
        }
        if (!std::isfinite(uf.max_tilt_deg) || uf.max_tilt_deg < 0.0 || uf.max_tilt_deg >= 90.0) {
            throw std::runtime_error("safety.user_floor_constraint.max_tilt_deg must be in [0, 90)");
        }
        if (!std::isfinite(uf.runtime_min_point_z_m) || !std::isfinite(uf.runtime_max_point_z_m) ||
            uf.runtime_min_point_z_m > uf.runtime_max_point_z_m) {
            throw std::runtime_error(
                "safety.user_floor_constraint requires finite runtime_min_point_z_m <= runtime_max_point_z_m");
        }
        if (!std::isfinite(uf.max_margin_m) || uf.max_margin_m < 0.0) {
            throw std::runtime_error("safety.user_floor_constraint.max_margin_m must be finite and non-negative");
        }
        if (!cfg.kinematics.enable) {
            throw std::runtime_error(
                "safety.user_floor_constraint.enable=true requires kinematics.enable=true (TCP FK source)");
        }
        validateTcpOffsetPoints(uf.tcp_offset_points,
                                "safety.user_floor_constraint.tcp_offset_points");
        // A config-provided initial plane must satisfy the SAME envelope the runtime
        // SetUserSafetyFloorPlane command is held to (one validator for both paths).
        if (uf.has_initial_plane) {
            const std::optional<std::string> reject =
                validateUserFloorPlaneRequest(uf.point_m, uf.normal, uf.margin_m, uf);
            if (reject) {
                throw std::runtime_error(
                    "safety.user_floor_constraint initial plane invalid: " + *reject);
            }
        }
    }
    validatePositiveFinite(cfg.servo.filter_dt_min_ratio, "servo.filter_dt_min_ratio");
    if (cfg.servo.output_moving_average_window < 0 || cfg.servo.output_moving_average_window > 5000) {
        throw std::runtime_error("servo.output_moving_average_window must be in [0, 5000]");
    }
    validatePositiveFinite(cfg.servo.filter_dt_max_ratio, "servo.filter_dt_max_ratio");
    validatePositiveFinite(cfg.servo.worker_read_period_sec, "servo.worker_read_period_sec");
    validateNonNegativeFinite(
        cfg.servo.servo_t1_rate_match_tolerance_ratio,
        "servo.servo_t1_rate_match_tolerance_ratio"
    );
    validatePositiveFinite(
        static_cast<double>(cfg.servo.rbpodo_async_streaming.rate_hz),
        "servo.rbpodo_async_streaming.rate_hz"
    );
    validatePositiveFinite(
        cfg.servo.rbpodo_async_streaming.max_pending_age_ms,
        "servo.rbpodo_async_streaming.max_pending_age_ms"
    );
    validatePositiveFinite(
        cfg.servo.rbpodo_async_streaming.ack_supervision.expected_ack_timeout_ms,
        "servo.rbpodo_async_streaming.ack_supervision.expected_ack_timeout_ms"
    );
    validatePositiveFinite(
        cfg.servo.rbpodo_async_streaming.ack_supervision.missing_ack_fault_after_ms,
        "servo.rbpodo_async_streaming.ack_supervision.missing_ack_fault_after_ms"
    );
    if (cfg.servo.rbpodo_async_streaming.ack_supervision.max_consecutive_missing_ack <= 0) {
        throw std::runtime_error(
            "servo.rbpodo_async_streaming.ack_supervision.max_consecutive_missing_ack must be positive"
        );
    }
    validatePositiveFinite(
        cfg.servo.rbpodo_async_streaming.reference_supervision.q_ref_update_timeout_ms,
        "servo.rbpodo_async_streaming.reference_supervision.q_ref_update_timeout_ms"
    );
    validatePositiveFinite(
        cfg.servo.rbpodo_async_streaming.reference_supervision.q_ref_target_tolerance_deg,
        "servo.rbpodo_async_streaming.reference_supervision.q_ref_target_tolerance_deg"
    );
    validatePositiveFinite(
        cfg.servo.rbpodo_async_streaming.reference_supervision.q_ref_target_fault_after_ms,
        "servo.rbpodo_async_streaming.reference_supervision.q_ref_target_fault_after_ms"
    );
    validatePositiveFinite(
        cfg.servo.rbpodo_async_streaming.reference_supervision.tcp_ref_update_timeout_ms,
        "servo.rbpodo_async_streaming.reference_supervision.tcp_ref_update_timeout_ms"
    );
    validatePositiveFinite(
        cfg.servo.rbpodo_async_streaming.reference_supervision.tcp_ref_target_tolerance_m,
        "servo.rbpodo_async_streaming.reference_supervision.tcp_ref_target_tolerance_m"
    );
    validatePositiveFinite(
        cfg.servo.rbpodo_async_streaming.reference_supervision.tcp_ref_target_fault_after_ms,
        "servo.rbpodo_async_streaming.reference_supervision.tcp_ref_target_fault_after_ms"
    );
    validatePositiveFinite(static_cast<double>(cfg.network.state_pub_rate_hz), "network.state_pub_rate_hz");
    validatePositiveFinite(static_cast<double>(cfg.scope.publish_rate_hz), "scope.publish_rate_hz");
    if (cfg.scope.max_samples_per_batch == 0 || cfg.scope.max_samples_per_batch > 1024) {
        throw std::runtime_error("scope.max_samples_per_batch must be in [1, 1024]");
    }
    validatePositiveFinite(cfg.command_source.lease_timeout_sec, "command_source.lease_timeout_sec");
    {
        const auto validate_read_miss_tolerance = [](const BackendConfig& robot, const std::string& name) {
            if (robot.max_consecutive_read_misses < 0) {
                throw std::runtime_error(name + ".max_consecutive_read_misses must be >= 0");
            }
            if (robot.max_consecutive_read_misses > 100) {
                throw std::runtime_error(name + ".max_consecutive_read_misses must be <= 100");
            }
            // Read-miss tolerance holds the last state instead of failing closed on a
            // missing frame; only the rbpodo controller-simulation (pgmode) carve-out
            // may use it. Physical real must fail closed on any read miss.
            if (robot.max_consecutive_read_misses > 0 && !isRbpodoControllerSimulationBackend(robot)) {
                throw std::runtime_error(
                    name + ".max_consecutive_read_misses > 0 requires rbpodo controller simulation "
                    "(run_mode: real, backend_type: rbpodo, operation_mode: simulation)");
            }
        };
        validate_read_miss_tolerance(cfg.left_robot, "left_robot");
        validate_read_miss_tolerance(cfg.right_robot, "right_robot");
    }
    if (cfg.servo.filter_dt_max_ratio < cfg.servo.filter_dt_min_ratio) {
        throw std::runtime_error("servo.filter_dt_max_ratio must be >= filter_dt_min_ratio");
    }
    if (cfg.servo.realtime_priority < 1 || cfg.servo.realtime_priority > 99) {
        throw std::runtime_error("servo.realtime_priority must be in [1, 99]");
    }
    if (cfg.servo.spin_slack_us < 0) {
        throw std::runtime_error("servo.spin_slack_us must be >= 0");
    }
    {
        const int spin_rate_hz = cfg.servo.rate_hz > 0 ? cfg.servo.rate_hz : 500;
        const long long period_us = 1'000'000LL / spin_rate_hz;
        if (cfg.servo.spin_slack_us >= period_us) {
            throw std::runtime_error("servo.spin_slack_us must be smaller than the tick period");
        }
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
    std::set<std::string> seen_state_pub_endpoints;
    for (const std::string& endpoint : cfg.network.state_pub_endpoints) {
        if (!isValidUdpEndpointUri(endpoint)) {
            throw std::runtime_error("network.state_pub_endpoints entries must be udp://host:port endpoints: " + endpoint);
        }
        if (!seen_state_pub_endpoints.insert(endpoint).second) {
            warn("duplicate network.state_pub_endpoints entry configured: " + endpoint);
        }
    }
    if (cfg.network.scope_pub_endpoints.empty()) {
        throw std::runtime_error("network.scope_pub_endpoints must not be empty");
    }
    std::set<std::string> seen_scope_pub_endpoints;
    for (const std::string& endpoint : cfg.network.scope_pub_endpoints) {
        if (!isValidUdpEndpointUri(endpoint)) {
            throw std::runtime_error("network.scope_pub_endpoints entries must be udp://host:port endpoints: " + endpoint);
        }
        if (!seen_scope_pub_endpoints.insert(endpoint).second) {
            warn("duplicate network.scope_pub_endpoints entry configured: " + endpoint);
        }
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
    if (cfg.servo.controller_simulation_treat_unreliable_status_fields_as_unavailable &&
        !cfg.servo.allow_controller_simulation_motion) {
        throw std::runtime_error(
            "servo.controller_simulation_treat_unreliable_status_fields_as_unavailable requires "
            "servo.allow_controller_simulation_motion=true"
        );
    }
    if (cfg.servo.controller_simulation_async_supervision_nonlatching &&
        !cfg.servo.allow_controller_simulation_motion) {
        throw std::runtime_error(
            "servo.controller_simulation_async_supervision_nonlatching requires "
            "servo.allow_controller_simulation_motion=true"
        );
    }
    if (cfg.safety.controller_simulation_tracking_error_nonlatching &&
        !cfg.servo.allow_controller_simulation_motion) {
        throw std::runtime_error(
            "safety.controller_simulation_tracking_error_nonlatching requires "
            "servo.allow_controller_simulation_motion=true"
        );
    }
    if (cfg.servo.allow_controller_simulation_init_error &&
        !cfg.servo.allow_controller_simulation_motion) {
        throw std::runtime_error(
            "servo.allow_controller_simulation_init_error requires "
            "servo.allow_controller_simulation_motion=true"
        );
    }
    if (cfg.servo.allow_controller_simulation_not_activated &&
        !cfg.servo.allow_controller_simulation_motion) {
        throw std::runtime_error(
            "servo.allow_controller_simulation_not_activated requires "
            "servo.allow_controller_simulation_motion=true"
        );
    }
    if (cfg.servo.allow_controller_simulation_motion ||
        cfg.servo.allow_controller_simulation_diagnostics_suspect ||
        cfg.servo.controller_simulation_treat_unreliable_status_fields_as_unavailable ||
        cfg.servo.controller_simulation_async_supervision_nonlatching ||
        cfg.servo.allow_controller_simulation_init_error ||
        cfg.servo.allow_controller_simulation_not_activated) {
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
    }
    const RbpodoAsyncStreamingConfig& async_streaming = cfg.servo.rbpodo_async_streaming;
    if (!async_streaming.enable && async_streaming.mode != RbpodoAsyncStreamingMode::Disabled) {
        throw std::runtime_error(
            "servo.rbpodo_async_streaming.mode must be disabled when "
            "servo.rbpodo_async_streaming.enable=false"
        );
    }
    if (async_streaming.enable) {
        if (async_streaming.mode == RbpodoAsyncStreamingMode::Disabled) {
            throw std::runtime_error(
                "servo.rbpodo_async_streaming.enable=true requires mode sdk_ack_worker "
                "or socket_send_supervised"
            );
        }
        if (!cfg.servo.send_servo_commands) {
            throw std::runtime_error(
                "servo.rbpodo_async_streaming.enable=true requires servo.send_servo_commands=true"
            );
        }
        if (!bothBackendsAreRbpodoControllerSimulation(cfg)) {
            throw std::runtime_error(
                "servo.rbpodo_async_streaming.enable=true requires both rbpodo backends "
                "to use run_mode=real and operation_mode=simulation"
            );
        }
        // Real/sim env gates retired: RB_ALLOW_REAL_ROBOT/MOTION are no longer required.
        if (async_streaming.mode == RbpodoAsyncStreamingMode::SocketSendSupervised &&
            (!cfg.left_robot.disable_waiting_ack || !cfg.right_robot.disable_waiting_ack)) {
            throw std::runtime_error(
                "servo.rbpodo_async_streaming.mode=socket_send_supervised requires "
                "left_robot.disable_waiting_ack=true and right_robot.disable_waiting_ack=true"
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
    const bool force_provider_null =
        force_provider == "null" || force_provider == "none" || force_provider.empty();
    if (!(force_provider_null || force_provider == "project_native")) {
        throw std::runtime_error("force_control.provider must be null or project_native");
    }
    if (cfg.force_control.enable && force_provider != "project_native") {
        throw std::runtime_error(
            "force_control.enable=true requires force_control.provider=project_native"
        );
    }
    const std::string force_mode = lower(cfg.force_control.operating_mode);
    if (!(force_mode == "monitor" || force_mode == "guard" ||
          force_mode == "guarded_admittance" ||
          force_mode == "cartesian_admittance")) {
        throw std::runtime_error(
            "force_control.operating_mode must be monitor, guard, "
            "guarded_admittance, or cartesian_admittance"
        );
    }
    if (!cfg.force_control.enable &&
        (cfg.force_control.left.enable || cfg.force_control.right.enable)) {
        throw std::runtime_error(
            "per-arm force_control enable requires force_control.enable=true"
        );
    }
    if (cfg.force_control.supervised_experimental_real && !cfg.force_control.allow_in_real) {
        throw std::runtime_error(
            "force_control.supervised_experimental_real=true requires allow_in_real=true"
        );
    }
    validatePositiveFinite(
        static_cast<double>(cfg.force_control.update_rate_hz),
        "force_control.update_rate_hz"
    );
    if (cfg.force_control.enable &&
        cfg.force_control.update_rate_hz != cfg.servo.rate_hz) {
        throw std::runtime_error(
            "enabled force_control.update_rate_hz must match servo.rate_hz"
        );
    }
    validatePositiveFiniteArray(cfg.force_control.virtual_mass, "force_control.virtual_mass");
    validateNonNegativeFiniteArray(cfg.force_control.damping, "force_control.damping");
    validateNonNegativeFiniteArray(cfg.force_control.stiffness, "force_control.stiffness");
    validateNonNegativeFiniteArray(
        cfg.force_control.release_bleed_stiffness,
        "force_control.release_bleed_stiffness"
    );
    for (std::size_t i = 0; i < 6; ++i) {
        if (cfg.force_control.release_bleed_stiffness[i] > 0.0 &&
            cfg.force_control.stiffness[i] > 0.0) {
            throw std::runtime_error(
                "force_control.release_bleed_stiffness is only meaningful on "
                "zero-stiffness axes (axis " + std::to_string(i) +
                " has both stiffness and release_bleed_stiffness > 0)"
            );
        }
    }
    validateNonNegativeFiniteArray(
        cfg.force_control.wrench_deadband,
        "force_control.wrench_deadband"
    );
    validateNonNegativeFinite(cfg.force_control.force_limit_n, "force_control.force_limit_n");
    validateNonNegativeFinite(
        cfg.force_control.backoff_gain_m_s_per_n,
        "force_control.backoff_gain_m_s_per_n"
    );
    validateNonNegativeFinite(
        cfg.force_control.backoff_max_velocity_m_s,
        "force_control.backoff_max_velocity_m_s"
    );
    if (cfg.force_control.hard_limit_policy != "latch" &&
        cfg.force_control.hard_limit_policy != "retreat") {
        throw std::runtime_error(
            "force_control.hard_limit_policy must be 'latch' or 'retreat'"
        );
    }
    if (cfg.force_control.hard_limit_policy == "retreat") {
        if (cfg.force_control.enable &&
            cfg.force_control.operating_mode != "cartesian_admittance") {
            throw std::runtime_error(
                "force_control.hard_limit_policy 'retreat' requires "
                "operating_mode cartesian_admittance"
            );
        }
        validatePositiveFinite(
            cfg.force_control.retreat_distance_m,
            "force_control.retreat_distance_m"
        );
        validatePositiveFinite(
            cfg.force_control.retreat_virtual_force_n,
            "force_control.retreat_virtual_force_n"
        );
        validatePositiveFinite(
            cfg.force_control.retreat_timeout_sec,
            "force_control.retreat_timeout_sec"
        );
        if (cfg.force_control.retreat_max_attempts < 0) {
            throw std::runtime_error(
                "force_control.retreat_max_attempts must be non-negative"
            );
        }
        validatePositiveFinite(
            cfg.force_control.retreat_attempt_window_sec,
            "force_control.retreat_attempt_window_sec"
        );
        if (cfg.force_control.retreat_distance_m >
            cfg.force_control.max_pos_offset_m) {
            throw std::runtime_error(
                "force_control.retreat_distance_m must fit inside "
                "max_pos_offset_m (the retreat rides the admittance offset)"
            );
        }
    }
    if (cfg.force_control.force_limit_n > 0.0) {
        if (!(cfg.force_control.backoff_gain_m_s_per_n > 0.0)) {
            throw std::runtime_error(
                "force_control.force_limit_n > 0 requires a positive "
                "backoff_gain_m_s_per_n"
            );
        }
        if (cfg.force_control.backoff_max_velocity_m_s <
            cfg.force_control.max_linear_velocity_m_s) {
            throw std::runtime_error(
                "force_control.backoff_max_velocity_m_s must be >= "
                "max_linear_velocity_m_s when the force limiter is enabled"
            );
        }
    }
    if (cfg.force_control.blockwise_release_recenter) {
        const auto validate_isotropic_recenter_block = [&](std::size_t begin,
                                                            std::size_t end,
                                                            const std::string& name) {
            const double damping_ratio = cfg.force_control.damping[begin] /
                cfg.force_control.virtual_mass[begin];
            const double stiffness_ratio = cfg.force_control.stiffness[begin] /
                cfg.force_control.virtual_mass[begin];
            // The released-state bleed spring drives the same block-coupled
            // recenter motion, so it must obey the same isotropy contract.
            const double bleed_ratio =
                cfg.force_control.release_bleed_stiffness[begin] /
                cfg.force_control.virtual_mass[begin];
            for (std::size_t i = begin + 1; i < end; ++i) {
                const double candidate_damping = cfg.force_control.damping[i] /
                    cfg.force_control.virtual_mass[i];
                const double candidate_stiffness = cfg.force_control.stiffness[i] /
                    cfg.force_control.virtual_mass[i];
                const double candidate_bleed =
                    cfg.force_control.release_bleed_stiffness[i] /
                    cfg.force_control.virtual_mass[i];
                const double damping_scale = std::max({
                    1.0, std::abs(damping_ratio), std::abs(candidate_damping),
                });
                const double stiffness_scale = std::max({
                    1.0, std::abs(stiffness_ratio), std::abs(candidate_stiffness),
                });
                const double bleed_scale = std::max({
                    1.0, std::abs(bleed_ratio), std::abs(candidate_bleed),
                });
                if (std::abs(candidate_damping - damping_ratio) >
                        1e-9 * damping_scale ||
                    std::abs(candidate_stiffness - stiffness_ratio) >
                        1e-9 * stiffness_scale ||
                    std::abs(candidate_bleed - bleed_ratio) >
                        1e-9 * bleed_scale) {
                    throw std::runtime_error(
                        "force_control.blockwise_release_recenter requires equal "
                        "damping/mass, stiffness/mass, and "
                        "release_bleed_stiffness/mass ratios within the " +
                        name + " block"
                    );
                }
            }
        };
        validate_isotropic_recenter_block(0, 3, "translation");
        validate_isotropic_recenter_block(3, 6, "rotation");
    }
    validatePositiveFinite(cfg.force_control.max_dt_sec, "force_control.max_dt_sec");
    validatePositiveFinite(cfg.force_control.max_pos_offset_m, "force_control.max_pos_offset_m");
    validatePositiveFinite(cfg.force_control.max_rot_offset_rad, "force_control.max_rot_offset_rad");
    validatePositiveFinite(
        cfg.force_control.max_linear_velocity_m_s,
        "force_control.max_linear_velocity_m_s"
    );
    validatePositiveFinite(
        cfg.force_control.max_angular_velocity_rad_s,
        "force_control.max_angular_velocity_rad_s"
    );
    validatePositiveFinite(
        cfg.force_control.max_linear_acceleration_m_s2,
        "force_control.max_linear_acceleration_m_s2"
    );
    validatePositiveFinite(
        cfg.force_control.max_angular_acceleration_rad_s2,
        "force_control.max_angular_acceleration_rad_s2"
    );
    validatePositiveFinite(
        cfg.force_control.max_linear_jerk_m_s3,
        "force_control.max_linear_jerk_m_s3"
    );
    validatePositiveFinite(
        cfg.force_control.max_angular_jerk_rad_s3,
        "force_control.max_angular_jerk_rad_s3"
    );
    validatePositiveFinite(cfg.force_control.max_pos_step_m, "force_control.max_pos_step_m");
    validatePositiveFinite(cfg.force_control.max_rot_step_rad, "force_control.max_rot_step_rad");
    validatePositiveFinite(cfg.force_control.max_energy_j, "force_control.max_energy_j");

    const auto& normal = cfg.force_control.normal_admittance;
    validatePositiveFinite(normal.virtual_mass_kg, "force_control.normal_admittance.virtual_mass_kg");
    validateNonNegativeFinite(normal.damping_n_s_m, "force_control.normal_admittance.damping_n_s_m");
    validateNonNegativeFinite(normal.stiffness_n_m, "force_control.normal_admittance.stiffness_n_m");
    validatePositiveFinite(normal.max_unload_offset_m, "force_control.normal_admittance.max_unload_offset_m");
    validatePositiveFinite(normal.max_normal_velocity_m_s, "force_control.normal_admittance.max_normal_velocity_m_s");
    validatePositiveFinite(normal.max_normal_acceleration_m_s2, "force_control.normal_admittance.max_normal_acceleration_m_s2");
    validatePositiveFinite(normal.max_normal_jerk_m_s3, "force_control.normal_admittance.max_normal_jerk_m_s3");
    validatePositiveFinite(normal.max_normal_step_m, "force_control.normal_admittance.max_normal_step_m");
    validatePositiveFinite(normal.max_energy_j, "force_control.normal_admittance.max_energy_j");

    const bool force_motion_affecting =
        cfg.force_control.enable && force_mode != "monitor";
    const auto validate_force_arm = [&](
        const ForceControlArmConfig& arm,
        const FtWrenchPipelineConfig& ft,
        const std::string& path
    ) {
        const std::string surface = lower(arm.surface_source);
        if (!(surface == "floor_constraint" || surface == "user_floor_plane" ||
              surface == "contact_force" || surface == "none")) {
            throw std::runtime_error(
                path + ".surface_source must be floor_constraint, user_floor_plane, "
                "contact_force, or none"
            );
        }
        // surface_source: none is the floorless posture. The server-owned floor
        // and user-plane constraints may be fully disabled; the wrench pipeline
        // falls back to the nominal stand +Z as the hard-normal reference axis
        // (see dual_arm_servo_loop normal_stand). It carries no geometric contact
        // surface, so it is only meaningful for the modes that do NOT regulate a
        // unilateral surface-normal contact: monitor preflight and the symmetric
        // zero-wrench 6D cartesian_admittance. guard / guarded_admittance drive a
        // surface-normal unload and MUST bind an enforcing floor / user plane.
        if (surface == "none" &&
            !(force_mode == "monitor" || force_mode == "cartesian_admittance")) {
            throw std::runtime_error(
                path + ".surface_source=none requires operating_mode monitor or "
                "cartesian_admittance"
            );
        }
        const std::string compliance_frame = lower(arm.compliance_frame);
        if (!(compliance_frame == "surface" ||
              compliance_frame == "sensor_origin" ||
              compliance_frame == "tcp_origin")) {
            throw std::runtime_error(
                path + ".compliance_frame must be surface, sensor_origin, or tcp_origin"
            );
        }
        validateNonNegativeFinite(arm.target_force_n, path + ".target_force_n");
        validateNonNegativeFinite(arm.contact_enter_force_n, path + ".contact_enter_force_n");
        validateNonNegativeFinite(arm.contact_release_force_n, path + ".contact_release_force_n");
        validateNonNegativeFinite(arm.force_deadband_n, path + ".force_deadband_n");
        validateNonNegativeFinite(arm.hard_normal_force_n, path + ".hard_normal_force_n");
        validateNonNegativeFinite(arm.hard_force_norm_n, path + ".hard_force_norm_n");
        validateNonNegativeFinite(arm.hard_torque_norm_nm, path + ".hard_torque_norm_nm");
        validateNonNegativeFinite(
            arm.hard_limit_rate_n_per_ms,
            path + ".hard_limit_rate_n_per_ms"
        );
        validateNonNegativeFinite(
            arm.hard_limit_rate_floor_n,
            path + ".hard_limit_rate_floor_n"
        );
        if (arm.hard_limit_rate_n_per_ms > 0.0 &&
            !(arm.hard_limit_rate_floor_n > 0.0)) {
            throw std::runtime_error(
                path + ".hard_limit_rate_n_per_ms > 0 requires a positive "
                "hard_limit_rate_floor_n"
            );
        }
        if (arm.debounce_samples < 1) {
            throw std::runtime_error(path + ".debounce_samples must be >= 1");
        }
        if (arm.hard_limit_debounce_samples < 1) {
            throw std::runtime_error(
                path + ".hard_limit_debounce_samples must be >= 1"
            );
        }
        validateNonNegativeFinite(arm.release_dwell_sec, path + ".release_dwell_sec");
        validatePositiveFinite(
            arm.release_velocity_threshold_m_s,
            path + ".release_velocity_threshold_m_s"
        );
        validateNonNegativeFinite(
            arm.transverse_contact_enter_force_n,
            path + ".transverse_contact_enter_force_n"
        );
        validateNonNegativeFinite(
            arm.transverse_contact_release_force_n,
            path + ".transverse_contact_release_force_n"
        );
        validateNonNegativeFinite(
            arm.torque_contact_enter_nm,
            path + ".torque_contact_enter_nm"
        );
        validateNonNegativeFinite(
            arm.torque_contact_release_nm,
            path + ".torque_contact_release_nm"
        );
        if (arm.release_velocity_threshold_m_s >
            cfg.force_control.normal_admittance.max_normal_velocity_m_s) {
            throw std::runtime_error(
                path + ".release_velocity_threshold_m_s must be <= "
                "force_control.normal_admittance.max_normal_velocity_m_s"
            );
        }
        if (!arm.enable) return;
        if (!ft.enable) {
            throw std::runtime_error(path + ".enable=true requires matching force_torque arm enable=true");
        }
        if ((compliance_frame == "sensor_origin" ||
             compliance_frame == "tcp_origin") && !ft.frame_configured) {
            throw std::runtime_error(
                path + ".compliance_frame=" + compliance_frame +
                " requires the matching force_torque frame_configured=true"
            );
        }
        if (surface == "contact_force" && !ft.frame_configured) {
            throw std::runtime_error(
                path + ".surface_source=contact_force requires the matching "
                "force_torque frame_configured=true"
            );
        }
        validateNonNegativeFinite(
            arm.contact_entry_max_speed_m_s,
            path + ".contact_entry_max_speed_m_s"
        );
        if (surface == "contact_force" && force_motion_affecting &&
            !(arm.contact_entry_max_speed_m_s > 0.0)) {
            throw std::runtime_error(
                path + ".surface_source=contact_force requires an explicit "
                "positive contact_entry_max_speed_m_s (episode entry gate)"
            );
        }
        if (!(arm.contact_release_force_n < arm.contact_enter_force_n)) {
            throw std::runtime_error(path + " requires contact_release_force_n < contact_enter_force_n");
        }
        if (force_mode == "guarded_admittance" ||
            force_mode == "cartesian_admittance") {
            if (!(arm.target_force_n < arm.contact_release_force_n)) {
                throw std::runtime_error(
                    path + " requires target_force_n < contact_release_force_n "
                    "for guarded_admittance"
                );
            }
            if (arm.target_force_n + arm.force_deadband_n >
                arm.contact_release_force_n) {
                throw std::runtime_error(
                    path + " requires target_force_n + force_deadband_n <= "
                    "contact_release_force_n for guarded_admittance"
                );
            }
        }
        if (force_mode == "cartesian_admittance") {
            if (!(arm.transverse_contact_release_force_n <
                  arm.transverse_contact_enter_force_n)) {
                throw std::runtime_error(
                    path + " requires transverse_contact_release_force_n < "
                    "transverse_contact_enter_force_n"
                );
            }
            if (!(arm.torque_contact_release_nm < arm.torque_contact_enter_nm)) {
                throw std::runtime_error(
                    path + " requires torque_contact_release_nm < torque_contact_enter_nm"
                );
            }
            const ForceControlAxis& axes = arm.compliance_axes;
            if (!(axes.x || axes.y || axes.z || axes.roll || axes.pitch || axes.yaw)) {
                throw std::runtime_error(
                    path + ".compliance_axes must enable at least one axis for "
                    "cartesian_admittance"
                );
            }
        }
        if (arm.target_force_n > arm.contact_enter_force_n) {
            throw std::runtime_error(path + ".target_force_n must be <= contact_enter_force_n");
        }
        if (!(arm.contact_enter_force_n < arm.hard_normal_force_n)) {
            throw std::runtime_error(path + " requires contact_enter_force_n < hard_normal_force_n");
        }
        if (arm.hard_force_norm_n < arm.hard_normal_force_n) {
            throw std::runtime_error(path + ".hard_force_norm_n must be >= hard_normal_force_n");
        }
        if (!(arm.hard_torque_norm_nm > 0.0)) {
            throw std::runtime_error(path + ".hard_torque_norm_nm must be positive");
        }
        if (force_motion_affecting) {
            if (surface == "floor_constraint" &&
                (!cfg.safety.floor_constraint.enable || cfg.safety.floor_constraint.monitor_only)) {
                throw std::runtime_error(
                    path + " requires an enforcing safety.floor_constraint"
                );
            }
            if (surface == "user_floor_plane" &&
                (!cfg.safety.user_floor_constraint.enable ||
                 cfg.safety.user_floor_constraint.monitor_only)) {
                throw std::runtime_error(
                    path + " requires an enforcing safety.user_floor_constraint"
                );
            }
        }
    };

    const auto validate_wrench = [](const Wrench6D& value, const std::string& name) {
        const std::array<double, 6> values{
            value.fx, value.fy, value.fz, value.tx, value.ty, value.tz,
        };
        for (double item : values) {
            if (!std::isfinite(item)) throw std::runtime_error(name + " values must be finite");
        }
    };
    const std::string ft_source = lower(cfg.force_torque.source);
    if (!(ft_source == "null" || ft_source == "none" || ft_source.empty() ||
          ft_source == "rbpodo_eft")) {
        throw std::runtime_error("force_torque.source must be null or rbpodo_eft");
    }
    const auto validate_ft_arm = [&](const FtWrenchPipelineConfig& ft, const std::string& path) {
        if (ft.frame_configured &&
            (ft.sensor_identity.empty() || ft.calibration_id.empty())) {
            throw std::runtime_error(
                path + ".frame_configured=true requires sensor_identity and calibration_id"
            );
        }
        const std::string freshness_source = lower(ft.freshness_source);
        if (!(freshness_source == "sequence" || freshness_source == "source_time")) {
            throw std::runtime_error(path + ".freshness_source must be sequence or source_time");
        }
        validatePositiveFinite(ft.max_sample_age_sec, path + ".max_sample_age_sec");
        validatePositiveFinite(ft.max_source_stall_sec, path + ".max_source_stall_sec");
        if (!std::isfinite(ft.control_lpf_alpha) ||
            ft.control_lpf_alpha < 0.0 || ft.control_lpf_alpha > 1.0) {
            throw std::runtime_error(path + ".control_lpf_alpha must be in [0, 1]");
        }
        validateNonNegativeFinite(
            ft.inertial_effective_mass_kg,
            path + ".inertial_effective_mass_kg"
        );
        if (ft.inertial_compensation_enable) {
            if (!(ft.inertial_effective_mass_kg > 0.0)) {
                throw std::runtime_error(
                    path + ".inertial_compensation_enable=true requires "
                    "inertial_effective_mass_kg > 0"
                );
            }
            if (!std::isfinite(ft.inertial_accel_lpf_alpha) ||
                ft.inertial_accel_lpf_alpha <= 0.0 ||
                ft.inertial_accel_lpf_alpha > 1.0) {
                throw std::runtime_error(
                    path + ".inertial_accel_lpf_alpha must be in (0, 1] when "
                    "inertial compensation is enabled"
                );
            }
        } else if (!std::isfinite(ft.inertial_accel_lpf_alpha) ||
                   ft.inertial_accel_lpf_alpha < 0.0 ||
                   ft.inertial_accel_lpf_alpha > 1.0) {
            throw std::runtime_error(
                path + ".inertial_accel_lpf_alpha must be in [0, 1]"
            );
        }
        validateNonNegativeFinite(ft.max_tcp_speed_m_s, path + ".max_tcp_speed_m_s");
        validateNonNegativeFinite(ft.max_tcp_accel_m_s2, path + ".max_tcp_accel_m_s2");
        validateNonNegativeFinite(ft.auto_tare_settle_sec, path + ".auto_tare_settle_sec");
        if (ft.residual_tare_min_samples < 2) {
            throw std::runtime_error(path + ".residual_tare_min_samples must be >= 2");
        }
        validateNonNegativeFinite(
            ft.residual_tare_max_force_stddev_n,
            path + ".residual_tare_max_force_stddev_n"
        );
        validateNonNegativeFinite(
            ft.residual_tare_max_torque_stddev_nm,
            path + ".residual_tare_max_torque_stddev_nm"
        );
        validateNonNegativeFinite(ft.payload_mass_kg, path + ".payload_mass_kg");
        for (double item : ft.payload_com_tcp_m) {
            if (!std::isfinite(item)) {
                throw std::runtime_error(path + ".payload_com_tcp_m values must be finite");
            }
        }
        if (!(ft.gravity_compensation_model == "rigid_payload" ||
              ft.gravity_compensation_model == "controller_compensated_linear")) {
            throw std::runtime_error(
                path + ".gravity_compensation_model must be rigid_payload or "
                "controller_compensated_linear"
            );
        }
        const auto validate_matrix = [&](const std::array<double, 9>& matrix,
                                         const std::string& matrix_path) {
            for (double item : matrix) {
                if (!std::isfinite(item)) {
                    throw std::runtime_error(matrix_path + " values must be finite");
                }
            }
        };
        validate_matrix(
            ft.gravity_force_matrix_n_per_m_s2,
            path + ".gravity_force_matrix_n_per_m_s2"
        );
        validate_matrix(
            ft.gravity_torque_matrix_nm_per_m_s2,
            path + ".gravity_torque_matrix_nm_per_m_s2"
        );
        if (ft.gravity_compensation_model == "controller_compensated_linear") {
            if (!ft.gravity_force_matrix_configured ||
                !ft.gravity_torque_matrix_configured ||
                ft.gravity_compensation_calibration_id.empty()) {
                throw std::runtime_error(
                    path + ".gravity_compensation_model=controller_compensated_linear "
                    "requires gravity_compensation_calibration_id and both explicit "
                    "gravity matrices"
                );
            }
            if (ft.payload_mass_kg != 0.0 ||
                std::any_of(
                    ft.payload_com_tcp_m.begin(),
                    ft.payload_com_tcp_m.end(),
                    [](double item) { return item != 0.0; }
                )) {
                throw std::runtime_error(
                    path + " cannot combine controller_compensated_linear with a "
                    "rigid payload mass/CoG"
                );
            }
        } else if (ft.gravity_force_matrix_configured ||
                   ft.gravity_torque_matrix_configured ||
                   !ft.gravity_compensation_calibration_id.empty()) {
            throw std::runtime_error(
                path + " gravity matrices/calibration id require "
                "gravity_compensation_model=controller_compensated_linear"
            );
        }
        validate_wrench(ft.sensor_bias, path + ".sensor_bias");
        validate_wrench(ft.residual_tare_tcp, path + ".residual_tare_tcp");
        const Pose6D& sensor_pose = ft.t_tcp_sensor;
        const std::array<double, 6> sensor_pose_values{
            sensor_pose.x, sensor_pose.y, sensor_pose.z,
            sensor_pose.rx, sensor_pose.ry, sensor_pose.rz,
        };
        for (double item : sensor_pose_values) {
            if (!std::isfinite(item)) {
                throw std::runtime_error(path + ".T_tcp_sensor values must be finite");
            }
        }
    };
    validate_ft_arm(cfg.force_torque.left, "force_torque.left");
    validate_ft_arm(cfg.force_torque.right, "force_torque.right");
    validate_force_arm(
        cfg.force_control.left,
        cfg.force_torque.left,
        "force_control.left"
    );
    validate_force_arm(
        cfg.force_control.right,
        cfg.force_torque.right,
        "force_control.right"
    );

    const bool any_ft_enabled = cfg.force_torque.left.enable || cfg.force_torque.right.enable;
    const auto& payload_id = cfg.force_torque.payload_identification;
    if (payload_id.enable) {
        if (!(payload_id.observation_model == "rigid_payload" ||
              payload_id.observation_model == "controller_compensated_linear")) {
            throw std::runtime_error(
                "force_torque.payload_identification.observation_model must be "
                "rigid_payload or controller_compensated_linear"
            );
        }
        if (payload_id.observation_model == "rigid_payload") {
            if (!(payload_id.wrench_convention == "payload_load" ||
                  payload_id.wrench_convention == "sensor_reaction")) {
                throw std::runtime_error(
                    "force_torque.payload_identification.wrench_convention must be "
                    "payload_load or sensor_reaction for rigid_payload"
                );
            }
        } else if (!payload_id.wrench_convention.empty()) {
            throw std::runtime_error(
                "force_torque.payload_identification.wrench_convention must be omitted "
                "for controller_compensated_linear"
            );
        }
        if (payload_id.min_poses < 5) {
            throw std::runtime_error(
                "force_torque.payload_identification.min_poses must be >= 5"
            );
        }
        validatePositiveFinite(
            payload_id.arrival_tolerance_deg,
            "force_torque.payload_identification.arrival_tolerance_deg"
        );
        validatePositiveFinite(
            payload_id.settle_sec,
            "force_torque.payload_identification.settle_sec"
        );
        if (payload_id.samples_per_pose < 2) {
            throw std::runtime_error(
                "force_torque.payload_identification.samples_per_pose must be >= 2"
            );
        }
        validatePositiveFinite(
            payload_id.max_force_stddev_n,
            "force_torque.payload_identification.max_force_stddev_n"
        );
        validatePositiveFinite(
            payload_id.max_torque_stddev_nm,
            "force_torque.payload_identification.max_torque_stddev_nm"
        );
        validatePositiveFinite(
            payload_id.max_force_fit_rms_n,
            "force_torque.payload_identification.max_force_fit_rms_n"
        );
        validatePositiveFinite(
            payload_id.max_torque_fit_rms_nm,
            "force_torque.payload_identification.max_torque_fit_rms_nm"
        );
        if (!std::isfinite(payload_id.max_design_condition_number) ||
            payload_id.max_design_condition_number <= 1.0) {
            throw std::runtime_error(
                "force_torque.payload_identification.max_design_condition_number "
                "must be finite and > 1"
            );
        }
        if (!any_ft_enabled || ft_source != "rbpodo_eft") {
            throw std::runtime_error(
                "force_torque.payload_identification.enable=true requires an enabled "
                "rbpodo_eft force_torque arm"
            );
        }
    }
    const bool any_auto_tare =
        (cfg.force_torque.left.enable &&
         cfg.force_torque.left.auto_tare_after_init_motion) ||
        (cfg.force_torque.right.enable &&
         cfg.force_torque.right.auto_tare_after_init_motion);
    if (any_ft_enabled && ft_source != "rbpodo_eft") {
        throw std::runtime_error(
            "enabled force_torque arms require force_torque.source=rbpodo_eft"
        );
    }
    if (any_auto_tare && !cfg.safety.init_motion_planner.enable) {
        throw std::runtime_error(
            "force_torque auto_tare_after_init_motion requires "
            "safety.init_motion_planner.enable=true so completion is measured"
        );
    }
    if (force_motion_affecting) {
        if (!cfg.kinematics.enable) {
            throw std::runtime_error("motion-affecting force control requires kinematics.enable=true");
        }
        if (cfg.servo.send_at_tick_start) {
            throw std::runtime_error(
                "motion-affecting force control requires servo.send_at_tick_start=false"
            );
        }
        if (!cfg.force_control.left.enable && !cfg.force_control.right.enable) {
            throw std::runtime_error(
                "motion-affecting force control requires at least one enabled arm"
            );
        }
    }

    const auto physical_real = [](const BackendConfig& backend) {
        return backend.run_mode == RunMode::Real &&
            lower(backend.operation_mode) == "real";
    };
    if (force_motion_affecting &&
        (physical_real(cfg.left_robot) || physical_real(cfg.right_robot)) &&
        (!cfg.force_control.allow_in_real ||
         !cfg.force_control.supervised_experimental_real)) {
        throw std::runtime_error(
            "real force control requires allow_in_real=true and "
            "supervised_experimental_real=true"
        );
    }

    // Real/sim env gates retired: real Cartesian control no longer requires
    // RB_ALLOW_REAL_CARTESIAN.
    if (anyReal(cfg) && cfg.cartesian_control.allow_in_controller_simulation) {
        if (!cfg.cartesian_control.enable) {
            throw std::runtime_error(
                "cartesian_control.allow_in_controller_simulation requires cartesian_control.enable=true"
            );
        }
        if (!cfg.servo.allow_controller_simulation_motion) {
            throw std::runtime_error(
                "cartesian_control.allow_in_controller_simulation requires "
                "servo.allow_controller_simulation_motion=true"
            );
        }
        if (!bothBackendsAreRbpodoControllerSimulation(cfg)) {
            throw std::runtime_error(
                "cartesian_control.allow_in_controller_simulation requires both rbpodo backends "
                "to use run_mode=real and operation_mode=simulation"
            );
        }
        // Real/sim env gates retired: RB_ALLOW_REAL_ROBOT/MOTION are no longer required.
    }
    validateNonNegativeFinite(cfg.cartesian_control.warn_ik_duration_us, "cartesian_control.warn_ik_duration_us");
    validateNonNegativeFinite(cfg.cartesian_control.fail_ik_duration_us, "cartesian_control.fail_ik_duration_us");
    validatePositiveFinite(cfg.cartesian_control.path_kp, "cartesian_control.path_kp");
    validatePositiveFinite(cfg.cartesian_control.path_kp_pos, "cartesian_control.path_kp_pos");
    validatePositiveFinite(cfg.cartesian_control.path_kp_ori, "cartesian_control.path_kp_ori");
    validatePositiveFinite(cfg.cartesian_control.velocity_damping, "cartesian_control.velocity_damping");
    validatePositiveFinite(cfg.cartesian_control.max_linear_move_speed_m_s, "cartesian_control.max_linear_move_speed_m_s");
    validatePositiveFinite(cfg.cartesian_control.max_angular_move_speed_rad_s, "cartesian_control.max_angular_move_speed_rad_s");
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
    const auto validate_pose_track_smd = [](const PoseTrackSmdConfig& smd, const std::string& path) {
        validatePositiveFinite(smd.damping_ratio_linear, path + ".damping_ratio_linear");
        validatePositiveFinite(smd.natural_frequency_linear_hz, path + ".natural_frequency_linear_hz");
        validatePositiveFinite(smd.damping_ratio_angular, path + ".damping_ratio_angular");
        validatePositiveFinite(smd.natural_frequency_angular_hz, path + ".natural_frequency_angular_hz");
        for (const auto& [value, name] : {
                 std::pair<double, const char*>{smd.max_linear_velocity_m_s, ".max_linear_velocity_m_s"},
                 {smd.max_linear_accel_m_s2, ".max_linear_accel_m_s2"},
                 {smd.max_angular_velocity_rad_s, ".max_angular_velocity_rad_s"},
                 {smd.max_angular_accel_rad_s2, ".max_angular_accel_rad_s2"},
             }) {
            if (!std::isfinite(value) || value < 0.0) {
                throw std::runtime_error(path + name + " must be finite and >= 0 (0 = unlimited)");
            }
        }
    };
    validate_pose_track_smd(cfg.cartesian_control.pose_track_smd, "cartesian_control.pose_track_smd");
    const auto validate_ruckig_follower = [&cfg](const RuckigFollowerConfig& rf, const std::string& path) {
        validatePositiveFinite(rf.max_linear_velocity_m_s, path + ".max_linear_velocity_m_s");
        validatePositiveFinite(rf.max_linear_accel_m_s2, path + ".max_linear_accel_m_s2");
        validatePositiveFinite(rf.max_linear_jerk_m_s3, path + ".max_linear_jerk_m_s3");
        validatePositiveFinite(rf.max_angular_velocity_rad_s, path + ".max_angular_velocity_rad_s");
        validatePositiveFinite(rf.max_angular_accel_rad_s2, path + ".max_angular_accel_rad_s2");
        validatePositiveFinite(rf.max_angular_jerk_rad_s3, path + ".max_angular_jerk_rad_s3");
        validatePositiveFinite(rf.chunk_feed_timeout_sec, path + ".chunk_feed_timeout_sec");
        validatePositiveFinite(rf.engage_timeout_sec, path + ".engage_timeout_sec");
        if (rf.enable) {
            validatePositiveFinite(
                rf.hold_bounce_resume_sec,
                path + ".hold_bounce_resume_sec"
            );
        }
        if (rf.discard_head_steps < 0) {
            throw std::runtime_error(path + ".discard_head_steps must be >= 0");
        }
        if (rf.consume_steps < 1) {
            throw std::runtime_error(path + ".consume_steps must be >= 1");
        }
        if (rf.reserve_steps < 1) {
            throw std::runtime_error(path + ".reserve_steps must be >= 1 (central difference needs a forward neighbor)");
        }
        if (rf.smoothing_window < 1 || rf.smoothing_window % 2 == 0) {
            throw std::runtime_error(path + ".smoothing_window must be an odd integer >= 1");
        }
        if (!std::isfinite(rf.af_damping_beta_lin) || rf.af_damping_beta_lin <= 0.0 ||
            rf.af_damping_beta_lin > 1.0) {
            throw std::runtime_error(path +
                                     ".af_damping_beta_lin must be in (0, 1] (also set by the "
                                     "legacy af_damping_beta key)");
        }
        if (!std::isfinite(rf.af_damping_beta_ang) || rf.af_damping_beta_ang <= 0.0 ||
            rf.af_damping_beta_ang > 1.0) {
            throw std::runtime_error(path +
                                     ".af_damping_beta_ang must be in (0, 1] (also set by the "
                                     "legacy af_damping_beta key)");
        }
        // Corner guard. Deadbands are >= 0 (0 = every non-zero step is signed, so the
        // guard is maximally sensitive); a larger deadband ignores more wobble. The
        // velocity ring-down lives in [0, 1]: 0 brakes to a standstill at every
        // reversal, 1 keeps only the acceleration reset. Above 1 would ACCELERATE into
        // a reversal, so it is rejected rather than clamped.
        if (!std::isfinite(rf.corner_deadband_lin_m) || rf.corner_deadband_lin_m < 0.0) {
            throw std::runtime_error(path + ".corner_deadband_lin_m must be finite and >= 0");
        }
        if (!std::isfinite(rf.corner_deadband_ang_rad) || rf.corner_deadband_ang_rad < 0.0) {
            throw std::runtime_error(path + ".corner_deadband_ang_rad must be finite and >= 0");
        }
        if (!std::isfinite(rf.corner_velocity_scale) ||
            rf.corner_velocity_scale < 0.0 || rf.corner_velocity_scale > 1.0) {
            throw std::runtime_error(path + ".corner_velocity_scale must be in [0, 1]");
        }
        // Quasi-static gate for the wrench-gated loading projection. The gate is a
        // `plan_accel <= bound` comparison (cartesian_chunk_follower.cpp), so NaN or a
        // negative bound makes it unsatisfiable and silently stands the contact assist
        // down for the whole run while the config still declares it. Fail closed at load
        // instead: a contact-bounding value must come from its authoritative source.
        validatePositiveFinite(
            rf.loading_projection_max_accel_m_s2,
            path + ".loading_projection_max_accel_m_s2");
        validatePositiveFinite(rf.delta_twist_tau_sec, path + ".delta_twist_tau_sec");
        if (rf.delta_twist_residual_drain_steps < 1) {
            throw std::runtime_error(path + ".delta_twist_residual_drain_steps must be >= 1");
        }
        validatePositiveFinite(rf.delta_twist_min_time_to_go_sec, path + ".delta_twist_min_time_to_go_sec");
        validatePositiveFinite(rf.delta_twist_max_residual_m, path + ".delta_twist_max_residual_m");
        validatePositiveFinite(rf.delta_twist_max_residual_rad, path + ".delta_twist_max_residual_rad");
        validatePositiveFinite(rf.delta_twist_max_lead_m, path + ".delta_twist_max_lead_m");
        validatePositiveFinite(rf.delta_twist_max_lead_rad, path + ".delta_twist_max_lead_rad");
        validatePositiveFinite(
            rf.delta_twist_stale_residual_timeout_sec,
            path + ".delta_twist_stale_residual_timeout_sec"
        );
        if (rf.controller == RuckigFollowerController::DeltaPreview) {
            validatePositiveFinite(
                rf.preview_max_projection_error_m,
                path + ".preview_max_projection_error_m");
            validatePositiveFinite(
                rf.preview_max_projection_error_rad,
                path + ".preview_max_projection_error_rad");
            validatePositiveFinite(
                rf.preview_max_actual_lead_m,
                path + ".preview_max_actual_lead_m");
            validatePositiveFinite(
                rf.preview_max_actual_lead_rad,
                path + ".preview_max_actual_lead_rad");
            if (rf.preview_max_consecutive_projection_errors < 1) {
                throw std::runtime_error(
                    path + ".preview_max_consecutive_projection_errors must be >= 1");
            }
            if (rf.preview_max_consecutive_actual_lead_errors < 1) {
                throw std::runtime_error(
                    path + ".preview_max_consecutive_actual_lead_errors must be >= 1");
            }
            if (rf.fallback_policy != RuckigFollowerFallbackPolicy::Fault) {
                throw std::runtime_error(
                    path + ".controller=delta_preview requires fallback_policy=fault");
            }
        }
        if (rf.enable && cfg.network.chunk_frame_bind.empty()) {
            throw std::runtime_error(
                path + ".enable=true requires network.chunk_frame_bind (dedicated chunk-frame UDP ingest)"
            );
        }
    };
    validate_ruckig_follower(cfg.cartesian_control.ruckig_follower, "cartesian_control.ruckig_follower");
    if (cfg.cartesian_control.tcp_pose_target_profile_default.empty()) {
        throw std::runtime_error("cartesian_control.tcp_pose_target_profile_default must not be empty");
    }
    if (cfg.cartesian_control.tcp_pose_target_profiles.empty()) {
        throw std::runtime_error("cartesian_control.tcp_pose_target_profiles must not be empty");
    }
    std::set<std::string> tcp_profile_names;
    bool has_default_tcp_profile = false;
    for (const TcpPoseTargetProfileConfig& profile : cfg.cartesian_control.tcp_pose_target_profiles) {
        if (profile.name.empty()) {
            throw std::runtime_error("cartesian_control.tcp_pose_target_profiles contains an empty profile name");
        }
        if (!tcp_profile_names.insert(profile.name).second) {
            throw std::runtime_error("duplicate cartesian_control.tcp_pose_target_profiles entry: " + profile.name);
        }
        validate_pose_track_smd(
            profile.pose_track_smd,
            "cartesian_control.tcp_pose_target_profiles." + profile.name + ".pose_track_smd"
        );
        validate_ruckig_follower(
            profile.ruckig_follower,
            "cartesian_control.tcp_pose_target_profiles." + profile.name + ".ruckig_follower"
        );
        validateNonNegativeFinite(
            profile.max_smd_goal_lead_m,
            "cartesian_control.tcp_pose_target_profiles." + profile.name + ".max_smd_goal_lead_m"
        );
        validateNonNegativeFinite(
            profile.max_smd_goal_lead_rad,
            "cartesian_control.tcp_pose_target_profiles." + profile.name + ".max_smd_goal_lead_rad"
        );
        has_default_tcp_profile = has_default_tcp_profile ||
            profile.name == cfg.cartesian_control.tcp_pose_target_profile_default;
    }
    if (!has_default_tcp_profile) {
        throw std::runtime_error(
            "cartesian_control.tcp_pose_target_profile_default must name an entry in tcp_pose_target_profiles"
        );
    }

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
        warnIfRbpodoSafetyRangeDiffersFromKnownUrdf(cfg);
    }
    if (cfg.kinematics.ik.max_iterations <= 0) {
        throw std::runtime_error("kinematics.ik.max_iterations must be positive");
    }
    validatePositiveFinite(cfg.kinematics.ik.timeout_ms, "kinematics.ik.timeout_ms");
    validatePositiveFinite(cfg.kinematics.ik.damping, "kinematics.ik.damping");
    validatePositiveFinite(cfg.kinematics.ik.position_tolerance_m, "kinematics.ik.position_tolerance_m");
    validatePositiveFinite(cfg.kinematics.ik.orientation_tolerance_rad, "kinematics.ik.orientation_tolerance_rad");
    validatePositiveFiniteArray(cfg.kinematics.ik.max_step_deg, "kinematics.ik.max_step_deg");
    validateNonNegativeFinite(cfg.kinematics.ik.singular_region_eps, "kinematics.ik.singular_region_eps");
    validateNonNegativeFinite(cfg.kinematics.ik.damping_max, "kinematics.ik.damping_max");
    validateNonNegativeFinite(cfg.kinematics.ik.max_solution_jump_deg, "kinematics.ik.max_solution_jump_deg");
    validateNonNegativeFinite(cfg.kinematics.ik.branch_jump_damping_scale, "kinematics.ik.branch_jump_damping_scale");
    if (cfg.kinematics.ik.branch_jump_max_retries < 0) {
        throw std::runtime_error("kinematics.ik.branch_jump_max_retries must be >= 0");
    }

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
        // RB_ALLOW_RBPODO_SERVO_PARAM_UNSAFE env gate retired: servo params only
        // need to be positive and finite; out-of-vendor-range values get a WARN.
        validatePositiveFinite(backend.servo_t2_sec, label + ".servo_t2_sec");
        validatePositiveFinite(backend.servo_gain, label + ".servo_gain");
        validatePositiveFinite(backend.servo_alpha, label + ".servo_alpha");
        // Rainbow scales move_servo_j gain/alpha by 0.1 INSIDE the controller
        // (vendor-confirmed): the script-level value we send is 10x the
        // effective value. So the effective vendor range 0 < alpha <= 1 maps to
        // a script-level range 0 < servo_alpha <= 10. servo_alpha=10 is the
        // LPF-off diagnostic value; the tracked real profile uses 1.0 to retain
        // filtering after LPF-off motion showed jerk/jitter on hardware.
        const bool out_of_vendor_range =
            !(backend.servo_t2_sec > 0.02 && backend.servo_t2_sec < 0.2) ||
            !(backend.servo_alpha > 0.0 && backend.servo_alpha <= 10.0);
        if (out_of_vendor_range) {
            warn(
                label + ": servo params outside the vendor-recommended range accepted: "
                "servo_t2_sec=" + std::to_string(backend.servo_t2_sec) +
                ", servo_gain=" + std::to_string(backend.servo_gain) +
                ", servo_alpha=" + std::to_string(backend.servo_alpha)
            );
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
        // Real/sim env gates retired: disable_waiting_ack no longer requires
        // RB_ALLOW_RBPODO_ACK_DISABLED_MOTION (ACK-off acceptance is config-driven).
    };
    validate_rbpodo_backend(cfg.left_robot, "left_robot");
    validate_rbpodo_backend(cfg.right_robot, "right_robot");

    if (anyReal(cfg)) {
        if (cfg.servo.io_model == ServoIoModel::Worker) {
            throw std::runtime_error("Refusing servo.io_model=worker in real mode until worker I/O has real-hardware acceptance.");
        }
        // Real/sim env gates retired: real mode/motion no longer require
        // RB_ALLOW_REAL_ROBOT/RB_ALLOW_REAL_MOTION.
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
        if (cfg.scope.enable) {
            for (const std::string& endpoint : cfg.network.scope_pub_endpoints) {
                exposed_network = exposed_network || bindRequiresExposureOverride(endpoint);
            }
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
        "scope",
        "force_torque",
        "force_control",
        "cartesian_control",
        "kinematics",
        "gripper",
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
    cfg.safety.q_min_deg = rbpodoDefaultSafetyJointMinDeg();
    cfg.safety.q_max_deg = rbpodoDefaultSafetyJointMaxDeg();
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
            "send_at_tick_start",
            "allow_readonly_faulted_startup",
            "allow_readonly_q_range_violation_startup",
            "allow_readonly_wrong_mode_startup",
            "allow_controller_simulation_motion",
            "allow_controller_simulation_diagnostics_suspect",
            "controller_simulation_treat_unreliable_status_fields_as_unavailable",
            "controller_simulation_demote_self_collision_fault",
            "allow_real_motion_with_suspect_diagnostics",
            "controller_simulation_async_supervision_nonlatching",
            "allow_controller_simulation_init_error",
            "allow_controller_simulation_not_activated",
            "allow_freedrive",
            "enable_realtime_priority",
            "realtime_priority",
            "cpu_core",
            "spin_slack_us",
            "worker_read_period_sec",
            "worker_read_rate_hz",
            "filter_dt_min_ratio",
            "filter_dt_max_ratio",
            "output_moving_average_window",
            "servo_t1_rate_match_tolerance_ratio",
            "allow_servo_t1_rate_mismatch",
            "rbpodo_async_streaming",
        }, "servo");
        if (has(sec, "rate_hz")) cfg.servo.rate_hz = asInt(sec["rate_hz"], "servo.rate_hz");
        if (has(sec, "command_timeout_sec")) cfg.servo.command_timeout_sec = asDouble(sec["command_timeout_sec"], "servo.command_timeout_sec");
        if (has(sec, "io_model")) cfg.servo.io_model = parseServoIoModel(sec["io_model"], "servo.io_model");
        if (has(sec, "startup_mode")) cfg.servo.startup_mode = controlModeFromString(asString(sec["startup_mode"], "servo.startup_mode"));
        if (has(sec, "send_servo_commands")) cfg.servo.send_servo_commands = asBool(sec["send_servo_commands"], "servo.send_servo_commands");
        if (has(sec, "send_at_tick_start")) cfg.servo.send_at_tick_start = asBool(sec["send_at_tick_start"], "servo.send_at_tick_start");
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
        if (has(sec, "controller_simulation_treat_unreliable_status_fields_as_unavailable")) {
            cfg.servo.controller_simulation_treat_unreliable_status_fields_as_unavailable =
                asBool(
                    sec["controller_simulation_treat_unreliable_status_fields_as_unavailable"],
                    "servo.controller_simulation_treat_unreliable_status_fields_as_unavailable"
                );
        }
        if (has(sec, "controller_simulation_demote_self_collision_fault")) {
            cfg.servo.controller_simulation_demote_self_collision_fault =
                asBool(
                    sec["controller_simulation_demote_self_collision_fault"],
                    "servo.controller_simulation_demote_self_collision_fault"
                );
        }
        if (has(sec, "allow_real_motion_with_suspect_diagnostics")) {
            cfg.servo.allow_real_motion_with_suspect_diagnostics =
                asBool(
                    sec["allow_real_motion_with_suspect_diagnostics"],
                    "servo.allow_real_motion_with_suspect_diagnostics"
                );
        }
        if (has(sec, "controller_simulation_async_supervision_nonlatching")) {
            cfg.servo.controller_simulation_async_supervision_nonlatching =
                asBool(
                    sec["controller_simulation_async_supervision_nonlatching"],
                    "servo.controller_simulation_async_supervision_nonlatching"
                );
        }
        if (has(sec, "allow_controller_simulation_init_error")) {
            cfg.servo.allow_controller_simulation_init_error =
                asBool(
                    sec["allow_controller_simulation_init_error"],
                    "servo.allow_controller_simulation_init_error"
                );
        }
        if (has(sec, "allow_controller_simulation_not_activated")) {
            cfg.servo.allow_controller_simulation_not_activated =
                asBool(
                    sec["allow_controller_simulation_not_activated"],
                    "servo.allow_controller_simulation_not_activated"
                );
        }
        if (has(sec, "allow_freedrive")) {
            cfg.servo.allow_freedrive =
                asBool(sec["allow_freedrive"], "servo.allow_freedrive");
        }
        if (has(sec, "enable_realtime_priority")) cfg.servo.enable_realtime_priority = asBool(sec["enable_realtime_priority"], "servo.enable_realtime_priority");
        if (has(sec, "realtime_priority")) cfg.servo.realtime_priority = asInt(sec["realtime_priority"], "servo.realtime_priority");
        if (has(sec, "cpu_core")) cfg.servo.cpu_core = asInt(sec["cpu_core"], "servo.cpu_core");
        if (has(sec, "spin_slack_us")) cfg.servo.spin_slack_us = asInt(sec["spin_slack_us"], "servo.spin_slack_us");
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
        if (has(sec, "output_moving_average_window")) {
            cfg.servo.output_moving_average_window =
                asInt(sec["output_moving_average_window"], "servo.output_moving_average_window");
        }
        if (has(sec, "filter_dt_max_ratio")) cfg.servo.filter_dt_max_ratio = asDouble(sec["filter_dt_max_ratio"], "servo.filter_dt_max_ratio");
        if (has(sec, "servo_t1_rate_match_tolerance_ratio")) {
            cfg.servo.servo_t1_rate_match_tolerance_ratio =
                asDouble(sec["servo_t1_rate_match_tolerance_ratio"], "servo.servo_t1_rate_match_tolerance_ratio");
        }
        if (has(sec, "allow_servo_t1_rate_mismatch")) {
            cfg.servo.allow_servo_t1_rate_mismatch =
                asBool(sec["allow_servo_t1_rate_mismatch"], "servo.allow_servo_t1_rate_mismatch");
        }
        if (has(sec, "rbpodo_async_streaming")) {
            applyRbpodoAsyncStreamingSection(
                sec["rbpodo_async_streaming"],
                &cfg.servo.rbpodo_async_streaming,
                "servo.rbpodo_async_streaming"
            );
        }
    }
    cfg.left_robot.allow_controller_simulation_diagnostics_suspect =
        cfg.servo.allow_controller_simulation_diagnostics_suspect;
    cfg.right_robot.allow_controller_simulation_diagnostics_suspect =
        cfg.servo.allow_controller_simulation_diagnostics_suspect;
    cfg.left_robot.controller_simulation_treat_unreliable_status_fields_as_unavailable =
        cfg.servo.controller_simulation_treat_unreliable_status_fields_as_unavailable;
    cfg.right_robot.controller_simulation_treat_unreliable_status_fields_as_unavailable =
        cfg.servo.controller_simulation_treat_unreliable_status_fields_as_unavailable;
    cfg.left_robot.controller_simulation_demote_self_collision_fault =
        cfg.servo.controller_simulation_demote_self_collision_fault;
    cfg.right_robot.controller_simulation_demote_self_collision_fault =
        cfg.servo.controller_simulation_demote_self_collision_fault;
    cfg.left_robot.allow_real_motion_with_suspect_diagnostics =
        cfg.servo.allow_real_motion_with_suspect_diagnostics;
    cfg.right_robot.allow_real_motion_with_suspect_diagnostics =
        cfg.servo.allow_real_motion_with_suspect_diagnostics;
    cfg.left_robot.allow_controller_simulation_init_error =
        cfg.servo.allow_controller_simulation_init_error;
    cfg.right_robot.allow_controller_simulation_init_error =
        cfg.servo.allow_controller_simulation_init_error;

    if (has(root, "safety")) {
        const YAML::Node sec = root["safety"];
        validateAllowedKeys(sec, {
            "q_min_deg",
            "q_max_deg",
            "dq_max_deg_s",
            "ddq_max_deg_s2",
            "joint_wrap_period_deg",
            "joint_target_literal_axes",
            "command_timeout_sec",
            "max_tracking_error_deg",
            "stop_both_arms_on_single_arm_error",
            "tracking_error_policy",
            "latch_fault_on_robot_state_error",
            "joint_wrap_for_startup_validation",
            "joint_wrap_for_motion_safety",
            "controller_simulation_tracking_error_source",
            "controller_simulation_physical_motion_policy",
            "controller_simulation_physical_motion_threshold_deg",
            "controller_simulation_tracking_error_nonlatching",
            "self_collision",
            "floor_constraint",
            "roi_box",
            "reach_constraint",
            "user_floor_constraint",
            "joint_target_smd",
            "init_motion_planner",
        }, "safety");
        if (has(sec, "q_min_deg")) cfg.safety.q_min_deg = parseJointArray(sec["q_min_deg"], "safety.q_min_deg");
        if (has(sec, "q_max_deg")) cfg.safety.q_max_deg = parseJointArray(sec["q_max_deg"], "safety.q_max_deg");
        if (has(sec, "dq_max_deg_s")) cfg.safety.dq_max_deg_s = parseJointArray(sec["dq_max_deg_s"], "safety.dq_max_deg_s");
        if (has(sec, "ddq_max_deg_s2")) cfg.safety.ddq_max_deg_s2 = parseJointArray(sec["ddq_max_deg_s2"], "safety.ddq_max_deg_s2");
        if (has(sec, "joint_wrap_period_deg")) cfg.safety.joint_wrap_period_deg = parseJointArray(sec["joint_wrap_period_deg"], "safety.joint_wrap_period_deg");
        if (has(sec, "joint_target_literal_axes")) cfg.safety.joint_target_literal_axes = parseJointBoolArray(sec["joint_target_literal_axes"], "safety.joint_target_literal_axes");
        if (has(sec, "command_timeout_sec")) cfg.safety.command_timeout_sec = asDouble(sec["command_timeout_sec"], "safety.command_timeout_sec");
        if (has(sec, "max_tracking_error_deg")) cfg.safety.max_tracking_error_deg = asDouble(sec["max_tracking_error_deg"], "safety.max_tracking_error_deg");
        if (has(sec, "stop_both_arms_on_single_arm_error")) cfg.safety.stop_both_arms_on_single_arm_error = asBool(sec["stop_both_arms_on_single_arm_error"], "safety.stop_both_arms_on_single_arm_error");
        if (has(sec, "tracking_error_policy")) cfg.safety.tracking_error_policy = trackingErrorPolicyFromString(asString(sec["tracking_error_policy"], "safety.tracking_error_policy"));
        if (has(sec, "latch_fault_on_robot_state_error")) cfg.safety.latch_fault_on_robot_state_error = asBool(sec["latch_fault_on_robot_state_error"], "safety.latch_fault_on_robot_state_error");
        if (has(sec, "joint_wrap_for_startup_validation")) cfg.safety.joint_wrap_for_startup_validation = asBool(sec["joint_wrap_for_startup_validation"], "safety.joint_wrap_for_startup_validation");
        if (has(sec, "joint_wrap_for_motion_safety")) cfg.safety.joint_wrap_for_motion_safety = asBool(sec["joint_wrap_for_motion_safety"], "safety.joint_wrap_for_motion_safety");
        if (has(sec, "controller_simulation_tracking_error_source")) {
            cfg.safety.controller_simulation_tracking_error_source =
                parseControllerSimulationTrackingErrorSource(
                    sec["controller_simulation_tracking_error_source"],
                    "safety.controller_simulation_tracking_error_source"
                );
        }
        if (has(sec, "controller_simulation_physical_motion_policy")) {
            cfg.safety.controller_simulation_physical_motion_policy =
                parseControllerSimulationPhysicalMotionPolicy(
                    sec["controller_simulation_physical_motion_policy"],
                    "safety.controller_simulation_physical_motion_policy"
                );
        }
        if (has(sec, "controller_simulation_physical_motion_threshold_deg")) {
            cfg.safety.controller_simulation_physical_motion_threshold_deg = asDouble(
                sec["controller_simulation_physical_motion_threshold_deg"],
                "safety.controller_simulation_physical_motion_threshold_deg"
            );
        }
        if (has(sec, "controller_simulation_tracking_error_nonlatching")) {
            cfg.safety.controller_simulation_tracking_error_nonlatching = asBool(
                sec["controller_simulation_tracking_error_nonlatching"],
                "safety.controller_simulation_tracking_error_nonlatching"
            );
        }
        if (has(sec, "self_collision")) {
            const YAML::Node sc = sec["self_collision"];
            validateAllowedKeys(sc, {
                "enable",
                "fail_policy",
                "monitor_only",
                "mesh",
            }, "safety.self_collision");
            if (has(sc, "enable")) {
                cfg.safety.self_collision.enable = asBool(sc["enable"], "safety.self_collision.enable");
            }
            if (has(sc, "monitor_only")) {
                cfg.safety.self_collision.monitor_only =
                    asBool(sc["monitor_only"], "safety.self_collision.monitor_only");
            }
            if (has(sc, "fail_policy")) {
                cfg.safety.self_collision.fail_policy =
                    parseSelfCollisionFailPolicy(sc["fail_policy"], "safety.self_collision.fail_policy");
            }
            if (has(sc, "mesh")) {
                const YAML::Node m = sc["mesh"];
                validateAllowedKeys(m, {
                    "unified_urdf",
                    "package_dirs",
                    "pika_gripper_mesh",
                    "pika_gripper_base_mesh",
                    "pika_finger_left_mesh",
                    "pika_finger_right_mesh",
                    "gripper_finger_travel_m",
                    "left_prefix",
                    "right_prefix",
                    "stand_frame",
                    "stand_ignore_arm_substrings",
                    "left_arm_root_frame",
                    "right_arm_root_frame",
                    "check_intra_arm",
                    "intra_arm_min_chain_separation",
                    "disabled_collision_pairs",
                    "debug_pair_curation",
                    "swept_samples",
                    "d_hard_m",
                    "d_slow_m",
                    "a_brake_m_s2",
                    "hyst_m",
                    "projection_iterations",
                    "recover_speed_m_s",
                    "latency_s",
                    "max_staleness_s",
                    "monitor_core",
                    "max_near_pairs",
                    "viz_near_pairs_m",
                    "extra_collision",
                    "ground_plane",
                    "external",
                    "intra_arm",
                    "external_boxes",
                }, "safety.self_collision.mesh");
                auto& mc = cfg.safety.self_collision.mesh;
                if (has(m, "unified_urdf")) {
                    mc.unified_urdf = resolvePathForConfig(
                        asString(m["unified_urdf"], "safety.self_collision.mesh.unified_urdf"), path);
                }
                if (has(m, "package_dirs")) {
                    const YAML::Node dirs = m["package_dirs"];
                    if (!dirs.IsSequence()) {
                        fail("safety.self_collision.mesh.package_dirs must be a sequence", dirs);
                    }
                    mc.package_dirs.clear();
                    for (std::size_t i = 0; i < dirs.size(); ++i) {
                        mc.package_dirs.push_back(resolvePathForConfig(
                            asString(dirs[i], "safety.self_collision.mesh.package_dirs"), path));
                    }
                }
                if (has(m, "pika_gripper_mesh")) {
                    mc.pika_gripper_mesh = resolvePathForConfig(
                        asString(m["pika_gripper_mesh"], "safety.self_collision.mesh.pika_gripper_mesh"), path);
                }
                if (has(m, "pika_gripper_base_mesh")) {
                    mc.pika_gripper_base_mesh = resolvePathForConfig(
                        asString(m["pika_gripper_base_mesh"], "safety.self_collision.mesh.pika_gripper_base_mesh"), path);
                }
                if (has(m, "pika_finger_left_mesh")) {
                    mc.pika_finger_left_mesh = resolvePathForConfig(
                        asString(m["pika_finger_left_mesh"], "safety.self_collision.mesh.pika_finger_left_mesh"), path);
                }
                if (has(m, "pika_finger_right_mesh")) {
                    mc.pika_finger_right_mesh = resolvePathForConfig(
                        asString(m["pika_finger_right_mesh"], "safety.self_collision.mesh.pika_finger_right_mesh"), path);
                }
                if (has(m, "gripper_finger_travel_m")) {
                    mc.gripper_finger_travel_m = asDouble(m["gripper_finger_travel_m"], "safety.self_collision.mesh.gripper_finger_travel_m");
                }
                if (has(m, "left_prefix")) mc.left_prefix = asString(m["left_prefix"], "safety.self_collision.mesh.left_prefix");
                if (has(m, "right_prefix")) mc.right_prefix = asString(m["right_prefix"], "safety.self_collision.mesh.right_prefix");
                if (has(m, "stand_frame")) mc.stand_frame = asString(m["stand_frame"], "safety.self_collision.mesh.stand_frame");
                if (has(m, "stand_ignore_arm_substrings")) {
                    const YAML::Node subs = m["stand_ignore_arm_substrings"];
                    if (!subs.IsSequence()) {
                        fail("safety.self_collision.mesh.stand_ignore_arm_substrings must be a sequence", subs);
                    }
                    mc.stand_ignore_arm_substrings.clear();
                    for (std::size_t i = 0; i < subs.size(); ++i) {
                        mc.stand_ignore_arm_substrings.push_back(
                            asString(subs[i], "safety.self_collision.mesh.stand_ignore_arm_substrings"));
                    }
                }
                if (has(m, "left_arm_root_frame")) mc.left_arm_root_frame = asString(m["left_arm_root_frame"], "safety.self_collision.mesh.left_arm_root_frame");
                if (has(m, "right_arm_root_frame")) mc.right_arm_root_frame = asString(m["right_arm_root_frame"], "safety.self_collision.mesh.right_arm_root_frame");
                if (has(m, "check_intra_arm")) mc.check_intra_arm = asBool(m["check_intra_arm"], "safety.self_collision.mesh.check_intra_arm");
                if (has(m, "intra_arm_min_chain_separation")) mc.intra_arm_min_chain_separation = asInt(m["intra_arm_min_chain_separation"], "safety.self_collision.mesh.intra_arm_min_chain_separation");
                if (has(m, "disabled_collision_pairs")) {
                    const YAML::Node rules = m["disabled_collision_pairs"];
                    if (!rules.IsSequence()) {
                        fail("safety.self_collision.mesh.disabled_collision_pairs must be a sequence", rules);
                    }
                    mc.disabled_collision_pairs.clear();
                    for (std::size_t i = 0; i < rules.size(); ++i) {
                        const YAML::Node rule = rules[i];
                        if (!rule.IsSequence() || rule.size() != 2) {
                            fail("safety.self_collision.mesh.disabled_collision_pairs entries must be 2-element string lists", rule);
                        }
                        CollisionPairPattern p;
                        p.pattern_a = asString(rule[0], "safety.self_collision.mesh.disabled_collision_pairs[0]");
                        p.pattern_b = asString(rule[1], "safety.self_collision.mesh.disabled_collision_pairs[1]");
                        mc.disabled_collision_pairs.push_back(std::move(p));
                    }
                }
                if (has(m, "debug_pair_curation")) {
                    mc.debug_pair_curation = asBool(m["debug_pair_curation"],
                                                    "safety.self_collision.mesh.debug_pair_curation");
                }
                if (has(m, "swept_samples")) mc.swept_samples = asInt(m["swept_samples"], "safety.self_collision.mesh.swept_samples");
                if (has(m, "d_hard_m")) mc.d_hard_m = asDouble(m["d_hard_m"], "safety.self_collision.mesh.d_hard_m");
                if (has(m, "d_slow_m")) mc.d_slow_m = asDouble(m["d_slow_m"], "safety.self_collision.mesh.d_slow_m");
                if (has(m, "a_brake_m_s2")) mc.a_brake_m_s2 = asDouble(m["a_brake_m_s2"], "safety.self_collision.mesh.a_brake_m_s2");
                if (has(m, "hyst_m")) mc.hyst_m = asDouble(m["hyst_m"], "safety.self_collision.mesh.hyst_m");
                if (has(m, "projection_iterations")) mc.projection_iterations = asInt(m["projection_iterations"], "safety.self_collision.mesh.projection_iterations");
                if (has(m, "recover_speed_m_s")) mc.recover_speed_m_s = asDouble(m["recover_speed_m_s"], "safety.self_collision.mesh.recover_speed_m_s");
                if (has(m, "latency_s")) mc.latency_s = asDouble(m["latency_s"], "safety.self_collision.mesh.latency_s");
                if (has(m, "max_staleness_s")) mc.max_staleness_s = asDouble(m["max_staleness_s"], "safety.self_collision.mesh.max_staleness_s");
                if (has(m, "monitor_core")) mc.monitor_core = asInt(m["monitor_core"], "safety.self_collision.mesh.monitor_core");
                if (has(m, "max_near_pairs")) mc.max_near_pairs = asInt(m["max_near_pairs"], "safety.self_collision.mesh.max_near_pairs");
                if (has(m, "viz_near_pairs_m")) mc.viz_near_pairs_m = asDouble(m["viz_near_pairs_m"], "safety.self_collision.mesh.viz_near_pairs_m");
                if (has(m, "extra_collision")) {
                    const YAML::Node arr = m["extra_collision"];
                    if (!arr.IsSequence()) {
                        fail("safety.self_collision.mesh.extra_collision must be a sequence", arr);
                    }
                    mc.extra_collision.clear();
                    for (std::size_t i = 0; i < arr.size(); ++i) {
                        const YAML::Node e = arr[i];
                        validateAllowedKeys(e, {
                            "name", "shape", "parent_frame", "size_m",
                            "radius_m", "length_m", "xyz_m", "rpy",
                        }, "safety.self_collision.mesh.extra_collision");
                        ExtraCollisionConfig ec;
                        if (!has(e, "name")) fail("extra_collision entry requires name", e);
                        if (!has(e, "parent_frame")) fail("extra_collision entry requires parent_frame", e);
                        ec.name = asString(e["name"], "extra_collision.name");
                        ec.parent_frame = asString(e["parent_frame"], "extra_collision.parent_frame");
                        if (has(e, "shape")) ec.shape = asString(e["shape"], "extra_collision.shape");
                        const auto vec3 = [&](const YAML::Node& n, const char* key, std::array<double, 3>* out) {
                            if (!n.IsSequence() || n.size() != 3)
                                fail(std::string("extra_collision.") + key + " must be 3 values (" + ec.name + ")", n);
                            for (std::size_t a = 0; a < 3; ++a)
                                (*out)[a] = asDouble(n[a], std::string("extra_collision.") + key);
                        };
                        if (has(e, "size_m")) vec3(e["size_m"], "size_m", &ec.size_m);
                        if (has(e, "xyz_m")) vec3(e["xyz_m"], "xyz_m", &ec.xyz_m);
                        if (has(e, "rpy")) vec3(e["rpy"], "rpy", &ec.rpy);
                        if (has(e, "radius_m")) ec.radius_m = asDouble(e["radius_m"], "extra_collision.radius_m");
                        if (has(e, "length_m")) ec.length_m = asDouble(e["length_m"], "extra_collision.length_m");
                        mc.extra_collision.push_back(ec);
                    }
                }
                if (has(m, "ground_plane")) {
                    const YAML::Node gp = m["ground_plane"];
                    validateAllowedKeys(gp, {
                        "enable", "z_m", "size_m", "thickness_m", "parent_frame",
                        "follow_safety_floors",
                    }, "safety.self_collision.mesh.ground_plane");
                    auto& g = mc.ground_plane;
                    if (has(gp, "enable")) g.enable = asBool(gp["enable"], "safety.self_collision.mesh.ground_plane.enable");
                    if (has(gp, "follow_safety_floors")) g.follow_safety_floors = asBool(gp["follow_safety_floors"], "safety.self_collision.mesh.ground_plane.follow_safety_floors");
                    if (has(gp, "z_m")) g.z_m = asDouble(gp["z_m"], "safety.self_collision.mesh.ground_plane.z_m");
                    if (has(gp, "thickness_m")) g.thickness_m = asDouble(gp["thickness_m"], "safety.self_collision.mesh.ground_plane.thickness_m");
                    if (has(gp, "parent_frame")) g.parent_frame = asString(gp["parent_frame"], "safety.self_collision.mesh.ground_plane.parent_frame");
                    if (has(gp, "size_m")) {
                        const YAML::Node sz = gp["size_m"];
                        if (!sz.IsSequence() || sz.size() != 2)
                            fail("safety.self_collision.mesh.ground_plane.size_m must be 2 values [Lx, Ly]", sz);
                        g.size_m[0] = asDouble(sz[0], "safety.self_collision.mesh.ground_plane.size_m");
                        g.size_m[1] = asDouble(sz[1], "safety.self_collision.mesh.ground_plane.size_m");
                    }
                }
                if (has(m, "external")) {
                    const YAML::Node ex = m["external"];
                    validateAllowedKeys(ex, {
                        "d_hard_m", "d_slow_m", "a_brake_m_s2", "hyst_m",
                        "recover_speed_m_s", "latency_s",
                    }, "safety.self_collision.mesh.external");
                    auto& x = mc.external;
                    if (has(ex, "d_hard_m")) x.d_hard_m = asDouble(ex["d_hard_m"], "safety.self_collision.mesh.external.d_hard_m");
                    if (has(ex, "d_slow_m")) x.d_slow_m = asDouble(ex["d_slow_m"], "safety.self_collision.mesh.external.d_slow_m");
                    if (has(ex, "a_brake_m_s2")) x.a_brake_m_s2 = asDouble(ex["a_brake_m_s2"], "safety.self_collision.mesh.external.a_brake_m_s2");
                    if (has(ex, "hyst_m")) x.hyst_m = asDouble(ex["hyst_m"], "safety.self_collision.mesh.external.hyst_m");
                    if (has(ex, "recover_speed_m_s")) x.recover_speed_m_s = asDouble(ex["recover_speed_m_s"], "safety.self_collision.mesh.external.recover_speed_m_s");
                    if (has(ex, "latency_s")) x.latency_s = asDouble(ex["latency_s"], "safety.self_collision.mesh.external.latency_s");
                }
                if (has(m, "intra_arm")) {
                    const YAML::Node ia = m["intra_arm"];
                    validateAllowedKeys(ia, {
                        "d_hard_m", "d_slow_m", "a_brake_m_s2", "hyst_m",
                        "recover_speed_m_s", "latency_s",
                    }, "safety.self_collision.mesh.intra_arm");
                    auto& x = mc.intra_arm;
                    if (has(ia, "d_hard_m")) x.d_hard_m = asDouble(ia["d_hard_m"], "safety.self_collision.mesh.intra_arm.d_hard_m");
                    if (has(ia, "d_slow_m")) x.d_slow_m = asDouble(ia["d_slow_m"], "safety.self_collision.mesh.intra_arm.d_slow_m");
                    if (has(ia, "a_brake_m_s2")) x.a_brake_m_s2 = asDouble(ia["a_brake_m_s2"], "safety.self_collision.mesh.intra_arm.a_brake_m_s2");
                    if (has(ia, "hyst_m")) x.hyst_m = asDouble(ia["hyst_m"], "safety.self_collision.mesh.intra_arm.hyst_m");
                    if (has(ia, "recover_speed_m_s")) x.recover_speed_m_s = asDouble(ia["recover_speed_m_s"], "safety.self_collision.mesh.intra_arm.recover_speed_m_s");
                    if (has(ia, "latency_s")) x.latency_s = asDouble(ia["latency_s"], "safety.self_collision.mesh.intra_arm.latency_s");
                }
                if (has(m, "external_boxes")) {
                    const YAML::Node eb = m["external_boxes"];
                    validateAllowedKeys(eb, {
                        "enable", "max_count", "size_m", "margin_m", "monitor_only",
                        "stale_timeout_s", "stale_policy", "barrier",
                    }, "safety.self_collision.mesh.external_boxes");
                    auto& x = mc.external_boxes;
                    if (has(eb, "enable")) x.enable = asBool(eb["enable"], "safety.self_collision.mesh.external_boxes.enable");
                    if (has(eb, "max_count")) x.max_count = asInt(eb["max_count"], "safety.self_collision.mesh.external_boxes.max_count");
                    if (has(eb, "size_m")) {
                        const YAML::Node sz = eb["size_m"];
                        if (!sz.IsSequence() || sz.size() != 3) {
                            fail("safety.self_collision.mesh.external_boxes.size_m must be 3 values [x, y, z]", sz);
                        }
                        for (std::size_t i = 0; i < 3; ++i) {
                            x.size_m[i] = asDouble(sz[i], "safety.self_collision.mesh.external_boxes.size_m");
                        }
                    }
                    if (has(eb, "margin_m")) {
                        const YAML::Node mg = eb["margin_m"];
                        if (mg.IsSequence()) {
                            if (mg.size() != 3) {
                                fail("safety.self_collision.mesh.external_boxes.margin_m must be a scalar or 3 values [x, y, z]", mg);
                            }
                            for (std::size_t i = 0; i < 3; ++i) {
                                x.margin_m[i] = asDouble(mg[i], "safety.self_collision.mesh.external_boxes.margin_m");
                            }
                        } else {
                            const double v = asDouble(mg, "safety.self_collision.mesh.external_boxes.margin_m");
                            x.margin_m = {v, v, v};
                        }
                    }
                    if (has(eb, "monitor_only")) x.monitor_only = asBool(eb["monitor_only"], "safety.self_collision.mesh.external_boxes.monitor_only");
                    if (has(eb, "stale_timeout_s")) x.stale_timeout_s = asDouble(eb["stale_timeout_s"], "safety.self_collision.mesh.external_boxes.stale_timeout_s");
                    if (has(eb, "stale_policy")) x.stale_policy = asString(eb["stale_policy"], "safety.self_collision.mesh.external_boxes.stale_policy");
                    if (has(eb, "barrier")) {
                        const YAML::Node br = eb["barrier"];
                        validateAllowedKeys(br, {
                            "d_hard_m", "d_slow_m", "a_brake_m_s2", "hyst_m",
                            "recover_speed_m_s", "latency_s",
                        }, "safety.self_collision.mesh.external_boxes.barrier");
                        auto& b = x.barrier;
                        if (has(br, "d_hard_m")) b.d_hard_m = asDouble(br["d_hard_m"], "safety.self_collision.mesh.external_boxes.barrier.d_hard_m");
                        if (has(br, "d_slow_m")) b.d_slow_m = asDouble(br["d_slow_m"], "safety.self_collision.mesh.external_boxes.barrier.d_slow_m");
                        if (has(br, "a_brake_m_s2")) b.a_brake_m_s2 = asDouble(br["a_brake_m_s2"], "safety.self_collision.mesh.external_boxes.barrier.a_brake_m_s2");
                        if (has(br, "hyst_m")) b.hyst_m = asDouble(br["hyst_m"], "safety.self_collision.mesh.external_boxes.barrier.hyst_m");
                        if (has(br, "recover_speed_m_s")) b.recover_speed_m_s = asDouble(br["recover_speed_m_s"], "safety.self_collision.mesh.external_boxes.barrier.recover_speed_m_s");
                        if (has(br, "latency_s")) b.latency_s = asDouble(br["latency_s"], "safety.self_collision.mesh.external_boxes.barrier.latency_s");
                    }
                }
            }
        }
        if (has(sec, "floor_constraint")) {
            const YAML::Node fc = sec["floor_constraint"];
            validateAllowedKeys(fc, {
                "enable",
                "z_min_m",
                "runtime_min_z_m",
                "runtime_max_z_m",
                "fail_policy",
                "monitor_only",
                "tcp_offset_points",
                "a_brake_m_s2",
                "d_slow_m",
            }, "safety.floor_constraint");
            if (has(fc, "enable")) {
                cfg.safety.floor_constraint.enable = asBool(fc["enable"], "safety.floor_constraint.enable");
            }
            if (has(fc, "z_min_m")) {
                cfg.safety.floor_constraint.z_min_m =
                    asDouble(fc["z_min_m"], "safety.floor_constraint.z_min_m");
            }
            if (has(fc, "runtime_min_z_m")) {
                cfg.safety.floor_constraint.runtime_min_z_m =
                    asDouble(fc["runtime_min_z_m"], "safety.floor_constraint.runtime_min_z_m");
            }
            if (has(fc, "runtime_max_z_m")) {
                cfg.safety.floor_constraint.runtime_max_z_m =
                    asDouble(fc["runtime_max_z_m"], "safety.floor_constraint.runtime_max_z_m");
            }
            if (has(fc, "fail_policy")) {
                cfg.safety.floor_constraint.fail_policy =
                    parseFloorConstraintFailPolicy(fc["fail_policy"], "safety.floor_constraint.fail_policy");
            }
            if (has(fc, "monitor_only")) {
                cfg.safety.floor_constraint.monitor_only =
                    asBool(fc["monitor_only"], "safety.floor_constraint.monitor_only");
            }
            if (has(fc, "a_brake_m_s2")) {
                cfg.safety.floor_constraint.a_brake_m_s2 =
                    asDouble(fc["a_brake_m_s2"], "safety.floor_constraint.a_brake_m_s2");
            }
            if (has(fc, "d_slow_m")) {
                cfg.safety.floor_constraint.d_slow_m =
                    asDouble(fc["d_slow_m"], "safety.floor_constraint.d_slow_m");
            }
            if (has(fc, "tcp_offset_points")) {
                cfg.safety.floor_constraint.tcp_offset_points = parseTcpOffsetPoints(
                    fc["tcp_offset_points"], "safety.floor_constraint.tcp_offset_points");
            }
        }
        if (has(sec, "roi_box")) {
            const YAML::Node rb = sec["roi_box"];
            validateAllowedKeys(rb, {
                "enable",
                "min_m",
                "max_m",
                "runtime_min_m",
                "runtime_max_m",
                "fail_policy",
                "monitor_only",
                "tcp_offset_points",
                "a_brake_m_s2",
                "d_slow_m",
            }, "safety.roi_box");
            const auto parseVec3 = [&](const YAML::Node& node, const std::string& path,
                                       std::array<double, 3>& out) {
                if (!node.IsSequence() || node.size() != 3) {
                    fail(path + " must be a [x, y, z] sequence", node);
                }
                for (std::size_t axis = 0; axis < 3; ++axis) {
                    out[axis] = asDouble(node[axis], path);
                }
            };
            if (has(rb, "enable")) {
                cfg.safety.roi_box.enable = asBool(rb["enable"], "safety.roi_box.enable");
            }
            if (has(rb, "min_m")) parseVec3(rb["min_m"], "safety.roi_box.min_m", cfg.safety.roi_box.min_m);
            if (has(rb, "max_m")) parseVec3(rb["max_m"], "safety.roi_box.max_m", cfg.safety.roi_box.max_m);
            if (has(rb, "runtime_min_m")) {
                parseVec3(rb["runtime_min_m"], "safety.roi_box.runtime_min_m",
                          cfg.safety.roi_box.runtime_min_m);
            }
            if (has(rb, "runtime_max_m")) {
                parseVec3(rb["runtime_max_m"], "safety.roi_box.runtime_max_m",
                          cfg.safety.roi_box.runtime_max_m);
            }
            if (has(rb, "fail_policy")) {
                cfg.safety.roi_box.fail_policy =
                    parseFloorConstraintFailPolicy(rb["fail_policy"], "safety.roi_box.fail_policy");
            }
            if (has(rb, "monitor_only")) {
                cfg.safety.roi_box.monitor_only = asBool(rb["monitor_only"], "safety.roi_box.monitor_only");
            }
            if (has(rb, "a_brake_m_s2")) {
                cfg.safety.roi_box.a_brake_m_s2 = asDouble(rb["a_brake_m_s2"], "safety.roi_box.a_brake_m_s2");
            }
            if (has(rb, "d_slow_m")) {
                cfg.safety.roi_box.d_slow_m = asDouble(rb["d_slow_m"], "safety.roi_box.d_slow_m");
            }
            if (has(rb, "tcp_offset_points")) {
                cfg.safety.roi_box.tcp_offset_points = parseTcpOffsetPoints(
                    rb["tcp_offset_points"], "safety.roi_box.tcp_offset_points");
            }
        }
        if (has(sec, "reach_constraint")) {
            const YAML::Node rc = sec["reach_constraint"];
            validateAllowedKeys(rc, {
                "enable",
                "r_max_m",
                "r_min_m",
                "fail_policy",
                "monitor_only",
                "tcp_offset_points",
                "a_brake_m_s2",
                "d_slow_m",
            }, "safety.reach_constraint");
            if (has(rc, "enable")) {
                cfg.safety.reach_constraint.enable =
                    asBool(rc["enable"], "safety.reach_constraint.enable");
            }
            if (has(rc, "r_max_m")) {
                cfg.safety.reach_constraint.r_max_m =
                    asDouble(rc["r_max_m"], "safety.reach_constraint.r_max_m");
            }
            if (has(rc, "r_min_m")) {
                cfg.safety.reach_constraint.r_min_m =
                    asDouble(rc["r_min_m"], "safety.reach_constraint.r_min_m");
            }
            if (has(rc, "fail_policy")) {
                cfg.safety.reach_constraint.fail_policy =
                    parseFloorConstraintFailPolicy(rc["fail_policy"], "safety.reach_constraint.fail_policy");
            }
            if (has(rc, "monitor_only")) {
                cfg.safety.reach_constraint.monitor_only =
                    asBool(rc["monitor_only"], "safety.reach_constraint.monitor_only");
            }
            if (has(rc, "a_brake_m_s2")) {
                cfg.safety.reach_constraint.a_brake_m_s2 =
                    asDouble(rc["a_brake_m_s2"], "safety.reach_constraint.a_brake_m_s2");
            }
            if (has(rc, "d_slow_m")) {
                cfg.safety.reach_constraint.d_slow_m =
                    asDouble(rc["d_slow_m"], "safety.reach_constraint.d_slow_m");
            }
            if (has(rc, "tcp_offset_points")) {
                cfg.safety.reach_constraint.tcp_offset_points = parseTcpOffsetPoints(
                    rc["tcp_offset_points"], "safety.reach_constraint.tcp_offset_points");
            }
        }
        if (has(sec, "user_floor_constraint")) {
            const YAML::Node uf = sec["user_floor_constraint"];
            validateAllowedKeys(uf, {
                "enable",
                "point_m",
                "normal",
                "margin_m",
                "max_tilt_deg",
                "runtime_min_point_z_m",
                "runtime_max_point_z_m",
                "max_margin_m",
                "fail_policy",
                "monitor_only",
                "tcp_offset_points",
                "a_brake_m_s2",
                "d_slow_m",
            }, "safety.user_floor_constraint");
            const auto parseVec3 = [&](const YAML::Node& node, const std::string& path,
                                       std::array<double, 3>& out) {
                if (!node.IsSequence() || node.size() != 3) {
                    fail(path + " must be a [x, y, z] sequence", node);
                }
                for (std::size_t axis = 0; axis < 3; ++axis) {
                    out[axis] = asDouble(node[axis], path);
                }
            };
            if (has(uf, "enable")) {
                cfg.safety.user_floor_constraint.enable =
                    asBool(uf["enable"], "safety.user_floor_constraint.enable");
            }
            // point_m and/or normal present => treat the config plane as usable at startup.
            if (has(uf, "point_m")) {
                parseVec3(uf["point_m"], "safety.user_floor_constraint.point_m",
                          cfg.safety.user_floor_constraint.point_m);
                cfg.safety.user_floor_constraint.has_initial_plane = true;
            }
            if (has(uf, "normal")) {
                parseVec3(uf["normal"], "safety.user_floor_constraint.normal",
                          cfg.safety.user_floor_constraint.normal);
                cfg.safety.user_floor_constraint.has_initial_plane = true;
            }
            if (has(uf, "margin_m")) {
                cfg.safety.user_floor_constraint.margin_m =
                    asDouble(uf["margin_m"], "safety.user_floor_constraint.margin_m");
            }
            if (has(uf, "max_tilt_deg")) {
                cfg.safety.user_floor_constraint.max_tilt_deg =
                    asDouble(uf["max_tilt_deg"], "safety.user_floor_constraint.max_tilt_deg");
            }
            if (has(uf, "runtime_min_point_z_m")) {
                cfg.safety.user_floor_constraint.runtime_min_point_z_m =
                    asDouble(uf["runtime_min_point_z_m"], "safety.user_floor_constraint.runtime_min_point_z_m");
            }
            if (has(uf, "runtime_max_point_z_m")) {
                cfg.safety.user_floor_constraint.runtime_max_point_z_m =
                    asDouble(uf["runtime_max_point_z_m"], "safety.user_floor_constraint.runtime_max_point_z_m");
            }
            if (has(uf, "max_margin_m")) {
                cfg.safety.user_floor_constraint.max_margin_m =
                    asDouble(uf["max_margin_m"], "safety.user_floor_constraint.max_margin_m");
            }
            if (has(uf, "fail_policy")) {
                cfg.safety.user_floor_constraint.fail_policy =
                    parseFloorConstraintFailPolicy(uf["fail_policy"], "safety.user_floor_constraint.fail_policy");
            }
            if (has(uf, "monitor_only")) {
                cfg.safety.user_floor_constraint.monitor_only =
                    asBool(uf["monitor_only"], "safety.user_floor_constraint.monitor_only");
            }
            if (has(uf, "a_brake_m_s2")) {
                cfg.safety.user_floor_constraint.a_brake_m_s2 =
                    asDouble(uf["a_brake_m_s2"], "safety.user_floor_constraint.a_brake_m_s2");
            }
            if (has(uf, "d_slow_m")) {
                cfg.safety.user_floor_constraint.d_slow_m =
                    asDouble(uf["d_slow_m"], "safety.user_floor_constraint.d_slow_m");
            }
            if (has(uf, "tcp_offset_points")) {
                cfg.safety.user_floor_constraint.tcp_offset_points = parseTcpOffsetPoints(
                    uf["tcp_offset_points"], "safety.user_floor_constraint.tcp_offset_points");
            }
        }
        if (has(sec, "joint_target_smd")) {
            const YAML::Node js = sec["joint_target_smd"];
            validateAllowedKeys(js, {
                "enable",
                "damping_ratio",
                "natural_frequency_hz",
                "max_velocity_deg_s",
                "max_accel_deg_s2",
                "arrival_taper_enable",
                "arrival_decel_deg_s2",
                "arrival_min_speed_deg_s",
            }, "safety.joint_target_smd");
            if (has(js, "enable")) {
                cfg.safety.joint_target_smd.enable =
                    asBool(js["enable"], "safety.joint_target_smd.enable");
            }
            if (has(js, "damping_ratio")) {
                cfg.safety.joint_target_smd.damping_ratio =
                    asDouble(js["damping_ratio"], "safety.joint_target_smd.damping_ratio");
            }
            if (has(js, "natural_frequency_hz")) {
                cfg.safety.joint_target_smd.natural_frequency_hz =
                    asDouble(js["natural_frequency_hz"], "safety.joint_target_smd.natural_frequency_hz");
            }
            if (has(js, "max_velocity_deg_s")) {
                cfg.safety.joint_target_smd.max_velocity_deg_s =
                    parseJointArray(js["max_velocity_deg_s"], "safety.joint_target_smd.max_velocity_deg_s");
            }
            if (has(js, "max_accel_deg_s2")) {
                cfg.safety.joint_target_smd.max_accel_deg_s2 =
                    parseJointArray(js["max_accel_deg_s2"], "safety.joint_target_smd.max_accel_deg_s2");
            }
            if (has(js, "arrival_taper_enable")) {
                cfg.safety.joint_target_smd.arrival_taper_enable =
                    asBool(js["arrival_taper_enable"], "safety.joint_target_smd.arrival_taper_enable");
            }
            if (has(js, "arrival_decel_deg_s2")) {
                cfg.safety.joint_target_smd.arrival_decel_deg_s2 =
                    asDouble(js["arrival_decel_deg_s2"], "safety.joint_target_smd.arrival_decel_deg_s2");
            }
            if (has(js, "arrival_min_speed_deg_s")) {
                cfg.safety.joint_target_smd.arrival_min_speed_deg_s =
                    asDouble(js["arrival_min_speed_deg_s"], "safety.joint_target_smd.arrival_min_speed_deg_s");
            }
        }
        if (has(sec, "init_motion_planner")) {
            const YAML::Node ip = sec["init_motion_planner"];
            validateAllowedKeys(ip, {
                "enable",
                "max_planning_time_sec",
                "max_iterations",
                "step_size_rad",
                "edge_resolution_rad",
                "goal_bias",
                "shortcut_passes",
                "sample_margin_deg",
                "sample_margin_deg_per_joint",
                "global_sample_fraction",
                "global_sample_margin_deg",
                "collision_margin_m",
                "seed",
                "waypoint_tol_deg",
                "noop_tol_deg",
                "max_segment_deg",
                "escape_max_time_sec",
                "escape_max_steps",
                "escape_restart_attempts",
                "escape_perturb_deg",
                "lazy_edges",
                "execution_lookahead_deg",
                "execution_timeout_sec",
                "single_arm_freeze_other_arm",
            }, "safety.init_motion_planner");
            auto& ipc = cfg.safety.init_motion_planner;
            if (has(ip, "enable")) ipc.enable = asBool(ip["enable"], "safety.init_motion_planner.enable");
            if (has(ip, "max_planning_time_sec")) ipc.max_planning_time_sec = asDouble(ip["max_planning_time_sec"], "safety.init_motion_planner.max_planning_time_sec");
            if (has(ip, "max_iterations")) ipc.max_iterations = asInt(ip["max_iterations"], "safety.init_motion_planner.max_iterations");
            if (has(ip, "step_size_rad")) ipc.step_size_rad = asDouble(ip["step_size_rad"], "safety.init_motion_planner.step_size_rad");
            if (has(ip, "edge_resolution_rad")) ipc.edge_resolution_rad = asDouble(ip["edge_resolution_rad"], "safety.init_motion_planner.edge_resolution_rad");
            if (has(ip, "goal_bias")) ipc.goal_bias = asDouble(ip["goal_bias"], "safety.init_motion_planner.goal_bias");
            if (has(ip, "shortcut_passes")) ipc.shortcut_passes = asInt(ip["shortcut_passes"], "safety.init_motion_planner.shortcut_passes");
            if (has(ip, "sample_margin_deg")) ipc.sample_margin_deg = asDouble(ip["sample_margin_deg"], "safety.init_motion_planner.sample_margin_deg");
            if (has(ip, "sample_margin_deg_per_joint")) {
                ipc.sample_margin_deg_per_joint = parseJointArray(ip["sample_margin_deg_per_joint"],
                    "safety.init_motion_planner.sample_margin_deg_per_joint");
            }
            if (has(ip, "global_sample_fraction")) ipc.global_sample_fraction = asDouble(ip["global_sample_fraction"], "safety.init_motion_planner.global_sample_fraction");
            if (has(ip, "global_sample_margin_deg")) ipc.global_sample_margin_deg = asDouble(ip["global_sample_margin_deg"], "safety.init_motion_planner.global_sample_margin_deg");
            if (has(ip, "collision_margin_m")) ipc.collision_margin_m = asDouble(ip["collision_margin_m"], "safety.init_motion_planner.collision_margin_m");
            if (has(ip, "seed")) ipc.seed = static_cast<unsigned int>(asInt(ip["seed"], "safety.init_motion_planner.seed"));
            if (has(ip, "waypoint_tol_deg")) ipc.waypoint_tol_deg = asDouble(ip["waypoint_tol_deg"], "safety.init_motion_planner.waypoint_tol_deg");
            if (has(ip, "noop_tol_deg")) ipc.noop_tol_deg = asDouble(ip["noop_tol_deg"], "safety.init_motion_planner.noop_tol_deg");
            if (has(ip, "max_segment_deg")) ipc.max_segment_deg = asDouble(ip["max_segment_deg"], "safety.init_motion_planner.max_segment_deg");
            if (has(ip, "escape_max_time_sec")) ipc.escape_max_time_sec = asDouble(ip["escape_max_time_sec"], "safety.init_motion_planner.escape_max_time_sec");
            if (has(ip, "escape_max_steps")) ipc.escape_max_steps = asInt(ip["escape_max_steps"], "safety.init_motion_planner.escape_max_steps");
            if (has(ip, "escape_restart_attempts")) ipc.escape_restart_attempts = asInt(ip["escape_restart_attempts"], "safety.init_motion_planner.escape_restart_attempts");
            if (has(ip, "escape_perturb_deg")) ipc.escape_perturb_deg = asDouble(ip["escape_perturb_deg"], "safety.init_motion_planner.escape_perturb_deg");
            if (has(ip, "lazy_edges")) ipc.lazy_edges = asBool(ip["lazy_edges"], "safety.init_motion_planner.lazy_edges");
            if (has(ip, "execution_lookahead_deg")) ipc.execution_lookahead_deg = asDouble(ip["execution_lookahead_deg"], "safety.init_motion_planner.execution_lookahead_deg");
            if (has(ip, "execution_timeout_sec")) ipc.execution_timeout_sec = asDouble(ip["execution_timeout_sec"], "safety.init_motion_planner.execution_timeout_sec");
            if (has(ip, "single_arm_freeze_other_arm")) ipc.single_arm_freeze_other_arm = asBool(ip["single_arm_freeze_other_arm"], "safety.init_motion_planner.single_arm_freeze_other_arm");
        }
    }

    if (has(root, "network")) {
        const YAML::Node sec = root["network"];
        validateAllowedKeys(sec, {
            "command_bind",
            "state_pub_endpoint",
            "state_pub_endpoints",
            "state_pub_bind",
            "scope_pub_endpoints",
            "state_pub_rate_hz",
            "command_source_allowlist",
            "chunk_frame_bind",
        }, "network");
        cfg.network.command_bind = getString(sec, "command_bind", cfg.network.command_bind, "network");
        cfg.network.chunk_frame_bind = getString(sec, "chunk_frame_bind", cfg.network.chunk_frame_bind, "network");
        if (has(sec, "state_pub_endpoints") && (has(sec, "state_pub_endpoint") || has(sec, "state_pub_bind"))) {
            fail("network.state_pub_endpoints cannot be combined with state_pub_endpoint or deprecated state_pub_bind", sec["state_pub_endpoints"]);
        }
        if (has(sec, "state_pub_endpoint") && has(sec, "state_pub_bind")) {
            fail("network cannot set both state_pub_endpoint and deprecated state_pub_bind", sec["state_pub_bind"]);
        }
        if (has(sec, "state_pub_endpoints")) {
            cfg.network.state_pub_endpoints = dedupeStatePubEndpoints(
                asStringArray(sec["state_pub_endpoints"], "network.state_pub_endpoints")
            );
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
        if (has(sec, "scope_pub_endpoints")) {
            cfg.network.scope_pub_endpoints = dedupeScopePubEndpoints(
                asStringArray(sec["scope_pub_endpoints"], "network.scope_pub_endpoints")
            );
            if (cfg.network.scope_pub_endpoints.empty()) {
                fail("network.scope_pub_endpoints must not be empty", sec["scope_pub_endpoints"]);
            }
        }
        if (has(sec, "state_pub_rate_hz")) cfg.network.state_pub_rate_hz = asInt(sec["state_pub_rate_hz"], "network.state_pub_rate_hz");
        if (has(sec, "command_source_allowlist")) {
            cfg.network.command_source_allowlist = asStringArray(sec["command_source_allowlist"], "network.command_source_allowlist");
        }
    } else {
        cfg.network.state_pub_bind = cfg.network.state_pub_endpoint;
        cfg.network.state_pub_endpoints = {cfg.network.state_pub_endpoint};
    }
    cfg.network.command_timeout_sec = cfg.servo.command_timeout_sec;

    if (has(root, "gripper")) {
        const YAML::Node sec = root["gripper"];
        validateAllowedKeys(sec, {
            "enable",
            "command_endpoint",
            "feedback_bind",
            "forward_rate_hz",
            "feedback_stale_timeout_ms",
        }, "gripper");
        if (has(sec, "enable")) cfg.gripper.enable = asBool(sec["enable"], "gripper.enable");
        if (has(sec, "command_endpoint")) cfg.gripper.command_endpoint = asString(sec["command_endpoint"], "gripper.command_endpoint");
        if (has(sec, "feedback_bind")) cfg.gripper.feedback_bind = asString(sec["feedback_bind"], "gripper.feedback_bind");
        if (has(sec, "forward_rate_hz")) cfg.gripper.forward_rate_hz = asInt(sec["forward_rate_hz"], "gripper.forward_rate_hz");
        if (has(sec, "feedback_stale_timeout_ms")) cfg.gripper.feedback_stale_timeout_ms = asDouble(sec["feedback_stale_timeout_ms"], "gripper.feedback_stale_timeout_ms");
        if (cfg.gripper.enable) {
            if (cfg.gripper.command_endpoint.rfind("udp://", 0) != 0) {
                fail("gripper.command_endpoint must be udp://host:port", sec);
            }
            if (cfg.gripper.feedback_bind.rfind("udp://", 0) != 0) {
                fail("gripper.feedback_bind must be udp://host:port", sec);
            }
            if (cfg.gripper.forward_rate_hz <= 0) fail("gripper.forward_rate_hz must be > 0", sec);
        }
    }

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

    if (has(root, "scope")) {
        const YAML::Node sec = root["scope"];
        validateAllowedKeys(sec, {
            "enable",
            "publish_rate_hz",
            "max_samples_per_batch",
        }, "scope");
        if (has(sec, "enable")) cfg.scope.enable = asBool(sec["enable"], "scope.enable");
        if (has(sec, "publish_rate_hz")) {
            cfg.scope.publish_rate_hz = asInt(sec["publish_rate_hz"], "scope.publish_rate_hz");
        }
        if (has(sec, "max_samples_per_batch")) {
            const int max_samples = asInt(sec["max_samples_per_batch"], "scope.max_samples_per_batch");
            if (max_samples <= 0) {
                throw std::runtime_error("scope.max_samples_per_batch must be positive");
            }
            cfg.scope.max_samples_per_batch = static_cast<size_t>(max_samples);
        }
    }

    if (has(root, "force_control")) {
        const YAML::Node sec = root["force_control"];
        validateAllowedKeys(sec, {
            "provider",
            "enable",
            "operating_mode",
            "allow_in_real",
            "supervised_experimental_real",
            "update_rate_hz",
            "left",
            "right",
            "normal_admittance",
            "force_limit_n",
            "backoff_gain_m_s_per_n",
            "backoff_max_velocity_m_s",
            "hard_limit_policy",
            "retreat_distance_m",
            "retreat_virtual_force_n",
            "retreat_timeout_sec",
            "retreat_max_attempts",
            "retreat_attempt_window_sec",
            "virtual_mass",
            "damping",
            "stiffness",
            "release_bleed_stiffness",
            "wrench_deadband",
            "blockwise_release_recenter",
            "max_dt_sec",
            "max_pos_offset_m",
            "max_rot_offset_rad",
            "max_linear_velocity_m_s",
            "max_angular_velocity_rad_s",
            "max_linear_acceleration_m_s2",
            "max_angular_acceleration_rad_s2",
            "max_linear_jerk_m_s3",
            "max_angular_jerk_rad_s3",
            "max_pos_step_m",
            "max_rot_step_rad",
            "max_energy_j",
        }, "force_control");
        if (has(sec, "provider")) cfg.force_control.provider = lower(asString(sec["provider"], "force_control.provider"));
        if (has(sec, "enable")) cfg.force_control.enable = asBool(sec["enable"], "force_control.enable");
        if (has(sec, "operating_mode")) cfg.force_control.operating_mode = lower(asString(sec["operating_mode"], "force_control.operating_mode"));
        if (has(sec, "allow_in_real")) cfg.force_control.allow_in_real = asBool(sec["allow_in_real"], "force_control.allow_in_real");
        if (has(sec, "supervised_experimental_real")) cfg.force_control.supervised_experimental_real = asBool(sec["supervised_experimental_real"], "force_control.supervised_experimental_real");
        if (has(sec, "update_rate_hz")) cfg.force_control.update_rate_hz = asInt(sec["update_rate_hz"], "force_control.update_rate_hz");
        const auto parse_force_arm = [&](
            const YAML::Node& arm,
            ForceControlArmConfig& out,
            const std::string& path
        ) {
            validateAllowedKeys(arm, {
                "enable",
                "surface_source",
                "contact_entry_max_speed_m_s",
                "compliance_frame",
                "target_force_n",
                "contact_enter_force_n",
                "contact_release_force_n",
                "force_deadband_n",
                "hard_normal_force_n",
                "hard_force_norm_n",
                "hard_torque_norm_nm",
                "hard_limit_rate_n_per_ms",
                "hard_limit_rate_floor_n",
                "debounce_samples",
                "hard_limit_debounce_samples",
                "release_dwell_sec",
                "release_velocity_threshold_m_s",
                "transverse_contact_enter_force_n",
                "transverse_contact_release_force_n",
                "torque_contact_enter_nm",
                "torque_contact_release_nm",
                "compliance_axes",
            }, path);
            if (has(arm, "enable")) out.enable = asBool(arm["enable"], path + ".enable");
            if (has(arm, "surface_source")) out.surface_source = lower(asString(arm["surface_source"], path + ".surface_source"));
            if (has(arm, "contact_entry_max_speed_m_s")) {
                out.contact_entry_max_speed_m_s = asDouble(
                    arm["contact_entry_max_speed_m_s"],
                    path + ".contact_entry_max_speed_m_s");
            }
            if (has(arm, "compliance_frame")) out.compliance_frame = lower(asString(arm["compliance_frame"], path + ".compliance_frame"));
            if (has(arm, "target_force_n")) out.target_force_n = asDouble(arm["target_force_n"], path + ".target_force_n");
            if (has(arm, "contact_enter_force_n")) out.contact_enter_force_n = asDouble(arm["contact_enter_force_n"], path + ".contact_enter_force_n");
            if (has(arm, "contact_release_force_n")) out.contact_release_force_n = asDouble(arm["contact_release_force_n"], path + ".contact_release_force_n");
            if (has(arm, "force_deadband_n")) out.force_deadband_n = asDouble(arm["force_deadband_n"], path + ".force_deadband_n");
            if (has(arm, "hard_normal_force_n")) out.hard_normal_force_n = asDouble(arm["hard_normal_force_n"], path + ".hard_normal_force_n");
            if (has(arm, "hard_force_norm_n")) out.hard_force_norm_n = asDouble(arm["hard_force_norm_n"], path + ".hard_force_norm_n");
            if (has(arm, "hard_torque_norm_nm")) out.hard_torque_norm_nm = asDouble(arm["hard_torque_norm_nm"], path + ".hard_torque_norm_nm");
            if (has(arm, "hard_limit_rate_n_per_ms")) {
                out.hard_limit_rate_n_per_ms = asDouble(
                    arm["hard_limit_rate_n_per_ms"],
                    path + ".hard_limit_rate_n_per_ms");
            }
            if (has(arm, "hard_limit_rate_floor_n")) {
                out.hard_limit_rate_floor_n = asDouble(
                    arm["hard_limit_rate_floor_n"],
                    path + ".hard_limit_rate_floor_n");
            }
            if (has(arm, "debounce_samples")) out.debounce_samples = asInt(arm["debounce_samples"], path + ".debounce_samples");
            if (has(arm, "hard_limit_debounce_samples")) out.hard_limit_debounce_samples = asInt(arm["hard_limit_debounce_samples"], path + ".hard_limit_debounce_samples");
            if (has(arm, "release_dwell_sec")) out.release_dwell_sec = asDouble(arm["release_dwell_sec"], path + ".release_dwell_sec");
            if (has(arm, "release_velocity_threshold_m_s")) out.release_velocity_threshold_m_s = asDouble(arm["release_velocity_threshold_m_s"], path + ".release_velocity_threshold_m_s");
            if (has(arm, "transverse_contact_enter_force_n")) out.transverse_contact_enter_force_n = asDouble(arm["transverse_contact_enter_force_n"], path + ".transverse_contact_enter_force_n");
            if (has(arm, "transverse_contact_release_force_n")) out.transverse_contact_release_force_n = asDouble(arm["transverse_contact_release_force_n"], path + ".transverse_contact_release_force_n");
            if (has(arm, "torque_contact_enter_nm")) out.torque_contact_enter_nm = asDouble(arm["torque_contact_enter_nm"], path + ".torque_contact_enter_nm");
            if (has(arm, "torque_contact_release_nm")) out.torque_contact_release_nm = asDouble(arm["torque_contact_release_nm"], path + ".torque_contact_release_nm");
            if (has(arm, "compliance_axes")) {
                const YAML::Node axes = arm["compliance_axes"];
                if (!axes.IsSequence() || axes.size() != 6) {
                    throw std::runtime_error(path + ".compliance_axes must contain 6 booleans");
                }
                out.compliance_axes = {
                    axes[0].as<bool>(), axes[1].as<bool>(), axes[2].as<bool>(),
                    axes[3].as<bool>(), axes[4].as<bool>(), axes[5].as<bool>(),
                };
            }
        };
        if (has(sec, "left")) parse_force_arm(sec["left"], cfg.force_control.left, "force_control.left");
        if (has(sec, "right")) parse_force_arm(sec["right"], cfg.force_control.right, "force_control.right");
        if (has(sec, "normal_admittance")) {
            const YAML::Node normal = sec["normal_admittance"];
            validateAllowedKeys(normal, {
                "virtual_mass_kg",
                "damping_n_s_m",
                "stiffness_n_m",
                "max_unload_offset_m",
                "max_normal_velocity_m_s",
                "max_normal_acceleration_m_s2",
                "max_normal_jerk_m_s3",
                "max_normal_step_m",
                "max_energy_j",
            }, "force_control.normal_admittance");
            auto& out = cfg.force_control.normal_admittance;
            if (has(normal, "virtual_mass_kg")) out.virtual_mass_kg = asDouble(normal["virtual_mass_kg"], "force_control.normal_admittance.virtual_mass_kg");
            if (has(normal, "damping_n_s_m")) out.damping_n_s_m = asDouble(normal["damping_n_s_m"], "force_control.normal_admittance.damping_n_s_m");
            if (has(normal, "stiffness_n_m")) out.stiffness_n_m = asDouble(normal["stiffness_n_m"], "force_control.normal_admittance.stiffness_n_m");
            if (has(normal, "max_unload_offset_m")) out.max_unload_offset_m = asDouble(normal["max_unload_offset_m"], "force_control.normal_admittance.max_unload_offset_m");
            if (has(normal, "max_normal_velocity_m_s")) out.max_normal_velocity_m_s = asDouble(normal["max_normal_velocity_m_s"], "force_control.normal_admittance.max_normal_velocity_m_s");
            if (has(normal, "max_normal_acceleration_m_s2")) out.max_normal_acceleration_m_s2 = asDouble(normal["max_normal_acceleration_m_s2"], "force_control.normal_admittance.max_normal_acceleration_m_s2");
            if (has(normal, "max_normal_jerk_m_s3")) out.max_normal_jerk_m_s3 = asDouble(normal["max_normal_jerk_m_s3"], "force_control.normal_admittance.max_normal_jerk_m_s3");
            if (has(normal, "max_normal_step_m")) out.max_normal_step_m = asDouble(normal["max_normal_step_m"], "force_control.normal_admittance.max_normal_step_m");
            if (has(normal, "max_energy_j")) out.max_energy_j = asDouble(normal["max_energy_j"], "force_control.normal_admittance.max_energy_j");
        }
        if (has(sec, "virtual_mass")) cfg.force_control.virtual_mass = parseJointArray(sec["virtual_mass"], "force_control.virtual_mass");
        if (has(sec, "damping")) cfg.force_control.damping = parseJointArray(sec["damping"], "force_control.damping");
        if (has(sec, "stiffness")) cfg.force_control.stiffness = parseJointArray(sec["stiffness"], "force_control.stiffness");
        if (has(sec, "release_bleed_stiffness")) cfg.force_control.release_bleed_stiffness = parseJointArray(sec["release_bleed_stiffness"], "force_control.release_bleed_stiffness");
        if (has(sec, "hard_limit_policy")) cfg.force_control.hard_limit_policy = lower(asString(sec["hard_limit_policy"], "force_control.hard_limit_policy"));
        if (has(sec, "retreat_distance_m")) cfg.force_control.retreat_distance_m = asDouble(sec["retreat_distance_m"], "force_control.retreat_distance_m");
        if (has(sec, "retreat_virtual_force_n")) cfg.force_control.retreat_virtual_force_n = asDouble(sec["retreat_virtual_force_n"], "force_control.retreat_virtual_force_n");
        if (has(sec, "retreat_timeout_sec")) cfg.force_control.retreat_timeout_sec = asDouble(sec["retreat_timeout_sec"], "force_control.retreat_timeout_sec");
        if (has(sec, "retreat_max_attempts")) cfg.force_control.retreat_max_attempts = asInt(sec["retreat_max_attempts"], "force_control.retreat_max_attempts");
        if (has(sec, "retreat_attempt_window_sec")) cfg.force_control.retreat_attempt_window_sec = asDouble(sec["retreat_attempt_window_sec"], "force_control.retreat_attempt_window_sec");
        if (has(sec, "wrench_deadband")) cfg.force_control.wrench_deadband = parseJointArray(sec["wrench_deadband"], "force_control.wrench_deadband");
        if (has(sec, "force_limit_n")) cfg.force_control.force_limit_n = asDouble(sec["force_limit_n"], "force_control.force_limit_n");
        if (has(sec, "backoff_gain_m_s_per_n")) cfg.force_control.backoff_gain_m_s_per_n = asDouble(sec["backoff_gain_m_s_per_n"], "force_control.backoff_gain_m_s_per_n");
        if (has(sec, "backoff_max_velocity_m_s")) cfg.force_control.backoff_max_velocity_m_s = asDouble(sec["backoff_max_velocity_m_s"], "force_control.backoff_max_velocity_m_s");
        if (has(sec, "blockwise_release_recenter")) cfg.force_control.blockwise_release_recenter = asBool(sec["blockwise_release_recenter"], "force_control.blockwise_release_recenter");
        if (has(sec, "max_dt_sec")) cfg.force_control.max_dt_sec = asDouble(sec["max_dt_sec"], "force_control.max_dt_sec");
        if (has(sec, "max_pos_offset_m")) cfg.force_control.max_pos_offset_m = asDouble(sec["max_pos_offset_m"], "force_control.max_pos_offset_m");
        if (has(sec, "max_rot_offset_rad")) cfg.force_control.max_rot_offset_rad = asDouble(sec["max_rot_offset_rad"], "force_control.max_rot_offset_rad");
        if (has(sec, "max_linear_velocity_m_s")) cfg.force_control.max_linear_velocity_m_s = asDouble(sec["max_linear_velocity_m_s"], "force_control.max_linear_velocity_m_s");
        if (has(sec, "max_angular_velocity_rad_s")) cfg.force_control.max_angular_velocity_rad_s = asDouble(sec["max_angular_velocity_rad_s"], "force_control.max_angular_velocity_rad_s");
        if (has(sec, "max_linear_acceleration_m_s2")) cfg.force_control.max_linear_acceleration_m_s2 = asDouble(sec["max_linear_acceleration_m_s2"], "force_control.max_linear_acceleration_m_s2");
        if (has(sec, "max_angular_acceleration_rad_s2")) cfg.force_control.max_angular_acceleration_rad_s2 = asDouble(sec["max_angular_acceleration_rad_s2"], "force_control.max_angular_acceleration_rad_s2");
        if (has(sec, "max_linear_jerk_m_s3")) cfg.force_control.max_linear_jerk_m_s3 = asDouble(sec["max_linear_jerk_m_s3"], "force_control.max_linear_jerk_m_s3");
        if (has(sec, "max_angular_jerk_rad_s3")) cfg.force_control.max_angular_jerk_rad_s3 = asDouble(sec["max_angular_jerk_rad_s3"], "force_control.max_angular_jerk_rad_s3");
        if (has(sec, "max_pos_step_m")) cfg.force_control.max_pos_step_m = asDouble(sec["max_pos_step_m"], "force_control.max_pos_step_m");
        if (has(sec, "max_rot_step_rad")) cfg.force_control.max_rot_step_rad = asDouble(sec["max_rot_step_rad"], "force_control.max_rot_step_rad");
        if (has(sec, "max_energy_j")) cfg.force_control.max_energy_j = asDouble(sec["max_energy_j"], "force_control.max_energy_j");
    }

    if (has(root, "force_torque")) {
        const YAML::Node sec = root["force_torque"];
        validateAllowedKeys(
            sec,
            {"source", "payload_identification", "left", "right"},
            "force_torque"
        );
        if (has(sec, "source")) {
            cfg.force_torque.source = lower(asString(sec["source"], "force_torque.source"));
        }
        if (has(sec, "payload_identification")) {
            const YAML::Node profile = sec["payload_identification"];
            const std::string path = "force_torque.payload_identification";
            validateAllowedKeys(profile, {
                "enable",
                "observation_model",
                "wrench_convention",
                "min_poses",
                "arrival_tolerance_deg",
                "settle_sec",
                "samples_per_pose",
                "max_force_stddev_n",
                "max_torque_stddev_nm",
                "max_force_fit_rms_n",
                "max_torque_fit_rms_nm",
                "max_design_condition_number",
            }, path);
            auto& out = cfg.force_torque.payload_identification;
            if (has(profile, "enable")) out.enable = asBool(profile["enable"], path + ".enable");
            if (has(profile, "observation_model")) out.observation_model = lower(asString(profile["observation_model"], path + ".observation_model"));
            if (has(profile, "wrench_convention")) out.wrench_convention = lower(asString(profile["wrench_convention"], path + ".wrench_convention"));
            if (has(profile, "min_poses")) out.min_poses = asInt(profile["min_poses"], path + ".min_poses");
            if (has(profile, "arrival_tolerance_deg")) out.arrival_tolerance_deg = asDouble(profile["arrival_tolerance_deg"], path + ".arrival_tolerance_deg");
            if (has(profile, "settle_sec")) out.settle_sec = asDouble(profile["settle_sec"], path + ".settle_sec");
            if (has(profile, "samples_per_pose")) out.samples_per_pose = asInt(profile["samples_per_pose"], path + ".samples_per_pose");
            if (has(profile, "max_force_stddev_n")) out.max_force_stddev_n = asDouble(profile["max_force_stddev_n"], path + ".max_force_stddev_n");
            if (has(profile, "max_torque_stddev_nm")) out.max_torque_stddev_nm = asDouble(profile["max_torque_stddev_nm"], path + ".max_torque_stddev_nm");
            if (has(profile, "max_force_fit_rms_n")) out.max_force_fit_rms_n = asDouble(profile["max_force_fit_rms_n"], path + ".max_force_fit_rms_n");
            if (has(profile, "max_torque_fit_rms_nm")) out.max_torque_fit_rms_nm = asDouble(profile["max_torque_fit_rms_nm"], path + ".max_torque_fit_rms_nm");
            if (has(profile, "max_design_condition_number")) out.max_design_condition_number = asDouble(profile["max_design_condition_number"], path + ".max_design_condition_number");
        }
        const auto parse_ft_arm = [&](
            const YAML::Node& ft,
            FtWrenchPipelineConfig& out,
            const std::string& path
        ) {
            validateAllowedKeys(ft, {
                "enable",
                "frame_configured",
                "sensor_identity",
                "calibration_id",
                "freshness_source",
                "max_sample_age_sec",
                "max_source_stall_sec",
                "control_lpf_alpha",
                "inertial_compensation_enable",
                "inertial_effective_mass_kg",
                "inertial_accel_lpf_alpha",
                "max_tcp_speed_m_s",
                "max_tcp_accel_m_s2",
                "auto_tare_after_init_motion",
                "auto_tare_settle_sec",
                "residual_tare_min_samples",
                "residual_tare_max_force_stddev_n",
                "residual_tare_max_torque_stddev_nm",
                "T_tcp_sensor",
                "sensor_bias",
                "gravity_compensation_model",
                "gravity_compensation_calibration_id",
                "gravity_force_matrix_n_per_m_s2",
                "gravity_torque_matrix_nm_per_m_s2",
                "payload_mass_kg",
                "payload_com_tcp_m",
                "residual_tare_tcp",
            }, path);
            if (has(ft, "enable")) out.enable = asBool(ft["enable"], path + ".enable");
            if (has(ft, "frame_configured")) out.frame_configured = asBool(ft["frame_configured"], path + ".frame_configured");
            if (has(ft, "sensor_identity")) out.sensor_identity = asString(ft["sensor_identity"], path + ".sensor_identity");
            if (has(ft, "calibration_id")) out.calibration_id = asString(ft["calibration_id"], path + ".calibration_id");
            if (has(ft, "freshness_source")) out.freshness_source = lower(asString(ft["freshness_source"], path + ".freshness_source"));
            if (has(ft, "max_sample_age_sec")) out.max_sample_age_sec = asDouble(ft["max_sample_age_sec"], path + ".max_sample_age_sec");
            if (has(ft, "max_source_stall_sec")) out.max_source_stall_sec = asDouble(ft["max_source_stall_sec"], path + ".max_source_stall_sec");
            if (has(ft, "control_lpf_alpha")) out.control_lpf_alpha = asDouble(ft["control_lpf_alpha"], path + ".control_lpf_alpha");
            if (has(ft, "inertial_compensation_enable")) out.inertial_compensation_enable = asBool(ft["inertial_compensation_enable"], path + ".inertial_compensation_enable");
            if (has(ft, "inertial_effective_mass_kg")) out.inertial_effective_mass_kg = asDouble(ft["inertial_effective_mass_kg"], path + ".inertial_effective_mass_kg");
            if (has(ft, "inertial_accel_lpf_alpha")) out.inertial_accel_lpf_alpha = asDouble(ft["inertial_accel_lpf_alpha"], path + ".inertial_accel_lpf_alpha");
            if (has(ft, "max_tcp_speed_m_s")) out.max_tcp_speed_m_s = asDouble(ft["max_tcp_speed_m_s"], path + ".max_tcp_speed_m_s");
            if (has(ft, "max_tcp_accel_m_s2")) out.max_tcp_accel_m_s2 = asDouble(ft["max_tcp_accel_m_s2"], path + ".max_tcp_accel_m_s2");
            if (has(ft, "auto_tare_after_init_motion")) out.auto_tare_after_init_motion = asBool(ft["auto_tare_after_init_motion"], path + ".auto_tare_after_init_motion");
            if (has(ft, "auto_tare_settle_sec")) out.auto_tare_settle_sec = asDouble(ft["auto_tare_settle_sec"], path + ".auto_tare_settle_sec");
            if (has(ft, "residual_tare_min_samples")) out.residual_tare_min_samples = asInt(ft["residual_tare_min_samples"], path + ".residual_tare_min_samples");
            if (has(ft, "residual_tare_max_force_stddev_n")) out.residual_tare_max_force_stddev_n = asDouble(ft["residual_tare_max_force_stddev_n"], path + ".residual_tare_max_force_stddev_n");
            if (has(ft, "residual_tare_max_torque_stddev_nm")) out.residual_tare_max_torque_stddev_nm = asDouble(ft["residual_tare_max_torque_stddev_nm"], path + ".residual_tare_max_torque_stddev_nm");
            if (has(ft, "T_tcp_sensor")) out.t_tcp_sensor = parsePose6D(ft["T_tcp_sensor"], path + ".T_tcp_sensor");
            if (has(ft, "sensor_bias")) out.sensor_bias = parseWrench6D(ft["sensor_bias"], path + ".sensor_bias");
            if (has(ft, "gravity_compensation_model")) out.gravity_compensation_model = lower(asString(ft["gravity_compensation_model"], path + ".gravity_compensation_model"));
            if (has(ft, "gravity_compensation_calibration_id")) out.gravity_compensation_calibration_id = asString(ft["gravity_compensation_calibration_id"], path + ".gravity_compensation_calibration_id");
            if (has(ft, "gravity_force_matrix_n_per_m_s2")) {
                out.gravity_force_matrix_n_per_m_s2 = parseMatrix3RowMajor(ft["gravity_force_matrix_n_per_m_s2"], path + ".gravity_force_matrix_n_per_m_s2");
                out.gravity_force_matrix_configured = true;
            }
            if (has(ft, "gravity_torque_matrix_nm_per_m_s2")) {
                out.gravity_torque_matrix_nm_per_m_s2 = parseMatrix3RowMajor(ft["gravity_torque_matrix_nm_per_m_s2"], path + ".gravity_torque_matrix_nm_per_m_s2");
                out.gravity_torque_matrix_configured = true;
            }
            if (has(ft, "payload_mass_kg")) out.payload_mass_kg = asDouble(ft["payload_mass_kg"], path + ".payload_mass_kg");
            if (has(ft, "payload_com_tcp_m")) out.payload_com_tcp_m = parseVec3(ft["payload_com_tcp_m"], path + ".payload_com_tcp_m");
            if (has(ft, "residual_tare_tcp")) out.residual_tare_tcp = parseWrench6D(ft["residual_tare_tcp"], path + ".residual_tare_tcp");
        };
        if (has(sec, "left")) parse_ft_arm(sec["left"], cfg.force_torque.left, "force_torque.left");
        if (has(sec, "right")) parse_ft_arm(sec["right"], cfg.force_torque.right, "force_torque.right");
    }

    if (has(root, "cartesian_control")) {
        const YAML::Node sec = root["cartesian_control"];
        validateAllowedKeys(sec, {
            "enable",
            "allow_in_simulation",
            "allow_in_real",
            "allow_in_controller_simulation",
            "warn_ik_duration_us",
            "fail_ik_duration_us",
            "path_kp",
            "path_kp_pos",
            "path_kp_ori",
            "velocity_damping",
            "max_linear_move_speed_m_s",
            "max_angular_move_speed_rad_s",
            "max_cartesian_step_m",
            "max_cartesian_step_rad",
            "exceed_limit_policy",
            "controller_simulation_servo_state_source",
            "controller_simulation_divergence_source",
            "linear_move",
            "pose_track_smd",
            "ruckig_follower",
            "tcp_pose_target_profile_default",
            "tcp_pose_target_profiles",
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
        if (has(sec, "allow_in_controller_simulation")) {
            cfg.cartesian_control.allow_in_controller_simulation =
                asBool(sec["allow_in_controller_simulation"], "cartesian_control.allow_in_controller_simulation");
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
        if (has(sec, "velocity_damping")) {
            cfg.cartesian_control.velocity_damping = asDouble(sec["velocity_damping"], "cartesian_control.velocity_damping");
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
        if (has(sec, "controller_simulation_servo_state_source")) {
            cfg.cartesian_control.controller_simulation_servo_state_source =
                parseCartesianControllerSimulationStateSource(
                    sec["controller_simulation_servo_state_source"],
                    "cartesian_control.controller_simulation_servo_state_source"
                );
        }
        if (has(sec, "controller_simulation_divergence_source")) {
            cfg.cartesian_control.controller_simulation_divergence_source =
                parseCartesianControllerSimulationStateSource(
                    sec["controller_simulation_divergence_source"],
                    "cartesian_control.controller_simulation_divergence_source"
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
                "collision_free",
                "collision_check_samples",
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
            if (has(linear, "collision_free")) {
                cfg.cartesian_control.linear_move.collision_free =
                    asBool(linear["collision_free"], "cartesian_control.linear_move.collision_free");
            }
            if (has(linear, "collision_check_samples")) {
                cfg.cartesian_control.linear_move.collision_check_samples =
                    asInt(linear["collision_check_samples"], "cartesian_control.linear_move.collision_check_samples");
            }
        }
        if (has(sec, "pose_track_smd")) {
            parsePoseTrackSmdConfig(
                sec["pose_track_smd"],
                "cartesian_control.pose_track_smd",
                &cfg.cartesian_control.pose_track_smd
            );
        }
        if (has(sec, "ruckig_follower")) {
            parseRuckigFollowerConfig(
                sec["ruckig_follower"],
                "cartesian_control.ruckig_follower",
                &cfg.cartesian_control.ruckig_follower
            );
        }
        if (has(sec, "tcp_pose_target_profile_default")) {
            cfg.cartesian_control.tcp_pose_target_profile_default =
                asString(sec["tcp_pose_target_profile_default"], "cartesian_control.tcp_pose_target_profile_default");
        }
        if (has(sec, "tcp_pose_target_profiles")) {
            const YAML::Node profiles = sec["tcp_pose_target_profiles"];
            requireMapping(profiles, "cartesian_control.tcp_pose_target_profiles");
            cfg.cartesian_control.tcp_pose_target_profiles.clear();
            for (const auto& item : profiles) {
                if (!item.first.IsScalar()) {
                    fail("cartesian_control.tcp_pose_target_profiles contains a non-scalar profile name", item.first);
                }
                const std::string name = item.first.as<std::string>();
                const YAML::Node profile_node = item.second;
                validateAllowedKeys(profile_node, {
                    "pose_track_smd",
                    "ruckig_follower",
                    "max_smd_goal_lead_m",
                    "max_smd_goal_lead_rad",
                }, "cartesian_control.tcp_pose_target_profiles." + name);
                TcpPoseTargetProfileConfig profile;
                profile.name = name;
                profile.pose_track_smd = cfg.cartesian_control.pose_track_smd;
                profile.ruckig_follower = cfg.cartesian_control.ruckig_follower;
                if (has(profile_node, "pose_track_smd")) {
                    parsePoseTrackSmdConfig(
                        profile_node["pose_track_smd"],
                        "cartesian_control.tcp_pose_target_profiles." + name + ".pose_track_smd",
                        &profile.pose_track_smd
                    );
                }
                if (has(profile_node, "ruckig_follower")) {
                    parseRuckigFollowerConfig(
                        profile_node["ruckig_follower"],
                        "cartesian_control.tcp_pose_target_profiles." + name + ".ruckig_follower",
                        &profile.ruckig_follower
                    );
                }
                if (has(profile_node, "max_smd_goal_lead_m")) {
                    profile.max_smd_goal_lead_m = asDouble(
                        profile_node["max_smd_goal_lead_m"],
                        "cartesian_control.tcp_pose_target_profiles." + name + ".max_smd_goal_lead_m"
                    );
                }
                if (has(profile_node, "max_smd_goal_lead_rad")) {
                    profile.max_smd_goal_lead_rad = asDouble(
                        profile_node["max_smd_goal_lead_rad"],
                        "cartesian_control.tcp_pose_target_profiles." + name + ".max_smd_goal_lead_rad"
                    );
                }
                cfg.cartesian_control.tcp_pose_target_profiles.push_back(profile);
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
                "singular_region_eps",
                "damping_max",
                "max_solution_jump_deg",
                "branch_jump_damping_scale",
                "branch_jump_max_retries",
                "branch_jump_clamp_to_seed",
                "branch_jump_rate_limit",
            }, "kinematics.ik");
            if (has(ik, "enable")) cfg.kinematics.ik.enable = asBool(ik["enable"], "kinematics.ik.enable");
            if (has(ik, "max_iterations")) cfg.kinematics.ik.max_iterations = asInt(ik["max_iterations"], "kinematics.ik.max_iterations");
            if (has(ik, "timeout_ms")) cfg.kinematics.ik.timeout_ms = asDouble(ik["timeout_ms"], "kinematics.ik.timeout_ms");
            if (has(ik, "damping")) cfg.kinematics.ik.damping = asDouble(ik["damping"], "kinematics.ik.damping");
            if (has(ik, "position_tolerance_m")) cfg.kinematics.ik.position_tolerance_m = asDouble(ik["position_tolerance_m"], "kinematics.ik.position_tolerance_m");
            if (has(ik, "orientation_tolerance_rad")) cfg.kinematics.ik.orientation_tolerance_rad = asDouble(ik["orientation_tolerance_rad"], "kinematics.ik.orientation_tolerance_rad");
            if (has(ik, "max_step_deg")) cfg.kinematics.ik.max_step_deg = parseJointArray(ik["max_step_deg"], "kinematics.ik.max_step_deg");
            if (has(ik, "singular_region_eps")) cfg.kinematics.ik.singular_region_eps = asDouble(ik["singular_region_eps"], "kinematics.ik.singular_region_eps");
            if (has(ik, "damping_max")) cfg.kinematics.ik.damping_max = asDouble(ik["damping_max"], "kinematics.ik.damping_max");
            if (has(ik, "max_solution_jump_deg")) cfg.kinematics.ik.max_solution_jump_deg = asDouble(ik["max_solution_jump_deg"], "kinematics.ik.max_solution_jump_deg");
            if (has(ik, "branch_jump_damping_scale")) cfg.kinematics.ik.branch_jump_damping_scale = asDouble(ik["branch_jump_damping_scale"], "kinematics.ik.branch_jump_damping_scale");
            if (has(ik, "branch_jump_max_retries")) cfg.kinematics.ik.branch_jump_max_retries = asInt(ik["branch_jump_max_retries"], "kinematics.ik.branch_jump_max_retries");
            if (has(ik, "branch_jump_clamp_to_seed")) cfg.kinematics.ik.branch_jump_clamp_to_seed = asBool(ik["branch_jump_clamp_to_seed"], "kinematics.ik.branch_jump_clamp_to_seed");
            if (has(ik, "branch_jump_rate_limit")) cfg.kinematics.ik.branch_jump_rate_limit = asBool(ik["branch_jump_rate_limit"], "kinematics.ik.branch_jump_rate_limit");
        }
    }

    if ((cfg.left_robot.run_mode == RunMode::Real || cfg.right_robot.run_mode == RunMode::Real) &&
        (!has(root, "safety") || !has(root["safety"], "tracking_error_policy"))) {
        cfg.safety.tracking_error_policy = TrackingErrorPolicy::FaultLatch;
    }

    ensureTcpPoseTargetProfiles(&cfg);
    validateConfig(cfg);

    std::cerr << "[INFO] loaded config: " << path << "\n";
    return cfg;
}

}  // namespace rb_servo
