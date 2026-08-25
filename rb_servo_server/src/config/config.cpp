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

void parseFollowerOutputSmdConfig(
    const YAML::Node& node,
    const std::string& path,
    FollowerOutputSmdConfig* out
) {
    if (!out) return;
    validateAllowedKeys(node, {
        "enable",
        "nf_linear_hz",
        "nf_angular_hz",
        "damping_ratio",
        "velocity_ff",
        "velocity_ff_lpf_hz",
    }, path);
    if (has(node, "enable")) {
        out->enable = asBool(node["enable"], path + ".enable");
    }
    if (has(node, "nf_linear_hz")) {
        out->nf_linear_hz = asDouble(node["nf_linear_hz"], path + ".nf_linear_hz");
    }
    if (has(node, "nf_angular_hz")) {
        out->nf_angular_hz = asDouble(node["nf_angular_hz"], path + ".nf_angular_hz");
    }
    if (has(node, "damping_ratio")) {
        out->damping_ratio = asDouble(node["damping_ratio"], path + ".damping_ratio");
    }
    if (has(node, "velocity_ff")) {
        out->velocity_ff = asBool(node["velocity_ff"], path + ".velocity_ff");
    }
    if (has(node, "velocity_ff_lpf_hz")) {
        out->velocity_ff_lpf_hz =
            asDouble(node["velocity_ff_lpf_hz"], path + ".velocity_ff_lpf_hz");
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
        "output_smd",
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
    if (has(node, "output_smd")) {
        parseFollowerOutputSmdConfig(
            node["output_smd"], path + ".output_smd", &out->output_smd);
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
    validatePositiveFinite(cfg.safety.ddq_max_decel_ratio, "safety.ddq_max_decel_ratio");
    if (cfg.safety.ddq_max_decel_ratio < 1.0) {
        throw std::runtime_error(
            "safety.ddq_max_decel_ratio must be >= 1.0 (deceleration may not be limited "
            "harder than acceleration; 1.0 == symmetric)");
    }
    validateNonNegativeFinite(
        cfg.safety.decel_overshoot_budget_deg, "safety.decel_overshoot_budget_deg");
    validateNonNegativeFinite(
        cfg.safety.throttle_intervention_deg_s, "safety.throttle_intervention_deg_s");
    if (cfg.safety.joint_limit_barrier.enable) {
        const auto& jb = cfg.safety.joint_limit_barrier;
        validateNonNegativeFiniteArray(jb.d_slow_deg, "safety.joint_limit_barrier.d_slow_deg");
        validatePositiveFiniteArray(jb.a_brake_deg_s2, "safety.joint_limit_barrier.a_brake_deg_s2");
        for (int i = 0; i < kDof; ++i) {
            const double lo = jb.inherit_bounds ? cfg.safety.q_min_deg[i] : jb.q_min_deg[i];
            const double hi = jb.inherit_bounds ? cfg.safety.q_max_deg[i] : jb.q_max_deg[i];
            if (!(hi > lo)) {
                throw std::runtime_error(
                    "safety.joint_limit_barrier: q_max_deg must exceed q_min_deg on every joint");
            }
            // The band must be wide enough to bleed off the commanded ceiling, or the
            // barrier engages too late to stop before the bound: d_slow >= dq_max^2/(2*a).
            const double needed =
                cfg.safety.dq_max_deg_s[i] * cfg.safety.dq_max_deg_s[i] / (2.0 * jb.a_brake_deg_s2[i]);
            if (jb.d_slow_deg[i] > 0.0 && jb.d_slow_deg[i] < needed) {
                throw std::runtime_error(
                    "safety.joint_limit_barrier.d_slow_deg[" + std::to_string(i) +
                    "] is too narrow to brake dq_max_deg_s from full speed: need >= " +
                    std::to_string(needed) + " deg");
            }
            if (!jb.inherit_bounds &&
                (jb.q_min_deg[i] < cfg.safety.q_min_deg[i] || jb.q_max_deg[i] > cfg.safety.q_max_deg[i])) {
                throw std::runtime_error(
                    "safety.joint_limit_barrier bounds must sit INSIDE safety.q_min_deg/"
                    "q_max_deg (the barrier tightens the range, it never widens it)");
            }
        }
    }
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
        validateNonNegativeFinite(ip.brake_enter_deg_s,
                                  "safety.init_motion_planner.brake_enter_deg_s");
        validateNonNegativeFinite(ip.brake_exit_deg_s,
                                  "safety.init_motion_planner.brake_exit_deg_s");
        validatePositiveFinite(ip.brake_timeout_sec,
                               "safety.init_motion_planner.brake_timeout_sec");
        validateNonNegativeFinite(ip.brake_max_travel_deg,
                                  "safety.init_motion_planner.brake_max_travel_deg");
        if (ip.brake_exit_deg_s > ip.brake_enter_deg_s) {
            throw std::runtime_error(
                "safety.init_motion_planner.brake_exit_deg_s must be <= brake_enter_deg_s "
                "(the brake phase must be able to finish)");
        }
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
    if (cfg.queue_sync.enable) {
        const QueueSyncConfig& q = cfg.queue_sync;
        // The trim is added to the control period, so a trim that could drive
        // the period to zero or negative would desynchronise the loop from its
        // own cadence. Bound it against the configured rate, fail-closed.
        const double period_us = cfg.servo.rate_hz > 0 ? 1e6 / cfg.servo.rate_hz : 0.0;
        if (period_us <= 0.0) {
            throw std::runtime_error("queue_sync.enable requires a positive servo.rate_hz");
        }
        if (q.target_fill < 1) {
            throw std::runtime_error("queue_sync.target_fill must be >= 1 (0 starves the box)");
        }
        if (q.protect_fill < 0 || q.protect_fill >= q.target_fill) {
            throw std::runtime_error("queue_sync.protect_fill must be in [0, target_fill)");
        }
        if (!(q.lpf_alpha > 0.0 && q.lpf_alpha <= 1.0)) {
            throw std::runtime_error("queue_sync.lpf_alpha must be in (0, 1]");
        }
        if (q.kp_above_us < 0.0 || q.kp_below_us < 0.0 || q.ki_us < 0.0) {
            throw std::runtime_error("queue_sync gains must be non-negative (sign is applied by the law)");
        }
        if (q.protect_adj_us > 0.0) {
            throw std::runtime_error("queue_sync.protect_adj_us must be <= 0 (it shortens the period)");
        }
        if (q.drain_adj_us <= 0.0 || q.drain_max_us < q.drain_adj_us) {
            throw std::runtime_error("queue_sync requires 0 < drain_adj_us <= drain_max_us");
        }
        const double most_negative = std::min(q.protect_adj_us, -q.adj_clamp_us);
        if (period_us + most_negative <= 0.0) {
            throw std::runtime_error(
                "queue_sync trim bounds can drive the control period to <= 0 at servo.rate_hz");
        }
        if (q.stall_cycles < 1 || q.redrain_fill_margin < 1 || q.highwater_fill <= q.target_fill) {
            throw std::runtime_error("queue_sync stall_cycles/redrain_fill_margin/highwater_fill are out of range");
        }
    }
    // Below ~2 periods a worker-cached read is stale by construction (its age is
    // the inter-thread phase offset); above ~10 a dead link goes unnoticed for
    // 20 ms at 500 Hz. Fail closed outside that band rather than silently
    // accepting a value that disables the staleness check.
    if (!(cfg.servo.worker_state_max_age_periods >= 2.0 &&
          cfg.servo.worker_state_max_age_periods <= 10.0)) {
        throw std::runtime_error(
            "servo.worker_state_max_age_periods must be in [2, 10] control periods");
    }
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
    // Worker RT scheduling. 0 = off; otherwise the same [1, 99] band as the loop.
    if (cfg.servo.worker_realtime_priority != 0 &&
        (cfg.servo.worker_realtime_priority < 1 || cfg.servo.worker_realtime_priority > 99)) {
        throw std::runtime_error(
            "servo.worker_realtime_priority must be 0 (off) or in [1, 99]");
    }
    // A worker pinned onto the loop's own core would contend with the thing it is
    // supposed to feed, on a core chosen precisely so nothing else runs there.
    // Refuse rather than let a plausible-looking config quietly serialise them.
    for (const auto& [core, name] : {
             std::pair<int, const char*>{cfg.servo.worker_cpu_core_left, "servo.worker_cpu_core_left"},
             std::pair<int, const char*>{cfg.servo.worker_cpu_core_right, "servo.worker_cpu_core_right"}}) {
        if (core >= 0 && cfg.servo.cpu_core >= 0 && core == cfg.servo.cpu_core) {
            throw std::runtime_error(
                std::string(name) + " must not equal servo.cpu_core (the loop's core)");
        }
    }
    if (cfg.servo.worker_cpu_core_left >= 0 &&
        cfg.servo.worker_cpu_core_left == cfg.servo.worker_cpu_core_right) {
        throw std::runtime_error(
            "servo.worker_cpu_core_left and _right must differ: the two arms' cadences "
            "are independent and sharing a core reintroduces the contention pinning removes");
    }
    // Pinning without FIFO is the trap this setting exists to avoid: the worker
    // gets a dedicated core but still yields to any CFS thread that lands there.
    if ((cfg.servo.worker_cpu_core_left >= 0 || cfg.servo.worker_cpu_core_right >= 0) &&
        cfg.servo.worker_realtime_priority == 0) {
        throw std::runtime_error(
            "servo.worker_cpu_core_* requires servo.worker_realtime_priority > 0");
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

    // ---- F/T + force control ------------------------------------------------
    {
        const auto validate_ft_arm = [](const FtArmConfig& a, const std::string& path) {
            if (!a.enable) return;
            // THE AXIS TRIAD MUST BE A BASIS. A degenerate triad silently collapses
            // one measured direction onto another, which reads as a plausible wrench
            // that is missing an axis - the worst failure shape there is.
            Eigen::Matrix3d m;
            m.col(0) = Eigen::Vector3d(a.axis_fx[0], a.axis_fx[1], a.axis_fx[2]);
            m.col(1) = Eigen::Vector3d(a.axis_fy[0], a.axis_fy[1], a.axis_fy[2]);
            m.col(2) = Eigen::Vector3d(a.axis_fz[0], a.axis_fz[1], a.axis_fz[2]);
            const double det = m.determinant();
            if (!std::isfinite(det) || std::abs(det) < 0.5) {
                throw std::runtime_error(
                    path + ".axes is degenerate (|det| = " + std::to_string(std::abs(det)) +
                    ") - the three rows must be orthonormal sensor axes in flange coordinates");
            }
            // det < 0 is NOT an error. The RFT64 on this cell reports in a LEFT-HANDED
            // axis set and its measured triad has det = -1; refusing that would refuse
            // the calibration.
            for (int i = 0; i < 3; ++i) {
                validateNonNegativeFinite(a.deadzone_force_n[i], path + ".deadzone_force_n");
                validateNonNegativeFinite(a.deadzone_torque_nm[i], path + ".deadzone_torque_nm");
            }
            validatePositiveFinite(a.tool_load_tau_s, path + ".tool_load_tau_s");
            validateNonNegativeFinite(a.tool_mass_kg, path + ".tool_mass_kg");
            validatePositiveFinite(a.liveness_window_sec, path + ".liveness_window_sec");
            validateNonNegativeFinite(a.liveness_min_force_pp_n, path + ".liveness_min_force_pp_n");
            validateNonNegativeFinite(a.liveness_min_torque_pp_nm, path + ".liveness_min_torque_pp_nm");
            // A tool with mass but no CoM is a torque error the force law cannot see.
            // Zero CoM with zero mass is fine (no tool); zero CoM with a real mass is
            // a value nobody entered.
            if (a.tool_mass_kg > 1e-6) {
                const double com = std::abs(a.tool_com_mm[0]) + std::abs(a.tool_com_mm[1]) +
                                   std::abs(a.tool_com_mm[2]);
                if (com < 1e-9) {
                    throw std::runtime_error(
                        path + ".tool_com_mm is zero with a non-zero tool_mass_kg - enter the "
                               "measured centre of mass (controller-manager's `ft identify` "
                               "result) or set the mass to zero");
                }
            }
        };
        validate_ft_arm(cfg.force_torque.left, "force_torque.left");
        validate_ft_arm(cfg.force_torque.right, "force_torque.right");

        const ForceControlConfig& fc = cfg.force_control;
        if (fc.enable) {
            // Force control without a wrench is a law reading zeros, which converges
            // to "no contact anywhere" and never says so.
            if (!cfg.force_torque.enable ||
                !(cfg.force_torque.left.enable || cfg.force_torque.right.enable)) {
                throw std::runtime_error(
                    "force_control.enable requires force_torque.enable and at least one arm's "
                    "force_torque.<arm>.enable - a force law with no sensor reads zeros forever");
            }
            // The overlay composes a Cartesian deviation, so it needs the Cartesian
            // path to exist at all.
            if (!cfg.kinematics.enable) {
                throw std::runtime_error("force_control.enable requires kinematics.enable");
            }
            bool stream_spring = false;
            const auto validate_axes = [&](const std::array<ForceAxisConfig, 3>& axes,
                                           const std::string& path, bool* spring) {
                for (std::size_t i = 0; i < 3; ++i) {
                    const ForceAxisConfig& a = axes[i];
                    const std::string ap = path + "[" + std::to_string(i) + "]";
                    if (a.mode == ForceAxisMode::Rigid) continue;
                    if (!(a.m > 0.0)) continue;   // m <= 0 IS the rigid spelling
                    validatePositiveFinite(a.m, ap + ".m");
                    validateNonNegativeFinite(a.b, ap + ".b");
                    validateNonNegativeFinite(a.k, ap + ".k");
                    validateNonNegativeFinite(std::abs(a.ref_force), ap + ".ref_force");
                    // SEMI-IMPLICIT EULER: b < m/dt is the no-per-tick-oscillation
                    // limit and b < 2m/dt diverges outright. Refuse rather than let a
                    // retune walk into an oscillation that looks like contact chatter.
                    const double b_limit = a.m / 0.002;
                    if (a.b >= b_limit) {
                        throw std::runtime_error(
                            ap + ".b (" + std::to_string(a.b) + ") is at or above the discrete "
                            "stability limit m/dt = " + std::to_string(b_limit) +
                            " - the integrator oscillates every tick at this damping");
                    }
                    if (a.k > 0.0 && spring != nullptr) *spring = true;
                }
            };
            validate_axes(fc.stream.translation, "force_control.stream.translation", &stream_spring);
            validate_axes(fc.stream.rotation, "force_control.stream.rotation", &stream_spring);
            // The HOLD law is checked for well-formedness but NOT for the gate
            // pairing below: a compliant Hold has no advancing plan for a gate to
            // hold back, so its stiffness IS the bound on the force and k > 0 with
            // no gate is not only legal there, it is the only safe shape.
            validate_axes(fc.hold.translation, "force_control.hold.translation", nullptr);
            validate_axes(fc.hold.rotation, "force_control.hold.rotation", nullptr);
            if (fc.hold_compliance) {
                bool hold_spring = false;
                for (int i = 0; i < 3; ++i) {
                    if (fc.hold.translation[i].k > 0.0 || fc.hold.rotation[i].k > 0.0) hold_spring = true;
                }
                if (!hold_spring) {
                    throw std::runtime_error(
                        "force_control.hold_compliance is on but every force_control.hold "
                        "stiffness is 0 - the gate does not act on a Hold (there is no plan "
                        "advance to attenuate), so nothing would bound how far a hand can "
                        "push the arm");
                }
            }

            // *** THE TWO HALVES MAY NOT SHIP APART. *** Measured by controller-manager
            // on this hardware:
            //   k > 0 with no gate  -> the contact force ramps forever (961 N in 40 s)
            //   the gate with k = 0 -> the force is bounded but the deviation is not
            //                          (9.5 m of offset in 300 s)
            // Neither is a tuning mistake to be discovered on the robot.
            if (stream_spring && !fc.gate_enable) {
                throw std::runtime_error(
                    "force_control.stream has a stiffness (k > 0) but force_gate.enable is false - the "
                    "spring ramps the contact force without bound (measured 961 N in 40 s). "
                    "Ship the gate with the spring, or set every k to 0");
            }
            if (fc.gate_enable && !stream_spring) {
                throw std::runtime_error(
                    "force_control.force_gate is enabled but every force_control.stream k is 0 - the gate bounds the "
                    "force and nothing bounds the deviation (measured 9.5 m in 300 s). Give the "
                    "compliance axes a stiffness, or disable the gate");
            }
            if (fc.gate_enable) {
                validatePositiveFinite(fc.gate_max_force_n, "force_control.force_gate.max_force_n");
                validatePositiveFinite(fc.gate_max_torque_nm, "force_control.force_gate.max_torque_nm");
                validatePositiveFinite(fc.gate_close_tau_s, "force_control.force_gate.close_tau_s");
                validatePositiveFinite(fc.gate_open_tau_s, "force_control.force_gate.open_tau_s");
                // FAST TO CLOSE, SLOW TO OPEN. Reversing them makes the gate a relay
                // against the contact and sustains a limit cycle.
                if (fc.gate_close_tau_s > fc.gate_open_tau_s) {
                    throw std::runtime_error(
                        "force_control.force_gate.close_tau_s must be <= open_tau_s - a gate that "
                        "re-opens faster than it closes is a relay against the contact");
                }
            }
            validatePositiveFinite(fc.max_velocity_m_s, "force_control.max_velocity_m_s");
            validatePositiveFinite(fc.max_acceleration_m_s2, "force_control.max_acceleration_m_s2");
            validatePositiveFinite(fc.max_velocity_rad_s, "force_control.max_velocity_rad_s");
            validatePositiveFinite(fc.max_acceleration_rad_s2, "force_control.max_acceleration_rad_s2");
            validatePositiveFinite(fc.max_state_age_sec, "force_control.max_state_age_sec");
            validateNonNegativeFinite(fc.max_deviation_m, "force_control.max_deviation_m");
            validateNonNegativeFinite(fc.max_deviation_rad, "force_control.max_deviation_rad");
        }
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
        const auto validate_output_nf = [&path](double value, const char* field) {
            if (!std::isfinite(value) || value <= 0.5 || value >= 25.0) {
                throw std::runtime_error(
                    path + ".output_smd." + field + " must be in (0.5, 25) Hz");
            }
        };
        validate_output_nf(rf.output_smd.nf_linear_hz, "nf_linear_hz");
        validate_output_nf(rf.output_smd.nf_angular_hz, "nf_angular_hz");
        if (!std::isfinite(rf.output_smd.damping_ratio) ||
            rf.output_smd.damping_ratio < 0.7 || rf.output_smd.damping_ratio > 2.0) {
            throw std::runtime_error(
                path + ".output_smd.damping_ratio must be in [0.7, 2]");
        }
        if (!std::isfinite(rf.output_smd.velocity_ff_lpf_hz) ||
            (rf.output_smd.velocity_ff_lpf_hz != 0.0 &&
             (rf.output_smd.velocity_ff_lpf_hz <= 0.5 ||
              rf.output_smd.velocity_ff_lpf_hz >= 25.0))) {
            throw std::runtime_error(
                path + ".output_smd.velocity_ff_lpf_hz must be 0 or in (0.5, 25) Hz");
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
    validateNonNegativeFinite(
        cfg.kinematics.ik.singular_step_scale_full_sigma,
        "kinematics.ik.singular_step_scale_full_sigma");
    validateNonNegativeFinite(
        cfg.kinematics.ik.singular_step_scale_floor_sigma,
        "kinematics.ik.singular_step_scale_floor_sigma");
    if (cfg.kinematics.ik.singular_step_scale_full_sigma > 0.0) {
        if (!(cfg.kinematics.ik.singular_step_scale_floor_sigma <
              cfg.kinematics.ik.singular_step_scale_full_sigma)) {
            throw std::runtime_error(
                "kinematics.ik.singular_step_scale_floor_sigma must be < "
                "singular_step_scale_full_sigma");
        }
        if (!(cfg.kinematics.ik.singular_step_scale_min > 0.0 &&
              cfg.kinematics.ik.singular_step_scale_min <= 1.0)) {
            throw std::runtime_error(
                "kinematics.ik.singular_step_scale_min must be in (0, 1] "
                "(0 would freeze the arm inside the singular region)");
        }
        if (!(cfg.kinematics.ik.max_solution_jump_deg > 0.0)) {
            throw std::runtime_error(
                "kinematics.ik.singular_step_scale_full_sigma > 0 requires "
                "max_solution_jump_deg > 0 (there is no step ceiling to scale)");
        }
    }
    if (cfg.kinematics.ik.branch_jump_max_retries < 0) {
        throw std::runtime_error("kinematics.ik.branch_jump_max_retries must be >= 0");
    }
    validateNonNegativeFinite(
        cfg.kinematics.ik.joint_limit_best_effort_position_tolerance_m,
        "kinematics.ik.joint_limit_best_effort_position_tolerance_m");
    validateNonNegativeFinite(
        cfg.kinematics.ik.max_iterations_best_effort_position_tolerance_m,
        "kinematics.ik.max_iterations_best_effort_position_tolerance_m");
    validateNonNegativeFinite(
        cfg.kinematics.ik.max_iterations_best_effort_orientation_tolerance_rad,
        "kinematics.ik.max_iterations_best_effort_orientation_tolerance_rad");
    if (cfg.kinematics.ik.max_iterations_best_effort_position_tolerance_m > 0.0 &&
        cfg.kinematics.ik.max_iterations_best_effort_position_tolerance_m <
            cfg.kinematics.ik.position_tolerance_m) {
        throw std::runtime_error(
            "kinematics.ik.max_iterations_best_effort_position_tolerance_m must be >= "
            "position_tolerance_m (a best-effort window tighter than convergence itself "
            "can never accept anything)");
    }
    validateNonNegativeFinite(
        cfg.kinematics.ik.joint_limit_best_effort_orientation_tolerance_rad,
        "kinematics.ik.joint_limit_best_effort_orientation_tolerance_rad");
    // The best-effort residual is unrealized command: never let it exceed the tolerance
    // the solver would have had to hit anyway, or a "success" could hide a real miss.
    if (cfg.kinematics.ik.joint_limit_best_effort_position_tolerance_m > 0.0 &&
        cfg.kinematics.ik.joint_limit_best_effort_position_tolerance_m <
            cfg.kinematics.ik.position_tolerance_m) {
        throw std::runtime_error(
            "kinematics.ik.joint_limit_best_effort_position_tolerance_m must be >= "
            "kinematics.ik.position_tolerance_m");
    }
    if (cfg.kinematics.ik.joint_limit_best_effort_orientation_tolerance_rad > 0.0 &&
        cfg.kinematics.ik.joint_limit_best_effort_orientation_tolerance_rad <
            cfg.kinematics.ik.orientation_tolerance_rad) {
        throw std::runtime_error(
            "kinematics.ik.joint_limit_best_effort_orientation_tolerance_rad must be >= "
            "kinematics.ik.orientation_tolerance_rad");
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
        // The servo.io_model=worker real-mode refusal was retired: per-arm worker
        // I/O is the supported path for control-box queue sync, which requires
        // each arm to own its own send cadence (one loop has a single period for
        // two boxes running two different clocks).
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
        "queue_sync",
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
            "worker_realtime_priority",
            "worker_cpu_core_left",
            "worker_cpu_core_right",
            "spin_slack_us",
            "worker_read_period_sec",
            "worker_state_max_age_periods",
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
        if (has(sec, "worker_realtime_priority")) cfg.servo.worker_realtime_priority = asInt(sec["worker_realtime_priority"], "servo.worker_realtime_priority");
        if (has(sec, "worker_cpu_core_left")) cfg.servo.worker_cpu_core_left = asInt(sec["worker_cpu_core_left"], "servo.worker_cpu_core_left");
        if (has(sec, "worker_cpu_core_right")) cfg.servo.worker_cpu_core_right = asInt(sec["worker_cpu_core_right"], "servo.worker_cpu_core_right");
        if (has(sec, "cpu_core")) cfg.servo.cpu_core = asInt(sec["cpu_core"], "servo.cpu_core");
        if (has(sec, "spin_slack_us")) cfg.servo.spin_slack_us = asInt(sec["spin_slack_us"], "servo.spin_slack_us");
        if (has(sec, "worker_read_period_sec") && has(sec, "worker_read_rate_hz")) {
            fail("servo cannot set both worker_read_period_sec and worker_read_rate_hz", sec["worker_read_rate_hz"]);
        }
        if (has(sec, "worker_state_max_age_periods")) {
            cfg.servo.worker_state_max_age_periods =
                asDouble(sec["worker_state_max_age_periods"], "servo.worker_state_max_age_periods");
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
            "ddq_max_decel_ratio",
            "decel_overshoot_budget_deg",
            "throttle_intervention_deg_s",
            "joint_limit_barrier",
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
        if (has(sec, "ddq_max_decel_ratio")) cfg.safety.ddq_max_decel_ratio = asDouble(sec["ddq_max_decel_ratio"], "safety.ddq_max_decel_ratio");
        if (has(sec, "decel_overshoot_budget_deg")) cfg.safety.decel_overshoot_budget_deg = asDouble(sec["decel_overshoot_budget_deg"], "safety.decel_overshoot_budget_deg");
        if (has(sec, "throttle_intervention_deg_s")) cfg.safety.throttle_intervention_deg_s = asDouble(sec["throttle_intervention_deg_s"], "safety.throttle_intervention_deg_s");
        if (has(sec, "joint_limit_barrier")) {
            const auto& jb = sec["joint_limit_barrier"];
            validateAllowedKeys(jb, {
                "enable", "q_min_deg", "q_max_deg", "d_slow_deg", "a_brake_deg_s2",
            }, "safety.joint_limit_barrier");
            auto& out = cfg.safety.joint_limit_barrier;
            if (has(jb, "enable")) out.enable = asBool(jb["enable"], "safety.joint_limit_barrier.enable");
            if (has(jb, "d_slow_deg")) out.d_slow_deg = parseJointArray(jb["d_slow_deg"], "safety.joint_limit_barrier.d_slow_deg");
            if (has(jb, "a_brake_deg_s2")) out.a_brake_deg_s2 = parseJointArray(jb["a_brake_deg_s2"], "safety.joint_limit_barrier.a_brake_deg_s2");
            const bool has_min = has(jb, "q_min_deg");
            const bool has_max = has(jb, "q_max_deg");
            if (has_min != has_max) {
                throw std::runtime_error(
                    "safety.joint_limit_barrier: q_min_deg and q_max_deg must be given "
                    "together (or both omitted to inherit safety.q_min_deg/q_max_deg)");
            }
            if (has_min) {
                out.q_min_deg = parseJointArray(jb["q_min_deg"], "safety.joint_limit_barrier.q_min_deg");
                out.q_max_deg = parseJointArray(jb["q_max_deg"], "safety.joint_limit_barrier.q_max_deg");
                out.inherit_bounds = false;
            }
        }
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
                "brake_before_plan",
                "brake_enter_deg_s",
                "brake_exit_deg_s",
                "brake_timeout_sec",
                "brake_max_travel_deg",
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
            if (has(ip, "brake_before_plan")) ipc.brake_before_plan = asBool(ip["brake_before_plan"], "safety.init_motion_planner.brake_before_plan");
            if (has(ip, "brake_enter_deg_s")) ipc.brake_enter_deg_s = asDouble(ip["brake_enter_deg_s"], "safety.init_motion_planner.brake_enter_deg_s");
            if (has(ip, "brake_exit_deg_s")) ipc.brake_exit_deg_s = asDouble(ip["brake_exit_deg_s"], "safety.init_motion_planner.brake_exit_deg_s");
            if (has(ip, "brake_timeout_sec")) ipc.brake_timeout_sec = asDouble(ip["brake_timeout_sec"], "safety.init_motion_planner.brake_timeout_sec");
            if (has(ip, "brake_max_travel_deg")) ipc.brake_max_travel_deg = asDouble(ip["brake_max_travel_deg"], "safety.init_motion_planner.brake_max_travel_deg");
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

    if (has(root, "queue_sync")) {
        const YAML::Node sec = root["queue_sync"];
        validateAllowedKeys(sec, {
            "enable", "target_fill", "protect_fill", "lpf_alpha",
            "kp_above_us", "kp_below_us", "ki_us", "integral_clamp_us",
            "adj_clamp_us", "protect_adj_us", "drain_adj_us", "drain_max_us",
            "drain_per_fill_us", "redrain_fill_margin", "highwater_fill",
            "warmup_min_sec", "warmup_max_sec", "drain_timeout_sec",
            "stall_cycles", "no_consumption_rise_per_sec",
        }, "queue_sync");
        QueueSyncConfig& q = cfg.queue_sync;
        if (has(sec, "enable")) q.enable = asBool(sec["enable"], "queue_sync.enable");
        if (has(sec, "target_fill")) q.target_fill = asInt(sec["target_fill"], "queue_sync.target_fill");
        if (has(sec, "protect_fill")) q.protect_fill = asInt(sec["protect_fill"], "queue_sync.protect_fill");
        if (has(sec, "lpf_alpha")) q.lpf_alpha = asDouble(sec["lpf_alpha"], "queue_sync.lpf_alpha");
        if (has(sec, "kp_above_us")) q.kp_above_us = asDouble(sec["kp_above_us"], "queue_sync.kp_above_us");
        if (has(sec, "kp_below_us")) q.kp_below_us = asDouble(sec["kp_below_us"], "queue_sync.kp_below_us");
        if (has(sec, "ki_us")) q.ki_us = asDouble(sec["ki_us"], "queue_sync.ki_us");
        if (has(sec, "integral_clamp_us")) q.integral_clamp_us = asDouble(sec["integral_clamp_us"], "queue_sync.integral_clamp_us");
        if (has(sec, "adj_clamp_us")) q.adj_clamp_us = asDouble(sec["adj_clamp_us"], "queue_sync.adj_clamp_us");
        if (has(sec, "protect_adj_us")) q.protect_adj_us = asDouble(sec["protect_adj_us"], "queue_sync.protect_adj_us");
        if (has(sec, "drain_adj_us")) q.drain_adj_us = asDouble(sec["drain_adj_us"], "queue_sync.drain_adj_us");
        if (has(sec, "drain_max_us")) q.drain_max_us = asDouble(sec["drain_max_us"], "queue_sync.drain_max_us");
        if (has(sec, "drain_per_fill_us")) q.drain_per_fill_us = asDouble(sec["drain_per_fill_us"], "queue_sync.drain_per_fill_us");
        if (has(sec, "redrain_fill_margin")) q.redrain_fill_margin = asInt(sec["redrain_fill_margin"], "queue_sync.redrain_fill_margin");
        if (has(sec, "highwater_fill")) q.highwater_fill = asInt(sec["highwater_fill"], "queue_sync.highwater_fill");
        if (has(sec, "warmup_min_sec")) q.warmup_min_sec = asDouble(sec["warmup_min_sec"], "queue_sync.warmup_min_sec");
        if (has(sec, "warmup_max_sec")) q.warmup_max_sec = asDouble(sec["warmup_max_sec"], "queue_sync.warmup_max_sec");
        if (has(sec, "drain_timeout_sec")) q.drain_timeout_sec = asDouble(sec["drain_timeout_sec"], "queue_sync.drain_timeout_sec");
        if (has(sec, "stall_cycles")) q.stall_cycles = asInt(sec["stall_cycles"], "queue_sync.stall_cycles");
        if (has(sec, "no_consumption_rise_per_sec")) q.no_consumption_rise_per_sec = asInt(sec["no_consumption_rise_per_sec"], "queue_sync.no_consumption_rise_per_sec");
    }

    if (has(root, "force_torque")) {
        const YAML::Node sec = root["force_torque"];
        validateAllowedKeys(sec, {"enable", "push_zero_payload_to_box", "left", "right"},
                            "force_torque");
        FtConfig& ft = cfg.force_torque;
        if (has(sec, "enable")) ft.enable = asBool(sec["enable"], "force_torque.enable");
        if (has(sec, "push_zero_payload_to_box")) {
            ft.push_zero_payload_to_box =
                asBool(sec["push_zero_payload_to_box"], "force_torque.push_zero_payload_to_box");
        }
        const auto parse_arm = [&](const YAML::Node& n, FtArmConfig& out, const std::string& path) {
            validateAllowedKeys(n, {
                "enable", "sensor_name", "sensor_offset_mm", "axes", "sensor_mass_kg",
                "sensor_com_mm", "deadzone_force_n", "deadzone_torque_nm", "tool_load_tau_s",
                "tool_name", "tool_xyz_mm", "tool_rpy_deg", "tool_mass_kg", "tool_com_mm",
                "applied_force_mm", "bias_force_n", "bias_torque_nm",
                "liveness_window_sec", "liveness_min_force_pp_n", "liveness_min_torque_pp_nm",
            }, path);
            if (has(n, "enable")) out.enable = asBool(n["enable"], path + ".enable");
            if (has(n, "sensor_name")) out.sensor_name = asString(n["sensor_name"], path + ".sensor_name");
            if (has(n, "sensor_offset_mm")) out.sensor_offset_mm = parseVec3(n["sensor_offset_mm"], path + ".sensor_offset_mm");
            if (has(n, "axes")) {
                const YAML::Node ax = n["axes"];
                validateAllowedKeys(ax, {"fx", "fy", "fz"}, path + ".axes");
                // ALL THREE OR NONE. A half-declared triad is a different sensor
                // orientation than the one the operator measured, and the two
                // missing rows would silently keep the identity default.
                if (!has(ax, "fx") || !has(ax, "fy") || !has(ax, "fz")) {
                    throw std::runtime_error(path + ".axes must declare fx, fy AND fz - "
                                             "a partial triad is a different sensor orientation");
                }
                out.axis_fx = parseVec3(ax["fx"], path + ".axes.fx");
                out.axis_fy = parseVec3(ax["fy"], path + ".axes.fy");
                out.axis_fz = parseVec3(ax["fz"], path + ".axes.fz");
            }
            if (has(n, "sensor_mass_kg")) out.sensor_mass_kg = asDouble(n["sensor_mass_kg"], path + ".sensor_mass_kg");
            if (has(n, "sensor_com_mm")) out.sensor_com_mm = parseVec3(n["sensor_com_mm"], path + ".sensor_com_mm");
            if (has(n, "deadzone_force_n")) out.deadzone_force_n = parseVec3(n["deadzone_force_n"], path + ".deadzone_force_n");
            if (has(n, "deadzone_torque_nm")) out.deadzone_torque_nm = parseVec3(n["deadzone_torque_nm"], path + ".deadzone_torque_nm");
            if (has(n, "tool_load_tau_s")) out.tool_load_tau_s = asDouble(n["tool_load_tau_s"], path + ".tool_load_tau_s");
            if (has(n, "tool_name")) out.tool_name = asString(n["tool_name"], path + ".tool_name");
            if (has(n, "tool_xyz_mm")) out.tool_xyz_mm = parseVec3(n["tool_xyz_mm"], path + ".tool_xyz_mm");
            if (has(n, "tool_rpy_deg")) out.tool_rpy_deg = parseVec3(n["tool_rpy_deg"], path + ".tool_rpy_deg");
            if (has(n, "tool_mass_kg")) out.tool_mass_kg = asDouble(n["tool_mass_kg"], path + ".tool_mass_kg");
            if (has(n, "tool_com_mm")) out.tool_com_mm = parseVec3(n["tool_com_mm"], path + ".tool_com_mm");
            if (has(n, "applied_force_mm")) out.applied_force_mm = parseVec3(n["applied_force_mm"], path + ".applied_force_mm");
            if (has(n, "bias_force_n")) { out.bias_force_n = parseVec3(n["bias_force_n"], path + ".bias_force_n"); out.bias_from_config = true; }
            if (has(n, "bias_torque_nm")) { out.bias_torque_nm = parseVec3(n["bias_torque_nm"], path + ".bias_torque_nm"); out.bias_from_config = true; }
            if (has(n, "liveness_window_sec")) out.liveness_window_sec = asDouble(n["liveness_window_sec"], path + ".liveness_window_sec");
            if (has(n, "liveness_min_force_pp_n")) out.liveness_min_force_pp_n = asDouble(n["liveness_min_force_pp_n"], path + ".liveness_min_force_pp_n");
            if (has(n, "liveness_min_torque_pp_nm")) out.liveness_min_torque_pp_nm = asDouble(n["liveness_min_torque_pp_nm"], path + ".liveness_min_torque_pp_nm");
        };
        if (has(sec, "left")) parse_arm(sec["left"], ft.left, "force_torque.left");
        if (has(sec, "right")) parse_arm(sec["right"], ft.right, "force_torque.right");
    }

    if (has(root, "force_control")) {
        const YAML::Node sec = root["force_control"];
        validateAllowedKeys(sec, {
            "enable", "stream", "hold", "force_gate",
            "max_deviation_m", "max_deviation_rad",
            "max_velocity_m_s", "max_acceleration_m_s2",
            "max_velocity_rad_s", "max_acceleration_rad_s2",
            "hold_compliance", "max_state_age_sec",
            "command_execution_tau_sec", "command_execution_min_rate_dps",
            "command_execution_min_ratio",
        }, "force_control");
        ForceControlConfig& fc = cfg.force_control;
        if (has(sec, "enable")) fc.enable = asBool(sec["enable"], "force_control.enable");
        const auto parse_axes = [&](const YAML::Node& n, std::array<ForceAxisConfig, 3>& out,
                                    const std::string& path) {
            if (!n.IsSequence() || n.size() != 3) {
                // ALL THREE ROWS OR NONE: a half-declared law is a different law
                // than the one the author thinks they wrote.
                fail(path + " must be a sequence of exactly 3 axis rows", n);
            }
            for (std::size_t i = 0; i < 3; ++i) {
                const std::string ap = path + "[" + std::to_string(i) + "]";
                const YAML::Node row = n[i];
                validateAllowedKeys(row, {"m", "b", "k", "mode", "ref_force"}, ap);
                ForceAxisConfig& a = out[i];
                if (has(row, "m")) a.m = asDouble(row["m"], ap + ".m");
                if (has(row, "b")) a.b = asDouble(row["b"], ap + ".b");
                if (has(row, "k")) a.k = asDouble(row["k"], ap + ".k");
                if (has(row, "ref_force")) a.ref_force = asDouble(row["ref_force"], ap + ".ref_force");
                if (has(row, "mode")) {
                    const std::string m = lower(asString(row["mode"], ap + ".mode"));
                    if (m == "compliance") a.mode = ForceAxisMode::Compliance;
                    else if (m == "force") a.mode = ForceAxisMode::Force;
                    else if (m == "rigid") a.mode = ForceAxisMode::Rigid;
                    else throw std::runtime_error(ap + ".mode must be compliance, force or rigid");
                }
            }
        };
        // TWO LAWS, NAMED FOR THEIR CONSUMER. A single unnamed block is what let the
        // streaming law be applied to an operator hand-push.
        const auto parse_law = [&](const YAML::Node& n, ForceLawConfig& out,
                                   const std::string& path) {
            validateAllowedKeys(n, {"translation", "rotation"}, path);
            if (has(n, "translation")) parse_axes(n["translation"], out.translation, path + ".translation");
            if (has(n, "rotation")) parse_axes(n["rotation"], out.rotation, path + ".rotation");
        };
        if (has(sec, "stream")) parse_law(sec["stream"], fc.stream, "force_control.stream");
        if (has(sec, "hold")) parse_law(sec["hold"], fc.hold, "force_control.hold");
        if (has(sec, "force_gate")) {
            const YAML::Node g = sec["force_gate"];
            validateAllowedKeys(g, {"enable", "max_force_n", "max_torque_nm", "close_tau_s", "open_tau_s"},
                                "force_control.force_gate");
            if (has(g, "enable")) fc.gate_enable = asBool(g["enable"], "force_control.force_gate.enable");
            if (has(g, "max_force_n")) fc.gate_max_force_n = asDouble(g["max_force_n"], "force_control.force_gate.max_force_n");
            if (has(g, "max_torque_nm")) fc.gate_max_torque_nm = asDouble(g["max_torque_nm"], "force_control.force_gate.max_torque_nm");
            if (has(g, "close_tau_s")) fc.gate_close_tau_s = asDouble(g["close_tau_s"], "force_control.force_gate.close_tau_s");
            if (has(g, "open_tau_s")) fc.gate_open_tau_s = asDouble(g["open_tau_s"], "force_control.force_gate.open_tau_s");
        }
        if (has(sec, "max_deviation_m")) fc.max_deviation_m = asDouble(sec["max_deviation_m"], "force_control.max_deviation_m");
        if (has(sec, "max_deviation_rad")) fc.max_deviation_rad = asDouble(sec["max_deviation_rad"], "force_control.max_deviation_rad");
        if (has(sec, "max_velocity_m_s")) fc.max_velocity_m_s = asDouble(sec["max_velocity_m_s"], "force_control.max_velocity_m_s");
        if (has(sec, "max_acceleration_m_s2")) fc.max_acceleration_m_s2 = asDouble(sec["max_acceleration_m_s2"], "force_control.max_acceleration_m_s2");
        if (has(sec, "max_velocity_rad_s")) fc.max_velocity_rad_s = asDouble(sec["max_velocity_rad_s"], "force_control.max_velocity_rad_s");
        if (has(sec, "max_acceleration_rad_s2")) fc.max_acceleration_rad_s2 = asDouble(sec["max_acceleration_rad_s2"], "force_control.max_acceleration_rad_s2");
        if (has(sec, "hold_compliance")) fc.hold_compliance = asBool(sec["hold_compliance"], "force_control.hold_compliance");
        if (has(sec, "max_state_age_sec")) fc.max_state_age_sec = asDouble(sec["max_state_age_sec"], "force_control.max_state_age_sec");
        if (has(sec, "command_execution_tau_sec")) fc.command_execution_tau_sec = asDouble(sec["command_execution_tau_sec"], "force_control.command_execution_tau_sec");
        if (has(sec, "command_execution_min_rate_dps")) fc.command_execution_min_rate_dps = asDouble(sec["command_execution_min_rate_dps"], "force_control.command_execution_min_rate_dps");
        if (has(sec, "command_execution_min_ratio")) fc.command_execution_min_ratio = asDouble(sec["command_execution_min_ratio"], "force_control.command_execution_min_ratio");
    }

    // ONE ANSWER, DECIDED IN ONE PLACE. The zero-payload push is a property of the
    // F/T setup, not of a backend, but the backend is what talks to the box - so it
    // is resolved here rather than branched at the use site.
    // The box is told where the TCP is, for the same reason it is told the payload:
    // not telling it leaves it believing whatever the pendant or controller-manager
    // last set. Our own motion does not depend on it (FK/IK are Pinocchio's and we
    // stream joint targets), but the box's collision detection and every Cartesian
    // number it reports do.
    const auto arm_tcp = [](const FtArmConfig& a, std::array<double, 3>* xyz,
                            std::array<double, 3>* rpy) {
        for (int i = 0; i < 3; ++i) (*xyz)[i] = a.sensor_offset_mm[i] + a.tool_xyz_mm[i];
        *rpy = a.tool_rpy_deg;
    };
    arm_tcp(cfg.force_torque.left, &cfg.left_robot.tcp_xyz_mm, &cfg.left_robot.tcp_rpy_deg);
    arm_tcp(cfg.force_torque.right, &cfg.right_robot.tcp_xyz_mm, &cfg.right_robot.tcp_rpy_deg);
    cfg.left_robot.push_tcp = cfg.force_torque.enable && cfg.force_torque.left.enable;
    cfg.right_robot.push_tcp = cfg.force_torque.enable && cfg.force_torque.right.enable;

    cfg.left_robot.push_zero_payload =
        cfg.force_torque.enable && cfg.force_torque.push_zero_payload_to_box &&
        cfg.force_torque.left.enable;
    cfg.right_robot.push_zero_payload =
        cfg.force_torque.enable && cfg.force_torque.push_zero_payload_to_box &&
        cfg.force_torque.right.enable;

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
            "flange_link",
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
        if (has(sec, "flange_link")) cfg.kinematics.flange_link = asString(sec["flange_link"], "kinematics.flange_link");
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
                "singular_step_scale_full_sigma",
                "singular_step_scale_floor_sigma",
                "singular_step_scale_min",
                "joint_limit_best_effort_position_tolerance_m",
                "joint_limit_best_effort_orientation_tolerance_rad",
                "joint_limit_track_feasible",
                "max_iterations_best_effort_position_tolerance_m",
                "max_iterations_best_effort_orientation_tolerance_rad",
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
            if (has(ik, "singular_step_scale_full_sigma")) cfg.kinematics.ik.singular_step_scale_full_sigma = asDouble(ik["singular_step_scale_full_sigma"], "kinematics.ik.singular_step_scale_full_sigma");
            if (has(ik, "singular_step_scale_floor_sigma")) cfg.kinematics.ik.singular_step_scale_floor_sigma = asDouble(ik["singular_step_scale_floor_sigma"], "kinematics.ik.singular_step_scale_floor_sigma");
            if (has(ik, "singular_step_scale_min")) cfg.kinematics.ik.singular_step_scale_min = asDouble(ik["singular_step_scale_min"], "kinematics.ik.singular_step_scale_min");
            if (has(ik, "joint_limit_best_effort_position_tolerance_m")) cfg.kinematics.ik.joint_limit_best_effort_position_tolerance_m = asDouble(ik["joint_limit_best_effort_position_tolerance_m"], "kinematics.ik.joint_limit_best_effort_position_tolerance_m");
            if (has(ik, "joint_limit_best_effort_orientation_tolerance_rad")) cfg.kinematics.ik.joint_limit_best_effort_orientation_tolerance_rad = asDouble(ik["joint_limit_best_effort_orientation_tolerance_rad"], "kinematics.ik.joint_limit_best_effort_orientation_tolerance_rad");
            if (has(ik, "joint_limit_track_feasible")) cfg.kinematics.ik.joint_limit_track_feasible = asBool(ik["joint_limit_track_feasible"], "kinematics.ik.joint_limit_track_feasible");
            if (has(ik, "max_iterations_best_effort_position_tolerance_m")) cfg.kinematics.ik.max_iterations_best_effort_position_tolerance_m = asDouble(ik["max_iterations_best_effort_position_tolerance_m"], "kinematics.ik.max_iterations_best_effort_position_tolerance_m");
            if (has(ik, "max_iterations_best_effort_orientation_tolerance_rad")) cfg.kinematics.ik.max_iterations_best_effort_orientation_tolerance_rad = asDouble(ik["max_iterations_best_effort_orientation_tolerance_rad"], "kinematics.ik.max_iterations_best_effort_orientation_tolerance_rad");
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
