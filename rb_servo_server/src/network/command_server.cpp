#include "rb_servo/network/command_server.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include "rb_servo/core/clock.hpp"

namespace rb_servo {
namespace {

struct UdpEndpoint {
    std::string host = "0.0.0.0";
    int port = 0;
};

UdpEndpoint parseUdpUri(const std::string& uri) {
    const std::string prefix = "udp://";
    if (uri.rfind(prefix, 0) != 0) {
        throw std::runtime_error("Only udp:// command_bind is supported now: " + uri);
    }
    const std::string rest = uri.substr(prefix.size());
    const auto colon = rest.rfind(':');
    if (colon == std::string::npos) {
        throw std::runtime_error("Invalid udp uri: " + uri);
    }
    UdpEndpoint ep;
    ep.host = rest.substr(0, colon);
    ep.port = std::stoi(rest.substr(colon + 1));
    return ep;
}

using json = nlohmann::json;

bool isFiniteNumber(const json& value, double* out) {
    if (!value.is_number()) return false;
    double parsed = 0.0;
    try {
        parsed = value.get<double>();
    } catch (const json::exception&) {
        return false;
    }
    if (!std::isfinite(parsed)) return false;
    if (out) *out = parsed;
    return true;
}

bool readOptionalNumber(const json& object, const char* key, double* out) {
    const auto it = object.find(key);
    if (it == object.end()) return true;
    return isFiniteNumber(*it, out);
}

bool readRequiredNumber(const json& object, const char* key, double* out) {
    const auto it = object.find(key);
    if (it == object.end()) return false;
    return isFiniteNumber(*it, out);
}

bool readOptionalBool(const json& object, const char* key, bool* out) {
    const auto it = object.find(key);
    if (it == object.end()) return true;
    if (!it->is_boolean()) return false;
    if (out) *out = it->get<bool>();
    return true;
}

bool readOptionalString(const json& object, const char* key, std::string* out) {
    const auto it = object.find(key);
    if (it == object.end()) return true;
    if (!it->is_string()) return false;
    if (out) *out = it->get<std::string>();
    return true;
}

bool readOptionalUint64(const json& object, const char* key, uint64_t* out) {
    const auto it = object.find(key);
    if (it == object.end()) return true;
    if (!it->is_number_unsigned()) return false;
    if (out) *out = it->get<uint64_t>();
    return true;
}

bool parseIpv4Address(const std::string& value, uint32_t* out) {
    in_addr addr{};
    if (::inet_pton(AF_INET, value.c_str(), &addr) != 1) return false;
    if (out) *out = ntohl(addr.s_addr);
    return true;
}

bool allowlistEntryMatches(const std::string& entry, uint32_t source_ip) {
    const auto slash = entry.find('/');
    const std::string host = slash == std::string::npos ? entry : entry.substr(0, slash);

    uint32_t network = 0;
    if (!parseIpv4Address(host, &network)) return false;

    int prefix = 32;
    if (slash != std::string::npos) {
        const std::string prefix_text = entry.substr(slash + 1);
        if (prefix_text.empty()) return false;
        for (char c : prefix_text) {
            if (c < '0' || c > '9') return false;
        }
        prefix = std::stoi(prefix_text);
        if (prefix < 0 || prefix > 32) return false;
    }

    const uint32_t mask = prefix == 0 ? 0U : (0xffffffffU << (32 - prefix));
    return (source_ip & mask) == (network & mask);
}

template <typename Target, typename Assign>
bool readOptionalArray6(const json& object, const char* key, Target* out, bool* present, Assign assign) {
    const auto it = object.find(key);
    if (present) *present = false;
    if (it == object.end()) return true;
    if (!it->is_array() || it->size() != 6) return false;

    std::array<double, 6> values{};
    for (size_t i = 0; i < values.size(); ++i) {
        if (!isFiniteNumber((*it)[i], &values[i])) return false;
    }
    assign(values, out);
    if (present) *present = true;
    return true;
}

bool readOptionalJointArray(const json& object, const char* key, JointArray* out, bool* present) {
    return readOptionalArray6(object, key, out, present, [](const std::array<double, 6>& values, JointArray* target) {
        for (int i = 0; i < kDof; ++i) {
            (*target)[i] = values[static_cast<size_t>(i)];
        }
    });
}

bool readOptionalPose6D(const json& object, const char* key, Pose6D* out, bool* present) {
    const auto it = object.find(key);
    if (present) *present = false;
    if (it == object.end()) return true;

    if (it->is_array()) {
        if (it->size() != 6) return false;
        std::array<double, 6> values{};
        for (size_t i = 0; i < values.size(); ++i) {
            if (!isFiniteNumber((*it)[i], &values[i])) return false;
        }
        if (out) {
            *out = Pose6D{values[0], values[1], values[2], values[3], values[4], values[5]};
        }
        if (present) *present = true;
        return true;
    }

    if (!it->is_object()) return false;
    const json& pose = *it;
    Pose6D parsed;
    if (!readRequiredNumber(pose, "x", &parsed.x)) return false;
    if (!readRequiredNumber(pose, "y", &parsed.y)) return false;
    if (!readRequiredNumber(pose, "z", &parsed.z)) return false;
    if (!readRequiredNumber(pose, "rx", &parsed.rx)) return false;
    if (!readRequiredNumber(pose, "ry", &parsed.ry)) return false;
    if (!readRequiredNumber(pose, "rz", &parsed.rz)) return false;

    const auto quat_it = pose.find("quaternion_xyzw");
    if (quat_it != pose.end()) {
        if (!quat_it->is_array() || quat_it->size() != 4) return false;
        std::array<double, 4> quaternion{};
        double norm2 = 0.0;
        for (size_t i = 0; i < quaternion.size(); ++i) {
            if (!isFiniteNumber((*quat_it)[i], &quaternion[i])) return false;
            norm2 += quaternion[i] * quaternion[i];
        }
        if (!std::isfinite(norm2) || norm2 <= 0.0) return false;
        parsed.quaternion_xyzw = quaternion;
    }

    if (out) *out = parsed;
    if (present) *present = true;
    return true;
}

bool readOptionalWrench6D(const json& object, const char* key, Wrench6D* out, bool* present) {
    return readOptionalArray6(object, key, out, present, [](const std::array<double, 6>& values, Wrench6D* target) {
        *target = Wrench6D{values[0], values[1], values[2], values[3], values[4], values[5]};
    });
}

bool readOptionalVec6(const json& object, const char* key, Vec6* out, bool* present) {
    return readOptionalArray6(object, key, out, present, [](const std::array<double, 6>& values, Vec6* target) {
        *target = Vec6{values[0], values[1], values[2], values[3], values[4], values[5]};
    });
}

bool parseForceControlObject(const json& object, ForceControlCommand* cmd) {
    const auto force_it = object.find("force_control");
    if (force_it == object.end()) return true;
    if (!force_it->is_object()) return false;

    const json& force = *force_it;
    std::string mode;
    if (!readOptionalString(force, "mode", &mode)) return false;
    if (!mode.empty()) cmd->mode = forceControlModeFromString(mode);

    bool present = false;
    if (!readOptionalWrench6D(force, "target_wrench", &cmd->target_wrench, &present)) return false;
    if (!readOptionalNumber(force, "max_pos_offset_m", &cmd->max_pos_offset_m)) return false;
    if (!readOptionalNumber(force, "max_rot_offset_rad", &cmd->max_rot_offset_rad)) return false;
    if (!readOptionalNumber(force, "max_pos_step_m", &cmd->max_pos_step_m)) return false;
    if (!readOptionalNumber(force, "max_rot_step_rad", &cmd->max_rot_step_rad)) return false;

    const auto axis_it = force.find("enabled_axis");
    if (axis_it != force.end()) {
        if (!axis_it->is_object()) return false;
        const json& axis = *axis_it;
        if (!readOptionalBool(axis, "x", &cmd->enabled_axis.x)) return false;
        if (!readOptionalBool(axis, "y", &cmd->enabled_axis.y)) return false;
        if (!readOptionalBool(axis, "z", &cmd->enabled_axis.z)) return false;
        if (!readOptionalBool(axis, "roll", &cmd->enabled_axis.roll)) return false;
        if (!readOptionalBool(axis, "pitch", &cmd->enabled_axis.pitch)) return false;
        if (!readOptionalBool(axis, "yaw", &cmd->enabled_axis.yaw)) return false;
    }
    return true;
}

bool parseLinearMoveOrientationMode(const std::string& value, LinearMoveOrientationMode* out) {
    std::string normalized = value;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    if (normalized == "constant") {
        if (out) *out = LinearMoveOrientationMode::Constant;
        return true;
    }
    if (normalized == "slerp") {
        if (out) *out = LinearMoveOrientationMode::Slerp;
        return true;
    }
    return false;
}

bool parseCirclePlane(const std::string& value, TcpCirclePlane* out) {
    std::string normalized = value;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    if (normalized == "xy") {
        if (out) *out = TcpCirclePlane::XY;
        return true;
    }
    if (normalized == "xz") {
        if (out) *out = TcpCirclePlane::XZ;
        return true;
    }
    if (normalized == "yz") {
        if (out) *out = TcpCirclePlane::YZ;
        return true;
    }
    return false;
}

bool parseCircleCenterMode(const std::string& value, TcpCircleCenterMode* out) {
    std::string normalized = value;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    if (normalized == "start_on_circle") {
        if (out) *out = TcpCircleCenterMode::StartOnCircle;
        return true;
    }
    return false;
}

bool parseCircleFrame(const std::string& value, TcpCircleFrame* out) {
    std::string normalized = value;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    if (normalized == "stand") {
        if (out) *out = TcpCircleFrame::Stand;
        return true;
    }
    return false;
}

bool parseArmId(const std::string& value, ArmId* out) {
    std::string normalized = value;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    if (normalized == "left" || normalized == "left_robot") {
        if (out) *out = ArmId::Left;
        return true;
    }
    if (normalized == "right" || normalized == "right_robot") {
        if (out) *out = ArmId::Right;
        return true;
    }
    return false;
}

bool parseCircleTrackTrackingSource(const std::string& value, TcpCircleTrackTrackingSource* out) {
    std::string normalized = value;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    if (normalized == "auto") {
        if (out) *out = TcpCircleTrackTrackingSource::Auto;
        return true;
    }
    if (normalized == "tcp_actual_stand" || normalized == "actual") {
        if (out) *out = TcpCircleTrackTrackingSource::TcpActualStand;
        return true;
    }
    if (normalized == "tcp_ref_stand" || normalized == "reference" || normalized == "ref") {
        if (out) *out = TcpCircleTrackTrackingSource::TcpRefStand;
        return true;
    }
    return false;
}

bool readOptionalPositiveNumber(
    const json& object,
    const char* key,
    double* out,
    bool* present
) {
    if (present) *present = false;
    const auto it = object.find(key);
    if (it == object.end()) return true;
    double value = 0.0;
    if (!isFiniteNumber(*it, &value) || value <= 0.0) return false;
    if (out) *out = value;
    if (present) *present = true;
    return true;
}

bool readRequiredPositiveNumber(
    const json& object,
    const char* key,
    double* out
) {
    bool present = false;
    if (!readOptionalPositiveNumber(object, key, out, &present)) return false;
    return present;
}

bool readRequiredNonNegativeNumber(
    const json& object,
    const char* key,
    double* out
) {
    const auto it = object.find(key);
    if (it == object.end()) return false;
    double value = 0.0;
    if (!isFiniteNumber(*it, &value) || value < 0.0) return false;
    if (out) *out = value;
    return true;
}

bool readRequiredPoint3(const json& object, const char* key, std::array<double, 3>* out) {
    const auto it = object.find(key);
    if (it == object.end()) return false;

    std::array<double, 3> values{};
    if (it->is_array()) {
        if (it->size() != values.size()) return false;
        for (size_t i = 0; i < values.size(); ++i) {
            if (!isFiniteNumber((*it)[i], &values[i])) return false;
        }
    } else if (it->is_object()) {
        const json& point = *it;
        if (!readRequiredNumber(point, "x", &values[0])) return false;
        if (!readRequiredNumber(point, "y", &values[1])) return false;
        if (!readRequiredNumber(point, "z", &values[2])) return false;
    } else {
        return false;
    }
    if (out) *out = values;
    return true;
}

bool readOptionalLinearMoveFields(const json& object, ArmCommand* out) {
    bool present = false;
    if (!readOptionalPositiveNumber(
            object,
            "duration_sec",
            &out->linear_move_duration_sec,
            &present
        )) {
        return false;
    }
    out->has_linear_move_duration = out->has_linear_move_duration || present;

    if (!readOptionalPositiveNumber(
            object,
            "linear_speed_m_s",
            &out->linear_move_linear_speed_m_s,
            &present
        )) {
        return false;
    }
    out->has_linear_move_linear_speed = out->has_linear_move_linear_speed || present;

    if (!readOptionalPositiveNumber(
            object,
            "angular_speed_rad_s",
            &out->linear_move_angular_speed_rad_s,
            &present
        )) {
        return false;
    }
    out->has_linear_move_angular_speed = out->has_linear_move_angular_speed || present;

    std::string orientation_mode;
    if (!readOptionalString(object, "orientation_mode", &orientation_mode)) return false;
    if (!orientation_mode.empty()) {
        if (!parseLinearMoveOrientationMode(orientation_mode, &out->linear_move_orientation_mode)) return false;
        out->has_linear_move_orientation_mode = true;
    }
    return true;
}

bool readOptionalCircleTrackFields(const json& object, ArmCommand* out) {
    const bool circle_track_fields_present =
        object.contains("center_stand") ||
        object.contains("radius_m") ||
        object.contains("period_sec") ||
        object.contains("repeat") ||
        object.contains("plane") ||
        object.contains("start_phase_rad") ||
        object.contains("orientation_hold") ||
        object.contains("feedback_kp_pos") ||
        object.contains("feedback_kp_ori") ||
        object.contains("max_linear_m_s") ||
        object.contains("max_angular_rad_s") ||
        object.contains("tracking_source");
    if (!circle_track_fields_present) return true;

    TcpCircleTrackCommand parsed = out->tcp_circle_track;
    if (!readRequiredPoint3(object, "center_stand", &parsed.center_stand)) return false;
    if (!readRequiredPositiveNumber(object, "radius_m", &parsed.radius_m)) return false;
    if (!readRequiredPositiveNumber(object, "period_sec", &parsed.period_sec)) return false;

    const auto repeat_it = object.find("repeat");
    if (repeat_it == object.end() || !repeat_it->is_number_integer()) return false;
    parsed.repeat = repeat_it->get<int>();
    if (parsed.repeat <= 0) return false;

    if (!readRequiredNumber(object, "start_phase_rad", &parsed.start_phase_rad)) return false;
    if (!readOptionalBool(object, "orientation_hold", &parsed.orientation_hold)) return false;
    if (!object.contains("orientation_hold")) return false;
    if (!readRequiredNonNegativeNumber(object, "feedback_kp_pos", &parsed.feedback_kp_pos)) return false;
    if (!readRequiredNonNegativeNumber(object, "feedback_kp_ori", &parsed.feedback_kp_ori)) return false;
    if (!readRequiredPositiveNumber(object, "max_linear_m_s", &parsed.max_linear_m_s)) return false;
    if (!readRequiredPositiveNumber(object, "max_angular_rad_s", &parsed.max_angular_rad_s)) return false;

    std::string value;
    if (!readOptionalString(object, "plane", &value) || value.empty()) return false;
    if (!parseCirclePlane(value, &parsed.plane)) return false;
    value.clear();
    if (!readOptionalString(object, "tracking_source", &value) || value.empty()) return false;
    if (!parseCircleTrackTrackingSource(value, &parsed.tracking_source)) return false;

    out->tcp_circle_track = parsed;
    out->has_tcp_circle_track = true;
    return true;
}

bool readOptionalCircleMoveFields(const json& object, ArmCommand* out) {
    const bool circle_fields_present =
        object.contains("diameter_m") ||
        object.contains("period_sec") ||
        object.contains("repeat") ||
        object.contains("phase_advance_sec") ||
        object.contains("plane") ||
        object.contains("center_mode") ||
        object.contains("frame");
    if (!circle_fields_present) return true;

    bool present = false;
    double number = out->tcp_circle_move.diameter_m;
    if (!readOptionalPositiveNumber(object, "diameter_m", &number, &present)) return false;
    if (present) {
        out->tcp_circle_move.diameter_m = number;
        out->has_tcp_circle_move = true;
    }
    number = out->tcp_circle_move.period_sec;
    if (!readOptionalPositiveNumber(object, "period_sec", &number, &present)) return false;
    if (present) {
        out->tcp_circle_move.period_sec = number;
        out->has_tcp_circle_move = true;
    }
    const auto phase_advance_it = object.find("phase_advance_sec");
    if (phase_advance_it != object.end()) {
        number = out->tcp_circle_move.phase_advance_sec;
        if (!isFiniteNumber(*phase_advance_it, &number)) return false;
        if (!std::isfinite(number) || number < 0.0) return false;
        out->tcp_circle_move.phase_advance_sec = number;
        out->has_tcp_circle_move = true;
    }

    const auto repeat_it = object.find("repeat");
    if (repeat_it != object.end()) {
        if (!repeat_it->is_number_integer()) return false;
        const int repeat = repeat_it->get<int>();
        if (repeat <= 0) return false;
        out->tcp_circle_move.repeat = repeat;
        out->has_tcp_circle_move = true;
    }

    std::string value;
    if (!readOptionalString(object, "plane", &value)) return false;
    if (!value.empty()) {
        if (!parseCirclePlane(value, &out->tcp_circle_move.plane)) return false;
        out->has_tcp_circle_move = true;
    }
    value.clear();
    if (!readOptionalString(object, "center_mode", &value)) return false;
    if (!value.empty()) {
        if (!parseCircleCenterMode(value, &out->tcp_circle_move.center_mode)) return false;
        out->has_tcp_circle_move = true;
    }
    value.clear();
    if (!readOptionalString(object, "orientation_mode", &value)) return false;
    if (!value.empty()) {
        if (!parseLinearMoveOrientationMode(value, &out->tcp_circle_move.orientation_mode)) return false;
        out->has_tcp_circle_move = true;
    }
    value.clear();
    if (!readOptionalString(object, "frame", &value)) return false;
    if (!value.empty()) {
        if (!parseCircleFrame(value, &out->tcp_circle_move.frame)) return false;
        out->has_tcp_circle_move = true;
    }
    if (out->has_tcp_circle_move) {
        return out->tcp_circle_move.diameter_m > 0.0 &&
               out->tcp_circle_move.period_sec > 0.0 &&
               out->tcp_circle_move.repeat > 0 &&
               out->tcp_circle_move.phase_advance_sec <=
                   0.25 * out->tcp_circle_move.period_sec + 1e-12;
    }
    return true;
}

bool requiresPayload(ControlMode mode) {
    return mode == ControlMode::JointTarget ||
           mode == ControlMode::InitMotion ||
           mode == ControlMode::JointVelocity ||
           mode == ControlMode::TcpPoseTarget ||
           mode == ControlMode::TcpLinearMove ||
           mode == ControlMode::TcpCircleMove ||
           mode == ControlMode::TcpCircleTrack ||
           mode == ControlMode::TcpDeltaStand ||
           mode == ControlMode::TcpDeltaLocal ||
           mode == ControlMode::TcpTwistStand ||
           mode == ControlMode::TcpTwistLocal;
}

bool hasRequiredPayload(const ArmCommand& command) {
    switch (command.mode) {
        case ControlMode::JointTarget:
        case ControlMode::InitMotion:
            return command.has_joint_target;
        case ControlMode::JointVelocity:
            return command.has_joint_velocity;
        case ControlMode::TcpPoseTarget:
            return command.has_tcp_target;
        case ControlMode::TcpLinearMove:
            return command.has_tcp_target &&
                   (command.has_linear_move_duration || command.has_linear_move_linear_speed);
        case ControlMode::TcpCircleMove:
            return command.has_tcp_circle_move;
        case ControlMode::TcpCircleTrack:
            return command.has_tcp_circle_track;
        case ControlMode::TcpDeltaStand:
            return command.has_tcp_delta_stand;
        case ControlMode::TcpDeltaLocal:
            return command.has_tcp_delta_local;
        case ControlMode::TcpTwistStand:
            return command.has_tcp_twist_stand;
        case ControlMode::TcpTwistLocal:
            return command.has_tcp_twist_local;
        default:
            return true;
    }
}

bool isAcquireLeaseModeString(const std::string& mode) {
    return mode == "AcquireLease" || mode == "acquire_lease" || mode == "acquirelease";
}

bool isReleaseLeaseModeString(const std::string& mode) {
    return mode == "ReleaseLease" || mode == "release_lease" || mode == "releaselease";
}

// EmergencyStop, SetSafetyFloorZ, SetSafetyFloorEnabled, SetSafetyRoiBounds and
// SetUserSafetyFloorPlane are intentionally leaseless: an operator must be able to
// stop motion or adjust
// the safety floor / ROI box / user floor plane while another client (e.g.
// policy_runner) holds the command lease. SetSafetyFloorZ is bounded server-side
// to safety.floor_constraint.[runtime_min_z_m, runtime_max_z_m]; SetSafetyRoiBounds
// to safety.roi_box.[runtime_min_m, runtime_max_m] per axis; SetUserSafetyFloorPlane
// to safety.user_floor_constraint via validateUserFloorPlaneRequest.
bool commandRequiresLease(ControlMode mode) {
    return mode == ControlMode::ArmMotion ||
           mode == ControlMode::DisarmMotion ||
           mode == ControlMode::JointTarget ||
           mode == ControlMode::InitMotion ||
           mode == ControlMode::JointVelocity ||
           mode == ControlMode::TcpPoseTarget ||
           mode == ControlMode::TcpLinearMove ||
           mode == ControlMode::TcpCircleMove ||
           mode == ControlMode::TcpCircleTrack ||
           mode == ControlMode::TcpDeltaStand ||
           mode == ControlMode::TcpDeltaLocal ||
           mode == ControlMode::TcpTwistStand ||
           mode == ControlMode::TcpTwistLocal ||
           mode == ControlMode::ResetFault ||
           mode == ControlMode::Freedrive;
}

bool dualCommandRequiresLease(const DualArmCommand& command) {
    return commandRequiresLease(command.left.mode) || commandRequiresLease(command.right.mode);
}

bool isEmergencyStopCommand(const DualArmCommand& command) {
    return command.left.mode == ControlMode::EmergencyStop || command.right.mode == ControlMode::EmergencyStop;
}

std::string sourceKey(const CommandSourceMetadata& source) {
    if (source.source_id.empty() && source.session_id.empty()) return "__legacy__";
    return source.source_id + "\n" + source.session_id;
}

bool sameSource(const CommandSourceLeaseState& lease, const CommandSourceMetadata& source) {
    return lease.source_id == source.source_id && lease.session_id == source.session_id;
}

uint64_t timeoutNs(double timeout_sec) {
    if (timeout_sec <= 0.0 || !std::isfinite(timeout_sec)) return 0;
    return static_cast<uint64_t>(timeout_sec * 1e9);
}

std::string generatedLeaseToken(const CommandSourceMetadata& source, uint64_t now_ns, uint64_t counter) {
    std::ostringstream out;
    out << "lease-" << std::hex << now_ns << "-" << counter;
    if (!source.source_id.empty()) out << "-" << source.source_id;
    return out.str();
}

bool parseArmObject(
    const json& object,
    ArmId arm_id,
    uint64_t seq,
    uint64_t receive_time_ns,
    ControlMode default_mode,
    double default_timeout_sec,
    ArmCommand* out
) {
    out->arm_id = arm_id;
    out->seq = seq;
    out->host_time_ns = receive_time_ns;
    out->mode = default_mode;
    out->timeout_sec = default_timeout_sec;

    if (!object.is_null()) {
        if (!object.is_object()) return false;

        std::string mode;
        if (!readOptionalString(object, "mode", &mode)) return false;
        if (!mode.empty()) out->mode = controlModeFromString(mode);
        std::string arm_name;
        if (!readOptionalString(object, "arm", &arm_name)) return false;
        if (!arm_name.empty()) {
            ArmId parsed_arm = arm_id;
            if (!parseArmId(arm_name, &parsed_arm) || parsed_arm != arm_id) return false;
        }

        double timeout = out->timeout_sec;
        if (!readOptionalNumber(object, "timeout_sec", &timeout)) return false;
        out->timeout_sec = timeout;

        double gripper = out->gripper_target;
        if (object.contains("gripper_target")) {
            if (!readOptionalNumber(object, "gripper_target", &gripper)) return false;
            out->gripper_target = gripper;
            out->has_gripper = true;  // open-percentage setpoint present this command
        } else if (object.contains("gripper")) {
            if (!readOptionalNumber(object, "gripper", &gripper)) return false;
            out->gripper_target = gripper;
            out->has_gripper = true;
        }

        if (object.contains("freedrive_on")) {
            if (!readOptionalBool(object, "freedrive_on", &out->freedrive_on)) return false;
            out->has_freedrive = true;
        }

        bool present = false;
        if (!readOptionalJointArray(object, "q_target_deg", &out->q_target_deg, &present)) return false;
        out->has_joint_target = present;
        if (!readOptionalJointArray(object, "dq_target_deg_s", &out->dq_target_deg_s, &present)) return false;
        out->has_joint_velocity = present;
        if (!readOptionalPose6D(object, "tcp_target_stand", &out->tcp_target_stand, &present)) return false;
        out->has_tcp_target = present;
        bool alias_present = false;
        Pose6D alias_target;
        if (!readOptionalPose6D(object, "target_tcp_stand", &alias_target, &alias_present)) return false;
        if (out->has_tcp_target && alias_present) return false;
        if (alias_present) {
            out->tcp_target_stand = alias_target;
            out->has_tcp_target = true;
        }
        if (!readOptionalPose6D(object, "tcp_delta_stand", &out->tcp_delta_stand, &present)) return false;
        out->has_tcp_delta_stand = present;
        if (!readOptionalPose6D(object, "tcp_delta_local", &out->tcp_delta_local, &present)) return false;
        out->has_tcp_delta_local = present;
        if (!readOptionalVec6(object, "tcp_twist_stand", &out->tcp_twist_stand, &present)) return false;
        out->has_tcp_twist_stand = present;
        if (!readOptionalVec6(object, "tcp_twist_local", &out->tcp_twist_local, &present)) return false;
        out->has_tcp_twist_local = present;
        if (!readOptionalVec6(object, "tcp_target_twist_stand", &out->tcp_target_twist_stand, &present)) return false;
        out->has_tcp_target_twist_stand = present;
        if (!readOptionalLinearMoveFields(object, out)) return false;
        if (out->mode == ControlMode::TcpCircleMove &&
            !readOptionalCircleMoveFields(object, out)) {
            return false;
        }
        if (out->mode == ControlMode::TcpCircleTrack &&
            !readOptionalCircleTrackFields(object, out)) {
            return false;
        }
        if (!parseForceControlObject(object, &out->force_control)) return false;
    }
    if (out->timeout_sec <= 0.0 || !std::isfinite(out->timeout_sec)) return false;
    return true;
}

}  // namespace

CommandServer::CommandServer(
    const NetworkConfig& config,
    CommandBuffer* command_buffer
) : config_(config), command_buffer_(command_buffer) {
    command_source_config_.enforce_lease = config.command_source_enforce_lease;
    command_source_config_.lease_timeout_sec = config.command_source_lease_timeout_sec;
    active_lease_.enforce_lease = command_source_config_.enforce_lease;
}

CommandServer::~CommandServer() {
    stop();
}

bool CommandServer::start() {
    if (thread_.joinable()) return running_.load();
    running_ = true;
    std::promise<bool> startup_result;
    std::future<bool> startup_ready = startup_result.get_future();
    thread_ = std::thread(&CommandServer::threadMain, this, std::move(startup_result));
    const bool started = startup_ready.get();
    if (!started) {
        running_ = false;
        if (thread_.joinable()) {
            thread_.join();
        }
        return false;
    }
    return running_.load();
}

void CommandServer::stop() {
    running_ = false;
    if (thread_.joinable()) {
        thread_.join();
    }
}

void CommandServer::threadMain(std::promise<bool> startup_result) {
    int socket_fd = -1;
    bool startup_reported = false;
    auto report_startup = [&](bool ok) {
        if (!startup_reported) {
            startup_result.set_value(ok);
            startup_reported = true;
        }
    };
    auto close_socket = [&]() {
        if (socket_fd >= 0) {
            ::close(socket_fd);
            socket_fd = -1;
        }
    };

    try {
        const UdpEndpoint ep = parseUdpUri(config_.command_bind);
        socket_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd < 0) {
            throw std::runtime_error(std::string("socket() failed: ") + std::strerror(errno));
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<uint16_t>(ep.port));
        if (::inet_pton(AF_INET, ep.host.c_str(), &addr.sin_addr) != 1) {
            throw std::runtime_error("Invalid bind host: " + ep.host);
        }
        if (::bind(socket_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            throw std::runtime_error(std::string("bind() failed: ") + std::strerror(errno));
        }

        std::cerr << "[INFO] CommandServer listening on " << config_.command_bind << "\n";
        report_startup(true);
        std::array<char, 8192> buffer{};
        while (running_) {
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(socket_fd, &fds);
            timeval tv{};
            tv.tv_sec = 0;
            tv.tv_usec = 100000;
            const int ready = ::select(socket_fd + 1, &fds, nullptr, nullptr, &tv);
            if (ready <= 0) continue;

            sockaddr_in src{};
            socklen_t src_len = sizeof(src);
            const ssize_t n = ::recvfrom(socket_fd, buffer.data(), buffer.size() - 1, 0,
                                         reinterpret_cast<sockaddr*>(&src), &src_len);
            if (n <= 0) continue;
            if (static_cast<size_t>(n) >= buffer.size() - 1) {
                std::cerr << "[WARN] command packet too large; dropped\n";
                continue;
            }
            char source_ip[INET_ADDRSTRLEN] = {};
            if (!::inet_ntop(AF_INET, &src.sin_addr, source_ip, sizeof(source_ip))) {
                std::cerr << "[WARN] command packet source could not be decoded; dropped\n";
                continue;
            }
            if (!acceptsCommandSource(source_ip)) {
                std::cerr << "[WARN] command packet from untrusted source " << source_ip << " dropped\n";
                continue;
            }
            buffer[static_cast<size_t>(n)] = '\0';
            const uint64_t receive_time_ns = nowSteadyNs();
            DualArmCommand cmd;
            bool parsed = false;
            try {
                parsed = parseMessage(std::string(buffer.data(), static_cast<size_t>(n)), receive_time_ns, &cmd);
            } catch (const std::exception& e) {
                std::cerr << "[WARN] invalid command packet: " << e.what() << "\n";
            }
            if (parsed) {
                // Lease-admin packets (AcquireLease/ReleaseLease) only mutate
                // the lease; they must not displace the buffered motion command.
                // Their lease grant/clear still has to reach the published state
                // (the lease readback is snapshot.command.lease) — otherwise an
                // acquiring client waits forever for a grant it already has.
                if (command_buffer_) {
                    if (cmd.lease_admin_only) {
                        command_buffer_->updateLease(cmd.lease, receive_time_ns);
                    } else {
                        command_buffer_->setCommand(cmd);
                    }
                }
            } else {
                std::cerr << "[WARN] command packet dropped";
                if (!last_reject_reason_.empty()) {
                    std::cerr << ": " << last_reject_reason_;
                }
                std::cerr << "\n";
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] CommandServer failed: " << e.what() << "\n";
        running_ = false;
        report_startup(false);
    }
    close_socket();
}

bool CommandServer::parseMessage(
    const std::string& message,
    uint64_t receive_time_ns,
    DualArmCommand* out_command
) {
    if (!out_command) return false;
    last_reject_reason_.clear();

    json root;
    try {
        root = json::parse(message);
    } catch (const json::exception&) {
        return false;
    }
    if (!root.is_object()) return false;

    DualArmCommand cmd;
    cmd.host_time_ns = receive_time_ns;  // authoritative timestamp for timeout checks
    cmd.lease.enforce_lease = command_source_config_.enforce_lease;

    uint64_t schema_version = 1;
    if (!readOptionalUint64(root, "schema_version", &schema_version)) return false;
    if (schema_version != 1) return false;

    if (!root.contains("seq")) return false;
    if (!readOptionalUint64(root, "seq", &cmd.seq)) return false;
    if (!readOptionalBool(root, "coupled_timeout", &cmd.coupled_timeout)) return false;
    if (!readOptionalString(root, "source_id", &cmd.source.source_id)) return false;
    if (!readOptionalString(root, "session_id", &cmd.source.session_id)) return false;
    if (!readOptionalString(root, "lease_token", &cmd.source.lease_token)) return false;
    const auto priority_it = root.find("source_priority");
    if (priority_it != root.end()) {
        if (!priority_it->is_number_integer()) return false;
        cmd.source.source_priority = priority_it->get<int>();
    }

    std::string mode_string = "Hold";
    if (!readOptionalString(root, "mode", &mode_string)) return false;
    ControlMode default_mode = ControlMode::Hold;
    const bool acquire_lease_only = isAcquireLeaseModeString(mode_string);
    const bool release_lease_only = isReleaseLeaseModeString(mode_string);
    cmd.lease_admin_only = acquire_lease_only || release_lease_only;
    try {
        if (!acquire_lease_only && !release_lease_only) {
            default_mode = controlModeFromString(mode_string);
        }
    } catch (const std::exception&) {
        return false;
    }

    double timeout_sec = config_.command_timeout_sec;
    if (timeout_sec <= 0.0 || !std::isfinite(timeout_sec)) return false;
    if (!readOptionalNumber(root, "timeout_sec", &timeout_sec)) return false;
    if (timeout_sec <= 0.0 || !std::isfinite(timeout_sec)) return false;

    // SetSafetyFloorZ payload (top-level: the floor plane is global, not per-arm).
    if (root.contains("floor_z_m")) {
        if (!readOptionalNumber(root, "floor_z_m", &cmd.floor_z_m)) return false;
        cmd.has_floor_z = true;
    }
    // SetSafetyFloorEnabled payload (top-level): runtime enforce on/off for the stand floor.
    if (root.contains("floor_enabled")) {
        if (!readOptionalBool(root, "floor_enabled", &cmd.floor_enabled)) return false;
        cmd.has_floor_enabled = true;
    }

    // SetSafetyRoiBounds payload (top-level: the ROI box is global). Both
    // roi_min_m and roi_max_m must be present together as [x, y, z] arrays of
    // finite numbers; the server bounds-checks against the runtime envelope.
    if (root.contains("roi_min_m") || root.contains("roi_max_m")) {
        const auto read_vec3 = [](const json& object, const char* key,
                                  std::array<double, 3>* out) -> bool {
            const auto it = object.find(key);
            if (it == object.end() || !it->is_array() || it->size() != 3) return false;
            for (std::size_t i = 0; i < 3; ++i) {
                double v = 0.0;
                if (!isFiniteNumber((*it)[i], &v)) return false;
                (*out)[i] = v;
            }
            return true;
        };
        if (!read_vec3(root, "roi_min_m", &cmd.roi_min_m)) return false;
        if (!read_vec3(root, "roi_max_m", &cmd.roi_max_m)) return false;
        cmd.has_roi_bounds = true;
    }

    // SetUserSafetyFloorPlane payload (top-level: the plane is global). point + normal
    // present together as [x, y, z] arrays; optional margin_m (>=0) and enable bool.
    // The server validates the plane via validateUserFloorPlaneRequest before applying;
    // an enable=false request turns the constraint off unconditionally.
    if (root.contains("user_floor_point_m") || root.contains("user_floor_normal")) {
        const auto read_vec3 = [](const json& object, const char* key,
                                  std::array<double, 3>* out) -> bool {
            const auto it = object.find(key);
            if (it == object.end() || !it->is_array() || it->size() != 3) return false;
            for (std::size_t i = 0; i < 3; ++i) {
                double v = 0.0;
                if (!isFiniteNumber((*it)[i], &v)) return false;
                (*out)[i] = v;
            }
            return true;
        };
        if (!read_vec3(root, "user_floor_point_m", &cmd.user_floor_point_m)) return false;
        if (!read_vec3(root, "user_floor_normal", &cmd.user_floor_normal)) return false;
        if (!readOptionalNumber(root, "user_floor_margin_m", &cmd.user_floor_margin_m)) return false;
        cmd.user_floor_enable = true;  // default: a plane payload enables the constraint
        if (!readOptionalBool(root, "user_floor_enable", &cmd.user_floor_enable)) return false;
        cmd.has_user_floor_plane = true;
    }

    const json left_object = root.contains("left") ? root.at("left") : json();
    const json right_object = root.contains("right") ? root.at("right") : json();
    std::string root_arm;
    if (!readOptionalString(root, "arm", &root_arm)) return false;

    try {
        if (!parseArmObject(left_object, ArmId::Left, cmd.seq, receive_time_ns, default_mode, timeout_sec, &cmd.left)) return false;
        if (!parseArmObject(right_object, ArmId::Right, cmd.seq, receive_time_ns, default_mode, timeout_sec, &cmd.right)) return false;
    } catch (const std::exception&) {
        return false;
    }

    if (!root.contains("left") && !root.contains("right") &&
        default_mode == ControlMode::TcpCircleTrack) {
        if (root_arm.empty()) return false;
        ArmId selected_arm = ArmId::Left;
        if (!parseArmId(root_arm, &selected_arm)) return false;

        ArmCommand selected;
        if (!parseArmObject(root, selected_arm, cmd.seq, receive_time_ns, default_mode, timeout_sec, &selected)) {
            return false;
        }
        ArmCommand hold;
        hold.arm_id = selected_arm == ArmId::Left ? ArmId::Right : ArmId::Left;
        hold.seq = cmd.seq;
        hold.host_time_ns = receive_time_ns;
        hold.mode = ControlMode::Hold;
        hold.timeout_sec = timeout_sec;
        if (selected_arm == ArmId::Left) {
            cmd.left = selected;
            cmd.right = hold;
        } else {
            cmd.right = selected;
            cmd.left = hold;
        }
    } else if (!root.contains("left") && !root.contains("right")) {
        bool present = false;
        if (!readOptionalJointArray(root, "q_target_deg", &cmd.left.q_target_deg, &present)) return false;
        cmd.left.has_joint_target = cmd.left.has_joint_target || present;
        cmd.right.q_target_deg = cmd.left.q_target_deg;
        cmd.right.has_joint_target = cmd.right.has_joint_target || present;

        if (!readOptionalJointArray(root, "dq_target_deg_s", &cmd.left.dq_target_deg_s, &present)) return false;
        cmd.left.has_joint_velocity = cmd.left.has_joint_velocity || present;
        cmd.right.dq_target_deg_s = cmd.left.dq_target_deg_s;
        cmd.right.has_joint_velocity = cmd.right.has_joint_velocity || present;

        if (!readOptionalPose6D(root, "tcp_target_stand", &cmd.left.tcp_target_stand, &present)) return false;
        cmd.left.has_tcp_target = cmd.left.has_tcp_target || present;
        bool alias_present = false;
        Pose6D alias_target;
        if (!readOptionalPose6D(root, "target_tcp_stand", &alias_target, &alias_present)) return false;
        if (present && alias_present) return false;
        if (alias_present) {
            cmd.left.tcp_target_stand = alias_target;
            cmd.left.has_tcp_target = true;
        }
        cmd.right.tcp_target_stand = cmd.left.tcp_target_stand;
        cmd.right.has_tcp_target = cmd.right.has_tcp_target || present || alias_present;

        if (!readOptionalPose6D(root, "tcp_delta_stand", &cmd.left.tcp_delta_stand, &present)) return false;
        cmd.left.has_tcp_delta_stand = cmd.left.has_tcp_delta_stand || present;
        cmd.right.tcp_delta_stand = cmd.left.tcp_delta_stand;
        cmd.right.has_tcp_delta_stand = cmd.right.has_tcp_delta_stand || present;

        if (!readOptionalPose6D(root, "tcp_delta_local", &cmd.left.tcp_delta_local, &present)) return false;
        cmd.left.has_tcp_delta_local = cmd.left.has_tcp_delta_local || present;
        cmd.right.tcp_delta_local = cmd.left.tcp_delta_local;
        cmd.right.has_tcp_delta_local = cmd.right.has_tcp_delta_local || present;

        if (!readOptionalVec6(root, "tcp_twist_stand", &cmd.left.tcp_twist_stand, &present)) return false;
        cmd.left.has_tcp_twist_stand = cmd.left.has_tcp_twist_stand || present;
        cmd.right.tcp_twist_stand = cmd.left.tcp_twist_stand;
        cmd.right.has_tcp_twist_stand = cmd.right.has_tcp_twist_stand || present;

        if (!readOptionalVec6(root, "tcp_twist_local", &cmd.left.tcp_twist_local, &present)) return false;
        cmd.left.has_tcp_twist_local = cmd.left.has_tcp_twist_local || present;
        cmd.right.tcp_twist_local = cmd.left.tcp_twist_local;
        cmd.right.has_tcp_twist_local = cmd.right.has_tcp_twist_local || present;

        if (!readOptionalVec6(root, "tcp_target_twist_stand", &cmd.left.tcp_target_twist_stand, &present)) return false;
        cmd.left.has_tcp_target_twist_stand = cmd.left.has_tcp_target_twist_stand || present;
        cmd.right.tcp_target_twist_stand = cmd.left.tcp_target_twist_stand;
        cmd.right.has_tcp_target_twist_stand = cmd.right.has_tcp_target_twist_stand || present;

        if (!readOptionalLinearMoveFields(root, &cmd.left)) return false;
        cmd.right.linear_move_duration_sec = cmd.left.linear_move_duration_sec;
        cmd.right.linear_move_linear_speed_m_s = cmd.left.linear_move_linear_speed_m_s;
        cmd.right.linear_move_angular_speed_rad_s = cmd.left.linear_move_angular_speed_rad_s;
        cmd.right.linear_move_orientation_mode = cmd.left.linear_move_orientation_mode;
        cmd.right.has_linear_move_duration = cmd.right.has_linear_move_duration || cmd.left.has_linear_move_duration;
        cmd.right.has_linear_move_linear_speed = cmd.right.has_linear_move_linear_speed || cmd.left.has_linear_move_linear_speed;
        cmd.right.has_linear_move_angular_speed = cmd.right.has_linear_move_angular_speed || cmd.left.has_linear_move_angular_speed;
        cmd.right.has_linear_move_orientation_mode =
            cmd.right.has_linear_move_orientation_mode || cmd.left.has_linear_move_orientation_mode;

        if (cmd.left.mode == ControlMode::TcpCircleMove) {
            if (!readOptionalCircleMoveFields(root, &cmd.left)) return false;
            cmd.right.tcp_circle_move = cmd.left.tcp_circle_move;
            cmd.right.has_tcp_circle_move = cmd.right.has_tcp_circle_move || cmd.left.has_tcp_circle_move;
        }

        if (cmd.left.mode == ControlMode::TcpCircleTrack) {
            if (!readOptionalCircleTrackFields(root, &cmd.left)) return false;
            cmd.right.tcp_circle_track = cmd.left.tcp_circle_track;
            cmd.right.has_tcp_circle_track = cmd.right.has_tcp_circle_track || cmd.left.has_tcp_circle_track;
        }
    }

    if (requiresPayload(cmd.left.mode) && !hasRequiredPayload(cmd.left)) return false;
    if (requiresPayload(cmd.right.mode) && !hasRequiredPayload(cmd.right)) return false;

    const std::string key = sourceKey(cmd.source);
    const auto last_seq_it = last_accepted_seq_by_source_.find(key);
    if (last_seq_it != last_accepted_seq_by_source_.end() && cmd.seq <= last_seq_it->second) return false;

    CommandSourceLeaseState lease = currentLeaseState(receive_time_ns);
    const bool source_can_own_active_lease = lease.active && sameSource(lease, cmd.source);
    const bool provided_token_matches = cmd.source.lease_token.empty() ||
        (lease.active && cmd.source.lease_token == lease.lease_token);
    const bool requests_lease = acquire_lease_only ||
        cmd.left.mode == ControlMode::ArmMotion ||
        cmd.right.mode == ControlMode::ArmMotion;
    const bool requires_lease = !isEmergencyStopCommand(cmd) && dualCommandRequiresLease(cmd);
    cmd.lease = lease;
    cmd.lease.command_requires_lease = requires_lease;
    cmd.lease.command_has_lease = !requires_lease || (source_can_own_active_lease && provided_token_matches);

    if (command_source_config_.enforce_lease && !isEmergencyStopCommand(cmd)) {
        if (lease.active && source_can_own_active_lease && !provided_token_matches) {
            last_reject_reason_ = "command_source_lease_token_mismatch";
            return false;
        }
        if (requests_lease && lease.active && !source_can_own_active_lease) {
            last_reject_reason_ = "command_source_lease_conflict: active source_id=" +
                lease.source_id + " session_id=" + lease.session_id;
            return false;
        }
        if (requires_lease && !requests_lease && !cmd.lease.command_has_lease) {
            last_reject_reason_ = "command_source_lease_required: active source_id=" +
                lease.source_id + " session_id=" + lease.session_id;
            return false;
        }
    }

    if (release_lease_only) {
        // Voluntary lease handoff (e.g. client shutdown): only the owning
        // (source_id, session_id) — with a matching token when one is provided
        // (token mismatch already rejected above) — may clear the active
        // lease. A foreign or stale release cannot kick a live operator.
        // Releasing when no lease is active is an accepted no-op.
        if (lease.active && !source_can_own_active_lease) {
            last_reject_reason_ = "command_source_lease_release_denied: active source_id=" +
                lease.source_id + " session_id=" + lease.session_id;
            return false;
        }
        active_lease_ = CommandSourceLeaseState{};
        active_lease_.enforce_lease = command_source_config_.enforce_lease;
        cmd.lease = currentLeaseState(receive_time_ns);
        cmd.lease.command_requires_lease = false;
        cmd.lease.command_has_lease = true;
        *out_command = cmd;
        last_accepted_seq_by_source_[key] = cmd.seq;
        return true;
    }

    if (requests_lease || (requires_lease && source_can_own_active_lease && provided_token_matches)) {
        active_lease_.enforce_lease = command_source_config_.enforce_lease;
        active_lease_.active = true;
        active_lease_.command_requires_lease = requires_lease;
        active_lease_.command_has_lease = true;
        active_lease_.source_id = cmd.source.source_id;
        active_lease_.session_id = cmd.source.session_id;
        active_lease_.lease_token = cmd.source.lease_token.empty()
            ? (lease.lease_token.empty() || !sameSource(lease, cmd.source)
                ? generatedLeaseToken(cmd.source, receive_time_ns, ++lease_counter_)
                : lease.lease_token)
            : cmd.source.lease_token;
        active_lease_.acquired_time_ns = active_lease_.acquired_time_ns == 0 || !lease.active || !sameSource(lease, cmd.source)
            ? receive_time_ns
            : active_lease_.acquired_time_ns;
        active_lease_.expires_time_ns = receive_time_ns + timeoutNs(command_source_config_.lease_timeout_sec);
        active_lease_.verdict = "Ok";
        active_lease_.reason.clear();
        cmd.lease = active_lease_;
        cmd.lease.command_requires_lease = requires_lease;
        cmd.lease.command_has_lease = true;
        if (cmd.source.lease_token.empty()) {
            cmd.source.lease_token = active_lease_.lease_token;
        }
    } else {
        cmd.lease = currentLeaseState(receive_time_ns);
        cmd.lease.command_requires_lease = requires_lease;
        cmd.lease.command_has_lease = !requires_lease || (source_can_own_active_lease && provided_token_matches);
    }

    *out_command = cmd;
    last_accepted_seq_by_source_[key] = cmd.seq;
    return true;
}

bool CommandServer::acceptsCommandSource(const std::string& source_ip) const {
    uint32_t parsed_source = 0;
    if (!parseIpv4Address(source_ip, &parsed_source)) return false;
    for (const std::string& entry : config_.command_source_allowlist) {
        if (allowlistEntryMatches(entry, parsed_source)) return true;
    }
    return false;
}

std::string CommandServer::lastRejectReason() const {
    return last_reject_reason_;
}

CommandSourceLeaseState CommandServer::currentLeaseState(uint64_t now_ns) const {
    CommandSourceLeaseState lease = active_lease_;
    lease.enforce_lease = command_source_config_.enforce_lease;
    if (lease.active && lease.expires_time_ns > 0 && now_ns > lease.expires_time_ns) {
        lease.active = false;
        lease.verdict = "Expired";
        lease.reason = "command source lease expired";
    }
    return lease;
}

}  // namespace rb_servo
