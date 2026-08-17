#include "rb_servo/core/types.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace rb_servo {
namespace {
std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return s;
}
}

std::string toString(ArmId arm_id) {
    return arm_id == ArmId::Left ? "left" : "right";
}

std::string toString(ControlMode mode) {
    switch (mode) {
        case ControlMode::Idle: return "Idle";
        case ControlMode::Hold: return "Hold";
        case ControlMode::ArmMotion: return "ArmMotion";
        case ControlMode::DisarmMotion: return "DisarmMotion";
        case ControlMode::JointTarget: return "JointTarget";
        case ControlMode::TcpPoseTarget: return "TcpPoseTarget";
        case ControlMode::TcpLinearMove: return "TcpLinearMove";
        case ControlMode::EmergencyStop: return "EmergencyStop";
        case ControlMode::ResetFault: return "ResetFault";
        case ControlMode::SetSafetyFloorZ: return "SetSafetyFloorZ";
        case ControlMode::SetSafetyFloorEnabled: return "SetSafetyFloorEnabled";
        case ControlMode::SetSafetyRoiBounds: return "SetSafetyRoiBounds";
        case ControlMode::SetUserSafetyFloorPlane: return "SetUserSafetyFloorPlane";
        case ControlMode::Freedrive: return "Freedrive";
    }
    return "Unknown";
}

std::string toString(JointTargetProfile profile) {
    switch (profile) {
        case JointTargetProfile::Direct: return "direct";
        case JointTargetProfile::InitMotion: return "init_motion";
    }
    return "unknown";
}

std::string toString(ServerMotionState state) {
    switch (state) {
        case ServerMotionState::Disconnected: return "Disconnected";
        case ServerMotionState::ConnectedHold: return "ConnectedHold";
        case ServerMotionState::ArmedHold: return "ArmedHold";
        case ServerMotionState::Running: return "Running";
        case ServerMotionState::FaultLatched: return "FaultLatched";
        case ServerMotionState::EmergencyLatched: return "EmergencyLatched";
    }
    return "Unknown";
}

std::string toString(BackendAckPolicy policy) {
    switch (policy) {
        case BackendAckPolicy::BackendDefault: return "backend_default";
        case BackendAckPolicy::Wait: return "wait";
        case BackendAckPolicy::Disabled: return "disabled";
    }
    return "unknown";
}

std::string toString(SafetyVerdict verdict) {
    switch (verdict) {
        case SafetyVerdict::Ok: return "Ok";
        case SafetyVerdict::JointLimitClamped: return "JointLimitClamped";
        case SafetyVerdict::TrackingError: return "TrackingError";
        case SafetyVerdict::RobotStateError: return "RobotStateError";
        case SafetyVerdict::SendFailure: return "SendFailure";
        case SafetyVerdict::EmergencyStop: return "EmergencyStop";
        case SafetyVerdict::FaultLatched: return "FaultLatched";
        case SafetyVerdict::InvalidCommand: return "InvalidCommand";
        case SafetyVerdict::CartesianUnavailable: return "CartesianUnavailable";
        case SafetyVerdict::IkFailed: return "IkFailed";
        case SafetyVerdict::SelfCollision: return "SelfCollision";
        case SafetyVerdict::FloorViolation: return "FloorViolation";
        case SafetyVerdict::RoiViolation: return "RoiViolation";
        case SafetyVerdict::ChunkFollowerFault: return "ChunkFollowerFault";
        case SafetyVerdict::UnknownError: return "UnknownError";
    }
    return "Unknown";
}

std::string toString(FaultDomain domain) {
    switch (domain) {
        case FaultDomain::None: return "None";
        case FaultDomain::SafetyPolicy: return "SafetyPolicy";
        case FaultDomain::Backend: return "Backend";
        case FaultDomain::RobotState: return "RobotState";
        case FaultDomain::Command: return "Command";
        case FaultDomain::Kinematics: return "Kinematics";
        case FaultDomain::Emergency: return "Emergency";
    }
    return "Unknown";
}

std::string toString(TrackingErrorPolicy policy) {
    switch (policy) {
        case TrackingErrorPolicy::SnapToActual: return "snap_to_actual";
        case TrackingErrorPolicy::FaultLatch: return "fault_latch";
    }
    return "unknown";
}

ControlMode controlModeFromString(const std::string& mode) {
    const std::string m = lower(mode);
    if (m == "idle") return ControlMode::Idle;
    if (m == "hold") return ControlMode::Hold;
    if (m == "armmotion" || m == "arm_motion" || m == "arm") return ControlMode::ArmMotion;
    if (m == "disarmmotion" || m == "disarm_motion" || m == "disarm") return ControlMode::DisarmMotion;
    if (m == "jointtarget" || m == "joint_target") return ControlMode::JointTarget;
    if (m == "tcpposetarget" || m == "tcp_pose_target") return ControlMode::TcpPoseTarget;
    if (m == "tcplinearmove" || m == "tcp_linear_move") return ControlMode::TcpLinearMove;
    if (m == "emergencystop" || m == "emergency_stop" || m == "estop") return ControlMode::EmergencyStop;
    if (m == "resetfault" || m == "reset_fault" || m == "reset") return ControlMode::ResetFault;
    if (m == "setsafetyfloorz" || m == "set_safety_floor_z") return ControlMode::SetSafetyFloorZ;
    if (m == "setsafetyfloorenabled" || m == "set_safety_floor_enabled") {
        return ControlMode::SetSafetyFloorEnabled;
    }
    if (m == "setsafetyroibounds" || m == "set_safety_roi_bounds") return ControlMode::SetSafetyRoiBounds;
    if (m == "setusersafetyfloorplane" || m == "set_user_safety_floor_plane") {
        return ControlMode::SetUserSafetyFloorPlane;
    }
    if (m == "freedrive" || m == "free_drive" || m == "directteaching" || m == "direct_teaching") {
        return ControlMode::Freedrive;
    }
    throw std::invalid_argument("Unknown ControlMode string: " + mode);
}

JointTargetProfile jointTargetProfileFromString(const std::string& value) {
    const std::string v = lower(value);
    if (v == "direct") return JointTargetProfile::Direct;
    if (v == "init_motion") return JointTargetProfile::InitMotion;
    throw std::invalid_argument("Unknown joint_target_profile string: " + value);
}

TrackingErrorPolicy trackingErrorPolicyFromString(const std::string& value) {
    const std::string v = lower(value);
    if (v == "snap_to_actual" || v == "snap" || v == "forgive") return TrackingErrorPolicy::SnapToActual;
    if (v == "fault_latch" || v == "latch" || v == "fault") return TrackingErrorPolicy::FaultLatch;
    throw std::invalid_argument("Unknown tracking_error_policy string: " + value);
}

}  // namespace rb_servo
