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
        case ControlMode::JointVelocity: return "JointVelocity";
        case ControlMode::TcpPoseTarget: return "TcpPoseTarget";
        case ControlMode::TcpDeltaStand: return "TcpDeltaStand";
        case ControlMode::TcpDeltaLocal: return "TcpDeltaLocal";
        case ControlMode::EmergencyStop: return "EmergencyStop";
        case ControlMode::ResetFault: return "ResetFault";
    }
    return "Unknown";
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

std::string toString(ForceControlMode mode) {
    switch (mode) {
        case ForceControlMode::Off: return "Off";
        case ForceControlMode::Admittance: return "Admittance";
        case ForceControlMode::Impedance: return "Impedance";
        case ForceControlMode::ExternalForceSafety: return "ExternalForceSafety";
    }
    return "Unknown";
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
        case SafetyVerdict::UnknownError: return "UnknownError";
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
    if (m == "jointvelocity" || m == "joint_velocity") return ControlMode::JointVelocity;
    if (m == "tcpposetarget" || m == "tcp_pose_target") return ControlMode::TcpPoseTarget;
    if (m == "tcpdeltastand" || m == "tcp_delta_stand") return ControlMode::TcpDeltaStand;
    if (m == "tcpdeltalocal" || m == "tcp_delta_local") return ControlMode::TcpDeltaLocal;
    if (m == "emergencystop" || m == "emergency_stop" || m == "estop") return ControlMode::EmergencyStop;
    if (m == "resetfault" || m == "reset_fault" || m == "reset") return ControlMode::ResetFault;
    throw std::invalid_argument("Unknown ControlMode string: " + mode);
}

ForceControlMode forceControlModeFromString(const std::string& mode) {
    const std::string m = lower(mode);
    if (m == "off") return ForceControlMode::Off;
    if (m == "admittance") return ForceControlMode::Admittance;
    if (m == "impedance") return ForceControlMode::Impedance;
    if (m == "externalforcesafety" || m == "external_force_safety" || m == "safety") {
        return ForceControlMode::ExternalForceSafety;
    }
    throw std::invalid_argument("Unknown ForceControlMode string: " + mode);
}

TrackingErrorPolicy trackingErrorPolicyFromString(const std::string& value) {
    const std::string v = lower(value);
    if (v == "snap_to_actual" || v == "snap" || v == "forgive") return TrackingErrorPolicy::SnapToActual;
    if (v == "fault_latch" || v == "latch" || v == "fault") return TrackingErrorPolicy::FaultLatch;
    throw std::invalid_argument("Unknown tracking_error_policy string: " + value);
}

}  // namespace rb_servo
