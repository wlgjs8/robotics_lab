#pragma once

#include <string>

#include "rb_servo/config/config.hpp"
#include "rb_servo/core/types.hpp"

namespace rb_servo {

struct IkResult {
    bool success = false;
    JointArray q_solution_deg{};
    double position_error_m = 0.0;
    double orientation_error_rad = 0.0;
    double duration_us = 0.0;
    int iterations = 0;
    bool timed_out = false;
    std::string reason;
};

struct CartesianVelocityResult {
    bool success = false;
    JointArray qdot_deg_s{};
    std::string reason;
};

class IKinematics {
public:
    virtual ~IKinematics() = default;

    // External command/state protocol uses degrees. Implementations convert to
    // their internal representation before running FK.
    virtual Pose6D computeTcpBase(const JointArray& q_deg) const = 0;

    // Pose6D rotation fields are XYZ roll/pitch/yaw angles in radians. P2 uses
    // them only as a TCP pose publication boundary, not as an IK command API.
    virtual Pose6D computeTcpStand(
        ArmId arm,
        const JointArray& q_deg,
        const ArmMountConfig& mount
    ) const = 0;

    virtual IkResult solveIk(
        ArmId arm,
        const Pose6D& target_tcp_stand,
        const JointArray& seed_q_deg,
        const ArmMountConfig& mount
    ) const = 0;

    virtual CartesianVelocityResult solveCartesianVelocity(
        ArmId arm,
        const JointArray& q_deg,
        const ArmMountConfig& mount,
        const Vec6& tcp_twist_local,
        double damping
    ) const = 0;
};

}  // namespace rb_servo
