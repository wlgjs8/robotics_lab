#pragma once

#include "rb_servo/config/config.hpp"
#include "rb_servo/core/types.hpp"

namespace rb_servo {

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
};

}  // namespace rb_servo
