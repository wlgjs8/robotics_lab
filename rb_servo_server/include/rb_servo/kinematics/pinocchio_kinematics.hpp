#pragma once

#include <memory>

#include "rb_servo/kinematics/i_kinematics.hpp"

namespace rb_servo {

class PinocchioKinematics final : public IKinematics {
public:
    explicit PinocchioKinematics(KinematicsConfig config);
    ~PinocchioKinematics() override;

    PinocchioKinematics(const PinocchioKinematics&) = delete;
    PinocchioKinematics& operator=(const PinocchioKinematics&) = delete;
    PinocchioKinematics(PinocchioKinematics&&) noexcept;
    PinocchioKinematics& operator=(PinocchioKinematics&&) noexcept;

    Pose6D computeTcpBase(const JointArray& q_deg) const override;
    Pose6D computeTcpStand(
        ArmId arm,
        const JointArray& q_deg,
        const ArmMountConfig& mount
    ) const override;
    IkResult solveIk(
        ArmId arm,
        const Pose6D& target_tcp_stand,
        const JointArray& seed_q_deg,
        const ArmMountConfig& mount
    ) const override;
    CartesianVelocityResult solveCartesianVelocity(
        ArmId arm,
        const JointArray& q_deg,
        const ArmMountConfig& mount,
        const Vec6& tcp_twist_local,
        double damping
    ) const override;

    std::vector<std::array<double, 3>> linkCollisionPointsInStand(
        ArmId arm,
        const JointArray& q_deg,
        const ArmMountConfig& mount
    ) const override;

    std::vector<ArmCapsule> armCollisionCapsulesInStand(
        ArmId arm,
        const JointArray& q_deg,
        const ArmMountConfig& mount,
        const std::vector<ArmCapsuleConfig>& templates
    ) const override;

private:
    struct Impl;

    // Core damped-least-squares solve; damping_scale multiplies the base and
    // singular-region damping (1.0 = configured damping). solveIk() wraps this
    // with the branch-jump clamp (re-solve at higher damping / hold seed).
    IkResult solveIkDamped(
        ArmId arm,
        const Pose6D& target_tcp_stand,
        const JointArray& seed_q_deg,
        const ArmMountConfig& mount,
        double damping_scale
    ) const;

    KinematicsConfig config_;
    std::unique_ptr<Impl> impl_;
};

}  // namespace rb_servo
