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

    static bool isAvailable();

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

private:
    struct Impl;

    KinematicsConfig config_;
    std::unique_ptr<Impl> impl_;
};

}  // namespace rb_servo
