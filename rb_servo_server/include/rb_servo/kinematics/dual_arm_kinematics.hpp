#pragma once
// TWO CALIBRATED ARMS, ONE IKinematics (2026-09-05). Each box carries its own DH
// table, so the left and right chains are different models once the box
// calibration is adopted. This wrapper owns one provider per arm and forwards by
// ArmId; the arm-less computeTcpBase() answers with the LEFT model (it is only
// used where no arm is known, and both models agree to the calibration delta).
#include "rb_servo/kinematics/i_kinematics.hpp"

#include <memory>
#include <stdexcept>

namespace rb_servo {

class DualArmKinematics final : public IKinematics {
public:
    DualArmKinematics(std::shared_ptr<IKinematics> left, std::shared_ptr<IKinematics> right)
        : left_(std::move(left)), right_(std::move(right)) {
        if (!left_ || !right_) throw std::runtime_error("DualArmKinematics needs both arms");
    }
    Pose6D computeTcpBase(const JointArray& q_deg) const override { return left_->computeTcpBase(q_deg); }
    Pose6D computeTcpStand(ArmId arm, const JointArray& q_deg, const ArmMountConfig& mount) const override {
        return pick(arm).computeTcpStand(arm, q_deg, mount);
    }
    std::optional<Pose6D> computeFlangeStand(ArmId arm, const JointArray& q_deg,
                                             const ArmMountConfig& mount) const override {
        return pick(arm).computeFlangeStand(arm, q_deg, mount);
    }
    IkResult solveIk(ArmId arm, const Pose6D& target_tcp_stand, const JointArray& seed_q_deg,
                     const ArmMountConfig& mount) const override {
        return pick(arm).solveIk(arm, target_tcp_stand, seed_q_deg, mount);
    }
    IkResult solveIkToTarget(ArmId arm, const Pose6D& target_tcp_stand, const JointArray& seed_q_deg,
                             const ArmMountConfig& mount) const override {
        return pick(arm).solveIkToTarget(arm, target_tcp_stand, seed_q_deg, mount);
    }
    std::vector<std::array<double, 3>> linkCollisionPointsInStand(ArmId arm, const JointArray& q_deg,
                                                                  const ArmMountConfig& mount) const override {
        return pick(arm).linkCollisionPointsInStand(arm, q_deg, mount);
    }
    bool computeFloorPointZJacobian(ArmId arm, const JointArray& q_deg, const ArmMountConfig& mount,
                                    const std::array<double, 3>& tcp_offset_m, JointArray& Jz_out) const override {
        return pick(arm).computeFloorPointZJacobian(arm, q_deg, mount, tcp_offset_m, Jz_out);
    }
    bool computeStandAxisJacobian(ArmId arm, const JointArray& q_deg, const ArmMountConfig& mount,
                                  const std::array<double, 3>& tcp_offset_m, int axis,
                                  JointArray& J_out) const override {
        return pick(arm).computeStandAxisJacobian(arm, q_deg, mount, tcp_offset_m, axis, J_out);
    }
    bool computeStandDirectionJacobian(ArmId arm, const JointArray& q_deg, const ArmMountConfig& mount,
                                       const std::array<double, 3>& tcp_offset_m,
                                       const std::array<double, 3>& dir_stand, JointArray& J_out) const override {
        return pick(arm).computeStandDirectionJacobian(arm, q_deg, mount, tcp_offset_m, dir_stand, J_out);
    }

private:
    const IKinematics& pick(ArmId arm) const { return arm == ArmId::Left ? *left_ : *right_; }
    std::shared_ptr<IKinematics> left_;
    std::shared_ptr<IKinematics> right_;
};

}  // namespace rb_servo
