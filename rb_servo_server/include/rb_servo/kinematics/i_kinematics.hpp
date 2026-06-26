#pragma once

#include <array>
#include <string>
#include <vector>

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
    // Conditioning / singularity-robust-damping diagnostics (last DLS step).
    double min_singular_value = 0.0;     // smallest task-Jacobian singular value
    double applied_damping = 0.0;        // effective lambda used on the singular dir
    // Branch-jump guard (see IkSolverConfig). solution_jump_deg / suspected are
    // observability; clamped=true means the solve held the seed (zero motion this
    // tick) to avoid flipping to a distant branch.
    double solution_jump_deg = 0.0;      // max |q_solution - seed| over joints
    bool branch_jump_suspected = false;
    bool branch_jump_clamped = false;
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

    // Ordered kinematic chain points (xyz, meters) in the STAND frame used to
    // build per-link self-collision capsules: [base, joint1..joint6 origins, tcp].
    // Consecutive points are capsule bone endpoints. Default returns empty,
    // meaning the implementation does not provide link geometry (the dual-arm
    // self-collision guard then treats geometry as unavailable / fails closed).
    virtual std::vector<std::array<double, 3>> linkCollisionPointsInStand(
        ArmId arm,
        const JointArray& q_deg,
        const ArmMountConfig& mount
    ) const {
        (void)arm;
        (void)q_deg;
        (void)mount;
        return {};
    }

    // Stand-frame z-velocity Jacobian of a TCP-frame offset point (Stage 3 floor
    // velocity projection): fills Jz_out (1x6, per arm joint, rad) such that
    // d(p_z_stand)/dt = Jz_out . qdot, where p = TCP + R_tcp * tcp_offset_m. Returns
    // false (and leaves Jz_out zero) when kinematics/FK is unavailable or non-finite.
    // Default: unsupported -> false (caller fails closed / keeps the FK backstop).
    virtual bool computeFloorPointZJacobian(
        ArmId arm,
        const JointArray& q_deg,
        const ArmMountConfig& mount,
        const std::array<double, 3>& tcp_offset_m,
        JointArray& Jz_out
    ) const {
        (void)arm;
        (void)q_deg;
        (void)mount;
        (void)tcp_offset_m;
        Jz_out = JointArray{};
        return false;
    }

    // Stand-frame single-axis velocity Jacobian of a TCP-frame offset point
    // (Stage 3 ROI-box velocity projection): fills J_out (1x6, per arm joint,
    // rad) such that d(p_axis_stand)/dt = J_out . qdot, where p = TCP + R_tcp *
    // tcp_offset_m and axis in {0=x, 1=y, 2=z} selects the stand-frame axis. The
    // generalization of computeFloorPointZJacobian (which is axis=2). Returns
    // false (and leaves J_out zero) when kinematics/FK is unavailable, non-finite,
    // or axis is out of range. Default: unsupported -> false (caller fails closed).
    virtual bool computeStandAxisJacobian(
        ArmId arm,
        const JointArray& q_deg,
        const ArmMountConfig& mount,
        const std::array<double, 3>& tcp_offset_m,
        int axis,
        JointArray& J_out
    ) const {
        (void)arm;
        (void)q_deg;
        (void)mount;
        (void)tcp_offset_m;
        (void)axis;
        J_out = JointArray{};
        return false;
    }

    // Stand-frame directional velocity Jacobian of a TCP-frame offset point
    // (Stage 3 reach-shell velocity projection): fills J_out (1x6, per arm joint,
    // rad) such that d(dir . p_stand)/dt = J_out . qdot, where p = TCP + R_tcp *
    // tcp_offset_m and dir_stand is a STAND-frame direction (need not be unit; the
    // caller passes the radial unit vector from the arm base to the point). The
    // generalization of computeStandAxisJacobian (axis = a unit stand axis). Returns
    // false (and leaves J_out zero) when kinematics/FK is unavailable, non-finite,
    // or dir_stand is ~zero. Default: unsupported -> false (caller fails closed).
    virtual bool computeStandDirectionJacobian(
        ArmId arm,
        const JointArray& q_deg,
        const ArmMountConfig& mount,
        const std::array<double, 3>& tcp_offset_m,
        const std::array<double, 3>& dir_stand,
        JointArray& J_out
    ) const {
        (void)arm;
        (void)q_deg;
        (void)mount;
        (void)tcp_offset_m;
        (void)dir_stand;
        J_out = JointArray{};
        return false;
    }

};

}  // namespace rb_servo
