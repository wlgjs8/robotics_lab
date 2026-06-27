#include "rb_servo/kinematics/pinocchio_kinematics.hpp"

#include "rb_servo/kinematics/ik_solver.hpp"
#include "rb_servo/math/se3.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/se3.hpp>

namespace rb_servo {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

double degToRad(double deg) {
    return deg * kPi / 180.0;
}

double radToDeg(double rad) {
    return rad * 180.0 / kPi;
}

bool finitePose(const Pose6D& pose) {
    return std::isfinite(pose.x) &&
           std::isfinite(pose.y) &&
           std::isfinite(pose.z) &&
           std::isfinite(pose.rx) &&
           std::isfinite(pose.ry) &&
           std::isfinite(pose.rz);
}

JointArray jointDelta(const JointArray& q, const JointArray& seed) {
    JointArray out{};
    for (std::size_t i = 0; i < out.size(); ++i) {
        out[i] = q[i] - seed[i];
    }
    return out;
}

double maxAbsJointDelta(const JointArray& q, const JointArray& seed) {
    double max_abs = 0.0;
    for (std::size_t i = 0; i < kDof; ++i) {
        max_abs = std::max(max_abs, std::abs(q[i] - seed[i]));
    }
    return max_abs;
}

void populateBranchJumpDetails(
    IkResult& result,
    const JointArray& seed_q_deg,
    double limit_deg,
    int retry_count,
    const JointArray& raw_solution_q_deg
) {
    result.branch_jump_details_valid = true;
    result.q_seed_deg = seed_q_deg;
    result.q_raw_solution_deg = raw_solution_q_deg;
    result.q_raw_delta_deg = jointDelta(raw_solution_q_deg, seed_q_deg);
    result.q_solution_delta_deg = jointDelta(result.q_solution_deg, seed_q_deg);
    result.raw_solution_jump_deg = maxAbsJointDelta(raw_solution_q_deg, seed_q_deg);
    result.branch_jump_limit_deg = limit_deg;
    result.branch_jump_retry_count = retry_count;
}

void populateBranchJumpDetails(
    IkResult& result,
    const JointArray& seed_q_deg,
    double limit_deg,
    int retry_count
) {
    populateBranchJumpDetails(
        result,
        seed_q_deg,
        limit_deg,
        retry_count,
        result.q_solution_deg
    );
}

}  // namespace

struct PinocchioKinematics::Impl {
    explicit Impl(pinocchio::Model model_in)
        : model(std::move(model_in)), data(model) {}

    pinocchio::Model model;
    pinocchio::Data data;
    pinocchio::FrameIndex base_frame = 0;
    pinocchio::FrameIndex tip_frame = 0;
    std::array<pinocchio::JointIndex, kDof> joints{};
};

pinocchio::FrameIndex requireFrame(const pinocchio::Model& model, const std::string& name, const std::string& label) {
    const pinocchio::FrameIndex id = model.getFrameId(name);
    if (id >= model.nframes) {
        throw std::runtime_error("kinematics." + label + " not found in URDF frames: " + name);
    }
    return id;
}

pinocchio::JointIndex requireSingleDofJoint(const pinocchio::Model& model, const std::string& name) {
    const pinocchio::JointIndex id = model.getJointId(name);
    if (id >= model.njoints) {
        throw std::runtime_error("kinematics.joint_names entry not found in URDF joints: " + name);
    }
    if (model.nqs[id] != 1 || model.nvs[id] != 1) {
        throw std::runtime_error("kinematics.joint_names entry is not a single-DOF joint: " + name);
    }
    return id;
}

Eigen::VectorXd toPinocchioQ(
    const JointArray& q_deg,
    const pinocchio::Model& model,
    const std::array<pinocchio::JointIndex, kDof>& joints
) {
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    for (std::size_t i = 0; i < q_deg.size(); ++i) {
        const pinocchio::JointIndex joint_id = joints[i];
        q[model.idx_qs[joint_id]] = degToRad(q_deg[i]);
    }
    return q;
}

JointArray fromPinocchioQ(
    const Eigen::VectorXd& q,
    const pinocchio::Model& model,
    const std::array<pinocchio::JointIndex, kDof>& joints
) {
    JointArray out{};
    for (std::size_t i = 0; i < out.size(); ++i) {
        const pinocchio::JointIndex joint_id = joints[i];
        out[i] = radToDeg(q[model.idx_qs[joint_id]]);
    }
    return out;
}

bool clampJointLimits(
    Eigen::VectorXd* q,
    const pinocchio::Model& model,
    const std::array<pinocchio::JointIndex, kDof>& joints
) {
    bool clamped = false;
    for (std::size_t i = 0; i < joints.size(); ++i) {
        const pinocchio::JointIndex joint_id = joints[i];
        const Eigen::DenseIndex q_index = model.idx_qs[joint_id];
        const double lower = model.lowerPositionLimit[q_index];
        const double upper = model.upperPositionLimit[q_index];
        if (std::isfinite(lower) && (*q)[q_index] < lower) {
            (*q)[q_index] = lower;
            clamped = true;
        }
        if (std::isfinite(upper) && (*q)[q_index] > upper) {
            (*q)[q_index] = upper;
            clamped = true;
        }
    }
    return clamped;
}

Eigen::Matrix<double, 6, 1> clampJointStep(
    const Eigen::VectorXd& dq_full,
    const KinematicsConfig& config,
    const pinocchio::Model& model,
    const std::array<pinocchio::JointIndex, kDof>& joints
) {
    Eigen::Matrix<double, 6, 1> dq = Eigen::Matrix<double, 6, 1>::Zero();
    for (std::size_t i = 0; i < joints.size(); ++i) {
        const pinocchio::JointIndex joint_id = joints[i];
        const Eigen::DenseIndex v_index = model.idx_vs[joint_id];
        const double max_step = degToRad(config.ik.max_step_deg[i]);
        dq[static_cast<Eigen::Index>(i)] = std::clamp(dq_full[v_index], -max_step, max_step);
    }
    return dq;
}

void applyJointStep(
    Eigen::VectorXd* q,
    const Eigen::Matrix<double, 6, 1>& dq,
    const pinocchio::Model& model,
    const std::array<pinocchio::JointIndex, kDof>& joints
) {
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
    for (std::size_t i = 0; i < joints.size(); ++i) {
        const pinocchio::JointIndex joint_id = joints[i];
        v[model.idx_vs[joint_id]] = dq[static_cast<Eigen::Index>(i)];
    }
    Eigen::VectorXd integrated = *q;
    pinocchio::integrate(model, *q, v, integrated);
    *q = integrated;
}

PinocchioKinematics::PinocchioKinematics(KinematicsConfig config)
    : config_(std::move(config)) {
    pinocchio::Model model;
    pinocchio::urdf::buildModel(config_.urdf, model);

    impl_ = std::make_unique<Impl>(std::move(model));
    impl_->base_frame = requireFrame(impl_->model, config_.base_link, "base_link");
    impl_->tip_frame = requireFrame(impl_->model, config_.tip_link, "tip_link");
    if (config_.joint_names.size() != kDof) {
        throw std::runtime_error("kinematics.joint_names must contain exactly 6 names");
    }
    for (std::size_t i = 0; i < config_.joint_names.size(); ++i) {
        impl_->joints[i] = requireSingleDofJoint(impl_->model, config_.joint_names[i]);
    }
}

PinocchioKinematics::~PinocchioKinematics() = default;
PinocchioKinematics::PinocchioKinematics(PinocchioKinematics&&) noexcept = default;
PinocchioKinematics& PinocchioKinematics::operator=(PinocchioKinematics&&) noexcept = default;

Pose6D PinocchioKinematics::computeTcpBase(const JointArray& q_deg) const {
    if (!impl_) {
        throw std::runtime_error("Pinocchio kinematics is not initialized");
    }
    Eigen::VectorXd q = toPinocchioQ(q_deg, impl_->model, impl_->joints);
    pinocchio::forwardKinematics(impl_->model, impl_->data, q);
    pinocchio::updateFramePlacements(impl_->model, impl_->data);

    const pinocchio::SE3& world_base = impl_->data.oMf[impl_->base_frame];
    const pinocchio::SE3& world_tip = impl_->data.oMf[impl_->tip_frame];
    Pose6D pose = math::poseFromSe3(world_base.inverse() * world_tip);
    if (!finitePose(pose)) {
        throw std::runtime_error("Pinocchio FK produced a non-finite TCP pose");
    }
    return pose;
}

Pose6D PinocchioKinematics::computeTcpStand(
    ArmId arm,
    const JointArray& q_deg,
    const ArmMountConfig& mount
) const {
    (void)arm;
    const Pose6D tcp_base = computeTcpBase(q_deg);
    return math::poseFromSe3(math::se3FromPose(mount.base_pose_in_stand) * math::se3FromPose(tcp_base));
}

std::vector<std::array<double, 3>> PinocchioKinematics::linkCollisionPointsInStand(
    ArmId arm,
    const JointArray& q_deg,
    const ArmMountConfig& mount
) const {
    (void)arm;
    if (!impl_) {
        throw std::runtime_error("Pinocchio kinematics is not initialized");
    }
    Eigen::VectorXd q = toPinocchioQ(q_deg, impl_->model, impl_->joints);
    pinocchio::forwardKinematics(impl_->model, impl_->data, q);
    pinocchio::updateFramePlacements(impl_->model, impl_->data);

    const pinocchio::SE3& world_base = impl_->data.oMf[impl_->base_frame];
    const pinocchio::SE3 stand_T_world =
        math::se3FromPose(mount.base_pose_in_stand) * world_base.inverse();

    // World-frame chain points: base origin, each joint origin, then the tip.
    std::vector<Eigen::Vector3d> world_points;
    world_points.reserve(impl_->joints.size() + 2);
    world_points.push_back(world_base.translation());
    for (std::size_t i = 0; i < impl_->joints.size(); ++i) {
        world_points.push_back(impl_->data.oMi[impl_->joints[i]].translation());
    }
    world_points.push_back(impl_->data.oMf[impl_->tip_frame].translation());

    std::vector<std::array<double, 3>> stand_points;
    stand_points.reserve(world_points.size());
    for (const Eigen::Vector3d& wp : world_points) {
        const Eigen::Vector3d sp = stand_T_world.act(wp);
        if (!sp.allFinite()) {
            throw std::runtime_error("Pinocchio self-collision FK produced a non-finite point");
        }
        stand_points.push_back({sp.x(), sp.y(), sp.z()});
    }
    return stand_points;
}

bool PinocchioKinematics::computeFloorPointZJacobian(
    ArmId arm,
    const JointArray& q_deg,
    const ArmMountConfig& mount,
    const std::array<double, 3>& tcp_offset_m,
    JointArray& Jz_out
) const {
    // The floor plane is the stand-frame z (axis=2) face of the general ROI box;
    // share one implementation so the numerics stay identical.
    return computeStandAxisJacobian(arm, q_deg, mount, tcp_offset_m, 2, Jz_out);
}

bool PinocchioKinematics::computeStandAxisJacobian(
    ArmId arm,
    const JointArray& q_deg,
    const ArmMountConfig& mount,
    const std::array<double, 3>& tcp_offset_m,
    int axis,
    JointArray& J_out
) const {
    J_out = JointArray{};
    if (axis < 0 || axis > 2) return false;
    // A stand-frame axis is just the unit direction in that axis; reuse the
    // general directional projection so the numerics stay identical.
    std::array<double, 3> dir_stand{0.0, 0.0, 0.0};
    dir_stand[static_cast<std::size_t>(axis)] = 1.0;
    return computeStandDirectionJacobian(arm, q_deg, mount, tcp_offset_m, dir_stand, J_out);
}

bool PinocchioKinematics::computeStandDirectionJacobian(
    ArmId arm,
    const JointArray& q_deg,
    const ArmMountConfig& mount,
    const std::array<double, 3>& tcp_offset_m,
    const std::array<double, 3>& dir_stand,
    JointArray& J_out
) const {
    (void)arm;
    J_out = JointArray{};
    if (!config_.enable || !impl_) return false;
    if (!ik_solver::isFiniteJoints(q_deg)) return false;
    const Eigen::Vector3d dir(dir_stand[0], dir_stand[1], dir_stand[2]);
    if (!dir.allFinite() || dir.squaredNorm() < 1e-18) return false;

    const Eigen::VectorXd q = toPinocchioQ(q_deg, impl_->model, impl_->joints);
    pinocchio::forwardKinematics(impl_->model, impl_->data, q);
    pinocchio::computeJointJacobians(impl_->model, impl_->data, q);
    pinocchio::updateFramePlacements(impl_->model, impl_->data);

    // Tip-frame spatial Jacobian in world axes (linear = tip-origin velocity, angular
    // = angular velocity), sliced to the arm's 6 joint columns.
    Eigen::Matrix<double, 6, Eigen::Dynamic> Jf(6, impl_->model.nv);
    Jf.setZero();
    pinocchio::getFrameJacobian(impl_->model, impl_->data, impl_->tip_frame,
                                pinocchio::LOCAL_WORLD_ALIGNED, Jf);
    Eigen::Matrix<double, 6, 6> J;
    J.setZero();
    for (std::size_t i = 0; i < impl_->joints.size() && i < kDof; ++i) {
        J.col(static_cast<Eigen::Index>(i)) = Jf.col(impl_->model.idx_vs[impl_->joints[i]]);
    }

    const pinocchio::SE3& world_base = impl_->data.oMf[impl_->base_frame];
    const pinocchio::SE3& world_tip = impl_->data.oMf[impl_->tip_frame];
    // Offset point velocity in world axes: p = tip_origin + r, r = R_tip * offset.
    // pdot = Jv qdot + omega x r = [Jv - skew(r) Jw] qdot.
    const Eigen::Vector3d offset(tcp_offset_m[0], tcp_offset_m[1], tcp_offset_m[2]);
    const Eigen::Vector3d r = world_tip.rotation() * offset;
    Eigen::Matrix3d S;
    S << 0.0, -r.z(), r.y(), r.z(), 0.0, -r.x(), -r.y(), r.x(), 0.0;
    const Eigen::Matrix<double, 3, 6> Jp = J.template topRows<3>() - S * J.template bottomRows<3>();
    // Requested stand-frame direction expressed in world axes (mount rotation is
    // constant). d(dir . p_stand)/dt = (R_world_stand dir)^T pdot_world.
    const pinocchio::SE3 stand_T_world =
        math::se3FromPose(mount.base_pose_in_stand) * world_base.inverse();
    const Eigen::Vector3d n = stand_T_world.rotation().transpose() * dir;
    const Eigen::Matrix<double, 1, 6> Jdir = n.transpose() * Jp;
    if (!Jdir.allFinite()) return false;
    for (int i = 0; i < kDof; ++i) J_out[i] = Jdir(i);
    return true;
}

IkResult PinocchioKinematics::solveIkDamped(
    ArmId arm,
    const Pose6D& target_tcp_stand,
    const JointArray& seed_q_deg,
    const ArmMountConfig& mount,
    double damping_scale
) const {
    (void)arm;
    const double eff_damping = config_.ik.damping * damping_scale;
    const double eff_damping_max = config_.ik.damping_max * damping_scale;
    const auto started = std::chrono::steady_clock::now();
    const auto elapsedUs = [&]() {
        return std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - started
        ).count();
    };
    if (!config_.enable || !config_.ik.enable || !impl_) {
        return ik_solver::failureResult(
            ik_solver::kReasonKinematicsUnavailable,
            seed_q_deg,
            0.0,
            0.0,
            0,
            elapsedUs()
        );
    }
    if (!ik_solver::isFinitePose(target_tcp_stand) || !ik_solver::isFiniteJoints(seed_q_deg)) {
        return ik_solver::failureResult(
            ik_solver::kReasonInvalidTarget,
            seed_q_deg,
            0.0,
            0.0,
            0,
            elapsedUs()
        );
    }

    Eigen::VectorXd q = toPinocchioQ(seed_q_deg, impl_->model, impl_->joints);
    bool hit_joint_limit = clampJointLimits(&q, impl_->model, impl_->joints);
    const pinocchio::SE3 target_base =
        math::se3FromPose(mount.base_pose_in_stand).inverse() * math::se3FromPose(target_tcp_stand);
    double position_error_m = 0.0;
    double orientation_error_rad = 0.0;
    int iterations = 0;
    // Conditioning diagnostics from the most recent DLS step (carried to the
    // convergence return, where the step itself is not recomputed).
    double last_min_singular_value = 0.0;
    double last_applied_damping = eff_damping;

    for (; iterations <= config_.ik.max_iterations; ++iterations) {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed_ms =
            std::chrono::duration<double, std::milli>(now - started).count();
        if (elapsed_ms > config_.ik.timeout_ms) {
            return ik_solver::failureResult(
                ik_solver::kReasonTimeout,
                fromPinocchioQ(q, impl_->model, impl_->joints),
                position_error_m,
                orientation_error_rad,
                iterations,
                elapsedUs(),
                true
            );
        }

        pinocchio::forwardKinematics(impl_->model, impl_->data, q);
        pinocchio::computeJointJacobians(impl_->model, impl_->data, q);
        pinocchio::updateFramePlacements(impl_->model, impl_->data);

        const pinocchio::SE3& world_base = impl_->data.oMf[impl_->base_frame];
        const pinocchio::SE3& world_tip = impl_->data.oMf[impl_->tip_frame];
        const pinocchio::SE3 current_base = world_base.inverse() * world_tip;
        const pinocchio::SE3 current_to_target = current_base.actInv(target_base);
        const Eigen::Matrix<double, 6, 1> error = math::log6Local(current_base, target_base);
        position_error_m = error.head<3>().norm();
        orientation_error_rad = error.tail<3>().norm();

        if (!std::isfinite(position_error_m) || !std::isfinite(orientation_error_rad)) {
            return ik_solver::failureResult(
                ik_solver::kReasonInvalidTarget,
                fromPinocchioQ(q, impl_->model, impl_->joints),
                position_error_m,
                orientation_error_rad,
                iterations,
                elapsedUs()
            );
        }
        if (position_error_m <= config_.ik.position_tolerance_m &&
            orientation_error_rad <= config_.ik.orientation_tolerance_rad) {
            IkResult result;
            result.success = true;
            result.q_solution_deg = fromPinocchioQ(q, impl_->model, impl_->joints);
            result.position_error_m = position_error_m;
            result.orientation_error_rad = orientation_error_rad;
            result.duration_us = elapsedUs();
            result.iterations = iterations;
            result.min_singular_value = last_min_singular_value;
            result.applied_damping = last_applied_damping;
            result.solution_jump_deg = maxAbsJointDelta(result.q_solution_deg, seed_q_deg);
            result.branch_jump_suspected =
                config_.ik.max_solution_jump_deg > 0.0 &&
                result.solution_jump_deg > config_.ik.max_solution_jump_deg;
            populateBranchJumpDetails(
                result,
                seed_q_deg,
                config_.ik.max_solution_jump_deg,
                0
            );
            return result;
        }
        if (iterations == config_.ik.max_iterations) break;

        Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, impl_->model.nv);
        jacobian.setZero();
        pinocchio::getFrameJacobian(
            impl_->model,
            impl_->data,
            impl_->tip_frame,
            pinocchio::LOCAL,
            jacobian
        );
        const Eigen::Matrix<double, 6, 6> jlog =
            pinocchio::Jlog6(current_to_target.inverse());
        // Pinocchio CLIK convention: body-frame log residual with LOCAL frame
        // Jacobian requires the Jlog6 correction before the DLS update.
        const Eigen::MatrixXd task_jacobian = -jlog * jacobian;
        if (!task_jacobian.array().isFinite().all()) {
            return ik_solver::failureResult(
                ik_solver::kReasonSingularOrIllConditioned,
                fromPinocchioQ(q, impl_->model, impl_->joints),
                position_error_m,
                orientation_error_rad,
                iterations,
                elapsedUs()
            );
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(task_jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
        if (svd.singularValues().size() == 0 ||
            !svd.singularValues().array().isFinite().all() ||
            svd.singularValues().maxCoeff() < 1e-12) {
            return ik_solver::failureResult(
                ik_solver::kReasonSingularOrIllConditioned,
                fromPinocchioQ(q, impl_->model, impl_->joints),
                position_error_m,
                orientation_error_rad,
                iterations,
                elapsedUs()
            );
        }

        // Selective singularity-robust damped least squares via the SVD:
        //   dq = -V diag(sigma_i / (sigma_i^2 + lambda_i^2)) U^T error
        // lambda_i is the base damping outside the singular region and ramps up
        // to `damping_max` only on directions whose singular value falls below
        // `singular_region_eps`. This caps the inverse gain on the degenerate
        // direction (preventing a branch-flipping joint blow-up) while leaving
        // well-conditioned directions at full tracking accuracy.
        const Eigen::VectorXd& singular_values = svd.singularValues();
        const double base_lambda_sq = eff_damping * eff_damping;
        const double eps = config_.ik.singular_region_eps;
        const double extra_lambda_sq_max = eff_damping_max * eff_damping_max;
        last_min_singular_value = singular_values.minCoeff();
        double max_lambda_sq = base_lambda_sq;
        Eigen::VectorXd inv_factors(singular_values.size());
        for (Eigen::Index i = 0; i < singular_values.size(); ++i) {
            const double sigma = singular_values[i];
            double lambda_sq = base_lambda_sq;
            if (eps > 0.0 && extra_lambda_sq_max > 0.0 && sigma < eps) {
                const double ratio = sigma / eps;  // [0, 1)
                lambda_sq += extra_lambda_sq_max * (1.0 - ratio * ratio);
                max_lambda_sq = std::max(max_lambda_sq, lambda_sq);
            }
            inv_factors[i] = sigma / (sigma * sigma + lambda_sq);
        }
        last_applied_damping = std::sqrt(max_lambda_sq);

        const Eigen::VectorXd dq_full =
            -svd.matrixV() * (inv_factors.asDiagonal() *
                              (svd.matrixU().transpose() * error));
        if (!dq_full.array().isFinite().all()) {
            return ik_solver::failureResult(
                ik_solver::kReasonSingularOrIllConditioned,
                fromPinocchioQ(q, impl_->model, impl_->joints),
                position_error_m,
                orientation_error_rad,
                iterations,
                elapsedUs()
            );
        }

        applyJointStep(&q, clampJointStep(dq_full, config_, impl_->model, impl_->joints), impl_->model, impl_->joints);
        hit_joint_limit = clampJointLimits(&q, impl_->model, impl_->joints) || hit_joint_limit;
    }

    return ik_solver::failureResult(
        hit_joint_limit ? ik_solver::kReasonJointLimit : ik_solver::kReasonMaxIterations,
        fromPinocchioQ(q, impl_->model, impl_->joints),
        position_error_m,
        orientation_error_rad,
        iterations,
        elapsedUs()
    );
}

IkResult PinocchioKinematics::solveIk(
    ArmId arm,
    const Pose6D& target_tcp_stand,
    const JointArray& seed_q_deg,
    const ArmMountConfig& mount
) const {
    IkResult result = solveIkDamped(arm, target_tcp_stand, seed_q_deg, mount, 1.0);
    const double thresh = config_.ik.max_solution_jump_deg;
    // Feature off, solve failed, or no branch jump -> observability path unchanged.
    if (thresh <= 0.0 || !result.success || result.solution_jump_deg <= thresh) {
        return result;
    }

    // Branch-jump CLAMP step 1: re-solve the same tick with escalating damping so
    // the step stays on the local IK branch instead of flipping to a distant one.
    // The first attempt whose jump is within threshold wins; otherwise keep the
    // most-damped successful attempt as best-effort.
    const double scale = config_.ik.branch_jump_damping_scale;
    const int retries = config_.ik.branch_jump_max_retries;
    int retry_count = 0;
    if (scale > 1.0 && retries > 0) {
        double cur_scale = 1.0;
        for (int r = 0; r < retries; ++r) {
            cur_scale *= scale;
            IkResult retry =
                solveIkDamped(arm, target_tcp_stand, seed_q_deg, mount, cur_scale);
            retry_count = r + 1;
            if (retry.success && retry.solution_jump_deg <= thresh) {
                retry.branch_jump_retry_count = retry_count;
                return retry;  // higher damping resolved the jump -> smooth small step
            }
            if (retry.success) {
                result = retry;  // best-effort: carry the most-damped solution
                result.branch_jump_retry_count = retry_count;
            }
        }
    }

    // Branch-jump RATE-LIMIT (preferred over clamp): still jumping after damping
    // retries. Instead of freezing at the seed (which deadlocks under a moving
    // target) or accepting the full jump (rough), scale the whole seed->solution
    // joint delta so the largest per-joint step equals max_solution_jump_deg. The
    // arm advances toward the solution along the SAME joint-space direction at a
    // bounded joint speed: no deadlock (seed advances every tick) and no abrupt
    // flip. max_solution_jump_deg is the smoothness/lag knob.
    if (config_.ik.branch_jump_rate_limit) {
        const JointArray raw_solution = result.q_solution_deg;
        const double max_abs = maxAbsJointDelta(raw_solution, seed_q_deg);
        if (max_abs > thresh && max_abs > 0.0) {
            const double s = thresh / max_abs;
            IkResult limited = result;
            for (std::size_t i = 0; i < kDof; ++i) {
                limited.q_solution_deg[i] =
                    seed_q_deg[i] + (result.q_solution_deg[i] - seed_q_deg[i]) * s;
            }
            limited.success = true;
            limited.solution_jump_deg = thresh;
            limited.branch_jump_suspected = true;
            limited.branch_jump_clamped = false;
            limited.branch_jump_rate_limited = true;
            limited.branch_jump_scale = s;
            limited.reason = "branch_jump_rate_limited";
            populateBranchJumpDetails(limited, seed_q_deg, thresh, retry_count, raw_solution);
            return limited;
        }
        return result;
    }

    // Branch-jump CLAMP step 2: still jumping (or re-solve disabled). Hold the
    // seed so the downstream integrator produces zero motion this tick rather
    // than a violent branch-flip. Gated, so the default (clamp off) stays pure
    // observability (returns the flagged solution unchanged).
    if (config_.ik.branch_jump_clamp_to_seed) {
        const JointArray raw_solution = result.q_solution_deg;
        IkResult held = result;
        held.success = true;
        held.q_solution_deg = seed_q_deg;
        held.solution_jump_deg = 0.0;
        held.branch_jump_suspected = true;
        held.branch_jump_clamped = true;
        held.branch_jump_scale = 0.0;
        held.reason = "branch_jump_clamped_to_seed";
        populateBranchJumpDetails(held, seed_q_deg, thresh, retry_count, raw_solution);
        return held;
    }
    return result;
}

}  // namespace rb_servo
