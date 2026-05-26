#include "rb_servo/kinematics/pinocchio_kinematics.hpp"

#include "rb_servo/kinematics/ik_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#if defined(RB_SERVO_ENABLE_PINOCCHIO) && RB_SERVO_ENABLE_PINOCCHIO
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/se3.hpp>
#endif

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

bool finiteTwist(const Vec6& twist) {
    return std::isfinite(twist.x) &&
           std::isfinite(twist.y) &&
           std::isfinite(twist.z) &&
           std::isfinite(twist.rx) &&
           std::isfinite(twist.ry) &&
           std::isfinite(twist.rz);
}

}  // namespace

#if defined(RB_SERVO_ENABLE_PINOCCHIO) && RB_SERVO_ENABLE_PINOCCHIO

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

Eigen::Matrix3d rotationFromPose(const Pose6D& pose) {
    if (pose.quaternion_xyzw.has_value()) {
        const auto& q = *pose.quaternion_xyzw;
        const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        if (!std::isfinite(norm) || norm <= 0.0) {
            throw std::runtime_error("Pose6D quaternion is non-finite or zero length");
        }
        Eigen::Quaterniond quaternion(q[3] / norm, q[0] / norm, q[1] / norm, q[2] / norm);
        return quaternion.toRotationMatrix();
    }
    const Eigen::AngleAxisd roll(pose.rx, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch(pose.ry, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw(pose.rz, Eigen::Vector3d::UnitZ());
    return (yaw * pitch * roll).toRotationMatrix();
}

pinocchio::SE3 se3FromPose(const Pose6D& pose) {
    return pinocchio::SE3(
        rotationFromPose(pose),
        Eigen::Vector3d(pose.x, pose.y, pose.z)
    );
}

Pose6D poseFromSe3(const pinocchio::SE3& placement) {
    const Eigen::Vector3d ypr = placement.rotation().eulerAngles(2, 1, 0);
    Eigen::Quaterniond quaternion(placement.rotation());
    quaternion.normalize();
    Pose6D pose;
    pose.x = placement.translation().x();
    pose.y = placement.translation().y();
    pose.z = placement.translation().z();
    pose.rx = ypr.z();
    pose.ry = ypr.y();
    pose.rz = ypr.x();
    pose.quaternion_xyzw = std::array<double, 4>{
        quaternion.x(),
        quaternion.y(),
        quaternion.z(),
        quaternion.w(),
    };
    if (!finitePose(pose)) {
        throw std::runtime_error("Pinocchio FK produced a non-finite TCP pose");
    }
    return pose;
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

#else

struct PinocchioKinematics::Impl {};

#endif

PinocchioKinematics::PinocchioKinematics(KinematicsConfig config)
    : config_(std::move(config)) {
#if defined(RB_SERVO_ENABLE_PINOCCHIO) && RB_SERVO_ENABLE_PINOCCHIO
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
#endif
}

PinocchioKinematics::~PinocchioKinematics() = default;
PinocchioKinematics::PinocchioKinematics(PinocchioKinematics&&) noexcept = default;
PinocchioKinematics& PinocchioKinematics::operator=(PinocchioKinematics&&) noexcept = default;

bool PinocchioKinematics::isAvailable() {
#if defined(RB_SERVO_ENABLE_PINOCCHIO) && RB_SERVO_ENABLE_PINOCCHIO
    return true;
#else
    return false;
#endif
}

Pose6D PinocchioKinematics::computeTcpBase(const JointArray& q_deg) const {
#if defined(RB_SERVO_ENABLE_PINOCCHIO) && RB_SERVO_ENABLE_PINOCCHIO
    if (!impl_) {
        throw std::runtime_error("Pinocchio kinematics is not initialized");
    }
    Eigen::VectorXd q = toPinocchioQ(q_deg, impl_->model, impl_->joints);
    pinocchio::forwardKinematics(impl_->model, impl_->data, q);
    pinocchio::updateFramePlacements(impl_->model, impl_->data);

    const pinocchio::SE3& world_base = impl_->data.oMf[impl_->base_frame];
    const pinocchio::SE3& world_tip = impl_->data.oMf[impl_->tip_frame];
    return poseFromSe3(world_base.inverse() * world_tip);
#else
    (void)q_deg;
    throw std::runtime_error("Pinocchio kinematics is unavailable; rebuild with RB_SERVO_ENABLE_PINOCCHIO=ON");
#endif
}

Pose6D PinocchioKinematics::computeTcpStand(
    ArmId arm,
    const JointArray& q_deg,
    const ArmMountConfig& mount
) const {
    (void)arm;
#if defined(RB_SERVO_ENABLE_PINOCCHIO) && RB_SERVO_ENABLE_PINOCCHIO
    const Pose6D tcp_base = computeTcpBase(q_deg);
    return poseFromSe3(se3FromPose(mount.base_pose_in_stand) * se3FromPose(tcp_base));
#else
    (void)q_deg;
    (void)mount;
    throw std::runtime_error("Pinocchio kinematics is unavailable; rebuild with RB_SERVO_ENABLE_PINOCCHIO=ON");
#endif
}

IkResult PinocchioKinematics::solveIk(
    ArmId arm,
    const Pose6D& target_tcp_stand,
    const JointArray& seed_q_deg,
    const ArmMountConfig& mount
) const {
    (void)arm;
    const auto started = std::chrono::steady_clock::now();
    const auto elapsedUs = [&]() {
        return std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - started
        ).count();
    };
#if defined(RB_SERVO_ENABLE_PINOCCHIO) && RB_SERVO_ENABLE_PINOCCHIO
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
        se3FromPose(mount.base_pose_in_stand).inverse() * se3FromPose(target_tcp_stand);
    double position_error_m = 0.0;
    double orientation_error_rad = 0.0;
    int iterations = 0;

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
        const Eigen::Matrix<double, 6, 1> error = pinocchio::log6(current_to_target).toVector();
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

        const double lambda = config_.ik.damping;
        const Eigen::Matrix<double, 6, 6> dls_matrix =
            task_jacobian * task_jacobian.transpose() +
            (lambda * lambda) * Eigen::Matrix<double, 6, 6>::Identity();
        const Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(dls_matrix);
        if (ldlt.info() != Eigen::Success) {
            return ik_solver::failureResult(
                ik_solver::kReasonSingularOrIllConditioned,
                fromPinocchioQ(q, impl_->model, impl_->joints),
                position_error_m,
                orientation_error_rad,
                iterations,
                elapsedUs()
            );
        }

        const Eigen::VectorXd dq_full = -task_jacobian.transpose() * ldlt.solve(error);
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
#else
    (void)target_tcp_stand;
    (void)mount;
    return ik_solver::failureResult(
        ik_solver::kReasonKinematicsUnavailable,
        seed_q_deg,
        0.0,
        0.0,
        0,
        elapsedUs()
    );
#endif
}

CartesianVelocityResult PinocchioKinematics::solveCartesianVelocity(
    ArmId arm,
    const JointArray& q_deg,
    const ArmMountConfig& mount,
    const Vec6& tcp_twist_local,
    double damping
) const {
    (void)arm;
    (void)mount;
    CartesianVelocityResult result;
#if defined(RB_SERVO_ENABLE_PINOCCHIO) && RB_SERVO_ENABLE_PINOCCHIO
    if (!config_.enable || !config_.ik.enable || !impl_) {
        result.reason = ik_solver::kReasonKinematicsUnavailable;
        return result;
    }
    if (!ik_solver::isFiniteJoints(q_deg) || !finiteTwist(tcp_twist_local)) {
        result.reason = ik_solver::kReasonInvalidTarget;
        return result;
    }

    const Eigen::VectorXd q = toPinocchioQ(q_deg, impl_->model, impl_->joints);
    pinocchio::forwardKinematics(impl_->model, impl_->data, q);
    pinocchio::computeJointJacobians(impl_->model, impl_->data, q);
    pinocchio::updateFramePlacements(impl_->model, impl_->data);

    Eigen::Matrix<double, 6, Eigen::Dynamic> full_jacobian(6, impl_->model.nv);
    full_jacobian.setZero();
    pinocchio::getFrameJacobian(
        impl_->model,
        impl_->data,
        impl_->tip_frame,
        pinocchio::LOCAL,
        full_jacobian
    );
    Eigen::Matrix<double, 6, 6> jacobian;
    jacobian.setZero();
    for (std::size_t i = 0; i < impl_->joints.size(); ++i) {
        const pinocchio::JointIndex joint_id = impl_->joints[i];
        jacobian.col(static_cast<Eigen::Index>(i)) = full_jacobian.col(impl_->model.idx_vs[joint_id]);
    }
    if (!jacobian.array().isFinite().all()) {
        result.reason = ik_solver::kReasonSingularOrIllConditioned;
        return result;
    }

    Eigen::Matrix<double, 6, 1> twist;
    twist << tcp_twist_local.x,
             tcp_twist_local.y,
             tcp_twist_local.z,
             tcp_twist_local.rx,
             tcp_twist_local.ry,
             tcp_twist_local.rz;
    const double lambda = std::max(0.0, damping);
    const Eigen::Matrix<double, 6, 6> dls_matrix =
        jacobian * jacobian.transpose() +
        (lambda * lambda) * Eigen::Matrix<double, 6, 6>::Identity();
    const Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(dls_matrix);
    if (ldlt.info() != Eigen::Success) {
        result.reason = ik_solver::kReasonSingularOrIllConditioned;
        return result;
    }
    const Eigen::Matrix<double, 6, 1> qdot_rad_s = jacobian.transpose() * ldlt.solve(twist);
    if (!qdot_rad_s.array().isFinite().all()) {
        result.reason = ik_solver::kReasonSingularOrIllConditioned;
        return result;
    }
    for (int i = 0; i < kDof; ++i) {
        result.qdot_deg_s[i] = radToDeg(qdot_rad_s[i]);
    }
    result.success = true;
    return result;
#else
    (void)q_deg;
    (void)tcp_twist_local;
    (void)damping;
    result.reason = ik_solver::kReasonKinematicsUnavailable;
    return result;
#endif
}

}  // namespace rb_servo
