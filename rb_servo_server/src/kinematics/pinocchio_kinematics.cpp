#include "rb_servo/kinematics/pinocchio_kinematics.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#if defined(RB_SERVO_ENABLE_PINOCCHIO) && RB_SERVO_ENABLE_PINOCCHIO
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/se3.hpp>
#endif

namespace rb_servo {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

double degToRad(double deg) {
    return deg * kPi / 180.0;
}

bool finitePose(const Pose6D& pose) {
    return std::isfinite(pose.x) &&
           std::isfinite(pose.y) &&
           std::isfinite(pose.z) &&
           std::isfinite(pose.rx) &&
           std::isfinite(pose.ry) &&
           std::isfinite(pose.rz);
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

Eigen::Matrix3d rotationFromPose(const Pose6D& pose) {
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
    const Eigen::Vector3d rpy = placement.rotation().eulerAngles(0, 1, 2);
    Pose6D pose;
    pose.x = placement.translation().x();
    pose.y = placement.translation().y();
    pose.z = placement.translation().z();
    pose.rx = rpy.x();
    pose.ry = rpy.y();
    pose.rz = rpy.z();
    if (!finitePose(pose)) {
        throw std::runtime_error("Pinocchio FK produced a non-finite TCP pose");
    }
    return pose;
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

}  // namespace rb_servo
