#pragma once

#include <string>

#include "rb_servo/kinematics/i_kinematics.hpp"

namespace rb_servo::ik_solver {

constexpr const char* kReasonMaxIterations = "max_iterations";
constexpr const char* kReasonTimeout = "timeout";
constexpr const char* kReasonSingularOrIllConditioned = "singular_or_ill_conditioned";
constexpr const char* kReasonJointLimit = "joint_limit";
// A joint-limit-clamped iterate accepted as a success because its residual was
// inside ik.joint_limit_best_effort_* (see config.hpp). Success, not failure:
// the arm keeps tracking every direction the pinned joint does not block.
constexpr const char* kReasonJointLimitBestEffort = "joint_limit_best_effort";
// A non-converged iterate accepted as a success because its residual was inside
// ik.max_iterations_best_effort_* (see config.hpp). Success, not failure: near a
// singularity the DLS damping ramp shrinks the step until the iteration budget runs
// out while the residual is already micrometres from the target, and refusing the
// tick there is what makes the arm chatter.
constexpr const char* kReasonMaxIterationsBestEffort = "max_iterations_best_effort";
constexpr const char* kReasonInvalidTarget = "invalid_target";
constexpr const char* kReasonKinematicsUnavailable = "kinematics_unavailable";

bool isFinitePose(const Pose6D& pose);
bool isFiniteJoints(const JointArray& joints);
IkResult failureResult(
    const std::string& reason,
    const JointArray& q_solution_deg,
    double position_error_m = 0.0,
    double orientation_error_rad = 0.0,
    int iterations = 0,
    double duration_us = 0.0,
    bool timed_out = false
);

}  // namespace rb_servo::ik_solver
