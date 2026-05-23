#include "rb_servo/kinematics/ik_solver.hpp"

#include <cmath>
#include <utility>

namespace rb_servo {

namespace ik_solver {

bool isFinitePose(const Pose6D& pose) {
    return std::isfinite(pose.x) &&
           std::isfinite(pose.y) &&
           std::isfinite(pose.z) &&
           std::isfinite(pose.rx) &&
           std::isfinite(pose.ry) &&
           std::isfinite(pose.rz);
}

bool isFiniteJoints(const JointArray& joints) {
    for (double joint : joints) {
        if (!std::isfinite(joint)) return false;
    }
    return true;
}

IkResult failureResult(
    const std::string& reason,
    const JointArray& q_solution_deg,
    double position_error_m,
    double orientation_error_rad,
    int iterations,
    double duration_us,
    bool timed_out
) {
    IkResult result;
    result.success = false;
    result.q_solution_deg = q_solution_deg;
    result.position_error_m = position_error_m;
    result.orientation_error_rad = orientation_error_rad;
    result.duration_us = duration_us;
    result.iterations = iterations;
    result.timed_out = timed_out;
    result.reason = reason;
    return result;
}

}  // namespace ik_solver
}  // namespace rb_servo
