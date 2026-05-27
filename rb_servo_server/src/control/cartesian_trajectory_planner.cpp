#include "rb_servo/control/cartesian_trajectory_planner.hpp"

#include "rb_servo/math/se3.hpp"

namespace rb_servo {

Pose6D LinearCartesianPlanner::sample(
    const CartesianTrajectoryRequest& request,
    double s
) const {
    return math::interpolateLinear(
        request.start_tcp_stand,
        request.target_tcp_stand,
        request.orientation_mode == CartesianOrientationInterpolation::Slerp,
        s
    );
}

}  // namespace rb_servo
