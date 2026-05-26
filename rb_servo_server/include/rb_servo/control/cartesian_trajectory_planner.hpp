#pragma once

#include "rb_servo/core/types.hpp"

namespace rb_servo {

enum class CartesianOrientationInterpolation {
    Constant,
    Slerp
};

struct CartesianTrajectoryRequest {
    Pose6D start_tcp_stand;
    Pose6D target_tcp_stand;
    CartesianOrientationInterpolation orientation_mode = CartesianOrientationInterpolation::Slerp;
};

class CartesianTrajectoryPlanner {
public:
    virtual ~CartesianTrajectoryPlanner() = default;

    virtual Pose6D sample(
        const CartesianTrajectoryRequest& request,
        double s
    ) const = 0;
};

class LinearCartesianPlanner final : public CartesianTrajectoryPlanner {
public:
    Pose6D sample(
        const CartesianTrajectoryRequest& request,
        double s
    ) const override;
};

}  // namespace rb_servo
