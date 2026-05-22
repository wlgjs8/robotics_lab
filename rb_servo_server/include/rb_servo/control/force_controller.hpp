#pragma once

#include "rb_servo/config/config.hpp"
#include "rb_servo/core/types.hpp"

namespace rb_servo {

// Lightweight force/admittance design placeholder.
// This is intentionally independent from DualArmServoLoop until CartesianController/IK is enabled.
class ForceController {
public:
    explicit ForceController(const ForceControlConfig& config);

    void reset();

    Pose6D computeTcpCompensation(
        const Pose6D& current_tcp_stand,
        const Pose6D& reference_tcp_stand,
        const Wrench6D& measured_wrench_tcp,
        const ForceControlCommand& command,
        double dt_sec
    );

private:
    Wrench6D lowPass(const Wrench6D& measured);
    Pose6D clampStepAndOffset(const Pose6D& desired, const ForceControlCommand& command) const;

private:
    ForceControlConfig config_;
    Wrench6D filtered_wrench_{};
    Pose6D offset_{};
    bool initialized_ = false;
};

}  // namespace rb_servo
