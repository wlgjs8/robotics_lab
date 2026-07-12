#pragma once

#include "rb_servo/config/config.hpp"
#include "rb_servo/core/types.hpp"

#include <cstdint>
#include <string>

namespace rb_servo {

struct ForceControllerState {
    Pose6D offset_tcp;
    Vec6 velocity_tcp;
    Vec6 acceleration_tcp;
    double observed_energy_j = 0.0;
};

struct ForceControllerProposal {
    ForceControllerState state;
    Wrench6D wrench_error_tcp;
    std::array<bool, 6> limit_axes{};
    std::string limit_reason;
    bool translation_recenter_coupled = false;
    bool rotation_recenter_coupled = false;
    bool translation_recenter_deferred = false;
    bool rotation_recenter_deferred = false;
    bool valid = false;
    bool saturated = false;
    std::string reason;
    uint64_t controller_id = 0;
    uint64_t lifecycle_generation = 0;
    uint64_t base_state_revision = 0;
};

// Project-native bounded Cartesian admittance. DualArmServoLoop supplies the
// wrench and measured twist in the configured compliance frame, then commits
// the proposal only after the corrected Cartesian target passes IK, every
// final safety gate, and send.
class ForceController {
public:
    explicit ForceController(ForceControlConfig config);

    void engage();
    void release();
    ForceControllerProposal propose(
        const Wrench6D& measured_external_wrench_tcp,
        const ForceControlCommand& command,
        const Vec6& measured_actual_twist_tcp,
        double dt_sec
    ) const;
    bool commit(const ForceControllerProposal& proposal);
    void reject();
    void reset();

    const ForceControllerState& state() const { return state_; }
    bool engaged() const { return engaged_; }

private:
    ForceControlConfig config_;
    ForceControllerState state_{};
    bool engaged_ = false;
    uint64_t controller_id_ = 0;
    uint64_t lifecycle_generation_ = 0;
    uint64_t state_revision_ = 0;
};

}  // namespace rb_servo
