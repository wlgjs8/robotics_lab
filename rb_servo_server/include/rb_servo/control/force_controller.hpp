#pragma once

#include "rb_servo/config/config.hpp"
#include "rb_servo/core/types.hpp"
#include "rb_servo/math/se3.hpp"

#include <cstdint>
#include <string>

namespace rb_servo {

// Episode-scoped measured contact normal. The first active sample captures
// -normalize(force_control_stand); subsequent active samples cannot steer it.
// Leaving the episode invalidates the frame. A failed entry capture remains
// invalid until the next episode instead of silently guessing a direction.
class ContactForceNormalEstimator {
public:
    void update(bool episode_active, const math::Vector3& force_control_stand);
    void reset();
    bool valid() const { return valid_; }
    const math::Vector3& normalStand() const { return normal_stand_; }

private:
    bool episode_active_ = false;
    bool valid_ = false;
    math::Vector3 normal_stand_{math::Vector3::Zero()};
};

struct ForceControllerState {
    Pose6D offset_tcp;
    Vec6 velocity_tcp;
    Vec6 acceleration_tcp;
    double observed_energy_j = 0.0;
    // Committed Layer-3 envelope opening (velocity boost, m/s). The boost
    // rises instantly with the wrench-error excess but may only DECAY at a
    // rate the jerk governor can brake against — a step collapse (fast force
    // release) strands a legally-fast state outside the shrunk envelope and
    // faults the proposal (2026-07-23 12:55/13:08 runs).
    double envelope_boost_m_s = 0.0;
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

// Inverse of the applyForceCorrection composition: strip a COMMITTED compliance
// offset from a measured stand-frame pose. Follower divergence checks use this
// so deliberate compliance never counts as tracking divergence (actual lead
// measures the servo's true plan-vs-robot split, not the admittance offset).
// t_tcp_compliance_pose is the compliance frame in the TCP frame (the same
// transform applyForceCorrection composed the offset with); offset_tcp is the
// committed ForceControllerState offset. A zero offset returns the input pose.
Pose6D removeComplianceOffsetFromMeasured(const Pose6D& measured_stand,
                                          const Pose6D& t_tcp_compliance_pose,
                                          const Pose6D& offset_tcp);

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
