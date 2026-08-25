#pragma once

#include "rb_servo/config/config.hpp"
#include "rb_servo/core/types.hpp"

namespace rb_servo {

struct SafetyTrackingState {
    bool override_tracking_q = false;
    JointArray tracking_q_deg{};
    std::string source = "actual";
    bool source_valid = true;
    std::string reason;
    double command_reference_tracking_error_deg = 0.0;
    double physical_command_actual_error_deg = 0.0;
    bool controller_simulation_physical_motion_detected = false;
    bool controller_simulation_physical_motion_fault = false;
    // controller-simulation: the tracking "reference" is the controller's own
    // reported jnt_ref, which does NOT advance while the (no-motion) sim servo is
    // disabled. Comparing the streaming command to it pins the command after
    // ~max_tracking_error_deg. When advisory, report the tracking error but do
    // NOT snap the command back, so streaming Cartesian accumulates the full path.
    bool tracking_error_advisory = false;
};

struct SafetyCheckResult {
    JointArray filtered_q_deg{};
    SafetyVerdict verdict = SafetyVerdict::Ok;
    bool ok = true;
    bool joint_limit_clamped = false;
    std::string reason;
    SafetyTrackingTelemetry tracking;
    SafetyClampTelemetry clamp;
};

class SafetyFilter {
public:
    explicit SafetyFilter(const SafetyConfig& config);

    SafetyCheckResult filterJointTarget(
        const JointArray& desired_q_deg,
        const JointArray& previous_q_deg,
        const JointArray& previous_previous_q_deg,
        const RobotState& state,
        double dt_sec,
        const SafetyTrackingState& tracking_state = SafetyTrackingState{}
    ) const;

    SafetyVerdict checkState(const RobotState& state) const;
    bool hasTrackingError(
        const JointArray& previous_q_deg,
        const RobotState& state,
        const SafetyTrackingState& tracking_state = SafetyTrackingState{}
    ) const;

    bool isStateSafe(const RobotState& state) const;

    bool shouldStopBothArms(
        const RobotState& left_state,
        const RobotState& right_state
    ) const;

    // Joint-limit + velocity + acceleration clamp chain applied to a desired target.
    // Exposed so callers that bypass the tracking-error hold (e.g. the controller-sim
    // tracking-error advisory) still rate-limit the followed target. `clamped` (if
    // non-null) reports whether a joint-limit clamp occurred.
    JointArray clampMotion(
        const JointArray& desired_q_deg,
        const JointArray& previous_q_deg,
        const JointArray& previous_previous_q_deg,
        double dt_sec,
        bool* clamped = nullptr
    ) const;

    SafetyClampTelemetry clampMotionDetailed(
        const JointArray& desired_q_deg,
        const JointArray& previous_q_deg,
        const JointArray& previous_previous_q_deg,
        double dt_sec
    ) const;

private:
    JointArray clampJointLimits(const JointArray& q, bool* clamped) const;

    // Approach barrier for the joint range: inside d_slow of a bound, cap the CLOSING
    // joint speed at sqrt(2*a_brake*margin) so the joint coasts to a stop AT the bound
    // instead of arriving at full speed and pinning. Motion away from a bound is never
    // touched. Inert when safety.joint_limit_barrier.enable is false.
    JointArray applyJointLimitBarrier(
        const JointArray& q,
        const JointArray& q_prev,
        double dt_sec
    ) const;

    JointArray clampVelocity(
        const JointArray& q,
        const JointArray& q_prev,
        double dt_sec
    ) const;

    JointArray clampAcceleration(
        const JointArray& q,
        const JointArray& q_prev,
        const JointArray& q_prevprev,
        double dt_sec
    ) const;

private:
    SafetyConfig config_;
};

}  // namespace rb_servo
