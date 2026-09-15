#pragma once

#include <Eigen/Core>
#include <ruckig/ruckig.hpp>

namespace rb_servo::experimental {

// Explicit physical parameters; zero-initialized configurations are invalid.
// The stiffness is a local contact predictor, not a certified environment bound.
struct ForceReferenceConfig {
    double sustained_force_n{0};
    double recovery_force_n{0};
    double noise_force_n{0};
    double mass_kg{0};
    double damping_ns_m{0};
    double contact_stiffness_n_m{0};
    double max_velocity_m_s{0};
    double max_acceleration_m_s2{0};
    double max_jerk_m_s3{0};
};

struct ForceReferenceState {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
};

struct ForceReferenceResult {
    ForceReferenceState state{};
    Eigen::Vector3d requested_velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d compliant_velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d predicted_force{Eigen::Vector3d::Zero()};
    double measured_force_n{0};
    bool constraint_active{false};
    bool valid{false};
    const char* reason{"uninitialized"};
};

// OFFLINE CANDIDATE ONLY: not connected to the servo loop or launch configuration.
// One generated Cartesian translation state. There is no displacement overlay,
// return-to-offset motion, force gate, contact latch or arm/release timer.
// External force and measured position must refer to the same TCP and stand frame.
// Gravity, bias AND tool inertia must already be removed. The current production
// F/T pipeline does not provide this last operation; do not feed it directly here.
// Stability under measurement/model error and multiple contacts is unverified.
// The caller retains source freshness, dispatch feedback, rotation and IK safety.
class ForceReference {
public:
    ForceReference(const ForceReferenceConfig& config, double period_sec);
    static void validate(const ForceReferenceConfig& config, double period_sec);
    void reset(const ForceReferenceState& accepted);
    void clear();
    bool initialized() const { return initialized_; }
    const ForceReferenceState& state() const { return state_; }
    const ForceReferenceConfig& config() const { return config_; }

    ForceReferenceResult step(const Eigen::Vector3d& target_position,
                              const Eigen::Vector3d& target_velocity,
                              const Eigen::Vector3d& measured_position,
                              const Eigen::Vector3d& external_force);

    // Velocity intent has no inaccessible position target to catch up to on
    // release. A stopped source must supply zero velocity; an absolute goal
    // requires a caller-owned, explicitly speed-bounded pursuit task. Do not
    // integrate rejected displacement and feed it back as a later velocity.
    // Like step(), this advances a generated state, not an acknowledged robot
    // state. A refused dispatch requires clear/reset from an accepted seed.
    ForceReferenceResult stepVelocity(const Eigen::Vector3d& requested_velocity,
                                      const Eigen::Vector3d& measured_position,
                                      const Eigen::Vector3d& external_force);

    // Read-only offline inspection of the most recent generated servo interval.
    // Endpoint acceleration alone can miss an entire sub-tick Ruckig ramp.
    // This sampler covers only [0, period_sec], never an arbitrary future plan.
    bool sampleLastStep(double relative_time_sec, ForceReferenceState& output) const;

private:
    ForceReferenceConfig config_;
    double dt_;
    ForceReferenceState state_{};
    bool initialized_{false};
    bool last_step_valid_{false};
    ruckig::Ruckig<3> trajectory_;
    ruckig::InputParameter<3> input_;
    ruckig::OutputParameter<3> output_;
};

} // namespace rb_servo::experimental
