#pragma once

#include <cstdint>
#include <string>

namespace rb_servo {

// Scalar contact-normal admittance. Positive force is compressive contact force;
// positive offset/velocity is the accepted outward (unloading) direction.
// The controller may return toward the nominal pose, but can never command an
// inward correction beyond it.
struct NormalForceControllerConfig {
    bool enable = false;
    double virtual_mass_kg = 5.0;
    double damping_n_s_per_m = 80.0;
    double stiffness_n_per_m = 0.0;
    double force_deadband_n = 0.5;
    double max_dt_sec = 0.02;
    double max_unload_offset_m = 0.01;
    double max_unload_velocity_m_s = 0.02;
    double max_unload_acceleration_m_s2 = 0.2;
    double max_unload_jerk_m_s3 = 2.0;
    double max_unload_step_m = 0.001;
    double max_observed_energy_j = 2.0;
};

struct NormalForceControllerCommand {
    // Non-negative desired compressive contact force. A positive target does
    // not seek contact beyond the nominal pose because unload offset is
    // constrained to be non-negative.
    double target_contact_force_n = 0.0;
};

struct NormalForceControllerState {
    double unload_offset_m = 0.0;
    double unload_velocity_m_s = 0.0;
    double unload_acceleration_m_s2 = 0.0;
    double observed_energy_j = 0.0;
};

struct NormalForceControllerProposal {
    NormalForceControllerState state;
    double force_error_n = 0.0;
    double controlled_force_error_n = 0.0;
    bool valid = false;
    bool saturated = false;
    bool in_deadband = false;
    std::string reason;
    uint64_t controller_id = 0;
    uint64_t lifecycle_generation = 0;
    uint64_t base_state_revision = 0;
};

class NormalForceController {
public:
    explicit NormalForceController(NormalForceControllerConfig config);

    void engage();
    void release();
    NormalForceControllerProposal propose(
        double measured_contact_force_n,
        const NormalForceControllerCommand& command,
        double measured_actual_normal_velocity_m_s,
        double dt_sec
    ) const;
    bool commit(const NormalForceControllerProposal& proposal);
    void reject();
    void reset();

    const NormalForceControllerState& state() const { return state_; }
    bool engaged() const { return engaged_; }

private:
    NormalForceControllerConfig config_;
    NormalForceControllerState state_{};
    bool engaged_ = false;
    uint64_t controller_id_ = 0;
    uint64_t lifecycle_generation_ = 0;
    uint64_t state_revision_ = 0;
};

}  // namespace rb_servo
