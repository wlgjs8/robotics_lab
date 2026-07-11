#include "rb_servo/control/normal_force_controller.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace rb_servo {
namespace {

std::atomic<uint64_t> g_next_normal_force_controller_id{1};

bool finitePositive(double value) {
    return std::isfinite(value) && value > 0.0;
}

bool finiteNonNegative(double value) {
    return std::isfinite(value) && value >= 0.0;
}

void validateConfig(const NormalForceControllerConfig& config) {
    if (!finitePositive(config.virtual_mass_kg)) {
        throw std::invalid_argument("normal force virtual mass must be positive and finite");
    }
    if (!finiteNonNegative(config.damping_n_s_per_m) ||
        !finiteNonNegative(config.stiffness_n_per_m) ||
        !finiteNonNegative(config.force_deadband_n)) {
        throw std::invalid_argument(
            "normal force damping, stiffness, and deadband must be non-negative and finite"
        );
    }
    if (!finitePositive(config.max_dt_sec) ||
        !finitePositive(config.max_unload_offset_m) ||
        !finitePositive(config.max_unload_velocity_m_s) ||
        !finitePositive(config.max_unload_acceleration_m_s2) ||
        !finitePositive(config.max_unload_jerk_m_s3) ||
        !finitePositive(config.max_unload_step_m) ||
        !finitePositive(config.max_observed_energy_j)) {
        throw std::invalid_argument("normal force controller bounds must be positive and finite");
    }
}

double applyContinuousDeadband(double error, double deadband, bool* in_deadband) {
    const double magnitude = std::abs(error);
    if (magnitude <= deadband) {
        *in_deadband = true;
        return 0.0;
    }
    return std::copysign(magnitude - deadband, error);
}

struct AccelerationInterval {
    double lower = 0.0;
    double upper = 0.0;
};

bool within(double value, double lower, double upper) {
    constexpr double kTolerance = 1e-12;
    return value >= lower - kTolerance && value <= upper + kTolerance;
}

// Test whether a jerk-limited braking trajectory can avoid the positive
// position/velocity boundaries. This look-ahead is what a plain velocity clamp
// lacks: arriving at vmax with positive acceleration is already too late to
// respect the jerk limit on the following tick.
bool upperBrakingFeasible(
    double offset,
    double velocity,
    double acceleration,
    const NormalForceControllerConfig& config,
    double dt_sec,
    double velocity_limit
) {
    if (!within(offset, 0.0, config.max_unload_offset_m) ||
        !within(velocity, -velocity_limit, velocity_limit)) {
        return false;
    }
    if (velocity <= 0.0) {
        return true;
    }
    if (velocity > 0.0 &&
        (velocity > velocity_limit + 1e-12 ||
         offset > config.max_unload_offset_m + 1e-12)) {
        return false;
    }

    const double jerk_step = config.max_unload_jerk_m_s3 * dt_sec;
    const auto ramp_to_zero_velocity_delta = [&](double candidate_acceleration) {
        if (candidate_acceleration >= 0.0) {
            return 0.0;
        }
        const double ramp_steps = std::ceil(
            -candidate_acceleration / jerk_step
        ) - 1.0;
        const double negative_steps = std::max(0.0, ramp_steps);
        return dt_sec * (
            negative_steps * candidate_acceleration +
            jerk_step * negative_steps * (negative_steps + 1.0) * 0.5
        );
    };
    // The bound follows from the maximum time needed to ramp acceleration from
    // +amax to -amax plus the maximum time needed to remove vmax at -amax.
    const double required_steps = std::ceil(
        2.0 * config.max_unload_acceleration_m_s2 / jerk_step +
        2.0 * velocity_limit /
            (config.max_unload_acceleration_m_s2 * dt_sec)
    ) + 4.0;
    constexpr double kMaxBrakingLookaheadSteps = 4096.0;
    if (!std::isfinite(required_steps) ||
        required_steps > kMaxBrakingLookaheadSteps) {
        return false;
    }
    const int max_steps = static_cast<int>(required_steps);
    for (int i = 0; i < max_steps; ++i) {
        const double more_negative = std::max(
            -config.max_unload_acceleration_m_s2,
            acceleration - jerk_step
        );
        const auto terminal_velocity = [&](double candidate_acceleration) {
            return velocity + candidate_acceleration * dt_sec +
                ramp_to_zero_velocity_delta(candidate_acceleration);
        };
        if (terminal_velocity(more_negative) >= 0.0) {
            acceleration = more_negative;
        } else {
            double infeasible = more_negative;
            double feasible = std::min(
                config.max_unload_acceleration_m_s2,
                acceleration + jerk_step
            );
            if (terminal_velocity(feasible) < -1e-12) {
                // Even maximum positive jerk cannot finish at v=0,a=0 without
                // first reversing. Reversal is nevertheless safe for this
                // one-sided boundary, so follow that jerk and stop checking as
                // soon as velocity points away from the boundary.
                acceleration = feasible;
            } else {
                for (int search = 0; search < 40; ++search) {
                    const double midpoint = 0.5 * (infeasible + feasible);
                    if (terminal_velocity(midpoint) >= 0.0) {
                        feasible = midpoint;
                    } else {
                        infeasible = midpoint;
                    }
                }
                acceleration = feasible;
            }
        }
        velocity += acceleration * dt_sec;
        offset += velocity * dt_sec;
        if (velocity > 0.0 &&
            (velocity > velocity_limit + 1e-12 ||
             offset > config.max_unload_offset_m + 1e-12)) {
            return false;
        }
        if (velocity <= 1e-12) {
            return true;
        }
    }
    return false;
}

bool lowerBrakingFeasible(
    double offset,
    double velocity,
    double acceleration,
    const NormalForceControllerConfig& config,
    double dt_sec,
    double velocity_limit
) {
    // Mirror the scalar state about the midpoint so the positive-boundary test
    // also covers return motion toward the unilateral x=0 boundary.
    return upperBrakingFeasible(
        config.max_unload_offset_m - offset,
        -velocity,
        -acceleration,
        config,
        dt_sec,
        velocity_limit
    );
}

bool tightenToJerkLimitedEnvelope(
    const NormalForceControllerState& state,
    const NormalForceControllerConfig& config,
    double dt_sec,
    double velocity_limit,
    AccelerationInterval* interval
) {
    const auto upper_ok = [&](double acceleration) {
        const double velocity = state.unload_velocity_m_s + acceleration * dt_sec;
        const double offset = state.unload_offset_m + velocity * dt_sec;
        return upperBrakingFeasible(
            offset, velocity, acceleration, config, dt_sec, velocity_limit
        );
    };
    const auto lower_ok = [&](double acceleration) {
        const double velocity = state.unload_velocity_m_s + acceleration * dt_sec;
        const double offset = state.unload_offset_m + velocity * dt_sec;
        return lowerBrakingFeasible(
            offset, velocity, acceleration, config, dt_sec, velocity_limit
        );
    };

    if (!upper_ok(interval->upper)) {
        if (!upper_ok(interval->lower)) {
            return false;
        }
        double feasible = interval->lower;
        double infeasible = interval->upper;
        for (int i = 0; i < 40; ++i) {
            const double midpoint = 0.5 * (feasible + infeasible);
            if (upper_ok(midpoint)) {
                feasible = midpoint;
            } else {
                infeasible = midpoint;
            }
        }
        interval->upper = std::max(interval->lower, feasible - 1e-5);
    }

    if (!lower_ok(interval->lower)) {
        if (!lower_ok(interval->upper)) {
            return false;
        }
        double infeasible = interval->lower;
        double feasible = interval->upper;
        for (int i = 0; i < 40; ++i) {
            const double midpoint = 0.5 * (infeasible + feasible);
            if (lower_ok(midpoint)) {
                feasible = midpoint;
            } else {
                infeasible = midpoint;
            }
        }
        interval->lower = std::min(interval->upper, feasible + 1e-5);
    }
    return interval->lower <= interval->upper + 1e-12;
}

}  // namespace

NormalForceController::NormalForceController(NormalForceControllerConfig config)
    : config_(std::move(config)),
      controller_id_(
          g_next_normal_force_controller_id.fetch_add(1, std::memory_order_relaxed)
      ) {
    validateConfig(config_);
}

void NormalForceController::reset() {
    state_ = NormalForceControllerState{};
    engaged_ = false;
    ++lifecycle_generation_;
    ++state_revision_;
}

void NormalForceController::engage() {
    reset();
    engaged_ = true;
}

void NormalForceController::release() {
    reset();
}

void NormalForceController::reject() {
    // Retain the last committed state. Downstream rejection must not advance
    // the admittance integrators or passivity observer.
}

bool NormalForceController::commit(const NormalForceControllerProposal& proposal) {
    if (!proposal.valid || !engaged_ || proposal.controller_id != controller_id_ ||
        proposal.lifecycle_generation != lifecycle_generation_ ||
        proposal.base_state_revision != state_revision_) {
        return false;
    }
    state_ = proposal.state;
    ++state_revision_;
    return true;
}

NormalForceControllerProposal NormalForceController::propose(
    double measured_contact_force_n,
    const NormalForceControllerCommand& command,
    double measured_actual_normal_velocity_m_s,
    double dt_sec
) const {
    NormalForceControllerProposal proposal;
    proposal.state = state_;
    proposal.controller_id = controller_id_;
    proposal.lifecycle_generation = lifecycle_generation_;
    proposal.base_state_revision = state_revision_;

    if (!config_.enable) {
        proposal.reason = "normal force control disabled";
        return proposal;
    }
    if (!engaged_) {
        proposal.reason = "normal force controller is not engaged";
        return proposal;
    }
    if (!std::isfinite(measured_contact_force_n) ||
        !std::isfinite(measured_actual_normal_velocity_m_s) ||
        !std::isfinite(command.target_contact_force_n) ||
        command.target_contact_force_n < 0.0) {
        proposal.reason = "invalid normal force controller input";
        return proposal;
    }
    if (!std::isfinite(dt_sec) || dt_sec <= 0.0 || dt_sec > config_.max_dt_sec) {
        proposal.reason = "invalid normal force controller dt";
        return proposal;
    }

    proposal.force_error_n =
        measured_contact_force_n - command.target_contact_force_n;
    if (command.brake_to_hold) {
        proposal.controlled_force_error_n = 0.0;
        proposal.in_deadband = true;
    } else {
        proposal.controlled_force_error_n = applyContinuousDeadband(
            proposal.force_error_n,
            config_.force_deadband_n,
            &proposal.in_deadband
        );
    }

    const double raw_acceleration =
        (proposal.controlled_force_error_n -
         config_.damping_n_s_per_m * state_.unload_velocity_m_s -
         config_.stiffness_n_per_m * state_.unload_offset_m) /
        config_.virtual_mass_kg;

    const double velocity_limit = std::min(
        config_.max_unload_velocity_m_s,
        config_.max_unload_step_m / dt_sec
    );
    const double jerk_step = config_.max_unload_jerk_m_s3 * dt_sec;
    AccelerationInterval interval{
        std::max(
            -config_.max_unload_acceleration_m_s2,
            state_.unload_acceleration_m_s2 - jerk_step
        ),
        std::min(
            config_.max_unload_acceleration_m_s2,
            state_.unload_acceleration_m_s2 + jerk_step
        )
    };
    interval.lower = std::max(interval.lower, (
        -velocity_limit - state_.unload_velocity_m_s
    ) / dt_sec);
    interval.upper = std::min(interval.upper, (
        velocity_limit - state_.unload_velocity_m_s
    ) / dt_sec);
    interval.lower = std::max(interval.lower, (
        -state_.unload_offset_m / dt_sec - state_.unload_velocity_m_s
    ) / dt_sec);
    interval.upper = std::min(interval.upper, (
        (config_.max_unload_offset_m - state_.unload_offset_m) / dt_sec -
        state_.unload_velocity_m_s
    ) / dt_sec);

    if (interval.lower > interval.upper + 1e-6) {
        proposal.reason =
            "normal force controller immediate motion bounds are infeasible";
        return proposal;
    }
    if (interval.lower > interval.upper) {
        const double boundary = 0.5 * (interval.lower + interval.upper);
        interval.lower = boundary;
        interval.upper = boundary;
    }
    if (!tightenToJerkLimitedEnvelope(
            state_, config_, dt_sec, velocity_limit, &interval
        )) {
        proposal.reason =
            "normal force controller state is outside the jerk-limited motion envelope";
        return proposal;
    }
    if (interval.lower > interval.upper) {
        const double boundary = 0.5 * (interval.lower + interval.upper);
        interval.lower = boundary;
        interval.upper = boundary;
    }

    double acceleration = std::clamp(
        raw_acceleration, interval.lower, interval.upper
    );
    proposal.saturated = acceleration != raw_acceleration;
    double velocity = std::clamp(
        state_.unload_velocity_m_s + acceleration * dt_sec,
        -velocity_limit,
        velocity_limit
    );
    double offset = std::clamp(
        state_.unload_offset_m + velocity * dt_sec,
        0.0,
        config_.max_unload_offset_m
    );
    // Reconcile only floating-point boundary residue. The look-ahead envelope
    // has already made the clamp dynamically feasible; unlike the old path,
    // this must never absorb a material acceleration/jerk discontinuity.
    velocity = (offset - state_.unload_offset_m) / dt_sec;
    acceleration = (velocity - state_.unload_velocity_m_s) / dt_sec;
    if (std::abs(acceleration) > config_.max_unload_acceleration_m_s2 + 1e-6 ||
        std::abs(acceleration - state_.unload_acceleration_m_s2) >
            config_.max_unload_jerk_m_s3 * dt_sec + 1e-6) {
        proposal.reason =
            "normal force controller numerical boundary reconciliation exceeded limits";
        return proposal;
    }

    // Positive measured velocity is outward, matching the positive contact
    // reaction direction used for the scalar power calculation.
    const double power_w =
        measured_contact_force_n * measured_actual_normal_velocity_m_s;
    const double observed_energy_j = std::max(
        0.0,
        state_.observed_energy_j + power_w * dt_sec
    );
    if (!std::isfinite(observed_energy_j) ||
        observed_energy_j > config_.max_observed_energy_j) {
        proposal.reason = "normal force passivity energy limit exceeded";
        return proposal;
    }

    proposal.state.unload_offset_m = offset;
    proposal.state.unload_velocity_m_s = velocity;
    proposal.state.unload_acceleration_m_s2 = acceleration;
    proposal.state.observed_energy_j = observed_energy_j;
    proposal.valid = true;
    proposal.reason = proposal.saturated
        ? "bounded"
        : (command.brake_to_hold ? "braking" :
            (proposal.in_deadband ? "deadband" : "ok"));
    return proposal;
}

}  // namespace rb_servo
