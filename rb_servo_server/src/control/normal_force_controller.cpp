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

double clampAbs(double value, double limit, bool* saturated) {
    const double clamped = std::clamp(value, -limit, limit);
    *saturated = *saturated || clamped != value;
    return clamped;
}

double applyContinuousDeadband(double error, double deadband, bool* in_deadband) {
    const double magnitude = std::abs(error);
    if (magnitude <= deadband) {
        *in_deadband = true;
        return 0.0;
    }
    return std::copysign(magnitude - deadband, error);
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
    proposal.controlled_force_error_n = applyContinuousDeadband(
        proposal.force_error_n,
        config_.force_deadband_n,
        &proposal.in_deadband
    );

    const double raw_acceleration =
        (proposal.controlled_force_error_n -
         config_.damping_n_s_per_m * state_.unload_velocity_m_s -
         config_.stiffness_n_per_m * state_.unload_offset_m) /
        config_.virtual_mass_kg;

    double acceleration = clampAbs(
        raw_acceleration,
        config_.max_unload_acceleration_m_s2,
        &proposal.saturated
    );
    acceleration = state_.unload_acceleration_m_s2 + clampAbs(
        acceleration - state_.unload_acceleration_m_s2,
        config_.max_unload_jerk_m_s3 * dt_sec,
        &proposal.saturated
    );
    double velocity = clampAbs(
        state_.unload_velocity_m_s + acceleration * dt_sec,
        config_.max_unload_velocity_m_s,
        &proposal.saturated
    );
    const double step = clampAbs(
        velocity * dt_sec,
        config_.max_unload_step_m,
        &proposal.saturated
    );
    const double unclamped_offset = state_.unload_offset_m + step;
    const double offset = std::clamp(
        unclamped_offset,
        0.0,
        config_.max_unload_offset_m
    );
    proposal.saturated = proposal.saturated || offset != unclamped_offset;

    // Back-calculate dynamics from the correction that can actually be emitted.
    // This prevents hidden wind-up at either unilateral position boundary.
    velocity = (offset - state_.unload_offset_m) / dt_sec;
    const double realized_acceleration =
        (velocity - state_.unload_velocity_m_s) / dt_sec;
    const double realized_jerk =
        (realized_acceleration - state_.unload_acceleration_m_s2) / dt_sec;
    constexpr double kRelativeLimitTolerance = 1e-9;
    const double acceleration_tolerance = kRelativeLimitTolerance *
        std::max(1.0, config_.max_unload_acceleration_m_s2);
    const double jerk_tolerance = kRelativeLimitTolerance *
        std::max(1.0, config_.max_unload_jerk_m_s3);
    if (std::abs(realized_acceleration) >
            config_.max_unload_acceleration_m_s2 + acceleration_tolerance ||
        std::abs(realized_jerk) > config_.max_unload_jerk_m_s3 + jerk_tolerance) {
        proposal.reason = "normal force output bounds are infeasible at unilateral limit";
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
    proposal.state.unload_acceleration_m_s2 = realized_acceleration;
    proposal.state.observed_energy_j = observed_energy_j;
    proposal.valid = true;
    proposal.reason = proposal.saturated
        ? "bounded"
        : (proposal.in_deadband ? "deadband" : "ok");
    return proposal;
}

}  // namespace rb_servo
