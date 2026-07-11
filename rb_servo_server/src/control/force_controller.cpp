#include "rb_servo/control/force_controller.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <utility>

namespace rb_servo {
namespace {

using AxisValues = std::array<double, 6>;

std::atomic<uint64_t> g_next_controller_id{1};

AxisValues poseValues(const Pose6D& value) {
    return {value.x, value.y, value.z, value.rx, value.ry, value.rz};
}

AxisValues vecValues(const Vec6& value) {
    return {value.x, value.y, value.z, value.rx, value.ry, value.rz};
}

AxisValues wrenchValues(const Wrench6D& value) {
    return {value.fx, value.fy, value.fz, value.tx, value.ty, value.tz};
}

Pose6D toPose(const AxisValues& value) {
    return {value[0], value[1], value[2], value[3], value[4], value[5]};
}

Vec6 toVec(const AxisValues& value) {
    return {value[0], value[1], value[2], value[3], value[4], value[5]};
}

Wrench6D toWrench(const AxisValues& value) {
    return {value[0], value[1], value[2], value[3], value[4], value[5]};
}

bool allFinite(const AxisValues& value) {
    return std::all_of(value.begin(), value.end(), [](double item) {
        return std::isfinite(item);
    });
}

AxisValues enabledAxes(const ForceControlAxis& axis) {
    return {
        axis.x ? 1.0 : 0.0,
        axis.y ? 1.0 : 0.0,
        axis.z ? 1.0 : 0.0,
        axis.roll ? 1.0 : 0.0,
        axis.pitch ? 1.0 : 0.0,
        axis.yaw ? 1.0 : 0.0,
    };
}

double tighten(double server_limit, double command_limit) {
    return command_limit > 0.0 ? std::min(server_limit, command_limit) : server_limit;
}

double clampAbs(double value, double limit, bool& saturated) {
    const double clamped = std::clamp(value, -limit, limit);
    saturated = saturated || clamped != value;
    return clamped;
}

}  // namespace

ForceController::ForceController(ForceControlConfig config)
    : config_(std::move(config)),
      controller_id_(g_next_controller_id.fetch_add(1, std::memory_order_relaxed)) {}

void ForceController::reset() {
    state_ = ForceControllerState{};
    engaged_ = false;
    ++lifecycle_generation_;
    ++state_revision_;
}

void ForceController::engage() {
    reset();
    engaged_ = true;
}

void ForceController::release() {
    reset();
}

void ForceController::reject() {
    // Deliberately retain the last committed state. A rejected IK/safety result
    // must never advance the controller's internal integrators.
}

bool ForceController::commit(const ForceControllerProposal& proposal) {
    if (!proposal.valid || !engaged_ || proposal.controller_id != controller_id_ ||
        proposal.lifecycle_generation != lifecycle_generation_ ||
        proposal.base_state_revision != state_revision_) {
        return false;
    }
    state_ = proposal.state;
    ++state_revision_;
    return true;
}

ForceControllerProposal ForceController::propose(
    const Wrench6D& measured_external_wrench_tcp,
    const ForceControlCommand& command,
    const Vec6& measured_actual_twist_tcp,
    double dt_sec
) const {
    ForceControllerProposal proposal;
    proposal.state = state_;
    proposal.controller_id = controller_id_;
    proposal.lifecycle_generation = lifecycle_generation_;
    proposal.base_state_revision = state_revision_;

    if (!config_.enable) {
        proposal.reason = "force control disabled";
        return proposal;
    }
    if (!engaged_) {
        proposal.reason = "force controller is not engaged";
        return proposal;
    }
    if (command.mode != ForceControlMode::Admittance &&
        command.mode != ForceControlMode::ExternalForceSafety) {
        proposal.reason = "unsupported or inactive force-control mode";
        return proposal;
    }
    if (!std::isfinite(dt_sec) || dt_sec <= 0.0 || dt_sec > config_.max_dt_sec) {
        proposal.reason = "invalid controller dt";
        return proposal;
    }

    const AxisValues measured = wrenchValues(measured_external_wrench_tcp);
    const AxisValues target = wrenchValues(command.target_wrench);
    const AxisValues actual_twist = vecValues(measured_actual_twist_tcp);
    if (!allFinite(measured) || !allFinite(target) || !allFinite(actual_twist)) {
        proposal.reason = "non-finite controller input";
        return proposal;
    }

    const AxisValues old_offset = poseValues(state_.offset_tcp);
    const AxisValues old_velocity = vecValues(state_.velocity_tcp);
    const AxisValues old_acceleration = vecValues(state_.acceleration_tcp);
    const AxisValues enabled = enabledAxes(command.enabled_axis);
    AxisValues next_offset = old_offset;
    AxisValues next_velocity{};
    AxisValues next_acceleration{};
    AxisValues wrench_error{};

    const double pos_offset = tighten(config_.max_pos_offset_m, command.max_pos_offset_m);
    const double rot_offset = tighten(config_.max_rot_offset_rad, command.max_rot_offset_rad);
    const double pos_step = tighten(config_.max_pos_step_m, command.max_pos_step_m);
    const double rot_step = tighten(config_.max_rot_step_rad, command.max_rot_step_rad);

    for (std::size_t i = 0; i < 6; ++i) {
        if (enabled[i] == 0.0) continue;

        wrench_error[i] = measured[i] - target[i];
        const double raw_acceleration =
            (wrench_error[i] - config_.damping[i] * old_velocity[i] -
             config_.stiffness[i] * old_offset[i]) /
            config_.virtual_mass[i];

        const bool translation = i < 3;
        const double acceleration_limit = translation
            ? config_.max_linear_acceleration_m_s2
            : config_.max_angular_acceleration_rad_s2;
        const double jerk_limit = translation
            ? config_.max_linear_jerk_m_s3
            : config_.max_angular_jerk_rad_s3;
        const double velocity_limit = translation
            ? config_.max_linear_velocity_m_s
            : config_.max_angular_velocity_rad_s;
        const double offset_limit = translation ? pos_offset : rot_offset;
        const double step_limit = translation ? pos_step : rot_step;

        double acceleration = clampAbs(raw_acceleration, acceleration_limit, proposal.saturated);
        acceleration = old_acceleration[i] + clampAbs(
            acceleration - old_acceleration[i], jerk_limit * dt_sec, proposal.saturated);
        double velocity = clampAbs(
            old_velocity[i] + acceleration * dt_sec,
            velocity_limit,
            proposal.saturated
        );
        const double raw_step = velocity * dt_sec;
        const double step = clampAbs(raw_step, step_limit, proposal.saturated);
        const double offset = clampAbs(old_offset[i] + step, offset_limit, proposal.saturated);

        // Back-calculate velocity when a position/step cap is active so the
        // integrator cannot wind up behind a saturated output.
        next_offset[i] = offset;
        next_velocity[i] = (offset - old_offset[i]) / dt_sec;
        const double realized_acceleration =
            (next_velocity[i] - old_velocity[i]) / dt_sec;
        const double realized_jerk =
            (realized_acceleration - old_acceleration[i]) / dt_sec;
        constexpr double kRelativeLimitTolerance = 1e-9;
        const double acceleration_tolerance =
            kRelativeLimitTolerance * std::max(1.0, acceleration_limit);
        const double jerk_tolerance =
            kRelativeLimitTolerance * std::max(1.0, jerk_limit);
        if (std::abs(realized_acceleration) > acceleration_limit + acceleration_tolerance ||
            std::abs(realized_jerk) > jerk_limit + jerk_tolerance) {
            proposal.reason = "output bounds are infeasible at the hard offset/step limit";
            return proposal;
        }
        next_acceleration[i] = realized_acceleration;
    }

    double power_w = 0.0;
    for (std::size_t i = 0; i < 6; ++i) power_w += measured[i] * actual_twist[i];
    const double energy = std::max(0.0, state_.observed_energy_j + power_w * dt_sec);
    if (!std::isfinite(energy) || energy > config_.max_energy_j) {
        proposal.reason = "passivity energy limit exceeded";
        return proposal;
    }

    proposal.state.offset_tcp = toPose(next_offset);
    proposal.state.velocity_tcp = toVec(next_velocity);
    proposal.state.acceleration_tcp = toVec(next_acceleration);
    proposal.state.observed_energy_j = energy;
    proposal.wrench_error_tcp = toWrench(wrench_error);
    proposal.valid = true;
    proposal.reason = proposal.saturated ? "bounded" : "ok";
    return proposal;
}

}  // namespace rb_servo
