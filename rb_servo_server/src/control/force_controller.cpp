#include "rb_servo/control/force_controller.hpp"

#include <algorithm>

namespace rb_servo {
namespace {

double clampAbs(double v, double limit) {
    return std::clamp(v, -limit, limit);
}

}  // namespace

ForceController::ForceController(const ForceControlConfig& config) : config_(config) {}

void ForceController::reset() {
    filtered_wrench_ = Wrench6D{};
    offset_ = Pose6D{};
    initialized_ = false;
}

Wrench6D ForceController::lowPass(const Wrench6D& measured) {
    const double a = std::clamp(config_.force_lpf_alpha, 0.0, 1.0);
    if (!initialized_) {
        filtered_wrench_ = measured;
        initialized_ = true;
        return filtered_wrench_;
    }
    filtered_wrench_.fx = (1.0 - a) * filtered_wrench_.fx + a * measured.fx;
    filtered_wrench_.fy = (1.0 - a) * filtered_wrench_.fy + a * measured.fy;
    filtered_wrench_.fz = (1.0 - a) * filtered_wrench_.fz + a * measured.fz;
    filtered_wrench_.tx = (1.0 - a) * filtered_wrench_.tx + a * measured.tx;
    filtered_wrench_.ty = (1.0 - a) * filtered_wrench_.ty + a * measured.ty;
    filtered_wrench_.tz = (1.0 - a) * filtered_wrench_.tz + a * measured.tz;
    return filtered_wrench_;
}

Pose6D ForceController::computeTcpCompensation(
    const Pose6D& current_tcp_stand,
    const Pose6D& reference_tcp_stand,
    const Wrench6D& measured_wrench_tcp,
    const ForceControlCommand& command,
    double dt_sec
) {
    (void)current_tcp_stand;
    (void)reference_tcp_stand;
    if (!config_.enable || command.mode == ForceControlMode::Off || dt_sec <= 0.0) {
        return Pose6D{};
    }

    const Wrench6D wrench = lowPass(measured_wrench_tcp);
    Pose6D desired = offset_;

    // Minimal admittance fallback: offset_dot = gain * (target - measured).
    // Axis sign convention must be validated with the real TCP/sensor frame before use.
    if (command.mode == ForceControlMode::Admittance || command.mode == ForceControlMode::ExternalForceSafety) {
        if (command.enabled_axis.x) desired.x += config_.admittance_gain_pos * (command.target_wrench.fx - wrench.fx) * dt_sec;
        if (command.enabled_axis.y) desired.y += config_.admittance_gain_pos * (command.target_wrench.fy - wrench.fy) * dt_sec;
        if (command.enabled_axis.z) desired.z += config_.admittance_gain_pos * (command.target_wrench.fz - wrench.fz) * dt_sec;
        if (command.enabled_axis.roll) desired.rx += config_.admittance_gain_rot * (command.target_wrench.tx - wrench.tx) * dt_sec;
        if (command.enabled_axis.pitch) desired.ry += config_.admittance_gain_rot * (command.target_wrench.ty - wrench.ty) * dt_sec;
        if (command.enabled_axis.yaw) desired.rz += config_.admittance_gain_rot * (command.target_wrench.tz - wrench.tz) * dt_sec;
    }

    offset_ = clampStepAndOffset(desired, command);
    return offset_;
}

Pose6D ForceController::clampStepAndOffset(const Pose6D& desired, const ForceControlCommand& command) const {
    const double max_pos_offset = command.max_pos_offset_m > 0.0 ? command.max_pos_offset_m : config_.max_pos_offset_m;
    const double max_rot_offset = command.max_rot_offset_rad > 0.0 ? command.max_rot_offset_rad : config_.max_rot_offset_rad;
    const double max_pos_step = command.max_pos_step_m > 0.0 ? command.max_pos_step_m : config_.max_pos_step_m;
    const double max_rot_step = command.max_rot_step_rad > 0.0 ? command.max_rot_step_rad : config_.max_rot_step_rad;

    Pose6D out;
    out.x = clampAbs(offset_.x + clampAbs(desired.x - offset_.x, max_pos_step), max_pos_offset);
    out.y = clampAbs(offset_.y + clampAbs(desired.y - offset_.y, max_pos_step), max_pos_offset);
    out.z = clampAbs(offset_.z + clampAbs(desired.z - offset_.z, max_pos_step), max_pos_offset);
    out.rx = clampAbs(offset_.rx + clampAbs(desired.rx - offset_.rx, max_rot_step), max_rot_offset);
    out.ry = clampAbs(offset_.ry + clampAbs(desired.ry - offset_.ry, max_rot_step), max_rot_offset);
    out.rz = clampAbs(offset_.rz + clampAbs(desired.rz - offset_.rz, max_rot_step), max_rot_offset);
    return out;
}

}  // namespace rb_servo
