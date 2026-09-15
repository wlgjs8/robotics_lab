#pragma once

#include <Eigen/Core>
#include <array>
#include <cstddef>
#include <memory>

namespace rb_servo::experimental {

// Offline translation experiment. No robot, force gate, position target,
// displacement accumulator, source queue, or deployment configuration.
struct VelocityPreviewConfig {
    double step_sec{0};
    std::size_t horizon_steps{0}; // 1..30
    double max_velocity_m_s{0};     // per fixed Cartesian axis
    double max_acceleration_m_s2{0}; // per fixed Cartesian axis
    double max_jerk_m_s3{0};         // per fixed Cartesian axis
    double tracking_scale_m_s{0};
    double acceleration_weight{0}; // squared a/max_acceleration, target zero
    double jerk_weight{0};
    double jerk_difference_weight{0};
    double feasibility_tolerance{0};
    int max_working_set_recalculations{0};
    double max_solve_time_sec{0};
};
struct VelocityPreviewState {
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
};
struct VelocityPreviewSample : VelocityPreviewState {
    Eigen::Vector3d jerk{Eigen::Vector3d::Zero()};
};
struct VelocityPreviewReference {
    static constexpr std::size_t kCapacity=32;
    struct Knot { double time_sec{0}; Eigen::Vector3d velocity{Eigen::Vector3d::Zero()}; };
    std::array<Knot,kCapacity> knots{};
    std::size_t count{0};
};
struct VelocityPreviewTrajectory {
    static constexpr std::size_t kCapacity=30;
    std::array<Eigen::Vector3d,kCapacity+1> velocity{}, acceleration{};
    std::array<Eigen::Vector3d,kCapacity> jerk{};
    double step_sec{0};
    std::size_t count{0};
    bool valid{false};
    bool sample(double relative_time_sec,VelocityPreviewSample& output) const;
    double durationSec() const { return count*step_sec; }
};
struct VelocityPreviewResult {
    bool valid{false};
    const char* reason{"invalid_reference"};
    double solve_time_sec{0};
    double max_constraint_violation{0};
    int working_set_recalculations{0};
};

// Minimizes velocity tracking error, acceleration and jerk over the explicitly supplied
// known horizon. Position is absent from BOTH the objective and state, so
// rejected motion cannot become a positional catch-up term. Each plan starts
// with the caller's previous nominal velocity/acceleration, not a clipped seed.
// The caller owns source age/epoch, plan splice/expiry and force dispatch. A
// force controller may consume each nominal velocity sample at its servo rate;
// preview completion must never gate acquisition of a fresh force sample.
// Continuous derivative limits use sufficient Bernstein bounds, which can
// refuse a physically feasible state close to a velocity limit. Refusal is
// explicit and never clips the seed or grants an old plan a renewed lifetime.
class VelocityPreview {
public:
    explicit VelocityPreview(const VelocityPreviewConfig& config);
    ~VelocityPreview();
    VelocityPreview(const VelocityPreview&)=delete;
    VelocityPreview& operator=(const VelocityPreview&)=delete;
    VelocityPreviewResult plan(const VelocityPreviewReference& reference,
                               const VelocityPreviewState& initial);
    const VelocityPreviewTrajectory& trajectory() const;
    void clear();
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace rb_servo::experimental
