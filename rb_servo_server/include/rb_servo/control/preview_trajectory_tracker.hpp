#pragma once

#include "rb_servo/core/types.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cstddef>
#include <limits>
#include <memory>

namespace rb_servo::control {

// A kinematic preview optimizer, not a backend or a low-pass filter. Reference
// poses are absolute stand poses at times relative to this proposed splice.
// The caller owns epochs, accepted-command state, arrival time and lifecycle.
struct PreviewTrackerConfig {
  double planning_dt_sec{0.01};
  std::size_t horizon_steps{24};  // 200--300 ms; at most 30 intervals
  // Physical limits have no usable defaults: every caller must supply them.
  double max_linear_velocity_m_s{0.0};       // per stand axis
  double max_linear_acceleration_m_s2{0.0};   // per stand axis
  double max_linear_jerk_m_s3{0.0};           // per stand axis
  // New explicit norm semantics, distinct from legacy per-tangent-axis caps.
  double max_angular_velocity_rad_s{0.0};
  double max_angular_acceleration_rad_s2{0.0};
  double max_angular_jerk_rad_s3{0.0}; // d(stand angular acceleration)/dt
  double linear_tracking_scale_m{0.01};
  double angular_tracking_scale_rad{0.03};
  double jerk_weight{0.02};
  double jerk_difference_weight{0.01};
  // Tracking is a soft objective. Slack is excess over these soft tolerances;
  // a trajectory whose dense tracking slack exceeds its budget is rejected.
  // Acceptance limits also require explicit caller values. NaN distinguishes
  // an omitted nonnegative tolerance from an intentionally exact zero budget.
  double linear_tracking_tolerance_m{std::numeric_limits<double>::quiet_NaN()};
  double angular_tracking_tolerance_rad{std::numeric_limits<double>::quiet_NaN()};
  double max_linear_tracking_slack_m{std::numeric_limits<double>::quiet_NaN()};
  double max_angular_tracking_slack_rad{std::numeric_limits<double>::quiet_NaN()};
  double max_reference_chart_angle_rad{0.0};
  double feasibility_tolerance{0.0};
  int max_working_set_recalculations{0}; // per axis, finite total <= 6 * this
  double max_solve_time_sec{0.0};        // total wall budget, checked after solve
};

struct PreviewTimedPose {
  double time_sec{0.0};
  Pose6D pose{};
};
struct PreviewReference {
  static constexpr std::size_t kCapacity = 32;
  std::array<PreviewTimedPose, kCapacity> knots{};
  std::size_t count{0};
};

// Conditional physical contact authority, independent of the tracking cursor.
// The bound is max(0, projection of UNRETIMED permitted canonical velocity)
// on one unit stand-frame force normal. Between knots it is piecewise linear;
// the producer must insert every zero crossing before taking the positive part.
// A changing physical normal is handled by the current servo guard; this
// snapshot is rechecked by the live guard. No position offset or brake allowance
// is granted: a seed faster into contact must first use the separate brake.
struct PreviewContactConstraint {
  static constexpr std::size_t kCapacity = 384;
  struct Knot { double time_sec{0.0}; double upper_velocity_m_s{0.0}; };
  bool enabled{false};
  Eigen::Vector3d normal_stand{Eigen::Vector3d::Zero()};
  std::array<Knot, kCapacity> knots{};
  std::size_t count{0};
};
// CoupledOnly is an offline validation oracle for the exact reduced solve.
enum class PreviewContactSolveMode { Automatic, CoupledOnly };

struct PreviewMotionState {
  Pose6D pose{};
  Eigen::Vector3d linear_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linear_acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity_body{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_acceleration_body{Eigen::Vector3d::Zero()};
};
struct PreviewMotionSample : PreviewMotionState {
  Eigen::Vector3d linear_jerk{Eigen::Vector3d::Zero()};
  // Physical angular jerk in stand coordinates. It is NOT just body alpha_dot.
  Eigen::Vector3d angular_jerk_stand{Eigen::Vector3d::Zero()};
};

// Shared Pinocchio/Eigen kinematics for a local exponential-coordinate curve.
// The fixed-size preview and braking samplers use the same physical derivative
// definition, including the stand-frame angular jerk term.
void previewAngularKinematics(const Eigen::Vector3d& position,
                              const Eigen::Vector3d& velocity,
                              const Eigen::Vector3d& acceleration,
                              const Eigen::Vector3d& jerk,
                              const Eigen::Matrix3d& rotation,
                              Eigen::Vector3d& angular_velocity_body,
                              Eigen::Vector3d& angular_acceleration_body,
                              Eigen::Vector3d& angular_jerk_stand);

// Fixed-size value exported by the planning thread. It owns no solver, heap
// storage or clock. Treat it as immutable after publishing it to the servo;
// sample() performs only bounded fixed-size Cartesian math and never extrapolates.
struct PreviewPolynomialTrajectory {
  static constexpr std::size_t kMaxHorizonSteps = 30;
  Eigen::Matrix<double, kMaxHorizonSteps + 1, 6, Eigen::RowMajor> p{}, v{}, a{};
  Eigen::Matrix<double, kMaxHorizonSteps, 6, Eigen::RowMajor> jerk{};
  Eigen::Matrix3d rotation0{Eigen::Matrix3d::Identity()};
  double step_sec{0.0};
  std::size_t count{0};
  bool valid{false};

  double durationSec() const { return count * step_sec; }
  bool sample(double relative_time_sec, PreviewMotionSample& output) const;
};

enum class PreviewSolveStatus {
  Solved,
  InvalidReference,
  InvalidInitialState,
  Infeasible,
  IterationLimit,
  TimeBudgetExceeded,
  NumericalFailure,
  TrackingBudgetExceeded
};
struct PreviewSolveDiagnostics {
  PreviewSolveStatus status{PreviewSolveStatus::InvalidReference};
  double solve_time_sec{0.0};
  int working_set_recalculations{0};
  double max_constraint_violation{0.0};
  double max_position_tracking_error_m{0.0};
  double max_orientation_tracking_error_rad{0.0};
  double max_position_tracking_slack_m{0.0};
  double max_orientation_tracking_slack_rad{0.0};
  double max_linear_velocity{0.0};
  double max_linear_acceleration{0.0};
  double max_linear_jerk{0.0};
  double max_angular_velocity_norm{0.0};
  double max_angular_acceleration_norm{0.0};
  double max_angular_jerk_norm{0.0};
  // Continuous exponential-coordinate norm certificate, distinct from sampled
  // physical derivatives. Coupling is needed only when independent optima fail it.
  bool angular_norm_coupled{false};
  std::size_t angular_norm_cuts{0};
  double max_angular_chart_velocity_norm{0.0};
  double max_angular_chart_acceleration_norm{0.0};
  bool contact_constrained{false};
  bool contact_decomposed{false};
  bool contact_coupled_fallback{false};
  std::size_t contact_constraint_rows{0};
  double max_contact_velocity_violation_m_s{0.0};
};
struct PreviewSolveResult {
  PreviewSolveStatus status{PreviewSolveStatus::InvalidReference};
  PreviewSolveDiagnostics diagnostics{};
  bool accepted() const { return status == PreviewSolveStatus::Solved; }
};

class PreviewTrajectoryTracker {
 public:
  static constexpr std::size_t kMaxHorizonSteps = 30;
  explicit PreviewTrajectoryTracker(const PreviewTrackerConfig& config);
  ~PreviewTrajectoryTracker();
  PreviewTrajectoryTracker(const PreviewTrajectoryTracker&) = delete;
  PreviewTrajectoryTracker& operator=(const PreviewTrajectoryTracker&) = delete;

  // Constructor allocates/prewarms fixed-sized QPs. A failed plan never replaces
  // the previously accepted trajectory or resets its caller-owned time origin.
  // This does not authorize replaying an old trajectory after an epoch change.
  PreviewSolveResult plan(const PreviewReference& reference,
                          const PreviewMotionState& initial);
  PreviewSolveResult plan(const PreviewReference& reference,
                         const PreviewMotionState& initial,
                         const PreviewContactConstraint& contact,
                         PreviewContactSolveMode mode = PreviewContactSolveMode::Automatic);
  // Absolute evaluation of the accepted piecewise-cubic trajectory: no dt
  // integration drift, no allocation, p/v/a continuous between 10ms intervals.
  // Returns false outside [0,duration]; it never extrapolates an expired plan.
  bool sample(double relative_time_sec, PreviewMotionSample& output) const;
  // Copies fixed-size coefficients only. A reset/failed first plan has no export.
  bool exportTrajectory(PreviewPolynomialTrajectory& output) const;
  bool hasTrajectory() const;
  double durationSec() const;
  void reset();
  // Common force/fold gauge: every position += translation; every orientation
  // left-composed by rotation. Positions/linear derivatives are not rotated.
  // Body angular derivatives are invariant; stand angular jerk rotates.
  void shiftCommonFrame(const Eigen::Vector3d& translation_stand,
                        const Eigen::Quaterniond& left_rotation_stand);
  const PreviewTrackerConfig& config() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace rb_servo::control
