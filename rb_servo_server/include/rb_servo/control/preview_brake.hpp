#pragma once

#include "rb_servo/control/preview_trajectory_tracker.hpp"
#include <ruckig/ruckig.hpp>

namespace rb_servo::control {

enum class PreviewBrakeStatus {
  Inactive, Ready, InvalidInitialState, InitialOutsideLimits,
  Infeasible, LimitViolation, NumericalFailure
};
const char* previewBrakeStatusName(PreviewBrakeStatus status);

// Ruckig 0.9.2's velocity interface does not set Profile::pf, while its extrema
// routine reads pf. Use the integrated terminal position, never that unused
// position-interface target. Exposed for the compatibility regression test.
ruckig::PositionExtrema previewBrakeIntegratedPositionExtrema(ruckig::Profile profile);

// Fixed-size snapshot: no worker/solver ownership or clock. A completed brake
// can be sampled as an explicit stationary hold, so future worker splices after
// its finite stop deadline inherit the same terminal p/v/a. The caller still
// owns lifecycle/epoch validity and must not use this across a reset.
struct PreviewBrakeTrajectory {
  ruckig::Trajectory<6> trajectory;
  Eigen::Matrix3d rotation0{Eigen::Matrix3d::Identity()};
  // Orthonormal local chart basis: physical rotation is R0*exp(B*theta).
  // Rotating this basis preserves the conservative physical norm budgets.
  Eigen::Matrix3d angular_basis{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d translation_offset{Eigen::Vector3d::Zero()};
  bool valid{false};
  double durationSec() const;
  bool sample(double relative_time_sec, PreviewMotionSample& output) const;
  void shiftCommonFrame(const Eigen::Vector3d& translation_stand,
                        const Eigen::Quaterniond& left_rotation_stand);
};

// Fixed angular part of a translation-only contact stop. This descriptor owns
// no solver and never invents a future stopping seed. The runtime may retain an
// accepted polynomial only through its original deadline, then install an
// angular brake calculated from an actually accepted sample. sample() uses an
// ABSOLUTE clock and changes only orientation and angular derivatives.
class PreviewAngularContinuation {
 public:
  enum class Kind { None, Polynomial, Brake };
  void clear();
  bool retainPolynomial(const PreviewPolynomialTrajectory& polynomial,
                        double origin_sec, double original_valid_until_sec);
  bool startBrake(const PreviewBrakeTrajectory& brake, double origin_sec);
  Kind kind() const { return kind_; }
  bool retainsPolynomial() const { return kind_==Kind::Polynomial; }
  bool needsBrake(double absolute_time_sec) const;
  bool sample(double absolute_time_sec, PreviewMotionSample& inout) const;
  bool terminalHoldAvailableAt(double accepted_absolute_time_sec) const;
  void shiftCommonFrame(const Eigen::Vector3d& translation_stand,
                        const Eigen::Quaterniond& left_rotation_stand);

 private:
  Kind kind_{Kind::None};
  PreviewPolynomialTrajectory polynomial_;
  PreviewBrakeTrajectory brake_;
  double origin_sec_{0.0};
  double original_valid_until_sec_{0.0};
};

// Prewarmed, fixed-DOF Ruckig velocity-mode stop. Physical caps come only from
// the existing preview tracker configuration. Initial p/v/a are never clipped.
// Identity chart is preferred. If its angular box cannot hold the accepted
// state, one quaternion-derived balanced basis is tried with the SAME box and
// physical norm budgets. States not fitting either basis still refuse explicitly.
class PreviewBrake {
 public:
  PreviewBrake(const PreviewTrackerConfig& config, double servo_period_sec);
  PreviewBrakeStatus start(const PreviewMotionState& initial);
  void reset();
  bool sample(double relative_time_sec, PreviewMotionSample& output) const;
  bool exportTrajectory(PreviewBrakeTrajectory& output) const;
  double durationSec() const;
  PreviewBrakeStatus status() const { return status_; }

 private:
  PreviewTrackerConfig config_;
  ruckig::Ruckig<6> calculator_;
  ruckig::InputParameter<6> input_;
  PreviewBrakeTrajectory accepted_;
  PreviewBrakeStatus status_{PreviewBrakeStatus::Inactive};
};

} // namespace rb_servo::control
