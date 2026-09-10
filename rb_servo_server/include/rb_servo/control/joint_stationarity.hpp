#pragma once
// Sent-joint stationarity for recovery seeding (2026-09-10).
//
// The preview recovery seed used to require the last two SENT joint targets to be
// bit-identical. A Cartesian hold ramp converges asymptotically, so after a policy
// session is interrupted the sent target keeps creeping below any logged precision
// (the CSV shows 1e-7..1e-8 deg alternations for seconds); an exact comparison then
// never certifies the stop and the next session faults at its first command
// ("preview recovery cannot certify a stop", servo_log_20260910_111012 @440.34 s and
// servo_log_20260910_113700 @50.95 s). A physical robot at rest is one whose sent
// target moved less than a numerical tail: 1e-5 deg per 2 ms tick is 0.005 deg/s,
// three orders below the slowest commanded motion and three above the tail.
#include "rb_servo/core/types.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace rb_servo::control {

constexpr double kSentJointStationaryToleranceDeg = 1e-5;

inline double maxAbsJointDeltaDeg(const JointArray& a, const JointArray& b) {
  double worst = 0.0;
  for (int i = 0; i < kDof; ++i) {
    const double d = std::abs(a[static_cast<std::size_t>(i)] - b[static_cast<std::size_t>(i)]);
    if (!std::isfinite(d)) return std::numeric_limits<double>::infinity();
    worst = std::max(worst, d);
  }
  return worst;
}

inline bool sentJointsStationary(const JointArray& a, const JointArray& b,
                                 double tolerance_deg = kSentJointStationaryToleranceDeg) {
  const double d = maxAbsJointDeltaDeg(a, b);
  return std::isfinite(d) && d <= tolerance_deg;
}

}  // namespace rb_servo::control
