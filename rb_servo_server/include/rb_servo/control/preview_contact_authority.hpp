#pragma once

#include <Eigen/Core>

#include "rb_servo/math/se3.hpp"

namespace rb_servo::control {

// The preview QP's contact bound: the ONE curve's ratio and the ONE contact normal
// the servo loop publishes per tick (ForceGate::contactNormal, the unit measured
// physical force = the free-space direction). Before 2026-09-11 it was the stream
// classifier's armed state and a filtered wrench direction; from 2026-09-11 to
// 2026-09-15 it was a DECLARED tool-frame press axis; now that the law is isotropic
// along the measured force there is nothing to declare and nothing to arm. The
// chunk follower's advance cut (CartesianChunkFollower::setAdvanceGate), the
// pose-track stage's state hold and this bound all read the same (gate, normal)
// pair from the follower's slot, so the three cannot drift apart.
struct PreviewContactAuthority {
  double gate{1.0};
  math::Vector3 normal_into_stand{math::Vector3::Zero()};
};

// `follower_free_direction` is the follower's published direction: +F_hat, i.e. the
// force ON the tool, pointing OUT of the contact. The QP wants the direction INTO
// contact (the closing direction it bounds), hence the negation. A zero direction
// (no contact above the noise band, or the reference not eligible) returns the
// default authority: gate 1, no normal - the QP's contactAllows() is then a no-op.
inline PreviewContactAuthority followerPreviewContactAuthority(
    bool reference_eligible, double tick_gate, const math::Vector3& follower_free_direction) {
  if (!reference_eligible || follower_free_direction.isZero(0.0)) return {};
  return {tick_gate, math::Vector3(-follower_free_direction)};
}

}  // namespace rb_servo::control
