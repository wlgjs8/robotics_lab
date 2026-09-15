#pragma once

#include <Eigen/Core>

#include "rb_servo/math/se3.hpp"

namespace rb_servo::control {

// Effective scalar confidence/force authority and a measured outward direction.
// Preview consumes this pair independently of the raw follower geometry slot.
// Presence of a unit normal alone must never impose a binary speed constraint.
struct PreviewContactAuthority {
  double gate{1.0};
  math::Vector3 normal_into_stand{math::Vector3::Zero()};
};

// `follower_free_direction` is the force gate's outward direction: +F_hat, i.e. the
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
