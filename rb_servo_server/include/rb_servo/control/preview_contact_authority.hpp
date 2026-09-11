#pragma once

#include "rb_servo/control/admittance_overlay.hpp"

namespace rb_servo::control {

struct PreviewContactAuthority {
  double gate{1.0};
  math::Vector3 normal_into_stand{math::Vector3::Zero()};
};

// The preview QP's contact bound comes from the SAME pair the follower's advance gate
// uses: the one curve's ratio and the DECLARED press axis (2026-09-11). Before that it
// was the stream classifier's armed state and a filtered wrench direction; both are
// gone - an axis that is declared cannot rotate, so nothing has to be armed before it
// can be trusted. `follower_contact_direction` is the follower's own
// into-contact direction (DualArmServoLoop::declaredContactNormal), so the advance
// gate, the executor's slew and the QP authority still see exactly one direction.
inline PreviewContactAuthority followerPreviewContactAuthority(
    bool reference_eligible,double tick_gate,const math::Vector3& follower_contact_direction) {
  if(!reference_eligible||follower_contact_direction.isZero(0.0))return {};
  return {tick_gate,math::Vector3(-follower_contact_direction)};
}

} // namespace rb_servo::control
