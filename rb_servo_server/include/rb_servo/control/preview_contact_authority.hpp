#pragma once

#include <cmath>

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
// THE DECLARED CONTACT NORMAL. The press axis is a column of the tool-frame rotation
// (the law's single mode:force row); only its SIGN comes from the measurement, because
// only the sensor knows which side the surface is on.
//
// THE SIGN IS TAKEN AT ZERO, NOT AT A FORCE LEVEL (2026-09-11, second revision). The
// first version returned ZERO whenever |component| <= rest_force_n, reasoning that
// below the rest force there is no contact to attenuate an advance into. That put a
// HARD SWITCH EXACTLY AT THE OPERATING POINT: with the gate's ratio at 0.01 and the
// normal present, 99 % of the plan's advance is removed; with the normal absent,
// NOTHING is removed, whatever the ratio says. A contact sitting at the rest force
// therefore alternated between 1 % and 100 % authority at the wrench's own ripple rate
// - measured 52 Hz on BOTH arms over 5.3 s (servo_log_20260911_134703, 520-525 s:
// 276/273 transitions, the normal present 37 % of ticks), |F| swinging 4-50 N, q_sent
// acceleration to 3,216 deg/s^2, ending in accepted_deviation.
//
// Zeroing it was never needed: the GATE is already exactly 1.0 in free space (g(0) = 1),
// and a normal with g = 1 removes nothing. So the direction can be handed over
// unconditionally and the continuous ratio does all of the modulation - which is the
// property the whole curve rests on. What remains is a band around ZERO, where the sign
// is genuinely undefined; there the force is a few tenths of a newton, the gate is ~1,
// and the choice cannot matter.
inline math::Vector3 declaredContactNormal(const math::Matrix3& tool_in_stand,
                                           int press_axis_index,
                                           const math::Vector3& physical_force_stand,
                                           double sign_deadband_n) {
  if (press_axis_index < 0 || press_axis_index > 2) return math::Vector3::Zero();
  const math::Vector3 axis = tool_in_stand.col(press_axis_index);
  const double component = axis.dot(physical_force_stand);
  if (!std::isfinite(component) || std::abs(component) <= sign_deadband_n)
    return math::Vector3::Zero();
  return component > 0.0 ? axis : math::Vector3(-axis);
}

inline PreviewContactAuthority followerPreviewContactAuthority(
    bool reference_eligible,double tick_gate,const math::Vector3& follower_contact_direction) {
  if(!reference_eligible||follower_contact_direction.isZero(0.0))return {};
  return {tick_gate,math::Vector3(-follower_contact_direction)};
}

} // namespace rb_servo::control
