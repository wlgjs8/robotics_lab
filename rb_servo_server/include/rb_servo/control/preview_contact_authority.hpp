#pragma once

#include "rb_servo/control/admittance_overlay.hpp"

namespace rb_servo::control {

struct PreviewContactAuthority {
  double gate{1.0};
  math::Vector3 normal_into_stand{math::Vector3::Zero()};
};

// Activation for the NEW extra preview-QP constraint. Sustained-contact arming
// uses the existing stream classifier; the active bound retains the current
// tick gate and its filtered deadzoned direction. This query does not change
// canonical tick gating or the overlay, and is NOT applyStreamTranslation's
// proportional release-tail law.
inline PreviewContactAuthority sustainedPreviewContactAuthority(
    bool reference_eligible,bool stream_armed,double tick_gate,
    const math::Vector3& tick_force_direction) {
  if(!reference_eligible||!stream_armed)return {};
  return {tick_gate,-tick_force_direction};
}

// 2026-09-10 pm: the live executor takes its normal from the FOLLOWER's contact
// direction (the gate's 2 Hz slow vector, armed/released by the servo loop's
// Schmitt), so the follower's advance gate, the executor's contact slew and the QP
// authority all see one direction. The tick gate still drives the magnitude. The
// raw tick direction (forceDirection) turned > 45 deg in 18 % of ticks while the
// tool rang the F/T, and every direction change re-cut the plan.
inline PreviewContactAuthority followerPreviewContactAuthority(
    bool reference_eligible,double tick_gate,const math::Vector3& follower_force_direction) {
  if(!reference_eligible||follower_force_direction.isZero(0.0))return {};
  return {tick_gate,math::Vector3(-follower_force_direction)};
}

// THE FOLLOWER CONTACT-DIRECTION SCHMITT (2026-09-10 pm). Arming is instantaneous
// - the slow vector standing over the arm level IS a contact, and holding the
// advance back sooner is the safe side - but releasing needs the slow vector to
// stand below the release level for a dwell. Without that dwell a violently
// varying push (|F| 0.4-46 N) still re-crossed both levels inside 16 ms even
// through the 2 Hz filter, and the dispatched direction toggled at ~30 Hz, i.e.
// the arm alternated between a complete hold-back and a free advance 30 times a
// second (servo_log_20260910_183004, right arm 56.27 and 56.41 s).
struct FollowerContactDirectionArming {
  bool armed{false};
  double release_sec{0.0};
  // Unit direction of the slow force, LATCHED at the last sample that stood over
  // the release level. During the dwell the slow vector is by definition small, and
  // the direction of a small vector is noise; an armed arm removes its whole advance
  // along this direction, so it must not rotate while the contact fades out.
  math::Vector3 direction{math::Vector3::Zero()};
};

inline bool updateFollowerContactDirectionArming(FollowerContactDirectionArming& state,
    const math::Vector3& slow_force,double dt_sec,double arm_force_n,double release_force_n,
    double release_dwell_sec) {
  const double n=slow_force.norm();
  if(state.armed) {
    if(n<release_force_n) {
      state.release_sec+=dt_sec>0?dt_sec:0.0;
      if(state.release_sec>=release_dwell_sec) {
        state.armed=false;state.release_sec=0.0;state.direction.setZero();
      }
    } else {
      state.release_sec=0.0;
      if(n>1e-9)state.direction=slow_force/n;
    }
  } else if(n>=arm_force_n) {
    state.armed=true;state.release_sec=0.0;state.direction=slow_force/n;
  }
  return state.armed;
}

// The recorded replay supplies the same already-observed classifier state and
// tick authority to the overload above; it does not recreate classifier history.
inline PreviewContactAuthority sustainedPreviewContactAuthority(
    bool reference_eligible,const ForceGate& force_gate) {
  return sustainedPreviewContactAuthority(reference_eligible,force_gate.streamArmed(),
      force_gate.translation(),force_gate.forceDirection());
}

} // namespace rb_servo::control
