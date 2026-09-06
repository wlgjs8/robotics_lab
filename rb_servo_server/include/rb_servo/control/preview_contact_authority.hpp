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

// The recorded replay supplies the same already-observed classifier state and
// tick authority to the overload above; it does not recreate classifier history.
inline PreviewContactAuthority sustainedPreviewContactAuthority(
    bool reference_eligible,const ForceGate& force_gate) {
  return sustainedPreviewContactAuthority(reference_eligible,force_gate.streamArmed(),
      force_gate.translation(),force_gate.forceDirection());
}

} // namespace rb_servo::control
