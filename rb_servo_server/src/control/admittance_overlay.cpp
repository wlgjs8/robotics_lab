#include "rb_servo/control/admittance_overlay.hpp"

#include <algorithm>
#include <cmath>

namespace rb_servo {
namespace control {
namespace {

double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Isotropic rate cap: the law is a vector law, so its caps are on the NORM. A
// per-component clamp would bend a diagonal yield toward the axes.
void clampNorm(math::Vector3& v, double cap) {
    if (!(cap > 0.0)) return;
    const double n = v.norm();
    if (n > cap && n > 1e-18) v *= cap / n;
}

}  // namespace

// ---------------------------------------------------------------------------
// AdmittanceOverlay
// ---------------------------------------------------------------------------

void AdmittanceOverlay::configure(const ForceControlConfig& cfg, double control_period_sec) {
    cfg_ = cfg;
    dt_ = control_period_sec > 0.0 ? control_period_sec : 0.002;
    reset();
}

void AdmittanceOverlay::reset() {
    dp_.setZero();
    vp_.setZero();
    er_.setZero();
    w_.setZero();
    bounded_ = false;
    osc_frozen_ = false;
    osc_quiet_ticks_ = 0;
    osc_prev_v_ = {math::Vector3::Zero(), math::Vector3::Zero()};
    osc_reversal_ticks_ = {};
    osc_reversal_head_ = {0, 0};
}

void AdmittanceOverlay::step(const math::Vector3& force_phys_stand,
                             const math::Vector3& torque_phys_stand) {
    ++osc_tick_;
    if (osc_frozen_) {
        // Latched by the oscillation guard: hold the deviation, drop momentum,
        // and only rejoin after the wrench has been QUIET for the release
        // window - releasing into a still-pushing hand would re-enter the same
        // loop that tripped the guard.
        vp_.setZero();
        w_.setZero();
        const bool quiet =
            force_phys_stand.norm() < cfg_.oscillation_release_force_n &&
            torque_phys_stand.norm() < cfg_.oscillation_release_torque_nm;
        osc_quiet_ticks_ = quiet ? osc_quiet_ticks_ + 1 : 0;
        const uint64_t need =
            static_cast<uint64_t>(cfg_.oscillation_release_quiet_sec / dt_);
        if (osc_quiet_ticks_ >= need) {
            osc_frozen_ = false;
            osc_quiet_ticks_ = 0;
            osc_prev_v_ = {math::Vector3::Zero(), math::Vector3::Zero()};
            osc_reversal_ticks_ = {};
        }
        return;
    }
    const double m = cfg_.law.m;
    const double b = cfg_.law.b;
    if (!(m > 0.0) || !(b >= 0.0)) {
        // No law (m <= 0 is the rigid spelling): the loader refuses this with force
        // control enabled, so this is the offline default, not a live path.
        vp_.setZero();
        w_.setZero();
        return;
    }
    // Isotropic excess force drives translation. Zero drive allows m/b coasting;
    // it never seeks a surface. The nominal gate shares this target, so head-on
    // pressing has zero closing authority and zero yield drive at equilibrium.
    const double fn = force_phys_stand.norm();
    const double excess = fn - cfg_.target_force_n;
    math::Vector3 drive = math::Vector3::Zero();
    if (excess > 0.0 && fn > 1e-12 && std::isfinite(excess)) {
        drive = (excess / fn) * force_phys_stand;
    }
    math::Vector3 a = (drive - b * vp_) / m;
    clampNorm(a, cfg_.max_acceleration_m_s2);
    vp_ += a * dt_;
    clampNorm(vp_, cfg_.max_velocity_m_s);
    dp_ += vp_ * dt_;
    // Rotation is RIGID: the tool holds its orientation whatever the torque.
    er_.setZero();
    w_.setZero();
    applyFence();
    if (cfg_.oscillation_guard_enable) {
        stepOscillationGuard();
    }
}

void AdmittanceOverlay::stepOscillationGuard() {
    // A reversal = the part's velocity, while ABOVE the amplitude floor, pointing
    // against the LAST above-floor direction. The direction is latched only at
    // amplitude, because a continuous oscillation passes THROUGH zero at every
    // flip - a naive tick-to-tick sign test never sees amplitude on both sides
    // of the crossing. The dot product makes the test direction-agnostic (the
    // 2026-08-27 incident oscillated on the wrist axes, not a config axis); the
    // floor keeps noise flips out; a push-then-pull by an operator is ONE
    // reversal and stays far below min_reversals.
    const uint64_t window_ticks =
        static_cast<uint64_t>(cfg_.oscillation_window_sec / dt_);
    for (int part = 0; part < 2; ++part) {
        const math::Vector3& v = part == 0 ? vp_ : w_;
        const double cap =
            part == 0 ? cfg_.max_velocity_m_s : cfg_.max_velocity_rad_s;
        const double floor_v = cfg_.oscillation_min_velocity_frac * cap;
        if (v.norm() <= floor_v) continue;   // only speak at amplitude
        math::Vector3& last_dir = osc_prev_v_[part];  // last above-floor direction
        const bool have_dir = last_dir.squaredNorm() > 0.5;
        const bool reversal = have_dir && v.dot(last_dir) < 0.0;
        last_dir = v.normalized();
        if (!reversal) continue;
        auto& ring = osc_reversal_ticks_[part];
        int& head = osc_reversal_head_[part];
        ring[head % kOscRingSize] = osc_tick_;
        head = (head + 1) % kOscRingSize;
        int recent = 0;
        for (uint64_t stamp : ring) {
            if (stamp != 0 && osc_tick_ - stamp <= window_ticks) ++recent;
        }
        if (recent >= cfg_.oscillation_min_reversals) {
            osc_frozen_ = true;
            ++osc_trips_;
            osc_quiet_ticks_ = 0;
            freeze();
            return;
        }
    }
}

void AdmittanceOverlay::applyFence() {
    // NON-POSITIVE = NO FENCE on that part; translation and rotation decide
    // independently. Hitting a bound ZEROES the velocity along the clamped direction
    // so nothing winds up against the fence, and latches `bounded_` for the caller to
    // publish: a silent saturation is a lie about where the arm is being asked to go.
    // On a fold path the deviation is booked into the plan every tick and this is
    // unreachable; it is the dead backstop of the absolute-target source only.
    bounded_ = false;
    if (cfg_.max_deviation_m > 0.0) {
        const double n = dp_.norm();
        if (n > cfg_.max_deviation_m && n > 1e-12) {
            dp_ *= cfg_.max_deviation_m / n;
            const math::Vector3 dir = dp_.normalized();
            const double along = vp_.dot(dir);
            if (along > 0.0) vp_ -= along * dir;
            bounded_ = true;
        }
    }
    if (cfg_.max_deviation_rad > 0.0) {
        const double n = er_.norm();
        if (n > cfg_.max_deviation_rad && n > 1e-12) {
            er_ *= cfg_.max_deviation_rad / n;
            const math::Vector3 dir = er_.normalized();
            const double along = w_.dot(dir);
            if (along > 0.0) w_ -= along * dir;
            bounded_ = true;
        }
    }
}

void AdmittanceOverlay::freeze() {
    vp_.setZero();
    w_.setZero();
}

Pose6D AdmittanceOverlay::compose(const Pose6D& nominal_stand) const {
    // The pivot IS the TCP (the same point the wrench is referenced at), so the
    // rotation carry `(dR - I) * (p_tcp - p_pivot)` vanishes identically and the
    // tool turns about its own control point. Translation therefore adds directly.
    Pose6D out = nominal_stand;
    out.x += dp_.x();
    out.y += dp_.y();
    out.z += dp_.z();
    const double ang = er_.norm();
    if (ang >= 1e-9) {
        const math::Matrix3 d_r = math::exp3(er_);
        const math::Matrix3 r_out = d_r * math::rotationFromPose(nominal_stand);
        const math::Vector3 rpy = r_out.eulerAngles(2, 1, 0);
        out.rz = rpy[0];
        out.ry = rpy[1];
        out.rx = rpy[2];
        if (nominal_stand.quaternion_xyzw.has_value()) {
            const Eigen::Quaterniond q(r_out);
            out.quaternion_xyzw = std::array<double, 4>{q.x(), q.y(), q.z(), q.w()};
        }
    }
    return out;
}

Pose6D AdmittanceOverlay::strip(const Pose6D& emitted_stand) const {
    Pose6D out = emitted_stand;
    out.x -= dp_.x();
    out.y -= dp_.y();
    out.z -= dp_.z();
    const double ang = er_.norm();
    if (ang >= 1e-9) {
        const math::Matrix3 d_r = math::exp3(er_);
        const math::Matrix3 r_out = d_r.transpose() * math::rotationFromPose(emitted_stand);
        const math::Vector3 rpy = r_out.eulerAngles(2, 1, 0);
        out.rz = rpy[0];
        out.ry = rpy[1];
        out.rx = rpy[2];
        if (emitted_stand.quaternion_xyzw.has_value()) {
            const Eigen::Quaterniond q(r_out);
            out.quaternion_xyzw = std::array<double, 4>{q.x(), q.y(), q.z(), q.w()};
        }
    }
    return out;
}

bool AdmittanceOverlay::quiescent(double eps_m) const {
    return dp_.norm() < eps_m && er_.norm() < 1e-9 && vp_.norm() < 1e-9 && w_.norm() < 1e-9;
}

void AdmittanceOverlay::dropDeviation() {
    dp_.setZero();
    er_.setZero();
    bounded_ = false;   // there is nothing left to be pinned against the fence
}

// ---------------------------------------------------------------------------
// ForceGate
// ---------------------------------------------------------------------------

void ForceGate::configure(const ForceControlConfig& cfg, double control_period_sec) {
    cfg_ = cfg;
    dt_ = control_period_sec > 0.0 ? control_period_sec : 0.002;
    b_eff_ = cfg_.law.b;
    m_eff_ = cfg_.law.m;
    reset();
}

void ForceGate::reset() {
    gate_t_ = physical_gate_ = 1.0;
    confidence_ = 0.0;
    force_dir_.setZero();
    contact_normal_.setZero();
    force_n_ = 0.0;
    torque_nm_ = 0.0;
    demand_ = 0.0;
}

void ForceGate::update(const math::Vector3& force_phys_stand, const math::Vector3& torque_phys_stand,
                       double source_demand_m_s) {
    const double fv = force_phys_stand.norm();
    demand_ = std::isfinite(source_demand_m_s) ? std::max(0.0,source_demand_m_s) : 0.0;
    torque_nm_ = torque_phys_stand.norm();
    if(!std::isfinite(fv) || !std::isfinite(torque_nm_)) {
        // Normal is retained only as a defensive closed direction; the upstream
        // coverage gate rejects invalid sensor samples before motion composition.
        force_n_=fv;confidence_=1.0;gate_t_=physical_gate_=0.0;return;
    }
    force_n_=fv;
    force_dir_=fv>0 ? math::Vector3(force_phys_stand/fv) : math::Vector3::Zero();
    const auto smooth=[](double s) { s=clamp(s,0.0,1.0);return s*s*(3.0-2.0*s); };
    const double width=cfg_.contact_noise_full_n-cfg_.contact_noise_low_n;
    if(!(width>0) || !(cfg_.target_force_n>cfg_.contact_noise_full_n)) {
        gate_t_=physical_gate_=0.0;confidence_=1.0;contact_normal_=force_dir_;return;
    }
    confidence_=smooth((fv-cfg_.contact_noise_low_n)/width);
    contact_normal_=confidence_>0 ? force_dir_ : math::Vector3::Zero();
    if(!cfg_.gate_enable) {gate_t_=physical_gate_=1.0;return;}
    // At target, both inward nominal speed and excess-force drive are zero.
    // No source-speed feedback: slowing the output cannot reopen its own gate.
    const double desired=1.0-smooth(fv/cfg_.target_force_n);
    const double tau=desired<physical_gate_ ? cfg_.gate_close_tau_s : cfg_.gate_open_tau_s;
    physical_gate_+=std::min(dt_/tau,1.0)*(desired-physical_gate_);
    physical_gate_=clamp(physical_gate_,0.0,1.0);
    gate_t_=snapOpen(1.0-confidence_*(1.0-physical_gate_));
}

// A FIRST-ORDER SLEW ONLY EVER APPROACHES 1.0. Left alone, a gate that closed once
// stays at 0.9999... forever, every `>= 1.0` guard downstream stays false, and the
// "cut" it books is nanometres - which would be harmless if the tracker's hold
// were proportional, but its velocity drop was all-or-nothing. Measured on the UMI
// run of 2026-09-04 22:32: one 0.7 s press on the right arm left it at 1 - 1e-7 for
// the next 22 s, and 17,351 moving ticks then had their velocity INTO the slow-force
// direction dropped over a 6 nm cut; the left arm, never armed, was clean. Snap to
// exactly 1.0 within 1e-6.
double ForceGate::snapOpen(double g) {
    return g > 1.0 - 1e-6 ? 1.0 : g;
}

}  // namespace control
}  // namespace rb_servo
