#include "rb_servo/control/admittance_overlay.hpp"

#include <algorithm>
#include <cmath>

namespace rb_servo {
namespace control {
namespace {

double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

// KNEE-LESS SMOOTHSTEP. g(0) = 1 and g(1) = 0 EXACTLY, both endpoints with zero
// slope. The exact endpoints matter: g(0) = 1 means free space and light contact
// cost the plan nothing, and g(1) = 0 means the advance can actually STOP, which is
// what creates the equilibrium that bounds the deviation at F/k.
double fade(double u) {
    if (u <= 0.0) return 1.0;
    if (u >= 1.0) return 0.0;
    return 1.0 - 3.0 * u * u + 2.0 * u * u * u;
}

}  // namespace

// ---------------------------------------------------------------------------
// AdmittanceOverlay
// ---------------------------------------------------------------------------

void AdmittanceOverlay::configure(const ForceControlConfig& cfg, double control_period_sec) {
    cfg_ = cfg;
    // Default to the STREAM law; whoever owns the tick selects the real one. A
    // default of "nothing" would make an unset caller silently rigid.
    law_ = cfg.stream;
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

void AdmittanceOverlay::step(const math::Vector3& force_stand,
                             const math::Vector3& torque_stand,
                             double gate) {
    ++osc_tick_;
    if (osc_frozen_) {
        // Latched by the oscillation guard: hold the deviation, drop momentum,
        // and only rejoin after the wrench has been QUIET for the release
        // window — releasing into a still-pushing hand would re-enter the same
        // loop that tripped the guard.
        vp_.setZero();
        w_.setZero();
        const bool quiet =
            force_stand.norm() < cfg_.oscillation_release_force_n &&
            torque_stand.norm() < cfg_.oscillation_release_torque_nm;
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
    // Rotate the state INTO the workspace frame, integrate per axis there, rotate
    // back. The state itself stays in the stand frame — see the header for why a
    // deviation stored in a rotating frame would sweep as the tool turns.
    const math::Matrix3& rw = r_ws_;
    math::Vector3 dp = rw.transpose() * dp_;
    math::Vector3 vp = rw.transpose() * vp_;
    math::Vector3 er = rw.transpose() * er_;
    math::Vector3 wv = rw.transpose() * w_;
    const math::Vector3 fw = rw.transpose() * force_stand;
    const math::Vector3 mw = rw.transpose() * torque_stand;
    const double g = clamp(gate, 0.0, 1.0);

    for (int i = 0; i < 6; ++i) {
        const bool rot = i >= 3;
        const ForceAxisConfig& ax = rot ? law_.rotation[i - 3] : law_.translation[i];
        double& d = rot ? er[i - 3] : dp[i];
        double& v = rot ? wv[i - 3] : vp[i];
        const double f = rot ? mw[i - 3] : fw[i];

        // RIGID: an axis whose dynamics were left empty asked for the nominal path
        // on that axis, so it does not deviate at all.
        if (ax.mode == ForceAxisMode::Rigid || !(ax.m > 0.0) || ax.b < 0.0) {
            d = 0.0;
            v = 0.0;
            continue;
        }

        // FORCE mode drops the stiffness and regulates against ref_force; the GATE
        // throttles that walk, because in free space the walk has nothing to stop it
        // but the fence.
        const double drive = (ax.mode == ForceAxisMode::Force)
                                 ? (f - ax.ref_force) * g
                                 : f;
        const double k = (ax.mode == ForceAxisMode::Force) ? 0.0 : ax.k;
        double a = (drive - ax.b * v - k * d) / ax.m;

        const double a_max = rot ? cfg_.max_acceleration_rad_s2 : cfg_.max_acceleration_m_s2;
        const double v_max = rot ? cfg_.max_velocity_rad_s : cfg_.max_velocity_m_s;
        a = clamp(a, -a_max, a_max);
        v = clamp(v + a * dt_, -v_max, v_max);
        d += v * dt_;
    }

    dp_ = rw * dp;
    vp_ = rw * vp;
    er_ = rw * er;
    w_ = rw * wv;
    applyFence();
    if (cfg_.oscillation_guard_enable) {
        stepOscillationGuard(force_stand, torque_stand);
    }
}

void AdmittanceOverlay::stepOscillationGuard(const math::Vector3& force_stand,
                                             const math::Vector3& torque_stand) {
    (void)force_stand;
    (void)torque_stand;
    // A reversal = the part's velocity, while ABOVE the amplitude floor, pointing
    // against the LAST above-floor direction. The direction is latched only at
    // amplitude, because a continuous oscillation passes THROUGH zero at every
    // flip — a naive tick-to-tick sign test never sees amplitude on both sides
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

bool AdmittanceOverlay::pureDamperTriad(const std::array<ForceAxisConfig, 3>& axes) {
    for (const ForceAxisConfig& ax : axes) {
        // RIGID holds d = 0 by construction - it neither helps nor hinders the transfer.
        if (ax.mode == ForceAxisMode::Rigid || !(ax.m > 0.0) || ax.b < 0.0) continue;
        if (ax.mode != ForceAxisMode::Compliance) return false;   // FORCE walks by design
        if (ax.k != 0.0 || ax.ref_force != 0.0) return false;
    }
    return true;
}

void AdmittanceOverlay::dropDeviation() {
    dp_.setZero();
    er_.setZero();
    bounded_ = false;   // there is nothing left to be pinned against the fence
}

// ---------------------------------------------------------------------------
// HoldEngageLatch
// ---------------------------------------------------------------------------

void HoldEngageLatch::configure(double engage_n, double release_n) {
    engage_n_ = engage_n;
    release_n_ = release_n;
    engaged_ = false;
}

bool HoldEngageLatch::update(double force_magnitude_n) {
    if (!enabled()) return true;
    if (engaged_) {
        if (force_magnitude_n <= release_n_) engaged_ = false;
    } else if (force_magnitude_n >= engage_n_) {
        engaged_ = true;
    }
    return engaged_;
}

// ---------------------------------------------------------------------------
// ForceGate
// ---------------------------------------------------------------------------

void ForceGate::configure(const ForceControlConfig& cfg, double control_period_sec) {
    cfg_ = cfg;
    dt_ = control_period_sec > 0.0 ? control_period_sec : 0.002;
    reset();
}

void ForceGate::reset() {
    gate_t_ = 1.0;
    gate_r_ = 1.0;
    force_dir_.setZero();
    torque_dir_.setZero();
    force_n_ = 0.0;
    torque_nm_ = 0.0;
}

void ForceGate::update(const math::Vector3& force_stand, const math::Vector3& torque_stand,
                       double force_magnitude_n, double torque_magnitude_nm) {
    const double fv = force_stand.norm();
    const double mv = torque_stand.norm();
    force_n_ = force_magnitude_n >= 0.0 ? force_magnitude_n : fv;
    torque_nm_ = torque_magnitude_nm >= 0.0 ? torque_magnitude_nm : mv;
    force_dir_ = fv > 1e-9 ? math::Vector3(force_stand / fv) : math::Vector3::Zero();
    torque_dir_ = mv > 1e-9 ? math::Vector3(torque_stand / mv) : math::Vector3::Zero();

    if (!cfg_.gate_enable) {
        gate_t_ = 1.0;
        gate_r_ = 1.0;
        return;
    }
    const double t_raw =
        cfg_.gate_max_force_n > 0.0 ? fade(force_n_ / cfg_.gate_max_force_n) : 1.0;
    const double r_raw =
        cfg_.gate_max_torque_nm > 0.0 ? fade(torque_nm_ / cfg_.gate_max_torque_nm) : 1.0;

    // ASYMMETRIC first-order slew: FAST TO CLOSE, SLOW TO OPEN. A fast re-open is
    // what turns the gate into a relay against the contact and sustains a limit cycle.
    const auto slew = [&](double g, double target) {
        const double tau = (target < g) ? cfg_.gate_close_tau_s : cfg_.gate_open_tau_s;
        const double a = tau > 1e-6 ? (dt_ / tau) : 1.0;
        return g + (target - g) * std::min(a, 1.0);
    };
    gate_t_ = slew(gate_t_, t_raw);
    gate_r_ = slew(gate_r_, r_raw);
}

math::Vector3 ForceGate::applyTranslation(const math::Vector3& advance_stand,
                                          double* removed) const {
    if (removed != nullptr) *removed = 0.0;
    if (gate_t_ >= 1.0 || force_n_ <= 1e-9) return advance_stand;
    // ONLY THE COMPONENT PUSHING INTO THE MEASURED WRENCH. `proj < 0` means the
    // advance drives against the force the sensor reports, i.e. deeper into the
    // contact; the tangential and retreating components pass at full authority.
    const double proj = advance_stand.dot(force_dir_);
    if (proj >= 0.0) return advance_stand;
    const math::Vector3 cut = (1.0 - gate_t_) * proj * force_dir_;
    if (removed != nullptr) *removed = cut.norm();
    return advance_stand - cut;
}

math::Vector3 ForceGate::applyRotation(const math::Vector3& advance_stand, double* removed) const {
    if (removed != nullptr) *removed = 0.0;
    if (gate_r_ >= 1.0 || torque_nm_ <= 1e-9) return advance_stand;
    const double proj = advance_stand.dot(torque_dir_);
    if (proj >= 0.0) return advance_stand;
    const math::Vector3 cut = (1.0 - gate_r_) * proj * torque_dir_;
    if (removed != nullptr) *removed = cut.norm();
    return advance_stand - cut;
}

}  // namespace control
}  // namespace rb_servo
