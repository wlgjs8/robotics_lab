#include "rb_servo/control/admittance_overlay.hpp"

#include <algorithm>
#include <cmath>

namespace rb_servo {
namespace control {
namespace {

double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

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
                             const math::Vector3& force_stand_physical,
                             const math::Vector3& torque_stand_physical) {
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
    const math::Vector3 fp = rw.transpose() * force_stand_physical;
    const math::Vector3 mp = rw.transpose() * torque_stand_physical;

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

        // FORCE MODE IS ONE-SIDED AND JUDGED ON THE PHYSICAL COMPONENT (2026-09-11).
        // Only the EXCESS over ref_force drives the deviation, in the direction the
        // force points. Three properties follow, and all three are requirements:
        //   * |f| <= ref_force is an equilibrium, so the axis does not move at all
        //     there. FREE SPACE (f = 0) can therefore never be sought - the walk a
        //     two-sided setpoint has to stop with a fence does not exist here.
        //   * A contact RESTS at ref_force instead of retreating to zero, which is
        //     what a pure damper does (its only equilibrium is f = 0; measured
        //     2026-09-11: 28 N at the floor, then 6.3 mm of retreat to 0.1-1.0 N).
        //   * The yield line is v = (|f| - ref)/b, which is what the gate's curve is
        //     pinned to cross at peak_force_n.
        // The stiffness is dropped: a spring plus a setpoint converges at the balance
        // w = ref + k*d, which is a different (and surface-position dependent) force.
        const double phys = rot ? mp[i - 3] : fp[i];
        double drive = f;
        double k = ax.k;
        if (ax.mode == ForceAxisMode::Force) {
            const double excess = std::abs(phys) - ax.ref_force;
            drive = excess > 0.0 ? std::copysign(excess, phys) : 0.0;
            k = 0.0;
        }
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
        // A SPRING ALWAYS REFUSES: at any k != 0 the -k*d term revives and the gauge
        // change becomes an origin walk.
        if (ax.k != 0.0) return false;
        // A ONE-SIDED FORCE AXIS IS FOLDABLE (2026-09-11). The old exclusion was written
        // for the TWO-SIDED setpoint (drive = f - ref), which is designed to walk until
        // it meets the fence when nothing presses back; handing that walk to the plan
        // would have deleted the designed stop. The one-sided drive - only the excess
        // over ref_force, in the direction the force points - has |f| <= ref as an
        // equilibrium, so there is no free-space walk to hand over at all. And the
        // gauge argument is untouched: the drive is a function of the MEASURED wrench
        // only, so with k = 0 the deviation is still a bare integrator nothing reads
        // back. REFUSING IT WAS A REAL BUG: with the press axis declared and the fold
        // declined, a hand push accumulated the whole yield in the overlay and pinned
        // the 40 mm fence - the arm went rigid there, 3606 ticks of bounded() in
        // servo_log_20260911_133829 (right arm, |F| 21 N).
        if (ax.mode == ForceAxisMode::Force) continue;
        if (ax.ref_force != 0.0) return false;   // a ref on a COMPLIANCE row is malformed
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
    // `b` FOR THE CROSSING IS THE LARGEST COMPLIANT TRANSLATION DAMPING, never row 0
    // (CM found this twice in review): a RIGID row carries b = 0, which would leave
    // v_cross = 0 and disable the gate on all axes, and with anisotropic damping the
    // crossing would be pinned to one axis while the contact is on another. The max
    // is conservative in both cases - a larger b means a smaller v_cross, so the
    // converged force lands AT or BELOW the declaration, never above it. The loader
    // WARNs when the three rows disagree, because only then is one v_cross the whole
    // story. The STREAM law owns this: the hold law ships the same rows by decision
    // and the loader refuses a disagreement.
    b_eff_ = 0.0;
    m_eff_ = 0.0;
    for (const ForceAxisConfig& ax : cfg_.stream.translation) {
        if (ax.mode == ForceAxisMode::Rigid || !(ax.m > 0.0) || ax.b < 0.0) continue;
        if (ax.b > b_eff_) b_eff_ = ax.b;
        if (ax.m > m_eff_) m_eff_ = ax.m;
    }
    const double span = cfg_.gate_peak_force_n - cfg_.gate_rest_force_n;
    v_cross_ = (b_eff_ > 1e-9 && span > 0.0) ? span / b_eff_ : 0.0;
    reset();
}

void ForceGate::reset() {
    gate_t_ = 1.0;
    gate_r_ = 1.0;
    force_dir_.setZero();
    torque_dir_.setZero();
    force_n_ = 0.0;
    torque_nm_ = 0.0;
    stream_speed_ = 0.0;
}

void ForceGate::update(const math::Vector3& force_stand, const math::Vector3& torque_stand,
                       double force_magnitude_n, double torque_magnitude_nm,
                       double stream_speed_m_s) {
    const double fv = force_stand.norm();
    const double mv = torque_stand.norm();
    force_n_ = force_magnitude_n >= 0.0 ? force_magnitude_n : fv;
    torque_nm_ = torque_magnitude_nm >= 0.0 ? torque_magnitude_nm : mv;
    force_dir_ = fv > 1e-9 ? math::Vector3(force_stand / fv) : math::Vector3::Zero();
    torque_dir_ = mv > 1e-9 ? math::Vector3(torque_stand / mv) : math::Vector3::Zero();
    stream_speed_ = std::isfinite(stream_speed_m_s) && stream_speed_m_s > 0.0
                        ? stream_speed_m_s : 0.0;

    if (!cfg_.gate_enable) {
        gate_t_ = 1.0;
        gate_r_ = 1.0;
        return;
    }
    // THE CURVE (CM 0049). Only attenuate when the stream is actually faster than the
    // crossing speed: below it the contact converges to rest + b*v_s < peak_force_n on
    // its own and the gate has nothing to give. NOTE the scope this leaves open, the
    // same one CM records: v_s is a TRANSLATION rate, so a rotation-dominant stream is
    // ungated and its contact torque is bounded by the rotational law alone - which on
    // this cell is RIGID, so there is no rotational yield to run away, but also no
    // rotational compliance to absorb the torque.
    double t_raw = 1.0;
    if (cfg_.gate_peak_force_n > 0.0 && v_cross_ > 0.0 && stream_speed_ > v_cross_) {
        const double u = force_n_ / cfg_.gate_peak_force_n;
        const double e = std::pow(u, ForceControlConfig::kGateCurveExponent);
        t_raw = std::pow(v_cross_ / stream_speed_, e);
        // A NON-FINITE RESULT CLOSES THE GATE, IT DOES NOT OPEN IT: the only way here
        // is a non-finite force, i.e. the F/T pipeline produced a NaN, and the one
        // thing that must not follow a broken force sensor is the stream running on at
        // full authority into whatever it was pressing.
        if (!std::isfinite(t_raw)) t_raw = 0.0;
        t_raw = clamp(t_raw, 0.0, 1.0);
    }
    // ROTATION TAKES THE RATIO THE FORCE PRODUCED (CM 0049 SS5.11). A single contact
    // point carries a torque that is a dependent component of the same force, so there
    // is no second threshold to declare and `max_torque_nm` is gone.
    const double r_raw = t_raw;

    // ASYMMETRIC first-order slew: FAST TO CLOSE, SLOW TO OPEN. A fast re-open is
    // what turns the gate into a relay against the contact and sustains a limit cycle.
    const auto slew = [&](double g, double target) {
        const double tau = (target < g) ? cfg_.gate_close_tau_s : cfg_.gate_open_tau_s;
        const double a = tau > 1e-6 ? (dt_ / tau) : 1.0;
        return snapOpen(g + (target - g) * std::min(a, 1.0));
    };
    gate_t_ = slew(gate_t_, t_raw);
    gate_r_ = slew(gate_r_, r_raw);
}

// A FIRST-ORDER SLEW ONLY EVER APPROACHES 1.0. Left alone, a gate that closed once
// stays at 0.9999... forever, every `>= 1.0` guard downstream stays false, and the
// "cut" it books is nanometres - which would be harmless if the tracker's hold
// were proportional, but its velocity drop was all-or-nothing. Measured on the UMI
// run of 2026-09-04 22:32: one 0.7 s press on the right arm's stream channel left
// it at 1 - 1e-7 for the next 22 s, and 17,351 moving ticks then had their
// velocity INTO the slow-force direction dropped over a 6 nm cut; the left arm,
// never armed, was clean. Snap to exactly 1.0 within 1e-6.
double ForceGate::snapOpen(double g) {
    return g > 1.0 - 1e-6 ? 1.0 : g;
}



}  // namespace control
}  // namespace rb_servo
