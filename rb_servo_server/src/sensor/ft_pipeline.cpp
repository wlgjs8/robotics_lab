#include "rb_servo/sensor/ft_pipeline.hpp"

#include <algorithm>
#include <cmath>

namespace rb_servo {
namespace sensor {
namespace {

// Standard gravity. MUST match controller-manager's ftident::GRAVITY_MS2 and
// Arm.cpp's GRAVITY_MS2 — the same physical constant in the same pipeline, and the
// tool mass this cell runs was FITTED against that value.
constexpr double kGravityMs2 = 9.80665;

math::Vector3 vec(const std::array<double, 3>& v) { return math::Vector3(v[0], v[1], v[2]); }

math::Vector3 force(const Wrench6D& w) { return math::Vector3(w.fx, w.fy, w.fz); }
math::Vector3 torque(const Wrench6D& w) { return math::Vector3(w.tx, w.ty, w.tz); }

Wrench6D wrench(const math::Vector3& f, const math::Vector3& m) {
    return Wrench6D{f.x(), f.y(), f.z(), m.x(), m.y(), m.z()};
}

// SOFT per-axis dead-band: |v| <= d -> 0, else v shifted toward zero by d. Stays
// CONTINUOUS across the threshold, which a hard band does not. d <= 0 = passthrough.
double deadzone(double v, double d) {
    if (d <= 0.0) return v;
    if (v > d) return v - d;
    if (v < -d) return v + d;
    return 0.0;
}

math::Vector3 deadzone3(const math::Vector3& v, const std::array<double, 3>& d) {
    return math::Vector3(deadzone(v.x(), d[0]), deadzone(v.y(), d[1]), deadzone(v.z(), d[2]));
}

bool finite(const Wrench6D& w) {
    return std::isfinite(w.fx) && std::isfinite(w.fy) && std::isfinite(w.fz) &&
           std::isfinite(w.tx) && std::isfinite(w.ty) && std::isfinite(w.tz);
}

}  // namespace

void FtPipeline::configure(const FtArmConfig& arm_config, double control_period_sec) {
    cfg_ = &arm_config;
    dt_ = control_period_sec > 0.0 ? control_period_sec : 0.002;

    // THE AXIS MAP IS BUILT FROM BASIS COLUMNS, NOT FROM A ROTATION. Each configured
    // axis is one sensor axis expressed in flange coordinates, so they are the
    // COLUMNS of the map that carries a sensor reading into flange-aligned axes.
    // On this cell the triad's determinant is -1 — the unit reports in a left-handed
    // set — which is exactly why no rpy could have stood in for these three vectors.
    axis_map_.col(0) = vec(arm_config.axis_fx);
    axis_map_.col(1) = vec(arm_config.axis_fy);
    axis_map_.col(2) = vec(arm_config.axis_fz);
    axes_det_ = axis_map_.determinant();

    Pose6D tool_pose{};
    tool_pose.rx = arm_config.tool_rpy_deg[0] * M_PI / 180.0;
    tool_pose.ry = arm_config.tool_rpy_deg[1] * M_PI / 180.0;
    tool_pose.rz = arm_config.tool_rpy_deg[2] * M_PI / 180.0;
    r_flange_tool_ = math::rotationFromPose(tool_pose);

    sensor_offset_m_ = vec(arm_config.sensor_offset_mm) * 1e-3;
    tool_com_m_ = vec(arm_config.tool_com_mm) * 1e-3;
    tcp_from_sro_m_ = vec(arm_config.tool_xyz_mm) * 1e-3;

    if (arm_config.bias_from_config) {
        bias_ = wrench(vec(arm_config.bias_force_n), vec(arm_config.bias_torque_nm));
        bias_valid_ = true;
        bias_source_ = "config";
    }
}

bool FtPipeline::step(const FtPipelineInput& in) {
    // PIN EVERY COMPENSATED CHANNEL TO EXACT ZERO when there is nothing trustworthy
    // to compensate. Zero is the one value force logic reads as "nothing is being
    // felt"; a bias- or gravity-derived number here would be a force nobody measured.
    const auto pin_zero = [this]() {
        comp_sensor_nodz_ = Wrench6D{};
        comp_sensor_ = Wrench6D{};
        comp_tcp_ = Wrench6D{};
        comp_stand_ = Wrench6D{};
        comp_stand_nodz_ = Wrench6D{};
        gravity_sensor_ = Wrench6D{};
        load_force_n_ = 0.0;
        load_mass_kg_ = 0.0;
        load_settled_ = false;
        load_seeded_ = false;
        load_ticks_ = 0;
    };

    if (cfg_ == nullptr || !cfg_->enable || !connected_ || !in.raw_valid ||
        !in.kinematics_valid || !finite(in.raw_sensor_axes)) {
        raw_sensor_ = Wrench6D{};
        pin_zero();
        return false;
    }

    // ---- (1) AXIS MAP ONLY --------------------------------------------------
    // No origin shift: the torque stays referenced at the SENSING REFERENCE ORIGIN.
    // `sensor_offset` relates the SRO to the flange and is NOT applied here.
    const math::Vector3 fs = axis_map_ * force(in.raw_sensor_axes);
    const math::Vector3 ms = axis_map_ * torque(in.raw_sensor_axes);
    raw_sensor_ = wrench(fs, ms);

    // ---- (2) TOOL GRAVITY ---------------------------------------------------
    // The box is told a ZERO payload (see FtConfig::push_zero_payload_to_box), so it
    // subtracts NOTHING and the FULL tool weight belongs here. Stand Z is up, so
    // gravity points down in stand coordinates.
    const math::Vector3 g_stand(0.0, 0.0, -kGravityMs2);
    const math::Vector3 g_sensor = in.r_stand_flange.transpose() * g_stand;
    const math::Vector3 fg = cfg_->tool_mass_kg * g_sensor;
    const math::Vector3 mg = tool_com_m_.cross(fg);
    gravity_sensor_ = wrench(fg, mg);

    // ---- (3) BIAS -----------------------------------------------------------
    // Subtracted in the SENSOR frame, before anything rotates: the bias is a sensor
    // offset, not a pose-dependent quantity.
    const math::Vector3 bias_f = force(bias_);
    const math::Vector3 bias_m = torque(bias_);
    const math::Vector3 fc = fs - bias_f - fg;
    const math::Vector3 mc = ms - bias_m - mg;
    comp_sensor_nodz_ = wrench(fc, mc);

    // ---- (4) REFERENCE-POINT SHIFT -> ROTATE -> DEADZONE --------------------
    // The shift moves the torque reference from the SRO to the TCP; the rotation
    // then expresses both in TOOL axes. The deadzone is per-axis, so it lands last,
    // on the axes the consumer reasons about.
    const math::Vector3 f_tcp = r_flange_tool_.transpose() * fc;
    const math::Vector3 m_tcp = r_flange_tool_.transpose() * (mc - tcp_from_sro_m_.cross(fc));

    // The STAND-axes copy, taken from the PRE-deadzone values so the two surfaces
    // are the same wrench in two frames rather than two different clippings.
    const math::Matrix3 r_stand_tool = in.r_stand_flange * r_flange_tool_;
    const math::Vector3 f_stand = r_stand_tool * f_tcp;
    const math::Vector3 m_stand = r_stand_tool * m_tcp;

    comp_sensor_ = wrench(deadzone3(fc, cfg_->deadzone_force_n),
                          deadzone3(mc, cfg_->deadzone_torque_nm));
    comp_tcp_ = wrench(deadzone3(f_tcp, cfg_->deadzone_force_n),
                       deadzone3(m_tcp, cfg_->deadzone_torque_nm));
    // The stand surface is deadzoned in the TOOL axes it was clipped in, then
    // rotated: deadzoning in stand axes would clip a different set of directions
    // than the law's, and the two would disagree about when contact began.
    comp_stand_ = wrench(r_stand_tool * force(comp_tcp_), r_stand_tool * torque(comp_tcp_));
    comp_stand_nodz_ = wrench(f_stand, m_stand);

    // ---- the tool-load estimate --------------------------------------------
    // Taken from the PRE-deadzone stand force, because this channel exists precisely
    // to escape the deadzone. SEEDED to the live sample rather than ramped from
    // zero, or the first 5*tau would report a load that is not there.
    {
        const math::Vector3 f_base = r_stand_tool * f_tcp;
        const double tau = cfg_->tool_load_tau_s > 0.0 ? cfg_->tool_load_tau_s : 1.0;
        const double a = dt_ / (tau + dt_);
        if (!load_seeded_) {
            load_f_ = f_base;
            load_seeded_ = true;
            load_ticks_ = 0;
        } else {
            load_f_ += a * (f_base - load_f_);
        }
        const auto need = static_cast<std::uint32_t>(5.0 * tau / dt_) + 1u;
        if (load_ticks_ < need) ++load_ticks_;
        load_settled_ = load_ticks_ >= need;
        // The norm AFTER the per-axis filter. The opposite order would leave the
        // rectification bias as an unremovable DC term.
        load_force_n_ = load_f_.norm();
        load_mass_kg_ = load_force_n_ / kGravityMs2;
    }
    return true;
}

void FtPipeline::tareSample() {
    // AVERAGE `raw - gravity`, NEVER `raw`. The box subtracts no payload, so raw
    // still contains the tool's weight: averaging raw would fold the tare pose's
    // gravity into the bias, and then step() would subtract gravity a second time.
    tare_force_sum_ += force(raw_sensor_) - force(gravity_sensor_);
    tare_torque_sum_ += torque(raw_sensor_) - torque(gravity_sensor_);
    ++tare_count_;
}

void FtPipeline::invalidateBias() {
    bias_ = Wrench6D{};
    bias_valid_ = false;
    bias_source_ = "none";
    tareReset();
}

void FtPipeline::tareReset() {
    tare_force_sum_.setZero();
    tare_torque_sum_.setZero();
    tare_count_ = 0;
}

bool FtPipeline::tareCommit(int min_samples, std::string* reason) {
    if (tare_count_ < min_samples) {
        if (reason != nullptr) {
            *reason = "tare needs " + std::to_string(min_samples) + " samples, got " +
                      std::to_string(tare_count_);
        }
        tareReset();
        return false;
    }
    const double n = static_cast<double>(tare_count_);
    bias_ = wrench(tare_force_sum_ / n, tare_torque_sum_ / n);
    bias_valid_ = true;
    bias_source_ = "tare";
    ++bias_generation_;
    tareReset();
    // A NEW ZERO MOVED WHAT ZERO MEANS, so the tool-load estimate must forget what it
    // had converged to and re-seed against the new bias instead of asserting a mass
    // measured against a zero that no longer exists.
    load_seeded_ = false;
    load_settled_ = false;
    load_ticks_ = 0;
    if (reason != nullptr) *reason = "accepted";
    return true;
}

void FtPipeline::livenessSample(const Wrench6D& raw_sensor_axes) {
    if (!finite(raw_sensor_axes)) return;
    const std::array<double, 3> f{raw_sensor_axes.fx, raw_sensor_axes.fy, raw_sensor_axes.fz};
    const std::array<double, 3> m{raw_sensor_axes.tx, raw_sensor_axes.ty, raw_sensor_axes.tz};
    ++live_samples_;
    if (!live_have_) {
        fmin_ = fmax_ = f;
        mmin_ = mmax_ = m;
        live_have_ = true;
        return;
    }
    for (int i = 0; i < 3; ++i) {
        fmin_[i] = std::min(fmin_[i], f[i]);
        fmax_[i] = std::max(fmax_[i], f[i]);
        mmin_[i] = std::min(mmin_[i], m[i]);
        mmax_[i] = std::max(mmax_[i], m[i]);
    }
}

bool FtPipeline::livenessDecide() {
    if (cfg_ == nullptr || !cfg_->enable) {
        connected_ = false;
        connect_reason_ = "force_torque disabled for this arm";
        return connected_;
    }
    if (!live_have_ || live_samples_ < 2) {
        connected_ = false;
        connect_reason_ = "no F/T samples arrived during the liveness window";
        return connected_;
    }
    force_pp_ = 0.0;
    torque_pp_ = 0.0;
    for (int i = 0; i < 3; ++i) {
        force_pp_ = std::max(force_pp_, fmax_[i] - fmin_[i]);
        torque_pp_ = std::max(torque_pp_, mmax_[i] - mmin_[i]);
    }
    // A LIVE RFT ALWAYS JITTERS above its noise floor. A stream that arrives but
    // never varies is a frozen or unplugged sensor — which is NOT fatal here, but
    // must not be compensated as if it were a reading.
    const bool live = force_pp_ >= cfg_->liveness_min_force_pp_n ||
                      torque_pp_ >= cfg_->liveness_min_torque_pp_nm;
    connected_ = live;
    connect_reason_ = live ? "sensor stream varies above the noise floor"
                           : "sensor stream is FLAT across the window (unplugged or frozen) - "
                             "every compensated channel is pinned to zero";
    return connected_;
}

void FtPipeline::setExternallyVerifiedConnection(bool connected) {
    connected_ = connected;
    connect_reason_ = connected ? "externally stepped sensor: sample sequence/time verified"
                                : "externally stepped sensor: invalid or stale sample";
}

void FtPipeline::fillTelemetry(FtTelemetry* out) const {
    if (out == nullptr) return;
    out->enabled = cfg_ != nullptr && cfg_->enable;
    out->connected = connected_;
    out->connect_reason = connect_reason_;
    out->liveness_force_pp_n = force_pp_;
    out->liveness_torque_pp_nm = torque_pp_;
    out->raw_sensor = raw_sensor_;
    out->gravity_sensor = gravity_sensor_;
    out->comp_sensor_nodz = comp_sensor_nodz_;
    out->comp_sensor = comp_sensor_;
    out->comp_tcp = comp_tcp_;
    out->comp_stand = comp_stand_;
    out->bias = bias_;
    out->bias_valid = bias_valid_;
    out->bias_source = bias_source_;
    out->bias_generation = bias_generation_;
    out->tare_samples = tare_count_;
    out->load_force_n = load_force_n_;
    out->load_mass_kg = load_mass_kg_;
    out->load_settled = load_settled_;
    out->axes_determinant = axes_det_;
    if (cfg_ != nullptr) {
        out->tool_mass_kg = cfg_->tool_mass_kg;
        out->tool_com_mm = cfg_->tool_com_mm;
        out->sensor_offset_mm = cfg_->sensor_offset_mm;
        out->tcp_from_sro_mm = cfg_->tool_xyz_mm;
    }
}

}  // namespace sensor
}  // namespace rb_servo
