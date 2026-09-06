#include "rb_servo/control/follower_output_smd.hpp"

#include "rb_servo/math/se3.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>
#include <nlohmann/json.hpp>

#include <Eigen/Geometry>

namespace {

constexpr double kDt = 0.002;
constexpr double kTwoPi = 6.283185307179586476925286766559;

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ':' \
                      << __LINE__ << '\n'; \
            return false; \
        } \
    } while (0)

rb_servo::FollowerOutputSmdConfig config(bool velocity_ff = true, bool profile_ff = false) {
    rb_servo::FollowerOutputSmdConfig cfg;
    cfg.enable = true;
    cfg.nf_linear_hz = 3.5;
    cfg.nf_angular_hz = 2.5;
    cfg.damping_ratio = 1.0;
    cfg.velocity_ff = velocity_ff;
    cfg.velocity_ff_lpf_hz = 0.0;
    cfg.profile_feedforward = profile_ff;
    return cfg;
}

rb_servo::Pose6D poseAt(double x) {
    rb_servo::Pose6D pose;
    pose.x = x;
    return pose;
}

bool testStepNoOvershoot() {
    rb_servo::control::FollowerOutputSmd smd(config());
    smd.reset(poseAt(0.0), rb_servo::Vec6{});
    const rb_servo::Pose6D target = poseAt(0.020);
    double previous = 0.0;
    rb_servo::Pose6D out;
    for (int i = 0; i < 1500; ++i) {
        out = smd.step(target, rb_servo::Vec6{}, kDt);
        RB_CHECK(out.x >= previous - 1e-12);
        RB_CHECK(out.x <= target.x + 1e-10);
        previous = out.x;
    }
    RB_CHECK(std::abs(out.x - target.x) < 1e-6);
    return true;
}

double rampLag(bool velocity_ff) {
    constexpr double speed = 0.3;
    rb_servo::control::FollowerOutputSmd smd(config(velocity_ff));
    rb_servo::Vec6 xi;
    xi.x = speed;
    smd.reset(poseAt(0.0), xi);
    rb_servo::Pose6D reference;
    rb_servo::Pose6D out;
    for (int i = 0; i < 2500; ++i) {
        reference = poseAt(speed * static_cast<double>(i) * kDt);
        out = smd.step(reference, xi, kDt);
    }
    // step() advances the state by dt, so compare its output with the ramp at
    // the end of that interval (the analytic continuous-time sample).
    const double reference_after_step = speed * 2500.0 * kDt;
    return reference_after_step - out.x;
}

bool testRampLag() {
    const double ff_lag = rampLag(true);
    RB_CHECK(std::abs(ff_lag) < 0.0005);

    const double no_ff_lag = rampLag(false);
    const double wn = kTwoPi * 3.5;
    const double analytic = 2.0 * 0.3 / wn;
    RB_CHECK(std::abs(no_ff_lag - analytic) <= 0.2 * analytic);
    return true;
}

double analyticFfGain(double frequency_hz, double natural_frequency_hz) {
    const double w = kTwoPi * frequency_hz;
    const double wn = kTwoPi * natural_frequency_hz;
    const double numerator = wn * wn * std::sqrt(wn * wn + 9.0 * w * w);
    const double denominator = std::pow(std::sqrt(wn * wn + w * w), 3.0);
    return numerator / denominator;
}

double measuredSineGain(double frequency_hz,
                        const rb_servo::FollowerOutputSmdConfig& cfg = config(true)) {
    constexpr double amplitude = 0.001;
    constexpr double settle_sec = 3.0;
    constexpr double measure_sec = 4.0;
    const double w = kTwoPi * frequency_hz;
    rb_servo::control::FollowerOutputSmd smd(cfg);
    rb_servo::Vec6 initial_xi;
    initial_xi.x = amplitude * w;
    smd.reset(poseAt(0.0), initial_xi);

    double sin_sum = 0.0;
    double cos_sum = 0.0;
    int count = 0;
    const int total = static_cast<int>((settle_sec + measure_sec) / kDt);
    for (int i = 1; i <= total; ++i) {
        const double t = static_cast<double>(i) * kDt;
        const double phase = w * t;
        rb_servo::Vec6 xi;
        xi.x = amplitude * w * std::cos(phase);
        const rb_servo::Pose6D out = smd.step(poseAt(amplitude * std::sin(phase)), xi, kDt);
        if (t > settle_sec) {
            sin_sum += out.x * std::sin(phase);
            cos_sum += out.x * std::cos(phase);
            ++count;
        }
    }
    const double output_amplitude =
        2.0 * std::sqrt(sin_sum * sin_sum + cos_sum * cos_sum) /
        static_cast<double>(count);
    return output_amplitude / amplitude;
}

bool testSineResponse() {
    const double gain_1 = measuredSineGain(1.0);
    const double gain_13 = measuredSineGain(13.0);
    const double gain_17 = measuredSineGain(17.0);
    for (const auto& [frequency, measured] :
         std::vector<std::pair<double, double>>{{1.0, gain_1}, {13.0, gain_13}, {17.0, gain_17}}) {
        const double analytic = analyticFfGain(frequency, 3.5);
        RB_CHECK(std::abs(measured - analytic) <= 0.25 * analytic);
    }
    RB_CHECK(gain_1 >= 0.9);
    RB_CHECK(gain_13 <= 0.25);
    RB_CHECK(gain_17 <= 0.16);
    return true;
}

bool testChunkBoundaryStep() {
    rb_servo::control::FollowerOutputSmd smd(config(true));
    smd.reset(poseAt(0.0), rb_servo::Vec6{});
    std::vector<double> acceleration;
    acceleration.reserve(1100);
    double x_nm2 = 0.0;
    double x_nm1 = 0.0;
    for (int i = 1; i <= 1100; ++i) {
        const double t = static_cast<double>(i) * kDt;
        // The second 2 mm command discontinuity represents a chunk swap. The
        // first identical transition provides the pre-boundary acceleration
        // peak, so this asserts that a boundary has no special dynamics.
        const double reference_x = t >= 1.25 ? 0.004 : (t >= 0.25 ? 0.002 : 0.0);
        const rb_servo::Pose6D out = smd.step(poseAt(reference_x), rb_servo::Vec6{}, kDt);
        acceleration.push_back((out.x - 2.0 * x_nm1 + x_nm2) / (kDt * kDt));
        x_nm2 = x_nm1;
        x_nm1 = out.x;
        if (t >= 1.25) {
            RB_CHECK(out.x <= 0.004 + 1e-10);
        }
    }

    const auto peak_abs = [&acceleration](int begin, int end) {
        double peak = 0.0;
        for (int i = begin; i < end; ++i) {
            peak = std::max(peak, std::abs(acceleration[static_cast<std::size_t>(i)]));
        }
        return peak;
    };
    const double pre_peak = peak_abs(125, 500);   // response to the first step
    const double post_peak = peak_abs(625, 1000); // response after the chunk swap
    RB_CHECK(pre_peak > 0.0);
    RB_CHECK(post_peak < 2.0 * pre_peak);

    int sign_changes = 0;
    int previous_sign = 0;
    const double sign_threshold = 0.01 * post_peak;
    for (int i = 625; i < 1000; ++i) {
        const double a = acceleration[static_cast<std::size_t>(i)];
        const int sign = (a > sign_threshold) - (a < -sign_threshold);
        if (sign != 0 && previous_sign != 0 && sign != previous_sign) ++sign_changes;
        if (sign != 0) previous_sign = sign;
    }
    RB_CHECK(sign_changes <= 2);
    return true;
}

bool testReseedRule() {
    // 2026-09-06: the reseed bounds moved from 0.05 m / 0.10 rad to the HARD divergence
    // latch (0.10 m / 0.30 rad). A 60 mm reference jump is now tracked, not snapped.
    rb_servo::control::FollowerOutputSmd smd(config());
    smd.reset(poseAt(0.0), rb_servo::Vec6{});
    const rb_servo::Pose6D near = poseAt(0.060);
    const rb_servo::Pose6D tracked = smd.step(near, rb_servo::Vec6{}, kDt);
    RB_CHECK(!smd.reseededLastStep());
    RB_CHECK(tracked.x < 0.010);              // one tick of a 3.5 Hz tracker, not a snap
    RB_CHECK(smd.lagPos() > 0.050);
    const rb_servo::Pose6D jumped = poseAt(0.200);
    const rb_servo::Pose6D out = smd.step(jumped, rb_servo::Vec6{}, kDt);
    RB_CHECK(smd.reseededLastStep());
    RB_CHECK(std::abs(out.x - jumped.x) < 1e-12);
    RB_CHECK(smd.lagPos() < 1e-12);
    const rb_servo::Pose6D again = smd.step(jumped, rb_servo::Vec6{}, kDt);
    RB_CHECK(!smd.reseededLastStep());
    (void)again;
    return true;
}

// PROFILE FEED-FORWARD: a constant-acceleration reference (what a jerk-limited plan
// looks like between jerk phases) is tracked without lag when the SMD is fed the
// sampled velocity AND acceleration; the legacy velocity-only path lags ~3a/wn^2.
double constantAccelLag(bool profile_ff) {
    constexpr double accel = 3.0;   // m/s^2, a quarter of the follower's linear a_max
    rb_servo::control::FollowerOutputSmd smd(config(true, profile_ff));
    smd.reset(poseAt(0.0), rb_servo::Vec6{});
    rb_servo::Pose6D out;
    double ref_after = 0.0;
    for (int i = 0; i < 500; ++i) {  // 1 s
        const double t = static_cast<double>(i) * kDt;
        rb_servo::Vec6 xi;
        xi.x = accel * t;
        rb_servo::Vec6 xi_dot;
        xi_dot.x = accel;
        out = smd.step(poseAt(0.5 * accel * t * t), xi, kDt, &xi_dot);
        ref_after = 0.5 * accel * (t + kDt) * (t + kDt);
    }
    return ref_after - out.x;
}

bool testProfileFeedforwardTracksAcceleration() {
    const double legacy = constantAccelLag(false);
    const double profile = constantAccelLag(true);
    const double wn = kTwoPi * 3.5;
    // Legacy: the ff velocity is low-passed at wn, so it trails by a/wn and the
    // tracker settles at (1 + 2 zeta) a / wn^2.
    const double legacy_analytic = 3.0 * 3.0 / (wn * wn);
    RB_CHECK(legacy > 0.5 * legacy_analytic);
    RB_CHECK(std::abs(profile) < 0.0005);
    RB_CHECK(std::abs(profile) < 0.05 * legacy);
    return true;
}

bool testOrientationStep() {
    rb_servo::control::FollowerOutputSmd smd(config());
    const rb_servo::Pose6D initial = rb_servo::math::poseFromSe3(pinocchio::SE3::Identity());
    const Eigen::AngleAxisd rotation(0.1, Eigen::Vector3d::UnitX());
    const rb_servo::Pose6D target = rb_servo::math::poseFromSe3(
        pinocchio::SE3(rotation.toRotationMatrix(), Eigen::Vector3d::Zero()));
    smd.reset(initial, rb_servo::Vec6{});

    double previous_angle = 0.0;
    rb_servo::Pose6D out;
    for (int i = 0; i < 2000; ++i) {
        out = smd.step(target, rb_servo::Vec6{}, kDt);
        const Eigen::Vector3d rotvec = rb_servo::math::log3(rb_servo::math::rotationFromPose(out));
        if (i == 0) RB_CHECK(rotvec.x() < 0.01);  // 0.1 rad is not a reseed (> only)
        RB_CHECK(rotvec.x() >= previous_angle - 1e-12);
        RB_CHECK(rotvec.x() <= 0.1 + 1e-10);
        previous_angle = rotvec.x();
        RB_CHECK(out.quaternion_xyzw.has_value());
        double norm_sq = 0.0;
        for (double value : *out.quaternion_xyzw) norm_sq += value * value;
        RB_CHECK(std::abs(std::sqrt(norm_sq) - 1.0) < 1e-12);
    }
    RB_CHECK(rb_servo::math::orientationDistanceRad(out, target) < 1e-6);
    return true;
}

// A changing-axis orientation profile exercises BODY angular derivatives; a
// fixed-axis sine cannot reveal reference/output frame confusion. Derivatives
// here come independently from SE(3) finite differences, not the follower helper.
bool testChangingAxisProfile() {
    const auto rotation = [](double t) {
        return rb_servo::math::exp3(Eigen::Vector3d(.12*std::sin(1.7*t),
            .09*std::sin(2.3*t), .08*std::sin(.8*t)));
    };
    const auto velocity = [&](double t) {
        constexpr double h = 1e-5;
        return Eigen::Vector3d(
            rb_servo::math::log3(rotation(t-h).transpose()*rotation(t+h))/(2*h));
    };
    const auto acceleration = [&](double t) {
        constexpr double h = 2e-5;
        return Eigen::Vector3d((velocity(t+h)-velocity(t-h))/(2*h));
    };
    rb_servo::control::FollowerOutputSmd smd(config(true,true));
    rb_servo::Pose6D initial = rb_servo::math::poseFromSe3(
        pinocchio::SE3(rotation(0), Eigen::Vector3d::Zero()));
    const auto w0 = velocity(0);
    smd.reset(initial, {0,0,0,w0.x(),w0.y(),w0.z()});
    double max_error = 0;
    for (int i=1;i<=2000;++i) {
        const double t=i*kDt;
        const auto target=rb_servo::math::poseFromSe3(
            pinocchio::SE3(rotation(t),Eigen::Vector3d::Zero()));
        const auto w=velocity(t), a=acceleration(t);
        const rb_servo::Vec6 v{0,0,0,w.x(),w.y(),w.z()}, acc{0,0,0,a.x(),a.y(),a.z()};
        const auto out=smd.step(target,v,kDt,&acc,true);
        RB_CHECK(!smd.reseededLastStep());
        max_error=std::max(max_error,rb_servo::math::orientationDistanceRad(out,target));
    }
    std::cout << "changing-axis profile max angular error=" << max_error << " rad\n";
    RB_CHECK(max_error < .003);
    return true;
}

// Independently describe the filtered angular-velocity vector in the fixed
// stand frame. The production implementation stores it in a moving output
// body, so matching this recurrence checks both input and retained-state
// transport, including a gate stop and a force-frame gauge change.
bool testPhysicalAngularLowPass() {
    auto cfg=config(true,false);
    cfg.profile_feedforward=false;
    rb_servo::control::FollowerOutputSmd physical(cfg),legacy(cfg),no_accel(cfg);
    Eigen::Matrix3d basis=rb_servo::math::exp3(Eigen::Vector3d(.4,-.3,.2));
    auto pose=[&](double tau) {
        const Eigen::Matrix3d R=basis*
            rb_servo::math::exp3(Eigen::Vector3d(.8*tau,0,0))*
            rb_servo::math::exp3(Eigen::Vector3d(0,.6*std::sin(.9*tau),0));
        return rb_servo::math::poseFromSe3(pinocchio::SE3(R,Eigen::Vector3d::Zero()));
    };
    auto omega_stand=[&](double tau,double gate) {
        const Eigen::Matrix3d Rx=rb_servo::math::exp3(Eigen::Vector3d(.8*tau,0,0));
        return Eigen::Vector3d(basis*(Eigen::Vector3d(.8,0,0)+
            Rx*Eigen::Vector3d(0,.54*std::cos(.9*tau),0))*gate);
    };
    const auto initial=pose(0);
    Eigen::Vector3d expected=omega_stand(0,1);
    const Eigen::Vector3d seed=rb_servo::math::rotationFromPose(initial).transpose()*expected;
    const rb_servo::Vec6 xi0{0,0,0,seed.x(),seed.y(),seed.z()};
    physical.reset(initial,xi0);legacy.reset(initial,xi0);no_accel.reset(initial,xi0);
    RB_CHECK((physical.filteredAngularVelocityStand()-expected).norm()<1e-12);
    double tau=0,max_error=0,legacy_error=0;
    for(int i=1;i<=3000;++i) {
        const double gate=i<1000?1.:(i<1500?.25:(i<1700?0.:.7));
        if(i==2000) {
            const Eigen::Quaterniond fold(rb_servo::math::exp3(Eigen::Vector3d(.08,-.05,.07)));
            physical.shift(Eigen::Vector3d::Zero(),fold);
            legacy.shift(Eigen::Vector3d::Zero(),fold);
            no_accel.shift(Eigen::Vector3d::Zero(),fold);
            basis=fold.toRotationMatrix()*basis;expected=fold*expected;
            RB_CHECK((physical.filteredAngularVelocityStand()-expected).norm()<1e-12);
        }
        tau+=gate*kDt;
        const auto target=pose(tau);const Eigen::Vector3d world=omega_stand(tau,gate);
        const Eigen::Vector3d body=rb_servo::math::rotationFromPose(target).transpose()*world;
        const rb_servo::Vec6 v{0,0,0,body.x(),body.y(),body.z()};
        expected+=kTwoPi*cfg.nf_angular_hz*(world-expected)*kDt;
        const auto output=physical.step(target,v,kDt,nullptr,true);
        legacy.step(target,v,kDt,nullptr,false);
        const rb_servo::Vec6 unused_acceleration{1e6,-1e6,1e6,1e6,-1e6,1e6};
        const auto control=no_accel.step(target,v,kDt,&unused_acceleration,true);
        RB_CHECK(!physical.reseededLastStep());
        RB_CHECK(rb_servo::math::orientationDistanceRad(output,control)<1e-12);
        RB_CHECK(rb_servo::math::positionDistance(output,control)<1e-12);
        max_error=std::max(max_error,(physical.filteredAngularVelocityStand()-expected).norm());
        legacy_error=std::max(legacy_error,(legacy.filteredAngularVelocityStand()-expected).norm());
    }
    std::cout<<"physical FF-off stand-frame LPF error="<<max_error
             <<" rad/s; untransported negative control="<<legacy_error<<" rad/s\n";
    RB_CHECK(max_error<1e-10);
    RB_CHECK(legacy_error>1e-5);
    return true;
}

// The low-frequency fix must remove the known 2 Hz amplification, not merely
// make a step plot look smooth. Exercise the production integrator across the
// input band, and preserve the independent angular response exactly.
bool testLinearFeedforwardGain() {
    auto candidate = config();
    candidate.nf_linear_hz = 6.0;
    for (double gain : {0.0, 0.2, 0.3660254037844386}) {
        candidate.velocity_ff_linear_gain = gain;
        for (double f : {0.5, 1.0, 1.5, 2.0, 3.0, 5.0, 10.0, 13.0, 20.0, 40.0}) {
            RB_CHECK(measuredSineGain(f, candidate) <= 1.001);
        }
    }
    RB_CHECK(measuredSineGain(2.0) > 1.25);  // Historical negative control.
    candidate.velocity_ff_linear_gain = 0.0;
    rb_servo::control::FollowerOutputSmd linear_only(candidate), baseline(config());
    linear_only.reset(poseAt(0.0), {}); baseline.reset(poseAt(0.0), {});
    for (int i = 1; i <= 2000; ++i) {
        const double t = i * kDt;
        const Eigen::Matrix3d R = rb_servo::math::exp3(Eigen::Vector3d(
            0.06 * std::sin(2.0*t), 0.04 * std::sin(3.0*t), 0.02 * std::cos(t)));
        const auto reference = rb_servo::math::poseFromSe3(pinocchio::SE3(
            R, Eigen::Vector3d(0.01 * std::sin(t),0,0)));
        const rb_servo::Vec6 velocity{0.01*std::cos(t),0,0,0.12*std::cos(2*t),0.12*std::cos(3*t),-0.02*std::sin(t)};
        const auto a = linear_only.step(reference,velocity,kDt,nullptr,true);
        const auto b = baseline.step(reference,velocity,kDt,nullptr,true);
        RB_CHECK(rb_servo::math::orientationDistanceRad(a,b) < 1e-12);
    }
    return true;
}

bool testSelectedRealConditioner() {
    const auto path=std::filesystem::path(__FILE__).parent_path().parent_path()/"config/stack_real.yaml";
    const auto stack=rb_servo::loadConfigFromYaml(path.string());
    const auto& profiles=stack.cartesian_control.tcp_pose_target_profiles;
    const auto selected=std::find_if(profiles.begin(),profiles.end(),[](const auto& p){return p.name=="flow_infer_fresh";});
    RB_CHECK(selected!=profiles.end());
    const auto cfg=selected->ruckig_follower.output_smd;
    if (cfg.mode == rb_servo::FollowerOutputSmdMode::PositionLowpass2) {
        RB_CHECK(cfg.enable && !cfg.velocity_ff && !cfg.profile_feedforward);
        RB_CHECK(cfg.damping_ratio >= std::sqrt(0.5));
        for (double f : {0.5,1.0,2.0,3.0,5.0,8.0,10.0,13.0,20.0,25.0}) {
            RB_CHECK(measuredSineGain(f,cfg) <= 1.001);
        }
    } else {
        RB_CHECK(cfg.enable && cfg.velocity_ff && !cfg.profile_feedforward);
        RB_CHECK(cfg.velocity_ff_linear_gain==0.8 && cfg.nf_linear_hz==3.5);
        RB_CHECK(cfg.nf_angular_hz==2.5 && cfg.damping_ratio==1.0);
        for (double f : {2.0,3.0,10.0,13.0,20.0}) {
            RB_CHECK(measuredSineGain(f,cfg) < 0.92*measuredSineGain(f));
        }
    }
    // A moving reference that actually stops exercises retained velocity FF;
    // a position step with zero input velocity does not expose this transient.
    auto stop=[](const rb_servo::FollowerOutputSmdConfig& c) {
        rb_servo::control::FollowerOutputSmd filter(c);
        constexpr double speed=.15, stop_time=2.0;
        filter.reset(poseAt(0.0),rb_servo::Vec6{speed,0,0,0,0,0});
        double overshoot=0,ramp_error=0;rb_servo::Pose6D output;
        for (int i=1;i<=2000;++i) {
            const double t=i*kDt;const bool moving=t<=stop_time;
            const auto target=poseAt(speed*std::min(t,stop_time));
            output=filter.step(target,rb_servo::Vec6{moving?speed:0,0,0,0,0,0},kDt);
            if (i==1000) ramp_error=std::abs(target.x-output.x);
            if (!moving) overshoot=std::max(overshoot,output.x-target.x);
        }
        return std::array<double,3>{ramp_error,overshoot,std::abs(output.x-speed*stop_time)};
    };
    const auto candidate=stop(cfg),baseline=stop(config());
    std::cout << "selected 150 mm/s ramp: lag=" << candidate[0]*1000
              << " mm, stop overshoot=" << baseline[1]*1000 << " -> " << candidate[1]*1000 << " mm\n";
    if (cfg.mode == rb_servo::FollowerOutputSmdMode::PositionLowpass2) {
        const double analytic_ramp_lag = 2.0*cfg.damping_ratio*.15/(kTwoPi*cfg.nf_linear_hz);
        RB_CHECK(std::abs(candidate[0]-analytic_ramp_lag) < 1e-6);
    } else {
        RB_CHECK(candidate[0] < .003);
    }
    RB_CHECK(candidate[1] < baseline[1]);
    RB_CHECK(candidate[2] < 1e-8);
    return true;
}

rb_servo::FollowerOutputSmdConfig positionLowpass2Config(double damping = std::sqrt(0.5)) {
    auto cfg = config(false, false);
    cfg.mode = rb_servo::FollowerOutputSmdMode::PositionLowpass2;
    cfg.nf_linear_hz = 5.0;
    cfg.nf_angular_hz = 4.0;
    cfg.damping_ratio = damping;
    return cfg;
}

// A fixed-axis SO(3) trajectory commutes, so its transfer must match the scalar
// Tustin response independently of the chosen physical axis and quaternion sign.
// Test >=4 complete periods, including .1 Hz; short non-integer sine records can
// hide low-frequency peaking through leakage/transient bias.
bool testPositionLowpass2FrequencyResponse() {
    const Eigen::Vector3d axis = Eigen::Vector3d(.4,-.7,.2).normalized();
    constexpr double amplitude_m = .001, amplitude_rad = .02;
    for (double damping : {std::sqrt(0.5), .8, 1.0}) {
        const auto cfg = positionLowpass2Config(damping);
        for (double f : {.1,.5,1.,2.,3.,4.,5.,8.,10.,13.,17.,20.,25.}) {
            rb_servo::control::FollowerOutputSmd smd(cfg);
            const double w = kTwoPi*f;
            const Eigen::Vector3d initial_w = amplitude_rad*w*axis;
            smd.reset(poseAt(0), {amplitude_m*w,0,0,initial_w.x(),initial_w.y(),initial_w.z()});
            const int settle = 1500;
            const int samples = static_cast<int>(std::ceil(std::max(4.,4./f)/kDt));
            double linear_sin=0,linear_cos=0,angular_sin=0,angular_cos=0;
            for (int i=1;i<=settle+samples;++i) {
                const double phase=w*i*kDt;
                auto reference=rb_servo::math::poseFromSe3(pinocchio::SE3(
                    rb_servo::math::exp3(amplitude_rad*std::sin(phase)*axis),
                    Eigen::Vector3d(amplitude_m*std::sin(phase),0,0)));
                if (i%2==0) for (double& q:*reference.quaternion_xyzw) q=-q;
                // Position-only means velocity/acceleration must not be a second
                // inconsistent reference. Large finite values make leakage fail.
                const rb_servo::Vec6 unused{100,-100,100,-100,100,-100};
                const auto out=smd.step(reference,unused,kDt,&unused,true);
                RB_CHECK(!smd.reseededLastStep());
                if (i<=settle) continue;
                const double angle=axis.dot(rb_servo::math::log3(rb_servo::math::rotationFromPose(out)));
                linear_sin+=out.x*std::sin(phase); linear_cos+=out.x*std::cos(phase);
                angular_sin+=angle*std::sin(phase); angular_cos+=angle*std::cos(phase);
            }
            const double linear_gain=2*std::hypot(linear_sin,linear_cos)/(samples*amplitude_m);
            const double angular_gain=2*std::hypot(angular_sin,angular_cos)/(samples*amplitude_rad);
            const auto analytic=[&](double nf) {
                const double wn=kTwoPi*nf;
                const double warped=2/kDt*std::tan(.5*w*kDt);
                return wn*wn/std::hypot(wn*wn-warped*warped,2*damping*wn*warped);
            };
            RB_CHECK(linear_gain<=1.00001 && angular_gain<=1.00001);
            RB_CHECK(std::abs(linear_gain-analytic(cfg.nf_linear_hz))<1e-5);
            RB_CHECK(std::abs(angular_gain-analytic(cfg.nf_angular_hz))<1e-5);
        }
    }
    return true;
}

bool testPositionLowpass2RampAndStop() {
    for (double damping : {std::sqrt(0.5), .8, 1.0}) {
        const auto cfg=positionLowpass2Config(damping);
        rb_servo::control::FollowerOutputSmd smd(cfg);
        constexpr double speed=.15, omega=.2;
        smd.reset(poseAt(0), {speed,0,0,0,0,omega});
        double max_stop_overshoot=0;
        rb_servo::Pose6D out;
        for (int i=1;i<=2500;++i) {
            const double t=i*kDt,phase=std::min(t,2.0);
            const auto reference=rb_servo::math::poseFromSe3(pinocchio::SE3(
                rb_servo::math::exp3(Eigen::Vector3d(0,0,omega*phase)),
                Eigen::Vector3d(speed*phase,0,0)));
            out=smd.step(reference,{},kDt,nullptr,true);
            RB_CHECK(!smd.reseededLastStep());
            if (i==1000) {
                RB_CHECK(std::abs((reference.x-out.x)-2*damping*speed/(kTwoPi*cfg.nf_linear_hz))<1e-8);
                const double lag=rb_servo::math::orientationDistanceRad(reference,out);
                RB_CHECK(std::abs(lag-2*damping*omega/(kTwoPi*cfg.nf_angular_hz))<1e-8);
            }
            if (i>1000) max_stop_overshoot=std::max(max_stop_overshoot,out.x-.3);
        }
        RB_CHECK(std::abs(out.x-.3)<1e-10);
        // A nonpeaking sinusoidal response need not have a monotone step/stop.
        // The bounded overshoot is measured explicitly, rather than claiming it
        // is absent for underdamping (Butterworth damping has 4.3% step overshoot).
        RB_CHECK(max_stop_overshoot<.001);
        std::cout<<"position_lowpass2 zeta="<<damping<<" 150 mm/s stop overshoot="
                 <<max_stop_overshoot*1000<<" mm\n";
    }
    return true;
}

bool testPositionLowpass2ChangingAxisFoldAndReset() {
    const auto cfg=positionLowpass2Config(.8);
    rb_servo::control::FollowerOutputSmd baseline(cfg),folded(cfg);
    Eigen::Quaterniond gauge=Eigen::Quaterniond::Identity();
    Eigen::Vector3d dp=Eigen::Vector3d::Zero();
    const auto target=[](double t) {
        return rb_servo::math::poseFromSe3(pinocchio::SE3(
            rb_servo::math::exp3(Eigen::Vector3d(.4,-.2,.3))*
            rb_servo::math::exp3(Eigen::Vector3d(.12*std::sin(2*t),.08*std::cos(3*t),.1*std::sin(t))),
            Eigen::Vector3d(.01*std::sin(t),.02*std::sin(2*t),.01*std::cos(t))));
    };
    baseline.reset(target(0),{});folded.reset(target(0),{});
    double t=0,max_error=0;
    for (int i=1;i<=4000;++i) {
        // Recorded 500 Hz scheduling jitter changes dt, not the physical state.
        const double dt=std::array<double,4>{.0018,.0022,.0019,.0021}[i%4];
        t+=dt;
        if (i==1500) {
            gauge=Eigen::Quaterniond(rb_servo::math::exp3(Eigen::Vector3d(.08,-.05,.07)));
            dp=Eigen::Vector3d(.007,-.004,.006);
            folded.shift(dp,gauge);
        }
        const auto reference=target(t);
        auto shifted=rb_servo::math::poseFromSe3(pinocchio::SE3(
            gauge.toRotationMatrix()*rb_servo::math::rotationFromPose(reference),
            Eigen::Vector3d(reference.x,reference.y,reference.z)+dp));
        if (i%2==0) for(double& q:*shifted.quaternion_xyzw) q=-q;
        const auto a=baseline.step(reference,{},dt,nullptr,true);
        const rb_servo::Vec6 ignored{42,-7,13,100,-200,300};
        const auto b=folded.step(shifted,ignored,dt,&ignored,false);
        const auto expected=rb_servo::math::poseFromSe3(pinocchio::SE3(
            gauge.toRotationMatrix()*rb_servo::math::rotationFromPose(a),
            Eigen::Vector3d(a.x,a.y,a.z)+dp));
        RB_CHECK(!baseline.reseededLastStep() && !folded.reseededLastStep());
        RB_CHECK(rb_servo::math::positionDistance(b,expected)<1e-11);
        RB_CHECK(rb_servo::math::orientationDistanceRad(b,expected)<1e-11);
        RB_CHECK(std::abs(Eigen::Quaterniond(rb_servo::math::rotationFromPose(b)).norm()-1)<1e-12);
        max_error=std::max(max_error,rb_servo::math::orientationDistanceRad(a,reference));
    }
    RB_CHECK(max_error<.05);
    // A stationary hold reset must discard BOTH state velocity and reference
    // history. Otherwise the first trapezoidal step can pull toward an old input.
    const auto hold=poseAt(.6);
    folded.reset(hold,{});
    for(int i=0;i<100;++i) {
        const auto out=folded.step(hold,{},kDt,nullptr,true);
        RB_CHECK(rb_servo::math::positionDistance(out,hold)<1e-12);
        RB_CHECK(rb_servo::math::orientationDistanceRad(out,hold)<1e-12);
    }
    const auto unchanged=folded.step(poseAt(.61),{},0,nullptr,true);
    RB_CHECK(rb_servo::math::positionDistance(unchanged,hold)<1e-12);
    folded.step(poseAt(.61),{},std::numeric_limits<double>::quiet_NaN(),nullptr,true);
    RB_CHECK(rb_servo::math::positionDistance(folded.step(hold,{},kDt),hold)<1e-12);
    folded.deactivate();
    const auto cold=folded.step(poseAt(.7),{},kDt,nullptr,true);
    RB_CHECK(folded.reseededLastStep() && std::abs(cold.x-.7)<1e-12);
    const auto jumped=folded.step(poseAt(.9),{},kDt,nullptr,true);
    RB_CHECK(folded.reseededLastStep() && std::abs(jumped.x-.9)<1e-12);
    RB_CHECK(std::abs(folded.step(poseAt(.9),{},kDt).x-.9)<1e-12);
    // A moving reset does retain the supplied physical velocity.
    folded.reset(hold,{.04,0,0,0,0,0});
    const auto moving=folded.step(hold,{},kDt,nullptr,true);
    RB_CHECK(moving.x>hold.x && moving.x<hold.x+.04*kDt);
    return true;
}

// Socket-free recorded-reference replay through the production conditioner.
// It intentionally does not recreate the follower, IK, force feedback or plant.
// Explicit initial state is mandatory; missing reference velocity is accepted
// only for a translation-only fixture with zero linear FF and identity rotation.
int replayRecordedReference(const char* input_path, const char* output_path,
                            const char* stack_path, const char* profile_name) {
    using Json = nlohmann::json;
    const auto cfg = rb_servo::loadConfigFromYaml(stack_path);
    const auto& profiles = cfg.cartesian_control.tcp_pose_target_profiles;
    const auto it = std::find_if(profiles.begin(),profiles.end(),[&](const auto& p){return p.name==profile_name;});
    if (it==profiles.end()) throw std::runtime_error("recorded replay profile missing");
    const auto smd_cfg = it->ruckig_follower.output_smd;
    if (!smd_cfg.enable || smd_cfg.profile_feedforward)
        throw std::runtime_error("recorded replay requires enabled conditioner with profile FF off");
    rb_servo::control::FollowerOutputSmd smd(smd_cfg);
    std::ifstream input(input_path); std::ofstream output(output_path);
    if (!input || !output) throw std::runtime_error("recorded replay file unavailable");
    auto finiteArray=[](const Json& a, std::size_t n) {
        if (!a.is_array() || a.size()!=n) throw std::runtime_error("invalid replay array");
        for (const auto& x:a) if (!x.is_number() || !std::isfinite(x.get<double>()))
            throw std::runtime_error("nonfinite replay array");
    };
    auto pose=[&](const Json& a) {
        finiteArray(a,7);
        const Eigen::Quaterniond q(a[3].get<double>(),a[4].get<double>(),a[5].get<double>(),a[6].get<double>());
        if (std::abs(q.norm()-1)>1e-6) throw std::runtime_error("invalid replay quaternion");
        return rb_servo::math::poseFromSe3(pinocchio::SE3(q.normalized().toRotationMatrix(),
            Eigen::Vector3d(a[0].get<double>(),a[1].get<double>(),a[2].get<double>())));
    };
    auto velocity=[&](const Json& a) {finiteArray(a,6); return rb_servo::Vec6{a[0],a[1],a[2],a[3],a[4],a[5]};};
    std::string line; std::size_t count=0;
    while (std::getline(input,line)) {
        if (line.empty()) continue;
        const auto row=Json::parse(line);
        if (!count) smd.reset(pose(row.at("initial_pose")),velocity(row.at("initial_velocity")));
        const auto reference=pose(row.at("reference"));
        const double dt=row.at("dt_sec");
        if (!std::isfinite(dt) || dt<=0 || dt>0.02) throw std::runtime_error("invalid replay dt");
        rb_servo::Vec6 xi{};
        if (row.contains("reference_velocity")) xi=velocity(row.at("reference_velocity"));
        else if (smd_cfg.mode==rb_servo::FollowerOutputSmdMode::LegacySmd &&
                 (smd_cfg.velocity_ff_linear_gain!=0.0 ||
                  rb_servo::math::orientationDistanceRad(reference,poseAt(0.0))>1e-12))
            throw std::runtime_error("reference_velocity required for this replay");
        if (row.contains("shift_m")) {
            const auto& d=row.at("shift_m"); const auto& q=row.at("shift_quat_wxyz");
            finiteArray(d,3);finiteArray(q,4);
            const Eigen::Quaterniond rotation(q[0].get<double>(),q[1].get<double>(),q[2].get<double>(),q[3].get<double>());
            if (std::abs(rotation.norm()-1)>1e-6) throw std::runtime_error("invalid shift quaternion");
            smd.shift(Eigen::Vector3d(d[0].get<double>(),d[1].get<double>(),d[2].get<double>()),rotation.normalized());
        }
        const auto result=smd.step(reference,xi,dt,nullptr,true);
        const Eigen::Quaterniond q(rb_servo::math::rotationFromPose(result));
        output << Json{{"t",row.at("t")},{"stage",{result.x,result.y,result.z,q.w(),q.x(),q.y(),q.z()}},
            {"lag_m",smd.lagPos()},{"lag_rad",smd.lagAng()},
            {"reseeded",smd.reseededLastStep()}}.dump() << '\n';
        ++count;
    }
    if (!count) throw std::runtime_error("empty recorded replay");
    std::cout << "replayed " << count << " conditioner samples\n";
    return 0;
}

}  // namespace

bool testCurrentTwistStandAfterReset() {
    // The hand-over twist a Hold reads off this conditioner is the stand-frame twist it
    // was seeded with (2026-09-06), independent of the output orientation.
    rb_servo::control::FollowerOutputSmd smd(config());
    rb_servo::Pose6D pose = poseAt(0.0);
    pose.rz = 1.2;
    pose.rx = -0.4;
    const rb_servo::Vec6 xi{0.12, -0.03, 0.05, 0.2, 0.1, -0.3};
    smd.reset(pose, xi);
    const rb_servo::Vec6 back = smd.currentTwistStand();
    RB_CHECK(std::abs(back.x - xi.x) < 1e-12 && std::abs(back.y - xi.y) < 1e-12 &&
             std::abs(back.z - xi.z) < 1e-12);
    // reset() stores the angular part as given; the accessor rotates the body rate out
    // through the same orientation, so the norm is preserved.
    const double n_in = std::sqrt(xi.rx * xi.rx + xi.ry * xi.ry + xi.rz * xi.rz);
    const double n_out = std::sqrt(back.rx * back.rx + back.ry * back.ry + back.rz * back.rz);
    RB_CHECK(std::abs(n_in - n_out) < 1e-9);
    return true;
}

int main(int argc, char** argv) {
    if (argc!=1) {
        try {
            if (argc==6 && std::string(argv[1])=="--replay-reference")
                return replayRecordedReference(argv[2],argv[3],argv[4],argv[5]);
            throw std::runtime_error("usage: test_follower_output_smd [--replay-reference INPUT OUTPUT STACK_YAML PROFILE]");
        } catch (const std::exception& e) {std::cerr << e.what() << '\n';return 1;}
    }
    if (!testCurrentTwistStandAfterReset()) return 1;
    if (!testStepNoOvershoot()) return 1;
    if (!testRampLag()) return 1;
    if (!testSineResponse()) return 1;
    if (!testChunkBoundaryStep()) return 1;
    if (!testReseedRule()) return 1;
    if (!testProfileFeedforwardTracksAcceleration()) return 1;
    if (!testOrientationStep()) return 1;
    if (!testChangingAxisProfile()) return 1;
    if (!testPhysicalAngularLowPass()) return 1;
    if (!testLinearFeedforwardGain()) return 1;
    if (!testPositionLowpass2FrequencyResponse()) return 1;
    if (!testPositionLowpass2RampAndStop()) return 1;
    if (!testPositionLowpass2ChangingAxisFoldAndReset()) return 1;
    if (!testSelectedRealConditioner()) return 1;
    return 0;
}
