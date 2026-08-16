#include "rb_servo/control/follower_output_smd.hpp"

#include "rb_servo/math/se3.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

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

rb_servo::FollowerOutputSmdConfig config(bool velocity_ff = true) {
    rb_servo::FollowerOutputSmdConfig cfg;
    cfg.enable = true;
    cfg.nf_linear_hz = 3.5;
    cfg.nf_angular_hz = 2.5;
    cfg.damping_ratio = 1.0;
    cfg.velocity_ff = velocity_ff;
    cfg.velocity_ff_lpf_hz = 0.0;
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

double measuredSineGain(double frequency_hz) {
    constexpr double amplitude = 0.001;
    constexpr double settle_sec = 3.0;
    constexpr double measure_sec = 4.0;
    const double w = kTwoPi * frequency_hz;
    rb_servo::control::FollowerOutputSmd smd(config(true));
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
    rb_servo::control::FollowerOutputSmd smd(config());
    smd.reset(poseAt(0.0), rb_servo::Vec6{});
    const rb_servo::Pose6D jumped = poseAt(0.060);
    const rb_servo::Pose6D out = smd.step(jumped, rb_servo::Vec6{}, kDt);
    RB_CHECK(std::abs(out.x - jumped.x) < 1e-12);
    RB_CHECK(smd.lagPos() < 1e-12);
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

}  // namespace

int main() {
    if (!testStepNoOvershoot()) return 1;
    if (!testRampLag()) return 1;
    if (!testSineResponse()) return 1;
    if (!testChunkBoundaryStep()) return 1;
    if (!testReseedRule()) return 1;
    if (!testOrientationStep()) return 1;
    return 0;
}
