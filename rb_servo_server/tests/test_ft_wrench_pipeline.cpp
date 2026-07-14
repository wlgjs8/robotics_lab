#include <cmath>
#include <iostream>
#include <limits>

#include "rb_servo/sensor/ft_wrench_pipeline.hpp"

namespace {

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.80665;

bool near(double lhs, double rhs, double tolerance = 1e-9) {
    return std::abs(lhs - rhs) <= tolerance;
}

rb_servo::FtWrenchPipelineConfig baseConfig() {
    rb_servo::FtWrenchPipelineConfig config;
    config.enable = true;
    config.frame_configured = true;
    config.max_sample_age_sec = 0.1;
    config.max_source_stall_sec = 0.01;
    config.control_lpf_alpha = 0.25;
    return config;
}

rb_servo::FtRawSample sample(uint64_t sequence, uint64_t host_time_ns) {
    rb_servo::FtRawSample value;
    value.host_time_ns = host_time_ns;
    value.source_sequence = sequence;
    value.source_sequence_valid = true;
    value.fields_present = true;
    value.sensor_present = true;
    return value;
}

bool testBiasThenTransformWithMomentArm() {
    rb_servo::FtWrenchPipelineConfig config = baseConfig();
    config.t_tcp_sensor.y = 1.0;
    config.sensor_bias.fx = 1.0;
    rb_servo::FtWrenchPipeline pipeline(config);

    rb_servo::FtRawSample raw = sample(1, 1'000'000'000ULL);
    raw.wrench_sensor.fx = 2.0;
    const auto out = pipeline.process(raw, rb_servo::Pose6D{}, raw.host_time_ns);

    RB_CHECK(out.healthy);
    RB_CHECK(near(out.wrench_tcp.fx, 1.0));
    RB_CHECK(near(out.wrench_tcp.tz, -1.0));
    return true;
}

bool testAcceptedRbpodoEftForceAxisMapping() {
    rb_servo::FtWrenchPipelineConfig config = baseConfig();
    config.t_tcp_sensor.z = -0.202642;
    rb_servo::FtWrenchPipeline pipeline(config);

    rb_servo::FtRawSample raw = sample(1, 1'000'000'000ULL);
    raw.wrench_sensor.fx = 3.0;
    raw.wrench_sensor.fy = 4.0;
    raw.wrench_sensor.fz = 5.0;
    const auto out = pipeline.process(raw, rb_servo::Pose6D{}, raw.host_time_ns);

    RB_CHECK(out.healthy);
    // The accepted physical profile has no runtime yaw: rbpodo EFT +X/+Y/+Z
    // remain TCP +X/+Y/+Z. The 202.642 mm sensor-to-TCP lever arm still shifts
    // the wrench moment to the TCP origin.
    RB_CHECK(near(out.wrench_tcp.fx, 3.0));
    RB_CHECK(near(out.wrench_tcp.fy, 4.0));
    RB_CHECK(near(out.wrench_tcp.fz, 5.0));
    RB_CHECK(near(out.wrench_tcp.tx, 0.202642 * 4.0));
    RB_CHECK(near(out.wrench_tcp.ty, -0.202642 * 3.0));
    RB_CHECK(near(out.wrench_tcp.tz, 0.0));
    return true;
}

bool testRotationAndPayloadCompensation() {
    rb_servo::FtWrenchPipelineConfig rotation_config = baseConfig();
    rotation_config.t_tcp_sensor.rz = kPi * 0.5;
    rb_servo::FtWrenchPipeline rotation_pipeline(rotation_config);
    rb_servo::FtRawSample rotated = sample(1, 1'000'000'000ULL);
    rotated.wrench_sensor.fx = 3.0;
    const auto rotation_out = rotation_pipeline.process(
        rotated,
        rb_servo::Pose6D{},
        rotated.host_time_ns
    );
    RB_CHECK(rotation_out.healthy);
    RB_CHECK(near(rotation_out.wrench_tcp.fx, 0.0, 1e-12));
    RB_CHECK(near(rotation_out.wrench_tcp.fy, 3.0, 1e-12));
    RB_CHECK(near(rotation_out.gravity_tcp[0], 0.0, 1e-12));
    RB_CHECK(near(rotation_out.gravity_tcp[1], 0.0, 1e-12));
    RB_CHECK(near(rotation_out.gravity_tcp[2], -kGravity, 1e-12));

    rb_servo::FtWrenchPipelineConfig payload_config = baseConfig();
    payload_config.payload_mass_kg = 1.0;
    payload_config.payload_com_tcp_m = {0.1, 0.0, 0.0};
    rb_servo::FtWrenchPipeline payload_pipeline(payload_config);
    rb_servo::FtRawSample payload = sample(1, 2'000'000'000ULL);
    payload.wrench_sensor.fz = -kGravity;
    payload.wrench_sensor.ty = kGravity * 0.1;
    const auto payload_out = payload_pipeline.process(
        payload,
        rb_servo::Pose6D{},
        payload.host_time_ns
    );
    RB_CHECK(payload_out.healthy);
    RB_CHECK(near(payload_out.fast_external_wrench_tcp.fz, 0.0, 1e-10));
    RB_CHECK(near(payload_out.fast_external_wrench_tcp.ty, 0.0, 1e-10));

    rb_servo::FtWrenchPipeline tilted_pipeline(payload_config);
    rb_servo::Pose6D tilted_tcp;
    tilted_tcp.rx = kPi * 0.5;
    rb_servo::FtRawSample tilted = sample(1, 3'000'000'000ULL);
    tilted.wrench_sensor.fy = -kGravity;
    tilted.wrench_sensor.tz = -kGravity * 0.1;
    const auto tilted_out = tilted_pipeline.process(tilted, tilted_tcp, tilted.host_time_ns);
    RB_CHECK(tilted_out.healthy);
    RB_CHECK(near(tilted_out.gravity_tcp[0], 0.0, 1e-12));
    RB_CHECK(near(tilted_out.gravity_tcp[1], -kGravity, 1e-12));
    RB_CHECK(near(tilted_out.gravity_tcp[2], 0.0, 1e-12));
    RB_CHECK(near(tilted_out.fast_external_wrench_tcp.fy, 0.0, 1e-10));
    RB_CHECK(near(tilted_out.fast_external_wrench_tcp.tz, 0.0, 1e-10));
    return true;
}

bool testResidualTareAndLowPass() {
    rb_servo::FtWrenchPipelineConfig config = baseConfig();
    config.residual_tare_tcp.fx = 1.0;
    rb_servo::FtWrenchPipeline pipeline(config);

    rb_servo::FtRawSample first = sample(1, 1'000'000'000ULL);
    first.wrench_sensor.fx = 1.0;
    const auto first_out = pipeline.process(first, rb_servo::Pose6D{}, first.host_time_ns);
    RB_CHECK(first_out.healthy);
    RB_CHECK(near(first_out.fast_external_wrench_tcp.fx, 0.0));

    rb_servo::FtRawSample second = sample(2, 1'001'000'000ULL);
    second.wrench_sensor.fx = 11.0;
    const auto second_out = pipeline.process(second, rb_servo::Pose6D{}, second.host_time_ns);
    RB_CHECK(second_out.healthy);
    RB_CHECK(near(second_out.fast_external_wrench_tcp.fx, 10.0));
    RB_CHECK(near(second_out.control_external_wrench_tcp.fx, 2.5));
    return true;
}

bool testLowPassAdvancesOnlyOnNewAcquisition() {
    rb_servo::FtWrenchPipeline pipeline(baseConfig());

    rb_servo::FtRawSample first = sample(1, 1'000'000'000ULL);
    const auto first_out = pipeline.process(first, rb_servo::Pose6D{}, first.host_time_ns);
    RB_CHECK(first_out.healthy);
    RB_CHECK(first_out.freshness_advanced);
    RB_CHECK(near(first_out.control_external_wrench_tcp.fx, 0.0));

    rb_servo::FtRawSample second = sample(2, 1'001'000'000ULL);
    second.wrench_sensor.fx = 8.0;
    const auto second_out = pipeline.process(second, rb_servo::Pose6D{}, second.host_time_ns);
    RB_CHECK(second_out.healthy);
    RB_CHECK(second_out.freshness_advanced);
    RB_CHECK(near(second_out.control_external_wrench_tcp.fx, 2.0));

    rb_servo::FtRawSample duplicate = second;
    duplicate.host_time_ns = 1'002'000'000ULL;
    const auto duplicate_out = pipeline.process(
        duplicate,
        rb_servo::Pose6D{},
        duplicate.host_time_ns
    );
    RB_CHECK(duplicate_out.healthy);
    RB_CHECK(!duplicate_out.freshness_advanced);
    RB_CHECK(near(duplicate_out.fast_external_wrench_tcp.fx, 8.0));
    RB_CHECK(near(duplicate_out.control_external_wrench_tcp.fx, 2.0));

    rb_servo::FtRawSample third = sample(3, 1'003'000'000ULL);
    third.wrench_sensor.fx = 8.0;
    const auto third_out = pipeline.process(third, rb_servo::Pose6D{}, third.host_time_ns);
    RB_CHECK(third_out.healthy);
    RB_CHECK(third_out.freshness_advanced);
    RB_CHECK(near(third_out.control_external_wrench_tcp.fx, 3.5));
    return true;
}

bool testFreshnessUsesSourceSequenceNotValueChanges() {
    rb_servo::FtWrenchPipeline pipeline(baseConfig());
    rb_servo::FtRawSample first = sample(10, 1'000'000'000ULL);
    first.wrench_sensor.fz = 4.0;
    const auto first_out = pipeline.process(first, rb_servo::Pose6D{}, first.host_time_ns);
    RB_CHECK(first_out.healthy);
    RB_CHECK(first_out.freshness_advanced);

    rb_servo::FtRawSample same_value = sample(11, 1'005'000'000ULL);
    same_value.wrench_sensor.fz = 4.0;
    const auto same_value_out = pipeline.process(
        same_value,
        rb_servo::Pose6D{},
        same_value.host_time_ns
    );
    RB_CHECK(same_value_out.healthy);
    RB_CHECK(same_value_out.freshness_advanced);

    rb_servo::FtRawSample repeated_acquisition = same_value;
    repeated_acquisition.host_time_ns = 1'009'000'000ULL;
    const auto repeated_out = pipeline.process(
        repeated_acquisition,
        rb_servo::Pose6D{},
        repeated_acquisition.host_time_ns
    );
    RB_CHECK(repeated_out.healthy);
    RB_CHECK(!repeated_out.freshness_advanced);

    rb_servo::FtRawSample held = sample(11, 1'020'000'000ULL);
    held.wrench_sensor.fz = 4.0;
    const auto held_out = pipeline.process(held, rb_servo::Pose6D{}, held.host_time_ns);
    RB_CHECK(!held_out.healthy);
    RB_CHECK(held_out.stale);

    rb_servo::FtRawSample regressed = sample(9, 1'021'000'000ULL);
    const auto regressed_out = pipeline.process(
        regressed,
        rb_servo::Pose6D{},
        regressed.host_time_ns
    );
    RB_CHECK(!regressed_out.healthy);
    RB_CHECK(regressed_out.stale);
    return true;
}

bool testSourceTimestampFreshnessAlternative() {
    rb_servo::FtWrenchPipelineConfig config = baseConfig();
    config.freshness_source = "source_time";
    rb_servo::FtWrenchPipeline pipeline(config);

    rb_servo::FtRawSample first = sample(0, 1'000'000'000ULL);
    first.source_sequence_valid = false;
    first.source_time_valid = true;
    first.source_time_ns = 100;
    RB_CHECK(pipeline.process(first, rb_servo::Pose6D{}, first.host_time_ns).healthy);

    rb_servo::FtRawSample next = first;
    next.host_time_ns += 5'000'000ULL;
    next.source_time_ns = 200;
    RB_CHECK(pipeline.process(next, rb_servo::Pose6D{}, next.host_time_ns).healthy);

    rb_servo::FtRawSample held = next;
    held.host_time_ns += 20'000'000ULL;
    const auto held_out = pipeline.process(held, rb_servo::Pose6D{}, held.host_time_ns);
    RB_CHECK(!held_out.healthy);
    RB_CHECK(held_out.stale);
    return true;
}

bool testHealthFailuresAreFailClosed() {
    rb_servo::FtWrenchPipelineConfig config = baseConfig();
    rb_servo::FtWrenchPipeline pipeline(config);
    rb_servo::FtRawSample raw = sample(1, 1'000'000'000ULL);

    raw.sensor_present = false;
    RB_CHECK(!pipeline.process(raw, rb_servo::Pose6D{}, raw.host_time_ns).healthy);
    pipeline.reset();

    raw.sensor_present = true;
    raw.source_sequence_valid = false;
    RB_CHECK(!pipeline.process(raw, rb_servo::Pose6D{}, raw.host_time_ns).healthy);
    pipeline.reset();

    raw.source_sequence_valid = true;
    raw.wrench_sensor.fx = std::numeric_limits<double>::quiet_NaN();
    RB_CHECK(!pipeline.process(raw, rb_servo::Pose6D{}, raw.host_time_ns).healthy);
    pipeline.reset();

    raw.wrench_sensor.fx = 0.0;
    RB_CHECK(!pipeline.process(raw, rb_servo::Pose6D{}, raw.host_time_ns + 200'000'000ULL).healthy);
    return true;
}

bool testResidualTareEligibilityAndFilterReset() {
    rb_servo::FtWrenchPipelineConfig config = baseConfig();
    config.residual_tare_min_samples = 3;
    config.residual_tare_max_force_stddev_n = 0.01;
    rb_servo::FtWrenchPipeline pipeline(config);

    pipeline.beginResidualTare();
    rb_servo::FtRawSample raw = sample(1, 1'000'000'000ULL);
    raw.wrench_sensor.fx = 2.0;
    auto out = pipeline.process(raw, rb_servo::Pose6D{}, raw.host_time_ns);
    const auto first_tare_sample = pipeline.updateResidualTare(out, true, true);
    RB_CHECK(first_tare_sample.state == rb_servo::FtTareState::Collecting);
    RB_CHECK(first_tare_sample.sample_count == 1);
    const auto duplicate_tare_sample = pipeline.updateResidualTare(out, true, true);
    RB_CHECK(duplicate_tare_sample.state == rb_servo::FtTareState::Collecting);
    RB_CHECK(duplicate_tare_sample.sample_count == 1);
    raw.source_sequence = 2;
    raw.host_time_ns += 1'000'000ULL;
    out = pipeline.process(raw, rb_servo::Pose6D{}, raw.host_time_ns);
    RB_CHECK(
        pipeline.updateResidualTare(out, true, true).state ==
        rb_servo::FtTareState::Collecting
    );
    raw.source_sequence = 3;
    raw.host_time_ns += 1'000'000ULL;
    out = pipeline.process(raw, rb_servo::Pose6D{}, raw.host_time_ns);
    const auto accepted = pipeline.updateResidualTare(out, true, true);
    RB_CHECK(accepted.state == rb_servo::FtTareState::Accepted);
    RB_CHECK(accepted.sample_count == 3);

    raw.source_sequence = 4;
    raw.host_time_ns += 1'000'000ULL;
    out = pipeline.process(raw, rb_servo::Pose6D{}, raw.host_time_ns);
    RB_CHECK(out.healthy);
    RB_CHECK(near(out.fast_external_wrench_tcp.fx, 0.0));
    RB_CHECK(near(out.control_external_wrench_tcp.fx, 0.0));

    pipeline.beginResidualTare();
    RB_CHECK(
        pipeline.updateResidualTare(out, false, true).state ==
        rb_servo::FtTareState::Rejected
    );

    rb_servo::FtWrenchPipeline noisy_pipeline(config);
    noisy_pipeline.beginResidualTare();
    for (int i = 0; i < 3; ++i) {
        rb_servo::FtRawSample noisy = sample(
            static_cast<uint64_t>(i + 1),
            2'000'000'000ULL + static_cast<uint64_t>(i) * 1'000'000ULL
        );
        noisy.wrench_sensor.fx = i == 1 ? 1.0 : 0.0;
        const auto noisy_out = noisy_pipeline.process(
            noisy,
            rb_servo::Pose6D{},
            noisy.host_time_ns
        );
        const auto update = noisy_pipeline.updateResidualTare(noisy_out, true, true);
        if (i < 2) {
            RB_CHECK(update.state == rb_servo::FtTareState::Collecting);
        } else {
            RB_CHECK(update.state == rb_servo::FtTareState::Rejected);
        }
    }
    return true;
}

}  // namespace

int main() {
    if (!testBiasThenTransformWithMomentArm()) return 1;
    if (!testAcceptedRbpodoEftForceAxisMapping()) return 1;
    if (!testRotationAndPayloadCompensation()) return 1;
    if (!testResidualTareAndLowPass()) return 1;
    if (!testLowPassAdvancesOnlyOnNewAcquisition()) return 1;
    if (!testFreshnessUsesSourceSequenceNotValueChanges()) return 1;
    if (!testSourceTimestampFreshnessAlternative()) return 1;
    if (!testHealthFailuresAreFailClosed()) return 1;
    if (!testResidualTareEligibilityAndFilterReset()) return 1;
    std::cout << "ft_wrench_pipeline tests passed\n";
    return 0;
}
