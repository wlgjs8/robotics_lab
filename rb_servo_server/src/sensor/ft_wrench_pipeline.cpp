#include "rb_servo/sensor/ft_wrench_pipeline.hpp"

#include "rb_servo/math/se3.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <limits>
#include <utility>

#include <Eigen/Core>
#include <pinocchio/spatial/force.hpp>

namespace rb_servo {
namespace {

constexpr double kGravityMps2 = 9.80665;

std::array<double, 6> toArray(const Wrench6D& value) {
    return {value.fx, value.fy, value.fz, value.tx, value.ty, value.tz};
}

Wrench6D fromArray(const std::array<double, 6>& value) {
    return {value[0], value[1], value[2], value[3], value[4], value[5]};
}

bool finite(const Wrench6D& value) {
    const auto values = toArray(value);
    return std::all_of(values.begin(), values.end(), [](double item) {
        return std::isfinite(item);
    });
}

Wrench6D subtract(const Wrench6D& lhs, const Wrench6D& rhs) {
    const auto a = toArray(lhs);
    const auto b = toArray(rhs);
    std::array<double, 6> out{};
    for (std::size_t i = 0; i < out.size(); ++i) out[i] = a[i] - b[i];
    return fromArray(out);
}

Wrench6D transformWrench(const Pose6D& t_target_source, const Wrench6D& source) {
    pinocchio::Force source_force;
    source_force.linear() = Eigen::Vector3d(source.fx, source.fy, source.fz);
    source_force.angular() = Eigen::Vector3d(source.tx, source.ty, source.tz);
    const pinocchio::Force target_force = math::se3FromPose(t_target_source).act(source_force);
    return {
        target_force.linear().x(),
        target_force.linear().y(),
        target_force.linear().z(),
        target_force.angular().x(),
        target_force.angular().y(),
        target_force.angular().z(),
    };
}

Eigen::Vector3d gravityTcp(const Pose6D& tcp_pose_stand) {
    const Eigen::Vector3d gravity_stand(0.0, 0.0, -kGravityMps2);
    return math::rotationFromPose(tcp_pose_stand).transpose() * gravity_stand;
}

Wrench6D payloadWrenchTcp(const FtWrenchPipelineConfig& config, const Eigen::Vector3d& gravity_tcp) {
    const Eigen::Vector3d force_tcp = config.payload_mass_kg * gravity_tcp;
    const Eigen::Vector3d com(
        config.payload_com_tcp_m[0],
        config.payload_com_tcp_m[1],
        config.payload_com_tcp_m[2]
    );
    const Eigen::Vector3d torque_tcp = com.cross(force_tcp);
    return {
        force_tcp.x(),
        force_tcp.y(),
        force_tcp.z(),
        torque_tcp.x(),
        torque_tcp.y(),
        torque_tcp.z(),
    };
}

Eigen::Vector3d applyRowMajorMatrix(
    const std::array<double, 9>& matrix,
    const Eigen::Vector3d& vector
) {
    Eigen::Matrix3d eigen_matrix;
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            eigen_matrix(row, column) =
                matrix[static_cast<std::size_t>(3 * row + column)];
        }
    }
    return eigen_matrix * vector;
}

Wrench6D modeledGravityWrenchTcp(
    const FtWrenchPipelineConfig& config,
    const Eigen::Vector3d& gravity_tcp
) {
    if (config.gravity_compensation_model == "rigid_payload") {
        return payloadWrenchTcp(config, gravity_tcp);
    }
    if (config.gravity_compensation_model == "controller_compensated_linear") {
        const Eigen::Vector3d force = applyRowMajorMatrix(
            config.gravity_force_matrix_n_per_m_s2,
            gravity_tcp
        );
        const Eigen::Vector3d torque = applyRowMajorMatrix(
            config.gravity_torque_matrix_nm_per_m_s2,
            gravity_tcp
        );
        return {
            force.x(), force.y(), force.z(),
            torque.x(), torque.y(), torque.z(),
        };
    }
    // process() rejects unsupported values before this helper is reached.
    return Wrench6D{};
}

Wrench6D lowPass(const Wrench6D& previous, const Wrench6D& current, double alpha) {
    const auto old_values = toArray(previous);
    const auto new_values = toArray(current);
    std::array<double, 6> out{};
    for (std::size_t i = 0; i < out.size(); ++i) {
        out[i] = (1.0 - alpha) * old_values[i] + alpha * new_values[i];
    }
    return fromArray(out);
}

std::string lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char item) {
        return static_cast<char>(std::tolower(item));
    });
    return value;
}

}  // namespace

FtWrenchPipeline::FtWrenchPipeline(FtWrenchPipelineConfig config) : config_(std::move(config)) {}

void FtWrenchPipeline::setTcpLinearAcceleration(const math::Vector3& a_tcp_mps2) {
    tcp_linear_acceleration_m_s2_ = a_tcp_mps2;
    tcp_linear_acceleration_supplied_ = true;
}

void FtWrenchPipeline::reset() {
    filtered_external_ = Wrench6D{};
    filter_initialized_ = false;
    source_initialized_ = false;
    last_source_value_ = 0;
    last_source_advance_ns_ = 0;
    tcp_linear_acceleration_m_s2_.setZero();
    filtered_tcp_linear_acceleration_m_s2_.setZero();
    tcp_linear_acceleration_supplied_ = false;
    inertial_accel_filter_initialized_ = false;
    cancelResidualTare();
}

void FtWrenchPipeline::beginResidualTare() {
    tare_collecting_ = true;
    tare_sample_count_ = 0;
    tare_sum_.fill(0.0);
    tare_sum_squares_.fill(0.0);
    tare_source_initialized_ = false;
    last_tare_freshness_value_ = 0;
}

void FtWrenchPipeline::cancelResidualTare() {
    tare_collecting_ = false;
    tare_sample_count_ = 0;
    tare_sum_.fill(0.0);
    tare_sum_squares_.fill(0.0);
    tare_source_initialized_ = false;
    last_tare_freshness_value_ = 0;
}

Wrench6D FtWrenchPipeline::residualTareTcp() const {
    return config_.residual_tare_tcp;
}

FtTareUpdate FtWrenchPipeline::updateResidualTare(
    const FtWrenchPipelineOutput& output,
    bool robot_stationary,
    bool contact_clear
) {
    FtTareUpdate result;
    result.sample_count = tare_sample_count_;
    const auto reject = [&](const std::string& message) {
        cancelResidualTare();
        result.state = FtTareState::Rejected;
        result.sample_count = 0;
        result.reason = message;
        return result;
    };

    if (!tare_collecting_) {
        result.reason = "tare collection is not active";
        return result;
    }
    if (!output.healthy) return reject("tare sample is unhealthy");
    if (!robot_stationary) return reject("robot is not stationary");
    if (!contact_clear) return reject("contact is not clear");
    if (!finite(output.pre_tare_external_wrench_tcp)) {
        return reject("tare sample is non-finite");
    }
    if (tare_source_initialized_) {
        if (output.freshness_value < last_tare_freshness_value_) {
            return reject("tare freshness source regressed");
        }
        if (output.freshness_value == last_tare_freshness_value_) {
            result.state = FtTareState::Collecting;
            result.reason = "waiting for a new acquisition";
            return result;
        }
    }
    tare_source_initialized_ = true;
    last_tare_freshness_value_ = output.freshness_value;

    const auto values = toArray(output.pre_tare_external_wrench_tcp);
    for (std::size_t i = 0; i < values.size(); ++i) {
        tare_sum_[i] += values[i];
        tare_sum_squares_[i] += values[i] * values[i];
    }
    ++tare_sample_count_;
    result.sample_count = tare_sample_count_;
    if (tare_sample_count_ < config_.residual_tare_min_samples) {
        result.state = FtTareState::Collecting;
        result.reason = "collecting";
        return result;
    }

    std::array<double, 6> means{};
    for (std::size_t i = 0; i < means.size(); ++i) {
        means[i] = tare_sum_[i] / static_cast<double>(tare_sample_count_);
        const double second_moment =
            tare_sum_squares_[i] / static_cast<double>(tare_sample_count_);
        const double variance = std::max(0.0, second_moment - means[i] * means[i]);
        const double stddev = std::sqrt(variance);
        const double limit = i < 3
            ? config_.residual_tare_max_force_stddev_n
            : config_.residual_tare_max_torque_stddev_nm;
        if (stddev > limit) {
            static constexpr const char* kAxisNames[6] = {
                "fx", "fy", "fz", "tx", "ty", "tz"};
            char detail[128];
            std::snprintf(
                detail, sizeof(detail),
                "tare window variance exceeds limit (%s stddev %.3f > %.3f %s)",
                kAxisNames[i], stddev, limit, i < 3 ? "N" : "Nm");
            return reject(detail);
        }
    }

    config_.residual_tare_tcp = fromArray(means);
    filtered_external_ = Wrench6D{};
    filter_initialized_ = false;
    const int accepted_samples = tare_sample_count_;
    cancelResidualTare();
    result.state = FtTareState::Accepted;
    result.sample_count = accepted_samples;
    result.reason = "accepted";
    return result;
}

FtWrenchPipelineOutput FtWrenchPipeline::process(
    const FtRawSample& sample,
    const Pose6D& tcp_pose_stand,
    uint64_t now_ns
) {
    FtWrenchPipelineOutput out;
    const Eigen::Vector3d gravity_tcp = gravityTcp(tcp_pose_stand);
    out.gravity_tcp = {gravity_tcp.x(), gravity_tcp.y(), gravity_tcp.z()};
    const auto reject = [&](const std::string& reason, bool stale = false) {
        out.healthy = false;
        out.stale = stale;
        out.reason = reason;
        return out;
    };

    if (!config_.enable) return reject("disabled");
    if (!config_.frame_configured) return reject("T_tcp_sensor is not configured");
    if (config_.inertial_compensation_enable &&
        (!tcp_linear_acceleration_supplied_ ||
         !tcp_linear_acceleration_m_s2_.allFinite())) {
        return reject("commanded TCP acceleration is unavailable or non-finite");
    }
    if (!(config_.gravity_compensation_model == "rigid_payload" ||
          config_.gravity_compensation_model == "controller_compensated_linear")) {
        return reject("unsupported gravity compensation model");
    }
    if (config_.gravity_compensation_model == "controller_compensated_linear" &&
        (!config_.gravity_force_matrix_configured ||
         !config_.gravity_torque_matrix_configured ||
         config_.gravity_compensation_calibration_id.empty() ||
         config_.payload_mass_kg != 0.0 ||
         std::any_of(
             config_.payload_com_tcp_m.begin(),
             config_.payload_com_tcp_m.end(),
             [](double item) { return item != 0.0; }
         ))) {
        return reject("controller-compensated gravity model is incomplete or mixed");
    }
    if (!sample.fields_present) return reject("wrench fields missing");
    if (!sample.sensor_present) return reject("sensor presence not confirmed");
    if (sample.source_fault) return reject("sensor source fault");
    if (sample.overrange) return reject("sensor overrange");
    if (!finite(sample.wrench_sensor)) return reject("non-finite sensor wrench");
    if (sample.host_time_ns == 0 || now_ns < sample.host_time_ns) {
        return reject("invalid sample timestamp", true);
    }

    const double sample_age_sec = static_cast<double>(now_ns - sample.host_time_ns) * 1e-9;
    if (sample_age_sec > config_.max_sample_age_sec) return reject("sample is stale", true);

    const std::string freshness_source = lower(config_.freshness_source);
    uint64_t source_value = 0;
    if (freshness_source == "sequence") {
        if (!sample.source_sequence_valid) return reject("source sequence unavailable", true);
        source_value = sample.source_sequence;
    } else if (freshness_source == "source_time") {
        if (!sample.source_time_valid) return reject("source timestamp unavailable", true);
        source_value = sample.source_time_ns;
    } else {
        return reject("unsupported freshness source", true);
    }

    bool freshness_advanced = false;
    {
        if (!source_initialized_) {
            source_initialized_ = true;
            last_source_value_ = source_value;
            last_source_advance_ns_ = now_ns;
            freshness_advanced = true;
        } else if (source_value > last_source_value_) {
            last_source_value_ = source_value;
            last_source_advance_ns_ = now_ns;
            freshness_advanced = true;
        } else if (source_value < last_source_value_) {
            return reject("freshness source regressed", true);
        } else {
            const double stall_sec = static_cast<double>(now_ns - last_source_advance_ns_) * 1e-9;
            if (stall_sec > config_.max_source_stall_sec) {
                return reject("freshness source stalled", true);
            }
        }
    }

    const Wrench6D unbiased_sensor = subtract(sample.wrench_sensor, config_.sensor_bias);
    out.wrench_tcp = transformWrench(config_.t_tcp_sensor, unbiased_sensor);
    out.payload_wrench_tcp = payloadWrenchTcp(config_, gravity_tcp);
    out.modeled_gravity_wrench_tcp = modeledGravityWrenchTcp(config_, gravity_tcp);
    Wrench6D gravity_removed_external = subtract(
        out.wrench_tcp,
        out.modeled_gravity_wrench_tcp
    );
    if (config_.inertial_compensation_enable) {
        if (!inertial_accel_filter_initialized_) {
            filtered_tcp_linear_acceleration_m_s2_ = tcp_linear_acceleration_m_s2_;
            inertial_accel_filter_initialized_ = true;
        } else if (freshness_advanced) {
            const double alpha = config_.inertial_accel_lpf_alpha;
            filtered_tcp_linear_acceleration_m_s2_ =
                (1.0 - alpha) * filtered_tcp_linear_acceleration_m_s2_ +
                alpha * tcp_linear_acceleration_m_s2_;
        }
        const math::Vector3 inertial_force_tcp =
            config_.inertial_effective_mass_kg *
            filtered_tcp_linear_acceleration_m_s2_;
        // V1 compensates translation only. Rotational inertia and the torque
        // from an offset CoM stay out of scope until a CoM/inertia model has
        // been identified and reviewed on both physical arms.
        const Wrench6D inertial_wrench_tcp{
            inertial_force_tcp.x(), inertial_force_tcp.y(), inertial_force_tcp.z(),
            0.0, 0.0, 0.0,
        };
        gravity_removed_external = subtract(
            gravity_removed_external,
            inertial_wrench_tcp
        );
    }
    out.pre_tare_external_wrench_tcp = gravity_removed_external;
    out.fast_external_wrench_tcp = subtract(out.pre_tare_external_wrench_tcp, config_.residual_tare_tcp);

    if (!finite(out.fast_external_wrench_tcp)) return reject("non-finite compensated wrench");

    const double alpha = std::clamp(config_.control_lpf_alpha, 0.0, 1.0);
    if (!filter_initialized_) {
        filtered_external_ = out.fast_external_wrench_tcp;
        filter_initialized_ = true;
    } else if (freshness_advanced) {
        filtered_external_ = lowPass(filtered_external_, out.fast_external_wrench_tcp, alpha);
    }
    out.control_external_wrench_tcp = filtered_external_;
    out.healthy = true;
    out.freshness_value = source_value;
    out.freshness_advanced = freshness_advanced;
    out.reason = "ok";
    return out;
}

}  // namespace rb_servo
