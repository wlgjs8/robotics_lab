#pragma once

#include "rb_servo/config/config.hpp"
#include "rb_servo/core/types.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace rb_servo {

// Backend-independent raw sensor sample. A production adapter must provide a
// true acquisition sequence (or equivalent source timestamp); poll time alone
// is not sufficient to prove freshness.
struct FtRawSample {
    Wrench6D wrench_sensor;
    uint64_t host_time_ns = 0;
    uint64_t source_sequence = 0;
    uint64_t source_time_ns = 0;
    bool source_sequence_valid = false;
    bool source_time_valid = false;
    bool fields_present = false;
    bool sensor_present = false;
    bool source_fault = false;
    bool overrange = false;
};

struct FtWrenchPipelineOutput {
    Wrench6D wrench_tcp;
    Wrench6D payload_wrench_tcp;
    Wrench6D pre_tare_external_wrench_tcp;
    Wrench6D fast_external_wrench_tcp;
    Wrench6D control_external_wrench_tcp;
    bool healthy = false;
    bool stale = false;
    uint64_t freshness_value = 0;
    bool freshness_advanced = false;
    std::string reason;
};

enum class FtTareState { Idle, Collecting, Accepted, Rejected };

struct FtTareUpdate {
    FtTareState state = FtTareState::Idle;
    int sample_count = 0;
    std::string reason;
};

class FtWrenchPipeline {
public:
    explicit FtWrenchPipeline(FtWrenchPipelineConfig config);

    FtWrenchPipelineOutput process(
        const FtRawSample& sample,
        const Pose6D& tcp_pose_stand,
        uint64_t now_ns
    );
    void beginResidualTare();
    FtTareUpdate updateResidualTare(
        const FtWrenchPipelineOutput& output,
        bool robot_stationary,
        bool contact_clear
    );
    void cancelResidualTare();
    Wrench6D residualTareTcp() const;
    void reset();

private:
    FtWrenchPipelineConfig config_;
    Wrench6D filtered_external_{};
    bool filter_initialized_ = false;
    bool source_initialized_ = false;
    uint64_t last_source_value_ = 0;
    uint64_t last_source_advance_ns_ = 0;
    bool tare_collecting_ = false;
    int tare_sample_count_ = 0;
    std::array<double, 6> tare_sum_{};
    std::array<double, 6> tare_sum_squares_{};
    bool tare_source_initialized_ = false;
    uint64_t last_tare_freshness_value_ = 0;
};

}  // namespace rb_servo
