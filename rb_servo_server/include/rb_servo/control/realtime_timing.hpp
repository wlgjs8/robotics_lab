#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "rb_servo/core/types.hpp"

namespace rb_servo {

struct RealtimeFeedbackTimingTick {
    uint64_t host_time_ns = 0;
    uint64_t robot_time_ns = 0;
    bool explicit_cached_hold = false;
};

struct RealtimeTimingTick {
    uint64_t loop_start_ns = 0;
    uint64_t loop_end_ns = 0;
    uint64_t scheduled_wake_ns = 0;
    uint64_t previous_sleep_enter_ns = 0;
    uint64_t nominal_period_ns = 0;
    bool send_cycle = false;
    uint64_t pre_send_ns = 0;
    uint64_t send_duration_ns = 0;
    RealtimeFeedbackTimingTick left_feedback;
    RealtimeFeedbackTimingTick right_feedback;
};

// Allocation-free one-second rolling timing accumulator for the RT loop. The
// histograms use logarithmic integer bins; exported p95/max values are the
// upper bound of the selected bin (last values remain exact).
class RealtimeTimingAccumulator {
public:
    static constexpr uint64_t kWindowNs = 1'000'000'000ULL;
    static constexpr std::size_t kCapacity = 1024;

    void reset();
    void add(const RealtimeTimingTick& tick);
    RealtimeTimingTelemetry snapshot() const;

private:
    static constexpr std::size_t kHistogramBins = 128;

    struct Histogram {
        std::array<uint16_t, kHistogramBins> count{};
        uint32_t total = 0;

        static uint8_t binFor(uint64_t value_ns);
        static uint64_t upperBoundNs(uint8_t bin);
        void add(uint8_t bin);
        void remove(uint8_t bin);
        uint64_t percentileUpperNs(double quantile) const;
        uint64_t maxUpperNs() const;
    };

    struct ArmEntry {
        bool observed = false;
        bool frame = false;
        bool fresh = false;
        bool held = false;
        bool period_valid = false;
        uint8_t period_bin = 0;
        uint8_t jitter_bin = 0;
        uint8_t age_bin = 0;
        uint8_t phase_bin = 0;
    };

    struct Entry {
        uint64_t loop_start_ns = 0;
        bool deadline_miss = false;
        bool catch_up = false;
        bool send_cycle = false;
        uint8_t period_bin = 0;
        uint8_t jitter_bin = 0;
        uint8_t wake_bin = 0;
        uint8_t pre_send_bin = 0;
        uint8_t send_duration_bin = 0;
        ArmEntry left;
        ArmEntry right;
    };

    struct ArmAggregate {
        uint64_t frame_count = 0;
        uint64_t fresh_count = 0;
        uint64_t held_count = 0;
        Histogram period;
        Histogram jitter;
        Histogram age;
        Histogram phase;
        uint64_t last_period_ns = 0;
        uint64_t last_jitter_ns = 0;
        uint64_t last_age_ns = 0;
        uint64_t last_phase_ns = 0;
        uint64_t previous_host_time_ns = 0;
        uint64_t previous_robot_time_ns = 0;
        bool robot_time_available = false;
        bool robot_time_monotonic = true;
    };

    static uint64_t absoluteDifference(uint64_t lhs, uint64_t rhs);
    static uint64_t receiptPhaseNs(
        uint64_t host_time_ns,
        uint64_t scheduled_wake_ns,
        uint64_t nominal_period_ns
    );
    static ArmEntry makeArmEntry(
        const RealtimeFeedbackTimingTick& feedback,
        uint64_t loop_end_ns,
        uint64_t scheduled_wake_ns,
        uint64_t nominal_period_ns,
        ArmAggregate* aggregate
    );
    static void addArmEntry(const ArmEntry& entry, ArmAggregate* aggregate);
    static void removeArmEntry(const ArmEntry& entry, ArmAggregate* aggregate);
    static FeedbackRealtimeTimingTelemetry armSnapshot(
        const ArmAggregate& aggregate,
        double window_sec
    );
    void removeOldest();

    std::array<Entry, kCapacity> entries_{};
    std::size_t head_ = 0;
    std::size_t size_ = 0;
    uint64_t nominal_period_ns_ = 0;
    uint64_t deadline_miss_count_ = 0;
    uint64_t catch_up_count_ = 0;
    uint64_t send_cycle_count_ = 0;
    Histogram period_;
    Histogram jitter_;
    Histogram wake_;
    Histogram pre_send_;
    Histogram send_duration_;
    uint64_t last_period_ns_ = 0;
    uint64_t last_jitter_ns_ = 0;
    uint64_t last_wake_ns_ = 0;
    uint64_t last_pre_send_ns_ = 0;
    uint64_t last_send_duration_ns_ = 0;
    uint64_t previous_loop_start_ns_ = 0;
    ArmAggregate left_;
    ArmAggregate right_;
};

}  // namespace rb_servo
