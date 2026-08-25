#pragma once

#include <cstdint>
#include <string>

#include "rb_servo/config/config.hpp"

namespace rb_servo {

// Locks one arm's servo_j send cadence to its Rainbow control box.
//
// PLANT. Firmware v8.7.3 schedules streamed commands from a FIFO, so the
// box-side dead time IS the queue fill: measured 2026-08-25 on RB3-730E,
// dead time = RBACK fill + 1 tick, exactly, on both arms. The queue is a pure
// integrator, dfill/dt = f_send - f_box, and the clocks do not match --
// measured f_box = 499.34 Hz against a 500.00 Hz send, so an unregulated stream
// grows the queue by +0.67 ticks/s (+1.33 ms/s of latency) without bound.
//
// Regulating the fill to a small constant is what turns "always 5 ticks" from a
// firmware claim into a property of the system. It is also a PHASE lock, not
// just a rate lock: the integral converges to the per-cycle period extension
// that matches the box's consumption rate, so each send lands at a fixed phase
// inside the box's tick instead of wandering across it. Rate-only regulation
// (e.g. dropping a send every N ticks) holds the average fill but leaves that
// sub-tick phase free, which shows up as +/-1 tick of dead-time jitter.
//
// ACTUATOR. A trim on this arm's send PERIOD, in microseconds, applied to the
// nominal control period. Identical to controller-manager's Arm::qsync_step
// (sleep_adjust_ns_), so its gains transfer as-is. This requires the arm to own
// its own cadence -- a single loop driving both arms has one input for two
// boxes and can only regulate one of them. The two boxes measured +0.665 and
// +0.670 ticks/s, a difference that accumulates ~18 ticks/hour between arms, so
// per-arm actuation is required for a long-running session, not a refinement.
//
// SIGN. Positive trim = longer period = slower sends = fill falls.
struct QueueSyncDecision {
    double period_trim_us = 0.0;       // add to the nominal control period
    double fill_lpf = 0.0;
    double integral_us = 0.0;
    int last_fill = -1;                // -1 = no RBACK observed yet
    bool fill_valid = false;
    std::string phase = "idle";        // idle | warmup | drain | track
    bool locked = false;               // Track phase and fill within tolerance of target
    uint64_t underrun_events = 0;      // fill fell to <= protect_fill
    uint64_t stall_events = 0;         // no fresh RBACK for stall_cycles
    uint64_t highwater_events = 0;     // absurd backlog; box likely stopped consuming
    uint64_t redrain_events = 0;       // queue rebase forced a re-drain
    uint64_t no_consumption_events = 0;// fill rising faster than a trim can correct
};

class QueueSyncController {
public:
    struct Observation {
        bool streaming = false;        // this arm is actually streaming servo_j this tick
        bool fill_valid = false;       // an RBACK has been parsed at least once
        int fill = -1;
        uint64_t rback_sequence = 0;   // increments per parsed RBACK (freshness)
        uint64_t now_ns = 0;
    };

    explicit QueueSyncController(QueueSyncConfig config);

    // One control cycle. A pure function of (config, internal state,
    // observation): no clock reads, no I/O, no allocation on the steady path,
    // so the whole law is testable hardware-free.
    QueueSyncDecision step(const Observation& observation);

    // Drop regulation state. Call when the stream stops or the backend
    // reconnects: a pre-gap fill observation says nothing about the new queue.
    void reset();

    const QueueSyncConfig& config() const { return config_; }

private:
    enum class Phase { Idle, Warmup, Drain, Track };

    QueueSyncConfig config_;
    Phase phase_ = Phase::Idle;
    uint64_t phase_start_ns_ = 0;
    uint64_t last_rback_sequence_ = 0;
    int last_fill_ = -1;
    double fill_lpf_ = 0.0;
    double integral_us_ = 0.0;
    int stale_cycles_ = 0;
    bool underrun_active_ = false;
    bool highwater_active_ = false;
    // Consumption watch: compare against a ~1 s-old reference so a box that has
    // stopped consuming is reported instead of being trimmed at forever.
    uint64_t consumption_ref_ns_ = 0;
    int consumption_ref_fill_ = -1;
    QueueSyncDecision counters_;
};

}  // namespace rb_servo
