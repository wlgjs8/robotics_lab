#pragma once

#include <cstdint>
#include <string>

#include "rb_servo/config/config.hpp"

namespace rb_servo {

// Holds the Rainbow control box's command-queue occupancy at a setpoint.
//
// Firmware v8.7.3 schedules streamed servo_j commands from a FIFO, so the
// box-side dead time IS the queue fill (measured 2026-08-25: dead time =
// RBACK fill + 1 tick, exactly, on both arms). The queue is a pure integrator,
// dfill/dt = f_send - f_box, and the two clocks do not match: measured
// f_box = 499.34 Hz against our 500.00 Hz, so an unregulated stream grows the
// queue by +0.67 ticks/s (+1.33 ms/s of latency) without bound. Holding the
// fill at a small constant is what turns "always 5 ticks" from a claim into a
// property.
//
// ACTUATOR. controller-manager regulates this by trimming the send PERIOD
// (Arm::qsync_step -> sleep_adjust_ns_), which it can do because it runs one
// thread per arm. This server drives BOTH arms from a single 500 Hz loop, so a
// per-arm period trim is not available: slowing the loop for one arm would slow
// the other arm, the state reads, and every downstream timing assumption.
//
// Instead the actuator is a per-arm SEND SKIP, distributed by a fractional
// accumulator (a Bresenham/DDA rate divider). Skipping one send in N ticks runs
// that arm's stream at 500*(1 - 1/N) Hz with the loop period untouched. This is
// not a lossy approximation of the period trim -- the box consumes at 499.34 Hz
// either way, so the rate mismatch must be absorbed somewhere; a skip absorbs it
// by dropping one setpoint locally instead of stretching time for everything.
//
// The gains below are controller-manager's, converted through the two
// actuators' plant gains rather than re-tuned from scratch:
//   CM:   +1 us of period trim  ~= -0.25 fill/s
//   here: skip fraction f       ~= -500*f fill/s
//   => f = adj_us / 2000
// so CM's 6 us/fill becomes 0.003 skip/fill, and so on. Any change to one side
// should be re-derived, not guessed.
//
// ONE-SIDED. A skip can only make the stream SLOWER. If a box ever drains
// faster than the loop sends, the fill falls to zero and this controller cannot
// correct it -- it can only stop skipping. That is safe (an underrun starves the
// box, it does not overrun it) but it is not regulation, so underruns are
// counted and surfaced rather than silently absorbed.
struct QueueSyncDecision {
    bool send = true;                  // false = skip this arm's servo_j this tick
    double skip_fraction = 0.0;        // commanded skip rate in [0, skip_fraction_max]
    double fill_lpf = 0.0;
    double integral = 0.0;
    int last_fill = -1;                // -1 = no RBACK observed yet
    bool fill_valid = false;
    std::string phase = "idle";        // idle | warmup | drain | track
    uint64_t skipped_total = 0;
    uint64_t sent_total = 0;
    uint64_t underrun_events = 0;      // fill fell to <= protect_fill
    uint64_t stall_events = 0;         // no fresh RBACK for stall_cycles
    uint64_t highwater_events = 0;     // absurd backlog; box likely stopped consuming
    uint64_t redrain_events = 0;       // queue rebase forced a re-drain
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

    // One control cycle. Pure function of (config, internal state, observation):
    // no clocks, no I/O, no allocation on the steady path -- so the whole law is
    // testable hardware-free.
    QueueSyncDecision step(const Observation& observation);

    void reset();

private:
    enum class Phase { Idle, Warmup, Drain, Track };

    QueueSyncConfig config_;
    Phase phase_ = Phase::Idle;
    uint64_t phase_start_ns_ = 0;
    uint64_t last_rback_sequence_ = 0;
    int last_fill_ = -1;
    double fill_lpf_ = 0.0;
    double integral_ = 0.0;
    double credit_ = 0.0;              // DDA accumulator for the skip distribution
    int stale_cycles_ = 0;
    bool underrun_active_ = false;
    bool highwater_active_ = false;
    QueueSyncDecision counters_;
};

}  // namespace rb_servo
