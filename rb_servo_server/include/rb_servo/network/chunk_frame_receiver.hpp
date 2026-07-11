// chunk_frame_receiver.hpp — dedicated UDP ingest for whole action-chunk frames
// feeding the Ruckig chunk-follower.
//
// Wire format: the producer's chunk-overlay packet (chunk_overlay_publisher.py,
// schema "robotics_lab.chunk_overlay.v2") — measured-anchored absolute stand
// pose7+grip per step, plus optional conditioned local/body-frame delta rows:
//   { "schema_version": "robotics_lab.chunk_overlay.v2", "seq": N,
//     "policy_dt_sec": 0.033, "horizon": H, "host_time_ns": ...,
//     "left":  [[x,y,z,qx,qy,qz,qw,grip] * H] | null,
//     "right": [[...] * H] | null,
//     "left_delta":  [[dx,dy,dz,drx,dry,drz,grip] * H],   // optional
//     "right_delta": [[dx,dy,dz,drx,dry,drz,grip] * H] }  // optional
// The producer fans the same packet out to the GUI overlay port and to this
// bind; DELIBERATELY separate from the lease-gated command socket so the 500 Hz
// command stream can never starve the ~1-2 Hz chunk feed (see
// external-box-feed-starvation precedent).
//
// Threading: a network thread parses JSON into a FIXED-SIZE POD frame and
// stores it under a mutex; the 500 Hz control thread polls latestSeq() (atomic,
// no lock) and only on a seq change try_lock-copies the POD out. No allocation
// on the control thread; a missed try_lock (writer mid-store, ~µs each ~500 ms)
// just retries next tick.

#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace rb_servo {

class ChunkFrameReceiver {
public:
    static constexpr int kMaxSteps = 64;
    static constexpr int kPoseStepDims = 8;   // x y z qx qy qz qw grip
    static constexpr int kDeltaStepDims = 7;  // dx dy dz drx dry drz grip
    static constexpr int kStepDims = kPoseStepDims;  // compatibility alias

    struct ArmSteps {
        int count = 0;
        std::array<std::array<double, kPoseStepDims>, kMaxSteps> step{};
    };
    struct ArmDeltaSteps {
        int count = 0;
        std::array<std::array<double, kDeltaStepDims>, kMaxSteps> step{};
    };
    // Optional producer-side diagnostics carried by chunk_overlay.v2. These are
    // telemetry only: malformed or absent values remain zero and must never
    // invalidate an otherwise valid motion frame.
    struct Diagnostics {
        int execute_steps = 0;
        int runway_steps = 0;
        std::uint64_t inference_seq = 0;
        double inference_queue_wait_ms = 0.0;
        double inference_latency_ms = 0.0;
        double inference_ready_wait_ms = 0.0;
        double inference_period_ms = 0.0;
        double inference_period_jitter_ms = 0.0;
        std::uint64_t inference_stall_count = 0;
        std::uint64_t camera_bundle_seq = 0;
        double camera_bundle_age_ms = 0.0;
        double camera_max_skew_ms = 0.0;
        std::uint64_t camera_left_frame_number = 0;
        std::uint64_t camera_right_frame_number = 0;
        double camera_left_frame_age_ms = 0.0;
        double camera_right_frame_age_ms = 0.0;
        double camera_left_focus_score = 0.0;
        double camera_right_focus_score = 0.0;
    };
    struct Frame {
        std::uint64_t seq = 0;           // producer seq (may reset on restart)
        std::uint64_t receiver_seq = 0;  // receiver-monotonic; use for dedup
        double policy_dt_sec = 0.0;
        double recv_steady_sec = 0.0;  // steadyNowSec() at parse time
        double interarrival_sec = 0.0; // receiver-local accepted-frame interval
        bool has_left = false;
        bool has_right = false;
        ArmSteps left;
        ArmSteps right;
        bool has_left_delta = false;
        bool has_right_delta = false;
        ArmDeltaSteps left_delta;
        ArmDeltaSteps right_delta;
        Diagnostics diagnostics;
    };

    explicit ChunkFrameReceiver(const std::string& bind_uri);
    ~ChunkFrameReceiver();

    ChunkFrameReceiver(const ChunkFrameReceiver&) = delete;
    ChunkFrameReceiver& operator=(const ChunkFrameReceiver&) = delete;

    bool start();
    void stop();

    // Monotonically bumped on every accepted frame (independent of the producer
    // seq, which may reset when the producer restarts). 0 = none yet.
    std::uint64_t latestSeq() const { return latest_seq_.load(std::memory_order_acquire); }

    // Copy the latest frame out (control thread). Returns false when no frame
    // has arrived yet or the writer holds the lock right now (retry next tick).
    bool copyLatest(Frame* out) const;

    static double steadyNowSec();

    // Exposed for tests: parse one datagram payload into a frame.
    static bool parsePacket(const char* data, std::size_t size, Frame* out);

private:
    void threadMain();

    std::string bind_uri_;
    int socket_fd_ = -1;
    std::thread thread_;
    std::atomic<bool> running_{false};

    mutable std::mutex mutex_;
    Frame latest_;
    std::atomic<std::uint64_t> latest_seq_{0};
};

}  // namespace rb_servo
