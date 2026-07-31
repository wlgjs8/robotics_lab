#pragma once

#include <atomic>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string>
#include <thread>

#include <netinet/in.h>

#include "rb_servo/config/config.hpp"
#include "rb_servo/core/types.hpp"

namespace rb_servo {

struct GripperArmFeedback {
    bool valid = false;        // false => no fresh feedback (stale / never received)
    double percent = 0.0;      // live opening %, 0 closed .. 100 open
    double target_percent = 0.0;
    bool moving = false;
    bool ok = false;
    std::string fault;         // empty => none
    // Age of the gripper_state.v1 message this block came from:
    // (bridge receive time) - (gripper_server publish stamp). The two processes
    // stamp with DIFFERENT clocks -- gripper_server uses Python time.time_ns()
    // (CLOCK_REALTIME) while the rest of this server runs on steady_clock -- so
    // the receive side must use system_clock for this one comparison or the
    // number is meaningless. NaN when the payload carried no usable host_time_ns.
    //
    // Why it exists: the measured gripper close->feedback latency was 104-139 ms
    // onset and 278-347 ms to settle (2026-07-31 rollouts), more than the command
    // pipeline can account for (bridge 50 Hz + server 50 Hz + backend 60 Hz cap
    // ~= 45-95 ms round trip), but there was NO telemetry separating "the jaw is
    // slow" from "the feedback is reported late". Without that split, compensating
    // by shifting the gripper channel forward is guesswork: over-shifting closes
    // the jaw before contact and pushes the object away.
    double feedback_age_ms = std::numeric_limits<double>::quiet_NaN();
};

// Bridge to the out-of-process gripper_server (docs/plans/gripper_server_design.md).
// Forwards arbitrated per-arm gripper setpoints (gripper_cmd.v1) and caches
// gripper_state.v1 feedback. ALL I/O runs OFF the 500 Hz control loop: forward()
// is driven by the StatePublisher thread and feedback is read on an internal
// receive thread. Nothing here blocks the servo loop.
class GripperBridge {
public:
    explicit GripperBridge(const GripperConfig& config);
    ~GripperBridge();

    GripperBridge(const GripperBridge&) = delete;
    GripperBridge& operator=(const GripperBridge&) = delete;

    bool start();   // resolve/open sockets, start the receive thread
    void stop();
    bool enabled() const { return config_.enable; }

    // Forward the arbitrated per-arm gripper setpoint (only arms whose command
    // carried has_gripper). Rate-limited to config.forward_rate_hz; cheap no-op
    // when disabled or not started.
    void forward(const DualArmCommand& command);

    // Latest cached feedback for an arm; valid=false if none or older than the
    // configured stale timeout.
    GripperArmFeedback latest(ArmId arm) const;

private:
    void receiveLoop();

    GripperConfig config_;
    int send_fd_ = -1;
    int recv_fd_ = -1;
    sockaddr_storage cmd_addr_{};
    socklen_t cmd_addr_len_ = 0;
    uint64_t seq_ = 0;
    uint64_t last_forward_ns_ = 0;
    uint64_t forward_period_ns_ = 0;
    uint64_t stale_timeout_ns_ = 0;

    mutable std::mutex fb_mutex_;
    GripperArmFeedback left_fb_;
    GripperArmFeedback right_fb_;
    uint64_t left_fb_time_ns_ = 0;
    uint64_t right_fb_time_ns_ = 0;

    std::atomic<bool> running_{false};
    std::thread recv_thread_;
};

}  // namespace rb_servo
