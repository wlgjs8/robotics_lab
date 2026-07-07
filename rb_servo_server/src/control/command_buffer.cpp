#include "rb_servo/control/command_buffer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rb_servo {
namespace {
DualArmCommand makeHold(uint64_t now_ns) {
    DualArmCommand hold;
    hold.host_time_ns = now_ns;
    hold.left.arm_id = ArmId::Left;
    hold.right.arm_id = ArmId::Right;
    hold.left.mode = ControlMode::Hold;
    hold.right.mode = ControlMode::Hold;
    return hold;
}

bool validTimeout(double timeout_sec) {
    return timeout_sec > 0.0 && std::isfinite(timeout_sec);
}

double commandTimeoutMs(const DualArmCommand& command) {
    if (!validTimeout(command.left.timeout_sec) || !validTimeout(command.right.timeout_sec)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return std::min(command.left.timeout_sec, command.right.timeout_sec) * 1000.0;
}

double commandAgeMs(const DualArmCommand& command, uint64_t now_ns) {
    if (command.host_time_ns == 0 || now_ns < command.host_time_ns) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return static_cast<double>(now_ns - command.host_time_ns) / 1'000'000.0;
}

double clientSendAgeMs(const DualArmCommand& command, uint64_t now_ns) {
    if (!command.has_client_send_monotonic_ns ||
        command.client_send_monotonic_ns == 0 ||
        now_ns < command.client_send_monotonic_ns) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return static_cast<double>(now_ns - command.client_send_monotonic_ns) / 1'000'000.0;
}

bool isLifecycleMode(ControlMode mode) {
    return mode == ControlMode::ArmMotion ||
           mode == ControlMode::DisarmMotion ||
           mode == ControlMode::EmergencyStop ||
           mode == ControlMode::ResetFault ||
           mode == ControlMode::Freedrive;
}

bool isLifecycleCommand(const DualArmCommand& command) {
    return isLifecycleMode(command.left.mode) || isLifecycleMode(command.right.mode);
}

bool isExternalBoxesCommand(const DualArmCommand& command) {
    return command.left.mode == ControlMode::SetExternalBoxes ||
           command.right.mode == ControlMode::SetExternalBoxes;
}

bool isUsableCommand(const DualArmCommand& command, uint64_t now_ns) {
    if (!validTimeout(command.left.timeout_sec) || !validTimeout(command.right.timeout_sec)) {
        return false;
    }
    const double timeout = std::min(command.left.timeout_sec, command.right.timeout_sec);
    const uint64_t timeout_ns = static_cast<uint64_t>(timeout * 1e9);
    return command.host_time_ns == 0 || now_ns <= command.host_time_ns + timeout_ns;
}

void recordReturned(
    CommandBufferReadTelemetry* telemetry,
    const DualArmCommand& command,
    uint64_t now_ns
) {
    if (telemetry == nullptr) return;
    telemetry->returned_seq = command.seq;
    telemetry->returned_left_mode = command.left.mode;
    telemetry->returned_right_mode = command.right.mode;
    telemetry->returned_host_time_ns = command.host_time_ns;
    telemetry->returned_age_ms = commandAgeMs(command, now_ns);
    telemetry->returned_client_send_age_ms = clientSendAgeMs(command, now_ns);
}

void recordLatest(
    CommandBufferReadTelemetry* telemetry,
    const DualArmCommand& command,
    uint64_t now_ns
) {
    if (telemetry == nullptr) return;
    telemetry->latest_seq = command.seq;
    telemetry->latest_left_mode = command.left.mode;
    telemetry->latest_right_mode = command.right.mode;
    telemetry->latest_host_time_ns = command.host_time_ns;
    telemetry->latest_age_ms = commandAgeMs(command, now_ns);
    telemetry->latest_timeout_ms = commandTimeoutMs(command);
    telemetry->latest_timeout_valid =
        validTimeout(command.left.timeout_sec) && validTimeout(command.right.timeout_sec);
    telemetry->latest_usable = isUsableCommand(command, now_ns);
    telemetry->latest_client_send_age_ms = clientSendAgeMs(command, now_ns);
}

void recordLifecycle(
    CommandBufferReadTelemetry* telemetry,
    const DualArmCommand& command,
    uint64_t now_ns
) {
    if (telemetry == nullptr) return;
    telemetry->lifecycle_seq = command.seq;
    telemetry->lifecycle_left_mode = command.left.mode;
    telemetry->lifecycle_right_mode = command.right.mode;
    telemetry->lifecycle_host_time_ns = command.host_time_ns;
    telemetry->lifecycle_age_ms = commandAgeMs(command, now_ns);
    telemetry->lifecycle_timeout_ms = commandTimeoutMs(command);
    telemetry->lifecycle_timeout_valid =
        validTimeout(command.left.timeout_sec) && validTimeout(command.right.timeout_sec);
    telemetry->lifecycle_usable = isUsableCommand(command, now_ns);
}

void recordExternalBoxes(
    CommandBufferReadTelemetry* telemetry,
    const DualArmCommand& command,
    uint64_t now_ns
) {
    if (telemetry == nullptr) return;
    telemetry->external_boxes_consumed = true;
    telemetry->external_boxes_seq = command.seq;
    telemetry->external_boxes_left_mode = command.left.mode;
    telemetry->external_boxes_right_mode = command.right.mode;
    telemetry->external_boxes_host_time_ns = command.host_time_ns;
    telemetry->external_boxes_age_ms = commandAgeMs(command, now_ns);
    telemetry->external_boxes_client_send_age_ms = clientSendAgeMs(command, now_ns);
}
}  // namespace

void CommandBuffer::setCommand(const DualArmCommand& command) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (isExternalBoxesCommand(command)) {
        pending_external_boxes_command_ = command;
    } else if (isLifecycleCommand(command)) {
        constexpr size_t kMaxPendingLifecycleCommands = 16;
        if (pending_lifecycle_commands_.size() >= kMaxPendingLifecycleCommands) {
            pending_lifecycle_commands_.pop_front();
        }
        pending_lifecycle_commands_.push_back(command);
        latest_command_ = makeHold(command.host_time_ns);
    } else {
        latest_command_ = command;
    }
}

void CommandBuffer::updateLease(const CommandSourceLeaseState& lease, uint64_t now_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!latest_command_.has_value() || !isUsableCommand(*latest_command_, now_ns)) {
        // Absent OR EXPIRED latest command: writing the lease onto an expired
        // command would hide it (latestOrHold falls back to a fresh empty-lease
        // Hold), so an acquiring client's readback would never see the grant —
        // e.g. re-engaging teleop after idle, when the buffer still holds the
        // last timed-out streaming command. Replace with a non-expiring Hold
        // (host_time_ns == 0) so the grant stays visible until motion resumes.
        latest_command_ = makeHold(0);
    }
    latest_command_->lease = lease;
}

void CommandBuffer::noteExternalBoxReceived(uint64_t receive_time_ns) {
    // Lock-free single-scalar publish from the receive thread; independent of the
    // latest_command_ slot so a saturating motion stream cannot hide it.
    last_external_box_receive_ns_.store(receive_time_ns, std::memory_order_relaxed);
}

uint64_t CommandBuffer::lastExternalBoxReceiveNs() const {
    return last_external_box_receive_ns_.load(std::memory_order_relaxed);
}

DualArmCommand CommandBuffer::latestOrHold(
    uint64_t now_ns,
    CommandBufferReadTelemetry* telemetry
) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (telemetry != nullptr) {
        *telemetry = CommandBufferReadTelemetry{};
        telemetry->pending_lifecycle_count =
            static_cast<uint64_t>(pending_lifecycle_commands_.size());
        telemetry->external_boxes_pending = pending_external_boxes_command_.has_value();
    }
    while (!pending_lifecycle_commands_.empty()) {
        DualArmCommand command = pending_lifecycle_commands_.front();
        pending_lifecycle_commands_.pop_front();
        recordLifecycle(telemetry, command, now_ns);
        if (isUsableCommand(command, now_ns)) {
            if (telemetry != nullptr) telemetry->result = "lifecycle";
            recordReturned(telemetry, command, now_ns);
            return command;
        }
        if (telemetry != nullptr) {
            telemetry->result = "stale_lifecycle_skipped";
            ++telemetry->skipped_lifecycle_count;
        }
    }

    if (!latest_command_.has_value()) {
        DualArmCommand hold = makeHold(now_ns);
        if (telemetry != nullptr) telemetry->result = "no_latest_hold";
        recordReturned(telemetry, hold, now_ns);
        return hold;
    }

    DualArmCommand cmd = *latest_command_;
    recordLatest(telemetry, cmd, now_ns);
    if (!isUsableCommand(cmd, now_ns)) {
        // The streaming command went stale, but the command-source lease has its
        // own (longer) timeout. Carry the lease onto the fallback Hold so the
        // published snapshot keeps reporting the active lease; an empty-lease Hold
        // makes the owner (e.g. policy_runner) see "no active command-source
        // lease" in the gaps between teleop sends and flap acquire/reacquire,
        // which stutters teleop. The state publisher still recomputes lease expiry
        // against now, so a genuinely expired lease is still reported inactive.
        DualArmCommand hold = makeHold(now_ns);
        hold.lease = cmd.lease;
        if (telemetry != nullptr) {
            telemetry->result =
                telemetry->latest_timeout_valid ? "stale_latest_hold" : "invalid_timeout_latest_hold";
        }
        recordReturned(telemetry, hold, now_ns);
        return hold;
    }
    if (telemetry != nullptr) {
        telemetry->result =
            telemetry->skipped_lifecycle_count > 0 ? "latest_after_stale_lifecycle" : "latest";
    }
    recordReturned(telemetry, cmd, now_ns);
    return cmd;
}

std::optional<DualArmCommand> CommandBuffer::consumeLatestExternalBoxes(
    uint64_t now_ns,
    CommandBufferReadTelemetry* telemetry
) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (telemetry != nullptr) {
        telemetry->external_boxes_pending =
            telemetry->external_boxes_pending || pending_external_boxes_command_.has_value();
    }
    if (!pending_external_boxes_command_.has_value()) {
        return std::nullopt;
    }
    DualArmCommand command = *pending_external_boxes_command_;
    pending_external_boxes_command_.reset();
    recordExternalBoxes(telemetry, command, now_ns);
    return command;
}

}  // namespace rb_servo
