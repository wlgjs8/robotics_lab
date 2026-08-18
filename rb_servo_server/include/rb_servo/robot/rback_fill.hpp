#pragma once

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>

namespace rb_servo {

// Controller command-queue occupancy observed from the box's RBACK[<n>] ACKs.
//
// The rbpodo controller ACKs every streamed command on the command channel with
// "RBACK[<n>]", where n is the queue occupancy at RECEIVE TIME -- the depth the
// box still had to work through BEFORE this command was appended, which is the
// convention that makes  executing_row = emitted_row - fill  exact rather than
// exact-to-within-a-tick.
//
// rb_servo_server already drains that channel every servo cycle (see
// rbpodo_backend.cpp: the SDK's disable_waiting_ack does not stop the controller
// from answering, so the responses must be read or the socket eventually
// corrupts). Parsing the same bytes on the way to the bin turns a pure drain
// into a free phase observer.
//
// Why we want the observer: the servo loop streams at exactly servo.rate_hz off
// the host clock while the box consumes at its own slightly slower rate, so the
// queue grows without bound and the plan-to-robot delay grows with it. Measured
// on hardware 2026-08-18 (logs/servo_log_20260818_104415.csv, sliding-window
// q_sent -> q_actual cross-correlation): 54 ms at t=9 s rising to 138 ms at
// t=69 s, i.e. +1.3-1.4 ms/s, which eventually latched a ChunkFollowerFault
// actual_lead trip on the right arm (4.65 deg against a 4.0 deg budget). The
// budget is sized for the ~65 ms a fresh session shows, so the defect is the
// drift, not the budget.
//
// This header is parsing only -- nothing here acts on the value.
struct RbackScan {
    std::optional<int> last_fill;  // newest occupancy seen; nullopt if none parsed
    int samples = 0;               // well-formed RBACK[<int>] tokens parsed
    int unparsed = 0;              // inspected responses that held no such token
};

// Scans one raw response line for every complete "RBACK[<int>]" and folds the
// result into `scan`. The newest value wins: within a single drain the last
// token is the one that describes the queue now.
//
// Robustness note: the rbpodo SDK splits the response stream on '\n' with no
// carry buffer (cobot.hpp::_read_response_collector_from_buffer), so a token
// straddling a TCP chunk boundary arrives as two fragments and matches neither
// half. We deliberately drop those instead of guessing -- at servo rate the
// signal is massively oversampled for a loop whose time constant is seconds, and
// `unparsed` keeps the loss rate visible in telemetry. (controller-manager's own
// reader keeps a 15-byte carry for this; we do not have access to the raw socket
// through the SDK, hence the simpler fail-quiet rule.)
inline void scanRbackFillLine(std::string_view raw, RbackScan& scan) {
    static constexpr std::string_view kToken = "RBACK[";
    bool matched = false;
    for (std::size_t pos = raw.find(kToken); pos != std::string_view::npos;
         pos = raw.find(kToken, pos)) {
        pos += kToken.size();
        const std::size_t close = raw.find(']', pos);
        if (close == std::string_view::npos) {
            break;  // truncated token: the tail of a split chunk
        }
        // Bound the digit run so the conversion cannot overflow on a garbled
        // fragment; a real occupancy never needs 9 digits.
        const bool digits_only =
            close > pos && close - pos <= 9 &&
            std::all_of(raw.begin() + static_cast<std::ptrdiff_t>(pos),
                        raw.begin() + static_cast<std::ptrdiff_t>(close),
                        [](unsigned char c) { return std::isdigit(c) != 0; });
        if (digits_only) {
            scan.last_fill = std::stoi(std::string(raw.substr(pos, close - pos)));
            ++scan.samples;
            matched = true;
        }
        pos = close + 1;
    }
    if (!matched) {
        ++scan.unparsed;
    }
}

}  // namespace rb_servo
