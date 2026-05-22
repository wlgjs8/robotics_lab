#include "camera_server/core/clock.hpp"

#include <ctime>
#include <stdexcept>

namespace camera_server {

ClockKind parse_clock_kind(const std::string& value) {
  if (value == "monotonic") return ClockKind::Monotonic;
  if (value == "monotonic_raw" || value.empty()) return ClockKind::MonotonicRaw;
  throw std::runtime_error("unsupported clock: " + value);
}

uint64_t now_ns(ClockKind kind) {
  timespec ts{};
  const clockid_t clock_id = kind == ClockKind::MonotonicRaw ? CLOCK_MONOTONIC_RAW : CLOCK_MONOTONIC;
  if (clock_gettime(clock_id, &ts) != 0) return 0;
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ull + static_cast<uint64_t>(ts.tv_nsec);
}

}  // namespace camera_server
