#pragma once
#include <cstdint>
#include <string>

namespace camera_server {

enum class ClockKind { Monotonic, MonotonicRaw };
ClockKind parse_clock_kind(const std::string& value);
uint64_t now_ns(ClockKind kind = ClockKind::MonotonicRaw);

}  // namespace camera_server
