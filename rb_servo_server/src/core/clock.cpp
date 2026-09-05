#include "rb_servo/core/clock.hpp"

#include <chrono>
#include <atomic>

namespace rb_servo {
namespace { std::atomic<uint64_t> external_ns{0}; }

void setExternalSteadyNs(uint64_t ns) { external_ns.store(ns); }

uint64_t nowSteadyNs() {
    if (const auto ns = external_ns.load()) return ns;
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

uint64_t nowSystemNs() {
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

double nsToSec(uint64_t ns) {
    return static_cast<double>(ns) * 1e-9;
}

double nsToMs(uint64_t ns) {
    return static_cast<double>(ns) * 1e-6;
}

}  // namespace rb_servo
