#include "rb_servo/core/clock.hpp"

#include <chrono>

namespace rb_servo {

uint64_t nowSteadyNs() {
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
