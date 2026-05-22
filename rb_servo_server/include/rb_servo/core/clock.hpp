#pragma once

#include <cstdint>

namespace rb_servo {

uint64_t nowSteadyNs();
uint64_t nowSystemNs();
double nsToSec(uint64_t ns);
double nsToMs(uint64_t ns);

}  // namespace rb_servo
