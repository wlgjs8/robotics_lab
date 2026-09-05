#pragma once

#include <cstdint>

namespace rb_servo {

uint64_t nowSteadyNs();
// Dedicated externally stepped process only. Zero restores the wall clock.
// Never enabled by rb_servo_server or by a launch YAML.
void setExternalSteadyNs(uint64_t ns);
uint64_t nowSystemNs();
double nsToSec(uint64_t ns);
double nsToMs(uint64_t ns);

}  // namespace rb_servo
