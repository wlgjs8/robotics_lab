#pragma once

namespace rb_servo {

bool lockMemory();
bool setCurrentThreadRealtimePriority(int priority);
bool pinCurrentThreadToCpu(int cpu_core);

}  // namespace rb_servo
