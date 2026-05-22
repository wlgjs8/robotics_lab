#include "rb_servo/core/realtime.hpp"

#include <iostream>

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#endif

namespace rb_servo {

bool lockMemory() {
#ifdef __linux__
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::cerr << "[WARN] mlockall failed. Continue without memory lock.\n";
        return false;
    }
    return true;
#else
    std::cerr << "[WARN] lockMemory is only supported on Linux.\n";
    return false;
#endif
}

bool setCurrentThreadRealtimePriority(int priority) {
#ifdef __linux__
    sched_param sch_params{};
    sch_params.sched_priority = priority;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch_params) != 0) {
        std::cerr << "[WARN] failed to set realtime priority. Continue normal scheduling.\n";
        return false;
    }
    return true;
#else
    std::cerr << "[WARN] realtime priority is only supported on Linux.\n";
    return false;
#endif
}

bool pinCurrentThreadToCpu(int cpu_core) {
#ifdef __linux__
    if (cpu_core < 0) {
        return true;
    }
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_core, &cpuset);
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
        std::cerr << "[WARN] failed to pin thread to CPU " << cpu_core << ".\n";
        return false;
    }
    return true;
#else
    (void)cpu_core;
    std::cerr << "[WARN] CPU pinning is only supported on Linux.\n";
    return false;
#endif
}

}  // namespace rb_servo
