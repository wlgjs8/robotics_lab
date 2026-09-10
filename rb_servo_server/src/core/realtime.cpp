#include "rb_servo/core/realtime.hpp"

#include <cstring>
#include <iostream>

#ifdef __linux__
#include <dlfcn.h>
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

int pinBlasThreads(int threads) {
#ifdef __linux__
    using GetFn = int (*)();
    using SetFn = void (*)(int);
    void* get_symbol = dlsym(RTLD_DEFAULT, "openblas_get_num_threads");
    void* set_symbol = dlsym(RTLD_DEFAULT, "openblas_set_num_threads");
    if (!get_symbol || !set_symbol || threads < 1) return -1;
    GetFn get = nullptr;
    SetFn set = nullptr;
    std::memcpy(&get, &get_symbol, sizeof(get));
    std::memcpy(&set, &set_symbol, sizeof(set));
    const int previous = get();
    set(threads);
    return previous;
#else
    (void)threads;
    return -1;
#endif
}

}  // namespace rb_servo
