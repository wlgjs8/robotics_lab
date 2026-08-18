// noaffinity.c - LD_PRELOAD shim: make pthread_setaffinity_np a no-op.
//
// WHY: controller-manager pins its two arm RT threads to cpu1/cpu2 UNCONDITIONALLY
// (src/arm/Arm.cpp, Right ? 2 : 1). A SECOND controller instance started for an
// isolated SILS test (different ROS domain, loopback boxes) would land its 500 Hz
// loops on the SAME two cores as the live one and time-slice against it - on
// energized arms that is a qsync/RT-jitter trip, not a test. With this shim the
// test instance's threads simply keep the process's default mask (isolcpus leaves
// that as 0,4-15), and the live loops on cpu1/cpu2 never see it. Test-only; never
// use it on the controller that drives the robot.
//
//   gcc -shared -fPIC -o noaffinity.so noaffinity.c
//   LD_PRELOAD=$PWD/noaffinity.so <controller>
#define _GNU_SOURCE
#include <pthread.h>
#include <sched.h>
#include <stddef.h>
int pthread_setaffinity_np(pthread_t thread, size_t cpusetsize, const cpu_set_t* cpuset) {
    (void)thread; (void)cpusetsize; (void)cpuset;
    return 0;
}
int sched_setaffinity(pid_t pid, size_t cpusetsize, const cpu_set_t* mask) {
    (void)pid; (void)cpusetsize; (void)mask;
    return 0;
}
