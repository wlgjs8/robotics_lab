#pragma once

namespace rb_servo {

bool lockMemory();
bool setCurrentThreadRealtimePriority(int priority);
bool pinCurrentThreadToCpu(int cpu_core);
// Caps the OpenBLAS worker pool that qpOASES's LAPACK/BLAS calls (dpotrf, dgemv)
// fan out over. Returns the pool size before the call, or -1 when no OpenBLAS
// runtime is loaded in this process (nothing to pin). Measured 2026-09-10 in the
// preview worker's coupled angular-norm QP: 27-55 ms per solve with the default
// pool versus 5-6 ms single-threaded (63-cut case), 3-17 ms versus <2 ms for the
// real 1-9-cut solves that blew the 8 ms request budget and braked the arm.
int pinBlasThreads(int threads);

}  // namespace rb_servo
