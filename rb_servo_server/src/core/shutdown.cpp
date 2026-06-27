#include "rb_servo/core/shutdown.hpp"

#include <atomic>

namespace rb_servo {
namespace {
std::atomic<bool> g_shutdown_requested{false};
}  // namespace

void requestShutdown() {
    g_shutdown_requested.store(true, std::memory_order_relaxed);
}

bool shutdownRequested() {
    return g_shutdown_requested.load(std::memory_order_relaxed);
}

}  // namespace rb_servo
