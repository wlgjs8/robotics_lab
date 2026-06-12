#pragma once

namespace rb_servo {

// Process-wide shutdown request flag (set by the SIGINT/SIGTERM handler in
// main). Long-running blocking sequences outside the servo loop — e.g. the
// rbpodo initialize pgmode-switch / activation confirmation polls — must
// check this so Ctrl-C aborts them promptly instead of leaving a half-started
// server holding the command port.
void requestShutdown();
bool shutdownRequested();

}  // namespace rb_servo
