#include "rb_servo/control/joint_moving_average.hpp"

#include <cmath>
#include <iostream>

namespace {

#define RB_CHECK(cond)                                                       \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::cerr << "CHECK failed at " << __FILE__ << ":" << __LINE__  \
                      << ": " #cond << "\n";                                 \
            return false;                                                    \
        }                                                                    \
    } while (false)

rb_servo::JointArray joints(double value) {
    rb_servo::JointArray q{};
    for (int i = 0; i < rb_servo::kDof; ++i) q[i] = value + i;
    return q;
}

bool testPassThroughWhenDisabled() {
    for (int window : {0, 1}) {
        rb_servo::JointMovingAverage ma(window);
        const rb_servo::JointArray out = ma.apply(joints(3.0));
        for (int i = 0; i < rb_servo::kDof; ++i) RB_CHECK(out[i] == joints(3.0)[i]);
    }
    return true;
}

bool testFirstSampleFillsWindowNoWarmupRamp() {
    rb_servo::JointMovingAverage ma(40);
    const rb_servo::JointArray out = ma.apply(joints(5.0));
    for (int i = 0; i < rb_servo::kDof; ++i) RB_CHECK(std::abs(out[i] - joints(5.0)[i]) < 1e-12);
    return true;
}

bool testBoxcarAverageAndWindowSlide() {
    rb_servo::JointMovingAverage ma(4);
    ma.apply(joints(0.0));  // fills window with 0-based joints
    ma.apply(joints(0.0));
    ma.apply(joints(0.0));
    const rb_servo::JointArray step = ma.apply(joints(4.0));
    // window now holds three joints(0) and one joints(4) -> mean offset +1
    RB_CHECK(std::abs(step[0] - 1.0) < 1e-12);
    RB_CHECK(std::abs(step[3] - 4.0) < 1e-12);  // base 3 + offset 1
    ma.apply(joints(4.0));
    ma.apply(joints(4.0));
    const rb_servo::JointArray converged = ma.apply(joints(4.0));
    RB_CHECK(std::abs(converged[0] - 4.0) < 1e-12);  // window fully slid
    return true;
}

bool testStepResponseDelayIsHalfWindow() {
    rb_servo::JointMovingAverage ma(40);
    ma.apply(joints(0.0));
    rb_servo::JointArray out{};
    for (int i = 0; i < 20; ++i) out = ma.apply(joints(1.0));
    // After half the window the boxcar should be at 50% of the step.
    RB_CHECK(std::abs(out[0] - 0.5) < 1e-12);
    for (int i = 0; i < 20; ++i) out = ma.apply(joints(1.0));
    RB_CHECK(std::abs(out[0] - 1.0) < 1e-12);
    return true;
}

bool testResetRefills() {
    rb_servo::JointMovingAverage ma(8);
    for (int i = 0; i < 8; ++i) ma.apply(joints(2.0));
    ma.reset();
    const rb_servo::JointArray out = ma.apply(joints(9.0));
    RB_CHECK(std::abs(out[0] - 9.0) < 1e-12);
    return true;
}

}  // namespace

int main() {
    if (!testPassThroughWhenDisabled()) return 1;
    if (!testFirstSampleFillsWindowNoWarmupRamp()) return 1;
    if (!testBoxcarAverageAndWindowSlide()) return 1;
    if (!testStepResponseDelayIsHalfWindow()) return 1;
    if (!testResetRefills()) return 1;
    std::cout << "joint_moving_average tests passed\n";
    return 0;
}
