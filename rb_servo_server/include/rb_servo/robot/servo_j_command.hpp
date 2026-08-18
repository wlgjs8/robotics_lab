#pragma once

#include <cstddef>
#include <iomanip>
#include <sstream>
#include <string>

#include "rb_servo/core/types.hpp"

namespace rb_servo {

// Builds the exact `move_servo_j(...)` script the rbpodo controller expects, with
// the joint angles at a chosen decimal precision.
//
// WHY WE FORMAT THIS OURSELVES
// ----------------------------
// The SDK's Cobot::move_servo_j is:
//
//     ss << "move_servo_j(" << Type::joint_to_string(joint) << "," << t1 << ","
//        << t2 << "," << gain << "," << alpha << ")";
//     sock_.send(ss.str());
//     return wait_until_ack_message(...);
//
// and Cobot::eval is `sock_.send(script); return wait_until_ack_message(...)`.
// They are byte-equivalent on the wire, so producing the string here and handing
// it to the public eval() changes nothing except the precision -- no patch to the
// third-party SDK, no /usr/local side effects for other consumers.
//
// joint_to_string uses a plain std::stringstream at default precision: 6
// SIGNIFICANT digits. At our joint magnitudes (26-240 deg) that is a 0.1-1 m-deg
// wire grid, against 5-11 m-deg of measured per-tick motion on the coarse joints
// (2026-08-18, logs/servo_log_20260818_134558.csv). The box differentiates that
// staircase over its servo_t2_sec lookahead, so the quantization shows up in the
// velocity/acceleration domain rather than in position -- currently masked by the
// box's own LPF, and the reason the previous "servo_alpha 10 = jerk" result
// cannot be trusted. controller-manager uses %.7f for the same reason.
//
// EVERYTHING EXCEPT THE JOINTS IS FORMATTED EXACTLY AS THE SDK DOES IT, so the
// only wire difference is the digits after the decimal point in the angles. t1,
// t2, gain and alpha go out through a default-precision stream, matching
// move_servo_j byte for byte (0.002, 0.021, 1, 10).
inline std::string formatServoJCommand(
    const JointArray& q_deg,
    double t1,
    double t2,
    double gain,
    double alpha,
    int joint_decimals
) {
    std::ostringstream ss;
    ss << "move_servo_j(jnt[";
    if (joint_decimals > 0) {
        ss << std::fixed << std::setprecision(joint_decimals);
    }
    for (std::size_t i = 0; i < q_deg.size(); ++i) {
        if (i != 0) {
            ss << ", ";
        }
        ss << q_deg[i];
    }
    // Restore stream defaults so the trailing scalars match the SDK exactly.
    ss << std::defaultfloat << std::setprecision(6);
    ss << "]," << t1 << "," << t2 << "," << gain << "," << alpha << ")";
    return ss.str();
}

}  // namespace rb_servo
