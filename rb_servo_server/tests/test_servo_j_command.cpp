// These strings go on the wire to a real control box, so the format is a
// contract, not a detail: a stray space or a missing paren is a rejected command
// mid-stream. The expectations below mirror rbpodo's Cobot::move_servo_j and
// Type::joint_to_string (rbpodo_src/include/rbpodo/cobot.hpp:1389-1396 and
// data_type.hpp:45-56) so that our formatter can only differ from the SDK in the
// digits after the decimal point.

#include <iostream>
#include <string>

#include "rb_servo/robot/servo_j_command.hpp"

namespace {

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

bool expectEq(const std::string& got, const std::string& want) {
    if (got != want) {
        std::cerr << "  got:  " << got << "\n  want: " << want << "\n";
        return false;
    }
    return true;
}

rb_servo::JointArray joints(double a, double b, double c, double d, double e, double f) {
    return rb_servo::JointArray{a, b, c, d, e, f};
}

// The joint list only. Needed because the command NAME contains 'e'
// ("move_servo_j"), so scanning the whole string for an exponent marker matches
// unconditionally and silently passes.
std::string jointList(const std::string& command) {
    const std::size_t open = command.find("jnt[");
    const std::size_t close = command.find(']', open);
    if (open == std::string::npos || close == std::string::npos) return "";
    return command.substr(open + 4, close - open - 4);
}

// decimals = 0 must reproduce the SDK's own output byte for byte: a plain
// stringstream at default precision, i.e. 6 SIGNIFICANT digits. This pins the
// surrounding structure (the "jnt[" prefix, ", " between joints, no space around
// the scalar commas) which the high-precision path reuses.
bool testDefaultPrecisionMatchesTheSdkFormat() {
    const std::string cmd = rb_servo::formatServoJCommand(
        joints(125.3441234, -57.7812345, 0.0, 1.5, -0.25, 239.8006789),
        0.002, 0.021, 1.0, 1.0, 0
    );
    return expectEq(
        cmd,
        "move_servo_j(jnt[125.344, -57.7812, 0, 1.5, -0.25, 239.801],0.002,0.021,1,1)"
    );
}

// The whole point of the change: 7 decimals, fixed, regardless of magnitude.
bool testHighPrecisionEmitsSevenDecimals() {
    const std::string cmd = rb_servo::formatServoJCommand(
        joints(125.3441234, -57.7812345, 0.0, 1.5, -0.25, 239.8006789),
        0.002, 0.021, 1.0, 10.0, 7
    );
    return expectEq(
        cmd,
        "move_servo_j(jnt[125.3441234, -57.7812345, 0.0000000, 1.5000000, "
        "-0.2500000, 239.8006789],0.002,0.021,1,10)"
    );
}

// Raising joint precision must NOT change how t1/t2/gain/alpha are written, or
// the command differs from the SDK's in a second place nobody reviewed.
bool testTrailingScalarsKeepSdkFormatting() {
    const std::string cmd = rb_servo::formatServoJCommand(
        joints(1, 2, 3, 4, 5, 6), 0.002, 0.021, 1.0, 10.0, 7
    );
    RB_CHECK(cmd.find("],0.002,0.021,1,10)") != std::string::npos);

    // A fractional gain must still round-trip as the SDK writes it (the soft-entry
    // ramp feeds non-integer gains during servo engagement).
    const std::string ramped = rb_servo::formatServoJCommand(
        joints(1, 2, 3, 4, 5, 6), 0.002, 0.021, 0.35, 10.0, 7
    );
    RB_CHECK(ramped.find("],0.002,0.021,0.35,10)") != std::string::npos);
    return true;
}

// The defect this exists to fix: at our joint magnitudes the 6-significant-digit
// path cannot represent a single tick of motion on the coarse joints, so two
// distinct targets collapse to the same wire value. The high-precision path must
// keep them distinct.
bool testSixDigitPathCollapsesMotionThatSevenDecimalsKeeps() {
    // 240 deg with 0.2 m-deg of motion -- below the 1 m-deg grid that 6
    // significant digits leaves at this magnitude.
    const auto a = joints(239.8006000, 0, 0, 0, 0, 0);
    const auto b = joints(239.8008000, 0, 0, 0, 0, 0);

    const std::string a6 = rb_servo::formatServoJCommand(a, 0.002, 0.021, 1.0, 1.0, 0);
    const std::string b6 = rb_servo::formatServoJCommand(b, 0.002, 0.021, 1.0, 1.0, 0);
    if (a6 != b6) {
        std::cerr << "  expected the 6-significant-digit path to collapse these:\n"
                  << "   " << a6 << "\n   " << b6 << "\n";
        return false;
    }

    const std::string a7 = rb_servo::formatServoJCommand(a, 0.002, 0.021, 1.0, 10.0, 7);
    const std::string b7 = rb_servo::formatServoJCommand(b, 0.002, 0.021, 1.0, 10.0, 7);
    RB_CHECK(a7 != b7);
    return true;
}

// Scientific notation is a token shape the box's parser has never been shown.
// The SDK's default-precision path DOES emit it for angles near zero -- verified
// here rather than assumed:
//
//     decimals=0 -> move_servo_j(jnt[1e-08, -1e-07, 1e-07, 360, -360, 1e-09],...)
//
// That is latent in the stock path (a joint passing through zero is the trigger)
// and is a second, independent reason to prefer the fixed-notation path. Our
// high-precision path must never do it.
bool testHighPrecisionNeverEmitsScientificNotation() {
    const auto tiny = joints(1e-8, -1e-7, 0.0000001, 359.9999999, -359.9999999, 1e-9);

    const std::string sdk_joints =
        jointList(rb_servo::formatServoJCommand(tiny, 0.002, 0.021, 1.0, 1.0, 0));
    // Pin the known-bad behaviour so nobody "fixes" it silently and changes the
    // stock path's bytes without meaning to.
    RB_CHECK(!sdk_joints.empty());
    RB_CHECK(sdk_joints.find('e') != std::string::npos);

    const std::string fixed_joints =
        jointList(rb_servo::formatServoJCommand(tiny, 0.002, 0.021, 1.0, 10.0, 7));
    RB_CHECK(!fixed_joints.empty());
    if (fixed_joints.find('e') != std::string::npos ||
        fixed_joints.find('E') != std::string::npos) {
        std::cerr << "  scientific notation leaked from the high-precision path:\n   "
                  << fixed_joints << "\n";
        return false;
    }
    return true;
}

}  // namespace

int main() {
    if (!testDefaultPrecisionMatchesTheSdkFormat()) return 1;
    if (!testHighPrecisionEmitsSevenDecimals()) return 1;
    if (!testTrailingScalarsKeepSdkFormatting()) return 1;
    if (!testSixDigitPathCollapsesMotionThatSevenDecimalsKeeps()) return 1;
    if (!testHighPrecisionNeverEmitsScientificNotation()) return 1;
    return 0;
}
