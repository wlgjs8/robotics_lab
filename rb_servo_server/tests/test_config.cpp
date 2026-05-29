#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>

#include "rb_servo/config/config.hpp"

namespace {

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

std::string writeTempConfig(const std::string& name, const std::string& body) {
    const std::string path = "/tmp/rb-servo-wrap-config-" + name + "-" + std::to_string(getpid()) + ".yaml";
    std::ofstream file(path);
    file << body;
    return path;
}

bool loadRejects(const std::string& path) {
    try {
        (void)rb_servo::loadConfigFromYaml(path);
    } catch (const std::exception&) {
        return true;
    }
    return false;
}

bool near(double a, double b) {
    return std::abs(a - b) < 1e-12;
}

bool testJointWrapConfigParses() {
    const std::string path = writeTempConfig(
        "valid",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  send_servo_commands: false\n"
        "safety:\n"
        "  joint_wrap_period_deg: [0, 0, 0, 360, 0, 360]\n"
        "  joint_wrap_for_startup_validation: true\n"
        "  joint_wrap_for_motion_safety: false\n"
    );
    const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
    ::unlink(path.c_str());
    RB_CHECK(near(cfg.safety.joint_wrap_period_deg[3], 360.0));
    RB_CHECK(near(cfg.safety.joint_wrap_period_deg[5], 360.0));
    RB_CHECK(cfg.safety.joint_wrap_for_startup_validation);
    RB_CHECK(!cfg.safety.joint_wrap_for_motion_safety);
    return true;
}

bool testInvalidJointWrapConfigRejects() {
    const std::string negative_path = writeTempConfig(
        "negative-period",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "safety:\n"
        "  joint_wrap_period_deg: [0, 0, 0, -360, 0, 360]\n"
    );
    RB_CHECK(loadRejects(negative_path));
    ::unlink(negative_path.c_str());

    const std::string wrong_length_path = writeTempConfig(
        "wrong-length",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "safety:\n"
        "  joint_wrap_period_deg: [0, 0, 0, 360, 0]\n"
    );
    RB_CHECK(loadRejects(wrong_length_path));
    ::unlink(wrong_length_path.c_str());

    const std::string nonfinite_path = writeTempConfig(
        "nonfinite-period",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "safety:\n"
        "  joint_wrap_period_deg: [0, 0, 0, .nan, 0, 360]\n"
    );
    RB_CHECK(loadRejects(nonfinite_path));
    ::unlink(nonfinite_path.c_str());

    const std::string motion_wrap_path = writeTempConfig(
        "motion-wrap",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  send_servo_commands: false\n"
        "safety:\n"
        "  joint_wrap_period_deg: [0, 0, 0, 360, 0, 360]\n"
        "  joint_wrap_for_motion_safety: true\n"
    );
    RB_CHECK(loadRejects(motion_wrap_path));
    ::unlink(motion_wrap_path.c_str());

    const std::string startup_motion_path = writeTempConfig(
        "startup-motion",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  send_servo_commands: true\n"
        "safety:\n"
        "  joint_wrap_period_deg: [0, 0, 0, 360, 0, 360]\n"
        "  joint_wrap_for_startup_validation: true\n"
    );
    RB_CHECK(loadRejects(startup_motion_path));
    ::unlink(startup_motion_path.c_str());

    return true;
}

}  // namespace

int main() {
    if (!testJointWrapConfigParses()) return 1;
    if (!testInvalidJointWrapConfigRejects()) return 1;
    return 0;
}
