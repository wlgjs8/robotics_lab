#pragma once

#include <memory>

#include "rb_servo/config/config.hpp"
#include "rb_servo/robot/i_robot_backend.hpp"

namespace rb_servo {

class BackendFactory {
public:
    static std::unique_ptr<IRobotBackend> create(
        ArmId arm_id,
        const BackendConfig& config
    );
};

}  // namespace rb_servo
