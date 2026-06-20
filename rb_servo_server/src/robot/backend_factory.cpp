#include "rb_servo/robot/backend_factory.hpp"

#include <stdexcept>

#include "rb_servo/robot/mock_backend.hpp"
#include "rb_servo/robot/rbpodo_backend.hpp"

namespace rb_servo {

std::unique_ptr<IRobotBackend> BackendFactory::create(
    ArmId arm_id,
    const BackendConfig& config
) {
    switch (config.backend_type) {
        case BackendType::Mock:
            return std::make_unique<MockBackend>(arm_id, config);
        case BackendType::Rbpodo:
            return std::make_unique<RbpodoBackend>(arm_id, config);
    }
    throw std::runtime_error("Unsupported backend type");
}

}  // namespace rb_servo
