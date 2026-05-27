#include <atomic>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include "rb_servo/config/config.hpp"
#include "rb_servo/control/command_buffer.hpp"
#include "rb_servo/control/dual_arm_servo_loop.hpp"
#include "rb_servo/logging/servo_logger.hpp"
#include "rb_servo/network/command_server.hpp"
#include "rb_servo/network/state_publisher.hpp"
#include "rb_servo/robot/backend_factory.hpp"

namespace {
std::atomic<bool> g_running{true};

void signalHandler(int) {
    g_running = false;
}
}  // namespace

int main(int argc, char** argv) {
    std::string config_path = "config/dual_mock.yaml";
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        }
    }

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    try {
        auto config = rb_servo::loadConfigFromYaml(config_path);

        auto left_robot = rb_servo::BackendFactory::create(
            rb_servo::ArmId::Left,
            config.left_robot
        );
        auto right_robot = rb_servo::BackendFactory::create(
            rb_servo::ArmId::Right,
            config.right_robot
        );

        rb_servo::CommandBuffer command_buffer;
        rb_servo::ServoLogger logger(config.logging);
        rb_servo::CommandServer command_server(config.network, &command_buffer);

        rb_servo::DualArmServoLoop servo_loop(
            std::move(left_robot),
            std::move(right_robot),
            config,
            &command_buffer,
            &logger
        );
        rb_servo::StatePublisher state_publisher(
            config,
            [&servo_loop]() {
                return servo_loop.latestSnapshot();
            }
        );

        if (!logger.start()) {
            return 1;
        }
        if (!servo_loop.start()) {
            std::cerr << "[ERROR] failed to start servo loop\n";
            logger.stop();
            return 1;
        }
        if (!command_server.start()) {
            std::cerr << "[ERROR] failed to start command server\n";
            servo_loop.stop();
            logger.stop();
            return 1;
        }
        if (!state_publisher.start()) {
            std::cerr << "[ERROR] failed to start state publisher\n";
            command_server.stop();
            servo_loop.stop();
            logger.stop();
            return 1;
        }

        std::cout << "rb_servo_server started with config: " << config_path << "\n";
        while (g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        state_publisher.stop();
        command_server.stop();
        servo_loop.stop();
        logger.stop();
        std::cout << "rb_servo_server stopped\n";
    } catch (const std::exception& e) {
        std::cerr << "[FATAL] " << e.what() << "\n";
        return 1;
    }

    return 0;
}
