#include <atomic>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "rb_servo/config/config.hpp"
#include "rb_servo/core/shutdown.hpp"
#include "rb_servo/control/command_buffer.hpp"
#include "rb_servo/control/dual_arm_servo_loop.hpp"
#include "rb_servo/logging/servo_logger.hpp"
#include "rb_servo/network/chunk_frame_receiver.hpp"
#include "rb_servo/network/command_server.hpp"
#include "rb_servo/network/scope_publisher.hpp"
#include "rb_servo/network/state_publisher.hpp"
#include "rb_servo/robot/backend_factory.hpp"

namespace {
std::atomic<bool> g_running{true};

void signalHandler(int) {
    g_running = false;
    // Also unblock long-running startup sequences (e.g. the rbpodo pgmode
    // switch / activation confirmation polls) so Ctrl-C during initialize
    // exits promptly instead of leaving a zombie holding the command port.
    rb_servo::requestShutdown();
}
}  // namespace

int main(int argc, char** argv) {
    std::string config_path;
    bool check_config_only = false;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        } else if (arg == "--config") {
            std::cerr << "--config requires a path\n";
            return 2;
        } else if (arg == "--check-config") {
            // Dry run: load + validate the config (the same strict fail-closed
            // loader the server uses), then exit without touching any backend.
            // For pre-flighting a tracked-config edit without the hardware.
            check_config_only = true;
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "usage: rb_servo_server --config <path> [--check-config]\n";
            return 0;
        }
    }
    if (config_path.empty()) {
        std::cerr << "usage: rb_servo_server --config <path> [--check-config]\n";
        return 2;
    }

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    try {
        auto config = rb_servo::loadConfigFromYaml(config_path);
        if (check_config_only) {
            std::cout << "config OK: " << config_path << "\n";
            return 0;
        }

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
        std::unique_ptr<rb_servo::ScopePublisher> scope_publisher;
        if (config.scope.enable) {
            scope_publisher = std::make_unique<rb_servo::ScopePublisher>(
                config.scope,
                config.network
            );
        }
        rb_servo::CommandServer command_server(config.network, &command_buffer, config.cartesian_control);
        // Dedicated chunk-frame ingest for the Ruckig chunk-follower (empty bind
        // = disabled). Declared before the servo loop so it outlives it.
        rb_servo::ChunkFrameReceiver chunk_frame_receiver(config.network.chunk_frame_bind);

        rb_servo::DualArmServoLoop servo_loop(
            std::move(left_robot),
            std::move(right_robot),
            config,
            &command_buffer,
            &logger,
            nullptr,
            scope_publisher.get()
        );
        servo_loop.setChunkFrameReceiver(&chunk_frame_receiver);
        rb_servo::StatePublisher state_publisher(
            config,
            [&servo_loop]() {
                return servo_loop.latestSnapshot();
            },
            [&servo_loop](rb_servo::ArmId arm, double percent, bool valid) {
                servo_loop.setGripperFeedback(arm, percent, valid);
            }
        );

        if (!logger.start()) {
            return 1;
        }
        if (scope_publisher && !scope_publisher->start()) {
            logger.stop();
            return 1;
        }
        if (!servo_loop.start()) {
            std::cerr << "[ERROR] failed to start servo loop\n";
            if (scope_publisher) scope_publisher->stop();
            logger.stop();
            return 1;
        }
        if (!command_server.start()) {
            std::cerr << "[ERROR] failed to start command server\n";
            servo_loop.stop();
            if (scope_publisher) scope_publisher->stop();
            logger.stop();
            return 1;
        }
        if (!chunk_frame_receiver.start()) {
            std::cerr << "[ERROR] failed to start chunk frame receiver\n";
            command_server.stop();
            servo_loop.stop();
            if (scope_publisher) scope_publisher->stop();
            logger.stop();
            return 1;
        }
        if (!state_publisher.start()) {
            std::cerr << "[ERROR] failed to start state publisher\n";
            chunk_frame_receiver.stop();
            command_server.stop();
            servo_loop.stop();
            if (scope_publisher) scope_publisher->stop();
            logger.stop();
            return 1;
        }

        std::cout << "rb_servo_server started with config: " << config_path << "\n";
        while (g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        state_publisher.stop();
        chunk_frame_receiver.stop();
        command_server.stop();
        servo_loop.stop();
        if (scope_publisher) scope_publisher->stop();
        logger.stop();
        std::cout << "rb_servo_server stopped\n";
    } catch (const std::exception& e) {
        std::cerr << "[FATAL] " << e.what() << "\n";
        return 1;
    }

    return 0;
}
