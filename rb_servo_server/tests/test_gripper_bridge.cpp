#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include <nlohmann/json.hpp>

#include "rb_servo/network/gripper_bridge.hpp"

namespace {

using json = nlohmann::json;

#define RB_CHECK(expr)                                                          \
    do {                                                                       \
        if (!(expr)) {                                                         \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":"   \
                      << __LINE__ << "\n";                                     \
            return false;                                                      \
        }                                                                      \
    } while (0)

int makeBoundUdp(int* port) {
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    ::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    socklen_t len = sizeof(addr);
    ::getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len);
    *port = ntohs(addr.sin_port);
    timeval tv{1, 0};
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    return fd;
}

int freePort() {
    int port = 0;
    const int fd = makeBoundUdp(&port);
    ::close(fd);  // small race, fine for a test
    return port;
}

void sendTo(int port, const std::string& payload) {
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(static_cast<uint16_t>(port));
    ::sendto(fd, payload.data(), payload.size(), 0, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    ::close(fd);
}

rb_servo::GripperConfig makeConfig(int cmd_port, int fb_port) {
    rb_servo::GripperConfig cfg;
    cfg.enable = true;
    cfg.command_endpoint = "udp://127.0.0.1:" + std::to_string(cmd_port);
    cfg.feedback_bind = "udp://127.0.0.1:" + std::to_string(fb_port);
    cfg.forward_rate_hz = 1000;        // first forward not rate-limited
    cfg.feedback_stale_timeout_ms = 60.0;
    return cfg;
}

bool testForwardEmitsGripperCmd() {
    int cmd_port = 0;
    const int listener = makeBoundUdp(&cmd_port);  // stand-in gripper_server
    const int fb_port = freePort();

    rb_servo::GripperBridge bridge(makeConfig(cmd_port, fb_port));
    RB_CHECK(bridge.start());

    rb_servo::DualArmCommand command;
    command.left.has_gripper = true;
    command.left.gripper_target = 70.0;
    command.right.has_gripper = false;  // right not commanded -> omitted
    bridge.forward(command);

    char buf[4096];
    const ssize_t n = ::recvfrom(listener, buf, sizeof(buf), 0, nullptr, nullptr);
    RB_CHECK(n > 0);
    const json msg = json::parse(std::string(buf, n), nullptr, false);
    RB_CHECK(!msg.is_discarded());
    RB_CHECK(msg.value("schema", std::string()) == "robotics_lab.gripper_cmd.v1");
    RB_CHECK(msg.contains("left"));
    RB_CHECK(msg["left"].value("percent", 0.0) == 70.0);
    RB_CHECK(!msg.contains("right"));

    bridge.stop();
    ::close(listener);
    return true;
}

bool testFeedbackCachedAndGoesStale() {
    int cmd_port = 0;
    const int listener = makeBoundUdp(&cmd_port);
    const int fb_port = freePort();

    rb_servo::GripperBridge bridge(makeConfig(cmd_port, fb_port));
    RB_CHECK(bridge.start());

    json state;
    state["schema"] = "robotics_lab.gripper_state.v1";
    state["left"] = {{"percent", 55.0}, {"target_percent", 60.0}, {"moving", true}, {"ok", true}, {"fault", nullptr}};
    sendTo(fb_port, state.dump());

    // wait for the receive thread to cache it
    rb_servo::GripperArmFeedback left{};
    for (int i = 0; i < 100; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        left = bridge.latest(rb_servo::ArmId::Left);
        if (left.valid) break;
    }
    RB_CHECK(left.valid);
    RB_CHECK(left.percent == 55.0);
    RB_CHECK(left.target_percent == 60.0);
    RB_CHECK(left.moving);
    RB_CHECK(left.ok);

    // right never sent -> not valid
    RB_CHECK(!bridge.latest(rb_servo::ArmId::Right).valid);

    // after the stale timeout the cache reads invalid again
    std::this_thread::sleep_for(std::chrono::milliseconds(90));
    RB_CHECK(!bridge.latest(rb_servo::ArmId::Left).valid);

    bridge.stop();
    ::close(listener);
    return true;
}

bool testDisabledBridgeIsNoop() {
    rb_servo::GripperConfig cfg;  // enable=false
    rb_servo::GripperBridge bridge(cfg);
    RB_CHECK(bridge.start());                       // disabled start succeeds
    rb_servo::DualArmCommand command;
    command.left.has_gripper = true;
    bridge.forward(command);                        // no-op, must not crash
    RB_CHECK(!bridge.latest(rb_servo::ArmId::Left).valid);
    bridge.stop();
    return true;
}

}  // namespace

int main() {
    if (!testForwardEmitsGripperCmd()) return 1;
    if (!testFeedbackCachedAndGoesStale()) return 1;
    if (!testDisabledBridgeIsNoop()) return 1;
    std::cout << "gripper_bridge tests passed\n";
    return 0;
}
