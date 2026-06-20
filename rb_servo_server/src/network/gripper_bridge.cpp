#include "rb_servo/network/gripper_bridge.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <vector>

#include <nlohmann/json.hpp>

#include "rb_servo/network/state_publisher.hpp"  // parseUdpEndpointUri

namespace rb_servo {
namespace {

using json = nlohmann::json;

constexpr char kCommandSchema[] = "robotics_lab.gripper_cmd.v1";
constexpr char kStateSchema[] = "robotics_lab.gripper_state.v1";

uint64_t nowSteadyNs() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

// Read a per-arm feedback block; tolerates null percent (no live read => ok=false).
void parseArmFeedback(const json& msg, const char* arm, GripperArmFeedback* out) {
    *out = GripperArmFeedback{};
    auto it = msg.find(arm);
    if (it == msg.end() || !it->is_object()) return;
    const json& block = *it;
    const json pct = block.value("percent", json(nullptr));
    if (pct.is_number()) {
        out->percent = pct.get<double>();
        out->valid = true;
    }
    const json tgt = block.value("target_percent", json(nullptr));
    if (tgt.is_number()) out->target_percent = tgt.get<double>();
    out->moving = block.value("moving", false);
    out->ok = block.value("ok", out->valid);
    const json fault = block.value("fault", json(nullptr));
    if (fault.is_string()) out->fault = fault.get<std::string>();
}

}  // namespace

GripperBridge::GripperBridge(const GripperConfig& config) : config_(config) {
    forward_period_ns_ = config_.forward_rate_hz > 0
        ? static_cast<uint64_t>(1e9 / config_.forward_rate_hz)
        : 0;
    stale_timeout_ns_ = static_cast<uint64_t>(config_.feedback_stale_timeout_ms * 1e6);
}

GripperBridge::~GripperBridge() { stop(); }

bool GripperBridge::start() {
    if (!config_.enable) return true;  // disabled bridge is a successful no-op

    // Resolve gripper_server command endpoint (send target).
    std::string host;
    int port = 0;
    if (!StatePublisher::parseUdpEndpointUri(config_.command_endpoint, &host, &port)) {
        std::cerr << "[ERROR] GripperBridge bad command_endpoint: " << config_.command_endpoint << "\n";
        return false;
    }
    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;
    addrinfo* results = nullptr;
    if (::getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &results) != 0 || !results) {
        std::cerr << "[ERROR] GripperBridge cannot resolve " << host << ":" << port << "\n";
        return false;
    }
    std::memcpy(&cmd_addr_, results->ai_addr, results->ai_addrlen);
    cmd_addr_len_ = static_cast<socklen_t>(results->ai_addrlen);
    ::freeaddrinfo(results);

    send_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (send_fd_ < 0) {
        std::cerr << "[ERROR] GripperBridge send socket failed\n";
        return false;
    }

    // Bind feedback receive socket.
    std::string fb_host;
    int fb_port = 0;
    if (!StatePublisher::parseUdpEndpointUri(config_.feedback_bind, &fb_host, &fb_port)) {
        std::cerr << "[ERROR] GripperBridge bad feedback_bind: " << config_.feedback_bind << "\n";
        return false;
    }
    recv_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (recv_fd_ < 0) {
        std::cerr << "[ERROR] GripperBridge recv socket failed\n";
        return false;
    }
    int reuse = 1;
    ::setsockopt(recv_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    // 200 ms recv timeout so the receive thread can observe running_=false.
    timeval tv{0, 200000};
    ::setsockopt(recv_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    sockaddr_in bind_addr{};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(static_cast<uint16_t>(fb_port));
    bind_addr.sin_addr.s_addr = (fb_host == "0.0.0.0" || fb_host.empty())
        ? htonl(INADDR_ANY) : ::inet_addr(fb_host.c_str());
    if (::bind(recv_fd_, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr)) < 0) {
        std::cerr << "[ERROR] GripperBridge bind feedback failed on " << fb_host << ":" << fb_port << "\n";
        return false;
    }

    running_.store(true);
    recv_thread_ = std::thread(&GripperBridge::receiveLoop, this);
    std::cerr << "[INFO] GripperBridge up: cmd->" << config_.command_endpoint
              << " feedback<-" << config_.feedback_bind
              << " rate=" << config_.forward_rate_hz << "Hz\n";
    return true;
}

void GripperBridge::stop() {
    running_.store(false);
    if (recv_thread_.joinable()) recv_thread_.join();
    if (send_fd_ >= 0) { ::close(send_fd_); send_fd_ = -1; }
    if (recv_fd_ >= 0) { ::close(recv_fd_); recv_fd_ = -1; }
}

void GripperBridge::forward(const DualArmCommand& command) {
    if (!config_.enable || send_fd_ < 0) return;
    const uint64_t now = nowSteadyNs();
    if (forward_period_ns_ > 0 && now - last_forward_ns_ < forward_period_ns_) return;
    last_forward_ns_ = now;

    json msg;
    msg["schema"] = kCommandSchema;
    msg["seq"] = ++seq_;
    msg["deadman"] = true;
    msg["host_time_ns"] = now;
    bool any = false;
    if (command.left.has_gripper) {
        msg["left"] = {{"percent", command.left.gripper_target}, {"valid", true}};
        any = true;
    }
    if (command.right.has_gripper) {
        msg["right"] = {{"percent", command.right.gripper_target}, {"valid", true}};
        any = true;
    }
    if (!any) return;  // no fresh setpoint -> let gripper_server hold (stale policy)
    const std::string payload = msg.dump();
    ::sendto(send_fd_, payload.data(), payload.size(), 0,
             reinterpret_cast<const sockaddr*>(&cmd_addr_), cmd_addr_len_);
}

GripperArmFeedback GripperBridge::latest(ArmId arm) const {
    std::lock_guard<std::mutex> lock(fb_mutex_);
    const GripperArmFeedback& fb = (arm == ArmId::Left) ? left_fb_ : right_fb_;
    const uint64_t t = (arm == ArmId::Left) ? left_fb_time_ns_ : right_fb_time_ns_;
    if (t == 0) return GripperArmFeedback{};
    if (stale_timeout_ns_ > 0 && nowSteadyNs() - t > stale_timeout_ns_) {
        return GripperArmFeedback{};  // stale -> invalid
    }
    return fb;
}

void GripperBridge::receiveLoop() {
    std::vector<char> buf(4096);
    while (running_.load()) {
        const ssize_t n = ::recvfrom(recv_fd_, buf.data(), buf.size(), 0, nullptr, nullptr);
        if (n <= 0) continue;  // timeout / error -> re-check running_
        json msg = json::parse(buf.data(), buf.data() + n, nullptr, false);
        if (msg.is_discarded() || !msg.is_object()) continue;
        if (msg.value("schema", std::string()) != kStateSchema) continue;
        const uint64_t now = nowSteadyNs();
        std::lock_guard<std::mutex> lock(fb_mutex_);
        parseArmFeedback(msg, "left", &left_fb_);
        parseArmFeedback(msg, "right", &right_fb_);
        left_fb_time_ns_ = now;
        right_fb_time_ns_ = now;
    }
}

}  // namespace rb_servo
