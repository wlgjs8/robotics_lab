#include "rb_servo/network/chunk_frame_receiver.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace rb_servo {
namespace {

using json = nlohmann::json;

struct UdpEndpoint {
    std::string host = "0.0.0.0";
    int port = 0;
};

UdpEndpoint parseUdpUri(const std::string& uri) {
    const std::string prefix = "udp://";
    if (uri.rfind(prefix, 0) != 0) {
        throw std::runtime_error("Only udp:// chunk_frame_bind is supported: " + uri);
    }
    const std::string rest = uri.substr(prefix.size());
    const auto colon = rest.rfind(':');
    if (colon == std::string::npos) {
        throw std::runtime_error("Invalid udp uri: " + uri);
    }
    UdpEndpoint ep;
    ep.host = rest.substr(0, colon);
    ep.port = std::stoi(rest.substr(colon + 1));
    return ep;
}

bool parseArm(const json& arm_node, ChunkFrameReceiver::ArmSteps* out) {
    if (arm_node.is_null()) return false;
    if (!arm_node.is_array()) return false;
    const int n = static_cast<int>(arm_node.size());
    if (n < 1) return false;
    const int count = std::min(n, ChunkFrameReceiver::kMaxSteps);
    for (int i = 0; i < count; ++i) {
        const json& row = arm_node[i];
        if (!row.is_array() || row.size() < ChunkFrameReceiver::kStepDims) return false;
        for (int d = 0; d < ChunkFrameReceiver::kStepDims; ++d) {
            if (!row[d].is_number()) return false;
            const double v = row[d].get<double>();
            if (!std::isfinite(v)) return false;
            out->step[i][d] = v;
        }
        // quaternion must be non-degenerate (producer emits normalized xyzw).
        const double qn = std::sqrt(
            out->step[i][3] * out->step[i][3] + out->step[i][4] * out->step[i][4] +
            out->step[i][5] * out->step[i][5] + out->step[i][6] * out->step[i][6]);
        if (qn < 1e-6) return false;
    }
    out->count = count;
    return true;
}

}  // namespace

ChunkFrameReceiver::ChunkFrameReceiver(const std::string& bind_uri) : bind_uri_(bind_uri) {}

ChunkFrameReceiver::~ChunkFrameReceiver() { stop(); }

double ChunkFrameReceiver::steadyNowSec() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool ChunkFrameReceiver::parsePacket(const char* data, std::size_t size, Frame* out) {
    if (!out) return false;
    json packet;
    try {
        packet = json::parse(data, data + size);
    } catch (const json::exception&) {
        return false;
    }
    if (!packet.is_object()) return false;
    // Accept the overlay schema (and a future chunk_frame alias).
    const auto schema_it = packet.find("schema_version");
    if (schema_it == packet.end() || !schema_it->is_string()) return false;
    const std::string schema = schema_it->get<std::string>();
    if (schema != "robotics_lab.chunk_overlay.v2" &&
        schema != "robotics_lab.chunk_frame.v1") {
        return false;
    }
    Frame frame;
    const auto seq_it = packet.find("seq");
    if (seq_it == packet.end() || !seq_it->is_number()) return false;
    frame.seq = seq_it->get<std::uint64_t>();
    const auto dt_it = packet.find("policy_dt_sec");
    if (dt_it == packet.end() || !dt_it->is_number()) return false;
    frame.policy_dt_sec = dt_it->get<double>();
    if (!std::isfinite(frame.policy_dt_sec) || frame.policy_dt_sec <= 1e-4) return false;

    const auto left_it = packet.find("left");
    if (left_it != packet.end()) frame.has_left = parseArm(*left_it, &frame.left);
    const auto right_it = packet.find("right");
    if (right_it != packet.end()) frame.has_right = parseArm(*right_it, &frame.right);
    if (!frame.has_left && !frame.has_right) return false;

    frame.recv_steady_sec = steadyNowSec();
    *out = frame;
    return true;
}

bool ChunkFrameReceiver::start() {
    if (bind_uri_.empty()) return true;  // disabled
    UdpEndpoint ep;
    try {
        ep = parseUdpUri(bind_uri_);
    } catch (const std::exception& e) {
        std::cerr << "[chunk_frame] invalid bind uri: " << e.what() << "\n";
        return false;
    }
    socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        std::cerr << "[chunk_frame] socket() failed: " << std::strerror(errno) << "\n";
        return false;
    }
    int reuse = 1;
    ::setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(ep.port));
    if (::inet_pton(AF_INET, ep.host.c_str(), &addr.sin_addr) != 1) {
        std::cerr << "[chunk_frame] invalid bind host: " << ep.host << "\n";
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }
    if (::bind(socket_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        std::cerr << "[chunk_frame] bind(" << bind_uri_ << ") failed: "
                  << std::strerror(errno) << "\n";
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }
    running_.store(true, std::memory_order_release);
    thread_ = std::thread(&ChunkFrameReceiver::threadMain, this);
    std::cout << "[chunk_frame] listening on " << bind_uri_ << "\n";
    return true;
}

void ChunkFrameReceiver::stop() {
    running_.store(false, std::memory_order_release);
    if (socket_fd_ >= 0) {
        ::shutdown(socket_fd_, SHUT_RDWR);
    }
    if (thread_.joinable()) thread_.join();
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
}

bool ChunkFrameReceiver::copyLatest(Frame* out) const {
    if (!out) return false;
    if (latest_seq_.load(std::memory_order_acquire) == 0) return false;
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock()) return false;  // writer mid-store; retry next tick
    *out = latest_;
    return true;
}

void ChunkFrameReceiver::threadMain() {
    std::array<char, 65536> buffer;
    while (running_.load(std::memory_order_acquire)) {
        fd_set read_set;
        FD_ZERO(&read_set);
        FD_SET(socket_fd_, &read_set);
        timeval tv{0, 200 * 1000};  // 200 ms poll for shutdown
        const int ready = ::select(socket_fd_ + 1, &read_set, nullptr, nullptr, &tv);
        if (ready <= 0) continue;
        const ssize_t received = ::recvfrom(
            socket_fd_, buffer.data(), buffer.size() - 1, 0, nullptr, nullptr);
        if (received <= 0) continue;
        Frame frame;
        if (!parsePacket(buffer.data(), static_cast<std::size_t>(received), &frame)) continue;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            // receiver_seq is assigned UNDER the same lock the reader copies
            // under, so a copied frame always carries its own consistent seq
            // (the atomic latest_seq_ is only a cheap "anything new?" gate).
            frame.receiver_seq = latest_seq_.load(std::memory_order_relaxed) + 1;
            latest_ = frame;
        }
        latest_seq_.fetch_add(1, std::memory_order_acq_rel);
    }
}

}  // namespace rb_servo
