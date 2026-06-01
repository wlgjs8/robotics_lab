#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>

#include "rb_servo/network/state_publisher.hpp"

namespace {

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

struct UdpSocket {
    int fd = -1;
    int port = 0;

    ~UdpSocket() {
        if (fd >= 0) {
            ::close(fd);
        }
    }
};

bool bindLoopbackUdp(UdpSocket* out) {
    out->fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (out->fd < 0) {
        std::cerr << "SKIP bindLoopbackUdp: socket failed: " << std::strerror(errno) << "\n";
        return false;
    }

    timeval timeout{};
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    ::setsockopt(out->fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    if (::bind(out->fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        std::cerr << "SKIP bindLoopbackUdp: bind failed: " << std::strerror(errno) << "\n";
        return false;
    }

    socklen_t len = sizeof(addr);
    if (::getsockname(out->fd, reinterpret_cast<sockaddr*>(&addr), &len) != 0) {
        std::cerr << "SKIP bindLoopbackUdp: getsockname failed: " << std::strerror(errno) << "\n";
        return false;
    }
    out->port = ntohs(addr.sin_port);
    return out->port > 0;
}

std::string endpointFor(const UdpSocket& socket) {
    return "udp://127.0.0.1:" + std::to_string(socket.port);
}

bool receivePacket(int fd, std::string* payload) {
    char buffer[65536];
    const ssize_t count = ::recv(fd, buffer, sizeof(buffer), 0);
    if (count <= 0) return false;
    payload->assign(buffer, buffer + count);
    return true;
}

rb_servo::ServoSnapshot snapshotWithTick(uint64_t tick) {
    rb_servo::ServoSnapshot snapshot;
    snapshot.tick = tick;
    snapshot.loop_start_time_ns = 1000;
    snapshot.loop_end_time_ns = 2000;
    return snapshot;
}

bool testStatePublisherFanoutSendsSamePayloadToTwoSockets() {
    UdpSocket recorder;
    UdpSocket gui;
    if (!bindLoopbackUdp(&recorder) || !bindLoopbackUdp(&gui)) {
        return true;
    }

    rb_servo::DualArmConfig cfg;
    cfg.network.state_pub_endpoint = endpointFor(recorder);
    cfg.network.state_pub_bind = cfg.network.state_pub_endpoint;
    cfg.network.state_pub_endpoints = {endpointFor(recorder), endpointFor(gui)};
    cfg.network.state_pub_rate_hz = 100;

    rb_servo::StatePublisher publisher(cfg, []() {
        return snapshotWithTick(42);
    });
    RB_CHECK(publisher.start());

    std::string recorder_payload;
    std::string gui_payload;
    const bool recorder_received = receivePacket(recorder.fd, &recorder_payload);
    const bool gui_received = receivePacket(gui.fd, &gui_payload);
    publisher.stop();

    RB_CHECK(recorder_received);
    RB_CHECK(gui_received);
    RB_CHECK(recorder_payload == gui_payload);
    RB_CHECK(recorder_payload.find("\"tick\":42") != std::string::npos);
    return true;
}

bool testStatePublisherLegacySingleEndpointStillWorks() {
    UdpSocket sink;
    if (!bindLoopbackUdp(&sink)) {
        return true;
    }

    rb_servo::NetworkConfig network;
    network.state_pub_endpoint = endpointFor(sink);
    network.state_pub_bind = network.state_pub_endpoint;
    network.state_pub_endpoints = {network.state_pub_endpoint};
    network.state_pub_rate_hz = 100;

    rb_servo::StatePublisher publisher(network);
    publisher.updateSnapshot(snapshotWithTick(7));
    RB_CHECK(publisher.start());

    std::string payload;
    const bool received = receivePacket(sink.fd, &payload);
    publisher.stop();

    RB_CHECK(received);
    RB_CHECK(payload.find("\"tick\":7") != std::string::npos);
    return true;
}

bool testStatePublisherSerializesJointReferenceFields() {
    rb_servo::ServoSnapshot snapshot = snapshotWithTick(99);
    snapshot.left_state.arm_id = rb_servo::ArmId::Left;
    snapshot.left_state.q_actual_deg = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    snapshot.left_state.q_target_deg = {7.0, 8.0, 9.0, 10.0, 11.0, 12.0};
    snapshot.left_state.q_actual_valid = true;
    snapshot.left_state.q_ref_valid = true;
    snapshot.left_state.has_valid_joint_state = true;
    snapshot.left_state.q_ref_source = "rbpodo.sdata.jnt_ref";
    snapshot.left_state.rbpodo_sdk_state_source = "CobotData.request_data";
    snapshot.left_state.rbpodo_state_decode_policy =
        "strict_boolean_flags_with_suspect_large_values";

    rb_servo::DualArmConfig cfg;
    cfg.left_robot.backend_type = rb_servo::BackendType::Rbpodo;
    rb_servo::StatePublisher publisher(cfg);
    const nlohmann::json json = nlohmann::json::parse(publisher.serializeSnapshot(snapshot));
    const nlohmann::json& left = json.at("left");

    RB_CHECK(left.at("q_actual_deg").at(0).get<double>() == 1.0);
    RB_CHECK(left.at("q_target_deg").at(0).get<double>() == 7.0);
    RB_CHECK(left.at("q_ref_deg").at(0).get<double>() == 7.0);
    RB_CHECK(left.at("q_actual_valid").get<bool>());
    RB_CHECK(left.at("q_ref_valid").get<bool>());
    RB_CHECK(left.at("q_ref_source").get<std::string>() == "rbpodo.sdata.jnt_ref");
    RB_CHECK(left.at("rbpodo_sdk_state_source").get<std::string>() == "CobotData.request_data");
    RB_CHECK(
        left.at("rbpodo_state_decode_policy").get<std::string>() ==
        "strict_boolean_flags_with_suspect_large_values"
    );
    return true;
}

}  // namespace

int main() {
    if (!testStatePublisherFanoutSendsSamePayloadToTwoSockets()) return 1;
    if (!testStatePublisherLegacySingleEndpointStillWorks()) return 1;
    if (!testStatePublisherSerializesJointReferenceFields()) return 1;
    return 0;
}
