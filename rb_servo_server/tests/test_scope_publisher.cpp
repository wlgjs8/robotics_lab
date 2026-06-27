#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

#include "rb_servo/network/scope_publisher.hpp"

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

bool bindLoopbackUdp(UdpSocket* out, int timeout_ms = 1000) {
    out->fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (out->fd < 0) {
        std::cerr << "SKIP bindLoopbackUdp: socket failed: " << std::strerror(errno) << "\n";
        return false;
    }

    timeval timeout{};
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;
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

rb_servo::ScopeSample sampleWithIndex(uint64_t index) {
    rb_servo::ScopeSample sample;
    sample.t_host_ns = 1'000'000'000ULL + index * 2'000'000ULL;
    sample.l_robot_ns = 2'000'000'000ULL + index;
    sample.r_robot_ns = 3'000'000'000ULL + index;
    for (int joint = 0; joint < rb_servo::kDof; ++joint) {
        const double base = static_cast<double>(index * 10 + static_cast<uint64_t>(joint));
        sample.l_sent[static_cast<size_t>(joint)] = base + 0.1;
        sample.r_sent[static_cast<size_t>(joint)] = base + 0.2;
        sample.l_ref[static_cast<size_t>(joint)] = base + 0.3;
        sample.r_ref[static_cast<size_t>(joint)] = base + 0.4;
        sample.l_actual[static_cast<size_t>(joint)] = base + 0.5;
        sample.r_actual[static_cast<size_t>(joint)] = base + 0.6;
    }
    return sample;
}

rb_servo::ScopeConfig enabledScopeConfig(int publish_rate_hz = 100, size_t max_samples = 64) {
    rb_servo::ScopeConfig scope;
    scope.enable = true;
    scope.publish_rate_hz = publish_rate_hz;
    scope.max_samples_per_batch = max_samples;
    return scope;
}

bool testScopePublisherSerializesCompactBatch() {
    const std::vector<rb_servo::ScopeSample> samples = {
        sampleWithIndex(0),
        sampleWithIndex(1),
    };
    const nlohmann::json json = nlohmann::json::parse(rb_servo::ScopePublisher::serializeBatch(samples));
    RB_CHECK(json.at("schema").get<std::string>() == "robotics_lab.scope.v1");
    RB_CHECK(json.at("n").get<size_t>() == 2);
    RB_CHECK(json.at("t_host_ns").size() == 2);
    RB_CHECK(json.at("left").at("t_robot_ns").at(1).get<uint64_t>() == samples[1].l_robot_ns);
    RB_CHECK(json.at("right").at("t_robot_ns").at(1).get<uint64_t>() == samples[1].r_robot_ns);
    RB_CHECK(json.at("left").at("q_sent").size() == 2);
    RB_CHECK(json.at("left").at("q_sent").at(0).size() == rb_servo::kDof);
    RB_CHECK(json.at("left").at("q_sent").at(0).at(0).get<double>() == samples[0].l_sent[0]);
    RB_CHECK(json.at("right").at("q_ref").at(1).at(5).get<double>() == samples[1].r_ref[5]);
    RB_CHECK(json.at("left").at("q_actual").at(1).at(2).get<double>() == samples[1].l_actual[2]);
    return true;
}

bool testScopePublisherSendsOneBatchToEachEndpoint() {
    UdpSocket first;
    UdpSocket second;
    if (!bindLoopbackUdp(&first) || !bindLoopbackUdp(&second)) {
        return true;
    }

    rb_servo::NetworkConfig network;
    network.scope_pub_endpoints = {endpointFor(first), endpointFor(second)};
    rb_servo::ScopePublisher publisher(enabledScopeConfig(), network);
    RB_CHECK(publisher.start());
    for (uint64_t i = 0; i < 5; ++i) {
        publisher.push(sampleWithIndex(i));
    }

    std::string first_payload;
    std::string second_payload;
    const bool first_received = receivePacket(first.fd, &first_payload);
    const bool second_received = receivePacket(second.fd, &second_payload);
    publisher.stop();

    RB_CHECK(first_received);
    RB_CHECK(second_received);
    RB_CHECK(first_payload == second_payload);
    const nlohmann::json json = nlohmann::json::parse(first_payload);
    RB_CHECK(json.at("n").get<size_t>() == 5);
    RB_CHECK(json.at("t_host_ns").at(1).get<uint64_t>() - json.at("t_host_ns").at(0).get<uint64_t>() == 2'000'000ULL);
    return true;
}

bool testScopePublisherDropsOldestAtBatchLimit() {
    UdpSocket sink;
    if (!bindLoopbackUdp(&sink)) {
        return true;
    }

    rb_servo::NetworkConfig network;
    network.scope_pub_endpoints = {endpointFor(sink)};
    rb_servo::ScopePublisher publisher(enabledScopeConfig(100, 3), network);
    RB_CHECK(publisher.start());
    for (uint64_t i = 0; i < 5; ++i) {
        publisher.push(sampleWithIndex(i));
    }
    RB_CHECK(publisher.droppedSamples() == 2);

    std::string payload;
    const bool received = receivePacket(sink.fd, &payload);
    publisher.stop();

    RB_CHECK(received);
    const nlohmann::json json = nlohmann::json::parse(payload);
    RB_CHECK(json.at("n").get<size_t>() == 3);
    RB_CHECK(json.at("t_host_ns").at(0).get<uint64_t>() == sampleWithIndex(2).t_host_ns);
    RB_CHECK(json.at("t_host_ns").at(2).get<uint64_t>() == sampleWithIndex(4).t_host_ns);
    return true;
}

bool testScopePublisherSkipsEmptyBatch() {
    UdpSocket sink;
    if (!bindLoopbackUdp(&sink, 200)) {
        return true;
    }

    rb_servo::NetworkConfig network;
    network.scope_pub_endpoints = {endpointFor(sink)};
    rb_servo::ScopePublisher publisher(enabledScopeConfig(100, 64), network);
    RB_CHECK(publisher.start());

    std::string payload;
    const bool received = receivePacket(sink.fd, &payload);
    publisher.stop();

    RB_CHECK(!received);
    return true;
}

}  // namespace

int main() {
    if (!testScopePublisherSerializesCompactBatch()) return 1;
    if (!testScopePublisherSendsOneBatchToEachEndpoint()) return 1;
    if (!testScopePublisherDropsOldestAtBatchLimit()) return 1;
    if (!testScopePublisherSkipsEmptyBatch()) return 1;
    return 0;
}
