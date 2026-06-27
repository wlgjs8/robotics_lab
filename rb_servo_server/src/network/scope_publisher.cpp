#include "rb_servo/network/scope_publisher.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <set>
#include <utility>

#include <nlohmann/json.hpp>

#include "rb_servo/network/state_publisher.hpp"

namespace rb_servo {
namespace {

constexpr const char* kScopeSchema = "robotics_lab.scope.v1";

struct UdpDestination {
    std::string endpoint;
    std::vector<char> storage;
    socklen_t len{0};

    const sockaddr* addr() const {
        return reinterpret_cast<const sockaddr*>(storage.data());
    }
};

std::chrono::nanoseconds publishPeriod(int publish_rate_hz) {
    const int rate_hz = publish_rate_hz > 0 ? publish_rate_hz : 100;
    return std::chrono::nanoseconds(1'000'000'000LL / rate_hz);
}

nlohmann::json jointArrayJson(const JointArray& joints) {
    nlohmann::json out = nlohmann::json::array();
    for (double value : joints) out.push_back(value);
    return out;
}

bool resolveUdpEndpoint(const std::string& endpoint, UdpDestination* destination) {
    std::string host;
    int port = 0;
    if (!StatePublisher::parseUdpEndpointUri(endpoint, &host, &port)) {
        std::cerr << "[ERROR] ScopePublisher only supports udp://host:port endpoints, got "
                  << endpoint << "\n";
        return false;
    }

    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;

    addrinfo* results = nullptr;
    const std::string port_string = std::to_string(port);
    const int gai = ::getaddrinfo(host.c_str(), port_string.c_str(), &hints, &results);
    if (gai != 0 || results == nullptr) {
        std::cerr << "[ERROR] ScopePublisher failed to resolve host '" << host
                  << "': " << ::gai_strerror(gai) << "\n";
        return false;
    }

    for (addrinfo* item = results; item != nullptr; item = item->ai_next) {
        if (!item->ai_addr || item->ai_addrlen <= 0) continue;
        destination->endpoint = endpoint;
        destination->storage.assign(
            reinterpret_cast<const char*>(item->ai_addr),
            reinterpret_cast<const char*>(item->ai_addr) + item->ai_addrlen
        );
        destination->len = static_cast<socklen_t>(item->ai_addrlen);
        ::freeaddrinfo(results);
        return true;
    }
    ::freeaddrinfo(results);

    std::cerr << "[ERROR] ScopePublisher found no UDP address for host '" << host << "'\n";
    return false;
}

std::vector<UdpDestination> resolveUdpDestinations(const NetworkConfig& config) {
    std::vector<UdpDestination> destinations;
    destinations.reserve(config.scope_pub_endpoints.size());
    for (const std::string& endpoint : config.scope_pub_endpoints) {
        UdpDestination destination;
        if (!resolveUdpEndpoint(endpoint, &destination)) {
            continue;
        }
        destinations.push_back(std::move(destination));
    }
    return destinations;
}

bool validateUdpEndpointSyntax(const NetworkConfig& config) {
    if (config.scope_pub_endpoints.empty()) {
        std::cerr << "[ERROR] ScopePublisher requires at least one scope_pub_endpoints entry\n";
        return false;
    }
    for (const std::string& endpoint : config.scope_pub_endpoints) {
        if (!StatePublisher::parseUdpEndpointUri(endpoint, nullptr, nullptr)) {
            std::cerr << "[ERROR] ScopePublisher only supports udp://host:port endpoints, got "
                      << endpoint << "\n";
            return false;
        }
    }
    return true;
}

}  // namespace

ScopePublisher::ScopePublisher(const ScopeConfig& scope_config, const NetworkConfig& network_config)
    : scope_config_(scope_config),
      network_config_(network_config),
      ring_(scope_config.max_samples_per_batch) {}

ScopePublisher::~ScopePublisher() {
    stop();
}

bool ScopePublisher::start() {
    if (!scope_config_.enable) return true;
    if (running_) return true;
    if (scope_config_.max_samples_per_batch == 0) {
        std::cerr << "[ERROR] ScopePublisher max_samples_per_batch must be positive\n";
        return false;
    }
    if (ring_.size() != scope_config_.max_samples_per_batch) {
        ring_.assign(scope_config_.max_samples_per_batch, ScopeSample{});
        head_ = 0;
        size_ = 0;
    }
    if (!validateUdpEndpointSyntax(network_config_)) {
        return false;
    }

    running_ = true;
    thread_ = std::thread(&ScopePublisher::threadMain, this);
    return true;
}

void ScopePublisher::stop() {
    running_ = false;
    cv_.notify_all();
    if (thread_.joinable()) {
        thread_.join();
    }
}

void ScopePublisher::push(const ScopeSample& sample) {
    if (!scope_config_.enable || !running_) return;
    {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock.owns_lock()) {
            dropped_samples_.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        if (ring_.empty()) {
            dropped_samples_.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        if (size_ >= ring_.size()) {
            head_ = (head_ + 1) % ring_.size();
            --size_;
            dropped_samples_.fetch_add(1, std::memory_order_relaxed);
        }
        const size_t write_index = (head_ + size_) % ring_.size();
        ring_[write_index] = sample;
        ++size_;
    }
    cv_.notify_one();
}

uint64_t ScopePublisher::droppedSamples() const {
    return dropped_samples_.load(std::memory_order_relaxed);
}

std::vector<ScopeSample> ScopePublisher::drainPending() {
    std::vector<ScopeSample> samples;
    std::lock_guard<std::mutex> lock(mutex_);
    samples.reserve(size_);
    for (size_t i = 0; i < size_; ++i) {
        samples.push_back(ring_[(head_ + i) % ring_.size()]);
    }
    head_ = 0;
    size_ = 0;
    return samples;
}

std::string ScopePublisher::serializeBatch(const std::vector<ScopeSample>& samples) {
    nlohmann::json t_host_ns = nlohmann::json::array();
    nlohmann::json left_t_robot_ns = nlohmann::json::array();
    nlohmann::json right_t_robot_ns = nlohmann::json::array();
    nlohmann::json left_q_sent = nlohmann::json::array();
    nlohmann::json right_q_sent = nlohmann::json::array();
    nlohmann::json left_q_ref = nlohmann::json::array();
    nlohmann::json right_q_ref = nlohmann::json::array();
    nlohmann::json left_q_actual = nlohmann::json::array();
    nlohmann::json right_q_actual = nlohmann::json::array();

    for (const ScopeSample& sample : samples) {
        t_host_ns.push_back(sample.t_host_ns);
        left_t_robot_ns.push_back(sample.l_robot_ns);
        right_t_robot_ns.push_back(sample.r_robot_ns);
        left_q_sent.push_back(jointArrayJson(sample.l_sent));
        right_q_sent.push_back(jointArrayJson(sample.r_sent));
        left_q_ref.push_back(jointArrayJson(sample.l_ref));
        right_q_ref.push_back(jointArrayJson(sample.r_ref));
        left_q_actual.push_back(jointArrayJson(sample.l_actual));
        right_q_actual.push_back(jointArrayJson(sample.r_actual));
    }

    nlohmann::json message = {
        {"schema", kScopeSchema},
        {"n", samples.size()},
        {"t_host_ns", std::move(t_host_ns)},
        {"left", {
            {"t_robot_ns", std::move(left_t_robot_ns)},
            {"q_sent", std::move(left_q_sent)},
            {"q_ref", std::move(left_q_ref)},
            {"q_actual", std::move(left_q_actual)},
        }},
        {"right", {
            {"t_robot_ns", std::move(right_t_robot_ns)},
            {"q_sent", std::move(right_q_sent)},
            {"q_ref", std::move(right_q_ref)},
            {"q_actual", std::move(right_q_actual)},
        }},
    };
    return message.dump();
}

void ScopePublisher::threadMain() {
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        std::cerr << "[ERROR] ScopePublisher socket failed: " << std::strerror(errno) << "\n";
        running_ = false;
        return;
    }

    const std::vector<UdpDestination> destinations = resolveUdpDestinations(network_config_);
    if (destinations.empty()) {
        std::cerr << "[ERROR] ScopePublisher resolved no UDP destinations\n";
        ::close(fd);
        running_ = false;
        return;
    }

    const auto period = publishPeriod(scope_config_.publish_rate_hz);
    std::set<std::string> send_warned_endpoints;
    while (running_) {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait_for(lock, period, [&] {
                return !running_.load(std::memory_order_relaxed);
            });
        }
        if (!running_) break;

        std::vector<ScopeSample> samples = drainPending();
        if (samples.empty()) {
            continue;
        }

        const std::string payload = serializeBatch(samples);
        for (const UdpDestination& destination : destinations) {
            const ssize_t sent = ::sendto(
                fd,
                payload.data(),
                payload.size(),
                0,
                destination.addr(),
                destination.len
            );
            if (sent < 0 && send_warned_endpoints.insert(destination.endpoint).second) {
                std::cerr << "[WARN] ScopePublisher send failed to "
                          << destination.endpoint << ": " << std::strerror(errno) << "\n";
            }
        }
    }

    ::close(fd);
}

}  // namespace rb_servo
