#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <atomic>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include <nlohmann/json.hpp>

#include "rb_servo/control/realtime_timing.hpp"
#include "rb_servo/network/state_publisher.hpp"
#include "state_publication_precision_fixture.hpp"

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

bool testStatePublisherSerializesAsyncStreamingFields() {
    rb_servo::ServoSnapshot snapshot = snapshotWithTick(123);
    snapshot.left_last_send.acceptance_semantics = "controller_ack_observed";
    snapshot.left_async_streaming.commands_enqueued_total = 10;
    snapshot.left_async_streaming.commands_sent_total = 9;
    snapshot.left_async_streaming.commands_socket_sent_total = 9;
    snapshot.left_async_streaming.commands_dropped_total = 1;
    snapshot.left_async_streaming.commands_overwritten_total = 1;
    snapshot.left_async_streaming.ack_timeout_count = 2;
    snapshot.left_async_streaming.missing_ack_count = 3;
    snapshot.left_async_streaming.q_ref_watchdog_miss_count = 4;
    snapshot.left_async_streaming.tcp_ref_watchdog_miss_count = 5;
    snapshot.left_async_streaming.last_command_seq = 99;
    snapshot.left_async_streaming.last_ack_seq = 88;
    snapshot.left_async_streaming.last_q_ref_update_host_time_ns = 777;
    snapshot.left_async_streaming.last_tcp_ref_update_host_time_ns = 778;
    snapshot.left_async_streaming.last_socket_send_host_time_ns = 888;
    snapshot.left_async_streaming.q_ref_update_age_ms = 12.5;
    snapshot.left_async_streaming.tcp_ref_update_age_ms = 13.5;
    snapshot.left_async_streaming.q_ref_target_error_deg_max = 1.25;
    snapshot.left_async_streaming.tcp_ref_target_error_m = 0.031;
    snapshot.left_async_streaming.last_controller_acceptance_semantics = "controller_ack_observed";
    snapshot.left_async_streaming.worker_backlog = 6;
    snapshot.left_async_streaming.max_pending_age_ms_observed = 7.5;
    snapshot.left_async_streaming.supervision_state =
        rb_servo::RbpodoAsyncStreamingSupervisionState::Warning;
    snapshot.left_async_streaming.reference_supervision_state =
        rb_servo::RbpodoAsyncStreamingSupervisionState::Fault;
    snapshot.left_async_streaming.reference_supervision_reason = "async_q_ref_target_error";
    snapshot.left_async_streaming.reference_supervision_fault_count = 2;
    snapshot.async_supervision_degraded = true;

    rb_servo::DualArmConfig cfg;
    cfg.servo.rbpodo_async_streaming.enable = true;
    cfg.servo.rbpodo_async_streaming.mode =
        rb_servo::RbpodoAsyncStreamingMode::SocketSendSupervised;
    cfg.servo.rbpodo_async_streaming.queue_policy =
        rb_servo::RbpodoAsyncQueuePolicy::LatestWins;

    rb_servo::StatePublisher publisher(cfg);
    const nlohmann::json json = nlohmann::json::parse(publisher.serializeSnapshot(snapshot));
    const nlohmann::json& async = json.at("left").at("async_streaming");

    RB_CHECK(json.at("async_streaming_enabled").get<bool>());
    RB_CHECK(json.at("async_supervision_degraded").get<bool>());
    RB_CHECK(json.at("async_streaming_mode").get<std::string>() == "socket_send_supervised");
    RB_CHECK(json.at("async_streaming_policy").get<std::string>() == "latest_wins");
    RB_CHECK(async.at("enabled").get<bool>());
    RB_CHECK(async.at("mode").get<std::string>() == "socket_send_supervised");
    RB_CHECK(async.at("queue_policy").get<std::string>() == "latest_wins");
    RB_CHECK(async.at("commands_enqueued_total").get<uint64_t>() == 10);
    RB_CHECK(async.at("commands_sent_total").get<uint64_t>() == 9);
    RB_CHECK(async.at("commands_socket_sent_total").get<uint64_t>() == 9);
    RB_CHECK(async.at("commands_dropped_total").get<uint64_t>() == 1);
    RB_CHECK(async.at("commands_overwritten_total").get<uint64_t>() == 1);
    RB_CHECK(async.at("ack_timeout_count").get<uint64_t>() == 2);
    RB_CHECK(async.at("missing_ack_count").get<uint64_t>() == 3);
    RB_CHECK(async.at("q_ref_watchdog_miss_count").get<uint64_t>() == 4);
    RB_CHECK(async.at("tcp_ref_watchdog_miss_count").get<uint64_t>() == 5);
    RB_CHECK(async.at("last_command_seq").get<uint64_t>() == 99);
    RB_CHECK(async.at("last_ack_seq").get<uint64_t>() == 88);
    RB_CHECK(async.at("last_q_ref_update_host_time_ns").get<uint64_t>() == 777);
    RB_CHECK(async.at("last_tcp_ref_update_host_time_ns").get<uint64_t>() == 778);
    RB_CHECK(async.at("last_socket_send_host_time_ns").get<uint64_t>() == 888);
    RB_CHECK(async.at("q_ref_update_age_ms").get<double>() == 12.5);
    RB_CHECK(async.at("tcp_ref_update_age_ms").get<double>() == 13.5);
    RB_CHECK(async.at("q_ref_target_error_deg_max").get<double>() == 1.25);
    RB_CHECK(async.at("tcp_ref_target_error_m").get<double>() == 0.031);
    RB_CHECK(async.at("last_controller_acceptance_semantics").get<std::string>() == "socket_send_only");
    RB_CHECK(async.at("last_controller_acceptance_semantics").get<std::string>() != "controller_ack_observed");
    RB_CHECK(async.at("worker_backlog").get<uint64_t>() == 6);
    RB_CHECK(async.at("max_pending_age_ms_observed").get<double>() == 7.5);
    RB_CHECK(async.at("supervision_state").get<std::string>() == "warning");
    RB_CHECK(async.at("reference_supervision_state").get<std::string>() == "fault");
    RB_CHECK(async.at("reference_supervision_reason").get<std::string>() == "async_q_ref_target_error");
    RB_CHECK(async.at("reference_supervision_fault_count").get<uint64_t>() == 2);
    return true;
}

bool testRealtimeTimingAccumulatorAndSerialization() {
    rb_servo::RealtimeTimingAccumulator accumulator;
    constexpr uint64_t kBaseNs = 1'000'000'000ULL;
    constexpr uint64_t kPeriodNs = 2'000'000ULL;
    uint64_t left_host_ns = kBaseNs;
    uint64_t right_host_ns = kBaseNs;
    for (uint64_t i = 0; i < 500; ++i) {
        rb_servo::RealtimeTimingTick tick;
        tick.scheduled_wake_ns = kBaseNs + i * kPeriodNs;
        tick.loop_start_ns = tick.scheduled_wake_ns + 100'000ULL;
        tick.loop_end_ns = tick.loop_start_ns + 1'000'000ULL;
        if (i == 100) tick.loop_end_ns = tick.scheduled_wake_ns + 2'100'000ULL;
        tick.previous_sleep_enter_ns = i == 200
            ? tick.scheduled_wake_ns : tick.scheduled_wake_ns - 500'000ULL;
        tick.nominal_period_ns = kPeriodNs;
        tick.send_cycle = true;
        tick.pre_send_ns = 300'000ULL;
        tick.send_duration_ns = 150'000ULL;
        if (i != 250) left_host_ns = tick.loop_start_ns + 200'000ULL;
        tick.left_feedback.host_time_ns = left_host_ns;
        tick.left_feedback.robot_time_ns = i * kPeriodNs + 1;
        if ((i % 2) == 0) right_host_ns = tick.loop_start_ns + 400'000ULL;
        tick.right_feedback.host_time_ns = right_host_ns;
        tick.right_feedback.robot_time_ns = (i / 2) * 2 * kPeriodNs + 1;
        accumulator.add(tick);
    }

    const rb_servo::RealtimeTimingTelemetry timing = accumulator.snapshot();
    RB_CHECK(timing.window_sec > 0.99 && timing.window_sec <= 1.001);
    RB_CHECK(timing.servo.target_rate_hz == 500.0);
    RB_CHECK(timing.servo.observed_rate_hz > 499.0 && timing.servo.observed_rate_hz < 501.0);
    RB_CHECK(timing.servo.send_rate_hz > 499.0 && timing.servo.send_rate_hz < 501.0);
    RB_CHECK(timing.servo.period_ms.last == 2.0);
    RB_CHECK(timing.servo.wake_latency_us.last == 100.0);
    RB_CHECK(timing.servo.pre_send_us.last == 300.0);
    RB_CHECK(timing.servo.send_duration_us.last == 150.0);
    RB_CHECK(timing.servo.deadline_miss_count == 1);
    RB_CHECK(timing.servo.catch_up_count == 1);
    RB_CHECK(timing.left_feedback.frame_rate_hz > 498.0);
    RB_CHECK(timing.left_feedback.frame_rate_hz < 500.0);
    RB_CHECK(timing.left_feedback.held_count == 1);
    RB_CHECK(timing.right_feedback.fresh_rate_hz > 249.0);
    RB_CHECK(timing.right_feedback.fresh_rate_hz < 251.0);
    RB_CHECK(timing.right_feedback.held_count == 250);
    RB_CHECK(timing.right_feedback.period_ms.last == 4.0);
    RB_CHECK(!timing.left_feedback.freshness_reliable);
    RB_CHECK(timing.left_feedback.robot_time_available);
    RB_CHECK(timing.left_feedback.robot_time_monotonic);

    rb_servo::ServoSnapshot snapshot = snapshotWithTick(125);
    rb_servo::StatePublisher publisher(rb_servo::DualArmConfig{});
    const nlohmann::json absent =
        nlohmann::json::parse(publisher.serializeSnapshot(snapshot));
    RB_CHECK(!absent.contains("realtime_timing"));

    snapshot.realtime_timing = timing;
    const nlohmann::json json =
        nlohmann::json::parse(publisher.serializeSnapshot(snapshot));
    const nlohmann::json& realtime = json.at("realtime_timing");
    RB_CHECK(realtime.at("servo").at("target_rate_hz").get<double>() == 500.0);
    RB_CHECK(realtime.at("servo").at("period_ms").contains("p95"));
    RB_CHECK(realtime.at("servo").at("wake_latency_us").contains("max"));
    RB_CHECK(realtime.at("servo").at("pre_send_us").at("last").get<double>() == 300.0);
    RB_CHECK(realtime.at("servo").at("send_duration_us").at("last").get<double>() == 150.0);
    const nlohmann::json& feedback = realtime.at("feedback").at("right");
    RB_CHECK(feedback.at("held_count").get<uint64_t>() == 250);
    RB_CHECK(feedback.at("frame_rate_basis").get<std::string>() ==
             "host_frame_timestamp_change");
    RB_CHECK(feedback.at("freshness_basis").get<std::string>() ==
             "controller_robot_time_diagnostic_only");
    RB_CHECK(feedback.at("robot_time_trust").get<std::string>() == "diagnostic_only");
    return true;
}

bool testStatePublisherKeepsForceTelemetryInsideTheArmObjects() {
    // A REGRESSION GUARD ON ASSIGNMENT ORDER, not on the wrench numbers.
    // `message["left"]["force_torque"] = ...` followed by `message["left"] = armStateJson(...)`
    // REPLACES the arm object, so the F/T block is silently dropped: the pipeline runs,
    // the tare is accepted, the servo log carries the wrench, and every consumer of the
    // state stream still sees no `force_torque` key at all. Nothing else in this file
    // reaches into `json["left"]` after the arm object is built, so nothing else caught it.
    rb_servo::ServoSnapshot snapshot = snapshotWithTick(7);
    snapshot.left_ft.enabled = true;
    snapshot.left_ft.connected = true;
    snapshot.left_ft.bias_valid = true;
    snapshot.left_ft.comp_stand = rb_servo::Wrench6D{1.0, 2.0, 3.0, 0.1, 0.2, 0.3};
    snapshot.right_ft.enabled = true;
    snapshot.right_ft.connected = true;
    snapshot.right_force_control.enabled = true;
    snapshot.left_force_control.enabled = true;
    snapshot.left_force_control.covered = true;

    rb_servo::DualArmConfig cfg;
    rb_servo::StatePublisher publisher(cfg);
    const nlohmann::json json = nlohmann::json::parse(publisher.serializeSnapshot(snapshot));

    for (const char* arm : {"left", "right"}) {
        RB_CHECK(json.at(arm).contains("force_torque"));
        RB_CHECK(json.at(arm).contains("force_control"));
        RB_CHECK(json.at(arm).at("force_torque").at("enabled").get<bool>());
        // The arm object itself must survive too — the fix must not have traded one
        // wholesale overwrite for the other.
        RB_CHECK(json.at(arm).contains("q_actual_deg"));
    }
    const nlohmann::json& w = json.at("left").at("force_torque").at("comp_stand_axes_at_tcp");
    RB_CHECK(w.size() == 6);
    RB_CHECK(w.at(0).get<double>() == 1.0);
    RB_CHECK(w.at(5).get<double>() == 0.3);
    RB_CHECK(json.at("left").at("force_control").at("covered").get<bool>());
    return true;
}

bool testStatePublisherPreservesForceReferenceWhenUncovered() {
    rb_servo::ServoSnapshot snapshot = snapshotWithTick(8);
    snapshot.left_force_control.covered = false;
    snapshot.left_force_control.reference_deviation_m = {0.011, -0.022, 0.033};
    snapshot.left_force_control.reference_deviation_rad = {-0.044, 0.055, -0.066};
    snapshot.left_force_control.reference_strip_enabled = false;
    snapshot.left_force_control.reference_reset_count = 9'007'199'254'740'993ULL;
    snapshot.right_force_control.covered = true;
    snapshot.right_force_control.reference_deviation_m = {-0.071, 0.082, -0.093};
    snapshot.right_force_control.reference_deviation_rad = {0.104, -0.115, 0.126};
    snapshot.right_force_control.reference_strip_enabled = true;
    snapshot.right_force_control.reference_reset_count = 7;

    rb_servo::StatePublisher publisher(rb_servo::DualArmConfig{});
    const nlohmann::json json = nlohmann::json::parse(publisher.serializeSnapshot(snapshot));
    for (const char* arm : {"left", "right"}) {
        const auto& fc = json.at(arm).at("force_control");
        const auto& expected = std::string(arm) == "left"
            ? snapshot.left_force_control : snapshot.right_force_control;
        RB_CHECK(fc.at("reference_deviation_stand_m").get<std::vector<double>>() ==
                 std::vector<double>(expected.reference_deviation_m.begin(), expected.reference_deviation_m.end()));
        RB_CHECK(fc.at("reference_deviation_stand_rad").get<std::vector<double>>() ==
                 std::vector<double>(expected.reference_deviation_rad.begin(), expected.reference_deviation_rad.end()));
        RB_CHECK(fc.at("reference_strip_enabled").get<bool>() == expected.reference_strip_enabled);
        RB_CHECK(fc.at("reference_reset_count").is_number_unsigned());
        RB_CHECK(fc.at("reference_reset_count").get<uint64_t>() == expected.reference_reset_count);
    }
    const auto& left = json.at("left").at("force_control");
    RB_CHECK(!left.at("covered").get<bool>());
    RB_CHECK(left.at("deviation_stand_m").get<std::vector<double>>() == std::vector<double>(3, 0.0));
    RB_CHECK(left.at("deviation_stand_rad").get<std::vector<double>>() == std::vector<double>(3, 0.0));
    return true;
}

bool testStatePublisherSerializesPerPairSelfCollisionBands() {
    // The near list is ordered by RAW clearance, so a consumer cannot tell which pairs
    // are in hard violation unless each pair carries its OWN floor. Measured on the RB5
    // (2026-09-06): the structural intra-arm link3<->link5 pair sits at ~23 mm against a
    // 5 mm floor and is near[0] on 99.4% of ticks, while a gripper<->gripper pair at
    // 8.5 mm IS violating its 25 mm floor. Banding both against the single 40 mm self
    // d_hard_m calls the structural pair a violation and can name the wrong parts.
    rb_servo::ServoSnapshot snapshot = snapshotWithTick(321);
    snapshot.self_collision_mesh = true;
    rb_servo::SelfCollisionNearPairViz intra;
    intra.name_a = "dual_rb5_850e_left_link3_1";
    intra.name_b = "dual_rb5_850e_left_link5_0";
    intra.clearance_m = 0.0229;
    intra.intra_arm = true;
    intra.d_hard_m = 0.005;
    intra.d_slow_m = 0.015;
    rb_servo::SelfCollisionNearPairViz grip;
    grip.name_a = "dual_rb5_850e_left_pika_gripper_base";
    grip.name_b = "dual_rb5_850e_right_pika_gripper_base";
    grip.clearance_m = 0.0085;
    grip.gripper_gripper = true;
    grip.d_hard_m = 0.025;
    grip.d_slow_m = 0.067;
    // + = separating. The viewer needs this to tell a pair the barrier is braking from
    // one that merely parks inside the band (this cell holds nine of those all run).
    intra.rate_m_s = 0.0;
    grip.rate_m_s = -0.012;
    snapshot.self_collision_near_pairs = {intra, grip};

    rb_servo::StatePublisher publisher(rb_servo::DualArmConfig{});
    const nlohmann::json json = nlohmann::json::parse(publisher.serializeSnapshot(snapshot));
    const auto& pairs = json.at("self_collision").at("near_pairs");
    RB_CHECK(pairs.size() == 2);

    const auto& a = pairs.at(0);
    RB_CHECK(a.at("intra_arm").get<bool>());
    RB_CHECK(!a.at("gripper_gripper").get<bool>());
    RB_CHECK(a.at("d_hard_m").get<double>() == 0.005);
    RB_CHECK(a.at("d_slow_m").get<double>() == 0.015);
    // Nearest, but NOT violating: 22.9 mm is well outside its own 5 mm floor.
    RB_CHECK(a.at("clearance_m").get<double>() >= a.at("d_hard_m").get<double>());
    // ...and not closing either, so a viewer must not paint it as being braked.
    RB_CHECK(a.at("rate_m_s").get<double>() == 0.0);

    const auto& b = pairs.at(1);
    RB_CHECK(b.at("gripper_gripper").get<bool>());
    RB_CHECK(!b.at("intra_arm").get<bool>());
    RB_CHECK(b.at("d_hard_m").get<double>() == 0.025);
    // Ranked second, but this is the pair actually in hard violation.
    RB_CHECK(b.at("clearance_m").get<double>() < b.at("d_hard_m").get<double>());
    RB_CHECK(b.at("rate_m_s").get<double>() < 0.0);  // and closing
    return true;
}

bool testPublisherAdvertisesConfiguredChunkExecutionWithoutMotion() {
    rb_servo::DualArmConfig config;
    rb_servo::TcpPoseTargetProfileConfig legacy;
    legacy.name = "flow_infer_smooth";
    legacy.ruckig_follower.enable = true;
    legacy.ruckig_follower.controller = rb_servo::RuckigFollowerController::DeltaPreview;
    auto fresh = legacy;
    fresh.name = "flow_infer_fresh";
    fresh.ruckig_follower.fresh_chunk_replan = true;
    fresh.ruckig_follower.deadline_jerk_minimization = true;
    fresh.ruckig_follower.continuous_hold_resume = true;
    fresh.ruckig_follower.output_smd.velocity_ff_linear_gain = 0.25;
    fresh.ruckig_follower.output_smd.mode = rb_servo::FollowerOutputSmdMode::PositionLowpass2;
    auto preview = fresh;
    preview.name = "flow_infer_preview";
    preview.ruckig_follower.preview_execution.enable = true;
    preview.ruckig_follower.output_smd.enable = false;
    preview.ruckig_follower.output_smd.mode = rb_servo::FollowerOutputSmdMode::LegacySmd;
    config.cartesian_control.tcp_pose_target_profiles = {legacy, fresh, preview};
    rb_servo::StatePublisher publisher(config);
    const auto json = nlohmann::json::parse(publisher.serializeSnapshot(snapshotWithTick(0)));
    const auto& profiles = json.at("chunk_execution_profiles");
    RB_CHECK(profiles.size() == 3);
    RB_CHECK(profiles.at(0).at("name") == "flow_infer_smooth");
    RB_CHECK(!profiles.at(0).at("fresh_chunk_replan").get<bool>());
    RB_CHECK(!profiles.at(0).at("deadline_jerk_minimization").get<bool>());
    RB_CHECK(profiles.at(1).at("name") == "flow_infer_fresh");
    RB_CHECK(profiles.at(1).at("controller") == "delta_preview");
    RB_CHECK(profiles.at(1).at("enabled").get<bool>());
    RB_CHECK(profiles.at(1).at("fresh_chunk_replan").get<bool>());
    RB_CHECK(profiles.at(1).at("deadline_jerk_minimization").get<bool>());
    RB_CHECK(profiles.at(1).at("continuous_hold_resume").get<bool>());
    RB_CHECK(profiles.at(1).at("output_smd").at("velocity_ff_linear_gain") == 0.25);
    RB_CHECK(profiles.at(0).at("output_smd").at("velocity_ff_linear_gain") == 1.0);
    RB_CHECK(profiles.at(0).at("output_smd").at("mode") == "legacy_smd");
    RB_CHECK(profiles.at(1).at("output_smd").at("mode") == "position_lowpass2");
    RB_CHECK(!profiles.at(0).at("preview_execution").get<bool>());
    RB_CHECK(!profiles.at(1).at("preview_execution").get<bool>());
    RB_CHECK(profiles.at(2).at("name") == "flow_infer_preview");
    RB_CHECK(profiles.at(2).at("preview_execution").get<bool>());
    RB_CHECK(profiles.at(2).at("enabled").get<bool>());
    RB_CHECK(!profiles.at(2).at("output_smd").at("enabled").get<bool>());
    RB_CHECK(profiles.at(2).at("gripper_state_max_age_sec") == config.servo.command_timeout_sec);
    config.cartesian_control.enable = false;
    rb_servo::StatePublisher disabled(config);
    const auto off = nlohmann::json::parse(disabled.serializeSnapshot(snapshotWithTick(0)));
    RB_CHECK(!off.at("chunk_execution_profiles").at(1).at("enabled").get<bool>());
    RB_CHECK(!off.at("chunk_execution_profiles").at(2).at("enabled").get<bool>());
    return true;
}

rb_servo::ServoSnapshot witnessStressSnapshot(int count, bool all_hard = false) {
    auto snapshot = snapshotWithTick(912);
    snapshot.loop_start_time_ns = 1582481108339256ULL;
    snapshot.motion_epoch = 17;
    snapshot.left_force_control.reference_reset_count = 6;
    snapshot.right_force_control.reference_reset_count = 8;
    snapshot.left_state.tcp_command_stand = rb_servo::Pose6D{0.13, -0.27, 0.14, 0.2, 0.3, 0.4};
    snapshot.self_collision_mesh = true;
    snapshot.self_collision_enabled = true;
    snapshot.self_collision_checked = true;
    snapshot.self_collision_violated = true;
    snapshot.self_collision_near_count = count;
    snapshot.self_collision_pair = "left_stand";
    for (int i = 0; i < count; ++i) {
        rb_servo::SelfCollisionNearPairViz pair;
        pair.name_a = "dual_rb5_850e_left_link3_" + std::to_string(i);
        pair.name_b = "dual_rb5_850e_right_pika_gripper_finger_" + std::to_string(i);
        pair.p_a_m = {0.12345678912345678, -0.23456789123456789, 0.34567891234567891};
        pair.p_b_m = {0.14345678912345678, -0.24456789123456789, 0.35567891234567891};
        // Raw-clearance sorted, but the last two are the urgent ones because
        // their own hard floor differs. The first/nearest pairs are harmless.
        const bool hard = all_hard || i >= count - 2;
        pair.clearance_m = 0.020123456789 + i * 0.0001;
        pair.d_hard_m = hard ? 0.1 : 0.005;
        pair.d_slow_m = 0.12;
        snapshot.self_collision_near_pairs.push_back(pair);
    }
    return snapshot;
}

void populateDetailedPreviewFixture(rb_servo::PreviewExecutionTelemetry& p) {
    p.gate_revision = 9007199254740993ULL;
    p.gauge_revision = 9007199254740995ULL;
    p.parent_plan_id = 9007199254740997ULL;
    p.request_id = 9007199254740999ULL;
    p.result_valid = true;
    p.result_solve_attempted = true;
    p.last_worker_status = "fixture_last_worker_status,\"quoted\"";
    p.last_solve_status = "fixture_last_solve_status,\"quoted\"";
    p.last_admission_reason = "fixture_last_admission_reason,\"quoted\"";
    p.result_request_id = 9007199254741011ULL;
    p.result_epoch = 9007199254741013ULL;
    p.result_gate_revision = 9007199254741015ULL;
    p.result_gauge_revision = 9007199254741017ULL;
    p.result_source_wire_seq = 9007199254741019ULL;
    p.result_source_recv_seq = 9007199254741021ULL;
    p.result_parent_plan_id = 9007199254741023ULL;
    p.result_gauge_transported = 9007199254741025ULL;
    p.staged_gauge_transported = 9007199254741027ULL;
    p.gauge_transport_failed = 9007199254741029ULL;
    p.result_generated_at_sec = 0.002;
    p.result_splice_at_sec = 0.0021;
    p.result_valid_until_sec = 0.0022;
    p.result_completed_at_sec = 0.0023;
    p.result_observed_at_sec = 0.0024;
    p.solve_iterations = 19;
    p.solve_contact_constrained = true;
    p.solve_contact_decomposed = true;
    p.solve_contact_coupled_fallback = true;
    p.solve_max_constraint_violation = 0.0029;
    p.solve_max_contact_velocity_violation_m_s = 0.003;
    p.ready_not_staged = 9007199254741053ULL;
    p.staged_identity_rejected = 9007199254741055ULL;
    p.staged_expired = 9007199254741057ULL;
    p.staged_sample_rejected = 9007199254741059ULL;
    p.staged_contact_rejected = 9007199254741061ULL;
    p.last_staged_cancel_reason = "fixture_last_staged_cancel_reason,\"quoted\"";
    p.last_staged_cancel_time_sec = 0.0037;
    p.last_staged_cancel_request_id = 9007199254741067ULL;
    p.last_admission_time_sec = 0.0039;
    p.last_admission_gap_sec = 0.004;
    p.last_admitted_request_id = 9007199254741073ULL;
    p.last_admitted_parent_plan_id = 9007199254741075ULL;
    p.last_brake_reason = "fixture_last_brake_reason,\"quoted\"";
    p.last_brake_start_time_sec = 0.0044;
    p.last_brake_origin_sec = 0.0045;
    p.angular_continuations_started = 9007199254741083ULL;
    p.angular_brakes_started = 9007199254741085ULL;
    p.last_contact_reject_time_sec = 0.0048;
    p.last_contact_reject_gate = 0.0049;
    p.last_contact_reject_closing_m_s = 0.005;
    p.last_contact_reject_allowed_m_s = 0.0051;
    p.fold_count = 9007199254741095ULL;
    p.fold_force_count = 9007199254741097ULL;
    p.fold_roi_floor_count = 9007199254741099ULL;
    p.fold_geometry_hold_count = 9007199254741101ULL;
    p.fold_unknown_count = 9007199254741103ULL;
    p.fold_booked_time_ns = 9007199254741105ULL;
    p.fold_applied_time_ns = 9007199254741107ULL;
    p.fold_revision = 9007199254741109ULL;
    p.fold_geometry_cause_mask = 13;
    p.pending_geometry_fold_valid = true;
    p.pending_geometry_fold_time_ns = 9007199254749991ULL;
    p.pending_geometry_fold_cause_mask = 6;
    p.pending_geometry_fold_translation_m = {-0.051,0.052,-0.053};
    p.pending_geometry_fold_quaternion_xyzw = {0.6,0.0,0.0,0.8};
    p.request_invalid = 9007199254741113ULL;
    p.request_mailbox_full = 9007199254741115ULL;
    p.request_coalesced = 9007199254741117ULL;
    p.result_publish_dropped = 9007199254741119ULL;
    p.result_coalesced = 9007199254741121ULL;
    p.solve_angular_norm_coupled = true;
    p.solve_angular_norm_cuts = 9007199254748881ULL;
    p.solve_max_angular_chart_velocity_norm = 1.23;
    p.solve_max_angular_chart_acceleration_norm = 2.34;
    p.result_initial_linear_velocity_max_m_s = 0.345;
    p.result_initial_linear_acceleration_max_m_s2 = 4.56;
    p.result_initial_angular_velocity_norm_rad_s = 0.789;
    p.result_initial_angular_acceleration_norm_rad_s2 = 7.89;
    p.fold_cause = rb_servo::PreviewFoldCause::GeometryHold;
    for (std::size_t i=0; i<p.worker_status_counts.size(); ++i) p.worker_status_counts[i] = 9007199254741993ULL + 0*100 + i*2;
    for (std::size_t i=0; i<p.solve_status_counts.size(); ++i) p.solve_status_counts[i] = 9007199254741993ULL + 1*100 + i*2;
    for (std::size_t i=0; i<p.result_checks.size(); ++i) p.result_checks[i] = 9007199254741993ULL + 2*100 + i*2;
    for (std::size_t i=0; i<p.staged_cancel_counts.size(); ++i) p.staged_cancel_counts[i] = 9007199254741993ULL + 3*100 + i*2;
    for (std::size_t i=0; i<p.brake_counts.size(); ++i) p.brake_counts[i] = 9007199254741993ULL + 4*100 + i*2;
    p.last_contact_reject_normal = {0.011,0.012,0.013};
    p.fold_translation_m = {0.021,0.022,0.023};
    p.fold_quaternion_xyzw = {0,0,0.6,0.8};
    p.fold_booked_translation_m = {0.041,0.042,0.043};
    p.fold_booked_quaternion_xyzw = {0,0,0.6,0.8};
    p.gauge_translation_m = {0.101,-0.202,0.303};
    p.gauge_quaternion_xyzw = {0.0,0.8,0.0,0.6};
}

rb_servo::ServoSnapshot populatedTaredSnapshot(bool preview) {
    auto snapshot = witnessStressSnapshot(0);
    state_publication_fixture::wideSnapshot(snapshot);
    state_publication_fixture::taredForce(snapshot.left_ft, snapshot.left_force_control);
    state_publication_fixture::taredForce(snapshot.right_ft, snapshot.right_force_control);
    snapshot.tick = 4068;
    snapshot.loop_start_time_ns = 1610048494348843ULL;
    snapshot.loop_end_time_ns = 1610048496611833ULL;
    for (auto* p : {&snapshot.left_cartesian_solve.preview_execution,
                   &snapshot.right_cartesian_solve.preview_execution}) {
        populateDetailedPreviewFixture(*p);
        p->enabled=preview;p->active=preview;p->status=preview?"active":"disabled";
        p->sample_time_ns=snapshot.loop_start_time_ns;
        p->backlog_sec=.012345678912345678;p->rate=1.0123456789123456;
        p->plan_age_sec=.023456789123456789;p->solve_time_sec=.0012345678912345678;
        p->accepted_position_error_m=.00012345678912345678;
        p->accepted_rotation_error_rad=.00023456789123456789;
    }
    return snapshot;
}

bool testFullPrecisionTaredCoreFitsUdp() {
    const auto path = std::filesystem::path(__FILE__).parent_path().parent_path() / "config/stack_real.yaml";
    const auto cfg = rb_servo::loadConfigFromYaml(path.string());
    rb_servo::StatePublisher publisher(cfg);
    for (const bool preview : {false,true}) {
        auto snapshot=populatedTaredSnapshot(preview);
        snapshot.self_collision_near_count=220;
        const auto core_payload=publisher.serializeSnapshot(snapshot);
        const auto core=nlohmann::json::parse(core_payload);
        std::cout << "full_precision_tared_preview=" << preview << " core_bytes=" << core_payload.size() << '\n';
        RB_CHECK(core_payload.size()<=64'000);
        RB_CHECK(!core.contains("last_cartesian_solve"));
        RB_CHECK(core.at("state_publication").at("wire_revision")==2);
        RB_CHECK(core.at("state_publication").at("cartesian_solve_layout")=="per_arm");
        RB_CHECK(core.at("state_publication").at("cartesian_solve_paths")==
            nlohmann::json({"left.cartesian_solve","right.cartesian_solve"}));
        RB_CHECK(core.at("state_publication").at("omitted_legacy_aliases")==
            nlohmann::json({"last_cartesian_solve"}));
        RB_CHECK(core.at("state_publication").at("payload_bytes")==core_payload.size());
        RB_CHECK(core.at("loop_start_time_ns")==snapshot.loop_start_time_ns);
        RB_CHECK(core.at("motion_epoch")==snapshot.motion_epoch);
        RB_CHECK(core.at("left").at("cartesian_solve").at("ik_duration_us")==snapshot.left_cartesian_solve.ik_duration_us);
        for(const char* side:{"left","right"}) {
            const auto& tcp=core.at(side).at("tcp_command_stand");
            const auto pose=state_publication_fixture::widePose();
            RB_CHECK(tcp.at("x")==pose.x && tcp.at("y")==pose.y && tcp.at("z")==pose.z);
            // Canonical normalization predates this transport fix; compare
            // the exact normalized values, without reducing wire precision.
            auto quaternion=*pose.quaternion_xyzw;
            const double norm=std::sqrt(quaternion[0]*quaternion[0]+quaternion[1]*quaternion[1]+
                quaternion[2]*quaternion[2]+quaternion[3]*quaternion[3]);
            for(auto& component:quaternion)component/=norm;
            RB_CHECK(tcp.at("quaternion_xyzw")==nlohmann::json(quaternion));
            RB_CHECK(core.at(side).at("q_actual_deg").at(0)==snapshot.left_state.q_actual_deg[0]);
            RB_CHECK(core.at(side).at("force_torque").at("bias_valid")==true);
            RB_CHECK(core.at(side).at("force_torque").at("tare_state")=="accepted");
            RB_CHECK(core.at(side).at("force_torque").at("bias_sensor").at(0)==snapshot.left_ft.bias.fx);
            RB_CHECK(core.at(side).at("force_control").at("reference_deviation_stand_m").at(0)==snapshot.left_force_control.reference_deviation_m[0]);
            RB_CHECK(core.at("preview_execution").at(side).at("enabled")==preview);
            RB_CHECK(core.at("preview_execution").at(side).at("sample_time_ns")==snapshot.loop_start_time_ns);
            RB_CHECK(core.at("preview_execution").at(side).at("diagnostics_detail")=="summary");
        }
        const auto witnesses=witnessStressSnapshot(220);
        snapshot.self_collision_near_pairs=witnesses.self_collision_near_pairs;
        snapshot.self_collision_near_count=witnesses.self_collision_near_count;
        const auto payload=publisher.serializeSnapshot(snapshot);
        auto with_witnesses=nlohmann::json::parse(payload);
        RB_CHECK(payload.size()<=64'000);
        RB_CHECK(with_witnesses.at("self_collision").at("near_pairs_truncated").get<bool>());
        // Every core value, including all floating-point precision, is identical
        // with and without optional witnesses. Only witness metadata/size change.
        auto core_copy=core;
        for(auto* value:{&with_witnesses,&core_copy}) {
            value->at("state_publication").erase("payload_bytes");
            for(const char* field:{"near_pairs","near_pairs_total","near_pairs_published",
                "near_pairs_truncated","near_pairs_hard_total","near_pairs_hard_published",
                "near_pairs_hard_truncated"})value->at("self_collision").erase(field);
        }
        RB_CHECK(with_witnesses==core_copy);
    }
    return true;
}

bool testFullPrecisionTaredFanout() {
    UdpSocket recorder,gui,gripper_command_sink,feedback_port;
    RB_CHECK(bindLoopbackUdp(&recorder));RB_CHECK(bindLoopbackUdp(&gui));
    RB_CHECK(bindLoopbackUdp(&gripper_command_sink));RB_CHECK(bindLoopbackUdp(&feedback_port));
    const auto path = std::filesystem::path(__FILE__).parent_path().parent_path() / "config/stack_real.yaml";
    auto cfg=rb_servo::loadConfigFromYaml(path.string());
    // Test-only bridge uses ephemeral loopback sockets in both directions.
    // It cannot connect to the configured physical gripper endpoint.
    cfg.gripper.enable=true;
    cfg.gripper.command_endpoint=endpointFor(gripper_command_sink);
    cfg.gripper.feedback_bind=endpointFor(feedback_port);
    cfg.network.state_pub_endpoint=endpointFor(recorder);
    cfg.network.state_pub_endpoints={endpointFor(recorder),endpointFor(gui)};
    cfg.network.state_pub_rate_hz=100;
    ::close(feedback_port.fd);feedback_port.fd=-1;
    const auto snapshot=populatedTaredSnapshot(true);
    rb_servo::StatePublisher publisher(cfg);publisher.updateSnapshot(snapshot);
    const bool started=publisher.start();
    const auto now_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    const nlohmann::json arm={{"percent",23.456789123456789},{"target_percent",34.567891234567891},
        {"sample_age_ms",12.345678912345678},{"moving",true},{"ok",true}};
    const nlohmann::json feedback={{"schema","robotics_lab.gripper_state.v1"},
        {"host_time_ns",now_ns-123456789},{"left",arm},{"right",arm}};
    const auto bytes=feedback.dump();sockaddr_in address{};
    address.sin_family=AF_INET;address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    address.sin_port=htons(static_cast<uint16_t>(feedback_port.port));
    const bool injected=started && ::sendto(recorder.fd,bytes.data(),bytes.size(),0,
        reinterpret_cast<const sockaddr*>(&address),sizeof(address))==static_cast<ssize_t>(bytes.size());
    std::string a,b;bool received=false;
    for(int packet=0;injected&&packet<20;++packet) {
        if(!receivePacket(recorder.fd,&a)||!receivePacket(gui.fd,&b))break;
        const auto candidate=nlohmann::json::parse(a);
        if(candidate.at("left").at("gripper").at("valid")==true &&
           candidate.at("right").at("gripper").at("valid")==true) {received=true;break;}
    }
    publisher.stop();
    RB_CHECK(started&&injected&&received);RB_CHECK(a==b);RB_CHECK(a.size()<=64'000);
    const auto message=nlohmann::json::parse(a);
    std::cout << "full_precision_tared_valid_gripper core_bytes=" << a.size() << '\n';
    RB_CHECK(message.at("state_publication").at("oversize_dropped_total")==0);
    RB_CHECK(message.at("state_publication").at("send_errors_total")==0);
    RB_CHECK(message.at("left").at("force_torque").at("bias_valid")==true);
    RB_CHECK(message.at("right").at("force_torque").at("bias_valid")==true);
    RB_CHECK(message.at("preview_execution").at("left").at("active")==true);
    RB_CHECK(message.at("left").at("gripper").at("percent")==arm.at("percent"));
    RB_CHECK(message.at("right").at("gripper").at("sample_age_ms")==arm.at("sample_age_ms"));
    return true;
}

bool testPreviewTelemetryAndCapabilitySurviveWitnessBudget() {
    const auto path = std::filesystem::path(__FILE__).parent_path().parent_path() / "config/stack_real.yaml";
    const auto cfg = rb_servo::loadConfigFromYaml(path.string());
    rb_servo::StatePublisher publisher(cfg);
    auto snapshot = witnessStressSnapshot(220);
    auto& p = snapshot.left_cartesian_solve.preview_execution;
    p.enabled = true; p.active = true; p.status = "tracking";
    p.sample_time_ns = 9'007'199'254'740'993ULL;
    p.epoch = 7; p.plan_id = 11; p.source_wire_seq = 13; p.source_recv_seq = 17;
    p.backlog_sec = .012; p.rate = 1.03; p.plan_age_sec = .024;
    p.accepted_position_error_m = .00015; p.accepted_rotation_error_rad = .00025;
    p.solve_time_sec = .0004; p.submitted = 23; p.accepted = 19;
    p.rejected = 3; p.expired = 2; p.contact_guard_count = 5;
    populateDetailedPreviewFixture(p);
    // Populate both arms to test the actual worst-side diagnostic wire growth.
    snapshot.right_cartesian_solve.preview_execution = p;
    const auto payload = publisher.serializeSnapshot(snapshot);
    const auto message = nlohmann::json::parse(payload);
    RB_CHECK(payload.size() <= 64'000);
    RB_CHECK(message.at("self_collision").at("near_pairs_truncated").get<bool>());
    const auto& left = message.at("preview_execution").at("left");
    const nlohmann::json expected = {
        {"enabled",true},{"active",true},{"status","tracking"},{"sample_time_ns",9'007'199'254'740'993ULL},
        {"epoch",7},{"plan_id",11},{"source_wire_seq",13},{"source_recv_seq",17},
        {"backlog_sec",.012},{"rate",1.03},{"plan_age_sec",.024},
        {"accepted_position_error_m",.00015},{"accepted_rotation_error_rad",.00025},
        {"solve_time_sec",.0004},{"submitted",23},{"accepted",19},{"rejected",3},
        {"expired",2},{"contact_guard_count",5}};
    // Check the complete original telemetry contract while allowing explicitly
    // additive fields. Large integer source IDs must not pass through double.
    for (auto it = expected.begin(); it != expected.end(); ++it) RB_CHECK(left.at(it.key()) == it.value());
    RB_CHECK(left.at("sample_time_ns").is_number_unsigned());
    static_assert(std::is_trivially_copyable_v<rb_servo::PreviewExecutionTelemetry>);
    // The new diagnostic extension is deliberately a fixed wire summary.
    // The complete extension is tested separately against the unchanged CSV.
    const nlohmann::json detailed_expected = {
        {"last_worker_status","fixture_last_worker_status,\"quoted\""},
        {"last_solve_status","fixture_last_solve_status,\"quoted\""},
        {"last_admission_reason","fixture_last_admission_reason,\"quoted\""},
        {"last_brake_reason","fixture_last_brake_reason,\"quoted\""},
        {"last_staged_cancel_reason","fixture_last_staged_cancel_reason,\"quoted\""},
        {"diagnostics_detail","summary"},
        {"diagnostics_full_source","servo_csv"},
    };
    for (const char* side : {"left", "right"}) {
        const auto& details = message.at("preview_execution").at(side);
        RB_CHECK(details.size() == expected.size() + detailed_expected.size());
        for (auto it=detailed_expected.begin(); it!=detailed_expected.end(); ++it)
            RB_CHECK(details.at(it.key()) == it.value());
        for (const char* key : {"worker_status_counts","solve_status_counts","result_checks",
                               "staged_cancel_counts","brake_counts","fold_translation_m",
                               "gauge_quaternion_xyzw","pending_geometry_fold_translation_m",
                               "gate_revision","request_id","result_request_id","fold_cause"})
            RB_CHECK(!details.contains(key));
    }

    const auto defaults = nlohmann::json::parse(publisher.serializeSnapshot(snapshotWithTick(0)));
    const auto& right = defaults.at("preview_execution").at("right");
    RB_CHECK(!right.at("enabled").get<bool>() && !right.at("active").get<bool>());
    RB_CHECK(right.at("status") == "disabled");
    RB_CHECK(right.at("plan_id") == 0 && right.at("sample_time_ns") == 0);
    RB_CHECK(right.at("rate") == 1.0);
    RB_CHECK(right.at("last_worker_status") == "not_observed");
    RB_CHECK(right.at("diagnostics_detail") == "summary");
    RB_CHECK(right.at("diagnostics_full_source") == "servo_csv");

    const auto& profiles = message.at("chunk_execution_profiles");
    RB_CHECK(profiles.size() == cfg.cartesian_control.tcp_pose_target_profiles.size());
    int preview_count = 0, fresh_count = 0;
    for (const auto& profile : profiles) {
        if (profile.at("name") == "flow_infer_preview") {
            ++preview_count;
            RB_CHECK(profile.at("enabled").get<bool>() && profile.at("preview_execution").get<bool>());
            RB_CHECK(!profile.at("output_smd").at("enabled").get<bool>());
            RB_CHECK(profile.at("gripper_state_max_age_sec") == cfg.servo.command_timeout_sec);
        }
        if (profile.at("name") == "flow_infer_fresh") {
            ++fresh_count;
            RB_CHECK(!profile.at("preview_execution").get<bool>());
            RB_CHECK(profile.at("output_smd").at("enabled").get<bool>());
            RB_CHECK(profile.at("output_smd").at("mode") == "position_lowpass2");
        }
    }
    RB_CHECK(preview_count == 1 && fresh_count == 1);
    return true;
}

bool testWitnessBudgetPreservesCoreAndUrgentPairs() {
    rb_servo::StatePublisher publisher(rb_servo::DualArmConfig{});
    const auto snapshot = witnessStressSnapshot(220);
    const std::string payload = publisher.serializeSnapshot(snapshot);
    auto result = nlohmann::json::parse(payload);
    const auto& sc = result.at("self_collision");
    const auto& pairs = sc.at("near_pairs");
    RB_CHECK(payload.size() <= 64'000);
    RB_CHECK(result.at("state_publication").at("payload_bytes") == payload.size());
    RB_CHECK(sc.at("near_pairs_total") == 220);
    RB_CHECK(sc.at("near_pairs_published") == pairs.size());
    RB_CHECK(sc.at("near_pairs_truncated").get<bool>());
    RB_CHECK(sc.at("near_pairs_hard_total") == 2);
    RB_CHECK(sc.at("near_pairs_hard_published") == 2);
    RB_CHECK(!sc.at("near_pairs_hard_truncated").get<bool>());
    RB_CHECK(pairs.at(pairs.size() - 2).at("name_a") == snapshot.self_collision_near_pairs[218].name_a);
    RB_CHECK(pairs.back().at("name_a") == snapshot.self_collision_near_pairs[219].name_a);
    for (std::size_t i = 1; i < pairs.size(); ++i) {
        RB_CHECK(pairs[i - 1].at("clearance_m").get<double>() <= pairs[i].at("clearance_m").get<double>());
    }
    // The const servo snapshot and all non-witness JSON are unchanged. This
    // covers the full legacy tree, not just a selected list of proprio fields.
    RB_CHECK(snapshot.self_collision_near_pairs.size() == 220);
    auto core_snapshot = snapshot;
    core_snapshot.self_collision_near_pairs.clear();
    auto core = nlohmann::json::parse(publisher.serializeSnapshot(core_snapshot));
    for (auto* j : {&core, &result}) {
        j->at("state_publication").erase("payload_bytes");
        for (const char* name : {"near_pairs", "near_pairs_total", "near_pairs_published",
             "near_pairs_truncated", "near_pairs_hard_total", "near_pairs_hard_published",
             "near_pairs_hard_truncated"}) {
            j->at("self_collision").erase(name);
        }
    }
    RB_CHECK(core == result);
    return true;
}

bool testWitnessBudgetMarksIncompleteHardPairsAndEscapedBytes() {
    rb_servo::StatePublisher publisher(rb_servo::DualArmConfig{});
    auto snapshot = witnessStressSnapshot(220, true);
    // JSON escaping/UTF-8 byte length, not character count or pair count, owns
    // the wire bound. A single huge witness must not crowd out the core state.
    snapshot.self_collision_near_pairs.front().name_a = std::string(40'000, '\n') + u8"로봇";
    snapshot.self_collision_near_pairs[1].clearance_m = std::numeric_limits<double>::quiet_NaN();
    snapshot.self_collision_near_pairs[2].d_hard_m = std::numeric_limits<double>::infinity();
    const auto payload = publisher.serializeSnapshot(snapshot);
    const auto result = nlohmann::json::parse(payload);
    const auto& sc = result.at("self_collision");
    RB_CHECK(payload.size() <= 64'000);
    RB_CHECK(sc.at("near_pairs_hard_truncated").get<bool>());
    RB_CHECK(sc.at("near_pairs_hard_total") == 218);
    RB_CHECK(sc.at("near_pairs_hard_published").get<int>() > 0);
    RB_CHECK(sc.at("near_pairs_hard_published").get<int>() < 220);
    RB_CHECK(result.at("state_publication").at("payload_bytes") == payload.size());
    return true;
}

bool testOversizeCoreIsNeverSilentlyRemoved() {
    rb_servo::StatePublisher publisher(rb_servo::DualArmConfig{});
    auto snapshot = witnessStressSnapshot(220);
    snapshot.fault_reason = std::string(70'000, 'x');
    const auto payload = publisher.serializeSnapshot(snapshot);
    const auto result = nlohmann::json::parse(payload);
    RB_CHECK(payload.size() > 65'507);
    RB_CHECK(result.at("fault_reason") == snapshot.fault_reason);
    RB_CHECK(result.at("fault_context").at("reason") == snapshot.fault_reason);
    RB_CHECK(result.at("self_collision").at("near_pairs_published") == 0);
    RB_CHECK(result.at("self_collision").at("near_pairs_hard_truncated").get<bool>());
    RB_CHECK(result.at("state_publication").at("payload_bytes") == payload.size());
    return true;
}

bool testBoundedWitnessPayloadActuallyFansOutOverUdp() {
    UdpSocket recorder, gui;
    if (!bindLoopbackUdp(&recorder) || !bindLoopbackUdp(&gui)) return true;
    rb_servo::DualArmConfig cfg;
    cfg.network.state_pub_endpoints = {endpointFor(recorder), endpointFor(gui)};
    cfg.network.state_pub_rate_hz = 100;
    const auto snapshot = witnessStressSnapshot(220);
    rb_servo::StatePublisher publisher(cfg, [&]() { return snapshot; });
    RB_CHECK(publisher.start());
    std::string recorder_payload, gui_payload;
    const bool received = receivePacket(recorder.fd, &recorder_payload) &&
        receivePacket(gui.fd, &gui_payload);
    publisher.stop();
    RB_CHECK(received);
    RB_CHECK(recorder_payload.size() <= 64'000);
    RB_CHECK(recorder_payload == gui_payload);
    const auto result = nlohmann::json::parse(recorder_payload);
    RB_CHECK(result.at("self_collision").at("near_pairs_truncated").get<bool>());
    RB_CHECK(result.at("motion_epoch") == 17);
    RB_CHECK(result.at("left").at("force_control").at("reference_reset_count") == 6);
    return true;
}

bool testOversizeDropDiagnosticAndRecovery() {
    UdpSocket sink;
    if (!bindLoopbackUdp(&sink)) return true;
    rb_servo::DualArmConfig cfg;
    cfg.network.state_pub_endpoints = {endpointFor(sink)};
    cfg.network.state_pub_rate_hz = 100;
    std::atomic<int> calls{0};
    rb_servo::StatePublisher publisher(cfg, [&]() {
        auto snapshot = snapshotWithTick(100 + calls.load());
        if (calls.fetch_add(1) < 3) snapshot.fault_reason = std::string(70'000, 'x');
        return snapshot;
    });
    std::ostringstream diagnostics;
    auto* previous = std::cerr.rdbuf(diagnostics.rdbuf());
    const bool started = publisher.start();
    std::string payload;
    const bool received = started && receivePacket(sink.fd, &payload);
    publisher.stop();
    std::cerr.rdbuf(previous);
    RB_CHECK(started && received);
    const auto result = nlohmann::json::parse(payload);
    RB_CHECK(result.at("state_publication").at("oversize_dropped_total") == 3);
    RB_CHECK(result.at("state_publication").at("last_error_code") == EMSGSIZE);
    RB_CHECK(result.at("state_publication").at("last_error_time_ns").get<uint64_t>() > 0);
    RB_CHECK(diagnostics.str().find("core payload exceeds IPv4 UDP limit") != std::string::npos);
    RB_CHECK(diagnostics.str().find("monotonic_ns=") != std::string::npos);
    RB_CHECK(diagnostics.str().find("bytes=") != std::string::npos);
    RB_CHECK(diagnostics.str().find("payload size recovered") != std::string::npos);
    RB_CHECK(diagnostics.str().find("dropped=3") != std::string::npos);
    return true;
}

}  // namespace

int main() {
    if (!testFullPrecisionTaredCoreFitsUdp()) return 1;
    if (!testFullPrecisionTaredFanout()) return 1;
    if (!testPreviewTelemetryAndCapabilitySurviveWitnessBudget()) return 1;
    if (!testWitnessBudgetPreservesCoreAndUrgentPairs()) return 1;
    if (!testWitnessBudgetMarksIncompleteHardPairsAndEscapedBytes()) return 1;
    if (!testOversizeCoreIsNeverSilentlyRemoved()) return 1;
    if (!testBoundedWitnessPayloadActuallyFansOutOverUdp()) return 1;
    if (!testOversizeDropDiagnosticAndRecovery()) return 1;
    if (!testPublisherAdvertisesConfiguredChunkExecutionWithoutMotion()) return 1;
    if (!testStatePublisherFanoutSendsSamePayloadToTwoSockets()) return 1;
    if (!testStatePublisherLegacySingleEndpointStillWorks()) return 1;
    if (!testStatePublisherSerializesJointReferenceFields()) return 1;
    if (!testStatePublisherSerializesAsyncStreamingFields()) return 1;
    if (!testStatePublisherKeepsForceTelemetryInsideTheArmObjects()) return 1;
    if (!testStatePublisherPreservesForceReferenceWhenUncovered()) return 1;
    if (!testStatePublisherSerializesPerPairSelfCollisionBands()) return 1;
    if (!testRealtimeTimingAccumulatorAndSerialization()) return 1;
    return 0;
}
