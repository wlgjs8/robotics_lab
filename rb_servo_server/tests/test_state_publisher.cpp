#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>

#include "rb_servo/control/realtime_timing.hpp"
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

bool testStatePublisherSerializesForceTelemetry() {
    rb_servo::ServoSnapshot snapshot = snapshotWithTick(124);
    snapshot.motion_epoch = 7;
    snapshot.left_force_torque.enabled = true;
    snapshot.left_force_torque.source = "rbpodo_eft";
    snapshot.left_force_torque.source_assurance = "controller_frame_only";
    snapshot.left_force_torque.raw_sensor_wrench.fx = 8.0;
    snapshot.left_force_torque.control_external_wrench.fz = 5.0;
    snapshot.left_force_torque.healthy = true;
    snapshot.left_force_torque.freshness_value = 42;
    snapshot.left_force_torque.auto_tare_enabled = true;
    snapshot.left_force_torque.tare_valid = true;
    snapshot.left_force_torque.tare_state = "accepted";
    snapshot.left_force_torque.tare_sample_count = 500;
    snapshot.left_force_torque.tare_generation = 4;
    snapshot.left_force_torque.tare_reason = "accepted";
    snapshot.left_force_torque.residual_tare_tcp.fz = 23.5;
    snapshot.left_force_control.enabled = true;
    snapshot.left_force_control.operating_mode = "cartesian_admittance";
    snapshot.left_force_control.state = "release_braking";
    snapshot.left_force_control.contact_active = true;
    snapshot.left_force_control.normal_contact_active = false;
    snapshot.left_force_control.transverse_contact_active = true;
    snapshot.left_force_control.rotational_contact_active = true;
    snapshot.left_force_control.measured_force_n = 5.0;
    snapshot.left_force_control.fast_normal_force_n = 5.5;
    snapshot.left_force_control.fast_force_norm_n = 7.0;
    snapshot.left_force_control.fast_torque_norm_nm = 0.8;
    snapshot.left_force_control.contact_threshold_exceeded = true;
    snapshot.left_force_control.hard_limit_threshold_exceeded = true;
    snapshot.left_force_control.hard_limit_sample_count = 2;
    snapshot.left_force_control.hard_limit_exceeded = false;
    snapshot.left_force_control.target_force_n = 3.0;
    snapshot.left_force_control.correction_m = 0.001;
    snapshot.left_force_control.compliance_active = true;
    snapshot.left_force_control.normal_regulating = true;
    snapshot.left_force_control.transverse_regulating = true;
    snapshot.left_force_control.rotational_regulating = true;
    snapshot.left_force_control.loading_projection_active = true;
    snapshot.left_force_control.compliance_frame = "sensor_origin";
    snapshot.left_force_control.control_wrench_surface = {1.0, 2.0, 5.0, 0.1, 0.2, 0.3};
    snapshot.left_force_control.control_wrench_compliance = {4.0, 5.0, 6.0, 0.4, 0.5, 0.6};
    snapshot.left_force_control.wrench_error_surface = {0.5, 1.5, 2.5, 0.05, 0.15, 0.25};
    snapshot.left_force_control.wrench_error_compliance = {3.5, 4.5, 5.5, 0.35, 0.45, 0.55};
    snapshot.left_force_control.compliance_offset_surface = {0.001, 0.002, 0.003, 0.01, 0.02, 0.03};
    snapshot.left_force_control.compliance_velocity_surface = {0.01, 0.02, 0.03, 0.1, 0.2, 0.3};
    snapshot.left_force_control.compliance_acceleration_surface = {0.1, 0.2, 0.3, 1.0, 2.0, 3.0};
    snapshot.left_force_control.raw_policy_delta_surface = {0.004, 0.005, -0.006, 0.04, 0.05, -0.06};
    snapshot.left_force_control.accepted_policy_delta_surface = {0.004, 0.0, 0.0, 0.04, 0.0, 0.0};
    snapshot.left_force_control.compliance_equilibrium_stand = {
        0.41, -0.22, 0.33, 0.1, -0.2, 0.3,
    };
    snapshot.left_force_control.compliance_equilibrium_source = "policy_target";
    snapshot.left_force_control.compliance_recenter_active = true;
    snapshot.left_force_control.compliance_limit_axes = {
        true, false, false, false, true, false,
    };
    snapshot.left_force_control.compliance_limit_reason =
        "jerk_limited_motion_envelope";
    snapshot.left_force_control.motion_epoch = 7;

    rb_servo::StatePublisher publisher(rb_servo::DualArmConfig{});
    const nlohmann::json json = nlohmann::json::parse(publisher.serializeSnapshot(snapshot));
    const nlohmann::json& ft = json.at("left").at("force_torque");
    const nlohmann::json& force = json.at("left").at("force_control");
    RB_CHECK(json.at("motion_epoch").get<uint64_t>() == 7);
    RB_CHECK(ft.at("source").get<std::string>() == "rbpodo_eft");
    RB_CHECK(ft.at("source_assurance").get<std::string>() == "controller_frame_only");
    RB_CHECK(!ft.at("sensor_health_verified").get<bool>());
    RB_CHECK(!ft.at("safety_rated").get<bool>());
    RB_CHECK(ft.at("raw_sensor_wrench").at(0).get<double>() == 8.0);
    RB_CHECK(ft.at("auto_tare_enabled").get<bool>());
    RB_CHECK(ft.at("tare_valid").get<bool>());
    RB_CHECK(ft.at("tare_state").get<std::string>() == "accepted");
    RB_CHECK(ft.at("tare_sample_count").get<int>() == 500);
    RB_CHECK(ft.at("tare_generation").get<uint64_t>() == 4);
    RB_CHECK(ft.at("residual_tare_tcp").at(2).get<double>() == 23.5);
    RB_CHECK(force.at("state").get<std::string>() == "release_braking");
    RB_CHECK(force.at("compliance_frame").get<std::string>() == "sensor_origin");
    RB_CHECK(force.at("contact_active").get<bool>());
    RB_CHECK(!force.at("normal_contact_active").get<bool>());
    RB_CHECK(force.at("transverse_contact_active").get<bool>());
    RB_CHECK(force.at("rotational_contact_active").get<bool>());
    RB_CHECK(force.at("fast_normal_force_n").get<double>() == 5.5);
    RB_CHECK(force.at("fast_force_norm_n").get<double>() == 7.0);
    RB_CHECK(force.at("fast_torque_norm_nm").get<double>() == 0.8);
    RB_CHECK(force.at("contact_threshold_exceeded").get<bool>());
    RB_CHECK(force.at("hard_limit_threshold_exceeded").get<bool>());
    RB_CHECK(force.at("hard_limit_sample_count").get<int>() == 2);
    RB_CHECK(!force.at("hard_limit_exceeded").get<bool>());
    RB_CHECK(force.at("correction_m").get<double>() == 0.001);
    RB_CHECK(force.at("compliance_active").get<bool>());
    RB_CHECK(force.at("normal_regulating").get<bool>());
    RB_CHECK(force.at("transverse_regulating").get<bool>());
    RB_CHECK(force.at("rotational_regulating").get<bool>());
    RB_CHECK(force.at("loading_projection_active").get<bool>());
    RB_CHECK(force.at("control_wrench_surface").at(2).get<double>() == 5.0);
    RB_CHECK(force.at("control_wrench_compliance").at(0).get<double>() == 4.0);
    RB_CHECK(force.at("wrench_error_surface").at(1).get<double>() == 1.5);
    RB_CHECK(force.at("wrench_error_compliance").at(4).get<double>() == 0.45);
    RB_CHECK(force.at("compliance_offset_surface").at(5).get<double>() == 0.03);
    RB_CHECK(force.at("compliance_velocity_surface").at(0).get<double>() == 0.01);
    RB_CHECK(force.at("compliance_acceleration_surface").at(3).get<double>() == 1.0);
    RB_CHECK(force.at("raw_policy_delta_surface").at(2).get<double>() == -0.006);
    RB_CHECK(force.at("accepted_policy_delta_surface").at(4).get<double>() == 0.0);
    RB_CHECK(force.at("compliance_equilibrium_stand").at(0).get<double>() == 0.41);
    RB_CHECK(force.at("compliance_equilibrium_stand").at(5).get<double>() == 0.3);
    RB_CHECK(force.at("compliance_equilibrium_source").get<std::string>() ==
             "policy_target");
    RB_CHECK(force.at("compliance_recenter_active").get<bool>());
    RB_CHECK(force.at("compliance_limit_axes").at(0).get<bool>());
    RB_CHECK(force.at("compliance_limit_axes").at(4).get<bool>());
    RB_CHECK(force.at("compliance_limit_reason").get<std::string>() ==
             "jerk_limited_motion_envelope");
    RB_CHECK(force.at("motion_epoch").get<uint64_t>() == 7);
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

}  // namespace

int main() {
    if (!testStatePublisherFanoutSendsSamePayloadToTwoSockets()) return 1;
    if (!testStatePublisherLegacySingleEndpointStillWorks()) return 1;
    if (!testStatePublisherSerializesJointReferenceFields()) return 1;
    if (!testStatePublisherSerializesAsyncStreamingFields()) return 1;
    if (!testStatePublisherSerializesForceTelemetry()) return 1;
    if (!testRealtimeTimingAccumulatorAndSerialization()) return 1;
    return 0;
}
