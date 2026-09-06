// Regression for repeated subtraction of a frozen force deviation while a
// delta_preview source waits for its first chunk after InitMotion/tare.
// Runs the real servo tick + Pinocchio FK/IK against an in-memory plant. No
// receiver is started, no socket/device/model/controller is contacted.
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <functional>
#include <stdexcept>
#include <string>
#include <thread>
#include <chrono>

#include <nlohmann/json.hpp>

#include "rb_servo/config/config.hpp"
#include "rb_servo/control/command_buffer.hpp"
#include "rb_servo/control/dual_arm_servo_loop.hpp"
#include "rb_servo/core/clock.hpp"
#include "rb_servo/kinematics/pinocchio_kinematics.hpp"
#include "rb_servo/math/se3.hpp"
#include "rb_servo/network/chunk_frame_receiver.hpp"

namespace {
using namespace rb_servo;
constexpr uint64_t kPeriodNs = 2'000'000;

void require(bool ok, const std::string& why) {
    if (!ok) throw std::runtime_error(why);
}

std::string zeroDeltaChunk(ArmId selected, const Pose6D& pose) {
    constexpr int horizon = 8;
    const Eigen::Quaterniond q(math::rotationFromPose(pose));
    const nlohmann::json point = {
        pose.x, pose.y, pose.z, q.x(), q.y(), q.z(), q.w(), 0.0};
    nlohmann::json points = nlohmann::json::array();
    nlohmann::json deltas = nlohmann::json::array();
    for (int i = 0; i < horizon; ++i) {
        points.push_back(point);
        deltas.push_back({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    }
    const char* arm = selected == ArmId::Left ? "left" : "right";
    nlohmann::json packet = {
        {"schema_version", "robotics_lab.chunk_overlay.v3"},
        {"host_time_ns", nowSteadyNs()}, {"seq", 1},
        {"policy_dt_sec", 0.0334}, {"horizon", horizon},
        {"left", nullptr}, {"right", nullptr},
        {"chunk_metadata", {{"observation_step_seq", 0}, {"activation_step_seq", 0},
                            {"source_start_index", 0}, {"original_horizon", horizon},
                            {"selected_horizon", horizon}, {"proprio", {{"valid", true}}}}}};
    packet[arm] = std::move(points);
    packet[std::string(arm) + "_delta"] = std::move(deltas);
    return packet.dump();
}

// The loop may only use manual time when both injected backends explicitly
// advertise it. This ideal plant realizes the previous sent joints exactly;
// physical actuator lag/contact response are intentionally outside this test.
class MemoryPlant final : public IRobotBackend {
public:
    MemoryPlant(ArmId arm, JointArray q) : arm_(arm), q_(q) {}
    bool supportsExternalStepping() const override { return true; }
    bool isConnected() const override { return connected_; }
    ArmId armId() const override { return arm_; }
    std::string name() const override { return "force_resume_memory_plant"; }
    BackendResult<RobotState> connect() override {
        connected_ = true;
        return result(BackendOp::Connect);
    }
    BackendResult<RobotState> initialize() override {
        initialized_ = true;
        return result(BackendOp::Initialize);
    }
    BackendResult<RobotState> readState() override { return result(BackendOp::ReadState); }
    BackendResult<RobotState> stop() override { return result(BackendOp::Stop); }
    BackendResult<RobotState> resetFault() override { return result(BackendOp::ResetFault); }
    SendServoJResult sendServoJ(const SendServoJRequest& request) override {
        require(initialized_, "send before memory-plant initialization");
        q_ = request.q_target_deg;
        SendServoJResult out;
        out.accepted = true;
        out.requested_q_deg = q_;
        out.acceptance_semantics = "memory_plant_applied";
        out.timing = makeBackendTiming(nowSteadyNs(), nowSteadyNs());
        return out;
    }
    void setWrench(const Wrench6D& wrench) { wrench_ = wrench; }
    void setWrenchValid(bool valid) { wrench_valid_ = valid; }

private:
    BackendResult<RobotState> result(BackendOp op) {
        RobotState state;
        state.arm_id = arm_;
        state.connection_state = connected_ ? RobotConnectionState::Connected
                                            : RobotConnectionState::Disconnected;
        state.servo_enabled = initialized_;
        state.q_actual_deg = state.q_target_deg = q_;
        state.q_actual_valid = state.q_ref_valid = state.has_valid_joint_state = true;
        state.q_ref_source = "memory_plant_sent";
        state.host_time_ns = state.robot_time_ns = nowSteadyNs();
        state.acquisition_sequence = ++sequence_;
        state.eft_valid = wrench_valid_;
        state.eft_wrench = wrench_;
        BackendResult<RobotState> out;
        out.ok = connected_;
        out.op = op;
        out.value = state;
        out.timing = makeBackendTiming(nowSteadyNs(), nowSteadyNs());
        return out;
    }
    ArmId arm_;
    JointArray q_{};
    Wrench6D wrench_{};
    uint64_t sequence_ = 0;
    bool connected_ = false;
    bool initialized_ = false;
    bool wrench_valid_ = true;
};

struct ManualClock {
    uint64_t time_ns = 1'000'000'000;
    ManualClock() { setExternalSteadyNs(time_ns); }
    ~ManualClock() { setExternalSteadyNs(0); }
    void advance() { setExternalSteadyNs(time_ns += kPeriodNs); }
};

DualArmConfig fixtureConfig(ArmId selected, bool rotation) {
    const auto root = std::filesystem::path(__FILE__).parent_path().parent_path();
    DualArmConfig cfg;
    for (auto* backend : {&cfg.left_robot, &cfg.right_robot}) {
        backend->backend_type = BackendType::Mock;
        backend->run_mode = RunMode::Mock;
        backend->ip.clear();
    }
    cfg.servo.rate_hz = 500;
    cfg.servo.io_model = ServoIoModel::Direct;
    cfg.servo.enable_realtime_priority = false;
    cfg.servo.cpu_core = -1;
    cfg.servo.send_servo_commands = true;
    cfg.servo.command_timeout_sec = 1.0;
    cfg.logging.enable = false;
    cfg.gripper.enable = false;
    cfg.safety.q_min_deg = rbpodoDefaultSafetyJointMinDeg();
    cfg.safety.q_max_deg = rbpodoDefaultSafetyJointMaxDeg();
    cfg.safety.dq_max_deg_s.fill(170.0);
    cfg.safety.ddq_max_deg_s2.fill(3000.0);
    cfg.safety.max_tracking_error_deg = 5.0;
    cfg.safety.tracking_error_policy = TrackingErrorPolicy::FaultLatch;
    cfg.kinematics.enable = true;
    cfg.kinematics.provider = "pinocchio";
    cfg.kinematics.urdf = (root / "descriptions/urdf/rb5_850e.urdf").string();
    cfg.kinematics.publish_tcp = true;
    cfg.kinematics.ik.enable = true;
    cfg.kinematics.ik.timeout_ms = 100.0;
    cfg.kinematics.ik.max_iterations = 100;
    cfg.kinematics.ik.position_tolerance_m = 1e-7;
    cfg.kinematics.ik.orientation_tolerance_rad = 1e-7;
    cfg.left_mount.arm_id = ArmId::Left;
    cfg.right_mount.arm_id = ArmId::Right;
    cfg.left_mount.base_pose_in_stand.x = -1.0;
    cfg.right_mount.base_pose_in_stand.x = 1.0;

    // InitMotion's measured no-op still exercises the production request,
    // reanchor, Done, and auto-tare lifecycle. It needs the real planner object,
    // but no path search or physical collision claim is made by this fixture.
    cfg.safety.self_collision.enable = true;
    cfg.safety.self_collision.monitor_only = true;
    auto& mesh = cfg.safety.self_collision.mesh;
    mesh.unified_urdf = (root / "descriptions/urdf/dual_rb5_850e_ver3.urdf").string();
    mesh.package_dirs = {(root / "descriptions/urdf").string()};
    mesh.left_prefix = "dual_rb5_850e_left_";
    mesh.right_prefix = "dual_rb5_850e_right_";
    cfg.safety.init_motion_planner.enable = true;
    cfg.safety.init_motion_planner.noop_tol_deg = 0.05;
    cfg.safety.init_motion_planner.waypoint_tol_deg = 0.05;
    cfg.safety.init_motion_planner.brake_before_plan = false;

    cfg.force_torque.enable = true;
    cfg.force_torque.push_zero_payload_to_box = false;
    auto& ft = selected == ArmId::Left ? cfg.force_torque.left : cfg.force_torque.right;
    ft.enable = true;
    ft.sensor_name = "synthetic_identity_axes";
    ft.bias_from_config = true;
    // Massless, noiseless in-memory sensor. Its sample freshness is verified by
    // external stepping, not by impersonating a physical RFT liveness signal.
    ft.tool_mass_kg = 0.0;
    auto& tare = cfg.force_torque.auto_tare_after_init_motion;
    tare.enable = true;
    tare.settle_sec = 0.5;
    tare.max_sent_speed_deg_s = 0.1;
    tare.invalidate_on_request = true;
    auto& fc = cfg.force_control;
    fc.enable = true;
    fc.gate_enable = true;
    fc.gate_max_force_n = 10.0;
    fc.gate_max_torque_nm = 1.4;
    fc.max_deviation_m = 0.04;
    fc.max_deviation_rad = 0.03;
    fc.coverage_recover_sec = 0.5;
    fc.hold_compliance = true;
    fc.fold_deviation = true;
    fc.hold_engage_force_n = 5.0;
    fc.hold_release_force_n = 2.0;
    for (int i = 0; i < 3; ++i) {
        fc.stream.translation[i] = {ForceAxisMode::Compliance, 12.0, 1000.0, 400.0, 0.0};
        fc.hold.translation[i] = {ForceAxisMode::Compliance, 12.0, 1000.0, 0.0, 0.0};
        fc.stream.rotation[i] = rotation
            ? ForceAxisConfig{ForceAxisMode::Compliance, 0.3, 30.0, 8.0, 0.0}
            : ForceAxisConfig{ForceAxisMode::Rigid, 0.0, 0.0, 0.0, 0.0};
        fc.hold.rotation[i] = {ForceAxisMode::Rigid, 0.0, 0.0, 0.0, 0.0};
    }
    TcpPoseTargetProfileConfig plain;
    plain.name = "plain_force_fixture";
    plain.pose_track_smd.enable = false;
    TcpPoseTargetProfileConfig preview = plain;
    preview.name = "flow_infer_smooth";
    auto& follower = preview.ruckig_follower;
    follower.enable = true;
    follower.controller = RuckigFollowerController::DeltaPreview;
    follower.fallback_policy = RuckigFollowerFallbackPolicy::Hold;
    follower.engage_timeout_sec = 3.0;
    follower.preview_max_projection_error_m = 0.002;
    follower.preview_max_projection_error_rad = 0.00436;
    follower.preview_max_consecutive_projection_errors = 12;
    follower.preview_max_actual_lead_m = 0.035;
    follower.preview_max_actual_lead_rad = 0.0873;
    follower.preview_max_consecutive_actual_lead_errors = 3;
    cfg.cartesian_control.tcp_pose_target_profile_default = plain.name;
    cfg.cartesian_control.tcp_pose_target_profiles = {plain, preview};
    return cfg;
}

double norm3(const std::array<double, 3>& v) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// Smaller lifecycle fixtures reuse the same public servo entry points. The
// force law's damping is shortened only in these synthetic invariance tests,
// so a standing offset is reached without repeating a long contact warmup.
struct Fixture {
    ManualClock clock;
    DualArmConfig cfg;
    std::shared_ptr<PinocchioKinematics> kin;
    CommandBuffer buffer;
    ChunkFrameReceiver receiver{""};
    MemoryPlant* left = nullptr;
    MemoryPlant* right = nullptr;
    std::unique_ptr<DualArmServoLoop> loop;
    uint64_t seq = 0;
    ServoSnapshot latest;
    JointArray initial{10.0, -20.0, 35.0, 5.0, 25.0, -15.0};

    explicit Fixture(const std::function<void(DualArmConfig&)>& configure,
                     JointArray initial_q = {10.0, -20.0, 35.0, 5.0, 25.0, -15.0})
        : initial(initial_q) {
        cfg = fixtureConfig(ArmId::Right, false);
        for (auto& axis : cfg.force_control.stream.translation) axis.b = 100.0;
        configure(cfg);
        kin = std::make_shared<PinocchioKinematics>(cfg.kinematics);
        auto l = std::make_unique<MemoryPlant>(ArmId::Left, initial);
        auto r = std::make_unique<MemoryPlant>(ArmId::Right, initial);
        left = l.get();
        right = r.get();
        loop = std::make_unique<DualArmServoLoop>(std::move(l), std::move(r), cfg, &buffer, nullptr, kin);
        loop->setChunkFrameReceiver(&receiver);
        loop->enableExternalStepping();
        require(loop->start(), "additional fixture failed to start");
        auto arm = command(ControlMode::ArmMotion, ControlMode::ArmMotion);
        tick(arm);
    }
    ~Fixture() { if (loop) loop->stop(); }
    DualArmCommand command(ControlMode lm, ControlMode rm,
                           const std::string& profile = "plain_force_fixture") const {
        DualArmCommand cmd;
        cmd.left.arm_id = ArmId::Left;
        cmd.right.arm_id = ArmId::Right;
        cmd.left.mode = lm;
        cmd.right.mode = rm;
        cmd.left.timeout_sec = cmd.right.timeout_sec = 1.0;
        cmd.tcp_target_profile = profile;
        cmd.tcp_target_profile_provided = true;
        for (auto arm : {ArmId::Left, ArmId::Right}) {
            auto& a = arm == ArmId::Left ? cmd.left : cmd.right;
            if (a.mode == ControlMode::TcpPoseTarget) {
                a.has_tcp_target = true;
                a.tcp_target_stand = kin->computeTcpStand(
                    arm, initial, arm == ArmId::Left ? cfg.left_mount : cfg.right_mount);
            }
        }
        return cmd;
    }
    const ServoSnapshot& tick(DualArmCommand cmd) {
        clock.advance();
        cmd.seq = ++seq;
        cmd.host_time_ns = nowSteadyNs();
        buffer.setCommand(cmd);
        require(loop->stepOnce(), "additional external tick failed");
        latest = loop->latestSnapshot();
        require(!latest.fault_latched, "unexpected fault in additional force fixture");
        return latest;
    }
    Pose6D rightSent() const {
        return kin->computeTcpStand(ArmId::Right, latest.right_sent_q_deg, cfg.right_mount);
    }
    void warm(const DualArmCommand& stream, double right_force, double left_force = 0.0) {
        Wrench6D wr, wl;
        wr.fz = right_force;
        wl.fz = left_force;
        right->setWrench(wr);
        left->setWrench(wl);
        for (int i = 0; i < 1800; ++i) tick(stream);
        require(latest.right_force_control.covered, "additional fixture never covered the right arm");
    }
};

bool testInitWithoutAutoTareResetsOnlySelectedArmAndDeduplicates() {
    Fixture f([](DualArmConfig& cfg) {
        cfg.force_torque.auto_tare_after_init_motion.enable = false;
        cfg.force_torque.left = cfg.force_torque.right;
    });
    auto stream = f.command(ControlMode::TcpPoseTarget, ControlMode::TcpPoseTarget);
    f.warm(stream, 0.896, 0.448);
    require(f.latest.left_force_control.deviation_norm_m > 0.0005,
            "peer arm did not establish a standing force deviation");
    const auto right_reset_before = f.latest.right_force_control.reference_reset_count;
    const auto left_reset_before = f.latest.left_force_control.reference_reset_count;
    const double peer_before = f.latest.left_force_control.deviation_norm_m;
    auto init = stream;
    init.right.mode = ControlMode::JointTarget;
    init.right.has_tcp_target = false;
    init.right.has_joint_target = true;
    init.right.q_target_deg = f.latest.right_sent_q_deg;
    init.right.joint_target_profile = JointTargetProfile::InitMotion;
    init.right.init_motion_request_id = 201;
    f.right->setWrench({});
    f.tick(init);
    require(f.latest.init_motion_right.status == "done", "auto-tare-off InitMotion did not complete");
    require(f.latest.right_ft.bias_valid, "auto-tare-off request unexpectedly invalidated bias");
    require(f.latest.right_force_control.reference_reset_count == right_reset_before + 1,
            "fresh InitMotion must reset its reference even when auto-tare is disabled");
    require(norm3(f.latest.right_force_control.reference_deviation_m) < 1e-12,
            "selected InitMotion retained its old force reference");
    require(f.latest.left_force_control.reference_reset_count == left_reset_before &&
            norm3(f.latest.left_force_control.reference_deviation_m) > peer_before * 0.99,
            "single-arm InitMotion disturbed the peer force reference");
    for (int i = 0; i < 30; ++i) {
        f.tick(init);  // New packet sequence, same logical request and same goal.
        require(f.latest.right_force_control.reference_reset_count == right_reset_before + 1,
                "a retransmitted logical InitMotion reset the reference again");
        require(f.latest.left_force_control.reference_reset_count == left_reset_before,
                "InitMotion retransmission reset the peer reference");
    }
    init.right.init_motion_request_id = 202;
    f.tick(init);
    require(f.latest.right_force_control.reference_reset_count == right_reset_before + 2,
            "a new logical request with an unchanged goal was not reset");
    std::cout << "auto-tare-off: selected reference reset, peer preserved, logical request deduplicated\n";
    return true;
}

bool testSampledFollowerTelemetryClearsOnEmergencyStopBypass() {
    Fixture f([](DualArmConfig& cfg) {
        cfg.safety.init_motion_planner.enable = false;
        cfg.safety.self_collision.enable = false;
        cfg.force_torque.auto_tare_after_init_motion.enable = false;
        for (auto& profile : cfg.cartesian_control.tcp_pose_target_profiles) {
            if (profile.name != "flow_infer_smooth") continue;
            auto& follower = profile.ruckig_follower;
            follower.fresh_chunk_replan = true;
            follower.continuous_hold_resume = true;
            follower.output_smd.enable = true;
            follower.output_smd.profile_feedforward = false;
            follower.output_smd.velocity_ff = true;
            follower.output_smd.velocity_ff_linear_gain = 0.8;
        }
    });
    const Pose6D start = f.rightSent();
    const Pose6D delta{0.00001, 0.0, 0.0, 0.0, 0.0, 0.0};
    auto packet = nlohmann::json::parse(zeroDeltaChunk(ArmId::Right, start));
    Pose6D cursor = start;
    for (std::size_t i = 0; i < packet["right"].size(); ++i) {
        cursor = math::composeDeltaLocal(cursor, delta);
        const Eigen::Quaterniond q(math::rotationFromPose(cursor));
        packet["right"][i] = {cursor.x, cursor.y, cursor.z, q.x(), q.y(), q.z(), q.w(), 0.0};
        packet["right_delta"][i] = {delta.x, delta.y, delta.z, 0.0, 0.0, 0.0, 0.0};
    }
    const auto serialized = packet.dump();
    require(f.receiver.acceptPacket(serialized.data(), serialized.size()),
            "telemetry-bypass fixture rejected its moving fresh chunk");
    const auto stream = f.command(ControlMode::Hold, ControlMode::TcpPoseTarget, "flow_infer_smooth");
    bool sampled_motion = false;
    for (int i = 0; i < 30; ++i) {
        const auto& solve = f.tick(stream).right_cartesian_solve;
        if (solve.follower_active && solve.follower_prefilter_stand &&
            solve.follower_sample_velocity && solve.follower_sample_acceleration) {
            const auto& v = *solve.follower_sample_velocity;
            const auto& a = *solve.follower_sample_acceleration;
            sampled_motion = std::hypot(v.x, v.y, v.z) > 1e-9 &&
                             std::hypot(a.x, a.y, a.z) > 1e-9;
        }
        if (sampled_motion) break;
    }
    require(sampled_motion, "telemetry-bypass fixture never recorded nonzero fresh derivatives");

    // Use the real command-buffer / latch / snapshot merge path. Fixture::tick
    // intentionally rejects faults, so the expected emergency-stop ticks use
    // the same public external-step calls directly.
    auto stop = f.command(ControlMode::EmergencyStop, ControlMode::EmergencyStop);
    for (int i = 0; i < 3; ++i) {
        f.clock.advance();
        stop.seq = ++f.seq;
        stop.host_time_ns = nowSteadyNs();
        f.buffer.setCommand(stop);
        require(f.loop->stepOnce(), "emergency-stop bypass tick failed");
        f.latest = f.loop->latestSnapshot();
        require(f.latest.fault_latched &&
                f.loop->latchedFaultReason() == SafetyVerdict::EmergencyStop,
                "telemetry fixture did not take the real emergency-stop bypass");
        for (const auto* solve : {&f.latest.left_cartesian_solve, &f.latest.right_cartesian_solve}) {
            require(!solve->follower_prefilter_stand && !solve->follower_sample_velocity &&
                    !solve->follower_sample_acceleration,
                    "snapshot merge revived stale sampled follower telemetry after EmergencyStop");
        }
    }
    std::cout << "sampled follower telemetry cleared on EmergencyStop and latched bypass ticks\n";
    return true;
}

bool testCoverageLossPreservesFrozenDeviationWithoutPendingChunkDrift() {
    Fixture f([](DualArmConfig& cfg) {
        cfg.safety.init_motion_planner.enable = false;
        cfg.safety.self_collision.enable = false;
        cfg.force_torque.auto_tare_after_init_motion.enable = false;
    });
    auto stream = f.command(ControlMode::Hold, ControlMode::TcpPoseTarget);
    f.warm(stream, 0.896);
    const Pose6D held = f.rightSent();
    const double frozen = f.latest.right_force_control.deviation_norm_m;
    require(frozen > 0.001, "generic coverage fixture has no nonzero deviation");
    const auto reset_count = f.latest.right_force_control.reference_reset_count;
    auto waiting = f.command(ControlMode::Hold, ControlMode::TcpPoseTarget, "flow_infer_smooth");
    waiting.right.tcp_target_stand = held;
    f.right->setWrench({});
    f.right->setWrenchValid(false);
    double max_sent = 0.0, max_stage = 0.0;
    for (int i = 0; i < 70; ++i) {
        if (i == 3) f.right->setWrenchValid(true);
        const auto& s = f.tick(waiting);
        require(!s.right_force_control.covered && !s.right_force_control.reference_strip_enabled,
                "pending recovery falsely enabled nominal subtraction");
        require(std::abs(norm3(s.right_force_control.reference_deviation_m) - frozen) < 1e-9,
                "ordinary coverage loss erased the frozen contact deviation");
        require(s.right_force_control.reference_reset_count == reset_count,
                "ordinary coverage loss was treated as a fresh InitMotion");
        require(!s.right_cartesian_solve.follower_active &&
                s.right_cartesian_solve.stage_tcp_target_stand.has_value(),
                "generic coverage test did not reach the first-chunk hold path");
        max_sent = std::max(max_sent, math::positionDistance(f.rightSent(), held));
        max_stage = std::max(max_stage, math::positionDistance(*s.right_cartesian_solve.stage_tcp_target_stand, held));
    }
    std::cout << "coverage loss: frozen_mm=" << frozen * 1000.0
              << " sent_drift_mm=" << max_sent * 1000.0
              << " stage_drift_mm=" << max_stage * 1000.0 << '\n';
    require(max_sent < 2e-5 && max_stage < 2e-5,
            "pending first-chunk hold repeatedly subtracted the frozen contact deviation");
    return true;
}

bool testCoveredSubmicronDeviationStillComposes() {
    Fixture f([](DualArmConfig& cfg) {
        cfg.safety.init_motion_planner.enable = false;
        cfg.safety.self_collision.enable = false;
        cfg.force_torque.auto_tare_after_init_motion.enable = false;
        cfg.force_control.fold_deviation = false;
        cfg.force_control.hold = cfg.force_control.stream;
        cfg.kinematics.ik.position_tolerance_m = 1e-10;
        cfg.kinematics.ik.orientation_tolerance_rad = 1e-10;
    });
    auto stream = f.command(ControlMode::Hold, ControlMode::TcpPoseTarget);
    f.warm(stream, 0.0002);  // F/k = 0.5 micrometers: below quiescent()'s reporting threshold.
    const Pose6D held = f.rightSent();
    const double tiny = f.latest.right_force_control.deviation_norm_m;
    require(tiny > 2e-7 && tiny < 9e-7, "tiny-deviation fixture missed the submicron band");
    auto hold = f.command(ControlMode::Hold, ControlMode::Hold);
    f.right->setWrench({});
    double max_drift = 0.0;
    for (int i = 0; i < 80; ++i) {
        const auto& s = f.tick(hold);
        require(s.right_force_control.covered && s.right_force_control.reference_strip_enabled,
                "submicron test did not exercise covered subtraction");
        require(!s.right_force_control.hold_engaged, "quiet covered Hold must freeze the deviation dynamics");
        max_drift = std::max(max_drift, math::positionDistance(f.rightSent(), held));
    }
    std::cout << "tiny covered deviation_um=" << tiny * 1e6
              << " frozen Hold drift_um=" << max_drift * 1e6 << '\n';
    require(max_drift < 2e-8,
            "covered submicron strip was not exactly canceled by force compose");
    return true;
}

// Use the real URDF bound, not a fabricated failed-solver response: this elbow
// starts inside +165 deg and the first chunk asks it to cross the actual bound.
// A second chunk returns toward the reachable side during the refusal debounce.
// This covers the servo coordinator, packet cache, Pinocchio IK, downstream joint
// safety, output SMD and the memory plant together. The plant itself has no delay.
bool testFreshChunkResumesAfterActualJointLimitRefusal(bool fresh_execution = true,
                                                      bool profile_feedforward = true,
                                                      double linear_ff_gain = -1.0,
                                                      double nf_linear_hz = 3.5,
                                                      const RuckigFollowerConfig* selected_follower = nullptr) {
    Fixture f([=](DualArmConfig& cfg) {
        cfg.safety.init_motion_planner.enable = false;
        cfg.safety.self_collision.enable = false;
        cfg.force_torque.auto_tare_after_init_motion.enable = false;
        cfg.force_torque.enable = false;
        cfg.force_control.enable = false;
        // This fixture constructs config directly, bypassing the loader's
        // fitted-arm normalization of the generic +/-360 deg defaults.
        cfg.safety.q_min_deg[2] = -165.0;
        cfg.safety.q_max_deg[2] = 165.0;
        // Keep the real URDF/catalog limit and the default strict IK failure
        // policy. No best-effort acceptance, branch-clamp or mocked IK result.
        cfg.kinematics.ik.joint_limit_track_feasible = false;
        for (auto& profile : cfg.cartesian_control.tcp_pose_target_profiles) {
            if (profile.name != "flow_infer_smooth") continue;
            auto& rf = profile.ruckig_follower;
            rf.fresh_chunk_replan = fresh_execution;
            rf.continuous_hold_resume = fresh_execution;
            rf.hold_bounce_resume_sec = 0.5;
            rf.core_time_stretch_enable = true;
            rf.core_time_stretch_max_ratio = 4.0;
            rf.max_linear_velocity_m_s = 0.6;
            rf.max_linear_accel_m_s2 = 12.0;
            rf.max_linear_jerk_m_s3 = 2000.0;
            rf.max_angular_velocity_rad_s = 1.4;
            rf.max_angular_accel_rad_s2 = 40.0;
            rf.max_angular_jerk_rad_s3 = 4000.0;
            rf.consume_steps = 4;
            rf.reserve_steps = 4;
            rf.smoothing_window = 1;
            rf.output_smd.enable = true;
            rf.output_smd.profile_feedforward = profile_feedforward;
            if (linear_ff_gain >= 0.0) {
                rf.output_smd.profile_feedforward = false;
                rf.output_smd.velocity_ff = true;
                rf.output_smd.velocity_ff_linear_gain = linear_ff_gain;
                rf.output_smd.damping_ratio = 1.0;
                rf.output_smd.nf_linear_hz = nf_linear_hz;
                rf.output_smd.nf_angular_hz = 2.5;
            }
            rf.af_damping_beta_lin = rf.af_damping_beta_ang = 1.0;
            rf.corner_velocity_scale = 1.0;
            rf.preview_projection_fault_policy = RuckigProjectionFaultPolicy::Warn;
            if (selected_follower) {
                rf.output_smd = selected_follower->output_smd;
                rf.deadline_jerk_minimization = selected_follower->deadline_jerk_minimization;
            }
        }
    }, {10.0, -20.0, 164.9, 5.0, 25.0, -15.0});
    require(std::abs(f.cfg.safety.q_max_deg[2] - 165.0) < 1e-9,
            "joint-limit fixture must preserve the RB5-850E catalog elbow bound");
    const auto toward = [&](const JointArray& q, double elbow_delta_deg) {
        JointArray goal_q = q;
        goal_q[2] += elbow_delta_deg;
        const auto from = math::se3FromPose(f.kin->computeTcpStand(ArmId::Right, q, f.cfg.right_mount));
        const auto to = math::se3FromPose(f.kin->computeTcpStand(ArmId::Right, goal_q, f.cfg.right_mount));
        const auto local = from.actInv(to);
        const Eigen::Vector3d r = math::log3(local.rotation());
        return Pose6D{local.translation().x(), local.translation().y(), local.translation().z(),
                      r.x(), r.y(), r.z()};
    };
    const auto submit = [&](uint64_t seq, const Pose6D& start, const Pose6D& delta) {
        auto packet = nlohmann::json::parse(zeroDeltaChunk(ArmId::Right, start));
        packet["seq"] = seq;
        Pose6D cursor = start;
        for (std::size_t i = 0; i < packet["right"].size(); ++i) {
            cursor = math::composeDeltaLocal(cursor, delta);
            const Eigen::Quaterniond q(math::rotationFromPose(cursor));
            packet["right"][i] = {cursor.x, cursor.y, cursor.z, q.x(), q.y(), q.z(), q.w(), 0.0};
            packet["right_delta"][i] = {delta.x, delta.y, delta.z, delta.rx, delta.ry, delta.rz, 0.0};
        }
        const std::string text = packet.dump();
        require(f.receiver.acceptPacket(text.data(), text.size()), "IK-refusal chunk packet rejected");
    };
    auto command = f.command(ControlMode::Hold, ControlMode::TcpPoseTarget, "flow_infer_smooth");
    if (!profile_feedforward) {
        // The first cold follower sample is at t=0, with zero sampled velocity.
        // Its solved endpoint already has nonzero velocity for this reachable
        // inward frame. Selecting endpoint velocity would incorrectly seed/move
        // the SMD on this very tick, despite an unchanged position reference.
        const Pose6D cold_reference = f.rightSent();
        const JointArray cold_q = f.latest.right_sent_q_deg;
        submit(1, cold_reference, toward(cold_q, -0.01));
        const auto& first = f.tick(command);
        const auto& solve = first.right_cartesian_solve;
        require(solve.success && solve.follower_active &&
                solve.follower_t_in_seg_sec == 0.0 && solve.stage_tcp_target_stand.has_value(),
                "velocity-only cold sample did not engage at segment time zero");
        require(math::positionDistance(*solve.stage_tcp_target_stand, cold_reference) < 1e-12 &&
                math::orientationDistanceRad(*solve.stage_tcp_target_stand, cold_reference) < 1e-12,
                "velocity-only cold SMD used the future endpoint velocity at sample time zero");
        for (int j = 0; j < kDof; ++j)
            require(std::abs(first.right_sent_q_deg[j] - cold_q[j]) < 1e-8,
                    "velocity-only cold sample moved the actual sent joint target");
        for (int i = 0; i < 3; ++i) f.tick(command);
    } else {
        submit(1, f.rightSent(), {});
        for (int i = 0; i < 4; ++i) f.tick(command);
    }
    require(f.latest.right_cartesian_solve.success && f.latest.right_cartesian_solve.follower_active,
            "joint-limit fixture could not engage at its legal initial pose");
    submit(2, f.rightSent(), toward(f.latest.right_sent_q_deg, 1.0));
    bool refused = false;
    int approach_ticks = 0;
    for (; approach_ticks < 150; ++approach_ticks) {
        f.tick(command);
        const auto& solve = f.latest.right_cartesian_solve;
        if (solve.attempted && !solve.success) {
            require(solve.reason == "joint_limit" && solve.ik_joint_limit_worst_index == 2,
                    "fixture failed for a reason other than the actual elbow bound: " + solve.reason);
            refused = true;
            break;
        }
    }
    require(refused, "outward chunk never reached a real Pinocchio joint-limit refusal");
    const uint64_t warm_before = f.latest.right_cartesian_solve.follower_warm_resume_count;
    // Replace the window while paused. Do not refresh it again: recovery must
    // consume this cached request, not depend on another conveniently timed frame.
    submit(3, f.rightSent(), toward(f.latest.right_sent_q_deg, -0.25));
    int blocked_ticks = 0, blocked_resets = 0, resumed_ticks = 0;
    bool resumed = false;
    double max_held_stage_error_m = 0.0, max_held_stage_error_rad = 0.0;
    double max_settled_hold_joint_step_deg = 0.0;
    double first_resume_stage_step_m = 0.0, first_resume_stage_step_rad = 0.0;
    double first_resume_joint_step_deg = 0.0, resumed_elbow_deg = 0.0;
    for (int i = 0; i < 100; ++i) {
        const Pose6D previous_sent = f.rightSent();
        const JointArray previous_q = f.latest.right_sent_q_deg;
        const auto& s = f.tick(command);
        const auto& solve = s.right_cartesian_solve;
        require(solve.stage_tcp_target_stand.has_value(), "refusal/resume omitted the actual stage target");
        double joint_step = 0.0;
        for (int j = 0; j < kDof; ++j)
            joint_step = std::max(joint_step, std::abs(s.right_sent_q_deg[j] - previous_q[j]));
        if (solve.cartesian_solve_blocked_recent) {
            require(!resumed, "reachable recovery chunk caused another IK refusal");
            ++blocked_ticks;
            blocked_resets += solve.follower_output_smd_reseeded ? 1 : 0;
            require(solve.follower_active && solve.success,
                    "debounce hold did not solve the prior sent reference successfully");
            max_held_stage_error_m = std::max(max_held_stage_error_m,
                math::positionDistance(*solve.stage_tcp_target_stand, previous_sent));
            max_held_stage_error_rad = std::max(max_held_stage_error_rad,
                math::orientationDistanceRad(*solve.stage_tcp_target_stand, previous_sent));
            // The final joint acceleration clamp may decelerate for a few ticks
            // after refusal. Its settled tail must not keep creeping.
            if (blocked_ticks > 20)
                max_settled_hold_joint_step_deg = std::max(max_settled_hold_joint_step_deg, joint_step);
        } else if (solve.follower_warm_resume_count > warm_before) {
            require(solve.success, "first warm-resume solve failed");
            if (!resumed) {
                resumed = true;
                require(solve.follower_wire_seq == 3, "warm resume did not consume the cached fresh frame");
                require(solve.follower_output_smd_reseeded, "resume did not reseed output from the held reference");
                first_resume_stage_step_m = math::positionDistance(*solve.stage_tcp_target_stand, previous_sent);
                first_resume_stage_step_rad = math::orientationDistanceRad(*solve.stage_tcp_target_stand, previous_sent);
                first_resume_joint_step_deg = joint_step;
                resumed_elbow_deg = s.right_sent_q_deg[2];
            }
            if (++resumed_ticks == 20) break;
        }
    }
    std::cout << "actual Pinocchio joint-limit refusal: fresh=" << fresh_execution
              << " profile_feedforward=" << profile_feedforward
              << " linear_ff_gain=" << linear_ff_gain
              << " nf_linear_hz=" << nf_linear_hz
              << " approach_ticks=" << approach_ticks
              << " blocked_ticks=" << blocked_ticks << " blocked_output_resets=" << blocked_resets
              << " held_stage_error_um=" << max_held_stage_error_m * 1e6
              << " settled_hold_joint_step_deg=" << max_settled_hold_joint_step_deg
              << " first_resume_stage_step_um=" << first_resume_stage_step_m * 1e6
              << " first_resume_joint_step_deg=" << first_resume_joint_step_deg
              << " recovery_elbow_delta_deg=" << f.latest.right_sent_q_deg[2] - resumed_elbow_deg << '\n';
    require(blocked_ticks >= 40 && blocked_ticks <= 60 && resumed && resumed_ticks == 20,
            "actual 100 ms refusal debounce and warm resume were not fully exercised");
    require(blocked_resets == 1, "output SMD reset repeatedly during a single IK-refusal hold");
    require(max_held_stage_error_m < 1e-9 && max_held_stage_error_rad < 1e-9,
            "paused follower emitted its rejected plan instead of the last sent reference");
    require(max_settled_hold_joint_step_deg < 1e-7,
            "settled IK-refusal hold accumulated joint motion");
    require(first_resume_stage_step_m < 2e-6 && first_resume_stage_step_rad < 2e-6 &&
            first_resume_joint_step_deg < 1e-5,
            "first recovery tick jumped from the held command");
    if (linear_ff_gain < 0.0 && !selected_follower) {
        require(f.latest.right_sent_q_deg[2] < resumed_elbow_deg - 1e-4,
                "fresh reachable chunk never resumed actual command motion away from the bound");
    } else {
        // The candidate explicitly trades some following speed for filtering.
        // Report its 40 ms recovery displacement for comparison instead of
        // assuming the original FF-on displacement remains the right threshold.
        require(f.latest.right_sent_q_deg[2] < resumed_elbow_deg - 1e-8,
                "candidate remained frozen after a reachable fresh chunk resumed");
    }
    return true;
}

bool runCase(ArmId selected, bool rotation, bool fresh_execution = false,
             double linear_ff_gain = -1.0, double nf_linear_hz = 3.5,
             const RuckigFollowerConfig* selected_follower = nullptr) {
    ManualClock clock;
    auto cfg = fixtureConfig(selected, rotation);
    for (auto& profile : cfg.cartesian_control.tcp_pose_target_profiles) {
        if (profile.name == "flow_infer_smooth") {
            profile.ruckig_follower.fresh_chunk_replan = fresh_execution;
            profile.ruckig_follower.continuous_hold_resume = fresh_execution;
            if (linear_ff_gain >= 0.0) {
                auto& smd = profile.ruckig_follower.output_smd;
                smd.enable = true;
                smd.profile_feedforward = false;
                smd.velocity_ff = true;
                smd.velocity_ff_linear_gain = linear_ff_gain;
                smd.damping_ratio = 1.0;
                smd.nf_linear_hz = nf_linear_hz;
                smd.nf_angular_hz = 2.5;
            }
            if (selected_follower) {
                profile.ruckig_follower.output_smd = selected_follower->output_smd;
                profile.ruckig_follower.deadline_jerk_minimization = selected_follower->deadline_jerk_minimization;
            }
        }
    }
    std::cout << "force resume fresh_execution=" << fresh_execution
              << " linear_ff_gain=" << linear_ff_gain
              << " nf_linear_hz=" << nf_linear_hz << '\n';
    auto kin = std::make_shared<PinocchioKinematics>(cfg.kinematics);
    const JointArray initial{10.0, -20.0, 35.0, 5.0, 25.0, -15.0};
    auto left = std::make_unique<MemoryPlant>(ArmId::Left, initial);
    auto right = std::make_unique<MemoryPlant>(ArmId::Right, initial);
    MemoryPlant* plant = selected == ArmId::Left ? left.get() : right.get();
    const auto& mount = selected == ArmId::Left ? cfg.left_mount : cfg.right_mount;
    CommandBuffer buffer;
    ChunkFrameReceiver receiver("");  // acceptPacket only; never start a network receiver.
    DualArmServoLoop loop(std::move(left), std::move(right), cfg, &buffer, nullptr, kin);
    loop.setChunkFrameReceiver(&receiver);
    loop.enableExternalStepping();
    require(loop.start(), "external-stepped servo loop failed to start");
    uint64_t seq = 0;
    const auto commandFor = [&](ControlMode mode, const std::string& profile) {
        DualArmCommand cmd;
        cmd.left.arm_id = ArmId::Left;
        cmd.right.arm_id = ArmId::Right;
        cmd.left.mode = cmd.right.mode = ControlMode::Hold;
        (selected == ArmId::Left ? cmd.left : cmd.right).mode = mode;
        cmd.tcp_target_profile = profile;
        cmd.tcp_target_profile_provided = true;
        cmd.left.timeout_sec = cmd.right.timeout_sec = 1.0;
        return cmd;
    };
    const auto tick = [&](DualArmCommand cmd) {
        clock.advance();
        cmd.seq = ++seq;
        cmd.host_time_ns = nowSteadyNs();
        buffer.setCommand(cmd);
        require(loop.stepOnce(), "external servo tick failed");
        auto snap = loop.latestSnapshot();
        require(!snap.fault_latched, "unexpected fixture fault: " +
            (snap.latched_fault_context ? snap.latched_fault_context->reason
                                        : std::string(toString(snap.safety_verdict))));
        return snap;
    };
    const auto fcOf = [&](const ServoSnapshot& s) -> const ForceControlTelemetry& {
        return selected == ArmId::Left ? s.left_force_control : s.right_force_control;
    };
    const auto ftOf = [&](const ServoSnapshot& s) -> const FtTelemetry& {
        return selected == ArmId::Left ? s.left_ft : s.right_ft;
    };
    const auto sentOf = [&](const ServoSnapshot& s) -> const JointArray& {
        return selected == ArmId::Left ? s.left_sent_q_deg : s.right_sent_q_deg;
    };
    auto arm_motion = commandFor(ControlMode::ArmMotion, "plain_force_fixture");
    arm_motion.left.mode = arm_motion.right.mode = ControlMode::ArmMotion;
    tick(arm_motion);
    auto stream = commandFor(ControlMode::TcpPoseTarget, "plain_force_fixture");
    auto& stream_arm = selected == ArmId::Left ? stream.left : stream.right;
    stream_arm.has_tcp_target = true;
    stream_arm.tcp_target_stand = kin->computeTcpStand(selected, initial, mount);
    Wrench6D force;
    force.fz = 0.896;  // F/k equilibrium = 2.24 mm, matching the incident's scale.
    force.tz = rotation ? 0.008 : 0.0;
    plant->setWrench(force);
    ServoSnapshot snap;
    for (int i = 0; i < 6000; ++i) snap = tick(stream);
    require(fcOf(snap).covered && fcOf(snap).compose_applied,
            "fixture never applied its force overlay");
    const double old_deviation = fcOf(snap).deviation_norm_m;
    const double old_rotation = fcOf(snap).deviation_norm_rad;
    require(old_deviation > 0.0015 && old_deviation < 0.003,
            "fixture did not establish a 2.24 mm-scale standing deviation");
    if (rotation) require(old_rotation > 0.0003, "rotation fixture has no standing rotation");

    plant->setWrench({});
    auto init = commandFor(ControlMode::JointTarget, "plain_force_fixture");
    auto& init_arm = selected == ArmId::Left ? init.left : init.right;
    init_arm.has_joint_target = true;
    init_arm.q_target_deg = sentOf(snap);
    init_arm.joint_target_profile = JointTargetProfile::InitMotion;
    init_arm.init_motion_request_id = 73;
    snap = tick(init);
    const auto& init_status = selected == ArmId::Left ? snap.init_motion_left : snap.init_motion_right;
    require(init_status.status == "done", "fixture InitMotion did not complete as a measured no-op");
    const Pose6D init_pose = kin->computeTcpStand(selected, sentOf(snap), mount);
    const auto old_generation = ftOf(snap).bias_generation;
    auto hold = commandFor(ControlMode::Hold, "plain_force_fixture");
    bool saw_invalid_bias = false;
    bool tare_ready = false;
    for (int i = 0; i < 650; ++i) {
        snap = tick(hold);
        saw_invalid_bias |= !ftOf(snap).bias_valid;
        if (ftOf(snap).bias_valid && ftOf(snap).bias_generation > old_generation) {
            tare_ready = true;
            break;
        }
    }
    require(saw_invalid_bias && tare_ready, "real auto-tare invalidation/sample/commit path was not exercised");
    require(!fcOf(snap).covered, "fixture must resume during coverage recovery");

    // Intentionally withhold every chunk, as during the 66-tick incident wait.
    // The command carries a TCP target so the strict preview branch really runs;
    // that branch must preserve the live emitted pose while no frame is present.
    auto waiting = commandFor(ControlMode::TcpPoseTarget, "flow_infer_smooth");
    auto& wait_arm = selected == ArmId::Left ? waiting.left : waiting.right;
    wait_arm.has_tcp_target = true;
    wait_arm.tcp_target_stand = init_pose;
    double max_stage_m = 0.0, max_stage_rad = 0.0, max_sent_m = 0.0, max_sent_rad = 0.0;
    int uncovered_ticks = 0;
    bool recovered = false;
    double recover_step_m = 0.0, recover_step_rad = 0.0;
    Pose6D previous_pose = kin->computeTcpStand(selected, sentOf(snap), mount);
    for (int i = 0; i < 300; ++i) {
        snap = tick(waiting);
        const auto& solve = selected == ArmId::Left ? snap.left_cartesian_solve : snap.right_cartesian_solve;
        require(!solve.follower_active, "a follower activated without a chunk");
        require(solve.stage_tcp_target_stand.has_value(), "strict first-chunk hold did not expose its stage target");
        const Pose6D sent = kin->computeTcpStand(selected, sentOf(snap), mount);
        if (fcOf(snap).covered) {
            recovered = true;
            recover_step_m = math::positionDistance(sent, previous_pose);
            recover_step_rad = math::orientationDistanceRad(sent, previous_pose);
            break;
        }
        ++uncovered_ticks;
        max_stage_m = std::max(max_stage_m, math::positionDistance(*solve.stage_tcp_target_stand, init_pose));
        max_stage_rad = std::max(max_stage_rad, math::orientationDistanceRad(*solve.stage_tcp_target_stand, init_pose));
        max_sent_m = std::max(max_sent_m, math::positionDistance(sent, init_pose));
        max_sent_rad = std::max(max_sent_rad, math::orientationDistanceRad(sent, init_pose));
        if (uncovered_ticks == 66) {
            std::cout << toString(selected) << " first_66_wait_ticks: stage_drift_mm="
                      << max_stage_m * 1000.0 << " sent_drift_mm="
                      << max_sent_m * 1000.0 << " sent_rotation_rad="
                      << max_sent_rad << '\n';
            require(max_stage_m < 2e-5 && max_sent_m < 2e-5 &&
                    max_stage_rad < 2e-5 && max_sent_rad < 2e-5,
                    "first 66 uncovered wait ticks repeatedly moved the emitted pose");
        }
        previous_pose = sent;
    }
    std::cout << toString(selected) << " rotation=" << rotation
              << " prior_deviation_mm=" << old_deviation * 1000.0
              << " prior_rotation_rad=" << old_rotation
              << " uncovered_ticks=" << uncovered_ticks
              << " stage_drift_mm=" << max_stage_m * 1000.0
              << " sent_drift_mm=" << max_sent_m * 1000.0
              << " stage_rotation_rad=" << max_stage_rad
              << " sent_rotation_rad=" << max_sent_rad
              << " recover_step_mm=" << recover_step_m * 1000.0
              << " recover_rotation_rad=" << recover_step_rad << '\n';
    require(uncovered_ticks >= 66 && recovered, "coverage recovery window was not fully exercised");
    require(max_stage_m < 2e-5 && max_sent_m < 2e-5,
            "uncovered first-chunk hold accumulated translation after InitMotion");
    require(max_stage_rad < 2e-5 && max_sent_rad < 2e-5,
            "uncovered first-chunk hold accumulated rotation after InitMotion");
    // Recovery may take one small dynamics step as the zero-wrench spring resumes;
    // it must not apply or remove the entire old standing deviation at once.
    require(recover_step_m < 2e-5 && recover_step_rad < 2e-5,
            "coverage recovery applied a discontinuous pose jump");

    // Feed the first real wire-format frame through the public in-process
    // receiver, then run several actual follower ticks after the handoff.
    // Zero deltas isolate handoff continuity from a policy's desired motion.
    const Pose6D before_chunk = kin->computeTcpStand(selected, sentOf(snap), mount);
    const auto packet = zeroDeltaChunk(selected, before_chunk);
    require(receiver.acceptPacket(packet.data(), packet.size()), "first chunk packet was rejected");
    bool first_chunk_active = false;
    double engage_drift_m = 0.0, engage_drift_rad = 0.0;
    for (int i = 0; i < 6; ++i) {
        snap = tick(waiting);
        const auto& solve = selected == ArmId::Left ? snap.left_cartesian_solve : snap.right_cartesian_solve;
        first_chunk_active |= solve.follower_active;
        const Pose6D sent = kin->computeTcpStand(selected, sentOf(snap), mount);
        engage_drift_m = std::max(engage_drift_m, math::positionDistance(sent, before_chunk));
        engage_drift_rad = std::max(engage_drift_rad, math::orientationDistanceRad(sent, before_chunk));
    }
    require(first_chunk_active, "first valid chunk did not engage the actual delta follower");
    require(engage_drift_m < 2e-5 && engage_drift_rad < 2e-5,
            "zero-delta first chunk moved the recovered emitted reference");
    std::cout << toString(selected) << " first_chunk_engaged=" << first_chunk_active
              << " engage_drift_um=" << engage_drift_m * 1e6
              << " engage_rotation_rad=" << engage_drift_rad << '\n';
    loop.stop();
    return true;
}

#ifdef RB_SERVO_ENABLE_PREVIEW_EXECUTION
bool testPreviewExecutionForceTareResume() {
    Fixture f([](DualArmConfig& cfg) {
        const auto root=std::filesystem::path(__FILE__).parent_path().parent_path();
        const auto tracked=loadConfigFromYaml((root/"config/stack_real.yaml").string());
        const auto& profiles=tracked.cartesian_control.tcp_pose_target_profiles;
        const auto selected=std::find_if(profiles.begin(),profiles.end(),
            [](const auto& p){return p.name=="flow_infer_preview";});
        require(selected!=profiles.end(),"preview execution profile absent");
        cfg.cartesian_control.tcp_pose_target_profiles.push_back(*selected);
        // Explicit synthetic force-filter corner makes a double prepare visible
        // numerically. Motion caps and the tracked preview profile are unchanged.
        cfg.force_control.wrench_filter_hz=8.0;
    });
    const auto plain=f.command(ControlMode::Hold,ControlMode::TcpPoseTarget);
    f.warm(plain,.896);
    require(f.latest.right_force_control.deviation_norm_m>.0005,
            "preview force fixture has no standing overlay");
    uint64_t wire=0;
    const auto publishZero=[&] {
        auto packet=nlohmann::json::parse(zeroDeltaChunk(ArmId::Right,f.rightSent()));
        packet["seq"]=++wire;packet["host_time_ns"]=nowSteadyNs();
        const auto body=packet.dump();
        require(f.receiver.acceptPacket(body.data(),body.size()),"preview fixture chunk rejected");
    };
    const auto pacedTick=[&](const DualArmCommand& command) {
        f.tick(command);
        // The actual asynchronous solver gets wall time; the in-memory plant
        // remains stepped only by the explicit production servo tick above.
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
    };
    auto preview=f.command(ControlMode::Hold,ControlMode::TcpPoseTarget,"flow_infer_preview");
    publishZero();
    for(int i=0;i<40&&!f.latest.right_cartesian_solve.preview_execution.active;++i) pacedTick(preview);
    require(f.latest.right_cartesian_solve.preview_execution.active,
            "force-covered preview never accepted its first command");
    require(f.latest.right_force_control.covered&&f.latest.right_force_control.compose_applied,
            "preview bypassed the covered force overlay");
    const auto prior=f.latest.right_force_control.wrench_filtered_stand;
    Wrench6D force;force.fz=1.792;f.right->setWrench(force);pacedTick(preview);
    const auto& fc=f.latest.right_force_control;
    const double dt=1./f.cfg.servo.rate_hz;
    const double alpha=dt/(1./(2.*M_PI*f.cfg.force_control.wrench_filter_hz)+dt);
    const Eigen::Vector3d old(prior.fx,prior.fy,prior.fz);
    const Eigen::Vector3d raw(fc.wrench_stand.fx,fc.wrench_stand.fy,fc.wrench_stand.fz);
    const Eigen::Vector3d actual(fc.wrench_filtered_stand.fx,fc.wrench_filtered_stand.fy,fc.wrench_filtered_stand.fz);
    require((raw-old).norm()>.1,"force step was not observed by the production pipeline");
    require((actual-(old+alpha*(raw-old))).norm()<1e-10,
            "preview early preparation and overlay applied the force filter twice");

    const auto old_epoch=f.latest.right_cartesian_solve.preview_execution.epoch;
    const auto old_generation=f.latest.right_ft.bias_generation;
    const auto old_resets=f.latest.right_force_control.reference_reset_count;
    f.right->setWrench({});
    auto init=f.command(ControlMode::Hold,ControlMode::JointTarget,"flow_infer_preview");
    init.right.has_joint_target=true;init.right.q_target_deg=f.latest.right_sent_q_deg;
    init.right.joint_target_profile=JointTargetProfile::InitMotion;
    init.right.init_motion_request_id=907;
    f.tick(init);
    require(f.latest.init_motion_right.status=="done","preview InitMotion no-op did not complete");
    require(!f.latest.right_cartesian_solve.preview_execution.active,
            "InitMotion retained active preview authority");
    require(f.latest.right_force_control.reference_reset_count==old_resets+1,
            "preview InitMotion did not reset its force reference exactly once");
    const Pose6D init_pose=f.rightSent();
    for(int i=0;i<15;++i){f.tick(init);
        require(f.latest.right_force_control.reference_reset_count==old_resets+1,
                "retransmitted InitMotion reset preview force state twice");}
    preview.right.tcp_target_stand=init_pose;
    bool saw_invalid=false,recovered=false;
    double maximum_wait_drift=0.;
    // Settle and recovery each require 250 ticks in this fixture, and the
    // production tare accumulator itself requires another 250 samples. Keep
    // a bounded margin for stage transitions; do not shorten any real wait.
    const int lifecycle_ticks=static_cast<int>(std::ceil(
        (f.cfg.force_torque.auto_tare_after_init_motion.settle_sec+
         f.cfg.force_control.coverage_recover_sec)*f.cfg.servo.rate_hz))+250+100;
    for(int i=0;i<lifecycle_ticks;++i){
        f.tick(preview);saw_invalid=saw_invalid||!f.latest.right_ft.bias_valid;
        const auto& solve=f.latest.right_cartesian_solve;
        require(!solve.preview_execution.active&&!solve.follower_active,
                "pre-Init cached chunk revived while tare/coverage/fresh-frame waited");
        maximum_wait_drift=std::max(maximum_wait_drift,math::positionDistance(f.rightSent(),init_pose));
        // A planner wait deliberately publishes joint Hold, so no overlay is
        // composed and `covered` remains false. The pre-conversion eligibility
        // latch is the correct recovery predicate before a fresh frame arrives.
        if(f.latest.right_force_control.reference_strip_enabled&&f.latest.right_ft.bias_valid&&
           f.latest.right_ft.bias_generation>old_generation){recovered=true;break;}
    }
    if(!saw_invalid||!recovered)std::cerr<<"preview tare diagnostic invalid="<<saw_invalid
        <<" recovered="<<recovered<<" bias_valid="<<f.latest.right_ft.bias_valid
        <<" generation="<<f.latest.right_ft.bias_generation<<" prior_generation="<<old_generation
        <<" covered="<<f.latest.right_force_control.covered
        <<" coverage_streak="<<f.latest.right_force_control.coverage_recover_streak
        <<" coverage_needed="<<f.latest.right_force_control.coverage_recover_needed
        <<" tare_state="<<f.latest.right_ft.tare_state
        <<" coverage_reason="<<f.latest.right_force_control.coverage_reason
        <<" strip_enabled="<<f.latest.right_force_control.reference_strip_enabled
        <<" command_mode="<<toString(f.latest.command.right.mode)
        <<" has_tcp="<<f.latest.command.right.has_tcp_target
        <<" requested_mode="<<toString(preview.right.mode)
        <<" requested_tcp="<<preview.right.has_tcp_target
        <<" init_status="<<f.latest.init_motion_right.status<<'\n';
    require(saw_invalid&&recovered,"preview real tare invalidation/commit/coverage lifecycle incomplete");
    require(maximum_wait_drift<2e-5,"preview wait repeatedly subtracted the frozen force reference");
    publishZero();
    for(int i=0;i<40&&!f.latest.right_cartesian_solve.preview_execution.active;++i) pacedTick(preview);
    require(f.latest.right_cartesian_solve.preview_execution.active,
            "fresh post-tare chunk did not resume preview execution");
    require(f.latest.right_force_control.covered&&f.latest.right_force_control.reference_strip_enabled,
            "post-tare preview did not restore the actual covered overlay");
    // The massless zero-wrench fixture has zero deviation after tare. The law
    // runs, but compose_applied correctly stays false for an identity transform.
    require(f.latest.right_force_control.deviation_norm_m<1e-12,
            "post-tare preview restored an old standing force deviation");
    require(f.latest.right_cartesian_solve.preview_execution.epoch>old_epoch,
            "post-Init preview retained its previous epoch");
    require(math::positionDistance(f.rightSent(),init_pose)<2e-5,
            "post-tare zero chunk jumped from the held pose");
    require(!f.latest.right_cartesian_solve.follower_output_smd_active,
            "new preview profile unexpectedly activated output low-pass");
    std::cout<<"preview force: single filter update, accepted covered execution, InitMotion dedup, "
             <<"tare/coverage wait, fresh-epoch resume; maximum wait drift um="<<maximum_wait_drift*1e6<<'\n';
    return true;
}
#endif

void testLinearConditionerLifecycle(double nf_linear_hz, double linear_ff_gain) {
    require(std::isfinite(nf_linear_hz) && nf_linear_hz > 0.0 &&
            std::isfinite(linear_ff_gain) && linear_ff_gain >= 0.0 && linear_ff_gain <= 1.0,
            "conditioner audit needs positive finite frequency and linear gain in [0,1]");
    std::cout << "linear conditioner lifecycle nf_linear_hz=" << nf_linear_hz
              << " linear_ff_gain=" << linear_ff_gain << '\n';
    // Retain force/tare/coverage/first-frame assertions. Angular filtering stays
    // at the baseline 2.5 Hz, velocity_ff=true, damping_ratio=1 configuration.
    runCase(ArmId::Right, false, true, linear_ff_gain, nf_linear_hz);
    runCase(ArmId::Left, true, true, linear_ff_gain, nf_linear_hz);
    testFreshChunkResumesAfterActualJointLimitRefusal(true, false, linear_ff_gain, nf_linear_hz);
}

void testLinearConditionerCandidates() {
    testLinearConditionerLifecycle(4.0, 0.0);
    testLinearConditionerLifecycle(6.0, 0.2);
    testLinearConditionerLifecycle(8.0, 0.3660254037844386);
}
}  // namespace

int main(int argc, char** argv) {
    try {
#ifdef RB_SERVO_ENABLE_PREVIEW_EXECUTION
        if(argc==2&&std::string(argv[1])=="--preview-execution-only"){
            testPreviewExecutionForceTareResume();return 0;
        }
#endif
        if (argc == 2 && std::string(argv[1]) == "--telemetry-bypass-only") {
            testSampledFollowerTelemetryClearsOnEmergencyStopBypass();
            return 0;
        }
        if ((argc == 2 || argc == 4) && std::string(argv[1]) == "--conditioner-resume-audit") {
            if (argc == 4) testLinearConditionerLifecycle(std::stod(argv[2]), std::stod(argv[3]));
            else testLinearConditionerCandidates();
            return 0;
        }
        if (argc == 2 && std::string(argv[1]) == "--ik-refusal-only") {
            testFreshChunkResumesAfterActualJointLimitRefusal();
            return 0;
        }
        if (argc == 2 && std::string(argv[1]) == "--ik-refusal-no-profile-ff") {
            testFreshChunkResumesAfterActualJointLimitRefusal(true, false);
            return 0;
        }
        // Negative control, intentionally expected to fail the new invariant.
        if (argc == 2 && std::string(argv[1]) == "--ik-refusal-legacy") {
            testFreshChunkResumesAfterActualJointLimitRefusal(false);
            return 0;
        }
        runCase(rb_servo::ArmId::Right, false);
        testSampledFollowerTelemetryClearsOnEmergencyStopBypass();
        runCase(rb_servo::ArmId::Left, true);
        runCase(rb_servo::ArmId::Right, false, true);
        runCase(rb_servo::ArmId::Left, true, true);
        testInitWithoutAutoTareResetsOnlySelectedArmAndDeduplicates();
        testCoverageLossPreservesFrozenDeviationWithoutPendingChunkDrift();
        testCoveredSubmicronDeviationStillComposes();
        testFreshChunkResumesAfterActualJointLimitRefusal();
        testFreshChunkResumesAfterActualJointLimitRefusal(true, false);
        testLinearConditionerCandidates();
        // Load only the selected motion conditioning knobs into the existing
        // memory-plant fixtures. Parsing this YAML never constructs a backend.
        const auto stack = loadConfigFromYaml((std::filesystem::path(__FILE__).parent_path().parent_path() /
                                               "config/stack_real.yaml").string());
        const auto& profiles = stack.cartesian_control.tcp_pose_target_profiles;
        const auto selected = std::find_if(profiles.begin(), profiles.end(),
            [](const auto& p) { return p.name == "flow_infer_fresh"; });
        require(selected != profiles.end(), "selected real follower profile missing");
        const auto& rf = selected->ruckig_follower;
        runCase(ArmId::Left, false, true, -1.0, 3.5, &rf);
        runCase(ArmId::Right, true, true, -1.0, 3.5, &rf);
        testFreshChunkResumesAfterActualJointLimitRefusal(true, false, -1.0, 3.5, &rf);
        std::cout << "force overlay resume regressions passed\n";
        return 0;
    } catch (const std::exception& e) {
        rb_servo::setExternalSteadyNs(0);
        std::cerr << "force overlay resume regression: " << e.what() << '\n';
        return 1;
    }
}
