// Regression for repeated subtraction of a frozen force deviation while a
// delta_preview source waits for its first chunk after InitMotion/tare.
// Runs the real servo tick + Pinocchio FK/IK against an in-memory plant. No
// receiver is started, no socket/device/model/controller is contacted.
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
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

namespace rb_servo {
struct FtPipelineTestAccess {
    static bool sample(DualArmServoLoop& loop, ArmId arm, const RobotState& state,
                       const JointArray& sent, Wrench6D* output) {
        (arm == ArmId::Left ? loop.left_prev_sent_q_deg_ : loop.right_prev_sent_q_deg_) = sent;
        const bool valid = loop.stepFtPipeline(arm, state);
        *output = (arm == ArmId::Left ? loop.left_ft_pipeline_ : loop.right_ft_pipeline_)
                      .compStandNoDeadzone();
        return valid;
    }
};
}

#ifdef RB_SERVO_ENABLE_PREVIEW_EXECUTION
namespace rb_servo {
// Inject only a planning event. The production force pipeline, finite brake,
// observation fence, fresh-frame admission, FK/IK and dispatch remain intact.
struct PreviewRecoveryTestAccess {
    static void request(DualArmServoLoop& loop,PreviewRecoveryCause cause) {
        loop.preview_recovery_request_=cause;
    }
    static control::PreviewMotionSample rightAccepted(const DualArmServoLoop& loop) {
        if(!loop.preview_executor_[1])throw std::runtime_error("missing right preview executor");
        return loop.preview_executor_[1]->acceptedSample();
    }
    static control::FollowerOutputKinematics rightRaw(const DualArmServoLoop& loop) {
        return loop.right_chunk_follower_.outputKinematics();
    }
};
}
#endif

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
    fc.gate_peak_force_n = 12.0;
    fc.gate_rest_force_n = 10.0;
    fc.gate_peak_vel_mm_s = 4.0;
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

void testWrenchUsesAcquiredJointPose() {
    const auto stack = loadConfigFromYaml((std::filesystem::path(__FILE__).parent_path().parent_path() /
                                           "config/stack_real.yaml").string());
    Fixture f([&](DualArmConfig& cfg) {
        cfg.force_control.enable = false;
        cfg.force_torque.auto_tare_after_init_motion.enable = false;
        cfg.force_torque.left = stack.force_torque.left;
        cfg.force_torque.right = stack.force_torque.right;
        for (auto* sensor : {&cfg.force_torque.left, &cfg.force_torque.right}) {
            sensor->enable = true;
            sensor->bias_from_config = true;
            sensor->bias_force_n = {0.2, -0.3, 0.4};
            sensor->bias_torque_nm = {0.01, 0.02, -0.03};
        }
    });
    const auto vec = [](const std::array<double,3>& v) { return Eigen::Vector3d(v[0],v[1],v[2]); };
    for (const auto arm : {ArmId::Left, ArmId::Right}) {
        const auto& ft = arm == ArmId::Left ? f.cfg.force_torque.left : f.cfg.force_torque.right;
        const auto& mount = arm == ArmId::Left ? f.cfg.left_mount : f.cfg.right_mount;
        Eigen::Matrix3d axes;
        axes.col(0)=vec(ft.axis_fx);axes.col(1)=vec(ft.axis_fy);axes.col(2)=vec(ft.axis_fz);
        require(axes.determinant()<0, "fixture must exercise calibrated left-handed sensor axes");
        for (const double wrist : {-40.0, 0.0, 40.0}) {
            RobotState state;
            state.q_actual_deg=f.initial;
            state.q_actual_deg[4]+=wrist;
            state.has_valid_joint_state=state.q_actual_valid=state.eft_valid=true;
            state.host_time_ns=nowSteadyNs();
            const auto flange=f.kin->computeFlangeStand(arm,state.q_actual_deg,mount);
            require(flange.has_value(), "measured FK unavailable in wrench fixture");
            const Eigen::Matrix3d rotation=math::rotationFromPose(*flange);
            const Eigen::Vector3d gravity=rotation.transpose()*Eigen::Vector3d(0,0,-9.80665)*ft.tool_mass_kg;
            JointArray sent=state.q_actual_deg;
            sent[3]+=25;sent[4]-=30; // an in-flight command, never sent to a backend
            for (const Eigen::Vector3d external : {Eigen::Vector3d(0,0,0),Eigen::Vector3d(4,-6,2)}) {
                const Eigen::Vector3d force=rotation.transpose()*external;
                const Eigen::Vector3d raw_force=axes.transpose()*(gravity+force+vec(ft.bias_force_n));
                const Eigen::Vector3d raw_torque=axes.transpose()*(
                    (vec(ft.tool_com_mm)*1e-3).cross(gravity)+
                    (vec(ft.tool_xyz_mm)*1e-3).cross(force)+vec(ft.bias_torque_nm));
                state.eft_wrench={raw_force.x(),raw_force.y(),raw_force.z(),
                                  raw_torque.x(),raw_torque.y(),raw_torque.z()};
                Wrench6D compensated;
                require(FtPipelineTestAccess::sample(*f.loop,arm,state,sent,&compensated),
                        "valid acquired wrench rejected");
                require((Eigen::Vector3d(compensated.fx,compensated.fy,compensated.fz)-external).norm()<1e-7,
                        "sent target contaminated measured gravity or external force direction");
                require(Eigen::Vector3d(compensated.tx,compensated.ty,compensated.tz).norm()<1e-7,
                        "measured wrench lost its TCP reference point");
            }
            Wrench6D compensated;
            state.q_actual_valid=false; // q_ref alone cannot locate the physical sensor
            require(!FtPipelineTestAccess::sample(*f.loop,arm,state,sent,&compensated),
                    "invalid acquired joints accepted via command fallback");
            require(Eigen::Vector3d(compensated.fx,compensated.fy,compensated.fz).isZero(0),
                    "invalid acquired joints fabricated a gravity-derived force");
            state.q_actual_valid=true;
            state.q_actual_deg[0]=std::numeric_limits<double>::quiet_NaN();
            require(!FtPipelineTestAccess::sample(*f.loop,arm,state,sent,&compensated),
                    "nonfinite acquired joints accepted");
        }
    }
    std::cout << "measured-pose wrench: both calibrated arms, gravity, force direction, TCP moment, invalid joints passed\n";
}

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

// THE WALL ON THE HOLD FOLD SINK (2026-09-07). Hand-guide a compliant Hold into an
// ROI face, keep pushing past it, then reverse the hand: the command must leave the
// face on the reversal at the hand's own rate, not after the whole out-of-box
// excursion has been unwound. MEASURED servo_log_20260907_021834.csv (right arm,
// y_min): the fold banked ~130 mm of hand travel into the latched nominal while the
// Tier-2 clamp held the command on the face; the operator then pushed back at
// 17-35 N for 2.2 s (~108 mm) with the command frozen, and needed 25 mm more before
// it moved. The sink is now clamped to the ROI/floor (foldForceDeviation).
bool testHoldFoldSinkIsWalledAtTheRoi() {
    Fixture f([](DualArmConfig& cfg) {
        cfg.safety.init_motion_planner.enable = false;
        cfg.safety.self_collision.enable = false;
        cfg.force_torque.auto_tare_after_init_motion.enable = false;
        auto& roi = cfg.safety.roi_box;
        roi.enable = true;
        roi.monitor_only = false;
        roi.min_m = {-3.0, -3.0, -3.0};
        roi.max_m = {3.0, 3.0, 3.0};
        roi.runtime_min_m = {-4.0, -4.0, -4.0};
        roi.runtime_max_m = {4.0, 4.0, 4.0};
        roi.tcp_offset_points.clear();
    }, JointArray{0.0, -45.0, 90.0, 0.0, 45.0, 0.0});   // mid-reach: the default fixture pose
                                                        // sits 1.09 m out, on the reach envelope,
                                                        // where a few mm of hand travel is IK-infeasible
    auto stream = f.command(ControlMode::Hold, ControlMode::TcpPoseTarget);
    f.warm(stream, 0.0);
    auto hold = f.command(ControlMode::Hold, ControlMode::Hold);
    for (int i = 0; i < 50; ++i) f.tick(hold);
    const auto pos_of = [](const Pose6D& p) { return std::array<double, 3>{p.x, p.y, p.z}; };
    const Pose6D start = f.rightSent();
    // 8 N through the Hold law's pure damper (b = 1000 N s/m) walks the nominal at 8 mm/s.
    Wrench6D push;
    push.fz = 8.0;
    f.right->setWrench(push);
    for (int i = 0; i < 250; ++i) {
        f.tick(hold);
        require(f.latest.right_cartesian_solve.status == "ok" || i < 5,
                "hold-wall fixture: IK failed during the free hand-guide push (" +
                    f.latest.right_cartesian_solve.status + "/" + f.latest.right_cartesian_solve.reason + ")");
    }
    require(f.latest.right_force_control.hold_engaged, "hold-wall fixture: hand-guide did not engage");
    const Pose6D moved = f.rightSent();
    const std::array<double, 3> d{moved.x - start.x, moved.y - start.y, moved.z - start.z};
    int axis = 0;
    for (int k = 1; k < 3; ++k) {
        if (std::fabs(d[k]) > std::fabs(d[axis])) axis = k;
    }
    require(std::fabs(d[axis]) > 0.001, "hold-wall fixture: hand-guide did not move the arm");
    const double sign = d[axis] > 0.0 ? 1.0 : -1.0;
    // Put the face 5 mm ahead on the axis the hand is moving along.
    const double face = pos_of(moved)[axis] + sign * 0.005;
    DualArmCommand set_roi = hold;
    set_roi.right.mode = ControlMode::SetSafetyRoiBounds;
    set_roi.has_roi_bounds = true;
    set_roi.roi_min_m = {-3.0, -3.0, -3.0};
    set_roi.roi_max_m = {3.0, 3.0, 3.0};
    (sign > 0.0 ? set_roi.roi_max_m : set_roi.roi_min_m)[axis] = face;
    f.tick(set_roi);
    // Keep pushing for 3 s: 24 mm of hand travel past the face. The ROI rows hold
    // the sent pose a hair inside the face (1.6 mm here, the 2 mm pose-track
    // standoff on the real stack); "reached" means inside that band.
    double deepest = -1.0;
    int ticks_to_face = -1;
    for (int i = 0; i < 1500; ++i) {
        f.tick(hold);
        const double over = sign * (pos_of(f.rightSent())[axis] - face);
        deepest = std::max(deepest, over);
        if (ticks_to_face < 0 && over > -0.0025) ticks_to_face = i + 1;
        if (std::getenv("RB_HOLD_WALL_TRACE") && i % 50 == 0) {
            const auto& cs = f.latest.right_cartesian_solve;
            std::cout << "  push tick " << i << " over_mm=" << over * 1e3 << " verdict=" << static_cast<int>(f.latest.safety_verdict)
                      << " ik=" << cs.status << "/" << cs.reason << " iters=" << cs.ik_iterations
                      << " jl_idx=" << cs.ik_joint_limit_worst_index << " jl_margin=" << cs.ik_joint_limit_worst_margin_deg
                      << " pinned=" << cs.ik_joint_limit_pinned << " branch=" << cs.ik_branch_jump_clamped
                      << " pos_err_mm=" << cs.position_error_m * 1e3 << " ori_err_deg=" << cs.orientation_error_rad * 180.0 / M_PI;
            if (cs.stage_tcp_target_stand) {
                const Pose6D& t = *cs.stage_tcp_target_stand;
                const Pose6D sent = f.rightSent();
                std::cout << " stage_target=(" << t.x << "," << t.y << "," << t.z << " rpy " << t.rx << "," << t.ry << "," << t.rz
                          << " q?" << t.quaternion_xyzw.has_value() << ") sent=(" << sent.x << "," << sent.y << "," << sent.z
                          << " rpy " << sent.rx << "," << sent.ry << "," << sent.rz << ")";
            }
            std::cout << '\n';
        }
    }
    require(ticks_to_face > 0, "hold-wall fixture: the command never reached the ROI face");
    require(deepest < 0.0005, "the hand-guided command crossed the ROI face");
    require(f.latest.right_force_control.fold_sink == "hold_nominal",
            "hold-wall fixture: the Hold fold sink was not the one exercised");
    const double at_face = pos_of(f.rightSent())[axis];
    // Reverse the hand.
    Wrench6D pull;
    pull.fz = -8.0;
    f.right->setWrench(pull);
    int ticks_to_leave = -1;
    for (int i = 0; i < 1500; ++i) {
        f.tick(hold);
        if (std::getenv("RB_HOLD_WALL_TRACE") && i % 25 == 0) {
            const auto& fc = f.latest.right_force_control;
            std::cout << "  pull tick " << i << " back_mm=" << sign * (at_face - pos_of(f.rightSent())[axis]) * 1e3
                      << " fold_mm=" << norm3(fc.fold_m) * 1e3 << " dev_mm=" << fc.deviation_norm_m * 1e3
                      << " vel=" << fc.velocity_m_s[axis] << " engaged=" << fc.hold_engaged
                      << " verdict=" << static_cast<int>(f.latest.safety_verdict) << '\n';
        }
        if (sign * (at_face - pos_of(f.rightSent())[axis]) > 0.003) {
            ticks_to_leave = i + 1;
            break;
        }
    }
    std::cout << "hold fold sink wall: axis " << axis << " sign " << sign << " reached the face in "
              << ticks_to_face << " ticks, deepest " << deepest * 1e3 << " mm, left it "
              << (ticks_to_leave > 0 ? std::to_string(ticks_to_leave) : std::string("never"))
              << " ticks after the reversal\n";
    // 3 mm at 8 mm/s is 188 ticks plus the damper's 12 ms ramp; the banked nominal
    // used to need the full 24 mm (1500 ticks) first.
    require(ticks_to_leave > 0 && ticks_to_leave <= 400,
            "reversing the hand at the ROI face did not move the command promptly: "
            "the fold banked the hand's travel past the face into the nominal");
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
        packet["chunk_metadata"]["preview_recovery_epoch"]=f.latest.preview_recovery.epoch;
        packet["chunk_metadata"]["observation_time_ns"]=nowSteadyNs();
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

    // A planning backlog retires the nominal chunk, not the force reference.
    // Keep a steady nonzero external wrench throughout recovery so lost strip,
    // double compose, a hold-law switch or a measured-pose restart is visible.
    for(int i=0;i<30;++i)pacedTick(preview);
    const auto recovery_bias=f.latest.right_ft.bias_generation;
    const auto recovery_resets=f.latest.right_force_control.reference_reset_count;
    const auto recovery_epoch=f.latest.preview_recovery.epoch;
    const auto recovery_completed=f.latest.preview_recovery.completed;
    const auto recovery_source=f.latest.right_cartesian_solve.preview_execution.source_wire_seq;
    const auto nominal_before=PreviewRecoveryTestAccess::rightAccepted(*f.loop);
    const double recovery_origin_sec=static_cast<double>(nowSteadyNs())*1e-9;
    const auto& recovery_profile=*std::find_if(f.cfg.cartesian_control.tcp_pose_target_profiles.begin(),
        f.cfg.cartesian_control.tcp_pose_target_profiles.end(),
        [](const auto& p){return p.name=="flow_infer_preview";});
    control::PreviewBrake expected_brake(recovery_profile.ruckig_follower.preview_execution.tracker,
                                        1./f.cfg.servo.rate_hz);
    require(expected_brake.start(nominal_before)==control::PreviewBrakeStatus::Ready,
            "accepted covered state has no finite reference brake");
    const double standing_deviation=f.latest.right_force_control.deviation_norm_m;
    require(standing_deviation>.0005,"covered recovery lost its standing deviation before the event");
    double maximum_nominal_drift=0.,maximum_composed_error=0.,maximum_raw_rotation=0.;
    double maximum_stage_raw_rotation=0.;
    double ik_position=f.cfg.kinematics.ik.position_tolerance_m;
    double ik_rotation=f.cfg.kinematics.ik.orientation_tolerance_rad;
    const auto include_best_effort=[&](double position,double rotation) {
        if(position>0.&&rotation>0.) {
            ik_position=std::max(ik_position,position);ik_rotation=std::max(ik_rotation,rotation);
        }
    };
    include_best_effort(f.cfg.kinematics.ik.joint_limit_best_effort_position_tolerance_m,
                        f.cfg.kinematics.ik.joint_limit_best_effort_orientation_tolerance_rad);
    include_best_effort(f.cfg.kinematics.ik.max_iterations_best_effort_position_tolerance_m,
                        f.cfg.kinematics.ik.max_iterations_best_effort_orientation_tolerance_rad);
    const auto check_force_recovery=[&] {
        const auto& force_state=f.latest.right_force_control;
        const auto& solve=f.latest.right_cartesian_solve;
        require(f.latest.right_ft.bias_valid&&f.latest.right_ft.bias_generation==recovery_bias&&
                force_state.reference_reset_count==recovery_resets,
                "planning recovery reset the covered force reference or tare generation");
        require(force_state.reference_strip_enabled&&force_state.law=="stream",
                "planning recovery lost force strip eligibility or switched to the hand-guide law");
        require(norm3(force_state.reference_deviation_m)>.0005,
                "planning recovery discarded the standing force deviation");
        require(!force_state.bounded&&!force_state.folded,
                "planning recovery hit a force fence or folded the spring-law reference");
        require(!f.latest.left_cartesian_solve.preview_execution.active,
                "single-arm recovery gave preview authority to the held peer");
        if(!solve.stage_tcp_target_stand) {
            require(f.latest.preview_recovery.state==PreviewRecoveryState::Starting&&
                    !solve.preview_execution.active&&!force_state.covered,
                    "force-covered recovery lost its nominal target outside first-plan waiting");
            return;
        }
        require(force_state.covered&&force_state.compose_applied,
                "finite recovery target bypassed its standing force overlay");
        const auto nominal=PreviewRecoveryTestAccess::rightAccepted(*f.loop);
        control::PreviewMotionSample expected_nominal;
        require(expected_brake.sample(static_cast<double>(nowSteadyNs())*1e-9-recovery_origin_sec,
                                      expected_nominal),"reference covered brake sample unavailable");
        maximum_nominal_drift=std::max(maximum_nominal_drift,
            math::positionDistance(nominal.pose,nominal_before.pose));
        // A zero-delta source can still have small accepted angular p/v/a from
        // its cold-start solve. Preserve its finite stopping trajectory first;
        // only the dispatched terminal is the stationary fresh-plan seed.
        const bool matches_brake=math::positionDistance(nominal.pose,expected_nominal.pose)<1e-10&&
                math::orientationDistanceRad(nominal.pose,expected_nominal.pose)<1e-10&&
                (nominal.linear_velocity-expected_nominal.linear_velocity).norm()<1e-10&&
                (nominal.linear_acceleration-expected_nominal.linear_acceleration).norm()<1e-8&&
                (nominal.angular_velocity_body-expected_nominal.angular_velocity_body).norm()<1e-10&&
                (nominal.angular_acceleration_body-expected_nominal.angular_acceleration_body).norm()<1e-8;
        const auto& limits=recovery_profile.ruckig_follower.preview_execution.tracker;
        // After release the next QPs own derivatives. The canonical reference
        // must still start at the exact dispatched terminal; output tracking is
        // judged by the declared tracking envelope, not feasibility precision.
        const bool tracking=f.latest.preview_recovery.state==PreviewRecoveryState::Tracking;
        const auto raw_reference=PreviewRecoveryTestAccess::rightRaw(*f.loop);
        const double raw_rotation=math::orientationDistanceRad(raw_reference.pose,expected_nominal.pose);
        const double stage_raw_rotation=math::orientationDistanceRad(nominal.pose,raw_reference.pose);
        if(tracking) {
            maximum_raw_rotation=std::max(maximum_raw_rotation,raw_rotation);
            maximum_stage_raw_rotation=std::max(maximum_stage_raw_rotation,stage_raw_rotation);
            require(math::positionDistance(raw_reference.pose,expected_nominal.pose)<1e-10&&raw_rotation<1e-10,
                    "fresh zero chunk reanchored to the composed/readback pose instead of the nominal terminal");
        }
        const bool correct_nominal=tracking ?
            math::positionDistance(nominal.pose,raw_reference.pose)<=limits.linear_tracking_tolerance_m&&
            stage_raw_rotation<=limits.angular_tracking_tolerance_rad&&
            nominal.linear_velocity.cwiseAbs().maxCoeff()<=limits.max_linear_velocity_m_s&&
            nominal.linear_acceleration.cwiseAbs().maxCoeff()<=limits.max_linear_acceleration_m_s2&&
            nominal.angular_velocity_body.norm()<=limits.max_angular_velocity_rad_s&&
            nominal.angular_acceleration_body.norm()<=limits.max_angular_acceleration_rad_s2 : matches_brake;
        if(!correct_nominal)std::cerr<<"covered recovery nominal diagnostic time="<<nowSteadyNs()
            <<" state="<<toString(f.latest.preview_recovery.state)
            <<" preview="<<solve.preview_execution.status
            <<" dp_m="<<math::positionDistance(nominal.pose,nominal_before.pose)
            <<" dr_rad="<<math::orientationDistanceRad(nominal.pose,nominal_before.pose)
            <<" before_v="<<nominal_before.linear_velocity.transpose()
            <<" before_a="<<nominal_before.linear_acceleration.transpose()
            <<" before_w="<<nominal_before.angular_velocity_body.transpose()
            <<" before_alpha="<<nominal_before.angular_acceleration_body.transpose()
            <<" now_v="<<nominal.linear_velocity.transpose()
            <<" now_a="<<nominal.linear_acceleration.transpose()
            <<" now_w="<<nominal.angular_velocity_body.transpose()
            <<" now_alpha="<<nominal.angular_acceleration_body.transpose()
            <<" expected_dr="<<math::orientationDistanceRad(nominal.pose,expected_nominal.pose)
            <<" raw_dr="<<math::orientationDistanceRad(PreviewRecoveryTestAccess::rightRaw(*f.loop).pose,
                                                       expected_nominal.pose)
            <<" stage_raw_dr="<<math::orientationDistanceRad(nominal.pose,
                                                            PreviewRecoveryTestAccess::rightRaw(*f.loop).pose)
            <<" folds="<<solve.preview_execution.fold_count
            <<" force_deviation="<<force_state.deviation_norm_m<<'\n';
        require(correct_nominal,
                "covered recovery departed from its accepted-state brake or dispatched terminal seed");
        Pose6D composed=nominal.pose;
        composed.x+=force_state.deviation_m[0];composed.y+=force_state.deviation_m[1];
        composed.z+=force_state.deviation_m[2];
        // Rotation is rigid in this fixture. Translation is the live overlay,
        // including any residual settling; its offset is neither reset nor frozen by this assertion.
        const double error=math::positionDistance(f.rightSent(),composed);
        maximum_composed_error=std::max(maximum_composed_error,error);
        require(error<=ik_position&&math::orientationDistanceRad(f.rightSent(),composed)<=ik_rotation,
                "accepted recovery command did not compose its current force deviation exactly once");
    };
    PreviewRecoveryTestAccess::request(*f.loop,PreviewRecoveryCause::Backlog);
    pacedTick(preview);
    require(f.latest.preview_recovery.state==PreviewRecoveryState::Braking&&
            f.latest.preview_recovery.epoch>recovery_epoch,
            "covered backlog event did not enter a new finite recovery epoch");
    check_force_recovery();
    for(int i=0;i<200&&f.latest.preview_recovery.state==PreviewRecoveryState::Braking;++i) {
        pacedTick(preview);check_force_recovery();
    }
    require(f.latest.preview_recovery.state==PreviewRecoveryState::WaitingFresh,
            "covered nominal stop never reached the fresh-observation barrier");
    const auto observation_fence=f.latest.preview_recovery.min_observation_time_ns;
    require(observation_fence>0,"covered recovery did not publish an observation fence");
    for(int i=0;i<6;++i) {pacedTick(preview);check_force_recovery();}
    require(nowSteadyNs()>observation_fence,"fresh covered observation is not after the stop barrier");
    publishZero();pacedTick(preview);check_force_recovery();
    require(f.latest.preview_recovery.state==PreviewRecoveryState::Starting&&
            f.latest.preview_recovery.candidate_source_wire_seq>recovery_source,
            "covered recovery failed to select the new post-stop chunk");
    for(int i=0;i<80&&f.latest.preview_recovery.state==PreviewRecoveryState::Starting;++i) {
        pacedTick(preview);check_force_recovery();
    }
    require(f.latest.preview_recovery.state==PreviewRecoveryState::Tracking&&
            f.latest.preview_recovery.completed==recovery_completed+1&&
            f.latest.right_cartesian_solve.preview_execution.active,
            "covered recovery never resumed the fresh nominal chunk");
    for(int i=0;i<20;++i) {pacedTick(preview);check_force_recovery();}
    std::cout<<"preview force backlog recovery: nominal drift um="<<maximum_nominal_drift*1e6
             <<" composed FK error um="<<maximum_composed_error*1e6
             <<" raw-terminal rotation rad="<<maximum_raw_rotation
             <<" stage-raw rotation rad="<<maximum_stage_raw_rotation
             <<" retained deviation mm="<<f.latest.right_force_control.deviation_norm_m*1e3<<'\n';

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
        if (argc == 2 && std::string(argv[1]) == "--measured-wrench-only") {
            testWrenchUsesAcquiredJointPose();
            return 0;
        }
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
        if (argc == 2 && std::string(argv[1]) == "--hold-roi-wall-only") {
            testHoldFoldSinkIsWalledAtTheRoi();
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
        testHoldFoldSinkIsWalledAtTheRoi();
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
