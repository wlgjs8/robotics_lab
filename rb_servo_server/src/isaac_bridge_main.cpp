// A pipe-only plant adapter. It never constructs BackendFactory, rbpodo,
// CommandServer sockets, a gripper bridge, or the box-calibration probe.
#include <iostream>
#include <memory>
#include <stdexcept>
#include <cmath>
#include <nlohmann/json.hpp>
#include "rb_servo/config/config.hpp"
#include "rb_servo/control/dual_arm_servo_loop.hpp"
#include "rb_servo/core/clock.hpp"
#include "rb_servo/network/command_server.hpp"
#include "rb_servo/network/state_publisher.hpp"
#include "rb_servo/math/se3.hpp"

using namespace rb_servo;
using json = nlohmann::json;
namespace {
struct Plant {
    RobotState state;
    JointArray target{};
    double grip = 100.0, grip_target = 100.0;
};

JointArray joints(const json& j) {
    if (!j.is_array() || j.size() != kDof) throw std::runtime_error("expected six joints");
    JointArray q{};
    for (int i = 0; i < kDof; ++i) {
        q[i] = j.at(i).get<double>();
        if (!std::isfinite(q[i])) throw std::runtime_error("nonfinite joint");
    }
    return q;
}

void measure(Plant& p, const json& j, uint64_t seq, const FtArmConfig* ft = nullptr) {
    p.state.q_actual_deg = joints(j.at("q_deg"));
    p.state.dq_actual_deg_s = joints(j.at("dq_deg_s"));
    p.grip = j.at("gripper_percent").get<double>();
    if (!std::isfinite(p.grip) || p.grip < 0 || p.grip > 100)
        throw std::runtime_error("invalid measured gripper percent");
    p.state.host_time_ns = p.state.robot_time_ns = nowSteadyNs();
    p.state.acquisition_sequence = seq;
    p.state.q_target_deg = p.target;
    p.state.q_actual_valid = p.state.q_ref_valid = p.state.has_valid_joint_state = true;
    p.state.q_ref_source = "isaac_drive_target";
    p.state.eft_valid = false;
    if (ft) {
        const auto& sample = j.at("force_sensor");
        if (sample.at("seq").get<uint64_t>() != seq ||
            sample.at("time_ns").get<uint64_t>() != nowSteadyNs() ||
            sample.at("frame") != "flange_at_flange" || !sample.at("valid").get<bool>())
            throw std::runtime_error("invalid/stale PhysX force sample or frame");
        const auto w = joints(sample.at("wrench"));
        const math::Vector3 f(w[0], w[1], w[2]), m(w[3], w[4], w[5]);
        const math::Vector3 offset(ft->sensor_offset_mm[0]*1e-3,
                                  ft->sensor_offset_mm[1]*1e-3, ft->sensor_offset_mm[2]*1e-3);
        math::Matrix3 axes;
        axes.col(0) = math::Vector3(ft->axis_fx[0], ft->axis_fx[1], ft->axis_fx[2]);
        axes.col(1) = math::Vector3(ft->axis_fy[0], ft->axis_fy[1], ft->axis_fy[2]);
        axes.col(2) = math::Vector3(ft->axis_fz[0], ft->axis_fz[1], ft->axis_fz[2]);
        // Match the existing electrical channel map, including its det=-1.
        // Move the measurement reference from flange to SRO before encoding it.
        const math::Vector3 raw_f = axes.transpose()*f;
        const math::Vector3 raw_m = axes.transpose()*(m-offset.cross(f));
        p.state.eft_wrench = {raw_f.x(),raw_f.y(),raw_f.z(),raw_m.x(),raw_m.y(),raw_m.z()};
        p.state.eft_valid = true;
    }
}

class IsaacPlantBackend final : public IRobotBackend {
    Plant& p_;
    bool connected_ = false, initialized_ = false;
    BackendResult<RobotState> result(BackendOp op) {
        p_.state.connection_state = connected_ ? RobotConnectionState::Connected : RobotConnectionState::Disconnected;
        p_.state.servo_enabled = initialized_;
        BackendResult<RobotState> r;
        r.ok = connected_; r.op = op; r.value = p_.state;
        r.timing = makeBackendTiming(nowSteadyNs(), nowSteadyNs());
        if (!r.ok) r.error = backendError(BackendErrorKind::RobotDisconnected, "Isaac plant disconnected");
        return r;
    }
public:
    IsaacPlantBackend(Plant& p, ArmId id) : p_(p) { p_.state.arm_id = id; }
    BackendResult<RobotState> connect() override { connected_ = true; return result(BackendOp::Connect); }
    BackendResult<RobotState> initialize() override { initialized_ = true; return result(BackendOp::Initialize); }
    BackendResult<RobotState> readState() override { return result(BackendOp::ReadState); }
    BackendResult<RobotState> stop() override {
        // Stop motion at the measured pose, retaining the initialized plant so
        // the shared ResetFault/ArmMotion lifecycle can recover explicitly.
        p_.target = p_.state.q_actual_deg;
        p_.state.q_target_deg = p_.target;
        p_.grip_target = p_.grip;
        return result(BackendOp::Stop);
    }
    BackendResult<RobotState> resetFault() override { return result(BackendOp::ResetFault); }
    SendServoJResult sendServoJ(const SendServoJRequest& request) override {
        const auto timing = makeBackendTiming(nowSteadyNs(), nowSteadyNs());
        if (!initialized_) return rejectedSend(request, backendError(BackendErrorKind::ServoDisabled, "plant not initialized"), timing);
        for (double q : request.q_target_deg) if (!std::isfinite(q))
            return rejectedSend(request, backendError(BackendErrorKind::InvalidTarget, "nonfinite target"), timing);
        if (request.deadline_ns && nowSteadyNs() > request.deadline_ns)
            return rejectedSend(request, backendError(BackendErrorKind::CommandTimeout, "expired plant target"), timing);
        p_.target = request.q_target_deg;
        SendServoJResult r;
        r.accepted = true; r.timing = timing; r.requested_q_deg = p_.target;
        r.acceptance_semantics = "isaac_drive_target_staged";
        return r;
    }
    bool isConnected() const override { return connected_; }
    ArmId armId() const override { return p_.state.arm_id; }
    std::string name() const override { return "isaac_physx_plant"; }
    bool supportsExternalStepping() const override { return true; }
};

bool readLine(std::string& s) {
    s.clear();
    char c;
    while (std::cin.get(c)) {
        if (c == '\n') return true;
        if (s.size() >= 1024 * 1024) throw std::runtime_error("RPC exceeds 1 MiB");
        s += c;
    }
    return !s.empty();
}
} // namespace

int main(int argc, char** argv) {
    // Library diagnostics may use cout. Reserve the original stream for JSONL.
    std::ostream rpc(std::cout.rdbuf());
    std::cout.rdbuf(std::cerr.rdbuf());
    try {
        if (argc != 2) throw std::runtime_error("usage: rb_isaac_bridge /absolute/stack_real.yaml");
        auto cfg = loadConfigFromYaml(argv[1]);
        if (cfg.servo.rate_hz != 500 || !cfg.kinematics.enable)
            throw std::runtime_error("Isaac requires 500 Hz and configured Pinocchio kinematics");
        // Explicit plant-only projection; the source YAML is never rewritten.
        for (auto* b : {&cfg.left_robot, &cfg.right_robot}) {
            b->backend_type = BackendType::Mock;
            b->run_mode = RunMode::Mock;
            b->operation_mode = "simulation";
            b->ip.clear();
        }
        cfg.servo.io_model = ServoIoModel::Direct;
        cfg.servo.enable_realtime_priority = false;
        cfg.servo.cpu_core = -1;
        cfg.servo.spin_slack_us = 0;
        cfg.force_torque.auto_tare_after_init_motion.enable = false;
        cfg.gripper.enable = false;
        cfg.kinematics.calibration.source = "nominal";
        const uint64_t dt_ns = 2'000'000;
        uint64_t time_ns = 1'000'000'000, seq = 1;
        setExternalSteadyNs(time_ns);
        std::string line;
        if (!readLine(line)) return 0;
        const auto init = json::parse(line);
        if (init.at("op") != "init") throw std::runtime_error("first RPC must be init");
        const bool force_sensor = init.value("force_sensor_enabled", false);
        if (force_sensor && (!cfg.force_torque.enable || !cfg.force_control.enable ||
                             !cfg.force_torque.left.enable || !cfg.force_torque.right.enable))
            throw std::runtime_error("PhysX F/T requires the source stack's complete force configuration");
        if (!force_sensor) {
            cfg.force_torque.enable = false;
            cfg.force_control.enable = false;
        }
        Plant left, right;
        for (auto pair : {std::pair<Plant*, const char*>{&left, "left"}, {&right, "right"}}) {
            measure(*pair.first, init.at(pair.second), seq, force_sensor ?
                    (pair.first == &left ? &cfg.force_torque.left : &cfg.force_torque.right) : nullptr);
            pair.first->target = pair.first->state.q_actual_deg;
            pair.first->state.q_target_deg = pair.first->target;
            pair.first->grip_target = pair.first->grip;
        }
        CommandBuffer buffer;
        CommandServer command(cfg.network, &buffer, cfg.cartesian_control);
        ChunkFrameReceiver chunks("");
        DualArmServoLoop loop(std::make_unique<IsaacPlantBackend>(left, ArmId::Left),
                              std::make_unique<IsaacPlantBackend>(right, ArmId::Right),
                              cfg, &buffer, nullptr);
        loop.enableExternalStepping();
        loop.setChunkFrameReceiver(&chunks);
        StatePublisher publisher(cfg);
        auto step = [&] {
            loop.setGripperFeedback(ArmId::Left, left.grip, true);
            loop.setGripperFeedback(ArmId::Right, right.grip, true);
            if (!loop.stepOnce()) throw std::runtime_error("C++ tick failed or timed out");
        };
        auto response = [&] {
            const auto snap = loop.latestSnapshot();
            auto state = json::parse(publisher.serializeSnapshot(snap));
            for (auto pair : {std::pair<Plant*, const char*>{&left, "left"}, {&right, "right"}}) {
                auto& p = *pair.first;
                const auto& ac = pair.first == &left ? snap.command.left : snap.command.right;
                const auto& solve = pair.first == &left ? snap.left_cartesian_solve : snap.right_cartesian_solve;
                if (!snap.fault_latched && ac.has_gripper && !snap.send_suppressed)
                    p.grip_target = ac.gripper_target;
                state[pair.second]["gripper"] = {{"valid",true},{"ok",true},{"stale",false},
                    {"percent",p.grip},{"target_percent",p.grip_target},
                    {"moving",std::abs(p.grip - p.grip_target) > 0.1},
                    {"fault",nullptr},{"feedback_age_ms",0.0},{"sample_age_ms",0.0}};
                state[pair.second]["shared_control"] = {
                    {"controller",solve.follower_controller},{"active",solve.follower_active},
                    {"chunk_seq",solve.follower_wire_seq},{"step",solve.follower_step},
                    {"segment_time_sec",solve.follower_t_in_seg_sec},
                    {"stall",solve.follower_stall},{"actual_lead_m",solve.follower_actual_lead_m},
                    {"reanchors",solve.follower_reanchor_count},
                    {"output_smd_active",solve.follower_output_smd_active}};
            }
            state["plant"] = "isaac_physx";
            state["simulated_force_sensor"] = force_sensor;
            return json{{"ok",true},{"time_ns",time_ns},{"seq",seq},{"state",state},
                {"left",{{"q_target_deg",left.target},{"gripper_percent",left.grip_target}}},
                {"right",{{"q_target_deg",right.target},{"gripper_percent",right.grip_target}}},
                {"overrides",{"direct_io","external_2ms_clock","synchronous_collision",
                    force_sensor ? "physx_joint_force_sensor" : "force_control_off", "nominal_urdf", "no_hardware_transports"}}};
        };
        if (!loop.start()) throw std::runtime_error("shared servo startup failed");
        step();
        rpc << response().dump() << std::endl;
        while (readLine(line)) {
            const auto req = json::parse(line);
            const auto op = req.at("op").get<std::string>();
            if (op == "close") break;
            if (op == "step") {
                if (req.at("seq").get<uint64_t>() != seq + 1 ||
                    req.at("time_ns").get<uint64_t>() != time_ns + dt_ns)
                    throw std::runtime_error("plant sequence/time must advance exactly one 2 ms tick");
                ++seq; time_ns += dt_ns; setExternalSteadyNs(time_ns);
                measure(left, req.at("left"), seq, force_sensor ? &cfg.force_torque.left : nullptr);
                measure(right, req.at("right"), seq, force_sensor ? &cfg.force_torque.right : nullptr);
                step();
                rpc << response().dump() << std::endl;
            } else if (op == "command") {
                DualArmCommand cmd;
                const bool ok = command.parseMessage(req.at("packet").dump(), nowSteadyNs(), &cmd);
                if (ok) {
                    if (cmd.lease_admin_only) buffer.updateLease(cmd.lease, nowSteadyNs());
                    else buffer.setCommand(cmd);
                    if (cmd.has_external_boxes) buffer.noteExternalBoxReceived(nowSteadyNs());
                }
                rpc << json{{"ok",ok},{"reason",command.lastRejectReason()}}.dump() << std::endl;
            } else if (op == "chunk") {
                const auto packet = req.at("packet").dump();
                rpc << json{{"ok",chunks.acceptPacket(packet.data(),packet.size())}}.dump() << std::endl;
            } else throw std::runtime_error("unknown RPC operation");
        }
        loop.stop();
        return 0;
    } catch (const std::exception& e) {
        rpc << json{{"ok",false},{"error",e.what()}}.dump() << std::endl;
        return 1;
    }
}
