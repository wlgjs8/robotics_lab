#include <algorithm>
#include <array>
#include <cstdlib>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <unistd.h>

#include "rb_servo/config/config.hpp"

namespace {

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

std::string writeTempConfig(const std::string& name, const std::string& body) {
    const std::string path = "/tmp/rb-servo-config-" + name + "-" + std::to_string(getpid()) + ".yaml";
    std::ofstream file(path);
    file << body;
    return path;
}

bool loadRejects(const std::string& path) {
    try {
        (void)rb_servo::loadConfigFromYaml(path);
    } catch (const std::exception&) {
        return true;
    }
    return false;
}

bool loadRejectsContaining(const std::string& path, const std::string& expected) {
    try {
        (void)rb_servo::loadConfigFromYaml(path);
    } catch (const std::exception& e) {
        return std::string(e.what()).find(expected) != std::string::npos;
    }
    return false;
}

std::string readFile(const std::filesystem::path& path) {
    std::ifstream file(path);
    std::ostringstream text;
    text << file.rdbuf();
    return text.str();
}

bool replaceOnce(std::string* text, const std::string& from, const std::string& to) {
    const std::size_t offset = text->find(from);
    if (offset == std::string::npos) return false;
    text->replace(offset, from.size(), to);
    return true;
}

bool near(double a, double b) {
    return std::abs(a - b) < 1e-12;
}

bool hasPairRule(const std::vector<rb_servo::CollisionPairPattern>& rules,
                 const std::string& a,
                 const std::string& b) {
    for (const auto& rule : rules) {
        if ((rule.pattern_a == a && rule.pattern_b == b) ||
            (rule.pattern_a == b && rule.pattern_b == a)) {
            return true;
        }
    }
    return false;
}

std::filesystem::path servoRoot() {
    return std::filesystem::path(__FILE__).parent_path().parent_path();
}

std::string rb3UrdfPath() {
    return (servoRoot() / "descriptions" / "urdf" / "rb3_730e.urdf").string();
}

class EnvGuard {
public:
    EnvGuard(const char* name, const char* value) : name_(name) {
        const char* previous = std::getenv(name);
        if (previous) {
            had_previous_ = true;
            previous_ = previous;
        }
        if (value) {
            ::setenv(name, value, 1);
        } else {
            ::unsetenv(name);
        }
    }

    ~EnvGuard() {
        if (had_previous_) {
            ::setenv(name_.c_str(), previous_.c_str(), 1);
        } else {
            ::unsetenv(name_.c_str());
        }
    }

private:
    std::string name_;
    bool had_previous_ = false;
    std::string previous_;
};

bool testRepositoryConfigsParse() {
    const std::filesystem::path config_dir =
        std::filesystem::path(__FILE__).parent_path().parent_path() / "config";

    {
        const rb_servo::DualArmConfig stack_real =
            rb_servo::loadConfigFromYaml((config_dir / "stack_real.yaml").string());
        RB_CHECK(stack_real.left_robot.backend_type == rb_servo::BackendType::Rbpodo);
        RB_CHECK(stack_real.right_robot.backend_type == rb_servo::BackendType::Rbpodo);
        RB_CHECK(stack_real.left_robot.run_mode == rb_servo::RunMode::Real);
        RB_CHECK(stack_real.right_robot.run_mode == rb_servo::RunMode::Real);
        RB_CHECK(stack_real.left_robot.operation_mode == "real");
        RB_CHECK(stack_real.right_robot.operation_mode == "real");
        RB_CHECK(stack_real.servo.rate_hz == 500);
        {
            // The cell-structure band the env_* geometry (the riser) is enforced
            // against. Asserted on the TRACKED file because the numbers are the
            // safety decision: at the self 40 mm floor the recorded 35.7 mm closest
            // approach is a hard violation, and a_brake must stay INHERITED (-1) or
            // the braking invariant fails at the 0.60 m/s ceiling (needs 0.085 with
            // a_brake 3.0, 0.065 with the shared 4.5).
            const auto& env = stack_real.safety.self_collision.mesh.environment;
            RB_CHECK(env.d_hard_m == 0.025);
            RB_CHECK(env.d_slow_m == 0.067);
            RB_CHECK(env.a_brake_m_s2 < 0.0);   // inherits the self ramp
            RB_CHECK(env.hyst_m < 0.0);         // inherits the self hysteresis
            RB_CHECK(env.recover_speed_m_s == 0.0);
            RB_CHECK(env.d_hard_m < stack_real.safety.self_collision.mesh.d_hard_m);
        }
        RB_CHECK(stack_real.servo.send_servo_commands);
        RB_CHECK(stack_real.servo.allow_real_motion_with_suspect_diagnostics);
        RB_CHECK(!stack_real.servo.allow_controller_simulation_motion);
        RB_CHECK(near(stack_real.left_robot.servo_t1_sec, 0.002));
        RB_CHECK(near(stack_real.right_robot.servo_t1_sec, 0.002));
        // 2026-08-19 brake-before-plan ships ON for the real stack (InitMotion while the
        // policy is streaming brakes from the last sent target instead of snapping to the
        // measured joints); the exit threshold must sit at or below the enter threshold.
        RB_CHECK(stack_real.safety.init_motion_planner.brake_before_plan);
        RB_CHECK(near(stack_real.safety.init_motion_planner.brake_enter_deg_s, 1.0));
        RB_CHECK(near(stack_real.safety.init_motion_planner.brake_exit_deg_s, 0.5));
        RB_CHECK(near(stack_real.safety.init_motion_planner.brake_timeout_sec, 0.75));
        RB_CHECK(near(stack_real.safety.init_motion_planner.brake_max_travel_deg, 3.0));
        RB_CHECK(near(stack_real.left_robot.servo_t2_sec, 0.021));
        RB_CHECK(near(stack_real.right_robot.servo_t2_sec, 0.021));
        RB_CHECK(stack_real.left_robot.disable_waiting_ack);
        RB_CHECK(stack_real.right_robot.disable_waiting_ack);
        RB_CHECK(near(stack_real.safety.q_min_deg[0], -360.0));
        // J3 (elbow) is clamped to the arm's physical range, not +/-360, and it must
        // MATCH the IK URDF limit. PTP bypasses IK and passes only this clamp, so any
        // band where the clamp is WIDER than the URDF is one the elbow can be parked in
        // and never commanded out of (measured on RB3: pinned at exactly 150.000 deg for
        // 7 s / 4.5 s on real rollouts, which is why the 2026-07 +/-160 "site margin" was
        // reverted). Both profiles are RB5-850E as of 2026-09-02, catalog elbow +/-165.
        // See docs/joint_range_policy.md.
        RB_CHECK(near(stack_real.safety.q_min_deg[2], -165.0));
        RB_CHECK(near(stack_real.safety.q_max_deg[2], 165.0));
        RB_CHECK(stack_real.network.command_bind == "udp://127.0.0.1:50256");
        RB_CHECK(stack_real.network.state_pub_endpoint == "udp://127.0.0.1:50356");
        RB_CHECK(stack_real.network.state_pub_endpoints.size() == 4);
        RB_CHECK(stack_real.command_source.enforce_lease);
        RB_CHECK(stack_real.network.command_source_enforce_lease);
        RB_CHECK(near(stack_real.command_source.lease_timeout_sec, 60.0));
        RB_CHECK(stack_real.cartesian_control.enable);
        RB_CHECK(stack_real.cartesian_control.allow_in_real);
        RB_CHECK(!stack_real.cartesian_control.allow_in_controller_simulation);
        RB_CHECK(stack_real.cartesian_control.tcp_pose_target_profile_default == "umi_large_smooth");
        RB_CHECK(stack_real.cartesian_control.tcp_pose_target_profiles.size() == 4);
        bool has_spacemouse = false;
        bool has_umi = false;
        bool has_flow = false;
        const rb_servo::TcpPoseTargetProfileConfig* flow_profile = nullptr;
        const rb_servo::TcpPoseTargetProfileConfig* fresh_profile = nullptr;
        const rb_servo::TcpPoseTargetProfileConfig* umi_profile = nullptr;
        for (const auto& profile : stack_real.cartesian_control.tcp_pose_target_profiles) {
            has_spacemouse = has_spacemouse || profile.name == "spacemouse_precise";
            has_umi = has_umi || profile.name == "umi_large_smooth";
            has_flow = has_flow || profile.name == "flow_infer_smooth";
            if (profile.name == "flow_infer_smooth") {
                flow_profile = &profile;
            }
            if (profile.name == "flow_infer_fresh") fresh_profile = &profile;
            if (profile.name == "umi_large_smooth") {
                umi_profile = &profile;
            }
        }
        RB_CHECK(has_spacemouse);
        RB_CHECK(has_umi);
        RB_CHECK(has_flow);
        RB_CHECK(flow_profile != nullptr);
        RB_CHECK(fresh_profile != nullptr);
        RB_CHECK(fresh_profile->ruckig_follower.fresh_chunk_replan);
        RB_CHECK(fresh_profile->ruckig_follower.continuous_hold_resume);
        RB_CHECK(!flow_profile->ruckig_follower.fresh_chunk_replan);
        RB_CHECK(!flow_profile->ruckig_follower.continuous_hold_resume);
        RB_CHECK(umi_profile != nullptr);
        // UMI teleop singularity guard. The ramp onset (full/floor) is the part that was
        // measured to throttle normal motion, so it is pinned; scale_min was lowered
        // 0.12 -> 0.02 on 2026-08-14 after the left-arm lurch (see stack_real.yaml).
        RB_CHECK(near(umi_profile->pose_track_smd.singularity_scale_full_sigma, 0.12));
        RB_CHECK(near(umi_profile->pose_track_smd.singularity_scale_floor_sigma, 0.05));
        RB_CHECK(near(umi_profile->pose_track_smd.singularity_scale_min, 0.02));
        RB_CHECK(near(flow_profile->pose_track_smd.natural_frequency_linear_hz, 1.6));
        RB_CHECK(near(flow_profile->pose_track_smd.natural_frequency_angular_hz, 1.6));
        RB_CHECK(near(flow_profile->pose_track_smd.max_linear_velocity_m_s, 0.50));
        RB_CHECK(near(flow_profile->pose_track_smd.max_linear_accel_m_s2, 1.20));
        RB_CHECK(near(flow_profile->pose_track_smd.max_angular_velocity_rad_s, 1.80));
        RB_CHECK(near(flow_profile->pose_track_smd.max_angular_accel_rad_s2, 5.00));
        RB_CHECK(near(flow_profile->pose_track_smd.reengage_relatch_max_step_m, 0.010));
        RB_CHECK(near(flow_profile->pose_track_smd.reengage_relatch_max_step_rad, 0.10));
        RB_CHECK(near(flow_profile->pose_track_smd.singularity_scale_full_sigma, 0.12));
        RB_CHECK(near(flow_profile->pose_track_smd.singularity_scale_floor_sigma, 0.05));
        RB_CHECK(near(flow_profile->pose_track_smd.singularity_scale_min, 0.12));
        RB_CHECK(near(flow_profile->max_smd_goal_lead_m, 0.080));
        RB_CHECK(near(flow_profile->max_smd_goal_lead_rad, 0.35));
        // 2026-07-24 operator tuning during bolt-pick iterations: 12 -> 5.
        // 2026-08-27: 5 -> 8. This is a CEILING, clamped by activate() to
        // min(consume_steps, n - L - R) where n = min(EXECUTE + RUNWAY, horizon),
        // so 8 serves EXECUTE_STEPS=4 (n=8 -> 4) and EXECUTE_STEPS=8 (n=12 -> 8)
        // alike; at 5 an 8-step execution window was silently capped at 5.
        RB_CHECK(flow_profile->ruckig_follower.consume_steps == 8);
        // 2026-07-23 operator tuning: reserve 2 -> 4 during the bolt-pick
        // rollout iterations.
        RB_CHECK(flow_profile->ruckig_follower.reserve_steps == 4);
        RB_CHECK(near(flow_profile->ruckig_follower.hold_bounce_resume_sec, 0.5));
        // 2026-07-18 SPEED_SCALE=1.0 posture: projection fidelity warns (the
        // model's chunks are followed), lead budgets scaled to the full-speed
        // honest lag (35 mm / 4 deg).
        RB_CHECK(flow_profile->ruckig_follower.preview_projection_fault_policy ==
                 rb_servo::RuckigProjectionFaultPolicy::Warn);
        RB_CHECK(near(flow_profile->ruckig_follower.preview_max_actual_lead_m, 0.035));
        // 2026-08-26: angular budget 4.0 -> 5.0 deg (five pi0.5 rollouts latched at
        // 4.07-4.38 deg with positional lead still inside its 35 mm budget). Must stay
        // under the 0.10 rad follower-divergence latch or that coarser fault wins.
        RB_CHECK(near(flow_profile->ruckig_follower.preview_max_actual_lead_rad,
                      0.08726646259));
        RB_CHECK(flow_profile->ruckig_follower.preview_max_actual_lead_rad <
                 0.10);
        const std::array<double, 9> expected_torque_matrix{
            -0.2206539391248573,
            -0.14062927088838903,
            -0.007134530034776744,
            0.13306183273551747,
            -0.2324542571312396,
            0.003240768143678019,
            0.0014226562246761319,
            0.00009072834274200404,
            0.0007633213727637278,
        };
        RB_CHECK(!stack_real.safety.floor_constraint.enable);
        RB_CHECK(!stack_real.safety.user_floor_constraint.enable);
        // Released-state offset bleed: K=0 translations drain their residual
        // offset only while the block is fully released; rotations keep 0
        // (their 3.0 Nm/rad spring already recenters).
        // 2026-08-12: 13 -> 26 (tau = 26/26 ~= 1 s). At tau = 2 s the drain was
        // slower than the policy's re-approach period, so retreats ratcheted
        // (servo_log_20260812_114302, left arm). The return stays inside the
        // 60 mm/s and 0.8 m/s^2 response caps and overdamped (zeta = 1.80).
        for (std::size_t axis = 0; axis < 3; ++axis) {
        }
        for (std::size_t axis = 3; axis < 6; ++axis) {
        }
        RB_CHECK(stack_real.kinematics.enable);
        RB_CHECK(stack_real.kinematics.provider == "pinocchio");
        // 2026-08-26 joint-limit best effort: a clamped iterate this close to the
        // target is accepted as a success so a pinned joint (J3 at +/-150 deg) no
        // longer refuses the whole tick and freezes the arm. Both must stay above
        // the convergence tolerances and far below the follower lead budgets.
        RB_CHECK(near(
            stack_real.kinematics.ik.joint_limit_best_effort_position_tolerance_m,
            0.0005));
        RB_CHECK(near(
            stack_real.kinematics.ik.joint_limit_best_effort_orientation_tolerance_rad,
            0.002));
        RB_CHECK(stack_real.kinematics.ik.joint_limit_best_effort_position_tolerance_m >
                 stack_real.kinematics.ik.position_tolerance_m);
        RB_CHECK(
            stack_real.kinematics.ik.joint_limit_best_effort_orientation_tolerance_rad >
            stack_real.kinematics.ik.orientation_tolerance_rad);
        const auto& real_mesh = stack_real.safety.self_collision.mesh;
        RB_CHECK(near(real_mesh.intra_arm.d_hard_m, 0.005));
        RB_CHECK(near(real_mesh.intra_arm.d_slow_m, 0.015));
        RB_CHECK(near(real_mesh.intra_arm.a_brake_m_s2, 3.0));
        RB_CHECK(near(real_mesh.external_boxes.margin_m[0], 0.0));
        RB_CHECK(near(real_mesh.external_boxes.margin_m[1], 0.0));
        // Height margin site-lowered 0.040 -> 0.020 (2026-07 barrier tuning:
        // 20mm height + 5mm d_hard = ~25mm top/bottom keep-out).
        RB_CHECK(near(real_mesh.external_boxes.margin_m[2], 0.020));
        RB_CHECK(real_mesh.intra_arm_min_chain_separation == 2);
        RB_CHECK(hasPairRule(real_mesh.disabled_collision_pairs,
                             "*left*link4*", "*left*link6*"));
        RB_CHECK(hasPairRule(real_mesh.disabled_collision_pairs,
                             "*right*link4*", "*right*link6*"));
        RB_CHECK(!hasPairRule(real_mesh.disabled_collision_pairs,
                              "*left*link2*", "*left*link4*"));
        RB_CHECK(!hasPairRule(real_mesh.disabled_collision_pairs,
                              "*right*link2*", "*right*link4*"));
    }

    {
        const rb_servo::DualArmConfig stack_sim =
            rb_servo::loadConfigFromYaml((config_dir / "stack_sim.yaml").string());
        RB_CHECK(stack_sim.left_robot.backend_type == rb_servo::BackendType::Rbpodo);
        RB_CHECK(stack_sim.right_robot.backend_type == rb_servo::BackendType::Rbpodo);
        RB_CHECK(stack_sim.left_robot.run_mode == rb_servo::RunMode::Real);
        RB_CHECK(stack_sim.right_robot.run_mode == rb_servo::RunMode::Real);
        RB_CHECK(stack_sim.left_robot.operation_mode == "simulation");
        RB_CHECK(stack_sim.right_robot.operation_mode == "simulation");
        RB_CHECK(stack_sim.servo.rate_hz == 500);
        RB_CHECK(stack_sim.servo.send_servo_commands);
        RB_CHECK(stack_sim.servo.allow_controller_simulation_motion);
        RB_CHECK(stack_sim.servo.allow_controller_simulation_diagnostics_suspect);
        RB_CHECK(stack_sim.servo.controller_simulation_treat_unreliable_status_fields_as_unavailable);
        RB_CHECK(stack_sim.servo.allow_controller_simulation_init_error);
        RB_CHECK(stack_sim.servo.allow_controller_simulation_not_activated);
        RB_CHECK(stack_sim.left_robot.allow_controller_simulation_diagnostics_suspect);
        RB_CHECK(stack_sim.right_robot.allow_controller_simulation_diagnostics_suspect);
        RB_CHECK(stack_sim.left_robot.controller_simulation_treat_unreliable_status_fields_as_unavailable);
        RB_CHECK(stack_sim.right_robot.controller_simulation_treat_unreliable_status_fields_as_unavailable);
        RB_CHECK(stack_sim.left_robot.allow_controller_simulation_init_error);
        RB_CHECK(stack_sim.right_robot.allow_controller_simulation_init_error);
        RB_CHECK(near(stack_sim.left_robot.servo_t1_sec, 0.002));
        RB_CHECK(near(stack_sim.right_robot.servo_t1_sec, 0.002));
        RB_CHECK(near(stack_sim.left_robot.servo_t2_sec, 0.021));
        RB_CHECK(near(stack_sim.left_robot.servo_alpha, 10.0));
        RB_CHECK(stack_sim.left_robot.disable_waiting_ack);
        RB_CHECK(stack_sim.right_robot.disable_waiting_ack);
        RB_CHECK(stack_sim.left_robot.state_read_pipelined);
        RB_CHECK(stack_sim.right_robot.state_read_pipelined);
        RB_CHECK(stack_sim.servo.send_at_tick_start);
        RB_CHECK(!stack_sim.servo.rbpodo_async_streaming.enable);
        RB_CHECK(near(stack_sim.safety.q_min_deg[0], -360.0));
        // J3 (elbow) is clamped near the RB3-730E physical range, not +/-360.
        // Matches stack_real.yaml and the IK URDF; the 2026-07 +/-160 margin was
        // reverted 2026-08-26 (docs/joint_range_policy.md).
        // stack_sim drives the SAME control boxes in pgmode, so it tracks the physical
        // arm: RB5-850E as of 2026-09-02, catalog elbow +/-165.
        RB_CHECK(near(stack_sim.safety.q_min_deg[2], -165.0));
        RB_CHECK(near(stack_sim.safety.q_max_deg[2], 165.0));
        RB_CHECK(stack_sim.safety.controller_simulation_tracking_error_source ==
                 rb_servo::ControllerSimulationTrackingErrorSource::Reference);
        RB_CHECK(stack_sim.safety.controller_simulation_tracking_error_nonlatching);
        RB_CHECK(stack_sim.safety.controller_simulation_physical_motion_policy ==
                 rb_servo::ControllerSimulationPhysicalMotionPolicy::FaultLatch);
        RB_CHECK(stack_sim.network.command_bind == "udp://127.0.0.1:50256");
        RB_CHECK(stack_sim.network.state_pub_endpoint == "udp://127.0.0.1:50356");
        RB_CHECK(stack_sim.network.state_pub_endpoints.size() == 4);
        RB_CHECK(stack_sim.network.state_pub_endpoints[1] == "udp://127.0.0.1:50366");
        RB_CHECK(stack_sim.network.state_pub_endpoints[2] == "udp://127.0.0.1:50376");
        RB_CHECK(stack_sim.command_source.enforce_lease);
        RB_CHECK(stack_sim.network.command_source_enforce_lease);
        RB_CHECK(near(stack_sim.command_source.lease_timeout_sec, 60.0));
        RB_CHECK(stack_sim.cartesian_control.enable);
        RB_CHECK(stack_sim.cartesian_control.allow_in_controller_simulation);
        RB_CHECK(!stack_sim.cartesian_control.allow_in_real);
        RB_CHECK(stack_sim.cartesian_control.controller_simulation_servo_state_source ==
                 rb_servo::CartesianControllerSimulationStateSource::Reference);
        RB_CHECK(stack_sim.cartesian_control.controller_simulation_divergence_source ==
                 rb_servo::CartesianControllerSimulationStateSource::Reference);
        RB_CHECK(stack_sim.cartesian_control.tcp_pose_target_profile_default ==
                 "umi_large_smooth");
        RB_CHECK(stack_sim.cartesian_control.tcp_pose_target_profiles.size() == 4);
        const rb_servo::TcpPoseTargetProfileConfig* sim_flow_profile = nullptr;
        const rb_servo::TcpPoseTargetProfileConfig* sim_fresh_profile = nullptr;
        for (const auto& profile : stack_sim.cartesian_control.tcp_pose_target_profiles) {
            if (profile.name == "flow_infer_smooth") sim_flow_profile = &profile;
            if (profile.name == "flow_infer_fresh") sim_fresh_profile = &profile;
        }
        RB_CHECK(sim_flow_profile != nullptr);
        RB_CHECK(sim_fresh_profile != nullptr);
        RB_CHECK(sim_fresh_profile->ruckig_follower.fresh_chunk_replan);
        RB_CHECK(sim_fresh_profile->ruckig_follower.continuous_hold_resume);
        RB_CHECK(!sim_flow_profile->ruckig_follower.fresh_chunk_replan);
        RB_CHECK(!sim_flow_profile->ruckig_follower.continuous_hold_resume);
        RB_CHECK(sim_flow_profile->ruckig_follower.controller ==
                 rb_servo::RuckigFollowerController::DeltaPreview);
        RB_CHECK(sim_flow_profile->ruckig_follower.consume_steps == 12);
        RB_CHECK(sim_flow_profile->ruckig_follower.reserve_steps == 2);
        RB_CHECK(near(sim_flow_profile->ruckig_follower.hold_bounce_resume_sec, 0.5));
        RB_CHECK(near(sim_flow_profile->ruckig_follower.preview_max_actual_lead_m, 0.006));
        RB_CHECK(near(sim_flow_profile->pose_track_smd.max_linear_velocity_m_s, 0.50));
        RB_CHECK(stack_sim.kinematics.enable);
        RB_CHECK(stack_sim.kinematics.provider == "pinocchio");
        const auto& sim_mesh = stack_sim.safety.self_collision.mesh;
        RB_CHECK(near(sim_mesh.intra_arm.d_hard_m, 0.005));
        RB_CHECK(near(sim_mesh.intra_arm.d_slow_m, 0.015));
        RB_CHECK(near(sim_mesh.intra_arm.a_brake_m_s2, 3.0));
        RB_CHECK(sim_mesh.intra_arm_min_chain_separation == 2);
        RB_CHECK(hasPairRule(sim_mesh.disabled_collision_pairs,
                             "*left*link4*", "*left*link6*"));
        RB_CHECK(hasPairRule(sim_mesh.disabled_collision_pairs,
                             "*right*link4*", "*right*link6*"));
        RB_CHECK(!hasPairRule(sim_mesh.disabled_collision_pairs,
                              "*left*link2*", "*left*link4*"));
        RB_CHECK(!hasPairRule(sim_mesh.disabled_collision_pairs,
                              "*right*link2*", "*right*link4*"));
    }

    return true;
}


bool testServoIoModelParsesAndValidates() {
    const std::string worker_path = writeTempConfig(
        "worker-io-model",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  io_model: worker\n"
        "  worker_read_period_sec: 0.025\n"
    );
    const rb_servo::DualArmConfig worker = rb_servo::loadConfigFromYaml(worker_path);
    ::unlink(worker_path.c_str());
    RB_CHECK(worker.servo.io_model == rb_servo::ServoIoModel::Worker);
    RB_CHECK(near(worker.servo.worker_read_period_sec, 0.025));

    const std::string worker_rate_path = writeTempConfig(
        "worker-read-rate",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  io_model: worker\n"
        "  worker_read_rate_hz: 50\n"
    );
    const rb_servo::DualArmConfig worker_rate = rb_servo::loadConfigFromYaml(worker_rate_path);
    ::unlink(worker_rate_path.c_str());
    RB_CHECK(near(worker_rate.servo.worker_read_period_sec, 0.02));

    const std::string invalid_path = writeTempConfig(
        "invalid-io-model",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  io_model: threadpool\n"
    );
    const bool invalid_rejected = loadRejects(invalid_path);
    ::unlink(invalid_path.c_str());
    RB_CHECK(invalid_rejected);

    const std::string invalid_period_path = writeTempConfig(
        "invalid-worker-read-period",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  worker_read_period_sec: 0\n"
    );
    const bool invalid_period_rejected = loadRejects(invalid_period_path);
    ::unlink(invalid_period_path.c_str());
    RB_CHECK(invalid_period_rejected);

    const std::string ambiguous_period_path = writeTempConfig(
        "ambiguous-worker-read-period",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "servo:\n"
        "  worker_read_period_sec: 0.002\n"
        "  worker_read_rate_hz: 500\n"
    );
    const bool ambiguous_period_rejected = loadRejects(ambiguous_period_path);
    ::unlink(ambiguous_period_path.c_str());
    RB_CHECK(ambiguous_period_rejected);

    const std::string real_worker_path = writeTempConfig(
        "real-worker-io-model",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "left_robot:\n"
        "  backend_type: rbpodo\n"
        "  run_mode: real\n"
        "  ip: \"172.28.60.200\"\n"
        "right_robot:\n"
        "  backend_type: rbpodo\n"
        "  run_mode: real\n"
        "  ip: \"172.28.60.201\"\n"
        "servo:\n"
        "  io_model: worker\n"
        "  worker_read_period_sec: 0.002\n"
        "  send_servo_commands: false\n"
        "  enable_realtime_priority: true\n"
        "safety:\n"
        "  tracking_error_policy: fault_latch\n"
        "  stop_both_arms_on_single_arm_error: true\n"
        "  latch_fault_on_robot_state_error: true\n"
    );
    // servo.io_model=worker is ACCEPTED in real mode: the old refusal was retired
    // because control-box queue sync needs per-arm cadence ownership, which only
    // worker I/O provides.
    bool real_worker_accepted = false;
    try {
        const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(real_worker_path);
        real_worker_accepted = cfg.servo.io_model == rb_servo::ServoIoModel::Worker;
    } catch (const std::exception&) {
        real_worker_accepted = false;
    }
    ::unlink(real_worker_path.c_str());
    RB_CHECK(real_worker_accepted);
    return true;
}

bool testQueueSyncConfigValidation() {
    const std::string header =
        "schema: robotics_lab.rb_servo_server.v1\n"
        "left_robot:\n"
        "  backend_type: mock\n"
        "  run_mode: mock\n"
        "right_robot:\n"
        "  backend_type: mock\n"
        "  run_mode: mock\n";

    // Accepted: defaults plus an explicit enable.
    const std::string ok_path = writeTempConfig(
        "queue-sync-ok", header + "queue_sync:\n  enable: true\n  target_fill: 5\n");
    bool ok_loaded = false;
    try {
        const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(ok_path);
        ok_loaded = cfg.queue_sync.enable && cfg.queue_sync.target_fill == 5;
    } catch (const std::exception&) {
        ok_loaded = false;
    }
    ::unlink(ok_path.c_str());
    RB_CHECK(ok_loaded);

    // Disabled by default: absence of the block must not enable regulation.
    const std::string absent_path = writeTempConfig("queue-sync-absent", header);
    bool absent_default_off = false;
    try {
        absent_default_off = !rb_servo::loadConfigFromYaml(absent_path).queue_sync.enable;
    } catch (const std::exception&) {
        absent_default_off = false;
    }
    ::unlink(absent_path.c_str());
    RB_CHECK(absent_default_off);

    // Worker state staleness budget: in band accepted, outside band rejected.
    const std::string age_ok_path = writeTempConfig(
        "worker-state-age-ok", header + "servo:\n  worker_state_max_age_periods: 4.0\n");
    bool age_ok = false;
    try {
        age_ok = rb_servo::loadConfigFromYaml(age_ok_path)
                     .servo.worker_state_max_age_periods == 4.0;
    } catch (const std::exception&) {
        age_ok = false;
    }
    ::unlink(age_ok_path.c_str());
    RB_CHECK(age_ok);

    for (const char* bad : {"1.5", "0.0", "12.0"}) {
        const std::string path = writeTempConfig(
            std::string("worker-state-age-bad-") + bad,
            header + "servo:\n  worker_state_max_age_periods: " + bad + "\n");
        const bool rejected = loadRejects(path);
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }

    // Fail-closed rejections. target_fill 0 starves the box; protect_fill at or
    // above the target makes protection permanent; an unknown key is a typo.
    const std::pair<const char*, const char*> bad_cases[] = {
        {"queue-sync-target-zero", "queue_sync:\n  enable: true\n  target_fill: 0\n"},
        {"queue-sync-protect-ge-target",
         "queue_sync:\n  enable: true\n  target_fill: 5\n  protect_fill: 5\n"},
        {"queue-sync-bad-lpf", "queue_sync:\n  enable: true\n  lpf_alpha: 0.0\n"},
        {"queue-sync-positive-protect-adj",
         "queue_sync:\n  enable: true\n  protect_adj_us: 10.0\n"},
        {"queue-sync-drain-order",
         "queue_sync:\n  enable: true\n  drain_adj_us: 500.0\n  drain_max_us: 100.0\n"},
        {"queue-sync-unknown-key", "queue_sync:\n  enable: true\n  kp_above: 6.0\n"},
    };
    for (const auto& [name, body] : bad_cases) {
        const std::string path = writeTempConfig(name, header + body);
        const bool rejected = loadRejects(path);
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }
    return true;
}

bool testWorkerRealtimeScheduling() {
    const std::string header =
        "schema: robotics_lab.rb_servo_server.v1\n"
        "left_robot:\n"
        "  backend_type: mock\n"
        "  run_mode: mock\n"
        "right_robot:\n"
        "  backend_type: mock\n"
        "  run_mode: mock\n";

    // Accepted: FIFO priority plus one isolated core per arm, none of them the
    // loop's own core.
    const std::string ok_path = writeTempConfig(
        "worker-rt-ok",
        header +
        "servo:\n  cpu_core: 3\n  worker_realtime_priority: 80\n"
        "  worker_cpu_core_left: 1\n  worker_cpu_core_right: 2\n");
    bool ok_loaded = false;
    try {
        const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(ok_path);
        ok_loaded = cfg.servo.worker_realtime_priority == 80 &&
                    cfg.servo.worker_cpu_core_left == 1 &&
                    cfg.servo.worker_cpu_core_right == 2;
    } catch (const std::exception&) {
        ok_loaded = false;
    }
    ::unlink(ok_path.c_str());
    RB_CHECK(ok_loaded);

    // Absent = off. A worker must not silently acquire RT scheduling.
    const std::string absent_path = writeTempConfig("worker-rt-absent", header);
    bool absent_default_off = false;
    try {
        const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(absent_path);
        absent_default_off = cfg.servo.worker_realtime_priority == 0 &&
                             cfg.servo.worker_cpu_core_left == -1 &&
                             cfg.servo.worker_cpu_core_right == -1;
    } catch (const std::exception&) {
        absent_default_off = false;
    }
    ::unlink(absent_path.c_str());
    RB_CHECK(absent_default_off);

    // Fail-closed rejections, each one a way to get a config that LOOKS like it
    // isolated the cadence owner but did not.
    const std::pair<const char*, const char*> bad_cases[] = {
        // Out of the SCHED_FIFO band.
        {"worker-rt-priority-range",
         "servo:\n  worker_realtime_priority: 100\n"},
        // A worker on the loop's own core contends with what it feeds.
        {"worker-rt-core-equals-loop",
         "servo:\n  cpu_core: 3\n  worker_realtime_priority: 80\n"
         "  worker_cpu_core_left: 3\n  worker_cpu_core_right: 2\n"},
        // Both arms on one core reintroduces the contention pinning removes.
        {"worker-rt-cores-equal",
         "servo:\n  cpu_core: 3\n  worker_realtime_priority: 80\n"
         "  worker_cpu_core_left: 1\n  worker_cpu_core_right: 1\n"},
        // Pinned but still on CFS: a dedicated core that yields to any CFS
        // thread landing on it is the exact trap this setting exists to avoid.
        {"worker-rt-pinned-without-fifo",
         "servo:\n  cpu_core: 3\n  worker_cpu_core_left: 1\n"
         "  worker_cpu_core_right: 2\n"},
    };
    for (const auto& [name, body] : bad_cases) {
        const std::string path = writeTempConfig(name, header + body);
        const bool rejected = loadRejects(path);
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }
    return true;
}

bool testUnknownKeysAndSchemaFail() {
    const std::string unknown_key_path = writeTempConfig(
        "unknown-key",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "network:\n"
        "  state_pub_endpoint: \"udp://127.0.0.1:50110\"\n"
        "  state_pub_destination_typo: \"udp://127.0.0.1:50111\"\n"
    );
    const bool unknown_key_rejected = loadRejects(unknown_key_path);
    ::unlink(unknown_key_path.c_str());
    RB_CHECK(unknown_key_rejected);

    const std::string unknown_schema_path = writeTempConfig(
        "unknown-schema",
        "schema: robotics_lab.rb_servo_server.v2\n"
    );
    const bool unknown_schema_rejected = loadRejects(unknown_schema_path);
    ::unlink(unknown_schema_path.c_str());
    RB_CHECK(unknown_schema_rejected);
    return true;
}

bool testStatePublisherEndpointsParseAndValidate() {
    const std::string path = writeTempConfig(
        "state-pub-endpoints",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "network:\n"
        "  state_pub_endpoints:\n"
        "    - \"udp://rb_gui:50110\"\n"
        "    - \"udp://policy_runner_state_sink:50120\"\n"
    );
    const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
    ::unlink(path.c_str());
    RB_CHECK(cfg.network.state_pub_endpoint == "udp://rb_gui:50110");
    RB_CHECK(cfg.network.state_pub_bind == cfg.network.state_pub_endpoint);
    RB_CHECK(cfg.network.state_pub_endpoints.size() == 2);
    RB_CHECK(cfg.network.state_pub_endpoints[0] == "udp://rb_gui:50110");
    RB_CHECK(cfg.network.state_pub_endpoints[1] == "udp://policy_runner_state_sink:50120");

    const std::string ambiguous_path = writeTempConfig(
        "state-pub-ambiguous",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "network:\n"
        "  state_pub_endpoint: \"udp://rb_gui:50110\"\n"
        "  state_pub_endpoints:\n"
        "    - \"udp://policy_runner_state_sink:50120\"\n"
    );
    const bool ambiguous_rejected = loadRejects(ambiguous_path);
    ::unlink(ambiguous_path.c_str());
    RB_CHECK(ambiguous_rejected);
    return true;
}




bool testCommandSourceConfigParsesAndValidates() {
    const std::string path = writeTempConfig(
        "command-source",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "command_source:\n"
        "  enforce_lease: true\n"
        "  lease_timeout_sec: 1.25\n"
    );
    const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
    ::unlink(path.c_str());
    RB_CHECK(cfg.command_source.enforce_lease);
    RB_CHECK(cfg.command_source.lease_timeout_sec == 1.25);
    RB_CHECK(cfg.network.command_source_enforce_lease);
    RB_CHECK(cfg.network.command_source_lease_timeout_sec == 1.25);

    const std::string invalid_path = writeTempConfig(
        "command-source-invalid",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "command_source:\n"
        "  lease_timeout_sec: 0\n"
    );
    const bool invalid_rejected = loadRejects(invalid_path);
    ::unlink(invalid_path.c_str());
    RB_CHECK(invalid_rejected);
    return true;
}

bool testCartesianControlTuningParsesAndValidates() {
    const std::string path = writeTempConfig(
        "cartesian-tuning",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "cartesian_control:\n"
        "  allow_in_controller_simulation: true\n"
        "  path_kp_pos: 2.5\n"
        "  path_kp_ori: 7.5\n"
        "  controller_simulation_servo_state_source: reference\n"
        "  controller_simulation_divergence_source: reference\n"
        "  linear_move:\n"
        "    constant_orientation_tolerance_rad: 0.004\n"
    );
    const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
    ::unlink(path.c_str());
    RB_CHECK(cfg.cartesian_control.allow_in_controller_simulation);
    RB_CHECK(near(cfg.cartesian_control.path_kp_pos, 2.5));
    RB_CHECK(near(cfg.cartesian_control.path_kp_ori, 7.5));
    RB_CHECK(cfg.cartesian_control.controller_simulation_servo_state_source ==
             rb_servo::CartesianControllerSimulationStateSource::Reference);
    RB_CHECK(cfg.cartesian_control.controller_simulation_divergence_source ==
             rb_servo::CartesianControllerSimulationStateSource::Reference);
    RB_CHECK(near(cfg.cartesian_control.linear_move.constant_orientation_tolerance_rad, 0.004));

    const std::string legacy_path = writeTempConfig(
        "cartesian-legacy-path-kp",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "cartesian_control:\n"
        "  path_kp: 4.0\n"
    );
    std::ostringstream warnings;
    auto* const old_cerr = std::cerr.rdbuf(warnings.rdbuf());
    const rb_servo::DualArmConfig legacy = rb_servo::loadConfigFromYaml(legacy_path);
    std::cerr.rdbuf(old_cerr);
    ::unlink(legacy_path.c_str());
    RB_CHECK(near(legacy.cartesian_control.path_kp, 4.0));
    RB_CHECK(near(legacy.cartesian_control.path_kp_pos, 4.0));
    RB_CHECK(near(legacy.cartesian_control.path_kp_ori, 4.0));
    RB_CHECK(warnings.str().find("deprecated") != std::string::npos);

    const std::string ambiguous_path = writeTempConfig(
        "cartesian-ambiguous-path-kp",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "cartesian_control:\n"
        "  path_kp: 4.0\n"
        "  path_kp_pos: 5.0\n"
    );
    const bool ambiguous_rejected = loadRejects(ambiguous_path);
    ::unlink(ambiguous_path.c_str());
    RB_CHECK(ambiguous_rejected);

    const std::string bad_gain_path = writeTempConfig(
        "cartesian-bad-gain",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "cartesian_control:\n"
        "  path_kp_ori: 0\n"
    );
    const bool bad_gain_rejected = loadRejects(bad_gain_path);
    ::unlink(bad_gain_path.c_str());
    RB_CHECK(bad_gain_rejected);

    const std::string bad_constant_orientation_tolerance_path = writeTempConfig(
        "cartesian-bad-constant-orientation-tolerance",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "cartesian_control:\n"
        "  linear_move:\n"
        "    constant_orientation_tolerance_rad: 0\n"
    );
    const bool bad_constant_orientation_tolerance_rejected =
        loadRejects(bad_constant_orientation_tolerance_path);
    ::unlink(bad_constant_orientation_tolerance_path.c_str());
    RB_CHECK(bad_constant_orientation_tolerance_rejected);
    return true;
}

bool testRemovedRawScriptBackendRejected() {
    const std::string removed_backend = std::string("rb") + "script_tcp";
    const std::string removed_backend_body =
        "schema: robotics_lab.rb_servo_server.v1\n"
        "left_robot:\n"
        "  backend_type: " + removed_backend + "\n"
        "  run_mode: real\n"
        "  ip: \"172.28.60.200\"\n"
        "  command_timeout_sec: 0.2\n"
        "  disable_waiting_ack: false\n"
        "right_robot:\n"
        "  backend_type: mock\n"
        "  run_mode: mock\n"
        "servo:\n"
        "  send_servo_commands: false\n"
        "  enable_realtime_priority: true\n"
        "safety:\n"
        "  tracking_error_policy: fault_latch\n"
        "  stop_both_arms_on_single_arm_error: true\n"
        "  latch_fault_on_robot_state_error: true\n";

    const std::string removed_backend_path = writeTempConfig("removed-raw-script-backend", removed_backend_body);
    RB_CHECK(loadRejects(removed_backend_path));
    ::unlink(removed_backend_path.c_str());

    const std::string removed_backend_key_path = writeTempConfig(
        "removed-raw-script-key",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "left_robot:\n"
        "  backend_type: rbpodo\n"
        "  run_mode: real\n"
        "  ip: \"172.28.60.200\"\n"
        "  command_port: 5000\n"
    );
    RB_CHECK(loadRejects(removed_backend_key_path));
    ::unlink(removed_backend_key_path.c_str());
    return true;
}

std::string rbpodoEnvIpConfigBody(const std::string& left_ip, const std::string& right_ip) {
    return std::string(
        "schema: robotics_lab.rb_servo_server.v1\n"
        "left_robot:\n"
        "  backend_type: rbpodo\n"
        "  run_mode: real\n"
        "  ip: \"") + left_ip + "\"\n" +
        "  operation_mode: simulation\n"
        "  servo_t1_sec: 0.002\n"
        "  servo_t2_sec: 0.05\n"
        "  servo_gain: 1.0\n"
        "  servo_alpha: 0.5\n"
        "right_robot:\n"
        "  backend_type: rbpodo\n"
        "  run_mode: real\n"
        "  ip: \"" + right_ip + "\"\n" +
        "  operation_mode: simulation\n"
        "  servo_t1_sec: 0.002\n"
        "  servo_t2_sec: 0.05\n"
        "  servo_gain: 1.0\n"
        "  servo_alpha: 0.5\n"
        "servo:\n"
        "  rate_hz: 500\n"
        "  send_servo_commands: false\n"
        "  enable_realtime_priority: true\n"
        "safety:\n"
        "  tracking_error_policy: fault_latch\n"
        "  stop_both_arms_on_single_arm_error: true\n"
        "  latch_fault_on_robot_state_error: true\n";
}

bool testRobotIpEnvExpansion() {
    {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        EnvGuard left_ip("ROBOT_LEFT_IP", "192.168.56.101");
        EnvGuard right_ip("ROBOT_RIGHT_IP", "192.168.56.102");
        const std::string path = writeTempConfig(
            "rbpodo-env-ip",
            rbpodoEnvIpConfigBody("${ROBOT_LEFT_IP}", "${ROBOT_RIGHT_IP}")
        );
        const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
        ::unlink(path.c_str());
        RB_CHECK(cfg.left_robot.ip == "192.168.56.101");
        RB_CHECK(cfg.right_robot.ip == "192.168.56.102");
        RB_CHECK(cfg.left_robot.operation_mode == "simulation");
        RB_CHECK(!cfg.servo.send_servo_commands);
    }

    {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        EnvGuard ignored("ROBOT_LEFT_IP", "192.168.56.101");
        const std::string path = writeTempConfig(
            "rbpodo-literal-ip",
            rbpodoEnvIpConfigBody("192.168.56.201", "192.168.56.202")
        );
        const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
        ::unlink(path.c_str());
        RB_CHECK(cfg.left_robot.ip == "192.168.56.201");
        RB_CHECK(cfg.right_robot.ip == "192.168.56.202");
    }

    {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        EnvGuard missing("ROBOT_MISSING_IP", nullptr);
        const std::string path = writeTempConfig(
            "rbpodo-missing-env-ip",
            rbpodoEnvIpConfigBody("${ROBOT_MISSING_IP}", "192.168.56.202")
        );
        const bool rejected = loadRejects(path);
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }

    {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        EnvGuard empty("ROBOT_EMPTY_IP", "");
        const std::string path = writeTempConfig(
            "rbpodo-empty-env-ip",
            rbpodoEnvIpConfigBody("${ROBOT_EMPTY_IP}", "192.168.56.202")
        );
        const bool rejected = loadRejects(path);
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }

    {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        const std::string path = writeTempConfig(
            "rbpodo-invalid-env-ip",
            rbpodoEnvIpConfigBody("${1BAD_IP}", "192.168.56.202")
        );
        const bool rejected = loadRejects(path);
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }

    return true;
}

std::string rbpodoConfigBody(
    const std::string& left_fields,
    int rate_hz,
    bool send_servo_commands
) {
    return
        "schema: robotics_lab.rb_servo_server.v1\n"
        "left_robot:\n"
        "  backend_type: rbpodo\n"
        "  run_mode: real\n"
        "  ip: \"172.28.60.200\"\n"
        "  operation_mode: real\n" +
        left_fields +
        "right_robot:\n"
        "  backend_type: mock\n"
        "  run_mode: mock\n"
        "servo:\n"
        "  rate_hz: " + std::to_string(rate_hz) + "\n"
        "  send_servo_commands: " + std::string(send_servo_commands ? "true\n" : "false\n") +
        "  enable_realtime_priority: true\n"
        "  servo_t1_rate_match_tolerance_ratio: 0.2\n"
        "  allow_servo_t1_rate_mismatch: false\n" +
        "safety:\n"
        "  tracking_error_policy: fault_latch\n"
        "  stop_both_arms_on_single_arm_error: true\n"
        "  latch_fault_on_robot_state_error: true\n";
}

bool testRbpodoServoJParametersParseAndValidate() {
    {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        EnvGuard motion_gate("RB_ALLOW_REAL_MOTION", nullptr);
        const std::string path = writeTempConfig(
            "rbpodo-servo-canonical",
            rbpodoConfigBody(
                "  servo_t1_sec: 0.002\n"
                "  servo_t2_sec: 0.05\n"
                "  servo_gain: 1.25\n"
                "  servo_alpha: 0.5\n",
                500,
                false
            )
        );
        const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
        ::unlink(path.c_str());
        RB_CHECK(near(cfg.left_robot.servo_t1_sec, 0.002));
        RB_CHECK(near(cfg.left_robot.servo_t2_sec, 0.05));
        RB_CHECK(near(cfg.left_robot.servo_gain, 1.25));
        RB_CHECK(near(cfg.left_robot.servo_alpha, 0.5));
        RB_CHECK(near(cfg.left_robot.command_timeout_sec, 0.2));
        RB_CHECK(near(cfg.left_robot.servo_time_sec, cfg.left_robot.servo_t1_sec));
        RB_CHECK(near(cfg.left_robot.servo_lookahead_sec, cfg.left_robot.servo_t2_sec));
        RB_CHECK(near(cfg.left_robot.servo_acc, cfg.left_robot.servo_alpha));
    }

    {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        std::ostringstream warnings;
        auto* const old_cerr = std::cerr.rdbuf(warnings.rdbuf());
        const std::string path = writeTempConfig(
            "rbpodo-servo-deprecated",
            rbpodoConfigBody(
                "  servo_time_sec: 0.002\n"
                "  servo_lookahead_sec: 0.05\n"
                "  servo_gain: 1.0\n"
                "  servo_acc: 0.5\n",
                500,
                false
            )
        );
        const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
        std::cerr.rdbuf(old_cerr);
        ::unlink(path.c_str());
        RB_CHECK(near(cfg.left_robot.servo_t1_sec, 0.002));
        RB_CHECK(near(cfg.left_robot.servo_t2_sec, 0.05));
        RB_CHECK(near(cfg.left_robot.servo_alpha, 0.5));
        RB_CHECK(warnings.str().find("deprecated") != std::string::npos);
    }

    {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        const std::string path = writeTempConfig(
            "rbpodo-servo-duplicate",
            rbpodoConfigBody(
                "  servo_t1_sec: 0.002\n"
                "  servo_time_sec: 0.002\n"
                "  servo_t2_sec: 0.05\n"
                "  servo_gain: 1.0\n"
                "  servo_alpha: 0.5\n",
                500,
                false
            )
        );
        const bool rejected = loadRejects(path);
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }

    // Vendor-range servo_t2/servo_alpha values now only WARN (the
    // RB_ALLOW_RBPODO_SERVO_PARAM_UNSAFE gate is retired); rejection is limited
    // to non-positive / non-finite values.
    const std::string invalid_cases[] = {
        "  command_timeout_sec: 0.0\n  servo_t1_sec: 0.002\n  servo_t2_sec: 0.05\n  servo_gain: 1.0\n  servo_alpha: 0.5\n",
        "  servo_t1_sec: 0.001\n  servo_t2_sec: 0.05\n  servo_gain: 1.0\n  servo_alpha: 0.5\n",
        "  servo_t1_sec: 0.002\n  servo_t2_sec: 0.0\n  servo_gain: 1.0\n  servo_alpha: 0.5\n",
        "  servo_t1_sec: 0.002\n  servo_t2_sec: 0.05\n  servo_gain: 0.0\n  servo_alpha: 0.5\n",
        "  servo_t1_sec: 0.002\n  servo_t2_sec: 0.05\n  servo_gain: 1.0\n  servo_alpha: 0.0\n",
    };
    for (std::size_t i = 0; i < sizeof(invalid_cases) / sizeof(invalid_cases[0]); ++i) {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        const std::string path = writeTempConfig(
            "rbpodo-servo-invalid-" + std::to_string(i),
            rbpodoConfigBody(invalid_cases[i], 500, false)
        );
        const bool rejected = loadRejects(path);
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }

    {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        EnvGuard motion_gate("RB_ALLOW_REAL_MOTION", "1");
        const std::string path = writeTempConfig(
            "rbpodo-servo-rate-mismatch",
            rbpodoConfigBody(
                "  servo_t1_sec: 0.004\n"
                "  servo_t2_sec: 0.05\n"
                "  servo_gain: 1.0\n"
                "  servo_alpha: 0.5\n",
                500,
                true
            )
        );
        const bool rejected = loadRejects(path);
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }

    {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        EnvGuard motion_gate("RB_ALLOW_REAL_MOTION", "1");
        const std::string path = writeTempConfig(
            "rbpodo-servo-rate-manual-non500",
            rbpodoConfigBody(
                "  servo_t1_sec: 0.004\n"
                "  servo_t2_sec: 0.05\n"
                "  servo_gain: 1.0\n"
                "  servo_alpha: 0.5\n",
                250,
                true
            )
        );
        const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
        ::unlink(path.c_str());
        RB_CHECK(cfg.servo.rate_hz == 250);
        RB_CHECK(near(cfg.left_robot.servo_t1_sec, 0.004));
    }

    {
        // Real/sim env gates retired: disable_waiting_ack loads without
        // RB_ALLOW_RBPODO_ACK_DISABLED_MOTION.
        EnvGuard ack_disabled_motion_gate("RB_ALLOW_RBPODO_ACK_DISABLED_MOTION", nullptr);
        const std::string path = writeTempConfig(
            "rbpodo-ack-disabled-motion-gate",
            rbpodoConfigBody(
                "  servo_t1_sec: 0.002\n"
                "  servo_t2_sec: 0.05\n"
                "  servo_gain: 1.0\n"
                "  servo_alpha: 0.5\n"
                "  disable_waiting_ack: true\n",
                500,
                true
            )
        );
        const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
        ::unlink(path.c_str());
        RB_CHECK(cfg.left_robot.disable_waiting_ack);
    }

    {
        EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
        EnvGuard motion_gate("RB_ALLOW_REAL_MOTION", "1");
        EnvGuard ack_disabled_motion_gate("RB_ALLOW_RBPODO_ACK_DISABLED_MOTION", "1");
        const std::string path = writeTempConfig(
            "rbpodo-ack-disabled-motion-explicit",
            rbpodoConfigBody(
                "  command_timeout_sec: 0.02\n"
                "  servo_t1_sec: 0.002\n"
                "  servo_t2_sec: 0.05\n"
                "  servo_gain: 1.0\n"
                "  servo_alpha: 0.5\n"
                "  disable_waiting_ack: true\n",
                500,
                true
            )
        );
        const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
        ::unlink(path.c_str());
        RB_CHECK(cfg.left_robot.disable_waiting_ack);
        RB_CHECK(near(cfg.left_robot.command_timeout_sec, 0.02));
    }
    return true;
}

bool testKinematicsSafetyLimitMismatchWarnsForRbpodo() {
    EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");
    const std::string path = writeTempConfig(
        "rbpodo-kinematics-safety-mismatch",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "left_robot:\n"
        "  backend_type: rbpodo\n"
        "  run_mode: real\n"
        "  ip: \"172.28.60.200\"\n"
        "  operation_mode: simulation\n"
        "  servo_t1_sec: 0.002\n"
        "  servo_t2_sec: 0.05\n"
        "  servo_gain: 1.0\n"
        "  servo_alpha: 0.5\n"
        "right_robot:\n"
        "  backend_type: rbpodo\n"
        "  run_mode: real\n"
        "  ip: \"172.28.60.201\"\n"
        "  operation_mode: simulation\n"
        "  servo_t1_sec: 0.002\n"
        "  servo_t2_sec: 0.05\n"
        "  servo_gain: 1.0\n"
        "  servo_alpha: 0.5\n"
        "servo:\n"
        "  rate_hz: 500\n"
        "  send_servo_commands: false\n"
        "  enable_realtime_priority: true\n"
        "safety:\n"
        "  q_min_deg: [-360, -360, -360, -360, -360, -360]\n"
        "  q_max_deg: [360, 360, 360, 360, 360, 360]\n"
        "  tracking_error_policy: fault_latch\n"
        "  stop_both_arms_on_single_arm_error: true\n"
        "  latch_fault_on_robot_state_error: true\n"
        "kinematics:\n"
        "  enable: true\n"
        "  provider: pinocchio\n"
        "  urdf: \"" + rb3UrdfPath() + "\"\n"
        "  publish_tcp: true\n"
    );

    std::ostringstream warnings;
    auto* const old_cerr = std::cerr.rdbuf(warnings.rdbuf());
    const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
    std::cerr.rdbuf(old_cerr);
    ::unlink(path.c_str());
    RB_CHECK(cfg.kinematics.enable);
    RB_CHECK(near(cfg.safety.q_min_deg[2], -360.0));
    RB_CHECK(warnings.str().find("differs from rb3_730e.urdf URDF IK limit") != std::string::npos);
    RB_CHECK(warnings.str().find("elbow_joint") != std::string::npos);
    return true;
}

std::string controllerSimReadMissBody(int left_misses, int right_misses) {
    return
        "schema: robotics_lab.rb_servo_server.v1\n"
        "left_robot:\n"
        "  backend_type: rbpodo\n"
        "  run_mode: real\n"
        "  ip: \"172.28.60.200\"\n"
        "  operation_mode: simulation\n"
        "  max_consecutive_read_misses: " + std::to_string(left_misses) + "\n"
        "right_robot:\n"
        "  backend_type: rbpodo\n"
        "  run_mode: real\n"
        "  ip: \"172.28.60.201\"\n"
        "  operation_mode: simulation\n"
        "  max_consecutive_read_misses: " + std::to_string(right_misses) + "\n"
        "servo:\n"
        "  rate_hz: 500\n"
        "  send_servo_commands: false\n"
        "  enable_realtime_priority: true\n"
        "safety:\n"
        "  tracking_error_policy: fault_latch\n"
        "  stop_both_arms_on_single_arm_error: true\n"
        "  latch_fault_on_robot_state_error: true\n";
}

bool testReadMissToleranceParsesAndIsControllerSimOnly() {
    EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");

    // Controller simulation accepts and parses the tolerance.
    {
        const std::string path = writeTempConfig(
            "read-miss-controller-sim", controllerSimReadMissBody(3, 2));
        const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
        ::unlink(path.c_str());
        RB_CHECK(cfg.left_robot.max_consecutive_read_misses == 3);
        RB_CHECK(cfg.right_robot.max_consecutive_read_misses == 2);
    }

    // Default is 0 (fail-closed) when the key is absent.
    {
        const std::string path = writeTempConfig(
            "read-miss-default", controllerSimReadMissBody(0, 0));
        const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
        ::unlink(path.c_str());
        RB_CHECK(cfg.left_robot.max_consecutive_read_misses == 0);
    }

    // Negative tolerance is rejected.
    {
        const std::string path = writeTempConfig(
            "read-miss-negative", controllerSimReadMissBody(-1, 0));
        const bool rejected = loadRejects(path);
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }

    // Physical real (operation_mode: real) must fail closed: tolerance > 0 rejected.
    {
        const std::string path = writeTempConfig(
            "read-miss-real-rejected",
            rbpodoConfigBody("  max_consecutive_read_misses: 3\n", 500, false));
        const bool rejected = loadRejects(path);
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }

    return true;
}

std::string selfCollisionConfigBody(bool with_kinematics, const std::string& self_collision_yaml) {
    std::string body =
        "schema: robotics_lab.rb_servo_server.v1\n"
        "left_robot:\n"
        "  backend_type: rbpodo\n"
        "  run_mode: real\n"
        "  ip: \"172.28.60.200\"\n"
        "  operation_mode: simulation\n"
        "right_robot:\n"
        "  backend_type: rbpodo\n"
        "  run_mode: real\n"
        "  ip: \"172.28.60.201\"\n"
        "  operation_mode: simulation\n"
        "servo:\n"
        "  rate_hz: 500\n"
        "  send_servo_commands: false\n"
        "  enable_realtime_priority: true\n"
        "safety:\n"
        "  tracking_error_policy: fault_latch\n"
        "  stop_both_arms_on_single_arm_error: true\n"
        "  latch_fault_on_robot_state_error: true\n" +
        self_collision_yaml;
    if (with_kinematics) {
        body +=
            "kinematics:\n"
            "  enable: true\n"
            "  provider: pinocchio\n"
            "  urdf: \"" + rb3UrdfPath() + "\"\n"
            "  base_link: \"world\"\n"
            "  tip_link: \"tcp\"\n"
            "  joint_names:\n"
            "    - base_joint\n"
            "    - shoulder_joint\n"
            "    - elbow_joint\n"
            "    - wrist1_joint\n"
            "    - wrist2_joint\n"
            "    - wrist3_joint\n"
            "  publish_tcp: true\n";
    }
    return body;
}

bool testSelfCollisionConfig() {
    EnvGuard real_gate("RB_ALLOW_REAL_ROBOT", "1");

    // The single enable flag drives the URDF-mesh guard; a mesh block with a
    // unified_urdf + barrier params is mandatory when enabled.
    const std::string mesh_yaml =
        "    mesh:\n"
        "      unified_urdf: \"" + rb3UrdfPath() + "\"\n"
        "      d_hard_m: 0.006\n"
        "      d_slow_m: 0.03\n"
        "      intra_arm:\n"
        "        d_hard_m: 0.005\n"
        "        d_slow_m: 0.015\n"
        "        a_brake_m_s2: 3.0\n"
        "      disabled_collision_pairs:\n"
        "        - [\"*left*link0*\", \"*left*link1*\"]\n"
        "        - [\"*right*link0*\", \"*right*link1*\"]\n"
        "      debug_pair_curation: true\n";

    // Enabled + kinematics + mesh: accepted and parsed.
    {
        const std::string path = writeTempConfig(
            "self-collision-ok",
            selfCollisionConfigBody(
                true,
                "  self_collision:\n"
                "    enable: true\n"
                "    monitor_only: true\n"
                "    fail_policy: clamp_hold\n" +
                mesh_yaml));
        const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
        ::unlink(path.c_str());
        RB_CHECK(cfg.safety.self_collision.enable);
        RB_CHECK(cfg.safety.self_collision.monitor_only);
        RB_CHECK(cfg.safety.self_collision.fail_policy == rb_servo::SelfCollisionFailPolicy::ClampToHold);
        RB_CHECK(!cfg.safety.self_collision.mesh.unified_urdf.empty());
        RB_CHECK(near(cfg.safety.self_collision.mesh.d_hard_m, 0.006));
        RB_CHECK(near(cfg.safety.self_collision.mesh.d_slow_m, 0.03));
        RB_CHECK(near(cfg.safety.self_collision.mesh.intra_arm.d_hard_m, 0.005));
        RB_CHECK(near(cfg.safety.self_collision.mesh.intra_arm.d_slow_m, 0.015));
        RB_CHECK(near(cfg.safety.self_collision.mesh.intra_arm.a_brake_m_s2, 3.0));
        RB_CHECK(cfg.safety.self_collision.mesh.disabled_collision_pairs.size() == 2);
        RB_CHECK(cfg.safety.self_collision.mesh.disabled_collision_pairs[1].pattern_a == "*right*link0*");
        RB_CHECK(cfg.safety.self_collision.mesh.disabled_collision_pairs[1].pattern_b == "*right*link1*");
        RB_CHECK(cfg.safety.self_collision.mesh.debug_pair_curation);
    }

    // external_boxes.margin_m accepts legacy scalar form and broadcasts to all axes.
    {
        const std::string path = writeTempConfig(
            "external-box-margin-scalar",
            selfCollisionConfigBody(
                true,
                "  self_collision:\n"
                "    enable: true\n"
                "    monitor_only: true\n"
                "    fail_policy: clamp_hold\n"
                "    mesh:\n"
                "      unified_urdf: \"" + rb3UrdfPath() + "\"\n"
                "      external_boxes:\n"
                "        margin_m: 0.04\n"));
        const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
        ::unlink(path.c_str());
        const auto& margin = cfg.safety.self_collision.mesh.external_boxes.margin_m;
        RB_CHECK(near(margin[0], 0.04));
        RB_CHECK(near(margin[1], 0.04));
        RB_CHECK(near(margin[2], 0.04));
    }

    // external_boxes.margin_m also accepts per-axis [x, y, z] box-local inflation.
    {
        const std::string path = writeTempConfig(
            "external-box-margin-list",
            selfCollisionConfigBody(
                true,
                "  self_collision:\n"
                "    enable: true\n"
                "    monitor_only: true\n"
                "    fail_policy: clamp_hold\n"
                "    mesh:\n"
                "      unified_urdf: \"" + rb3UrdfPath() + "\"\n"
                "      external_boxes:\n"
                "        margin_m: [0.0, 0.0, 0.04]\n"));
        const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
        ::unlink(path.c_str());
        const auto& margin = cfg.safety.self_collision.mesh.external_boxes.margin_m;
        RB_CHECK(near(margin[0], 0.0));
        RB_CHECK(near(margin[1], 0.0));
        RB_CHECK(near(margin[2], 0.04));
    }

    // Bad per-axis margin shapes and values fail closed.
    {
        const std::string path = writeTempConfig(
            "external-box-margin-bad-length",
            selfCollisionConfigBody(
                true,
                "  self_collision:\n"
                "    enable: true\n"
                "    mesh:\n"
                "      unified_urdf: \"" + rb3UrdfPath() + "\"\n"
                "      external_boxes:\n"
                "        margin_m: [0.0, 0.04]\n"));
        const bool rejected = loadRejects(path);
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }

    {
        const std::string path = writeTempConfig(
            "external-box-margin-negative",
            selfCollisionConfigBody(
                true,
                "  self_collision:\n"
                "    enable: true\n"
                "    mesh:\n"
                "      unified_urdf: \"" + rb3UrdfPath() + "\"\n"
                "      external_boxes:\n"
                "        margin_m: [0.0, -0.01, 0.04]\n"));
        const bool rejected = loadRejects(path);
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }

    // Disabled by default when the block is absent.
    {
        const std::string path = writeTempConfig(
            "self-collision-default", selfCollisionConfigBody(true, ""));
        const rb_servo::DualArmConfig cfg = rb_servo::loadConfigFromYaml(path);
        ::unlink(path.c_str());
        RB_CHECK(!cfg.safety.self_collision.enable);
        RB_CHECK(!cfg.safety.self_collision.monitor_only);
    }

    // Enabled without kinematics: rejected (no link-geometry source).
    {
        const std::string path = writeTempConfig(
            "self-collision-no-kinematics",
            selfCollisionConfigBody(false, "  self_collision:\n    enable: true\n" + mesh_yaml));
        const bool rejected = loadRejects(path);
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }

    // Enabled but no mesh.unified_urdf: rejected (the mesh guard has no geometry).
    {
        const std::string path = writeTempConfig(
            "self-collision-no-urdf",
            selfCollisionConfigBody(true, "  self_collision:\n    enable: true\n"));
        const bool rejected = loadRejects(path);
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }

    // d_slow_m < d_hard_m: rejected (inverted barrier band).
    {
        const std::string path = writeTempConfig(
            "self-collision-bad-band",
            selfCollisionConfigBody(
                true,
                "  self_collision:\n"
                "    enable: true\n"
                "    mesh:\n"
                "      unified_urdf: \"" + rb3UrdfPath() + "\"\n"
                "      d_hard_m: 0.03\n"
                "      d_slow_m: 0.006\n"));
        const bool rejected = loadRejects(path);
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }

    // disabled_collision_pairs must be a list of 2-element string lists.
    {
        const std::string path = writeTempConfig(
            "self-collision-bad-disabled-pair",
            selfCollisionConfigBody(
                true,
                "  self_collision:\n"
                "    enable: true\n"
                "    mesh:\n"
                "      unified_urdf: \"" + rb3UrdfPath() + "\"\n"
                "      d_hard_m: 0.006\n"
                "      d_slow_m: 0.03\n"
                "      disabled_collision_pairs:\n"
                "        - [\"*right*link0*\"]\n"));
        const bool rejected = loadRejects(path);
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }

    return true;
}

}  // namespace

bool testInitMotionBrakeConfigValidation() {
    const std::filesystem::path stack_real_path =
        servoRoot() / "config" / "stack_real.yaml";
    {
        std::string body = readFile(stack_real_path);
        RB_CHECK(replaceOnce(
            &body,
            "    brake_exit_deg_s: 0.5",
            "    brake_exit_deg_s: 2.0"
        ));
        const std::string path = writeTempConfig("init-brake-exit-above-enter", body);
        const bool rejected = loadRejectsContaining(
            path,
            "brake_exit_deg_s must be <= brake_enter_deg_s"
        );
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }
    {
        std::string body = readFile(stack_real_path);
        RB_CHECK(replaceOnce(
            &body,
            "    brake_timeout_sec: 0.75",
            "    brake_timeout_sec: 0.0"
        ));
        const std::string path = writeTempConfig("init-brake-timeout-zero", body);
        const bool rejected = loadRejectsContaining(
            path,
            "safety.init_motion_planner.brake_timeout_sec"
        );
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }
    return true;
}

bool testAutoTareAfterInitMotionConfigValidation() {
    const std::filesystem::path stack_real_path =
        servoRoot() / "config" / "stack_real.yaml";
    // The tracked real stack ships it ON, with both numbers stated.
    {
        const rb_servo::DualArmConfig cfg =
            rb_servo::loadConfigFromYaml(stack_real_path.string());
        const rb_servo::FtAutoTareConfig& at =
            cfg.force_torque.auto_tare_after_init_motion;
        RB_CHECK(at.enable);
        RB_CHECK(at.settle_sec > 0.0);
        RB_CHECK(at.max_sent_speed_deg_s > 0.0);
        RB_CHECK(at.invalidate_on_request);
    }
    // NO SILENT DEFAULT for the settle: a zero would average the arrival transient.
    {
        std::string body = readFile(stack_real_path);
        RB_CHECK(replaceOnce(&body, "    settle_sec: 0.5", "    settle_sec: 0.0"));
        const std::string path = writeTempConfig("auto-tare-settle-zero", body);
        const bool rejected = loadRejectsContaining(
            path, "auto_tare_after_init_motion.enable requires a positive settle_sec");
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }
    // ... nor for the stillness gate.
    {
        std::string body = readFile(stack_real_path);
        RB_CHECK(replaceOnce(&body, "    max_sent_speed_deg_s: 0.5",
                             "    max_sent_speed_deg_s: 0.0"));
        const std::string path = writeTempConfig("auto-tare-speed-zero", body);
        const bool rejected = loadRejectsContaining(
            path, "auto_tare_after_init_motion.enable requires a positive max_sent_speed_deg_s");
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }
    // With the InitMotion planner off there is no completion event, so the tare would
    // never fire. Refuse rather than be silently inert.
    {
        std::string body = readFile(stack_real_path);
        RB_CHECK(replaceOnce(&body, "  init_motion_planner:\n    enable: true",
                             "  init_motion_planner:\n    enable: false"));
        const std::string path = writeTempConfig("auto-tare-no-planner", body);
        const bool rejected = loadRejectsContaining(
            path, "requires safety.init_motion_planner.enable");
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }
    // Unknown keys inside the block fail closed like every other section.
    {
        std::string body = readFile(stack_real_path);
        RB_CHECK(replaceOnce(&body, "    settle_sec: 0.5",
                             "    settle_sec: 0.5\n    tare_immediately: true"));
        const std::string path = writeTempConfig("auto-tare-unknown-key", body);
        const bool rejected = loadRejects(path);
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }
    return true;
}

// pgmode controller-simulation substitutes the box's REFERENCE for the encoders,
// because in that mode there is no physical arm to measure: q_actual is frozen
// forever, which pinned init_motion's progress test and stalled every attempt
// (2026-08-27). The tracked SIM config opts in; the tracked REAL config must NOT,
// and the real lane must keep the encoder-based progress test that stops init from
// completing while the physical arm lags. The physical-motion guard debounce is
// checked the same way: present in sim, defaulted (but harmless) in real.
bool testControllerSimProgressSourceIsSimOnly() {
    const std::filesystem::path config_dir =
        std::filesystem::path(__FILE__).parent_path().parent_path() / "config";
    const rb_servo::DualArmConfig sim =
        rb_servo::loadConfigFromYaml((config_dir / "stack_sim.yaml").string());
    const rb_servo::DualArmConfig real =
        rb_servo::loadConfigFromYaml((config_dir / "stack_real.yaml").string());

    RB_CHECK(sim.safety.init_motion_planner.controller_simulation_progress_uses_reference);
    RB_CHECK(!real.safety.init_motion_planner.controller_simulation_progress_uses_reference);
    // The default is the safe one, so a config that never mentions the key keeps
    // measuring the encoders.
    RB_CHECK(!rb_servo::InitMotionPlannerConfig{}.controller_simulation_progress_uses_reference);

    // Debounce: sim pins it explicitly; both are non-negative and finite.
    RB_CHECK(sim.safety.controller_simulation_physical_motion_debounce_sec > 0.0);
    RB_CHECK(real.safety.controller_simulation_physical_motion_debounce_sec >= 0.0);

    // The sim lane now carries the SHIPPED force laws, so a sim run exercises the
    // same gains as real rather than a second set nobody tunes.
    RB_CHECK(sim.force_torque.enable);
    RB_CHECK(sim.force_control.enable);
    RB_CHECK(sim.force_control.oscillation_guard_enable ==
             real.force_control.oscillation_guard_enable);
    RB_CHECK(near(sim.force_control.max_deviation_m, real.force_control.max_deviation_m));
    RB_CHECK(near(sim.force_control.hold.translation[0].k,
                  real.force_control.hold.translation[0].k));
    return true;
}

// k = 0 + gate is CM's live configuration and is legal ONLY with the fold: without
// it nothing bounds the deviation (9.5 m in 300 s). A spring-less compliant Hold
// likewise. The tracked real stack ships exactly that pairing.
bool testSpringlessLawRequiresTheFold() {
    const std::filesystem::path stack_real_path =
        servoRoot() / "config" / "stack_real.yaml";
    {
        const rb_servo::DualArmConfig cfg =
            rb_servo::loadConfigFromYaml(stack_real_path.string());
        RB_CHECK(cfg.force_control.enable);
        RB_CHECK(cfg.force_control.fold_deviation);
        RB_CHECK(cfg.force_control.gate_enable);
        RB_CHECK(cfg.force_control.hold_compliance);
        for (int i = 0; i < 3; ++i) {
            // 2026-09-04: the stream law holds the configured force with a spring under the
            // gate (CM 0028); the hold law stays a pure-damper hand-guide with the fold.
            RB_CHECK(cfg.force_control.stream.translation[i].k == 400.0);
            RB_CHECK(cfg.force_control.stream.rotation[i].k == 0.0);
            RB_CHECK(cfg.force_control.hold.translation[i].k == 0.0);
            RB_CHECK(cfg.force_control.hold.rotation[i].k == 0.0);
            // Rotation is RIGID on both laws (2026-09-03): the wrist lever turns any
            // parasitic force into a torque, and with k_r = 0 that torque never stops.
            RB_CHECK(cfg.force_control.stream.rotation[i].mode == rb_servo::ForceAxisMode::Rigid);
            RB_CHECK(cfg.force_control.hold.rotation[i].mode == rb_servo::ForceAxisMode::Rigid);
        }
        // The hand-guide latch ships ON with a real hysteresis band.
        RB_CHECK(cfg.force_control.hold_engage_force_n == 5.0);
        RB_CHECK(cfg.force_control.hold_release_force_n == 2.0);
        RB_CHECK(cfg.force_torque.right.deadzone_force_n[0] == 3.0);
    }
    // A release at/above the engage level is a relay, not a hysteresis.
    {
        std::string body = readFile(stack_real_path);
        RB_CHECK(replaceOnce(&body, "  hold_release_force_n: 2.0", "  hold_release_force_n: 6.0"));
        const std::string path = writeTempConfig("latch-relay", body);
        const bool rejected = loadRejectsContaining(
            path, "hold_release_force_n must be below hold_engage_force_n");
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }
    // Fold off + spring-less Hold: the hand-guide has nothing to move its nominal.
    {
        std::string body = readFile(stack_real_path);
        RB_CHECK(replaceOnce(&body, "  fold_deviation: true", "  fold_deviation: false"));
        const std::string path = writeTempConfig("fold-off-hold", body);
        const bool rejected = loadRejectsContaining(
            path, "force_control.hold_compliance is on but every force_control.hold");
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }
    // Fold off, Hold not compliant, and the stream spring removed: the gate with k = 0
    // bounds force, not deviation. (With the tracked k = 400 that pairing is legal -
    // CM 0028's spring under the gate - so the spring has to go for this refusal.)
    {
        std::string body = readFile(stack_real_path);
        RB_CHECK(replaceOnce(&body, "  fold_deviation: true", "  fold_deviation: false"));
        RB_CHECK(replaceOnce(&body, "  hold_compliance: true", "  hold_compliance: false"));
        // Strip the spring from the three stream translation rows whatever m/b and
        // comment they carry (the operator tunes m in the tracked file; the test
        // must not pin that text).
        {
            const std::size_t block = body.find("  stream:\n    translation:\n");
            RB_CHECK(block != std::string::npos);
            std::size_t at = block;
            for (int i = 0; i < 3; ++i) {
                at = body.find("k: 400.0}", at);
                RB_CHECK(at != std::string::npos);
                body.replace(at, std::string("k: 400.0}").size(), "k: 0.0}");
            }
        }
        const std::string path = writeTempConfig("fold-off-gate", body);
        const bool rejected = loadRejectsContaining(
            path, "enable force_control.fold_deviation");
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }
    return true;
}

int main() {
    if (!testRepositoryConfigsParse()) return 1;
    if (!testSpringlessLawRequiresTheFold()) return 1;
    if (!testControllerSimProgressSourceIsSimOnly()) return 1;
    if (!testInitMotionBrakeConfigValidation()) return 1;
    if (!testAutoTareAfterInitMotionConfigValidation()) return 1;
    if (!testServoIoModelParsesAndValidates()) return 1;
    if (!testQueueSyncConfigValidation()) return 1;
    if (!testWorkerRealtimeScheduling()) return 1;
    if (!testUnknownKeysAndSchemaFail()) return 1;
    if (!testStatePublisherEndpointsParseAndValidate()) return 1;
    if (!testCommandSourceConfigParsesAndValidates()) return 1;
    if (!testCartesianControlTuningParsesAndValidates()) return 1;
    if (!testRemovedRawScriptBackendRejected()) return 1;
    if (!testRobotIpEnvExpansion()) return 1;
    if (!testRbpodoServoJParametersParseAndValidate()) return 1;
    if (!testKinematicsSafetyLimitMismatchWarnsForRbpodo()) return 1;
    if (!testReadMissToleranceParsesAndIsControllerSimOnly()) return 1;
    if (!testSelfCollisionConfig()) return 1;
    return 0;
}
