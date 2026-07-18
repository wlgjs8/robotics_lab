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
        RB_CHECK(stack_real.servo.send_servo_commands);
        RB_CHECK(stack_real.servo.allow_real_motion_with_suspect_diagnostics);
        RB_CHECK(!stack_real.servo.allow_controller_simulation_motion);
        RB_CHECK(near(stack_real.left_robot.servo_t1_sec, 0.002));
        RB_CHECK(near(stack_real.right_robot.servo_t1_sec, 0.002));
        RB_CHECK(near(stack_real.left_robot.servo_t2_sec, 0.021));
        RB_CHECK(near(stack_real.right_robot.servo_t2_sec, 0.021));
        RB_CHECK(stack_real.left_robot.disable_waiting_ack);
        RB_CHECK(stack_real.right_robot.disable_waiting_ack);
        RB_CHECK(near(stack_real.safety.q_min_deg[0], -360.0));
        // J3 (elbow) is clamped near the RB3-730E physical range, not +/-360.
        // Site-raised from the +/-150 catalog value to +/-160 (2026-07 stack
        // config safety-margin adjustment).
        RB_CHECK(near(stack_real.safety.q_min_deg[2], -160.0));
        RB_CHECK(near(stack_real.safety.q_max_deg[2], 160.0));
        RB_CHECK(stack_real.network.command_bind == "udp://127.0.0.1:50256");
        RB_CHECK(stack_real.network.state_pub_endpoint == "udp://127.0.0.1:50356");
        RB_CHECK(stack_real.network.state_pub_endpoints.size() == 5);
        RB_CHECK(stack_real.command_source.enforce_lease);
        RB_CHECK(stack_real.network.command_source_enforce_lease);
        RB_CHECK(near(stack_real.command_source.lease_timeout_sec, 60.0));
        RB_CHECK(stack_real.cartesian_control.enable);
        RB_CHECK(stack_real.cartesian_control.allow_in_real);
        RB_CHECK(!stack_real.cartesian_control.allow_in_controller_simulation);
        RB_CHECK(stack_real.cartesian_control.tcp_pose_target_profile_default == "umi_large_smooth");
        RB_CHECK(stack_real.cartesian_control.tcp_pose_target_profiles.size() == 3);
        bool has_spacemouse = false;
        bool has_umi = false;
        bool has_flow = false;
        const rb_servo::TcpPoseTargetProfileConfig* flow_profile = nullptr;
        for (const auto& profile : stack_real.cartesian_control.tcp_pose_target_profiles) {
            has_spacemouse = has_spacemouse || profile.name == "spacemouse_precise";
            has_umi = has_umi || profile.name == "umi_large_smooth";
            has_flow = has_flow || profile.name == "flow_infer_smooth";
            if (profile.name == "flow_infer_smooth") {
                flow_profile = &profile;
            }
        }
        RB_CHECK(has_spacemouse);
        RB_CHECK(has_umi);
        RB_CHECK(has_flow);
        RB_CHECK(flow_profile != nullptr);
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
        RB_CHECK(flow_profile->ruckig_follower.consume_steps == 12);
        RB_CHECK(flow_profile->ruckig_follower.reserve_steps == 2);
        RB_CHECK(near(flow_profile->ruckig_follower.hold_bounce_resume_sec, 0.5));
        // 2026-07-18 SPEED_SCALE=1.0 posture: projection fidelity warns (the
        // model's chunks are followed), lead budgets scaled to the full-speed
        // honest lag (35 mm / 4 deg).
        RB_CHECK(flow_profile->ruckig_follower.preview_projection_fault_policy ==
                 rb_servo::RuckigProjectionFaultPolicy::Warn);
        RB_CHECK(near(flow_profile->ruckig_follower.preview_max_actual_lead_m, 0.035));
        RB_CHECK(near(flow_profile->ruckig_follower.preview_max_actual_lead_rad,
                      0.06981317008));
        RB_CHECK(stack_real.force_torque.source == "rbpodo_eft");
        RB_CHECK(stack_real.force_torque.left.enable);
        RB_CHECK(stack_real.force_torque.right.enable);
        RB_CHECK(stack_real.force_torque.left.frame_configured);
        RB_CHECK(stack_real.force_torque.right.frame_configured);
        RB_CHECK(!stack_real.force_torque.left.inertial_compensation_enable);
        RB_CHECK(!stack_real.force_torque.right.inertial_compensation_enable);
        RB_CHECK(near(stack_real.force_torque.left.inertial_effective_mass_kg, 0.0));
        RB_CHECK(near(stack_real.force_torque.right.inertial_effective_mass_kg, 0.0));
        RB_CHECK(near(stack_real.force_torque.left.inertial_accel_lpf_alpha, 0.0));
        RB_CHECK(near(stack_real.force_torque.right.inertial_accel_lpf_alpha, 0.0));
        const auto& payload_id = stack_real.force_torque.payload_identification;
        RB_CHECK(payload_id.enable);
        RB_CHECK(payload_id.observation_model == "controller_compensated_linear");
        RB_CHECK(payload_id.wrench_convention.empty());
        RB_CHECK(payload_id.min_poses == 5);
        RB_CHECK(near(payload_id.arrival_tolerance_deg, 1.5));
        RB_CHECK(near(payload_id.settle_sec, 0.5));
        RB_CHECK(payload_id.samples_per_pose == 500);
        RB_CHECK(near(payload_id.max_force_stddev_n, 0.75));
        RB_CHECK(near(payload_id.max_torque_stddev_nm, 0.15));
        const auto& left_ft = stack_real.force_torque.left;
        const auto& right_ft = stack_real.force_torque.right;
        // 2026-07-17: left arm-specific gravity map captured and applied
        // (left-20260717T104253Z-7e5aaba4); the interim rigid_payload
        // zero-mass posture is retired for both arms.
        RB_CHECK(left_ft.gravity_compensation_model ==
                 "controller_compensated_linear");
        RB_CHECK(left_ft.gravity_compensation_calibration_id ==
                 "left-20260717T104253Z-7e5aaba4");
        RB_CHECK(left_ft.gravity_force_matrix_configured);
        RB_CHECK(left_ft.gravity_torque_matrix_configured);
        RB_CHECK(right_ft.gravity_compensation_model ==
                 "controller_compensated_linear");
        RB_CHECK(right_ft.gravity_compensation_calibration_id ==
                 "right-20260716T060343Z-421f0157");
        RB_CHECK(right_ft.gravity_force_matrix_configured);
        RB_CHECK(right_ft.gravity_torque_matrix_configured);
        const std::array<double, 9> expected_force_matrix{
            -0.7603111049987973,
            1.0013958357019646,
            -0.01518380064332614,
            -0.9254727708956185,
            -0.7791934817445652,
            -0.020435340561226173,
            -0.23394181627988272,
            -0.006012858617312489,
            0.25715992798221454,
        };
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
        for (std::size_t i = 0; i < expected_force_matrix.size(); ++i) {
            RB_CHECK(near(right_ft.gravity_force_matrix_n_per_m_s2[i],
                          expected_force_matrix[i]));
            RB_CHECK(near(right_ft.gravity_torque_matrix_nm_per_m_s2[i],
                          expected_torque_matrix[i]));
        }
        RB_CHECK(near(right_ft.payload_mass_kg, 0.0));
        RB_CHECK(near(right_ft.payload_com_tcp_m[0], 0.0));
        RB_CHECK(near(right_ft.payload_com_tcp_m[1], 0.0));
        RB_CHECK(near(right_ft.payload_com_tcp_m[2], 0.0));
        RB_CHECK(left_ft.calibration_id ==
                 "right-derived-identical-positive-force-ft-axis-20260712");
        RB_CHECK(right_ft.calibration_id ==
                 "physical-positive-force-ft-axis-20260712");
        RB_CHECK(near(left_ft.t_tcp_sensor.x, 0.0));
        RB_CHECK(near(left_ft.t_tcp_sensor.y, 0.0));
        RB_CHECK(near(left_ft.t_tcp_sensor.z, -0.202642));
        RB_CHECK(near(left_ft.t_tcp_sensor.rx, 0.0));
        RB_CHECK(near(left_ft.t_tcp_sensor.ry, 0.0));
        RB_CHECK(near(left_ft.t_tcp_sensor.rz, 1.5707963267948966));
        RB_CHECK(near(right_ft.t_tcp_sensor.x, left_ft.t_tcp_sensor.x));
        RB_CHECK(near(right_ft.t_tcp_sensor.y, left_ft.t_tcp_sensor.y));
        RB_CHECK(near(right_ft.t_tcp_sensor.z, left_ft.t_tcp_sensor.z));
        RB_CHECK(near(right_ft.t_tcp_sensor.rx, left_ft.t_tcp_sensor.rx));
        RB_CHECK(near(right_ft.t_tcp_sensor.ry, left_ft.t_tcp_sensor.ry));
        RB_CHECK(near(right_ft.t_tcp_sensor.rz, left_ft.t_tcp_sensor.rz));
        RB_CHECK(stack_real.force_control.provider == "project_native");
        RB_CHECK(stack_real.force_control.enable);
        // 2026-07-17 operator decision: supervised dual-arm cartesian_admittance
        // (translation-only axes) with the freedrive-calibrated 30/35 N hard
        // limits; the fail-closed monitor posture is retired.
        RB_CHECK(stack_real.force_control.operating_mode == "cartesian_admittance");
        RB_CHECK(stack_real.force_control.allow_in_real);
        RB_CHECK(stack_real.force_control.supervised_experimental_real);
        RB_CHECK(stack_real.force_control.left.enable);
        RB_CHECK(stack_real.force_control.right.enable);
        RB_CHECK(!stack_real.safety.floor_constraint.enable);
        RB_CHECK(!stack_real.safety.user_floor_constraint.enable);
        const auto& left_force = stack_real.force_control.left;
        const auto& right_force = stack_real.force_control.right;
        // 2026-07-18: supervised first activation of guarded contact after the
        // 15:09 full-speed 32 N pick floor spike (see stack_real.yaml note).
        RB_CHECK(left_force.surface_source == "contact_force");
        RB_CHECK(left_force.surface_source == right_force.surface_source);
        RB_CHECK(near(left_force.target_force_n, right_force.target_force_n));
        RB_CHECK(near(left_force.contact_enter_force_n, right_force.contact_enter_force_n));
        RB_CHECK(near(left_force.contact_release_force_n, right_force.contact_release_force_n));
        RB_CHECK(near(left_force.target_force_n, 2.0));
        RB_CHECK(near(left_force.contact_release_force_n, 2.75));
        RB_CHECK(near(left_force.contact_enter_force_n, 3.5));
        RB_CHECK(
            left_force.target_force_n + left_force.force_deadband_n <=
            left_force.contact_release_force_n
        );
        RB_CHECK(near(left_force.force_deadband_n, right_force.force_deadband_n));
        RB_CHECK(near(left_force.hard_normal_force_n, 30.0));
        RB_CHECK(near(left_force.hard_normal_force_n, right_force.hard_normal_force_n));
        // Translation-only compliance: rotations stay off until the closed-jaw
        // fingertip-centre check passes with a raised torque deadband.
        RB_CHECK(left_force.compliance_axes.x);
        RB_CHECK(left_force.compliance_axes.y);
        RB_CHECK(left_force.compliance_axes.z);
        RB_CHECK(!left_force.compliance_axes.roll);
        RB_CHECK(!left_force.compliance_axes.pitch);
        RB_CHECK(!left_force.compliance_axes.yaw);
        RB_CHECK(near(left_force.transverse_contact_enter_force_n, 2.5));
        RB_CHECK(near(left_force.transverse_contact_release_force_n, 1.5));
        RB_CHECK(near(left_force.torque_contact_enter_nm, 0.45));
        RB_CHECK(near(left_force.torque_contact_release_nm, 0.30));
        RB_CHECK(near(
            left_force.transverse_contact_enter_force_n,
            right_force.transverse_contact_enter_force_n
        ));
        RB_CHECK(near(
            left_force.transverse_contact_release_force_n,
            right_force.transverse_contact_release_force_n
        ));
        RB_CHECK(near(
            left_force.torque_contact_enter_nm,
            right_force.torque_contact_enter_nm
        ));
        RB_CHECK(near(
            left_force.torque_contact_release_nm,
            right_force.torque_contact_release_nm
        ));
        RB_CHECK(right_force.compliance_axes.x && right_force.compliance_axes.y);
        RB_CHECK(right_force.compliance_axes.z);
        RB_CHECK(!right_force.compliance_axes.roll);
        RB_CHECK(!right_force.compliance_axes.pitch);
        RB_CHECK(!right_force.compliance_axes.yaw);
        RB_CHECK(left_force.compliance_frame == "tcp_origin");
        RB_CHECK(left_force.compliance_frame == right_force.compliance_frame);
        RB_CHECK(near(left_force.hard_force_norm_n, 35.0));
        RB_CHECK(near(left_force.hard_force_norm_n, right_force.hard_force_norm_n));
        RB_CHECK(near(left_force.hard_torque_norm_nm, 7.0));
        RB_CHECK(near(left_force.hard_torque_norm_nm, right_force.hard_torque_norm_nm));
        RB_CHECK(left_force.debounce_samples == right_force.debounce_samples);
        // 3 fresh samples (~11 ms at the measured ~280 Hz EFT freshness): the
        // 36 N freedrive floor impact held >30 N for only 12 ms, so 5 samples
        // (~18 ms) would miss that impact class.
        RB_CHECK(left_force.hard_limit_debounce_samples == 3);
        RB_CHECK(
            left_force.hard_limit_debounce_samples ==
            right_force.hard_limit_debounce_samples
        );
        RB_CHECK(near(left_force.release_dwell_sec, right_force.release_dwell_sec));
        RB_CHECK(near(left_force.release_velocity_threshold_m_s, 0.002));
        RB_CHECK(near(
            left_force.release_velocity_threshold_m_s,
            right_force.release_velocity_threshold_m_s
        ));
        const auto& normal_force = stack_real.force_control.normal_admittance;
        RB_CHECK(near(normal_force.virtual_mass_kg, 8.0));
        RB_CHECK(near(stack_real.force_control.virtual_mass[0], 2.0));
        RB_CHECK(near(stack_real.force_control.virtual_mass[3], 0.2));
        RB_CHECK(near(stack_real.force_control.damping[0], 26.0));
        RB_CHECK(near(stack_real.force_control.damping[3], 1.55));
        RB_CHECK(near(stack_real.force_control.damping[4], 1.55));
        RB_CHECK(near(stack_real.force_control.damping[5], 1.55));
        // Translation stiffness 0: pure mass-damper so the compliance never
        // spring-recenters back into a contact (the 19:21 left-arm 372 N
        // spike); rotations keep their spring (axes disabled anyway).
        RB_CHECK(near(stack_real.force_control.stiffness[2], 0.0));
        RB_CHECK(near(stack_real.force_control.stiffness[3], 3.0));
        RB_CHECK(near(stack_real.force_control.stiffness[4], 3.0));
        RB_CHECK(near(stack_real.force_control.stiffness[5], 3.0));
        RB_CHECK(near(stack_real.force_control.wrench_deadband[0], 1.5));
        RB_CHECK(near(stack_real.force_control.wrench_deadband[2], 1.5));
        RB_CHECK(near(stack_real.force_control.wrench_deadband[3], 0.10));
        RB_CHECK(near(stack_real.force_control.wrench_deadband[4], 0.10));
        RB_CHECK(near(stack_real.force_control.wrench_deadband[5], 0.10));
        RB_CHECK(stack_real.force_control.blockwise_release_recenter);
        // Offset cap sits under the follower's 20 mm actual-lead budget; the
        // response caps are the 2026-07-17 contact-tracking raise.
        RB_CHECK(near(stack_real.force_control.max_pos_offset_m, 0.012));
        RB_CHECK(near(stack_real.force_control.max_linear_velocity_m_s, 0.06));
        RB_CHECK(near(stack_real.force_control.max_linear_jerk_m_s3, 8.0));
        RB_CHECK(near(normal_force.damping_n_s_m, 160.0));
        RB_CHECK(near(normal_force.stiffness_n_m, 0.0));
        RB_CHECK(near(normal_force.max_unload_offset_m, 0.01));
        RB_CHECK(near(normal_force.max_normal_velocity_m_s, 0.015));
        RB_CHECK(near(normal_force.max_normal_acceleration_m_s2, 0.12));
        RB_CHECK(near(normal_force.max_normal_jerk_m_s3, 0.8));
        RB_CHECK(stack_real.kinematics.enable);
        RB_CHECK(stack_real.kinematics.provider == "pinocchio");
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
        // Site-raised from the +/-150 catalog value to +/-160 (2026-07 stack
        // config safety-margin adjustment).
        RB_CHECK(near(stack_sim.safety.q_min_deg[2], -160.0));
        RB_CHECK(near(stack_sim.safety.q_max_deg[2], 160.0));
        RB_CHECK(stack_sim.safety.controller_simulation_tracking_error_source ==
                 rb_servo::ControllerSimulationTrackingErrorSource::Reference);
        RB_CHECK(stack_sim.safety.controller_simulation_tracking_error_nonlatching);
        RB_CHECK(stack_sim.safety.controller_simulation_physical_motion_policy ==
                 rb_servo::ControllerSimulationPhysicalMotionPolicy::FaultLatch);
        RB_CHECK(stack_sim.network.command_bind == "udp://127.0.0.1:50256");
        RB_CHECK(stack_sim.network.state_pub_endpoint == "udp://127.0.0.1:50356");
        RB_CHECK(stack_sim.network.state_pub_endpoints.size() == 5);
        RB_CHECK(stack_sim.network.state_pub_endpoints[1] == "udp://127.0.0.1:50366");
        RB_CHECK(stack_sim.network.state_pub_endpoints[2] == "udp://127.0.0.1:50376");
        RB_CHECK(stack_sim.command_source.enforce_lease);
        RB_CHECK(stack_sim.network.command_source_enforce_lease);
        RB_CHECK(!stack_sim.force_torque.payload_identification.enable);
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
        RB_CHECK(stack_sim.cartesian_control.tcp_pose_target_profiles.size() == 3);
        const rb_servo::TcpPoseTargetProfileConfig* sim_flow_profile = nullptr;
        for (const auto& profile : stack_sim.cartesian_control.tcp_pose_target_profiles) {
            if (profile.name == "flow_infer_smooth") sim_flow_profile = &profile;
        }
        RB_CHECK(sim_flow_profile != nullptr);
        RB_CHECK(sim_flow_profile->ruckig_follower.controller ==
                 rb_servo::RuckigFollowerController::DeltaPreview);
        RB_CHECK(sim_flow_profile->ruckig_follower.consume_steps == 12);
        RB_CHECK(sim_flow_profile->ruckig_follower.reserve_steps == 2);
        RB_CHECK(near(sim_flow_profile->ruckig_follower.hold_bounce_resume_sec, 0.5));
        RB_CHECK(near(sim_flow_profile->ruckig_follower.preview_max_actual_lead_m, 0.006));
        RB_CHECK(near(sim_flow_profile->pose_track_smd.max_linear_velocity_m_s, 0.50));
        RB_CHECK(stack_sim.force_control.provider == "null");
        RB_CHECK(!stack_sim.force_control.enable);
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

bool testGuardedAdmittanceReleaseProfileValidation() {
    const std::filesystem::path stack_real_path =
        servoRoot() / "config" / "stack_real.yaml";

    {
        std::string body = readFile(stack_real_path);
        // The tracked real profile already runs cartesian_admittance
        // (2026-07-17), so per-arm admittance validation is active as-is.
        RB_CHECK(replaceOnce(
            &body,
            "    contact_release_force_n: 2.75\n",
            "    contact_release_force_n: 2.0\n"
        ));
        const std::string path = writeTempConfig("force-release-at-target", body);
        const bool rejected = loadRejectsContaining(
            path,
            "target_force_n < contact_release_force_n"
        );
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }

    {
        std::string body = readFile(stack_real_path);
        // The tracked real profile already runs cartesian_admittance
        // (2026-07-17), so per-arm admittance validation is active as-is.
        RB_CHECK(replaceOnce(
            &body,
            "    force_deadband_n: 0.5\n",
            "    force_deadband_n: 1.1\n"
        ));
        const std::string path = writeTempConfig("force-release-inside-deadband", body);
        const bool rejected = loadRejectsContaining(
            path,
            "target_force_n + force_deadband_n <= contact_release_force_n"
        );
        ::unlink(path.c_str());
        RB_CHECK(rejected);
    }

    {
        std::string body = readFile(stack_real_path);
        // The tracked real profile already runs cartesian_admittance
        // (2026-07-17), so per-arm admittance validation is active as-is.
        RB_CHECK(replaceOnce(
            &body,
            "    contact_release_force_n: 2.75\n",
            "    contact_release_force_n: 3.5\n"
        ));
        const std::string path = writeTempConfig("force-release-at-entry", body);
        const bool rejected = loadRejectsContaining(
            path,
            "contact_release_force_n < contact_enter_force_n"
        );
        ::unlink(path.c_str());
        RB_CHECK(rejected);
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
    const bool real_worker_rejected = loadRejects(real_worker_path);
    ::unlink(real_worker_path.c_str());
    RB_CHECK(real_worker_rejected);
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

bool testForceControlSchemaAndActivation() {
    const std::string inactive_path = writeTempConfig(
        "force-inactive-schema",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "force_torque:\n"
        "  source: rbpodo_eft\n"
        "  left:\n"
        "    enable: false\n"
        "    frame_configured: true\n"
        "    sensor_identity: left-ft-profile\n"
        "    calibration_id: left-ft-cal-v1\n"
        "    freshness_source: sequence\n"
        "    max_sample_age_sec: 0.01\n"
        "    max_source_stall_sec: 0.02\n"
        "    control_lpf_alpha: 0.3\n"
        "    max_tcp_speed_m_s: 0.05\n"
        "    max_tcp_accel_m_s2: 0.5\n"
        "    auto_tare_after_init_motion: true\n"
        "    auto_tare_settle_sec: 0.6\n"
        "    residual_tare_min_samples: 20\n"
        "    residual_tare_max_force_stddev_n: 0.2\n"
        "    residual_tare_max_torque_stddev_nm: 0.02\n"
        "    T_tcp_sensor: [0.0, 0.0, 0.03, 0.0, 0.0, 0.0]\n"
        "    sensor_bias: [1.0, 2.0, 3.0, 0.1, 0.2, 0.3]\n"
        "    payload_mass_kg: 0.5\n"
        "    payload_com_tcp_m: [0.0, 0.0, 0.05]\n"
        "    residual_tare_tcp: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]\n"
        "  right:\n"
        "    enable: false\n"
        "    frame_configured: true\n"
        "    sensor_identity: right-ft-profile\n"
        "    calibration_id: right-ft-cal-v2\n"
        "    freshness_source: source_time\n"
        "    T_tcp_sensor: [0.0, 0.0, 0.04, 0.0, 0.0, 0.0]\n"
        "    payload_mass_kg: 0.7\n"
        "force_control:\n"
        "  provider: null\n"
        "  enable: false\n"
        "  allow_in_real: false\n"
        "  update_rate_hz: 500\n"
        "  virtual_mass: [4.0, 4.0, 5.0, 0.4, 0.4, 0.5]\n"
        "  damping: [60.0, 60.0, 80.0, 6.0, 6.0, 8.0]\n"
        "  stiffness: [0.0, 0.0, 20.0, 0.0, 0.0, 2.0]\n"
        "  max_dt_sec: 0.01\n"
        "  max_pos_offset_m: 0.005\n"
        "  max_rot_offset_rad: 0.05\n"
        "  max_linear_velocity_m_s: 0.01\n"
        "  max_angular_velocity_rad_s: 0.1\n"
        "  max_linear_acceleration_m_s2: 0.1\n"
        "  max_angular_acceleration_rad_s2: 1.0\n"
        "  max_linear_jerk_m_s3: 1.0\n"
        "  max_angular_jerk_rad_s3: 10.0\n"
        "  max_pos_step_m: 0.0005\n"
        "  max_rot_step_rad: 0.005\n"
        "  max_energy_j: 1.0\n"
    );
    const rb_servo::DualArmConfig inactive = rb_servo::loadConfigFromYaml(inactive_path);
    ::unlink(inactive_path.c_str());
    RB_CHECK(!inactive.force_control.enable);
    RB_CHECK(inactive.force_control.update_rate_hz == 500);
    RB_CHECK(near(inactive.force_control.virtual_mass[2], 5.0));
    RB_CHECK(near(inactive.force_control.max_linear_velocity_m_s, 0.01));
    RB_CHECK(inactive.force_torque.source == "rbpodo_eft");
    RB_CHECK(!inactive.force_torque.left.enable);
    RB_CHECK(inactive.force_torque.left.frame_configured);
    RB_CHECK(inactive.force_torque.left.auto_tare_after_init_motion);
    RB_CHECK(near(inactive.force_torque.left.auto_tare_settle_sec, 0.6));
    RB_CHECK(near(inactive.force_torque.left.t_tcp_sensor.z, 0.03));
    RB_CHECK(near(inactive.force_torque.left.payload_mass_kg, 0.5));
    RB_CHECK(inactive.force_torque.right.freshness_source == "source_time");
    RB_CHECK(near(inactive.force_torque.right.t_tcp_sensor.z, 0.04));
    RB_CHECK(near(inactive.force_torque.right.payload_mass_kg, 0.7));

    const std::string enabled_path = writeTempConfig(
        "force-enabled",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "force_control:\n"
        "  provider: null\n"
        "  enable: true\n"
    );
    const bool enabled_rejected = loadRejects(enabled_path);
    ::unlink(enabled_path.c_str());
    RB_CHECK(enabled_rejected);

    const std::string provider_path = writeTempConfig(
        "force-provider",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "force_control:\n"
        "  provider: admittance\n"
        "  enable: false\n"
    );
    const bool provider_rejected = loadRejects(provider_path);
    ::unlink(provider_path.c_str());
    RB_CHECK(provider_rejected);

    const std::string pipeline_enabled_path = writeTempConfig(
        "force-pipeline-enabled",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "force_torque:\n"
        "  left:\n"
        "    enable: true\n"
    );
    const bool pipeline_enabled_rejected = loadRejects(pipeline_enabled_path);
    ::unlink(pipeline_enabled_path.c_str());
    RB_CHECK(pipeline_enabled_rejected);

    const std::string auto_tare_without_planner_path = writeTempConfig(
        "force-auto-tare-without-planner",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "force_torque:\n"
        "  source: rbpodo_eft\n"
        "  left:\n"
        "    enable: true\n"
        "    frame_configured: true\n"
        "    sensor_identity: rft64-left\n"
        "    calibration_id: characterization-v1\n"
        "    auto_tare_after_init_motion: true\n"
    );
    const bool auto_tare_without_planner_rejected =
        loadRejects(auto_tare_without_planner_path);
    ::unlink(auto_tare_without_planner_path.c_str());
    RB_CHECK(auto_tare_without_planner_rejected);

    const std::string monitor_enabled_path = writeTempConfig(
        "force-monitor-enabled",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "force_torque:\n"
        "  source: rbpodo_eft\n"
        "  left:\n"
        "    enable: true\n"
        "    frame_configured: true\n"
        "    sensor_identity: rft64-left\n"
        "    calibration_id: characterization-v1\n"
        "    freshness_source: source_time\n"
        "force_control:\n"
        "  provider: project_native\n"
        "  enable: true\n"
        "  operating_mode: monitor\n"
        "  left:\n"
        "    enable: true\n"
        "    surface_source: floor_constraint\n"
        "    compliance_frame: sensor_origin\n"
        "    target_force_n: 3.0\n"
        "    contact_enter_force_n: 5.0\n"
        "    contact_release_force_n: 1.0\n"
        "    force_deadband_n: 0.5\n"
        "    hard_normal_force_n: 20.0\n"
        "    hard_force_norm_n: 30.0\n"
        "    hard_torque_norm_nm: 5.0\n"
        "    debounce_samples: 3\n"
        "    hard_limit_debounce_samples: 4\n"
        "    release_dwell_sec: 0.1\n"
        "    release_velocity_threshold_m_s: 0.003\n"
        "  normal_admittance:\n"
        "    virtual_mass_kg: 4.0\n"
        "    damping_n_s_m: 60.0\n"
        "    stiffness_n_m: 0.0\n"
        "    max_unload_offset_m: 0.005\n"
        "    max_normal_velocity_m_s: 0.01\n"
        "    max_normal_acceleration_m_s2: 0.1\n"
        "    max_normal_jerk_m_s3: 1.0\n"
        "    max_normal_step_m: 0.0005\n"
        "    max_energy_j: 1.0\n"
    );
    const rb_servo::DualArmConfig monitor_enabled =
        rb_servo::loadConfigFromYaml(monitor_enabled_path);
    ::unlink(monitor_enabled_path.c_str());
    RB_CHECK(monitor_enabled.force_control.enable);
    RB_CHECK(monitor_enabled.force_control.provider == "project_native");
    RB_CHECK(monitor_enabled.force_control.operating_mode == "monitor");
    RB_CHECK(monitor_enabled.force_control.update_rate_hz == 500);
    RB_CHECK(monitor_enabled.force_control.left.enable);
    RB_CHECK(monitor_enabled.force_control.left.compliance_frame == "sensor_origin");
    RB_CHECK(near(monitor_enabled.force_control.left.target_force_n, 3.0));
    RB_CHECK(monitor_enabled.force_control.left.hard_limit_debounce_samples == 4);
    RB_CHECK(near(
        monitor_enabled.force_control.left.release_velocity_threshold_m_s,
        0.003
    ));
    RB_CHECK(near(
        monitor_enabled.force_control.normal_admittance.max_unload_offset_m,
        0.005
    ));

    const std::string rate_mismatch_path = writeTempConfig(
        "force-rate-mismatch",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "force_control:\n"
        "  provider: project_native\n"
        "  enable: true\n"
        "  operating_mode: monitor\n"
        "  update_rate_hz: 200\n"
    );
    const bool rate_mismatch_rejected = loadRejects(rate_mismatch_path);
    ::unlink(rate_mismatch_path.c_str());
    RB_CHECK(rate_mismatch_rejected);

    const std::string invalid_release_velocity_path = writeTempConfig(
        "force-invalid-release-velocity",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "force_control:\n"
        "  provider: null\n"
        "  enable: false\n"
        "  left:\n"
        "    release_velocity_threshold_m_s: 0.0\n"
    );
    const bool invalid_release_velocity_rejected =
        loadRejects(invalid_release_velocity_path);
    ::unlink(invalid_release_velocity_path.c_str());
    RB_CHECK(invalid_release_velocity_rejected);

    const std::string invalid_compliance_frame_path = writeTempConfig(
        "force-invalid-compliance-frame",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "force_control:\n"
        "  provider: null\n"
        "  enable: false\n"
        "  left:\n"
        "    compliance_frame: tool_magic\n"
    );
    const bool invalid_compliance_frame_rejected =
        loadRejects(invalid_compliance_frame_path);
    ::unlink(invalid_compliance_frame_path.c_str());
    RB_CHECK(invalid_compliance_frame_rejected);

    const std::string unconfigured_sensor_frame_path = writeTempConfig(
        "force-unconfigured-sensor-frame",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "force_torque:\n"
        "  source: rbpodo_eft\n"
        "  left:\n"
        "    enable: true\n"
        "force_control:\n"
        "  provider: project_native\n"
        "  enable: true\n"
        "  operating_mode: monitor\n"
        "  left:\n"
        "    enable: true\n"
        "    compliance_frame: sensor_origin\n"
    );
    const bool unconfigured_sensor_frame_rejected =
        loadRejects(unconfigured_sensor_frame_path);
    ::unlink(unconfigured_sensor_frame_path.c_str());
    RB_CHECK(unconfigured_sensor_frame_rejected);

    const std::string invalid_mass_path = writeTempConfig(
        "force-invalid-mass",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "force_control:\n"
        "  provider: null\n"
        "  enable: false\n"
        "  virtual_mass: [1.0, 1.0, 0.0, 1.0, 1.0, 1.0]\n"
    );
    const bool invalid_mass_rejected = loadRejects(invalid_mass_path);
    ::unlink(invalid_mass_path.c_str());
    RB_CHECK(invalid_mass_rejected);

    const std::string anisotropic_coupled_recenter_path = writeTempConfig(
        "force-anisotropic-coupled-recenter",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "force_control:\n"
        "  provider: null\n"
        "  enable: false\n"
        "  blockwise_release_recenter: true\n"
        "  virtual_mass: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]\n"
        "  damping: [1.0, 2.0, 1.0, 1.0, 1.0, 1.0]\n"
    );
    const bool anisotropic_coupled_recenter_rejected =
        loadRejects(anisotropic_coupled_recenter_path);
    ::unlink(anisotropic_coupled_recenter_path.c_str());
    RB_CHECK(anisotropic_coupled_recenter_rejected);

    const std::string invalid_alpha_path = writeTempConfig(
        "force-invalid-alpha",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "force_torque:\n"
        "  left:\n"
        "    control_lpf_alpha: 1.1\n"
    );
    const bool invalid_alpha_rejected = loadRejects(invalid_alpha_path);
    ::unlink(invalid_alpha_path.c_str());
    RB_CHECK(invalid_alpha_rejected);

    const std::string invalid_inertial_mass_path = writeTempConfig(
        "force-invalid-inertial-mass",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "force_torque:\n"
        "  left:\n"
        "    inertial_compensation_enable: true\n"
        "    inertial_effective_mass_kg: 0.0\n"
        "    inertial_accel_lpf_alpha: 0.5\n"
    );
    const bool invalid_inertial_mass_rejected =
        loadRejects(invalid_inertial_mass_path);
    ::unlink(invalid_inertial_mass_path.c_str());
    RB_CHECK(invalid_inertial_mass_rejected);

    const std::string missing_inertial_alpha_path = writeTempConfig(
        "force-missing-inertial-alpha",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "force_torque:\n"
        "  left:\n"
        "    inertial_compensation_enable: true\n"
        "    inertial_effective_mass_kg: 2.0\n"
    );
    const bool missing_inertial_alpha_rejected =
        loadRejects(missing_inertial_alpha_path);
    ::unlink(missing_inertial_alpha_path.c_str());
    RB_CHECK(missing_inertial_alpha_rejected);

    const std::string invalid_freshness_path = writeTempConfig(
        "force-invalid-freshness",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "force_torque:\n"
        "  left:\n"
        "    freshness_source: host_time\n"
    );
    const bool invalid_freshness_rejected = loadRejects(invalid_freshness_path);
    ::unlink(invalid_freshness_path.c_str());
    RB_CHECK(invalid_freshness_rejected);

    const std::string missing_identity_path = writeTempConfig(
        "force-missing-identity",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "force_torque:\n"
        "  left:\n"
        "    frame_configured: true\n"
        "    T_tcp_sensor: [0, 0, 0, 0, 0, 0]\n"
    );
    const bool missing_identity_rejected = loadRejects(missing_identity_path);
    ::unlink(missing_identity_path.c_str());
    RB_CHECK(missing_identity_rejected);
    return true;
}

bool testPayloadIdentificationConfigIsExplicitAndFailClosed() {
    const std::string valid_path = writeTempConfig(
        "payload-identification-valid",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "force_torque:\n"
        "  source: rbpodo_eft\n"
        "  payload_identification:\n"
        "    enable: true\n"
        "    observation_model: rigid_payload\n"
        "    wrench_convention: sensor_reaction\n"
        "    min_poses: 5\n"
        "    arrival_tolerance_deg: 1.5\n"
        "    settle_sec: 0.5\n"
        "    samples_per_pose: 500\n"
        "    max_force_stddev_n: 0.75\n"
        "    max_torque_stddev_nm: 0.15\n"
        "    max_force_fit_rms_n: 0.75\n"
        "    max_torque_fit_rms_nm: 0.15\n"
        "    max_design_condition_number: 1000.0\n"
        "  left:\n"
        "    enable: true\n"
    );
    const rb_servo::DualArmConfig valid = rb_servo::loadConfigFromYaml(valid_path);
    ::unlink(valid_path.c_str());
    const auto& profile = valid.force_torque.payload_identification;
    RB_CHECK(profile.enable);
    RB_CHECK(profile.observation_model == "rigid_payload");
    RB_CHECK(profile.wrench_convention == "sensor_reaction");
    RB_CHECK(profile.min_poses == 5);
    RB_CHECK(near(profile.arrival_tolerance_deg, 1.5));
    RB_CHECK(near(profile.settle_sec, 0.5));
    RB_CHECK(profile.samples_per_pose == 500);
    RB_CHECK(near(profile.max_force_stddev_n, 0.75));
    RB_CHECK(near(profile.max_torque_stddev_nm, 0.15));
    RB_CHECK(near(profile.max_force_fit_rms_n, 0.75));
    RB_CHECK(near(profile.max_torque_fit_rms_nm, 0.15));
    RB_CHECK(near(profile.max_design_condition_number, 1000.0));

    const std::string incomplete_path = writeTempConfig(
        "payload-identification-incomplete",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "force_torque:\n"
        "  source: rbpodo_eft\n"
        "  payload_identification:\n"
        "    enable: true\n"
        "  left:\n"
        "    enable: true\n"
    );
    const bool incomplete_rejected = loadRejects(incomplete_path);
    ::unlink(incomplete_path.c_str());
    RB_CHECK(incomplete_rejected);

    const std::string invalid_convention_path = writeTempConfig(
        "payload-identification-invalid-convention",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "force_torque:\n"
        "  source: rbpodo_eft\n"
        "  payload_identification:\n"
        "    enable: true\n"
        "    observation_model: rigid_payload\n"
        "    wrench_convention: guessed\n"
        "    min_poses: 5\n"
        "    arrival_tolerance_deg: 1.5\n"
        "    settle_sec: 0.5\n"
        "    samples_per_pose: 500\n"
        "    max_force_stddev_n: 0.75\n"
        "    max_torque_stddev_nm: 0.15\n"
        "    max_force_fit_rms_n: 0.75\n"
        "    max_torque_fit_rms_nm: 0.15\n"
        "    max_design_condition_number: 1000.0\n"
        "  left:\n"
        "    enable: true\n"
    );
    const bool invalid_convention_rejected = loadRejects(invalid_convention_path);
    ::unlink(invalid_convention_path.c_str());
    RB_CHECK(invalid_convention_rejected);

    const std::string no_sensor_path = writeTempConfig(
        "payload-identification-no-sensor",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "force_torque:\n"
        "  source: rbpodo_eft\n"
        "  payload_identification:\n"
        "    enable: true\n"
        "    observation_model: rigid_payload\n"
        "    wrench_convention: sensor_reaction\n"
        "    min_poses: 5\n"
        "    arrival_tolerance_deg: 1.5\n"
        "    settle_sec: 0.5\n"
        "    samples_per_pose: 500\n"
        "    max_force_stddev_n: 0.75\n"
        "    max_torque_stddev_nm: 0.15\n"
        "    max_force_fit_rms_n: 0.75\n"
        "    max_torque_fit_rms_nm: 0.15\n"
        "    max_design_condition_number: 1000.0\n"
    );
    const bool no_sensor_rejected = loadRejects(no_sensor_path);
    ::unlink(no_sensor_path.c_str());
    RB_CHECK(no_sensor_rejected);

    const std::string invalid_condition_path = writeTempConfig(
        "payload-identification-invalid-condition",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "force_torque:\n"
        "  source: rbpodo_eft\n"
        "  payload_identification:\n"
        "    enable: true\n"
        "    observation_model: rigid_payload\n"
        "    wrench_convention: sensor_reaction\n"
        "    min_poses: 5\n"
        "    arrival_tolerance_deg: 1.5\n"
        "    settle_sec: 0.5\n"
        "    samples_per_pose: 500\n"
        "    max_force_stddev_n: 0.75\n"
        "    max_torque_stddev_nm: 0.15\n"
        "    max_force_fit_rms_n: 0.75\n"
        "    max_torque_fit_rms_nm: 0.15\n"
        "    max_design_condition_number: 1.0\n"
        "  left:\n"
        "    enable: true\n"
    );
    const bool invalid_condition_rejected = loadRejects(invalid_condition_path);
    ::unlink(invalid_condition_path.c_str());
    RB_CHECK(invalid_condition_rejected);

    const std::string linear_profile_path = writeTempConfig(
        "payload-identification-linear",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "force_torque:\n"
        "  source: rbpodo_eft\n"
        "  payload_identification:\n"
        "    enable: true\n"
        "    observation_model: controller_compensated_linear\n"
        "    min_poses: 5\n"
        "    arrival_tolerance_deg: 1.5\n"
        "    settle_sec: 0.5\n"
        "    samples_per_pose: 500\n"
        "    max_force_stddev_n: 0.75\n"
        "    max_torque_stddev_nm: 0.15\n"
        "    max_force_fit_rms_n: 0.75\n"
        "    max_torque_fit_rms_nm: 0.15\n"
        "    max_design_condition_number: 1000.0\n"
        "  left:\n"
        "    enable: true\n"
    );
    const rb_servo::DualArmConfig linear_profile =
        rb_servo::loadConfigFromYaml(linear_profile_path);
    ::unlink(linear_profile_path.c_str());
    RB_CHECK(
        linear_profile.force_torque.payload_identification.observation_model ==
        "controller_compensated_linear"
    );
    RB_CHECK(linear_profile.force_torque.payload_identification.wrench_convention.empty());

    const std::string linear_runtime_path = writeTempConfig(
        "gravity-compensation-linear",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "force_torque:\n"
        "  right:\n"
        "    gravity_compensation_model: controller_compensated_linear\n"
        "    gravity_compensation_calibration_id: right-linear-test\n"
        "    gravity_force_matrix_n_per_m_s2: [1, 2, 3, 4, 5, 6, 7, 8, 9]\n"
        "    gravity_torque_matrix_nm_per_m_s2: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]\n"
        "    payload_mass_kg: 0.0\n"
        "    payload_com_tcp_m: [0, 0, 0]\n"
    );
    const rb_servo::DualArmConfig linear_runtime =
        rb_servo::loadConfigFromYaml(linear_runtime_path);
    ::unlink(linear_runtime_path.c_str());
    RB_CHECK(
        linear_runtime.force_torque.right.gravity_compensation_model ==
        "controller_compensated_linear"
    );
    RB_CHECK(
        near(linear_runtime.force_torque.right.gravity_force_matrix_n_per_m_s2[5], 6.0)
    );

    const std::string incomplete_runtime_path = writeTempConfig(
        "gravity-compensation-linear-incomplete",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "force_torque:\n"
        "  right:\n"
        "    gravity_compensation_model: controller_compensated_linear\n"
        "    gravity_compensation_calibration_id: right-linear-test\n"
        "    gravity_force_matrix_n_per_m_s2: [1, 2, 3, 4, 5, 6, 7, 8, 9]\n"
    );
    const bool incomplete_runtime_rejected = loadRejects(incomplete_runtime_path);
    ::unlink(incomplete_runtime_path.c_str());
    RB_CHECK(incomplete_runtime_rejected);

    const std::string combined_runtime_path = writeTempConfig(
        "gravity-compensation-linear-combined",
        "schema: robotics_lab.rb_servo_server.v1\n"
        "force_torque:\n"
        "  right:\n"
        "    gravity_compensation_model: controller_compensated_linear\n"
        "    gravity_compensation_calibration_id: right-linear-test\n"
        "    gravity_force_matrix_n_per_m_s2: [1, 2, 3, 4, 5, 6, 7, 8, 9]\n"
        "    gravity_torque_matrix_nm_per_m_s2: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]\n"
        "    payload_mass_kg: 0.5\n"
    );
    const bool combined_runtime_rejected = loadRejects(combined_runtime_path);
    ::unlink(combined_runtime_path.c_str());
    RB_CHECK(combined_runtime_rejected);
    return true;
}

// The floorless posture: motion-affecting cartesian_admittance must load with
// BOTH the stand floor_constraint and the user_floor_constraint disabled, via
// surface_source: none. This is the User/Stand-floor-free run path (bolt-pick
// compliance without the geometric floor velocity damper). guard /
// guarded_admittance still bind an enforcing floor, and floor_constraint /
// user_floor_plane surfaces still require their plane (no fail-closed regression).
bool testFloorlessForceControlSurfaceSourceNone() {
    // Shared valid cartesian_admittance body; only surface_source / mode vary.
    const auto forceControlBody = [](const std::string& surface_source,
                                     const std::string& operating_mode) {
        return std::string(
            "schema: robotics_lab.rb_servo_server.v1\n"
            "servo:\n"
            "  send_at_tick_start: false\n"
            "kinematics:\n"
            "  enable: true\n"
            "  provider: pinocchio\n"
            "  urdf: \"" + rb3UrdfPath() + "\"\n"
            "safety:\n"
            "  floor_constraint:\n"
            "    enable: false\n"
            "  user_floor_constraint:\n"
            "    enable: false\n"
            "force_torque:\n"
            "  source: rbpodo_eft\n"
            "  left:\n"
            "    enable: true\n"
            "    frame_configured: true\n"
            "    sensor_identity: rft64-left\n"
            "    calibration_id: characterization-v1\n"
            "    freshness_source: sequence\n"
            "force_control:\n"
            "  provider: project_native\n"
            "  enable: true\n"
            "  operating_mode: ") + operating_mode + "\n"
            "  update_rate_hz: 500\n"
            "  left:\n"
            "    enable: true\n"
            "    surface_source: " + surface_source + "\n"
            "    compliance_frame: tcp_origin\n"
            "    target_force_n: 2.0\n"
            "    contact_enter_force_n: 3.5\n"
            "    contact_release_force_n: 2.75\n"
            "    force_deadband_n: 0.5\n"
            "    hard_normal_force_n: 15.0\n"
            "    hard_force_norm_n: 20.0\n"
            "    hard_torque_norm_nm: 3.0\n"
            "    debounce_samples: 3\n"
            "    hard_limit_debounce_samples: 5\n"
            "    release_dwell_sec: 0.1\n"
            "    release_velocity_threshold_m_s: 0.002\n"
            "    compliance_axes: [false, true, true, true, false, false]\n"
            "  right:\n"
            "    enable: false\n";
    };

    // 1) Floorless acceptance: cartesian_admittance + surface_source: none loads
    //    with both floors disabled.
    const std::string floorless_path = writeTempConfig(
        "force-floorless-none", forceControlBody("none", "cartesian_admittance"));
    const rb_servo::DualArmConfig floorless =
        rb_servo::loadConfigFromYaml(floorless_path);
    ::unlink(floorless_path.c_str());
    RB_CHECK(floorless.force_control.enable);
    RB_CHECK(floorless.force_control.operating_mode == "cartesian_admittance");
    RB_CHECK(floorless.force_control.left.surface_source == "none");
    RB_CHECK(!floorless.safety.floor_constraint.enable);
    RB_CHECK(!floorless.safety.user_floor_constraint.enable);

    // contact_force derives and freezes its frame from the debounced measured
    // contact, so it requires an F/T frame but no geometric floor.
    const std::string contact_force_path = writeTempConfig(
        "force-floorless-contact-force",
        forceControlBody("contact_force", "cartesian_admittance"));
    const rb_servo::DualArmConfig contact_force =
        rb_servo::loadConfigFromYaml(contact_force_path);
    ::unlink(contact_force_path.c_str());
    RB_CHECK(contact_force.force_control.left.surface_source == "contact_force");
    RB_CHECK(contact_force.force_torque.left.frame_configured);
    RB_CHECK(!contact_force.safety.floor_constraint.enable);
    RB_CHECK(!contact_force.safety.user_floor_constraint.enable);

    // 2) Regression: surface_source: floor_constraint with the floor disabled
    //    still fails closed (motion-affecting force control needs its plane).
    const std::string missing_floor_path = writeTempConfig(
        "force-floorless-floor-required",
        forceControlBody("floor_constraint", "cartesian_admittance"));
    const bool missing_floor_rejected = loadRejectsContaining(
        missing_floor_path, "requires an enforcing safety.floor_constraint");
    ::unlink(missing_floor_path.c_str());
    RB_CHECK(missing_floor_rejected);

    // 3) Restriction: surface_source: none is rejected for a surface-normal
    //    unload mode (guarded_admittance) — it has no geometric contact surface.
    const std::string none_guarded_path = writeTempConfig(
        "force-floorless-none-guarded",
        forceControlBody("none", "guarded_admittance"));
    const bool none_guarded_rejected = loadRejectsContaining(
        none_guarded_path,
        "surface_source=none requires operating_mode monitor or cartesian_admittance");
    ::unlink(none_guarded_path.c_str());
    RB_CHECK(none_guarded_rejected);

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
    RB_CHECK(warnings.str().find("differs from rb3_730e URDF IK limit") != std::string::npos);
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

int main() {
    if (!testRepositoryConfigsParse()) return 1;
    if (!testGuardedAdmittanceReleaseProfileValidation()) return 1;
    if (!testServoIoModelParsesAndValidates()) return 1;
    if (!testUnknownKeysAndSchemaFail()) return 1;
    if (!testStatePublisherEndpointsParseAndValidate()) return 1;
    if (!testForceControlSchemaAndActivation()) return 1;
    if (!testPayloadIdentificationConfigIsExplicitAndFailClosed()) return 1;
    if (!testFloorlessForceControlSurfaceSourceNone()) return 1;
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
