#pragma once
// Serializer-only stress fixture. These finite, full-precision values model
// populated real pose/IK/force surfaces; they are not a captured packet or a
// dynamically coherent robot state, and must never be sent as motion commands.
#include "rb_servo/core/types.hpp"

namespace state_publication_fixture {
using namespace rb_servo;
Pose6D widePose() {
  Pose6D p{0.12345678912345678, -0.23456789123456789, 0.34567891234567891,
           0.12345678912345678, -0.23456789123456789, 0.34567891234567891};
  p.quaternion_xyzw = std::array<double,4>{0.14357217502739192,-0.10424769119010212,0.183147211203082,0.9669515210660256};
  return p;
}
void activeArm(RobotState& s) {
  s.host_time_ns=1582481108339256ULL;s.robot_time_ns=1482481108339256ULL;
  s.acquisition_sequence=78115;s.q_actual_valid=true;s.q_ref_valid=true;
  s.has_valid_joint_state=true;s.has_valid_tcp_pose=true;s.tcp_actual_valid=true;
  s.tcp_ref_valid=true;s.tcp_deferred=false;s.servo_enabled=true;
  s.q_ref_source="rbpodo_jnt_ref";s.rbpodo_sdk_state_source="sdk_full";
  s.rbpodo_state_decode_policy="controller_joint_reference";
  s.tcp_base=widePose();s.tcp_stand=widePose();s.tcp_actual_base=widePose();
  s.tcp_actual_stand=widePose();s.tcp_ref_base=widePose();s.tcp_ref_stand=widePose();
  s.tcp_command_stand=widePose();
  s.q_actual_deg.fill(23.456789123456789);s.q_target_deg.fill(23.456789123456789);
  s.dq_actual_deg_s.fill(3.456789123456789);s.box_tcp_valid=true;
  s.box_tcp_pos.fill(23.456789123456789);s.box_tcp_ref.fill(23.456789123456789);
  s.box_link_parameter_count=16;s.box_link_parameter.fill(0.12345678912345678);
  s.rbpodo_diagnostics.emplace();
  s.rbpodo_diagnostics->raw.time_sec=15982.123456789123;
  s.rbpodo_diagnostics->raw.init_state_info=6;
  s.lifecycle_state="motion_ready";
  s.connection_state=RobotConnectionState::Connected;
}
void activeSolve(CartesianSolveTelemetry& t) {
  t.attempted=true;t.success=true;t.status="ok";t.tcp_target_profile="flow_infer_fresh";
  t.tcp_target_profile_found=true;t.smd_goal_stand=widePose();t.smd_ref_stand=widePose();
  t.q_ik_seed_deg.fill(23.456789123456789);t.q_ik_raw_solution_deg.fill(23.456789123456789);
  t.q_ik_solution_deg.fill(23.456789123456789);t.q_ik_raw_delta_deg.fill(0.12345678912345678);
  t.q_ik_delta_deg.fill(0.12345678912345678);t.output_ma_present=true;t.output_ma_window=3;
  t.q_target_before_output_ma_deg.fill(23.456789123456789);
  t.q_target_after_output_ma_deg.fill(23.456789123456789);
  t.safety_clamp.present=true;t.safety_clamp.q_before_safety_deg.fill(23.456789123456789);
  t.safety_clamp.q_after_joint_limit_deg.fill(23.456789123456789);
  t.safety_clamp.q_after_velocity_limit_deg.fill(23.456789123456789);
  t.safety_clamp.q_after_accel_limit_deg.fill(23.456789123456789);
  t.fk_duration_us=0.12345678912345678;
  t.ik_duration_us=0.12345678912345678;
  t.position_error_m=0.12345678912345678;
  t.orientation_error_rad=0.12345678912345678;
  t.ik_min_singular_value=0.12345678912345678;
  t.ik_applied_damping=0.12345678912345678;
  t.ik_solution_jump_deg=0.12345678912345678;
  t.ik_branch_jump_raw_deg=0.12345678912345678;
  t.ik_branch_jump_limit_deg=0.12345678912345678;
  t.ik_branch_jump_scale=0.12345678912345678;
  t.ik_joint_limit_worst_margin_deg=0.12345678912345678;
  t.ik_limit_relief_weight=0.12345678912345678;
  t.ik_limit_avoidance_step_deg=0.12345678912345678;
  t.warn_ik_duration_us=0.12345678912345678;
  t.fail_ik_duration_us=0.12345678912345678;
  t.path_s=0.12345678912345678;
  t.path_position_error_m=0.12345678912345678;
  t.path_orientation_error_rad=0.12345678912345678;
  t.path_line_deviation_m=0.12345678912345678;
  t.linear_move_duration_sec=0.12345678912345678;
  t.linear_move_elapsed_sec=0.12345678912345678;
  t.floor_lowest_z_m=0.12345678912345678;
  t.goal_minus_measured_pos_m=0.12345678912345678;
  t.goal_minus_measured_ori_rad=0.12345678912345678;
  t.smd_profile_nf_linear_hz=0.12345678912345678;
  t.smd_profile_nf_angular_hz=0.12345678912345678;
  t.smd_profile_max_linear_velocity_m_s=0.12345678912345678;
  t.smd_profile_max_linear_accel_m_s2=0.12345678912345678;
  t.smd_profile_max_angular_velocity_rad_s=0.12345678912345678;
  t.smd_profile_max_angular_accel_rad_s2=0.12345678912345678;
  t.smd_profile_max_goal_lead_m=0.12345678912345678;
  t.smd_profile_max_goal_lead_rad=0.12345678912345678;
  t.smd_goal_linear_velocity_norm_m_s=0.12345678912345678;
  t.smd_goal_angular_velocity_norm_rad_s=0.12345678912345678;
  t.smd_wall_margin_m=0.12345678912345678;
  t.smd_wall_cap_m_s=0.12345678912345678;
  t.smd_wall_clamp_m=0.12345678912345678;
  t.follower_t_in_seg_sec=0.12345678912345678;
  t.follower_duration_sec=0.12345678912345678;
  t.follower_advance_gate=0.12345678912345678;
  t.follower_plan_rate_gate=0.12345678912345678;
  t.follower_core_gate=0.12345678912345678;
  t.follower_leash_gate=0.12345678912345678;
  t.follower_alpha=0.12345678912345678;
  t.follower_output_smd_lag_m=0.12345678912345678;
  t.follower_output_smd_lag_rad=0.12345678912345678;
  t.follower_divergence_pos_m=0.12345678912345678;
  t.follower_divergence_ang_rad=0.12345678912345678;
  t.follower_projection_error_m=0.12345678912345678;
  t.follower_projection_error_rad=0.12345678912345678;
  t.follower_actual_lead_m=0.12345678912345678;
  t.follower_actual_lead_rad=0.12345678912345678;
  t.follower_cmd_track_pos_m=0.12345678912345678;
  t.follower_cmd_track_rad=0.12345678912345678;
  t.delta_twist_pending_linear_norm_m=0.12345678912345678;
  t.delta_twist_pending_angular_norm_rad=0.12345678912345678;
  t.delta_twist_step_linear_norm_m=0.12345678912345678;
  t.delta_twist_step_angular_norm_rad=0.12345678912345678;
  t.delta_twist_step_yaw_rad=0.12345678912345678;
  t.delta_twist_realized_linear_norm_m=0.12345678912345678;
  t.delta_twist_realized_angular_norm_rad=0.12345678912345678;
  t.delta_twist_realized_yaw_rad=0.12345678912345678;
  t.delta_twist_realized_linear_ratio=0.12345678912345678;
  t.delta_twist_realized_angular_ratio=0.12345678912345678;
  t.delta_twist_realized_yaw_ratio=0.12345678912345678;
  t.delta_twist_phase_sec=0.12345678912345678;
  t.delta_twist_xi_ref_linear_norm_m_s=0.12345678912345678;
  t.delta_twist_xi_ref_angular_norm_rad_s=0.12345678912345678;
  t.delta_twist_xi_cmd_linear_norm_m_s=0.12345678912345678;
  t.delta_twist_xi_cmd_angular_norm_rad_s=0.12345678912345678;
  t.delta_twist_lead_linear_norm_m=0.12345678912345678;
  t.delta_twist_lead_angular_norm_rad=0.12345678912345678;
  t.delta_twist_lin_feedback_cos=0.12345678912345678;
  t.delta_twist_ang_feedback_cos=0.12345678912345678;
}
void activeWorker(ArmWorkerTelemetry& w) {
  w.worker_command_drops_total=17;w.worker_pending_overwrites_total=158;
  w.worker_last_dropped_seq=48701;w.worker_last_enqueued_seq=48715;
  w.worker_last_dispatched_seq=48714;w.worker_last_completed_seq=48713;
  // Some worker fields are currently CSV-only. Populate them too so a future
  // JSON addition cannot accidentally regain a zero-width stress fixture.
  w.worker_repeated_sends_total=123;w.worker_wire_dispatches_total=48713;
  w.worker_last_wire_send_start_ns=1610048494348843ULL;
  w.worker_last_wire_send_end_ns=1610048494472367ULL;
  w.worker_interp_active=true;w.worker_interp_delay_setpoints=1.0123456789123456;
  w.worker_interp_rebase_total=123;w.worker_interp_hold_total=17;
}
void activeAsync(RbpodoAsyncStreamingTelemetry& t) {
  t.commands_enqueued_total=48715;t.commands_sent_total=48714;
  t.commands_acked_total=48713;t.commands_socket_sent_total=48714;
  t.commands_dropped_total=17;t.commands_overwritten_total=158;
  t.last_command_seq=48715;t.last_sent_seq=48714;t.last_ack_seq=48713;
  t.first_goal_command_send_ns=1610048394348843ULL;
  t.last_goal_command_send_ns=1610048494348843ULL;
  t.first_worker_send_ns=1610048394368191ULL;
  t.last_worker_send_ns=1610048494368191ULL;
  t.goal_window_commands_sent=48714;t.goal_window_commands_acked=48713;
  t.last_q_ref_update_host_time_ns=1610048493384193ULL;
  t.last_tcp_ref_update_host_time_ns=1610048493384193ULL;
  t.last_socket_send_host_time_ns=1610048494377123ULL;
  t.q_ref_update_age_ms=1.2345678912345678;t.tcp_ref_update_age_ms=1.2345678912345678;
  t.q_ref_target_error_deg_max=.12345678912345678;
  t.tcp_ref_target_error_m=.00012345678912345678;
  t.last_async_send_duration_us=17.123456789123456;
  t.last_async_ack_duration_us=217.12345678912345;
  t.max_async_send_duration_us=137.12345678912345;
  t.max_async_ack_duration_us=417.12345678912345;
  t.last_controller_acceptance_semantics="controller_ack_observed";
  t.last_async_acceptance_semantics="async_enqueued";
  t.command_phase="streaming";t.last_send_result="ok";t.last_ack_result="ok";
  t.worker_backlog=1;t.max_pending_age_ms_observed=3.1234567891234567;
}
void activeTransport(std::optional<BackendTransportTelemetry>& out) {
  out.emplace();auto& t=*out;
  t.connect_attempts_total=1;t.connections_opened_total=1;
  t.requests_total=48715;t.read_syscalls_total=97430;t.write_syscalls_total=48715;
}
void activeBackendCalls(BackendCallSnapshot& read,BackendCallSnapshot& send) {
  read.ok=true;read.duration_us=117.12345678912345;
  auto& e=read.read_exchange_timing;e.available=true;e.exchange_sequence=48715;
  e.source="request_data";
  e.request_data_call_start_steady_ns=1610048493208843ULL;
  e.request_data_call_return_steady_ns=1610048493325966ULL;
  e.request_data_call_start_system_ns=1788703327493208843ULL;
  e.request_data_call_return_system_ns=1788703327493325966ULL;
  e.request_data_call_duration_us=117.12345678912345;
  send.ok=true;send.accepted=true;send.duration_us=7.1234567891234567;
  send.acceptance_semantics="async_enqueued";send.state_after_source="cached_state";
}
void activeTiming(RealtimeTimingTelemetry& t) {
  t.window_sec=1.;t.servo.target_rate_hz=500.;
  t.servo.observed_rate_hz=499.99712345678912;t.servo.send_rate_hz=499.32123456789123;
  t.servo.period_ms={2.0123456789123456,2.0534512345678912,2.1765432198765432};
  t.servo.jitter_ms={.012345678912345678,.05345123456789123,.17654321987654321};
  t.servo.wake_latency_us={12.123456789123456,53.123456789123456,176.12345678912345};
  t.servo.pre_send_us={.12345678912345678,5.1234567891234567,37.123456789123456};
  t.servo.send_duration_us={7.1234567891234567,13.123456789123456,77.123456789123456};
  for(auto* f:{&t.left_feedback,&t.right_feedback}) {
    f->frame_rate_hz=499.32123456789123;f->fresh_rate_hz=249.67123456789123;
    f->held_count=250;f->period_ms=t.servo.period_ms;f->jitter_ms=t.servo.jitter_ms;
    f->age_us={1271.1234567891235,1971.1234567891235,3971.1234567891235};
    f->phase_us={271.12345678912345,971.12345678912345,1971.1234567891235};
    f->freshness_reliable=true;f->robot_time_available=true;f->robot_time_monotonic=true;
  }
}
template<class Init> void completedInit(Init& d) {
  d.status="done";d.message="goal_reached";
  d.goal_self_min_clearance_m=.034567891234567891;
  d.goal_external_min_clearance_m=.045678912345678912;
  d.goal_nearest_pair_name_a="left_wrist3_collision";
  d.goal_nearest_pair_name_b="left_forearm_collision";
  d.goal_nearest_pair_category="intra_arm";
  d.goal_nearest_pair_distance_m=.034567891234567891;
  d.goal_clear_threshold_self_m=.01;d.goal_clear_threshold_external_m=.025;
  d.goal_clear_margin_deficit_m=0.;d.waypoint_index=17;d.waypoint_count=17;
  d.dist_to_goal_deg=.012345678912345678;d.clear_threshold_m=.01;
  d.external_clear_threshold_m=.025;
  d.nearest_pair="left_wrist3_collision <-> left_forearm_collision";
  d.nearest_pair_distance_m=.034567891234567891;
}
void activeSafetySurface(ServoSnapshot& s) {
  // ServoSnapshot exposes the geometric verdict, rather than the CSV-only
  // SafetyProjectionTelemetry joint-stage arrays. Exercise those actual wires.
  s.self_collision_checked=true;s.self_collision_gripper_excluded=true;
  s.self_collision_verdict_age_ms=3.1234567891234567;
  s.self_collision_eval_ms=2.1234567891234567;s.self_collision_clamp_count=127;
  s.self_collision_self_min_clearance_m=.027123456789123456;
  s.self_collision_intra_arm_min_clearance_m=.0071234567891234567;
  s.self_collision_gripper_min_clearance_m=.031234567891234567;
  s.self_collision_has_closest_points=true;
  s.self_collision_closest_point_a_m={.12345678912345678,-.23456789123456789,.34567891234567891};
  s.self_collision_closest_point_b_m={.12745678912345678,-.23856789123456789,.34967891234567891};
  s.floor_constraint_enabled=true;s.floor_constraint_left_checked=true;
  s.floor_constraint_right_checked=true;s.floor_constraint_clamp_count=127;
  s.floor_constraint_left_tcp_z_m=.035123456789123456;
  s.floor_constraint_right_tcp_z_m=.045123456789123456;
  s.floor_constraint_left_lowest_point="gripper_finger_left";
  s.floor_constraint_right_lowest_point="gripper_finger_right";
  s.floor_constraint_left_lowest_point_m=s.self_collision_closest_point_a_m;
  s.floor_constraint_right_lowest_point_m=s.self_collision_closest_point_b_m;
  s.roi_box_enabled=true;s.roi_box_left_checked=true;s.roi_box_right_checked=true;
  s.roi_box_left_min_margin_m=.0012345678912345678;
  s.roi_box_right_min_margin_m=.0023456789123456789;
  s.roi_box_left_closest_face="min_x";s.roi_box_right_closest_face="max_y";
  s.roi_box_left_measured_checked=true;s.roi_box_right_measured_checked=true;
  s.roi_box_left_measured_min_margin_m=.0012345678912345678;
  s.roi_box_right_measured_min_margin_m=.0023456789123456789;
  s.roi_box_left_measured_closest_face="min_x";s.roi_box_right_measured_closest_face="max_y";
  for(auto* t:{&s.left_safety_tracking,&s.right_safety_tracking}) {
    t->reference_valid=true;t->command_reference_tracking_error_deg=.12345678912345678;
    t->physical_command_actual_error_deg=.23456789123456789;
    t->command_vs_actual_deg=.23456789123456789;t->reference_vs_actual_deg=.012345678912345678;
  }
}
void wideSnapshot(ServoSnapshot& s) {
  activeArm(s.left_state);activeArm(s.right_state);
  activeSolve(s.left_cartesian_solve);activeSolve(s.right_cartesian_solve);
  s.left_sent_q_deg.fill(23.456789123456789);s.right_sent_q_deg.fill(23.456789123456789);
  s.left_prev_sent_q_deg.fill(23.456789123456789);s.right_prev_sent_q_deg.fill(23.456789123456789);
  s.command.left.tcp_target_stand=widePose();s.command.right.tcp_target_stand=widePose();
  s.command.left.mode=ControlMode::TcpPoseTarget;s.command.right.mode=ControlMode::TcpPoseTarget;
  s.period_ms=2.0123456789123456;s.jitter_ms=.012345678912345678;
  s.filter_dt_ms=2.0123456789123456;
  activeWorker(s.left_worker_telemetry);activeWorker(s.right_worker_telemetry);
  activeAsync(s.left_async_streaming);activeAsync(s.right_async_streaming);
  activeTransport(s.left_transport_telemetry);activeTransport(s.right_transport_telemetry);
  activeBackendCalls(s.left_last_read,s.left_last_send);
  activeBackendCalls(s.right_last_read,s.right_last_send);
  s.left_send_ok=true;s.right_send_ok=true;
  s.left_send_start_ns=1610048494348843ULL;s.right_send_start_ns=1610048494356189ULL;
  s.left_send_end_ns=1610048494355966ULL;s.right_send_end_ns=1610048494363312ULL;
  s.left_send_duration_us=7.1234567891234567;s.right_send_duration_us=7.1234567891234567;
  s.send_skew_us=7.3461234567891234;
  s.realtime_timing.emplace();activeTiming(*s.realtime_timing);
  completedInit(s.init_motion);completedInit(s.init_motion_left);completedInit(s.init_motion_right);
  s.init_motion.start_clear_m=.034567891234567891;
  s.init_motion.goal_clear_m=.045678912345678912;
  s.init_motion.tree_start=19;s.init_motion.tree_goal=17;s.init_motion.iterations=31;
  s.init_motion.planning_time_s=.12345678912345678;
  s.init_motion_left.request_id=1788703320123456789ULL;
  s.init_motion_right.request_id=1788703320123456790ULL;
  activeSafetySurface(s);
  // Valid gripper feedback lives in GripperBridge's receive cache, not this
  // snapshot; exercise it with loopback-only feedback in the publisher test.
}
void taredForce(FtTelemetry& ft, ForceControlTelemetry& fc) {
  ft.enabled=true;ft.connected=true;ft.connect_reason="sensor_stream_varies";
  ft.bias_valid=true;ft.bias_source="tare";ft.bias_generation=1;
  ft.tare_state="accepted";ft.tare_reason="accepted";ft.tare_samples=250;
  ft.auto_tare_stage="idle";ft.auto_tare_reason="settled_after_init_motion";
  ft.axes_determinant=-1.;ft.tool_mass_kg=.8137123456789123;
  ft.tool_com_mm={12.34567891234567,-23.45678912345678,34.56789123456789};
  ft.sensor_offset_mm=ft.tool_com_mm;ft.tcp_from_sro_mm=ft.tool_com_mm;
  ft.load_force_n=1.2345678912345678;ft.load_mass_kg=.12345678912345678;ft.load_settled=true;
  ft.liveness_force_pp_n=3.381234567891234;ft.liveness_torque_pp_nm=.0531234567891234;
  Wrench6D w{8.731331234567891,-12.91341234567891,46.95791234567891,
             -.7075091234567891,.1693951234567891,-.2774161234567891};
  ft.raw_sensor=w;ft.gravity_sensor=w;ft.comp_sensor_nodz=w;
  ft.comp_sensor=w;ft.comp_tcp=w;ft.comp_stand=w;ft.bias=w;
  fc.enabled=true;fc.covered=true;fc.coverage_reason="covered";
  fc.law="hold";fc.compose_applied=true;fc.reference_strip_enabled=true;
  fc.reference_reset_count=1;fc.hold_engaged=true;fc.folded=true;fc.fold_sink="hold_nominal";
  fc.reference_deviation_m={.012345678912345678,-.023456789123456789,.034567891234567891};
  fc.reference_deviation_rad=fc.reference_deviation_m;fc.deviation_m=fc.reference_deviation_m;
  fc.deviation_rad=fc.reference_deviation_m;fc.velocity_m_s=fc.reference_deviation_m;
  fc.velocity_rad_s=fc.reference_deviation_m;fc.fold_m=fc.reference_deviation_m;
  fc.fold_rad=fc.reference_deviation_m;fc.absorbed_m=fc.reference_deviation_m;
  fc.deviation_norm_m=.045678912345678912;fc.deviation_norm_rad=.045678912345678912;
  fc.absorbed_norm_m=.045678912345678912;fc.absorbed_norm_rad=.045678912345678912;
  fc.wrench_stand=w;fc.wrench_filtered_stand=w;
  fc.wrench_filter_hz=40.;fc.fence_m=.08;fc.fence_rad=.2;
  fc.gate_translation=.91234567891234567;fc.gate_rotation=1.;
  fc.gate_force_n=3.1234567891234567;fc.gate_torque_nm=.31234567891234567;
  fc.gate_removed_m=.0012345678912345678;fc.gate_removed_rad=.00012345678912345678;
  fc.gate_stream_translation=.9876543219876543;fc.gate_stream_force_n=2.1234567891234567;
  fc.hold_force_n=5.1234567891234567;
}
} // namespace state_publication_fixture
