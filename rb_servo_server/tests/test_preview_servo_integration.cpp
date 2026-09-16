// Actual production coordinator + Pinocchio + in-memory backend. No network,
// controller, camera, gripper or policy process is started by this test.
#include "rb_servo/control/dual_arm_servo_loop.hpp"
#include "rb_servo/core/clock.hpp"
#include "rb_servo/kinematics/pinocchio_kinematics.hpp"
#include "rb_servo/math/se3.hpp"
#include <nlohmann/json.hpp>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <thread>

using namespace rb_servo;
namespace rb_servo {
// Deterministically inject a planning event, not a hardware/safety verdict.
// The production coordinator, accepted-state brake, FK, dispatch, source and
// fresh-frame handshake all execute unchanged below.
struct PreviewRecoveryTestAccess {
  static control::LivePreviewExecution* executor(DualArmServoLoop& loop,int i) {
    return loop.preview_executor_[i];
  }
  static void receiveWithinTick(DualArmServoLoop& loop,const DualArmCommand& command) {
    const auto original_start=loop.last_loop_start_ns_;
    loop.last_loop_start_ns_=original_start-1'000'000;
    loop.pollChunkFrames();
    for(const auto& profile:loop.config_.cartesian_control.tcp_pose_target_profiles)
      if(profile.name=="flow_infer_preview")loop.updatePreviewRecoveryInput(command,profile);
    loop.last_loop_start_ns_=original_start;
  }
  static void request(DualArmServoLoop& loop,PreviewRecoveryCause cause) {
    loop.preview_recovery_request_=cause;
  }
  static void waitBudget(DualArmServoLoop& loop,double seconds,int attempts) {
    for(auto& p:loop.config_.cartesian_control.tcp_pose_target_profiles)
      if(p.name=="flow_infer_preview") {
        p.ruckig_follower.preview_execution.recovery.fresh_plan_timeout_sec=seconds;
        p.ruckig_follower.preview_execution.recovery.max_attempts=attempts;
      }
  }
};
}
namespace {
void require(bool good,const std::string& why) {if(!good)throw std::runtime_error(why);}
class MemoryBackend final:public IRobotBackend {
 public:
  MemoryBackend(ArmId arm,const JointArray& q):arm_(arm),q_(q){}
  bool supportsExternalStepping()const override{return true;}
  bool isConnected()const override{return connected_;}
  ArmId armId()const override{return arm_;}
  std::string name()const override{return "preview_memory_backend";}
  BackendResult<RobotState> connect()override{connected_=true;return result(BackendOp::Connect);}
  BackendResult<RobotState> initialize()override{ready_=true;return result(BackendOp::Initialize);}
  BackendResult<RobotState> readState()override{return result(BackendOp::ReadState);}
  BackendResult<RobotState> stop()override{return result(BackendOp::Stop);}
  BackendResult<RobotState> resetFault()override{return result(BackendOp::ResetFault);}
  void refuseNextSend(){refuse_next_=true;}
  SendServoJResult sendServoJ(const SendServoJRequest& r)override{
    require(ready_,"uninitialized memory backend send");
    if(refuse_next_){refuse_next_=false;SendServoJResult out;out.accepted=false;
      out.requested_q_deg=r.q_target_deg;out.acceptance_semantics="memory_refused";
      out.timing=makeBackendTiming(nowSteadyNs(),nowSteadyNs());return out;}
    if(!frozen_)q_=r.q_target_deg;
    SendServoJResult out;out.accepted=true;out.requested_q_deg=q_;
    out.acceptance_semantics="memory_applied";out.timing=makeBackendTiming(nowSteadyNs(),nowSteadyNs());return out;
  }
 private:
  BackendResult<RobotState> result(BackendOp op){
    RobotState s;s.arm_id=arm_;s.connection_state=connected_?RobotConnectionState::Connected:RobotConnectionState::Disconnected;
    s.servo_enabled=ready_;s.q_actual_deg=s.q_target_deg=q_;
    s.q_actual_valid=s.q_ref_valid=s.has_valid_joint_state=true;s.q_ref_source="memory_applied";
    s.host_time_ns=s.robot_time_ns=nowSteadyNs();s.acquisition_sequence=++sequence_;
    BackendResult<RobotState> out;out.ok=connected_;out.op=op;out.value=s;
    out.timing=makeBackendTiming(nowSteadyNs(),nowSteadyNs());return out;
  }
  ArmId arm_;JointArray q_;bool connected_{false},ready_{false},refuse_next_{false},frozen_{false};uint64_t sequence_{0};
 public:
  // The arm stops following its commands (a stuck joint); the servo loop's tracking
  // error must latch and the delivered brake must follow.
  void freeze(){frozen_=true;}
};

DualArmConfig config(bool send_at_top,bool geometry=false){
  const auto root=std::filesystem::path(__FILE__).parent_path().parent_path();
  const auto tracked=loadConfigFromYaml((root/"config/stack_real.yaml").string());
  DualArmConfig c;
  for(auto* b:{&c.left_robot,&c.right_robot}){b->backend_type=BackendType::Mock;b->run_mode=RunMode::Mock;b->ip.clear();}
  c.servo.rate_hz=500;c.servo.io_model=ServoIoModel::Direct;c.servo.send_at_tick_start=send_at_top;
  c.servo.enable_realtime_priority=false;c.servo.cpu_core=-1;c.servo.send_servo_commands=true;
  c.servo.command_timeout_sec=1;c.logging.enable=false;c.gripper.enable=false;
  c.safety.q_min_deg=rbpodoDefaultSafetyJointMinDeg();c.safety.q_max_deg=rbpodoDefaultSafetyJointMaxDeg();
  c.safety.dq_max_deg_s.fill(170);c.safety.ddq_max_deg_s2.fill(3000);
  c.safety.max_tracking_error_deg=5;c.safety.tracking_error_policy=TrackingErrorPolicy::FaultLatch;
  c.kinematics=tracked.kinematics;c.kinematics.enable=true;c.kinematics.ik.timeout_ms=100;
  c.kinematics.ik.max_iterations=100;c.kinematics.ik.position_tolerance_m=2e-5;
  c.kinematics.ik.orientation_tolerance_rad=.0002;
  c.left_mount.arm_id=ArmId::Left;c.right_mount.arm_id=ArmId::Right;
  c.left_mount.base_pose_in_stand.x=-1;c.right_mount.base_pose_in_stand.x=1;
  c.cartesian_control.enable=true;c.cartesian_control.tcp_pose_target_profile_default="flow_infer_preview";
  for(const auto& p:tracked.cartesian_control.tcp_pose_target_profiles)
    if(p.name=="flow_infer_preview")c.cartesian_control.tcp_pose_target_profiles.push_back(p);
  if(geometry) {
    // Use the production fold thresholds and existing user-plane row law. The
    // synthetic plane is positioned from this fixture's FK before construction.
    c.safety.hold_fold=tracked.safety.hold_fold;
    c.safety.user_floor_constraint=tracked.safety.user_floor_constraint;
    c.safety.user_floor_constraint.enable=true;
    c.safety.user_floor_constraint.has_initial_plane=true;
    c.safety.user_floor_constraint.normal={0.,0.,1.};
    c.safety.user_floor_constraint.margin_m=0.;
  }
  // This fixture tests command ownership in free space; physical force/contact
  // acceptance is a separate test and is never inferred from this ideal plant.
  return c;
}
struct Fixture {
  uint64_t time{1'000'000'000},seq{0},wire{0};
  DualArmConfig cfg;CommandBuffer buffer;ChunkFrameReceiver receiver{""};
  std::shared_ptr<PinocchioKinematics> kin;std::unique_ptr<DualArmServoLoop> loop;
  MemoryBackend* left_backend{nullptr};
  JointArray q{10,-20,35,5,25,-15};ServoSnapshot snapshot;
  std::array<ServoSnapshot,8> recent{};std::size_t recent_count{0};
  explicit Fixture(bool top,bool geometry=false):cfg(config(top,geometry)){
    setExternalSteadyNs(time);kin=std::make_shared<PinocchioKinematics>(cfg.kinematics);
    if(geometry) {
      const auto initial=kin->computeTcpStand(ArmId::Left,q,cfg.left_mount);
      cfg.safety.user_floor_constraint.point_m={initial.x,initial.y,initial.z-.0002};
    }
    auto left=std::make_unique<MemoryBackend>(ArmId::Left,q);left_backend=left.get();
    loop=std::make_unique<DualArmServoLoop>(std::move(left),
      std::make_unique<MemoryBackend>(ArmId::Right,q),cfg,&buffer,nullptr,kin);
    loop->setChunkFrameReceiver(&receiver);loop->enableExternalStepping();require(loop->start(),"start failed");
    tick(command(ControlMode::ArmMotion));
  }
  ~Fixture(){loop->stop();setExternalSteadyNs(0);}
  DualArmCommand command(ControlMode mode){
    DualArmCommand cmd;cmd.tcp_target_profile="flow_infer_preview";cmd.tcp_target_profile_provided=true;
    cmd.source.source_id="preview_fixture";cmd.source.session_id="session-a";cmd.source.lease_token="token-a";
    cmd.left.arm_id=ArmId::Left;cmd.right.arm_id=ArmId::Right;
    for(int i=0;i<2;++i){auto& arm=i==0?cmd.left:cmd.right;arm.mode=mode;arm.timeout_sec=1;
      arm.has_tcp_target=mode==ControlMode::TcpPoseTarget;
      arm.tcp_target_stand=kin->computeTcpStand(i==0?ArmId::Left:ArmId::Right,q,i==0?cfg.left_mount:cfg.right_mount);}
    return cmd;
  }
  void frame(double step=.00005,bool stand_down=false,
             uint64_t recovery_epoch=UINT64_MAX,uint64_t observation=UINT64_MAX){
    nlohmann::json packet={{"schema_version","robotics_lab.chunk_overlay.v3"},
      {"host_time_ns",nowSteadyNs()},{"seq",++wire},{"policy_dt_sec",.0334},{"horizon",24},
      {"chunk_metadata",{{"observation_step_seq",0},{"activation_step_seq",0},{"source_start_index",0},
        {"original_horizon",24},{"selected_horizon",24},{"proprio",{{"valid",true}}}}}};
    packet["chunk_metadata"]["preview_recovery_epoch"]=recovery_epoch==UINT64_MAX?snapshot.preview_recovery.epoch:recovery_epoch;
    packet["chunk_metadata"]["observation_time_ns"]=observation==UINT64_MAX?time:observation;
    for(int i=0;i<2;++i){const char* side=i==0?"left":"right";
      const auto p=kin->computeTcpStand(i==0?ArmId::Left:ArmId::Right,q,i==0?cfg.left_mount:cfg.right_mount);
      const Eigen::Quaterniond rot(math::rotationFromPose(p));
      const Eigen::Vector3d delta=stand_down?rot.conjugate()*Eigen::Vector3d{0.,0.,-step}:Eigen::Vector3d{step,0.,0.};
      packet[side]=nlohmann::json::array();packet[std::string(side)+"_delta"]=nlohmann::json::array();
      for(int k=0;k<24;++k){packet[side].push_back({p.x,p.y,p.z,rot.x(),rot.y(),rot.z(),rot.w(),0.});
        packet[std::string(side)+"_delta"].push_back({delta.x(),delta.y(),delta.z(),0.,0.,0.,0.});}}
    const auto text=packet.dump();require(receiver.acceptPacket(text.data(),text.size()),"frame rejected");
  }
  void tick(DualArmCommand cmd,bool expect_fault=false,bool refresh=true){
    time+=2'000'000;setExternalSteadyNs(time);
    if(refresh){cmd.seq=++seq;cmd.host_time_ns=time;buffer.setCommand(cmd);}
    require(loop->stepOnce(),"tick failed");snapshot=loop->latestSnapshot();
    recent[recent_count++%recent.size()]=snapshot;
    if(snapshot.fault_latched&&!expect_fault){
      // Keep diagnosis tied to the actual staged/safety-passed command. This
      // fixture never relaxes IK or dispatch acceptance to hide a final clamp.
      for(std::size_t k=recent_count>recent.size()?recent_count-recent.size():0;k<recent_count;++k){
        const auto& s=recent[k%recent.size()];const auto& c=s.left_cartesian_solve;
        std::cerr<<"preview dispatch diagnostic top="<<cfg.servo.send_at_tick_start<<" tick="<<s.tick
          <<" status="<<c.preview_execution.status<<" accepted_pos="<<c.preview_execution.accepted_position_error_m
          <<" accepted_rot="<<c.preview_execution.accepted_rotation_error_rad<<" ik_pos="<<c.position_error_m
          <<" accel_clamp="<<c.safety_clamp.accel_clamp_max_delta_deg<<" ma="<<c.output_ma_window;
        if(c.stage_tcp_target_stand){const auto p=kin->computeTcpStand(ArmId::Left,s.left_sent_q_deg,cfg.left_mount);
          std::cerr<<" stage_sent_error="<<math::positionDistance(p,*c.stage_tcp_target_stand);}
        std::cerr<<" sent="<<nlohmann::json(s.left_sent_q_deg).dump()
          <<" desired="<<nlohmann::json(c.safety_clamp.q_before_safety_deg).dump()
          <<" after_accel="<<nlohmann::json(c.safety_clamp.q_after_accel_limit_deg).dump()<<'\n';
      }
    }
    if(!expect_fault)require(!snapshot.fault_latched,"unexpected fault: "+snapshot.fault_reason);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  void move(int ticks){auto cmd=command(ControlMode::TcpPoseTarget);for(int k=0;k<ticks;++k){if(k%25==0)frame();tick(cmd);}}
};
void sourceTimeoutDispatchesFiniteBrake(bool top) {
  Fixture f(top);f.move(100);
  auto* executor=PreviewRecoveryTestAccess::executor(*f.loop,0);
  require(executor && executor->telemetry().active,"timeout test did not engage preview");
  auto cmd=f.command(ControlMode::TcpPoseTarget);cmd.left.timeout_sec=cmd.right.timeout_sec=.01;
  f.tick(cmd);
  bool saw_stop=false,saw_terminal=false;
  for(int k=0;k<160;++k) {
    f.tick(cmd,false,false); // production CommandBuffer timeout, no heartbeat
    if(executor->stopping()) {
      saw_stop=true;
      require(!executor->telemetry().active,"stale source retained tracking authority");
      saw_terminal=saw_terminal||executor->stopComplete();
    }
  }
  saw_terminal=saw_terminal||executor->telemetry().source_stop_completed>0;
  require(saw_stop && saw_terminal,"timeout skipped the brake's accepted terminal sample");
  const auto held=f.snapshot.left_sent_q_deg;
  for(int k=0;k<10;++k)f.tick(cmd,false,false);
  require(f.snapshot.left_sent_q_deg==held,"timed-out source did not stay stopped");
  f.move(70);require(f.snapshot.left_cartesian_solve.preview_execution.active,"fresh source did not resume after timeout");
}

void exercise(bool top){
  Fixture f(top);f.move(100);
  const auto l=f.snapshot.left_cartesian_solve.preview_execution;
  const auto r=f.snapshot.right_cartesian_solve.preview_execution;
  require(l.active&&r.active&&l.accepted>=3&&r.accepted>=3,"first plan engagement/dispatch failed");
  require(!f.snapshot.left_cartesian_solve.follower_output_smd_active,"LPF remained active");
  auto hold=f.command(ControlMode::Hold);for(int i=0;i<15;++i)f.tick(hold);
  require(!f.snapshot.left_cartesian_solve.preview_execution.active,"hold revived preview");
  require(!f.snapshot.right_cartesian_solve.preview_execution.active,"hold revived right preview");
  require(!f.snapshot.left_cartesian_solve.smd_release_braking &&
          !f.snapshot.right_cartesian_solve.smd_release_braking,"Hold release did not stop");
  // A completed Hold must repeatedly send one stationary accepted target. Even
  // a microscopic two-cycle can keep the next preview's exact cold-start guard
  // waiting forever; do not hide it with a tolerance or a longer resume window.
  const auto& previous_hold=f.recent[(f.recent_count-2)%f.recent.size()];
  require(f.snapshot.left_prev_sent_q_deg==previous_hold.left_prev_sent_q_deg &&
          f.snapshot.right_prev_sent_q_deg==previous_hold.right_prev_sent_q_deg,
          "settled Hold alternated accepted targets");
  f.move(60);
  if(!f.snapshot.left_cartesian_solve.preview_execution.active){
    const auto& c=f.snapshot.left_cartesian_solve;const auto& p=c.preview_execution;
    std::cerr<<"resume diagnostic top="<<top<<" status="<<p.status<<" enabled="<<p.enabled
      <<" epoch="<<p.epoch<<" plan="<<p.plan_id<<" submitted="<<p.submitted
      <<" accepted="<<p.accepted<<" rejected="<<p.rejected<<" expired="<<p.expired
      <<" solve_blocked="<<c.cartesian_solve_blocked_recent<<" safety_recent="<<c.safety_intervention_recent
      <<" warm_resumes="<<c.follower_warm_resume_count<<" follower_wire="<<c.follower_wire_seq
      <<" release_braking="<<c.smd_release_braking<<'\n';
    for(std::size_t k=f.recent_count-f.recent.size();k<f.recent_count;++k){
      const auto& s=f.recent[k%f.recent.size()];
      std::cerr<<"resume history tick="<<s.tick<<" status="<<s.left_cartesian_solve.preview_execution.status
        <<" prev="<<nlohmann::json(s.left_prev_sent_q_deg).dump()
        <<" sent="<<nlohmann::json(s.left_sent_q_deg).dump()<<'\n';
    }
  }
  require(f.snapshot.left_cartesian_solve.preview_execution.active,"resume failed");
  require(f.snapshot.left_cartesian_solve.preview_execution.epoch>l.epoch,"resume retained epoch");
  auto init=f.command(ControlMode::JointTarget);
  init.left.has_joint_target=init.right.has_joint_target=true;
  init.left.q_target_deg=f.snapshot.left_sent_q_deg;init.right.q_target_deg=f.snapshot.right_sent_q_deg;
  init.left.joint_target_profile=init.right.joint_target_profile=JointTargetProfile::InitMotion;
  init.left.init_motion_request_id=init.right.init_motion_request_id=91;
  for(int i=0;i<15;++i)f.tick(init);
  require(!f.snapshot.right_cartesian_solve.preview_execution.active,"InitMotion revived preview");
  f.move(60);require(f.snapshot.right_cartesian_solve.preview_execution.active,"post-Init resume failed");
}
void oneArmAndRejectedTopDispatch(){
  Fixture f(true);auto cmd=f.command(ControlMode::TcpPoseTarget);
  cmd.right.mode=ControlMode::Hold;cmd.right.has_tcp_target=false;
  for(int k=0;k<100;++k){if(k%25==0)f.frame();f.tick(cmd);}
  require(f.snapshot.left_cartesian_solve.preview_execution.active,"one-arm top preview failed");
  require(!f.snapshot.right_cartesian_solve.preview_execution.active,"held arm gained preview authority");
  require(f.snapshot.right_sent_q_deg==f.q,"held arm moved during one-arm preview");
  const auto accepted=f.snapshot.left_prev_sent_q_deg;
  f.left_backend->refuseNextSend();f.tick(cmd,true);
  require(f.snapshot.left_prev_sent_q_deg==accepted,"rejected top dispatch advanced accepted history");
  require(f.snapshot.left_state.q_actual_deg==accepted,"refused in-memory plant moved");
  require(!f.snapshot.left_cartesian_solve.preview_execution.active,"refused dispatch retained preview authority");
}
void productionGeometryFoldMetadata(bool top){
  Fixture f(top,true);const auto cmd=f.command(ControlMode::TcpPoseTarget);
  bool booked=false,applied=false;std::uint64_t first_admitted=0,authority=0;
  PreviewExecutionTelemetry previous;
  for(int k=0;k<160;++k){
    if(k%25==0)f.frame(.0005,true);
    f.tick(cmd);const auto current=f.snapshot.left_cartesian_solve.preview_execution;
    if(previous.pending_geometry_fold_valid) {
      require(current.fold_geometry_hold_count>previous.fold_geometry_hold_count,
              "booked production geometry fold was not applied next tick");
      require(current.fold_cause==PreviewFoldCause::GeometryHold,"geometry fold routed as authority invalidation");
      require(current.fold_booked_time_ns==previous.pending_geometry_fold_time_ns,
              "applied fold lost original booking timestamp");
      require(current.fold_applied_time_ns==f.time && current.fold_booked_time_ns<current.fold_applied_time_ns,
              "fold booking/application tick ordering is wrong");
      require(current.fold_geometry_cause_mask==previous.pending_geometry_fold_cause_mask,
              "applied fold lost row participation");
      for(int axis=0;axis<3;++axis)
        require(std::abs(current.fold_translation_m[axis]-previous.pending_geometry_fold_translation_m[axis])<1e-12,
                "applied fold translation differs from booked correction");
      for(int axis=0;axis<4;++axis)
        require(std::abs(current.fold_quaternion_xyzw[axis]-previous.pending_geometry_fold_quaternion_xyzw[axis])<1e-12,
                "applied fold rotation differs from booked correction");
      applied=true;
    }
    if(current.pending_geometry_fold_valid) {
      require(current.pending_geometry_fold_time_ns==f.time,"booking is not visible on its decision tick");
      require((current.pending_geometry_fold_cause_mask&2u)!=0,"user-plane row participation missing");
      const auto& delta=current.pending_geometry_fold_translation_m;
      require(std::hypot(delta[0],delta[1],delta[2])>0,"pending fold has no geometric displacement");
      if(!booked){booked=true;first_admitted=current.accepted;authority=current.gate_revision;}
    }
    if(booked)require(current.gate_revision==authority,"geometry hold changed authority revision");
    previous=current;
    if(applied && current.fold_geometry_hold_count>=3 && current.accepted>=first_admitted+3)break;
  }
  const auto final=f.snapshot.left_cartesian_solve.preview_execution;
  require(booked&&applied&&final.fold_geometry_hold_count>=3,"production row did not exercise geometry fold");
  require(final.accepted>=first_admitted+3,"geometry folds starved future plan admission");
  require(final.expired==0&&final.gauge_transport_failed==0,"geometry transport expired or failed");
  require(final.fold_force_count==0&&final.fold_roi_floor_count==0,"fixture used another fold path");
  require(final.staged_cancel_counts[0]==0,"geometry fold cancelled staged work");
  // Interrupt an actual pending booking BEFORE its next-tick consumer. An
  // emergency bypass clears Cartesian state and must discard this old gauge.
  for(int k=0;k<80 && !f.snapshot.left_cartesian_solve.preview_execution.pending_geometry_fold_valid;++k){
    if(k%25==0)f.frame(.0005,true);f.tick(cmd);
  }
  const auto pending=f.snapshot.left_cartesian_solve.preview_execution;
  require(pending.pending_geometry_fold_valid,"no pending geometry booking for lifecycle test");
  const auto stale_booking=pending.pending_geometry_fold_time_ns;
  f.tick(f.command(ControlMode::EmergencyStop),true);
  require(f.snapshot.fault_latched&&f.snapshot.motion_state==ServerMotionState::EmergencyLatched,
          "fixture did not enter real emergency-stop bypass");
  const auto stopped=f.snapshot.left_cartesian_solve.preview_execution;
  require(!stopped.pending_geometry_fold_valid,"emergency bypass retained a pending geometry booking");
  require(stopped.fold_geometry_hold_count==pending.fold_geometry_hold_count,
          "emergency bypass applied a pending geometry booking");
  require(stopped.gauge_revision==0&&stopped.epoch>pending.epoch,"emergency bypass retained preview gauge epoch");
  f.tick(f.command(ControlMode::ResetFault));
  for(int k=0;k<15;++k)f.tick(f.command(ControlMode::Hold));
  for(int k=0;k<60;++k){
    if(k%25==0)f.frame(-.0002,true); // new-source escape from the same live plane
    f.tick(cmd);const auto& resumed=f.snapshot.left_cartesian_solve.preview_execution;
    require(resumed.fold_booked_time_ns!=stale_booking,"old geometry gauge applied after new-source resume");
  }
  const auto resumed=f.snapshot.left_cartesian_solve.preview_execution;
  require(resumed.active&&resumed.epoch>pending.epoch&&resumed.source_wire_seq>pending.source_wire_seq,
          "new-source preview did not resume after emergency reset");
}

void bimanualRecovery(bool top) {
  Fixture f(top);f.move(80);auto cmd=f.command(ControlMode::TcpPoseTarget);
  require(f.snapshot.left_cartesian_solve.preview_execution.active &&
          f.snapshot.right_cartesian_solve.preview_execution.active,"recovery fixture did not engage");
  const auto old_epoch=f.snapshot.preview_recovery.epoch;
  const auto old_source=f.snapshot.left_cartesian_solve.preview_execution.source_wire_seq;
  PreviewRecoveryTestAccess::request(*f.loop,PreviewRecoveryCause::Backlog);
  f.tick(cmd);
  require(f.snapshot.preview_recovery.state==PreviewRecoveryState::Braking,"planning event did not brake shared group");
  require(f.snapshot.preview_recovery.epoch>old_epoch,"no shared recovery epoch");
  for(const auto* p:{&f.snapshot.left_cartesian_solve.preview_execution,&f.snapshot.right_cartesian_solve.preview_execution})
    require(!p->active && std::string(p->status).find("recovery_")==0,"peer consumed ordinary plan on recovery tick");
  for(int k=0;k<200 && f.snapshot.preview_recovery.state==PreviewRecoveryState::Braking;++k)f.tick(cmd);
  require(f.snapshot.preview_recovery.state==PreviewRecoveryState::WaitingFresh,"finite accepted-state stop did not finish");
  const auto barrier=f.snapshot.preview_recovery.min_observation_time_ns;
  const auto epoch=f.snapshot.preview_recovery.epoch;
  require(barrier>0 && barrier<=f.time,"fresh observation fence absent");
  const auto held_left_target=f.snapshot.left_cartesian_solve.stage_tcp_target_stand;
  const auto held_right_target=f.snapshot.right_cartesian_solve.stage_tcp_target_stand;
  require(held_left_target && held_right_target,"recovery terminal target missing");
  const auto stationary_targets=[&] {
    for(int i=0;i<2;++i) {
      const auto& solve=i==0?f.snapshot.left_cartesian_solve:f.snapshot.right_cartesian_solve;
      const auto& held=i==0?*held_left_target:*held_right_target;
      const auto& q=i==0?f.snapshot.left_sent_q_deg:f.snapshot.right_sent_q_deg;
      const auto sent=f.kin->computeTcpStand(i==0?ArmId::Left:ArmId::Right,q,i==0?f.cfg.left_mount:f.cfg.right_mount);
      if(solve.stage_tcp_target_stand) {
        require(math::positionDistance(*solve.stage_tcp_target_stand,held)<1e-12 &&
                math::orientationDistanceRad(*solve.stage_tcp_target_stand,held)<1e-12,
                "waiting/restart advanced a stationary nominal target");
      } else {
        require(f.snapshot.preview_recovery.state==PreviewRecoveryState::Starting &&
                !solve.preview_execution.active && std::string(solve.preview_execution.status)=="waiting",
                "restart lost its stationary waiting authority");
      }
      // IK may refine a previous accepted joint solution within its declared
      // tolerance. Test the unchanged TCP authority and that same envelope,
      // rather than imposing bitwise identity on an iterative joint solver.
      require(math::positionDistance(sent,held)<=f.cfg.kinematics.ik.position_tolerance_m &&
              math::orientationDistanceRad(sent,held)<=f.cfg.kinematics.ik.orientation_tolerance_rad,
              "stationary target escaped existing IK acceptance");
    }
  };
  f.frame(.0001,false,old_epoch,f.time);f.tick(cmd);
  f.frame(.0001,false,epoch,barrier);f.tick(cmd);
  f.frame(.0001,false,epoch,f.time+1000000000);f.tick(cmd);
  require(f.snapshot.preview_recovery.state==PreviewRecoveryState::WaitingFresh &&
          f.snapshot.preview_recovery.rejected_frames>=3,"old epoch/equal fence/future frame was accepted");
  stationary_targets();
  f.frame();
  // Emulate a single candidate arriving after this tick's start but before its
  // ingest phase. It must not be permanently consumed as future-dated.
  PreviewRecoveryTestAccess::receiveWithinTick(*f.loop,cmd);
  f.tick(cmd);
  require(f.snapshot.preview_recovery.state==PreviewRecoveryState::Starting,"fresh paired frame not selected");
  const auto candidate=f.snapshot.preview_recovery.candidate_source_wire_seq;
  require(candidate>old_source,"recovery replayed abandoned source");
  for(int k=0;k<80 && f.snapshot.preview_recovery.state==PreviewRecoveryState::Starting;++k) {
    f.tick(cmd);
    // Both raw clocks are held while asynchronous first plans engage. One fast
    // worker is not permission for its arm to run the next task phase alone.
    if(f.snapshot.preview_recovery.state==PreviewRecoveryState::Starting) {
      stationary_targets();
    }
  }
  require(f.snapshot.preview_recovery.state==PreviewRecoveryState::Tracking &&
          f.snapshot.preview_recovery.completed==1,"paired recovery never resumed");
  f.move(40);require(!f.snapshot.fault_latched,"recovered policy latched");
  // An authority token renewal is not an operator retry/new session.
  const auto e=f.snapshot.preview_recovery.epoch;cmd.source.lease_token="renewed";f.tick(cmd);
  require(f.snapshot.preview_recovery.epoch==e,"lease renewal created recovery");
  cmd.source.session_id="session-b";f.tick(cmd);
  require(f.snapshot.preview_recovery.epoch>e &&
          f.snapshot.preview_recovery.state==PreviewRecoveryState::Braking,"new session crossed old plan authority");
  for(int k=0;k<200 && f.snapshot.preview_recovery.state==PreviewRecoveryState::Braking;++k)f.tick(cmd);
  PreviewRecoveryTestAccess::waitBudget(*f.loop,.052,1);
  for(int k=0;k<30;++k)f.tick(cmd);
  require(f.snapshot.preview_recovery.state==PreviewRecoveryState::Paused && !f.snapshot.fault_latched,
          "missing fresh plan did not produce local policy pause");
  const auto paused_epoch=f.snapshot.preview_recovery.epoch;
  cmd.source.lease_token="renewed-again";for(int k=0;k<5;++k)f.tick(cmd);
  require(f.snapshot.preview_recovery.epoch==paused_epoch &&
          f.snapshot.preview_recovery.state==PreviewRecoveryState::Paused,"lease renewal escaped bounded pause");
  f.tick(f.command(ControlMode::ResetFault));
  require(f.snapshot.preview_recovery.state==PreviewRecoveryState::Tracking &&
          f.snapshot.preview_recovery.epoch>paused_epoch,"explicit reset did not reset policy lifecycle");
}

void boundedRecoveryRetries(bool top) {
  Fixture f(top);f.move(80);auto cmd=f.command(ControlMode::TcpPoseTarget);
  PreviewRecoveryTestAccess::waitBudget(*f.loop,2.,2);
  PreviewRecoveryTestAccess::request(*f.loop,PreviewRecoveryCause::Backlog);f.tick(cmd);
  const auto stop=[&] {
    for(int k=0;k<200 && f.snapshot.preview_recovery.state==PreviewRecoveryState::Braking;++k)f.tick(cmd);
  };
  stop();
  for(int attempt=0;attempt<2;++attempt) {
    require(f.snapshot.preview_recovery.state==PreviewRecoveryState::WaitingFresh,"retry did not wait for fresh observation");
    f.tick(cmd); // Camera arrival must be strictly later than the stop barrier.
    f.frame();f.tick(cmd);require(f.snapshot.preview_recovery.state==PreviewRecoveryState::Starting,"retry candidate missing");
    // Inject a failed first-plan attempt before either arm can advance its raw
    // clock. Real worker starvation/expiry causes are exercised by live unit
    // tests; here the real coordinator must bound the number of transactions.
    PreviewRecoveryTestAccess::request(*f.loop,PreviewRecoveryCause::RetryTimeout);f.tick(cmd);stop();
  }
  require(f.snapshot.preview_recovery.state==PreviewRecoveryState::Paused &&
          f.snapshot.preview_recovery.attempts==2 && !f.snapshot.fault_latched,
          "planning retry exhaustion did not settle into bounded local pause");
  const auto epoch=f.snapshot.preview_recovery.epoch;
  for(int k=0;k<8;++k)f.tick(f.command(ControlMode::Hold));
  for(int k=0;k<8;++k){f.frame();f.tick(cmd);}
  require(f.snapshot.preview_recovery.state==PreviewRecoveryState::Paused &&
          f.snapshot.preview_recovery.epoch==epoch &&
          !f.snapshot.left_cartesian_solve.preview_execution.active &&
          !f.snapshot.right_cartesian_solve.preview_execution.active,
          "old policy heartbeat escaped pause after explicit Hold");
  cmd.source.session_id="operator-retry";f.tick(cmd);stop();
  require(f.snapshot.preview_recovery.state==PreviewRecoveryState::WaitingFresh &&
          f.snapshot.preview_recovery.epoch>epoch && f.snapshot.preview_recovery.attempts==1,
          "new policy session did not receive a fresh bounded recovery transaction");
}

}
// DELIVERED FAULT BRAKE (2026-09-15 night). On the 18:25 and 18:30 accepted_deviation
// latches the decelerate-then-latch ramp was computed but never sent: the latch tick
// already suppressed regular servo_j and stopped booking prev_sent, so the box drained
// its FIFO and hard-stopped from 65 / 31 deg/s with 10-12 Hz ringing. A software latch
// must keep delivering the ramp (send policy "fault_brake") until the sent velocity is
// zero on every joint, and only then go silent under "fault_latched".
void faultBrakeDelivered(bool top){
  Fixture f(top);
  auto cmd=f.command(ControlMode::TcpPoseTarget);
  for(int k=0;k<150;++k){if(k%25==0)f.frame(.003);f.tick(cmd);}   // ~90 mm/s along +x
  f.left_backend->freeze();
  int latch=-1;
  for(int k=0;k<2000&&latch<0;++k){if(k%25==0)f.frame(.003);f.tick(cmd,true);if(f.snapshot.fault_latched)latch=k;}
  require(latch>=0,"no software latch after the left arm stopped following");
  require(f.snapshot.latched_fault_reason==SafetyVerdict::TrackingError||
          f.snapshot.latched_fault_reason==SafetyVerdict::ChunkFollowerFault,
          "unexpected latch kind: "+f.snapshot.fault_reason);
  require(f.snapshot.motion_state==ServerMotionState::FaultLatched,"not a non-emergency latch");
  // Walk the delivered sends after the latch: every delivered step decelerates each
  // joint by at most ddq_max*dt, never accelerates, and the window closes silent.
  const double dt=.002,ddq=f.cfg.safety.ddq_max_deg_s2[0]*dt;
  const ServoSnapshot& before=f.recent[(f.recent_count-2)%f.recent.size()];
  JointArray prev=before.left_sent_q_deg,last=f.snapshot.left_sent_q_deg;
  std::array<double,kDof> v{};for(int i=0;i<kDof;++i)v[i]=(last[i]-prev[i])/dt;
  double peak=0;for(double x:v)peak=std::max(peak,std::abs(x));
  require(peak>1.,"fixture did not latch while moving (peak sent velocity "+std::to_string(peak)+" deg/s)");
  int brake_ticks=0,silent_at=-1;bool saw_brake=false;
  for(int k=0;k<200&&silent_at<0;++k){
    f.tick(cmd,true,false);
    if(f.snapshot.send_policy=="fault_brake"){
      require(!f.snapshot.send_suppressed,"fault_brake must deliver");saw_brake=true;++brake_ticks;
      const JointArray q=f.snapshot.left_sent_q_deg;
      for(int i=0;i<kDof;++i){
        const double v_next=(q[i]-last[i])/dt;
        require(std::abs(v_next)<=std::abs(v[i])+1e-6,"brake accelerated joint "+std::to_string(i));
        require(std::abs(v_next-v[i])<=ddq+1e-6,"brake step exceeded ddq_max on joint "+std::to_string(i));
        v[i]=v_next;
      }
      last=q;
    } else {
      require(f.snapshot.send_policy=="fault_latched"&&f.snapshot.send_suppressed,
              "unexpected policy after latch: "+f.snapshot.send_policy);
      silent_at=k;
    }
  }
  require(saw_brake,"the brake was never delivered");
  require(silent_at>=0,"the brake window never closed");
  double residual=0;for(double x:v)residual=std::max(residual,std::abs(x));
  require(residual<1e-6,"went silent before the sent velocity reached zero ("+std::to_string(residual)+" deg/s)");
  require(brake_ticks<=static_cast<int>((2.*f.cfg.safety.dq_max_deg_s[0]/f.cfg.safety.ddq_max_deg_s2[0]+.02)/dt)+2,
          "brake window outlived its derived deadline");
  // Silence is sticky: no later tick re-opens the wire.
  for(int k=0;k<20;++k){f.tick(cmd,true,false);require(f.snapshot.send_suppressed,"silence must be sticky");}
  std::cout<<"fault brake top="<<top<<" delivered over "<<brake_ticks<<" ticks from "<<peak<<" deg/s\n";
}

int main(){try{faultBrakeDelivered(false);faultBrakeDelivered(true);sourceTimeoutDispatchesFiniteBrake(false);sourceTimeoutDispatchesFiniteBrake(true);exercise(false);exercise(true);oneArmAndRejectedTopDispatch();productionGeometryFoldMetadata(false);productionGeometryFoldMetadata(true);bimanualRecovery(false);bimanualRecovery(true);boundedRecoveryRetries(false);boundedRecoveryRetries(true);std::cout<<"preview servo integration PASS\n";return 0;}
  catch(const std::exception& e){setExternalSteadyNs(0);std::cerr<<e.what()<<'\n';return 1;}}
