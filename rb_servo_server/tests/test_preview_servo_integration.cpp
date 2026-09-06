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
    q_=r.q_target_deg;
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
  ArmId arm_;JointArray q_;bool connected_{false},ready_{false},refuse_next_{false};uint64_t sequence_{0};
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
    cmd.left.arm_id=ArmId::Left;cmd.right.arm_id=ArmId::Right;
    for(int i=0;i<2;++i){auto& arm=i==0?cmd.left:cmd.right;arm.mode=mode;arm.timeout_sec=1;
      arm.has_tcp_target=mode==ControlMode::TcpPoseTarget;
      arm.tcp_target_stand=kin->computeTcpStand(i==0?ArmId::Left:ArmId::Right,q,i==0?cfg.left_mount:cfg.right_mount);}
    return cmd;
  }
  void frame(double step=.00005,bool stand_down=false){
    nlohmann::json packet={{"schema_version","robotics_lab.chunk_overlay.v3"},
      {"host_time_ns",nowSteadyNs()},{"seq",++wire},{"policy_dt_sec",.0334},{"horizon",24},
      {"chunk_metadata",{{"observation_step_seq",0},{"activation_step_seq",0},{"source_start_index",0},
        {"original_horizon",24},{"selected_horizon",24},{"proprio",{{"valid",true}}}}}};
    for(int i=0;i<2;++i){const char* side=i==0?"left":"right";
      const auto p=kin->computeTcpStand(i==0?ArmId::Left:ArmId::Right,q,i==0?cfg.left_mount:cfg.right_mount);
      const Eigen::Quaterniond rot(math::rotationFromPose(p));
      const Eigen::Vector3d delta=stand_down?rot.conjugate()*Eigen::Vector3d{0.,0.,-step}:Eigen::Vector3d{step,0.,0.};
      packet[side]=nlohmann::json::array();packet[std::string(side)+"_delta"]=nlohmann::json::array();
      for(int k=0;k<24;++k){packet[side].push_back({p.x,p.y,p.z,rot.x(),rot.y(),rot.z(),rot.w(),0.});
        packet[std::string(side)+"_delta"].push_back({delta.x(),delta.y(),delta.z(),0.,0.,0.,0.});}}
    const auto text=packet.dump();require(receiver.acceptPacket(text.data(),text.size()),"frame rejected");
  }
  void tick(DualArmCommand cmd,bool expect_fault=false){
    time+=2'000'000;setExternalSteadyNs(time);cmd.seq=++seq;cmd.host_time_ns=time;buffer.setCommand(cmd);
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

}
int main(){try{exercise(false);exercise(true);oneArmAndRejectedTopDispatch();productionGeometryFoldMetadata(false);productionGeometryFoldMetadata(true);std::cout<<"preview servo integration PASS\n";return 0;}
  catch(const std::exception& e){setExternalSteadyNs(0);std::cerr<<e.what()<<'\n';return 1;}}
