// Hardware-free delta-preview transition tests. No sockets, backend or model calls.
#include "rb_servo/control/cartesian_chunk_follower.hpp"
#include "rb_servo/control/follower_output_smd.hpp"
#include "rb_servo/math/se3.hpp"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <vector>

using namespace rb_servo;
using namespace rb_servo::control;
namespace {
constexpr double dt = .002, policy_dt = .0334;
void require(bool ok, const char* why) { if (!ok) throw std::runtime_error(why); }
double norm(const Vec6& v) { return std::sqrt(v.x*v.x+v.y*v.y+v.z*v.z+v.rx*v.rx+v.ry*v.ry+v.rz*v.rz); }
double difference(const Vec6& a, const Vec6& b) {
  return norm({a.x-b.x,a.y-b.y,a.z-b.z,a.rx-b.rx,a.ry-b.ry,a.rz-b.rz});
}
CartesianChunkFollowerConfig config(bool fresh) {
  CartesianChunkFollowerConfig c;
  c.lin={.45,12.,2000.}; c.ang={1.4,40.,4000.};
  c.window={0,8,4,1}; c.core_time_stretch_enable=true;c.core_time_stretch_max_ratio=4.;
  c.guard.af_damping_beta_lin=1.;c.guard.af_damping_beta_ang=1.;
  c.guard.corner_deadband_lin_m=.0003;c.guard.corner_deadband_ang_rad=.0005;
  c.guard.corner_velocity_scale=.25;
  c.fresh_chunk_replan=fresh;c.continuous_hold_resume=fresh;
  return c;
}
Pose6D origin() { return math::poseFromSe3(pinocchio::SE3(
    Eigen::Matrix3d::Identity(),Eigen::Vector3d(.3,.1,.2))); }
ChunkFrame frame(std::uint64_t seq, bool turn=false) {
  ChunkFrame c; c.policy_dt=policy_dt;c.wire_seq=seq;c.recv_seq=seq;
  for(int i=0;i<8;++i) {
    c.delta.push_back(turn ? Vec6{-.004,.003,0,.004,-.009,.006}
                          : Vec6{.003,.0004,0,.006,.004,.002});
    c.grip.push_back(30.);
  }
  return c;
}
void synthetic() {
  // A coordinate change can expose an already-existing current-state limit
  // excess. Ruckig must brake from that state, not hide it with a sample jump.
  for(const std::array<double,2> initial :
      {std::array<double,2>{.5,40.6},std::array<double,2>{1.42,0.}}) {
    const std::array<AxisLimit,1> limits{AxisLimit{1.4,40.,4000.}};
    ChunkFollowerSegment<1> core(limits,policy_dt,{},true);
    core.seed({0.},{initial[0]},{initial[1]});
    BoundarySample<1> goal;goal.pf={.12};
    const auto solved=core.solve(goal,4*policy_dt);
    require(solved.result==ruckig::Result::Working,"Ruckig current-state brake failed");
    std::array<double,1> p{},v{},a{},previous_a{};
    core.sample(0.,p,v,a);
    require(std::abs(v[0]-initial[0])<1e-12 && std::abs(a[0]-initial[1])<1e-12,
            "current-state brake clipped its initial derivatives");
    previous_a=a;
    const double h=1e-5;
    for(double t=h;t<solved.duration;t+=h) {
      core.sample(t,p,v,a);
      require(std::abs(a[0]-previous_a[0])<=4000.*h+1e-7,
              "current-state brake exceeded the configured jerk");
      previous_a=a;
    }
    core.sample(solved.duration,p,v,a);
    require(std::abs(v[0])<1e-8 && std::abs(a[0])<1e-8,
            "current-state brake did not reach the bounded stationary target");
    core.sample(.002,p,v,a);
    require(std::abs(a[0])<=40.+1e-8,"initial acceleration excess did not brake inside limit");
  }
  for(int phase=1;phase<17;++phase) {
    auto cfg=config(true);CartesianChunkFollower f(cfg);
    f.submitDeltaFrame(frame(1),origin());
    for(int i=0;i<phase;++i) f.tick(dt);
    const auto before=f.outputKinematics();
    f.submitDeltaFrame(frame(2,true),f.lastPose());
    const auto after=f.outputKinematics();
    require(math::positionDistance(before.pose,after.pose)<1e-12,"fresh swap changed sample position");
    require(math::orientationDistanceRad(before.pose,after.pose)<1e-12,"fresh swap changed orientation");
    require(difference(before.velocity,after.velocity)<1e-11,"fresh swap changed physical velocity");
    require(difference(before.acceleration,after.acceleration)<1e-9,"fresh swap changed physical acceleration");
    const auto emitted=f.tick(dt);
    require(f.diag().seg_wire_seq==2,"fresh delta not consumed on next tick");
    require(math::positionDistance(before.pose,emitted)<=std::sqrt(3.)*cfg.lin.v_max*dt+1e-9,
            "fresh swap exceeded linear speed bound");
    require(f.diag().last_solve.result==ruckig::Result::Working,"fresh solve failed");
  }
  // A disabled opt-in preserves endpoint-at-boundary consumption.
  CartesianChunkFollower old(config(false));old.submitDeltaFrame(frame(1),origin());
  old.tick(dt);old.tick(dt);old.submitDeltaFrame(frame(2,true),old.lastPose());old.tick(dt);
  require(old.diag().seg_wire_seq==1,"disabled fresh mode changed legacy segment consumption");

  // Test derivative continuity at an orientation tangent change as well as at
  // fresh-frame receipt. Near-limit changing-axis rotations can expose a
  // current-state clamp which a single-axis trajectory cannot.
  CartesianChunkFollower rotating(config(true));
  auto curved=frame(1);
  for(std::size_t i=0;i<curved.delta.size();++i) {
    curved.delta[i]={0,0,0,1.39*policy_dt,
        1.3*std::cos(.6*i)*policy_dt,1.3*std::sin(.6*i)*policy_dt};
  }
  rotating.submitDeltaFrame(curved,origin());rotating.tick(dt);
  double max_boundary_dv=0,max_boundary_da=0;
  for(int i=0;i<7;++i) {
    const double remaining=rotating.segmentLengthSec()-rotating.tInSegment();
    rotating.tick(std::max(0.,remaining-1e-8));const auto a=rotating.outputKinematics();
    rotating.tick(2e-8);const auto b=rotating.outputKinematics();
    max_boundary_dv=std::max(max_boundary_dv,difference(a.velocity,b.velocity));
    max_boundary_da=std::max(max_boundary_da,difference(a.acceleration,b.acceleration));
  }
  std::cout<<"near-limit tangent boundary derivative changes "<<max_boundary_dv
           <<" velocity, "<<max_boundary_da<<" acceleration\n";
  require(max_boundary_dv<1e-4,"rotation tangent change clipped physical velocity");
  require(max_boundary_da<1e-2,"rotation tangent change clipped physical acceleration");
  double max_preempt_solve_dv=0,max_preempt_solve_da=0;
  for(int phase=1;phase<=100;++phase) {
    CartesianChunkFollower candidate(config(true));
    candidate.submitDeltaFrame(curved,origin());
    for(int i=0;i<phase;++i) candidate.tick(dt);
    const auto a=candidate.outputKinematics();
    auto next=curved;next.wire_seq=next.recv_seq=2;
    candidate.submitDeltaFrame(next,a.pose);
    candidate.tick(1e-8);const auto b=candidate.outputKinematics();
    max_preempt_solve_dv=std::max(max_preempt_solve_dv,difference(a.velocity,b.velocity));
    max_preempt_solve_da=std::max(max_preempt_solve_da,difference(a.acceleration,b.acceleration));
    require(candidate.diag().last_solve.result==ruckig::Result::Working,
            "near-limit preemption solve failed");
  }
  std::cout<<"near-limit preemption solve changes "<<max_preempt_solve_dv
           <<" velocity, "<<max_preempt_solve_da<<" acceleration\n";
  require(max_preempt_solve_dv<1e-5,"fresh next solve clipped physical velocity");
  require(max_preempt_solve_da<1e-3,"fresh next solve clipped physical acceleration");

  // A fresh frame can arrive before EVERY segment boundary when the plan clock
  // is gated. The orientation tangent must still roll forward: otherwise the
  // principal logarithm of a future knot wraps at pi and reverses the plan.
  CartesianChunkFollower preempted_rotation(config(true));
  preempted_rotation.setPlanRateGate(.25);
  auto spin=frame(1);
  for(auto& d:spin.delta) d={0,0,0,0,0,.02};
  double accumulated_rotation=0,minimum_rotation_step=0;
  double max_fresh_dv=0,max_fresh_da=0;
  for(int tick=0;tick<20000;++tick) {
    if(tick%33==0) {
      spin.wire_seq=spin.recv_seq=1+tick/33;
      const auto a=preempted_rotation.outputKinematics();
      preempted_rotation.submitDeltaFrame(spin,tick==0?origin():a.pose);
      if(tick>0) {
        const auto b=preempted_rotation.outputKinematics();
        max_fresh_dv=std::max(max_fresh_dv,difference(a.velocity,b.velocity));
        max_fresh_da=std::max(max_fresh_da,difference(a.acceleration,b.acceleration));
      }
    }
    const auto a=preempted_rotation.lastPose();
    const auto b=preempted_rotation.tick(dt);
    const double advance=math::log3(math::rotationFromPose(a).transpose()*
                                     math::rotationFromPose(b))[2];
    accumulated_rotation+=advance;
    minimum_rotation_step=std::min(minimum_rotation_step,advance);
    require(preempted_rotation.diag().last_solve.result==ruckig::Result::Working,
            "repeated rotation preemption solve failed");
  }
  std::cout<<"gated fresh rotation progress "<<accumulated_rotation
           <<" rad, minimum step "<<minimum_rotation_step<<" rad; splice dv "
           <<max_fresh_dv<<", da "<<max_fresh_da<<"\n";
  require(accumulated_rotation>4.,"gated fresh rotations stopped before crossing pi");
  require(minimum_rotation_step>=-1e-10,"fresh rotation tangent wrapped and reversed");
  require(max_fresh_dv<1e-10 && max_fresh_da<1e-8,
          "repeated rotation preemption changed physical derivatives");

  CartesianChunkFollower f(config(true));f.submitDeltaFrame(frame(1),origin());
  for(int i=0;i<40;++i) f.tick(dt);
  const auto before=f.outputKinematics();
  f.setPlanRateGate(.25);
  const auto slow=f.outputKinematics();
  Vec6 v=before.velocity,a=before.acceleration;
  v={v.x*.25,v.y*.25,v.z*.25,v.rx*.25,v.ry*.25,v.rz*.25};
  a={a.x*.0625,a.y*.0625,a.z*.0625,a.rx*.0625,a.ry*.0625,a.rz*.0625};
  require(difference(v,slow.velocity)<1e-12 && difference(a,slow.acceleration)<1e-10,
          "profile derivatives disagree with piecewise-constant plan clock");
  const auto frozen=f.lastPose();f.setPlanRateGate(0);f.submitDeltaFrame(frame(3,true),f.lastPose());
  for(int i=0;i<10;++i) f.tick(dt);
  require(math::positionDistance(frozen,f.lastPose())<1e-12,"fresh frame bypassed zero plan gate");
  require(norm(f.outputKinematics().velocity)==0 && norm(f.outputKinematics().acceleration)==0,
          "zero plan gate has nonzero feedforward");

  // Body derivative conversion checked against the canonical SO(3) pose samples.
  f.setPlanRateGate(.7);f.tick(dt);
  double velocity_error=0,acceleration_error=0;
  for(int i=0;i<100;++i) {
    const auto x=f.outputKinematics();
    if(f.tInSegment()+2e-5>=f.segmentLengthSec()) {f.tick(dt);continue;}
    const double h=1e-5;
    f.tick(h);const auto y=f.outputKinematics();
    const Eigen::Vector3d w_fd=math::log3(math::rotationFromPose(x.pose).transpose()*
                                          math::rotationFromPose(y.pose))/h;
    const Eigen::Vector3d w0(x.velocity.rx,x.velocity.ry,x.velocity.rz);
    const Eigen::Vector3d w1(y.velocity.rx,y.velocity.ry,y.velocity.rz);
    const Eigen::Vector3d alpha(x.acceleration.rx,x.acceleration.ry,x.acceleration.rz);
    velocity_error=std::max(velocity_error,(w_fd-w0).norm());
    acceleration_error=std::max(acceleration_error,((w1-w0)/h-alpha).norm());
  }
  require(velocity_error<.001,"body velocity differs from SO(3) finite difference");
  require(acceleration_error<.1,"body acceleration differs from velocity finite difference");

  // Force fold remains a gauge transfer through fresh preemption.
  f.setPlanRateGate(1);f.tick(dt);
  const Eigen::Vector3d shift(.001,-.002,.0003);
  const Eigen::Quaterniond rot(Eigen::AngleAxisd(.03,Eigen::Vector3d(1,2,3).normalized()));
  require(f.absorbOffset(shift,rot),"active fold declined");
  const auto folded=f.outputKinematics();f.submitDeltaFrame(frame(4),f.lastPose());
  const auto rebased=f.outputKinematics();
  require(math::positionDistance(folded.pose,rebased.pose)<1e-12 &&
          math::orientationDistanceRad(folded.pose,rebased.pose)<1e-12,"fresh frame lost folded pose");
  require(difference(folded.velocity,rebased.velocity)<1e-10 &&
          difference(folded.acceleration,rebased.acceleration)<1e-8,"fresh frame lost folded dynamics");
  require(f.foldShift().norm()==0,"fresh integrated frame double-counts fold shift");

  // A refusal already pinned the accepted command. Keep only that reference;
  // latest frames may replace the paused window but cannot move it or renew grace.
  auto held=f.lastPose();held.x-=.006;
  f.holdAtSentReference(held,10.0);
  for(int i=0;i<100;++i) {
    f.holdAtSentReference(held,10.+i*dt);
    if(i%20==0) f.submitDeltaFrame(frame(10+i,true),held);
    require(math::positionDistance(f.tick(dt),held)<1e-12,"held plan ran ahead");
    require(norm(f.outputKinematics().velocity)==0,"held plan inherited rejected velocity");
  }
  require(!f.absorbOffset(shift,rot),"force fold applied into paused plan");
  require(f.resumeFromHold(held,10.2,.5,.05,.1)==HoldResumeResult::WarmResumed,
          "sent-reference hold could not resume");
  const auto first=f.tick(dt);require(math::positionDistance(first,held)<1e-12,"resume position jumped");
  require(norm(f.outputKinematics().velocity)<1e-10 && norm(f.outputKinematics().acceleration)<1e-8,
          "resume inherited unaccepted endpoint dynamics");
  f.tick(dt);const auto moving=f.outputKinematics();
  require(std::hypot(moving.velocity.x,moving.velocity.y)<=std::sqrt(2.)*.5*2000.*dt*dt+1e-5,
          "resume did not accelerate within jerk envelope from held command");
  f.holdAtSentReference(f.lastPose(),20.);
  for(int i=0;i<100;++i) f.holdAtSentReference(f.lastPose(),20.+i*.004);
  require(f.expireHoldPause(20.501,.5) && !f.active(),"held updates extended grace timer");
  std::cout<<"synthetic continuity/hold/gate/fold checks passed; body FD errors "
           <<velocity_error<<" rad/s, "<<acceleration_error<<" rad/s2\n";
}

// Optional recorded-delta replay. Input ticks include exogenous logged gate and
// refusal schedules. It compares generators; it is not a physical robot or IK
// counterfactual (the accepted-pose surrogate holds the prior output on refusal).
struct ReplaySettings {
  CartesianChunkFollowerConfig follower;
  FollowerOutputSmdConfig smd;
  double hold_grace_sec{.5};
  std::string profile{"recorded-era-fixed"};
};
ReplaySettings replaySettings(bool fresh,const DualArmConfig* stack) {
  ReplaySettings out;out.follower=config(fresh);
  out.smd.enable=true;out.smd.nf_linear_hz=3.5;out.smd.nf_angular_hz=2.5;
  out.smd.damping_ratio=1.;out.smd.velocity_ff=true;out.smd.profile_feedforward=true;
  if(!stack) return out;
  out.profile=fresh?"flow_infer_fresh":"flow_infer_smooth";
  const auto& profiles=stack->cartesian_control.tcp_pose_target_profiles;
  const auto it=std::find_if(profiles.begin(),profiles.end(),[&](const auto& p){
    return p.name==out.profile;
  });
  require(it!=profiles.end(),"requested replay profile missing from stack");
  const auto& rf=it->ruckig_follower;
  require(rf.enable && rf.controller==RuckigFollowerController::DeltaPreview,
          "replay requires an enabled delta_preview profile");
  require(rf.fresh_chunk_replan==fresh && rf.continuous_hold_resume==fresh,
          "replay profile does not have the expected execution flags");
  auto& c=out.follower;
  c.lin={rf.max_linear_velocity_m_s,rf.max_linear_accel_m_s2,rf.max_linear_jerk_m_s3};
  c.ang={rf.max_angular_velocity_rad_s,rf.max_angular_accel_rad_s2,rf.max_angular_jerk_rad_s3};
  c.window={rf.discard_head_steps,rf.consume_steps,rf.reserve_steps,rf.smoothing_window};
  c.guard.af_damping_beta_lin=rf.af_damping_beta_lin;
  c.guard.af_damping_beta_ang=rf.af_damping_beta_ang;
  c.guard.corner_deadband_lin_m=rf.corner_deadband_lin_m;
  c.guard.corner_deadband_ang_rad=rf.corner_deadband_ang_rad;
  c.guard.corner_velocity_scale=rf.corner_velocity_scale;
  c.core_time_stretch_enable=rf.core_time_stretch_enable;
  c.core_time_stretch_max_ratio=rf.core_time_stretch_max_ratio;
  c.fresh_chunk_replan=rf.fresh_chunk_replan;
  c.continuous_hold_resume=rf.continuous_hold_resume;
  out.hold_grace_sec=rf.hold_bounce_resume_sec;
  out.smd=rf.output_smd;
  return out;
}
nlohmann::json replayDynamics(const ReplaySettings& settings) {
  const auto& c=settings.follower;const auto& s=settings.smd;
  return {{"linear_v_a_j",{c.lin.v_max,c.lin.a_max,c.lin.j_max}},
    {"angular_v_a_j",{c.ang.v_max,c.ang.a_max,c.ang.j_max}},
    {"window_L_C_R_smoothing",{c.window.discard_head_L,c.window.consume_C,
      c.window.reserve_R,c.window.smoothing_window}},
    {"af_beta_lin_ang",{c.guard.af_damping_beta_lin,c.guard.af_damping_beta_ang}},
    {"corner_deadband_lin_ang",{c.guard.corner_deadband_lin_m,c.guard.corner_deadband_ang_rad}},
    {"corner_velocity_scale",c.guard.corner_velocity_scale},{"eps_clamp",c.guard.eps_clamp},
    {"time_stretch_enabled",c.core_time_stretch_enable},
    {"time_stretch_max_ratio",c.core_time_stretch_max_ratio},
    {"hold_grace_sec",settings.hold_grace_sec},
    {"output_smd",{{"enable",s.enable},{"nf_linear_hz",s.nf_linear_hz},
      {"nf_angular_hz",s.nf_angular_hz},{"damping_ratio",s.damping_ratio},
      {"velocity_ff",s.velocity_ff},{"velocity_ff_lpf_hz",s.velocity_ff_lpf_hz},
      {"velocity_ff_linear_gain",s.velocity_ff_linear_gain},
      {"profile_feedforward",s.profile_feedforward}}}};
}
nlohmann::json poseArray(const Pose6D& pose) {
  const Eigen::Quaterniond q(math::rotationFromPose(pose));
  return {pose.x,pose.y,pose.z,q.w(),q.x(),q.y(),q.z()};
}
nlohmann::json vectorArray(const Vec6& v) { return {v.x,v.y,v.z,v.rx,v.ry,v.rz}; }
void replay(const std::string& path,const std::string& stack_path={},
            const std::string& trace_path={}) {
  std::ifstream input(path);require(input.good(),"cannot open recorded replay input");
  std::vector<nlohmann::json> rows;std::string line;
  while(std::getline(input,line)) if(!line.empty()) rows.push_back(nlohmann::json::parse(line));
  require(!rows.empty(),"recorded replay is empty");
  Pose6D initial=origin();
  if(rows.front().contains("initial_pose")) {
    const auto& p=rows.front().at("initial_pose");
    require(p.is_array() && p.size()==7,"initial_pose must be xyz,qw,qx,qy,qz");
    for(const auto& x:p) require(x.is_number() && std::isfinite(x.get<double>()),
                                "initial_pose must be finite");
    const Eigen::Quaterniond q(p[3].get<double>(),p[4].get<double>(),p[5].get<double>(),p[6].get<double>());
    require(std::abs(q.norm()-1.)<1e-3,"initial_pose quaternion must be unit length");
    initial=math::poseFromSe3(pinocchio::SE3(q.normalized().toRotationMatrix(),
      Eigen::Vector3d(p[0].get<double>(),p[1].get<double>(),p[2].get<double>())));
  }
  std::ofstream trace;
  if(!trace_path.empty()) {trace.open(trace_path);require(trace.good(),"cannot open replay trace");}
  std::optional<DualArmConfig> stack;
  if(!stack_path.empty()) stack=loadConfigFromYaml(stack_path);
  const std::array<ReplaySettings,2> settings{
    replaySettings(false,stack?&*stack:nullptr),replaySettings(true,stack?&*stack:nullptr)};
  auto fixed_dynamics=replayDynamics(settings[0]),fresh_dynamics=replayDynamics(settings[1]);
  // Conditioner comparisons may differ; motion limits and guards must still
  // match. Both complete effective configurations are emitted with the results.
  fixed_dynamics.erase("output_smd");fresh_dynamics.erase("output_smd");
  require(fixed_dynamics==fresh_dynamics,
          "replay profiles differ in motion dynamics or guards");
  for(bool fresh:{false,true}) {
    const auto& selected=settings[fresh?1:0];
    CartesianChunkFollower follower(selected.follower);
    FollowerOutputSmd smd(selected.smd);
    Pose6D accepted=initial,previous=accepted;Vec6 prev_v{},prev_a{};
    std::uint64_t last_consumed=0;int reset_ticks=0,received=0,consumed=0;
    double max_step=0,max_dv_linear=0,max_dv_angular=0,max_da_linear=0,max_da_angular=0,sum_path=0;
    std::map<std::uint64_t,double> receipt;
    std::vector<double> delays_ms;
    std::optional<ChunkFrame> latest_frame;
    std::uint64_t submitted_frame=0;
    for(const auto& row:rows) {
      const double t=row.at("t");const bool blocked=row.value("blocked",false);
      const double tick_dt=row.value("dt_sec",dt);
      require(std::isfinite(tick_dt) && tick_dt>0.,"invalid replay tick dt");
      const int prior_resets=reset_ticks;
      if(blocked && follower.active()) {
        if(fresh) {if(!follower.holdPaused()){smd.reset(accepted,{});++reset_ticks;}
          follower.holdAtSentReference(accepted,t);
        } else {follower.pauseForHold(t);smd.deactivate();}
        follower.expireHoldPause(t,selected.hold_grace_sec);
      } else if(follower.holdPaused()) {
        smd.deactivate();follower.resumeFromHold(accepted,t,selected.hold_grace_sec,.05,.1);
      }
      if(row.contains("delta")) {
        ChunkFrame chunk;chunk.policy_dt=row.value("policy_dt",policy_dt);chunk.recv_time=t;
        chunk.wire_seq=row.at("seq");chunk.recv_seq=chunk.wire_seq;
        for(const auto& d:row.at("delta")) {
          chunk.delta.push_back({d[0],d[1],d[2],d[3],d[4],d[5]});chunk.grip.push_back(d[6]);
        }
        receipt[chunk.wire_seq]=t;++received;
        latest_frame=std::move(chunk);
      }
      if(latest_frame && latest_frame->wire_seq!=submitted_frame &&
         !(fresh && blocked && !follower.active())) {
        follower.submitDeltaFrame(*latest_frame,accepted);
        submitted_frame=latest_frame->wire_seq;
      }
      const auto direction=row.at("force_dir");
      follower.setAdvanceGate(row.at("advance_gate").get<double>(),Eigen::Vector3d(
          direction[0].get<double>(),direction[1].get<double>(),direction[2].get<double>()));
      follower.setPlanRateGate(row.at("plan_gate"));
      if(!follower.active()) continue;
      const auto pose=follower.tick(tick_dt);auto state=follower.outputKinematics();
      if(!fresh && !selected.smd.profile_feedforward) {
        state.velocity=follower.currentVelocity().value_or(Vec6{});
        state.acceleration={};
      } else if(!fresh) {
        state.velocity=follower.sampledVelocity().value_or(Vec6{});
        state.acceleration=follower.sampledAcceleration().value_or(Vec6{});
      }
      Pose6D out;
      bool internal_reset=false;
      if(fresh && follower.holdPaused()) out=accepted;
      else if(!selected.smd.enable) out=pose;
      else {
        if(!smd.active()) {
          Vec6 seed_velocity=state.velocity;
          if(fresh) {
            const Eigen::Vector3d omega=math::rotationFromPose(accepted).transpose()*
                math::rotationFromPose(state.pose)*
                Eigen::Vector3d(state.velocity.rx,state.velocity.ry,state.velocity.rz);
            seed_velocity.rx=omega.x();seed_velocity.ry=omega.y();seed_velocity.rz=omega.z();
          }
          smd.reset(accepted,seed_velocity);++reset_ticks;
        }
        out=smd.step(pose,state.velocity,tick_dt,
            selected.smd.profile_feedforward?&state.acceleration:nullptr,fresh);
        internal_reset=smd.reseededLastStep();
      }
      if(follower.diag().seg_wire_seq!=last_consumed){
        last_consumed=follower.diag().seg_wire_seq;
        if(receipt.count(last_consumed)) {++consumed;delays_ms.push_back(1000*(t-receipt[last_consumed]));}
      }
      const double step=math::positionDistance(previous,out);
      max_step=std::max(max_step,step);sum_path+=step;
      max_dv_linear=std::max(max_dv_linear,Eigen::Vector3d(state.velocity.x-prev_v.x,
          state.velocity.y-prev_v.y,state.velocity.z-prev_v.z).norm());
      max_dv_angular=std::max(max_dv_angular,Eigen::Vector3d(state.velocity.rx-prev_v.rx,
          state.velocity.ry-prev_v.ry,state.velocity.rz-prev_v.rz).norm());
      max_da_linear=std::max(max_da_linear,Eigen::Vector3d(state.acceleration.x-prev_a.x,
          state.acceleration.y-prev_a.y,state.acceleration.z-prev_a.z).norm());
      max_da_angular=std::max(max_da_angular,Eigen::Vector3d(state.acceleration.rx-prev_a.rx,
          state.acceleration.ry-prev_a.ry,state.acceleration.rz-prev_a.rz).norm());
      if(trace.is_open()) trace<<nlohmann::json{{"fresh",fresh},{"t",t},{"dt_sec",tick_dt},
        {"prefilter",poseArray(pose)},{"stage",poseArray(out)},
        {"sample_velocity",vectorArray(state.velocity)},
        {"sample_acceleration",vectorArray(state.acceleration)},
        {"segment",follower.diag().segments},{"seg_wire_seq",follower.diag().seg_wire_seq},
        {"seg_step_index",follower.diag().seg_step_index},
        {"solve_corner",follower.diag().last_solve.corner},
        {"solve_duration",follower.diag().last_solve.duration},
        {"t_in_segment",follower.tInSegment()},{"segment_length",follower.segmentLengthSec()},
        {"blocked",blocked},{"hold_paused",follower.holdPaused()},
        {"outer_reset",reset_ticks>prior_resets},{"internal_reset",internal_reset},
        {"new_frame",row.contains("delta")},{"plan_gate",row.at("plan_gate")},
        {"advance_gate",row.at("advance_gate")}}.dump()<<'\n';
      previous=out;prev_v=state.velocity;prev_a=state.acceleration;
      if(!blocked) accepted=out;
    }
    std::sort(delays_ms.begin(),delays_ms.end());
    const auto percentile=[&](double p){return delays_ms.empty()?0.:delays_ms[
        static_cast<std::size_t>(p*(delays_ms.size()-1))];};
    std::cout<<nlohmann::json{{"fresh",fresh},{"ticks",rows.size()},{"received",received},
      {"stack_config",stack_path},{"profile",selected.profile},
      {"initial_pose",poseArray(initial)},
      {"initial_pose_source",rows.front().contains("initial_pose")?"recorded_first_row":"legacy_fixture_origin"},
      {"effective_dynamics",replayDynamics(selected)},
      {"consumed_frames",consumed},{"outer_reset_ticks",reset_ticks},
      {"stage_max_step_mm",1000*max_step},{"stage_path_m",sum_path},
      {"sample_max_velocity_step_m_s",max_dv_linear},{"sample_max_velocity_step_rad_s",max_dv_angular},
      {"sample_max_acceleration_step_m_s2",max_da_linear},{"sample_max_acceleration_step_rad_s2",max_da_angular},
      {"receipt_to_consumption_p50_ms",percentile(.5)},
      {"receipt_to_consumption_p95_ms",percentile(.95)},
      {"receipt_to_consumption_max_ms",percentile(1.)},
      {"ruckig_failures",follower.diag().solve_failure_count}}.dump()<<'\n';
  }
}
}
int main(int argc,char** argv) {
  try {
    if(argc==1) synthetic();
    else if(argc>=3 && std::string(argv[1])=="--replay") {
      std::string stack_path,trace_path;
      for(int i=3;i<argc;i+=2) {
        require(i+1<argc,"missing replay option value");
        if(std::string(argv[i])=="--stack-config") stack_path=argv[i+1];
        else if(std::string(argv[i])=="--trace-jsonl") trace_path=argv[i+1];
        else throw std::runtime_error("unknown replay option");
      }
      replay(argv[2],stack_path,trace_path);
    } else throw std::runtime_error("usage: test_chunk_fresh_continuity [--replay JSONL [--stack-config YAML] [--trace-jsonl JSONL]]");
  }
  catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
