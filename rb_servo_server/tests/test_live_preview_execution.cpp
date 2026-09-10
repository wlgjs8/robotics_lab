#include "rb_servo/control/live_preview_execution.hpp"
#include "rb_servo/control/joint_stationarity.hpp"
#include "rb_servo/core/clock.hpp"
#include "rb_servo/math/se3.hpp"
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <limits>
#include <thread>

// A real background planning thread with a deterministic host clock. No backend,
// socket, sensor or model process is constructed. Clock advancement never claims
// real-time scheduling acceptance; it makes late/epoch cases reproducible.
namespace {
using namespace rb_servo;
using namespace rb_servo::control;
#define CHECK(x) do { if(!(x)) {std::cerr<<"CHECK " #x " failed at "<<__LINE__<<'\n';return false;} } while(false)
constexpr std::uint64_t kStartNs=1'000'000'000ULL;
constexpr std::uint64_t kDtNs=2'000'000ULL;
constexpr double kDt=.002;

CartesianChunkFollowerConfig rawConfig() {
  CartesianChunkFollowerConfig c;c.lin={.6,12,2000};c.ang={1.4,40,4000};
  c.window={0,8,4,1};c.fresh_chunk_replan=true;c.continuous_hold_resume=true;return c;
}
RuckigFollowerConfig config(bool recovery=false) {
  RuckigFollowerConfig c;c.enable=true;c.controller=RuckigFollowerController::DeltaPreview;
  c.fresh_chunk_replan=true;c.continuous_hold_resume=true;
  c.plan_leash_enable=true;c.plan_leash_start_m=.01;c.plan_leash_start_rad=.0349;
  c.plan_leash_full_m=.05;c.plan_leash_full_rad=.1;c.plan_leash_min_gate=.25;
  auto& p=c.preview_execution;p.enable=true;p.replan_period_sec=.01;p.splice_lead_sec=.01;
  p.max_result_age_sec=.05;p.worker_poll_period_sec=.0005;p.max_source_rows=32;
  p.cursor={true,.1,.2,1.1,1e-6,1e-6};
  if(recovery)p.recovery={true,.25,3};
  auto& t=p.tracker;t.planning_dt_sec=.01;t.horizon_steps=24;
  t.max_linear_velocity_m_s=.6;t.max_linear_acceleration_m_s2=12;t.max_linear_jerk_m_s3=2000;
  t.max_angular_velocity_rad_s=1.4;t.max_angular_acceleration_rad_s2=40;t.max_angular_jerk_rad_s3=4000;
  t.linear_tracking_scale_m=.01;t.angular_tracking_scale_rad=.03;
  t.jerk_weight=2000;t.jerk_difference_weight=.01;
  t.linear_tracking_tolerance_m=.02;t.angular_tracking_tolerance_rad=.08;
  t.max_linear_tracking_slack_m=.06;t.max_angular_tracking_slack_rad=.27;
  t.max_reference_chart_angle_rad=1;t.feasibility_tolerance=1e-7;
  t.max_working_set_recalculations=200;t.max_solve_time_sec=.05;return c;
}
Pose6D pose(double x=.4) {return math::poseFromSe3(pinocchio::SE3(Eigen::Matrix3d::Identity(),Eigen::Vector3d{x,.1,.3}));}
ChunkFrame frame(std::uint64_t wire=17,std::uint64_t recv=8,double delta=.001,double angular_delta=0.) {
  ChunkFrame f;f.policy_dt=1./30;f.wire_seq=wire;f.recv_seq=recv;f.recv_time=1;
  for(int i=0;i<24;++i) {auto p=math::poseFromSe3(pinocchio::SE3(
        math::exp3(Eigen::Vector3d{0.,0.,angular_delta*i}),Eigen::Vector3d{.4+delta*i,.1,.3}));
    f.pose.push_back(p);f.grip.push_back(.5);f.delta.push_back(Vec6{delta,0,0,0,0,angular_delta});}
  return f;
}
void letWorkerRun() {std::this_thread::sleep_for(std::chrono::milliseconds(3));}
struct Fixture {
  CartesianChunkFollower raw{rawConfig()};
  LivePreviewExecution exec;
  Pose6D accepted{pose()};std::uint64_t tick{0};
  explicit Fixture(double delta=.001,double angular_delta=0.,bool recovery=false)
      : exec(config(recovery),rawConfig(),kDt) {raw.submitDeltaFrame(frame(17,8,delta,angular_delta),accepted);}
  double now() const {return static_cast<double>(kStartNs+tick*kDtNs)*1e-9;}
  LivePreviewOutput step(bool stationary=true,double gate=1.,
                         const Eigen::Vector3d& normal=Eigen::Vector3d::Zero()) {
    setExternalSteadyNs(kStartNs+tick*kDtNs);raw.tick(kDt);
    auto out=exec.step(now(),raw,accepted,stationary,gate,normal);++tick;return out;
  }
  bool accept(const LivePreviewOutput& out) {
    if(!out.active)return true;
    const auto tx=exec.transaction(out.pose,out.pose);
    if(!exec.observeDispatch(tx,out.pose,true,.002,.01))return false;
    accepted=out.pose;return true;
  }
  bool engage() {
    for(int i=0;i<20;++i) {auto out=step();if(out.fault)return false;if(out.active)return accept(out);letWorkerRun();}
    return false;
  }
};

bool sameMotion(const PreviewMotionState& a,const PreviewMotionState& b) {
  return math::positionDistance(a.pose,b.pose)<1e-11&&math::orientationDistanceRad(a.pose,b.pose)<1e-10&&
      (a.linear_velocity-b.linear_velocity).norm()<1e-10&&
      (a.linear_acceleration-b.linear_acceleration).norm()<1e-8&&
      (a.angular_velocity_body-b.angular_velocity_body).norm()<1e-10&&
      (a.angular_acceleration_body-b.angular_acceleration_body).norm()<1e-8;
}

bool coldAndC2Splice() {
  setExternalSteadyNs(kStartNs);Fixture f;
  auto first=f.step(false);CHECK(!first.active&&!first.fault);CHECK(!f.exec.initialized());
  CHECK(std::string(f.exec.telemetry().status)=="braking");
  const auto initial=f.step();CHECK(!initial.active&&!initial.fault);CHECK(f.exec.initialized());
  CHECK(!f.exec.telemetry().active);CHECK(math::positionDistance(initial.pose,f.accepted)==0);
  letWorkerRun();CHECK(f.engage());CHECK(f.exec.telemetry().active);
  bool saw_splice=false;
  for(int i=0;i<28;++i) {
    const auto before=f.exec.sample();const auto previous_id=f.exec.telemetry().plan_id;
    letWorkerRun();auto out=f.step();CHECK(!out.fault);CHECK(out.active);
    const auto after=f.exec.sample();
    if(f.exec.telemetry().plan_id!=previous_id) {
      // Requests use a 10 ms splice lead and a 10 ms jerk grid. The last 2 ms
      // before a splice therefore lies in one predecessor polynomial interval.
      const Eigen::Vector3d expected_p=Eigen::Vector3d(before.pose.x,before.pose.y,before.pose.z)+
          kDt*before.linear_velocity+.5*kDt*kDt*before.linear_acceleration+
          (kDt*kDt*kDt/6)*before.linear_jerk;
      const Eigen::Vector3d expected_v=before.linear_velocity+kDt*before.linear_acceleration+
          .5*kDt*kDt*before.linear_jerk;
      const Eigen::Vector3d expected_a=before.linear_acceleration+kDt*before.linear_jerk;
      CHECK((Eigen::Vector3d(after.pose.x,after.pose.y,after.pose.z)-expected_p).norm()<1e-9);
      CHECK((after.linear_velocity-expected_v).norm()<1e-8);
      CHECK((after.linear_acceleration-expected_a).norm()<1e-6);
      saw_splice=true;
    }
    CHECK(f.accept(out));
  }
  CHECK(saw_splice);return true;
}

bool epochsAndContinuousGateIdentity() {
  setExternalSteadyNs(kStartNs);Fixture f;f.step();letWorkerRun();
  const auto epoch=f.exec.telemetry().epoch;f.exec.reset("init_motion");
  CHECK(f.exec.telemetry().epoch>epoch);
  auto out=f.step();CHECK(!out.active&&!out.fault);CHECK(f.exec.telemetry().rejected>=1);
  letWorkerRun();f.raw.setAdvanceGate(.5,{-1,0,0});
  const auto rejects=f.exec.telemetry().rejected;
  out=f.step();CHECK(!out.active&&!out.fault);CHECK(f.exec.telemetry().rejected==rejects);
  letWorkerRun();CHECK(f.engage());
  // A changing force forecast is not a coordinate reset. The current output
  // authority is checked separately; changing raw gates alone cannot starve
  // every future splice as in the original exact-identity implementation.
  for(int i=0;i<40;++i){f.raw.setPlanRateGate(i%2?.8:.9);letWorkerRun();out=f.step();
    CHECK(!out.fault);CHECK(f.accept(out));}
  auto tx=f.exec.transaction(f.exec.sample().pose,f.exec.sample().pose);
  f.exec.reset("profile_exit");CHECK(f.exec.observeDispatch(tx,pose(),true,.002,.01));
  CHECK(!f.exec.telemetry().active&&!f.exec.initialized());
  return true;
}

bool acceptedTransactionGaugeAndDeviation() {
  setExternalSteadyNs(kStartNs);Fixture f;CHECK(f.engage());
  const auto nominal=f.exec.sample().pose;
  auto T=math::se3FromPose(nominal);T.translation()+=Eigen::Vector3d{.013,-.006,.002};
  T.rotation()=math::exp3(Eigen::Vector3d{.03,-.02,.01})*T.rotation();
  const auto composed=math::poseFromSe3(T);
  const auto tx=f.exec.transaction(nominal,composed);CHECK(tx.valid);
  // A queued command carries the original gauge even if a later target uses a
  // different force offset. There is no live-overlay argument to reinterpret it.
  auto other=f.exec.transaction(nominal,pose(.45));(void)other;
  CHECK(f.exec.observeDispatch(tx,composed,true,.002,.01));
  CHECK(f.exec.telemetry().accepted_position_error_m<1e-12);
  CHECK(f.exec.telemetry().accepted_rotation_error_rad<1e-12);
  auto bad=composed;bad.x+=.003;
  CHECK(!f.exec.observeDispatch(tx,bad,true,.002,.01));CHECK(f.exec.failed());
  CHECK(std::string(f.exec.telemetry().status)=="accepted_deviation");
  CHECK(!f.exec.telemetry().active);
  return true;
}

bool frameShiftAndCanonicalIndependence() {
  setExternalSteadyNs(kStartNs);Fixture f;CHECK(f.engage());
  const auto before=f.exec.sample();const Eigen::Vector3d dp{.001,-.002,.003};
  const Eigen::Quaterniond dR(math::exp3(Eigen::Vector3d{.04,.02,-.03}));
  CHECK(f.raw.absorbOffset(dp,dR));f.exec.shiftCommonFrame(dp,dR);
  auto expected=math::se3FromPose(before.pose);expected.translation()+=dp;expected.rotation()=dR*expected.rotation();
  CHECK(math::positionDistance(f.exec.sample().pose,math::poseFromSe3(expected))<1e-12);
  CHECK(math::orientationDistanceRad(f.exec.sample().pose,math::poseFromSe3(expected))<1e-12);
  CHECK((f.exec.sample().linear_velocity-before.linear_velocity).norm()==0);
  CHECK((f.exec.sample().angular_velocity_body-before.angular_velocity_body).norm()==0);
  f.accepted=f.exec.sample().pose;
  CartesianChunkFollower baseline=f.raw;
  for(int i=0;i<20;++i) {
    if(i==5) {const auto fresh=frame(18,9,-.0005);f.raw.submitDeltaFrame(fresh,f.raw.lastPose());baseline.submitDeltaFrame(fresh,baseline.lastPose());}
    const auto reference=baseline.tick(kDt);letWorkerRun();const auto out=f.step();
    CHECK(math::positionDistance(f.raw.lastPose(),reference)==0);
    CHECK(math::orientationDistanceRad(f.raw.lastPose(),reference)<1e-12);
    CHECK(f.raw.windowIndex()==baseline.windowIndex());CHECK(f.raw.tInSegment()==baseline.tInSegment());
    CHECK(!out.fault);CHECK(f.accept(out));
  }
  return true;
}

bool expiryAndDispatchRefusal() {
  setExternalSteadyNs(kStartNs);Fixture f;CHECK(f.engage());
  // Replace source identity before each worker result can be admitted. Unlike
  // ordinary continuous gate changes, a different source row cannot silently
  // authorize its predecessor's proposal. The old finite plan must brake.
  bool braked=false,rest=false;
  for(int i=0;i<80;++i) {
    f.raw.submitDeltaFrame(frame(100+i,200+i),f.raw.lastPose());
    letWorkerRun();const auto out=f.step();
    if(out.fault){CHECK(braked&&rest);CHECK(!out.active);break;}
    if(f.exec.braking()){
      braked=true;CHECK(out.active);CHECK(!f.exec.telemetry().active);
      rest=rest||(f.exec.sample().linear_velocity.norm()<1e-10&&
                  f.exec.sample().angular_velocity_body.norm()<1e-10&&
                  f.exec.sample().linear_acceleration.norm()<1e-10);
    }
    CHECK(f.accept(out));
  }
  CHECK(braked&&rest);CHECK(f.exec.telemetry().expired>=1);
  f.exec.reset();CHECK(!f.exec.failed());CHECK(f.engage());
  const auto tx=f.exec.transaction(f.exec.sample().pose,f.exec.sample().pose);
  CHECK(!f.exec.observeDispatch(tx,f.exec.sample().pose,false,.002,.01));
  CHECK(std::string(f.exec.telemetry().status)=="dispatch_rejected");
  return true;
}

bool invalidInputAndContactStop() {
  setExternalSteadyNs(kStartNs);Fixture f;CHECK(f.engage());
  for(int invalid=0;invalid<6;++invalid){
    auto tx=f.exec.transaction(f.exec.sample().pose,f.exec.sample().pose);
    if(invalid==0)tx.nominal.x=std::numeric_limits<double>::quiet_NaN();
    if(invalid==1)tx.motion.linear_jerk.x()=std::numeric_limits<double>::quiet_NaN();
    if(invalid==2)tx.sample_time_sec+=kDt;
    if(invalid==3)tx.sample_time_sec-=kDt;
    if(invalid==4)tx.motion.pose.x+=.00001;
    if(invalid==5)tx.fold_rotation=Eigen::Quaterniond(2.,0.,0.,0.);
    CHECK(!f.exec.observeDispatch(tx,f.exec.sample().pose,true,.002,.01));CHECK(f.exec.failed());
    CHECK(std::string(f.exec.telemetry().status)=="invalid_acceptance");
    f.exec.reset();CHECK(f.engage());
  }
  const auto before=f.exec.sample();CHECK(f.exec.contactGuardStopped());
  CHECK(!f.exec.failed()&&f.exec.braking());CHECK(f.exec.telemetry().contact_guard_count==1);
  CHECK(!f.exec.telemetry().active);
  CHECK(math::positionDistance(before.pose,f.exec.sample().pose)<1e-12);
  CHECK((before.linear_velocity-f.exec.sample().linear_velocity).norm()<1e-12);
  CHECK((before.linear_acceleration-f.exec.sample().linear_acceleration).norm()<1e-10);
  CHECK(f.exec.transaction(f.exec.sample().pose,f.exec.sample().pose).valid);
  return true;
}

bool oldAcceptedTransactionAcrossFoldSeedsBrake(){
  setExternalSteadyNs(kStartNs);Fixture f;CHECK(f.engage());
  const auto before=f.exec.sample();const auto tx=f.exec.transaction(before.pose,before.pose);
  const Eigen::Vector3d dp{.003,-.002,.001};const Eigen::Quaterniond dr(math::exp3(Eigen::Vector3d{.04,-.02,.01}));
  CHECK(f.raw.absorbOffset(dp,dr));f.exec.shiftCommonFrame(dp,dr);
  CHECK(f.exec.observeDispatch(tx,tx.composed,true,.002,.01));
  CHECK(f.exec.contactGuardStopped());const auto after=f.exec.sample();
  auto expected=math::se3FromPose(before.pose);expected.translation()+=dp;expected.rotation()=dr*expected.rotation();
  CHECK(math::positionDistance(after.pose,math::poseFromSe3(expected))<1e-12);
  CHECK(math::orientationDistanceRad(after.pose,math::poseFromSe3(expected))<1e-12);
  CHECK((after.angular_velocity_body-before.angular_velocity_body).norm()<1e-12);
  CHECK((after.angular_acceleration_body-before.angular_acceleration_body).norm()<1e-10);
  return true;
}

bool currentVelocityAuthority() {
  setExternalSteadyNs(kStartNs);Fixture f;CHECK(f.engage());
  bool witnessed_no_double_gate=false;
  // Find an actual positive preview velocity that is inside the canonical
  // closing authority but exceeds a second multiplication by the force gate.
  for(int i=0;i<35&&!witnessed_no_double_gate;++i) {
    letWorkerRun();const auto out=f.step(true,.01,Eigen::Vector3d::UnitX());
    CHECK(!out.fault&&!f.exec.braking());CHECK(f.accept(out));
    const double raw_v=f.raw.outputKinematics().velocity.x;
    const double preview_v=f.exec.sample().linear_velocity.x();
    witnessed_no_double_gate=out.active&&raw_v>1e-5&&preview_v>.01*raw_v+1e-7;
    CHECK(preview_v<=std::max(0.,raw_v)+config().preview_execution.tracker.feasibility_tolerance);
  }
  CHECK(witnessed_no_double_gate);CHECK(f.exec.telemetry().contact_guard_count==0);
  const auto accepted_seed=f.exec.sample();
  PreviewBrake expected(config().preview_execution.tracker,kDt);
  CHECK(expected.start(accepted_seed)==PreviewBrakeStatus::Ready);
  // This test explicitly freezes the canonical phase; it does not assume a
  // force gate instantly removes in-flight canonical Ruckig velocity.
  f.raw.setPlanRateGate(0);letWorkerRun();
  const auto before_clamp=f.exec.sample();
  const auto stopped=f.step(true,.01,Eigen::Vector3d::UnitX());
  CHECK(f.raw.outputKinematics().velocity.x==0);
  // 2026-09-10: the active plan losing closing authority is CLAMPED, not braked.
  // Its closing velocity is cut to the authority (0 here), the refused advance is
  // held back, and the plan keeps executing until the constrained replan splices.
  CHECK(stopped.active&&!stopped.fault&&!f.exec.braking());
  CHECK(f.exec.telemetry().contact_guard_count==0);
  CHECK(f.exec.telemetry().contact_clamp_count==1&&f.exec.contactClampActive());
  const double tol=config().preview_execution.tracker.feasibility_tolerance;
  CHECK(f.exec.sample().linear_velocity.x()<=tol);
  CHECK(f.exec.sample().linear_acceleration.x()<=tol);
  // held back: no advance into the contact beyond the refused displacement's
  // second-order residual of one tick
  CHECK(f.exec.sample().pose.x<=before_clamp.pose.x+1e-6);
  CHECK(f.exec.telemetry().contact_clamp_shift_m>0);
  CHECK(f.accept(stopped));
  // The constrained replan (contact knots at authority 0) takes over without a
  // brake; the clamp retires with it and closing stays inside authority.
  bool replanned=false;
  for(int i=0;i<40&&!replanned;++i) {
    letWorkerRun();const auto out=f.step(true,.01,Eigen::Vector3d::UnitX());
    if(!(out.active&&!out.fault&&!f.exec.braking())) {
      const auto& tl=f.exec.telemetry();
      std::cerr<<"clamp loop i="<<i<<" status="<<tl.status<<" brake="<<tl.last_brake_reason
               <<" clamp_shift="<<tl.contact_clamp_shift_m<<" clamps="<<tl.contact_clamp_count
               <<" accepted="<<tl.accepted<<" rejected="<<tl.rejected<<" expired="<<tl.expired
               <<" last_cancel="<<tl.last_staged_cancel_reason<<" worker="<<tl.last_worker_status
               <<" solve="<<tl.last_solve_status<<" raw_v="<<f.raw.outputKinematics().velocity.x<<'\n';
    }
    CHECK(out.active&&!out.fault&&!f.exec.braking());
    CHECK(f.exec.sample().linear_velocity.x()<=std::max(0.,f.raw.outputKinematics().velocity.x)+tol);
    CHECK(f.accept(out));
    replanned=!f.exec.contactClampActive();
  }
  CHECK(replanned);CHECK(f.exec.telemetry().contact_guard_count==0);
  CHECK(f.exec.telemetry().contact_clamp_shift_m==0);
  return true;
}

bool contactClampFallsBackToBrakeOnlyWhenNoReplanArrives() {
  // Starve the worker: the clamp holds back the refused advance tick after tick
  // and never executes a closing velocity; with no compliant replan admitted the
  // violating plan expires inside max_result_age_sec and the finite brake remains
  // the last resort. There is no separate displacement bound.
  setExternalSteadyNs(kStartNs);Fixture f(.01);CHECK(f.engage());
  bool moving=false;
  for(int i=0;i<90&&!moving;++i) {
    letWorkerRun();const auto out=f.step();CHECK(!out.fault&&!f.exec.braking());CHECK(f.accept(out));
    moving=f.exec.sample().linear_velocity.x()>.08;
  }
  CHECK(moving);
  f.raw.setPlanRateGate(0);
  bool braked=false;std::uint64_t clamps=0;
  for(int i=0;i<200&&!braked;++i) {
    const auto out=f.step(true,0,Eigen::Vector3d::UnitX());  // no letWorkerRun: no replan can land
    CHECK(!out.fault);CHECK(f.accept(out));
    clamps=f.exec.telemetry().contact_clamp_count;
    braked=f.exec.braking();
    if(!braked)CHECK(f.exec.sample().linear_velocity.x()<=config().preview_execution.tracker.feasibility_tolerance);
  }
  CHECK(clamps>1);
  // the plan expired (braking_expired): a brake, not a fault, and no clamp ever
  // executed a closing velocity
  CHECK(braked);CHECK(std::string(f.exec.telemetry().last_brake_reason)=="braking_expired");
  return true;
}

bool coldRetreatAuthority() {
  // A retreating raw reference must admit a cold stationary state, even when
  // that state is spatially ahead of the retreating reference. Give an admitted
  // plan the same 20 active samples regardless of where inside the unchanged
  // first-plan deadline its background solve completes. A fixed 25 cold ticks
  // could end immediately after admission without testing any motion at all.
  // The prior fixture has been destroyed before resetting the shared clock.
  setExternalSteadyNs(kStartNs);Fixture retreat(-.001);bool escaped=false;
  int active_samples=0;
  for(int i=0;i<50&&active_samples<20;++i) {
    letWorkerRun();const auto out=retreat.step(true,0,Eigen::Vector3d::UnitX());
    CHECK(!out.fault&&!retreat.exec.braking());CHECK(retreat.accept(out));
    if(out.active) {
      ++active_samples;
      CHECK(retreat.exec.sample().linear_velocity.x()<=1e-7);
      escaped=escaped||retreat.exec.sample().linear_velocity.x()<-1e-5;
    }
  }
  if(active_samples<20||!escaped) {
    const auto& t=retreat.exec.telemetry();
    std::cerr<<"cold retreat: active_samples="<<active_samples<<" elapsed="<<retreat.now()-1.
        <<" status="<<t.status<<" submitted="<<t.submitted<<" accepted="<<t.accepted
        <<" rejected="<<t.rejected<<" raw_v="<<retreat.raw.outputKinematics().velocity.x
        <<" preview_v="<<retreat.exec.sample().linear_velocity.x()<<'\n';
  }
  CHECK(active_samples==20);
  CHECK(escaped);CHECK(retreat.exec.telemetry().contact_guard_count==0);
  return true;
}

bool rejectedStagedPlanRetainsBrakeClock() {
  setExternalSteadyNs(kStartNs);Fixture f(.01);CHECK(f.engage());
  PreviewBrake expected(config().preview_execution.tracker,kDt);bool moving_seed=false;
  for(int i=0;i<90&&!moving_seed;++i) {
    letWorkerRun();const auto out=f.step();CHECK(!out.fault&&!f.exec.braking());CHECK(f.accept(out));
    moving_seed=f.exec.sample().linear_velocity.x()>.08&&
        expected.start(f.exec.sample())==PreviewBrakeStatus::Ready&&expected.durationSec()>.028;
  }
  CHECK(moving_seed);
  const double origin=f.now()-kDt;
  CHECK(f.exec.contactGuardStopped());const auto brake_id=f.exec.telemetry().plan_id;
  const auto rejected_before=f.exec.admissionDiagnostics().staged_contact_rejected;
  // The first brake tick requests a free-space successor with a 10 ms lead.
  // Keep that forecast until just before its exact splice, then revoke closing
  // authority. Its still-moving initial state must be rejected before replacing
  // the fixed stop predecessor.
  for(int i=0;i<5;++i) {
    letWorkerRun();const auto out=f.step();CHECK(out.active&&!out.fault&&f.exec.braking());
    CHECK(f.exec.telemetry().plan_id==brake_id);CHECK(f.accept(out));
  }
  f.raw.setPlanRateGate(0);letWorkerRun();
  auto out=f.step(true,0,Eigen::Vector3d::UnitX());
  CHECK(out.active&&!out.fault&&f.exec.braking());
  CHECK(f.exec.admissionDiagnostics().staged_contact_rejected==rejected_before+1);
  CHECK(f.exec.admissionDiagnostics().last_contact_reject_closing_m_s>1e-7);
  CHECK(f.exec.admissionDiagnostics().last_contact_reject_allowed_m_s==0);
  CHECK(f.exec.telemetry().plan_id==brake_id);
  PreviewMotionSample original_stop;CHECK(expected.sample(f.now()-kDt-origin,original_stop));
  CHECK(math::positionDistance(f.exec.sample().pose,original_stop.pose)<1e-11);
  CHECK((f.exec.sample().linear_velocity-original_stop.linear_velocity).norm()<1e-10);
  CHECK((f.exec.sample().linear_acceleration-original_stop.linear_acceleration).norm()<1e-8);
  CHECK(std::abs(f.exec.telemetry().plan_age_sec-(f.now()-kDt-origin))<1e-12);
  CHECK(f.accept(out));
  return true;
}

bool contactRetainsAngularUntilOriginalExpiry() {
  setExternalSteadyNs(kStartNs);Fixture f(.001,.012);CHECK(f.engage());
  PreviewExecutionResult source;bool moving=false;
  for(int i=0;i<60&&!moving;++i) {
    letWorkerRun();const auto out=f.step();CHECK(out.active&&!out.fault&&!f.exec.braking());CHECK(f.accept(out));
    moving=f.exec.sample().linear_velocity.x()>1e-4&&f.exec.sample().angular_velocity_body.norm()>.03&&
        f.exec.lastResult().identity.request_id==f.exec.telemetry().plan_id;
    if(moving)source=f.exec.lastResult();
  }
  CHECK(moving);const auto seed=f.exec.sample();const double origin=f.now()-kDt;
  PreviewMotionState linear_seed=seed;linear_seed.angular_velocity_body.setZero();linear_seed.angular_acceleration_body.setZero();
  PreviewBrake linear_stop(config().preview_execution.tracker,kDt);
  CHECK(linear_stop.start(linear_seed)==PreviewBrakeStatus::Ready);
  const auto starts=f.exec.admissionDiagnostics().angular_continuations_started;
  const auto angular_starts=f.exec.admissionDiagnostics().angular_brakes_started;
  CHECK(f.exec.contactGuardStopped());CHECK(f.exec.admissionDiagnostics().angular_continuations_started==starts+1);
  CHECK((f.exec.sample().angular_velocity_body-seed.angular_velocity_body).norm()<1e-10);
  CHECK((f.exec.sample().angular_acceleration_body-seed.angular_acceleration_body).norm()<1e-9);
  const auto translation_id=f.exec.telemetry().plan_id;std::uint64_t angular_id=0;
  PreviewBrake angular_stop(config().preview_execution.tracker,kDt);double angular_origin=0.;bool terminal=false;
  for(int i=0;i<40;++i) {
    // New source identities prevent replacement plans from being admitted, so
    // this test reaches the retained source's ORIGINAL deadline deliberately.
    f.raw.submitDeltaFrame(frame(1000+i,2000+i,.001,.012),f.raw.lastPose());
    const double now=f.now();
    if(now>=source.valid_until_sec&&angular_id==0) {
      auto accepted=f.exec.sample();accepted.linear_velocity.setZero();accepted.linear_acceleration.setZero();
      CHECK(angular_stop.start(accepted)==PreviewBrakeStatus::Ready);angular_origin=now-kDt;
    }
    letWorkerRun();const auto out=f.step();CHECK(out.active&&!out.fault&&f.exec.braking());
    PreviewMotionSample linear;CHECK(linear_stop.sample(now-origin,linear));
    CHECK(math::positionDistance(f.exec.sample().pose,linear.pose)<1e-10);
    CHECK((f.exec.sample().linear_velocity-linear.linear_velocity).norm()<1e-10);
    CHECK((f.exec.sample().linear_acceleration-linear.linear_acceleration).norm()<1e-8);
    CHECK(std::abs(f.exec.telemetry().plan_age_sec-(now-origin))<1e-12);
    PreviewMotionSample expected;
    if(now<source.valid_until_sec) {
      CHECK(f.exec.telemetry().plan_id==translation_id);
      CHECK(f.exec.admissionDiagnostics().angular_brakes_started==angular_starts);
      CHECK(source.trajectory.sample(now-source.splice_at_sec,expected));
    } else {
      CHECK(f.exec.admissionDiagnostics().angular_brakes_started==angular_starts+1);
      if(angular_id==0){angular_id=f.exec.telemetry().plan_id;CHECK(angular_id!=translation_id);}
      CHECK(f.exec.telemetry().plan_id==angular_id);
      CHECK(angular_stop.sample(now-angular_origin,expected));
      terminal=terminal||(f.exec.sample().angular_velocity_body.norm()==0.&&
                         f.exec.sample().angular_acceleration_body.norm()==0.);
    }
    CHECK(math::log3(math::rotationFromPose(f.exec.sample().pose).transpose()*math::rotationFromPose(expected.pose)).norm()<1e-10);
    CHECK((f.exec.sample().angular_velocity_body-expected.angular_velocity_body).norm()<1e-9);
    CHECK((f.exec.sample().angular_acceleration_body-expected.angular_acceleration_body).norm()<1e-7);
    CHECK(f.accept(out));
  }
  CHECK(angular_id!=0&&terminal);return true;
}
bool forceFoldIsTransportedLikeGeometry() {
  // 2026-09-07: a force fold is "where the arm was actually sent", the same rigid
  // shift the follower and the output SMD took; the in-flight result is transported
  // into the new gauge and admitted, the authority revision does not move and no
  // staged work is cancelled. (Before: gate_mismatch, expiry, a 10 s brake.)
  setExternalSteadyNs(kStartNs);Fixture f;
  f.step();letWorkerRun(); // Result computed before the fold.
  const auto gate=f.exec.telemetry().gate_revision;
  f.exec.shiftCommonFrame(Eigen::Vector3d{.0001,0,0},Eigen::Quaterniond::Identity(),PreviewFoldCause::Force);
  CHECK(f.raw.absorbOffset({.0001,0,0},Eigen::Quaterniond::Identity()));f.accepted.x+=.0001;
  auto out=f.step();CHECK(!out.fault);
  CHECK(f.exec.telemetry().gate_revision==gate);
  CHECK(f.exec.telemetry().result_checks[static_cast<std::size_t>(PreviewExecutionAcceptance::GateMismatch)]==0);
  CHECK(f.exec.telemetry().result_checks[static_cast<std::size_t>(PreviewExecutionAcceptance::Ready)]==1);
  CHECK(f.exec.telemetry().result_gauge_transported==1);
  CHECK(f.exec.telemetry().fold_force_count==1&&f.exec.telemetry().staged_cancel_counts[0]==0);
  CHECK(std::string(f.exec.telemetry().last_admission_reason)=="ready");
  return true;
}
bool authorityFoldAndResetCancellationAreAccounted() {
  setExternalSteadyNs(kStartNs);Fixture f;
  f.step();letWorkerRun(); // Result computed before authority changes.
  // Untagged: since 2026-09-07 the only fold that still breaks authority.
  f.exec.shiftCommonFrame(Eigen::Vector3d{.0001,0,0},Eigen::Quaterniond::Identity(),PreviewFoldCause::Unknown);
  CHECK(f.raw.absorbOffset({.0001,0,0},Eigen::Quaterniond::Identity()));f.accepted.x+=.0001;
  auto out=f.step();CHECK(!out.fault);
  CHECK(f.exec.telemetry().result_checks[static_cast<std::size_t>(PreviewExecutionAcceptance::GateMismatch)]==1);
  CHECK(std::string(f.exec.telemetry().last_admission_reason)=="gate_mismatch");
  // A fresh request in the changed authority is allowed to stage, but a reset
  // cancels it explicitly before its future splice rather than losing it.
  while(f.exec.telemetry().submitted<2) {letWorkerRun();out=f.step();CHECK(!out.fault);}
  letWorkerRun();out=f.step();CHECK(!out.fault);
  CHECK(f.exec.telemetry().result_checks[static_cast<std::size_t>(PreviewExecutionAcceptance::Ready)]==1);
  const auto before=f.exec.telemetry().rejected;f.exec.reset("init_motion");
  CHECK(f.exec.telemetry().staged_cancel_counts[1]==1);
  CHECK(std::string(f.exec.telemetry().last_staged_cancel_reason)=="reset");
  CHECK(f.exec.telemetry().rejected==before+1);
  CHECK(f.exec.telemetry().gauge_revision==0);
  CHECK(f.exec.telemetry().worker_status_counts[static_cast<std::size_t>(PreviewExecutionWorkerStatus::Solved)]==2);
  return true;
}

bool geometryFoldCannotTransportReplacedSource() {
  setExternalSteadyNs(kStartNs);Fixture f;f.step();letWorkerRun();f.step();
  CHECK(f.exec.telemetry().result_checks[static_cast<std::size_t>(PreviewExecutionAcceptance::Ready)]==1);
  const auto authority=f.exec.telemetry().gate_revision;
  f.raw.submitDeltaFrame(frame(18,9),f.raw.lastPose());
  const Eigen::Vector3d dp{.0001,0,0};const auto q=Eigen::Quaterniond::Identity();
  CHECK(f.raw.absorbOffset(dp,q));f.exec.shiftCommonFrame(dp,q,PreviewFoldCause::GeometryHold);
  f.accepted.x+=dp.x();const auto out=f.step();CHECK(!out.fault);
  CHECK(f.exec.telemetry().gate_revision==authority);
  CHECK(f.exec.telemetry().staged_cancel_counts[2]==1);
  CHECK(f.exec.telemetry().staged_gauge_transported==0);
  CHECK(std::string(f.exec.telemetry().last_staged_cancel_reason)=="source");
  return true;
}

bool geometryFoldsTransportPendingStagedAndQueuedDispatch() {
  setExternalSteadyNs(kStartNs);Fixture f(.001,.001);
  f.step(); // The first request is in flight in the original gauge.
  const auto authority=f.exec.telemetry().gate_revision;
  std::uint64_t source_wire=f.raw.windowWireSeq(),source_recv=f.raw.windowRecvSeq();
  Eigen::Vector3d total_dp=Eigen::Vector3d::Zero();Eigen::Quaterniond total_q=Eigen::Quaterniond::Identity();
  bool saw_pending=false,saw_staged=false,saw_splice=false;
  for(int i=0;i<70;++i) {
    // Alternate noncommuting rotations and stand translations. The same fold
    // moves the canonical follower and every nominal predecessor together.
    const Eigen::Vector3d dp{i%2?.00003:-.00002,.00001,-.000004};
    const Eigen::Quaterniond q(math::exp3(Eigen::Vector3d{i%2?.0001:0.,i%2?0.:.00013,.00002}));
    const auto before=f.exec.sample();
    const auto queued=f.exec.transaction(before.pose,before.pose);
    CHECK(f.raw.absorbOffset(dp,q));
    f.exec.shiftCommonFrame(dp,q,PreviewFoldCause::GeometryHold,kStartNs+(f.tick-1)*kDtNs,kStartNs+f.tick*kDtNs,2);
    total_dp+=dp;total_q=(q*total_q).normalized();
    CHECK(f.exec.telemetry().gate_revision==authority);
    CHECK(f.raw.windowWireSeq()==source_wire&&f.raw.windowRecvSeq()==source_recv);
    const auto shifted=f.exec.sample();auto expected=math::se3FromPose(before.pose);
    expected.translation()+=dp;expected.rotation()=q*expected.rotation();
    CHECK(math::positionDistance(shifted.pose,math::poseFromSe3(expected))<1e-12);
    CHECK(math::orientationDistanceRad(shifted.pose,math::poseFromSe3(expected))<1e-10);
    CHECK((shifted.linear_velocity-before.linear_velocity).norm()==0);
    CHECK((shifted.linear_acceleration-before.linear_acceleration).norm()==0);
    CHECK((shifted.angular_velocity_body-before.angular_velocity_body).norm()==0);
    CHECK((shifted.angular_acceleration_body-before.angular_acceleration_body).norm()==0);
    if(queued.valid) {
      // A command selected before this fold may still be enqueued now. Its own
      // nominal/composed pair is checked first, then its accepted seed is moved.
      CHECK(f.exec.observeDispatch(queued,queued.composed,true,.002,.01));
    }
    f.accepted=shifted.pose;
    const auto previous_plan=f.exec.telemetry().plan_id;
    letWorkerRun();const auto out=f.step();CHECK(!out.fault);CHECK(!f.exec.braking());
    if(previous_plan&&f.exec.telemetry().plan_id!=previous_plan) {
      const auto after=f.exec.sample();
      const auto expected_v=shifted.linear_velocity+kDt*shifted.linear_acceleration+.5*kDt*kDt*shifted.linear_jerk;
      const auto expected_a=shifted.linear_acceleration+kDt*shifted.linear_jerk;
      CHECK((after.linear_velocity-expected_v).norm()<1e-8);
      CHECK((after.linear_acceleration-expected_a).norm()<1e-6);saw_splice=true;
    }
    CHECK(f.accept(out));
    saw_pending=saw_pending||f.exec.telemetry().result_gauge_transported>0;
    saw_staged=saw_staged||f.exec.telemetry().staged_gauge_transported>0;
  }
  const auto& t=f.exec.telemetry();
  CHECK(saw_pending&&saw_staged&&saw_splice);CHECK(t.accepted>=10);
  CHECK(t.expired==0&&t.gauge_transport_failed==0&&t.staged_cancel_counts[0]==0);
  CHECK(t.fold_geometry_hold_count==70&&t.gauge_revision==70);
  for(int axis=0;axis<3;++axis)CHECK(std::abs(t.gauge_translation_m[axis]-total_dp[axis])<1e-12);
  for(int axis=0;axis<4;++axis)CHECK(std::abs(t.gauge_quaternion_xyzw[axis]-total_q.coeffs()[axis])<1e-12);
  CHECK(t.fold_geometry_cause_mask==2&&t.fold_booked_time_ns<t.fold_applied_time_ns);
  // Every TAGGED fold is a transport (2026-09-07): force and ROI/floor folds move
  // the gauge without touching authority, exactly like the geometry hold. Only an
  // untagged correction still cancels pending/staged work.
  const auto previous_gate=t.gate_revision;
  f.exec.shiftCommonFrame(Eigen::Vector3d::Zero(),Eigen::Quaterniond::Identity(),PreviewFoldCause::Force);
  CHECK(f.exec.telemetry().gate_revision==previous_gate);
  f.exec.shiftCommonFrame(Eigen::Vector3d::Zero(),Eigen::Quaterniond::Identity(),PreviewFoldCause::RoiFloor);
  CHECK(f.exec.telemetry().gate_revision==previous_gate);
  f.exec.shiftCommonFrame(Eigen::Vector3d::Zero(),Eigen::Quaterniond::Identity());
  CHECK(f.exec.telemetry().gate_revision==previous_gate+1);
  return true;
}

bool firstPlanStarvationRecoversWithoutLatch() {
  setExternalSteadyNs(kStartNs);Fixture f(0.,0.,true);
  auto out=f.step();CHECK(!out.active&&!out.fault);
  // Every response belongs to a source replaced before the servo can read it.
  // This makes first-plan starvation causal and independent of worker speed.
  for(int i=0;i<40&&!f.exec.recovering();++i) {
    letWorkerRun();f.raw.submitDeltaFrame(frame(100+i,200+i,0.),f.raw.lastPose());
    out=f.step();CHECK(!out.fault);
  }
  CHECK(f.exec.recovering()&&!f.exec.failed()&&out.active);
  CHECK(f.exec.recoveryCause()==PreviewRecoveryCause::FirstPlanTimeout);
  CHECK(f.exec.telemetry().accepted==0&&f.exec.telemetry().expired==1);
  CHECK(!f.exec.telemetry().active&&f.exec.telemetry().backlog_sec==0.);
  CHECK(!f.exec.recoveryStopped());CHECK(!f.exec.restartRecovery());
  const auto origin=f.exec.telemetry().last_brake_origin_sec;
  const auto plan=f.exec.telemetry().plan_id;
  const auto gate=f.exec.telemetry().gate_revision;
  CHECK(f.exec.requestRecovery(PreviewRecoveryCause::Peer));
  CHECK(f.exec.telemetry().last_brake_origin_sec==origin&&f.exec.telemetry().plan_id==plan);
  CHECK(f.exec.telemetry().gate_revision==gate);
  CHECK(f.exec.recoveryCause()==PreviewRecoveryCause::FirstPlanTimeout);
  CHECK(f.accept(out));CHECK(f.exec.recoveryStopped());
  const auto terminal=f.exec.acceptedSample();const auto epoch=f.exec.telemetry().epoch;
  CHECK(f.exec.restartRecovery());CHECK(!f.exec.recovering()&&!f.exec.failed());
  CHECK(f.exec.telemetry().epoch>epoch&&!f.exec.initialized());
  f.raw.deactivate();f.raw.submitDeltaFrame(frame(900,901,0.),terminal.pose);
  f.accepted.x+=.017; // A different readback must not overwrite the accepted stop seed.
  out=f.step();CHECK(!out.fault&&!out.active);CHECK(sameMotion(f.exec.sample(),terminal));
  letWorkerRun();CHECK(f.engage());CHECK(f.exec.telemetry().active);
  return true;
}

bool stationaryExpiredPlanRecoversWithoutBacklog() {
  setExternalSteadyNs(kStartNs);Fixture f(0.,0.,true);CHECK(f.engage());
  const auto admitted=f.exec.telemetry().accepted;
  bool expired=false;
  for(int i=0;i<90&&!f.exec.recovering();++i) {
    letWorkerRun();f.raw.submitDeltaFrame(frame(1000+i,2000+i,0.),f.raw.lastPose());
    const auto out=f.step();CHECK(!out.fault);CHECK(f.accept(out));
    expired=expired||f.exec.telemetry().expired>0;
    CHECK(f.exec.telemetry().backlog_sec==0.);
  }
  CHECK(expired&&f.exec.recovering()&&!f.exec.failed());
  CHECK(f.exec.recoveryCause()==PreviewRecoveryCause::PlanExpired);
  CHECK(f.exec.telemetry().accepted==admitted);
  CHECK(f.exec.recoveryStopped()&&!f.exec.telemetry().active);
  return true;
}

bool recoveryPreservesBrakeAndDispatchedTerminal() {
  setExternalSteadyNs(kStartNs);Fixture f(.006,.006,true);CHECK(f.engage());
  PreviewBrake expected(config().preview_execution.tracker,kDt);bool moving=false;
  for(int i=0;i<80&&!moving;++i) {
    letWorkerRun();const auto out=f.step();CHECK(!out.fault&&out.active);CHECK(f.accept(out));
    moving=f.exec.acceptedSample().linear_velocity.norm()>.02&&
        f.exec.acceptedSample().angular_velocity_body.norm()>.01&&
        expected.start(f.exec.acceptedSample())==PreviewBrakeStatus::Ready&&expected.durationSec()>.006;
  }
  CHECK(moving);const auto accepted=f.exec.acceptedSample();const double origin=f.now()-kDt;
  // The next optimizer sample is deliberately not dispatched. Recovery must
  // seed the last accepted command, not this newer unaccepted proposal.
  letWorkerRun();const auto proposal=f.step();CHECK(!proposal.fault&&proposal.active);
  CHECK(sameMotion(f.exec.acceptedSample(),accepted));
  CHECK(f.exec.requestRecovery(PreviewRecoveryCause::Backlog));
  CHECK(f.exec.telemetry().last_brake_origin_sec==origin);
  const auto plan=f.exec.telemetry().plan_id;const auto gate=f.exec.telemetry().gate_revision;
  bool terminal=false;PreviewMotionSample accepted_terminal;
  for(int i=0;i<100;++i) {
    CHECK(f.exec.requestRecovery(PreviewRecoveryCause::Peer));
    CHECK(f.exec.telemetry().plan_id==plan&&f.exec.telemetry().gate_revision==gate);
    CHECK(f.exec.telemetry().last_brake_origin_sec==origin);
    CHECK(f.exec.recoveryCause()==PreviewRecoveryCause::Backlog);
    const auto out=f.step();CHECK(out.active&&!out.fault&&!f.exec.failed());
    CHECK(!f.exec.telemetry().active);
    PreviewMotionSample original;const double elapsed=f.now()-kDt-origin;
    CHECK(expected.sample(elapsed,original));CHECK(sameMotion(f.exec.sample(),original));
    if(elapsed>=expected.durationSec()) {
      CHECK(!f.exec.recoveryStopped());CHECK(!f.exec.restartRecovery());
      // Reaching the terminal time alone cannot authorize a new epoch.
      CHECK(f.accept(out));CHECK(f.exec.recoveryStopped());
      accepted_terminal=f.exec.acceptedSample();terminal=true;break;
    }
    CHECK(!f.exec.recoveryStopped());CHECK(!f.exec.restartRecovery());CHECK(f.accept(out));
  }
  CHECK(terminal);const auto epoch=f.exec.telemetry().epoch;
  CHECK(f.exec.restartRecovery());CHECK(f.exec.telemetry().epoch>epoch);
  f.raw.deactivate();f.raw.submitDeltaFrame(frame(3000,4000,0.),accepted_terminal.pose);
  const auto fresh=f.step();CHECK(!fresh.fault&&!fresh.active);
  CHECK(sameMotion(f.exec.sample(),accepted_terminal));
  CHECK(f.exec.sample().linear_velocity.isZero(0.)&&f.exec.sample().linear_acceleration.isZero(0.));
  CHECK(f.exec.sample().angular_velocity_body.isZero(0.)&&f.exec.sample().angular_acceleration_body.isZero(0.));
  return true;
}

bool recoveryDoesNotDowngradeSafetyFailures() {
  {
    setExternalSteadyNs(kStartNs);Fixture f(.001,0.,true);CHECK(f.engage());
    CHECK(f.exec.requestRecovery(PreviewRecoveryCause::Peer));
    const auto tx=f.exec.transaction(f.exec.sample().pose,f.exec.sample().pose);CHECK(tx.valid);
    CHECK(!f.exec.observeDispatch(tx,tx.composed,false,.002,.01));
    CHECK(f.exec.failed()&&std::string(f.exec.telemetry().status)=="dispatch_rejected");
    CHECK(!f.exec.restartRecovery());
  }
  {
    setExternalSteadyNs(kStartNs);Fixture f(0.,0.,true);f.accepted.x=std::numeric_limits<double>::quiet_NaN();
    const auto out=f.step();CHECK(out.fault&&f.exec.failed());
    CHECK(std::string(f.exec.telemetry().status)=="invalid_input");
  }
  {
    setExternalSteadyNs(kStartNs);Fixture f(0.,0.,true);CHECK(f.engage());
    CHECK(f.exec.requestRecovery(PreviewRecoveryCause::Peer));
    const auto out=f.exec.recoveryOutput(std::numeric_limits<double>::quiet_NaN());
    CHECK(out.fault&&f.exec.failed());CHECK(!f.exec.restartRecovery());
    CHECK(std::string(f.exec.telemetry().status)=="invalid_recovery_state");
  }
  return true;
}

bool stationaryRecoverySeedRefusalsAreTransactional() {
  {
    setExternalSteadyNs(kStartNs);Fixture f(0.,0.,false);
    const auto before=f.exec.sample();const auto epoch=f.exec.telemetry().epoch;
    CHECK(!f.exec.seedStationaryRecovery(f.now(),pose(.43),true,PreviewRecoveryCause::Peer));
    CHECK(!f.exec.initialized()&&!f.exec.recovering()&&!f.exec.hasPlan()&&!f.exec.failed());
    CHECK(f.exec.telemetry().epoch==epoch&&sameMotion(f.exec.sample(),before));
  }
  {
    setExternalSteadyNs(kStartNs);Fixture f(0.,0.,true);
    const auto before=f.exec.sample();const auto epoch=f.exec.telemetry().epoch;
    const double upper_time=static_cast<double>(UINT64_MAX)/1e9;
    auto invalid_pose=pose(.43);invalid_pose.rz=std::numeric_limits<double>::quiet_NaN();
    CHECK(!f.exec.seedStationaryRecovery(f.now(),pose(.43),true,PreviewRecoveryCause::None));
    CHECK(!f.exec.seedStationaryRecovery(f.now(),pose(.43),false,PreviewRecoveryCause::Peer));
    CHECK(!f.exec.seedStationaryRecovery(f.now(),invalid_pose,true,PreviewRecoveryCause::Peer));
    for(const double time:{0.,-1.,upper_time,std::numeric_limits<double>::max(),
                          std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
      CHECK(!f.exec.seedStationaryRecovery(time,pose(.43),true,PreviewRecoveryCause::Peer));
      CHECK(!f.exec.initialized()&&!f.exec.recovering()&&!f.exec.hasPlan()&&!f.exec.failed());
      CHECK(f.exec.telemetry().epoch==epoch&&sameMotion(f.exec.sample(),before));
    }
    // Refused cold seeds do not poison the subsequent valid stationary seed.
    CHECK(f.exec.seedStationaryRecovery(f.now(),pose(.43),true,PreviewRecoveryCause::Peer));
    CHECK(f.exec.initialized()&&f.exec.recovering()&&!f.exec.failed());
    CHECK(math::positionDistance(f.exec.sample().pose,pose(.43))==0.);
    const auto seeded=f.exec.sample();const auto plan=f.exec.telemetry().plan_id;
    const auto gate=f.exec.telemetry().gate_revision;
    CHECK(!f.exec.seedStationaryRecovery(f.now(),pose(.47),true,PreviewRecoveryCause::Backlog));
    CHECK(sameMotion(f.exec.sample(),seeded)&&f.exec.telemetry().plan_id==plan);
    CHECK(f.exec.telemetry().gate_revision==gate&&f.exec.recoveryCause()==PreviewRecoveryCause::Peer);
  }
  return true;
}

bool recoveryOutputRejectsNanosecondOverflowBoundary() {
  setExternalSteadyNs(kStartNs);Fixture f(0.,0.,true);
  const double upper_time=static_cast<double>(UINT64_MAX)/1e9;
  const double last_valid=std::nextafter(upper_time,0.);
  // Start immediately below the clock boundary so the ordinary tick-gap fence
  // cannot accidentally cover a missing upper-time guard.
  CHECK(upper_time-last_valid<config(true).preview_execution.max_result_age_sec);
  CHECK(f.exec.seedStationaryRecovery(last_valid,pose(),true,PreviewRecoveryCause::Peer));
  const auto valid=f.exec.recoveryOutput(last_valid);CHECK(valid.active&&!valid.fault);
  const auto rejected=f.exec.recoveryOutput(upper_time);
  CHECK(rejected.fault&&!rejected.active&&f.exec.failed());
  CHECK(std::string(f.exec.telemetry().status)=="invalid_recovery_state");
  CHECK(!f.exec.restartRecovery());
  return true;
}

bool recoverySeedSurvivesWaitingForStationaryDispatch() {
  setExternalSteadyNs(kStartNs);Fixture f(0.,0.,true);
  auto anchor=pose(.43);anchor.rz=.03;
  CHECK(f.exec.seedStationaryRecovery(f.now(),anchor,true,PreviewRecoveryCause::Peer));
  const auto held=f.exec.recoveryOutput(f.now());CHECK(held.active&&!held.fault);
  CHECK(f.accept(held)&&f.exec.recoveryStopped());
  const auto old_transaction=f.exec.transaction(held.pose,held.pose);
  const auto terminal=f.exec.acceptedSample();CHECK(f.exec.restartRecovery());
  f.accepted=pose(.47); // FK/readback residual cannot replace accepted terminal p/v/a.
  f.raw.deactivate();
  for(int i=0;i<3;++i) {
    const auto waiting=f.step();CHECK(!waiting.active&&!waiting.fault&&!f.exec.initialized());
    CHECK(math::positionDistance(waiting.pose,terminal.pose)==0.);
    CHECK(math::orientationDistanceRad(waiting.pose,terminal.pose)<1e-12);
    CHECK(math::positionDistance(f.exec.recoveryAnchorPose(),terminal.pose)==0.);
  }
  f.raw.submitDeltaFrame(frame(7000,7001,0.),terminal.pose);
  f.raw.pauseForHold(f.now());CHECK(f.raw.holdPaused());
  const auto paused=f.step();CHECK(!paused.active&&!paused.fault&&!f.exec.initialized());
  CHECK(math::positionDistance(paused.pose,terminal.pose)==0.);
  f.raw.deactivate();f.raw.submitDeltaFrame(frame(7002,7003,0.),terminal.pose);
  for(int i=0;i<3;++i) {
    const auto waiting=f.step(false);CHECK(!waiting.active&&!waiting.fault&&!f.exec.initialized());
    CHECK(math::positionDistance(waiting.pose,terminal.pose)==0.);
    CHECK(math::positionDistance(f.exec.recoveryAnchorPose(),terminal.pose)==0.);
    CHECK(math::orientationDistanceRad(f.exec.recoveryAnchorPose(),terminal.pose)<1e-12);
  }
  f.raw.setPlanRateGate(0.); // Shared Starting freezes the reference until both arms are ready.
  const auto first=f.step();CHECK(!first.active&&!first.fault&&f.exec.initialized());
  CHECK(sameMotion(f.exec.sample(),terminal));
  CHECK(math::positionDistance(first.pose,terminal.pose)==0.);
  CHECK(math::orientationDistanceRad(first.pose,terminal.pose)<1e-12);
  bool proposed=false;
  for(int i=0;i<20;++i) {
    letWorkerRun();const auto output=f.step();CHECK(!output.fault&&!f.exec.telemetry().active);
    CHECK(sameMotion(f.exec.sample(),terminal));
    if(!output.active)continue;
    // A previous epoch's accepted brake receipt cannot open the new barrier.
    CHECK(f.exec.observeDispatch(old_transaction,old_transaction.composed,true,.002,.01));
    CHECK(!f.exec.telemetry().active);
    CHECK(f.accept(output)&&f.exec.telemetry().active);proposed=true;break;
  }
  CHECK(proposed);
  return true;
}

bool explicitResetCancelsPendingRecoverySeed() {
  setExternalSteadyNs(kStartNs);Fixture f(0.,0.,true);
  CHECK(f.exec.seedStationaryRecovery(f.now(),pose(.43),true,PreviewRecoveryCause::Peer));
  const auto held=f.exec.recoveryOutput(f.now());CHECK(held.active&&!held.fault);
  CHECK(f.accept(held)&&f.exec.recoveryStopped()&&f.exec.restartRecovery());
  f.exec.reset("profile_exit");f.accepted=pose(.47);
  f.raw.deactivate();f.raw.submitDeltaFrame(frame(8000,8001,0.),f.accepted);
  const auto fresh=f.step();CHECK(!fresh.active&&!fresh.fault&&f.exec.initialized());
  CHECK(math::positionDistance(fresh.pose,f.accepted)==0.);
  CHECK(math::positionDistance(f.exec.sample().pose,f.accepted)==0.);
  return true;
}

bool pendingRecoverySeedSharesGeometricFold() {
  setExternalSteadyNs(kStartNs);Fixture f(0.,0.,true);
  auto anchor=pose(.43);anchor.rz=.03;
  CHECK(f.exec.seedStationaryRecovery(f.now(),anchor,true,PreviewRecoveryCause::Peer));
  const auto held=f.exec.recoveryOutput(f.now());CHECK(held.active&&!held.fault);
  CHECK(f.accept(held)&&f.exec.recoveryStopped());
  auto expected=f.exec.acceptedSample();CHECK(f.exec.restartRecovery());
  f.raw.deactivate();f.raw.submitDeltaFrame(frame(9000,9001,0.),expected.pose);
  const Eigen::Vector3d dp{.001,-.002,.0005};
  const Eigen::Quaterniond dR(math::exp3(Eigen::Vector3d{.04,.02,-.03}));
  auto transformed=math::se3FromPose(expected.pose);
  transformed.translation()+=dp;transformed.rotation()=dR*transformed.rotation();
  expected.pose=math::poseFromSe3(transformed);
  // Native geometry may book its common frame after the new raw anchor exists
  // but before the first preview step initializes the new execution epoch.
  CHECK(f.raw.absorbOffset(dp,dR));
  ++f.tick;setExternalSteadyNs(kStartNs+kDtNs);
  const auto folds=f.exec.telemetry().fold_geometry_hold_count;
  f.exec.shiftCommonFrame(dp,dR,PreviewFoldCause::GeometryHold,kStartNs,kStartNs+kDtNs,2);
  CHECK(!f.exec.initialized()&&!f.exec.failed());
  CHECK(math::positionDistance(f.exec.recoveryAnchorPose(),expected.pose)<1e-12);
  CHECK(math::orientationDistanceRad(f.exec.recoveryAnchorPose(),expected.pose)<1e-12);
  CHECK(f.exec.telemetry().fold_geometry_hold_count==folds+1);
  CHECK(f.exec.telemetry().gauge_revision==1&&f.exec.telemetry().fold_geometry_cause_mask==2);
  CHECK(f.exec.telemetry().fold_booked_time_ns==kStartNs&&
        f.exec.telemetry().fold_applied_time_ns==kStartNs+kDtNs);
  f.raw.setPlanRateGate(0.);
  const auto first=f.step();CHECK(!first.active&&!first.fault&&f.exec.initialized());
  CHECK(sameMotion(f.exec.sample(),expected));
  CHECK(math::positionDistance(first.pose,expected.pose)<1e-12);
  letWorkerRun();CHECK(f.engage());
  CHECK(sameMotion(f.exec.acceptedSample(),expected));
  CHECK(f.exec.telemetry().gauge_revision==1&&f.exec.telemetry().active);
  return true;
}

bool recordedAngularExpiryStateHasFiniteBrake() {
  // 214623 LEFT phase-0 replay: last accepted tick 1605519767005249
  // (t=22.792005348252133), immediately before braking_expired. Fixture:
  // outputs/preview_recovery_redesign_20260907/214623_first_expiry_previous_accepted.json.
  // No contact or common-frame fold was active in this captured interval.
  PreviewMotionState initial;
  const Eigen::Quaterniond rotation(.0551202148002129,.8100621927610276,
                                    .5505109656161757,-.194161485665721);
  initial.pose=math::poseFromSe3(pinocchio::SE3(rotation.normalized().toRotationMatrix(),
      Eigen::Vector3d{.4249954211115257,.081616673815217,-.1998582424954639}));
  initial.linear_velocity={.0022494897871945,-.0010306031914868,.0074318886980092};
  initial.linear_acceleration={.6219668921986238,-.0728622416362507,.4135063107990034};
  initial.angular_velocity_body={.1144594572847579,.0447025387041554,.3439951551857169};
  initial.angular_acceleration_body={-2.769263637670227,-.2849283236274891,-8.141883016311898};
  const auto limits=config(true).preview_execution.tracker;
  PreviewBrake brake(limits,kDt);CHECK(brake.start(initial)==PreviewBrakeStatus::Ready);
  PreviewMotionSample first,terminal,held;
  CHECK(brake.sample(0.,first)&&sameMotion(first,initial));
  CHECK(brake.durationSec()>0.&&brake.durationSec()<config(true).preview_execution.max_result_age_sec);
  CHECK(brake.sample(brake.durationSec(),terminal));
  CHECK(terminal.linear_velocity.isZero(0.)&&terminal.linear_acceleration.isZero(0.));
  CHECK(terminal.angular_velocity_body.isZero(0.)&&terminal.angular_acceleration_body.isZero(0.));
  const double precision=limits.feasibility_tolerance;
  for(int k=0;k<=4000;++k) {
    PreviewMotionSample sample;CHECK(brake.sample(brake.durationSec()*k/4000.,sample));
    CHECK(sample.linear_velocity.cwiseAbs().maxCoeff()<=limits.max_linear_velocity_m_s+precision);
    CHECK(sample.linear_acceleration.cwiseAbs().maxCoeff()<=limits.max_linear_acceleration_m_s2+precision);
    CHECK(sample.linear_jerk.cwiseAbs().maxCoeff()<=limits.max_linear_jerk_m_s3+precision);
    CHECK(sample.angular_velocity_body.norm()<=limits.max_angular_velocity_rad_s+precision);
    CHECK(sample.angular_acceleration_body.norm()<=limits.max_angular_acceleration_rad_s2+precision);
    CHECK(sample.angular_jerk_stand.norm()<=limits.max_angular_jerk_rad_s3+precision);
  }
  CHECK(brake.sample(brake.durationSec()+1.,held)&&sameMotion(held,terminal));
  std::cout<<"recorded angular expiry brake: duration ms="<<brake.durationSec()*1e3
           <<" displacement um="<<math::positionDistance(initial.pose,terminal.pose)*1e6
           <<" rotation rad="<<math::orientationDistanceRad(initial.pose,terminal.pose)<<'\n';
  return true;
}

}
bool stationarySeedRefusalNamesThePredicate() {
  // 2026-09-10: a refused restart must name its precondition (the same-process
  // restart fault reported only "cannot certify a stop").
  {
    setExternalSteadyNs(kStartNs);Fixture f(0.,0.,false);
    CHECK(std::string(f.exec.stationarySeedRefusal(f.now(),pose(.43),true,PreviewRecoveryCause::Peer))=="recovery_disabled");
  }
  {
    setExternalSteadyNs(kStartNs);Fixture f(0.,0.,true);
    auto invalid_pose=pose(.43);invalid_pose.rz=std::numeric_limits<double>::quiet_NaN();
    CHECK(std::string(f.exec.stationarySeedRefusal(f.now(),pose(.43),true,PreviewRecoveryCause::None))=="cause_none");
    CHECK(std::string(f.exec.stationarySeedRefusal(f.now(),pose(.43),false,PreviewRecoveryCause::Peer))=="not_stationary");
    CHECK(std::string(f.exec.stationarySeedRefusal(f.now(),invalid_pose,true,PreviewRecoveryCause::Peer))=="nominal_not_finite");
    CHECK(std::string(f.exec.stationarySeedRefusal(-1.,pose(.43),true,PreviewRecoveryCause::Peer))=="invalid_time");
    CHECK(f.exec.stationarySeedRefusal(f.now(),pose(.43),true,PreviewRecoveryCause::Peer)==nullptr);
    // the query has no side effects: the seed still succeeds afterwards, and then names itself initialized
    CHECK(f.exec.seedStationaryRecovery(f.now(),pose(.43),true,PreviewRecoveryCause::Peer));
    CHECK(std::string(f.exec.stationarySeedRefusal(f.now(),pose(.43),true,PreviewRecoveryCause::Peer))=="already_initialized");
  }
  return true;
}

bool sentJointStationarityUsesTolerance() {
  using rb_servo::control::maxAbsJointDeltaDeg;
  using rb_servo::control::sentJointsStationary;
  using rb_servo::control::kSentJointStationaryToleranceDeg;
  const JointArray a{-64.8019149,62.5795989,86.6957974,10.,-20.,30.};
  JointArray b=a;
  CHECK(sentJointsStationary(a,b));
  b[2]+=1e-9;                      // the hold-ramp tail seen in the 2026-09-10 logs
  CHECK(maxAbsJointDeltaDeg(a,b)<1e-8&&sentJointsStationary(a,b));
  b[2]=a[2]+kSentJointStationaryToleranceDeg*0.5;
  CHECK(sentJointsStationary(a,b));
  b[2]=a[2]+1e-3;                  // a real 0.5 deg/s motion at 2 ms is not stationary
  CHECK(!sentJointsStationary(a,b)&&std::abs(maxAbsJointDeltaDeg(a,b)-1e-3)<1e-12);
  b[4]=std::numeric_limits<double>::quiet_NaN();
  CHECK(!sentJointsStationary(a,b)&&!std::isfinite(maxAbsJointDeltaDeg(a,b)));
  return true;
}

int main() {
  const bool ok=coldAndC2Splice()&&epochsAndContinuousGateIdentity()&&acceptedTransactionGaugeAndDeviation()&&
      frameShiftAndCanonicalIndependence()&&expiryAndDispatchRefusal()&&invalidInputAndContactStop()&&
      oldAcceptedTransactionAcrossFoldSeedsBrake()&&currentVelocityAuthority()&&coldRetreatAuthority()&&
      rejectedStagedPlanRetainsBrakeClock()&&contactRetainsAngularUntilOriginalExpiry()&&
      geometryFoldsTransportPendingStagedAndQueuedDispatch()&&authorityFoldAndResetCancellationAreAccounted()&&forceFoldIsTransportedLikeGeometry()&&
      geometryFoldCannotTransportReplacedSource()&&firstPlanStarvationRecoversWithoutLatch()&&
      stationaryExpiredPlanRecoversWithoutBacklog()&&recoveryPreservesBrakeAndDispatchedTerminal()&&
      recoveryDoesNotDowngradeSafetyFailures()&&stationaryRecoverySeedRefusalsAreTransactional()&&
      recoveryOutputRejectsNanosecondOverflowBoundary()&&recoverySeedSurvivesWaitingForStationaryDispatch()&&
      explicitResetCancelsPendingRecoverySeed()&&pendingRecoverySeedSharesGeometricFold()&&
      recordedAngularExpiryStateHasFiniteBrake()&&stationarySeedRefusalNamesThePredicate()&&
      sentJointStationarityUsesTolerance()&&contactClampFallsBackToBrakeOnlyWhenNoReplanArrives();
  setExternalSteadyNs(0);
  if(!ok)return 1;
  std::cout<<"live preview execution: all checks passed (no hardware)\n";return 0;
}
