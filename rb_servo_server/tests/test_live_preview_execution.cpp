#include "rb_servo/control/live_preview_execution.hpp"
#include "rb_servo/core/clock.hpp"
#include "rb_servo/math/se3.hpp"
#include <chrono>
#include <cmath>
#include <iostream>
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
RuckigFollowerConfig config() {
  RuckigFollowerConfig c;c.enable=true;c.controller=RuckigFollowerController::DeltaPreview;
  c.fresh_chunk_replan=true;c.continuous_hold_resume=true;
  c.plan_leash_enable=true;c.plan_leash_start_m=.01;c.plan_leash_start_rad=.0349;
  c.plan_leash_full_m=.05;c.plan_leash_full_rad=.1;c.plan_leash_min_gate=.25;
  auto& p=c.preview_execution;p.enable=true;p.replan_period_sec=.01;p.splice_lead_sec=.01;
  p.max_result_age_sec=.05;p.worker_poll_period_sec=.0005;p.max_source_rows=32;
  p.cursor={true,.1,.2,1.1,1e-6,1e-6};
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
  LivePreviewExecution exec{config(),rawConfig(),kDt};
  Pose6D accepted{pose()};std::uint64_t tick{0};
  explicit Fixture(double delta=.001,double angular_delta=0.) {raw.submitDeltaFrame(frame(17,8,delta,angular_delta),accepted);}
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
  const auto stopped=f.step(true,.01,Eigen::Vector3d::UnitX());
  CHECK(f.raw.outputKinematics().velocity.x==0);
  CHECK(stopped.active&&!stopped.fault&&f.exec.braking());
  CHECK(f.exec.telemetry().contact_guard_count==1);
  PreviewMotionSample expected_tick;CHECK(expected.sample(kDt,expected_tick));
  CHECK(math::positionDistance(f.exec.sample().pose,expected_tick.pose)<1e-11);
  CHECK((f.exec.sample().linear_velocity-expected_tick.linear_velocity).norm()<1e-10);
  CHECK((f.exec.sample().linear_acceleration-expected_tick.linear_acceleration).norm()<1e-8);
  CHECK(f.accept(stopped));
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
bool authorityFoldAndResetCancellationAreAccounted() {
  setExternalSteadyNs(kStartNs);Fixture f;
  f.step();letWorkerRun(); // Result computed before authority changes.
  f.exec.shiftCommonFrame(Eigen::Vector3d{.0001,0,0},Eigen::Quaterniond::Identity(),PreviewFoldCause::Force);
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
  // Authority folds still cancel pending/staged work even when their numerical
  // transform is identical to a transported geometry fold.
  const auto previous_gate=t.gate_revision;
  f.exec.shiftCommonFrame(Eigen::Vector3d::Zero(),Eigen::Quaterniond::Identity(),PreviewFoldCause::Force);
  CHECK(f.exec.telemetry().gate_revision==previous_gate+1);
  f.exec.shiftCommonFrame(Eigen::Vector3d::Zero(),Eigen::Quaterniond::Identity(),PreviewFoldCause::RoiFloor);
  CHECK(f.exec.telemetry().gate_revision==previous_gate+2);
  f.exec.shiftCommonFrame(Eigen::Vector3d::Zero(),Eigen::Quaterniond::Identity());
  CHECK(f.exec.telemetry().gate_revision==previous_gate+3);
  return true;
}

}
int main() {
  const bool ok=coldAndC2Splice()&&epochsAndContinuousGateIdentity()&&acceptedTransactionGaugeAndDeviation()&&
      frameShiftAndCanonicalIndependence()&&expiryAndDispatchRefusal()&&invalidInputAndContactStop()&&
      oldAcceptedTransactionAcrossFoldSeedsBrake()&&currentVelocityAuthority()&&coldRetreatAuthority()&&
      rejectedStagedPlanRetainsBrakeClock()&&contactRetainsAngularUntilOriginalExpiry()&&
      geometryFoldsTransportPendingStagedAndQueuedDispatch()&&authorityFoldAndResetCancellationAreAccounted()&&
      geometryFoldCannotTransportReplacedSource();
  setExternalSteadyNs(0);
  if(!ok)return 1;
  std::cout<<"live preview execution: all checks passed (no hardware)\n";return 0;
}
