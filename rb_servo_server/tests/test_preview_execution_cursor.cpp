#include "rb_servo/control/preview_execution_cursor.hpp"
#include <iostream>
#include <limits>

using namespace rb_servo;
using namespace rb_servo::control;
namespace {
void check(bool ok,const char* message) {if(!ok)throw std::runtime_error(message);}
PreviewExecutionCursorConfig config() {return {.1,.2,1.1,1e-6,1e-6};}
PlanLeashParams leash() {PlanLeashParams l;l.start_m=.01;l.full_m=.05;l.start_rad=.0349;l.full_rad=.1;l.min_gate=.25;return l;}
FollowerOutputKinematics state(double t) {
  FollowerOutputKinematics s;
  s.pose=math::poseFromSe3(pinocchio::SE3(math::exp3({.1,.2,.3}),Eigen::Vector3d(t,0,0)));
  s.velocity.x=.1;s.velocity.rz=.2;return s;
}
void oneSidedLagAndRecovery() {
  PreviewExecutionCursor c(config(),leash());c.reset(1.);
  auto r=state(0.);auto ahead=r.pose;ahead.x+=.04;
  auto s=c.step(1.002,r,ahead);
  check(s.valid&&s.gate==1.&&s.backlog_sec==0.&&s.positive_lag_m==0.,"ahead output slowed reference phase");
  auto behind=r.pose;behind.x-=.04;s=c.step(1.004,r,behind);
  check(s.valid&&s.gate<1.&&s.backlog_sec>0.,"lag did not slow independent cursor");
  const double initial_backlog=s.backlog_sec;
  for(int k=1;k<=400;++k) {
    s=c.step(1.004+.002*k,r,r.pose);
    check(s.valid&&s.time_sec<=1.004+.002*k,"cursor advanced beyond canonical clock");
    check(previewCursorReferenceTime(1.004+.002*k,.24,s)<=1.004+.002*k+.24,"future catchup overshot canonical time");
  }
  check(s.backlog_sec<initial_backlog*.03,"phase delay failed to recover");
  auto cross=r.pose;cross.y+=.1;s=c.step(1.806,r,cross);
  check(s.valid&&s.gate==1.&&s.cross_track_m>.09,"cross-track error misreported as temporal lag");
  // A temporal governor is not the Cartesian safety gate: cross-track authority
  // remains with the optimizer and final safety checks, not this projection.
}
void rotationAndTransactionalFailure() {
  PreviewExecutionCursor c(config(),leash());c.reset(10.);
  auto r=state(0.);auto out=r.pose;
  out=math::poseFromSe3(pinocchio::SE3(math::rotationFromPose(r.pose)*math::exp3({0,0,.08}),Eigen::Vector3d::Zero()));
  auto s=c.step(10.002,r,out);check(s.valid&&s.positive_lag_rad<1e-12,"rotational lead sign is wrong");
  out=math::poseFromSe3(pinocchio::SE3(math::rotationFromPose(r.pose)*math::exp3({0,0,-.08}),Eigen::Vector3d::Zero()));
  s=c.step(10.004,r,out);check(s.valid&&s.positive_lag_rad>.079&&s.gate<1.,"rotational lag sign is wrong");
  auto tight=config();tight.max_backlog_sec=.001;PreviewExecutionCursor limited(tight,leash());limited.reset(20.);
  out=r.pose;out.x-=.1;s=limited.step(20.01,r,out);
  check(!s.valid&&s.status==PreviewExecutionCursorStatus::BacklogExceeded&&limited.timeSec()==20.,"backlog failure mutated or clipped phase");
  out=r.pose;out.quaternion_xyzw=std::array<double,4>{0,0,0,0};
  s=limited.step(20.01,r,out);check(!s.valid&&s.status==PreviewExecutionCursorStatus::InvalidInput,"invalid quaternion escaped input rejection");
  r.velocity.x=std::numeric_limits<double>::max();s=limited.step(20.01,r,state(0.).pose);
  check(!s.valid,"overflowing direction norm was accepted");
}
void finiteWindowPhaseAndReversal() {
  auto cfg=config();cfg.phase_lookahead_sec=.04;
  PreviewExecutionCursor old(config(),leash()),windowed(cfg,leash());old.reset(1.);windowed.reset(1.);
  PreviewExecutionPhaseWindow window;window.count=5;
  for(std::size_t k=0;k<window.count;++k) {
    const double t=.01*k;window.relative_time_sec[k]=t;
    auto& r=window.reference[k];auto& y=window.output[k];
    // Known future reference makes a brief reversal. The accepted output is a
    // separate smooth curve; all p/v/a samples are mutually consistent here.
    r.pose=math::poseFromSe3(pinocchio::SE3(math::exp3({0,0,.06+.2*t-10*t*t}),Eigen::Vector3d::Zero()));
    r.velocity.rz=.2-20*t;r.acceleration.rz=-20.;
    y.pose=math::poseFromSe3(pinocchio::SE3(math::exp3({0,0,-.1*t}),Eigen::Vector3d::Zero()));
    y.velocity.rz=-.1;
  }
  const auto instantaneous=old.step(1.002,window.reference[0],window.output[0].pose);
  const auto result=windowed.step(1.002,window);
  check(instantaneous.valid&&instantaneous.gate<.7,"reversal fixture did not exercise old projection");
  check(result.valid&&result.phase_window_used&&result.phase_window_sec==.04&&result.gate==1.,
        "known short reversal still slowed the phase clock");
  // A real delayed ramp must NOT disappear in the phase integral: both curves
  // have the same derivatives and constant separation at every future sample.
  for(std::size_t k=0;k<window.count;++k) {
    const double t=window.relative_time_sec[k];auto& r=window.reference[k];auto& y=window.output[k];
    r={};y={};r.pose.x=.04+.1*t;y.pose.x=.1*t;r.velocity.x=y.velocity.x=.1;
    r.pose=math::poseFromSe3(pinocchio::SE3(math::exp3({0,0,.06+.5*t}),Eigen::Vector3d(r.pose.x,0,0)));
    y.pose=math::poseFromSe3(pinocchio::SE3(math::exp3({0,0,.5*t}),Eigen::Vector3d(y.pose.x,0,0)));
    r.velocity.rz=y.velocity.rz=.5;
  }
  const auto lag=windowed.step(1.004,window);
  check(lag.valid&&std::abs(lag.positive_lag_m-.04)<1e-12&&
        std::abs(lag.positive_lag_rad-.06)<1e-12&&lag.gate<1.,"true delayed ramp was hidden");
  // A stationary output behind a moving target needs at least as much pacing.
  for(auto& y:window.output) {y={};}
  const auto stationary=windowed.step(1.006,window);
  check(stationary.valid&&stationary.positive_lag_m>.04&&stationary.positive_lag_rad>.06,
        "stationary output lag was masked");
  // No arbitrary direction is invented at zero speed. Absolute separation is
  // still owned by QP tracking acceptance and the independent final safety gate.
  for(auto& r:window.reference) {r.velocity={};r.acceleration={};}
  const auto still=windowed.step(1.008,window);
  check(still.valid&&still.gate==1.&&still.positive_lag_rad==0.,"zero-speed window invented phase lag");
  const double time=windowed.timeSec();window.relative_time_sec[4]=.041;
  const auto invalid=windowed.step(1.01,window);
  check(!invalid.valid&&windowed.timeSec()==time,"window exceeded explicit lookahead or changed failed state");
  window.relative_time_sec[4]=.04;window.output[2].velocity.x=std::numeric_limits<double>::quiet_NaN();
  check(!windowed.step(1.01,window).valid&&windowed.timeSec()==time,"nonfinite future output accepted");
  auto badcfg=cfg;badcfg.phase_lookahead_sec=.101;bool rejected=false;
  try {PreviewExecutionCursor bad(badcfg,leash());}catch(const std::invalid_argument&){rejected=true;}
  check(rejected,"lookahead beyond bounded cursor history accepted");
}
void historyAndCausalFuture() {
  CanonicalReferenceHistory h(3);const double epoch=1589440.296027291;
  for(int i=0;i<4;++i)h.append(epoch+.01*i,state(.01*i));
  FollowerOutputKinematics sampled;
  check(!h.sample(epoch,sampled),"expired history extrapolated");
  check(h.sample(epoch+.015,sampled)&&std::abs(sampled.pose.x-.015)<1e-8,"history interpolation failed");
  FollowerPreviewReference future;future.status=FollowerPreviewReferenceStatus::Ready;
  future.generated_at_sec=h.latestTimeSec();
  for(int k=0;k<=36;++k) {
    FollowerPreviewReferenceSample v;v.relative_time_sec=.01*k;v.kinematics=state(.03+.01*k);future.samples.push_back(v);
  }
  check(sampleKnownReference(h,future,future.generated_at_sec+.36,sampled),"valid large-epoch horizon endpoint rejected");
  check(std::abs(sampled.pose.x-.39)<1e-12,"horizon endpoint differs");
  check(!sampleKnownReference(h,future,future.generated_at_sec+.361,sampled),"future extrapolated beyond current known frame");
  future.generated_at_sec+=.001;
  check(!sampleKnownReference(h,future,h.latestTimeSec()+.01,sampled),"history/future time mismatch accepted");
  auto bad=state(0.);bad.acceleration.x=std::numeric_limits<double>::quiet_NaN();
  bool rejected=false;try {h.append(epoch+.04,bad);}catch(const std::invalid_argument&){rejected=true;}
  check(rejected&&h.latestTimeSec()==epoch+.03,"invalid history mutated buffer");
}
}
int main() {
  try {oneSidedLagAndRecovery();rotationAndTransactionalFailure();finiteWindowPhaseAndReversal();historyAndCausalFuture();
    std::cout<<"preview execution cursor tests passed\n";return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
