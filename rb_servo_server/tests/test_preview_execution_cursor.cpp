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
  try {oneSidedLagAndRecovery();rotationAndTransactionalFailure();historyAndCausalFuture();
    std::cout<<"preview execution cursor tests passed\n";return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
