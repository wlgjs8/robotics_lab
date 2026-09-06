#include "rb_servo/control/preview_brake.hpp"
#include "rb_servo/math/se3.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <new>

// Counts C++ allocations on this thread only; this is not a libc allocation
// audit or a worst-case scheduling guarantee.
namespace allocation_audit {
thread_local bool enabled=false;
thread_local std::size_t count=0;
void record(){if(enabled)++count;}
}
void* operator new(std::size_t n){if(void* p=std::malloc(std::max<std::size_t>(n,1))){allocation_audit::record();return p;}throw std::bad_alloc();}
void* operator new[](std::size_t n){return ::operator new(n);}
void operator delete(void* p)noexcept{std::free(p);}
void operator delete[](void* p)noexcept{std::free(p);}
void operator delete(void* p,std::size_t)noexcept{std::free(p);}
void operator delete[](void* p,std::size_t)noexcept{std::free(p);}
void* operator new(std::size_t n,std::align_val_t align){void* p=nullptr;if(posix_memalign(&p,static_cast<std::size_t>(align),std::max<std::size_t>(n,1)))throw std::bad_alloc();allocation_audit::record();return p;}
void* operator new[](std::size_t n,std::align_val_t align){return ::operator new(n,align);}
void operator delete(void* p,std::align_val_t)noexcept{std::free(p);}
void operator delete[](void* p,std::align_val_t)noexcept{std::free(p);}
void operator delete(void* p,std::size_t,std::align_val_t)noexcept{std::free(p);}
void operator delete[](void* p,std::size_t,std::align_val_t)noexcept{std::free(p);}

namespace {
using namespace rb_servo;
using namespace rb_servo::control;
#define CHECK(x) do{if(!(x)){std::cerr<<"CHECK " #x " failed at "<<__LINE__<<'\n';return false;}}while(false)
PreviewTrackerConfig config(){
  PreviewTrackerConfig c;c.max_linear_velocity_m_s=.6;c.max_linear_acceleration_m_s2=12.;c.max_linear_jerk_m_s3=2000.;
  c.max_angular_velocity_rad_s=1.4;c.max_angular_acceleration_rad_s2=40.;c.max_angular_jerk_rad_s3=4000.;
  c.feasibility_tolerance=1e-7;c.max_reference_chart_angle_rad=1.;return c;
}
PreviewMotionState moving(){
  PreviewMotionState s;s.pose=math::poseFromSe3(pinocchio::SE3(math::exp3(Eigen::Vector3d{.4,-.3,.2}),Eigen::Vector3d{.4,.1,.3}));
  s.linear_velocity={.3,-.2,.1};s.linear_acceleration={2.,-1.,-.5};
  s.angular_velocity_body={.3,-.2,.4};s.angular_acceleration_body={2.,3.,-1.};return s;
}
bool same(const PreviewMotionState&a,const PreviewMotionState&b,double tolerance=1e-9){
  return math::positionDistance(a.pose,b.pose)<tolerance&&math::orientationDistanceRad(a.pose,b.pose)<tolerance&&
      (a.linear_velocity-b.linear_velocity).norm()<tolerance&&(a.linear_acceleration-b.linear_acceleration).norm()<tolerance&&
      (a.angular_velocity_body-b.angular_velocity_body).norm()<tolerance&&
      (a.angular_acceleration_body-b.angular_acceleration_body).norm()<tolerance;
}
bool finiteStopAndLimits(){
  const auto c=config();PreviewBrake brake(c,.002);const auto initial=moving();
  CHECK(brake.start(initial)==PreviewBrakeStatus::Ready);
  CHECK(brake.durationSec()>0.&&brake.durationSec()<1.);PreviewMotionSample first,last;
  CHECK(brake.sample(0.,first));CHECK(same(first,initial));
  CHECK(brake.sample(brake.durationSec(),last));
  CHECK(last.linear_velocity.norm()<1e-10&&last.linear_acceleration.norm()<1e-10);
  CHECK(last.angular_velocity_body.norm()<1e-10&&last.angular_acceleration_body.norm()<1e-10);
  CHECK(last.linear_jerk.norm()==0.&&last.angular_jerk_stand.norm()==0.);
  CHECK(math::positionDistance(last.pose,initial.pose)>0.); // A brake is not an instantaneous hold.
  for(int k=0;k<=4000;++k){
    PreviewMotionSample x;CHECK(brake.sample(brake.durationSec()*k/4000.,x));
    CHECK(x.linear_velocity.cwiseAbs().maxCoeff()<=c.max_linear_velocity_m_s+1e-7);
    CHECK(x.linear_acceleration.cwiseAbs().maxCoeff()<=c.max_linear_acceleration_m_s2+1e-7);
    CHECK(x.linear_jerk.cwiseAbs().maxCoeff()<=c.max_linear_jerk_m_s3+1e-7);
    CHECK(x.angular_velocity_body.norm()<=c.max_angular_velocity_rad_s+1e-7);
    CHECK(x.angular_acceleration_body.norm()<=c.max_angular_acceleration_rad_s2+1e-7);
    CHECK(x.angular_jerk_stand.norm()<=c.max_angular_jerk_rad_s3+1e-7);
  }
  PreviewMotionSample held;CHECK(brake.sample(brake.durationSec()+100.,held));CHECK(same(held,last));
  CHECK(!brake.sample(-.001,held));CHECK(!brake.sample(std::numeric_limits<double>::quiet_NaN(),held));
  return true;
}
bool physicalAngularDerivatives(){
  PreviewBrake brake(config(),.002);CHECK(brake.start(moving())==PreviewBrakeStatus::Ready);
  // Select a time inside one jerk interval rather than a derivative jump.
  const double t=.0005,h=1e-6;PreviewMotionSample lo,mid,hi;
  CHECK(brake.sample(t-h,lo));CHECK(brake.sample(t,mid));CHECK(brake.sample(t+h,hi));
  const auto rlo=math::rotationFromPose(lo.pose),rmid=math::rotationFromPose(mid.pose),rhi=math::rotationFromPose(hi.pose);
  const Eigen::Vector3d body_velocity=math::log3(rlo.transpose()*rhi)/(2.*h);
  CHECK((body_velocity-mid.angular_velocity_body).norm()<1e-4);
  const Eigen::Vector3d stand_acceleration_derivative=(rhi*hi.angular_acceleration_body-rlo*lo.angular_acceleration_body)/(2.*h);
  CHECK((stand_acceleration_derivative-mid.angular_jerk_stand).norm()<.01);
  CHECK((rmid*mid.angular_velocity_body).allFinite());return true;
}
bool refuseWithoutClipping(){
  PreviewBrake brake(config(),.002);auto s=moving();CHECK(brake.start(s)==PreviewBrakeStatus::Ready);
  s.linear_velocity.x()=.601;CHECK(brake.start(s)==PreviewBrakeStatus::InitialOutsideLimits);
  PreviewMotionSample out;CHECK(!brake.sample(0.,out));
  s=moving();s.linear_velocity.x()=.599;s.linear_acceleration.x()=10.;
  CHECK(brake.start(s)==PreviewBrakeStatus::InitialOutsideLimits); // unavoidable speed overshoot
  s=moving();s.angular_velocity_body={1.,0.,0.};
  CHECK(s.angular_velocity_body.norm()<config().max_angular_velocity_rad_s);
  CHECK(brake.start(s)==PreviewBrakeStatus::Ready); // same norm budget, rotated chart
  CHECK(brake.sample(0.,out));CHECK(same(out,s));
  s=moving();s.angular_velocity_body={1.401,0.,0.};
  CHECK(brake.start(s)==PreviewBrakeStatus::InitialOutsideLimits); // physical norm cannot fit either chart
  s=moving();s.angular_velocity_body={1.2,0.,0.};s.angular_acceleration_body={0.,38.,0.};
  CHECK(brake.start(s)==PreviewBrakeStatus::InitialOutsideLimits); // balancing omega cannot fit this alpha
  s=moving();s.angular_acceleration_body.setZero();
  s.angular_velocity_body.setConstant(config().max_angular_velocity_rad_s/std::sqrt(3.)+.8*config().feasibility_tolerance);
  CHECK(s.angular_velocity_body.cwiseAbs().maxCoeff()<config().max_angular_velocity_rad_s/std::sqrt(3.)+config().feasibility_tolerance);
  CHECK(s.angular_velocity_body.norm()>config().max_angular_velocity_rad_s+config().feasibility_tolerance);
  CHECK(brake.start(s)==PreviewBrakeStatus::InitialOutsideLimits);
  s=moving();s.pose.rx=std::numeric_limits<double>::quiet_NaN();
  CHECK(brake.start(s)==PreviewBrakeStatus::InvalidInitialState);
  auto invalid=config();invalid.max_linear_jerk_m_s3=0.;bool threw=false;
  try{PreviewBrake bad(invalid,.002);}catch(const std::invalid_argument&){threw=true;}CHECK(threw);
  s=moving();s.linear_velocity.setZero();s.linear_acceleration.setZero();
  s.angular_velocity_body.setZero();s.angular_acceleration_body.setZero();
  CHECK(brake.start(s)==PreviewBrakeStatus::Ready);CHECK(brake.durationSec()==0.);
  CHECK(brake.sample(1.,out));CHECK(same(out,s));brake.reset();CHECK(!brake.sample(0.,out));return true;
}
bool exportShiftAndNoAllocation(){
  PreviewBrake brake(config(),.002);const auto initial=moving();CHECK(brake.start(initial)==PreviewBrakeStatus::Ready);
  PreviewBrakeTrajectory exported;CHECK(brake.exportTrajectory(exported));const double t=.001;
  PreviewMotionSample before,after;CHECK(exported.sample(t,before));
  const Eigen::Vector3d dp{.02,-.01,.03};const Eigen::Quaterniond rotation(math::exp3(Eigen::Vector3d{.2,-.1,.15}));
  exported.shiftCommonFrame(dp,rotation);CHECK(exported.sample(t,after));
  CHECK((Eigen::Vector3d(after.pose.x-before.pose.x,after.pose.y-before.pose.y,after.pose.z-before.pose.z)-dp).norm()<1e-10);
  CHECK((math::rotationFromPose(after.pose)-rotation.toRotationMatrix()*math::rotationFromPose(before.pose)).norm()<1e-10);
  CHECK((after.linear_velocity-before.linear_velocity).norm()<1e-10);
  CHECK((after.angular_velocity_body-before.angular_velocity_body).norm()<1e-10);
  CHECK((after.angular_jerk_stand-rotation*before.angular_jerk_stand).norm()<1e-7);
  // Warm all exercised sample/export paths before auditing repeated starts.
  const auto start=std::chrono::steady_clock::now();allocation_audit::count=0;allocation_audit::enabled=true;
  bool good=true;
  for(int k=0;k<1000;++k){
    good=good&&brake.start(initial)==PreviewBrakeStatus::Ready;
    good=good&&brake.exportTrajectory(exported);good=good&&exported.sample(.001,after);
  }
  allocation_audit::enabled=false;
  CHECK(good);CHECK(allocation_audit::count==0);
  const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
  std::cout<<"1000 start/export/sample calls: "<<elapsed<<" sec; C++ allocations="<<allocation_audit::count<<'\n';
  exported.shiftCommonFrame(dp,Eigen::Quaterniond(0.,0.,0.,0.));CHECK(!exported.valid);return true;
}
bool variedReuseMatchesFreshCalculator(){
  PreviewBrake reused(config(),.002);
  for(int k=0;k<128;++k){
    auto state=moving();
    for(int axis=0;axis<3;++axis){
      const double phase=.37*k+.61*axis;
      state.linear_velocity[axis]=k%7==0?0.:.1*std::sin(phase);
      state.linear_acceleration[axis]=.5*std::cos(phase);
      state.angular_velocity_body[axis]=k%11==0?0.:.08*std::cos(phase);
      state.angular_acceleration_body[axis]=.5*std::sin(phase);
    }
    if(k==0){state.linear_velocity={-.01356,-.01025,-.002758};
      state.linear_acceleration={.2333,-.2832,.05017};
      state.angular_velocity_body={-.012061,-.007705,-.037410};
      state.angular_acceleration_body={.13934,-.03321,-.29611};}
    PreviewBrake fresh(config(),.002);
    CHECK(reused.start(state)==PreviewBrakeStatus::Ready);CHECK(fresh.start(state)==PreviewBrakeStatus::Ready);
    CHECK(std::abs(reused.durationSec()-fresh.durationSec())<1e-12);
    for(int sample=0;sample<4;++sample){PreviewMotionSample a,b;const double t=reused.durationSec()*sample/3.;
      CHECK(reused.sample(t,a)&&fresh.sample(t,b));CHECK(same(a,b));}
  }
  return true;
}
bool velocityModeUnusedPositionTargetCannotChangeChartBound(){
  ruckig::Ruckig<1> solver(.002);ruckig::InputParameter<1> input;ruckig::Trajectory<1> trajectory;
  input.control_interface=ruckig::ControlInterface::Velocity;
  input.current_position={0.};input.current_velocity={.2};input.current_acceleration={0.};
  input.target_position={0.};input.target_velocity={0.};input.target_acceleration={0.};
  input.max_velocity={.6};input.max_acceleration={1.};input.max_jerk={2.};
  CHECK(solver.calculate(input,trajectory)>=ruckig::Result::Working);
  auto profile=trajectory.get_profiles()[0][0];
  const auto expected=previewBrakeIntegratedPositionExtrema(profile);
  CHECK(expected.min>=-1e-12&&expected.max>0.&&expected.max<.1);
  for(double poison:{-1e250,1e250}){
    profile.pf=poison;
    const auto broken=profile.get_position_extrema();
    CHECK(std::max(std::abs(broken.min),std::abs(broken.max))==std::abs(poison));
    const auto certified=previewBrakeIntegratedPositionExtrema(profile);
    CHECK(certified.min==expected.min&&certified.max==expected.max);
  }
  return true;
}
PreviewPolynomialTrajectory angularPolynomial(){
  PreviewPolynomialTrajectory p;p.count=24;p.step_sec=.01;p.valid=true;
  p.p.setZero();p.v.setZero();p.a.setZero();p.jerk.setZero();
  p.rotation0=math::rotationFromPose(moving().pose);
  p.p.row(0).head<3>()=Eigen::Vector3d{.4,.1,.3};
  p.v.row(0).tail<3>()=Eigen::Vector3d{.3,-.2,.2};
  p.a.row(0).tail<3>()=Eigen::Vector3d{.2,.1,-.1};
  const double h=p.step_sec;
  for(std::size_t k=0;k<p.count;++k){
    p.jerk.row(k).tail<3>()=Eigen::Vector3d{.3,.2,-.2};
    p.p.row(k+1)=p.p.row(k)+h*p.v.row(k)+.5*h*h*p.a.row(k)+h*h*h/6.*p.jerk.row(k);
    p.v.row(k+1)=p.v.row(k)+h*p.a.row(k)+.5*h*h*p.jerk.row(k);
    p.a.row(k+1)=p.a.row(k)+h*p.jerk.row(k);
  }
  return p;
}
bool sameLinear(const PreviewMotionSample& a,const PreviewMotionSample& b){
  return a.pose.x==b.pose.x&&a.pose.y==b.pose.y&&a.pose.z==b.pose.z&&
      (a.linear_velocity-b.linear_velocity).norm()==0.&&
      (a.linear_acceleration-b.linear_acceleration).norm()==0.&&
      (a.linear_jerk-b.linear_jerk).norm()==0.;
}
bool sameAngular(const PreviewMotionSample& a,const PreviewMotionSample& b){
  const double acos_angle=math::orientationDistanceRad(a.pose,b.pose);
  const double log_angle=math::log3(math::rotationFromPose(a.pose).transpose()*math::rotationFromPose(b.pose)).norm();
  const double velocity=(a.angular_velocity_body-b.angular_velocity_body).norm();
  const double acceleration=(a.angular_acceleration_body-b.angular_acceleration_body).norm();
  const double jerk=(a.angular_jerk_stand-b.angular_jerk_stand).norm();
  if(acos_angle>=1e-10)
    std::cout<<"angular equality: acos="<<acos_angle<<" log="<<log_angle<<" dv="<<velocity
        <<" da="<<acceleration<<" dj="<<jerk<<'\n';
  // acos(trace) loses first-order resolution near identity. Use the same SO3
  // log residual as runtime provenance checks without changing its tolerance.
  return log_angle<1e-10&&velocity<1e-10&&acceleration<1e-10&&jerk<1e-8;
}
bool angularContinuationDeadlinesAndFolds(){
  PreviewAngularContinuation continuation;PreviewMotionSample sentinels;
  static_cast<PreviewMotionState&>(sentinels)=moving();sentinels.linear_jerk={3.,4.,5.};
  sentinels.angular_jerk_stand={6.,7.,8.};auto combined=sentinels;
  CHECK(continuation.sample(10.,combined));CHECK(same(combined,sentinels)&&sameLinear(combined,sentinels));
  CHECK(continuation.terminalHoldAvailableAt(10.));
  const auto polynomial=angularPolynomial();
  CHECK(continuation.retainPolynomial(polynomial,10.,10.034));CHECK(continuation.retainsPolynomial());
  CHECK(!continuation.terminalHoldAvailableAt(11.));
  PreviewMotionSample expected;CHECK(polynomial.sample(.011,expected));
  CHECK(continuation.sample(10.011,combined));CHECK(sameAngular(combined,expected));CHECK(sameLinear(combined,sentinels));
  const auto unchanged=combined;
  CHECK(!continuation.sample(9.999,combined));CHECK(!continuation.sample(10.034,combined));
  CHECK(!continuation.sample(11.,combined));CHECK(sameAngular(combined,unchanged)&&sameLinear(combined,unchanged));
  CHECK(!continuation.needsBrake(10.033));CHECK(continuation.needsBrake(10.034));
  const Eigen::Vector3d dp{.5,-.2,.1};const Eigen::Quaterniond dr(math::exp3(Eigen::Vector3d{.2,-.1,.15}));
  continuation.shiftCommonFrame(dp,dr);CHECK(continuation.sample(10.011,combined));
  CHECK(sameLinear(combined,sentinels));
  CHECK((math::rotationFromPose(combined.pose)-dr.toRotationMatrix()*math::rotationFromPose(expected.pose)).norm()<1e-10);
  CHECK((combined.angular_velocity_body-expected.angular_velocity_body).norm()<1e-10);
  CHECK((combined.angular_jerk_stand-dr*expected.angular_jerk_stand).norm()<1e-8);
  // The runtime, not this helper, selects the actually accepted stopping seed.
  PreviewBrake solver(config(),.002);CHECK(solver.start(moving())==PreviewBrakeStatus::Ready);
  PreviewBrakeTrajectory stop;CHECK(solver.exportTrajectory(stop));
  CHECK(continuation.startBrake(stop,10.032));CHECK(!continuation.retainsPolynomial());
  CHECK(stop.sample(.002,expected));CHECK(continuation.sample(10.034,combined));
  CHECK(sameAngular(combined,expected)&&sameLinear(combined,sentinels));
  CHECK(!continuation.terminalHoldAvailableAt(10.032));
  CHECK(continuation.terminalHoldAvailableAt(10.032+stop.durationSec()));
  CHECK(continuation.sample(10.032+stop.durationSec(),combined));
  CHECK(combined.angular_velocity_body.norm()==0.&&combined.angular_acceleration_body.norm()==0.);
  CHECK(combined.angular_jerk_stand.norm()==0.);
  CHECK(continuation.sample(11.,combined));CHECK(combined.angular_velocity_body.norm()==0.);
  CHECK(combined.angular_acceleration_body.norm()==0.&&combined.angular_jerk_stand.norm()==0.);
  CHECK(sameLinear(combined,sentinels));
  continuation.shiftCommonFrame(dp,Eigen::Quaterniond(0.,0.,0.,0.));CHECK(!continuation.sample(11.,combined));
  auto bad=polynomial;bad.jerk(5,3)=std::numeric_limits<double>::quiet_NaN();
  CHECK(!continuation.retainPolynomial(bad,10.,10.034));
  CHECK(!continuation.retainPolynomial(polynomial,10.,10.+polynomial.durationSec()+.001));
  CHECK(!continuation.startBrake(stop,std::numeric_limits<double>::infinity()));
  CHECK(continuation.retainPolynomial(polynomial,10.,10.034));
  continuation.shiftCommonFrame(dp,Eigen::Quaterniond(0.,0.,0.,0.));CHECK(!continuation.sample(10.011,combined));
  continuation.clear();CHECK(continuation.kind()==PreviewAngularContinuation::Kind::None);
  CHECK(continuation.sample(11.,combined));
  allocation_audit::count=0;allocation_audit::enabled=true;bool good=true;
  for(int i=0;i<1000;++i){
    good=good&&continuation.retainPolynomial(polynomial,10.,10.034)&&continuation.sample(10.011,combined);
    auto copy=continuation;copy.shiftCommonFrame(dp,dr);good=good&&copy.sample(10.012,combined);
    good=good&&continuation.startBrake(stop,10.032)&&continuation.sample(11.,combined);
  }
  allocation_audit::enabled=false;CHECK(good);CHECK(allocation_audit::count==0);
  return true;
}
PreviewMotionState recordedChartResetSeed(){
  PreviewMotionState state;
  state.pose={.5145451877258425,-.0565606211366557,-.16473598479706844,
              2.9897841925756428,-.3253564021457395,2.861293362199305};
  state.linear_velocity={-.0007869244973829888,.01579752533432918,-.07910387946153395};
  state.linear_acceleration={-.04987056356333698,.11073684751917438,.2260607905218742};
  state.angular_velocity_body={.1541753819806628,-.057801185345461144,-.8083053880092625};
  state.angular_acceleration_body={-.021831240078575544,.37281443658324265,-.001155749352398988};
  return state;
}
bool balancedAngularChartPreservesPhysicalMotion(){
  const auto cfg=config();const auto initial=recordedChartResetSeed();PreviewBrake brake(cfg,.002);
  CHECK(std::abs(initial.angular_velocity_body.z())>cfg.max_angular_velocity_rad_s/std::sqrt(3.));
  CHECK(initial.angular_velocity_body.norm()<cfg.max_angular_velocity_rad_s);
  CHECK(brake.start(initial)==PreviewBrakeStatus::Ready);
  PreviewBrakeTrajectory trajectory;CHECK(brake.exportTrajectory(trajectory));
  CHECK((trajectory.angular_basis-Eigen::Matrix3d::Identity()).norm()>.1);
  CHECK((trajectory.angular_basis.transpose()*trajectory.angular_basis-Eigen::Matrix3d::Identity()).norm()<1e-12);
  CHECK(std::abs(trajectory.angular_basis.determinant()-1.)<1e-12);
  const auto chart_v=trajectory.angular_basis.transpose()*initial.angular_velocity_body;
  CHECK((chart_v-Eigen::Vector3d::Constant(initial.angular_velocity_body.norm()/std::sqrt(3.))).norm()<1e-12);
  PreviewMotionSample first;CHECK(trajectory.sample(0.,first));
  CHECK(math::log3(math::rotationFromPose(first.pose).transpose()*math::rotationFromPose(initial.pose)).norm()<1e-12);
  CHECK((first.angular_velocity_body-initial.angular_velocity_body).norm()<1e-12);
  CHECK((first.angular_acceleration_body-initial.angular_acceleration_body).norm()<1e-12);
  CHECK((first.linear_velocity-initial.linear_velocity).norm()<1e-12);
  CHECK((first.linear_acceleration-initial.linear_acceleration).norm()<1e-12);
  for(int i=0;i<=4000;++i){
    PreviewMotionSample s;CHECK(trajectory.sample(trajectory.durationSec()*i/4000.,s));
    CHECK(s.angular_velocity_body.norm()<=cfg.max_angular_velocity_rad_s+cfg.feasibility_tolerance);
    CHECK(s.angular_acceleration_body.norm()<=cfg.max_angular_acceleration_rad_s2+cfg.feasibility_tolerance);
    CHECK(s.angular_jerk_stand.norm()<=cfg.max_angular_jerk_rad_s3+cfg.feasibility_tolerance);
    CHECK(s.linear_velocity.cwiseAbs().maxCoeff()<=cfg.max_linear_velocity_m_s+cfg.feasibility_tolerance);
    CHECK(s.linear_acceleration.cwiseAbs().maxCoeff()<=cfg.max_linear_acceleration_m_s2+cfg.feasibility_tolerance);
    CHECK(s.linear_jerk.cwiseAbs().maxCoeff()<=cfg.max_linear_jerk_m_s3+cfg.feasibility_tolerance);
  }
  const double t=.0005,h=1e-6;PreviewMotionSample lo,mid,hi;
  CHECK(trajectory.sample(t-h,lo)&&trajectory.sample(t,mid)&&trajectory.sample(t+h,hi));
  const auto rlo=math::rotationFromPose(lo.pose),rhi=math::rotationFromPose(hi.pose);
  CHECK((math::log3(rlo.transpose()*rhi)/(2*h)-mid.angular_velocity_body).norm()<1e-4);
  CHECK(((rhi*hi.angular_acceleration_body-rlo*lo.angular_acceleration_body)/(2*h)-mid.angular_jerk_stand).norm()<.01);
  const Eigen::Vector3d dp{.01,.02,-.03};const Eigen::Quaterniond dr(math::exp3(Eigen::Vector3d{.1,-.2,.05}));
  const auto basis=trajectory.angular_basis;trajectory.shiftCommonFrame(dp,dr);PreviewMotionSample folded;
  CHECK(trajectory.sample(t,folded));CHECK((trajectory.angular_basis-basis).norm()==0.);
  CHECK((math::rotationFromPose(folded.pose)-dr.toRotationMatrix()*math::rotationFromPose(mid.pose)).norm()<1e-10);
  CHECK((folded.angular_velocity_body-mid.angular_velocity_body).norm()<1e-10);
  CHECK((folded.angular_acceleration_body-mid.angular_acceleration_body).norm()<1e-10);
  CHECK((folded.angular_jerk_stand-dr*mid.angular_jerk_stand).norm()<1e-8);
  CHECK((Eigen::Vector3d(folded.pose.x-mid.pose.x,folded.pose.y-mid.pose.y,folded.pose.z-mid.pose.z)-dp).norm()<1e-10);
  CHECK(brake.start(moving())==PreviewBrakeStatus::Ready);CHECK(brake.exportTrajectory(trajectory));
  CHECK((trajectory.angular_basis-Eigen::Matrix3d::Identity()).norm()==0.); // identity preference survives reuse
  allocation_audit::count=0;allocation_audit::enabled=true;bool good=true;
  for(int i=0;i<1000;++i){
    good=good&&brake.start(initial)==PreviewBrakeStatus::Ready&&brake.exportTrajectory(trajectory)&&trajectory.sample(t,mid);
  }
  allocation_audit::enabled=false;CHECK(good);CHECK(allocation_audit::count==0);
  return true;
}
}
int main(){
  if(!finiteStopAndLimits()||!physicalAngularDerivatives()||!refuseWithoutClipping()||!exportShiftAndNoAllocation()||
     !variedReuseMatchesFreshCalculator()||!velocityModeUnusedPositionTargetCannotChangeChartBound()||
     !angularContinuationDeadlinesAndFolds()||!balancedAngularChartPreservesPhysicalMotion())return 1;
  std::cout<<"Preview brake tests passed\n";return 0;
}
