#include "rb_servo/control/preview_trajectory_tracker.hpp"
#include "rb_servo/math/se3.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <new>
#include <string>
#include <vector>

// Count C++ new calls from the actual linked qpOASES library as well as this
// module. This intentionally does not claim to intercept every libc malloc.
namespace allocation_audit {
std::atomic<bool> enabled{false};
std::atomic<std::size_t> count{0};
void record() { if(enabled.load(std::memory_order_relaxed)) count.fetch_add(1,std::memory_order_relaxed); }
}
void* operator new(std::size_t size) {
  if(void* p=std::malloc(std::max<std::size_t>(size,1))) { allocation_audit::record(); return p; }
  throw std::bad_alloc();
}
void* operator new[](std::size_t size) { return ::operator new(size); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p,std::size_t) noexcept { std::free(p); }
void operator delete[](void* p,std::size_t) noexcept { std::free(p); }
void* operator new(std::size_t size,std::align_val_t align) {
  void* p=nullptr;
  if(posix_memalign(&p,static_cast<std::size_t>(align),std::max<std::size_t>(size,1))!=0) throw std::bad_alloc();
  allocation_audit::record(); return p;
}
void* operator new[](std::size_t size,std::align_val_t align) { return ::operator new(size,align); }
void operator delete(void* p,std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p,std::align_val_t) noexcept { std::free(p); }
void operator delete(void* p,std::size_t,std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p,std::size_t,std::align_val_t) noexcept { std::free(p); }

namespace {
using namespace rb_servo;
using namespace rb_servo::control;
constexpr double kPi=3.14159265358979323846;
#define CHECK(condition) do { if(!(condition)) { std::cerr << "CHECK failed: " #condition << " at " << __LINE__ << '\n'; return false; } } while(false)

PreviewTrackerConfig config() {
  PreviewTrackerConfig c;
  // All physical limits are explicit. Unit tests allow a generous wall budget
  // to test correctness independently of scheduling/load; --benchmark reports
  // measured planning/sample cost, not hard real-time admission or WCET.
  c.max_linear_velocity_m_s=.6; c.max_linear_acceleration_m_s2=12.; c.max_linear_jerk_m_s3=2000.;
  c.max_angular_velocity_rad_s=1.4; c.max_angular_acceleration_rad_s2=40.; c.max_angular_jerk_rad_s3=4000.;
  c.linear_tracking_tolerance_m=.02; c.angular_tracking_tolerance_rad=.08;
  c.max_linear_tracking_slack_m=.08; c.max_angular_tracking_slack_rad=.25;
  c.max_reference_chart_angle_rad=1.; c.feasibility_tolerance=1e-7;
  c.max_solve_time_sec=.5; c.max_working_set_recalculations=300;
  return c;
}
Pose6D pose(const Eigen::Vector3d& xyz,const Eigen::Vector3d& rotation={0.,0.,0.}) {
  return math::poseFromSe3(pinocchio::SE3(math::exp3(rotation),xyz));
}
template<class F> PreviewReference reference(F function) {
  PreviewReference r; r.count=25;
  for(std::size_t k=0;k<r.count;++k) { r.knots[k].time_sec=.01*k; r.knots[k].pose=function(.01*k); }
  return r;
}
bool accepted(const PreviewSolveResult& r) {
  if(!r.accepted()) std::cerr << "solve status=" << static_cast<int>(r.status)
      << " time=" << r.diagnostics.solve_time_sec << " iterations=" << r.diagnostics.working_set_recalculations
      << " residual=" << r.diagnostics.max_constraint_violation
      << " angular_cuts=" << r.diagnostics.angular_norm_cuts << '\n';
  return r.accepted();
}
bool sameState(const PreviewMotionSample& a,const PreviewMotionSample& b,double tol=1e-9) {
  return math::positionDistance(a.pose,b.pose)<tol && math::orientationDistanceRad(a.pose,b.pose)<tol &&
      (a.linear_velocity-b.linear_velocity).norm()<tol && (a.linear_acceleration-b.linear_acceleration).norm()<tol &&
      (a.angular_velocity_body-b.angular_velocity_body).norm()<tol &&
      (a.angular_acceleration_body-b.angular_acceleration_body).norm()<tol;
}

bool testExplicitConfigAndConstantVelocity() {
  bool threw=false; try { PreviewTrajectoryTracker tracker(PreviewTrackerConfig{}); } catch(const std::invalid_argument&) { threw=true; }
  CHECK(threw);
  auto missing=config(); missing.linear_tracking_tolerance_m=std::numeric_limits<double>::quiet_NaN();
  threw=false; try { PreviewTrajectoryTracker tracker(missing); } catch(const std::invalid_argument&) { threw=true; }
  CHECK(threw);
  // Validate size_t before narrowing it to the QP's integer dimension.
  auto overflow=config(); overflow.horizon_steps=std::numeric_limits<std::size_t>::max();
  threw=false; try { PreviewTrajectoryTracker tracker(overflow); } catch(const std::invalid_argument&) { threw=true; }
  CHECK(threw);
  auto cfg=config(); PreviewTrajectoryTracker tracker(cfg);
  PreviewMotionState initial; initial.pose=pose({.4,-.2,.1});
  initial.linear_velocity={.1,-.03,.02}; initial.angular_velocity_body={.1,.03,-.04};
  const Eigen::Vector3d p0(initial.pose.x,initial.pose.y,initial.pose.z);
  const auto r=reference([&](double t) { return pose(p0+initial.linear_velocity*t,initial.angular_velocity_body*t); });
  const auto solved=tracker.plan(r,initial); CHECK(accepted(solved));
  for(int k=0;k<=120;++k) {
    PreviewMotionSample s; const double t=.002*k; CHECK(tracker.sample(t,s));
    CHECK(math::positionDistance(s.pose,pose(p0+initial.linear_velocity*t,initial.angular_velocity_body*t))<1e-8);
    CHECK((s.linear_velocity-initial.linear_velocity).norm()<1e-7);
    CHECK(s.linear_acceleration.norm()<1e-6);
    CHECK((s.angular_velocity_body-initial.angular_velocity_body).norm()<1e-7);
    CHECK(s.angular_acceleration_body.norm()<1e-6);
  }
  PreviewMotionSample s; CHECK(!tracker.sample(-.001,s)); CHECK(!tracker.sample(.241,s));
  CHECK(!tracker.sample(std::numeric_limits<double>::quiet_NaN(),s));
  return true;
}

bool testContinuousEnvelopeAndPhysicalAngularDerivatives() {
  auto cfg=config(); cfg.max_linear_velocity_m_s=.25; cfg.max_linear_acceleration_m_s2=1.2;
  cfg.max_linear_jerk_m_s3=12.; cfg.jerk_weight=.001;
  PreviewTrajectoryTracker tracker(cfg); PreviewMotionState initial; initial.pose=pose({0.,0.,0.});
  const auto r=reference([](double t) {
    return pose({.045*std::min(t/.04,1.),-.018*std::sin(12.*t),.015*std::sin(9.*t)},
                {.04*std::sin(9.*t),.035*std::sin(13.*t),.025*std::sin(17.*t)});
  });
  CHECK(accepted(tracker.plan(r,initial)));
  for(int k=0;k<=960;++k) {
    PreviewMotionSample s; CHECK(tracker.sample(k*.00025,s));
    CHECK(s.linear_velocity.cwiseAbs().maxCoeff()<=cfg.max_linear_velocity_m_s+2e-7);
    CHECK(s.linear_acceleration.cwiseAbs().maxCoeff()<=cfg.max_linear_acceleration_m_s2+2e-7);
    CHECK(s.linear_jerk.cwiseAbs().maxCoeff()<=cfg.max_linear_jerk_m_s3+2e-5);
    CHECK(s.angular_velocity_body.norm()<=cfg.max_angular_velocity_rad_s+2e-7);
    CHECK(s.angular_acceleration_body.norm()<=cfg.max_angular_acceleration_rad_s2+2e-7);
    CHECK(s.angular_jerk_stand.norm()<=cfg.max_angular_jerk_rad_s3+2e-5);
  }
  for(int k=1;k<24;++k) {
    PreviewMotionSample lo,hi; CHECK(tracker.sample(k*.01-1e-9,lo)); CHECK(tracker.sample(k*.01+1e-9,hi));
    CHECK(sameState(lo,hi,2e-5));
  }
  // Independent physical derivative oracle from neighboring output rotations,
  // excluding interval boundaries where jerk is legitimately discontinuous.
  const double eps=1e-6;
  for(double t : {.023,.047,.083,.127,.163,.217}) {
    PreviewMotionSample lo,mid,hi; CHECK(tracker.sample(t-eps,lo)); CHECK(tracker.sample(t,mid)); CHECK(tracker.sample(t+eps,hi));
    const auto Rlo=math::rotationFromPose(lo.pose), Rmid=math::rotationFromPose(mid.pose), Rhi=math::rotationFromPose(hi.pose);
    const Eigen::Vector3d omega_stand=math::log3(Rhi*Rlo.transpose())/(2.*eps);
    CHECK((omega_stand-Rmid*mid.angular_velocity_body).norm()<2e-6);
    const Eigen::Vector3d alpha_stand=(Rhi*hi.angular_velocity_body-Rlo*lo.angular_velocity_body)/(2.*eps);
    CHECK((alpha_stand-Rmid*mid.angular_acceleration_body).norm()<2e-5);
    const Eigen::Vector3d jerk_stand=(Rhi*hi.angular_acceleration_body-Rlo*lo.angular_acceleration_body)/(2.*eps);
    CHECK((jerk_stand-mid.angular_jerk_stand).norm()<2e-3);
  }
  return true;
}

bool testReplanContinuityGaugeAndAntipodal() {
  auto cfg=config(); PreviewTrajectoryTracker tracker(cfg), antipodal(cfg);
  PreviewMotionState initial; initial.pose=pose({.2,.1,.05},{.2,-.1,.3});
  const auto R0=math::rotationFromPose(initial.pose);
  auto r=reference([&](double t) {
    return math::poseFromSe3(pinocchio::SE3(R0*math::exp3({.10*std::sin(5*t),.08*std::sin(8*t),.04*t}),
        Eigen::Vector3d(.2+.05*t,.1+.004*std::sin(9*t),.05)));
  });
  auto rsign=r;
  for(std::size_t k=0;k<rsign.count;k+=2) for(double& q:*rsign.knots[k].pose.quaternion_xyzw) q=-q;
  CHECK(accepted(tracker.plan(r,initial))); CHECK(accepted(antipodal.plan(rsign,initial)));
  PreviewMotionSample before,other; CHECK(tracker.sample(.037,before)); CHECK(antipodal.sample(.037,other)); CHECK(sameState(before,other));
  PreviewMotionState splice=before;
  auto next=reference([&](double t) { return math::poseFromSe3(pinocchio::SE3(
      math::rotationFromPose(splice.pose)*math::exp3({.08*t,.12*t,-.03*t}),
      Eigen::Vector3d(splice.pose.x+.04*t,splice.pose.y-.02*t,splice.pose.z+.01*t))); });
  CHECK(accepted(tracker.plan(next,splice))); CHECK(tracker.sample(0.,other)); CHECK(sameState(before,other,2e-8));
  CHECK(tracker.sample(.071,before));
  const Eigen::Vector3d translation(.003,-.002,.004);
  const Eigen::Quaterniond rotation(Eigen::AngleAxisd(.12,Eigen::Vector3d(1.,2.,3.).normalized()));
  tracker.shiftCommonFrame(translation,rotation); CHECK(tracker.sample(.071,other));
  CHECK((Eigen::Vector3d(other.pose.x-before.pose.x,other.pose.y-before.pose.y,other.pose.z-before.pose.z)-translation).norm()<1e-12);
  CHECK(math::log3(math::rotationFromPose(other.pose)*(rotation.toRotationMatrix()*math::rotationFromPose(before.pose)).transpose()).norm()<1e-12);
  CHECK((before.linear_velocity-other.linear_velocity).norm()<1e-12);
  CHECK((before.angular_velocity_body-other.angular_velocity_body).norm()<1e-12);
  CHECK((before.angular_acceleration_body-other.angular_acceleration_body).norm()<1e-12);
  CHECK((rotation*before.angular_jerk_stand-other.angular_jerk_stand).norm()<1e-9);
  tracker.reset(); CHECK(!tracker.hasTrajectory()); CHECK(!tracker.sample(0.,other));
  return true;
}

bool testFailClosedAndOldPlanPreserved() {
  auto cfg=config(); PreviewTrajectoryTracker tracker(cfg);
  PreviewMotionState initial; initial.pose=pose({0.,0.,0.});
  const auto good=reference([](double t){return pose({.05*t,0.,0.});});
  CHECK(accepted(tracker.plan(good,initial)));
  PreviewMotionSample before,after; CHECK(tracker.sample(.08,before));
  auto invalid=good; invalid.knots[2].time_sec=invalid.knots[1].time_sec;
  CHECK(tracker.plan(invalid,initial).status==PreviewSolveStatus::InvalidReference);
  invalid=good; invalid.count=1; CHECK(tracker.plan(invalid,initial).status==PreviewSolveStatus::InvalidReference);
  invalid=good; invalid.knots[4].pose.x=std::numeric_limits<double>::quiet_NaN();
  CHECK(tracker.plan(invalid,initial).status==PreviewSolveStatus::InvalidReference);
  auto bad=initial; bad.linear_velocity.x()=cfg.max_linear_velocity_m_s+.01;
  CHECK(tracker.plan(good,bad).status==PreviewSolveStatus::InvalidInitialState);
  bad=initial; bad.angular_velocity_body={1.0,1.0,0.0};
  CHECK(bad.angular_velocity_body.norm()>cfg.max_angular_velocity_rad_s);
  CHECK(tracker.plan(good,bad).status==PreviewSolveStatus::InvalidInitialState);
  const auto remote=reference([](double){return pose({2.,0.,0.});});
  CHECK(tracker.plan(remote,initial).status==PreviewSolveStatus::TrackingBudgetExceeded);
  CHECK(tracker.sample(.08,after)); CHECK(sameState(before,after));
  auto tiny=cfg; tiny.max_solve_time_sec=1e-12; PreviewTrajectoryTracker timeout(tiny);
  CHECK(timeout.plan(good,initial).status==PreviewSolveStatus::TimeBudgetExceeded);
  CHECK(!timeout.hasTrajectory()); CHECK(!timeout.sample(0.,after));
  auto restricted=cfg; restricted.max_working_set_recalculations=1;
  PreviewTrajectoryTracker work_limited(restricted);
  const auto limited=work_limited.plan(remote,initial);
  CHECK(!limited.accepted()); CHECK(limited.diagnostics.working_set_recalculations<=6);
  return true;
}

bool testAngularNormAuthorityAndRebasedSplice() {
  auto cfg=config();cfg.jerk_weight=2000.;
  for(const Eigen::Vector3d velocity : {Eigen::Vector3d(0.,0.,1.2),Eigen::Vector3d(.95,.95,0.)}) {
    PreviewTrajectoryTracker tracker(cfg);
    PreviewMotionState initial;initial.pose=pose({0.,0.,0.},{.2,-.1,.3});
    initial.angular_velocity_body=velocity;
    for(int plan=0;plan<30;++plan) {
      const auto R0=math::rotationFromPose(initial.pose);
      const auto ref=reference([&](double t) {
        return math::poseFromSe3(pinocchio::SE3(R0*math::exp3(velocity*t),Eigen::Vector3d::Zero()));
      });
      const auto result=tracker.plan(ref,initial);CHECK(accepted(result));
      CHECK(!result.diagnostics.angular_norm_coupled);
      PreviewMotionSample first,next;CHECK(tracker.sample(0.,first));CHECK(tracker.sample(.012,next));
      CHECK((first.angular_velocity_body-initial.angular_velocity_body).norm()<1e-10);
      CHECK((first.angular_acceleration_body-initial.angular_acceleration_body).norm()<1e-10);
      CHECK((next.angular_velocity_body-velocity).norm()<1e-7);
      CHECK(next.angular_acceleration_body.norm()<1e-6);
      initial=next;
    }
  }
  // Simultaneous-axis demand must satisfy one physical norm ball even though
  // each component separately fits its outer box. Verify every Bernstein
  // control, not just dense samples that might miss an intersample overshoot.
  cfg.jerk_weight=.001;PreviewTrajectoryTracker coupled(cfg);
  PreviewMotionState initial;initial.pose=pose({0.,0.,0.});
  const auto ref=reference([](double t){return pose({0.,0.,0.},{1.1*t,1.1*t,.2*t});});
  const auto solved=coupled.plan(ref,initial);CHECK(accepted(solved));
  CHECK(solved.diagnostics.angular_norm_coupled);CHECK(solved.diagnostics.angular_norm_cuts>0);
  PreviewPolynomialTrajectory p;CHECK(coupled.exportTrajectory(p));
  const double V=cfg.max_angular_velocity_rad_s,A=cfg.max_angular_acceleration_rad_s2-.5*V*V;
  for(std::size_t k=0;k<p.count;++k) {
    CHECK(p.v.row(k).tail<3>().norm()<=V+cfg.feasibility_tolerance);
    CHECK((p.v.row(k).tail<3>()+.5*p.step_sec*p.a.row(k).tail<3>()).norm()<=V+cfg.feasibility_tolerance);
    CHECK(p.v.row(k+1).tail<3>().norm()<=V+cfg.feasibility_tolerance);
    CHECK(p.a.row(k).tail<3>().norm()<=A+cfg.feasibility_tolerance);
    CHECK(p.a.row(k+1).tail<3>().norm()<=A+cfg.feasibility_tolerance);
  }
  for(int k=0;k<=2400;++k) {
    PreviewMotionSample sample;CHECK(coupled.sample(coupled.durationSec()*k/2400.,sample));
    CHECK(sample.angular_velocity_body.norm()<=V+cfg.feasibility_tolerance);
    CHECK(sample.angular_acceleration_body.norm()<=cfg.max_angular_acceleration_rad_s2+cfg.feasibility_tolerance);
    CHECK(sample.angular_jerk_stand.norm()<=cfg.max_angular_jerk_rad_s3+cfg.feasibility_tolerance);
  }
  // Exercise changed constraints through SQProblem hotstarts and preserve
  // nonzero accepted angular acceleration when the exponential chart rebases.
  for(const Eigen::Vector3d demand : {Eigen::Vector3d(1.15,.9,.3),Eigen::Vector3d(1.,1.12,.15)}) {
    PreviewMotionSample splice,at_zero;CHECK(coupled.sample(.017,splice));
    CHECK(splice.angular_acceleration_body.norm()>1e-3);
    const auto R0=math::rotationFromPose(splice.pose);
    const auto next=reference([&](double t) {
      return math::poseFromSe3(pinocchio::SE3(R0*math::exp3(demand*t),Eigen::Vector3d::Zero()));
    });
    CHECK(accepted(coupled.plan(next,splice)));CHECK(coupled.sample(0.,at_zero));
    CHECK(sameState(splice,at_zero,1e-9));
    CHECK(coupled.exportTrajectory(p));
    for(std::size_t k=0;k<p.count;++k) {
      CHECK(p.v.row(k).tail<3>().norm()<=V+cfg.feasibility_tolerance);
      CHECK((p.v.row(k).tail<3>()+.5*p.step_sec*p.a.row(k).tail<3>()).norm()<=V+cfg.feasibility_tolerance);
      CHECK(p.v.row(k+1).tail<3>().norm()<=V+cfg.feasibility_tolerance);
      CHECK(p.a.row(k+1).tail<3>().norm()<=A+cfg.feasibility_tolerance);
    }
  }
  return true;
}

bool testJerkRegularizationAndSamplerAllocation() {
  auto weak=config(),strong=config(); weak.jerk_weight=1e-6; weak.jerk_difference_weight=0.;
  strong.jerk_weight=.4; strong.jerk_difference_weight=.2;
  PreviewTrajectoryTracker a(weak),b(strong);
  PreviewMotionState initial; initial.pose=pose({0.,0.,0.});
  const auto r=reference([](double t){return pose({.004*std::sin(2*kPi*8*t),0.,0.});});
  CHECK(accepted(a.plan(r,initial))); CHECK(accepted(b.plan(r,initial)));
  double ja=0.,jb=0.; PreviewMotionSample sa,sb;
  for(int k=0;k<120;++k) {
    CHECK(a.sample(k*.002,sa)); CHECK(b.sample(k*.002,sb));
    ja+=sa.linear_jerk.squaredNorm(); jb+=sb.linear_jerk.squaredNorm();
  }
  CHECK(jb<.5*ja);
  allocation_audit::count=0; allocation_audit::enabled=true;
  bool valid=true; for(int k=0;k<1000;++k) valid=b.sample((k%120)*.002,sb)&&valid;
  allocation_audit::enabled=false;
  CHECK(valid); CHECK(allocation_audit::count.load()==0);
  return true;
}

bool testInitialTolerancePreservesSplice() {
  const auto cfg=config(); PreviewTrajectoryTracker tracker(cfg);
  PreviewMotionState initial; initial.pose=pose({0.,0.,0.});
  initial.linear_velocity.x()=cfg.max_linear_velocity_m_s+1e-14;
  const auto constant=reference([&](double t){return pose({initial.linear_velocity.x()*t,0.,0.});});
  CHECK(accepted(tracker.plan(constant,initial)));
  PreviewMotionSample actual;
  CHECK(tracker.sample(0.,actual));
  CHECK(actual.linear_velocity.x()==initial.linear_velocity.x());
  auto excessive=initial; excessive.linear_velocity.x()=cfg.max_linear_velocity_m_s+2.*cfg.feasibility_tolerance;
  CHECK(tracker.plan(constant,excessive).status==PreviewSolveStatus::InvalidInitialState);

  initial.linear_velocity.setZero();
  initial.linear_acceleration.y()=-cfg.max_linear_acceleration_m_s2-1e-14;
  const auto stopped=reference([](double){return pose({0.,0.,0.});});
  CHECK(accepted(tracker.plan(stopped,initial)));
  CHECK(tracker.sample(0.,actual));
  CHECK(actual.linear_acceleration.y()==initial.linear_acceleration.y());
  excessive=initial;
  excessive.linear_acceleration.y()=-cfg.max_linear_acceleration_m_s2-2.*cfg.feasibility_tolerance;
  CHECK(tracker.plan(stopped,excessive).status==PreviewSolveStatus::InvalidInitialState);
  return true;
}

int benchmark() {
  using Clock=std::chrono::steady_clock;
  auto cfg=config(); PreviewTrajectoryTracker tracker(cfg); PreviewMotionState initial; initial.pose=pose({0.,0.,0.});
  std::vector<double> times,sample_times; times.reserve(200); sample_times.reserve(200);
  int solved=0,rejected=0; std::size_t new_calls=0;
  for(int k=0;k<200;++k) {
    const double now=.01*k;
    auto r=reference([&](double t){ const double s=now+t; return pose({.015*std::sin(4*s),.01*std::sin(7*s),.005*std::sin(3*s)},
        {.04*std::sin(3*s),.03*std::sin(6*s),.02*std::sin(5*s)}); });
    allocation_audit::count=0; allocation_audit::enabled=true;
    const auto begin=Clock::now(); const auto result=tracker.plan(r,initial);
    const double duration=std::chrono::duration<double,std::micro>(Clock::now()-begin).count();
    allocation_audit::enabled=false; new_calls+=allocation_audit::count.load(); times.push_back(duration);
    if(result.accepted()) { ++solved; PreviewMotionSample next; const auto start=Clock::now(); tracker.sample(.01,next);
      sample_times.push_back(std::chrono::duration<double,std::micro>(Clock::now()-start).count()); initial=next; }
    else ++rejected;
  }
  auto percentile=[](std::vector<double> v,double p){std::sort(v.begin(),v.end());return v.empty()?0.:v[static_cast<std::size_t>((v.size()-1)*p)];};
  std::cout << "{\"plans\":200,\"accepted\":" << solved << ",\"rejected\":" << rejected
      << ",\"plan_us_p50\":" << percentile(times,.5) << ",\"plan_us_p95\":" << percentile(times,.95)
      << ",\"plan_us_max\":" << percentile(times,1.) << ",\"sample_us_p95\":" << percentile(sample_times,.95)
      << ",\"cpp_new_calls_during_plan\":" << new_calls << "}\n";
  return solved>0?0:1;
}
} // namespace

int main(int argc,char** argv) {
  if(argc==2 && std::string(argv[1])=="--benchmark") return benchmark();
  const std::pair<const char*,bool(*)()> tests[]={
      {"explicit config and constant velocity",testExplicitConfigAndConstantVelocity},
      {"continuous envelope and physical angular derivatives",testContinuousEnvelopeAndPhysicalAngularDerivatives},
      {"replan continuity, gauge and antipodal quaternion",testReplanContinuityGaugeAndAntipodal},
      {"fail closed and preserve previous plan",testFailClosedAndOldPlanPreserved},
      {"angular norm authority and rebased splice",testAngularNormAuthorityAndRebasedSplice},
      {"jerk objective and sampler allocation",testJerkRegularizationAndSamplerAllocation},
      {"initial tolerance preserves unmodified splice",testInitialTolerancePreservesSplice}};
  try {
    for(const auto& test:tests) { if(!test.second()) return 1; std::cout << "PASS " << test.first << '\n'; }
  } catch(const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
  return 0;
}
