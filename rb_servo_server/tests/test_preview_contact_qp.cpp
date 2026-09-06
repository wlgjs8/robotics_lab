#include "rb_servo/control/preview_trajectory_tracker.hpp"
#include "rb_servo/math/se3.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

namespace {
using namespace rb_servo;
using namespace rb_servo::control;
#define CHECK(c) do { if(!(c)){std::cerr<<"CHECK failed: " #c " at "<<__LINE__<<'\n';return false;} } while(false)
PreviewTrackerConfig config() {
  PreviewTrackerConfig c;
  c.max_linear_velocity_m_s=.6;c.max_linear_acceleration_m_s2=12;c.max_linear_jerk_m_s3=2000;
  c.max_angular_velocity_rad_s=1.4;c.max_angular_acceleration_rad_s2=40;c.max_angular_jerk_rad_s3=4000;
  c.linear_tracking_tolerance_m=.02;c.angular_tracking_tolerance_rad=.08;
  c.max_linear_tracking_slack_m=.08;c.max_angular_tracking_slack_rad=.25;
  c.max_reference_chart_angle_rad=1.;c.feasibility_tolerance=1e-7;
  c.max_solve_time_sec=1.;c.max_working_set_recalculations=500;
  c.jerk_weight=2000.;c.jerk_difference_weight=.01;
  return c;
}
Pose6D pose(const Eigen::Vector3d& p) {return math::poseFromSe3(pinocchio::SE3(Eigen::Matrix3d::Identity(),p));}
Eigen::Vector3d xyz(const Pose6D& p) {return {p.x,p.y,p.z};}
PreviewReference reference(const Eigen::Vector3d& p,const Eigen::Vector3d& velocity) {
  PreviewReference r;r.count=25;
  for(std::size_t k=0;k<r.count;++k)r.knots[k]={.01*k,pose(p+.01*k*velocity)};
  return r;
}
PreviewContactConstraint authority(const Eigen::Vector3d& normal,double velocity=0,double acceleration=0) {
  PreviewContactConstraint a;a.enabled=true;a.normal_stand=normal;a.count=121;
  for(std::size_t k=0;k<a.count;++k)a.knots[k]={.002*k,velocity+acceleration*.002*k};
  return a;
}
bool solved(const PreviewSolveResult& result) {
  if(!result.accepted())std::cerr<<"status="<<static_cast<int>(result.status)<<" nWSR="<<result.diagnostics.working_set_recalculations
      <<" solve_ms="<<result.diagnostics.solve_time_sec*1e3<<" violation="<<result.diagnostics.max_constraint_violation<<'\n';
  return result.accepted();
}
bool testObliquePlaneContinuous() {
  PreviewTrajectoryTracker tracker(config());const Eigen::Vector3d p{.4,-.2,.3};
  const Eigen::Vector3d n=Eigen::Vector3d(1,2,3).normalized();
  const Eigen::Vector3d tangent=n.cross(Eigen::Vector3d::UnitZ()).normalized();
  PreviewMotionState initial;initial.pose=pose(p);
  const auto ref=reference(p,.08*n+.04*tangent);const auto bound=authority(n);
  const auto result=tracker.plan(ref,initial,bound);CHECK(solved(result));
  CHECK(result.diagnostics.contact_constrained);
  // Two initial velocity Bernstein controls depend only on fixed v/a and become
  // separately checked constant inequalities, not rows handed to the solver.
  CHECK(result.diagnostics.contact_constraint_rows>=358);
  PreviewMotionSample sample;
  for(int k=0;k<=2400;++k) {
    CHECK(tracker.sample(.24*k/2400,sample));
    CHECK(n.dot(sample.linear_velocity)<=config().feasibility_tolerance);
    CHECK(sample.linear_velocity.cwiseAbs().maxCoeff()<=.6+1e-7);
  }
  CHECK(tangent.dot(xyz(sample.pose)-p)>.005);
  // The hard boundary applies to a coupled normal, not a guessed per-axis box.
  CHECK((xyz(sample.pose)-p).cwiseAbs().maxCoeff()>.001);
  return true;
}
bool testMovingPlaneAndChangedNormal() {
  PreviewTrajectoryTracker tracker(config());const Eigen::Vector3d p{.4,.1,.3};
  const Eigen::Vector3d n=Eigen::Vector3d(1,-2,0).normalized();
  const Eigen::Vector3d tangent=Eigen::Vector3d(2,1,0).normalized();
  PreviewMotionState initial;initial.pose=pose(p);initial.linear_velocity=.025*n+.04*tangent;initial.linear_acceleration=.01*n;
  const auto result=tracker.plan(reference(p,.08*n+.04*tangent),initial,authority(n,.025,.01));
  CHECK(solved(result));PreviewMotionSample sample;
  for(int k=0;k<=2400;++k) {
    const double t=.24*k/2400;CHECK(tracker.sample(t,sample));
    CHECK(n.dot(sample.linear_velocity)<=.025+.01*t+1e-7);
  }
  CHECK(tracker.sample(0,sample));CHECK((sample.linear_velocity-initial.linear_velocity).norm()<1e-12);
  // Change the coupled constraint matrix in a hotstart; an independent solver
  // or bounds-only hotstart would silently retain the previous normal.
  const auto other=authority(tangent);
  initial.linear_velocity.setZero();initial.linear_acceleration.setZero();
  CHECK(solved(tracker.plan(reference(p,.08*tangent+.04*n),initial,other)));
  for(int k=0;k<=1200;++k) {
    CHECK(tracker.sample(.24*k/1200,sample));CHECK(tangent.dot(sample.linear_velocity)<=1e-7);
  }
  return true;
}
bool testInfeasibleSpliceKeepsAcceptedState() {
  PreviewTrajectoryTracker tracker(config());const Eigen::Vector3d p{.4,.1,.3};
  const Eigen::Vector3d n=Eigen::Vector3d(1,1,0).normalized();
  PreviewMotionState initial;initial.pose=pose(p);
  const auto ref=reference(p,.04*Eigen::Vector3d::UnitZ());
  CHECK(solved(tracker.plan(ref,initial)));
  PreviewMotionSample before,after;CHECK(tracker.sample(.1,before));
  initial.linear_velocity=.05*n;
  const auto closing=tracker.plan(ref,initial,authority(n));
  CHECK(closing.status==PreviewSolveStatus::Infeasible);
  CHECK(tracker.sample(.1,after));CHECK(math::positionDistance(before.pose,after.pose)==0);
  initial.linear_velocity.setZero();initial.linear_acceleration=.1*n;
  CHECK(tracker.plan(ref,initial,authority(n)).status==PreviewSolveStatus::Infeasible);
  // Spatial lag is not forward velocity authority. A held pose may start ahead
  // of a retreating reference and escape, with no teleport or plane expansion.
  initial.linear_acceleration.setZero();initial.pose=pose(p+.01*n);
  CHECK(solved(tracker.plan(reference(p,-.04*n),initial,authority(n))));
  CHECK(tracker.sample(0,after));CHECK(math::positionDistance(after.pose,initial.pose)<1e-12);
  for(int k=0;k<=1200;++k) {
    CHECK(tracker.sample(.24*k/1200,after));CHECK(n.dot(after.linear_velocity)<=1e-7);
  }
  CHECK(n.dot(xyz(after.pose)-xyz(initial.pose))<-.005);
  auto malformed=authority(2*n);initial.pose=pose(p);
  CHECK(tracker.plan(ref,initial,malformed).status==PreviewSolveStatus::InvalidReference);
  malformed=authority(n);malformed.knots[3].time_sec=malformed.knots[2].time_sec;
  CHECK(tracker.plan(ref,initial,malformed).status==PreviewSolveStatus::InvalidReference);
  return true;
}
bool benchmark() {
  PreviewTrajectoryTracker tracker(config());const Eigen::Vector3d p{.4,.1,.3};
  std::vector<double> times;times.reserve(100);
  for(int i=0;i<100;++i) {
    const Eigen::Vector3d n=Eigen::Vector3d(1.,.3+.001*i,.2).normalized();
    const Eigen::Vector3d tangent=n.cross(Eigen::Vector3d::UnitZ()).normalized();
    PreviewMotionState initial;initial.pose=pose(p);
    const auto result=tracker.plan(reference(p,.03*n+.02*tangent),initial,authority(n));
    CHECK(solved(result));times.push_back(result.diagnostics.solve_time_sec*1e6);
  }
  std::sort(times.begin(),times.end());
  std::cout<<"contact_qp_us median="<<times[50]<<" p95="<<times[95]<<" max="<<times.back()<<'\n';
  return true;
}
bool testReducedEqualsCoupledAndFallback() {
  const Eigen::Vector3d p{.4,.1,.3};
  auto cfg=config();PreviewTrajectoryTracker fast(cfg),oracle(cfg);
  for(int i=0;i<12;++i) {
    const Eigen::Vector3d n=Eigen::Vector3d(1.,2.+.01*i,-3.).normalized();
    const Eigen::Vector3d tangent=n.cross(Eigen::Vector3d::UnitX()).normalized();
    PreviewMotionState initial;initial.pose=pose(p);initial.linear_velocity=.01*n+.015*tangent;
    initial.linear_acceleration=.002*n-.003*tangent;
    auto bound=authority(n*(i%2?1.+.25*cfg.feasibility_tolerance:1.),.02);
    const auto ref=reference(p,.07*n+.035*tangent);
    const auto a=fast.plan(ref,initial,bound);
    const auto b=oracle.plan(ref,initial,bound,PreviewContactSolveMode::CoupledOnly);
    CHECK(solved(a));CHECK(solved(b));CHECK(a.diagnostics.contact_decomposed);
    CHECK(!a.diagnostics.contact_coupled_fallback);
    PreviewPolynomialTrajectory x,y;CHECK(fast.exportTrajectory(x));CHECK(oracle.exportTrajectory(y));
    CHECK((x.jerk.topRows(24)-y.jerk.topRows(24)).cwiseAbs().maxCoeff()<.001);
    PreviewMotionSample first;CHECK(fast.sample(0,first));
    CHECK((first.linear_velocity-initial.linear_velocity).norm()<1e-12);
    CHECK((first.linear_acceleration-initial.linear_acceleration).norm()<1e-12);
  }
  // Large tangent demand violates an ORIGINAL stand-axis limit. The reduced
  // contact-only optimum must fail verification and use the coupled oracle.
  cfg.linear_tracking_tolerance_m=1.;cfg.max_linear_tracking_slack_m=1.;
  PreviewTrajectoryTracker capped(cfg),capped_oracle(cfg);
  PreviewMotionState initial;initial.pose=pose(p);initial.linear_velocity={.5,0,0};
  const auto ref=reference(p,{2.,0,0});const auto bound=authority(Eigen::Vector3d::UnitZ());
  const auto a=capped.plan(ref,initial,bound);
  const auto b=capped_oracle.plan(ref,initial,bound,PreviewContactSolveMode::CoupledOnly);
  CHECK(solved(a));CHECK(solved(b));CHECK(a.diagnostics.contact_coupled_fallback);
  CHECK(!a.diagnostics.contact_decomposed);
  PreviewPolynomialTrajectory x,y;CHECK(capped.exportTrajectory(x));CHECK(capped_oracle.exportTrajectory(y));
  CHECK((x.jerk.topRows(24)-y.jerk.topRows(24)).cwiseAbs().maxCoeff()<.001);
  for(int k=0;k<=2400;++k) {
    PreviewMotionSample s;CHECK(capped.sample(.24*k/2400,s));
    CHECK(s.linear_velocity.cwiseAbs().maxCoeff()<=cfg.max_linear_velocity_m_s+cfg.feasibility_tolerance);
    CHECK(s.linear_acceleration.cwiseAbs().maxCoeff()<=cfg.max_linear_acceleration_m_s2+cfg.feasibility_tolerance);
    CHECK(s.linear_jerk.cwiseAbs().maxCoeff()<=cfg.max_linear_jerk_m_s3+cfg.feasibility_tolerance);
  }
  return true;
}
}
int main() {
  if(!testObliquePlaneContinuous()||!testMovingPlaneAndChangedNormal()||
     !testInfeasibleSpliceKeepsAcceptedState()||!testReducedEqualsCoupledAndFallback()||!benchmark())return 1;
  std::cout<<"contact QP: oblique/source velocity bounds, escape from held lag, unchanged infeasible C2 splice PASS\n";
}
