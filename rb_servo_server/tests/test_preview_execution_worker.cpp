#include "rb_servo/control/preview_execution_worker.hpp"
#include "rb_servo/control/follower_preview_reference.hpp"
#include "rb_servo/core/clock.hpp"
#include "rb_servo/math/se3.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <new>
#include <thread>

// The planner deliberately allocates on its own thread. This audit counts only
// the calling (servo analogue) thread, including linked C++ new calls; it does
// not claim to intercept every libc allocation or establish a WCET bound.
namespace allocation_audit {
thread_local bool enabled = false;
thread_local std::size_t count = 0;
void record() { if (enabled) ++count; }
}
void* operator new(std::size_t size) {
  if (void* p = std::malloc(std::max<std::size_t>(size, 1))) { allocation_audit::record(); return p; }
  throw std::bad_alloc();
}
void* operator new[](std::size_t size) { return ::operator new(size); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
void* operator new(std::size_t size, std::align_val_t align) {
  void* p = nullptr;
  if (posix_memalign(&p, static_cast<std::size_t>(align), std::max<std::size_t>(size, 1))) throw std::bad_alloc();
  allocation_audit::record(); return p;
}
void* operator new[](std::size_t size, std::align_val_t align) { return ::operator new(size, align); }
void operator delete(void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }

namespace {
using namespace rb_servo;
using namespace rb_servo::control;
#define CHECK(condition) do { if (!(condition)) { std::cerr << "CHECK failed: " #condition << " at " << __LINE__ << '\n'; return false; } } while (false)

PreviewTrackerConfig trackerConfig() {
  PreviewTrackerConfig c;
  c.max_linear_velocity_m_s=.6; c.max_linear_acceleration_m_s2=12.; c.max_linear_jerk_m_s3=2000.;
  c.max_angular_velocity_rad_s=1.4; c.max_angular_acceleration_rad_s2=40.; c.max_angular_jerk_rad_s3=4000.;
  c.linear_tracking_tolerance_m=.02; c.angular_tracking_tolerance_rad=.08;
  c.max_linear_tracking_slack_m=.08; c.max_angular_tracking_slack_rad=.25;
  c.max_reference_chart_angle_rad=1.; c.feasibility_tolerance=1e-7;
  c.max_solve_time_sec=.5; c.max_working_set_recalculations=300;
  c.jerk_weight=2000.; c.jerk_difference_weight=10.;
  return c;
}
CartesianChunkFollowerConfig followerConfig() {
  CartesianChunkFollowerConfig c;
  c.lin={.6,12.,2000.}; c.ang={1.4,40.,4000.};
  c.window={0,8,4,1}; c.fresh_chunk_replan=true;
  return c;
}
PreviewExecutionWorkerConfig workerConfig() { return {.002,.0005,.05,32}; }
Pose6D pose(double x) { return math::poseFromSe3(pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d{x,.1,.3})); }
ChunkFrame frame(std::size_t count=24) {
  ChunkFrame f; f.policy_dt=1./30.; f.wire_seq=17; f.recv_seq=8; f.recv_time=1.;
  for (std::size_t k=0;k<count;++k) { f.pose.push_back(pose(.4+.001*k)); f.grip.push_back(.5); f.delta.push_back(Vec6{.001,0,0,0,0,0}); }
  return f;
}
PreviewExecutionRequest request(const CartesianChunkFollower& f, double generation=1.) {
  PreviewExecutionRequest r;
  r.identity={3,7,f.windowWireSeq(),f.windowRecvSeq(),0,1};
  r.generated_at_sec=generation; r.splice_at_sec=generation+.01; r.valid_until_sec=generation+.05;
  r.cursor_time_sec=generation; r.cursor_rate=1.;
  r.history_count=1; r.history[0]={generation,f.outputKinematics()};
  r.cold_start=true; r.cold_initial.pose=pose(.4);
  return r;
}
bool waitResult(PreviewExecutionWorker& worker, PreviewExecutionResult& out) {
  for (int i=0;i<1000;++i) {
    if (worker.tryTake(out)) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}
bool sameState(const PreviewMotionState& a,const PreviewMotionState& b,double tol=1e-8) {
  return math::positionDistance(a.pose,b.pose)<tol && math::orientationDistanceRad(a.pose,b.pose)<tol &&
      (a.linear_velocity-b.linear_velocity).norm()<tol && (a.linear_acceleration-b.linear_acceleration).norm()<tol &&
      (a.angular_velocity_body-b.angular_velocity_body).norm()<tol &&
      (a.angular_acceleration_body-b.angular_acceleration_body).norm()<tol;
}

bool testSnapshotAndExport() {
  auto cfg=followerConfig(); CartesianChunkFollower live(cfg), copy(cfg);
  live.submitDeltaFrame(frame(),pose(.4));
  for (int k=0;k<17;++k) live.tick(.002);
  live.setAdvanceGate(.4,{1,0,0}); live.setPlanRateGate(.8);
  CHECK(live.absorbOffset({.001,-.002,.003},Eigen::Quaterniond(math::exp3({.01,-.02,.03}))));
  CHECK(!copy.canCopySnapshotFrom(live)); copy.reserveSnapshotCapacity(32);
  CHECK(copy.canCopySnapshotFrom(live));
  allocation_audit::count=0; allocation_audit::enabled=true;
  copy=live;
  allocation_audit::enabled=false;
  CHECK(allocation_audit::count==0);
  for (int k=0;k<180;++k) {
    const auto a=live.tick(.002),b=copy.tick(.002);
    CHECK(math::positionDistance(a,b)<1e-12);
    CHECK(math::orientationDistanceRad(a,b)<1e-12);
    CHECK(live.windowIndex()==copy.windowIndex());
    CHECK(live.tInSegment()==copy.tInSegment());
  }
  PreviewTrajectoryTracker tracker(trackerConfig()); PreviewReference ref; ref.count=25;
  for (std::size_t k=0;k<25;++k) ref.knots[k]={.01*k,pose(.4+.01*k*.06)};
  PreviewMotionState seed; seed.pose=pose(.4); seed.linear_velocity={.06,0,0};
  CHECK(tracker.plan(ref,seed).accepted());
  PreviewPolynomialTrajectory exported; CHECK(tracker.exportTrajectory(exported));
  allocation_audit::count=0; allocation_audit::enabled=true;
  bool equal=true;
  for (int k=0;k<=1200;++k) {
    PreviewMotionSample a,b; const double t=.24*k/1200;
    equal=equal&&tracker.sample(t,a)&&exported.sample(t,b)&&sameState(a,b);
  }
  allocation_audit::enabled=false;
  CHECK(equal); CHECK(allocation_audit::count==0);
  PreviewMotionSample out; CHECK(!exported.sample(.241,out)); CHECK(!exported.sample(-.001,out));
  tracker.reset(); CHECK(!tracker.exportTrajectory(exported));
  CHECK(exported.sample(.1,out)); // published value is independent of solver lifetime
  return true;
}

bool testWorkerSpliceAndAdmission() {
  setExternalSteadyNs(1000000000ULL);
  CartesianChunkFollower follower(followerConfig()); follower.submitDeltaFrame(frame(),pose(.4)); follower.tick(.002);
  PreviewExecutionWorker worker(trackerConfig(),followerConfig(),workerConfig());
  auto r=request(follower);
  allocation_audit::count=0; allocation_audit::enabled=true;
  const bool submitted=worker.trySubmit(follower,r);
  allocation_audit::enabled=false;
  CHECK(submitted); CHECK(allocation_audit::count==0);
  PreviewExecutionResult first;
  allocation_audit::count=0; allocation_audit::enabled=true;
  const bool received=waitResult(worker,first);
  allocation_audit::enabled=false;
  CHECK(received); CHECK(allocation_audit::count==0);
  if (!first.accepted()) std::cerr<<"worker status="<<static_cast<int>(first.status)<<" solve="<<static_cast<int>(first.diagnostics.status)<<'\n';
  CHECK(first.accepted()); CHECK(sameState(first.initial,r.cold_initial));
  CHECK(validatePreviewExecutionResult(first,1.,r.identity)==PreviewExecutionAcceptance::Ready);
  auto stale=r.identity; ++stale.epoch;
  CHECK(validatePreviewExecutionResult(first,1.,stale)==PreviewExecutionAcceptance::EpochMismatch);
  stale=r.identity; ++stale.gate_revision;
  CHECK(validatePreviewExecutionResult(first,1.,stale)==PreviewExecutionAcceptance::GateMismatch);
  stale=r.identity; ++stale.source_recv_seq;
  CHECK(validatePreviewExecutionResult(first,1.,stale)==PreviewExecutionAcceptance::SourceMismatch);
  stale=r.identity; ++stale.parent_plan_id;
  CHECK(validatePreviewExecutionResult(first,1.,stale)==PreviewExecutionAcceptance::ParentMismatch);
  CHECK(validatePreviewExecutionResult(first,first.splice_at_sec,r.identity)==PreviewExecutionAcceptance::Late);

  setExternalSteadyNs(1020000000ULL);
  for(int k=0;k<10;++k) follower.tick(.002);
  auto next=request(follower,1.02); next.identity.request_id=2; next.identity.parent_plan_id=1;
  next.cold_start=false; next.predecessor=first.trajectory; next.predecessor_origin_sec=first.splice_at_sec;
  CHECK(worker.trySubmit(follower,next)); PreviewExecutionResult second; CHECK(waitResult(worker,second));
  CHECK(second.accepted());
  PreviewMotionSample expected,start;
  CHECK(first.trajectory.sample(next.splice_at_sec-first.splice_at_sec,expected));
  CHECK(second.trajectory.sample(0.,start));
  CHECK(sameState(expected,second.initial)); CHECK(sameState(expected,start));
  allocation_audit::count=0; allocation_audit::enabled=true;
  PreviewExecutionResult unused; worker.tryTake(unused);
  for (int k=0;k<1000;++k) second.trajectory.sample(.0002*k,start);
  allocation_audit::enabled=false; CHECK(allocation_audit::count==0);
  return true;
}

bool testRefusalsAndBoundedSnapshots() {
  setExternalSteadyNs(1000000000ULL);
  CartesianChunkFollower follower(followerConfig()); follower.submitDeltaFrame(frame(),pose(.4)); follower.tick(.002);
  PreviewExecutionWorker worker(trackerConfig(),followerConfig(),workerConfig());
  auto r=request(follower); r.cold_initial.linear_velocity.x()=.01;
  CHECK(worker.trySubmit(follower,r)); PreviewExecutionResult out; CHECK(waitResult(worker,out));
  CHECK(out.status==PreviewExecutionWorkerStatus::SpliceUnavailable);
  r=request(follower); r.identity.source_recv_seq++;
  CHECK(worker.trySubmit(follower,r)); CHECK(waitResult(worker,out));
  CHECK(out.status==PreviewExecutionWorkerStatus::SourceMismatch);
  r=request(follower); r.history_count=129; CHECK(!worker.trySubmit(follower,r));
  r=request(follower); r.history[0].time_sec=.9;
  CHECK(worker.trySubmit(follower,r)); CHECK(waitResult(worker,out));
  CHECK(out.status==PreviewExecutionWorkerStatus::InvalidRequest);
  r=request(follower); setExternalSteadyNs(1020000000ULL);
  CHECK(worker.trySubmit(follower,r)); CHECK(waitResult(worker,out));
  CHECK(out.status==PreviewExecutionWorkerStatus::Late);
  follower.submitDeltaFrame(frame(33),pose(.4)); r=request(follower,1.02);
  CHECK(!worker.trySubmit(follower,r));
  bool rejected=false;
  try { PreviewExecutionWorker invalid(trackerConfig(),followerConfig(),{}); }
  catch(const std::invalid_argument&) { rejected=true; }
  CHECK(rejected);
  const auto diagnostics=worker.diagnostics();
  CHECK(diagnostics.request_invalid==2);
  CHECK(diagnostics.worker_status_counts[static_cast<std::size_t>(PreviewExecutionWorkerStatus::SpliceUnavailable)]==1);
  CHECK(diagnostics.worker_status_counts[static_cast<std::size_t>(PreviewExecutionWorkerStatus::SourceMismatch)]==1);
  CHECK(diagnostics.worker_status_counts[static_cast<std::size_t>(PreviewExecutionWorkerStatus::InvalidRequest)]==1);
  CHECK(diagnostics.worker_status_counts[static_cast<std::size_t>(PreviewExecutionWorkerStatus::Late)]==1);
  for(const auto count:diagnostics.solve_status_counts)CHECK(count==0);
  CHECK(!out.solve_attempted);
  return true;
}

bool testPhysicalContactAndBrakePredecessor() {
  setExternalSteadyNs(1000000000ULL);
  CartesianChunkFollower follower(followerConfig());follower.submitDeltaFrame(frame(),pose(.4));follower.tick(.002);
  // The raw follower reports no force direction here. The physical force
  // normal in the request must nevertheless activate contact constraints.
  CHECK(follower.advanceDirection().isZero(0.0));
  PreviewExecutionWorker worker(trackerConfig(),followerConfig(),workerConfig());
  auto r=request(follower);r.contact_gate=.9;r.contact_normal_stand={1,0,0};
  CHECK(worker.trySubmit(follower,r));PreviewExecutionResult out;CHECK(waitResult(worker,out));
  if(!out.accepted())std::cerr<<"contact worker status="<<static_cast<int>(out.status)
      <<" solve="<<static_cast<int>(out.diagnostics.status)<<'\n';
  CHECK(out.accepted());CHECK(out.diagnostics.contact_constrained);
  // A stopped brake carries an explicit stationary terminal hold. Its future
  // splice is valid even after the finite braking trajectory has completed.
  PreviewBrake brake(trackerConfig(),.002);PreviewMotionState initial;initial.pose=pose(.4);
  CHECK(brake.start(initial)==PreviewBrakeStatus::Ready);
  r=request(follower);r.cold_start=false;r.has_brake_predecessor=true;r.identity.parent_plan_id=99;
  r.predecessor_origin_sec=.5;CHECK(brake.exportTrajectory(r.brake_predecessor));
  CHECK(worker.trySubmit(follower,r));CHECK(waitResult(worker,out));CHECK(out.accepted());
  CHECK(sameState(initial,out.initial));
  // Translation may hold while angular motion retains a different, original
  // polynomial deadline. The worker composes both at the SAME future timestamp.
  PreviewTrajectoryTracker angular(trackerConfig());PreviewReference turning;turning.count=25;
  for(std::size_t k=0;k<turning.count;++k) {
    turning.knots[k]={.01*k,pose(.4)};turning.knots[k].pose.rx=.05*.01*k;
  }
  CHECK(angular.plan(turning,initial).accepted());PreviewPolynomialTrajectory angular_polynomial;
  CHECK(angular.exportTrajectory(angular_polynomial));
  CHECK(r.angular_predecessor.retainPolynomial(angular_polynomial,.995,1.03));
  CHECK(worker.trySubmit(follower,r));CHECK(waitResult(worker,out));CHECK(out.accepted());
  PreviewMotionSample expected;CHECK(angular_polynomial.sample(r.splice_at_sec-.995,expected));
  CHECK((out.initial.angular_velocity_body-expected.angular_velocity_body).norm()<1e-12);
  CHECK((out.initial.angular_acceleration_body-expected.angular_acceleration_body).norm()<1e-12);
  CHECK(math::orientationDistanceRad(out.initial.pose,expected.pose)<1e-10);
  CHECK((out.initial.linear_velocity-initial.linear_velocity).norm()==0);
  // A longer-lived translation hold cannot renew an expired angular plan.
  CHECK(r.angular_predecessor.retainPolynomial(angular_polynomial,.995,1.005));
  CHECK(worker.trySubmit(follower,r));CHECK(waitResult(worker,out));
  CHECK(out.status==PreviewExecutionWorkerStatus::SpliceUnavailable);
  r.has_brake_predecessor=false;
  CHECK(worker.trySubmit(follower,r));CHECK(waitResult(worker,out));
  CHECK(out.status==PreviewExecutionWorkerStatus::InvalidRequest);
  r.has_brake_predecessor=true;r.cold_start=true;
  CHECK(worker.trySubmit(follower,r));CHECK(waitResult(worker,out));
  CHECK(out.status==PreviewExecutionWorkerStatus::InvalidRequest);
  return true;
}

bool testVelocityAuthorityAtSourceZeroCrossing() {
  setExternalSteadyNs(1000000000ULL);
  auto changing=frame();
  for(std::size_t k=0;k<changing.delta.size();++k)changing.delta[k].x=k<3?.001:-.001;
  CartesianChunkFollower follower(followerConfig());follower.submitDeltaFrame(changing,pose(.4));follower.tick(.002);
  PreviewExecutionWorker worker(trackerConfig(),followerConfig(),workerConfig());
  auto r=request(follower);r.splice_at_sec=1.011; // Deliberately between canonical2ms ticks.
  r.contact_gate=.9;r.contact_normal_stand={1,0,0};
  CHECK(worker.trySubmit(follower,r));PreviewExecutionResult out;CHECK(waitResult(worker,out));CHECK(out.accepted());
  FollowerPreviewReferenceRequest forecast;
  forecast.sample_count=127;forecast.sample_period_sec=.002;forecast.servo_period_sec=.002;
  forecast.generated_at_sec=1.;forecast.valid_until_sec=1.05;forecast.epoch=3;forecast.revision=7;
  const auto canonical=makeFollowerPreviewReference(follower,forecast);
  CHECK(canonical.status==FollowerPreviewReferenceStatus::Ready);
  bool crosses=false;
  for(std::size_t k=1;k<canonical.samples.size();++k) {
    const auto a=canonical.samples[k-1].kinematics.velocity.x;
    const auto b=canonical.samples[k].kinematics.velocity.x;
    crosses=crosses||(a>0&&b<0)||(a<0&&b>0);
  }
  CHECK(crosses);
  for(int k=0;k<=24000;++k) {
    const double t=.24*k/24000,source_t=.011+t;
    const std::size_t index=std::min<std::size_t>(source_t/.002,canonical.samples.size()-2);
    const double u=(source_t-index*.002)/.002;
    const double source=(1-u)*canonical.samples[index].kinematics.velocity.x+
        u*canonical.samples[index+1].kinematics.velocity.x;
    PreviewMotionSample output;CHECK(out.trajectory.sample(t,output));
    CHECK(output.linear_velocity.x()<=std::max(0.,source)+trackerConfig().feasibility_tolerance);
  }
  return true;
}
bool testGaugeTransportPreservesC2AndIdentity() {
  setExternalSteadyNs(1000000000ULL);
  auto turning=frame();for(auto& delta:turning.delta){delta.rx=.001;delta.ry=-.002;delta.rz=.003;}
  CartesianChunkFollower follower(followerConfig());follower.submitDeltaFrame(turning,pose(.4));follower.tick(.002);
  PreviewExecutionWorker worker(trackerConfig(),followerConfig(),workerConfig());
  auto r=request(follower);r.gauge.revision=4;
  CHECK(worker.trySubmit(follower,r));PreviewExecutionResult result;CHECK(waitResult(worker,result));
  CHECK(result.accepted()&&result.solve_attempted);CHECK(result.gauge.revision==4);
  const auto original=result;
  const Eigen::Quaterniond q1(math::exp3(Eigen::Vector3d{.21,-.1,.03}));
  const Eigen::Quaterniond q2(math::exp3(Eigen::Vector3d{-.07,.04,.17}));
  PreviewExecutionGauge target;target.revision=6;target.translation={.003,-.002,.007};target.rotation=q2*q1;
  allocation_audit::count=0;allocation_audit::enabled=true;
  const bool moved=transportPreviewExecutionResult(result,target,1e-7);
  allocation_audit::enabled=false;CHECK(moved&&allocation_audit::count==0);
  CHECK(result.identity.epoch==original.identity.epoch && result.identity.gate_revision==original.identity.gate_revision);
  CHECK(result.identity.source_wire_seq==original.identity.source_wire_seq && result.identity.source_recv_seq==original.identity.source_recv_seq);
  CHECK(result.identity.parent_plan_id==original.identity.parent_plan_id && result.identity.request_id==original.identity.request_id);
  CHECK(result.generated_at_sec==original.generated_at_sec && result.splice_at_sec==original.splice_at_sec);
  CHECK(result.valid_until_sec==original.valid_until_sec && result.completed_at_sec==original.completed_at_sec);
  for(int k=0;k<=120;++k) {
    PreviewMotionSample before,after;CHECK(original.trajectory.sample(.002*k,before));CHECK(result.trajectory.sample(.002*k,after));
    auto expected=math::se3FromPose(before.pose);expected.translation()+=target.translation;expected.rotation()=target.rotation*expected.rotation();
    CHECK(math::positionDistance(after.pose,math::poseFromSe3(expected))<1e-12);
    CHECK(math::orientationDistanceRad(after.pose,math::poseFromSe3(expected))<1e-10);
    CHECK((after.linear_velocity-before.linear_velocity).norm()<1e-12);
    CHECK((after.linear_acceleration-before.linear_acceleration).norm()<1e-12);
    CHECK((after.linear_jerk-before.linear_jerk).norm()<1e-12);
    CHECK((after.angular_velocity_body-before.angular_velocity_body).norm()<1e-12);
    CHECK((after.angular_acceleration_body-before.angular_acceleration_body).norm()<1e-12);
    CHECK((after.angular_jerk_stand-target.rotation*before.angular_jerk_stand).norm()<1e-10);
  }
  PreviewMotionSample start;CHECK(result.trajectory.sample(0,start));CHECK(sameState(start,result.initial));
  CHECK(validatePreviewExecutionResult(result,1.005,r.identity)==PreviewExecutionAcceptance::Ready);
  auto current=r.identity;++current.gate_revision;
  CHECK(validatePreviewExecutionResult(result,1.005,current)==PreviewExecutionAcceptance::GateMismatch);
  current=r.identity;++current.epoch;CHECK(validatePreviewExecutionResult(result,1.005,current)==PreviewExecutionAcceptance::EpochMismatch);
  current=r.identity;++current.source_wire_seq;CHECK(validatePreviewExecutionResult(result,1.005,current)==PreviewExecutionAcceptance::SourceMismatch);
  current=r.identity;++current.parent_plan_id;CHECK(validatePreviewExecutionResult(result,1.005,current)==PreviewExecutionAcceptance::ParentMismatch);
  CHECK(validatePreviewExecutionResult(result,1.01,r.identity)==PreviewExecutionAcceptance::Late);
  auto corrupt=result;corrupt.gauge.revision=7;CHECK(!transportPreviewExecutionResult(corrupt,target,1e-7));
  corrupt=result;corrupt.gauge.translation.x()+=.001;CHECK(!transportPreviewExecutionResult(corrupt,target,1e-7));
  corrupt=result;corrupt.gauge.rotation.coeffs().setZero();CHECK(!transportPreviewExecutionResult(corrupt,target,1e-7));
  return true;
}
} // namespace

int main() {
  const bool okay=testSnapshotAndExport()&&testWorkerSpliceAndAdmission()&&testRefusalsAndBoundedSnapshots()&&
      testPhysicalContactAndBrakePredecessor()&&testVelocityAuthorityAtSourceZeroCrossing()&&testGaugeTransportPreservesC2AndIdentity();
  setExternalSteadyNs(0);
  if (!okay) return 1;
  std::cout<<"preview worker: fixed snapshots, future C2 splice, stale/late refusal, and RT C++ allocation audit PASS\n";
}
