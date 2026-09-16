#include "rb_servo/control/preview_execution_worker.hpp"
#include "rb_servo/config/config.hpp"
#include "rb_servo/control/follower_preview_reference.hpp"
#include "rb_servo/core/clock.hpp"
#include "rb_servo/math/se3.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
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
  c.contact_slew_jerk_m_s3=400.;
  c.trusted_future_sec=0.;   // legacy: the selected segment only; the window variant is tested explicitly
  c.max_angular_velocity_rad_s=1.4; c.max_angular_acceleration_rad_s2=40.; c.max_angular_jerk_rad_s3=4000.;
  c.linear_tracking_tolerance_m=.02; c.angular_tracking_tolerance_rad=.08;
  c.max_linear_tracking_slack_m=.08; c.max_angular_tracking_slack_rad=.25;
  c.max_reference_chart_angle_rad=1.; c.feasibility_tolerance=1e-7;
  c.max_solve_time_sec=.5; c.max_working_set_recalculations=300;
  c.jerk_weight=2000.; c.jerk_difference_weight=10.;
  c.reference_trust_full_sec=.13; c.reference_trust_tail_sec=.24; c.reference_trust_tail=.1;
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

// Regression for the free-space run: changing only g=1 to g=1-eps used
// to reverse the 40 ms output from +9.57 to -1.94 mm/s.
bool testContactRestrictionConvergesToTheFreeCandidate() {
  setExternalSteadyNs(1000000000ULL);
  auto fc=followerConfig();CartesianChunkFollower f(fc);auto fr=frame();
  for(std::size_t k=0;k<fr.delta.size();++k)fr.delta[k].x=k<4?.0002:.008;
  f.submitDeltaFrame(fr,pose(.4));f.tick(.002);
  auto tc=trackerConfig();tc.jerk_difference_weight=.01;
  PreviewExecutionWorker worker(tc,fc,workerConfig());
  auto r=request(f);r.contact_normal_stand={1,0,0};
  CHECK(worker.trySubmit(f,r));PreviewExecutionResult free;CHECK(waitResult(worker,free));CHECK(free.accepted());
  for(const Eigen::Vector3d normal:{Eigen::Vector3d(1,0,0),Eigen::Vector3d(1,2,3).normalized(),
                                   Eigen::Vector3d(-1,2,-3).normalized()}) {
    r.contact_normal_stand=normal;
  double previous_error=1e9;
  for(double eps:{1e-2,1e-4,1e-6}) {
    r.contact_gate=1.-eps;
    CHECK(worker.trySubmit(f,r));PreviewExecutionResult cut;CHECK(waitResult(worker,cut));
    CHECK(cut.accepted());double error=0.;
    for(int k=0;k<=120;++k) {
      PreviewMotionSample a,b;CHECK(free.trajectory.sample(k*.002,a));CHECK(cut.trajectory.sample(k*.002,b));
      error=std::max(error,(a.linear_velocity-b.linear_velocity).norm());
    }
    std::cout<<"gate epsilon "<<eps<<" max velocity difference "<<error<<" m/s\n";
    CHECK(error<=previous_error+1e-6);previous_error=error;
    if(eps==1e-6)CHECK(error<1e-5);
  }
  }
  // An unselected tail can change radically without changing this prefix.
  auto other=fr;for(std::size_t k=4;k<other.delta.size();++k)other.delta[k].x=-.03;
  CartesianChunkFollower f2(fc);f2.submitDeltaFrame(other,pose(.4));f2.tick(.002);
  r=request(f2);CHECK(worker.trySubmit(f2,r));PreviewExecutionResult tail;CHECK(waitResult(worker,tail));CHECK(tail.accepted());
  for(int k=0;k<=120;++k) {
    PreviewMotionSample a,b;CHECK(free.trajectory.sample(k*.002,a));CHECK(tail.trajectory.sample(k*.002,b));
    CHECK(sameState(a,b,1e-7));
  }
  // TRUSTED FUTURE (2026-09-15 night): a 0.10 s window covers rows 0-2 of the
  // four-row execute window (row 3's central-difference velocity reads row 4), so a
  // tail changed from row 4 on still cannot move the plan; the window itself must be
  // exposed as the trusted prefix and must not exceed the configured value.
  {
    auto tw=tc;tw.trusted_future_sec=.10;
    PreviewExecutionWorker window(tw,fc,workerConfig());
    CartesianChunkFollower g1(fc),g2(fc);g1.submitDeltaFrame(fr,pose(.4));g1.tick(.002);
    g2.submitDeltaFrame(other,pose(.4));g2.tick(.002);
    auto r1=request(g1);r1.contact_normal_stand={1,0,0};auto r2=request(g2);r2.contact_normal_stand={1,0,0};
    CHECK(window.trySubmit(g1,r1));PreviewExecutionResult w1;CHECK(waitResult(window,w1));CHECK(w1.accepted());
    CHECK(window.trySubmit(g2,r2));PreviewExecutionResult w2;CHECK(waitResult(window,w2));CHECK(w2.accepted());
    CHECK(w1.trusted_prefix_sec>.05&&w1.trusted_prefix_sec<=.10+1e-9);
    for(int k=0;k<=120;++k) {
      PreviewMotionSample a,b;CHECK(w1.trajectory.sample(k*.002,a));CHECK(w2.trajectory.sample(k*.002,b));
      CHECK(sameState(a,b,1e-7));
    }
    auto bad=tw;bad.trusted_future_sec=1.;bool threw=false;
    try {PreviewExecutionWorker w(bad,fc,workerConfig());} catch(const std::invalid_argument&) {threw=true;}
    CHECK(threw);
  }
  return true;
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
  FollowerOutputKinematics phase;
  CHECK(first.phase_reference.count>2);
  CHECK(samplePreviewExecutionPhaseReference(first,1.004,1.,r.identity,r.gauge,1e-7,phase));
  auto rejected_forecast=first;rejected_forecast.status=PreviewExecutionWorkerStatus::SolveRejected;
  CHECK(samplePreviewExecutionPhaseReference(rejected_forecast,1.004,1.,r.identity,r.gauge,1e-7,phase));
  auto unrelated=r.identity;++unrelated.parent_plan_id;
  CHECK(samplePreviewExecutionPhaseReference(first,1.004,1.,unrelated,r.gauge,1e-7,phase));
  ++unrelated.source_recv_seq;
  CHECK(!samplePreviewExecutionPhaseReference(first,1.004,1.,unrelated,r.gauge,1e-7,phase));
  auto changed_gauge=r.gauge;++changed_gauge.revision;
  CHECK(!samplePreviewExecutionPhaseReference(first,1.004,1.,r.identity,changed_gauge,1e-7,phase));
  changed_gauge=r.gauge;changed_gauge.translation.x()=.01;
  CHECK(!samplePreviewExecutionPhaseReference(first,1.004,1.,r.identity,changed_gauge,1e-7,phase));
  CHECK(!samplePreviewExecutionPhaseReference(first,1.051,1.,r.identity,r.gauge,1e-7,phase));
  CHECK(!samplePreviewExecutionPhaseReference(first,1.02,1.05,r.identity,r.gauge,1e-7,phase));
  CHECK(!samplePreviewExecutionPhaseReference(first,.999,1.,r.identity,r.gauge,1e-7,phase));
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

bool testClampedDispatchSplice() {
  // 2026-09-10: the executor no longer brakes when the active plan loses closing
  // authority; it clamps its output and asks for a replan from THAT dispatched
  // state. The worker splices from the predecessor shifted by the held-back
  // displacement with its closing velocity cut to the knot-0 authority.
  setExternalSteadyNs(1000000000ULL);
  CartesianChunkFollower follower(followerConfig());follower.submitDeltaFrame(frame(),pose(.4));follower.tick(.002);
  PreviewExecutionWorker worker(trackerConfig(),followerConfig(),workerConfig());
  auto cold=request(follower);
  CHECK(worker.trySubmit(follower,cold));PreviewExecutionResult first;CHECK(waitResult(worker,first));CHECK(first.accepted());
  PreviewMotionSample at_splice;CHECK(first.trajectory.sample(.02,at_splice));
  CHECK(at_splice.linear_velocity.x()>1e-4);  // the predecessor is closing along +x
  // The force gate then removes all +x authority: the canonical follower stops.
  follower.setPlanRateGate(0);follower.tick(.002);
  CHECK(follower.outputKinematics().velocity.x==0);
  setExternalSteadyNs(1020000000ULL);
  auto contact=request(follower,1.02);contact.cold_start=false;contact.identity.parent_plan_id=first.identity.request_id;
  contact.predecessor=first.trajectory;contact.predecessor_origin_sec=first.splice_at_sec;
  contact.contact_gate=.5;contact.contact_normal_stand={1,0,0};
  // Without the dispatched state the predecessor's closing velocity violates knot 0.
  // 2026-09-10 pm: no longer refused. The splice keeps the predecessor's state
  // (never clipped) and the authority is widened along the fastest realisable brake.
  CHECK(worker.trySubmit(follower,contact));PreviewExecutionResult out;CHECK(waitResult(worker,out));
  if(!out.accepted())std::cerr<<"unflagged contact splice status="<<static_cast<int>(out.status)
      <<" solve="<<static_cast<int>(out.diagnostics.status)<<" violation="<<out.diagnostics.max_contact_velocity_violation_m_s
      <<" rows="<<out.diagnostics.contact_constraint_rows<<" decomposed="<<out.diagnostics.contact_decomposed
      <<" coupled_fallback="<<out.diagnostics.contact_coupled_fallback<<" nwsr="<<out.diagnostics.working_set_recalculations
      <<" maxviol="<<out.diagnostics.max_constraint_violation<<" init v="<<out.initial.linear_velocity.transpose()
      <<" a="<<out.initial.linear_acceleration.transpose()<<'\n';
  CHECK(out.accepted());
  CHECK((out.initial.linear_velocity-at_splice.linear_velocity).norm()<1e-12);   // never clipped
  CHECK((out.initial.linear_acceleration-at_splice.linear_acceleration).norm()<1e-12);
  {
    const auto& tr=trackerConfig();const double tol=tr.feasibility_tolerance;
    const double v0=at_splice.linear_velocity.x();
    // Since 2026-09-15 night the authority is SLEWED from the dispatched closing state
    // (see slewContactAuthority) instead of widened along the fastest brake, so the
    // stop lands one to two planning intervals after the fastest-brake time and is
    // never faster than the authority itself; the plan must still sit under the
    // certified bound at every sample.
    const double t_brake=v0/tr.max_linear_acceleration_m_s2+2*tr.max_linear_acceleration_m_s2/tr.max_linear_jerk_m_s3+
        5*tr.planning_dt_sec;
    for(int k=0;k<=120;++k) {
      PreviewMotionSample s;CHECK(out.trajectory.sample(.002*k,s));
      const auto& cert=out.contact_authority;std::size_t hi=1;
      while(hi+1<cert.count && cert.knots[hi].time_sec<.002*k)++hi;
      const auto& a=cert.knots[hi-1];const auto& b=cert.knots[hi];
      const double u=std::clamp((.002*k-a.time_sec)/(b.time_sec-a.time_sec),0.,1.);
      CHECK(s.linear_velocity.x()<=(1-u)*a.upper_velocity_m_s+u*b.upper_velocity_m_s+tol);
      if(.002*k>=t_brake)CHECK(s.linear_velocity.x()<=tol);
    }
  }
  // A SPLICE THAT STILL CLOSES FASTER THAN THE NEW KNOT-0 BOUND IS WIDENED, NEVER
  // REFUSED (2026-09-10 pm; the dispatched-state offset and the contact clamp that
  // produced it are gone since 2026-09-11). The authority can fall between request and
  // splice, so the initial state legitimately exceeds it; refusing that as Infeasible
  // fed the expiry-brake cycle (27 refusals in 5 s under a hand push). Since
  // 2026-09-15 night the plan gets the SLEWED authority from that state (not the
  // fastest brake, which the 10 ms planning jerk could only meet by overshooting).
  {
    auto tighter=contact;
    tighter.contact_gate=0.0;                // authority 0 at every knot
    CHECK(worker.trySubmit(follower,tighter));CHECK(waitResult(worker,out));
    if(!out.accepted())std::cerr<<"widened splice worker status="<<static_cast<int>(out.status)
        <<" solve="<<static_cast<int>(out.diagnostics.status)
        <<" violation="<<out.diagnostics.max_contact_velocity_violation_m_s<<'\n';
    CHECK(out.accepted());CHECK(out.diagnostics.contact_constrained);
    // NOT clipped: the initial state is the predecessor's own sample, because that is
    // what the arm was sent.
    CHECK((out.initial.linear_velocity-at_splice.linear_velocity).norm()<1e-12);
    CHECK((out.initial.linear_acceleration-at_splice.linear_acceleration).norm()<1e-12);
    PreviewMotionSample started;CHECK(out.trajectory.sample(0.,started));
    CHECK(std::abs(started.pose.x-out.initial.pose.x)<1e-12);
    const auto& tr=trackerConfig();const double tol=tr.feasibility_tolerance;
    const double v0=at_splice.linear_velocity.x();
    const double t_brake=v0/tr.max_linear_acceleration_m_s2+
        2*tr.max_linear_acceleration_m_s2/tr.max_linear_jerk_m_s3+5*tr.planning_dt_sec;   // slewed stop, see above
    for(int k=0;k<=120;++k) {
      PreviewMotionSample sm;CHECK(out.trajectory.sample(.002*k,sm));
      CHECK(sm.linear_velocity.x()<=v0+2e-3+tol);          // never faster than dispatched
      if(.002*k>=t_brake)CHECK(sm.linear_velocity.x()<=tol);
    }
  }
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
    const double t=.24*k/24000;
    const auto& cert=out.contact_authority;std::size_t hi=1;
    while(hi+1<cert.count && cert.knots[hi].time_sec<t)++hi;
    const auto& a=cert.knots[hi-1];const auto& b=cert.knots[hi];
    const double u=(t-a.time_sec)/(b.time_sec-a.time_sec);
    const double bound=(1-u)*a.upper_velocity_m_s+u*b.upper_velocity_m_s;
    PreviewMotionSample output;CHECK(out.trajectory.sample(t,output));
    CHECK(output.linear_velocity.x()<=bound+trackerConfig().feasibility_tolerance);

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
  CHECK(result.phase_reference.count==original.phase_reference.count);
  for(std::size_t k=0;k<result.phase_reference.count;++k) {
    const auto& before=original.phase_reference.samples[k];const auto& after=result.phase_reference.samples[k];
    CHECK(before.relative_time_sec==after.relative_time_sec);
    auto expected=math::se3FromPose(before.kinematics.pose);
    expected.translation()+=target.translation;expected.rotation()=target.rotation*expected.rotation();
    CHECK(math::positionDistance(after.kinematics.pose,math::poseFromSe3(expected))<1e-12);
    CHECK(math::orientationDistanceRad(after.kinematics.pose,math::poseFromSe3(expected))<1e-10);
    for(const auto pair : {std::pair{before.kinematics.velocity,after.kinematics.velocity},
                           std::pair{before.kinematics.acceleration,after.kinematics.acceleration}}) {
      CHECK(pair.first.x==pair.second.x&&pair.first.y==pair.second.y&&pair.first.z==pair.second.z);
      CHECK(pair.first.rx==pair.second.rx&&pair.first.ry==pair.second.ry&&pair.first.rz==pair.second.rz);
    }
  }
  FollowerOutputKinematics phase;
  CHECK(samplePreviewExecutionPhaseReference(result,1.014,1.005,r.identity,target,1e-7,phase));
  CHECK(!samplePreviewExecutionPhaseReference(original,1.014,1.005,r.identity,target,1e-7,phase));
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
// CONTACT AUTHORITY SLEW (2026-09-15 night). Regression for the 19:54 floor bounce: the
// envelope is g x the free candidate's own closing controls and the splice is the
// dispatched state, so any g < 1 put the plan above its bound. Widening only the first
// ~3 ms along the fastest brake then demanded a cut the 10 ms planning jerk could only
// realise by overshooting: measured offline on this scenario, g=0.9 turned a 10 mm/s
// approach into a -69 mm/s retreat and g=0.5 into -183 mm/s. The slewed envelope keeps
// the constrained plan near g x nominal and never lets it retreat past the free plan.
bool testContactSlewKeepsClosingNearScaledNominal() {
  auto tc=trackerConfig();
  const Eigen::Vector3d n=Eigen::Vector3d(.18,.04,-.98).normalized();
  const auto line=[&](const Eigen::Vector3d& p0,const Eigen::Vector3d& v_prefix,const Eigen::Vector3d& v_cont,double prefix) {
    PreviewReference r;r.count=25;
    for(std::size_t k=0;k<r.count;++k) {
      const double t=.01*k;
      const Eigen::Vector3d p=t<=prefix?Eigen::Vector3d(p0+v_prefix*t):Eigen::Vector3d(p0+v_prefix*prefix+v_cont*(t-prefix));
      Pose6D knot;knot.x=p.x();knot.y=p.y();knot.z=p.z();
      r.knots[k]={t,knot};
    }
    return r;
  };
  const auto envelope=[&](const PreviewPolynomialTrajectory& nominal_path,double g,PreviewContactConstraint& contact) {
    contact.enabled=true;contact.normal_stand=n;contact.count=1;contact.knots[0]={0.,0.};
    for(std::size_t segment=0;segment<nominal_path.count;++segment) {
      const double start=segment*nominal_path.step_sec,end=(segment+1)*nominal_path.step_sec;
      for(double a=start;a<end;) {
        const double b=std::min(a+.002,end);PreviewMotionSample sample;
        if(!nominal_path.sample(a,sample))return false;
        const double v=n.dot(sample.linear_velocity),accel=n.dot(sample.linear_acceleration);
        const double jerk=n.dot(nominal_path.jerk.row(segment).head<3>()),dt=b-a;
        const double upper=g*std::max({0.,v,v+.5*dt*accel,v+dt*accel+.5*dt*dt*jerk});
        auto& previous=contact.knots[contact.count-1];
        previous.upper_velocity_m_s=std::max(previous.upper_velocity_m_s,upper);
        contact.knots[contact.count++]={b,upper};a=b;
      }
    }
    return true;
  };
  struct Case {const char* name;PreviewMotionState initial;PreviewReference ref;double g;double floor_m_s;double late_ratio;};
  const auto at=[](double x,double y,double z){Pose6D p;p.x=x;p.y=y;p.z=z;return p;};
  PreviewMotionState approach;approach.pose=at(.35,.10,-.2103);
  approach.linear_velocity={.02,0.,-.0065};approach.linear_acceleration={.1,0.,.1};   // closing ~10 mm/s
  const auto descending=line({.35,.10,-.2111},{.02,0.,-.017},{.02,0.,-.005},.02);
  PreviewMotionState impact;impact.pose=at(.35,.10,-.2141);
  impact.linear_velocity={0.,0.,-.043};impact.linear_acceleration={0.,0.,2.};          // 42 mm/s onto a surface
  const auto stationary=line({.35,.10,-.2145},{0.,0.,0.},{0.,0.,.007},.02);
  const Case cases[]={
    {"approach g=.98",approach,descending,.98,-.001,.8},
    {"approach g=.90",approach,descending,.90,-.001,.8},
    {"approach g=.50",approach,descending,.50,-.005,.8},
    {"impact g=.62",impact,stationary,.62,-.015,0.},
  };
  for(const auto& c:cases) {
    PreviewTrajectoryTracker nominal_tracker(tc),tracker(tc);
    const auto nominal=nominal_tracker.plan(c.ref,c.initial,{},PreviewContactSolveMode::Automatic,.05);
    PreviewPolynomialTrajectory nominal_path;
    CHECK(nominal.accepted());CHECK(nominal_tracker.exportTrajectory(nominal_path));
    PreviewContactConstraint contact;CHECK(envelope(nominal_path,c.g,contact));
    const double v0=n.dot(c.initial.linear_velocity),a0=n.dot(c.initial.linear_acceleration);
    CHECK(slewContactAuthority(contact,v0,a0,tc,.002));
    // The slewed knot 0 admits the dispatched state; nothing below the envelope is admitted.
    CHECK(contact.knots[0].upper_velocity_m_s>=v0-tc.feasibility_tolerance);
    const auto solved=tracker.plan(c.ref,c.initial,contact,PreviewContactSolveMode::Automatic,.05);
    PreviewPolynomialTrajectory path;
    if(!solved.accepted())std::cerr<<c.name<<" status="<<static_cast<int>(solved.status)<<'\n';
    CHECK(solved.accepted());CHECK(tracker.exportTrajectory(path));
    double worst_retreat=0.,worst_gap=0.;
    for(int k=0;k<=120;++k) {
      const double t=.002*k;PreviewMotionSample a,b;
      CHECK(nominal_path.sample(t,a));CHECK(path.sample(t,b));
      const double vn=n.dot(a.linear_velocity),vc=n.dot(b.linear_velocity);
      std::size_t hi=1;while(hi+1<contact.count&&contact.knots[hi].time_sec<t)++hi;
      const auto& ka=contact.knots[hi-1];const auto& kb=contact.knots[hi];
      const double u=std::clamp((t-ka.time_sec)/(kb.time_sec-ka.time_sec),0.,1.);
      CHECK(vc<=(1-u)*ka.upper_velocity_m_s+u*kb.upper_velocity_m_s+tc.feasibility_tolerance);
      worst_retreat=std::min(worst_retreat,vc-std::min(vn,0.));   // retreat beyond the free plan's own
      if(t>=.15)worst_gap=std::max(worst_gap,c.g*vn-vc);           // late-horizon tracking of g x nominal
    }
    std::cout<<c.name<<" worst retreat beyond nominal "<<worst_retreat*1e3<<" mm/s, late gap to g*nominal "<<worst_gap*1e3<<" mm/s\n";
    CHECK(worst_retreat>=c.floor_m_s);
    if(c.late_ratio>0.) {
      PreviewMotionSample a,b;CHECK(nominal_path.sample(.2,a));CHECK(path.sample(.2,b));
      CHECK(n.dot(b.linear_velocity)>=c.late_ratio*c.g*n.dot(a.linear_velocity)-.001);
    }
  }
  // Refusals: a cap above the physical jerk limit or a non-positive cap is not a slew.
  PreviewContactConstraint contact;contact.enabled=true;contact.normal_stand=n;contact.count=2;
  contact.knots[0]={0.,0.};contact.knots[1]={.24,0.};
  auto bad=tc;bad.contact_slew_jerk_m_s3=0.;CHECK(!slewContactAuthority(contact,.01,0.,bad,.002));
  bad.contact_slew_jerk_m_s3=bad.max_linear_jerk_m_s3*2.;CHECK(!slewContactAuthority(contact,.01,0.,bad,.002));
  bool threw=false;
  try {PreviewExecutionWorker worker(bad,followerConfig(),workerConfig());} catch(const std::invalid_argument&) {threw=true;}
  CHECK(threw);
  return true;
}

bool wallClockBenchmark() {
  setExternalSteadyNs(0);
  const auto root=std::filesystem::path(__FILE__).parent_path().parent_path();
  const auto config=loadConfigFromYaml((root/"config/stack_real.yaml").string());
  PreviewExecutionConfig execution;
  for(const auto& p:config.cartesian_control.tcp_pose_target_profiles)
    if(p.name=="flow_infer_preview")execution=p.ruckig_follower.preview_execution;
  CHECK(execution.enable);
  auto fc=followerConfig();
  PreviewExecutionWorker worker(execution.tracker,fc,{.002,execution.worker_poll_period_sec,
      execution.max_result_age_sec,static_cast<std::size_t>(execution.max_source_rows)});
  for(double gate:{1.,.99,.5,0.}) {
    std::vector<double> times;int accepted=0,rejected=0;
    for(int k=0;k<40;++k) {
      CartesianChunkFollower follower(fc);auto fr=frame();
      fr.recv_time=PreviewExecutionWorker::monotonicNowSec();
      follower.submitDeltaFrame(fr,pose(.4));follower.tick(.002);
      const double generated=PreviewExecutionWorker::monotonicNowSec();
      auto r=request(follower,generated);r.splice_at_sec=generated+execution.splice_lead_sec;
      r.valid_until_sec=generated+execution.max_result_age_sec;
      r.contact_gate=gate;r.contact_normal_stand=Eigen::Vector3d(1,2,3).normalized();
      CHECK(worker.trySubmit(follower,r));PreviewExecutionResult result;CHECK(waitResult(worker,result));
      times.push_back((result.completed_at_sec-generated)*1e3);
      if(result.accepted()) {++accepted;CHECK(result.completed_at_sec<r.splice_at_sec);}
      else ++rejected;
    }
    std::sort(times.begin(),times.end());
    std::cout<<"wall-clock gate="<<gate<<" accepted="<<accepted<<" rejected="<<rejected
      <<" worker_ms median="<<times[times.size()/2]<<" p95="<<times[37]<<" max="<<times.back()
      <<" splice_budget_ms="<<execution.splice_lead_sec*1000<<'\n';
    CHECK(accepted>0); // throughput is reported; this is not a WCET guarantee.
  }
  return true;
}
} // namespace

int main(int argc,char** argv) {
  if(argc==2 && std::string(argv[1])=="--wall-clock-benchmark")return wallClockBenchmark()?0:1;
  const bool okay=testContactSlewKeepsClosingNearScaledNominal()&&testContactRestrictionConvergesToTheFreeCandidate()&&testSnapshotAndExport()&&testWorkerSpliceAndAdmission()&&testRefusalsAndBoundedSnapshots()&&
      testPhysicalContactAndBrakePredecessor()&&testClampedDispatchSplice()&&testVelocityAuthorityAtSourceZeroCrossing()&&testGaugeTransportPreservesC2AndIdentity();
  setExternalSteadyNs(0);
  if (!okay) return 1;
  std::cout<<"preview worker: fixed snapshots, future C2 splice, stale/late refusal, and RT C++ allocation audit PASS\n";
}
