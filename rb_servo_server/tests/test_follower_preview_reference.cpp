// Native, hardware-free preview tests. No backend, sockets or model calls.
#include "rb_servo/control/follower_preview_reference.hpp"
#include "rb_servo/math/se3.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

using namespace rb_servo;
using namespace rb_servo::control;
namespace {
constexpr double dt = .002;
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
double difference(const Vec6& a, const Vec6& b) {
  const double d[] = {a.x-b.x,a.y-b.y,a.z-b.z,a.rx-b.rx,a.ry-b.ry,a.rz-b.rz};
  double result = 0.; for (double x : d) result = std::max(result,std::abs(x));
  return result;
}
void same(const FollowerOutputKinematics& a, const FollowerOutputKinematics& b,
          double tolerance = 1e-11) {
  require(math::positionDistance(a.pose,b.pose)<tolerance,"preview position mismatch");
  require(math::orientationDistanceRad(a.pose,b.pose)<tolerance,"preview rotation mismatch");
  require(difference(a.velocity,b.velocity)<tolerance,"preview physical velocity mismatch");
  require(difference(a.acceleration,b.acceleration)<100*tolerance,
          "preview physical acceleration mismatch");
}
CartesianChunkFollowerConfig config() {
  CartesianChunkFollowerConfig c;
  c.lin={.6,12.,2000.};c.ang={1.4,40.,4000.};
  c.window={0,4,4,1};c.fresh_chunk_replan=true;c.continuous_hold_resume=true;
  c.core_time_stretch_enable=true;c.core_time_stretch_max_ratio=4.;
  return c;
}
Pose6D origin(double yaw = 0.) {
  return math::poseFromSe3(pinocchio::SE3(
      Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix(),
      Eigen::Vector3d(.3,-.2,.4)));
}
ChunkFrame frame(std::uint64_t seq, int count = 24, bool turning = true) {
  ChunkFrame f;f.wire_seq=seq;f.recv_seq=seq+100;f.recv_time=40.;f.policy_dt=.0334;
  for(int i=0;i<count;++i) {
    f.delta.push_back(turning ? Vec6{.001+.0004*std::sin(.5*i),-.0003,.0002,
                                     .002*std::cos(.2*i),.003,.004}
                              : Vec6{.001,0,0,0,0,0});
    f.grip.push_back(10.+i);
  }
  return f;
}
FollowerPreviewReferenceRequest request() {
  FollowerPreviewReferenceRequest r;
  r.sample_count=25;r.sample_period_sec=.010;r.servo_period_sec=dt;
  r.generated_at_sec=40.02;r.valid_until_sec=40.22;r.epoch=7;r.revision=13;
  return r;
}
void sameObservable(const CartesianChunkFollower& a, const CartesianChunkFollower& b) {
  same(a.outputKinematics(),b.outputKinematics());
  require(a.active()==b.active() && a.holdPaused()==b.holdPaused() &&
          a.windowIndex()==b.windowIndex() && a.windowConsumed()==b.windowConsumed() &&
          a.windowWireSeq()==b.windowWireSeq() && a.windowRecvSeq()==b.windowRecvSeq() &&
          a.currentGrip()==b.currentGrip() && a.tInSegment()==b.tInSegment() &&
          a.planRateGate()==b.planRateGate() && a.advanceGate()==b.advanceGate() &&
          a.advanceDirection()==b.advanceDirection() && a.planShift()==b.planShift() &&
          a.foldShift()==b.foldShift(),"preview changed live phase/gate/gripper state");
  const auto& x=a.diag();const auto& y=b.diag();
  require(x.segments==y.segments && x.stall_count==y.stall_count && x.stall==y.stall &&
          x.solve_failure_count==y.solve_failure_count && x.infeasible_fault==y.infeasible_fault &&
          x.actual_lead_fault==y.actual_lead_fault &&
          x.consecutive_actual_lead_errors==y.consecutive_actual_lead_errors &&
          x.consecutive_projection_errors==y.consecutive_projection_errors,
          "speculative diagnostics escaped into the live follower");
}

void regularAndPure() {
  CartesianChunkFollower live(config());live.submitDeltaFrame(frame(1),origin());
  for(int i=0;i<13;++i) live.tick(dt);
  auto untouched=live;
  const auto r=request();const auto preview=makeFollowerPreviewReference(live,r);
  require(preview.status==FollowerPreviewReferenceStatus::Ready && preview.samples.size()==25,
          "regular preview unavailable");
  require(preview.canonical_ticks==120 && preview.fractional_ticks==0,
          "10-ms knots did not use the 500-Hz rollout");
  require(preview.samples.front().relative_time_sec==0. &&
          preview.samples.back().relative_time_sec==.24,"preview time grid shifted");
  same(preview.samples.front().kinematics,live.outputKinematics());
  const auto& first_pose=preview.samples.front().kinematics.pose;
  const auto& live_pose=live.lastPose();
  require(first_pose.x==live_pose.x && first_pose.y==live_pose.y && first_pose.z==live_pose.z &&
          first_pose.rx==live_pose.rx && first_pose.ry==live_pose.ry && first_pose.rz==live_pose.rz &&
          first_pose.quaternion_xyzw==live_pose.quaternion_xyzw,
          "sample zero changed the live pose's exact Euler/quaternion representation");
  auto oracle=live;
  for(std::size_t i=0;i<preview.samples.size();++i) {
    if(i) for(int k=0;k<5;++k) oracle.tick(dt);
    same(preview.samples[i].kinematics,oracle.outputKinematics());
  }
  sameObservable(live,untouched);
  // Check future state, not only a fingerprint: hidden trajectory/window state
  // must stay identical through a later fresh packet and reserve exhaustion.
  for(int i=0;i<350;++i) {
    if(i==17) {
      live.submitDeltaFrame(frame(2),live.lastPose());
      untouched.submitDeltaFrame(frame(2),untouched.lastPose());
    }
    live.tick(dt);untouched.tick(dt);sameObservable(live,untouched);
  }
}

void fractionalGrid() {
  CartesianChunkFollower live(config());auto f=frame(1);
  f.policy_dt=.040;live.submitDeltaFrame(f,origin());
  for(int i=0;i<6;++i) live.tick(dt);
  auto untouched=live;auto r=request();r.sample_period_sec=.007;r.sample_count=31;
  const auto preview=makeFollowerPreviewReference(live,r);
  require(preview.status==FollowerPreviewReferenceStatus::Ready && preview.fractional_ticks==15,
          "fractional knots were silently rounded to servo ticks");
  // Independent denser 1-kHz native rollout reaches all 7-ms knots exactly.
  // Policy boundaries are multiples of 2 ms here, avoiding a changed boundary
  // crossing schedule while checking off-grid pose and SO(3) derivatives.
  auto oracle=live;
  for(std::size_t i=0;i<preview.samples.size();++i) {
    if(i) for(int k=0;k<7;++k) oracle.tick(.001);
    require(std::abs(preview.samples[i].relative_time_sec-.007*i)<1e-14,
            "fractional knot timestamp drift");
    same(preview.samples[i].kinematics,oracle.outputKinematics(),1e-8);
  }
  sameObservable(live,untouched);live.tick(dt);untouched.tick(dt);sameObservable(live,untouched);

  // Also use non-integral policy boundaries and a slowed plan clock. A 1-kHz
  // rollout is not the oracle for that case: preserve the real 500-Hz boundary
  // schedule and sample odd 7-ms knots from its independent half-tick branch.
  CartesianChunkFollower gated(config());gated.submitDeltaFrame(frame(2),origin(3.12));
  for(int i=0;i<9;++i)gated.tick(dt);
  gated.setPlanRateGate(.37);
  gated.setAdvanceGate(.23,Eigen::Vector3d::UnitX());
  const auto gated_untouched=gated;
  const auto off_grid=makeFollowerPreviewReference(gated,r);
  require(off_grid.status==FollowerPreviewReferenceStatus::Ready &&
          off_grid.canonical_ticks==105 && off_grid.fractional_ticks==15,
          "gated fractional preview did not preserve its canonical timeline");
  auto on_grid_request=r;on_grid_request.sample_period_sec=.014;
  on_grid_request.sample_count=16;
  const auto on_grid=makeFollowerPreviewReference(gated,on_grid_request);
  require(on_grid.status==FollowerPreviewReferenceStatus::Ready,
          "gated canonical preview unavailable");
  auto native=gated;
  for(std::size_t tick=0;tick<=105;++tick) {
    if(tick)native.tick(dt);
    if(tick%7==0) {
      const std::size_t i=2*(tick/7);
      same(off_grid.samples[i].kinematics,native.outputKinematics());
      same(off_grid.samples[i].kinematics,on_grid.samples[i/2].kinematics);
    } else if(tick%7==3) {
      const std::size_t i=2*(tick/7)+1;
      auto half_tick=native;half_tick.tick(.001);
      same(off_grid.samples[i].kinematics,half_tick.outputKinematics());
    }
  }
  sameObservable(gated,gated_untouched);
}

void gatesTailAndFaultIsolation() {
  auto c=config();c.max_projection_error_m=1e-12;c.max_projection_error_rad=1e-12;
  c.max_consecutive_projection_errors=2;
  CartesianChunkFollower live(c);auto short_frame=frame(4,8);
  for(auto& d:short_frame.delta) d={.02,.03,0,.2,.1,0};
  live.submitDeltaFrame(short_frame,origin());live.tick(dt);
  live.setAdvanceGate(.2,Eigen::Vector3d::UnitX());live.setPlanRateGate(.37);
  auto untouched=live;auto r=request();r.sample_count=40;r.sample_period_sec=.040;
  r.valid_until_sec=42.;
  const auto preview=makeFollowerPreviewReference(live,r);
  require(preview.status==FollowerPreviewReferenceStatus::Ready,"gated preview failed");
  auto oracle=live;
  for(std::size_t i=0;i<preview.samples.size();++i) {
    if(i) for(int k=0;k<20;++k) oracle.tick(dt);
    same(preview.samples[i].kinematics,oracle.outputKinematics());
  }
  sameObservable(live,untouched);
  require(oracle.diag().infeasible_fault || oracle.diag().stall,
          "speculative fault/tail fixture did not reach a diagnostic event");

  live.setPlanRateGate(0.);r=request();const auto frozen=makeFollowerPreviewReference(live,r);
  for(const auto& sample:frozen.samples) {
    same(sample.kinematics,live.outputKinematics());
    require(difference(sample.kinematics.velocity,Vec6{})==0.,"zero clock gate advanced velocity");
  }
  // A small finite chunk exhausts its complete reserve, then has a conditional
  // ring-down tail. The helper labels it without changing live stall counters.
  CartesianChunkFollower tail(config());tail.submitDeltaFrame(frame(5,8,false),origin());tail.tick(dt);
  r=request();r.sample_count=101;r.valid_until_sec=42.;
  const auto tail_preview=makeFollowerPreviewReference(tail,r);
  require(tail_preview.status==FollowerPreviewReferenceStatus::Ready &&
          std::isfinite(tail_preview.first_stall_relative_time_sec) &&
          tail_preview.non_stalled_sample_count>0 &&
          tail_preview.non_stalled_sample_count<tail_preview.samples.size() &&
          !tail.diag().stall,"reserve ring-down was absent or changed the live follower");
}

void holdEpochAndValidity() {
  CartesianChunkFollower live(config());auto r=request();
  require(makeFollowerPreviewReference(live,r).status==FollowerPreviewReferenceStatus::Inactive,
          "inactive follower produced a usable preview");
  live.submitDeltaFrame(frame(8),origin());for(int i=0;i<10;++i)live.tick(dt);
  const auto old=makeFollowerPreviewReference(live,r);
  require(old.isCurrent(40.03,7,13,8,108),"current preview rejected");
  require(!old.isCurrent(40.03,8,13,8,108) && !old.isCurrent(40.03,7,14,8,108) &&
          !old.isCurrent(40.03,7,13,9,108) && !old.isCurrent(40.03,7,13,8,109) &&
          !old.isCurrent(r.valid_until_sec,7,13,8,108) &&
          !old.isCurrent(40.,7,13,8,108),"stale epoch/revision/packet/time was accepted");
  require(old.samples.front().time_before_valid_until &&
          !old.samples.back().time_before_valid_until,
          "future sample beyond the declared validity was not marked");
  const Pose6D held=live.lastPose();live.holdAtSentReference(held,40.03);
  const auto paused=makeFollowerPreviewReference(live,r);
  require(paused.status==FollowerPreviewReferenceStatus::Paused && paused.samples.empty(),
          "paused preview advanced or automatically resumed the source");
  live.submitDeltaFrame(frame(9),held);live.holdAtSentReference(held,40.08);
  require(live.resumeFromHold(held,40.09,.5,1e-5,1e-5)==HoldResumeResult::WarmResumed,
          "helper disturbed the immutable hold grace timer");
  live.tick(dt);++r.revision;
  const auto resumed=makeFollowerPreviewReference(live,r);
  require(resumed.status==FollowerPreviewReferenceStatus::Ready && resumed.source_wire_seq==9,
          "cached fresh frame was lost at resume");
  same(resumed.samples.front().kinematics,live.outputKinematics());
  require(math::positionDistance(live.lastPose(),held)<1e-12,"resume sample zero did not hold");
  live.holdAtSentReference(held,41.);live.holdAtSentReference(held,41.4);
  require(live.expireHoldPause(41.51,.5),"repeated hold extended its original grace");
  require(makeFollowerPreviewReference(live,r).status==FollowerPreviewReferenceStatus::Inactive,
          "expired hold produced a preview");
  auto invalid=request();invalid.valid_until_sec=invalid.generated_at_sec;
  require(makeFollowerPreviewReference(live,invalid).status==FollowerPreviewReferenceStatus::Expired,
          "expired request was accepted");
  invalid=request();invalid.servo_period_sec=0.;
  require(makeFollowerPreviewReference(live,invalid).status==FollowerPreviewReferenceStatus::InvalidRequest,
          "missing servo sampling contract was silently defaulted");
  invalid=request();invalid.sample_count=1000000;
  require(makeFollowerPreviewReference(live,invalid).status==FollowerPreviewReferenceStatus::InvalidRequest,
          "unbounded preview allocation request accepted");
  invalid=request();invalid.servo_period_sec=1e-9;
  require(makeFollowerPreviewReference(live,invalid).status==FollowerPreviewReferenceStatus::InvalidRequest,
          "unbounded canonical rollout work accepted");
  invalid=request();invalid.generated_at_sec=std::numeric_limits<double>::quiet_NaN();
  require(makeFollowerPreviewReference(live,invalid).status==FollowerPreviewReferenceStatus::InvalidRequest,
          "nonfinite preview generation time accepted");

  // A malformed source derivative is an unavailable forecast, never a partial
  // Ready trajectory or a mutation of the caller's current pose/phase.
  live.submitDeltaFrame(frame(10),held);live.tick(dt);
  const Pose6D last_good_pose=live.lastPose();const auto last_good_index=live.windowIndex();
  live.setPlanRateGate(std::numeric_limits<double>::quiet_NaN());
  const auto bad_sample=makeFollowerPreviewReference(live,request());
  require(bad_sample.status==FollowerPreviewReferenceStatus::NonfiniteSample && bad_sample.samples.empty(),
          "nonfinite source derivative produced a usable or partial preview");
  require(math::positionDistance(live.lastPose(),last_good_pose)==0. &&
          live.windowIndex()==last_good_index && std::isnan(live.planRateGate()),
          "invalid preview modified its source instead of reporting failure");
}

void foldAndNearPi() {
  CartesianChunkFollower live(config());live.submitDeltaFrame(frame(1),origin(3.12));
  for(int i=0;i<15;++i)live.tick(dt);
  const auto r=request();const auto before=makeFollowerPreviewReference(live,r);
  const Eigen::Vector3d dp(.013,-.007,.005);
  const Eigen::Quaterniond dR(Eigen::AngleAxisd(.09,Eigen::Vector3d(.3,.7,.2).normalized()));
  require(live.absorbOffset(dp,dR),"active follower declined test fold");
  const auto after=makeFollowerPreviewReference(live,r);
  auto folded_native=live;
  for(std::size_t i=0;i<before.samples.size();++i) {
    if(i)for(int k=0;k<5;++k)folded_native.tick(dt);
    same(after.samples[i].kinematics,folded_native.outputKinematics());
    const auto& p=before.samples[i].kinematics;const auto& q=after.samples[i].kinematics;
    auto expected=math::se3FromPose(p.pose);
    expected.translation()+=dp;expected.rotation()=dR.toRotationMatrix()*expected.rotation();
    const double position_error=math::positionDistance(math::poseFromSe3(expected),q.pose);
    const double rotation_error=math::orientationDistanceRad(math::poseFromSe3(expected),q.pose);
    // Re-solving future segments after a gauge shift has roundoff sensitivity
    // in Ruckig/SO(3), even though the current segment is shifted exactly.
    // Match the existing native fold regression's 1-nm / 1-nrad tolerance;
    // a stricter 1e-11 rad rejected a 3.1e-11 rad tail difference on this fixture.
    if (position_error>=1e-9 || rotation_error>=1e-9) {
      std::ostringstream error;
      error<<"preview fold changed the nominal gauge or applied it twice: sample="<<i
           <<" position_m="<<position_error<<" rotation_rad="<<rotation_error;
      throw std::runtime_error(error.str());
    }
    const double velocity_error=difference(p.velocity,q.velocity);
    const double acceleration_error=difference(p.acceleration,q.acceleration);
    // Knot zero is the same in-flight state and must retain its derivatives.
    // Future solves can select slightly different floating-point roots after
    // equivalent frame transforms (this fixture: 3.85e-8 rad/s at 240 ms).
    // The helper-to-folded-native comparison above remains at 1e-11; only the
    // separate gauge-equivalence comparison allows 1e-7 m/s or rad/s.
    const double velocity_tolerance=i==0 ? 1e-11 : 1e-7;
    if(velocity_error>=velocity_tolerance || acceleration_error>=1e-7) {
      std::ostringstream error;
      error<<"fold changed the physical derivative body convention: sample="<<i
           <<" velocity="<<velocity_error<<" acceleration="<<acceleration_error;
      throw std::runtime_error(error.str());
    }
  }
  // Fresh interruptions near the principal-angle branch retain finite physical
  // derivatives; preview must never mutate the real tangent or consume pointer.
  for(int seq=2;seq<35;++seq) {
    live.submitDeltaFrame(frame(seq),live.lastPose());
    for(int k=0;k<11;++k)live.tick(dt);
    auto untouched=live;const auto preview=makeFollowerPreviewReference(live,r);
    require(preview.status==FollowerPreviewReferenceStatus::Ready,"near-pi preview failed");
    same(preview.samples.front().kinematics,live.outputKinematics());
    sameObservable(live,untouched);live.tick(dt);untouched.tick(dt);sameObservable(live,untouched);
  }
}

void benchmark() {
  CartesianChunkFollower live(config());live.submitDeltaFrame(frame(1),origin());
  for(int i=0;i<15;++i)live.tick(dt);
  const auto r=request();std::vector<double> us;
  for(int i=0;i<64;++i) {
    const auto start=std::chrono::steady_clock::now();
    const auto preview=makeFollowerPreviewReference(live,r);
    const auto end=std::chrono::steady_clock::now();
    require(preview.status==FollowerPreviewReferenceStatus::Ready,"benchmark preview failed");
    us.push_back(std::chrono::duration<double,std::micro>(end-start).count());
  }
  std::sort(us.begin(),us.end());
  std::cout<<"preview_25x10ms_500Hz_us median="<<us[us.size()/2]
           <<" p95="<<us[us.size()*95/100]<<" max="<<us.back()
           <<" (offline informational; no RT acceptance)\n";
}
}  // namespace

int main() {
  try {
    regularAndPure();fractionalGrid();gatesTailAndFaultIsolation();holdEpochAndValidity();
    foldAndNearPi();benchmark();
    std::cout<<"follower preview reference tests passed\n";
    return 0;
  } catch(const std::exception& e) {
    std::cerr<<"follower preview reference: "<<e.what()<<'\n';return 1;
  }
}
