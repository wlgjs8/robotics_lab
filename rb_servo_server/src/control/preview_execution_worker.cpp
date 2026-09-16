#include "rb_servo/control/preview_execution_worker.hpp"
#include "rb_servo/control/preview_execution_cursor.hpp"
#include "rb_servo/core/clock.hpp"
#include "rb_servo/math/se3.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>

namespace rb_servo::control {
bool buildContactEnvelope(PreviewContactConstraint& contact,const PreviewPolynomialTrajectory& nominal_path,
                          double gate,double servo_period_sec,double retreat_slack_m_s) {
  // A certified TWO-SIDED envelope of the free candidate's closing velocity. On each
  // servo subinterval use its three Bernstein controls; each end knot takes the
  // maximum (ceiling) / minimum (floor) of its adjacent intervals, hence the linear
  // envelope bounds the full quadratic, including between samples. Its conservatism
  // shrinks with the servo grid; at g=1 the free optimum is feasible exactly.
  //   ceiling = g x max(0, controls)              closing is scaled by the authority
  //   floor   = min(g x min, min) - slack         retreat keeps full authority
  // RETREAT FLOOR (2026-09-16). The ceiling alone left the plan free to back out of a
  // contact on the objective's own account: a replan that starts mid-brake after an
  // impact unwound its braking acceleration at the jerk the cost prefers - -72 mm/s
  // and 4.5 mm offline, -86 mm/s and a lost contact on the 14:38 run (left
  // 317.69-317.80 s) with the source 8 mm DEEPER. The floor makes the plan close at
  // no less than g x the free candidate, so retreat happens only when the free
  // candidate itself retreats (a source above the plan) - and then unscaled, so a
  // lift-off at g ~ 0 is never held back (the force law's yield is a gauge fold, never
  // a plan velocity). The scaled candidate g x v_nominal lies inside the tube at every
  // sample, and the slew lands on exactly that state, so the tube is always feasible.
  if(!contact.enabled||!std::isfinite(gate)||gate<0.0||gate>1.0||!(servo_period_sec>0.0)||
     !std::isfinite(retreat_slack_m_s)||retreat_slack_m_s<0.0||
     nominal_path.count==0||!(nominal_path.step_sec>0.0))return false;
  const Eigen::Vector3d& n=contact.normal_stand;
  contact.count=1;contact.knots[0]={0.,0.,std::numeric_limits<double>::infinity()};
  for(std::size_t segment=0;segment<nominal_path.count;++segment) {
    const double start=segment*nominal_path.step_sec;
    const double end=(segment+1)*nominal_path.step_sec;
    for(double a=start;a<end;) {
      const double b=std::min(a+servo_period_sec,end);
      if(!(b>a) || contact.count>=contact.knots.size())return false;
      PreviewMotionSample sample;
      if(!nominal_path.sample(a,sample))return false;
      const double v=n.dot(sample.linear_velocity);
      const double accel=n.dot(sample.linear_acceleration);
      const double jerk=n.dot(nominal_path.jerk.row(segment).head<3>());
      const double dt=b-a;
      const double c0=v,c1=v+.5*dt*accel,c2=v+dt*accel+.5*dt*dt*jerk;
      const double hi=std::max({c0,c1,c2}),lo=std::min({c0,c1,c2});
      const double ceiling=gate*std::max(0.0,hi);
      const double floor=std::min(gate*lo,lo)-retreat_slack_m_s;
      if(!std::isfinite(ceiling)||!std::isfinite(floor))return false;
      auto& previous=contact.knots[contact.count-1];
      previous.upper_velocity_m_s=std::max(previous.upper_velocity_m_s,ceiling);
      previous.lower_velocity_m_s=std::min(previous.lower_velocity_m_s,floor);
      contact.knots[contact.count++]={b,ceiling,floor};
      a=b;
    }
  }
  return std::isfinite(contact.knots[0].lower_velocity_m_s);
}

bool slewContactAuthority(PreviewContactConstraint& contact,double v0,double a0,
                          const PreviewTrackerConfig& tracker,double servo_period_sec,
                          const PreviewPolynomialTrajectory* nominal_path,double gate) {
  // WHY A SLEW AND NOT THE FASTEST BRAKE (2026-09-15 night). The envelope is g x the
  // free candidate's own closing controls and the splice is the dispatched state, so at
  // g < 1 the plan starts above its bound by (1-g) v0. The previous widening followed
  // the fastest brake (max jerk): it reached zero in ~3 ms and from there the plan had
  // to be under g*v_nominal at every 2 ms control. With jerk constant per planning
  // interval, a 1 mm/s cut required within 2 ms costs ~-500 m/s^3 for the whole 10 ms
  // (-25 mm/s), the acceleration must then be unwound over the next interval (-50 mm/s
  // at 20 ms), and each replan started from that retreat. Offline on the 19:54 run's
  // inputs: g=0.9 alone turned a 10 mm/s approach into -69 mm/s. The bound now leaves
  // the splice state along the two-interval profile below, whose jerk the QP can
  // spend without overshoot, and only then follows the envelope.
  //
  // TWO-SIDED SINCE 2026-09-16. The same profile also carries the retreat floor
  // (buildContactEnvelope): during the slew the plan is held in a tube of one
  // Bernstein margin around the profile, after it between the floor and the ceiling.
  // With the free candidate given, the landing is EXACTLY the scaled candidate's
  // state (g v, g a) at T - the one trajectory known to lie inside the two-sided tube
  // for the rest of the horizon, on the same jerk grid, so the constrained problem is
  // feasible by construction. Without it (one-sided authority) the landing is the
  // ceiling when the free coast would be above it, the floor when the coast would be
  // below it, and the coast itself otherwise.
  using Knot=PreviewContactConstraint::Knot;
  constexpr double kNoFloor=-std::numeric_limits<double>::infinity();
  const double a_max=tracker.max_linear_acceleration_m_s2,j_max=tracker.max_linear_jerk_m_s3;
  const double j_soft=tracker.contact_slew_jerk_m_s3;
  const double h_plan=tracker.planning_dt_sec,h=servo_period_sec;
  if(!std::isfinite(v0)||!std::isfinite(a0)||!(a_max>0.0)||!(j_max>0.0)||!(j_soft>0.0)||j_soft>j_max||
     !(h_plan>0.0)||!(h>0.0)||!contact.enabled||contact.count<2||contact.count>contact.knots.size()||
     !std::isfinite(gate)||gate<0.0||gate>1.0)return false;
  const double horizon=contact.knots[contact.count-1].time_sec;
  if(!(horizon>0.0))return false;
  if(nominal_path && (nominal_path->count==0 || !(nominal_path->step_sec>0.0) ||
                      nominal_path->count*nominal_path->step_sec+1e-9<horizon))return false;
  for(std::size_t k=0;k<contact.count;++k) {
    const auto& knot=contact.knots[k];
    if(!(knot.lower_velocity_m_s==kNoFloor ||
         (std::isfinite(knot.lower_velocity_m_s) && knot.lower_velocity_m_s<=knot.upper_velocity_m_s)))return false;
  }
  const auto locate=[&](double t) {
    std::size_t hi=1;
    while(hi+1<contact.count && contact.knots[hi].time_sec<t)++hi;
    return hi;
  };
  const auto bound=[&](double t) {
    const std::size_t hi=locate(t);
    const auto& a=contact.knots[hi-1];const auto& b=contact.knots[hi];
    const double u=std::clamp((t-a.time_sec)/(b.time_sec-a.time_sec),0.0,1.0);
    return (1.0-u)*a.upper_velocity_m_s+u*b.upper_velocity_m_s;
  };
  const auto floor_of=[&](double t) {
    const std::size_t hi=locate(t);
    const auto& a=contact.knots[hi-1];const auto& b=contact.knots[hi];
    if(!std::isfinite(a.lower_velocity_m_s)||!std::isfinite(b.lower_velocity_m_s))return kNoFloor;
    const double u=std::clamp((t-a.time_sec)/(b.time_sec-a.time_sec),0.0,1.0);
    return (1.0-u)*a.lower_velocity_m_s+u*b.lower_velocity_m_s;
  };
  const auto slope_of=[&](const auto& f,double t_at) {
    const double t_next=std::min(t_at+h,horizon);
    if(!(t_next>t_at))return 0.0;
    const double s=(f(t_next)-f(t_at))/(t_next-t_at);
    return std::isfinite(s)?std::clamp(s,-a_max,a_max):0.0;
  };
  // Two-interval profile: jerk j1 for tau, then j2 for tau (T = 2 tau, tau a whole
  // number of planning intervals) from (v0, a0) to (v_T, a_T):
  //   a_T = a0 + (j1 + j2) tau
  //   v_T = v0 + 1.5 a0 tau + 0.5 a_T tau + j1 tau^2
  // The landing point is the envelope at T with the envelope's own slope, unless the
  // plan's free coast v0 + a0 t is already under the envelope there, in which case the
  // coast itself is the profile (j1 = j2 = 0) and only the max with the envelope
  // shapes the bound. The shortest T whose jerks fit the soft cap wins; the physical
  // cap is a last resort before refusing the request.
  const int max_m=std::max(1,static_cast<int>(std::floor(horizon/(2.0*h_plan)+1e-9)));
  double tau=0.0,T=0.0,j1=0.0,j2=0.0;bool found=false;
  for(const double j_cap:{j_soft,j_max}) {
    for(int m=1;m<=max_m && !found;++m) {
      tau=m*h_plan;T=2.0*tau;
      const double free_T=v0+a0*T,env_T=bound(T),floor_T=floor_of(T);
      double v_T,a_T;
      if(nominal_path) {
        PreviewMotionSample landing;
        if(!nominal_path->sample(std::min(T,horizon),landing))return false;
        v_T=gate*contact.normal_stand.dot(landing.linear_velocity);
        a_T=gate*contact.normal_stand.dot(landing.linear_acceleration);
        if(!std::isfinite(v_T)||!std::isfinite(a_T))return false;
      }
      else if(free_T>env_T) {v_T=env_T;a_T=slope_of(bound,T);}
      else if(std::isfinite(floor_T) &&
              (free_T<floor_T || (a0<0.0 && free_T-a0*a0/(2.0*j_cap)-j_cap*h*h/4.0<floor_T))) {
        v_T=std::min(floor_T,env_T);a_T=slope_of(floor_of,T);
      }
      else {v_T=free_T;a_T=a0;}
      j1=(v_T-v0-1.5*a0*tau-0.5*a_T*tau)/(tau*tau);
      j2=(a_T-a0)/tau-j1;
      const double a_mid=a0+j1*tau;
      if(std::isfinite(j1)&&std::isfinite(j2)&&std::abs(j1)<=j_cap&&std::abs(j2)<=j_cap&&
         std::abs(a_mid)<=a_max&&std::abs(a_T)<=a_max)found=true;
    }
    if(found)break;
  }
  if(!found)return false;
  const auto profile=[&](double t,double& jerk_at) {
    if(t<=tau) {jerk_at=j1;return v0+a0*t+0.5*j1*t*t;}
    const double v_tau=v0+a0*tau+0.5*j1*tau*tau,a_tau=a0+j1*tau,s=t-tau;
    jerk_at=j2;return v_tau+a_tau*s+0.5*j2*s*s;
  };
  // The tracker certifies contact rows on Bernstein controls per servo sub-interval:
  // the middle control of a quadratic velocity piece sits |j| h^2 / 8 above the curve
  // and the chord between knots |j| h^2 / 8 below it, so a piecewise-linear envelope
  // needs j h^2 / 4 of headroom for the plan to follow the profile exactly.
  const auto slew_ceiling=[&](double t) {
    if(t>T+1e-12)return -std::numeric_limits<double>::infinity();
    double jerk_at=0.0;const double v=profile(std::min(t,T),jerk_at);
    return std::max(v,0.0)+std::abs(jerk_at)*h*h/4.0;
  };
  const auto slew_floor=[&](double t) {
    if(t>T+1e-12)return std::numeric_limits<double>::infinity();
    double jerk_at=0.0;const double v=profile(std::min(t,T),jerk_at);
    return v-std::abs(jerk_at)*h*h/4.0;
  };
  // The floor is the LOWER of the source floor and the profile's floor: the minimum of
  // two near-linear pieces is concave, so its chord between 2 ms knots lies below it
  // and no crossing knots are needed on this side.
  const auto floor_at=[&](double t) {return std::min(floor_of(t),slew_floor(t));};
  // Merged knot times: the original grid, T itself, and every envelope/slew crossing.
  std::array<double,PreviewContactConstraint::kCapacity> times{};
  std::size_t time_count=0;bool inserted_T=false;
  for(std::size_t k=0;k<contact.count;++k) {
    const double t=contact.knots[k].time_sec;
    if(!inserted_T && t>T+1e-12) {
      if(time_count>=times.size())return false;
      times[time_count++]=T;inserted_T=true;
    }
    if(std::abs(t-T)<=1e-12)inserted_T=true;
    if(time_count>=times.size())return false;
    times[time_count++]=t;
  }
  std::array<Knot,PreviewContactConstraint::kCapacity> out{};
  std::size_t count=0;
  const auto push=[&](double t,double upper,double lower) {
    if(count>=out.size())return false;
    out[count++]={t,upper,std::min(lower,upper)};return true;
  };
  double prev_t=0.0,prev_d=0.0;bool have_prev=false;
  for(std::size_t k=0;k<time_count;++k) {
    const double t=times[k],bb=bound(t),sb=slew_ceiling(t);
    const bool finite=std::isfinite(sb);
    const double d=finite?sb-bb:-1.0;
    if(have_prev && finite && (prev_d>0.0)!=(d>0.0) && prev_d!=d) {
      const double u=prev_d/(prev_d-d),tc=prev_t+u*(t-prev_t);
      if(tc>prev_t && tc<t && !push(tc,bound(tc),floor_at(tc)))return false;
    }
    if(!push(t,std::max(bb,finite?sb:bb),floor_at(t)))return false;
    prev_t=t;prev_d=d;have_prev=finite;
  }
  contact.knots=out;contact.count=count;
  return true;
}
namespace {
using Clock = std::chrono::steady_clock;
enum class SlotState : unsigned char { Free, Writing, Ready, Reading };
constexpr std::size_t kSlots = 3;
constexpr std::size_t kMaxFutureSamples = 256;
bool positive(double value) { return std::isfinite(value) && value > 0.0; }

bool sameSource(const PreviewExecutionIdentity& a, const CartesianChunkFollower& f) {
  return a.source_wire_seq == f.windowWireSeq() &&
         a.source_recv_seq == f.windowRecvSeq();
}

bool sampleHistory(const PreviewExecutionRequest& r, double time,
                   FollowerOutputKinematics& out) {
  if (r.history_count == 0 || time < r.history[0].time_sec ||
      time > r.history[r.history_count - 1].time_sec) return false;
  const auto end = r.history.begin() + r.history_count;
  auto upper = std::lower_bound(r.history.begin(), end, time,
      [](const PreviewExecutionHistoryEntry& e, double t) { return e.time_sec < t; });
  if (upper == end) return false;
  if (upper == r.history.begin() || upper->time_sec == time) {
    out = upper->state;
    return true;
  }
  const auto lower = upper - 1;
  out = interpolatePreviewKinematics(lower->state, upper->state,
      (time - lower->time_sec)/(upper->time_sec - lower->time_sec));
  return true;
}

bool sampleFuture(const FollowerPreviewReference& future, double time,
                  FollowerOutputKinematics& out) {
  if (future.samples.size() < 2 || time < future.generated_at_sec ||
      time > future.generated_at_sec + future.samples.back().relative_time_sec) return false;
  const double relative = std::clamp(time - future.generated_at_sec, 0.0,
                                    future.samples.back().relative_time_sec);
  auto upper = std::lower_bound(future.samples.begin(), future.samples.end(), relative,
      [](const FollowerPreviewReferenceSample& e, double t) { return e.relative_time_sec < t; });
  if (upper == future.samples.end()) return false;
  if (upper == future.samples.begin() || upper->relative_time_sec == relative) {
    out = upper->kinematics;
    return true;
  }
  const auto lower = upper - 1;
  out = interpolatePreviewKinematics(lower->kinematics, upper->kinematics,
      (relative - lower->relative_time_sec)/(upper->relative_time_sec - lower->relative_time_sec));
  return true;
}

bool validGauge(const PreviewExecutionGauge& gauge, double tolerance) {
  return gauge.translation.allFinite() && gauge.rotation.coeffs().allFinite() &&
      std::abs(gauge.rotation.norm()-1.0)<=tolerance;
}

bool stationary(const PreviewMotionState& s) {
  return finitePreviewPose(s.pose) && s.linear_velocity.isZero(0.0) &&
      s.linear_acceleration.isZero(0.0) && s.angular_velocity_body.isZero(0.0) &&
      s.angular_acceleration_body.isZero(0.0);
}
} // namespace

double PreviewExecutionWorker::monotonicNowSec() {
  return static_cast<double>(nowSteadyNs()) * 1e-9;
}

bool samplePreviewExecutionPhaseReference(const PreviewExecutionResult& r,
    double reference_time,double now,const PreviewExecutionIdentity& current,
    const PreviewExecutionGauge& gauge,double tolerance,FollowerOutputKinematics& out) {
  const auto& forecast=r.phase_reference;
  if(!positive(tolerance)||!validGauge(gauge,tolerance)||!validGauge(r.gauge,tolerance)||
     gauge.revision!=r.gauge.revision||(gauge.translation-r.gauge.translation).norm()>tolerance||
     math::log3(gauge.rotation.toRotationMatrix().transpose()*r.gauge.rotation.toRotationMatrix()).norm()>tolerance||
     forecast.count<2||forecast.count>forecast.kCapacity||
     forecast.samples[0].relative_time_sec!=0.||
     !positive(forecast.samples[forecast.count-1].relative_time_sec)||
     r.identity.epoch!=current.epoch||r.identity.gate_revision!=current.gate_revision||
     r.identity.source_wire_seq!=current.source_wire_seq||r.identity.source_recv_seq!=current.source_recv_seq||
     !std::isfinite(now)||!std::isfinite(reference_time)||!std::isfinite(r.generated_at_sec)||
     !std::isfinite(r.completed_at_sec)||!std::isfinite(r.valid_until_sec)||
     now<r.generated_at_sec||now<r.completed_at_sec||now>=r.valid_until_sec||
     reference_time<r.generated_at_sec||reference_time>r.valid_until_sec||
     reference_time>r.generated_at_sec+forecast.samples[forecast.count-1].relative_time_sec)return false;
  const double time=std::clamp(reference_time-r.generated_at_sec,0.,
      forecast.samples[forecast.count-1].relative_time_sec);
  const auto end=forecast.samples.begin()+forecast.count;
  auto upper=std::lower_bound(forecast.samples.begin(),end,time,
      [](const FollowerPreviewReferenceSample& s,double t){return s.relative_time_sec<t;});
  if(upper==end)return false;
  if(upper==forecast.samples.begin()||upper->relative_time_sec==time)out=upper->kinematics;
  else {
    const auto lower=upper-1;
    out=interpolatePreviewKinematics(lower->kinematics,upper->kinematics,
        (time-lower->relative_time_sec)/(upper->relative_time_sec-lower->relative_time_sec));
  }
  return finitePreviewState(out);
}

PreviewExecutionAcceptance validatePreviewExecutionResult(
    const PreviewExecutionResult& r, double now, const PreviewExecutionIdentity& current) {
  if (!r.accepted() || !r.trajectory.valid) return PreviewExecutionAcceptance::WorkerRejected;
  if (r.identity.epoch != current.epoch) return PreviewExecutionAcceptance::EpochMismatch;
  if (r.identity.gate_revision != current.gate_revision) return PreviewExecutionAcceptance::GateMismatch;
  if (r.identity.source_wire_seq != current.source_wire_seq ||
      r.identity.source_recv_seq != current.source_recv_seq) return PreviewExecutionAcceptance::SourceMismatch;
  if (r.identity.parent_plan_id != current.parent_plan_id) return PreviewExecutionAcceptance::ParentMismatch;
  if (!std::isfinite(now) || !std::isfinite(r.generated_at_sec) ||
      !std::isfinite(r.splice_at_sec) || !std::isfinite(r.valid_until_sec) ||
      !std::isfinite(r.completed_at_sec) || now < r.generated_at_sec ||
      r.completed_at_sec < r.generated_at_sec || r.completed_at_sec > now ||
      r.splice_at_sec <= r.generated_at_sec || r.valid_until_sec <= r.splice_at_sec ||
      r.trajectory.durationSec() <= 0.0)
    return PreviewExecutionAcceptance::InvalidTiming;
  if (now >= r.splice_at_sec || now >= r.valid_until_sec ||
      r.completed_at_sec >= r.splice_at_sec) return PreviewExecutionAcceptance::Late;
  return PreviewExecutionAcceptance::Ready;
}

bool transportPreviewExecutionResult(PreviewExecutionResult& result,
    const PreviewExecutionGauge& current, double tolerance) {
  if(!std::isfinite(tolerance) || tolerance<=0 || !result.accepted() || !result.trajectory.valid ||
     result.trajectory.count==0 || result.trajectory.count>PreviewPolynomialTrajectory::kMaxHorizonSteps ||
     !validGauge(result.gauge,tolerance) || !validGauge(current,tolerance) ||
     result.gauge.revision>current.revision) return false;
  if(!finitePreviewPose(result.initial.pose))return false;
  if(result.gauge.revision==current.revision &&
     (result.gauge.translation-current.translation).isZero(0.0) &&
     (result.gauge.rotation.coeffs()-current.rotation.coeffs()).isZero(0.0)) return true;
  const Eigen::Vector3d dp=current.translation-result.gauge.translation;
  const Eigen::Quaterniond dR=(current.rotation*result.gauge.rotation.conjugate()).normalized();
  if(!dp.allFinite() || !dR.coeffs().allFinite()) return false;
  if(result.gauge.revision==current.revision &&
     (dp.norm()>tolerance || Eigen::AngleAxisd(dR).angle()>tolerance)) return false;
  auto initial=math::se3FromPose(result.initial.pose);
  initial.translation()+=dp;initial.rotation()=dR*initial.rotation();
  if(!initial.translation().allFinite() || !initial.rotation().allFinite()) return false;
  for(std::size_t i=0;i<=result.trajectory.count;++i)
    if(!(result.trajectory.p.row(i).head<3>().transpose()+dp).allFinite()) return false;
  if(result.phase_reference.count>result.phase_reference.samples.size())return false;
  for(std::size_t i=0;i<result.phase_reference.count;++i) {
    const auto& state=result.phase_reference.samples[i].kinematics;
    if(!finitePreviewState(state)||
       !(Eigen::Vector3d(state.pose.x,state.pose.y,state.pose.z)+dp).allFinite())return false;
  }
  result.initial.pose=math::poseFromSe3(initial);
  for(std::size_t i=0;i<=result.trajectory.count;++i)
    result.trajectory.p.row(i).head<3>()+=dp.transpose();
  result.trajectory.rotation0=dR*result.trajectory.rotation0;
  if(result.nominal_trajectory.valid) {
    for(std::size_t i=0;i<=result.nominal_trajectory.count;++i)
      result.nominal_trajectory.p.row(i).head<3>()+=dp.transpose();
    result.nominal_trajectory.rotation0=dR*result.nominal_trajectory.rotation0;
  }
  for(std::size_t i=0;i<result.phase_reference.count;++i) {
    auto& state=result.phase_reference.samples[i].kinematics;
    auto pose=math::se3FromPose(state.pose);pose.translation()+=dp;pose.rotation()=dR*pose.rotation();
    state.pose=math::poseFromSe3(pose);
  }
  result.gauge=current;
  return true;
}

struct PreviewExecutionWorker::Impl {
  struct RequestSlot {
    explicit RequestSlot(const CartesianChunkFollowerConfig& c) : follower(c) {}
    std::atomic<SlotState> state{SlotState::Free};
    CartesianChunkFollower follower;
    PreviewExecutionRequest request{};
  };
  struct ResultSlot {
    std::atomic<SlotState> state{SlotState::Free};
    PreviewExecutionResult result{};
  };
  PreviewExecutionWorkerConfig cfg;
  PreviewTrajectoryTracker tracker;
  PreviewTrajectoryTracker nominal_tracker;
  std::array<std::unique_ptr<RequestSlot>, kSlots> requests;
  std::array<ResultSlot, kSlots> results;
  std::array<std::atomic<std::uint64_t>,8> worker_status_counts{}, solve_status_counts{};
  std::atomic<std::uint64_t> request_invalid{0}, request_mailbox_full{0}, request_coalesced{0};
  std::atomic<std::uint64_t> result_publish_dropped{0}, result_coalesced{0};
  std::atomic<bool> stopping{false};
  std::thread thread;

  Impl(const PreviewTrackerConfig& tracker_cfg, const CartesianChunkFollowerConfig& follower_cfg,
       const PreviewExecutionWorkerConfig& worker_cfg)
      : cfg(worker_cfg), tracker(tracker_cfg), nominal_tracker(tracker_cfg) {
    static_assert(std::atomic<std::uint64_t>::is_always_lock_free);
    static_assert(std::atomic<SlotState>::is_always_lock_free,
                  "Preview mailboxes require lock-free slot ownership");
    if (!positive(cfg.servo_period_sec) || !positive(cfg.poll_period_sec) ||
        !positive(cfg.max_request_age_sec) || cfg.poll_period_sec >= cfg.max_request_age_sec ||
        cfg.max_snapshot_horizon == 0 || cfg.max_snapshot_horizon > 256 ||
        (tracker.durationSec() + cfg.max_request_age_sec)/cfg.servo_period_sec + 2 > kMaxFutureSamples ||
        2*(std::ceil(tracker.durationSec()/cfg.servo_period_sec)+2)-1>PreviewContactConstraint::kCapacity ||
        !(tracker.config().contact_slew_jerk_m_s3 > 0.0) ||
        tracker.config().contact_slew_jerk_m_s3 > tracker.config().max_linear_jerk_m_s3 ||
        !std::isfinite(tracker.config().trusted_future_sec) || tracker.config().trusted_future_sec < 0.0 ||
        tracker.config().trusted_future_sec > tracker.durationSec() ||
        !std::isfinite(tracker.config().contact_realign_sec) || !(tracker.config().contact_realign_sec > 0.0) ||
        tracker.config().contact_realign_sec > tracker.durationSec() ||
        !std::isfinite(tracker.config().contact_retreat_slack_m_s) || tracker.config().contact_retreat_slack_m_s < 0.0 ||
        tracker.config().contact_retreat_slack_m_s > tracker.config().max_linear_velocity_m_s)
      throw std::invalid_argument("Invalid explicit preview worker configuration");
    for (auto& slot : requests) {
      slot = std::make_unique<RequestSlot>(follower_cfg);
      slot->follower.reserveSnapshotCapacity(cfg.max_snapshot_horizon);
    }
    thread = std::thread([this] { run(); });
  }

  ~Impl() {
    stopping.store(true, std::memory_order_release);
    if (thread.joinable()) thread.join();
  }

  PreviewExecutionResult process(const CartesianChunkFollower& follower,
                                 const PreviewExecutionRequest& r) {
    PreviewExecutionResult out;
    out.identity = r.identity;
    out.gauge = r.gauge;
    out.generated_at_sec = r.generated_at_sec;
    out.splice_at_sec = r.splice_at_sec;
    out.valid_until_sec = r.valid_until_sec;
    const auto finish = [&](PreviewExecutionWorkerStatus status) {
      out.status = status;
      out.completed_at_sec = PreviewExecutionWorker::monotonicNowSec();
      return out;
    };
    if (!validGauge(r.gauge,tracker.config().feasibility_tolerance) ||
        !std::isfinite(r.generated_at_sec) || !std::isfinite(r.splice_at_sec) ||
        !std::isfinite(r.valid_until_sec) || !std::isfinite(r.cursor_time_sec) ||
        !std::isfinite(r.cursor_rate) || r.cursor_rate < 0.0 ||
        !std::isfinite(r.reference_rate) || r.reference_rate < 0.0 || r.reference_rate > 1.0 ||
        !std::isfinite(r.contact_gate) || r.contact_gate<0.0 || r.contact_gate>1.0 ||
        !r.contact_normal_stand.allFinite() ||
        r.splice_at_sec <= r.generated_at_sec || r.valid_until_sec <= r.splice_at_sec ||
        r.valid_until_sec > r.generated_at_sec + cfg.max_request_age_sec ||
        r.cursor_time_sec > r.generated_at_sec || r.history_count == 0 ||
        r.history_count > r.history.size() ||
        r.history[r.history_count - 1].time_sec != r.generated_at_sec)
      return finish(PreviewExecutionWorkerStatus::InvalidRequest);
    if(r.angular_predecessor.kind()!=PreviewAngularContinuation::Kind::None &&
       (r.cold_start || !r.has_brake_predecessor))
      return finish(PreviewExecutionWorkerStatus::InvalidRequest);
    const double now = PreviewExecutionWorker::monotonicNowSec();
    if (now < r.generated_at_sec) return finish(PreviewExecutionWorkerStatus::InvalidRequest);
    if (now >= r.splice_at_sec || now >= r.valid_until_sec)
      return finish(PreviewExecutionWorkerStatus::Late);
    if (!sameSource(r.identity, follower)) return finish(PreviewExecutionWorkerStatus::SourceMismatch);
    for (std::size_t k = 0; k < r.history_count; ++k) {
      if (!std::isfinite(r.history[k].time_sec) || !finitePreviewState(r.history[k].state) ||
          (k && r.history[k].time_sec <= r.history[k - 1].time_sec))
        return finish(PreviewExecutionWorkerStatus::InvalidRequest);
    }
    if (r.cursor_time_sec < r.history[0].time_sec)
      return finish(PreviewExecutionWorkerStatus::InvalidRequest);
    const bool contact_active=r.contact_gate<1.0 && !r.contact_normal_stand.isZero(0.0);
    if(contact_active && std::abs(r.contact_normal_stand.norm()-1.0)>tracker.config().feasibility_tolerance)
      return finish(PreviewExecutionWorkerStatus::InvalidRequest);
    if (r.cold_start) {
      if (r.predecessor.valid || r.has_brake_predecessor ||
          r.identity.parent_plan_id != 0 || !stationary(r.cold_initial))
        return finish(PreviewExecutionWorkerStatus::SpliceUnavailable);
      out.initial = r.cold_initial;
    } else {
      PreviewMotionSample initial;
      const double predecessor_duration=r.has_brake_predecessor?r.brake_predecessor.durationSec():r.predecessor.durationSec();
      const bool predecessor_valid=r.has_brake_predecessor?r.brake_predecessor.valid:r.predecessor.valid;
      // The predecessor and successor share physical seconds. No alternate
      // clock can supply derivatives that differ from the dispatched motion.
      if (!std::isfinite(r.predecessor_origin_sec) || !predecessor_valid ||
          r.identity.parent_plan_id == 0 || r.splice_at_sec < r.predecessor_origin_sec ||
          (!r.has_brake_predecessor && r.splice_at_sec > r.predecessor_origin_sec + predecessor_duration))
        return finish(PreviewExecutionWorkerStatus::SpliceUnavailable);
      const double t=r.has_brake_predecessor?r.splice_at_sec-r.predecessor_origin_sec:
          std::clamp(r.splice_at_sec-r.predecessor_origin_sec,0.0,predecessor_duration);
      if (!(r.has_brake_predecessor?r.brake_predecessor.sample(t, initial):r.predecessor.sample(t, initial)))
        return finish(PreviewExecutionWorkerStatus::SpliceUnavailable);
      if(r.has_brake_predecessor && !r.angular_predecessor.sample(r.splice_at_sec,initial))
        return finish(PreviewExecutionWorkerStatus::SpliceUnavailable);
      out.initial = initial;
      if(!r.has_brake_predecessor) out.spliced_predecessor_time_sec = t;
    }

    FollowerPreviewReferenceRequest preview_request;
    const double lead = r.splice_at_sec - r.generated_at_sec;
    preview_request.sample_count = static_cast<std::size_t>(
        std::ceil((lead + tracker.durationSec())/cfg.servo_period_sec)) + 1;
    if (preview_request.sample_count > kMaxFutureSamples)
      return finish(PreviewExecutionWorkerStatus::InvalidRequest);
    preview_request.sample_period_sec = cfg.servo_period_sec;
    preview_request.servo_period_sec = cfg.servo_period_sec;
    preview_request.generated_at_sec = r.generated_at_sec;
    preview_request.valid_until_sec = r.valid_until_sec;
    preview_request.epoch = r.identity.epoch;
    preview_request.revision = r.identity.gate_revision;
    const auto future = makeFollowerPreviewReference(follower, preview_request);
    if (future.status != FollowerPreviewReferenceStatus::Ready)
      return finish(PreviewExecutionWorkerStatus::PreviewUnavailable);

    if(future.samples.size()>out.phase_reference.samples.size())
      return finish(PreviewExecutionWorkerStatus::InvalidRequest);
    out.phase_reference.count=future.samples.size();
    std::copy(future.samples.begin(),future.samples.end(),out.phase_reference.samples.begin());

    // Only the already selected segment may create future position demand.
    // execute_steps is a publisher replacement cadence, not a commitment to
    // execute later rows. Continue the last sample of this prefix at its own
    // velocity; the next live tick/replan will expose the next selected row.
    // TRUSTED FUTURE (2026-09-15 night): with tracker.trusted_future_sec > 0 the
    // forecast is trusted for that long (the committed execute window) instead of
    // one segment, so row jitter is averaged, not extrapolated. A stall still ends it.
    std::size_t prefix_end=0;
    const double trusted=tracker.config().trusted_future_sec;
    for(std::size_t k=1;k<future.samples.size();++k) {
      if(future.samples[k].stalled)break;
      if(trusted>0.0) {if(future.samples[k].relative_time_sec>trusted+1e-12)break;}
      else if(future.samples[k].step_index!=future.samples[0].step_index)break;
      prefix_end=k;
    }
    const auto& prefix=future.samples[prefix_end];
    out.trusted_prefix_sec=prefix.relative_time_sec;
    const auto reference_state=[&](double time,FollowerOutputKinematics& state) {
      if(time<=r.generated_at_sec)return sampleHistory(r,time,state);
      const double relative=time-r.generated_at_sec;
      if(relative<=prefix.relative_time_sec)return sampleFuture(future,time,state);
      state=prefix.kinematics;
      const double dt=relative-prefix.relative_time_sec;
      auto pose=math::se3FromPose(state.pose);
      pose.translation()+=dt*Eigen::Vector3d(state.velocity.x,state.velocity.y,state.velocity.z);
      pose.rotation()=pose.rotation()*math::exp3(dt*Eigen::Vector3d(
          state.velocity.rx,state.velocity.ry,state.velocity.rz));
      state.pose=math::poseFromSe3(pose);state.acceleration={};
      return finitePreviewState(state);
    };

    PreviewReference reference;
    reference.count = tracker.config().horizon_steps + 1;
    for (std::size_t k = 0; k < reference.count; ++k) {
      const double relative = k * tracker.config().planning_dt_sec;
      const double from_generation = lead + r.reference_rate * relative;
      const double time = std::min(r.generated_at_sec + from_generation,
          r.cursor_time_sec + r.cursor_rate * from_generation);
      FollowerOutputKinematics state;
      const bool valid = reference_state(time,state);
      if (!valid) return finish(PreviewExecutionWorkerStatus::PreviewUnavailable);
      reference.knots[k].time_sec = relative;
      reference.knots[k].pose = state.pose;
    }
    // Obtain a physically feasible free candidate from the SAME reference and
    // seed. An arbitrarily small force restriction must approach this candidate,
    // not suddenly substitute the (often much slower) raw follower velocity.
    const auto remaining_budget=[&] {
      return std::min(r.splice_at_sec,r.valid_until_sec)-
          PreviewExecutionWorker::monotonicNowSec()-cfg.servo_period_sec;
    };
    PreviewContactConstraint contact;
    if(contact_active) {
      if(remaining_budget()<=0)return finish(PreviewExecutionWorkerStatus::Late);
      out.solve_attempted=true;
      // THE FREE CANDIDATE IS SOLVED FROM A DE-BRAKED SPLICE (2026-09-16). It is the
      // measure of DEMAND - what the plan would do without the contact - but it used to
      // inherit the constrained plan's braking acceleration along the normal, and from
      // (+8 mm/s, -3.5 m/s^2) a free plan retreats for 100 ms before it turns (offline:
      // -48 mm/s) even with its source 8 mm deeper: the bound collapsed to zero and the
      // executor left the contact. The braking component is contact-induced, not demand,
      // so it is dropped from the candidate's initial state; the constrained plan still
      // starts from the true splice and the slew bridges the difference.
      PreviewMotionState nominal_initial=out.initial;
      {
        const Eigen::Vector3d& normal=r.contact_normal_stand;
        const double braking=std::min(0.0,normal.dot(nominal_initial.linear_acceleration));
        nominal_initial.linear_acceleration-=braking*normal;
      }
      const auto nominal=nominal_tracker.plan(reference,nominal_initial,{},
          PreviewContactSolveMode::Automatic,remaining_budget());
      out.nominal_solve_time_sec=nominal.diagnostics.solve_time_sec;
      if(!nominal.accepted() || !nominal_tracker.exportTrajectory(out.nominal_trajectory)) {
        out.diagnostics=nominal.diagnostics;
        return finish(PreviewExecutionWorkerStatus::SolveRejected);
      }
      contact.enabled=true;contact.normal_stand=r.contact_normal_stand;
      if(!buildContactEnvelope(contact,out.nominal_trajectory,r.contact_gate,cfg.servo_period_sec,
                               tracker.config().contact_retreat_slack_m_s))
        return finish(PreviewExecutionWorkerStatus::InvalidRequest);
    }
    // SPLICE FROM THE PREDECESSOR'S OWN SAMPLE. Since the contact clamp's deletion
    // (2026-09-11) that sample IS what the arm was sent, so nothing is shifted or cut
    // here; the plan is continuous with the dispatched path by construction.
    if(contact_active && !r.cold_start) {
      const Eigen::Vector3d& normal=r.contact_normal_stand;
      // A splice that closes faster than the authority is never refused as Infeasible
      // (2026-09-10 pm: 27 such refusals in 5 s fed the expiry-brake cycle under a hand
      // push). Since 2026-09-15 night the authority is not widened along the fastest
      // brake either: that demanded a cut the 10 ms planning jerk could only realise by
      // overshooting into a retreat (see slewContactAuthority). The envelope is slewed
      // from the dispatched closing state; a coast already under the envelope is a no-op.
      if(!slewContactAuthority(contact,normal.dot(out.initial.linear_velocity),
                               normal.dot(out.initial.linear_acceleration),
                               tracker.config(),cfg.servo_period_sec,&out.nominal_trajectory,r.contact_gate))
        return finish(PreviewExecutionWorkerStatus::InvalidRequest);
    }
    else if(contact_active) {
      // A cold plan has no dispatched state to slew from: keep the one-sided authority
      // (the floor would otherwise demand the scaled candidate from an arbitrary seed).
      for(std::size_t k=0;k<contact.count;++k)
        contact.knots[k].lower_velocity_m_s=-std::numeric_limits<double>::infinity();
    }
    // Solver state is strictly worker-owned. Never publish its previous result
    // when this request fails; the servo owns the predecessor's finite lifetime.
    // Leave one configured servo period for mailbox delivery and admission.
    // Spending the looser offline QP budget after this request's splice would
    // only starve fresher requests; the deadline and predecessor stay immutable.
    const double solve_budget=remaining_budget();
    if(solve_budget<=0.0)return finish(PreviewExecutionWorkerStatus::Late);
    out.solve_attempted = true;
    const auto solved = tracker.plan(reference,out.initial,contact,
        PreviewContactSolveMode::Automatic,solve_budget);
    out.diagnostics = solved.diagnostics;
    out.diagnostics.solve_time_sec+=out.nominal_solve_time_sec;
    out.contact_authority=contact;
    if (!solved.accepted()) return finish(PreviewExecutionWorkerStatus::SolveRejected);
    if (!tracker.exportTrajectory(out.trajectory))
      return finish(PreviewExecutionWorkerStatus::SolveRejected);
    if(!contact_active)out.nominal_trajectory=out.trajectory;
    const double completed = PreviewExecutionWorker::monotonicNowSec();
    if (completed >= r.splice_at_sec || completed >= r.valid_until_sec)
      return finish(PreviewExecutionWorkerStatus::Late);
    return finish(PreviewExecutionWorkerStatus::Solved);
  }

  void publish(const PreviewExecutionResult& result) {
    for (auto& slot : results) {
      SlotState expected = SlotState::Free;
      if (!slot.state.compare_exchange_strong(expected, SlotState::Writing,
                                             std::memory_order_acquire)) continue;
      slot.result = result;
      slot.state.store(SlotState::Ready, std::memory_order_release);
      return;
    }
    result_publish_dropped.fetch_add(1,std::memory_order_relaxed);
    // No overwrite of a slot the servo may be reading. An undelivered result
    // does not extend any accepted trajectory's clock or source validity.
  }

  void run() {
    while (!stopping.load(std::memory_order_acquire)) {
      RequestSlot* selected = nullptr;
      // Only this thread consumes Ready requests; their contents cannot change
      // until it returns their slots to Free after reading them.
      for (auto& slot : requests) {
        if (slot->state.load(std::memory_order_acquire) != SlotState::Ready) continue;
        if (!selected || slot->request.generated_at_sec > selected->request.generated_at_sec ||
            (slot->request.generated_at_sec == selected->request.generated_at_sec &&
             slot->request.identity.request_id > selected->request.identity.request_id))
          selected = slot.get();
      }
      if (!selected) {
        std::this_thread::sleep_for(std::chrono::duration<double>(cfg.poll_period_sec));
        continue;
      }
      selected->state.store(SlotState::Reading, std::memory_order_release);
      // Older pending work has already lost the freshness contest. Reclaim it
      // without computing it, keeping the input backlog strictly bounded.
      for (auto& slot : requests) {
        if (slot.get() == selected || slot->state.load(std::memory_order_acquire) != SlotState::Ready) continue;
        if (slot->request.generated_at_sec <= selected->request.generated_at_sec) {
          slot->state.store(SlotState::Free, std::memory_order_release);
          request_coalesced.fetch_add(1,std::memory_order_relaxed);
        }
      }
      PreviewExecutionResult result;
      try {
        result = process(selected->follower, selected->request);
      } catch (...) {
        result.identity = selected->request.identity;
        result.gauge = selected->request.gauge;
        result.generated_at_sec = selected->request.generated_at_sec;
        result.splice_at_sec = selected->request.splice_at_sec;
        result.valid_until_sec = selected->request.valid_until_sec;
        result.completed_at_sec = PreviewExecutionWorker::monotonicNowSec();
        result.status = PreviewExecutionWorkerStatus::WorkerException;
      }
      selected->state.store(SlotState::Free, std::memory_order_release);
      worker_status_counts[static_cast<std::size_t>(result.status)].fetch_add(1,std::memory_order_relaxed);
      if(result.solve_attempted)
        solve_status_counts[static_cast<std::size_t>(result.diagnostics.status)].fetch_add(1,std::memory_order_relaxed);
      publish(result);
    }
  }
};

PreviewExecutionWorker::PreviewExecutionWorker(const PreviewTrackerConfig& tracker,
    const CartesianChunkFollowerConfig& follower, const PreviewExecutionWorkerConfig& worker)
    : impl_(std::make_unique<Impl>(tracker, follower, worker)) {}
PreviewExecutionWorker::~PreviewExecutionWorker() = default;

bool PreviewExecutionWorker::trySubmit(const CartesianChunkFollower& follower,
                                     const PreviewExecutionRequest& request) noexcept {
  if (request.history_count > request.history.size()) {
    impl_->request_invalid.fetch_add(1,std::memory_order_relaxed);return false;
  }
  for (auto& slot : impl_->requests) {
    SlotState expected = SlotState::Free;
    if (!slot->state.compare_exchange_strong(expected, SlotState::Writing,
                                           std::memory_order_acquire)) continue;
    if (!slot->follower.canCopySnapshotFrom(follower)) {
      impl_->request_invalid.fetch_add(1,std::memory_order_relaxed);
      slot->state.store(SlotState::Free, std::memory_order_release);
      return false;
    }
    slot->follower = follower;
    slot->request = request;
    slot->state.store(SlotState::Ready, std::memory_order_release);
    return true;
  }
  impl_->request_mailbox_full.fetch_add(1,std::memory_order_relaxed);
  return false;
}

bool PreviewExecutionWorker::tryTake(PreviewExecutionResult& result) noexcept {
  bool found = false;
  for (auto& slot : impl_->results) {
    SlotState expected = SlotState::Ready;
    if (!slot.state.compare_exchange_strong(expected, SlotState::Reading,
                                           std::memory_order_acquire)) continue;
    if(found)impl_->result_coalesced.fetch_add(1,std::memory_order_relaxed);
    if (!found || slot.result.generated_at_sec > result.generated_at_sec ||
        (slot.result.generated_at_sec == result.generated_at_sec &&
         slot.result.identity.request_id > result.identity.request_id)) {
      result = slot.result;
      found = true;
    }
    slot.state.store(SlotState::Free, std::memory_order_release);
  }
  return found;
}
PreviewExecutionWorkerDiagnostics PreviewExecutionWorker::diagnostics() const noexcept {
  PreviewExecutionWorkerDiagnostics out;
  for(std::size_t i=0;i<8;++i) {
    out.worker_status_counts[i]=impl_->worker_status_counts[i].load(std::memory_order_relaxed);
    out.solve_status_counts[i]=impl_->solve_status_counts[i].load(std::memory_order_relaxed);
  }
  out.request_invalid=impl_->request_invalid.load(std::memory_order_relaxed);
  out.request_mailbox_full=impl_->request_mailbox_full.load(std::memory_order_relaxed);
  out.request_coalesced=impl_->request_coalesced.load(std::memory_order_relaxed);
  out.result_publish_dropped=impl_->result_publish_dropped.load(std::memory_order_relaxed);
  out.result_coalesced=impl_->result_coalesced.load(std::memory_order_relaxed);
  return out;
}
} // namespace rb_servo::control
