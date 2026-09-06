#include "rb_servo/control/live_preview_execution.hpp"
#include "rb_servo/core/clock.hpp"
#include "rb_servo/math/se3.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace rb_servo::control {
namespace {
static_assert(static_cast<std::size_t>(PreviewExecutionWorkerStatus::WorkerException)+1==kPreviewWorkerStatusNames.size());
static_assert(static_cast<std::size_t>(PreviewSolveStatus::TrackingBudgetExceeded)+1==kPreviewSolveStatusNames.size());
static_assert(static_cast<std::size_t>(PreviewExecutionAcceptance::InvalidTiming)+1==kPreviewResultCheckNames.size());
enum CancelReason : std::size_t { Fold, Reset, Source, Parent, Contact, Expiry, Sample, Other };
Eigen::Vector3d xyz(const Pose6D& p) { return {p.x,p.y,p.z}; }
double angle(const Pose6D& a,const Pose6D& b) {
  return math::log3(math::rotationFromPose(a).transpose()*math::rotationFromPose(b)).norm();
}
void shiftPose(Pose6D& p,const Eigen::Vector3d& dp,const Eigen::Quaterniond& dR) {
  auto T=math::se3FromPose(p);T.translation()+=dp;T.rotation()=dR* T.rotation();
  p=math::poseFromSe3(T);
}
PlanLeashParams leash(const RuckigFollowerConfig& c) {
  return {c.plan_leash_start_m,c.plan_leash_start_rad,c.plan_leash_full_m,
          c.plan_leash_full_rad,c.plan_leash_min_gate};
}
PreviewExecutionCursorConfig cursorConfig(const PreviewExecutionConfig& c) {
  return {c.cursor.max_backlog_sec,c.cursor.catchup_time_sec,c.cursor.max_rate,
          c.cursor.translation_velocity_floor,c.cursor.angular_velocity_floor};
}
}

LivePreviewExecution::LivePreviewExecution(const RuckigFollowerConfig& c,
    const CartesianChunkFollowerConfig& raw,double dt)
  : config_(c),worker_(c.preview_execution.tracker,raw,
      {dt,c.preview_execution.worker_poll_period_sec,c.preview_execution.max_result_age_sec,
       static_cast<std::size_t>(c.preview_execution.max_source_rows)}),
    cursor_(cursorConfig(c.preview_execution),leash(c)),
    brake_calculator_(c.preview_execution.tracker,dt) {
  const auto& pc=c.preview_execution;
  if(!pc.enable || !pc.cursor.enable || !(dt>0) ||
     pc.cursor.max_backlog_sec+dt>dt*(history_.size()-1) ||
     !(pc.splice_lead_sec>dt) || !(pc.max_result_age_sec>pc.splice_lead_sec+pc.replan_period_sec))
    throw std::invalid_argument("invalid live preview timing/history configuration");
  telemetry_.enabled=true;telemetry_.epoch=epoch_;telemetry_.status="inactive";
}

void LivePreviewExecution::reset(const char* reason) {
  if(initialized_ || staged_valid_ || hasPlan() || faulted_) {++epoch_;++gate_revision_;}
  cancelStaged(Reset,last_time_);
  initialized_=false;faulted_=false;accepted_epoch_=false;
  active_.status=PreviewExecutionWorkerStatus::InvalidRequest;
  active_.trajectory.valid=false;
  brake_calculator_.reset();brake_trajectory_.valid=false;brake_plan_id_=0;angular_continuation_.clear();
  stop_fault_reason_=nullptr;accepted_sample_time_sec_=0;accepted_plan_id_=0;
  fold_translation_.setZero();fold_rotation_.setIdentity();gauge_revision_=0;
  cursor_.clear();history_count_=history_begin_=0;
  telemetry_.active=false;telemetry_.status=reason;telemetry_.epoch=epoch_;telemetry_.plan_id=0;
  telemetry_.backlog_sec=0;telemetry_.rate=1;telemetry_.plan_age_sec=0;
}
void LivePreviewExecution::fail(const char* reason) {
  reset(reason);faulted_=true;
}
bool LivePreviewExecution::contactGuardStopped() {
  ++telemetry_.contact_guard_count;
  return beginBrake("braking_contact",true);
}
bool LivePreviewExecution::calculateBrake(const PreviewMotionState& initial,PreviewBrakeTrajectory& output) {
  const auto status=brake_calculator_.start(initial);
  if(status!=PreviewBrakeStatus::Ready) {
    const char* failure="brake_inactive";
    switch(status) {
      case PreviewBrakeStatus::InvalidInitialState: failure="brake_invalid_initial";break;
      case PreviewBrakeStatus::InitialOutsideLimits: failure="brake_initial_outside_limits";break;
      case PreviewBrakeStatus::Infeasible: failure="brake_infeasible";break;
      case PreviewBrakeStatus::LimitViolation: failure="brake_limit_violation";break;
      case PreviewBrakeStatus::NumericalFailure: failure="brake_numerical_failure";break;
      default: break;
    }
    fail(failure);return false;
  }
  if(!brake_calculator_.exportTrajectory(output)) {
    fail("brake_export_failed");return false;
  }
  return true;
}
bool LivePreviewExecution::sampleBrake() {
  if(!brake_trajectory_.sample(last_time_-brake_origin_sec_,sample_) ||
     !angular_continuation_.sample(last_time_,sample_)) {
    fail("brake_sample_failed");return false;
  }
  return true;
}
bool LivePreviewExecution::beginAngularBrake() {
  if(!angular_continuation_.retainsPolynomial())return true;
  if(!accepted_epoch_ || accepted_sample_time_sec_>last_time_ ||
     last_time_-accepted_sample_time_sec_>=config_.preview_execution.max_result_age_sec) {
    fail("angular_brake_stale_seed");return false;
  }
  // This helper supplies only angular motion. Translation keeps its original
  // finite brake, including the accepted seed and elapsed stopping clock.
  PreviewMotionState angular_initial=accepted_sample_;
  angular_initial.linear_velocity.setZero();angular_initial.linear_acceleration.setZero();
  PreviewBrakeTrajectory angular_brake;
  if(!calculateBrake(angular_initial,angular_brake))return false;
  if(!angular_continuation_.startBrake(angular_brake,accepted_sample_time_sec_)) {
    fail("angular_brake_export_failed");return false;
  }
  ++admission_diagnostics_.angular_brakes_started;
  // Only the composite predecessor identity changes. No unaccepted future
  // angular seed is used, and the translation origin is never renewed.
  brake_plan_id_=++request_id_;cancelStaged(Parent,last_time_);next_request_at_=last_time_;
  telemetry_.plan_id=brake_plan_id_;
  return sampleBrake();
}
bool LivePreviewExecution::beginBrake(const char* reason,bool contact_only) {
  if(brake_trajectory_.valid) {
    if(!contact_only && !beginAngularBrake())return false;
    brake_reason_=reason;telemetry_.status=reason;
    return sampleBrake();
  }
  if(!initialized_||faulted_){fail("brake_no_epoch");return false;}
  PreviewMotionState initial;
  if(accepted_epoch_) {
    initial=accepted_sample_;brake_origin_sec_=accepted_sample_time_sec_;
  } else {
    initial=cold_;brake_origin_sec_=last_time_;
  }
  if(brake_origin_sec_>last_time_ ||
     last_time_-brake_origin_sec_>=config_.preview_execution.max_result_age_sec) {
    fail("brake_stale_seed");return false;
  }
  angular_continuation_.clear();
  PreviewMotionSample source;
  const double precision=config_.preview_execution.tracker.feasibility_tolerance;
  if(contact_only && accepted_epoch_ && active_.accepted() &&
     active_.identity.request_id==accepted_plan_id_ &&
     accepted_sample_time_sec_>=active_.splice_at_sec && last_time_<active_.valid_until_sec &&
     active_.trajectory.sample(accepted_sample_time_sec_-active_.splice_at_sec,source) &&
     angle(source.pose,accepted_sample_.pose)<=precision &&
     (source.angular_velocity_body-accepted_sample_.angular_velocity_body).norm()<=precision &&
     (source.angular_acceleration_body-accepted_sample_.angular_acceleration_body).norm()<=precision &&
     angular_continuation_.retainPolynomial(active_.trajectory,active_.splice_at_sec,active_.valid_until_sec)) {
    // Contact authority here is translational. Preserve the same accepted
    // angular certificate only until its ORIGINAL expiry. These zeros affect
    // unused angular components of the translation helper, never sent motion.
    initial.angular_velocity_body.setZero();initial.angular_acceleration_body.setZero();
    ++admission_diagnostics_.angular_continuations_started;
  }
  if(!calculateBrake(initial,brake_trajectory_) || !sampleBrake())return false;
  brake_plan_id_=++request_id_;brake_reason_=reason;
  active_.status=PreviewExecutionWorkerStatus::InvalidRequest;
  active_.trajectory.valid=false;cancelStaged(Parent,last_time_);next_request_at_=last_time_;
  const std::size_t brake_cause=std::strcmp(reason,"braking_expired")==0?0:
      std::strcmp(reason,"braking_contact")==0?1:std::strcmp(reason,"braking_backlog")==0?2:
      std::strcmp(reason,"braking_history")==0?3:4;
  ++telemetry_.brake_counts[brake_cause];telemetry_.last_brake_reason=reason;
  telemetry_.last_brake_start_time_sec=last_time_;telemetry_.last_brake_origin_sec=brake_origin_sec_;
  telemetry_.plan_id=brake_plan_id_;telemetry_.active=false;telemetry_.status=reason;
  return true;
}
bool LivePreviewExecution::contactAllows(const PreviewMotionSample& proposed,const FollowerOutputKinematics& raw,
    double gate,const Eigen::Vector3d& normal) const {
  if(gate>=1 || normal.isZero(0))return true;
  // The canonical follower has already applied the force gate. Match the
  // worker's closing-velocity authority without applying that gate twice.
  // A retreating reference permits a stationary output; existing inertia is
  // handled by the separate finite brake, never by a renewed QP allowance.
  const Eigen::Vector3d raw_velocity(raw.velocity.x,raw.velocity.y,raw.velocity.z);
  const double allowed_velocity=std::max(0.0,normal.dot(raw_velocity));
  return normal.dot(proposed.linear_velocity)<=allowed_velocity+
      config_.preview_execution.tracker.feasibility_tolerance;
}
void LivePreviewExecution::append(double now,const FollowerOutputKinematics& state) {
  if(history_count_==history_.size()) {history_begin_=(history_begin_+1)%history_.size();--history_count_;}
  history_[(history_begin_+history_count_)%history_.size()]={now,state};++history_count_;
}
bool LivePreviewExecution::historySample(double time,FollowerOutputKinematics& out) const {
  if(!history_count_ || time<history_[history_begin_].time_sec)return false;
  for(std::size_t i=0;i<history_count_;++i) {
    const auto& hi=history_[(history_begin_+i)%history_.size()];
    if(time>hi.time_sec)continue;
    if(i==0||time==hi.time_sec){out=hi.state;return true;}
    const auto& lo=history_[(history_begin_+i-1)%history_.size()];
    out=interpolatePreviewKinematics(lo.state,hi.state,(time-lo.time_sec)/(hi.time_sec-lo.time_sec));
    return true;
  }
  return false;
}
PreviewExecutionIdentity LivePreviewExecution::identity(const CartesianChunkFollower& raw) const {
  return {epoch_,gate_revision_,raw.windowWireSeq(),raw.windowRecvSeq(),
          brake_trajectory_.valid?brake_plan_id_:(active_.accepted()?active_.identity.request_id:0),request_id_};
}
bool LivePreviewExecution::stagedCurrent(const CartesianChunkFollower& raw) const {
  const auto current=identity(raw);const auto& id=staged_.identity;
  return id.epoch==current.epoch && id.gate_revision==current.gate_revision &&
      id.source_wire_seq==current.source_wire_seq && id.source_recv_seq==current.source_recv_seq &&
      id.parent_plan_id==current.parent_plan_id;
}

LivePreviewOutput LivePreviewExecution::step(double now,const CartesianChunkFollower& raw,
    const Pose6D& accepted_nominal,bool stationary,double contact_gate,
    const Eigen::Vector3d& contact_normal) {
  telemetry_.active=false;
  LivePreviewOutput out;out.pose=accepted_nominal;
  if(faulted_) {out.fault=true;out.reason=telemetry_.status;return out;}
  if(!std::isfinite(now)||now<=0||now>=static_cast<double>(UINT64_MAX)/1e9||!finitePreviewPose(accepted_nominal)||
     !std::isfinite(contact_gate)||contact_gate<0||contact_gate>1||!contact_normal.allFinite()||
     (!contact_normal.isZero(0)&&std::abs(contact_normal.norm()-1)>config_.preview_execution.tracker.feasibility_tolerance)) {
    fail("invalid_input");out.fault=true;out.reason=telemetry_.status;return out;
  }
  telemetry_.sample_time_ns=static_cast<std::uint64_t>(now*1e9);
  if(!raw.active()||raw.holdPaused()) {reset("inactive");return out;}
  if(!initialized_) {
    if(!stationary) {telemetry_.status="braking";return out;}
    initialized_=true;initialized_at_=last_time_=now;next_request_at_=now;
    cold_={};cold_.pose=accepted_nominal;sample_={};sample_.pose=accepted_nominal;
    cursor_.reset(now);
  } else if(now<=last_time_ || now-last_time_>=config_.preview_execution.max_result_age_sec) {
    fail("tick_gap");out.fault=true;out.reason=telemetry_.status;return out;
  }
  last_time_=now;
  // Continuously changing force magnitude/direction and plan-rate forecasts
  // do not change the coordinate/lifecycle identity. They are re-read in every
  // request AND checked against the current sample below. Authority folds and
  // resets invalidate; tagged geometry common-frame shifts transport the gauge.
  const auto raw_sample=raw.outputKinematics();
  if(!finitePreviewState(raw_sample)) {fail("invalid_reference");out.fault=true;out.reason=telemetry_.status;return out;}
  append(now,raw_sample);
  telemetry_.source_wire_seq=raw.windowWireSeq();telemetry_.source_recv_seq=raw.windowRecvSeq();
  for(int n=0;n<3 && worker_.tryTake(received_);++n) {
    const double observed=PreviewExecutionWorker::monotonicNowSec();
    const auto check=validatePreviewExecutionResult(received_,observed,identity(raw));
    recordResult(check,observed);
    if(check==PreviewExecutionAcceptance::Ready && !staged_valid_ && !stop_fault_reason_) {
      staged_=received_;
      if(transportPreviewExecutionResult(staged_,gauge(),config_.preview_execution.tracker.feasibility_tolerance)) {
        staged_valid_=true;
        if(received_.gauge.revision!=gauge_revision_)++telemetry_.result_gauge_transported;
      } else {
        ++telemetry_.rejected;++telemetry_.gauge_transport_failed;
        telemetry_.last_admission_reason="invalid_gauge";
      }
    } else {
      ++telemetry_.rejected;
      if(check==PreviewExecutionAcceptance::Ready)++admission_diagnostics_.ready_not_staged;
    }
  }
  if(staged_valid_ && !stagedCurrent(raw)) {
    const auto current=identity(raw);
    const auto& id=staged_.identity;
    const auto reason=id.source_wire_seq!=current.source_wire_seq || id.source_recv_seq!=current.source_recv_seq?
        Source:id.parent_plan_id!=current.parent_plan_id?Parent:Other;
    cancelStaged(reason,now);++admission_diagnostics_.staged_identity_rejected;
  }
  if(staged_valid_ && now<staged_.valid_until_sec && staged_.gauge.revision!=gauge_revision_) {
    // Fold callbacks may precede this tick's source replacement. Only the
    // current identity check above permits transporting an already staged plan.
    if(transportPreviewExecutionResult(staged_,gauge(),config_.preview_execution.tracker.feasibility_tolerance))
      ++telemetry_.staged_gauge_transported;
    else {++telemetry_.gauge_transport_failed;cancelStaged(Other,now);}
  }
  if(staged_valid_ && now>=staged_.splice_at_sec) {
    PreviewMotionSample candidate;
    // Recheck current authority before retiring the predecessor. In particular,
    // an obsolete contact forecast must not replace a finite brake and then
    // start a new brake from its latest sample, renewing the stopping clock.
    const bool expired=now>=staged_.valid_until_sec;
    const bool sampled=!expired && staged_.trajectory.sample(now-staged_.splice_at_sec,candidate);
    const bool contact_ok=sampled && contactAllows(candidate,raw_sample,contact_gate,contact_normal);
    if(!contact_ok) {
      cancelStaged(expired?Expiry:!sampled?Sample:Contact,now);
      if(expired)++admission_diagnostics_.staged_expired;
      else if(!sampled)++admission_diagnostics_.staged_sample_rejected;
      else {
        auto& d=admission_diagnostics_;
        ++d.staged_contact_rejected;d.last_contact_reject_time_sec=now;
        d.last_contact_reject_gate=contact_gate;d.last_contact_reject_normal=contact_normal;
        d.last_contact_reject_closing_m_s=contact_normal.dot(candidate.linear_velocity);
        d.last_contact_reject_allowed_m_s=std::max(0.0,contact_normal.dot(
            Eigen::Vector3d(raw_sample.velocity.x,raw_sample.velocity.y,raw_sample.velocity.z)));
      }
    }
    else {
      active_=staged_;staged_valid_=false;++telemetry_.accepted;
      telemetry_.last_admission_gap_sec=telemetry_.last_admission_time_sec>0?now-telemetry_.last_admission_time_sec:0;
      telemetry_.last_admission_time_sec=now;
      telemetry_.last_admitted_request_id=active_.identity.request_id;
      telemetry_.last_admitted_parent_plan_id=active_.identity.parent_plan_id;
      brake_trajectory_.valid=false;brake_plan_id_=0;angular_continuation_.clear();
      telemetry_.plan_id=active_.identity.request_id;
    }
  }
  if(active_.accepted()) {
    if(now>=active_.valid_until_sec ||
       !active_.trajectory.sample(now-active_.splice_at_sec,sample_)) {
      ++telemetry_.expired;
      if(!beginBrake("braking_expired")){out.fault=true;out.reason=telemetry_.status;return out;}
    } else if(!contactAllows(sample_,raw_sample,contact_gate,contact_normal)) {
      if(!contactGuardStopped()){out.fault=true;out.reason=telemetry_.status;return out;}
    }
  }
  if(brake_trajectory_.valid) {
    if((angular_continuation_.needsBrake(now) && !beginAngularBrake()) || !sampleBrake()) {
      out.fault=true;out.reason=telemetry_.status;return out;
    }
    out.pose=sample_.pose;out.active=true;
    telemetry_.active=false;telemetry_.status=brake_reason_;
    telemetry_.plan_age_sec=now-brake_origin_sec_;
    // The terminal hold must actually pass dispatch before the fault policy
    // suppresses further sends. Reaching its timestamp in the planner alone
    // would abandon the final stop sample one tick early.
    if(stop_fault_reason_ && accepted_plan_id_==brake_plan_id_ &&
       angular_continuation_.terminalHoldAvailableAt(accepted_sample_time_sec_) &&
       accepted_sample_time_sec_-brake_origin_sec_>=brake_trajectory_.durationSec()) {
      const char* reason=stop_fault_reason_;fail(reason);
      out.fault=true;out.active=false;out.reason=reason;return out;
    }
  } else if(active_.accepted()) {
    out.pose=sample_.pose;out.active=true;
    telemetry_.active=accepted_epoch_;telemetry_.status="active";
    telemetry_.plan_age_sec=now-active_.generated_at_sec;
  } else {
    out.pose=cold_.pose;telemetry_.status="waiting";
    if(now-initialized_at_>=config_.preview_execution.max_result_age_sec) {
      ++telemetry_.expired;fail("first_plan_timeout");out.fault=true;out.reason=telemetry_.status;return out;
    }
  }
  FollowerOutputKinematics phase_reference;
  if(!historySample(cursor_.timeSec(),phase_reference)) {
    stop_fault_reason_="history_unavailable";
    if(!beginBrake("braking_history")){out.fault=true;out.active=false;}
    else {out.pose=sample_.pose;out.active=true;}
    out.reason=telemetry_.status;return out;
  }
  const auto phase=cursor_.step(now,phase_reference,out.pose);
  if(!phase.valid) {
    stop_fault_reason_="backlog_exceeded";
    if(!beginBrake("braking_backlog")){out.fault=true;out.active=false;}
    else {out.pose=sample_.pose;out.active=true;}
    out.reason=telemetry_.status;return out;
  }
  telemetry_.backlog_sec=phase.backlog_sec;telemetry_.rate=phase.rate;
  if(!stop_fault_reason_ && !staged_valid_ && now>=next_request_at_) {
    request_.identity=identity(raw);request_.identity.request_id=++request_id_;request_.gauge=gauge();
    request_.generated_at_sec=now;request_.splice_at_sec=now+config_.preview_execution.splice_lead_sec;
    request_.valid_until_sec=now+config_.preview_execution.max_result_age_sec;
    request_.cursor_time_sec=phase.time_sec;request_.cursor_rate=phase.rate;
    request_.history_count=history_count_;
    for(std::size_t i=0;i<history_count_;++i)request_.history[i]=history_[(history_begin_+i)%history_.size()];
    request_.contact_gate=contact_gate;request_.contact_normal_stand=contact_normal;
    request_.cold_start=!hasPlan();request_.cold_initial=cold_;
    request_.predecessor=active_.trajectory;
    request_.has_brake_predecessor=brake_trajectory_.valid;
    request_.brake_predecessor=brake_trajectory_;
    request_.angular_predecessor=angular_continuation_;
    request_.predecessor_origin_sec=brake_trajectory_.valid?brake_origin_sec_:active_.splice_at_sec;
    if(worker_.trySubmit(raw,request_)) {++telemetry_.submitted;next_request_at_=now+config_.preview_execution.replan_period_sec;}
  }
  out.reason=telemetry_.status;return out;
}

void LivePreviewExecution::shiftCommonFrame(const Eigen::Vector3d& dp,const Eigen::Quaterniond& dR,
    PreviewFoldCause cause,std::uint64_t booked_ns,std::uint64_t applied_ns,std::uint32_t geometry_mask) {
  if(!initialized_)return;
  if(!dp.allFinite()||!dR.coeffs().allFinite()||!std::isfinite(dR.norm())||dR.norm()==0 ||
     !(fold_translation_+dp).allFinite()) {fail("invalid_fold");return;}
  const auto q=dR.normalized();
  const double at=applied_ns?static_cast<double>(applied_ns)*1e-9:last_time_;
  const bool transport=cause==PreviewFoldCause::GeometryHold;
  // Only the explicitly tagged geometry correction is a proven common gauge.
  // Force/ROI/unknown corrections retain the prior authority invalidation.
  if(!transport) {++gate_revision_;cancelStaged(Fold,at);}
  fold_translation_+=dp;fold_rotation_=(q*fold_rotation_).normalized();++gauge_revision_;
  if(active_.accepted() && !transportPreviewExecutionResult(active_,gauge(),
      config_.preview_execution.tracker.feasibility_tolerance)) {fail("invalid_fold");return;}
  // A staged plan retains its captured gauge until step() verifies current
  // source/parent/epoch/authority; then it is transported before activation.
  if(brake_trajectory_.valid)brake_trajectory_.shiftCommonFrame(dp,q);
  angular_continuation_.shiftCommonFrame(dp,q);
  shiftPose(cold_.pose,dp,q);shiftSample(sample_,dp,q);
  if(accepted_epoch_)shiftSample(accepted_sample_,dp,q);
  for(std::size_t i=0;i<history_count_;++i)shiftPose(history_[(history_begin_+i)%history_.size()].state.pose,dp,q);
  telemetry_.fold_cause=cause;telemetry_.fold_booked_time_ns=booked_ns;
  telemetry_.fold_applied_time_ns=applied_ns;telemetry_.fold_geometry_cause_mask=geometry_mask;
  telemetry_.fold_revision=++telemetry_.fold_count;
  switch(cause) {
    case PreviewFoldCause::Force:++telemetry_.fold_force_count;break;
    case PreviewFoldCause::RoiFloor:++telemetry_.fold_roi_floor_count;break;
    case PreviewFoldCause::GeometryHold:++telemetry_.fold_geometry_hold_count;break;
    default:++telemetry_.fold_unknown_count;break;
  }
  for(std::size_t i=0;i<3;++i)telemetry_.fold_translation_m[i]=dp[i];
  for(std::size_t i=0;i<4;++i)telemetry_.fold_quaternion_xyzw[i]=q.coeffs()[i];
  telemetry_.fold_booked_translation_m=telemetry_.fold_translation_m;
  telemetry_.fold_booked_quaternion_xyzw=telemetry_.fold_quaternion_xyzw;
}

PreviewExecutionGauge LivePreviewExecution::gauge() const {
  return {gauge_revision_,fold_translation_,fold_rotation_};
}
void LivePreviewExecution::cancelStaged(std::size_t reason,double now) {
  if(!staged_valid_)return;
  ++telemetry_.rejected;++telemetry_.staged_cancel_counts[reason];
  telemetry_.last_staged_cancel_reason=kPreviewStagedCancelNames[reason];
  telemetry_.last_staged_cancel_time_sec=now;
  telemetry_.last_staged_cancel_request_id=staged_.identity.request_id;
  staged_valid_=false;
}
void LivePreviewExecution::recordResult(PreviewExecutionAcceptance check,double observed) {
  ++admission_diagnostics_.result_checks[static_cast<std::size_t>(check)];
  const auto& r=received_;auto& t=telemetry_;
  t.result_valid=true;t.result_solve_attempted=r.solve_attempted;
  t.last_worker_status=kPreviewWorkerStatusNames[static_cast<std::size_t>(r.status)];
  t.last_solve_status=r.solve_attempted?kPreviewSolveStatusNames[static_cast<std::size_t>(r.diagnostics.status)]:
      r.status==PreviewExecutionWorkerStatus::WorkerException?"unavailable_worker_exception":"not_attempted";
  t.last_admission_reason=kPreviewResultCheckNames[static_cast<std::size_t>(check)];
  t.result_request_id=r.identity.request_id;t.result_epoch=r.identity.epoch;
  t.result_gate_revision=r.identity.gate_revision;t.result_gauge_revision=r.gauge.revision;
  t.result_source_wire_seq=r.identity.source_wire_seq;t.result_source_recv_seq=r.identity.source_recv_seq;
  t.result_parent_plan_id=r.identity.parent_plan_id;t.result_generated_at_sec=r.generated_at_sec;
  t.result_splice_at_sec=r.splice_at_sec;t.result_valid_until_sec=r.valid_until_sec;
  t.result_completed_at_sec=r.completed_at_sec;t.result_observed_at_sec=observed;
  t.solve_iterations=r.diagnostics.working_set_recalculations;
  t.solve_time_sec=r.diagnostics.solve_time_sec;
  t.solve_angular_norm_coupled=r.diagnostics.angular_norm_coupled;
  t.solve_angular_norm_cuts=r.diagnostics.angular_norm_cuts;
  t.solve_max_angular_chart_velocity_norm=r.diagnostics.max_angular_chart_velocity_norm;
  t.solve_max_angular_chart_acceleration_norm=r.diagnostics.max_angular_chart_acceleration_norm;
  t.result_initial_linear_velocity_max_m_s=r.solve_attempted?r.initial.linear_velocity.cwiseAbs().maxCoeff():0;
  t.result_initial_linear_acceleration_max_m_s2=r.solve_attempted?r.initial.linear_acceleration.cwiseAbs().maxCoeff():0;
  t.result_initial_angular_velocity_norm_rad_s=r.solve_attempted?r.initial.angular_velocity_body.norm():0;
  t.result_initial_angular_acceleration_norm_rad_s2=r.solve_attempted?r.initial.angular_acceleration_body.norm():0;
  t.solve_contact_constrained=r.diagnostics.contact_constrained;
  t.solve_contact_decomposed=r.diagnostics.contact_decomposed;
  t.solve_contact_coupled_fallback=r.diagnostics.contact_coupled_fallback;
  t.solve_max_constraint_violation=r.diagnostics.max_constraint_violation;
  t.solve_max_contact_velocity_violation_m_s=r.diagnostics.max_contact_velocity_violation_m_s;
}
const PreviewExecutionTelemetry& LivePreviewExecution::telemetry() const {
  auto& t=telemetry_;const auto& d=admission_diagnostics_;const auto w=worker_.diagnostics();
  t.gate_revision=gate_revision_;t.gauge_revision=gauge_revision_;t.request_id=request_id_;
  for(std::size_t i=0;i<3;++i)t.gauge_translation_m[i]=fold_translation_[i];
  for(std::size_t i=0;i<4;++i)t.gauge_quaternion_xyzw[i]=fold_rotation_.coeffs()[i];
  t.parent_plan_id=brake_trajectory_.valid?brake_plan_id_:(active_.accepted()?active_.identity.request_id:0);
  t.worker_status_counts=w.worker_status_counts;t.solve_status_counts=w.solve_status_counts;
  t.request_invalid=w.request_invalid;t.request_mailbox_full=w.request_mailbox_full;
  t.request_coalesced=w.request_coalesced;t.result_publish_dropped=w.result_publish_dropped;t.result_coalesced=w.result_coalesced;
  t.result_checks=d.result_checks;t.ready_not_staged=d.ready_not_staged;
  t.staged_identity_rejected=d.staged_identity_rejected;t.staged_expired=d.staged_expired;
  t.staged_sample_rejected=d.staged_sample_rejected;t.staged_contact_rejected=d.staged_contact_rejected;
  t.angular_continuations_started=d.angular_continuations_started;t.angular_brakes_started=d.angular_brakes_started;
  t.last_contact_reject_time_sec=d.last_contact_reject_time_sec;t.last_contact_reject_gate=d.last_contact_reject_gate;
  t.last_contact_reject_closing_m_s=d.last_contact_reject_closing_m_s;t.last_contact_reject_allowed_m_s=d.last_contact_reject_allowed_m_s;
  for(std::size_t i=0;i<3;++i)t.last_contact_reject_normal[i]=d.last_contact_reject_normal[i];
  return t;
}
void LivePreviewExecution::shiftSample(PreviewMotionSample& sample,const Eigen::Vector3d& dp,
    const Eigen::Quaterniond& dR) {
  shiftPose(sample.pose,dp,dR);sample.angular_jerk_stand=dR*sample.angular_jerk_stand;
}
PreviewDispatchTransaction LivePreviewExecution::transaction(const Pose6D& nominal,const Pose6D& composed) const {
  PreviewDispatchTransaction tx;
  tx.valid=initialized_&&!faulted_&&hasPlan();tx.epoch=epoch_;tx.plan_id=telemetry_.plan_id;
  tx.sample_time_sec=last_time_;tx.nominal=nominal;tx.composed=composed;tx.motion=sample_;
  tx.fold_translation=fold_translation_;tx.fold_rotation=fold_rotation_;
  return tx;
}
bool LivePreviewExecution::observeDispatch(const PreviewDispatchTransaction& tx,const Pose6D& emitted,
    bool accepted,double pos_tolerance,double rot_tolerance) {
  if(!tx.valid || tx.epoch!=epoch_)return true;
  if(!accepted) {fail("dispatch_rejected");return false;}
  if(tx.plan_id==0 || !std::isfinite(tx.sample_time_sec) || tx.sample_time_sec<=0 ||
     tx.sample_time_sec>last_time_ || tx.sample_time_sec<accepted_sample_time_sec_ ||
     !finitePreviewPose(tx.nominal)||!finitePreviewPose(tx.composed)||!finitePreviewPose(emitted)||
     !finitePreviewPose(tx.motion.pose)||!tx.motion.linear_velocity.allFinite()||
     !tx.motion.linear_acceleration.allFinite()||!tx.motion.angular_velocity_body.allFinite()||
     !tx.motion.angular_acceleration_body.allFinite()||!tx.motion.linear_jerk.allFinite()||
     !tx.motion.angular_jerk_stand.allFinite()||!tx.fold_translation.allFinite()||
     !tx.fold_rotation.coeffs().allFinite()||
     std::abs(tx.fold_rotation.norm()-1)>config_.preview_execution.tracker.feasibility_tolerance||
     !std::isfinite(pos_tolerance)||!std::isfinite(rot_tolerance)||!(pos_tolerance>0)||!(rot_tolerance>0)) {
    fail("invalid_acceptance");return false;
  }
  // The analytical state must belong to this exact nominal target. The looser
  // final IK acceptance envelope cannot authorize a mismatched derivative seed.
  const double precision=config_.preview_execution.tracker.feasibility_tolerance;
  if((xyz(tx.motion.pose)-xyz(tx.nominal)).norm()>precision||
     angle(tx.motion.pose,tx.nominal)>precision) {
    fail("invalid_acceptance");return false;
  }
  auto T=math::se3FromPose(emitted);
  T.translation()-=xyz(tx.composed)-xyz(tx.nominal);
  const auto gauge=math::rotationFromPose(tx.composed)*math::rotationFromPose(tx.nominal).transpose();
  T.rotation()=gauge.transpose()*T.rotation();
  const auto nominal=math::poseFromSe3(T);
  telemetry_.accepted_position_error_m=(xyz(nominal)-xyz(tx.nominal)).norm();
  telemetry_.accepted_rotation_error_rad=angle(nominal,tx.nominal);
  if(telemetry_.accepted_position_error_m>pos_tolerance || telemetry_.accepted_rotation_error_rad>rot_tolerance) {
    fail("accepted_deviation");return false;
  }
  if(tx.sample_time_sec>=accepted_sample_time_sec_) {
    accepted_sample_=tx.motion;accepted_sample_time_sec_=tx.sample_time_sec;
    accepted_plan_id_=tx.plan_id;
    const Eigen::Quaterniond change=fold_rotation_*tx.fold_rotation.conjugate();
    shiftSample(accepted_sample_,fold_translation_-tx.fold_translation,change);
  }
  accepted_epoch_=true;telemetry_.active=initialized_&&active_.accepted()&&!brake_trajectory_.valid;
  return true;
}
}  // namespace rb_servo::control
