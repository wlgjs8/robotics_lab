#pragma once

// Offline progress-owner experiment. Canonical model poses remain immutable;
// this cursor changes only which reference time the output optimizer follows.
// It does not authorize historical motion through a newly closed force gate.
#include "rb_servo/control/follower_preview_reference.hpp"
#include "rb_servo/control/plan_gate.hpp"
#include "rb_servo/math/se3.hpp"
#include <deque>
#include <iterator>
#include <stdexcept>

namespace rb_servo::control {

inline bool finitePreviewPose(const Pose6D& p) {
  for(double v:{p.x,p.y,p.z,p.rx,p.ry,p.rz})if(!std::isfinite(v))return false;
  if(p.quaternion_xyzw) {
    double norm2=0;
    for(double v:*p.quaternion_xyzw) {if(!std::isfinite(v))return false;norm2+=v*v;}
    if(!std::isfinite(norm2)||norm2<1e-24)return false;
  }
  return true;
}
inline bool finitePreviewState(const FollowerOutputKinematics& s) {
  if(!finitePreviewPose(s.pose))return false;
  for(const auto* v:{&s.velocity,&s.acceleration})
    for(double x:{v->x,v->y,v->z,v->rx,v->ry,v->rz})if(!std::isfinite(x))return false;
  return true;
}

struct PreviewExecutionCursorConfig {
  double max_backlog_sec{0};
  double catchup_time_sec{0};
  double max_rate{0};
  double translation_velocity_floor{0};
  double angular_velocity_floor{0};
};
enum class PreviewExecutionCursorStatus { Ready, Inactive, InvalidInput, BacklogExceeded };
struct PreviewExecutionCursorStep {
  bool valid{false};
  PreviewExecutionCursorStatus status{PreviewExecutionCursorStatus::Inactive};
  double time_sec{0}, rate{1}, backlog_sec{0}, gate{1};
  double positive_lag_m{0}, positive_lag_rad{0};
  double cross_track_m{0}, cross_track_rad{0};
};

class PreviewExecutionCursor {
 public:
  PreviewExecutionCursor(const PreviewExecutionCursorConfig& c, const PlanLeashParams& leash)
      : cfg_(c), leash_(leash) {
    const double positive[] = {c.max_backlog_sec,c.catchup_time_sec,c.max_rate,
      c.translation_velocity_floor,c.angular_velocity_floor,leash.start_m,
      leash.start_rad,leash.full_m,leash.full_rad,leash.min_gate};
    for(double v:positive) if(!std::isfinite(v)||v<=0)
      throw std::invalid_argument("explicit positive execution cursor parameters required");
    if(c.max_rate<1 || leash.full_m<=leash.start_m || leash.full_rad<=leash.start_rad || leash.min_gate>1)
      throw std::invalid_argument("invalid execution cursor range");
  }
  void reset(double now) {
    if(!std::isfinite(now)) throw std::invalid_argument("nonfinite cursor epoch");
    time_=wall_=now; active_=true;
  }
  void clear() { active_=false; }
  bool initialized() const { return active_; }
  double timeSec() const { return time_; }
  PreviewExecutionCursorStep step(double now, const FollowerOutputKinematics& ref,
                                  const Pose6D& previous_output) {
    PreviewExecutionCursorStep out;
    if(!active_) return out;
    out.time_sec=time_;
    out.status=PreviewExecutionCursorStatus::InvalidInput;
    const Eigen::Vector3d ep(ref.pose.x-previous_output.x,ref.pose.y-previous_output.y,ref.pose.z-previous_output.z);
    const Eigen::Vector3d v(ref.velocity.x,ref.velocity.y,ref.velocity.z);
    const Eigen::Vector3d w(ref.velocity.rx,ref.velocity.ry,ref.velocity.rz);
    if(!std::isfinite(now)||now<wall_||!finitePreviewState(ref)||!finitePreviewPose(previous_output)||
       !ep.allFinite()||!v.allFinite()||!w.allFinite())return out;
    // Log error is in the reference body; negate so positive projection means
    // the output is behind, rather than ahead along the reference rotation.
    const Eigen::Vector3d er=-math::log3(math::rotationFromPose(ref.pose).transpose()*
                                        math::rotationFromPose(previous_output));
    if(!er.allFinite())return out;
    const double speed=v.norm(), angular_speed=w.norm();
    if(!std::isfinite(speed)||!std::isfinite(angular_speed))return out;
    const double along=speed>=cfg_.translation_velocity_floor?ep.dot(v)/speed:0.;
    const double along_r=angular_speed>=cfg_.angular_velocity_floor?er.dot(w)/angular_speed:0.;
    if(!std::isfinite(along)||!std::isfinite(along_r))return out;
    out.positive_lag_m=std::max(0.,along);
    out.positive_lag_rad=std::max(0.,along_r);
    out.cross_track_m=speed>=cfg_.translation_velocity_floor?(ep-along*v/speed).norm():ep.norm();
    out.cross_track_rad=angular_speed>=cfg_.angular_velocity_floor?(er-along_r*w/angular_speed).norm():er.norm();
    out.gate=planLeashGate(out.positive_lag_m,out.positive_lag_rad,leash_);
    // Recover delayed reference time rather than integrating a permanent delay.
    // The downstream QP still owns all physical velocity/acceleration/jerk caps.
    const double prior_backlog=std::max(0.,wall_-time_);
    out.rate=out.gate*std::clamp(1.+prior_backlog/cfg_.catchup_time_sec,1.,cfg_.max_rate);
    const double dt=now-wall_;
    out.time_sec=std::min(now,time_+dt*out.rate);
    out.backlog_sec=now-out.time_sec;
    if(!std::isfinite(dt)||!std::isfinite(out.rate)||!std::isfinite(out.time_sec)||
       !std::isfinite(out.backlog_sec)||!std::isfinite(out.cross_track_m)||!std::isfinite(out.cross_track_rad))return out;
    if(out.backlog_sec>cfg_.max_backlog_sec) {
      out.status=PreviewExecutionCursorStatus::BacklogExceeded;
      return out; // no clamp/reset of either persistent cursor state
    }
    time_=out.time_sec;wall_=now;out.valid=true;out.status=PreviewExecutionCursorStatus::Ready;
    return out;
  }
 private:
  PreviewExecutionCursorConfig cfg_;
  PlanLeashParams leash_;
  double time_{0},wall_{0};bool active_{false};
};

inline FollowerOutputKinematics interpolatePreviewKinematics(
    const FollowerOutputKinematics& a,const FollowerOutputKinematics& b,double alpha) {
  FollowerOutputKinematics out;
  out.pose=math::interpolateLinear(a.pose,b.pose,true,alpha);
  const auto R=math::rotationFromPose(out.pose),Ra=math::rotationFromPose(a.pose),Rb=math::rotationFromPose(b.pose);
  auto blend=[&](const Vec6& av,const Vec6& bv) {
    const Eigen::Vector3d omega=R.transpose()*((1-alpha)*Ra*Eigen::Vector3d(av.rx,av.ry,av.rz)+
                                                    alpha*Rb*Eigen::Vector3d(bv.rx,bv.ry,bv.rz));
    return Vec6{(1-alpha)*av.x+alpha*bv.x,(1-alpha)*av.y+alpha*bv.y,(1-alpha)*av.z+alpha*bv.z,
                omega.x(),omega.y(),omega.z()};
  };
  // These derivatives are interpolated reference metadata for the local phase
  // estimate, not a claim that interpolated pose has exactly these derivatives.
  out.velocity=blend(a.velocity,b.velocity);out.acceleration=blend(a.acceleration,b.acceleration);
  return out;
}

class CanonicalReferenceHistory {
 public:
  explicit CanonicalReferenceHistory(std::size_t capacity):capacity_(capacity) {
    if(capacity<2||capacity>65536)throw std::invalid_argument("invalid history capacity");
  }
  void clear() { entries_.clear(); }
  void append(double stamp,const FollowerOutputKinematics& state) {
    if(!std::isfinite(stamp)||!finitePreviewState(state)||(!entries_.empty()&&stamp<=entries_.back().stamp))
      throw std::invalid_argument("canonical history timestamp must increase");
    entries_.push_back({stamp,state});
    if(entries_.size()>capacity_)entries_.pop_front();
  }
  double latestTimeSec() const {
    return entries_.empty()?std::numeric_limits<double>::quiet_NaN():entries_.back().stamp;
  }
  bool sample(double time,FollowerOutputKinematics& out) const {
    if(entries_.empty()||!std::isfinite(time)||time<entries_.front().stamp||time>entries_.back().stamp)return false;
    auto hi=std::lower_bound(entries_.begin(),entries_.end(),time,
      [](const Entry& e,double t){return e.stamp<t;});
    if(hi->stamp==time||hi==entries_.begin()) {out=hi->state;return true;}
    const auto lo=std::prev(hi);
    out=interpolatePreviewKinematics(lo->state,hi->state,(time-lo->stamp)/(hi->stamp-lo->stamp));
    return true;
  }
 private:
  struct Entry {double stamp;FollowerOutputKinematics state;};
  std::size_t capacity_;std::deque<Entry> entries_;
};

inline bool sampleKnownReference(const CanonicalReferenceHistory& history,
                                const FollowerPreviewReference& future,
                                double time,FollowerOutputKinematics& out) {
  if(!std::isfinite(time))return false;
  if(time<=history.latestTimeSec())return history.sample(time,out);
  if(future.status!=FollowerPreviewReferenceStatus::Ready||future.samples.size()<2||
     future.generated_at_sec!=history.latestTimeSec())return false;
  // Compare absolute endpoints first: subtracting a ~1.6e6-second epoch can
  // round a valid endpoint above its small relative-time representation.
  const double last=future.samples.back().relative_time_sec;
  if(time<future.generated_at_sec||time>future.generated_at_sec+last)return false;
  const double t=std::clamp(time-future.generated_at_sec,0.,last);
  auto hi=std::lower_bound(future.samples.begin(),future.samples.end(),t,
    [](const FollowerPreviewReferenceSample& e,double v){return e.relative_time_sec<v;});
  if(hi==future.samples.end())return false;
  if(hi->relative_time_sec==t||hi==future.samples.begin()) {out=hi->kinematics;return true;}
  const auto lo=std::prev(hi);
  out=interpolatePreviewKinematics(lo->kinematics,hi->kinematics,
    (t-lo->relative_time_sec)/(hi->relative_time_sec-lo->relative_time_sec));
  return true;
}

// Catch-up ends when execution reaches canonical wall time. Projecting a rate
// >1 indefinitely would manufacture a future phase lead after backlog recovery.
inline double previewCursorReferenceTime(double now,double relative_time,
                                         const PreviewExecutionCursorStep& cursor) {
  if(!cursor.valid||!std::isfinite(now)||!std::isfinite(relative_time)||relative_time<0||
     !std::isfinite(cursor.time_sec)||!std::isfinite(cursor.rate)||cursor.rate<0)
    return std::numeric_limits<double>::quiet_NaN();
  return std::min(now+relative_time,cursor.time_sec+cursor.rate*relative_time);
}
} // namespace rb_servo::control
