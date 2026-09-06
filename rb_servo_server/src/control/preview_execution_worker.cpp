#include "rb_servo/control/preview_execution_worker.hpp"
#include "rb_servo/control/preview_execution_cursor.hpp"
#include "rb_servo/core/clock.hpp"
#include "rb_servo/math/se3.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <thread>

namespace rb_servo::control {
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
  result.initial.pose=math::poseFromSe3(initial);
  for(std::size_t i=0;i<=result.trajectory.count;++i)
    result.trajectory.p.row(i).head<3>()+=dp.transpose();
  result.trajectory.rotation0=dR*result.trajectory.rotation0;
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
  std::array<std::unique_ptr<RequestSlot>, kSlots> requests;
  std::array<ResultSlot, kSlots> results;
  std::array<std::atomic<std::uint64_t>,8> worker_status_counts{}, solve_status_counts{};
  std::atomic<std::uint64_t> request_invalid{0}, request_mailbox_full{0}, request_coalesced{0};
  std::atomic<std::uint64_t> result_publish_dropped{0}, result_coalesced{0};
  std::atomic<bool> stopping{false};
  std::thread thread;

  Impl(const PreviewTrackerConfig& tracker_cfg, const CartesianChunkFollowerConfig& follower_cfg,
       const PreviewExecutionWorkerConfig& worker_cfg)
      : cfg(worker_cfg), tracker(tracker_cfg) {
    static_assert(std::atomic<std::uint64_t>::is_always_lock_free);
    static_assert(std::atomic<SlotState>::is_always_lock_free,
                  "Preview mailboxes require lock-free slot ownership");
    if (!positive(cfg.servo_period_sec) || !positive(cfg.poll_period_sec) ||
        !positive(cfg.max_request_age_sec) || cfg.poll_period_sec >= cfg.max_request_age_sec ||
        cfg.max_snapshot_horizon == 0 || cfg.max_snapshot_horizon > 256 ||
        (tracker.durationSec() + cfg.max_request_age_sec)/cfg.servo_period_sec + 2 > kMaxFutureSamples ||
        2*(std::ceil(tracker.durationSec()/cfg.servo_period_sec)+2)-1>PreviewContactConstraint::kCapacity)
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
      if (!std::isfinite(r.predecessor_origin_sec) || !predecessor_valid ||
          r.identity.parent_plan_id == 0 || r.splice_at_sec < r.predecessor_origin_sec ||
          (!r.has_brake_predecessor && r.splice_at_sec > r.predecessor_origin_sec + predecessor_duration))
        return finish(PreviewExecutionWorkerStatus::SpliceUnavailable);
      // Absolute endpoint comparison above permits only cancellation roundoff,
      // not a late or expired predecessor, to be clamped at its own endpoint.
      const double t = r.has_brake_predecessor?r.splice_at_sec-r.predecessor_origin_sec:
          std::clamp(r.splice_at_sec-r.predecessor_origin_sec,0.0,predecessor_duration);
      if (!(r.has_brake_predecessor?r.brake_predecessor.sample(t, initial):r.predecessor.sample(t, initial)))
        return finish(PreviewExecutionWorkerStatus::SpliceUnavailable);
      if(r.has_brake_predecessor && !r.angular_predecessor.sample(r.splice_at_sec,initial))
        return finish(PreviewExecutionWorkerStatus::SpliceUnavailable);
      out.initial = initial;
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

    PreviewReference reference;
    reference.count = tracker.config().horizon_steps + 1;
    for (std::size_t k = 0; k < reference.count; ++k) {
      const double relative = k * tracker.config().planning_dt_sec;
      const double from_generation = lead + relative;
      const double time = std::min(r.generated_at_sec + from_generation,
          r.cursor_time_sec + r.cursor_rate * from_generation);
      FollowerOutputKinematics state;
      const bool valid = time <= r.generated_at_sec ? sampleHistory(r, time, state)
                                                   : sampleFuture(future, time, state);
      if (!valid) return finish(PreviewExecutionWorkerStatus::PreviewUnavailable);
      reference.knots[k].time_sec = relative;
      reference.knots[k].pose = state.pose;
    }
    PreviewContactConstraint contact;
    if(contact_active) {
      contact.enabled=true;contact.normal_stand=r.contact_normal_stand;
      const auto intervals=static_cast<std::size_t>(std::ceil(tracker.durationSec()/cfg.servo_period_sec))+1;
      if(2*intervals+1>contact.knots.size())return finish(PreviewExecutionWorkerStatus::InvalidRequest);
      const double first_canonical_tick=std::floor(lead/cfg.servo_period_sec)+1;
      double previous_time=0,previous_velocity=0;
      for(std::size_t k=0;k<=intervals;++k) {
        // Preserve original canonical-grid corners even for a splice between
        // servo ticks. Resampling onto a shifted grid would change its bound.
        const double relative=k==0?0.:std::min((first_canonical_tick+k-1)*cfg.servo_period_sec-lead,
                                             tracker.durationSec());
        FollowerOutputKinematics allowed;
        // Authority follows canonical wall time, never a lagged/catching-up
        // tracking cursor or old historical contact direction.
        if(!sampleFuture(future,r.splice_at_sec+relative,allowed))
          return finish(PreviewExecutionWorkerStatus::PreviewUnavailable);
        const double velocity=r.contact_normal_stand.dot(
            Eigen::Vector3d{allowed.velocity.x,allowed.velocity.y,allowed.velocity.z});
        // The canonical follower already applied its advance gate. Multiplying
        // by contact_gate again would impose a second attenuation. Escape is
        // unrestricted; zero crossing insertion represents max(0, linear v)
        // exactly, instead of a secant that invents positive closing authority.
        if(k && ((previous_velocity<0&&velocity>0)||(previous_velocity>0&&velocity<0))) {
          const double crossing=previous_time+(relative-previous_time)*
              (-previous_velocity)/(velocity-previous_velocity);
          if(crossing>previous_time && crossing<relative)
            contact.knots[contact.count++]={crossing,0.0};
        }
        contact.knots[contact.count++]={relative,std::max(0.0,velocity)};
        previous_time=relative;previous_velocity=velocity;
        if(relative==tracker.durationSec())break;
      }
    }
    // Solver state is strictly worker-owned. Never publish its previous result
    // when this request fails; the servo owns the predecessor's finite lifetime.
    out.solve_attempted = true;
    const auto solved = tracker.plan(reference, out.initial,contact);
    out.diagnostics = solved.diagnostics;
    if (!solved.accepted()) return finish(PreviewExecutionWorkerStatus::SolveRejected);
    if (!tracker.exportTrajectory(out.trajectory))
      return finish(PreviewExecutionWorkerStatus::SolveRejected);
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
