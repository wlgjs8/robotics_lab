// Offline preview of the authoritative raw follower. This is not an execution
// controller: force-direction output lead constraints, accepted-output/gauge
// reconciliation and real-time allocation/timing acceptance are not implemented.
#pragma once

#include "rb_servo/control/cartesian_chunk_follower.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iterator>
#include <limits>
#include <vector>

namespace rb_servo::control {

enum class FollowerPreviewReferenceStatus {
  Ready,
  InvalidRequest,
  Inactive,
  Paused,
  Expired,
  NonfiniteSample,
  EvaluationFailed,
};

struct FollowerPreviewReferenceRequest {
  // Explicit sampling choices, not motion limits. A 24-step/240-ms optimizer
  // requests 25 samples at .010 sec, including the current sample at t=0.
  std::size_t sample_count{0};
  double sample_period_sec{0.0};
  double servo_period_sec{0.0};
  double generated_at_sec{std::numeric_limits<double>::quiet_NaN()};
  double valid_until_sec{std::numeric_limits<double>::quiet_NaN()};
  std::uint64_t epoch{0};
  // Caller-owned snapshot revision: change on every real tick, gate change,
  // fold, hold/resume or other mutation, even if the packet IDs are unchanged.
  std::uint64_t revision{0};
};

struct FollowerPreviewReferenceSample {
  double relative_time_sec{0.0};
  FollowerOutputKinematics kinematics{};
  bool stalled{false};  // speculative ring-down, not a live fault
  int step_index{-1};
  // A long preview can extend past the caller's feed/validity deadline.
  // It remains a conditional prediction, not permission to execute that time.
  bool time_before_valid_until{false};
};

struct FollowerPreviewReference {
  FollowerPreviewReferenceStatus status{FollowerPreviewReferenceStatus::InvalidRequest};
  std::vector<FollowerPreviewReferenceSample> samples;
  std::uint64_t epoch{0};
  std::uint64_t revision{0};
  std::uint64_t source_wire_seq{0};
  std::uint64_t source_recv_seq{0};
  double generated_at_sec{std::numeric_limits<double>::quiet_NaN()};
  double valid_until_sec{std::numeric_limits<double>::quiet_NaN()};
  double servo_period_sec{0.0};
  double sample_period_sec{0.0};
  double first_stall_relative_time_sec{std::numeric_limits<double>::quiet_NaN()};
  std::size_t non_stalled_sample_count{0};
  std::size_t canonical_ticks{0};
  std::size_t fractional_ticks{0};

  // Validity is for reuse of THIS snapshot. It does not predict future packet,
  // contact or safety updates. Packet matching alone cannot detect a stale gate
  // or fold: callers must also match their epoch and snapshot revision.
  bool isCurrent(double now_sec, std::uint64_t current_epoch,
                 std::uint64_t current_revision, std::uint64_t wire_seq,
                 std::uint64_t recv_seq) const {
    return status == FollowerPreviewReferenceStatus::Ready &&
        std::isfinite(now_sec) && now_sec >= generated_at_sec &&
        now_sec < valid_until_sec && epoch == current_epoch &&
        revision == current_revision && source_wire_seq == wire_seq &&
        source_recv_seq == recv_seq;
  }
};

inline FollowerPreviewReference makeFollowerPreviewReference(
    const CartesianChunkFollower& live,
    const FollowerPreviewReferenceRequest& request) {
  FollowerPreviewReference out;
  out.epoch = request.epoch;
  out.revision = request.revision;
  out.source_wire_seq = live.windowWireSeq();
  out.source_recv_seq = live.windowRecvSeq();
  out.generated_at_sec = request.generated_at_sec;
  out.valid_until_sec = request.valid_until_sec;
  out.servo_period_sec = request.servo_period_sec;
  out.sample_period_sec = request.sample_period_sec;
  // Bound computational work and allocation even for a malformed offline input.
  constexpr std::size_t kMaxSamples = 256;
  constexpr std::size_t kMaxCanonicalTicks = 4096;
  if (request.sample_count == 0 || request.sample_count > kMaxSamples ||
      !std::isfinite(request.sample_period_sec) || request.sample_period_sec <= 0.0 ||
      !std::isfinite(request.servo_period_sec) || request.servo_period_sec <= 0.0 ||
      !std::isfinite(request.generated_at_sec) ||
      !std::isfinite(request.valid_until_sec)) return out;
  const double horizon = (request.sample_count - 1) * request.sample_period_sec;
  if (!std::isfinite(horizon) ||
      horizon / request.servo_period_sec > kMaxCanonicalTicks) return out;
  if (request.valid_until_sec <= request.generated_at_sec) {
    out.status = FollowerPreviewReferenceStatus::Expired;
    return out;
  }
  if (!live.active()) {
    out.status = FollowerPreviewReferenceStatus::Inactive;
    return out;
  }
  if (live.holdPaused()) {
    out.status = FollowerPreviewReferenceStatus::Paused;
    return out;
  }

  const auto finite_sample = [](const FollowerOutputKinematics& k) {
    const double values[] = {k.pose.x, k.pose.y, k.pose.z, k.pose.rx, k.pose.ry, k.pose.rz,
        k.velocity.x, k.velocity.y, k.velocity.z,
        k.velocity.rx, k.velocity.ry, k.velocity.rz,
        k.acceleration.x, k.acceleration.y, k.acceleration.z,
        k.acceleration.rx, k.acceleration.ry, k.acceleration.rz};
    if (!std::all_of(std::begin(values), std::end(values),
                     [](double v) { return std::isfinite(v); })) return false;
    if (k.pose.quaternion_xyzw.has_value()) {
      double squared_norm = 0.0;
      for (double q : *k.pose.quaternion_xyzw) {
        if (!std::isfinite(q)) return false;
        squared_norm += q*q;
      }
      if (!std::isfinite(squared_norm) || squared_norm <= 0.0) return false;
    }
    return true;
  };
  const auto note_stall = [&out](const CartesianChunkFollower& f, double t) {
    if (f.diag().stall && (!std::isfinite(out.first_stall_relative_time_sec) ||
                         t < out.first_stall_relative_time_sec))
      out.first_stall_relative_time_sec = t;
  };
  const auto append = [&](const CartesianChunkFollower& f, double t) {
    FollowerPreviewReferenceSample sample;
    sample.relative_time_sec = t;
    sample.kinematics = f.outputKinematics();
    // Exactly preserve the loop-facing pose, including its Euler representation.
    sample.kinematics.pose = f.lastPose();
    sample.stalled = f.diag().stall;
    sample.step_index = f.diag().seg_step_index;
    sample.time_before_valid_until = t < request.valid_until_sec - request.generated_at_sec;
    if (!finite_sample(sample.kinematics)) return false;
    if (!sample.stalled) ++out.non_stalled_sample_count;
    note_stall(f, t);
    out.samples.push_back(sample);
    return true;
  };

  // All future solves/counters/window consumption belong to this private copy.
  // No speculative diagnostic is returned as an executable live fault request.
  try {
   auto future = live;
   out.samples.reserve(request.sample_count);
   for (std::size_t sample_index = 0; sample_index < request.sample_count; ++sample_index) {
    const double t = sample_index * request.sample_period_sec;
    const double ratio = t / request.servo_period_sec;
    const double nearest = std::round(ratio);
    const double eps = 32.0 * std::numeric_limits<double>::epsilon() *
                       std::max(1.0, std::abs(ratio));
    const auto target_tick = static_cast<std::size_t>(
        std::abs(ratio - nearest) <= eps ? nearest : std::floor(ratio));
    while (out.canonical_ticks < target_tick) {
      future.tick(request.servo_period_sec);
      ++out.canonical_ticks;
      note_stall(future, out.canonical_ticks * request.servo_period_sec);
    }
    const double remainder = t - out.canonical_ticks * request.servo_period_sec;
    bool good = false;
    if (remainder > eps * request.servo_period_sec) {
      // A fractional output knot must not split the canonical 500-Hz rollout:
      // sample it from a second copy, then continue the canonical copy unchanged.
      auto fractional = future;
      fractional.tick(remainder);
      ++out.fractional_ticks;
      good = append(fractional, t);
    } else {
      good = append(future, t);
    }
    if (!good) {
      out.samples.clear();
      out.status = FollowerPreviewReferenceStatus::NonfiniteSample;
      return out;
    }
   }
  } catch (const std::exception&) {
    // A future sample/solve that fails is only an unavailable preview. Never
    // translate it into a live follower fault or alter the authoritative state.
    out.samples.clear();
    out.status = FollowerPreviewReferenceStatus::EvaluationFailed;
    return out;
  }
  out.status = FollowerPreviewReferenceStatus::Ready;
  return out;
}

}  // namespace rb_servo::control
