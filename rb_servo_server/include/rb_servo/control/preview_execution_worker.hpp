#pragma once

#include "rb_servo/control/cartesian_chunk_follower.hpp"
#include "rb_servo/control/preview_trajectory_tracker.hpp"
#include "rb_servo/control/preview_brake.hpp"
#include "rb_servo/control/follower_preview_reference.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>

namespace rb_servo::control {

struct PreviewExecutionWorkerConfig {
  double servo_period_sec{0.0};
  double poll_period_sec{0.0};
  double max_request_age_sec{0.0};
  std::size_t max_snapshot_horizon{0};
};

struct PreviewExecutionIdentity {
  std::uint64_t epoch{0};
  // Authority revision; coordinate-only geometry transport has its own stamp.
  std::uint64_t gate_revision{0};
  std::uint64_t source_wire_seq{0};
  std::uint64_t source_recv_seq{0};
  std::uint64_t parent_plan_id{0};
  std::uint64_t request_id{0};
};

// Cumulative common-reference gauge within an epoch. Translation is additive
// in stand coordinates; orientation is left multiplication only. This is NOT
// an SE(3) rotation of position/linear derivatives.
struct PreviewExecutionGauge {
  std::uint64_t revision{0};
  Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
};

struct PreviewExecutionHistoryEntry {
  double time_sec{0.0};
  FollowerOutputKinematics state{};
};

// Fixed storage, filled by the single servo producer. History is in chronological
// order and ends exactly at generated_at_sec. The copied follower is AFTER that
// tick. Neither historical poses nor the conditional future grants permission to
// cross a force/safety boundary; the current output guard remains authoritative.
struct PreviewExecutionRequest {
  static constexpr std::size_t kHistoryCapacity = 128;
  PreviewExecutionIdentity identity{};
  PreviewExecutionGauge gauge{};
  double generated_at_sec{std::numeric_limits<double>::quiet_NaN()};
  double splice_at_sec{std::numeric_limits<double>::quiet_NaN()};
  double valid_until_sec{std::numeric_limits<double>::quiet_NaN()};
  double cursor_time_sec{std::numeric_limits<double>::quiet_NaN()};
  double cursor_rate{0.0};
  // Leash the forecast inside the constrained planner. Execution and splice
  // derivatives always use physical seconds; no output time dilation.
  double reference_rate{1.0};
  // Physical force authority, independent of whether the RAW current velocity
  // happens to point into the contact. Normal points INTO contact (opposite F).
  double contact_gate{1.0};
  Eigen::Vector3d contact_normal_stand{Eigen::Vector3d::Zero()};
  std::array<PreviewExecutionHistoryEntry, kHistoryCapacity> history{};
  std::size_t history_count{0};
  // The predecessor is the currently accepted nominal output, sampled AT the
  // future splice time by the worker, not at request generation or completion.
  PreviewPolynomialTrajectory predecessor{};
  PreviewBrakeTrajectory brake_predecessor{};
  // A translation contact stop can retain an independently finite angular
  // polynomial/brake. Its own original absolute deadline remains authoritative.
  PreviewAngularContinuation angular_predecessor{};
  bool has_brake_predecessor{false};
  double predecessor_origin_sec{std::numeric_limits<double>::quiet_NaN()};
  // Only an explicitly stationary, accepted held pose permits a cold start.
  // Nonzero cold derivatives are rejected, never clipped to zero.
  bool cold_start{false};
  PreviewMotionState cold_initial{};
  // NO DISPATCHED-STATE OFFSET (2026-09-11). The executor used to hold the active
  // plan back at a contact and carry the refused displacement here so a replan could
  // splice from what the arm was actually sent. That hold-back is deleted - its
  // book-and-retire owned 91-100 % of the violent command accelerations in the day's
  // policy runs - so the predecessor's own sample IS the dispatched state, and the
  // splice needs nothing extra. What remains is the authority WIDENING below: the
  // spliced state may still close faster than the new knot-0 bound whenever the
  // follower's gated authority fell between request and splice, and refusing that as
  // Infeasible fed the expiry-brake cycle (27 refusals in 5 s, 2026-09-10 pm).
};

enum class PreviewExecutionWorkerStatus {
  Solved,
  InvalidRequest,
  SourceMismatch,
  PreviewUnavailable,
  SpliceUnavailable,
  SolveRejected,
  Late,
  WorkerException,
};

struct PreviewExecutionPhaseReference {
  static constexpr std::size_t kCapacity=256;
  std::array<FollowerPreviewReferenceSample,kCapacity> samples{};
  std::size_t count{0};
};

struct PreviewExecutionResult {
  PreviewExecutionIdentity identity{};
  PreviewExecutionGauge gauge{};
  double generated_at_sec{0.0};
  double splice_at_sec{0.0};
  double valid_until_sec{0.0};
  // The predecessor plan time this plan was spliced from (NaN: brake or cold splice).
  double spliced_predecessor_time_sec{std::numeric_limits<double>::quiet_NaN()};
  double completed_at_sec{0.0};
  PreviewExecutionWorkerStatus status{PreviewExecutionWorkerStatus::InvalidRequest};
  // WorkerException can lose in-flight diagnostics; false then means unknown,
  // not proof the solver was never entered. Counters cover returned QP statuses.
  bool solve_attempted{false};
  PreviewSolveDiagnostics diagnostics{};
  PreviewMotionState initial{};
  PreviewPolynomialTrajectory trajectory{};
  PreviewPolynomialTrajectory nominal_trajectory{};
  PreviewContactConstraint contact_authority{};
  double nominal_solve_time_sec{0.0};
  double trusted_prefix_sec{0.0};
  // Canonical forecast data only, never executable motion authority. Even a
  // rejected QP may expose this snapshot, under its original source/epoch/expiry.
  PreviewExecutionPhaseReference phase_reference{};
  bool accepted() const { return status == PreviewExecutionWorkerStatus::Solved; }
};

// Samples only the overlap with the snapshot's original validity. Parent plan
// identity is not canonical reference authority; epoch/gate/source still are.
bool samplePreviewExecutionPhaseReference(const PreviewExecutionResult& result,
    double reference_time_sec,double now_sec,const PreviewExecutionIdentity& current,
    const PreviewExecutionGauge& gauge,double tolerance,FollowerOutputKinematics& state);

enum class PreviewExecutionAcceptance {
  Ready,
  WorkerRejected,
  EpochMismatch,
  GateMismatch,
  SourceMismatch,
  ParentMismatch,
  Late,
  InvalidTiming,
};

// Admission before staging a future splice. At/after the splice is too late:
// activating a result retrospectively would jump over an unexecuted interval.
PreviewExecutionAcceptance validatePreviewExecutionResult(
    const PreviewExecutionResult& result, double now_sec,
    const PreviewExecutionIdentity& current);

// Called only AFTER ordinary epoch/authority/source/parent/timing admission.
// It changes neither identity nor any timestamp and never extends validity.
bool transportPreviewExecutionResult(PreviewExecutionResult& result,
    const PreviewExecutionGauge& current, double tolerance);

// CONTACT AUTHORITY SLEW. Raises the contact envelope near the splice so that a plan
// starting from the dispatched closing state (velocity, acceleration along the contact
// normal) can meet the authority along the smoothest two-interval jerk profile that
// stays within tracker.contact_slew_jerk_m_s3 (escalating to max_linear_jerk_m_s3 and
// the horizon only if that is impossible). The result is max(envelope, slew) plus the
// servo-grid chord margin of the slew's quadratic pieces; nothing below the original
// envelope is ever admitted, and beyond the slew the envelope is untouched. Returns
// false on invalid input or knot overflow. See PreviewTrackerConfig::contact_slew_jerk_m_s3.
// With `nominal_path` given (and the envelope built by buildContactEnvelope from it
// at `gate`), the slew lands exactly on the scaled candidate's state at T, which makes
// the two-sided tube feasible by construction; without it the landing follows the
// one-sided rule (ceiling / floor / coast).
bool slewContactAuthority(PreviewContactConstraint& contact, double closing_velocity_m_s,
    double closing_acceleration_m_s2, const PreviewTrackerConfig& tracker,
    double servo_period_sec, const PreviewPolynomialTrajectory* nominal_path = nullptr,
    double gate = 1.0);
// Two-sided certified envelope of the free candidate's closing velocity along
// contact.normal_stand (2026-09-16): ceiling g x max(0, controls), floor
// min(g x min, min) - retreat_slack_m_s per servo subinterval, end knots taking the
// adjacent extrema.
bool buildContactEnvelope(PreviewContactConstraint& contact,
    const PreviewPolynomialTrajectory& nominal_path, double gate, double servo_period_sec,
    double retreat_slack_m_s);

struct PreviewExecutionWorkerDiagnostics {
  std::array<std::uint64_t, 8> worker_status_counts{}, solve_status_counts{};
  std::uint64_t request_invalid{0}, request_mailbox_full{0}, request_coalesced{0};
  std::uint64_t result_publish_dropped{0}, result_coalesced{0};
};

// One servo producer/consumer and one private planning thread. Construction,
// QP work, clone preview and destruction are non-RT. trySubmit()/tryTake() scan
// three atomic slots, never wait, and allocate no memory. Full mailboxes drop a
// request/result; old trajectories retain only their original finite validity.
class PreviewExecutionWorker {
 public:
  PreviewExecutionWorker(const PreviewTrackerConfig& tracker_config,
                         const CartesianChunkFollowerConfig& follower_config,
                         const PreviewExecutionWorkerConfig& worker_config);
  ~PreviewExecutionWorker();
  PreviewExecutionWorker(const PreviewExecutionWorker&) = delete;
  PreviewExecutionWorker& operator=(const PreviewExecutionWorker&) = delete;

  bool trySubmit(const CartesianChunkFollower& follower,
                 const PreviewExecutionRequest& request) noexcept;
  bool tryTake(PreviewExecutionResult& result) noexcept;
  static double monotonicNowSec();
  PreviewExecutionWorkerDiagnostics diagnostics() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace rb_servo::control
