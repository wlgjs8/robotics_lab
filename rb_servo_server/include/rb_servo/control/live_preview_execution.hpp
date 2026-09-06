#pragma once

#include "rb_servo/config/config.hpp"
#include "rb_servo/control/preview_execution_cursor.hpp"
#include "rb_servo/control/preview_execution_worker.hpp"

namespace rb_servo::control {

// Travel with the safety-filtered target through next-tick send staging. The
// nominal/composed pair records the force gauge of THAT command, even if the
// live overlay has changed before dispatch. This is not a controller ACK.
struct PreviewDispatchTransaction {
  bool valid{false};
  std::uint64_t epoch{0}, plan_id{0};
  double sample_time_sec{0};
  Pose6D nominal{}, composed{};
  // Analytical p/v/a actually selected for this command. Derivatives describe
  // the nominal command, not an estimate of measured robot motion.
  PreviewMotionSample motion{};
  Eigen::Vector3d fold_translation{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond fold_rotation{Eigen::Quaterniond::Identity()};
};

struct LivePreviewOutput {
  Pose6D pose{};
  bool active{false};
  bool fault{false};
  const char* reason{"waiting"};
};

// Fixed-size, servo-owned audit data. Counters are cumulative across resets,
// like the public telemetry counters; no logging or wire allocation occurs here.
struct LivePreviewAdmissionDiagnostics {
  std::array<std::uint64_t,
      static_cast<std::size_t>(PreviewExecutionAcceptance::InvalidTiming)+1> result_checks{};
  std::uint64_t ready_not_staged{0}, staged_identity_rejected{0};
  std::uint64_t staged_expired{0}, staged_sample_rejected{0}, staged_contact_rejected{0};
  std::uint64_t angular_continuations_started{0}, angular_brakes_started{0};
  double last_contact_reject_time_sec{0}, last_contact_reject_gate{1};
  double last_contact_reject_closing_m_s{0}, last_contact_reject_allowed_m_s{0};
  Eigen::Vector3d last_contact_reject_normal{Eigen::Vector3d::Zero()};
};

// Servo-owned finite execution state. Only the private worker forecasts the
// follower or solves QPs. All history, coefficient and mailbox storage is fixed.
class LivePreviewExecution {
 public:
  LivePreviewExecution(const RuckigFollowerConfig& config,
                       const CartesianChunkFollowerConfig& raw_config,
                       double servo_period_sec);
  LivePreviewOutput step(double now_sec, const CartesianChunkFollower& raw,
                         const Pose6D& accepted_nominal, bool stationary,
                         double contact_gate = 1.0,
                         const Eigen::Vector3d& contact_normal_stand = Eigen::Vector3d::Zero());
  void reset(const char* reason = "inactive");
  void fail(const char* reason);
  void shiftCommonFrame(const Eigen::Vector3d& dp, const Eigen::Quaterniond& dR,
                        PreviewFoldCause cause = PreviewFoldCause::Unknown,
                        std::uint64_t booked_time_ns = 0, std::uint64_t applied_time_ns = 0,
                        std::uint32_t geometry_cause_mask = 0);
  // Requests a bounded nominal stop from the last successfully dispatched
  // analytical state. It does not promise zero stopping distance. IK, contact
  // overlay and final joint safety retain their independent veto.
  bool contactGuardStopped();
  PreviewDispatchTransaction transaction(const Pose6D& nominal,
                                         const Pose6D& composed) const;
  bool observeDispatch(const PreviewDispatchTransaction& transaction,
                       const Pose6D& actually_enqueued_emitted,
                       bool accepted, double position_tolerance_m,
                       double rotation_tolerance_rad);
  const PreviewExecutionTelemetry& telemetry() const;
  bool initialized() const { return initialized_; }
  bool failed() const { return faulted_; }
  bool hasPlan() const { return active_.accepted() || brake_trajectory_.valid; }
  bool braking() const { return brake_trajectory_.valid; }
  const PreviewMotionSample& sample() const { return sample_; }
  const PreviewExecutionResult& lastResult() const { return received_; }
  const LivePreviewAdmissionDiagnostics& admissionDiagnostics() const { return admission_diagnostics_; }

 private:
  PreviewExecutionGauge gauge() const;
  void cancelStaged(std::size_t reason, double now);
  void recordResult(PreviewExecutionAcceptance check, double observed_at);
  void append(double now, const FollowerOutputKinematics& state);
  bool historySample(double stamp, FollowerOutputKinematics& out) const;
  PreviewExecutionIdentity identity(const CartesianChunkFollower& raw) const;
  bool beginBrake(const char* reason, bool contact_only = false);
  bool beginAngularBrake();
  bool calculateBrake(const PreviewMotionState& initial, PreviewBrakeTrajectory& output);
  bool sampleBrake();
  bool contactAllows(const PreviewMotionSample& proposed, const FollowerOutputKinematics& raw,
                     double gate, const Eigen::Vector3d& normal) const;
  void shiftSample(PreviewMotionSample& sample, const Eigen::Vector3d& dp,
                   const Eigen::Quaterniond& dR);
  bool stagedCurrent(const CartesianChunkFollower& raw) const;
  RuckigFollowerConfig config_;
  PreviewExecutionWorker worker_;
  PreviewExecutionCursor cursor_;
  PreviewBrake brake_calculator_;
  PreviewBrakeTrajectory brake_trajectory_{};
  PreviewAngularContinuation angular_continuation_{};
  std::array<PreviewExecutionHistoryEntry, PreviewExecutionRequest::kHistoryCapacity> history_{};
  std::size_t history_begin_{0}, history_count_{0};
  PreviewExecutionRequest request_{};
  PreviewExecutionResult active_{}, staged_{}, received_{};
  PreviewMotionSample sample_{}, accepted_sample_{};
  PreviewMotionState cold_{};
  mutable PreviewExecutionTelemetry telemetry_{};
  LivePreviewAdmissionDiagnostics admission_diagnostics_{};
  std::uint64_t epoch_{1}, gate_revision_{1}, request_id_{0}, gauge_revision_{0};
  double initialized_at_{0}, last_time_{0}, next_request_at_{0};
  double brake_origin_sec_{0}, accepted_sample_time_sec_{0};
  std::uint64_t brake_plan_id_{0}, accepted_plan_id_{0};
  Eigen::Vector3d fold_translation_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond fold_rotation_{Eigen::Quaterniond::Identity()};
  const char* brake_reason_{"braking"};
  const char* stop_fault_reason_{nullptr};
  bool initialized_{false}, staged_valid_{false}, faulted_{false}, accepted_epoch_{false};
};

}  // namespace rb_servo::control
