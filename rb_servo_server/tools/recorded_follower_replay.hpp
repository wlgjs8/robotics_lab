// Strict offline-only recorded-consumer replay. No backend, socket or device.
// This wrapper is also compilable against an archived baseline include/library
// tree; optional new jerk-search fields are detected at compile time.
#pragma once
#include "rb_servo/config/config.hpp"
#include "rb_servo/control/cartesian_chunk_follower.hpp"
#include "rb_servo/control/follower_output_smd.hpp"
#include "rb_servo/control/plan_gate.hpp"
#include "rb_servo/math/se3.hpp"
#include <nlohmann/json.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

namespace recorded_follower_replay {
using namespace rb_servo;
using namespace rb_servo::control;
using json = nlohmann::json;

template<class T, class = void> struct HasDeadline : std::false_type {};
template<class T> struct HasDeadline<T, std::void_t<decltype(std::declval<T>().deadline_jerk_minimization)>> : std::true_type {};
template<class T, class = void> struct HasJerkDiag : std::false_type {};
template<class T> struct HasJerkDiag<T, std::void_t<decltype(std::declval<T>().jerk_scale), decltype(std::declval<T>().jerk_search_calculations)>> : std::true_type {};
template<class C, class R> void copyDeadline(C& c, const R& r) {
    if constexpr (HasDeadline<C>::value && HasDeadline<R>::value) c.deadline_jerk_minimization = r.deadline_jerk_minimization;
}
template<class D> double jerkScale(const D& d) {
    if constexpr (HasJerkDiag<D>::value) return d.jerk_scale;
    else return 1.0;
}
template<class D> int jerkCalculations(const D& d) {
    if constexpr (HasJerkDiag<D>::value) return d.jerk_search_calculations;
    else return 0;
}

inline double finiteNumber(const json& value) {
    if (!value.is_number()) throw std::runtime_error("required numeric replay value");
    const double v = value.get<double>();
    if (!std::isfinite(v)) throw std::runtime_error("nonfinite replay value");
    return v;
}
template<std::size_t N> std::array<double, N> finiteArray(const json& value) {
    if (!value.is_array() || value.size() != N) throw std::runtime_error("wrong replay array dimension");
    std::array<double, N> out{};
    for (std::size_t i = 0; i < N; ++i) out[i] = finiteNumber(value[i]);
    return out;
}
inline Pose6D readPose(const json& value) {
    const auto v = finiteArray<7>(value);
    Eigen::Quaterniond q(v[6], v[3], v[4], v[5]);
    if (q.norm() < 1e-9) throw std::runtime_error("zero replay quaternion");
    return math::poseFromSe3(pinocchio::SE3(q.normalized().toRotationMatrix(), Eigen::Vector3d(v[0], v[1], v[2])));
}
inline Vec6 readTwist(const json& value) {
    const auto v = finiteArray<6>(value);
    return {v[0], v[1], v[2], v[3], v[4], v[5]};
}
inline Pose6D strip(const Pose6D& emitted, const std::array<double, 6>& d, bool enabled) {
    if (!enabled) return emitted;
    Eigen::Matrix3d rotation = math::rotationFromPose(emitted);
    const Eigen::Vector3d er(d[3], d[4], d[5]);
    // Match AdmittanceOverlay::strip's exact angular no-op threshold.
    if (er.norm() >= 1e-9) rotation = math::exp3(er).transpose() * rotation;
    return math::poseFromSe3(pinocchio::SE3(rotation, Eigen::Vector3d(emitted.x-d[0], emitted.y-d[1], emitted.z-d[2])));
}
inline CartesianChunkFollowerConfig followerConfig(const RuckigFollowerConfig& r) {
    CartesianChunkFollowerConfig c;
    c.lin = {r.max_linear_velocity_m_s, r.max_linear_accel_m_s2, r.max_linear_jerk_m_s3};
    c.ang = {r.max_angular_velocity_rad_s, r.max_angular_accel_rad_s2, r.max_angular_jerk_rad_s3};
    c.window = {r.discard_head_steps, r.consume_steps, r.reserve_steps, r.smoothing_window};
    c.guard.af_damping_beta_lin = r.af_damping_beta_lin;
    c.guard.af_damping_beta_ang = r.af_damping_beta_ang;
    c.guard.corner_deadband_lin_m = r.corner_deadband_lin_m;
    c.guard.corner_deadband_ang_rad = r.corner_deadband_ang_rad;
    c.guard.corner_velocity_scale = r.corner_velocity_scale;
    c.max_projection_error_m = r.preview_max_projection_error_m;
    c.max_projection_error_rad = r.preview_max_projection_error_rad;
    c.max_consecutive_projection_errors = r.preview_max_consecutive_projection_errors;
    c.max_actual_lead_m = r.preview_max_actual_lead_m;
    c.max_actual_lead_rad = r.preview_max_actual_lead_rad;
    c.max_consecutive_actual_lead_errors = r.preview_max_consecutive_actual_lead_errors;
    c.core_time_stretch_enable = r.core_time_stretch_enable;
    c.core_time_stretch_max_ratio = r.core_time_stretch_max_ratio;
    c.fresh_chunk_replan = r.fresh_chunk_replan;
    c.continuous_hold_resume = r.continuous_hold_resume;
    copyDeadline(c, r);
    return c;
}
inline void writePose(std::ostream& out, const Pose6D& p) {
    const Eigen::Quaterniond q(math::rotationFromPose(p));
    out << ',' << p.x << ',' << p.y << ',' << p.z << ',' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w();
}
inline void writeTwist(std::ostream& out, const Vec6& v) {
    out << ',' << v.x << ',' << v.y << ',' << v.z << ',' << v.rx << ',' << v.ry << ',' << v.rz;
}

// Offline reconstruction only: the original log's magnitude is an independent
// residual check, never the source of a guessed direction. The caller applies
// this once on the following tick, after raw advancement and before planning.
struct ReconstructedHoldFold {
    Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
    std::uint64_t booked_tick{0};
    double magnitude_residual_m{0};
};
inline ReconstructedHoldFold readReconstructedHoldFold(const json& j, std::uint64_t tick) {
    if(j.at("provenance")!="reconstructed_same_tick_stage_to_overlay_stripped_command_fk" ||
       j.at("apply_order")!="next_tick_after_raw_before_preview" ||
       j.at("booked_tick").get<std::uint64_t>()!=tick ||
       j.at("rotation_independently_verified").get<bool>())
        throw std::runtime_error("unsupported reconstructed fold provenance/order");
    const auto dp=finiteArray<3>(j.at("translation_m"));
    const auto q=finiteArray<4>(j.at("rotation_xyzw"));
    ReconstructedHoldFold f;f.translation={dp[0],dp[1],dp[2]};f.rotation={q[3],q[0],q[1],q[2]};f.booked_tick=tick;
    const double magnitude=finiteNumber(j.at("recorded_magnitude_m"));
    f.magnitude_residual_m=finiteNumber(j.at("magnitude_residual_m"));
    if(magnitude<=0 || std::abs(f.rotation.norm()-1)>1e-9 ||
       f.magnitude_residual_m<0 || f.magnitude_residual_m>2e-9 ||
       std::abs(f.translation.norm()-magnitude)>2e-9)
        throw std::runtime_error("invalid reconstructed hold fold");
    return f;
}

template<class T,class=void> struct HasLiveAudit:std::false_type {};
template<class T> struct HasLiveAudit<T,std::void_t<decltype(std::declval<T>().fold_cause),
    decltype(std::declval<T>().staged_cancel_counts),decltype(std::declval<T>().result_gauge_transported)>>:std::true_type {};
template<class Live> void applyReconstructedHoldFold(Live& live,const ReconstructedHoldFold& f,
                                                    std::uint64_t applied,bool transport) {
    if constexpr(HasLiveAudit<std::decay_t<decltype(live.telemetry())>>::value) {
        using Cause=std::decay_t<decltype(live.telemetry().fold_cause)>;
        // No per-row cause mask is recoverable from the original CSV.
        live.shiftCommonFrame(f.translation,f.rotation,transport?Cause::GeometryHold:Cause::Unknown,
                              f.booked_tick,applied,0);
    } else {
        if(transport)throw std::runtime_error("geometry transport unavailable in archived coordinator");
        live.shiftCommonFrame(f.translation,f.rotation);
    }
}
template<class Telemetry> json liveAudit(const Telemetry& t) {
    json j={{"available",HasLiveAudit<Telemetry>::value}};
    if constexpr(HasLiveAudit<Telemetry>::value) {
#define AUDIT(field) j[#field]=t.field
        AUDIT(gate_revision);AUDIT(gauge_revision);AUDIT(parent_plan_id);AUDIT(request_id);
        AUDIT(result_valid);AUDIT(result_solve_attempted);AUDIT(last_worker_status);AUDIT(last_solve_status);AUDIT(last_admission_reason);
        AUDIT(worker_status_counts);AUDIT(solve_status_counts);AUDIT(result_checks);AUDIT(staged_cancel_counts);
        AUDIT(last_staged_cancel_reason);AUDIT(last_staged_cancel_time_sec);AUDIT(last_staged_cancel_request_id);
        AUDIT(last_admission_time_sec);AUDIT(last_admission_gap_sec);AUDIT(last_admitted_request_id);AUDIT(last_admitted_parent_plan_id);
        AUDIT(result_request_id);AUDIT(result_epoch);AUDIT(result_gate_revision);AUDIT(result_gauge_revision);
        AUDIT(result_source_wire_seq);AUDIT(result_source_recv_seq);AUDIT(result_parent_plan_id);
        AUDIT(result_generated_at_sec);AUDIT(result_splice_at_sec);AUDIT(result_valid_until_sec);
        AUDIT(result_completed_at_sec);AUDIT(result_observed_at_sec);
        AUDIT(result_gauge_transported);AUDIT(staged_gauge_transported);AUDIT(gauge_transport_failed);
        AUDIT(fold_count);AUDIT(fold_booked_time_ns);AUDIT(fold_applied_time_ns);AUDIT(fold_revision);
        AUDIT(fold_geometry_cause_mask);AUDIT(fold_translation_m);AUDIT(fold_quaternion_xyzw);
        AUDIT(brake_counts);AUDIT(last_brake_reason);AUDIT(last_brake_start_time_sec);AUDIT(last_brake_origin_sec);
        AUDIT(request_invalid);AUDIT(request_mailbox_full);AUDIT(request_coalesced);AUDIT(result_publish_dropped);AUDIT(result_coalesced);
#undef AUDIT
    }
    return j;
}

inline int run(const char* config_path, const char* profile_name, const char* input_path, const char* output_path,
               bool conservative_stage_leash = false) {
    const auto all = loadConfigFromYaml(config_path);
    const RuckigFollowerConfig* r = nullptr;
    for (const auto& profile : all.cartesian_control.tcp_pose_target_profiles) {
        if (profile.name != profile_name) continue;
        if (r) throw std::runtime_error("ambiguous recorded replay profile");
        r = &profile.ruckig_follower;
    }
    if (!r || !r->enable || r->controller != RuckigFollowerController::DeltaPreview)
        throw std::runtime_error("selected recorded replay profile must enable delta_preview");
    const auto cfg = followerConfig(*r);
    CartesianChunkFollower f(cfg);
    FollowerOutputSmd smd(r->output_smd);
    const bool physical = r->fresh_chunk_replan || r->continuous_hold_resume;
    std::ifstream input(input_path);
    std::ofstream output(output_path);
    if (!input || !output) throw std::runtime_error("cannot open recorded replay input/output");
    output << "tick,t,mono,segment,wire_seq,recv_seq,step,t_in_segment,segment_length,policy_dt,duration,converged,stall,solve_failures,projection_m,projection_rad,actual_lead_m,actual_lead_rad,lead_fault,projection_fault,gate,plan_gate,core_gate,reseeded,lag_m,lag_rad,jerk_scale,jerk_search_calculations,compute_us,recorded_plan_gate,recomputed_leash_gate";
    for (const auto* prefix : {"reference", "raw", "stage", "target", "actual", "observed_raw", "observed_stage"})
        for (const auto* axis : {"x", "y", "z", "qx", "qy", "qz", "qw"}) output << ',' << prefix << '_' << axis;
    for (const auto* prefix : {"sample_v", "sample_a", "target_v", "target_a", "axis_duration"})
        for (int i=0; i<6; ++i) output << ',' << prefix << '_' << i;
    output << '\n' << std::setprecision(17);
    double previous_time = -1e100, policy_dt = 0;
    std::uint64_t previous_tick = 0, previous_recv = 0;
    std::size_t rows = 0, active_rows = 0, frames = 0, resets = 0;
    std::optional<Pose6D> previous_stage;
    std::string line;
    while (std::getline(input, line)) {
        const auto j = json::parse(line);
        if (j.at("schema") != "robotics_lab.recorded_follower_input.v2") throw std::runtime_error("wrong recorded replay schema");
        const auto tick = j.at("tick").get<std::uint64_t>();
        const double t = finiteNumber(j.at("t"));
        const double mono = finiteNumber(j.at("mono"));
        const double dt = finiteNumber(j.at("dt"));
        if (dt <= 0 || (rows && (tick <= previous_tick || t <= previous_time))) throw std::runtime_error("invalid recorded replay clock");
        previous_tick = tick; previous_time = t; ++rows;
        if (!j.at("active").get<bool>()) {
            if (f.active()) ++resets;
            f.deactivate(); smd.deactivate(); previous_recv = 0;
            previous_stage.reset();
            continue;
        }
        const auto start = std::chrono::steady_clock::now();
        const auto deviation = finiteArray<6>(j.at("reference_deviation"));
        const bool strip_enabled = j.at("reference_strip_enabled").get<bool>();
        const Pose6D reference = strip(readPose(j.at("previous_emitted")), deviation, strip_enabled);
        const Pose6D actual = strip(readPose(j.at("actual")), deviation, strip_enabled);
        double recomputed_leash = 1.0;
        if (conservative_stage_leash && r->plan_leash_enable && f.active() && previous_stage) {
            const Pose6D prior_raw = f.lastPose();
            const double ep = Eigen::Vector3d(prior_raw.x-previous_stage->x, prior_raw.y-previous_stage->y,
                                              prior_raw.z-previous_stage->z).norm();
            const double ea = math::log3(math::rotationFromPose(prior_raw).transpose()*math::rotationFromPose(*previous_stage)).norm();
            PlanLeashParams leash;
            leash.start_m=r->plan_leash_start_m; leash.start_rad=r->plan_leash_start_rad;
            leash.full_m=r->plan_leash_full_m; leash.full_rad=r->plan_leash_full_rad;
            leash.min_gate=r->plan_leash_min_gate;
            recomputed_leash = planLeashGate(ep, ea, leash);
        }
        if (j.contains("frame")) {
            const auto& b = j.at("frame");
            ChunkFrame frame;
            frame.wire_seq = b.at("wire_seq").get<std::uint64_t>();
            frame.recv_seq = b.at("recv_seq").get<std::uint64_t>();
            frame.recv_time = finiteNumber(b.at("recv_time"));
            frame.policy_dt = finiteNumber(b.at("policy_dt"));
            if (!frame.wire_seq || !frame.recv_seq || frame.recv_seq == previous_recv || frame.policy_dt <= 0 || frame.recv_time > mono)
                throw std::runtime_error("invalid consumed frame identity/time");
            for (const auto& row : b.at("delta")) {
                const auto v = finiteArray<7>(row);
                frame.delta.push_back({v[0],v[1],v[2],v[3],v[4],v[5]});
                frame.grip.push_back(v[6]);
            }
            if (frame.delta.size() < 2) throw std::runtime_error("recorded delta frame too short");
            f.submitDeltaFrame(frame, reference);
            previous_recv = frame.recv_seq; policy_dt = frame.policy_dt; ++frames;
        }
        if (!f.active()) throw std::runtime_error("active recorded tick lacks a valid seed frame");
        const double gate = finiteNumber(j.at("advance_gate"));
        const double recorded_plan_gate = finiteNumber(j.at("plan_rate_gate"));
        // Recorded total gate cannot be decomposed into independent safety and
        // leash here. Conservatively retain it and only add the candidate leash;
        // this experiment cannot release an intervention in the original run.
        const double plan_gate = conservative_stage_leash ? std::min(recorded_plan_gate, recomputed_leash) : recorded_plan_gate;
        const auto dir = finiteArray<3>(j.at("advance_direction"));
        if (gate < 0 || gate > 1 || recorded_plan_gate < 0 || recorded_plan_gate > 1) throw std::runtime_error("recorded gate outside [0,1]");
        f.setAdvanceGate(gate, Eigen::Vector3d(dir[0], dir[1], dir[2]));
        f.setPlanRateGate(plan_gate);
        const Pose6D raw = f.tick(dt);
        const auto sampled = f.outputKinematics();
        const Vec6 velocity = physical ? sampled.velocity :
            (r->output_smd.profile_feedforward ? f.sampledVelocity() : f.currentVelocity()).value_or(Vec6{});
        const Vec6 accel = physical ? sampled.acceleration : f.sampledAcceleration().value_or(Vec6{});
        bool reseeded = false;
        Pose6D stage = raw;
        if (r->output_smd.enable) {
            if (!smd.active()) {
                Vec6 seed = velocity;
                if (physical) {
                    const Eigen::Vector3d omega = math::rotationFromPose(reference).transpose() * math::rotationFromPose(sampled.pose) * Eigen::Vector3d(seed.rx, seed.ry, seed.rz);
                    seed.rx = omega.x(); seed.ry = omega.y(); seed.rz = omega.z();
                }
                smd.reset(reference, seed); reseeded = true;
            }
            stage = smd.step(raw, velocity, dt, r->output_smd.profile_feedforward ? &accel : nullptr, physical);
            reseeded = reseeded || smd.reseededLastStep();
        }
        f.updateActualLead(actual);
        previous_stage = stage;
        const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now()-start).count();
        const auto& d = f.diag();
        output << tick << ',' << t << ',' << mono << ',' << d.segments << ',' << d.seg_wire_seq << ',' << d.seg_recv_seq << ',' << d.seg_step_index
               << ',' << f.tInSegment() << ',' << f.segmentLengthSec() << ',' << policy_dt << ',' << d.last_solve.duration << ',' << d.last_solve.converged
               << ',' << d.stall << ',' << d.solve_failure_count << ',' << d.projection_error_m << ',' << d.projection_error_rad
               << ',' << d.actual_lead_m << ',' << d.actual_lead_rad << ',' << d.actual_lead_fault << ',' << d.infeasible_fault
               << ',' << gate << ',' << plan_gate << ',' << f.coreGate() << ',' << reseeded << ',' << smd.lagPos() << ',' << smd.lagAng()
               << ',' << jerkScale(d.last_solve) << ',' << jerkCalculations(d.last_solve) << ',' << us
               << ',' << recorded_plan_gate << ',' << recomputed_leash;
        for (const auto& p : {reference, raw, stage, d.seg_target_stand, actual, readPose(j.at("observed_prefilter")), readPose(j.at("observed_stage"))}) writePose(output, p);
        writeTwist(output, sampled.velocity); writeTwist(output, sampled.acceleration);
        for (const auto* values : {&d.last_solve.target_velocity, &d.last_solve.target_acceleration, &d.last_solve.axis_duration_sec})
            for (const auto v : *values) output << ',' << v;
        output << '\n'; ++active_rows;
    }
    if (!rows || !active_rows || !output.good()) throw std::runtime_error("empty or unwritable recorded replay");
    std::cout << "recorded replay profile=" << profile_name << " rows=" << rows << " active_rows=" << active_rows
              << " consumed_frames=" << frames << " lifecycle_deactivations=" << resets
              << " physical_derivatives=" << physical << " optional_jerk_fields=" << HasJerkDiag<SegmentSolve>::value
              << " conservative_stage_leash=" << conservative_stage_leash << '\n';
    return 0;
}
}  // namespace recorded_follower_replay
