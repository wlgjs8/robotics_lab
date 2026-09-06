// Offline-only experiment. No backend/server/device is constructed. The raw
// follower and strict recorded schema remain owned by the existing replay.
#include "recorded_follower_replay.hpp"
#include "rb_servo/control/follower_preview_reference.hpp"
#include "rb_servo/control/preview_execution_cursor.hpp"
#include "rb_servo/control/preview_trajectory_tracker.hpp"
#include <algorithm>
#include <iostream>
#include <optional>
#include <set>
#include <sstream>

namespace {
using namespace rb_servo;
using namespace rb_servo::control;
namespace rr = recorded_follower_replay;
using json = nlohmann::json;

void exactKeys(const json& j, const std::set<std::string>& keys, const char* name) {
    if (!j.is_object() || j.size() != keys.size()) throw std::runtime_error(std::string(name)+": missing/extra fields");
    for (const auto& item : j.items()) if (!keys.count(item.key())) throw std::runtime_error(std::string(name)+": unknown field "+item.key());
}
struct Parameters {
    PreviewTrackerConfig tracker;
    double replan_period_sec = 0;
    double preview_servo_period_sec = 0;
};
PreviewExecutionCursorConfig readExecutionCursorParameters(const char* path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open explicit execution-cursor parameters");
    const json j=json::parse(in);
    exactKeys(j, {"schema", "max_backlog_sec", "catchup_time_sec", "max_rate",
        "translation_velocity_floor", "angular_velocity_floor"}, "execution cursor");
    if(j.at("schema")!="robotics_lab.preview_execution_cursor.v1")
        throw std::runtime_error("unsupported execution-cursor schema");
    PreviewExecutionCursorConfig p;
#define FIELD(name) p.name=rr::finiteNumber(j.at(#name))
    FIELD(max_backlog_sec); FIELD(catchup_time_sec); FIELD(max_rate);
    FIELD(translation_velocity_floor); FIELD(angular_velocity_floor);
#undef FIELD
    if(p.max_backlog_sec<=0 || p.catchup_time_sec<=0 || p.max_rate<1 ||
       p.translation_velocity_floor<=0 || p.angular_velocity_floor<=0)
        throw std::runtime_error("invalid explicit execution-cursor parameters");
    return p;
}
Parameters readParameters(const char* path) {
    std::ifstream in(path); if (!in) throw std::runtime_error("cannot open explicit tracker parameters");
    const json j = json::parse(in);
    exactKeys(j, {"schema", "replan_period_sec", "preview_servo_period_sec", "initial_state_policy", "tracker"}, "parameters");
    if (j.at("schema") != "robotics_lab.preview_follower_replay.v1" ||
        j.at("initial_state_policy") != "recorded_previous_emitted_nominal_at_rest")
        throw std::runtime_error("unsupported replay schema/initial-state policy");
    Parameters p;
    p.replan_period_sec = rr::finiteNumber(j.at("replan_period_sec"));
    p.preview_servo_period_sec = rr::finiteNumber(j.at("preview_servo_period_sec"));
    if (p.replan_period_sec <= 0 || p.preview_servo_period_sec <= 0) throw std::runtime_error("nonpositive explicit replay period");
    const auto& c=j.at("tracker");
    exactKeys(c, {"planning_dt_sec", "horizon_steps", "max_linear_velocity_m_s", "max_linear_acceleration_m_s2", "max_linear_jerk_m_s3",
      "max_angular_velocity_rad_s", "max_angular_acceleration_rad_s2", "max_angular_jerk_rad_s3",
      "linear_tracking_scale_m", "angular_tracking_scale_rad", "jerk_weight", "jerk_difference_weight",
      "linear_tracking_tolerance_m", "angular_tracking_tolerance_rad", "max_linear_tracking_slack_m", "max_angular_tracking_slack_rad",
      "max_reference_chart_angle_rad", "feasibility_tolerance", "max_working_set_recalculations", "max_solve_time_sec"}, "tracker");
#define FIELD(name) p.tracker.name=rr::finiteNumber(c.at(#name))
    FIELD(planning_dt_sec); FIELD(max_linear_velocity_m_s); FIELD(max_linear_acceleration_m_s2); FIELD(max_linear_jerk_m_s3);
    FIELD(max_angular_velocity_rad_s); FIELD(max_angular_acceleration_rad_s2); FIELD(max_angular_jerk_rad_s3);
    FIELD(linear_tracking_scale_m); FIELD(angular_tracking_scale_rad); FIELD(jerk_weight); FIELD(jerk_difference_weight);
    FIELD(linear_tracking_tolerance_m); FIELD(angular_tracking_tolerance_rad); FIELD(max_linear_tracking_slack_m); FIELD(max_angular_tracking_slack_rad);
    FIELD(max_reference_chart_angle_rad); FIELD(feasibility_tolerance); FIELD(max_solve_time_sec);
#undef FIELD
    if (!c.at("horizon_steps").is_number_integer() || !c.at("max_working_set_recalculations").is_number_integer())
        throw std::runtime_error("integer tracker dimensions required");
    const auto h=c.at("horizon_steps").get<long long>();
    const auto n=c.at("max_working_set_recalculations").get<long long>();
    if(h<1 || h>static_cast<long long>(PreviewTrajectoryTracker::kMaxHorizonSteps) || n<1 || n>1000)
        throw std::runtime_error("tracker dimensions out of range");
    p.tracker.horizon_steps=static_cast<std::size_t>(h);
    p.tracker.max_working_set_recalculations=static_cast<int>(n);
    if(p.replan_period_sec > p.tracker.planning_dt_sec*p.tracker.horizon_steps)
        throw std::runtime_error("replan period longer than accepted trajectory horizon");
    return p;
}
const char* statusName(PreviewSolveStatus s) {
    switch(s) {
#define STATUS(name) case PreviewSolveStatus::name: return #name
      STATUS(Solved); STATUS(InvalidReference); STATUS(InvalidInitialState); STATUS(Infeasible); STATUS(IterationLimit);
      STATUS(TimeBudgetExceeded); STATUS(NumericalFailure); STATUS(TrackingBudgetExceeded);
#undef STATUS
    }
    return "Unknown";
}
double positionError(const Pose6D& a,const Pose6D& b) {return Eigen::Vector3d(a.x-b.x,a.y-b.y,a.z-b.z).norm();}
double rotationError(const Pose6D& a,const Pose6D& b) {return math::log3(math::rotationFromPose(a).transpose()*math::rotationFromPose(b)).norm();}
Vec6 velocity(const PreviewMotionState& s) {return {s.linear_velocity.x(),s.linear_velocity.y(),s.linear_velocity.z(),s.angular_velocity_body.x(),s.angular_velocity_body.y(),s.angular_velocity_body.z()};}
Vec6 acceleration(const PreviewMotionState& s) {return {s.linear_acceleration.x(),s.linear_acceleration.y(),s.linear_acceleration.z(),s.angular_acceleration_body.x(),s.angular_acceleration_body.y(),s.angular_acceleration_body.z()};}
PreviewMotionSample motionFromRaw(const FollowerOutputKinematics& k) {
    PreviewMotionSample s; s.pose=k.pose;
    s.linear_velocity={k.velocity.x,k.velocity.y,k.velocity.z};
    s.linear_acceleration={k.acceleration.x,k.acceleration.y,k.acceleration.z};
    s.angular_velocity_body={k.velocity.rx,k.velocity.ry,k.velocity.rz};
    s.angular_acceleration_body={k.acceleration.rx,k.acceleration.ry,k.acceleration.rz}; return s;
}
struct Fingerprint {
    Pose6D pose; FollowerOutputKinematics k; double phase, duration, gate, force_gate;
    double shift; std::size_t index; FollowerDiag diag;
};
Fingerprint fingerprint(const CartesianChunkFollower& f) {
    return {f.lastPose(),f.outputKinematics(),f.tInSegment(),f.segmentLengthSec(),f.planRateGate(),f.advanceGate(),f.planShift(),f.windowIndex(),f.diag()};
}
bool unchanged(const Fingerprint& a,const Fingerprint& b) {
    const auto va=motionFromRaw(a.k),vb=motionFromRaw(b.k);
    return positionError(a.pose,b.pose)==0 && rotationError(a.pose,b.pose)<1e-14 &&
      (va.linear_velocity-vb.linear_velocity).norm()==0 && (va.linear_acceleration-vb.linear_acceleration).norm()==0 &&
      (va.angular_velocity_body-vb.angular_velocity_body).norm()==0 && (va.angular_acceleration_body-vb.angular_acceleration_body).norm()==0 &&
      a.phase==b.phase && a.duration==b.duration && a.gate==b.gate && a.force_gate==b.force_gate && a.shift==b.shift && a.index==b.index &&
      a.diag.segments==b.diag.segments && a.diag.seg_step_index==b.diag.seg_step_index && a.diag.stall_count==b.diag.stall_count &&
      a.diag.solve_failure_count==b.diag.solve_failure_count && a.diag.seg_recv_seq==b.diag.seg_recv_seq;
}

int run(const char* stack,const char* profile,const char* params,const char* events,const char* out_path,bool own_leash,bool reference_only,const char* cursor_params) {
    const auto p=readParameters(params);
    const auto phase=cursor_params?std::optional<PreviewExecutionCursorConfig>(readExecutionCursorParameters(cursor_params)):std::nullopt;
    const double dense_preview_size=phase?std::ceil(
        phase->max_rate*p.tracker.planning_dt_sec*p.tracker.horizon_steps/p.preview_servo_period_sec)+2:0;
    // makeFollowerPreviewReference bounds its sample allocation to 256. Reject
    // incompatible experiment parameters rather than truncating known preview.
    if(phase && (!std::isfinite(dense_preview_size) || dense_preview_size<2 || dense_preview_size>256))
        throw std::runtime_error("execution-cursor dense preview exceeds bounded reference capacity");
    const auto dense_preview_count=static_cast<std::size_t>(dense_preview_size);
    const auto all=loadConfigFromYaml(stack);
    const RuckigFollowerConfig* r=nullptr;
    for(const auto& v:all.cartesian_control.tcp_pose_target_profiles) if(v.name==profile) {
        if(r) throw std::runtime_error("ambiguous profile"); r=&v.ruckig_follower;
    }
    if(!r || !r->enable || r->controller!=RuckigFollowerController::DeltaPreview || !r->fresh_chunk_replan)
        throw std::runtime_error("enabled fresh delta_preview profile required");
    if(phase && !r->plan_leash_enable)throw std::runtime_error("execution cursor requires explicit enabled profile leash parameters");
    PlanLeashParams leash_params;
    leash_params.start_m=r->plan_leash_start_m;leash_params.start_rad=r->plan_leash_start_rad;
    leash_params.full_m=r->plan_leash_full_m;leash_params.full_rad=r->plan_leash_full_rad;
    leash_params.min_gate=r->plan_leash_min_gate;
    std::optional<PreviewExecutionCursor> cursor;
    if(phase)cursor.emplace(*phase,leash_params);
    // Purely computational history bound. An unavailable old sample terminates
    // the experiment; it never substitutes a new delta anchor or guessed pose.
    CanonicalReferenceHistory canonical_history(1024);
    CartesianChunkFollower f(rr::followerConfig(*r));
    PreviewTrajectoryTracker tracker(p.tracker);
    std::ifstream input(events); std::ofstream output(out_path);
    if(!input || !output) throw std::runtime_error("cannot open replay input/output");
    output << "tick,t,mono,segment,wire_seq,recv_seq,step,t_in_segment,segment_length,policy_dt,duration,converged,stall,solve_failures,projection_m,projection_rad,actual_lead_m,actual_lead_rad,lead_fault,projection_fault,gate,plan_gate,core_gate,reseeded,lag_m,lag_rad,jerk_scale,jerk_search_calculations,compute_us,recorded_plan_gate,recomputed_leash_gate,plan_attempt,plan_accepted,solve_status,old_plan_used,plan_age,plan_duration,preview_us,solve_us,working_set_recalculations,constraint_violation,solve_tracking_m,solve_tracking_rad,solve_slack_m,solve_slack_rad,preview_count,preview_nonstalled_count,preview_first_stall_sec,preview_valid_until,clone_zero_pos_error,clone_zero_rot_error,clone_live_mutated,splice_position_error,splice_rotation_error,splice_velocity_error,splice_acceleration_error,stage_actual_lead_m,stage_actual_lead_rad";
    if(phase)output<<",cursor_time,cursor_backlog_sec,cursor_rate,cursor_gate,cursor_positive_lag_m,cursor_positive_lag_rad,cursor_cross_track_m,cursor_cross_track_rad";
    for(const auto* pre:{"reference","raw","stage","target","actual","observed_raw","observed_stage"})
      for(const auto* axis:{"x","y","z","qx","qy","qz","qw"}) output << ',' << pre << '_' << axis;
    if(phase)for(const auto* axis:{"x","y","z","qx","qy","qz","qw"})output<<",cursor_reference_"<<axis;
    for(const auto* pre:{"sample_v","sample_a","target_v","target_a","axis_duration","stage_v","stage_a"})
      for(int i=0;i<6;++i) output << ',' << pre << '_' << i;
    for(const auto* pre:{"stage_linear_jerk","stage_angular_jerk_stand"}) for(int i=0;i<3;++i) output << ',' << pre << '_' << i;
    output << '\n' << std::setprecision(17);
    json summary={{"schema","robotics_lab.preview_follower_replay_summary.v1"},{"status","running"},{"reference_only",reference_only},{"own_stage_leash",own_leash},
      {"initial_state_policy","recorded_previous_emitted_nominal_at_rest"},
      {"scope","Offline; frozen recorded model/force/measured poses. Clone has only current frame and frozen gates. New trajectory starts at current old-plan sample time; no raw fallback."}};
    summary["execution_cursor"]=phase.has_value();
    if(phase) {
        summary["execution_cursor_parameters"]={{"schema","robotics_lab.preview_execution_cursor.v1"},
          {"max_backlog_sec",phase->max_backlog_sec},{"catchup_time_sec",phase->catchup_time_sec},{"max_rate",phase->max_rate},
          {"translation_velocity_floor",phase->translation_velocity_floor},{"angular_velocity_floor",phase->angular_velocity_floor}};
        summary["execution_cursor_scope"]="Offline only. Recorded gates preserve canonical raw; one-sided output lag retimes a separate bounded history cursor. Interpolated derivatives are metadata for local phase estimation, not exact derivatives of the interpolated pose. Current-contact invalidation of old history, gripper phase and production force/accepted-command guards are not implemented.";
    }
    std::size_t rows=0,active=0,frames=0,resets=0,attempts=0,accepted=0,old_used=0;
    std::uint64_t prev_tick=0,prev_recv=0,epoch=0; double prev_t=-1e100,prev_mono=-1e100,policy_dt=0;
    double plan_origin=0,next_replan=-1e100; std::optional<Pose6D> previous_stage;
    json status_counts=json::object(); bool finished=true; std::string line;
    auto terminate=[&](const char* reason,std::uint64_t tick,double t,double mono) {
      summary["status"]="terminated"; summary["termination_reason"]=reason; summary["termination_tick"]=tick;
      summary["termination_time_sec"]=t;summary["termination_mono"]=mono; finished=false;
    };
    while(std::getline(input,line)) {
      const auto j=json::parse(line);
      if(j.at("schema")!="robotics_lab.recorded_follower_input.v2") throw std::runtime_error("wrong recorded input schema");
      const auto tick=j.at("tick").get<std::uint64_t>(); const double t=rr::finiteNumber(j.at("t")),mono=rr::finiteNumber(j.at("mono")),dt=rr::finiteNumber(j.at("dt"));
      if(dt<=0 || (rows && (tick<=prev_tick || t<=prev_t || mono<=prev_mono))) throw std::runtime_error("invalid replay clock");
      prev_tick=tick;prev_t=t;prev_mono=mono;++rows;
      if(!j.at("active").get<bool>()) {
        if(f.active())++resets; f.deactivate();tracker.reset(); previous_stage.reset();prev_recv=0;next_replan=-1e100;++epoch;
        canonical_history.clear();if(cursor)cursor->clear();continue;
      }
      const auto begin=std::chrono::steady_clock::now();
      const auto deviation=rr::finiteArray<6>(j.at("reference_deviation")); const bool strip_enabled=j.at("reference_strip_enabled").get<bool>();
      const Pose6D reference=rr::strip(rr::readPose(j.at("previous_emitted")),deviation,strip_enabled),actual=rr::strip(rr::readPose(j.at("actual")),deviation,strip_enabled);
      double leash=1;
      if(own_leash && r->plan_leash_enable && f.active() && previous_stage) {
        leash=planLeashGate(positionError(f.lastPose(),*previous_stage),rotationError(f.lastPose(),*previous_stage),leash_params);
      }
      if(j.contains("frame")) {
        const auto& b=j.at("frame"); ChunkFrame frame;
        frame.wire_seq=b.at("wire_seq").get<std::uint64_t>();frame.recv_seq=b.at("recv_seq").get<std::uint64_t>();
        frame.recv_time=rr::finiteNumber(b.at("recv_time"));frame.policy_dt=rr::finiteNumber(b.at("policy_dt"));
        if(!frame.wire_seq || !frame.recv_seq || frame.recv_seq==prev_recv || frame.policy_dt<=0 || frame.recv_time>mono) throw std::runtime_error("invalid consumed frame identity");
        for(const auto& row:b.at("delta")) {const auto d=rr::finiteArray<7>(row);frame.delta.push_back({d[0],d[1],d[2],d[3],d[4],d[5]});frame.grip.push_back(d[6]);}
        if(frame.delta.size()<2)throw std::runtime_error("frame too short");
        f.submitDeltaFrame(frame,reference);prev_recv=frame.recv_seq;policy_dt=frame.policy_dt;++frames;
      }
      if(!f.active())throw std::runtime_error("active tick lacks valid frame");
      const double gate=rr::finiteNumber(j.at("advance_gate")),recorded_gate=rr::finiteNumber(j.at("plan_rate_gate"));
      const auto dir=rr::finiteArray<3>(j.at("advance_direction"));
      if(gate<0 || gate>1 || recorded_gate<0 || recorded_gate>1)throw std::runtime_error("gate outside [0,1]");
      const double plan_gate=own_leash?std::min(recorded_gate,leash):recorded_gate;
      f.setAdvanceGate(gate,Eigen::Vector3d(dir[0],dir[1],dir[2]));f.setPlanRateGate(plan_gate);
      const Pose6D raw=f.tick(dt);auto raw_k=f.outputKinematics();if(phase)raw_k.pose=raw;
      PreviewExecutionCursorStep cursor_step;FollowerOutputKinematics cursor_reference=raw_k;
      if(cursor) {
        canonical_history.append(mono,raw_k);
        if(!cursor->initialized())cursor->reset(mono);
        FollowerOutputKinematics old_cursor_reference;
        if(!canonical_history.sample(cursor->timeSec(),old_cursor_reference)) {
          terminate("execution_cursor_history_unavailable",tick,t,mono);break;
        }
        if(previous_stage)cursor_step=cursor->step(mono,old_cursor_reference,*previous_stage);
        else {cursor_step.valid=true;cursor_step.status=PreviewExecutionCursorStatus::Ready;cursor_step.time_sec=mono;}
        if(!cursor_step.valid) {
          terminate("execution_cursor_rejected",tick,t,mono);
          summary["last_cursor_status"]=static_cast<int>(cursor_step.status);
          summary["last_cursor_backlog_sec"]=cursor_step.backlog_sec;break;
        }
        if(!canonical_history.sample(cursor_step.time_sec,cursor_reference)) {
          terminate("execution_cursor_advanced_history_unavailable",tick,t,mono);break;
        }
      }
      PreviewMotionSample current; bool seeded=false;
      if(reference_only)current=motionFromRaw(raw_k);
      else if(tracker.hasTrajectory()) {
        if(!tracker.sample(mono-plan_origin,current)) {terminate("accepted_plan_expired",tick,t,mono);break;}
      } else { current.pose=reference;seeded=true; } // explicit one-tick at-rest cold hold
      bool attempted=false,accepted_now=false,old_plan_used=false,mutated=false;
      double preview_us=0,zero_p=0,zero_r=0,splice_p=0,splice_r=0,splice_v=0,splice_a=0,first_stall=-1,preview_valid_until=0;
      std::size_t preview_count=0,nonstalled_count=0; PreviewSolveResult solve;
      if(mono+1e-12>=next_replan) {
        attempted=true;++attempts;
        if(next_replan < -1e90)next_replan=mono+p.replan_period_sec;
        else { do {next_replan+=p.replan_period_sec;} while(next_replan<=mono); }
        FollowerPreviewReferenceRequest request;
        request.sample_period_sec=phase?p.preview_servo_period_sec:p.tracker.planning_dt_sec;
        request.sample_count=phase?dense_preview_count:p.tracker.horizon_steps+1;request.servo_period_sec=p.preview_servo_period_sec;
        request.generated_at_sec=mono;request.valid_until_sec=mono+std::min(p.replan_period_sec,r->chunk_feed_timeout_sec-f.ageSince(mono));
        request.epoch=epoch;request.revision=tick;
        const auto before=fingerprint(f);const auto preview_start=std::chrono::steady_clock::now();
        const auto future=makeFollowerPreviewReference(f,request);
        preview_us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-preview_start).count();
        mutated=!unchanged(before,fingerprint(f));
        if(mutated)throw std::runtime_error("preview mutated live raw follower");
        PreviewReference refs;
        if(future.status == FollowerPreviewReferenceStatus::Ready) {
          if(!future.isCurrent(mono,epoch,tick,f.windowWireSeq(),f.windowRecvSeq()))
            throw std::runtime_error("preview source identity or validity mismatch");
          preview_count=future.samples.size();nonstalled_count=future.non_stalled_sample_count;
          first_stall=std::isfinite(future.first_stall_relative_time_sec)?future.first_stall_relative_time_sec:-1;preview_valid_until=future.valid_until_sec;
          refs.count=phase?p.tracker.horizon_steps+1:preview_count;
          if(refs.count>PreviewReference::kCapacity)throw std::runtime_error("preview exceeds tracker reference capacity");
          for(std::size_t i=0;i<refs.count;++i) {
            if(phase) {
              const double relative=i*p.tracker.planning_dt_sec;
              FollowerOutputKinematics k;
              if(!sampleKnownReference(canonical_history,future,previewCursorReferenceTime(mono,relative,cursor_step),k))
                throw std::runtime_error("execution cursor reference outside known history/future");
              refs.knots[i]={relative,k.pose};
            } else refs.knots[i]={future.samples[i].relative_time_sec,future.samples[i].kinematics.pose};
          }
          // The retimed knot zero may be in history. Check clone zero against
          // the current canonical raw state, never against that older cursor.
          zero_p=positionError(raw,future.samples[0].kinematics.pose);zero_r=rotationError(raw,future.samples[0].kinematics.pose);
          if(zero_p>1e-12 || zero_r>1e-12)throw std::runtime_error("clone sample zero time misalignment");
        }
        if(!reference_only) {
          solve=tracker.plan(refs,current); const std::string status=statusName(solve.status);
          status_counts[status]=status_counts.value(status,0)+1;
          if(solve.accepted()) {
            ++accepted;accepted_now=true;plan_origin=mono;PreviewMotionSample next;
            if(!tracker.sample(0,next))throw std::runtime_error("accepted plan has no sample zero");
            splice_p=positionError(current.pose,next.pose);splice_r=rotationError(current.pose,next.pose);
            splice_v=std::max((current.linear_velocity-next.linear_velocity).norm(),(current.angular_velocity_body-next.angular_velocity_body).norm());
            splice_a=std::max((current.linear_acceleration-next.linear_acceleration).norm(),(current.angular_acceleration_body-next.angular_acceleration_body).norm());
            current=next;
          } else {
            if(!tracker.hasTrajectory()) {terminate("first_plan_rejected_no_valid_old_plan",tick,t,mono);summary["last_solve_status"]=status;break;}
            old_plan_used=true;++old_used; // current was sampled at the existing origin; it remains unchanged
          }
        }
      }
      f.updateActualLead(actual);previous_stage=current.pose;
      const auto& d=f.diag(); const auto& sd=solve.diagnostics;
      const double compute_us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-begin).count();
      const double age=tracker.hasTrajectory()?mono-plan_origin:0;
      output<<tick<<','<<t<<','<<mono<<','<<d.segments<<','<<d.seg_wire_seq<<','<<d.seg_recv_seq<<','<<d.seg_step_index<<','<<f.tInSegment()<<','<<f.segmentLengthSec()<<','<<policy_dt<<','<<d.last_solve.duration<<','<<d.last_solve.converged<<','<<d.stall<<','<<d.solve_failure_count<<','<<d.projection_error_m<<','<<d.projection_error_rad<<','<<d.actual_lead_m<<','<<d.actual_lead_rad<<','<<d.actual_lead_fault<<','<<d.infeasible_fault<<','<<gate<<','<<plan_gate<<','<<f.coreGate()<<','<<seeded<<','<<positionError(raw,current.pose)<<','<<rotationError(raw,current.pose)<<','<<rr::jerkScale(d.last_solve)<<','<<rr::jerkCalculations(d.last_solve)<<','<<compute_us<<','<<recorded_gate<<','<<leash;
      output<<','<<attempted<<','<<accepted_now<<','<<(attempted&&!reference_only?statusName(solve.status):"NotAttempted")<<','<<old_plan_used<<','<<age<<','<<(tracker.hasTrajectory()?tracker.durationSec():0)<<','<<preview_us<<','<<sd.solve_time_sec*1e6<<','<<sd.working_set_recalculations<<','<<sd.max_constraint_violation<<','<<sd.max_position_tracking_error_m<<','<<sd.max_orientation_tracking_error_rad<<','<<sd.max_position_tracking_slack_m<<','<<sd.max_orientation_tracking_slack_rad<<','<<preview_count<<','<<nonstalled_count<<','<<first_stall<<','<<preview_valid_until<<','<<zero_p<<','<<zero_r<<','<<mutated<<','<<splice_p<<','<<splice_r<<','<<splice_v<<','<<splice_a<<','<<positionError(current.pose,actual)<<','<<rotationError(current.pose,actual);
      if(phase)output<<','<<cursor_step.time_sec<<','<<cursor_step.backlog_sec<<','<<cursor_step.rate<<','<<cursor_step.gate
        <<','<<cursor_step.positive_lag_m<<','<<cursor_step.positive_lag_rad<<','<<cursor_step.cross_track_m<<','<<cursor_step.cross_track_rad;
      for(const auto& v:{reference,raw,current.pose,d.seg_target_stand,actual,rr::readPose(j.at("observed_prefilter")),rr::readPose(j.at("observed_stage"))})rr::writePose(output,v);
      if(phase)rr::writePose(output,cursor_reference.pose);
      rr::writeTwist(output,raw_k.velocity);rr::writeTwist(output,raw_k.acceleration);
      for(const auto* a:{&d.last_solve.target_velocity,&d.last_solve.target_acceleration,&d.last_solve.axis_duration_sec})for(double v:*a)output<<','<<v;
      rr::writeTwist(output,velocity(current));rr::writeTwist(output,acceleration(current));
      for(int i=0;i<3;++i)output<<','<<current.linear_jerk[i];for(int i=0;i<3;++i)output<<','<<current.angular_jerk_stand[i];output<<'\n';++active;
    }
    summary["input_rows_read"]=rows;summary["active_rows_written"]=active;summary["consumed_frames"]=frames;summary["lifecycle_resets"]=resets;
    summary["replan_attempts"]=attempts;summary["accepted_plans"]=accepted;summary["failed_replan_old_plan_used"]=old_used;summary["solve_status_counts"]=status_counts;
    if(finished && !active)terminate("no_active_samples",prev_tick,prev_t,prev_mono);
    if(finished)summary["status"]="completed";
    std::ofstream summary_out(std::string(out_path)+".summary.json");summary_out<<summary.dump(2)<<'\n';std::cout<<summary.dump()<<'\n';
    if(!output.good() || !summary_out.good() || !rows)throw std::runtime_error("empty/unwritable replay");
    return finished?0:2;
}
}
int main(int argc,char** argv) {
  try {
    if(argc<6 || argc>10)throw std::runtime_error("usage: preview_follower_replay CONFIG PROFILE PARAMS.json EVENTS.jsonl OUTPUT.csv [--own-stage-leash] [--reference-only] [--execution-cursor PHASE.json]");
    bool leash=false,reference=false;const char* cursor_params=nullptr;
    for(int i=6;i<argc;++i) {
      const std::string a=argv[i];
      if(a=="--own-stage-leash"&&!leash)leash=true;
      else if(a=="--reference-only"&&!reference)reference=true;
      else if(a=="--execution-cursor"&&!cursor_params&&i+1<argc)cursor_params=argv[++i];
      else throw std::runtime_error("unknown/duplicate option or missing option value");
    }
    if(cursor_params&&(leash||reference))throw std::runtime_error("execution cursor is mutually exclusive with own-stage-leash and reference-only");
    return run(argv[1],argv[2],argv[3],argv[4],argv[5],leash,reference,cursor_params);
  } catch(const std::exception& e) {std::cerr<<"preview_follower_replay: "<<e.what()<<'\n';return 1;}
}
