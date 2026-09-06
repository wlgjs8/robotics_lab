// Recorded reference replay through the actual asynchronous live coordinator.
// No backend, socket, camera or gripper is constructed. Dispatch is an explicitly
// ideal nominal acknowledgement. Recorded tick gate, filtered deadzoned force
// direction and sustained-contact classifier drive the current guard and brake;
// no dynamic plant/force feedback, admittance
// overlay, IK or final joint safety is simulated. Wall pacing controls worker
// delivery opportunities, not robot dt.
#include "recorded_follower_replay.hpp"
#include "rb_servo/control/live_preview_execution.hpp"
#include "rb_servo/control/preview_contact_authority.hpp"
#include "rb_servo/core/clock.hpp"
#include <iostream>
#include <optional>
#include <thread>

namespace {
using namespace rb_servo;
using namespace rb_servo::control;
namespace rr=recorded_follower_replay;
using json=nlohmann::json;
constexpr const char* admissionNames[]={"ready","worker_rejected","epoch_mismatch","gate_mismatch",
    "source_mismatch","parent_mismatch","late","invalid_timing"};

const char* workerStatus(PreviewExecutionWorkerStatus status) {
  switch(status) {
    case PreviewExecutionWorkerStatus::Solved:return "solved";
    case PreviewExecutionWorkerStatus::InvalidRequest:return "invalid_request";
    case PreviewExecutionWorkerStatus::SourceMismatch:return "source_mismatch";
    case PreviewExecutionWorkerStatus::PreviewUnavailable:return "preview_unavailable";
    case PreviewExecutionWorkerStatus::SpliceUnavailable:return "splice_unavailable";
    case PreviewExecutionWorkerStatus::SolveRejected:return "solve_rejected";
    case PreviewExecutionWorkerStatus::Late:return "late";
    case PreviewExecutionWorkerStatus::WorkerException:return "worker_exception";
  }
  return "unknown";
}
const char* solveStatus(PreviewSolveStatus status) {
  switch(status) {
    case PreviewSolveStatus::Solved:return "solved";
    case PreviewSolveStatus::InvalidReference:return "invalid_reference";
    case PreviewSolveStatus::InvalidInitialState:return "invalid_initial_state";
    case PreviewSolveStatus::Infeasible:return "infeasible";
    case PreviewSolveStatus::IterationLimit:return "iteration_limit";
    case PreviewSolveStatus::TimeBudgetExceeded:return "time_budget_exceeded";
    case PreviewSolveStatus::NumericalFailure:return "numerical_failure";
    case PreviewSolveStatus::TrackingBudgetExceeded:return "tracking_budget_exceeded";
  }
  return "unknown";
}

int run(const char* stack,const char* profile,const char* events,const char* contact_path,const char* output_path,
        double wall_pace_sec,std::size_t max_active_rows,const char* options_path) {
  if(!std::isfinite(wall_pace_sec)||wall_pace_sec<0 || wall_pace_sec>.1)
    throw std::runtime_error("wall pace must be explicit, finite and in [0,.1] seconds");
  const auto all=loadConfigFromYaml(stack);
  const RuckigFollowerConfig* selected=nullptr;
  for(const auto& p:all.cartesian_control.tcp_pose_target_profiles)if(p.name==profile) {
    if(selected)throw std::runtime_error("ambiguous profile");selected=&p.ruckig_follower;
  }
  if(!selected)throw std::runtime_error("missing selected profile");
  auto candidate=*selected;const RuckigFollowerConfig* c=&candidate;
  json options=json::object();bool reconstructed_geometry=false,transport_geometry=false;
  if(options_path) {
    std::ifstream o(options_path);if(!o)throw std::runtime_error("cannot read explicit offline options");options=json::parse(o);
    if(options.at("schema")!="robotics_lab.live_preview_replay_options.v1")throw std::runtime_error("wrong offline options schema");
    for(const auto& entry:options.items())
      if(entry.key()!="schema"&&entry.key()!="geometry_fold_mode"&&entry.key()!="jerk_weight"&&
         entry.key()!="jerk_difference_weight"&&entry.key()!="linear_tracking_scale_m"&&entry.key()!="angular_tracking_scale_rad")
        throw std::runtime_error("unknown offline option (physical caps and tolerances are not tunable here)");
    if(options.contains("geometry_fold_mode")) {
      const auto mode=options.at("geometry_fold_mode").get<std::string>();
      if(mode!="invalidate"&&mode!="transport")throw std::runtime_error("geometry fold mode must be invalidate or transport");
      reconstructed_geometry=true;transport_geometry=mode=="transport";
    }
    auto assign=[&](const char* name,double& value){if(options.contains(name)){value=rr::finiteNumber(options.at(name));if(value<=0)throw std::runtime_error("positive offline objective weight/scale required");}};
    assign("jerk_weight",candidate.preview_execution.tracker.jerk_weight);
    assign("jerk_difference_weight",candidate.preview_execution.tracker.jerk_difference_weight);
    assign("linear_tracking_scale_m",candidate.preview_execution.tracker.linear_tracking_scale_m);
    assign("angular_tracking_scale_rad",candidate.preview_execution.tracker.angular_tracking_scale_rad);
  }
  if(!c || !c->enable || !c->preview_execution.enable || c->output_smd.enable ||
      c->controller!=RuckigFollowerController::DeltaPreview)
    throw std::runtime_error("enabled no-LPF live preview delta profile required");
  std::ifstream input(events),contact_input(contact_path);std::ofstream output(output_path);
  std::ofstream worker_output(std::string(output_path)+".worker.jsonl");
  std::ofstream audit_output(std::string(output_path)+".audit.jsonl");
  if(!input||!contact_input||!output||!worker_output||!audit_output)throw std::runtime_error("cannot open replay input/contact/output");
  const auto raw_config=rr::followerConfig(*c);
  CartesianChunkFollower follower(raw_config);
  std::unique_ptr<LivePreviewExecution> live;
  std::optional<Pose6D> accepted;
  std::optional<PreviewMotionSample> previous_accepted_motion;
  std::optional<rr::ReconstructedHoldFold> pending_fold;
  std::size_t applied_folds=0,discarded_folds=0,missing_stage_observations=0;
  double max_fold_residual=0,last_admission_time=0,max_admission_gap=0;
  std::uint64_t previous_admissions=0;
  if(all.servo.rate_hz<=0)throw std::runtime_error("positive server servo rate required");
  const double dt_contract=1.0/all.servo.rate_hz;
  double previous_mono=0,previous_t=0,max_raw_error=0,max_stage_error=0;
  double max_raw_rotation_error=0;
  double max_backlog=0,max_step_compute_us=0,max_dispatch_error=0;
  double max_linear_velocity=0,max_linear_acceleration=0,max_linear_jerk=0;
  double max_angular_velocity=0,max_angular_acceleration=0,max_angular_jerk=0,max_contact_velocity_violation=0;
  std::uint64_t previous_tick=0,previous_recv=0;
  std::size_t input_rows=0,active_rows=0,frames=0,active_output_rows=0;
  std::size_t tracking_rows=0,braking_rows=0,braking_stationary_rows=0,waiting_rows=0,fault_rows=0;
  std::size_t stream_armed_input_rows=0,stream_armed_active_rows=0,contact_authority_active_rows=0;
  std::uint64_t previous_result=0;
  json worker_histogram=json::object(),solve_histogram=json::object(),row_status_histogram=json::object();
  auto count=[](json& histogram,const char* name) {histogram[name]=histogram.value(name,std::size_t{0})+1;};
  json summary={{"schema","robotics_lab.live_preview_replay.v1"},{"status","running"},
    {"scope","Actual asynchronous coordinator, worker, current-contact guard and brake with aligned canonical reference and the same sustainedPreviewContactAuthority mapping as the production loop. Required recorded stream_armed and unmasked tick force gate/direction are supplied directly; classifier history is not reconstructed. Ideal nominal dispatch only; no backend, dynamic plant/force feedback, admittance overlay, IK, final joint safety or gripper. Worker completion uses externally stepped source clock; wall pacing governs delivery opportunities."},
    {"wall_pace_sec",wall_pace_sec},{"profile",profile}};
  summary["offline_options"]=options;
  summary["geometry_scope"]=reconstructed_geometry?
    "Explicit exogenous reconstructed hold dp/dR at CSV precision, after raw tick before preview on next tick. This is not recomputed geometry/ROI/IK, not exact original RT fold telemetry, and not a counterfactual robot safety simulation.":"No geometry folds applied; v2 retains strict fold-free input scope.";
  output<<"tick,t,mono,active,fault,status,submitted,accepted_plans,rejected,expired,plan_id,epoch,source_wire_seq,source_recv_seq,plan_age_sec,backlog_sec,cursor_rate,solve_us,step_compute_us,raw_observed_error_m,stage_raw_error_m,stage_raw_error_rad,contact_gate,contact_nx,contact_ny,contact_nz,braking,braking_stationary,contact_guard_count,worker_request_id,worker_status,worker_solve_status,worker_solve_us,worker_contact_velocity_violation_m_s,worker_working_set_recalculations,raw_vx,raw_vy,raw_vz,stage_vx,stage_vy,stage_vz,stage_wx_body,stage_wy_body,stage_wz_body,stage_ax,stage_ay,stage_az,stage_alphax_body,stage_alphay_body,stage_alphaz_body,stage_jx,stage_jy,stage_jz,stage_jwx_stand,stage_jwy_stand,stage_jwz_stand,contact_allowed_closing_m_s,stage_closing_m_s,current_contact_velocity_violation_m_s";
  for(const char* prefix:{"raw","stage","observed_raw","observed_stage"})
    for(const char* axis:{"x","y","z","qx","qy","qz","qw"})output<<','<<prefix<<'_'<<axis;
  for(const auto name:admissionNames)output<<",admission_"<<name;
  output<<",ready_not_staged,staged_identity_rejected,staged_expired,staged_sample_rejected,staged_contact_rejected,last_contact_reject_time_sec,last_contact_reject_gate,last_contact_reject_closing_m_s,last_contact_reject_allowed_m_s,last_contact_reject_nx,last_contact_reject_ny,last_contact_reject_nz,recorded_contact_stream_armed,recorded_tick_contact_gate";
  output<<",raw_observed_error_rad\n"<<std::setprecision(17);
  bool failed=false,limited=false;
  std::string line,contact_line;
  while(std::getline(input,line)) {
    const auto j=json::parse(line);
    const char* expected_schema=reconstructed_geometry?"robotics_lab.recorded_follower_input.v3":"robotics_lab.recorded_follower_input.v2";
    if(j.at("schema")!=expected_schema)throw std::runtime_error("wrong recorded replay schema/options");
    if(j.contains("booked_hold_fold")&&!reconstructed_geometry)throw std::runtime_error("geometry event requires explicit v3 mode");
    const auto tick=j.at("tick").get<std::uint64_t>();
    if(!std::getline(contact_input,contact_line))throw std::runtime_error("missing aligned contact input");
    const auto cj=json::parse(contact_line);
    if(cj.at("schema")!="robotics_lab.preview_contact_input.v2" || cj.at("tick").get<std::uint64_t>()!=tick)
      throw std::runtime_error("contact input schema/tick mismatch");
    const bool covered=cj.at("covered").get<bool>();
    const bool contact_stream_armed=cj.at("stream_armed").get<bool>();
    const double tick_contact_gate=rr::finiteNumber(cj.at("gate"));
    const auto contact_n=rr::finiteArray<3>(cj.at("normal_into_stand"));
    const Eigen::Vector3d tick_contact_normal{contact_n[0],contact_n[1],contact_n[2]};
    if(tick_contact_gate<0 || tick_contact_gate>1 ||
       (!covered&&(tick_contact_gate!=1||!tick_contact_normal.isZero(0))))
      throw std::runtime_error("invalid contact authority");
    const bool recorded_active=j.at("active").get<bool>();
    const auto contact=sustainedPreviewContactAuthority(
        recorded_active&&j.at("reference_strip_enabled").get<bool>(),contact_stream_armed,
        tick_contact_gate,-tick_contact_normal);
    const double contact_gate=contact.gate;
    const auto& contact_normal=contact.normal_into_stand;
    const double t=rr::finiteNumber(j.at("t")),mono=rr::finiteNumber(j.at("mono")),dt=rr::finiteNumber(j.at("dt"));
    if(dt<=0 || (input_rows&&(tick<=previous_tick||mono<=previous_mono||t<=previous_t)))
      throw std::runtime_error("invalid recorded clock");
    if(!live)live=std::make_unique<LivePreviewExecution>(*c,raw_config,dt_contract);
    ++input_rows;if(contact_stream_armed)++stream_armed_input_rows;
    previous_tick=tick;previous_mono=mono;previous_t=t;
    setExternalSteadyNs(static_cast<std::uint64_t>(std::llround(mono*1e9)));
    const double execution_mono=PreviewExecutionWorker::monotonicNowSec();
    if(!recorded_active) {
      follower.deactivate();live->reset("recorded_inactive");accepted.reset();previous_accepted_motion.reset();previous_recv=0;
      if(pending_fold){++discarded_folds;pending_fold.reset();}
      continue;
    }
    if(contact_stream_armed)++stream_armed_active_rows;
    if(contact_gate<1&&!contact_normal.isZero(0))++contact_authority_active_rows;
    const auto dev=rr::finiteArray<6>(j.at("reference_deviation"));
    const auto initial=rr::strip(rr::readPose(j.at("previous_emitted")),dev,j.at("reference_strip_enabled").get<bool>());
    if(!accepted)accepted=initial;
    if(j.contains("frame")) {
      const auto& b=j.at("frame");ChunkFrame frame;
      frame.wire_seq=b.at("wire_seq").get<std::uint64_t>();frame.recv_seq=b.at("recv_seq").get<std::uint64_t>();
      frame.recv_time=rr::finiteNumber(b.at("recv_time"));frame.policy_dt=rr::finiteNumber(b.at("policy_dt"));
      if(!frame.wire_seq||!frame.recv_seq||frame.recv_seq==previous_recv||frame.recv_time>mono||frame.policy_dt<=0)
        throw std::runtime_error("invalid frame identity");
      for(const auto& row:b.at("delta")) {
        const auto v=rr::finiteArray<7>(row);frame.delta.push_back({v[0],v[1],v[2],v[3],v[4],v[5]});frame.grip.push_back(v[6]);
      }
      if(frame.delta.size()<2)throw std::runtime_error("short frame");
      follower.submitDeltaFrame(frame,initial);previous_recv=frame.recv_seq;++frames;
    }
    if(!follower.active())throw std::runtime_error("active input without frame");
    const double gate=rr::finiteNumber(j.at("advance_gate")),rate=rr::finiteNumber(j.at("plan_rate_gate"));
    const auto direction=rr::finiteArray<3>(j.at("advance_direction"));
    if(gate<0||gate>1||rate<0||rate>1)throw std::runtime_error("invalid recorded gate");
    follower.setAdvanceGate(gate,{direction[0],direction[1],direction[2]});follower.setPlanRateGate(rate);
    const auto raw_before_fold=follower.tick(dt);
    json applied_fold_json=nullptr;
    if(pending_fold) {
      if(pending_fold->booked_tick>=tick)throw std::runtime_error("fold must be booked before application tick");
      if(!follower.absorbOffset(pending_fold->translation,pending_fold->rotation))throw std::runtime_error("recorded fold rejected by active raw follower");
      rr::applyReconstructedHoldFold(*live,*pending_fold,tick,transport_geometry);
      applied_fold_json={{"booked_tick",pending_fold->booked_tick},{"applied_tick",tick},
        {"translation_m",{pending_fold->translation.x(),pending_fold->translation.y(),pending_fold->translation.z()}},
        {"rotation_xyzw",{pending_fold->rotation.x(),pending_fold->rotation.y(),pending_fold->rotation.z(),pending_fold->rotation.w()}},
        {"magnitude_residual_m",pending_fold->magnitude_residual_m}};
      max_fold_residual=std::max(max_fold_residual,pending_fold->magnitude_residual_m);++applied_folds;pending_fold.reset();
    }
    const auto raw=follower.lastPose();
    const auto begin=std::chrono::steady_clock::now();
    const auto current=live->step(execution_mono,follower,*accepted,!live->initialized(),contact_gate,contact_normal);
    const double compute_us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-begin).count();
    max_step_compute_us=std::max(max_step_compute_us,compute_us);
    const auto transaction=live->transaction(current.pose,current.pose);
    if(!live->observeDispatch(transaction,current.pose,!current.fault,
        all.kinematics.ik.position_tolerance_m,all.kinematics.ik.orientation_tolerance_rad))
      throw std::runtime_error("ideal dispatch unexpectedly refused");
    if(!current.fault)*accepted=current.pose;
    const auto& tele=live->telemetry();
    double admission_gap=0;
    if(tele.accepted>previous_admissions) {
      if(last_admission_time>0)admission_gap=execution_mono-last_admission_time;
      max_admission_gap=std::max(max_admission_gap,admission_gap);
      last_admission_time=execution_mono;previous_admissions=tele.accepted;
    }
    const auto& last_result=live->lastResult();
    const auto& motion=live->sample();
    const auto raw_motion=follower.outputKinematics();
    const Eigen::Vector3d raw_velocity{raw_motion.velocity.x,raw_motion.velocity.y,raw_motion.velocity.z};
    const double closing=contact_normal.dot(motion.linear_velocity),allowed_closing=std::max(0.,contact_normal.dot(raw_velocity));
    const double contact_violation=contact_gate<1&&!contact_normal.isZero(0)?std::max(0.,closing-allowed_closing):0.;
    const bool braking=live->braking();
    const bool brake_stationary=braking&&motion.linear_velocity.isZero(0)&&motion.linear_acceleration.isZero(0)
      &&motion.angular_velocity_body.isZero(0)&&motion.angular_acceleration_body.isZero(0);
    if(current.fault)++fault_rows;else if(braking)++braking_rows;
    else if(current.active)++tracking_rows;else ++waiting_rows;
    if(brake_stationary)++braking_stationary_rows;
    if(current.active&&!current.fault) {
      max_linear_velocity=std::max(max_linear_velocity,motion.linear_velocity.cwiseAbs().maxCoeff());
      max_linear_acceleration=std::max(max_linear_acceleration,motion.linear_acceleration.cwiseAbs().maxCoeff());
      max_linear_jerk=std::max(max_linear_jerk,motion.linear_jerk.cwiseAbs().maxCoeff());
      max_angular_velocity=std::max(max_angular_velocity,motion.angular_velocity_body.norm());
      max_angular_acceleration=std::max(max_angular_acceleration,motion.angular_acceleration_body.norm());
      max_angular_jerk=std::max(max_angular_jerk,motion.angular_jerk_stand.norm());
      if(!braking)max_contact_velocity_violation=std::max(max_contact_velocity_violation,contact_violation);
    }
    count(row_status_histogram,current.reason);
    if(last_result.identity.request_id && last_result.identity.request_id!=previous_result) {
      previous_result=last_result.identity.request_id;
      count(worker_histogram,workerStatus(last_result.status));
      if(last_result.status==PreviewExecutionWorkerStatus::Solved || last_result.status==PreviewExecutionWorkerStatus::SolveRejected ||
          (last_result.status==PreviewExecutionWorkerStatus::Late&&last_result.diagnostics.solve_time_sec>0))
        count(solve_histogram,solveStatus(last_result.diagnostics.status));
      const auto vector=[](const Eigen::Vector3d& v){return json::array({v.x(),v.y(),v.z()});};
      const auto& s=last_result.initial;const auto& id=last_result.identity;
      worker_output<<json({{"schema","robotics_lab.preview_worker_observed_result.v1"},
          {"request_id",id.request_id},{"epoch",id.epoch},{"gate_revision",id.gate_revision},
          {"source_wire_seq",id.source_wire_seq},{"source_recv_seq",id.source_recv_seq},{"parent_plan_id",id.parent_plan_id},
          {"generated_at_sec",last_result.generated_at_sec},{"splice_at_sec",last_result.splice_at_sec},
          {"completed_at_sec",last_result.completed_at_sec},{"valid_until_sec",last_result.valid_until_sec},
          {"worker_status",workerStatus(last_result.status)},{"solve_status",solveStatus(last_result.diagnostics.status)},
          {"solve_time_sec",last_result.diagnostics.solve_time_sec},
          {"working_set_recalculations",last_result.diagnostics.working_set_recalculations},
          {"contact_constrained",last_result.diagnostics.contact_constrained},
          {"contact_decomposed",last_result.diagnostics.contact_decomposed},
          {"contact_coupled_fallback",last_result.diagnostics.contact_coupled_fallback},
          {"contact_constraint_rows",last_result.diagnostics.contact_constraint_rows},
          {"initial_pose",{s.pose.x,s.pose.y,s.pose.z,s.pose.rx,s.pose.ry,s.pose.rz}},
          {"initial_linear_velocity",vector(s.linear_velocity)},{"initial_linear_acceleration",vector(s.linear_acceleration)},
          {"initial_angular_velocity_body",vector(s.angular_velocity_body)},
          {"initial_angular_acceleration_body",vector(s.angular_acceleration_body)}}).dump()<<'\n';
    }
    const auto observed=rr::readPose(j.at("observed_prefilter"));
    const double raw_error=math::positionDistance(raw_before_fold,observed),stage_error=math::positionDistance(current.pose,raw);
    const double raw_rotation_error=math::log3(math::rotationFromPose(raw_before_fold).transpose()*math::rotationFromPose(observed)).norm();
    max_raw_rotation_error=std::max(max_raw_rotation_error,raw_rotation_error);
    max_raw_error=std::max(max_raw_error,raw_error);max_stage_error=std::max(max_stage_error,stage_error);
    max_backlog=std::max(max_backlog,tele.backlog_sec);max_dispatch_error=std::max(max_dispatch_error,tele.accepted_position_error_m);
    output<<tick<<','<<t<<','<<mono<<','<<current.active<<','<<current.fault<<','<<current.reason
      <<','<<tele.submitted<<','<<tele.accepted<<','<<tele.rejected<<','<<tele.expired<<','<<tele.plan_id<<','<<tele.epoch
      <<','<<tele.source_wire_seq<<','<<tele.source_recv_seq<<','<<tele.plan_age_sec<<','<<tele.backlog_sec<<','<<tele.rate
      <<','<<tele.solve_time_sec*1e6<<','<<compute_us<<','<<raw_error<<','<<stage_error
      <<','<<math::orientationDistanceRad(current.pose,raw)<<','<<contact_gate
      <<','<<contact_normal.x()<<','<<contact_normal.y()<<','<<contact_normal.z()
      <<','<<braking<<','<<brake_stationary<<','<<tele.contact_guard_count
      <<','<<last_result.identity.request_id<<','<<workerStatus(last_result.status)
      <<','<<solveStatus(last_result.diagnostics.status)<<','<<last_result.diagnostics.solve_time_sec*1e6
      <<','<<last_result.diagnostics.max_contact_velocity_violation_m_s
      <<','<<last_result.diagnostics.working_set_recalculations;
    for(const auto& v:{raw_velocity,motion.linear_velocity,motion.angular_velocity_body,motion.linear_acceleration,
        motion.angular_acceleration_body,motion.linear_jerk,motion.angular_jerk_stand})
      output<<','<<v.x()<<','<<v.y()<<','<<v.z();
    output<<','<<allowed_closing<<','<<closing<<','<<contact_violation;
    for(const auto& p:{raw,current.pose,observed,rr::readPose(j.at("observed_stage"))})rr::writePose(output,p);
    const auto& admission=live->admissionDiagnostics();
    for(const auto value:admission.result_checks)output<<','<<value;
    output<<','<<admission.ready_not_staged<<','<<admission.staged_identity_rejected
      <<','<<admission.staged_expired<<','<<admission.staged_sample_rejected<<','<<admission.staged_contact_rejected
      <<','<<admission.last_contact_reject_time_sec<<','<<admission.last_contact_reject_gate
      <<','<<admission.last_contact_reject_closing_m_s<<','<<admission.last_contact_reject_allowed_m_s
      <<','<<admission.last_contact_reject_normal.x()<<','<<admission.last_contact_reject_normal.y()<<','<<admission.last_contact_reject_normal.z();
    output<<','<<contact_stream_armed<<','<<tick_contact_gate<<','<<raw_rotation_error;
    output<<'\n';++active_rows;if(current.active)++active_output_rows;
    const bool observed_stage_valid=j.value("observed_stage_valid",true);
    if(!observed_stage_valid)++missing_stage_observations;
    auto audit=rr::liveAudit(tele);
    audit["schema"]="robotics_lab.preview_replay_audit.v1";audit["tick"]=tick;audit["t"]=t;audit["mono"]=mono;
    audit["status"]=current.reason;audit["admission_gap_sec"]=admission_gap;
    audit["time_since_admission_sec"]=last_admission_time>0?execution_mono-last_admission_time:0.;
    audit["applied_reconstructed_fold"]=applied_fold_json;audit["observed_stage_valid"]=observed_stage_valid;
    if(j.contains("recorded_geometry"))audit["recorded_geometry"]=j.at("recorded_geometry");
    audit_output<<audit.dump()<<'\n';
    if(j.contains("booked_hold_fold"))pending_fold=rr::readReconstructedHoldFold(j.at("booked_hold_fold"),tick);
    if(current.fault) {
      failed=true;summary["termination_reason"]=current.reason;summary["termination_t"]=t;
      summary["termination_tick"]=tick;
      if(previous_accepted_motion) {
        // Terminal-only diagnostic: construction/extra solve cannot affect any
        // worker delivery before the failure being measured.
        PreviewBrake probe(c->preview_execution.tracker,dt_contract);
        const auto& m=*previous_accepted_motion;
        const auto vector=[](const Eigen::Vector3d& v){return json::array({v.x(),v.y(),v.z()});};
        summary["terminal_previous_accepted_motion"]={
          {"brake_probe_status",previewBrakeStatusName(probe.start(m))},
          {"pose",{m.pose.x,m.pose.y,m.pose.z,m.pose.rx,m.pose.ry,m.pose.rz}},
          {"linear_velocity",vector(m.linear_velocity)},
          {"linear_acceleration",vector(m.linear_acceleration)},
          {"angular_velocity_body",vector(m.angular_velocity_body)},
          {"angular_acceleration_body",vector(m.angular_acceleration_body)}};
      }
      break;
    }
    previous_accepted_motion=motion;
    if(max_active_rows&&active_rows>=max_active_rows) {limited=true;break;}
    if(wall_pace_sec>0)std::this_thread::sleep_for(std::chrono::duration<double>(wall_pace_sec));
  }
  if(!active_rows)throw std::runtime_error("no active recorded rows");
  const auto& tele=live->telemetry();
  summary["status"]=failed?"terminated":limited?"bounded_sample_completed":"completed";
  summary["input_rows"]=input_rows;summary["active_rows"]=active_rows;summary["frames"]=frames;
  summary["contact_stream_armed_input_rows"]=stream_armed_input_rows;
  summary["contact_stream_armed_active_rows"]=stream_armed_active_rows;
  summary["contact_authority_active_rows"]=contact_authority_active_rows;
  summary["active_output_rows"]=active_output_rows;summary["submitted"]=tele.submitted;summary["accepted_plans"]=tele.accepted;
  summary["tracking_rows"]=tracking_rows;summary["braking_rows"]=braking_rows;
  summary["braking_stationary_rows"]=braking_stationary_rows;summary["waiting_rows"]=waiting_rows;
  summary["fault_rows"]=fault_rows;
  summary["contact_guard_count"]=tele.contact_guard_count;
  summary["reconstructed_folds_applied"]=applied_folds;summary["reconstructed_folds_discarded_on_inactive"]=discarded_folds;
  summary["reconstructed_fold_pending_at_end"]=pending_fold.has_value();summary["max_fold_magnitude_residual_m"]=max_fold_residual;
  summary["missing_stage_observation_rows"]=missing_stage_observations;
  summary["max_plan_admission_gap_sec"]=max_admission_gap;
  summary["time_since_last_plan_admission_sec"]=last_admission_time>0?previous_mono-last_admission_time:0.;
  summary["final_live_audit"]=rr::liveAudit(tele);
  summary["max_linear_velocity_axis_m_s"]=max_linear_velocity;
  summary["max_linear_acceleration_axis_m_s2"]=max_linear_acceleration;
  summary["max_linear_jerk_axis_m_s3"]=max_linear_jerk;
  summary["max_angular_velocity_norm_rad_s"]=max_angular_velocity;
  summary["max_angular_acceleration_norm_rad_s2"]=max_angular_acceleration;
  summary["max_angular_jerk_norm_rad_s3"]=max_angular_jerk;
  summary["max_tracking_contact_velocity_violation_m_s"]=max_contact_velocity_violation;
  const auto& admission=live->admissionDiagnostics();
  for(std::size_t k=0;k<admission.result_checks.size();++k)summary["result_admission"][admissionNames[k]]=admission.result_checks[k];
  summary["staged_admission"]={{"ready_not_staged",admission.ready_not_staged},
      {"identity_rejected",admission.staged_identity_rejected},{"expired",admission.staged_expired},
      {"sample_rejected",admission.staged_sample_rejected},{"contact_rejected",admission.staged_contact_rejected}};
  summary["angular_continuations_started"]=admission.angular_continuations_started;
  summary["angular_brakes_started"]=admission.angular_brakes_started;
  summary["worker_result_status_histogram"]=worker_histogram;summary["qp_status_histogram"]=solve_histogram;
  summary["row_status_histogram"]=row_status_histogram;
  summary["rejected_results"]=tele.rejected;summary["expired"]=tele.expired;summary["max_backlog_sec"]=max_backlog;
  summary["max_raw_observed_position_error_m"]=max_raw_error;summary["max_stage_raw_position_error_m"]=max_stage_error;
  summary["max_raw_observed_rotation_error_rad"]=max_raw_rotation_error;
  summary["max_step_compute_us"]=max_step_compute_us;summary["max_ideal_dispatch_error_m"]=max_dispatch_error;
  std::ofstream report(std::string(output_path)+".summary.json");report<<summary.dump(2)<<'\n';
  if(!output.good()||!report.good()||!worker_output.good()||!audit_output.good())throw std::runtime_error("output write failed");
  std::cout<<summary.dump()<<'\n';return failed?2:0;
}
}
int main(int argc,char** argv) {
  try {
    if(argc<7||argc>9)throw std::runtime_error("usage: live_preview_replay CONFIG PROFILE EVENTS.jsonl CONTACT.jsonl OUTPUT.csv WALL_PACE_SEC [MAX_ACTIVE_ROWS [OFFLINE_OPTIONS.json]]");
    const int result=run(argv[1],argv[2],argv[3],argv[4],argv[5],std::stod(argv[6]),argc>=8?std::stoull(argv[7]):0,argc==9?argv[8]:nullptr);
    setExternalSteadyNs(0);return result;
  } catch(const std::exception& e) {
    setExternalSteadyNs(0);std::cerr<<"live_preview_replay: "<<e.what()<<'\n';return 1;
  }
}
