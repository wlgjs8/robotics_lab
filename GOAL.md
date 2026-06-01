Read AGENTS.md, REVIEW.md, docs/runbooks/rbpodo_500hz_acceptance.md, docs/runbooks/rbpodo_controller_sim_circle.md, docs/runbooks/rbpodo_measurement_reliability.md, rb_servo_server/src/control/dual_arm_servo_loop.cpp, rb_servo_server/src/control/arm_worker.cpp, rb_servo_server/src/control/servo_dispatcher.cpp, rb_servo_server/src/control/cartesian_servo_controller.cpp, rb_servo_server/src/control/cartesian_trajectory_planner.cpp, rb_servo_server/src/robot/rbpodo_backend.cpp, rb_servo_server/config/dual_real_rbpodo_circle_15cm4s_500hz.example.yaml, scripts/rbpodo_500hz_acceptance.py, scripts/rbpodo_circle_tracking_benchmark.py, scripts/run_rbpodo_circle_ablation.py, scripts/generate_rbpodo_500hz_report.py, scripts/generate_circle_benchmark_report.py, and the latest rbpodo artifacts first.

Implement ONLY ACKON500-GENE-GOAL-01:
Aggressively optimize rbpodo controller-simulation 500Hz ACK-ON circle tracking for the GENE-style 15cm/4s benchmark, with strict non-cheating measurement semantics.
Do not optimize the report; optimize the real measured behavior. If the requested goal is impossible under rbpodo SDK ACK-ON semantics, prove the limit with artifacts instead of weakening the benchmark.

Context:
- The project primary real-controller backend is rbpodo.
- The robot controllers are real Rainbow controller boxes in pgmode simulation.
- Physical robot motion is not expected and must not occur.
- Previous 500Hz direct rbpodo no-op evidence showed that 500Hz ServoJ was possible in pgmode simulation no-op conditions:
  - 5000/5000 ServoJ success over 10s.
  - Loop p99 around 2ms.
  - Send duration max within 2ms tick.
- However, this does not prove full dual-arm rb_servo_server 500Hz circle tracking.
- Previous ACK-off 500Hz circle results are promising but socket_send_only and must not be treated as ACK-ON.
- Previous 100Hz ACK-on tuned result achieved about 4mm-class RMS in controller-reference tracking under some parameter combinations.
- The goal is now stricter:
  ACK-ON semantics + 500Hz target generation + 15cm/4s circle + RMS <= 3mm + effective latency <= 5ms.

Hard fixed constraints:
1. Backend:
   - rbpodo only.
2. Controller mode:
   - Rainbow controller pgmode simulation only.
   - operation_mode must be simulation.
   - physical_motion_expected=false.
3. Motion profile:
   - diameter_m = 0.15.
   - period_sec = 4.0.
   - repeat >= 5 for official pass.
   - tracking_source = tcp_ref_stand.
4. Rate:
   - servo.rate_hz = 500.
   - servo_t1_sec = 0.002.
   - target generation / server-side trajectory update should be 500Hz.
5. ACK semantics:
   - ACK-ON is mandatory.
   - Do not use disable_waiting_ack=true for the official pass.
   - Do not report socket_send_only as success.
   - Valid ACK-ON semantics are:
     a) synchronous controller_ack_observed, or
     b) async sdk_worker_ack_observed where every sent command returns controller ACK in the worker.
   - If async worker is used, servo loop may not block on ACK, but ACK observation must still be recorded per sent command.
6. Physical real:
   - Do not enable physical real motion.
   - Do not use operation_mode=real.
   - Do not use RB_ALLOW_REAL_CARTESIAN as a shortcut.
7. Safety:
   - fault_latched must remain false.
   - physical_motion_detected must remain false.
   - diagnostics_suspect may remain a caveat for controller-simulation experiments but must be reported.
8. Measurement honesty:
   - Do not change the benchmark target to make the score easier.
   - Do not change tracking source to desired/commanded pose.
   - Do not reduce repeat count for pass.
   - Do not filter out bad samples unless explicitly reported as a separate diagnostic.
   - Do not hide phase advance as latency improvement.

Pass criteria:
- profile: gene_15cm_4s.
- repeat >= 5.
- tracking_source_used == tcp_ref_stand.
- servo_rate_hz == 500.
- servo_t1_sec == 0.002.
- acceptance_semantics in:
  - controller_ack_observed
  - sdk_worker_ack_observed
- socket_send_only_count == 0 for the official pass.
- controller_ack_observed_count or async_worker_acked_count >= 0.98 * commands_sent_total.
- effective_command_rate_hz >= 490.
- fault_latched == false.
- physical_motion_detected == false.
- cartesian_unavailable_count == 0.
- feedback_saturation_count / command_count <= 0.01.
- RMS tracking error <= 0.003 m.
- p95 tracking error <= 0.006 m.
- fit_center_error_m <= 0.003 m.
- radius_gain in [0.98, 1.02].
- p95_orientation_drift_rad <= 0.02 rad.
- effective_phase_latency_ms_abs <= 5.0.
- state_age_us p95 <= 5000.
- deadline_miss_count == 0 or explicitly justified and below a strict threshold.
- measurement_reliability_level must not be unreliable.
- Report must state that this is controller-reference lower bound, not physical real TCP tracking.

Latency rules:
- The report must include at least three latency-related fields:
  1. uncompensated_estimated_latency_ms
  2. phase_advance_sec, if used
  3. effective_phase_latency_ms after compensation
- If phase advance is used, success may claim "effective tracking latency <= 5ms", but the report must not claim the controller/system physical latency is <=5ms unless uncompensated_estimated_latency_ms also satisfies the threshold.
- Phase advance is allowed as a control technique, but it must be visible.

Allowed modifications:
You may modify:
- rb_servo_server controller-simulation configs.
- rbpodo async ACK worker / supervision architecture.
- server-side circle tracking / TcpCircleTrack implementation.
- benchmark and ablation scripts.
- cartesian servo gains, feedforward, dead-time compensation, t2/alpha, speed_bar, max_twist limits.
- state publisher telemetry.
- report generation.
- local config generation helpers.
- C++ tests and Python tests.
- docs/runbooks.

You may NOT:
- Modify physical real gates to pass.
- Disable safety checks globally.
- Change the definition of desired circle.
- Use ACK-off for official pass.
- Treat q_actual as tracking source in pgmode simulation.
- Mark diagnostics_suspect as healthy.
- Delete evidence of failed attempts.

Preferred technical direction:
1. First, implement or complete async SDK ACK worker if synchronous ACK-on cannot sustain 500Hz:
   - servo loop should not block on ACK.
   - per-arm worker sends ServoJ and waits for ACK in worker thread.
   - ACK-observed commands are counted.
   - queue policy must be explicit.
   - latest-wins is allowed only if overwritten commands are counted and command effective rate remains >=490Hz.
2. If async SDK worker cannot achieve ACK-observed 500Hz because rbpodo SDK blocks too long:
   - report this as an architectural limit.
   - Do not silently switch to ACK-off.
3. Implement server-side circle tracking if Python external loop latency prevents reaching target:
   - Add or complete TcpCircleTrack / CartesianCircleTrack.
   - Server generates desired circle at 500Hz.
   - Server computes feedback from latest reference state at 500Hz.
   - Python benchmark becomes recorder/orchestrator only.
4. Tune parameters in a structured way:
   - Kp_pos, Kp_ori separated.
   - phase_advance_sec.
   - speed_bar.
   - servo_t2_sec.
   - servo_alpha.
   - max_twist_linear_m_s.
   - max_twist_angular_rad_s.
   - state_pub_rate_hz for telemetry, not necessarily 500Hz.
5. Prioritize these candidate regions:
   - Kp_pos around 0.3 to 0.8.
   - Kp_ori around 0.0 to 0.3 initially.
   - speed_bar around 0.2 to 0.5.
   - servo_t2_sec around 0.05 to 0.08.
   - servo_alpha around 0.8, but include 0.5 comparisons.
   - phase_advance around 0.02 to 0.06 seconds.
   - max_twist_linear_m_s around 0.15 to 0.20 before trying 0.25.
6. Keep 100Hz ACK-on best baseline for comparison.

Required deliverables:
1. Code:
   - 500Hz ACK-ON controller-simulation profile.
   - Official ACK-ON 500Hz GENE-style benchmark matrix.
   - If needed, server-side TcpCircleTrack / CartesianCircleTrack implementation.
   - If needed, async SDK ACK worker with per-command ACK telemetry.
   - Robust pass/fail report generator.
2. Configs:
   - configs/rbpodo_circle_ablation/ackon500_gene_goal.yaml
   - config must be controller-simulation only.
3. Scripts:
   - A one-command runner:
     tools/rbpodo_ackon500_gene_goal.sh
   - It should refuse to run unless all required env/confirmation flags are present.
   - It should not set dangerous env silently unless --with-required-env is explicitly passed.
4. Artifacts:
   - summary.json per run.
   - ablation_summary.csv.
   - gene_goal_report.md.
   - state_stream.jsonl.
   - command_packets.jsonl.
   - async_ack_telemetry.jsonl if async worker used.
   - timing_report.md.
   - error_decomposition_report.md.
5. Report:
   - best official candidate.
   - whether pass criteria were met.
   - if not, top limiting factor:
     ACK latency, command drop, state age, orientation drift, feedback saturation, center drift, q_ref supervision, or SDK limitation.
   - comparison against 100Hz ACK-on best and 500Hz ACK-off best.
   - caveats:
     controller-reference lower bound,
     diagnostics_suspect,
     not physical real,
     no IL data recommendation if stress.

Required tests:
- python3 -m compileall -q scripts
- bash -n tools/rbpodo_ackon500_gene_goal.sh
- python3 scripts/rbpodo_circle_tracking_benchmark.py --help
- python3 scripts/run_rbpodo_circle_ablation.py --help
- python3 scripts/generate_rbpodo_500hz_report.py --help
- PYTHONPATH=scripts python3 -m unittest discover scripts -p 'test_*.py'
- python3 -m unittest discover rb_gui/tests
- PYTHONPATH=policy_runner python3 -m unittest discover policy_runner/tests
- PYTHONPATH=rb_simulator/src python3 -m unittest discover rb_simulator/tests
- If C++ deps are available:
  rb_servo_server CTest.

Experimental run plan:
1. Preflight:
   - state parity must be at least suspect_but_consistent.
   - raw diagnostics report must exist.
   - diagnostics_suspect caveat remains explicit.
2. ACK-on 500Hz no-op:
   - pass before circle.
3. ACK-on 500Hz safe_5cm_10s:
   - pass before 15cm.
4. ACK-on 500Hz circle_15cm_16s:
   - pass before 4s.
5. ACK-on 500Hz gene_15cm_4s:
   - run official matrix repeat>=5.
6. If fail:
   - do not hide failure.
   - produce limiting-factor report.
   - propose next architecture.

Final response format:
1. Summary
2. Whether official pass criteria were met
3. Best candidate parameters
4. Evidence table
5. If failed, top 3 blockers
6. ACK semantics confirmation
7. Latency interpretation
8. Safety caveats
9. Tests run
10. Test results
11. Next recommended actions