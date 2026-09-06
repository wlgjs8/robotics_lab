#!/usr/bin/env python3
"""Export actual delta-preview consumer ticks to a strict hardware-free replay.

The selected Feather cache must originate from the complete CSV. Frames are
joined through receiver-local consumed sequence and wire sequence, never through
policy step chunk_id. Force/plan gates, measured feedback and lifecycle are
recorded exogenous inputs, not a counterfactual robot/contact simulation.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import pandas as pd

SCHEMA = 'robotics_lab.recorded_follower_input.v2'
DIAGNOSTIC_SCHEMA = 'robotics_lab.recorded_follower_input.v3'
POSE_AXES = ['x_m', 'y_m', 'z_m', 'qx', 'qy', 'qz', 'qw']
SIDES = ['left', 'right']

def required_columns():
    cols = ['loop_start_time_ns', 'filter_dt_ms', 'chunk_frame_wire_seq', 'chunk_frame_recv_seq',
            'chunk_frame_policy_dt_sec', 'chunk_frame_age_ms']
    for arm in SIDES:
        cols += [arm+'_'+k for k in ['mode', 'tcp_target_profile', 'follower_controller', 'follower_active',
            'follower_wire_seq', 'follower_recv_seq', 'follower_step', 'follower_segments', 'follower_duration_sec',
            'follower_t_in_seg_sec', 'follower_core_gate', 'follower_advance_gate', 'follower_plan_rate_gate',
            'follower_output_smd_reseeded', 'follower_stall', 'follower_solve_failures',
            'follower_reanchor_count', 'follower_warm_resume_count', 'fc_reference_strip_enabled',
            'fc_reference_reset_count', 'fc_folded', 'hold_fold_m', 'cartesian_solve_blocked_recent']]
        cols += [arm+'_follower_advance_dir_'+axis for axis in 'xyz']
        cols += [arm+'_fc_reference_dev_'+axis for axis in ['x_m', 'y_m', 'z_m', 'rx_rad', 'ry_rad', 'rz_rad']]
        for stem in ['tcp_command_stand', 'tcp_actual_stand', 'follower_prefilter_stand', 'stage_tcp_target_stand', 'follower_pf_stand']:
            cols += [arm+'_'+stem+'_'+axis for axis in POSE_AXES]
        for stem in ['follower_sample_velocity', 'follower_sample_acceleration', 'follower_target_velocity', 'follower_target_acceleration', 'follower_axis_duration_sec']:
            cols += [arm+'_'+stem+'_'+str(i) for i in range(6)]
    return cols

def diagnostic_columns():
    """Additional original CSV fields, never synthesized from fold magnitude."""
    cols = ['safety_verdict', 'projection_active', 'projection_min_headroom_class',
            'projection_min_headroom_pair', 'projection_min_headroom_m']
    for arm in SIDES:
        cols += [arm+'_'+k for k in ['fc_covered', 'fc_gate_translation', 'fc_gate_stream_armed',
                    'projection_applied_correction_deg_s', 'reach_engaged']]
        cols += [arm+'_fc_dev_'+axis for axis in ['x_m','y_m','z_m','rx_rad','ry_rad','rz_rad']]
        cols += [arm+'_fc_wrench_filt_f'+axis+'_n' for axis in 'xyz']
    return cols

def contact_columns():
    return [arm+'_'+key for arm in SIDES for key in
            ['fc_covered','fc_gate_translation','fc_gate_stream_armed',
             'fc_wrench_filt_fx_n','fc_wrench_filt_fy_n','fc_wrench_filt_fz_n']]

def reconstruct_hold_fold(row, arm):
    """Logged stage -> same-tick nominal achieved FK; not original RT dp/dR.

    Position precision is 9 significant decimals in the original CSV. Require
    an independent norm residual <=2 nm; do not derive a direction by scaling
    the logged magnitude. Rotation has no independent logged fold-angle check.
    """
    from scipy.spatial.transform import Rotation
    magnitude = float(row[arm+'_hold_fold_m'])
    if not np.isfinite(magnitude) or magnitude < 0:
        raise ValueError('invalid recorded hold-fold magnitude')
    if magnitude == 0:
        return None
    emitted = np.array(finite([row[arm+'_stage_tcp_target_stand_'+k] for k in POSE_AXES], 'fold emitted'))
    achieved = np.array(finite([row[arm+'_tcp_command_stand_'+k] for k in POSE_AXES], 'fold achieved'))
    dev = np.array(finite([row[arm+'_fc_dev_'+k] for k in ['x_m','y_m','z_m','rx_rad','ry_rad','rz_rad']], 'fold overlay'))
    for q in [emitted[3:], achieved[3:]]:
        if abs(np.linalg.norm(q)-1) > 1e-6:
            raise ValueError('recorded fold quaternion is not unit length')
    dp = achieved[:3]-dev[:3]-emitted[:3]
    residual = abs(np.linalg.norm(dp)-magnitude)
    if residual > 2e-9:
        raise ValueError(f'hold-fold magnitude reconstruction residual {residual:g} exceeds 2 nm')
    rotation = Rotation.from_quat(achieved[3:])
    if np.linalg.norm(dev[3:]) >= 1e-9:
        rotation = Rotation.from_rotvec(-dev[3:])*rotation
    dR = rotation*Rotation.from_quat(emitted[3:]).inv()
    return {'provenance': 'reconstructed_same_tick_stage_to_overlay_stripped_command_fk',
            'booked_tick': int(row['loop_start_time_ns']), 'apply_order': 'next_tick_after_raw_before_preview',
            'translation_m': dp.tolist(), 'rotation_xyzw': dR.as_quat().tolist(),
            'recorded_magnitude_m': magnitude, 'magnitude_residual_m': float(residual),
            'rotation_independently_verified': False}

def contact_event(row, arm):
    force = np.array(finite([row[arm+'_fc_wrench_filt_f'+k+'_n'] for k in 'xyz'], 'filtered deadzoned force'))
    norm = np.linalg.norm(force)
    # Match ForceGate::update, including its existing zero-vector threshold.
    normal = -force/norm if norm > 1e-9 else np.zeros(3)
    covered = bool(row[arm+'_fc_covered'])
    gate = float(row[arm+'_fc_gate_translation'])
    if not np.isfinite(gate) or not 0 <= gate <= 1:
        raise ValueError('invalid recorded contact gate')
    return {'schema':'robotics_lab.preview_contact_input.v2', 'tick':int(row['loop_start_time_ns']),
            'covered':covered, 'stream_armed':bool(row[arm+'_fc_gate_stream_armed']),
            'gate':gate if covered else 1., 'normal_into_stand':normal.tolist() if covered else [0.,0.,0.]}

def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()

def finite(values, label):
    a = np.asarray(values, dtype=float)
    if not np.isfinite(a).all():
        raise ValueError('nonfinite '+label)
    return a.tolist()

def export_frame(df, chunks, end_mono, output, profile='flow_infer_fresh', *, reconstruct_geometry=False,
                 preview_lifecycle_readback=False, include_contact=False):
    """Pure recorded-data exporter; factored for in-memory regression fixtures."""
    if not chunks:
        raise ValueError('empty chunk log')
    by_seq = {int(c['seq']): c for c in chunks}
    if len(by_seq) != len(chunks):
        raise ValueError('wire sequence is not unique within this policy run')
    missing = sorted(set(required_columns() + (diagnostic_columns() if reconstruct_geometry else []) +
                         (contact_columns() if include_contact else [])) - set(df.columns))
    if missing:
        raise ValueError('missing recorded columns: '+', '.join(missing))
    origin = float(chunks[0]['t_mono'])
    df = df.copy()
    df['mono'] = df.loop_start_time_ns.astype(np.int64)*1e-9
    if np.any(np.diff(df.mono) <= 0):
        raise ValueError('servo monotonic timestamps must increase')
    # Retain an actual preceding row for previous-sent FK; do not synthesize it.
    indices = np.flatnonzero((df.mono >= origin) & (df.mono <= end_mono))
    if not len(indices) or indices[0] == 0:
        raise ValueError('recording lacks run ticks or preceding sent-pose row')
    df = df.iloc[indices[0]-1:indices[-1]+1].reset_index(drop=True)
    df['t'] = df.mono-origin
    output.mkdir(parents=True, exist_ok=True)
    schema = DIAGNOSTIC_SCHEMA if reconstruct_geometry else SCHEMA
    meta = {'schema': schema, 'origin_mono': origin, 'end_mono': end_mono, 'profile': profile,
        'pose_order': 'x,y,z,qx,qy,qz,qw',
        'reference': 'previous-row tcp_command_stand, stripped using current-tick fc_reference deviation/eligibility',
        'force_input': 'exact pre-overlay follower_advance_gate/direction and follower_plan_rate_gate from same tick',
        'lifecycle': 'reset on recorded non-TcpPoseTarget/inactive; Init joint motion is recorded feedback only',
        'feedback': 'measured poses, gates, frame timing and mode transitions held exogenous in all variants',
        'unsupported': 'active reanchor, warm-resume, force/hold fold and IK-refusal changes are rejected, never silently omitted',
        'arms': {}}
    if reconstruct_geometry:
        meta['unsupported'] = 'Active force folds, reanchors, warm resumes and IK-refusal changes remain rejected.'
        meta['geometry_scope'] = ('Reconstructed same-row stage to overlay-stripped commanded FK hold folds, '
            'booked for next active tick after raw tick and before preview. Independent magnitude residual <=2 nm; '
            'rotation has no independent original fold-angle check. Original RT dp/dR, per-row geometry correction, '
            'and coordinator worker schedule are unavailable. Exogenous recorded transforms are not counterfactual '
            'collision/ROI safety or hardware simulation. Zero-translation rotation-only folds cannot be identified '
            'from hold_fold_m alone; raw position AND orientation replay residuals must be checked. Full cold-start prefix is retained.')
    if preview_lifecycle_readback:
        if profile != 'flow_infer_preview':
            raise ValueError('preview lifecycle readback requires flow_infer_preview profile')
        meta['missing_stage_observation'] = ('Only preview waiting/braking/reset rows with no stage use current '
            'overlay-stripped command FK as a display-only placeholder; observed_stage_valid=false distinguishes '
            'them. This observation never enters follower or live coordinator control. No derivative/state seed is invented.')
    rows = df.to_dict('records')
    all_consumer_frames = []
    for arm in SIDES:
        previous_recv = None
        active_ticks = 0
        frame_count = 0
        cold_count = 0
        emitted_seqs = []
        durations = []
        fold_residuals = []
        contact_rows = []
        def pose(row, stem):
            return finite([row[f'{arm}_{stem}_{axis}'] for axis in POSE_AXES], arm+' '+stem)
        with (output/(arm+'.events.jsonl')).open('w') as f:
            for i in range(1, len(rows)):
                row = rows[i]; prev = rows[i-1]
                active = row[arm+'_mode'] == 'TcpPoseTarget' and row[arm+'_follower_active'] == 1
                event = {'schema': schema, 'tick': int(row['loop_start_time_ns']), 't': row['t'], 'mono': row['mono'],
                         'dt': float(row['filter_dt_ms'])*.001, 'active': bool(active), 'recorded_mode': row[arm+'_mode']}
                if active:
                    if row[arm+'_tcp_target_profile'] != profile or row[arm+'_follower_controller'] != 'delta_preview':
                        raise ValueError('active tick has a different profile/controller')
                    for k in ['fc_folded', 'cartesian_solve_blocked_recent'] + ([] if reconstruct_geometry else ['hold_fold_m']):
                        if row[arm+'_'+k] != 0:
                            raise ValueError(f'unsupported active {k} at {row["t"]}')
                    for k in ['follower_reanchor_count', 'follower_warm_resume_count']:
                        if row[arm+'_'+k] > prev[arm+'_'+k]:
                            raise ValueError(f'unsupported active {k} increment at {row["t"]}')
                    # Use the selected, captured strip state even when coverage is
                    # false. This distinction matters at tare/Init boundaries.
                    stage_values = [row[f'{arm}_stage_tcp_target_stand_{axis}'] for axis in POSE_AXES]
                    stage_valid = bool(np.isfinite(np.array(stage_values, dtype=float)).all())
                    if not stage_valid and preview_lifecycle_readback:
                        if row.get(arm+'_preview_execution_status') not in ['waiting', 'braking', 'reset']:
                            raise ValueError('missing stage observation outside explicit preview wait/reset')
                        from scipy.spatial.transform import Rotation
                        stage_values = np.array(pose(row, 'tcp_command_stand'))
                        dev = np.array(finite([row[arm+'_fc_dev_'+k] for k in ['x_m','y_m','z_m','rx_rad','ry_rad','rz_rad']], 'missing stage overlay'))
                        stage_values[:3] -= dev[:3]
                        if np.linalg.norm(dev[3:]) >= 1e-9:
                            stage_values[3:] = (Rotation.from_rotvec(-dev[3:])*Rotation.from_quat(stage_values[3:])).as_quat()
                        event['observed_stage_valid'] = False
                        event['observed_stage_source'] = 'display_only_current_nominal_command_no_preview_stage'
                    event.update(previous_emitted=pose(prev, 'tcp_command_stand'), actual=pose(row, 'tcp_actual_stand'),
                        reference_strip_enabled=bool(row[arm+'_fc_reference_strip_enabled']),
                        reference_deviation=finite([row[arm+'_fc_reference_dev_'+k] for k in ['x_m','y_m','z_m','rx_rad','ry_rad','rz_rad']], 'reference deviation'),
                        advance_gate=float(row[arm+'_follower_advance_gate']), plan_rate_gate=float(row[arm+'_follower_plan_rate_gate']),
                        advance_direction=finite([row[arm+'_follower_advance_dir_'+k] for k in 'xyz'], 'advance direction'),
                        observed_prefilter=pose(row, 'follower_prefilter_stand'), observed_stage=finite(stage_values, 'observed stage'))
                    if reconstruct_geometry:
                        fold = reconstruct_hold_fold(row, arm)
                        if fold:
                            event['booked_hold_fold'] = fold
                            fold_residuals.append(fold['magnitude_residual_m'])
                        event['recorded_geometry'] = {
                            'projection_active': bool(row['projection_active']),
                            'applied_correction_deg_s': float(row[arm+'_projection_applied_correction_deg_s']),
                            'reach_engaged': bool(row[arm+'_reach_engaged']),
                            'verdict': str(row['safety_verdict']),
                            'tightest_class': str(row['projection_min_headroom_class']),
                            'tightest_pair': str(row['projection_min_headroom_pair']),
                            'headroom_m': float(row['projection_min_headroom_m']) if pd.notna(row['projection_min_headroom_m']) else None}
                    recv = int(row[arm+'_follower_recv_seq']); wire = int(row[arm+'_follower_wire_seq'])
                    if recv != previous_recv:
                        # A newer cache can exist before a legacy segment consumes
                        # it. This export deliberately supports immediate fresh
                        # delivery only when the captured cache matches consumption.
                        if recv != int(row['chunk_frame_recv_seq']) or wire != int(row['chunk_frame_wire_seq']):
                            raise ValueError(f'consumed/cache identity differs at {row["t"]}; need retained receive history')
                        if wire not in by_seq:
                            raise ValueError(f'consumed wire {wire} missing from producer chunk log')
                        c = by_seq[wire]; cm = c['chunk_metadata']
                        if c['t_mono'] > row['mono']:
                            raise ValueError('consumed chunk predates its publication')
                        if cm['tcp_target_profile'] != profile or cm['activation_mode'] != 'ready_event' or not cm['proprio']['valid']:
                            raise ValueError('chunk is not a valid selected fresh-policy activation')
                        if abs(c['policy_dt_sec']-row['chunk_frame_policy_dt_sec']) > 1e-9:
                            raise ValueError('policy and receiver dt disagree')
                        delta = np.asarray(c[arm+'_delta'], dtype=float)
                        absolute = np.asarray(c[arm], dtype=float)
                        if delta.ndim != 2 or delta.shape[1] != 7 or absolute.shape != (len(delta), 8):
                            raise ValueError('recorded frame shape mismatch')
                        if not np.array_equal(delta[:, 6], absolute[:, 7]):
                            raise ValueError('delta and pose gripper rows disagree')
                        if not np.isfinite(delta).all():
                            raise ValueError('nonfinite recorded delta')
                        event['frame'] = {'wire_seq': wire, 'recv_seq': recv,
                            'recv_time': row['mono']-float(row['chunk_frame_age_ms'])*.001,
                            'policy_dt': float(c['policy_dt_sec']), 'delta': delta.tolist()}
                        all_consumer_frames.append({'arm': arm, 't': row['t'], 'mono': row['mono'], 'wire_seq': wire, 'recv_seq': recv,
                            'activation_mono': c['t_mono'], 'generation': cm['generation'], 'cold': previous_recv is None,
                            'activation_step': cm['activation_step_seq'], 'observation_step': cm['observation_step_seq'], 'source_start_index': cm['source_start_index']})
                        cold_count += previous_recv is None
                        previous_recv = recv; frame_count += 1; emitted_seqs.append(wire)
                    active_ticks += 1
                    durations.append(float(row['filter_dt_ms'])*.001)
                else:
                    previous_recv = None
                if reconstruct_geometry or include_contact:
                    contact_rows.append(contact_event(row, arm))
                f.write(json.dumps(event, allow_nan=False, separators=(',', ':'))+'\n')
        meta['arms'][arm] = {'rows': len(rows)-1, 'active_ticks': active_ticks, 'active_filter_dt_sum_sec': sum(durations),
            'consumed_frames': frame_count, 'cold_starts': cold_count, 'first_wire_seq': emitted_seqs[0] if emitted_seqs else None, 'last_wire_seq': emitted_seqs[-1] if emitted_seqs else None,
            'missing_producer_wire_sequences': sorted(set(by_seq)-set(emitted_seqs))}
        if reconstruct_geometry:
            meta['arms'][arm]['reconstructed_hold_folds'] = len(fold_residuals)
            meta['arms'][arm]['max_fold_magnitude_residual_m'] = max(fold_residuals, default=0.)
        if reconstruct_geometry or include_contact:
            with (output/(arm+'.contact.jsonl')).open('w') as f:
                for event in contact_rows:
                    f.write(json.dumps(event, allow_nan=False, separators=(',', ':'))+'\n')
    pd.DataFrame(all_consumer_frames).to_csv(output/'consumed_frames.csv', index=False)
    df.iloc[1:].reset_index(drop=True).to_feather(output/'observed.feather')
    (output/'manifest.json').write_text(json.dumps(meta, indent=2)+'\n')
    return meta

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('servo_feather', type=Path)
    p.add_argument('chunks', type=Path)
    p.add_argument('steps', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--profile', default='flow_infer_fresh')
    p.add_argument('--tail-sec', type=float, default=.5)
    p.add_argument('--reconstruct-geometry', action='store_true', help='Explicit approximate v3 geometry replay; strict v2 remains the default')
    p.add_argument('--preview-lifecycle-readback', action='store_true', help='Label missing preview wait-stage observations and retain full raw cold-start prefix')
    p.add_argument('--contact-companion', action='store_true', help='Export aligned unmasked contact v2 alongside strict fold-free v2 events')
    p.add_argument('--extra-feather', type=Path, help='Same-row complementary original CSV columns; clocks must match exactly')
    args = p.parse_args()
    chunks = [json.loads(line) for line in args.chunks.open() if line.strip()]
    steps = [json.loads(line) for line in args.steps.open() if line.strip()]
    df = pd.read_feather(args.servo_feather)
    sources = [args.servo_feather, args.chunks, args.steps]
    if args.extra_feather:
        extra = pd.read_feather(args.extra_feather)
        clock = 'loop_start_time_ns' if 'loop_start_time_ns' in df else 'mono'
        if len(extra) != len(df) or clock not in extra or not np.array_equal(df[clock], extra[clock]):
            raise ValueError('extra Feather row clock mismatch')
        for c in extra:
            if c not in df:
                df[c] = extra[c]
        sources.append(args.extra_feather)
    keep = required_columns() + (diagnostic_columns() if args.reconstruct_geometry else [])
    if args.contact_companion:
        keep += contact_columns()
    if args.preview_lifecycle_readback:
        for arm in SIDES:
            keep += [arm+'_preview_execution_status']
            keep += [arm+'_fc_dev_'+axis for axis in ['x_m','y_m','z_m','rx_rad','ry_rad','rz_rad']]
    df = df[[c for c in dict.fromkeys(keep) if c in df]]
    meta = export_frame(df, chunks, float(steps[-1]['t_mono'])+args.tail_sec, args.output, args.profile,
                        reconstruct_geometry=args.reconstruct_geometry, preview_lifecycle_readback=args.preview_lifecycle_readback,
                        include_contact=args.contact_companion)
    meta['sources'] = {str(path.resolve()): sha256(path) for path in sources}
    (args.output/'manifest.json').write_text(json.dumps(meta, indent=2)+'\n')
    print(json.dumps(meta, indent=2))

if __name__ == '__main__':
    main()
