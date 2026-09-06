#!/usr/bin/env python3
"""Compare recorded-consumer replays, without simulating new plant feedback.

CSV inputs are produced by delta_follower_replay --recorded. Fixed recorded
gates, actual poses, arrival times and cold starts are identical in all variants.
The leash calculation is a one-pass diagnostic, never fed back into the replay.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.signal import welch
from scipy.spatial.transform import Rotation, Slerp

WINDOWS = [(0.8, 18.3), (23.1, 90.7)]
BANDS = [(0.15, 0.5), (0.5, 3), (3, 8), (8, 20)]
POSE = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']

def live_motion_bands(d):
    """Nominal motion energy, including intended task motion; not vibration alone."""
    t=d.t.to_numpy(float)
    if len(t)<3 or t[-1]-t[0]<4 or np.max(np.diff(t))>.003:
        return {'available':False,'reason':'Need at least four contiguous seconds on common active ticks'}
    fs=500.;grid=np.arange(t[0],t[-1],1/fs);p=pose(d,'stage')
    xyz=np.column_stack([np.interp(grid,t,p[:,k]) for k in range(3)])
    rot=Slerp(t,Rotation.from_quat(p[:,3:]))(grid)
    omega=(rot[1:]*rot[:-1].inv()).as_rotvec()*fs
    nperseg=min(4096,len(omega));overlap=nperseg//2
    frequency,linear=welch(xyz,fs=fs,nperseg=nperseg,noverlap=overlap,detrend='linear',axis=0)
    _,angular=welch(omega,fs=fs,nperseg=nperseg,noverlap=overlap,detrend='linear',axis=0)
    integrate=getattr(np,'trapezoid',np.trapz)
    out={'available':True,'fs_hz':fs,'nperseg':nperseg,'interpretation':live_motion_bands.__doc__,'bands':{}}
    for low,high in [(0.5,3),(3,8),(8,20)]:
        take=(frequency>=low)&(frequency<=high);f=frequency[take]
        out['bands'][f'{low:g}-{high:g}']={
            'linear_position_rms_mm':float(np.sqrt(integrate(linear[take].sum(axis=1),f))*1000),
            'angular_speed_rms_deg_s':float(np.rad2deg(np.sqrt(integrate(angular[take].sum(axis=1),f))))}
    return out

def live_preview_comparison(directory, variants, windows):
    """Actual coordinator replay diagnostics; retain every variant's stop time.

    Frames/force/gates/folds are fixed recorded inputs. Dispatch is ideal nominal
    acceptance. This cannot predict plant feedback, changing geometry, gripper
    timing or task success. Compare objectives only on their common tick prefix,
    while reporting truncated/faulted coverage separately.
    """
    result={'scope':live_preview_comparison.__doc__, 'windows_sec':windows, 'arms':{}}
    for arm in ['left','right']:
        paths={v:directory/f'{arm}_{v}.csv' for v in variants}
        existing={v:p for v,p in paths.items() if p.exists()}
        if not existing:
            continue
        traces={v:pd.read_csv(p) for v,p in existing.items()}
        common=set.intersection(*(set(d.tick.to_numpy()) for d in traces.values()))
        if not common:
            raise ValueError(f'{arm}: no common replay ticks')
        common=np.array(sorted(common),dtype=np.uint64)
        baseline=traces[next(iter(existing))].set_index('tick').loc[common]
        hashes={}
        for suffix in ['events.jsonl','contact.jsonl']:
            p=directory/f'{arm}.{suffix}'
            if p.exists():hashes[p.name]=hashlib.sha256(p.read_bytes()).hexdigest()
        arm_result={'fixed_input_sha256':hashes,'common_ticks':len(common),'variants':{}}
        for name,d in traces.items():
            summary_path=Path(str(existing[name])+'.summary.json')
            native=json.loads(summary_path.read_text()) if summary_path.exists() else {}
            out={'rows':len(d),'first_t':float(d.t.iloc[0]),'last_t':float(d.t.iloc[-1]),
                 'native_summary':native, 'row_status_counts':d.status.value_counts().to_dict(),
                 'status_transitions':[], 'worker_failures':[], 'windows':{}}
            starts=np.r_[True,d.status.to_numpy()[1:]!=d.status.to_numpy()[:-1]]
            for row in d.loc[starts,['tick','t','status','plan_id','accepted_plans','rejected','expired','backlog_sec']].to_dict('records'):
                out['status_transitions'].append(row)
            worker_path=Path(str(existing[name])+'.worker.jsonl')
            if worker_path.exists():
                origin=float((d.mono-d.t).iloc[0])
                for line in worker_path.open():
                    w=json.loads(line)
                    if w['worker_status']!='solved' or w['solve_status']!='solved':
                        w['generated_t']=w['generated_at_sec']-origin
                        w['completed_t']=w['completed_at_sec']-origin
                        w['splice_t']=w['splice_at_sec']-origin
                        out['worker_failures'].append(w)
            admit=d.accepted_plans.diff().fillna(0).gt(0)
            out['admission_gap_sec']=stats(np.diff(d.loc[admit,'t'])) if admit.sum()>1 else None
            out['time_since_last_plan_admission_at_end_sec']=float(d.t.iloc[-1]-d.loc[admit,'t'].iloc[-1]) if admit.any() else None
            shared=d.set_index('tick').loc[common]
            pe,ae=distances(pose(shared,'raw'),pose(baseline,'raw'))
            out['raw_common_prefix_vs_first_variant']={'position_mm':stats(pe*1000),'angle_deg':stats(np.rad2deg(ae))}
            active=shared.active.astype(bool)&~shared.fault.astype(bool)
            take_windows=windows or [(float(shared.t.min()),float(shared.t.max())+.002)]
            for start,end in take_windows:
                take=(shared.t>=start)&(shared.t<end)&active
                cut=shared.loc[take]
                if not len(cut):
                    out['windows'][f'{start:g}-{end:g}']={'ticks':0};continue
                pe,ae=distances(pose(cut,'stage'),pose(cut,'raw'))
                metrics={'ticks':len(cut),'raw_to_stage_position_mm':stats(pe*1000),
                    'raw_to_stage_angle_deg':stats(np.rad2deg(ae)),
                    'raw_to_stage_position_rms_mm':float(np.sqrt(np.mean(pe**2))*1000),
                    'raw_to_stage_angle_rms_deg':float(np.rad2deg(np.sqrt(np.mean(ae**2)))),
                    'backlog_ms':stats(cut.backlog_sec*1000)}
                metrics['nominal_motion_bands']=live_motion_bands(cut)
                for label,columns in [
                    ('linear_speed_m_s',['stage_vx','stage_vy','stage_vz']),
                    ('angular_speed_rad_s',['stage_wx_body','stage_wy_body','stage_wz_body']),
                    ('linear_jerk_m_s3',['stage_jx','stage_jy','stage_jz']),
                    ('angular_jerk_rad_s3',['stage_jwx_stand','stage_jwy_stand','stage_jwz_stand'])]:
                    if set(columns)<=set(cut):
                        v=np.linalg.norm(cut[columns].to_numpy(),axis=1)
                        metrics[label]=dict(stats(v),rms=float(np.sqrt(np.mean(v*v))))
                out['windows'][f'{start:g}-{end:g}']=metrics
            arm_result['variants'][name]=out
        result['arms'][arm]=arm_result
    if not result['arms']:raise ValueError('no named live replay variants found')
    target=directory/'live_comparison_metrics.json'
    target.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(target)

def pose(d, prefix):
    return d[[prefix+'_'+x for x in POSE]].to_numpy(float)

def distances(a, b):
    return (np.linalg.norm(a[:, :3]-b[:, :3], axis=1),
            (Rotation.from_quat(a[:, 3:]).inv()*Rotation.from_quat(b[:, 3:])).magnitude())

def stats(a):
    a = np.asarray(a)
    return dict(zip(['median', 'p95', 'p99', 'max'], map(float, np.percentile(a, [50, 95, 99, 100]))))

def normal_mask(t):
    return np.logical_or.reduce([(t >= a) & (t < b) for a, b in WINDOWS])

def leash(pos_error, angle_error):
    gp = np.clip(1-(pos_error-.010)/(.050-.010), 0, 1)
    ga = np.clip(1-(angle_error-.0349)/(.10-.0349), 0, 1)
    return np.maximum(.25, np.minimum(gp, ga))

def movement_bands(t, p):
    """500 Hz grids; separate normal windows, Hann Welch 16s, 50% overlap.

    Positions use stand XYZ. Angular velocities use Log(R[t+dt]R[t]^T)/dt
    in stand coordinates. Spectral angular displacement is integrated from
    angular velocity PSD only inside each nonzero band (not Euler angles).
    Average PSDs by Welch window count, never concatenate across Init.
    """
    fs = 500.; nperseg = 8000
    linear = []; angular = []; weights = []
    for start, end in WINDOWS:
        grid = np.arange(start, end, 1/fs)
        xyz = np.column_stack([np.interp(grid, t, p[:, k]) for k in range(3)])
        rot = Slerp(t, Rotation.from_quat(p[:, 3:]))(grid)
        omega = (rot[1:]*rot[:-1].inv()).as_rotvec()*fs
        f, ps = welch(xyz, fs=fs, nperseg=nperseg, noverlap=nperseg//2,
                      detrend='linear', axis=0)
        _, ws = welch(omega, fs=fs, nperseg=nperseg, noverlap=nperseg//2,
                      detrend='linear', axis=0)
        linear.append(ps.sum(axis=1)); angular.append(ws.sum(axis=1))
        weights.append(1+(len(omega)-nperseg)//(nperseg//2))
    ps = np.average(linear, axis=0, weights=weights)
    ws = np.average(angular, axis=0, weights=weights)
    out = {}
    for a, b in BANDS:
        take = (f >= a) & (f <= b); freq = f[take]
        integrate = getattr(np, 'trapezoid', np.trapz)
        out[f'{a:g}-{b:g}'] = {
            'linear_position_rms_mm': float(np.sqrt(integrate(ps[take], freq))*1000),
            'linear_speed_rms_mm_s': float(np.sqrt(integrate(ps[take]*(2*np.pi*freq)**2, freq))*1000),
            'angular_position_spectral_rms_deg': float(np.rad2deg(np.sqrt(integrate(ws[take]/(2*np.pi*freq)**2, freq)))),
            'angular_speed_rms_deg_s': float(np.rad2deg(np.sqrt(integrate(ws[take], freq))))}
    return out

def export_native(d, path):
    # Root's native IK reader expects [x,y,z,qw,qx,qy,qz].
    keep = d[(d.t >= 22.1955) & (d.t <= 90.884)]
    with path.open('w') as f:
        for row in keep.itertuples(index=False):
            json.dump({'t': row.t, 'stage': [getattr(row, 'stage_'+k) for k in ['x','y','z','qw','qx','qy','qz']]}, f, allow_nan=False)
            f.write('\n')

def candidate_summary(d, baseline, observed, arm, directory, name):
    if not np.array_equal(d.tick, baseline.tick):
        raise ValueError('candidate and baseline active ticks differ')
    m = normal_mask(d.t.to_numpy()); n = int(m.sum())
    p = {k: pose(d, k) for k in ['raw', 'stage', 'reference', 'actual']}
    b = {k: pose(baseline, k) for k in ['raw', 'stage']}
    out = {'normal_ticks': n, 'normal_seconds': float(observed.filter_dt_ms.to_numpy()[m].sum()*.001)}
    applied_change = d.plan_gate.to_numpy()-baseline.plan_gate.to_numpy()
    out['applied_plan_gate'] = {'minimum': float(d.plan_gate.to_numpy()[m].min()),
        'difference_gt_0p01_ticks': int((np.abs(applied_change[m]) > .01).sum()),
        'difference_gt_0p01_pct': float(100*np.mean(np.abs(applied_change[m]) > .01)),
        'max_abs_difference': float(np.abs(applied_change[m]).max()),
        'recorded_total_gate_exceeded_ticks': int((applied_change[m] > 1e-12).sum())}
    out['replay_mode'] = 'conservative-stage-leash' if name.endswith('_leash') else 'recorded-gates'
    for k in ['raw', 'stage']:
        pe, ae = distances(p[k], b[k])
        out[k+'_vs_baseline'] = {'position_mm': stats(pe[m]*1000), 'angle_deg': stats(np.rad2deg(ae[m]))}
        out[k+'_bands'] = movement_bands(d.t.to_numpy(), p[k])
    lp, la = distances(p['raw'], p['stage'])
    ap, aa = distances(p['raw'], p['actual'])
    out['raw_to_stage_lag'] = {'position_mm': stats(lp[m]*1000), 'angle_deg': stats(np.rad2deg(la[m]))}
    out['raw_to_recorded_actual'] = {'position_mm': stats(ap[m]*1000), 'angle_deg': stats(np.rad2deg(aa[m]))}
    out['normal_events'] = {k: int((d[k].to_numpy()[m] != 0).sum()) for k in ['stall','lead_fault','projection_fault','reseeded']}
    out['all_reseed_times'] = d.loc[d.reseeded.astype(bool), 't'].tolist()
    out['normal_wire_mismatches'] = int((d.wire_seq.to_numpy()[m] != baseline.wire_seq.to_numpy()[m]).sum())
    # Last-solve diagnostics repeat per tick; count each segment once.
    seam = (np.r_[True, np.diff(d.segment) != 0] | np.r_[True, np.diff(d.t) > .01]) & m
    out['segment_jerk_search'] = {'segments': int(seam.sum()), 'reduced': int((d.jerk_scale.to_numpy()[seam] < .999).sum()),
        'scale': stats(d.jerk_scale.to_numpy()[seam]), 'extra_calculations': stats(d.jerk_search_calculations.to_numpy()[seam])}
    out['compute_us'] = stats(d.compute_us.to_numpy()[m])
    out['segment_compute_us'] = stats(d.compute_us.to_numpy()[seam])
    # Physical translation acceleration is in stand coordinates; use actual
    # recorded filter dt, not an assumed 2ms. Angular components rotate with the
    # current body frame, so they must not be treated as fixed-frame box jerk.
    dt = observed.filter_dt_ms.to_numpy()*.001
    contiguous = np.r_[False, np.diff(d.t) < .003] & m & np.r_[False, m[:-1]]
    stable_gate = np.r_[False, np.abs(np.diff(d.plan_gate)) < 1e-10]
    take = contiguous & stable_gate
    acc = d[[f'sample_a_{i}' for i in range(3)]].to_numpy()
    j = np.max(np.abs(np.diff(acc, axis=0)/dt[1:, None]), axis=1)
    out['fixed_plan_gate_linear_axis_jerk_m_s3'] = stats(j[take[1:]])
    # Exact baseline screen uses current nominal previous sent FK. Candidate
    # proxy uses previous stage; a subsequent native IK audit can replace that
    # with candidate previous sent FK while preserving recorded force strip.
    prev_raw = np.vstack([p['raw'][:1], p['raw'][:-1]])
    prev_stage = np.vstack([p['stage'][:1], p['stage'][:-1]])
    ep, ea = distances(prev_raw, prev_stage)
    proxy = leash(ep, ea)
    exactp, exacta = distances(prev_raw, p['reference'])
    exact_recorded = leash(exactp, exacta)
    prior_base_raw = np.vstack([b['raw'][:1], b['raw'][:-1]])
    bpe, bae = distances(prior_base_raw, pose(baseline, 'reference'))
    baseline_exact = leash(bpe, bae)
    # Recover only a floor on the other safety gate where baseline leash was
    # binding. Reporting leash-alone difference keeps that ambiguity explicit.
    diff = proxy-baseline_exact
    out['leash_proxy'] = {'below_one_ticks': int((proxy[m] < .999999).sum()), 'minimum': float(proxy[m].min()),
        'difference_gt_0p01_ticks': int((np.abs(diff[m]) > .01).sum()),
        'difference_gt_0p01_pct': float(100*np.mean(np.abs(diff[m]) > .01)),
        'max_abs_difference': float(np.abs(diff[m]).max()),
        'first_difference_time': float(d.t.to_numpy()[m & (np.abs(diff) > .01)][0]) if np.any(m & (np.abs(diff) > .01)) else None,
        'baseline_exact_vs_logged_plan_max': float(np.abs(baseline_exact[m]-baseline.plan_gate.to_numpy()[m]).max())}
    frame = pd.DataFrame({'t': d.t, 'mono': d.mono, 'normal': m,
                          'proxy_leash': proxy, 'baseline_exact_leash': baseline_exact,
                          'logged_plan_gate': baseline.plan_gate, 'proxy_minus_baseline': diff,
                          'raw_previous_to_stage_previous_m': ep, 'raw_previous_to_stage_previous_rad': ea})
    frame.to_csv(directory/f'{arm}_{name}_leash_screen.csv', index=False)
    export_native(d, directory/f'{arm}_{name}_postresume.jsonl')
    return out

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory', type=Path)
    p.add_argument('--variants', nargs='+', default=['baseline','jerk_only','filter_only_51_41','combined_51_41'])
    p.add_argument('--live-preview', action='store_true', help='Analyze live_preview_replay outputs; preserves stop/coverage and full worker failure seeds')
    p.add_argument('--window', nargs=2, type=float, action='append', help='Explicit live comparison interval, repeatable; default full common prefix')
    a = p.parse_args(); directory = a.directory
    if a.live_preview:
        live_preview_comparison(directory,a.variants,a.window)
        return
    observed = pd.read_feather(directory/'observed.feather').set_index('loop_start_time_ns', drop=False)
    result = {'normal_windows_sec': WINDOWS, 'spectral_method': movement_bands.__doc__,
              'fixed_feedback_limitation': 'Recorded force gates, lifecycle, model deltas, arrival times and actual poses remain fixed despite candidate commands. Plan gates are also fixed except explicitly named _leash variants, which apply min(recorded total gate, own previous-stage leash). Not a closed-loop task or robot/contact prediction.',
              'leash_screen_scope': 'One pass previous raw vs previous stage proxy, compared with exact baseline previous raw vs logged current nominal previous sent FK; no gates are altered. Native candidate FK audit is separate.',
              'timing_scope': 'Replay steady_clock includes follower/SMD and frame preparation, excludes line parsing and CSV output; concurrent replay processes are not a realtime deployment benchmark.', 'arms': {}}
    traces = {}
    for arm in ['left','right']:
        base = pd.read_csv(directory/f'{arm}_baseline.csv')
        obs = observed.loc[base.tick]
        fidelity = {}
        for k in ['raw','stage']:
            pe, ae = distances(pose(base,k), pose(base,'observed_'+k))
            fidelity[k] = {'max_position_mm': float(pe.max()*1000), 'max_angle_deg': float(np.rad2deg(ae).max())}
        fidelity['wire_mismatches'] = int((base.wire_seq.to_numpy() != obs[arm+'_follower_wire_seq'].to_numpy()).sum())
        fidelity['step_mismatches'] = int((base.step.to_numpy() != obs[arm+'_follower_step'].to_numpy()).sum())
        fidelity['duration_max_error_ms'] = float(np.abs(base.duration.to_numpy()-obs[arm+'_follower_duration_sec'].to_numpy()).max()*1000)
        result['arms'][arm] = {'baseline_fidelity': fidelity, 'variants': {}}
        for name in a.variants:
            d = pd.read_csv(directory/f'{arm}_{name}.csv')
            result['arms'][arm]['variants'][name] = candidate_summary(d, base, obs, arm, directory, name)
            traces[arm,name] = d
    (directory/'comparison_metrics.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    rows=[]
    for arm, ar in result['arms'].items():
        base = ar['variants']['baseline']
        for name, v in ar['variants'].items():
            for stage in ['raw','stage']:
                for band, values in v[stage+'_bands'].items():
                    for metric, value in values.items():
                        rows.append({'arm': arm, 'variant': name, 'signal':stage,'band_hz':band,'metric':metric,'value':value,
                                     'ratio_to_baseline_same_signal':value/base[stage+'_bands'][band][metric]})
    pd.DataFrame(rows).to_csv(directory/'band_comparison.csv',index=False)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(13,7), sharex=True)
    for i, arm in enumerate(['left','right']):
        for name in a.variants:
            d = traces[arm,name]; m = (d.t>=33)&(d.t<40)
            axes[i,0].plot(d.t[m], d.stage_y[m]*1000, label=name, lw=.9)
            axes[i,1].plot(d.t[m],d.lag_m[m]*1000,label=name,lw=.9)
        axes[i,0].set_ylabel(arm+' stage Y (mm)');axes[i,1].set_ylabel(arm+' raw-stage lag (mm)')
        axes[i,1].axhline(10,color='k',ls=':',lw=.6)
    axes[0,0].legend(fontsize=7); axes[1,0].set_xlabel('Seconds from first policy chunk');axes[1,1].set_xlabel('Seconds from first policy chunk')
    fig.suptitle('Same recorded deltas and force inputs; candidate commands, no simulated robot feedback')
    fig.tight_layout();fig.savefig(directory/'same_input_comparison.png',dpi=170)
    print(directory/'comparison_metrics.json')

if __name__ == '__main__':
    main()
