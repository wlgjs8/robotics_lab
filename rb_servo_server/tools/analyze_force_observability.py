#!/usr/bin/env python3
"""Read-only servo-log audit. No robot, network or backend is constructed.

Fresh host packets are not necessarily simultaneous fresh encoder samples.
Report observable value changes, spectral content and alignment separately.
Acceleration fits are offline diagnostics at their window midpoint, NOT an
external-force observer or a filter to place in the force feedback path.
Requires numpy; CSV parsing does not require pandas or the robot runtime.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


ARMS = ("left", "right")


def columns(arm):
    return (["tick", "loop_start_time_ns", f"{arm}_state_host_time_ns",
             f"{arm}_preview_execution_active"] +
            [f"{arm}_q_{kind}_{j}" for kind in ("actual", "sent", "ref") for j in range(6)] +
            [f"{arm}_tcp_actual_stand_{axis}_m" for axis in "xyz"] +
            [f"{arm}_tcp_actual_stand_r{axis}_rad" for axis in "xyz"] +
            [f"{arm}_ft_comp_sensor_nodz_f{axis}_n" for axis in "xyz"] +
            [f"{arm}_ft_tool_mass_kg", f"{arm}_state_acquisition_sequence",
             f"{arm}_state_robot_time_ns", f"{arm}_qsync_underrun_events",
             f"{arm}_worker_pending_overwrites_total", f"{arm}_worker_repeated_sends_total"])


def read_log(path):
    wanted = set(columns("left") + columns("right"))
    with Path(path).open(newline="") as stream:
        reader = csv.reader(stream)
        header = next(reader)
        selected = [(i, name) for i, name in enumerate(header) if name in wanted]
        result = {name: [] for _, name in selected}
        malformed = 0
        for row in reader:
            if len(row) != len(header):
                malformed += 1
                continue
            parsed = {}
            for index, name in selected:
                try:
                    # Preserve nanosecond clocks and identities above 2**53.
                    integer = name.endswith(("_ns", "_sequence")) or name == "tick"
                    parsed[name] = int(row[index]) if integer else float(row[index])
                except ValueError:
                    parsed[name] = 0 if integer else float("nan")
            for name in result:
                result[name].append(parsed[name])
    arrays = {name: np.asarray(values) for name, values in result.items()}
    return arrays, malformed


def segments(ticks, stamps_ns, active):
    """Contiguous selected log intervals. Never concatenate separate rollouts."""
    ticks, stamps_ns, active = np.asarray(ticks), np.asarray(stamps_ns), np.asarray(active, dtype=bool)
    positive = np.diff(stamps_ns)
    positive = positive[positive > 0]
    if not len(positive):
        return []
    max_gap = 2.5 * float(np.median(positive))
    output, start = [], None
    for i in range(len(active)):
        connected = (i > 0 and ticks[i] == ticks[i-1]+1 and
                     0 < stamps_ns[i]-stamps_ns[i-1] <= max_gap)
        if start is not None and (not active[i] or not connected):
            output.append(np.arange(start, i))
            start = None
        if active[i] and start is None:
            start = i
    if start is not None:
        output.append(np.arange(start, len(active)))
    return output


def quantiles(values):
    values = np.asarray(values)
    values = values[np.isfinite(values)]
    return np.quantile(values, [0.5, 0.99, 1.0]).tolist() if len(values) else None


def change_pattern(q, stamps_ns):
    fresh = np.r_[True, np.diff(stamps_ns) > 0]
    q, stamps_ns = np.asarray(q)[fresh], np.asarray(stamps_ns)[fresh]
    if len(q) < 4:
        return {"available": False}
    changed = np.abs(np.diff(q, axis=0)) > 1e-8  # CSV value comparison tolerance, not a motion gate
    masks = (changed * (1 << np.arange(q.shape[1]))).sum(axis=1)
    values, counts = np.unique(masks, return_counts=True)
    order = np.argsort(-counts)[:8]
    return {
        "available": True,
        "packet_repeat_fraction": float(1-fresh.mean()),
        "packet_intervals_ms_p50_p99_max": quantiles(np.diff(stamps_ns)*1e-6),
        "joint_value_change_fraction": changed.mean(axis=0).tolist(),
        "joint_change_toggle_fraction": (changed[1:] != changed[:-1]).mean(axis=0).tolist(),
        "partial_joint_change_fraction": float(((changed.sum(axis=1)>0)&(changed.sum(axis=1)<q.shape[1])).mean()),
        "top_masks_joint_1_first": [{"mask": format(int(values[i]), "06b")[::-1],
                                    "count": int(counts[i])} for i in order],
        "meaning": "value changes only; stationary or quantized joints do not reveal acquisition times",
    }


def spectrum(values, dt):
    values = np.asarray(values, dtype=float)
    if values.ndim == 1:
        values = values[:, None]
    if len(values) < 32 or not np.isfinite(values).all() or not np.isfinite(dt) or dt <= 0:
        return None
    # Remove a linear trend, then use one Hann-windowed periodogram. No coherence
    # or transfer-function claim is made from a single short closed-loop segment.
    design = np.column_stack([np.ones(len(values)), np.linspace(-1,1,len(values))])
    residual = values-design@np.linalg.lstsq(design, values, rcond=None)[0]
    window = np.hanning(len(values))
    transform = np.fft.rfft(residual*window[:,None], axis=0)
    power = (abs(transform)**2).sum(axis=1)/(len(values)*np.sum(window**2))
    power[1:-1 if len(values)%2 == 0 else None] *= 2
    frequency = np.fft.rfftfreq(len(values), dt)
    return frequency, power


def spectral_summary(values, dt):
    result = spectrum(values, dt)
    if result is None:
        return {"available": False}
    frequency, power = result
    bands = {}
    for low, high in [(2,40),(40,100),(100,180),(180,251)]:
        selected = (frequency>=low)&(frequency<high)
        bands[f"{low}_{high}_hz"] = {
            "vector_rms": float(np.sqrt(power[selected].sum())),
            "peak_hz": float(frequency[selected][np.argmax(power[selected])]) if selected.any() else None,
        }
    return {"available": True, "resolution_hz": float(frequency[1]), "bands": bands}


def quadratic_acceleration(stamps_ns, positions, window):
    """Causal quadratic fit, returned at the window's central time with age.

    A midpoint timestamp is appropriate for a local quadratic/cubic model; it is
    not the acquisition instant of a physical accelerometer. Irregular host times
    and staggered encoders can still bias the estimate. noise_gain is the L2 gain
    from independent position noise in meters to acceleration in m/s², not a bound
    on real sensor error. Do not subtract this estimate from a current F/T sample.
    """
    stamps_ns = np.asarray(stamps_ns, dtype=np.int64)
    positions = np.asarray(positions, dtype=float)
    if window < 3 or window % 2 == 0:
        raise ValueError("window must be an odd integer >= 3")
    if positions.ndim != 2 or len(positions) != len(stamps_ns):
        raise ValueError("position/timestamp dimensions differ")
    output = []
    for end in range(window-1, len(stamps_ns)):
        stamps = stamps_ns[end-window+1:end+1]
        values = positions[end-window+1:end+1]
        gaps = np.diff(stamps)
        if (not np.isfinite(values).all() or np.any(gaps<=0) or
                gaps.max()>2.5*np.median(gaps)):
            continue
        span = int(stamps[-1])-int(stamps[0])
        midpoint = int(stamps[0])+span//2
        half_sec = span*0.5e-9
        scaled = (stamps-midpoint)*1e-9/half_sec
        design = np.column_stack([np.ones(window), scaled, scaled**2])
        inverse = np.linalg.pinv(design)
        weights = 2*inverse[2]/half_sec**2
        acceleration = weights@values
        fit = design@(inverse@values)
        output.append((end, midpoint, (int(stamps[-1])-midpoint)*1e-9,
                       acceleration, np.sqrt(np.mean((values-fit)**2,axis=0)), float(np.linalg.norm(weights))))
    return output


def alignment(source, response, dt, max_lag_ms=60):
    """Observed position alignment only, not identified plant/queue delay."""
    if len(source) < 128:
        return {"available": False, "reason": "short_segment"}
    if (not np.isfinite(dt) or dt<=0 or not np.isfinite(source).all() or
            not np.isfinite(response).all()):
        return {"available": False, "reason": "invalid_signal_or_period"}
    half = int(round(max_lag_ms*1e-3/dt))
    if half<1 or len(source) <= 2*half+32:
        return {"available": False, "reason": "insufficient_lag_overlap"}
    # Same support for every lag, per-joint centering, then vector correlation.
    scores = []
    centered_source = source[half:-half]-source[half:-half].mean(axis=0)
    source_norm = np.linalg.norm(centered_source)
    for lag in range(-half,half+1):
        segment = response[half+lag:len(response)-half+lag]
        centered = segment-segment.mean(axis=0)
        norm = source_norm*np.linalg.norm(centered)
        scores.append(float(np.sum(centered_source*centered)/norm) if norm>1e-12 else 0)
    best = int(np.argmax(scores))
    return {"available": True, "best_lag_ms": (best-half)*dt*1e3,
            "correlation": scores[best], "at_search_boundary": best in (0,2*half),
            "meaning": "positive = response later; trend/feedback/confounding can bias this alignment"}


def rotations(rpy):
    """Flange/tool-to-stand Rz Ry Rx, matching math::rotationFromPose."""
    sx,sy,sz=np.sin(rpy).T
    cx,cy,cz=np.cos(rpy).T
    return np.asarray([[cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx],
                       [sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx],
                       [-sy,cy*sx,cy*cx]]).transpose(2,0,1)


def com_and_force(tcp_position,tcp_rpy,force_flange,calibration):
    mass=float(calibration['tool_mass_kg'])
    offset=(np.asarray(calibration['tool_com_mm'])-np.asarray(calibration['tool_xyz_mm']))*1e-3
    tool=rotations(np.deg2rad(np.asarray([calibration['tool_rpy_deg']])))[0]
    if not np.isfinite(mass) or mass<=0 or offset.shape!=(3,) or not np.isfinite(offset).all():
        raise ValueError('invalid explicit tool calibration')
    flange=rotations(tcp_rpy)@tool.T
    # Both origins are relative to the same SRO, so sensor_offset cancels.
    com=tcp_position+np.einsum('nij,j->ni',flange,offset)
    force=np.einsum('nij,nj->ni',flange,force_flange)
    return com,force,mass


def inertia_diagnostic(stamps,com,force,mass):
    """Fixed calibrated mass comparison, without fitting mass from contact data."""
    time=(stamps-stamps[0])*1e-9
    output={'method':'F_external hypothesis = F_gravity_bias_compensated + calibrated_mass * a_COM',
            'limitation':'COM reconstructed with supplied calibration; host receipt time is not sensor acquisition time. '
                         'Correlation in closed-loop motion cannot distinguish inertia from contact.',
            'windows':[]}
    for window in (9,17,33):
        fitted=quadratic_acceleration(stamps,com,window)
        if len(fitted)<20:
            continue
        center=(np.asarray([item[1] for item in fitted],dtype=np.int64)-stamps[0])*1e-9
        acceleration=np.asarray([item[3] for item in fitted])
        # Keep identical overlap for every tested force/kinematics alignment.
        keep=(center>=time[0]+0.03)&(center<=time[-1]-0.03)
        center,acceleration=center[keep],acceleration[keep]
        if len(center)<20:
            continue
        trials=[]
        for lag_ms in [0]+[sign*lag for lag in range(2,31,2) for sign in (-1,1)]:
            observed=np.column_stack([np.interp(center+lag_ms*1e-3,time,force[:,axis]) for axis in range(3)])
            residual=observed+mass*acceleration
            force_rms=float(np.sqrt(np.mean(np.sum((observed-observed.mean(axis=0))**2,axis=1))))
            residual_rms=float(np.sqrt(np.mean(np.sum((residual-residual.mean(axis=0))**2,axis=1))))
            trials.append({'force_relative_lag_ms':lag_ms,'force_fluctuation_rms_n':force_rms,
                           'fixed_mass_residual_fluctuation_rms_n':residual_rms,
                           'residual_norm_n_p50_p99_max':quantiles(np.linalg.norm(residual,axis=1))})
        best=min(trials,key=lambda item:item['fixed_mass_residual_fluctuation_rms_n'])
        output['windows'].append({'samples':window,'fit_age_ms':float(np.median([item[2] for item in fitted]))*1e3,
                                  'same_logged_time':trials[0],'best_diagnostic_alignment':best})
    return output


def audit_arm(data, arm, calibration=None):
    required = columns(arm)[:4]+[f"{arm}_q_{kind}_{j}" for kind in ("actual","sent","ref") for j in range(6)]
    missing = [key for key in required if key not in data]
    if missing:
        return {"available": False, "missing_columns": missing}, {}
    valid = (data[f"{arm}_state_host_time_ns"]>0)&(data['loop_start_time_ns']>0)
    for key in required[4:]:
        valid &= np.isfinite(data[key])
    intervals = segments(data["tick"], data["loop_start_time_ns"],
                         (data[f"{arm}_preview_execution_active"]==1)&valid)
    if not intervals:
        return {"available": False, "reason": "no_preview_active_samples"}, {}
    index = max(intervals, key=len)
    t_ns = data["loop_start_time_ns"][index].astype(np.int64)
    dt = float(np.median(np.diff(t_ns)))*1e-9 if len(t_ns)>1 else 0
    state_ns = data[f"{arm}_state_host_time_ns"][index].astype(np.int64)
    joint = {kind: np.column_stack([data[f"{arm}_q_{kind}_{j}"][index] for j in range(6)])
             for kind in ("actual","sent","ref")}
    summary = {"available": True, "active_segment_rows": [len(item) for item in intervals],
               "analysis_segment_first_tick": int(data['tick'][index[0]]),
               "analysis_segment_rows": len(index), "analysis_duration_sec": float((t_ns[-1]-t_ns[0])*1e-9),
               "loop_dt_ms_p50_p99_max": quantiles(np.diff(t_ns)*1e-6),
               "actual_sampling": change_pattern(joint['actual'],state_ns),
               "reference_sampling": change_pattern(joint['ref'],state_ns),
               "recorded_acquisition_sequence": f"{arm}_state_acquisition_sequence" in data,
               "recorded_robot_time": f"{arm}_state_robot_time_ns" in data,
               "sent_to_actual_alignment": alignment(joint['sent'],joint['actual'],dt),
               "sent_to_reference_alignment": alignment(joint['sent'],joint['ref'],dt)}
    trace = {'time_sec':(t_ns-t_ns[0])*1e-9, **{'q_'+kind:values for kind,values in joint.items()}}
    if dt<=0:
        return summary,trace
    for kind, values in joint.items():
        summary[kind+'_velocity_spectrum_deg_s'] = spectral_summary(np.diff(values,axis=0)/dt,dt)
    force_keys = [f"{arm}_ft_comp_sensor_nodz_f{axis}_n" for axis in 'xyz']
    if all(key in data for key in force_keys):
        force = np.column_stack([data[key][index] for key in force_keys])
        trace['force_sensor_nodz_n'] = force
        magnitude = np.linalg.norm(force,axis=1)
        summary['force_spectrum_n'] = spectral_summary(force,dt)
        summary['measured_wrench_norm_n_p50_p99_max'] = quantiles(magnitude)
        measured = magnitude[np.isfinite(magnitude)]
        summary['measured_wrench_valid_samples'] = len(measured)
        summary['measured_wrench_above_10_fraction'] = float((measured>=10).mean()) if len(measured) else None
    position_keys = [f"{arm}_tcp_actual_stand_{axis}_m" for axis in 'xyz']
    if all(key in data for key in position_keys):
        positions = np.column_stack([data[key][index] for key in position_keys])
        fresh = np.r_[True,np.diff(state_ns)>0]
        mass_key = f"{arm}_ft_tool_mass_kg"
        mass = float(np.nanmedian(data[mass_key][index])) if mass_key in data else None
        if mass is not None and not np.isfinite(mass):
            mass = None
        summary['tcp_acceleration_diagnostics'] = {'position':'TCP, not COM', 'tool_mass_kg':mass, 'windows':[]}
        trace['tcp_actual_m'] = positions
        for window in (3,9,17,33):
            fitted = quadratic_acceleration(state_ns[fresh],positions[fresh],window)
            if not fitted:
                continue
            acceleration = np.asarray([item[3] for item in fitted])
            norms = np.linalg.norm(acceleration,axis=1)
            summary['tcp_acceleration_diagnostics']['windows'].append({
                'samples':window,'fit_age_ms_p50_p99_max':quantiles([item[2]*1e3 for item in fitted]),
                'acceleration_m_s2_p50_p99_max':quantiles(norms),
                'mass_times_tcp_acceleration_proxy_n_p50_p99_max':quantiles(norms*mass) if mass is not None else None,
                'position_noise_gain_per_s2_p50_p99_max':quantiles([item[5] for item in fitted])})
        if calibration is not None and all(key in data for key in force_keys):
            rotation_keys=[f'{arm}_tcp_actual_stand_r{axis}_rad' for axis in 'xyz']
            if all(key in data for key in rotation_keys):
                rpy=np.column_stack([data[key][index] for key in rotation_keys])
                com,stand_force,calibrated_mass=com_and_force(positions,rpy,force,calibration)
                if mass is None or abs(calibrated_mass-mass)>1e-5:
                    summary['inertia_diagnostic']={'available':False,'reason':'supplied and recorded mass differ or missing'}
                elif np.isfinite(com).all() and np.isfinite(stand_force).all():
                    summary['inertia_diagnostic']=inertia_diagnostic(state_ns[fresh],com[fresh],stand_force[fresh],mass)
                    trace['com_m']=com
                    trace['force_stand_n']=stand_force
    return summary,trace


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs',nargs='+',type=Path)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--trace-dir',type=Path)
    parser.add_argument('--calibration-config',type=Path,
                        help='Optional tracked YAML for COM reconstruction; not a historical run config snapshot. Requires PyYAML.')
    args = parser.parse_args()
    output = {'schema':'robotics_lab.force_observability.v1',
              'scope':'longest contiguous preview-active interval per arm; recorded closed-loop data, no ground-truth contact labels',
              'cautions':['Host receive timestamps are not encoder acquisition timestamps.',
                          'Spectra use nominal median period; resolution and timing jitter are reported.',
                          'Force includes residual tool inertia; TCP acceleration is not calibrated COM acceleration.',
                          'A short closed-loop alignment cannot identify a causal plant transfer function.'], 'runs':[]}
    calibration=None
    if args.calibration_config:
        import yaml
        raw=args.calibration_config.read_bytes()
        config=yaml.safe_load(raw)
        calibration={arm:config['force_torque'][arm] for arm in ARMS}
        output['supplied_calibration']={'path':str(args.calibration_config.resolve()),
            'sha256':hashlib.sha256(raw).hexdigest(),
            'assumption':'This supplied current calibration also applies to the historical runs. Mass is cross-checked against logs.'}
    for path in args.logs:
        data, malformed = read_log(path)
        item = {'file':str(path.resolve()),'rows':len(data.get('tick',[])), 'malformed_rows':malformed, 'arms':{}}
        for arm in ARMS:
            summary,trace = audit_arm(data,arm,calibration[arm] if calibration else None)
            item['arms'][arm] = summary
            if args.trace_dir and trace:
                args.trace_dir.mkdir(parents=True,exist_ok=True)
                np.savez_compressed(args.trace_dir/(path.stem+'_'+arm+'.npz'),**trace)
        output['runs'].append(item)
        print(f'{path.name}: {item["rows"]} rows, {malformed} malformed',flush=True)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(output,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':
    main()
