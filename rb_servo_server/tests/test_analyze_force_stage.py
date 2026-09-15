"""analyze_force_stage.py against synthetic logs that obey, and deliberately break,
the one-law force stage (2026-09-15). No robot or runtime is involved."""
import csv
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import analyze_force_stage as audit


REST, PEAK, B, V_CROSS = 10.0, 12.0, 500.0, 4.0
DT = 0.002


def synth_rows(arm="right", *, hold_gate=1.0, hold_demand=0.0, speed_scale=1.0,
               drop=(), duration_s=12.0):
    """A Hold-source hand push on a floor: 0-3 s rest, 3-3.2 s ramp to 14 N, 14 N to
    3.4 s, decay to 10.5 N by 3.6 s, 10.5 N held to 9 s, released, rest to the end.
    The arm yields at (|F|-rest)/b along F_hat and the command follows (the fold)."""
    n = int(round(duration_s / DT))
    t = np.arange(n) * DT
    direction = np.array([0.6, 0.0, -0.8])
    rng = np.random.default_rng(3)
    mag = np.full(n, 0.0) + 0.3 * np.abs(rng.standard_normal(n))     # free-space noise
    mag[(t >= 3.0) & (t < 3.2)] = np.interp(t[(t >= 3.0) & (t < 3.2)], [3.0, 3.2], [0.0, 14.0])
    mag[(t >= 3.2) & (t < 3.4)] = 14.0
    mag[(t >= 3.4) & (t < 3.6)] = np.interp(t[(t >= 3.4) & (t < 3.6)], [3.4, 3.6], [14.0, 10.5])
    hold = (t >= 3.6) & (t < 9.0)
    mag[hold] = 10.5 + 0.05 * rng.standard_normal(int(hold.sum()))
    force = mag[:, None] * direction[None, :]
    speed = speed_scale * np.clip(mag - REST, 0.0, None) / B
    vel = speed[:, None] * direction[None, :]
    cmd = np.array([0.30, -0.20, 0.10]) + np.cumsum(vel, axis=0) * DT
    pushed = mag > REST
    header = ["tick", "loop_start_time_ns", "fault_latched",
              f"{arm}_fc_covered", f"{arm}_fc_source", f"{arm}_fc_source_demand_m_s",
              f"{arm}_fc_gate_translation", f"{arm}_fc_gate_force_n", f"{arm}_fc_fold_sink",
              f"{arm}_fc_bounded"]
    header += [f"{arm}_ft_comp_sensor_nodz_f{ax}_n" for ax in "xyz"]
    header += [f"{arm}_fc_wrench_filt_f{ax}_n" for ax in "xyz"]
    header += [f"{arm}_fc_vel_{ax}_m_s" for ax in "xyz"]
    header += [f"{arm}_tcp_command_stand_{ax}_m" for ax in "xyz"]
    header += [f"{arm}_fc_contact_normal_{ax}" for ax in "xyz"]
    header += [f"{arm}_q_sent_accel_deg_s2_{j}" for j in range(6)]
    rows = []
    for i in range(n):
        normal = direction if mag[i] > 0.5 else np.zeros(3)
        row = [i, 10**16 + i * int(DT * 1e9), 0,
               1, "hold", hold_demand if pushed[i] else 0.0,
               hold_gate if pushed[i] else 1.0, mag[i], "hold" if pushed[i] else "", 0]
        row += list(force[i]) + list(force[i]) + list(vel[i]) + list(cmd[i]) + list(normal)
        row += [200.0 * np.sin(0.01 * i + j) for j in range(6)]
        rows.append(row)
    keep = [k for k, name in enumerate(header) if not any(name.endswith(d) for d in drop)]
    return [header[k] for k in keep], [[row[k] for k in keep] for row in rows]


def write_csv(header, rows):
    handle = tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False, newline="")
    with handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        writer.writerows(rows)
    return Path(handle.name)


def by_name(report, arm="right"):
    (arm_report,) = [a for a in report["arms"] if a["arm"] == arm]
    return {c["check"]: c for c in arm_report["checks"]}


class AnalyzeForceStageTests(unittest.TestCase):
    params = audit.Params(REST, PEAK, B, V_CROSS)

    def test_a_lawful_hand_push_passes(self):
        path = write_csv(*synth_rows())
        checks = by_name(audit.analyze(path, ("right",), self.params))
        self.assertEqual(checks["rest_equilibrium"]["status"], audit.PASS, checks["rest_equilibrium"])
        self.assertEqual(checks["yield_law"]["status"], audit.PASS, checks["yield_law"])
        self.assertAlmostEqual(checks["yield_law"]["numbers"]["speed_ratio_median"], 1.0, places=3)
        self.assertGreater(checks["yield_law"]["numbers"]["cos_median"], 0.999)
        self.assertEqual(checks["hold_source"]["status"], audit.PASS, checks["hold_source"])
        self.assertEqual(checks["floor_episodes"]["status"], audit.PASS, checks["floor_episodes"])
        self.assertEqual(checks["gate_model"]["status"], audit.PASS, checks["gate_model"])
        self.assertEqual(checks["deadlock"]["status"], audit.PASS)
        self.assertEqual(checks["chatter_gate_transitions"]["status"], audit.PASS)
        self.assertEqual(checks["chatter_contact_normal"]["status"], audit.PASS)
        self.assertEqual(checks["global"]["status"], audit.PASS, checks["global"])
        # The command stayed where the hand left it: the two rest segments drift 0.
        self.assertLess(checks["rest_equilibrium"]["numbers"]["command_drift_max_m"], 1e-6)

    def test_a_hold_whose_yield_was_read_back_as_demand_fails(self):
        """The 2026-09-15 11:25 incident: the compliant Hold's own yield fed the gate as
        stream demand and closed it to 0.095 during a hand push."""
        path = write_csv(*synth_rows(hold_gate=0.095, hold_demand=0.02))
        checks = by_name(audit.analyze(path, ("right",), self.params))
        self.assertEqual(checks["hold_source"]["status"], audit.FAIL, checks["hold_source"])
        self.assertLess(checks["hold_source"]["numbers"]["gate_open_fraction"], 1.0)
        self.assertLess(checks["hold_source"]["numbers"]["zero_demand_fraction"], 1.0)
        self.assertEqual(checks["gate_model"]["status"], audit.FAIL, checks["gate_model"])
        # A wrong gate is not a deadlock: the arm still yielded.
        self.assertEqual(checks["deadlock"]["status"], audit.PASS)

    def test_an_arm_that_yields_too_slowly_fails_the_law(self):
        path = write_csv(*synth_rows(speed_scale=0.4))
        checks = by_name(audit.analyze(path, ("right",), self.params))
        self.assertEqual(checks["yield_law"]["status"], audit.FAIL)
        self.assertAlmostEqual(checks["yield_law"]["numbers"]["speed_ratio_median"], 0.4, places=3)

    def test_missing_new_columns_skip_instead_of_crashing(self):
        """A pre-2026-09-15 log has no source / demand / contact-normal columns."""
        path = write_csv(*synth_rows(drop=("_fc_source", "_fc_source_demand_m_s",
                                           "_fc_contact_normal_x", "_fc_contact_normal_y",
                                           "_fc_contact_normal_z")))
        checks = by_name(audit.analyze(path, ("right",), self.params))
        self.assertEqual(checks["hold_source"]["status"], audit.SKIPPED)
        self.assertEqual(checks["gate_model"]["status"], audit.SKIPPED)
        self.assertEqual(checks["chatter_contact_normal"]["status"], audit.SKIPPED)
        self.assertEqual(checks["rest_equilibrium"]["status"], audit.PASS)
        self.assertEqual(checks["yield_law"]["status"], audit.PASS)
        self.assertEqual(checks["deadlock"]["status"], audit.PASS)
        self.assertEqual(checks["global"]["status"], audit.PASS)
        self.assertIn("fc_source column missing", checks["global"]["detail"])
        # And the whole report is JSON-serialisable.
        import json
        json.dumps(audit.analyze(path, ("right",), self.params))

    def test_an_arm_with_no_columns_at_all_is_all_skipped(self):
        header, rows = synth_rows(arm="right")
        path = write_csv(header, rows)
        checks = by_name(audit.analyze(path, ("left",), self.params), arm="left")
        self.assertTrue(all(c["status"] == audit.SKIPPED for c in checks.values()), checks)

    def test_gate_model_matches_cm_0049(self):
        g = audit.gate_model(np.array([0.0, 12.0, 12.0, 6.0]), np.array([0.1, 0.1, 0.002, 0.1]), PEAK, 0.004)
        self.assertAlmostEqual(g[0], 1.0)              # no force: open whatever the demand
        self.assertAlmostEqual(g[1], 0.004 / 0.1)      # at peak the gate returns v_cross
        self.assertAlmostEqual(g[2], 1.0)              # demand below v_cross: nothing to cut
        self.assertAlmostEqual(g[3], (0.004 / 0.1) ** 0.25)

    def test_cli_runs_and_signals_failure(self):
        path = write_csv(*synth_rows(hold_gate=0.095, hold_demand=0.02))
        out = Path(tempfile.mkdtemp()) / "report.json"
        code = audit.main([str(path), "--arm", "right", "--json", str(out)])
        self.assertEqual(code, 1)
        self.assertTrue(out.exists())


class SingleTargetAuditTests(unittest.TestCase):
    def make_log(self, speed_scale=1., wrong_gate=False, missing_force=False):
        n=1500
        force=np.zeros((n,3));force[200:700]=[18.,0.,24.]
        force[700:1000]=[-24.,18.,0.]
        vel=np.zeros((n,3));physical=np.ones(n);confidence=np.zeros(n)
        smooth=lambda x: np.clip(x,0.,1.)**2*(3.-2.*np.clip(x,0.,1.))
        for i in range(1,n):
            mag=np.linalg.norm(force[i]);drive=force[i]*max(0.,mag-20.)/max(mag,1e-30)
            vel[i]=vel[i-1]+DT*(drive-500.*vel[i-1])/20.
            desired=1.-smooth(mag/20.);tau=.1 if desired<physical[i-1] else .4
            physical[i]=physical[i-1]+DT/tau*(desired-physical[i-1])
            confidence[i]=smooth(mag-2.)
        cols={"loop_start_time_ns":10**16+np.arange(n)*2_000_000,
              "right_fc_covered":np.ones(n),"right_fc_source":np.full(n,"hold"),
              "right_fc_source_demand_m_s":np.zeros(n),"right_fc_target_force_n":np.full(n,20.),
              "right_fc_gate_m_eff":np.full(n,20.),"right_fc_gate_b_eff":np.full(n,500.),
              "right_fc_physical_gate":physical,"right_fc_contact_confidence":confidence,
              "right_fc_gate_translation":np.ones(n) if wrong_gate else 1.-confidence*(1.-physical),
              "right_fc_gate_force_n":np.linalg.norm(force,axis=1)}
        for j,axis in enumerate("xyz"):
            if not missing_force:cols[f"right_fc_wrench_filt_f{axis}_n"]=force[:,j]
            cols[f"right_fc_vel_{axis}_m_s"]=speed_scale*vel[:,j]
            cols[f"right_ft_comp_sensor_nodz_f{axis}_n"]=np.full(n,100.)
        path=write_csv(list(cols),zip(*cols.values()));self.addCleanup(path.unlink)
        return path

    def test_dynamic_law_uses_its_filtered_input_and_coast(self):
        report=audit.analyze(self.make_log(),("right",),audit.Params())
        checks=by_name(report)
        self.assertEqual(report["arms"][0]["law_schema"],"single_target")
        self.assertEqual(checks["yield_law"]["status"],audit.PASS,checks["yield_law"])
        self.assertEqual(checks["rest_equilibrium"]["status"],audit.PASS,checks["rest_equilibrium"])
        self.assertEqual(checks["gate_model"]["status"],audit.PASS,checks["gate_model"])
        self.assertEqual(checks["hold_source"]["status"],audit.PASS)
        self.assertEqual(checks["floor_episodes"]["status"],audit.SKIPPED)

    def test_wrong_dynamics_and_gate_fail(self):
        checks=by_name(audit.analyze(self.make_log(.5,True),("right",),audit.Params()))
        self.assertEqual(checks["yield_law"]["status"],audit.FAIL)
        self.assertEqual(checks["gate_model"]["status"],audit.FAIL)

    def test_missing_law_vector_never_falls_back_to_raw_force(self):
        checks=by_name(audit.analyze(self.make_log(missing_force=True),("right",),audit.Params()))
        self.assertEqual(checks["yield_law"]["status"],audit.SKIPPED)
        self.assertEqual(checks["rest_equilibrium"]["status"],audit.SKIPPED)


if __name__ == "__main__":
    unittest.main()
