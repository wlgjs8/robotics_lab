import csv
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import analyze_force_observability as audit


class ForceObservabilityTests(unittest.TestCase):
    def test_staggered_joint_updates_are_not_packet_repeats(self):
        time = np.arange(1000)*0.002
        sampled = np.empty((1000,6))
        for joint in range(6):
            phase = 0 if joint<3 else 1
            source = np.maximum(0,np.arange(1000)-(np.arange(1000)-phase)%2)
            sampled[:,joint]=(joint+1)*time[source]
        stamp = 10**16+np.arange(1000,dtype=np.int64)*2000000
        result = audit.change_pattern(sampled,stamp)
        self.assertEqual(result['packet_repeat_fraction'],0)
        self.assertGreater(result['partial_joint_change_fraction'],0.99)
        self.assertTrue(all(0.49<f<0.51 for f in result['joint_value_change_fraction']))
        self.assertTrue(all(f>0.99 for f in result['joint_change_toggle_fraction']))
        masks = {item['mask'] for item in result['top_masks_joint_1_first'][:2]}
        self.assertEqual(masks,{'111000','000111'})
        # Constant true velocity can create a 250 Hz readback derivative signal.
        velocity = np.diff(sampled,axis=0)/0.002
        band = audit.spectral_summary(velocity,0.002)['bands']['180_251_hz']
        self.assertGreater(band['peak_hz'],240)
        self.assertGreater(band['vector_rms'],1)

    def test_stationary_values_do_not_imply_dead_packets(self):
        result = audit.change_pattern(np.zeros((100,6)),np.arange(100)*2000000)
        self.assertEqual(result['packet_repeat_fraction'],0)
        self.assertEqual(result['joint_value_change_fraction'],[0]*6)

    def test_windows_do_not_join_rollouts_or_missing_ticks(self):
        stamp = np.arange(10)*2000000
        active = np.array([1,1,1,0,0,1,1,1,1,1])
        tick = np.array([0,1,2,3,4,5,6,8,9,10])
        result = audit.segments(tick,stamp,active)
        self.assertEqual([v.tolist() for v in result],[[0,1,2],[5,6],[7,8,9]])

    def test_nonuniform_quadratic_preserves_midpoint_and_derivative(self):
        stamp = 10**16+np.array([0,2,4,7,9,12,14,17,19])*1000000
        time = (stamp-stamp[0])*1e-9
        acceleration = np.array([2,-3,1])
        positions = 1+time[:,None]*np.array([0.1,0.2,-0.3])+0.5*time[:,None]**2*acceleration
        result = audit.quadratic_acceleration(stamp,positions,9)
        self.assertEqual(len(result),1)
        self.assertEqual(result[0][1],10**16+9500000)
        self.assertAlmostEqual(result[0][2],0.0095)
        np.testing.assert_allclose(result[0][3],acceleration,atol=1e-7)

    def test_smoothing_reduces_synthetic_derivative_noise_but_has_age(self):
        rng = np.random.default_rng(7)
        stamp = 10**16+np.arange(700)*2000000
        positions = np.zeros((700,3))+rng.normal(0,5e-6,(700,3))
        short = audit.quadratic_acceleration(stamp,positions,3)
        long = audit.quadratic_acceleration(stamp,positions,17)
        short_rms = np.sqrt(np.mean(np.asarray([v[3] for v in short])**2))
        long_rms = np.sqrt(np.mean(np.asarray([v[3] for v in long])**2))
        self.assertLess(long_rms,short_rms/20)
        self.assertAlmostEqual(long[0][2],0.016)
        self.assertGreater(long[0][2],short[0][2])

    def test_missing_or_repeated_time_is_not_an_acceleration_sample(self):
        stamp = np.array([1,3,3,7,9])*1000000
        self.assertEqual(audit.quadratic_acceleration(stamp,np.zeros((5,3)),5),[])
        stamp = np.array([1,3,5,70,72])*1000000
        self.assertEqual(audit.quadratic_acceleration(stamp,np.zeros((5,3)),5),[])
        with self.assertRaises(ValueError):
            audit.quadratic_acceleration(stamp,np.zeros((5,3)),4)

    def test_spectral_peak_and_alignment_have_distinct_meanings(self):
        dt=0.002
        time=np.arange(1000)*dt
        source=np.column_stack([np.sin(2*np.pi*7*time),np.sin(2*np.pi*13*time)])
        response=np.column_stack([np.sin(2*np.pi*7*(time-.018)),np.sin(2*np.pi*13*(time-.018))])
        match=audit.alignment(source,response,dt)
        self.assertAlmostEqual(match['best_lag_ms'],18)
        self.assertGreater(match['correlation'],0.999)
        self.assertFalse(match['at_search_boundary'])
        spectral=audit.spectral_summary(np.sin(2*np.pi*8*time),dt)
        self.assertAlmostEqual(spectral['bands']['2_40_hz']['peak_hz'],8)
        self.assertFalse(audit.alignment(source[:20],response[:20],dt)['available'])

    def test_csv_preserves_integer_clocks_and_reports_partial_rows(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'sample.csv'
            clock=10000000000000001
            with path.open('w',newline='') as stream:
                writer=csv.writer(stream)
                writer.writerow(['tick','loop_start_time_ns','left_state_host_time_ns'])
                writer.writerow([1,clock,clock-1])
                writer.writerow([2,clock+2000000,clock+1999999])
                writer.writerow([3])
            data,malformed=audit.read_log(path)
            self.assertEqual(int(data['loop_start_time_ns'][0]),clock)
            self.assertEqual(int(data['left_state_host_time_ns'][0]),clock-1)
            self.assertEqual(malformed,1)
            summary,_=audit.audit_arm(data,'left')
            self.assertFalse(summary['available'])
            self.assertIn('missing_columns',summary)

    def test_com_uses_calibrated_sro_offsets_and_tool_rotation(self):
        calibration={'tool_mass_kg':0.8,'tool_com_mm':[0,0,25],
                     'tool_xyz_mm':[0,0,200],'tool_rpy_deg':[0,0,90]}
        # TCP and tool rotations cancel: flange is identity in stand coordinates.
        com,force,mass=audit.com_and_force(np.array([[1.,2.,3.]]),
            np.array([[0.,0.,np.pi/2]]),np.array([[4.,5.,6.]]),calibration)
        np.testing.assert_allclose(com,[[1,2,2.825]],atol=1e-12)
        np.testing.assert_allclose(force,[[4,5,6]],atol=1e-12)
        self.assertEqual(mass,0.8)

    def test_known_inertia_comparison_uses_midpoint_not_latest_sample(self):
        stamp=10**16+np.arange(800)*2000000
        time=(stamp-stamp[0])*1e-9
        omega=2*np.pi*3
        com=np.column_stack([0.001*np.sin(omega*time),np.zeros((800,2))])
        force=0.8*omega**2*com
        result=audit.inertia_diagnostic(stamp,com,force,0.8)
        match=result['windows'][1]['same_logged_time']
        self.assertLess(match['fixed_mass_residual_fluctuation_rms_n'],match['force_fluctuation_rms_n']*0.01)


if __name__=='__main__':
    unittest.main()
