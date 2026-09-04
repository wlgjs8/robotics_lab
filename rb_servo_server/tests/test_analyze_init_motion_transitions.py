import csv
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))

import analyze_init_motion_transitions as amt

TICK_NS = 2_000_000


def fields():
    out = ["loop_start_time_ns", "init_motion_left_status", "init_motion_right_status"]
    for arm in ("left", "right"):
        out += [
            f"{arm}_mode",
            f"{arm}_follower_active",
            f"{arm}_tcp_target_profile",
            f"{arm}_safety_velocity_clamp_max_delta_deg",
            f"{arm}_safety_accel_clamp_max_delta_deg",
            f"{arm}_fc_enabled",
            f"{arm}_fc_covered",
            f"{arm}_fc_coverage_reason",
        ]
        out += [f"{arm}_q_sent_{j}" for j in range(6)]
        out += [f"{arm}_tcp_command_stand_{c}_m" for c in "xyz"]
    return out


class Sim:
    """Scripted per-arm state that the synthetic log is rendered from."""

    def __init__(self):
        self.q = {"left": [0.0] * 6, "right": [0.0] * 6}
        self.x = {"left": 0.5, "right": 0.5}
        self.mode = {"left": "TcpPoseTarget", "right": "TcpPoseTarget"}
        self.init = {"left": "idle", "right": "idle"}
        self.follower = {"left": 1.0, "right": 1.0}
        self.profile = {"left": "flow_infer_smooth", "right": "flow_infer_smooth"}
        self.vclamp = {"left": 0.0, "right": 0.0}
        self.covered = {"left": 1.0, "right": 1.0}
        self.reason = {"left": "covered", "right": "covered"}

    def row(self, i):
        r = {"loop_start_time_ns": 10_000_000_000 + i * TICK_NS}
        for arm in ("left", "right"):
            r[f"init_motion_{arm}_status"] = self.init[arm]
            r[f"{arm}_mode"] = self.mode[arm]
            r[f"{arm}_follower_active"] = int(self.follower[arm])
            r[f"{arm}_tcp_target_profile"] = self.profile[arm]
            r[f"{arm}_safety_velocity_clamp_max_delta_deg"] = self.vclamp[arm]
            r[f"{arm}_safety_accel_clamp_max_delta_deg"] = 0.0
            r[f"{arm}_fc_enabled"] = 1
            r[f"{arm}_fc_covered"] = int(self.covered[arm])
            r[f"{arm}_fc_coverage_reason"] = self.reason[arm]
            for j in range(6):
                r[f"{arm}_q_sent_{j}"] = self.q[arm][j]
            r[f"{arm}_tcp_command_stand_x_m"] = self.x[arm]
            r[f"{arm}_tcp_command_stand_y_m"] = 0.0
            r[f"{arm}_tcp_command_stand_z_m"] = 0.0
        return r


def both_arm_episode(kick: bool, snap: bool, recover_force: bool, fast_arrival: bool = False):
    """500 ticks streaming, 300 ticks init on both arms, 500 ticks streaming."""
    sim = Sim()
    rows = []
    i = 0
    for _ in range(500):
        for arm in ("left", "right"):
            sim.x[arm] += 0.0002  # 100 mm/s
            sim.q[arm][0] += 0.01
        rows.append(sim.row(i)); i += 1
    for k in range(300):
        for arm in ("left", "right"):
            sim.mode[arm] = "JointTarget"
            sim.init[arm] = "planning" if k < 60 else "executing"
            sim.follower[arm] = 0.0
            sim.covered[arm] = 0.0
            sim.reason[arm] = "F/T has no bias yet - run a tare before enabling compliance"
            if k == 60 and snap:
                sim.q[arm][2] += 0.03  # measured re-anchor snap at the hold entry
            # 2 deg/s approach, or 10 deg/s when the episode arrives too fast
            sim.q[arm][1] += 0.02 if fast_arrival else 0.004
        rows.append(sim.row(i)); i += 1
    for arm in ("left", "right"):
        sim.init[arm] = "done"
    for k in range(500):
        for arm in ("left", "right"):
            sim.mode[arm] = "TcpPoseTarget"
            sim.follower[arm] = 1.0 if k >= 100 else 0.0
            if kick and k < 10:
                sim.vclamp[arm] = 58.0 - 5.0 * k
                sim.q[arm][5] += 0.024 * (k + 1)  # accel-limit ramp
                sim.x[arm] += 0.0005
            else:
                sim.vclamp[arm] = 0.0
                sim.x[arm] += 0.0002
            if recover_force and k >= 250:
                sim.covered[arm] = 1.0
                sim.reason[arm] = "covered"
        rows.append(sim.row(i)); i += 1
    return rows


def single_arm_episode(peer_drop: bool):
    sim = Sim()
    rows = []
    i = 0
    for _ in range(300):
        for arm in ("left", "right"):
            sim.x[arm] += 0.0003  # 150 mm/s
        rows.append(sim.row(i)); i += 1
    for k in range(400):
        sim.mode["right"] = "JointTarget"
        sim.init["right"] = "executing"
        sim.follower["right"] = 0.0
        if peer_drop:
            sim.follower["left"] = 0.0
            sim.profile["left"] = "umi_large_smooth"
            sim.x["left"] += 0.00002 if k == 0 else 0.0003  # one-tick stop, then chase
        else:
            sim.x["left"] += 0.0003
        rows.append(sim.row(i)); i += 1
    sim.init["right"] = "done"
    for k in range(300):
        sim.mode["right"] = "TcpPoseTarget"
        sim.follower["right"] = 1.0 if k >= 50 else 0.0
        sim.follower["left"] = 1.0
        sim.profile["left"] = "flow_infer_smooth"
        for arm in ("left", "right"):
            sim.x[arm] += 0.0002
        rows.append(sim.row(i)); i += 1
    return rows


class AnalyzeInitMotionTransitionsTest(unittest.TestCase):
    def write(self, rows):
        tmp = tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False, newline="")
        self.addCleanup(lambda: Path(tmp.name).unlink(missing_ok=True))
        writer = csv.DictWriter(tmp, fieldnames=fields())
        writer.writeheader()
        for r in rows:
            writer.writerow(r)
        tmp.close()
        return tmp.name

    def test_clean_both_arm_episode_passes(self):
        report = amt.analyze(self.write(both_arm_episode(kick=False, snap=False, recover_force=True)))
        self.assertEqual([e["arm"] for e in report["episodes"]], ["left", "right"])
        for ep in report["episodes"]:
            self.assertTrue(ep["onset"]["pass"], ep["onset"])
            self.assertTrue(ep["arrival"]["pass"], ep["arrival"])
            self.assertAlmostEqual(ep["arrival"]["sent_speed_deg_s"], 2.0, places=6)
            self.assertTrue(ep["resume"]["pass"], ep["resume"])
            self.assertAlmostEqual(ep["resume"]["max_velocity_clamp_deg"], 0.0)
            self.assertAlmostEqual(ep["resume"]["follower_engage_sec"], 0.2, places=3)
            self.assertIsNone(ep["peer"]["pass"])  # both arms reset: no streaming peer
            self.assertTrue(ep["force"]["pass"], ep["force"])
            self.assertAlmostEqual(ep["force"]["covered_after_sec"], 0.5, places=3)
        text = amt.render(report)
        self.assertIn("resume  PASS", text)
        self.assertIn("force   PASS", text)

    def test_resume_kick_hold_snap_and_missing_tare_fail(self):
        report = amt.analyze(self.write(both_arm_episode(kick=True, snap=True, recover_force=False,
                                                         fast_arrival=True)))
        for ep in report["episodes"]:
            self.assertFalse(ep["onset"]["pass"])
            self.assertFalse(ep["arrival"]["pass"], ep["arrival"])
            self.assertAlmostEqual(ep["arrival"]["sent_speed_deg_s"], 10.0, places=6)
            self.assertGreater(ep["arrival"]["max_accel_deg_s2"], amt.ONSET_ACCEL_MAX_DEG_S2)
            self.assertGreaterEqual(ep["onset"]["max_step_deg"], 0.03 - 1e-9)
            self.assertGreater(ep["onset"]["max_accel_deg_s2"], amt.ONSET_ACCEL_MAX_DEG_S2)
            self.assertFalse(ep["resume"]["pass"])
            self.assertAlmostEqual(ep["resume"]["max_velocity_clamp_deg"], 58.0)
            self.assertGreater(ep["resume"]["max_accel_deg_s2"], amt.RESUME_ACCEL_MAX_DEG_S2)
            self.assertGreater(ep["resume"]["tcp_peak_speed_mm_s"], 200.0)
            self.assertFalse(ep["force"]["pass"])
            self.assertIsNone(ep["force"]["covered_after_sec"])
        text = amt.render(report)
        self.assertIn("onset   FAIL", text)
        self.assertIn("arrival FAIL", text)
        self.assertIn("resume  FAIL", text)
        self.assertIn("force   FAIL", text)

    def test_single_arm_episode_reports_peer(self):
        clean = amt.analyze(self.write(single_arm_episode(peer_drop=False)))
        self.assertEqual(len(clean["episodes"]), 1)
        ep = clean["episodes"][0]
        self.assertEqual(ep["arm"], "right")
        self.assertEqual(ep["peer"]["arm"], "left")
        self.assertTrue(ep["peer"]["pass"], ep["peer"])
        self.assertEqual(ep["peer"]["profiles"], ["flow_infer_smooth"])

        dropped = amt.analyze(self.write(single_arm_episode(peer_drop=True)))
        ep = dropped["episodes"][0]
        self.assertFalse(ep["peer"]["pass"], ep["peer"])
        self.assertAlmostEqual(ep["peer"]["follower_active_fraction"], 0.0)
        self.assertEqual(ep["peer"]["profiles"], ["umi_large_smooth"])
        self.assertGreater(ep["peer"]["max_one_tick_speed_drop_mm_s"], amt.PEER_SPEED_DROP_MAX_MM_S)
        self.assertIn("peer    FAIL", amt.render(dropped))

    def test_planning_tail_after_done_is_merged_into_the_episode(self):
        sim = Sim()
        rows = []
        i = 0
        for _ in range(100):
            rows.append(sim.row(i)); i += 1
        for k in range(200):
            sim.mode["right"] = "JointTarget"
            sim.init["right"] = "executing"
            rows.append(sim.row(i)); i += 1
        for k in range(40):  # done, then the re-emitted latch reports planning for 80 ms
            sim.mode["right"] = "Hold"
            sim.init["right"] = "done" if k < 10 else "planning"
            rows.append(sim.row(i)); i += 1
        for k in range(100):
            sim.mode["right"] = "TcpPoseTarget"
            sim.init["right"] = "idle"
            rows.append(sim.row(i)); i += 1
        report = amt.analyze(self.write(rows))
        self.assertEqual(len(report["episodes"]), 1)
        ep = report["episodes"][0]
        self.assertAlmostEqual(ep["t_start"], 0.2, places=3)
        self.assertAlmostEqual(ep["resume"]["t_resume"], 0.68, places=3)
        self.assertFalse(ep["resume"]["cold_start"])
        self.assertIn("summary:", amt.render(report))

    def test_no_episode(self):
        sim = Sim()
        rows = [sim.row(i) for i in range(50)]
        report = amt.analyze(self.write(rows))
        self.assertEqual(report["episodes"], [])
        self.assertIn("no InitMotion episode", amt.render(report))


if __name__ == "__main__":
    unittest.main()
