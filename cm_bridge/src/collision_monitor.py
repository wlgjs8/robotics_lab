#!/usr/bin/env python3
"""cm_bridge collision monitor (P2) — host-side self-collision guard.

Ports rb_servo_server's mesh self-collision guard posture to the CM stack:
loads the SAME production URDF (dual_rb3_730e_ver5, CoACD hulls), applies the
stack_real.yaml pair curation (adjacent-link globs + stand<->link0), and
watches the bridge's servo_state.v1 fanout (q_actual_deg both arms). On a
margin breach OR a stale state stream it TRIPS fail-closed: a UDP control
message latches the bridge (follow chunks dropped, fault_latched=true on the
fanout, so flow-infer's SafetyGate blocks).

Scope (operator decision 2026-08-16): PROXIMAL ONLY — arm links 0..5 vs
each other / the other arm / the stand. Everything distal of joint 6 (link6,
attachment site, pika gripper/fingers/tool) is EXCLUDED from the hard gate:
task contact there is delegated to CM's force controller (Admittance) — that
delegation requires the force path to be ARMED on real rollouts (P3 item;
the current follow overlay ships admittance_overlay off for pure replay).
No velocity barrier/braking — the CM controller owns motion; our action is
stream cutoff + latch. Margins: inter d_hard 0.025 m, intra-arm 0.005 m
(stack_real.yaml values).

Run (host):  .venv/bin/python cm_bridge/src/collision_monitor.py
"""
from __future__ import annotations

import argparse
import fnmatch
import json
import socket
import time

import numpy as np
import pinocchio as pin

URDF = "/home/plaif/workspace/mo_robot_descriptions/mo_robot_descriptions/robots/urdf/dual_rb3_730e/dual_rb3_730e_ver5.urdf"
PKG = "/home/plaif/workspace/mo_robot_descriptions/mo_robot_descriptions/robots/urdf/dual_rb3_730e"

DISABLED_GLOBS = [  # stack_real.yaml collision.mesh.disabled_collision_pairs
    ("*left*link0*", "*left*link1*"), ("*right*link0*", "*right*link1*"),
    ("*left*link0*", "*left*link2*"), ("*right*link0*", "*right*link2*"),
    ("*left*link4*", "*left*link6*"), ("*right*link4*", "*right*link6*"),
]
STAND_IGNORE_ARM_SUBSTRINGS = ["link0"]  # stand<->link0 structural
# Distal-of-joint-6 bodies: excluded from the hard collision gate entirely —
# contact at the tool side is force-control territory, not a latch condition.
DISTAL_SUBSTRINGS = ["link6", "attachment", "gripper", "finger", "tool"]
D_HARD_INTER = 0.025   # arm<->arm, arm<->stand [m]
D_HARD_INTRA = 0.005   # same-arm non-adjacent [m]
STALE_TRIP_S = 0.25


def arm_of(name):
    if "left" in name:
        return "left"
    if "right" in name:
        return "right"
    return "stand"


def build():
    model, cmodel, _ = pin.buildModelsFromUrdf(URDF, package_dirs=[PKG])
    cmodel.addAllCollisionPairs()
    keep, klass = [], []
    geoms = cmodel.geometryObjects
    for pr in cmodel.collisionPairs:
        a, b = geoms[pr.first], geoms[pr.second]
        na, nb = a.name, b.name
        if any(d in na or d in nb for d in DISTAL_SUBSTRINGS):
            continue  # distal of joint 6 -> force-control territory
        # adjacent links on the same joint chain: parentJoint distance <= 1
        if arm_of(na) == arm_of(nb) != "stand":
            if abs(int(a.parentJoint) - int(b.parentJoint)) <= 1:
                continue
        if any(
            (fnmatch.fnmatch(na, ga) and fnmatch.fnmatch(nb, gb))
            or (fnmatch.fnmatch(na, gb) and fnmatch.fnmatch(nb, ga))
            for ga, gb in DISABLED_GLOBS
        ):
            continue
        pair_arms = {arm_of(na), arm_of(nb)}
        if "stand" in pair_arms and any(
            s in na or s in nb for s in STAND_IGNORE_ARM_SUBSTRINGS
        ):
            continue
        keep.append(pr)
        klass.append("intra" if (arm_of(na) == arm_of(nb) != "stand") else "inter")
    cmodel.removeAllCollisionPairs()  # rebuild with curated set
    for pr in keep:
        cmodel.addCollisionPair(pr)
    print(f"[colmon] curated pairs: {len(keep)} "
          f"(intra {klass.count('intra')}, inter {klass.count('inter')})")
    return model, cmodel, klass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--state-port", type=int, default=50388)
    ap.add_argument("--bridge-control", default="127.0.0.1:50259")
    ap.add_argument("--rate-hz", type=float, default=50.0)
    ap.add_argument("--once", action="store_true",
                    help="single check of the current state, print verdict, exit")
    args = ap.parse_args()

    model, cmodel, klass = build()
    data, cdata = model.createData(), cmodel.createData()
    margins = np.array([D_HARD_INTRA if k == "intra" else D_HARD_INTER for k in klass])

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.bind(("127.0.0.1", args.state_port))
    rx.setblocking(False)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    chost, cport = args.bridge_control.split(":")

    def trip(reason):
        msg = {"cmd": "collision_trip", "reason": reason, "t": time.time()}
        tx.sendto(json.dumps(msg).encode(), (chost, int(cport)))
        print(f"[colmon] TRIP: {reason}")

    latest, t_last = None, time.monotonic()
    tripped = False
    period = 1.0 / args.rate_hz
    while True:
        # drain-then-read (stale-buffer trap, 2026-08-16)
        while True:
            try:
                latest_raw = rx.recvfrom(65535)[0]
                latest, t_last = latest_raw, time.monotonic()
            except BlockingIOError:
                break
        now = time.monotonic()
        if latest is None:
            time.sleep(period)
            continue
        if now - t_last > STALE_TRIP_S:
            if not tripped:
                trip(f"state stale {now - t_last:.2f}s")
                tripped = True
            time.sleep(period)
            continue
        try:
            st = json.loads(latest)
            ql = st["left"]["q_actual_deg"][:6]
            qr = st["right"]["q_actual_deg"][:6]
        except (KeyError, json.JSONDecodeError, TypeError):
            time.sleep(period)
            continue
        q = np.deg2rad(np.array(ql + qr, dtype=float))  # URDF order: left6, right6
        pin.computeDistances(model, data, cmodel, cdata, q)
        dists = np.array([r.min_distance for r in cdata.distanceResults])
        breach = dists < margins
        if breach.any():
            i = int(np.argmin(dists - margins))
            pr = cmodel.collisionPairs[i]
            na = cmodel.geometryObjects[pr.first].name
            nb = cmodel.geometryObjects[pr.second].name
            reason = f"{na}<->{nb} d={dists[i]*1000:.1f}mm < {margins[i]*1000:.0f}mm"
            if args.once:
                print(f"[colmon] BREACH {reason}")
                return 1
            if not tripped:
                trip(reason)
                tripped = True
        else:
            if args.once:
                print(f"[colmon] CLEAR min_margin={float((dists-margins).min())*1000:.1f}mm "
                      f"min_d={float(dists.min())*1000:.1f}mm")
                return 0
            tripped = False  # auto-clear only our local edge; bridge latch stays until reset
        time.sleep(period)


if __name__ == "__main__":
    raise SystemExit(main())
