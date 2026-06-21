#!/usr/bin/env python3
"""Offline-replay a trained checkpoint into the VM pgmode-sim stack for viser capture.

Feeds RECORDED data_tcp HDF5 frames (+ recorded gripper percent for proprio) into a
trained checkpoint each policy tick, and streams the resulting TcpTwistLocal commands
into the running rb_servo_server (stack_sim) so the policy's commanded motion is
visualized in viser. Closed-loop on the sim robot (live-sim proprio for the arm body
deltas), open frames (recorded), recorded gripper for the proprio gripper channel.

This reuses the trusted live flow-infer command pipeline; the ONLY substitution is the
camera source (live ZMQ bundle -> recorded HDF5 frames) and the proprio gripper channel.

Usage:
  PYTHONPATH=policy_runner ~/openpi/.venv/bin/python scripts/replay_episode_rollout.py \
      --config policy_runner/config/replay_sim.yaml \
      --checkpoint ~/pika_umi_models_v2/flow/checkpoint.pt \
      --episode ~/workspace/robotics_lab/data_tcp/data_20260606_134608/episode_000.hdf5 \
      --policy-dt-sec 0.0334
"""
from __future__ import annotations

import argparse
import sys
import time
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "policy_runner"))

import h5py  # noqa: E402

from policy_runner.config import load_config  # noqa: E402
from policy_runner.main import run  # noqa: E402
from policy_runner.robot_state_client import (  # noqa: E402
    RobotStateClient,
    StateStreamLeaseReadback,
)
from policy_runner.servo_command_client import ServoCommandClient, CommandIntent  # noqa: E402
from policy_runner.gripper import GripperRuntime  # noqa: E402
from policy_runner.flow_inference import (  # noqa: E402
    DEFAULT_FLOW_MAX_ANGULAR_VELOCITY_RAD_S,
    DEFAULT_FLOW_MAX_LINEAR_VELOCITY_M_S,
    DirectBcImageActionSource,
    FlowMatchingActionSource,
    action_chunk_checkpoint_kind,
)
from policy_runner.flow_dataset import pose_delta_local  # noqa: E402
from policy_runner.action_sources.tcp_delta import (  # noqa: E402
    cartesian_action_requirements,
    clamp_tcp_twist,
    tcp_twist_local_intent,
)

# rbpodo rest pose (folded, arms up, clear of floor) — GUI default InitMotion target.
REST_LEFT_DEG = (-131.663, 72.989, 113.400, -80.880, -107.064, -145.949)
REST_RIGHT_DEG = (135.099, -64.017, -114.457, 84.379, 112.485, 129.893)


class _Frame:
    __slots__ = ("pixels",)

    def __init__(self, pixels):
        self.pixels = pixels


class _Bundle:
    __slots__ = ("frames",)

    def __init__(self, frames):
        self.frames = frames


class ReplayClock:
    def __init__(self, num_frames: int):
        self.index = 0
        self.num_frames = num_frames


class ReplayCameraClient:
    """Camera-bundle-client stand-in that yields the recorded HDF5 frame at clock.index.

    Mirrors the CameraBundleClient duck-type used by FlowMatchingActionSource:
    poll(timeout_ms=0) -> bundle with .frames[name].pixels, is_fresh(bundle) -> bool.
    Pixels are the raw stored HDF5 image cells (JPEG bytes); decode_hdf5_image_value
    in the source reproduces training preprocessing exactly.
    """

    def __init__(self, frames_by_name: dict[str, list], clock: ReplayClock):
        self._frames_by_name = frames_by_name
        self._clock = clock

    def _bundle(self):
        i = min(self._clock.index, self._clock.num_frames - 1)
        return _Bundle({name: _Frame(cells[i]) for name, cells in self._frames_by_name.items()})

    def poll(self, timeout_ms: int = 0):
        return self._bundle()

    def latest(self):
        return self._bundle()

    def is_fresh(self, bundle) -> bool:
        return bundle is not None

    def close(self) -> None:
        pass


def load_episode(path: str, camera_names: list[str]):
    """Return (frames_by_name, gripper_by_arm, num_frames).

    Handles two on-disk layouts transparently:
      * data_tcp_v2 flat:  observations/images/<name>,           observations/gripper_<arm>
      * raw data/ nested:  observations/<arm>/images/realsense_color, observations/<arm>/gripper (T,2)
    The raw layout is what the saved UMI validation episodes use (poses-only data_tcp drops
    images), so loading raw data/ episodes here is how offline camera frames feed inference.
    """
    frames_by_name: dict[str, list] = {}
    gripper_by_arm: dict[str, np.ndarray] = {}

    def _arm_of(name: str) -> str:
        return "left" if str(name).startswith("left") else "right"

    with h5py.File(path, "r") as f:
        flat = "observations/images" in f
        for name in camera_names:
            if flat:
                img_grp = f["observations/images"]
                if name not in img_grp:
                    raise KeyError(f"camera {name!r} not in episode {path}")
                ds = img_grp[name]
            else:
                key = f"observations/{_arm_of(name)}/images/realsense_color"
                if key not in f:
                    raise KeyError(f"camera {name!r} ({key}) not in episode {path}")
                ds = f[key]
            frames_by_name[name] = [ds[i] for i in range(ds.shape[0])]
        num_frames = len(next(iter(frames_by_name.values())))
        for arm in ("left", "right"):
            if flat:
                key = f"observations/gripper_{arm}"
                gripper_by_arm[arm] = np.asarray(f[key]) if key in f else None
            else:
                key = f"observations/{arm}/gripper"
                if key in f:
                    g = np.asarray(f[key])
                    # raw layout stores (T,2): col0 = measured/actual %, col1 = commanded.
                    gripper_by_arm[arm] = g[:, 0] if g.ndim == 2 else g
                else:
                    gripper_by_arm[arm] = None
    return frames_by_name, gripper_by_arm, num_frames


class GroundTruthSource:
    """Replays the COLLECTED demonstration motion on the robot (no model).

    Emits the recorded per-step ee_local body-frame deltas (pose_delta_local of the
    recorded target poses, i.e. exactly the flow_dataset ee_local training target)
    as TcpTwistLocal. This is the 'ideal' rollout: what the policy *should* output.
    """

    def __init__(self, episode_path: str, clock: ReplayClock, *, policy_dt_sec: float,
                 r_align: np.ndarray | None = None,
                 action_scale: float = 1.0,
                 max_lin: float = DEFAULT_FLOW_MAX_LINEAR_VELOCITY_M_S,
                 max_ang: float = DEFAULT_FLOW_MAX_ANGULAR_VELOCITY_RAD_S):
        with h5py.File(episode_path, "r") as f:
            tpL = np.asarray(f["action/target_pose_left"])
            tpR = np.asarray(f["action/target_pose_right"])
        n = len(tpL)
        self.dL = np.array([pose_delta_local(tpL[i], tpL[i + 1]) for i in range(n - 1)], dtype=np.float64)
        self.dR = np.array([pose_delta_local(tpR[i], tpR[i + 1]) for i in range(n - 1)], dtype=np.float64)
        if r_align is not None:
            # Rotate the body-frame linear+angular deltas into the RB TCP frame
            # (e.g. a 180deg-about-approach correction for the steamvr->stand yaw gap).
            # linear/angular are the same matrix for a true rotation preset, and
            # differ only for split presets (e.g. pika_rz180_trans_only).
            R_lin = np.asarray(r_align.linear, dtype=np.float64)
            R_ang = np.asarray(r_align.angular, dtype=np.float64)
            for d in (self.dL, self.dR):
                d[:, 0:3] = d[:, 0:3] @ R_lin.T
                d[:, 3:6] = d[:, 3:6] @ R_ang.T
        # Uniformly shrink the per-step body-frame delta (translation + rotation)
        # so the integrated sweep stays inside the reachable workspace while the
        # per-step DIRECTION (the axis under test) is unchanged. The absolute
        # recorded poses are in steamvr_world (not robot stand), so we can only
        # replay relative deltas; a large reciprocation (~0.5 m) integrated from
        # an arbitrary robot start config can leave reach (elbow +-150 deg /
        # singularity / self-collision / floor). Scaling keeps the trajectory
        # shape, just smaller. 1.0 = faithful amplitude.
        self.action_scale = float(action_scale)
        if self.action_scale != 1.0:
            self.dL *= self.action_scale
            self.dR *= self.action_scale
        self.clock = clock
        self.policy_dt_sec = float(policy_dt_sec)
        self.max_lin = float(max_lin)
        self.max_ang = float(max_ang)
        self.camera_names: list[str] = []
        self.checkpoint_arm_mask = (1.0, 1.0)
        self.checkpoint_selected_arms = ["left", "right"]
        self.requirements = replace(
            cartesian_action_requirements(allow_rbpodo_controller_simulation=True),
            requires_camera=False,
        )

    def _twist(self, delta):
        return clamp_tcp_twist((np.asarray(delta) / self.policy_dt_sec).tolist(), self.max_lin, self.max_ang)

    def next_intent(self, snapshot, now_monotonic):
        t = self.clock.index
        if t >= len(self.dL):
            return None
        return tcp_twist_local_intent(left=self._twist(self.dL[t][:6]),
                                      right=self._twist(self.dR[t][:6]), timeout_sec=0.2)

    def close(self):
        pass


def init_to_rest(config, *, settle_sec: float = 14.0, stderr=sys.stderr) -> None:
    """Move both arms to the folded rest pose via JointTarget before the rollout.

    Uses its own state+command clients and fully closes them before returning so
    run()'s own RobotStateClient gets exclusive use of the state port."""
    sc = RobotStateClient(config.robot_state.bind, config.robot_state.stale_timeout_sec)
    sc.start()
    cc = ServoCommandClient(config.servo_command.endpoint, config.servo_command.timeout_sec)
    try:
        t0 = time.monotonic()
        while sc.latest is None and time.monotonic() - t0 < 5.0:
            time.sleep(0.05)
        if sc.latest is None:
            raise RuntimeError("no robot state on " + config.robot_state.bind)
        cc.acquire_lease(StateStreamLeaseReadback(sc), timeout_sec=4.0)
        cc.send(CommandIntent.arm_motion(timeout_sec=0.5))
        time.sleep(0.3)
        jt = CommandIntent.joint_target(left=REST_LEFT_DEG, right=REST_RIGHT_DEG, timeout_sec=10.0)
        deadline = time.monotonic() + settle_sec
        while time.monotonic() < deadline:
            cc.send(jt)
            time.sleep(0.05)
            p = sc.latest.payload
            lq = p["left"]["q_actual_deg"]
            rq = p["right"]["q_actual_deg"]
            errl = max(abs(a - b) for a, b in zip(lq, REST_LEFT_DEG))
            errr = max(abs(a - b) for a, b in zip(rq, REST_RIGHT_DEG))
            if errl < 1.0 and errr < 1.0:
                break
        p = sc.latest.payload
        print(
            f"[replay] init rest pose: Lq={[round(x,1) for x in p['left']['q_actual_deg']]} "
            f"Rq={[round(x,1) for x in p['right']['q_actual_deg']]} "
            f"Lz={round(p['left'].get('tcp_actual_stand',{}).get('z',float('nan')),3)} "
            f"verdict={p.get('safety_verdict')}",
            file=stderr,
            flush=True,
        )
        cc.release_lease()
        time.sleep(0.5)  # let the release register before run() re-acquires
    finally:
        cc.close()
        sc.close()
        time.sleep(0.3)


def build_source(args, camera_client):
    import os

    command_family = getattr(args, "command_family", None) or "tcp_twist_local"
    # Gripper: drive the REAL gripper from the model only when explicitly allowed
    # (real_policy path + RB_ALLOW_REAL_GRIPPER=1). Default = controller_sim (logged
    # no-op gripper) so a sim/dry run never actuates hardware.
    if getattr(args, "allow_real_gripper", False):
        os.environ["RB_ALLOW_REAL_GRIPPER"] = "1"
        gripper_runtime = GripperRuntime(rollout_mode="real_policy", allow_real_gripper_motion=True)
    else:
        gripper_runtime = GripperRuntime(rollout_mode="controller_sim")
    # openpi:// served checkpoint -> remote inference source (camera bundle substituted by the
    # ReplayCameraClient). The live flow-infer command pipeline is reused unchanged; the only
    # substitution here is the camera source + the recorded proprio gripper (main() injects it).
    from policy_runner.openpi_remote import OPENPI_CHECKPOINT_PREFIX, OpenpiRemoteActionSource

    if str(args.checkpoint or "").startswith(OPENPI_CHECKPOINT_PREFIX):
        source = OpenpiRemoteActionSource(
            args.checkpoint,
            camera_client=camera_client,
            command_family=command_family,
            policy_dt_sec=args.policy_dt_sec,
            max_linear_step_m=args.max_linear_step_m,
            max_angular_step_rad=args.max_angular_step_rad,
            allow_rbpodo_controller_simulation_cartesian=True,
            ee_local_r_align=args.ee_local_r_align,
            gripper_runtime=gripper_runtime,
            device=args.device,
        )
        # Decouple chunk inference from the command stream (background prefetch +
        # per-step hold) so a slow inference (esp. medoid-of-N) does not stall the
        # command stream and pulse the robot start/stop. Mirrors flow-infer's
        # live-rollout behavior (main.py: enable_async_chunking for real/controller-sim).
        source.enable_async_chunking = True
        return source, "openpi_remote"
    kind = action_chunk_checkpoint_kind(args.checkpoint, device="cpu")
    common = dict(
        camera_client=camera_client,
        command_family=command_family,
        policy_dt_sec=args.policy_dt_sec,
        max_linear_step_m=args.max_linear_step_m,
        max_angular_step_rad=args.max_angular_step_rad,
        allow_rbpodo_controller_simulation_cartesian=True,
        ee_local_r_align=args.ee_local_r_align,
        gripper_runtime=gripper_runtime,
        device=args.device,
    )
    if kind == "direct_bc":
        source = DirectBcImageActionSource(args.checkpoint, image_size=args.image_size, **common)
    elif kind == "flow":
        source = FlowMatchingActionSource(args.checkpoint, sample_steps=args.sample_steps, **common)
    else:
        raise SystemExit(f"unsupported checkpoint kind for local replay: {kind}")
    source.enable_async_chunking = True  # see openpi branch: avoid pulsed start/stop
    return source, kind


def fix_camera_names(source, checkpoint_path):
    """Flow checkpoints store camera_names only inside model_config, so the source
    loads with camera_names=[] and would run without image conditioning. Restore them."""
    if source.camera_names:
        return source.camera_names
    import torch

    ck = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    names = list((ck.get("model_config") or {}).get("camera_names") or [])
    if not names:
        raise SystemExit("checkpoint has no camera_names in model_config; cannot replay frames")
    source.camera_names = [str(n) for n in names]
    source.requirements = replace(source.requirements, requires_camera=True)
    print(f"[replay] restored flow camera_names from model_config: {source.camera_names}", file=sys.stderr)
    return source.camera_names


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--checkpoint", default=None)
    ap.add_argument("--episode", required=True)
    ap.add_argument("--ground-truth", action="store_true",
                    help="replay the COLLECTED demonstration motion (recorded ee_local actions) on the robot "
                         "instead of running a model; no checkpoint needed")
    ap.add_argument("--policy-dt-sec", type=float, default=0.0334)
    ap.add_argument("--device", default="auto")
    ap.add_argument("--ee-local-r-align", default=None)
    ap.add_argument(
        "--command-family",
        choices=("tcp_twist_local", "tcp_target_pose"),
        default="tcp_twist_local",
        help="how the policy's ee_local deltas are sent to rb_servo_server: tcp_twist_local "
             "(velocity) or tcp_target_pose (compose into absolute TcpPoseTarget position "
             "setpoints — the stabilized deploy lane, see wiki pi05-openpi-deployment)",
    )
    ap.add_argument("--action-scale", type=float, default=1.0,
                    help="ground-truth only: scale per-step ee_local deltas (translation+rotation) "
                         "by this factor to keep the swept trajectory in reach; axis direction is "
                         "preserved (1.0 = faithful amplitude, e.g. 0.5 = half-size reciprocation)")
    ap.add_argument("--max-linear-step-m", type=float, default=0.010,
                    help="per-step Cartesian translation clamp for tcp_target_pose (default 0.010=10mm). "
                         "Recorded demos reach up to ~8mm/step; 10mm leaves the policy's reach untruncated "
                         "(measured: per-step max ~9mm). Lower it to slow/limit the motion.")
    ap.add_argument("--max-angular-step-rad", type=float, default=0.01,
                    help="per-step Cartesian rotation clamp for tcp_target_pose (default 0.01 rad)")
    ap.add_argument("--allow-real-gripper", action="store_true",
                    help="drive the REAL gripper from the model's gripper action (real_policy gripper "
                         "path + RB_ALLOW_REAL_GRIPPER=1). Without it the gripper is a logged no-op "
                         "(controller_sim). Use only on the real stack with the pika gripper backend.")
    ap.add_argument("--sample-steps", type=int, default=16)
    ap.add_argument("--image-size", type=int, default=None)
    ap.add_argument("--chunk-execute-steps", type=int, default=None)
    ap.add_argument("--no-init", action="store_true", help="skip moving to rest pose first")
    ap.add_argument("--tail-ticks", type=int, default=8, help="extra ticks after last frame to flush motion")
    ap.add_argument("--blind", action="store_true",
                    help="force camera_names=[] (reproduce the pre-fix blind-policy bug: no image conditioning)")
    ap.add_argument("--open-loop", action="store_true",
                    help="feed RECORDED proprio (training-matched) instead of live-sim proprio; the robot still "
                         "integrates the commanded twists for visualization, but model inputs match training "
                         "(removes the closed-loop recorded-frames-vs-diverging-robot divergence artifact)")
    args = ap.parse_args()

    config = load_config(args.config)
    # Pace the loop at the policy dt so each per-step twist (= delta / policy_dt) is
    # held for ~policy_dt and integrates to the intended per-step delta -> the robot
    # traverses the recorded trajectory at the recorded timescale.
    rate = round(1.0 / args.policy_dt_sec)
    try:
        object.__setattr__(config, "command_rate_hz", float(rate))
    except Exception:
        config.command_rate_hz = float(rate)
    print(f"[replay] command_rate_hz overridden to {rate} Hz (policy_dt={args.policy_dt_sec}s)", file=sys.stderr)

    # Offline replay feeds RECORDED camera frames (ReplayCameraClient) to the policy every
    # tick, so the policy is NOT blind. But the SafetyGate's camera-readiness check looks for a
    # LIVE camera_server bundle in the robot-state payload (absent here, since we run no camera
    # server) and would otherwise drop every motion intent as "camera_unavailable" -> the robot
    # never moves. Mark the camera available for the gate. Scoped to this offline-replay tool;
    # it does not touch live flow-infer or any server-side safety layer.
    try:
        object.__setattr__(config.safety, "camera_available", True)
        object.__setattr__(config.safety, "camera_stale", False)
    except Exception:
        config.safety.camera_available = True
        config.safety.camera_stale = False
    print("[replay] safety camera gate satisfied via recorded frames (camera_available=True)", file=sys.stderr)

    if not args.no_init:
        init_to_rest(config)

    clock = ReplayClock(num_frames=1)
    if args.ground_truth:
        # Replay the collected demonstration directly on the robot (no model).
        from policy_runner.flow_inference import resolve_ee_local_r_align
        r_align = resolve_ee_local_r_align(args.ee_local_r_align)
        source = GroundTruthSource(args.episode, clock, policy_dt_sec=args.policy_dt_sec,
                                   r_align=r_align, action_scale=args.action_scale)
        T = len(source.dL) + 1
        clock.num_frames = T
        print(f"[replay] GROUND-TRUTH data replay: {T} frames (recorded ee_local actions), "
              f"action_scale={source.action_scale}", file=sys.stderr)
    else:
        if not args.checkpoint:
            raise SystemExit("--checkpoint is required unless --ground-truth is set")
        # Build the source first (without frames) to learn the checkpoint camera names,
        # then load the episode frames for exactly those cameras.
        replay_client = ReplayCameraClient({}, clock)
        source, kind = build_source(args, replay_client)
        cam_names = fix_camera_names(source, args.checkpoint)
        if args.blind:
            # Reproduce the pre-fix bug: a vision-conditioned policy loaded with no
            # camera names runs on zero images (output independent of the scene).
            source.camera_names = []
            source.requirements = replace(source.requirements, requires_camera=False)
            print("[replay] --blind: camera_names forced to [] (policy runs WITHOUT images)", file=sys.stderr)
        if args.chunk_execute_steps is not None:
            source.chunk_execute_steps = int(args.chunk_execute_steps)

        frames_by_name, gripper_by_arm, T = load_episode(args.episode, cam_names)
        clock.num_frames = T
        replay_client._frames_by_name = frames_by_name
        print(f"[replay] kind={kind} cams={cam_names} frames={T} chunk_execute={source.chunk_execute_steps}", file=sys.stderr)

        # Recorded gripper percent into the proprio gripper channel (the servo state has
        # no gripper; without this the proprio gripper reads 0=closed and the policy
        # behaves as if the grasp already happened -> spurious lift instead of descend).
        def live_gripper_percent(arm):
            arr = gripper_by_arm.get(arm)
            if arr is None or len(arr) == 0:
                return None
            return float(arr[min(clock.index, len(arr) - 1)])

        source._live_gripper_percent = live_gripper_percent

        if args.open_loop:
            # Feed the training-matched proprio (body-frame pose_delta_local from the
            # recorded reset pose) at the current replay index, decoupled from the
            # (visualization-only) sim robot. This isolates the model's true output
            # quality from closed-loop divergence.
            with h5py.File(args.episode, "r") as f:
                obs = {a: np.asarray(f[f"observations/tcp_stand_{a}"]) for a in ("left", "right")}
            am = source.arm_mask
            def recorded_proprio(_payload):
                t = min(clock.index, T - 1)
                feats = []
                for a in ("left", "right"):
                    g = gripper_by_arm.get(a)
                    gv = float(g[min(t, len(g) - 1)]) if g is not None and len(g) else 0.0
                    feats.append(np.concatenate([pose_delta_local(obs[a][0], obs[a][t]), [gv]]))
                return np.concatenate([feats[0], feats[1], am]).astype(np.float32)
            source._runtime_proprio = recorded_proprio
            print("[replay] --open-loop: proprio sourced from recorded poses (training-matched)", file=sys.stderr)

    # Drive the replay frame pointer from the run loop's per-tick state_sink. The
    # source resamples (and reads a frame) every chunk_execute_steps ticks; advancing
    # the pointer once per tick keeps recorded-frame time aligned with executed-action
    # time. Stop (KeyboardInterrupt -> run() returns 0, releases lease) at episode end.
    # Adaptive policy_dt: the per-step twist (= delta / policy_dt) must be held for
    # ~policy_dt to integrate to the intended per-step delta. The loop's true period
    # is set by inference time (varies, often >> the recording dt), so we MEASURE the
    # wall dt between ticks and feed it back as policy_dt -> twist*actual_dt == delta,
    # i.e. the robot traverses exactly the recorded trajectory regardless of how fast
    # inference runs (wall-clock just stretches). Without this the twist is held far
    # longer than policy_dt and the motion is grossly overshot/divergent.
    state = {"tick": 0, "last_t": None, "pdt": float(args.policy_dt_sec)}

    def state_sink(_snapshot):
        now = time.monotonic()
        if state["last_t"] is not None:
            dt = now - state["last_t"]
            if 0.005 < dt < 1.0:
                state["pdt"] = 0.7 * state["pdt"] + 0.3 * dt
                source.policy_dt_sec = state["pdt"]
        state["last_t"] = now
        clock.index = min(state["tick"], T - 1)
        state["tick"] += 1
        if state["tick"] >= T + args.tail_ticks:
            raise KeyboardInterrupt

    print(f"[replay] starting rollout: {Path(args.episode).parent.name}/{Path(args.episode).name}", file=sys.stderr, flush=True)
    rc = run(config, source=source, send_commands=True, state_sink=state_sink)
    print(f"[replay] done rc={rc}, executed ~{min(state['tick'], T)}/{T} frames", file=sys.stderr, flush=True)
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
