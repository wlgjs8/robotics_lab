"""Remote openpi (pi0.5) policy server as a policy_runner action source.

Bridges a running `openpi serve_policy.py` websocket server (pi05_pika_umi config)
into the standard flow-infer runtime so the pi0.5 checkpoint goes through the SAME
camera polling, reset anchoring, TcpTwistLocal emission, clamps, rollout-mode gates
and gripper runtime as the in-house checkpoints.

Usage (after starting the server):
    .venv/bin/python scripts/serve_policy.py policy:checkpoint \
        --policy.config=pi05_pika_umi --policy.dir=~/pika_umi_models_v2/pi05_v2_20k
    python3 -m policy_runner flow-infer \
        --checkpoint "openpi://127.0.0.1:8000" \
        --config <runtime yaml> --rollout-mode sim_dryrun

Contract with the server (see pika_umi_policy.PikaUmiInputs/Outputs):
    obs  : observation/{left,right}_wrist_0_rgb (HWC uint8 RGB, full camera res),
           observation/state (14 = [pos(3), rotvec(3), grip/100] x left,right,
           ABSOLUTE stand-frame), prompt (fixed task sentence)
    out  : actions (H,14) per-step ee_local deltas; gripper dims (6,13) in /100
           units -> converted here to percent deltas (in-house chunk convention).

Smoke testing without cameras: set OPENPI_REMOTE_FAKE_IMAGES=1 to feed zero images
(policy output is meaningless but the full pipe can be exercised in sim_dryrun).

Requires `openpi_client` (and cv2 for live camera decode) in the runtime python
environment — e.g. run policy_runner inside the openpi .venv.
"""

from __future__ import annotations

import os
import sys
from dataclasses import replace
from typing import Any, TextIO

import numpy as np

from .action_sources.tcp_delta import cartesian_action_requirements
from .camera_bundle_client import resolve_frame
from .flow_dataset import pose_from_state_payload
from .flow_inference import (
    DEFAULT_FLOW_MAX_ANGULAR_VELOCITY_RAD_S,
    DEFAULT_FLOW_MAX_LINEAR_VELOCITY_M_S,
    FlowMatchingActionSource,
    _gripper_value_from_payload,
    canonical_flow_command_family,
    default_action_log_path,
    normalize_flow_command_family,
    resolve_ee_local_r_align,
    rotate_flow_arm_vectors,
)
from .gripper import GripperRuntime
from .tcp_target_pose_conditioner import CONDITIONING_MODES, REANCHOR_MODES

OPENPI_CHECKPOINT_PREFIX = "openpi://"
OPENPI_DEFAULT_PROMPT = (
    "pick up the black bolt with the right arm and put it in the right box, "
    "then pick up the gray bolt with the left arm and put it in the left box"
)
_GRIP_DIMS = (6, 13)
_FAKE_IMAGES_ENV = "OPENPI_REMOTE_FAKE_IMAGES"
_SKIP_WARMUP_ENV = "OPENPI_REMOTE_SKIP_WARMUP"


class _OpenpiWebsocketClient:
    """Minimal openpi policy-server client (same wire protocol as
    openpi_client.WebsocketClientPolicy) with keepalive pings DISABLED so the
    server's first-inference warmup (torch compile/autotune, minutes) does not
    kill the connection, plus reconnect-on-failure."""

    def __init__(self, uri: str):
        import websockets.sync.client  # lazy: only needed for the remote source
        from openpi_client import msgpack_numpy

        self._connect = websockets.sync.client.connect
        self._msgpack = msgpack_numpy
        self._packer = msgpack_numpy.Packer()
        self._uri = uri
        self._conn = None

    def _ensure_connection(self) -> None:
        if self._conn is not None:
            return
        conn = self._connect(
            self._uri,
            compression=None,
            max_size=None,
            ping_interval=None,  # warmup can block the server for minutes
            close_timeout=5,
        )
        self._msgpack.unpackb(conn.recv())  # server metadata handshake
        self._conn = conn

    def close(self) -> None:
        if self._conn is not None:
            try:
                self._conn.close()
            except Exception:  # noqa: BLE001 - best effort
                pass
            self._conn = None

    def infer(self, obs: dict) -> dict:
        last_error: Exception | None = None
        for _attempt in range(2):  # one reconnect retry
            try:
                self._ensure_connection()
                self._conn.send(self._packer.pack(obs))
                response = self._conn.recv()
                if isinstance(response, str):
                    raise RuntimeError(f"inference server error: {response}")
                return self._msgpack.unpackb(response)
            except Exception as exc:  # noqa: BLE001 - reconnect once, then surface
                last_error = exc
                self.close()
        raise last_error  # type: ignore[misc]


def _quat_xyzw_to_rotvec(q: np.ndarray) -> np.ndarray:
    """Quaternion (x,y,z,w) -> rotation vector (axis * angle), numpy only."""
    q = np.asarray(q, dtype=np.float64)
    norm = float(np.linalg.norm(q))
    if norm <= 0.0:
        return np.zeros(3, dtype=np.float64)
    x, y, z, w = q / norm
    sin_half = float(np.linalg.norm((x, y, z)))
    if sin_half < 1e-12:
        return np.zeros(3, dtype=np.float64)
    angle = 2.0 * float(np.arctan2(sin_half, w))
    if angle > np.pi:
        angle -= 2.0 * np.pi
    return np.asarray((x, y, z), dtype=np.float64) / sin_half * angle


def _gripper_from_arm_payload(value: Any) -> float:
    arm = value if isinstance(value, dict) else {}
    for key in ("gripper", "gripper_position"):
        raw = arm.get(key)
        if isinstance(raw, dict):
            for nested_key in ("position", "value", "percent"):
                nested = raw.get(nested_key)
                if isinstance(nested, (int, float)):
                    return float(nested)
        elif isinstance(raw, (int, float)):
            return float(raw)
    return 0.0


class OpenpiRemoteActionSource(FlowMatchingActionSource):
    """flow-infer action source backed by a remote openpi policy server."""

    def __init__(  # noqa: PLR0913 - mirrors FlowMatchingActionSource construction surface.
        self,
        server_url: str,
        *,
        timeout_sec: float = 0.2,
        camera_client: Any | None = None,
        command_family: str | None = None,
        policy_dt_sec: float | None = None,
        max_linear_velocity_m_s: float | None = None,
        max_angular_velocity_rad_s: float | None = None,
        max_linear_step_m: float = 0.002,
        max_angular_step_rad: float = 0.01,
        chunk_execute_steps: int | None = None,
        chunk_crossfade_steps: int = 0,
        tcp_target_pose_conditioning: str = "legacy_step_hold",
        tcp_target_pose_reanchor_mode: str = "measured_blend",
        tcp_target_pose_blend_steps: int = 2,
        tcp_target_pose_send_twist: bool = False,
        allow_rbpodo_controller_simulation_cartesian: bool = False,
        gripper_runtime: GripperRuntime | None = None,
        ee_local_r_align: Any = None,
        prompt: str = OPENPI_DEFAULT_PROMPT,
        action_horizon: int = 16,
        camera_names: tuple[str, str] = ("left_realsense_color", "right_realsense_color"),
        sample_steps: int = 1,  # accepted for CLI symmetry; unused remotely
        device: str = "remote",  # accepted for CLI symmetry; inference is server-side
        stderr: TextIO = sys.stderr,
    ):
        # Deliberately NOT calling super().__init__: there is no local checkpoint to
        # load. Every attribute the inherited runtime methods touch is set below.
        url = server_url[len(OPENPI_CHECKPOINT_PREFIX):] if server_url.startswith(OPENPI_CHECKPOINT_PREFIX) else server_url
        host, _, port = url.partition(":")
        try:
            self._client = _OpenpiWebsocketClient(f"ws://{host or '127.0.0.1'}:{int(port or 8000)}")
        except ImportError as exc:  # pragma: no cover - environment guard
            raise RuntimeError(
                "openpi_client/websockets are required for OpenpiRemoteActionSource; run "
                "policy_runner inside the openpi virtualenv (see ~/pika_umi_models_v2/README_v2.md)"
            ) from exc
        self.prompt = str(prompt)
        self._fake_images = os.environ.get(_FAKE_IMAGES_ENV, "") == "1"

        self.timeout_sec = float(timeout_sec)
        self.camera_client = camera_client
        self.sample_steps = int(sample_steps)
        self.max_linear_step_m = float(max_linear_step_m)
        self.max_angular_step_rad = float(max_angular_step_rad)
        self.policy_label = "openpi remote policy"
        self.gripper_command_source = "openpi_remote_policy"
        # Interpret the action gripper dim as an ABSOLUTE next-step opening percent
        # (default; the latest openpi `--gripper-mode absolute` checkpoints) rather
        # than a per-step delta. The server returns it in /100 units; _sample_chunk
        # scales it to percent below, then _integrate_gripper_targets /
        # _dispatch_gripper_step command it directly. main.py overrides this from
        # the `--gripper-action-mode` flag; the default keeps direct construction
        # (e.g. tests) consistent with the base FlowMatchingActionSource default.
        self.gripper_action_absolute = True
        self.stderr = stderr
        self.device = device
        self.stats: dict[str, Any] = {}
        self.action_frame = "ee_local"
        self.ee_local_r_align = resolve_ee_local_r_align(ee_local_r_align)
        self.command_family_option = (
            normalize_flow_command_family(command_family) if command_family else "tcp_twist_local"
        )
        self.command_family = canonical_flow_command_family(self.command_family_option)
        # Fake-image smoke mode runs camera-less so the runtime does not gate on frames.
        self.camera_names = [] if self._fake_images else [str(name) for name in camera_names]
        self.image_size = 224  # zero-fallback shape only; live frames are sent full-res
        self.action_horizon = int(action_horizon)
        default_execute = max(1, self.action_horizon // 2)
        self.chunk_execute_steps = int(chunk_execute_steps) if chunk_execute_steps else default_execute
        self.chunk_execute_steps = max(1, min(self.chunk_execute_steps, self.action_horizon))
        self.policy_dt_sec = float(policy_dt_sec) if policy_dt_sec else (1.0 / 30.0)
        self.max_linear_velocity_m_s = (
            float(max_linear_velocity_m_s) if max_linear_velocity_m_s is not None else DEFAULT_FLOW_MAX_LINEAR_VELOCITY_M_S
        )
        self.max_angular_velocity_rad_s = (
            float(max_angular_velocity_rad_s)
            if max_angular_velocity_rad_s is not None
            else DEFAULT_FLOW_MAX_ANGULAR_VELOCITY_RAD_S
        )
        self.model = None  # inference happens on the server
        self.arm_mask = np.asarray([1.0, 1.0], dtype=np.float32)
        self.checkpoint_arm_mask = (1.0, 1.0)
        self.checkpoint_selected_arms = ["left", "right"]
        self.checkpoint_has_nonzero_gripper_commands = True
        self.gripper_runtime = gripper_runtime or GripperRuntime(rollout_mode="sim_dryrun")
        self.requirements = replace(
            cartesian_action_requirements(
                allow_rbpodo_controller_simulation=allow_rbpodo_controller_simulation_cartesian,
            ),
            requires_camera=bool(self.camera_names),
        )
        self._reset_left_pose: np.ndarray | None = None
        self._reset_right_pose: np.ndarray | None = None
        self._chunk: np.ndarray | None = None
        self._chunk_index = 0
        self._warned_missing_camera_client = False
        self.last_image_decode_count = 0
        self.last_missing_camera_count = 0
        self.image_decode_count = 0
        self.missing_camera_count = 0
        self._last_nonzero_twist_by_arm = {"left": False, "right": False}
        # Chunk-boundary twist crossfade state (mirrors FlowMatchingActionSource;
        # this class skips super().__init__). 0 = off.
        self._chunk_crossfade_steps = int(chunk_crossfade_steps)
        self._steps_since_boundary = 0
        self._prev_emitted_twist_by_arm: dict[str, tuple[float, ...] | None] = {
            "left": None,
            "right": None,
        }
        self._target_pose_by_arm: dict[str, np.ndarray | None] = {"left": None, "right": None}
        self._gripper_targets_by_arm: dict[str, float | None] = {"left": None, "right": None}
        # Patch 3: online tcp_target_pose A-stage conditioning (this class skips
        # super().__init__, so the state the inherited foh helpers touch is set here).
        if str(tcp_target_pose_conditioning) not in CONDITIONING_MODES:
            raise ValueError(f"tcp_target_pose_conditioning must be one of {CONDITIONING_MODES}")
        if str(tcp_target_pose_reanchor_mode) not in REANCHOR_MODES:
            raise ValueError(f"tcp_target_pose_reanchor_mode must be one of {REANCHOR_MODES}")
        self._tcp_tp_mode = str(tcp_target_pose_conditioning)
        self._tcp_tp_reanchor_mode = str(tcp_target_pose_reanchor_mode)
        self._tcp_tp_blend_steps = int(tcp_target_pose_blend_steps)
        self._tcp_tp_send_twist = bool(tcp_target_pose_send_twist)
        self._tcp_tp_conditioners = None
        self._tcp_tp_chunk_seq = 0
        self._current_gripper_targets: dict[str, float | None] = {"left": None, "right": None}
        # Per-policy-step action logger (env-gated, debug only). Mirrors
        # FlowMatchingActionSource; this class skips super().__init__, so the
        # attributes the inherited _log_action_step touches must be set here.
        # Set POLICY_RUNNER_ACTION_LOG=/path/to/actions.jsonl to capture one JSON
        # line per executed policy step (raw flow delta, sent twist, chunk index).
        self._action_log: TextIO | None = None
        self._action_log_seq = 0
        _action_log_path = default_action_log_path()
        self._action_log = open(_action_log_path, "w", buffering=1)
        print(f"[flow-infer] logging per-step actions to {_action_log_path}", file=self.stderr)

        # The server's first inference triggers torch compile/kernel autotune and can
        # take minutes; absorb that at startup so the control loop never stalls.
        if os.environ.get(_SKIP_WARMUP_ENV, "") != "1":
            self._warmup()

    def _warmup(self) -> None:
        import time

        zero = np.zeros((480, 640, 3), dtype=np.uint8)
        obs = {
            "observation/left_wrist_0_rgb": zero,
            "observation/right_wrist_0_rgb": zero,
            "observation/state": np.zeros(14, dtype=np.float32),
            "prompt": self.prompt,
        }
        print("openpi remote: warmup inference (server compile may take minutes)...", file=self.stderr, flush=True)
        started = time.perf_counter()
        try:
            self._client.infer(obs)
            self._client.infer(obs)  # second call absorbs the secondary compile path (~10s)
        except Exception as exc:  # noqa: BLE001 - warmup failure is not fatal; runtime retries
            print(f"openpi remote: warmup failed ({type(exc).__name__}: {exc}); continuing", file=self.stderr, flush=True)
            return
        print(f"openpi remote: warmup done in {time.perf_counter() - started:.1f}s", file=self.stderr, flush=True)

    # ------------------------------------------------------------------ obs --
    def _proprio_state(self, payload: dict[str, Any]) -> np.ndarray:
        """14-dim RESET-RELATIVE state (pi05 reset-relative retrain convention).

        Per arm: the current pose relative to the rollout reset pose, expressed
        in the reset body frame -- pos_rel(3), rotvec_rel(3) -- then gripper
        percent/100. Reset-relative cancels the absolute capture-world frame, the
        gap that made the v2 absolute-pose checkpoint fail on the robot (live
        stand-frame proprio sat ~2.7 m / z-score ~28 outside the training
        distribution). Mirrors the in-house flow reset-relative proprio. The
        reset anchor is latched on the first state this rollout sees, and MUST
        match the episode-first-frame anchor used when building the training
        dataset (examples/pika_umi/convert_pika_umi_data_to_lerobot.py)."""
        from scipy.spatial.transform import Rotation

        features: list[np.ndarray] = []
        for side, reset_attr in (("left", "_reset_left_pose"), ("right", "_reset_right_pose")):
            pose = np.asarray(pose_from_state_payload(payload, side), dtype=np.float64)
            reset = getattr(self, reset_attr)
            if reset is None:
                reset = pose.copy()
                setattr(self, reset_attr, reset)
            r_reset = Rotation.from_quat(reset[3:7])
            r_cur = Rotation.from_quat(pose[3:7])
            pos_rel = r_reset.inv().apply(pose[:3] - reset[:3])
            rot_rel = (r_reset.inv() * r_cur).as_rotvec()
            # Proprio gripper: the servo state carries no gripper channel, so
            # prefer the live physical motor percent, then the integrated policy
            # target. Absolute percent, NOT reset-relative (matches training).
            # Without this the channel reads 0 (= fully closed) and the policy
            # drives the gripper command away monotonically (right-arm runaway).
            grip = _gripper_value_from_payload(payload, side)
            if grip is None:
                live = self._live_gripper_percent(side)
                grip = live if live is not None else _gripper_from_arm_payload(payload.get(side, {}))
            features.append(np.concatenate([pos_rel, rot_rel, [float(grip) / 100.0]]))
        state = np.concatenate(features).astype(np.float32)
        if self.ee_local_r_align is not None:
            # Mirror FlowMatchingActionSource._runtime_proprio: the reset-relative
            # body vectors are in the RB TCP frame, but an ee_local checkpoint was
            # trained in the EE (pika tip) frame -> v_tip = R_align . v_tcp. The
            # inherited next_intent / _sample_and_align_chunk already convert the
            # output back with R_align.T; without this the input/output frames are
            # asymmetric (input left in TCP frame while output is rotated to tip).
            state = rotate_flow_arm_vectors(state, self.ee_local_r_align)
        return state

    def _raw_camera_images(self) -> tuple[dict[str, np.ndarray] | None, int, int]:
        """Full-resolution HWC uint8 RGB frames keyed left/right; None if missing."""
        if self._fake_images:
            zero = np.zeros((480, 640, 3), dtype=np.uint8)
            return {"left": zero, "right": zero.copy()}, 2, 0
        bundle = self._poll_camera_bundle()
        bundle_frames = getattr(bundle, "frames", {}) if bundle is not None else {}
        try:
            import cv2
        except ImportError as exc:  # pragma: no cover - environment guard
            raise RuntimeError("cv2 is required to decode live camera frames for openpi") from exc
        images: dict[str, np.ndarray] = {}
        decode_count = 0
        missing_count = 0
        for key, camera_name in (("left", self.camera_names[0]), ("right", self.camera_names[1])):
            # Bundle frames are keyed '<camera>.<stream>' (e.g. 'left_realsense.color')
            # while checkpoint camera names use '<camera>_<stream>'
            # ('left_realsense_color'); resolve_frame bridges the two. A plain
            # .get() here silently missed every frame (camera fail-closed -> no motion).
            frame = resolve_frame(bundle_frames, camera_name)
            pixels = getattr(frame, "pixels", None)
            if pixels is None:
                missing_count += 1
                continue
            array = np.asarray(pixels)
            if array.ndim == 3 and array.shape[-1] == 3:
                rgb = array.astype(np.uint8)  # already decoded HWC
            else:
                bgr = cv2.imdecode(array.astype(np.uint8), cv2.IMREAD_COLOR)
                if bgr is None:
                    missing_count += 1
                    continue
                rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            images[key] = rgb
            decode_count += 1
        if missing_count > 0 or len(images) < 2:
            return None, decode_count, max(missing_count, 2 - len(images))
        return images, decode_count, 0

    # ---------------------------------------------------------------- infer --
    def _sample_chunk(self, payload: dict[str, Any]) -> np.ndarray | None:
        images, decode_count, missing_count = self._raw_camera_images()
        self.last_image_decode_count = decode_count
        self.last_missing_camera_count = missing_count
        self.image_decode_count += decode_count
        self.missing_camera_count += missing_count
        if images is None:
            return None  # fail-closed without frames, same as the in-house sources
        obs = {
            "observation/left_wrist_0_rgb": images["left"],
            "observation/right_wrist_0_rgb": images["right"],
            "observation/state": self._proprio_state(payload),
            "prompt": self.prompt,
        }
        try:
            result = self._client.infer(obs)
        except Exception as exc:  # noqa: BLE001 - remote failure must not crash the loop
            print(f"openpi remote inference failed: {type(exc).__name__}: {exc}", file=self.stderr, flush=True)
            return None
        chunk = np.asarray(result.get("actions"), dtype=np.float32)
        if chunk.ndim != 2 or chunk.shape[1] < 14:
            print(f"openpi remote returned unexpected action shape {chunk.shape}", file=self.stderr, flush=True)
            return None
        chunk = chunk[:, :14].copy()
        # The server emits the gripper dim in /100 units (opening fraction) for BOTH
        # the legacy delta convention `(target-current)/100` and the absolute
        # `--gripper-mode absolute` convention `grip/100`. Scale to percent here; the
        # delta-vs-absolute interpretation is handled downstream by
        # gripper_action_absolute (_integrate_gripper_targets / _dispatch_gripper_step).
        chunk[:, _GRIP_DIMS[0]] *= 100.0
        chunk[:, _GRIP_DIMS[1]] *= 100.0
        return chunk[: self.action_horizon]
