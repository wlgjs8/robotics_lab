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
import time
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
_LEGACY_OPENPI_ACTION_HORIZON = 16


def _action_horizon_from_metadata(metadata: Any) -> int | None:
    if not isinstance(metadata, dict):
        return None
    raw = metadata.get("action_horizon")
    if raw is None:
        return None
    horizon = int(raw)
    if horizon <= 0:
        raise ValueError(f"openpi server metadata action_horizon must be positive, got {raw!r}")
    return horizon


def _resolve_openpi_action_horizon(action_horizon: int | None, metadata: Any) -> int:
    metadata_horizon = _action_horizon_from_metadata(metadata)
    if action_horizon is not None:
        resolved = int(action_horizon)
        if resolved <= 0:
            raise ValueError("openpi action_horizon must be positive")
        if metadata_horizon is not None and metadata_horizon != resolved:
            raise ValueError(
                "openpi action_horizon mismatch: "
                f"CLI requested {resolved}, server metadata reports {metadata_horizon}"
            )
        return resolved
    if metadata_horizon is not None:
        return metadata_horizon
    return _LEGACY_OPENPI_ACTION_HORIZON


def _resolve_openpi_chunk_execute_steps(value: int | None, action_horizon: int) -> int:
    if action_horizon <= 0:
        raise ValueError("openpi action_horizon must be positive")
    if value is None:
        return max(1, int(action_horizon) // 2)
    resolved = int(value)
    if resolved <= 0:
        raise ValueError("chunk_execute_steps must be positive")
    if resolved > int(action_horizon):
        raise ValueError(
            "chunk_execute_steps must not exceed openpi action_horizon "
            f"({resolved} > {int(action_horizon)})"
        )
    return resolved


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
        self._metadata: dict[str, Any] = {}

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
        try:
            metadata = self._msgpack.unpackb(conn.recv())  # server metadata handshake
        except Exception:
            try:
                conn.close()
            except Exception:  # noqa: BLE001 - best effort cleanup before retry.
                pass
            raise
        self._metadata = dict(metadata) if isinstance(metadata, dict) else {}
        self._conn = conn

    @property
    def metadata(self) -> dict[str, Any]:
        return dict(self._metadata)

    def fetch_metadata(self) -> dict[str, Any]:
        self._ensure_connection()
        return self.metadata

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


def _center_crop(img: np.ndarray, frac: float) -> np.ndarray:
    """Centered crop keeping `frac` of each dimension (aspect preserved).

    Byte-for-byte mirror of the training-time crop baked into the fisheye LeRobot
    dataset (openpi examples/pika_umi/convert_pika_umi_storage_video.py `_center_crop`):
    frac=0.65 on a 480x640 fisheye frame -> 312x416. The openpi server only
    resize_with_pads, so this crop is what keeps the live wrist images in
    distribution for the fe65 checkpoints.
    """
    h, w = img.shape[:2]
    ch, cw = int(round(h * frac)), int(round(w * frac))
    y0, x0 = (h - ch) // 2, (w - cw) // 2
    return img[y0 : y0 + ch, x0 : x0 + cw]


def _depth_to_image(
    depth_raw: np.ndarray, z_near_mm: float = 120.0, z_far_mm: float = 700.0, depth_units_m: float = 1e-4
) -> np.ndarray:
    """Live D405 raw depth (uint16, `depth_units_m` m/count) -> 3ch uint8, for the SigLIP encoder.

    MUST stay BIT-IDENTICAL to openpi examples/pika_umi/convert_pika_umi_storage_video.py
    `_depth_to_image` (same z_near/z_far/units, hole=far, 3ch replicate) so the depth channel is
    in-distribution at inference. The bundle client (include_depth=True) hands raw z16 as uint16
    (H,W) — same as the stored training depth — so the same transform applies."""
    valid = depth_raw > 0
    d_mm = depth_raw.astype(np.float32) * (depth_units_m * 1000.0)
    d = np.clip((d_mm - z_near_mm) / (z_far_mm - z_near_mm), 0.0, 1.0)
    d[~valid] = 1.0
    g = (d * 255.0).astype(np.uint8)
    return np.repeat(g[..., None], 3, axis=2)


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
        proprio_mode: str = "pose",
        prompt: str = OPENPI_DEFAULT_PROMPT,
        action_horizon: int | None = None,
        camera_names: tuple[str, str] = ("left_realsense_color", "right_realsense_color"),
        wrist_crop_frac: float = 0.0,
        include_depth: bool = False,
        depth_z_near_mm: float = 120.0,
        depth_z_far_mm: float = 700.0,
        depth_units_m: float = 1e-4,
        sample_steps: int = 1,  # accepted for CLI symmetry; unused remotely
        device: str = "remote",  # accepted for CLI symmetry; inference is server-side
        rtc_enabled: bool = False,
        rtc_inference_delay: int = 2,
        rtc_prefix_attention_schedule: str = "exp",
        rtc_max_guidance_weight: float = 5.0,
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
        # Close-bias (percent subtracted from the absolute opening target so a
        # marginal grasp clamps); main.py overrides from --gripper-close-bias.
        self.gripper_close_bias = 0.0
        # BINARY gripper (binarized open/close checkpoints, e.g. openpi `binary 25`):
        # threshold the model's bimodal gripper output and snap to the open/close
        # presets. A flavour of the absolute path. main.py overrides these from
        # --gripper-action-mode binary / --gripper-open|close-percent / --gripper-binary-threshold.
        self.gripper_binary = False
        self.gripper_open_percent = 50.0
        self.gripper_close_percent = 7.0
        self.gripper_binary_threshold = 50.0
        self.stderr = stderr
        self.device = device
        self.stats: dict[str, Any] = {}
        self.action_frame = "ee_local"
        self.ee_local_r_align = resolve_ee_local_r_align(ee_local_r_align)
        # Proprio representation sent as observation/state, MUST match the served
        # checkpoint's training distribution (openpi convert --state-mode):
        #   pose          -> 14-D reset-relative pose [pos3, rotvec3, grip] x L,R (default; legacy)
        #   velocity      -> 12-D ee_local velocity   [pos_vel3, rot_vel3]    x L,R (no gripper)
        #   velocity_grip -> 14-D ee_local velocity + abs gripper [pos_vel3, rot_vel3, grip] x L,R
        # nostate (zero_state=True) checkpoints ignore this server-side, so any mode works there.
        # velocity/velocity_grip are init-pose-INDEPENDENT (egocentric) -> avoid the per-episode
        # rotated reset frame that makes reset-relative pose non-ego-centric (see the velproprio
        # configs in openpi training/config.py). The velocity is finite-differenced from the robot
        # TCP pose between proprio samples and normalized to one policy step (the training per-frame
        # delta); see _proprio_state_velocity.
        if str(proprio_mode) not in ("pose", "velocity", "velocity_grip"):
            raise ValueError(
                f"proprio_mode must be 'pose', 'velocity', or 'velocity_grip', got {proprio_mode!r}"
            )
        self.proprio_mode = str(proprio_mode)
        self._state_dim = 12 if self.proprio_mode == "velocity" else 14
        # Velocity-proprio finite-difference memory (per arm): the previous proprio-sample TCP pose
        # and the wall-clock of that sample (to rescale a multi-step displacement to one policy step).
        self._vel_prev_pose_by_arm: dict[str, np.ndarray | None] = {"left": None, "right": None}
        self._vel_prev_sample_t: float | None = None
        self.command_family_option = (
            normalize_flow_command_family(command_family) if command_family else "tcp_twist_local"
        )
        self.command_family = canonical_flow_command_family(self.command_family_option)
        # Fake-image smoke mode runs camera-less so the runtime does not gate on frames.
        self.camera_names = [] if self._fake_images else [str(name) for name in camera_names]
        # Depth (Option A): an include_depth checkpoint also gets *_wrist_0_depth through the same
        # SigLIP. The live D405 depth stream names are the color names with color->depth; the camera
        # bundle client MUST be built with include_depth=True (main.py) so z16 is decoded.
        self.include_depth = bool(include_depth)
        self.depth_camera_names = [n.replace("color", "depth") for n in self.camera_names]
        self.depth_z_near_mm = float(depth_z_near_mm)
        self.depth_z_far_mm = float(depth_z_far_mm)
        self.depth_units_m = float(depth_units_m)
        # Center-crop fraction applied to each wrist frame before sending to the server.
        # 0.0 = off (send full frame). The fisheye fe65 checkpoints were trained on a
        # 0.65 center-crop of the 640x480 fisheye (-> 416x312) baked in at LeRobot
        # conversion time (openpi convert_pika_umi_storage_video.py `_center_crop`); the
        # openpi server only resize_with_pads, so the SAME crop MUST be applied here or
        # the wrist images are out-of-distribution. Realsense deploys leave this 0.0.
        self.wrist_crop_frac = float(wrist_crop_frac)
        self.image_size = 224  # zero-fallback shape only; live frames are sent full-res
        server_metadata: dict[str, Any] = {}
        metadata_error: Exception | None = None
        try:
            server_metadata = self._client.fetch_metadata()
        except Exception as exc:  # noqa: BLE001 - old/offline servers fall back below; inference retries later.
            metadata_error = exc
        self.action_horizon = _resolve_openpi_action_horizon(action_horizon, server_metadata)
        metadata_horizon = _action_horizon_from_metadata(server_metadata)
        if metadata_horizon is None:
            if metadata_error is not None:
                print(
                    "[flow-infer] openpi server metadata unavailable "
                    f"({type(metadata_error).__name__}: {metadata_error}); "
                    f"using action_horizon={self.action_horizon}",
                    file=self.stderr,
                    flush=True,
                )
            elif action_horizon is None:
                print(
                    "[flow-infer] openpi server metadata has no action_horizon; "
                    f"using legacy default action_horizon={self.action_horizon}. "
                    "Pass --action-horizon for h8/h24/h50 checkpoints.",
                    file=self.stderr,
                    flush=True,
                )
        self.chunk_execute_steps = _resolve_openpi_chunk_execute_steps(
            chunk_execute_steps,
            self.action_horizon,
        )
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
        # Real-Time Chunking (RTC). When enabled, each infer sends the previous
        # chunk back to the server (`prev_action_chunk`) so it freezes the first
        # `inference_delay` actions and inpaints the rest -- smooth async replan
        # without the boundary crossfade. The prev chunk MUST be the server's
        # MODEL-SPACE output (`rtc_raw_actions`), round-tripped untouched (NOT the
        # gripper-rescaled / r_aligned `actions`). RTC stays OFF by default. See
        # robotics_lab/docs/rtc_design.md and openpi models_pytorch/rtc.py.
        self.rtc_enabled = bool(rtc_enabled)
        self.rtc_inference_delay = int(rtc_inference_delay)
        self.rtc_prefix_attention_schedule = str(rtc_prefix_attention_schedule)
        self.rtc_max_guidance_weight = float(rtc_max_guidance_weight)
        self._rtc_prev_raw_chunk: np.ndarray | None = None
        self._rtc_warned_no_raw = False
        # Chunk-boundary twist crossfade state (mirrors FlowMatchingActionSource;
        # this class skips super().__init__). 0 = off. RTC subsumes the crossfade
        # (it makes boundaries continuous at the flow level), so disable it when on.
        self._chunk_crossfade_steps = 0 if self.rtc_enabled else int(chunk_crossfade_steps)
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
        # Wrist-camera routing + crop, so the deploy provenance is visible at startup.
        print(
            f"[flow-infer] wrist cameras={self.camera_names} crop_frac={self.wrist_crop_frac}"
            + (" (no crop)" if self.wrist_crop_frac <= 0.0 else ""),
            file=self.stderr,
            flush=True,
        )
        print(
            f"[flow-infer] openpi action_horizon={self.action_horizon} "
            f"chunk_execute_steps={self.chunk_execute_steps}",
            file=self.stderr,
            flush=True,
        )
        print(
            f"[flow-infer] proprio_mode={self.proprio_mode} (state_dim={self._state_dim}) "
            + (
                "-- MUST match the served checkpoint's --state-mode; velocity is finite-differenced "
                "from TCP pose (use a small --chunk-execute-steps for the cleanest per-step velocity)"
                if self.proprio_mode != "pose"
                else "-- 14-D reset-relative pose"
            ),
            file=self.stderr,
            flush=True,
        )
        self._logged_wrist_shape = False

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
            "observation/state": np.zeros(self._state_dim, dtype=np.float32),
            "prompt": self.prompt,
        }
        if getattr(self, "include_depth", False):
            obs["observation/left_wrist_0_depth"] = zero.copy()
            obs["observation/right_wrist_0_depth"] = zero.copy()
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
        if self.proprio_mode in ("velocity", "velocity_grip"):
            return self._proprio_state_velocity(payload)
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

    @staticmethod
    def _rotate_vel6(vel6: np.ndarray, r_align: Any) -> np.ndarray:
        """Rotate a per-arm [pos_vel(3), rot_vel(3)] body vector by an r_align.

        Mirrors rotate_flow_arm_vectors (v @ R.T) but on a bare 6-vector, so it is
        layout-agnostic (works for the 12-D velocity state whose arms sit at
        offsets 0/6, where rotate_flow_arm_vectors' fixed 0/7 offsets do not fit).
        Accepts an EeLocalRAlign (separate linear/angular) or a plain 3x3 matrix.
        """
        linear = np.asarray(getattr(r_align, "linear", r_align), dtype=np.float64)
        angular = np.asarray(getattr(r_align, "angular", r_align), dtype=np.float64)
        out = np.asarray(vel6, dtype=np.float64).copy()
        out[0:3] = out[0:3] @ linear.T
        out[3:6] = out[3:6] @ angular.T
        return out

    def _proprio_state_velocity(self, payload: dict[str, Any]) -> np.ndarray:
        """ee_local VELOCITY proprio (--state-mode velocity / velocity_grip).

        Per arm the proprio is the INCOMING per-step motion in the previous body
        frame -- pos_vel(3), rot_vel(3) [, grip] -- the exact quantity the training
        converter bakes (openpi `_arm_velocity`: cur^-1 . (p_next - p_cur),
        cur^-1 . R_next). Because it is a body-frame delta it is W-invariant AND
        init-pose-INDEPENDENT (no per-episode reset anchor), the whole point of the
        velproprio variant over reset-relative pose.

        Cadence note: proprio is sampled at chunk-replan boundaries, not every
        policy step, so the raw pose difference spans `chunk_execute_steps` steps.
        We rescale it to ONE policy step (policy_dt / wall_dt, clamped <=1) so the
        magnitude matches the training per-frame delta the server's norm-stats
        expect. Exact when --chunk-execute-steps is 1; a close approximation for a
        few steps (the body frame barely rotates over ~0.1 s). The first sample of
        a rollout (and after reset_rtc) has no previous pose -> velocity 0 (matches
        the converter's vel[0]=0 / segment-start zeroing)."""
        now = time.perf_counter()
        wall_dt = (now - self._vel_prev_sample_t) if self._vel_prev_sample_t is not None else None
        self._vel_prev_sample_t = now
        # Rescale the multi-step displacement down to one policy step. Never amplify
        # (clamp <=1); a long stall (wall_dt large) -> ~0 velocity (stale, report rest).
        scale = 1.0
        if wall_dt is not None and wall_dt > 1e-6:
            scale = float(np.clip(float(self.policy_dt_sec) / wall_dt, 0.0, 1.0))

        from scipy.spatial.transform import Rotation

        include_grip = self.proprio_mode == "velocity_grip"
        features: list[np.ndarray] = []
        for side in ("left", "right"):
            pose = np.asarray(pose_from_state_payload(payload, side), dtype=np.float64)
            prev = self._vel_prev_pose_by_arm[side]
            if prev is None:
                vel = np.zeros(6, dtype=np.float64)
            else:
                r_prev = Rotation.from_quat(prev[3:7])
                pos_vel = r_prev.inv().apply(pose[:3] - prev[:3]) * scale
                rot_vel = (r_prev.inv() * Rotation.from_quat(pose[3:7])).as_rotvec() * scale
                vel = np.concatenate([pos_vel, rot_vel])
            self._vel_prev_pose_by_arm[side] = pose.copy()
            if self.ee_local_r_align is not None:
                # Body deltas are in the RB TCP frame; the checkpoint trained in the
                # EE (pika tip) frame -> v_tip = R_align . v_tcp (same as the pose path).
                vel = self._rotate_vel6(vel, self.ee_local_r_align)
            if include_grip:
                # velocity_grip keeps the ABSOLUTE gripper opening (matches the converter's
                # _state_velocity_grip), read exactly like the pose-state gripper dim.
                grip = _gripper_value_from_payload(payload, side)
                if grip is None:
                    live = self._live_gripper_percent(side)
                    grip = live if live is not None else _gripper_from_arm_payload(payload.get(side, {}))
                features.append(np.concatenate([vel, [float(grip) / 100.0]]))
            else:
                features.append(vel)
        return np.concatenate(features).astype(np.float32)

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
            if self.wrist_crop_frac > 0.0:
                rgb = _center_crop(rgb, self.wrist_crop_frac)
            images[key] = rgb
            decode_count += 1
        if missing_count > 0 or len(images) < 2:
            return None, decode_count, max(missing_count, 2 - len(images))
        if getattr(self, "include_depth", False):
            # Resolve the live D405 depth (z16 -> uint16 HxW from the bundle client when its
            # include_depth=True) and encode it with the SAME _depth_to_image as the training
            # converter. Fail-closed if depth is missing (an include_depth checkpoint needs it).
            for key, depth_name in (("left_depth", self.depth_camera_names[0]), ("right_depth", self.depth_camera_names[1])):
                frame = resolve_frame(bundle_frames, depth_name)
                px = getattr(frame, "pixels", None)
                if px is None:
                    return None, decode_count, 2
                images[key] = _depth_to_image(
                    np.asarray(px), self.depth_z_near_mm, self.depth_z_far_mm, self.depth_units_m
                )
        # One-time proof of the actual image fed to the server (HWC). With crop_frac=0.65
        # on a 480x640 fisheye this prints (312, 416, 3); uncropped it prints (480, 640, 3).
        if not getattr(self, "_logged_wrist_shape", False):
            print(
                f"[flow-infer] sending wrist images shape={images['left'].shape} "
                f"(crop_frac={self.wrist_crop_frac})",
                file=self.stderr,
                flush=True,
            )
            self._logged_wrist_shape = True
        return images, decode_count, 0

    def reset_rtc(self) -> None:
        """Drop the cached previous chunk so the next infer cold-starts (vanilla).

        Call on rollout reset so RTC does not guide a new rollout's first chunk
        toward the previous rollout's committed plan.
        """
        self._rtc_prev_raw_chunk = None
        self._rtc_warned_no_raw = False
        # Velocity-proprio is a per-step finite difference; drop the previous-pose
        # memory so a new rollout's first sample reports rest (vel 0), not a jump
        # across the reset teleport.
        self._vel_prev_pose_by_arm = {"left": None, "right": None}
        self._vel_prev_sample_t = None

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
        if getattr(self, "include_depth", False):
            obs["observation/left_wrist_0_depth"] = images["left_depth"]
            obs["observation/right_wrist_0_depth"] = images["right_depth"]
        if self.rtc_enabled and self._rtc_prev_raw_chunk is not None:
            # Round-trip the previous MODEL-SPACE chunk so the server freezes the
            # first `inference_delay` actions and inpaints the rest toward it.
            # execute_horizon = the committed steps per replan (chunk_execute_steps).
            obs["prev_action_chunk"] = self._rtc_prev_raw_chunk
            obs["inference_delay"] = int(np.clip(self.rtc_inference_delay, 0, self.chunk_execute_steps))
            obs["execute_horizon"] = int(self.chunk_execute_steps)
            obs["prefix_attention_schedule"] = self.rtc_prefix_attention_schedule
            obs["max_guidance_weight"] = float(self.rtc_max_guidance_weight)
        try:
            result = self._client.infer(obs)
        except Exception as exc:  # noqa: BLE001 - remote failure must not crash the loop
            print(f"openpi remote inference failed: {type(exc).__name__}: {exc}", file=self.stderr, flush=True)
            return None
        chunk = np.asarray(result.get("actions"), dtype=np.float32)
        if chunk.ndim != 2 or chunk.shape[1] < 14:
            print(f"openpi remote returned unexpected action shape {chunk.shape}", file=self.stderr, flush=True)
            return None
        if int(chunk.shape[0]) != int(self.action_horizon):
            print(
                "openpi remote returned action_horizon "
                f"{int(chunk.shape[0])}, expected {int(self.action_horizon)}; "
                "check --action-horizon and the served openpi checkpoint config",
                file=self.stderr,
                flush=True,
            )
            return None
        chunk = chunk[:, :14].copy()
        if self.rtc_enabled:
            # Cache the server's MODEL-SPACE chunk (pre output-transform) to seed
            # the next infer's prev_action_chunk. Round-tripped opaque: NOT the
            # gripper-rescaled `actions` below. Absent => server too old => vanilla.
            raw = result.get("rtc_raw_actions")
            if raw is not None:
                self._rtc_prev_raw_chunk = np.asarray(raw, dtype=np.float32)
            elif not self._rtc_warned_no_raw:
                print(
                    "[flow-infer] RTC enabled but server returned no 'rtc_raw_actions' -> "
                    "staying vanilla; update the openpi server (policy.py rtc_raw_actions).",
                    file=self.stderr,
                    flush=True,
                )
                self._rtc_warned_no_raw = True
        # The server emits the gripper dim in /100 units (opening fraction) for BOTH
        # the legacy delta convention `(target-current)/100` and the absolute
        # `--gripper-mode absolute` convention `grip/100`. Scale to percent here; the
        # delta-vs-absolute interpretation is handled downstream by
        # gripper_action_absolute (_integrate_gripper_targets / _dispatch_gripper_step).
        chunk[:, _GRIP_DIMS[0]] *= 100.0
        chunk[:, _GRIP_DIMS[1]] *= 100.0
        return chunk
