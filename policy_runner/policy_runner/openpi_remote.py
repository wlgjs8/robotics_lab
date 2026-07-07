"""Remote openpi (pi0.5) policy server as a policy_runner action source.

Bridges a running `openpi serve_policy.py` websocket server (pi05_pika_umi config)
into the standard flow-infer runtime so the pi0.5 checkpoint goes through the SAME
camera polling, reset anchoring, TcpPoseTarget emission, clamps, rollout-mode gates
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
import threading
import time
from collections import deque
from dataclasses import replace
from typing import Any, TextIO

import numpy as np

from .action_sources.tcp_pose_target import cartesian_action_requirements
from .camera_bundle_client import resolve_frame
from .flow_dataset import pose_from_state_payload
from .chunk_overlay_publisher import ChunkOverlayPublisher
from .flow_inference import (
    DEFAULT_CHUNK_OVERLAY_RUNWAY_STEPS,
    DEFAULT_FLOW_MAX_ANGULAR_VELOCITY_RAD_S,
    DEFAULT_FLOW_MAX_LINEAR_VELOCITY_M_S,
    FLOW_COMMAND_LABEL,
    FlowMatchingActionSource,
    _env_truthy,
    _gripper_value_from_payload,
    _open_action_log,
    _resolve_chunk_overlay_runway_steps,
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


def rtc_shift_prev_chunk(raw_chunk, execute_steps):
    """Advance the cached raw chunk by the steps that will have EXECUTED when the
    next chunk takes over (paper/Kinetix caller-side roll, missing from the port).

    The freeze/guidance must reference the previous plan's UNEXECUTED tail:
    new[0:d] is pinned to prev_shifted[0:d] = raw[execute : execute+d]. Without
    this shift the freeze pins to raw[0:d] — actions already executed one window
    ago — replaying ~d steps at every boundary. The zero-pad tail is never
    referenced: get_prefix_weights() is exactly 0 for index >= H - execute.
    """
    import numpy as _np

    raw = _np.asarray(raw_chunk, dtype=_np.float32)
    steps = int(max(0, min(int(execute_steps), raw.shape[0])))
    if steps == 0:
        return raw
    pad = _np.zeros((steps, raw.shape[1]), dtype=raw.dtype)
    return _np.concatenate([raw[steps:], pad], axis=0)


# Fixed-rate velocity-proprio pose history: one (t, pose7) per control tick per arm.
# ~512 samples covers >1 s at any realistic control rate — only the last ~policy_dt
# window is read, so this is a cheap bounded ring.
_VELPROPRIO_HISTORY_MAXLEN = 512
_VELPROPRIO_SOURCES = ("measured", "command")
_VELPROPRIO_SAMPLE_MODES = ("replan", "fixed_step", "camera_frame")
_CAMERA_TIME_MAX_PLAUSIBLE_AGE_SEC = 5.0
_CAMERA_TIME_FUTURE_TOLERANCE_SEC = 0.010


class OpenpiRemoteActionSource(FlowMatchingActionSource):
    """flow-infer action source backed by a remote openpi policy server."""

    def __init__(  # noqa: PLR0913 - mirrors FlowMatchingActionSource construction surface.
        self,
        server_url: str,
        *,
        timeout_sec: float = 0.2,
        camera_client: Any | None = None,
        policy_dt_sec: float | None = None,
        max_linear_velocity_m_s: float | None = None,
        max_angular_velocity_rad_s: float | None = None,
        max_linear_step_m: float = 0.002,
        max_angular_step_rad: float = 0.01,
        chunk_execute_steps: int | None = None,
        chunk_overlay_runway_steps: int = DEFAULT_CHUNK_OVERLAY_RUNWAY_STEPS,
        chunk_crossfade_steps: int = 0,
        tcp_target_pose_conditioning: str = "legacy_step_hold",
        tcp_target_pose_reanchor_mode: str = "measured_blend",
        tcp_target_pose_blend_steps: int = 2,
        allow_rbpodo_controller_simulation_cartesian: bool = False,
        gripper_runtime: GripperRuntime | None = None,
        ee_local_r_align: Any = None,
        proprio_mode: str = "pose",
        velproprio_sample_mode: str = "replan",
        velproprio_source: str = "measured",
        prompt: str = OPENPI_DEFAULT_PROMPT,
        action_horizon: int | None = None,
        camera_names: tuple[str, str] = ("left_realsense_color", "right_realsense_color"),
        wrist_crop_frac: float = 0.0,
        include_depth: bool = False,
        depth_z_near_mm: float = 120.0,
        depth_z_far_mm: float = 700.0,
        depth_units_m: float = 1e-4,
        blank_depth: bool = False,
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
        validated_velproprio_source = self._validate_velproprio_source(
            velproprio_source,
            proprio_mode=str(proprio_mode),
            velproprio_sample_mode=str(velproprio_sample_mode),
        )
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
        # Shared base + per-arm overrides (None -> use base). main.py resolves the
        # per-arm values from --gripper-close-bias-left/right (defaults 2.0/6.0).
        self.gripper_close_bias = 0.0
        self.gripper_close_bias_left = None
        self.gripper_close_bias_right = None
        # BINARY gripper (binarized open/close checkpoints, e.g. openpi `binary 25`):
        # threshold the model's bimodal gripper output and snap to the open/close
        # presets. A flavour of the absolute path. main.py overrides these from
        # --gripper-action-mode binary / --gripper-open|close-percent / --gripper-binary-threshold.
        self.gripper_binary = False
        self.gripper_open_percent = 50.0
        self.gripper_close_percent = 7.0
        self.gripper_binary_threshold = 50.0
        # Close-snap deadzone: a mapped opening percent below this snaps to 0
        # (fully closed). main.py overrides from --gripper-close-snap-percent.
        self.gripper_close_snap_percent = 0.0
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
        #   velocity_grav -> 20-D ee_local velocity + gravity-tilt anchor + abs gripper
        #                    [pos_vel3, rot_vel3, gravity3, grip] x L,R (openpi --state-mode velocity_grav)
        # nostate (zero_state=True) checkpoints ignore this server-side, so any mode works there.
        # velocity/velocity_grip/velocity_grav are init-pose-INDEPENDENT (egocentric) -> avoid the
        # per-episode rotated reset frame that makes reset-relative pose non-ego-centric (see the
        # velproprio configs in openpi training/config.py). The "velocity" value is a body-frame
        # per-frame delta, not SI velocity; only legacy modes rescale it to approximate one policy
        # step. velocity_grav ADDS an absolute gravity-tilt anchor (world-down in the tool frame,
        # still yaw-invariant/ego-centric). See _proprio_state_velocity.
        if str(proprio_mode) not in ("pose", "velocity", "velocity_grip", "velocity_grav"):
            raise ValueError(
                f"proprio_mode must be 'pose', 'velocity', 'velocity_grip', or 'velocity_grav', got {proprio_mode!r}"
            )
        self.proprio_mode = str(proprio_mode)
        self._state_dim = {"velocity": 12, "velocity_grav": 20}.get(self.proprio_mode, 14)
        # How the velocity-proprio finite-difference window is chosen:
        #   replan     (default): difference the TCP pose between successive proprio SAMPLES
        #              (chunk-replan boundaries) and rescale the multi-step displacement to one
        #              policy step via policy_dt/wall_dt (clamped <=1). Simple, but the window
        #              length AND the wall-clock (which includes inference latency / SEQUENTIAL
        #              holds) both depend on the controller/pipeline timing, so a slower or
        #              burstier controller reports a SMALLER velocity than training saw.
        #   fixed_step: difference the MEASURED pose between two samples exactly ~policy_dt apart
        #              taken from a per-tick pose history (see _record_pose_history), independent
        #              of replan cadence and inference latency. Reproduces the training converter's
        #              single 30 Hz frame delta (openpi _arm_velocity) regardless of the controller.
        #   camera_frame: interpolate the MEASURED pose history at the camera observation time and
        #              one policy_dt earlier, then emit the raw local/body delta with NO dt scaling.
        #              This is the closest live equivalent of OpenPI/UMI converter _arm_velocity.
        if str(velproprio_sample_mode) not in _VELPROPRIO_SAMPLE_MODES:
            raise ValueError(
                f"velproprio_sample_mode must be one of {_VELPROPRIO_SAMPLE_MODES}, got {velproprio_sample_mode!r}"
            )
        self.velproprio_sample_mode = str(velproprio_sample_mode)
        self.velproprio_source = validated_velproprio_source
        # Velocity-proprio finite-difference memory (per arm): the previous proprio-sample TCP pose
        # and the wall-clock of that sample (to rescale a multi-step displacement to one policy step).
        self._vel_prev_pose_by_arm: dict[str, np.ndarray | None] = {"left": None, "right": None}
        self._vel_prev_sample_t: float | None = None
        # The streamed OpenPI path samples proprio in a prefetch thread while the
        # main 500 Hz loop records measured poses. Guard history deque iteration
        # with a short snapshot lock; Python deque raises if mutated mid-iteration.
        self._velproprio_history_lock = threading.Lock()
        # Per-tick measured-pose history for history-based velproprio modes: (t_monotonic, pose7).
        self._pose_history: dict[str, deque] = {
            "left": deque(maxlen=_VELPROPRIO_HISTORY_MAXLEN),
            "right": deque(maxlen=_VELPROPRIO_HISTORY_MAXLEN),
        }
        # Parallel command-pose history for velproprio_source='command': one emitted absolute
        # TcpPoseTarget pose7 per control tick per arm. Missing/hold ticks re-append the
        # last emitted target (ZOH) so held command velocity decays to zero.
        self._command_pose_history: dict[str, deque] = {
            "left": deque(maxlen=_VELPROPRIO_HISTORY_MAXLEN),
            "right": deque(maxlen=_VELPROPRIO_HISTORY_MAXLEN),
        }
        self._last_command_pose_by_arm: dict[str, np.ndarray | None] = {"left": None, "right": None}
        self._last_now_monotonic: float | None = None
        self._last_obs_camera_bundle: Any | None = None
        self._last_obs_camera_time_sec: float | None = None
        self._last_obs_camera_seq: int | None = None
        self.command_family = FLOW_COMMAND_LABEL
        # Fake-image smoke mode runs camera-less so the runtime does not gate on frames.
        self.camera_names = [] if self._fake_images else [str(name) for name in camera_names]
        # Depth (Option A): an include_depth checkpoint also gets *_wrist_0_depth through the same
        # SigLIP. The live D405 depth stream names are the color names with color->depth; the camera
        # bundle client MUST be built with include_depth=True (main.py) so z16 is decoded.
        self.include_depth = bool(include_depth)
        # Depth ABLATION (option C): still send the *_wrist_0_depth keys (so the
        # RGB-D server's input transform is satisfied and the 5-camera token
        # structure stays training-matched) but fill them with a constant all-far
        # frame instead of live depth. Isolates whether the policy uses depth
        # CONTENT — live depth is not read in this mode.
        self.blank_depth = bool(blank_depth)
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
        self.chunk_overlay_runway_steps = _resolve_chunk_overlay_runway_steps(chunk_overlay_runway_steps)
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
        # Chunk-boundary action crossfade state (mirrors FlowMatchingActionSource;
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
        # rb_gui predicted action-chunk overlay publisher. This class skips
        # super().__init__, so mirror FlowMatchingActionSource's setup here or NOTHING
        # publishes for openpi-remote rollouts (the inherited next_intent/_activate_chunk
        # call _publish_chunk_overlay, which no-ops when the publisher is None). Env-gated
        # via RB_GUI_CHUNK_OVERLAY_ENDPOINT; telemetry-only, best-effort UDP.
        self._last_overlay_payload = None
        self._chunk_overlay_seq = 0
        # Terminal chunk dump (debug). This class skips super().__init__, so the
        # attributes the inherited _print_chunk_steps reads must be set here or the
        # dump silently no-ops for openpi-remote rollouts. Gated on
        # FLOW_INFER_PRINT_CHUNK; prints each fresh chunk step-by-step for both arms
        # (position deltas in meters, rotation deltas in degrees).
        self._print_chunk_enabled = _env_truthy(os.environ.get("FLOW_INFER_PRINT_CHUNK"))
        self._print_chunk_seq = 0
        self._print_delta_overlay_enabled = _env_truthy(os.environ.get("FLOW_INFER_PRINT_DELTA_OVERLAY"))
        self._print_velproprio_enabled = _env_truthy(os.environ.get("FLOW_INFER_PRINT_VELPROPRIO"))
        # Per-step chunk-vs-actual tracking dump (FLOW_INFER_PRINT_TRACKING). This
        # class skips super().__init__, so the state the inherited _begin/_log_chunk
        # _tracking helpers touch must be seeded here too.
        self._print_tracking_enabled = _env_truthy(os.environ.get("FLOW_INFER_PRINT_TRACKING"))
        self._trk_predicted = {"left": None, "right": None}
        self._trk_prev_measured = {"left": None, "right": None}
        self._trk_start_monotonic = 0.0
        self._chunk_overlay_publisher = None
        _overlay_endpoint = os.environ.get("RB_GUI_CHUNK_OVERLAY_ENDPOINT")
        if _overlay_endpoint and _overlay_endpoint.strip():
            try:
                self._chunk_overlay_publisher = ChunkOverlayPublisher(_overlay_endpoint.strip())
            except Exception as exc:
                print(
                    f"WARNING: {self.policy_label}: chunk overlay publisher disabled "
                    f"for {_overlay_endpoint!r}: {type(exc).__name__}: {exc}",
                    file=self.stderr,
                )
        # Patch 3: online tcp_target_pose A-stage conditioning (this class skips
        # super().__init__, so the state the inherited foh helpers touch is set here).
        if str(tcp_target_pose_conditioning) not in CONDITIONING_MODES:
            raise ValueError(f"tcp_target_pose_conditioning must be one of {CONDITIONING_MODES}")
        if str(tcp_target_pose_reanchor_mode) not in REANCHOR_MODES:
            raise ValueError(f"tcp_target_pose_reanchor_mode must be one of {REANCHOR_MODES}")
        self._tcp_tp_mode = str(tcp_target_pose_conditioning)
        self._tcp_tp_reanchor_mode = str(tcp_target_pose_reanchor_mode)
        self._tcp_tp_blend_steps = int(tcp_target_pose_blend_steps)
        self._tcp_tp_conditioners = None
        self._tcp_tp_chunk_seq = 0
        self._current_gripper_targets: dict[str, float | None] = {"left": None, "right": None}
        # Per-policy-step action logger (env-gated, debug only). Mirrors
        # FlowMatchingActionSource; this class skips super().__init__, so the
        # attributes the inherited _log_action_step touches must be set here.
        # Set POLICY_RUNNER_ACTION_LOG=/path/to/actions.jsonl to capture one JSON
        # line per executed policy step (raw flow delta, sent target, chunk index).
        self._action_log: TextIO | None = None
        self._action_log_seq = 0
        self._action_log = _open_action_log(self.stderr)
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
        if self.proprio_mode != "pose":
            print(
                f"[flow-infer] velproprio_sample_mode={self.velproprio_sample_mode}"
                + (
                    " -- velocity from a fixed ~policy_dt window (per-tick pose history); "
                    "decoupled from replan cadence + inference latency"
                    if self.velproprio_sample_mode == "fixed_step"
                    else (
                        " -- measured TCP local delta over [camera_time - policy_dt, camera_time]; "
                        "no dt normalization, closest to OpenPI/UMI converter semantics"
                        if self.velproprio_sample_mode == "camera_frame"
                        else " -- velocity from successive replan-boundary samples, rescaled by policy_dt/wall_dt"
                    )
                ),
                file=self.stderr,
                flush=True,
            )
            if self.velproprio_source == "command":
                print(
                    "[flow-infer] velproprio_source=command: velocity proprio finite-differences "
                    "the EMITTED target stream (training semantics: velocity == realized intent); "
                    "measured pose still feeds reset/anchor/gravity",
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
        if self.proprio_mode in ("velocity", "velocity_grip", "velocity_grav"):
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

    @staticmethod
    def _validate_velproprio_source(
        velproprio_source: str,
        *,
        proprio_mode: str,
        velproprio_sample_mode: str,
    ) -> str:
        source = str(velproprio_source)
        if source not in _VELPROPRIO_SOURCES:
            raise ValueError(
                f"velproprio_source must be one of {_VELPROPRIO_SOURCES}, got {velproprio_source!r}"
            )
        if source == "command":
            if str(velproprio_sample_mode) == "camera_frame":
                raise ValueError(
                    "velproprio_source='command' is not supported with "
                    "velproprio_sample_mode='camera_frame'; camera_frame is camera/image aligned "
                    "and requires velproprio_source='measured'"
                )
            if str(velproprio_sample_mode) != "fixed_step":
                raise ValueError(
                    "velproprio_source='command' requires velproprio_sample_mode='fixed_step'; "
                    f"got {velproprio_sample_mode!r}"
                )
            if str(proprio_mode) == "pose":
                raise ValueError(
                    "velproprio_source='command' requires a velocity proprio mode "
                    "('velocity', 'velocity_grip', or 'velocity_grav'), not proprio_mode='pose'"
                )
        return source

    @staticmethod
    def _state_history_time_sec(payload: Any, now_monotonic: float) -> float:
        """Best monotonic timestamp for a state payload, falling back to receipt time.

        Server state ``host_time_ns`` is expected to be in the monotonic/steady-clock
        domain. Use it only when it is close to Python's monotonic receipt time; this
        avoids accidentally mixing epoch or camera raw-clock stamps into pose history.
        """
        try:
            raw = payload.get("host_time_ns") if isinstance(payload, dict) else None
            state_time = float(raw) * 1e-9
        except Exception:  # noqa: BLE001 - malformed timestamps are not fatal to proprio
            return float(now_monotonic)
        if np.isfinite(state_time) and abs(state_time - float(now_monotonic)) < 5.0:
            return state_time
        return float(now_monotonic)

    @staticmethod
    def _normalize_pose7_or_none(pose: Any) -> np.ndarray | None:
        arr = np.asarray(pose, dtype=np.float64)
        if arr.shape != (7,) or not np.all(np.isfinite(arr)):
            return None
        q_norm = float(np.linalg.norm(arr[3:7]))
        if q_norm <= 1e-9 or not np.isfinite(q_norm):
            return None
        out = arr.copy()
        out[3:7] /= q_norm
        return out

    def _clear_last_obs_camera_bundle(self) -> None:
        self._last_obs_camera_bundle = None
        self._last_obs_camera_time_sec = None
        self._last_obs_camera_seq = None

    def _camera_bundle_time_monotonic(self, bundle: Any) -> float | None:
        """Map a camera bundle timestamp into Python's monotonic domain.

        ``bundle_time_ns`` is stamped by camera_server's CLOCK_MONOTONIC_RAW clock,
        while state history and Python control timing use the monotonic domain. Map
        raw->monotonic using the current offset at polling time. If the raw stamp is
        absent or maps to a future/implausibly old time, fall back to the bundle's
        Python-side receipt timestamp instead of comparing raw camera time directly
        to robot ``host_time_ns``.
        """
        mono_now_sec = time.monotonic()

        def fallback_received() -> float | None:
            try:
                received = float(getattr(bundle, "received_monotonic"))
            except Exception:  # noqa: BLE001
                return None
            return received if np.isfinite(received) else None

        try:
            bundle_time_ns = int(getattr(bundle, "bundle_time_ns", 0) or 0)
        except Exception:  # noqa: BLE001
            bundle_time_ns = 0
        if bundle_time_ns <= 0:
            return fallback_received()

        raw_clock = getattr(time, "CLOCK_MONOTONIC_RAW", None)
        if raw_clock is None:
            return fallback_received()
        raw_now_sec = time.clock_gettime_ns(raw_clock) * 1e-9
        t_img_mono = mono_now_sec - (raw_now_sec - bundle_time_ns * 1e-9)
        age_sec = mono_now_sec - t_img_mono
        if (
            not np.isfinite(t_img_mono)
            or age_sec < -_CAMERA_TIME_FUTURE_TOLERANCE_SEC
            or age_sec > _CAMERA_TIME_MAX_PLAUSIBLE_AGE_SEC
        ):
            return fallback_received()
        return float(t_img_mono)

    def next_intent(self, snapshot, now_monotonic):  # type: ignore[override]
        # Record measured pose every control tick for history-based velocity-proprio modes.
        # fixed_step reads a fixed ~policy_dt window; camera_frame interpolates at the camera
        # observation time and one policy_dt earlier. Cheap bounded ring.
        self._last_now_monotonic = now_monotonic
        if (
            getattr(self, "velproprio_sample_mode", "replan") in ("fixed_step", "camera_frame")
            and self.proprio_mode != "pose"
        ):
            self._record_pose_history(getattr(snapshot, "payload", None), now_monotonic)
        intent = super().next_intent(snapshot, now_monotonic)
        if (
            getattr(self, "velproprio_sample_mode", "replan") == "fixed_step"
            and self.proprio_mode != "pose"
            and getattr(self, "velproprio_source", "measured") == "command"
        ):
            self._record_command_pose_history(intent, now_monotonic)
        return intent

    def _record_pose_history(self, payload, now_monotonic) -> None:
        if payload is None or now_monotonic is None:
            return
        history_t = self._state_history_time_sec(payload, float(now_monotonic))
        updates: list[tuple[str, float, np.ndarray]] = []
        for side in ("left", "right"):
            try:
                pose = np.asarray(pose_from_state_payload(payload, side), dtype=np.float64)
            except Exception:  # noqa: BLE001 - a malformed/partial payload just skips this tick
                continue
            updates.append((side, float(history_t), pose))
        if not updates:
            return
        with self._get_velproprio_history_lock():
            for side, t_sec, pose in updates:
                self._pose_history[side].append((t_sec, pose))

    def _record_command_pose_history(self, intent, now_monotonic) -> None:
        if now_monotonic is None:
            return
        with self._get_velproprio_history_lock():
            for side in ("left", "right"):
                pose = self._command_pose_from_intent(intent, side)
                if pose is not None:
                    pose = pose.copy()
                    self._last_command_pose_by_arm[side] = pose
                else:
                    last = self._last_command_pose_by_arm.get(side)
                    pose = None if last is None else last.copy()
                if pose is not None:
                    self._command_pose_history[side].append((float(now_monotonic), pose))

    def _get_velproprio_history_lock(self):
        lock = getattr(self, "_velproprio_history_lock", None)
        if lock is None:
            # Tests and old deserialized/manual instances can bypass __init__.
            lock = threading.Lock()
            self._velproprio_history_lock = lock
        return lock

    def _velproprio_history_snapshot(
        self,
        side: str,
        *,
        history: dict[str, deque] | None = None,
    ) -> list[tuple[float, np.ndarray]]:
        with self._get_velproprio_history_lock():
            histories = self._pose_history if history is None else history
            hist = histories.get(side)
            if not hist:
                return []
            return list(hist)

    @staticmethod
    def _command_pose_from_intent(intent, side: str) -> np.ndarray | None:
        if intent is None:
            return None
        arm = getattr(intent, side, None)
        if not isinstance(arm, dict):
            return None
        if str(arm.get("mode", "")) != "TcpPoseTarget":
            return None
        raw = arm.get("tcp_target_stand")
        if raw is None:
            raw = arm.get("target_tcp_stand")
        if raw is None:
            raise ValueError(f"{side} TcpPoseTarget intent missing tcp_target_stand")
        return OpenpiRemoteActionSource._tcp_target_pose7(raw, side=side)

    @staticmethod
    def _tcp_target_pose7(raw: Any, *, side: str) -> np.ndarray:
        if isinstance(raw, dict):
            try:
                xyz = [float(raw["x"]), float(raw["y"]), float(raw["z"])]
            except Exception as exc:  # noqa: BLE001
                raise ValueError(f"{side} TcpPoseTarget tcp_target_stand must include x/y/z") from exc
            quat_raw = raw.get("quaternion_xyzw")
            if quat_raw is None:
                try:
                    rotvec = [float(raw["rx"]), float(raw["ry"]), float(raw["rz"])]
                except Exception as exc:  # noqa: BLE001
                    raise ValueError(
                        f"{side} TcpPoseTarget tcp_target_stand must include quaternion_xyzw or rx/ry/rz"
                    ) from exc
                from scipy.spatial.transform import Rotation

                quat = Rotation.from_rotvec(rotvec).as_quat().tolist()
            else:
                quat = [float(v) for v in quat_raw]
            values = xyz + quat
        else:
            values = [float(v) for v in raw]
            if len(values) == 6:
                from scipy.spatial.transform import Rotation

                values = values[:3] + Rotation.from_rotvec(values[3:6]).as_quat().tolist()
        pose = np.asarray(values, dtype=np.float64)
        if pose.shape != (7,):
            raise ValueError(
                f"{side} TcpPoseTarget pose must normalize to pose7 [x,y,z,qx,qy,qz,qw], got {pose.shape}"
            )
        if not np.all(np.isfinite(pose)):
            raise ValueError(f"{side} TcpPoseTarget pose7 contains non-finite values")
        q_norm = float(np.linalg.norm(pose[3:7]))
        if q_norm <= 1e-9:
            raise ValueError(f"{side} TcpPoseTarget pose7 quaternion has zero norm")
        if not np.isclose(q_norm, 1.0, rtol=1e-3, atol=1e-6):
            raise ValueError(
                f"{side} TcpPoseTarget pose7 quaternion must be unit xyzw; norm={q_norm:.6g}"
            )
        return pose

    def _arm_body_velocities(self, payload: dict[str, Any]) -> dict[str, np.ndarray]:
        """Per-arm body-frame motion delta (pos3, rotvec3) in the TCP frame.

        The OpenPI/UMI converter calls this "velocity", but the training value is a
        per-frame local/body delta, not m/s or rad/s. Legacy modes keep their
        historical normalization behavior; camera_frame deliberately does no dt math.
        Defaults to 'replan' for sources constructed without the flag (legacy tests).
        """
        mode = getattr(self, "velproprio_sample_mode", "replan")
        if mode == "camera_frame":
            return self._arm_body_velocities_camera_frame(payload)
        if mode == "fixed_step":
            return self._arm_body_velocities_fixed_step(payload)
        return self._arm_body_velocities_replan(payload)

    def _arm_body_velocities_replan(self, payload: dict[str, Any]) -> dict[str, np.ndarray]:
        """Legacy: difference the TCP pose between successive proprio SAMPLES (replan
        boundaries) and rescale the multi-step displacement to one policy step via
        policy_dt/wall_dt (clamped <=1, never amplify)."""
        from scipy.spatial.transform import Rotation

        now = time.perf_counter()
        wall_dt = (now - self._vel_prev_sample_t) if self._vel_prev_sample_t is not None else None
        self._vel_prev_sample_t = now
        scale = 1.0
        if wall_dt is not None and wall_dt > 1e-6:
            scale = float(np.clip(float(self.policy_dt_sec) / wall_dt, 0.0, 1.0))
        out: dict[str, np.ndarray] = {}
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
            out[side] = vel
        return out

    def _arm_body_velocities_fixed_step(self, payload: dict[str, Any]) -> dict[str, np.ndarray]:
        """Fixed-window velocity from either measured pose or emitted command pose.

        Measured mode differences the CURRENT measured pose against the measured pose
        ~policy_dt earlier. Command mode differences the latest emitted absolute
        TcpPoseTarget against the command stream ~policy_dt earlier, preserving the
        training semantics where velocity is finite-differenced from the same trajectory
        the actions describe. Until the selected history spans a full policy step
        (rollout start / after reset) -> velocity 0.
        """
        from scipy.spatial.transform import Rotation

        now = self._last_now_monotonic
        use_command = getattr(self, "velproprio_source", "measured") == "command"
        out: dict[str, np.ndarray] = {}
        for side in ("left", "right"):
            if use_command:
                hist_samples = self._velproprio_history_snapshot(side, history=self._command_pose_history)
                if not hist_samples:
                    out[side] = np.zeros(6, dtype=np.float64)
                    continue
                cur_t, cur = hist_samples[-1]
                cur = np.asarray(cur, dtype=np.float64)
                prev, dt = self._velproprio_lookback(
                    side,
                    float(cur_t),
                    history=self._command_pose_history,
                    samples=hist_samples,
                )
            else:
                cur = np.asarray(pose_from_state_payload(payload, side), dtype=np.float64)
                prev, dt = self._velproprio_lookback(side, now)
            if prev is None or dt is None:
                out[side] = np.zeros(6, dtype=np.float64)
                continue
            # Normalize the ~one-step window to exactly one policy step. dt ~= policy_dt so the
            # ratio ~1; the clamp only guards a sparse buffer (never the latency blow-up the
            # replan path suffers, because dt here is measured motion time, not wall-clock).
            scale = float(np.clip(float(self.policy_dt_sec) / dt, 0.25, 4.0))
            r_prev = Rotation.from_quat(prev[3:7])
            pos_vel = r_prev.inv().apply(cur[:3] - prev[:3]) * scale
            rot_vel = (r_prev.inv() * Rotation.from_quat(cur[3:7])).as_rotvec() * scale
            out[side] = np.concatenate([pos_vel, rot_vel])
        return out

    def _velproprio_lookback(
        self,
        side: str,
        now: float | None,
        *,
        history: dict[str, deque] | None = None,
        samples: list[tuple[float, np.ndarray]] | None = None,
    ):
        """Newest history sample at least policy_dt older than `now` -> (pose7, dt). Returns
        (None, None) until the buffer spans a full policy step (-> cold-start velocity 0)."""
        hist = self._velproprio_history_snapshot(side, history=history) if samples is None else samples
        if not hist or now is None:
            return None, None
        target = float(now) - float(self.policy_dt_sec)
        for t, pose in reversed(hist):
            if t <= target:
                dt = float(now) - float(t)
                return (pose, dt) if dt > 1e-6 else (None, None)
        return None, None

    def _pose_history_at(self, side: str, t_sec: float) -> np.ndarray | None:
        """Interpolated measured TCP pose7 at ``t_sec`` from bounded pose history.

        Position is linear; orientation uses quaternion slerp. Returning ``None`` for
        out-of-bounds or sparse windows makes cold starts, resets, camera gaps, and
        hold-like missing windows emit zero velocity instead of inventing motion.
        """
        hist = self._velproprio_history_snapshot(side)
        if not hist or t_sec is None:
            return None
        try:
            target = float(t_sec)
        except Exception:  # noqa: BLE001
            return None
        if not np.isfinite(target):
            return None

        samples: list[tuple[float, np.ndarray]] = []
        for t, pose in hist:
            try:
                sample_t = float(t)
            except Exception:  # noqa: BLE001
                continue
            norm_pose = self._normalize_pose7_or_none(pose)
            if np.isfinite(sample_t) and norm_pose is not None:
                samples.append((sample_t, norm_pose))
        if not samples:
            return None
        samples.sort(key=lambda item: item[0])
        eps = 1e-9
        if target < samples[0][0] - eps or target > samples[-1][0] + eps:
            return None
        for sample_t, pose in samples:
            if abs(target - sample_t) <= eps:
                return pose.copy()

        max_gap_sec = max(2.5 * float(self.policy_dt_sec), 0.10)
        for (t0, pose0), (t1, pose1) in zip(samples[:-1], samples[1:]):
            if t0 <= target <= t1:
                gap = float(t1 - t0)
                if gap <= eps or gap > max_gap_sec:
                    return None
                alpha = (target - t0) / gap
                pos = (1.0 - alpha) * pose0[:3] + alpha * pose1[:3]
                from scipy.spatial.transform import Rotation, Slerp

                rots = Rotation.from_quat(np.stack([pose0[3:7], pose1[3:7]], axis=0))
                quat = Slerp([t0, t1], rots)([target]).as_quat()[0]
                out = np.concatenate([pos, quat])
                return self._normalize_pose7_or_none(out)
        return None

    def _arm_body_velocities_camera_frame(self, payload: dict[str, Any]) -> dict[str, np.ndarray]:
        """Measured TCP local delta over [camera_time - policy_dt, camera_time].

        This path intentionally does NOT divide by dt and does NOT rescale by
        policy_dt/actual_dt. The depth_z50_real OpenPI training converter stores a
        per-frame body delta, so the closest live equivalent is the raw measured pose
        delta at the image observation time.
        """
        del payload  # camera_frame uses timestamped measured pose history, not command/current payload pose.
        if getattr(self, "velproprio_source", "measured") != "measured":
            raise ValueError("velproprio_sample_mode='camera_frame' requires velproprio_source='measured'")
        from scipy.spatial.transform import Rotation

        t_cur = getattr(self, "_last_obs_camera_time_sec", None)
        out: dict[str, np.ndarray] = {}
        try:
            t_cur_float = float(t_cur)
        except Exception:  # noqa: BLE001
            t_cur_float = float("nan")
        if t_cur is None or not np.isfinite(t_cur_float):
            return {"left": np.zeros(6, dtype=np.float64), "right": np.zeros(6, dtype=np.float64)}
        t_cur = t_cur_float
        t_prev = t_cur - float(self.policy_dt_sec)
        for side in ("left", "right"):
            cur_pose = self._pose_history_at(side, t_cur)
            prev_pose = self._pose_history_at(side, t_prev)
            if cur_pose is None or prev_pose is None:
                out[side] = np.zeros(6, dtype=np.float64)
                continue
            r_prev = Rotation.from_quat(prev_pose[3:7])
            pos_delta_local = r_prev.inv().apply(cur_pose[:3] - prev_pose[:3])
            rot_delta = (r_prev.inv() * Rotation.from_quat(cur_pose[3:7])).as_rotvec()
            out[side] = np.concatenate([pos_delta_local, rot_delta])
        return out

    def _proprio_state_velocity(self, payload: dict[str, Any]) -> np.ndarray:
        """ee_local VELOCITY proprio (--state-mode velocity / velocity_grip / velocity_grav).

        Per arm the proprio is the INCOMING per-step motion in the previous body
        frame -- pos_vel(3), rot_vel(3) [, grip] -- the exact quantity the training
        converter bakes (openpi `_arm_velocity`: cur^-1 . (p_next - p_cur),
        cur^-1 . R_next). Because it is a body-frame delta it is W-invariant AND
        init-pose-INDEPENDENT (no per-episode reset anchor), the whole point of the
        velproprio variant over reset-relative pose.

        Cadence note: velproprio_sample_mode selects HOW the finite-difference
        window is chosen (see _arm_body_velocities): 'replan' differences successive
        replan-boundary samples and rescales by policy_dt/wall_dt (controller/latency
        dependent); 'fixed_step' differences the measured pose over a fixed ~policy_dt
        window from the per-tick history (controller-independent, matches the training
        per-frame delta), or the emitted command stream when velproprio_source='command';
        'camera_frame' differences measured pose history at [camera_time - policy_dt,
        camera_time] with no dt normalization, matching the converter's per-frame delta.
        The first sample of a rollout (and after reset_rtc) has no
        previous pose -> velocity 0 (matches the converter's vel[0]=0 / segment-start
        zeroing)."""
        from scipy.spatial.transform import Rotation

        include_grip = self.proprio_mode in ("velocity_grip", "velocity_grav")
        include_grav = self.proprio_mode == "velocity_grav"
        vel_by_side = self._arm_body_velocities(payload)
        features: list[np.ndarray] = []
        vel_for_print: dict[str, np.ndarray] = {}
        for side in ("left", "right"):
            pose = np.asarray(pose_from_state_payload(payload, side), dtype=np.float64)
            vel = vel_by_side[side]
            if self.ee_local_r_align is not None:
                # Body deltas are in the RB TCP frame; the checkpoint trained in the
                # EE (pika tip) frame -> v_tip = R_align . v_tcp (same as the pose path).
                vel = self._rotate_vel6(vel, self.ee_local_r_align)
            vel_for_print[side] = np.asarray(vel, dtype=np.float64).copy()
            parts: list[np.ndarray] = [vel]
            if include_grav:
                # GRAVITY-TILT ANCHOR (velocity_grav): world "down" expressed in the current TCP body
                # frame, then through the SAME R_align (TCP->tip) as the velocity vectors. Inserted between
                # rot_vel and grip to match the converter's _arm_velocity_grav layout
                # [pos_vel3, rot_vel3, gravity3, grip1]. world_down=[0,0,-1] (z-up stand); training used
                # steamvr Y-up [0,-1,0] -> the SAME physical gravity, so the unmeasured steamvr->stand
                # heading cancels (gravity is yaw-invariant). It is a LINEAR direction -> R_align.linear.
                grav = Rotation.from_quat(pose[3:7]).inv().apply(np.array([0.0, 0.0, -1.0]))
                if self.ee_local_r_align is not None:
                    lin = np.asarray(getattr(self.ee_local_r_align, "linear", self.ee_local_r_align), dtype=np.float64)
                    grav = grav @ lin.T
                parts.append(grav)
            if include_grip:
                # ABSOLUTE gripper opening (matches the converter's _state_velocity_grip / _arm_velocity_grav).
                grip = _gripper_value_from_payload(payload, side)
                if grip is None:
                    live = self._live_gripper_percent(side)
                    grip = live if live is not None else _gripper_from_arm_payload(payload.get(side, {}))
                parts.append(np.array([float(grip) / 100.0]))
            features.append(np.concatenate(parts))
        if getattr(self, "_print_velproprio_enabled", False):
            try:
                parts: list[str] = []
                for side, tag in (("left", "L"), ("right", "R")):
                    vel = vel_for_print[side]
                    pos_mm = vel[:3] * 1000.0
                    rot_deg = np.degrees(vel[3:6])
                    parts.append(
                        f"{tag} vel_pos_mm=[{pos_mm[0]:+.2f} {pos_mm[1]:+.2f} {pos_mm[2]:+.2f}] "
                        f"vel_rot_deg=[{rot_deg[0]:+.2f} {rot_deg[1]:+.2f} {rot_deg[2]:+.2f}]"
                    )
                print("[flow-infer] velproprio " + "  |  ".join(parts), file=self.stderr, flush=True)
            except Exception:
                pass
        return np.concatenate(features).astype(np.float32)

    def _raw_camera_images(self) -> tuple[dict[str, np.ndarray] | None, int, int]:
        """Full-resolution HWC uint8 RGB frames keyed left/right; None if missing."""
        if self._fake_images:
            self._clear_last_obs_camera_bundle()
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
            self._clear_last_obs_camera_bundle()
            return None, decode_count, max(missing_count, 2 - len(images))
        if getattr(self, "include_depth", False):
            # Resolve the live D405 depth (z16 -> uint16 HxW from the bundle client when its
            # include_depth=True) and encode it with the SAME _depth_to_image as the training
            # converter. Fail-closed if depth is missing (an include_depth checkpoint needs it).
            blank = getattr(self, "blank_depth", False)
            for key, depth_name in (("left_depth", self.depth_camera_names[0]), ("right_depth", self.depth_camera_names[1])):
                if blank:
                    # Ablation: keep the depth token, strip the information. All-far
                    # (255) == the model's invalid-depth/hole appearance. Shape only
                    # needs to be a valid HWC uint8 image; the server resizes it.
                    rgb = images["left" if key == "left_depth" else "right"]
                    images[key] = np.full((rgb.shape[0], rgb.shape[1], 3), 255, dtype=np.uint8)
                    continue
                frame = resolve_frame(bundle_frames, depth_name)
                px = getattr(frame, "pixels", None)
                if px is None:
                    self._clear_last_obs_camera_bundle()
                    return None, decode_count, 2
                images[key] = _depth_to_image(
                    np.asarray(px), self.depth_z_near_mm, self.depth_z_far_mm, self.depth_units_m
                )
            if blank and not getattr(self, "_logged_blank_depth", False):
                print(
                    "[flow-infer] BLANK-DEPTH ablation active: sending constant all-far depth, "
                    "live depth IGNORED (tests whether the policy uses depth content)",
                    file=self.stderr,
                    flush=True,
                )
                self._logged_blank_depth = True
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
        if bundle is not None:
            self._last_obs_camera_bundle = bundle
            self._last_obs_camera_time_sec = self._camera_bundle_time_monotonic(bundle)
            try:
                self._last_obs_camera_seq = int(getattr(bundle, "bundle_seq", 0) or 0)
            except Exception:  # noqa: BLE001
                self._last_obs_camera_seq = None
        else:
            self._clear_last_obs_camera_bundle()
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
        # Drop history-based pose buffers too, so a new rollout's first samples report rest
        # (no finite difference across the reset teleport) until a fresh policy_dt accumulates.
        with self._get_velproprio_history_lock():
            self._pose_history = {
                "left": deque(maxlen=_VELPROPRIO_HISTORY_MAXLEN),
                "right": deque(maxlen=_VELPROPRIO_HISTORY_MAXLEN),
            }
            self._command_pose_history = {
                "left": deque(maxlen=_VELPROPRIO_HISTORY_MAXLEN),
                "right": deque(maxlen=_VELPROPRIO_HISTORY_MAXLEN),
            }
            self._last_command_pose_by_arm = {"left": None, "right": None}
        self._last_now_monotonic = None
        self._clear_last_obs_camera_bundle()

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
            # SHIFT by the executed window first (caller-side roll, as in the
            # Kinetix reference): the freeze must pin to the previous plan's
            # UNEXECUTED tail, not its already-executed head.
            replan = int(getattr(self, "rtc_replan_period", 0) or self.chunk_execute_steps)
            obs["prev_action_chunk"] = rtc_shift_prev_chunk(self._rtc_prev_raw_chunk, replan)
            obs["inference_delay"] = int(np.clip(self.rtc_inference_delay, 0, replan))
            obs["execute_horizon"] = int(replan)
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
