# Prompt α — HDF5 episode recorder + reset anchor + episode lifecycle

## Task
`policy_runner` 에 ACT 호환 HDF5 episode recorder 를 추가한다. state + action 만 기록 (camera 는 다음 prompt 에서). episode lifecycle (start/end) 과 reset pose anchor 를 갖춘다.

## Context

기존 `policy_runner/policy_runner/recording.py` 는 JSONL append-only `EpisodeRecorder` 만 제공한다. 학습용 데이터셋으로는 (1) 카메라 obs 부재, (2) reset pose anchor 부재, (3) episode 경계 부재, (4) 학습 시 random access 성능 부족 등의 한계가 있다.

이번 작업은 그중 (2)(3)(4) 를 해결하는 HDF5 recorder 를 추가한다. 카메라는 별도 prompt 에서 추가 (이번 작업은 그 사전 작업).

핵심 design decisions (이미 결정됨):
- **Action 은 그대로 `TcpTwistLocal` (delta-by-construction)** — relative action, 추가 변환 없음
- **State 는 absolute 로 저장 + `reset_pose` 를 episode attribute 로 anchor** — 학습 코드에서 `obs - reset` 변환 가능
- **HDF5 schema 는 원조 ACT (Tony Zhao 의 ALOHA codebase, `tonyzhaozh/aloha` / `MarkFzp/act-plus-plus`) 호환** — LeRobot 의 Parquet 포맷이 아닌 `h5py` 로 직접 inspect 가능한 단일 파일 per episode
- **Reset pose 는 `start_episode()` 호출 시점의 robot state snapshot** 으로 capture — 운영자가 spacemouse 로 reset 자세까지 이동한 후 episode 시작 가능
- **h5py 는 optional dependency** — `policy_runner[recording]` extras 로 분리. recording 안 쓰는 사용자는 의존성 깔 필요 없음
- **Fixed recording rate** (기본 30 Hz). state 200 Hz, policy 100 Hz 보다 느려서 recorder 가 dataset 의 time axis 를 정의

기존 `EpisodeRecorder` (JSONL) 는 backward compatibility 위해 유지. 새 `Hdf5EpisodeRecorder` 는 별도 class.

## Files to change

- `policy_runner/policy_runner/recording.py` (extend with new class)
- `policy_runner/policy_runner/config.py` (recording config 추가)
- `policy_runner/policy_runner/main.py` (CLI subcommand 추가 또는 `teleop-record` 확장)
- `policy_runner/pyproject.toml` (h5py optional extra 추가)
- `policy_runner/tests/` (새 테스트 파일)
- `README.md` 또는 `policy_runner/README.md` (사용법 한 문단 추가)

## Required changes

### 1. Optional h5py dependency

`policy_runner/pyproject.toml` 에 optional dependency 추가:

```toml
[project.optional-dependencies]
spacemouse = ["pyspacemouse>=1.1.0"]
recording = ["h5py>=3.10.0", "numpy>=1.24.0"]
```

(`numpy` 가 이미 존재하는 dependency 면 recording 에서 빼고 base 에 남겨도 OK.)

### 2. Config

`policy_runner/policy_runner/config.py` 에 `RecordingConfig` dataclass 추가:

```python
@dataclass
class RecordingConfig:
    output_dir: str = "/data/episodes"
    rate_hz: float = 30.0
    format: str = "hdf5"  # "hdf5" or "jsonl"
    
    def __post_init__(self):
        if self.rate_hz < 1.0 or self.rate_hz > 100.0:
            raise ValueError("recording.rate_hz must be in [1.0, 100.0]")
        if self.format not in {"hdf5", "jsonl"}:
            raise ValueError(f"recording.format must be 'hdf5' or 'jsonl', got: {self.format}")
```

`PolicyRunnerConfig` 에 `recording: RecordingConfig = field(default_factory=RecordingConfig)` 추가. YAML loader (`from_dict`) 에 `recording = RecordingConfig(**raw.get("recording", {}))` 로직 추가.

### 3. `Hdf5EpisodeRecorder` 구현

`policy_runner/policy_runner/recording.py` 에 추가. 기존 `EpisodeRecorder` 는 그대로 유지.

```python
class Hdf5EpisodeRecorder:
    """ACT-compatible per-episode HDF5 recorder.
    
    Buffers frames in memory and flushes to disk on end_episode().
    Each call to start_episode() creates a new buffer; multiple episodes
    can be recorded sequentially through the same recorder instance.
    
    h5py is an optional dependency; importing this class without
    `policy_runner[recording]` installed raises ImportError at instantiation.
    """
    
    SCHEMA = "robotics_lab.episode.v1"
    
    def __init__(
        self,
        output_dir: str | Path,
        *,
        recording_rate_hz: float = 30.0,
    ):
        try:
            import h5py  # noqa: F401
            import numpy  # noqa: F401
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "Hdf5EpisodeRecorder requires optional dependencies: "
                "install policy_runner with the recording extra"
            ) from exc
        if recording_rate_hz < 1.0 or recording_rate_hz > 100.0:
            raise ValueError("recording_rate_hz must be in [1.0, 100.0]")
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.recording_rate_hz = float(recording_rate_hz)
        self._period_sec = 1.0 / self.recording_rate_hz
        self._current_episode: _EpisodeBuffer | None = None
    
    def start_episode(
        self,
        *,
        reset_snapshot: StateSnapshot,
        task_description: str,
        action_source: str,
        operator_id: str | None = None,
    ) -> None:
        """Begin a new episode. The reset_snapshot's q_actual and tcp_stand
        for both arms are captured as reset_qpos_* and reset_tcp_stand_* attrs.
        
        Raises RuntimeError if an episode is already in progress (must call
        end_episode() first).
        """
        ...
    
    def record_frame(
        self,
        *,
        state_snapshot: StateSnapshot,
        action_packet: dict[str, Any] | None,
        action_host_time_ns: int | None,
        action_seq: int | None,
    ) -> None:
        """Append a (state, action) frame to the current episode buffer.
        
        Implements rate limiting: only appends if at least period_sec has
        elapsed since the last appended frame (based on monotonic time).
        
        Raises RuntimeError if no episode is active.
        """
        ...
    
    def end_episode(
        self,
        *,
        success: bool,
        end_reason: str,
        notes: str | None = None,
    ) -> Path:
        """Flush the current episode buffer to an HDF5 file and return path.
        
        end_reason: "operator_success" | "operator_failed" | "timeout" |
                    "fault_latched" | "operator_abort" | "other"
        
        Raises RuntimeError if no episode is active.
        """
        ...
    
    def close(self) -> None:
        """If an episode is in progress, end it with end_reason='operator_abort'."""
        ...
```

#### Internal `_EpisodeBuffer`

dataclass 로 구현하거나 plain class. fields:

```python
@dataclass
class _EpisodeBuffer:
    episode_id: str   # e.g. "ep_20251211T143022Z"
    task_description: str
    action_source: str
    operator_id: str | None
    
    reset_qpos_left: list[float]
    reset_qpos_right: list[float]
    reset_tcp_stand_left: list[float]   # [x,y,z, qx,qy,qz,qw]
    reset_tcp_stand_right: list[float]
    
    start_wall_time_ns: int
    start_monotonic: float
    last_appended_monotonic: float
    
    # per-frame state buffers
    qpos_left: list[list[float]]        # (T, 6)
    qpos_right: list[list[float]]
    qsent_left: list[list[float]]
    qsent_right: list[list[float]]
    tcp_stand_left: list[list[float]]   # (T, 7)
    tcp_stand_right: list[list[float]]
    fault_latched: list[bool]
    state_age_us: list[int]
    state_host_time_ns: list[int]
    command_seq: list[int]
    
    # per-frame action buffers
    action_mode: list[str]              # "TcpTwistLocal" | "Hold" | ...
    action_twist_left: list[list[float]]   # (T, 6) — zeros if mode != TcpTwistLocal for that arm
    action_twist_right: list[list[float]]
    action_deadman_left: list[bool]
    action_deadman_right: list[bool]
    action_host_time_ns: list[int]
    action_seq: list[int]
```

#### Helper: pose extraction from state payload

`record_frame` 와 `start_episode` 에서 공통으로 쓸 pose extractor:

```python
def _extract_tcp_stand_7(arm_state: dict) -> list[float]:
    """Extract [x, y, z, qx, qy, qz, qw] from arm state dict.
    
    Server state JSON has both 'tcp_stand' (dict with x,y,z,rx,ry,rz,quaternion_xyzw,qx,qy,qz,qw)
    and individual quaternion fields. We use quaternion_xyzw if present, else qx/qy/qz/qw.
    Returns [0,0,0, 0,0,0,1] (identity quaternion) if tcp_stand is missing.
    """
    tcp = arm_state.get("tcp_stand")
    if not isinstance(tcp, dict):
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    x = float(tcp.get("x", 0.0) or 0.0)
    y = float(tcp.get("y", 0.0) or 0.0)
    z = float(tcp.get("z", 0.0) or 0.0)
    quat = tcp.get("quaternion_xyzw")
    if isinstance(quat, list) and len(quat) == 4:
        return [x, y, z, float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])]
    qx = float(tcp.get("qx", 0.0) or 0.0)
    qy = float(tcp.get("qy", 0.0) or 0.0)
    qz = float(tcp.get("qz", 0.0) or 0.0)
    qw = float(tcp.get("qw", 1.0) or 1.0)
    return [x, y, z, qx, qy, qz, qw]
```

#### `start_episode` 구현

```python
def start_episode(self, *, reset_snapshot, task_description, action_source, operator_id=None):
    if self._current_episode is not None:
        raise RuntimeError(
            f"Episode '{self._current_episode.episode_id}' is still active; "
            "call end_episode() first"
        )
    payload = reset_snapshot.payload
    left = payload.get("left", {}) if isinstance(payload.get("left"), dict) else {}
    right = payload.get("right", {}) if isinstance(payload.get("right"), dict) else {}
    
    episode_id = f"ep_{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}"
    self._current_episode = _EpisodeBuffer(
        episode_id=episode_id,
        task_description=task_description,
        action_source=action_source,
        operator_id=operator_id,
        reset_qpos_left=_float_list(left.get("q_actual_deg"), 6),
        reset_qpos_right=_float_list(right.get("q_actual_deg"), 6),
        reset_tcp_stand_left=_extract_tcp_stand_7(left),
        reset_tcp_stand_right=_extract_tcp_stand_7(right),
        start_wall_time_ns=time.time_ns(),
        start_monotonic=time.monotonic(),
        last_appended_monotonic=0.0,   # so the first record_frame always appends
        qpos_left=[], qpos_right=[],
        qsent_left=[], qsent_right=[],
        tcp_stand_left=[], tcp_stand_right=[],
        fault_latched=[], state_age_us=[], state_host_time_ns=[], command_seq=[],
        action_mode=[], action_twist_left=[], action_twist_right=[],
        action_deadman_left=[], action_deadman_right=[],
        action_host_time_ns=[], action_seq=[],
    )
```

#### `record_frame` 구현

rate limit + buffer append:

```python
def record_frame(self, *, state_snapshot, action_packet, action_host_time_ns, action_seq):
    ep = self._current_episode
    if ep is None:
        raise RuntimeError("no active episode; call start_episode() first")
    
    now = time.monotonic()
    if (now - ep.last_appended_monotonic) < self._period_sec - 1e-6:
        return  # too soon for next frame
    ep.last_appended_monotonic = now
    
    payload = state_snapshot.payload
    left = payload.get("left", {}) if isinstance(payload.get("left"), dict) else {}
    right = payload.get("right", {}) if isinstance(payload.get("right"), dict) else {}
    
    ep.qpos_left.append(_float_list(left.get("q_actual_deg"), 6))
    ep.qpos_right.append(_float_list(right.get("q_actual_deg"), 6))
    ep.qsent_left.append(_float_list(left.get("q_sent_deg"), 6))
    ep.qsent_right.append(_float_list(right.get("q_sent_deg"), 6))
    ep.tcp_stand_left.append(_extract_tcp_stand_7(left))
    ep.tcp_stand_right.append(_extract_tcp_stand_7(right))
    ep.fault_latched.append(bool(payload.get("fault_latched", False)))
    ep.state_age_us.append(int(payload.get("state_age_us", 0) or 0))
    ep.state_host_time_ns.append(int(payload.get("host_time_ns", 0) or 0))
    ep.command_seq.append(int(payload.get("command_seq", 0) or 0))
    
    if isinstance(action_packet, dict):
        left_action = action_packet.get("left", {}) if isinstance(action_packet.get("left"), dict) else {}
        right_action = action_packet.get("right", {}) if isinstance(action_packet.get("right"), dict) else {}
        # 'mode' 는 dual command 의 left/right 가 같은 mode 라고 가정.
        # 다를 경우 left 우선; "TcpTwistLocal" 이 한쪽에 있으면 TcpTwistLocal 로 기록.
        left_mode = str(left_action.get("mode", "Hold"))
        right_mode = str(right_action.get("mode", "Hold"))
        mode = "TcpTwistLocal" if "TcpTwistLocal" in (left_mode, right_mode) else left_mode
        ep.action_mode.append(mode)
        ep.action_twist_left.append(_float_list(left_action.get("tcp_twist_local"), 6))
        ep.action_twist_right.append(_float_list(right_action.get("tcp_twist_local"), 6))
        ep.action_deadman_left.append(bool(left_action.get("deadman", False)))
        ep.action_deadman_right.append(bool(right_action.get("deadman", False)))
    else:
        ep.action_mode.append("Hold")
        ep.action_twist_left.append([0.0] * 6)
        ep.action_twist_right.append([0.0] * 6)
        ep.action_deadman_left.append(False)
        ep.action_deadman_right.append(False)
    
    ep.action_host_time_ns.append(int(action_host_time_ns or 0))
    ep.action_seq.append(int(action_seq or 0))
```

`_float_list` 는 기존 `training.py` 의 헬퍼와 같은 시그니처를 새 모듈로 재사용하거나 `recording.py` 안에 작은 copy 를 둔다.

#### `end_episode` 구현

```python
def end_episode(self, *, success, end_reason, notes=None):
    ep = self._current_episode
    if ep is None:
        raise RuntimeError("no active episode")
    if end_reason not in {
        "operator_success", "operator_failed", "timeout",
        "fault_latched", "operator_abort", "other",
    }:
        raise ValueError(f"unknown end_reason: {end_reason}")
    
    import h5py
    import numpy as np
    
    duration_sec = time.monotonic() - ep.start_monotonic
    out_path = self.output_dir / f"{ep.episode_id}.hdf5"
    
    with h5py.File(out_path, "w") as f:
        # Attributes
        f.attrs["schema"] = self.SCHEMA
        f.attrs["episode_id"] = ep.episode_id
        f.attrs["created_wall_time_ns"] = ep.start_wall_time_ns
        f.attrs["task_description"] = ep.task_description
        f.attrs["action_source"] = ep.action_source
        if ep.operator_id is not None:
            f.attrs["operator_id"] = ep.operator_id
        f.attrs["reset_qpos_left"] = np.asarray(ep.reset_qpos_left, dtype=np.float32)
        f.attrs["reset_qpos_right"] = np.asarray(ep.reset_qpos_right, dtype=np.float32)
        f.attrs["reset_tcp_stand_left"] = np.asarray(ep.reset_tcp_stand_left, dtype=np.float32)
        f.attrs["reset_tcp_stand_right"] = np.asarray(ep.reset_tcp_stand_right, dtype=np.float32)
        f.attrs["success"] = bool(success)
        f.attrs["end_reason"] = end_reason
        if notes is not None:
            f.attrs["notes"] = notes
        f.attrs["duration_sec"] = duration_sec
        f.attrs["recording_rate_hz"] = self.recording_rate_hz
        f.attrs["frame_count"] = len(ep.qpos_left)
        
        # Datasets — /observations/
        obs = f.create_group("observations")
        obs.create_dataset("qpos_left", data=np.asarray(ep.qpos_left, dtype=np.float32), compression="gzip", compression_opts=1)
        obs.create_dataset("qpos_right", data=np.asarray(ep.qpos_right, dtype=np.float32), compression="gzip", compression_opts=1)
        obs.create_dataset("qsent_left", data=np.asarray(ep.qsent_left, dtype=np.float32), compression="gzip", compression_opts=1)
        obs.create_dataset("qsent_right", data=np.asarray(ep.qsent_right, dtype=np.float32), compression="gzip", compression_opts=1)
        obs.create_dataset("tcp_stand_left", data=np.asarray(ep.tcp_stand_left, dtype=np.float32), compression="gzip", compression_opts=1)
        obs.create_dataset("tcp_stand_right", data=np.asarray(ep.tcp_stand_right, dtype=np.float32), compression="gzip", compression_opts=1)
        obs.create_dataset("fault_latched", data=np.asarray(ep.fault_latched, dtype=bool))
        obs.create_dataset("state_age_us", data=np.asarray(ep.state_age_us, dtype=np.int64))
        obs.create_dataset("state_host_time_ns", data=np.asarray(ep.state_host_time_ns, dtype=np.int64))
        obs.create_dataset("command_seq", data=np.asarray(ep.command_seq, dtype=np.int64))
        
        # Datasets — /action/
        act = f.create_group("action")
        # variable-length string for mode
        mode_dtype = h5py.string_dtype(encoding="utf-8")
        act.create_dataset("mode", data=np.asarray(ep.action_mode, dtype=object), dtype=mode_dtype)
        act.create_dataset("tcp_twist_local_left", data=np.asarray(ep.action_twist_left, dtype=np.float32), compression="gzip", compression_opts=1)
        act.create_dataset("tcp_twist_local_right", data=np.asarray(ep.action_twist_right, dtype=np.float32), compression="gzip", compression_opts=1)
        act.create_dataset("deadman_left", data=np.asarray(ep.action_deadman_left, dtype=bool))
        act.create_dataset("deadman_right", data=np.asarray(ep.action_deadman_right, dtype=bool))
        act.create_dataset("action_host_time_ns", data=np.asarray(ep.action_host_time_ns, dtype=np.int64))
        act.create_dataset("seq", data=np.asarray(ep.action_seq, dtype=np.int64))
    
    self._current_episode = None
    return out_path
```

`close()` 는 in-progress episode 가 있으면 `end_episode(success=False, end_reason="operator_abort")` 호출.

### 4. CLI integration

`policy_runner/policy_runner/main.py` 에 새 subcommand `hdf5-record` 추가 (기존 `teleop-record` 와 병행):

```python
hdf5_record = sub.add_parser(
    "hdf5-record",
    help="Teleop and record episodes to ACT-compatible HDF5 files. "
         "Episode lifecycle is controlled by SIGINT (Ctrl-C ends current episode).",
)
hdf5_record.add_argument("--config", required=True, help="policy_runner YAML config")
hdf5_record.add_argument("--output-dir", default=None,
                         help="Override recording.output_dir from config")
hdf5_record.add_argument("--task", required=True, help="Task description for this batch")
hdf5_record.add_argument("--operator", default=None, help="Operator ID (optional)")
hdf5_record.add_argument("--rate", type=float, default=None,
                         help="Override recording.rate_hz from config")
```

CLI handler:

```python
if args.command == "hdf5-record":
    from .recording import Hdf5EpisodeRecorder
    
    config = load_config(args.config)
    output_dir = args.output_dir or config.recording.output_dir
    rate_hz = args.rate or config.recording.rate_hz
    
    recorder = Hdf5EpisodeRecorder(output_dir, recording_rate_hz=rate_hz)
    state_client = RobotStateClient(config.robot_state.bind, config.robot_state.stale_timeout_sec)
    
    # Wait for first state to use as reset anchor
    print("Waiting for robot state to anchor reset_pose...", flush=True)
    reset_snapshot = None
    deadline = time.monotonic() + (config.runtime.startup_timeout_sec or 10.0)
    while time.monotonic() < deadline:
        reset_snapshot = state_client.poll_once(timeout_sec=0.2)
        if reset_snapshot is not None:
            break
    if reset_snapshot is None:
        print("ERROR: did not receive robot state within startup timeout", flush=True)
        state_client.close()
        return 2
    
    recorder.start_episode(
        reset_snapshot=reset_snapshot,
        task_description=args.task,
        action_source=config.action_source,
        operator_id=args.operator,
    )
    print(f"Started episode; reset anchored. Press Ctrl-C to end.", flush=True)
    
    command_client = ServoCommandClient(
        config.servo_command.endpoint,
        config.servo_command.timeout_sec,
    )
    
    # Hook: record_frame is called from the policy_runner main loop via state_sink+packet_sink
    last_packet: dict[str, Any] | None = None
    last_packet_time_ns = 0
    last_packet_seq = 0
    
    def packet_sink(packet: dict[str, Any]) -> None:
        nonlocal last_packet, last_packet_time_ns, last_packet_seq
        last_packet = packet
        last_packet_time_ns = time.time_ns()
        last_packet_seq = int(packet.get("seq", 0))
    
    def state_sink(snapshot: StateSnapshot) -> None:
        recorder.record_frame(
            state_snapshot=snapshot,
            action_packet=last_packet,
            action_host_time_ns=last_packet_time_ns,
            action_seq=last_packet_seq,
        )
    
    command_client = ServoCommandClient(
        config.servo_command.endpoint,
        config.servo_command.timeout_sec,
        packet_sink=packet_sink,
    )
    
    try:
        rc = run(
            config,
            state_client=state_client,
            command_client=command_client,
            state_sink=state_sink,
        )
        end_reason = "operator_success"
    except KeyboardInterrupt:
        rc = 0
        end_reason = "operator_abort"
    finally:
        if recorder._current_episode is not None:
            path = recorder.end_episode(success=(end_reason == "operator_success"), end_reason=end_reason)
            print(f"Episode written: {path}", flush=True)
    return rc
```

위 코드의 `recorder._current_episode is not None` private 접근은 awkward — `recorder.has_active_episode` property 를 추가하는 게 깔끔. 그리고 `recorder.close()` 가 in-progress episode 를 `operator_abort` 로 종료하니 `close()` 만 호출해도 동일 효과.

### 5. README 한 문단

`policy_runner/README.md` (없으면 신설) 또는 `README.md` 의 정책 섹션에 추가:

```markdown
## HDF5 Episode Recording

Record teleop episodes to ACT-compatible HDF5 files (one file per episode):

```bash
python -m policy_runner hdf5-record \
    --config policy_runner/config/dual_simulator_spacemouse.yaml \
    --task "pick up cup with left arm" \
    --operator user_a
```

The episode's reset_pose is anchored to the robot's joint state at the moment
this command receives its first state packet. Move the robot to your desired
reset configuration (via the GUI or scripted move) before launching this
command. Press Ctrl-C to end the current episode and flush to disk.

Schema: `robotics_lab.episode.v1`. Actions are recorded as TcpTwistLocal twists
(delta-by-construction). States are absolute (q_actual, q_sent, tcp_stand with
quaternion). The reset_qpos_left/right and reset_tcp_stand_left/right are
stored as HDF5 root attributes so training code can compute
`delta_obs = obs - reset` consistently.

Requires the recording extra: `pip install -e ".[recording]"`.
```

## Tests

`policy_runner/tests/test_hdf5_recording.py` 신설.

각 케이스는 `h5py` 가 설치되어 있다는 전제. test 진입 시 `import h5py` 실패하면 `unittest.skipUnless` 또는 `pytest.importorskip` 으로 skip.

테스트 케이스 최소 8개:

```python
class Hdf5EpisodeRecorderTest(unittest.TestCase):
    
    def setUp(self):
        try:
            import h5py  # noqa: F401
            import numpy  # noqa: F401
        except ModuleNotFoundError:
            self.skipTest("recording extras not installed")
        self.tmpdir = tempfile.mkdtemp()
    
    def tearDown(self):
        shutil.rmtree(self.tmpdir, ignore_errors=True)
    
    def _state_snapshot(self, **overrides):
        # build a StateSnapshot whose payload has left.q_actual_deg, right.q_actual_deg,
        # left.tcp_stand with x/y/z/quaternion_xyzw, etc.
        ...
    
    def test_start_episode_captures_reset_qpos_from_snapshot(self):
        # start_episode with a snapshot that has known q_actual values for left and right.
        # end_episode immediately. Open hdf5 file; assert reset_qpos_left/right attrs match.
        ...
    
    def test_start_episode_captures_reset_tcp_stand_from_snapshot(self):
        # snapshot with known tcp_stand quaternion_xyzw; assert reset_tcp_stand_*
        # attrs are [x,y,z, qx,qy,qz,qw].
        ...
    
    def test_record_frame_buffers_frames_at_target_rate(self):
        # start_episode, then call record_frame at 100 Hz monotonic ticks for 1 second
        # (use a monkeypatched monotonic clock). With recording_rate_hz=30, the resulting
        # HDF5 should have between 28 and 32 frames in /observations/qpos_left.
        ...
    
    def test_end_episode_writes_observations_and_action_groups(self):
        # start, record 5 frames at rate, end. Open file; assert /observations/{qpos_left,
        # qsent_left, tcp_stand_left, fault_latched, state_age_us, state_host_time_ns,
        # command_seq} all exist with first-dim length 5. Assert /action/{mode,
        # tcp_twist_local_left, deadman_left, action_host_time_ns, seq} exist with len 5.
        ...
    
    def test_end_episode_attrs_contain_success_and_end_reason(self):
        # end_episode(success=True, end_reason="operator_success"), open file,
        # assert f.attrs["success"] == True and f.attrs["end_reason"] == "operator_success".
        ...
    
    def test_end_episode_with_invalid_end_reason_raises(self):
        # start_episode, then end_episode(end_reason="bogus") raises ValueError.
        ...
    
    def test_start_episode_twice_without_end_raises(self):
        # start_episode, then second start_episode without end_episode raises RuntimeError.
        ...
    
    def test_action_mode_recorded_as_string(self):
        # record a frame with action_packet={"left": {"mode": "TcpTwistLocal", "tcp_twist_local": [...]}, ...}
        # end_episode. open file; assert /action/mode[0] == b"TcpTwistLocal" or str equivalent.
        ...
    
    def test_action_twist_zero_when_mode_is_hold(self):
        # record a frame with action_packet={"left": {"mode": "Hold"}, "right": {"mode": "Hold"}}
        # end. /action/tcp_twist_local_left[0] should be all zeros.
        ...
    
    def test_close_with_active_episode_aborts(self):
        # start_episode, do not call end_episode, call close().
        # the file should still be written with end_reason="operator_abort", success=False.
        ...
    
    def test_recording_config_rate_validation(self):
        # RecordingConfig(rate_hz=0.5) raises ValueError.
        # RecordingConfig(rate_hz=200.0) raises ValueError.
        # RecordingConfig(rate_hz=30.0) OK.
        ...
    
    def test_recording_config_default_from_dict(self):
        # PolicyRunnerConfig.from_dict({}) has recording.format=="hdf5", rate_hz==30.0.
        ...
```

## Do not change

- 기존 `EpisodeRecorder` (JSONL) class 의 동작
- 기존 `record_state_stream` 함수
- `training.py` 의 `load_dataset`, `state_vector`, `action_vector` — 기존 JSONL 학습 path 는 그대로 유지 (HDF5 학습 path 는 후속 prompt 에서)
- `ServoCommandClient.packet_sink` 시그니처
- `RobotStateClient` API
- server 측 어떤 코드도 변경 금지 (state JSON schema 그대로 사용)
- 다른 action source 코드

## Acceptance

- `PYTHONPATH=policy_runner python3 -m unittest discover -s policy_runner/tests -p "test_*.py"` 모든 테스트 통과
- 기존 91개 테스트 회귀 없음
- 새 `Hdf5EpisodeRecorder` 테스트 최소 11개 통과 (h5py 미설치 시 skip 처리)
- `python -m policy_runner hdf5-record --help` 가 새 subcommand 의 help 를 보여줌
- 생성된 HDF5 파일이 `h5py.File(path, "r")` 로 열리고 attrs 와 datasets 가 위 schema 와 일치
- `pyproject.toml` 의 optional extra 가 `pip install -e ".[recording]"` 로 설치 가능
- README 에 사용법 한 문단 추가
