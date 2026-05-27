# Prompt γ — Server config echo + episode config snapshot + inference invariant check

## Task

Server 측 state JSON 이 Cartesian/kinematics config 의 핵심 파라미터를 echo 하도록 추가한다. Recorder 가 episode 시작 시 config snapshot 을 capture 해서 HDF5 attribute 로 저장. Inference 시점에 episode metadata 의 config 와 현재 server config 를 비교해서 drift 감지.

## Context

Prerequisites: prompt α 와 β 가 완료된 상태.

Env parity 의 중요한 차원 중 하나는 **server config drift 검출**. 학습 데이터는 특정 `cartesian_control.path_kp_pos = 6.0`, `velocity_damping = 0.01`, `max_twist_linear_m_s = 0.03`, `kinematics.ik.damping = 0.001` 같은 server config 하에서 수집됨. 운영자가 server config 를 만져도 같은 policy emit 이 다른 robot motion 을 만들어내고, 그 결과 학습된 policy 가 silently 다른 environment 에서 추론하게 됨.

해결 전략:
1. **Server 측**: state JSON 에 cartesian_control + kinematics.ik 의 핵심 필드를 echo (이미 `observed_mode`, `observed_backend` 가 echo 되고 있는 패턴과 동일).
2. **Recorder 측 (Hdf5EpisodeRecorder)**: episode start 시점에 state JSON 의 config echo 부분을 SHA-256 hash 해서 episode attribute 로 저장. 또한 핵심 값 자체도 attribute 로 저장 (운영자가 hash 만 보고도 의미를 파악할 수 있도록).
3. **Inference 측 (BehaviorCloningActionSource)**: 추론 시작 시 episode metadata 의 config hash 와 현재 server state 의 config hash 를 비교. 다르면 warning 또는 error.

## Files to change

### Server (C++)

- `rb_servo_server/src/network/state_publisher.cpp` — state JSON 에 `cartesian_control_snapshot` 와 `kinematics_snapshot` 추가
- `rb_servo_server/include/rb_servo/network/state_publisher.hpp` — 필요시 helper 선언
- `rb_servo_server/src/control/dual_arm_servo_loop.cpp` — snapshot 에 config 참조 전달 (이미 config_ 보유 중)
- `rb_servo_server/tests/test_safety_policy.cpp` 또는 별도 test — server state JSON 에 새 필드 존재 검증

### Client (Python)

- `policy_runner/policy_runner/recording.py` — `Hdf5EpisodeRecorder.start_episode` 가 state snapshot 에서 config 추출 + hash + attrs 저장
- `policy_runner/policy_runner/training.py` — `BehaviorCloningActionSource` 에 invariant check 추가 (checkpoint 에 config hash 저장 + inference 시 비교)
- `policy_runner/policy_runner/main.py` — `train` subcommand 가 episode metadata 의 config hash 를 checkpoint 에 카피
- `policy_runner/tests/test_hdf5_recording.py` — config snapshot 테스트 추가
- `policy_runner/tests/test_policy_runner_contract.py` — invariant check 테스트 추가

## Required changes

### 1. Server: state JSON 에 config snapshot echo

`rb_servo_server/src/network/state_publisher.cpp` 의 state JSON 생성부에 두 group 추가:

```cpp
// helper at top of file (anonymous namespace)
nlohmann::json cartesianControlSnapshotJson(const CartesianControlConfig& cfg) {
    return {
        {"schema", "robotics_lab.cartesian_control_snapshot.v1"},
        {"enable", cfg.enable},
        {"allow_in_simulation", cfg.allow_in_simulation},
        {"allow_in_real", cfg.allow_in_real},
        {"path_kp_pos", cfg.path_kp_pos},
        {"path_kp_ori", cfg.path_kp_ori},
        {"twist_orientation_hold_kp", cfg.twist_orientation_hold_kp},
        {"twist_angular_deadband_rad_s", cfg.twist_angular_deadband_rad_s},
        {"velocity_damping", cfg.velocity_damping},
        {"max_twist_linear_m_s", cfg.max_twist_linear_m_s},
        {"max_twist_angular_rad_s", cfg.max_twist_angular_rad_s},
        {"max_linear_move_speed_m_s", cfg.max_linear_move_speed_m_s},
        {"max_angular_move_speed_rad_s", cfg.max_angular_move_speed_rad_s},
        {"max_cartesian_step_m", cfg.max_cartesian_step_m.value_or(0.0)},
        {"max_cartesian_step_rad", cfg.max_cartesian_step_rad.value_or(0.0)},
        {"exceed_limit_policy", toString(cfg.exceed_limit_policy)},
        {"linear_move", {
            {"min_duration_sec", cfg.linear_move.min_duration_sec},
            {"max_duration_sec", cfg.linear_move.max_duration_sec},
            {"default_linear_speed_m_s", cfg.linear_move.default_linear_speed_m_s},
            {"default_angular_speed_rad_s", cfg.linear_move.default_angular_speed_rad_s},
            {"default_orientation_mode", toString(cfg.linear_move.default_orientation_mode)},
        }},
    };
}

nlohmann::json kinematicsSnapshotJson(const KinematicsConfig& cfg) {
    return {
        {"schema", "robotics_lab.kinematics_snapshot.v1"},
        {"enable", cfg.enable},
        {"publish_tcp", cfg.publish_tcp},
        {"urdf", cfg.urdf},
        {"tip_frame", cfg.tip_frame},
        {"ik", {
            {"enable", cfg.ik.enable},
            {"damping", cfg.ik.damping},
            {"max_iterations", cfg.ik.max_iterations},
            {"timeout_ms", cfg.ik.timeout_ms},
            {"position_tolerance_m", cfg.ik.position_tolerance_m},
            {"orientation_tolerance_rad", cfg.ik.orientation_tolerance_rad},
            // max_step_deg 는 array — 일관되게 vector<double>
            {"max_step_deg", cfg.ik.max_step_deg},
        }},
    };
}
```

State JSON 의 top level 에 두 그룹 추가 (이미 `observed_mode`, `observed_backend` 가 publish 되는 같은 자리):

```cpp
out["cartesian_control_snapshot"] = cartesianControlSnapshotJson(config_.cartesian_control);
out["kinematics_snapshot"] = kinematicsSnapshotJson(config_.kinematics);
```

`state_publisher.cpp` 가 `DualArmConfig` 에 접근 가능한지 확인. 현재 `ServoSnapshot` 가 publisher 로 들어오므로, snapshot 에 config 참조를 같이 전달하거나 publisher 가 config 를 보유하도록 변경. 작업량 최소화를 위해 후자 권장: `StatePublisher::StatePublisher(const DualArmConfig& config, ...)` 시그니처에 const-ref 보관.

`StatePublisher::generateJson(const ServoSnapshot& snapshot)` 같은 메서드 안에서 `config_.cartesian_control` 및 `config_.kinematics` 를 직접 참조해서 위 헬퍼 호출.

### 2. Server: 변경 가능성 있는 필드만 echo, 변경 불가능 또는 비밀 필드는 제외

- `kinematics.urdf` 는 path. echo 해도 OK (env identification 에 유용).
- `kinematics.ik.max_step_deg` 는 6-vector. JSON array.
- robot IP, lease token, allowlist 같은 보안 민감 필드는 echo 금지 — 위 두 helper 가 그 필드들을 건드리지 않으니 안전.
- 새로 추가되는 필드가 있으면 helper 만 갱신하면 됨.

### 3. Server tests

`rb_servo_server/tests/test_safety_policy.cpp` 또는 `test_rbsim_hardware_free_gate.py` 에 케이스 추가:

```python
def test_state_json_contains_cartesian_control_snapshot(self):
    # Start servo + simulator. Read one state packet. Assert top-level
    # has 'cartesian_control_snapshot' with expected fields.
    snapshot = self._receive_state_json()
    self.assertIn("cartesian_control_snapshot", snapshot)
    cc = snapshot["cartesian_control_snapshot"]
    self.assertEqual(cc["schema"], "robotics_lab.cartesian_control_snapshot.v1")
    self.assertIn("path_kp_pos", cc)
    self.assertIn("max_twist_linear_m_s", cc)

def test_state_json_contains_kinematics_snapshot(self):
    snapshot = self._receive_state_json()
    self.assertIn("kinematics_snapshot", snapshot)
    km = snapshot["kinematics_snapshot"]
    self.assertEqual(km["schema"], "robotics_lab.kinematics_snapshot.v1")
    self.assertIn("ik", km)
    self.assertIn("damping", km["ik"])

def test_state_json_snapshot_reflects_config_change(self):
    # Run server with config A; verify path_kp_pos == A.value.
    # Restart server with config B (different path_kp_pos); verify echo == B.value.
    ...
```

C++ unit test 가 더 적합한 경우 (server state JSON serialization 단위 테스트):

```cpp
TEST(StatePublisher, EmitsCartesianControlSnapshot) {
    DualArmConfig config = makeMinimalConfig();
    config.cartesian_control.path_kp_pos = 7.5;
    config.cartesian_control.max_twist_linear_m_s = 0.04;
    StatePublisher publisher(config);
    ServoSnapshot snap = makeMinimalSnapshot();
    nlohmann::json out = publisher.serializeToJson(snap);
    ASSERT_TRUE(out.contains("cartesian_control_snapshot"));
    const auto& cc = out["cartesian_control_snapshot"];
    EXPECT_EQ(cc["schema"], "robotics_lab.cartesian_control_snapshot.v1");
    EXPECT_DOUBLE_EQ(cc["path_kp_pos"].get<double>(), 7.5);
    EXPECT_DOUBLE_EQ(cc["max_twist_linear_m_s"].get<double>(), 0.04);
}
```

### 4. Client: recorder 가 config snapshot 을 episode attribute 로 저장

`policy_runner/policy_runner/recording.py` 의 `Hdf5EpisodeRecorder.start_episode` 에서:

```python
import hashlib
import json as json_module

def start_episode(self, *, reset_snapshot, task_description, action_source, operator_id=None):
    # ... 기존 reset_pose 추출 ...
    
    payload = reset_snapshot.payload
    cc_snapshot = payload.get("cartesian_control_snapshot")
    km_snapshot = payload.get("kinematics_snapshot")
    
    # Compute hashes; missing snapshots → empty hash markers, not failure
    # (recorder must work with older server versions that don't echo these yet)
    cc_hash = _hash_canonical_json(cc_snapshot)
    km_hash = _hash_canonical_json(km_snapshot)
    
    self._current_episode = _EpisodeBuffer(
        ...,
        cartesian_control_snapshot=cc_snapshot,
        kinematics_snapshot=km_snapshot,
        cartesian_control_hash=cc_hash,
        kinematics_hash=km_hash,
    )


def _hash_canonical_json(obj) -> str:
    if obj is None:
        return ""
    canonical = json_module.dumps(obj, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()
```

`_EpisodeBuffer` 에 4개 필드 추가:
- `cartesian_control_snapshot: dict | None`
- `kinematics_snapshot: dict | None`
- `cartesian_control_hash: str`
- `kinematics_hash: str`

`end_episode` 에서 HDF5 attrs 와 group 으로 저장:

```python
f.attrs["cartesian_control_hash"] = ep.cartesian_control_hash
f.attrs["kinematics_hash"] = ep.kinematics_hash

# 사람이 읽을 수 있도록 전체 snapshot 도 group 으로 저장
if ep.cartesian_control_snapshot is not None:
    cc_grp = f.create_group("config/cartesian_control")
    _write_dict_as_attrs(cc_grp, ep.cartesian_control_snapshot)
if ep.kinematics_snapshot is not None:
    km_grp = f.create_group("config/kinematics")
    _write_dict_as_attrs(km_grp, ep.kinematics_snapshot)


def _write_dict_as_attrs(group, value):
    """Recursively write dict into HDF5 attrs and subgroups.
    
    - scalar values (str/int/float/bool) → attrs
    - list values → attrs (as numpy array if numeric, else json string)
    - nested dict → subgroup
    """
    import numpy as np
    for key, item in value.items():
        if isinstance(item, dict):
            sub = group.create_group(str(key))
            _write_dict_as_attrs(sub, item)
        elif isinstance(item, list):
            if all(isinstance(x, (int, float)) for x in item):
                group.attrs[str(key)] = np.asarray(item, dtype=np.float64)
            else:
                group.attrs[str(key)] = json.dumps(item)
        elif isinstance(item, (str, int, float, bool)):
            group.attrs[str(key)] = item
        else:
            group.attrs[str(key)] = json.dumps(item)
```

### 5. Training: checkpoint 에 config hash 카피

`policy_runner/policy_runner/training.py` 의 `train_behavior_cloning` 안에서 episode 폴더의 첫 HDF5 파일을 열어서 `cartesian_control_hash` 와 `kinematics_hash` 를 읽고 checkpoint dict 에 저장:

```python
def train_behavior_cloning(*, episodes_dir, checkpoint_path, ...):
    # ... 기존 코드 ...
    
    # Capture config hashes from the first episode (assumes all episodes share env)
    cc_hash, km_hash = _read_config_hashes_from_first_episode(episodes_dir)
    
    checkpoint = {
        "schema": "robotics_lab.policy_runner.bc_checkpoint.v2",  # bumped from v1
        "input_dim": x.shape[1],
        "output_dim": y.shape[1],
        "obs_mean": obs_mean.tolist(),
        "obs_std": obs_std.tolist(),
        "model_state": model.cpu().state_dict(),
        "training_cartesian_control_hash": cc_hash,
        "training_kinematics_hash": km_hash,
    }
    ...


def _read_config_hashes_from_first_episode(episodes_dir):
    """Return (cc_hash, km_hash) from the first .hdf5 file found, or ("", "")."""
    root = Path(episodes_dir)
    for path in sorted(root.glob("**/ep_*.hdf5")):
        try:
            import h5py
            with h5py.File(path, "r") as f:
                return (
                    str(f.attrs.get("cartesian_control_hash", "")),
                    str(f.attrs.get("kinematics_hash", "")),
                )
        except Exception:
            continue
    return "", ""
```

또한 학습이 multi-episode 라면 episode 간 hash 가 일치하는지 검증 (다르면 abort or warn):

```python
def _validate_episodes_share_config(episodes_dir, expected_cc_hash, expected_km_hash):
    import h5py
    root = Path(episodes_dir)
    mismatches = []
    for path in sorted(root.glob("**/ep_*.hdf5")):
        try:
            with h5py.File(path, "r") as f:
                cc = str(f.attrs.get("cartesian_control_hash", ""))
                km = str(f.attrs.get("kinematics_hash", ""))
                if cc != expected_cc_hash or km != expected_km_hash:
                    mismatches.append((path, cc, km))
        except Exception:
            continue
    return mismatches
```

`train_behavior_cloning` 에서 mismatch 발견 시:
- 기본: stderr warning, training 계속
- `--strict-config-check` 플래그가 있으면 abort

### 6. Inference: BehaviorCloningActionSource 가 invariant check 수행

`BehaviorCloningActionSource.__init__` 마지막에 checkpoint 의 hash 와 현재 server state 의 hash 비교:

```python
def __init__(self, checkpoint_path, timeout_sec=0.2):
    import torch
    # ... 기존 모델 로드 ...
    self.training_cartesian_control_hash = str(checkpoint.get("training_cartesian_control_hash", ""))
    self.training_kinematics_hash = str(checkpoint.get("training_kinematics_hash", ""))
    self._invariant_check_done = False
    self._invariant_check_passed = False

def next_intent(self, snapshot, now_monotonic):
    # 첫 inference 호출 시 1회만 invariant check
    if not self._invariant_check_done:
        self._invariant_check_done = True
        runtime_cc = snapshot.payload.get("cartesian_control_snapshot")
        runtime_km = snapshot.payload.get("kinematics_snapshot")
        runtime_cc_hash = _hash_canonical_json(runtime_cc)
        runtime_km_hash = _hash_canonical_json(runtime_km)
        cc_match = (runtime_cc_hash == self.training_cartesian_control_hash)
        km_match = (runtime_km_hash == self.training_kinematics_hash)
        self._invariant_check_passed = cc_match and km_match
        if not self._invariant_check_passed:
            import sys
            print(
                f"[WARN] BehaviorCloningActionSource: server config has drifted "
                f"from training data.\n"
                f"  training cartesian_control_hash: {self.training_cartesian_control_hash[:16]}...\n"
                f"  runtime cartesian_control_hash : {runtime_cc_hash[:16]}...\n"
                f"  training kinematics_hash       : {self.training_kinematics_hash[:16]}...\n"
                f"  runtime kinematics_hash        : {runtime_km_hash[:16]}...\n"
                f"  policy output may not reproduce training motion. "
                f"Re-train or restore the server config.",
                file=sys.stderr, flush=True,
            )
            # Behavior: still emit actions but log once.
            # An optional strict mode could be added later to refuse outputs.
    
    # ... 기존 inference 로직 ...
```

> 권장: warn 까지만. abort 는 의도된 config 변경 (예: max_twist_linear 를 안전을 위해 더 낮춤) 도 막아버려서 운영 부담. 운영자가 strict 가 필요하면 옵션 추가.

### 7. CLI: 옵션 추가

`policy_runner/policy_runner/main.py` 의 `train` subcommand 에:

```python
train_parser.add_argument(
    "--strict-config-check", action="store_true",
    help="Abort training if episodes have differing cartesian_control_hash "
         "or kinematics_hash (default: warn).",
)
```

`infer` subcommand 에:

```python
infer.add_argument(
    "--ignore-config-drift", action="store_true",
    help="Suppress the cartesian_control / kinematics drift warning at startup.",
)
```

`BehaviorCloningActionSource` 가 `ignore_drift` 인자를 받도록.

## Tests

### Recording side

`policy_runner/tests/test_hdf5_recording.py` 에 추가:

```python
def test_episode_captures_cartesian_control_snapshot_hash(self):
    # state snapshot payload with cartesian_control_snapshot dict
    # start_episode + end_episode
    # open hdf5: assert attrs["cartesian_control_hash"] is non-empty 64-char hex
    ...

def test_episode_captures_kinematics_snapshot_hash(self):
    ...

def test_hash_changes_when_config_changes(self):
    # two episodes with different cartesian_control_snapshot.path_kp_pos values
    # assert their cartesian_control_hash differ
    ...

def test_hash_is_empty_when_snapshot_missing(self):
    # state without cartesian_control_snapshot (older server)
    # episode is still written, attrs["cartesian_control_hash"] == ""
    ...

def test_config_group_is_human_readable_attrs(self):
    # cartesian_control_snapshot dict with nested 'linear_move' subdict
    # after end_episode, hdf5 has /config/cartesian_control/path_kp_pos attribute
    # and /config/cartesian_control/linear_move/min_duration_sec attribute
    ...
```

### Inference side

`policy_runner/tests/test_policy_runner_contract.py` 에 추가:

```python
def test_inference_warns_on_cartesian_config_drift(self):
    # build a mock checkpoint with training_cartesian_control_hash = "abc..."
    # build a mock state snapshot with cartesian_control_snapshot whose hash differs
    # instantiate BehaviorCloningActionSource with the mock checkpoint
    # call next_intent once with the mismatched snapshot
    # capture stderr: assert it contains "config has drifted"
    # call next_intent again: assert stderr does NOT receive a second warning
    ...

def test_inference_no_warning_when_hashes_match(self):
    # matching hashes; stderr is silent
    ...

def test_inference_no_warning_when_training_hash_empty(self):
    # checkpoint without training_cartesian_control_hash (v1 checkpoint)
    # runtime snapshot has full hash → no warning emitted
    # (silent backward-compat behavior)
    ...
```

### Training side

```python
def test_training_records_config_hashes_in_checkpoint(self):
    # write a fake episode hdf5 with known hashes
    # run train_behavior_cloning
    # load checkpoint, assert training_cartesian_control_hash matches
    ...

def test_training_warns_on_episode_hash_mismatch(self):
    # write two episode hdf5 files with different cartesian_control_hash
    # run train_behavior_cloning (without --strict-config-check)
    # capture stderr: warning about mismatch
    # training still completes
    ...

def test_training_aborts_on_episode_hash_mismatch_in_strict_mode(self):
    # same setup but with strict=True
    # assert RuntimeError or SystemExit
    ...
```

## Do not change

- prompt α / β 의 결과물 (Hdf5EpisodeRecorder state/action/image recording)
- existing checkpoint v1 backward compat (without hash fields)
- server 측 robot_state, fault, lease 관련 필드들
- `EpisodeRecorder` (JSONL)
- safety filter / fault classifier / arm worker
- 다른 action source

## Acceptance

- `PYTHONPATH=policy_runner python3 -m unittest discover -s policy_runner/tests -p "test_*.py"` 모든 테스트 통과
- 기존 테스트 회귀 없음
- 새 테스트 (recording: 5개, inference: 3개, training: 3개) 모두 통과
- C++ server hardware-free gate (`./scripts/codex_gate.sh HARDEN-10` 또는 동등 명령) 통과
- C++ 단위 테스트 (state JSON snapshot 검증) 통과
- 실제 server-policy_runner 통신 시:
  - server 의 state JSON 에 `cartesian_control_snapshot` 와 `kinematics_snapshot` 가 매 tick echo
  - recorder 가 생성한 HDF5 파일에 `cartesian_control_hash` 와 `kinematics_hash` attrs 가 SHA-256 hex 로 저장
  - HDF5 파일에 `/config/cartesian_control` 와 `/config/kinematics` group 이 인간 읽을 수 있는 attribute 로 존재
  - checkpoint v2 에 `training_cartesian_control_hash` 와 `training_kinematics_hash` 저장
  - inference 시 drift 가 있으면 stderr 에 한 번만 warning 출력, 동일 hash 면 silent
- README 또는 docs 에 "config drift detection" 한 문단 추가
