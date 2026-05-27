# Codex Prompts — HDF5 Episode Recording + Env Parity

이 패키지는 `policy_runner` 에 ACT 호환 HDF5 episode recording 을 단계적으로 추가하기 위한 codex 작업 프롬프트 모음.

## 작업 순서 (반드시 이 순서로)

| # | 파일 | 작업 단위 | 변경 영향 |
|---|---|---|---|
| 1 | `prompt-alpha-hdf5-recorder.md` | HDF5 episode recorder + reset anchor + episode lifecycle (state+action 만, camera 없음) | policy_runner 만 |
| 2 | `prompt-beta-camera-bundle.md` | ZMQ camera bundle subscriber + shm reader + recorder 의 이미지 통합 | policy_runner 만 |
| 3 | `prompt-gamma-config-snapshot.md` | Server state JSON 에 config echo + recorder 의 config hash + inference 의 drift 검출 | rb_servo_server + policy_runner |

각 prompt 는 self-contained 이고 그 앞 prompt 가 완료된 상태를 가정한다. 한 prompt 를 끝낼 때마다:

```bash
PYTHONPATH=policy_runner python3 -m unittest discover -s policy_runner/tests -p "test_*.py"
```

테스트가 모두 통과하면 다음 prompt 로 넘어간다. γ 는 C++ gate (`./scripts/codex_gate.sh`) 도 같이 통과 확인.

## 핵심 설계 결정 (이미 합의됨)

- **Action format**: TcpTwistLocal[6] (delta-by-construction, m/s + rad/s)
- **State format**: absolute joint angles + tcp_stand with quaternion
- **Reset pose anchor**: episode start 시점의 state snapshot 에서 `q_actual_deg`, `tcp_stand` 캡처 → HDF5 attribute 로 저장
- **HDF5 schema**: 원조 ACT (Tony Zhao 의 ALOHA codebase) 호환. LeRobot Parquet 포맷 아님
- **Schema 버전**: `robotics_lab.episode.v1`
- **Optional dependencies**: `h5py` (recording extra), `pyzmq` (camera extra), `numpy` (둘 다 공유)

## 학습 코드의 입력 변환

수집된 HDF5 episode 의 학습 입력은 다음과 같이 derive:

```python
import h5py
import numpy as np

with h5py.File("ep_20251211T143022Z.hdf5", "r") as f:
    qpos_left = f["observations/qpos_left"][...]       # (T, 6) absolute
    reset_qpos_left = f.attrs["reset_qpos_left"]       # (6,)
    delta_qpos_left = qpos_left - reset_qpos_left      # (T, 6) reset-anchored
    
    action_left = f["action/tcp_twist_local_left"][...]  # (T, 6) twist
    images_head = f["observations/images/head"][...]     # (T, H, W, 3) uint8
```

이 변환은 학습 코드에서 일관되게 정의 가능 (ACT, Diffusion Policy, π0 모두).

## Server config drift detection (Prompt γ 결과)

학습 데이터의 cartesian_control + kinematics 파라미터 hash 가 episode HDF5 의 attribute 로 저장.
Training 이 끝나면 checkpoint 에 그 hash 가 카피됨.
Inference 시작 시 현재 server state 의 hash 와 비교, 다르면 stderr warning.

이 메커니즘이 silent env drift 를 감지함.
