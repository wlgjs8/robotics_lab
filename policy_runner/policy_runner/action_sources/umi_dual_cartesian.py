from __future__ import annotations

import datetime
import json
import math
import os
import socket
import time
from collections import deque
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Protocol

from policy_runner.action_sources.tcp_pose_target import (
    cartesian_action_requirements,
    clamp_pose_delta,
    tcp_pose_target_stand_intent,
)
from policy_runner.robot_state_client import StateSnapshot, parse_udp_endpoint
from policy_runner.servo_command_client import CommandIntent
from policy_runner.config import (
    UmiTargetLeadClampConfig,
    UmiTcpPoseTargetConditioningConfig,
)


# Default: no receiver-side offset — the pika publisher already streams the
# official gripper-tip pose (pika_sdk T_raw·R_corr·Trans(0.172,0,-0.076)).
# Legacy raw-tracker wire pairs with gripper_offset (0.172, 0.0, -0.076) in
# local/site configs when the publisher sends the raw tracker origin.
GRIPPER_OFFSET = (0.0, 0.0, 0.0)
IDENTITY_R_ALIGN = (
    1.0,
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
    0.0,
    0.0,
    1.0,
)
TCP_TARGET_PROFILE = "umi_large_smooth"


def _legacy_conditioning_config() -> UmiTcpPoseTargetConditioningConfig:
    return UmiTcpPoseTargetConditioningConfig(enable=False, mode="none")


def _legacy_lead_clamp_config() -> UmiTargetLeadClampConfig:
    return UmiTargetLeadClampConfig(enable=False)


@dataclass(frozen=True)
class UmiSample:
    pose_xyzw: tuple[float, float, float, float, float, float, float]
    gripper: float
    deadman: bool
    monotonic: float
    # Receiver-side monotonically increasing packet counter, stamped by the
    # reader once per distinct packet actually received (0 = never assigned).
    # The action source compares it across reads to tell a genuinely NEW packet
    # apart from a reader-cached "latest" replay (reuse / packet hold), which a
    # plain `read() is not None` cannot distinguish.
    seq: int = 0


class UmiPoseReader(Protocol):
    def read(self) -> UmiSample | None:
        ...

    def close(self) -> None:
        ...


@dataclass(frozen=True)
class _Transform:
    translation: tuple[float, float, float]
    rotation: tuple[float, ...]


@dataclass
class _ArmTeleopState:
    arm_init: _Transform | None = None
    pika_init: _Transform | None = None
    previous_target: tuple[float, ...] | None = None
    last_sample: UmiSample | None = None
    # seq of the last sample this arm consumed; used to detect a fresh packet
    # (read() returned a higher seq) vs a reader-cached replay (same seq).
    last_seq: int = 0
    was_armed: bool = False
    deadband_target: tuple[float, ...] | None = None
    # Monotonic time the deadman first dropped while still armed (None when the
    # clutch is engaged). Drives the brief-drop grace window in _target_for_side.
    deadman_drop_mono: float | None = None
    conditioner_engaged: bool = False
    last_raw_target_stand: tuple[float, ...] | None = None
    last_emitted_target_stand: tuple[float, ...] | None = None
    schedule_start_target_stand: tuple[float, ...] | None = None
    schedule_end_target_stand: tuple[float, ...] | None = None
    schedule_start_time_ns: int | None = None
    schedule_end_time_ns: int | None = None
    last_fresh_sample_time_ns: int | None = None
    last_fresh_seq: int = 0


class _PoseMovingAverage:
    """Moving average over the last N distinct tracker SAMPLES (not 500 Hz
    ticks): position is the arithmetic mean, rotation is the hemisphere-aligned
    normalized quaternion mean. The buffer keeps filling while the deadman is
    released so the window is already warm at clutch engage; pika_init latches
    from the same filtered stream, so there is no engage offset."""

    def __init__(self, window: int):
        self.window = int(window)
        self._positions: deque[tuple[float, float, float]] = deque(maxlen=max(1, self.window))
        self._quats: deque[tuple[float, float, float, float]] = deque(maxlen=max(1, self.window))
        self._last_monotonic: float | None = None

    def filter(self, sample: UmiSample) -> UmiSample:
        if self.window <= 1:
            return sample
        if self._last_monotonic is None or sample.monotonic != self._last_monotonic:
            self._last_monotonic = sample.monotonic
            x, y, z, qx, qy, qz, qw = sample.pose_xyzw
            self._positions.append((x, y, z))
            self._quats.append((qx, qy, qz, qw))
        n = len(self._positions)
        mean_position = (
            sum(p[0] for p in self._positions) / n,
            sum(p[1] for p in self._positions) / n,
            sum(p[2] for p in self._positions) / n,
        )
        mean_quat = _average_quaternions(self._quats)
        return UmiSample(
            (*mean_position, *mean_quat),
            sample.gripper,
            sample.deadman,
            sample.monotonic,
            sample.seq,
        )


# 한국 표준시(KST, UTC+9) — 서버 시스템 TZ 와 무관하게 항상 KST 타임스탬프.
# .40 publisher(umi_teleop_publish.py)의 송신 로그와 대칭으로 수신측을 기록한다.
_KST = datetime.timezone(datetime.timedelta(hours=9), name="KST")
# 120Hz publish 기준 정상 간격 8.3ms — 이보다 크게 벌어지면 수신 적체/누락 의심.
_RECV_GAP_MS = 20.0


def _kst_now() -> datetime.datetime:
    return datetime.datetime.now(_KST)


def _resolve_teleop_log_path() -> str | None:
    """``POLICY_RUNNER_UMI_TELEOP_LOG`` 게이트.

    - 미설정/빈값 → None (로깅 비활성, 단위테스트·기본 경로에서 파일을 안 만든다).
    - '1'/'on'/'auto'/'true' → repo logs/ 하위에 실행마다 KST 타임스탬프 새 파일.
    - 그 외 값 → 명시 파일 경로로 사용.
    """
    configured = os.environ.get("POLICY_RUNNER_UMI_TELEOP_LOG")
    if not configured:
        return None
    if configured.lower() in ("1", "on", "auto", "true", "yes"):
        logs_dir = Path(__file__).resolve().parents[3] / "logs"
        logs_dir.mkdir(parents=True, exist_ok=True)
        return str(logs_dir / f"umi_teleop_recv_{_kst_now().strftime('%Y%m%d_%H%M%S')}_KST.log")
    return configured


class _TeleopStepLogger:
    """텔레옵 수신 경로 per-step 진단 로거 (실행마다 새 파일, KST 기준).

    .40 publisher 송신 로그와 대칭. 무장(armed)된 매 스텝에 대해:
      - recv 타이밍: 처리한 샘플의 publisher monotonic age(수신 지연) + 직전
        처리 샘플과의 dt(누락/적체 → [GAP])
      - raw 스텝(클램프 전, 손이 실제 요구한 변위) vs applied 스텝(클램프 후)
      - 축별 클램프 히트([CLAMP]) / deadband 동결([DB])
    "빠르면 끊김"이 클램프 포화/수신 누락 때문인지 데이터로 판별하기 위함.
    """

    def __init__(self, path: str):
        self._fh = open(path, "w", buffering=1, encoding="utf-8")
        self.path = path
        self._last_mono: dict[str, float] = {}
        self._fh.write(f"# umi_dual_cartesian 수신 per-step 로그  started={_kst_now().isoformat()}\n")
        self._fh.write(
            "# fields: <KST>  side  st=<ARMED|ENGAGE|HOLD|RELEASE|HOLD_TIMEOUT|STALE>  "
            "age_ms=<수신지연>  dt_ms=<직전 fresh 샘플간격>  "
            "has=<reader가 샘플 반환?>  fresh=<신규 packet?>  seq=<수신 packet 카운터>  "
            "raw_mm/raw_deg=<클램프전 요구 변위>  app_mm/app_deg=<적용 변위>  "
            "clamp=<xyz|rpy 포화축>  db=<deadband동결>  "
            "profile cond cond_active cond_alpha cond_horizon_ms cond_elapsed_ms "
            "fresh_packet reuse_tick emitted_delta raw_to_emitted lead_clamp "
            "tokens(GAP/REUSE/CLAMP/DB)\n"
        )

    def _dt_ms(self, side: str, mono: float) -> float:
        prev = self._last_mono.get(side)
        self._last_mono[side] = mono
        return -1.0 if prev is None else (mono - prev) * 1000.0

    def log_event(self, side: str, status: str, sample_mono: float | None, now_monotonic: float) -> None:
        age = -1.0 if sample_mono is None else (now_monotonic - sample_mono) * 1000.0
        self._fh.write(
            f"{_kst_now().strftime('%H:%M:%S.%f')[:-3]}  side={side[0].upper()}  "
            f"st={status}  age_ms={age:7.1f}\n"
        )

    def log_step(
        self,
        side: str,
        *,
        sample_mono: float,
        now_monotonic: float,
        has_sample: bool,
        fresh_packet: bool,
        sample_seq: int,
        just_engaged: bool,
        prev_target: tuple[float, ...] | None,
        prev_deadband: tuple[float, ...] | None,
        raw_pose6: tuple[float, ...],
        deadband_pose6: tuple[float, ...],
        applied_pose6: tuple[float, ...],
        deadband_linear_m: float,
        deadband_angular_rad: float,
        max_linear_step_m: float,
        max_angular_step_rad: float,
        conditioner: Mapping[str, Any] | None = None,
    ) -> None:
        conditioner = conditioner or {}
        age = (now_monotonic - sample_mono) * 1000.0
        # dt only advances on a genuinely fresh packet, so it reports the real
        # publisher inter-packet interval (a reuse step shares sample_mono and
        # would otherwise collapse dt to 0 and muddy the gap accounting).
        dt = self._dt_ms(side, sample_mono) if fresh_packet else -1.0
        base = prev_target if prev_target is not None else applied_pose6
        # raw = 손이 요구한 변위(deadband·클램프 전), app = 실제 적용된 변위(클램프 후)
        raw_lin = math.sqrt(sum((raw_pose6[i] - base[i]) ** 2 for i in range(3)))
        raw_ang = math.sqrt(sum(_angle_diff(raw_pose6[3 + i], base[3 + i]) ** 2 for i in range(3)))
        app_lin = math.sqrt(sum((applied_pose6[i] - base[i]) ** 2 for i in range(3)))
        app_ang = math.sqrt(sum(_angle_diff(applied_pose6[3 + i], base[3 + i]) ** 2 for i in range(3)))
        # signed per-axis deltas (stand frame) for direction diagnosis (symptom: wrong direction)
        mono_ns = int(now_monotonic * 1e9)
        raw_sx, raw_sy, raw_sz = ((raw_pose6[i] - base[i]) * 1000.0 for i in range(3))
        app_sx, app_sy, app_sz = ((applied_pose6[i] - base[i]) * 1000.0 for i in range(3))
        raw_sr, raw_sp, raw_syaw = (math.degrees(_angle_diff(raw_pose6[3 + i], base[3 + i])) for i in range(3))
        app_sr, app_sp, app_syaw = (math.degrees(_angle_diff(applied_pose6[3 + i], base[3 + i])) for i in range(3))
        # 클램프는 (deadband-applied - prev_target)를 축별로 자른다 → 그 기준으로 포화축 판정
        lin_axes = "xyz"
        ang_axes = "rpy"
        clamp = ""
        for i in range(3):
            if abs(deadband_pose6[i] - base[i]) > max_linear_step_m * 0.999:
                clamp += lin_axes[i]
        clamp += "|"
        for i in range(3):
            if abs(_angle_diff(deadband_pose6[3 + i], base[3 + i])) > max_angular_step_rad * 0.999:
                clamp += ang_axes[i]
        # deadband 동결 판정 (_apply_target_deadband 조건 복제)
        db_hit = False
        if (deadband_linear_m > 0.0 or deadband_angular_rad > 0.0) and prev_deadband is not None:
            ld = math.sqrt(sum((raw_pose6[i] - prev_deadband[i]) ** 2 for i in range(3)))
            ad = math.sqrt(sum(_angle_diff(raw_pose6[3 + i], prev_deadband[3 + i]) ** 2 for i in range(3)))
            db_hit = ld <= deadband_linear_m and ad <= deadband_angular_rad
        tokens = ""
        if dt > _RECV_GAP_MS:
            tokens += " [GAP]"
        if not fresh_packet:
            # No new packet this step: the server is being fed a held/reused
            # setpoint. This is the row the old fresh=1 logging hid.
            tokens += " [REUSE]"
        if clamp not in ("", "|"):
            tokens += " [CLAMP]"
        if db_hit:
            tokens += " [DB]"
        st = "ENGAGE" if just_engaged else "ARMED"
        self._fh.write(
            f"{_kst_now().strftime('%H:%M:%S.%f')[:-3]}  side={side[0].upper()}  st={st}  "
            f"mono_ns={mono_ns}  "
            f"age_ms={age:6.1f}  dt_ms={dt:6.1f}  "
            f"has={int(has_sample)}  fresh={int(fresh_packet)}  seq={sample_seq}  "
            f"profile={conditioner.get('profile', TCP_TARGET_PROFILE)}  "
            f"cond={conditioner.get('cond', 'none')}  "
            f"cond_active={int(bool(conditioner.get('cond_active', False)))}  "
            f"cond_alpha={float(conditioner.get('cond_alpha', 0.0)):5.3f}  "
            f"cond_horizon_ms={float(conditioner.get('cond_horizon_ms', 0.0)):6.2f}  "
            f"cond_elapsed_ms={float(conditioner.get('cond_elapsed_ms', 0.0)):6.2f}  "
            f"fresh_packet={int(bool(conditioner.get('fresh_packet', fresh_packet)))}  "
            f"reuse_tick={int(bool(conditioner.get('reuse_tick', not fresh_packet)))}  "
            f"raw_mm={raw_lin * 1000:6.2f} raw_deg={math.degrees(raw_ang):5.2f}  "
            f"app_mm={app_lin * 1000:6.2f} app_deg={math.degrees(app_ang):5.2f}  "
            f"emitted_delta_mm={float(conditioner.get('emitted_delta_mm', app_lin * 1000.0)):6.2f} "
            f"emitted_delta_deg={float(conditioner.get('emitted_delta_deg', math.degrees(app_ang))):5.2f}  "
            f"raw_to_emitted_mm={float(conditioner.get('raw_to_emitted_mm', 0.0)):6.2f} "
            f"raw_to_emitted_deg={float(conditioner.get('raw_to_emitted_deg', 0.0)):5.2f}  "
            f"lead_before_mm={float(conditioner.get('lead_before_mm', -1.0)):6.2f} "
            f"lead_before_deg={float(conditioner.get('lead_before_deg', -1.0)):5.2f}  "
            f"lead_after_mm={float(conditioner.get('lead_after_mm', -1.0)):6.2f} "
            f"lead_after_deg={float(conditioner.get('lead_after_deg', -1.0)):5.2f}  "
            f"lead_clamp={int(bool(conditioner.get('lead_clamp', False)))}  "
            f"lead_clamp_axes={conditioner.get('lead_clamp_axes', '')}  "
            f"schedule_reset={int(bool(conditioner.get('schedule_reset', False)))}  "
            f"stale_stop={int(bool(conditioner.get('stale_stop', False)))}  "
            f"raw_sgn_mm=({raw_sx:+.1f},{raw_sy:+.1f},{raw_sz:+.1f}) raw_sgn_deg=({raw_sr:+.2f},{raw_sp:+.2f},{raw_syaw:+.2f})  "
            f"app_sgn_mm=({app_sx:+.1f},{app_sy:+.1f},{app_sz:+.1f}) app_sgn_deg=({app_sr:+.2f},{app_sp:+.2f},{app_syaw:+.2f})  "
            f"clamp={clamp}  db={int(db_hit)}{tokens}\n"
        )

    def close(self) -> None:
        try:
            self._fh.write(f"# closed={_kst_now().isoformat()}\n")
            self._fh.close()
        except (OSError, ValueError):
            pass


# External-move re-anchor: how close (in pika device space) the device must be to
# the engage anchor for the operator to count as "commanding ~no offset". Beyond
# these the operator is actively teleoping (target legitimately leads live) and the
# anchor is never touched, so the guard cannot regress normal motion; only a
# near-engage device + externally-displaced arm re-latches. The displacement gate
# (reanchor_linear_m/_angular_rad) is the real discriminator, so this stays loose
# enough to tolerate hand jitter. Device-space tolerances; tune on hardware if needed.
_REANCHOR_DEVICE_STILL_LINEAR_M = 0.01
_REANCHOR_DEVICE_STILL_ANGULAR_RAD = 0.10


class UmiDualCartesianActionSource:
    requirements = cartesian_action_requirements(allow_rbpodo_controller_simulation=True)

    def __init__(
        self,
        left_reader: UmiPoseReader,
        right_reader: UmiPoseReader,
        *,
        max_linear_step_m: float = 0.005,
        max_angular_step_rad: float = 0.04,
        input_moving_average_window: int = 1,
        deadband_linear_m: float = 0.0,
        deadband_angular_rad: float = 0.0,
        linear_axis_signs: Sequence[float] = (1.0, 1.0, 1.0),
        angular_axis_signs: Sequence[float] = (1.0, 1.0, 1.0),
        gripper_offset: Sequence[float] = GRIPPER_OFFSET,
        r_align: Sequence[float] = IDENTITY_R_ALIGN,
        workspace_bounds: Mapping[str, Sequence[float]] | Sequence[float] | None = None,
        sample_hold_timeout_sec: float | None = None,
        sample_stale_timeout_sec: float | None = None,
        timeout_sec: float = 0.05,
        deadman_release_grace_sec: float = 0.2,
        tcp_pose_target_conditioning: UmiTcpPoseTargetConditioningConfig | None = None,
        target_lead_clamp: UmiTargetLeadClampConfig | None = None,
        reanchor_on_external_move: bool = True,
        reanchor_linear_m: float = 0.02,
        reanchor_angular_rad: float = 0.05,
    ):
        if sample_hold_timeout_sec is not None and sample_stale_timeout_sec is not None:
            raise ValueError(
                "set only one of sample_hold_timeout_sec or deprecated sample_stale_timeout_sec"
            )
        if sample_hold_timeout_sec is None:
            sample_hold_timeout_sec = (
                0.05 if sample_stale_timeout_sec is None else sample_stale_timeout_sec
            )
        if max_linear_step_m < 0.0:
            raise ValueError("max_linear_step_m must be non-negative")
        if max_angular_step_rad < 0.0:
            raise ValueError("max_angular_step_rad must be non-negative")
        if int(input_moving_average_window) < 0:
            raise ValueError("input_moving_average_window must be non-negative")
        if deadband_linear_m < 0.0:
            raise ValueError("deadband_linear_m must be non-negative")
        if deadband_angular_rad < 0.0:
            raise ValueError("deadband_angular_rad must be non-negative")
        if sample_hold_timeout_sec <= 0.0:
            raise ValueError("sample_hold_timeout_sec must be positive")
        if deadman_release_grace_sec < 0.0:
            raise ValueError("deadman_release_grace_sec must be non-negative")
        if reanchor_linear_m < 0.0:
            raise ValueError("reanchor_linear_m must be non-negative")
        if reanchor_angular_rad < 0.0:
            raise ValueError("reanchor_angular_rad must be non-negative")
        self.left_reader = left_reader
        self.right_reader = right_reader
        self.max_linear_step_m = float(max_linear_step_m)
        self.max_angular_step_rad = float(max_angular_step_rad)
        self.deadband_linear_m = float(deadband_linear_m)
        self.deadband_angular_rad = float(deadband_angular_rad)
        self.linear_axis_signs = _signs3(linear_axis_signs, "linear_axis_signs")
        self.angular_axis_signs = _signs3(angular_axis_signs, "angular_axis_signs")
        self.gripper_offset = _tuple3(gripper_offset, "gripper_offset")
        self.r_align = _matrix3(r_align, "r_align")
        self.workspace_bounds = _workspace_bounds(workspace_bounds)
        self.sample_hold_timeout_sec = float(sample_hold_timeout_sec)
        self.timeout_sec = float(timeout_sec)
        self.deadman_release_grace_sec = float(deadman_release_grace_sec)
        self.tcp_pose_target_conditioning = (
            tcp_pose_target_conditioning if tcp_pose_target_conditioning is not None else _legacy_conditioning_config()
        )
        self.target_lead_clamp = target_lead_clamp if target_lead_clamp is not None else _legacy_lead_clamp_config()
        self.reanchor_on_external_move = bool(reanchor_on_external_move)
        self.reanchor_linear_m = float(reanchor_linear_m)
        self.reanchor_angular_rad = float(reanchor_angular_rad)
        self.input_moving_average_window = int(input_moving_average_window)
        self._left = _ArmTeleopState()
        self._right = _ArmTeleopState()
        self._left_ma = _PoseMovingAverage(self.input_moving_average_window)
        self._right_ma = _PoseMovingAverage(self.input_moving_average_window)
        # 수신 per-step 진단 로그 (POLICY_RUNNER_UMI_TELEOP_LOG 게이트, 기본 비활성)
        log_path = _resolve_teleop_log_path()
        self._step_log = _TeleopStepLogger(log_path) if log_path else None

    def next_intent(self, snapshot: StateSnapshot, now_monotonic: float) -> CommandIntent | None:
        left_pose, left_gripper, left_changed, left_diag = self._target_for_side(
            "left",
            self.left_reader,
            self._left,
            self._left_ma,
            snapshot,
            now_monotonic,
        )
        right_pose, right_gripper, right_changed, right_diag = self._target_for_side(
            "right",
            self.right_reader,
            self._right,
            self._right_ma,
            snapshot,
            now_monotonic,
        )
        if not left_changed and not right_changed:
            return None
        metadata = {
            "action_source": "umi_dual_cartesian",
            "source_conditioning_mode": self._conditioning_mode(),
        }
        sample_ns = max(
            [int(v) for v in (left_diag.get("input_sample_monotonic_ns"), right_diag.get("input_sample_monotonic_ns")) if v],
            default=0,
        )
        if sample_ns:
            metadata["input_sample_monotonic_ns"] = sample_ns
        intent = tcp_pose_target_stand_intent(
            left=left_pose,
            right=right_pose,
            left_gripper=left_gripper,
            right_gripper=right_gripper,
            timeout_sec=self.timeout_sec,
            tcp_target_profile=TCP_TARGET_PROFILE,
            metadata=metadata,
        )
        _annotate_arm_payload(intent.left, left_diag)
        _annotate_arm_payload(intent.right, right_diag)
        return intent

    @property
    def engaged(self) -> bool:
        """True while either arm's deadman clutch is latched. Used by
        TeleopMuxActionSource."""
        return self._left.was_armed or self._right.was_armed

    def reset_engagement(self) -> None:
        """Clear both arms' relative-init latches (mux suppression hook): a
        suppressed source must never accumulate clutch state, so a later
        takeover re-latches arm_init/pika_init fresh from the live snapshot.
        The moving-average buffers stay warm (they fill from the reader stream
        independently of the latches)."""
        _clear_latches(self._left)
        _clear_latches(self._right)

    def close(self) -> None:
        self.left_reader.close()
        self.right_reader.close()
        if self._step_log is not None:
            self._step_log.close()

    def _target_for_side(
        self,
        side: str,
        reader: UmiPoseReader,
        state: _ArmTeleopState,
        moving_average: _PoseMovingAverage,
        snapshot: StateSnapshot,
        now_monotonic: float,
    ) -> tuple[tuple[float, ...] | None, float | None, bool, dict[str, Any]]:
        diag = _conditioner_diag(self._conditioning_mode())
        sample = reader.read()
        # has_sample: the reader returned data at all (fresh packet OR its own
        # cached latest replayed between packets).
        # fresh_packet: that data is a genuinely NEW packet this step, detected
        # by the seq advancing past the last seq this arm consumed. A cached
        # replay keeps the same seq -> fresh_packet is False even though
        # has_sample is True (that is the reuse/hold case the old `fresh` flag
        # could not see, where dt_ms collapses to 0).
        has_sample = sample is not None
        fresh_packet = has_sample and sample.seq != state.last_seq
        if has_sample:
            state.last_seq = sample.seq
        if sample is None:
            sample = state.last_sample
            if sample is None or now_monotonic - sample.monotonic > self.sample_hold_timeout_sec:
                if state.was_armed:
                    if self._step_log is not None:
                        self._step_log.log_event(
                            side, "HOLD_TIMEOUT",
                            sample.monotonic if sample is not None else None, now_monotonic)
                    diag["stale_stop"] = True
                    _clear_latches(state)
                    return None, None, True, diag
                _clear_latches(state)
                return None, None, False, diag
        else:
            state.last_sample = sample

        if now_monotonic - sample.monotonic > self.sample_hold_timeout_sec:
            if state.was_armed:
                if self._step_log is not None:
                    self._step_log.log_event(side, "STALE", sample.monotonic, now_monotonic)
                diag["stale_stop"] = True
                _clear_latches(state)
                return None, None, True, diag
            _clear_latches(state)
            return None, None, False, diag

        # Input conditioning before any latch/compose math: the moving average
        # buffer also fills while the deadman is released (warm at engage).
        sample = moving_average.filter(sample)
        diag["fresh_packet"] = fresh_packet
        diag["reuse_tick"] = not fresh_packet
        diag["input_sample_monotonic_ns"] = int(sample.monotonic * 1_000_000_000)
        diag["input_age_ms"] = (now_monotonic - sample.monotonic) * 1000.0

        if not sample.deadman:
            if state.was_armed:
                # The foot-switch clutch deadman can drop spuriously under
                # vibration / fast motion (confirmed on the wire: bursts of
                # deadman=False during the fastest teleop). For absolute
                # TcpPoseTarget a brief drop is safe to ride out — keep
                # streaming the last latched setpoint so the arm holds in place
                # and the server stays fresh in TcpPoseTarget, instead of
                # tearing down to Hold (which forces a re-press and a
                # re-anchored resume). Only a drop sustained past
                # deadman_release_grace_sec is treated as a genuine release.
                if self.deadman_release_grace_sec > 0.0 and state.previous_target is not None:
                    if state.deadman_drop_mono is None:
                        state.deadman_drop_mono = now_monotonic
                    if now_monotonic - state.deadman_drop_mono < self.deadman_release_grace_sec:
                        if self._step_log is not None:
                            self._step_log.log_event(side, "HOLD", sample.monotonic, now_monotonic)
                        held = state.last_emitted_target_stand or state.previous_target
                        return held, _gripper_percent(sample.gripper), True, diag
                if self._step_log is not None:
                    self._step_log.log_event(side, "RELEASE", sample.monotonic, now_monotonic)
                _clear_latches(state)
                return None, _gripper_percent(sample.gripper), True, diag
            _clear_latches(state)
            return None, None, False, diag

        # Clutch is engaged this step: reset the brief-drop grace window so a
        # later spurious drop starts a fresh grace measurement.
        state.deadman_drop_mono = None

        pika_now = _tracker_transform(sample, self.gripper_offset, self.r_align)
        just_engaged = not state.was_armed
        if not state.was_armed:
            arm_init = _tcp_stand_transform(snapshot, side)
            if arm_init is None:
                _clear_latches(state)
                return None, None, False, diag
            state.arm_init = arm_init
            state.pika_init = pika_now
            state.previous_target = _transform_to_pose6(arm_init)
            state.was_armed = True
            if self.tcp_pose_target_conditioning.reset_on_engage:
                _reset_conditioner(state, state.previous_target, now_monotonic)
                diag["schedule_reset"] = True
        elif self.reanchor_on_external_move:
            # Absorb external motion under an idle-but-engaged clutch. The relative-init
            # anchor is latched once at engage (arm_init = TCP-at-engage); if the arm is
            # then moved out from under the operator by another authority (they press
            # InitMotion, or a flow move runs) while they are NOT commanding motion, the
            # held target stays at the engage-point pose and drags the arm back there the
            # instant it reaches the command slot (root cause of the InitMotion-then-revert
            # loop). Gate on BOTH: (a) device still ~at the engage anchor, i.e. the operator
            # has commanded ~no offset (so a legitimately-led target is never disturbed) AND
            # (b) live TCP displaced from our last emitted target beyond the thresholds (an
            # external move happened). Then re-latch the anchor from the live snapshot so the
            # hold tracks the new pose instead of reverting.
            dev_lin, dev_ang = _pose6_distance(
                _transform_to_pose6(pika_now), _transform_to_pose6(state.pika_init))
            operator_neutral = (dev_lin <= _REANCHOR_DEVICE_STILL_LINEAR_M
                                and dev_ang <= _REANCHOR_DEVICE_STILL_ANGULAR_RAD)
            anchor = state.last_emitted_target_stand or state.previous_target
            live_tf = _tcp_stand_transform(snapshot, side) if operator_neutral else None
            if live_tf is not None and anchor is not None:
                disp_lin, disp_ang = _pose6_distance(_transform_to_pose6(live_tf), anchor)
                if disp_lin > self.reanchor_linear_m or disp_ang > self.reanchor_angular_rad:
                    state.arm_init = live_tf
                    state.pika_init = pika_now
                    state.previous_target = _transform_to_pose6(live_tf)
                    if self.tcp_pose_target_conditioning.reset_on_engage:
                        _reset_conditioner(state, state.previous_target, now_monotonic)
                    diag["reanchor_external"] = True

        assert state.arm_init is not None
        assert state.pika_init is not None
        delta = self._apply_axis_signs(_compose(_inverse(state.pika_init), pika_now))
        target = _compose(state.arm_init, delta)
        raw_pose6 = _clamp_workspace(_transform_to_pose6(target), self.workspace_bounds)
        # 진단: deadband/클램프 전후를 모두 잡기 위해 prev 상태를 미리 보관
        prev_target = state.previous_target
        prev_deadband = state.deadband_target
        deadband_pose6 = self._apply_target_deadband(state, raw_pose6)
        raw_target_pose6 = self._clamp_against_previous(state, deadband_pose6)
        pose6 = self._condition_pose_target(
            state,
            raw_target_pose6,
            sample=sample,
            fresh_packet=fresh_packet,
            now_monotonic=now_monotonic,
            diag=diag,
        )
        pose6 = self._apply_target_lead_clamp(
            state,
            pose6,
            snapshot=snapshot,
            side=side,
            diag=diag,
        )
        _update_delta_diag(prev_target, raw_target_pose6, pose6, diag)
        if self._step_log is not None:
            self._step_log.log_step(
                side,
                sample_mono=sample.monotonic,
                now_monotonic=now_monotonic,
                has_sample=has_sample,
                fresh_packet=fresh_packet,
                sample_seq=sample.seq,
                just_engaged=just_engaged,
                prev_target=prev_target,
                prev_deadband=prev_deadband,
                raw_pose6=raw_pose6,
                deadband_pose6=deadband_pose6,
                applied_pose6=pose6,
                deadband_linear_m=self.deadband_linear_m,
                deadband_angular_rad=self.deadband_angular_rad,
                max_linear_step_m=self.max_linear_step_m,
                max_angular_step_rad=self.max_angular_step_rad,
                conditioner=diag,
            )
        state.previous_target = pose6
        state.last_emitted_target_stand = pose6
        return pose6, _gripper_percent(sample.gripper), True, diag

    def _apply_axis_signs(self, delta: _Transform) -> _Transform:
        """Mirror the latched relative delta per axis.

        Linear signs flip the delta translation componentwise; angular signs
        flip the rotation-axis components (via the quaternion vector part, so
        the angle is preserved and there is no RPY singularity). This is more
        general than an r_align conjugation: e.g. flipping x/y translation
        while flipping all of roll/pitch/yaw is not expressible as any rigid
        tool-frame alignment.
        """
        if self.linear_axis_signs == (1.0, 1.0, 1.0) and self.angular_axis_signs == (1.0, 1.0, 1.0):
            return delta
        sx, sy, sz = self.linear_axis_signs
        ax, ay, az = self.angular_axis_signs
        qx, qy, qz, qw = _matrix_to_quat(delta.rotation)
        return _Transform(
            (
                sx * delta.translation[0],
                sy * delta.translation[1],
                sz * delta.translation[2],
            ),
            _quat_to_matrix((ax * qx, ay * qy, az * qz, qw)),
        )

    def _apply_target_deadband(
        self,
        state: _ArmTeleopState,
        pose6: tuple[float, ...],
    ) -> tuple[float, ...]:
        if self.deadband_linear_m <= 0.0 and self.deadband_angular_rad <= 0.0:
            return pose6
        previous = state.deadband_target
        if previous is None:
            state.deadband_target = pose6
            return pose6
        # Freeze against the current deadband output so the command stays
        # bit-exact while the hand-held tracker only jitters in place.
        linear_dist = math.sqrt(
            (pose6[0] - previous[0]) ** 2
            + (pose6[1] - previous[1]) ** 2
            + (pose6[2] - previous[2]) ** 2
        )
        angular_dist = math.sqrt(
            _angle_diff(pose6[3], previous[3]) ** 2
            + _angle_diff(pose6[4], previous[4]) ** 2
            + _angle_diff(pose6[5], previous[5]) ** 2
        )
        if linear_dist <= self.deadband_linear_m and angular_dist <= self.deadband_angular_rad:
            return previous
        target = pose6
        state.deadband_target = target
        return target

    def _clamp_against_previous(
        self,
        state: _ArmTeleopState,
        pose6: tuple[float, ...],
    ) -> tuple[float, ...]:
        previous = state.previous_target
        if previous is None:
            return pose6
        delta = (
            pose6[0] - previous[0],
            pose6[1] - previous[1],
            pose6[2] - previous[2],
            _angle_diff(pose6[3], previous[3]),
            _angle_diff(pose6[4], previous[4]),
            _angle_diff(pose6[5], previous[5]),
        )
        clamped = clamp_pose_delta(delta, self.max_linear_step_m, self.max_angular_step_rad)
        return (
            previous[0] + clamped[0],
            previous[1] + clamped[1],
            previous[2] + clamped[2],
            _wrap_pi(previous[3] + clamped[3]),
            _wrap_pi(previous[4] + clamped[4]),
            _wrap_pi(previous[5] + clamped[5]),
        )

    def _conditioning_mode(self) -> str:
        config = self.tcp_pose_target_conditioning
        if not config.enable or config.mode == "none":
            return "none"
        return config.mode

    def _condition_pose_target(
        self,
        state: _ArmTeleopState,
        raw_target_pose6: tuple[float, ...],
        *,
        sample: UmiSample,
        fresh_packet: bool,
        now_monotonic: float,
        diag: dict[str, Any],
    ) -> tuple[float, ...]:
        config = self.tcp_pose_target_conditioning
        if not config.enable or config.mode != "foh_se3":
            state.last_raw_target_stand = raw_target_pose6
            state.last_emitted_target_stand = raw_target_pose6
            return raw_target_pose6

        now_ns = int(now_monotonic * 1_000_000_000)
        sample_ns = int(sample.monotonic * 1_000_000_000)
        if not state.conditioner_engaged:
            anchor = state.last_emitted_target_stand or state.previous_target or raw_target_pose6
            _reset_conditioner(state, anchor, now_monotonic)
            diag["schedule_reset"] = True

        if fresh_packet:
            horizon_sec = config.default_interpolation_horizon_sec
            if state.last_fresh_sample_time_ns is not None and sample_ns > state.last_fresh_sample_time_ns:
                horizon_sec = (sample_ns - state.last_fresh_sample_time_ns) / 1_000_000_000.0
            horizon_sec = max(
                config.min_interpolation_horizon_sec,
                min(config.max_interpolation_horizon_sec, horizon_sec),
            )
            start = state.last_emitted_target_stand or state.previous_target or raw_target_pose6
            state.schedule_start_target_stand = start
            state.schedule_end_target_stand = raw_target_pose6
            state.schedule_start_time_ns = now_ns
            state.schedule_end_time_ns = now_ns + max(1, int(horizon_sec * 1_000_000_000))
            state.last_fresh_sample_time_ns = sample_ns
            state.last_fresh_seq = sample.seq
            state.last_raw_target_stand = raw_target_pose6

        if (
            state.schedule_start_target_stand is None
            or state.schedule_end_target_stand is None
            or state.schedule_start_time_ns is None
            or state.schedule_end_time_ns is None
        ):
            emitted = state.last_emitted_target_stand or raw_target_pose6
            state.last_emitted_target_stand = emitted
            return emitted

        duration_ns = max(1, state.schedule_end_time_ns - state.schedule_start_time_ns)
        elapsed_ns = max(0, now_ns - state.schedule_start_time_ns)
        alpha = min(1.0, elapsed_ns / duration_ns)
        emitted = _interpolate_pose6_se3(
            state.schedule_start_target_stand,
            state.schedule_end_target_stand,
            alpha,
        )
        state.last_emitted_target_stand = emitted
        diag["cond_active"] = alpha < 1.0
        diag["cond_alpha"] = alpha
        diag["cond_horizon_ms"] = duration_ns / 1_000_000.0
        diag["cond_elapsed_ms"] = elapsed_ns / 1_000_000.0
        if alpha >= 1.0:
            state.schedule_start_target_stand = None
            state.schedule_end_target_stand = None
            state.schedule_start_time_ns = None
            state.schedule_end_time_ns = None
        return emitted

    def _apply_target_lead_clamp(
        self,
        state: _ArmTeleopState,
        pose6: tuple[float, ...],
        *,
        snapshot: StateSnapshot,
        side: str,
        diag: dict[str, Any],
    ) -> tuple[float, ...]:
        config = self.target_lead_clamp
        ref = _tcp_stand_transform(snapshot, side)
        if ref is None:
            return pose6
        ref_pose6 = _transform_to_pose6(ref)
        lead_before_m, lead_before_rad = _pose6_distance(ref_pose6, pose6)
        diag["lead_before_mm"] = lead_before_m * 1000.0
        diag["lead_before_deg"] = math.degrees(lead_before_rad)
        if not config.enable:
            diag["lead_after_mm"] = diag["lead_before_mm"]
            diag["lead_after_deg"] = diag["lead_before_deg"]
            return pose6
        clamped, axes = _clamp_pose6_lead(
            ref_pose6,
            pose6,
            max_translation_m=config.max_target_lead_m,
            max_rotation_rad=config.max_target_lead_rad,
        )
        lead_after_m, lead_after_rad = _pose6_distance(ref_pose6, clamped)
        diag["lead_after_mm"] = lead_after_m * 1000.0
        diag["lead_after_deg"] = math.degrees(lead_after_rad)
        if axes:
            diag["lead_clamp"] = True
            diag["lead_clamp_axes"] = axes
            if config.rebase_conditioner_on_clamp:
                _rebase_conditioner(state, clamped)
        return clamped


class MockUmiPoseReader:
    """Deterministic UMI reader for hardware-free policy_runner tests and demos."""

    def __init__(self, script: str | Iterable[Mapping[str, Any] | UmiSample | None]):
        self._samples = _script_to_samples(script)
        self.closed = False
        self._seq = 0

    def read(self) -> UmiSample | None:
        if not self._samples:
            return None
        sample = self._samples.pop(0)
        if sample is None:
            return None
        self._seq += 1
        return replace(sample, seq=self._seq)

    def close(self) -> None:
        self.closed = True


class UdpUmiPoseReader:
    """UDP JSON reader for one side of the Windows SteamVR UMI publisher schema."""

    _MAX_DRAIN_PACKETS = 64

    def __init__(
        self,
        endpoint: str,
        side: str,
        *,
        socket_factory: Any = socket.socket,
        monotonic_fn: Any = time.monotonic,
    ):
        if side not in {"left", "right"}:
            raise ValueError("side must be left or right")
        self.endpoint = endpoint
        self.side = side
        self._socket_factory = socket_factory
        self._monotonic_fn = monotonic_fn
        self._socket: socket.socket | None = None
        self._latest: UmiSample | None = None
        self._seq = 0

    def read(self) -> UmiSample | None:
        self._open()
        assert self._socket is not None
        for _ in range(self._MAX_DRAIN_PACKETS):
            try:
                data, _address = self._socket.recvfrom(65536)
            except BlockingIOError:
                break
            sample = _sample_from_udp_packet(data, self.side, self._monotonic_fn)
            if sample is None:
                # Packet arrived but carried no data for this side; do not count
                # it as a new sample for this reader (no seq bump, latest held).
                continue
            self._seq += 1
            self._latest = replace(sample, seq=self._seq)
        # Returns the cached latest between packets (reuse): the seq stays put so
        # the consumer can tell this apart from a genuinely fresh packet.
        return self._latest

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()
            self._socket = None

    def _open(self) -> None:
        if self._socket is not None:
            return
        endpoint = parse_udp_endpoint(self.endpoint)
        sock = self._socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((endpoint.host, endpoint.port))
        sock.setblocking(False)
        self._socket = sock


_NAMED_SCRIPTS: dict[str, tuple[Mapping[str, Any] | None, ...]] = {
    "pgmode_umi_smoke": (
        {
            "pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            "gripper": 0.0,
            "deadman": False,
            "monotonic": 0.00,
        },
        {
            "pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            "gripper": 0.25,
            "deadman": True,
            "monotonic": 0.01,
        },
        {
            "pose": [0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            "gripper": 0.50,
            "deadman": True,
            "monotonic": 0.02,
        },
        {
            "pose": [0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            "gripper": 50.0,
            "deadman": False,
            "monotonic": 0.03,
        },
        None,
    ),
}


def _clear_latches(state: _ArmTeleopState) -> None:
    state.arm_init = None
    state.pika_init = None
    state.previous_target = None
    state.last_sample = None
    state.last_seq = 0
    state.was_armed = False
    state.deadband_target = None
    state.deadman_drop_mono = None
    _reset_conditioner(state, None, 0.0)


def _reset_conditioner(
    state: _ArmTeleopState,
    anchor_pose6: tuple[float, ...] | None,
    now_monotonic: float,
) -> None:
    now_ns = int(now_monotonic * 1_000_000_000)
    state.conditioner_engaged = anchor_pose6 is not None
    state.last_raw_target_stand = anchor_pose6
    state.last_emitted_target_stand = anchor_pose6
    state.schedule_start_target_stand = anchor_pose6
    state.schedule_end_target_stand = anchor_pose6
    state.schedule_start_time_ns = now_ns if anchor_pose6 is not None else None
    state.schedule_end_time_ns = now_ns if anchor_pose6 is not None else None
    state.last_fresh_sample_time_ns = None
    state.last_fresh_seq = 0


def _rebase_conditioner(state: _ArmTeleopState, pose6: tuple[float, ...]) -> None:
    state.conditioner_engaged = True
    state.last_raw_target_stand = pose6
    state.last_emitted_target_stand = pose6
    state.schedule_start_target_stand = None
    state.schedule_end_target_stand = None
    state.schedule_start_time_ns = None
    state.schedule_end_time_ns = None


def _conditioner_diag(mode: str) -> dict[str, Any]:
    return {
        "profile": TCP_TARGET_PROFILE,
        "cond": mode,
        "cond_active": False,
        "cond_alpha": 0.0,
        "cond_horizon_ms": 0.0,
        "cond_elapsed_ms": 0.0,
        "fresh_packet": False,
        "reuse_tick": False,
        "raw_delta_mm": 0.0,
        "raw_delta_deg": 0.0,
        "emitted_delta_mm": 0.0,
        "emitted_delta_deg": 0.0,
        "raw_to_emitted_mm": 0.0,
        "raw_to_emitted_deg": 0.0,
        "lead_before_mm": -1.0,
        "lead_before_deg": -1.0,
        "lead_after_mm": -1.0,
        "lead_after_deg": -1.0,
        "lead_clamp": False,
        "lead_clamp_axes": "",
        "schedule_reset": False,
        "stale_stop": False,
    }


def _annotate_arm_payload(payload: dict[str, Any] | None, diag: Mapping[str, Any]) -> None:
    if not payload or payload.get("mode") != "TcpPoseTarget":
        return
    for key in (
        "input_sample_monotonic_ns",
        "input_age_ms",
        "fresh_packet",
        "reuse_tick",
        "source_conditioning_mode",
        "raw_target_stand",
        "emitted_target_stand",
        "raw_delta_mm",
        "raw_delta_deg",
        "emitted_delta_mm",
        "emitted_delta_deg",
        "raw_to_emitted_mm",
        "raw_to_emitted_deg",
        "target_lead_before_mm",
        "target_lead_before_deg",
        "target_lead_after_mm",
        "target_lead_after_deg",
        "lead_clamped",
    ):
        if key in diag:
            payload[key] = diag[key]


def _update_delta_diag(
    previous: tuple[float, ...] | None,
    raw_pose6: tuple[float, ...],
    emitted_pose6: tuple[float, ...],
    diag: dict[str, Any],
) -> None:
    base = previous or emitted_pose6
    raw_m, raw_rad = _pose6_distance(base, raw_pose6)
    emitted_m, emitted_rad = _pose6_distance(base, emitted_pose6)
    raw_to_emitted_m, raw_to_emitted_rad = _pose6_distance(raw_pose6, emitted_pose6)
    diag["raw_delta_mm"] = raw_m * 1000.0
    diag["raw_delta_deg"] = math.degrees(raw_rad)
    diag["emitted_delta_mm"] = emitted_m * 1000.0
    diag["emitted_delta_deg"] = math.degrees(emitted_rad)
    diag["raw_to_emitted_mm"] = raw_to_emitted_m * 1000.0
    diag["raw_to_emitted_deg"] = math.degrees(raw_to_emitted_rad)
    diag["source_conditioning_mode"] = diag.get("cond", "none")
    diag["raw_target_stand"] = [float(v) for v in raw_pose6]
    diag["emitted_target_stand"] = [float(v) for v in emitted_pose6]
    diag["target_lead_before_mm"] = diag.get("lead_before_mm", -1.0)
    diag["target_lead_before_deg"] = diag.get("lead_before_deg", -1.0)
    diag["target_lead_after_mm"] = diag.get("lead_after_mm", -1.0)
    diag["target_lead_after_deg"] = diag.get("lead_after_deg", -1.0)
    diag["lead_clamped"] = bool(diag.get("lead_clamp", False))


def _script_to_samples(script: str | Iterable[Mapping[str, Any] | UmiSample | None]) -> list[UmiSample | None]:
    if isinstance(script, str):
        try:
            entries = _NAMED_SCRIPTS[script]
        except KeyError as exc:
            known = ", ".join(sorted(_NAMED_SCRIPTS))
            raise ValueError(f"unknown UMI mock_script {script!r}; known scripts: {known}") from exc
    else:
        entries = tuple(script)
    return [_sample_from_mapping(entry, index) for index, entry in enumerate(entries)]


def _sample_from_udp_packet(data: bytes, side: str, monotonic_fn: Any) -> UmiSample | None:
    raw = json.loads(data.decode("utf-8"))
    if not isinstance(raw, Mapping):
        raise ValueError("UMI UDP packet must be a JSON object")
    side_raw = raw.get(side)
    if side_raw is None:
        return None
    if not isinstance(side_raw, Mapping):
        raise ValueError(f"UMI UDP packet {side} field must be an object")
    entry = dict(side_raw)
    # Staleness MUST be measured against the local arrival clock. A remote
    # publisher's "t" (and any per-side "monotonic") is from an unrelated
    # monotonic domain on another machine and is not comparable cross-host, so
    # we always stamp the sample with the consumer's own monotonic at parse time.
    entry["monotonic"] = monotonic_fn()
    return _sample_from_mapping(entry, 0)


def _sample_from_mapping(entry: Mapping[str, Any] | UmiSample | None, index: int) -> UmiSample | None:
    if entry is None:
        return None
    if isinstance(entry, UmiSample):
        return entry
    pose = entry.get("pose", entry.get("pose_xyzw"))
    if not isinstance(pose, (list, tuple)) or len(pose) != 7:
        raise ValueError("UMI sample pose must be [x,y,z,qx,qy,qz,qw]")
    return UmiSample(
        pose_xyzw=tuple(float(value) for value in pose),  # type: ignore[arg-type]
        gripper=float(entry.get("gripper", 0.0)),
        deadman=bool(entry.get("deadman", False)),
        monotonic=float(entry.get("monotonic", entry.get("timestamp_monotonic", index * 0.01))),
    )


def _tracker_transform(
    sample: UmiSample,
    gripper_offset: tuple[float, float, float],
    r_align: tuple[float, ...],
) -> _Transform:
    x, y, z, qx, qy, qz, qw = sample.pose_xyzw
    rotation = _mat_mul(_quat_to_matrix((qx, qy, qz, qw)), r_align)
    offset_world = _mat_vec(rotation, gripper_offset)
    return _Transform(
        (x + offset_world[0], y + offset_world[1], z + offset_world[2]),
        rotation,
    )


def _tcp_stand_transform(snapshot: StateSnapshot, side: str) -> _Transform | None:
    arm = snapshot.payload.get(side)
    if not isinstance(arm, Mapping):
        return None
    raw = arm.get("tcp_ref_stand") or arm.get("tcp_actual_stand") or arm.get("tcp_stand")
    if not isinstance(raw, Mapping):
        return None
    translation = (
        float(raw.get("x", 0.0)),
        float(raw.get("y", 0.0)),
        float(raw.get("z", 0.0)),
    )
    quat = raw.get("quaternion_xyzw")
    if isinstance(quat, (list, tuple)) and len(quat) == 4:
        rotation = _quat_to_matrix(tuple(float(value) for value in quat))
    elif all(key in raw for key in ("qx", "qy", "qz", "qw")):
        rotation = _quat_to_matrix(
            (
                float(raw.get("qx", 0.0)),
                float(raw.get("qy", 0.0)),
                float(raw.get("qz", 0.0)),
                float(raw.get("qw", 1.0)),
            )
        )
    else:
        rotation = _rpy_to_matrix(
            float(raw.get("rx", 0.0)),
            float(raw.get("ry", 0.0)),
            float(raw.get("rz", 0.0)),
        )
    return _Transform(translation, rotation)


def _compose(a: _Transform, b: _Transform) -> _Transform:
    rotated = _mat_vec(a.rotation, b.translation)
    return _Transform(
        (
            a.translation[0] + rotated[0],
            a.translation[1] + rotated[1],
            a.translation[2] + rotated[2],
        ),
        _mat_mul(a.rotation, b.rotation),
    )


def _inverse(transform: _Transform) -> _Transform:
    rotation_t = _transpose(transform.rotation)
    inv_t = _mat_vec(rotation_t, (-transform.translation[0], -transform.translation[1], -transform.translation[2]))
    return _Transform(inv_t, rotation_t)


def _transform_to_pose6(transform: _Transform) -> tuple[float, ...]:
    roll, pitch, yaw = _matrix_to_rpy(transform.rotation)
    return (
        transform.translation[0],
        transform.translation[1],
        transform.translation[2],
        roll,
        pitch,
        yaw,
    )


def _pose6_to_quat(pose6: Sequence[float]) -> tuple[float, float, float, float]:
    return _matrix_to_quat(_rpy_to_matrix(float(pose6[3]), float(pose6[4]), float(pose6[5])))


def _quat_normalize(q: Sequence[float]) -> tuple[float, float, float, float]:
    qx, qy, qz, qw = (float(v) for v in q)
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (qx / norm, qy / norm, qz / norm, qw / norm)


def _quat_slerp(
    q0: Sequence[float],
    q1: Sequence[float],
    alpha: float,
) -> tuple[float, float, float, float]:
    a = _quat_normalize(q0)
    b = _quat_normalize(q1)
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0.0:
        b = tuple(-v for v in b)  # type: ignore[assignment]
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    t = max(0.0, min(1.0, float(alpha)))
    if dot > 0.9995:
        return _quat_normalize(tuple((1.0 - t) * x + t * y for x, y in zip(a, b)))
    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * t
    s0 = math.cos(theta) - dot * math.sin(theta) / sin_theta_0
    s1 = math.sin(theta) / sin_theta_0
    return _quat_normalize(tuple(s0 * x + s1 * y for x, y in zip(a, b)))


def _interpolate_pose6_se3(
    start: tuple[float, ...],
    end: tuple[float, ...],
    alpha: float,
) -> tuple[float, ...]:
    t = max(0.0, min(1.0, float(alpha)))
    xyz = tuple(float(start[i]) + (float(end[i]) - float(start[i])) * t for i in range(3))
    quat = _quat_slerp(_pose6_to_quat(start), _pose6_to_quat(end), t)
    rpy = _matrix_to_rpy(_quat_to_matrix(quat))
    return (xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2])


def _quat_angle(q0: Sequence[float], q1: Sequence[float]) -> float:
    a = _quat_normalize(q0)
    b = _quat_normalize(q1)
    dot = abs(sum(x * y for x, y in zip(a, b)))
    dot = max(-1.0, min(1.0, dot))
    return 2.0 * math.acos(dot)


def _pose6_distance(a: Sequence[float], b: Sequence[float]) -> tuple[float, float]:
    linear = math.sqrt(sum((float(b[i]) - float(a[i])) ** 2 for i in range(3)))
    angular = _quat_angle(_pose6_to_quat(a), _pose6_to_quat(b))
    return linear, angular


def _clamp_pose6_lead(
    reference: tuple[float, ...],
    target: tuple[float, ...],
    *,
    max_translation_m: float,
    max_rotation_rad: float,
) -> tuple[tuple[float, ...], str]:
    x, y, z = float(target[0]), float(target[1]), float(target[2])
    axes = ""
    dx = (x - float(reference[0]), y - float(reference[1]), z - float(reference[2]))
    dist = math.sqrt(dx[0] * dx[0] + dx[1] * dx[1] + dx[2] * dx[2])
    if max_translation_m >= 0.0 and dist > max_translation_m > 0.0:
        scale = max_translation_m / dist
        x = float(reference[0]) + dx[0] * scale
        y = float(reference[1]) + dx[1] * scale
        z = float(reference[2]) + dx[2] * scale
        axes += "xyz"

    q_ref = _pose6_to_quat(reference)
    q_target = _pose6_to_quat(target)
    angle = _quat_angle(q_ref, q_target)
    roll, pitch, yaw = float(target[3]), float(target[4]), float(target[5])
    if max_rotation_rad >= 0.0 and angle > max_rotation_rad > 0.0:
        q_clamped = _quat_slerp(q_ref, q_target, max_rotation_rad / angle)
        roll, pitch, yaw = _matrix_to_rpy(_quat_to_matrix(q_clamped))
        axes += ("|" if axes else "") + "rpy"
    return (x, y, z, roll, pitch, yaw), axes


def _quat_to_matrix(q: Sequence[float]) -> tuple[float, ...]:
    if len(q) != 4:
        raise ValueError("quaternion must contain 4 values")
    qx, qy, qz, qw = (float(value) for value in q)
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0:
        raise ValueError("quaternion must be nonzero")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    return (
        1.0 - 2.0 * (qy * qy + qz * qz),
        2.0 * (qx * qy - qz * qw),
        2.0 * (qx * qz + qy * qw),
        2.0 * (qx * qy + qz * qw),
        1.0 - 2.0 * (qx * qx + qz * qz),
        2.0 * (qy * qz - qx * qw),
        2.0 * (qx * qz - qy * qw),
        2.0 * (qy * qz + qx * qw),
        1.0 - 2.0 * (qx * qx + qy * qy),
    )


def _average_quaternions(
    quats: Iterable[tuple[float, float, float, float]],
) -> tuple[float, float, float, float]:
    """Hemisphere-aligned normalized quaternion mean.

    Each quaternion is sign-aligned to the running sum before accumulating
    (q and -q encode the same rotation), then the sum is normalized. For the
    nearby orientations of a tracker stream this matches the geodesic mean to
    first order; for exactly two samples it is the exact slerp midpoint.
    """
    sx = sy = sz = sw = 0.0
    count = 0
    last = (0.0, 0.0, 0.0, 1.0)
    for qx, qy, qz, qw in quats:
        last = (qx, qy, qz, qw)
        if count > 0 and (sx * qx + sy * qy + sz * qz + sw * qw) < 0.0:
            qx, qy, qz, qw = -qx, -qy, -qz, -qw
        sx += qx
        sy += qy
        sz += qz
        sw += qw
        count += 1
    if count == 0:
        return last
    norm = math.sqrt(sx * sx + sy * sy + sz * sz + sw * sw)
    if norm < 1e-9:
        # Degenerate accumulation (antipodal spread) — fall back to the most
        # recent sample rather than emit a non-unit quaternion.
        return last
    return (sx / norm, sy / norm, sz / norm, sw / norm)


def _matrix_to_quat(m: Sequence[float]) -> tuple[float, float, float, float]:
    """Row-major 3x3 rotation matrix -> (qx, qy, qz, qw), Shepperd's method."""
    trace = float(m[0]) + float(m[4]) + float(m[8])
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return (
            (float(m[7]) - float(m[5])) / s,
            (float(m[2]) - float(m[6])) / s,
            (float(m[3]) - float(m[1])) / s,
            0.25 * s,
        )
    if float(m[0]) > float(m[4]) and float(m[0]) > float(m[8]):
        s = math.sqrt(1.0 + float(m[0]) - float(m[4]) - float(m[8])) * 2.0
        return (
            0.25 * s,
            (float(m[1]) + float(m[3])) / s,
            (float(m[2]) + float(m[6])) / s,
            (float(m[7]) - float(m[5])) / s,
        )
    if float(m[4]) > float(m[8]):
        s = math.sqrt(1.0 + float(m[4]) - float(m[0]) - float(m[8])) * 2.0
        return (
            (float(m[1]) + float(m[3])) / s,
            0.25 * s,
            (float(m[5]) + float(m[7])) / s,
            (float(m[2]) - float(m[6])) / s,
        )
    s = math.sqrt(1.0 + float(m[8]) - float(m[0]) - float(m[4])) * 2.0
    return (
        (float(m[2]) + float(m[6])) / s,
        (float(m[5]) + float(m[7])) / s,
        0.25 * s,
        (float(m[3]) - float(m[1])) / s,
    )


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> tuple[float, ...]:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        cy * cp,
        cy * sp * sr - sy * cr,
        cy * sp * cr + sy * sr,
        sy * cp,
        sy * sp * sr + cy * cr,
        sy * sp * cr - cy * sr,
        -sp,
        cp * sr,
        cp * cr,
    )


def _matrix_to_rpy(m: Sequence[float]) -> tuple[float, float, float]:
    sy = -float(m[6])
    sy = max(-1.0, min(1.0, sy))
    pitch = math.asin(sy)
    cp = math.cos(pitch)
    if abs(cp) > 1e-9:
        roll = math.atan2(float(m[7]), float(m[8]))
        yaw = math.atan2(float(m[3]), float(m[0]))
    else:
        roll = 0.0
        yaw = math.atan2(-float(m[1]), float(m[4]))
    return (_wrap_pi(roll), _wrap_pi(pitch), _wrap_pi(yaw))


def _mat_mul(a: Sequence[float], b: Sequence[float]) -> tuple[float, ...]:
    return tuple(
        sum(float(a[row * 3 + k]) * float(b[k * 3 + col]) for k in range(3))
        for row in range(3)
        for col in range(3)
    )


def _mat_vec(m: Sequence[float], v: Sequence[float]) -> tuple[float, float, float]:
    return (
        float(m[0]) * float(v[0]) + float(m[1]) * float(v[1]) + float(m[2]) * float(v[2]),
        float(m[3]) * float(v[0]) + float(m[4]) * float(v[1]) + float(m[5]) * float(v[2]),
        float(m[6]) * float(v[0]) + float(m[7]) * float(v[1]) + float(m[8]) * float(v[2]),
    )


def _transpose(m: Sequence[float]) -> tuple[float, ...]:
    return (
        float(m[0]),
        float(m[3]),
        float(m[6]),
        float(m[1]),
        float(m[4]),
        float(m[7]),
        float(m[2]),
        float(m[5]),
        float(m[8]),
    )


def _tuple3(value: Sequence[float], label: str) -> tuple[float, float, float]:
    if len(value) != 3:
        raise ValueError(f"{label} must contain 3 values")
    return (float(value[0]), float(value[1]), float(value[2]))


def _signs3(value: Sequence[float], label: str) -> tuple[float, float, float]:
    signs = _tuple3(value, label)
    if any(sign not in (-1.0, 1.0) for sign in signs):
        raise ValueError(f"{label} entries must be -1 or 1")
    return signs


def _matrix3(value: Sequence[float], label: str) -> tuple[float, ...]:
    if len(value) == 9:
        return tuple(float(v) for v in value)
    if len(value) == 3:
        return _rpy_to_matrix(float(value[0]), float(value[1]), float(value[2]))
    raise ValueError(f"{label} must contain 3 RPY values or 9 matrix values")


def _workspace_bounds(
    raw: Mapping[str, Sequence[float]] | Sequence[float] | None,
) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float]] | None:
    if raw is None:
        return None
    if isinstance(raw, Mapping):
        return (
            _bounds_pair(raw.get("x", (-math.inf, math.inf)), "workspace_bounds.x"),
            _bounds_pair(raw.get("y", (-math.inf, math.inf)), "workspace_bounds.y"),
            _bounds_pair(raw.get("z", (-math.inf, math.inf)), "workspace_bounds.z"),
        )
    if len(raw) != 6:
        raise ValueError("workspace_bounds must be [xmin,xmax,ymin,ymax,zmin,zmax]")
    return (
        (float(raw[0]), float(raw[1])),
        (float(raw[2]), float(raw[3])),
        (float(raw[4]), float(raw[5])),
    )


def _bounds_pair(value: Sequence[float], label: str) -> tuple[float, float]:
    if len(value) != 2:
        raise ValueError(f"{label} must contain [min,max]")
    lower, upper = float(value[0]), float(value[1])
    if lower > upper:
        raise ValueError(f"{label} min must be <= max")
    return lower, upper


def _clamp_workspace(
    pose6: tuple[float, ...],
    bounds: tuple[tuple[float, float], tuple[float, float], tuple[float, float]] | None,
) -> tuple[float, ...]:
    if bounds is None:
        return pose6
    return (
        max(bounds[0][0], min(bounds[0][1], pose6[0])),
        max(bounds[1][0], min(bounds[1][1], pose6[1])),
        max(bounds[2][0], min(bounds[2][1], pose6[2])),
        pose6[3],
        pose6[4],
        pose6[5],
    )


def _gripper_percent(value: float) -> float:
    value = float(value)
    if 0.0 <= value <= 1.0:
        value *= 100.0
    return max(0.0, min(100.0, value))


def _angle_diff(value: float, previous: float) -> float:
    return _wrap_pi(value - previous)


def _wrap_pi(value: float) -> float:
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi
