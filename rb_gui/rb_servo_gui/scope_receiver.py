from __future__ import annotations

import json
import math
import socket
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Mapping


SCOPE_SCHEMA = "robotics_lab.scope.v1"
DEFAULT_SCOPE_PORT = 50357
ARMS = ("left", "right")


@dataclass(frozen=True)
class ArmScopeSample:
    time_s: float
    q_sent_deg: tuple[float, ...]
    q_ref_deg: tuple[float, ...]
    q_actual_deg: tuple[float, ...]


@dataclass(frozen=True)
class ScopeArmBatch:
    t_robot_ns: tuple[int, ...]
    q_sent: tuple[tuple[float, ...], ...]
    q_ref: tuple[tuple[float, ...], ...]
    q_actual: tuple[tuple[float, ...], ...]


@dataclass(frozen=True)
class ScopeBatch:
    t_host_ns: tuple[int, ...]
    arms: Mapping[str, ScopeArmBatch]

    @property
    def n(self) -> int:
        return len(self.t_host_ns)


@dataclass(frozen=True)
class ScopeStats:
    received_batches: int
    invalid_packets: int
    received_samples: int
    dropped_samples: int
    batch_rate_hz: float | None
    latest_receive_age_sec: float | None
    bind_error: str | None
    buffer_samples: Mapping[str, int]


def _as_uint64_sequence(value: Any, *, length: int, field: str) -> tuple[int, ...]:
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"{field} must be a list of length {length}")
    out: list[int] = []
    for item in value:
        if not isinstance(item, int) or item < 0:
            raise ValueError(f"{field} values must be non-negative integers")
        out.append(item)
    return tuple(out)


def _as_joint_matrix(value: Any, *, length: int, field: str) -> tuple[tuple[float, ...], ...]:
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"{field} must be a list of length {length}")
    rows: list[tuple[float, ...]] = []
    for row in value:
        if not isinstance(row, list) or len(row) < 6:
            raise ValueError(f"{field} rows must contain at least 6 values")
        parsed = tuple(float(item) for item in row[:6])
        if not all(math.isfinite(item) for item in parsed):
            raise ValueError(f"{field} rows must be finite")
        rows.append(parsed)
    return tuple(rows)


def parse_scope_payload(payload: Mapping[str, Any]) -> ScopeBatch:
    if payload.get("schema") != SCOPE_SCHEMA:
        raise ValueError("unsupported scope schema")
    n_raw = payload.get("n")
    if not isinstance(n_raw, int) or n_raw < 0:
        raise ValueError("n must be a non-negative integer")
    n = int(n_raw)
    t_host_ns = _as_uint64_sequence(payload.get("t_host_ns"), length=n, field="t_host_ns")

    arms: dict[str, ScopeArmBatch] = {}
    for arm in ARMS:
        arm_payload = payload.get(arm)
        if not isinstance(arm_payload, Mapping):
            raise ValueError(f"{arm} scope payload missing")
        arms[arm] = ScopeArmBatch(
            t_robot_ns=_as_uint64_sequence(
                arm_payload.get("t_robot_ns"), length=n, field=f"{arm}.t_robot_ns"
            ),
            q_sent=_as_joint_matrix(
                arm_payload.get("q_sent"), length=n, field=f"{arm}.q_sent"
            ),
            q_ref=_as_joint_matrix(
                arm_payload.get("q_ref"), length=n, field=f"{arm}.q_ref"
            ),
            q_actual=_as_joint_matrix(
                arm_payload.get("q_actual"), length=n, field=f"{arm}.q_actual"
            ),
        )
    return ScopeBatch(t_host_ns=t_host_ns, arms=arms)


class ScopeStore:
    """Thread-safe ring buffer for the slim 500 Hz rb_servo_server scope stream."""

    def __init__(
        self, *, history_sec: float = 20.0, max_samples_per_arm: int = 30000
    ) -> None:
        self._history_sec = max(1.0, float(history_sec))
        self._max_samples_per_arm = max(8, int(max_samples_per_arm))
        self._samples: dict[str, deque[ArmScopeSample]] = {arm: deque() for arm in ARMS}
        self._batch_receive_times: deque[float] = deque(maxlen=256)
        self._lock = threading.Lock()
        self._received_batches = 0
        self._invalid_packets = 0
        self._received_samples = 0
        self._dropped_samples = 0
        self._last_receive_monotonic: float | None = None
        self._bind_error: str | None = None

    def set_history_sec(self, history_sec: float) -> None:
        with self._lock:
            self._history_sec = max(1.0, float(history_sec))
            self._trim_locked()

    def set_bind_error(self, message: str) -> None:
        with self._lock:
            self._bind_error = str(message)

    def update_from_json_bytes(
        self, payload: bytes, *, received_monotonic: float | None = None
    ) -> bool:
        try:
            decoded = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            with self._lock:
                self._invalid_packets += 1
            return False
        if not isinstance(decoded, Mapping):
            with self._lock:
                self._invalid_packets += 1
            return False
        try:
            batch = parse_scope_payload(decoded)
        except (TypeError, ValueError):
            with self._lock:
                self._invalid_packets += 1
            return False
        self.append_batch(batch, received_monotonic=received_monotonic)
        return True

    def append_batch(
        self, batch: ScopeBatch, *, received_monotonic: float | None = None
    ) -> None:
        now = float(received_monotonic) if received_monotonic is not None else time.monotonic()
        with self._lock:
            self._received_batches += 1
            self._last_receive_monotonic = now
            self._batch_receive_times.append(now)
            appended = 0
            dropped = 0
            for arm in ARMS:
                arm_batch = batch.arms[arm]
                arm_samples = self._samples[arm]
                for index, t_host_ns in enumerate(batch.t_host_ns):
                    time_s = float(t_host_ns) * 1e-9
                    if not math.isfinite(time_s):
                        dropped += 1
                        continue
                    if arm_samples and time_s <= arm_samples[-1].time_s:
                        if time_s < arm_samples[-1].time_s - 1.0:
                            arm_samples.clear()
                        else:
                            dropped += 1
                            continue
                    arm_samples.append(
                        ArmScopeSample(
                            time_s=time_s,
                            q_sent_deg=arm_batch.q_sent[index],
                            q_ref_deg=arm_batch.q_ref[index],
                            q_actual_deg=arm_batch.q_actual[index],
                        )
                    )
                    appended += 1
            self._received_samples += appended
            self._dropped_samples += dropped
            self._trim_locked()

    def snapshot_samples(self) -> dict[str, tuple[ArmScopeSample, ...]]:
        with self._lock:
            return {arm: tuple(self._samples[arm]) for arm in ARMS}

    def stats(self, *, now: float | None = None) -> ScopeStats:
        current = time.monotonic() if now is None else float(now)
        with self._lock:
            times = tuple(self._batch_receive_times)
            rate = None
            if len(times) >= 2:
                duration = times[-1] - times[0]
                if duration > 0.0:
                    rate = (len(times) - 1) / duration
            age = (
                current - self._last_receive_monotonic
                if self._last_receive_monotonic is not None
                else None
            )
            return ScopeStats(
                received_batches=self._received_batches,
                invalid_packets=self._invalid_packets,
                received_samples=self._received_samples,
                dropped_samples=self._dropped_samples,
                batch_rate_hz=rate,
                latest_receive_age_sec=age,
                bind_error=self._bind_error,
                buffer_samples={arm: len(self._samples[arm]) for arm in ARMS},
            )

    def _trim_locked(self) -> None:
        for arm_samples in self._samples.values():
            while len(arm_samples) > self._max_samples_per_arm:
                arm_samples.popleft()
                self._dropped_samples += 1
            if not arm_samples:
                continue
            cutoff = arm_samples[-1].time_s - self._history_sec
            while len(arm_samples) > 1 and arm_samples[0].time_s < cutoff:
                arm_samples.popleft()


class ScopeReceiver:
    def __init__(
        self, store: ScopeStore, host: str = "0.0.0.0", port: int = DEFAULT_SCOPE_PORT
    ) -> None:
        self.store = store
        self.host = host
        self.port = int(port)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._sock: socket.socket | None = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="rb-servo-scope-receiver", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def _run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock = sock
        sock.settimeout(0.2)
        try:
            sock.bind((self.host, self.port))
        except OSError as exc:
            self.store.set_bind_error(f"{self.host}:{self.port}: {exc}")
            try:
                sock.close()
            except OSError:
                pass
            return
        while not self._stop.is_set():
            try:
                payload, _ = sock.recvfrom(262144)
            except socket.timeout:
                continue
            except OSError:
                break
            self.store.update_from_json_bytes(payload, received_monotonic=time.monotonic())
