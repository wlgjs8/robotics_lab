from __future__ import annotations

import json
import socket
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable


@dataclass(frozen=True)
class UdpEndpoint:
    host: str
    port: int


@dataclass(frozen=True)
class StateSnapshot:
    payload: dict[str, Any]
    received_monotonic: float

    def is_stale(self, now_monotonic: float, stale_timeout_sec: float) -> bool:
        return now_monotonic - self.received_monotonic > stale_timeout_sec


@dataclass(frozen=True)
class CommandSourceLeaseReadback:
    active: bool
    expired: bool
    enforce_lease: bool
    active_source_id: str | None
    active_session_id: str | None
    active_lease_token: str | None
    verdict: str | None
    reason: str | None
    command_requires_lease: bool | None
    command_has_lease: bool | None

    def matches(self, source_id: str, session_id: str, lease_token: str | None = None) -> bool:
        if not self.active or self.expired:
            return False
        if self.active_source_id != source_id or self.active_session_id != session_id:
            return False
        return lease_token is None or self.active_lease_token == lease_token


def parse_udp_endpoint(endpoint: str) -> UdpEndpoint:
    prefix = "udp://"
    if not endpoint.startswith(prefix):
        raise ValueError(f"only udp:// endpoints are supported: {endpoint}")
    rest = endpoint[len(prefix):]
    host, sep, port_text = rest.rpartition(":")
    if not sep or not host:
        raise ValueError(f"invalid UDP endpoint: {endpoint}")
    port = int(port_text)
    if port < 0 or port > 65535:
        raise ValueError(f"UDP port out of range: {endpoint}")
    return UdpEndpoint("127.0.0.1" if host == "localhost" else host, port)


def command_source_lease_from_snapshot(snapshot: StateSnapshot) -> CommandSourceLeaseReadback:
    raw = snapshot.payload.get("command_source", {})
    if not isinstance(raw, dict):
        raw = {}
    return CommandSourceLeaseReadback(
        active=bool(raw.get("active", False)),
        expired=bool(raw.get("expired", False)),
        enforce_lease=bool(raw.get("enforce_lease", False)),
        active_source_id=_optional_str(raw.get("active_source_id")),
        active_session_id=_optional_str(raw.get("active_session_id")),
        active_lease_token=_optional_str(raw.get("active_lease_token")),
        verdict=_optional_str(raw.get("verdict")),
        reason=_optional_str(raw.get("reason")),
        command_requires_lease=_optional_bool(raw.get("command_requires_lease")),
        command_has_lease=_optional_bool(raw.get("command_has_lease")),
    )


class StateStreamLeaseReadback:
    """Waits for command-source lease confirmation on the UDP state stream."""

    def __init__(self, state_client: Any):
        self.state_client = state_client

    def wait_for_active_lease(
        self,
        *,
        source_id: str,
        session_id: str,
        lease_token: str | None = None,
        timeout_sec: float = 1.0,
        poll_interval_sec: float = 0.01,
        monotonic_fn: Callable[[], float] = time.monotonic,
        sleep_fn: Callable[[float], None] = time.sleep,
    ) -> CommandSourceLeaseReadback:
        deadline = monotonic_fn() + max(timeout_sec, 0.0)
        last_readback: CommandSourceLeaseReadback | None = None
        while True:
            snapshot = getattr(self.state_client, "latest", None)
            if snapshot is not None:
                last_readback = command_source_lease_from_snapshot(snapshot)
                if last_readback.matches(source_id, session_id, lease_token):
                    return last_readback
            now = monotonic_fn()
            if now >= deadline:
                raise TimeoutError(_lease_timeout_message(source_id, session_id, last_readback))
            sleep_fn(min(max(poll_interval_sec, 0.0), deadline - now))


class RobotStateClient:
    """UDP JSON state subscriber with latest-snapshot cache."""

    def __init__(
        self,
        bind: str,
        stale_timeout_sec: float = 0.5,
        socket_factory: Callable[..., socket.socket] = socket.socket,
    ):
        self.bind = bind
        self.stale_timeout_sec = stale_timeout_sec
        self._socket_factory = socket_factory
        self._socket: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._running = False
        self._lock = threading.Lock()
        self._latest: StateSnapshot | None = None

    @property
    def latest(self) -> StateSnapshot | None:
        with self._lock:
            return self._latest

    @property
    def local_port(self) -> int:
        if self._socket is None:
            raise RuntimeError("state client is not open")
        return int(self._socket.getsockname()[1])

    def open(self) -> None:
        if self._socket is not None:
            return
        endpoint = parse_udp_endpoint(self.bind)
        sock = self._socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((endpoint.host, endpoint.port))
        self._socket = sock

    def close(self) -> None:
        self.stop()
        if self._socket is not None:
            self._socket.close()
            self._socket = None

    def start(self) -> None:
        self.open()
        if self._thread is not None:
            return
        self._running = True
        self._thread = threading.Thread(target=self._thread_main, name="policy-runner-state", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def poll_once(self, timeout_sec: float = 0.0) -> StateSnapshot | None:
        self.open()
        assert self._socket is not None
        self._socket.settimeout(timeout_sec)
        try:
            data, _addr = self._socket.recvfrom(65536)
        except socket.timeout:
            return None
        snapshot = StateSnapshot(json.loads(data.decode("utf-8")), time.monotonic())
        with self._lock:
            self._latest = snapshot
        return snapshot

    def is_latest_stale(self, now_monotonic: float | None = None) -> bool:
        snapshot = self.latest
        if snapshot is None:
            return True
        now = time.monotonic() if now_monotonic is None else now_monotonic
        return snapshot.is_stale(now, self.stale_timeout_sec)

    def _thread_main(self) -> None:
        while self._running:
            self.poll_once(timeout_sec=0.1)


def _optional_str(value: Any) -> str | None:
    return value if isinstance(value, str) and value else None


def _optional_bool(value: Any) -> bool | None:
    return value if isinstance(value, bool) else None


def _lease_timeout_message(
    source_id: str,
    session_id: str,
    last_readback: CommandSourceLeaseReadback | None,
) -> str:
    if last_readback is None:
        return (
            "command source lease readback timed out: "
            f"no state command_source observed for source_id={source_id} session_id={session_id}"
        )
    return (
        "command source lease readback timed out: "
        f"wanted source_id={source_id} session_id={session_id}, "
        f"last_active={last_readback.active} expired={last_readback.expired} "
        f"active_source_id={last_readback.active_source_id} "
        f"active_session_id={last_readback.active_session_id} "
        f"verdict={last_readback.verdict} reason={last_readback.reason}"
    )
