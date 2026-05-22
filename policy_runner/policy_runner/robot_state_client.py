from __future__ import annotations

import json
import socket
import threading
import time
from dataclasses import dataclass
from typing import Any


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


class RobotStateClient:
    """UDP JSON state subscriber with latest-snapshot cache."""

    def __init__(self, bind: str, stale_timeout_sec: float = 0.5):
        self.bind = bind
        self.stale_timeout_sec = stale_timeout_sec
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
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
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
