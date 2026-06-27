from __future__ import annotations

import json
import socket
import threading
import time
from dataclasses import dataclass
from typing import Any
from urllib.parse import urlparse

from .models import CircleOverlaySnapshot


def parse_udp_bind(endpoint: str | None) -> tuple[str, int] | None:
    if endpoint is None:
        return None
    value = endpoint.strip()
    if value == "" or value.lower() == "none":
        return None
    parsed = urlparse(value)
    if parsed.scheme != "udp" or not parsed.hostname or parsed.port is None:
        raise ValueError(f"circle overlay bind must be udp://host:port, got {endpoint!r}")
    if parsed.port < 1 or parsed.port > 65535:
        raise ValueError(f"circle overlay bind port out of range: {parsed.port}")
    return parsed.hostname, parsed.port


@dataclass
class CircleOverlayStore:
    stale_after_sec: float = 1.0

    def __post_init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: CircleOverlaySnapshot | None = None
        self.invalid_packets = 0
        self.received_packets = 0

    def update_from_json_bytes(self, payload: bytes, *, received_monotonic: float | None = None) -> bool:
        try:
            decoded: Any = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            with self._lock:
                self.invalid_packets += 1
            return False
        if not isinstance(decoded, dict):
            with self._lock:
                self.invalid_packets += 1
            return False
        snapshot = CircleOverlaySnapshot.parse(decoded, received_monotonic=received_monotonic)
        if snapshot is None:
            with self._lock:
                self.invalid_packets += 1
            return False
        with self._lock:
            self._latest = snapshot
            self.received_packets += 1
        return True

    def latest(self) -> CircleOverlaySnapshot | None:
        with self._lock:
            return self._latest

    def is_stale(self, *, now: float | None = None) -> bool:
        latest = self.latest()
        return latest is None or latest.stale(now=now, threshold_sec=self.stale_after_sec)


class CircleOverlayReceiver:
    def __init__(self, store: CircleOverlayStore, host: str = "0.0.0.0", port: int = 50261) -> None:
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
        self._thread = threading.Thread(target=self._run, name="rb-gui-circle-overlay-receiver", daemon=True)
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
        sock.bind((self.host, self.port))
        while not self._stop.is_set():
            try:
                payload, _ = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            self.store.update_from_json_bytes(payload, received_monotonic=time.monotonic())
