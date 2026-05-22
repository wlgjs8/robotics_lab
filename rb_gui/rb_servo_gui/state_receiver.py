from __future__ import annotations

import json
import socket
import threading
import time
from dataclasses import dataclass
from typing import Any

from .models import StateSnapshot


@dataclass
class StateStore:
    stale_after_sec: float = 0.5

    def __post_init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: StateSnapshot | None = None
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
        snapshot = StateSnapshot.parse(decoded, received_monotonic=received_monotonic)
        if snapshot is None:
            with self._lock:
                self.invalid_packets += 1
            return False
        with self._lock:
            self._latest = snapshot
            self.received_packets += 1
        return True

    def latest(self) -> StateSnapshot | None:
        with self._lock:
            return self._latest

    def is_stale(self, *, now: float | None = None) -> bool:
        latest = self.latest()
        return latest is None or latest.stale(now=now, threshold_sec=self.stale_after_sec)


class StateReceiver:
    def __init__(self, store: StateStore, host: str = "0.0.0.0", port: int = 50110) -> None:
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
        self._thread = threading.Thread(target=self._run, name="rb-servo-state-receiver", daemon=True)
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
