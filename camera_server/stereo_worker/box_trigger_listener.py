"""rb_gui 박스 재탐지(UDP JSON) 원샷 trigger 수신 → bounded detection burst 요청.

rb_gui의 "박스 재탐지" 버튼이 보내는 `detect_now` 명령을 stereo_worker가 별도 UDP
포트에서 받아 큐에 쌓는다. worker 메인 루프 연결은 별도 단계에서 수행하며, 이 모듈은
수신/파싱/드레인만 담당한다.
"""
from __future__ import annotations

import json
import socket
import threading
from collections import deque


TRIGGER_SCHEMA = "robotics_lab.box_detect_cmd.v1"
KNOWN_LABELS = ("green", "gray")


def _parse_endpoint(ep: str) -> tuple[str, int]:
    s = ep.split("://", 1)[-1]
    host, port = s.rsplit(":", 1)
    return host, int(port)


def _parse_trigger(data: bytes, known_labels=KNOWN_LABELS) -> dict | None:
    try:
        m = json.loads(data.decode("utf-8", "replace"))
    except Exception:
        return None
    if not isinstance(m, dict):
        return None
    if m.get("schema") != TRIGGER_SCHEMA:
        return None
    if m.get("command") != "detect_now":
        return None

    known = tuple(known_labels)
    if "labels" in m:
        raw_labels = m.get("labels")
        if not isinstance(raw_labels, list) or not raw_labels:
            return None
        labels = []
        for label in raw_labels:
            if label not in known:
                return None
            labels.append(label)
    else:
        labels = list(known)

    return {"seq": m.get("seq"), "command": "detect_now", "labels": labels}


class BoxTriggerListener:
    """rb_gui 박스 재탐지 trigger UDP JSON을 큐에 보관(스레드)."""

    def __init__(self, endpoint="udp://127.0.0.1:50387", known_labels=KNOWN_LABELS, maxlen=8):
        host, port = _parse_endpoint(endpoint)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((host, port))
        self._sock.settimeout(0.5)
        self._known_labels = tuple(known_labels)
        self._lock = threading.Lock()
        self._queue = deque(maxlen=maxlen)
        self._rx = 0
        self._run = True
        self._thr = threading.Thread(target=self._loop, daemon=True)
        self._thr.start()

    def _loop(self):
        while self._run:
            try:
                data, _ = self._sock.recvfrom(1 << 16)
            except socket.timeout:
                continue
            except OSError:
                break
            trigger = _parse_trigger(data, self._known_labels)
            if trigger is None:
                print("[box_trigger_listener] WARN: dropped invalid trigger payload", flush=True)
                continue
            with self._lock:
                self._queue.append(trigger)
                self._rx += 1

    def drain(self) -> list[dict]:
        """마지막 호출 이후 큐에 쌓인 trigger를 FIFO 순서로 모두 반환하고 비운다."""
        with self._lock:
            out = list(self._queue)
            self._queue.clear()
        return out

    @property
    def rx_count(self) -> int:
        return self._rx

    def close(self):
        self._run = False
        try:
            self._sock.close()
        except Exception:
            pass


__all__ = ["BoxTriggerListener", "TRIGGER_SCHEMA", "KNOWN_LABELS"]
