from __future__ import annotations

import json
import os
import socket
import uuid

import numpy as np


class ExternalBoxesSender:
    """UDP sender for rb_servo_server SetExternalBoxes keep-out updates."""

    def __init__(
        self,
        endpoint="127.0.0.1:50010",
        source_id="stereo_worker",
        labels=("green", "gray"),
        enabled=True,
    ):
        self.host, self.port = self._parse_endpoint(endpoint)
        self.source_id = source_id
        self.labels = tuple(labels)
        self.enabled = bool(enabled)
        self.session_id = uuid.uuid4().hex
        self._seq = 0
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    @staticmethod
    def _parse_endpoint(endpoint):
        value = str(endpoint).strip()
        if "://" in value:
            value = value.split("://", 1)[1]
        host, sep, port_text = value.rpartition(":")
        if not sep or not host or not port_text:
            raise ValueError(f"endpoint must be host:port, got {endpoint!r}")
        port = int(port_text)
        if port < 0 or port > 65535:
            raise ValueError(f"endpoint port out of range: {port}")
        return host, port

    @staticmethod
    def _identity_t():
        return [float(v) for v in np.eye(4, dtype=float).reshape(-1)]

    @staticmethod
    def _warn(message):
        try:
            os.write(2, f"[external_boxes_sender] WARN: {message}\n".encode("utf-8"))
        except Exception:
            pass

    @staticmethod
    def _flatten_t(box):
        try:
            T = np.asarray(box["T"], dtype=float).reshape(4, 4)
        except Exception:
            return None
        if not np.all(np.isfinite(T)):
            return None
        return [float(v) for v in T.reshape(-1)]

    def _find_box(self, boxes, label):
        if boxes is None:
            return None
        for box in boxes:
            if isinstance(box, dict) and box.get("label") == label:
                return box
        return None

    def build_payload(self, boxes):
        entries = []
        identity = self._identity_t()
        for label in self.labels:
            box = self._find_box(boxes, label)
            flat_t = self._flatten_t(box) if box is not None else None
            if flat_t is None:
                if box is not None:
                    self._warn(f"invalid T for label {label!r}; parking external box")
                entries.append({"label": label, "T": list(identity), "enable": False})
            else:
                entries.append({"label": label, "T": flat_t, "enable": True})
        return {
            "seq": int(self._seq),
            "source_id": self.source_id,
            "session_id": self.session_id,
            "mode": "SetExternalBoxes",
            "boxes": entries,
        }

    def send(self, boxes):
        if not self.enabled:
            return False
        try:
            self._seq += 1
            payload = self.build_payload(boxes)
            data = json.dumps(payload, separators=(",", ":"), allow_nan=False).encode("utf-8")
            self._sock.sendto(data, (self.host, self.port))
            return True
        except Exception as exc:  # noqa: BLE001 - this must never crash the perception loop.
            self._warn(f"SetExternalBoxes send failed: {exc}")
            return False

    def close(self):
        self._sock.close()


__all__ = ["ExternalBoxesSender"]
