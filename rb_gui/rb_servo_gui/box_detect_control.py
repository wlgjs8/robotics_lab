from __future__ import annotations

import json
import socket
from dataclasses import dataclass
from typing import Any, Callable


BOX_DETECT_COMMAND_SCHEMA = "robotics_lab.box_detect_cmd.v1"
KNOWN_BOX_LABELS = ("green", "gray")


@dataclass(frozen=True)
class BoxDetectCommandResult:
    ok: bool
    message: str
    payload: dict[str, Any] | None = None


class BoxDetectCommandClient:
    """UDP sender for stereo_worker's box_detect_cmd.v1 (click-to-detect-and-lock trigger)."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 50387,
        *,
        socket_factory: Callable[..., socket.socket] = socket.socket,
    ) -> None:
        self.endpoint = (host, int(port))
        self.seq = 0
        self.sent_packets: list[dict[str, Any]] = []
        self._socket = socket_factory(socket.AF_INET, socket.SOCK_DGRAM)

    def send_detect_now(
        self, labels: list[str] | tuple[str, ...] | None = None
    ) -> BoxDetectCommandResult:
        if labels is not None:
            labels = list(labels)
            if not labels or any(label not in KNOWN_BOX_LABELS for label in labels):
                return BoxDetectCommandResult(False, f"invalid labels: {labels}")
        self.seq += 1
        payload: dict[str, Any] = {
            "schema": BOX_DETECT_COMMAND_SCHEMA,
            "seq": self.seq,
            "command": "detect_now",
        }
        if labels is not None:
            payload["labels"] = labels
        try:
            self._socket.sendto(
                json.dumps(payload, separators=(",", ":")).encode("utf-8"), self.endpoint
            )
        except OSError as exc:
            return BoxDetectCommandResult(False, f"box detect_now send failed: {exc}", payload)
        self.sent_packets.append(payload)
        return BoxDetectCommandResult(True, "box detect_now sent", payload)

    def close(self) -> None:
        self._socket.close()
