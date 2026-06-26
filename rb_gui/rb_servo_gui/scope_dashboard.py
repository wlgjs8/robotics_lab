from __future__ import annotations

import json
import mimetypes
import os
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

from .scope_receiver import ARMS, ArmScopeSample, ScopeStats, ScopeStore


DEFAULT_DASHBOARD_PORT = 8081
DASHBOARD_SCHEMA = "robotics_lab.scope.dashboard.v1"
STATIC_DIR = Path(__file__).resolve().parent / "static" / "scope"


def _public_host(host: str) -> str:
    if host in {"", "0.0.0.0", "::"}:
        return "127.0.0.1"
    return host


def _sample_after_cursor(sample: ArmScopeSample, cursor: float | None) -> bool:
    return cursor is None or sample.time_s > cursor


def _arm_delta(
    samples: tuple[ArmScopeSample, ...], cursor: float | None
) -> tuple[dict[str, Any], float | None]:
    if cursor is not None and samples and samples[-1].time_s < cursor:
        cursor = None
    selected = [sample for sample in samples if _sample_after_cursor(sample, cursor)]
    if selected:
        cursor = selected[-1].time_s
    return (
        {
            "t": [sample.time_s for sample in selected],
            "q_sent": [list(sample.q_sent_deg[:6]) for sample in selected],
            "q_ref": [list(sample.q_ref_deg[:6]) for sample in selected],
            "q_actual": [list(sample.q_actual_deg[:6]) for sample in selected],
        },
        cursor,
    )


def _stats_payload(stats: ScopeStats) -> dict[str, Any]:
    return {
        "received_batches": stats.received_batches,
        "invalid_packets": stats.invalid_packets,
        "received_samples": stats.received_samples,
        "dropped_samples": stats.dropped_samples,
        "batch_rate_hz": stats.batch_rate_hz,
        "latest_receive_age_sec": stats.latest_receive_age_sec,
        "bind_error": stats.bind_error,
        "buffer_samples": dict(stats.buffer_samples),
    }


def scope_delta_payload(
    scope_store: ScopeStore,
    cursors: dict[str, float | None],
) -> tuple[dict[str, Any], dict[str, float | None]]:
    samples_by_arm = scope_store.snapshot_samples()
    next_cursors = {arm: cursors.get(arm) for arm in ARMS}
    payload: dict[str, Any] = {
        "schema": DASHBOARD_SCHEMA,
        "host_time_ns": time.time_ns(),
        "arms": {},
        "stats": _stats_payload(scope_store.stats()),
    }
    for arm in ARMS:
        arm_payload, next_cursor = _arm_delta(
            tuple(samples_by_arm.get(arm, ())), next_cursors.get(arm)
        )
        payload["arms"][arm] = arm_payload
        next_cursors[arm] = next_cursor
    return payload, next_cursors


class _DashboardHttpServer(ThreadingHTTPServer):
    def __init__(self, *args: Any, dashboard: "ScopeDashboardServer", **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.dashboard = dashboard


class _ScopeDashboardHandler(BaseHTTPRequestHandler):
    server: _DashboardHttpServer

    def do_GET(self) -> None:  # noqa: N802 - http.server API
        path = urlparse(self.path).path
        if path == "/events":
            self._serve_events()
            return
        self._serve_static(path)

    def log_message(self, _format: str, *_args: Any) -> None:
        return

    def _serve_static(self, path: str) -> None:
        route = "/index.html" if path in {"", "/"} else path
        static_name = route.lstrip("/")
        allowed = {
            "index.html",
            "uPlot.iife.min.js",
            "uPlot.min.css",
            "README.md",
        }
        if static_name not in allowed:
            self.send_error(404)
            return
        file_path = STATIC_DIR / static_name
        try:
            data = file_path.read_bytes()
        except OSError:
            self.send_error(404)
            return
        mime = mimetypes.guess_type(file_path.name)[0] or "application/octet-stream"
        if file_path.suffix == ".js":
            mime = "application/javascript"
        elif file_path.suffix == ".css":
            mime = "text/css"
        elif file_path.suffix == ".html":
            mime = "text/html; charset=utf-8"
        self.send_response(200)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(data)

    def _serve_events(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()

        cursors: dict[str, float | None] = {arm: None for arm in ARMS}
        period = 1.0 / 60.0
        while not self.server.dashboard.stopped:
            payload, cursors = scope_delta_payload(self.server.dashboard.scope_store, cursors)
            line = (
                "data: "
                + json.dumps(payload, separators=(",", ":"), allow_nan=False)
                + "\n\n"
            ).encode("utf-8")
            try:
                self.wfile.write(line)
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                return
            time.sleep(period)


@dataclass
class ScopeDashboardServer:
    scope_store: ScopeStore
    host: str = "127.0.0.1"
    port: int = DEFAULT_DASHBOARD_PORT

    def __post_init__(self) -> None:
        self.host = str(self.host or "127.0.0.1")
        self.port = int(self.port)
        self._httpd: _DashboardHttpServer | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()

    @property
    def stopped(self) -> bool:
        return self._stop.is_set()

    @property
    def url(self) -> str:
        port = self.port
        if self._httpd is not None:
            port = int(self._httpd.server_address[1])
        return f"http://{_public_host(self.host)}:{port}"

    def start(self) -> "ScopeDashboardServer":
        if self._thread is not None and self._thread.is_alive():
            return self
        self._stop.clear()
        self._httpd = _DashboardHttpServer(
            (self.host, self.port),
            _ScopeDashboardHandler,
            dashboard=self,
        )
        self._thread = threading.Thread(
            target=self._httpd.serve_forever,
            name="rb-servo-scope-dashboard",
            daemon=True,
        )
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        httpd = self._httpd
        if httpd is not None:
            httpd.shutdown()
            httpd.server_close()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._httpd = None
        self._thread = None


def dashboard_host_from_env(default: str = "127.0.0.1") -> str:
    return os.environ.get("RB_GUI_SCOPE_DASHBOARD_HOST", default)
