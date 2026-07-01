#!/usr/bin/env python3
"""Serve a local joint kinematics dashboard from rb_servo_server UDP state.

The dashboard consumes rb_servo_server's existing state fanout and plots, per
arm, joint position plus finite-difference velocity, acceleration, and jerk.
It also writes a CSV log with the same derived q_actual signals.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import socket
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Mapping
from urllib.parse import urlparse


ARMS = ("left", "right")
JOINT_COUNT = 6
SCOPE_SCHEMA = "robotics_lab.scope.v1"
DEFAULT_STATE_LISTEN = "0.0.0.0:50376"
DEFAULT_HTTP_HOST = "0.0.0.0"
DEFAULT_HTTP_PORT = 8081
# Retain enough server-side history to back the dashboard's longest finite
# window (60 s) with margin so a fresh page load / reconnect snapshot fills the
# selected window immediately instead of growing from empty. The "ALL" window
# keeps accumulating client-side beyond this via the SSE delta stream.
DEFAULT_HISTORY_SEC = 120.0


@dataclass(frozen=True)
class ArmInput:
    q_sent: tuple[float, ...]
    q_ref: tuple[float, ...]
    q_actual: tuple[float, ...]


@dataclass(frozen=True)
class ArmSample:
    t: float
    q_sent: tuple[float, ...]
    q_ref: tuple[float, ...]
    q_actual: tuple[float, ...]
    velocity: tuple[float, ...]
    acceleration: tuple[float, ...]
    jerk: tuple[float, ...]


@dataclass
class _DerivativeState:
    t: float | None = None
    q_actual: tuple[float, ...] | None = None
    velocity: tuple[float, ...] | None = None
    acceleration: tuple[float, ...] | None = None


def _parse_host_port(value: str, *, default_host: str = "0.0.0.0") -> tuple[str, int]:
    raw = value.strip()
    if raw.startswith("udp://"):
        raw = raw[len("udp://") :]
    host, sep, port_raw = raw.rpartition(":")
    if not sep:
        return default_host, int(raw)
    return host or default_host, int(port_raw)


def _public_host(host: str) -> str:
    return "127.0.0.1" if host in {"", "0.0.0.0", "::"} else host


def _joint6(value: Any) -> tuple[float, ...] | None:
    if not isinstance(value, list) or len(value) < JOINT_COUNT:
        return None
    out: list[float] = []
    try:
        for item in value[:JOINT_COUNT]:
            parsed = float(item)
            if not math.isfinite(parsed):
                return None
            out.append(parsed)
    except (TypeError, ValueError):
        return None
    return tuple(out)


def _arm_input_from_state(arm_payload: Any) -> ArmInput | None:
    if not isinstance(arm_payload, Mapping):
        return None
    q_actual = _joint6(arm_payload.get("q_actual_deg"))
    if q_actual is None:
        return None
    q_sent = _joint6(arm_payload.get("q_sent_deg")) or q_actual
    q_ref = (
        _joint6(arm_payload.get("q_ref_deg"))
        or _joint6(arm_payload.get("q_target_deg"))
        or q_actual
    )
    return ArmInput(q_sent=q_sent, q_ref=q_ref, q_actual=q_actual)


def _arm_input_from_scope(arm_payload: Any, index: int) -> ArmInput | None:
    if not isinstance(arm_payload, Mapping):
        return None
    q_sent_rows = arm_payload.get("q_sent")
    q_ref_rows = arm_payload.get("q_ref")
    q_actual_rows = arm_payload.get("q_actual")
    if not (
        isinstance(q_sent_rows, list)
        and isinstance(q_ref_rows, list)
        and isinstance(q_actual_rows, list)
        and index < len(q_sent_rows)
        and index < len(q_ref_rows)
        and index < len(q_actual_rows)
    ):
        return None
    q_actual = _joint6(q_actual_rows[index])
    if q_actual is None:
        return None
    return ArmInput(
        q_sent=_joint6(q_sent_rows[index]) or q_actual,
        q_ref=_joint6(q_ref_rows[index]) or q_actual,
        q_actual=q_actual,
    )


def _host_time_s(payload: Mapping[str, Any], fallback: float) -> float:
    raw = payload.get("host_time_ns")
    if isinstance(raw, (int, float)) and math.isfinite(float(raw)):
        return float(raw) * 1e-9
    return fallback


def extract_packet_samples(payload: Mapping[str, Any], *, fallback_time_s: float) -> list[tuple[float, dict[str, ArmInput]]]:
    """Extract one or more timestamped arm samples from state or scope JSON."""
    if payload.get("schema") == SCOPE_SCHEMA:
        n_raw = payload.get("n")
        times = payload.get("t_host_ns")
        if not isinstance(n_raw, int) or n_raw <= 0 or not isinstance(times, list):
            return []
        out: list[tuple[float, dict[str, ArmInput]]] = []
        for index in range(min(n_raw, len(times))):
            t_raw = times[index]
            if not isinstance(t_raw, (int, float)):
                continue
            t = float(t_raw) * 1e-9
            if not math.isfinite(t):
                continue
            arms: dict[str, ArmInput] = {}
            for arm in ARMS:
                arm_input = _arm_input_from_scope(payload.get(arm), index)
                if arm_input is not None:
                    arms[arm] = arm_input
            if arms:
                out.append((t, arms))
        return out

    t = _host_time_s(payload, fallback_time_s)
    arms = {}
    for arm in ARMS:
        arm_input = _arm_input_from_state(payload.get(arm))
        if arm_input is not None:
            arms[arm] = arm_input
    return [(t, arms)] if arms else []


class DashboardStore:
    def __init__(
        self,
        *,
        history_sec: float = DEFAULT_HISTORY_SEC,
        # ~80 s at the 500 Hz servo tick rate (scope publishes every control-loop
        # step), enough to back the 60 s window and a long ALL view.
        max_samples_per_arm: int = 40000,
        csv_path: str | None = None,
    ) -> None:
        self.history_sec = max(1.0, float(history_sec))
        self.max_samples_per_arm = max(16, int(max_samples_per_arm))
        self._samples: dict[str, deque[ArmSample]] = {arm: deque() for arm in ARMS}
        self._derivative: dict[str, _DerivativeState] = {arm: _DerivativeState() for arm in ARMS}
        self._lock = threading.Lock()
        self._received_packets = 0
        self._invalid_packets = 0
        self._received_samples = 0
        self._last_receive_monotonic: float | None = None
        self._receive_times: deque[float] = deque(maxlen=256)
        self._bind_error: str | None = None
        self._csv_file = None
        self._csv_writer: csv.writer | None = None
        self.csv_path = csv_path
        if csv_path:
            path = Path(csv_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            self._csv_file = path.open("w", newline="")
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow(self._csv_header())
            self._csv_file.flush()

    def close(self) -> None:
        if self._csv_file is not None:
            self._csv_file.flush()
            self._csv_file.close()
            self._csv_file = None

    def set_bind_error(self, message: str) -> None:
        with self._lock:
            self._bind_error = str(message)

    def update_from_json_bytes(self, data: bytes, *, received_monotonic: float | None = None) -> bool:
        now = time.monotonic() if received_monotonic is None else float(received_monotonic)
        try:
            payload = json.loads(data.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            with self._lock:
                self._invalid_packets += 1
            return False
        if not isinstance(payload, Mapping):
            with self._lock:
                self._invalid_packets += 1
            return False
        samples = extract_packet_samples(payload, fallback_time_s=time.time())
        if not samples:
            with self._lock:
                self._invalid_packets += 1
            return False
        with self._lock:
            self._received_packets += 1
            self._last_receive_monotonic = now
            self._receive_times.append(now)
            for t, arms in samples:
                self._append_sample_locked(t, arms)
        return True

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return self._snapshot_locked(cursor_by_arm={arm: None for arm in ARMS})[0]

    def delta(self, cursor_by_arm: Mapping[str, float | None]) -> tuple[dict[str, Any], dict[str, float | None]]:
        with self._lock:
            return self._snapshot_locked(cursor_by_arm=cursor_by_arm)

    def _append_sample_locked(self, t: float, arms: Mapping[str, ArmInput]) -> None:
        derived_by_arm: dict[str, ArmSample] = {}
        for arm, arm_input in arms.items():
            sample = self._derive_arm_locked(arm, t, arm_input)
            self._samples[arm].append(sample)
            derived_by_arm[arm] = sample
            self._received_samples += 1
            self._trim_arm_locked(arm)
        if self._csv_writer is not None:
            self._csv_writer.writerow(self._csv_row(t, derived_by_arm))
            if self._csv_file is not None:
                self._csv_file.flush()

    def _derive_arm_locked(self, arm: str, t: float, arm_input: ArmInput) -> ArmSample:
        state = self._derivative[arm]
        zero = (0.0,) * JOINT_COUNT
        velocity = zero
        acceleration = zero
        jerk = zero
        if state.t is not None and state.q_actual is not None:
            dt = t - state.t
            if 1e-6 <= dt <= 0.5:
                velocity = tuple(
                    (arm_input.q_actual[i] - state.q_actual[i]) / dt
                    for i in range(JOINT_COUNT)
                )
                if state.velocity is not None:
                    acceleration = tuple(
                        (velocity[i] - state.velocity[i]) / dt
                        for i in range(JOINT_COUNT)
                    )
                if state.acceleration is not None:
                    jerk = tuple(
                        (acceleration[i] - state.acceleration[i]) / dt
                        for i in range(JOINT_COUNT)
                    )
        state.t = t
        state.q_actual = arm_input.q_actual
        state.velocity = velocity
        state.acceleration = acceleration
        return ArmSample(
            t=t,
            q_sent=arm_input.q_sent,
            q_ref=arm_input.q_ref,
            q_actual=arm_input.q_actual,
            velocity=velocity,
            acceleration=acceleration,
            jerk=jerk,
        )

    def _trim_arm_locked(self, arm: str) -> None:
        samples = self._samples[arm]
        while len(samples) > self.max_samples_per_arm:
            samples.popleft()
        if samples:
            cutoff = samples[-1].t - self.history_sec
            while len(samples) > 1 and samples[0].t < cutoff:
                samples.popleft()

    def _snapshot_locked(self, cursor_by_arm: Mapping[str, float | None]) -> tuple[dict[str, Any], dict[str, float | None]]:
        arms: dict[str, Any] = {}
        next_cursors: dict[str, float | None] = {}
        for arm in ARMS:
            cursor = cursor_by_arm.get(arm)
            selected = [
                sample
                for sample in self._samples[arm]
                if cursor is None or sample.t > cursor
            ]
            if selected:
                next_cursors[arm] = selected[-1].t
            else:
                next_cursors[arm] = cursor
            arms[arm] = {
                "t": [sample.t for sample in selected],
                "q_sent": [list(sample.q_sent) for sample in selected],
                "q_ref": [list(sample.q_ref) for sample in selected],
                "q_actual": [list(sample.q_actual) for sample in selected],
                "velocity": [list(sample.velocity) for sample in selected],
                "acceleration": [list(sample.acceleration) for sample in selected],
                "jerk": [list(sample.jerk) for sample in selected],
            }
        return (
            {
                "schema": "robotics_lab.servo_scope_dashboard.v1",
                "host_time_ns": time.time_ns(),
                "arms": arms,
                "stats": self._stats_locked(),
            },
            next_cursors,
        )

    def _stats_locked(self) -> dict[str, Any]:
        now = time.monotonic()
        times = tuple(self._receive_times)
        rate = None
        if len(times) >= 2 and times[-1] > times[0]:
            rate = (len(times) - 1) / (times[-1] - times[0])
        age = None if self._last_receive_monotonic is None else now - self._last_receive_monotonic
        return {
            "received_packets": self._received_packets,
            "invalid_packets": self._invalid_packets,
            "received_samples": self._received_samples,
            "packet_rate_hz": rate,
            "latest_receive_age_sec": age,
            "bind_error": self._bind_error,
            "buffer_samples": {arm: len(self._samples[arm]) for arm in ARMS},
            "csv_path": self.csv_path,
        }

    def _csv_header(self) -> list[str]:
        fields = ["t_sec"]
        for arm in ARMS:
            for prefix in ("q_actual_deg", "dq_actual_deg_s", "ddq_actual_deg_s2", "jerk_actual_deg_s3", "q_sent_deg", "q_ref_deg"):
                for joint in range(JOINT_COUNT):
                    fields.append(f"{arm}_{prefix}_{joint + 1}")
        return fields

    def _csv_row(self, t: float, samples: Mapping[str, ArmSample]) -> list[Any]:
        row: list[Any] = [f"{t:.9f}"]
        for arm in ARMS:
            sample = samples.get(arm)
            if sample is None:
                row.extend([""] * (JOINT_COUNT * 6))
                continue
            for values in (
                sample.q_actual,
                sample.velocity,
                sample.acceleration,
                sample.jerk,
                sample.q_sent,
                sample.q_ref,
            ):
                row.extend(f"{value:.9g}" for value in values)
        return row


class StateFanoutReceiver:
    def __init__(self, store: DashboardStore, host: str, port: int) -> None:
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
        self._thread = threading.Thread(target=self._run, name="servo-scope-udp", daemon=True)
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
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(0.2)
        self._sock = sock
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
                data, _ = sock.recvfrom(262144)
            except socket.timeout:
                continue
            except OSError:
                break
            self.store.update_from_json_bytes(data, received_monotonic=time.monotonic())


INDEX_HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>RB Joint Scope</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #101419;
      --panel: #171d24;
      --plot: #0d1117;
      --grid: #2a3440;
      --text: #e7edf5;
      --muted: #97a3b3;
      --sent: #5aa0ff;
      --ref: #a98bff;
      --actual: #ff9b54;
    }
    * { box-sizing: border-box; }
    html, body {
      width: 100%;
      height: 100%;
      margin: 0;
      overflow: hidden;
      background: var(--bg);
      color: var(--text);
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      font-size: 13px;
    }
    body { display: grid; grid-template-rows: 42px minmax(0, 1fr); }
    #toolbar {
      min-width: 0;
      display: flex;
      align-items: center;
      gap: 12px;
      padding: 6px 10px;
      background: var(--panel);
      border-bottom: 1px solid var(--grid);
      white-space: nowrap;
    }
    .brand { font-weight: 700; margin-right: 4px; }
    label { display: inline-flex; align-items: center; gap: 5px; color: var(--muted); }
    select, button {
      height: 28px;
      color: var(--text);
      background: #10161d;
      border: 1px solid var(--grid);
      border-radius: 4px;
      padding: 0 8px;
      font: inherit;
    }
    button { cursor: pointer; }
    input[type="checkbox"] { margin: 0; }
    .sent { color: var(--sent); }
    .ref { color: var(--ref); }
    .actual { color: var(--actual); }
    #status {
      margin-left: auto;
      min-width: 280px;
      overflow: hidden;
      text-align: right;
      text-overflow: ellipsis;
      color: var(--muted);
    }
    #plots {
      min-height: 0;
      display: grid;
      grid-template-columns: minmax(0, 1fr) minmax(0, 1fr);
      grid-template-rows: repeat(4, minmax(0, 1fr));
      gap: 1px;
      background: var(--grid);
    }
    .plot {
      min-width: 0;
      min-height: 0;
      display: grid;
      grid-template-rows: 24px minmax(0, 1fr);
      background: var(--plot);
    }
    .title {
      display: flex;
      align-items: center;
      padding: 0 8px;
      color: var(--muted);
      border-bottom: 1px solid rgba(255,255,255,0.06);
      font-size: 12px;
    }
    canvas { width: 100%; height: 100%; display: block; }
  </style>
</head>
<body>
  <div id="toolbar">
    <div class="brand">RB Joint Scope</div>
    <label>Joint
      <select id="joint">
        <option value="0">J1</option><option value="1">J2</option><option value="2">J3</option>
        <option value="3">J4</option><option value="4">J5</option><option value="5">J6</option>
      </select>
    </label>
    <label>Window
      <select id="window">
        <option value="30" selected>30 s</option><option value="60">60 s</option><option value="all">ALL</option>
      </select>
    </label>
    <label class="sent"><input id="trace-sent" type="checkbox" checked> q_sent</label>
    <label class="ref"><input id="trace-ref" type="checkbox" checked> q_ref</label>
    <label class="actual"><input id="trace-actual" type="checkbox" checked> q_actual</label>
    <label><input id="smooth" type="checkbox" checked> Smooth</label>
    <label title="Position plots show tracking error (q_sent/q_ref minus q_actual, actual=0 baseline)"><input id="pos-error" type="checkbox"> Pos err</label>
    <button id="reset" type="button">Live</button>
    <div id="status">connecting...</div>
  </div>
  <div id="plots"></div>
  <script>
  (() => {
    "use strict";
    const ARMS = ["left", "right"];
    const ARM_LABEL = {left: "Left arm", right: "Right arm"};
    const METRICS = [
      {key: "position", label: "Position", unit: "deg"},
      {key: "velocity", label: "Velocity", unit: "deg/s"},
      {key: "acceleration", label: "Acceleration", unit: "deg/s^2"},
      {key: "jerk", label: "Jerk", unit: "deg/s^3"},
    ];
    const TRACE = {
      sent: {key: "q_sent", label: "q_sent", color: "#5aa0ff"},
      ref: {key: "q_ref", label: "q_ref", color: "#a98bff"},
      actual: {key: "q_actual", label: "q_actual", color: "#ff9b54"},
    };
    const buffers = {
      left: {t: [], q_sent: [], q_ref: [], q_actual: [], velocity: [], acceleration: [], jerk: []},
      right: {t: [], q_sent: [], q_ref: [], q_actual: [], velocity: [], acceleration: [], jerk: []},
    };
    const plots = [];
    const eventTimes = [];
    let latestStats = null;
    let renderQueued = false;
    // Manual x-axis (time) zoom shared by ALL plots. `view` is an absolute-time
    // window {min,max} (same units as buffer t) that freezes/overrides the live
    // window; null => live mode driven by the Window selector. `drag` holds an
    // in-progress left-drag box-zoom selection; `pan` an in-progress
    // middle-button (wheel-press) grab-to-scroll along the time axis. All are
    // global so the 8 plots stay synchronized on a single time domain. "Live" /
    // double-click clears.
    let view = null;
    let drag = null;
    let pan = null;
    const el = {
      plots: document.getElementById("plots"),
      joint: document.getElementById("joint"),
      window: document.getElementById("window"),
      smooth: document.getElementById("smooth"),
      posError: document.getElementById("pos-error"),
      sent: document.getElementById("trace-sent"),
      ref: document.getElementById("trace-ref"),
      actual: document.getElementById("trace-actual"),
      reset: document.getElementById("reset"),
      status: document.getElementById("status"),
    };

    function windowSec() {
      // "all" => Infinity: no time-based trim/cull; the live x-axis falls back to
      // the full retained data span (see xDomain/dataMaxAge). The client buffer
      // cap in trimArm still bounds memory.
      if (el.window.value === "all") return Infinity;
      return Math.max(0.5, Number(el.window.value) || 30);
    }
    function dataMaxAge() {
      // Largest (globalEnd - earliest sample) across both arms.
      const end = globalEndTime();
      let age = 0;
      for (const arm of ARMS) {
        const b = buffers[arm].t;
        if (b.length) age = Math.max(age, end - b[0]);
      }
      return age;
    }
    // The single absolute-time x-domain shared by every plot. While a drag
    // box-zoom is in progress we freeze on the domain captured at mousedown so
    // the selection band and mapping stay stable as data streams in. A committed
    // manual zoom (`view`) overrides the live window; otherwise the domain is
    // the live Window selection ending at the latest sample.
    function xDomain() {
      if (drag) return drag.dom;
      const end = globalEndTime();
      if (view) return {min: view.min, max: view.max, end};
      const w = windowSec();
      const span = Number.isFinite(w) ? w : (dataMaxAge() || 1);
      return {min: end - span, max: end, end};
    }
    function jointIndex() { return Math.max(0, Math.min(5, Number(el.joint.value) || 0)); }
    function clearArm(arm) {
      const b = buffers[arm];
      for (const key of Object.keys(b)) b[key].length = 0;
    }
    function validRow(row) {
      return Array.isArray(row) && row.length >= 6 && row.slice(0, 6).every(Number.isFinite);
    }
    function appendArm(arm, payload) {
      if (!payload || !Array.isArray(payload.t)) return;
      const b = buffers[arm];
      for (let i = 0; i < payload.t.length; i++) {
        const t = Number(payload.t[i]);
        if (!Number.isFinite(t)) continue;
        if (b.t.length > 0 && t <= b.t[b.t.length - 1]) {
          if (t < b.t[b.t.length - 1] - 1.0) clearArm(arm);
          else continue;
        }
        const rows = {};
        let ok = true;
        for (const key of ["q_sent", "q_ref", "q_actual", "velocity", "acceleration", "jerk"]) {
          rows[key] = payload[key] && payload[key][i];
          if (!validRow(rows[key])) ok = false;
        }
        if (!ok) continue;
        b.t.push(t);
        for (const key of ["q_sent", "q_ref", "q_actual", "velocity", "acceleration", "jerk"]) {
          b[key].push(rows[key].slice(0, 6).map(Number));
        }
      }
      trimArm(arm);
    }
    function trimArm(arm) {
      const b = buffers[arm];
      if (!b.t.length) return;
      // While a manual zoom is frozen, keep history (bounded only by the sample
      // cap) so the frozen window doesn't get trimmed out from under the user.
      if (!view) {
        const cutoff = b.t[b.t.length - 1] - windowSec();
        let drop = 0;
        while (drop < b.t.length - 1 && b.t[drop] < cutoff) drop++;
        if (drop > 0) {
          for (const key of Object.keys(b)) b[key].splice(0, drop);
        }
      }
      const extra = Math.max(0, b.t.length - 40000);  // ~80 s at the 500 Hz servo tick rate
      if (extra > 0) {
        for (const key of Object.keys(b)) b[key].splice(0, extra);
      }
    }
    function selectedTrace(rows, j) {
      return rows.map(row => Array.isArray(row) && row.length > j ? Number(row[j]) : NaN);
    }
    function finiteDiff(values, times) {
      const out = new Array(values.length).fill(NaN);
      for (let i = 1; i < values.length; i++) {
        const dt = times[i] - times[i - 1];
        if (dt > 0 && Number.isFinite(values[i]) && Number.isFinite(values[i - 1])) {
          out[i] = (values[i] - values[i - 1]) / dt;
        }
      }
      return out;
    }
    function movingAverage(values, width) {
      if (width <= 1) return values;
      const out = [];
      for (let i = 0; i < values.length; i++) {
        let sum = 0, n = 0;
        const start = Math.max(0, i - width + 1);
        for (let j = start; j <= i; j++) {
          if (Number.isFinite(values[j])) { sum += values[j]; n++; }
        }
        out.push(n ? sum / n : NaN);
      }
      return out;
    }
    function axisDecimals(step) {
      // Decimal places so a tick step is just distinguishable; grows as you zoom
      // in (down to 1 ms / fine joint units) and shrinks when zoomed out.
      if (!Number.isFinite(step) || step <= 0) return 1;
      return Math.max(0, Math.min(6, Math.ceil(-Math.log10(step)) + 1));
    }
    function fmtSci(v) {
      // Y-axis labels in scientific notation (e.g. 1.2e-3) so magnitude is read
      // off the exponent regardless of zoom.
      if (!Number.isFinite(v)) return "";
      if (v === 0) return "0";
      return v.toExponential(1);
    }
    function globalEndTime() {
      let t = -Infinity;
      for (const arm of ARMS) {
        const b = buffers[arm].t;
        if (b.length) t = Math.max(t, b[b.length - 1]);
      }
      return Number.isFinite(t) ? t : 0;
    }
    function metricSeries(arm, metric, traceName, dom) {
      const b = buffers[arm];
      if (!b.t.length) return {x: [], y: []};
      dom = dom || xDomain();
      // Slice to the visible domain (+1 sample of context on each side for line
      // and finite-difference continuity at the edges).
      let start = 0;
      while (start < b.t.length - 1 && b.t[start] < dom.min) start++;
      if (start > 0) start--;
      let stop = b.t.length;
      while (stop > start + 1 && b.t[stop - 1] > dom.max) stop--;
      if (stop < b.t.length) stop++;  // keep one context sample past the right edge (mirror of start--)
      const times = b.t.slice(start, stop);
      let y;
      if (metric === "position") {
        const j = jointIndex();
        y = selectedTrace(b[TRACE[traceName].key].slice(start, stop), j);
        // Error view: subtract q_actual so tracking deviations (sub-degree) fill the
        // panel instead of being dwarfed by the absolute joint sweep. actual => 0 baseline.
        if (el.posError.checked) {
          const act = selectedTrace(b.q_actual.slice(start, stop), j);
          y = y.map((v, i) => v - act[i]);
        }
      } else if (traceName === "actual") {
        y = selectedTrace(b[metric].slice(start, stop), jointIndex());
      } else {
        let pos = selectedTrace(b[TRACE[traceName].key].slice(start, stop), jointIndex());
        let vel = finiteDiff(pos, times);
        let acc = finiteDiff(vel, times);
        y = metric === "velocity" ? vel : metric === "acceleration" ? acc : finiteDiff(acc, times);
      }
      if (el.smooth.checked) y = movingAverage(y, 5);
      return {x: times.slice(), y};
    }
    function visibleTraces() {
      const out = [];
      if (el.sent.checked) out.push("sent");
      if (el.ref.checked) out.push("ref");
      if (el.actual.checked) out.push("actual");
      return out;
    }
    function makePlot(arm, metric) {
      const panel = document.createElement("div");
      panel.className = "plot";
      const title = document.createElement("div");
      title.className = "title";
      title.textContent = `${ARM_LABEL[arm]} - ${metric.label}`;
      const canvas = document.createElement("canvas");
      panel.appendChild(title);
      panel.appendChild(canvas);
      el.plots.appendChild(panel);
      const item = {arm, metric: metric.key, unit: metric.unit, canvas, titleEl: title, baseTitle: `${ARM_LABEL[arm]} - ${metric.label}`};
      plots.push(item);
      return item;
    }
    function resizeCanvas(canvas) {
      const rect = canvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      const w = Math.max(80, Math.floor(rect.width * dpr));
      const h = Math.max(60, Math.floor(rect.height * dpr));
      if (canvas.width !== w || canvas.height !== h) {
        canvas.width = w;
        canvas.height = h;
      }
      return {w, h, dpr};
    }
    function drawPlot(item, range, dom) {
      const canvas = item.canvas;
      const {w, h, dpr} = resizeCanvas(canvas);
      const ctx = canvas.getContext("2d");
      ctx.clearRect(0, 0, w, h);
      const padL = 54 * dpr, padR = 10 * dpr, padT = 8 * dpr, padB = 24 * dpr;
      const x0 = padL, x1 = w - padR, y0 = padT, y1 = h - padB;
      ctx.fillStyle = "#0d1117";
      ctx.fillRect(0, 0, w, h);
      dom = dom || xDomain();
      const traces = visibleTraces().map(name => ({name, ...metricSeries(item.arm, item.metric, name, dom)}));
      // Shared per-metric y-range: both arms use the SAME scale (synced to the
      // larger arm) so left/right are directly comparable for tuning. Falls back
      // to this chart's own extent if no shared range was supplied.
      let ymin, ymax;
      if (range) {
        ymin = range.ymin; ymax = range.ymax;
      } else {
        ymin = Infinity; ymax = -Infinity;
        for (const tr of traces) {
          for (const v of tr.y) {
            if (Number.isFinite(v)) { ymin = Math.min(ymin, v); ymax = Math.max(ymax, v); }
          }
        }
        if (!Number.isFinite(ymin) || !Number.isFinite(ymax)) { ymin = -1; ymax = 1; }
        if (Math.abs(ymax - ymin) < 1e-9) { ymin -= 1; ymax += 1; }
        const margin = (ymax - ymin) * 0.08;
        ymin -= margin; ymax += margin;
      }
      const xmin = dom.min, xmax = dom.max;
      const xspan = (xmax - xmin) || 1;
      const sx = x => x0 + (x - xmin) / xspan * (x1 - x0);
      const sy = y => y1 - (y - ymin) / (ymax - ymin) * (y1 - y0);
      ctx.strokeStyle = "#2a3440";
      ctx.lineWidth = 1 * dpr;
      ctx.font = `${11 * dpr}px ui-sans-serif, system-ui`;
      ctx.fillStyle = "#97a3b3";
      ctx.textAlign = "right";
      ctx.textBaseline = "middle";
      for (let i = 0; i <= 4; i++) {
        const y = y0 + (y1 - y0) * i / 4;
        ctx.beginPath(); ctx.moveTo(x0, y); ctx.lineTo(x1, y); ctx.stroke();
        const value = ymax - (ymax - ymin) * i / 4;
        ctx.fillText(fmtSci(value), x0 - 6 * dpr, y);
      }
      ctx.textAlign = "center";
      ctx.textBaseline = "top";
      // Time axis relative to the right edge (0 = newest sample in view, negative
      // = older). Switch to milliseconds once zoomed below ~200 ms so individual
      // servo_j ticks read in ms; decimals adapt to the zoom (down to 1 ms).
      const useMs = xspan <= 0.2;
      const xscale = useMs ? 1000 : 1;
      const xunit = useMs ? "ms" : "s";
      const xdec = axisDecimals((xspan / 5) * xscale);
      for (let i = 0; i <= 5; i++) {
        const x = x0 + (x1 - x0) * i / 5;
        ctx.beginPath(); ctx.moveTo(x, y0); ctx.lineTo(x, y1); ctx.stroke();
        const value = ((xmin + (xmax - xmin) * i / 5) - xmax) * xscale;
        let label = value.toFixed(xdec);
        if (i === 0) label += xunit;  // unit on the leftmost (oldest) tick only
        ctx.fillText(label, x, y1 + 5 * dpr);
      }
      ctx.textAlign = "left";
      ctx.fillText(item.unit, x0, y0 + 2 * dpr);
      // Clip to the plot area so a trace that runs through the off-domain context
      // samples (one on each side, kept by metricSeries) reaches the panel edges
      // without painting over the axes/labels.
      ctx.save();
      ctx.beginPath();
      ctx.rect(x0, y0, x1 - x0, y1 - y0);
      ctx.clip();
      for (const tr of traces) {
        const color = TRACE[tr.name].color;
        ctx.strokeStyle = color;
        ctx.lineWidth = 1.6 * dpr;
        ctx.beginPath();
        let started = false;
        const pts = [];  // in-view sample dots only (context points extend the line, not the dots)
        for (let i = 0; i < tr.x.length; i++) {
          const x = tr.x[i], y = tr.y[i];
          if (!Number.isFinite(x) || !Number.isFinite(y)) { started = false; continue; }
          const px = sx(x), py = sy(y);
          // Draw the line THROUGH out-of-domain context points so segments span the
          // full width (clip trims the overshoot); previously they were dropped,
          // leaving up to one sample-gap (~2 ms) blank at each edge — very visible
          // when zoomed to a few ms span.
          if (!started) { ctx.moveTo(px, py); started = true; }
          else ctx.lineTo(px, py);
          if (x >= xmin && x <= xmax) pts.push(px, py);
        }
        ctx.stroke();
        // Once zoomed in enough that few samples are visible, mark each sample so
        // individual servo_j ticks (one dot per control-loop step) are readable.
        const n = pts.length / 2;
        if (n > 0 && n <= 80) {
          ctx.fillStyle = color;
          const r = Math.min(3.2, Math.max(1.4, (x1 - x0) / n / 3)) * dpr;
          for (let k = 0; k < pts.length; k += 2) {
            ctx.beginPath(); ctx.arc(pts[k], pts[k + 1], r, 0, 6.2832); ctx.fill();
          }
        }
      }
      ctx.restore();
      let lx = x1 - 170 * dpr;
      for (const name of visibleTraces()) {
        ctx.fillStyle = TRACE[name].color;
        ctx.fillRect(lx, y0 + 2 * dpr, 10 * dpr, 10 * dpr);
        ctx.fillStyle = "#e7edf5";
        ctx.textAlign = "left";
        ctx.textBaseline = "top";
        ctx.fillText(TRACE[name].label, lx + 14 * dpr, y0);
        lx += 56 * dpr;
      }
      // In-progress left-drag box-zoom selection, mirrored on every plot since
      // the time domain is shared (synchronized scaling across all 8 graphs).
      if (drag) {
        const a = Math.max(xmin, Math.min(drag.startT, drag.curT));
        const b = Math.min(xmax, Math.max(drag.startT, drag.curT));
        if (b > a) {
          const bx0 = sx(a), bx1 = sx(b);
          ctx.fillStyle = "rgba(120,170,255,0.16)";
          ctx.fillRect(bx0, y0, bx1 - bx0, y1 - y0);
          ctx.strokeStyle = "rgba(120,170,255,0.65)";
          ctx.lineWidth = 1 * dpr;
          ctx.strokeRect(bx0, y0, bx1 - bx0, y1 - y0);
        }
      }
    }
    function metricExtent(metricKey, dom) {
      // Union y-extent across BOTH arms for one metric, so both arms share a
      // single auto-scale (the larger arm sets the range) instead of each arm
      // scaling independently. Per-metric (position/velocity/accel/jerk keep
      // their own units), synced left<->right. Scoped to the visible time
      // domain so zooming x also rescales y to what's on screen.
      let ymin = Infinity, ymax = -Infinity;
      const names = visibleTraces();
      for (const arm of ARMS) {
        for (const name of names) {
          const {y} = metricSeries(arm, metricKey, name, dom);
          for (const v of y) {
            if (Number.isFinite(v)) { ymin = Math.min(ymin, v); ymax = Math.max(ymax, v); }
          }
        }
      }
      if (!Number.isFinite(ymin) || !Number.isFinite(ymax)) { ymin = -1; ymax = 1; }
      if (Math.abs(ymax - ymin) < 1e-9) { ymin -= 1; ymax += 1; }
      const margin = (ymax - ymin) * 0.08;
      return {ymin: ymin - margin, ymax: ymax + margin};
    }
    function renderAll() {
      renderQueued = false;
      const dom = xDomain();
      const ranges = {};
      for (const metric of METRICS) ranges[metric.key] = metricExtent(metric.key, dom);
      for (const item of plots) {
        if (item.metric === "position") {
          const t = el.posError.checked ? item.baseTitle + "  (err: q − q_actual)" : item.baseTitle;
          if (item.titleEl.textContent !== t) item.titleEl.textContent = t;
        }
        drawPlot(item, ranges[item.metric], dom);
      }
      updateStatus();
    }
    function scheduleRender() {
      if (renderQueued) return;
      renderQueued = true;
      requestAnimationFrame(renderAll);
    }
    function rateFromTimes(times) {
      if (times.length < 2) return null;
      const dt = (times[times.length - 1] - times[0]) / 1000;
      return dt > 0 ? (times.length - 1) / dt : null;
    }
    function density(arm) {
      const t = buffers[arm].t;
      if (t.length < 2) return null;
      const dt = t[t.length - 1] - t[0];
      return dt > 0 ? (t.length - 1) / dt : null;
    }
    function fmt(value, suffix) {
      return Number.isFinite(value) ? value.toFixed(1) + suffix : "n/a";
    }
    function updateStatus(prefix) {
      const sseHz = rateFromTimes(eventTimes);
      const packetHz = latestStats && Number.isFinite(latestStats.packet_rate_hz) ? latestStats.packet_rate_hz : NaN;
      const age = latestStats && Number.isFinite(latestStats.latest_receive_age_sec) ? latestStats.latest_receive_age_sec : NaN;
      const invalid = latestStats ? latestStats.invalid_packets : 0;
      let zoom = "";
      if (view) {
        const sp = view.max - view.min;
        const spTxt = sp < 0.2 ? (sp * 1000).toFixed(1) + "ms" : sp.toFixed(2) + "s";
        zoom = " | VIEW " + spTxt + " (Live/dbl-click to reset)";
      }
      el.status.textContent = (prefix ? prefix + " | " : "")
        + "SSE " + fmt(sseHz, "Hz")
        + " | UDP " + fmt(packetHz, "Hz")
        + " | density L/R " + fmt(density("left"), "Hz") + " / " + fmt(density("right"), "Hz")
        + " | age " + fmt(age, "s")
        + " | invalid " + invalid
        + zoom;
    }
    function handlePayload(payload) {
      if (!payload || payload.schema !== "robotics_lab.servo_scope_dashboard.v1") return;
      latestStats = payload.stats || latestStats;
      const arms = payload.arms || {};
      for (const arm of ARMS) appendArm(arm, arms[arm]);
      const now = performance.now();
      eventTimes.push(now);
      while (eventTimes.length > 240 || (eventTimes.length && now - eventTimes[0] > 4000)) eventTimes.shift();
      scheduleRender();
    }
    function connect() {
      const es = new EventSource("./events");
      es.onopen = () => updateStatus("connected");
      es.onmessage = event => {
        try { handlePayload(JSON.parse(event.data)); }
        catch (_err) { updateStatus("bad event"); }
      };
      es.onerror = () => updateStatus("reconnecting");
    }
    for (const metric of METRICS) for (const arm of ARMS) makePlot(arm, metric);

    // ---- Synchronized time-axis zoom (shared across all 8 plots) ----
    // Map a canvas-space clientX to absolute time using a domain. Plot paddings
    // are dpr-scaled in device space, so in CSS px they are the raw constants
    // (padL=54, padR=10). The fraction is clamped to the plot area [0,1].
    function clientXToTime(canvas, clientX, dom) {
      const rect = canvas.getBoundingClientRect();
      const left = 54, right = rect.width - 10;
      let frac = (clientX - rect.left - left) / Math.max(1, right - left);
      frac = Math.max(0, Math.min(1, frac));
      return dom.min + frac * (dom.max - dom.min);
    }
    function plotWidthPx(canvas) {
      return Math.max(1, canvas.getBoundingClientRect().width - 64);  // padL 54 + padR 10
    }
    // Deepest zoom: a 5 ms span renders as exactly 1 ms per division (5
    // divisions), so fully zoomed in the time axis reads in 1 ms units.
    const MIN_SPAN_SEC = 0.005;
    function setView(min, max) {
      if (max - min < MIN_SPAN_SEC) {
        const c = (min + max) / 2;
        min = c - MIN_SPAN_SEC / 2;
        max = c + MIN_SPAN_SEC / 2;
      }
      view = {min, max};
    }
    function setLive() { view = null; drag = null; pan = null; scheduleRender(); }
    function attachZoom(canvas) {
      // Wheel: zoom the time axis about the cursor (up = in, down = out). The
      // cursor's time stays put; both edges scale, freezing into a manual view.
      canvas.addEventListener("wheel", (ev) => {
        ev.preventDefault();
        const dom = xDomain();
        const span = dom.max - dom.min;
        if (!(span > 0)) return;
        const tc = clientXToTime(canvas, ev.clientX, dom);
        const f = ev.deltaY < 0 ? 1 / 1.2 : 1.2;
        const nmin = tc - (tc - dom.min) * f;
        const nmax = tc + (dom.max - tc) * f;
        setView(nmin, nmax);  // clamps to MIN_SPAN_SEC (1 ms-per-division floor)
        scheduleRender();
      }, {passive: false});
      // Left-drag: rubber-band box-zoom on the time axis; release zooms all
      // plots to the selected span. Domain is frozen at mousedown so the band
      // and mapping stay stable while data streams in.
      canvas.addEventListener("mousedown", (ev) => {
        if (ev.button === 0) {
          ev.preventDefault();
          const dom = xDomain();
          const t = clientXToTime(canvas, ev.clientX, dom);
          drag = {canvas, dom, startT: t, curT: t};
          scheduleRender();
        } else if (ev.button === 1) {
          // Middle button (wheel press): grab-to-pan along the time axis. Freeze
          // on the current domain so the cursor drags the same data on all plots.
          ev.preventDefault();
          const dom = xDomain();
          if (!(dom.max - dom.min > 0)) return;
          pan = {canvas, startX: ev.clientX, dom0: {min: dom.min, max: dom.max}};
          view = {min: dom.min, max: dom.max};
          document.body.style.cursor = "grabbing";
          scheduleRender();
        }
      });
      canvas.addEventListener("dblclick", (ev) => { ev.preventDefault(); setLive(); });
    }
    for (const item of plots) attachZoom(item.canvas);
    window.addEventListener("mousemove", (ev) => {
      if (pan) {
        // Grab-to-pan: shift the frozen window so the data under the cursor
        // follows it; absolute math off dom0 avoids drift accumulation.
        const dt = ((ev.clientX - pan.startX) / plotWidthPx(pan.canvas)) * (pan.dom0.max - pan.dom0.min);
        view = {min: pan.dom0.min - dt, max: pan.dom0.max - dt};
        scheduleRender();
        return;
      }
      if (!drag) return;
      drag.curT = clientXToTime(drag.canvas, ev.clientX, drag.dom);
      scheduleRender();
    });
    window.addEventListener("mouseup", (ev) => {
      if (pan) { pan = null; document.body.style.cursor = ""; scheduleRender(); return; }
      if (!drag) return;
      const a = Math.min(drag.startT, drag.curT), b = Math.max(drag.startT, drag.curT);
      const minSpan = (drag.dom.max - drag.dom.min) * 0.01;
      drag = null;
      if (b - a > Math.max(1e-3, minSpan)) setView(a, b);  // else treat as a click
      scheduleRender();
    });

    for (const control of [el.joint, el.window, el.smooth, el.posError, el.sent, el.ref, el.actual]) {
      control.addEventListener("change", () => {
        if (control === el.window) setLive();  // changing the live window exits manual zoom
        for (const arm of ARMS) trimArm(arm);
        scheduleRender();
      });
    }
    el.reset.addEventListener("click", setLive);
    window.addEventListener("resize", scheduleRender);
    setInterval(scheduleRender, 1000);
    scheduleRender();
    connect();
  })();
  </script>
</body>
</html>
"""


class _DashboardHttpServer(ThreadingHTTPServer):
    def __init__(self, *args: Any, store: DashboardStore, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.store = store
        self.stop_event = threading.Event()


class _Handler(BaseHTTPRequestHandler):
    server: _DashboardHttpServer

    def log_message(self, _format: str, *_args: Any) -> None:
        return

    def do_GET(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        if parsed.path in {"", "/"}:
            self._send_bytes(INDEX_HTML.encode("utf-8"), "text/html; charset=utf-8")
        elif parsed.path == "/events":
            self._serve_events()
        elif parsed.path == "/data":
            self._send_json(self.server.store.snapshot())
        elif parsed.path == "/healthz":
            self._send_json({"ok": True, "stats": self.server.store.snapshot()["stats"]})
        else:
            self.send_error(404)

    def _send_bytes(self, data: bytes, content_type: str) -> None:
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(data)

    def _send_json(self, payload: Mapping[str, Any]) -> None:
        data = json.dumps(payload, separators=(",", ":"), allow_nan=False).encode("utf-8")
        self._send_bytes(data, "application/json")

    def _serve_events(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()
        cursor_by_arm: dict[str, float | None] = {arm: None for arm in ARMS}
        while not self.server.stop_event.is_set():
            payload, cursor_by_arm = self.server.store.delta(cursor_by_arm)
            data = (
                "data: "
                + json.dumps(payload, separators=(",", ":"), allow_nan=False)
                + "\n\n"
            ).encode("utf-8")
            try:
                self.wfile.write(data)
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                return
            time.sleep(1.0 / 30.0)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--listen",
        default=DEFAULT_STATE_LISTEN,
        help=f"UDP state/scope endpoint to bind (default {DEFAULT_STATE_LISTEN})",
    )
    parser.add_argument("--host", default=DEFAULT_HTTP_HOST, help="HTTP bind host")
    parser.add_argument("--port", type=int, default=DEFAULT_HTTP_PORT, help="HTTP port")
    parser.add_argument("--history-sec", type=float, default=DEFAULT_HISTORY_SEC)
    parser.add_argument("--csv", default="", help="CSV log path; empty disables CSV")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    udp_host, udp_port = _parse_host_port(args.listen)
    store = DashboardStore(
        history_sec=args.history_sec,
        csv_path=args.csv or None,
    )
    receiver = StateFanoutReceiver(store, udp_host, udp_port)
    receiver.start()
    httpd = _DashboardHttpServer((args.host, int(args.port)), _Handler, store=store)
    public_url = f"http://{_public_host(args.host)}:{httpd.server_address[1]}"
    print(
        f"[scope-dashboard] listening {public_url}; UDP {udp_host}:{udp_port}; "
        f"csv={args.csv or 'disabled'}",
        flush=True,
    )
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        return 0
    finally:
        httpd.stop_event.set()
        httpd.server_close()
        receiver.stop()
        store.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
