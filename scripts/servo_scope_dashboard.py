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
DEFAULT_HISTORY_SEC = 10.0


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
        max_samples_per_arm: int = 20000,
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
        <option value="2">2 s</option><option value="5" selected>5 s</option><option value="10">10 s</option>
      </select>
    </label>
    <label class="sent"><input id="trace-sent" type="checkbox" checked> q_sent</label>
    <label class="ref"><input id="trace-ref" type="checkbox" checked> q_ref</label>
    <label class="actual"><input id="trace-actual" type="checkbox" checked> q_actual</label>
    <label><input id="smooth" type="checkbox" checked> Smooth</label>
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
    const el = {
      plots: document.getElementById("plots"),
      joint: document.getElementById("joint"),
      window: document.getElementById("window"),
      smooth: document.getElementById("smooth"),
      sent: document.getElementById("trace-sent"),
      ref: document.getElementById("trace-ref"),
      actual: document.getElementById("trace-actual"),
      reset: document.getElementById("reset"),
      status: document.getElementById("status"),
    };

    function windowSec() { return Math.max(0.5, Number(el.window.value) || 5); }
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
      const cutoff = b.t[b.t.length - 1] - windowSec();
      let drop = 0;
      while (drop < b.t.length - 1 && b.t[drop] < cutoff) drop++;
      if (drop > 0) {
        for (const key of Object.keys(b)) b[key].splice(0, drop);
      }
      const extra = Math.max(0, b.t.length - 20000);
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
    function globalEndTime() {
      let t = -Infinity;
      for (const arm of ARMS) {
        const b = buffers[arm].t;
        if (b.length) t = Math.max(t, b[b.length - 1]);
      }
      return Number.isFinite(t) ? t : 0;
    }
    function metricSeries(arm, metric, traceName) {
      const b = buffers[arm];
      if (!b.t.length) return {x: [], y: []};
      const end = globalEndTime();
      const cutoff = end - windowSec();
      let start = 0;
      while (start < b.t.length - 1 && b.t[start] < cutoff) start++;
      const times = b.t.slice(start);
      let y;
      if (metric === "position") {
        y = selectedTrace(b[TRACE[traceName].key].slice(start), jointIndex());
      } else if (traceName === "actual") {
        y = selectedTrace(b[metric].slice(start), jointIndex());
      } else {
        let pos = selectedTrace(b[TRACE[traceName].key].slice(start), jointIndex());
        let vel = finiteDiff(pos, times);
        let acc = finiteDiff(vel, times);
        y = metric === "velocity" ? vel : metric === "acceleration" ? acc : finiteDiff(acc, times);
      }
      if (el.smooth.checked) y = movingAverage(y, 5);
      return {x: times.map(t => t - end), y};
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
      const item = {arm, metric: metric.key, unit: metric.unit, canvas};
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
    function drawPlot(item) {
      const canvas = item.canvas;
      const {w, h, dpr} = resizeCanvas(canvas);
      const ctx = canvas.getContext("2d");
      ctx.clearRect(0, 0, w, h);
      const padL = 54 * dpr, padR = 10 * dpr, padT = 8 * dpr, padB = 24 * dpr;
      const x0 = padL, x1 = w - padR, y0 = padT, y1 = h - padB;
      ctx.fillStyle = "#0d1117";
      ctx.fillRect(0, 0, w, h);
      const traces = visibleTraces().map(name => ({name, ...metricSeries(item.arm, item.metric, name)}));
      let ymin = Infinity, ymax = -Infinity;
      for (const tr of traces) {
        for (const v of tr.y) {
          if (Number.isFinite(v)) { ymin = Math.min(ymin, v); ymax = Math.max(ymax, v); }
        }
      }
      if (!Number.isFinite(ymin) || !Number.isFinite(ymax)) { ymin = -1; ymax = 1; }
      if (Math.abs(ymax - ymin) < 1e-9) { ymin -= 1; ymax += 1; }
      const margin = (ymax - ymin) * 0.08;
      ymin -= margin; ymax += margin;
      const xmin = -windowSec(), xmax = 0;
      const sx = x => x0 + (x - xmin) / (xmax - xmin) * (x1 - x0);
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
        ctx.fillText(value.toPrecision(3), x0 - 6 * dpr, y);
      }
      ctx.textAlign = "center";
      ctx.textBaseline = "top";
      for (let i = 0; i <= 5; i++) {
        const x = x0 + (x1 - x0) * i / 5;
        ctx.beginPath(); ctx.moveTo(x, y0); ctx.lineTo(x, y1); ctx.stroke();
        const value = xmin + (xmax - xmin) * i / 5;
        ctx.fillText(value.toFixed(1), x, y1 + 5 * dpr);
      }
      ctx.textAlign = "left";
      ctx.fillText(item.unit, x0, y0 + 2 * dpr);
      for (const tr of traces) {
        const color = TRACE[tr.name].color;
        ctx.strokeStyle = color;
        ctx.lineWidth = 1.6 * dpr;
        ctx.beginPath();
        let started = false;
        for (let i = 0; i < tr.x.length; i++) {
          const x = tr.x[i], y = tr.y[i];
          if (!Number.isFinite(x) || !Number.isFinite(y)) { started = false; continue; }
          if (x < xmin || x > xmax) continue;
          const px = sx(x), py = sy(y);
          if (!started) { ctx.moveTo(px, py); started = true; }
          else ctx.lineTo(px, py);
        }
        ctx.stroke();
      }
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
    }
    function renderAll() {
      renderQueued = false;
      for (const item of plots) drawPlot(item);
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
      el.status.textContent = (prefix ? prefix + " | " : "")
        + "SSE " + fmt(sseHz, "Hz")
        + " | UDP " + fmt(packetHz, "Hz")
        + " | density L/R " + fmt(density("left"), "Hz") + " / " + fmt(density("right"), "Hz")
        + " | age " + fmt(age, "s")
        + " | invalid " + invalid;
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
    for (const control of [el.joint, el.window, el.smooth, el.sent, el.ref, el.actual]) {
      control.addEventListener("change", () => {
        for (const arm of ARMS) trimArm(arm);
        scheduleRender();
      });
    }
    el.reset.addEventListener("click", scheduleRender);
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
