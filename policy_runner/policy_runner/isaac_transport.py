"""Pipe transport and clock for the hardware-free, externally stepped C++ loop.

No Isaac imports: the plant and camera implementation belong to simulation.
There are no robot/control UDP sockets in this adapter.
"""
from __future__ import annotations

import json
import math
import selectors
import subprocess
import threading
import time
from pathlib import Path

from .robot_state_client import StateSnapshot


class SimulationClock:
    dt_ns = 2_000_000

    def __init__(self):
        self._ns = 1_000_000_000
        self._cv = threading.Condition()
        self.cancelled = False

    def now_ns(self):
        with self._cv:
            return self._ns

    def monotonic(self):
        return self.now_ns() * 1e-9

    def advance(self):
        with self._cv:
            self._ns += self.dt_ns
            self._cv.notify_all()

    def wait_until(self, deadline_ns):
        with self._cv:
            self._cv.wait_for(lambda: self._ns >= deadline_ns or self.cancelled)

    def cancel(self):
        with self._cv:
            self.cancelled = True
            self._cv.notify_all()


class TimedInferenceClient:
    """Release responses after measured service latency on the simulated clock.

    Physics continues while the policy's existing worker waits. If inference
    itself takes longer than simulated time, the response arrives late, as on
    hardware. Rendering slower than real time cannot erase model latency.
    """
    def __init__(self, client, clock):
        self.client, self.clock = client, clock
        self.calls = []

    def infer(self, observation):
        sim_start = self.clock.now_ns()
        started = time.perf_counter_ns()
        result = self.client.infer(observation)
        latency = time.perf_counter_ns() - started
        self.clock.wait_until(sim_start + latency)
        self.calls.append({"request_ns": sim_start, "service_ns": latency,
                           "delivered_ns": self.clock.now_ns()})
        return result

    def close(self):
        self.clock.cancel()
        self.client.close()

    def __getattr__(self, name):
        return getattr(self.client, name)


class ServoBridge:
    def __init__(self, executable, config, initial_measurement, *, cwd, log_path):
        self.log = open(log_path, "w")
        self._lock = threading.Lock()
        self.process = subprocess.Popen(
            [str(Path(executable).resolve()), str(Path(config).resolve())],
            cwd=cwd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=self.log, text=True, bufsize=1,
        )
        self.seq = 1
        self.chunk_count = self.command_count = 0
        try:
            self.reply = self.rpc({"op": "init", **initial_measurement}, timeout=60)
        except BaseException:
            self.close()
            raise

    def rpc(self, packet, timeout=15):
        with self._lock:
            data = json.dumps(packet, allow_nan=False, separators=(",", ":"))
            if len(data.encode()) >= 1024 * 1024:
                raise ValueError("plant RPC exceeds 1 MiB")
            self.process.stdin.write(data + "\n")
            self.process.stdin.flush()
            with selectors.DefaultSelector() as selector:
                selector.register(self.process.stdout, selectors.EVENT_READ)
                if not selector.select(timeout):
                    raise TimeoutError("C++ plant bridge did not respond")
            line = self.process.stdout.readline()
            if not line:
                raise RuntimeError(f"C++ plant bridge exited ({self.process.poll()}); see {self.log.name}")
            reply = json.loads(line)
            if not reply.get("ok"):
                raise RuntimeError(f"C++ rejected {packet['op']}: {reply}")
            return reply

    def step(self, clock, measurement):
        self.seq += 1
        self.reply = self.rpc({"op": "step", "seq": self.seq,
                               "time_ns": clock.now_ns(), **measurement})
        if self.reply["seq"] != self.seq or self.reply["time_ns"] != clock.now_ns():
            raise RuntimeError("C++ plant clock mismatch")
        return self.reply

    def close(self):
        p = self.process
        if p.poll() is None:
            try:
                p.stdin.close()  # EOF -> loop.stop(); no staged command escapes.
                p.wait(timeout=10)
            except (OSError, subprocess.TimeoutExpired):
                p.terminate()
                try:
                    p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    p.kill()
                    p.wait()
        for pipe in (p.stdin, p.stdout):
            if pipe is not None:
                pipe.close()
        self.log.close()


class PipeDatagram:
    """Explicit socket-factory adapter for existing packet encoders.

    sendto's address is only a routing label; no network socket is opened.
    """
    def __init__(self, bridge, operation):
        self.bridge, self.operation = bridge, operation

    def sendto(self, data, address):
        self.bridge.rpc({"op": self.operation, "packet": json.loads(data)})
        if self.operation == "chunk":
            self.bridge.chunk_count += 1
        else:
            self.bridge.command_count += 1
        return len(data)

    def setblocking(self, value):
        pass

    def close(self):
        pass


class BridgeStateClient:
    def __init__(self, bridge, clock):
        self.bridge, self.clock = bridge, clock

    @property
    def latest(self):
        return StateSnapshot(self.bridge.reply["state"],
                             self.bridge.reply["time_ns"] * 1e-9)

    def start(self):
        pass

    def stop(self):
        pass

    def close(self):
        pass


class BoundedSource:
    """End a rollout through the runner's normal Hold/completion path."""
    def __init__(self, source, clock, duration):
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError("duration must be positive")
        self.source, self.clock = source, clock
        self.end_ns = clock.now_ns() + round(duration * 1e9)
        self.held = False

    def next_intent(self, snapshot, now_monotonic):
        if self.clock.now_ns() >= self.end_ns:
            from .servo_command_client import CommandIntent
            self.held = True
            return CommandIntent.hold()
        return self.source.next_intent(snapshot, now_monotonic)

    def completion_reason(self):
        if self.held:
            self.clock.cancel()
            return "Isaac episode duration reached"
        fn = getattr(self.source, "completion_reason", None)
        return fn() if fn else None

    def close(self):
        self.clock.cancel()
        fn = getattr(self.source, "close", None)
        if fn:
            fn()

    def __getattr__(self, name):
        return getattr(self.source, name)
