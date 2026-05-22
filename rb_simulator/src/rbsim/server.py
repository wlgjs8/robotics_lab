"""Loopback JSON Lines TCP server for the hardware-free simulator."""

from __future__ import annotations

import argparse
import socket
import socketserver
import threading
import time
from dataclasses import replace
from pathlib import Path
from urllib.parse import urlparse

from .config import SimulatorConfig, load_simulator_config
from .protocol import SimulatorProtocol
from .state_machine import DualArmSimulator


class RbsimService:
    """Owns one simulator plus control and admin JSONL endpoints."""

    def __init__(self, config: SimulatorConfig):
        self.config = config
        self.simulator = DualArmSimulator(config)
        self.protocol = SimulatorProtocol(self.simulator)
        self._servers: list[_RbsimTCPServer] = []
        self._threads: list[threading.Thread] = []
        self._tick_stop = threading.Event()
        self._tick_thread: threading.Thread | None = None

    @property
    def control_address(self) -> tuple[str, int]:
        return _server_address(self._servers, "control")

    @property
    def admin_address(self) -> tuple[str, int]:
        return _server_address(self._servers, "admin")

    @classmethod
    def with_binds(cls, config: SimulatorConfig, control_bind: str, admin_bind: str) -> "RbsimService":
        return cls(replace(config, control_bind=control_bind, admin_bind=admin_bind))

    def start(self, *, start_tick_loop: bool = True) -> None:
        if self._servers:
            return

        self._servers = [
            _make_server(self.config.control_bind, "control", self.protocol),
            _make_server(self.config.admin_bind, "admin", self.protocol),
        ]
        for server in self._servers:
            thread = threading.Thread(
                target=server.serve_forever,
                name=f"rbsim-{server.endpoint_role}",
                daemon=True,
            )
            thread.start()
            self._threads.append(thread)

        if start_tick_loop:
            self._tick_thread = threading.Thread(target=self._run_tick_loop, name="rbsim-tick", daemon=True)
            self._tick_thread.start()

    def stop(self) -> None:
        self._tick_stop.set()
        if self._tick_thread is not None:
            self._tick_thread.join(timeout=2.0)
        for server in self._servers:
            server.shutdown()
            server.server_close()
        for thread in self._threads:
            thread.join(timeout=2.0)
        self._servers = []
        self._threads = []

    def serve_forever(self) -> None:
        self.start(start_tick_loop=True)
        try:
            while True:
                time.sleep(3600)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def _run_tick_loop(self) -> None:
        period = 1.0 / self.config.update_rate_hz
        while not self._tick_stop.wait(period):
            self.simulator.tick(1)


class _RbsimTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(
        self,
        server_address: tuple[str, int],
        endpoint_role: str,
        protocol: SimulatorProtocol,
    ):
        self.endpoint_role = endpoint_role
        self.protocol = protocol
        super().__init__(server_address, _JsonLineHandler)


class _JsonLineHandler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        server = self.server
        assert isinstance(server, _RbsimTCPServer)
        for line in self.rfile:
            response = server.protocol.handle_json_line(line, server.endpoint_role)
            self.wfile.write(response.encode("utf-8") + b"\n")


def parse_tcp_bind(bind: str) -> tuple[str, int]:
    parsed = urlparse(bind)
    if parsed.scheme != "tcp" or not parsed.hostname or parsed.port is None:
        raise ValueError(f"bind must be tcp://host:port, got {bind!r}")
    if not _is_loopback(parsed.hostname):
        raise ValueError(f"rb_simulator only binds loopback addresses by default, got {parsed.hostname!r}")
    return (parsed.hostname, parsed.port)


def _is_loopback(host: str) -> bool:
    if host == "localhost":
        return True
    try:
        return socket.gethostbyname(host).startswith("127.")
    except OSError:
        return False


def _make_server(bind: str, endpoint_role: str, protocol: SimulatorProtocol) -> _RbsimTCPServer:
    return _RbsimTCPServer(parse_tcp_bind(bind), endpoint_role, protocol)


def _server_address(servers: list[_RbsimTCPServer], endpoint_role: str) -> tuple[str, int]:
    for server in servers:
        if server.endpoint_role == endpoint_role:
            host, port = server.server_address
            return (str(host), int(port))
    raise RuntimeError(f"{endpoint_role} server is not started")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the hardware-free RB simulator JSONL service")
    parser.add_argument("--config", type=Path, required=True, help="Path to rb_simulator YAML config")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    service = RbsimService(load_simulator_config(args.config))
    service.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
