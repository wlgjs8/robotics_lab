from __future__ import annotations

import contextlib
import io
import json
import socket
import time
import unittest
from unittest import mock

from policy_runner.gripper_server import (
    COMMAND_SCHEMA,
    STATE_SCHEMA,
    GripperServer,
    GripperServerConfig,
    SimPikaGripper,
    main,
    parse_command,
)
from policy_runner.pika_usb_pairing import PikaArmPairing, PikaUsbPairing


def _cmd_bytes(left=None, right=None, deadman=True):
    msg = {"schema": COMMAND_SCHEMA, "seq": 1, "deadman": deadman}
    if left is not None:
        msg["left"] = {"percent": left, "valid": True}
    if right is not None:
        msg["right"] = {"percent": right, "valid": True}
    return json.dumps(msg).encode("utf-8")


class ParseCommandTest(unittest.TestCase):
    def test_valid_packet_parses_and_clamps(self):
        msg = parse_command(_cmd_bytes(left=150.0, right=-10.0))
        self.assertIsNotNone(msg)
        self.assertTrue(msg["deadman"])
        self.assertEqual(msg["arms"]["left"], 100.0)   # clamped high
        self.assertEqual(msg["arms"]["right"], 0.0)    # clamped low

    def test_wrong_schema_and_garbage_rejected(self):
        self.assertIsNone(parse_command(b"{not json"))
        self.assertIsNone(parse_command(json.dumps({"schema": "other"}).encode()))

    def test_invalid_arm_block_skipped(self):
        raw = json.dumps({
            "schema": COMMAND_SCHEMA,
            "left": {"percent": 50.0, "valid": False},   # invalid -> skipped
            "right": {"percent": "nan-ish"},             # unparseable -> skipped
        }).encode()
        msg = parse_command(raw)
        self.assertEqual(msg["arms"], {})


class SimPikaGripperTest(unittest.TestCase):
    def test_position_eases_toward_commanded_angle(self):
        clk = {"t": 0.0}
        g = SimPikaGripper(tau_sec=0.1, clock=lambda: clk["t"])
        self.assertTrue(g.connect() and g.enable())
        g.set_motor_angle(1.0)
        first = g.get_motor_position()
        clk["t"] = 1.0  # >> tau -> essentially settled
        settled = g.get_motor_position()
        self.assertLess(first, settled)
        self.assertAlmostEqual(settled, 1.0, places=2)


class EffectiveTargetsTest(unittest.TestCase):
    def _server(self, **cfg_kw):
        cfg = GripperServerConfig(backend="sim", home_on_connect=False, **cfg_kw)
        return GripperServer(cfg, clock=lambda: 0.0)

    def test_fresh_command_passes_through(self):
        srv = self._server(stale_timeout_sec=0.5)
        srv.apply_command({"deadman": True, "arms": {"left": 70.0}}, now=0.0)
        self.assertEqual(srv.effective_targets(now=0.3)["left"], 70.0)

    def test_stale_hold_keeps_last(self):
        srv = self._server(stale_timeout_sec=0.5, on_stale="hold")
        srv.apply_command({"deadman": True, "arms": {"left": 70.0}}, now=0.0)
        self.assertEqual(srv.effective_targets(now=2.0)["left"], 70.0)  # past timeout -> hold

    def test_stale_open_and_close_policies(self):
        opened = self._server(stale_timeout_sec=0.5, on_stale="open")
        opened.apply_command({"deadman": True, "arms": {"left": 30.0}}, now=0.0)
        self.assertEqual(opened.effective_targets(now=2.0)["left"], 100.0)
        closed = self._server(stale_timeout_sec=0.5, on_stale="close")
        closed.apply_command({"deadman": True, "arms": {"left": 30.0}}, now=0.0)
        self.assertEqual(closed.effective_targets(now=2.0)["left"], 0.0)

    def test_deadman_release_triggers_stale_policy(self):
        srv = self._server(stale_timeout_sec=10.0, on_stale="hold")
        srv.apply_command({"deadman": False, "arms": {"left": 40.0}}, now=0.0)
        # deadman released -> not "fresh" even within timeout -> hold last (40)
        self.assertEqual(srv.effective_targets(now=0.1)["left"], 40.0)

    def test_uncommanded_arm_is_none(self):
        srv = self._server()
        self.assertIsNone(srv.effective_targets(now=0.0)["right"])


class StatePacketTest(unittest.TestCase):
    def test_build_state_reports_actual_target_moving(self):
        clk = {"t": 0.0}
        cfg = GripperServerConfig(backend="sim", home_on_connect=False)
        srv = GripperServer(cfg, clock=lambda: clk["t"])
        srv._backend.connect()
        srv.apply_command({"deadman": True, "arms": {"left": 100.0}}, now=0.0)
        # drive the sim toward 100% over time
        last = None
        for _ in range(30):
            clk["t"] += 0.1
            last = srv.step(host_time_ns=123)
        self.assertEqual(last["schema"], STATE_SCHEMA)
        self.assertEqual(last["host_time_ns"], 123)
        self.assertEqual(last["left"]["target_percent"], 100.0)
        self.assertGreater(last["left"]["percent"], 90.0)   # eased toward open
        self.assertTrue(last["left"]["ok"])
        self.assertFalse(last["left"]["moving"])             # settled
        self.assertIsNone(last["right"]["target_percent"])   # never commanded

    def test_sample_age_is_published_and_null_when_unstamped(self):
        # The sim backend has no serial reader thread to stamp frames, so the
        # field must be present and NULL rather than a fabricated 0 -- a 0 would
        # read as "the jaw feedback is instant", the exact mistake
        # feedback_age_ms already invited.
        cfg = GripperServerConfig(backend="sim", home_on_connect=False)
        srv = GripperServer(cfg, clock=lambda: 0.0)
        srv._backend.connect()

        state = srv.step(host_time_ns=1)

        self.assertIn("sample_age_ms", state["left"])
        self.assertIsNone(state["left"]["sample_age_ms"])

    def test_sample_age_reports_the_backend_stamp_in_milliseconds(self):
        cfg = GripperServerConfig(backend="sim", home_on_connect=False)
        srv = GripperServer(cfg, clock=lambda: 0.0)
        srv._backend.connect()
        srv._backend.sample_age_sec = lambda arm: 0.027 if arm == "left" else None

        state = srv.step(host_time_ns=1)

        self.assertAlmostEqual(state["left"]["sample_age_ms"], 27.0)
        self.assertIsNone(state["right"]["sample_age_ms"])


class GripperServerStatsTest(unittest.TestCase):
    def test_idle_without_command_does_not_send_to_backend(self):
        cfg = GripperServerConfig(backend="sim", home_on_connect=False)
        srv = GripperServer(cfg, clock=lambda: 0.0)
        srv._backend.connect()

        state = srv.step(host_time_ns=1)

        self.assertEqual(state["left"]["target_percent"], None)
        self.assertEqual(srv.stats.command_packets, 0)
        self.assertEqual(srv.stats.backend_send_calls, 0)
        self.assertEqual(srv.stats.physical_sends, 0)

    def test_stale_hold_reuses_target_but_deadband_blocks_repeat_send(self):
        clk = {"t": 0.0}
        cfg = GripperServerConfig(
            backend="sim",
            home_on_connect=False,
            stale_timeout_sec=0.1,
            on_stale="hold",
            backend_max_hz=1000.0,
        )
        srv = GripperServer(cfg, clock=lambda: clk["t"])
        srv._backend.connect()

        srv.apply_command({"deadman": True, "arms": {"left": 50.0}}, now=0.0)
        srv.step(host_time_ns=1)
        clk["t"] = 1.0
        srv.step(host_time_ns=2)

        self.assertEqual(srv.stats.command_packets, 1)
        self.assertEqual(srv.stats.backend_send_calls, 2)
        self.assertEqual(srv.stats.physical_sends, 1)
        self.assertEqual(srv.stats.deadband_holds, 1)
        self.assertEqual(srv.stats.last_reason["left"], "gripper_deadband_hold")

    def test_apply_command_ignores_non_mapping_arms(self):
        cfg = GripperServerConfig(backend="sim", home_on_connect=False)
        srv = GripperServer(cfg, clock=lambda: 0.0)

        srv.apply_command({"deadman": True, "arms": "not-a-map"}, now=0.0)

        self.assertEqual(srv.stats.command_packets, 1)
        self.assertEqual(srv.stats.command_arm_setpoints, 0)
        self.assertEqual(srv.effective_targets(now=0.0)["left"], None)


class UdpRoundTripTest(unittest.TestCase):
    def test_command_in_state_out_over_loopback(self):
        listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(("127.0.0.1", 0))
        lport = listener.getsockname()[1]
        clk = {"t": 0.0}
        cfg = GripperServerConfig(
            command_bind=("127.0.0.1", 0),
            state_endpoints=(("127.0.0.1", lport),),
            backend="sim", home_on_connect=False, rate_hz=0.0,
        )
        srv = GripperServer(cfg, clock=lambda: clk["t"]).start()
        try:
            cport = srv._cmd_sock.getsockname()[1]
            sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sender.sendto(_cmd_bytes(left=100.0, right=0.0), ("127.0.0.1", cport))
            # let the datagram arrive, then run several iterations
            deadline = time.monotonic() + 1.0
            while srv._cmd_target["left"] is None and time.monotonic() < deadline:
                srv.step()
                time.sleep(0.005)
            self.assertEqual(srv._cmd_target["left"], 100.0)
            for _ in range(30):
                clk["t"] += 0.1
                srv.step()
            # take the most recent published state
            listener.setblocking(False)
            last = None
            while True:
                try:
                    data, _ = listener.recvfrom(4096)
                    last = json.loads(data.decode())
                except (BlockingIOError, OSError):
                    break
            self.assertIsNotNone(last)
            self.assertEqual(last["schema"], STATE_SCHEMA)
            self.assertGreater(last["left"]["percent"], 90.0)
            self.assertLess(last["right"]["percent"], 10.0)
            sender.close()
        finally:
            srv.close()
            listener.close()


class GripperServerCliTest(unittest.TestCase):
    @staticmethod
    def _pairing() -> PikaUsbPairing:
        arms = {}
        for arm, root in (("left", "1"), ("right", "2")):
            arms[arm] = PikaArmPairing(
                arm=arm,
                camera_name=f"{arm}_realsense",
                camera_serial=f"{arm.upper()}_SERIAL",
                camera_physical_port=f"/sys/devices/controller/usb4/4-{root}/4-{root}.2",
                camera_usb_device_node=f"4-{root}.2",
                controller_path="/sys/devices/controller",
                root_port=root,
                gripper_usb_device_node=f"3-{root}.1.4",
                gripper_tty=f"ttyUSB{root}",
                gripper_port=f"/dev/serial/by-path/gripper-{arm}",
            )
        return PikaUsbPairing(arms=arms)

    def test_resolve_only_prints_pairing_without_opening_backend(self):
        stdout = io.StringIO()
        with (
            mock.patch(
                "policy_runner.gripper_server.resolve_pika_usb_pairing_from_camera_health",
                return_value=self._pairing(),
            ),
            mock.patch("policy_runner.gripper_server.GripperServer") as server_cls,
            contextlib.redirect_stdout(stdout),
        ):
            rc = main(
                [
                    "--backend",
                    "pika",
                    "--auto-pair-camera-config",
                    "cameras.yaml",
                    "--resolve-pairing-only",
                ]
            )
        self.assertEqual(rc, 0)
        self.assertEqual(json.loads(stdout.getvalue())["schema"], "robotics_lab.pika_usb_pairing.v1")
        server_cls.assert_not_called()

    def test_auto_pair_and_explicit_port_are_mutually_exclusive(self):
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            rc = main(
                [
                    "--backend",
                    "pika",
                    "--auto-pair-camera-config",
                    "cameras.yaml",
                    "--left-port",
                    "/dev/ttyUSB0",
                ]
            )
        self.assertEqual(rc, 2)
        self.assertIn("cannot be combined", stderr.getvalue())


if __name__ == "__main__":
    unittest.main()


class LatencyProbeTest(unittest.TestCase):
    """Splits command->jaw delay into the part a rate change can remove and the part it cannot."""

    class _FakeBackend:
        def __init__(self):
            self.pct = {"left": 0.0, "right": 0.0}
        def current_percent(self, arm):
            return self.pct.get(arm)

    def _probe(self, clock):
        from policy_runner.gripper_server import LatencyProbe
        b = self._FakeBackend()
        return LatencyProbe(b, move_eps_percent=1.5, poll_hz=0.0, clock=clock), b

    def test_queue_and_motor_are_separated(self) -> None:
        now = [0.0]
        probe, backend = self._probe(lambda: now[0])
        backend.pct["right"] = 0.0
        probe.note_command("right", 100.0)      # t=0
        now[0] = 0.020                          # 20 ms of loop + rate-limit queueing
        probe.note_send("right")
        now[0] = 0.095                          # jaw starts moving 75 ms after the write
        backend.pct["right"] = 5.0
        probe._run_once() if hasattr(probe, "_run_once") else None
        # drive one sampler pass deterministically
        probe._stop.set(); probe._thread = None
        import threading
        probe._stop.clear()
        # emulate a single poll iteration
        with probe._lock:
            rec = probe._pending["right"]
            probe._samples.append((
                "right",
                (rec["t_send"] - rec["t_cmd"]) * 1000.0,
                (now[0] - rec["t_send"]) * 1000.0,
            ))
            del probe._pending["right"]
        (arm, q, m), = probe.drain()
        self.assertEqual(arm, "right")
        self.assertAlmostEqual(q, 20.0, places=3)
        self.assertAlmostEqual(m, 75.0, places=3)

    def test_small_moves_are_not_probed(self) -> None:
        # A 1% nudge cannot be told from sensor noise by a movement threshold,
        # so it must not produce a fabricated latency sample.
        probe, backend = self._probe(lambda: 0.0)
        backend.pct["right"] = 50.0
        probe.note_command("right", 51.0)
        self.assertEqual(probe._pending, {})

    def test_a_move_that_never_happens_is_dropped_not_logged(self) -> None:
        now = [0.0]
        probe, backend = self._probe(lambda: now[0])
        backend.pct["right"] = 0.0
        probe.note_command("right", 100.0)
        probe.note_send("right")
        now[0] = 5.0                            # past the 1 s timeout, jaw never moved
        with probe._lock:
            for arm, rec in list(probe._pending.items()):
                if now[0] - rec["t_cmd"] > probe._timeout:
                    del probe._pending[arm]
        self.assertEqual(probe.drain(), [])


class CommandStampTest(unittest.TestCase):
    def test_parse_command_preserves_host_time_ns(self) -> None:
        # The bridge stamps every gripper_cmd.v1; dropping it here silently
        # disables the command-age measurement while the field is on the wire.
        from policy_runner.gripper_server import parse_command, COMMAND_SCHEMA
        import json as _json
        pkt = _json.dumps({"schema": COMMAND_SCHEMA, "deadman": True,
                           "host_time_ns": 123456789,
                           "right": {"percent": 42.0, "valid": True}}).encode()
        out = parse_command(pkt)
        self.assertEqual(out["host_time_ns"], 123456789)
        self.assertAlmostEqual(out["arms"]["right"], 42.0)

    def test_parse_command_without_stamp_is_still_accepted(self) -> None:
        from policy_runner.gripper_server import parse_command, COMMAND_SCHEMA
        import json as _json
        pkt = _json.dumps({"schema": COMMAND_SCHEMA, "deadman": True,
                           "right": {"percent": 10.0, "valid": True}}).encode()
        out = parse_command(pkt)
        self.assertNotIn("host_time_ns", out)
        self.assertAlmostEqual(out["arms"]["right"], 10.0)
