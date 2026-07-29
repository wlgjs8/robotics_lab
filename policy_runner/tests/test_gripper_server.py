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
