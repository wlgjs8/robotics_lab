"""Unit tests for the RTC client wiring in OpenpiRemoteActionSource (M3).

Drives _sample_chunk with a fake websocket client (no server, no model) to lock:
- RTC OFF: no prev fields in the obs, nothing cached.
- RTC ON cold start: no prev sent, but the server's MODEL-SPACE rtc_raw_actions is
  cached (NOT the gripper-rescaled actions).
- RTC ON warm: prev_action_chunk + d/execute_horizon/schedule/max_guidance_weight
  ride in the obs, and prev is advanced by the executed window.
- reset_rtc cold-starts again.

Built via __new__ to skip the connecting __init__. Guarded by torch availability
(OpenpiRemoteActionSource imports flow_inference, which imports torch).
"""

from __future__ import annotations

import unittest

try:
    import numpy as np

    from policy_runner.openpi_remote import (
        OpenpiRemoteActionSource,
        _resolve_openpi_action_horizon,
        _resolve_openpi_chunk_execute_steps,
        rtc_shift_prev_chunk,
    )
except Exception:  # torch (a transitive import) may be absent
    np = None
    OpenpiRemoteActionSource = None  # type: ignore[assignment]
    _resolve_openpi_action_horizon = None  # type: ignore[assignment]
    _resolve_openpi_chunk_execute_steps = None  # type: ignore[assignment]
    rtc_shift_prev_chunk = None  # type: ignore[assignment]

_HORIZON = 4
_GRIP_LEFT, _GRIP_RIGHT = 6, 13


class _FakeClient:
    """Records every obs sent and returns a fixed chunk + a DISTINCT raw chunk."""

    def __init__(self, actions, raw):
        self._actions = actions
        self._raw = raw
        self.sent_obs: list[dict] = []

    def infer(self, obs):
        self.sent_obs.append(obs)
        out = {"actions": self._actions}
        if self._raw is not None:
            out["rtc_raw_actions"] = self._raw
        return out


def _make_source(
    *,
    rtc_enabled: bool,
    raw=None,
    horizon: int = _HORIZON,
    chunk_execute_steps: int = 3,
) -> "OpenpiRemoteActionSource":
    assert OpenpiRemoteActionSource is not None
    src = OpenpiRemoteActionSource.__new__(OpenpiRemoteActionSource)
    # actions: gripper dims at 0.5 (-> *100 = 50 in the returned chunk).
    actions = np.zeros((horizon, 14), dtype=np.float32)
    actions[:, _GRIP_LEFT] = 0.5
    actions[:, _GRIP_RIGHT] = 0.5
    # raw (model space) is a DISTINCT constant so we can tell it apart from actions.
    raw_chunk = np.full((horizon, 14), 7.0, dtype=np.float32) if raw is None else raw
    src._client = _FakeClient(actions, raw_chunk if rtc_enabled else None)
    # Stub the obs builders so _sample_chunk runs without cameras / a model.
    src._raw_camera_images = lambda: ({"left": object(), "right": object()}, 2, 0)
    src._proprio_state = lambda payload: np.zeros(14, dtype=np.float32)
    src.prompt = "task"
    src.action_horizon = horizon
    src.stderr = __import__("sys").stderr
    src.last_image_decode_count = 0
    src.last_missing_camera_count = 0
    src.image_decode_count = 0
    src.missing_camera_count = 0
    # RTC state.
    src.rtc_enabled = rtc_enabled
    src.rtc_inference_delay = 2
    src.chunk_execute_steps = chunk_execute_steps
    src.rtc_prefix_attention_schedule = "exp"
    src.rtc_max_guidance_weight = 5.0
    src._rtc_prev_raw_chunk = None
    src._rtc_warned_no_raw = False
    return src, raw_chunk


@unittest.skipIf(OpenpiRemoteActionSource is None, "torch is not installed")
class OpenpiRemoteHorizonTest(unittest.TestCase):
    def test_metadata_horizon_supports_h8_h24_h50(self) -> None:
        assert _resolve_openpi_action_horizon is not None
        for horizon in (8, 24, 50):
            self.assertEqual(
                _resolve_openpi_action_horizon(None, {"action_horizon": horizon}),
                horizon,
            )

    def test_cli_horizon_validates_metadata(self) -> None:
        assert _resolve_openpi_action_horizon is not None
        self.assertEqual(_resolve_openpi_action_horizon(24, {"action_horizon": 24}), 24)
        with self.assertRaisesRegex(ValueError, "action_horizon mismatch"):
            _resolve_openpi_action_horizon(24, {"action_horizon": 8})

    def test_legacy_default_requires_execute_steps_within_horizon(self) -> None:
        assert _resolve_openpi_action_horizon is not None
        assert _resolve_openpi_chunk_execute_steps is not None
        self.assertEqual(_resolve_openpi_action_horizon(None, {}), 16)
        self.assertEqual(_resolve_openpi_chunk_execute_steps(8, 8), 8)
        self.assertEqual(_resolve_openpi_chunk_execute_steps(24, 24), 24)
        self.assertEqual(_resolve_openpi_chunk_execute_steps(24, 50), 24)
        with self.assertRaisesRegex(ValueError, "must not exceed"):
            _resolve_openpi_chunk_execute_steps(24, 8)


@unittest.skipIf(OpenpiRemoteActionSource is None, "torch is not installed")
class OpenpiRemoteRtcTest(unittest.TestCase):
    def test_rtc_off_sends_no_prev_and_caches_nothing(self) -> None:
        src, _ = _make_source(rtc_enabled=False)
        out = src._sample_chunk({})
        self.assertIsNotNone(out)
        obs = src._client.sent_obs[-1]
        self.assertNotIn("prev_action_chunk", obs)
        self.assertIsNone(src._rtc_prev_raw_chunk)

    def test_rtc_on_cold_start_caches_raw_not_scaled_actions(self) -> None:
        src, raw = _make_source(rtc_enabled=True)
        out = src._sample_chunk({})
        # First call: no prev yet.
        self.assertNotIn("prev_action_chunk", src._client.sent_obs[-1])
        # Returned chunk has the gripper rescaled to percent (0.5 -> 50).
        self.assertAlmostEqual(float(out[0, _GRIP_LEFT]), 50.0)
        # Cached prev is the MODEL-SPACE raw (7.0), NOT the *100 actions.
        self.assertIsNotNone(src._rtc_prev_raw_chunk)
        self.assertTrue(np.allclose(src._rtc_prev_raw_chunk, raw))

    def test_rtc_on_warm_sends_prev_and_knobs(self) -> None:
        src, raw = _make_source(rtc_enabled=True)
        src._sample_chunk({})  # cold start seeds prev
        src._sample_chunk({})  # warm call sends prev
        obs = src._client.sent_obs[-1]
        self.assertIn("prev_action_chunk", obs)
        self.assertTrue(
            np.allclose(
                obs["prev_action_chunk"],
                rtc_shift_prev_chunk(raw, src.chunk_execute_steps),
            )
        )
        self.assertEqual(obs["inference_delay"], 2)  # min(2, chunk_execute_steps=3)
        self.assertEqual(obs["execute_horizon"], 3)
        self.assertEqual(obs["prefix_attention_schedule"], "exp")
        self.assertEqual(obs["max_guidance_weight"], 5.0)

    def test_rtc_h24_sends_full_execute_horizon_and_keeps_full_raw_chunk(self) -> None:
        src, raw = _make_source(rtc_enabled=True, horizon=24, chunk_execute_steps=24)
        out = src._sample_chunk({})  # cold start seeds prev
        self.assertEqual(out.shape[0], 24)
        src._sample_chunk({})  # warm call sends prev
        obs = src._client.sent_obs[-1]
        self.assertEqual(obs["execute_horizon"], 24)
        self.assertEqual(obs["inference_delay"], 2)
        self.assertEqual(obs["prev_action_chunk"].shape[0], 24)
        self.assertTrue(
            np.allclose(
                obs["prev_action_chunk"],
                rtc_shift_prev_chunk(raw, src.chunk_execute_steps),
            )
        )

    def test_inference_delay_clamped_to_execute_horizon(self) -> None:
        src, _ = _make_source(rtc_enabled=True)
        src.rtc_inference_delay = 10
        src.chunk_execute_steps = 3
        src._sample_chunk({})
        src._sample_chunk({})
        self.assertEqual(src._client.sent_obs[-1]["inference_delay"], 3)

    def test_reset_rtc_cold_starts_again(self) -> None:
        src, _ = _make_source(rtc_enabled=True)
        src._sample_chunk({})
        self.assertIsNotNone(src._rtc_prev_raw_chunk)
        src.reset_rtc()
        self.assertIsNone(src._rtc_prev_raw_chunk)
        src._sample_chunk({})
        self.assertNotIn("prev_action_chunk", src._client.sent_obs[-1])

    def test_missing_raw_warns_once_and_stays_vanilla(self) -> None:
        # Server returns no rtc_raw_actions -> stays vanilla, warns once.
        src, _ = _make_source(rtc_enabled=True)
        src._client._raw = None  # simulate an old server
        src._sample_chunk({})
        self.assertIsNone(src._rtc_prev_raw_chunk)
        self.assertTrue(src._rtc_warned_no_raw)

    def test_action_shape_mismatch_fails_closed_instead_of_truncating(self) -> None:
        src, _ = _make_source(rtc_enabled=False, horizon=16)
        src.action_horizon = 24
        self.assertIsNone(src._sample_chunk({}))


@unittest.skipIf(OpenpiRemoteActionSource is None, "torch is not installed")
class OpenpiRemoteRtcReadyEventTest(unittest.TestCase):
    """ready_event + RTC (2026-09-09): adaptive freeze depth, measured shift, and the
    previous chunk being the last ACTIVATED result rather than the last inference."""

    def _adaptive_source(self, samples_ms, *, margin=0, d_max=8, horizon=24, execute=4):
        import threading
        from collections import deque

        src, raw = _make_source(rtc_enabled=True, horizon=horizon, chunk_execute_steps=execute)
        src.rtc_delay_policy = "adaptive"
        src.rtc_delay_margin_steps = margin
        src.rtc_delay_max_steps = d_max
        src.rtc_inference_delay = 3
        src.policy_dt_sec = 0.0334
        src._inference_timing_lock = threading.Lock()
        src._inference_timing_history = {"request_to_activation_ms": deque(samples_ms, maxlen=64)}
        src._stream_emitted_policy_steps = 0
        return src, raw

    @staticmethod
    def _activate(src, seq, obs, ssi=3):
        src.on_rtc_chunk_activated({"inference_seq": seq, "observation_step_seq": obs, "source_start_index": ssi})

    def test_worker_context_stashes_raw_until_activation(self) -> None:
        src, raw = self._adaptive_source([70, 80, 95, 100, 110])
        src.note_rtc_request_context(10, 1)
        src._sample_chunk({})
        src.clear_rtc_request_context()
        # not yet the previous chunk: the result has not been activated
        self.assertIsNone(src._rtc_prev_raw_chunk)
        self.assertIn(1, src._rtc_pending_raw)
        self.assertTrue(src._rtc_last_sent["cold_start"])
        self._activate(src, 1, 10)
        self.assertTrue(np.allclose(src._rtc_prev_raw_chunk, raw))
        self.assertEqual(src._rtc_prev_obs_step_seq, 10)
        self.assertEqual(src._rtc_pending_raw, {})
        self.assertEqual(list(src._rtc_realized_delay_history), [3])

    def test_discarded_result_never_seeds_the_freeze(self) -> None:
        src, raw = self._adaptive_source([70, 80, 95, 100, 110])
        src.note_rtc_request_context(10, 1)
        src._sample_chunk({})
        src.clear_rtc_request_context()
        src.on_rtc_chunk_discarded({"inference_seq": 1})
        self.assertIsNone(src._rtc_prev_raw_chunk)
        self.assertEqual(src._rtc_pending_raw, {})
        # the next request therefore stays vanilla (no prev to continue)
        src.note_rtc_request_context(13, 2)
        src._sample_chunk({})
        src.clear_rtc_request_context()
        self.assertNotIn("prev_action_chunk", src._client.sent_obs[-1])

    def test_adaptive_delay_and_measured_shift_ride_in_the_request(self) -> None:
        src, raw = self._adaptive_source([70, 80, 95, 100, 110])
        # realized-delay history 3,3,3,4 -> p10 = 3 -> d = 3 (never above the typical ssi)
        for seq, ssi in ((11, 3), (12, 3), (13, 3), (14, 4)):
            src._note_rtc_realized_delay(ssi)
        src.note_rtc_request_context(10, 1)
        src._sample_chunk({})
        src.clear_rtc_request_context()
        self._activate(src, 1, 10)
        src.note_rtc_request_context(13, 2)  # observed 3 steps after the executing chunk's observation
        src._sample_chunk({})
        src.clear_rtc_request_context()
        obs = src._client.sent_obs[-1]
        self.assertEqual(obs["inference_delay"], 3)
        self.assertEqual(obs["execute_horizon"], 3)  # = measured shift, NOT chunk_execute_steps (4)
        self.assertTrue(np.allclose(obs["prev_action_chunk"], rtc_shift_prev_chunk(raw, 3)))
        sent = src._rtc_last_sent
        self.assertEqual((sent["policy"], sent["shift"], sent["delay"], sent["execute_horizon"]), ("adaptive", 3, 3, 3))
        self.assertTrue(sent["prev_conditioned"])
        self.assertEqual(sent["prev_observation_step_seq"], 10)

    def test_adaptive_delay_uses_low_quantile_of_realized_ssi(self) -> None:
        # over-freeze (d > ssi) executes the frozen->guided seam, so the estimate is p10 of
        # the realized ssi, not a high quantile of latency: 3,3,3,3,4,4 -> 3
        src, _ = self._adaptive_source([70, 80, 95, 100, 110])
        for ssi in (3, 3, 3, 3, 4, 4):
            src._note_rtc_realized_delay(ssi)
        self.assertEqual(src._adaptive_rtc_delay(), 3)
        src.rtc_delay_margin_steps = 1
        self.assertEqual(src._adaptive_rtc_delay(), 4)
        src.rtc_delay_margin_steps = -1
        self.assertEqual(src._adaptive_rtc_delay(), 2)
        src.rtc_delay_max_steps = 2
        src.rtc_delay_margin_steps = 3
        self.assertEqual(src._adaptive_rtc_delay(), 2)

    def test_adaptive_delay_time_fallback_floors_the_median(self) -> None:
        # no realized history yet: floor(p50(request->activation)/policy_dt): 95 ms -> 2
        src, _ = self._adaptive_source([70, 80, 95, 100, 110])
        self.assertEqual(src._adaptive_rtc_delay(), 2)
        src.rtc_delay_margin_steps = 1
        self.assertEqual(src._adaptive_rtc_delay(), 3)

    def test_adaptive_delay_falls_back_to_static_seed_until_history_exists(self) -> None:
        src, _ = self._adaptive_source([70, 80], margin=1)  # < 4 timing samples, no realized ssi
        self.assertEqual(src._adaptive_rtc_delay(), 3 + 1)

    def test_static_policy_keeps_fixed_steps_contract_with_hooks(self) -> None:
        src, raw = _make_source(rtc_enabled=True, horizon=24, chunk_execute_steps=4)
        src.rtc_delay_policy = "static"
        src.rtc_inference_delay = 3
        src.note_rtc_request_context(0, 1)
        src._sample_chunk({})
        src.clear_rtc_request_context()
        src.on_rtc_chunk_activated({"inference_seq": 1, "observation_step_seq": 0, "source_start_index": 3})
        src.note_rtc_request_context(4, 2)
        src._sample_chunk({})
        src.clear_rtc_request_context()
        obs = src._client.sent_obs[-1]
        self.assertEqual(obs["inference_delay"], 3)
        self.assertEqual(obs["execute_horizon"], 4)  # replan period, unchanged
        self.assertTrue(np.allclose(obs["prev_action_chunk"], rtc_shift_prev_chunk(raw, 4)))
        self.assertEqual(src._rtc_last_sent["policy"], "static")

    def test_reset_rtc_clears_pending_and_bookkeeping(self) -> None:
        src, _ = self._adaptive_source([70, 80, 95, 100, 110])
        src.note_rtc_request_context(10, 1)
        src._sample_chunk({})
        src.clear_rtc_request_context()
        src._vel_prev_pose_by_arm = {"left": None, "right": None}
        src._vel_prev_sample_t = None
        src._velproprio_history_lock = __import__("threading").Lock()
        src._velproprio_history = {"left": [], "right": []}
        try:
            src.reset_rtc()
        except AttributeError:
            # reset_rtc also touches velocity-proprio buffers the bare source lacks; the RTC
            # part runs first and is what this test locks.
            pass
        self.assertEqual(src._rtc_pending_raw, {})
        self.assertIsNone(src._rtc_prev_raw_chunk)
        self.assertIsNone(src._rtc_last_sent)


if __name__ == "__main__":
    unittest.main()
