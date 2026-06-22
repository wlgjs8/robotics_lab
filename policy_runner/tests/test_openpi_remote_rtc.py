"""Unit tests for the RTC client wiring in OpenpiRemoteActionSource (M3).

Drives _sample_chunk with a fake websocket client (no server, no model) to lock:
- RTC OFF: no prev fields in the obs, nothing cached.
- RTC ON cold start: no prev sent, but the server's MODEL-SPACE rtc_raw_actions is
  cached (NOT the gripper-rescaled actions).
- RTC ON warm: prev_action_chunk + d/execute_horizon/schedule/max_guidance_weight
  ride in the obs, and prev == the cached raw chunk.
- reset_rtc cold-starts again.

Built via __new__ to skip the connecting __init__. Guarded by torch availability
(OpenpiRemoteActionSource imports flow_inference, which imports torch).
"""

from __future__ import annotations

import unittest

try:
    import numpy as np

    from policy_runner.openpi_remote import OpenpiRemoteActionSource
except Exception:  # torch (a transitive import) may be absent
    np = None
    OpenpiRemoteActionSource = None  # type: ignore[assignment]

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


def _make_source(*, rtc_enabled: bool, raw=None) -> "OpenpiRemoteActionSource":
    assert OpenpiRemoteActionSource is not None
    src = OpenpiRemoteActionSource.__new__(OpenpiRemoteActionSource)
    # actions: gripper dims at 0.5 (-> *100 = 50 in the returned chunk).
    actions = np.zeros((_HORIZON, 14), dtype=np.float32)
    actions[:, _GRIP_LEFT] = 0.5
    actions[:, _GRIP_RIGHT] = 0.5
    # raw (model space) is a DISTINCT constant so we can tell it apart from actions.
    raw_chunk = np.full((_HORIZON, 14), 7.0, dtype=np.float32) if raw is None else raw
    src._client = _FakeClient(actions, raw_chunk if rtc_enabled else None)
    # Stub the obs builders so _sample_chunk runs without cameras / a model.
    src._raw_camera_images = lambda: ({"left": object(), "right": object()}, 2, 0)
    src._proprio_state = lambda payload: np.zeros(14, dtype=np.float32)
    src.prompt = "task"
    src.action_horizon = _HORIZON
    src.stderr = __import__("sys").stderr
    src.last_image_decode_count = 0
    src.last_missing_camera_count = 0
    src.image_decode_count = 0
    src.missing_camera_count = 0
    # RTC state.
    src.rtc_enabled = rtc_enabled
    src.rtc_inference_delay = 2
    src.chunk_execute_steps = 3
    src.rtc_prefix_attention_schedule = "exp"
    src.rtc_max_guidance_weight = 5.0
    src._rtc_prev_raw_chunk = None
    src._rtc_warned_no_raw = False
    return src, raw_chunk


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
        self.assertTrue(np.allclose(obs["prev_action_chunk"], raw))
        self.assertEqual(obs["inference_delay"], 2)  # min(2, chunk_execute_steps=3)
        self.assertEqual(obs["execute_horizon"], 3)
        self.assertEqual(obs["prefix_attention_schedule"], "exp")
        self.assertEqual(obs["max_guidance_weight"], 5.0)

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


if __name__ == "__main__":
    unittest.main()
