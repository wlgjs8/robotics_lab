from __future__ import annotations

import unittest

try:
    import numpy as np

    from policy_runner.flow_inference import FlowMatchingActionSource, _ZERO_TWIST
except Exception:  # pragma: no cover - numpy/torch optional in some envs
    np = None


@unittest.skipIf(np is None, "numpy is not installed")
class ChunkCrossfadeTest(unittest.TestCase):
    """Unit-tests the chunk-boundary twist crossfade in isolation.

    The crossfade blends the first ``chunk_crossfade_steps`` twists of a freshly
    activated chunk from the previously emitted twist so velocity is continuous
    across the resample boundary, without adding steady-state lag (steps past the
    window pass through unchanged)."""

    def _source(self, k: int, prev: tuple[float, ...] | None) -> FlowMatchingActionSource:
        # Skip __init__ (it loads a checkpoint); set only what the method reads.
        src = FlowMatchingActionSource.__new__(FlowMatchingActionSource)
        src._chunk_crossfade_steps = k
        src._steps_since_boundary = 0
        src._prev_emitted_twist_by_arm = {"left": prev, "right": prev}
        return src

    def test_disabled_is_passthrough(self) -> None:
        src = self._source(k=0, prev=_ZERO_TWIST)
        twist = (0.2, 0.0, 0.1, 0.0, 0.0, 0.0)
        self.assertEqual(src._apply_chunk_crossfade("left", twist), twist)

    def test_no_prior_twist_is_passthrough(self) -> None:
        src = self._source(k=3, prev=None)
        twist = (0.2, 0.0, 0.1, 0.0, 0.0, 0.0)
        self.assertEqual(src._apply_chunk_crossfade("left", twist), twist)

    def test_ramps_from_prev_to_new_then_passthrough(self) -> None:
        k = 2
        prev = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        new = (0.3, 0.0, 0.0, 0.0, 0.0, 0.0)
        src = self._source(k=k, prev=prev)
        # step 0: alpha = 1/(k+1) = 1/3
        src._steps_since_boundary = 0
        out0 = src._apply_chunk_crossfade("left", new)
        self.assertAlmostEqual(out0[0], 0.3 * (1.0 / 3.0))
        # step 1: alpha = 2/3
        src._steps_since_boundary = 1
        out1 = src._apply_chunk_crossfade("left", new)
        self.assertAlmostEqual(out1[0], 0.3 * (2.0 / 3.0))
        # step 2 (>= k): full new twist, no blend (no lingering lag)
        src._steps_since_boundary = 2
        out2 = src._apply_chunk_crossfade("left", new)
        self.assertEqual(out2, new)
        # monotonic ramp in
        self.assertLess(out0[0], out1[0])
        self.assertLess(out1[0], out2[0])

    def test_blend_reduces_boundary_jump(self) -> None:
        # A large velocity reversal at the boundary: prev moving +x, new -x.
        prev = (0.25, 0.0, 0.0, 0.0, 0.0, 0.0)
        new = (-0.25, 0.0, 0.0, 0.0, 0.0, 0.0)
        src = self._source(k=3, prev=prev)
        src._steps_since_boundary = 0
        out = src._apply_chunk_crossfade("left", new)
        raw_jump = abs(new[0] - prev[0])
        blended_jump = abs(out[0] - prev[0])
        self.assertLess(blended_jump, raw_jump)  # boundary jerk reduced

    def test_emit_advances_boundary_counter_and_activate_resets(self) -> None:
        src = self._source(k=2, prev=_ZERO_TWIST)
        src._steps_since_boundary = 5  # mid-chunk
        # _activate_chunk must restart the ramp so a new chunk crossfades.
        src._chunk_index = 9
        src._step_deadline = 0.0
        src.policy_dt_sec = 1.0 / 30.0
        chunk = np.zeros((4, 14), dtype=np.float64)
        src._activate_chunk(chunk, now_monotonic=0.0)
        self.assertEqual(src._steps_since_boundary, 0)


if __name__ == "__main__":
    unittest.main()
