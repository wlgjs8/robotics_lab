"""A peer arm must not lose its chunk stream because the OTHER arm is recovering.

`roi_auto_recover` moves one arm home with a JointTarget. That arm's
reset-relative anchor is stale afterwards, so its chunk rows are worthless --
but the peer arm never moved, and its rows are still good.

on_arm_init_override_start used to call _invalidate_policy_chunks()
unconditionally, which sets `self._chunk = None`. _publish_chunk_overlay then
returns early on `chunk is None`, for BOTH arms, so the whole stream went dark.

MEASURED, servo_log_20260828_004539.csv: 7 right-arm roi_auto_recover events
(all on the y_max face) each cost the LEFT arm its chunk follower. The left mode
dropped to Hold, the server logged "disengaged (mode/enable)", and the arm ran
on the SMD fallback for the entire override -- 0.9-2.1 s, with 5-11 chunk frames
arriving and being skipped -- before re-engaging with a 12,113 deg/s^2 burst,
the largest command acceleration anywhere in that 231 s run. The left arm had
not moved and had no reason to stop.

The overridden arm is masked out the moment the override starts, and every
consumer of the chunk skips a masked arm (_foh_begin_chunk, _foh_tick_intent,
_remember_emitted_deltas_for_step, _publish_chunk_overlay all `continue` on
arm_mask[idx] <= 0), so keeping the chunk costs nothing and keeps the peer
running.

The decision lives in arm_init_control so it is testable without importing
flow_inference, which needs torch.
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from policy_runner.arm_init_control import should_invalidate_chunks_for_override


class PeerArmKeepsItsChunk(unittest.TestCase):
    def test_right_recovery_keeps_the_left_chunk(self):
        """THE 2026-08-28 INCIDENT: 7 right-arm recoveries, left arm collateral."""
        self.assertFalse(should_invalidate_chunks_for_override([1.0, 1.0], ("right",)))

    def test_left_recovery_keeps_the_right_chunk(self):
        self.assertFalse(should_invalidate_chunks_for_override([1.0, 1.0], ("left",)))

    def test_both_arms_overridden_still_invalidates(self):
        """With nothing left to protect the old global behaviour is right."""
        self.assertTrue(
            should_invalidate_chunks_for_override([1.0, 1.0], ("left", "right"))
        )

    def test_single_arm_checkpoint_still_invalidates(self):
        """A checkpoint driving only the right arm: overriding it leaves no peer."""
        self.assertTrue(should_invalidate_chunks_for_override([0.0, 1.0], ("right",)))
        self.assertTrue(should_invalidate_chunks_for_override([1.0, 0.0], ("left",)))

    def test_an_already_masked_peer_does_not_count_as_driven(self):
        """Keeping a chunk for an arm that is not being commanded protects nothing."""
        self.assertTrue(should_invalidate_chunks_for_override([0.0, 1.0], ("right",)))
        self.assertFalse(should_invalidate_chunks_for_override([1.0, 0.0], ("right",)))

    def test_no_override_arms_keeps_the_chunk(self):
        self.assertFalse(should_invalidate_chunks_for_override([1.0, 1.0], ()))

    def test_malformed_mask_is_treated_as_not_driven(self):
        """A mask that cannot be read must not silently keep a stale chunk alive."""
        self.assertTrue(should_invalidate_chunks_for_override([], ("right",)))
        self.assertTrue(should_invalidate_chunks_for_override(["x", "y"], ("right",)))
        self.assertTrue(should_invalidate_chunks_for_override([1.0], ("left",)))

    def test_numpy_mask(self):
        try:
            import numpy as np
        except ImportError:  # pragma: no cover
            self.skipTest("numpy unavailable")
        mask = np.asarray([1.0, 1.0], dtype=np.float32)
        self.assertFalse(should_invalidate_chunks_for_override(mask, ("right",)))
        self.assertTrue(should_invalidate_chunks_for_override(mask, ("left", "right")))


if __name__ == "__main__":
    unittest.main()
