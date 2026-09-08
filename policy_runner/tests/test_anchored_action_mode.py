"""FLOW_INFER_ACTION_MODE=anchored: reception-time conversion + RTC re-anchoring.

Contract under test (the TRAINING side's, openpi pika_umi_policy._anchor_relative_chunk
on a loader window starting at the observation frame):
  row k of an anchored chunk = pose at t0+k in the chunk-start frame
  (p_k = p0 + R0 a_k, R_k = R0 A_k), so row 0 is the identity. The conversion yields
  H-1 per-step deltas, delta j = motion over [t0+j, t0+j+1]; chaining d_0..d_{j-1}
  must reproduce anchored row j exactly, the model's noisy row 0 must be ignored, and
  the RTC shift must re-express the unexecuted tail relative to the row the robot has
  reached after s executed steps, row s (T'_k = T_s^-1 T_{k+s}, T'_0 = I).
"""
import numpy as np
import pytest

from policy_runner.openpi_remote import (
    _mat_to_rotvec,
    _rotvec_to_mat,
    anchored_chunk_to_deltas,
    rtc_shift_prev_chunk,
)


def _random_anchored_chunk(rng, horizon=24, row0_noise=0.0):
    """Training-shaped chunk: row 0 = identity (optionally + model noise), rows 1.. a
    random walk of poses expressed in the row-0 frame."""
    chunk = np.zeros((horizon, 14), dtype=np.float32)
    for b in (0, 7):
        a = np.zeros(3)
        A = np.eye(3)
        for k in range(1, horizon):
            a = a + rng.uniform(-0.01, 0.01, 3)
            A = A @ _rotvec_to_mat(rng.uniform(-0.05, 0.05, 3))
            chunk[k, b:b + 3] = a
            chunk[k, b + 3:b + 6] = _mat_to_rotvec(A)
        if row0_noise > 0.0:
            chunk[0, b:b + 6] = rng.uniform(-row0_noise, row0_noise, 6)
        chunk[:, b + 6] = rng.uniform(0.0, 1.0, horizon)
    return chunk


def test_rotvec_roundtrip_near_pi():
    r = np.array([0.0, 3.10, 0.0])
    assert np.allclose(_mat_to_rotvec(_rotvec_to_mat(r)), r, atol=1e-9)


def test_deltas_chain_back_to_anchored_waypoints():
    rng = np.random.default_rng(0)
    chunk = _random_anchored_chunk(rng)
    deltas = anchored_chunk_to_deltas(chunk)
    # the identity row is consumed: H-1 motions for H poses
    assert deltas.shape == (chunk.shape[0] - 1, chunk.shape[1])
    # gripper columns NOT shifted: delta j (motion over [t0+j, t0+j+1]) carries grip[t0+j],
    # the delta-mode convention (a step's gripper is read at the START of the step)
    assert np.array_equal(deltas[:, 6], chunk[:-1, 6])
    assert np.array_equal(deltas[:, 13], chunk[:-1, 13])
    for b in (0, 7):
        p = np.zeros(3)
        R = np.eye(3)
        for j in range(deltas.shape[0]):
            p = p + R @ deltas[j, b:b + 3].astype(np.float64)
            R = R @ _rotvec_to_mat(deltas[j, b + 3:b + 6])
            # after executing d_0..d_j the robot is at anchored row j+1
            assert np.allclose(p, chunk[j + 1, b:b + 3], atol=1e-5), (b, j)
            assert np.allclose(_mat_to_rotvec(R), chunk[j + 1, b + 3:b + 6], atol=1e-4), (b, j)


def test_first_delta_row_equals_row1_and_row0_noise_is_ignored():
    """Row 0 is the identity by construction; the model's ~0.5 mm regression noise there
    must not leak into the first executed motion (the pre-2026-09-08 code executed row 0
    as a null step and every later motion one step late)."""
    rng = np.random.default_rng(1)
    chunk = _random_anchored_chunk(rng)
    deltas = anchored_chunk_to_deltas(chunk)
    for b in (0, 7):
        assert np.allclose(deltas[0, b:b + 6], chunk[1, b:b + 6], atol=1e-6)
    noisy = chunk.copy()
    for b in (0, 7):
        noisy[0, b:b + 6] = rng.uniform(-0.002, 0.002, 6)  # model noise on the identity row
    noisy_deltas = anchored_chunk_to_deltas(noisy)
    assert np.allclose(noisy_deltas, deltas, atol=1e-7)


def test_short_chunk_yields_no_deltas():
    assert anchored_chunk_to_deltas(np.zeros((1, 14), dtype=np.float32)).shape == (0, 14)


def test_rtc_shift_anchored_reanchors_to_boundary_row():
    rng = np.random.default_rng(2)
    chunk = _random_anchored_chunk(rng)
    steps = 4
    ident = (np.full(14, -1.0), np.full(14, 1.0))   # q01=-1,q99=1 -> near-identity affine
    shifted = rtc_shift_prev_chunk(chunk, steps, action_mode="anchored", norm_q=ident)
    assert shifted.shape == chunk.shape
    assert np.array_equal(shifted[-steps:], np.zeros_like(shifted[-steps:]))
    for b in (0, 7):
        # after `steps` executed motions the robot stands on row `steps`: that row is
        # the new anchor, so the shifted chunk starts with an identity row like the
        # model's own output does
        assert np.allclose(shifted[0, b:b + 6], 0.0, atol=1e-6)
        ps = chunk[steps, b:b + 3].astype(np.float64)
        Rs = _rotvec_to_mat(chunk[steps, b + 3:b + 6])
        for k in range(chunk.shape[0] - steps):
            pk = chunk[steps + k, b:b + 3].astype(np.float64)
            Rk = _rotvec_to_mat(chunk[steps + k, b + 3:b + 6])
            assert np.allclose(shifted[k, b:b + 3], Rs.T @ (pk - ps), atol=1e-5)
            assert np.allclose(shifted[k, b + 3:b + 6], _mat_to_rotvec(Rs.T @ Rk), atol=1e-4)
        # gripper columns ride along unshifted
        assert np.array_equal(shifted[:-steps, b + 6], chunk[steps:, b + 6])


def test_rtc_shift_anchored_exhausted_plan_returns_zeros():
    rng = np.random.default_rng(9)
    chunk = _random_anchored_chunk(rng)
    ident = (np.full(14, -1.0), np.full(14, 1.0))
    out = rtc_shift_prev_chunk(chunk, chunk.shape[0], action_mode="anchored", norm_q=ident)
    assert out.shape == chunk.shape and not out.any()


def test_rtc_shift_delta_mode_unchanged():
    rng = np.random.default_rng(3)
    chunk = _random_anchored_chunk(rng)
    out = rtc_shift_prev_chunk(chunk, 4)
    assert np.array_equal(out[:-4], chunk[4:])


def test_action_mode_validation():
    from policy_runner.openpi_remote import OpenpiRemoteActionSource
    with pytest.raises(ValueError, match="action_mode"):
        OpenpiRemoteActionSource("openpi://127.0.0.1:9", action_mode="bogus")


def _fake_norm_q():
    q01 = np.concatenate([[-0.08, -0.12, -0.13], [-1.1, -0.9, -1.0], [0.0],
                          [-0.07, -0.1, -0.12], [-1.0, -1.2, -0.8], [0.0]])
    q99 = np.concatenate([[0.03, 0.02, 0.09], [1.2, 1.0, 0.9], [1.0],
                          [0.05, 0.03, 0.08], [1.1, 0.9, 1.2], [1.0]])
    return q01, q99


def _norm(x, q01, q99):
    return (x - q01) / (q99 - q01 + 1e-6) * 2.0 - 1.0


def test_rtc_shift_anchored_normalized_space_roundtrip():
    """The shift must equal direct SE(3) math on UNNORMALIZED rows (the 20260825
    real-robot regression: matrix algebra on normalized values -> base-ward drift)."""
    rng = np.random.default_rng(7)
    q01, q99 = _fake_norm_q()
    chunk_un = _random_anchored_chunk(rng).astype(np.float64)
    chunk_norm = _norm(chunk_un, q01, q99).astype(np.float32)
    steps = 4
    out_norm = rtc_shift_prev_chunk(chunk_norm, steps, action_mode="anchored",
                                    norm_q=(q01, q99))
    # reference: direct math on unnormalized rows, then renormalize
    ref_un = chunk_un[steps:].copy()
    for b in (0, 7):
        ps = chunk_un[steps, b:b + 3]
        Rs = _rotvec_to_mat(chunk_un[steps, b + 3:b + 6])
        for k in range(ref_un.shape[0]):
            pk = chunk_un[steps + k, b:b + 3]
            Rk = _rotvec_to_mat(chunk_un[steps + k, b + 3:b + 6])
            ref_un[k, b:b + 3] = Rs.T @ (pk - ps)
            ref_un[k, b + 3:b + 6] = _mat_to_rotvec(Rs.T @ Rk)
    ref_norm = _norm(ref_un, q01, q99)
    assert np.allclose(out_norm[:-steps], ref_norm, atol=1e-4)
    assert np.array_equal(out_norm[-steps:], np.zeros_like(out_norm[-steps:]))


def test_rtc_shift_anchored_without_stats_raises():
    rng = np.random.default_rng(8)
    with pytest.raises(ValueError, match="norm stats"):
        rtc_shift_prev_chunk(_random_anchored_chunk(rng), 4, action_mode="anchored")


def _bare_source(**attrs):
    from policy_runner.openpi_remote import OpenpiRemoteActionSource
    src = OpenpiRemoteActionSource.__new__(OpenpiRemoteActionSource)
    for k, v in attrs.items():
        setattr(src, k, v)
    return src


def test_dynamic_rtc_params_first_call_uses_static():
    s = _bare_source(rtc_inference_delay=3, _stream_emitted_policy_steps=10)
    shift, delay = s._dynamic_rtc_params(4)
    assert (shift, delay) == (4, 3)


def test_dynamic_rtc_params_tracks_obs_spacing_and_realized():
    s = _bare_source(rtc_inference_delay=3, _stream_emitted_policy_steps=10)
    s._dynamic_rtc_params(4)                       # seeds last obs seq = 10
    s._stream_emitted_policy_steps = 16            # stall stretched spacing to 6
    s._active_chunk_metadata = {"source_start_index": 2}
    shift, delay = s._dynamic_rtc_params(4)
    assert shift == 6                              # measured obs-to-obs, not static 4
    assert delay == 3                              # STATIC configured (d>=realized safety)


def test_dynamic_rtc_params_clamps():
    s = _bare_source(rtc_inference_delay=3, _stream_emitted_policy_steps=0)
    s._dynamic_rtc_params(4)
    s._stream_emitted_policy_steps = 1000          # absurd gap -> clamped
    s._active_chunk_metadata = {"source_start_index": 99}
    shift, delay = s._dynamic_rtc_params(4)
    assert shift == 16                             # 4 * replan cap
    assert delay == 3                              # static configured, replan-clamped
