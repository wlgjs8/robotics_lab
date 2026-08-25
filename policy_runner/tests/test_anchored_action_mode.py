"""FLOW_INFER_ACTION_MODE=anchored: reception-time conversion + RTC re-anchoring.

Contract under test (matches the sim rig's ACTION_MODE=anchored):
  row k of an anchored chunk = pose at t0+k+1 in the chunk-start frame
  (p_k = p0 + R0 a_k, R_k = R0 A_k). Chaining the converted per-step deltas must
  reproduce those waypoints exactly, and the RTC shift must re-express the
  unexecuted tail relative to the executed boundary row (T'_k = T_s^-1 T_{k+s}).
"""
import numpy as np
import pytest

from policy_runner.openpi_remote import (
    _mat_to_rotvec,
    _rotvec_to_mat,
    anchored_chunk_to_deltas,
    rtc_shift_prev_chunk,
)


def _random_anchored_chunk(rng, horizon=24):
    chunk = np.zeros((horizon, 14), dtype=np.float32)
    for b in (0, 7):
        a = np.zeros(3)
        A = np.eye(3)
        for k in range(horizon):
            a = a + rng.uniform(-0.01, 0.01, 3)
            A = A @ _rotvec_to_mat(rng.uniform(-0.05, 0.05, 3))
            chunk[k, b:b + 3] = a
            chunk[k, b + 3:b + 6] = _mat_to_rotvec(A)
        chunk[:, b + 6] = rng.uniform(0.0, 1.0, horizon)
    return chunk


def test_rotvec_roundtrip_near_pi():
    r = np.array([0.0, 3.10, 0.0])
    assert np.allclose(_mat_to_rotvec(_rotvec_to_mat(r)), r, atol=1e-9)


def test_deltas_chain_back_to_anchored_waypoints():
    rng = np.random.default_rng(0)
    chunk = _random_anchored_chunk(rng)
    deltas = anchored_chunk_to_deltas(chunk)
    assert deltas.shape == chunk.shape
    # gripper columns untouched
    assert np.array_equal(deltas[:, 6], chunk[:, 6])
    assert np.array_equal(deltas[:, 13], chunk[:, 13])
    for b in (0, 7):
        p = np.zeros(3)
        R = np.eye(3)
        for k in range(chunk.shape[0]):
            p = p + R @ deltas[k, b:b + 3].astype(np.float64)
            R = R @ _rotvec_to_mat(deltas[k, b + 3:b + 6])
            assert np.allclose(p, chunk[k, b:b + 3], atol=1e-5), (b, k)
            assert np.allclose(_mat_to_rotvec(R), chunk[k, b + 3:b + 6], atol=1e-4), (b, k)


def test_first_delta_row_equals_first_anchored_row():
    rng = np.random.default_rng(1)
    chunk = _random_anchored_chunk(rng)
    deltas = anchored_chunk_to_deltas(chunk)
    for b in (0, 7):
        assert np.allclose(deltas[0, b:b + 6], chunk[0, b:b + 6], atol=1e-6)


def test_rtc_shift_anchored_reanchors_to_boundary_row():
    rng = np.random.default_rng(2)
    chunk = _random_anchored_chunk(rng)
    steps = 4
    ident = (np.full(14, -1.0), np.full(14, 1.0))   # q01=-1,q99=1 -> near-identity affine
    shifted = rtc_shift_prev_chunk(chunk, steps, action_mode="anchored", norm_q=ident)
    assert shifted.shape == chunk.shape
    assert np.array_equal(shifted[-steps:], np.zeros_like(shifted[-steps:]))
    for b in (0, 7):
        ps = chunk[steps - 1, b:b + 3].astype(np.float64)
        Rs = _rotvec_to_mat(chunk[steps - 1, b + 3:b + 6])
        for k in range(chunk.shape[0] - steps):
            pk = chunk[steps + k, b:b + 3].astype(np.float64)
            Rk = _rotvec_to_mat(chunk[steps + k, b + 3:b + 6])
            assert np.allclose(shifted[k, b:b + 3], Rs.T @ (pk - ps), atol=1e-5)
            assert np.allclose(shifted[k, b + 3:b + 6], _mat_to_rotvec(Rs.T @ Rk), atol=1e-4)


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
        ps = chunk_un[steps - 1, b:b + 3]
        Rs = _rotvec_to_mat(chunk_un[steps - 1, b + 3:b + 6])
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
