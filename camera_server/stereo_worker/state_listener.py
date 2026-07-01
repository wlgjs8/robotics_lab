"""rb_servo_server state(UDP JSON) 구독 → 실시간 TCP pose(T_stand_tcp) 제공.

아이디어 2(head+손목 클라우드 병합)용. 손목 raw 클라우드를 stand 프레임에 놓으려면
  P_stand = T_stand_tcp(arm) @ T_tcp_cam @ P_wrist_cam
가 필요한데, T_stand_tcp 는 rb_servo_server 가 state fanout(udp)에 publish 하는
per-arm `tcp_actual_stand`(position + quaternion_xyzw)에서 얻는다.

rb_servo_server config(network.state_pub_endpoints)에 worker용 엔드포인트를 추가해야 한다
(예: "udp://127.0.0.1:50386"). state 미수신 시 get()은 None → 병합은 자동 비활성(head만).
"""
from __future__ import annotations
import json
import socket
import threading
import time
from collections import deque
import numpy as np


def quat_xyzw_to_mat(q) -> np.ndarray:
    x, y, z, w = [float(v) for v in q]
    n = x*x + y*y + z*z + w*w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s*(y*y + z*z), s*(x*y - z*w),     s*(x*z + y*w)],
        [s*(x*y + z*w),     1 - s*(x*x + z*z), s*(y*z - x*w)],
        [s*(x*z - y*w),     s*(y*z + x*w),     1 - s*(x*x + y*y)],
    ])


def _parse_endpoint(ep: str):
    s = ep.split("://", 1)[-1]
    host, port = s.rsplit(":", 1)
    return host, int(port)


def _pose_to_T(p):
    """state의 tcp_actual_stand dict → 4x4 T_stand_tcp. 쿼터니언 없으면 None."""
    if not isinstance(p, dict):
        return None
    try:
        pos = [float(p["x"]), float(p["y"]), float(p["z"])]
    except Exception:
        return None
    q = None
    raw = p.get("quaternion_xyzw")
    if isinstance(raw, (list, tuple)) and len(raw) == 4:
        q = raw
    elif all(k in p for k in ("qx", "qy", "qz", "qw")):
        q = (p["qx"], p["qy"], p["qz"], p["qw"])
    if q is None:
        return None
    T = np.eye(4)
    T[:3, :3] = quat_xyzw_to_mat(q)
    T[:3, 3] = pos
    return T


class TcpPoseListener:
    """state fanout(udp)를 구독해 per-arm 최신 T_stand_tcp를 보관(스레드)."""

    def __init__(self, endpoint="udp://127.0.0.1:50386", stale_s=0.3, hist_window_s=0.2):
        host, port = _parse_endpoint(endpoint)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((host, port))
        self._sock.settimeout(0.5)
        self._stale_s = stale_s
        self._hist_window_s = hist_window_s
        self._lock = threading.Lock()
        self._poses: dict[str, tuple[np.ndarray, float]] = {}
        # per-arm 최근 pose 이력(모션 추정용). (t, pos(3), R(3x3)).
        self._hist: dict[str, deque] = {}
        self._rx = 0
        self._run = True
        self._thr = threading.Thread(target=self._loop, daemon=True)
        self._thr.start()

    def _loop(self):
        while self._run:
            try:
                data, _ = self._sock.recvfrom(1 << 16)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                m = json.loads(data.decode("utf-8", "replace"))
            except Exception:
                continue
            if not isinstance(m, dict):
                continue
            now = time.monotonic()
            for arm in ("left", "right"):
                a = m.get(arm)
                if not isinstance(a, dict):
                    continue
                T = _pose_to_T(a.get("tcp_actual_stand") or a.get("tcp_stand"))
                if T is not None:
                    with self._lock:
                        self._poses[arm] = (T, now)
                        h = self._hist.get(arm)
                        if h is None:
                            h = self._hist[arm] = deque(maxlen=32)
                        h.append((now, T[:3, 3].copy(), T[:3, :3].copy()))
                        self._rx += 1

    def get(self, arm: str):
        """최신 T_stand_tcp(4x4) 또는 None(미수신/stale)."""
        with self._lock:
            v = self._poses.get(arm)
        if v is None:
            return None
        T, t = v
        if time.monotonic() - t > self._stale_s:
            return None
        return T

    def motion(self, arm: str):
        """최근 ~hist_window_s 동안의 TCP 속도 (lin m/s, ang rad/s). 추정 불가 시 None.
        손목 클라우드를 stand로 옮길 때 팔이 움직이면 pose↔프레임 비동기로 점이 어긋나므로
        '정지 판정'(is_settled)에 쓴다."""
        now = time.monotonic()
        with self._lock:
            h = self._hist.get(arm)
            samples = list(h) if h is not None else []
        if len(samples) < 2:
            return None
        t_new, p_new, R_new = samples[-1]
        if now - t_new > self._stale_s:            # 최신 샘플이 stale → 신뢰 불가
            return None
        base = samples[0]                          # newest 기준 ~window 과거 baseline
        for s in samples:
            if t_new - s[0] <= self._hist_window_s:
                base = s
                break
        t_old, p_old, R_old = base
        dt = t_new - t_old
        if dt < 1e-3:
            return None
        lin = float(np.linalg.norm(p_new - p_old) / dt)
        cos = (float(np.trace(R_old.T @ R_new)) - 1.0) / 2.0
        ang = float(np.arccos(max(-1.0, min(1.0, cos))) / dt)
        return lin, ang

    def is_settled(self, arm: str, lin_thr=0.03, ang_thr=0.15):
        """팔이 (거의) 정지 상태인가. 모션 추정 불가/이동 중이면 False (보수적: 융합 보류)."""
        mo = self.motion(arm)
        if mo is None:
            return False
        return mo[0] <= lin_thr and mo[1] <= ang_thr

    @property
    def rx_count(self) -> int:
        return self._rx

    def close(self):
        self._run = False
        try:
            self._sock.close()
        except Exception:
            pass
