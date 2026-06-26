"""stereo_worker가 publish하는 pointcloud(ZMQ)를 구독해 보관.

stereo_worker(camera_server 컨테이너)가 `stereo.cloud` 토픽으로 **카메라 광학 좌표계**의
XYZRGB 클라우드를 PUB한다. 여기서는 그대로(카메라 프레임) 저장만 하고, stand 프레임 배치는
viser 씬에서 `/stereo_cam` 프레임(=T_stand_cam)의 자식으로 렌더해서 처리한다(기즈모 수동
캘리브레이션 지원). rb_gui는 torch 불필요 — numpy+zmq만.

publish 계약: multipart [topic, header_json{seq,ts_ns,n,frame}, xyz_f32(Nx3), rgb_u8(Nx3)].
"""
from __future__ import annotations
import json
import os
import threading
import time
import numpy as np

DEFAULT_T_STAND_CAM = os.environ.get("RB_GUI_T_STAND_CAM", "/home/plaif/workspace/T_stand_cam.npy")


def load_T_stand_cam(path: str = DEFAULT_T_STAND_CAM) -> np.ndarray:
    try:
        T = np.load(path)
        if T.shape == (4, 4):
            return T.astype(np.float64)
    except Exception:
        pass
    return np.eye(4)


def save_T_stand_cam(T: np.ndarray, path: str = DEFAULT_T_STAND_CAM) -> tuple[bool, str]:
    try:
        d = os.path.dirname(path)
        if d:
            os.makedirs(d, exist_ok=True)
        np.save(path, np.asarray(T, dtype=np.float64))
        return True, path
    except Exception as exc:  # noqa: BLE001
        return False, str(exc)


def mat_to_wxyz(R: np.ndarray) -> np.ndarray:
    m = np.asarray(R, dtype=np.float64)
    t = np.trace(m)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25*s, (m[2,1]-m[1,2])/s, (m[0,2]-m[2,0])/s, (m[1,0]-m[0,1])/s
    elif m[0,0] > m[1,1] and m[0,0] > m[2,2]:
        s = np.sqrt(1.0+m[0,0]-m[1,1]-m[2,2])*2
        w, x, y, z = (m[2,1]-m[1,2])/s, 0.25*s, (m[0,1]+m[1,0])/s, (m[0,2]+m[2,0])/s
    elif m[1,1] > m[2,2]:
        s = np.sqrt(1.0+m[1,1]-m[0,0]-m[2,2])*2
        w, x, y, z = (m[0,2]-m[2,0])/s, (m[0,1]+m[1,0])/s, 0.25*s, (m[1,2]+m[2,1])/s
    else:
        s = np.sqrt(1.0+m[2,2]-m[0,0]-m[1,1])*2
        w, x, y, z = (m[1,0]-m[0,1])/s, (m[0,2]+m[2,0])/s, (m[1,2]+m[2,1])/s, 0.25*s
    q = np.array([w, x, y, z]); return q/np.linalg.norm(q)


def wxyz_to_mat(wxyz) -> np.ndarray:
    w, x, y, z = np.asarray(wxyz, dtype=np.float64)
    n = w*w+x*x+y*y+z*z
    if n < 1e-12:
        return np.eye(3)
    w, x, y, z = np.array([w, x, y, z])/np.sqrt(n)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])


class StereoCloudStore:
    """스레드 안전 최신 클라우드 보관 (카메라 프레임 XYZ + RGB)."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._xyz: np.ndarray | None = None
        self._rgb: np.ndarray | None = None
        self._seq: int = -1
        self._recv_monotonic: float = 0.0

    def update(self, xyz: np.ndarray, rgb: np.ndarray, seq: int) -> None:
        with self._lock:
            self._xyz, self._rgb, self._seq = xyz, rgb, seq
            self._recv_monotonic = time.monotonic()

    def latest(self):
        """returns (xyz_cam, rgb, seq, age_ms) or None."""
        with self._lock:
            if self._xyz is None:
                return None
            age_ms = (time.monotonic() - self._recv_monotonic) * 1000.0
            return self._xyz, self._rgb, self._seq, age_ms


class StereoCloudReceiver:
    def __init__(self, store: StereoCloudStore, endpoint: str = "tcp://127.0.0.1:5601",
                 topic: str = "stereo.cloud") -> None:
        self.store = store
        self.endpoint = endpoint
        self.topic = topic
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="rb-gui-stereo-cloud", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def _run(self) -> None:
        try:
            import zmq
        except Exception:
            print("StereoCloudReceiver: pyzmq 없음 — pointcloud 비활성", flush=True)
            return
        ctx = zmq.Context.instance()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt_string(zmq.SUBSCRIBE, self.topic)
        sock.setsockopt(zmq.RCVHWM, 4)
        sock.connect(self.endpoint)
        poller = zmq.Poller()
        poller.register(sock, zmq.POLLIN)
        while not self._stop.is_set():
            if not dict(poller.poll(200)):
                continue
            try:
                parts = sock.recv_multipart()
                if len(parts) != 4:
                    continue
                _topic, header, xyz_b, rgb_b = parts
                meta = json.loads(header.decode("utf-8", "replace"))
                n = int(meta.get("n", 0))
                if n <= 0:
                    continue
                xyz = np.frombuffer(xyz_b, dtype=np.float32).reshape(n, 3)
                rgb = np.frombuffer(rgb_b, dtype=np.uint8).reshape(n, 3)
                self.store.update(np.ascontiguousarray(xyz), np.ascontiguousarray(rgb),
                                  int(meta.get("seq", -1)))
            except Exception:
                continue
        sock.close(linger=0)
