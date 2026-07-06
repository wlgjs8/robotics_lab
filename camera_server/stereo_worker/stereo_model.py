"""Fast-FoundationStereo 래퍼: (left,right) -> disparity -> depth -> colored pointcloud.

rb_gui와 분리된 GPU 워커에서 사용. torch는 이 프로세스(컨테이너)에만 필요.
백엔드 2종 (동일 인터페이스: infer_disparity / disparity_to_cloud):
  - FoundationStereoModel : PyTorch (.pth)
  - TrtStereoModel        : TensorRT fp16 엔진 (~14ms @640x480, RTX5090)
"""
from __future__ import annotations
import os
import sys
import numpy as np


def to3(img: np.ndarray) -> np.ndarray:
    """IR/grayscale(HxW) -> HxWx3, RGBA -> RGB."""
    if img.ndim == 2:
        img = np.tile(img[..., None], (1, 1, 3))
    return np.ascontiguousarray(img[..., :3])


def disp_to_cloud(depth2xyzmap, disp, rgb, K, baseline, zfar=3.0, remove_invisible=True):
    """disparity -> (xyz Nx3 [m, camera frame], colors Nx3 uint8). 두 백엔드 공용."""
    disp = disp.astype(np.float32).copy()
    if remove_invisible:
        _, xx = np.meshgrid(np.arange(disp.shape[0]), np.arange(disp.shape[1]), indexing="ij")
        disp[(xx - disp) < 0] = np.inf
    with np.errstate(divide="ignore", invalid="ignore"):
        depth = K[0, 0] * baseline / disp
    xyz = depth2xyzmap(depth, K).reshape(-1, 3)
    col = to3(rgb).reshape(-1, 3).astype(np.uint8)
    m = (xyz[:, 2] > 0) & (xyz[:, 2] <= zfar) & np.isfinite(xyz).all(1)
    return xyz[m], col[m]


class FoundationStereoModel:
    """PyTorch 백엔드."""
    def __init__(self, ffs_dir: str, weights: str,
                 valid_iters: int = 8, max_disp: int = 192, device: str = "cuda"):
        self.ffs_dir = os.path.realpath(ffs_dir)
        if self.ffs_dir not in sys.path:
            sys.path.insert(0, self.ffs_dir)
        import torch
        from core.utils.utils import InputPadder
        from Utils import AMP_DTYPE, depth2xyzmap
        self.torch = torch
        self._InputPadder = InputPadder
        self._AMP_DTYPE = AMP_DTYPE
        self._depth2xyzmap = depth2xyzmap
        self.device = device
        self.valid_iters = valid_iters

        torch.autograd.set_grad_enabled(False)
        model = torch.load(weights, map_location="cpu", weights_only=False)
        model.args.valid_iters = valid_iters
        model.args.max_disp = max_disp
        self.model = model.to(device).eval()

    def infer_disparity(self, left: np.ndarray, right: np.ndarray) -> np.ndarray:
        torch = self.torch
        l, r = to3(left), to3(right)
        H, W = l.shape[:2]
        i0 = torch.as_tensor(l).to(self.device).float()[None].permute(0, 3, 1, 2)
        i1 = torch.as_tensor(r).to(self.device).float()[None].permute(0, 3, 1, 2)
        padder = self._InputPadder(i0.shape, divis_by=32, force_square=False)
        i0, i1 = padder.pad(i0, i1)
        with torch.amp.autocast("cuda", enabled=True, dtype=self._AMP_DTYPE):
            disp = self.model.forward(i0, i1, iters=self.valid_iters,
                                      test_mode=True, optimize_build_volume="pytorch1")
        disp = padder.unpad(disp.float())
        return disp.data.cpu().numpy().reshape(H, W).clip(0, None)

    def disparity_to_cloud(self, disp, rgb, K, baseline, zfar=3.0, remove_invisible=True):
        return disp_to_cloud(self._depth2xyzmap, disp, rgb, K, baseline, zfar, remove_invisible)


class TrtStereoModel:
    """TensorRT fp16 단일 엔진 백엔드. single-ONNX는 pre-normalized 입력(ImageNet) 기대."""
    MEAN = np.array([123.675, 116.28, 103.53], np.float32)
    STD = np.array([58.395, 57.12, 57.375], np.float32)

    def __init__(self, ffs_dir: str, engine_path: str, height: int = 736, width: int = 1280):
        self.ffs_dir = os.path.realpath(ffs_dir)
        if self.ffs_dir not in sys.path:
            sys.path.insert(0, self.ffs_dir)
        import torch
        import tensorrt as trt
        from Utils import depth2xyzmap
        self.torch = torch
        self.trt = trt
        self._depth2xyzmap = depth2xyzmap
        self.h, self.w = height, width
        logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f:
            self.engine = trt.Runtime(logger).deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError(f"TRT 엔진 deserialize 실패 (TRT {trt.__version__}, 버전 불일치 가능): {engine_path}")
        self.context = self.engine.create_execution_context()
        # CPU 경량화: TRT 엔진은 GPU 단일 스트림만 쓰므로 torch CPU 스레드풀을 최소화
        # (기본은 코어수만큼 스폰 → 45개 스레드/배경 wakeup의 주범). STEREO_TORCH_THREADS로 조정.
        torch.set_num_threads(int(os.environ.get("STEREO_TORCH_THREADS", "2")))
        try:
            torch.set_num_interop_threads(1)
        except RuntimeError:
            pass  # interop 풀이 이미 초기화됨(재호출 불가) — 무시
        # blocking sync 이벤트: cudaEventBlockingSync 플래그로 GPU 완료 대기 시 CPU가
        # busy-spin(cuda-EvtHandlr가 코어를 태움) 대신 sleep/yield 하게 한다.
        self._done_event = torch.cuda.Event(blocking=True)

    def _dt(self, dt):
        trt, torch = self.trt, self.torch
        return {trt.DataType.FLOAT: torch.float32, trt.DataType.HALF: torch.float16,
                trt.DataType.BF16: torch.bfloat16, trt.DataType.INT32: torch.int32,
                trt.DataType.BOOL: torch.bool}[dt]

    def _prep(self, img):
        """HxW(IR) 또는 HxWx3 uint8 -> 정규화 NCHW cuda fp32 (1,3,H,W).

        엔진 입력(self.h,self.w)이 캡처와 다르면 **패딩**으로 맞춘다(리사이즈 왜곡 금지):
        부족분을 아래/오른쪽에 replicate. disparity는 width 패딩에만 영향받는데, 1280은
        32의 배수라 보통 width 패딩=0이고 height만 720->736 패딩된다(disparity 불변).
        엔진보다 큰 입력만 부득이 리사이즈 폴백(disparity 스케일 왜곡, 비권장)."""
        import cv2
        x = to3(img).astype(np.float32)
        H, W = x.shape[:2]
        self._src_hw = (H, W)
        if H != self.h or W != self.w:
            if H <= self.h and W <= self.w:
                x = cv2.copyMakeBorder(x, 0, self.h - H, 0, self.w - W, cv2.BORDER_REPLICATE)
            else:
                x = cv2.resize(x, (self.w, self.h), interpolation=cv2.INTER_LINEAR)
        x = (x - self.MEAN) / self.STD
        return self.torch.as_tensor(x, device="cuda").permute(2, 0, 1)[None].contiguous().float()

    def infer_disparity(self, left: np.ndarray, right: np.ndarray) -> np.ndarray:
        torch, trt = self.torch, self.trt
        inp = {"left_image": self._prep(left), "right_image": self._prep(right)}
        for name, t in inp.items():
            self.context.set_input_shape(name, tuple(t.shape))
        outs = {}
        for i in range(self.engine.num_io_tensors):
            nm = self.engine.get_tensor_name(i)
            if self.engine.get_tensor_mode(nm) == trt.TensorIOMode.OUTPUT:
                shp = tuple(self.context.get_tensor_shape(nm))
                outs[nm] = torch.empty(shp, device="cuda", dtype=self._dt(self.engine.get_tensor_dtype(nm)))
        for nm, t in {**inp, **outs}.items():
            self.context.set_tensor_address(nm, int(t.data_ptr()))
        assert self.context.execute_async_v3(torch.cuda.current_stream().cuda_stream)
        # 전체 device synchronize(스핀) 대신 blocking 이벤트 대기 → CPU가 잠들어 코어 절약
        self._done_event.record()
        self._done_event.synchronize()
        disp = next(iter(outs.values())).float().reshape(self.h, self.w).cpu().numpy()
        H, W = getattr(self, "_src_hw", (self.h, self.w))
        if (H, W) != (self.h, self.w) and H <= self.h and W <= self.w:
            disp = disp[:H, :W]   # 패딩 영역 제거 -> 캡처 해상도로 복원
        return disp.clip(0, None)

    def disparity_to_cloud(self, disp, rgb, K, baseline, zfar=3.0, remove_invisible=True):
        return disp_to_cloud(self._depth2xyzmap, disp, rgb, K, baseline, zfar, remove_invisible)
