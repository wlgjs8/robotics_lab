"""Fast-FoundationStereo 래퍼: (left,right) -> disparity -> depth -> colored pointcloud.

rb_gui와 분리된 GPU 워커에서 사용. torch는 이 프로세스(컨테이너)에만 필요.
"""
from __future__ import annotations
import os
import sys
import numpy as np


class FoundationStereoModel:
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

    @staticmethod
    def _to3(img: np.ndarray) -> np.ndarray:
        """IR/grayscale(HxW) -> HxWx3, RGBA -> RGB."""
        if img.ndim == 2:
            img = np.tile(img[..., None], (1, 1, 3))
        return np.ascontiguousarray(img[..., :3])

    def infer_disparity(self, left: np.ndarray, right: np.ndarray) -> np.ndarray:
        """left/right: rectified pair (HxW IR 또는 HxWx3). returns disparity (H,W) px."""
        torch = self.torch
        l, r = self._to3(left), self._to3(right)
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

    def disparity_to_cloud(self, disp: np.ndarray, rgb: np.ndarray, K: np.ndarray,
                           baseline: float, zfar: float = 3.0,
                           remove_invisible: bool = True):
        """returns (xyz Nx3 [m, camera frame], colors Nx3 uint8)."""
        disp = disp.astype(np.float32).copy()
        if remove_invisible:
            yy, xx = np.meshgrid(np.arange(disp.shape[0]), np.arange(disp.shape[1]), indexing="ij")
            disp[(xx - disp) < 0] = np.inf
        with np.errstate(divide="ignore", invalid="ignore"):
            depth = K[0, 0] * baseline / disp
        xyz = self._depth2xyzmap(depth, K).reshape(-1, 3)
        col = (self._to3(rgb)).reshape(-1, 3).astype(np.uint8)
        m = (xyz[:, 2] > 0) & (xyz[:, 2] <= zfar) & np.isfinite(xyz).all(1)
        return xyz[m], col[m]
