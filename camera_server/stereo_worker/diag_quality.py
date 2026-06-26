#!/usr/bin/env python3
"""현재 IR 프레임에서 TF32 엔진 vs PyTorch(fp32) disparity 비교 + IR 밝기/텍스처 점검.
차이가 작으면 TF32 무관(조명/텍스처 문제), 크면 TF32 영향."""
import os, sys
sys.path.insert(0, "/app/stereo_worker")
import numpy as np, cv2
from bundle_reader import BundleReader
from stereo_model import TrtStereoModel, FoundationStereoModel

FFS = "/app/Fast-FoundationStereo"
ENGINE = "/app/stereo_worker/engines/fast_foundationstereo.engine"
WEIGHTS = f"{FFS}/weights/23-36-37/model_best_bp2_serialize.pth"

r = BundleReader(endpoint="tcp://127.0.0.1:5600")
fr = {}
for _ in range(80):
    fr = r.poll({"head.ir_left", "head.ir_right"}, 200)
    if len(fr) == 2:
        break
irl, irr = fr["head.ir_left"].pixels, fr["head.ir_right"].pixels
print(f"IR-left  brightness mean={irl.mean():.1f} std(texture)={irl.std():.1f} "
      f"min={int(irl.min())} max={int(irl.max())} sat>250={np.mean(irl>250)*100:.1f}% dark<20={np.mean(irl<20)*100:.1f}%")

trt = TrtStereoModel(FFS, ENGINE)
d_trt = trt.infer_disparity(irl, irr)
th = FoundationStereoModel(FFS, WEIGHTS)
d_th = th.infer_disparity(irl, irr)

diff = np.abs(d_trt - d_th)
print(f"PyTorch disp: min={d_th.min():.2f} max={d_th.max():.2f} mean={d_th.mean():.2f}")
print(f"TF32    disp: min={d_trt.min():.2f} max={d_trt.max():.2f} mean={d_trt.mean():.2f}")
print(f"|TF32-PyTorch|: mean={diff.mean():.3f}px  p95={np.percentile(diff,95):.3f}px  max={diff.max():.2f}px")
print(f"  within 0.5px={np.mean(diff<0.5)*100:.1f}%  within 1px={np.mean(diff<1)*100:.1f}%")

def cm(d):
    v = np.clip(d, 0, max(d_th.max(), 1))
    return cv2.applyColorMap(cv2.convertScaleAbs(v, alpha=255.0/max(v.max(), 1e-6)), cv2.COLORMAP_TURBO)

irl3 = cv2.cvtColor(irl, cv2.COLOR_GRAY2BGR)
irr3 = cv2.cvtColor(irr, cv2.COLOR_GRAY2BGR)
diffv = cv2.applyColorMap(cv2.convertScaleAbs(diff, alpha=255.0/max(diff.max(), 1e-6)), cv2.COLORMAP_HOT)
row1 = np.concatenate([irl3, irr3], axis=1)
row2 = np.concatenate([cm(d_th), cm(d_trt)], axis=1)
row3 = np.concatenate([diffv, np.zeros_like(diffv)], axis=1)
mont = np.concatenate([row1, row2, row3], axis=0)
for txt, y in [("IR-L | IR-R", 30), ("PyTorch disp | TF32 disp", 30+480), ("|diff| (hot)", 30+960)]:
    cv2.putText(mont, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
os.makedirs("/app/stereo_worker/out", exist_ok=True)
cv2.imwrite("/app/stereo_worker/out/quality_diag.png", mont)
print("saved /app/stereo_worker/out/quality_diag.png")
