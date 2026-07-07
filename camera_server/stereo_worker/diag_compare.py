#!/usr/bin/env python3
"""tf32 / fp16(orig) / A:normfix / B:epsfix 4개 엔진을 PyTorch(fp32) 기준으로 일괄 비교.
지표: NaN%, infer 지연(ms, 10회 평균), |disparity - PyTorch| (finite 영역 mean/p95/max).
GPU는 serve_policy와 공유 중이라 지연은 상대비교용(절대값 부풀 수 있음)."""
import os, sys, time, gc
sys.path.insert(0, "/app/stereo_worker")
import numpy as np
import torch
from bundle_reader import BundleReader
from stereo_model import TrtStereoModel, FoundationStereoModel

FFS = "/app/Fast-FoundationStereo"
WEIGHTS = f"{FFS}/weights/23-36-37/model_best_bp2_serialize.pth"
E = "/app/stereo_worker/engines"
ENGINES = {
    "tf32(base)": f"{E}/fast_foundationstereo.engine",
    "fp16(orig)": f"{E}/fast_foundationstereo_fp16.engine",
    "A:normfix":  f"{E}/fast_foundationstereo_fp16_normfix.engine",
    "B:epsfix":   f"{E}/fast_foundationstereo_fp16_epsfix.engine",
}


def get_pair():
    r = BundleReader(endpoint="tcp://127.0.0.1:5600")
    for _ in range(100):
        fr = r.poll({"head.ir_left", "head.ir_right"}, 200)
        if len(fr) == 2:
            return fr["head.ir_left"].pixels, fr["head.ir_right"].pixels
    import imageio.v2 as imageio
    return imageio.imread(f"{FFS}/demo_data/left.png"), imageio.imread(f"{FFS}/demo_data/right.png")


def bench(m, irl, irr, n=10):
    m.infer_disparity(irl, irr)  # warmup
    t = time.perf_counter()
    for _ in range(n):
        d = m.infer_disparity(irl, irr)
    return d, (time.perf_counter() - t) / n * 1000.0


def main():
    irl, irr = get_pair()
    th = FoundationStereoModel(FFS, WEIGHTS)
    dref = th.infer_disparity(irl, irr)
    del th; gc.collect(); torch.cuda.empty_cache()

    print(f"\n{'engine':<12}{'NaN%':>7}{'lat_ms':>9}{'|dpx|mean':>11}{'p95':>8}{'max':>8}   판정")
    print("-" * 66)
    for name, path in ENGINES.items():
        if not os.path.exists(path):
            print(f"{name:<12}  (엔진 없음)"); continue
        try:
            m = TrtStereoModel(FFS, path)
            d, lat = bench(m, irl, irr)
            nan = float(np.isnan(d).mean() * 100)
            diff = np.abs(d - dref); fin = np.isfinite(diff)
            dm = float(diff[fin].mean()); p95 = float(np.percentile(diff[fin], 95)); mx = float(diff[fin].max())
            verdict = "BROKEN" if nan > 0.1 else ("OK" if p95 < 1.0 else "정확도의심")
            print(f"{name:<12}{nan:>7.1f}{lat:>9.1f}{dm:>11.3f}{p95:>8.3f}{mx:>8.2f}   {verdict}")
            del m; gc.collect(); torch.cuda.empty_cache()
        except Exception as e:  # noqa: BLE001
            print(f"{name:<12}  ERROR {repr(e)[:60]}")


if __name__ == "__main__":
    main()
