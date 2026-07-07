#!/usr/bin/env python3
"""fp16 엔진 NaN 재현·확인 하네스. tf32 엔진은 건드리지 않고 별도 *_fp16.engine을 비교한다.

- 라이브 IR 페어(camera bundle tcp 5600)가 있으면 그걸, 없으면 FFS demo 페어로.
- fp16 disparity의 NaN/Inf 비율·범위를 보고, PyTorch(fp32) 및 tf32 엔진과 대조.
- 결론: fp16이 실제로 깨지는지(=cost-volume 오버플로우 재현), 어느 정도로 깨지는지.

사용: docker exec camera_server python3 /app/stereo_worker/diag_fp16.py
"""
import os, sys
sys.path.insert(0, "/app/stereo_worker")
import numpy as np

FFS = os.environ.get("FFS_DIR", "/app/Fast-FoundationStereo")
ENG_FP16 = "/app/stereo_worker/engines/fast_foundationstereo_fp16.engine"
ENG_TF32 = "/app/stereo_worker/engines/fast_foundationstereo.engine"
WEIGHTS = f"{FFS}/weights/23-36-37/model_best_bp2_serialize.pth"


def get_ir_pair():
    """라이브 IR 페어 우선, 실패 시 demo png."""
    try:
        from bundle_reader import BundleReader
        r = BundleReader(endpoint="tcp://127.0.0.1:5600")
        for _ in range(100):
            fr = r.poll({"head.ir_left", "head.ir_right"}, 200)
            if len(fr) == 2:
                print("[diag] source=live head IR pair")
                return fr["head.ir_left"].pixels, fr["head.ir_right"].pixels
    except Exception as e:
        print(f"[diag] live bundle 실패({e}) -> demo 페어")
    import imageio.v2 as imageio
    left = imageio.imread(f"{FFS}/demo_data/left.png")
    right = imageio.imread(f"{FFS}/demo_data/right.png")
    print("[diag] source=FFS demo pair")
    return left, right


def stats(name, d):
    d = np.asarray(d, np.float32)
    nan = float(np.mean(np.isnan(d)) * 100)
    inf = float(np.mean(np.isinf(d)) * 100)
    fin = d[np.isfinite(d)]
    fmin = float(fin.min()) if fin.size else float("nan")
    fmax = float(fin.max()) if fin.size else float("nan")
    print(f"[{name}] shape={d.shape} NaN={nan:.2f}% Inf={inf:.2f}% "
          f"finite[min={fmin:.2f} max={fmax:.2f} mean={ (fin.mean() if fin.size else float('nan')):.2f}]")
    return dict(nan=nan, inf=inf, fmin=fmin, fmax=fmax, d=d)


def main():
    irl, irr = get_ir_pair()
    from stereo_model import TrtStereoModel, FoundationStereoModel

    print("\n=== fp16 engine ===")
    if not os.path.exists(ENG_FP16):
        print(f"[diag] FATAL: fp16 엔진 없음 {ENG_FP16} (빌드 먼저)")
        return
    m16 = TrtStereoModel(FFS, ENG_FP16)
    s16 = stats("fp16", m16.infer_disparity(irl, irr))

    print("\n=== tf32 engine (기준선) ===")
    m32 = TrtStereoModel(FFS, ENG_TF32)
    s32 = stats("tf32", m32.infer_disparity(irl, irr))

    print("\n=== PyTorch fp32 (정답) ===")
    mth = FoundationStereoModel(FFS, WEIGHTS)
    sth = stats("torch", mth.infer_disparity(irl, irr))

    print("\n=== 판정 ===")
    broke = s16["nan"] > 0.01 or s16["inf"] > 0.01 or not np.isfinite(s16["fmax"])
    if broke:
        print(f">>> fp16 NaN/Inf 재현됨 (NaN={s16['nan']:.2f}% Inf={s16['inf']:.2f}%) "
              f"— cost-volume 오버플로우 가설과 일치")
    else:
        diff = np.abs(s16["d"] - sth["d"])
        fin = np.isfinite(diff)
        print(f">>> fp16이 NaN 없이 완주. |fp16-torch| mean={diff[fin].mean():.3f}px "
              f"p95={np.percentile(diff[fin],95):.3f}px max={diff[fin].max():.2f}px")
        print("    (TRT 10.16/5090에서 과거와 달리 안 깨질 수 있음 — diff가 작으면 fp16 채택 후보)")


if __name__ == "__main__":
    main()
