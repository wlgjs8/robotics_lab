#!/usr/bin/env python3
"""fp16 NaN 위치특정: PyTorch(fp32) forward에서 모듈별 활성값 max|.|를 측정해
fp16 표현한계(max 65504)를 넘거나 근접하는 모듈을 찾는다. cost-volume/집계가 유력.

hook은 상위 모듈 + cost_agg(hourglass) 내부 자식까지. 입력/출력 모두 측정
(gwc_volume은 인라인 텐서라 corr_stem의 '입력'으로 잡힌다).
"""
import os, sys
sys.path.insert(0, "/app/stereo_worker")
import numpy as np
import torch

FFS = os.environ.get("FFS_DIR", "/app/Fast-FoundationStereo")
WEIGHTS = f"{FFS}/weights/23-36-37/model_best_bp2_serialize.pth"
FP16_MAX = 65504.0


def get_ir_pair():
    try:
        from bundle_reader import BundleReader
        r = BundleReader(endpoint="tcp://127.0.0.1:5600")
        for _ in range(100):
            fr = r.poll({"head.ir_left", "head.ir_right"}, 200)
            if len(fr) == 2:
                print("[loc] source=live head IR pair")
                return fr["head.ir_left"].pixels, fr["head.ir_right"].pixels
    except Exception as e:
        print(f"[loc] live 실패({e}) -> demo")
    import imageio.v2 as imageio
    return (imageio.imread(f"{FFS}/demo_data/left.png"),
            imageio.imread(f"{FFS}/demo_data/right.png"))


def tmax(x):
    """텐서/튜플/리스트에서 max|.| (유한값만). 없으면 None."""
    best = None
    stack = [x]
    while stack:
        v = stack.pop()
        if isinstance(v, torch.Tensor) and v.is_floating_point():
            f = v.detach().float()
            f = f[torch.isfinite(f)]
            if f.numel():
                m = float(f.abs().max())
                best = m if best is None else max(best, m)
        elif isinstance(v, (list, tuple)):
            stack.extend(v)
    return best


def main():
    irl, irr = get_ir_pair()
    from stereo_model import FoundationStereoModel
    fm = FoundationStereoModel(FFS, WEIGHTS)
    model = fm.model

    print("\n[loc] 모델 상위 모듈:", ", ".join(n for n, _ in model.named_children()))

    # hook 대상: 상위 자식 전부 + cost_agg 내부 자식(hourglass 3D)
    targets = {}
    for name, mod in model.named_children():
        targets[name] = mod
    if hasattr(model, "cost_agg"):
        for cn, cm in model.cost_agg.named_children():
            targets[f"cost_agg.{cn}"] = cm

    rec = {}   # name -> [in_max, out_max]

    def mk(name):
        def hook(m, inp, out):
            im, om = tmax(inp), tmax(out)
            r = rec.setdefault(name, [0.0, 0.0])
            if im is not None:
                r[0] = max(r[0], im)
            if om is not None:
                r[1] = max(r[1], om)
        return hook

    handles = [mod.register_forward_hook(mk(name)) for name, mod in targets.items()]

    with torch.no_grad():
        _ = fm.infer_disparity(irl, irr)

    for h in handles:
        h.remove()

    print(f"\n[loc] 모듈별 max|activation|  (fp16 한계={FP16_MAX:.0f})")
    print(f"{'module':<28}{'in_max':>14}{'out_max':>14}  fp16?")
    for name in sorted(rec, key=lambda k: -max(rec[k])):
        im, om = rec[name]
        risk = "OVERFLOW" if max(im, om) > FP16_MAX else ("RISK" if max(im, om) > FP16_MAX * 0.3 else "")
        print(f"{name:<28}{im:>14.1f}{om:>14.1f}  {risk}")


if __name__ == "__main__":
    main()
