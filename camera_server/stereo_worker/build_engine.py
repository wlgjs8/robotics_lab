#!/usr/bin/env python3
"""ONNX -> TensorRT fp16 엔진 빌드 (tensorrt Python API, trtexec 불필요).

사용:
  python3 build_engine.py --onnx output/fast_foundationstereo.onnx \
                          --engine output/fast_foundationstereo.engine --fp16
엔진은 GPU/TRT 버전 종속이므로 배포 GPU(RTX 5090)에서 빌드한다.
"""
from __future__ import annotations
import argparse
import time


def build(onnx_path, engine_path, precision="bf16", workspace_gb=12):
    import tensorrt as trt
    logger = trt.Logger(trt.Logger.INFO)
    builder = trt.Builder(logger)
    # TRT 10/11은 explicit batch가 기본(EXPLICIT_BATCH 플래그 제거됨). 구버전 호환만 유지.
    flags = 0
    if hasattr(trt, "NetworkDefinitionCreationFlag") and \
       hasattr(trt.NetworkDefinitionCreationFlag, "EXPLICIT_BATCH"):
        flags = 1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    network = builder.create_network(flags)
    parser = trt.OnnxParser(network, logger)
    with open(onnx_path, "rb") as f:
        if not parser.parse(f.read()):
            for i in range(parser.num_errors):
                print("[onnx-parse]", parser.get_error(i))
            raise RuntimeError("ONNX parse 실패")
    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, workspace_gb << 30)
    # fp16은 stereo cost-volume에서 오버플로우->NaN. bf16(지수 fp32급) 권장.
    avail = [f for f in ("FP16", "BF16", "TF32") if hasattr(trt.BuilderFlag, f)]
    print(f"[build] available precision flags: {avail}")
    for mode in (["bf16", "fp16", "tf32"] if precision == "auto" else [precision]):
        flag = {"fp16": "FP16", "bf16": "BF16", "tf32": "TF32"}.get(mode)
        if flag and hasattr(trt.BuilderFlag, flag):
            config.set_flag(getattr(trt.BuilderFlag, flag))
            print(f"[build] {flag} enabled")
    print(f"[build] TRT {trt.__version__}  parsing OK, building engine...", flush=True)
    t0 = time.time()
    serialized = builder.build_serialized_network(network, config)
    if serialized is None:
        raise RuntimeError("엔진 빌드 실패 (build_serialized_network -> None)")
    with open(engine_path, "wb") as f:
        f.write(serialized)
    import os
    print(f"[build] 엔진 저장 {engine_path}  "
          f"({os.path.getsize(engine_path)/1e6:.0f}MB, {time.time()-t0:.0f}s)", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--onnx", required=True)
    ap.add_argument("--engine", required=True)
    # tf32 기본: fp16은 이 모델 cost-volume에서 NaN, bf16은 deconv tactic 부재로 빌드 실패.
    # tf32 = 정확도 fp32급 + 텐서코어 가속(~21ms@640x480, RTX5090).
    ap.add_argument("--precision", choices=["bf16", "fp16", "tf32", "fp32", "auto"], default="tf32")
    ap.add_argument("--fp16", dest="precision", action="store_const", const="fp16")
    ap.add_argument("--fp32", dest="precision", action="store_const", const="fp32")
    ap.add_argument("--workspace-gb", type=int, default=12)
    args = ap.parse_args()
    build(args.onnx, args.engine, precision=args.precision, workspace_gb=args.workspace_gb)


if __name__ == "__main__":
    main()
