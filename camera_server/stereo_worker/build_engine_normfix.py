#!/usr/bin/env python3
"""Path A: fp16 엔진 + cost-volume normalize 체인만 fp32 고정.

fp16 NaN 원인은 volume normalize(F.normalize)의 eps(1e-12)가 fp16에서 언더플로→0÷0.
그래서 FP16 전역 + OBEY_PRECISION_CONSTRAINTS 로 두고, normalize 체인
(ReduceL2 -> Clip(eps) -> Expand -> Div [-> Cast])만 fp32로 못박는다.
무거운 ViT-L 백본/Conv/MatMul은 fp16 유지 → 속도 확보.
"""
from __future__ import annotations
import argparse, time
import onnx
import tensorrt as trt

# normalize 체인에서 fp32로 유지할 op (Conv/MatMul 만나면 중단)
CHAIN_OPS = {"Clip", "Expand", "Cast", "Div", "Reciprocal", "Mul", "Sqrt", "Pow", "Sub", "Max"}


def normalize_pin_names(onnx_path):
    """각 ReduceL2에서 시작해 normalize 나눗셈 체인 노드 이름을 수집."""
    g = onnx.load(onnx_path).graph
    cons = {}
    for n in g.node:
        for i in n.input:
            cons.setdefault(i, []).append(n)
    pin = set()
    for n in g.node:
        if n.op_type != "ReduceL2":
            continue
        pin.add(n.name)
        frontier, seen, hops = list(n.output), set(), 0
        while frontier and hops < 10:
            hops += 1
            nxt = []
            for t in frontier:
                for c in cons.get(t, []):
                    if c.name in seen or c.op_type not in CHAIN_OPS:
                        continue
                    pin.add(c.name); seen.add(c.name); nxt.extend(c.output)
            frontier = nxt
    return pin


def build(onnx_path, engine_path, workspace_gb=8):
    logger = trt.Logger(trt.Logger.WARNING)
    builder = trt.Builder(logger)
    flags = 0
    if hasattr(trt.NetworkDefinitionCreationFlag, "EXPLICIT_BATCH"):
        flags = 1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    network = builder.create_network(flags)
    parser = trt.OnnxParser(network, logger)
    with open(onnx_path, "rb") as f:
        if not parser.parse(f.read()):
            for i in range(parser.num_errors):
                print("[onnx]", parser.get_error(i))
            raise RuntimeError("ONNX parse 실패")

    pin = normalize_pin_names(onnx_path)
    print(f"[normfix] normalize 체인 pin 후보 노드 {len(pin)}개")

    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, workspace_gb << 30)
    config.set_flag(trt.BuilderFlag.FP16)
    config.set_flag(trt.BuilderFlag.OBEY_PRECISION_CONSTRAINTS)

    # shape/index 계산 레이어는 fp32 지정 불가(Expand-as-Slice 등) → 산술 레이어만.
    SKIP = {trt.LayerType.SLICE, trt.LayerType.SHUFFLE, trt.LayerType.GATHER,
            trt.LayerType.CONSTANT, trt.LayerType.CONCATENATION, trt.LayerType.SHAPE,
            trt.LayerType.IDENTITY, trt.LayerType.CAST}
    pinned, skipped = 0, 0
    for i in range(network.num_layers):
        layer = network.get_layer(i)
        if layer.name not in pin:      # 정확 매칭만(substring 과다고정 방지)
            continue
        if layer.type in SKIP:
            skipped += 1
            continue
        try:
            layer.precision = trt.float32
            for j in range(layer.num_outputs):
                if layer.get_output(j).is_execution_tensor:
                    layer.set_output_type(j, trt.float32)
            pinned += 1
        except Exception as e:  # noqa: BLE001
            skipped += 1
    print(f"[normfix] fp32 고정 {pinned}개 (skip shape/index {skipped}) / 전체 {network.num_layers}")

    print(f"[normfix] TRT {trt.__version__} building (FP16+OBEY, normalize→fp32)...", flush=True)
    t0 = time.time()
    ser = builder.build_serialized_network(network, config)
    if ser is None:
        raise RuntimeError("엔진 빌드 실패")
    with open(engine_path, "wb") as f:
        f.write(ser)
    import os
    print(f"[normfix] 저장 {engine_path} ({os.path.getsize(engine_path)/1e6:.0f}MB, {time.time()-t0:.0f}s)", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--onnx", required=True)
    ap.add_argument("--engine", required=True)
    ap.add_argument("--workspace-gb", type=int, default=8)
    a = ap.parse_args()
    build(a.onnx, a.engine, a.workspace_gb)


if __name__ == "__main__":
    main()
