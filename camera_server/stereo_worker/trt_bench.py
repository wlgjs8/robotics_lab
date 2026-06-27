#!/usr/bin/env python3
"""TRT 엔진 추론 시간 측정 (480x640 입력). run_demo_single_trt의 러너를 복제."""
from __future__ import annotations
import argparse, time
import numpy as np
import torch
import tensorrt as trt


class SingleEngineTrtRunner:
    def __init__(self, engine_path):
        self.logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f:
            self.engine = trt.Runtime(self.logger).deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError(f"엔진 deserialize 실패 (TRT {trt.__version__})")
        self.context = self.engine.create_execution_context()

    @staticmethod
    def _dt(dt):
        return {trt.DataType.FLOAT: torch.float32, trt.DataType.HALF: torch.float16,
                trt.DataType.BF16: torch.bfloat16, trt.DataType.INT32: torch.int32,
                trt.DataType.BOOL: torch.bool}[dt]

    def __call__(self, inputs: dict) -> dict:
        for name, t in inputs.items():
            self.context.set_input_shape(name, tuple(t.shape))
        outs = {}
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.OUTPUT:
                shape = tuple(self.context.get_tensor_shape(name))
                outs[name] = torch.empty(shape, device="cuda", dtype=self._dt(self.engine.get_tensor_dtype(name)))
        for name, t in {**inputs, **outs}.items():
            self.context.set_tensor_address(name, int(t.data_ptr()))
        assert self.context.execute_async_v3(torch.cuda.current_stream().cuda_stream)
        return outs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--engine", required=True)
    ap.add_argument("--h", type=int, default=480)
    ap.add_argument("--w", type=int, default=640)
    ap.add_argument("--warmup", type=int, default=15)
    ap.add_argument("--iters", type=int, default=60)
    args = ap.parse_args()

    r = SingleEngineTrtRunner(args.engine)
    g = torch.Generator(device="cuda").manual_seed(0)
    left = torch.rand((1, 3, args.h, args.w), device="cuda", generator=g)
    right = torch.rand((1, 3, args.h, args.w), device="cuda", generator=g)
    inp = {"left_image": left, "right_image": right}

    for _ in range(args.warmup):
        r(inp); torch.cuda.synchronize()
    ts = []
    for _ in range(args.iters):
        torch.cuda.synchronize(); t0 = time.perf_counter()
        r(inp); torch.cuda.synchronize()
        ts.append((time.perf_counter() - t0) * 1000)
    ts = np.array(ts)
    print(f"[bench] TRT {trt.__version__}  {args.w}x{args.h} fp16  "
          f"avg={ts.mean():.1f}ms  p50={np.percentile(ts,50):.1f}  p95={np.percentile(ts,95):.1f}  "
          f"min={ts.min():.1f}  ->  {1000/ts.mean():.0f} fps", flush=True)


if __name__ == "__main__":
    main()
