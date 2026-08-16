# Archived: head-stereo perception (Fast-FoundationStereo + box detect)

Removed from the codebase on **2026-08-16** by operator decision. This is a
record of what existed and what was given up, not runnable guidance. Everything
described here is recoverable from git history.

## What it was

`submodules/Fast-FoundationStereo` (FFS) was a learned stereo-matching network.
`camera_server/stereo_worker` fed it the D435 head's IR-left/IR-right pair and
turned the resulting disparity into:

```
head IR pair ──► FFS disparity ──┬─► head point cloud   ──► ZMQ stereo.cloud  ──► rb_gui /stereo_cam
   (D435)        (.pth or TRT)   └─► box_detect (ICP)   ──┬─► ZMQ stereo.boxes ──► rb_gui overlay + 재탐지 버튼
                                                          └─► external_boxes_sender
                                                              ──► UDP 50256 SetExternalBoxes
                                                              ──► rb_servo_server CollisionMonitor keep-out
```

It also fused wrist D405 clouds into the head detection when the arms were at
rest (`state_listener` + `T_tcp_cam` hand-eye), and clipped published clouds to
the rb_gui Safety ROI.

## Why it was removed

The stack is being simplified so the remaining code is unambiguous. FFS was a
large, GPU-bound dependency (320 MB submodule, ~820 MB of TRT engines, a CUDA
12.8 base image with torch + TensorRT) serving a capability the deployed policy
does not use: `policy_runner`'s pi0.5 rollout is wrist-camera-only
(`camera.bundle.policy`), and `cm_bridge` has no perception input at all. Its
only live consumers were an rb_gui visualization panel and the automatic
keep-out feed below.

## What was given up (read this before assuming nothing changed)

**Automatic keep-out boxes are gone** — but they were already switched off on
real hardware well before this. `stack_real.yaml` set
`external_boxes.enable: false` on 2026-07-10 because the box keep-out barrier's
deceleration corrupted the policy's observed velocity; **F/T-sensor contact
detection replaced it**. Only `stack_sim.yaml` still had it enabled.

Removing the producer here left the server-side `SetExternalBoxes` path with no
feed at all, so that whole feature was deleted too (see below). The server's own
URDF-mesh self-collision guard, floor plane, tracking-error latch, and
lease/deadman are unaffected, and remain the real-motion safety layers.

Also gone: the head 3D point cloud in the viser scene, the `🎯 박스 재탐지`
button and box lock telemetry, head/wrist cloud fusion, and Safety-ROI clipping
of published clouds.

## What survives

- **Wrist D405 clouds** (`stereo.wrist` on `tcp://127.0.0.1:5601`) — these never
  used a model; `wrist_cloud()` deprojects the D405's own hardware depth. This
  is now the whole job of `camera_server/stereo_worker/worker.py`.
- **The head D435 color stream** and the rb_gui `카메라 품질` head-view panel
  (`camera.bundle.stereo` / `head.color`). Only the IR pair was removed from the
  head rigs.
- **Wrist RGB-D policy bundles** (`camera.bundle.policy`, `wrist_left`,
  `wrist_right`) — the real-policy input path, untouched.

## Removed artifacts

Submodule `submodules/Fast-FoundationStereo`; in `camera_server/stereo_worker/`:
`stereo_model.py`, `box_detect.py`, `box_trigger_listener.py`,
`external_boxes_sender.py`, `state_listener.py`, `engines/`, `out/`,
`rebuild_engine_1280.sh`, `build_engine*.py`, `trt_bench.py`, `diag_*.py`,
`localize_fp16.py`, `d435_ir_1280x720_K.txt`; `camera_server/config/`
`d435_ir_640x480_K.txt` and `d435_color_calib.json`; the `tools/box_*.py`
probes; the `camera_server/tests/test_box_*.py` and
`test_external_boxes_sender.py` suites; the entire server-side `external_box`
feature (the `SetExternalBoxes` ControlMode, its command parser and buffer side
slot, `CollisionMonitor`'s box geometry/barrier/telemetry, the feed-liveness
abort watchdog, the `external_boxes` config block, 3 `servo_state.v1` fields and
9 CSV log columns, and `tools/sim_box_barrier.cpp`); `make cam-engine-rebuild`; the
`dump_intrinsics_path` camera control (its only consumer was the FFS K.txt); and
the CUDA/torch/TensorRT layers of `camera_server/Dockerfile`, which is now a
plain `ubuntu:22.04` image.
