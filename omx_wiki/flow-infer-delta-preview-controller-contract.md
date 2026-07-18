---
title: "Flow Infer Delta Preview Controller Contract"
tags: ["flow-infer", "vla", "velocity-proprio", "delta-preview", "ruckig", "safety"]
created: 2026-07-12T08:28:05.396Z
updated: 2026-07-12T08:28:05.396Z
sources: []
links: []
category: decision
confidence: medium
schemaVersion: 1
---

# Flow Infer Delta Preview Controller Contract

The flow-infer chunk path uses robotics_lab.chunk_overlay.v3. The publisher records a global emitted policy-step sequence, drops warm-inference rows already emitted since the camera observation, and keeps row 0 on cold start. Both arms and grippers stay on the same source row. Camera-frame velocity proprio is a raw measured ee-local body delta over [camera_time-policy_dt, camera_time]; both arm pose brackets must be valid. The server delta_preview controller rejects missing/inconsistent v3 metadata or invalid proprio, integrates ee-local deltas through canonical Eigen/Pinocchio SE(3), and previews absolute knots through the existing Ruckig p/v/a chain. Projection error and command-to-measured actual lead have mandatory positive config bounds and consecutive-error fault budgets; fallback_policy must be fault. The tracked real profile selects delta_preview, but only hardware-free tests have run. Mock/controller-simulation replay and supervised physical speed/blur acceptance remain required before treating the thresholds as accepted real-hardware values.
