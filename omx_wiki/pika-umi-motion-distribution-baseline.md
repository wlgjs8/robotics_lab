---
title: "PIKA UMI Motion Distribution Baseline"
tags: ["pika-umi", "training-data", "motion-distribution", "velocity-proprio", "flow-infer"]
created: 2026-07-12T08:29:21.194Z
updated: 2026-07-12T08:29:21.194Z
sources: []
links: ["flow-infer-delta-preview-controller-contract.md"]
category: reference
confidence: medium
schemaVersion: 1
---

# PIKA UMI Motion Distribution Baseline

Prior read-only analysis of the depth_z50 training conversion covered 527 episodes / 151871 frames at 30 Hz. The dominant moving arm had a median translation delta of about 5.2 mm per frame, while the inactive arm in the sequential left/right collection phases had a median tracker/noise delta of about 0.65 mm per frame. A representative pre-change rollout emitted about 1.5-1.9 mm policy deltas but the delta_twist controller saturated about 98-99 percent and reached its 20 mm command-lead cap. Warm OpenPI inference latency was about 0.34-0.38 s, or roughly 10-11 policy rows, while the old runtime replayed row 0. These measurements motivate [[flow-infer-delta-preview-controller-contract]]. They were not recomputed on 2026-07-12; rerun the storage-side analysis and record an immutable artifact before using them as formal acceptance thresholds.
