# Current Review

This document mirrors the root `REVIEW.md` baseline for agents that only inspect `docs/`.

## Current Milestone

Simulator-first Cartesian acceptance hardening.

Before real robot work, repeatedly validate in simulation:

- per-arm simulator topology
- structured backend result contract
- joint commands
- `TcpPoseTarget`
- `TcpLinearMove`
- `TcpTwistLocal`
- `TcpTwistStand`
- GUI controls
- policy_runner SpaceMouse path
- command-source lease
- camera readiness contracts

## Current Status

The code now has distinct motion primitive semantics:

- `TcpPoseTarget`: point-to-point final pose
- `TcpLinearMove`: simulator-only MoveL-like path primitive
- `TcpTwistLocal` / `TcpTwistStand`: streaming Cartesian velocity
- `TcpDeltaLocal` / `TcpDeltaStand`: low-level one-shot/debug jog

Real robot and real Cartesian motion remain blocked.

## Main Open Items

1. C++ hardware-free gate must pass on the target development machine.
2. Pinocchio-enabled C++ gate must pass.
3. Full Cartesian simulator acceptance must pass repeatedly.
4. `TcpLinearMove path_done` telemetry should be robust across state publish rates.
5. Constant-orientation mismatch semantics should be explicit and tested.
6. Real rbpodo read-only acceptance is still separate.
7. Real motion is still blocked.
8. Measured calibration is still absent.

## Review Checklist

A change is suspicious if it:

- enables real motion
- weakens real env gates
- weakens command-source lease or deadman behavior
- mixes PTP/Linear/Twist/Delta semantics
- removes structured backend result fields
- hides fault context
- changes Cartesian behavior without updating acceptance
