# Calibration Registry

This directory is the global setup source of truth for robot, camera, and stand
frame relationships.

The active file is [active_calibration.yaml](active_calibration.yaml). It is a
configured estimate, not a measured calibration artifact:

- `status: configured_estimate`
- `geometry_valid_for_real_policy: false`

Configured estimates may be used for visualization, simulation, and frame
contract development. They are not valid for real geometry-dependent policy,
real Cartesian/TCP policy, or camera-to-robot policy decisions.

Joint-only action sources do not require this file to be measured. Missing or
unmeasured camera intrinsics, camera extrinsics, or wrist hand-eye calibration
must not block joint-only control.

Measured and accepted calibration is a later milestone. Future calibration
artifacts should replace or supersede this file with measured stand mounts,
camera intrinsics, head-camera extrinsics, and wrist-camera hand-eye transforms.

Policy and teleop dataset metadata should record the active calibration status.
For the current file, use `calibration_status: configured_estimate` and keep
`geometry_valid_for_real_policy: false` visible in dataset review. Do not label
controller-simulation or future physical real policy data as measured until a
measured calibration artifact is accepted.

`umi_retarget.example.yaml` is the template for offline UMI-to-stand retarget
metadata. Its default `status: configured_estimate` is suitable for synthetic
fixtures and training pipeline smoke tests only. Real policy rollout must remain
blocked until a site-specific retarget config has `status: measured` or
`status: accepted`. `accepted` means a measured transform was accepted by a
documented acceptance artifact.
