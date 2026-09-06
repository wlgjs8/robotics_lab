# Servo coordinator command proprio

`--velproprio-source servo_command --velproprio-sample-mode fixed_step` is an
explicit OpenPI velocity-proprio source. Existing `measured` and `command`
sources retain their behavior and remain available for comparison.

## Signal and training contract

The signal is `left/right.tcp_command_stand`, the server's FK of its
coordinator **attempted q_sent target** after target generation and safety.
It is neither proof of a backend send/ACK nor the controller's executed
reference or measured encoder pose. Worker interpolation, pending replacement,
network delivery and physical tracking may produce different signals.

The collector reads actual state packets; it never synthesizes servo motion
from the policy runner's virtual `TcpPoseTarget` integration. A missing
coordinator pose does not fall back to measured TCP or Python command pose.

Model feature dimensions and units are unchanged. `velocity` remains 12 values,
`velocity_grip` 14, and `velocity_grav` 20. The two endpoint poses produce an
incoming TCP-body translation and relative rotation vector over one policy
period, using SciPy Rotation/Slerp. Despite the historical name “velocity”, the
training feature is a per-frame delta, **not meters/second or radians/second**.
No inference-duration division or additional scaling is applied. Existing
`ee_local_r_align` and gripper-source selection still apply. In velocity_grav,
gravity uses the sampled coordinator orientation at the window endpoint.
For velocity_grip/velocity_grav, the selected gripper percent and source are
frozen when the inference request is created. Later command/hybrid/live gripper
updates cannot change the queued observation's two gripper channels. The
gripper metadata records **request selection time**, not a device sample or
ACK time; it is distinct from the server-time window of the arm features.

## Time and lifecycle contract

- The frozen observation payload's `loop_start_time_ns` is the window endpoint;
  the beginning is exactly one configured policy period earlier. This timestamp
  identifies the logical coordinator tick, not a backend send/ACK time.
- Both arms use the same endpoints. Position is linearly interpolated and
  quaternion orientation uses Slerp. Each interpolation bracket must span no
  more than one policy period. There is no extrapolation.
- History later than the frozen endpoint is excluded, even when inference is
  delayed and the collector has received newer packets. Request/worker timing
  therefore cannot silently slide the proprio window forward.
- Server steady-clock timestamps and Python monotonic time must be from the
  **same host**. The state client enables this collector only on explicit
  request and accepts a loopback UDP sender for this source. Remote hosts or a
  relay of remote-clock packets are unsupported; clock synchronization is not
  inferred from a plausible timestamp. A future or stale sample is invalid.
- This is state-time alignment. Camera images keep the existing worker-time
  selection and may have a different timestamp from the frozen server window
  and request-selected gripper channels.
  Diagnostics report the existing camera raw-to-monotonic mapping (or its
  receipt-time fallback), and `camera_minus_state_ms`. This source does not
  claim exact image-time synchronization.
- `motion_epoch` and each arm's force-control `reference_reset_count` must be
  present. Their changes clear the applicable history. Increasing timestamp
  with decreasing tick is treated as a server restart and clears both arms;
  old/repeated UDP timestamps are ignored rather than rolling epochs back.
- Policy/RTC reset adds a local cutoff without clearing the shared collector.
  A new window must fit entirely after that cutoff. The collector keeps
  recording while policy execution is paused for InitMotion/tare/Hold.
- A missing epoch/pose/timestamp, history gap, stale observation or fault yields
  zero-valued unavailable features with `valid=false` and an explicit reason.
  The new source returns before the model request when either arm's window is
  invalid. It does not present unavailable motion as a valid rest observation.

The existing state-client stale timeout governs observation age. The bounded
history retains 2048 state samples; it is sampled at the actual UDP publication
rate, not the nominal 500 Hz servo rate. No server schema or safety limits are
changed by this feature.

## Audit and offline validation

Per-inference `chunk_metadata.proprio` and the existing inference diagnostics
carry source/semantics, exact start/end nanoseconds, observation tick, motion
epoch, arm reset counts, interpolation brackets, endpoint poses, body deltas,
age, camera/state skew, and validity reasons. This records which trajectory and
time window the model actually received. Gripper selection timestamps, selected
source/value, and selection-minus-state skew are carried separately. An absent
or invalid frozen gripper observation also prevents a model request.

Hardware/network-free tests:

```bash
PYTHONPATH=policy_runner python3 -m unittest discover -s policy_runner/tests \
  -p 'test_servo_command_proprio.py'
PYTHONPATH=policy_runner python3 -m unittest discover -s policy_runner/tests \
  -p 'test_openpi_remote_velproprio.py'
```

Coverage includes delayed-worker cutoff invariance, body-frame/rotation math,
Init/motion/reset epochs, server restart vs UDP reordering, unavailable/stale
signals, unsupported remote clocks, state collection without policy ticks,
feature layout and refusal to invoke the model on invalid history. These tests
also change the live gripper command after request capture and verify that the
complete 14-value input stays frozen. They do not establish hardware or
model-success acceptance.
