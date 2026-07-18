# Guarded contact from measured force

`surface_source: contact_force` is an opt-in Cartesian-admittance mode for
placement without a configured geometric plane. It is implemented but remains
unselected in the tracked real stack until a supervised operator activation.

## Episode state machine

Outside contact, the measured surface frame is invalid and scalar normal
regulation is inactive. Fresh control-wrench samples enter an episode only
after `debounce_samples` consecutive samples exceed
`contact_enter_force_n`. Entry captures

```text
n_stand = -normalize(force_control_stand)
```

and freezes that outward normal for the entire episode. Later force-direction
rotation cannot steer the controller. An invalid capture fails closed instead
of substituting a direction.

While active, the scalar `NormalForceController` owns the frozen normal and
regulates `target_force_n`. Policy translation and the symmetric Cartesian
admittance are projected onto the tangent plane; tangential translation and
the configured rotational axes retain their existing behavior. The Cartesian
offset component already present on entry is held fixed on the normal so the
two controllers cannot compete.

The chunk follower receives the same frozen inward direction. Direction
consistency and the per-segment removal clamp remain mandatory. The usual
quasi-static gate is bypassed only while this debounced `contact_force` episode
owns the normal: the latched episode is the contact evidence. Other surface
sources retain the existing wrench, quasi-static, loading-only, and clamp
semantics unchanged.

Release uses the existing brake-to-hold path. Below
`contact_release_force_n`, the scalar controller brakes; once its velocity is
inside `release_velocity_threshold_m_s` for `release_dwell_sec`, the server
latches the measured pose, invalidates the normal, resets the episode-specific
controller state, publishes a new motion epoch, and waits for the policy to
re-anchor. This avoids handing a stale normal-axis target back to the policy.

Out of scope: per-tick normal re-estimation, geometric-plane inference,
friction/cone estimation, torque-side inertial compensation, CoM/inertia
identification, and safety-rated contact sensing. The 30/35 N and 7 Nm hard
limits remain unchanged and authoritative.
