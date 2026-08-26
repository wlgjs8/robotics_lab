# Config Policy And Examples

The parser is strict YAML (`yaml-cpp`) with known-key allowlists. Config struct,
parser, validation, and loader tests must change together for any future schema
work.

## Runnable configurations

There are exactly two tracked launch configs:

```text
config/stack_real.yaml  # physical real
config/stack_sim.yaml   # rbpodo controller pgmode simulation
```

Do not create `config/local` launch variants. Relative URDF/package paths can
resolve differently one directory deeper, the effective real-motion profile
disappears from review, and a local file carrying real enables can be committed
accidentally.

For a reviewed acceptance stage, edit one setting in the appropriate tracked
config, record the diff with the artifact, preflight it without hardware, run
the supervised procedure, and restore that setting explicitly:

```bash
rb_servo_server/build/rbpodo_real_gate/rb_servo_server \
  --check-config --config rb_servo_server/config/stack_real.yaml

git diff -- rb_servo_server/config/stack_real.yaml
```

The simulation equivalent uses `stack_sim.yaml`. A hardware-free mock smoke may
use an explicit temporary YAML outside the repository, such as a `mktemp -d`
path; it is not a third launch profile.

## Stable public values

```yaml
run_mode: mock | simulation | real
backend_type: mock | rbpodo
```

Supported rbpodo Servo J fields are `servo_t1_sec`, `servo_t2_sec`,
`servo_gain`, and `servo_alpha`. The supported 500 Hz profile uses
`0.002`, `0.021`, `1.0`, and `10.0` respectively.

The raw joint envelope is:

```yaml
safety:
  q_min_deg: [-360, -360, -150, -360, -360, -360]
  q_max_deg: [360, 360, 150, 360, 360, 360]
```

J3 must remain exactly `[-150 deg, +150 deg]`, matching Rainbow and the URDF.

## Force-control ownership

`force_torque:` and `force_control:` are live server config sections in the
tracked real profile. Their measured sensor/tool parameters and coupled
gate/spring/fence values must be reviewed as one safety-relevant unit. A client
command may request `TareForceSensor`; it may not override the force law with a
`force_control` payload.
