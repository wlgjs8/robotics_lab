# Test Placeholder

Tests added here must be hardware-free. They may start local loopback simulator
processes, but must not require privileged Docker, OVA images, Rainbow hardware,
RealSense devices, or external network access.

Run from the workspace root:

```bash
python3 -m unittest discover rb_simulator/tests
```
