# Rainbow VM Network Bring-Up

This runbook brings up two Rainbow controller-simulation VMs for rbpodo parity.
It does not authorize physical robot motion.

## Topology

Use one VM per arm. rbpodo takes only a controller IP; its command/data ports
are fixed by the SDK/controller protocol as `5000` and `5001`. Do not try to
map two VMs through one localhost IP with different host ports.

Recommended host-only shape:

```text
rb_servo_server
  left_robot  backend_type=rbpodo, run_mode=real, operation_mode=simulation -> <left-vm-ip>:5000/5001
  right_robot backend_type=rbpodo, run_mode=real, operation_mode=simulation -> <right-vm-ip>:5000/5001
```

Example host-only addresses are `192.168.56.101` and `192.168.56.102`; replace
them locally and keep actual site values out of tracked configs.

The repository-managed stack uses `vboxnet0` with host address `10.0.2.1/24`.
`make vm-up` first queries VirtualBox so its service recreates configured
host-only adapters after a host reboot. It then verifies the Linux interface
and synchronizes the VirtualBox user configuration's persisted `vboxnet0`
address to `10.0.2.1/24` before changing the Linux interface, host routes, or
DNAT state. This prevents a later `VBoxManage` call from restoring
VirtualBox's default `192.168.56.1/24` address.

## OVA Metadata Check

```bash
tools/vm/verify_ova.sh /path/to/rainbow-controller-sim.ova \
  --output artifacts/vm_parity/WU-01/ova_verify.json
```

The script records SHA-256, OVA tar contents, OVF/VMDK presence, Linux-like OS
metadata, NIC count, and optional `ovftool`/`VBoxManage` dry-run results. It
does not boot or modify the appliance.

## VM Import Checklist

1. Import the OVA twice as separate VMs, for example `rbvm-left` and
   `rbvm-right`.
2. Regenerate MAC addresses during import.
3. Attach each VM NIC to the same host-only network, or to bridged networking
   only when the site network policy allows it.
4. Assign each VM a distinct IP through the appliance UI, controller tooling, or
   host DHCP reservation. Do not edit guest internals blindly.
5. Confirm each controller is in Rainbow pgmode simulation before motion tests.

## Reachability Probe

```bash
tools/vm/probe_vm_reachability.sh \
  --left <left-vm-ip> \
  --right <right-vm-ip> \
  --output artifacts/vm_parity/WU-01/reachability.json
```

Optional read-only rbpodo state check:

```bash
tools/vm/probe_vm_reachability.sh \
  --left <left-vm-ip> \
  --right <right-vm-ip> \
  --try-rbpodo-state
```

Acceptance for WU-01 requires all four TCP endpoints
`left:5000`, `left:5001`, `right:5000`, and `right:5001` to connect. If the
optional state probe is enabled, it must also receive `CobotData` once per VM.

All artifacts under `artifacts/vm_parity/` must carry:

```json
{
  "source": "controller_simulation_vm",
  "physical_motion": false
}
```
