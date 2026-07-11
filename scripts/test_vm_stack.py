#!/usr/bin/env python3
"""Regression tests for the VirtualBox VM stack helper."""

from __future__ import annotations

import os
import subprocess
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
VM_STACK = REPO_ROOT / "tools/vm_stack.sh"


def write_executable(path: Path, contents: str) -> None:
    path.write_text(contents, encoding="utf-8")
    path.chmod(0o755)


class VmStackTest(unittest.TestCase):
    def run_vm_up(
        self, interface_present: bool
    ) -> tuple[subprocess.CompletedProcess[str], list[str]]:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            bin_dir = root / "bin"
            bin_dir.mkdir()
            event_log = root / "events.log"

            write_executable(
                bin_dir / "vboxmanage",
                """#!/usr/bin/env bash
printf 'vboxmanage %s\\n' "$*" >>"$VM_STACK_TEST_LOG"
if [ "${1:-} ${2:-}" = "list hostonlyifs" ]; then
  printf 'Name:            vboxnet0\\n'
fi
exit 0
""",
            )
            write_executable(
                bin_dir / "ip",
                """#!/usr/bin/env bash
printf 'ip %s\\n' "$*" >>"$VM_STACK_TEST_LOG"
if [ "$*" = "link show dev vboxnet0" ]; then
  [ "$VM_STACK_TEST_INTERFACE_PRESENT" = "1" ]
  exit
fi
exit 0
""",
            )
            write_executable(
                bin_dir / "sudo",
                """#!/usr/bin/env bash
printf 'sudo %s\\n' "$*" >>"$VM_STACK_TEST_LOG"
exit 0
""",
            )
            write_executable(
                bin_dir / "timeout",
                """#!/usr/bin/env bash
printf 'timeout %s\\n' "$*" >>"$VM_STACK_TEST_LOG"
exit 0
""",
            )

            env = os.environ.copy()
            env.update(
                {
                    "PATH": f"{bin_dir}:{env['PATH']}",
                    "SUDO_ASKPASS": str(bin_dir / "unused-askpass"),
                    "SUDO_PASSWORD": "test-only",
                    "VM_STACK_TEST_INTERFACE_PRESENT": "1" if interface_present else "0",
                    "VM_STACK_TEST_LOG": str(event_log),
                }
            )
            completed = subprocess.run(
                [str(VM_STACK), "up"],
                cwd=REPO_ROOT,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=10,
                check=False,
            )
            events = event_log.read_text(encoding="utf-8").splitlines()
            return completed, events

    def test_virtualbox_is_configured_before_network_mapping(self) -> None:
        completed, events = self.run_vm_up(interface_present=True)

        self.assertEqual(completed.returncode, 0, completed.stderr)
        initialize_index = events.index("vboxmanage list hostonlyifs")
        configure_ip_index = events.index(
            "vboxmanage setextradata global "
            "HostOnly/vboxnet0/IPAddress 10.0.2.1"
        )
        configure_mask_index = events.index(
            "vboxmanage setextradata global "
            "HostOnly/vboxnet0/IPNetMask 255.255.255.0"
        )
        mapping_index = next(
            index
            for index, event in enumerate(events)
            if event.startswith("sudo -A ip addr replace 10.0.2.1/24 dev vboxnet0")
        )
        self.assertLess(initialize_index, configure_ip_index)
        self.assertLess(configure_ip_index, configure_mask_index)
        self.assertLess(configure_mask_index, mapping_index)

    def test_missing_hostonly_interface_fails_before_network_changes(self) -> None:
        completed, events = self.run_vm_up(interface_present=False)

        self.assertNotEqual(completed.returncode, 0)
        self.assertIn(
            "required VirtualBox host-only interface vboxnet0 is unavailable",
            completed.stderr,
        )
        self.assertFalse(any(event.startswith("sudo ") for event in events))


if __name__ == "__main__":
    unittest.main()
