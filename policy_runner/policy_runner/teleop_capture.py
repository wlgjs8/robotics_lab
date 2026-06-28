"""Fail-safe per-tick teleop capture logger (symptom-3 focus).

Records the mux arbitration state + the intent actually produced each tick, keyed
by CLOCK_MONOTONIC ns so it aligns directly with the C++ servo_log
loop_start_time_ns and the operator marker log. Enabled by env
POLICY_RUNNER_TELEOP_CAPTURE in {1,on,true,yes,auto}.

Every method swallows its own exceptions: a logging fault must NEVER perturb the
teleop control loop.
"""
from __future__ import annotations

import os
from datetime import datetime
from pathlib import Path
from typing import Any


def _enabled() -> bool:
    return os.environ.get("POLICY_RUNNER_TELEOP_CAPTURE", "").lower() in (
        "1", "on", "true", "yes", "auto",
    )


def _arm_mode(arm: Any) -> str:
    if isinstance(arm, dict):
        return str(arm.get("mode", "-"))
    return getattr(arm, "mode", "-") if arm is not None else "-"


class TeleopCaptureLogger:
    def __init__(self) -> None:
        self._fh = None
        self.path = None
        if not _enabled():
            return
        try:
            logs_dir = Path(__file__).resolve().parents[2] / "logs"
            logs_dir.mkdir(parents=True, exist_ok=True)
            self.path = logs_dir / f"teleop_mux_{datetime.now().strftime('%Y%m%d_%H%M%S')}_KST.log"
            self._fh = open(self.path, "w", buffering=1, encoding="utf-8")
            self._fh.write(f"# teleop mux capture  started={datetime.now().isoformat()}\n")
            self._fh.write(
                "# fields: mono_ns=<CLOCK_MONOTONIC ns == servo loop_start_time_ns>  wall=<iso>  "
                "owner=<idle|spacemouse|umi|?>  sm_eng=<0/1>  umi_eng=<0/1>  "
                "imode=<intent.mode>  intentL=<arm mode>  intentR=<arm mode>  "
                "motion=<0/1>  allowed=<0/1>\n"
            )
            print(f"[teleop-capture] mux state -> {self.path}", flush=True)
        except Exception:
            self._fh = None

    def log(self, now_monotonic: float, source: Any, intent: Any, allowed: Any) -> None:
        if self._fh is None:
            return
        try:
            owner = getattr(source, "owner", "?")
            sm = getattr(source, "spacemouse_source", None)
            umi = getattr(source, "umi_source", None)
            sm_eng = int(bool(getattr(sm, "engaged", False))) if sm is not None else "-"
            umi_eng = int(bool(getattr(umi, "engaged", False))) if umi is not None else "-"
            if intent is None:
                imode, il, ir, motion = "None", "-", "-", 0
            else:
                imode = getattr(intent, "mode", "-")
                il = _arm_mode(getattr(intent, "left", None))
                ir = _arm_mode(getattr(intent, "right", None))
                motion = int(bool(getattr(intent, "is_motion", False)))
            mono_ns = int(now_monotonic * 1e9)
            self._fh.write(
                f"mono_ns={mono_ns}  wall={datetime.now().isoformat()}  "
                f"owner={owner}  sm_eng={sm_eng}  umi_eng={umi_eng}  "
                f"imode={imode}  intentL={il}  intentR={ir}  "
                f"motion={motion}  allowed={int(bool(allowed))}\n"
            )
        except Exception:
            pass

    def close(self) -> None:
        try:
            if self._fh is not None:
                self._fh.close()
        except Exception:
            pass
