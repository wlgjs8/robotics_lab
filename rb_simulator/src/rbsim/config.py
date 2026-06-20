"""Configuration loading for the hardware-free simulator."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class ArmConfig:
    name: str
    initial_q_deg: tuple[float, float, float, float, float, float]
    mount: tuple[float, ...]


@dataclass(frozen=True)
class FaultDefaults:
    connected: bool
    initialized: bool
    servo_enabled: bool
    has_valid_joint_state: bool
    has_error: bool
    error_code: int


@dataclass(frozen=True)
class SimulatorConfig:
    arm: str
    control_bind: str
    admin_bind: str
    update_rate_hz: float
    motion_time_constant_sec: float
    max_joint_velocity_deg_s: float
    model: str
    arm_config: ArmConfig
    fault_defaults: FaultDefaults


def load_simulator_config(path: str | Path) -> SimulatorConfig:
    """Load a single-arm simulator YAML profile without requiring PyYAML."""

    raw = _load_yaml_subset(Path(path))
    try:
        simulator = raw["simulator"]
        fault_defaults = raw["fault_defaults"]
        arm = str(simulator["arm"])
        if arm not in {"left", "right"}:
            raise ValueError("simulator.arm must be 'left' or 'right'")
        return SimulatorConfig(
            arm=arm,
            control_bind=str(simulator["control_bind"]),
            admin_bind=str(simulator["admin_bind"]),
            update_rate_hz=float(simulator["update_rate_hz"]),
            motion_time_constant_sec=float(simulator["motion_time_constant_sec"]),
            max_joint_velocity_deg_s=float(simulator.get("max_joint_velocity_deg_s", 360.0)),
            model=str(simulator["model"]),
            arm_config=_parse_arm(raw["arm"]),
            fault_defaults=FaultDefaults(
                connected=bool(fault_defaults["connected"]),
                initialized=bool(fault_defaults["initialized"]),
                servo_enabled=bool(fault_defaults["servo_enabled"]),
                has_valid_joint_state=bool(fault_defaults["has_valid_joint_state"]),
                has_error=bool(fault_defaults["has_error"]),
                error_code=int(fault_defaults["error_code"]),
            ),
        )
    except KeyError as exc:
        raise ValueError(f"missing simulator config field: {exc.args[0]}") from exc


# Canonical hardware-free startup pose, shared with the C++ servo-server default
# in loadConfigFromYaml (rb_servo_server/src/config/config.cpp). Configs no longer
# carry initial_q_deg; this is the single source of the simulator start pose.
DEFAULT_INITIAL_Q_DEG: tuple[float, float, float, float, float, float] = (
    0.0,
    -30.0,
    80.0,
    0.0,
    60.0,
    0.0,
)


def _parse_arm(raw: dict[str, Any]) -> ArmConfig:
    if "initial_q_deg" in raw:
        initial_q = _six_joint_tuple(raw["initial_q_deg"], "initial_q_deg")
    else:
        initial_q = DEFAULT_INITIAL_Q_DEG
    mount = tuple(float(value) for value in raw.get("mount", ()))
    return ArmConfig(name=str(raw["name"]), initial_q_deg=initial_q, mount=mount)


def _six_joint_tuple(values: Any, field_name: str) -> tuple[float, float, float, float, float, float]:
    parsed = tuple(float(value) for value in values)
    if len(parsed) != 6:
        raise ValueError(f"{field_name} must contain exactly 6 joints")
    return parsed  # type: ignore[return-value]


def _load_yaml_subset(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8")
    try:
        import yaml  # type: ignore
    except ModuleNotFoundError:
        return _parse_simple_yaml(text)

    loaded = yaml.safe_load(text)
    if not isinstance(loaded, dict):
        raise ValueError(f"{path} did not contain a YAML mapping")
    return loaded


def _parse_simple_yaml(text: str) -> dict[str, Any]:
    root: dict[str, Any] = {}
    current_section: dict[str, Any] | None = None

    for line_number, raw_line in enumerate(text.splitlines(), start=1):
        line = raw_line.split("#", 1)[0].rstrip()
        if not line:
            continue
        indent = len(line) - len(line.lstrip(" "))
        stripped = line.strip()

        if indent == 0:
            if not stripped.endswith(":"):
                raise ValueError(f"line {line_number}: expected top-level section")
            section_name = stripped[:-1]
            current_section = {}
            root[section_name] = current_section
            continue

        if indent != 2 or current_section is None:
            raise ValueError(f"line {line_number}: expected key inside a section")

        key, separator, value = stripped.partition(":")
        if separator != ":":
            raise ValueError(f"line {line_number}: expected key: value")
        current_section[key] = _parse_scalar_or_list(value.strip())

    return root


def _parse_scalar_or_list(value: str) -> Any:
    if value.startswith("[") and value.endswith("]"):
        inner = value[1:-1].strip()
        if not inner:
            return []
        return [_parse_scalar_or_list(part.strip()) for part in inner.split(",")]
    if value.startswith('"') and value.endswith('"'):
        return value[1:-1]
    lowered = value.lower()
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    try:
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        return value
