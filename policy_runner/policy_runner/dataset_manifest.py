from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


DATASET_MANIFEST_SCHEMA = "robotics_lab.policy_runner.dataset_manifest.v1"
KNOWN_DATASET_FORMATS = {
    "pika_umi_single_arm",
    "pika_umi_bimanual",
    "robotics_lab_dual_arm",
}


@dataclass(frozen=True)
class DatasetManifest:
    """Lightweight dataset adapter manifest for HDF5 flow training."""

    episodes_dir: str | None = None
    include_formats: tuple[str, ...] = field(default_factory=tuple)
    single_arm_side: str = "left"
    camera_names: tuple[str, ...] = field(default_factory=tuple)
    exclude_camera_names: tuple[str, ...] = field(default_factory=tuple)
    required_attrs: dict[str, str] = field(default_factory=dict)
    retarget: dict[str, str] = field(default_factory=dict)
    schema: str = DATASET_MANIFEST_SCHEMA

    @classmethod
    def load(cls, path: str | Path) -> "DatasetManifest":
        manifest_path = Path(path)
        data = _load_mapping(manifest_path)
        return cls.from_mapping(data, source=manifest_path)

    @classmethod
    def from_mapping(
        cls,
        data: dict[str, Any],
        *,
        source: str | Path = "<manifest>",
    ) -> "DatasetManifest":
        schema = str(data.get("schema", DATASET_MANIFEST_SCHEMA) or "")
        if schema != DATASET_MANIFEST_SCHEMA:
            raise ValueError(f"{source}: unsupported dataset manifest schema: {schema}")

        single_arm_side = str(data.get("single_arm_side", "left") or "left")
        if single_arm_side not in {"left", "right"}:
            raise ValueError(f"{source}: single_arm_side must be left or right")

        include_formats = tuple(_string_list(data.get("include_formats", [])))
        unknown_formats = sorted(set(include_formats) - KNOWN_DATASET_FORMATS)
        if unknown_formats:
            raise ValueError(f"{source}: unknown include_formats: {', '.join(unknown_formats)}")

        required_attrs = _string_mapping(data.get("required_attrs", {}), field_name="required_attrs")
        retarget = _string_mapping(data.get("retarget", {}), field_name="retarget")

        return cls(
            episodes_dir=_optional_string(data.get("episodes_dir")),
            include_formats=include_formats,
            single_arm_side=single_arm_side,
            camera_names=tuple(_string_list(data.get("camera_names", []))),
            exclude_camera_names=tuple(_string_list(data.get("exclude_camera_names", []))),
            required_attrs=required_attrs,
            retarget=retarget,
            schema=schema,
        )

    def resolved_episodes_dir(self, override: str | Path | None = None) -> str:
        if override is not None:
            return str(override)
        if self.episodes_dir:
            return self.episodes_dir
        return "data/episodes"

    def selected_camera_names(
        self,
        override: list[str] | tuple[str, ...] | None = None,
    ) -> list[str] | None:
        names = list(override) if override is not None else list(self.camera_names)
        if not names:
            return None
        excluded = set(self.exclude_camera_names)
        return [name for name in names if name not in excluded]

    def training_dataset_kwargs(
        self,
        *,
        camera_names_override: list[str] | None = None,
        exclude_camera_names_override: list[str] | None = None,
        single_arm_side_override: str | None = None,
    ) -> dict[str, Any]:
        side = single_arm_side_override or self.single_arm_side
        if side not in {"left", "right"}:
            raise ValueError("single_arm_side must be left or right")
        excluded = list(dict.fromkeys([*self.exclude_camera_names, *(exclude_camera_names_override or [])]))
        return {
            "camera_names": _exclude_names(self.selected_camera_names(camera_names_override), excluded),
            "single_arm_side": side,
            "include_formats": list(self.include_formats) or None,
            "exclude_camera_names": excluded,
            "required_attrs": dict(self.required_attrs),
        }

    def retarget_is_measured_for(self, pose_frame: str, target_frame: str = "stand") -> bool:
        return (
            str(self.retarget.get("pose_frame", "") or "") == pose_frame
            and str(self.retarget.get("target_frame", "") or "") == target_frame
            and str(self.retarget.get("transform_status", "") or "") in {"measured", "accepted"}
        )


def parse_camera_names(value: str | None) -> list[str] | None:
    if value is None:
        return None
    names = [part.strip() for part in value.split(",") if part.strip()]
    return names or []


def _exclude_names(names: list[str] | None, excluded: list[str]) -> list[str] | None:
    if names is None:
        return None
    excluded_set = set(excluded)
    return [name for name in names if name not in excluded_set]


def _load_mapping(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8")
    if path.suffix.lower() == ".json":
        loaded = json.loads(text)
    else:
        try:
            import yaml  # type: ignore
        except ModuleNotFoundError:
            loaded = _parse_simple_yaml(text)
        else:
            loaded = yaml.safe_load(text)
    if not isinstance(loaded, dict):
        raise ValueError(f"{path}: dataset manifest must contain a mapping")
    return loaded


def _parse_simple_yaml(text: str) -> dict[str, Any]:
    root: dict[str, Any] = {}
    pending_key: str | None = None

    for line_number, raw_line in enumerate(text.splitlines(), start=1):
        line = raw_line.split("#", 1)[0].rstrip()
        if not line:
            continue
        indent = len(line) - len(line.lstrip(" "))
        stripped = line.strip()

        if indent == 0:
            key, separator, value = stripped.partition(":")
            if separator != ":":
                raise ValueError(f"line {line_number}: expected key: value")
            key = key.strip()
            value = value.strip()
            if value:
                root[key] = _parse_scalar_or_inline_list(value)
                pending_key = None
            else:
                root[key] = {}
                pending_key = key
            continue

        if indent != 2 or pending_key is None:
            raise ValueError(f"line {line_number}: expected one nested level")

        if stripped.startswith("- "):
            current = root.get(pending_key)
            if current == {}:
                current = []
                root[pending_key] = current
            if not isinstance(current, list):
                raise ValueError(f"line {line_number}: mixed list and mapping values")
            current.append(_parse_scalar_or_inline_list(stripped[2:].strip()))
            continue

        key, separator, value = stripped.partition(":")
        if separator != ":":
            raise ValueError(f"line {line_number}: expected key: value")
        current = root.get(pending_key)
        if current == []:
            raise ValueError(f"line {line_number}: mixed list and mapping values")
        if current == {}:
            root[pending_key] = current
        if not isinstance(current, dict):
            raise ValueError(f"line {line_number}: expected mapping values")
        current[key.strip()] = _parse_scalar_or_inline_list(value.strip())

    return root


def _parse_scalar_or_inline_list(value: str) -> Any:
    if value.startswith("[") and value.endswith("]"):
        inner = value[1:-1].strip()
        if not inner:
            return []
        return [_parse_scalar_or_inline_list(part.strip()) for part in inner.split(",")]
    if (
        (value.startswith('"') and value.endswith('"'))
        or (value.startswith("'") and value.endswith("'"))
    ):
        return value[1:-1]
    lowered = value.lower()
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    if lowered in {"null", "none"}:
        return None
    try:
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        return value


def _string_list(value: Any) -> list[str]:
    if value is None:
        return []
    if isinstance(value, str):
        return [part.strip() for part in value.split(",") if part.strip()]
    if isinstance(value, (list, tuple)):
        return [str(item) for item in value]
    raise ValueError(f"expected a list of strings, got {type(value).__name__}")


def _string_mapping(value: Any, *, field_name: str) -> dict[str, str]:
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError(f"{field_name} must be a mapping")
    return {str(key): str(item) for key, item in value.items()}


def _optional_string(value: Any) -> str | None:
    if value is None:
        return None
    text = str(value)
    return text if text else None
