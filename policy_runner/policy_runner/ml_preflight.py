from __future__ import annotations

import importlib
import sys
import traceback
from dataclasses import dataclass
from typing import Any, Callable, TextIO


REQUESTED_BACKBONES = (
    "tiny_cnn",
    "resnet18",
    "resnet50",
    "dinov3",
    "dinov3_convnext_tiny",
    "dinov3_convnext_small",
    "dinov3_convnext_base",
    "dinov3_convnext_large",
)
TORCHVISION_BACKBONES = {
    "resnet18",
    "resnet50",
    "dinov3_convnext_tiny",
    "dinov3_convnext_small",
    "dinov3_convnext_base",
    "dinov3_convnext_large",
}


@dataclass(frozen=True)
class ImportStatus:
    name: str
    ok: bool
    version: str
    error: str
    traceback: str
    skipped: bool = False
    skip_reason: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "ok": self.ok,
            "version": self.version,
            "error": self.error,
            "traceback": self.traceback,
            "skipped": self.skipped,
            "skip_reason": self.skip_reason,
        }


def run_ml_preflight(
    *,
    vision_backbone: str,
    require_cuda: bool = False,
    expect_cuda_device_count: int | None = None,
    stdout: TextIO = sys.stdout,
    import_module: Callable[[str], Any] = importlib.import_module,
) -> int:
    report = check_ml_preflight(
        vision_backbone=vision_backbone,
        require_cuda=require_cuda,
        expect_cuda_device_count=expect_cuda_device_count,
        import_module=import_module,
    )
    stdout.write(render_ml_preflight(report))
    stdout.flush()
    return 0 if bool(report["ok"]) else 1


def check_ml_preflight(
    *,
    vision_backbone: str,
    require_cuda: bool = False,
    expect_cuda_device_count: int | None = None,
    import_module: Callable[[str], Any] = importlib.import_module,
) -> dict[str, Any]:
    if expect_cuda_device_count is not None and expect_cuda_device_count < 0:
        raise ValueError("expect_cuda_device_count must be non-negative")
    backbone = str(vision_backbone)
    torch_status, torch_module = _import_status("torch", import_module=import_module)
    if backbone in TORCHVISION_BACKBONES:
        torchvision_status, _ = _import_status("torchvision", import_module=import_module)
    else:
        torchvision_status = _skipped_import_status(
            "torchvision",
            reason=f"not required for {backbone}",
        )
    h5py_status, _ = _import_status("h5py", import_module=import_module)
    pillow_status, _ = _import_status("PIL", import_module=import_module)

    cuda_available = False
    cuda_device_count = 0
    cuda_device_names: list[str] = []
    if torch_status.ok:
        cuda_available = bool(torch_module.cuda.is_available())
        if cuda_available:
            cuda_device_count = int(torch_module.cuda.device_count())
            cuda_device_names = [
                str(torch_module.cuda.get_device_name(index))
                for index in range(cuda_device_count)
            ]

    forward = _check_backbone_forward(
        backbone,
        torch_status=torch_status,
        torchvision_status=torchvision_status,
        torch_module=torch_module,
    )
    requested_ok = bool(forward["ok"])
    cuda_ok = _cuda_status_ok(
        available=cuda_available,
        device_count=cuda_device_count,
        require_cuda=require_cuda,
        expect_device_count=expect_cuda_device_count,
    )
    ok = bool(requested_ok and cuda_ok["ok"])

    return {
        "ok": ok,
        "requested_backbone": {
            "name": backbone,
            "ok": requested_ok,
            "error": "" if requested_ok else str(forward.get("error", "")),
        },
        "torch": torch_status.to_dict(),
        "torchvision": torchvision_status.to_dict(),
        "cuda_available": cuda_available,
        "cuda_device_count": cuda_device_count,
        "cuda_device_names": cuda_device_names,
        "cuda_check": cuda_ok,
        "hdf5": h5py_status.to_dict(),
        "pillow": pillow_status.to_dict(),
        "backbone_forward": forward,
    }


def render_ml_preflight(report: dict[str, Any]) -> str:
    lines = [
        "policy_runner ML preflight",
        f"requested_backbone: {report['requested_backbone']['name']}",
        _format_import("torch", report["torch"]),
        _format_import("torchvision", report["torchvision"]),
        f"cuda_available: {str(bool(report['cuda_available'])).lower()}",
        f"cuda_device_count: {int(report.get('cuda_device_count', 0))}",
        _format_cuda_device_names(report.get("cuda_device_names", [])),
        _format_cuda_check(report.get("cuda_check", {})),
        _format_import("hdf5", report["hdf5"]),
        _format_import("pillow", report["pillow"]),
        _format_forward(report["backbone_forward"]),
    ]
    if report["torchvision"].get("traceback"):
        lines.extend(["torchvision_error:", str(report["torchvision"]["traceback"]).rstrip()])
    if report["torch"].get("traceback"):
        lines.extend(["torch_error:", str(report["torch"]["traceback"]).rstrip()])
    if not report["requested_backbone"]["ok"]:
        lines.append(f"requested_backbone_error: {report['requested_backbone']['error']}")
    return "\n".join(lines).rstrip() + "\n"


def _cuda_status_ok(
    *,
    available: bool,
    device_count: int,
    require_cuda: bool,
    expect_device_count: int | None,
) -> dict[str, Any]:
    errors: list[str] = []
    if require_cuda and not available:
        errors.append("cuda unavailable")
    if expect_device_count is not None and int(device_count) != int(expect_device_count):
        errors.append(
            f"expected {int(expect_device_count)} CUDA devices, found {int(device_count)}"
        )
    return {
        "ok": not errors,
        "require_cuda": bool(require_cuda),
        "expect_device_count": expect_device_count,
        "errors": errors,
    }


def _import_status(
    module_name: str,
    *,
    import_module: Callable[[str], Any],
) -> tuple[ImportStatus, Any | None]:
    try:
        module = import_module(module_name)
    except Exception as exc:  # noqa: BLE001 - preflight must report full import failures.
        return (
            ImportStatus(
                name=module_name,
                ok=False,
                version="",
                error=f"{type(exc).__name__}: {exc}",
                traceback=traceback.format_exc(),
            ),
            None,
        )
    return (
        ImportStatus(
            name=module_name,
            ok=True,
            version=str(getattr(module, "__version__", "") or ""),
            error="",
            traceback="",
        ),
        module,
    )


def _check_backbone_forward(
    backbone: str,
    *,
    torch_status: ImportStatus,
    torchvision_status: ImportStatus,
    torch_module: Any | None,
) -> dict[str, Any]:
    if backbone not in REQUESTED_BACKBONES:
        return {
            "ok": False,
            "backbone": backbone,
            "error": f"unsupported vision_backbone {backbone!r}",
            "output_shape": [],
        }
    if not torch_status.ok or torch_module is None:
        return {
            "ok": False,
            "backbone": backbone,
            "error": "torch import failed",
            "output_shape": [],
        }
    if backbone in TORCHVISION_BACKBONES and not torchvision_status.ok:
        return {
            "ok": False,
            "backbone": backbone,
            "error": "torchvision import failed",
            "output_shape": [],
        }

    try:
        from .flow_model import VisionBackbone

        model = VisionBackbone(backbone, output_dim=8, frozen=True)
        model.eval()
        images = torch_module.zeros(2, 3, 32, 32)
        with torch_module.no_grad():
            output = model(images)
    except Exception as exc:  # noqa: BLE001 - this is diagnostic output.
        return {
            "ok": False,
            "backbone": backbone,
            "error": f"{type(exc).__name__}: {exc}",
            "traceback": traceback.format_exc(),
            "output_shape": [],
        }
    return {
        "ok": True,
        "backbone": backbone,
        "error": "",
        "traceback": "",
        "output_shape": list(output.shape),
    }


def _skipped_import_status(module_name: str, *, reason: str) -> ImportStatus:
    return ImportStatus(
        name=module_name,
        ok=True,
        version="",
        error="",
        traceback="",
        skipped=True,
        skip_reason=reason,
    )


def _format_import(label: str, status: dict[str, Any]) -> str:
    if status.get("skipped"):
        return f"{label}: SKIPPED {status.get('skip_reason', '')}".rstrip()
    if status.get("ok"):
        version = str(status.get("version") or "unknown")
        return f"{label}: OK version={version}"
    return f"{label}: ERROR {status.get('error', '')}"


def _format_cuda_device_names(names: list[str]) -> str:
    if not names:
        return "cuda_device_names: []"
    return "cuda_device_names: " + json_dumps_compact(names)


def _format_cuda_check(status: dict[str, Any]) -> str:
    if status.get("ok", True):
        return "cuda_check: OK"
    return "cuda_check: ERROR " + "; ".join(str(item) for item in status.get("errors", []))


def _format_forward(forward: dict[str, Any]) -> str:
    if forward.get("ok"):
        return (
            "backbone_forward: OK "
            f"backbone={forward.get('backbone', '')} "
            f"output_shape={forward.get('output_shape', [])}"
        )
    return (
        "backbone_forward: ERROR "
        f"backbone={forward.get('backbone', '')} "
        f"{forward.get('error', '')}"
    )


def json_dumps_compact(value: Any) -> str:
    import json

    return json.dumps(value, ensure_ascii=True, separators=(",", ":"))
