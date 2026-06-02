from __future__ import annotations

import importlib
import math
import os
from dataclasses import dataclass
from typing import Any

import torch
from torch import nn

from .flow_dataset import FLOW_ACTION_DIM, FLOW_PROPRIO_DIM


SUPPORTED_VISION_BACKBONES = ("resnet18", "resnet50", "dinov3")


@dataclass(frozen=True)
class FlowModelConfig:
    action_horizon: int
    action_dim: int = FLOW_ACTION_DIM
    proprio_dim: int = FLOW_PROPRIO_DIM
    camera_names: tuple[str, ...] = ()
    vision_backbone: str = "resnet18"
    hidden_dim: int = 128
    condition_encoder: str = "transformer"
    frozen_vision: bool = True

    @property
    def camera_count(self) -> int:
        return len(self.camera_names)

    def to_dict(self) -> dict[str, Any]:
        return {
            "action_horizon": int(self.action_horizon),
            "action_dim": int(self.action_dim),
            "proprio_dim": int(self.proprio_dim),
            "camera_names": list(self.camera_names),
            "vision_backbone": self.vision_backbone,
            "hidden_dim": int(self.hidden_dim),
            "condition_encoder": self.condition_encoder,
            "frozen_vision": bool(self.frozen_vision),
        }

    @classmethod
    def from_mapping(cls, raw: dict[str, Any]) -> "FlowModelConfig":
        return cls(
            action_horizon=int(raw["action_horizon"]),
            action_dim=int(raw.get("action_dim", FLOW_ACTION_DIM)),
            proprio_dim=int(raw.get("proprio_dim", FLOW_PROPRIO_DIM)),
            camera_names=tuple(str(name) for name in raw.get("camera_names", [])),
            vision_backbone=str(raw.get("vision_backbone", "resnet18")),
            hidden_dim=int(raw.get("hidden_dim", 128)),
            condition_encoder=str(raw.get("condition_encoder", "transformer")),
            frozen_vision=bool(raw.get("frozen_vision", True)),
        )


class VisionBackbone(nn.Module):
    """Frozen-by-default image encoder for multi-view policy conditioning."""

    def __init__(self, name: str, output_dim: int, *, frozen: bool = True):
        super().__init__()
        if output_dim <= 0:
            raise ValueError("output_dim must be positive")
        self.name = str(name)
        self.output_dim = int(output_dim)
        self.frozen = bool(frozen)
        if self.name in {"resnet18", "resnet50"}:
            self.encoder, feature_dim = _build_resnet(self.name)
        elif self.name == "dinov3":
            self.encoder, feature_dim = _build_dinov3_plugin()
        else:
            supported = ", ".join(SUPPORTED_VISION_BACKBONES)
            raise ValueError(f"unsupported vision_backbone '{self.name}', expected one of: {supported}")
        self.projection = nn.Linear(feature_dim, self.output_dim)
        if self.frozen:
            for parameter in self.encoder.parameters():
                parameter.requires_grad = False

    def forward(self, images: torch.Tensor) -> torch.Tensor:
        if images.ndim != 4:
            raise ValueError("images must be B,C,H,W")
        if self.frozen:
            self.encoder.eval()
        with torch.set_grad_enabled(self.training and not self.frozen):
            features = self.encoder(images)
        if features.ndim > 2:
            features = torch.flatten(features, start_dim=1)
        return self.projection(features)


class SinusoidalTimeEmbedding(nn.Module):
    def __init__(self, dim: int):
        super().__init__()
        if dim <= 0:
            raise ValueError("dim must be positive")
        self.dim = int(dim)

    def forward(self, t: torch.Tensor) -> torch.Tensor:
        if t.ndim == 0:
            t = t[None]
        half = max(1, self.dim // 2)
        freqs = torch.exp(
            torch.linspace(
                0.0,
                math.log(10000.0),
                half,
                dtype=t.dtype,
                device=t.device,
            )
        )
        angles = t[:, None] * freqs[None, :]
        out = torch.cat([torch.sin(angles), torch.cos(angles)], dim=-1)
        if out.shape[-1] < self.dim:
            out = torch.nn.functional.pad(out, (0, self.dim - out.shape[-1]))
        return out[:, : self.dim]


class FlowMatchingPolicy(nn.Module):
    """Image/proprio-conditioned rectified-flow action chunk policy."""

    def __init__(self, config: FlowModelConfig):
        super().__init__()
        if config.action_horizon <= 0:
            raise ValueError("action_horizon must be positive")
        if config.action_dim <= 0:
            raise ValueError("action_dim must be positive")
        if config.condition_encoder not in {"transformer", "mlp"}:
            raise ValueError("condition_encoder must be transformer or mlp")
        self.config = config
        self.vision = VisionBackbone(
            config.vision_backbone,
            config.hidden_dim,
            frozen=config.frozen_vision,
        )
        self.camera_embedding = nn.Embedding(max(config.camera_count, 1), config.hidden_dim)
        self.proprio_mlp = nn.Sequential(
            nn.Linear(config.proprio_dim, config.hidden_dim),
            nn.GELU(),
            nn.Linear(config.hidden_dim, config.hidden_dim),
        )
        if config.condition_encoder == "transformer":
            heads = max(1, min(8, config.hidden_dim // 32))
            layer = nn.TransformerEncoderLayer(
                d_model=config.hidden_dim,
                nhead=heads,
                dim_feedforward=config.hidden_dim * 4,
                dropout=0.0,
                activation="gelu",
                batch_first=True,
                norm_first=True,
            )
            self.condition_encoder = nn.TransformerEncoder(layer, num_layers=2)
            self.condition_projection = nn.LayerNorm(config.hidden_dim)
        else:
            self.condition_encoder = None
            self.condition_projection = nn.Sequential(
                nn.Linear(config.hidden_dim, config.hidden_dim),
                nn.GELU(),
                nn.Linear(config.hidden_dim, config.hidden_dim),
            )
        self.time_embedding = nn.Sequential(
            SinusoidalTimeEmbedding(config.hidden_dim),
            nn.Linear(config.hidden_dim, config.hidden_dim),
            nn.GELU(),
            nn.Linear(config.hidden_dim, config.hidden_dim),
        )
        self.step_embedding = nn.Embedding(config.action_horizon, config.hidden_dim)
        self.decoder = nn.Sequential(
            nn.Linear(config.action_dim + config.hidden_dim, config.hidden_dim),
            nn.GELU(),
            nn.Linear(config.hidden_dim, config.hidden_dim),
            nn.GELU(),
            nn.Linear(config.hidden_dim, config.action_dim),
        )

    @classmethod
    def from_checkpoint_config(cls, raw: dict[str, Any]) -> "FlowMatchingPolicy":
        return cls(FlowModelConfig.from_mapping(raw))

    def encode_condition(self, images: torch.Tensor, proprio: torch.Tensor) -> torch.Tensor:
        if proprio.ndim != 2:
            raise ValueError("proprio must be B,D")
        batch_size = proprio.shape[0]
        tokens = [self.proprio_mlp(proprio).unsqueeze(1)]
        if images.ndim != 5:
            raise ValueError("images must be B,V,C,H,W")
        view_count = images.shape[1]
        if view_count:
            flat = images.reshape(batch_size * view_count, *images.shape[2:])
            vision = self.vision(flat).reshape(batch_size, view_count, -1)
            camera_ids = torch.arange(view_count, device=images.device).clamp_max(
                self.camera_embedding.num_embeddings - 1
            )
            vision = vision + self.camera_embedding(camera_ids)[None, :, :]
            tokens.append(vision)
        token_tensor = torch.cat(tokens, dim=1)
        if self.condition_encoder is not None:
            encoded = self.condition_encoder(token_tensor)
            pooled = encoded.mean(dim=1)
        else:
            pooled = token_tensor.mean(dim=1)
        return self.condition_projection(pooled)

    def forward(
        self,
        images: torch.Tensor,
        proprio: torch.Tensor,
        x_t: torch.Tensor,
        t: torch.Tensor,
    ) -> torch.Tensor:
        if x_t.ndim != 3:
            raise ValueError("x_t must be B,H,A")
        if x_t.shape[1] != self.config.action_horizon:
            raise ValueError("x_t horizon does not match model config")
        cond = self.encode_condition(images, proprio)
        cond = cond + self.time_embedding(t)
        step_ids = torch.arange(self.config.action_horizon, device=x_t.device)
        step_tokens = self.step_embedding(step_ids)[None, :, :] + cond[:, None, :]
        decoder_in = torch.cat([x_t, step_tokens], dim=-1)
        return self.decoder(decoder_in)


def flow_matching_loss(
    model: FlowMatchingPolicy,
    batch: dict[str, torch.Tensor],
) -> torch.Tensor:
    actions = batch["action_chunk"].float()
    mask = batch.get("action_mask")
    if mask is None:
        mask = torch.ones_like(actions)
    else:
        mask = mask.float()
    batch_size = actions.shape[0]
    t = torch.rand(batch_size, dtype=actions.dtype, device=actions.device)
    x0 = torch.randn_like(actions)
    x_t = (1.0 - t[:, None, None]) * x0 + t[:, None, None] * actions
    target_v = actions - x0
    pred_v = model(batch["images"].float(), batch["proprio"].float(), x_t, t)
    loss = ((pred_v - target_v) ** 2) * mask
    return loss.sum() / mask.sum().clamp_min(1.0)


@torch.no_grad()
def sample_action_chunks(
    model: FlowMatchingPolicy,
    images: torch.Tensor,
    proprio: torch.Tensor,
    *,
    steps: int = 16,
    initial_noise: torch.Tensor | None = None,
) -> torch.Tensor:
    if steps <= 0:
        raise ValueError("steps must be positive")
    batch_size = proprio.shape[0]
    if initial_noise is None:
        x = torch.zeros(
            (batch_size, model.config.action_horizon, model.config.action_dim),
            dtype=proprio.dtype,
            device=proprio.device,
        )
    else:
        x = initial_noise.to(device=proprio.device, dtype=proprio.dtype)
    dt = 1.0 / float(steps)
    for step in range(steps):
        t = torch.full(
            (batch_size,),
            (step + 0.5) / float(steps),
            dtype=proprio.dtype,
            device=proprio.device,
        )
        x = x + dt * model(images, proprio, x, t)
    return x


def _build_resnet(name: str) -> tuple[nn.Module, int]:
    try:
        from torchvision import models
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "ResNet vision backbones require torchvision; install policy_runner with the ml extra"
        ) from exc
    builder = getattr(models, name)
    try:
        model = builder(weights=None)
    except TypeError:
        model = builder(pretrained=False)
    feature_dim = int(model.fc.in_features)
    model.fc = nn.Identity()
    return model, feature_dim


def _build_dinov3_plugin() -> tuple[nn.Module, int]:
    module_name = os.environ.get("POLICY_RUNNER_DINOV3_MODULE", "")
    if not module_name:
        raise RuntimeError(
            "dinov3 is a placeholder/plugin backbone. Set POLICY_RUNNER_DINOV3_MODULE "
            "to a module exposing build_backbone()."
        )
    module = importlib.import_module(module_name)
    builder = getattr(module, "build_backbone", None)
    if not callable(builder):
        raise RuntimeError(f"{module_name}.build_backbone() is required for dinov3")
    result = builder()
    if isinstance(result, tuple) and len(result) == 2:
        model, feature_dim = result
    else:
        model = result
        feature_dim = getattr(model, "feature_dim", None) or getattr(model, "embed_dim", None)
    if not isinstance(model, nn.Module) or feature_dim is None:
        raise RuntimeError("dinov3 plugin must return (nn.Module, feature_dim) or expose feature_dim")
    return model, int(feature_dim)
