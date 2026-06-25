"""Variant B: RGB-D dual-branch flow-matching policy.

Per arm we fuse two views of the same wrist RealSense frame:
  * a 2D RGB branch (DINOv3 ConvNeXt / ResNet image encoder) for appearance —
    important here because the task sorts bolts by COLOR (gray vs black);
  * a 3D PointNet branch over the egocentric XYZRGB cloud for metric geometry.
Tokens (proprio + per-arm image + per-arm cloud) go through a small transformer
to form the flow-matching condition. Action stays ee_local with ABSOLUTE gripper.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import torch
from torch import nn

from .flow_dataset import FLOW_ACTION_DIM, FLOW_PROPRIO_DIM
from .flow_model import SinusoidalTimeEmbedding, VisionBackbone
from .pc_model import MAX_ARMS, PC_FEATURE_DIM, PointNetEncoder


class SpatialVisionTokens(nn.Module):
    """Image encoder that keeps the spatial feature map as a set of tokens.

    Unlike VisionBackbone (which global-average-pools to one vector per view, so
    the policy knows WHAT is in view but not WHERE), this returns the H*W trunk
    cells projected to `hidden` so the condition transformer can attend over
    location -- the signal a manipulation policy needs to servo toward the bolt.
    Supports the DINOv3 ConvNeXt trunks (pretrained weights loaded by
    flow_model._build_dinov3_convnext) and torchvision ResNets; finetunes when
    not frozen.
    """

    def __init__(self, name: str, hidden_dim: int, *, frozen: bool = True):
        super().__init__()
        from .flow_model import _build_dinov3_convnext

        self.name = str(name)
        self.frozen = bool(frozen)
        if self.name.startswith("dinov3_convnext_"):
            model, feature_dim = _build_dinov3_convnext(self.name)
            # torchvision ConvNeXt: model.features -> (B, C, H, W) before avgpool.
            self.trunk = model.features
        else:
            raise ValueError(
                f"image_head=spatial currently supports dinov3_convnext_* only, got '{self.name}'"
            )
        self.proj = nn.Linear(int(feature_dim), hidden_dim)
        self.norm = nn.LayerNorm(hidden_dim)
        if self.frozen:
            for p in self.trunk.parameters():
                p.requires_grad = False

    def forward(self, images: torch.Tensor) -> torch.Tensor:
        # images: (B, 3, H, W) -> tokens (B, P, hidden) with P = H'*W' trunk cells.
        if images.ndim != 4:
            raise ValueError("images must be B,C,H,W")
        if self.frozen:
            self.trunk.eval()
        with torch.set_grad_enabled(self.training and not self.frozen):
            fmap = self.trunk(images)
        if fmap.ndim != 4:
            raise ValueError("spatial trunk must emit B,C,H,W")
        b, c, h, w = fmap.shape
        tokens = fmap.flatten(2).transpose(1, 2)  # (B, H*W, C)
        return self.norm(self.proj(tokens))


@dataclass(frozen=True)
class PointCloudRGBFlowConfig:
    action_horizon: int
    action_dim: int = FLOW_ACTION_DIM
    proprio_dim: int = FLOW_PROPRIO_DIM
    num_points: int = 4096
    hidden_dim: int = 256
    pc_feature_dim: int = PC_FEATURE_DIM
    num_cameras: int = MAX_ARMS
    vision_backbone: str = "dinov3_convnext_base"
    frozen_vision: bool = True
    condition_encoder: str = "transformer"
    dropout: float = 0.0
    use_pointcloud: bool = True  # False -> RGB-only (image + proprio, no 3D branch)
    use_proprio: bool = True     # False -> drop proprio token (pure ego vision)
    # image_head: how the ConvNeXt/ResNet feature map becomes condition tokens.
    #   "gap"     -> 1 globally-average-pooled token per view (loses WHERE; appearance only)
    #   "spatial" -> the H*W feature-map cells become tokens so the condition transformer
    #                can attend over location (needed for visual servoing / localizing the bolt)
    image_head: str = "gap"

    def to_dict(self) -> dict[str, Any]:
        return {
            "variant": "b",
            "action_horizon": int(self.action_horizon),
            "action_dim": int(self.action_dim),
            "proprio_dim": int(self.proprio_dim),
            "num_points": int(self.num_points),
            "hidden_dim": int(self.hidden_dim),
            "pc_feature_dim": int(self.pc_feature_dim),
            "num_cameras": int(self.num_cameras),
            "vision_backbone": self.vision_backbone,
            "frozen_vision": bool(self.frozen_vision),
            "condition_encoder": self.condition_encoder,
            "dropout": float(self.dropout),
            "use_pointcloud": bool(self.use_pointcloud),
            "use_proprio": bool(self.use_proprio),
            "image_head": str(self.image_head),
        }

    @classmethod
    def from_mapping(cls, raw: dict[str, Any]) -> "PointCloudRGBFlowConfig":
        return cls(
            action_horizon=int(raw["action_horizon"]),
            action_dim=int(raw.get("action_dim", FLOW_ACTION_DIM)),
            proprio_dim=int(raw.get("proprio_dim", FLOW_PROPRIO_DIM)),
            num_points=int(raw.get("num_points", 4096)),
            hidden_dim=int(raw.get("hidden_dim", 256)),
            pc_feature_dim=int(raw.get("pc_feature_dim", PC_FEATURE_DIM)),
            num_cameras=int(raw.get("num_cameras", MAX_ARMS)),
            vision_backbone=str(raw.get("vision_backbone", "dinov3_convnext_base")),
            frozen_vision=bool(raw.get("frozen_vision", True)),
            condition_encoder=str(raw.get("condition_encoder", "transformer")),
            dropout=float(raw.get("dropout", 0.0)),
            use_pointcloud=bool(raw.get("use_pointcloud", True)),
            use_proprio=bool(raw.get("use_proprio", True)),
            image_head=str(raw.get("image_head", "gap")),
        )


class PointCloudRGBFlowPolicy(nn.Module):
    def __init__(self, config: PointCloudRGBFlowConfig):
        super().__init__()
        if config.condition_encoder not in {"transformer", "mlp"}:
            raise ValueError("condition_encoder must be transformer or mlp")
        self.config = config
        hidden = config.hidden_dim

        self.image_head = str(getattr(config, "image_head", "gap"))
        if self.image_head == "spatial":
            self.vision = SpatialVisionTokens(config.vision_backbone, hidden, frozen=config.frozen_vision)
        else:
            self.vision = VisionBackbone(config.vision_backbone, hidden, frozen=config.frozen_vision)
        self.camera_embedding = nn.Embedding(max(config.num_cameras, 1), hidden)
        self.use_pointcloud = bool(config.use_pointcloud)
        if self.use_pointcloud:
            self.pc_encoder = PointNetEncoder(config.pc_feature_dim, hidden)
            self.arm_embedding = nn.Embedding(MAX_ARMS, hidden)
        self.use_proprio = bool(getattr(config, "use_proprio", True))
        if self.use_proprio:
            self.proprio_mlp = nn.Sequential(
                nn.Linear(config.proprio_dim, hidden), nn.GELU(), nn.Linear(hidden, hidden)
            )
        if config.condition_encoder == "transformer":
            heads = max(1, min(8, hidden // 32))
            layer = nn.TransformerEncoderLayer(
                d_model=hidden, nhead=heads, dim_feedforward=hidden * 4,
                dropout=0.0, activation="gelu", batch_first=True, norm_first=True,
            )
            self.condition_encoder = nn.TransformerEncoder(layer, num_layers=2)
            self.condition_projection = nn.LayerNorm(hidden)
        else:
            self.condition_encoder = None
            self.condition_projection = nn.Sequential(
                nn.Linear(hidden, hidden), nn.GELU(), nn.Linear(hidden, hidden)
            )
        self.time_embedding = nn.Sequential(
            SinusoidalTimeEmbedding(hidden), nn.Linear(hidden, hidden), nn.GELU(), nn.Linear(hidden, hidden)
        )
        self.step_embedding = nn.Embedding(config.action_horizon, hidden)
        p = float(getattr(config, "dropout", 0.0))
        self.decoder = nn.Sequential(
            nn.Linear(config.action_dim + hidden, hidden), nn.GELU(), nn.Dropout(p),
            nn.Linear(hidden, hidden), nn.GELU(), nn.Dropout(p),
            nn.Linear(hidden, config.action_dim),
        )

    @classmethod
    def from_checkpoint_config(cls, raw: dict[str, Any]) -> "PointCloudRGBFlowPolicy":
        return cls(PointCloudRGBFlowConfig.from_mapping(raw))

    def encode_condition(self, pointcloud, image, proprio) -> torch.Tensor:
        if pointcloud.ndim != 4 or image.ndim != 5 or proprio.ndim != 2:
            raise ValueError("expect pointcloud(B,V,N,F), image(B,V,3,H,W), proprio(B,D)")
        b, v = pointcloud.shape[0], pointcloud.shape[1]
        tokens = [self.proprio_mlp(proprio).unsqueeze(1)] if self.use_proprio else []
        # image branch
        img_flat = image.reshape(b * v, *image.shape[2:])
        cam_ids = torch.arange(v, device=image.device).clamp_max(self.camera_embedding.num_embeddings - 1)
        if self.image_head == "spatial":
            # (b*v, P, hidden) spatial tokens -> (b, v*P, hidden); tag each view's
            # patches with that view's camera embedding so geometry stays per-arm.
            img_tok = self.vision(img_flat)
            p = img_tok.shape[1]
            img_tok = img_tok.reshape(b, v * p, -1)
            cam_emb = self.camera_embedding(cam_ids).repeat_interleave(p, dim=0)  # (v*P, hidden)
            tokens.append(img_tok + cam_emb[None, :, :])
        else:
            img_tok = self.vision(img_flat).reshape(b, v, -1)
            tokens.append(img_tok + self.camera_embedding(cam_ids)[None, :, :])
        # point-cloud branch (skipped for RGB-only)
        if self.use_pointcloud:
            pc_flat = pointcloud.reshape(b * v, pointcloud.shape[2], pointcloud.shape[3])
            pc_tok = self.pc_encoder(pc_flat).reshape(b, v, -1)
            arm_ids = torch.arange(v, device=pointcloud.device).clamp_max(MAX_ARMS - 1)
            tokens.append(pc_tok + self.arm_embedding(arm_ids)[None, :, :])

        token_tensor = torch.cat(tokens, dim=1)
        if self.condition_encoder is not None:
            pooled = self.condition_encoder(token_tensor).mean(dim=1)
        else:
            pooled = token_tensor.mean(dim=1)
        return self.condition_projection(pooled)

    def forward(self, pointcloud, image, proprio, x_t, t) -> torch.Tensor:
        if x_t.ndim != 3 or x_t.shape[1] != self.config.action_horizon:
            raise ValueError("x_t must be B,H,A with H matching config")
        cond = self.encode_condition(pointcloud, image, proprio) + self.time_embedding(t)
        step_ids = torch.arange(self.config.action_horizon, device=x_t.device)
        step_tokens = self.step_embedding(step_ids)[None, :, :] + cond[:, None, :]
        return self.decoder(torch.cat([x_t, step_tokens], dim=-1))


def pc_b_flow_matching_loss(model: PointCloudRGBFlowPolicy, batch: dict[str, torch.Tensor]) -> torch.Tensor:
    actions = batch["action_chunk"].float()
    mask = batch.get("action_mask")
    mask = torch.ones_like(actions) if mask is None else mask.float()
    bsz = actions.shape[0]
    t = torch.rand(bsz, dtype=actions.dtype, device=actions.device)
    x0 = torch.randn_like(actions)
    x_t = (1.0 - t[:, None, None]) * x0 + t[:, None, None] * actions
    target_v = actions - x0
    pred_v = model(batch["pointcloud"].float(), batch["image"].float(), batch["proprio"].float(), x_t, t)
    loss = ((pred_v - target_v) ** 2) * mask
    return loss.sum() / mask.sum().clamp_min(1.0)


@torch.no_grad()
def pc_b_sample_action_chunks(model, pointcloud, image, proprio, *, steps: int = 16, initial_noise=None) -> torch.Tensor:
    if steps <= 0:
        raise ValueError("steps must be positive")
    bsz = proprio.shape[0]
    if initial_noise is None:
        x = torch.zeros((bsz, model.config.action_horizon, model.config.action_dim),
                        dtype=proprio.dtype, device=proprio.device)
    else:
        x = initial_noise.to(device=proprio.device, dtype=proprio.dtype)
    dt = 1.0 / float(steps)
    for step in range(steps):
        t = torch.full((bsz,), (step + 0.5) / float(steps), dtype=proprio.dtype, device=proprio.device)
        x = x + dt * model(pointcloud, image, proprio, x, t)
    return x
