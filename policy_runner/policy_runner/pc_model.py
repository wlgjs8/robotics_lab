"""Point-cloud conditioned flow-matching policy (Variant A).

A PointNet-style encoder turns each arm's egocentric XYZRGB cloud into a token;
the tokens are fused with the proprio token through a small transformer to form
the flow-matching condition. The rectified-flow velocity head / sampler mirror
:mod:`flow_model` so checkpoints and inference stay conceptually identical.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import torch
from torch import nn

from .flow_dataset import FLOW_ACTION_DIM, FLOW_PROPRIO_DIM
from .flow_model import SinusoidalTimeEmbedding

PC_FEATURE_DIM = 6  # x,y,z,r,g,b
MAX_ARMS = 2


@dataclass(frozen=True)
class PointCloudFlowConfig:
    action_horizon: int
    action_dim: int = FLOW_ACTION_DIM
    proprio_dim: int = FLOW_PROPRIO_DIM
    num_points: int = 4096
    hidden_dim: int = 256
    pc_feature_dim: int = PC_FEATURE_DIM
    condition_encoder: str = "transformer"
    dropout: float = 0.0
    use_proprio: bool = True
    pc_encoder: str = "pointnet"   # pointnet | deep | attn

    def to_dict(self) -> dict[str, Any]:
        return {
            "action_horizon": int(self.action_horizon),
            "action_dim": int(self.action_dim),
            "proprio_dim": int(self.proprio_dim),
            "num_points": int(self.num_points),
            "hidden_dim": int(self.hidden_dim),
            "pc_feature_dim": int(self.pc_feature_dim),
            "condition_encoder": self.condition_encoder,
            "dropout": float(self.dropout),
            "use_proprio": bool(self.use_proprio),
            "pc_encoder": self.pc_encoder,
        }

    @classmethod
    def from_mapping(cls, raw: dict[str, Any]) -> "PointCloudFlowConfig":
        return cls(
            action_horizon=int(raw["action_horizon"]),
            action_dim=int(raw.get("action_dim", FLOW_ACTION_DIM)),
            proprio_dim=int(raw.get("proprio_dim", FLOW_PROPRIO_DIM)),
            num_points=int(raw.get("num_points", 4096)),
            hidden_dim=int(raw.get("hidden_dim", 256)),
            pc_feature_dim=int(raw.get("pc_feature_dim", PC_FEATURE_DIM)),
            condition_encoder=str(raw.get("condition_encoder", "transformer")),
            dropout=float(raw.get("dropout", 0.0)),
            use_proprio=bool(raw.get("use_proprio", True)),
            pc_encoder=str(raw.get("pc_encoder", "pointnet")),
        )


class PointNetEncoder(nn.Module):
    """Permutation-invariant XYZRGB encoder (shared MLP + max/mean pool).

    width: feature dim of the per-point MLP (default 256). deep: extra MLP block.
    """

    def __init__(self, in_dim: int, hidden_dim: int, width: int = 256, deep: bool = False):
        super().__init__()
        self.mlp1 = nn.Sequential(
            nn.Linear(in_dim, 64), nn.GELU(),
            nn.Linear(64, 128), nn.GELU(),
        )
        layers = [nn.Linear(128, width), nn.GELU(), nn.Linear(width, width), nn.GELU()]
        if deep:
            layers += [nn.Linear(width, width), nn.GELU(), nn.Linear(width, width), nn.GELU()]
        self.mlp2 = nn.Sequential(*layers)
        self.head = nn.Sequential(
            nn.Linear(2 * width, hidden_dim), nn.GELU(),
            nn.Linear(hidden_dim, hidden_dim),
        )

    def forward(self, points: torch.Tensor) -> torch.Tensor:
        feat = self.mlp2(self.mlp1(points))            # (B, N, width)
        pooled = torch.cat([feat.amax(dim=1), feat.mean(dim=1)], dim=-1)
        return self.head(pooled)


class PointAttnEncoder(nn.Module):
    """Set-transformer-lite: per-point MLP -> self-attention over a point subset ->
    max/mean pool. Captures point-point spatial relations that PointNet's maxpool loses
    (the scene encoder is the key lever once we go ego/cloud-only)."""

    def __init__(self, in_dim: int, hidden_dim: int, attn_points: int = 256, layers: int = 2, heads: int = 4):
        super().__init__()
        self.attn_points = int(attn_points)
        self.embed = nn.Sequential(
            nn.Linear(in_dim, 128), nn.GELU(), nn.Linear(128, hidden_dim), nn.GELU(),
        )
        enc = nn.TransformerEncoderLayer(
            d_model=hidden_dim, nhead=heads, dim_feedforward=hidden_dim * 4,
            dropout=0.0, activation="gelu", batch_first=True, norm_first=True,
        )
        self.attn = nn.TransformerEncoder(enc, num_layers=layers)
        self.head = nn.Sequential(
            nn.Linear(2 * hidden_dim, hidden_dim), nn.GELU(), nn.Linear(hidden_dim, hidden_dim),
        )

    def forward(self, points: torch.Tensor) -> torch.Tensor:
        f = self.embed(points)                          # (B, N, hidden)
        m = min(self.attn_points, f.shape[1])
        f = self.attn(f[:, :m, :])                      # cloud points are randomly ordered -> random subset
        pooled = torch.cat([f.amax(dim=1), f.mean(dim=1)], dim=-1)
        return self.head(pooled)


def build_point_encoder(kind: str, in_dim: int, hidden_dim: int) -> nn.Module:
    if kind == "pointnet":
        return PointNetEncoder(in_dim, hidden_dim)
    if kind == "deep":
        return PointNetEncoder(in_dim, hidden_dim, width=512, deep=True)
    if kind == "attn":
        return PointAttnEncoder(in_dim, hidden_dim)
    raise ValueError(f"unknown pc_encoder '{kind}'")


class PointCloudFlowPolicy(nn.Module):
    def __init__(self, config: PointCloudFlowConfig):
        super().__init__()
        if config.action_horizon <= 0 or config.action_dim <= 0:
            raise ValueError("action_horizon and action_dim must be positive")
        if config.condition_encoder not in {"transformer", "mlp"}:
            raise ValueError("condition_encoder must be transformer or mlp")
        self.config = config
        hidden = config.hidden_dim

        self.pc_encoder = build_point_encoder(getattr(config, "pc_encoder", "pointnet"), config.pc_feature_dim, hidden)
        self.arm_embedding = nn.Embedding(MAX_ARMS, hidden)
        self.use_proprio = bool(getattr(config, "use_proprio", True))
        if self.use_proprio:
            self.proprio_mlp = nn.Sequential(
                nn.Linear(config.proprio_dim, hidden), nn.GELU(),
                nn.Linear(hidden, hidden),
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
                nn.Linear(hidden, hidden), nn.GELU(), nn.Linear(hidden, hidden),
            )
        self.time_embedding = nn.Sequential(
            SinusoidalTimeEmbedding(hidden),
            nn.Linear(hidden, hidden), nn.GELU(), nn.Linear(hidden, hidden),
        )
        self.step_embedding = nn.Embedding(config.action_horizon, hidden)
        p = float(getattr(config, "dropout", 0.0))
        self.decoder = nn.Sequential(
            nn.Linear(config.action_dim + hidden, hidden), nn.GELU(), nn.Dropout(p),
            nn.Linear(hidden, hidden), nn.GELU(), nn.Dropout(p),
            nn.Linear(hidden, config.action_dim),
        )

    @classmethod
    def from_checkpoint_config(cls, raw: dict[str, Any]) -> "PointCloudFlowPolicy":
        return cls(PointCloudFlowConfig.from_mapping(raw))

    def encode_condition(self, pointcloud: torch.Tensor, proprio: torch.Tensor) -> torch.Tensor:
        # pointcloud: (B, V, N, F); proprio: (B, D)
        if proprio.ndim != 2:
            raise ValueError("proprio must be B,D")
        if pointcloud.ndim != 4:
            raise ValueError("pointcloud must be B,V,N,F")
        batch_size, view_count = pointcloud.shape[0], pointcloud.shape[1]
        tokens = [self.proprio_mlp(proprio).unsqueeze(1)] if self.use_proprio else []
        flat = pointcloud.reshape(batch_size * view_count, pointcloud.shape[2], pointcloud.shape[3])
        pc_tok = self.pc_encoder(flat).reshape(batch_size, view_count, -1)
        arm_ids = torch.arange(view_count, device=pointcloud.device).clamp_max(MAX_ARMS - 1)
        pc_tok = pc_tok + self.arm_embedding(arm_ids)[None, :, :]
        tokens.append(pc_tok)
        token_tensor = torch.cat(tokens, dim=1)
        if self.condition_encoder is not None:
            pooled = self.condition_encoder(token_tensor).mean(dim=1)
        else:
            pooled = token_tensor.mean(dim=1)
        return self.condition_projection(pooled)

    def forward(
        self,
        pointcloud: torch.Tensor,
        proprio: torch.Tensor,
        x_t: torch.Tensor,
        t: torch.Tensor,
    ) -> torch.Tensor:
        if x_t.ndim != 3 or x_t.shape[1] != self.config.action_horizon:
            raise ValueError("x_t must be B,H,A with H matching config")
        return self.decode_with_condition(self.encode_condition(pointcloud, proprio), x_t, t)

    def decode_with_condition(self, cond_base: torch.Tensor, x_t: torch.Tensor, t: torch.Tensor) -> torch.Tensor:
        """Velocity head from a PRE-COMPUTED condition (constant over the flow ODE),
        so the sampler runs the cloud encoder once per chunk, not once per step."""
        cond = cond_base + self.time_embedding(t)
        step_ids = torch.arange(self.config.action_horizon, device=x_t.device)
        step_tokens = self.step_embedding(step_ids)[None, :, :] + cond[:, None, :]
        return self.decoder(torch.cat([x_t, step_tokens], dim=-1))


def pc_flow_matching_loss(model: PointCloudFlowPolicy, batch: dict[str, torch.Tensor]) -> torch.Tensor:
    actions = batch["action_chunk"].float()
    mask = batch.get("action_mask")
    mask = torch.ones_like(actions) if mask is None else mask.float()
    bsz = actions.shape[0]
    t = torch.rand(bsz, dtype=actions.dtype, device=actions.device)
    x0 = torch.randn_like(actions)
    x_t = (1.0 - t[:, None, None]) * x0 + t[:, None, None] * actions
    target_v = actions - x0
    pred_v = model(batch["pointcloud"].float(), batch["proprio"].float(), x_t, t)
    loss = ((pred_v - target_v) ** 2) * mask
    return loss.sum() / mask.sum().clamp_min(1.0)


@torch.no_grad()
def pc_sample_action_chunks(
    model: PointCloudFlowPolicy,
    pointcloud: torch.Tensor,
    proprio: torch.Tensor,
    *,
    steps: int = 16,
    initial_noise: torch.Tensor | None = None,
) -> torch.Tensor:
    if steps <= 0:
        raise ValueError("steps must be positive")
    bsz = proprio.shape[0]
    if initial_noise is None:
        x = torch.zeros(
            (bsz, model.config.action_horizon, model.config.action_dim),
            dtype=proprio.dtype, device=proprio.device,
        )
    else:
        x = initial_noise.to(device=proprio.device, dtype=proprio.dtype)
    dt = 1.0 / float(steps)
    # Cache the condition once (constant over the flow integration) instead of
    # re-running the cloud encoder at every sampling step.
    cond_base = model.encode_condition(pointcloud, proprio)
    for step in range(steps):
        t = torch.full((bsz,), (step + 0.5) / float(steps), dtype=proprio.dtype, device=proprio.device)
        x = x + dt * model.decode_with_condition(cond_base, x, t)
    return x
