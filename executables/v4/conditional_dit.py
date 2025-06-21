# ===== SequenceMinimalDiT: 1-to-1 with HF DiTTransformer2DModel core =====
# Minimal DiT core adapted to a sequence of N patches (8×8×4) with alternating cross-attention
# Key components:
#  - conv_in: patch embedding
#  - pos_embed: 1D positional encoding
#  - time_mlp: timestep embedding MLP
#  - scene_encoder: pretrained ResNet-18 backbone (layers conv1..layer3)
#  - blocks: BasicTransformerBlock layers alternating self- and cross-attention
#  - conv_out: reconstruct patches

import torch
import torch.nn as nn
import math
from diffusers import AutoencoderKL
from diffusers.models.attention import BasicTransformerBlock
from diffusers.models.normalization import AdaLayerNorm
from torchvision.models import resnet18  # pretrained scene encoder backbone

class SinusoidalTimeEmbedding(nn.Module):
    """Sinusoidal embedding for diffusion timesteps, as in HF DiT."""
    def __init__(self, dim: int):
        super().__init__()
        self.dim = dim
    def forward(self, t: torch.Tensor) -> torch.Tensor:
        half = self.dim // 2
        freq = torch.exp(-math.log(10000.0) * torch.arange(half, device=t.device) / (half - 1))
        args = t.float().unsqueeze(1) * freq.unsqueeze(0)  # [B, half]
        emb = torch.cat([torch.sin(args), torch.cos(args)], dim=-1)
        if self.dim % 2:
            emb = torch.cat([emb, emb[:, :1]], dim=-1)
        return emb  # [B, dim]

class PositionalEncoding(nn.Module):
    """Absolute sinusoidal positional embeddings over sequence length."""
    def __init__(self, d_model: int, max_len: int = 16):
        super().__init__()
        pe = torch.zeros(max_len, d_model)
        pos = torch.arange(0, max_len).unsqueeze(1).float()
        div = torch.exp(torch.arange(0, d_model, 2).float() * -(math.log(10000.0) / d_model))
        pe[:, 0::2] = torch.sin(pos * div)
        pe[:, 1::2] = torch.cos(pos * div)
        pe = pe.unsqueeze(0)  # [1, max_len, d_model]
        self.register_buffer('pe', pe)
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        seq_len = x.size(1)
        return x + self.pe[:, :seq_len]

class SequenceMinimalDiT(nn.Module):
    """
    Denoiser for a sequence of N patches (8x8x4), conditioned on a scene via alternating cross-attention.

    Inputs:
      x:     torch.Tensor [B, N, 4, 8, 8]  # noisy patches
      t:     torch.Tensor [B]              # diffusion timesteps
      scene: torch.Tensor [B, 3, 224, 224] # scene image
    Output:
      torch.Tensor [B, N, 4, 8, 8]         # denoised patches
    """
    def __init__(
        self,
        in_channels: int = 4,
        hidden_size: int = 256,
        num_layers: int = 6,
        num_heads: int = 8,
        head_dim: int = 32,
        mlp_ratio: float = 4.0,
        dropout: float = 0.1,
        max_seq_len: int = 16
    ):
        super().__init__()
        # 1) Patch embedding
        self.conv_in = nn.Conv2d(in_channels, hidden_size, kernel_size=8, stride=8)
        # 2) Positional encoding
        self.pos_embed = PositionalEncoding(hidden_size, max_len=max_seq_len)
        # 3) Timestep embedding MLP
        self.time_mlp = nn.Sequential(
            SinusoidalTimeEmbedding(hidden_size),
            nn.Linear(hidden_size, hidden_size),
            nn.GELU(),
            nn.Linear(hidden_size, hidden_size)
        )
        # 4) Scene encoder: pretrained ResNet-18 up to layer3 -> [B, hidden_size, 14, 14]
        backbone = resnet18(pretrained=True)
        self.scene_encoder = nn.Sequential(
            backbone.conv1,
            backbone.bn1,
            backbone.relu,
            backbone.maxpool,
            backbone.layer1,
            backbone.layer2,
            backbone.layer3,
        )
        # Freeze scene encoder
        for param in self.scene_encoder.parameters():
            param.requires_grad = False
        self.scene_encoder.eval()
            
        self.scene_adapter = nn.Sequential(
            nn.Conv2d(hidden_size, hidden_size, kernel_size=1),
            nn.GELU(),
            nn.Conv2d(hidden_size, hidden_size, kernel_size=1)
        )
        
        # 5) Transformer blocks: alternate self-attn and cross-attn
        self.blocks = nn.ModuleList()
        for i in range(num_layers):
            cross_dim = hidden_size if (i % 2 == 1) else None
            self.blocks.append(
                BasicTransformerBlock(
                    dim=hidden_size,
                    num_attention_heads=num_heads,
                    attention_head_dim=head_dim,
                    mlp_ratio=mlp_ratio,
                    dropout=dropout,
                    norm_layer=AdaLayerNorm,
                    norm_type="ada_norm_zero",
                    qk_norm=False,
                    use_dual_attention=False,
                    cross_attention_dim=cross_dim,
                )
            )
        # 6) Reconstruction
        self.conv_out = nn.Conv2d(hidden_size, in_channels, kernel_size=1, stride=1)

    def forward(self, x: torch.Tensor, t: torch.Tensor, scene_embeddings: torch.Tensor) -> torch.Tensor:
        B, N, C, H, W = x.shape
        # A) Patch tokens
        x_flat = x.view(B * N, C, H, W)
        h = self.conv_in(x_flat)           # [B*N, hidden_size, 1, 1]
        tokens = h.view(B, N, -1)          # [B, N, hidden_size]
        # B) Positional encoding
        tokens = self.pos_embed(tokens)
        # C) Time embedding
        te = self.time_mlp(t)              # [B, hidden_size]
        # E) Transformer with alternating cross-attention
        out = tokens
        for blk in self.blocks:
            if blk.cross_attention_dim is not None:
                out, _ = blk(out, te, encoder_hidden_states=scene_embeddings)
            else:
                out, _ = blk(out, te)
        # F) Reconstruct patches
        out_flat = out.view(B * N, -1).unsqueeze(-1).unsqueeze(-1)
        recon = self.conv_out(out_flat)    # [B*N, 4, 1, 1]
        return recon.view(B, N, C, H, W)
    

    def forward_scene_encoder(self, scene: torch.Tensor) -> torch.Tensor:
        F = self.scene_encoder(scene)      # [B, hidden_size, 14, 14]
        F = self.scene_adapter(F)          # adapt features to your custom RGB scene
        S = F.flatten(2).transpose(1,2)    # [B, 196, hidden_size]
        return S







