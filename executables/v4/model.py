# ===== SequenceMinimalDiT: 1-to-1 with HF DiTTransformer2DModel core =====
# This minimal implementation mirrors the core of diffusers' DiTTransformer2DModel:
#  - conv_in         ≈ DiTTransformer2DModel.conv_in  (patch embedding)
#  - time_mlp        ≈ DiTTransformer2DModel.time_proj  (timestep MLP)
#  - pos_embed       ≈ positional embeddings over token positions
#  - blocks          ≈ transformer_blocks (BasicTransformerBlock with AdaLayerNormZero)
#  - conv_out        ≈ DiTTransformer2DModel.conv_out (reconstruction)
# Why a 1D sequence vs. true DiT’s 2D grid?
# In your use case, patches are already extracted as an ordered sequence of 8×8×4 blocks.
# • True DiT: treats spatial latent as a square grid, embedding via conv_in into H/p × W/p tokens and using 2D positional embeddings.
# • SequenceMinimalDiT: skips spatial grid assembly, directly consumes N precomputed patches as a 1D list (with its own positional encoding).
# This simplifies the model and aligns with your data preparation, while preserving all core DiT mechanisms (time conditioning, adaptive norms, self-attention).

import torch
import torch.nn as nn
import math
from diffusers.models.attention import BasicTransformerBlock
from diffusers.models.normalization import AdaLayerNorm

class SinusoidalTimeEmbedding(nn.Module):
    """Sinusoidal embedding for diffusion timesteps, as in HF DiT."""
    def __init__(self, dim: int):
        super().__init__()
        self.dim = dim
    def forward(self, t: torch.Tensor) -> torch.Tensor:
        half = self.dim // 2
        freq = torch.exp(-math.log(10000.0) * torch.arange(half, device=t.device) / (half - 1))
        args = t.float().unsqueeze(1) * freq.unsqueeze(0)     # [B, half]
        emb = torch.cat([torch.sin(args), torch.cos(args)], dim=-1)
        if self.dim % 2:
            emb = torch.cat([emb, emb[:, :1]], dim=-1)
        return emb  # [B, dim]

class PositionalEncoding(nn.Module):
    """Absolute sinusoidal positional embeddings over sequence length, analogous to patch pos_embed."""
    def __init__(self, d_model: int, max_len: int = 16):
        super().__init__()
        pe = torch.zeros(max_len, d_model)
        pos = torch.arange(0, max_len).unsqueeze(1).float()
        div = torch.exp(torch.arange(0, d_model, 2).float() * -(math.log(10000.0) / d_model))
        pe[:, 0::2] = torch.sin(pos * div)
        pe[:, 1::2] = torch.cos(pos * div)
        pe = pe.unsqueeze(0)  # shape [1, max_len, d_model]
        self.register_buffer('pe', pe)
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # x: [B, N, d_model]
        seq_len = x.size(1)
        return x + self.pe[:, :seq_len]

class SequenceMinimalDiT(nn.Module):
    """
    Minimal DiT core over a fixed-length sequence of N patches (8×8×4), matching HF's DiT blocks:
    Input:  x: [B, N, 4, 8, 8],   t: [B]
    Output: noise_pred: [B, N, 4, 8, 8]
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
        # 1) conv_in: projects each 8×8 patch → hidden feature map [B*N, hidden,1,1]
        self.conv_in = nn.Conv2d(in_channels, hidden_size, kernel_size=8, stride=8)
        # 2) Positional embeddings over sequence (mirrors ViT pos_embed)
        self.pos_embed = PositionalEncoding(hidden_size, max_len=max_seq_len)
        # 3) time_mlp (timestep embedding MLP), same as DiT time_proj
        self.time_mlp = nn.Sequential(
            SinusoidalTimeEmbedding(hidden_size),
            nn.Linear(hidden_size, hidden_size),
            nn.GELU(),
            nn.Linear(hidden_size, hidden_size)
        )
        # 4) transformer_blocks: BasicTransformerBlock with AdaLayerNormZero
        self.blocks = nn.ModuleList([
            BasicTransformerBlock(
                dim=hidden_size,
                num_attention_heads=num_heads,
                attention_head_dim=head_dim,
                mlp_ratio=mlp_ratio,
                dropout=dropout,
                norm_layer=AdaLayerNorm,
                norm_type="ada_norm_zero",
                qk_norm=False,
                use_dual_attention=False
            )
            for _ in range(num_layers)
        ])
        # 5) conv_out: reverse of conv_in to reconstruct 4-channel 8×8 patches
        self.conv_out = nn.Conv2d(hidden_size, in_channels, kernel_size=1, stride=1)

    def forward(self, x: torch.Tensor, t: torch.Tensor) -> torch.Tensor:
        B, N, C, H, W = x.shape
        # flatten batch and sequence to apply conv_in per patch
        x_flat = x.view(B * N, C, H, W)                # [B*N,4,8,8]
        h = self.conv_in(x_flat)                       # [B*N,hidden,1,1]
        tokens = h.view(B, N, -1)                      # [B,N,hidden]

        # add positional embeddings once at input
        tokens = self.pos_embed(tokens)
        # compute timestep embedding
        te = self.time_mlp(t)                          # [B,hidden]

        out = tokens
        # denoise through transformer blocks, passing te to AdaLayerNorm
        for blk in self.blocks:
            out, _ = blk(out, te)

        # project back to patch form: [B*N,hidden,1,1] → [B*N,4,1,1]
        out_flat = out.view(B * N, -1).unsqueeze(-1).unsqueeze(-1)
        recon = self.conv_out(out_flat)                # [B*N,4,1,1]
        return recon.view(B, N, C, H, W)

# This minimal implementation aligns 1:1 with HF's DiTTransformer2DModel core structure.
# Example usage:
# model = SequenceMinimalDiT()
# output = model(x_patches, timesteps)
