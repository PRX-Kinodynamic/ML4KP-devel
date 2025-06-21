import torch
import torch.nn as nn
import torch.nn.functional as F

# ----------------------------------------
# Binary-specific U-Net for Diffusion
# ----------------------------------------
class TimeFiLM(nn.Module):
    def __init__(self, time_emb_dim, feature_dim):
        super().__init__()
        self.time_proj = nn.Linear(time_emb_dim, feature_dim * 2)
        
    def forward(self, x, time_emb):
        time_emb = self.time_proj(time_emb)[:, :, None, None]
        scale, shift = time_emb.chunk(2, dim=1)
        return x * (1 + scale) + shift

class ResidualBlock(nn.Module):
    def __init__(self, in_channels, out_channels, time_emb_dim=128):
        super(ResidualBlock, self).__init__()
        
        # Residual connection
        self.residual_conv = nn.Conv2d(in_channels, out_channels, kernel_size=1) if in_channels != out_channels else nn.Identity()
        
        # Main path
        self.conv1 = nn.Conv2d(in_channels, out_channels, kernel_size=3, padding=1)
        self.norm1 = nn.GroupNorm(8, out_channels)
        self.film1 = TimeFiLM(time_emb_dim, out_channels)
        self.act1 = nn.SiLU()
        
        self.conv2 = nn.Conv2d(out_channels, out_channels, kernel_size=3, padding=1)
        self.norm2 = nn.GroupNorm(8, out_channels)
        self.film2 = TimeFiLM(time_emb_dim, out_channels)
        self.act2 = nn.SiLU()
        
    def forward(self, x, time_emb):
        # Residual branch
        residual = self.residual_conv(x)
        
        # Main branch
        h = self.conv1(x)
        h = self.norm1(h)
        h = self.film1(h, time_emb)
        h = self.act1(h)
        
        h = self.conv2(h)
        h = self.norm2(h)
        h = self.film2(h, time_emb)
        h = h + residual  # Add residual connection
        h = self.act2(h)
        
        return h

class BinaryMaskUNet(nn.Module):
    def __init__(self, in_channels=1, out_channels=1, time_emb_dim=256, base_channels=128, dropout_rate=0.1):
        super(BinaryMaskUNet, self).__init__()
        
        # Increase dimensionality of time embedding
        self.time_embed = nn.Sequential(
            nn.Linear(1, time_emb_dim),
            nn.SiLU(),
            nn.Linear(time_emb_dim, time_emb_dim),
            nn.SiLU(),
            nn.Linear(time_emb_dim, time_emb_dim),
        )
        
        # Initial convolution to process input
        self.init_conv = nn.Conv2d(in_channels, base_channels, kernel_size=3, padding=1)
        
        # Encoder blocks with residual connections
        self.down1 = ResidualBlock(base_channels, base_channels, time_emb_dim)
        self.down2 = ResidualBlock(base_channels, base_channels*2, time_emb_dim)
        self.down3 = ResidualBlock(base_channels*2, base_channels*2, time_emb_dim)
        
        # Downsampling
        self.pool1 = nn.MaxPool2d(2)
        self.pool2 = nn.MaxPool2d(2)
        
        # Middle blocks at lowest resolution
        self.mid1 = ResidualBlock(base_channels*2, base_channels*4, time_emb_dim)
        self.mid2 = ResidualBlock(base_channels*4, base_channels*4, time_emb_dim)
        self.mid3 = ResidualBlock(base_channels*4, base_channels*2, time_emb_dim)
        
        # Upsampling
        self.upsample1 = nn.Upsample(scale_factor=2, mode='bilinear', align_corners=False)
        self.upsample2 = nn.Upsample(scale_factor=2, mode='bilinear', align_corners=False)
        
        # Decoder blocks with skip connections and dropout for regularization
        self.up1 = ResidualBlock(base_channels*4, base_channels*2, time_emb_dim)
        self.up2 = ResidualBlock(base_channels*3, base_channels, time_emb_dim)
        self.up3 = ResidualBlock(base_channels*2, base_channels, time_emb_dim)
        
        # Extra blocks for stability
        self.extra1 = ResidualBlock(base_channels, base_channels, time_emb_dim)
        self.extra2 = ResidualBlock(base_channels, base_channels, time_emb_dim)
        
        # Dropout for better generalization
        self.dropout = nn.Dropout(dropout_rate)
        
        # Final layers with higher capacity
        self.final = nn.Sequential(
            nn.Conv2d(base_channels, base_channels, kernel_size=3, padding=1),
            nn.GroupNorm(8, base_channels),
            nn.SiLU(),
            nn.Dropout(dropout_rate),
            nn.Conv2d(base_channels, out_channels, kernel_size=1),
           
        )
        
    def forward(self, x, t):
        # Time embedding with better scaling for stability
        t = t[:, None].float() / 100.0 # Normalize timestep
        time_emb = self.time_embed(t)
        
        # Initial convolution
        x = self.init_conv(x)
        
        # Encoder path with skip connections
        d1 = self.down1(x, time_emb)
        x = self.pool1(d1)
        
        d2 = self.down2(x, time_emb)
        x = self.pool2(d2)
        
        d3 = self.down3(x, time_emb)
        
        # Middle bottleneck blocks
        x = self.mid1(d3, time_emb)
        x = self.dropout(x)
        x = self.mid2(x, time_emb)
        x = self.dropout(x)
        x = self.mid3(x, time_emb)
        
        # Decoder path with skip connections
        x = self.upsample1(x)
        x = torch.cat([x, d2], dim=1)
        x = self.up1(x, time_emb)
        x = self.dropout(x)
        
        x = self.upsample2(x)
        x = torch.cat([x, d1], dim=1)
        x = self.up2(x, time_emb)
        
        # Extra processing
        x = self.extra1(x, time_emb)
        x = self.extra2(x, time_emb)
        
        # Final layers
        x = self.final(x)
        
        return x

# Rename to match API
SimpleDiffusionModel = BinaryMaskUNet

# ----------------------------------------
# Diffusion noise schedule and helpers
# ----------------------------------------
def linear_beta_schedule(timesteps, s=0.008):
    betas = torch.linspace(s, 0.999, timesteps)
    return betas

def cosine_beta_schedule(timesteps, s=0.008):
    """
    Cosine schedule as proposed in https://openreview.net/forum?id=-NEXDKk8gZ
    Better for binary masks than linear schedule
    """
    steps = timesteps + 1
    x = torch.linspace(0, timesteps, steps)
    alphas_cumprod = torch.cos(((x / timesteps) + s) / (1 + s) * torch.pi * 0.5) ** 2
    alphas_cumprod = alphas_cumprod / alphas_cumprod[0]
    betas = 1 - (alphas_cumprod[1:] / alphas_cumprod[:-1])
    return torch.clamp(betas, 0.0001, 0.9999)

def q_sample(x_start, t, noise, sqrt_alphas_cumprod, sqrt_one_minus_alphas_cumprod):
    return sqrt_alphas_cumprod[t][:, None, None, None] * x_start + \
           sqrt_one_minus_alphas_cumprod[t][:, None, None, None] * noise

def extract(a, t, x_shape):
    batch_size = t.shape[0]
    out = a.gather(-1, t).float()
    return out.reshape(batch_size, *((1,) * (len(x_shape) - 1)))

# ----------------------------------------
# Standard MSE loss function (without BCE)
# ----------------------------------------
def binary_diffusion_loss(pred, target):
    """
    Simple MSE loss for standard diffusion training
    """
    return F.mse_loss(pred, target)

# ----------------------------------------
# Training step
# ----------------------------------------
def train_step(model, x_start, timesteps, optimizer, noise_scheduler, device):
    model.train()

    B = x_start.shape[0]
    t = torch.randint(0, timesteps, (B,), device=device).long()

    noise = torch.randn_like(x_start) # epsilon # dont predict noise? predict the value directly.
    
    x_noisy = q_sample(
        x_start=x_start, t=t, noise=noise,
        sqrt_alphas_cumprod=noise_scheduler['sqrt_alphas_cumprod'],
        sqrt_one_minus_alphas_cumprod=noise_scheduler['sqrt_one_minus_alphas_cumprod']
    )

    predicted_noise = model(x_noisy, t)
    
    # Use MSE loss only
    loss = F.mse_loss(predicted_noise, noise)

    optimizer.zero_grad()
    loss.backward()
    
    # Gradient clipping for stability
    torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
    
    optimizer.step()

    return loss.item()

# ----------------------------------------
# Prepare noise scheduler
# ----------------------------------------
def prepare_noise_schedule(timesteps=1000, device="cuda"):
    # Use cosine schedule instead of linear
    betas = linear_beta_schedule(timesteps).to(device)
    alphas = 1. - betas
    alphas_cumprod = torch.cumprod(alphas, dim=0)
    # alphas_cumprod_prev = torch.cat([torch.ones(1, device=device), alphas_cumprod[:-1]], dim=0)
    
    # sqrt_reciprocal_alphas_cumprod = torch.sqrt(1 / alphas_cumprod)
    # sqrt_reciprocal_m1_alphas_cumprod = torch.sqrt((1 / alphas_cumprod) - 1)
    
    # posterior_mean_coef_1 = betas * torch.sqrt(alphas_cumprod_prev) / (1 - alphas_cumprod)
    # posterior_mean_coef_2 = (1 - alphas_cumprod_prev) * torch.sqrt(alphas) / (1 - alphas_cumprod)
    
    # posterior_variance = betas * (1 - alphas_cumprod_prev) / (1 - alphas_cumprod)
    # posterior_variance = torch.clamp(posterior_variance, min=1e-20)

    return {
        'betas': betas,
        'alphas': alphas,
        'alphas_cumprod': alphas_cumprod,
        # 'alphas_cumprod_prev': alphas_cumprod_prev,
        'sqrt_alphas_cumprod': torch.sqrt(alphas_cumprod),
        'sqrt_one_minus_alphas_cumprod': torch.sqrt(1 - alphas_cumprod),
        # 'sqrt_reciprocal_alphas_cumprod': sqrt_reciprocal_alphas_cumprod,
        # 'sqrt_reciprocal_m1_alphas_cumprod': sqrt_reciprocal_m1_alphas_cumprod,
        # 'posterior_mean_coef_1': posterior_mean_coef_1,
        # 'posterior_mean_coef_2': posterior_mean_coef_2,
        # 'posterior_variance': posterior_variance,
        'timesteps': timesteps
    }

# # ----------------------------------------
# # Early stopping utility
# # ----------------------------------------
# class EarlyStopping:
#     def __init__(self, patience=5, min_delta=0.0):
#         self.patience = patience
#         self.min_delta = min_delta
#         self.best_loss = float('inf')
#         self.counter = 0
#         self.should_stop = False

#     def step(self, val_loss):
#         if val_loss < self.best_loss - self.min_delta:
#             self.best_loss = val_loss
#             self.counter = 0
#         else:
#             self.counter += 1
#             if self.counter >= self.patience:
#                 self.should_stop = True
#         return self.should_stop
