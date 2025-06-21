from diffusers import AutoencoderKL
from torch import nn

class VAE(nn.Module):
    def __init__(self):
        super().__init__()
        self.model = AutoencoderKL(
            in_channels=1,
            out_channels=1,
            latent_channels=4,
            layers_per_block=2,
            down_block_types=["DownEncoderBlock2D"]*4,
            up_block_types=["UpDecoderBlock2D"]*4,
            block_out_channels=[128, 256, 256, 256],
            sample_size=84,
            scaling_factor=0.18215,
            act_fn="silu",
            norm_num_groups=32,
        )
        
    def forward(self, x):
        posterior = self.model.encode(x).latent_dist
        z = posterior.sample()
        x_reconstructed = self.model.decode(z).sample
        return posterior, z, x_reconstructed
    
    def encode(self, x):
        return self.model.encode(x).latent_dist.sample()
    
    def decode(self, z):
        return self.model.decode(z).sample
    
    
    
        
        
        
        
        
        
        
        
        
        