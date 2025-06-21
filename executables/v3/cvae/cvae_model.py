import torch
import torch.nn as nn
import torch.nn.functional as F

class CVAE(nn.Module):
    def __init__(self, latent_dim=128):
        super(CVAE, self).__init__()
        
        # Encoder
        self.encoder = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=4, stride=2, padding=1),  # 112x112
            nn.BatchNorm2d(32),
            nn.ReLU(),
            nn.Conv2d(32, 32, kernel_size=4, stride=2, padding=1),  # 56x56
            nn.BatchNorm2d(32),
            nn.ReLU(),
            nn.Conv2d(32, 32, kernel_size=4, stride=2, padding=1),  # 28x28
            nn.BatchNorm2d(32),
            nn.ReLU(),
            nn.Conv2d(32, 8, kernel_size=4, stride=2, padding=1),  # 14x14
            nn.BatchNorm2d(8),
            nn.ReLU(),
            nn.Flatten()
        )
        
        # Calculate flattened size
        self.flatten_size = 8 * 14 * 14
        
        # Mu and log variance layers
        self.fc_mu = nn.Linear(self.flatten_size, latent_dim)
        self.fc_var = nn.Linear(self.flatten_size, latent_dim)
        
        # Instead of concatenating the full condition, we can encode it first
        self.condition_encoder = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=4, stride=2, padding=1),
            nn.BatchNorm2d(32),
            nn.ReLU(),
            nn.Conv2d(32, 16, kernel_size=4, stride=2, padding=1),
            nn.BatchNorm2d(16),
            nn.ReLU(),
            nn.Conv2d(16, 8, kernel_size=4, stride=2, padding=1),
            nn.BatchNorm2d(8),
            nn.ReLU(),
            nn.Conv2d(8, 4, kernel_size=4, stride=2, padding=1),
            nn.BatchNorm2d(4),
            nn.ReLU(),
            nn.Flatten(),
            nn.Linear(4 * 14 * 14, 128),
            nn.BatchNorm1d(128),  # 1D BatchNorm for the linear layer
            nn.ReLU()
        )
        
        # Decoder now takes smaller input
        self.decoder_linear = nn.Linear(128, 32 * 14 * 14)
        
        self.decoder = nn.Sequential(
            nn.ConvTranspose2d(32, 32, kernel_size=4, stride=2, padding=1),  # 28x28
            nn.BatchNorm2d(32),
            nn.ReLU(),
            nn.ConvTranspose2d(32, 32, kernel_size=4, stride=2, padding=1),  # 56x56
            nn.BatchNorm2d(32),
            nn.ReLU(),
            nn.ConvTranspose2d(32, 32, kernel_size=4, stride=2, padding=1),  # 112x112
            nn.BatchNorm2d(32),
            nn.ReLU(),
            nn.ConvTranspose2d(32, 1, kernel_size=4, stride=2, padding=1),  # 224x224
            nn.Sigmoid()  # No BatchNorm before Sigmoid
        )
        
    def encode(self, x):
        x = self.encoder(x)
        mu = self.fc_mu(x)
        log_var = self.fc_var(x)
        return mu, log_var
    
    def reparameterize(self, mu, log_var):
        std = torch.exp(0.5 * log_var)
        eps = torch.randn_like(std)
        return mu + eps * std
    
    def decode(self, z, condition):
        # Encode condition to a more compact representation
        # condition_encoded = self.condition_encoder(condition)
        
        # Concatenate latent vector with encoded condition
        # z_cond = torch.cat([z, condition_encoded], dim=1)
        z_cond = z # + condition_encoded
        # Decode
        x = self.decoder_linear(z_cond)
        x = x.view(-1, 32, 14, 14)
        x = self.decoder(x)
        return x
    
    def forward(self, x, condition):
        mu, log_var = self.encode(x)
        z = self.reparameterize(mu, log_var)
        return self.decode(z, condition), mu, log_var

    def count_parameters(self):
        return sum(p.numel() for p in self.parameters() if p.requires_grad)

# Example usage:
# model = CVAE()
# model = model.to("cuda")
# # print(model.get_trainable_parameters())
# mask = torch.randn(2, 1, 224, 224).to("cuda")  # Batch size 1, 3 channels, 224x224
# input_image = torch.randn(2, 3, 224, 224).to("cuda")  # Batch size 1, 3 channels, 224x224
# reconstructed_mask, mu, log_var = model(mask, input_image)
# print("reconstructed_mask.shape", reconstructed_mask.shape)

# print(model)
# print(f"Total trainable parameters: {model.count_parameters():,}")
