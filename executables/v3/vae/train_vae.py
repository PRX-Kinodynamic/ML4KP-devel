import os
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms
from torchvision.utils import save_image
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
from tqdm import tqdm
import random

from vae_model import VAE

# Define a custom dataset for your mask and condition pairs
class MaskDataset(Dataset):
    def __init__(self, folder_path, split, transform=None):
        """
        Args:
            folder_path (string): Directory with npz files
        """
        self.folder_path = folder_path
        # Get all file names
        self.filenames = [f for f in os.listdir(folder_path) if (f.endswith('.npz') and not f.endswith('00.npz'))]
        
        length = len(self.filenames)
        train_length = int(length * 0.9)
        
        if split == "train":
            self.filenames = self.filenames[:train_length]
        else:
            self.filenames = self.filenames[train_length:]
      
        self.transform = transform
    
    def __len__(self):
        return len(self.filenames)
    
    def __getitem__(self, idx):
        npz_file = np.load(os.path.join(self.folder_path, self.filenames[idx]))
        
        condition_image = npz_file['scene']
        object_mask = npz_file['object_mask']
        
        if self.transform:
            condition_image = self.transform(condition_image)
            object_mask = self.transform(object_mask)
            
        return condition_image, object_mask

# VAE loss function
def vae_loss_function(recon_x, x, mu, log_var, beta=1.0):
    batch_size = recon_x.shape[0]
    
    # Reconstruction loss (binary cross entropy)
    BCE = nn.functional.binary_cross_entropy(recon_x, x, reduction='sum')/batch_size
    
    # KL divergence
    KLD = -0.5 * torch.sum(1 + log_var - mu.pow(2) - log_var.exp()) / batch_size
    
    return BCE + beta * KLD, BCE, KLD

def train(model, train_loader, optimizer, device, epoch, beta=1.0):
    model.train()
    train_loss = 0
    recon_loss = 0
    kl_loss = 0
    
    pbar = tqdm(train_loader)
    for batch_idx, (condition_image, object_mask) in enumerate(pbar):
        condition_image = condition_image.to(device)
        object_mask = object_mask.to(device)
        
        optimizer.zero_grad()
        
        if condition_image.shape[0] == 1:
            continue
            
        # Forward pass - VAE takes condition image as input and outputs mask
        recon_mask, mu, log_var = model(condition_image)
        loss, bce, kld = vae_loss_function(recon_mask, object_mask, mu, log_var, beta)
        
        loss.backward()
        optimizer.step()
        
        train_loss += loss.item()
        recon_loss += bce.item()
        kl_loss += kld.item()
        
        # Update progress bar
        pbar.set_description(f"Epoch {epoch} [Train] Loss: {loss.item()/len(object_mask):.4f}")
        
    avg_loss = train_loss / len(train_loader.dataset)
    avg_recon = recon_loss / len(train_loader.dataset)
    avg_kl = kl_loss / len(train_loader.dataset)
    
    print(f"Epoch {epoch}: Train Loss: {avg_loss:.4f}, Recon: {avg_recon:.4f}, KL: {avg_kl:.4f}")
    
    return avg_loss

def evaluate(model, val_loader, device, epoch, beta=1.0):
    model.eval()
    val_loss = 0
    recon_loss = 0
    kl_loss = 0
    
    with torch.no_grad():
        for condition_image, object_mask in val_loader:
            condition_image = condition_image.to(device)
            object_mask = object_mask.to(device)
            
            recon_mask, mu, log_var = model(condition_image)
            loss, bce, kld = vae_loss_function(recon_mask, object_mask, mu, log_var, beta)
            
            val_loss += loss.item()
            recon_loss += bce.item()
            kl_loss += kld.item()
    
    avg_loss = val_loss / len(val_loader.dataset)
    avg_recon = recon_loss / len(val_loader.dataset)
    avg_kl = kl_loss / len(val_loader.dataset)
    
    print(f"Epoch {epoch}: Val Loss: {avg_loss:.4f}, Recon: {avg_recon:.4f}, KL: {avg_kl:.4f}")
    
    # Save some validation reconstructions for visualization
    if epoch % 5 == 0:
        with torch.no_grad():
            condition_image, object_mask = next(iter(val_loader))
            condition_image = condition_image.to(device)
            object_mask = object_mask.to(device)
            
            recon_mask, _, _ = model(condition_image)
            
            # Save the reconstructions
            n = min(8, object_mask.size(0))
            comparison = torch.cat([object_mask[:n], recon_mask[:n]])
            save_image(comparison.cpu(), f'vae_reconstruction_epoch_{epoch}.png', nrow=n)
    
    return avg_loss

def generate_samples(model, device, num_samples=5):
    """
    Generate mask samples from the latent space
    
    Args:
        model: trained VAE model
        device: device to run inference on
        num_samples: number of different masks to generate
        
    Returns:
        List of generated masks
    """
    model.eval()
    with torch.no_grad():
        # Sample from prior distribution (standard normal)
        z = torch.randn(num_samples, model.fc_mu.out_features, device=device)
        
        # Decode
        masks = model.decode(z)
    
    return masks

def main():
    # Hyperparameters
    batch_size = 32
    learning_rate = 1e-3
    epochs = 100
    latent_dim = 128
    beta = 0.1  # KL weight
    
    # Device
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # Data paths (replace with your actual paths)
    folder_path = "/common/users/dm1487/namo_data/env_config_1_tr_images"
    
    # Data transformations
    transform = transforms.Compose([
        transforms.ToTensor(),
    ])
    
    # Create datasets and dataloaders
    train_dataset = MaskDataset(folder_path=folder_path, split="train", transform=transform)
    val_dataset = MaskDataset(folder_path=folder_path, split="val", transform=transform)
    
    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True, num_workers=4)
    val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False, num_workers=4)
    
    # Initialize model
    model = VAE(latent_dim=latent_dim).to(device)
    
    # Print model information
    print(f"Total trainable parameters: {model.count_parameters():,}")
    
    # Optimizer
    optimizer = optim.AdamW(model.parameters(), lr=learning_rate)
    
    # Training loop
    train_losses = []
    val_losses = []
    
    for epoch in range(1, epochs + 1):
        # Train
        train_loss = train(model, train_loader, optimizer, device, epoch, beta)
        train_losses.append(train_loss)
        
        # Evaluate
        val_loss = evaluate(model, val_loader, device, epoch, beta)
        val_losses.append(val_loss)
        
        # Generate samples every 10 epochs
        if epoch % 10 == 0:
            samples = generate_samples(model, device, num_samples=5)
            save_image(samples.cpu(), f'vae_samples_epoch_{epoch}.png', nrow=5)
        
        # Save model checkpoint every 10 epochs
        if epoch % 10 == 0:
            torch.save(model.state_dict(), f"vae_model_epoch_{epoch}.pt")
    
    # Save final model
    torch.save(model.state_dict(), "vae_model_final.pt")
    
    # Plot training curves
    plt.figure(figsize=(10, 5))
    plt.plot(range(1, epochs + 1), train_losses, label='Train Loss')
    plt.plot(range(1, epochs + 1), val_losses, label='Validation Loss')
    plt.xlabel('Epochs')
    plt.ylabel('Loss')
    plt.legend()
    plt.savefig('vae_training_curve.png')
    plt.close()
    
    # Load best model and generate final samples
    model.load_state_dict(torch.load("vae_model_final.pt"))
    
    # Generate some final samples
    print("Generating final samples...")
    final_samples = generate_samples(model, device, num_samples=10)
    save_image(final_samples.cpu(), 'vae_final_samples.png', nrow=5)
    
    # Also show some reconstructions from validation set
    with torch.no_grad():
        condition_images, object_masks = next(iter(val_loader))
        condition_images = condition_images.to(device)
        object_masks = object_masks.to(device)
        
        recon_masks, _, _ = model(condition_images)
        
        n = min(8, object_masks.size(0))
        comparison = torch.cat([object_masks[:n], recon_masks[:n]])
        save_image(comparison.cpu(), 'vae_final_reconstructions.png', nrow=n)
    
    print("Training and evaluation completed!")

if __name__ == "__main__":
    main()
