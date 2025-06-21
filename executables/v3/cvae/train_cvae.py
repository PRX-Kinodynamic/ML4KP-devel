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

from cvae_model import CVAE

# Define a custom dataset for your mask and condition pairs
class MaskDataset(Dataset):
    def __init__(self, folder_path, split, transform=None):
        """
        Args:
            folder_path (string): Directory with npz files
        """
        self.folder_path = folder_path
        # Get all file names (assuming masks and conditions have same filenames)
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
        
        # goal_mask = npz_file['goal_mask']
        if self.transform:
            condition_image = self.transform(condition_image)
            object_mask = self.transform(object_mask)
            # goal_mask = self.transform(goal_mask)
            
        # inp_mask = torch.cat([condition_image, object_mask], dim=0)
        return condition_image, condition_image * 2 - 1.0, object_mask

# VAE loss function
def vae_loss_function(recon_x, x, mu, log_var, beta=1.0):
    batch_size = recon_x.shape[0]
    
    # Reconstruction loss (binary cross entropy)
    BCE = nn.functional.binary_cross_entropy(recon_x, x, reduction='mean')
    
    # KL divergence
    KLD = -0.5 * torch.sum(1 + log_var - mu.pow(2) - log_var.exp()) / batch_size
    
    return BCE + beta * KLD, BCE, KLD

def train(model, train_loader, optimizer, device, epoch, beta=1.0):
    model.train()
    train_loss = 0
    recon_loss = 0
    kl_loss = 0
    
    pbar = tqdm(train_loader)
    for batch_idx, (inp_mask, condition, out_mask) in enumerate(pbar):
        inp_mask = inp_mask.to(device)
        condition = condition.to(device)
        out_mask = out_mask.to(device)
        
        optimizer.zero_grad()
        
        if inp_mask.shape[0] == 1:
            continue
        recon_mask, mu, log_var = model(inp_mask, condition)
        loss, bce, kld = vae_loss_function(recon_mask, out_mask, mu, log_var, beta)
        
        loss.backward()
        optimizer.step()
        
        train_loss += loss.item()
        recon_loss += bce.item()
        kl_loss += kld.item()
        
        # Update progress bar
        pbar.set_description(f"Epoch {epoch} [Train] Loss: {loss.item()/len(out_mask):.4f}")
        
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
        for inp_mask, condition, out_mask in val_loader:
            inp_mask = inp_mask.to(device)
            condition = condition.to(device)
            out_mask = out_mask.to(device)
            
            recon_mask, mu, log_var = model(inp_mask, condition)
            loss, bce, kld = vae_loss_function(recon_mask, out_mask, mu, log_var, beta)
            
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
            inp_mask, condition, out_mask = next(iter(val_loader))
            inp_mask = inp_mask.to(device)
            condition = condition.to(device)
            out_mask = out_mask.to(device)
            
            recon_mask, _, _ = model(inp_mask, condition)
            
            # Save the reconstructions
            n = min(8, out_mask.size(0))
            comparison = torch.cat([out_mask[:n], recon_mask[:n]])
            save_image(comparison.cpu(), f'reconstruction_epoch_{epoch}.png', nrow=n)
    
    return avg_loss

def predict(model, condition_image, device, num_samples=1):
    """
    Generate mask predictions for a given conditional image
    
    Args:
        model: trained CVAE model
        condition_image: the input image to condition on (3-channel)
        device: device to run inference on
        num_samples: number of different masks to generate
        
    Returns:
        List of predicted masks
    """
    model.eval()
    
    # Make sure condition is properly formatted (batch dimension, proper size, etc.)
    if len(condition_image.shape) == 3:  # Add batch dimension if missing
        condition_image = condition_image.unsqueeze(0)
    
    condition_image = condition_image.to(device)
    
    # Generate different masks by sampling from the latent space
    generated_masks = []
    
    with torch.no_grad():
        for _ in range(num_samples):
            # Get condition encoding
            # condition_encoded = model.condition_encoder(condition_image)
            
            # Sample from prior distribution (standard normal)
            z = torch.randn(condition_image.size(0), model.fc_mu.out_features, device=device)
            
            # Decode
            # z_cond = torch.cat([z, condition_encoded], dim=1)
            z_cond = z
            x = model.decoder_linear(z_cond)
            x = x.view(-1, 32, 14, 14)
            mask = model.decoder(x)
            
            print(mask.min(), mask.max())
            
            # mask = (mask > 0.5).float()
            generated_masks.append(mask)
    
    return generated_masks

def main():
    # Set random seeds for reproducibility
    # random.seed(42)
    # np.random.seed(42)
    # torch.manual_seed(42)
    # torch.cuda.manual_seed(42)
    
    # Hyperparameters
    batch_size = 32
    learning_rate = 1e-4
    epochs = 50
    latent_dim = 128
    beta = 0.01 # KL weight
    
    # Device
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # Data paths (replace with your actual paths)
    folder_path = "/common/users/dm1487/namo_data/env_config_1_tr_images"
    
    # Data transformations
    transform = transforms.Compose([
        # transforms.Resize((224, 224)),
        transforms.ToTensor(),
    ])
    
    # Create datasets and dataloaders
    # Note: You'll need to split your data into train/val sets
    train_dataset = MaskDataset(folder_path=folder_path, split="train", transform=transform)
    val_dataset = MaskDataset(folder_path=folder_path, split="val", transform=transform)  # Use different data for validation
    
    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True, num_workers=4)
    val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False, num_workers=4)
    
    # Initialize model
    model = CVAE(latent_dim=latent_dim).to(device)
    
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
        
        # Save model checkpoint
        # if epoch % 10 == 0:
        #     torch.save(model.state_dict(), f"cvae_model_epoch_{epoch}.pt")
    
    # Save final model
    torch.save(model.state_dict(), "cvae_model_final.pt")
    
    # Plot training curves
    plt.figure(figsize=(10, 5))
    plt.plot(range(1, epochs + 1), train_losses, label='Train Loss')
    plt.plot(range(1, epochs + 1), val_losses, label='Validation Loss')
    plt.xlabel('Epochs')
    plt.ylabel('Loss')
    plt.legend()
    plt.savefig('training_curve.png')
    plt.close()
    
    model.load_state_dict(torch.load("cvae_model_final.pt"))
    
    # Example prediction
    print("Generating sample predictions...")
    # reset the val_loader
    val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False, num_workers=4)
    _, sample_condition, _ = next(iter(val_loader))
    sample_condition = sample_condition[0].unsqueeze(0).to(device)
    
    # Generate 5 different possible masks
    predictions = predict(model, sample_condition, device, num_samples=5)
    
    # Save predictions
    for i, pred in enumerate(predictions):
        save_image(pred.cpu(), f'sample_prediction_{i}.png')
    
    print("Training and evaluation completed!")

if __name__ == "__main__":
    main()
