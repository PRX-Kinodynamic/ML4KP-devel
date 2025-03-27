import os
import glob
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader, random_split
from PIL import Image
import matplotlib.pyplot as plt
from tqdm import tqdm
from dino_heatmap import DinoHeatmap
from transformers import AutoImageProcessor

class HeatmapDataset(Dataset):
    def __init__(self, data_dir, transform=None):
        self.data_dir = data_dir
        self.file_paths = sorted(glob.glob(os.path.join(data_dir, "*.npz")))
        self.transform = transform
        self.processor = AutoImageProcessor.from_pretrained('facebook/dinov2-small')
        
    def __len__(self):
        return len(self.file_paths)
    
    def __getitem__(self, idx):
        data = np.load(self.file_paths[idx])
        
        # Input image
        inp = data['inp']
        ## Calculate padding sizes
        pad_h = (224 - inp.shape[0]) // 2
        pad_w = (224 - inp.shape[1]) // 2
        # Handle odd dimensions by adding extra padding to the right/bottom if needed
        pad_h_extra = 224 - inp.shape[0] - 2 * pad_h
        pad_w_extra = 224 - inp.shape[1] - 2 * pad_w
        
        # Pad the image ((top, bottom), (left, right))
        inp_padded = np.pad(inp, 
                    ((pad_h, pad_h + pad_h_extra), 
                    (pad_w, pad_w + pad_w_extra),
                    (0, 0)),
                    mode='constant',
                    constant_values=0)
        
        
        input_img = (inp_padded * 255).astype(np.uint8)
        input_img = Image.fromarray(input_img)
        
        inputs = self.processor(images=input_img, return_tensors='pt')
        # inputs = {k: v.to(self.device) for k, v in inputs.items()}
        
        # Target heatmap - assuming data contains a 'heatmap' key
        # If your data structure is different, adjust accordingly
        target = torch.tensor(data['out'][:, :, 0]).float()
        
        if self.transform:
            input_img = self.transform(input_img)
        
        return inputs, target, torch.from_numpy(inp)

class FocalLoss(nn.Module):
    def __init__(self, alpha=1, gamma=2):
        super().__init__()
        self.alpha = alpha
        self.gamma = gamma
        
    def forward(self, inputs, targets):
        bce_loss = nn.functional.binary_cross_entropy(inputs, targets, reduction='none')
        pt = torch.exp(-bce_loss)  # prevents nans when probability 0
        focal_loss = self.alpha * (1-pt)**self.gamma * bce_loss
        return focal_loss.mean()

def dice_coefficient(pred, target, smooth=1e-6):
    pred = pred.view(-1)
    target = target.view(-1)
    intersection = (pred * target).sum()
    return (2. * intersection + smooth) / (pred.sum() + target.sum() + smooth)

def train_model(model, train_loader, val_loader, criterion, optimizer, scheduler, 
                num_epochs=10, device='cuda', save_model_name=None):
    best_val_loss = float('inf')
    train_losses = []
    val_losses = []
    train_dices = []
    val_dices = []
    
    for epoch in range(num_epochs):
        # Training phase
        model.train()
        running_loss = 0.0
        running_dice = 0.0
        
        with tqdm(train_loader, desc=f"Epoch {epoch+1}/{num_epochs} [Train]") as pbar:
            for inputs, targets, _ in pbar:
                inputs = {k: v.squeeze(1) for k, v in inputs.items()}
                inputs = {k: v.to(device) for k, v in inputs.items()}
                targets = targets.to(device)
                # Forward pass
                outputs = model(inputs)
                
                # Reshape outputs or targets if needed to match dimensions
                if outputs.shape != targets.shape:
                    targets = torch.nn.functional.interpolate(
                        targets.unsqueeze(1) if targets.dim() == 3 else targets, 
                        size=(128, 128), 
                        mode='bilinear', 
                        align_corners=False
                    )
                
                loss = criterion(outputs, targets)
                
                # Backward and optimize
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
                
                running_loss += loss.item()
                with torch.no_grad():
                    dice = dice_coefficient(outputs, targets)
                running_dice += dice.item()
                pbar.set_postfix(loss=loss.item(), dice=dice.item())
        
        train_loss = running_loss / len(train_loader)
        train_dice = running_dice / len(train_loader)
        train_losses.append(train_loss)
        train_dices.append(train_dice)
        
        # Validation phase
        model.eval()
        val_loss = 0.0
        val_dice = 0.0
        
        with torch.no_grad():
            with tqdm(val_loader, desc=f"Epoch {epoch+1}/{num_epochs} [Val]") as pbar:
                for inputs, targets, _ in pbar:
                    inputs = {k: v.squeeze(1) for k, v in inputs.items()}
                    inputs = {k: v.to(device) for k, v in inputs.items()}
                    targets = targets.to(device)
                    outputs = model(inputs)
                    
                    # Reshape if needed
                    if outputs.shape != targets.shape:
                        targets = torch.nn.functional.interpolate(
                            targets.unsqueeze(1) if targets.dim() == 3 else targets, 
                            size=(128, 128), 
                            mode='bilinear', 
                            align_corners=False
                        )
                    
                    loss = criterion(outputs, targets)
                    dice = dice_coefficient(outputs, targets)
                    
                    val_loss += loss.item()
                    val_dice += dice.item()
                    pbar.set_postfix(loss=loss.item(), dice=dice.item())
        
        val_loss = val_loss / len(val_loader)
        val_dice = val_dice / len(val_loader)
        val_losses.append(val_loss)
        val_dices.append(val_dice)
        
        print(f"Epoch {epoch+1}/{num_epochs}:")
        print(f"Train Loss: {train_loss:.4f}, Train Dice: {train_dice:.4f}")
        print(f"Val Loss: {val_loss:.4f}, Val Dice: {val_dice:.4f}")
        
        # Save best model (now using dice as metric)
        if val_dice > max(val_dices[:-1], default=0):
            model.save_model(save_model_name)
            print(f"Saved best model with val dice: {val_dice:.4f}")
            
        # scheduler.step()
    
    # Plot loss curves
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 5))
    
    # Loss plot
    ax1.plot(train_losses, label='Training Loss')
    ax1.plot(val_losses, label='Validation Loss')
    ax1.set_xlabel('Epoch')
    ax1.set_ylabel('Loss')
    ax1.legend()
    ax1.set_title('Loss Curves')
    
    # Dice plot
    ax2.plot(train_dices, label='Training Dice')
    ax2.plot(val_dices, label='Validation Dice')
    ax2.set_xlabel('Epoch')
    ax2.set_ylabel('Dice Coefficient')
    ax2.legend()
    ax2.set_title('Dice Coefficient Curves')
    
    plt.tight_layout()
    plt.savefig('training_metrics.png')
    plt.close()
    
    return model, train_losses, val_losses, train_dices, val_dices

def visualize_predictions(model, val_loader, device, num_samples=5):
    model.eval()
    fig, axes = plt.subplots(num_samples, 3, figsize=(15, 5*num_samples))
    
    with torch.no_grad():
        for i, (inputs, targets, inp) in enumerate(val_loader):
            if i >= num_samples:
                break
            inputs = {k: v.squeeze(1) for k, v in inputs.items()}
            inputs = {k: v.to(device) for k, v in inputs.items()}
            outputs = model(inputs)
            
            # Resize targets if needed
            if outputs.shape != targets.shape:
                targets = torch.nn.functional.interpolate(
                    targets.unsqueeze(1) if targets.dim() == 3 else targets, 
                    size=(128, 128), 
                    mode='bilinear', 
                    align_corners=False
                )
                
            print(outputs.shape, targets.shape)
            
            # Convert to numpy and normalize input images
            input_np = inp[0].numpy()
            # Normalize input image to [0, 1] range
            input_np = (input_np - input_np.min()) / (input_np.max() - input_np.min())
            
            target_np = targets[0].cpu().squeeze().numpy()
            output_np = outputs[0].cpu().squeeze().numpy()
            
            print(output_np.shape, target_np.shape) 
            
            print(output_np.sum(), output_np.max(), output_np.min())
            
            # Plot
            axes[i, 0].imshow(input_np, cmap='gray' if len(input_np.shape) == 2 else None)
            axes[i, 0].set_title('Input')
            axes[i, 0].axis('off')
            
            im1 = axes[i, 1].imshow(target_np, cmap='viridis')
            axes[i, 1].set_title('Target Heatmap')
            axes[i, 1].axis('off')
            axes[i, 1].grid()
            plt.colorbar(im1, ax=axes[i, 1], fraction=0.046, pad=0.04)
            
            im2 = axes[i, 2].imshow(output_np, cmap='viridis')
            axes[i, 2].set_title('Predicted Heatmap')
            axes[i, 2].axis('off')
            axes[i, 2].grid()
            plt.colorbar(im2, ax=axes[i, 2], fraction=0.046, pad=0.04)
            
    
    plt.tight_layout()
    plt.savefig('predictions.png', bbox_inches='tight', dpi=300)
    plt.close()

def main():
    # Set device
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Using device: {device}")
    
    # Data directory
    data_dir = '/common/users/dm1487/namo_data/binary_images/env_config_1'
    
    # Create dataset
    dataset = HeatmapDataset(data_dir)
    
    # Train-test split (90:10)
    train_size = int(0.9 * len(dataset))
    val_size = len(dataset) - train_size
    train_dataset, val_dataset = random_split(dataset, [train_size, val_size])
    
    print(f"Train dataset size: {len(train_dataset)}")
    print(f"Validation dataset size: {len(val_dataset)}")
    
    # Create data loaders
    train_loader = DataLoader(train_dataset, batch_size=16, shuffle=True, num_workers=4)
    val_loader = DataLoader(val_dataset, batch_size=16, shuffle=False, num_workers=4)
    
    # Initialize model with LoRA
    model = DinoHeatmap(device=device, lora_rank=8)
    
    # Replace BCE Loss with Focal Loss
    criterion = nn.BCELoss()
    # criterion = nn.MSELoss()
    # criterion = FocalLoss(alpha=1, gamma=2)  # alpha and gamma are hyperparameters you can tune
    
    # Create optimizer with different learning rates
    optimizer = optim.AdamW([
        {
            'params': model.dinov2.parameters(),
            'lr': 1e-5,  # Lower learning rate for LoRA parameters
            'weight_decay': 0.01
        },
        {
            'params': model.upsample.parameters(),
            'lr': 1e-4,  # Higher learning rate for upsampling layers
            'weight_decay': 0.01
        }
    ])
    
    # Learning rate scheduler (optional)
    scheduler = optim.lr_scheduler.CosineAnnealingLR(
        optimizer, 
        T_max=20,
        eta_min=1e-6
    )
    
    # Train model
    model, train_losses, val_losses, train_dices, val_dices = train_model(
        model, 
        train_loader, 
        val_loader, 
        criterion, 
        optimizer,
        scheduler=None,
        num_epochs=20,
        device=device,
        save_model_name='best_dino_heatmap'
    )
    
    
    model = load_trained_model(device, 'best_dino_heatmap')
    model.eval()
    # Visualize some predictions
    visualize_predictions(model, val_loader, device)
    
    print("Training completed!")

def load_trained_model(device, weights_path):
    """Load a trained model with both LoRA and upsampling weights"""
    model = DinoHeatmap(device=device)
    model.load_model(weights_path)
    return model

if __name__ == "__main__":
    main()