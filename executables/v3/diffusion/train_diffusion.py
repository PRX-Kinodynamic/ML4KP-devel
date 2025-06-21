import os
import torch
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms
from torchvision.utils import save_image
import numpy as np
from tqdm import tqdm
import matplotlib.pyplot as plt
from diffusion_model import SimpleDiffusionModel, train_step, prepare_noise_schedule, q_sample, binary_diffusion_loss
import torch.nn.functional as F

class MaskDataset(Dataset):
    def __init__(self, folder_path, split, transform=None):
        self.folder_path = folder_path
        self.filenames = [f for f in os.listdir(folder_path) if f.endswith('.npz') and (f.endswith('01.npz'))]
        length = len(self.filenames)
        train_len = int(0.9 * length)

        if split == "train":
            self.filenames = self.filenames[:train_len]
        else:
            self.filenames = self.filenames[train_len:]

        self.transform = transform

    def __len__(self):
        return len(self.filenames)

    def __getitem__(self, idx):
        npz = np.load(os.path.join(self.folder_path, self.filenames[idx]))
        # We only need the mask for unconditional training
        mask = npz['object_mask'].astype(np.float32)  # HW
        
        # Ensure binary values are exactly 0 and 1 (no intermediate values)
        mask = (mask > 0.5).astype(np.float32)

        if self.transform:
            mask = self.transform(mask)

        return (mask * 2.0) - 1.0 # -1 to 1


def validate(model, val_loader, noise_scheduler, timesteps, device):
    model.eval()
    total_loss = 0
    with torch.no_grad():
        for mask in val_loader:
            mask = mask.to(device)
            B = mask.size(0)
            t = torch.randint(0, timesteps, (B,), device=device).long()
            noise = torch.randn_like(mask)
            noisy_mask = q_sample(mask, t, noise, noise_scheduler['sqrt_alphas_cumprod'], noise_scheduler['sqrt_one_minus_alphas_cumprod'])
            predicted = model(noisy_mask, t)
            # Use the same loss function as training
            loss = binary_diffusion_loss(predicted, noise)
            total_loss += loss.item()
    return total_loss / len(val_loader)


def sample(model, noise_scheduler, device, num_samples=4, img_size=224, steps=100, temperature=1.0):
    model.eval()
    from matplotlib import pyplot as plt
    with torch.no_grad():
        # Start with pure noise
        x = torch.randn(num_samples, 1, img_size, img_size).to(device) # * temperature
        
        # Store intermediate results
        intermediates = []
        save_folder = "executables/v3/diffusion/out2"
        os.makedirs(save_folder, exist_ok=True)
        
        # Reverse diffusion process
        for t in reversed(range(steps)):
            t_tensor = torch.full((num_samples,), t, device=device, dtype=torch.long)
            noise_pred = model(x, t_tensor)
            
            if t == (steps-1):
                noise_pred_1 = noise_pred.clone()
                print(noise_pred_1.shape)
                plt.imshow(noise_pred_1[0].cpu().permute(1, 2, 0).numpy(), cmap='gray')
                plt.colorbar()
                plt.savefig(f"{save_folder}/noise_pred_{t}.png")
                plt.close()
            
            # DDIM-style update (more stable for binary masks)
            alpha = noise_scheduler['alphas'][t]
            alpha_hat = noise_scheduler['alphas_cumprod'][t]
            beta = noise_scheduler['betas'][t]
            
            # posterior_mean_coef_1 = noise_scheduler['posterior_mean_coef_1'][t]
            # posterior_mean_coef_2 = noise_scheduler['posterior_mean_coef_2'][t]
            # posterior_variance = noise_scheduler['posterior_variance'][t]
            
            
            ## see this
            if t > 0:
                next_alpha_hat = noise_scheduler['alphas_cumprod'][t-1]
            else:
                next_alpha_hat = torch.tensor(1.0).to(device)
                
            # DDIM formula
            c1 = torch.sqrt(next_alpha_hat / alpha_hat)
            c2 = torch.sqrt(1 - next_alpha_hat) - torch.sqrt(alpha) * torch.sqrt(1 - alpha_hat) / torch.sqrt(alpha_hat)
            x = c1 * x - c2 * noise_pred
            ##
            
            intermediate = x.clone()
            intermediates.append(intermediate)
            
            
            # sqrt_reciprocal_alphas_cumprod = noise_scheduler['sqrt_reciprocal_alphas_cumprod'][t]
            # sqrt_reciprocal_m1_alphas_cumprod = noise_scheduler['sqrt_reciprocal_m1_alphas_cumprod'][t]
            
            # x_recon = sqrt_reciprocal_alphas_cumprod * x - sqrt_reciprocal_m1_alphas_cumprod * noise_pred
            
            # # x_recon = torch.clamp(x_recon, min=0, max=1)
            
            # x_recon = posterior_mean_coef_1 * x_recon + posterior_mean_coef_2 * noise_pred
            # # x_recon = torch.clamp(x_recon, min=0, max=1)
            
            # # gaussian diffusion
            # x = x_recon + torch.sqrt(posterior_variance) * torch.randn_like(x)
            # x = torch.clamp(x, min=0, max=1)
            
            
            # Only add noise at higher noise levels (helps make sharper boundaries)
            # if t > steps // 4:
            #     noise = torch.randn_like(x)
            #     x += temperature * torch.sqrt(beta) * noise
                
            # # Store intermediates 
            # if t % (steps // 10) == 0 or t == steps-1:
            #     intermediates.append(x.clone())
        
        # # Binary thresholding for final result
        # x = torch.sigmoid(x * 5)  # Sharpen
        # x = (x > 0.5).float()
        
        return x, intermediates


def main():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    folder_path = "/common/users/dm1487/namo_data/env_config_1_tr_images"
    epochs = 100  # Significantly more epochs
    batch_size = 1  # Use all 4 examples in each batch
    lr = 1e-3 # Moderate learning rate
    timesteps = 100  # Fewer timesteps (we only have simple shapes)

    transform = transforms.Compose([
        transforms.ToTensor(),
    ])
    
    train_dataset = torch.utils.data.Subset(
        MaskDataset(folder_path, "train", transform),
        indices=list(range(1))  # only use 4 examples
    )
    
    # Use a batch size of 4 to get all examples in each batch
    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True, num_workers=0, pin_memory=True)
    val_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=False, num_workers=0, pin_memory=True)

    # Custom overfitting-focused model
    model = SimpleDiffusionModel(in_channels=1, base_channels=128, time_emb_dim=256).to(device)
    
    # Stronger optimizer for aggressive fitting
    optimizer = optim.AdamW(model.parameters(), lr=lr, weight_decay=0, betas=(0.9, 0.999))
    
    # More aggressive learning rate schedule
    # scheduler = torch.optim.lr_scheduler.OneCycleLR(
    #     optimizer, 
    #     max_lr=lr,
    #     total_steps=epochs,
    #     pct_start=0.1,  # Quick ramp-up
    #     div_factor=25.0,
    #     final_div_factor=1000.0
    # )
    
    # scheduler = torch.optim.lr_scheduler.ConstantLR(optimizer, factor=0.5, total_iters=100)
    
    noise_scheduler = prepare_noise_schedule(timesteps=timesteps, device=device)
    
    # sample = train_dataset[0].unsqueeze(0).to(device)
    # for i in range(5):
    #     img = q_sample(sample, i, torch.randn_like(sample), noise_scheduler['sqrt_alphas_cumprod'].unsqueeze(-1), noise_scheduler['sqrt_one_minus_alphas_cumprod'].unsqueeze(-1))
    #     save_image(img, f"executables/v3/diffusion/out/noising_step_{i}.png", nrow=1)
    
    # exit()

    train_losses = []
    val_losses = []

    # Save initial random samples
    sampled, _ = sample(model, noise_scheduler, device, num_samples=1, img_size=224, steps=timesteps)
    
    save_folder = "executables/v3/diffusion/out1"
    os.makedirs(save_folder, exist_ok=True)
    
    save_image(sampled, f"{save_folder}/epoch_0_sample.png", nrow=2)

    # Visualize ground truth
    val_iter = iter(val_loader)
    gt_masks = next(val_iter).to(device)
    save_image(gt_masks, f"{save_folder}/ground_truth.png", nrow=2)

    for epoch in range(1, epochs + 1):
        model.train()
        total_loss = 0
        pbar = tqdm(train_loader)

        for mask in pbar:
            mask = mask.to(device)
            
            # Extreme overfitting: multiple steps per batch
            for _ in range(10):  # 10 optimization steps per batch
                loss = train_step(model, mask, timesteps, optimizer, noise_scheduler, device)
            
            total_loss += loss
            pbar.set_description(f"Epoch {epoch} | Train Loss: {loss:.6f}")

        avg_train_loss = total_loss / len(train_loader)
        train_losses.append(avg_train_loss)
        val_loss = validate(model, val_loader, noise_scheduler, timesteps, device)
        val_losses.append(val_loss)
        
        # Step the scheduler
        # scheduler.step()

        print(f"Epoch {epoch} - Train Loss: {avg_train_loss:.6f} | Val Loss: {val_loss:.6f} | LR: {optimizer.param_groups[0]['lr']:.6f}")

        # Save more frequently at the beginning
        if epoch % 10 == 0 or epoch == epochs:
            # Generate samples from noise with decreased temperature for sharper results
            sampled, intermediates = sample(
                model, 
                noise_scheduler, 
                device, 
                num_samples=4, 
                img_size=224, 
                steps=timesteps,
                temperature=0.8  # Lower temperature for sharper results
            )
            
            # Save final result
            plt.imshow(sampled[0].cpu().permute(1, 2, 0).numpy(), cmap='gray')
            plt.colorbar()
            plt.savefig(f"{save_folder}/epoch_{epoch}_sample.png")
            plt.close()
            # save_image(sampled, f"{save_folder}/epoch_{epoch}_sample.png", nrow=2)
            
            # Save some intermediate steps (optional)
            if len(intermediates) > 0:
                for i, intermediate in enumerate(intermediates):
                    plt.imshow(intermediate[0].cpu().permute(1, 2, 0).numpy(), cmap='gray')
                    plt.colorbar()
                    plt.savefig(f"{save_folder}/epoch_{epoch}_step_{i}.png")
                    plt.close()
            
            # Save model checkpoint
            torch.save({
                'epoch': epoch,
                'model_state_dict': model.state_dict(),
                'optimizer_state_dict': optimizer.state_dict(),
                'train_loss': avg_train_loss,
                'val_loss': val_loss,
            }, f"model_checkpoint_latest.pt")

    # Plot loss curves
    plt.figure(figsize=(10, 6))
    plt.plot(range(1, epochs + 1), train_losses, label='Train Loss')
    plt.plot(range(1, epochs + 1), val_losses, label='Val Loss')
    plt.xlabel('Epoch')
    plt.ylabel('Loss')
    plt.yscale('log')  # Log scale helps see the trends better
    plt.title('Training & Validation Loss')
    plt.legend()
    plt.grid(True)
    plt.savefig('loss_curves.png')

    torch.save(model.state_dict(), "simple_diffusion_final.pt")


if __name__ == "__main__":
    main()
