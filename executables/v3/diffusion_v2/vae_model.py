from diffusers import AutoencoderKL
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms
from matplotlib import pyplot as plt
import os
import numpy as np
import random
from tqdm import tqdm
import random
import torch
from torch.optim.lr_scheduler import CosineAnnealingLR
from accelerate import Accelerator
import torch.nn.functional as F
torch.cuda.set_device(0)

class CustomDataset(Dataset):
    def __init__(self, folder_path, transform, mode="train"):
        self.folder_path = folder_path
        self.image_paths = []
        if isinstance(folder_path, list):
            for folder in folder_path:
                self.image_paths.extend([os.path.join(folder, f) for f in os.listdir(folder)])
        else:
            self.image_paths = [os.path.join(folder_path, f) for f in os.listdir(folder_path)]
        random.shuffle(self.image_paths)
        self.image_paths = self.image_paths
        
        if mode == "zmq":
            self.image_paths = self.image_paths[:10]
            
        self.final_image_paths = []
        for idx in tqdm(range(len(self.image_paths))):
            file = self.image_paths[idx]
            npz_file = np.load(file)
            object_mask = npz_file['object_mask']
            if np.sum(object_mask) == 0:
                continue
            self.final_image_paths.append(file)
        
        self.half_len = len(self.final_image_paths) // 2
        self.final_image_paths = self.final_image_paths + self.final_image_paths
        # self.final_image_paths = self.final_image_paths
        self.transform = transform

    def __len__(self):
        return len(self.final_image_paths)

    def __getitem__(self, index):
        image_path = self.final_image_paths[index]
        npz_file = np.load(image_path)
        object_mask = npz_file['object_mask']
        goal_mask = npz_file['goal_mask']
        if index < self.half_len:
            image = object_mask
        else:
            image = goal_mask
        image = self.transform(image)
        return {'pixel_values': image}

autoencoder = AutoencoderKL(
    in_channels=1,
    out_channels=1,
    latent_channels=4,
    down_block_types=("DownEncoderBlock2D", "DownEncoderBlock2D", "DownEncoderBlock2D", "DownEncoderBlock2D"),
    up_block_types=("UpDecoderBlock2D", "UpDecoderBlock2D", "UpDecoderBlock2D", "UpDecoderBlock2D"),
    block_out_channels=(128, 256, 512, 512),  # Standard SD pattern
    layers_per_block=2,
    use_quant_conv=True,
    use_post_quant_conv=True,
    act_fn="silu",  # Standard activation in SD
    scaling_factor=0.18215,  # Explicitly set
)

transform = transforms.Compose([
    transforms.ToTensor(),
    transforms.Resize((64, 64)),
    transforms.Lambda(lambda x: x*2-1),
])

dataset = CustomDataset(folder_path="/common/users/dm1487/namo_data/images/apr21/fixed_start_fixed_goal_many_env", transform=transform)

dataloader = DataLoader(dataset, batch_size=64, num_workers=4, shuffle=True)

optimizer = torch.optim.AdamW(autoencoder.parameters(), lr=5e-5, weight_decay=1e-6)
lr_scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=200)

accelerator = Accelerator()
autoencoder, optimizer, dataloader = accelerator.prepare(autoencoder, optimizer, dataloader)

device = autoencoder.device

def dice_loss(pred, target):
    smooth = 1.0
    pred = torch.sigmoid(pred)
    intersection = (pred * target).sum(dim=(1, 2, 3))
    union = pred.sum(dim=(1, 2, 3)) + target.sum(dim=(1, 2, 3))
    dice = (2.0 * intersection + smooth) / (union + smooth)
    return 1 - dice.mean()

def get_kl_weight(epoch, total_epochs, start_weight=0.00001, end_weight=0.005, start_at=0, warmup_epochs=5):
    """
    Calculate KL weight using linear annealing
    Increases from start_weight to end_weight over warmup_epochs
    """
    if epoch < start_at:
        return start_weight
    if epoch >= warmup_epochs:
        return end_weight
    
    # Linear annealing
    return start_weight + (end_weight - start_weight) * (epoch / (warmup_epochs - start_at))

# Loss function selection
loss_type = "mse"  # Options: "mse", "bce", "bce_dice"

epochs = 300
for epoch in range(epochs):
    train_loss = 0
    avg_recon_loss = 0
    avg_kl_loss = 0
    
    # Calculate KL weight for current epoch
    kl_weight = get_kl_weight(epoch, epochs, start_weight=0.0, end_weight=0.001, start_at=0, warmup_epochs=epochs-50)
    print(f"Epoch {epoch}, KL weight: {kl_weight:.6f}, Loss type: {loss_type}")
    
    for batch in dataloader:
        input_imgs = batch['pixel_values'].to(device)
        posterior = autoencoder.encode(input_imgs).latent_dist
        z = posterior.sample()
        recon = autoencoder.decode(z).sample
        
        # Select reconstruction loss based on loss_type
        if loss_type == "mse":
            recon_loss = F.mse_loss(recon, input_imgs)
        elif loss_type == "bce":
            # Map inputs from [-1,1] to [0,1] for BCE
            recon_sigmoid = (recon + 1) / 2
            input_imgs_scaled = (input_imgs + 1) / 2
            recon_loss = F.binary_cross_entropy_with_logits(recon_sigmoid, input_imgs_scaled)
        elif loss_type == "bce_dice":
            # Combine BCE and Dice loss
            recon_sigmoid = (recon + 1) / 2
            input_imgs_scaled = (input_imgs + 1) / 2
            bce_loss = F.binary_cross_entropy_with_logits(recon_sigmoid, input_imgs_scaled)
            dice = dice_loss(recon_sigmoid, input_imgs_scaled)
            recon_loss = bce_loss + dice
        
        kl_loss = posterior.kl().mean()
        
        # Use annealed KL weight
        loss = recon_loss + kl_weight * kl_loss
        
        optimizer.zero_grad()
        torch.nn.utils.clip_grad_norm_(autoencoder.parameters(), 1.0)
        accelerator.backward(loss)
        optimizer.step()
        lr_scheduler.step()
        train_loss += loss.item()
        avg_recon_loss += recon_loss.item()
        avg_kl_loss += kl_loss.item()
    train_loss /= len(dataloader)
    avg_recon_loss /= len(dataloader)
    avg_kl_loss /= len(dataloader)
    print(f"Epoch {epoch} loss: {train_loss}, recon_loss: {avg_recon_loss}, kl_loss: {avg_kl_loss}")

    # Add model saving
    if (epoch + 1) % 20 == 0:
        unwrapped_model = accelerator.unwrap_model(autoencoder)
        torch.save(unwrapped_model.state_dict(), f"vae_checkpoint_epoch_{epoch+1}.pt")
        
unwrapped_model = accelerator.unwrap_model(autoencoder)
torch.save(unwrapped_model.state_dict(), f"vae_checkpoint_epoch_best.pt")
    
# # save some images using matplotlib
autoencoder.eval()
with torch.no_grad():
    for i in range(10):
        image = dataset[np.random.randint(0, len(dataset))]['pixel_values'].unsqueeze(0).to(device)
        z = autoencoder.encode(image).latent_dist.sample()
        recon = autoencoder.decode(z).sample
        
        image = image.squeeze(0)
        recon = recon.squeeze(0)
        image = image.permute(1, 2, 0).cpu().numpy()
        recon = recon.permute(1, 2, 0).cpu().numpy()
        fig, axs = plt.subplots(1, 2)
        axs[0].imshow(image[:, :, 0], cmap="gray")
        axs[0].set_title("Object mask")
        axs[1].imshow(recon[:, :, 0], cmap="gray")
        axs[1].set_title("Reconstructed object mask")
        plt.savefig(f"output_{i}.png")
        plt.close()
