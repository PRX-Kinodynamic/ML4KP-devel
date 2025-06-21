import torch
from data import CustomDataset
from torch.utils.data import DataLoader
from diffusers import AutoencoderKL, UNet2DModel, UNet2DConditionModel, DDPMScheduler
from accelerate import Accelerator
from torch.optim import AdamW
from torchvision.models import resnet18
import torch.nn as nn
import torch.nn.functional as F
from tqdm import tqdm
import matplotlib.pyplot as plt
import numpy as np

config = {
    'env_config_name': 'env_config_2',
    "num_epochs": 100,
    "save_every": 10,
    "batch_size": 16,
    "learning_rate": 1e-4,
    "weight_decay": 0.01,
    "num_workers": 4,
    "device_id": 2,
    "in_channels": 4,
    "out_channels": 4,
    "image_size": 8,
    "scene_size": 224,
    "num_diffusion_timesteps": 100,
    "save_dir": "executables/v3/diffusion_v2/saved_models/only_object_mask/apr25/fixed_start_fixed_goal_many_env_config_2_dino_arrakis_with_cond_inp",
    "mode": "zmq", # train, test, zmq
    "load_model_path": "executables/v3/diffusion_v2/saved_models/only_object_mask/apr25/fixed_start_fixed_goal_many_env_config_2_dino_arrakis_with_cond_inp/best",
    "cross_attention_dim": 256,
    # "train_data_dir": "/common/users/dm1487/namo_data/env_config_1_tr_images",
    "train_data_dir": ["/common/users/dm1487/namo_data/images/apr22/fixed_start_fixed_goal_many_env"]
}


torch.cuda.set_device(config["device_id"])

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

# pretrained resnet18
backbone = resnet18(pretrained=True)
scene_encoder = nn.Sequential(
    backbone.conv1,
    backbone.bn1,
    backbone.relu,
    backbone.maxpool,
    backbone.layer1,
    backbone.layer2,
    backbone.layer3,
)
scene_encoder.eval()
# scene_encoder.requires_grad = False

# I dont know the hidden size of the resnet18 backbone
hidden_size = 256
scene_adapter = nn.Sequential(
    nn.Conv2d(hidden_size, hidden_size, kernel_size=1),
    nn.GELU(),
    nn.Conv2d(hidden_size, hidden_size, kernel_size=1),
)

autoencoder_model_path = "vae_checkpoint_epoch_best.pt"
autoencoder.load_state_dict(torch.load(autoencoder_model_path))
autoencoder.eval()
# autoencoder.requires_grad = False

conditional_unet_model = UNet2DModel(
            sample_size=config["image_size"],
            in_channels=config["in_channels"],
            out_channels=config["out_channels"],
            layers_per_block=2,
            # downsample_type="resnet",
            # upsample_type="resnet",
            # block_out_channels=(32, 64, 128, 256, 512, 1024, 1024),
            # down_block_types=(
            #     "DownBlock2D",
            #     "DownBlock2D",
            #     "DownBlock2D",
            #     "DownBlock2D",
            #     "DownBlock2D",
            #     "DownBlock2D",
            #     "DownBlock2D",
            # ),
            # up_block_types=(
            #     "UpBlock2D",  # a regular ResNet upsampling block
            #     "UpBlock2D",
            #     "UpBlock2D",
            #     "UpBlock2D",
            #     "UpBlock2D",
            #     "UpBlock2D",
            #     "UpBlock2D",
            # ),
            # cross_attention_dim=config["cross_attention_dim"],
        )

noise_scheduler = DDPMScheduler(num_train_timesteps=config["num_diffusion_timesteps"], beta_schedule='squaredcos_cap_v2')

dataset = CustomDataset(config["train_data_dir"], 64, 224, config["mode"])
dataloader = DataLoader(dataset, batch_size=config["batch_size"], shuffle=True, num_workers=config["num_workers"])

optimizer = AdamW(list(conditional_unet_model.parameters()), lr=config["learning_rate"], weight_decay=config["weight_decay"])
# conditional_unet_model.enable_xformers_memory_efficient_attention()
conditional_unet_model.enable_gradient_checkpointing()

accelerator = Accelerator(mixed_precision="fp16")
autoencoder, conditional_unet_model, scene_encoder, scene_adapter, dataloader, optimizer = accelerator.prepare(autoencoder, conditional_unet_model, scene_encoder, scene_adapter, dataloader, optimizer)


# pbar with logging per epoch
pbar = tqdm(range(config["num_epochs"]), desc="Epochs")
# train loop
for epoch in pbar:
    epoch_loss = 0
    conditional_unet_model.train()
    scene_adapter.train()
    
    for batch in dataloader:
        optimizer.zero_grad()
        # get the scene image
        scene_image = batch["scene"]
        decision_target = batch["decision_target"]
        # print(decision_target.shape)

        # get the scene embedding
        with torch.no_grad():
            # print("here", decision_target.shape)
            z = autoencoder.encode(decision_target).latent_dist.sample() * autoencoder.config.scaling_factor
        #     scene_embedding = scene_encoder(scene_image)
        # B, C, H, W = scene_embedding.shape
        # scene_embedding = scene_embedding.view(B, C, H*W).permute(0, 2, 1)
        
        # get the noise
        noise = torch.randn_like(z)
        # print(noise.shape)
        timesteps = torch.randint(0, config["num_diffusion_timesteps"], (z.shape[0],), device=z.device).long()
        noisy_z = noise_scheduler.add_noise(z, noise, timesteps)
        
        # get the model prediction
        noise_prediction = conditional_unet_model(noisy_z, timesteps).sample
        
        # get the loss
        loss = F.mse_loss(noise_prediction, noise)
        
        # backprop
        
        accelerator.backward(loss)
        optimizer.step()
        epoch_loss += loss.item()
    pbar.update(1)
    pbar.set_postfix(loss=epoch_loss/len(dataloader))
    
    
    if (pbar.n+1) % 10 == 0:
        
        data_idx = np.random.randint(0, len(dataset))
        # check the prediction
        with torch.no_grad():
            conditional_unet_model.eval()
            # scene_adapter.eval()
            data = dataset[data_idx]
            decision_target = data["decision_target"].unsqueeze(0).to(autoencoder.device)
            # scene_image = data["scene"]
            z = autoencoder.encode(decision_target).latent_dist.sample() * autoencoder.config.scaling_factor
            # scene_embedding = scene_encoder(scene_image)
            # scene_embedding = scene_embedding.view(B, C, H*W).permute(0, 2, 1)
            
            noise = torch.randn_like(z, device=z.device)
            for t in reversed(range(0, noise_scheduler.config.num_train_timesteps)):
                noise_prediction = conditional_unet_model(noise, t).sample
                noise = noise_scheduler.step(noise_prediction, t, noise).prev_sample
                    
            predicted_decision_target = autoencoder.decode(noise / autoencoder.config.scaling_factor).sample 
            
            fig, axs = plt.subplots(1, 2, figsize=(10, 5))
            axs[0].imshow(decision_target[0, 0].cpu().numpy(), cmap="gray")
            axs[1].imshow(predicted_decision_target[0, 0].cpu().numpy(), cmap="gray")
            plt.savefig(f"test_object_mask_{pbar.n}.png")
            
            # predicted_goal_mask = predicted_decision_target[0, 1, :, :]
            # plt.imshow(predicted_goal_mask.cpu().numpy(), cmap="gray")
            # plt.savefig(f"test_goal_mask.png")
        
        
        
        
        