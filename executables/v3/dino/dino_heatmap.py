from transformers import AutoImageProcessor, AutoModel
import torch.nn as nn
import torch
from peft import get_peft_config, get_peft_model, LoraConfig, TaskType
import logging

class DinoHeatmap(nn.Module):
    def __init__(self, device, lora_rank=8):
        super().__init__()
        self.device = device
        
        # Initialize DINO model
        self.dinov2 = AutoModel.from_pretrained('facebook/dinov2-small')
        
        # Print model structure to identify target modules
        # print(logging.getLogger().level, logging.DEBUG)
        # if logging.getLogger().level == logging.DEBUG:
        # for name, module in self.dinov2.named_modules():
        #     print(f"Layer name: {name}")
        
        # Configure LoRA
        lora_config = LoraConfig(
            r=lora_rank,                    # rank of the update matrices
            lora_alpha=16,                  # scaling factor
            target_modules=["query", "key", "value", "dense"],  # attention layers
            lora_dropout=0.1,               # dropout probability for LoRA layers
            bias="none",                    # don't train biases
            modules_to_save=[],             # any other modules to fully fine-tune
            # task_type=TaskType.FEATURE_EXTRACTION,
            inference_mode=False,
        )
        
        # Apply LoRA to DINO
        print("Applying LoRA to DINO...")
        self.dinov2 = get_peft_model(self.dinov2, lora_config)
        self.dinov2.print_trainable_parameters()  # Print number of trainable parameters
        
        # Upsampling layers remain the same
        self.upsample = nn.Sequential(
            nn.ConvTranspose2d(384, 128, kernel_size=4, stride=2, padding=1),
            nn.ReLU(),
            nn.ConvTranspose2d(128, 64, kernel_size=4, stride=2, padding=1),
            nn.ReLU(),
            nn.ConvTranspose2d(64, 1, kernel_size=4, stride=2, padding=1),
            nn.Sigmoid()
        )
        
        self.dinov2.to(self.device)
        self.upsample.to(self.device)
    
    def forward(self, x):
        pixel_values = x['pixel_values']
        outputs = self.dinov2(pixel_values=pixel_values)
        patch_embeddings = outputs.last_hidden_state[:, 1:, :]
        
        x = patch_embeddings.reshape(patch_embeddings.shape[0], 16, 16, -1)
        x = x.permute(0, 3, 1, 2)
        x = self.upsample(x)
        return x
    
    def save_model(self, path):
        """Save both LoRA weights and upsampling layers"""
        # Save LoRA weights
        self.dinov2.save_pretrained(f"{path}_lora")
        
        # Save upsampling layers
        torch.save(self.upsample.state_dict(), f"{path}_upsample.pth")

    def load_model(self, path):
        """Load both LoRA weights and upsampling layers"""
        # Load LoRA weights
        self.dinov2.load_adapter(f"{path}_lora", adapter_name="default")
        
        # Load upsampling layers
        upsample_state = torch.load(f"{path}_upsample.pth")
        self.upsample.load_state_dict(upsample_state)

if __name__ == '__main__':
    import numpy as np
    from PIL import Image
    
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    model = DinoHeatmap(device=device)
    npz_file = '/common/users/dm1487/namo_data/binary_images/env_config_1/0.npz'
    data = np.load(npz_file)
    img = Image.fromarray((data['inp'] * 255).astype(np.uint8))
    y = model(img)
    print(y.shape)