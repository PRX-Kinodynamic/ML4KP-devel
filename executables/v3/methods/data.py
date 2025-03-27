from torch.utils.data import Dataset
import os
import numpy as np
import torch

class Img2ImgDataset(Dataset):
    def __init__(self, data_path):
        self.data_path = data_path

    def __len__(self):
        return len(self.data_path)
    
    def __getitem__(self, idx):
        data = np.load(self.data_path[idx])
        # Convert numpy arrays to torch tensors with specified data types
        inp = torch.from_numpy(data["inp"]).float()  # or .byte() if input should be uint8
        out = torch.from_numpy(data["out"][:, :, :1]).float()  # or .byte()
        return inp, out

if __name__ == "__main__":
    dataset = Img2ImgDataset("/common/users/dm1487/namo_data/binary_images/env_config_1")
    print(len(dataset))
    print(dataset[0])