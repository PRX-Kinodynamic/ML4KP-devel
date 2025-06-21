import os
import random
import numpy as np
import torchvision.transforms as transforms
from torch.utils.data import Dataset
from tqdm import tqdm

class CustomDataset(Dataset):
    def __init__(self, folder_path, image_size, scene_size, mode="train"):
        self.folder_path = folder_path
        self.image_paths = []
        if isinstance(folder_path, list):
            for folder in folder_path:
                self.image_paths.extend([os.path.join(folder, f) for f in os.listdir(folder)])
        else:
            self.image_paths = [os.path.join(folder_path, f) for f in os.listdir(folder_path)]
        random.shuffle(self.image_paths)
        self.image_paths = self.image_paths
        
        # if mode == "zmq":
        #     self.image_paths = self.image_paths[:10]
        
        self.final_image_paths = []
        for idx in tqdm(range(len(self.image_paths))):
            file = self.image_paths[idx]
            npz_file = np.load(file)
            object_mask = npz_file['object_mask']
            if np.sum(object_mask) == 0:
                continue
            self.final_image_paths.append(file)
            
        self.final_image_paths = self.final_image_paths
        self.half_len = len(self.final_image_paths)
        self.final_image_paths = self.final_image_paths + self.final_image_paths
        
        self.transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Resize((image_size, image_size)),
            transforms.Lambda(lambda x: x * 2 - 1)
        ])
        
        self.scene_transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Resize((scene_size, scene_size)),
            transforms.Normalize(
                    mean=[0.485, 0.456, 0.406],
                    std=[0.229, 0.224, 0.225]
                )
        ])

    def __len__(self):
        return len(self.final_image_paths)

    def __getitem__(self, index):
        image_path = self.final_image_paths[index]
        npz_file = np.load(image_path)
        
        object_mask = npz_file['object_mask']
        goal_mask = npz_file['goal_mask']
        scene = npz_file['scene']
        
        if index < self.half_len:
            image = object_mask
        else:
            image = goal_mask
        
        image = self.transform(image)
        scene = self.scene_transform(scene)
        return { "decision_target": image, "scene": scene}
    
