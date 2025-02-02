# torch image dataset, class parameter is folder that contains folders positive and negative that contain images
# convert images to tensor and normalize them
# resize to 256x256
# dont convert to grayscale
# return label as well

from torch.utils.data import Dataset
from glob import glob
from PIL import Image
from torchvision import transforms
import torch
import os
import numpy as np
import matplotlib.pyplot as plt
import random
import pandas as pd

class ImageDataset(Dataset):
    def __init__(self, folders):
        self.images = []
        for root in folders:
            self.images.extend(glob(os.path.join(root, '*', '*.png')))
        random.shuffle(self.images)

        # self.images = self.images[:500]  # to check if neural network is working

        self.positive_images = [image for image in self.images if 'positive' in image]
        self.negative_images = [image for image in self.images if 'negative' in image]

        # balance the dataset
        min_len = min(len(self.positive_images), len(self.negative_images))
        self.positive_images = self.positive_images[:min_len]
        self.negative_images = self.negative_images[:min_len]

        self.images = self.positive_images + self.negative_images

        random.shuffle(self.images)

    def __len__(self):
        return len(self.images)
    
    def __getitem__(self, idx):
        image_path = self.images[idx]
        image = Image.open(image_path)
        image = image.convert('RGB')
        
        # Add data augmentation for training
        transform = transforms.Compose([
            transforms.Resize((256, 256)),
            # transforms.RandomHorizontalFlip(),
            # transforms.RandomRotation(10),
            # transforms.ColorJitter(brightness=0.2, contrast=0.2),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], 
                               std=[0.229, 0.224, 0.225])
        ])
        
        image = transform(image)
        return image, torch.tensor([1 if 'positive' in image_path else 0])
    

class InferenceDataset(Dataset):
    
    def __init__(self, csv_path):
        self.df = pd.read_csv(csv_path)

    def __len__(self):
        return len(self.df)
    
    def __getitem__(self, idx):
        row = self.df.iloc[idx]
        image = Image.open(row['image_path'])
        image = image.convert('RGB')
        transform = transforms.Compose([
            transforms.Resize((256, 256)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], 
                               std=[0.229, 0.224, 0.225])
        ])
        image = transform(image)
        robot_x, robot_y, goal_x, goal_y = row['robot_x'], row['robot_y'], row['goal_x'], row['goal_y']
        
        return image, torch.tensor([robot_x, robot_y, goal_x, goal_y])
    
if __name__ == '__main__':
    data = ImageDataset(['/media/dhruv/a7519aee-b272-44ae-a117-1f1ea1796db6/2024/NAMO/dataset_1', '/media/dhruv/a7519aee-b272-44ae-a117-1f1ea1796db6/2024/NAMO/dataset_2', '/media/dhruv/a7519aee-b272-44ae-a117-1f1ea1796db6/2024/NAMO/dataset_3'])
    for i in range(50):
        # if data[i][1] == 0:
        print(data[i][1])
        plt.imshow(torch.permute(data[i][0], (1, 2, 0)).numpy())
        plt.pause(0.5)