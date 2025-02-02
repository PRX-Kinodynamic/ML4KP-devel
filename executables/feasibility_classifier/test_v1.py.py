# %%
import torch
import numpy as np
import os
from glob import glob
from tqdm import tqdm

# %%


# %%
from torch.utils.data import DataLoader, Dataset
from PIL import Image
from torchvision import transforms
import matplotlib.pyplot as plt
import random


data_folder = "/media/dhruv/a7519aee-b272-44ae-a117-1f1ea1796db6/2024/NAMO/test1/data_images"

class FeasibilityDataset(Dataset):
    def __init__(self, datafiles):
        # self.data_folder = data_folder
        # success_data = glob(os.path.join(data_folder, 'success', '*.png'))
        # failure_data = glob(os.path.join(data_folder, 'failure', '*.png'))
        # min_data_len = min(len(success_data), len(failure_data))
        # print(min_data_len)
        self.datafiles = datafiles # success_data[:min_data_len] + failure_data[:min_data_len]
        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            # transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

    def __len__(self):
        return len(self.datafiles)

    def __getitem__(self, idx):
        image_path = self.datafiles[idx]
        if 'success' in image_path:
            label = [1]
        else:
            label = [0]
        image = Image.open(image_path)
        image = image.convert('RGB')
        image = 2 * self.transform(image) - 1
        return image, torch.tensor(label)

# train test split

success_data = glob(os.path.join(data_folder, 'success', '*.png'))
failure_data = glob(os.path.join(data_folder, 'failure', '*.png'))
min_data_len = min(len(success_data), len(failure_data))
print(min_data_len)
data = success_data[:min_data_len] + failure_data[:min_data_len]
random.shuffle(data)
train_data = data[:int(1 * len(data))]
train_dataset = FeasibilityDataset(train_data)
train_dataloader = DataLoader(train_dataset, batch_size=32, shuffle=True, pin_memory=False)

data_folder = "/media/dhruv/a7519aee-b272-44ae-a117-1f1ea1796db6/2024/NAMO/data_images_10"
test_success_data = glob(os.path.join(data_folder, 'success', '*.png')) 
test_failure_data = glob(os.path.join(data_folder, 'failure', '*.png'))
min_test_data_len = min(len(test_success_data), len(test_failure_data))
test_data = test_success_data[:min(32*5, min_test_data_len)] + test_failure_data[:min(32*5, min_test_data_len)]
random.shuffle(test_data)
test_dataset = FeasibilityDataset(test_data)
test_dataloader = DataLoader(test_dataset, batch_size=32, shuffle=False, pin_memory=False)

# %%
# build a cnn model for classification
import torch.nn as nn

class FeasibilityClassifier(nn.Module):
    def __init__(self):
        super(FeasibilityClassifier, self).__init__()
        
        # Convolutional layers
        self.conv_layers = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
            
            nn.Conv2d(32, 64, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
            
            nn.Conv2d(64, 128, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
            
            nn.Conv2d(128, 256, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
        )
        
        # Fully connected layers
        self.fc_layers = nn.Sequential(
            nn.Linear(256 * 14 * 14, 512),
            nn.ReLU(),
            nn.Dropout(0.5),
            
            nn.Linear(512, 128),
            nn.ReLU(),
            nn.Dropout(0.5),
            
            nn.Linear(128, 1),
            nn.Sigmoid()
        )
        
    def forward(self, x):
        x = self.conv_layers(x)
        x = x.view(x.size(0), -1)  # Flatten
        x = self.fc_layers(x)
        return x
    
model = FeasibilityClassifier()
optimizer = torch.optim.Adam(model.parameters(), lr=0.0001)
criterion = nn.BCELoss()

# %%
from torch.optim.lr_scheduler import ReduceLROnPlateau

# Early stopping parameters
patience = 10
best_val_loss = float('inf')
patience_counter = 0
best_model_state = None



# Training loop
num_epochs = 50
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = model.to(device)

# Add learning rate scheduler
scheduler = ReduceLROnPlateau(optimizer, mode='min', factor=0.5, patience=5, verbose=True)

for epoch in range(num_epochs):
    # Training phase
    model.train()
    train_running_loss = 0.0
    
    for images, labels in tqdm(train_dataloader, desc=f"Epoch {epoch+1}/{num_epochs} - Training"):
        images = images.to(device)
        labels = labels.to(device).float()
        
        # Forward pass
        outputs = model(images)
        loss = criterion(outputs, labels)
        
        # Backward pass and optimize
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        
        train_running_loss += loss.item()
    
    train_epoch_loss = train_running_loss / len(train_dataloader)
    
    # Validation phase
    model.eval()
    val_running_loss = 0.0
    
    with torch.no_grad():
        for images, labels in tqdm(test_dataloader, desc=f"Epoch {epoch+1}/{num_epochs} - Validation"):
            images = images.to(device)
            labels = labels.to(device).float()
            
            outputs = model(images)
            loss = criterion(outputs, labels)
            val_running_loss += loss.item()
    
    val_epoch_loss = val_running_loss / len(test_dataloader)
    print(f"Epoch [{epoch+1}/{num_epochs}], Train Loss: {train_epoch_loss:.4f}, Val Loss: {val_epoch_loss:.4f}")

    # Learning rate scheduling
    scheduler.step(val_epoch_loss)
    

    # Early stopping
    if val_epoch_loss < best_val_loss:
        best_val_loss = val_epoch_loss
        patience_counter = 0
        best_model_state = model.state_dict()
    else:
        patience_counter += 1
        if patience_counter >= patience:
            print("Early stopping triggered.")
            break

# save the model
torch.save(best_model_state, 'feasibility_classifier_1.pth')

# %%
model.load_state_dict(torch.load('feasibility_classifier_1.pth'))
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model.to(device)

# %%
from IPython.display import display, clear_output
import ipywidgets as widgets

model.eval()
# Create widgets
fig, ax = plt.subplots(figsize=(8, 8))
button = widgets.Button(description="Next")
output = widgets.Output()

# Display widgets once
display(button, output)

# Iterator for the dataloader
vis_test_dataloader = iter(DataLoader(test_dataset, batch_size=1, shuffle=True, pin_memory=False))

def show_next_image(b):
    with output:
        clear_output(wait=True)
        try:
            # Get next batch
            images, labels = next(vis_test_dataloader)
            
            
            # Show image
            ax.clear()
            ax.imshow((images[0].permute(1, 2, 0) + 1) / 2)
            display(fig)
            
            # Process prediction
            images = images.to(device)
            labels = labels.to(device).float()
            with torch.no_grad():
                outputs = model(images)
            print("prediction: ", np.round(outputs[0].item(), 2), "label: ", labels[0].item())
            print(((outputs[0].item() > 0.5) * 1.0) == (labels[0].item() * 1.0))
            # print((outputs > 0.5) == labels, )
            
        except StopIteration:
            print("End of dataset reached")
            button.disabled = True

# Connect button to function
button.on_click(show_next_image)

# %%
vis_test_dataloader = iter(DataLoader(test_dataset, batch_size=1, shuffle=True, pin_memory=False))
model.eval()
correct_pred = 0
total_pred = 0
for img, label in tqdm(vis_test_dataloader):
    img = img.to(device)
    label = label.to(device).float()
    with torch.no_grad():
        outputs = model(img)
    if ((outputs > 0.5) * 1.0) == (label.item() * 1.0):
        correct_pred += 1
    total_pred += 1
print(f"Accuracy: {correct_pred / total_pred * 100:.2f}%")


# %%



