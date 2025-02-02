import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, random_split
from tqdm import tqdm
import matplotlib.pyplot as plt
from data import ImageDataset, InferenceDataset
import numpy as np
import os
from matplotlib.patches import Rectangle
class CNNClassifier(nn.Module):
    def __init__(self):
        super(CNNClassifier, self).__init__()
        self.features = nn.Sequential(
            # First conv block
            nn.Conv2d(3, 32, kernel_size=3, padding=1),
            nn.BatchNorm2d(32),
            nn.ReLU(),
            nn.MaxPool2d(2),
            
            # Second conv block
            nn.Conv2d(32, 64, kernel_size=3, padding=1),
            nn.BatchNorm2d(64),
            nn.ReLU(),
            nn.MaxPool2d(2),
            
            # Third conv block
            nn.Conv2d(64, 128, kernel_size=3, padding=1),
            nn.BatchNorm2d(128),
            nn.ReLU(),
            nn.MaxPool2d(2),
            
            # Flatten and FC layers
            nn.Flatten(),
            nn.Linear(128 * 32 * 32, 512),  # Adjust size based on your input
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(512, 1)
        )

    def forward(self, x):
        return self.features(x)

class Runner:
    def __init__(self, data_path, batch_size=32, learning_rate=0.001, num_epochs=20):
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.batch_size = batch_size
        self.learning_rate = learning_rate
        self.num_epochs = num_epochs
        self.model = CNNClassifier().to(self.device)
        self.criterion = nn.BCEWithLogitsLoss()
        
        # Add weight decay
        self.optimizer = optim.AdamW(
            self.model.parameters(), 
            lr=learning_rate,
            weight_decay=0.01
        )
        
        # Dataset loading
        dataset = ImageDataset(data_path)
        train_size = int(0.8 * len(dataset))
        val_size = len(dataset) - train_size
        self.train_dataset, self.val_dataset = random_split(dataset, [train_size, val_size])
        
        self.train_loader = DataLoader(
            self.train_dataset, 
            batch_size=batch_size, 
            shuffle=True,
            num_workers=4,
            pin_memory=True
        )
        self.val_loader = DataLoader(
            self.val_dataset, 
            batch_size=batch_size,
            num_workers=4,
            pin_memory=True
        )

        # OneCycleLR scheduler
        # total_steps = len(self.train_loader) * num_epochs
        # self.scheduler = torch.optim.lr_scheduler.OneCycleLR(
        #     self.optimizer,
        #     max_lr=learning_rate,
        #     total_steps=total_steps,
        #     pct_start=0.3,
        #     div_factor=25,  # initial_lr = max_lr/25
        #     final_div_factor=1000,  # final_lr = initial_lr/1000
        #     anneal_strategy='cos'
        # )

        # reduce on plateau
        self.scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
            self.optimizer, 
            mode='min', 
            factor=0.1, 
            patience=10, 
            verbose=True
        )

 
    def train_epoch(self):
        self.model.train()
        total_loss = 0
        correct = 0
        total = 0
        
        for batch_idx, (images, labels) in enumerate(tqdm(self.train_loader, desc="Training")):
            # Add data verification
            if batch_idx == 0:
                print(f"Batch shape: {images.shape}")
                print(f"Labels shape: {labels.shape}")
                print(f"Labels distribution: {labels.sum().item()}/{len(labels)}")
                print(f"Image range: {images.min().item():.3f} to {images.max().item():.3f}")
            
            images, labels = images.to(self.device), labels.float().to(self.device)
            self.optimizer.zero_grad()
            outputs = self.model(images)
            loss = self.criterion(outputs, labels)
            
            loss.backward()
            # Add gradient clipping
            torch.nn.utils.clip_grad_norm_(self.model.parameters(), max_norm=1.0)
            
            self.optimizer.step()
            
            total_loss += loss.item()
            predicted = (outputs > 0.5).float()
            total += labels.size(0)
            correct += (predicted == labels).sum().item()
            
            # Add loss printing every 100 batches
            if batch_idx % 100 == 0:
                print(f"Batch {batch_idx}, Loss: {loss.item():.4f}")
        
        return total_loss / len(self.train_loader), correct / total

    def validate(self):
        self.model.eval()
        total_loss = 0
        correct = 0
        total = 0
        
        with torch.no_grad():
            for images, labels in tqdm(self.val_loader, desc="Validation"):
                images, labels = images.to(self.device), labels.float().to(self.device)
                outputs = self.model(images)
                loss = self.criterion(outputs, labels)
                
                total_loss += loss.item()
                predicted = (outputs > 0.5).float()
                total += labels.size(0)
                # print(predicted, labels)
                correct += (predicted == labels).sum().item()
                
        return total_loss / len(self.val_loader), correct / total

    def train(self, save_path='models'):
        os.makedirs(save_path, exist_ok=True)
        best_val_acc = 0
        train_losses, train_accs = [], []
        val_losses, val_accs = [], []
        
        print("\nStarting training...")
        print("=" * 50) 
        
        for epoch in range(self.num_epochs):
            print(f"\nEpoch {epoch+1}/{self.num_epochs}")
            print("-" * 50)
            
            train_loss, train_acc = self.train_epoch()
            val_loss, val_acc = self.validate()

            self.scheduler.step(val_loss)
            
            train_losses.append(train_loss)
            train_accs.append(train_acc)
            val_losses.append(val_loss)
            val_accs.append(val_acc)
            
            # Detailed epoch summary
            print("\nEpoch Summary:")
            print(f"Training Loss:     {train_loss:.4f}")
            print(f"Training Accuracy: {train_acc:.4f} ({train_acc*100:.2f}%)")
            print(f"Validation Loss:   {val_loss:.4f}")
            print(f"Validation Accuracy: {val_acc:.4f} ({val_acc*100:.2f}%)")
            
            if val_acc > best_val_acc:
                best_val_acc = val_acc
                self.save_model(os.path.join(save_path, 'best_model_1.pth'))
                print(f"New best model saved! (Validation Accuracy: {val_acc*100:.2f}%)")
        
        print("\nTraining completed!")
        print(f"Best validation accuracy: {best_val_acc*100:.2f}%")
        self.plot_training_history(train_losses, val_losses, train_accs, val_accs)

    def save_model(self, path):
        torch.save({
            'model_state_dict': self.model.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
        }, path)
        print(f"Model saved to {path}")

    def load_model(self, path):
        checkpoint = torch.load(path)
        self.model.load_state_dict(checkpoint['model_state_dict'])
        self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
        print(f"Model loaded from {path}")

    def plot_training_history(self, train_losses, val_losses, train_accs, val_accs):
        plt.figure(figsize=(12, 4))
        
        plt.subplot(1, 2, 1)
        plt.plot(train_losses, label='Train Loss')
        plt.plot(val_losses, label='Val Loss')
        plt.xlabel('Epoch')
        plt.ylabel('Loss')
        plt.legend()
        
        plt.subplot(1, 2, 2)
        plt.plot(train_accs, label='Train Acc')
        plt.plot(val_accs, label='Val Acc')
        plt.xlabel('Epoch')
        plt.ylabel('Accuracy')
        plt.legend()
        
        plt.tight_layout()
        plt.savefig('training_history.png')
        plt.close()

    def infer_and_plot(self, inference_path):
        # Check if data file exists
        if False and os.path.exists('map_data_3.csv'):
            print("Loading existing data from map_data_3.csv")
            data_plot = np.loadtxt('map_data_3.csv', delimiter=',', skiprows=1)
            robot_pos = np.loadtxt(inference_path, delimiter=',', skiprows=1)[0, :2]  # Get robot position from first row
        else:
            print("Generating new data...")
            inference_dataset = InferenceDataset(inference_path)
            inference_loader = DataLoader(inference_dataset, batch_size=1, shuffle=False, num_workers=1, pin_memory=True)

            data_plot = []
            self.model.eval()
            with torch.no_grad():
                for images, labels in tqdm(inference_loader, desc="Inference"):
                    images = images.to(self.device)
                    outputs = torch.sigmoid(self.model(images))
                    data_plot.append(labels.view(-1)[2:].numpy().tolist() + outputs.view(-1).detach().cpu().numpy().tolist())
            
            data_plot = np.array(data_plot)
            robot_pos = labels.view(-1)[:2].numpy().tolist()
            
            # Save the raw data
            np.savetxt('map_data_3.csv', data_plot, delimiter=',', 
                       header='goal_x,goal_y,success_prob', comments='')

        # Create scatter plot
        plt.figure(figsize=(10, 8))
        
        # Create a scatter plot first to use for colorbar
        scatter = plt.scatter(data_plot[:,0], data_plot[:,1], 
                             c=data_plot[:,2],
                             cmap='Greys',
                             alpha=0)  # Make scatter invisible
        
        # Add rectangles
        for i in range(len(data_plot)):
            rect = Rectangle((data_plot[i,0]-0.05, data_plot[i,1]-0.05), 0.1, 0.1, 
                            facecolor=plt.cm.Greys(data_plot[i,2]),
                            alpha=0.6)
            plt.gca().add_patch(rect)

        plt.scatter(robot_pos[0], robot_pos[1], c='black', s=100, alpha=1)
        
        # Add colorbar using the scatter plot
        plt.colorbar(scatter, label='Success Probability')
        
        plt.xlabel('Goal X Position')
        plt.ylabel('Goal Y Position')
        plt.title('Success Probability Map')
        plt.axis('equal')

        # Save the plot
        plt.savefig('rect_success_map_3.png')
        plt.close()

        # Save the raw data
        np.savetxt('map_data_3.csv', data_plot, delimiter=',', 
                   header='goal_x,goal_y,success_prob', comments='')

if __name__ == '__main__':
    # Example usage
    data_path = ['/media/dhruv/a7519aee-b272-44ae-a117-1f1ea1796db6/2024/NAMO/dataset_1', '/media/dhruv/a7519aee-b272-44ae-a117-1f1ea1796db6/2024/NAMO/dataset_2', '/media/dhruv/a7519aee-b272-44ae-a117-1f1ea1796db6/2024/NAMO/dataset_3']

    inference_path = '/media/dhruv/a7519aee-b272-44ae-a117-1f1ea1796db6/2024/NAMO/test_dataset_3/positions.csv'

    runner = Runner(data_path, batch_size=32, learning_rate=0.001, num_epochs=30)
    # runner.train()
    runner.load_model('models/best_model.pth')
    runner.infer_and_plot(inference_path)