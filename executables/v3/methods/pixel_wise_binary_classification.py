import os
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader
import matplotlib.pyplot as plt
from tqdm import tqdm
import time

from .base_method import Method
from .model import get_model
from .data import Img2ImgDataset

class PixelWiseBinaryClassification(Method):
    """Pixel-wise binary classification using UNet architecture."""
    
    def __init__(self, name, config):
        """
        Initialize the pixel-wise binary classification method.
        
        Args:
            name (str): Name of the method
            config (dict): Configuration parameters
        """
        self.name = name
        
        # Extract configuration parameters
        self.device = config.get('device', torch.device('cuda' if torch.cuda.is_available() else 'cpu'))
        self.batch_size = int(config.get('batch_size', 8))
        self.epochs = int(config.get('epochs', 100))
        self.learning_rate = float(config.get('learning_rate', 0.001))
        self.save_dir = config.get('save_dir', 'saved_models')
        self.model_name = config.get('model_name', 'unet_segmentation')
        
        # Early stopping parameters
        self.patience = int(config.get('patience', 10))
        self.min_delta = float(config.get('min_delta', 1e-4))
        
        # Scheduler parameters
        self.factor = float(config.get('factor', 0.5))
        self.scheduler_patience = int(config.get('scheduler_patience', 5))
        
        # Additional parameters specific to image segmentation
        self.n_channels = int(config.get('n_channels', 3))
        self.n_classes = int(config.get('n_classes', 1))
        self.classification_threshold = float(config.get('classification_threshold', 0.5))
        
        # Loss function configuration
        self.loss_type = config.get('loss_type', 'bce')  # Options: 'bce', 'dice', 'focal', 'combo', 'tversky', 'mse'
        self.focal_alpha = float(config.get('focal_alpha', 0.25))
        self.focal_gamma = float(config.get('focal_gamma', 2.0))
        self.dice_smooth = float(config.get('dice_smooth', 1e-6))
        self.tversky_alpha = float(config.get('tversky_alpha', 0.7))  # Controls FP weight
        self.tversky_beta = float(config.get('tversky_beta', 0.3))    # Controls FN weight
        self.combo_alpha = float(config.get('combo_alpha', 0.5))      # Weight for BCE in combo loss
        
        # Initialize model
        self.model = get_model(n_channels=self.n_channels, n_classes=self.n_classes)
        self.model.to(self.device)
        
        # Initialize optimizer
        self.optimizer = optim.Adam(self.model.parameters(), lr=self.learning_rate)
        
        # Initialize scheduler (keeping ReduceLROnPlateau as requested)
        self.scheduler = optim.lr_scheduler.ReduceLROnPlateau(
            self.optimizer, mode='min', factor=self.factor, 
            patience=self.scheduler_patience, verbose=True
        )
        
        # Initialize loss function
        self.criterion = self._get_loss_function()
        
        # Create save directory if it doesn't exist
        os.makedirs(self.save_dir, exist_ok=True)
        
        # Best validation loss for model saving
        self.best_val_loss = float('-inf')
    
    def train(self, training_data, evaluation_data, balance_ratio=None, verbose=False):
        """
        Train the model given training data.
        
        Args:
            training_data (str): Path to training data directory
            evaluation_data (str): Path to evaluation data directory
            balance_ratio (float): Ratio for balancing positive/negative samples
            verbose (bool): Whether to print detailed progress
            
        Returns:
            dict: Training history and metrics
        """
        # Setup loss function with class balancing if needed
        if self.loss_type == 'bce' and balance_ratio is not None and balance_ratio != 1.0:
            pos_weight = torch.tensor([balance_ratio]).to(self.device)
            self.criterion = self._get_loss_function(pos_weight=pos_weight)
        else:
            self.criterion = self._get_loss_function()
        
        # Setup datasets and dataloaders
        train_dataset = Img2ImgDataset(training_data)
        val_dataset = Img2ImgDataset(evaluation_data)
        
        train_loader = DataLoader(train_dataset, batch_size=self.batch_size, 
                                 shuffle=True, num_workers=4)
        val_loader = DataLoader(val_dataset, batch_size=self.batch_size, 
                               shuffle=False, num_workers=4)
        
        # Training setup
        start_time = time.time()
        history = {
            'train_loss': [],
            'val_loss': [],
            'train_dice': [],
            'val_dice': [],
            'lr': []
        }
        
        best_epoch = 0
        epochs_no_improve = 0
        best_model_state = None
        
        for epoch in range(self.epochs):
            # Training phase
            train_loss, train_dice = self._train_epoch(train_loader, verbose)
            
            # Validation phase
            val_loss, val_dice = self._evaluate(val_loader, verbose)
            
            # Get current learning rate
            current_lr = self.optimizer.param_groups[0]['lr']
            
            # Update learning rate scheduler
            self.scheduler.step(val_loss)
            
            # Save metrics
            history['train_loss'].append(train_loss)
            history['val_loss'].append(val_loss)
            history['train_dice'].append(train_dice)
            history['val_dice'].append(val_dice)
            history['lr'].append(current_lr)
            
            # Print epoch results
            if verbose:
                print(f'Epoch {epoch+1}/{self.epochs} - '
                      f'Train Loss: {train_loss:.4f}, Dice: {train_dice:.4f} | '
                      f'Val Loss: {val_loss:.4f}, Dice: {val_dice:.4f} | '
                      f'LR: {current_lr:.6f}')
            
            # Check for improvement
            if val_dice > self.best_val_loss:
                self.best_val_loss = val_dice
                best_epoch = epoch
                epochs_no_improve = 0
                best_model_state = self.model.state_dict().copy()
                
                # Save best model
                self._save_model(epoch, val_loss, val_dice)
                if verbose:
                    print(f'Model improved, saved checkpoint at epoch {epoch+1}')
            else:
                epochs_no_improve += 1
                if verbose and epochs_no_improve > 0:
                    print(f'No improvement for {epochs_no_improve} epochs')
            
            # Early stopping
            if epochs_no_improve >= self.patience:
                if verbose:
                    print(f'Early stopping triggered after {epoch+1} epochs')
                # Load best model state
                self.model.load_state_dict(best_model_state)
                break
        
        # Training complete
        total_time = time.time() - start_time
        if verbose:
            print(f'Training completed in {total_time:.2f} seconds')
            print(f'Best model found at epoch {best_epoch+1} with validation loss {self.best_val_loss:.4f}')
        
        # Plot training history if verbose
        if verbose:
            self._plot_training_history(history)
        
        return {
            'best_val_loss': self.best_val_loss,
            'epochs_trained': epoch + 1,
            'training_time': total_time,
            'history': history
        }
    
    def _train_epoch(self, train_loader, verbose=False):
        """
        Train for one epoch.
        
        Args:
            train_loader: DataLoader for training data
            verbose: Whether to show progress bar
            
        Returns:
            tuple: (average loss, average dice coefficient)
        """
        self.model.train()
        train_loss = 0.0
        train_dice = 0.0
        train_batches = 0
        
        train_progress = train_loader
        if verbose:
            train_progress = tqdm(train_loader, desc='Training')
            
        for inputs, targets in train_progress:
            # Move data to device
            inputs = self._prepare_inputs(inputs)
            targets = self._prepare_targets(targets)
            
            # Zero gradients
            self.optimizer.zero_grad()
            
            # Forward pass
            outputs = self.model(inputs)
            loss = self.criterion(outputs, targets)
            
            # Backward pass and optimize
            loss.backward()
            self.optimizer.step()
            
            # Update metrics
            train_loss += loss.item()
            train_dice += self._dice_coefficient(outputs, targets)
            train_batches += 1
        
        return train_loss / train_batches, train_dice / train_batches
    
    def _evaluate(self, dataloader, verbose=False):
        """
        Evaluate the model on a dataset.
        
        Args:
            dataloader: DataLoader for the dataset
            verbose: Whether to show progress bar
            
        Returns:
            tuple: (average loss, average dice coefficient)
        """
        self.model.eval()
        val_loss = 0.0
        val_dice = 0.0
        val_batches = 0
        
        val_progress = dataloader
        if verbose:
            val_progress = tqdm(dataloader, desc='Validation')
            
        with torch.no_grad():
            for inputs, targets in val_progress:
                # Move data to device
                inputs = self._prepare_inputs(inputs)
                targets = self._prepare_targets(targets)
                
                # Forward pass
                outputs = self.model(inputs)
                loss = self.criterion(outputs, targets)
                
                # Update metrics
                val_loss += loss.item()
                val_dice += self._dice_coefficient(outputs, targets)
                val_batches += 1
        
        return val_loss / val_batches, val_dice / val_batches
    
    def predict(self, input_data):
        """
        Predict the mask for the input image.
        
        Args:
            input_data: Input image (numpy array or path to image)
            
        Returns:
            Predicted mask, raw probabilities, and target (if available)
        """
        self.model.eval()
        
        # Handle different input types
        if isinstance(input_data, str) and os.path.exists(input_data):
            # Load from file if it's a path
            data = np.load(input_data)
            input_img = data['inp']
            target = data['out'][:, :, 0] + data['out'][:, :, 1]
        elif isinstance(input_data, tuple) and len(input_data) == 2:
            # Handle input as (image, target) pair
            input_img, target = input_data
        else:
            # Otherwise assume it's just the image
            input_img = input_data
            target = None
        
        with torch.no_grad():
            # Prepare input
            inputs = self._prepare_inputs(input_img)
            
            # Forward pass
            outputs = self.model(inputs)
            
            # Handle outputs based on loss type
            if self.loss_type == 'mse':
                # For MSE loss, outputs are already in the right range
                probs = outputs
                pred = (probs > self.classification_threshold).float()
            else:
                # For other losses, apply sigmoid to get probabilities
                probs = outputs
                pred = (torch.sigmoid(probs) > self.classification_threshold).float()
            
            # If target is available, prepare it too
            if target is not None:
                target_tensor = self._prepare_targets(target)
                return input_img, probs, pred, target_tensor
            
            # Return outputs for interface
            return input_img, probs, pred, None
    
    def get_name(self):
        """Name of the method."""
        return self.name
    
    def get_classification_threshold(self):
        """Get the classification threshold."""
        return self.classification_threshold
    
    def _prepare_inputs(self, inputs):
        """
        Prepare inputs for the model.
        
        Args:
            inputs: Input data (numpy array or torch tensor)
            
        Returns:
            torch.Tensor: Prepared inputs on the correct device
        """
        if isinstance(inputs, np.ndarray):
            inputs = torch.from_numpy(inputs).float()
        else:
            inputs = inputs.float()
        
        # Ensure inputs are in the format [batch_size, channels, height, width]
        if len(inputs.shape) == 3:  # [height, width, channels]
            inputs = inputs.permute(2, 0, 1)  # [channels, height, width]
            inputs = inputs.unsqueeze(0)  # [1, channels, height, width]
        elif len(inputs.shape) == 4 and inputs.shape[1] != self.n_channels:  # [batch_size, height, width, channels]
            inputs = inputs.permute(0, 3, 1, 2)  # [batch_size, channels, height, width]
        
        return inputs.to(self.device)
    
    def _prepare_targets(self, targets):
        """
        Prepare targets for the model.
        
        Args:
            targets: Target masks (numpy array or torch tensor)
            
        Returns:
            torch.Tensor: Prepared targets on the correct device
        """
        if isinstance(targets, np.ndarray):
            targets = torch.from_numpy(targets).float()
        else:
            targets = targets.float()
        
        # Ensure targets are in the format [batch_size, 1, height, width] for binary segmentation
        if len(targets.shape) == 3:  # [height, width, 1] or [batch_size, height, width]
            if targets.shape[2] == self.n_classes:  # [height, width, 1]
                targets = targets.permute(2, 0, 1)  # [1, height, width]
            targets = targets.unsqueeze(1)  # [batch_size, 1, height, width]
        elif len(targets.shape) == 2:  # [height, width]
            targets = targets.unsqueeze(0).unsqueeze(0)  # [1, 1, height, width]
            
        elif len(targets.shape) == 4 and targets.shape[1] != self.n_classes:  # [batch_size, height, width, channels]
            targets = targets.permute(0, 3, 1, 2)  # [batch_size, channels, height, width]
        
        return targets.to(self.device)
    
    def _dice_coefficient(self, outputs, targets, smooth=1e-6):
        """
        Calculate Dice coefficient.
        
        Args:
            outputs: Model outputs (logits)
            targets: Ground truth masks
            smooth: Smoothing factor to avoid division by zero
            
        Returns:
            float: Dice coefficient
        """
        # Apply sigmoid to outputs
        probs = torch.sigmoid(outputs)
        
        # Flatten the predictions and targets
        probs = probs.reshape(-1)  # Using reshape instead of view
        targets = targets.reshape(-1)  # Using reshape instead of view
        
        # Calculate intersection and union
        intersection = (probs * targets).sum()
        union = probs.sum() + targets.sum()
        
        # Calculate Dice coefficient
        dice = (2. * intersection + smooth) / (union + smooth)
        
        return dice.item()
    
    def _save_model(self, epoch, loss, dice_score):
        """
        Save model checkpoint with metadata.
        
        Args:
            epoch: Current epoch
            loss: Validation loss
            dice_score: Dice coefficient
        """
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            
        checkpoint = {
            'epoch': epoch,
            'model_state_dict': self.model.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'scheduler_state_dict': self.scheduler.state_dict(),
            'loss': loss,
            'dice_score': dice_score,
            'best_val_loss': self.best_val_loss
        }
        
        # Save as best model
        best_model_path = os.path.join(self.save_dir, f"{self.model_name}_best.pt")
        torch.save(checkpoint, best_model_path)
        print(f"Saved best model to {best_model_path}")
    
    def load_model(self):
        """Load the best model checkpoint."""
        model_path = os.path.join(self.save_dir, f"{self.model_name}_best.pt")
        if os.path.exists(model_path):
            checkpoint = torch.load(model_path, map_location=self.device)
            self.model.load_state_dict(checkpoint['model_state_dict'])
            if 'optimizer_state_dict' in checkpoint:
                self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
            if 'scheduler_state_dict' in checkpoint:
                self.scheduler.load_state_dict(checkpoint['scheduler_state_dict'])
            if 'best_val_loss' in checkpoint:
                self.best_val_loss = checkpoint['best_val_loss']
            print(f"Loaded model from {model_path}")
            return True
        else:
            print(f"No model found at {model_path}")
            return False
    
    def _plot_training_history(self, history):
        """
        Plot training history.
        
        Args:
            history: Dictionary containing training metrics
        """
        plt.figure(figsize=(15, 10))
        
        # Plot loss
        plt.subplot(2, 2, 1)
        plt.plot(history['train_loss'], label='Training Loss')
        plt.plot(history['val_loss'], label='Validation Loss')
        plt.title('Loss')
        plt.xlabel('Epoch')
        plt.ylabel('Loss')
        plt.legend()
        
        # Plot Dice coefficient
        plt.subplot(2, 2, 2)
        plt.plot(history['train_dice'], label='Training Dice')
        plt.plot(history['val_dice'], label='Validation Dice')
        plt.title('Dice Coefficient')
        plt.xlabel('Epoch')
        plt.ylabel('Dice')
        plt.legend()
        
        # Plot learning rate
        plt.subplot(2, 2, 3)
        plt.plot(history['lr'])
        plt.title('Learning Rate')
        plt.xlabel('Epoch')
        plt.ylabel('LR')
        plt.yscale('log')
        
        plt.tight_layout()
        plt.savefig(f'{self.save_dir}/training_history.png')
        plt.close()
    
    def _get_loss_function(self, pos_weight=None):
        """
        Get the appropriate loss function based on configuration.
        
        Args:
            pos_weight: Optional weight for positive class
            
        Returns:
            Loss function
        """
        if self.loss_type == 'bce':
            if pos_weight is not None:
                return nn.BCEWithLogitsLoss(pos_weight=pos_weight)
            return nn.BCEWithLogitsLoss()
        
        elif self.loss_type == 'mse':
            return self._mse_loss
        
        elif self.loss_type == 'dice':
            return self._dice_loss
        
        elif self.loss_type == 'focal':
            return self._focal_loss
        
        elif self.loss_type == 'combo':
            return self._combo_loss
        
        elif self.loss_type == 'tversky':
            return self._tversky_loss
        
        else:
            print(f"Warning: Unknown loss type '{self.loss_type}', defaulting to BCE")
            return nn.BCEWithLogitsLoss()
    
    def _dice_loss(self, outputs, targets, smooth=None):
        """
        Dice Loss - directly optimizes the Dice coefficient.
        
        Args:
            outputs: Model outputs (logits)
            targets: Ground truth masks
            smooth: Smoothing factor to avoid division by zero
            
        Returns:
            torch.Tensor: Dice loss
        """
        if smooth is None:
            smooth = self.dice_smooth
        
        # Apply sigmoid to outputs
        probs = torch.sigmoid(outputs)
        
        # Flatten the predictions and targets
        probs = probs.reshape(-1)  # Using reshape instead of view
        targets = targets.reshape(-1)  # Using reshape instead of view
        
        # Calculate intersection and union
        intersection = (probs * targets).sum()
        union = probs.sum() + targets.sum()
        
        # Calculate Dice coefficient and loss
        dice = (2. * intersection + smooth) / (union + smooth)
        
        return 1 - dice  # Return loss (1 - dice coefficient)
    
    def _focal_loss(self, outputs, targets, alpha=None, gamma=None):
        """
        Focal Loss - modified cross-entropy that down-weights easy examples.
        
        Args:
            outputs: Model outputs (logits)
            targets: Ground truth masks
            alpha: Weighting factor
            gamma: Focusing parameter
            
        Returns:
            torch.Tensor: Focal loss
        """
        if alpha is None:
            alpha = self.focal_alpha
        if gamma is None:
            gamma = self.focal_gamma
        
        # Apply sigmoid to get probabilities
        probs = torch.sigmoid(outputs)
        
        # Calculate binary cross entropy
        bce = nn.functional.binary_cross_entropy(probs, targets, reduction='none')
        
        # Calculate focal weight
        p_t = probs * targets + (1 - probs) * (1 - targets)
        focal_weight = (1 - p_t) ** gamma
        
        # Apply alpha weighting
        if alpha is not None:
            focal_weight = alpha * targets * focal_weight + (1 - alpha) * (1 - targets) * focal_weight
        
        return (focal_weight * bce).mean()
    
    def _tversky_loss(self, outputs, targets, alpha=None, beta=None, smooth=None):
        """
        Tversky Loss - generalized Dice with adjustable FP/FN weights.
        
        Args:
            outputs: Model outputs (logits)
            targets: Ground truth masks
            alpha: False positive weight
            beta: False negative weight
            smooth: Smoothing factor
            
        Returns:
            torch.Tensor: Tversky loss
        """
        if alpha is None:
            alpha = self.tversky_alpha
        if beta is None:
            beta = self.tversky_beta
        if smooth is None:
            smooth = self.dice_smooth
        
        # Apply sigmoid to outputs
        probs = torch.sigmoid(outputs)
        
        # Flatten the predictions and targets
        probs = probs.reshape(-1)  # Using reshape instead of view
        targets = targets.reshape(-1)  # Using reshape instead of view
        
        # Calculate true positives, false positives, and false negatives
        tp = (probs * targets).sum()
        fp = (probs * (1 - targets)).sum()
        fn = ((1 - probs) * targets).sum()
        
        # Calculate Tversky index
        tversky = (tp + smooth) / (tp + alpha * fp + beta * fn + smooth)
        
        return 1 - tversky  # Return loss (1 - Tversky index)
    
    def _combo_loss(self, outputs, targets, alpha=None):
        """
        Combo Loss - combination of BCE and Dice loss.
        
        Args:
            outputs: Model outputs (logits)
            targets: Ground truth masks
            alpha: Weight for BCE loss component
            
        Returns:
            torch.Tensor: Combined loss
        """
        if alpha is None:
            alpha = self.combo_alpha
        
        bce_loss = nn.functional.binary_cross_entropy_with_logits(outputs, targets)
        dice_loss = self._dice_loss(outputs, targets)
        
        return alpha * bce_loss + (1 - alpha) * dice_loss

    def _mse_loss(self, outputs, targets):
        """
        MSE Loss for heatmap prediction.
        
        Args:
            outputs: Model outputs (should be raw values, not logits)
            targets: Ground truth heatmaps
            
        Returns:
            torch.Tensor: MSE loss
        """
        # For MSE loss, we don't want to apply sigmoid to the outputs
        # since we're directly predicting the heatmap values
        return nn.functional.mse_loss(outputs, targets)

