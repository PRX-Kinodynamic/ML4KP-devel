from .base_method import Method
from .data import GNNRadiusDataset, MLPRadiusDataset
from .model import GNNRadiusModel, MLPRadiusModel
from torch_geometric.loader import DataLoader
from torch.optim.lr_scheduler import StepLR
from tqdm import tqdm
import torch.nn as nn
import torch
import os
import numpy as np

def floor_to_step(value, step=0.05):
    """Floor the value to the nearest step"""
    return np.floor(value / step) * step

class GNNRadiusRegressor(Method):
    def __init__(self, name, config):
        self.name = name
        self.node_features = int(config['node_features'])
        self.edge_features = int(config['edge_features'])
        self.batch_size = int(config['batch_size'])
        self.device = config['device']
        self.epochs = int(config['epochs'])
        self.save_dir = config['save_dir']
        self.model_name = config['model_name']
        
        # Initialize model
        self.model = GNNRadiusModel(self.node_features, self.edge_features, config)
        self.model.to(self.device)

        # Initialize optimizer
        self.optimizer = torch.optim.Adam(
            self.model.parameters(), 
            lr=float(config.get('learning_rate', 0.001))
        )
        
        # Initialize scheduler
        self.scheduler = StepLR(
            self.optimizer, 
            step_size=int(config.get('scheduler_step_size', 10)),
            gamma=float(config.get('scheduler_gamma', 0.5))
        )
        
        # Loss function for regression
        self.criterion = nn.MSELoss()
        
        # Early stopping parameters
        self.patience = int(config.get('patience', 10))
        self.min_delta = float(config.get('min_delta', 1e-4))

    def train(self, training_data, evaluation_data, balance_ratio=None, verbose=False):
        train_dataset = GNNRadiusDataset(training_data)
        train_loader = DataLoader(train_dataset, batch_size=self.batch_size, shuffle=True)
        eval_dataset = GNNRadiusDataset(evaluation_data)
        eval_loader = DataLoader(eval_dataset, batch_size=self.batch_size, shuffle=False)

        best_eval_loss = float('inf')
        patience_counter = 0
        best_model_state = None

        for epoch in range(self.epochs):
            train_loss = self._train_epoch(train_loader, verbose)
            eval_loss, mae = self._eval_epoch(eval_loader, verbose)
            current_lr = self.scheduler.get_last_lr()[0]
            self.scheduler.step()

            if verbose:
                print(f"Epoch {epoch+1}/{self.epochs}")
                print(f"LR: {current_lr:.6f}, Train loss: {train_loss:.4f}, "
                      f"Eval loss: {eval_loss:.4f}, MAE: {mae:.4f}")

            if eval_loss < best_eval_loss - self.min_delta:
                best_eval_loss = eval_loss
                patience_counter = 0
                best_model_state = self.model.state_dict().copy()
                self._save_model(epoch, eval_loss, mae)
            else:
                patience_counter += 1
                if patience_counter >= self.patience:
                    if verbose:
                        print(f"Early stopping triggered after {epoch + 1} epochs")
                    self.model.load_state_dict(best_model_state)
                    break

    def _train_epoch(self, train_loader, verbose=False):
        self.model.train()
        total_loss = 0
        pbar = tqdm(train_loader) if verbose else train_loader
        
        for data in pbar:
            self.optimizer.zero_grad()
            data = data.to(self.device)
            output = self.model(data)
            target = data.y.view(-1, 1)
            loss = self.criterion(output, target) * 10
            loss.backward()
            self.optimizer.step()
            total_loss += loss.item()
            
        return total_loss / len(train_loader)

    def _eval_epoch(self, eval_loader, verbose=False):
        self.model.eval()
        total_loss = 0
        total_mae = 0
        pbar = tqdm(eval_loader) if verbose else eval_loader
        
        with torch.no_grad():
            for data in pbar:
                data = data.to(self.device)
                output = self.model(data)
                target = data.y.view(-1, 1)
                loss = self.criterion(output, target) * 10
                mae = torch.mean(torch.abs(output - target))
                total_loss += loss.item()
                total_mae += mae.item()

        return total_loss / len(eval_loader), total_mae / len(eval_loader)

    def predict(self, input_data):
        self.model.eval()
        dataset = GNNRadiusDataset([input_data])
        loader = DataLoader(dataset, batch_size=1, shuffle=False)
        
        with torch.no_grad():
            for data in loader:
                data = data.to(self.device)
                output = self.model(data)
                # Denormalize and floor predictions
                pred_radius = dataset.denormalize_radius(output.item())
                # pred_radius = floor_to_step(pred_radius)
                true_radius = dataset.denormalize_radius(data.y.item())
                return pred_radius, true_radius

    def _save_model(self, epoch, loss, mae):
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        checkpoint = {
            'epoch': epoch,
            'model_state_dict': self.model.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'scheduler_state_dict': self.scheduler.state_dict(),
            'loss': loss,
            'mae': mae
        }
        best_model_path = os.path.join(self.save_dir, f"{self.model_name}_best.pt")
        torch.save(checkpoint, best_model_path)
        print(f"Saved model to {best_model_path}")

    def load_model(self):
        model_path = os.path.join(self.save_dir, f"{self.model_name}_best.pt")
        checkpoint = torch.load(model_path)
        self.model.load_state_dict(checkpoint['model_state_dict'])
        self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
        self.scheduler.load_state_dict(checkpoint['scheduler_state_dict'])
        print(f"Loaded model from {model_path}")

    def get_name(self):
        return self.name 

class MLPRadiusRegressor(Method):
    def __init__(self, name, config):
        self.name = name
        self.input_dims = int(config['input_dims'])
        self.hidden_dims = config['hidden_dims']
        self.batch_size = int(config['batch_size'])
        self.device = config['device']
        self.epochs = int(config['epochs'])
        self.save_dir = config['save_dir']
        self.model_name = config['model_name']
        
        # Initialize model
        self.model = MLPRadiusModel(self.input_dims, self.hidden_dims)
        self.model.to(self.device)

        # Initialize optimizer
        self.optimizer = torch.optim.Adam(
            self.model.parameters(), 
            lr=float(config.get('learning_rate', 0.001))
        )
        
        # Initialize scheduler
        self.scheduler = StepLR(
            self.optimizer, 
            step_size=int(config.get('scheduler_step_size', 10)),
            gamma=float(config.get('scheduler_gamma', 0.5))
        )
        
        # Loss function for regression
        self.criterion = nn.MSELoss()
        
        # Early stopping parameters
        self.patience = int(config.get('patience', 10))
        self.min_delta = float(config.get('min_delta', 1e-4))

    def train(self, training_data, evaluation_data, balance_ratio=None, verbose=False):
        train_dataset = MLPRadiusDataset(training_data)
        train_loader = DataLoader(train_dataset, batch_size=self.batch_size, shuffle=True)
        eval_dataset = MLPRadiusDataset(evaluation_data)
        eval_loader = DataLoader(eval_dataset, batch_size=self.batch_size, shuffle=False)

        best_eval_loss = float('inf')
        patience_counter = 0
        best_model_state = None

        for epoch in range(self.epochs):
            train_loss = self._train_epoch(train_loader, verbose)
            eval_loss, mae = self._eval_epoch(eval_loader, verbose)
            current_lr = self.scheduler.get_last_lr()[0]
            self.scheduler.step()

            if verbose:
                print(f"Epoch {epoch+1}/{self.epochs}")
                print(f"LR: {current_lr:.6f}, Train loss: {train_loss:.4f}, "
                      f"Eval loss: {eval_loss:.4f}, MAE: {mae:.4f}")

            if eval_loss < best_eval_loss - self.min_delta:
                best_eval_loss = eval_loss
                patience_counter = 0
                best_model_state = self.model.state_dict().copy()
                self._save_model(epoch, eval_loss, mae)
            else:
                patience_counter += 1
                if patience_counter >= self.patience:
                    if verbose:
                        print(f"Early stopping triggered after {epoch + 1} epochs")
                    self.model.load_state_dict(best_model_state)
                    break

    def _train_epoch(self, train_loader, verbose=False):
        self.model.train()
        total_loss = 0
        pbar = tqdm(train_loader) if verbose else train_loader
        
        for features, target in pbar:
            self.optimizer.zero_grad()
            features = features.to(self.device)
            target = target.to(self.device).view(-1, 1)
            output = self.model(features)
            loss = self.criterion(output, target)
            loss.backward()
            self.optimizer.step()
            total_loss += loss.item()
            
        return total_loss / len(train_loader)

    def _eval_epoch(self, eval_loader, verbose=False):
        self.model.eval()
        total_loss = 0
        total_mae = 0
        pbar = tqdm(eval_loader) if verbose else eval_loader
        
        with torch.no_grad():
            for features, target in pbar:
                features = features.to(self.device)
                target = target.to(self.device).view(-1, 1)
                output = self.model(features)
                loss = self.criterion(output, target)
                mae = torch.mean(torch.abs(output - target))
                total_loss += loss.item()
                total_mae += mae.item()

        return total_loss / len(eval_loader), total_mae / len(eval_loader)

    def predict(self, input_data):
        self.model.eval()
        dataset = MLPRadiusDataset([input_data])
        loader = DataLoader(dataset, batch_size=1, shuffle=False)
        
        with torch.no_grad():
            for features, target in loader:
                features = features.to(self.device)
                output = self.model(features)
                # Denormalize and floor predictions
                pred_radius = dataset.denormalize_radius(output.item())
                # pred_radius = floor_to_step(pred_radius)
                true_radius = dataset.denormalize_radius(target.item())
                return pred_radius, true_radius

    def _save_model(self, epoch, loss, mae):
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        checkpoint = {
            'epoch': epoch,
            'model_state_dict': self.model.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'scheduler_state_dict': self.scheduler.state_dict(),
            'loss': loss,
            'mae': mae
        }
        best_model_path = os.path.join(self.save_dir, f"{self.model_name}_best.pt")
        torch.save(checkpoint, best_model_path)

    def load_model(self):
        model_path = os.path.join(self.save_dir, f"{self.model_name}_best.pt")
        checkpoint = torch.load(model_path)
        self.model.load_state_dict(checkpoint['model_state_dict'])
        self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
        self.scheduler.load_state_dict(checkpoint['scheduler_state_dict'])
        print(f"Loaded model from {model_path}")

    def get_name(self):
        return self.name