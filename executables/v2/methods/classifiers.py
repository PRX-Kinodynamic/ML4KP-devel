from .base_method import Method
from .data import GNNClassifierDataset, MLPClassifierDataset
from torch_geometric.loader import DataLoader
from .model import GNNClassifierModel, MLPClassifierModel
from torch.optim.lr_scheduler import StepLR
from tqdm import tqdm
import torch.nn as nn
import torch
import os

class GNNClassifier(Method):
    def __init__(self, name, config):
        self.name = name
        self.node_features = int(config['node_features'])
        self.edge_features = int(config['edge_features'])
        # self.hidden_dims = int(config['hidden_dims'])
        self.output_dims = int(config['output_dims'])
        self.batch_size = int(config['batch_size'])
        self.device = config['device']
        self.epochs = int(config['epochs'])
        self.save_dir = config['save_dir']
        self.model_name = config['model_name']
        self.classification_threshold = float(config.get('classification_threshold', 0.5))
        
        # Early stopping parameters
        self.patience = int(config.get('patience', 10))  # Default 10 epochs
        self.min_delta = float(config.get('min_delta', 1e-4))  # Minimum change to qualify as an improvement
        
        # Initialize model
        self.model = GNNClassifierModel(self.node_features, self.edge_features, config)
        self.model.to(self.device)

        # Initialize optimizer
        self.optimizer = torch.optim.Adam(self.model.parameters(), lr=float(config.get('learning_rate', 0.001)))
        
        # Initialize scheduler
        self.scheduler = StepLR(
            self.optimizer, 
            step_size=int(config.get('scheduler_step_size', 10)),  # Decay LR every 10 epochs by default
            gamma=float(config.get('scheduler_gamma', 0.5))  # Multiply LR by 0.5 at each step
        )
        
    def _get_criterion(self, balance_ratio):
        """
        Balance the loss function by giving more weight to the negative class.
        if balance_ratio is 1, then the loss function is balanced.
        if balance_ratio is 2, then the loss function is 2x more weight on the negative class.
        """
        neg_weight = torch.tensor([balance_ratio]).to(self.device)
        return nn.BCEWithLogitsLoss(pos_weight=1/neg_weight)

    def train(self, training_data, evaluation_data, balance_ratio, verbose=False):
        # Setup dataset and dataloader
        self.criterion = self._get_criterion(balance_ratio)
        print(f"Balance ratio: {balance_ratio}")
        train_dataset = GNNClassifierDataset(training_data)
        train_loader = DataLoader(train_dataset, batch_size=self.batch_size, shuffle=True, num_workers=8)
        eval_dataset = GNNClassifierDataset(evaluation_data)
        eval_loader = DataLoader(eval_dataset, batch_size=self.batch_size, shuffle=False, num_workers=8)

        # Early stopping variables
        best_eval_loss = float('inf')
        patience_counter = 0
        best_model_state = None
        
        for epoch in range(self.epochs):
            # Training phase
            train_loss = self._train_epoch(train_loader, verbose)
            eval_loss, accuracy = self._eval_epoch(eval_loader, verbose)
            self.scheduler.step()
            current_lr = self.scheduler.get_last_lr()[0]

            if verbose:
                print(f"Epoch {epoch+1}/{self.epochs}")
                print(f"Train loss: {train_loss:.4f}, Eval loss: {eval_loss:.4f}, Accuracy: {accuracy:.4f}")
                print(f"Learning rate: {current_lr:.6f}")

            if eval_loss < best_eval_loss - self.min_delta:
                best_eval_loss = eval_loss
                patience_counter = 0
                best_model_state = self.model.state_dict().copy()
                self._save_model(epoch, eval_loss, accuracy)
            else:
                patience_counter += 1
                if patience_counter >= self.patience:
                    if verbose:
                        print(f"Early stopping triggered after {epoch + 1} epochs")
                    self.model.load_state_dict(best_model_state)
                    break

    def _train_epoch(self, train_loader, verbose=False):
        train_loss = 0
        self.model.train()
        if verbose:
            pbar = tqdm(train_loader, total=len(train_loader))
        else:
            pbar = train_loader
        for data in pbar:
            self.optimizer.zero_grad()
            data = data.to(self.device)
            output = self.model(data)
            target = data.y.float().view(-1, 1)
            loss = self.criterion(output, target)
            loss.backward()
            self.optimizer.step()
            train_loss += loss.item()
            # pbar.set_description(f"Train loss: {train_loss:.4f}")
            if verbose:
                pbar.update(1)

        return train_loss / len(train_loader)
    
    def _eval_epoch(self, eval_loader, verbose=False):
        self.model.eval()
        eval_loss = 0
        correct_preds = 0
        total_preds = 0
        if verbose:
            pbar = tqdm(eval_loader, total=len(eval_loader))
        else:
            pbar = eval_loader

        with torch.no_grad():
            for data in pbar:
                data = data.to(self.device)
                output = self.model(data)
                target = data.y.float().view(-1, 1)
                loss = self.criterion(output, target)
                eval_loss += loss.item()

                output = torch.sigmoid(output)
                pred = (output > self.classification_threshold).float() 
                correct_preds += (pred == target).sum().item()
                total_preds += target.size(0)
                # pbar.set_description(f"Eval loss: {eval_loss:.4f}")
                if verbose:
                    pbar.update(1)
        eval_loss /= len(eval_loader)
        accuracy = correct_preds / total_preds

        return eval_loss, accuracy
    
    def predict(self, input_data):
        self.model.eval()
        dataset = GNNClassifierDataset([input_data])
        loader = DataLoader(dataset, batch_size=1, shuffle=False)
        
        with torch.no_grad():
            for data in loader:
                data = data.to(self.device)
                output = self.model(data)
                output = torch.sigmoid(output)
                pred = (output > self.classification_threshold).float()
                return output.item(), pred.item(), data.y.item()

    def predict_list(self, data_list):
        predictions = []
        for data in data_list:
            pred = self.predict(data)
            predictions.append((data, pred))
        return predictions
        
    def _save_model(self, epoch, loss, accuracy):
        """Save model checkpoint with metadata"""
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        checkpoint = {
            'epoch': epoch,
            'model_state_dict': self.model.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'scheduler_state_dict': self.scheduler.state_dict(),
            'loss': loss,
            'accuracy': accuracy
        }
        # path = os.path.join(self.save_dir, f"{self.model_name}_epoch_{epoch}.pt")
        # torch.save(checkpoint, path)

        
        # Also save as best model if it's the best so far
        best_model_path = os.path.join(self.save_dir, f"{self.model_name}_best.pt")
        torch.save(checkpoint, best_model_path)
        print(f"Saved best model to {best_model_path}")

    def load_model(self):
        model_path = os.path.join(self.save_dir, f"{self.model_name}_best.pt")
        checkpoint = torch.load(model_path)
        self.model.load_state_dict(checkpoint['model_state_dict'])
        self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
        self.scheduler.load_state_dict(checkpoint['scheduler_state_dict'])
        print(f"Loaded model from {model_path}")
    
    def get_name(self):
        return self.name

    def get_classification_threshold(self):
        return self.classification_threshold

class MLPClassifier(Method):
    def __init__(self, name, config):
        self.name = name
        self.input_dims = int(config['input_dims'])
        self.output_dims = int(config['output_dims'])
        self.hidden_dims = config['hidden_dims']
        self.batch_size = int(config['batch_size'])
        self.device = config['device']
        self.epochs = int(config['epochs'])
        self.save_dir = config['save_dir']
        self.model_name = config['model_name']
        self.learning_rate = float(config['learning_rate'])
        self.classification_threshold = float(config.get('classification_threshold', 0.5))
        self.patience = int(config.get('patience', 10))  # Default 10 epochs
        self.min_delta = float(config.get('min_delta', 1e-4))  # Minimum change to qualify as an improvement
        self.model = MLPClassifierModel(self.input_dims, self.hidden_dims, self.output_dims)
        self.model.to(self.device)

        self.optimizer = torch.optim.Adam(self.model.parameters(), lr=self.learning_rate)
        self.scheduler = StepLR(self.optimizer, step_size=10, gamma=0.5)

        self.criterion = nn.BCELoss()

    def _get_criterion(self, balance_ratio):
        """
        Balance the loss function by giving more weight to the negative class.
        if balance_ratio is 1, then the loss function is balanced.
        if balance_ratio is 2, then the loss function is 2x more weight on the negative class.
        """
        neg_weight = torch.tensor([balance_ratio]).to(self.device)
        return nn.BCEWithLogitsLoss(pos_weight=1/neg_weight)

    def train(self, training_data, evaluation_data, balance_ratio, verbose=False):
        print(f"Balance ratio: {balance_ratio}")
        self.criterion = self._get_criterion(balance_ratio)
        train_dataset = MLPClassifierDataset(training_data)
        train_loader = DataLoader(train_dataset, batch_size=self.batch_size, shuffle=True, num_workers=8)
        eval_dataset = MLPClassifierDataset(evaluation_data)
        eval_loader = DataLoader(eval_dataset, batch_size=self.batch_size, shuffle=False, num_workers=8)

        best_eval_loss = float('inf')
        patience_counter = 0
        best_model_state = None
        

        for epoch in range(self.epochs):
            train_loss = self._train_epoch(train_loader)
            eval_loss, accuracy = self._eval_epoch(eval_loader)
            self.scheduler.step()
            current_lr = self.scheduler.get_last_lr()[0]

            if verbose:
                print(f"Epoch {epoch+1}/{self.epochs}")
                print(f"Train loss: {train_loss:.4f}, Eval loss: {eval_loss:.4f}, Accuracy: {accuracy:.4f}")
                print(f"Learning rate: {current_lr:.6f}")

            if eval_loss < best_eval_loss - self.min_delta:
                best_eval_loss = eval_loss
                patience_counter = 0
                best_model_state = self.model.state_dict().copy()
                self._save_model(epoch, eval_loss, accuracy)
            else:
                patience_counter += 1
                if patience_counter >= self.patience:
                    if verbose:
                        print(f"Early stopping triggered after {epoch + 1} epochs")
                    self.model.load_state_dict(best_model_state)
                    break

    def _train_epoch(self, train_loader):
        train_loss = 0
        self.model.train()
        pbar = tqdm(train_loader, total=len(train_loader))
        for data in pbar:
            self.optimizer.zero_grad()
            features, target = data
            features = features.to(self.device)
            target = target.to(self.device)
            output = self.model(features)
            loss = self.criterion(output, target)
            loss.backward()
            self.optimizer.step()
            train_loss += loss.item()
            # pbar.set_description(f"Train loss: {train_loss:.4f}")
            pbar.update(1)
        
        return train_loss / len(train_loader)

    def _eval_epoch(self, eval_loader):
        self.model.eval()
        eval_loss = 0
        correct_preds = 0
        total_preds = 0
        pbar = tqdm(eval_loader, total=len(eval_loader))
        with torch.no_grad():
            for features, target in pbar:
                features = features.to(self.device)
                target = target.to(self.device)
                output = self.model(features)
                loss = self.criterion(output, target)
                eval_loss += loss.item()
                # pbar.set_description(f"Eval loss: {eval_loss:.4f}")
                pbar.update(1)

                output = torch.sigmoid(output)
                pred = (output > self.classification_threshold).float()
                correct_preds += (pred == target).sum().item()
                total_preds += target.size(0)

        eval_loss /= len(eval_loader)
        accuracy = correct_preds / total_preds
        return eval_loss, accuracy

    def predict(self, input_data):
        self.model.eval()
        dataset = MLPClassifierDataset([input_data])
        loader = DataLoader(dataset, batch_size=1, shuffle=False)

        with torch.no_grad():
            for data in loader:
                features, target = data
                features = features.to(self.device)
                target = target.to(self.device)
                output = self.model(features)
                output = torch.sigmoid(output)
                pred = (output > self.classification_threshold).float()
                return output.item(), pred.item(), target.item()

    def predict_list(self, data_list):
        predictions = []
        for data in data_list:
            pred = self.predict(data)
            predictions.append((data, pred))
        return predictions
    
    def _save_model(self, epoch, loss, accuracy):
        """Save model checkpoint with metadata"""
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        checkpoint = {
            'epoch': epoch,
            'model_state_dict': self.model.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'scheduler_state_dict': self.scheduler.state_dict(),
            'loss': loss,
            'accuracy': accuracy
        }
        # path = os.path.join(self.save_dir, f"{self.model_name}_epoch_{epoch}.pt")
        # torch.save(checkpoint, path)

        # Also save as best model if it's the best so far
        best_model_path = os.path.join(self.save_dir, f"{self.model_name}_best.pt")
        torch.save(checkpoint, best_model_path)
        print(f"Saved best model to {best_model_path}")

    def load_model(self):
        print(f"Loading model from {self.save_dir}")
        model_path = os.path.join(self.save_dir, f"{self.model_name}_best.pt")
        checkpoint = torch.load(model_path)
        self.model.load_state_dict(checkpoint['model_state_dict'])
        self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
        self.scheduler.load_state_dict(checkpoint['scheduler_state_dict'])
        print(f"Loaded model from {model_path}")

    def get_name(self):
        return self.name

    def get_classification_threshold(self):
        return self.classification_threshold