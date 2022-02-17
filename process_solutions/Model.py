import torch.nn as nn
import torch.nn.functional as F
import torch

class MLP(nn.Module):
    def __init__(self, feature_dim=8, last_dim = 3, hidden_dim = 512, dropout = 0.5):
        super(MLP, self).__init__()
        self.Dropout = nn.Dropout(dropout)

        self.critic = nn.Sequential(
                               nn.Linear(feature_dim, hidden_dim),
                               nn.Dropout(dropout),
                               nn.ReLU(),
                               nn.Linear(hidden_dim, hidden_dim),
                               nn.Dropout(dropout),
                               nn.ReLU(),
                               nn.Linear(hidden_dim, hidden_dim),
                               nn.Dropout(dropout),
                               nn.ReLU(),
                               nn.Linear(hidden_dim, last_dim)
                               )

    def forward(self, x):
        return self.critic(x)