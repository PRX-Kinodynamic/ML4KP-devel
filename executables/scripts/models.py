import torch
import torch.nn as nn

class MLP(nn.Module):
    def __init__(self):
        super(MLP, self).__init__()

    def from_file(self, path):
        self.load_state_dict(torch.load(path, weights_only=True))
        self.eval()

    @staticmethod
    def create(input_dim, control_dim, hidden_sizes, output_dim, dropout_rate=0.05):    
        mlp = MLP()
        mlp.input_dim = input_dim
        mlp.control_dim = control_dim
        mlp.output_dim = output_dim

        # Create list to hold all layers
        layers = []
        
        # Input layer
        # activation_function = nn.Mish()
        activation_function = nn.SiLU()
        # activation_function = nn.ReLU()
        # activation_function = nn.LeakyReLU()
        layers.append(nn.Linear(input_dim + control_dim, hidden_sizes[0]))
        layers.append(activation_function)
        layers.append(nn.Dropout(dropout_rate))
        
        # Hidden layers
        for i in range(len(hidden_sizes)-1):
            layers.append(nn.Linear(hidden_sizes[i], hidden_sizes[i+1]))
            layers.append(activation_function)
            layers.append(nn.Dropout(dropout_rate))
        
        # Output layer
        layers.append(nn.Linear(hidden_sizes[-1], output_dim))
        
        # Combine all layers into a sequential model
        mlp.model = nn.Sequential(*layers)
        return mlp


    def forward(self, x):
        return self.model(x)
