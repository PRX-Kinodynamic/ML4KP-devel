import torch
import torch.nn as nn
import torch.nn.functional as F
from torch_geometric.nn import GCNConv, global_mean_pool

class GNNClassifierModel(nn.Module):
    def __init__(self, node_features, edge_features, hidden_dim=64):
        super().__init__()
        
        # Node feature processing
        self.node_mlp = nn.Sequential(
            nn.Linear(node_features, hidden_dim),
            nn.ReLU(),
            nn.LayerNorm(hidden_dim),
            nn.Linear(hidden_dim, hidden_dim)
        )
        
        # Edge feature processing
        self.edge_mlp = nn.Sequential(
            nn.Linear(edge_features, 32),  # Reduce edge features to a single weight
            nn.ReLU(),
            nn.Linear(32, 1),
            nn.Sigmoid()
        )
        
        # Graph convolution layers
        self.conv1 = GCNConv(hidden_dim, hidden_dim)
        self.conv2 = GCNConv(hidden_dim, hidden_dim)
        
        # Graph-level prediction layers
        self.graph_predictor = nn.Sequential(
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            # nn.Dropout(0.2),
            nn.Linear(hidden_dim, 1),
        )

    def forward(self, data):
        x, edge_index, edge_attr = data.x, data.edge_index, data.edge_attr
        batch = data.batch  # Batch vector to identify which nodes belong to which graph
        
        # Process initial node features
        x = self.node_mlp(x)
        
        # Process edge features to scalar weights
        edge_weights = self.edge_mlp(edge_attr).squeeze(-1)
        
        # Message passing with edge weights
        x = F.relu(self.conv1(x, edge_index, edge_weight=edge_weights))
        x = F.relu(self.conv2(x, edge_index, edge_weight=edge_weights))
        
        # Global pooling
        graph_features = global_mean_pool(x, batch)
        
        # Predict feasibility for the entire graph
        predictions = self.graph_predictor(graph_features)
        
        return predictions
    
class MLPClassifierModel(nn.Module):
    def __init__(self, input_dims: int, hidden_dims: list[int], output_dims: int):
        super().__init__()
        
        # hidden_dims is a where each element is the number of neurons in each layer
        # input_dims is the number of features in the input
        # output_dims is the number of classes in the output
        
        # Create a list of layers
        layers = []
        for i in range(len(hidden_dims)):
            layers.append(nn.Linear(input_dims if i == 0 else hidden_dims[i-1], hidden_dims[i]))
            layers.append(nn.ReLU())
        
        layers.append(nn.Linear(hidden_dims[-1], output_dims))
        
        self.layers = nn.Sequential(*layers)

    def forward(self, x):
        return self.layers(x)
    
    
