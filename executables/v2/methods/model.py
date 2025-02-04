import torch
import torch.nn as nn
import torch.nn.functional as F
from torch_geometric.nn import GATv2Conv, global_mean_pool

class GNNClassifierModel(nn.Module):
    def __init__(self, node_features, edge_features, config):
        super().__init__()
        
        node_hidden_dims = config["node_hidden_dims"]
        edge_hidden_dims = config["edge_hidden_dims"]
        gcn_hidden_dims = config["gcn_hidden_dims"]
        graph_predictor_hidden_dims = config["graph_predictor_hidden_dims"]
        output_dims = config["output_dims"]
        
        # Node feature processing
        layers = []
        for i in range(len(node_hidden_dims)):
            layers.append(nn.Linear(node_features if i == 0 else node_hidden_dims[i-1], node_hidden_dims[i]))
            layers.append(nn.ReLU())
            layers.append(nn.LayerNorm(node_hidden_dims[i]))
        self.node_mlp = nn.Sequential(*layers)
        
        # Edge feature processing
        layers = []
        for i in range(len(edge_hidden_dims)):
            layers.append(nn.Linear(edge_features if i == 0 else edge_hidden_dims[i-1], edge_hidden_dims[i]))
            layers.append(nn.ReLU())
            layers.append(nn.LayerNorm(edge_hidden_dims[i]))
        self.edge_mlp = nn.Sequential(*layers)
        
        # Graph attention layers with processed edge features
        layers = []
        for i in range(len(gcn_hidden_dims)):
            input_dim = node_hidden_dims[-1] if i == 0 else gcn_hidden_dims[i-1]
            layers.append(GATv2Conv(input_dim, gcn_hidden_dims[i], edge_dim=edge_hidden_dims[-1]))
            layers.append(nn.ReLU())
        layers.append(GATv2Conv(gcn_hidden_dims[-1], gcn_hidden_dims[-1], edge_dim=edge_hidden_dims[-1]))
        self.gat = nn.ModuleList(layers)
        
        # Graph-level prediction layers
        layers = []
        input_dim = gcn_hidden_dims[-1]  # Input dim is the output of final GAT layer
        for i in range(len(graph_predictor_hidden_dims)):
            layers.append(nn.Linear(input_dim if i == 0 else graph_predictor_hidden_dims[i-1], 
                                  graph_predictor_hidden_dims[i]))
            layers.append(nn.ReLU())
        layers.append(nn.Linear(graph_predictor_hidden_dims[-1], output_dims))
        self.graph_predictor = nn.Sequential(*layers)

    def forward(self, data):
        x, edge_index, edge_attr = data.x, data.edge_index, data.edge_attr
        batch = data.batch
        
        # Process initial node features
        x = self.node_mlp(x)
        
        # Process edge features
        edge_attr = self.edge_mlp(edge_attr)
        
        # Message passing with processed edge features
        for i, layer in enumerate(self.gat):
            if isinstance(layer, GATv2Conv):
                x = layer(x, edge_index, edge_attr=edge_attr)
            else:  # ReLU
                x = layer(x)
        
        # Global pooling
        graph_features = global_mean_pool(x, batch)
        
        # Predict for the entire graph
        predictions = self.graph_predictor(graph_features)
        
        return predictions

class MLPClassifierModel(nn.Module):
    def __init__(self, input_dims: int, hidden_dims: list[int], output_dims: int):
        super().__init__()
        
        layers = []
        for i in range(len(hidden_dims)):
            layers.append(nn.Linear(input_dims if i == 0 else hidden_dims[i-1], hidden_dims[i]))
            layers.append(nn.ReLU())
        
        layers.append(nn.Linear(hidden_dims[-1], output_dims))
        
        self.layers = nn.Sequential(*layers)

    def forward(self, x):
        return self.layers(x)
    
    
