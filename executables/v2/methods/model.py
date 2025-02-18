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
        
        # Node feature processing``
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
        # edge_attr = self.edge_mlp(edge_attr)
        
        # Message passing with processed edge features
        for i, layer in enumerate(self.gat):
            if isinstance(layer, GATv2Conv):
                x = layer(x, edge_index)
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

class GNNSpatialModel(nn.Module):
    def __init__(self, node_features, edge_features, config):
        super().__init__()

        node_hidden_dims = config["node_hidden_dims"]
        edge_hidden_dims = config["edge_hidden_dims"]
        gcn_hidden_dims = config["gcn_hidden_dims"]
        decoder_dims = config["decoder_dims"]  # List of [channels, height, width] for each deconv layer
        output_size = config["output_size"]  # Final image size (H=W)
        
        # Node feature processing
        layers = []
        for i in range(len(node_hidden_dims)):
            layers.append(nn.Linear(node_features if i == 0 else node_hidden_dims[i-1], 
                                  node_hidden_dims[i]))
            layers.append(nn.ReLU())
            layers.append(nn.LayerNorm(node_hidden_dims[i]))
        self.node_mlp = nn.Sequential(*layers)
        
        # Edge feature processing
        layers = []
        for i in range(len(edge_hidden_dims)):
            layers.append(nn.Linear(edge_features if i == 0 else edge_hidden_dims[i-1], 
                                  edge_hidden_dims[i]))
            layers.append(nn.ReLU())
            layers.append(nn.LayerNorm(edge_hidden_dims[i]))
        self.edge_mlp = nn.Sequential(*layers)
        
        # Graph attention layers
        layers = []
        for i in range(len(gcn_hidden_dims)):
            input_dim = node_hidden_dims[-1] if i == 0 else gcn_hidden_dims[i-1]
            layers.append(GATv2Conv(input_dim, gcn_hidden_dims[i], edge_dim=edge_hidden_dims[-1]))
            layers.append(nn.ReLU())
        layers.append(GATv2Conv(gcn_hidden_dims[-1], gcn_hidden_dims[-1], edge_dim=edge_hidden_dims[-1]))
        self.gat = nn.ModuleList(layers)
        
        # Initial projection to prepare for deconvolution
        self.initial_height = decoder_dims[0][1]
        self.initial_width = decoder_dims[0][2]
        self.project = nn.Sequential(
            nn.Linear(gcn_hidden_dims[-1], decoder_dims[0][0] * self.initial_height * self.initial_width),
            nn.ReLU()
        )
        
        # Decoder: Deconvolutional layers to reach 17x17
        deconv_layers = []
        for i in range(len(decoder_dims)-1):
            deconv_layers.extend([
                nn.ConvTranspose2d(
                    decoder_dims[i][0], decoder_dims[i+1][0],
                    kernel_size=4, stride=2, padding=1
                ),
                nn.ReLU(),
                nn.BatchNorm2d(decoder_dims[i+1][0])
            ])
        
        # Final layer to get to single channel
        self.deconv = nn.Sequential(*deconv_layers)
        self.final_conv = nn.Conv2d(decoder_dims[-1][0], 1, kernel_size=1)
        
        # Add a final size check
        self.output_size = config['output_size']
        self.resize = nn.AdaptiveAvgPool2d((self.output_size, self.output_size))

    def forward(self, data):
        x, edge_index, edge_attr = data.x, data.edge_index, data.edge_attr
        batch = data.batch if hasattr(data, 'batch') else None
        batch_size = 1 if batch is None else batch[-1].item() + 1
        
        # Process initial node and edge features
        x = self.node_mlp(x)
        edge_attr = self.edge_mlp(edge_attr)
        
        # Message passing
        for i, layer in enumerate(self.gat):
            if isinstance(layer, GATv2Conv):
                x = layer(x, edge_index, edge_attr)
            else:  # ReLU
                x = layer(x)
        
        # Global pooling
        x = global_mean_pool(x, batch) if batch is not None else torch.mean(x, dim=0, keepdim=True)
        
        # Project and reshape for deconvolution
        x = self.project(x)
        x = x.view(batch_size, -1, self.initial_height, self.initial_width)
        
        # Deconvolution
        x = self.deconv(x)
        x = self.final_conv(x)
        
        # Ensure output size is correct
        x = self.resize(x)
        
        return x.squeeze(1)  # Remove channel dimension

class GNNRadiusModel(nn.Module):
    def __init__(self, node_features, edge_features, config):
        super().__init__()
        
        node_hidden_dims = config["node_hidden_dims"]
        edge_hidden_dims = config["edge_hidden_dims"]
        gcn_hidden_dims = config["gcn_hidden_dims"]
        graph_predictor_hidden_dims = config["graph_predictor_hidden_dims"]
        
        # Node feature processing
        layers = []
        for i in range(len(node_hidden_dims)):
            layers.append(nn.Linear(node_features if i == 0 else node_hidden_dims[i-1], 
                                  node_hidden_dims[i]))
            layers.append(nn.ReLU())
            layers.append(nn.LayerNorm(node_hidden_dims[i]))
        self.node_mlp = nn.Sequential(*layers)
        
        # Edge feature processing
        layers = []
        for i in range(len(edge_hidden_dims)):
            layers.append(nn.Linear(edge_features if i == 0 else edge_hidden_dims[i-1], 
                                  edge_hidden_dims[i]))
            layers.append(nn.ReLU())
            layers.append(nn.LayerNorm(edge_hidden_dims[i]))
        self.edge_mlp = nn.Sequential(*layers)
        
        # Graph attention layers
        layers = []
        for i in range(len(gcn_hidden_dims)):
            input_dim = node_hidden_dims[-1] if i == 0 else gcn_hidden_dims[i-1]
            layers.append(GATv2Conv(input_dim, gcn_hidden_dims[i], edge_dim=edge_hidden_dims[-1]))
            layers.append(nn.ReLU())
        layers.append(GATv2Conv(gcn_hidden_dims[-1], gcn_hidden_dims[-1], edge_dim=edge_hidden_dims[-1]))
        self.gat = nn.ModuleList(layers)
        
        # Graph-level prediction layers
        layers = []
        input_dim = gcn_hidden_dims[-1]
        for i in range(len(graph_predictor_hidden_dims)):
            layers.append(nn.Linear(input_dim if i == 0 else graph_predictor_hidden_dims[i-1], 
                                  graph_predictor_hidden_dims[i]))
            layers.append(nn.ReLU())
        layers.append(nn.Linear(graph_predictor_hidden_dims[-1], 1))
        self.graph_predictor = nn.Sequential(*layers)

    def forward(self, data):
        x, edge_index, edge_attr = data.x, data.edge_index, data.edge_attr
        batch = data.batch if hasattr(data, 'batch') else None
        
        # Process initial node and edge features
        x = self.node_mlp(x)
        edge_attr = self.edge_mlp(edge_attr)
        
        # Message passing
        for i, layer in enumerate(self.gat):
            if isinstance(layer, GATv2Conv):
                x = layer(x, edge_index, edge_attr)
            else:  # ReLU
                x = layer(x)
        
        # Global pooling
        x = global_mean_pool(x, batch) if batch is not None else torch.mean(x, dim=0, keepdim=True)
        
        # Predict radius
        return self.graph_predictor(x)

class MLPRadiusModel(nn.Module):
    def __init__(self, input_dims: int, hidden_dims: list[int]):
        super().__init__()
        
        layers = []
        for i in range(len(hidden_dims)):
            layers.append(nn.Linear(input_dims if i == 0 else hidden_dims[i-1], hidden_dims[i]))
            layers.append(nn.ReLU())
        
        layers.append(nn.Linear(hidden_dims[-1], 1))
        
        self.layers = nn.Sequential(*layers)

    def forward(self, x):
        return self.layers(x)

class MLPMaskModel(nn.Module):
    def __init__(self, config):
        super().__init__()
        
        # Config params
        self.mask_size = config['mask_size']  # 40
        self.latent_dim = config['latent_dim']
        
        # Mask embedding networks - each takes 2 channels (full_scene + specific mask)
        self.movable_encoder = self._build_mask_encoder()
        self.static_encoder = self._build_mask_encoder() 
        self.robot_encoder = self._build_mask_encoder()
        
        # Decoder dimensions for 17x17 output
        self.decoder_dims = [
            [512, 3, 3],     # Start small
            [256, 5, 5],     # After first deconv
            [128, 9, 9],     # After second deconv
            [64, 17, 17]     # Final size 17x17
        ]
        
        # Initial projection dimensions
        self.initial_height = self.decoder_dims[0][1]  # 3
        self.initial_width = self.decoder_dims[0][2]   # 3
        
        # MLP layers to process concatenated embeddings
        self.mlp = nn.Sequential(
            nn.Linear(self.latent_dim * 3, 512),
            nn.ReLU(),
            nn.Linear(512, 512),
            nn.ReLU(),
            nn.Linear(512, self.decoder_dims[0][0] * self.initial_height * self.initial_width),
            nn.ReLU()
        )
        
        # Decoder network with precise output size control
        self.deconv1 = nn.ConvTranspose2d(
            512, 256, 
            kernel_size=3, stride=2, padding=1  # 3x3 -> 6x6
        )
        self.bn1 = nn.BatchNorm2d(256)
        
        self.deconv2 = nn.ConvTranspose2d(
            256, 128, 
            kernel_size=3, stride=2, padding=1, output_padding=0  # 6x6 -> 11x11
        )
        self.bn2 = nn.BatchNorm2d(128)
        
        self.deconv3 = nn.ConvTranspose2d(
            128, 64, 
            kernel_size=4, stride=2, padding=2, output_padding=1  # 11x11 -> 17x17
        )
        self.bn3 = nn.BatchNorm2d(64)
        
        self.final_conv = nn.Conv2d(64, 1, kernel_size=1)
        self.relu = nn.ReLU()
        
    def _build_mask_encoder(self):
        return nn.Sequential(
            # Input: [batch_size, 2, 40, 40]
            nn.Conv2d(2, 32, 3, padding=1),    # [batch_size, 32, 40, 40]
            nn.ReLU(),
            nn.MaxPool2d(2),                    # [batch_size, 32, 20, 20]
            nn.Conv2d(32, 64, 3, padding=1),   # [batch_size, 64, 20, 20]
            nn.ReLU(),
            nn.MaxPool2d(2),                    # [batch_size, 64, 10, 10]
            nn.Conv2d(64, 128, 3, padding=1),  # [batch_size, 128, 10, 10]
            nn.ReLU(),
            nn.MaxPool2d(2),                    # [batch_size, 128, 5, 5]
            nn.Flatten(),                       # [batch_size, 128 * 5 * 5]
            nn.Linear(128 * 5 * 5, self.latent_dim),
            nn.ReLU()
        )
        
    def forward(self, full_scene, movable_mask, static_mask, robot_mask):
        # Extract masks from batch and ensure correct dimensions
        # Each mask should be [batch_size, H, W]
        
        if len(full_scene.shape) == 4:  # If input is already [batch, C, H, W]
            full_scene = full_scene.squeeze(1)
            movable_mask = movable_mask.squeeze(1)
            static_mask = static_mask.squeeze(1)
            robot_mask = robot_mask.squeeze(1)
            
        # Add channel dimension [batch, 1, H, W]
        full_scene = full_scene.unsqueeze(1)
        movable_mask = movable_mask.unsqueeze(1)
        static_mask = static_mask.unsqueeze(1)
        robot_mask = robot_mask.unsqueeze(1)
        
        # Create combined inputs for each encoder [batch, 2, H, W]
        movable_input = torch.cat([full_scene, movable_mask], dim=1)
        static_input = torch.cat([full_scene, static_mask], dim=1)
        robot_input = torch.cat([full_scene, robot_mask], dim=1)

        # Get embeddings
        movable_embed = self.movable_encoder(movable_input)
        static_embed = self.static_encoder(static_input)
        robot_embed = self.robot_encoder(robot_input)
        
        # Concatenate embeddings
        x = torch.cat([movable_embed, static_embed, robot_embed], dim=1)
        
        # Process through MLP
        x = self.mlp(x)
        
        # Reshape for deconvolution
        x = x.view(-1, self.decoder_dims[0][0], self.initial_height, self.initial_width)
        
        # Controlled deconvolution to ensure 17x17 output
        x = self.relu(self.bn1(self.deconv1(x)))      # 3x3 -> 6x6
        x = self.relu(self.bn2(self.deconv2(x)))      # 6x6 -> 11x11
        x = self.relu(self.bn3(self.deconv3(x)))      # 11x11 -> 17x17
        x = self.final_conv(x)
        
        return x.squeeze(1)  # Output shape: [batch_size, 17, 17]