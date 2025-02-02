from torch_geometric.data import Data, Dataset
from pathlib import Path
import os
import torch
import json
import numpy as np

DATA_FOLDER = "/media/dhruv/a7519aee-b272-44ae-a117-1f1ea1796db6/2024/NAMO/cylinder_data2"
MODEL_PROPS = "resources/temp"

class GNNClassifierDataset(Dataset):
    def __init__(self, data):
        self.data = data

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        node_features = []
        edge_index = []
        edge_attr = []

        folder = self.data[idx]
        metadata = {}
        with open(os.path.join(folder, "metadata.txt"), 'r') as f:
            for line in f.readlines():
                key, value = line.strip().split(":")
                metadata[key.strip()] = value.strip()
        xml_path = Path(metadata["xml_path"])
        goal = [float(x) for x in metadata["goal_state"].strip().split()[:2]]

        trajectory_path = os.path.join(folder, "trajectory.txt")
        with open(trajectory_path, "r") as f:
            _ = f.readline()
            robot_pos = [float(x) for x in f.readline().split(' ')[:2]]
        # print(robot_pos)

        label = int(metadata['success'])
        
        with open(os.path.join(MODEL_PROPS, f"{xml_path.stem}.json"), 'r') as f:
            model_props = json.load(f)

        target_object = 0
        for key, value in model_props.items():
            if "obstacle" in key:
                pos = value['pos'][:2]
                rot = [value['rot']]
                size = value['size'][:2]
                node_features.append(torch.tensor(pos + rot + size))
                if "movable" in key:
                    target_object = len(node_features) - 1
            # adding robot node to the node features
            # if "robot" in key:
            #     size = value['size'][:2]
            #     robot_node = torch.tensor([robot_pos[0], robot_pos[1], 0, size[0], size[1]])
            #     node_features.append(robot_node)
        
        # including goal node in node features and then building a fully connected graph
        # goal_node = torch.tensor([goal[0], goal[1], 0, 0, 0])
        # node_features.append(goal_node)

        for i in range(len(node_features)):
            for j in range(i + 1, len(node_features)):
                if i != j:
                    edge_index.append([i, j])
                    rel_pos = node_features[j][:2] - node_features[i][:2]
                    edge_attr.append(rel_pos)

        # special edge from target object to goal
        goal_node = torch.tensor([goal[0], goal[1], 0, 0, 0])
        node_features.append(goal_node)
        edge_index.append([target_object, len(node_features) - 1])
        edge_attr.append(goal_node[:2] - node_features[target_object][:2])
       
        node_features = torch.stack(node_features)
        # node_features = node_features
        edge_index = torch.tensor(edge_index).long().transpose(0, 1) # transpose to make it COO format.
        edge_attr = torch.stack(edge_attr)
        label = torch.tensor([label])

        return Data(x=node_features, edge_index=edge_index, edge_attr=edge_attr, y=label)


class MLPClassifierDataset(Dataset):
    def __init__(self, data):
        self.data = data

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        folder = self.data[idx]
        features = []
        metadata = {}
        with open(os.path.join(folder, "metadata.txt"), 'r') as f:
            for line in f.readlines():
                key, value = line.strip().split(":")
                metadata[key.strip()] = value.strip()
        xml_path = Path(metadata["xml_path"])
        goal = [float(x) for x in metadata["goal_state"].strip().split()[:2]]
        label = int(metadata['success'])

        trajectory_path = os.path.join(folder, "trajectory.txt")
        with open(trajectory_path, "r") as f:
            trajectory = f.readlines()
        robot_pos = [float(x) for x in trajectory[1].split(' ')[:2]]
        
        with open(os.path.join(MODEL_PROPS, f"{xml_path.stem}.json"), 'r') as f:
            model_props = json.load(f)


        target_object = 0
        for key, value in model_props.items():
            if "obstacle" in key:
                pos = value['pos'][:2]
                rot = [value['rot']]
                size = value['size'][:2]
                features.append(torch.tensor(pos + rot + size))
            # if "robot" in key:
            #     size = value['size'][:2]
            #     robot_features = torch.tensor([robot_pos[0], robot_pos[1], 0, size[0], size[1]])
            #     features.append(robot_features)
                
        features = torch.stack(features)
        goal_node = torch.tensor([goal[0], goal[1], 0, 0, 0])
        features = torch.cat([features, goal_node.unsqueeze(0)], dim=0).float().flatten()
        label = torch.tensor([label]).float()

        return features, label

# if __name__ == "__main__":
#     list_data = os.listdir(DATA_FOLDER)
#     data = [os.path.join(DATA_FOLDER, x) for x in list_data]
#     dataset = MLPClassifierDataset(data)
#     print(dataset[1])