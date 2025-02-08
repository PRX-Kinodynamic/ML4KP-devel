from torch_geometric.data import Data, Dataset
from pathlib import Path
import os
import torch
import json
import numpy as np
from utils import get_rectangle_corners
from scipy.spatial.transform import Rotation as R

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

        goal_threshold = 0.1

        folder = self.data[idx]
        metadata = {}
        with open(os.path.join(folder, "metadata.json"), 'r') as f:
            metadata = json.load(f)

        env_props = metadata["environment"]
        xml_path = Path(metadata["xml_path"])
        goal = metadata["goal_state"][:2]



        # goal = [float(x) for x in metadata["goal_state"].strip().split()[:2]]
        # trajectory_path = os.path.join(folder, "trajectory.txt")
        # with open(trajectory_path, "r") as f:
        #     _ = f.readline()
        #     robot_pos = [float(x) for x in f.readline().split(' ')[:2]]
        # # print(robot_pos)
        # label = int(metadata['success'])

        label = float(metadata['success'])

        target_object = 0
        for model_props in env_props:
            name = model_props["name"]
            pos = model_props["pos"][:2]
            rot = R.from_quat(model_props["quat"], scalar_first=True).as_euler('xyz')[2]
            size = model_props["size"][:2]
            features = []
            if "obstacle" in name or "robot" in name:
                new_size = (np.array(size[:2]) * 2).tolist()
                # get four corners of the rectangle given the center, size, and rotation
                corners = get_rectangle_corners(pos, new_size, rot)
                features.extend([*pos, *corners.flatten()])
                if "robot" in name:
                    target_object = len(node_features) - 1
                    goal_size = (size[0] * 2) + goal_threshold
                    features.extend([1.0, 0.0, 0.0])
                else:
                    features.extend([0.0, 1.0, 0.0])
                
                node_features.append(torch.tensor(features))

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
        goal_corners = get_rectangle_corners(goal, [goal_size]*2, 0)
        goal_node = torch.tensor([*goal, *goal_corners.flatten(), 0.0, 0.0, 1.0])
        node_features.append(goal_node)
        edge_index.append([target_object, len(node_features) - 1])
        edge_attr.append(goal_node[:2] - node_features[target_object][:2])
       
        node_features = torch.stack(node_features).float()
        # node_features = node_features
        edge_index = torch.tensor(edge_index).long().transpose(0, 1) # transpose to make it COO format.
        edge_attr = torch.stack(edge_attr).float()
        label = torch.tensor([label]).float()

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
        goal_threshold = 0.1
        goal_size = None

        with open(os.path.join(folder, "metadata.json"), 'r') as f:
            metadata = json.load(f)

        env_props = metadata["environment"]
        xml_path = Path(metadata["xml_path"])
        goal = metadata["goal_state"][:2]

        label = float(metadata['success'])

        # trajectory_path = os.path.join(folder, "trajectory.txt")
        # with open(trajectory_path, "r") as f:
        #     trajectory = f.readlines()
        # robot_pos = [float(x) for x in trajectory[1].split(' ')[:2]]
        
        # with open(os.path.join(MODEL_PROPS, f"{xml_path.stem}.json"), 'r') as f:
        #     model_props = json.load(f)
        

        target_object = 0
        for model_props in env_props:
            name = model_props["name"]
            pos = model_props["pos"][:2]
            rot = R.from_quat(model_props["quat"], scalar_first=True).as_euler('xyz')[2]
            size = model_props["size"][:2]
            if "obstacle" in name or "robot" in name:
                new_size = (np.array(size[:2]) * 2).tolist()
                # get four corners of the rectangle given the center, size, and rotation
                corners = get_rectangle_corners(pos, new_size, rot)
                features.append(torch.tensor([*pos, *corners.flatten()]))
                if "robot" in name:
                    goal_size = (size[0] * 2) + goal_threshold
            # if "robot" in key:
            #     size = value['size'][:2]
            #     robot_features = torch.tensor([robot_pos[0], robot_pos[1], 0, size[0], size[1]])
            #     features.append(robot_features)
                
        features = torch.stack(features)
        goal_corners = get_rectangle_corners(goal, [goal_size]*2, 0)
        goal_node = torch.tensor([*goal, *goal_corners.flatten()])
        features = torch.cat([features, goal_node.unsqueeze(0)], dim=0).float().flatten()
        label = torch.tensor([label]).float()

        return features, label

# if __name__ == "__main__":
#     list_data = os.listdir(DATA_FOLDER)
#     data = [os.path.join(DATA_FOLDER, x) for x in list_data]
#     dataset = MLPClassifierDataset(data)
#     print(dataset[1])