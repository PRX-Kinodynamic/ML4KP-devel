from torch_geometric.data import Data, Dataset as GeometricDataset
from torch.utils.data import Dataset
from pathlib import Path
import os
import torch
import json
import numpy as np
from utils import get_rectangle_corners
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm
DATA_FOLDER = "/media/dhruv/a7519aee-b272-44ae-a117-1f1ea1796db6/2024/NAMO/cylinder_data2"
MODEL_PROPS = "resources/temp"

class GNNClassifierDataset(GeometricDataset):
    def __init__(self, data):
        self.data = data
        # print("processing loader")
        # for folder in tqdm(self.data):
        #     try:
        #         with open(os.path.join(folder, "metadata.json"), 'r') as f:
        #             _ = json.load(f)
        #     except:
        #         print(f"Error processing {folder}")
        #         self.data.remove(folder)
        #         # delete the folder with all the files in it
        #         for file in os.listdir(folder):
        #             os.remove(os.path.join(folder, file))
        #         os.rmdir(folder)
        # exit()

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
                    goal_size = goal_threshold
                    features.extend([1.0, 0.0, 0.0, 0.0])
                else:
                    if "movable" in name:
                        features.extend([0.0, 1.0, 0.0, 0.0])
                    else:
                        features.extend([0.0, 0.0, 1.0, 0.0])
                
                node_features.append(torch.tensor(features))

            # adding robot node to the node features
            # if "robot" in key:
            #     size = value['size'][:2]
            #     robot_node = torch.tensor([robot_pos[0], robot_pos[1], 0, size[0], size[1]])
            #     node_features.append(robot_node)
        
        # including goal node in node features and then building a fully connected graph
        # goal_node = torch.tensor([goal[0], goal[1], 0, 0, 0])
        # node_features.append(goal_node)
        # special edge from target object to goal
        goal_corners = get_rectangle_corners(goal, [goal_size]*2, 0)
        goal_node = torch.tensor([*goal, *goal_corners.flatten(), 0.0, 0.0, 0.0, 1.0])
        node_features.append(goal_node)
        # edge_index.append([target_object, len(node_features) - 1])
        # edge_attr.append(goal_node[:2] - node_features[target_object][:2])

        for i in range(len(node_features)):
            for j in range(i + 1, len(node_features)):
                if i != j:
                    edge_index.append([i, j])
                    rel_pos = node_features[j][:2] - node_features[i][:2]
                    edge_attr.append(rel_pos)

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

        # while True:
        #     if not os.path.exists(os.path.join(folder, "metadata.json")):
        #         for file in os.listdir(folder):
        #             os.remove(os.path.join(folder, file))
        #         os.rmdir(folder)
        #         print(f"Error processing {folder}")
        #         idx += 1
        #         folder = self.data[idx]
        #     else:
        #         try:
        #             with open(os.path.join(folder, "metadata.json"), 'r') as f:
        #                 metadata = json.load(f)
        #         except:
        #             for file in os.listdir(folder):
        #                 os.remove(os.path.join(folder, file))
        #             os.rmdir(folder)
        #             print(f"Error processing {folder}")
        #             idx += 1
        #             folder = self.data[idx]
        #         break

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
        features_arr = torch.zeros(4 * 14)

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
                    features.append(torch.tensor([1.0, 0.0, 0.0, 0.0]))
                else:
                    if "movable" in name:   
                        features.append(torch.tensor([0.0, 1.0, 0.0, 0.0]))
                    else:
                        features.append(torch.tensor([0.0, 0.0, 1.0, 0.0]))
            # if "robot" in key:
            #     size = value['size'][:2]
            #     robot_features = torch.tensor([robot_pos[0], robot_pos[1], 0, size[0], size[1]])
            #     features.append(robot_features)

        features = torch.stack(features)
        features_arr[:features.shape[0]] = features
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

class GNNSpatialClassifierDataset(GeometricDataset):
    def __init__(self, data):
        self.data = data

    def __len__(self):
        return len(self.data)
    
    def preproccess_env_props(self, env_props):

        node_features = []
        edge_index = []
        edge_attr = []
        goal_threshold = 0.1

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
                    features.extend([1.0, 0.0, 0.0])
                elif "movable" in name:
                        features.extend([0.0, 1.0, 0.0])
                else:
                    features.extend([0.0, 0.0, 1.0])
                
                node_features.append(torch.tensor(features))

        for i in range(len(node_features)):
            for j in range(i + 1, len(node_features)):
                if i != j:
                    edge_index.append([i, j])
                    rel_pos = node_features[j][:2] - node_features[i][:2]
                    edge_attr.append(rel_pos)

        node_features = torch.stack(node_features).float()
        edge_index = torch.tensor(edge_index).long().transpose(0, 1) # transpose to make it COO format.
        edge_attr = torch.stack(edge_attr).float()
        
        return node_features, edge_index, edge_attr
    
    def __getitem__(self, idx):
        file = self.data[idx]

        with open(file, 'r') as f:
            data = json.load(f)

        env_props = data["environment"]
        node_features, edge_index, edge_attr = self.preproccess_env_props(env_props)
        img_size = int(np.array(data['goal_success']).shape[0] ** 0.5)

        # curate output success mask
        target = np.array(data['goal_success'])[:, 2].reshape(img_size, img_size, 1)
        target = torch.from_numpy(target).float().squeeze(-1)

        return Data(x=node_features, edge_index=edge_index, edge_attr=edge_attr), target
    
class MLPSpatialClassifier(Dataset):
    def __init__(self, data):
        self.data = data

    def __len__(self):
        return len(self.data)
    
    def preproccess_env_props(self, env_props):
        node_features = []
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
                    features.extend([1.0, 0.0, 0.0])
                elif "movable" in name:
                    features.extend([0.0, 1.0, 0.0])
                else:
                    features.extend([0.0, 0.0, 1.0])
                node_features.append(torch.tensor(features))
        node_features = torch.stack(node_features).float()

    def __getitem__(self, idx):
        file = self.data[idx]

        with open(file, 'r') as f:
            data = json.load(f)

        env_props = data["environment"]
        node_features = self.preproccess_env_props(env_props)
        img_size = int(np.array(data['goal_success']).shape[0] ** 0.5)

        # curate output success mask
        output = np.array(data['goal_success'])[:, -1].reshape(img_size, img_size, 1)
        output = torch.from_numpy(output).float()

        return node_features, output

class GNNRadiusDataset(GeometricDataset):
    def __init__(self, data):
        self.data = data
        self.min_radius = 0.05
        self.max_radius = 2.0

    def normalize_radius(self, radius):
        """Normalize radius to [-1, 1] range"""
        return (radius - self.min_radius) / (self.max_radius - self.min_radius) * 2 - 1
    
    def denormalize_radius(self, normalized_radius):
        """Convert normalized radius back to original scale"""
        return (normalized_radius + 1) / 2 * (self.max_radius - self.min_radius) + self.min_radius

    def __len__(self):
        return len(self.data)
    
    def preproccess_env_props(self, env_props):
        node_features = []
        edge_index = []
        edge_attr = []

        for model_props in env_props:
            name = model_props["name"]
            pos = model_props["pos"][:2]
            rot = R.from_quat(model_props["quat"], scalar_first=True).as_euler('xyz')[2]
            size = model_props["size"][:2]
            features = []
            if "obstacle" in name or "robot" in name:
                new_size = (np.array(size[:2]) * 2).tolist()
                corners = get_rectangle_corners(pos, new_size, rot)
                features.extend([*pos, *corners.flatten()])
                if "robot" in name:
                    features.extend([1.0, 0.0, 0.0])
                elif "movable" in name:
                    features.extend([0.0, 1.0, 0.0])
                else:
                    features.extend([0.0, 0.0, 1.0])
                
                node_features.append(torch.tensor(features))

        for i in range(len(node_features)):
            for j in range(i + 1, len(node_features)):
                if i != j:
                    edge_index.append([i, j])
                    rel_pos = node_features[j][:2] - node_features[i][:2]
                    edge_attr.append(rel_pos)

        node_features = torch.stack(node_features).float()
        edge_index = torch.tensor(edge_index).long().transpose(0, 1)
        edge_attr = torch.stack(edge_attr).float() if edge_attr else torch.zeros((0, 2)).float()
        
        return node_features, edge_index, edge_attr
    
    def __getitem__(self, idx):
        file = self.data[idx]

        with open(file, 'r') as f:
            data = json.load(f)

        env_props = data["environment"]
        node_features, edge_index, edge_attr = self.preproccess_env_props(env_props)
        data['reachability_radius'] = data['reachability_radius']
        radius = self.normalize_radius(data['reachability_radius'])  # Normalize radius
        radius = torch.tensor([radius]).float()
        
        return Data(x=node_features, edge_index=edge_index, edge_attr=edge_attr, y=radius)

class MLPRadiusDataset(Dataset):
    def __init__(self, data):
        self.data = data
        self.min_radius = 0.05
        self.max_radius = 1.6

    def normalize_radius(self, radius):
        return (radius - self.min_radius) / (self.max_radius - self.min_radius)
    
    def denormalize_radius(self, normalized_radius):
        return normalized_radius * (self.max_radius - self.min_radius) + self.min_radius

    def __len__(self):
        return len(self.data)
    
    def preproccess_env_props(self, env_props):
        features = []
        for model_props in env_props:
            name = model_props["name"]
            pos = model_props["pos"][:2]
            rot = R.from_quat(model_props["quat"], scalar_first=True).as_euler('xyz')[2]
            size = model_props["size"][:2]
            if "obstacle" in name or "robot" in name:
                new_size = (np.array(size[:2]) * 2).tolist()
                corners = get_rectangle_corners(pos, new_size, rot)
                feature = [*pos, *corners.flatten()]
                if "robot" in name:
                    feature.extend([1.0, 0.0, 0.0])
                elif "movable" in name:
                    feature.extend([0.0, 1.0, 0.0])
                else:
                    feature.extend([0.0, 0.0, 1.0])
                features.append(torch.tensor(feature))
        
        return torch.cat(features).float()

    def __getitem__(self, idx):
        file = self.data[idx]

        with open(file, 'r') as f:
            data = json.load(f)

        env_props = data["environment"]
        features = self.preproccess_env_props(env_props)
        radius = self.normalize_radius(data['reachability_radius'])  # Normalize radius
        radius = torch.tensor([radius]).float()

        return features, radius

class MLPMaskDataset(Dataset):
    def __init__(self, data):
        self.data = data
        
    def __len__(self):
        return len(self.data)
    
    def __getitem__(self, idx):
        file = self.data[idx]
        
        with open(file, 'r') as f:
            data = json.load(f)
            
        # Get masks from scene_images
        scene_images = data['scene_images']
        
        # Convert masks to tensors
        full_scene = torch.tensor(scene_images['full_scene']).float()
        static_mask = torch.tensor(scene_images['static_mask']).float()
        movable_mask = torch.tensor(scene_images['movable_mask']).float()
        robot_mask = torch.tensor(scene_images['robot_mask']).float()
        
        # Get target reachability mask
        goal_success = np.array(data['goal_success'])
        img_size = int(np.sqrt(len(goal_success)))
        target = torch.tensor(goal_success[:, 2].reshape(img_size, img_size)).float()
        
        return full_scene, static_mask, movable_mask, robot_mask, target