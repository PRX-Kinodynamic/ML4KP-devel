import torchvision.transforms
from torchvision.transforms import Lambda
from diffusers import UNet2DModel, UNet2DConditionModel, DDPMScheduler
from transformers import AutoImageProcessor, AutoModel
from torch.utils.data import Dataset, DataLoader
from tqdm import tqdm
from torch.optim import AdamW
from accelerate import Accelerator
import torch
import os
import numpy as np
import random
from collections import Counter
import torch.nn.functional as F
from scipy.optimize import minimize
import cv2
import matplotlib.pyplot as plt
import torch.nn as nn
from torchvision.models import resnet18

config = {
    'env_config_name': 'env_config_2',
    "num_epochs": 5000,
    "save_every": 1000,
    "batch_size": 24,
    "learning_rate": 1e-4,
    "weight_decay": 0.01,
    "num_workers": 4,
    "in_channels": 5,
    "out_channels": 2,
    "image_size": 64,
    "num_diffusion_timesteps": 100,
    
    
    # "mode": "train", # train, test, zmq
    # "device_id": 4,
    # "save_dir": "executables/v3/diffusion_v2/saved_models/only_object_mask/apr30/fixed_start_fixed_goal_many_env_dinopeft_with_cond_inp_5_example",
    # "load_model_path": "executables/v3/diffusion_v2/saved_models/only_object_mask/apr30/fixed_start_fixed_goal_many_env_dinopeft_with_cond_inp_5_example/best",
    # "train_data_dir": ["/common/users/dm1487/namo_data/images/apr30/fixed_start_fixed_goal_many_env_5_example"],
    
    "cross_attention_dim": 196,
    "mode": "train",
    "device_id": 3,
    "save_dir": "/common/users/dm1487/namo/diffusion/saved_models/only_object_mask/may1/fixed_start_fixed_goal_many_env_resent_with_cond_inp_5_example",
    
    "load_model_path": "/common/users/dm1487/namo/diffusion/saved_models/only_object_mask/may1/fixed_start_fixed_goal_many_env_resent_with_cond_inp_5_example/best",
    "train_data_dir": ["/common/users/dm1487/namo_data/images/apr28/fixed_start_fixed_goal_many_env_5_example"]
}

torch.cuda.set_device(config["device_id"])

class CustomDataset(Dataset):
    def __init__(self, folder_path, transform, mode="train"):
        self.folder_path = folder_path
        self.image_paths = []
        if isinstance(folder_path, list):
            for folder in folder_path:
                self.image_paths.extend([os.path.join(folder, f) for f in os.listdir(folder)])
        else:
            self.image_paths = [os.path.join(folder_path, f) for f in os.listdir(folder_path)]
        random.shuffle(self.image_paths)
        self.image_paths = self.image_paths
        
        if mode == "zmq":
            self.image_paths = self.image_paths[:10]
            
        self.final_image_paths = []
        for idx in tqdm(range(len(self.image_paths))):
            file = self.image_paths[idx]
            npz_file = np.load(file)
            object_mask = npz_file['object_mask']
            if np.sum(object_mask) == 0:
                continue
            self.final_image_paths.append(file)
        # self.final_image_paths = self.final_image_paths
        self.transform = transform
        
        self.resnet_transforms = torchvision.transforms.Compose([
            torchvision.transforms.ToTensor(),
            torchvision.transforms.Resize(224),  # ResNet expects 224x224 images
            torchvision.transforms.Normalize(
                mean=[0.485, 0.456, 0.406],  # ImageNet mean
                std=[0.229, 0.224, 0.225]     # ImageNet std
            )
        ])

    def __len__(self):
        return len(self.final_image_paths)

    def __getitem__(self, index):
        image_path = self.final_image_paths[index]
        npz_file = np.load(image_path)
        object_mask = npz_file['object_mask']
        goal_mask = npz_file['goal_mask']
        scene = npz_file['scene']
        image = np.concatenate([scene, object_mask, goal_mask], axis=-1)
        image = self.transform(image)
        return { "diffusion_input": image, "dino_input": self.resnet_transforms(npz_file['scene'].copy()) , "scene": npz_file['scene'].copy()}
    
class Runner:
    def __init__(self, config):
        self.config = config
        self.image_size = config["image_size"]
        self.in_channels = config["in_channels"]
        self.out_channels = config["out_channels"]
        self._setup_accelerator(config)
        
    def _setup_accelerator(self, config):
        # Initialize ResNet18 and get layer3 features
        backbone = resnet18(pretrained=True)
        self.scene_encoder = nn.Sequential(
            backbone.conv1,
            backbone.bn1,
            backbone.relu,
            backbone.maxpool,
            backbone.layer1,
            backbone.layer2,
            backbone.layer3,
        )
        
        # Freeze ResNet parameters
        for param in self.scene_encoder.parameters():
            param.requires_grad = False
        self.scene_encoder.eval()
        
        # Add a projection layer to match cross-attention dimensions
        self.scene_adapter = nn.Sequential(
            nn.Conv2d(256, self.config["cross_attention_dim"], kernel_size=1),
            nn.GELU(),
            nn.Conv2d(self.config["cross_attention_dim"], self.config["cross_attention_dim"], kernel_size=1)
        )

        self.model = UNet2DConditionModel(
            sample_size=self.image_size,
            in_channels=self.in_channels,
            out_channels=self.out_channels,
            layers_per_block=2,
            block_out_channels=(320, 640, 1280, 1280),
            down_block_types=(
                "DownBlock2D",  # <- changed
                "CrossAttnDownBlock2D",
                "CrossAttnDownBlock2D",
                "CrossAttnDownBlock2D",
            ),
            up_block_types=(
                "CrossAttnUpBlock2D",
                "CrossAttnUpBlock2D",
                "CrossAttnUpBlock2D",
                "UpBlock2D",
            ),
            cross_attention_dim=self.config["cross_attention_dim"]
        )

        self.noise_scheduler = DDPMScheduler(num_train_timesteps=self.config["num_diffusion_timesteps"], beta_schedule='squaredcos_cap_v2')
        
        self.transforms = torchvision.transforms.Compose([
            torchvision.transforms.ToTensor(),
            torchvision.transforms.Resize((self.image_size, self.image_size)),
            Lambda(lambda x: x * 2 - 1)
        ])
        
        self.train_dataset = CustomDataset(folder_path=config["train_data_dir"], transform=self.transforms, mode=config["mode"])
            
        print("-"*50)
        print("Dataset length: ", len(self.train_dataset))
        print("-"*50)
            
        self.train_dataloader = DataLoader(self.train_dataset, batch_size=config["batch_size"], shuffle=True, num_workers=config["num_workers"])
        
        self.optimizer = AdamW(list(self.model.parameters()) + list(self.scene_adapter.parameters()), 
                             lr=config["learning_rate"], 
                             weight_decay=config["weight_decay"])

        self.accelerator = Accelerator()
        self.model, self.scene_encoder, self.scene_adapter, self.optimizer, self.train_dataloader = \
            self.accelerator.prepare(self.model, self.scene_encoder, self.scene_adapter, self.optimizer, self.train_dataloader)
        
        self.device = self.accelerator.device
        
        
    def train(self):
        # Training loop
        self.model.train()
        self.scene_adapter.train()
        
        num_epochs = self.config["num_epochs"]
        out_channels = self.out_channels
        
        for epoch in range(num_epochs):
            epoch_loss = 0
            pbar = tqdm(total=len(self.train_dataloader), desc=f"Epoch {epoch+1}/{num_epochs}")
            
            for step, batch in enumerate(self.train_dataloader):
                self.optimizer.zero_grad()
                
                clean_images = batch["diffusion_input"]
                dino_input = batch["dino_input"]
                
                # Use ResNet in eval mode with no_grad since it's frozen
                with torch.no_grad():
                    resnet_features = self.scene_encoder(dino_input)
                
                # Only adapt the features through our trainable adapter
                resnet_features = self.scene_adapter(resnet_features)
                
                B, C, H, W = resnet_features.shape
                resnet_features = resnet_features.view(B, C, H*W)
                
                noise = torch.randn(clean_images.shape).to(clean_images.device)
                timesteps = torch.randint(0, self.noise_scheduler.config.num_train_timesteps, (clean_images.shape[0],), device=clean_images.device).long()
                noisy_images = self.noise_scheduler.add_noise(clean_images[:, -out_channels:, :, :], noise[:, -out_channels:, :, :], timesteps)
                clean_images[:, -out_channels:, :, :] = noisy_images
                
                noise_pred = self.model(clean_images, timesteps, encoder_hidden_states=resnet_features).sample
                loss = F.mse_loss(noise_pred, noise[:, -out_channels:, :, :])
                self.accelerator.backward(loss)
                self.optimizer.step()
                epoch_loss += loss.item()
                pbar.update(1)
            pbar.close()
            
            print(f"Epoch {epoch+1}/{num_epochs} loss: {epoch_loss/len(self.train_dataloader)}")
            if (epoch+1) % self.config["save_every"] == 0:
                self.save_model(f"epoch_{epoch+1}")
        self.save_model("best")
        
    def save_model(self, epoch):
        if self.accelerator.is_main_process:
            if not os.path.exists(self.config["save_dir"] + f"/{epoch}"):
                os.makedirs(self.config["save_dir"] + f"/{epoch}")
            
            torch.save(self.scene_adapter.state_dict(), f"{self.config['save_dir']}/{epoch}/scene_adapter.pth")
            self.model.save_pretrained(f"{self.config['save_dir']}/{epoch}")
            self.noise_scheduler.save_pretrained(f"{self.config['save_dir']}/{epoch}")
            
    def load_model(self, model_folder):
        # if self.accelerator.is_main_process:
        if not os.path.exists(model_folder):
            raise FileNotFoundError(f"Model {model_folder} not found in {self.config['save_dir']}")
        
        self.scene_adapter.load_state_dict(torch.load(f"{model_folder}/scene_adapter.pth"))
        self.model = self.model.from_pretrained(f"{model_folder}", device_map={"": f"cuda:{self.config['device_id']}"})
        self.noise_scheduler = self.noise_scheduler.from_pretrained(f"{model_folder}", device_map={"": f"cuda:{self.config['device_id']}"})
        
    def get_goal_transformation(self, object_mask, predicted_goal_mask):
        # get the transformation that takes the object mask to the predicted goal mask with the highest IoU
        # Convert masks to numpy arrays if they're tensors
        # if isinstance(object_mask, torch.Tensor):
        #     object_mask = object_mask.cpu().numpy()
        # if isinstance(predicted_goal_mask, torch.Tensor):
        #     predicted_goal_mask = predicted_goal_mask.cpu().numpy()
            
        # # Ensure binary masks
        # object_mask = (object_mask > 0.5).astype(np.uint8)
        # predicted_goal_mask = (predicted_goal_mask > 0.5).astype(np.uint8)
        
        # Calculate moments and centroids
        m_obj = cv2.moments(object_mask)
        m_goal = cv2.moments(predicted_goal_mask)
        
        if m_obj['m00'] == 0 or m_goal['m00'] == 0:
            return np.eye(3)
            
        # Extract centroids
        cx_obj, cy_obj = m_obj['m10'] / m_obj['m00'], m_obj['m01'] / m_obj['m00']
        cx_goal, cy_goal = m_goal['m10'] / m_goal['m00'], m_goal['m01'] / m_goal['m00']
        
        # print(cx_obj, cy_obj, cx_goal, cy_goal)
        
        # Initial translation estimate
        tx = cx_goal - cx_obj
        ty = cy_goal - cy_obj
        
        # Calculate orientation using central moments (principal axes)
        theta_obj = 0.5 * np.arctan2(2 * m_obj['mu11'], m_obj['mu20'] - m_obj['mu02'])
        theta_goal = 0.5 * np.arctan2(2 * m_goal['mu11'], m_goal['mu20'] - m_goal['mu02'])
        
        # print(theta_obj, theta_goal)
        
        # Initial rotation estimate (considering 2-side symmetry)
        init_angle = np.rad2deg(theta_goal - theta_obj) % 90
        
        transformed_imgs = []
        # Efficient IoU calculation
        def calculate_neg_iou(params):
            angle, tx, ty = params
            # Create transformation matrix
            M = cv2.getRotationMatrix2D((cx_obj, cy_obj), angle, 1.0)
            M[0, 2] += tx
            M[1, 2] += ty
            
            # Apply transformation
            transformed = cv2.warpAffine(object_mask, M, (object_mask.shape[1], object_mask.shape[0]))
            transformed_imgs.append(transformed)
            
            m_transformed = cv2.moments(transformed)
            cx_transformed, cy_transformed = m_transformed['m10'] / m_transformed['m00'], m_transformed['m01'] / m_transformed['m00']
            # print(cx_transformed, cy_transformed)
            
            # Calculate IoU
            intersection = np.logical_and(transformed, predicted_goal_mask).sum()
            union = np.logical_or(transformed, predicted_goal_mask).sum()
            iou = intersection / union if union > 0 else 0
            
            # Return negative IoU for minimization
            return -iou
        
        # Initial guess: [angle, tx, ty]
        x0 = [init_angle, tx, ty]
        
        # Set bounds for optimization (angle: 0-90, tx/ty: limited range)
        bounds = [(-90, 90), (tx-30, tx+30), (ty-30, ty+30)]
        
        # Direct optimization using Powell's method (derivative-free)
        result = minimize(
            calculate_neg_iou, 
            x0, 
            method='Powell',
            bounds=bounds,
            options={'maxiter': 20, 'ftol': 0.01}  # Limited iterations for speed
        )
        
        # Get optimized parameters
        angle, tx, ty = result.x
        
        # Create final transformation matrix
        M = np.eye(3)
        rot_mat = cv2.getRotationMatrix2D((cx_obj, cy_obj), angle, 1.0)
        M[:2, :2] = rot_mat[:, :2]
        M[0, 2] = rot_mat[0, 2] + tx
        M[1, 2] = rot_mat[1, 2] + ty
        
        return M, transformed_imgs
        
    def generate_image(self, scene_image=None, num_images=1):
        self.model.eval()
        self.scene_encoder.eval()
        self.scene_adapter.eval()
        
        if scene_image is None:
            # sample random image from dataset for qualitative evaluation
            img_idx = np.random.randint(0, len(self.train_dataset))
            # print(img_idx)
            inputs = self.train_dataset[img_idx]
            condition_image = inputs['diffusion_input']
            scene_image = inputs['scene'].copy()
            # condition_image = self.transforms(inputs['diffusion_input'])
            
            # print(condition_image.shape)
            # show_image = np.zero
            # plt.imshow(condition_image.clone().permute(1, 2, 0).numpy()[:, :, :3])
            # plt.savefig("train_image.png")
            # plt.close()
            
        else:
            # when running with zmq, we get the scene image from the json message (planner)
            inputs = {
                'diffusion_input': scene_image,
                'dino_input': scene_image
            }
            condition_image = self.transforms(inputs['diffusion_input'])
            
            # print(condition_image.shape)
            plt.imshow(condition_image.clone().permute(1, 2, 0).numpy()[:, :, :3])
            plt.savefig("zmq_image.png")
            plt.close()
            
        condition_image = condition_image.unsqueeze(0).to(self.device)
        
        sample = torch.randn(1, self.in_channels, self.image_size, self.image_size).to(self.device)
        
        # if self.out_channels == 1: # for correct slicing
        #     if condition_image.shape[1] >= 3:
        #         sample[:, :3, :, :] = condition_image[:, :3, :, :]
        # else:
        #     if condition_image.shape[1] == self.in_channels:
        #         pass
        #     else:
        #         sample[:, :-self.out_channels, :, :] = condition_image[:, :-self.out_channels, :, :]
        
        sample[:, :3, :, :] = condition_image[:, :3, :, :]
        sample = sample.repeat(num_images, 1, 1, 1)
        
        with torch.no_grad():
            dino_input = inputs['dino_input']
            dino_input = dino_input.to(self.device).unsqueeze(0)
            print(dino_input.shape)
            
            resnet_features = self.scene_encoder(dino_input)
            # Only adapt the features through our trainable adapter
            resnet_features = self.scene_adapter(resnet_features)
            B, C, H, W = resnet_features.shape
            resnet_features = resnet_features.view(B, C, H*W)
            resnet_features = resnet_features.repeat(num_images, 1, 1)
            for t in tqdm(reversed(range(0, self.noise_scheduler.config.num_train_timesteps))):
                if "dino_input" in inputs:
                    noise_pred = self.model(sample, t, encoder_hidden_states=resnet_features).sample
                else:
                    noise_pred = self.model(sample, t).sample
                
                sample[:, -self.out_channels:, :, :] = self.noise_scheduler.step(noise_pred, t, sample[:, -self.out_channels:, :, :]).prev_sample
            
        sample = (sample + 1) / 2
        sample = sample.clamp(0, 1).permute(0, 2, 3, 1)
        sample = sample.cpu().numpy()
        return scene_image, sample
        
def signed_angle_between(rect1, rect2):
    # first turn each rect into a unit‐vector along its "width" side:
    def axis(rect):
        _, (w, h), ang = rect
        # if you want the long side, do the w<h trick; otherwise use ang as is
        if w < h:
            ang = ang + 90
        theta = np.deg2rad(ang)
        return np.array([np.cos(theta), np.sin(theta)])
    v1 = axis(rect1)
    v2 = axis(rect2)

    # 2D "cross product" z‐component:
    cross = v1[0]*v2[1] - v1[1]*v2[0]
    dot   = np.dot(v1, v2)
    delta = np.degrees(np.arctan2(cross, dot))   # ∈ (–180, +180]

    return delta
    
def find_rectangle_corners(mask, folder_path):
    # Convert mask to binary if needed
    # if not np.issubdtype(mask.dtype, np.uint8):
    #     mask = (mask > 0.5).astype(np.uint8)
        
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return None
    # fig, ax = plt.subplots(1, 1, figsize=(16, 16))
    # ax.imshow(mask, cmap='gray')
    # for contour in contours:
    #     pts = contour.squeeze()
    #     ax.plot(pts[:, 0], pts[:, 1], 'ro-')
    # fig.savefig("contour.png")
    # plt.close()
    
    # Get the largest contour
    contour = max(contours, key=cv2.contourArea)
    
    # Find minimum area rectangle
    rect = cv2.minAreaRect(contour)
    
    # Get the 4 corners
    box = cv2.boxPoints(rect)
    box = np.int8(box)
    # Order the corners (optional, can be useful)
    # Typically: top-left, top-right, bottom-right, bottom-left
    box = order_points(box)
    
    # assume `mask` is your single‐channel image
    # fig, ax = plt.subplots()
    # ax.imshow(mask, cmap='gray')
    # # to close the loop, append the first point at the end
    # closed = np.vstack([box, box[0]])

    # # plot rectangle edges
    # ax.plot(closed[:,0], closed[:,1], '-r', linewidth=2)

    # # ax.axis('off')
    # ax.invert_yaxis()
    # fig.savefig(f"{folder_path}/rectangle.png")
    # plt.close()
    
    # Get the center and angle
    center = rect[0]
    
    w, h = rect[1]
    if w < h:
        angle = rect[2] + 90  # OpenCV gives angle in (0, 90] range
    else:
        angle = rect[2]
    
    return rect, box, center, angle

def order_points(pts):
    # Sort points by x-coordinate
    x_sorted = pts[np.argsort(pts[:, 0]), :]
    
    # Grab left-most and right-most points
    left = x_sorted[:2, :]
    right = x_sorted[2:, :]
    
    # Sort left points by y-coordinate
    left = left[np.argsort(left[:, 1]), :]
    # Sort right points by y-coordinate
    right = right[np.argsort(right[:, 1]), :]
    
    # Return ordered points: tl, tr, br, bl
    return np.vstack([left[0], right[0], right[1], left[1]])
    
if __name__ == "__main__":
    from matplotlib import pyplot as plt
    from json2image_converter import ImageConverter
    import json
    import zmq
    import cv2
    
    threshold = 0.9
    
    runner = Runner(config)
    if config["mode"] == "train":
        # runner.load_model(config["load_model_path"])
        runner.train()

    elif config["mode"] == "test":
        folder_name = os.path.join("executables/v3/diffusion_v2/arp30/resnet_1example", config['env_config_name'])
        if not os.path.exists(folder_name):
            os.makedirs(folder_name)
        runner.load_model(config["load_model_path"])
        
        k = 0
        while True:
            scene_image, samples = runner.generate_image(num_images=16)
            # for i in range(16):
            #     object_mask = (samples[i ][:, :, -2] # > 0.9).astype(np.uint8).copy()
            #     goal_mask = (samples[i][:, :, -1] # > 0.9).astype(np.uint8).copy()
                
            #     print("object mask")
            #     num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(object_mask)
            #     if num_labels == 2:
            #         moments = cv2.moments(object_mask)
            #         print(moments)
            #         if moments['m00'] == 0:
            #             continue
            #     else:
            #         continue
                
            #     print("goal mask")
            #     num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(goal_mask)
            #     if num_labels == 2:
            #         moments = cv2.moments(goal_mask)
            #         print(moments)
            #         if moments['m00'] == 0:
            #             continue
            #     else:
            #         print("goal mask has more than 2 components")
            #         continue
            #     break
            # angle transformation
            # print(obj2center_px, obj2angle)
            
            # M, transformed_images = runner.get_goal_transformation(object_mask, goal_mask)
            # print(M)
            
            
            # transformed_path = os.path.join(folder_name, "transformed")
            # if not os.path.exists(transformed_path):
            #     os.makedirs(transformed_path)
                
            # for i in range(len(transformed_images)):
            #     fig, ax = plt.subplots(1, 1, figsize=(16, 16))
                
            #     tmp_img = np.zeros((config['image_size'], config['image_size'], 3))
            #     tmp_img[:, :, 0] = transformed_images[i]
            #     tmp_img[:, :, 1] = samples[0][:, :, -2] > 0.9
            #     tmp_img[:, :, 2] = samples[0][:, :, -1] > 0.9
            #     ax.imshow(tmp_img)
            #     ax.axis('off')
            #     plt.savefig(f"{transformed_path}/transformed_{i}.png")
            #     plt.close()
            
            fig, ax = plt.subplots(4, 4, figsize=(16, 16))
            axes = ax.flatten()
            for i in range(16):
                tmp_img = np.zeros((config['image_size'], config['image_size'], 3))
                # tmp_img[:, :, 0] = samples[i][:, :, 0]
                tmp_img[:, :, 1] = samples[i][:, :, -2] 
                tmp_img[:, :, 2] = samples[i][:, :, -1] 
                axes[i].imshow(tmp_img)
                axes[i].axis('off')        
            
            plt.tight_layout()
            plt.savefig(f"{folder_name}/test_samples_{k}.png")
            plt.close()
            
            fig, ax = plt.subplots(1, 1, figsize=(16, 16))
            ax.imshow(scene_image)
            plt.savefig(f"{folder_name}/scene_image_{k}.png")
            plt.axis('off')
            plt.close()
            
            # obj_rect, obj_box, obj_center, obj_angle = find_rectangle_corners(object_mask, folder_name)
            # goal_rect, goal_box, goal_center, goal_angle = find_rectangle_corners(goal_mask, folder_name)
            # print(obj_box - goal_box) # corner transformation
            # print(obj_center, goal_center)
            # print(obj_angle, goal_angle, goal_angle - obj_angle)
            
            
            
            # goal_angle - obj_angle = +ve clockwise, -ve counterclockwise relative rotation of goal to obj
            
            # d = signed_angle_between(obj_rect, goal_rect)
            # print(d)
            
            k += 1
            input("Press Enter to show next sample...")
        
    elif config["mode"] == "zmq":
        runner.load_model(config["load_model_path"])
        
        context = zmq.Context()
        socket = context.socket(zmq.REP)
        socket.bind("tcp://arrakis.cs.rutgers.edu:5555")

        name2imgConvertor = {}
        
        k = 0
        step = 0
        while True:
            message = socket.recv_string()
            json_message = json.loads(message)
            config_name = json_message['config_name']
            new_trial = json_message['new_trial']
            if new_trial:
                k += 1
                step = 0
            step += 1
            print(config_name)
            experiment_data_path = f"executables/v3/diffusion_v2/experiment_data/{config_name}/same_env_dino_without_distance_filtering/trial_{k}"
            if not os.path.exists(experiment_data_path):
                os.makedirs(experiment_data_path)
            if config_name not in name2imgConvertor:
                name2imgConvertor[config_name] = ImageConverter(config_name)
            img_creator = name2imgConvertor[config_name]
            data = img_creator.process_datapoint(json_message)
            scene_image = data['scene']
            obj2center_px = data['obj2center_px']
            obj2angle = data['obj2angle']
            if False:
                samples = runner.generate_image(scene_image, 64)
                objects = []
                for i in range(64):
                    binary_mask = ((samples[i][:, :, -1] > threshold) * 1.0).astype(np.uint8)
                    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_mask)
                    if num_labels == 2:
                        moments = cv2.moments(binary_mask)
                        if moments['m00'] == 0:
                            socket.send_string("error")
                            continue
                        
                        scale = img_creator.IMG_SIZE / config["image_size"]
                        
                        center_p = (int(moments['m10'] / moments['m00'] * scale), int(moments['m01'] / moments['m00'] * scale))
                        
                        dist = lambda p1, p2: np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
                        min_dist = float('inf')
                        for obj_name, obj_center in obj2center_px.items():
                            d = dist(obj_center, center_p)
                            if d < min_dist:
                                min_dist = d
                                min_obj_name = obj_name
                        objects.append(min_obj_name)
                        
                counter = dict(Counter(objects))
                socket.send_string(json.dumps(counter))
                
            else:
                # fig = plt.figure(figsize=(8, 4))
                # ax = fig.gca()
                # ax.imshow(scene_image) 
                # ax.invert_yaxis()
                # fig.savefig(f"{experiment_data_path}/scene_image_{step}.png")
                # plt.close()
                
                num_samples = 32
                samples = runner.generate_image(scene_image, num_samples)
                votes = {}
                
                message_sent = False
                
                for i in range(num_samples):
                    object_mask_unthresholded = samples[i][:, :, -2].copy()
                    goal_mask_unthresholded = samples[i][:, :, -1].copy()
                    object_mask = (samples[i][:, :, -2] > 0.9).astype(np.uint8).copy()
                    goal_mask = (samples[i][:, :, -1] > 0.9).astype(np.uint8).copy()
                    
                    fig, ax = plt.subplots(2, 3, figsize=(12, 8))
                    ax[0, 0].imshow(scene_image) 
                    ax[0, 0].invert_yaxis()
                    ax[0, 1].imshow(object_mask_unthresholded, cmap='gray')
                    ax[0, 1].invert_yaxis()
                    ax[0, 2].imshow(goal_mask_unthresholded, cmap='gray')
                    ax[0, 2].invert_yaxis()
                    ax[1, 0].imshow(scene_image)
                    ax[1, 0].invert_yaxis()
                    ax[1, 1].imshow(object_mask, cmap='gray')
                    ax[1, 1].invert_yaxis()
                    ax[1, 2].imshow(goal_mask, cmap='gray')
                    ax[1, 2].invert_yaxis()
                    if not os.path.exists(f"{experiment_data_path}/samples"):
                        os.makedirs(f"{experiment_data_path}/samples")
                    plt.savefig(f"{experiment_data_path}/samples/plan_step_{step}_sample_{i}.png")
                    plt.close()
                    
                    # print("object mask")
                    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(object_mask)
                    if num_labels == 2:
                        moments = cv2.moments(object_mask)
                        # print(moments)
                        if moments['m00'] == 0:
                            continue
                    else:
                        continue
                    
                    # print("goal mask")
                    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(goal_mask)
                    if num_labels == 2:
                        moments = cv2.moments(goal_mask)
                        # print(moments)
                        if moments['m00'] == 0:
                            continue
                    else:
                        # print("goal mask has more than 2 components")
                        continue
                
                    obj_rect, obj_box, predicted_obj_center, obj_angle = find_rectangle_corners(object_mask, experiment_data_path)
                    goal_rect, goal_box, goal_center, goal_angle = find_rectangle_corners(goal_mask, experiment_data_path)
                    
                    dist = lambda p1, p2: np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
                    
                    scale = img_creator.IMG_SIZE / config["image_size"]
                    predicted_obj_center = (int(predicted_obj_center[0] * scale), int(predicted_obj_center[1] * scale))
                    goal_center = (int(goal_center[0] * scale), int(goal_center[1] * scale))
                    
                    # find object
                    min_dist = float('inf')
                    for obj_name, obj_center in obj2center_px.items():
                        d = dist(obj_center, predicted_obj_center)
                        if d < min_dist:
                            min_dist = d
                            min_obj_name = obj_name
                    
                    
                    
                    obj_center = obj2center_px[min_obj_name]
                    obj_center = list(img_creator.pixel_to_world(obj_center[0], obj_center[1]))
                    goal_center = list(img_creator.pixel_to_world(goal_center[0], goal_center[1]))
                    
                    # print(obj_center, goal_center, dist(obj_center, goal_center))
                    
                    # distance filtering
                    # if dist(obj_center, goal_center) > 0.65:
                    #     continue
                    
                    print("selecting object", min_obj_name)
                    
                    fig, ax = plt.subplots(2, 3, figsize=(12, 8))
                    ax[0, 0].imshow(scene_image) 
                    ax[0, 0].invert_yaxis()
                    ax[0, 1].imshow(object_mask_unthresholded, cmap='gray')
                    ax[0, 1].invert_yaxis()
                    ax[0, 2].imshow(goal_mask_unthresholded, cmap='gray')
                    ax[0, 2].invert_yaxis()
                    ax[1, 0].imshow(scene_image)
                    ax[1, 0].invert_yaxis()
                    ax[1, 1].imshow(object_mask, cmap='gray')
                    ax[1, 1].invert_yaxis()
                    ax[1, 2].imshow(goal_mask, cmap='gray')
                    ax[1, 2].invert_yaxis()
                    plt.savefig(f"{experiment_data_path}/object_goal_mask_{step}.png")
                    plt.close()

                    final_quat = img_creator.rotate_relative_to_world(min_obj_name, goal_angle - obj_angle)
                    # print(goal_center, goal_angle, final_quat)
                    
                    # for i in range(num_samples):
                    #     binary_mask = ((samples[i][:, :, -1] > threshold) * 1.0).astype(np.uint8)
                    #     num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_mask)
                    #     if num_labels != 2:
                    #         continue
                    #     moments = cv2.moments(binary_mask)
                    #     if moments['m00'] == 0:
                    #         socket.send_string("error")
                    #         continue
                        
                    #     scale = img_creator.IMG_SIZE / config["image_size"]
                        
                    #     center_p = (int(moments['m10'] / moments['m00'] * scale), int(moments['m01'] / moments['m00'] * scale))
                        
                    #     dist = lambda p1, p2: np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
                    #     min_dist = float('inf')
                    #     for obj_name, obj_center in obj2center_px.items():
                            
                    #         d = dist(obj_center, center_p)
                    #         if d < min_dist:
                    #             min_dist = d
                    #             min_obj_name = obj_name
                    #     if min_obj_name not in votes:
                    #         votes[min_obj_name] = 0
                    #     votes[min_obj_name] += 1
                        
                    # json_message['object'] = max(votes, key=votes.get)
                    
                    # new_json_message = {
                    #     'object': json_message['object'],
                    # }
                    new_json_message = {
                        'object': min_obj_name,
                        'goal_center': goal_center,
                        'final_quat': final_quat.tolist(),
                        'error': False,
                        'error_message': ""
                    }
                    socket.send_string(json.dumps(new_json_message))
                    message_sent = True
                    break
                
                if not message_sent:
                    error_message = {
                        'error': True,
                        'error_message': "No object found"
                    }
                    socket.send_string(json.dumps(error_message))
                
                
                # print(min_obj_name)
                
                # fig, ax = plt.subplots(4, 4, figsize=(8, 4))
                # axes = ax.flatten()
                # tmp_img = np.zeros((config["image_size"], config["image_size"], 3))
                # for i in range(num_samples):
                #     tmp_img[:, :, 0] = 0
                #     tmp_img[:, :, 1] = samples[i][:, :, -2] > threshold
                #     tmp_img[:, :, 2] = samples[i][:, :, -1] > threshold
                #     axes[i].imshow(tmp_img)
                #     axes[i].axis('off')
                # folder_name = os.path.join("executables/v3/diffusion_v2", config_name)
                # if not os.path.exists(folder_name):
                #         os.makedirs(folder_name)
                # fig.savefig(f"{folder_name}/thresh_{k}.png")
                # plt.close()
                
            
            
            
        
        