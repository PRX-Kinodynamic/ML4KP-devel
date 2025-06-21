# zmq_server.py
import zmq
import time
import json
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import csv


class ImageConverter:
    IMG_SIZE = 224
    WORLD_SIZE = 6.0  # -3 to 3
    SCALE = IMG_SIZE / WORLD_SIZE  # pixels per world unit
    data_point = None
    def __init__(self, env_config_name):
        self.object_sizes = self._load_object_sizes(env_config_name)
    
    @staticmethod
    def _load_object_sizes(env_config_name):
        base_dir = "/common/home/dm1487/robotics_research/ktamp/ml4kp_ktamp/custom_walled_envs/apr20_25/fixed_start_fixed_goal_many_env"
        filename = f"{base_dir}/namo_objects_{env_config_name}.txt"
        object_sizes = {}
        with open(filename, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                name = row["object_name"]
                size_x = float(row["size_x"])
                size_y = float(row["size_y"])
                object_sizes[name] = (size_x, size_y)
        return object_sizes
    
    def pixel_to_world(self, px, py):
        x = (px / self.SCALE) - self.WORLD_SIZE/2
        y = (py / self.SCALE) - self.WORLD_SIZE/2
        return x, y
    
    def rotate_relative_to_world(self, obj_name, angle):
        if self.data_point is None:
            raise ValueError("data_point is not set")
        # obj_center = self.data_point['objects'][obj_name]['position']
        obj_angle = self.data_point['objects'][obj_name]['quaternion']
        obj_angle = R.from_quat(obj_angle, scalar_first=True).as_euler('xyz', degrees=True)[2]
        # print("converting obj_angle, angle", obj_angle, angle, "to", obj_angle + angle)
        final_quat  = R.from_euler('xyz', [0, 0, obj_angle + angle], degrees=True).as_quat(scalar_first=True)
        return final_quat
    
    def _world_to_pixel(self, x, y):
        px = int((x + self.WORLD_SIZE/2) * self.SCALE)
        py = int((y + self.WORLD_SIZE/2) * self.SCALE)
        return px, py
    
    def _pixel_to_world(self, px, py):
        x = (px / self.SCALE) - self.WORLD_SIZE/2
        y = (py / self.SCALE) - self.WORLD_SIZE/2
        return x, y
    
    def _draw_rotated_rectangle(self, img, center, size, angle, color):
        center_px = self._world_to_pixel(center[0], center[1])
        size_px = (int(size[0] * self.SCALE), int(size[1] * self.SCALE))
        rect = ((center_px[0], center_px[1]), size_px, angle)
        box = cv2.boxPoints(rect)
        box = np.int32(box)
        cv2.fillPoly(img, [box], color)
        return img, center_px
    
    def _draw_circle(self, img, center, radius, color):
        center_px = self._world_to_pixel(center[0], center[1])
        radius_px = int(radius * self.SCALE)
        cv2.circle(img, center_px, radius_px, color, -1)
        return img
    
    def process_datapoint(self, data_point):
        # Create base images
        self.data_point = data_point
        scene_image = np.zeros((self.IMG_SIZE, self.IMG_SIZE, 3), dtype=np.uint8)
        
        # Draw robot
        robot_pos = data_point['robot']['position']
        self._draw_circle(scene_image, (robot_pos[0], robot_pos[1]), 0.2, (255, 0, 0))
        
        # Draw goal indicator
        self._draw_circle(scene_image, (2.5, 2.5), 0.25, (0, 255, 0))
        
        obj2center_px = {}
        obj2angle = {}
        # Draw objects
        for obj_name, obj_data in data_point['objects'].items():
            size_x, size_y = self.object_sizes[obj_name]
            size_x *= 2
            size_y *= 2
            rotation = R.from_quat(obj_data['quaternion'], scalar_first=True).as_euler('xyz', degrees=True)[2]
            scene_image, center_px = self._draw_rotated_rectangle(scene_image,
                                      (obj_data['position'][0], obj_data['position'][1]),
                                      (size_x, size_y),
                                      rotation,
                                      (255, 255, 0))
            obj2center_px[obj_name] = center_px
            obj2angle[obj_name] = rotation
        # Normalize arrays
        return {
            'scene': scene_image.astype(np.float32) / 255.0,
            'obj2center_px': obj2center_px,
            'obj2angle': obj2angle
        }