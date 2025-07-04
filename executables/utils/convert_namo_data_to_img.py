import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import os
import json
import csv
import mujoco
from multiprocessing import Pool
import pickle
from tqdm.contrib.concurrent import process_map

class ImageConverter:
    IMG_SIZE = 224
    WORLD_SIZE = 6.0  # -3 to 3
    SCALE = IMG_SIZE / WORLD_SIZE  # pixels per world unit
    
    def __init__(self, env_config_name, model_folder, model_type):
        self.object_sizes = self.get_geom_sizes(env_config_name, model_folder, model_type)
    
    @staticmethod
    def get_geom_sizes(config_name, model_dir, model_type):
        model = mujoco.MjModel.from_xml_path(f"{model_dir}/{model_type}/{config_name}.xml")
        # Get all geom information
        geom_sizes = {}
        for i in range(model.ngeom):
            # Get geom name
            geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
            if geom_name is None:
                geom_name = f"geom_{i}"
            geom_sizes[geom_name] = model.geom_size[i]
        del model
        return geom_sizes
    
    def _world_to_pixel(self, x, y):
        px = int((x + self.WORLD_SIZE/2) * self.SCALE)
        py = int((y + self.WORLD_SIZE/2) * self.SCALE)
        return px, py
    
    def _draw_rotated_rectangle(self, img, center, size, angle, color):
        center_px = self._world_to_pixel(center[0], center[1])
        size_px = (int(size[0] * self.SCALE), int(size[1] * self.SCALE))
        rect = ((center_px[0], center_px[1]), size_px, angle)
        box = cv2.boxPoints(rect)
        box = np.int32(box)
        cv2.fillPoly(img, [box], color)
        return img
    
    def _draw_circle(self, img, center, radius, color):
        center_px = self._world_to_pixel(center[0], center[1])
        radius_px = int(radius * self.SCALE)
        cv2.circle(img, center_px, radius_px, color, -1)
        return img
    
    def process_datapoint(self, data_point, robot_goal_pos, obj_true_next_pos):
        # Create base images (already correctly initialized as 1-channel)
        robot_image = np.zeros((self.IMG_SIZE, self.IMG_SIZE, 1), dtype=np.uint8)
        goal_image = np.zeros((self.IMG_SIZE, self.IMG_SIZE, 1), dtype=np.uint8)
        movable_objects_image = np.zeros((self.IMG_SIZE, self.IMG_SIZE, 1), dtype=np.uint8)
        static_objects_image = np.zeros((self.IMG_SIZE, self.IMG_SIZE, 1), dtype=np.uint8)
        reachable_objects_image = np.zeros((self.IMG_SIZE, self.IMG_SIZE, 1), dtype=np.uint8)
        object_mask = np.zeros((self.IMG_SIZE, self.IMG_SIZE, 1), dtype=np.uint8)
        goal_mask = np.zeros((self.IMG_SIZE, self.IMG_SIZE, 1), dtype=np.uint8)
        true_goal_mask = np.zeros((self.IMG_SIZE, self.IMG_SIZE, 1), dtype=np.uint8)
        
        
        
        # Draw robot
        robot_pos = data_point['state']['robot']['position']
        self._draw_circle(robot_image, (robot_pos[0], robot_pos[1]), 0.2, 255)
        
        # Draw goal indicator
        self._draw_circle(goal_image, (robot_goal_pos[0], robot_goal_pos[1]), 0.25, 255)
        
        # Draw objects - differentiate between movable and static
        for obj_name, obj_data in data_point['state']['objects'].items():
            size_x, size_y = self.object_sizes[obj_name][0], self.object_sizes[obj_name][1]
            size_x *= 2
            size_y *= 2
            rotation = R.from_quat(obj_data['quaternion'], scalar_first=True).as_euler('xyz', degrees=True)[2]
            
            # Check if object is movable based on name containing "movable"
            if "movable" in obj_name.lower():
                # Draw on movable objects image
                self._draw_rotated_rectangle(movable_objects_image,
                                          (obj_data['position'][0], obj_data['position'][1]),
                                          (size_x, size_y),
                                          rotation,
                                          255)
            else:
                # Draw on static objects image
                self._draw_rotated_rectangle(static_objects_image,
                                          (obj_data['position'][0], obj_data['position'][1]),
                                          (size_x, size_y),
                                          rotation,
                                          255)
            
            # Draw reachable objects
            if obj_name in data_point['state']['reachable_objects']:
                self._draw_rotated_rectangle(reachable_objects_image,
                                          (obj_data['position'][0], obj_data['position'][1]),
                                          (size_x, size_y),
                                          rotation,
                                          255)
        
        # Draw masks if action exists
        if 'action' in data_point:
            goal_obj = data_point['action']['object_name']
            obj_data = data_point['state']['objects'][goal_obj]
            size_x, size_y = self.object_sizes[goal_obj][0], self.object_sizes[goal_obj][1]
            size_x *= 2
            size_y *= 2
            
            # Object mask
            rotation = R.from_quat(obj_data['quaternion'], scalar_first=True).as_euler('xyz', degrees=True)[2]
            self._draw_rotated_rectangle(object_mask,
                                      (obj_data['position'][0], obj_data['position'][1]),
                                      (size_x, size_y),
                                      rotation,
                                      255)
            
            # Goal mask
            goal_state = data_point['action']['goal_state']
            rotation = R.from_quat(goal_state['quaternion'], scalar_first=True).as_euler('xyz', degrees=True)[2]
            self._draw_rotated_rectangle(goal_mask,
                                      (goal_state['position'][0], goal_state['position'][1]),
                                      (size_x, size_y),
                                      rotation,
                                      255)
            
        # True goal mask
        self._draw_rotated_rectangle(true_goal_mask,
                                    (obj_true_next_pos['position'][0], obj_true_next_pos['position'][1]),
                                    (size_x, size_y),
                                    rotation,
                                    255)
        
        # Normalize arrays and return all images
        return {
            'robot_image': robot_image.astype(np.float32) / 255.0,
            'goal_image': goal_image.astype(np.float32) / 255.0,
            'movable_objects_image': movable_objects_image.astype(np.float32) / 255.0,
            'static_objects_image': static_objects_image.astype(np.float32) / 255.0,
            'reachable_objects_image': reachable_objects_image.astype(np.float32) / 255.0,
            'object_mask': object_mask.astype(np.float32) / 255.0,
            'goal_mask': goal_mask.astype(np.float32) / 255.0,
            'true_goal_mask': true_goal_mask.astype(np.float32) / 255.0
        }

def process_file(file_path, save_dir, model_folder, model_type):
    try:
        
        # env_config_name = file_path.split("/")[-1].split("_")[4:7]
        with open(file_path, 'r') as f:
            data = json.load(f)
            
        # print(data)
            
        # print(len(data['data_points']))
                    
        # if len(data['data_points']) > 2 and len(data['data_points']) < 11:
        env_config_name = data['config_name']
        # print(env_config_name)
        converter = ImageConverter(env_config_name, model_folder, model_type)
        
        base_filename = os.path.splitext(os.path.basename(file_path))[0]
        processed_count = 0
        
        robot_goal_pos = data['robot_goal']
        
        for idx, datapoint in enumerate(data['data_points'][:-1]):
            # Process single datapoint
            
            obj_name = datapoint['action']['object_name']
            obj_true_next_pos = {'position': data['data_points'][idx+1]['state']['objects'][obj_name]['position'], 'quaternion': data['data_points'][idx+1]['state']['objects'][obj_name]['quaternion']}
            
            processed_data = converter.process_datapoint(datapoint, robot_goal_pos, obj_true_next_pos)
            
            
            # Create unique filename with timestamp to avoid collisions
            save_path = os.path.join(save_dir, f"{base_filename}_dp{idx:06d}.npz")
            
            # Atomic save operation
            np.savez(save_path, **processed_data)
            processed_count += 1
        
        #     print("-" * 50)
        # print("-" * 50)
            
            return f"Successfully processed {processed_count} datapoints from {base_filename}"
    except Exception as e:
        return f"Error processing {file_path}: {str(e)}"

def main():
    
    model_folder = "resources/models/custom_walled_envs/jun22"
    model_type = "random_start_random_goal_single_obstacle_room_2_200k_halfrad"
    data_dir = f"/common/users/dm1487/namo_data/jul3/random_start_random_goal_single_obstacle_room_2_200k"    
    save_dir = f"/common/users/dm1487/namo_data/images/jul3/random_start_random_goal_single_obstacle_room_2_big"
    os.makedirs(save_dir, exist_ok=True)
    
    # Collect all json files
    json_files = [os.path.join(data_dir, f) for f in os.listdir(data_dir) if f.endswith('.json')]
    
    print(f"Found {len(json_files)} JSON files to process")
    
    # Create arguments for process_file
    args = [(f, save_dir, model_folder, model_type) for f in json_files]
    
    # Calculate optimal chunksize for better performance
    # Rule of thumb: chunksize = max(1, len(iterable) // (workers * 4))
    optimal_chunksize = max(1, len(args) // (12 * 4))
    
    # Process files in parallel with progress bar using tqdm's process_map
    results = process_map(
        process_file_wrapper, 
        args, 
        max_workers=12,
        chunksize=optimal_chunksize,
        desc="Processing files",
        unit="file"
    )
    
    # Count successful and failed processing
    successful = sum(1 for result in results if "Successfully processed" in result)
    failed = sum(1 for result in results if "Error processing" in result)
    
    print(f"\nProcessing complete!")
    print(f"Successfully processed: {successful} files")
    print(f"Failed to process: {failed} files")
    
    # Optionally print error details
    # if failed > 0:
    #     print("\nError details:")
    #     for result in results:
    #         if "Error processing" in result:
    #             print(f"  {result}")

def process_file_wrapper(args):
    """Wrapper function to unpack arguments for process_file"""
    return process_file(*args)

if __name__ == "__main__":
    main()