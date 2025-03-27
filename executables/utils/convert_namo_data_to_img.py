def load_object_sizes(env_config_name):
    filename = f"namo_objects_{env_config_name}.txt"
    import csv
    object_sizes = {}
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            name = row["object_name"]
            size_x = float(row["size_x"])
            size_y = float(row["size_y"])
            object_sizes[name] = (size_x, size_y)
    return object_sizes

def create_binary_images(data, object_sizes, grid_size=256, world_bounds=(-3, 3), use_gaussian=False):
    """
    Creates two binary images with specific channels:
    Image 1: [movable objects, robot]
    Image 2: [edge point, midpoint]
    
    Args:
        data: Data point dictionary with state and action information
        object_sizes: Dictionary mapping object names to their sizes
        grid_size: Resolution of the output images
        world_bounds: (min, max) bounds of the world in both dimensions
        use_gaussian: If True, use Gaussian distributions for points instead of binary circles
    
    Returns:
        Two numpy arrays representing the binary images
    """
    import numpy as np
    from scipy.spatial.transform import Rotation as R
    
    # Initialize empty images - both as float32
    image1 = np.zeros((grid_size, grid_size, 3), dtype=np.float32)  # [objects, robot]
    image2 = np.zeros((grid_size, grid_size, 3), dtype=np.float32)  # [edge point, midpoint]
    
    world_min, world_max = world_bounds
    world_range = world_max - world_min
    
    # Helper function to convert world coordinates to grid indices
    def world_to_grid(x, y):
        grid_x = int((x - world_min) / world_range * (grid_size - 1))
        grid_y = int((y - world_min) / world_range * (grid_size - 1))
        
        # Ensure values are within grid bounds
        grid_x = max(0, min(grid_size - 1, grid_x))
        grid_y = max(0, min(grid_size - 1, grid_y))
        return grid_x, grid_y
    
    # Add movable objects to image1, channel 0
    state_pt = data['state']
    for key, obj in state_pt['objects'].items():
        size = object_sizes[key]
        pos_x, pos_y = obj['position'][0], obj['position'][1]
        rot = R.from_quat(obj['quaternion'], scalar_first=True).as_euler('xyz', degrees=True)[2]
        
        # Create a higher resolution grid for the object and then downsample
        obj_grid_size = grid_size * 4  # Higher resolution for better accuracy
        obj_grid = np.zeros((obj_grid_size, obj_grid_size), dtype=np.uint8)
        
        # Convert object bounds to grid coordinates
        center_x, center_y = (pos_x - world_min) / world_range * (obj_grid_size - 1), (pos_y - world_min) / world_range * (obj_grid_size - 1)
        width = (size[0] * 2) / world_range * (obj_grid_size - 1)
        height = (size[1] * 2) / world_range * (obj_grid_size - 1)
        
        # Create mask for rotated rectangle
        from skimage.draw import polygon
        
        # Calculate corner points of the rectangle
        corners = [
            [center_x - width/2, center_y - height/2],
            [center_x + width/2, center_y - height/2],
            [center_x + width/2, center_y + height/2],
            [center_x - width/2, center_y + height/2]
        ]
        
        # Rotate corners around center
        import math
        rot_rad = math.radians(rot)
        rotated_corners = []
        for x, y in corners:
            dx, dy = x - center_x, y - center_y
            rotated_x = center_x + dx * math.cos(rot_rad) - dy * math.sin(rot_rad)
            rotated_y = center_y + dx * math.sin(rot_rad) + dy * math.cos(rot_rad)
            rotated_corners.append([rotated_x, rotated_y])
        
        # Extract x and y coordinates for polygon function
        rr, cc = polygon([c[1] for c in rotated_corners], [c[0] for c in rotated_corners], obj_grid.shape)
        obj_grid[rr, cc] = 1
        
        # Downsample to original grid size
        from skimage.transform import resize
        obj_grid_resized = resize(obj_grid, (grid_size, grid_size), order=0, anti_aliasing=False)
        image1[:, :, 0] = np.logical_or(image1[:, :, 0], obj_grid_resized > 0).astype(np.float32)
    
    # Add robot to image1, channel 1
    robot_pos = data['state']['robot']['position']
    robot_x, robot_y = world_to_grid(robot_pos[0], robot_pos[1])
    
    # Draw robot as a circle with radius
    robot_radius = int(0.1 / world_range * grid_size)
    y, x = np.ogrid[:grid_size, :grid_size]
    dist_from_center = np.sqrt((x - robot_x)**2 + (y - robot_y)**2)
    robot_mask = dist_from_center <= robot_radius
    image1[:, :, 1][robot_mask] = 1.0  # Use 1.0 instead of 1
    
    
    # Add edge point to image2, channel 0
    try:
        edge_pt = data['action']['edge_point']
        edge_x, edge_y = world_to_grid(edge_pt[0], edge_pt[1])
        
        if use_gaussian:
            # Create Gaussian distribution for edge point
            sigma = 0.1 / world_range * grid_size  # Remove int() to keep precision
            y, x = np.ogrid[:grid_size, :grid_size]
            gaussian = np.exp(-0.5 * ((x - edge_x)**2 + (y - edge_y)**2) / (sigma**2))
            # Normalize to ensure maximum value is 1
            gaussian = gaussian / gaussian.max()
            image2[:, :, 0] = gaussian.astype(np.float32)
        else:
            # Use binary circle
            edge_radius = int(0.1 / world_range * grid_size)
            y, x = np.ogrid[:grid_size, :grid_size]
            dist_from_center = np.sqrt((x - edge_x)**2 + (y - edge_y)**2)
            edge_pt_mask = dist_from_center <= edge_radius
            image2[:, :, 0][edge_pt_mask] = 1.0  # Use 1.0 instead of 1
        
    except (KeyError, IndexError):
        pass
    
    # Add midpoint to image2, channel 1
    try:
        mid_pt = data['action']['mid_point']
        mid_x, mid_y = world_to_grid(mid_pt[0], mid_pt[1])
        
        if use_gaussian:
            # Create Gaussian distribution for mid point
            sigma = 0.1 / world_range * grid_size  # Remove int() to keep precision
            y, x = np.ogrid[:grid_size, :grid_size]
            gaussian = np.exp(-0.5 * ((x - mid_x)**2 + (y - mid_y)**2) / (sigma**2))
            # Normalize to ensure maximum value is 1
            gaussian = gaussian / gaussian.max()
            image2[:, :, 1] = gaussian.astype(np.float32)
        else:
            # Use binary circle
            mid_radius = int(0.1 / world_range * grid_size)
            y, x = np.ogrid[:grid_size, :grid_size]
            dist_from_center = np.sqrt((x - mid_x)**2 + (y - mid_y)**2)
            mid_pt_mask = dist_from_center <= mid_radius
            image2[:, :, 1][mid_pt_mask] = 1.0  # Use 1.0 instead of 1
        
    except (KeyError, IndexError):
        pass
    
    return image1, image2

def visualize_binary_images(image1, image2):
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(10, 10))
    
    axes[0, 0].imshow((image1[:, :, :]))
    axes[0, 0].set_title('Movable Objects')
    axes[0, 0].axis('off')
    axes[0, 0].invert_yaxis()
    
    axes[0, 1].imshow(image1[:, :, 1], cmap='binary')
    axes[0, 1].set_title('Robot')
    axes[0, 1].axis('off')
    axes[0, 1].invert_yaxis()
    
    
    axes[1, 0].imshow(image2[:, :, :], cmap = 'plasma')
    axes[1, 0].set_title('Edge Point')
    axes[1, 0].axis('off')
    axes[1, 0].invert_yaxis()
    
    
    axes[1, 1].imshow(image2[:, :, 1], cmap='plasma')
    axes[1, 1].set_title('Mid Point')
    axes[1, 1].axis('off')
    axes[1, 1].invert_yaxis()
    
    
    plt.tight_layout()
    plt.savefig(f'binary_images.png')

def process_datapoint(args, use_gaussian=False):
    """
    Process a single datapoint and save the resulting images.
    
    Args:
        args: Tuple containing (index, datapoint, env_config_name, dest_dir)
    """
    i, dp, env_config_name, dest_dir = args
    image1, image2 = create_binary_images(dp, load_object_sizes(env_config_name), grid_size=128, world_bounds=(-3, 3), use_gaussian=use_gaussian)
    np.savez_compressed(f'{dest_dir}/{i}.npz', inp=image1, out=image2)
    # visualize_binary_images(image1, image2)
if __name__ == "__main__":
    # from collections import OrderedDict
    import os
    from glob import glob
    import json
    import numpy as np
    from tqdm import tqdm
    import multiprocessing as mp
    from functools import partial
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--use_gaussian', action='store_true', help='Use Gaussian distributions for points')
    parser.add_argument('--use_mp', action='store_true', help='Use multiprocessing')
    args = parser.parse_args()
    
    count = {}
    env_config_name = "env_config_1"
    BASE_DIR = '/common/users/dm1487/namo_data'
    src_dir = f'{BASE_DIR}/{env_config_name}'
    dest_dir = f'{BASE_DIR}/binary_images/{env_config_name}{"_gaussian" if args.use_gaussian else ""}'
    os.makedirs(dest_dir, exist_ok=True)
    FILES = glob(os.path.join(src_dir, '*.json'))
    
    ctr = 0
    all_datapoints = []
    for file in FILES:
        with open(file, 'r') as f:
            data = json.load(f)
        
        if len(data['data_points']) not in count:
            count[len(data['data_points'])] = 0
        count[len(data['data_points'])] += 1
        start_idx = 0
        if np.random.uniform() < 0.3:
            start_idx = 1
        all_datapoints.extend(data['data_points'][start_idx:-1])
    
    # Prepare arguments for parallel processing
    args_list = [(i, dp, env_config_name, dest_dir) for i, dp in enumerate(all_datapoints[:])]
    
    

    if args.use_mp:
        # Multiprocessing version
        num_cpus = 24
        print(f"Using {num_cpus} CPU cores for parallel processing")
        with mp.Pool(processes=num_cpus) as pool:
            list(tqdm(pool.imap(partial(process_datapoint, use_gaussian=args.use_gaussian), 
                               args_list), total=len(args_list)))
    else:
        # Single process version
        print("Running in single process mode")
        for arg in tqdm(args_list):
            process_datapoint(arg, use_gaussian=args.use_gaussian)
            break