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

def create_binary_images(data, object_sizes, grid_size=256, world_bounds=(-3, 3)):
    """
    Creates two binary images with specific channels:
    Image 1: [movable objects, robot]
    Image 2: [edge point, midpoint]
    
    Args:
        data: Data point dictionary with state and action information
        object_sizes: Dictionary mapping object names to their sizes
        grid_size: Resolution of the output images
        world_bounds: (min, max) bounds of the world in both dimensions
    
    Returns:
        Two numpy arrays representing the binary images
    """
    import numpy as np
    from scipy.spatial.transform import Rotation as R
    
    # Initialize empty images
    image1 = np.zeros((grid_size, grid_size, 3), dtype=np.uint8)  # [objects, robot]
    image2 = np.zeros((grid_size, grid_size, 3), dtype=np.uint8)  # [edge point, midpoint]
    
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
        image1[:, :, 0] = np.logical_or(image1[:, :, 0], obj_grid_resized > 0).astype(np.uint8)
    
    # Add robot to image1, channel 1
    robot_pos = data['state']['robot']['position']
    robot_x, robot_y = world_to_grid(robot_pos[0], robot_pos[1])
    
    # Draw robot as a circle with radius
    robot_radius = int(0.1 / world_range * grid_size)
    y, x = np.ogrid[:grid_size, :grid_size]
    dist_from_center = np.sqrt((x - robot_x)**2 + (y - robot_y)**2)
    robot_mask = dist_from_center <= robot_radius
    image1[:, :, 1][robot_mask] = 1
    
    
    # Add edge point to image2, channel 0
    try:
        edge_pt = data['action']['edge_point']
        edge_x, edge_y = world_to_grid(edge_pt[0], edge_pt[1])
        # image2[edge_y, edge_x, 0] = 1
        edge_radius = int(0.1 / world_range * grid_size)
        y, x = np.ogrid[:grid_size, :grid_size]
        dist_from_center = np.sqrt((x - edge_x)**2 + (y - edge_y)**2)
        edge_pt_mask = dist_from_center <= edge_radius
        # image1[:, :, 2][edge_pt_mask] = 1
        image2[:, :, 0][edge_pt_mask] = 1
        
    except (KeyError, IndexError):
        pass
    
    # Add midpoint to image2, channel 1
    try:
        mid_pt = data['action']['mid_point']
        mid_x, mid_y = world_to_grid(mid_pt[0], mid_pt[1])
        # image2[mid_y, mid_x, 1] = 1
        
        mid_radius = int(0.1 / world_range * grid_size)
        y, x = np.ogrid[:grid_size, :grid_size]
        dist_from_center = np.sqrt((x - mid_x)**2 + (y - mid_y)**2)
        mid_pt_mask = dist_from_center <= mid_radius
        # image1[:, :, 2][mid_pt_mask] = 1
        image2[:, :, 1][mid_pt_mask] = 1
    except (KeyError, IndexError):
        pass
    
    return image1, image2

# You can now use this function like:
# image1, image2 = create_binary_images(dp, load_object_sizes(env_config_name))

# To visualize these images:
def visualize_binary_images(image1, image2):
    import matplotlib.pyplot as plt
    
    fig, axes = plt.subplots(2, 2, figsize=(10, 10))
    
    axes[0, 0].imshow((image1[:, :, :]) * 255)
    axes[0, 0].set_title('Movable Objects')
    axes[0, 0].axis('off')
    axes[0, 0].invert_yaxis()
    
    axes[0, 1].imshow(image1[:, :, 1], cmap='binary')
    axes[0, 1].set_title('Robot')
    axes[0, 1].axis('off')
    axes[0, 1].invert_yaxis()
    
    
    axes[1, 0].imshow(image2[:, :, :] * 255)
    axes[1, 0].set_title('Edge Point')
    axes[1, 0].axis('off')
    axes[1, 0].invert_yaxis()
    
    
    axes[1, 1].imshow(image2[:, :, 1], cmap='binary')
    axes[1, 1].set_title('Mid Point')
    axes[1, 1].axis('off')
    axes[1, 1].invert_yaxis()
    
    
    plt.tight_layout()
    plt.savefig(f'{env_config_name}_binary_images.png')

if __name__ == "__main__":
    # from collections import OrderedDict
    import os
    from glob import glob
    import json
    import numpy as np

    count = {}
    env_config_name = "env_config_1"
    SRC_DIR = f'/common/users/dm1487/namo_data/{env_config_name}'
    FILES = glob(os.path.join(SRC_DIR, '*.json'))

    ctr = 0
    all_datapoints = []
    for file in FILES:
        with open(file, 'r') as f:
            data = json.load(f)
        
        if len(data['data_points']) not in count:
            count[len(data['data_points'])] = 0
        count[len(data['data_points'])] += 1
        all_datapoints.extend(data['data_points'][:-1])
        # plot_something(data, env_config_name)
        ctr += 1
        # print(data['data_points'][1]['action'])
        if ctr > 5:
            break

    
    for dp in all_datapoints[:]:
        image1, image2 = create_binary_images(dp, load_object_sizes(env_config_name), grid_size=256, world_bounds=(-3, 3))
        visualize_binary_images(image1, image2)
        