import yaml
import numpy as np

def convert_yaml_to_dict(yaml_file):
    with open(yaml_file, 'r') as file:
        return yaml.safe_load(file)

def get_rectangle_corners(pos, size, rot):
    # get four corners of the rectangle given the center, size, and rotation
    corners = np.array([[-size[0]/2, -size[1]/2], 
                       [size[0]/2, -size[1]/2], 
                       [size[0]/2, size[1]/2], 
                       [-size[0]/2, size[1]/2]])
    
    # Create 2x2 rotation matrix
    rotation_matrix = np.array([[np.cos(rot), -np.sin(rot)],
                              [np.sin(rot), np.cos(rot)]])
    
    # Apply rotation using matrix multiplication
    corners = np.dot(corners, rotation_matrix.T)
    
    # Translate to final position
    corners += pos
    return corners
