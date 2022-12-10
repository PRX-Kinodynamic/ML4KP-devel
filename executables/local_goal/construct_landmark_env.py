import numpy as np 
import matplotlib.pyplot as plt
import yaml
import os
from matplotlib.patches import Rectangle

np.random.seed(210896)

yaml_to_dump = {}
yaml_to_dump["environment"] = {}
yaml_to_dump["environment"]["type"] = "obstacle"
yaml_to_dump["environment"]["geometries"] = []

upper_row = {
    "name": "upper",
    "dynamic": False,
    "collision_geometry": {
        "type": "box",
        "dims": [21,0.5,0.2],
        "material": "red"
    },
    "config": {
        "position": [0,10.25,0],
        "orientation": [0,0,0,1]
    }
}

lower_row = {
    "name": "lower",
    "dynamic": False,
    "collision_geometry": {
        "type": "box",
        "dims": [21,0.5,0.2],
        "material": "red"
    },
    "config": {
        "position": [0,-10.25,0],
        "orientation": [0,0,0,1]
    }
}

right_row = {
    "name": "right",
    "dynamic": False,
    "collision_geometry": {
        "type": "box",
        "dims": [0.5,21,0.2],
        "material": "red"
    },
    "config": {
        "position": [10.25,0,0],
        "orientation": [0,0,0,1]
    }
}

left_row = {
    "name": "left",
    "dynamic": False,
    "collision_geometry": {
        "type": "box",
        "dims": [0.5,21,0.2],
        "material": "red"
    },
    "config": {
        "position": [-10.25,0,0],
        "orientation": [0,0,0,1]
    }
}

yaml_to_dump["environment"]["geometries"].append(upper_row)
yaml_to_dump["environment"]["geometries"].append(lower_row)
yaml_to_dump["environment"]["geometries"].append(right_row)
yaml_to_dump["environment"]["geometries"].append(left_row)

thickness = 1.0
width = 2.0
big_width = 3.0

num_passages = 2

landmarks = []

# Generate num_passages equispaced vertical passages
x_coords = np.arange(-10,10.5,((20)/(num_passages+1)))[1:-1]
dist = x_coords[1] - x_coords[0]
for i in range(num_passages):
    x_coord = x_coords[i]
    
    # Generate the y_coordinate of the narrow passage.
    y_coord = np.random.uniform(-2, 2)
    
    up_box_len = 10 - big_width - y_coord - width/2. 
    down_box_len = y_coord - big_width - (-10) - width/2.
    up_box_center = 10 - big_width - up_box_len/2.
    down_box_center = -10 + big_width + down_box_len/2.

    # Get two landmarks. One on either side of the passage.
    l1 = [x_coord - dist/2., y_coord]
    l2 = [x_coord + dist/2., y_coord]
    landmarks.append(l1)
    landmarks.append(l2)

    # Get two landmarks. Top and bottom of the column.
    l3 = [x_coord, up_box_center + up_box_len/2. + big_width/2.]
    l4 = [x_coord, down_box_center - down_box_len/2. - big_width/2.]
    landmarks.append(l3)
    landmarks.append(l4)

    up_box = {
        "name": "vertical_{}_a".format(i),
        "collision_geometry": {
            "type": "box",
            "dims": [thickness, up_box_len, 0.2],
            "material": "red"
        },
        "config": {
            "position": list([float(x_coord),up_box_center,0.0]),
            "orientation": [0,0,0,1]
        }
    }

    down_box = {
        "name": "vertical_{}_b".format(i),
        "collision_geometry": {
            "type": "box",
            "dims": [thickness, down_box_len, 0.2],
            "material": "red"
        },
        "config": {
            "position": list([float(x_coord),down_box_center,0.0]),
            "orientation": [0,0,0,1]
        }
    }

    yaml_to_dump["environment"]["landmarks"] = []
    
    
    yaml_to_dump["environment"]["geometries"].append(up_box)
    yaml_to_dump["environment"]["geometries"].append(down_box)

landmarks = np.array(landmarks)
landmarks = np.hstack([landmarks,np.zeros((landmarks.shape[0],3))])
for landmark in landmarks:
    yaml_to_dump["environment"]["landmarks"].append(
        [float(l) for l in landmark]
    )

np.savetxt(os.environ["DIRTMP_PATH"]+"out/landmarks.txt", landmarks, delimiter=",")

plt.figure(figsize=(8,8))
obstacles = yaml_to_dump["environment"]["geometries"]
for obstacle in obstacles:
    box_center = obstacle["config"]["position"][:2]
    box_dims = obstacle["collision_geometry"]["dims"][:2]
    rect = Rectangle((box_center[0]-box_dims[0]/2.0,box_center[1]-box_dims[1]/2.0),box_dims[0],box_dims[1],
    linewidth=1,edgecolor='r',facecolor='r')
    plt.gca().add_patch(rect)
plt.scatter(landmarks[:,0],landmarks[:,1],color='blue')

# grid_x = np.linspace(-10,10,11)
# grid_y = np.linspace(-10,10,11)
# for i in range(len(grid_x)):
#     for j in range(len(grid_y)):
#         plt.scatter(grid_x[i], grid_y[j], c="black")

plt.xlim(-11,11)
plt.ylim(-11,11)
plt.show()

with open(os.environ["DIRTMP_PATH"]+"resources/input_files/environments/landmark.yaml", "w") as yaml_file:
    yaml.dump(yaml_to_dump, yaml_file, default_flow_style=None)