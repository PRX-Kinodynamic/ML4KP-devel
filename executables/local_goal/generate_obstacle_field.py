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
yaml_to_dump["environment"]["dynamic_geometries"] = []

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

# num_boxes = np.random.randint(20,50)
num_boxes = 40
print(num_boxes)
for i in range(0,num_boxes):
    row = {}
    row["name"] = "box_"+str(i)
    row["collision_geometry"] = {}
    row["collision_geometry"]["type"] = "box"
    row["collision_geometry"]["material"] = "red"
    row["collision_geometry"]["dims"] = [np.random.uniform(0.5,1.0),np.random.uniform(0.5,1.0),0.2]
    row["config"] = {}
    row["config"]["position"] = [np.random.uniform(-10.0,10.0),np.random.uniform(-10.0,10.0),0.1]
    row["config"]["orientation"] = [0,0,0,1]
    yaml_to_dump["environment"]["geometries"].append(row)

plt.figure(figsize=(8,8))
obstacles = yaml_to_dump["environment"]["geometries"]
for obstacle in obstacles:
    box_center = obstacle["config"]["position"][:2]
    box_dims = obstacle["collision_geometry"]["dims"][:2]
    rect = Rectangle((box_center[0]-box_dims[0]/2.0,box_center[1]-box_dims[1]/2.0),box_dims[0],box_dims[1],
    linewidth=1,edgecolor='r',facecolor='r')
    plt.gca().add_patch(rect)
plt.xlim(-11,11)
plt.ylim(-11,11)
plt.show()

with open(os.environ["DIRTMP_PATH"]+"resources/input_files/environments/maze.yaml",'w') as f:
    yaml.safe_dump(yaml_to_dump, f, default_flow_style=None)


