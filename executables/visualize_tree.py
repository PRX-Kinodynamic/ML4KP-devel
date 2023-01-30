import numpy as np
import matplotlib.pyplot as plt
import os
import yaml

from matplotlib.patches import Circle, Rectangle

fname = os.environ['DIRTMP_PATH'] + "out/"
env_fname_prefix = os.environ['DIRTMP_PATH'] + "resources/input_files/environments/"
env_fname = "landmark.yaml"
goal_state = np.array([8.,0.])
goal_radius = 0.5

plt.figure(figsize=(8, 8))
plt.xlim(-10,10)
plt.ylim(-10,10)

with open(env_fname_prefix + env_fname) as f:
    try:
        data = yaml.safe_load(f)
    except yaml.YAMLError as exc:
        print(exc)
    
obstacles = data["environment"]["geometries"]
for obstacle in obstacles:
    if obstacle["collision_geometry"]["type"] == "box":
        box_center = obstacle["config"]["position"]
        box_dims   = obstacle["collision_geometry"]["dims"]
        rect = Rectangle((box_center[0]-box_dims[0]*0.5, box_center[1]-box_dims[1]*0.5), box_dims[0], box_dims[1], color='red')
        plt.gca().add_patch(rect)
    else:
        print("Only box obstacles are supported")
        exit()

circle = Circle(goal_state, goal_radius, color='green')
plt.gca().add_patch(circle)

for f in os.listdir(fname):
    if f.endswith(".txt"):
        data = np.loadtxt(fname+f,delimiter=',')
        if f.startswith("tree"):
            plt.plot(data[:,0], data[:,1], color='black',linewidth=0.5)
        if f.startswith("solution") and data.shape[0] > 1:
            plt.plot(data[:,0], data[:,1], color='red', linewidth=2)

plt.show()