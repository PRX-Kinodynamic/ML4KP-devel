import numpy as np 
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import os 
import yaml

out_dir = os.environ["DIRTMP_PATH"] + "out/robust/"
goal_state = [8., 0.]
goal_radius = 1.0
environment_file = os.environ["DIRTMP_PATH"]+"resources/input_files/environments/empty.yaml"

with open(environment_file, 'r') as stream:
    try:
        env_params = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)
obstacles = env_params["environment"]["geometries"]

plt.figure(figsize=(8, 8))
plt.xlim(-11,11)
plt.ylim(-11,11)

for obstacle in obstacles:
    box_center = obstacle["config"]["position"][:2]
    box_dims = obstacle["collision_geometry"]["dims"][:2]
    rect = Rectangle((box_center[0]-box_dims[0]/2.0,box_center[1]-box_dims[1]/2.0),box_dims[0],box_dims[1],
    linewidth=1,edgecolor='r',facecolor='r')
    plt.gca().add_patch(rect)

# Plot goal
plt.gca().add_patch(Circle(goal_state, goal_radius, color='g', alpha=0.75))

end_points = []

for fname in os.listdir(out_dir):
    if fname.endswith(".txt") and fname.startswith("traj"):
        data = np.loadtxt(out_dir + fname,delimiter=',')
        plt.plot(data[:,0], data[:,1], color='black')
        end_points.append(data[-1,:])
        plt.scatter(data[-1,0], data[-1,1], color='red',marker='x')

end_points = np.array(end_points)
# plt.scatter(np.mean(end_points[:,0]), np.mean(end_points[:,1]), color='blue',marker='o')

'''
hull = np.loadtxt(out_dir + "hull.txt",delimiter=',')
plt.plot(hull[:,0], hull[:,1], color='black')
# Plot a line from last point to first point
plt.plot([hull[-1,0],hull[0,0]], [hull[-1,1],hull[0,1]], color='black')
'''

plt.show()